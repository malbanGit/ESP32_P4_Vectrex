// license:BSD-3-Clause
// copyright-holders:Brad Oliver, Eric Smith, Juergen Buchmueller
// POKEY chip emulator 4.9 — adapted from libvgm/ValleyBell for ESP32-P4 vsim.
// Stripped of libvgm framework; poly tables promoted to shared statics;
// voltab removed (LEGACY_LINEAR only). Interface unchanged from original vsim.

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "../defines.h"
#include "memory.h"
#include "pokey.h"
#include "game.h"

/* ---- type aliases matching libvgm conventions ---- */
typedef uint8_t  UINT8;
typedef uint16_t UINT16;
typedef uint32_t UINT32;
typedef int16_t  INT16;
typedef int32_t  INT32;
#define INLINE static inline

/* ---- POKEY constants ---- */
#define FREQ_17_EXACT  1789790u   /* 1.79 MHz Atari clock */
#define POKEY_CHANNELS 4
#define POKEY_DEFAULT_GAIN (32767/11/4)

#define CHAN1 0
#define CHAN2 1
#define CHAN3 2
#define CHAN4 3

/* AUDCx bits */
#define NOTPOLY5    0x80
#define POLY4       0x40
#define PURE        0x20
#define VOLUME_ONLY 0x10
#define VOLUME_MASK 0x0f

/* AUDCTL bits */
#define POLY9       0x80
#define CH1_HICLK   0x40
#define CH3_HICLK   0x20
#define CH12_JOINED 0x10
#define CH34_JOINED 0x08
#define CH1_FILTER  0x04
#define CH2_FILTER  0x02
#define CLK_15KHZ   0x01

/* IRQ bits */
#define IRQ_BREAK   0x80
#define IRQ_KEYBD   0x40
#define IRQ_SERIN   0x20
#define IRQ_SEROR   0x10
#define IRQ_SEROC   0x08
#define IRQ_TIMR4   0x04
#define IRQ_TIMR2   0x02
#define IRQ_TIMR1   0x01

/* SKSTAT bits */
#define SK_FRAME    0x80
#define SK_KBERR    0x40
#define SK_OVERRUN  0x20
#define SK_SERIN    0x10
#define SK_SHIFT    0x08
#define SK_KEYBD    0x04
#define SK_BUSY     0x02

/* SKCTL bits */
#define SK_BREAK    0x80
#define SK_OCLK     0x60
#define SK_ICLK     0x30
#define SK_ASYNC    0x10
#define SK_TWOTONE  0x08
#define SK_PADDLE   0x04
#define SK_RESET    0x03
#define SK_KEYSCAN  0x02
#define SK_DEBOUNCE 0x01

/* read register addresses */
#define ALLPOT_C 0x08
#define KBCODE_C 0x09
#define RANDOM_C 0x0A
#define SERIN_C  0x0D
#define IRQST_C  0x0E
#define SKSTAT_C 0x0F

/* write register addresses */
#define AUDF1_C  0x00
#define AUDC1_C  0x01
#define AUDF2_C  0x02
#define AUDC2_C  0x03
#define AUDF3_C  0x04
#define AUDC3_C  0x05
#define AUDF4_C  0x06
#define AUDC4_C  0x07
#define AUDCTL_C 0x08
#define STIMER_C 0x09
#define SKREST_C 0x0A
#define POTGO_C  0x0B
#define SEROUT_C 0x0D
#define IRQEN_C  0x0E
#define SKCTL_C  0x0F

#define DIV_64  28
#define DIV_15  114
#define CLK_1   0
#define CLK_28  1
#define CLK_114 2

/* ---- shared poly tables (same values for every POKEY instance) ---- */
static UINT32 s_poly4 [0x0f];
static UINT32 s_poly5 [0x1f];
static UINT32 s_poly9 [0x1ff];
HUGE_DATA_LOCATION UINT32 s_poly17[0x1ffff];

/* Packed bit array of (s_poly17[i] & 1) stored in DRAM — avoids PSRAM hit
 * in the audio hot path. 0x1ffff bits = 4096 uint32_t words = 16 KB. */
static DRAM_ATTR uint32_t s_poly17_bits[(0x1ffff / 32) + 1];

static bool s_poly_inited = false;

/* ---- per-channel state ---- */
typedef struct pokey_device pokey_device;

typedef struct {
    pokey_device *m_parent;
    UINT8  m_INTMask;
    UINT8  m_AUDF;
    UINT8  m_AUDC;
    INT32  m_borrow_cnt;
    INT32  m_counter;
    UINT8  m_output;
    UINT8  m_filter_sample;
} pokey_channel;

/* ---- per-chip state (voltab removed; poly tables are globals) ---- */
struct pokey_device {
    UINT8  m_muted[POKEY_CHANNELS];
    pokey_channel m_channel[POKEY_CHANNELS];

    UINT32 m_out_raw;
    UINT8  m_old_raw_inval;
    double m_out_filter;

    INT32  m_clock_cnt[3];
    UINT32 m_p4, m_p5, m_p9, m_p17;

    UINT8  m_POTx[8];
    UINT8  m_AUDCTL;
    UINT8  m_ALLPOT;
    UINT8  m_KBCODE;
    UINT8  m_SERIN;
    UINT8  m_SEROUT;
    UINT8  m_IRQST;
    UINT8  m_IRQEN;
    UINT8  m_SKSTAT;
    UINT8  m_SKCTL;

    UINT8  m_pot_counter;
    UINT8  m_kbd_cnt;
    UINT8  m_kbd_latch;
    UINT8  m_kbd_state;

    UINT16 m_serin_shift;
    UINT16 m_serout_shift;
    UINT8  m_ser_iclk;
    UINT8  m_ser_oclk;
    UINT8  m_serout_full;
    UINT8  m_sod_twotone;

    double m_clock_period;
    int    m_icount;
};

static DRAM_ATTR pokey_device s_dev[MAX_POKEY];
static int s_pokey_count = 1;  /* active chips — set by pokey_set_count() */

/* ---- forward declarations ---- */
static void pokey_dev_write(pokey_device *d, UINT8 offset, UINT8 data);
static void pokey_potgo(pokey_device *d);
static IRAM_ATTR void pokey_step_one_clock(pokey_device *d);
static void pokey_step_pot(pokey_device *d);
static IRAM_ATTR void pokey_step_keyboard(pokey_device *d);
INLINE IRAM_ATTR void pokey_process_channel(pokey_device *d, int ch);

/* ---- channel helpers ---- */
INLINE IRAM_ATTR void pokey_channel_sample(pokey_channel *c)
{
    c->m_filter_sample = c->m_output;
}

INLINE IRAM_ATTR void pokey_channel_reset_channel(pokey_channel *c)
{
    c->m_counter   = c->m_AUDF ^ 0xff;
    c->m_borrow_cnt = 0;
}

INLINE IRAM_ATTR void pokey_channel_inc_chan(pokey_channel *c, pokey_device *host, int cycles)
{
    c->m_counter = (c->m_counter + 1) & 0xff;
    if (c->m_counter == 0 && c->m_borrow_cnt == 0)
        c->m_borrow_cnt = cycles;
}

INLINE IRAM_ATTR int pokey_channel_check_borrow(pokey_channel *c)
{
    if (c->m_borrow_cnt > 0) {
        c->m_borrow_cnt--;
        return (c->m_borrow_cnt == 0);
    }
    return 0;
}

/* ---- poly table init ---- */
static void poly_init_4_5(UINT32 *poly, int size)
{
    int mask   = (1 << size) - 1;
    int xorbit = size - 1;
    UINT32 lfsr = 0;
    int i;
    for (i = 0; i < mask; i++) {
        lfsr  = (lfsr << 1) | (~((lfsr >> 2) ^ (lfsr >> xorbit)) & 1);
        *poly = lfsr & mask;
        poly++;
    }
}

static void poly_init_9_17(UINT32 *poly, int size)
{
    int mask   = (1 << size) - 1;
    UINT32 lfsr = mask;
    int i;
    if (size == 17) {
        for (i = 0; i < mask; i++) {
            const UINT32 in8 = ((lfsr >> 8) & 1) ^ ((lfsr >> 13) & 1);
            const UINT32 in  = (lfsr & 1);
            lfsr = lfsr >> 1;
            lfsr = (lfsr & 0xff7f) | (in8 << 7);
            lfsr = (in << 16) | lfsr;
            *poly++ = lfsr;
        }
    } else {
        for (i = 0; i < mask; i++) {
            const UINT32 in = ((lfsr >> 0) & 1) ^ ((lfsr >> 5) & 1);
            lfsr = lfsr >> 1;
            lfsr = (in << 8) | lfsr;
            *poly++ = lfsr;
        }
    }
}

/* ---- device reset ---- */
static void pokey_device_reset(pokey_device *d)
{
    int i;
    memset(d, 0, sizeof(*d));

    for (i = 0; i < POKEY_CHANNELS; i++) {
        d->m_channel[i].m_parent        = d;
        d->m_channel[i].m_INTMask       = 0;
        d->m_channel[i].m_AUDF          = 0;
        d->m_channel[i].m_AUDC          = 0xb0;
        d->m_channel[i].m_borrow_cnt    = 0;
        d->m_channel[i].m_counter       = 0;
        d->m_channel[i].m_output        = 0;
        d->m_channel[i].m_filter_sample = 0;
    }
    d->m_channel[CHAN1].m_INTMask = IRQ_TIMR1;
    d->m_channel[CHAN2].m_INTMask = IRQ_TIMR2;
    d->m_channel[CHAN4].m_INTMask = IRQ_TIMR4;

    d->m_KBCODE       = 0x09;
    d->m_SKCTL        = SK_RESET;
    d->m_IRQST        = IRQ_SEROC;
    d->m_old_raw_inval = 1;
    d->m_ser_iclk     = 1;
    d->m_serin_shift  = 1;
    d->m_serout_shift = 1;
    d->m_clock_period = 1.0 / FREQ_17_EXACT;

    pokey_potgo(d);
}

/* ---- set number of active POKEY chips ---- */
void pokey_set_count(int n)
{
    if (n < 1) n = 1;
    if (n > MAX_POKEY) n = MAX_POKEY;
    s_pokey_count = n;
}

/* ---- public init ---- */
void pokey_init(void)
{
    int i;
    if (!s_poly_inited) {
        poly_init_4_5(s_poly4, 4);
        poly_init_4_5(s_poly5, 5);
        poly_init_9_17(s_poly9,  9);
        poly_init_9_17(s_poly17, 17);

        /* Build packed DRAM bit-array from s_poly17 (LSB only) */
        memset(s_poly17_bits, 0, sizeof(s_poly17_bits));
        for (i = 0; i < 0x1ffff; i++) {
            if (s_poly17[i] & 1)
                s_poly17_bits[i >> 5] |= (1u << (i & 31));
        }

        s_poly_inited = true;
    }
    for (i = 0; i < MAX_POKEY; i++)
        pokey_device_reset(&s_dev[i]);
}

/* ---- potentiometer ---- */
static void pokey_potgo(pokey_device *d)
{
    int pot;
    d->m_ALLPOT = 0x00;
    d->m_pot_counter = 0;
    for (pot = 0; pot < 8; pot++)
        d->m_POTx[pot] = 228;
}

static void pokey_step_pot(pokey_device *d)
{
    uint8_t upd = 0;
    int pot;
    d->m_pot_counter++;
    for (pot = 0; pot < 8; pot++) {
        if ((d->m_POTx[pot] < d->m_pot_counter) || (d->m_pot_counter == 228))
            upd |= (1 << pot);
    }
    if (upd)
        d->m_ALLPOT |= upd;
}

/* ---- keyboard stub (IRAM_ATTR: called from IRAM hot path) ---- */
static IRAM_ATTR void pokey_step_keyboard(pokey_device *d)
{
    if (++d->m_kbd_cnt > 63)
        d->m_kbd_cnt = 0;
    /* no keyboard input in vsim — intentionally a no-op */
}

/* ---- process channel output ---- */
INLINE IRAM_ATTR void pokey_process_channel(pokey_device *d, int ch)
{
    if ((d->m_channel[ch].m_AUDC & NOTPOLY5) || (s_poly5[d->m_p5] & 1)) {
        if (d->m_channel[ch].m_AUDC & PURE)
            d->m_channel[ch].m_output ^= 1;
        else if (d->m_channel[ch].m_AUDC & POLY4)
            d->m_channel[ch].m_output = (s_poly4[d->m_p4] & 1);
        else if (d->m_AUDCTL & POLY9)
            d->m_channel[ch].m_output = (s_poly9[d->m_p9] & 1);
        else
            /* Use packed DRAM bit array — avoids PSRAM access */
            d->m_channel[ch].m_output =
                (s_poly17_bits[d->m_p17 >> 5] >> (d->m_p17 & 31)) & 1;
        d->m_old_raw_inval = 1;
    }
}

/* ---- step one chip clock ---- */
static IRAM_ATTR void pokey_step_one_clock(pokey_device *d)
{
    UINT8 toggle_iclk = 0;
    UINT8 toggle_oclk = 0;

    if (d->m_SKCTL & SK_RESET) {
        int clk28_trig = 0, clk114_trig = 0;
        int base_clock;
        UINT8 async_reset;

        if (++d->m_p4  == 0x0000f) d->m_p4  = 0;
        if (++d->m_p5  == 0x0001f) d->m_p5  = 0;
        if (++d->m_p9  == 0x001ff) d->m_p9  = 0;
        if (++d->m_p17 == 0x1ffff) d->m_p17 = 0;

        if (++d->m_clock_cnt[CLK_28] >= DIV_64) {
            d->m_clock_cnt[CLK_28] = 0;
            clk28_trig = 1;
        }
        if (++d->m_clock_cnt[CLK_114] >= DIV_15) {
            d->m_clock_cnt[CLK_114] = 0;
            clk114_trig = 1;
        }

        /* CLK_1 always fires — CH1_HICLK / CH3_HICLK path */
        if (d->m_AUDCTL & CH1_HICLK) {
            if (d->m_AUDCTL & CH12_JOINED)
                pokey_channel_inc_chan(&d->m_channel[CHAN1], d, 7);
            else
                pokey_channel_inc_chan(&d->m_channel[CHAN1], d, 4);
        }

        base_clock = (d->m_AUDCTL & CLK_15KHZ) ? CLK_114 : CLK_28;
        int base_trig = (base_clock == CLK_114) ? clk114_trig : clk28_trig;

        if (!(d->m_AUDCTL & CH1_HICLK) && base_trig)
            pokey_channel_inc_chan(&d->m_channel[CHAN1], d, 1);

        async_reset = (d->m_SKCTL & SK_ASYNC) && !(d->m_SKSTAT & (SK_BUSY | SK_SERIN));
        if (async_reset)
            d->m_ser_iclk = 1;

        if ((d->m_AUDCTL & CH3_HICLK) && !async_reset) {
            if (d->m_AUDCTL & CH34_JOINED)
                pokey_channel_inc_chan(&d->m_channel[CHAN3], d, 7);
            else
                pokey_channel_inc_chan(&d->m_channel[CHAN3], d, 4);
        }
        if (!(d->m_AUDCTL & CH3_HICLK) && base_trig && !async_reset)
            pokey_channel_inc_chan(&d->m_channel[CHAN3], d, 1);

        if (base_trig) {
            if (!(d->m_AUDCTL & CH12_JOINED)) {
                if (d->m_channel[CHAN2].m_counter == 0xff && (d->m_SKCTL & SK_OCLK) == 0x60)
                    toggle_oclk = 1;
                pokey_channel_inc_chan(&d->m_channel[CHAN2], d, 1);
            }
            if (!(d->m_AUDCTL & CH34_JOINED) && !async_reset) {
                if (d->m_channel[CHAN4].m_counter == 0xff) {
                    if ((d->m_SKCTL & SK_ICLK) != 0) toggle_iclk = 1;
                    if ((d->m_SKCTL & SK_OCLK) && (d->m_SKCTL & SK_OCLK) != 0x60)
                        toggle_oclk = 1;
                }
                pokey_channel_inc_chan(&d->m_channel[CHAN4], d, 1);
            }
        }

        if ((clk114_trig || (d->m_SKCTL & SK_PADDLE)) && (d->m_pot_counter < 228))
            pokey_step_pot(d);
        if (clk114_trig && (d->m_SKCTL & SK_KEYSCAN))
            pokey_step_keyboard(d);
    }

    if (pokey_channel_check_borrow(&d->m_channel[CHAN3])) {
        if (d->m_AUDCTL & CH34_JOINED) {
            if (d->m_channel[CHAN4].m_counter == 0xff) {
                if ((d->m_SKCTL & SK_ICLK) != 0) toggle_iclk = 1;
                if ((d->m_SKCTL & SK_OCLK) != 0 && (d->m_SKCTL & SK_OCLK) != 0x60)
                    toggle_oclk = 1;
            }
            pokey_channel_inc_chan(&d->m_channel[CHAN4], d, 1);
        } else {
            pokey_channel_reset_channel(&d->m_channel[CHAN3]);
        }
        pokey_process_channel(d, CHAN3);
        if (d->m_AUDCTL & CH1_FILTER)
            pokey_channel_sample(&d->m_channel[CHAN1]);
        else
            d->m_channel[CHAN1].m_filter_sample = 1;
        d->m_old_raw_inval = 1;
    }

    if (pokey_channel_check_borrow(&d->m_channel[CHAN4])) {
        if (d->m_AUDCTL & CH34_JOINED)
            pokey_channel_reset_channel(&d->m_channel[CHAN3]);
        pokey_channel_reset_channel(&d->m_channel[CHAN4]);
        pokey_process_channel(d, CHAN4);
        if (d->m_AUDCTL & CH2_FILTER)
            pokey_channel_sample(&d->m_channel[CHAN2]);
        else
            d->m_channel[CHAN2].m_filter_sample = 1;
        d->m_old_raw_inval = 1;
    }

    if ((d->m_SKCTL & SK_TWOTONE) && (d->m_channel[CHAN2].m_borrow_cnt == 1)) {
        pokey_channel_reset_channel(&d->m_channel[CHAN1]);
        d->m_old_raw_inval = 1;
        d->m_sod_twotone = !d->m_sod_twotone;
    }

    if (pokey_channel_check_borrow(&d->m_channel[CHAN1])) {
        if (d->m_AUDCTL & CH12_JOINED) {
            if (d->m_channel[CHAN2].m_counter == 0xff && (d->m_SKCTL & SK_OCLK) == 0x60)
                toggle_oclk = 1;
            pokey_channel_inc_chan(&d->m_channel[CHAN2], d, 1);
        } else {
            pokey_channel_reset_channel(&d->m_channel[CHAN1]);
            if ((d->m_SKCTL & SK_TWOTONE) &&
                ((d->m_IRQST & IRQ_SEROC) ? !(d->m_SKCTL & SK_BREAK) : d->m_serout_shift & 1))
            {
                pokey_channel_reset_channel(&d->m_channel[CHAN2]);
                d->m_old_raw_inval = 1;
                d->m_sod_twotone = !d->m_sod_twotone;
            }
        }
        pokey_process_channel(d, CHAN1);
    }

    if (pokey_channel_check_borrow(&d->m_channel[CHAN2])) {
        if (d->m_AUDCTL & CH12_JOINED)
            pokey_channel_reset_channel(&d->m_channel[CHAN1]);
        pokey_channel_reset_channel(&d->m_channel[CHAN2]);
        pokey_process_channel(d, CHAN2);
    }

    if (d->m_old_raw_inval) {
        UINT32 sum = 0;
        int ch;
        for (ch = 0; ch < 4; ch++) {
            if (d->m_muted[ch]) continue;
            sum |= (((d->m_channel[ch].m_output ^ d->m_channel[ch].m_filter_sample) ||
                     (d->m_channel[ch].m_AUDC & VOLUME_ONLY)) ?
                    ((d->m_channel[ch].m_AUDC & VOLUME_MASK) << (ch * 4)) : 0);
        }
        d->m_old_raw_inval = 0;
        d->m_out_raw = sum;
    }

    if (toggle_iclk)
        d->m_ser_iclk = !d->m_ser_iclk;

    if (toggle_oclk && (d->m_serout_full || !(d->m_IRQST & IRQ_SEROC)))
        d->m_ser_oclk = !d->m_ser_oclk;
}

/* ---- skip idle clocks when no state changes can occur ---- */
/*
 * When neither HICLK is active (channels 1/3 not clocked every cycle),
 * nothing changes between CLK_28 / CLK_114 edge events except borrow
 * counters counting down.  We can mathematically advance poly counters,
 * clock dividers, and borrow counters without calling step_one_clock.
 *
 * Returns the number of clocks skipped (may be 0 if skip is not safe).
 */
static IRAM_ATTR int pokey_skip_idle_clocks(pokey_device *d, int remaining)
{
    int skip, i;
    int to_clk28, to_clk114, to_clk;

    /* Skip-ahead is only safe when neither channel uses CLK_1 directly */
    if (d->m_AUDCTL & (CH1_HICLK | CH3_HICLK))
        return 0;

    /* Clocks until the next CLK_28 edge (it fires when counter reaches DIV_64) */
    to_clk28  = DIV_64  - d->m_clock_cnt[CLK_28];
    to_clk114 = DIV_15  - d->m_clock_cnt[CLK_114];
    to_clk = (to_clk28 < to_clk114) ? to_clk28 : to_clk114;

    /* Stop one clock before the edge so step_one_clock fires it */
    skip = to_clk - 1;

    /* Also stop before any active borrow countdown reaches zero */
    for (i = 0; i < POKEY_CHANNELS; i++) {
        if (d->m_channel[i].m_borrow_cnt > 0) {
            int to_borrow = d->m_channel[i].m_borrow_cnt - 1;
            if (to_borrow < skip) skip = to_borrow;
        }
    }

    /* Never skip more clocks than we have left */
    if (skip > remaining - 1) skip = remaining - 1;
    if (skip <= 0) return 0;

    /* Advance poly counters by 'skip' steps (modular) */
//    d->m_p4  += skip; if (d->m_p4  >= 0x0000fu) d->m_p4  -= 0x0000fu;
//    d->m_p5  += skip; if (d->m_p5  >= 0x0001fu) d->m_p5  -= 0x0001fu;
//    d->m_p9  += skip; if (d->m_p9  >= 0x001ffu) d->m_p9  -= 0x001ffu;
//    d->m_p17 += skip; if (d->m_p17 >= 0x1ffffu) d->m_p17 -= 0x1ffffu;

    d->m_p4  = (d->m_p4+skip) & 0x0000fu;
    d->m_p5  = (d->m_p5+skip) & 0x0001fu;
    d->m_p9  = (d->m_p9+skip) & 0x001ffu;
    d->m_p17  = (d->m_p17+skip) & 0x1ffffu;


    /* Advance clock dividers — no wrap needed: skip < to_clk <= DIV_64/DIV_15 */
    d->m_clock_cnt[CLK_28]  += skip;
    d->m_clock_cnt[CLK_114] += skip;

    /* Decrement active borrow counters */
    for (i = 0; i < POKEY_CHANNELS; i++) {
        if (d->m_channel[i].m_borrow_cnt > 0)
            d->m_channel[i].m_borrow_cnt -= skip;
    }

    return skip;
}

/* ---- generate one sample from a device (LEGACY_LINEAR) ---- */
static IRAM_ATTR INT32 pokey_get_sample(pokey_device *d)
{
    INT32 out;
    int i;

    while (d->m_icount > 0) {
        if (d->m_SKCTL & SK_RESET) {
            int skipped = pokey_skip_idle_clocks(d, d->m_icount);
            d->m_icount -= skipped;
            if (d->m_icount <= 0) break;
        }
        pokey_step_one_clock(d);
        d->m_icount--;
    }

    out = 0;
    for (i = 0; i < 4; i++)
        out += ((d->m_out_raw >> (4 * i)) & 0x0f);
    out *= POKEY_DEFAULT_GAIN;
    if (out > 0x7fff) out = 0x7fff;
    return out;
}

/* ---- register read (libvgm logic) ---- */
static UINT8 pokey_dev_read(pokey_device *d, UINT8 offset)
{
    int data, pot;
    switch (offset & 15) {
    case 0x00: case 0x01: case 0x02: case 0x03:
    case 0x04: case 0x05: case 0x06: case 0x07:
        pot = offset & 7;
        data = (d->m_ALLPOT & (1 << pot)) ? d->m_POTx[pot] : d->m_pot_counter;
        break;
    case ALLPOT_C:
        data = ((d->m_SKCTL & SK_RESET) == 0) ? d->m_ALLPOT : (d->m_ALLPOT ^ 0xff);
        break;
    case KBCODE_C:
        data = d->m_KBCODE;
        break;
    case RANDOM_C:
        if (d->m_AUDCTL & POLY9)
            data = s_poly9[d->m_p9] & 0xff;
        else
            data = (s_poly17[d->m_p17] >> 8) & 0xff;  /* full PSRAM value needed here */
        break;
    case SERIN_C:
        data = d->m_SERIN;
        break;
    case IRQST_C:
        data = d->m_IRQST ^ 0xff;
        break;
    case SKSTAT_C:
        data = d->m_SKSTAT ^ 0xff;
        break;
    default:
        data = 0xff;
        break;
    }
    return (UINT8)data;
}

/* ---- register write (libvgm logic) ---- */
static void pokey_dev_write(pokey_device *d, UINT8 offset, UINT8 data)
{
    switch (offset & 15) {
    case AUDF1_C: d->m_channel[CHAN1].m_AUDF = data; break;
    case AUDC1_C: d->m_channel[CHAN1].m_AUDC = data; d->m_old_raw_inval = 1; break;
    case AUDF2_C: d->m_channel[CHAN2].m_AUDF = data; break;
    case AUDC2_C: d->m_channel[CHAN2].m_AUDC = data; d->m_old_raw_inval = 1; break;
    case AUDF3_C: d->m_channel[CHAN3].m_AUDF = data; break;
    case AUDC3_C: d->m_channel[CHAN3].m_AUDC = data; d->m_old_raw_inval = 1; break;
    case AUDF4_C: d->m_channel[CHAN4].m_AUDF = data; break;
    case AUDC4_C: d->m_channel[CHAN4].m_AUDC = data; d->m_old_raw_inval = 1; break;
    case AUDCTL_C:
        if (data == d->m_AUDCTL) return;
        d->m_AUDCTL = data;
        d->m_old_raw_inval = 1;
        break;
    case STIMER_C: {
        int i;
        for (i = 0; i < POKEY_CHANNELS; i++) {
            pokey_channel_reset_channel(&d->m_channel[i]);
            d->m_channel[i].m_output = 0;
            d->m_channel[i].m_filter_sample = (i < 2 ? 1 : 0);
        }
        d->m_old_raw_inval = 1;
        break;
    }
    case SKREST_C:
        d->m_SKSTAT &= ~(SK_FRAME | SK_OVERRUN | SK_KBERR);
        break;
    case POTGO_C:
        if (d->m_SKCTL & SK_RESET) pokey_potgo(d);
        break;
    case SEROUT_C:
        d->m_SEROUT = data;
        d->m_serout_full = 1;
        break;
    case IRQEN_C:
        if (d->m_IRQST & ~data)
            d->m_IRQST &= (IRQ_SEROC | data);
        d->m_IRQEN = data;
        break;
    case SKCTL_C:
        if (data == d->m_SKCTL) return;
        d->m_SKCTL = data;
        if (!(data & SK_RESET)) {
            pokey_dev_write(d, IRQEN_C,  0);
            pokey_dev_write(d, SKREST_C, 0);
            d->m_p9 = 0; d->m_p17 = 0;
            d->m_p4 = 0; d->m_p5  = 0;
            d->m_clock_cnt[0] = 0;
            d->m_clock_cnt[1] = 0;
            d->m_clock_cnt[2] = 0;
            d->m_serin_shift  = 1;
            d->m_serout_shift = 1;
            d->m_serout_full  = 0;
            d->m_SKSTAT &= ~SK_BUSY;
        }
        if (!(data & SK_KEYSCAN)) {
            d->m_SKSTAT &= ~SK_KEYBD;
            d->m_kbd_cnt   = 0;
            d->m_kbd_state = 0;
        }
        if ((data & SK_ICLK) == 0) d->m_ser_iclk = 1;
        if ((data & SK_OCLK) == 0) d->m_ser_oclk = 0;
        if ((data & SK_ASYNC) && !(d->m_SKSTAT & SK_BUSY)) {
            pokey_channel_reset_channel(&d->m_channel[CHAN3]);
            pokey_channel_reset_channel(&d->m_channel[CHAN4]);
        }
        d->m_old_raw_inval = 1;
        break;
    }
}

/* ---- Tempest-specific state (kept from original vsim pokey.c) ---- */
static int s_oldSpinner = 0;
static int s_spinnerSum = 0;

/* ---- public vsim interface ---- */

byte pokey_read(int pokeynum, int reg, int PC, unsigned long cyc)
{
    if (pokeynum < 0 || pokeynum >= MAX_POKEY) return 0xff;


    if ((((UINT8)reg) & 15) == RANDOM_C)
    {
        int data = 0xff;
        if (s_dev[pokeynum].m_AUDCTL & POLY9)
        {
            s_dev[pokeynum].m_p9  = (s_dev[pokeynum].m_p9+5) & 0x001ffu;
            data = s_poly9[s_dev[pokeynum].m_p9] & 0xff;
            return data;
        }
        else
        {
            s_dev[pokeynum].m_p17  = (s_dev[pokeynum].m_p17+7) & 0x1ffffu;
            data = (s_poly17[s_dev[pokeynum].m_p17] >> 8) & 0xff;  /* full PSRAM value needed here */
            return data;
        }
    }

    /* Tempest overrides — kept from original vsim pokey.c */
    if (game == TEMPEST) 
    {
        // if (reg == (RANDOM_C & 0x0f) && pokeynum == 1)  return 0xff;
        
        if (reg == (ALLPOT_C & 0x0f) && pokeynum == 0) 
        {
            signed char spin = (signed char)joystick.x;
            if (spin > 0)       spin = (spin - 30) / 30;
            else if (spin < 0)  spin = (spin + 30) / 30;
            s_spinnerSum += spin;
            if (s_spinnerSum >  7) { s_oldSpinner -= spin; s_spinnerSum += 7; }
            if (s_spinnerSum < -7) { s_oldSpinner -= spin; s_spinnerSum -= 7; }
            return (byte)(s_oldSpinner & 0x0f);
        }
        if (reg == (ALLPOT_C & 0x0f) && pokeynum == 1) {
            int ret = 0;
            if (switches[0].fire)   ret |= 0x04;
            if (switches[0].fire)   ret |= 0x08;
            if (switches[0].thrust) ret |= 0x10;
            if (start1)             ret |= 0x20;
            if (start2)             ret |= 0x40;
            return (byte)ret;
        }
    }

    return pokey_dev_read(&s_dev[pokeynum], (UINT8)reg);
}

void pokey_write(int pokeynum, int reg, byte val, int PC, unsigned long cyc)
{
    if (pokeynum < 0 || pokeynum >= MAX_POKEY) return;
    pokey_dev_write(&s_dev[pokeynum], (UINT8)reg, (UINT8)val);
}

/* ---- audio sample generation ---- */
/* Writes stereo interleaved 16-bit PCM: L, R, L, R, ...
 * buf must hold n_samples * 2 int16_t values (same as AY_CHANNEL=2 layout).
 * clocks_per_sample = FREQ_17_EXACT / sample_rate (e.g. 40 for 44100 Hz).
 * POKEY is mono; both channels receive the same mixed value.
 */
IRAM_ATTR void pokey_generate_samples(int16_t *buf, int n_samples, int clocks_per_sample)
{
    int i, p;
    for (i = 0; i < n_samples; i++) {
        INT32 mix = 0;
        for (p = 0; p < s_pokey_count; p++) {
            s_dev[p].m_icount = clocks_per_sample;
            mix += pokey_get_sample(&s_dev[p]);
        }
        mix /= s_pokey_count;
        if (mix >  32767) mix =  32767;
        if (mix < -32768) mix = -32768;
        buf[i * 2]     = (int16_t)mix;  /* L */
        buf[i * 2 + 1] = (int16_t)mix;  /* R */
    }
}
// length in byte!
IRAM_ATTR void callbackPokey(void *userdata, uint8_t *stream, int length)
{
	(void) userdata;

	/* hack to prevent us from hanging when starting filtered outputs */
	if (!s_poly_inited)
	{
		memset(stream, 0, length * sizeof(*stream));
		return;
	}
  // the set reg as a batch - is buggy
  // some noises do not fade if  regs are set with the batch!
  //	ayemu_set_regs(&ay, snd_regs);
  pokey_generate_samples((int16_t *)stream, 882, 1789790 / 44100);
}
