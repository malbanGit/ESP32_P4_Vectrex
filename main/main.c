// karl on LCD demo 1 46 !!!!
// spike is now very VERY slow???? -> Spike IS so slow!!!
#include "defines.h"

// from vecx.c
extern int SCREEN_WIDTH;
extern int SCREEN_HEIGHT;


/*

#define JOYSTICK_VERTICAL 1
#define JOYSTICK_HORIZONTAL 1
#define KEY_ ...
 
true / false or value
int getInput(JOYSTICK_VERTICAL);

int playWAV();

setSoundCallback(*functionPointer)


gives one FPS!

		 && alg_curr_x >= 0 && alg_curr_x < ALG_MAX_X && alg_curr_y >= 0 && alg_curr_y < ALG_MAX_Y

 






1) 6909 to slow in new version
Karl Quappe 47 statt 50 fps in emulation

SLOW1: Vectorblade demo last level reached 46 emu speed -> to slow! -> without overlay
SLOW1: Karl Quappe on LCD is still about 5 FPS slower then on HDMI out... FUCK WHY???

YUV is about 2 FPS faster (emulator)
YUV

Optimizing steps:
a) 
        via_sstep0(); 
        removed, gain 3 FPS! - this would be needed for imager and lightpen emulation!

b) 
        in via_sstep1 ();
            if (via_srb >= 8u) ,return
        inserted - minimalistic gain

c) 
        Following was not done, since for some reason or another, long uncalled for lines appeared in output
        Even only setting scl_factorx to uint32_t does this, despite the fact that the scale factor is NEVER negative
        Don't know why this does what it does.
        Theoretically a unsigned division is faster then a signed division!

        If you're doing division, unsigned variants are cheaper than signed on RISC-V 
        (signed division needs extra sign-handling instructions) — 
        if these are always non-negative in practice, uint32_t instead of long/int could
        be a real, measurable win, unlike the long-vs-int question itself.
        int32_t/uint32_t
        scl_factorx
        offy
        long x0, long y0, long x1, long y1

        IRAM_ATTR static einline  void alg_addline (long x0, long y0, long x1, long y1, unsigned char color)
        {
            if (intensityDrift>100000)
            {
                double degradePercent = (180000000.0-((double)intensityDrift))/180000000.0; // two minutes
                if (degradePercent<0) degradePercent = 0;
                color = (int)(((double)color)*degradePercent);
            }

            emu_draw_line(offx + x0 / scl_factorx, offy +y0 / scl_factory, offx + x1 / scl_factorx, offy + y1 / scl_factory, color); // For ESP32
            return;
        }




Try out ESP IDF 6 with 400 Mhz -> not working with my chip!!!


Highscore saving
joystick
settings save ti ini
sound volume

TODO: Calibration Ala Tuts


Pessimism to free DRAM
    - moved CART_ROM to PSRAM
    - moved STACK to PSRAM
    - moved all "line data" to PSRAM
-> seems not to make a great difference (perhaps 1 FPS?) - but we have some dram to spare - whenever we might need it!

NOTE:
EMU FPS
returns the Hz of display of the Vectrex.
If a game runs too slow - I mean if one round of display takes longer then 30000 cycles,
the the EMU FPS drops also. 
This is like the real vectrex - and no slowdown of emulation!



2. Two-layer PPA compositing (eliminate overlay from CPU entirely)
ESP32-P4 has a Pixel Processing Accelerator (PPA) hardware block that does alpha blending. The idea:

Emulator renders Vectrex lines on a pure-black fb — no overlay involvement at all
PPA blends the black fb + overlay → final display fb in hardware, asynchronously
Core 1 emulator never touches overlay PSRAM at all during rendering
Draw/undraw becomes trivial: write colored pixels or black. No palette lookup, no alpha, no bbox guards. The CPU completely exits the compositing business.

3. Frame-diff line caching (near-zero cost for static screens)
Many Vectrex frames are identical or nearly identical to the previous frame. Cache the line list (x0, y0, x1, y1, brightness, thickness) from last frame. If a line is unchanged, skip its draw+undraw entirely. For title screens, menus, or slow-moving games this could eliminate 80%+ of render work.



To start the current version connect USB POW/UART to computer (directly)
Connect USB (mid /down) to keyboard - the keyboard can be used to control the vectrex
Start
Should connect to COM 4, flash and play.


  BUG - no!
  FCYCLES_INIT = 50000
  should be 30000 for one round -> Nope the 50000 is fallback for "not" autosyncable... 
  watch it "flicker" when spike speaks!

*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_clk_tree.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_ldo_regulator.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_lt8912b.h"

#include "esp_attr.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "esp_task_wdt.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>


#include "board.h"
#include "mire.h"
#include "hdmi.h"
#include "lvds.h"
#include "lodepng.h"

#include "vecx\vecx.h"
#include "draw_line_yuv422.h"



char * audio_buf;
size_t audio_bufsize;


char cartName[MAX_ROM_NAME];
HUGE_DATA_LOCATION unsigned char cartData[MAX_CART_SIZE];
long cartSize = 0;

DRAM_ATTR uint32_t g_beam_r2_q16;
DRAM_ATTR int g_line_Rb = (((LINE_WIDTH >> 1) + LINE_GLOW_WIDTH) > 0) ? ((LINE_WIDTH >> 1) + LINE_GLOW_WIDTH - 1) : 0;
DRAM_ATTR int g_beam_r = LINE_WIDTH >> 1;
DRAM_ATTR uint32_t g_gs;


void changeGlobalLineValues(int w, int g)
{
    g_line_width = w;
    g_line_glow = g;

    // Precomputed bounding-box radius shared by all draw/undraw functions and ASM.
    // Must be updated if g_line_width or g_line_glow ever change at runtime.     
    g_line_Rb = (((g_line_width >> 1) + g_line_glow) > 0) ? ((g_line_width >> 1) + g_line_glow - 1) : 0;
    g_beam_r = g_line_width >> 1;
    g_beam_r2_q16  = cap_d2_q16(g_beam_r, 0);
    g_gs = gs_for_glow(g_line_glow);
}


esp_lcd_dsi_bus_handle_t dsi_bus = NULL;

i2c_master_bus_handle_t i2c_bus = NULL;
i2c_master_bus_config_t i2c_bus_cfg = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = BOARD_I2C_PORT,
    .sda_io_num = BOARD_I2C_SDA,
    .scl_io_num = BOARD_I2C_SCL,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};

// Shared flag — core 1 writes, core 0 reads
static volatile int s_toggle_display_mode = 0; // sentinel "no switch"
static volatile int s_toggle_overlay_mode = 0; // sentinel "no switch"


// ----------------------------------------------------
// Global line settings
// ----------------------------------------------------
DRAM_ATTR int  g_line_width = LINE_WIDTH;      // >= 1
DRAM_ATTR int  g_line_glow  = LINE_GLOW_WIDTH;
DRAM_ATTR int  brightnessAdjust = BRIGHTNESS_ADJUST;
DRAM_ATTR int  brightnessLCD = DEFAULT_LCD_BRIGHTNESS;
DRAM_ATTR int LCD_H_RES = 1280; // overwritten when screen is initialized
DRAM_ATTR int LCD_V_RES = 720;
//hdmi 1280x720
//vdsl 480x800



static const char *TAG = "main";

DRAM_ATTR uint8_t *s_overlay    = NULL;    // this is a palette based overlay. 127 palette entries. if bit 7 is set, then alpha is present
DRAM_ATTR uint8_t *s_overlay_bg = NULL;   /* precomputed RGB888 overlay-over-black */


// Logischer Frame-Slot: Liste von Linien + Hash + Status
// ----------------------------------------------------
// Types
// ----------------------------------------------------
// Frame-Slot-Status für die Logik-Frames (Linien)
typedef enum {
    FRAME_FREE = 0,      // vom Emulator frei nutzbar
    FRAME_BUILDING,      // Emulator schreibt Linien
    FRAME_READY,         // fertig und wartet auf Renderer
    FRAME_RENDERING      // Renderer liest/zeichnet
} frame_state_t;
/*
typedef struct {
    int x0;
    int y0;
    int x1;
    int y1;
    uint8_t brightness;
} vectrex_line_t;
*/
typedef struct {
    uint16_t x0;
    uint16_t y0;
    uint16_t x1;
    uint16_t y1;
    uint8_t brightness;
    uint8_t  _pad[7];
} vectrex_line_t; // power of 2 length

typedef struct {
    vectrex_line_t      lines[MAX_LINE_BUFFER]; // these are the "new" lines - that will be drawn by the renderer
    int                 line_count;
    uint32_t            hash;
    volatile frame_state_t state;
} frame_slot_t;

typedef struct { int bx0, by0, bx1, by1; } line_bbox_t;

// Diese beschreiben, was aktuell in jedem Hardware-Framebuffer gezeichnet ist
DRAM_ATTR  static vectrex_line_t s_fb_lines[NUM_FB][MAX_LINE_BUFFER]; // these are the last drawn lines by the renderer - used to undraw!
DRAM_ATTR static int            s_fb_line_count[NUM_FB] = {0};
DRAM_ATTR static uint8_t     s_diff_old_matched[MAX_LINE_BUFFER];
DRAM_ATTR static uint8_t     s_diff_new_matched[MAX_LINE_BUFFER];
DRAM_ATTR static uint8_t     s_diff_damaged[MAX_LINE_BUFFER];
DRAM_ATTR  static line_bbox_t s_diff_dirty_bboxes[MAX_LINE_BUFFER];

// ----------------------------------------------------
// Emulator frame storage (independent of framebuffers)
// ----------------------------------------------------

// Drei logische Frames als Ringpuffer
DRAM_ATTR static frame_slot_t s_frames[NUM_FRAME_SLOTS]; // 3 frames
DRAM_ATTR static int          s_build_frame_index = 0;   // Slot, in den der Emulator gerade schreibt


/* ── Palettised overlay (draw optimisation) ───────────────────────────────── *
 * s_overlay_pal: 1 byte/pixel covering the active image region only (PSRAM).
 *   bit 7 = 1 → drawable pixel; palette index in bits 6..0
 *   bit 7 = 0 → skip (transparent or opaque after alphaAdjust)
 * s_overlay_palette: BGR table in DRAM — stays in L1 cache permanently.
 * Built once at load time with the then-current alphaAdjust.              */
uint8_t       *s_overlay_pal = NULL;
DRAM_ATTR uint8_t  s_overlay_palette[128][3];
DRAM_ATTR uint8_t  s_overlay_palette_yuv[128][3];    /* {Y, U, V} full-range for YUV draw */
DRAM_ATTR uint8_t  s_overlay_palette_yuv_ea[128][3]; /* {Y, U, V} ea-scaled for YUV undraw */

DRAM_ATTR int      s_overlay_pal_n   = 0;
DRAM_ATTR uint8_t  s_overlay_alpha_val = GLOBAL_OVERLAY_ALPHA;  /* representative raw alpha */
DRAM_ATTR int      s_ov_off_x = 0;             /* active region x offset   */
DRAM_ATTR int      s_ov_off_y = 0;             /* active region y offset   */
DRAM_ATTR int      s_ov_w     = 0;             /* active region width      */
DRAM_ATTR int      s_ov_h     = 0;             /* active region height     */

// ----------------------------------------------------
// Per-framebuffer line storage for undraw (Hardware-FBs)
// ----------------------------------------------------

/* Task stacks pinned to internal SRAM so function calls never touch PSRAM. */
#define EMU_STACK_SIZE  8192
#define REND_STACK_SIZE 8192
static /*DRAM_ATTR*/ StackType_t  s_emu_stack[EMU_STACK_SIZE];
static DRAM_ATTR StaticTask_t s_emu_tcb;
static /*DRAM_ATTR*/ StackType_t  s_rend_stack[REND_STACK_SIZE];
static DRAM_ATTR StaticTask_t s_rend_tcb;

// ----------------------------------------------------
// Framebuffer / display (2 Hardware-FBs)
// ----------------------------------------------------
DRAM_ATTR static uint8_t            *s_framebuffers[NUM_FB] = { NULL, NULL };
DRAM_ATTR static uint8_t            *s_fb_front             = NULL;  // aktuell angezeigt
DRAM_ATTR static uint8_t            *s_fb_back              = NULL;  // Zeichenziel
DRAM_ATTR static int                s_front_fb_index       = 0;     // Index in s_framebuffers[]
DRAM_ATTR static int                s_back_fb_index        = 1;
DRAM_ATTR static esp_lcd_panel_handle_t panel_handle = NULL;

DRAM_ATTR static esp_lcd_panel_lt8912b_io_t lt_io = {0};


// Diagnostic: 1 = output the P4 DPI's built-in vertical color bars instead of our
// frame buffer. Rules out a black/buggy frame buffer and tests the exact
// P4 DPI -> DSI -> bridge -> LVDS data path with known-good pixels. 0 = normal.
// #define VIDEO_DPI_TEST_PATTERN  1

DRAM_ATTR static esp_lcd_panel_handle_t s_dpi_panel = NULL;
DRAM_ATTR static SemaphoreHandle_t      s_vsync_sem = NULL;
DRAM_ATTR int mode = VIDEO_OUT_SELECTED;          // default at boot
DRAM_ATTR int overlayEnabled = ENABLE_OVERLAYS;          // default at boot

// helper "c" files - that are included, instead of own objects
#include "audio.i"
#include "sdcard.i"
#include "usb.i"
#include "file.i"

// helper functions for optimizing line draws
// these are used to check whether lines intersect
static inline line_bbox_t line_compute_bbox(const vectrex_line_t *l)
{
    int Rb = g_line_Rb;
    line_bbox_t b;
    b.bx0 = (l->x0 < l->x1 ? l->x0 : l->x1) - Rb;
    b.bx1 = (l->x0 > l->x1 ? l->x0 : l->x1) + Rb;
    b.by0 = (l->y0 < l->y1 ? l->y0 : l->y1) - Rb;
    b.by1 = (l->y0 > l->y1 ? l->y0 : l->y1) + Rb;
    return b;
}
static inline int bboxes_overlap(const line_bbox_t *a, const line_bbox_t *b)
{
    return a->bx0 <= b->bx1 && a->bx1 >= b->bx0 &&
           a->by0 <= b->by1 && a->by1 >= b->by0;
}

// ----------------------------------------------------
// VSYNC callback
// renderer task is synchronized with VSYNC
// ----------------------------------------------------
IRAM_ATTR static bool lcd_on_refresh_done_cb(esp_lcd_panel_handle_t panel,
                                             esp_lcd_dpi_panel_event_data_t *edata,
                                             void *user_ctx)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(s_vsync_sem, &xHigherPriorityTaskWoken);
    return (xHigherPriorityTaskWoken == pdTRUE);
}

// a line draw - ONE
// used to draw and undraw (undraw brightness = 0)
// at the moment a few different line-draws are possible
// if RGS stays stable I will drop the YUV

IRAM_ATTR static inline void drawLine_raw_color(int x0, int y0, int x1, int y1, uint8_t colorPaletteEntry)
{
    if (colorPaletteEntry == 0) 
    {
        undraw_line_yuv422_color_c(s_fb_back, LCD_H_RES, LCD_V_RES, x0, y0, x1, y1, colorPaletteEntry);
    }
    else
    {
        draw_line_yuv422_color_c(s_fb_back, LCD_H_RES, LCD_V_RES, x0, y0, x1, y1, colorPaletteEntry);
    }
}
IRAM_ATTR static inline void drawLine_raw(int x0, int y0, int x1, int y1, uint8_t brightness)
{
    if (brightness == 0) 
    {
        if (s_overlay == NULL)
        {
            undraw_line_yuv422_brightness_c(s_fb_back, LCD_H_RES, LCD_V_RES, x0, y0, x1, y1, brightness);
        }
        else        
        {
            undraw_line_yuv422_overlay_c(s_fb_back, LCD_H_RES, LCD_V_RES, x0, y0, x1, y1, brightness, s_overlay);
        }
    }
    else
    {
        if (x0==x1 && y0==y1) 
        {
            int b = brightness*4+brightnessAdjust;
            if (b>0)
            {
                if (s_overlay == NULL)
                {
                    draw_line_yuv422_brightness_c(s_fb_back, LCD_H_RES, LCD_V_RES, x0, y0, x1, y1, b);
                }
                else
                {
                    draw_line_yuv422_overlay_c(s_fb_back, LCD_H_RES, LCD_V_RES, x0, y0, x1, y1, b, s_overlay);
                }
            }
            return;
        }
        int b = brightness*4/3+brightnessAdjust;
        if (b>0)
        {
            if (s_overlay == NULL)
            {
                draw_line_yuv422_brightness_c(s_fb_back, LCD_H_RES, LCD_V_RES, x0, y0, x1, y1, b);
            }
            else
            {
                draw_line_yuv422_overlay_c(s_fb_back, LCD_H_RES, LCD_V_RES, x0, y0, x1, y1, b, s_overlay);
            }
        }
    }
}

// ----------------------------------------------------
// LCD init
// ----------------------------------------------------

// Laurents init, taken from hdmi demo "hdmi_lt8912b_main.c"

// VCV_NOE asserted = SN74AVC4T245 translator on (I2S + HPD). Harmless for video,
// kept consistent and needed later for I2S/HPD.
static void board_assert_vcv_noe(void)
{
    gpio_set_level(BOARD_PIN_VCV_NOE, 0);   // pre-load latch before enabling driver
    gpio_config_t io = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << BOARD_PIN_VCV_NOE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_LOGI(TAG, "VCV_NOE asserted (level translator on)");
}

// VDD_MIPI_DPHY must be 2.5 V; on the P4 it comes from internal LDO_VO3.
static void board_enable_dsi_phy_power(void)
{
    esp_ldo_channel_handle_t phy_ldo = NULL;
    esp_ldo_channel_config_t cfg = {
        .chan_id = 3,            // LDO_VO3 -> VDD_MIPI_DPHY
        .voltage_mv = 2500,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&cfg, &phy_ldo));
    ESP_LOGI(TAG, "MIPI DSI D-PHY power on (2.5 V)");
}

static const char *mode_name(int m) { return m == VIDEO_OUT_LVDS ? "LVDS" : "HDMI"; }

// HDMI Hot-Plug Detect: LT8912B main bank reg 0xC1 bit7 = sink connected (5 V on HPD).
// Same read the driver's internal get_hpd does; we poll it directly off the main io.
static bool hdmi_hpd_present(const esp_lcd_panel_lt8912b_io_t *io)
{
    uint8_t data = 0;
    if (esp_lcd_panel_io_rx_param(io->main, 0xc1, &data, 1) != ESP_OK) return false;
    return (data & 0x80) != 0;
}
// Füllt einen YUV422-Framebuffer komplett mit Y=0 (schwarz) und U=V=128 (neutral/grau).
// Nutzt 32-Bit-Writes für maximale Schreibgeschwindigkeit (ein Store deckt ein
// komplettes Pixelpaar ab, statt 4 Einzel-Byte-Writes).
static void init_yuv422_framebuffer(uint8_t *fb, int width, int height)
{
    // Pattern pro Pixelpaar: Y0=0x00, U=0x80, Y1=0x00, V=0x80
    // Als little-endian uint32_t (ESP32 ist little-endian):
    const uint32_t pattern = 0x80008000u;

    size_t num_pairs = ((size_t)width * height) / 2;
    uint32_t *p32 = (uint32_t *)fb;

    for (size_t i = 0; i < num_pairs; ++i) {
        p32[i] = pattern;
    }
}
// Bring up the selected output, allocate the frame buffer, draw the test card.
static esp_err_t video_start(int mode, bool yuv422,
                             const esp_lcd_panel_lt8912b_io_t *io, esp_lcd_dsi_bus_handle_t dsi,
                             esp_lcd_panel_handle_t *panel, int *w, int *h)
{
    static const char *TAG = "video";
    esp_err_t err = (mode == VIDEO_OUT_LVDS)
                    ? lvds_start(io, dsi, yuv422, panel, w, h)
                    : hdmi_start(io, dsi, yuv422, panel, w, h);
    if (err != ESP_OK) return err;
    if (mode == VIDEO_OUT_HDMI) lvds_backlight(false,1023);   // LVDS panel unused -> backlight off

    esp_err_t err2 = esp_lcd_dpi_panel_get_frame_buffer(
        *panel,
        2,
        (void **)&s_framebuffers[0],
        (void **)&s_framebuffers[1]
    );
    if (err2 != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get framebuffer: %s", esp_err_to_name(err2));
        return err2;
    }
    if (s_framebuffers[0] == NULL)
    {
        ESP_LOGE(TAG, "Failed to get framebuffer 0");
        return err2;
    }
    if (s_framebuffers[1] == NULL)
    {
        ESP_LOGE(TAG, "Failed to get framebuffer 1");
        return err2;
    }

    if (err2 == ESP_OK &&
        s_framebuffers[0] != NULL &&
        s_framebuffers[1] != NULL)
    {
        s_fb_front       = s_framebuffers[0];
        s_fb_back        = s_framebuffers[1];
        s_front_fb_index = 0;
        s_back_fb_index  = 1;

        if (yuv422)
        {
            init_yuv422_framebuffer(s_fb_front,*w,*h);
            init_yuv422_framebuffer(s_fb_back,*w,*h);
        }
        else
        {
            memset(s_fb_front, 0x00, (*w) * (*h) * sizeof(uint8_t)*VIDEO_FB_BPP);
            memset(s_fb_back,  0x00, (*w) * (*h) * sizeof(uint8_t)*VIDEO_FB_BPP);
        }


        ESP_LOGI(TAG, "HW double buffer: FB0=%p FB1=%p", s_framebuffers[0], s_framebuffers[1]);
    }
    else
    {
        ESP_LOGI(TAG, "ERROR Allocating double buffer!\n");
    }

    ESP_LOGI(TAG, "%s up: test card %dx%d (%s)", mode_name(mode), *w, *h, yuv422 ? "YUV422" : "RGB888");

#if LVDS_SETTLE_SWEEP
    if (mode == VIDEO_OUT_LVDS) lvds_sweep_settle(io);   // diagnostic: find the right settle
#endif

    return ESP_OK;
}

// Tear down the current output so the other one can be brought up cleanly.
static void video_stop(esp_lcd_panel_handle_t panel)
{
    // also frees the framebuffers
    if (panel) esp_lcd_panel_del(panel);   // also deletes the inner DPI panel
    s_fb_front       = NULL;
    s_fb_back        = NULL;

}

static void lcd_deinit(void)
{
    // 1. Unregister VSYNC callback first — prevents callbacks firing
    //    during teardown.
    if (s_dpi_panel) {
        esp_lcd_dpi_panel_event_callbacks_t cbs = {
            .on_refresh_done     = NULL,
            .on_color_trans_done = NULL,
        };
        esp_lcd_dpi_panel_register_event_callbacks(s_dpi_panel, &cbs, NULL);
    }

    // 2. Turn display off.
    if (s_dpi_panel) {
        esp_lcd_panel_disp_on_off(s_dpi_panel, false);
    }

    // 3. Delete the panel (DPI panel, created inside video_start).
    if (panel_handle) {
        esp_lcd_panel_del(panel_handle);
        panel_handle = NULL;
        s_dpi_panel  = NULL;
    }
    s_fb_front = NULL; s_fb_back = NULL;
    // 4. Delete the DSI bus (releases MIPI PHY + lane config).
    if (dsi_bus) {
        esp_lcd_del_dsi_bus(dsi_bus);
        dsi_bus = NULL;
    }

    // 5. Delete the three LT8912B I2C panel IOs.
    if (lt_io.main)    { esp_lcd_panel_io_del(lt_io.main);    lt_io.main    = NULL; }
    if (lt_io.cec_dsi) { esp_lcd_panel_io_del(lt_io.cec_dsi); lt_io.cec_dsi = NULL; }
    if (lt_io.avi)     { esp_lcd_panel_io_del(lt_io.avi);     lt_io.avi     = NULL; }

    // 6. Delete the VSYNC semaphore.
    if (s_vsync_sem) {
        vSemaphoreDelete(s_vsync_sem);
        s_vsync_sem = NULL;
    }

    ESP_LOGI(TAG, "LCD deinitialized");
}

static void lcd_init(void)
{
    ESP_LOGI(TAG, "Create VSYNC semaphore");
    s_vsync_sem = xSemaphoreCreateBinary();
    configASSERT(s_vsync_sem);

    // 2) The three LT8912B I2C register banks (main / cec-dsi / avi).
    esp_lcd_panel_io_i2c_config_t io_main_cfg = LT8912B_IO_CFG(LT8912B_I2C_HZ, LT8912B_IO_I2C_MAIN_ADDRESS);
    esp_lcd_panel_io_i2c_config_t io_cec_cfg  = LT8912B_IO_CFG(LT8912B_I2C_HZ, LT8912B_IO_I2C_CEC_ADDRESS);
    esp_lcd_panel_io_i2c_config_t io_avi_cfg  = LT8912B_IO_CFG(LT8912B_I2C_HZ, LT8912B_IO_I2C_AVI_ADDRESS);
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_main_cfg, &lt_io.main));
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_cec_cfg,  &lt_io.cec_dsi));
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_avi_cfg,  &lt_io.avi));

    // 3) MIPI-DSI bus (2 lanes, 1000 Mbps/lane).
/*
    #define LT8912B_PANEL_BUS_DSI_2CH_CONFIG_LCD()               \
    {                                                    \
        .bus_id = 0,                                     \
        .num_data_lanes = 2,                             \
        .phy_clk_src = 0,                                \
        .lane_bit_rate_mbps = 720,                      \
    }
    #define LT8912B_PANEL_BUS_DSI_2CH_CONFIG_HDMI()               \
    {                                                    \
        .bus_id = 0,                                     \
        .num_data_lanes = 2,                             \
        .phy_clk_src = 0,                                \
        .lane_bit_rate_mbps = 1200,                      \
    }
    if (mode == VIDEO_OUT_LVDS)
    {
        esp_lcd_dsi_bus_config_t dsi_bus_cfg = LT8912B_PANEL_BUS_DSI_2CH_CONFIG_LCD();
        ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&dsi_bus_cfg, &dsi_bus));
    }
    else
    {
        esp_lcd_dsi_bus_config_t dsi_bus_cfg = LT8912B_PANEL_BUS_DSI_2CH_CONFIG_HDMI();
        ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&dsi_bus_cfg, &dsi_bus));
    }
*/
    esp_lcd_dsi_bus_handle_t dsi_bus = NULL;
    esp_lcd_dsi_bus_config_t dsi_bus_cfg = LT8912B_PANEL_BUS_DSI_2CH_CONFIG();
    if (mode == VIDEO_OUT_LVDS)
        dsi_bus_cfg.lane_bit_rate_mbps = BOARD_DSI_LANE_MBPS;   // single source: board.h
    else
        dsi_bus_cfg.lane_bit_rate_mbps = 1200;   // single source: board.h

    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&dsi_bus_cfg, &dsi_bus));


    const bool yuv422 = VIDEO_FB_YUV422; // 
    panel_handle = NULL;
    int w,h;
    ESP_ERROR_CHECK(video_start(mode, yuv422, &lt_io, dsi_bus, &panel_handle, &w, &h));

    LCD_H_RES = w;
    LCD_V_RES = h;
    s_dpi_panel = panel_handle;
//    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_dpi_panel, true));

    // Register VSYNC callback
    esp_lcd_dpi_panel_event_callbacks_t cbs = {
        .on_refresh_done     = lcd_on_refresh_done_cb,
        .on_color_trans_done = NULL,
    };
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_register_event_callbacks(s_dpi_panel, &cbs, NULL));

    ESP_LOGI(TAG, "LCD initialized");
}

// ----------------------------------------------------
// Frames initialisieren (logische Frame-Slots)
// ----------------------------------------------------
static void frames_init(void)
{
    for (int i = 0; i < NUM_FRAME_SLOTS; ++i) {
        s_frames[i].line_count = 0;
        s_frames[i].hash       = 0;
        s_frames[i].state      = FRAME_FREE;
    }
    s_build_frame_index      = 0;
    s_frames[0].state        = FRAME_BUILDING;
}

// ----------------------------------------------------
// Emulator-side API
// ----------------------------------------------------
// when in color mode
// color from palette 127
// color = 0 -> undraw

// Emulator calls this for each line in the CURRENT emulated frame
IRAM_ATTR void mini_draw_line(int x0, int y0, int x1, int y1, uint8_t brightness)
{
    int idx = s_build_frame_index;
    frame_slot_t *fs = &s_frames[idx];

    if (fs->state != FRAME_BUILDING) {
        // Slot ist nicht im BUILDING-Zustand – dann nichts schreiben
        return;
    }

    if (fs->line_count < MAX_LINE_BUFFER) 
    {
        vectrex_line_t *l = &fs->lines[fs->line_count++];
        if (mode == VIDEO_OUT_HDMI)
        {
            l->x0 = x0;
            l->y0 = y0;
            l->x1 = x1;
            l->y1 = y1;
        }
        else
        {
            /* 90° CW rotation for portrait LCD: (x,y) → (y, -x) */
            l->x0 =  LCD_H_RES-y0;  l->y0 = x0;
            l->x1 =  LCD_H_RES-y1;  l->y1 = x1;
        }

        l->brightness = brightness;

        // Hash für diesen Frame laufend aktualisieren
    }
    else {
        // Buffer voll – zusätzliche Linien werden verworfen
        // Optional: Debug-Ausgabe
        // printf("Emulation Exceeded line Count!!! (%d)\n", fs->line_count);
    }
}

// Emulator calls this once when a frame is complete
// the lines collected from the emulator
// are kept in 3 buffers
// the state of the buffer determines which is drawn on screen
// and which is used to recieve "new" lines.
IRAM_ATTR void mini_end_frame(void)
{
    static uint64_t emu_last_time   = 0;
    static int      emu_frame_count = 0;

    // EMU FPS
    // note!
    // this FPS displays how many frame ends were recieved from the emulator in 1 second
    // a frame end emulation wise is NOT a fixed time!
    // a frame end is decided by the vectrex programmer, usually by a call to WaitRecal().
    // Per standard a WaitRecal should be called every 30000 vectrex cycles
    // if that is done - then the FPS shows 50 FPS
    //
    // if the vectrex itself uses more the 30000 cycles per frame - then the framerate drops
    // but this does not mean the emulation is to slow.
    // this only means the frame of the vectrex took longer then 1/50 of a second
    //
    // many games have rounds that are slower then 30000 cycles!
    // -> a frame drop here does NOT mean the emulation is not running in full speed!
    // it rather shows how "fast" or how "good" a vectrex game runs and keeps up with the desired 1/50 per round
    //
    // NOTE:
    // If the emulator runs in "DEFAULT_AUTO_SYNC"
    // then it tries to find a call to WaitRecal or for timer T2 to expire.
    // If both of these options fail - the emulator "automatically" sends a frame end after 50000 cycles
    // if that happends the screen (in the emulation) will flicker!
    // This is mainly due to the fact, that then each displayed framebuffer does not get a full "load" of lines
    // but rather the lines that have been finished since last call!
    // and each of these 50000 cycle frames - have different lines, different count of lines -> flicker.
    uint64_t now = esp_timer_get_time();
    emu_frame_count++;
    int idx = s_build_frame_index;
    frame_slot_t *cur = &s_frames[idx];
    if (now - emu_last_time >= 1000000) // has one second passed?
    {
//        printf("EMU FPS = %d (Lines: %i )\n", emu_frame_count, cur->line_count);
        printf("EMU FPS = %d\n", emu_frame_count);
        emu_frame_count = 0;
        emu_last_time   = now;
    }


    // Aktuellen Slot als READY markieren
    cur->state = FRAME_READY;

    // Nächsten Build-Slot wählen:
    // 1) Bevorzugt einen FREE-Slot
    // 2) Wenn kein FREE, dann einen READY-Slot (älteren Frame droppen)
    int next = -1;

    for (int i = 0; i < NUM_FRAME_SLOTS; ++i) {
        int cand = i;
        if (s_frames[cand].state == FRAME_FREE) {
            next = cand;
            break;
        }
    }

    if (next < 0) {
        for (int i = 0; i < NUM_FRAME_SLOTS; ++i) {
            int cand = i;
            if (s_frames[cand].state == FRAME_READY) {
                next = cand;
                break;
            }
        }
    }

    if (next < 0) {
        // Extrem unwahrscheinlich (alle RENDERING/BUILDING),
        // im Zweifel aktuellen Slot weiterverwenden und Frame droppen.
        next = idx;
    }

    frame_slot_t *fs_next = &s_frames[next];
    fs_next->line_count = 0;
    fs_next->hash       = 0;
    fs_next->state      = FRAME_BUILDING;
    s_build_frame_index = next;
}

// ----------------------------------------------------
// Undraw previous contents of a given framebuffer index
// ----------------------------------------------------
// only kept for future easier testing!
// now we do not use the "SIMPLE_UNDRAW" per default anymore - it is much slower!
IRAM_ATTR static inline void undraw_previous_fb(int fb_index)
{
    int count = s_fb_line_count[fb_index];
//    ESP_LOGI(TAG, "LINES TO ERASE: %i", count);

    for (int i = 0; i < count; ++i) {
        vectrex_line_t *l = &s_fb_lines[fb_index][i];
        // Erase the line (brightness 0)
        drawLine_raw(l->x0, l->y0, l->x1, l->y1, 0);
    }

    s_fb_line_count[fb_index] = 0;
}

// ----------------------------------------------------
// Tasks
// ----------------------------------------------------
void mini_taskloop(int cycles);
IRAM_ATTR static void application_task(void *arg)
{
    while (1) 
    {
        uint64_t start = esp_timer_get_time();

        mini_taskloop(30000);  // internal vecx loop; calls emu_draw_line/emu_end_frame

        // slow down if we are too fast
        // emulating 30000 vectrex cycles should take 1/50 of a second...
        // if MAX_EMU_FPS is 50 and 50 is reached, we run with 100% original speed
        uint64_t now = esp_timer_get_time();
        if (now - start <= 1000000/MAX_EMU_FPS) 
        {
            int delay = (1000000/MAX_EMU_FPS) - (now - start);
            esp_rom_delay_us(delay);  
        }
    }
}

IRAM_ATTR static inline uint32_t line_hash_key(const vectrex_line_t *l)
{
    uint32_t h = (uint32_t)l->x0 * 2654435761u;
    h ^= (uint32_t)l->y0 * 40503u;
    h ^= (uint32_t)l->x1 * 2246822519u;
    h ^= (uint32_t)l->y1 * 3266489917u;
    h ^= (uint32_t)l->brightness;
    return h;
}
// Hash table for O(n) line matching (open addressing, linear probe)
// Size must be power-of-2 and > MAX_LINE_BUFFER for good load factor
#define MATCH_HT_BITS 11
#define MATCH_HT_SIZE (1 << MATCH_HT_BITS)   // 2048
#define MATCH_HT_MASK (MATCH_HT_SIZE - 1)
DRAM_ATTR static int16_t s_match_ht[MATCH_HT_SIZE]; // -1 = empty, else new-line index

// Renderer: VSYNC-driven, uses last finished emulated frame.
DRAM_ATTR int redraw = 0;
IRAM_ATTR static void renderer_task(void *arg)
{
    // Wait once for first VSYNC
    xSemaphoreTake(s_vsync_sem, portMAX_DELAY);

    uint64_t fps_last_time = esp_timer_get_time();
    int fps_frame_count    = 0;

    for (;;)
    {
        if (s_toggle_display_mode)
        {
            void toggleVideoMode();
            toggleVideoMode();
            s_toggle_display_mode = 0;
            redraw = 3;
        }
        else if (s_toggle_overlay_mode)
        {
            void toggleOverlay();
            toggleOverlay();
            s_toggle_overlay_mode = 0;
            redraw = 3;
        }

        // DISPLAY FPS = number of actually changed frames drawn per second
        // if the drawing keeps up,
        // this is the frequency of the screen refresh - so if it shows 60 on a 60Hz display - everything is good!
        fps_frame_count++;
        uint64_t now = esp_timer_get_time();
        if (now - fps_last_time >= 1000000) {
            printf("DISPLAY FPS = %d\n", fps_frame_count);
            fps_frame_count = 0;
            fps_last_time   = now;
        }

        // Wait for next VSYNC
        xSemaphoreTake(s_vsync_sem, portMAX_DELAY);

        // Einen READY-Slot holen – idealerweise den "neueren".
        int frame_idx  = -1;
        int line_count = 0;


        // 3 "frames" hier stehen nur die Linien - das hat nichts mit den framebuffern zu tun!
        // Hier: letzter READY in der Schleife gewinnt (neuester)
        for (int i = 0; i < NUM_FRAME_SLOTS; ++i) {
            if (s_frames[i].state == FRAME_READY) {
                frame_idx  = i;
                line_count = s_frames[i].line_count;
            }
        }

        // Kein fertiger Frame verfügbar
        if (frame_idx < 0) {
            continue;
        }


        frame_slot_t *fs = &s_frames[frame_idx]; // frame_idx = 0-2
        fs->state = FRAME_RENDERING;

        // Use current back framebuffer index
        int fb_idx = s_back_fb_index;           // the current backbuffer - we can write to - it is not displayed!
//        uint64_t t0 = esp_timer_get_time();

// simple version
// undraw all known old lines
// draw all known new lines
// regardless whether they are the same or not
    if (redraw>0)
    {
        redraw--;
        undraw_previous_fb(fb_idx);             // undraw all from that framebuffer

        // Draw new frame lines into backbuffer and remember them
        // linecount from "frame" (collection of lines)
        for (int i = 0; i < line_count; ++i) {
            vectrex_line_t *src = &fs->lines[i];

            drawLine_raw(src->x0, src->y0, src->x1, src->y1, src->brightness);

            if (s_fb_line_count[fb_idx] < MAX_LINE_BUFFER) {
                s_fb_lines[fb_idx][s_fb_line_count[fb_idx]++] = *src;
            }
            else
            {
                printf("-----------------------\n");
                printf("Line Buffer Exceeded!!!\n");
                printf("-----------------------\n");
            }
        }
    }
    else
    {
        // printf("linecount: %i\n", line_count);
        // non simple version
        // check which lines are IDENTICAL to last draw
        // check if any undelete lines interesect
        // undraw all deleted and marked as "dirty lines"
        // draw all lines that are not on the screen already

        // most games have somewhere "steady" lines... this really saves a lot!
        // for cleansweep - this is a party! - A maze full of "fixed" lines!

        // --- Stable/dirty/redraw frame-diff ---
            unsigned int old_count = s_fb_line_count[fb_idx];
            vectrex_line_t *old_lines = s_fb_lines[fb_idx];

            // Step 1: match old lines to new lines (exact match)
            /*
            memset(s_diff_old_matched, 0, old_count);
            memset(s_diff_new_matched, 0, line_count);
            for (int i = 0; i < old_count; i++) {
                for (int j = 0; j < line_count; j++) {
                    if (!s_diff_new_matched[j] &&
                        old_lines[i].x0 == fs->lines[j].x0 &&
                        old_lines[i].y0 == fs->lines[j].y0 &&
                        old_lines[i].x1 == fs->lines[j].x1 &&
                        old_lines[i].y1 == fs->lines[j].y1 &&
                        old_lines[i].brightness == fs->lines[j].brightness) {
                        s_diff_old_matched[i] = 1;
                        s_diff_new_matched[j] = 1;
                        break;
                    }
                }
            }
            */
            // Step 1: match old lines to new lines — O(n) hash table
            memset(s_match_ht, 0xFF, sizeof(s_match_ht)); // -1 = empty
            for (int j = 0; j < line_count; j++) {
                uint32_t h = line_hash_key(&fs->lines[j]) & MATCH_HT_MASK;
                while (s_match_ht[h] != -1) h = (h + 1) & MATCH_HT_MASK;
                s_match_ht[h] = (int16_t)j;
            }
            memset(s_diff_old_matched, 0, old_count);
            memset(s_diff_new_matched, 0, line_count);
            for (int i = 0; i < old_count; i++) {
                const vectrex_line_t *ol = &old_lines[i];      /* i*20 computed once */
                uint32_t h = line_hash_key(ol) & MATCH_HT_MASK;
                while (s_match_ht[h] != -1) {
                    int j = s_match_ht[h];
                    const vectrex_line_t *nw = &fs->lines[j];  /* j*20 computed once per probe */
                    if (!s_diff_new_matched[j] &&
                        ol->x0 == nw->x0 &&
                        ol->y0 == nw->y0 &&
                        ol->x1 == nw->x1 &&
                        ol->y1 == nw->y1 &&
                        ol->brightness == nw->brightness) {
                        s_diff_old_matched[i] = 1;
                        s_diff_new_matched[j] = 1;
                        break;
                    }
                    h = (h + 1) & MATCH_HT_MASK;
                }
            }


            // Step 2: compute bboxes for dirty (unmatched) old lines
            int dirty_bbox_count = 0;
            for (int i = 0; i < old_count; i++) {
                if (!s_diff_old_matched[i])
                    s_diff_dirty_bboxes[dirty_bbox_count++] = line_compute_bbox(&old_lines[i]);
            }


            // Step 3: detect stable lines damaged by dirty bbox overlap
            memset(s_diff_damaged, 0, old_count);
            for (int i = 0; i < old_count; i++) {
                if (!s_diff_old_matched[i]) continue;
                line_bbox_t sb = line_compute_bbox(&old_lines[i]);
                for (int d = 0; d < dirty_bbox_count; d++) {
                    if (bboxes_overlap(&sb, &s_diff_dirty_bboxes[d])) {
                        s_diff_damaged[i] = 1;
                        break;
                    }
                }
            }

            // Step 3b: propagate damage through stable-stable overlaps
            // (undrawing a damaged line erases contributions of overlapping stable lines)
            int propagated = 1;
            while (propagated) {
                propagated = 0;
                for (int i = 0; i < old_count; i++) {
                    if (!s_diff_old_matched[i] || !s_diff_damaged[i]) continue;
                    line_bbox_t ab = line_compute_bbox(&old_lines[i]);
                    for (int k = 0; k < old_count; k++) {
                        if (!s_diff_old_matched[k] || s_diff_damaged[k]) continue;
                        line_bbox_t kb = line_compute_bbox(&old_lines[k]);
                        if (bboxes_overlap(&ab, &kb)) {
                            s_diff_damaged[k] = 1;
                            propagated = 1;
                        }
                    }
                }
            }
    /*
            // Step 4: undraw dirty old lines
            for (int i = 0; i < old_count; i++) {
                if (!s_diff_old_matched[i]) {
                    vectrex_line_t *l = &old_lines[i];
                    drawLine_raw(l->x0, l->y0, l->x1, l->y1, 0);
                }
            }

            // Step 5a: undraw ALL damaged stable lines
            for (int i = 0; i < old_count; i++) {
                if (s_diff_old_matched[i] && s_diff_damaged[i]) {
                    vectrex_line_t *l = &old_lines[i];
                    drawLine_raw(l->x0, l->y0, l->x1, l->y1, 0);
                }
            }
            */
            // Steps 4+5a: undraw dirty old lines AND damaged stable lines in one pass
            for (int i = 0; i < old_count; i++) {
                if (!s_diff_old_matched[i] || s_diff_damaged[i]) {
                    vectrex_line_t *l = &old_lines[i];
                    drawLine_raw(l->x0, l->y0, l->x1, l->y1, 0);
                }
            }


            // Step 5b: redraw ALL damaged stable lines
            for (int i = 0; i < old_count; i++) {
                if (s_diff_old_matched[i] && s_diff_damaged[i]) {
                    vectrex_line_t *l = &old_lines[i];
                    drawLine_raw(l->x0, l->y0, l->x1, l->y1, l->brightness);
                }
            }

            // Step 6: draw dirty new lines + rebuild s_fb_lines for next frame
            s_fb_line_count[fb_idx] = 0;
            for (int j = 0; j < line_count; j++) {
                vectrex_line_t *src = &fs->lines[j];
                if (!s_diff_new_matched[j])
                    drawLine_raw(src->x0, src->y0, src->x1, src->y1, src->brightness);
                if (s_fb_line_count[fb_idx] < MAX_LINE_BUFFER) {
                    s_fb_lines[fb_idx][s_fb_line_count[fb_idx]++] = *src;
                } else {
                    printf("-----------------------\n");
                    printf("Line Buffer Exceeded!!!\n");
                    printf("-----------------------\n");
                }
            }
        }


        // Swap front/back pointers and indices
        uint8_t *tmp_fb = s_fb_front;
        s_fb_front       = s_fb_back;
        s_fb_back        = tmp_fb;

        int tmp_idx      = s_front_fb_index;
        s_front_fb_index = s_back_fb_index;
        s_back_fb_index  = tmp_idx;
        
        // Present new front buffer
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(
            s_dpi_panel,
            0, 0, LCD_H_RES, LCD_V_RES,
            s_fb_front
        ));

        // Frame-Slot wieder freigeben
        fs->state = FRAME_FREE;
    }
}

// audio task is now running in core 1 with the emulation
// if it ran together with the renderer - interferences occured massively!
// audio task is set to high prio - otherwise the sound can stutter 
// when the emulator is running under full load!
// The audio "write" set the buffer to the "player" logic
// those functions wait till the complete buffer is played
// thus they "wait" for about 1/50 of a second, and then
// request via callback the next sound package from the emulation
//
// the audio callback fills the buffer for exactly 1/50 of a secons
// see audio setup
static void audio_music_task(void *arg)
{
    while (1)
    {
        if (!s_audio_cb) {
            /* Noch kein Codec oder kein Callback gesetzt: kurz warten */
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (audio_buf==NULL)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        s_audio_cb(NULL, (int16_t *) audio_buf, audio_bufsize);

        esp_err_t ret;
        if (mode == VIDEO_OUT_HDMI)
        {
            size_t written = 0;
            ret = i2s_channel_write(s_i2s_tx_chan,
                                    (void *)audio_buf,
                                    audio_bufsize,
                                    &written, portMAX_DELAY);
        }
        else
        {
            ret = esp_codec_dev_write(s_codec_dev, (void *)audio_buf, audio_bufsize);
        }

        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "audio_music_task: esp_codec_dev_write failed: 0x%x", ret);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

/* Write s_overlay (BGRA, 4 bytes/pixel) into s_fb_back (BGR, 3 bytes/pixel).
 * Destination is assumed to be black, so partial-alpha pixels scale the source
 * colour directly: out = src * effective_alpha / 255.
 *   alpha == 0   → skip (dest stays black)
 *   alpha == 255 → straight copy
 *   otherwise    → alpha clamped by alphaAdjust, then out = src * a / 255     */
void drawOverlay(uint8_t *dest)
{
    if (s_overlay == NULL) return;

    const uint8_t *src  = s_overlay;
    int total = LCD_H_RES * LCD_V_RES;

    for (int i = 0; i < total; i++, src += 4, dest += 3)
    {
        uint8_t a = src[3];
        if (a == 0) /* fully transparent — leave black */
        {
            dest[0] = 0;
            dest[1] = 0;
            dest[2] = 0;
        }
        else if (a == 255) {                  /* fully opaque — straight copy */
            dest[0] = src[0];
            dest[1] = src[1];
            dest[2] = src[2];
        } else {                         /* semi-transparent: scale over black */
            int ea = (int)s_overlay_alpha_val;
            if (ea <= 0)  continue;
            if (ea > 255) ea = 255;
            dest[0] = (uint8_t)((src[0] * ea) >> 8);
            dest[1] = (uint8_t)((src[1] * ea) >> 8);
            dest[2] = (uint8_t)((src[2] * ea) >> 8);
        }
    }
}
/* Write s_overlay_pal (palette, 1 byte/pixel) into dest.
 *
 * Palette index byte:
 *   bit 7 clear → fully opaque pixel, use palette colour directly
 *   bit 7 set   → semi-transparent, scale by s_overlay_alpha_val
 *
 * Two pixel-format paths selected at compile time via VIDEO_FB_YUV422:
 *
 *   RGB888 (VIDEO_FB_YUV422 == 0):
 *     dest is 3 bytes/pixel (B, G, R order).
 *     Palette stores BGR so values are written straight through.
 *
 *   YUV422 / YUYV (VIDEO_FB_YUV422 == 1):
 *     dest is 2 bytes/pixel — memory layout per pixel pair:
 *       [Y0][U][Y1][V]  (YUYV, little-endian)
 *     Y  is computed per pixel  (BT.601 full-range: Y = (77R+150G+29B)>>8)
 *     U/V are written once per even-column pixel and shared with the
 *     following odd-column pixel.  An overlay starting at an odd column
 *     gets correct luma on the first pixel but inherits the initialised
 *     U=V=128 (neutral gray) chroma for that one pixel — acceptable.
 */
void drawOverlayPal(uint8_t *dest)
{
    if (s_overlay_pal == NULL) return;

    for (int y = 0; y < s_ov_h; y++)
    {
        const uint8_t *row_pal = s_overlay_pal + (size_t)y * s_ov_w;
        int dst_y = s_ov_off_y + y;

#if VIDEO_FB_YUV422
        uint8_t *row_dst = dest + (size_t)dst_y * LCD_H_RES * 2;

        for (int x = 0; x < s_ov_w; x++)
        {
            int px = s_ov_off_x + x;
            uint8_t pidx = row_pal[x];
            const uint8_t *c = s_overlay_palette[pidx & 0x7F];
            int b, g, r;
            if (!(pidx & 0x80)) {
                b = c[0]; g = c[1]; r = c[2];
            } else {
                b = (c[0] * s_overlay_alpha_val) >> 8;
                g = (c[1] * s_overlay_alpha_val) >> 8;
                r = (c[2] * s_overlay_alpha_val) >> 8;
            }

            /* Y (BT.601 full-range: 77+150+29 = 256) */
            int yv = (77 * r + 150 * g + 29 * b) >> 8;
            if (yv > 255) yv = 255;
            row_dst[px * 2] = (uint8_t)yv;

            /* U and V written only at even columns; covers this pixel pair */
            if (!(px & 1)) {
                int uv = 128 + ((-43 * r -  85 * g + 128 * b) >> 8);
                int vv = 128 + ((128 * r - 107 * g -  21 * b) >> 8);
                if (uv < 0) uv = 0; else if (uv > 255) uv = 255;
                if (vv < 0) vv = 0; else if (vv > 255) vv = 255;
                row_dst[px * 2 + 1] = (uint8_t)uv;
                row_dst[px * 2 + 3] = (uint8_t)vv;
            }
        }
#else
        uint8_t *row_dst = dest + ((size_t)dst_y * LCD_H_RES + s_ov_off_x) * 3;

        for (int x = 0; x < s_ov_w; x++, row_dst += 3)
        {
            uint8_t pidx = row_pal[x];
            const uint8_t *c = s_overlay_palette[pidx & 0x7F];
            if (!(pidx & 0x80)) {
                row_dst[0] = c[0];
                row_dst[1] = c[1];
                row_dst[2] = c[2];
            } else {
                row_dst[0] = (uint8_t)((c[0] * s_overlay_alpha_val) >> 8);
                row_dst[1] = (uint8_t)((c[1] * s_overlay_alpha_val) >> 8);
                row_dst[2] = (uint8_t)((c[2] * s_overlay_alpha_val) >> 8);
            }
        }
#endif
    }
}

// ---------------------------------------------------------------------------
// overlay_load_png_bgra  — decode PNG into a BGRA (4 bytes/pixel) buffer.
// lodepng gives RGBA; we store as BGRA to match the framebuffer's BGR order.
// fb must be fb_w * fb_h * 4 bytes.
// ---------------------------------------------------------------------------
esp_err_t overlay_load_png_bgra(const char *path, uint8_t *fb, int fb_w, int fb_h)
{
    unsigned char *raw   = NULL;
    unsigned       width = 0, height = 0;

    unsigned err = lodepng_decode32_file(&raw, &width, &height, path);
    if (err) {
        ESP_LOGE(TAG, "PNG decode failed (%u): %s", err, lodepng_error_text(err));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "PNG %s  %u×%u → %d×%d (BGRA)", path, width, height, fb_w, fb_h);

    for (int y = 0; y < fb_h; y++) {
        unsigned src_y = (unsigned)((uint64_t)y * height / (unsigned)fb_h);
        for (int x = 0; x < fb_w; x++) {
            unsigned src_x = (unsigned)((uint64_t)x * width / (unsigned)fb_w);
            const unsigned char *px = raw + (src_y * width + src_x) * 4;
            uint8_t *dst = fb + ((size_t)y * fb_w + x) * 4;
            dst[0] = px[2]; // B  (swap R↔B for BGR framebuffer)
            dst[1] = px[1]; // G
            dst[2] = px[0]; // R
            dst[3] = px[3]; // A  (preserved)
        }
    }

    free(raw);
    return ESP_OK;
}

/* ── Palette builder: frequency-ranked 4-bit histogram ──────────────────────
 * idx = (R>>4)<<8 | (G>>4)<<4 | (B>>4)  →  4096 buckets, 16 KB DRAM.
 * Fills palette with the ≤127 most-frequent colour buckets, most common first.
 * Bucket centre = (nibble << 4) | 8  (midpoint of the 16-unit range).
 * Transparent (alpha==0) pixels are excluded.
 */
static int build_palette_freq(
        const uint8_t *overlay_bgra,
        int off_x, int off_y,
        int img_w, int img_h,
        int lcd_w,
        uint8_t palette[128][3])
{
    static uint32_t hist[4096];   /* 4-bit per channel: 16×16×16 buckets, DRAM */
    memset(hist, 0, sizeof(hist));

    for (int y = 0; y < img_h; y++) {
        const uint8_t *row = overlay_bgra + ((size_t)(off_y + y) * lcd_w + off_x) * 4;
        for (int x = 0; x < img_w; x++, row += 4) {
            if (row[3] == 0) continue;
            uint32_t idx = ((uint32_t)(row[2] >> 4) << 8)   /* R → bits[11:8] */
                         | ((uint32_t)(row[1] >> 4) << 4)   /* G → bits[7:4]  */
                         |  (uint32_t)(row[0] >> 4);        /* B → bits[3:0]  */
            hist[idx]++;
        }
    }

    /* Pick top-127 buckets by pixel count (partial selection, O(127×4096)). */
    int n = 0;
    while (n < 127) {
        uint32_t best_cnt = 0;
        int      best_idx = -1;
        for (int i = 0; i < 4096; i++) {
            if (hist[i] > best_cnt) { best_cnt = hist[i]; best_idx = i; }
        }
        if (best_idx < 0) break;
        hist[best_idx] = 0;   /* remove from future rounds */

        palette[n][0] = (uint8_t)(((best_idx      ) & 0xF) << 4 | 8);  /* B */
        palette[n][1] = (uint8_t)(((best_idx >>  4) & 0xF) << 4 | 8);  /* G */
        palette[n][2] = (uint8_t)(((best_idx >>  8) & 0xF) << 4 | 8);  /* R */
        n++;
    }
    return n;
}
void clearFramebuffers()
{
    #if VIDEO_FB_YUV422
        init_yuv422_framebuffer(s_fb_front, LCD_H_RES, LCD_V_RES);
        init_yuv422_framebuffer(s_fb_back, LCD_H_RES, LCD_V_RES);
    #else
        memset(s_fb_front, 0x00, (LCD_H_RES) * (LCD_V_RES) * sizeof(uint8_t)*VIDEO_FB_BPP);
        memset(s_fb_back,  0x00, (LCD_H_RES) * (LCD_V_RES) * sizeof(uint8_t)*VIDEO_FB_BPP);
    #endif

}

// returns pointer
/* Load a PNG (with alpha), scale it to (img_w x img_h), and centre it on a
 * full-screen BGRA overlay buffer (LCD_H_RES x LCD_V_RES, 4 bytes/pixel).
 * Surrounding area is filled with transparent black (alpha=0).
 * Pass img_w=0 / img_h=0 to stretch to full screen.                      */
 char lastOverlay[MAX_ROM_NAME];
esp_err_t loadOverlayRGB(char *name, int img_w, int img_h)
{
    strncpy(lastOverlay, name, MAX_ROM_NAME-1);

    /* clamp / default to full screen */
    if (img_w <= 0 || img_w > LCD_H_RES) img_w = LCD_H_RES;
    if (img_h <= 0 || img_h > LCD_V_RES) img_h = LCD_V_RES;

    /* free previous overlay buffers */
    if (s_overlay != NULL)    { heap_caps_free(s_overlay);    s_overlay    = NULL; }
    if (s_overlay_bg != NULL) { heap_caps_free(s_overlay_bg); s_overlay_bg = NULL; }
    if (s_overlay_pal != NULL){ heap_caps_free(s_overlay_pal);s_overlay_pal= NULL; }

    clearFramebuffers();
    if (!overlayEnabled) return ESP_OK;

    s_overlay_pal_n = 0;
    s_ov_w = 0;

    /* allocate full-screen BGRA overlay buffer in PSRAM (4 bytes/pixel) */
    size_t buf_sz = (size_t)LCD_H_RES * LCD_V_RES * 4;
    s_overlay = heap_caps_malloc(buf_sz, MALLOC_CAP_SPIRAM);
    if (!s_overlay)
    {
        ESP_LOGE(TAG, "PSRAM alloc failed (%u bytes)", buf_sz);
        return ESP_ERR_NO_MEM;
    }

    /* fill entire buffer with transparent black */
    memset(s_overlay, 0, buf_sz);
    uint8_t *scaled=NULL;
    /* decode + scale PNG into a temporary BGRA buffer */
    if (mode ==VIDEO_OUT_HDMI)
    {
        scaled = heap_caps_malloc((size_t)img_w * img_h * 4, MALLOC_CAP_SPIRAM);
        if (!scaled)
        {
            ESP_LOGE(TAG, "PSRAM alloc for scaled image failed");
            heap_caps_free(s_overlay);
            s_overlay = NULL;
            return ESP_ERR_NO_MEM;
        }

        esp_err_t ret = overlay_load_png_bgra(name, scaled, img_w, img_h);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "PNG load failed");
            heap_caps_free(scaled);
            heap_caps_free(s_overlay);
            s_overlay = NULL;
            return ret;
        }
    }
    else
    {
        /* LCD portrait: load PNG in landscape orientation (swapped dims), then
        * rotate 90° CW so it fills the portrait screen.                       */
        int load_w = img_h, load_h = img_w;
        scaled = heap_caps_malloc((size_t)load_w * load_h * 4, MALLOC_CAP_SPIRAM);
        if (!scaled)
        {
            ESP_LOGE(TAG, "PSRAM alloc for scaled image failed");
            heap_caps_free(s_overlay);
            s_overlay = NULL;
            return ESP_ERR_NO_MEM;
        }

        esp_err_t ret = overlay_load_png_bgra(name, scaled, load_w, load_h);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "PNG load failed");
            heap_caps_free(scaled);
            heap_caps_free(s_overlay);
            s_overlay = NULL;
            return ret;
        }

        /* rotate 90° CW: src pixel (sx=dy, sy=load_h-1-dx) → dst pixel (dx, dy) */
        uint8_t *rotated = heap_caps_malloc((size_t)img_w * img_h * 4, MALLOC_CAP_SPIRAM);
        if (!rotated)
        {
            ESP_LOGE(TAG, "PSRAM alloc for rotated image failed");
            heap_caps_free(scaled);
            heap_caps_free(s_overlay);
            s_overlay = NULL;
            return ESP_ERR_NO_MEM;
        }
        for (int dy = 0; dy < img_h; dy++) {
            for (int dx = 0; dx < img_w; dx++) {
                const uint8_t *s = scaled  + ((size_t)(load_h - 1 - dx) * load_w + dy) * 4;
                uint8_t       *d = rotated + ((size_t)dy * img_w + dx) * 4;
                d[0] = s[0]; d[1] = s[1]; d[2] = s[2]; d[3] = s[3];
            }
        }
        heap_caps_free(scaled);
        scaled = rotated;
    }

    /* centre position */
    int off_x = (LCD_H_RES - img_w) / 2;
    int off_y = (LCD_V_RES - img_h) / 2;

    /* blit scaled image into the centre of the full-screen buffer */
    for (int y = 0; y < img_h; y++)
    {
        const uint8_t *src = scaled    + (size_t)y * img_w * 4;
        uint8_t       *dst = s_overlay + ((size_t)(off_y + y) * LCD_H_RES + off_x) * 4;
        memcpy(dst, src, (size_t)img_w * 4);
    }

    heap_caps_free(scaled);

    /* Build precomputed RGB888 background (overlay-over-black).
     * undraw_line_rgb888_overlay restores from this — no per-pixel alpha
     * math in the hot path, just two memcpys per bounding-box row.        */
    size_t bg_sz = (size_t)LCD_H_RES * LCD_V_RES * 3;
    s_overlay_bg = heap_caps_malloc(bg_sz, MALLOC_CAP_SPIRAM);
    if (s_overlay_bg) 
    {
        const uint8_t *src = s_overlay;
        uint8_t       *dst = s_overlay_bg;
        int total = LCD_H_RES * LCD_V_RES;
        for (int i = 0; i < total; i++, src += 4, dst += 3) {
            uint8_t a = src[3];
            if (a == 0) 
            {
                dst[0] = dst[1] = dst[2] = 0;
            } 
            else if (a == 255) 
            {
                dst[0] = src[0]; dst[1] = src[1]; dst[2] = src[2];
            } 
            else 
            {
                dst[0] = (uint8_t)((src[0] * s_overlay_alpha_val ) >> 8);
                dst[1] = (uint8_t)((src[1] * s_overlay_alpha_val ) >> 8);
                dst[2] = (uint8_t)((src[2] * s_overlay_alpha_val ) >> 8);
            }
        }
    }

    /* ── Build palettised index for the active region ──────────────────── */
    s_ov_off_x = off_x;
    s_ov_off_y = off_y;
    s_ov_w     = img_w;
    s_ov_h     = img_h;

    s_overlay_pal = heap_caps_malloc((size_t)img_w * img_h, MALLOC_CAP_SPIRAM);
    if (s_overlay_pal) 
    {
        s_overlay_pal_n = 0;
        memset(s_overlay_palette, 0, sizeof(s_overlay_palette));

        /* Pass 1: median-cut quantisation → 127-entry BGR palette. */
        s_overlay_pal_n = build_palette_freq(
                s_overlay, off_x, off_y, img_w, img_h, LCD_H_RES,
                s_overlay_palette);

        /* Pass 2: assign index bytes. */
        for (int y = 0; y < img_h; y++) 
        {
            const uint8_t *src = s_overlay + ((size_t)(off_y + y) * LCD_H_RES + off_x) * 4;
            uint8_t       *dst = s_overlay_pal + (size_t)y * img_w;
            for (int x = 0; x < img_w; x++, src += 4, dst++) 
            {
                uint8_t b = src[0], g = src[1], r = src[2], a = src[3];
                int best = 0, best_d = 0x7FFFFFFF;
                
                for (int i = 0; i < s_overlay_pal_n; i++) 
                {
                    int db = (int)b - s_overlay_palette[i][0];
                    int dg = (int)g - s_overlay_palette[i][1];
                    int dr = (int)r - s_overlay_palette[i][2];
                    int d  = db*db + dg*dg + dr*dr;
                    if (d < best_d) { best = i; best_d = d; }
                    if (d == 0) break;
                }
                if (a <= 0 || a >= 255) 
                { 
                    *dst = (uint8_t)(best & (~0x80));
                }
                else
                {
                    *dst = (uint8_t)(0x80 | best);
                }
            }
        }
        /* Build YUV palette caches used by the YUV422 distance-field renderer. */
        {
            int ea = s_overlay_alpha_val;
            for (int i = 0; i < 128; i++) {
                int b = s_overlay_palette[i][0];
                int g = s_overlay_palette[i][1];
                int r = s_overlay_palette[i][2];
                /* full-range YUV for draw blending */
                s_overlay_palette_yuv[i][0] = (uint8_t)((77*r + 150*g + 29*b) >> 8);
                int u = 128 + ((-43*r - 85*g + 128*b) >> 8);
                int v = 128 + ((128*r - 107*g - 21*b) >> 8);
                s_overlay_palette_yuv[i][1] = (uint8_t)(u < 0 ? 0 : u > 255 ? 255 : u);
                s_overlay_palette_yuv[i][2] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
                /* ea-scaled YUV for undraw restore */
                int bs = (b * ea) >> 8, gs = (g * ea) >> 8, rs = (r * ea) >> 8;
                s_overlay_palette_yuv_ea[i][0] = (uint8_t)((77*rs + 150*gs + 29*bs) >> 8);
                int ue = 128 + ((-43*rs - 85*gs + 128*bs) >> 8);
                int ve = 128 + ((128*rs - 107*gs - 21*bs) >> 8);
                s_overlay_palette_yuv_ea[i][1] = (uint8_t)(ue < 0 ? 0 : ue > 255 ? 255 : ue);
                s_overlay_palette_yuv_ea[i][2] = (uint8_t)(ve < 0 ? 0 : ve > 255 ? 255 : ve);
            }
        }
        ESP_LOGI(TAG, "overlay pal: %d colours, alpha_val=%d", s_overlay_pal_n, s_overlay_alpha_val);
    }
    drawOverlayPal(s_fb_front);
    drawOverlayPal(s_fb_back);

    ESP_LOGI(TAG, "overlay: %s scaled to %dx%d, centred on %dx%d screen (BGRA)",
             name, img_w, img_h, LCD_H_RES, LCD_V_RES);
    return ESP_OK;
}
void initGlobals()
{
    overlayEnabled =  ENABLE_OVERLAYS;
    mode = VIDEO_OUT_SELECTED;          // default at boot
    changeGlobalLineValues( LINE_WIDTH, LINE_GLOW_WIDTH);
}


// ----------------------------------------------------
// app_main
// ----------------------------------------------------
void app_main(void)
{
    initGlobals();
    
    // fill rom with 01, see https://vectrex-emu.blogspot.com/2006/07/
    memset(cartData, 0x01, MAX_CART_SIZE * sizeof(uint8_t));

#if VECX_DEBUG == 1    
    esp_log_level_set("lcd.dsi.dpi", ESP_LOG_DEBUG);   // show the actual DPI pixel clock achieved
    esp_log_level_set("lcd_panel.dpi", ESP_LOG_DEBUG);
    esp_log_level_set("mipi_dsi", ESP_LOG_DEBUG);
    esp_log_level_set("*", ESP_LOG_DEBUG);  

#else    
    /*
    ESP_LOG_NONE — no output at all
    ESP_LOG_ERROR — only errors
    ESP_LOG_WARN — errors and warnings
    ESP_LOG_INFO
    ESP_LOG_DEBUG
    ESP_LOG_VERBOSE
    */
    esp_log_level_set("lcd.dsi.dpi", ESP_LOG_ERROR);   // show the actual DPI pixel clock achieved
    esp_log_level_set("lcd_panel.dpi", ESP_LOG_ERROR);
    esp_log_level_set("mipi_dsi", ESP_LOG_ERROR);
    esp_log_level_set(TAG, ESP_LOG_ERROR);
    esp_log_level_set(TAG_A, ESP_LOG_ERROR);
    esp_log_level_set("*", ESP_LOG_ERROR);  
#endif


    board_assert_vcv_noe();
    board_enable_dsi_phy_power();

    // 1) I2C master bus (same pins/driver as the i2c_tools bring-up project).
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus));


    if (initSD() != ESP_OK)
    {
        printf("Something went wrong with the SDCard init - did you insert a card?");
    }
    else
    {
        if (!read_ini_rom_name(cartName, sizeof(cartName)))
        {
            printf("INI konnte nicht gelesen werden\n");
        }
        else
        {
            printf("ROM Name in ini: %s\n", cartName);
//            cartSize = load_rom_file(cartName, cartData, sizeof(cartData));

cartSize = load_rom_file("KARL.BIN", cartData, sizeof(cartData));
//cartSize = load_rom_file("VBLADE.NIB", cartData, sizeof(cartData));
//cartSize = load_rom_file("AKLABETH.BIN", cartData, sizeof(cartData));
//cartSize = load_rom_file("BERZERKU.BIN", cartData, sizeof(cartData));
            if (cartSize < 0) {
                printf("ROM konnte nicht geladen werden (%ld)\n", cartSize);
                cartSize = 0;
            }
            else
            {
                printf("ROM geladen: %ld Bytes\n", cartSize);
                for (int i = 0; i < 16; i++)
                    printf("$%02x ", cartData[i]);
                printf("\n");
            }
        }
    }

    if (usb_keyboard_init() != ESP_OK)
    {
        printf("Something went wrong with the keyboard init - did you connect a keyboard?");
    }

    ESP_LOGI(TAG, "Init LCD / DSI");
    lcd_init();


    ESP_LOGI(TAG, "VIDEO_FB_BPP: %i", VIDEO_FB_BPP);

    ESP_LOGI(TAG, "LCD_H_RES: %i, LCD_V_RES: %i", LCD_H_RES, LCD_V_RES);

    // Logische Frame-Slots initialisieren
    frames_init();

    if (mode == VIDEO_OUT_HDMI)
        loadOverlayRGB("/sdcard/KARL.png", HDMI_OVERLAY_WIDTH, HDMI_OVERLAY_HEIGHT);
    else
        loadOverlayRGB("/sdcard/KARL.png", LCD_OVERLAY_WIDTH, LCD_OVERLAY_HEIGHT);

    ESP_LOGI(TAG, "Start vectrex tasks");

#ifdef CONFIG_ESP_TASK_WDT_EN
    // Task-Watchdog aus (Entwicklung)
    esp_task_wdt_deinit();
#endif

    ESP_ERROR_CHECK(audio_init());


    if (mode == VIDEO_OUT_HDMI)
    {
        gpio_set_level(GPIO_OUTPUT_PA, 0);   // speaker off
        /* enable LT8912B I2S audio input — CEC bank reg 0xB2 */
        uint8_t val = 0x01;
        esp_lcd_panel_io_tx_param(lt_io.cec_dsi, 0xB2, &val, 1);
    }
    else
    {
        // switch off with
        gpio_set_level(GPIO_OUTPUT_PA, 1);   // speaker on
        uint8_t val = 0x00;
        esp_lcd_panel_io_tx_param(lt_io.cec_dsi, 0xB2, &val, 1);
    }

    void callbackAY(void *userdata, int16_t *stream, int length);
#ifndef NO_AUDIO
    audio_set_callback(callbackAY, NULL);
#endif
    printf("Audio init done\n");

    xTaskCreatePinnedToCore(
        audio_music_task,        // Task-Funktion
        "audio_music_task",      // Name
        8192,                    // Stackgröße (Wort, nicht Byte)
        NULL,                    // Parameter
        20,                       // Priorität
        &s_audio_task_handle,    // Handle
        1                        // Core 0
    );

    vecx_init();

    // Emulator task (core 1) — static stack in internal SRAM
    xTaskCreateStaticPinnedToCore(
        application_task,
        "application",
        EMU_STACK_SIZE,
        NULL,
        7,
        s_emu_stack,
        &s_emu_tcb,
        1
    );

    // Renderer task (core 0) — static stack in internal SRAM
    xTaskCreateStaticPinnedToCore(
        renderer_task,
        "renderer",
        REND_STACK_SIZE,
        NULL,
        3,
        s_rend_stack,
        &s_rend_tcb,
        0
    );

#if VECX_DEBUG == 1    
    printf("Free internal DRAM: %d bytes\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    printf("Largest free DRAM block: %d bytes\n", heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));    
    //heap_caps_print_heap_info(MALLOC_CAP_8BIT);
#endif    
}
void toggleVideoModeRequest()
{
    s_toggle_display_mode = 1;   // atomic on 32-bit aligned write on RISC-V
}
void toggleOverlayRequest()
{
    printf("Overlay toggled, from %i\n", overlayEnabled);
    s_toggle_overlay_mode = 1;   // atomic on 32-bit aligned write on RISC-V
}

void toggleOverlay()
{
	if (overlayEnabled) overlayEnabled=0; else overlayEnabled = 1; 
    if (mode == VIDEO_OUT_HDMI)
        loadOverlayRGB(lastOverlay, HDMI_OVERLAY_WIDTH, HDMI_OVERLAY_HEIGHT);
    else
        loadOverlayRGB(lastOverlay, LCD_OVERLAY_WIDTH, LCD_OVERLAY_HEIGHT);
}

void toggleVideoMode()
{
    // shut Down
    lcd_deinit();

    vTaskDelay(pdMS_TO_TICKS(20)); // I never needed this - but doc says after bus reinit - one should wait a bit?

    if (mode == VIDEO_OUT_HDMI)  mode = VIDEO_OUT_LVDS;
    else   mode = VIDEO_OUT_HDMI;

    lcd_init();
    // Vecx
    void resize(int width, int height);//

    // audio is simply a switch 
    if (mode == VIDEO_OUT_HDMI)
    {
        gpio_set_level(GPIO_OUTPUT_PA, 0);   // speaker off
        /* enable LT8912B I2S audio input — CEC bank reg 0xB2 */
        uint8_t val = 0x01;
        esp_lcd_panel_io_tx_param(lt_io.cec_dsi, 0xB2, &val, 1);

        // Vecx
        SCREEN_HEIGHT = LCD_V_RES;
		SCREEN_WIDTH = LCD_H_RES;
		resize( HDMI_VECX_WIDTH, HDMI_VECX_HEIGHT);
        loadOverlayRGB(lastOverlay, HDMI_OVERLAY_WIDTH, HDMI_OVERLAY_HEIGHT);
    }
    else
    {
        // switch off with
        gpio_set_level(GPIO_OUTPUT_PA, 1);   // speaker on
        uint8_t val = 0x00;
        esp_lcd_panel_io_tx_param(lt_io.cec_dsi, 0xB2, &val, 1);

        // Vecx
		SCREEN_HEIGHT = LCD_H_RES;
		SCREEN_WIDTH = LCD_V_RES;
		resize( LCD_VECX_WIDTH, LCD_VECX_HEIGHT);
        loadOverlayRGB(lastOverlay, LCD_OVERLAY_WIDTH, LCD_OVERLAY_HEIGHT);
    }

}
