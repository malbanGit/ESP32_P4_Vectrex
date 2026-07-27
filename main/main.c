// karl on LCD demo 1 46 !!!!
// spike is now very VERY slow???? -> Spike IS so slow!!!
#include "defines.h"

/*
Bug:

    pole position flimmert bei game over und hat emu fps von 43?
    done: spike speaks too fast - 17 is correct!

    lunar lander, red baron, Asteroids and battlezone analog sounds
    For BattleZone clipping!
    gravitar to fast???

    lunar lander after first live - slow, joystick does not center
    red baron joystick does not center

    reboot "m" does not always work (asteroids, lunar)


    ->
    DER THROTTLE
    MUSS in bezug auf 3000 Zyklen sein, NICHT
    in Bezug auf End frame!!!!
TODO    
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




 to test - switch when hdmi is not connected?

Highscore saving
joystick
settings save ti ini
sound volume

TODO: Calibration Ala Tuts

 build a audio mixer for samples - which should be "stackable"
int playWAV();
setSoundCallback(*functionPointer)


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


#include <math.h>

#include "board.h"
#include "hdmi.h"
#include "lvds.h"
#include "lodepng.h"

#include "draw_line_yuv422.h"



char * audio_buf;
size_t audio_bufsize;



DRAM_ATTR uint32_t g_beam_r2_q16;
DRAM_ATTR int g_line_Rb = (((LINE_WIDTH >> 1) + LINE_GLOW_WIDTH) > 0) ? ((LINE_WIDTH >> 1) + LINE_GLOW_WIDTH - 1) : 0;
DRAM_ATTR int g_beam_r = LINE_WIDTH >> 1;
DRAM_ATTR uint32_t g_gs;
DRAM_ATTR int g_color_mode; // 0 = not color, 1 = color
DRAM_ATTR int g_fpsToReach=  1000000/MAX_EMU_FPS;
DRAM_ATTR volatile input_state_t g_inputState={127,127,127,127,255}; 

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

DRAM_ATTR uint8_t  s_overlay_alpha_val = GLOBAL_OVERLAY_ALPHA;  /* representative raw alpha */

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

typedef struct {
    int16_t x0;
    int16_t y0;
    int16_t x1;
    int16_t y1;
    uint8_t brightness;
    uint32_t color; // 
    uint8_t  _pad[2];
} vectrex_line_t; // power of 2 length

typedef struct {
    vectrex_line_t      lines[MAX_LINE_BUFFER]; // these are the "new" lines - that will be drawn by the renderer
    int                 line_count;
    uint32_t            hash;
    volatile frame_state_t state;
} frame_slot_t;

typedef struct { int bx0, by0, bx1, by1; } line_bbox_t;

// Diese beschreiben, was aktuell in jedem Hardware-Framebuffer gezeichnet ist
DRAM_ATTR static vectrex_line_t s_fb_lines[NUM_FB][MAX_LINE_BUFFER]; // these are the last drawn lines by the renderer - used to undraw!
DRAM_ATTR static int            s_fb_line_count[NUM_FB] = {0};
DRAM_ATTR static uint8_t     s_diff_old_matched[MAX_LINE_BUFFER];
DRAM_ATTR static uint8_t     s_diff_new_matched[MAX_LINE_BUFFER];
DRAM_ATTR static uint8_t     s_diff_damaged[MAX_LINE_BUFFER];
DRAM_ATTR static line_bbox_t s_diff_dirty_bboxes[MAX_LINE_BUFFER];

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
DRAM_ATTR int      s_ov_off_x = 0;             /* active region x offset   */
DRAM_ATTR int      s_ov_off_y = 0;             /* active region y offset   */
DRAM_ATTR int      s_ov_w     = 0;             /* active region width      */
DRAM_ATTR int      s_ov_h     = 0;             /* active region height     */

// ----------------------------------------------------
// Per-framebuffer line storage for undraw (Hardware-FBs)
// ----------------------------------------------------

/* Task stacks pinned to internal SRAM so function calls never touch PSRAM. */
#define EMU_STACK_SIZE  (8192+4096)
#define REND_STACK_SIZE (8192+4096)
static StackType_t  s_emu_stack[EMU_STACK_SIZE];
static DRAM_ATTR StaticTask_t s_emu_tcb;
static StackType_t  s_rend_stack[REND_STACK_SIZE];
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
#include "esp_events.i"
#include "overlay.i"

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
// all YUV
IRAM_ATTR static inline void undrawLine_raw_color(int x0, int y0, int x1, int y1, uint8_t colorPaletteEntry, uint8_t brightness)
{
    int b = brightness+brightnessAdjust;
    undraw_line_yuv422_color(s_fb_back, LCD_H_RES, LCD_V_RES, x0, y0, x1, y1, colorPaletteEntry, b);
}
IRAM_ATTR static inline void drawLine_raw_color(int x0, int y0, int x1, int y1, uint8_t colorPaletteEntry, uint8_t brightness)
{
    int b = brightness+brightnessAdjust;
    draw_line_yuv422_color(s_fb_back, LCD_H_RES, LCD_V_RES, x0, y0, x1, y1, colorPaletteEntry, b);
}
IRAM_ATTR static inline void undrawLine_raw(int x0, int y0, int x1, int y1, uint8_t brightness)
{
    int b = brightness+brightnessAdjust;
    if (s_overlay == NULL)
    {
        if ((x0==x1) && (y0==y1))
        undraw_line_yuv422_brightness(s_fb_back, LCD_H_RES, LCD_V_RES, x0, y0, x1, y1, b+800);
        else
        undraw_line_yuv422_brightness(s_fb_back, LCD_H_RES, LCD_V_RES, x0, y0, x1, y1, b);
    }
    else        
    {
        undraw_line_yuv422_overlay(s_fb_back, LCD_H_RES, LCD_V_RES, x0, y0, x1, y1, b, s_overlay);
    }
}
IRAM_ATTR static inline void drawLine_raw(int x0, int y0, int x1, int y1, uint8_t brightness)
{
    if (brightness == 0)  return;
    int b = brightness+brightnessAdjust;
    if (s_overlay == NULL)
    {
        //draw_line_yuv422_brightness(s_fb_back, LCD_H_RES, LCD_V_RES, x0, y0, x1, y1, b);

        if ((x0==x1) && (y0==y1))
        draw_line_yuv422_brightness(s_fb_back, LCD_H_RES, LCD_V_RES, x0, y0, x1, y1, b+800);
        else
        draw_line_yuv422_brightness(s_fb_back, LCD_H_RES, LCD_V_RES, x0, y0, x1, y1, b);


    }
    else
    {
        draw_line_yuv422_overlay(s_fb_back, LCD_H_RES, LCD_V_RES, x0, y0, x1, y1, b, s_overlay);
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

    if (mode == VIDEO_OUT_HDMI)
    {
        if (!hdmi_hpd_present(&lt_io))
        {
            mode = VIDEO_OUT_LVDS;
        }
    }

    esp_lcd_dsi_bus_handle_t dsi_bus = NULL;
    esp_lcd_dsi_bus_config_t dsi_bus_cfg = LT8912B_PANEL_BUS_DSI_2CH_CONFIG();
    if (mode == VIDEO_OUT_LVDS)
        dsi_bus_cfg.lane_bit_rate_mbps = BOARD_DSI_LANE_MBPS;   // single source: board.h
    else
        dsi_bus_cfg.lane_bit_rate_mbps = 1200;   // single source: custom

    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&dsi_bus_cfg, &dsi_bus));

    const bool yuv422 = VIDEO_FB_YUV422; // 
    panel_handle = NULL;
    int w,h;
    ESP_ERROR_CHECK(video_start(mode, yuv422, &lt_io, dsi_bus, &panel_handle, &w, &h));

    LCD_H_RES = w;
    LCD_V_RES = h;
    s_dpi_panel = panel_handle;

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
    for (int i = 0; i < NUM_FB; ++i) 
        s_fb_line_count[i] = 0;

}

void clearFramebuffers()
{
    frames_init();
    for (int i=0;i<NUM_FB;i++) s_fb_line_count[i]=0;

    #if VIDEO_FB_YUV422
        init_yuv422_framebuffer(s_fb_front, LCD_H_RES, LCD_V_RES);
        init_yuv422_framebuffer(s_fb_back, LCD_H_RES, LCD_V_RES);
    #else
        memset(s_fb_front, 0x00, (LCD_H_RES) * (LCD_V_RES) * sizeof(uint8_t)*VIDEO_FB_BPP);
        memset(s_fb_back,  0x00, (LCD_H_RES) * (LCD_V_RES) * sizeof(uint8_t)*VIDEO_FB_BPP);
    #endif
}

// ----------------------------------------------------
// Emulator-side API
// ----------------------------------------------------
// when in color mode
// color from palette 127
// color = 0 -> undraw
IRAM_ATTR void mini_draw_line_color(int x0, int y0, int x1, int y1, int color, uint8_t brightness)
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
        l->color = color;
    }
    else {
        // Buffer voll – zusätzliche Linien werden verworfen
        // Optional: Debug-Ausgabe
        // printf("Emulation Exceeded line Count!!! (%d)\n", fs->line_count);
    }
}

// Emulator calls this for each line in the CURRENT emulated frame
IRAM_ATTR void mini_draw_line(int x0, int y0, int x1, int y1, uint8_t brightness)
{
    int idx = s_build_frame_index;
    frame_slot_t *fs = &s_frames[idx];

    if (fs->state != FRAME_BUILDING) {
        // Slot ist nicht im BUILDING-Zustand – dann nichts schreiben
        printf("No Building frame!");
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
    }
    else {
        // Buffer voll – zusätzliche Linien werden verworfen
        // Optional: Debug-Ausgabe
        //printf("Emulation Exceeded line Count!!! (%d)\n", fs->line_count);
    }
}

// Emulator calls this once when a frame is complete
// the lines collected from the emulator
// are kept in 3 buffers
// the state of the buffer determines which is drawn on screen
// and which is used to recieve "new" lines.
uint64_t start=0;
IRAM_ATTR void mini_end_frame(void)
{
    static uint64_t emu_last_time   = 0;
    static int      emu_frame_count = 0;
//printf("Frame end");
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
    if (now - emu_last_time > 1000000) // has one second passed?
    {
        printf("EMU FPS = %d\n", emu_frame_count);
        emu_frame_count = 0;
        emu_last_time   = now;
    }

    // keyboard / joystick events are "collected" once every round
    // if we ever support a spinner - it might have to be called more often
    void readevents();
    readevents();

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
    for (int i = 0; i < count; ++i) {
        vectrex_line_t *l = &s_fb_lines[fb_index][i];
        undrawLine_raw((int16_t)l->x0, (int16_t)l->y0, (int16_t)l->x1,(int16_t)l->y1, l->brightness);
    }
    s_fb_line_count[fb_index] = 0;
}
IRAM_ATTR static inline void undraw_previous_fb_color(int fb_index)
{
    int count = s_fb_line_count[fb_index];
    for (int i = 0; i < count; ++i) {
        vectrex_line_t *l = &s_fb_lines[fb_index][i];
        undrawLine_raw_color((int16_t)l->x0,(int16_t) l->y0, (int16_t)l->x1, (int16_t)l->y1, l->color, l->brightness);
    }
    s_fb_line_count[fb_index] = 0;
}

void set_yuv(int i, int r, int g, int b)
{
    s_overlay_palette_yuv[i][0] = (uint8_t)((77*r + 150*g + 29*b) >> 8);
    int u = 128 + ((-43*r - 85*g + 128*b) >> 8);
    int v = 128 + ((128*r - 107*g - 21*b) >> 8);
    s_overlay_palette_yuv[i][1] = (uint8_t)(u < 0 ? 0 : u > 255 ? 255 : u);
    s_overlay_palette_yuv[i][2] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
}

void yuv_palette_init(void)
{
    /* Neutrals */
    set_yuv(YUV_PALETTE_BLACK,          0,   0,   0);
    set_yuv(YUV_PALETTE_DARKGRAY,      45,  45,  45);
    set_yuv(YUV_PALETTE_GRAY,         100, 100, 100);
    set_yuv(YUV_PALETTE_MIDGRAY,      128, 128, 128);
    set_yuv(YUV_PALETTE_LIGHTGRAY,    180, 180, 180);
    set_yuv(YUV_PALETTE_SILVER,       210, 210, 210);
    set_yuv(YUV_PALETTE_OFFWHITE,     240, 240, 240);
    set_yuv(YUV_PALETTE_WHITE,        255, 255, 255);

    /* Reds */
    set_yuv(YUV_PALETTE_DARKMAROON,    64,   0,   0);
    set_yuv(YUV_PALETTE_MAROON,       128,   0,   0);
    set_yuv(YUV_PALETTE_DARKRED,      180,   0,   0);
    set_yuv(YUV_PALETTE_RED,          255,   0,   0);
    set_yuv(YUV_PALETTE_BRIGHTRED,    255,  40,  40);
    set_yuv(YUV_PALETTE_CORAL,        255, 127,  80);
    set_yuv(YUV_PALETTE_SALMON,       250, 128, 114);
    set_yuv(YUV_PALETTE_LIGHTCORAL,   240, 128, 128);

    /* Oranges / Yellows */
    set_yuv(YUV_PALETTE_DARKORANGE,   180,  80,   0);
    set_yuv(YUV_PALETTE_ORANGE,       255, 128,   0);
    set_yuv(YUV_PALETTE_BRIGHTORANGE, 255, 160,   0);
    set_yuv(YUV_PALETTE_GOLD,         255, 200,   0);
    set_yuv(YUV_PALETTE_YELLOW,       255, 255,   0);
    set_yuv(YUV_PALETTE_LIGHTYELLOW,  255, 255, 128);
    set_yuv(YUV_PALETTE_AMBER,        255, 180,   0);
    set_yuv(YUV_PALETTE_KHAKI,        200, 200, 100);

    /* Greens */
    set_yuv(YUV_PALETTE_DARKFOREST,     0,  50,   0);
    set_yuv(YUV_PALETTE_FOREST,         0, 100,   0);
    set_yuv(YUV_PALETTE_DARKGREEN,      0, 150,   0);
    set_yuv(YUV_PALETTE_GREEN,          0, 200,   0);
    set_yuv(YUV_PALETTE_BRIGHTGREEN,    0, 255,   0);
    set_yuv(YUV_PALETTE_LIGHTGREEN,   100, 255, 100);
    set_yuv(YUV_PALETTE_PALEGREEN,    180, 255, 180);
    set_yuv(YUV_PALETTE_YELLOWGREEN,  150, 220,  50);

    /* Cyans / Teals */
    set_yuv(YUV_PALETTE_DARKTEAL,       0,  64,  64);
    set_yuv(YUV_PALETTE_TEAL,           0, 128, 128);
    set_yuv(YUV_PALETTE_DARKCYAN,       0, 180, 180);
    set_yuv(YUV_PALETTE_CYAN,           0, 255, 255);
    set_yuv(YUV_PALETTE_LIGHTCYAN,    180, 255, 255);
    set_yuv(YUV_PALETTE_AQUAMARINE,   100, 230, 200);
    set_yuv(YUV_PALETTE_TURQUOISE,     64, 224, 208);
    set_yuv(YUV_PALETTE_MEDTURQUOISE,  72, 209, 204);

    /* Blues */
    set_yuv(YUV_PALETTE_DARKNAVY,       0,   0,  64);
    set_yuv(YUV_PALETTE_NAVY,           0,   0, 128);
    set_yuv(YUV_PALETTE_DARKBLUE,       0,   0, 180);
    set_yuv(YUV_PALETTE_BLUE,           0,   0, 255);
    set_yuv(YUV_PALETTE_ROYALBLUE,     65, 105, 225);
    set_yuv(YUV_PALETTE_CORNFLOWER,   100, 149, 237);
    set_yuv(YUV_PALETTE_SKYBLUE,      135, 206, 235);
    set_yuv(YUV_PALETTE_LIGHTBLUE,    180, 220, 255);

    /* Purples / Violets */
    set_yuv(YUV_PALETTE_DARKINDIGO,    40,   0,  80);
    set_yuv(YUV_PALETTE_INDIGO,        75,   0, 130);
    set_yuv(YUV_PALETTE_DARKPURPLE,   100,   0, 150);
    set_yuv(YUV_PALETTE_PURPLE,       128,   0, 128);
    set_yuv(YUV_PALETTE_VIOLET,       150,   0, 200);
    set_yuv(YUV_PALETTE_BLUEVIOLET,   138,  43, 226);
    set_yuv(YUV_PALETTE_MEDIUMPURPLE, 147, 112, 219);
    set_yuv(YUV_PALETTE_LAVENDER,     200, 180, 255);

    /* Pinks / Magentas */
    set_yuv(YUV_PALETTE_DARKMAGENTA,  139,   0, 139);
    set_yuv(YUV_PALETTE_MAGENTA,      255,   0, 255);
    set_yuv(YUV_PALETTE_FUCHSIA,      255,   0, 180);
    set_yuv(YUV_PALETTE_DEEPPINK,     255,  20, 147);
    set_yuv(YUV_PALETTE_HOTPINK,      255, 105, 180);
    set_yuv(YUV_PALETTE_PINK,         255, 180, 210);
    set_yuv(YUV_PALETTE_LIGHTPINK,    255, 210, 230);
    set_yuv(YUV_PALETTE_ROSE,         255,   0, 100);

    /* Browns / Earth */
    set_yuv(YUV_PALETTE_DARKBROWN,     80,  30,   0);
    set_yuv(YUV_PALETTE_BROWN,        139,  69,  19);
    set_yuv(YUV_PALETTE_SADDLEBROWN,  160,  90,  40);
    set_yuv(YUV_PALETTE_CHOCOLATE,    210, 105,  30);
    set_yuv(YUV_PALETTE_PERU,         205, 133,  63);
    set_yuv(YUV_PALETTE_TAN,          210, 180, 140);
    set_yuv(YUV_PALETTE_WHEAT,        245, 222, 179);
    set_yuv(YUV_PALETTE_SANDYBROWN,   244, 164,  96);

    /* Olive / Sea greens */
    set_yuv(YUV_PALETTE_OLIVEDARK,     60,  60,   0);
    set_yuv(YUV_PALETTE_OLIVE,        128, 128,   0);
    set_yuv(YUV_PALETTE_DARKOLIVE,    100, 110,  30);
    set_yuv(YUV_PALETTE_OLIVEDRAB,    107, 142,  35);
    set_yuv(YUV_PALETTE_SEAGREEN,      46, 139,  87);
    set_yuv(YUV_PALETTE_MEDSEAGREEN,   60, 179, 113);
    set_yuv(YUV_PALETTE_SPRINGGREEN,    0, 255, 127);
    set_yuv(YUV_PALETTE_MINTGREEN,    100, 255, 160);

    /* Blue-greens / Slate */
    set_yuv(YUV_PALETTE_CADETBLUE,     95, 158, 160);
    set_yuv(YUV_PALETTE_STEELBLUE,     70, 130, 180);
    set_yuv(YUV_PALETTE_DODGERBLUE,    30, 144, 255);
    set_yuv(YUV_PALETTE_DEEPSKYBLUE,    0, 191, 255);
    set_yuv(YUV_PALETTE_POWDERBLUE,   176, 224, 230);
    set_yuv(YUV_PALETTE_SLATEBLUE,    106,  90, 205);
    set_yuv(YUV_PALETTE_MIDNIGHTBLUE,  25,  25, 112);
    set_yuv(YUV_PALETTE_DARKSLATEBLUE, 72,  61, 139);

    /* Warm reds */
    set_yuv(YUV_PALETTE_CRIMSON,      220,  20,  60);
    set_yuv(YUV_PALETTE_SCARLET,      255,  36,   0);
    set_yuv(YUV_PALETTE_TOMATO,       255,  99,  71);
    set_yuv(YUV_PALETTE_FIREBRICK,    178,  34,  34);
    set_yuv(YUV_PALETTE_INDIANRED,    205,  92,  92);
    set_yuv(YUV_PALETTE_ROSEWOOD,     100,   0,  20);
    set_yuv(YUV_PALETTE_RUBY,         155,  17,  30);
    set_yuv(YUV_PALETTE_BURGUNDY,     128,   0,  32);

    /* Pastels */
    set_yuv(YUV_PALETTE_PASTELRED,    255, 180, 180);
    set_yuv(YUV_PALETTE_PASTELORANGE, 255, 210, 170);
    set_yuv(YUV_PALETTE_PASTELYELLOW, 255, 255, 190);
    set_yuv(YUV_PALETTE_PASTELGREEN,  180, 255, 180);
    set_yuv(YUV_PALETTE_PASTELCYAN,   180, 255, 255);
    set_yuv(YUV_PALETTE_PASTELBLUE,   180, 210, 255);
    set_yuv(YUV_PALETTE_PASTELPURPLE, 220, 180, 255);
    set_yuv(YUV_PALETTE_PASTELPINK,   255, 180, 230);

    /* Neons */
    set_yuv(YUV_PALETTE_NEONRED,      255,  20,  20);
    set_yuv(YUV_PALETTE_NEONORANGE,   255, 140,   0);
    set_yuv(YUV_PALETTE_NEONYELLOW,   240, 255,   0);
    set_yuv(YUV_PALETTE_NEONGREEN,     57, 255,  20);
    set_yuv(YUV_PALETTE_NEONCYAN,       0, 255, 240);
    set_yuv(YUV_PALETTE_NEONBLUE,      30,  80, 255);
    set_yuv(YUV_PALETTE_NEONPURPLE,   180,   0, 255);
    set_yuv(YUV_PALETTE_NEONPINK,     255,   0, 180);

    /* Electric / Saturated */
    set_yuv(YUV_PALETTE_ELECTRICBLUE,   0,  80, 255);
    set_yuv(YUV_PALETTE_ELECTRICGREEN,  0, 220,   0);
    set_yuv(YUV_PALETTE_ELECTRICPURP, 160,   0, 255);
    set_yuv(YUV_PALETTE_ELECTRICCYAN,   0, 230, 230);
    set_yuv(YUV_PALETTE_ELECTRICYELL,  220, 220,  0);
    set_yuv(YUV_PALETTE_ELECTRICORANG, 255, 100,  0);
    set_yuv(YUV_PALETTE_ELECTRICRED,   220,   0,  0);
    set_yuv(YUV_PALETTE_ELECTRICPINK,  220,   0, 150);

    /* Misc warm */
    set_yuv(YUV_PALETTE_OCHRE,        200, 150,  20);
    set_yuv(YUV_PALETTE_TERRACOTTA,   200,  90,  50);
    set_yuv(YUV_PALETTE_RUST,         180,  70,  20);
    set_yuv(YUV_PALETTE_COPPER,       185, 115,  50);
    set_yuv(YUV_PALETTE_BRONZE,       160, 100,  30);
    set_yuv(YUV_PALETTE_BRASS,        180, 160,  50);
    set_yuv(YUV_PALETTE_CHARTREUSE,   127, 255,   0);
    set_yuv(YUV_PALETTE_LIMEGREEN,     50, 205,  50);
}
// ----------------------------------------------------
// Tasks
// ----------------------------------------------------
int vecsimGame=1;
#define MAX_VECSIM_GAME 8
IRAM_ATTR static void application_task(void *arg)
{
    while (1) 
    {
        int vectrex(void); //ok

        int tempest(void); //ok
        int battlezone(void); //ok
        int blackwidow(void); //ok
        int deluxe(void); // ok
        int asteroids(void); // crashes while playing
        int gravitar(void); //ok
        int lunar(void); // to slow
        int redbaron(void); // ok
        int spaceDuel(void); // ok
        

        if (vecsimGame==1) tempest();
        if (vecsimGame==2) gravitar();
        if (vecsimGame==3) blackwidow();
        if (vecsimGame==4) spaceDuel();
        if (vecsimGame==5) redbaron();
        if (vecsimGame==6) battlezone();
        if (vecsimGame==7) deluxe();

        vectrex();
    }
}

IRAM_ATTR static inline uint32_t line_hash_key(const vectrex_line_t *l)
{
    uint32_t h = (uint32_t)l->x0 * 2654435761u;
    h ^= (uint32_t)l->y0 * 40503u;
    h ^= (uint32_t)l->x1 * 2246822519u;
    h ^= (uint32_t)l->y1 * 3266489917u;
    h ^= (uint32_t)l->brightness+l->color;
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
        if (now - fps_last_time >= 1000000) 
        {
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

        if (g_color_mode == 0)
        {
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

                    drawLine_raw((int16_t)src->x0, (int16_t)src->y0, (int16_t)src->x1, (int16_t)src->y1, src->brightness);

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
                // Steps 4+5a: undraw dirty old lines AND damaged stable lines in one pass
                for (int i = 0; i < old_count; i++) {
                    if (!s_diff_old_matched[i] || s_diff_damaged[i]) {
                        vectrex_line_t *l = &old_lines[i];
                        undrawLine_raw((int16_t)l->x0, (int16_t)l->y0, (int16_t)l->x1, (int16_t)l->y1, l->brightness);
                    }
                }

                // Step 5b: redraw ALL damaged stable lines
                for (int i = 0; i < old_count; i++) {
                    if (s_diff_old_matched[i] && s_diff_damaged[i]) {
                        vectrex_line_t *l = &old_lines[i];
                        drawLine_raw((int16_t)l->x0, (int16_t)l->y0, (int16_t)l->x1, (int16_t)l->y1, l->brightness);
                    }
                }

                // Step 6: draw dirty new lines + rebuild s_fb_lines for next frame
                s_fb_line_count[fb_idx] = 0;
                for (int j = 0; j < line_count; j++) {
                    vectrex_line_t *src = &fs->lines[j];
                    if (!s_diff_new_matched[j])
                        drawLine_raw((int16_t)src->x0, (int16_t)src->y0, (int16_t)src->x1, (int16_t)src->y1, src->brightness);
                    if (s_fb_line_count[fb_idx] < MAX_LINE_BUFFER) {
                        s_fb_lines[fb_idx][s_fb_line_count[fb_idx]++] = *src;
                    } else {
                        printf("-----------------------\n");
                        printf("Line Buffer Exceeded!!!\n");
                        printf("-----------------------\n");
                    }
                }
            }

        }
        else // now color mode = 1
        {
            // simple version
            // undraw all known old lines
            // draw all known new lines
            // regardless whether they are the same or not
            if (redraw>0)
            {
                redraw--;
                undraw_previous_fb_color(fb_idx);             // undraw all from that framebuffer

                // Draw new frame lines into backbuffer and remember them
                // linecount from "frame" (collection of lines)
                for (int i = 0; i < line_count; ++i) {
                    vectrex_line_t *src = &fs->lines[i];

                    drawLine_raw_color(src->x0, src->y0, src->x1, src->y1, src->color, src->brightness);

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
                            ol->color == nw->color &&
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

                // Steps 4+5a: undraw dirty old lines AND damaged stable lines in one pass
                for (int i = 0; i < old_count; i++) {
                    if (!s_diff_old_matched[i] || s_diff_damaged[i]) {
                        vectrex_line_t *l = &old_lines[i];
                        undrawLine_raw_color(l->x0, l->y0, l->x1, l->y1, l->color, l->brightness);
                    }
                }

                // Step 5b: redraw ALL damaged stable lines
                for (int i = 0; i < old_count; i++) {
                    if (s_diff_old_matched[i] && s_diff_damaged[i]) {
                        vectrex_line_t *l = &old_lines[i];
                        drawLine_raw_color(l->x0, l->y0, l->x1, l->y1, l->color, l->brightness);
                    }
                }

                // Step 6: draw dirty new lines + rebuild s_fb_lines for next frame
                s_fb_line_count[fb_idx] = 0;
                for (int j = 0; j < line_count; j++) {
                    vectrex_line_t *src = &fs->lines[j];
                    if (!s_diff_new_matched[j])
                        drawLine_raw_color(src->x0, src->y0, src->x1, src->y1, src->color, src->brightness);
                    if (s_fb_line_count[fb_idx] < MAX_LINE_BUFFER) {
                        s_fb_lines[fb_idx][s_fb_line_count[fb_idx]++] = *src;
                    } else {
                        printf("-----------------------\n");
                        printf("Line Buffer Exceeded!!!\n");
                        printf("-----------------------\n");
                    }
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
IRAM_ATTR static void audio_music_task(void *arg)
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

void setAppFPS(int fps)
{
    g_fpsToReach=  1000000/fps;
}
void initGlobals()
{
    overlayEnabled = ENABLE_OVERLAYS;
    mode = VIDEO_OUT_SELECTED;          // default at boot
    g_color_mode = 0;
    g_fpsToReach=  1000000/MAX_EMU_FPS;

    changeGlobalLineValues( LINE_WIDTH, LINE_GLOW_WIDTH);
}

// ----------------------------------------------------
// app_main
// ----------------------------------------------------
void app_main(void)
{
    initGlobals();
    

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

    // loading a disable overlay, frees all buffers and does not load...
    if (mode == VIDEO_OUT_HDMI)
        loadOverlayRGB(lastOverlay, HDMI_OVERLAY_WIDTH, HDMI_OVERLAY_HEIGHT);
    else
        loadOverlayRGB(lastOverlay, LCD_OVERLAY_WIDTH, LCD_OVERLAY_HEIGHT);
    fireResizeEvent();
}

void toggleVideoMode()
{
    // shut Down
    lcd_deinit();

    vTaskDelay(pdMS_TO_TICKS(20)); // I never needed this - but doc says after bus reinit - one should wait a bit?

    if (mode == VIDEO_OUT_HDMI)  mode = VIDEO_OUT_LVDS;
    else   mode = VIDEO_OUT_HDMI;

    lcd_init();

    // audio is simply a switch 
    if (mode == VIDEO_OUT_HDMI)
    {
        gpio_set_level(GPIO_OUTPUT_PA, 0);   // speaker off
        /* enable LT8912B I2S audio input — CEC bank reg 0xB2 */
        uint8_t val = 0x01;
        esp_lcd_panel_io_tx_param(lt_io.cec_dsi, 0xB2, &val, 1);

        loadOverlayRGB(lastOverlay, HDMI_OVERLAY_WIDTH, HDMI_OVERLAY_HEIGHT);
        fireResizeEvent();
    }
    else
    {
        // switch off with
        gpio_set_level(GPIO_OUTPUT_PA, 1);   // speaker on
        uint8_t val = 0x00;
        esp_lcd_panel_io_tx_param(lt_io.cec_dsi, 0xB2, &val, 1);
        loadOverlayRGB(lastOverlay, LCD_OVERLAY_WIDTH, LCD_OVERLAY_HEIGHT);
        fireResizeEvent();
    }
}

// physical device reolution
int getScreenWidth()
{
    return (mode == VIDEO_OUT_HDMI) ? LCD_H_RES: LCD_V_RES;
}
int getScreenHeight()
{
    return (mode == VIDEO_OUT_HDMI) ? LCD_V_RES: LCD_H_RES;
}

// actual part of the screen used for display
int getDisplayWidth()
{
   if (mode == VIDEO_OUT_HDMI)
    {
        if (overlayEnabled)
    		return HDMI_IN_OVERLAY_VECX_WIDTH;
        else
            return HDMI_VECX_WIDTH;
    }
    if (overlayEnabled)
        return LCD_IN_OVERLAY_VECX_WIDTH;
    return LCD_VECX_WIDTH;
}
int getDisplayHeight()
{
   if (mode == VIDEO_OUT_HDMI)
    {
        if (overlayEnabled)
    		return HDMI_IN_OVERLAY_VECX_HEIGHT;
        else
            return HDMI_VECX_HEIGHT;
    }
    if (overlayEnabled)
        return LCD_IN_OVERLAY_VECX_HEIGHT;
    return LCD_VECX_HEIGHT;
}


// called during endframe
// input as expected by vectrex
// analog from -128 - +127
// buttons 0 active!
// one byte
// 7654 3210 (bits)
// 2222 1111 (player)
// 4321 4321 (buttons)
IRAM_ATTR void readevents()
{
    // center is default unless pressed!
	g_inputState.j0_x=0;
	g_inputState.j0_y=0;
	g_inputState.j1_x=0;
	g_inputState.j1_y=0;

    // attached keyboard (slow due to pulls!)
    // mapping as in Vide
    /* Player 1 */
 	if (isKeyDown(HID_KEY_LEFT)) g_inputState.j0_x = -128;
 	if (isKeyDown(HID_KEY_RIGHT)) g_inputState.j0_x = 127;
 	if (isKeyDown(HID_KEY_UP)) g_inputState.j0_y = 127;
 	if (isKeyDown(HID_KEY_DOWN)) g_inputState.j0_y = -128;

    /* Player 2 */
 	if (isKeyDown('h')) g_inputState.j1_x = -128;
 	if (isKeyDown('j')) g_inputState.j1_x = 127;
 	if (isKeyDown('u')) g_inputState.j1_y = 127;
 	if (isKeyDown('n')) g_inputState.j1_y = -128;

    /* Player 1 */
    if (isAsciiDown('a'))
        g_inputState.buttonState &= ~1;
    else
        g_inputState.buttonState |= 1;

    if (isAsciiDown('s'))
        g_inputState.buttonState &= ~2;
    else
        g_inputState.buttonState |= 2;

    if (isAsciiDown('d'))
        g_inputState.buttonState &= ~4;
    else
        g_inputState.buttonState |= 4;

    if (isAsciiDown('f'))
        g_inputState.buttonState &= ~8;
    else
        g_inputState.buttonState |= 8;
        
    /* Player 2 */
    if (isAsciiDown('q'))
        g_inputState.buttonState &= ~16;
    else
        g_inputState.buttonState |= 16;

    if (isAsciiDown('w'))
        g_inputState.buttonState &= ~32;
    else
        g_inputState.buttonState |= 32;

    if (isAsciiDown('e'))
        g_inputState.buttonState &= ~64;
    else
        g_inputState.buttonState |= 64;

    if (isAsciiDown('r'))
        g_inputState.buttonState &= ~128;
    else
        g_inputState.buttonState |= 128;

    ////////////////////////////////////////
    ////////////////////////////////////////
    ////////////////////////////////////////

    static volatile int modeSwitchActive=0;
    if (isAsciiDown('m'))
	{
		if (modeSwitchActive==0)
		{
            audio_set_callback(NULL, NULL);

            modeSwitchActive = 3;
            vecsimGame++;
            if (vecsimGame==MAX_VECSIM_GAME) 
            {
                setAppFPS(MAX_EMU_FPS);
                vecsimGame=0;
            }
            fireKillEvent();
            clearFramebuffers();
            frames_init();
            redraw = 3;
		}
	}
	else if (modeSwitchActive==3)
	{
		modeSwitchActive = 0;
	}

    if (isAsciiDown('y'))
	{
		if (modeSwitchActive==0)
		{
            if (mode == VIDEO_OUT_LVDS)
            {
                if (!hdmi_hpd_present(&lt_io))
                {
                    mode = VIDEO_OUT_LVDS;
                    return;
                }
            }

            modeSwitchActive = 1;
			void toggleVideoModeRequest();
			toggleVideoModeRequest();
		}
	}
	else if (modeSwitchActive==1)
	{
		modeSwitchActive = 0;
	}
	
	if (isKeyDown(HID_KEY_SPACE))
	{
		if (modeSwitchActive==0)
		{
			modeSwitchActive = 2;
			void toggleOverlayRequest();
			toggleOverlayRequest();
		}
	}
	else 
	if (modeSwitchActive==2)
	{
		modeSwitchActive = 0;
	}

   	if (isAsciiDown('b'))
	{
		brightnessLCD = brightnessLCD + 1;
		if (brightnessLCD>1022) brightnessLCD=1022;
		printf("brightnessLCD: %d\n", brightnessLCD);
	    lvds_backlight(true, brightnessLCD);
	}
   	if (isAsciiDown('v'))
	{
		brightnessLCD = brightnessLCD - 1;
		if (brightnessLCD<0) brightnessLCD=0;
		printf("toggleOverlay: %d\n", brightnessLCD);
	    lvds_backlight(true, brightnessLCD);
	}
	if (isKeyDown(HID_KEY_F11))
	{
		brightnessAdjust = brightnessAdjust - 1;
        redraw=3;
		printf("brightnessAdjust: %d\n", brightnessAdjust);
	}
	if (isKeyDown(HID_KEY_F12))
	{
		brightnessAdjust = brightnessAdjust + 1;
        redraw=3;
		printf("brightnessAdjust: %d\n", brightnessAdjust);
	}

	if (isKeyDown(HID_KEY_F1))
	{
		g_line_width = g_line_width - 1;
        redraw=3;
		if (g_line_width<0) g_line_width = 0;
		printf("g_line_width: %d\n", g_line_width);
		changeGlobalLineValues(g_line_width, g_line_glow);
	}
	if (isKeyDown(HID_KEY_F2))
	{
		g_line_width = g_line_width + 1;
        redraw=3;
		printf("g_line_width: %d\n", g_line_width);
		changeGlobalLineValues(g_line_width, g_line_glow);
	}
	if (isKeyDown(HID_KEY_F3))
	{
		g_line_glow = g_line_glow - 1;
        redraw=3;
		if (g_line_glow<0) g_line_glow = 0;
		printf("g_line_glow: %d\n", g_line_glow);
		changeGlobalLineValues(g_line_width, g_line_glow);
	}
	if (isKeyDown(HID_KEY_F4))
	{
		g_line_glow = g_line_glow + 1;
        redraw=3;
		printf("g_line_glow: %d\n", g_line_glow);
		changeGlobalLineValues(g_line_width, g_line_glow);
	}

}
