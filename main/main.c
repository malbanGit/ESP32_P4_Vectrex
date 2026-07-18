/*
TODO: Calibration Ala Tuts

PNG file loading

overlays in YUV?

Overlays in assembler

1. RGB565 framebuffer (33% bandwidth reduction, system-wide)
Switch from RGB888 to RGB565. Every framebuffer pixel becomes 2 bytes instead of 3. That's 33% less PSRAM traffic for every read and write — draw, undraw, AND the LCD DMA transfer. Requires changing LCD init, fb allocation, and all pixel write math. But it's a clean architectural change that benefits everything, not just the overlay path.

2. Two-layer PPA compositing (eliminate overlay from CPU entirely)
ESP32-P4 has a Pixel Processing Accelerator (PPA) hardware block that does alpha blending. The idea:

Emulator renders Vectrex lines on a pure-black fb — no overlay involvement at all
PPA blends the black fb + overlay → final display fb in hardware, asynchronously
Core 1 emulator never touches overlay PSRAM at all during rendering
Draw/undraw becomes trivial: write colored pixels or black. No palette lookup, no alpha, no bbox guards. The CPU completely exits the compositing business.

3. Frame-diff line caching (near-zero cost for static screens)
Many Vectrex frames are identical or nearly identical to the previous frame. Cache the line list (x0, y0, x1, y1, brightness, thickness) from last frame. If a line is unchanged, skip its draw+undraw entirely. For title screens, menus, or slow-moving games this could eliminate 80%+ of render work.

4. Single frame-start reset instead of per-line undraw
Instead of N undraws per frame, track the combined bounding box of all lines drawn last frame (6 integers per buffer in DRAM). At frame start: one DMA transfer from overlay_bg covering that combined region resets everything. Then draw all lines fresh. Replaces N×undraw with 1×DMA. DMA runs without CPU, on bus cycles the emulator doesn't use.


To start the current version connect USB POW/UART to computer (directly)
Connect USB (mid /down) to keyboard - the keyboard can be used to control the vectrex
Start
Should connect to COM 4, flash and play.


/C:\Users\salom\ESP_Projects\ESP32_P4_Vectrex\main\vecx\e6809.c
Sound


  BUG
  FCYCLES_INIT = 50000
  should be 30000 for one round


  however 50000 displays seemingly only in one frame -> flickering
  that is also a bug and I keep it at 50000 till I find THAT bug
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
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

i2c_master_bus_handle_t i2c_bus = NULL;
i2c_master_bus_config_t i2c_bus_cfg = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = BOARD_I2C_PORT,
    .sda_io_num = BOARD_I2C_SDA,
    .scl_io_num = BOARD_I2C_SCL,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};


//#include "esp_lcd_mipi_dsi.h"      // esp_lcd_dpi_panel_* APIs
//#include "esp_lcd_panel_ops.h"     // esp_lcd_panel_disp_on_off()

int vecx_init(void);

// ----------------------------------------------------
// Global line settings
// ----------------------------------------------------
DRAM_ATTR int  g_line_width = 1;      // >= 1
DRAM_ATTR int  g_line_glow  = 2;
DRAM_ATTR int  brightnessAdjust = 0;


static const char *TAG = "vectrex_example";

#define MAX_LINE_BUFFER   1000
//#define LCD_H_RES         BSP_LCD_H_RES
//#define LCD_V_RES         BSP_LCD_V_RES
#define NUM_FB            2        // Hardware-Framebuffer
#define NUM_FRAME_SLOTS   3        // Logische Emu-Frames (Linien)
DRAM_ATTR static int LCD_H_RES = 1280;
DRAM_ATTR static int LCD_V_RES = 720;
//hdmi 1280x720
//vdsl 480x800
static uint8_t *s_overlay    = NULL;
uint8_t        *s_overlay_bg = NULL;   /* precomputed RGB888 overlay-over-black */

/* ── Palettised overlay (draw optimisation) ───────────────────────────────── *
 * s_overlay_pal: 1 byte/pixel covering the active image region only (PSRAM).
 *   bit 7 = 1 → drawable pixel; palette index in bits 6..0
 *   bit 7 = 0 → skip (transparent or opaque after alphaAdjust)
 * s_overlay_palette: BGR table in DRAM — stays in L1 cache permanently.
 * Built once at load time with the then-current alphaAdjust.              */
uint8_t       *s_overlay_pal = NULL;
DRAM_ATTR uint8_t  s_overlay_palette[128][3];
DRAM_ATTR int      s_overlay_pal_n   = 0;
DRAM_ATTR uint8_t  s_overlay_alpha_val = 40;  /* representative raw alpha */
DRAM_ATTR int      s_ov_off_x = 0;             /* active region x offset   */
DRAM_ATTR int      s_ov_off_y = 0;             /* active region y offset   */
DRAM_ATTR int      s_ov_w     = 0;             /* active region width      */
DRAM_ATTR int      s_ov_h     = 0;             /* active region height     */

// ----------------------------------------------------
// Types
// ----------------------------------------------------
typedef struct {
    int x0;
    int y0;
    int x1;
    int y1;
    uint8_t brightness;
} vectrex_line_t;
typedef struct { int bx0, by0, bx1, by1; } line_bbox_t;

static inline line_bbox_t line_compute_bbox(const vectrex_line_t *l)
{
    int glow = g_line_glow < 0 ? 0 : g_line_glow;
    int R    = (g_line_width >> 1) + glow;
    int Rb   = R > 0 ? R - 1 : 0;
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
// Frame-Slot-Status für die Logik-Frames (Linien)
typedef enum {
    FRAME_FREE = 0,      // vom Emulator frei nutzbar
    FRAME_BUILDING,      // Emulator schreibt Linien
    FRAME_READY,         // fertig und wartet auf Renderer
    FRAME_RENDERING      // Renderer liest/zeichnet
} frame_state_t;

// Logischer Frame-Slot: Liste von Linien + Hash + Status
typedef struct {
    vectrex_line_t      lines[MAX_LINE_BUFFER];
    int                 line_count;
    uint32_t            hash;
    volatile frame_state_t state;
} frame_slot_t;

// ----------------------------------------------------
// Emulator frame storage (independent of framebuffers)
// ----------------------------------------------------

// Drei logische Frames als Ringpuffer
DRAM_ATTR static frame_slot_t s_frames[NUM_FRAME_SLOTS];
DRAM_ATTR static int          s_build_frame_index = 0;   // Slot, in den der Emulator gerade schreibt

// ----------------------------------------------------
// Per-framebuffer line storage for undraw (Hardware-FBs)
// ----------------------------------------------------

/* Task stacks pinned to internal SRAM so function calls never touch PSRAM. */
#define EMU_STACK_SIZE  8192
#define REND_STACK_SIZE 8192
static DRAM_ATTR StackType_t  s_emu_stack[EMU_STACK_SIZE];
static DRAM_ATTR StaticTask_t s_emu_tcb;
static DRAM_ATTR StackType_t  s_rend_stack[REND_STACK_SIZE];
static DRAM_ATTR StaticTask_t s_rend_tcb;

// Diese beschreiben, was aktuell in jedem Hardware-Framebuffer gezeichnet ist
DRAM_ATTR static vectrex_line_t s_fb_lines[NUM_FB][MAX_LINE_BUFFER];
DRAM_ATTR static int            s_fb_line_count[NUM_FB] = {0};
DRAM_ATTR static uint8_t     s_diff_old_matched[MAX_LINE_BUFFER];
DRAM_ATTR static uint8_t     s_diff_new_matched[MAX_LINE_BUFFER];
DRAM_ATTR static uint8_t     s_diff_damaged[MAX_LINE_BUFFER];
DRAM_ATTR static line_bbox_t s_diff_dirty_bboxes[MAX_LINE_BUFFER];
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



typedef enum { VIDEO_OUT_HDMI, VIDEO_OUT_LVDS } video_out_t;
#define VIDEO_OUT_SELECTED VIDEO_OUT_HDMI

// Diagnostic: 1 = output the P4 DPI's built-in vertical color bars instead of our
// frame buffer. Rules out a black/buggy frame buffer and tests the exact
// P4 DPI -> DSI -> bridge -> LVDS data path with known-good pixels. 0 = normal.
#define VIDEO_DPI_TEST_PATTERN  1


DRAM_ATTR static esp_lcd_panel_handle_t s_dpi_panel = NULL;
DRAM_ATTR static SemaphoreHandle_t      s_vsync_sem = NULL;
DRAM_ATTR video_out_t mode = VIDEO_OUT_SELECTED;          // default at boot


// ----------------------------------------------------
// Hash helpers for change detection
// ----------------------------------------------------
IRAM_ATTR static inline uint32_t hash_step(uint32_t h, uint32_t v)
{
    h ^= v;
    h *= 16777619u;
    return h;
}

IRAM_ATTR static inline uint32_t hash_line(uint32_t h, const vectrex_line_t *l)
{
    h = hash_step(h, (uint32_t)l->x0);
    h = hash_step(h, (uint32_t)l->y0);
    h = hash_step(h, (uint32_t)l->x1);
    h = hash_step(h, (uint32_t)l->y1);
    h = hash_step(h, (uint32_t)l->brightness);
    return h;
}

// ----------------------------------------------------
// VSYNC callback
// ----------------------------------------------------
IRAM_ATTR static bool lcd_on_refresh_done_cb(esp_lcd_panel_handle_t panel,
                                             esp_lcd_dpi_panel_event_data_t *edata,
                                             void *user_ctx)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(s_vsync_sem, &xHigherPriorityTaskWoken);
    return (xHigherPriorityTaskWoken == pdTRUE);
}

   void draw_line_asm_rgb888(uint8_t *fb, int fw, int fh,
                              int x0, int y0, int x1, int y1,
                              int brightness, int thickness);
   void undraw_line_asm_rgb888(uint8_t *fb, int fw, int fh,
                                int x0, int y0, int x1, int y1,
                                int brightness, int thickness);
   void draw_line_asm_yuv422(uint8_t *fb, int fw, int fh,
                              int x0, int y0, int x1, int y1,
                              int brightness, int thickness);
   void undraw_line_asm_yuv422(uint8_t *fb, int fw, int fh,
                                int x0, int y0, int x1, int y1,
                                int brightness, int thickness);

#include "draw_line_dist_rgb888_overlay.h"
IRAM_ATTR static inline void drawLine_raw(int x0, int y0, int x1, int y1, uint8_t brightness)
{
#if VIDEO_FB_YUV422 == 1
    if (brightness == 0) 
    {
        undraw_line_asm_yuv422(s_fb_back, LCD_H_RES, LCD_V_RES, x0, y0, x1, y1, brightness, g_line_width);
    }
    else
    {
        if (x0==x1 && y0==y1) 
        {
            int b = brightness*4+brightnessAdjust;
            if (b>0)
                draw_line_asm_yuv422(s_fb_back, LCD_H_RES, LCD_V_RES, x0, y0, x1, y1, b, g_line_width);
            return;
        }
        int b = brightness*4/3+brightnessAdjust;
        if (b>0)
            draw_line_asm_yuv422(s_fb_back, LCD_H_RES, LCD_V_RES, x0, y0, x1, y1, b, g_line_width);

    }
    #else
    if (brightness == 0) 
    {
        if (s_overlay == NULL)
        {
            undraw_line_asm_rgb888(s_fb_back, LCD_H_RES, LCD_V_RES, x0, y0, x1, y1, brightness, g_line_width);
        }
        else        
        {
            undraw_line_rgb888_overlay(s_fb_back, LCD_H_RES, LCD_V_RES, x0, y0, x1, y1, brightness, g_line_width, s_overlay);

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
                    draw_line_asm_rgb888(s_fb_back, LCD_H_RES, LCD_V_RES, x0, y0, x1, y1, b, g_line_width);
                }
                else
                {
                    draw_line_rgb888_overlay(s_fb_back, LCD_H_RES, LCD_V_RES, x0, y0, x1, y1, b, g_line_width, s_overlay);
                }
            }
            return;
        }
        int b = brightness*4/3+brightnessAdjust;
        if (b>0)
        {
            if (s_overlay == NULL)
            {
                draw_line_asm_rgb888(s_fb_back, LCD_H_RES, LCD_V_RES, x0, y0, x1, y1, b, g_line_width);
            }
            else
            {
                draw_line_rgb888_overlay(s_fb_back, LCD_H_RES, LCD_V_RES, x0, y0, x1, y1, b, g_line_width, s_overlay);
            }
        }
    }
    #endif
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

static const char *mode_name(video_out_t m) { return m == VIDEO_OUT_LVDS ? "LVDS" : "HDMI"; }

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
static esp_err_t video_start(video_out_t mode, bool yuv422,
                             const esp_lcd_panel_lt8912b_io_t *io, esp_lcd_dsi_bus_handle_t dsi,
                             esp_lcd_panel_handle_t *panel, int *w, int *h)
{
    static const char *TAG = "video";
    esp_err_t err = (mode == VIDEO_OUT_LVDS)
                    ? lvds_start(io, dsi, yuv422, panel, w, h)
                    : hdmi_start(io, dsi, yuv422, panel, w, h);
    if (err != ESP_OK) return err;
    if (mode == VIDEO_OUT_HDMI) lvds_backlight(false);   // LVDS panel unused -> backlight off


//    ESP_RETURN_ON_ERROR(esp_lcd_dpi_panel_get_frame_buffer(*panel,2,(void **)&s_framebuffers[0],(void **)&s_framebuffers[1]), TAG, "Get double framebuffer");

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

//    ESP_RETURN_ON_ERROR(mire_render(s_fb_front, *w, *h, yuv422, mode_name(mode)), TAG, "mire");
//    ESP_RETURN_ON_ERROR(mire_render(s_fb_back, *w, *h, yuv422, mode_name(mode)), TAG, "mire");
//    ESP_RETURN_ON_ERROR(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, *w, *h, s_fb_front), TAG, "draw");
//    ESP_RETURN_ON_ERROR(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, *w, *h, s_fb_back), TAG, "draw");
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


static void lcd_init(void)
{
    ESP_LOGI(TAG, "Create VSYNC semaphore");
    s_vsync_sem = xSemaphoreCreateBinary();
    configASSERT(s_vsync_sem);

//    ESP_LOGI(TAG, "Create LCD via BSP (Waveshare P4 NANO)");

    // 2) The three LT8912B I2C register banks (main / cec-dsi / avi).
    esp_lcd_panel_io_i2c_config_t io_main_cfg = LT8912B_IO_CFG(LT8912B_I2C_HZ, LT8912B_IO_I2C_MAIN_ADDRESS);
    esp_lcd_panel_io_i2c_config_t io_cec_cfg  = LT8912B_IO_CFG(LT8912B_I2C_HZ, LT8912B_IO_I2C_CEC_ADDRESS);
    esp_lcd_panel_io_i2c_config_t io_avi_cfg  = LT8912B_IO_CFG(LT8912B_I2C_HZ, LT8912B_IO_I2C_AVI_ADDRESS);
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_main_cfg, &lt_io.main));
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_cec_cfg,  &lt_io.cec_dsi));
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_avi_cfg,  &lt_io.avi));

    // 3) MIPI-DSI bus (2 lanes, 1000 Mbps/lane).
    esp_lcd_dsi_bus_handle_t dsi_bus = NULL;
    esp_lcd_dsi_bus_config_t dsi_bus_cfg = LT8912B_PANEL_BUS_DSI_2CH_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&dsi_bus_cfg, &dsi_bus));



    const bool yuv422 = VIDEO_FB_YUV422; // 
    mode = VIDEO_OUT_SELECTED;          // default at boot
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

    // Backlight on
//    bsp_display_backlight_on();
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

// Emulator calls this for each line in the CURRENT emulated frame
IRAM_ATTR void emu_draw_line(int x0, int y0, int x1, int y1, uint8_t brightness)
{
    int idx = s_build_frame_index;
    frame_slot_t *fs = &s_frames[idx];

    if (fs->state != FRAME_BUILDING) {
        // Slot ist nicht im BUILDING-Zustand – dann nichts schreiben
        return;
    }

    if (fs->line_count < MAX_LINE_BUFFER) {
        vectrex_line_t *l = &fs->lines[fs->line_count++];
        l->x0 = x0;
        l->y0 = y0;
        l->x1 = x1;
        l->y1 = y1;
        l->brightness = brightness;

        // Hash für diesen Frame laufend aktualisieren
//        fs->hash = hash_line(fs->hash, l);
    }
    else {
        // Buffer voll – zusätzliche Linien werden verworfen
        // Optional: Debug-Ausgabe
        // printf("Emulation Exceeded line Count!!! (%d)\n", fs->line_count);
    }
}

// Emulator calls this once when a frame is complete
IRAM_ATTR void emu_end_frame(void)
{
    static uint64_t emu_last_time   = 0;
    static int      emu_frame_count = 0;

    // EMU FPS
    uint64_t now = esp_timer_get_time();
    emu_frame_count++;
    if (now - emu_last_time >= 1000000) {
        printf("EMU FPS = %d\n", emu_frame_count);
        emu_frame_count = 0;
        emu_last_time   = now;
    }

    int idx = s_build_frame_index;
    frame_slot_t *cur = &s_frames[idx];

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
void osint_emuloop(int cycles);
IRAM_ATTR static void emulator_task(void *arg)
{
    while (1) 
    {

        uint64_t start = esp_timer_get_time();


        osint_emuloop(30000);  // internal vecx loop; calls emu_draw_line/emu_end_frame
        uint64_t now = esp_timer_get_time();
#define MAX_EMU_FPS 100
        // 30000 cycles == 1/50 second
        if (now - start <= 1000000/MAX_EMU_FPS) 
        {
            int delay = (1000000/MAX_EMU_FPS) - (now - start);
            esp_rom_delay_us(delay);  
        }
    }
}

// Renderer: VSYNC-driven, uses last finished emulated frame.
// If frame hash unchanged: could skip (auskommentiert).
IRAM_ATTR static void renderer_task(void *arg)
{
    // Wait once for first VSYNC
    xSemaphoreTake(s_vsync_sem, portMAX_DELAY);

    uint64_t fps_last_time = esp_timer_get_time();
    int fps_frame_count    = 0;

    for (;;)
    {
        // DISPLAY FPS = number of actually changed frames drawn per second
        fps_frame_count++;
        uint64_t now = esp_timer_get_time();
        if (now - fps_last_time >= 1000000) {
            printf("DISPLAY FPS = %d\n", fps_frame_count);
            fps_frame_count = 0;
            fps_last_time   = now;
        }

        // Wait for next VSYNC
        xSemaphoreTake(s_vsync_sem, portMAX_DELAY);
//printf (".");

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
  //          printf("Render skipped, no finished frame found.\n");
            continue;
        }

//printf ("Front index: %i, delete Index: %i\n", s_front_fb_index, frame_idx);

        frame_slot_t *fs = &s_frames[frame_idx]; // frame_idx = 0-2
        fs->state = FRAME_RENDERING;

        // Use current back framebuffer index
        int fb_idx = s_back_fb_index;           // the current backbuffer - we can write to - it is not displayed!
//        uint64_t t0 = esp_timer_get_time();

#if 0


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
#else            

    // --- Stable/dirty/redraw frame-diff ---
        unsigned int old_count = s_fb_line_count[fb_idx];
        vectrex_line_t *old_lines = s_fb_lines[fb_idx];

        // Step 1: match old lines to new lines (exact match)
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
        // Step 4: undraw dirty old lines
        for (int i = 0; i < old_count; i++) {
            if (!s_diff_old_matched[i]) {
                vectrex_line_t *l = &old_lines[i];
                drawLine_raw(l->x0, l->y0, l->x1, l->y1, 0);
            }
        }
/*
        // Step 5: redraw damaged stable lines (before rebuilding old_lines buffer)
        for (int i = 0; i < old_count; i++) {
            if (s_diff_old_matched[i] && s_diff_damaged[i]) {
                vectrex_line_t *l = &old_lines[i];
                drawLine_raw(l->x0, l->y0, l->x1, l->y1, 0);
                drawLine_raw(l->x0, l->y0, l->x1, l->y1, l->brightness);
            }
        }
        */
        // Step 5a: undraw ALL damaged stable lines
        for (int i = 0; i < old_count; i++) {
            if (s_diff_old_matched[i] && s_diff_damaged[i]) {
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
#endif
/*
        t1 = esp_timer_get_time();
        uint64_t draw_asm = t1 - t0;

        printf("draw_c: %llu us | draw_asm: %llu us\n", draw_c, draw_asm);
*/



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


IRAM_ATTR void e8910_callback(void *userdata, int16_t *stream, int length);// from e8910.c

#include "audio.i"
static void audio_music_task(void *arg)
{
    while (1)
    {
        if (!s_audio_cb) {
            /* Noch kein Codec oder kein Callback gesetzt: kurz warten */
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // e8910_callback(NULL, uint16_t *stream, int length)
        s_audio_cb(NULL, audioBuffer, AUDIO_FRAME_SAMPLES);

        const size_t bytes = AUDIO_FRAME_SAMPLES * sizeof(int16_t);

        // this is a blocking (task freeing block) call
        esp_err_t ret = esp_codec_dev_write(s_codec_dev,
                                            (void *)audioBuffer,
                                            bytes);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "audio_music_task: esp_codec_dev_write failed: 0x%x", ret);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

#include "sdcard.i"
#include "usb.i"

#define INI_FILE_PATH   "/sdcard/ESP.INI"
#define MAX_ROM_NAME    128
#define MAX_CART_SIZE   32768*2 // for now! (256*1024)

char cartName[MAX_ROM_NAME];
DRAM_ATTR unsigned char cartData[MAX_CART_SIZE];
long cartSize = 0;
#include "file.i"


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
void drawOverlayPal(uint8_t *dest)
{
    if (s_overlay_pal == NULL) return;

    for (int y = 0; y < s_ov_h; y++) 
    {
        const uint8_t *row_pal = s_overlay_pal + (size_t)y * s_ov_w;
        uint8_t       *row_dst = dest + ((size_t)(s_ov_off_y + y) * LCD_H_RES + s_ov_off_x) * 3;
        for (int x = 0; x < s_ov_w; x++, row_dst += 3) 
        {
            uint8_t pidx = row_pal[x];
            const uint8_t *c = s_overlay_palette[pidx & 0x7F];
            if (!(pidx & 0x80))
            {
                row_dst[0] = c[0];
                row_dst[1] = c[1];
                row_dst[2] = c[2];
            }
            else
            {
                row_dst[0] = (uint8_t)((c[0] * s_overlay_alpha_val) >> 8);
                row_dst[1] = (uint8_t)((c[1] * s_overlay_alpha_val) >> 8);
                row_dst[2] = (uint8_t)((c[2] * s_overlay_alpha_val) >> 8);
            }
        }
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

// returns pointer 
/* Load a PNG (with alpha), scale it to (img_w x img_h), and centre it on a
 * full-screen BGRA overlay buffer (LCD_H_RES x LCD_V_RES, 4 bytes/pixel).
 * Surrounding area is filled with transparent black (alpha=0).
 * Pass img_w=0 / img_h=0 to stretch to full screen.                      */
esp_err_t loadOverlayRGB(char *name, int img_w, int img_h)
{
    /* clamp / default to full screen */
    if (img_w <= 0 || img_w > LCD_H_RES) img_w = LCD_H_RES;
    if (img_h <= 0 || img_h > LCD_V_RES) img_h = LCD_V_RES;

    /* free previous overlay buffers */
    if (s_overlay != NULL)    { heap_caps_free(s_overlay);    s_overlay    = NULL; }
    if (s_overlay_bg != NULL) { heap_caps_free(s_overlay_bg); s_overlay_bg = NULL; }
    if (s_overlay_pal != NULL){ heap_caps_free(s_overlay_pal);s_overlay_pal= NULL; }
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

    /* decode + scale PNG into a temporary BGRA buffer (img_w x img_h) */
    uint8_t *scaled = heap_caps_malloc((size_t)img_w * img_h * 4, MALLOC_CAP_SPIRAM);
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

        /* Pass 1: build palette from drawable pixels. */
        for (int y = 0; y < img_h; y++) 
        {
            const uint8_t *src = s_overlay + ((size_t)(off_y + y) * LCD_H_RES + off_x) * 4;
            for (int x = 0; x < img_w; x++, src += 4) 
            {
                uint8_t b = src[0], g = src[1], r = src[2];//, a = src[3];

                /* find nearest existing palette entry */
                int best;
                int best_d = 0x7FFFFFFF;
                for (int i = 0; i < s_overlay_pal_n; i++) 
                {
                    int db = (int)b - s_overlay_palette[i][0];
                    int dg = (int)g - s_overlay_palette[i][1];
                    int dr = (int)r - s_overlay_palette[i][2];
                    int d  = db*db + dg*dg + dr*dr;
                    if (d < best_d) { best = i; best_d = d; }
                    if (d == 0) break;
                }

                if (best_d != 0 && s_overlay_pal_n < 127) 
                {
                    best = s_overlay_pal_n;
                    s_overlay_palette[s_overlay_pal_n][0] = b;
                    s_overlay_palette[s_overlay_pal_n][1] = g;
                    s_overlay_palette[s_overlay_pal_n][2] = r;
                    s_overlay_pal_n++;
                }
            }
        }

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
        ESP_LOGI(TAG, "overlay pal: %d colours, alpha_val=%d", s_overlay_pal_n, s_overlay_alpha_val);
    }
    
    drawOverlayPal(s_fb_front);
    drawOverlayPal(s_fb_back);


    ESP_LOGI(TAG, "overlay: %s scaled to %dx%d, centred on %dx%d screen (BGRA)",
             name, img_w, img_h, LCD_H_RES, LCD_V_RES);
    return ESP_OK;
}



// ----------------------------------------------------
// app_main
// ----------------------------------------------------
void app_main(void)
{
    esp_log_level_set("lcd.dsi.dpi", ESP_LOG_DEBUG);   // show the actual DPI pixel clock achieved

esp_log_level_set("lcd_panel.dpi", ESP_LOG_DEBUG);
// evtl. auch:
esp_log_level_set("mipi_dsi", ESP_LOG_DEBUG);
esp_log_level_set("*", ESP_LOG_DEBUG);  

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
            printf("ROM Name: %s\n", cartName);
//            cartSize = load_rom_file(cartName, cartData, sizeof(cartData));
            cartSize = load_rom_file("SWEEP.BIN", cartData, sizeof(cartData));
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
/*
    // 4) VIDEO_MODE switch: momentary tactile button, reads 1 when released.
    //    Used as a toggle. Default output at boot = LVDS.
    gpio_config_t btn = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << BOARD_PIN_VIDEO_MODE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&btn));

*/

    if (usb_keyboard_init() != ESP_OK)
    {
        printf("Something went wrong with the keyboard init - did you connect a keyboard?");
    }

    ESP_LOGI(TAG, "Init LCD / DSI");
    lcd_init();


    ESP_LOGI(TAG, "VIDEO_FB_BPP: %i", VIDEO_FB_BPP);

    ESP_LOGI(TAG, "LCD_H_RES: %i, LCD_V_RES: %i", LCD_H_RES, LCD_V_RES);

// Vecx
extern int SCREEN_WIDTH;
extern int SCREEN_HEIGHT;
    SCREEN_WIDTH = LCD_H_RES;
    SCREEN_HEIGHT = LCD_V_RES;


    // Logische Frame-Slots initialisieren
    frames_init();



 loadOverlayRGB("/sdcard/sweep.png", 564, 720);



    ESP_LOGI(TAG, "Start vectrex tasks");

#ifdef CONFIG_ESP_TASK_WDT_EN
    // Task-Watchdog aus (Entwicklung)
    esp_task_wdt_deinit();
#endif

    ESP_ERROR_CHECK(audio_init());
//    audio_set_callback(e8910_callback, NULL);

    printf("Audio init done\n");

    xTaskCreatePinnedToCore(
        audio_music_task,        // Task-Funktion
        "audio_music_task",      // Name
        4096,                    // Stackgröße (Wort, nicht Byte)
        NULL,                    // Parameter
        2,                       // Priorität
        &s_audio_task_handle,    // Handle
        0                        // Core 0
    );

    vecx_init();

    // Emulator task (core 1) — static stack in internal SRAM
    xTaskCreateStaticPinnedToCore(
        emulator_task,
        "emulator",
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

    printf("Free internal DRAM: %d bytes\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    printf("Largest free DRAM block: %d bytes\n", heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));    
}
