#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "esp_task_wdt.h"

#include "bsp/esp-bsp.h" 
#include "bsp/display.h"

#include "esp_lcd_mipi_dsi.h"      // esp_lcd_dpi_panel_* APIs
#include "esp_lcd_panel_ops.h"     // esp_lcd_panel_disp_on_off()

int vecx_init(void);

// ----------------------------------------------------
// Global line settings
// ----------------------------------------------------
DRAM_ATTR int  g_line_width      = 1;      // >= 1
DRAM_ATTR bool g_line_antialias  = false;  // false = faster, true = smoother edges

static const char *TAG = "vectrex_example";

#define MAX_LINE_BUFFER 1000
#define LCD_H_RES       BSP_LCD_H_RES
#define LCD_V_RES       BSP_LCD_V_RES
#define NUM_FB          2

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

// ----------------------------------------------------
// Emulator frame storage (independent of framebuffers)
// ----------------------------------------------------

// Lines of two frames (ping-pong) the emulator uses
DRAM_ATTR static vectrex_line_t s_frame_lines[NUM_FB][MAX_LINE_BUFFER];
DRAM_ATTR static int            s_frame_line_count[NUM_FB] = {0};
DRAM_ATTR static uint32_t       s_frame_hash[NUM_FB]       = {0};

// Current frame index that emulator is building into (0 or 1)
DRAM_ATTR static int            s_build_frame_index = 0;
DRAM_ATTR static int            s_build_line_count[NUM_FB] = {0};
DRAM_ATTR static uint32_t       s_build_hash[NUM_FB]       = {0};

// Index of last COMPLETED frame (set by emulator, read by renderer)
// -1 = none yet
DRAM_ATTR static volatile int   s_ready_frame_index = -1;

// ----------------------------------------------------
// Per-framebuffer line storage for undraw
// ----------------------------------------------------

// These describe what is *currently* drawn into each hardware framebuffer
DRAM_ATTR static vectrex_line_t s_fb_lines[NUM_FB][MAX_LINE_BUFFER];
DRAM_ATTR static int            s_fb_line_count[NUM_FB] = {0};

// ----------------------------------------------------
// Framebuffer / display
// ----------------------------------------------------
DRAM_ATTR static uint16_t            *s_framebuffers[NUM_FB] = { NULL, NULL };
DRAM_ATTR static uint16_t            *s_fb_front             = NULL;  // currently displayed
DRAM_ATTR static uint16_t            *s_fb_back              = NULL;  // draw target
DRAM_ATTR static int                  s_front_fb_index       = 0;     // index in s_framebuffers[]
DRAM_ATTR static int                  s_back_fb_index        = 1;

DRAM_ATTR static esp_lcd_panel_handle_t s_dpi_panel = NULL;
DRAM_ATTR static SemaphoreHandle_t      s_vsync_sem = NULL;

// Last rendered frame info (for "no change" detection)
DRAM_ATTR static uint32_t s_last_rendered_hash = 0;

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

// ----------------------------------------------------
// Framebuffer pixel access & line drawing
// ----------------------------------------------------
IRAM_ATTR static inline void put_pixel(int x, int y, uint16_t color)
{
    if (x < 0 || x >= LCD_H_RES || y < 0 || y >= LCD_V_RES) {
        return;
    }
    s_fb_back[y * LCD_H_RES + x] = color;
}

// brightness: 0..255, grayscale in RGB565, 0 = black
IRAM_ATTR static uint16_t brightness_to_color(uint8_t b)
{
    if (b == 0) {
        return 0x0000; // black / erase
    }
    uint8_t v  = b;
    uint16_t r = (v >> 3) & 0x1F; // 5 bits
    uint16_t g = (v >> 2) & 0x3F; // 6 bits
    uint16_t bl = (v >> 3) & 0x1F;

    return (r << 11) | (g << 5) | bl;
}

// AA helper: scale brightness by alpha (0..255) and convert to RGB565
IRAM_ATTR static inline uint16_t brightness_to_color_scaled(uint8_t b, uint8_t alpha)
{
    uint16_t scaled = (uint16_t)b * (uint16_t)alpha / 255u;
    return brightness_to_color((uint8_t)scaled);
}

IRAM_ATTR inline static void draw_line_core_no_aa(int x0, int y0, int x1, int y1, uint8_t brightness)
{
    uint16_t color = (brightness != 0) ? brightness_to_color(brightness) : 0;

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);

    bool steep = (dy > dx);
    if (steep) {
        int tmp;
        tmp = x0; x0 = y0; y0 = tmp;
        tmp = x1; x1 = y1; y1 = tmp;
        dx = abs(x1 - x0);
        dy = abs(y1 - y0);
    }

    if (x0 > x1) {
        int tmp;
        tmp = x0; x0 = x1; x1 = tmp;
        tmp = y0; y0 = y1; y1 = tmp;
    }

    int sy  = (y0 < y1) ? 1 : -1;
    int err = dx / 2;
    int y   = y0;

    int half_w = (g_line_width > 0 ? g_line_width : 1) / 2;

    for (int x = x0; x <= x1; ++x) {
        for (int o = -half_w; o <= half_w; ++o) {
            int px, py;
            if (steep) {
                px = y + o;
                py = x;
            } else {
                px = x;
                py = y + o;
            }
            put_pixel(px, py, color);
        }

        err -= dy;
        if (err < 0) {
            y   += sy;
            err += dx;
        }
    }
}

IRAM_ATTR inline static void draw_line_core_aa(int x0, int y0, int x1, int y1, uint8_t brightness)
{
    // Only implemented for width 1; fallback otherwise
    if (g_line_width > 1) {
        draw_line_core_no_aa(x0, y0, x1, y1, brightness);
        return;
    }

    int dx = x1 - x0;
    int dy = y1 - y0;

    bool steep = (abs(dy) > abs(dx));
    if (steep) {
        int tmp;
        tmp = x0; x0 = y0; y0 = tmp;
        tmp = x1; x1 = y1; y1 = tmp;
        dx = x1 - x0;
        dy = y1 - y0;
    }

    if (x0 > x1) {
        int tmp;
        tmp = x0; x0 = x1; x1 = tmp;
        tmp = y0; y0 = y1; y1 = tmp;
        dx = x1 - x0;
        dy = y1 - y0;
    }

    dx = x1 - x0;
    dy = y1 - y0;

    if (dx == 0) {
        draw_line_core_no_aa(x0, y0, x1, y1, brightness);
        return;
    }

    int sy  = (dy >= 0) ? 1 : -1;
    int ady = abs(dy);

    int32_t y_fp = ((int32_t)y0) << 8;          // Q24.8
    int32_t grad = ((int32_t)ady << 8) / dx;    // Q0.8

    if (sy < 0) {
        grad = -grad;
    }

    for (int x = x0; x <= x1; ++x) {
        int y_int   = (int)(y_fp >> 8);
        uint8_t frac = (uint8_t)(y_fp & 0xFF);

        uint8_t w1 = (uint8_t)(255 - frac);
        uint8_t w2 = frac;

        uint16_t c1 = brightness_to_color_scaled(brightness, w1);
        uint16_t c2 = brightness_to_color_scaled(brightness, w2);

        if (steep) {
            put_pixel(y_int,     x, c1);
            put_pixel(y_int + 1, x, c2);
        } else {
            put_pixel(x, y_int,     c1);
            put_pixel(x, y_int + 1, c2);
        }

        y_fp += grad;
    }
}
// Anti-aliased line with thickness > 1
IRAM_ATTR inline static void draw_line_core_aa_thick(int x0, int y0, int x1, int y1, uint8_t brightness)
{
    int dx = x1 - x0;
    int dy = y1 - y0;

    bool steep = (abs(dy) > abs(dx));
    if (steep) {
        int tmp;
        tmp = x0; x0 = y0; y0 = tmp;
        tmp = x1; x1 = y1; y1 = tmp;
        dx = x1 - x0;
        dy = y1 - y0;
    }

    if (x0 > x1) {
        int tmp;
        tmp = x0; x0 = x1; x1 = tmp;
        tmp = y0; y0 = y1; y1 = tmp;
        dx = x1 - x0;
        dy = y1 - y0;
    }

    dx = x1 - x0;
    dy = y1 - y0;

    if (dx == 0) {
        // purely vertical in transformed space → fallback to non-AA thick
        draw_line_core_no_aa(x0, y0, x1, y1, brightness);
        return;
    }

    int sy  = (dy >= 0) ? 1 : -1;
    int ady = abs(dy);

    int32_t y_fp = ((int32_t)y0) << 8;          // Q24.8
    int32_t grad = ((int32_t)ady << 8) / dx;    // Q0.8

    if (sy < 0) {
        grad = -grad;
    }

    // thickness
    int half_w = (g_line_width > 0 ? g_line_width : 1) / 2;

    for (int x = x0; x <= x1; ++x) {
        int y_int   = (int)(y_fp >> 8);
        uint8_t frac = (uint8_t)(y_fp & 0xFF);

        uint8_t w1 = (uint8_t)(255 - frac);
        uint8_t w2 = frac;

        uint16_t c1 = brightness_to_color_scaled(brightness, w1);
        uint16_t c2 = brightness_to_color_scaled(brightness, w2);

        // Draw "vertical" stripe of thickness around the center, AA on the edge
        for (int o = -half_w; o <= half_w; ++o) {
            if (steep) {
                // original coords were swapped → (y, x)
                put_pixel(y_int + o,     x, c1);
                put_pixel(y_int + o + 1, x, c2);
            } else {
                put_pixel(x, y_int + o,     c1);
                put_pixel(x, y_int + o + 1, c2);
            }
        }

        y_fp += grad;
    }
}

IRAM_ATTR static inline void drawLine_raw(int x0, int y0, int x1, int y1, uint8_t brightness)
{
    // single pixel
    // Degenerate case: a single "dot" -> must respect line thickness and AA
    if (x0 == x1 && y0 == y1) {
        int half_w = (g_line_width > 0 ? g_line_width : 1) / 2;

        // If no AA: draw a filled square of size (2*half_w+1)
        if (!g_line_antialias) {
            uint16_t color = brightness_to_color(brightness);
            for (int dy = -half_w; dy <= half_w; ++dy) {
                for (int dx = -half_w; dx <= half_w; ++dx) {
                    put_pixel(x0 + dx, y0 + dy, color);
                }
            }
            return;
        } else {
            // AA on: draw a small disc-like kernel with radial alpha
            // brightness == 0 will still work (writes black).
            int core_r = half_w;        // inner radius = full intensity
            int max_r  = half_w + 1;    // outer radius = fades to 0

            for (int dy = -max_r; dy <= max_r; ++dy) {
                for (int dx = -max_r; dx <= max_r; ++dx) {
                    float dist = sqrtf((float)(dx * dx + dy * dy));
                    if (dist > (float)max_r) {
                        continue;
                    }

                    float w = 1.0f;
                    if (dist > (float)core_r) {
                        // linear falloff between core_r and max_r
                        w = (float)max_r - dist;
                        if (w <= 0.0f) {
                            continue;
                        }
                    }

                    uint8_t alpha = (uint8_t)(w * 255.0f);
                    uint16_t color = brightness_to_color_scaled(brightness, alpha);
                    put_pixel(x0 + dx, y0 + dy, color);
                }
            }
            return;
        }
    }

    if (brightness == 0) {
        // Erase with the same kernel shape that would draw the line
        if (g_line_antialias) {
            if (g_line_width <= 1) {
                draw_line_core_aa(x0, y0, x1, y1, 0);
            } else {
                draw_line_core_aa_thick(x0, y0, x1, y1, 0);
            }
        } else {
            draw_line_core_no_aa(x0, y0, x1, y1, 0);
        }
        return;
    }

    if (g_line_antialias) {
        if (g_line_width <= 1) {
            draw_line_core_aa(x0, y0, x1, y1, brightness);
        } else {
            draw_line_core_aa_thick(x0, y0, x1, y1, brightness);
        }
    } else {
        draw_line_core_no_aa(x0, y0, x1, y1, brightness);
    }
}

// ----------------------------------------------------
// LCD init
// ----------------------------------------------------
static void lcd_init(void)
{
    ESP_LOGI(TAG, "Create VSYNC semaphore");
    s_vsync_sem = xSemaphoreCreateBinary();
    configASSERT(s_vsync_sem);

    ESP_LOGI(TAG, "Create LCD via BSP (Waveshare P4 NANO)");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_io_handle_t io_handle = NULL;

    bsp_display_config_t bsp_disp_cfg = (bsp_display_config_t){ 0 };

    ESP_ERROR_CHECK(bsp_display_new(&bsp_disp_cfg, &panel_handle, &io_handle));
    s_dpi_panel = panel_handle;

    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_dpi_panel, true));

    // Get 2 hardware framebuffers
    esp_err_t err2 = esp_lcd_dpi_panel_get_frame_buffer(
        s_dpi_panel,
        2,
        (void **)&s_framebuffers[0],
        (void **)&s_framebuffers[1]
    );

    if (err2 == ESP_OK &&
        s_framebuffers[0] != NULL &&
        s_framebuffers[1] != NULL)
    {
        s_fb_front       = s_framebuffers[0];
        s_fb_back        = s_framebuffers[1];
        s_front_fb_index = 0;
        s_back_fb_index  = 1;

        memset(s_fb_front, 0x00, LCD_H_RES * LCD_V_RES * sizeof(uint16_t));
        memset(s_fb_back,  0x00, LCD_H_RES * LCD_V_RES * sizeof(uint16_t));

        ESP_LOGI(TAG, "HW double buffer: FB0=%p FB1=%p",
                 s_framebuffers[0], s_framebuffers[1]);
    }
    else
    {
        ESP_LOGI(TAG, "ERROR Allocating double buffer!\n");
    }

    // Register VSYNC callback
    esp_lcd_dpi_panel_event_callbacks_t cbs = {
        .on_refresh_done    = lcd_on_refresh_done_cb,
        .on_color_trans_done = NULL,
    };
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_register_event_callbacks(s_dpi_panel, &cbs, NULL));

    // Backlight on
    bsp_display_backlight_on();
    ESP_LOGI(TAG, "LCD initialized");
}

// ----------------------------------------------------
// Emulator-side API
// ----------------------------------------------------

// Emulator calls this for each line in the CURRENT emulated frame
IRAM_ATTR void emu_draw_line(int x0, int y0, int x1, int y1, uint8_t brightness)
{
    int idx = s_build_frame_index;

    if (s_build_line_count[idx] < MAX_LINE_BUFFER) {
        vectrex_line_t *l = &s_frame_lines[idx][s_build_line_count[idx]++];
        l->x0 = x0;
        l->y0 = y0;
        l->x1 = x1;
        l->y1 = y1;
        l->brightness = brightness;

        // Update running hash for this in-construction frame
        s_build_hash[idx] = hash_line(s_build_hash[idx], l);
    }
    // If full, extra lines are silently dropped for this frame.
}

// Emulator calls this once when a frame is complete
IRAM_ATTR void emu_end_frame(void)
{
    static uint64_t emu_last_time  = 0;
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

    // Publish finished frame (line count + hash)
    s_frame_line_count[idx] = s_build_line_count[idx];
    s_frame_hash[idx]       = s_build_hash[idx];

    // Mark this frame as the latest ready one
    s_ready_frame_index = idx;

    // Switch to the other buffer for building next frame
    s_build_frame_index ^= 1;
    s_build_line_count[s_build_frame_index] = 0;
    s_build_hash[s_build_frame_index]       = 0;
}

// ----------------------------------------------------
// Undraw previous contents of a given framebuffer index
// ----------------------------------------------------
IRAM_ATTR static inline void undraw_previous_fb(int fb_index)
{
    int count = s_fb_line_count[fb_index];

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
IRAM_ATTR static  void emulator_task(void *arg)
{
    while (1) {
        void osint_emuloop(void);
        osint_emuloop();  // internal vecx loop; calls emu_draw_line/emu_end_frame
    }
}


// Renderer: VSYNC-driven, uses last finished emulated frame.
// If frame hash unchanged: does nothing (no undraw, no draw).
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

        int frame_idx = s_ready_frame_index;

        // No emulated frame yet
        if (frame_idx < 0) {
//            printf("Uncomplete Frame Skipped 1\n");
            continue;
        }

        uint32_t frame_hash = s_frame_hash[frame_idx];

        // If content (hash) has not changed, keep the current front buffer
        if (frame_hash == s_last_rendered_hash) {
//            printf("Uncomplete Frame Skipped 2\n");
            continue;
        }

        int line_count = s_frame_line_count[frame_idx];

        // Use current back framebuffer index
        int fb_idx = s_back_fb_index;
// todo ?
        // Undraw old lines from this backbuffer
        undraw_previous_fb(fb_idx);

        // Draw new frame lines into backbuffer and remember them
        for (int i = 0; i < line_count; ++i) {
            vectrex_line_t *src = &s_frame_lines[frame_idx][i];

            drawLine_raw(src->x0, src->y0, src->x1, src->y1, src->brightness);

            if (s_fb_line_count[fb_idx] < MAX_LINE_BUFFER) {
                s_fb_lines[fb_idx][s_fb_line_count[fb_idx]++] = *src;
            }
        }

        // Swap front/back pointers and indices
        uint16_t *tmp_fb = s_fb_front;
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

        s_last_rendered_hash = frame_hash;
    }
}
/*
// Handshake semaphores
DRAM_ATTR SemaphoreHandle_t sem_job_available = NULL;   // Core0 waits on this
DRAM_ATTR SemaphoreHandle_t sem_job_done      = NULL;   // Core1 waits on this

// Data passed from Core1 -> Core0
DRAM_ATTR volatile uint16_t core0_cycles = 0;

IRAM_ATTR void vecxSteps (long icycles);

IRAM_ATTR static void core0_cycle_worker(void *arg)
{
    for (;;)
    {
        // Wait until Core1 posts a job
        if (xSemaphoreTake(sem_job_available, portMAX_DELAY) == pdTRUE)
        {
            vecxSteps(core0_cycles);
            // Signal Core1 that we are done with this job
            xSemaphoreGive(sem_job_done);
        }
    }
}
*/    
// ----------------------------------------------------
// app_main
// ----------------------------------------------------
void app_main(void)
{
    ESP_LOGI(TAG, "Init LCD / DSI");
    lcd_init();

    ESP_LOGI(TAG, "Start vectrex tasks");

    // Task-Watchdog aus (Entwicklung)
    esp_task_wdt_deinit();

    vecx_init();
/*
    if (!sem_job_available)
        sem_job_available = xSemaphoreCreateBinary();

    if (!sem_job_done)
        sem_job_done = xSemaphoreCreateBinary();

    if (!sem_job_available || !sem_job_done)
    {
        // Failed; you can decide to fall back to single-core here
        return;
    }
    xSemaphoreGive(sem_job_done);

    xTaskCreatePinnedToCore(
        core0_cycle_worker,
        "core0_cycle_worker",
        4096,      // adjust stack size if needed
        NULL,
        6,         // priority
        NULL,
        0          // run on Core 0
    );
*/
    // Renderer task (core 0)
    xTaskCreatePinnedToCore(
        renderer_task,
        "renderer",
        8192,
        NULL,
        6,
        NULL,
        0   // core 0
    );

    // Emulator task (core 1)
    xTaskCreatePinnedToCore(
        emulator_task,
        "emulator",
        8192,
        NULL,
        7,      // Prio
        NULL,
        1   // core 1
    );
}
