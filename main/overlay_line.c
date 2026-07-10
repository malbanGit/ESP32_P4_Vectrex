#include "overlay_line.h"
#include "lodepng.h"

#include <stdlib.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"

static const char *TAG = "overlay_line";

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

// Expand a 5-bit value to 8-bit.
static IRAM_ATTR inline int expand5(int v) { return (v << 3) | (v >> 2); }
// Expand a 6-bit value to 8-bit.
static IRAM_ATTR inline int expand6(int v) { return (v << 2) | (v >> 4); }

/**
 * Write one AA pixel to the framebuffer, blending the line color (which would
 * normally be pure white scaled by `alpha`) with the overlay color at the
 * same position.
 *
 * alpha        : 0-255  (brightness × AA coverage)
 * shine_through: 0-255  (0 = white line, 255 = overlay color)
 */
static IRAM_ATTR inline void put_overlay_pixel(
        uint16_t *fb, int fb_w, int fb_h,
        int px, int py,
        int alpha,
        const uint16_t *overlay,
        int shine_through)
{
    if ((unsigned)px >= (unsigned)fb_w || (unsigned)py >= (unsigned)fb_h)
        return;

    // Sample overlay RGB565 → expand to 8-bit per channel
    uint16_t ov  = overlay[py * fb_w + px];
    int ov_r = expand5((ov >> 11) & 0x1F);
    int ov_g = expand6((ov >>  5) & 0x3F);
    int ov_b = expand5( ov        & 0x1F);

    // Lerp between white (255,255,255) and overlay color by shine_through.
    // base_channel = 255 + (ov_channel - 255) * shine_through / 256
    int base_r = 255 + (((ov_r - 255) * shine_through) >> 8);
    int base_g = 255 + (((ov_g - 255) * shine_through) >> 8);
    int base_b = 255 + (((ov_b - 255) * shine_through) >> 8);

    // Scale by AA alpha (brightness is already folded into alpha by the caller)
    int fr = (base_r * alpha) >> 8;
    int fg = (base_g * alpha) >> 8;
    int fb_c = (base_b * alpha) >> 8;

    // Pack to RGB565 and write
    fb[py * fb_w + px] = (uint16_t)(
        ((fr  >> 3) << 11) |
        ((fg  >> 2) <<  5) |
         (fb_c >> 3)
    );
}

// ---------------------------------------------------------------------------
// draw_line_overlay
// ---------------------------------------------------------------------------
IRAM_ATTR void draw_line_overlay(
        uint16_t *fb, int fb_w, int fb_h,
        int x0, int y0, int x1, int y1,
        int brightness, int thickness,
        const uint16_t *overlay, int shine_through)
{
    // No overlay or no shine → delegate to the optimised asm routine.
    if (!overlay || shine_through <= 0) {
        draw_line_asm(fb, fb_w, fb_h, x0, y0, x1, y1, brightness, thickness);
        return;
    }
    if (shine_through > 255) shine_through = 255;

    int half_w = thickness > 0 ? thickness >> 1 : 0;

    // ---- Steep detection: if |dy| > |dx|, transpose axes ----
    int adx = x1 - x0; if (adx < 0) adx = -adx;
    int ady = y1 - y0; if (ady < 0) ady = -ady;

    int steep = (ady > adx);
    if (steep) {
        int t; t = x0; x0 = y0; y0 = t;
                t = x1; x1 = y1; y1 = t;
    }

    // ---- Ensure x0 <= x1 ----
    if (x0 > x1) {
        int t; t = x0; x0 = x1; x1 = t;
                t = y0; y0 = y1; y1 = t;
    }

    int dx = x1 - x0;
    int dy = y1 - y0;

    // ---- Degenerate: single point ----
    if (dx == 0) {
        int sx = steep ? y0 : x0;
        int sy = steep ? x0 : y0;
        for (int oy = -half_w; oy <= half_w; oy++)
            for (int ox = -half_w; ox <= half_w; ox++)
                put_overlay_pixel(fb, fb_w, fb_h, sx + ox, sy + oy,
                                  brightness, overlay, shine_through);
        return;
    }

    // ---- Q8 fixed-point gradient ----
    int32_t grad = ((int32_t)dy << 8) / dx;
    int32_t y_fp = (int32_t)y0 << 8;

    for (int x = x0; x <= x1; x++, y_fp += grad) {
        int y_int = (int)(y_fp >> 8);
        int frac  = (int)(y_fp & 0xFF);

        // Top AA edge:  alpha = brightness × (255 - frac) / 256
        int alpha_top = (brightness * (255 - frac)) >> 8;
        int ytop = y_int - half_w;
        if (steep) put_overlay_pixel(fb, fb_w, fb_h, ytop, x,
                                     alpha_top, overlay, shine_through);
        else       put_overlay_pixel(fb, fb_w, fb_h, x, ytop,
                                     alpha_top, overlay, shine_through);

        // Core pixels: full brightness
        for (int c = y_int - half_w + 1; c <= y_int + half_w; c++) {
            if (steep) put_overlay_pixel(fb, fb_w, fb_h, c, x,
                                         brightness, overlay, shine_through);
            else       put_overlay_pixel(fb, fb_w, fb_h, x, c,
                                         brightness, overlay, shine_through);
        }

        // Bottom AA edge: alpha = brightness × frac / 256  (skip if frac==0)
        if (frac) {
            int alpha_bot = (brightness * frac) >> 8;
            int ybot = y_int + half_w + 1;
            if (steep) put_overlay_pixel(fb, fb_w, fb_h, ybot, x,
                                         alpha_bot, overlay, shine_through);
            else       put_overlay_pixel(fb, fb_w, fb_h, x, ybot,
                                         alpha_bot, overlay, shine_through);
        }
    }
}

// ---------------------------------------------------------------------------
// overlay_load_png
// ---------------------------------------------------------------------------
esp_err_t overlay_load_png(const char *path, uint16_t *fb, int fb_w, int fb_h)
{
    unsigned char *raw   = NULL;
    unsigned       width = 0, height = 0;

    // lodepng_decode32_file opens the file, decodes it and returns RGBA8888.
    unsigned err = lodepng_decode32_file(&raw, &width, &height, path);
    if (err) {
        ESP_LOGE(TAG, "PNG decode failed (%u): %s", err, lodepng_error_text(err));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "PNG loaded: %s  %u×%u → scaled to %d×%d", path, width, height, fb_w, fb_h);

    // Scale to target resolution using nearest-neighbour.
    for (int y = 0; y < fb_h; y++) {
        unsigned src_y = (unsigned)((uint64_t)y * height / (unsigned)fb_h);
        for (int x = 0; x < fb_w; x++) {
            unsigned src_x = (unsigned)((uint64_t)x * width / (unsigned)fb_w);
            const unsigned char *px = raw + (src_y * width + src_x) * 4;
            uint8_t r = px[0], g = px[1], b = px[2]; // alpha channel ignored
            fb[y * fb_w + x] = (uint16_t)(
                ((r >> 3) << 11) |
                ((g >> 2) <<  5) |
                 (b >> 3)
            );
        }
    }

    free(raw);
    return ESP_OK;
}
