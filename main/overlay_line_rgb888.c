#include "overlay_line_rgb888.h"
#include "lodepng.h"

#include <stdlib.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"

static const char *TAG = "overlay_line_rgb888";

// ---------------------------------------------------------------------------
// Colour helpers — native 8-bit, no expansion needed for RGB888
// ---------------------------------------------------------------------------

#define SADD8(a, b)  ({ int _s = (a) + (b); _s > 255 ? 255 : _s; })

static IRAM_ATTR inline void unpack888(const uint8_t *ptr, int *r, int *g, int *b)
{
    *r = ptr[0]; *g = ptr[1]; *b = ptr[2];
}

static IRAM_ATTR inline void pack888(uint8_t *ptr, int r, int g, int b)
{
    ptr[0] = (uint8_t)r; ptr[1] = (uint8_t)g; ptr[2] = (uint8_t)b;
}

static IRAM_ATTR inline int sadd8(int a, int b)
{
    int s = a + b;
    return s > 255 ? 255 : s;
}

// ---------------------------------------------------------------------------
// Pixel-level primitives
// ---------------------------------------------------------------------------

/*
 * Compute beam colour at (px,py):
 *   beam = lerp(white=255, overlay_channel, shine_through/256) × alpha/256
 * Fills *br, *bg, *bb with the scaled beam contribution (0-255).
 */
static IRAM_ATTR inline void beam_color_rgb888(
        const uint8_t *overlay, int fb_w,
        int px, int py,
        int alpha, int shine_through,
        int *br, int *bg, int *bb)
{
    if (overlay && shine_through > 0) {
        const uint8_t *p = overlay + (py * fb_w + px) * 3;
        int or_ = p[0], og = p[1], ob = p[2];
        *br = 255 + (((or_ - 255) * shine_through) >> 8);
        *bg = 255 + (((og  - 255) * shine_through) >> 8);
        *bb = 255 + (((ob  - 255) * shine_through) >> 8);
    } else {
        *br = *bg = *bb = 255;
    }
    *br = (*br * alpha) >> 8;
    *bg = (*bg * alpha) >> 8;
    *bb = (*bb * alpha) >> 8;
}

/* REPLACE — overwrite fb pixel with beam colour. */
static IRAM_ATTR inline void put_replace_rgb888(
        uint8_t *fb, int fb_w, int fb_h,
        int px, int py,
        int alpha, const uint8_t *overlay, int shine_through)
{
    // if ((unsigned)px >= (unsigned)fb_w || (unsigned)py >= (unsigned)fb_h) return;
    int br, bg, bb;
    beam_color_rgb888(overlay, fb_w, px, py, alpha, shine_through, &br, &bg, &bb);
    pack888(fb + (py * fb_w + px) * 3, br, bg, bb);
}

/* ADDITIVE — saturating-add beam colour onto existing fb pixel. */
static IRAM_ATTR inline void put_additive_rgb888(
        uint8_t *fb, int fb_w, int fb_h,
        int px, int py,
        int alpha, const uint8_t *overlay, int shine_through)
{
    // if ((unsigned)px >= (unsigned)fb_w || (unsigned)py >= (unsigned)fb_h) return;
    int br, bg, bb;
    beam_color_rgb888(overlay, fb_w, px, py, alpha, shine_through, &br, &bg, &bb);
    uint8_t *dst = fb + (py * fb_w + px) * 3;
    dst[0] = (uint8_t)sadd8(dst[0], br);
    dst[1] = (uint8_t)sadd8(dst[1], bg);
    dst[2] = (uint8_t)sadd8(dst[2], bb);
}

// ---------------------------------------------------------------------------
// Generic rasteriser
// ---------------------------------------------------------------------------

typedef void (*put_pixel_fn_rgb888)(uint8_t *fb, int fb_w, int fb_h,
                                     int px, int py,
                                     int alpha, const uint8_t *overlay, int shine_through);

static IRAM_ATTR void rasterise_line_rgb888(
        uint8_t *fb, int fb_w, int fb_h,
        int x0, int y0, int x1, int y1,
        int brightness, int thickness,
        const uint8_t *overlay, int shine_through,
        put_pixel_fn_rgb888 put)
{
    int lo = thickness > 0 ? (thickness - 1) >> 1 : 0;
    int hi = thickness > 0 ? thickness >> 1 : 0;

    int adx = x1 - x0; if (adx < 0) adx = -adx;
    int ady = y1 - y0; if (ady < 0) ady = -ady;
    int steep = (ady > adx);
    if (steep) { int t; t=x0;x0=y0;y0=t; t=x1;x1=y1;y1=t; }
    if (x0 > x1) { int t; t=x0;x0=x1;x1=t; t=y0;y0=y1;y1=t; }

    int dx = x1 - x0;
    int dy = y1 - y0;

    if (dx == 0) {
        int sx = steep ? y0 : x0;
        int sy = steep ? x0 : y0;
        for (int oy = -lo; oy <= hi; oy++)
            for (int ox = -lo; ox <= hi; ox++)
                put(fb, fb_w, fb_h, sx+ox, sy+oy, brightness, overlay, shine_through);
        return;
    }

    int32_t grad = ((int32_t)dy << 8) / dx;
    int32_t y_fp = (int32_t)y0 << 8;

    for (int x = x0; x <= x1; x++, y_fp += grad) {
        int y_int = (int)(y_fp >> 8);
        int frac  = (int)(y_fp & 0xFF);

        if (hi == 0) {
            int alpha_top = (brightness * (255 - frac)) >> 8;
            if (steep) put(fb, fb_w, fb_h, y_int, x, alpha_top, overlay, shine_through);
            else       put(fb, fb_w, fb_h, x, y_int, alpha_top, overlay, shine_through);
            if (frac) {
                int alpha_bot = (brightness * frac) >> 8;
                if (steep) put(fb, fb_w, fb_h, y_int + 1, x, alpha_bot, overlay, shine_through);
                else       put(fb, fb_w, fb_h, x, y_int + 1, alpha_bot, overlay, shine_through);
            }
        } else {
            for (int c = y_int - lo; c <= y_int + hi; c++) {
                if (steep) put(fb, fb_w, fb_h, c, x, brightness, overlay, shine_through);
                else       put(fb, fb_w, fb_h, x, c, brightness, overlay, shine_through);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// overlay_draw_background_rgb888
// ---------------------------------------------------------------------------
void overlay_draw_background_rgb888(uint8_t *fb, int fb_w, int fb_h,
                                    const uint8_t *overlay, int brightness)
{
    size_t total = (size_t)fb_w * fb_h * 3;
    if (!overlay || brightness <= 0) {
        memset(fb, 0, total);
        return;
    }
    if (brightness >= 255) {
        memcpy(fb, overlay, total);
        return;
    }
    for (size_t i = 0; i < total; i++)
        fb[i] = (uint8_t)((overlay[i] * brightness) >> 8);
}

// ---------------------------------------------------------------------------
// draw_line_vectrex_rgb888  (ADDITIVE)
// ---------------------------------------------------------------------------
IRAM_ATTR void draw_line_vectrex_rgb888(
        uint8_t *fb, int fb_w, int fb_h,
        int x0, int y0, int x1, int y1,
        int brightness, int thickness,
        const uint8_t *overlay, int shine_through)
{
    if (shine_through < 0)   shine_through = 0;
    if (shine_through > 255) shine_through = 255;

    rasterise_line_rgb888(fb, fb_w, fb_h, x0, y0, x1, y1,
                          brightness, thickness, overlay, shine_through,
                          put_additive_rgb888);
}

// ---------------------------------------------------------------------------
// draw_line_overlay_rgb888  (REPLACE)
// ---------------------------------------------------------------------------
IRAM_ATTR void draw_line_overlay_rgb888(
        uint8_t *fb, int fb_w, int fb_h,
        int x0, int y0, int x1, int y1,
        int brightness, int thickness,
        const uint8_t *overlay, int shine_through)
{
    if (!overlay || shine_through <= 0) {
        draw_line_asm_rgb888(fb, fb_w, fb_h, x0, y0, x1, y1, brightness, thickness, g_line_glow);
        return;
    }
    if (shine_through > 255) shine_through = 255;

    rasterise_line_rgb888(fb, fb_w, fb_h, x0, y0, x1, y1,
                          brightness, thickness, overlay, shine_through,
                          put_replace_rgb888);
}

// ---------------------------------------------------------------------------
// overlay_load_png_rgb888
// ---------------------------------------------------------------------------
esp_err_t overlay_load_png_rgb888(const char *path, uint8_t *fb, int fb_w, int fb_h)
{
    unsigned char *raw   = NULL;
    unsigned       width = 0, height = 0;

    unsigned err = lodepng_decode32_file(&raw, &width, &height, path);
    if (err) {
        ESP_LOGE(TAG, "PNG decode failed (%u): %s", err, lodepng_error_text(err));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "PNG %s  %u×%u → %d×%d", path, width, height, fb_w, fb_h);

    for (int y = 0; y < fb_h; y++) {
        unsigned src_y = (unsigned)((uint64_t)y * height / (unsigned)fb_h);
        for (int x = 0; x < fb_w; x++) {
            unsigned src_x = (unsigned)((uint64_t)x * width / (unsigned)fb_w);
            const unsigned char *px = raw + (src_y * width + src_x) * 4;
            uint8_t *dst = fb + (y * fb_w + x) * 3;
            dst[0] = px[0]; // R
            dst[1] = px[1]; // G
            dst[2] = px[2]; // B  (alpha ignored)
        }
    }

    free(raw);
    return ESP_OK;
}
