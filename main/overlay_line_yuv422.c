#include "overlay_line_yuv422.h"
#include "lodepng.h"

#include <stdlib.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"

static const char *TAG = "overlay_line_yuv422";

// ---------------------------------------------------------------------------
// Colour helpers — BT.601 full-range RGB ↔ YUV
// ---------------------------------------------------------------------------

static inline int clamp8(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

// Convert 8-bit R,G,B → Y,U,V (BT.601, full-range 0-255).
static inline void rgb_to_yuv(int r, int g, int b, int *y, int *u, int *v)
{
    *y = clamp8((77*r + 150*g + 29*b) >> 8);
    *u = clamp8(128 + ((-43*r - 85*g + 128*b) >> 8));
    *v = clamp8(128 + ((128*r - 107*g - 21*b) >> 8));
}

// ---------------------------------------------------------------------------
// Per-pixel addressing:
//   Y byte  at (py*fb_w + px)*2
//   UV byte at (py*fb_w + px)*2 + 1
//   (U when px is even, V when px is odd — stored independently per pixel)
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Pixel-level primitives
// ---------------------------------------------------------------------------

/*
 * Compute beam Y and UV contributions:
 *   beam   = lerp(white{Y=255,UV=128}, overlay{Y,UV}, shine_through/256) × alpha/256
 * White beam shining through coloured foil tints the light.
 *
 * Returns scaled_Y (luma additive, 0-255) and scaled_UV_contrib (signed, chroma offset).
 */
static IRAM_ATTR inline void beam_yuv(
        const uint8_t *overlay, int fb_w,
        int px, int py,
        int alpha, int shine_through,
        int *scaled_Y, int *scaled_UV_contrib)
{
    int beam_Y, beam_UV;
    if (overlay && shine_through > 0) {
        const uint8_t *p = overlay + (py * fb_w + px) * 2;
        int ov_Y  = p[0];
        int ov_UV = p[1];
        if (shine_through >= 255) {
            beam_Y  = ov_Y;
            beam_UV = ov_UV;
        } else {
            beam_Y  = ov_Y  + (((255 - ov_Y)  * (255 - shine_through)) >> 8);
            beam_UV = ov_UV + (((128 - ov_UV) * (255 - shine_through)) >> 8);
        }
    } else {
        beam_Y  = 255;
        beam_UV = 128;
    }
    *scaled_Y          = (beam_Y * alpha) >> 8;
    *scaled_UV_contrib = ((beam_UV - 128) * alpha) >> 8;
}

/* REPLACE — overwrite Y and UV bytes with beam colour. */
static IRAM_ATTR inline void put_replace_yuv422(
        uint8_t *fb, int fb_w, int fb_h,
        int px, int py,
        int alpha, const uint8_t *overlay, int shine_through)
{
    // if ((unsigned)px >= (unsigned)fb_w || (unsigned)py >= (unsigned)fb_h) return;
    int scaled_Y, scaled_UV_contrib;
    beam_yuv(overlay, fb_w, px, py, alpha, shine_through, &scaled_Y, &scaled_UV_contrib);
    uint8_t *dst = fb + (py * fb_w + px) * 2;
    dst[0] = (uint8_t)clamp8(scaled_Y);
    dst[1] = (uint8_t)clamp8(128 + scaled_UV_contrib);
}

/* ADDITIVE — Y saturating-add, UV chroma blend onto existing fb pixel. */
static IRAM_ATTR inline void put_additive_yuv422(
        uint8_t *fb, int fb_w, int fb_h,
        int px, int py,
        int alpha, const uint8_t *overlay, int shine_through)
{
    // if ((unsigned)px >= (unsigned)fb_w || (unsigned)py >= (unsigned)fb_h) return;
    int scaled_Y, scaled_UV_contrib;
    beam_yuv(overlay, fb_w, px, py, alpha, shine_through, &scaled_Y, &scaled_UV_contrib);
    uint8_t *dst = fb + (py * fb_w + px) * 2;
    int out_Y  = dst[0] + scaled_Y;
    if (out_Y > 255) out_Y = 255;
    int out_UV = 128 + (dst[1] - 128) + scaled_UV_contrib;
    dst[0] = (uint8_t)out_Y;
    dst[1] = (uint8_t)clamp8(out_UV);
}

// ---------------------------------------------------------------------------
// Generic rasteriser
// ---------------------------------------------------------------------------

typedef void (*put_pixel_fn_yuv422)(uint8_t *fb, int fb_w, int fb_h,
                                     int px, int py,
                                     int alpha, const uint8_t *overlay, int shine_through);

static IRAM_ATTR void rasterise_line_yuv422(
        uint8_t *fb, int fb_w, int fb_h,
        int x0, int y0, int x1, int y1,
        int brightness, int thickness,
        const uint8_t *overlay, int shine_through,
        put_pixel_fn_yuv422 put)
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
// overlay_draw_background_yuv422
// ---------------------------------------------------------------------------
void overlay_draw_background_yuv422(uint8_t *fb, int fb_w, int fb_h,
                                    const uint8_t *overlay, int brightness)
{
    int total = fb_w * fb_h;
    if (!overlay || brightness <= 0) {
        // black in YUV: Y=0, UV=128
        for (int i = 0; i < total; i++) {
            fb[i*2]   = 0;
            fb[i*2+1] = 128;
        }
        return;
    }
    if (brightness >= 255) {
        memcpy(fb, overlay, (size_t)total * 2);
        return;
    }
    for (int i = 0; i < total; i++) {
        fb[i*2]   = (uint8_t)((overlay[i*2]   * brightness) >> 8);
        fb[i*2+1] = (uint8_t)clamp8(128 + (((overlay[i*2+1] - 128) * brightness) >> 8));
    }
}

// ---------------------------------------------------------------------------
// draw_line_vectrex_yuv422  (ADDITIVE)
// ---------------------------------------------------------------------------
IRAM_ATTR void draw_line_vectrex_yuv422(
        uint8_t *fb, int fb_w, int fb_h,
        int x0, int y0, int x1, int y1,
        int brightness, int thickness,
        const uint8_t *overlay, int shine_through)
{
    if (shine_through < 0)   shine_through = 0;
    if (shine_through > 255) shine_through = 255;

    rasterise_line_yuv422(fb, fb_w, fb_h, x0, y0, x1, y1,
                          brightness, thickness, overlay, shine_through,
                          put_additive_yuv422);
}

// ---------------------------------------------------------------------------
// draw_line_overlay_yuv422  (REPLACE)
// ---------------------------------------------------------------------------
IRAM_ATTR void draw_line_overlay_yuv422(
        uint8_t *fb, int fb_w, int fb_h,
        int x0, int y0, int x1, int y1,
        int brightness, int thickness,
        const uint8_t *overlay, int shine_through)
{
    if (!overlay || shine_through <= 0) {
        draw_line_asm_yuv422(fb, fb_w, fb_h, x0, y0, x1, y1, brightness, thickness);
        return;
    }
    if (shine_through > 255) shine_through = 255;

    rasterise_line_yuv422(fb, fb_w, fb_h, x0, y0, x1, y1,
                          brightness, thickness, overlay, shine_through,
                          put_replace_yuv422);
}

// ---------------------------------------------------------------------------
// overlay_load_png_yuv422
// ---------------------------------------------------------------------------
esp_err_t overlay_load_png_yuv422(const char *path, uint8_t *fb, int fb_w, int fb_h)
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
            int yy, u, v;
            rgb_to_yuv(px[0], px[1], px[2], &yy, &u, &v);
            uint8_t *dst = fb + (y * fb_w + x) * 2;
            dst[0] = (uint8_t)yy;
            dst[1] = (uint8_t)((x & 1) ? v : u); // U for even x, V for odd x
        }
    }

    free(raw);
    return ESP_OK;
}
