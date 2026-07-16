#include "overlay_fast_yuv422.h"

#include <string.h>
#include <stdlib.h>

#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "overlay_fast_yuv422";

static inline int clamp8(int v) { return v < 0 ? 0 : v > 255 ? 255 : v; }

// ---------------------------------------------------------------------------
// Context management
// ---------------------------------------------------------------------------

esp_err_t overlay_fast_init_yuv422(overlay_ctx_yuv422_t *ctx,
                                    const uint8_t *overlay, int fb_w, int fb_h,
                                    int bg_brightness)
{
    ctx->overlay = overlay;
    ctx->fb_w    = fb_w;
    ctx->fb_h    = fb_h;

    size_t bytes = (size_t)fb_w * fb_h * 2;
    ctx->dimmed  = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);
    if (!ctx->dimmed) {
        ESP_LOGE(TAG, "Failed to allocate %zu bytes for dimmed buffer", bytes);
        return ESP_ERR_NO_MEM;
    }

    overlay_fast_set_brightness_yuv422(ctx, bg_brightness);
    return ESP_OK;
}

void overlay_fast_free_yuv422(overlay_ctx_yuv422_t *ctx)
{
    if (ctx->dimmed) {
        free(ctx->dimmed);
        ctx->dimmed = NULL;
    }
}

void overlay_fast_set_brightness_yuv422(overlay_ctx_yuv422_t *ctx, int bg_brightness)
{
    if (bg_brightness < 0)   bg_brightness = 0;
    if (bg_brightness > 255) bg_brightness = 255;

    int total = ctx->fb_w * ctx->fb_h;
    const uint8_t *src = ctx->overlay;
    uint8_t       *dst = ctx->dimmed;

    if (bg_brightness == 0) {
        for (int i = 0; i < total; i++) { dst[i*2] = 0; dst[i*2+1] = 128; }
        return;
    }
    if (bg_brightness == 255) {
        memcpy(dst, src, (size_t)total * 2);
        return;
    }

    for (int i = 0; i < total; i++) {
        dst[i*2]   = (uint8_t)((src[i*2]   * bg_brightness) >> 8);
        dst[i*2+1] = (uint8_t)clamp8(128 + (((src[i*2+1] - 128) * bg_brightness) >> 8));
    }
}

// ---------------------------------------------------------------------------
// Pixel primitives (IRAM)
// ---------------------------------------------------------------------------

static IRAM_ATTR inline void fast_restore_yuv422(
        const uint8_t * restrict dimmed, uint8_t * restrict fb, int offset)
{
    // Copy both Y and UV bytes atomically via 16-bit load/store (aligned: offset*2)
    const uint8_t *s = dimmed + offset * 2;
    uint8_t       *d = fb    + offset * 2;
    d[0] = s[0]; d[1] = s[1];
}

static IRAM_ATTR inline void fast_draw_pixel_yuv422(
        const uint8_t * restrict dimmed,
        const uint8_t * restrict overlay,
        uint8_t       * restrict fb,
        int offset, int alpha, int shine_through)
{
    int bg_Y  = dimmed[offset*2];
    int bg_UV = dimmed[offset*2+1];

    int beam_Y, beam_UV;
    if (shine_through <= 0) {
        beam_Y  = 255;
        beam_UV = 128;
    } else {
        int ov_Y  = overlay[offset*2];
        int ov_UV = overlay[offset*2+1];
        if (shine_through >= 255) {
            beam_Y  = ov_Y;
            beam_UV = ov_UV;
        } else {
            beam_Y  = ov_Y  + (((255 - ov_Y)  * (255 - shine_through)) >> 8);
            beam_UV = ov_UV + (((128 - ov_UV) * (255 - shine_through)) >> 8);
        }
    }

    int contrib_Y  = (beam_Y * alpha) >> 8;
    int contrib_UV = ((beam_UV - 128) * alpha) >> 8;  // signed

    int out_Y  = bg_Y + contrib_Y;
    if (out_Y > 255) out_Y = 255;

    int out_UV = 128 + (bg_UV - 128) + contrib_UV;

    fb[offset*2]   = (uint8_t)out_Y;
    fb[offset*2+1] = (uint8_t)clamp8(out_UV);
}

// ---------------------------------------------------------------------------
// Shared rasteriser
// ---------------------------------------------------------------------------

typedef enum { MODE_ERASE = 0, MODE_DRAW = 1 } raster_mode_yuv422_t;

static IRAM_ATTR void rasterise_yuv422(
        const overlay_ctx_yuv422_t * restrict ctx,
        uint8_t * restrict fb,
        int x0, int y0, int x1, int y1,
        int brightness, int thickness, int shine_through,
        raster_mode_yuv422_t mode)
{
    const uint8_t *dimmed  = ctx->dimmed;
    const uint8_t *overlay = ctx->overlay;
    int fw = ctx->fb_w;
    int fh = ctx->fb_h;
    int lo = thickness > 0 ? (thickness - 1) >> 1 : 0;
    int hi = thickness > 0 ? thickness >> 1 : 0;

    int adx = x1 - x0; if (adx < 0) adx = -adx;
    int ady = y1 - y0; if (ady < 0) ady = -ady;
    int steep = (ady > adx);
    if (steep) { int t; t=x0;x0=y0;y0=t; t=x1;x1=y1;y1=t; }
    if (x0 > x1) { int t; t=x0;x0=x1;x1=t; t=y0;y0=y1;y1=t; }

    int dx = x1 - x0;
    int dy = y1 - y0;

#define DISPATCHYUV(px, py, alpha)                                     \
    do {                                                               \
        /* if ((unsigned)(px) < (unsigned)fw &&                     */ \
        /*     (unsigned)(py) < (unsigned)fh) {                     */ \
            int _off = (py) * fw + (px);                              \
            if (mode == MODE_ERASE)                                    \
                fast_restore_yuv422(dimmed, fb, _off);                 \
            else                                                       \
                fast_draw_pixel_yuv422(dimmed, overlay, fb,            \
                                       _off, (alpha), shine_through);  \
        /* }                                                         */ \
    } while (0)

    if (dx == 0) {
        int sx = steep ? y0 : x0;
        int sy = steep ? x0 : y0;
        for (int oy = -lo; oy <= hi; oy++)
            for (int ox = -lo; ox <= hi; ox++)
                DISPATCHYUV(sx + ox, sy + oy, brightness);
        return;
    }

    int32_t grad = ((int32_t)dy << 8) / dx;
    int32_t y_fp = (int32_t)y0 << 8;

    for (int x = x0; x <= x1; x++, y_fp += grad) {
        int y_int = (int)(y_fp >> 8);
        int frac  = (int)(y_fp & 0xFF);

        if (hi == 0) {
            int alpha_top = (brightness * (255 - frac)) >> 8;
            if (steep) DISPATCHYUV(y_int, x, alpha_top);
            else       DISPATCHYUV(x, y_int, alpha_top);
            if (frac) {
                int alpha_bot = (brightness * frac) >> 8;
                if (steep) DISPATCHYUV(y_int + 1, x, alpha_bot);
                else       DISPATCHYUV(x, y_int + 1, alpha_bot);
            }
        } else {
            for (int c = y_int - lo; c <= y_int + hi; c++) {
                if (steep) DISPATCHYUV(c, x, brightness);
                else       DISPATCHYUV(x, c, brightness);
            }
        }
    }

#undef DISPATCHYUV
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

IRAM_ATTR void overlay_fast_erase_line_yuv422(
        const overlay_ctx_yuv422_t *ctx, uint8_t *fb,
        int x0, int y0, int x1, int y1, int thickness)
{
    rasterise_yuv422(ctx, fb, x0, y0, x1, y1, 255, thickness, 0, MODE_ERASE);
}

IRAM_ATTR void overlay_fast_draw_line_yuv422(
        const overlay_ctx_yuv422_t *ctx, uint8_t *fb,
        int x0, int y0, int x1, int y1,
        int brightness, int thickness, int shine_through)
{
    if (brightness <= 0) return;
    if (brightness > 255) brightness = 255;
    if (shine_through < 0)   shine_through = 0;
    if (shine_through > 255) shine_through = 255;

    rasterise_yuv422(ctx, fb, x0, y0, x1, y1,
                     brightness, thickness, shine_through, MODE_DRAW);
}
