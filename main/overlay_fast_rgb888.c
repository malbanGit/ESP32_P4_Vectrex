#include "overlay_fast_rgb888.h"

#include <string.h>
#include <stdlib.h>

#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "overlay_fast_rgb888";

#define SADD8(a, b)  ({ int _s = (a) + (b); _s > 255 ? 255 : _s; })

// ---------------------------------------------------------------------------
// Context management
// ---------------------------------------------------------------------------

esp_err_t overlay_fast_init_rgb888(overlay_ctx_rgb888_t *ctx,
                                    const uint8_t *overlay, int fb_w, int fb_h,
                                    int bg_brightness)
{
    ctx->overlay = overlay;
    ctx->fb_w    = fb_w;
    ctx->fb_h    = fb_h;

    size_t bytes = (size_t)fb_w * fb_h * 3;
    ctx->dimmed  = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);
    if (!ctx->dimmed) {
        ESP_LOGE(TAG, "Failed to allocate %zu bytes for dimmed buffer", bytes);
        return ESP_ERR_NO_MEM;
    }

    overlay_fast_set_brightness_rgb888(ctx, bg_brightness);
    return ESP_OK;
}

void overlay_fast_free_rgb888(overlay_ctx_rgb888_t *ctx)
{
    if (ctx->dimmed) {
        free(ctx->dimmed);
        ctx->dimmed = NULL;
    }
}

void overlay_fast_set_brightness_rgb888(overlay_ctx_rgb888_t *ctx, int bg_brightness)
{
    if (bg_brightness < 0)   bg_brightness = 0;
    if (bg_brightness > 255) bg_brightness = 255;

    size_t total = (size_t)ctx->fb_w * ctx->fb_h * 3;
    const uint8_t *src = ctx->overlay;
    uint8_t       *dst = ctx->dimmed;

    if (bg_brightness == 0) {
        memset(dst, 0, total);
        return;
    }
    if (bg_brightness == 255) {
        memcpy(dst, src, total);
        return;
    }

    for (size_t i = 0; i < total; i++)
        dst[i] = (uint8_t)((src[i] * bg_brightness) >> 8);
}

// ---------------------------------------------------------------------------
// Pixel primitives (IRAM)
// ---------------------------------------------------------------------------

static IRAM_ATTR inline void fast_restore_rgb888(
        const uint8_t * restrict dimmed, uint8_t * restrict fb, int offset)
{
    const uint8_t *s = dimmed + offset * 3;
    uint8_t       *d = fb    + offset * 3;
    d[0] = s[0]; d[1] = s[1]; d[2] = s[2];
}

static IRAM_ATTR inline void fast_draw_pixel_rgb888(
        const uint8_t * restrict dimmed,
        const uint8_t * restrict overlay,
        uint8_t       * restrict fb,
        int offset, int alpha, int shine_through)
{
    const uint8_t *bg_p = dimmed + offset * 3;
    int bg_r = bg_p[0], bg_g = bg_p[1], bg_b = bg_p[2];

    int beam_r, beam_g, beam_b;
    if (shine_through <= 0) {
        beam_r = beam_g = beam_b = 255;
    } else {
        const uint8_t *ov_p = overlay + offset * 3;
        int ov_r = ov_p[0], ov_g = ov_p[1], ov_b = ov_p[2];
        if (shine_through >= 255) {
            beam_r = ov_r; beam_g = ov_g; beam_b = ov_b;
        } else {
            beam_r = ov_r + (((255 - ov_r) * (255 - shine_through)) >> 8);
            beam_g = ov_g + (((255 - ov_g) * (255 - shine_through)) >> 8);
            beam_b = ov_b + (((255 - ov_b) * (255 - shine_through)) >> 8);
        }
    }

    int fr   = (beam_r * alpha) >> 8;
    int fg   = (beam_g * alpha) >> 8;
    int fb_c = (beam_b * alpha) >> 8;

    fr   = SADD8(bg_r, fr);
    fg   = SADD8(bg_g, fg);
    fb_c = SADD8(bg_b, fb_c);

    uint8_t *d = fb + offset * 3;
    d[0] = (uint8_t)fr;
    d[1] = (uint8_t)fg;
    d[2] = (uint8_t)fb_c;
}

// ---------------------------------------------------------------------------
// Shared rasteriser
// ---------------------------------------------------------------------------

typedef enum { MODE_ERASE = 0, MODE_DRAW = 1 } raster_mode_rgb888_t;

static IRAM_ATTR void rasterise_rgb888(
        const overlay_ctx_rgb888_t * restrict ctx,
        uint8_t * restrict fb,
        int x0, int y0, int x1, int y1,
        int brightness, int thickness, int shine_through,
        raster_mode_rgb888_t mode)
{
    const uint8_t *dimmed  = ctx->dimmed;
    const uint8_t *overlay = ctx->overlay;
    int fw = ctx->fb_w;
    int fh = ctx->fb_h;
    int half_w = thickness > 1 ? (thickness - 1) >> 1 : 0;

    int adx = x1 - x0; if (adx < 0) adx = -adx;
    int ady = y1 - y0; if (ady < 0) ady = -ady;
    int steep = (ady > adx);
    if (steep) { int t; t=x0;x0=y0;y0=t; t=x1;x1=y1;y1=t; }
    if (x0 > x1) { int t; t=x0;x0=x1;x1=t; t=y0;y0=y1;y1=t; }

    int dx = x1 - x0;
    int dy = y1 - y0;

#define DISPATCH8(px, py, alpha)                                       \
    do {                                                               \
        /* if ((unsigned)(px) < (unsigned)fw &&                     */ \
        /*     (unsigned)(py) < (unsigned)fh) {                     */ \
            int _off = (py) * fw + (px);                              \
            if (mode == MODE_ERASE)                                    \
                fast_restore_rgb888(dimmed, fb, _off);                 \
            else                                                       \
                fast_draw_pixel_rgb888(dimmed, overlay, fb,            \
                                       _off, (alpha), shine_through);  \
        /* }                                                         */ \
    } while (0)

    if (dx == 0) {
        int sx = steep ? y0 : x0;
        int sy = steep ? x0 : y0;
        for (int oy = -half_w; oy <= half_w; oy++)
            for (int ox = -half_w; ox <= half_w; ox++)
                DISPATCH8(sx + ox, sy + oy, brightness);
        return;
    }

    int32_t grad = ((int32_t)dy << 8) / dx;
    int32_t y_fp = (int32_t)y0 << 8;

    for (int x = x0; x <= x1; x++, y_fp += grad) {
        int y_int = (int)(y_fp >> 8);
        int frac  = (int)(y_fp & 0xFF);

        int alpha_top = (brightness * (255 - frac)) >> 8;
        int ytop = y_int - half_w;
        if (steep) DISPATCH8(ytop, x, alpha_top);
        else       DISPATCH8(x, ytop, alpha_top);

        for (int c = y_int - half_w + 1; c <= y_int + half_w; c++) {
            if (steep) DISPATCH8(c, x, brightness);
            else       DISPATCH8(x, c, brightness);
        }

        if (frac) {
            int alpha_bot = (brightness * frac) >> 8;
            int ybot = y_int + half_w + 1;
            if (steep) DISPATCH8(ybot, x, alpha_bot);
            else       DISPATCH8(x, ybot, alpha_bot);
        }
    }

#undef DISPATCH8
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

IRAM_ATTR void overlay_fast_erase_line_rgb888(
        const overlay_ctx_rgb888_t *ctx, uint8_t *fb,
        int x0, int y0, int x1, int y1, int thickness)
{
    rasterise_rgb888(ctx, fb, x0, y0, x1, y1, 255, thickness, 0, MODE_ERASE);
}

IRAM_ATTR void overlay_fast_draw_line_rgb888(
        const overlay_ctx_rgb888_t *ctx, uint8_t *fb,
        int x0, int y0, int x1, int y1,
        int brightness, int thickness, int shine_through)
{
    if (brightness <= 0) return;
    if (brightness > 255) brightness = 255;
    if (shine_through < 0)   shine_through = 0;
    if (shine_through > 255) shine_through = 255;

    rasterise_rgb888(ctx, fb, x0, y0, x1, y1,
                     brightness, thickness, shine_through, MODE_DRAW);
}
