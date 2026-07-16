#include "overlay_fast.h"

#include <string.h>
#include <stdlib.h>

#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "overlay_fast";

// ---------------------------------------------------------------------------
// Colour helpers — stay in 5/6-bit space throughout to avoid
// expand-to-8-bit/repack roundtrips.
// ---------------------------------------------------------------------------

// Saturating add for a 5-bit channel (max 31).
#define SADD5(a, b)  ({ int _s = (a) + (b); _s > 31 ? 31 : _s; })
// Saturating add for a 6-bit channel (max 63).
#define SADD6(a, b)  ({ int _s = (a) + (b); _s > 63 ? 63 : _s; })

// ---------------------------------------------------------------------------
// Context management
// ---------------------------------------------------------------------------

esp_err_t overlay_fast_init(overlay_ctx_t *ctx,
                             const uint16_t *overlay, int fb_w, int fb_h,
                             int bg_brightness)
{
    ctx->overlay = overlay;
    ctx->fb_w    = fb_w;
    ctx->fb_h    = fb_h;

    size_t bytes = (size_t)fb_w * fb_h * sizeof(uint16_t);
    ctx->dimmed  = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);
    if (!ctx->dimmed) {
        ESP_LOGE(TAG, "Failed to allocate %zu bytes for dimmed buffer", bytes);
        return ESP_ERR_NO_MEM;
    }

    overlay_fast_set_brightness(ctx, bg_brightness);
    return ESP_OK;
}

void overlay_fast_free(overlay_ctx_t *ctx)
{
    if (ctx->dimmed) {
        free(ctx->dimmed);
        ctx->dimmed = NULL;
    }
}

void overlay_fast_set_brightness(overlay_ctx_t *ctx, int bg_brightness)
{
    if (bg_brightness < 0)   bg_brightness = 0;
    if (bg_brightness > 255) bg_brightness = 255;

    int total = ctx->fb_w * ctx->fb_h;
    const uint16_t *src = ctx->overlay;
    uint16_t       *dst = ctx->dimmed;

    if (bg_brightness == 0) {
        memset(dst, 0, (size_t)total * sizeof(uint16_t));
        return;
    }
    if (bg_brightness == 255) {
        memcpy(dst, src, (size_t)total * sizeof(uint16_t));
        return;
    }

    // Scale each channel in 5/6-bit space.
    // channel_out = channel_in * brightness / 256
    for (int i = 0; i < total; i++) {
        uint16_t c = src[i];
        int r = ((c >> 11) & 0x1F) * bg_brightness >> 8;
        int g = ((c >>  5) & 0x3F) * bg_brightness >> 8;
        int b = ( c        & 0x1F) * bg_brightness >> 8;
        dst[i] = (uint16_t)((r << 11) | (g << 5) | b);
    }
}

// ---------------------------------------------------------------------------
// Pixel primitives (IRAM — called from the hot render loop)
// ---------------------------------------------------------------------------

/**
 * Restore one pixel to its background value from the dimmed buffer.
 * Bounds check is done by the caller (rasteriser ensures valid coords).
 */
static IRAM_ATTR inline void fast_restore(
        const uint16_t * restrict dimmed, uint16_t * restrict fb,
        int offset)
{
    fb[offset] = dimmed[offset];
}

/**
 * Compute and write one beam pixel: saturating-add beam colour to dimmed bg.
 *
 * alpha      : 0-255  (brightness × AA coverage, already multiplied)
 * shine_through: 0-255
 *
 * Colour arithmetic stays in 5/6-bit RGB565 component space.
 */
static IRAM_ATTR inline void fast_draw_pixel(
        const uint16_t * restrict dimmed,
        const uint16_t * restrict overlay,
        uint16_t       * restrict fb,
        int offset, int alpha, int shine_through)
{
    // ---- background (always know this value — no FB read needed) ----
    uint16_t bg = dimmed[offset];
    int bg_r = (bg >> 11) & 0x1F;
    int bg_g = (bg >>  5) & 0x3F;
    int bg_b =  bg        & 0x1F;

    // ---- beam colour: lerp(white, overlay, shine_through) in 5/6-bit ----
    // white_r5=31, white_g6=63, white_b5=31
    // beam = white + (overlay - white) * shine_through / 256
    //      = white - (white - overlay) * shine_through / 256
    int beam_r, beam_g, beam_b;
    if (shine_through <= 0) {
        beam_r = 31;
        beam_g = 63;
        beam_b = 31;
    } else {
        uint16_t ov = overlay[offset];
        int ov_r = (ov >> 11) & 0x1F;
        int ov_g = (ov >>  5) & 0x3F;
        int ov_b =  ov        & 0x1F;
        if (shine_through >= 255) {
            beam_r = ov_r;
            beam_g = ov_g;
            beam_b = ov_b;
        } else {
            beam_r = ov_r + (((31 - ov_r) * (255 - shine_through)) >> 8);
            beam_g = ov_g + (((63 - ov_g) * (255 - shine_through)) >> 8);
            beam_b = ov_b + (((31 - ov_b) * (255 - shine_through)) >> 8);
        }
    }

    // ---- scale beam by AA alpha (alpha already encodes brightness) ----
    // beam_r5 * alpha >> 8  keeps result in 0..31 range
    int fr = (beam_r * alpha) >> 8;
    int fg = (beam_g * alpha) >> 8;
    int fb_c = (beam_b * alpha) >> 8;

    // ---- saturating add onto dimmed background ----
    fr   = SADD5(bg_r, fr);
    fg   = SADD6(bg_g, fg);
    fb_c = SADD5(bg_b, fb_c);

    fb[offset] = (uint16_t)((fr << 11) | (fg << 5) | fb_c);
}

// ---------------------------------------------------------------------------
// Shared Wu AA rasteriser
//
// mode == 0 → erase (restore from dimmed)
// mode == 1 → draw  (beam + dimmed)
//
// Keeping one rasteriser guarantees erase and draw always visit identical
// pixels, so no stale fragments are left behind.
// ---------------------------------------------------------------------------

typedef enum { MODE_ERASE = 0, MODE_DRAW = 1 } raster_mode_t;

static IRAM_ATTR void rasterise(
        const overlay_ctx_t * restrict ctx,
        uint16_t * restrict fb,
        int x0, int y0, int x1, int y1,
        int brightness, int thickness, int shine_through,
        raster_mode_t mode)
{
    const uint16_t *dimmed  = ctx->dimmed;
    const uint16_t *overlay = ctx->overlay;
    int fw = ctx->fb_w;
    int fh = ctx->fb_h;
    int half_w = thickness > 1 ? (thickness - 1) >> 1 : 0;

    // ---- steep detection ----
    int adx = x1 - x0; if (adx < 0) adx = -adx;
    int ady = y1 - y0; if (ady < 0) ady = -ady;
    int steep = (ady > adx);
    if (steep) { int t; t=x0;x0=y0;y0=t; t=x1;x1=y1;y1=t; }
    if (x0 > x1) { int t; t=x0;x0=x1;x1=t; t=y0;y0=y1;y1=t; }

    int dx = x1 - x0;
    int dy = y1 - y0;

// Pixel dispatch — bounds checks commented out; rasteriser stays within fb.
#define DISPATCH(px, py, alpha)                                        \
    do {                                                               \
        /* if ((unsigned)(px) < (unsigned)fw &&                     */ \
        /*     (unsigned)(py) < (unsigned)fh) {                     */ \
            int _off = (py) * fw + (px);                              \
            if (mode == MODE_ERASE)                                    \
                fast_restore(dimmed, fb, _off);                        \
            else                                                       \
                fast_draw_pixel(dimmed, overlay, fb,                   \
                                _off, (alpha), shine_through);        \
        /* }                                                         */ \
    } while (0)

    // ---- degenerate: single point → filled square ----
    if (dx == 0) {
        int sx = steep ? y0 : x0;
        int sy = steep ? x0 : y0;
        for (int oy = -half_w; oy <= half_w; oy++)
            for (int ox = -half_w; ox <= half_w; ox++)
                DISPATCH(sx + ox, sy + oy, brightness);
        return;
    }

    // ---- Q8 fixed-point gradient ----
    int32_t grad = ((int32_t)dy << 8) / dx;
    int32_t y_fp = (int32_t)y0 << 8;

    for (int x = x0; x <= x1; x++, y_fp += grad) {
        int y_int = (int)(y_fp >> 8);
        int frac  = (int)(y_fp & 0xFF);

        // Top AA edge
        int alpha_top = (brightness * (255 - frac)) >> 8;
        int ytop = y_int - half_w;
        if (steep) DISPATCH(ytop, x, alpha_top);
        else       DISPATCH(x, ytop, alpha_top);

        // Solid core pixels (full brightness)
        for (int c = y_int - half_w + 1; c <= y_int + half_w; c++) {
            if (steep) DISPATCH(c, x, brightness);
            else       DISPATCH(x, c, brightness);
        }

        // Bottom AA edge (skip at frac==0 — zero coverage)
        if (frac) {
            int alpha_bot = (brightness * frac) >> 8;
            int ybot = y_int + half_w + 1;
            if (steep) DISPATCH(ybot, x, alpha_bot);
            else       DISPATCH(x, ybot, alpha_bot);
        }
    }

#undef DISPATCH
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

IRAM_ATTR void overlay_fast_erase_line(
        const overlay_ctx_t *ctx, uint16_t *fb,
        int x0, int y0, int x1, int y1, int thickness)
{
    // brightness is irrelevant for erase (we restore all touched pixels)
    // but the rasteriser needs it to compute the same pixel positions.
    // Use 255 so every AA pixel (even dim ones) is touched.
    rasterise(ctx, fb, x0, y0, x1, y1, 255, thickness, 0, MODE_ERASE);
}

IRAM_ATTR void overlay_fast_draw_line(
        const overlay_ctx_t *ctx, uint16_t *fb,
        int x0, int y0, int x1, int y1,
        int brightness, int thickness, int shine_through)
{
    if (brightness <= 0) return;
    if (brightness > 255) brightness = 255;
    if (shine_through < 0)   shine_through = 0;
    if (shine_through > 255) shine_through = 255;

    rasterise(ctx, fb, x0, y0, x1, y1,
              brightness, thickness, shine_through, MODE_DRAW);
}
