#include "overlay_line.h"
#include "lodepng.h"

#include <stdlib.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"

static const char *TAG = "overlay_line";

// ---------------------------------------------------------------------------
// Colour helpers
// ---------------------------------------------------------------------------

static IRAM_ATTR inline int expand5(int v) { return (v << 3) | (v >> 2); }
static IRAM_ATTR inline int expand6(int v) { return (v << 2) | (v >> 4); }

// Unpack an RGB565 value into 8-bit R, G, B components.
static IRAM_ATTR inline void unpack565(uint16_t c, int *r, int *g, int *b)
{
    *r = expand5((c >> 11) & 0x1F);
    *g = expand6((c >>  5) & 0x3F);
    *b = expand5( c        & 0x1F);
}

// Pack 8-bit R, G, B into RGB565 (values must be 0-255).
static IRAM_ATTR inline uint16_t pack565(int r, int g, int b)
{
    return (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
}

// Saturating add: clamp result to [0, 255].
static IRAM_ATTR inline int sadd(int a, int b)
{
    int s = a + b;
    return s > 255 ? 255 : s;
}

// ---------------------------------------------------------------------------
// Pixel-level primitives (REPLACE and ADDITIVE)
// ---------------------------------------------------------------------------

/*
 * Compute the beam colour at (px, py):
 *   beam = lerp(white=255, overlay_channel, shine_through/256) × alpha/256
 *
 * alpha already incorporates both AA coverage and brightness.
 * Returns the RGB565 colour to write (or add).
 */
static IRAM_ATTR inline uint16_t beam_color(
        const uint16_t *overlay, int fb_w,
        int px, int py,
        int alpha, int shine_through)
{
    int br, bg, bb;
    if (overlay && shine_through > 0) {
        unpack565(overlay[py * fb_w + px], &br, &bg, &bb);
        // Lerp white → overlay colour
        br = 255 + (((br - 255) * shine_through) >> 8);
        bg = 255 + (((bg - 255) * shine_through) >> 8);
        bb = 255 + (((bb - 255) * shine_through) >> 8);
    } else {
        br = bg = bb = 255;
    }
    return pack565((br * alpha) >> 8, (bg * alpha) >> 8, (bb * alpha) >> 8);
}

/* Write beam colour to fb[py][px] — bounds-checked, REPLACE. */
static IRAM_ATTR inline void put_replace(
        uint16_t *fb, int fb_w, int fb_h,
        int px, int py,
        int alpha, const uint16_t *overlay, int shine_through)
{
    if ((unsigned)px >= (unsigned)fb_w || (unsigned)py >= (unsigned)fb_h) return;
    fb[py * fb_w + px] = beam_color(overlay, fb_w, px, py, alpha, shine_through);
}

/* Write beam colour to fb[py][px] — bounds-checked, ADDITIVE (saturating). */
static IRAM_ATTR inline void put_additive(
        uint16_t *fb, int fb_w, int fb_h,
        int px, int py,
        int alpha, const uint16_t *overlay, int shine_through)
{
    if ((unsigned)px >= (unsigned)fb_w || (unsigned)py >= (unsigned)fb_h) return;

    uint16_t beam = beam_color(overlay, fb_w, px, py, alpha, shine_through);

    // Saturating add with existing framebuffer content
    int er, eg, eb, br, bg, bb;
    unpack565(fb[py * fb_w + px], &er, &eg, &eb);
    unpack565(beam,               &br, &bg, &bb);

    fb[py * fb_w + px] = pack565(sadd(er, br), sadd(eg, bg), sadd(eb, bb));
}

// ---------------------------------------------------------------------------
// Generic line rasteriser — shared by both REPLACE and ADDITIVE variants
// ---------------------------------------------------------------------------

typedef void (*put_pixel_fn)(uint16_t *fb, int fb_w, int fb_h,
                              int px, int py,
                              int alpha, const uint16_t *overlay, int shine_through);

static IRAM_ATTR void rasterise_line(
        uint16_t *fb, int fb_w, int fb_h,
        int x0, int y0, int x1, int y1,
        int brightness, int thickness,
        const uint16_t *overlay, int shine_through,
        put_pixel_fn put)
{
    int half_w = thickness > 0 ? thickness >> 1 : 0;

    // Steep detection
    int adx = x1 - x0; if (adx < 0) adx = -adx;
    int ady = y1 - y0; if (ady < 0) ady = -ady;
    int steep = (ady > adx);
    if (steep) { int t; t=x0;x0=y0;y0=t; t=x1;x1=y1;y1=t; }
    if (x0 > x1) { int t; t=x0;x0=x1;x1=t; t=y0;y0=y1;y1=t; }

    int dx = x1 - x0;
    int dy = y1 - y0;

    // Degenerate: single point
    if (dx == 0) {
        int sx = steep ? y0 : x0;
        int sy = steep ? x0 : y0;
        for (int oy = -half_w; oy <= half_w; oy++)
            for (int ox = -half_w; ox <= half_w; ox++)
                put(fb, fb_w, fb_h, sx+ox, sy+oy, brightness, overlay, shine_through);
        return;
    }

    // Q8 fixed-point gradient
    int32_t grad = ((int32_t)dy << 8) / dx;
    int32_t y_fp = (int32_t)y0 << 8;

    for (int x = x0; x <= x1; x++, y_fp += grad) {
        int y_int = (int)(y_fp >> 8);
        int frac  = (int)(y_fp & 0xFF);

        // Top AA edge
        int alpha_top = (brightness * (255 - frac)) >> 8;
        int ytop = y_int - half_w;
        if (steep) put(fb, fb_w, fb_h, ytop, x, alpha_top, overlay, shine_through);
        else       put(fb, fb_w, fb_h, x, ytop, alpha_top, overlay, shine_through);

        // Solid core pixels
        for (int c = y_int - half_w + 1; c <= y_int + half_w; c++) {
            if (steep) put(fb, fb_w, fb_h, c, x, brightness, overlay, shine_through);
            else       put(fb, fb_w, fb_h, x, c, brightness, overlay, shine_through);
        }

        // Bottom AA edge (skip when frac==0, zero coverage)
        if (frac) {
            int alpha_bot = (brightness * frac) >> 8;
            int ybot = y_int + half_w + 1;
            if (steep) put(fb, fb_w, fb_h, ybot, x, alpha_bot, overlay, shine_through);
            else       put(fb, fb_w, fb_h, x, ybot, alpha_bot, overlay, shine_through);
        }
    }
}

// ---------------------------------------------------------------------------
// overlay_draw_background
// ---------------------------------------------------------------------------
void overlay_draw_background(uint16_t *fb, int fb_w, int fb_h,
                              const uint16_t *overlay, int brightness)
{
    if (!overlay) {
        memset(fb, 0, (size_t)fb_w * fb_h * sizeof(uint16_t));
        return;
    }
    if (brightness <= 0) {
        memset(fb, 0, (size_t)fb_w * fb_h * sizeof(uint16_t));
        return;
    }
    if (brightness >= 255) {
        memcpy(fb, overlay, (size_t)fb_w * fb_h * sizeof(uint16_t));
        return;
    }

    int total = fb_w * fb_h;
    for (int i = 0; i < total; i++) {
        int r, g, b;
        unpack565(overlay[i], &r, &g, &b);
        fb[i] = pack565((r * brightness) >> 8,
                         (g * brightness) >> 8,
                         (b * brightness) >> 8);
    }
}

// ---------------------------------------------------------------------------
// draw_line_vectrex  (ADDITIVE — the primary Vectrex render function)
// ---------------------------------------------------------------------------
IRAM_ATTR void draw_line_vectrex(
        uint16_t *fb, int fb_w, int fb_h,
        int x0, int y0, int x1, int y1,
        int brightness, int thickness,
        const uint16_t *overlay, int shine_through)
{
    if (shine_through < 0)   shine_through = 0;
    if (shine_through > 255) shine_through = 255;

    rasterise_line(fb, fb_w, fb_h, x0, y0, x1, y1,
                   brightness, thickness, overlay, shine_through,
                   put_additive);
}

// ---------------------------------------------------------------------------
// draw_line_overlay  (REPLACE — draws directly onto a black background)
// ---------------------------------------------------------------------------
IRAM_ATTR void draw_line_overlay(
        uint16_t *fb, int fb_w, int fb_h,
        int x0, int y0, int x1, int y1,
        int brightness, int thickness,
        const uint16_t *overlay, int shine_through)
{
    // No colour tinting: fall back to the optimised asm routine.
    if (!overlay || shine_through <= 0) {
        draw_line_asm(fb, fb_w, fb_h, x0, y0, x1, y1, brightness, thickness);
        return;
    }
    if (shine_through > 255) shine_through = 255;

    rasterise_line(fb, fb_w, fb_h, x0, y0, x1, y1,
                   brightness, thickness, overlay, shine_through,
                   put_replace);
}

// ---------------------------------------------------------------------------
// overlay_load_png
// ---------------------------------------------------------------------------
esp_err_t overlay_load_png(const char *path, uint16_t *fb, int fb_w, int fb_h)
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
            fb[y * fb_w + x] = pack565(px[0], px[1], px[2]); // alpha ignored
        }
    }

    free(raw);
    return ESP_OK;
}
