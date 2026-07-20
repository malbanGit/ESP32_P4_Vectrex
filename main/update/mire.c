/*
 * Self-generated test card (mire). Drawing is always done in RGB888; the final
 * frame-buffer format (RGB888 or YUV422) is handled in mire_render().
 */
#include <string.h>
#include <stdio.h>
#include "mire.h"
#include "esp_heap_caps.h"

// BT.601 full-range RGB -> YCbCr, clamped 0..255. Fixed-point (/256).
static inline uint8_t clamp_u8(int v) { return v < 0 ? 0 : (v > 255 ? 255 : (uint8_t)v); }
static inline uint8_t rgb_y (uint8_t r, uint8_t g, uint8_t b) { return clamp_u8(( 77 * r + 150 * g +  29 * b) >> 8); }
static inline uint8_t rgb_cb(uint8_t r, uint8_t g, uint8_t b) { return clamp_u8(128 + ((-43 * r -  85 * g + 128 * b) >> 8)); }
static inline uint8_t rgb_cr(uint8_t r, uint8_t g, uint8_t b) { return clamp_u8(128 + ((128 * r - 107 * g -  21 * b) >> 8)); }

// RGB of the test pattern at (x,y): white border, else one of 8 vertical bars.
static inline void pattern_rgb(int x, int y, int w, int h, uint8_t out[3])
{
    static const uint8_t bars[8][3] = {
        {255, 255, 255}, // white
        {255, 255,   0}, // yellow
        {  0, 255, 255}, // cyan
        {  0, 255,   0}, // green
        {255,   0, 255}, // magenta
        {255,   0,   0}, // red
        {  0,   0, 255}, // blue
        {  0,   0,   0}, // black
    };
    const int border = 4;
    if (x < border || x >= w - border || y < border || y >= h - border) {
        out[0] = out[1] = out[2] = 255;
    } else {
        const uint8_t *c = bars[(x * 8) / w];
        out[0] = c[0]; out[1] = c[1]; out[2] = c[2];
    }
}

// ---- Tiny 3x5 pixel font (retro look, fits the Vectrex vibe) ----------------
// Each glyph is 5 rows; per row bit2=left, bit1=middle, bit0=right pixel.
typedef struct { char c; uint8_t rows[5]; } glyph_t;
static const glyph_t FONT[] = {
    {' ', {0, 0, 0, 0, 0}}, {'-', {0, 0, 7, 0, 0}}, {'+', {0, 2, 7, 2, 0}},
    {':', {0, 2, 0, 2, 0}}, {'@', {7, 5, 5, 4, 7}},
    {'0', {7, 5, 5, 5, 7}}, {'1', {2, 6, 2, 2, 7}}, {'2', {7, 1, 7, 4, 7}},
    {'3', {7, 1, 7, 1, 7}}, {'4', {5, 5, 7, 1, 1}}, {'5', {7, 4, 7, 1, 7}},
    {'6', {7, 4, 7, 5, 7}}, {'7', {7, 1, 2, 2, 2}}, {'8', {7, 5, 7, 5, 7}},
    {'9', {7, 5, 7, 1, 7}},
    {'A', {7, 5, 7, 5, 5}}, {'B', {6, 5, 6, 5, 6}}, {'D', {6, 5, 5, 5, 6}},
    {'E', {7, 4, 6, 4, 7}}, {'G', {7, 4, 5, 5, 7}}, {'H', {5, 5, 7, 5, 5}},
    {'I', {7, 2, 2, 2, 7}}, {'K', {5, 6, 4, 6, 5}}, {'L', {4, 4, 4, 4, 7}},
    {'M', {5, 7, 7, 5, 5}}, {'O', {7, 5, 5, 5, 7}}, {'P', {7, 5, 7, 4, 4}},
    {'S', {7, 4, 7, 1, 7}}, {'T', {7, 2, 2, 2, 2}}, {'V', {5, 5, 5, 5, 2}},
    {'X', {5, 5, 2, 5, 5}}, {'Z', {7, 1, 2, 4, 7}},
};

// ---- RGB888 drawing primitives ---------------------------------------------
static inline void px(uint8_t *rgb, int w, int h, int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
    if ((unsigned)x >= (unsigned)w || (unsigned)y >= (unsigned)h) return;
    uint8_t *p = rgb + ((size_t)y * w + x) * 3;
    p[0] = r; p[1] = g; p[2] = b;
}

static void fill_rect(uint8_t *rgb, int w, int h, int x0, int y0, int rw, int rh,
                      uint8_t r, uint8_t g, uint8_t b)
{
    for (int y = y0; y < y0 + rh; y++)
        for (int x = x0; x < x0 + rw; x++)
            px(rgb, w, h, x, y, r, g, b);
}

// Alpha (0..255) blend of (r,g,b) over the existing pixels in the rectangle.
static void blend_rect(uint8_t *rgb, int w, int h, int x0, int y0, int rw, int rh,
                       uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
    for (int y = y0; y < y0 + rh; y++) {
        if ((unsigned)y >= (unsigned)h) continue;
        for (int x = x0; x < x0 + rw; x++) {
            if ((unsigned)x >= (unsigned)w) continue;
            uint8_t *p = rgb + ((size_t)y * w + x) * 3;
            p[0] = (uint8_t)((p[0] * (255 - a) + r * a) / 255);
            p[1] = (uint8_t)((p[1] * (255 - a) + g * a) / 255);
            p[2] = (uint8_t)((p[2] * (255 - a) + b * a) / 255);
        }
    }
}

// Rectangle outline of thickness t.
static void draw_frame(uint8_t *rgb, int w, int h, int x0, int y0, int rw, int rh,
                       int t, uint8_t r, uint8_t g, uint8_t b)
{
    fill_rect(rgb, w, h, x0, y0, rw, t, r, g, b);
    fill_rect(rgb, w, h, x0, y0 + rh - t, rw, t, r, g, b);
    fill_rect(rgb, w, h, x0, y0, t, rh, r, g, b);
    fill_rect(rgb, w, h, x0 + rw - t, y0, t, rh, r, g, b);
}

// Circle outline (ring) of thickness ~t, centered on (cx,cy).
static void draw_circle(uint8_t *rgb, int w, int h, int cx, int cy, int rad, int t,
                        uint8_t r, uint8_t g, uint8_t b)
{
    int r2o = (rad + t) * (rad + t), r2i = (rad - t) * (rad - t);
    for (int y = cy - rad - t; y <= cy + rad + t; y++)
        for (int x = cx - rad - t; x <= cx + rad + t; x++) {
            int d = (x - cx) * (x - cx) + (y - cy) * (y - cy);
            if (d <= r2o && d >= r2i) px(rgb, w, h, x, y, r, g, b);
        }
}

static const glyph_t *find_glyph(char c)
{
    for (size_t i = 0; i < sizeof(FONT) / sizeof(FONT[0]); i++)
        if (FONT[i].c == c) return &FONT[i];
    return &FONT[0]; // unknown -> space
}

// Draw a string at integer scale s (each font pixel = s x s block).
static void draw_text(uint8_t *rgb, int w, int h, int x, int y, const char *str, int s,
                      uint8_t r, uint8_t g, uint8_t b)
{
    for (const char *p = str; *p; p++) {
        const glyph_t *gl = find_glyph(*p);
        for (int row = 0; row < 5; row++)
            for (int col = 0; col < 3; col++)
                if (gl->rows[row] & (1 << (2 - col)))
                    fill_rect(rgb, w, h, x + col * s, y + row * s, s, s, r, g, b);
        x += 4 * s; // 3 px glyph + 1 px spacing
    }
}

// Pixel width of a string at scale s (no trailing space).
static int text_w(const char *str, int s)
{
    int n = 0;
    for (const char *p = str; *p; p++) n++;
    return n > 0 ? n * 4 * s - s : 0;
}

// Compose the test card into an RGB888 buffer (resolution-agnostic).
static void compose_rgb(uint8_t *rgb, int w, int h, const char *title)
{
    // Base: 8 SMPTE color bars + white border.
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            uint8_t c[3];
            pattern_rgb(x, y, w, h, c);
            px(rgb, w, h, x, y, c[0], c[1], c[2]);
        }

    int cx = w / 2, cy = h / 2;

    // Geometry helpers: centered ring, full-screen crosshair, corner L-marks.
    draw_circle(rgb, w, h, cx, cy, h * 9 / 20, 2, 255, 255, 255);
    fill_rect(rgb, w, h, cx - 1, 0, 2, h, 255, 255, 255);
    fill_rect(rgb, w, h, 0, cy - 1, w, 2, 255, 255, 255);
    const int m = 40, len = 40, t = 3;   // margin, arm length, thickness
    for (int i = 0; i < 4; i++) {
        int right = i & 1, bottom = i & 2;
        int ex = right ? w - m : m;       // elbow at the corner of the active area
        int ey = bottom ? h - m : m;
        int hx = right ? ex - len : ex;   // arms extend toward the screen center
        int hy = bottom ? ey - t : ey;
        int vx = right ? ex - t : ex;
        int vy = bottom ? ey - len : ey;
        fill_rect(rgb, w, h, hx, hy, len, t, 255, 255, 255);  // horizontal arm
        fill_rect(rgb, w, h, vx, vy, t, len, 255, 255, 255);  // vertical arm
    }

    // Centered translucent info panel with a white frame.
    int pw = 580, ph = 200, x0 = cx - pw / 2, y0 = cy - ph / 2;
    blend_rect(rgb, w, h, x0, y0, pw, ph, 0, 0, 0, 190);
    draw_frame(rgb, w, h, x0, y0, pw, ph, 3, 255, 255, 255);

    // Generated text, centered in the panel.
    const char *l1 = "ESP32-P4 + LT8912B";
    char l2[40], l3[24];
    snprintf(l2, sizeof(l2), "%s %dX%d @ 60HZ", title, w, h);
    snprintf(l3, sizeof(l3), "%s : OK", title);
    draw_text(rgb, w, h, cx - text_w(l1, 6) / 2, y0 + 28,  l1, 6, 120, 220, 255);
    draw_text(rgb, w, h, cx - text_w(l2, 5) / 2, y0 + 92,  l2, 5, 255, 255, 255);
    draw_text(rgb, w, h, cx - text_w(l3, 7) / 2, y0 + 144, l3, 7, 120, 255, 140);
}

esp_err_t mire_render(uint8_t *fb, int w, int h, bool yuv422, const char *title)
{
    if (!yuv422) {
        // Native RGB888: compose straight into the frame buffer.
        compose_rgb(fb, w, h, title);
        return ESP_OK;
    }

    // YUV422: compose in an RGB888 scratch buffer, then pack to YUYV into the fb.
    uint8_t *rgb = heap_caps_malloc((size_t)w * h * 3, MALLOC_CAP_SPIRAM);
    if (!rgb) return ESP_ERR_NO_MEM;
    compose_rgb(rgb, w, h, title);
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x += 2) {        // two pixels share one Cb/Cr
            uint8_t *s0 = rgb + ((size_t)y * w + x) * 3;
            uint8_t *s1 = s0 + 3;
            uint8_t *d  = fb + ((size_t)y * w + x) * 2;
            d[0] = rgb_y(s0[0], s0[1], s0[2]);   // Y0
            d[1] = rgb_cb(s0[0], s0[1], s0[2]);  // U  (shared)
            d[2] = rgb_y(s1[0], s1[1], s1[2]);   // Y1
            d[3] = rgb_cr(s0[0], s0[1], s0[2]);  // V  (shared)
        }
    heap_caps_free(rgb);
    return ESP_OK;
}
