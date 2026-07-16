// draw_line_gauss_rgb888.c — Gaussian-glow additive line drawer (RGB888)
//
// Replaces draw_line_rgb888.S.  Same external signature as the assembler
// version so every call site works unchanged.
//
// Algorithm:
//   Same steep-detection + Wu AA major-axis walk as the .S version, but
//   instead of writing a flat colour each pixel we ADD a Gaussian contribution
//   centred on the true sub-pixel line position.  Because we *add* to the
//   existing framebuffer value (and saturate at 255), two crossing lines
//   produce a bright hot-spot — exactly the CRT / Vectrex glow look.
//
// Gaussian kernel: σ = 0.8, evaluated at integer offsets -2..+2.
// LUT indexed by [frac>>5][k+2] where frac is the Q8 sub-pixel position.
// For thick lines (hi > 0) the solid core is written at full brightness and
// a 2-pixel Gaussian fringe is added on each outer edge.

#include "draw_line_rgb888.h"
#include <stdint.h>

#define IRAM_ATTR __attribute__((section(".iram0.text")))

// gauss_lut[f][k+2]:
//   f = frac >> 5  (0..7 — 8 evenly-spaced sub-pixel positions)
//   k = -2..+2     column offset from beam centre
// Values are gaussian(k - frac/255, σ=0.8) * 255, rounded.
static const uint8_t gauss_lut[8][5] = {
    {11, 117, 255, 117, 11},  // frac ≈ 0
    { 7,  95, 252, 140, 16},  // frac ≈ 32
    { 5,  75, 243, 164, 23},  // frac ≈ 64
    { 3,  58, 228, 188, 32},  // frac ≈ 96
    { 2,  44, 210, 210, 44},  // frac ≈ 128
    { 1,  32, 188, 228, 58},  // frac ≈ 160
    { 1,  23, 164, 243, 75},  // frac ≈ 192
    { 0,  16, 140, 252, 95},  // frac ≈ 224
};

// Gaussian values at offset 1 and 2 from the edge of the solid core,
// used for thick-line fringes.
static const uint8_t fringe_w[2] = {117, 11};  // gauss(1.0*255), gauss(2.0*255)

static IRAM_ATTR inline void add_px(uint8_t *fb, int fw, int fh,
                                     int px, int py, int contrib)
{
    if ((unsigned)px >= (unsigned)fw || (unsigned)py >= (unsigned)fh) return;
    uint8_t *p = fb + (py * fw + px) * 3;
    int v;
    v = p[0] + contrib; p[0] = v > 255 ? 255 : (uint8_t)v;
    v = p[1] + contrib; p[1] = v > 255 ? 255 : (uint8_t)v;
    v = p[2] + contrib; p[2] = v > 255 ? 255 : (uint8_t)v;
}

void IRAM_ATTR draw_line_asm_rgb888(uint8_t *fb, int fb_w, int fb_h,
                                     int x0, int y0, int x1, int y1,
                                     int brightness, int thickness)
{
    if (brightness <= 0) return;
    if (brightness > 255) brightness = 255;

    // Asymmetric lo/hi so exactly `thickness` pixels are in the solid core.
    int lo = 0, hi = 0;
    if (thickness > 1) {
        lo = (thickness - 1) >> 1;
        hi = thickness >> 1;
    }

    int dx = x1 - x0;
    int dy = y1 - y0;
    int adx = dx < 0 ? -dx : dx;
    int ady = dy < 0 ? -dy : dy;

    int steep = ady > adx;
    if (steep) {
        int t;
        t = x0; x0 = y0; y0 = t;
        t = x1; x1 = y1; y1 = t;
    }
    if (x0 > x1) {
        int t;
        t = x0; x0 = x1; x1 = t;
        t = y0; y0 = y1; y1 = t;
    }

    dx = x1 - x0;
    dy = y1 - y0;

    // Single-point degenerate case
    if (dx == 0) {
        int sx = steep ? y0 : x0;
        int sy = steep ? x0 : y0;
        for (int oy = -lo; oy <= hi; ++oy)
            for (int ox = -lo; ox <= hi; ++ox)
                add_px(fb, fb_w, fb_h, sx + ox, sy + oy, brightness);
        return;
    }

    // Q8 fixed-point gradient
    int gradient = (dy << 8) / dx;
    int y_fp = y0 << 8;

    for (int x = x0; x <= x1; ++x) {
        int y_int = y_fp >> 8;
        int frac  = y_fp & 0xFF;   // 0..255
        int fi    = frac >> 5;     // LUT row index 0..7

        if (hi == 0) {
            // Pure thin Gaussian: 5-pixel kernel centred between y_int and y_int+1
            for (int k = -2; k <= 2; ++k) {
                int contrib = (brightness * gauss_lut[fi][k + 2] + 127) >> 8;
                if (contrib == 0) continue;
                int yk = y_int + k;
                if (steep)
                    add_px(fb, fb_w, fb_h, yk, x, contrib);
                else
                    add_px(fb, fb_w, fb_h, x, yk, contrib);
            }
        } else {
            // Solid core: y_int - lo .. y_int + hi at full brightness
            for (int o = -lo; o <= hi; ++o) {
                int yk = y_int + o;
                if (steep)
                    add_px(fb, fb_w, fb_h, yk, x, brightness);
                else
                    add_px(fb, fb_w, fb_h, x, yk, brightness);
            }
            // Gaussian fringe beyond the solid core (2 pixels each side)
            for (int f = 0; f < 2; ++f) {
                int w = (brightness * fringe_w[f] + 127) >> 8;
                if (w == 0) continue;
                // outer edge below (positive offset side)
                {
                    int yk = y_int + hi + 1 + f;
                    if (steep) add_px(fb, fb_w, fb_h, yk, x, w);
                    else       add_px(fb, fb_w, fb_h, x, yk, w);
                }
                // outer edge above (negative offset side)
                {
                    int yk = y_int - lo - 1 - f;
                    if (steep) add_px(fb, fb_w, fb_h, yk, x, w);
                    else       add_px(fb, fb_w, fb_h, x, yk, w);
                }
            }
        }

        y_fp += gradient;
    }
}
