// draw_line_gauss_yuv422.c — Gaussian-glow additive line drawer (YUV422 / YUYV)
//
// Replaces draw_line_yuv422.S.  Same external signature.
//
// YUV422 layout (YUYV): each pixel at (px, py) has:
//   Y  at  (py*fw + px)*2 + 0
//   UV at  (py*fw + px)*2 + 1  (U for even px, V for odd px)
// Black = Y:0, U:128, V:128.
// We only ADD to the Y channel; UV stays at 128 (neutral chroma = white beam).

#include "draw_line_yuv422.h"
#include <stdint.h>

#define IRAM_ATTR __attribute__((section(".iram0.text")))

static const uint8_t gauss_lut[8][5] = {
    {11, 117, 255, 117, 11},
    { 7,  95, 252, 140, 16},
    { 5,  75, 243, 164, 23},
    { 3,  58, 228, 188, 32},
    { 2,  44, 210, 210, 44},
    { 1,  32, 188, 228, 58},
    { 1,  23, 164, 243, 75},
    { 0,  16, 140, 252, 95},
};

static const uint8_t fringe_w[2] = {117, 11};

static IRAM_ATTR inline void add_px(uint8_t *fb, int fw, int fh,
                                     int px, int py, int contrib)
{
    if ((unsigned)px >= (unsigned)fw || (unsigned)py >= (unsigned)fh) return;
    uint8_t *p = fb + (py * fw + px) * 2;
    int y = p[0] + contrib;
    p[0] = y > 255 ? 255 : (uint8_t)y;
    // p[1] (UV byte) intentionally left unchanged
}

void IRAM_ATTR draw_line_asm_yuv422(uint8_t *fb, int fb_w, int fb_h,
                                     int x0, int y0, int x1, int y1,
                                     int brightness, int thickness)
{
    if (brightness <= 0) return;
    if (brightness > 255) brightness = 255;

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

    if (dx == 0) {
        int sx = steep ? y0 : x0;
        int sy = steep ? x0 : y0;
        for (int oy = -lo; oy <= hi; ++oy)
            for (int ox = -lo; ox <= hi; ++ox)
                add_px(fb, fb_w, fb_h, sx + ox, sy + oy, brightness);
        return;
    }

    int gradient = (dy << 8) / dx;
    int y_fp = y0 << 8;

    for (int x = x0; x <= x1; ++x) {
        int y_int = y_fp >> 8;
        int frac  = y_fp & 0xFF;
        int fi    = frac >> 5;

        if (hi == 0) {
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
            for (int o = -lo; o <= hi; ++o) {
                int yk = y_int + o;
                if (steep)
                    add_px(fb, fb_w, fb_h, yk, x, brightness);
                else
                    add_px(fb, fb_w, fb_h, x, yk, brightness);
            }
            for (int f = 0; f < 2; ++f) {
                int w = (brightness * fringe_w[f] + 127) >> 8;
                if (w == 0) continue;
                {
                    int yk = y_int + hi + 1 + f;
                    if (steep) add_px(fb, fb_w, fb_h, yk, x, w);
                    else       add_px(fb, fb_w, fb_h, x, yk, w);
                }
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
