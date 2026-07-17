#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Distance-field CRT vector line renderer for YUV422 (YUYV) framebuffers.
 * Additive Gaussian glow, round caps, crossing lines accumulate brightness.
 * Only the Y (luma) byte is written; UV chroma bytes are left unchanged
 * (caller must initialise UV=128 for a neutral white-on-black display).
 *
 * YUYV memory layout per row (fb_w pixels):
 *   [Y0, U01, Y1, V01,  Y2, U23, Y3, V23, ...]
 *   pixel x: Y at (y*fb_w+x)*2,  UV-byte at (y*fb_w+x)*2+1
 *
 * g_line_glow (defined in draw_line_dist_rgb888.c) controls the glow
 * radius in pixels beyond the beam edge (default 2, 0 = hard beam).
 */
extern int g_line_glow;

// Gaussian-glow additive line (adds to Y channel only; UV bytes untouched).
// Crossing lines accumulate brightness.  Endpoints are rounded (filled circle).
// brightness > 255 saturates the core and spills as a halo to cardinal neighbours.
void draw_line_asm_yuv422(uint8_t *fb, int fb_w, int fb_h,
                           int x0, int y0, int x1, int y1,
                           int brightness, int thickness);

// Erase all pixels that draw_line_asm_yuv422 could have touched (sets Y=0).
// 'brightness' is accepted but ignored.  UV bytes are left unchanged.
// Uses g_line_glow — keep it constant between draw/undraw.
void undraw_line_asm_yuv422(uint8_t *fb, int fb_w, int fb_h,
                             int x0, int y0, int x1, int y1,
                             int brightness, int thickness);

#ifdef __cplusplus
}
#endif
