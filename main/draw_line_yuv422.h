#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * RV32IMAC assembler line-drawer for YUV422 (YUYV) framebuffers.
 * Writes Y=brightness to the luma byte; sets UV byte to 128 (neutral chroma
 * = white beam).  Same Wu AA + thickness algorithm as draw_line_asm().
 * Bounds checks commented out; caller guarantees valid screen coords.
 *
 * YUYV memory layout per row (fb_w pixels):
 *   [Y0, U01, Y1, V01,  Y2, U23, Y3, V23, ...]
 *   pixel x: Y at (y*fb_w+x)*2,  UV-byte at (y*fb_w+x)*2+1
 */
// Gaussian-glow additive line (adds to Y channel only; UV stays at 128).
// Crossing lines accumulate brightness.  Endpoints are rounded (filled circle).
void draw_line_asm_yuv422(uint8_t *fb, int fb_w, int fb_h,
                           int x0, int y0, int x1, int y1,
                           int brightness, int thickness);

// Erase all pixels that draw_line_asm_yuv422 could have touched (sets Y=0).
// 'brightness' accepted but ignored.  UV bytes left unchanged.
void undraw_line_asm_yuv422(uint8_t *fb, int fb_w, int fb_h,
                             int x0, int y0, int x1, int y1,
                             int brightness, int thickness);

#ifdef __cplusplus
}
#endif
