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
void draw_line_asm_yuv422(uint8_t *fb, int fb_w, int fb_h,
                           int x0, int y0, int x1, int y1,
                           int brightness, int thickness);

#ifdef __cplusplus
}
#endif
