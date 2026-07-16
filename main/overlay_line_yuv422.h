#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "draw_line_yuv422.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * YUV422 (YUYV, 2 bytes/pixel) overlay rendering — C reference implementations.
 * Parallel to overlay_line.h / overlay_line.c for RGB565.
 *
 * Colour space: BT.601 full-range.
 *   White beam:  Y=255, UV=128
 *   Black pixel: Y=0,   UV=128
 *
 * YUYV memory layout per row (fb_w pixels):
 *   [Y0, U01, Y1, V01,  Y2, U23, Y3, V23, ...]
 *   pixel x: Y at (y*fb_w+x)*2,  UV-byte at (y*fb_w+x)*2+1
 *   (U for even x, V for odd x — each pixel stores its own byte independently)
 */

/** Fill fb with overlay × (brightness/256).  Pass overlay=NULL for black. */
void overlay_draw_background_yuv422(uint8_t *fb, int fb_w, int fb_h,
                                    const uint8_t *overlay, int brightness);

/**
 * ADDITIVE blend in YUV space — Y saturating-add, UV chroma blend.
 * Models the real Vectrex beam shining through a coloured foil.
 */
void draw_line_vectrex_yuv422(uint8_t *fb, int fb_w, int fb_h,
                               int x0, int y0, int x1, int y1,
                               int brightness, int thickness,
                               const uint8_t *overlay, int shine_through);

/**
 * REPLACE mode — overwrites Y and UV bytes per pixel.
 * Falls back to draw_line_asm_yuv422() when shine_through <= 0.
 */
void draw_line_overlay_yuv422(uint8_t *fb, int fb_w, int fb_h,
                               int x0, int y0, int x1, int y1,
                               int brightness, int thickness,
                               const uint8_t *overlay, int shine_through);

/** Decode PNG → YUV422 YUYV buffer (BT.601 full-range, nearest-neighbour scale). */
esp_err_t overlay_load_png_yuv422(const char *path,
                                   uint8_t *fb, int fb_w, int fb_h);

#ifdef __cplusplus
}
#endif
