#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Anti-aliased thick line — RISC-V assembly, placed in IRAM.
 *
 * @param fb         RGB565 framebuffer (uint16_t[fb_h * fb_w])
 * @param fb_w       Framebuffer width  in pixels (e.g. 1280)
 * @param fb_h       Framebuffer height in pixels (e.g.  800)
 * @param x0, y0     Start point (screen coords)
 * @param x1, y1     End   point (screen coords)
 * @param brightness Intensity 0-255 (0 = black / erase)
 * @param thickness  Stroke width in pixels (>= 1)
 *
 * Algorithm: Xiaolin Wu anti-aliasing extended to arbitrary thickness.
 * For each column along the major axis the routine writes:
 *   - one AA pixel on the leading edge  (alpha = 255 - frac)
 *   - (thickness - 1) solid core pixels (alpha = 255)
 *   - one AA pixel on the trailing edge (alpha = frac)
 * Pixels outside [0, fb_w) × [0, fb_h) are silently clipped.
 */
void draw_line_asm(uint16_t *fb, int fb_w, int fb_h,
                   int x0, int y0, int x1, int y1,
                   int brightness, int thickness);

#ifdef __cplusplus
}
#endif
