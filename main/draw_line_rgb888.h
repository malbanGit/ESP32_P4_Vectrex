#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * RV32IMAC assembler line-drawer for RGB888 framebuffers (3 bytes/pixel).
 * Same Wu AA + thickness algorithm as draw_line_asm().
 * Bounds checks commented out; caller guarantees valid screen coords.
 */
void draw_line_asm_rgb888(uint8_t *fb, int fb_w, int fb_h,
                           int x0, int y0, int x1, int y1,
                           int brightness, int thickness);

#ifdef __cplusplus
}
#endif
