#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Distance-field CRT vector line renderer for RGB888 framebuffers.
 * Additive Gaussian glow, round caps, crossing lines accumulate brightness.
 *
 * g_line_glow: glow radius in pixels used by wrapper layers (default 2).
 *   0 = hard beam only.  Typical CRT look: 2–4.
 *   Change at runtime: g_line_glow = 3;
 */
extern int g_line_glow;
// Gaussian-glow additive line (reads framebuffer, saturating-adds Gaussian beam).
// Crossing lines accumulate brightness.  Endpoints are rounded (filled circle).
// glow: glow radius in pixels beyond beam edge (0 = hard beam, 2 = typical CRT glow).
void draw_line_asm_rgb888(uint8_t *fb, int fb_w, int fb_h,
                           int x0, int y0, int x1, int y1,
                           int brightness, int thickness, int glow);

// Erase all pixels that draw_line_asm_rgb888 could have touched (sets R=G=B=0).
// brightness ignored.  glow must match the value used during draw.
void undraw_line_asm_rgb888(uint8_t *fb, int fb_w, int fb_h,
                             int x0, int y0, int x1, int y1,
                             int brightness, int thickness, int glow);

#ifdef __cplusplus
}
#endif
