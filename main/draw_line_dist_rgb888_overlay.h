#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Overlay-aware distance-field line renderer for RGB888 framebuffers.
 *
 * The overlay buffer is BGRA (4 bytes/pixel, same layout as s_overlay in main.c).
 *
 * draw_line_rgb888_overlay:
 *   For each pixel in the line's glow bounding box:
 *     - If overlay alpha (after alphaAdjust) == 255: skip — overlay is opaque,
 *       line is hidden behind it.
 *     - Otherwise: compute the distance-field contribution (beam core + Gaussian
 *       glow, same as draw_line_asm_rgb888), then additively write
 *           dst += overlay_bgr * contrib / 255   (saturating per channel)
 *       This colours the line with the overlay's hue at the current pixel.
 *
 * undraw_line_rgb888_overlay:
 *   For each pixel in the same bounding box, restore the overlay-over-black
 *   rendering (no blending with the existing framebuffer):
 *     - alpha == 0   → write black  (0, 0, 0)
 *     - alpha == 255 → write overlay BGR directly
 *     - otherwise    → effective_alpha = clamp(alpha + alphaAdjust, 0, 255)
 *                      write overlay_bgr * effective_alpha / 255
 *
 * Both functions read g_line_glow and gauss_lut from draw_line_dist_rgb888.c,
 * and alphaAdjust from main.c.
 *
 * overlay: pointer to the full-screen BGRA buffer (fb_w * fb_h * 4 bytes).
 *          Pass NULL to behave identically to the plain rgb888 variants.
 */

/* Assembly versions (active) */
void draw_line_rgb888_overlay(uint8_t *fb, int fb_w, int fb_h,
                               int x0, int y0, int x1, int y1,
                               int brightness, int thickness,
                               const uint8_t *overlay);

void undraw_line_rgb888_overlay(uint8_t *fb, int fb_w, int fb_h,
                                 int x0, int y0, int x1, int y1,
                                 int brightness, int thickness,
                                 const uint8_t *overlay);

/* C reference versions (for comparison / fallback) */
void draw_line_rgb888_overlay_c(uint8_t *fb, int fb_w, int fb_h,
                                 int x0, int y0, int x1, int y1,
                                 int brightness, int thickness,
                                 const uint8_t *overlay);

void undraw_line_rgb888_overlay_c(uint8_t *fb, int fb_w, int fb_h,
                                   int x0, int y0, int x1, int y1,
                                   int brightness, int thickness,
                                   const uint8_t *overlay);

#ifdef __cplusplus
}
#endif
