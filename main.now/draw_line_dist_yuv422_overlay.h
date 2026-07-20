#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Overlay-aware distance-field line renderer for YUV422 (YUYV) framebuffers.
 *
 * Pixel layout in memory (2 bytes/pixel, pixels come in pairs):
 *   [Y0][U][Y1][V]   — Y per pixel, U/V shared between each even/odd pair.
 *
 * draw_line_yuv422_overlay_c:
 *   For each pixel in the line's glow bounding box that is covered by a
 *   non-transparent overlay palette entry:
 *     - Compute the distance-field contribution (beam core + Gaussian glow,
 *       same algorithm as draw_line_asm_yuv422).
 *     - Convert the overlay BGR palette colour to luma Y (BT.601 full-range).
 *     - Additively write  Y_dst += Y_overlay * contrib / 256  (saturating).
 *     - U/V bytes are left untouched — they were set by drawOverlayPal and
 *       carry the correct chroma for the overlay colour.
 *
 * undraw_line_yuv422_overlay_c:
 *   For each pixel in the same bounding box, restore the state that
 *   drawOverlayPal wrote:
 *     - Palette path: rewrite Y (scaled by s_overlay_alpha_val) and, at
 *       even columns, rewrite U/V from the palette BGR.
 *     - s_overlay_bg fallback: 2-byte/pixel memcpy from the background buffer.
 *     - BGRA overlay fallback: convert BGRA → YUYV with alpha scaling.
 *
 * Both functions read g_line_glow / g_line_Rb / gauss_lut from
 * draw_line_dist_rgb888.c and s_overlay_pal / s_overlay_palette / … from
 * main.c — the same externs as the RGB888 overlay variant.
 */

/* Assembly versions (active) */
void draw_line_yuv422_overlay(uint8_t *fb, int fb_w, int fb_h,
                               int x0, int y0, int x1, int y1,
                               int brightness,
                               const uint8_t *overlay);

void undraw_line_yuv422_overlay(uint8_t *fb, int fb_w, int fb_h,
                                 int x0, int y0, int x1, int y1,
                                 int brightness,
                                 const uint8_t *overlay);

/* C reference versions (for comparison / fallback) */
void draw_line_yuv422_overlay_c(uint8_t *fb, int fb_w, int fb_h,
                                 int x0, int y0, int x1, int y1,
                                 int brightness,
                                 const uint8_t *overlay);

void undraw_line_yuv422_overlay_c(uint8_t *fb, int fb_w, int fb_h,
                                   int x0, int y0, int x1, int y1,
                                   int brightness,
                                   const uint8_t *overlay);

#ifdef __cplusplus
}
#endif
