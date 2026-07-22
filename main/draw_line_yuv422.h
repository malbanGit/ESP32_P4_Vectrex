#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Palette-colour distance-field line renderer for YUV422 (YUYV) framebuffers.
 *
 * draw_line_yuv422 / draw_line_yuv422_c:
 *   Draws a line in the colour given by s_overlay_palette[colorPaletteEntry].
 *   Beam core (d <= beam_r): writes full colour × brightness / 256.
 *   Glow fringe: writes colour × brightness × Gaussian(d) / 256 / 256.
 *   Writes are additive (Y += y_contrib, U/V += delta) — same as overlay.
 *
 * undraw_line_yuv422 / undraw_line_yuv422_c:
 *   Sets every pixel in the line's beam+glow bounding box to black
 *   (Y=0, U=128, V=128) — no palette lookup, no distance test needed
 *   beyond the bounding box already used for the scan limits.
 *
 * Both functions use g_line_glow / g_line_width / g_line_Rb / gauss_lut
 * and s_overlay_palette[][3] (BGR) from the shared globals.
 */ 

/* Assembly versions */
void draw_line_yuv422(uint8_t *fb, int fb_w, int fb_h,
                      int x0, int y0, int x1, int y1,
                      int colorPaletteEntry);

void undraw_line_yuv422(uint8_t *fb, int fb_w, int fb_h,
                        int x0, int y0, int x1, int y1);

/* C reference versions */
void draw_line_yuv422_c(uint8_t *fb, int fb_w, int fb_h,
                        int x0, int y0, int x1, int y1,
                        int colorPaletteEntry);

void undraw_line_yuv422_c(uint8_t *fb, int fb_w, int fb_h,
                          int x0, int y0, int x1, int y1);

#ifdef __cplusplus
}
#endif
