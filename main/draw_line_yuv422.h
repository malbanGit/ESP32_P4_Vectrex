#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
/* Gaussian LUT: gauss_lut[i] = round(255 * exp(-i / 32))
 * Indexed by: (glow_d2_q16 * gauss_scale) >> (GAUSS_SHIFT + 16)
 * where gauss_scale is chosen per glow_r so the curve spans the full LUT.
 * Step size of LUT (in glow distance²) = 1/(gauss_scale/2^GAUSS_SHIFT) pixel².
 * See draw_gauss_scale() for how gauss_scale is computed at call time.
 */
#define GAUSS_SHIFT 14
#define GAUSS_LUT_SIZE 178

/*
 * YUV422 (YUYV) distance-field line renderers.
 * All functions use g_line_Rb / g_beam_r / g_beam_r2_q16 / g_gs / gauss_lut.
 *
 * _color variants:   fixed palette colour from s_overlay_palette[colorPaletteEntry].
 * _brightness variants: white line (R=G=B=255); UV never written (delta always 0).
 * _overlay variants:  overlay-palette draw/undraw (brightness + overlay pointer).
 */

/* Assembly — fixed palette colour */
void draw_line_yuv422_color(uint8_t *fb, int fb_w, int fb_h,
                            int x0, int y0, int x1, int y1,
                            int colorPaletteEntry);

void undraw_line_yuv422_color(uint8_t *fb, int fb_w, int fb_h,
                              int x0, int y0, int x1, int y1,
                              int colorPaletteEntry);

/* Assembly — brightness (white, UV never written) */
void draw_line_yuv422_brightness(uint8_t *fb, int fb_w, int fb_h,
                                 int x0, int y0, int x1, int y1,
                                 int brightness);

void undraw_line_yuv422_brightness(uint8_t *fb, int fb_w, int fb_h,
                                   int x0, int y0, int x1, int y1,
                                   int brightness);

/* Assembly — overlay palette */
void draw_line_yuv422_overlay(uint8_t *fb, int fb_w, int fb_h,
                              int x0, int y0, int x1, int y1,
                              int brightness, void *overlay);

void undraw_line_yuv422_overlay(uint8_t *fb, int fb_w, int fb_h,
                                int x0, int y0, int x1, int y1,
                                int brightness, void *overlay);

/* C reference — overlay palette */
void draw_line_yuv422_overlay_c(uint8_t *fb, int fb_w, int fb_h,
                                int x0, int y0, int x1, int y1,
                                int brightness, const uint8_t *overlay);

void undraw_line_yuv422_overlay_c(uint8_t *fb, int fb_w, int fb_h,
                                  int x0, int y0, int x1, int y1,
                                  int brightness, const uint8_t *overlay);

/* C reference — fixed palette colour */
void draw_line_yuv422_color_c(uint8_t *fb, int fb_w, int fb_h,
                              int x0, int y0, int x1, int y1,
                              int colorPaletteEntry);

void undraw_line_yuv422_color_c(uint8_t *fb, int fb_w, int fb_h,
                                int x0, int y0, int x1, int y1,
                                int colorPaletteEntry);

/* C reference — brightness (white) */
void draw_line_yuv422_brightness_c(uint8_t *fb, int fb_w, int fb_h,
                                   int x0, int y0, int x1, int y1,
                                   int brightness);

void undraw_line_yuv422_brightness_c(uint8_t *fb, int fb_w, int fb_h,
                                     int x0, int y0, int x1, int y1,
                                     int brightness);

#ifdef __cplusplus
}
#endif
