#pragma once

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
 *   Erases beam+glow footprint to black (Y=0, U=128, V=128) using the
 *   same colorPaletteEntry and delta gates as draw — a channel is only
 *   written if draw would have touched it (non-zero contribution).
 *
 * Both functions use g_line_glow / g_line_width / g_line_Rb / gauss_lut
 * and s_overlay_palette[][3] (BGR) from the shared globals.
 */

/* Assembly-optimised versions (RISC-V, ESP32-P4) */
void draw_line_yuv422_color(uint8_t *fb, int fb_w, int fb_h,
                            int x0, int y0, int x1, int y1,
                            int colorPaletteEntry, int brightness);

void undraw_line_yuv422_color(uint8_t *fb, int fb_w, int fb_h,
                              int x0, int y0, int x1, int y1,
                              int colorPaletteEntry, int brightness);

void draw_line_yuv422_brightness(uint8_t *fb, int fb_w, int fb_h,
                                 int x0, int y0, int x1, int y1,
                                 int brightness);

void undraw_line_yuv422_brightness(uint8_t *fb, int fb_w, int fb_h,
                                   int x0, int y0, int x1, int y1,
                                   int brightness);

void draw_line_yuv422_overlay(uint8_t *fb, int fb_w, int fb_h,
                              int x0, int y0, int x1, int y1,
                              int brightness,
                              const uint8_t *overlay);

void undraw_line_yuv422_overlay(uint8_t *fb, int fb_w, int fb_h,
                                int x0, int y0, int x1, int y1,
                                int brightness,
                                const uint8_t *overlay);

/* C reference versions */
void draw_line_yuv422_color_c(uint8_t *fb, int fb_w, int fb_h,
                        int x0, int y0, int x1, int y1,
                        int colorPaletteEntry, int brightness);

void undraw_line_yuv422_color_c(uint8_t *fb, int fb_w, int fb_h,
                          int x0, int y0, int x1, int y1,
                          int colorPaletteEntry, int brightness);

/* Brightness (white) variants — UV never written (delta always 0 for white) */
void draw_line_yuv422_brightness_c(uint8_t *fb, int fb_w, int fb_h,
                                   int x0, int y0, int x1, int y1,
                                   int brightness);

void undraw_line_yuv422_brightness_c(uint8_t *fb, int fb_w, int fb_h,
                                     int x0, int y0, int x1, int y1,
                                     int brightness);

void draw_line_yuv422_overlay_c(
                                uint8_t *fb, int fb_w, int fb_h,
                                int x0, int y0, int x1, int y1,
                                int brightness,
                                const uint8_t *overlay);

void undraw_line_yuv422_overlay_c(
                                uint8_t *fb, int fb_w, int fb_h,
                                int x0, int y0, int x1, int y1,
                                int brightness,
                                const uint8_t *overlay);

#define GAUSS_LUT_SIZE 178

extern int           g_line_glow;
extern int           g_line_width;
extern int           g_line_Rb;
extern const uint8_t gauss_lut[];

extern uint8_t *s_overlay_pal;
extern uint8_t  s_overlay_palette[128][3];
extern uint8_t  s_overlay_alpha_val;
extern int      s_ov_off_x;
extern int      s_ov_off_y;
extern int      s_ov_w;
extern int      s_ov_h;

extern const uint8_t gauss_lut[GAUSS_LUT_SIZE];

IRAM_ATTR static inline uint32_t gs_for_glow(int glow_r)
{
    int gr2 = glow_r * glow_r;
    return (gr2 == 0) ? 0u : (uint32_t)((177u << 14) / (uint32_t)gr2);
}

IRAM_ATTR static inline int iclamp(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

IRAM_ATTR static inline uint32_t cap_d2_q16(int apx, int apy)
{
    uint32_t ax = (uint32_t)(apx << 8);
    uint32_t ay = (uint32_t)(apy << 8);
    return ax * ax + ay * ay;
}

IRAM_ATTR static inline int bgr_to_y(int b, int g, int r)
{
    return (77 * r + 150 * g + 29 * b) >> 8;
}

IRAM_ATTR static inline int bgr_to_u(int b, int g, int r)
{
    int u = 128 + ((-43 * r - 85 * g + 128 * b) >> 8);
    return u < 0 ? 0 : (u > 255 ? 255 : u);
}

IRAM_ATTR static inline int bgr_to_v(int b, int g, int r)
{
    int v = 128 + ((128 * r - 107 * g - 21 * b) >> 8);
    return v < 0 ? 0 : (v > 255 ? 255 : v);
}
#ifdef __cplusplus
}
#endif
