#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "draw_line_rgb888.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * RGB888 (3 bytes/pixel) overlay rendering — C reference implementations.
 * Parallel to overlay_line.h / overlay_line.c for RGB565.
 *
 * The overlay image is a uint8_t array of fb_w * fb_h * 3 bytes, packed R,G,B.
 */

/** Fill fb with overlay × (brightness/256).  Pass overlay=NULL for black. */
void overlay_draw_background_rgb888(uint8_t *fb, int fb_w, int fb_h,
                                    const uint8_t *overlay, int brightness);

/**
 * ADDITIVE blend — beam saturating-adds onto existing fb pixels.
 * Models the real Vectrex beam shining through a transparent coloured foil.
 */
void draw_line_vectrex_rgb888(uint8_t *fb, int fb_w, int fb_h,
                               int x0, int y0, int x1, int y1,
                               int brightness, int thickness,
                               const uint8_t *overlay, int shine_through);

/**
 * REPLACE mode — overwrites pixels (use to build an overlay layer).
 * Falls back to draw_line_asm_rgb888() when shine_through <= 0.
 */
void draw_line_overlay_rgb888(uint8_t *fb, int fb_w, int fb_h,
                               int x0, int y0, int x1, int y1,
                               int brightness, int thickness,
                               const uint8_t *overlay, int shine_through);

/** Decode PNG at path into an RGB888 buffer (nearest-neighbour scale to fb dims). */
esp_err_t overlay_load_png_rgb888(const char *path,
                                   uint8_t *fb, int fb_w, int fb_h);

#ifdef __cplusplus
}
#endif
