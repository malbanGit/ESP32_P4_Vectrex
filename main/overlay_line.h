#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "draw_line.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Anti-aliased thick line with overlay color blending.
 *
 * Identical algorithm to draw_line_asm() but each pixel's color is derived
 * by blending a pure-white line with the color sampled from `overlay` at the
 * same screen position.
 *
 * @param fb           RGB565 framebuffer to draw into
 * @param fb_w         Framebuffer width  in pixels
 * @param fb_h         Framebuffer height in pixels
 * @param x0,y0        Line start (screen coords)
 * @param x1,y1        Line end   (screen coords)
 * @param brightness   Line intensity 0-255
 * @param thickness    Stroke width in pixels (>= 1)
 * @param overlay      RGB565 overlay image, same dimensions as fb.
 *                     Pass NULL to behave identically to draw_line_asm().
 * @param shine_through Blend amount 0-255:
 *                       0   → pure white/grayscale line (overlay ignored)
 *                       255 → line takes the overlay color exactly
 *                       128 → 50/50 mix
 */
void draw_line_overlay(uint16_t *fb, int fb_w, int fb_h,
                       int x0, int y0, int x1, int y1,
                       int brightness, int thickness,
                       const uint16_t *overlay, int shine_through);

/**
 * Load a PNG file from the SD card and convert it to an RGB565 framebuffer.
 *
 * The image is scaled to exactly (fb_w × fb_h) using nearest-neighbour
 * resampling. The alpha channel (if present) is ignored.
 *
 * The SD card must already be mounted via the ESP-IDF FatFS driver before
 * calling this function. A typical mount point is "/sdcard".
 *
 * Requires lodepng (lodepng.h / lodepng.c) in the same component.
 *
 * @param path   Full path to the PNG file, e.g. "/sdcard/bg.png"
 * @param fb     Destination RGB565 buffer (must be fb_w * fb_h * 2 bytes)
 * @param fb_w   Width  of the destination buffer in pixels
 * @param fb_h   Height of the destination buffer in pixels
 * @return ESP_OK on success, ESP_FAIL on decode or I/O error
 */
esp_err_t overlay_load_png(const char *path, uint16_t *fb, int fb_w, int fb_h);

#ifdef __cplusplus
}
#endif
