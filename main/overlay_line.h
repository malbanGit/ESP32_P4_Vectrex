#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "draw_line.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// Overlay image management
// ---------------------------------------------------------------------------

/**
 * Load a PNG file from the SD card and convert it to an RGB565 framebuffer.
 *
 * The image is nearest-neighbour scaled to exactly (fb_w × fb_h).
 * The alpha channel, if present, is ignored.
 * The SD card must already be mounted (e.g. via esp_vfs_fat_sdmmc_mount).
 *
 * Requires lodepng.h / lodepng.c placed in the same component (main/).
 *
 * @param path   Full path, e.g. "/sdcard/overlay.png"
 * @param fb     Destination RGB565 buffer  (fb_w * fb_h * sizeof(uint16_t))
 * @param fb_w   Buffer width  in pixels
 * @param fb_h   Buffer height in pixels
 * @return ESP_OK on success, ESP_FAIL on I/O or decode error
 */
esp_err_t overlay_load_png(const char *path, uint16_t *fb, int fb_w, int fb_h);

// ---------------------------------------------------------------------------
// Background pass  (call once per frame, before drawing lines)
// ---------------------------------------------------------------------------

/**
 * Render the overlay image into the framebuffer at reduced brightness.
 *
 * Simulates the real Vectrex foil: ambient room light shines through the
 * printed transparency, making the artwork faintly visible against the black
 * CRT even where no beam is active.
 *
 * Call this at the start of each frame to paint the "background", then draw
 * lines with draw_line_vectrex() on top.
 *
 * @param fb          RGB565 framebuffer to paint into
 * @param fb_w        Framebuffer width  in pixels
 * @param fb_h        Framebuffer height in pixels
 * @param overlay     Source overlay image, same dimensions as fb
 * @param brightness  0 = black (overlay invisible)
 *                    255 = overlay at full intensity
 *                    Typical range: 20–80 for a realistic foil glow
 */
void overlay_draw_background(uint16_t *fb, int fb_w, int fb_h,
                              const uint16_t *overlay, int brightness);

// ---------------------------------------------------------------------------
// Line drawing
// ---------------------------------------------------------------------------

/**
 * Vectrex-style anti-aliased thick line through an overlay.
 *
 * Models the real hardware: the electron beam (white line) shines through
 * the coloured overlay foil and ADDS its light to whatever is already in
 * the framebuffer.  Call overlay_draw_background() first so the foil artwork
 * is already in the buffer, then call this for every emulated line.
 *
 * Blend model per pixel:
 *   beam_color = lerp(white, overlay_color_at_pixel, shine_through/255)
 *   beam_color is scaled by the AA coverage × brightness
 *   final_pixel = saturating_add(existing_fb_pixel, beam_color)
 *
 * @param fb           RGB565 framebuffer (already contains background)
 * @param fb_w         Framebuffer width  in pixels
 * @param fb_h         Framebuffer height in pixels
 * @param x0,y0        Line start (screen coords)
 * @param x1,y1        Line end   (screen coords)
 * @param brightness   Beam intensity 0-255
 * @param thickness    Stroke width in pixels (>= 1)
 * @param overlay      Overlay RGB565 image, same dimensions as fb.
 *                     Pass NULL to draw a plain white/grey line additively.
 * @param shine_through Colour blending 0-255:
 *                       0   → pure white beam (overlay not used for colour)
 *                       255 → beam takes the overlay colour exactly
 *                       128 → 50/50 mix
 */
void draw_line_vectrex(uint16_t *fb, int fb_w, int fb_h,
                       int x0, int y0, int x1, int y1,
                       int brightness, int thickness,
                       const uint16_t *overlay, int shine_through);

/**
 * Anti-aliased thick line with overlay colour blending — REPLACE mode.
 *
 * Same algorithm as draw_line_vectrex() but OVERWRITES the framebuffer
 * pixel instead of adding to it.  Useful when drawing directly onto a black
 * background without a prior overlay_draw_background() pass.
 *
 * Pass overlay=NULL / shine_through=0 to fall back to draw_line_asm().
 */
void draw_line_overlay(uint16_t *fb, int fb_w, int fb_h,
                       int x0, int y0, int x1, int y1,
                       int brightness, int thickness,
                       const uint16_t *overlay, int shine_through);

#ifdef __cplusplus
}
#endif
