#pragma once
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Fast Vectrex overlay rendering — avoids full-frame background redraws.
 *
 * PROBLEM WITH THE NAIVE APPROACH
 * --------------------------------
 * overlay_draw_background() touches every one of the 1,024,000 pixels each
 * frame: at 50 fps that is ~100 MB/s of PSRAM traffic just for the
 * background, before a single line is drawn — right at the ESP32-P4 limit.
 *
 * FAST APPROACH
 * -------------
 * Precompute  dimmed[i] = overlay[i] × bg_brightness / 256  once.
 * Per frame, only touch the pixels that lines actually cross:
 *
 *   Erase old line → copy from dimmed[]  (one 16-bit store, no arithmetic)
 *   Draw  new line → write dimmed[px] + beam_color directly
 *                    (no FB read required; background value is always known)
 *
 * For a typical Vectrex frame with 50 lines × 200 pixels ≈ 10 000 pixels,
 * per-frame PSRAM traffic drops from ~2 MB to ~40 KB — a 50× reduction.
 *
 * USAGE
 * -----
 *   // Once at startup (after loading the overlay PNG):
 *   overlay_ctx_t ctx;
 *   overlay_fast_init(&ctx, overlay_fb, LCD_H_RES, LCD_V_RES, 50);
 *
 *   // Each frame — mirrors the existing undraw/draw pattern in main.c:
 *   // Erase previous lines (restore background under each old line)
 *   for (int i = 0; i < old_count; i++)
 *       overlay_fast_erase_line(&ctx, fb_back,
 *           old[i].x0, old[i].y0, old[i].x1, old[i].y1, old[i].thickness);
 *
 *   // Draw new lines
 *   for (int i = 0; i < new_count; i++)
 *       overlay_fast_draw_line(&ctx, fb_back,
 *           new[i].x0, new[i].y0, new[i].x1, new[i].y1,
 *           new[i].brightness, new[i].thickness, shine_through);
 */

typedef struct {
    uint16_t       *dimmed;   /* pre-scaled overlay (heap, PSRAM) */
    const uint16_t *overlay;  /* original overlay image            */
    int             fb_w;
    int             fb_h;
} overlay_ctx_t;

/**
 * Initialise the context and pre-compute the dimmed background buffer.
 *
 * Allocates  fb_w × fb_h × 2  bytes from the PSRAM heap.
 * Call once after overlay_load_png().
 *
 * @param ctx          Context to initialise
 * @param overlay      RGB565 overlay image (must remain valid for ctx lifetime)
 * @param fb_w / fb_h  Framebuffer dimensions in pixels
 * @param bg_brightness  Background brightness 0-255 (typical: 20-80)
 */
esp_err_t overlay_fast_init(overlay_ctx_t *ctx,
                             const uint16_t *overlay, int fb_w, int fb_h,
                             int bg_brightness);

/** Free the dimmed buffer and zero the context. */
void overlay_fast_free(overlay_ctx_t *ctx);

/**
 * Recompute the dimmed buffer with a new background brightness.
 * Call whenever the ambient light level changes.
 */
void overlay_fast_set_brightness(overlay_ctx_t *ctx, int bg_brightness);

/**
 * Erase a previously drawn line by restoring background pixels.
 *
 * Call with the SAME geometry (x0,y0,x1,y1,thickness) that was used when
 * the line was drawn so that exactly the same pixels are restored.
 * Brightness is not required — all potentially affected pixels are restored
 * regardless of their AA alpha value (restoring background is idempotent).
 *
 * @param ctx        Initialised overlay context
 * @param fb         Framebuffer that contains the line to erase
 * @param x0,y0      Line start
 * @param x1,y1      Line end
 * @param thickness  Stroke width used when the line was drawn
 */
void overlay_fast_erase_line(const overlay_ctx_t *ctx, uint16_t *fb,
                              int x0, int y0, int x1, int y1,
                              int thickness);

/**
 * Draw a Vectrex-style anti-aliased thick line onto the framebuffer.
 *
 * Each pixel is written as:
 *   beam_rgb  = lerp(white, overlay_rgb_at_pixel, shine_through/255)
 *   beam_rgb  = beam_rgb × (AA_alpha × brightness / 256) / 256
 *   fb[pixel] = saturating_add(dimmed[pixel], beam_rgb)
 *
 * All colour arithmetic stays in 5/6-bit RGB565 component space to avoid
 * expensive expand-to-8-bit / repack roundtrips.
 *
 * @param ctx          Initialised overlay context
 * @param fb           Framebuffer to draw into
 * @param x0,y0        Line start
 * @param x1,y1        Line end
 * @param brightness   Beam intensity 0-255
 * @param thickness    Stroke width in pixels (>= 1)
 * @param shine_through  0 = white beam, 255 = overlay colour, 128 = 50/50
 */
void overlay_fast_draw_line(const overlay_ctx_t *ctx, uint16_t *fb,
                             int x0, int y0, int x1, int y1,
                             int brightness, int thickness,
                             int shine_through);

#ifdef __cplusplus
}
#endif
