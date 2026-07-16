#pragma once
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Fast RGB888 overlay context — pre-scaled background avoids full-frame redraws.
 * Drop-in parallel to overlay_fast.h for RGB565.
 *
 * dimmed[i*3..i*3+2] = overlay[i*3..i*3+2] × bg_brightness / 256  (computed once).
 * Per frame only the pixels crossed by lines are touched (~50× less PSRAM traffic).
 */
typedef struct {
    uint8_t       *dimmed;   /**< pre-scaled overlay (PSRAM, fb_w*fb_h*3 bytes) */
    const uint8_t *overlay;  /**< original RGB888 overlay image                  */
    int            fb_w;
    int            fb_h;
} overlay_ctx_rgb888_t;

/** Allocate dimmed buffer and pre-compute it.  Call once after overlay_load_png_rgb888(). */
esp_err_t overlay_fast_init_rgb888(overlay_ctx_rgb888_t *ctx,
                                    const uint8_t *overlay, int fb_w, int fb_h,
                                    int bg_brightness);

void overlay_fast_free_rgb888(overlay_ctx_rgb888_t *ctx);

/** Recompute dimmed with a new brightness level. */
void overlay_fast_set_brightness_rgb888(overlay_ctx_rgb888_t *ctx, int bg_brightness);

/** Restore pixels under an old line back to the dimmed background. */
void overlay_fast_erase_line_rgb888(const overlay_ctx_rgb888_t *ctx, uint8_t *fb,
                                     int x0, int y0, int x1, int y1,
                                     int thickness);

/** Draw a beam line: saturating-add over the dimmed background. */
void overlay_fast_draw_line_rgb888(const overlay_ctx_rgb888_t *ctx, uint8_t *fb,
                                    int x0, int y0, int x1, int y1,
                                    int brightness, int thickness,
                                    int shine_through);

#ifdef __cplusplus
}
#endif
