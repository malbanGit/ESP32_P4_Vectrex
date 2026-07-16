#pragma once
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Fast YUV422 (YUYV) overlay context — pre-scaled background avoids full-frame redraws.
 * Drop-in parallel to overlay_fast.h for RGB565.
 *
 * Memory layout: same YUYV packing as the framebuffer (2 bytes/pixel).
 * Y luma is scaled by brightness; UV chroma is pulled toward neutral (128) proportionally.
 */
typedef struct {
    uint8_t       *dimmed;   /**< pre-scaled overlay (PSRAM, fb_w*fb_h*2 bytes) */
    const uint8_t *overlay;  /**< original YUV422 overlay image                  */
    int            fb_w;
    int            fb_h;
} overlay_ctx_yuv422_t;

/** Allocate dimmed buffer and pre-compute it.  Call once after overlay_load_png_yuv422(). */
esp_err_t overlay_fast_init_yuv422(overlay_ctx_yuv422_t *ctx,
                                    const uint8_t *overlay, int fb_w, int fb_h,
                                    int bg_brightness);

void overlay_fast_free_yuv422(overlay_ctx_yuv422_t *ctx);

/** Recompute dimmed with a new brightness level. */
void overlay_fast_set_brightness_yuv422(overlay_ctx_yuv422_t *ctx, int bg_brightness);

/** Restore pixels under an old line back to the dimmed background. */
void overlay_fast_erase_line_yuv422(const overlay_ctx_yuv422_t *ctx, uint8_t *fb,
                                     int x0, int y0, int x1, int y1,
                                     int thickness);

/** Draw a beam line: Y saturating-add, UV chroma blend, over the dimmed background. */
void overlay_fast_draw_line_yuv422(const overlay_ctx_yuv422_t *ctx, uint8_t *fb,
                                    int x0, int y0, int x1, int y1,
                                    int brightness, int thickness,
                                    int shine_through);

#ifdef __cplusplus
}
#endif
