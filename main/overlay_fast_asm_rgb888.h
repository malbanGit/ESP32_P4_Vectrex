#pragma once
#include <stdint.h>
#include "overlay_fast_rgb888.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * RV32IMAC assembler drop-ins for overlay_fast_{erase,draw}_line_rgb888().
 * Identical signatures; intended for side-by-side benchmarking vs the C versions.
 *
 * Optimisations vs C:
 *   - Both PSRAM loads (dimmed + overlay) issued before arithmetic to hide latency.
 *   - Colour math in native 8-bit; no expand/pack roundtrips.
 *   - Branchless 8-bit saturation per channel (srli/neg/or/andi).
 *   - No function-call overhead inside the hot rasteriser loop.
 *
 * Bounds checks present as comments; callers must guarantee valid coordinates.
 */
void overlay_fast_erase_line_asm_rgb888(const overlay_ctx_rgb888_t *ctx,
                                         uint8_t *fb,
                                         int x0, int y0, int x1, int y1,
                                         int thickness);

void overlay_fast_draw_line_asm_rgb888(const overlay_ctx_rgb888_t *ctx,
                                        uint8_t *fb,
                                        int x0, int y0, int x1, int y1,
                                        int brightness, int thickness,
                                        int shine_through);

#ifdef __cplusplus
}
#endif
