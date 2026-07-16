#pragma once
#include <stdint.h>
#include "overlay_fast_yuv422.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * RV32IMAC assembler drop-ins for overlay_fast_{erase,draw}_line_yuv422().
 * Identical signatures; intended for side-by-side benchmarking vs the C versions.
 *
 * Optimisations vs C:
 *   - Early PSRAM load issuance to overlap ~20-cycle access latency.
 *   - Y (luma): branchless saturating add.
 *   - UV (chroma): signed blend then branchless clamp to [0,255].
 *   - No function-call overhead inside the hot rasteriser loop.
 *
 * Bounds checks present as comments; callers must guarantee valid coordinates.
 */
void overlay_fast_erase_line_asm_yuv422(const overlay_ctx_yuv422_t *ctx,
                                         uint8_t *fb,
                                         int x0, int y0, int x1, int y1,
                                         int thickness);

void overlay_fast_draw_line_asm_yuv422(const overlay_ctx_yuv422_t *ctx,
                                        uint8_t *fb,
                                        int x0, int y0, int x1, int y1,
                                        int brightness, int thickness,
                                        int shine_through);

#ifdef __cplusplus
}
#endif
