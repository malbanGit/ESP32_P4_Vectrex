#pragma once
#include <stdint.h>
#include "overlay_fast.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * RISC-V (RV32IMAC) assembler implementations of the fast overlay line
 * functions. Drop-in replacements for overlay_fast_erase_line() and
 * overlay_fast_draw_line() — identical signatures and semantics, intended
 * for side-by-side benchmarking.
 *
 * Bounds checks are present in source as comments but not executed;
 * callers must pass valid coordinates.
 */

void overlay_fast_erase_line_asm(const overlay_ctx_t *ctx, uint16_t *fb,
                                  int x0, int y0, int x1, int y1,
                                  int thickness);

void overlay_fast_draw_line_asm(const overlay_ctx_t *ctx, uint16_t *fb,
                                 int x0, int y0, int x1, int y1,
                                 int brightness, int thickness,
                                 int shine_through);

#ifdef __cplusplus
}
#endif
