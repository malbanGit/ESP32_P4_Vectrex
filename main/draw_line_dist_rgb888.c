/* Distance-field CRT vector line renderer for RGB888 framebuffers.
 *
 * For each pixel in bounding box:
 *   1. Project onto segment (Q8 fixed-point, no sqrt) → closest point
 *   2. Compute d² to closest point with sub-pixel precision
 *   3. Beam core (d ≤ beam_r): contrib = brightness
 *      Glow fringe (d > beam_r): contrib = brightness × Gaussian(glow_d²)
 *   4. Additive saturating write: pixel = min(255, pixel + contrib)
 *
 * Round caps are free from the segment-projection formula.
 * undraw: memset-zeros the expanded bounding box.
 *
 * 'glow' parameter controls the glow radius in pixels beyond the beam edge.
 * Gaussian is normalised so it decays to ~1/255 at exactly glow pixels out.
 * glow=0 → hard beam, no fringe.  glow=2 is a good default for thin lines.
 */

#include <stdint.h>
#include <string.h>
#include "draw_line_rgb888.h"

/* Default glow radius used by the wrapper functions (overlay_line_rgb888.c etc.)
 * when they call draw_line_asm_rgb888 without a glow argument.
 * Override from main.c or elsewhere:  g_line_glow = 3;
 */
int g_line_glow = 2;

/* Gaussian LUT: gauss_lut[i] = round(255 * exp(-i / 32))
 * Indexed by: (glow_d2_q16 * gauss_scale) >> (GAUSS_SHIFT + 16)
 * where gauss_scale is chosen per glow_r so the curve spans the full LUT.
 * Step size of LUT (in glow distance²) = 1/(gauss_scale/2^GAUSS_SHIFT) pixel².
 * See draw_gauss_scale() for how gauss_scale is computed at call time.
 */
#define GAUSS_SHIFT 14
#define GAUSS_LUT_SIZE 178

static const uint8_t gauss_lut[GAUSS_LUT_SIZE] = {
    255, 247, 240, 232, 225, 218, 211, 205, 199, 192, 187, 181, 175, 170, 165, 160,
    155, 150, 145, 141, 136, 132, 128, 124, 120, 117, 113, 110, 106, 103, 100,  97,
     94,  91,  88,  85,  83,  80,  78,  75,  73,  71,  69,  67,  64,  62,  61,  59,
     57,  55,  53,  52,  50,  49,  47,  46,  44,  43,  42,  40,  39,  38,  37,  36,
     35,  33,  32,  31,  30,  30,  29,  28,  27,  26,  25,  24,  24,  23,  22,  22,
     21,  20,  20,  19,  18,  18,  17,  17,  16,  16,  15,  15,  14,  14,  14,  13,
     13,  12,  12,  12,  11,  11,  11,  10,  10,  10,   9,   9,   9,   8,   8,   8,
      8,   7,   7,   7,   7,   7,   6,   6,   6,   6,   6,   5,   5,   5,   5,   5,
      5,   5,   4,   4,   4,   4,   4,   4,   4,   4,   3,   3,   3,   3,   3,   3,
      3,   3,   3,   3,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,
      2,   2,   2,   2,   2,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
      1,   1,
};

static inline int iclamp(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Compute LUT scale factor so that glow_d² = glow_r² maps to lut index ~177.
 * gauss_scale * glow_r2_q16 >> GAUSS_SHIFT == 177
 * gauss_scale = 177 << GAUSS_SHIFT / glow_r2_q16
 * We use a precomputed shift of glow_r²: glow_r2_q16 = glow_r² * 65536
 * → gauss_scale = 177 * 65536 / (glow_r² * 65536 >> GAUSS_SHIFT)
 *               = 177 << GAUSS_SHIFT / glow_r²
 */
static inline int gauss_scale(int glow_r)
{
    int gr2 = glow_r * glow_r;
    if (gr2 == 0) return 0;
    return (177 << GAUSS_SHIFT) / gr2;
}

/* draw_line_asm_rgb888 is implemented in draw_line_dist_rgb888.S */

/* undraw_line_asm_rgb888 is implemented in draw_line_dist_rgb888.S */
