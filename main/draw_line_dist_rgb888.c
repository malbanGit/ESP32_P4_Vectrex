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

__attribute__((section(".iram0.text")))
void draw_line_asm_rgb888(uint8_t *fb, int fb_w, int fb_h,
                           int x0, int y0, int x1, int y1,
                           int brightness, int thickness)
{
    int glow = g_line_glow;
    if (!fb || fb_w <= 0 || fb_h <= 0 || brightness <= 0) return;
    if (glow < 0) glow = 0;

    /* beam radius in pixels: floor(thickness/2).
     * thickness=1 → beam_r=0 (single-pixel spine, Gaussian starts from centre).
     * thickness=2 → beam_r=1 (2-pixel solid core). */
    int beam_r = thickness >> 1;
    int R      = beam_r + glow;

    /* expanded bounding box, clamped to framebuffer */
    int bx0 = iclamp((x0 < x1 ? x0 : x1) - R, 0, fb_w - 1);
    int bx1 = iclamp((x0 > x1 ? x0 : x1) + R, 0, fb_w - 1);
    int by0 = iclamp((y0 < y1 ? y0 : y1) - R, 0, fb_h - 1);
    int by1 = iclamp((y0 > y1 ? y0 : y1) + R, 0, fb_h - 1);

    int dx   = x1 - x0, dy = y1 - y0;
    int len2 = dx * dx + dy * dy;

    /* beam core threshold in Q16 subpixel-squared units */
    int beam_r_q8   = beam_r << 8;
    int beam_r2_q16 = beam_r_q8 * beam_r_q8;

    /* Gaussian scale factor: maps glow_d2_q16 to LUT index */
    int gs = gauss_scale(glow);  /* 0 when glow==0 */

    for (int py = by0; py <= by1; py++) {
        uint8_t *row = fb + (py * fb_w) * 3;
        for (int px = bx0; px <= bx1; px++) {
            int32_t d2_q16;

            if (len2 == 0) {
                int ex_q8 = (px - x0) << 8;
                int ey_q8 = (py - y0) << 8;
                d2_q16 = (int32_t)ex_q8 * ex_q8 + (int32_t)ey_q8 * ey_q8;
            } else {
                int apx = px - x0, apy = py - y0;
                int dot = apx * dx + apy * dy;
                if (dot <= 0) {
                    /* before start cap */
                    int ex_q8 = apx << 8, ey_q8 = apy << 8;
                    d2_q16 = (int32_t)ex_q8 * ex_q8 + (int32_t)ey_q8 * ey_q8;
                } else if (dot >= len2) {
                    /* past end cap */
                    int bpx = px - x1, bpy = py - y1;
                    int ex_q8 = bpx << 8, ey_q8 = bpy << 8;
                    d2_q16 = (int32_t)ex_q8 * ex_q8 + (int32_t)ey_q8 * ey_q8;
                } else {
                    /* Perpendicular distance via cross product — no t quantization.
                     * cross = dx*apy - dy*apx; d² = cross²/len2.
                     * Within the bounding box |cross| ≤ R*sqrt(len2) so cross²
                     * fits in int32; the ×65536 shift needs int64. */
                    int cross = dx * apy - dy * apx;
                    d2_q16 = (int32_t)(((int64_t)cross * cross << 16) / len2);
                }
            }

            int contrib;
            if (d2_q16 <= beam_r2_q16) {
                contrib = brightness;
            } else {
                if (gs == 0) continue;  /* glow disabled */
                int32_t glow_d2_q16 = d2_q16 - beam_r2_q16;
                /* lut_idx = glow_d2_q16 * gs >> (GAUSS_SHIFT + 16)
                 * Use 64-bit: glow_d2_q16 max ≈ (6*256)² = 2.36M; gs max ≈ 11534
                 * product ≤ ~27B → needs int64 */
                int lut_idx = (int)(((int64_t)glow_d2_q16 * gs) >> (GAUSS_SHIFT + 16));
                if (lut_idx >= GAUSS_LUT_SIZE) continue;
                contrib = (brightness * gauss_lut[lut_idx]) >> 8;
                if (contrib == 0) continue;
            }

            int off = px * 3;
            int v   = row[off] + contrib;
            if (v > 255) {
                /* Saturation spill: spread ~25% of excess to each cardinal neighbour.
                 * All targets are within [bx0..bx1]×[by0..by1] so undraw covers them. */
                int spill = (v - 255 + 3) >> 2;
                if (spill > 0) {
                    if (px > bx0) {
                        uint8_t *p = row + (px - 1) * 3;
                        int nv = p[0] + spill; p[0] = p[1] = p[2] = nv > 255 ? 255 : (uint8_t)nv;
                    }
                    if (px < bx1) {
                        uint8_t *p = row + (px + 1) * 3;
                        int nv = p[0] + spill; p[0] = p[1] = p[2] = nv > 255 ? 255 : (uint8_t)nv;
                    }
                    if (py > by0) {
                        uint8_t *p = fb + ((py - 1) * fb_w + px) * 3;
                        int nv = p[0] + spill; p[0] = p[1] = p[2] = nv > 255 ? 255 : (uint8_t)nv;
                    }
                    if (py < by1) {
                        uint8_t *p = fb + ((py + 1) * fb_w + px) * 3;
                        int nv = p[0] + spill; p[0] = p[1] = p[2] = nv > 255 ? 255 : (uint8_t)nv;
                    }
                }
                v = 255;
            }
            row[off] = row[off+1] = row[off+2] = (uint8_t)v;
        }
    }
}

__attribute__((section(".iram0.text")))
void undraw_line_asm_rgb888(uint8_t *fb, int fb_w, int fb_h,
                             int x0, int y0, int x1, int y1,
                             int brightness, int thickness)
{
    int glow = g_line_glow;
    (void)brightness;
    if (!fb || fb_w <= 0 || fb_h <= 0) return;
    if (glow < 0) glow = 0;

    int beam_r = thickness >> 1;
    int R      = beam_r + glow;

    int bx0 = iclamp((x0 < x1 ? x0 : x1) - R, 0, fb_w - 1);
    int bx1 = iclamp((x0 > x1 ? x0 : x1) + R, 0, fb_w - 1);
    int by0 = iclamp((y0 < y1 ? y0 : y1) - R, 0, fb_h - 1);
    int by1 = iclamp((y0 > y1 ? y0 : y1) + R, 0, fb_h - 1);

    int span_bytes = (bx1 - bx0 + 1) * 3;
    for (int py = by0; py <= by1; py++) {
        memset(fb + (py * fb_w + bx0) * 3, 0, span_bytes);
    }
}
