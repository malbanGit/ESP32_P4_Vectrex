/* Distance-field CRT vector line renderer for RGB888 framebuffers.
 *
 * Algorithm (per ChatGPT recommendation):
 *   For each pixel in bounding box:
 *     1. Project pixel onto segment, clamp t to [0,1] → closest point
 *     2. Compute d² to closest point (sub-pixel Q8 precision)
 *     3. Beam core (d ≤ beam_r): contrib = brightness (full)
 *        Glow fringe (d > beam_r): contrib = brightness * gauss(glow_d²)
 *     4. Additive saturating write: pixel = min(255, pixel + contrib)
 *
 * Round caps are free: the segment projection formula naturally handles
 * endpoints as circle caps.
 *
 * undraw: zero the entire expanded bounding box (no per-pixel tracking).
 */

#include <stdint.h>
#include <string.h>
#include "draw_line_rgb888.h"

/* Gaussian glow LUT.
 * gauss_lut[i] = round(255 * exp(-i / 32))
 * Indexed by:  (glow_d2_q16 >> 14)
 *   where glow_d2_q16 = (distance_beyond_beam_edge)² in Q16 pixel² units.
 * Step size: 2^14 = 16384 Q16 units = 0.25 pixel².
 * Effective sigma = 2 pixels (2·σ² = 8 px² → index-scale = 8·65536/16384 = 32).
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

/* Glow radius beyond beam edge (pixels). Total reach = beam_r + GLOW_R. */
#define GLOW_R 5

static inline int iclamp(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

__attribute__((section(".iram0.text")))
void draw_line_asm_rgb888(uint8_t *fb, int fb_w, int fb_h,
                           int x0, int y0, int x1, int y1,
                           int brightness, int thickness)
{
    if (!fb || fb_w <= 0 || fb_h <= 0 || brightness <= 0) return;

    /* beam radius: half of thickness, rounded up */
    int beam_r = (thickness + 1) >> 1;
    int R = beam_r + GLOW_R;

    /* expanded bounding box, clamped to framebuffer */
    int bx0 = iclamp((x0 < x1 ? x0 : x1) - R, 0, fb_w - 1);
    int bx1 = iclamp((x0 > x1 ? x0 : x1) + R, 0, fb_w - 1);
    int by0 = iclamp((y0 < y1 ? y0 : y1) - R, 0, fb_h - 1);
    int by1 = iclamp((y0 > y1 ? y0 : y1) + R, 0, fb_h - 1);

    int dx = x1 - x0, dy = y1 - y0;
    int len2 = dx * dx + dy * dy;  /* max ~(1920²+1080²) ≈ 4.8M — fits int32 */

    /* beam_r in Q8 subpixel units (for d² comparison in Q16) */
    int beam_r_q8  = beam_r << 8;                  /* beam_r * 256            */
    int beam_r2_q16 = beam_r_q8 * beam_r_q8;       /* beam_r² in Q16 units    */

    for (int py = by0; py <= by1; py++) {
        uint8_t *row = fb + (py * fb_w) * 3;
        for (int px = bx0; px <= bx1; px++) {
            int32_t d2_q16;

            if (len2 == 0) {
                /* degenerate: segment is a dot */
                int ex_q8 = (px - x0) << 8;
                int ey_q8 = (py - y0) << 8;
                d2_q16 = (int32_t)ex_q8 * ex_q8 + (int32_t)ey_q8 * ey_q8;
            } else {
                /* t = dot(AP, AB) / len2, clamped to [0, 1]
                 * We compute t in Q8: t_q8 = clamp(dot * 256 / len2, 0, 256)
                 * dot max ≈ 1920*1920 ≈ 3.7M; *256 ≈ 950M → fits int32 for
                 * screen sizes ≤ 1920. For safety we clamp before multiply. */
                int apx = px - x0, apy = py - y0;
                int dot = apx * dx + apy * dy;
                int t_q8;
                if (dot <= 0)    t_q8 = 0;
                else if (dot >= len2) t_q8 = 256;
                else             t_q8 = (dot * 256) / len2;

                /* closest point in Q8 subpixel coords */
                int cx_q8 = (x0 << 8) + t_q8 * dx;
                int cy_q8 = (y0 << 8) + t_q8 * dy;

                int ex_q8 = (px << 8) - cx_q8;
                int ey_q8 = (py << 8) - cy_q8;
                /* |ex_q8| ≤ (GLOW_R+1)*256 = 1536 → ex_q8² ≤ ~2.36M → sum ≤ ~4.7M: fits int32 */
                d2_q16 = (int32_t)ex_q8 * ex_q8 + (int32_t)ey_q8 * ey_q8;
            }

            int contrib;
            if (d2_q16 <= beam_r2_q16) {
                /* inside beam core: full brightness */
                contrib = brightness;
            } else {
                /* glow fringe: look up Gaussian */
                int32_t glow_d2_q16 = d2_q16 - beam_r2_q16;
                int lut_idx = (int)(glow_d2_q16 >> GAUSS_SHIFT);
                if (lut_idx >= GAUSS_LUT_SIZE) continue;
                contrib = (brightness * gauss_lut[lut_idx]) >> 8;
                if (contrib == 0) continue;
            }

            /* additive saturating write: R = G = B (white beam) */
            int off = px * 3;
            int v = row[off] + contrib;
            uint8_t cv = v > 255 ? 255 : (uint8_t)v;
            row[off]   = cv;
            row[off+1] = cv;
            row[off+2] = cv;
        }
    }
}

__attribute__((section(".iram0.text")))
void undraw_line_asm_rgb888(uint8_t *fb, int fb_w, int fb_h,
                             int x0, int y0, int x1, int y1,
                             int brightness, int thickness)
{
    (void)brightness;
    if (!fb || fb_w <= 0 || fb_h <= 0) return;

    int beam_r = (thickness + 1) >> 1;
    int R = beam_r + GLOW_R;

    int bx0 = iclamp((x0 < x1 ? x0 : x1) - R, 0, fb_w - 1);
    int bx1 = iclamp((x0 > x1 ? x0 : x1) + R, 0, fb_w - 1);
    int by0 = iclamp((y0 < y1 ? y0 : y1) - R, 0, fb_h - 1);
    int by1 = iclamp((y0 > y1 ? y0 : y1) + R, 0, fb_h - 1);

    int span_bytes = (bx1 - bx0 + 1) * 3;
    for (int py = by0; py <= by1; py++) {
        memset(fb + (py * fb_w + bx0) * 3, 0, span_bytes);
    }
}
