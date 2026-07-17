/*
 * Overlay-aware distance-field line renderer for RGB888 framebuffers.
 *
 * Mirrors the algorithm in draw_line_dist_rgb888.S but conditions each pixel
 * on the overlay's alpha channel and colours the line with the overlay's RGB.
 *
 * See draw_line_dist_rgb888_overlay.h for the full contract.
 */

#include <stdint.h>
#include <string.h>
#include "esp_attr.h"
#include "draw_line_dist_rgb888_overlay.h"

/* One overlay row cached in IRAM so the inner loop never reads PSRAM directly.
 * Sized for the maximum supported framebuffer width (1920 × 4 bytes = 7680 B).
 * Both functions run on core 0 only, so a single static buffer is safe.      */
#define OV_ROW_MAX 1920
static DRAM_ATTR uint8_t s_ov_row[OV_ROW_MAX * 4];

/* ── Shared globals ────────────────────────────────────────────────────── */
extern int           g_line_glow;          /* draw_line_dist_rgb888.c */
extern const uint8_t gauss_lut[];          /* draw_line_dist_rgb888.c */
extern int           alphaAdjust;          /* main.c                  */

/* ── Constants matching draw_line_dist_rgb888.c ────────────────────────── */
#define GAUSS_SHIFT    14
#define GAUSS_LUT_SIZE 178

/* ── Helpers ────────────────────────────────────────────────────────────── */
static inline int iclamp(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Same formula as gauss_scale() in draw_line_dist_rgb888.c:
 *   gauss_scale * glow_r2 >> GAUSS_SHIFT == 177
 *   → gauss_scale = 177 << GAUSS_SHIFT / glow_r2                         */
static inline uint32_t gs_for_glow(int glow_r)
{
    int gr2 = glow_r * glow_r;
    return (gr2 == 0) ? 0u : (uint32_t)((177u << GAUSS_SHIFT) / (uint32_t)gr2);
}

/* Effective alpha after alphaAdjust (only modifies semi-transparent pixels). */
static inline int effective_alpha(uint8_t raw_a)
{
    if (raw_a == 0 || raw_a == 255) return raw_a;
    int ea = (int)raw_a + alphaAdjust;
    return iclamp(ea, 0, 255);
}

/* Distance² in Q16 (i.e. actual_d² * 65536) for a cap endpoint.           */
static inline uint32_t cap_d2_q16(int apx, int apy)
{
    /* (apx<<8)² + (apy<<8)² — safe for |apx|,|apy| ≤ beam_r+glow ≤ ~16  */
    uint32_t ax = (uint32_t)(apx << 8);
    uint32_t ay = (uint32_t)(apy << 8);
    return ax * ax + ay * ay;
}

/* ════════════════════════════════════════════════════════════════════════
 * draw_line_rgb888_overlay
 * ════════════════════════════════════════════════════════════════════════ */
IRAM_ATTR void draw_line_rgb888_overlay(
        uint8_t *fb, int fb_w, int fb_h,
        int x0, int y0, int x1, int y1,
        int brightness, int thickness,
        const uint8_t *overlay)
{
    if (!fb || !overlay || fb_w <= 0 || fb_h <= 0 || brightness <= 0) return;

    int glow   = (g_line_glow < 0) ? 0 : g_line_glow;
    int beam_r = thickness >> 1;
    int R      = beam_r + glow;

    /* Bounding box */
    int bx0 = iclamp((x0 < x1 ? x0 : x1) - R, 0, fb_w - 1);
    int bx1 = iclamp((x0 > x1 ? x0 : x1) + R, 0, fb_w - 1);
    int by0 = iclamp((y0 < y1 ? y0 : y1) - R, 0, fb_h - 1);
    int by1 = iclamp((y0 > y1 ? y0 : y1) + R, 0, fb_h - 1);

    int dx   = x1 - x0;
    int dy   = y1 - y0;
    int len2 = dx * dx + dy * dy;

    uint32_t beam_r2_q16 = cap_d2_q16(beam_r, 0);          /* (beam_r<<8)² */
    uint32_t gs           = gs_for_glow(glow);
    uint32_t inv_len2     = (len2 > 0)
                          ? (uint32_t)(0xFFFFFFFFULL / (uint32_t)len2)
                          : 0u;

    /* Incremental cross / dot at (bx0, by0) */
    int cross_row = dx * (by0 - y0) - dy * (bx0 - x0);
    int dot_row   = (bx0 - x0) * dx + (by0 - y0) * dy;

    int ov_span = bx1 - bx0 + 1;   /* pixels to copy per row */

    for (int py = by0; py <= by1; py++) {
        int            cross   = cross_row;
        int            dot     = dot_row;
        /* Copy the needed overlay span from PSRAM into IRAM once per row. */
        memcpy(s_ov_row,
               overlay + ((size_t)py * fb_w + bx0) * 4,
               (size_t)ov_span * 4);
        uint8_t        *row_fb = fb + (size_t)py * fb_w * 3;

        for (int px = bx0; px <= bx1; px++) {
            const uint8_t *ov = s_ov_row + (px - bx0) * 4;   /* B G R A */

            /* ── Alpha gate: skip if overlay is opaque ── */
            int ea = effective_alpha(ov[3]);
            if (ea >= 255) goto px_next;

            {
                /* ── Distance² in Q16 ── */
                uint32_t d2_q16;
                if (len2 == 0 || dot <= 0) {
                    d2_q16 = cap_d2_q16(px - x0, py - y0);
                } else if (dot >= len2) {
                    d2_q16 = cap_d2_q16(px - x1, py - y1);
                } else {
                    /* interior: d2_q16 = cross² * inv_len2 >> 16 */
                    int64_t  c2   = (int64_t)cross * cross;
                    d2_q16 = (uint32_t)(((uint64_t)c2 * inv_len2) >> 16);
                }

                /* ── Beam / glow → contrib ── */
                int contrib;
                if (d2_q16 <= beam_r2_q16) {
                    contrib = brightness;
                } else if (gs == 0) {
                    goto px_next;
                } else {
                    uint32_t glow_d2  = d2_q16 - beam_r2_q16;
                    uint32_t lut_idx  = (uint32_t)(((uint64_t)glow_d2 * gs) >> 30);
                    if (lut_idx >= GAUSS_LUT_SIZE) goto px_next;
                    contrib = (brightness * (int)gauss_lut[lut_idx]) >> 8;
                    if (contrib == 0) goto px_next;
                }

                /* ── Write: dst += overlay_bgr * contrib / 255 (saturating) ── */
                uint8_t *dst = row_fb + px * 3;
                int v;
                v = dst[0] + ((ov[0] * contrib) >> 8); dst[0] = (uint8_t)(v > 255 ? 255 : v);
                v = dst[1] + ((ov[1] * contrib) >> 8); dst[1] = (uint8_t)(v > 255 ? 255 : v);
                v = dst[2] + ((ov[2] * contrib) >> 8); dst[2] = (uint8_t)(v > 255 ? 255 : v);
            }

        px_next:
            if (len2 > 0) { cross -= dy; dot += dx; }
        }

        if (len2 > 0) { cross_row += dx; dot_row += dy; }
    }
}

/* ════════════════════════════════════════════════════════════════════════
 * undraw_line_rgb888_overlay
 *
 * For each pixel in the bounding box, restore the overlay-over-black
 * rendering (destination is treated as black — no read of the framebuffer).
 * ════════════════════════════════════════════════════════════════════════ */
IRAM_ATTR void undraw_line_rgb888_overlay(
        uint8_t *fb, int fb_w, int fb_h,
        int x0, int y0, int x1, int y1,
        int brightness, int thickness,
        const uint8_t *overlay)
{
    if (!fb || !overlay || fb_w <= 0 || fb_h <= 0) return;

    int glow   = (g_line_glow < 0) ? 0 : g_line_glow;
    int beam_r = thickness >> 1;
    int R      = beam_r + glow;

    int bx0 = iclamp((x0 < x1 ? x0 : x1) - R, 0, fb_w - 1);
    int bx1 = iclamp((x0 > x1 ? x0 : x1) + R, 0, fb_w - 1);
    int by0 = iclamp((y0 < y1 ? y0 : y1) - R, 0, fb_h - 1);
    int by1 = iclamp((y0 > y1 ? y0 : y1) + R, 0, fb_h - 1);

    int ov_span = bx1 - bx0 + 1;

    for (int py = by0; py <= by1; py++) {
        memcpy(s_ov_row,
               overlay + ((size_t)py * fb_w + bx0) * 4,
               (size_t)ov_span * 4);
        uint8_t *row_fb = fb + (size_t)py * fb_w * 3;

        for (int px = bx0; px <= bx1; px++) {
            const uint8_t *ov  = s_ov_row + (px - bx0) * 4;   /* B G R A */
            uint8_t       *dst = row_fb + px * 3;
            uint8_t        a   = ov[3];

            if (a == 0) {
                /* fully transparent: restore black */
                dst[0] = dst[1] = dst[2] = 0;
            } else if (a == 255) {
                /* fully opaque: restore overlay colour directly */
                dst[0] = ov[0];
                dst[1] = ov[1];
                dst[2] = ov[2];
            } else {
                /* semi-transparent: apply alphaAdjust, render over black */
                int ea = (int)a + alphaAdjust;
                if (ea <= 0) {
                    dst[0] = dst[1] = dst[2] = 0;
                } else {
                    if (ea > 255) ea = 255;
                    dst[0] = (uint8_t)((ov[0] * ea) >> 8);
                    dst[1] = (uint8_t)((ov[1] * ea) >> 8);
                    dst[2] = (uint8_t)((ov[2] * ea) >> 8);
                }
            }
        }
    }
}
