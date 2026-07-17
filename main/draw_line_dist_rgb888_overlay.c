/*
 * Overlay-aware distance-field line renderer for RGB888 framebuffers.
 *
 * Two hot-path optimisations over the original BGRA-per-pixel approach:
 *
 *  DRAW  — reads 1 byte/pixel from s_overlay_pal (PSRAM, 4× denser than
 *           BGRA), then looks up BGR colour from s_overlay_palette (DRAM,
 *           stays in L1 cache).  Falls back to the original 4-byte BGRA
 *           read if s_overlay_pal is NULL.
 *
 *  UNDRAW — uses s_dirty_bits (DRAM bitfield) to restore only the pixels
 *           that were actually written during draw, skipping bounding-box
 *           pixels where glow contribution was zero.  Falls back to the
 *           full-row memcpy when s_dirty_bits is NULL.
 */

#include <stdint.h>
#include <string.h>
#include "esp_attr.h"
#include "draw_line_dist_rgb888_overlay.h"

/* ── Shared globals from draw_line_dist_rgb888.c ────────────────────────── */
extern int           g_line_glow;
extern const uint8_t gauss_lut[];

/* ── Shared globals from main.c ─────────────────────────────────────────── */
extern int      alphaAdjust;
extern uint8_t *s_overlay_bg;

/* Palettised overlay */
extern uint8_t *s_overlay_pal;          /* PSRAM: 1 byte/pixel, active region */
extern uint8_t  s_overlay_palette[128][3]; /* DRAM: BGR, ≤127 entries        */
extern int      s_overlay_pal_n;
extern uint8_t  s_overlay_alpha_val;    /* representative raw alpha           */
extern int      s_ov_off_x;
extern int      s_ov_off_y;
extern int      s_ov_w;
extern int      s_ov_h;

/* Dirty-pixel bitfield */
extern uint8_t *s_dirty_bits;           /* DRAM: 1 bit/pixel, active region   */

/* ── Constants ──────────────────────────────────────────────────────────── */
#define GAUSS_SHIFT    14
#define GAUSS_LUT_SIZE 178

/* ── Helpers ────────────────────────────────────────────────────────────── */
static inline int iclamp(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline uint32_t gs_for_glow(int glow_r)
{
    int gr2 = glow_r * glow_r;
    return (gr2 == 0) ? 0u : (uint32_t)((177u << GAUSS_SHIFT) / (uint32_t)gr2);
}

static inline int effective_alpha(uint8_t raw_a)
{
    if (raw_a == 0 || raw_a == 255) return raw_a;
    int ea = (int)raw_a + alphaAdjust;
    return iclamp(ea, 0, 255);
}

static inline uint32_t cap_d2_q16(int apx, int apy)
{
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
    if (!fb || fb_w <= 0 || fb_h <= 0 || brightness <= 0) return;
    if (!overlay && !s_overlay_pal) return;

    int glow   = (g_line_glow < 0) ? 0 : g_line_glow;
    int beam_r = thickness >> 1;
    int R      = beam_r + glow;

    int bx0 = iclamp((x0 < x1 ? x0 : x1) - R, 0, fb_w - 1);
    int bx1 = iclamp((x0 > x1 ? x0 : x1) + R, 0, fb_w - 1);
    int by0 = iclamp((y0 < y1 ? y0 : y1) - R, 0, fb_h - 1);
    int by1 = iclamp((y0 > y1 ? y0 : y1) + R, 0, fb_h - 1);

    int dx   = x1 - x0;
    int dy   = y1 - y0;
    int len2 = dx * dx + dy * dy;

    uint32_t beam_r2_q16 = cap_d2_q16(beam_r, 0);
    uint32_t gs           = gs_for_glow(glow);
    uint32_t inv_len2     = (len2 > 0)
                          ? (uint32_t)(0xFFFFFFFFULL / (uint32_t)len2)
                          : 0u;

    int cross_row = dx * (by0 - y0) - dy * (bx0 - x0);
    int dot_row   = (bx0 - x0) * dx + (by0 - y0) * dy;

    /* Use palette path when available. */
    const int use_pal = (s_overlay_pal != NULL);

    /* Pre-check global alpha (palette path only). */
    if (use_pal) {
        int ea = effective_alpha(s_overlay_alpha_val);
        if (ea >= 255) return; /* whole overlay is opaque — nothing to draw */
    }

    for (int py = by0; py <= by1; py++) {
        int cross = cross_row;
        int dot   = dot_row;

        /* Row pointers for palette and dirty-bits paths. */
        int            ry         = py - s_ov_off_y;
        const uint8_t *row_pal    = NULL;
        int            dirty_base = -1;

        if (use_pal && (unsigned)ry < (unsigned)s_ov_h) {
            row_pal    = s_overlay_pal + (size_t)ry * s_ov_w - s_ov_off_x;
            dirty_base = ry * s_ov_w;  /* bit_idx = dirty_base + rx */
        }

        /* Fallback: original BGRA overlay row. */
        const uint8_t *row_ov = (!use_pal && overlay)
                              ? overlay + ((size_t)py * fb_w + bx0) * 4
                              : NULL;

        uint8_t *row_fb = fb + (size_t)py * fb_w * 3;

        for (int px = bx0; px <= bx1; px++) {

            /* ── Pixel colour + skip logic ─────────────────────────────── */
            int b_col, g_col, r_col;

            if (row_pal) {
                unsigned rx = (unsigned)(px - s_ov_off_x);
                if (rx >= (unsigned)s_ov_w) goto px_next;
                uint8_t pidx = row_pal[px];     /* 1 PSRAM byte */
                if (!(pidx & 0x80)) goto px_next;
                const uint8_t *c = s_overlay_palette[pidx & 0x7F]; /* DRAM */
                b_col = c[0]; g_col = c[1]; r_col = c[2];
            } else {
                const uint8_t *ov = row_ov + (px - bx0) * 4; /* 4 PSRAM bytes */
                int ea = effective_alpha(ov[3]);
                if (ea >= 255) goto px_next;
                b_col = ov[0]; g_col = ov[1]; r_col = ov[2];
            }

            /* ── Distance-field contribution ───────────────────────────── */
            {
                uint32_t d2_q16;
                if (len2 == 0 || dot <= 0) {
                    d2_q16 = cap_d2_q16(px - x0, py - y0);
                } else if (dot >= len2) {
                    d2_q16 = cap_d2_q16(px - x1, py - y1);
                } else {
                    int64_t c2 = (int64_t)cross * cross;
                    d2_q16 = (uint32_t)(((uint64_t)c2 * inv_len2) >> 16);
                }

                int contrib;
                if (d2_q16 <= beam_r2_q16) {
                    contrib = brightness;
                } else if (gs == 0) {
                    goto px_next;
                } else {
                    uint32_t glow_d2 = d2_q16 - beam_r2_q16;
                    uint32_t lut_idx = (uint32_t)(((uint64_t)glow_d2 * gs) >> 30);
                    if (lut_idx >= GAUSS_LUT_SIZE) goto px_next;
                    contrib = (brightness * (int)gauss_lut[lut_idx]) >> 8;
                    if (contrib == 0) goto px_next;
                }

                uint8_t *dst = row_fb + px * 3;
                int v;
                v = dst[0] + ((b_col * contrib) >> 8); dst[0] = (uint8_t)(v > 255 ? 255 : v);
                v = dst[1] + ((g_col * contrib) >> 8); dst[1] = (uint8_t)(v > 255 ? 255 : v);
                v = dst[2] + ((r_col * contrib) >> 8); dst[2] = (uint8_t)(v > 255 ? 255 : v);

                /* Mark pixel dirty (DRAM bit op — fast). */
                if (s_dirty_bits && dirty_base >= 0) {
                    int rx2 = px - s_ov_off_x;
                    int bit_idx = dirty_base + rx2;
                    s_dirty_bits[bit_idx >> 3] |= (uint8_t)(1u << (bit_idx & 7));
                }
            }

        px_next:
            if (len2 > 0) { cross -= dy; dot += dx; }
        }

        if (len2 > 0) { cross_row += dx; dot_row += dy; }
    }
}

/* ════════════════════════════════════════════════════════════════════════
 * undraw_line_rgb888_overlay
 * ════════════════════════════════════════════════════════════════════════ */
IRAM_ATTR void undraw_line_rgb888_overlay(
        uint8_t *fb, int fb_w, int fb_h,
        int x0, int y0, int x1, int y1,
        int brightness, int thickness,
        const uint8_t *overlay)
{
    if (!fb || fb_w <= 0 || fb_h <= 0) return;

    int glow   = (g_line_glow < 0) ? 0 : g_line_glow;
    int beam_r = thickness >> 1;
    int R      = beam_r + glow;

    int bx0 = iclamp((x0 < x1 ? x0 : x1) - R, 0, fb_w - 1);
    int bx1 = iclamp((x0 > x1 ? x0 : x1) + R, 0, fb_w - 1);
    int by0 = iclamp((y0 < y1 ? y0 : y1) - R, 0, fb_h - 1);
    int by1 = iclamp((y0 > y1 ? y0 : y1) + R, 0, fb_h - 1);

    if (s_dirty_bits && s_overlay_pal && s_overlay_bg) {
        for (int py = by0; py <= by1; py++) {
            int ry = py - s_ov_off_y;
            if ((unsigned)ry >= (unsigned)s_ov_h) continue;
            int            dirty_base = ry * s_ov_w;
            uint8_t       *row_fb    = fb           + (size_t)py * fb_w * 3;
            const uint8_t *row_bg    = s_overlay_bg + (size_t)py * fb_w * 3;
            for (int px = bx0; px <= bx1; px++) {
                int rx = px - s_ov_off_x;
                if ((unsigned)rx >= (unsigned)s_ov_w) continue;
                int     bit_idx = dirty_base + rx;
                uint8_t *byte   = &s_dirty_bits[bit_idx >> 3];
                uint8_t  mask   = (uint8_t)(1u << (bit_idx & 7));
                if (!(*byte & mask)) continue;
                *byte &= ~mask;
                const uint8_t *src = row_bg  + px * 3;
                uint8_t       *dst = row_fb  + px * 3;
                dst[0] = src[0]; dst[1] = src[1]; dst[2] = src[2];
            }
        }
    } else 
        
    if (s_overlay_bg) {
        /* Fast fallback: one memcpy per row, full bounding-box width. */
        int span3 = (bx1 - bx0 + 1) * 3;
        for (int py = by0; py <= by1; py++) {
            memcpy(fb           + ((size_t)py * fb_w + bx0) * 3,
                   s_overlay_bg + ((size_t)py * fb_w + bx0) * 3,
                   (size_t)span3);
        }
    } else if (overlay) {
        /* Fallback: compute from BGRA overlay (alphaAdjust applied). */
        for (int py = by0; py <= by1; py++) {
            const uint8_t *row_ov = overlay + ((size_t)py * fb_w + bx0) * 4;
            uint8_t       *row_fb = fb      + (size_t)py * fb_w * 3;
            for (int px = bx0; px <= bx1; px++) {
                const uint8_t *ov  = row_ov + (px - bx0) * 4;
                uint8_t       *dst = row_fb + px * 3;
                uint8_t        a   = ov[3];
                if (a == 0) {
                    dst[0] = dst[1] = dst[2] = 0;
                } else if (a == 255) {
                    dst[0] = ov[0]; dst[1] = ov[1]; dst[2] = ov[2];
                } else {
                    int ea = (int)a + alphaAdjust;
                    if (ea <= 0) { dst[0] = dst[1] = dst[2] = 0; }
                    else {
                        if (ea > 255) ea = 255;
                        dst[0] = (uint8_t)((ov[0] * ea) >> 8);
                        dst[1] = (uint8_t)((ov[1] * ea) >> 8);
                        dst[2] = (uint8_t)((ov[2] * ea) >> 8);
                    }
                }
            }
        }
    }
}
