/*
 * Overlay-aware distance-field line renderer for YUV422 (YUYV) framebuffers.
 *
 * Direct port of draw_line_dist_rgb888_overlay.c for the YUV422 pixel format:
 *
 *   Pixel layout: [Y0][U][Y1][V] per pixel pair (2 bytes/pixel).
 *   Y byte for pixel px : fb[py * fb_w * 2 + px * 2]
 *   U byte (even pair)  : fb[py * fb_w * 2 + (px & ~1) * 2 + 1]
 *   V byte (even pair)  : fb[py * fb_w * 2 + (px & ~1) * 2 + 3]
 *
 *  DRAW  — reads 1 byte/pixel from s_overlay_pal, looks up BGR from
 *           s_overlay_palette, converts to luma Y (BT.601), and additively
 *           blends Y onto the Y byte.  U/V are left untouched because
 *           drawOverlayPal already wrote the correct chroma for each pixel.
 *
 *  UNDRAW — palette path restores Y and (at even columns) U/V to the values
 *            drawOverlayPal would have written.  Fallbacks handle the
 *            s_overlay_bg buffer (2 bytes/pixel memcpy) and the raw BGRA
 *            overlay (converted to YUYV on the fly).
 */

#include <stdint.h>
#include <string.h>
#include "esp_attr.h"
#include "draw_line_dist_yuv422_overlay.h"

/* ── Shared globals from draw_line_dist_rgb888.c ────────────────────────── */
extern int           g_line_glow;
extern int           g_line_width;
extern int           g_line_Rb;
extern const uint8_t gauss_lut[];

/* ── Shared globals from main.c ─────────────────────────────────────────── */
extern uint8_t *s_overlay_bg;

/* Palettised overlay */
extern uint8_t *s_overlay_pal;             /* PSRAM: 1 byte/pixel, active region */
extern uint8_t  s_overlay_palette[128][3]; /* DRAM: BGR, ≤127 entries            */
extern int      s_overlay_pal_n;
extern uint8_t  s_overlay_alpha_val;       /* representative raw alpha            */
extern int      s_ov_off_x;
extern int      s_ov_off_y;
extern int      s_ov_w;
extern int      s_ov_h;

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

static inline uint32_t cap_d2_q16(int apx, int apy)
{
    uint32_t ax = (uint32_t)(apx << 8);
    uint32_t ay = (uint32_t)(apy << 8);
    return ax * ax + ay * ay;
}

/* BT.601 full-range BGR → Y  (77+150+29 = 256, so Y ∈ [0,255]) */
static inline int bgr_to_y(int b, int g, int r)
{
    return (77 * r + 150 * g + 29 * b) >> 8;
}

/* BT.601 full-range BGR → U (Cb), clamped to [0,255] */
static inline int bgr_to_u(int b, int g, int r)
{
    int u = 128 + ((-43 * r - 85 * g + 128 * b) >> 8);
    return u < 0 ? 0 : (u > 255 ? 255 : u);
}

/* BT.601 full-range BGR → V (Cr), clamped to [0,255] */
static inline int bgr_to_v(int b, int g, int r)
{
    int v = 128 + ((128 * r - 107 * g - 21 * b) >> 8);
    return v < 0 ? 0 : (v > 255 ? 255 : v);
}

/* ════════════════════════════════════════════════════════════════════════
 * draw_line_yuv422_overlay_c
 * ════════════════════════════════════════════════════════════════════════ */
IRAM_ATTR void draw_line_yuv422_overlay_c(
        uint8_t *fb, int fb_w, int fb_h,
        int x0, int y0, int x1, int y1,
        int brightness,
        const uint8_t *overlay)
{
    if (!fb || fb_w <= 0 || fb_h <= 0 || brightness <= 0) return;
    if (!overlay && !s_overlay_pal) return;

    int glow   = (g_line_glow < 0) ? 0 : g_line_glow;
    int beam_r = g_line_width >> 1;
    int Rb     = g_line_Rb;

    int bx0 = iclamp((x0 < x1 ? x0 : x1) - Rb, 0, fb_w - 1);
    int bx1 = iclamp((x0 > x1 ? x0 : x1) + Rb, 0, fb_w - 1);
    int by0 = iclamp((y0 < y1 ? y0 : y1) - Rb, 0, fb_h - 1);
    int by1 = iclamp((y0 > y1 ? y0 : y1) + Rb, 0, fb_h - 1);

    int wx0 = (x0 < x1 ? x0 : x1) - Rb;
    int wx1 = (x0 > x1 ? x0 : x1) + Rb;
    int wy0 = (y0 < y1 ? y0 : y1) - Rb;
    int wy1 = (y0 > y1 ? y0 : y1) + Rb;

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

    const int use_pal = (s_overlay_pal != NULL);

    /* Pre-check global alpha (palette path only). */
    if (use_pal) {
        if (s_overlay_alpha_val >= 255) return; /* whole overlay is opaque */
    }

    for (int py = by0; py <= by1; py++) {
        int cross = cross_row;
        int dot   = dot_row;

        int            ry      = py - s_ov_off_y;
        const uint8_t *row_pal = NULL;

        if (use_pal && (unsigned)ry < (unsigned)s_ov_h) {
            row_pal = s_overlay_pal + (size_t)ry * s_ov_w - s_ov_off_x;
        }

        /* Fallback: original BGRA overlay row. */
        const uint8_t *row_ov = (!use_pal && overlay)
                              ? overlay + ((size_t)py * fb_w + bx0) * 4
                              : NULL;

        /* YUV422: 2 bytes/pixel */
        uint8_t *row_fb = fb + (size_t)py * fb_w * 2;

        for (int px = bx0; px <= bx1; px++) {

            /* ── Pixel colour + skip logic ──────────────────────────────── */
            int b_col, g_col, r_col;

            if (row_pal) {
                unsigned rx = (unsigned)(px - s_ov_off_x);
                if (rx >= (unsigned)s_ov_w) goto px_next;
                uint8_t pidx = row_pal[px];
                if (!(pidx & 0x80)) goto px_next; /* transparent — skip */
                const uint8_t *c = s_overlay_palette[pidx & 0x7F];
                b_col = c[0]; g_col = c[1]; r_col = c[2];
            } else {
                const uint8_t *ov = row_ov + (px - bx0) * 4;
                if (s_overlay_alpha_val >= 255) goto px_next; /* opaque — skip */
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

                if (px < wx0 || px > wx1 || py < wy0 || py > wy1) goto px_next;

                /* Mirror the RGB additive blend: RGB adds c[i]*contrib/256 to
                 * the alpha-reduced base, driving the effective color toward full
                 * saturation at the beam core (blend = ea + contrib → 256).
                 * In YUV we do the same via a combined blend factor:
                 *   Y: additive (handles overlapping lines correctly)
                 *   U/V: recomputed from blend-scaled BGR at even columns so
                 *        chroma tracks the same saturation curve as RGB. */
                int blend = (int)s_overlay_alpha_val + contrib;
                if (blend > 256) blend = 256;
                int b_sc = (b_col * blend) >> 8;
                int g_sc = (g_col * blend) >> 8;
                int r_sc = (r_col * blend) >> 8;

                uint8_t *dst = row_fb + px * 2;
                int yv = bgr_to_y(b_sc, g_sc, r_sc);
                dst[0] = (uint8_t)(yv > 255 ? 255 : yv);
                if (!(px & 1)) {
                    dst[1] = (uint8_t)bgr_to_u(b_sc, g_sc, r_sc);
                    dst[3] = (uint8_t)bgr_to_v(b_sc, g_sc, r_sc);
                }
            }

        px_next:
            if (len2 > 0) { cross -= dy; dot += dx; }
        }

        if (len2 > 0) { cross_row += dx; dot_row += dy; }
    }
}

/* ════════════════════════════════════════════════════════════════════════
 * undraw_line_yuv422_overlay_c
 * ════════════════════════════════════════════════════════════════════════ */
IRAM_ATTR void undraw_line_yuv422_overlay_c(
        uint8_t *fb, int fb_w, int fb_h,
        int x0, int y0, int x1, int y1,
        int brightness,
        const uint8_t *overlay)
{
    if (!fb || fb_w <= 0 || fb_h <= 0) return;

    int Rb = g_line_Rb;

    int bx0 = iclamp((x0 < x1 ? x0 : x1) - Rb, 0, fb_w - 1);
    int bx1 = iclamp((x0 > x1 ? x0 : x1) + Rb, 0, fb_w - 1);
    int by0 = iclamp((y0 < y1 ? y0 : y1) - Rb, 0, fb_h - 1);
    int by1 = iclamp((y0 > y1 ? y0 : y1) + Rb, 0, fb_h - 1);

    int wx0 = (x0 < x1 ? x0 : x1) - Rb;
    int wx1 = (x0 > x1 ? x0 : x1) + Rb;
    int wy0 = (y0 < y1 ? y0 : y1) - Rb;
    int wy1 = (y0 > y1 ? y0 : y1) + Rb;

    if (s_overlay_pal) {
        /* Palette path: restore Y and (at even columns) U/V to the values
         * that drawOverlayPal wrote.  Transparent pixels are skipped — draw
         * never touched them, so no restoration is needed. */
        int ea = (int)s_overlay_alpha_val;
        for (int py = by0; py <= by1; py++) {
            int ry = py - s_ov_off_y;
            if ((unsigned)ry >= (unsigned)s_ov_h) continue;
            const uint8_t *row_pal = s_overlay_pal + (size_t)ry * s_ov_w - s_ov_off_x;
            uint8_t       *row_fb  = fb + (size_t)py * fb_w * 2;

            for (int px = bx0; px <= bx1; px++) {
                unsigned rx = (unsigned)(px - s_ov_off_x);
                if (rx >= (unsigned)s_ov_w) continue;
                uint8_t pidx = row_pal[px];
                if (!(pidx & 0x80)) continue;                         /* transparent */
                if (px < wx0 || px > wx1 || py < wy0 || py > wy1) continue;

                const uint8_t *c = s_overlay_palette[pidx & 0x7F];

                /* Scale by ea exactly as drawOverlayPal does for bit-7-set pixels,
                 * then derive Y/U/V from the same scaled values so the restored
                 * pixel matches what drawOverlayPal originally wrote. */
                int b_sc = (c[0] * ea) >> 8;
                int g_sc = (c[1] * ea) >> 8;
                int r_sc = (c[2] * ea) >> 8;

                uint8_t *dst = row_fb + px * 2;
                dst[0] = (uint8_t)bgr_to_y(b_sc, g_sc, r_sc);

                /* Restore U/V only at even columns (one pair covers two pixels) */
                if (!(px & 1)) {
                    dst[1] = (uint8_t)bgr_to_u(b_sc, g_sc, r_sc);
                    dst[3] = (uint8_t)bgr_to_v(b_sc, g_sc, r_sc);
                }
            }
        }
    } else if (s_overlay_bg) {
        /* Fallback: full bounding-box copy from background buffer (YUV422,
         * 2 bytes/pixel — must have been allocated and filled accordingly). */
        int span2 = (bx1 - bx0 + 1) * 2;
        for (int py = by0; py <= by1; py++) {
            memcpy(fb           + ((size_t)py * fb_w + bx0) * 2,
                   s_overlay_bg + ((size_t)py * fb_w + bx0) * 2,
                   (size_t)span2);
        }
    } else if (overlay) {
        /* Fallback: convert BGRA overlay → YUYV on the fly. */
        for (int py = by0; py <= by1; py++) {
            const uint8_t *row_ov = overlay + ((size_t)py * fb_w + bx0) * 4;
            uint8_t       *row_fb = fb      + (size_t)py * fb_w * 2;

            for (int px = bx0; px <= bx1; px++) {
                const uint8_t *ov  = row_ov + (px - bx0) * 4;
                uint8_t       *dst = row_fb + px * 2;
                uint8_t        a   = ov[3];
                int b = ov[0], g = ov[1], r = ov[2];

                if (a == 0) {
                    dst[0] = 0; /* Y = black */
                    if (!(px & 1)) { dst[1] = 128; dst[3] = 128; } /* neutral chroma */
                } else if (a == 255) {
                    dst[0] = (uint8_t)bgr_to_y(b, g, r);
                    if (!(px & 1)) {
                        dst[1] = (uint8_t)bgr_to_u(b, g, r);
                        dst[3] = (uint8_t)bgr_to_v(b, g, r);
                    }
                } else {
                    int ea = (int)s_overlay_alpha_val;
                    if (ea <= 0) {
                        dst[0] = 0;
                        if (!(px & 1)) { dst[1] = 128; dst[3] = 128; }
                    } else {
                        if (ea > 255) ea = 255;
                        int b_sc = (b * ea) >> 8;
                        int g_sc = (g * ea) >> 8;
                        int r_sc = (r * ea) >> 8;
                        dst[0] = (uint8_t)bgr_to_y(b_sc, g_sc, r_sc);
                        if (!(px & 1)) {
                            dst[1] = (uint8_t)bgr_to_u(b_sc, g_sc, r_sc);
                            dst[3] = (uint8_t)bgr_to_v(b_sc, g_sc, r_sc);
                        }
                    }
                }
            }
        }
    }
}
