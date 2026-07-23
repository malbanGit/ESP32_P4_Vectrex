/*
 * YUV422 (YUYV) distance-field line renderers — C reference implementations.
 * Covers overlay, fixed-palette colour, and brightness (white) variants.
 *
 * Optimisations applied:
 *  1. len2==0 dot case handled before main loops (no per-pixel beqz len2).
 *  2. beam_r2_q16 and gs loaded from precomputed globals g_beam_r2_q16 / g_gs.
 *  3. y-loop split into narrow (ady>0) and full-range bodies (constant per call).
 */

#include <stdint.h>
#include <stddef.h>

#include "esp_attr.h"
#include "defines.h"
#include "draw_line_yuv422.h"

 const DRAM_ATTR uint8_t gauss_lut[GAUSS_LUT_SIZE] = {
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


extern int           g_line_glow;
extern int           g_line_width;
extern int           g_line_Rb;

extern uint8_t *s_overlay_pal;
extern uint8_t  s_overlay_palette[128][3];
extern uint8_t  s_overlay_alpha_val;
extern int      s_ov_off_x;
extern int      s_ov_off_y;
extern int      s_ov_w;
extern int      s_ov_h;

#define GAUSS_LUT_SIZE 178

static inline int iclamp(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline uint32_t cap_d2_q16(int apx, int apy)
{
    uint32_t ax = (uint32_t)(apx << 8);
    uint32_t ay = (uint32_t)(apy << 8);
    return ax * ax + ay * ay;
}

static inline int bgr_to_y(int b, int g, int r)
{
    return (77 * r + 150 * g + 29 * b) >> 8;
}

static inline int bgr_to_u(int b, int g, int r)
{
    int u = 128 + ((-43 * r - 85 * g + 128 * b) >> 8);
    return u < 0 ? 0 : (u > 255 ? 255 : u);
}

static inline int bgr_to_v(int b, int g, int r)
{
    int v = 128 + ((128 * r - 107 * g - 21 * b) >> 8);
    return v < 0 ? 0 : (v > 255 ? 255 : v);
}

/* shared pixel write for draw */
#define WRITE_YUV_DRAW(dst, px, b_col, g_col, r_col, contrib) do { \
    int y_contrib = (bgr_to_y(b_col, g_col, r_col) * (contrib)) >> 8; \
    if (y_contrib) { \
        int yv = (dst)[0] + y_contrib; \
        (dst)[0] = (uint8_t)(yv > 255 ? 255 : yv); \
    } \
    int u_delta = ((bgr_to_u(b_col, g_col, r_col) - 128) * (contrib)) >> 8; \
    int v_delta = ((bgr_to_v(b_col, g_col, r_col) - 128) * (contrib)) >> 8; \
    if (u_delta | v_delta) { \
        int u_off = ((px) & 1) ? -1 : 1; \
        int uv    = (int)(dst)[u_off]     + u_delta; \
        int vv    = (int)(dst)[u_off + 2] + v_delta; \
        (dst)[u_off]     = (uint8_t)(uv < 0 ? 0 : uv > 255 ? 255 : uv); \
        (dst)[u_off + 2] = (uint8_t)(vv < 0 ? 0 : vv > 255 ? 255 : vv); \
    } \
} while(0)

/* shared pixel write for undraw — mirrors draw: compute deltas, gate on them, write */
#define WRITE_YUV_UNDRAW(dst, px, b_col, g_col, r_col, ea) do { \
    int b_sc    = ((b_col) * (ea)) >> 8; \
    int g_sc    = ((g_col) * (ea)) >> 8; \
    int r_sc    = ((r_col) * (ea)) >> 8; \
    int y_val   = bgr_to_y(b_sc, g_sc, r_sc); \
    if (y_val) (dst)[0] = (uint8_t)y_val; \
    int u_delta = ((bgr_to_u(b_col, g_col, r_col) - 128) * (ea)) >> 8; \
    int v_delta = ((bgr_to_v(b_col, g_col, r_col) - 128) * (ea)) >> 8; \
    if (u_delta | v_delta) { \
        int u_off = ((px) & 1) ? -1 : 1; \
        (dst)[u_off]     = (uint8_t)(128 + u_delta); \
        (dst)[u_off + 2] = (uint8_t)(128 + v_delta); \
    } \
} while(0)

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
    if (s_overlay_alpha_val >= 255) return;

    /* Opt 2: load precomputed globals */
    int      beam_r      = g_beam_r;
    uint32_t beam_r2_q16 = g_beam_r2_q16;
    uint32_t gs          = g_gs;
    int      Rb          = g_line_Rb;

    int bx0 = iclamp((x0 < x1 ? x0 : x1) - Rb, 0, fb_w - 1);
    int bx1 = iclamp((x0 > x1 ? x0 : x1) + Rb, 0, fb_w - 1);
    int by0 = iclamp((y0 < y1 ? y0 : y1) - Rb, 0, fb_h - 1);
    int by1 = iclamp((y0 > y1 ? y0 : y1) + Rb, 0, fb_h - 1);

    int dx   = x1 - x0;
    int dy   = y1 - y0;
    int len2 = dx * dx + dy * dy;

    int px_lo_g = bx0 > s_ov_off_x ? bx0 : s_ov_off_x;
    int px_hi_g = s_ov_off_x + s_ov_w - 1; if (px_hi_g > bx1) px_hi_g = bx1;

    /* Opt 1: degenerate dot */
    if (len2 == 0) {
        for (int py = by0; py <= by1; py++) {
            int ry = py - s_ov_off_y;
            if ((unsigned)ry >= (unsigned)s_ov_h) continue;
            const uint8_t *row_pal = s_overlay_pal + (size_t)ry * s_ov_w - s_ov_off_x;
            uint8_t *row_fb = fb + (size_t)py * fb_w * 2;
            for (int px = px_lo_g; px <= px_hi_g; px++) {
                uint8_t pidx = row_pal[px];
                if (!(pidx & 0x80)) continue;
                const uint8_t *c = s_overlay_palette[pidx & 0x7F];
                int b_col = c[0], g_col = c[1], r_col = c[2];
                uint32_t d2_q16 = cap_d2_q16(px - x0, py - y0);
                int contrib;
                if (d2_q16 <= beam_r2_q16) {
                    contrib = brightness;
                } else if (gs == 0) {
                    continue;
                } else {
                    uint32_t glow_d2 = d2_q16 - beam_r2_q16;
                    uint32_t lut_idx = (uint32_t)(((uint64_t)glow_d2 * gs) >> 30);
                    if (lut_idx >= GAUSS_LUT_SIZE) continue;
                    contrib = (brightness * (int)gauss_lut[lut_idx]) >> 8;
                    if (contrib == 0) continue;
                }
                uint8_t *dst = row_fb + px * 2;
                WRITE_YUV_DRAW(dst, px, b_col, g_col, r_col, contrib);
            }
        }
        return;
    }

    /* len2 > 0 from here */
    uint32_t beam_r2_len2 = (uint32_t)beam_r * beam_r * (uint32_t)len2;
    uint32_t inv_len2     = (uint32_t)(0xFFFFFFFFULL / (uint32_t)len2);

    int cross_row = dx * (by0 - y0) - dy * (bx0 - x0);
    int dot_row   = (bx0 - x0) * dx + (by0 - y0) * dy;

    int ady = dy < 0 ? -dy : dy;
    int adx = dx < 0 ? -dx : dx;
    /* Opt 3: hoist narrow-vs-full-range branch out of y-loop */
    if (ady > 0) {
        int half_w = Rb * (adx + ady) / ady + 1;
        for (int py = by0; py <= by1; py++) {
            int ry = py - s_ov_off_y;
            if ((unsigned)ry >= (unsigned)s_ov_h) {
                cross_row += dx; dot_row += dy;
                continue;
            }
            const uint8_t *row_pal = s_overlay_pal + (size_t)ry * s_ov_w - s_ov_off_x;

            int px_c = bx0 + cross_row / dy;
            int px_lo_r = px_c - half_w; if (px_lo_r < px_lo_g) px_lo_r = px_lo_g;
            int px_hi_r = px_c + half_w; if (px_hi_r > px_hi_g) px_hi_r = px_hi_g;
            if (px_lo_r > px_hi_r) { cross_row += dx; dot_row += dy; continue; }

            int dd    = px_lo_r - bx0;
            int cross = cross_row - dy * dd;
            int dot   = dot_row   + dx * dd;
            uint8_t *row_fb = fb + (size_t)py * fb_w * 2;

            for (int px = px_lo_r; px <= px_hi_r; px++) {
                uint8_t pidx = row_pal[px];
                if (!(pidx & 0x80)) goto ov_px_next_n;
                {
                    const uint8_t *c = s_overlay_palette[pidx & 0x7F];
                    int b_col = c[0], g_col = c[1], r_col = c[2];
                    uint32_t d2_q16 = 0;
                    int in_beam = 0;
                    if (dot <= 0) {
                        d2_q16  = cap_d2_q16(px - x0, py - y0);
                        in_beam = (d2_q16 <= beam_r2_q16);
                    } else if (dot >= len2) {
                        d2_q16  = cap_d2_q16(px - x1, py - y1);
                        in_beam = (d2_q16 <= beam_r2_q16);
                    } else {
                        uint64_t csq64 = (uint64_t)((int64_t)cross * cross);
                        if (csq64 > 0xFFFFFFFFu) goto ov_px_next_n;
                        uint32_t csq = (uint32_t)csq64;
                        if (csq <= beam_r2_len2) {
                            in_beam = 1;
                        } else {
                            d2_q16 = (uint32_t)(((uint64_t)csq * inv_len2) >> 16);
                        }
                    }
                    int contrib;
                    if (in_beam) {
                        contrib = brightness;
                    } else if (gs == 0) {
                        goto ov_px_next_n;
                    } else {
                        uint32_t glow_d2 = d2_q16 - beam_r2_q16;
                        uint32_t lut_idx = (uint32_t)(((uint64_t)glow_d2 * gs) >> 30);
                        if (lut_idx >= GAUSS_LUT_SIZE) goto ov_px_next_n;
                        contrib = (brightness * (int)gauss_lut[lut_idx]) >> 8;
                        if (contrib == 0) goto ov_px_next_n;
                    }
                    {
                        uint8_t *dst = row_fb + px * 2;
                        WRITE_YUV_DRAW(dst, px, b_col, g_col, r_col, contrib);
                    }
                }
            ov_px_next_n:
                cross -= dy; dot += dx;
            }
            cross_row += dx; dot_row += dy;
        }
    } else {
        /* ady == 0: full x-range every row */
        for (int py = by0; py <= by1; py++) {
            int ry = py - s_ov_off_y;
            if ((unsigned)ry >= (unsigned)s_ov_h) {
                cross_row += dx; dot_row += dy;
                continue;
            }
            const uint8_t *row_pal = s_overlay_pal + (size_t)ry * s_ov_w - s_ov_off_x;
            if (px_lo_g > px_hi_g) { cross_row += dx; dot_row += dy; continue; }

            int dd    = px_lo_g - bx0;
            int cross = cross_row - dy * dd;
            int dot   = dot_row   + dx * dd;
            uint8_t *row_fb = fb + (size_t)py * fb_w * 2;

            for (int px = px_lo_g; px <= px_hi_g; px++) {
                uint8_t pidx = row_pal[px];
                if (!(pidx & 0x80)) goto ov_px_next_f;
                {
                    const uint8_t *c = s_overlay_palette[pidx & 0x7F];
                    int b_col = c[0], g_col = c[1], r_col = c[2];
                    uint32_t d2_q16 = 0;
                    int in_beam = 0;
                    if (dot <= 0) {
                        d2_q16  = cap_d2_q16(px - x0, py - y0);
                        in_beam = (d2_q16 <= beam_r2_q16);
                    } else if (dot >= len2) {
                        d2_q16  = cap_d2_q16(px - x1, py - y1);
                        in_beam = (d2_q16 <= beam_r2_q16);
                    } else {
                        uint64_t csq64 = (uint64_t)((int64_t)cross * cross);
                        if (csq64 > 0xFFFFFFFFu) goto ov_px_next_f;
                        uint32_t csq = (uint32_t)csq64;
                        if (csq <= beam_r2_len2) {
                            in_beam = 1;
                        } else {
                            d2_q16 = (uint32_t)(((uint64_t)csq * inv_len2) >> 16);
                        }
                    }
                    int contrib;
                    if (in_beam) {
                        contrib = brightness;
                    } else if (gs == 0) {
                        goto ov_px_next_f;
                    } else {
                        uint32_t glow_d2 = d2_q16 - beam_r2_q16;
                        uint32_t lut_idx = (uint32_t)(((uint64_t)glow_d2 * gs) >> 30);
                        if (lut_idx >= GAUSS_LUT_SIZE) goto ov_px_next_f;
                        contrib = (brightness * (int)gauss_lut[lut_idx]) >> 8;
                        if (contrib == 0) goto ov_px_next_f;
                    }
                    {
                        uint8_t *dst = row_fb + px * 2;
                        WRITE_YUV_DRAW(dst, px, b_col, g_col, r_col, contrib);
                    }
                }
            ov_px_next_f:
                cross -= dy; dot += dx;
            }
            cross_row += dx; dot_row += dy;
        }
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

    /* Opt 2: load precomputed globals */
    int      beam_r      = g_beam_r;
    uint32_t beam_r2_q16 = g_beam_r2_q16;
    uint32_t gs          = g_gs;
    int      Rb          = g_line_Rb;

    int bx0 = iclamp((x0 < x1 ? x0 : x1) - Rb, 0, fb_w - 1);
    int bx1 = iclamp((x0 > x1 ? x0 : x1) + Rb, 0, fb_w - 1);
    int by0 = iclamp((y0 < y1 ? y0 : y1) - Rb, 0, fb_h - 1);
    int by1 = iclamp((y0 > y1 ? y0 : y1) + Rb, 0, fb_h - 1);

    int dx   = x1 - x0;
    int dy   = y1 - y0;
    int len2 = dx * dx + dy * dy;

    int ea = (int)s_overlay_alpha_val;

    int ud_px_lo_g = bx0 > s_ov_off_x ? bx0 : s_ov_off_x;
    int ud_px_hi_g = s_ov_off_x + s_ov_w - 1; if (ud_px_hi_g > bx1) ud_px_hi_g = bx1;

    /* Opt 1: degenerate dot */
    if (len2 == 0) {
        for (int py = by0; py <= by1; py++) {
            int ry = py - s_ov_off_y;
            if ((unsigned)ry >= (unsigned)s_ov_h) continue;
            const uint8_t *row_pal = s_overlay_pal + (size_t)ry * s_ov_w - s_ov_off_x;
            uint8_t *row_fb = fb + (size_t)py * fb_w * 2;
            for (int px = ud_px_lo_g; px <= ud_px_hi_g; px++) {
                uint32_t d2_q16 = cap_d2_q16(px - x0, py - y0);
                int in_beam = (d2_q16 <= beam_r2_q16);
                if (!in_beam) {
                    if (gs == 0) continue;
                    uint32_t glow_d2 = d2_q16 - beam_r2_q16;
                    uint32_t lut_idx = (uint32_t)(((uint64_t)glow_d2 * gs) >> 30);
                    if (lut_idx >= GAUSS_LUT_SIZE) continue;
                    if (gauss_lut[lut_idx] == 0) continue;
                }
                uint8_t pidx = row_pal[px];
                if (pidx & 0x80) {
                    const uint8_t *c = s_overlay_palette[pidx & 0x7F];
                    uint8_t *dst = row_fb + px * 2;
                    WRITE_YUV_UNDRAW(dst, px, c[0], c[1], c[2], ea);
                }
            }
        }
        return;
    }

    /* len2 > 0 from here */
    uint32_t beam_r2_len2 = (uint32_t)beam_r * beam_r * (uint32_t)len2;
    uint32_t inv_len2     = (uint32_t)(0xFFFFFFFFULL / (uint32_t)len2);

    int cross_row = dx * (by0 - y0) - dy * (bx0 - x0);
    int dot_row   = (bx0 - x0) * dx + (by0 - y0) * dy;

    int udy = dy < 0 ? -dy : dy;
    int udx = dx < 0 ? -dx : dx;
    /* Opt 3: hoist narrow-vs-full-range branch out of y-loop */
    if (udy > 0) {
        int ud_half_w = Rb * (udx + udy) / udy + 1;
        for (int py = by0; py <= by1; py++) {
            int ry = py - s_ov_off_y;
            if ((unsigned)ry >= (unsigned)s_ov_h) {
                cross_row += dx; dot_row += dy;
                continue;
            }
            const uint8_t *row_pal = s_overlay_pal + (size_t)ry * s_ov_w - s_ov_off_x;

            int px_c = bx0 + cross_row / dy;
            int px_lo_r = px_c - ud_half_w; if (px_lo_r < ud_px_lo_g) px_lo_r = ud_px_lo_g;
            int px_hi_r = px_c + ud_half_w; if (px_hi_r > ud_px_hi_g) px_hi_r = ud_px_hi_g;
            if (px_lo_r > px_hi_r) { cross_row += dx; dot_row += dy; continue; }

            int udd   = px_lo_r - bx0;
            int cross = cross_row - dy * udd;
            int dot   = dot_row   + dx * udd;
            uint8_t *row_fb = fb + (size_t)py * fb_w * 2;

            for (int px = px_lo_r; px <= px_hi_r; px++) {
                uint32_t d2_q16 = 0;
                int in_beam = 0;
                if (dot <= 0) {
                    d2_q16  = cap_d2_q16(px - x0, py - y0);
                    in_beam = (d2_q16 <= beam_r2_q16);
                } else if (dot >= len2) {
                    d2_q16  = cap_d2_q16(px - x1, py - y1);
                    in_beam = (d2_q16 <= beam_r2_q16);
                } else {
                    uint64_t csq64 = (uint64_t)((int64_t)cross * cross);
                    if (csq64 > 0xFFFFFFFFu) goto uov_px_next_n;
                    uint32_t csq = (uint32_t)csq64;
                    if (csq <= beam_r2_len2) {
                        in_beam = 1;
                    } else {
                        d2_q16 = (uint32_t)(((uint64_t)csq * inv_len2) >> 16);
                    }
                }
                if (!in_beam) {
                    if (gs == 0) goto uov_px_next_n;
                    uint32_t glow_d2 = d2_q16 - beam_r2_q16;
                    uint32_t lut_idx = (uint32_t)(((uint64_t)glow_d2 * gs) >> 30);
                    if (lut_idx >= GAUSS_LUT_SIZE) goto uov_px_next_n;
                    if (gauss_lut[lut_idx] == 0) goto uov_px_next_n;
                }
                {
                    uint8_t pidx = row_pal[px];
                    if (pidx & 0x80) {
                        const uint8_t *c = s_overlay_palette[pidx & 0x7F];
                        uint8_t *dst = row_fb + px * 2;
                        WRITE_YUV_UNDRAW(dst, px, c[0], c[1], c[2], ea);
                    }
                }
            uov_px_next_n:
                cross -= dy; dot += dx;
            }
            cross_row += dx; dot_row += dy;
        }
    } else {
        /* udy == 0: full x-range every row */
        for (int py = by0; py <= by1; py++) {
            int ry = py - s_ov_off_y;
            if ((unsigned)ry >= (unsigned)s_ov_h) {
                cross_row += dx; dot_row += dy;
                continue;
            }
            const uint8_t *row_pal = s_overlay_pal + (size_t)ry * s_ov_w - s_ov_off_x;
            if (ud_px_lo_g > ud_px_hi_g) { cross_row += dx; dot_row += dy; continue; }

            int udd   = ud_px_lo_g - bx0;
            int cross = cross_row - dy * udd;
            int dot   = dot_row   + dx * udd;
            uint8_t *row_fb = fb + (size_t)py * fb_w * 2;

            for (int px = ud_px_lo_g; px <= ud_px_hi_g; px++) {
                uint32_t d2_q16 = 0;
                int in_beam = 0;
                if (dot <= 0) {
                    d2_q16  = cap_d2_q16(px - x0, py - y0);
                    in_beam = (d2_q16 <= beam_r2_q16);
                } else if (dot >= len2) {
                    d2_q16  = cap_d2_q16(px - x1, py - y1);
                    in_beam = (d2_q16 <= beam_r2_q16);
                } else {
                    uint64_t csq64 = (uint64_t)((int64_t)cross * cross);
                    if (csq64 > 0xFFFFFFFFu) goto uov_px_next_f;
                    uint32_t csq = (uint32_t)csq64;
                    if (csq <= beam_r2_len2) {
                        in_beam = 1;
                    } else {
                        d2_q16 = (uint32_t)(((uint64_t)csq * inv_len2) >> 16);
                    }
                }
                if (!in_beam) {
                    if (gs == 0) goto uov_px_next_f;
                    uint32_t glow_d2 = d2_q16 - beam_r2_q16;
                    uint32_t lut_idx = (uint32_t)(((uint64_t)glow_d2 * gs) >> 30);
                    if (lut_idx >= GAUSS_LUT_SIZE) goto uov_px_next_f;
                    if (gauss_lut[lut_idx] == 0) goto uov_px_next_f;
                }
                {
                    uint8_t pidx = row_pal[px];
                    if (pidx & 0x80) {
                        const uint8_t *c = s_overlay_palette[pidx & 0x7F];
                        uint8_t *dst = row_fb + px * 2;
                        WRITE_YUV_UNDRAW(dst, px, c[0], c[1], c[2], ea);
                    }
                }
            uov_px_next_f:
                cross -= dy; dot += dx;
            }
            cross_row += dx; dot_row += dy;
        }
    }
}

/* ════════════════════════════════════════════════════════════════════════
 * draw_line_yuv422_color_c
 * ════════════════════════════════════════════════════════════════════════ */
IRAM_ATTR void draw_line_yuv422_color_c(
        uint8_t *fb, int fb_w, int fb_h,
        int x0, int y0, int x1, int y1,
        int colorPaletteEntry)
{
    if (!fb || fb_w <= 0 || fb_h <= 0) return;

    int brightness = 255;

    /* Opt 2: load precomputed globals */
    int      beam_r      = g_beam_r;
    uint32_t beam_r2_q16 = g_beam_r2_q16;
    uint32_t gs          = g_gs;
    int      Rb          = g_line_Rb;

    /* Fixed colour for the whole line — looked up once */
    const uint8_t *c = s_overlay_palette[colorPaletteEntry & 0x7F];
    int b_col = c[0], g_col = c[1], r_col = c[2];
    int u_full = bgr_to_u(b_col, g_col, r_col);   /* constant per call */
    int v_full = bgr_to_v(b_col, g_col, r_col);

    int bx0 = iclamp((x0 < x1 ? x0 : x1) - Rb, 0, fb_w - 1);
    int bx1 = iclamp((x0 > x1 ? x0 : x1) + Rb, 0, fb_w - 1);
    int by0 = iclamp((y0 < y1 ? y0 : y1) - Rb, 0, fb_h - 1);
    int by1 = iclamp((y0 > y1 ? y0 : y1) + Rb, 0, fb_h - 1);

    int dx   = x1 - x0;
    int dy   = y1 - y0;
    int len2 = dx * dx + dy * dy;

    /* Opt 1: degenerate dot */
    if (len2 == 0) {
        for (int py = by0; py <= by1; py++) {
            uint8_t *row_fb = fb + (size_t)py * fb_w * 2;
            for (int px = bx0; px <= bx1; px++) {
                uint32_t d2_q16 = cap_d2_q16(px - x0, py - y0);
                int contrib;
                if (d2_q16 <= beam_r2_q16) {
                    contrib = brightness;
                } else if (gs == 0) {
                    continue;
                } else {
                    uint32_t glow_d2 = d2_q16 - beam_r2_q16;
                    uint32_t lut_idx = (uint32_t)(((uint64_t)glow_d2 * gs) >> 30);
                    if (lut_idx >= GAUSS_LUT_SIZE) continue;
                    contrib = (brightness * (int)gauss_lut[lut_idx]) >> 8;
                    if (contrib == 0) continue;
                }
                uint8_t *dst    = row_fb + px * 2;
                int y_contrib   = (bgr_to_y(b_col, g_col, r_col) * contrib) >> 8;
                if (y_contrib) {
                    int yv      = dst[0] + y_contrib;
                    dst[0]      = (uint8_t)(yv > 255 ? 255 : yv);
                }
                int u_delta     = ((u_full - 128) * contrib) >> 8;
                int v_delta     = ((v_full - 128) * contrib) >> 8;
                if (u_delta | v_delta) {
                    int u_off   = (px & 1) ? -1 : 1;
                    int uv      = (int)dst[u_off]     + u_delta;
                    int vv      = (int)dst[u_off + 2] + v_delta;
                    dst[u_off]     = (uint8_t)(uv < 0 ? 0 : uv > 255 ? 255 : uv);
                    dst[u_off + 2] = (uint8_t)(vv < 0 ? 0 : vv > 255 ? 255 : vv);
                }
            }
        }
        return;
    }

    /* len2 > 0 from here */
    uint32_t beam_r2_len2 = (uint32_t)beam_r * beam_r * (uint32_t)len2;
    uint32_t inv_len2     = (uint32_t)(0xFFFFFFFFULL / (uint32_t)len2);

    int cross_row = dx * (by0 - y0) - dy * (bx0 - x0);
    int dot_row   = (bx0 - x0) * dx + (by0 - y0) * dy;

    int ady = dy < 0 ? -dy : dy;
    int adx = dx < 0 ? -dx : dx;

    /* Opt 3: hoist narrow-vs-full-range branch out of y-loop */
    if (ady > 0) {
        int half_w = Rb * (adx + ady) / ady + 1;
        for (int py = by0; py <= by1; py++) {
            int px_c = bx0 + cross_row / dy;
            int px_lo_r = px_c - half_w; if (px_lo_r < bx0) px_lo_r = bx0;
            int px_hi_r = px_c + half_w; if (px_hi_r > bx1) px_hi_r = bx1;
            if (px_lo_r > px_hi_r) { cross_row += dx; dot_row += dy; continue; }

            int dd    = px_lo_r - bx0;
            int cross = cross_row - dy * dd;
            int dot   = dot_row   + dx * dd;
            uint8_t *row_fb = fb + (size_t)py * fb_w * 2;

            for (int px = px_lo_r; px <= px_hi_r; px++) {
                uint32_t d2_q16 = 0;
                int in_beam = 0;
                if (dot <= 0) {
                    d2_q16  = cap_d2_q16(px - x0, py - y0);
                    in_beam = (d2_q16 <= beam_r2_q16);
                } else if (dot >= len2) {
                    d2_q16  = cap_d2_q16(px - x1, py - y1);
                    in_beam = (d2_q16 <= beam_r2_q16);
                } else {
                    uint64_t csq64 = (uint64_t)((int64_t)cross * cross);
                    if (csq64 > 0xFFFFFFFFu) goto px_next_n;
                    uint32_t csq = (uint32_t)csq64;
                    if (csq <= beam_r2_len2) {
                        in_beam = 1;
                    } else {
                        d2_q16 = (uint32_t)(((uint64_t)csq * inv_len2) >> 16);
                    }
                }
                int contrib;
                if (in_beam) {
                    contrib = brightness;
                } else if (gs == 0) {
                    goto px_next_n;
                } else {
                    uint32_t glow_d2 = d2_q16 - beam_r2_q16;
                    uint32_t lut_idx = (uint32_t)(((uint64_t)glow_d2 * gs) >> 30);
                    if (lut_idx >= GAUSS_LUT_SIZE) goto px_next_n;
                    contrib = (brightness * (int)gauss_lut[lut_idx]) >> 8;
                    if (contrib == 0) goto px_next_n;
                }
                {
                    uint8_t *dst    = row_fb + px * 2;
                    int y_contrib   = (bgr_to_y(b_col, g_col, r_col) * contrib) >> 8;
                    if (y_contrib) {
                        int yv      = dst[0] + y_contrib;
                        dst[0]      = (uint8_t)(yv > 255 ? 255 : yv);
                    }
                    int u_delta     = ((u_full - 128) * contrib) >> 8;
                    int v_delta     = ((v_full - 128) * contrib) >> 8;
                    if (u_delta | v_delta) {
                        int u_off   = (px & 1) ? -1 : 1;
                        int uv      = (int)dst[u_off]     + u_delta;
                        int vv      = (int)dst[u_off + 2] + v_delta;
                        dst[u_off]     = (uint8_t)(uv < 0 ? 0 : uv > 255 ? 255 : uv);
                        dst[u_off + 2] = (uint8_t)(vv < 0 ? 0 : vv > 255 ? 255 : vv);
                    }
                }
            px_next_n:
                cross -= dy; dot += dx;
            }
            cross_row += dx; dot_row += dy;
        }
    } else {
        /* ady == 0: full x-range every row */
        for (int py = by0; py <= by1; py++) {
            int cross = cross_row;
            int dot   = dot_row;
            uint8_t *row_fb = fb + (size_t)py * fb_w * 2;

            for (int px = bx0; px <= bx1; px++) {
                uint32_t d2_q16 = 0;
                int in_beam = 0;
                if (dot <= 0) {
                    d2_q16  = cap_d2_q16(px - x0, py - y0);
                    in_beam = (d2_q16 <= beam_r2_q16);
                } else if (dot >= len2) {
                    d2_q16  = cap_d2_q16(px - x1, py - y1);
                    in_beam = (d2_q16 <= beam_r2_q16);
                } else {
                    uint64_t csq64 = (uint64_t)((int64_t)cross * cross);
                    if (csq64 > 0xFFFFFFFFu) goto px_next_f;
                    uint32_t csq = (uint32_t)csq64;
                    if (csq <= beam_r2_len2) {
                        in_beam = 1;
                    } else {
                        d2_q16 = (uint32_t)(((uint64_t)csq * inv_len2) >> 16);
                    }
                }
                int contrib;
                if (in_beam) {
                    contrib = brightness;
                } else if (gs == 0) {
                    goto px_next_f;
                } else {
                    uint32_t glow_d2 = d2_q16 - beam_r2_q16;
                    uint32_t lut_idx = (uint32_t)(((uint64_t)glow_d2 * gs) >> 30);
                    if (lut_idx >= GAUSS_LUT_SIZE) goto px_next_f;
                    contrib = (brightness * (int)gauss_lut[lut_idx]) >> 8;
                    if (contrib == 0) goto px_next_f;
                }
                {
                    uint8_t *dst    = row_fb + px * 2;
                    int y_contrib   = (bgr_to_y(b_col, g_col, r_col) * contrib) >> 8;
                    if (y_contrib) {
                        int yv      = dst[0] + y_contrib;
                        dst[0]      = (uint8_t)(yv > 255 ? 255 : yv);
                    }
                    int u_delta     = ((u_full - 128) * contrib) >> 8;
                    int v_delta     = ((v_full - 128) * contrib) >> 8;
                    if (u_delta | v_delta) {
                        int u_off   = (px & 1) ? -1 : 1;
                        int uv      = (int)dst[u_off]     + u_delta;
                        int vv      = (int)dst[u_off + 2] + v_delta;
                        dst[u_off]     = (uint8_t)(uv < 0 ? 0 : uv > 255 ? 255 : uv);
                        dst[u_off + 2] = (uint8_t)(vv < 0 ? 0 : vv > 255 ? 255 : vv);
                    }
                }
            px_next_f:
                cross -= dy; dot += dx;
            }
            cross_row += dx; dot_row += dy;
        }
    }
}

/* ════════════════════════════════════════════════════════════════════════
 * draw_line_yuv422_brightness_c
 * Same algorithm as draw_line_yuv422_color_c but draws a white line at a
 * given brightness instead of a palette colour.
 * White = R=G=B=255 → bgr_to_y=255, bgr_to_u=bgr_to_v=128.
 * U/V deltas are therefore always zero — only Y is ever written.
 * ════════════════════════════════════════════════════════════════════════ */
IRAM_ATTR void draw_line_yuv422_brightness_c(
        uint8_t *fb, int fb_w, int fb_h,
        int x0, int y0, int x1, int y1,
        int brightness)
{
    if (!fb || fb_w <= 0 || fb_h <= 0) return;
    if (brightness <= 0) return;

    uint32_t beam_r2_q16 = g_beam_r2_q16;
    uint32_t gs          = g_gs;
    int      beam_r      = g_beam_r;
    int      Rb          = g_line_Rb;

    int bx0 = iclamp((x0 < x1 ? x0 : x1) - Rb, 0, fb_w - 1);
    int bx1 = iclamp((x0 > x1 ? x0 : x1) + Rb, 0, fb_w - 1);
    int by0 = iclamp((y0 < y1 ? y0 : y1) - Rb, 0, fb_h - 1);
    int by1 = iclamp((y0 > y1 ? y0 : y1) + Rb, 0, fb_h - 1);

    int dx   = x1 - x0;
    int dy   = y1 - y0;
    int len2 = dx * dx + dy * dy;

    /* Opt 1: degenerate dot */
    if (len2 == 0) {
        for (int py = by0; py <= by1; py++) {
            uint8_t *row_fb = fb + (size_t)py * fb_w * 2;
            for (int px = bx0; px <= bx1; px++) {
                uint32_t d2_q16 = cap_d2_q16(px - x0, py - y0);
                int contrib;
                if (d2_q16 <= beam_r2_q16) {
                    contrib = brightness;
                } else if (gs == 0) {
                    continue;
                } else {
                    uint32_t glow_d2 = d2_q16 - beam_r2_q16;
                    uint32_t lut_idx = (uint32_t)(((uint64_t)glow_d2 * gs) >> 30);
                    if (lut_idx >= GAUSS_LUT_SIZE) continue;
                    contrib = (brightness * (int)gauss_lut[lut_idx]) >> 8;
                    if (contrib == 0) continue;
                }
                uint8_t *dst  = row_fb + px * 2;
                /* White: y_contrib = (255 * contrib) >> 8 */
                int y_contrib = (255 * contrib) >> 8;
                if (y_contrib) {
                    int yv = dst[0] + y_contrib;
                    dst[0] = (uint8_t)(yv > 255 ? 255 : yv);
                }
                /* U/V deltas always zero for white — UV never written */
            }
        }
        return;
    }

    uint32_t beam_r2_len2 = (uint32_t)beam_r * beam_r * (uint32_t)len2;
    uint32_t inv_len2     = (uint32_t)(0xFFFFFFFFULL / (uint32_t)len2);

    int cross_row = dx * (by0 - y0) - dy * (bx0 - x0);
    int dot_row   = (bx0 - x0) * dx + (by0 - y0) * dy;

    int ady = dy < 0 ? -dy : dy;
    int adx = dx < 0 ? -dx : dx;

    if (ady > 0) {
        int half_w = Rb * (adx + ady) / ady + 1;
        for (int py = by0; py <= by1; py++) {
            int px_c    = bx0 + cross_row / dy;
            int px_lo_r = px_c - half_w; if (px_lo_r < bx0) px_lo_r = bx0;
            int px_hi_r = px_c + half_w; if (px_hi_r > bx1) px_hi_r = bx1;
            if (px_lo_r > px_hi_r) { cross_row += dx; dot_row += dy; continue; }

            int dd    = px_lo_r - bx0;
            int cross = cross_row - dy * dd;
            int dot   = dot_row   + dx * dd;
            uint8_t *row_fb = fb + (size_t)py * fb_w * 2;

            for (int px = px_lo_r; px <= px_hi_r; px++) {
                uint32_t d2_q16 = 0;
                int in_beam = 0;
                if (dot <= 0) {
                    d2_q16  = cap_d2_q16(px - x0, py - y0);
                    in_beam = (d2_q16 <= beam_r2_q16);
                } else if (dot >= len2) {
                    d2_q16  = cap_d2_q16(px - x1, py - y1);
                    in_beam = (d2_q16 <= beam_r2_q16);
                } else {
                    uint64_t csq64 = (uint64_t)((int64_t)cross * cross);
                    if (csq64 > 0xFFFFFFFFu) goto bpx_next_n;
                    uint32_t csq = (uint32_t)csq64;
                    if (csq <= beam_r2_len2) {
                        in_beam = 1;
                    } else {
                        d2_q16 = (uint32_t)(((uint64_t)csq * inv_len2) >> 16);
                    }
                }
                int contrib;
                if (in_beam) {
                    contrib = brightness;
                } else if (gs == 0) {
                    goto bpx_next_n;
                } else {
                    uint32_t glow_d2 = d2_q16 - beam_r2_q16;
                    uint32_t lut_idx = (uint32_t)(((uint64_t)glow_d2 * gs) >> 30);
                    if (lut_idx >= GAUSS_LUT_SIZE) goto bpx_next_n;
                    contrib = (brightness * (int)gauss_lut[lut_idx]) >> 8;
                    if (contrib == 0) goto bpx_next_n;
                }
                {
                    uint8_t *dst  = row_fb + px * 2;
                    int y_contrib = (255 * contrib) >> 8;
                    if (y_contrib) {
                        int yv = dst[0] + y_contrib;
                        dst[0] = (uint8_t)(yv > 255 ? 255 : yv);
                    }
                    /* U/V deltas always zero for white — UV never written */
                }
            bpx_next_n:
                cross -= dy; dot += dx;
            }
            cross_row += dx; dot_row += dy;
        }
    } else {
        for (int py = by0; py <= by1; py++) {
            int cross = cross_row;
            int dot   = dot_row;
            uint8_t *row_fb = fb + (size_t)py * fb_w * 2;

            for (int px = bx0; px <= bx1; px++) {
                uint32_t d2_q16 = 0;
                int in_beam = 0;
                if (dot <= 0) {
                    d2_q16  = cap_d2_q16(px - x0, py - y0);
                    in_beam = (d2_q16 <= beam_r2_q16);
                } else if (dot >= len2) {
                    d2_q16  = cap_d2_q16(px - x1, py - y1);
                    in_beam = (d2_q16 <= beam_r2_q16);
                } else {
                    uint64_t csq64 = (uint64_t)((int64_t)cross * cross);
                    if (csq64 > 0xFFFFFFFFu) goto bpx_next_f;
                    uint32_t csq = (uint32_t)csq64;
                    if (csq <= beam_r2_len2) {
                        in_beam = 1;
                    } else {
                        d2_q16 = (uint32_t)(((uint64_t)csq * inv_len2) >> 16);
                    }
                }
                int contrib;
                if (in_beam) {
                    contrib = brightness;
                } else if (gs == 0) {
                    goto bpx_next_f;
                } else {
                    uint32_t glow_d2 = d2_q16 - beam_r2_q16;
                    uint32_t lut_idx = (uint32_t)(((uint64_t)glow_d2 * gs) >> 30);
                    if (lut_idx >= GAUSS_LUT_SIZE) goto bpx_next_f;
                    contrib = (brightness * (int)gauss_lut[lut_idx]) >> 8;
                    if (contrib == 0) goto bpx_next_f;
                }
                {
                    uint8_t *dst  = row_fb + px * 2;
                    int y_contrib = (255 * contrib) >> 8;
                    if (y_contrib) {
                        int yv = dst[0] + y_contrib;
                        dst[0] = (uint8_t)(yv > 255 ? 255 : yv);
                    }
                    /* U/V deltas always zero for white — UV never written */
                }
            bpx_next_f:
                cross -= dy; dot += dx;
            }
            cross_row += dx; dot_row += dy;
        }
    }
}

/* ════════════════════════════════════════════════════════════════════════
 * undraw_line_yuv422_brightness_c
 * Same algorithm as undraw_line_yuv422_color_c but for a white/brightness draw.
 * Only Y is gated and erased (U/V deltas are always zero for white).
 * ════════════════════════════════════════════════════════════════════════ */
IRAM_ATTR void undraw_line_yuv422_brightness_c(
        uint8_t *fb, int fb_w, int fb_h,
        int x0, int y0, int x1, int y1,
        int brightness)
{
    if (!fb || fb_w <= 0 || fb_h <= 0) return;
    if (brightness <= 0) return;

    uint32_t beam_r2_q16 = g_beam_r2_q16;
    uint32_t gs          = g_gs;
    int      beam_r      = g_beam_r;
    int      Rb          = g_line_Rb;

    int bx0 = iclamp((x0 < x1 ? x0 : x1) - Rb, 0, fb_w - 1);
    int bx1 = iclamp((x0 > x1 ? x0 : x1) + Rb, 0, fb_w - 1);
    int by0 = iclamp((y0 < y1 ? y0 : y1) - Rb, 0, fb_h - 1);
    int by1 = iclamp((y0 > y1 ? y0 : y1) + Rb, 0, fb_h - 1);

    int dx   = x1 - x0;
    int dy   = y1 - y0;
    int len2 = dx * dx + dy * dy;

    /* Opt 1: degenerate dot */
    if (len2 == 0) {
        for (int py = by0; py <= by1; py++) {
            uint8_t *row_fb = fb + (size_t)py * fb_w * 2;
            for (int px = bx0; px <= bx1; px++) {
                uint32_t d2_q16 = cap_d2_q16(px - x0, py - y0);
                int contrib;
                if (d2_q16 <= beam_r2_q16) {
                    contrib = brightness;
                } else if (gs == 0) {
                    continue;
                } else {
                    uint32_t glow_d2 = d2_q16 - beam_r2_q16;
                    uint32_t lut_idx = (uint32_t)(((uint64_t)glow_d2 * gs) >> 30);
                    if (lut_idx >= GAUSS_LUT_SIZE) continue;
                    contrib = (brightness * (int)gauss_lut[lut_idx]) >> 8;
                    if (contrib == 0) continue;
                }
                uint8_t *dst  = row_fb + px * 2;
                int y_contrib = (255 * contrib) >> 8;
                if (y_contrib) dst[0] = 0;
                /* U/V deltas always zero for white — UV never written/erased */
            }
        }
        return;
    }

    uint32_t beam_r2_len2 = (uint32_t)beam_r * beam_r * (uint32_t)len2;
    uint32_t inv_len2     = (uint32_t)(0xFFFFFFFFULL / (uint32_t)len2);

    int cross_row = dx * (by0 - y0) - dy * (bx0 - x0);
    int dot_row   = (bx0 - x0) * dx + (by0 - y0) * dy;

    int udy = dy < 0 ? -dy : dy;
    int udx = dx < 0 ? -dx : dx;

    if (udy > 0) {
        int ud_half_w = Rb * (udx + udy) / udy + 1;
        for (int py = by0; py <= by1; py++) {
            int px_c    = bx0 + cross_row / dy;
            int px_lo_r = px_c - ud_half_w; if (px_lo_r < bx0) px_lo_r = bx0;
            int px_hi_r = px_c + ud_half_w; if (px_hi_r > bx1) px_hi_r = bx1;
            if (px_lo_r > px_hi_r) { cross_row += dx; dot_row += dy; continue; }

            int udd   = px_lo_r - bx0;
            int cross = cross_row - dy * udd;
            int dot   = dot_row   + dx * udd;
            uint8_t *row_fb = fb + (size_t)py * fb_w * 2;

            for (int px = px_lo_r; px <= px_hi_r; px++) {
                uint32_t d2_q16 = 0;
                int in_beam = 0;
                if (dot <= 0) {
                    d2_q16  = cap_d2_q16(px - x0, py - y0);
                    in_beam = (d2_q16 <= beam_r2_q16);
                } else if (dot >= len2) {
                    d2_q16  = cap_d2_q16(px - x1, py - y1);
                    in_beam = (d2_q16 <= beam_r2_q16);
                } else {
                    uint64_t csq64 = (uint64_t)((int64_t)cross * cross);
                    if (csq64 > 0xFFFFFFFFu) goto ubpx_next_n;
                    uint32_t csq = (uint32_t)csq64;
                    if (csq <= beam_r2_len2) {
                        in_beam = 1;
                    } else {
                        d2_q16 = (uint32_t)(((uint64_t)csq * inv_len2) >> 16);
                    }
                }
                int contrib;
                if (in_beam) {
                    contrib = brightness;
                } else if (gs == 0) {
                    goto ubpx_next_n;
                } else {
                    uint32_t glow_d2 = d2_q16 - beam_r2_q16;
                    uint32_t lut_idx = (uint32_t)(((uint64_t)glow_d2 * gs) >> 30);
                    if (lut_idx >= GAUSS_LUT_SIZE) goto ubpx_next_n;
                    contrib = (brightness * (int)gauss_lut[lut_idx]) >> 8;
                    if (contrib == 0) goto ubpx_next_n;
                }
                {
                    uint8_t *dst  = row_fb + px * 2;
                    int y_contrib = (255 * contrib) >> 8;
                    if (y_contrib) dst[0] = 0;
                }
            ubpx_next_n:
                cross -= dy; dot += dx;
            }
            cross_row += dx; dot_row += dy;
        }
    } else {
        for (int py = by0; py <= by1; py++) {
            int cross = cross_row;
            int dot   = dot_row;
            uint8_t *row_fb = fb + (size_t)py * fb_w * 2;

            for (int px = bx0; px <= bx1; px++) {
                uint32_t d2_q16 = 0;
                int in_beam = 0;
                if (dot <= 0) {
                    d2_q16  = cap_d2_q16(px - x0, py - y0);
                    in_beam = (d2_q16 <= beam_r2_q16);
                } else if (dot >= len2) {
                    d2_q16  = cap_d2_q16(px - x1, py - y1);
                    in_beam = (d2_q16 <= beam_r2_q16);
                } else {
                    uint64_t csq64 = (uint64_t)((int64_t)cross * cross);
                    if (csq64 > 0xFFFFFFFFu) goto ubpx_next_f;
                    uint32_t csq = (uint32_t)csq64;
                    if (csq <= beam_r2_len2) {
                        in_beam = 1;
                    } else {
                        d2_q16 = (uint32_t)(((uint64_t)csq * inv_len2) >> 16);
                    }
                }
                int contrib;
                if (in_beam) {
                    contrib = brightness;
                } else if (gs == 0) {
                    goto ubpx_next_f;
                } else {
                    uint32_t glow_d2 = d2_q16 - beam_r2_q16;
                    uint32_t lut_idx = (uint32_t)(((uint64_t)glow_d2 * gs) >> 30);
                    if (lut_idx >= GAUSS_LUT_SIZE) goto ubpx_next_f;
                    contrib = (brightness * (int)gauss_lut[lut_idx]) >> 8;
                    if (contrib == 0) goto ubpx_next_f;
                }
                {
                    uint8_t *dst  = row_fb + px * 2;
                    int y_contrib = (255 * contrib) >> 8;
                    if (y_contrib) dst[0] = 0;
                }
            ubpx_next_f:
                cross -= dy; dot += dx;
            }
            cross_row += dx; dot_row += dy;
        }
    }
}

/* ════════════════════════════════════════════════════════════════════════
 * undraw_line_yuv422_color_c
 * Erases beam/glow footprint to black (Y=0, U=128, V=128).
 * Gates each PSRAM write with the same delta test as draw — if draw
 * would not have touched a channel, undraw does not touch it either.
 * ════════════════════════════════════════════════════════════════════════ */
IRAM_ATTR void undraw_line_yuv422_color_c(
        uint8_t *fb, int fb_w, int fb_h,
        int x0, int y0, int x1, int y1,
        int colorPaletteEntry)
{
    if (!fb || fb_w <= 0 || fb_h <= 0) return;

    int brightness = 255;

    /* Opt 2: load precomputed globals */
    int      beam_r      = g_beam_r;
    uint32_t beam_r2_q16 = g_beam_r2_q16;
    uint32_t gs          = g_gs;
    int      Rb          = g_line_Rb;

    /* Fixed colour — same lookup as draw */
    const uint8_t *c = s_overlay_palette[colorPaletteEntry & 0x7F];
    int b_col = c[0], g_col = c[1], r_col = c[2];
    int u_full = bgr_to_u(b_col, g_col, r_col);
    int v_full = bgr_to_v(b_col, g_col, r_col);

    int bx0 = iclamp((x0 < x1 ? x0 : x1) - Rb, 0, fb_w - 1);
    int bx1 = iclamp((x0 > x1 ? x0 : x1) + Rb, 0, fb_w - 1);
    int by0 = iclamp((y0 < y1 ? y0 : y1) - Rb, 0, fb_h - 1);
    int by1 = iclamp((y0 > y1 ? y0 : y1) + Rb, 0, fb_h - 1);

    int dx   = x1 - x0;
    int dy   = y1 - y0;
    int len2 = dx * dx + dy * dy;

    /* Opt 1: degenerate dot */
    if (len2 == 0) {
        for (int py = by0; py <= by1; py++) {
            uint8_t *row_fb = fb + (size_t)py * fb_w * 2;
            for (int px = bx0; px <= bx1; px++) {
                uint32_t d2_q16 = cap_d2_q16(px - x0, py - y0);
                int contrib;
                if (d2_q16 <= beam_r2_q16) {
                    contrib = brightness;
                } else if (gs == 0) {
                    continue;
                } else {
                    uint32_t glow_d2 = d2_q16 - beam_r2_q16;
                    uint32_t lut_idx = (uint32_t)(((uint64_t)glow_d2 * gs) >> 30);
                    if (lut_idx >= GAUSS_LUT_SIZE) continue;
                    contrib = (brightness * (int)gauss_lut[lut_idx]) >> 8;
                    if (contrib == 0) continue;
                }
                uint8_t *dst    = row_fb + px * 2;
                int y_contrib   = (bgr_to_y(b_col, g_col, r_col) * contrib) >> 8;
                if (y_contrib) dst[0] = 0;
                int u_delta     = ((u_full - 128) * contrib) >> 8;
                int v_delta     = ((v_full - 128) * contrib) >> 8;
                if (u_delta | v_delta) {
                    int u_off      = (px & 1) ? -1 : 1;
                    dst[u_off]     = 128;
                    dst[u_off + 2] = 128;
                }
            }
        }
        return;
    }

    /* len2 > 0 from here */
    uint32_t beam_r2_len2 = (uint32_t)beam_r * beam_r * (uint32_t)len2;
    uint32_t inv_len2     = (uint32_t)(0xFFFFFFFFULL / (uint32_t)len2);

    int cross_row = dx * (by0 - y0) - dy * (bx0 - x0);
    int dot_row   = (bx0 - x0) * dx + (by0 - y0) * dy;

    int udy = dy < 0 ? -dy : dy;
    int udx = dx < 0 ? -dx : dx;

    /* Opt 3: hoist narrow-vs-full-range branch out of y-loop */
    if (udy > 0) {
        int ud_half_w = Rb * (udx + udy) / udy + 1;
        for (int py = by0; py <= by1; py++) {
            int px_c = bx0 + cross_row / dy;
            int px_lo_r = px_c - ud_half_w; if (px_lo_r < bx0) px_lo_r = bx0;
            int px_hi_r = px_c + ud_half_w; if (px_hi_r > bx1) px_hi_r = bx1;
            if (px_lo_r > px_hi_r) { cross_row += dx; dot_row += dy; continue; }

            int udd   = px_lo_r - bx0;
            int cross = cross_row - dy * udd;
            int dot   = dot_row   + dx * udd;
            uint8_t *row_fb = fb + (size_t)py * fb_w * 2;

            for (int px = px_lo_r; px <= px_hi_r; px++) {
                uint32_t d2_q16 = 0;
                int in_beam = 0;
                if (dot <= 0) {
                    d2_q16  = cap_d2_q16(px - x0, py - y0);
                    in_beam = (d2_q16 <= beam_r2_q16);
                } else if (dot >= len2) {
                    d2_q16  = cap_d2_q16(px - x1, py - y1);
                    in_beam = (d2_q16 <= beam_r2_q16);
                } else {
                    uint64_t csq64 = (uint64_t)((int64_t)cross * cross);
                    if (csq64 > 0xFFFFFFFFu) goto ud_px_next_n;
                    uint32_t csq = (uint32_t)csq64;
                    if (csq <= beam_r2_len2) {
                        in_beam = 1;
                    } else {
                        d2_q16 = (uint32_t)(((uint64_t)csq * inv_len2) >> 16);
                    }
                }
                int contrib;
                if (in_beam) {
                    contrib = brightness;
                } else if (gs == 0) {
                    goto ud_px_next_n;
                } else {
                    uint32_t glow_d2 = d2_q16 - beam_r2_q16;
                    uint32_t lut_idx = (uint32_t)(((uint64_t)glow_d2 * gs) >> 30);
                    if (lut_idx >= GAUSS_LUT_SIZE) goto ud_px_next_n;
                    contrib = (brightness * (int)gauss_lut[lut_idx]) >> 8;
                    if (contrib == 0) goto ud_px_next_n;
                }
                {
                    uint8_t *dst    = row_fb + px * 2;
                    int y_contrib   = (bgr_to_y(b_col, g_col, r_col) * contrib) >> 8;
                    if (y_contrib) dst[0] = 0;
                    int u_delta     = ((u_full - 128) * contrib) >> 8;
                    int v_delta     = ((v_full - 128) * contrib) >> 8;
                    if (u_delta | v_delta) {
                        int u_off      = (px & 1) ? -1 : 1;
                        dst[u_off]     = 128;
                        dst[u_off + 2] = 128;
                    }
                }
            ud_px_next_n:
                cross -= dy; dot += dx;
            }
            cross_row += dx; dot_row += dy;
        }
    } else {
        /* udy == 0: full x-range every row */
        for (int py = by0; py <= by1; py++) {
            int cross = cross_row;
            int dot   = dot_row;
            uint8_t *row_fb = fb + (size_t)py * fb_w * 2;

            for (int px = bx0; px <= bx1; px++) {
                uint32_t d2_q16 = 0;
                int in_beam = 0;
                if (dot <= 0) {
                    d2_q16  = cap_d2_q16(px - x0, py - y0);
                    in_beam = (d2_q16 <= beam_r2_q16);
                } else if (dot >= len2) {
                    d2_q16  = cap_d2_q16(px - x1, py - y1);
                    in_beam = (d2_q16 <= beam_r2_q16);
                } else {
                    uint64_t csq64 = (uint64_t)((int64_t)cross * cross);
                    if (csq64 > 0xFFFFFFFFu) goto ud_px_next_f;
                    uint32_t csq = (uint32_t)csq64;
                    if (csq <= beam_r2_len2) {
                        in_beam = 1;
                    } else {
                        d2_q16 = (uint32_t)(((uint64_t)csq * inv_len2) >> 16);
                    }
                }
                int contrib;
                if (in_beam) {
                    contrib = brightness;
                } else if (gs == 0) {
                    goto ud_px_next_f;
                } else {
                    uint32_t glow_d2 = d2_q16 - beam_r2_q16;
                    uint32_t lut_idx = (uint32_t)(((uint64_t)glow_d2 * gs) >> 30);
                    if (lut_idx >= GAUSS_LUT_SIZE) goto ud_px_next_f;
                    contrib = (brightness * (int)gauss_lut[lut_idx]) >> 8;
                    if (contrib == 0) goto ud_px_next_f;
                }
                {
                    uint8_t *dst    = row_fb + px * 2;
                    int y_contrib   = (bgr_to_y(b_col, g_col, r_col) * contrib) >> 8;
                    if (y_contrib) dst[0] = 0;
                    int u_delta     = ((u_full - 128) * contrib) >> 8;
                    int v_delta     = ((v_full - 128) * contrib) >> 8;
                    if (u_delta | v_delta) {
                        int u_off      = (px & 1) ? -1 : 1;
                        dst[u_off]     = 128;
                        dst[u_off + 2] = 128;
                    }
                }
            ud_px_next_f:
                cross -= dy; dot += dx;
            }
            cross_row += dx; dot_row += dy;
        }
    }
}
