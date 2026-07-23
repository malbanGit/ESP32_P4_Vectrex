/*
 * Palette-colour distance-field line renderer for YUV422 (YUYV) framebuffers.
 * Copied from draw_line_dist_yuv422_overlay.c — overlay pixel gate replaced
 * with a fixed colour from s_overlay_palette[colorPaletteEntry].
 *
 * Optimisations applied:
 *  1. len2==0 dot case handled before main loops (no per-pixel beqz len2).
 *  2. beam_r2_q16 and gs loaded from precomputed globals g_beam_r2_q16 / g_gs.
 *  3. y-loop split into narrow (ady>0) and full-range bodies (constant per call).
 */

#include "defines.h"
#include "draw_line_yuv422.h"



/* ════════════════════════════════════════════════════════════════════════
 * draw_line_yuv422_brightness_c
 * Same algorithm as draw_line_yuv422_c but draws a white line at a given
 * brightness instead of a palette colour.
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
 * Same algorithm as undraw_line_yuv422_c but for a white/brightness draw.
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
