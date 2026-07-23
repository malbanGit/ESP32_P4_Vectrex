/*
 * Palette-colour distance-field line renderer for YUV422 (YUYV) framebuffers.
 * Copied from draw_line_dist_yuv422_overlay.c — overlay pixel gate replaced
 * with a fixed colour from s_overlay_palette[colorPaletteEntry].
 */

#include <stdint.h>
#include <stddef.h>
#include "esp_attr.h"
#include "defines.h"
#include "draw_line_yuv422.h"

extern int           g_line_glow;
extern int           g_line_width;
extern int           g_line_Rb;
extern const uint8_t gauss_lut[];

extern uint8_t s_overlay_palette[128][3];

#define GAUSS_LUT_SIZE 178

static inline int iclamp(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline uint32_t gs_for_glow(int glow_r)
{
    int gr2 = glow_r * glow_r;
    return (gr2 == 0) ? 0u : (uint32_t)((177u << 14) / (uint32_t)gr2);
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

/* ════════════════════════════════════════════════════════════════════════
 * draw_line_yuv422_c
 * ════════════════════════════════════════════════════════════════════════ */
IRAM_ATTR void draw_line_yuv422_c(
        uint8_t *fb, int fb_w, int fb_h,
        int x0, int y0, int x1, int y1,
        int colorPaletteEntry)
{
    if (!fb || fb_w <= 0 || fb_h <= 0) return;

    int brightness = 255;
    int glow   = (g_line_glow < 0) ? 0 : g_line_glow;
    int beam_r = g_line_width >> 1;
    int Rb     = g_line_Rb;

    /* Fixed colour for the whole line — looked up once */
    const uint8_t *c = s_overlay_palette[colorPaletteEntry & 0x7F];
    int b_col = c[0], g_col = c[1], r_col = c[2];

    int bx0 = iclamp((x0 < x1 ? x0 : x1) - Rb, 0, fb_w - 1);
    int bx1 = iclamp((x0 > x1 ? x0 : x1) + Rb, 0, fb_w - 1);
    int by0 = iclamp((y0 < y1 ? y0 : y1) - Rb, 0, fb_h - 1);
    int by1 = iclamp((y0 > y1 ? y0 : y1) + Rb, 0, fb_h - 1);

    int dx   = x1 - x0;
    int dy   = y1 - y0;
    int len2 = dx * dx + dy * dy; 

    uint32_t beam_r2_q16  = cap_d2_q16(beam_r, 0);
    uint32_t beam_r2_len2 = (uint32_t)beam_r * beam_r * (uint32_t)len2;
    uint32_t gs            = gs_for_glow(glow);
    uint32_t inv_len2      = (len2 > 0)
                           ? (uint32_t)(0xFFFFFFFFULL / (uint32_t)len2) : 0u;

    int cross_row = dx * (by0 - y0) - dy * (bx0 - x0);
    int dot_row   = (bx0 - x0) * dx + (by0 - y0) * dy;

    int ady    = dy < 0 ? -dy : dy;
    int adx    = dx < 0 ? -dx : dx;
    int half_w = (ady > 0) ? Rb * (adx + ady) / ady + 1 : 0x7FFFFFFF;

    for (int py = by0; py <= by1; py++) {
        int px_lo_r, px_hi_r;
        if (len2 > 0 && ady > 0) {
            int px_c = bx0 + cross_row / dy;
            px_lo_r = px_c - half_w; if (px_lo_r < bx0) px_lo_r = bx0;
            px_hi_r = px_c + half_w; if (px_hi_r > bx1) px_hi_r = bx1;
        } else {
            px_lo_r = bx0;
            px_hi_r = bx1;
        }
        if (px_lo_r > px_hi_r) {
            if (len2 > 0) { cross_row += dx; dot_row += dy; }
            continue;
        }

        int dd    = px_lo_r - bx0;
        int cross = cross_row - dy * dd;
        int dot   = dot_row   + dx * dd;

        uint8_t *row_fb = fb + (size_t)py * fb_w * 2;

        for (int px = px_lo_r; px <= px_hi_r; px++) {
            uint32_t d2_q16 = 0;
            int in_beam = 0;

            if (len2 == 0 || dot <= 0) {
                d2_q16  = cap_d2_q16(px - x0, py - y0);
                in_beam = (d2_q16 <= beam_r2_q16);
            } else if (dot >= len2) {
                d2_q16  = cap_d2_q16(px - x1, py - y1);
                in_beam = (d2_q16 <= beam_r2_q16);
            } else {
                uint64_t csq64 = (uint64_t)((int64_t)cross * cross);
                if (csq64 > 0xFFFFFFFFu) goto px_next;
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
                goto px_next;
            } else {
                uint32_t glow_d2 = d2_q16 - beam_r2_q16;
                uint32_t lut_idx = (uint32_t)(((uint64_t)glow_d2 * gs) >> 30);
                if (lut_idx >= GAUSS_LUT_SIZE) goto px_next;
                contrib = (brightness * (int)gauss_lut[lut_idx]) >> 8;
                if (contrib == 0) goto px_next;
            }

            {
                uint8_t *dst  = row_fb + px * 2;
                int y_contrib = (bgr_to_y(b_col, g_col, r_col) * contrib) >> 8;
                int yv        = dst[0] + y_contrib;
                dst[0]        = (uint8_t)(yv > 255 ? 255 : yv);
                int u_off     = (px & 1) ? -1 : 1;
                int u_delta   = ((bgr_to_u(b_col, g_col, r_col) - 128) * contrib) >> 8;
                int v_delta   = ((bgr_to_v(b_col, g_col, r_col) - 128) * contrib) >> 8;
                int uv        = (int)dst[u_off]     + u_delta;
                int vv        = (int)dst[u_off + 2] + v_delta;
                dst[u_off]     = (uint8_t)(uv < 0 ? 0 : uv > 255 ? 255 : uv);
                dst[u_off + 2] = (uint8_t)(vv < 0 ? 0 : vv > 255 ? 255 : vv);
            }

        px_next:
            if (len2 > 0) { cross -= dy; dot += dx; }
        }
        if (len2 > 0) { cross_row += dx; dot_row += dy; }
    }
}

/* ════════════════════════════════════════════════════════════════════════
 * undraw_line_yuv422_c
 * Copied from undraw_line_yuv422_overlay_c — palette restore replaced
 * with a plain black write (Y=0, U=128, V=128).
 * ════════════════════════════════════════════════════════════════════════ */
IRAM_ATTR void undraw_line_yuv422_c(
        uint8_t *fb, int fb_w, int fb_h,
        int x0, int y0, int x1, int y1)
{
    if (!fb || fb_w <= 0 || fb_h <= 0) return;

    int glow   = (g_line_glow < 0) ? 0 : g_line_glow;
    int beam_r = g_line_width >> 1;
    int Rb     = g_line_Rb;

    int bx0 = iclamp((x0 < x1 ? x0 : x1) - Rb, 0, fb_w - 1);
    int bx1 = iclamp((x0 > x1 ? x0 : x1) + Rb, 0, fb_w - 1);
    int by0 = iclamp((y0 < y1 ? y0 : y1) - Rb, 0, fb_h - 1);
    int by1 = iclamp((y0 > y1 ? y0 : y1) + Rb, 0, fb_h - 1);

    int dx   = x1 - x0;
    int dy   = y1 - y0;
    int len2 = dx * dx + dy * dy;

    uint32_t beam_r2_q16  = cap_d2_q16(beam_r, 0);
    uint32_t beam_r2_len2 = (uint32_t)beam_r * beam_r * (uint32_t)len2;
    uint32_t gs            = gs_for_glow(glow);
    uint32_t inv_len2      = (len2 > 0)
                           ? (uint32_t)(0xFFFFFFFFULL / (uint32_t)len2) : 0u;

    int cross_row = dx * (by0 - y0) - dy * (bx0 - x0);
    int dot_row   = (bx0 - x0) * dx + (by0 - y0) * dy;

    int udy    = dy < 0 ? -dy : dy;
    int udx    = dx < 0 ? -dx : dx;
    int ud_half_w = (udy > 0) ? Rb * (udx + udy) / udy + 1 : 0x7FFFFFFF;

    for (int py = by0; py <= by1; py++) {
        int px_lo_r, px_hi_r;
        if (len2 > 0 && udy > 0) {
            int px_c = bx0 + cross_row / dy;
            px_lo_r = px_c - ud_half_w; if (px_lo_r < bx0) px_lo_r = bx0;
            px_hi_r = px_c + ud_half_w; if (px_hi_r > bx1) px_hi_r = bx1;
        } else {
            px_lo_r = bx0; px_hi_r = bx1;
        }
        if (px_lo_r > px_hi_r) {
            if (len2 > 0) { cross_row += dx; dot_row += dy; }
            continue;
        }

        int udd   = px_lo_r - bx0;
        int cross = cross_row - dy * udd;
        int dot   = dot_row   + dx * udd;

        uint8_t *row_fb = fb + (size_t)py * fb_w * 2;

        for (int px = px_lo_r; px <= px_hi_r; px++) {
            uint32_t d2_q16 = 0;
            int in_beam = 0;

            if (len2 == 0 || dot <= 0) {
                d2_q16  = cap_d2_q16(px - x0, py - y0);
                in_beam = (d2_q16 <= beam_r2_q16);
            } else if (dot >= len2) {
                d2_q16  = cap_d2_q16(px - x1, py - y1);
                in_beam = (d2_q16 <= beam_r2_q16);
            } else {
                uint64_t csq64 = (uint64_t)((int64_t)cross * cross);
                if (csq64 > 0xFFFFFFFFu) goto ud_px_next;
                uint32_t csq = (uint32_t)csq64;
                if (csq <= beam_r2_len2) {
                    in_beam = 1;
                } else {
                    d2_q16 = (uint32_t)(((uint64_t)csq * inv_len2) >> 16);
                }
            }

            if (in_beam) {
                /* in beam — write black */
            } else if (gs == 0) {
                goto ud_px_next;
            } else {
                uint32_t glow_d2 = d2_q16 - beam_r2_q16;
                uint32_t lut_idx = (uint32_t)(((uint64_t)glow_d2 * gs) >> 30);
                if (lut_idx >= GAUSS_LUT_SIZE) goto ud_px_next;
                if (gauss_lut[lut_idx] == 0)   goto ud_px_next;
            }

            {
                uint8_t *dst   = row_fb + px * 2;
                dst[0]         = 0;          /* Y = 0 */
                int u_off      = (px & 1) ? -1 : 1;
                dst[u_off]     = 128;        /* U = 128 */
                dst[u_off + 2] = 128;        /* V = 128 */
            }

        ud_px_next:
            if (len2 > 0) { cross -= dy; dot += dx; }
        }
        if (len2 > 0) { cross_row += dx; dot_row += dy; }
    }
}
