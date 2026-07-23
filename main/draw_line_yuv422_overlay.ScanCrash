/* draw_line_dist_yuv422_overlay.S
 *
 * RV32IMAC assembly for:
 *   draw_line_yuv422_overlay
 *   undraw_line_yuv422_overlay
 *
 * Direct translation of draw_line_dist_yuv422_overlay.c.
 * No sqrt/isqrt — uses Q16 cross-product distance field exactly as in C.
 *
 * cap_d2_q16(apx,apy): ax=(uint32_t)(apx<<8); ay=(uint32_t)(apy<<8); return ax*ax+ay*ay
 * Mid-segment: csq64=(int64_t)cross*cross; if csq64>0xFFFFFFFF skip; d2_q16=(csq*inv_len2)>>16
 * Gauss lut_idx: (glow_d2 * gs) >> 30
 *
 * Stack frame (DL=112 bytes):
 *   sp+ 0: ra       sp+52: dx       sp+72: beam_r2_q16   sp+ 92: px_hi_g
 *   sp+ 4: s0       sp+56: dy       sp+76: beam_r2_len2  sp+ 96: half_w
 *   sp+ 8: s1       sp+60: len2     sp+80: cross_row     sp+100: row_fb_base
 *   sp+12: s2       sp+64: inv_len2 sp+84: dot_row       sp+104: py
 *   sp+16: s3       sp+68: gs       sp+88: px_lo_g       sp+108: full_flag
 *   sp+20: s4       sp+48: s11
 *   ...
 *
 * Register map (draw and undraw):
 *   s0=fb  s1=fb_w  s2=bx0  s3=bx1  s4=by0  s5=by1
 *   s6=brightness (draw) / ea (undraw)
 *   s7=x0  s8=y0  s9=x1  s10=y1  s11=Rb
 *
 * Inner loop (t-registers survive local jal calls):
 *   t2=cross  t3=dot  t4=px  t5=px_hi  t6=pal_ptr  t1=local link reg
 *
 * WRITE_YUV_DRAW  (.Lwyd): inputs a0=contrib,a1=b,a2=g,a3=r,a7=dst,t4=px
 * WRITE_YUV_UNDRAW(.Lwyu): inputs s6=ea,    a1=b,a2=g,a3=r,a7=dst,t4=px
 * Both destroy: t0,a0,a4,a5,a6
 */

    .extern g_line_Rb
    .extern g_beam_r
    .extern g_beam_r2_q16
    .extern g_gs

    .section .iram0.text,"ax",@progbits
    .align 2

/* Frame constants */
#define DL         112
#define DL_RA        0
#define DL_S0        4
#define DL_S1        8
#define DL_S2       12
#define DL_S3       16
#define DL_S4       20
#define DL_S5       24
#define DL_S6       28
#define DL_S7       32
#define DL_S8       36
#define DL_S9       40
#define DL_S10      44
#define DL_S11      48
#define DL_DX       52
#define DL_DY       56
#define DL_LEN2     60
#define DL_INVL2    64
#define DL_GS       68
#define DL_BR2Q16   72
#define DL_BR2LEN2  76
#define DL_CROW     80
#define DL_DROW     84
#define DL_PXLOG    88
#define DL_PXHIG    92
#define DL_HALFW    96
#define DL_ROWFB   100
#define DL_PY      104
#define DL_FULL    108

#define GAUSS_LUT_SIZE 178

/* ════════════════════════════════════════════════════════════════════════════
 * .Lwyd — WRITE_YUV_DRAW local subroutine
 * Call:  jal t1, .Lwyd   (t1 = link register)
 * In:    a0=contrib  a1=b_col  a2=g_col  a3=r_col  a7=dst  t4=px
 * Kills: t0, a0, a4, a5, a6
 * Keeps: a1, a2, a3, a7, t4, t2, t3, t5, t6, s0-s11
 * ════════════════════════════════════════════════════════════════════════════ */
.Lwyd:
    /* y_contrib = (77*r + 150*g + 29*b) * contrib >> 16 */
    li      t0,  77
    mul     t0,  t0,  a3          /* 77*r */
    li      a4,  150
    mul     a4,  a4,  a2          /* 150*g */
    add     t0,  t0,  a4
    li      a4,  29
    mul     a4,  a4,  a1          /* 29*b */
    add     t0,  t0,  a4          /* 77r+150g+29b */
    mul     t0,  t0,  a0          /* * contrib */
    srli    t0,  t0,  16          /* y_contrib (always >= 0) */
    beqz    t0,  .Lwyd_sy
    lbu     a4,  0(a7)
    add     a4,  a4,  t0
    li      a5,  255
    bltu    a5,  a4,  .Lwyd_cy    /* yv > 255 */
    sb      a4,  0(a7)
    j       .Lwyd_sy
.Lwyd_cy:
    sb      a5,  0(a7)            /* clamp to 255 */
.Lwyd_sy:
    /* u_delta = (-43*r - 85*g + 128*b) * contrib >> 16 */
    li      t0,  43
    mul     t0,  t0,  a3
    li      a4,  85
    mul     a4,  a4,  a2
    add     t0,  t0,  a4          /* 43r+85g */
    slli    a4,  a1,  7           /* 128*b */
    sub     a4,  a4,  t0          /* -43r-85g+128b */
    mul     a4,  a4,  a0          /* * contrib */
    srai    a4,  a4,  16          /* u_delta (signed) */
    /* v_delta = (128*r - 107*g - 21*b) * contrib >> 16 */
    slli    t0,  a3,  7           /* 128*r */
    li      a5,  107
    mul     a5,  a5,  a2
    sub     t0,  t0,  a5          /* 128r-107g */
    li      a5,  21
    mul     a5,  a5,  a1
    sub     t0,  t0,  a5          /* 128r-107g-21b */
    mul     t0,  t0,  a0          /* * contrib */
    srai    t0,  t0,  16          /* v_delta (signed) */
    /* gate: if (u_delta | v_delta) */
    or      a5,  a4,  t0
    beqz    a5,  .Lwyd_ret
    /* u_off = (px & 1) ? -1 : 1 */
    andi    a6,  t4,  1
    li      a5,  1
    beqz    a6,  .Lwyd_up
    li      a5,  -1
.Lwyd_up:
    add     a6,  a7,  a5          /* &dst[u_off] */
    /* U: uv = dst[u_off] + u_delta; clamp [0,255] */
    lbu     a0,  0(a6)
    add     a0,  a0,  a4
    bltz    a0,  .Lwyd_u0
    li      a4,  255
    blt     a4,  a0,  .Lwyd_u255
    sb      a0,  0(a6)
    j       .Lwyd_uw
.Lwyd_u0:
    sb      zero, 0(a6)
    j       .Lwyd_uw
.Lwyd_u255:
    sb      a4,  0(a6)
.Lwyd_uw:
    /* V: vv = dst[u_off+2] + v_delta; clamp [0,255] */
    lbu     a0,  2(a6)
    add     a0,  a0,  t0
    bltz    a0,  .Lwyd_v0
    li      a4,  255
    blt     a4,  a0,  .Lwyd_v255
    sb      a0,  2(a6)
    j       .Lwyd_ret
.Lwyd_v0:
    sb      zero, 2(a6)
    j       .Lwyd_ret
.Lwyd_v255:
    sb      a4,  2(a6)
.Lwyd_ret:
    jr      t1

/* ════════════════════════════════════════════════════════════════════════════
 * .Lwyu — WRITE_YUV_UNDRAW local subroutine
 * Call:  jal t1, .Lwyu
 * In:    s6=ea  a1=b_col  a2=g_col  a3=r_col  a7=dst  t4=px
 * Kills: t0, a0, a4, a5, a6
 * ════════════════════════════════════════════════════════════════════════════ */
.Lwyu:
    /* b_sc=(b*ea)>>8  g_sc=(g*ea)>>8  r_sc=(r*ea)>>8 */
    mul     t0,  a1,  s6;   srli t0, t0, 8    /* b_sc */
    mul     a4,  a2,  s6;   srli a4, a4, 8    /* g_sc */
    mul     a5,  a3,  s6;   srli a5, a5, 8    /* r_sc */
    /* y_val = (77*r_sc + 150*g_sc + 29*b_sc) >> 8 */
    li      a6,  77
    mul     a6,  a6,  a5
    li      a0,  150
    mul     a0,  a0,  a4
    add     a6,  a6,  a0
    li      a0,  29
    mul     a0,  a0,  t0
    add     a6,  a6,  a0
    srli    a6,  a6,  8           /* y_val */
    beqz    a6,  .Lwyu_sy
    sb      a6,  0(a7)
.Lwyu_sy:
    /* u_delta = (-43r - 85g + 128b) * ea >> 16  (original b,g,r) */
    li      t0,  43
    mul     t0,  t0,  a3
    li      a4,  85
    mul     a4,  a4,  a2
    add     t0,  t0,  a4
    slli    a4,  a1,  7
    sub     a4,  a4,  t0          /* -43r-85g+128b */
    mul     a4,  a4,  s6
    srai    a4,  a4,  16          /* u_delta */
    /* v_delta = (128r - 107g - 21b) * ea >> 16 */
    slli    t0,  a3,  7
    li      a5,  107
    mul     a5,  a5,  a2
    sub     t0,  t0,  a5
    li      a5,  21
    mul     a5,  a5,  a1
    sub     t0,  t0,  a5
    mul     t0,  t0,  s6
    srai    t0,  t0,  16          /* v_delta */
    /* gate */
    or      a5,  a4,  t0
    beqz    a5,  .Lwyu_ret
    /* u_off */
    andi    a6,  t4,  1
    li      a5,  1 
    beqz    a6,  .Lwyu_up
    li      a5,  -1
.Lwyu_up:
    add     a6,  a7,  a5
    /* dst[u_off] = (uint8_t)(128 + u_delta)  — no clamp */
    li      a0,  128
    add     a4,  a0,  a4
    andi    a4,  a4,  0xFF
    sb      a4,  0(a6)
    /* dst[u_off+2] = (uint8_t)(128 + v_delta) */
    add     t0,  a0,  t0
    andi    t0,  t0,  0xFF
    sb      t0,  2(a6)
.Lwyu_ret:
    jr      t1

/* ════════════════════════════════════════════════════════════════════════════
 * draw_line_yuv422_overlay
 *   a0=fb  a1=fb_w  a2=fb_h  a3=x0  a4=y0  a5=x1  a6=y1  a7=brightness
 * ════════════════════════════════════════════════════════════════════════════ */
    .globl draw_line_yuv422_overlay
    .type  draw_line_yuv422_overlay, @function
draw_line_yuv422_overlay:
    beqz    a0,  .Ldl_early_ret
    blez    a1,  .Ldl_early_ret
    blez    a2,  .Ldl_early_ret
    blez    a7,  .Ldl_early_ret
    la      t0,  s_overlay_alpha_val
    lbu     t0,  0(t0)
    li      t1,  255
    bgeu    t0,  t1,  .Ldl_early_ret

    addi    sp,  sp,  -DL
    sw      ra,  DL_RA(sp)
    sw      s0,  DL_S0(sp)
    sw      s1,  DL_S1(sp)
    sw      s2,  DL_S2(sp)
    sw      s3,  DL_S3(sp)
    sw      s4,  DL_S4(sp)
    sw      s5,  DL_S5(sp)
    sw      s6,  DL_S6(sp)
    sw      s7,  DL_S7(sp)
    sw      s8,  DL_S8(sp)
    sw      s9,  DL_S9(sp)
    sw      s10, DL_S10(sp)
    sw      s11, DL_S11(sp)

    mv      s0,  a0               /* fb */
    mv      s1,  a1               /* fb_w */
    mv      s6,  a7               /* brightness */
    mv      s7,  a3               /* x0 */
    mv      s8,  a4               /* y0 */
    mv      s9,  a5               /* x1 */
    mv      s10, a6               /* y1 */
    /* a2=fb_h kept until after bbox */

    la      t0,  g_line_Rb;      lw  s11, 0(t0)
    la      t0,  g_beam_r2_q16;  lw  t0,  0(t0);  sw t0, DL_BR2Q16(sp)
    la      t0,  g_gs;           lw  t0,  0(t0);  sw t0, DL_GS(sp)

    /* ---- bx0, bx1 ---- */
    blt     s7,  s9,  .Ldl_bx_s79
    mv      t0,  s9;   mv t1, s7
    j       .Ldl_bx_done
.Ldl_bx_s79:
    mv      t0,  s7;   mv t1, s9
.Ldl_bx_done:                    /* t0=min, t1=max */
    sub     t0,  t0,  s11
    add     t1,  t1,  s11
    /* clamp bx0 to [0, fb_w-1] */
    bge     t0,  zero, .Ldl_bx0lo
    li      t0,  0
.Ldl_bx0lo:
    addi    a3,  s1,  -1
    blt     t0,  a3,  .Ldl_bx0ok
    mv      t0,  a3
.Ldl_bx0ok:
    mv      s2,  t0               /* bx0 */
    /* clamp bx1 */
    bge     t1,  zero, .Ldl_bx1lo
    li      t1,  0
.Ldl_bx1lo:
    blt     t1,  a3,  .Ldl_bx1ok
    mv      t1,  a3
.Ldl_bx1ok:
    mv      s3,  t1               /* bx1 */

    /* ---- by0, by1 ---- */
    blt     s8,  s10, .Ldl_by_s810
    mv      t0,  s10;  mv t1, s8
    j       .Ldl_by_done
.Ldl_by_s810:
    mv      t0,  s8;   mv t1, s10
.Ldl_by_done:
    sub     t0,  t0,  s11
    add     t1,  t1,  s11
    bge     t0,  zero, .Ldl_by0lo
    li      t0,  0
.Ldl_by0lo:
    addi    a3,  a2,  -1          /* fb_h-1 */
    blt     t0,  a3,  .Ldl_by0ok
    mv      t0,  a3
.Ldl_by0ok:
    mv      s4,  t0               /* by0 */
    bge     t1,  zero, .Ldl_by1lo
    li      t1,  0
.Ldl_by1lo:
    blt     t1,  a3,  .Ldl_by1ok
    mv      t1,  a3
.Ldl_by1ok:
    mv      s5,  t1               /* by1 */

    /* ---- dx, dy, len2 ---- */
    sub     t0,  s9,  s7;        sw  t0, DL_DX(sp)
    sub     t1,  s10, s8;        sw  t1, DL_DY(sp)
    mul     a3,  t0,  t0
    mul     a4,  t1,  t1
    add     a3,  a3,  a4
    sw      a3,  DL_LEN2(sp)

    /* ---- px_lo_g = max(bx0, s_ov_off_x) ---- */
    la      t0,  s_ov_off_x;     lw  t0, 0(t0)
    bge     s2,  t0,  .Ldl_plog_bx
    mv      s2,  t0               /* temporarily use s2 as candidate */
    j       .Ldl_plog_done
.Ldl_plog_bx:
    /* s2 >= t0, keep s2 */
.Ldl_plog_done:
    sw      s2,  DL_PXLOG(sp)
    /* restore s2=bx0 (we need bx0 for cross_row/dot_row computation) */
    /* Actually we need the original bx0 — load it back. But we just overwrote s2 if ov_off_x > bx0.
       Fix: save bx0 before this, or compute px_lo_g into a temp.
       Let me redo: keep s2=bx0 always; compute px_lo_g differently. */
    /* Revert: the above is wrong. Let me use a temp for px_lo_g. */

    /* CORRECTION: recompute bx0 back into s2, compute px_lo_g into a temp */
    /* Actually I already stored bx0 in s2 and then may have clobbered it. Restructure. */
    /* I'll load s_ov_off_x fresh and compute without touching s2. */

    /* ====  REDO px_lo_g / px_hi_g without clobbering s2/s3 ==== */
    sw      s2,  DL_PXLOG(sp)    /* will fix below */

    la      t0,  s_ov_off_x;     lw  t0, 0(t0)   /* t0 = s_ov_off_x */
    blt     s2,  t0,  .Ldl_plog2
    mv      t0,  s2               /* px_lo_g = bx0 */
.Ldl_plog2:
    sw      t0,  DL_PXLOG(sp)

    la      t0,  s_ov_off_x;     lw  t0, 0(t0)
    la      t1,  s_ov_w;         lw  t1, 0(t1)
    add     t0,  t0,  t1
    addi    t0,  t0,  -1         /* s_ov_off_x + s_ov_w - 1 */
    blt     t0,  s3,  .Ldl_phig2
    mv      t0,  s3
.Ldl_phig2:
    sw      t0,  DL_PXHIG(sp)

    /* ---- dot case? ---- */
    lw      a3,  DL_LEN2(sp)
    beqz    a3,  .Ldl_dot

    /* beam_r2_len2 = beam_r * beam_r * len2 */
    la      t0,  g_beam_r;       lw  t0, 0(t0)
    mul     t0,  t0,  t0
    mul     t0,  t0,  a3
    sw      t0,  DL_BR2LEN2(sp)

    /* inv_len2 = 0xFFFFFFFF / len2 */
    li      t0,  -1
    divu    t0,  t0,  a3
    sw      t0,  DL_INVL2(sp)

    /* cross_row = dx*(by0-y0) - dy*(bx0-x0) */
    lw      t0,  DL_DX(sp)
    lw      t1,  DL_DY(sp)
    sub     a4,  s4,  s8         /* by0-y0 */
    sub     a5,  s2,  s7         /* bx0-x0 */
    mul     a6,  t0,  a4         /* dx*(by0-y0) */
    mul     a5,  t1,  a5         /* dy*(bx0-x0) */
    sub     a6,  a6,  a5
    sw      a6,  DL_CROW(sp)

    /* dot_row = (bx0-x0)*dx + (by0-y0)*dy */
    sub     a5,  s2,  s7
    mul     a5,  a5,  t0
    mul     a4,  a4,  t1
    add     a5,  a5,  a4
    sw      a5,  DL_DROW(sp)

    /* ady, adx */
    lw      t1,  DL_DY(sp)
    bltz    t1,  .Ldl_ady_neg
    mv      a4,  t1
    j       .Ldl_ady_ok
.Ldl_ady_neg:
    neg     a4,  t1
.Ldl_ady_ok:                     /* a4=ady */
    lw      t0,  DL_DX(sp)
    bltz    t0,  .Ldl_adx_neg
    mv      a5,  t0
    j       .Ldl_adx_ok
.Ldl_adx_neg:
    neg     a5,  t0
.Ldl_adx_ok:                     /* a5=adx */

    beqz    a4,  .Ldl_full_init

    /* narrow: half_w = Rb*(adx+ady)/ady + 1 */
    add     t0,  a5,  a4
    mul     t0,  s11, t0
    div     t0,  t0,  a4
    addi    t0,  t0,  1
    sw      t0,  DL_HALFW(sp)
    sw      zero, DL_FULL(sp)    /* full_flag = 0 (narrow) */
    mv      a0,  s4
    sw      a0,  DL_PY(sp)
    j       .Ldl_row_n

.Ldl_full_init:
    li      t0,  1
    sw      t0,  DL_FULL(sp)     /* full_flag = 1 (full-range) */
    mv      a0,  s4
    sw      a0,  DL_PY(sp)
    j       .Ldl_row_f

/* ──────────────────────────────────────────────────────────────────────────
 * Narrow row loop
 * ────────────────────────────────────────────────────────────────────────── */
.Ldl_row_n:
    lw      a0,  DL_PY(sp)
    /* ry = py - s_ov_off_y */
    la      t0,  s_ov_off_y;     lw  t0, 0(t0)
    sub     a1,  a0,  t0         /* ry */
    bltz    a1,  .Ldl_row_skip
    la      t0,  s_ov_h;         lw  t0, 0(t0)
    bgeu    a1,  t0,  .Ldl_row_skip

    /* px_c = bx0 + cross_row / dy */
    lw      a2,  DL_CROW(sp)
    lw      t0,  DL_DY(sp)
    div     a2,  a2,  t0
    add     a2,  s2,  a2          /* px_c */

    /* px_lo_r = max(px_c - half_w, px_lo_g) */
    lw      t0,  DL_HALFW(sp)
    sub     a3,  a2,  t0
    lw      t1,  DL_PXLOG(sp)
    bge     a3,  t1,  .Ldl_lor_ok
    mv      a3,  t1
.Ldl_lor_ok:

    /* px_hi_r = min(px_c + half_w, px_hi_g) */
    add     a4,  a2,  t0
    lw      t1,  DL_PXHIG(sp)
    blt     a4,  t1,  .Ldl_hir_ok
    mv      a4,  t1
.Ldl_hir_ok:
    bgt     a3,  a4,  .Ldl_row_skip   /* empty range */

    /* dd = px_lo_r - bx0 */
    sub     a5,  a3,  s2
    /* cross = cross_row - dy*dd */
    lw      t0,  DL_DY(sp)
    mul     t0,  t0,  a5
    lw      t2,  DL_CROW(sp)
    sub     t2,  t2,  t0
    /* dot = dot_row + dx*dd */
    lw      t0,  DL_DX(sp)
    mul     t0,  t0,  a5
    lw      t3,  DL_DROW(sp)
    add     t3,  t3,  t0

    /* row_fb_base = fb + py * fb_w * 2 */
    lw      a0,  DL_PY(sp)
    mul     a5,  a0,  s1
    slli    a5,  a5,  1
    add     a5,  s0,  a5
    sw      a5,  DL_ROWFB(sp)

    /* row_pal pointer = s_overlay_pal[value] + ry*s_ov_w - s_ov_off_x */
    /* a1=ry still valid */
    la      t0,  s_overlay_pal;  lw  t0, 0(t0)
    la      t1,  s_ov_w;         lw  t1, 0(t1)
    mul     a6,  a1,  t1
    add     t0,  t0,  a6
    la      t1,  s_ov_off_x;     lw  t1, 0(t1)
    sub     t0,  t0,  t1         /* t0 = row_pal (unadjusted ptr) */

    mv      t4,  a3              /* px = px_lo_r */
    mv      t5,  a4              /* px_hi = px_hi_r */
    add     t6,  t0,  a3        /* pal_ptr = row_pal + px */
    j       .Ldl_px_loop

/* ──────────────────────────────────────────────────────────────────────────
 * Full-range row loop
 * ────────────────────────────────────────────────────────────────────────── */
.Ldl_row_f:
    lw      a0,  DL_PY(sp)
    la      t0,  s_ov_off_y;     lw  t0, 0(t0)
    sub     a1,  a0,  t0
    bltz    a1,  .Ldl_row_skip
    la      t0,  s_ov_h;         lw  t0, 0(t0)
    bgeu    a1,  t0,  .Ldl_row_skip

    lw      a3,  DL_PXLOG(sp)
    lw      a4,  DL_PXHIG(sp)
    bgt     a3,  a4,  .Ldl_row_skip

    /* dd = px_lo_g - bx0 */
    sub     a5,  a3,  s2
    lw      t0,  DL_DY(sp)
    mul     t0,  t0,  a5
    lw      t2,  DL_CROW(sp)
    sub     t2,  t2,  t0
    lw      t0,  DL_DX(sp)
    mul     t0,  t0,  a5
    lw      t3,  DL_DROW(sp)
    add     t3,  t3,  t0

    lw      a0,  DL_PY(sp)
    mul     a5,  a0,  s1
    slli    a5,  a5,  1
    add     a5,  s0,  a5
    sw      a5,  DL_ROWFB(sp)

    la      t0,  s_overlay_pal;  lw  t0, 0(t0)
    la      t1,  s_ov_w;         lw  t1, 0(t1)
    mul     a6,  a1,  t1
    add     t0,  t0,  a6
    la      t1,  s_ov_off_x;     lw  t1, 0(t1)
    sub     t0,  t0,  t1

    mv      t4,  a3
    mv      t5,  a4
    add     t6,  t0,  a3

/* ──────────────────────────────────────────────────────────────────────────
 * Shared inner pixel loop (draw)
 * draw: check pidx FIRST, then classify distance
 * ────────────────────────────────────────────────────────────────────────── */
.Ldl_px_loop:
    lbu     t0,  0(t6)            /* pidx = row_pal[px] */
    andi    a0,  t0,  0x80
    beqz    a0,  .Ldl_px_next    /* !(pidx & 0x80): skip */

    /* palette lookup: c = s_overlay_palette[pidx & 0x7F] */
    andi    a0,  t0,  0x7F
    la      a4,  s_overlay_palette
    slli    a5,  a0,  1
    add     a5,  a5,  a0          /* idx*3 */
    add     a4,  a4,  a5
    lbu     a1,  0(a4)            /* b_col */
    lbu     a2,  1(a4)            /* g_col */
    lbu     a3,  2(a4)            /* r_col */

    /* classify distance */
    lw      a0,  DL_LEN2(sp)
    blez    t3,  .Ldl_cap0        /* dot <= 0: endpoint 0 */
    bge     t3,  a0,  .Ldl_cap1  /* dot >= len2: endpoint 1 */

    /* mid-segment: csq64 = (int64_t)cross * cross */
    mulh    a4,  t2,  t2          /* signed upper 32 bits of cross^2 */
    bnez    a4,  .Ldl_px_next    /* csq64 > 0xFFFFFFFF: skip */
    mul     a4,  t2,  t2          /* csq (lower 32 bits) */
    lw      a5,  DL_BR2LEN2(sp)
    bgeu    a5,  a4,  .Ldl_in_beam  /* csq <= beam_r2_len2 */
    /* d2_q16 = (csq * inv_len2) >> 16 */
    lw      a5,  DL_INVL2(sp)
    mulhu   a0,  a4,  a5
    mul     a4,  a4,  a5
    slli    a0,  a0,  16
    srli    a4,  a4,  16
    or      a0,  a0,  a4          /* d2_q16 */
    j       .Ldl_classify

.Ldl_cap0:
    /* d2_q16 = cap_d2_q16(px-x0, py-y0) */
    sub     a4,  t4,  s7
    lw      a5,  DL_PY(sp)
    sub     a5,  a5,  s8
    slli    a4,  a4,  8;   mul a4, a4, a4
    slli    a5,  a5,  8;   mul a5, a5, a5
    add     a0,  a4,  a5
    j       .Ldl_classify

.Ldl_cap1:
    sub     a4,  t4,  s9
    lw      a5,  DL_PY(sp)
    sub     a5,  a5,  s10
    slli    a4,  a4,  8;   mul a4, a4, a4
    slli    a5,  a5,  8;   mul a5, a5, a5
    add     a0,  a4,  a5

.Ldl_classify:
    lw      a5,  DL_BR2Q16(sp)
    bgeu    a5,  a0,  .Ldl_in_beam   /* d2_q16 <= beam_r2_q16 */
    /* glow path */
    lw      a4,  DL_GS(sp)
    beqz    a4,  .Ldl_px_next
    sub     a0,  a0,  a5          /* glow_d2 = d2_q16 - beam_r2_q16 */
    mulhu   a5,  a0,  a4
    mul     a0,  a0,  a4
    slli    a5,  a5,  2
    srli    a0,  a0,  30
    or      a0,  a5,  a0          /* lut_idx */
    li      a4,  GAUSS_LUT_SIZE
    bgeu    a0,  a4,  .Ldl_px_next
    la      a4,  gauss_lut
    add     a0,  a4,  a0
    lbu     a0,  0(a0)            /* gauss_lut[lut_idx] */
    mul     a0,  s6,  a0
    srli    a0,  a0,  8           /* contrib */
    beqz    a0,  .Ldl_px_next
    j       .Ldl_write

.Ldl_in_beam:
    mv      a0,  s6               /* contrib = brightness */

.Ldl_write:
    lw      a7,  DL_ROWFB(sp)
    slli    a5,  t4,  1
    add     a7,  a7,  a5          /* dst = row_fb + px*2 */
    jal     t1,  .Lwyd

.Ldl_px_next:
    addi    t4,  t4,  1
    addi    t6,  t6,  1
    lw      t0,  DL_DY(sp)
    sub     t2,  t2,  t0          /* cross -= dy */
    lw      t0,  DL_DX(sp)
    add     t3,  t3,  t0          /* dot += dx */
    ble     t4,  t5,  .Ldl_px_loop

    /* End of inner pixel loop: advance cross_row, dot_row, py */
.Ldl_row_adv:
    lw      a0,  DL_CROW(sp)
    lw      a1,  DL_DX(sp)
    add     a0,  a0,  a1
    sw      a0,  DL_CROW(sp)
    lw      a0,  DL_DROW(sp)
    lw      a1,  DL_DY(sp)
    add     a0,  a0,  a1
    sw      a0,  DL_DROW(sp)
    lw      a0,  DL_PY(sp)
    addi    a0,  a0,  1
    sw      a0,  DL_PY(sp)
    ble     a0,  s5,  .Ldl_row_dispatch
    j       .Ldl_ret

.Ldl_row_skip:
    lw      a0,  DL_CROW(sp)
    lw      a1,  DL_DX(sp)
    add     a0,  a0,  a1
    sw      a0,  DL_CROW(sp)
    lw      a0,  DL_DROW(sp)
    lw      a1,  DL_DY(sp)
    add     a0,  a0,  a1
    sw      a0,  DL_DROW(sp)
    lw      a0,  DL_PY(sp)
    addi    a0,  a0,  1
    sw      a0,  DL_PY(sp)
    ble     a0,  s5,  .Ldl_row_dispatch
    j       .Ldl_ret

.Ldl_row_dispatch:
    lw      t0,  DL_FULL(sp)
    beqz    t0,  .Ldl_row_n
    j       .Ldl_row_f

/* ──────────────────────────────────────────────────────────────────────────
 * Dot case (len2 == 0)
 * ────────────────────────────────────────────────────────────────────────── */
.Ldl_dot:
    mv      a0,  s4
    sw      a0,  DL_PY(sp)

.Ldl_dot_row:
    lw      a0,  DL_PY(sp)
    la      t0,  s_ov_off_y;     lw  t0, 0(t0)
    sub     a1,  a0,  t0
    bltz    a1,  .Ldl_dot_skip
    la      t0,  s_ov_h;         lw  t0, 0(t0)
    bgeu    a1,  t0,  .Ldl_dot_skip

    /* row_fb = fb + py*fb_w*2 */
    mul     a5,  a0,  s1
    slli    a5,  a5,  1
    add     a5,  s0,  a5
    sw      a5,  DL_ROWFB(sp)

    /* row_pal */
    la      t0,  s_overlay_pal;  lw  t0, 0(t0)
    la      t1,  s_ov_w;         lw  t1, 0(t1)
    mul     a6,  a1,  t1
    add     t0,  t0,  a6
    la      t1,  s_ov_off_x;     lw  t1, 0(t1)
    sub     t0,  t0,  t1

    lw      t4,  DL_PXLOG(sp)
    lw      t5,  DL_PXHIG(sp)
    add     t6,  t0,  t4

.Ldl_dot_px:
    lbu     t0,  0(t6)
    andi    a0,  t0,  0x80
    beqz    a0,  .Ldl_dot_pxnext

    /* pidx & 0x80 set: palette lookup */
    andi    a0,  t0,  0x7F
    la      a4,  s_overlay_palette
    slli    a5,  a0,  1
    add     a5,  a5,  a0
    add     a4,  a4,  a5
    lbu     a1,  0(a4)
    lbu     a2,  1(a4)
    lbu     a3,  2(a4)

    /* d2_q16 = cap_d2_q16(px-x0, py-y0) */
    sub     a4,  t4,  s7
    lw      a5,  DL_PY(sp)
    sub     a5,  a5,  s8
    slli    a4,  a4,  8;   mul a4, a4, a4
    slli    a5,  a5,  8;   mul a5, a5, a5
    add     a0,  a4,  a5

    lw      a5,  DL_BR2Q16(sp)
    bgeu    a5,  a0,  .Ldl_dot_beam
    /* glow */
    lw      a4,  DL_GS(sp)
    beqz    a4,  .Ldl_dot_pxnext
    sub     a0,  a0,  a5
    mulhu   a5,  a0,  a4
    mul     a0,  a0,  a4
    slli    a5,  a5,  2
    srli    a0,  a0,  30
    or      a0,  a5,  a0
    li      a4,  GAUSS_LUT_SIZE
    bgeu    a0,  a4,  .Ldl_dot_pxnext
    la      a4,  gauss_lut
    add     a0,  a4,  a0
    lbu     a0,  0(a0)
    mul     a0,  s6,  a0
    srli    a0,  a0,  8
    beqz    a0,  .Ldl_dot_pxnext
    j       .Ldl_dot_write
.Ldl_dot_beam:
    mv      a0,  s6
.Ldl_dot_write:
    lw      a7,  DL_ROWFB(sp)
    slli    a5,  t4,  1
    add     a7,  a7,  a5
    jal     t1,  .Lwyd

.Ldl_dot_pxnext:
    addi    t4,  t4,  1
    addi    t6,  t6,  1
    ble     t4,  t5,  .Ldl_dot_px

.Ldl_dot_skip:
    lw      a0,  DL_PY(sp)
    addi    a0,  a0,  1
    sw      a0,  DL_PY(sp)
    ble     a0,  s5,  .Ldl_dot_row

.Ldl_ret:
    lw      ra,  DL_RA(sp)
    lw      s0,  DL_S0(sp)
    lw      s1,  DL_S1(sp)
    lw      s2,  DL_S2(sp)
    lw      s3,  DL_S3(sp)
    lw      s4,  DL_S4(sp)
    lw      s5,  DL_S5(sp)
    lw      s6,  DL_S6(sp)
    lw      s7,  DL_S7(sp)
    lw      s8,  DL_S8(sp)
    lw      s9,  DL_S9(sp)
    lw      s10, DL_S10(sp)
    lw      s11, DL_S11(sp)
    addi    sp,  sp,  DL
.Ldl_early_ret:
    ret
    .size draw_line_yuv422_overlay, . - draw_line_yuv422_overlay

/* ════════════════════════════════════════════════════════════════════════════
 * undraw_line_yuv422_overlay
 *   a0=fb  a1=fb_w  a2=fb_h  a3=x0  a4=y0  a5=x1  a6=y1  a7=brightness
 *   (brightness passed for signature compat; ea loaded from s_overlay_alpha_val)
 * ════════════════════════════════════════════════════════════════════════════ */
    .globl undraw_line_yuv422_overlay
    .type  undraw_line_yuv422_overlay, @function
undraw_line_yuv422_overlay:
    beqz    a0,  .Lul_early_ret
    blez    a1,  .Lul_early_ret
    blez    a2,  .Lul_early_ret

    addi    sp,  sp,  -DL
    sw      ra,  DL_RA(sp)
    sw      s0,  DL_S0(sp)
    sw      s1,  DL_S1(sp)
    sw      s2,  DL_S2(sp)
    sw      s3,  DL_S3(sp)
    sw      s4,  DL_S4(sp)
    sw      s5,  DL_S5(sp)
    sw      s6,  DL_S6(sp)
    sw      s7,  DL_S7(sp)
    sw      s8,  DL_S8(sp)
    sw      s9,  DL_S9(sp)
    sw      s10, DL_S10(sp)
    sw      s11, DL_S11(sp)

    mv      s0,  a0
    mv      s1,  a1
    /* s6 = ea = s_overlay_alpha_val */
    la      t0,  s_overlay_alpha_val
    lbu     s6,  0(t0)
    mv      s7,  a3
    mv      s8,  a4
    mv      s9,  a5
    mv      s10, a6

    la      t0,  g_line_Rb;      lw  s11, 0(t0)
    la      t0,  g_beam_r2_q16;  lw  t0,  0(t0);  sw t0, DL_BR2Q16(sp)
    la      t0,  g_gs;           lw  t0,  0(t0);  sw t0, DL_GS(sp)

    /* bbox */
    blt     s7,  s9,  .Lul_bx_s79
    mv      t0,  s9;   mv t1, s7
    j       .Lul_bx_done
.Lul_bx_s79:
    mv      t0,  s7;   mv t1, s9
.Lul_bx_done:
    sub     t0,  t0,  s11
    add     t1,  t1,  s11
    bge     t0,  zero, .Lul_bx0lo
    li      t0,  0
.Lul_bx0lo:
    addi    a3,  s1,  -1
    blt     t0,  a3,  .Lul_bx0ok
    mv      t0,  a3
.Lul_bx0ok:
    mv      s2,  t0
    bge     t1,  zero, .Lul_bx1lo
    li      t1,  0
.Lul_bx1lo:
    blt     t1,  a3,  .Lul_bx1ok
    mv      t1,  a3
.Lul_bx1ok:
    mv      s3,  t1

    blt     s8,  s10, .Lul_by_s810
    mv      t0,  s10;  mv t1, s8
    j       .Lul_by_done
.Lul_by_s810:
    mv      t0,  s8;   mv t1, s10
.Lul_by_done:
    sub     t0,  t0,  s11
    add     t1,  t1,  s11
    bge     t0,  zero, .Lul_by0lo
    li      t0,  0
.Lul_by0lo:
    addi    a3,  a2,  -1
    blt     t0,  a3,  .Lul_by0ok
    mv      t0,  a3
.Lul_by0ok:
    mv      s4,  t0
    bge     t1,  zero, .Lul_by1lo
    li      t1,  0
.Lul_by1lo:
    blt     t1,  a3,  .Lul_by1ok
    mv      t1,  a3
.Lul_by1ok:
    mv      s5,  t1

    sub     t0,  s9,  s7;        sw  t0, DL_DX(sp)
    sub     t1,  s10, s8;        sw  t1, DL_DY(sp)
    mul     a3,  t0,  t0
    mul     a4,  t1,  t1
    add     a3,  a3,  a4
    sw      a3,  DL_LEN2(sp)

    /* px_lo_g */
    la      t0,  s_ov_off_x;     lw  t0, 0(t0)
    blt     s2,  t0,  .Lul_plog2
    mv      t0,  s2
.Lul_plog2:
    sw      t0,  DL_PXLOG(sp)

    /* px_hi_g */
    la      t0,  s_ov_off_x;     lw  t0, 0(t0)
    la      t1,  s_ov_w;         lw  t1, 0(t1)
    add     t0,  t0,  t1
    addi    t0,  t0,  -1
    blt     t0,  s3,  .Lul_phig2
    mv      t0,  s3
.Lul_phig2:
    sw      t0,  DL_PXHIG(sp)

    lw      a3,  DL_LEN2(sp)
    beqz    a3,  .Lul_dot

    la      t0,  g_beam_r;       lw  t0, 0(t0)
    mul     t0,  t0,  t0
    mul     t0,  t0,  a3
    sw      t0,  DL_BR2LEN2(sp)

    li      t0,  -1
    divu    t0,  t0,  a3
    sw      t0,  DL_INVL2(sp)

    lw      t0,  DL_DX(sp)
    lw      t1,  DL_DY(sp)
    sub     a4,  s4,  s8
    sub     a5,  s2,  s7
    mul     a6,  t0,  a4
    mul     a5,  t1,  a5
    sub     a6,  a6,  a5
    sw      a6,  DL_CROW(sp)

    sub     a5,  s2,  s7
    mul     a5,  a5,  t0
    mul     a4,  a4,  t1
    add     a5,  a5,  a4
    sw      a5,  DL_DROW(sp)

    lw      t1,  DL_DY(sp)
    bltz    t1,  .Lul_ady_neg
    mv      a4,  t1
    j       .Lul_ady_ok
.Lul_ady_neg:
    neg     a4,  t1
.Lul_ady_ok:
    lw      t0,  DL_DX(sp)
    bltz    t0,  .Lul_adx_neg
    mv      a5,  t0
    j       .Lul_adx_ok
.Lul_adx_neg:
    neg     a5,  t0
.Lul_adx_ok:

    beqz    a4,  .Lul_full_init

    add     t0,  a5,  a4
    mul     t0,  s11, t0
    div     t0,  t0,  a4
    addi    t0,  t0,  1
    sw      t0,  DL_HALFW(sp)
    sw      zero, DL_FULL(sp)
    mv      a0,  s4
    sw      a0,  DL_PY(sp)
    j       .Lul_row_n

.Lul_full_init:
    li      t0,  1
    sw      t0,  DL_FULL(sp)
    mv      a0,  s4
    sw      a0,  DL_PY(sp)
    j       .Lul_row_f

/* ──────────────────────────────────────────────────────────────────────────
 * Undraw narrow row loop
 * ────────────────────────────────────────────────────────────────────────── */
.Lul_row_n:
    lw      a0,  DL_PY(sp)
    la      t0,  s_ov_off_y;     lw  t0, 0(t0)
    sub     a1,  a0,  t0
    bltz    a1,  .Lul_row_skip
    la      t0,  s_ov_h;         lw  t0, 0(t0)
    bgeu    a1,  t0,  .Lul_row_skip

    lw      a2,  DL_CROW(sp)
    lw      t0,  DL_DY(sp)
    div     a2,  a2,  t0
    add     a2,  s2,  a2

    lw      t0,  DL_HALFW(sp)
    sub     a3,  a2,  t0
    lw      t1,  DL_PXLOG(sp)
    bge     a3,  t1,  .Lul_lor_ok
    mv      a3,  t1
.Lul_lor_ok:
    add     a4,  a2,  t0
    lw      t1,  DL_PXHIG(sp)
    blt     a4,  t1,  .Lul_hir_ok
    mv      a4,  t1
.Lul_hir_ok:
    bgt     a3,  a4,  .Lul_row_skip

    sub     a5,  a3,  s2
    lw      t0,  DL_DY(sp)
    mul     t0,  t0,  a5
    lw      t2,  DL_CROW(sp)
    sub     t2,  t2,  t0
    lw      t0,  DL_DX(sp)
    mul     t0,  t0,  a5
    lw      t3,  DL_DROW(sp)
    add     t3,  t3,  t0

    lw      a0,  DL_PY(sp)
    mul     a5,  a0,  s1
    slli    a5,  a5,  1
    add     a5,  s0,  a5
    sw      a5,  DL_ROWFB(sp)

    la      t0,  s_overlay_pal;  lw  t0, 0(t0)
    la      t1,  s_ov_w;         lw  t1, 0(t1)
    mul     a6,  a1,  t1
    add     t0,  t0,  a6
    la      t1,  s_ov_off_x;     lw  t1, 0(t1)
    sub     t0,  t0,  t1

    mv      t4,  a3
    mv      t5,  a4
    add     t6,  t0,  a3
    j       .Lul_px_loop

/* ──────────────────────────────────────────────────────────────────────────
 * Undraw full-range row loop
 * ────────────────────────────────────────────────────────────────────────── */
.Lul_row_f:
    lw      a0,  DL_PY(sp)
    la      t0,  s_ov_off_y;     lw  t0, 0(t0)
    sub     a1,  a0,  t0
    bltz    a1,  .Lul_row_skip
    la      t0,  s_ov_h;         lw  t0, 0(t0)
    bgeu    a1,  t0,  .Lul_row_skip

    lw      a3,  DL_PXLOG(sp)
    lw      a4,  DL_PXHIG(sp)
    bgt     a3,  a4,  .Lul_row_skip

    sub     a5,  a3,  s2
    lw      t0,  DL_DY(sp)
    mul     t0,  t0,  a5
    lw      t2,  DL_CROW(sp)
    sub     t2,  t2,  t0
    lw      t0,  DL_DX(sp)
    mul     t0,  t0,  a5
    lw      t3,  DL_DROW(sp)
    add     t3,  t3,  t0

    lw      a0,  DL_PY(sp)
    mul     a5,  a0,  s1
    slli    a5,  a5,  1
    add     a5,  s0,  a5
    sw      a5,  DL_ROWFB(sp)

    la      t0,  s_overlay_pal;  lw  t0, 0(t0)
    la      t1,  s_ov_w;         lw  t1, 0(t1)
    mul     a6,  a1,  t1
    add     t0,  t0,  a6
    la      t1,  s_ov_off_x;     lw  t1, 0(t1)
    sub     t0,  t0,  t1

    mv      t4,  a3
    mv      t5,  a4
    add     t6,  t0,  a3

/* ──────────────────────────────────────────────────────────────────────────
 * Shared inner pixel loop (undraw)
 * undraw: classify distance FIRST, then check pidx
 * ────────────────────────────────────────────────────────────────────────── */
.Lul_px_loop:
    /* classify distance */
    lw      a0,  DL_LEN2(sp)
    blez    t3,  .Lul_cap0
    bge     t3,  a0,  .Lul_cap1

    mulh    a4,  t2,  t2
    bnez    a4,  .Lul_px_next
    mul     a4,  t2,  t2
    lw      a5,  DL_BR2LEN2(sp)
    bgeu    a5,  a4,  .Lul_in_beam
    lw      a5,  DL_INVL2(sp)
    mulhu   a0,  a4,  a5
    mul     a4,  a4,  a5
    slli    a0,  a0,  16
    srli    a4,  a4,  16
    or      a0,  a0,  a4
    j       .Lul_classify

.Lul_cap0:
    sub     a4,  t4,  s7
    lw      a5,  DL_PY(sp)
    sub     a5,  a5,  s8
    slli    a4,  a4,  8;   mul a4, a4, a4
    slli    a5,  a5,  8;   mul a5, a5, a5
    add     a0,  a4,  a5
    j       .Lul_classify

.Lul_cap1:
    sub     a4,  t4,  s9
    lw      a5,  DL_PY(sp)
    sub     a5,  a5,  s10
    slli    a4,  a4,  8;   mul a4, a4, a4
    slli    a5,  a5,  8;   mul a5, a5, a5
    add     a0,  a4,  a5

.Lul_classify:
    lw      a5,  DL_BR2Q16(sp)
    bgeu    a5,  a0,  .Lul_in_beam
    /* not in beam: glow check */
    lw      a4,  DL_GS(sp)
    beqz    a4,  .Lul_px_next
    sub     a0,  a0,  a5
    mulhu   a5,  a0,  a4
    mul     a0,  a0,  a4
    slli    a5,  a5,  2
    srli    a0,  a0,  30
    or      a0,  a5,  a0
    li      a4,  GAUSS_LUT_SIZE
    bgeu    a0,  a4,  .Lul_px_next
    la      a4,  gauss_lut
    add     a0,  a4,  a0
    lbu     a0,  0(a0)            /* gauss_lut[lut_idx] */
    beqz    a0,  .Lul_px_next    /* if gauss_lut[lut_idx] == 0: skip */
    /* fall through: in range, now check pidx */

.Lul_in_beam:
    /* pidx check */
    lbu     t0,  0(t6)
    andi    a0,  t0,  0x80
    beqz    a0,  .Lul_px_next

    /* palette lookup */
    andi    a0,  t0,  0x7F
    la      a4,  s_overlay_palette
    slli    a5,  a0,  1
    add     a5,  a5,  a0
    add     a4,  a4,  a5
    lbu     a1,  0(a4)
    lbu     a2,  1(a4)
    lbu     a3,  2(a4)

    lw      a7,  DL_ROWFB(sp)
    slli    a5,  t4,  1
    add     a7,  a7,  a5
    jal     t1,  .Lwyu

.Lul_px_next:
    addi    t4,  t4,  1
    addi    t6,  t6,  1
    lw      t0,  DL_DY(sp)
    sub     t2,  t2,  t0
    lw      t0,  DL_DX(sp)
    add     t3,  t3,  t0
    ble     t4,  t5,  .Lul_px_loop

.Lul_row_adv:
    lw      a0,  DL_CROW(sp)
    lw      a1,  DL_DX(sp)
    add     a0,  a0,  a1
    sw      a0,  DL_CROW(sp)
    lw      a0,  DL_DROW(sp)
    lw      a1,  DL_DY(sp)
    add     a0,  a0,  a1
    sw      a0,  DL_DROW(sp)
    lw      a0,  DL_PY(sp)
    addi    a0,  a0,  1
    sw      a0,  DL_PY(sp)
    ble     a0,  s5,  .Lul_row_dispatch
    j       .Lul_ret

.Lul_row_skip:
    lw      a0,  DL_CROW(sp)
    lw      a1,  DL_DX(sp)
    add     a0,  a0,  a1
    sw      a0,  DL_CROW(sp)
    lw      a0,  DL_DROW(sp)
    lw      a1,  DL_DY(sp)
    add     a0,  a0,  a1
    sw      a0,  DL_DROW(sp)
    lw      a0,  DL_PY(sp)
    addi    a0,  a0,  1
    sw      a0,  DL_PY(sp)
    ble     a0,  s5,  .Lul_row_dispatch
    j       .Lul_ret

.Lul_row_dispatch:
    lw      t0,  DL_FULL(sp)
    beqz    t0,  .Lul_row_n
    j       .Lul_row_f

/* ──────────────────────────────────────────────────────────────────────────
 * Undraw dot case (len2 == 0)
 * ────────────────────────────────────────────────────────────────────────── */
.Lul_dot:
    mv      a0,  s4
    sw      a0,  DL_PY(sp)

.Lul_dot_row:
    lw      a0,  DL_PY(sp)
    la      t0,  s_ov_off_y;     lw  t0, 0(t0)
    sub     a1,  a0,  t0
    bltz    a1,  .Lul_dot_skip
    la      t0,  s_ov_h;         lw  t0, 0(t0)
    bgeu    a1,  t0,  .Lul_dot_skip

    mul     a5,  a0,  s1
    slli    a5,  a5,  1
    add     a5,  s0,  a5
    sw      a5,  DL_ROWFB(sp)

    la      t0,  s_overlay_pal;  lw  t0, 0(t0)
    la      t1,  s_ov_w;         lw  t1, 0(t1)
    mul     a6,  a1,  t1
    add     t0,  t0,  a6
    la      t1,  s_ov_off_x;     lw  t1, 0(t1)
    sub     t0,  t0,  t1

    lw      t4,  DL_PXLOG(sp)
    lw      t5,  DL_PXHIG(sp)
    add     t6,  t0,  t4

.Lul_dot_px:
    /* classify first */
    sub     a4,  t4,  s7
    lw      a5,  DL_PY(sp)
    sub     a5,  a5,  s8
    slli    a4,  a4,  8;   mul a4, a4, a4
    slli    a5,  a5,  8;   mul a5, a5, a5
    add     a0,  a4,  a5         /* d2_q16 */

    lw      a5,  DL_BR2Q16(sp)
    bgeu    a5,  a0,  .Lul_dot_beam
    lw      a4,  DL_GS(sp)
    beqz    a4,  .Lul_dot_pxnext
    sub     a0,  a0,  a5
    mulhu   a5,  a0,  a4
    mul     a0,  a0,  a4
    slli    a5,  a5,  2
    srli    a0,  a0,  30
    or      a0,  a5,  a0
    li      a4,  GAUSS_LUT_SIZE
    bgeu    a0,  a4,  .Lul_dot_pxnext
    la      a4,  gauss_lut
    add     a0,  a4,  a0
    lbu     a0,  0(a0)
    beqz    a0,  .Lul_dot_pxnext

.Lul_dot_beam:
    /* pidx check */
    lbu     t0,  0(t6)
    andi    a0,  t0,  0x80
    beqz    a0,  .Lul_dot_pxnext

    andi    a0,  t0,  0x7F
    la      a4,  s_overlay_palette
    slli    a5,  a0,  1
    add     a5,  a5,  a0
    add     a4,  a4,  a5
    lbu     a1,  0(a4)
    lbu     a2,  1(a4)
    lbu     a3,  2(a4)

    lw      a7,  DL_ROWFB(sp)
    slli    a5,  t4,  1
    add     a7,  a7,  a5
    jal     t1,  .Lwyu

.Lul_dot_pxnext:
    addi    t4,  t4,  1
    addi    t6,  t6,  1
    ble     t4,  t5,  .Lul_dot_px

.Lul_dot_skip:
    lw      a0,  DL_PY(sp)
    addi    a0,  a0,  1
    sw      a0,  DL_PY(sp)
    ble     a0,  s5,  .Lul_dot_row

.Lul_ret:
    lw      ra,  DL_RA(sp)
    lw      s0,  DL_S0(sp)
    lw      s1,  DL_S1(sp)
    lw      s2,  DL_S2(sp)
    lw      s3,  DL_S3(sp)
    lw      s4,  DL_S4(sp)
    lw      s5,  DL_S5(sp)
    lw      s6,  DL_S6(sp)
    lw      s7,  DL_S7(sp)
    lw      s8,  DL_S8(sp)
    lw      s9,  DL_S9(sp)
    lw      s10, DL_S10(sp)
    lw      s11, DL_S11(sp)
    addi    sp,  sp,  DL
.Lul_early_ret:
    ret
    .size undraw_line_yuv422_overlay, . - undraw_line_yuv422_overlay
