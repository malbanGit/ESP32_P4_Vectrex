/* draw_line_dist_yuv422_overlay.S
 *
 * RV32IMAC hand-optimised assembly for:
 *   undraw_line_yuv422_overlay   (replaces C version; C renamed to _c)
 *   draw_line_yuv422_overlay     (replaces C version; C renamed to _c)
 *
 * Key optimisations over C:
 *   · Palette path: inner-loop px range precomputed as [px_lo, px_hi]
 *       px_lo = max(bx0, s_ov_off_x)
 *       px_hi = min(bx1, s_ov_off_x + s_ov_w - 1)
 *     Eliminates per-pixel rx-bounds check.
 *   · ea (s_overlay_alpha_val) checked once at function entry; no per-pixel reload.
 *   · pal_ptr and fb_ptr advanced incrementally (+1 / +2); no per-pixel multiply.
 *   · BT.601 full-range BGR→YUV inlined; no function-call overhead.
 * 
 * YUV422 (YUYV) pixel layout: [Y0][U][Y1][V] per 4-byte pair, 2 bytes/pixel.
 *   Y for pixel px : row_fb[px*2 + 0]
 *   U (pair)  : row_fb[(px&~1)*2 + 1]   (even: dst[+1], odd: dst[-1])
 *   V (pair)  : row_fb[(px&~1)*2 + 3]   (even: dst[+3], odd: dst[+1])
 * U/V are written for every pixel (both even and odd) to eliminate YUV422
 * chroma artifacts on nearly-vertical lines.
 *
 * BT.601 full-range integer formulas (77+150+29=256):
 *   Y = (77*R + 150*G + 29*B) >> 8          range [0,255] — no clamp needed
 *   U = 128 + ((128*B - 43*R - 85*G) >> 8)  clamped [0,255]
 *   V = 128 + ((128*R - 107*G - 21*B) >> 8) clamped [0,255]
 */

    .extern g_line_glow
    .extern g_line_width
    .extern g_line_Rb

    .section .iram0.text,"ax",@progbits
    .align 2

/* ══════════════════════════════════════════════════════════════════════════
 *  undraw_line_yuv422_overlay
 *
 *  void undraw_line_yuv422_overlay(
 *    a0=fb, a1=fb_w, a2=fb_h,
 *    a3=x0, a4=y0,   a5=x1, a6=y1, a7=brightness,
 *    [sp+0]=overlay )
 *
 *  Register map:
 *    s0  fb          s6  ea (s_overlay_alpha_val)
 *    s1  fb_w        s7  s_overlay_pal
 *    s2  px_lo       s8  &s_overlay_palette[0]
 *    s3  px_hi       s9  s_ov_off_x
 *    s4  by0         s10 s_ov_w
 *    s5  by1         s11 py
 *
 *  Stack frame 128 bytes:
 *    sp+ 0  ra       sp+52  x0      sp+68  s_ov_off_y    sp+ 88  dy
 *    sp+ 4  s0       sp+56  y0      sp+72  s_ov_h        sp+ 92  dx
 *    sp+ 8  s1       sp+60  x1      sp+76  bx0_orig      sp+ 96  half_cross
 *    sp+12  s2       sp+64  y1      sp+80  bx1_orig      sp+100  Cpy
 *    sp+16  s3                      sp+84  span2 (bg)    sp+104  px_lo_g
 *    sp+20  s4                                           sp+108  px_hi_g
 *    sp+24  s5       sp+128  overlay   (caller)          sp+112..127  pad
 *    sp+28  s6
 *    sp+32  s7
 *    sp+36  s8
 *    sp+40  s9
 *    sp+44  s10
 *    sp+48  s11
 * ══════════════════════════════════════════════════════════════════════════ */

#define UF         128
#define UF_RA        0
#define UF_X0       52
#define UF_Y0       56
#define UF_X1       60
#define UF_Y1       64
#define UF_OFFSY    68
#define UF_OVH      72
#define UF_BX0      76
#define UF_BX1      80
#define UF_SPAN2    84
#define UF_DY       88
#define UF_DX       92
#define UF_HCROSS   96
#define UF_CPY     100
#define UF_PXLOG   104
#define UF_PXHIG   108
#define UF_OV      (UF+0)

/* Distance-field undraw: reuse/extend stack slots (pal path only) */
#define UYDF_LEN2  96    /* overlaps UF_HCROSS — repurposed */
#define UYDF_INV   100   /* overlaps UF_CPY    — repurposed */
#define UYDF_GS    112
#define UYDF_CROW  116
#define UYDF_DROW  120
#define UYDF_HALFW 124   /* half_w = half_cross/|dy|+1 for per-row narrowing */
#define UYDF_BR2L   84   /* beam_r^2 * len2 (reuses freed UF_SPAN2 slot) */
/* beam_r2_q16 cached in a5 throughout pal path (not stored on stack) */

    .globl undraw_line_yuv422_overlay
    .type  undraw_line_yuv422_overlay, @function

undraw_line_yuv422_overlay:
    addi  sp, sp, -UF
    sw    ra,  0(sp)
    sw    s0,  4(sp)
    sw    s1,  8(sp)
    sw    s2,  12(sp)
    sw    s3,  16(sp)
    sw    s4,  20(sp)
    sw    s5,  24(sp)
    sw    s6,  28(sp)
    sw    s7,  32(sp)
    sw    s8,  36(sp)
    sw    s9,  40(sp)
    sw    s10, 44(sp)
    sw    s11, 48(sp)

    sw    a3, UF_X0(sp)
    sw    a4, UF_Y0(sp)
    sw    a5, UF_X1(sp)
    sw    a6, UF_Y1(sp)

    mv    s0, a0           /* fb  */
    mv    s1, a1           /* fb_w */

    /* Rb = g_line_Rb */
    la    t2, g_line_Rb
    lw    t2, 0(t2)

    /* bx0 = iclamp(min(x0,x1)-Rb, 0, fw-1) */
    lw    a3, UF_X0(sp); lw a5, UF_X1(sp)
    blt   a3, a5, 1f; mv a3, a5
1:  sub   a3, a3, t2
    bgez  a3, 1f; li a3, 0
1:  addi  t3, s1, -1
    blt   a3, t3, 1f; mv a3, t3
1:  sw    a3, UF_BX0(sp)

    /* bx1 = iclamp(max(x0,x1)+Rb, 0, fw-1) */
    lw    a3, UF_X0(sp); lw a5, UF_X1(sp)
    bgt   a3, a5, 1f; mv a3, a5
1:  add   a3, a3, t2
    bgez  a3, 1f; li a3, 0
1:  blt   a3, t3, 1f; mv a3, t3
1:  sw    a3, UF_BX1(sp)

    /* span2 = (bx1 - bx0 + 1) * 2 */
    lw    t4, UF_BX0(sp)
    sub   t5, a3, t4
    addi  t5, t5, 1
    slli  t5, t5, 1
    sw    t5, UF_SPAN2(sp)

    /* by0 = iclamp(min(y0,y1)-Rb, 0, fh-1) */
    lw    a4, UF_Y0(sp); lw a6, UF_Y1(sp)
    blt   a4, a6, 1f; mv a4, a6
1:  sub   a4, a4, t2
    bgez  a4, 1f; li a4, 0
1:  addi  t3, a2, -1
    blt   a4, t3, 1f; mv a4, t3
1:  mv    s4, a4

    /* by1 = iclamp(max(y0,y1)+Rb, 0, fh-1) */
    lw    a4, UF_Y0(sp); lw a6, UF_Y1(sp)
    bgt   a4, a6, 1f; mv a4, a6
1:  add   a4, a4, t2
    bgez  a4, 1f; li a4, 0
1:  blt   a4, t3, 1f; mv a4, t3
1:  mv    s5, a4

    /* Load overlay globals */
    la    t0, s_overlay_pal; lw s7, 0(t0)
    la    s8, s_overlay_palette_yuv_ea  /* ea-prescaled YUV palette for undraw restore */
    la    t0, s_overlay_alpha_val; lbu s6, 0(t0)
    la    t0, s_ov_off_x; lw s9, 0(t0)
    la    t0, s_ov_w;     lw s10, 0(t0)
    la    t0, s_ov_off_y; lw t1, 0(t0); sw t1, UF_OFFSY(sp)
    la    t0, s_ov_h;     lw t1, 0(t0); sw t1, UF_OVH(sp)

    /* px_lo_g = max(bx0, s_ov_off_x); px_hi_g = min(bx1, s_ov_off_x+s_ov_w-1) */
    lw    t0, UF_BX0(sp)
    blt   s9, t0, 1f; mv t0, s9
1:  sw    t0, UF_PXLOG(sp)

    lw    t0, UF_BX1(sp)
    addi  t1, s10, -1; add t1, t1, s9
    blt   t0, t1, 1f; mv t0, t1
1:  sw    t0, UF_PXHIG(sp)

    /* dx = x1 - x0;  dy = y1 - y0 */
    lw    t0, UF_X0(sp); lw t1, UF_X1(sp); sub t0, t1, t0; sw t0, UF_DX(sp)
    lw    t0, UF_Y0(sp); lw t1, UF_Y1(sp); sub t0, t1, t0; sw t0, UF_DY(sp)

    /* half_cross = Rb*(|dx|+|dy|) — t2 still holds Rb */
    lw    t0, UF_DX(sp); bgez t0, 1f; sub t0, zero, t0
1:  lw    t1, UF_DY(sp); bgez t1, 2f; sub t1, zero, t1
2:  add   t0, t0, t1; mul t0, t0, t2; sw t0, UF_HCROSS(sp)

    /* Cpy = dx*(by0-y0) + dy*x0 */
    lw    t0, UF_Y0(sp); sub t1, s4, t0
    lw    t0, UF_DX(sp); mul t1, t0, t1
    lw    t0, UF_DY(sp); lw t3, UF_X0(sp); mul t0, t0, t3
    add   t0, t0, t1; sw t0, UF_CPY(sp)

    j     .Luyd_pal_path

/* ──────────────────────────────────────────────────────────────────────────
 * PALETTE PATH — distance-field undraw
 * Runs the identical distance-field algorithm as the draw path to find
 * exactly which pixels were drawn, then restores each from the palette.
 * PSRAM is accessed only for ~length×Rb pixels (those within beam/glow).
 * Register a5 = beam_r2_q16 (cached for the entire palette path).
 * ────────────────────────────────────────────────────────────────────────── */
.Luyd_pal_path:
    lw    s2, UF_PXLOG(sp)
    lw    s3, UF_PXHIG(sp)
    bgt   s2, s3, .Luyd_ret
    blt   s5, s4, .Luyd_ret

    /* half_w = half_cross/|dy|+1 — read UF_HCROSS before slot is repurposed below */
    lw    t0, UF_HCROSS(sp)          /* half_cross = Rb*(|dx|+|dy|) */
    lw    t1, UF_DY(sp)
    beqz  t1, .Luydf_idy0
    bgez  t1, 1f; neg t1, t1
1:  divu  t0, t0, t1
    addi  t0, t0, 1
    sw    t0, UYDF_HALFW(sp)
    j     .Luydf_idy_done
.Luydf_idy0:
    sw    zero, UYDF_HALFW(sp)       /* dy=0: no narrowing (sentinel) */
.Luydf_idy_done:

    /* len2 = dx^2 + dy^2 */
    lw    a0, UF_DX(sp); lw a1, UF_DY(sp)
    mul   a2, a0, a0; mul a3, a1, a1; add a2, a2, a3
    sw    a2, UYDF_LEN2(sp)

    /* beam_r2_q16 = (beam_r<<8)^2;  beam_r = g_line_width >> 1 */
    la    t0, g_line_width; lw t0, 0(t0); srai t0, t0, 1
    slli  t0, t0, 8; mul t0, t0, t0
    mv    a5, t0                   /* a5 = beam_r2_q16 (cached) */

    /* beam_r2_len2 = beam_r^2 * len2 (fast beam check: no mulhu needed) */
    la    t0, g_line_width; lw t0, 0(t0); srai t0, t0, 1
    mul   t0, t0, t0
    lw    t1, UYDF_LEN2(sp)
    mul   t0, t0, t1
    sw    t0, UYDF_BR2L(sp)

    /* gs = (glow==0) ? 0 : (177<<14)/glow^2 */
    la    t0, g_line_glow; lw t0, 0(t0)
    bgez  t0, 1f; li t0, 0
1:  beqz  t0, .Luydf_gs0
    mul   t1, t0, t0
    li    t0, 2899968; div t0, t0, t1
    sw    t0, UYDF_GS(sp)
    j     .Luydf_gsd
.Luydf_gs0:
    sw    zero, UYDF_GS(sp)
.Luydf_gsd:

    /* inv_len2; cross_row/dot_row from (px_lo_g, by0) */
    lw    a2, UYDF_LEN2(sp)
    beqz  a2, .Luydf_noinv
    li    t0, -1; divu t0, t0, a2; sw t0, UYDF_INV(sp)
    lw    t0, UF_X0(sp); sub a0, s2, t0    /* apx = px_lo_g - x0 */
    lw    t0, UF_Y0(sp); sub a1, s4, t0    /* apy = by0 - y0    */
    lw    t0, UF_DX(sp); lw t1, UF_DY(sp)
    mul   a2, t0, a1; mul a3, t1, a0; sub a2, a2, a3; sw a2, UYDF_CROW(sp)
    mul   a2, a0, t0; mul a3, a1, t1; add a2, a2, a3; sw a2, UYDF_DROW(sp)
    j     .Luydf_outer
.Luydf_noinv:
    sw    zero, UYDF_INV(sp)
    sw    zero, UYDF_CROW(sp)
    sw    zero, UYDF_DROW(sp)

.Luydf_outer:
    mv    s11, s4                  /* py = by0 */

.Luydf_row:
    bgt   s11, s5, .Luyd_ret

    /* ry = py - s_ov_off_y; skip row if outside overlay */
    lw    t0, UF_OFFSY(sp); sub t0, s11, t0
    lw    t1, UF_OVH(sp); bgeu t0, t1, .Luydf_row_next

    /* row_fb = fb + py*fw*2 */
    mul   t5, s11, s1; slli t5, t5, 1; add t5, s0, t5

    /* pal_ptr = s_overlay_pal + ry*s_ov_w - s_ov_off_x + px_lo_g */
    mul   t6, t0, s10; add t6, s7, t6; sub t6, t6, s9; add t6, t6, s2

    /* fb_ptr = row_fb + px_lo_g*2 */
    slli  a0, s2, 1; add t1, t5, a0

    lw    a6, UYDF_LEN2(sp)

    /* Per-row x narrowing: only iterate ~2*half_w pixels near the line */
    lw    a1, UF_DY(sp)
    beqz  a1, .Luydf_nonarrow
    lw    a0, UYDF_HALFW(sp)
    beqz  a0, .Luydf_nonarrow      /* dy=0 sentinel → full range */
    lw    t2, UYDF_CROW(sp)        /* cross at px_lo_g for this row */
    div   a3, t2, a1               /* cross_row / dy */
    add   a3, a3, s2               /* px_center = px_lo_g + cross_row/dy */
    sub   a2, a3, a0               /* px_lo_r = px_center - half_w */
    bge   a2, s2, 1f; mv a2, s2   /* clamp to px_lo_g */
1:  add   a3, a3, a0               /* px_hi_r = px_center + half_w */
    ble   a3, s3, 2f; mv a3, s3   /* clamp to px_hi_g */
2:  bgt   a2, a3, .Luydf_px_done  /* empty range: jump to row-update */
    /* advance cross/dot from px_lo_g to px_lo_r */
    sub   a4, a2, s2               /* delta = px_lo_r - px_lo_g */
    mul   a0, a1, a4               /* dy * delta */
    lw    t2, UYDF_CROW(sp); sub t2, t2, a0  /* cross_start */
    lw    t3, UYDF_DROW(sp)
    lw    a0, UF_DX(sp); mul a0, a0, a4; add t3, t3, a0  /* dot_start */
    add   t6, t6, a4               /* pal_ptr += delta */
    slli  a0, a4, 1; add t1, t1, a0  /* fb_ptr += delta*2 */
    mv    t4, a2                   /* px = px_lo_r */
    mv    t5, a3                   /* t5 = px_hi_r (repurposed from row_fb) */
    j     .Luydf_px

.Luydf_nonarrow:
    lw    t2, UYDF_CROW(sp); lw t3, UYDF_DROW(sp)
    mv    t4, s2                   /* px = px_lo_g */
    mv    t5, s3                   /* t5 = px_hi_g */

.Luydf_px:
    bgt   t4, t5, .Luydf_px_done

    /* ── Distance → d2_q16 in a0 ── */
    beqz  a6, .Luydf_point
    blez  t3, .Luydf_scap
    bge   t3, a6, .Luydf_ecap

    mul   a0, t2, t2
    mulh  a1, t2, t2               /* upper 32 bits of cross^2 */
    bnez  a1, .Luydf_pxskip        /* overflow → way outside glow, skip */
    lw    a1, UYDF_BR2L(sp)
    bgeu  a1, a0, .Luydf_restore   /* csq <= beam_r2_len2 → in beam */
    lw    a1, UYDF_INV(sp)
    mulhu a2, a0, a1; mul a0, a0, a1
    slli  a2, a2, 16; srli a0, a0, 16; or a0, a2, a0
    j     .Luydf_beam

.Luydf_scap:
    lw    a0, UF_X0(sp); lw a1, UF_Y0(sp)
    sub   a2, t4, a0; sub a3, s11, a1
    slli  a0, a2, 8; mul a0, a0, a0
    slli  a1, a3, 8; mul a1, a1, a1; add a0, a0, a1
    j     .Luydf_beam

.Luydf_ecap:
    lw    a0, UF_X1(sp); lw a1, UF_Y1(sp)
    sub   a2, t4, a0; sub a3, s11, a1
    slli  a0, a2, 8; mul a0, a0, a0
    slli  a1, a3, 8; mul a1, a1, a1; add a0, a0, a1
    j     .Luydf_beam

.Luydf_point:
    lw    a0, UF_X0(sp); lw a1, UF_Y0(sp)
    sub   a2, t4, a0; sub a3, s11, a1
    slli  a0, a2, 8; mul a0, a0, a0
    slli  a1, a3, 8; mul a1, a1, a1; add a0, a0, a1

.Luydf_beam:                       /* a0=d2_q16, a5=beam_r2_q16 */
    bgt   a0, a5, .Luydf_glow
    j     .Luydf_restore           /* inside beam → restore */

.Luydf_glow:
    lw    a1, UYDF_GS(sp); beqz a1, .Luydf_pxskip
    sub   a2, a0, a5               /* glow_d2 */
    mul   a3, a2, a1; mulh a4, a2, a1
    slli  a4, a4, 2; srli a3, a3, 30; or a0, a4, a3   /* lut_idx */
    li    a1, 178; bgeu a0, a1, .Luydf_pxskip
    la    a1, gauss_lut; add a1, a1, a0; lbu a1, 0(a1)
    beqz  a1, .Luydf_pxskip        /* gauss==0: pixel not drawn, skip */

.Luydf_restore:
    lbu   a0, 0(t6)                /* pidx */
    andi  a1, a0, 0x80; beqz a1, .Luydf_pxskip   /* transparent */
    andi  a0, a0, 0x7f
    slli  a1, a0, 1; add a1, a1, a0; add a1, s8, a1   /* &yuv_ea[idx] */

    lbu   a2, 0(a1)                /* Y (ea-scaled, precomputed) */
    sb    a2, 0(t1)

    andi  a2, t4, 1                /* u_ptr: t1+1 even, t1-1 odd */
    slli  a2, a2, 1
    li    a3, 1; sub a3, a3, a2; add a3, a3, t1

    lbu   a2, 1(a1)                /* U (ea-scaled, precomputed) */
    sb    a2, 0(a3)
    lbu   a2, 2(a1)                /* V (ea-scaled, precomputed) */
    sb    a2, 2(a3)

.Luydf_pxskip:
    addi  t6, t6, 1
    addi  t1, t1, 2
    beqz  a6, .Luydf_px_incr
    lw    a1, UF_DY(sp); sub t2, t2, a1
    lw    a1, UF_DX(sp); add t3, t3, a1
.Luydf_px_incr:
    addi  t4, t4, 1
    ble   t4, t5, .Luydf_px      /* t5 = px_hi_r (narrowed) */

.Luydf_px_done:
    beqz  a6, .Luydf_row_next
    lw    a0, UYDF_CROW(sp); lw a1, UF_DX(sp); add a0, a0, a1; sw a0, UYDF_CROW(sp)
    lw    a0, UYDF_DROW(sp); lw a1, UF_DY(sp); add a0, a0, a1; sw a0, UYDF_DROW(sp)
.Luydf_row_next:
    addi  s11, s11, 1
    j     .Luydf_row

.Luyd_ret:
    lw    ra,  0(sp)
    lw    s0,  4(sp)
    lw    s1,  8(sp)
    lw    s2,  12(sp)
    lw    s3,  16(sp)
    lw    s4,  20(sp)
    lw    s5,  24(sp)
    lw    s6,  28(sp)
    lw    s7,  32(sp)
    lw    s8,  36(sp)
    lw    s9,  40(sp)
    lw    s10, 44(sp)
    lw    s11, 48(sp)
    addi  sp, sp, UF
    ret
    .size undraw_line_yuv422_overlay, .-undraw_line_yuv422_overlay

/* ══════════════════════════════════════════════════════════════════════════
 *  draw_line_yuv422_overlay
 *
 *  void draw_line_yuv422_overlay(
 *    a0=fb, a1=fb_w, a2=fb_h,
 *    a3=x0, a4=y0, a5=x1, a6=y1, a7=brightness,
 *    [sp+0]=overlay )
 *
 *  Register map:
 *    s0  fb          s6  brightness
 *    s1  fb_w        s7  s_overlay_pal
 *    s2  px_lo       s8  &s_overlay_palette[0]
 *    s3  px_hi       s9  s_ov_off_x
 *    s4  by0         s10 s_ov_w
 *    s5  by1         s11 py
 *
 *  Stack frame 128 bytes:
 *    sp+  0  ra      sp+ 52  x0      sp+ 80  inv_len2
 *    sp+  4  s0      sp+ 56  y0      sp+ 84  gs
 *    sp+  8  s1      sp+ 60  x1      sp+ 88  beam_r2_q16
 *    sp+ 12  s2      sp+ 64  y1      sp+ 92  cross_row
 *    sp+ 16  s3      sp+ 68  dx      sp+ 96  dot_row
 *    sp+ 20  s4      sp+ 72  dy      sp+100  s_ov_off_y
 *    sp+ 24  s5      sp+ 76  len2    sp+104  s_ov_h
 *    sp+ 28  s6                      sp+108  bx0_orig
 *    sp+ 32  s7                      sp+112  bx1_orig
 *    sp+ 36  s8                      sp+116  ea
 *    sp+ 40  s9                      sp+120  (pad)
 *    sp+ 44  s10                     sp+124  (pad)
 *    sp+ 48  s11
 *    sp+128  overlay   (caller)
 * ══════════════════════════════════════════════════════════════════════════ */

#define DF         128
#define DF_RA        0
#define DF_X0       52
#define DF_Y0       56
#define DF_X1       60
#define DF_Y1       64
#define DF_DX       68
#define DF_DY       72
#define DF_LEN2     76
#define DF_INV      80
#define DF_GS       84
#define DF_BR2      88
#define DF_CROW     92
#define DF_DROW     96
#define DF_OFFSY   100
#define DF_OVH     104
#define DF_BX0     108
#define DF_BX1     112
#define DF_EA      116
#define DF_HALFW   120   /* half_w for per-row x narrowing */
#define DF_BR2L    124   /* beam_r^2 * len2 (fast beam check) */
#define DF_OVLAY   (DF+0)

    .globl draw_line_yuv422_overlay
    .type  draw_line_yuv422_overlay, @function

draw_line_yuv422_overlay:
    addi  sp, sp, -DF
    sw    ra,  0(sp)
    sw    s0,  4(sp)
    sw    s1,  8(sp)
    sw    s2,  12(sp)
    sw    s3,  16(sp)
    sw    s4,  20(sp)
    sw    s5,  24(sp)
    sw    s6,  28(sp)
    sw    s7,  32(sp)
    sw    s8,  36(sp)
    sw    s9,  40(sp)
    sw    s10, 44(sp)
    sw    s11, 48(sp)

    sw    a3, DF_X0(sp)
    sw    a4, DF_Y0(sp)
    sw    a5, DF_X1(sp)
    sw    a6, DF_Y1(sp)

    mv    s0, a0
    mv    s1, a1
    mv    s6, a7                    /* brightness */

    /* Load palette globals */
    la    t0, s_overlay_pal; lw s7, 0(t0)

    la    s8, s_overlay_palette_yuv      /* precomputed {Y,U,V} palette for draw */
    la    t0, s_ov_off_x;  lw s9,  0(t0)
    la    t0, s_ov_w;      lw s10, 0(t0)
    la    t0, s_ov_off_y;  lw t1, 0(t0);  sw t1, DF_OFFSY(sp)
    la    t0, s_ov_h;      lw t1, 0(t0);  sw t1, DF_OVH(sp)

    /* ea check: if overlay is fully opaque, draw is a no-op */
    la    t0, s_overlay_alpha_val; lbu t0, 0(t0)
    li    t1, 255
    bge   t0, t1, .Ldyv_ret
    sw    t0, DF_EA(sp)             /* store ea for inner loop blend */

    /* glow = max(g_line_glow, 0) */
    la    t0, g_line_glow; lw t0, 0(t0)
    bgez  t0, 1f; li t0, 0
1:                                  /* t0 = glow */

    /* beam_r = g_line_width >> 1;  Rb = g_line_Rb */
    la    t1, g_line_width; lw t1, 0(t1); srai t1, t1, 1
    la    t2, g_line_Rb;    lw t2, 0(t2)

    /* Bounding box */
    lw    a3, DF_X0(sp); lw a5, DF_X1(sp)
    blt   a3, a5, 1f; mv a3, a5
1:  sub   a3, a3, t2
    bgez  a3, 1f; li a3, 0
1:  addi  t3, s1, -1
    blt   a3, t3, 1f; mv a3, t3
1:  sw    a3, DF_BX0(sp)

    lw    a3, DF_X0(sp); lw a5, DF_X1(sp)
    bgt   a3, a5, 1f; mv a3, a5
1:  add   a3, a3, t2
    bgez  a3, 1f; li a3, 0
1:  blt   a3, t3, 1f; mv a3, t3
1:  sw    a3, DF_BX1(sp)

    lw    a4, DF_Y0(sp); lw a6, DF_Y1(sp)
    blt   a4, a6, 1f; mv a4, a6
1:  sub   a4, a4, t2
    bgez  a4, 1f; li a4, 0
1:  addi  t3, a2, -1
    blt   a4, t3, 1f; mv a4, t3
1:  mv    s4, a4

    lw    a4, DF_Y0(sp); lw a6, DF_Y1(sp)
    bgt   a4, a6, 1f; mv a4, a6
1:  add   a4, a4, t2
    bgez  a4, 1f; li a4, 0
1:  blt   a4, t3, 1f; mv a4, t3
1:  mv    s5, a4

    /* px_lo/px_hi for palette path */
    lw    t0, DF_BX0(sp)
    blt   s9, t0, 1f; mv t0, s9
1:  mv    s2, t0

    lw    t0, DF_BX1(sp)
    addi  t1, s10, -1; add t1, t1, s9
    blt   t0, t1, 1f; mv t0, t1
1:  mv    s3, t0

    /* dx, dy, len2 */
    lw    a3, DF_X0(sp); lw a5, DF_X1(sp); sub t0, a5, a3; sw t0, DF_DX(sp)
    lw    a4, DF_Y0(sp); lw a6, DF_Y1(sp); sub t1, a6, a4; sw t1, DF_DY(sp)
    mul   a3, t0, t0;  mul a4, t1, t1;  add a3, a3, a4;  sw a3, DF_LEN2(sp)

    /* half_w = Rb*(|dx|+|dy|)/|dy|+1 for per-row x narrowing */
    bgez  t0, 1f; neg t0, t0
1:  bgez  t1, 2f; neg t1, t1
2:  add   t0, t0, t1               /* |dx|+|dy| */
    la    t1, g_line_Rb; lw t1, 0(t1); mul t0, t0, t1   /* half_cross */
    lw    t1, DF_DY(sp); beqz t1, .Ldyv_halfw_dy0
    bgez  t1, 3f; neg t1, t1
3:  divu  t0, t0, t1; addi t0, t0, 1
    sw    t0, DF_HALFW(sp)
    j     .Ldyv_halfw_done
.Ldyv_halfw_dy0:
    sw    zero, DF_HALFW(sp)
.Ldyv_halfw_done:

    /* beam_r2_q16 = (beam_r<<8)^2 */
    la    t1, g_line_width; lw t1, 0(t1); srai t1, t1, 1
    slli  t1, t1, 8; mul t1, t1, t1; sw t1, DF_BR2(sp)

    /* beam_r2_len2 = beam_r^2 * len2 (fast beam check: no mulhu needed) */
    la    t1, g_line_width; lw t1, 0(t1); srai t1, t1, 1
    mul   t1, t1, t1
    lw    t0, DF_LEN2(sp)
    mul   t1, t1, t0
    sw    t1, DF_BR2L(sp)

    /* gs = (glow==0) ? 0 : (177<<14)/glow^2 */
    la    t0, g_line_glow; lw t0, 0(t0)
    bgez  t0, 1f; li t0, 0
1:  beqz  t0, .Ldyv_gs_zero
    mul   t1, t0, t0
    li    t0, 2899968           /* 177<<14 */
    div   t0, t0, t1;  sw t0, DF_GS(sp)
    j     .Ldyv_gs_done
.Ldyv_gs_zero:
    sw    zero, DF_GS(sp)
.Ldyv_gs_done:

    /* inv_len2, cross_row, dot_row (only when len2 > 0) */
    lw    a3, DF_LEN2(sp)
    beqz  a3, .Ldyv_setup_done

    li    t0, -1; divu t0, t0, a3; sw t0, DF_INV(sp)

    /* Cross/dot starting from px_lo */
    lw    t0, DF_X0(sp); sub a0, s2, t0    /* apx0 = px_lo - x0 */
    lw    t0, DF_Y0(sp); sub a1, s4, t0    /* apy0 = by0  - y0  */
    lw    t0, DF_DX(sp); lw  t1, DF_DY(sp)
    mul   a2, t0, a1; mul a3, t1, a0; sub a2, a2, a3; sw a2, DF_CROW(sp)
    mul   a2, a0, t0; mul a3, a1, t1; add a2, a2, a3; sw a2, DF_DROW(sp)

.Ldyv_setup_done:
    j     .Ldyv_pal_outer

/* ──────────────────────────────────────────────────────────────────────────
 * PALETTE OUTER LOOP
 * Draw: Y additive, U/V SET at even columns from blend-scaled BGR.
 * ────────────────────────────────────────────────────────────────────────── */
.Ldyv_pal_outer:
    blt   s3, s2, .Ldyv_ret
    mv    s11, s4

.Ldyv_pal_py:
    bgt   s11, s5, .Ldyv_ret

    /* ry = py - s_ov_off_y; skip if outside overlay */
    lw    t0, DF_OFFSY(sp); sub t0, s11, t0
    lw    t1, DF_OVH(sp); bgeu t0, t1, .Ldyv_pal_py_next

    /* row_fb = fb + py*fw*2 */
    mul   t5, s11, s1; slli t5, t5, 1; add t5, s0, t5

    /* pal_ptr = s_overlay_pal + ry*s_ov_w - s_ov_off_x + px_lo */
    mul   a0, t0, s10; add a0, s7, a0; sub a0, a0, s9; add t6, a0, s2

    /* fb_ptr = row_fb + px_lo*2 */
    slli  a0, s2, 1; add t1, t5, a0
    /* t5 (row_fb) is no longer needed; repurposed as px_hi_r below */

    lw    a6, DF_LEN2(sp); lw a7, DF_BR2(sp)

    /* Per-row x narrowing */
    lw    a1, DF_DY(sp)
    beqz  a1, .Ldyv_nonarrow
    lw    a0, DF_HALFW(sp)
    beqz  a0, .Ldyv_nonarrow       /* dy=0 sentinel → full range */
    lw    t2, DF_CROW(sp)          /* cross at px_lo for this row */
    div   a3, t2, a1               /* cross_row / dy */
    add   a3, a3, s2               /* px_center */
    sub   a2, a3, a0               /* px_lo_r = px_center - half_w */
    bge   a2, s2, 1f; mv a2, s2   /* clamp to px_lo_g */
1:  add   a3, a3, a0               /* px_hi_r = px_center + half_w */
    ble   a3, s3, 2f; mv a3, s3   /* clamp to px_hi_g */
2:  bgt   a2, a3, .Ldyv_pal_px_done  /* empty → jump to row update */
    /* advance cross/dot from px_lo_g to px_lo_r */
    sub   a4, a2, s2               /* delta = px_lo_r - px_lo_g */
    mul   a0, a1, a4
    lw    t2, DF_CROW(sp); sub t2, t2, a0  /* cross_start */
    lw    t3, DF_DROW(sp)
    lw    a0, DF_DX(sp); mul a0, a0, a4; add t3, t3, a0  /* dot_start */
    add   t6, t6, a4               /* pal_ptr += delta */
    slli  a0, a4, 1; add t1, t1, a0  /* fb_ptr += delta*2 */
    mv    t4, a2                   /* px = px_lo_r */
    mv    t5, a3                   /* t5 = px_hi_r */
    j     .Ldyv_pal_px

.Ldyv_nonarrow:
    lw    t2, DF_CROW(sp); lw t3, DF_DROW(sp)
    mv    t4, s2                   /* px = px_lo_g */
    mv    t5, s3                   /* t5 = px_hi_g */

.Ldyv_pal_px:
    bgt   t4, t5, .Ldyv_pal_px_done   /* use t5 (narrowed hi) */

    lbu   a0, 0(t6)                 /* pidx */
    addi  t6, t6, 1

    andi  a1, a0, 0x80
    beqz  a1, .Ldyv_pal_skip       /* transparent */

    andi  t0, a0, 0x7f              /* palette index in t0 */

    /* ── Distance → d2_q16 in a0 ── */
    beqz  a6, .Ldyv_pal_point
    blez  t3, .Ldyv_pal_scap
    bge   t3, a6, .Ldyv_pal_ecap

    mul   a0, t2, t2
    mulh  a1, t2, t2               /* upper 32 bits of cross^2 */
    bnez  a1, .Ldyv_pal_skip       /* overflow → way outside glow, skip */
    lw    a1, DF_BR2L(sp)
    bgeu  a1, a0, .Ldyv_pal_in_beam /* csq <= beam_r2_len2 → in beam */
    lw    a1, DF_INV(sp)
    mulhu a2, a0, a1; mul a0, a0, a1
    slli  a2, a2, 16; srli a0, a0, 16; or a0, a2, a0
    j     .Ldyv_pal_beam

.Ldyv_pal_scap:
    lw    a0, DF_X0(sp); lw a1, DF_Y0(sp)
    sub   a2, t4, a0; sub a3, s11, a1
    slli  a0, a2, 8; mul a0, a0, a0
    slli  a1, a3, 8; mul a1, a1, a1; add a0, a0, a1
    j     .Ldyv_pal_beam

.Ldyv_pal_ecap:
    lw    a0, DF_X1(sp); lw a1, DF_Y1(sp)
    sub   a2, t4, a0; sub a3, s11, a1
    slli  a0, a2, 8; mul a0, a0, a0
    slli  a1, a3, 8; mul a1, a1, a1; add a0, a0, a1
    j     .Ldyv_pal_beam

.Ldyv_pal_point:
    lw    a0, DF_X0(sp); lw a1, DF_Y0(sp)
    sub   a2, t4, a0; sub a3, s11, a1
    slli  a0, a2, 8; mul a0, a0, a0
    slli  a1, a3, 8; mul a1, a1, a1; add a0, a0, a1

.Ldyv_pal_beam:
    bgt   a0, a7, .Ldyv_pal_glow
.Ldyv_pal_in_beam:
    mv    a0, s6                    /* contrib = brightness */
    j     .Ldyv_pal_write

.Ldyv_pal_glow:
    lw    a1, DF_GS(sp); beqz a1, .Ldyv_pal_skip
    sub   a2, a0, a7                /* glow_d2 */
    mul   a3, a2, a1; mulh a4, a2, a1
    slli  a4, a4, 2; srli a3, a3, 30; or a0, a4, a3   /* lut_idx */
    li    a1, 178; bgeu a0, a1, .Ldyv_pal_skip
    la    a1, gauss_lut; add a1, a1, a0; lbu a1, 0(a1)
    mul   a0, s6, a1; srai a0, a0, 8
    beqz  a0, .Ldyv_pal_skip

.Ldyv_pal_write:
    /* a0=contrib, t0=palette_idx, t1=fb_ptr, t4=px */

    /* YUV palette lookup: a1 = &palette_yuv[idx] — {Y, U, V} precomputed */
    slli  a1, t0, 1; add a1, a1, t0; add a1, s8, a1

    lbu   a2, 0(a1)                 /* Y */
    lbu   a3, 1(a1)                 /* U */
    lbu   a4, 2(a1)                 /* V */

    /* y_contrib = Y * contrib >> 8; dst[0] += y_contrib (saturating) */
    mul   a5, a2, a0;  srli a5, a5, 8
    lbu   t0, 0(t1);   add a5, a5, t0
    li    t0, 255;  blt a5, t0, 1f;  mv a5, t0
1:  sb    a5, 0(t1)

    /* u_ptr = t1+1 for even px, t1-1 for odd px */
    andi  t0, t4, 1
    slli  t0, t0, 1
    li    a5, 1;  sub a5, a5, t0;  add a5, a5, t1   /* a5 = u_ptr */

    /* U: (U-128)*contrib >> 8  (signed delta, clamp 0-255) */
    addi  a3, a3, -128
    mul   t0, a3, a0;  srai t0, t0, 8
    lbu   a1, 0(a5);   add a1, a1, t0
    bge   a1, zero, 1f;  li a1, 0
1:  li    t0, 255;  blt a1, t0, 2f;  mv a1, t0
2:  sb    a1, 0(a5)

    /* V: (V-128)*contrib >> 8  (signed delta, clamp 0-255) */
    addi  a4, a4, -128
    mul   t0, a4, a0;  srai t0, t0, 8
    lbu   a1, 2(a5);   add a1, a1, t0
    bge   a1, zero, 1f;  li a1, 0
1:  li    t0, 255;  blt a1, t0, 2f;  mv a1, t0
2:  sb    a1, 2(a5)

.Ldyv_pal_skip:
    addi  t1, t1, 2                 /* fb_ptr += 2 */
    beqz  a6, .Ldyv_pal_px_incr
    lw    a1, DF_DY(sp); sub t2, t2, a1
    lw    a1, DF_DX(sp); add t3, t3, a1
.Ldyv_pal_px_incr:
    addi  t4, t4, 1
    ble   t4, t5, .Ldyv_pal_px     /* t5 = px_hi_r (narrowed) */

.Ldyv_pal_px_done:
    lw    a0, DF_LEN2(sp)
    beqz  a0, .Ldyv_pal_py_next
    lw    a0, DF_CROW(sp); lw a1, DF_DX(sp); add a0, a0, a1; sw a0, DF_CROW(sp)
    lw    a0, DF_DROW(sp); lw a1, DF_DY(sp); add a0, a0, a1; sw a0, DF_DROW(sp)
.Ldyv_pal_py_next:
    addi  s11, s11, 1
    j     .Ldyv_pal_py

.Ldyv_ret:
    lw    ra,  0(sp)
    lw    s0,  4(sp)
    lw    s1,  8(sp)
    lw    s2,  12(sp)
    lw    s3,  16(sp)
    lw    s4,  20(sp)
    lw    s5,  24(sp)
    lw    s6,  28(sp)
    lw    s7,  32(sp)
    lw    s8,  36(sp)
    lw    s9,  40(sp)
    lw    s10, 44(sp)
    lw    s11, 48(sp)
    addi  sp, sp, DF
    ret
    .size draw_line_yuv422_overlay, .-draw_line_yuv422_overlay
