/* draw_line_yuv422.S
 *
 * RV32IMAC hand-optimised assembly for:
 *   draw_line_yuv422    — palette-colour distance-field draw, additive write
 *   undraw_line_yuv422  — distance-field undraw, writes Y=0 U=128 V=128 (black)
 *
 * Copied from draw_line_dist_yuv422_overlay.S — overlay pixel gate replaced
 * with a fixed colour from s_overlay_palette[colorPaletteEntry].
 * Undraw restore replaced with plain black write.
 * half_w uses Rb*(|dx|+|dy|)/|dy|+1 instead of isqrt32.
 *
 * YUV422 (YUYV) pixel layout: [Y0][U][Y1][V] per 4-byte pair, 2 bytes/pixel.
 *   Y for pixel px : row_fb[px*2 + 0]
 *   U (pair)  : even→dst[+1], odd→dst[-1]
 *   V (pair)  : even→dst[+3], odd→dst[+1]
 *
 * BT.601 full-range (palette is BGR):
 *   Y = (77*R + 150*G + 29*B) >> 8
 *   U = 128 + ((128*B - 43*R - 85*G) >> 8)  clamped [0,255]
 *   V = 128 + ((128*R - 107*G - 21*B) >> 8) clamped [0,255]
 */ 

    .extern g_line_glow
    .extern g_line_width
    .extern g_line_Rb

    .section .iram0.text,"ax",@progbits
    .align 2

/* ══════════════════════════════════════════════════════════════════════════
 *  undraw_line_yuv422
 *
 *  void undraw_line_yuv422(
 *    a0=fb, a1=fb_w, a2=fb_h,
 *    a3=x0, a4=y0, a5=x1, a6=y1)
 *
 *  Register map:
 *    s0  fb          s6  (free)
 *    s1  fb_w        s7  (free)
 *    s2  px_lo       s8  (free)
 *    s3  px_hi       s9  (free)
 *    s4  by0         s10 (free)
 *    s5  by1         s11 py
 *
 *  a5 = beam_r2_q16 cached throughout distance-field loop.
 *
 *  Stack frame 112 bytes:
 *    sp+  0  ra      sp+ 52  x0      sp+ 76  len2
 *    sp+  4  s0      sp+ 56  y0      sp+ 80  inv_len2
 *    sp+  8  s1      sp+ 60  x1      sp+ 84  gs
 *    sp+ 12  s2      sp+ 64  y1      sp+ 88  cross_row
 *    sp+ 16  s3      sp+ 68  dx      sp+ 92  dot_row
 *    sp+ 20  s4      sp+ 72  dy      sp+ 96  halfw
 *    sp+ 24  s5                      sp+100  br2l (beam_r^2*len2)
 *    sp+ 28  s6                      sp+104..111  pad
 *    sp+ 32  s7
 *    sp+ 36  s8
 *    sp+ 40  s9
 *    sp+ 44  s10
 *    sp+ 48  s11
 * ══════════════════════════════════════════════════════════════════════════ */

#define UL         112
#define UL_RA        0
#define UL_X0       52
#define UL_Y0       56
#define UL_X1       60
#define UL_Y1       64
#define UL_DX       68
#define UL_DY       72
#define UL_LEN2     76
#define UL_INV      80
#define UL_GS       84
#define UL_CROW     88
#define UL_DROW     92
#define UL_HALFW    96
#define UL_BR2L    100

    .globl undraw_line_yuv422
    .type  undraw_line_yuv422, @function

undraw_line_yuv422:
    addi  sp, sp, -UL
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

    sw    a3, UL_X0(sp)
    sw    a4, UL_Y0(sp)
    sw    a5, UL_X1(sp)
    sw    a6, UL_Y1(sp)

    mv    s0, a0           /* fb  */
    mv    s1, a1           /* fb_w */

    /* Rb = g_line_Rb */
    la    t2, g_line_Rb
    lw    t2, 0(t2)

    /* bx0 = iclamp(min(x0,x1)-Rb, 0, fw-1) */
    lw    a3, UL_X0(sp); lw a5, UL_X1(sp)
    blt   a3, a5, 1f; mv a3, a5
1:  sub   a3, a3, t2
    bgez  a3, 1f; li a3, 0
1:  addi  t3, s1, -1
    blt   a3, t3, 1f; mv a3, t3
1:  mv    s2, a3

    /* bx1 = iclamp(max(x0,x1)+Rb, 0, fw-1) */
    lw    a3, UL_X0(sp); lw a5, UL_X1(sp)
    bgt   a3, a5, 1f; mv a3, a5
1:  add   a3, a3, t2
    bgez  a3, 1f; li a3, 0
1:  blt   a3, t3, 1f; mv a3, t3
1:  mv    s3, a3

    /* by0 = iclamp(min(y0,y1)-Rb, 0, fh-1) */
    lw    a4, UL_Y0(sp); lw a6, UL_Y1(sp)
    blt   a4, a6, 1f; mv a4, a6
1:  sub   a4, a4, t2
    bgez  a4, 1f; li a4, 0
1:  addi  t3, a2, -1
    blt   a4, t3, 1f; mv a4, t3
1:  mv    s4, a4

    /* by1 = iclamp(max(y0,y1)+Rb, 0, fh-1) */
    lw    a4, UL_Y0(sp); lw a6, UL_Y1(sp)
    bgt   a4, a6, 1f; mv a4, a6
1:  add   a4, a4, t2
    bgez  a4, 1f; li a4, 0
1:  blt   a4, t3, 1f; mv a4, t3
1:  mv    s5, a4

    bgt   s2, s3, .Luyl_ret
    bgt   s4, s5, .Luyl_ret

    /* dx = x1 - x0;  dy = y1 - y0 */
    lw    t0, UL_X0(sp); lw t1, UL_X1(sp); sub t0, t1, t0; sw t0, UL_DX(sp)
    lw    t0, UL_Y0(sp); lw t1, UL_Y1(sp); sub t0, t1, t0; sw t0, UL_DY(sp)

    /* half_w = Rb*(|dx|+|dy|)/|dy|+1  (0 sentinel if dy=0 → full range) */
    lw    t0, UL_DX(sp); bgez t0, 1f; neg t0, t0
1:  lw    t1, UL_DY(sp); bgez t1, 2f; neg t1, t1
2:  beqz  t1, .Luyl_hw0
    add   t0, t0, t1          /* |dx|+|dy| */
    mul   t0, t0, t2          /* Rb*(|dx|+|dy|) */
    divu  t0, t0, t1
    addi  t0, t0, 1
    sw    t0, UL_HALFW(sp)
    j     .Luyl_hw_done
.Luyl_hw0:
    sw    zero, UL_HALFW(sp)
.Luyl_hw_done:

    /* len2 = dx^2 + dy^2 */
    lw    a0, UL_DX(sp); lw a1, UL_DY(sp)
    mul   a2, a0, a0; mul a3, a1, a1; add a2, a2, a3
    sw    a2, UL_LEN2(sp)

    /* beam_r2_q16 = (beam_r<<8)^2;  beam_r = g_line_width >> 1 */
    la    t0, g_line_width; lw t0, 0(t0); srai t0, t0, 1
    slli  t0, t0, 8; mul t0, t0, t0
    mv    a5, t0                   /* a5 = beam_r2_q16 (cached) */

    /* beam_r2_len2 = beam_r^2 * len2 */
    la    t0, g_line_width; lw t0, 0(t0); srai t0, t0, 1
    mul   t0, t0, t0
    lw    t1, UL_LEN2(sp); mul t0, t0, t1
    sw    t0, UL_BR2L(sp)

    /* gs = (glow==0) ? 0 : (177<<14)/glow^2 */
    la    t0, g_line_glow; lw t0, 0(t0)
    bgez  t0, 1f; li t0, 0
1:  beqz  t0, .Luyl_gs0
    mul   t1, t0, t0
    li    t0, 2899968; div t0, t0, t1
    sw    t0, UL_GS(sp)
    j     .Luyl_gs_done
.Luyl_gs0:
    sw    zero, UL_GS(sp)
.Luyl_gs_done:

    /* inv_len2; cross_row/dot_row from (s2=bx0, s4=by0) */
    lw    a2, UL_LEN2(sp)
    beqz  a2, .Luyl_setup_done
    li    t0, -1; divu t0, t0, a2; sw t0, UL_INV(sp)
    lw    t0, UL_X0(sp); sub a0, s2, t0    /* apx = bx0 - x0 */
    lw    t0, UL_Y0(sp); sub a1, s4, t0    /* apy = by0 - y0 */
    lw    t0, UL_DX(sp); lw t1, UL_DY(sp)
    mul   a2, t0, a1; mul a3, t1, a0; sub a2, a2, a3; sw a2, UL_CROW(sp)
    mul   a2, a0, t0; mul a3, a1, t1; add a2, a2, a3; sw a2, UL_DROW(sp)
    j     .Luyl_outer
.Luyl_setup_done:
    sw    zero, UL_INV(sp)
    sw    zero, UL_CROW(sp)
    sw    zero, UL_DROW(sp)

.Luyl_outer:
    mv    s11, s4                  /* py = by0 */

.Luyl_row:
    bgt   s11, s5, .Luyl_ret

    /* row_fb = fb + py*fw*2 */
    mul   t5, s11, s1; slli t5, t5, 1; add t5, s0, t5

    /* fb_ptr = row_fb + bx0*2 */
    slli  a0, s2, 1; add t1, t5, a0

    lw    a6, UL_LEN2(sp)

    /* Per-row x narrowing */
    lw    a1, UL_DY(sp)
    beqz  a1, .Luyl_nonarrow
    lw    a0, UL_HALFW(sp)
    beqz  a0, .Luyl_nonarrow      /* dy=0 sentinel → full range */
    lw    t2, UL_CROW(sp)
    div   a3, t2, a1               /* cross_row / dy */
    add   a3, a3, s2               /* px_center */
    sub   a2, a3, a0               /* px_lo_r */
    bge   a2, s2, 1f; mv a2, s2
1:  add   a3, a3, a0               /* px_hi_r */
    ble   a3, s3, 2f; mv a3, s3
2:  bgt   a2, a3, .Luyl_px_done
    sub   a4, a2, s2               /* delta = px_lo_r - bx0 */
    mul   a0, a1, a4
    lw    t2, UL_CROW(sp); sub t2, t2, a0
    lw    t3, UL_DROW(sp)
    lw    a0, UL_DX(sp); mul a0, a0, a4; add t3, t3, a0
    slli  a0, a4, 1; add t1, t1, a0
    mv    t4, a2
    mv    t5, a3
    j     .Luyl_px

.Luyl_nonarrow:
    lw    t2, UL_CROW(sp); lw t3, UL_DROW(sp)
    mv    t4, s2
    mv    t5, s3

.Luyl_px:
    bgt   t4, t5, .Luyl_px_done

    /* ── Distance → d2_q16 in a0 ── */
    beqz  a6, .Luyl_point
    blez  t3, .Luyl_scap
    bge   t3, a6, .Luyl_ecap

    mul   a0, t2, t2
    mulh  a1, t2, t2               /* upper 32 bits of cross^2 */
    bnez  a1, .Luyl_pxskip         /* overflow → far outside, skip */
    lw    a1, UL_BR2L(sp)
    bgeu  a1, a0, .Luyl_restore    /* csq <= beam_r2_len2 → in beam */
    lw    a1, UL_INV(sp)
    mulhu a2, a0, a1; mul a0, a0, a1
    slli  a2, a2, 16; srli a0, a0, 16; or a0, a2, a0
    j     .Luyl_beam

.Luyl_scap:
    lw    a0, UL_X0(sp); lw a1, UL_Y0(sp)
    sub   a2, t4, a0; sub a3, s11, a1
    slli  a0, a2, 8; mul a0, a0, a0
    slli  a1, a3, 8; mul a1, a1, a1; add a0, a0, a1
    j     .Luyl_beam

.Luyl_ecap:
    lw    a0, UL_X1(sp); lw a1, UL_Y1(sp)
    sub   a2, t4, a0; sub a3, s11, a1
    slli  a0, a2, 8; mul a0, a0, a0
    slli  a1, a3, 8; mul a1, a1, a1; add a0, a0, a1
    j     .Luyl_beam

.Luyl_point:
    lw    a0, UL_X0(sp); lw a1, UL_Y0(sp)
    sub   a2, t4, a0; sub a3, s11, a1
    slli  a0, a2, 8; mul a0, a0, a0
    slli  a1, a3, 8; mul a1, a1, a1; add a0, a0, a1

.Luyl_beam:                        /* a0=d2_q16, a5=beam_r2_q16 */
    bgt   a0, a5, .Luyl_glow
.Luyl_restore:
    /* write black: Y=0, U=128, V=128 */
    sb    zero, 0(t1)
    andi  t0, t4, 1; slli t0, t0, 1
    li    a1, 1; sub a1, a1, t0; add a1, a1, t1   /* u_ptr */
    li    t0, 128; sb t0, 0(a1); sb t0, 2(a1)
    j     .Luyl_pxskip

.Luyl_glow:
    lw    a1, UL_GS(sp); beqz a1, .Luyl_pxskip
    sub   a2, a0, a5               /* glow_d2 */
    mul   a3, a2, a1; mulh a4, a2, a1
    slli  a4, a4, 2; srli a3, a3, 30; or a0, a4, a3   /* lut_idx */
    li    a1, 178; bgeu a0, a1, .Luyl_pxskip
    la    a1, gauss_lut; add a1, a1, a0; lbu a1, 0(a1)
    beqz  a1, .Luyl_pxskip        /* gauss==0: pixel not drawn, skip */
    /* pixel was in glow region → write black */
    sb    zero, 0(t1)
    andi  t0, t4, 1; slli t0, t0, 1
    li    a1, 1; sub a1, a1, t0; add a1, a1, t1
    li    t0, 128; sb t0, 0(a1); sb t0, 2(a1)

.Luyl_pxskip:
    addi  t1, t1, 2
    beqz  a6, .Luyl_px_incr
    lw    a1, UL_DY(sp); sub t2, t2, a1
    lw    a1, UL_DX(sp); add t3, t3, a1
.Luyl_px_incr:
    addi  t4, t4, 1
    ble   t4, t5, .Luyl_px

.Luyl_px_done:
    beqz  a6, .Luyl_row_next
    lw    a0, UL_CROW(sp); lw a1, UL_DX(sp); add a0, a0, a1; sw a0, UL_CROW(sp)
    lw    a0, UL_DROW(sp); lw a1, UL_DY(sp); add a0, a0, a1; sw a0, UL_DROW(sp)
.Luyl_row_next:
    addi  s11, s11, 1
    j     .Luyl_row

.Luyl_ret:
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
    addi  sp, sp, UL
    ret
    .size undraw_line_yuv422, .-undraw_line_yuv422

/* ══════════════════════════════════════════════════════════════════════════
 *  draw_line_yuv422
 *
 *  void draw_line_yuv422(
 *    a0=fb, a1=fb_w, a2=fb_h,
 *    a3=x0, a4=y0, a5=x1, a6=y1, a7=colorPaletteEntry)
 *
 *  Register map:
 *    s0  fb          s6  brightness (255)
 *    s1  fb_w        s7  r_col
 *    s2  px_lo       s8  g_col
 *    s3  px_hi       s9  b_col
 *    s4  by0         s10 (free)
 *    s5  by1         s11 py
 *
 *  Stack frame 128 bytes:
 *    sp+  0  ra      sp+ 52  x0      sp+ 80  inv_len2
 *    sp+  4  s0      sp+ 56  y0      sp+ 84  gs
 *    sp+  8  s1      sp+ 60  x1      sp+ 88  beam_r2_q16
 *    sp+ 12  s2      sp+ 64  y1      sp+ 92  cross_row
 *    sp+ 16  s3      sp+ 68  dx      sp+ 96  dot_row
 *    sp+ 20  s4      sp+ 72  dy      sp+100  halfw
 *    sp+ 24  s5      sp+ 76  len2    sp+104  br2l (beam_r^2*len2)
 *    sp+ 28  s6                      sp+108..127  pad
 *    sp+ 32  s7
 *    sp+ 36  s8
 *    sp+ 40  s9
 *    sp+ 44  s10
 *    sp+ 48  s11
 * ══════════════════════════════════════════════════════════════════════════ */

#define DL         128
#define DL_RA        0
#define DL_X0       52
#define DL_Y0       56
#define DL_X1       60
#define DL_Y1       64
#define DL_DX       68
#define DL_DY       72
#define DL_LEN2     76
#define DL_INV      80
#define DL_GS       84
#define DL_BR2      88
#define DL_CROW     92
#define DL_DROW     96
#define DL_HALFW   100
#define DL_BR2L    104

    .globl draw_line_yuv422
    .type  draw_line_yuv422, @function

draw_line_yuv422:
    addi  sp, sp, -DL
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

    sw    a3, DL_X0(sp)
    sw    a4, DL_Y0(sp)
    sw    a5, DL_X1(sp)
    sw    a6, DL_Y1(sp)

    mv    s0, a0
    mv    s1, a1
    li    s6, 255                  /* brightness */

    /* Fixed colour from s_overlay_palette[colorPaletteEntry & 0x7F] */
    andi  a7, a7, 0x7F
    la    t0, s_overlay_palette
    slli  t1, a7, 1; add t1, t1, a7   /* t1 = idx*3 */
    add   t0, t0, t1
    lbu   s9, 0(t0)                /* b_col */
    lbu   s8, 1(t0)                /* g_col */
    lbu   s7, 2(t0)                /* r_col */

    /* Rb */
    la    t2, g_line_Rb; lw t2, 0(t2)

    /* bx0 = iclamp(min(x0,x1)-Rb, 0, fw-1) */
    lw    a3, DL_X0(sp); lw a5, DL_X1(sp)
    blt   a3, a5, 1f; mv a3, a5
1:  sub   a3, a3, t2
    bgez  a3, 1f; li a3, 0
1:  addi  t3, s1, -1
    blt   a3, t3, 1f; mv a3, t3
1:  mv    s2, a3

    /* bx1 = iclamp(max(x0,x1)+Rb, 0, fw-1) */
    lw    a3, DL_X0(sp); lw a5, DL_X1(sp)
    bgt   a3, a5, 1f; mv a3, a5
1:  add   a3, a3, t2
    bgez  a3, 1f; li a3, 0
1:  blt   a3, t3, 1f; mv a3, t3
1:  mv    s3, a3

    /* by0 = iclamp(min(y0,y1)-Rb, 0, fh-1) */
    lw    a4, DL_Y0(sp); lw a6, DL_Y1(sp)
    blt   a4, a6, 1f; mv a4, a6
1:  sub   a4, a4, t2
    bgez  a4, 1f; li a4, 0
1:  addi  t3, a2, -1
    blt   a4, t3, 1f; mv a4, t3
1:  mv    s4, a4

    /* by1 = iclamp(max(y0,y1)+Rb, 0, fh-1) */
    lw    a4, DL_Y0(sp); lw a6, DL_Y1(sp)
    bgt   a4, a6, 1f; mv a4, a6
1:  add   a4, a4, t2
    bgez  a4, 1f; li a4, 0
1:  blt   a4, t3, 1f; mv a4, t3
1:  mv    s5, a4

    /* dx, dy, len2 */
    lw    a3, DL_X0(sp); lw a5, DL_X1(sp); sub t0, a5, a3; sw t0, DL_DX(sp)
    lw    a4, DL_Y0(sp); lw a6, DL_Y1(sp); sub t1, a6, a4; sw t1, DL_DY(sp)
    mul   a3, t0, t0;  mul a4, t1, t1;  add a3, a3, a4;  sw a3, DL_LEN2(sp)

    /* half_w = Rb*(|dx|+|dy|)/|dy|+1  (0 sentinel if dy=0 → full range) */
    lw    t0, DL_DX(sp); bgez t0, 1f; neg t0, t0
1:  lw    t1, DL_DY(sp); bgez t1, 2f; neg t1, t1
2:  beqz  t1, .Ldyl_hw0
    add   t0, t0, t1
    mul   t0, t0, t2
    divu  t0, t0, t1; addi t0, t0, 1
    sw    t0, DL_HALFW(sp)
    j     .Ldyl_hw_done
.Ldyl_hw0:
    sw    zero, DL_HALFW(sp)
.Ldyl_hw_done:

    /* beam_r2_q16 = (beam_r<<8)^2 */
    la    t1, g_line_width; lw t1, 0(t1); srai t1, t1, 1
    slli  t1, t1, 8; mul t1, t1, t1; sw t1, DL_BR2(sp)

    /* beam_r2_len2 = beam_r^2 * len2 */
    la    t1, g_line_width; lw t1, 0(t1); srai t1, t1, 1
    mul   t1, t1, t1
    lw    t0, DL_LEN2(sp); mul t1, t1, t0; sw t1, DL_BR2L(sp)

    /* gs = (glow==0) ? 0 : (177<<14)/glow^2 */
    la    t0, g_line_glow; lw t0, 0(t0)
    bgez  t0, 1f; li t0, 0
1:  beqz  t0, .Ldyl_gs_zero
    mul   t1, t0, t0
    li    t0, 2899968           /* 177<<14 */
    div   t0, t0, t1;  sw t0, DL_GS(sp)
    j     .Ldyl_gs_done
.Ldyl_gs_zero:
    sw    zero, DL_GS(sp)
.Ldyl_gs_done:

    /* inv_len2, cross_row, dot_row */
    lw    a3, DL_LEN2(sp)
    beqz  a3, .Ldyl_setup_done
    li    t0, -1; divu t0, t0, a3; sw t0, DL_INV(sp)
    lw    t0, DL_X0(sp); sub a0, s2, t0    /* apx0 = bx0 - x0 */
    lw    t0, DL_Y0(sp); sub a1, s4, t0    /* apy0 = by0 - y0 */
    lw    t0, DL_DX(sp); lw  t1, DL_DY(sp)
    mul   a2, t0, a1; mul a3, t1, a0; sub a2, a2, a3; sw a2, DL_CROW(sp)
    mul   a2, a0, t0; mul a3, a1, t1; add a2, a2, a3; sw a2, DL_DROW(sp)

.Ldyl_setup_done:
    blt   s3, s2, .Ldyl_ret
    mv    s11, s4

.Ldyl_py:
    bgt   s11, s5, .Ldyl_ret

    /* row_fb = fb + py*fw*2 */
    mul   t5, s11, s1; slli t5, t5, 1; add t5, s0, t5

    /* fb_ptr = row_fb + bx0*2 */
    slli  a0, s2, 1; add t1, t5, a0
    /* t5 (row_fb) no longer needed; repurposed as px_hi_r */

    lw    a6, DL_LEN2(sp); lw a7, DL_BR2(sp)

    /* Per-row x narrowing */
    lw    a1, DL_DY(sp)
    beqz  a1, .Ldyl_nonarrow
    lw    a0, DL_HALFW(sp)
    beqz  a0, .Ldyl_nonarrow       /* dy=0 sentinel → full range */
    lw    t2, DL_CROW(sp)
    div   a3, t2, a1
    add   a3, a3, s2               /* px_center */
    sub   a2, a3, a0               /* px_lo_r */
    bge   a2, s2, 1f; mv a2, s2
1:  add   a3, a3, a0               /* px_hi_r */
    ble   a3, s3, 2f; mv a3, s3
2:  bgt   a2, a3, .Ldyl_px_done
    sub   a4, a2, s2               /* delta = px_lo_r - bx0 */
    mul   a0, a1, a4
    lw    t2, DL_CROW(sp); sub t2, t2, a0
    lw    t3, DL_DROW(sp)
    lw    a0, DL_DX(sp); mul a0, a0, a4; add t3, t3, a0
    slli  a0, a4, 1; add t1, t1, a0
    mv    t4, a2
    mv    t5, a3
    j     .Ldyl_px

.Ldyl_nonarrow:
    lw    t2, DL_CROW(sp); lw t3, DL_DROW(sp)
    mv    t4, s2
    mv    t5, s3

.Ldyl_px:
    bgt   t4, t5, .Ldyl_px_done

    /* ── Distance → d2_q16 in a0 ── */
    beqz  a6, .Ldyl_point
    blez  t3, .Ldyl_scap
    bge   t3, a6, .Ldyl_ecap

    mul   a0, t2, t2
    mulh  a1, t2, t2               /* upper 32 bits of cross^2 */
    bnez  a1, .Ldyl_skip           /* overflow → far outside, skip */
    lw    a1, DL_BR2L(sp)
    bgeu  a1, a0, .Ldyl_in_beam   /* csq <= beam_r2_len2 → in beam */
    lw    a1, DL_INV(sp)
    mulhu a2, a0, a1; mul a0, a0, a1
    slli  a2, a2, 16; srli a0, a0, 16; or a0, a2, a0
    j     .Ldyl_beam

.Ldyl_scap:
    lw    a0, DL_X0(sp); lw a1, DL_Y0(sp)
    sub   a2, t4, a0; sub a3, s11, a1
    slli  a0, a2, 8; mul a0, a0, a0
    slli  a1, a3, 8; mul a1, a1, a1; add a0, a0, a1
    j     .Ldyl_beam

.Ldyl_ecap:
    lw    a0, DL_X1(sp); lw a1, DL_Y1(sp)
    sub   a2, t4, a0; sub a3, s11, a1
    slli  a0, a2, 8; mul a0, a0, a0
    slli  a1, a3, 8; mul a1, a1, a1; add a0, a0, a1
    j     .Ldyl_beam

.Ldyl_point:
    lw    a0, DL_X0(sp); lw a1, DL_Y0(sp)
    sub   a2, t4, a0; sub a3, s11, a1
    slli  a0, a2, 8; mul a0, a0, a0
    slli  a1, a3, 8; mul a1, a1, a1; add a0, a0, a1

.Ldyl_beam:
    bgt   a0, a7, .Ldyl_glow
.Ldyl_in_beam:
    mv    a0, s6                   /* contrib = brightness (255) */
    j     .Ldyl_write

.Ldyl_glow:
    lw    a1, DL_GS(sp); beqz a1, .Ldyl_skip
    sub   a2, a0, a7               /* glow_d2 = d2_q16 - beam_r2_q16 */
    mul   a3, a2, a1; mulh a4, a2, a1
    slli  a4, a4, 2; srli a3, a3, 30; or a0, a4, a3   /* lut_idx */
    li    a1, 178; bgeu a0, a1, .Ldyl_skip
    la    a1, gauss_lut; add a1, a1, a0; lbu a1, 0(a1)
    mul   a0, s6, a1; srai a0, a0, 8
    beqz  a0, .Ldyl_skip

.Ldyl_write:
    /* a0=contrib, t1=fb_ptr, t4=px, s7=r_col, s8=g_col, s9=b_col */

    /* Y_full = (77*r + 150*g + 29*b) >> 8 */
    li    a5, 77;   mul a5, a5, s7
    li    t0, 150;  mul t0, t0, s8;  add a5, a5, t0
    li    t0, 29;   mul t0, t0, s9;  add a5, a5, t0
    srli  a5, a5, 8                 /* a5 = Y_full */

    /* y_contrib = Y_full * contrib >> 8; dst[0] += y_contrib (additive) */
    mul   a5, a5, a0; srli a5, a5, 8
    lbu   t0, 0(t1); add a5, a5, t0
    li    t0, 255;  blt a5, t0, 1f;  mv a5, t0
1:  sb    a5, 0(t1)

    /* u_ptr: t1-1 for odd px, t1+1 for even px */
    andi  t0, t4, 1; slli t0, t0, 1
    li    a5, 1;  sub a5, a5, t0;  add a5, a5, t1   /* a5 = u_ptr */

    /* U additive delta = (128*b - 43*r - 85*g) * contrib >> 16 */
    slli  t0, s9, 7
    li    a1, 43;   mul a1, a1, s7;  sub t0, t0, a1
    li    a1, 85;   mul a1, a1, s8;  sub t0, t0, a1
    mul   t0, t0, a0;  srai t0, t0, 16
    lbu   a1, 0(a5);   add a1, a1, t0
    bge   a1, zero, 1f;  li a1, 0
1:  li    t0, 255;  blt a1, t0, 2f;  mv a1, t0
2:  sb    a1, 0(a5)

    /* V additive delta = (128*r - 107*g - 21*b) * contrib >> 16 */
    slli  t0, s7, 7
    li    a1, 107;  mul a1, a1, s8;  sub t0, t0, a1
    li    a1, 21;   mul a1, a1, s9;  sub t0, t0, a1
    mul   t0, t0, a0;  srai t0, t0, 16
    lbu   a1, 2(a5);   add a1, a1, t0
    bge   a1, zero, 1f;  li a1, 0
1:  li    t0, 255;  blt a1, t0, 2f;  mv a1, t0
2:  sb    a1, 2(a5)

.Ldyl_skip:
    addi  t1, t1, 2                /* fb_ptr += 2 */
    beqz  a6, .Ldyl_px_incr
    lw    a1, DL_DY(sp); sub t2, t2, a1
    lw    a1, DL_DX(sp); add t3, t3, a1
.Ldyl_px_incr:
    addi  t4, t4, 1
    ble   t4, t5, .Ldyl_px

.Ldyl_px_done:
    lw    a0, DL_LEN2(sp)
    beqz  a0, .Ldyl_py_next
    lw    a0, DL_CROW(sp); lw a1, DL_DX(sp); add a0, a0, a1; sw a0, DL_CROW(sp)
    lw    a0, DL_DROW(sp); lw a1, DL_DY(sp); add a0, a0, a1; sw a0, DL_DROW(sp)
.Ldyl_py_next:
    addi  s11, s11, 1
    j     .Ldyl_py

.Ldyl_ret:
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
    addi  sp, sp, DL
    ret
    .size draw_line_yuv422, .-draw_line_yuv422
