/* draw_line_yuv422_brightness.S
 *
 * RV32IMAFC IRAM-resident assembly for white (brightness-only) YUV422 line
 * renderers.  Algorithm matches draw_line_yuv422_brightness.c /
 * undraw_line_yuv422_brightness.c.
 *
 *   draw_line_yuv422_brightness   — white additive Y-only draw
 *   undraw_line_yuv422_brightness — white Y-erase (no early-return on brightness=0)
 *
 * White (R=G=B=255) → bgr_to_y=255, bgr_to_u=bgr_to_v=128.
 * U/V deltas are always zero for white — UV is never written.
 *
 * YUV422 (YUYV) pixel layout: [Y0][U][Y1][V] per 4-byte pair, 2 bytes/pixel.
 *   Y for pixel px : row_fb[px*2 + 0]
 *
 * undraw_line_yuv422_brightness specific behaviour (diverges from C ref):
 *   - Does NOT early-return when brightness <= 0.  Caller passes brightness=0
 *     as the erase signal.
 *   - In beam  : always sb zero to Y (no y_contrib gate).
 *   - In glow  : erase Y iff gauss_lut[lut_idx] != 0 (no contrib multiply).
 *   - UV       : never written (white has no UV delta).
 */

    .extern g_line_Rb
    .extern g_beam_r
    .extern g_beam_r2_q16
    .extern g_gs
    .extern gauss_lut

    .section .iram.text,"ax",@progbits
    .align 2

/* ════════════════════════════════════════════════════════════════════════════
 * Stack frame layout (shared by both functions, DL=128 / UL=112)
 *
 *  sp+  0  ra          sp+ 52  x0        sp+ 80  inv_len2
 *  sp+  4  s0          sp+ 56  y0        sp+ 84  gs
 *  sp+  8  s1          sp+ 60  x1        sp+ 88  beam_r2_q16 (UL: pad)
 *  sp+ 12  s2          sp+ 64  y1        sp+ 92  cross_row
 *  sp+ 16  s3          sp+ 68  dx        sp+ 96  dot_row
 *  sp+ 20  s4          sp+ 72  dy        sp+100  halfw
 *  sp+ 24  s5          sp+ 76  len2      sp+104  br2l (beam_r^2 * len2)
 *  sp+ 28  s6
 *  sp+ 32  s7
 *  sp+ 36  s8
 *  sp+ 40  s9
 *  sp+ 44  s10
 *  sp+ 48  s11
 * ════════════════════════════════════════════════════════════════════════════ */

#define DL          128
#define DL_RA         0
#define DL_X0        52
#define DL_Y0        56
#define DL_X1        60
#define DL_Y1        64
#define DL_DX        68
#define DL_DY        72
#define DL_LEN2      76
#define DL_INV       80
#define DL_GS        84
#define DL_BR2       88
#define DL_CROW      92
#define DL_DROW      96
#define DL_HALFW    100
#define DL_BR2L     104

#define UL          112
#define UL_RA         0
#define UL_X0        52
#define UL_Y0        56
#define UL_X1        60
#define UL_Y1        64
#define UL_DX        68
#define UL_DY        72
#define UL_LEN2      76
#define UL_INV       80
#define UL_GS        84
#define UL_BR2       88   /* pad slot reused for beam_r2_q16 store */
#define UL_CROW      92
#define UL_DROW      96
#define UL_HALFW    100
#define UL_BR2L     104

/* ══════════════════════════════════════════════════════════════════════════
 * draw_line_yuv422_brightness
 *
 *  void draw_line_yuv422_brightness(
 *    a0=fb, a1=fb_w, a2=fb_h,
 *    a3=x0, a4=y0, a5=x1, a6=y1, a7=brightness)
 *
 *  Register map (saved/callee):
 *    s0  fb          s6  brightness
 *    s1  fb_w        s7-s9 unused (white: no palette)
 *    s2  bx0 (px_lo) s10 free
 *    s3  bx1 (px_hi) s11 py
 *    s4  by0
 *    s5  by1
 *
 *  Temporaries in inner loop (not saved):
 *    a5  beam_r2_q16 (cached throughout)
 *    a6  len2 (reloaded per row)
 *    a7  beam_r2_q16 per row (DL variant reloads into a7 before px loop)
 *    t2  cross,  t3  dot,  t4  px,  t5  px_hi_r,  t1  fb_ptr
 * ══════════════════════════════════════════════════════════════════════════ */

    .globl draw_line_yuv422_brightness
    .type  draw_line_yuv422_brightness, @function

draw_line_yuv422_brightness:
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
    mv    s6, a7                  /* brightness */
    blez  s6, .Lbyl_ret           /* draw: brightness<=0 → nothing to draw */

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
    mul   a3, t0, t0; mul a4, t1, t1; add a3, a3, a4; sw a3, DL_LEN2(sp)

    /* half_w = Rb*(|dx|+|dy|)/|dy|+1  (0 sentinel when dy=0 → full x-range) */
    lw    t0, DL_DX(sp); bgez t0, 1f; neg t0, t0
1:  lw    t1, DL_DY(sp); bgez t1, 2f; neg t1, t1
2:  beqz  t1, .Lbyl_hw0
    add   t0, t0, t1
    mul   t0, t0, t2
    divu  t0, t0, t1; addi t0, t0, 1
    sw    t0, DL_HALFW(sp)
    j     .Lbyl_hw_done
.Lbyl_hw0:
    sw    zero, DL_HALFW(sp)
.Lbyl_hw_done:

    /* precomputed beam/glow globals */
    la    t1, g_beam_r2_q16; lw t1, 0(t1); sw t1, DL_BR2(sp)
    la    t1, g_beam_r; lw t1, 0(t1)
    mul   t1, t1, t1
    lw    t0, DL_LEN2(sp); mul t1, t1, t0; sw t1, DL_BR2L(sp)
    la    t0, g_gs; lw t0, 0(t0); sw t0, DL_GS(sp)

    lw    a3, DL_LEN2(sp)
    beqz  a3, .Lbyl_setup_done
    li    t0, -1; divu t0, t0, a3; sw t0, DL_INV(sp)
    lw    t0, DL_X0(sp); sub a0, s2, t0    /* bx0 - x0 */
    lw    t0, DL_Y0(sp); sub a1, s4, t0    /* by0 - y0 */
    lw    t0, DL_DX(sp); lw  t1, DL_DY(sp)
    mul   a2, t0, a1; mul a3, t1, a0; sub a2, a2, a3; sw a2, DL_CROW(sp)
    mul   a2, a0, t0; mul a3, a1, t1; add a2, a2, a3; sw a2, DL_DROW(sp)

.Lbyl_setup_done:
    blt   s3, s2, .Lbyl_ret

    lw    t0, DL_LEN2(sp)
    beqz  t0, .Lbyl_dot_loop

    mv    s11, s4

/* ── Main row loop ── */
.Lbyl_py:
    bgt   s11, s5, .Lbyl_ret

    mul   t5, s11, s1; slli t5, t5, 1; add t5, s0, t5
    slli  a0, s2, 1; add t1, t5, a0

    lw    a6, DL_LEN2(sp); lw a7, DL_BR2(sp)

    /* narrow x range when dy != 0 */
    lw    a1, DL_DY(sp)
    beqz  a1, .Lbyl_nonarrow
    lw    a0, DL_HALFW(sp)
    beqz  a0, .Lbyl_nonarrow
    lw    t2, DL_CROW(sp)
    div   a3, t2, a1
    add   a3, a3, s2               /* px_center */
    sub   a2, a3, a0               /* px_lo_r */
    bge   a2, s2, 1f; mv a2, s2
1:  add   a3, a3, a0               /* px_hi_r */
    ble   a3, s3, 2f; mv a3, s3
2:  bgt   a2, a3, .Lbyl_px_done
    sub   a4, a2, s2               /* delta = px_lo_r - bx0 */
    mul   a0, a1, a4
    lw    t2, DL_CROW(sp); sub t2, t2, a0
    lw    t3, DL_DROW(sp)
    lw    a0, DL_DX(sp); mul a0, a0, a4; add t3, t3, a0
    slli  a0, a4, 1; add t1, t1, a0
    mv    t4, a2
    mv    t5, a3
    j     .Lbyl_px

.Lbyl_nonarrow:
    lw    t2, DL_CROW(sp); lw t3, DL_DROW(sp)
    mv    t4, s2
    mv    t5, s3

/* ── Per-pixel classification ── */
.Lbyl_px:
    bgt   t4, t5, .Lbyl_px_done

    blez  t3, .Lbyl_scap
    bge   t3, a6, .Lbyl_ecap

    mul   a0, t2, t2
    mulh  a1, t2, t2               /* upper 32b of cross^2 */
    bnez  a1, .Lbyl_skip           /* overflow → far outside */
    lw    a1, DL_BR2L(sp)
    bgeu  a1, a0, .Lbyl_in_beam   /* csq <= beam_r2_len2 → in beam */
    lw    a1, DL_INV(sp)
    mulhu a2, a0, a1; mul a0, a0, a1
    slli  a2, a2, 16; srli a0, a0, 16; or a0, a2, a0   /* d2_q16 */
    j     .Lbyl_beam

.Lbyl_scap:
    lw    a0, DL_X0(sp); lw a1, DL_Y0(sp)
    sub   a2, t4, a0; sub a3, s11, a1
    slli  a0, a2, 8; mul a0, a0, a0
    slli  a1, a3, 8; mul a1, a1, a1; add a0, a0, a1
    j     .Lbyl_beam

.Lbyl_ecap:
    lw    a0, DL_X1(sp); lw a1, DL_Y1(sp)
    sub   a2, t4, a0; sub a3, s11, a1
    slli  a0, a2, 8; mul a0, a0, a0
    slli  a1, a3, 8; mul a1, a1, a1; add a0, a0, a1
    j     .Lbyl_beam

.Lbyl_beam:
    bgt   a0, a7, .Lbyl_glow
.Lbyl_in_beam:
    mv    a0, s6
    j     .Lbyl_write

.Lbyl_glow:
    lw    a1, DL_GS(sp); beqz a1, .Lbyl_skip
    sub   a2, a0, a7               /* glow_d2 = d2_q16 - beam_r2_q16 */
    mul   a3, a2, a1; mulh a4, a2, a1
    slli  a4, a4, 2; srli a3, a3, 30; or a0, a4, a3   /* lut_idx */
    li    a1, 178; bgeu a0, a1, .Lbyl_skip
    la    a1, gauss_lut; add a1, a1, a0; lbu a1, 0(a1)
    mul   a0, s6, a1; srai a0, a0, 8   /* contrib = (brightness * lut) >> 8 */
    beqz  a0, .Lbyl_skip

/* ── Pixel write (additive Y; UV omitted for white) ── */
.Lbyl_write:
    li    t0, 255; mul t0, t0, a0; srli t0, t0, 8   /* y_contrib */
    beqz  t0, .Lbyl_y_skip
    lbu   a5, 0(t1); add a5, a5, t0
    li    t0, 255; blt a5, t0, 1f; mv a5, t0
1:  sb    a5, 0(t1)
.Lbyl_y_skip:
    /* UV: delta always zero for white — omitted */

.Lbyl_skip:
    addi  t1, t1, 2
    lw    a1, DL_DY(sp); sub t2, t2, a1
    lw    a1, DL_DX(sp); add t3, t3, a1
    addi  t4, t4, 1
    ble   t4, t5, .Lbyl_px

.Lbyl_px_done:
    lw    a0, DL_CROW(sp); lw a1, DL_DX(sp); add a0, a0, a1; sw a0, DL_CROW(sp)
    lw    a0, DL_DROW(sp); lw a1, DL_DY(sp); add a0, a0, a1; sw a0, DL_DROW(sp)
.Lbyl_py_next:
    addi  s11, s11, 1
    j     .Lbyl_py

/* ── Dot loop (len2==0) ── */
.Lbyl_dot_loop:
    lw    a2, DL_X0(sp)
    lw    a3, DL_Y0(sp)
    lw    a7, DL_BR2(sp)
    mv    s11, s4
.Lbyl_d_row:
    bgt   s11, s5, .Lbyl_ret
    mul   t5, s11, s1; slli t5, t5, 1; add t5, s0, t5
    slli  a0, s2, 1; add t1, t5, a0
    mv    t4, s2
.Lbyl_d_px:
    bgt   t4, s3, .Lbyl_d_row_next
    sub   a4, t4, a2; sub a5, s11, a3
    slli  a4, a4, 8; mul a4, a4, a4
    slli  a5, a5, 8; mul a5, a5, a5; add a0, a4, a5   /* d2_q16 */
    bgt   a0, a7, .Lbyl_d_glow
    mv    a0, s6
    j     .Lbyl_d_write
.Lbyl_d_glow:
    lw    a1, DL_GS(sp); beqz a1, .Lbyl_d_skip
    sub   a4, a0, a7
    mul   a5, a4, a1; mulh t0, a4, a1
    slli  t0, t0, 2; srli a5, a5, 30; or a0, t0, a5
    li    a1, 178; bgeu a0, a1, .Lbyl_d_skip
    la    a1, gauss_lut; add a1, a1, a0; lbu a1, 0(a1)
    mul   a0, s6, a1; srai a0, a0, 8
    beqz  a0, .Lbyl_d_skip
.Lbyl_d_write:
    li    t0, 255; mul t0, t0, a0; srli t0, t0, 8
    beqz  t0, .Lbyl_d_y_skip
    lbu   a5, 0(t1); add a5, a5, t0
    li    t0, 255; blt a5, t0, 1f; mv a5, t0
1:  sb    a5, 0(t1)
.Lbyl_d_y_skip:
    /* UV: always zero for white */
    lw    a3, DL_Y0(sp)            /* restore y0 clobbered by cap path */
.Lbyl_d_skip:
    addi  t1, t1, 2
    addi  t4, t4, 1
    j     .Lbyl_d_px
.Lbyl_d_row_next:
    addi  s11, s11, 1
    j     .Lbyl_d_row

.Lbyl_ret:
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
    .size draw_line_yuv422_brightness, .-draw_line_yuv422_brightness


/* ══════════════════════════════════════════════════════════════════════════
 * undraw_line_yuv422_brightness
 *
 *  void undraw_line_yuv422_brightness(
 *    a0=fb, a1=fb_w, a2=fb_h,
 *    a3=x0, a4=y0, a5=x1, a6=y1, a7=brightness)
 *
 *  DIFFERS from C reference:
 *    - Does NOT early-return when brightness <= 0.
 *    - In beam  : always sb zero to Y (unconditional).
 *    - In glow  : erase Y iff gauss_lut[lut_idx] != 0.
 *    - UV       : never written (white has no UV delta).
 *
 *  Register map (saved/callee):
 *    s0  fb          s6  brightness (kept for API symmetry, unused in erase)
 *    s1  fb_w        s7-s9 unused
 *    s2  bx0         s10 free
 *    s3  bx1         s11 py
 *    s4  by0
 *    s5  by1
 *
 *  a5 = beam_r2_q16 cached throughout inner loop.
 *  Stack frame UL=112 bytes.
 * ══════════════════════════════════════════════════════════════════════════ */

    .globl undraw_line_yuv422_brightness
    .type  undraw_line_yuv422_brightness, @function

undraw_line_yuv422_brightness:
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

    mv    s0, a0
    mv    s1, a1
    mv    s6, a7                  /* brightness (stored for completeness, not used in erase logic) */
    /* NOTE: no early-return on brightness<=0 — caller uses brightness=0 as erase signal */

    la    t2, g_line_Rb; lw t2, 0(t2)

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

    bgt   s2, s3, .Lubyl_ret
    bgt   s4, s5, .Lubyl_ret

    lw    t0, UL_X0(sp); lw t1, UL_X1(sp); sub t0, t1, t0; sw t0, UL_DX(sp)
    lw    t0, UL_Y0(sp); lw t1, UL_Y1(sp); sub t0, t1, t0; sw t0, UL_DY(sp)

    /* half_w = Rb*(|dx|+|dy|)/|dy|+1  (0 when dy=0) */
    lw    t0, UL_DX(sp); bgez t0, 1f; neg t0, t0
1:  lw    t1, UL_DY(sp); bgez t1, 2f; neg t1, t1
2:  beqz  t1, .Lubyl_hw0
    add   t0, t0, t1
    mul   t0, t0, t2
    divu  t0, t0, t1
    addi  t0, t0, 1
    sw    t0, UL_HALFW(sp)
    j     .Lubyl_hw_done
.Lubyl_hw0:
    sw    zero, UL_HALFW(sp)
.Lubyl_hw_done:

    /* len2 */
    lw    a0, UL_DX(sp); lw a1, UL_DY(sp)
    mul   a2, a0, a0; mul a3, a1, a1; add a2, a2, a3
    sw    a2, UL_LEN2(sp)

    la    t0, g_beam_r2_q16; lw t0, 0(t0)
    mv    a5, t0                   /* a5 = beam_r2_q16 (cached) */
    sw    t0, UL_BR2(sp)           /* also stored for per-row reload */

    la    t0, g_beam_r; lw t0, 0(t0)
    mul   t0, t0, t0
    lw    t1, UL_LEN2(sp); mul t0, t0, t1
    sw    t0, UL_BR2L(sp)

    la    t0, g_gs; lw t0, 0(t0); sw t0, UL_GS(sp)

    lw    a2, UL_LEN2(sp)
    beqz  a2, .Lubyl_setup_done
    li    t0, -1; divu t0, t0, a2; sw t0, UL_INV(sp)
    lw    t0, UL_X0(sp); sub a0, s2, t0    /* bx0 - x0 */
    lw    t0, UL_Y0(sp); sub a1, s4, t0    /* by0 - y0 */
    lw    t0, UL_DX(sp); lw t1, UL_DY(sp)
    mul   a2, t0, a1; mul a3, t1, a0; sub a2, a2, a3; sw a2, UL_CROW(sp)
    mul   a2, a0, t0; mul a3, a1, t1; add a2, a2, a3; sw a2, UL_DROW(sp)
    j     .Lubyl_outer

.Lubyl_setup_done:
    sw    zero, UL_INV(sp)
    sw    zero, UL_CROW(sp)
    sw    zero, UL_DROW(sp)

    lw    t0, UL_LEN2(sp)
    beqz  t0, .Lubyl_dot_loop

.Lubyl_outer:
    mv    s11, s4

/* ── Main row loop ── */
.Lubyl_row:
    bgt   s11, s5, .Lubyl_ret

    mul   t5, s11, s1; slli t5, t5, 1; add t5, s0, t5
    slli  a0, s2, 1; add t1, t5, a0

    lw    a6, UL_LEN2(sp)
    lw    a5, UL_BR2(sp)           /* reload beam_r2_q16 */

    /* narrow x range when dy != 0 */
    lw    a1, UL_DY(sp)
    beqz  a1, .Lubyl_nonarrow
    lw    a0, UL_HALFW(sp)
    beqz  a0, .Lubyl_nonarrow
    lw    t2, UL_CROW(sp)
    div   a3, t2, a1
    add   a3, a3, s2               /* px_center */
    sub   a2, a3, a0               /* px_lo_r */
    bge   a2, s2, 1f; mv a2, s2
1:  add   a3, a3, a0               /* px_hi_r */
    ble   a3, s3, 2f; mv a3, s3
2:  bgt   a2, a3, .Lubyl_px_done
    sub   a4, a2, s2               /* delta */
    mul   a0, a1, a4
    lw    t2, UL_CROW(sp); sub t2, t2, a0
    lw    t3, UL_DROW(sp)
    lw    a0, UL_DX(sp); mul a0, a0, a4; add t3, t3, a0
    slli  a0, a4, 1; add t1, t1, a0
    mv    t4, a2
    mv    t5, a3
    j     .Lubyl_px

.Lubyl_nonarrow:
    lw    t2, UL_CROW(sp); lw t3, UL_DROW(sp)
    mv    t4, s2
    mv    t5, s3

/* ── Per-pixel classification ── */
.Lubyl_px:
    bgt   t4, t5, .Lubyl_px_done

    blez  t3, .Lubyl_scap
    bge   t3, a6, .Lubyl_ecap

    mul   a0, t2, t2
    mulh  a1, t2, t2
    bnez  a1, .Lubyl_pxskip        /* cross^2 overflow → skip */
    lw    a1, UL_BR2L(sp)
    bgeu  a1, a0, .Lubyl_in_beam  /* csq <= beam_r2_len2 → beam */
    lw    a1, UL_INV(sp)
    mulhu a2, a0, a1; mul a0, a0, a1
    slli  a2, a2, 16; srli a0, a0, 16; or a0, a2, a0   /* d2_q16 */
    j     .Lubyl_beam

.Lubyl_scap:
    lw    a0, UL_X0(sp); lw a1, UL_Y0(sp)
    sub   a2, t4, a0; sub a3, s11, a1
    slli  a0, a2, 8; mul a0, a0, a0
    slli  a1, a3, 8; mul a1, a1, a1; add a0, a0, a1
    j     .Lubyl_beam

.Lubyl_ecap:
    lw    a0, UL_X1(sp); lw a1, UL_Y1(sp)
    sub   a2, t4, a0; sub a3, s11, a1
    slli  a0, a2, 8; mul a0, a0, a0
    slli  a1, a3, 8; mul a1, a1, a1; add a0, a0, a1
    j     .Lubyl_beam

.Lubyl_beam:                        /* a0=d2_q16, a5=beam_r2_q16 */
    bgt   a0, a5, .Lubyl_glow
.Lubyl_in_beam:
    /* Always erase Y in beam core */
    sb    zero, 0(t1)
    j     .Lubyl_pxskip

.Lubyl_glow:
    lw    a1, UL_GS(sp); beqz a1, .Lubyl_pxskip
    sub   a2, a0, a5               /* glow_d2 */
    mul   a3, a2, a1; mulh a4, a2, a1
    slli  a4, a4, 2; srli a3, a3, 30; or a0, a4, a3   /* lut_idx */
    li    a1, 178; bgeu a0, a1, .Lubyl_pxskip
    la    a1, gauss_lut; add a1, a1, a0; lbu a1, 0(a1)
    beqz  a1, .Lubyl_pxskip        /* gauss_lut[lut_idx]==0 → skip */
    /* gauss_lut[lut_idx] != 0 → erase Y */
    sb    zero, 0(t1)
    /* UV: never written for white */

.Lubyl_pxskip:
    addi  t1, t1, 2
    lw    a1, UL_DY(sp); sub t2, t2, a1
    lw    a1, UL_DX(sp); add t3, t3, a1
    addi  t4, t4, 1
    ble   t4, t5, .Lubyl_px

.Lubyl_px_done:
    lw    a0, UL_CROW(sp); lw a1, UL_DX(sp); add a0, a0, a1; sw a0, UL_CROW(sp)
    lw    a0, UL_DROW(sp); lw a1, UL_DY(sp); add a0, a0, a1; sw a0, UL_DROW(sp)
.Lubyl_row_next:
    addi  s11, s11, 1
    j     .Lubyl_row

/* ── Dot loop (len2==0) ── */
.Lubyl_dot_loop:
    mv    s11, s4
.Lubyl_d_row:
    bgt   s11, s5, .Lubyl_ret
    mul   t5, s11, s1; slli t5, t5, 1; add t5, s0, t5
    slli  a0, s2, 1; add t1, t5, a0
    mv    t4, s2
.Lubyl_d_px:
    bgt   t4, s3, .Lubyl_d_row_next
    lw    a0, UL_X0(sp); lw a1, UL_Y0(sp)
    sub   a2, t4, a0; sub a3, s11, a1
    slli  a0, a2, 8; mul a0, a0, a0
    slli  a1, a3, 8; mul a1, a1, a1; add a0, a0, a1   /* d2_q16 */
    bgt   a0, a5, .Lubyl_d_glow
    /* in beam: always erase Y */
    sb    zero, 0(t1)
    j     .Lubyl_d_px_next
.Lubyl_d_glow:
    lw    a1, UL_GS(sp); beqz a1, .Lubyl_d_px_next
    sub   a2, a0, a5
    mul   a3, a2, a1; mulh a4, a2, a1
    slli  a4, a4, 2; srli a3, a3, 30; or a0, a4, a3   /* lut_idx */
    li    a1, 178; bgeu a0, a1, .Lubyl_d_px_next
    la    a1, gauss_lut; add a1, a1, a0; lbu a1, 0(a1)
    beqz  a1, .Lubyl_d_px_next     /* gauss_lut[idx]==0 → skip */
    sb    zero, 0(t1)               /* gauss_lut[idx]!=0 → erase Y */
    /* UV: never for white */
.Lubyl_d_px_next:
    addi  t1, t1, 2
    addi  t4, t4, 1
    j     .Lubyl_d_px
.Lubyl_d_row_next:
    addi  s11, s11, 1
    j     .Lubyl_d_row

.Lubyl_ret:
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
    .size undraw_line_yuv422_brightness, .-undraw_line_yuv422_brightness
