/* draw_line_yuv422.S
 *
 * RV32IMAC hand-optimised assembly for all YUV422 (YUYV) line renderers:
 *
 *   undraw_line_yuv422_overlay  — overlay-palette undraw (distance-field)
 *   draw_line_yuv422_overlay    — overlay-palette draw   (distance-field)
 *   undraw_line_yuv422_color    — fixed-palette erase to black
 *   draw_line_yuv422_color      — fixed-palette additive draw
 *   draw_line_yuv422_brightness — white (brightness-only) additive draw
 *   undraw_line_yuv422_brightness — white (brightness-only) erase to black
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

    .extern g_line_Rb
    .extern g_beam_r
    .extern g_beam_r2_q16
    .extern g_gs


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

    la    t2, g_line_Rb
    lw    t2, 0(t2)

    lw    a3, UF_X0(sp); lw a5, UF_X1(sp)
    blt   a3, a5, 1f; mv a3, a5
1:  sub   a3, a3, t2
    bgez  a3, 1f; li a3, 0
1:  addi  t3, s1, -1
    blt   a3, t3, 1f; mv a3, t3
1:  sw    a3, UF_BX0(sp)

    lw    a3, UF_X0(sp); lw a5, UF_X1(sp)
    bgt   a3, a5, 1f; mv a3, a5
1:  add   a3, a3, t2
    bgez  a3, 1f; li a3, 0
1:  blt   a3, t3, 1f; mv a3, t3
1:  sw    a3, UF_BX1(sp)

    lw    t4, UF_BX0(sp)
    sub   t5, a3, t4
    addi  t5, t5, 1
    slli  t5, t5, 1
    sw    t5, UF_SPAN2(sp)

    lw    a4, UF_Y0(sp); lw a6, UF_Y1(sp)
    blt   a4, a6, 1f; mv a4, a6
1:  sub   a4, a4, t2
    bgez  a4, 1f; li a4, 0
1:  addi  t3, a2, -1
    blt   a4, t3, 1f; mv a4, t3
1:  mv    s4, a4

    lw    a4, UF_Y0(sp); lw a6, UF_Y1(sp)
    bgt   a4, a6, 1f; mv a4, a6
1:  add   a4, a4, t2
    bgez  a4, 1f; li a4, 0
1:  blt   a4, t3, 1f; mv a4, t3
1:  mv    s5, a4

    la    t0, s_overlay_pal; lw s7, 0(t0)
    la    s8, s_overlay_palette
    la    t0, s_overlay_alpha_val; lbu s6, 0(t0)
    la    t0, s_ov_off_x; lw s9, 0(t0)
    la    t0, s_ov_w;     lw s10, 0(t0)
    la    t0, s_ov_off_y; lw t1, 0(t0); sw t1, UF_OFFSY(sp)
    la    t0, s_ov_h;     lw t1, 0(t0); sw t1, UF_OVH(sp)

    lw    t0, UF_BX0(sp)
    blt   s9, t0, 1f; mv t0, s9
1:  sw    t0, UF_PXLOG(sp)

    lw    t0, UF_BX1(sp)
    addi  t1, s10, -1; add t1, t1, s9
    blt   t0, t1, 1f; mv t0, t1
1:  sw    t0, UF_PXHIG(sp)

    lw    t0, UF_X0(sp); lw t1, UF_X1(sp); sub t0, t1, t0; sw t0, UF_DX(sp)
    lw    t0, UF_Y0(sp); lw t1, UF_Y1(sp); sub t0, t1, t0; sw t0, UF_DY(sp)

    lw    a0, UF_DX(sp); bgez a0, 1f; neg a0, a0
1:  lw    a1, UF_DY(sp); bgez a1, 2f; neg a1, a1
2:  add   a0, a0, a1
    mul   t0, t2, a0; sw t0, UF_HCROSS(sp)

    lw    t0, UF_Y0(sp); sub t1, s4, t0
    lw    t0, UF_DX(sp); mul t1, t0, t1
    lw    t0, UF_DY(sp); lw t3, UF_X0(sp); mul t0, t0, t3
    add   t0, t0, t1; sw t0, UF_CPY(sp)

    j     .Luyd_pal_path

.Luyd_pal_path:
    lw    s2, UF_PXLOG(sp)
    lw    s3, UF_PXHIG(sp)
    bgt   s2, s3, .Luyd_ret
    blt   s5, s4, .Luyd_ret

    lw    t0, UF_HCROSS(sp)
    lw    t1, UF_DY(sp)
    beqz  t1, .Luydf_idy0
    bgez  t1, 1f; neg t1, t1
1:  divu  t0, t0, t1
    addi  t0, t0, 1
    sw    t0, UYDF_HALFW(sp)
    j     .Luydf_idy_done
.Luydf_idy0:
    sw    zero, UYDF_HALFW(sp)
.Luydf_idy_done:

    lw    a0, UF_DX(sp); lw a1, UF_DY(sp)
    mul   a2, a0, a0; mul a3, a1, a1; add a2, a2, a3
    sw    a2, UYDF_LEN2(sp)

    la    t0, g_beam_r2_q16; lw t0, 0(t0)
    mv    a5, t0

    la    t0, g_beam_r; lw t0, 0(t0); mul t0, t0, t0
    lw    t1, UYDF_LEN2(sp); mul t0, t0, t1; sw t0, UYDF_BR2L(sp)

    la    t0, g_gs; lw t0, 0(t0); sw t0, UYDF_GS(sp)

    lw    a2, UYDF_LEN2(sp)
    beqz  a2, .Luydf_noinv
    li    t0, -1; divu t0, t0, a2; sw t0, UYDF_INV(sp)
    lw    t0, UF_X0(sp); sub a0, s2, t0
    lw    t0, UF_Y0(sp); sub a1, s4, t0
    lw    t0, UF_DX(sp); lw t1, UF_DY(sp)
    mul   a2, t0, a1; mul a3, t1, a0; sub a2, a2, a3; sw a2, UYDF_CROW(sp)
    mul   a2, a0, t0; mul a3, a1, t1; add a2, a2, a3; sw a2, UYDF_DROW(sp)
    j     .Luydf_outer
.Luydf_noinv:
    sw    zero, UYDF_INV(sp)
    sw    zero, UYDF_CROW(sp)
    sw    zero, UYDF_DROW(sp)

    lw    t0, UYDF_LEN2(sp)
    beqz  t0, .Luydf_dot_loop

.Luydf_outer:
    mv    s11, s4

.Luydf_row:
    bgt   s11, s5, .Luyd_ret

    lw    t0, UF_OFFSY(sp); sub t0, s11, t0
    lw    t1, UF_OVH(sp); bgeu t0, t1, .Luydf_row_next

    mul   t5, s11, s1; slli t5, t5, 1; add t5, s0, t5
    mul   t6, t0, s10; add t6, s7, t6; sub t6, t6, s9; add t6, t6, s2
    slli  a0, s2, 1; add t1, t5, a0

    lw    a6, UYDF_LEN2(sp)

    lw    a1, UF_DY(sp)
    beqz  a1, .Luydf_nonarrow
    lw    a0, UYDF_HALFW(sp)
    beqz  a0, .Luydf_nonarrow
    lw    t2, UYDF_CROW(sp)
    div   a3, t2, a1
    add   a3, a3, s2
    sub   a2, a3, a0
    bge   a2, s2, 1f; mv a2, s2
1:  add   a3, a3, a0
    ble   a3, s3, 2f; mv a3, s3
2:  bgt   a2, a3, .Luydf_px_done
    sub   a4, a2, s2
    mul   a0, a1, a4
    lw    t2, UYDF_CROW(sp); sub t2, t2, a0
    lw    t3, UYDF_DROW(sp)
    lw    a0, UF_DX(sp); mul a0, a0, a4; add t3, t3, a0
    add   t6, t6, a4
    slli  a0, a4, 1; add t1, t1, a0
    mv    t4, a2
    mv    t5, a3
    j     .Luydf_px

.Luydf_nonarrow:
    lw    t2, UYDF_CROW(sp); lw t3, UYDF_DROW(sp)
    mv    t4, s2
    mv    t5, s3

.Luydf_px:
    bgt   t4, t5, .Luydf_px_done

    blez  t3, .Luydf_scap
    bge   t3, a6, .Luydf_ecap

    mul   a0, t2, t2
    mulh  a1, t2, t2
    bnez  a1, .Luydf_pxskip
    lw    a1, UYDF_BR2L(sp)
    bgeu  a1, a0, .Luydf_restore
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

.Luydf_beam:
    bgt   a0, a5, .Luydf_glow
    j     .Luydf_restore

.Luydf_glow:
    lw    a1, UYDF_GS(sp); beqz a1, .Luydf_pxskip
    sub   a2, a0, a5
    mul   a3, a2, a1; mulh a4, a2, a1
    slli  a4, a4, 2; srli a3, a3, 30; or a0, a4, a3
    li    a1, 178; bgeu a0, a1, .Luydf_pxskip
    la    a1, gauss_lut; add a1, a1, a0; lbu a1, 0(a1)
    beqz  a1, .Luydf_pxskip

.Luydf_restore:
    lbu   a0, 0(t6)
    andi  a1, a0, 0x80; beqz a1, .Luydf_pxskip
    andi  a0, a0, 0x7f
    slli  a1, a0, 1; add a1, a1, a0; add a1, s8, a1

    lbu   a2, 2(a1); mul a2, a2, s6; srli a2, a2, 8
    lbu   a3, 1(a1); mul a3, a3, s6; srli a3, a3, 8
    lbu   a4, 0(a1); mul a4, a4, s6; srli a4, a4, 8

    li    a0, 77;   mul a0, a0, a2
    li    a1, 150;  mul a1, a1, a3;  add a0, a0, a1
    li    a1, 29;   mul a1, a1, a4;  add a0, a0, a1
    srli  a0, a0, 8
    beqz  a0, .Luydf_y_skip
    sb    a0, 0(t1)
.Luydf_y_skip:

    slli  a0, a4, 7
    li    a1, 43;   mul a1, a1, a2;  sub a0, a0, a1
    li    a1, 85;   mul a1, a1, a3;  sub a0, a0, a1
    srai  a0, a0, 8
    slli  a1, a2, 7
    li    t0, 107;  mul t0, t0, a3;  sub a1, a1, t0
    li    t0, 21;   mul t0, t0, a4;  sub a1, a1, t0
    srai  a1, a1, 8
    or    t0, a0, a1
    beqz  t0, .Luydf_uv_skip
    andi  a2, t4, 1;  slli a2, a2, 1;  li a3, 1;  sub a2, a3, a2;  add a2, a2, t1
    addi  a3, a0, 128;  sb a3, 0(a2)
    addi  a3, a1, 128;  sb a3, 2(a2)
.Luydf_uv_skip:

.Luydf_pxskip:
    addi  t6, t6, 1
    addi  t1, t1, 2
    lw    a1, UF_DY(sp); sub t2, t2, a1
    lw    a1, UF_DX(sp); add t3, t3, a1
.Luydf_px_incr:
    addi  t4, t4, 1
    ble   t4, t5, .Luydf_px

.Luydf_px_done:
    lw    a0, UYDF_CROW(sp); lw a1, UF_DX(sp); add a0, a0, a1; sw a0, UYDF_CROW(sp)
    lw    a0, UYDF_DROW(sp); lw a1, UF_DY(sp); add a0, a0, a1; sw a0, UYDF_DROW(sp)
.Luydf_row_next:
    addi  s11, s11, 1
    j     .Luydf_row

.Luydf_dot_loop:
    mv    s11, s4
.Luydf_d_row:
    bgt   s11, s5, .Luyd_ret
    lw    t0, UF_OFFSY(sp); sub t0, s11, t0
    lw    t1, UF_OVH(sp); bgeu t0, t1, .Luydf_d_row_next
    mul   t5, s11, s1; slli t5, t5, 1; add t5, s0, t5
    mul   t6, t0, s10; add t6, s7, t6; sub t6, t6, s9; add t6, t6, s2
    slli  a0, s2, 1; add t1, t5, a0
    mv    t4, s2
.Luydf_d_px:
    bgt   t4, s3, .Luydf_d_row_next
    lbu   a0, 0(t6); addi t6, t6, 1
    andi  a1, a0, 0x80; beqz a1, .Luydf_d_px_next
    andi  t0, a0, 0x7f
    lw    a0, UF_X0(sp); lw a1, UF_Y0(sp)
    sub   a2, t4, a0; sub a3, s11, a1
    slli  a0, a2, 8; mul a0, a0, a0
    slli  a1, a3, 8; mul a1, a1, a1; add a0, a0, a1
    bgt   a0, a5, .Luydf_d_glow
.Luydf_d_restore:
    slli  a1, t0, 1; add a1, a1, t0; add a1, s8, a1
    lbu   a2, 2(a1); mul a2, a2, s6; srli a2, a2, 8
    lbu   a3, 1(a1); mul a3, a3, s6; srli a3, a3, 8
    lbu   a4, 0(a1); mul a4, a4, s6; srli a4, a4, 8
    li    a0, 77; mul a0, a0, a2
    li    a1, 150; mul a1, a1, a3; add a0, a0, a1
    li    a1, 29;  mul a1, a1, a4; add a0, a0, a1
    srli  a0, a0, 8
    beqz  a0, .Luydf_d_y_skip
    sb    a0, 0(t1)
.Luydf_d_y_skip:
    slli  a0, a4, 7
    li    a1, 43;  mul a1, a1, a2;  sub a0, a0, a1
    li    a1, 85;  mul a1, a1, a3;  sub a0, a0, a1
    srai  a0, a0, 8
    slli  a1, a2, 7
    li    t0, 107; mul t0, t0, a3;  sub a1, a1, t0
    li    t0, 21;  mul t0, t0, a4;  sub a1, a1, t0
    srai  a1, a1, 8
    or    t0, a0, a1
    beqz  t0, .Luydf_d_uv_skip
    andi  a2, t4, 1;  slli a2, a2, 1;  li a3, 1;  sub a2, a3, a2;  add a2, a2, t1
    addi  a3, a0, 128;  sb a3, 0(a2)
    addi  a3, a1, 128;  sb a3, 2(a2)
.Luydf_d_uv_skip:
    j     .Luydf_d_px_next
.Luydf_d_glow:
    lw    a1, UYDF_GS(sp); beqz a1, .Luydf_d_px_next
    sub   a2, a0, a5
    mul   a3, a2, a1; mulh a4, a2, a1
    slli  a4, a4, 2; srli a3, a3, 30; or a0, a4, a3
    li    a1, 178; bgeu a0, a1, .Luydf_d_px_next
    la    a1, gauss_lut; add a1, a1, a0; lbu a1, 0(a1)
    beqz  a1, .Luydf_d_px_next
    j     .Luydf_d_restore
.Luydf_d_px_next:
    addi  t1, t1, 2
    addi  t4, t4, 1
    j     .Luydf_d_px
.Luydf_d_row_next:
    addi  s11, s11, 1
    j     .Luydf_d_row

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
#define DF_HALFW   120
#define DF_BR2L    124
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
    mv    s6, a7

    la    t0, s_overlay_pal; lw s7, 0(t0)
    la    s8, s_overlay_palette
    la    t0, s_ov_off_x;  lw s9,  0(t0)
    la    t0, s_ov_w;      lw s10, 0(t0)
    la    t0, s_ov_off_y;  lw t1, 0(t0);  sw t1, DF_OFFSY(sp)
    la    t0, s_ov_h;      lw t1, 0(t0);  sw t1, DF_OVH(sp)

    la    t0, s_overlay_alpha_val; lbu t0, 0(t0)
    li    t1, 255
    bge   t0, t1, .Ldyv_ret
    sw    t0, DF_EA(sp)

    la    t2, g_line_Rb
    lw    t2, 0(t2)

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

    lw    t0, DF_BX0(sp)
    blt   s9, t0, 1f; mv t0, s9
1:  mv    s2, t0

    lw    t0, DF_BX1(sp)
    addi  t1, s10, -1; add t1, t1, s9
    blt   t0, t1, 1f; mv t0, t1
1:  mv    s3, t0

    lw    a3, DF_X0(sp); lw a5, DF_X1(sp); sub t0, a5, a3; sw t0, DF_DX(sp)
    lw    a4, DF_Y0(sp); lw a6, DF_Y1(sp); sub t1, a6, a4; sw t1, DF_DY(sp)
    mul   a3, t0, t0;  mul a4, t1, t1;  add a3, a3, a4;  sw a3, DF_LEN2(sp)

    bgez  t0, 1f; neg t0, t0
1:  bgez  t1, 2f; neg t1, t1
2:  add   t0, t0, t1
    la    t1, g_line_Rb; lw t1, 0(t1); mul t0, t0, t1
    lw    t1, DF_DY(sp); beqz t1, .Ldyv_halfw_dy0
    bgez  t1, 3f; neg t1, t1
3:  divu  t0, t0, t1; addi t0, t0, 1
    sw    t0, DF_HALFW(sp)
    j     .Ldyv_halfw_done
.Ldyv_halfw_dy0:
    sw    zero, DF_HALFW(sp)
.Ldyv_halfw_done:

    la    t1, g_beam_r2_q16; lw t1, 0(t1); sw t1, DF_BR2(sp)
    la    t1, g_beam_r; lw t1, 0(t1); mul t1, t1, t1
    lw    t0, DF_LEN2(sp); mul t1, t1, t0; sw t1, DF_BR2L(sp)
    la    t0, g_gs; lw t0, 0(t0); sw t0, DF_GS(sp)

    lw    a3, DF_LEN2(sp)
    beqz  a3, .Ldyv_dot_loop

    li    t0, -1; divu t0, t0, a3; sw t0, DF_INV(sp)

    lw    t0, DF_X0(sp); sub a0, s2, t0
    lw    t0, DF_Y0(sp); sub a1, s4, t0
    lw    t0, DF_DX(sp); lw  t1, DF_DY(sp)
    mul   a2, t0, a1; mul a3, t1, a0; sub a2, a2, a3; sw a2, DF_CROW(sp)
    mul   a2, a0, t0; mul a3, a1, t1; add a2, a2, a3; sw a2, DF_DROW(sp)

.Ldyv_setup_done:
    j     .Ldyv_pal_outer

.Ldyv_pal_outer:
    blt   s3, s2, .Ldyv_ret
    mv    s11, s4

.Ldyv_pal_py:
    bgt   s11, s5, .Ldyv_ret

    lw    t0, DF_OFFSY(sp); sub t0, s11, t0
    lw    t1, DF_OVH(sp); bgeu t0, t1, .Ldyv_pal_py_next

    mul   t5, s11, s1; slli t5, t5, 1; add t5, s0, t5
    mul   a0, t0, s10; add a0, s7, a0; sub a0, a0, s9; add t6, a0, s2
    slli  a0, s2, 1; add t1, t5, a0

    lw    a6, DF_LEN2(sp); lw a7, DF_BR2(sp)

    lw    a1, DF_DY(sp)
    beqz  a1, .Ldyv_nonarrow
    lw    a0, DF_HALFW(sp)
    beqz  a0, .Ldyv_nonarrow
    lw    t2, DF_CROW(sp)
    div   a3, t2, a1
    add   a3, a3, s2
    sub   a2, a3, a0
    bge   a2, s2, 1f; mv a2, s2
1:  add   a3, a3, a0
    ble   a3, s3, 2f; mv a3, s3
2:  bgt   a2, a3, .Ldyv_pal_px_done
    sub   a4, a2, s2
    mul   a0, a1, a4
    lw    t2, DF_CROW(sp); sub t2, t2, a0
    lw    t3, DF_DROW(sp)
    lw    a0, DF_DX(sp); mul a0, a0, a4; add t3, t3, a0
    add   t6, t6, a4
    slli  a0, a4, 1; add t1, t1, a0
    mv    t4, a2
    mv    t5, a3
    j     .Ldyv_pal_px

.Ldyv_nonarrow:
    lw    t2, DF_CROW(sp); lw t3, DF_DROW(sp)
    mv    t4, s2
    mv    t5, s3

.Ldyv_pal_px:
    bgt   t4, t5, .Ldyv_pal_px_done

    lbu   a0, 0(t6)
    addi  t6, t6, 1

    andi  a1, a0, 0x80
    beqz  a1, .Ldyv_pal_skip

    andi  t0, a0, 0x7f

    blez  t3, .Ldyv_pal_scap
    bge   t3, a6, .Ldyv_pal_ecap

    mul   a0, t2, t2
    mulh  a1, t2, t2
    bnez  a1, .Ldyv_pal_skip
    lw    a1, DF_BR2L(sp)
    bgeu  a1, a0, .Ldyv_pal_in_beam
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

.Ldyv_pal_beam:
    bgt   a0, a7, .Ldyv_pal_glow
.Ldyv_pal_in_beam:
    mv    a0, s6
    j     .Ldyv_pal_write

.Ldyv_pal_glow:
    lw    a1, DF_GS(sp); beqz a1, .Ldyv_pal_skip
    sub   a2, a0, a7
    mul   a3, a2, a1; mulh a4, a2, a1
    slli  a4, a4, 2; srli a3, a3, 30; or a0, a4, a3
    li    a1, 178; bgeu a0, a1, .Ldyv_pal_skip
    la    a1, gauss_lut; add a1, a1, a0; lbu a1, 0(a1)
    mul   a0, s6, a1; srai a0, a0, 8
    beqz  a0, .Ldyv_pal_skip

.Ldyv_pal_write:
    slli  a1, t0, 1; add a1, a1, t0; add a1, s8, a1
    lbu   a2, 2(a1)
    lbu   a3, 1(a1)
    lbu   a4, 0(a1)

    li    a5, 77;   mul a5, a5, a2
    li    t0, 150;  mul t0, t0, a3;  add a5, a5, t0
    li    t0, 29;   mul t0, t0, a4;  add a5, a5, t0
    srli  a5, a5, 8
    mul   a5, a5, a0
    srli  a5, a5, 8

    beqz  a5, .Ldyv_y_skip
    lbu   t0, 0(t1)
    add   a5, a5, t0
    li    t0, 255;  blt a5, t0, 1f;  mv a5, t0
1:  sb    a5, 0(t1)
.Ldyv_y_skip:

    slli  t0, a4, 7
    li    a1, 43;  mul a1, a1, a2;  sub t0, t0, a1
    li    a1, 85;  mul a1, a1, a3;  sub t0, t0, a1
    mul   t0, t0, a0;  srai t0, t0, 16
    slli  a5, a2, 7
    li    a1, 107; mul a1, a1, a3;  sub a5, a5, a1
    li    a1, 21;  mul a1, a1, a4;  sub a5, a5, a1
    mul   a5, a5, a0;  srai a5, a5, 16
    or    a1, t0, a5
    beqz  a1, .Ldyv_uv_skip
    andi  a1, t4, 1;  slli a1, a1, 1;  li a2, 1;  sub a2, a2, a1;  add a2, a2, t1
    lbu   a1, 0(a2);  add a1, a1, t0
    bge   a1, zero, 1f;  li a1, 0
1:  li    t0, 255;  blt a1, t0, 2f;  mv a1, t0
2:  sb    a1, 0(a2)
    lbu   a1, 2(a2);  add a1, a1, a5
    bge   a1, zero, 1f;  li a1, 0
1:  li    t0, 255;  blt a1, t0, 2f;  mv a1, t0
2:  sb    a1, 2(a2)
.Ldyv_uv_skip:

.Ldyv_pal_skip:
    addi  t1, t1, 2
    lw    a1, DF_DY(sp); sub t2, t2, a1
    lw    a1, DF_DX(sp); add t3, t3, a1
.Ldyv_pal_px_incr:
    addi  t4, t4, 1
    ble   t4, t5, .Ldyv_pal_px

.Ldyv_pal_px_done:
    lw    a0, DF_CROW(sp); lw a1, DF_DX(sp); add a0, a0, a1; sw a0, DF_CROW(sp)
    lw    a0, DF_DROW(sp); lw a1, DF_DY(sp); add a0, a0, a1; sw a0, DF_DROW(sp)
.Ldyv_pal_py_next:
    addi  s11, s11, 1
    j     .Ldyv_pal_py

.Ldyv_dot_loop:
    lw    a2, DF_X0(sp)
    lw    a3, DF_Y0(sp)
    lw    a7, DF_BR2(sp)
    mv    s11, s4
.Ldyv_dpy:
    bgt   s11, s5, .Ldyv_ret
    lw    t0, DF_OFFSY(sp); sub t0, s11, t0
    lw    t1, DF_OVH(sp); bgeu t0, t1, .Ldyv_dpy_next
    mul   t5, s11, s1; slli t5, t5, 1; add t5, s0, t5
    mul   a0, t0, s10; add a0, s7, a0; sub a0, a0, s9
    mv    t6, s2
.Ldyv_dpx:
    bgt   t6, s3, .Ldyv_dpy_next
    add   a1, a0, t6; lbu a1, 0(a1)
    andi  a5, a1, 0x80
    beqz  a5, .Ldyv_dpx_next
    andi  t0, a1, 0x7f
    sub   a4, t6, a2; slli a4, a4, 8; mul a4, a4, a4
    sub   a5, s11, a3; slli a5, a5, 8; mul a5, a5, a5; add a1, a4, a5
    bgt   a1, a7, .Ldyv_dglow
    mv    a1, s6
    j     .Ldyv_dwrite
.Ldyv_dglow:
    lw    a5, DF_GS(sp); beqz a5, .Ldyv_dpx_next
    sub   a4, a1, a7
    mul   a3, a4, a5; mulh a4, a4, a5
    slli  a4, a4, 2; srli a3, a3, 30; or a1, a4, a3
    li    a5, 178; bgeu a1, a5, .Ldyv_dpx_next
    la    a5, gauss_lut; add a5, a5, a1; lbu a5, 0(a5)
    mul   a1, s6, a5; srai a1, a1, 8
    beqz  a1, .Ldyv_dpx_next
    lw    a3, DF_Y0(sp)
.Ldyv_dwrite:
    slli  a5, t0, 1; add a5, a5, t0; add a5, s8, a5
    lbu   a2, 2(a5)
    lbu   a3, 1(a5)
    lbu   a4, 0(a5)
    li    a5, 77;  mul a5, a5, a2
    li    t0, 150; mul t0, t0, a3; add a5, a5, t0
    li    t0, 29;  mul t0, t0, a4; add a5, a5, t0
    srli  a5, a5, 8
    mul   a5, a5, a1; srli a5, a5, 8
    slli  t1, t6, 1; add t1, t5, t1
    beqz  a5, .Ldyv_d_y_skip
    lbu   t0, 0(t1); add a5, a5, t0
    li    t0, 255; blt a5, t0, 1f; mv a5, t0
1:  sb    a5, 0(t1)
.Ldyv_d_y_skip:
    slli  t0, a4, 7
    li    t2, 43;  mul t2, t2, a2;  sub t0, t0, t2
    li    t2, 85;  mul t2, t2, a3;  sub t0, t0, t2
    mul   t0, t0, a1;  srai t0, t0, 16
    slli  a5, a2, 7
    li    t2, 107; mul t2, t2, a3;  sub a5, a5, t2
    li    t2, 21;  mul t2, t2, a4;  sub a5, a5, t2
    mul   a5, a5, a1;  srai a5, a5, 16
    or    t2, t0, a5
    beqz  t2, .Ldyv_d_uv_skip
    andi  t2, t6, 1;  slli t2, t2, 1;  li a4, 1;  sub a4, a4, t2;  add a4, a4, t1
    lbu   t2, 0(a4);  add t2, t2, t0
    bge   t2, zero, 1f;  li t2, 0
1:  li    t0, 255;  blt t2, t0, 2f;  mv t2, t0
2:  sb    t2, 0(a4)
    lbu   t2, 2(a4);  add t2, t2, a5
    bge   t2, zero, 1f;  li t2, 0
1:  li    t0, 255;  blt t2, t0, 2f;  mv t2, t0
2:  sb    t2, 2(a4)
.Ldyv_d_uv_skip:
    lw    a2, DF_X0(sp)
    lw    a3, DF_Y0(sp)
.Ldyv_dpx_next:
    addi  t6, t6, 1
    j     .Ldyv_dpx
.Ldyv_dpy_next:
    addi  s11, s11, 1
    j     .Ldyv_dpy

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

/* ══════════════════════════════════════════════════════════════════════════
 *  undraw_line_yuv422
 *
 *  void undraw_line_yuv422(
 *    a0=fb, a1=fb_w, a2=fb_h,
 *    a3=x0, a4=y0, a5=x1, a6=y1, a7=colorPaletteEntry)
 *
 *  Erases beam/glow footprint to black (Y=0, U=128, V=128).
 *  Gates each channel write with the same delta test as draw — if draw
 *  would not have touched a channel, undraw does not touch it either.
 *
 *  Register map:
 *    s0  fb          s6  brightness (255)
 *    s1  fb_w        s7  r_col
 *    s2  px_lo       s8  g_col
 *    s3  px_hi       s9  b_col
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

    .globl undraw_line_yuv422_color
    .type  undraw_line_yuv422_color, @function

undraw_line_yuv422_color:
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
    li    s6, 255          /* brightness */

    /* Fixed colour from s_overlay_palette[colorPaletteEntry & 0x7F] */
    andi  a7, a7, 0x7F
    la    t0, s_overlay_palette
    slli  t1, a7, 1; add t1, t1, a7   /* t1 = idx*3 */
    add   t0, t0, t1
    lbu   s9, 0(t0)                    /* b_col */
    lbu   s8, 1(t0)                    /* g_col */
    lbu   s7, 2(t0)                    /* r_col */

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

    /* Opt 2: load precomputed beam/glow globals */
    la    t0, g_beam_r2_q16; lw t0, 0(t0)
    mv    a5, t0                   /* a5 = beam_r2_q16 (cached) */

    /* beam_r2_len2 = beam_r^2 * len2 */
    la    t0, g_beam_r; lw t0, 0(t0)
    mul   t0, t0, t0
    lw    t1, UL_LEN2(sp); mul t0, t0, t1
    sw    t0, UL_BR2L(sp)

    la    t0, g_gs; lw t0, 0(t0); sw t0, UL_GS(sp)

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

    /* Opt 1: dot case (len2==0) — separate simple loop */
    lw    t0, UL_LEN2(sp)
    beqz  t0, .Luyl_dot_loop

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
    blez  t3, .Luyl_scap
    bge   t3, a6, .Luyl_ecap

    mul   a0, t2, t2
    mulh  a1, t2, t2               /* upper 32 bits of cross^2 */
    bnez  a1, .Luyl_pxskip         /* overflow → far outside, skip */
    lw    a1, UL_BR2L(sp)
    bgeu  a1, a0, .Luyl_in_beam   /* csq <= beam_r2_len2 → in beam */
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

.Luyl_beam:                        /* a0=d2_q16, a5=beam_r2_q16 */
    bgt   a0, a5, .Luyl_glow
.Luyl_in_beam:
    mv    a0, s6                   /* contrib = brightness (255) */
    j     .Luyl_do_write

.Luyl_glow:
    lw    a1, UL_GS(sp); beqz a1, .Luyl_pxskip
    sub   a2, a0, a5               /* glow_d2 */
    mul   a3, a2, a1; mulh a4, a2, a1
    slli  a4, a4, 2; srli a3, a3, 30; or a0, a4, a3   /* lut_idx */
    li    a1, 178; bgeu a0, a1, .Luyl_pxskip
    la    a1, gauss_lut; add a1, a1, a0; lbu a1, 0(a1)
    mul   a0, s6, a1; srai a0, a0, 8
    beqz  a0, .Luyl_pxskip

.Luyl_do_write:
    /* a0=contrib; t1=fb_ptr; t2=cross; t4=px; a5=beam_r2_q16 — all preserved */

    /* y_contrib = (77*r+150*g+29*b)>>8 * contrib>>8; if nonzero: write Y=0 */
    li    t0, 77;  mul t0, t0, s7
    li    a1, 150; mul a1, a1, s8; add t0, t0, a1
    li    a1, 29;  mul a1, a1, s9; add t0, t0, a1
    srli  t0, t0, 8                     /* t0 = Y_full */
    mul   t0, t0, a0; srli t0, t0, 8   /* t0 = y_contrib */
    beqz  t0, .Luyl_y_skip
    sb    zero, 0(t1)
.Luyl_y_skip:

    /* u_delta = (128*b - 43*r - 85*g) * contrib >> 16 */
    slli  a1, s9, 7
    li    t0, 43;  mul t0, t0, s7; sub a1, a1, t0
    li    t0, 85;  mul t0, t0, s8; sub a1, a1, t0
    mul   a1, a1, a0; srai a1, a1, 16   /* a1 = u_delta */

    /* v_delta = (128*r - 107*g - 21*b) * contrib >> 16 */
    slli  a2, s7, 7
    li    t0, 107; mul t0, t0, s8; sub a2, a2, t0
    li    t0, 21;  mul t0, t0, s9; sub a2, a2, t0
    mul   a2, a2, a0; srai a2, a2, 16   /* a2 = v_delta */

    or    t0, a1, a2
    beqz  t0, .Luyl_uv_skip             /* both zero — no PSRAM */
    andi  a3, t4, 1; slli a3, a3, 1; li a4, 1; sub a3, a4, a3; add a3, a3, t1
    li    t0, 128; sb t0, 0(a3); sb t0, 2(a3)
.Luyl_uv_skip:

.Luyl_pxskip:
    addi  t1, t1, 2
    lw    a1, UL_DY(sp); sub t2, t2, a1
    lw    a1, UL_DX(sp); add t3, t3, a1
.Luyl_px_incr:
    addi  t4, t4, 1
    ble   t4, t5, .Luyl_px

.Luyl_px_done:
    lw    a0, UL_CROW(sp); lw a1, UL_DX(sp); add a0, a0, a1; sw a0, UL_CROW(sp)
    lw    a0, UL_DROW(sp); lw a1, UL_DY(sp); add a0, a0, a1; sw a0, UL_DROW(sp)
.Luyl_row_next:
    addi  s11, s11, 1
    j     .Luyl_row

    /* ════════════════════════════════════
     * Opt 1: Dot loop (len2==0) — full bbox, erase with same gates as draw
     * ════════════════════════════════════ */
.Luyl_dot_loop:
    mv    s11, s4
.Luyl_d_row:
    bgt   s11, s5, .Luyl_ret
    mul   t5, s11, s1; slli t5, t5, 1; add t5, s0, t5
    slli  a0, s2, 1; add t1, t5, a0
    mv    t4, s2
.Luyl_d_px:
    bgt   t4, s3, .Luyl_d_row_next
    /* distance to (x0,y0) */
    lw    a0, UL_X0(sp); lw a1, UL_Y0(sp)
    sub   a2, t4, a0; sub a3, s11, a1
    slli  a0, a2, 8; mul a0, a0, a0
    slli  a1, a3, 8; mul a1, a1, a1; add a0, a0, a1
    bgt   a0, a5, .Luyl_d_glow
    mv    a0, s6                    /* contrib = brightness */
    j     .Luyl_d_write
.Luyl_d_glow:
    lw    a1, UL_GS(sp); beqz a1, .Luyl_d_px_next
    sub   a2, a0, a5
    mul   a3, a2, a1; mulh a4, a2, a1
    slli  a4, a4, 2; srli a3, a3, 30; or a0, a4, a3
    li    a1, 178; bgeu a0, a1, .Luyl_d_px_next
    la    a1, gauss_lut; add a1, a1, a0; lbu a1, 0(a1)
    mul   a0, s6, a1; srai a0, a0, 8
    beqz  a0, .Luyl_d_px_next
.Luyl_d_write:
    /* a0=contrib; t1=fb_ptr; t4=px; a5=beam_r2_q16 — preserved */

    /* y_contrib gate */
    li    t0, 77;  mul t0, t0, s7
    li    a1, 150; mul a1, a1, s8; add t0, t0, a1
    li    a1, 29;  mul a1, a1, s9; add t0, t0, a1
    srli  t0, t0, 8
    mul   t0, t0, a0; srli t0, t0, 8   /* t0 = y_contrib */
    beqz  t0, .Luyl_d_y_skip
    sb    zero, 0(t1)
.Luyl_d_y_skip:

    /* u_delta / v_delta gate */
    slli  a1, s9, 7
    li    t0, 43;  mul t0, t0, s7; sub a1, a1, t0
    li    t0, 85;  mul t0, t0, s8; sub a1, a1, t0
    mul   a1, a1, a0; srai a1, a1, 16   /* a1 = u_delta */
    slli  a2, s7, 7
    li    t0, 107; mul t0, t0, s8; sub a2, a2, t0
    li    t0, 21;  mul t0, t0, s9; sub a2, a2, t0
    mul   a2, a2, a0; srai a2, a2, 16   /* a2 = v_delta */
    or    t0, a1, a2
    beqz  t0, .Luyl_d_uv_skip
    andi  a3, t4, 1; slli a3, a3, 1; li a4, 1; sub a3, a4, a3; add a3, a3, t1
    li    t0, 128; sb t0, 0(a3); sb t0, 2(a3)
.Luyl_d_uv_skip:

.Luyl_d_px_next:
    addi  t1, t1, 2
    addi  t4, t4, 1
    j     .Luyl_d_px
.Luyl_d_row_next:
    addi  s11, s11, 1
    j     .Luyl_d_row

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
    .size undraw_line_yuv422_color, .-undraw_line_yuv422_color

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

    .globl draw_line_yuv422_color
    .type  draw_line_yuv422_color, @function

draw_line_yuv422_color:
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

    /* Opt 2: load precomputed beam/glow globals */
    la    t1, g_beam_r2_q16; lw t1, 0(t1); sw t1, DL_BR2(sp)

    /* beam_r2_len2 = beam_r^2 * len2 */
    la    t1, g_beam_r; lw t1, 0(t1)
    mul   t1, t1, t1
    lw    t0, DL_LEN2(sp); mul t1, t1, t0; sw t1, DL_BR2L(sp)

    la    t0, g_gs; lw t0, 0(t0); sw t0, DL_GS(sp)

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

    /* Opt 1: dot case (len2==0) — separate simple loop */
    lw    t0, DL_LEN2(sp)
    beqz  t0, .Ldyl_dot_loop

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
    beqz  a5, .Ldyl_y_skip
    lbu   t0, 0(t1); add a5, a5, t0
    li    t0, 255;  blt a5, t0, 1f;  mv a5, t0
1:  sb    a5, 0(t1)
.Ldyl_y_skip:

    /* U/V deltas — compute before any PSRAM access; skip both if zero */
    slli  t0, s9, 7
    li    a1, 43;  mul a1, a1, s7;  sub t0, t0, a1
    li    a1, 85;  mul a1, a1, s8;  sub t0, t0, a1
    mul   t0, t0, a0;  srai t0, t0, 16              /* t0 = u_delta */
    slli  a5, s7, 7
    li    a1, 107; mul a1, a1, s8;  sub a5, a5, a1
    li    a1, 21;  mul a1, a1, s9;  sub a5, a5, a1
    mul   a5, a5, a0;  srai a5, a5, 16              /* a5 = v_delta */
    or    a1, t0, a5
    beqz  a1, .Ldyl_uv_skip                         /* both zero — no PSRAM */
    /* u_ptr into a1 */
    andi  a1, t4, 1;  slli a1, a1, 1;  li t2, 1;  sub a1, t2, a1;  add a1, a1, t1
    /* U write */
    lbu   t2, 0(a1);  add t2, t2, t0
    bge   t2, zero, 1f;  li t2, 0
1:  li    t0, 255;  blt t2, t0, 2f;  mv t2, t0
2:  sb    t2, 0(a1)
    /* V write */
    lbu   t2, 2(a1);  add t2, t2, a5
    bge   t2, zero, 1f;  li t2, 0
1:  li    t0, 255;  blt t2, t0, 2f;  mv t2, t0
2:  sb    t2, 2(a1)
.Ldyl_uv_skip:

.Ldyl_skip:
    addi  t1, t1, 2                /* fb_ptr += 2 */
    lw    a1, DL_DY(sp); sub t2, t2, a1
    lw    a1, DL_DX(sp); add t3, t3, a1
.Ldyl_px_incr:
    addi  t4, t4, 1
    ble   t4, t5, .Ldyl_px

.Ldyl_px_done:
    lw    a0, DL_CROW(sp); lw a1, DL_DX(sp); add a0, a0, a1; sw a0, DL_CROW(sp)
    lw    a0, DL_DROW(sp); lw a1, DL_DY(sp); add a0, a0, a1; sw a0, DL_DROW(sp)
.Ldyl_py_next:
    addi  s11, s11, 1
    j     .Ldyl_py

    /* ════════════════════════════════════
     * Opt 1: Dot loop (len2==0) — full bbox, distance to (x0,y0)
     * ════════════════════════════════════ */
.Ldyl_dot_loop:
    lw    a2, DL_X0(sp)            /* x0 */
    lw    a3, DL_Y0(sp)            /* y0 */
    lw    a7, DL_BR2(sp)           /* beam_r2_q16 */
    mv    s11, s4
.Ldyl_d_row:
    bgt   s11, s5, .Ldyl_ret
    mul   t5, s11, s1; slli t5, t5, 1; add t5, s0, t5
    slli  a0, s2, 1; add t1, t5, a0
    mv    t4, s2
.Ldyl_d_px:
    bgt   t4, s3, .Ldyl_d_row_next
    sub   a4, t4, a2; sub a5, s11, a3
    slli  a4, a4, 8; mul a4, a4, a4
    slli  a5, a5, 8; mul a5, a5, a5; add a0, a4, a5
    bgt   a0, a7, .Ldyl_d_glow
    mv    a0, s6                   /* contrib = brightness */
    j     .Ldyl_d_write
.Ldyl_d_glow:
    lw    a1, DL_GS(sp); beqz a1, .Ldyl_d_skip
    sub   a4, a0, a7
    mul   a5, a4, a1; mulh t0, a4, a1
    slli  t0, t0, 2; srli a5, a5, 30; or a0, t0, a5
    li    a1, 178; bgeu a0, a1, .Ldyl_d_skip
    la    a1, gauss_lut; add a1, a1, a0; lbu a1, 0(a1)
    mul   a0, s6, a1; srai a0, a0, 8
    beqz  a0, .Ldyl_d_skip
.Ldyl_d_write:
    /* Y additive */
    li    a5, 77;  mul a5, a5, s7
    li    t0, 150; mul t0, t0, s8; add a5, a5, t0
    li    t0, 29;  mul t0, t0, s9; add a5, a5, t0
    srli  a5, a5, 8
    mul   a5, a5, a0; srli a5, a5, 8
    beqz  a5, .Ldyl_d_y_skip
    lbu   t0, 0(t1); add a5, a5, t0
    li    t0, 255; blt a5, t0, 1f; mv a5, t0
1:  sb    a5, 0(t1)
.Ldyl_d_y_skip:
    /* U/V deltas — compute before any PSRAM access; skip both if zero */
    slli  t0, s9, 7
    li    a5, 43;  mul a5, a5, s7;  sub t0, t0, a5
    li    a5, 85;  mul a5, a5, s8;  sub t0, t0, a5
    mul   t0, t0, a0;  srai t0, t0, 16              /* t0 = u_delta */
    slli  a5, s7, 7
    li    a1, 107; mul a1, a1, s8;  sub a5, a5, a1
    li    a1, 21;  mul a1, a1, s9;  sub a5, a5, a1
    mul   a5, a5, a0;  srai a5, a5, 16              /* a5 = v_delta */
    or    a1, t0, a5
    beqz  a1, .Ldyl_d_uv_skip                       /* both zero — no PSRAM */
    /* u_ptr into a1 */
    andi  a1, t4, 1;  slli a1, a1, 1;  li t2, 1;  sub a1, t2, a1;  add a1, a1, t1
    /* U write */
    lbu   t2, 0(a1);  add t2, t2, t0
    bge   t2, zero, 1f;  li t2, 0
1:  li    t0, 255;  blt t2, t0, 2f;  mv t2, t0
2:  sb    t2, 0(a1)
    /* V write */
    lbu   t2, 2(a1);  add t2, t2, a5
    bge   t2, zero, 1f;  li t2, 0
1:  li    t0, 255;  blt t2, t0, 2f;  mv t2, t0
2:  sb    t2, 2(a1)
.Ldyl_d_uv_skip:
    lw    a3, DL_Y0(sp)            /* restore y0 */
.Ldyl_d_skip:
    addi  t1, t1, 2
    addi  t4, t4, 1
    j     .Ldyl_d_px
.Ldyl_d_row_next:
    addi  s11, s11, 1
    j     .Ldyl_d_row

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
    .size draw_line_yuv422_color, .-draw_line_yuv422_color

/* ══════════════════════════════════════════════════════════════════════════
 *  draw_line_yuv422_brightness
 *
 *  void draw_line_yuv422_brightness(
 *    a0=fb, a1=fb_w, a2=fb_h,
 *    a3=x0, a4=y0, a5=x1, a6=y1, a7=brightness)
 *
 *  White (R=G=B=255) additive line at given brightness level.
 *  Algorithmically identical to draw_line_yuv422 except:
 *    - a7=brightness stored directly in s6; no palette lookup needed.
 *    - Y_full = 255 always for white → y_contrib = (255*contrib)>>8.
 *    - UV deltas always zero for white → UV section omitted entirely.
 *
 *  Register map:
 *    s0  fb          s6  brightness
 *    s1  fb_w        s7-s9 unused (white: no palette)
 *    s2  px_lo       s10 (free)
 *    s3  px_hi       s11 py
 *    s4  by0
 *    s5  by1
 *
 *  Stack frame 128 bytes (same layout/offsets as draw_line_yuv422).
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
    blez  s6, .Lbyl_ret

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
2:  beqz  t1, .Lbyl_hw0
    add   t0, t0, t1
    mul   t0, t0, t2
    divu  t0, t0, t1; addi t0, t0, 1
    sw    t0, DL_HALFW(sp)
    j     .Lbyl_hw_done
.Lbyl_hw0:
    sw    zero, DL_HALFW(sp)
.Lbyl_hw_done:

    la    t1, g_beam_r2_q16; lw t1, 0(t1); sw t1, DL_BR2(sp)
    la    t1, g_beam_r; lw t1, 0(t1)
    mul   t1, t1, t1
    lw    t0, DL_LEN2(sp); mul t1, t1, t0; sw t1, DL_BR2L(sp)
    la    t0, g_gs; lw t0, 0(t0); sw t0, DL_GS(sp)

    lw    a3, DL_LEN2(sp)
    beqz  a3, .Lbyl_setup_done
    li    t0, -1; divu t0, t0, a3; sw t0, DL_INV(sp)
    lw    t0, DL_X0(sp); sub a0, s2, t0
    lw    t0, DL_Y0(sp); sub a1, s4, t0
    lw    t0, DL_DX(sp); lw  t1, DL_DY(sp)
    mul   a2, t0, a1; mul a3, t1, a0; sub a2, a2, a3; sw a2, DL_CROW(sp)
    mul   a2, a0, t0; mul a3, a1, t1; add a2, a2, a3; sw a2, DL_DROW(sp)

.Lbyl_setup_done:
    blt   s3, s2, .Lbyl_ret

    lw    t0, DL_LEN2(sp)
    beqz  t0, .Lbyl_dot_loop

    mv    s11, s4

.Lbyl_py:
    bgt   s11, s5, .Lbyl_ret

    mul   t5, s11, s1; slli t5, t5, 1; add t5, s0, t5
    slli  a0, s2, 1; add t1, t5, a0

    lw    a6, DL_LEN2(sp); lw a7, DL_BR2(sp)

    lw    a1, DL_DY(sp)
    beqz  a1, .Lbyl_nonarrow
    lw    a0, DL_HALFW(sp)
    beqz  a0, .Lbyl_nonarrow
    lw    t2, DL_CROW(sp)
    div   a3, t2, a1
    add   a3, a3, s2
    sub   a2, a3, a0
    bge   a2, s2, 1f; mv a2, s2
1:  add   a3, a3, a0
    ble   a3, s3, 2f; mv a3, s3
2:  bgt   a2, a3, .Lbyl_px_done
    sub   a4, a2, s2
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

.Lbyl_px:
    bgt   t4, t5, .Lbyl_px_done

    blez  t3, .Lbyl_scap
    bge   t3, a6, .Lbyl_ecap

    mul   a0, t2, t2
    mulh  a1, t2, t2
    bnez  a1, .Lbyl_skip
    lw    a1, DL_BR2L(sp)
    bgeu  a1, a0, .Lbyl_in_beam
    lw    a1, DL_INV(sp)
    mulhu a2, a0, a1; mul a0, a0, a1
    slli  a2, a2, 16; srli a0, a0, 16; or a0, a2, a0
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
    sub   a2, a0, a7
    mul   a3, a2, a1; mulh a4, a2, a1
    slli  a4, a4, 2; srli a3, a3, 30; or a0, a4, a3
    li    a1, 178; bgeu a0, a1, .Lbyl_skip
    la    a1, gauss_lut; add a1, a1, a0; lbu a1, 0(a1)
    mul   a0, s6, a1; srai a0, a0, 8
    beqz  a0, .Lbyl_skip

.Lbyl_write:
    /* y_contrib = (255 * contrib) >> 8; additive Y write (white: Y_full=255) */
    li    t0, 255; mul t0, t0, a0; srli t0, t0, 8
    beqz  t0, .Lbyl_y_skip
    lbu   a5, 0(t1); add a5, a5, t0
    li    t0, 255; blt a5, t0, 1f; mv a5, t0
1:  sb    a5, 0(t1)
.Lbyl_y_skip:
    /* UV: delta always zero for white (R=G=B=255) — omitted */

.Lbyl_skip:
    addi  t1, t1, 2
    lw    a1, DL_DY(sp); sub t2, t2, a1
    lw    a1, DL_DX(sp); add t3, t3, a1
.Lbyl_px_incr:
    addi  t4, t4, 1
    ble   t4, t5, .Lbyl_px

.Lbyl_px_done:
    lw    a0, DL_CROW(sp); lw a1, DL_DX(sp); add a0, a0, a1; sw a0, DL_CROW(sp)
    lw    a0, DL_DROW(sp); lw a1, DL_DY(sp); add a0, a0, a1; sw a0, DL_DROW(sp)
.Lbyl_py_next:
    addi  s11, s11, 1
    j     .Lbyl_py

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
    slli  a5, a5, 8; mul a5, a5, a5; add a0, a4, a5
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
    lw    a3, DL_Y0(sp)
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
 *  undraw_line_yuv422_brightness
 *
 *  void undraw_line_yuv422_brightness(
 *    a0=fb, a1=fb_w, a2=fb_h,
 *    a3=x0, a4=y0, a5=x1, a6=y1, a7=brightness)
 *
 *  White line undraw: erases Y to 0 wherever draw_line_yuv422_brightness
 *  would have written. UV never erased (delta always 0 for white).
 *  Algorithmically identical to undraw_line_yuv422 except:
 *    - a7=brightness stored directly in s6; no palette lookup.
 *    - Write section: y_contrib=(255*contrib)>>8; sb zero if nonzero.
 *    - UV section omitted entirely.
 *
 *  Register map:
 *    s0  fb          s6  brightness
 *    s1  fb_w        s7-s9 unused
 *    s2  px_lo       s10 (free)
 *    s3  px_hi       s11 py
 *    s4  by0
 *    s5  by1
 *
 *  a5 = beam_r2_q16 cached throughout distance-field loop.
 *  Stack frame 112 bytes (same layout/offsets as undraw_line_yuv422).
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
    mv    s6, a7                  /* brightness */
    blez  s6, .Lubyl_ret

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

    /* half_w */
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

    la    t0, g_beam_r; lw t0, 0(t0)
    mul   t0, t0, t0
    lw    t1, UL_LEN2(sp); mul t0, t0, t1
    sw    t0, UL_BR2L(sp)

    la    t0, g_gs; lw t0, 0(t0); sw t0, UL_GS(sp)

    lw    a2, UL_LEN2(sp)
    beqz  a2, .Lubyl_setup_done
    li    t0, -1; divu t0, t0, a2; sw t0, UL_INV(sp)
    lw    t0, UL_X0(sp); sub a0, s2, t0
    lw    t0, UL_Y0(sp); sub a1, s4, t0
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

.Lubyl_row:
    bgt   s11, s5, .Lubyl_ret

    mul   t5, s11, s1; slli t5, t5, 1; add t5, s0, t5
    slli  a0, s2, 1; add t1, t5, a0

    lw    a6, UL_LEN2(sp)

    lw    a1, UL_DY(sp)
    beqz  a1, .Lubyl_nonarrow
    lw    a0, UL_HALFW(sp)
    beqz  a0, .Lubyl_nonarrow
    lw    t2, UL_CROW(sp)
    div   a3, t2, a1
    add   a3, a3, s2
    sub   a2, a3, a0
    bge   a2, s2, 1f; mv a2, s2
1:  add   a3, a3, a0
    ble   a3, s3, 2f; mv a3, s3
2:  bgt   a2, a3, .Lubyl_px_done
    sub   a4, a2, s2
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

.Lubyl_px:
    bgt   t4, t5, .Lubyl_px_done

    blez  t3, .Lubyl_scap
    bge   t3, a6, .Lubyl_ecap

    mul   a0, t2, t2
    mulh  a1, t2, t2
    bnez  a1, .Lubyl_pxskip
    lw    a1, UL_BR2L(sp)
    bgeu  a1, a0, .Lubyl_in_beam
    lw    a1, UL_INV(sp)
    mulhu a2, a0, a1; mul a0, a0, a1
    slli  a2, a2, 16; srli a0, a0, 16; or a0, a2, a0
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

.Lubyl_beam:
    bgt   a0, a5, .Lubyl_glow
.Lubyl_in_beam:
    mv    a0, s6
    j     .Lubyl_do_write

.Lubyl_glow:
    lw    a1, UL_GS(sp); beqz a1, .Lubyl_pxskip
    sub   a2, a0, a5
    mul   a3, a2, a1; mulh a4, a2, a1
    slli  a4, a4, 2; srli a3, a3, 30; or a0, a4, a3
    li    a1, 178; bgeu a0, a1, .Lubyl_pxskip
    la    a1, gauss_lut; add a1, a1, a0; lbu a1, 0(a1)
    mul   a0, s6, a1; srai a0, a0, 8
    beqz  a0, .Lubyl_pxskip

.Lubyl_do_write:
    /* y_contrib = (255 * contrib) >> 8; if nonzero: Y=0 (white: Y_full=255) */
    li    t0, 255; mul t0, t0, a0; srli t0, t0, 8
    beqz  t0, .Lubyl_pxskip
    sb    zero, 0(t1)
    /* UV: delta always zero for white — omitted */

.Lubyl_pxskip:
    addi  t1, t1, 2
    lw    a1, UL_DY(sp); sub t2, t2, a1
    lw    a1, UL_DX(sp); add t3, t3, a1
.Lubyl_px_incr:
    addi  t4, t4, 1
    ble   t4, t5, .Lubyl_px

.Lubyl_px_done:
    lw    a0, UL_CROW(sp); lw a1, UL_DX(sp); add a0, a0, a1; sw a0, UL_CROW(sp)
    lw    a0, UL_DROW(sp); lw a1, UL_DY(sp); add a0, a0, a1; sw a0, UL_DROW(sp)
.Lubyl_row_next:
    addi  s11, s11, 1
    j     .Lubyl_row

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
    slli  a1, a3, 8; mul a1, a1, a1; add a0, a0, a1
    bgt   a0, a5, .Lubyl_d_glow
    mv    a0, s6
    j     .Lubyl_d_write
.Lubyl_d_glow:
    lw    a1, UL_GS(sp); beqz a1, .Lubyl_d_px_next
    sub   a2, a0, a5
    mul   a3, a2, a1; mulh a4, a2, a1
    slli  a4, a4, 2; srli a3, a3, 30; or a0, a4, a3
    li    a1, 178; bgeu a0, a1, .Lubyl_d_px_next
    la    a1, gauss_lut; add a1, a1, a0; lbu a1, 0(a1)
    mul   a0, s6, a1; srai a0, a0, 8
    beqz  a0, .Lubyl_d_px_next
.Lubyl_d_write:
    li    t0, 255; mul t0, t0, a0; srli t0, t0, 8
    beqz  t0, .Lubyl_d_px_next
    sb    zero, 0(t1)
    /* UV: always zero for white */
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
