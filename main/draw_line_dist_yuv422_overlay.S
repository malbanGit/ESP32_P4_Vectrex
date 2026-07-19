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
 *   U (even pair)  : row_fb[px*2 + 1]   (when px is even)
 *   V (even pair)  : row_fb[px*2 + 3]   (when px is even)
 * U/V are only written at even-column pixels.
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
 *  Stack frame 96 bytes:
 *    sp+ 0  ra       sp+52  x0      sp+68  s_ov_off_y
 *    sp+ 4  s0       sp+56  y0      sp+72  s_ov_h
 *    sp+ 8  s1       sp+60  x1      sp+76  bx0_orig
 *    sp+12  s2       sp+64  y1      sp+80  bx1_orig
 *    sp+16  s3                      sp+84  span2 (bg-path)
 *    sp+20  s4                      sp+88  (pad)
 *    sp+24  s5                      sp+92  (pad)
 *    sp+28  s6
 *    sp+32  s7       sp+96  overlay   (caller)
 *    sp+36  s8
 *    sp+40  s9
 *    sp+44  s10
 *    sp+48  s11
 * ══════════════════════════════════════════════════════════════════════════ */

#define UF         96
#define UF_RA       0
#define UF_X0      52
#define UF_Y0      56
#define UF_X1      60
#define UF_Y1      64
#define UF_OFFSY   68
#define UF_OVH     72
#define UF_BX0     76
#define UF_BX1     80
#define UF_SPAN2   84
#define UF_OV      (UF+0)

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
    la    s8, s_overlay_palette
    la    t0, s_overlay_alpha_val; lbu s6, 0(t0)
    la    t0, s_ov_off_x; lw s9, 0(t0)
    la    t0, s_ov_w;     lw s10, 0(t0)
    la    t0, s_ov_off_y; lw t1, 0(t0); sw t1, UF_OFFSY(sp)
    la    t0, s_ov_h;     lw t1, 0(t0); sw t1, UF_OVH(sp)

    /* Precompute px range for palette path */
    lw    t0, UF_BX0(sp)
    blt   s9, t0, 1f; mv t0, s9
1:  mv    s2, t0           /* px_lo = max(bx0, s_ov_off_x) */

    lw    t0, UF_BX1(sp)
    addi  t1, s10, -1; add t1, t1, s9
    blt   t0, t1, 1f; mv t0, t1
1:  mv    s3, t0           /* px_hi = min(bx1, s_ov_off_x+s_ov_w-1) */

    /* Dispatch */
    bnez  s7, .Luyd_pal_path
    la    t0, s_overlay_bg; lw t0, 0(t0)
    bnez  t0, .Luyd_bg_path
    lw    t0, UF_OV(sp)
    bnez  t0, .Luyd_ov_path
    j     .Luyd_ret

/* ──────────────────────────────────────────────────────────────────────────
 * PALETTE PATH  (hot path)
 * Write Y and (at even px) U, V from BT.601 conversion of alpha-scaled palette.
 * ────────────────────────────────────────────────────────────────────────── */
.Luyd_pal_path:
    blt   s5, s4, .Luyd_ret
    blt   s3, s2, .Luyd_ret
    mv    s11, s4                   /* py = by0 */

.Luyd_pal_py:
    bgt   s11, s5, .Luyd_ret

    /* ry = py - s_ov_off_y; skip row if outside overlay */
    lw    t0, UF_OFFSY(sp)
    sub   t0, s11, t0
    lw    t1, UF_OVH(sp)
    bgeu  t0, t1, .Luyd_pal_py_next

    /* row_fb = fb + py*fw*2 */
    mul   t5, s11, s1
    slli  t5, t5, 1
    add   t5, s0, t5

    /* pal_ptr = s_overlay_pal + ry*s_ov_w - s_ov_off_x + px_lo */
    mul   t6, t0, s10
    add   t6, s7, t6
    sub   t6, t6, s9
    add   t6, t6, s2

    /* fb_ptr = row_fb + px_lo*2 */
    slli  t0, s2, 1
    add   t1, t5, t0

    mv    t4, s2                    /* px = px_lo */

.Luyd_pal_px:
    bgt   t4, s3, .Luyd_pal_py_next

    lbu   a0, 0(t6)                 /* pidx = *pal_ptr */
    addi  t6, t6, 1

    andi  a1, a0, 0x80
    beqz  a1, .Luyd_pal_skip        /* transparent: skip */

    /* Palette lookup */
    andi  a0, a0, 0x7f
    slli  a1, a0, 1; add a1, a1, a0; add a1, s8, a1   /* &palette[idx] */

    /* Load R, G, B and scale by ea */
    lbu   a2, 2(a1);  mul a2, a2, s6; srli a2, a2, 8   /* r_sc */
    lbu   a3, 1(a1);  mul a3, a3, s6; srli a3, a3, 8   /* g_sc */
    lbu   a4, 0(a1);  mul a4, a4, s6; srli a4, a4, 8   /* b_sc */

    /* Y = (77*r_sc + 150*g_sc + 29*b_sc) >> 8  — always ∈ [0,255], no clamp */
    li    a0, 77;   mul a0, a0, a2
    li    a5, 150;  mul a5, a5, a3;  add a0, a0, a5
    li    a5, 29;   mul a5, a5, a4;  add a0, a0, a5
    srli  a0, a0, 8
    sb    a0, 0(t1)

    /* Even-column check: skip U/V if px is odd */
    andi  a0, t4, 1
    bnez  a0, .Luyd_pal_skip

    /* U = 128 + (128*b_sc - 43*r_sc - 85*g_sc) >> 8 */
    slli  a0, a4, 7
    li    a5, 43;   mul a5, a5, a2;  sub a0, a0, a5
    li    a5, 85;   mul a5, a5, a3;  sub a0, a0, a5
    srai  a0, a0, 8;  addi a0, a0, 128
    bge   a0, zero, 1f;  li a0, 0
1:  li    a5, 255;  blt a0, a5, 2f;  mv a0, a5
2:  sb    a0, 1(t1)

    /* V = 128 + (128*r_sc - 107*g_sc - 21*b_sc) >> 8 */
    slli  a0, a2, 7
    li    a5, 107;  mul a5, a5, a3;  sub a0, a0, a5
    li    a5, 21;   mul a5, a5, a4;  sub a0, a0, a5
    srai  a0, a0, 8;  addi a0, a0, 128
    bge   a0, zero, 1f;  li a0, 0
1:  li    a5, 255;  blt a0, a5, 2f;  mv a0, a5
2:  sb    a0, 3(t1)

.Luyd_pal_skip:
    addi  t1, t1, 2                 /* fb_ptr += 2 */
    addi  t4, t4, 1
    ble   t4, s3, .Luyd_pal_px

.Luyd_pal_py_next:
    addi  s11, s11, 1
    j     .Luyd_pal_py

/* ──────────────────────────────────────────────────────────────────────────
 * BACKGROUND COPY PATH  — memcpy rows from s_overlay_bg (YUV422 buffer)
 * ────────────────────────────────────────────────────────────────────────── */
.Luyd_bg_path:
    la    t0, s_overlay_bg; lw s7, 0(t0)

    mv    s11, s4
.Luyd_bg_py:
    bgt   s11, s5, .Luyd_ret

    /* offset = (py*fw + bx0) * 2 */
    mul   t0, s11, s1
    lw    t1, UF_BX0(sp)
    add   t0, t0, t1
    slli  t0, t0, 1

    add   a0, s0, t0             /* dst = fb + offset */
    add   a1, s7, t0             /* src = bg + offset */
    lw    a2, UF_SPAN2(sp)
    call  memcpy

    addi  s11, s11, 1
    j     .Luyd_bg_py

/* ──────────────────────────────────────────────────────────────────────────
 * BGRA OVERLAY FALLBACK PATH — convert BGRA → YUYV on the fly
 * ────────────────────────────────────────────────────────────────────────── */
.Luyd_ov_path:
    lw    s7, UF_OV(sp)
    lw    s2, UF_BX0(sp)
    lw    s3, UF_BX1(sp)

    mv    s11, s4
.Luyd_ov_py:
    bgt   s11, s5, .Luyd_ret

    /* row_ov = overlay + (py*fw + bx0)*4 */
    mul   t6, s11, s1
    add   t6, t6, s2
    slli  t6, t6, 2
    add   t6, s7, t6

    /* row_fb = fb + py*fw*2; fb_ptr = row_fb + bx0*2 */
    mul   t5, s11, s1
    slli  t5, t5, 1
    add   t5, s0, t5
    slli  t0, s2, 1
    add   t1, t5, t0

    mv    t4, s2                    /* px = bx0 */

.Luyd_ov_px:
    bgt   t4, s3, .Luyd_ov_py_next

    lbu   a3, 3(t6)                 /* alpha = ov[3] */
    beqz  a3, .Luyd_ov_zero
    li    t0, 255
    beq   a3, t0, .Luyd_ov_full

    /* Partial alpha: use ea */
    mv    a3, s6                    /* ea */
    blez  a3, .Luyd_ov_zero
    li    t0, 255
    bge   a3, t0, .Luyd_ov_full

    /* Scale BGR by s6=ea; a3 no longer needed as ea after load overwrites it */
    lbu   a2, 2(t6);  mul a2, a2, s6;  srli a2, a2, 8   /* r_sc */
    lbu   a4, 0(t6);  mul a4, a4, s6;  srli a4, a4, 8   /* b_sc */
    lbu   a3, 1(t6);  mul a3, a3, s6;  srli a3, a3, 8   /* g_sc (last, clobbers a3=ea) */
    j     .Luyd_ov_write_yuv

.Luyd_ov_full:
    lbu   a2, 2(t6)                 /* r */
    lbu   a3, 1(t6)                 /* g */
    lbu   a4, 0(t6)                 /* b */

.Luyd_ov_write_yuv:
    /* Y = (77*r + 150*g + 29*b) >> 8 */
    li    a0, 77;   mul a0, a0, a2
    li    a5, 150;  mul a5, a5, a3;  add a0, a0, a5
    li    a5, 29;   mul a5, a5, a4;  add a0, a0, a5
    srli  a0, a0, 8
    sb    a0, 0(t1)

    /* Even-column check */
    andi  a0, t4, 1
    bnez  a0, .Luyd_ov_skip

    /* U = 128 + (128*b - 43*r - 85*g) >> 8 */
    slli  a0, a4, 7
    li    a5, 43;   mul a5, a5, a2;  sub a0, a0, a5
    li    a5, 85;   mul a5, a5, a3;  sub a0, a0, a5
    srai  a0, a0, 8;  addi a0, a0, 128
    bge   a0, zero, 1f;  li a0, 0
1:  li    a5, 255;  blt a0, a5, 2f;  mv a0, a5
2:  sb    a0, 1(t1)

    /* V = 128 + (128*r - 107*g - 21*b) >> 8 */
    slli  a0, a2, 7
    li    a5, 107;  mul a5, a5, a3;  sub a0, a0, a5
    li    a5, 21;   mul a5, a5, a4;  sub a0, a0, a5
    srai  a0, a0, 8;  addi a0, a0, 128
    bge   a0, zero, 1f;  li a0, 0
1:  li    a5, 255;  blt a0, a5, 2f;  mv a0, a5
2:  sb    a0, 3(t1)
    j     .Luyd_ov_skip

.Luyd_ov_zero:
    sb    zero, 0(t1)               /* Y = 0 */
    andi  a0, t4, 1
    bnez  a0, .Luyd_ov_skip        /* odd: skip U/V */
    li    a0, 128
    sb    a0, 1(t1)                 /* U = 128 */
    sb    a0, 3(t1)                 /* V = 128 */

.Luyd_ov_skip:
    addi  t6, t6, 4
    addi  t1, t1, 2
    addi  t4, t4, 1
    ble   t4, s3, .Luyd_ov_px

.Luyd_ov_py_next:
    addi  s11, s11, 1
    j     .Luyd_ov_py

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

    bnez  s7, .Ldyv_have_src
    lw    t1, DF_OVLAY(sp)
.Ldyv_have_src:

    la    s8, s_overlay_palette
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

    /* beam_r2_q16 = (beam_r<<8)^2 */
    la    t1, g_line_width; lw t1, 0(t1); srai t1, t1, 1
    slli  t1, t1, 8; mul t1, t1, t1; sw t1, DF_BR2(sp)

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
    bnez  s7, .Ldyv_pal_outer
    j     .Ldyv_bgra_outer

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

    /* Load cross/dot and cache len2/beam_r2 in a6/a7 */
    lw    t2, DF_CROW(sp); lw t3, DF_DROW(sp)
    lw    a6, DF_LEN2(sp); lw a7, DF_BR2(sp)

    mv    t4, s2                    /* px = px_lo */

.Ldyv_pal_px:
    bgt   t4, s3, .Ldyv_pal_px_done

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

    /* Palette lookup: a1 = &palette[idx] */
    slli  a1, t0, 1; add a1, a1, t0; add a1, s8, a1

    /* Load R, G, B (unscaled palette colors) */
    lbu   a2, 2(a1)                 /* r */
    lbu   a3, 1(a1)                 /* g */
    lbu   a4, 0(a1)                 /* b */

    /* Y_full = (77*r + 150*g + 29*b) >> 8 */
    li    a5, 77;   mul a5, a5, a2
    li    t0, 150;  mul t0, t0, a3;  add a5, a5, t0
    li    t0, 29;   mul t0, t0, a4;  add a5, a5, t0
    srli  a5, a5, 8                 /* a5 = Y_full */

    /* y_contrib = Y_full * contrib >> 8 */
    mul   a5, a5, a0
    srli  a5, a5, 8

    /* dst[0] += y_contrib  (additive, saturating at 255) */
    lbu   t0, 0(t1)
    add   a5, a5, t0
    li    t0, 255;  blt a5, t0, 1f;  mv a5, t0
1:  sb    a5, 0(t1)

    /* Even-column check: skip U/V if px is odd */
    andi  t0, t4, 1
    bnez  t0, .Ldyv_pal_skip

    /* U additive delta = (128*b - 43*r - 85*g) * contrib >> 16
     * Mirrors RGB: dst[i] += c[i]*contrib/256, so U-128 accumulates the same way.
     * a2=r, a3=g, a4=b (palette, unscaled), a0=contrib (still valid) */
    slli  t0, a4, 7
    li    a5, 43;   mul a5, a5, a2;  sub t0, t0, a5
    li    a5, 85;   mul a5, a5, a3;  sub t0, t0, a5
    mul   t0, t0, a0;  srai t0, t0, 16   /* signed u_delta */
    lbu   a5, 1(t1);   add a5, a5, t0
    bge   a5, zero, 1f;  li a5, 0
1:  li    t0, 255;  blt a5, t0, 2f;  mv a5, t0
2:  sb    a5, 1(t1)

    /* V additive delta = (128*r - 107*g - 21*b) * contrib >> 16 */
    slli  t0, a2, 7
    li    a5, 107;  mul a5, a5, a3;  sub t0, t0, a5
    li    a5, 21;   mul a5, a5, a4;  sub t0, t0, a5
    mul   t0, t0, a0;  srai t0, t0, 16   /* signed v_delta */
    lbu   a5, 3(t1);   add a5, a5, t0
    bge   a5, zero, 1f;  li a5, 0
1:  li    t0, 255;  blt a5, t0, 2f;  mv a5, t0
2:  sb    a5, 3(t1)

.Ldyv_pal_skip:
    addi  t1, t1, 2                 /* fb_ptr += 2 */
    beqz  a6, .Ldyv_pal_px_incr
    lw    a1, DF_DY(sp); sub t2, t2, a1
    lw    a1, DF_DX(sp); add t3, t3, a1
.Ldyv_pal_px_incr:
    addi  t4, t4, 1
    ble   t4, s3, .Ldyv_pal_px

.Ldyv_pal_px_done:
    lw    a0, DF_LEN2(sp)
    beqz  a0, .Ldyv_pal_py_next
    lw    a0, DF_CROW(sp); lw a1, DF_DX(sp); add a0, a0, a1; sw a0, DF_CROW(sp)
    lw    a0, DF_DROW(sp); lw a1, DF_DY(sp); add a0, a0, a1; sw a0, DF_DROW(sp)
.Ldyv_pal_py_next:
    addi  s11, s11, 1
    j     .Ldyv_pal_py

/* ──────────────────────────────────────────────────────────────────────────
 * BGRA OVERLAY OUTER LOOP
 * Draw: Y additive, U/V SET at even columns from blend-scaled BGR.
 * ────────────────────────────────────────────────────────────────────────── */
.Ldyv_bgra_outer:
    lw    s2, DF_BX0(sp)
    lw    s3, DF_BX1(sp)

    /* Recompute cross_row/dot_row starting from bx0 */
    lw    a0, DF_LEN2(sp)
    beqz  a0, .Ldyv_bgra_outer2
    lw    t0, DF_X0(sp); sub a0, s2, t0
    lw    t0, DF_Y0(sp); sub a1, s4, t0
    lw    t0, DF_DX(sp); lw  t1, DF_DY(sp)
    mul   a2, t0, a1; mul a3, t1, a0; sub a2, a2, a3; sw a2, DF_CROW(sp)
    mul   a2, a0, t0; mul a3, a1, t1; add a2, a2, a3; sw a2, DF_DROW(sp)
.Ldyv_bgra_outer2:
    lw    s7, DF_OVLAY(sp)
    mv    s11, s4

.Ldyv_bgra_py:
    bgt   s11, s5, .Ldyv_ret

    /* row_fb = fb + py*fw*2 */
    mul   t5, s11, s1; slli t5, t5, 1; add t5, s0, t5

    /* row_ov = overlay + (py*fw + bx0)*4 */
    mul   t6, s11, s1; add t6, t6, s2; slli t6, t6, 2; add t6, s7, t6

    /* fb_ptr = row_fb + bx0*2 */
    slli  a0, s2, 1; add t1, t5, a0

    lw    t2, DF_CROW(sp); lw t3, DF_DROW(sp)
    lw    a6, DF_LEN2(sp); lw a7, DF_BR2(sp)
    mv    t4, s2

.Ldyv_bgra_px:
    bgt   t4, s3, .Ldyv_bgra_px_done

    /* Distance */
    beqz  a6, .Ldyv_bgra_point
    blez  t3, .Ldyv_bgra_scap
    bge   t3, a6, .Ldyv_bgra_ecap

    mul   a0, t2, t2
    lw    a1, DF_INV(sp)
    mulhu a2, a0, a1; mul a0, a0, a1
    slli  a2, a2, 16; srli a0, a0, 16; or a0, a2, a0
    j     .Ldyv_bgra_beam

.Ldyv_bgra_scap:
    lw    a0, DF_X0(sp); lw a1, DF_Y0(sp)
    sub   a2, t4, a0; sub a3, s11, a1
    slli  a0, a2, 8; mul a0, a0, a0
    slli  a1, a3, 8; mul a1, a1, a1; add a0, a0, a1
    j     .Ldyv_bgra_beam

.Ldyv_bgra_ecap:
    lw    a0, DF_X1(sp); lw a1, DF_Y1(sp)
    sub   a2, t4, a0; sub a3, s11, a1
    slli  a0, a2, 8; mul a0, a0, a0
    slli  a1, a3, 8; mul a1, a1, a1; add a0, a0, a1
    j     .Ldyv_bgra_beam

.Ldyv_bgra_point:
    lw    a0, DF_X0(sp); lw a1, DF_Y0(sp)
    sub   a2, t4, a0; sub a3, s11, a1
    slli  a0, a2, 8; mul a0, a0, a0
    slli  a1, a3, 8; mul a1, a1, a1; add a0, a0, a1

.Ldyv_bgra_beam:
    bgt   a0, a7, .Ldyv_bgra_glow
    mv    a0, s6; j .Ldyv_bgra_write

.Ldyv_bgra_glow:
    lw    a1, DF_GS(sp); beqz a1, .Ldyv_bgra_skip
    sub   a2, a0, a7
    mul   a3, a2, a1; mulh a4, a2, a1
    slli  a4, a4, 2; srli a3, a3, 30; or a0, a4, a3
    li    a1, 178; bgeu a0, a1, .Ldyv_bgra_skip
    la    a1, gauss_lut; add a1, a1, a0; lbu a1, 0(a1)
    mul   a0, s6, a1; srai a0, a0, 8
    beqz  a0, .Ldyv_bgra_skip

.Ldyv_bgra_write:
    /* a0=contrib, t6=ov_ptr, t1=fb_ptr, t4=px */

    /* Load R, G, B from overlay */
    lbu   a2, 2(t6)                 /* r */
    lbu   a3, 1(t6)                 /* g */
    lbu   a4, 0(t6)                 /* b */

    /* Y_full = (77*r + 150*g + 29*b) >> 8 */
    li    a5, 77;   mul a5, a5, a2
    li    t0, 150;  mul t0, t0, a3;  add a5, a5, t0
    li    t0, 29;   mul t0, t0, a4;  add a5, a5, t0
    srli  a5, a5, 8

    /* y_contrib = Y_full * contrib >> 8 */
    mul   a5, a5, a0; srli a5, a5, 8

    /* dst[0] += y_contrib (additive, saturating) */
    lbu   t0, 0(t1)
    add   a5, a5, t0
    li    t0, 255; blt a5, t0, 1f; mv a5, t0
1:  sb    a5, 0(t1)

    /* Even-column check */
    andi  t0, t4, 1
    bnez  t0, .Ldyv_bgra_skip

    /* U additive delta = (128*b - 43*r - 85*g) * contrib >> 16 */
    slli  t0, a4, 7
    li    a5, 43;   mul a5, a5, a2;  sub t0, t0, a5
    li    a5, 85;   mul a5, a5, a3;  sub t0, t0, a5
    mul   t0, t0, a0;  srai t0, t0, 16
    lbu   a5, 1(t1);   add a5, a5, t0
    bge   a5, zero, 1f;  li a5, 0
1:  li    t0, 255;  blt a5, t0, 2f;  mv a5, t0
2:  sb    a5, 1(t1)

    /* V additive delta = (128*r - 107*g - 21*b) * contrib >> 16 */
    slli  t0, a2, 7
    li    a5, 107;  mul a5, a5, a3;  sub t0, t0, a5
    li    a5, 21;   mul a5, a5, a4;  sub t0, t0, a5
    mul   t0, t0, a0;  srai t0, t0, 16
    lbu   a5, 3(t1);   add a5, a5, t0
    bge   a5, zero, 1f;  li a5, 0
1:  li    t0, 255;  blt a5, t0, 2f;  mv a5, t0
2:  sb    a5, 3(t1)

.Ldyv_bgra_skip:
    addi  t6, t6, 4
    addi  t1, t1, 2                 /* fb_ptr += 2 */
    beqz  a6, .Ldyv_bgra_px_incr
    lw    a1, DF_DY(sp); sub t2, t2, a1
    lw    a1, DF_DX(sp); add t3, t3, a1
.Ldyv_bgra_px_incr:
    addi  t4, t4, 1
    ble   t4, s3, .Ldyv_bgra_px

.Ldyv_bgra_px_done:
    lw    a0, DF_LEN2(sp)
    beqz  a0, .Ldyv_bgra_py_next
    lw    a0, DF_CROW(sp); lw a1, DF_DX(sp); add a0, a0, a1; sw a0, DF_CROW(sp)
    lw    a0, DF_DROW(sp); lw a1, DF_DY(sp); add a0, a0, a1; sw a0, DF_DROW(sp)
.Ldyv_bgra_py_next:
    addi  s11, s11, 1
    j     .Ldyv_bgra_py

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
