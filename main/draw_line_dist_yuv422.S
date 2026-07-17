/* draw_line_asm_yuv422 — RV32IMAC, incremental cross-product distance field.
 *
 * Same algorithm as draw_line_asm_rgb888 (draw_line_dist_rgb888.S) with
 * the following YUV422 (YUYV) adaptations:
 *   - Pixel stride = 2 bytes  (Y at px*2+0, UV at px*2+1)
 *   - Only the Y byte is written; the UV byte is always left untouched
 *   - Saturation spill offsets are ±2 bytes (not ±3), Y byte only
 *   - Row base: fb + py*fb_w*2  (slli 1, not x+x*2)
 *   - undraw zeros Y bytes one at a time (cannot memset whole rows)
 *
 * gauss_lut_yuv422 is defined locally in this file (.rodata).
 * g_line_glow is defined in draw_line_dist_rgb888.c.
 *
 * Register map (draw function):
 *   s0  = fb              s6  = dx
 *   s1  = fb_w            s7  = dy
 *   s2  = bx0             s8  = len2
 *   s3  = bx1             s9  = brightness
 *   s4  = by0             s10 = beam_r2_q16
 *   s5  = by1             s11 = gs  (gauss_scale)
 *   t3  = cross  (incremental)
 *   t0  = dot    (incremental)
 *   t4  = px
 *   t5  = row  (fb + py*fb_w*2)
 *   t6  = py
 * 
 * Stack frame (80 bytes):
 *   sp+ 0  : saved ra
 *   sp+ 4  : saved s0..s11  (48 bytes, sp+4..sp+51)
 *   sp+52  : x0
 *   sp+56  : y0
 *   sp+60  : x1
 *   sp+64  : y1
 *   sp+68  : cross_row
 *   sp+72  : dot_row
 *   sp+76  : inv_len2
 *   sp+80  : thickness  (9th arg from caller)
 */

    /* ── Local Gaussian LUT ─────────────────────────────────────────── */
    .section .rodata
    .align  2
gauss_lut_yuv422:
    .byte 255, 247, 240, 232, 225, 218, 211, 205, 199, 192, 187, 181, 175, 170, 165, 160
    .byte 155, 150, 145, 141, 136, 132, 128, 124, 120, 117, 113, 110, 106, 103, 100,  97
    .byte  94,  91,  88,  85,  83,  80,  78,  75,  73,  71,  69,  67,  64,  62,  61,  59
    .byte  57,  55,  53,  52,  50,  49,  47,  46,  44,  43,  42,  40,  39,  38,  37,  36
    .byte  35,  33,  32,  31,  30,  30,  29,  28,  27,  26,  25,  24,  24,  23,  22,  22
    .byte  21,  20,  20,  19,  18,  18,  17,  17,  16,  16,  15,  15,  14,  14,  14,  13
    .byte  13,  12,  12,  12,  11,  11,  11,  10,  10,  10,   9,   9,   9,   8,   8,   8
    .byte   8,   7,   7,   7,   7,   7,   6,   6,   6,   6,   6,   5,   5,   5,   5,   5
    .byte   5,   5,   4,   4,   4,   4,   4,   4,   4,   4,   3,   3,   3,   3,   3,   3
    .byte   3,   3,   3,   3,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2
    .byte   2,   2,   2,   2,   2,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1
    .byte   1,   1

    .extern g_line_glow

    .section .iram0.text,"ax",@progbits
    .global draw_line_asm_yuv422
    .type   draw_line_asm_yuv422, @function
    .align  2

draw_line_asm_yuv422:
    addi    sp, sp, -80
    sw      ra,   0(sp)
    sw      s0,   4(sp)
    sw      s1,   8(sp)
    sw      s2,  12(sp)
    sw      s3,  16(sp)
    sw      s4,  20(sp)
    sw      s5,  24(sp)
    sw      s6,  28(sp)
    sw      s7,  32(sp)
    sw      s8,  36(sp)
    sw      s9,  40(sp)
    sw      s10, 44(sp)
    sw      s11, 48(sp)

    mv      s0, a0          /* fb         */
    mv      s1, a1          /* fb_w       */
    mv      s9, a7          /* brightness */

    beqz    s0, .LYdraw_ret
    blez    s1, .LYdraw_ret
    blez    a2, .LYdraw_ret
    blez    s9, .LYdraw_ret

    sw      a3, 52(sp)      /* x0 */
    sw      a4, 56(sp)      /* y0 */
    sw      a5, 60(sp)      /* x1 */
    sw      a6, 64(sp)      /* y1 */

    /* glow = max(g_line_glow, 0) */
    la      t0, g_line_glow
    lw      t0, 0(t0)
    bgez    t0, .LYglow_ok
    li      t0, 0
.LYglow_ok:

    /* beam_r = thickness>>1,  R = beam_r+glow */
    lw      t1, 80(sp)
    srai    t1, t1, 1               /* t1 = beam_r */
    add     t2, t1, t0              /* t2 = R      */

    /* ── Bounding box ── */
    /* bx0 = iclamp(min(x0,x1)-R, 0, fb_w-1) */
    lw      a3, 52(sp)
    lw      a5, 60(sp)
    blt     a3, a5, 1f
    mv      a3, a5
1:  sub     a3, a3, t2
    bge     a3, zero, 1f
    li      a3, 0
1:  addi    t3, s1, -1              /* fb_w-1 */
    blt     a3, t3, 1f
    mv      a3, t3
1:  mv      s2, a3                  /* bx0 */

    /* bx1 = iclamp(max(x0,x1)+R, 0, fb_w-1) */
    lw      a3, 52(sp)
    lw      a5, 60(sp)
    bgt     a3, a5, 1f
    mv      a3, a5
1:  add     a3, a3, t2
    bge     a3, zero, 1f
    li      a3, 0
1:  blt     a3, t3, 1f
    mv      a3, t3
1:  mv      s3, a3                  /* bx1 */

    /* by0 = iclamp(min(y0,y1)-R, 0, fb_h-1) */
    lw      a4, 56(sp)
    lw      a6, 64(sp)
    blt     a4, a6, 1f
    mv      a4, a6
1:  sub     a4, a4, t2
    bge     a4, zero, 1f
    li      a4, 0
1:  addi    t3, a2, -1              /* fb_h-1 */
    blt     a4, t3, 1f
    mv      a4, t3
1:  mv      s4, a4                  /* by0 */

    /* by1 = iclamp(max(y0,y1)+R, 0, fb_h-1) */
    lw      a4, 56(sp)
    lw      a6, 64(sp)
    bgt     a4, a6, 1f
    mv      a4, a6
1:  add     a4, a4, t2
    bge     a4, zero, 1f
    li      a4, 0
1:  blt     a4, t3, 1f
    mv      a4, t3
1:  mv      s5, a4                  /* by1 */

    /* ── dx, dy, len2 ── */
    lw      a0, 52(sp)
    lw      a1, 60(sp)
    sub     s6, a1, a0              /* dx */
    lw      a0, 56(sp)
    lw      a1, 64(sp)
    sub     s7, a1, a0              /* dy */
    mul     a0, s6, s6
    mul     a1, s7, s7
    add     s8, a0, a1              /* len2 */

    /* ── beam_r2_q16 = (beam_r<<8)^2 ── */
    slli    a0, t1, 8
    mul     s10, a0, a0

    /* ── gs = gauss_scale(glow) ── */
    beqz    t0, .LYgs_zero
    mul     a0, t0, t0
    li      a1, 2899968             /* 177<<14 */
    div     s11, a1, a0
    j       .LYgs_done
.LYgs_zero:
    li      s11, 0
.LYgs_done:

    /* ── Incremental state (len2 > 0 only) ── */
    beqz    s8, .LYsetup_done

    /* inv_len2 ≈ 2^32/len2  via (2^32-1)/len2 */
    li      a0, -1
    divu    a0, a0, s8
    sw      a0, 76(sp)

    /* apx0 = bx0-x0,  apy0 = by0-y0 */
    lw      a1, 52(sp)              /* x0 */
    sub     a0, s2, a1              /* a0 = apx0 */
    lw      a1, 56(sp)              /* y0 */
    sub     a1, s4, a1              /* a1 = apy0 */

    /* cross_row = dx*apy0 - dy*apx0 */
    mul     a2, s6, a1
    mul     a3, s7, a0
    sub     a2, a2, a3
    sw      a2, 68(sp)

    /* dot_row = apx0*dx + apy0*dy */
    mul     a2, a0, s6
    mul     a3, a1, s7
    add     a2, a2, a3
    sw      a2, 72(sp)

.LYsetup_done:

    /* ════════════════════════════════════
     * Outer loop: py = by0..by1
     * ════════════════════════════════════ */
    mv      t6, s4
.LYpy_loop:
    bgt     t6, s5, .LYdraw_ret

    /* row = fb + py*fb_w*2 */
    mul     t5, t6, s1
    slli    t5, t5, 1
    add     t5, s0, t5

    /* Reload incremental cross/dot for this row */
    lw      t3, 68(sp)              /* cross = cross_row */
    lw      t0, 72(sp)              /* dot   = dot_row   */

    /* ────────────────────────────────────
     * Inner loop: px = bx0..bx1
     * ──────────────────────────────────── */
    mv      t4, s2
.LYpx_loop:
    bgt     t4, s3, .LYpx_done

    /* ── Distance → a0=d2_q16 ── */
    beqz    s8, .LYpoint_dist

    blez    t0, .LYstart_cap         /* dot <= 0 → start cap */
    bge     t0, s8, .LYend_cap       /* dot >= len2 → end cap */

    /* Interior: d2_q16 = (cross^2 * inv_len2) >> 16 */
    mul     a4, t3, t3              /* cross^2 */
    lw      a5, 76(sp)              /* inv_len2 */
    mulhu   a6, a4, a5              /* high 32 of product */
    mul     a5, a4, a5              /* low  32 of product */
    slli    a6, a6, 16
    srli    a5, a5, 16
    or      a0, a6, a5              /* a0 = d2_q16 */
    j       .LYbeam_test

.LYstart_cap:
    lw      a0, 52(sp)              /* x0 */
    lw      a1, 56(sp)              /* y0 */
    sub     a2, t4, a0              /* apx = px-x0 */
    sub     a3, t6, a1              /* apy = py-y0 */
    slli    a0, a2, 8
    slli    a1, a3, 8
    mul     a0, a0, a0
    mul     a1, a1, a1
    add     a0, a0, a1
    j       .LYbeam_test

.LYend_cap:
    lw      a0, 60(sp)              /* x1 */
    lw      a1, 64(sp)              /* y1 */
    sub     a2, t4, a0
    sub     a3, t6, a1
    slli    a0, a2, 8
    slli    a1, a3, 8
    mul     a0, a0, a0
    mul     a1, a1, a1
    add     a0, a0, a1
    j       .LYbeam_test

.LYpoint_dist:                       /* len2 == 0 */
    lw      a0, 52(sp)
    lw      a1, 56(sp)
    sub     a2, t4, a0
    sub     a3, t6, a1
    slli    a0, a2, 8
    slli    a1, a3, 8
    mul     a0, a0, a0
    mul     a1, a1, a1
    add     a0, a0, a1

.LYbeam_test:
    /* a0 = d2_q16 */
    bgt     a0, s10, .LYglow_path
    mv      a0, s9                  /* contrib = brightness */
    j       .LYpixel_write

.LYglow_path:
    beqz    s11, .LYpx_incr         /* glow disabled */

    sub     a1, a0, s10             /* glow_d2_q16 */
    mul     a2, a1, s11
    mulh    a3, a1, s11
    slli    a3, a3, 2
    srli    a2, a2, 30
    or      a0, a3, a2              /* lut_idx */

    li      t1, 178
    bgeu    a0, t1, .LYpx_incr     /* past end of LUT */

    la      t1, gauss_lut_yuv422
    add     t1, t1, a0
    lbu     a1, 0(t1)               /* gauss_lut_yuv422[lut_idx] */
    mul     a0, s9, a1
    srai    a0, a0, 8               /* contrib = brightness*lut >> 8 */
    beqz    a0, .LYpx_incr

.LYpixel_write:
    /* t2 = &row[px*2]  — Y byte; UV at t2+1 untouched */
    slli    t2, t4, 1
    add     t2, t5, t2

    lbu     t1, 0(t2)               /* existing Y byte */
    add     t1, t1, a0              /* v = Y + contrib */

    li      a5, 255
    ble     t1, a5, .LYwrite_simple

    /* ── Saturation spill (Y byte only; ±2 byte offsets) ── */
    sub     a2, t1, a5              /* v - 255 */
    addi    a2, a2, 3
    srai    a2, a2, 2               /* spill = (v-255+3)>>2 */
    blez    a2, .LYno_spill

    /* Left: px > bx0 */
    ble     t4, s2, .LYno_left
    addi    a0, t2, -2
    lbu     a1, 0(a0)
    add     a1, a1, a2
    blt     a1, a5, 1f
    mv      a1, a5
1:  sb      a1, 0(a0)
.LYno_left:

    /* Right: px < bx1 */
    bge     t4, s3, .LYno_right
    addi    a0, t2, 2
    lbu     a1, 0(a0)
    add     a1, a1, a2
    blt     a1, a5, 1f
    mv      a1, a5
1:  sb      a1, 0(a0)
.LYno_right:

    /* Top: py > by0  →  row - fb_w*2 */
    ble     t6, s4, .LYno_top
    slli    a1, s1, 1               /* fb_w*2 */
    sub     a0, t5, a1              /* prev_row */
    sub     a1, t2, t5              /* px*2 offset */
    add     a0, a0, a1
    lbu     a1, 0(a0)
    add     a1, a1, a2
    blt     a1, a5, 1f
    mv      a1, a5
1:  sb      a1, 0(a0)
.LYno_top:

    /* Bottom: py < by1  →  row + fb_w*2 */
    bge     t6, s5, .LYno_bottom
    slli    a1, s1, 1
    add     a0, t5, a1              /* next_row */
    sub     a1, t2, t5              /* px*2 offset */
    add     a0, a0, a1
    lbu     a1, 0(a0)
    add     a1, a1, a2
    blt     a1, a5, 1f
    mv      a1, a5
1:  sb      a1, 0(a0)
.LYno_bottom:

.LYno_spill:
    mv      t1, a5                  /* v = 255 */

.LYwrite_simple:
    sb      t1, 0(t2)               /* Y only */

    /* ── Increment cross/dot for next px ── */
.LYpx_incr:
    beqz    s8, .LYpx_next
    sub     t3, t3, s7              /* cross -= dy */
    add     t0, t0, s6              /* dot   += dx */
.LYpx_next:
    addi    t4, t4, 1
    j       .LYpx_loop

.LYpx_done:
    /* Advance row-start cross/dot for next py */
    beqz    s8, .LYpy_next
    lw      a0, 68(sp)
    add     a0, a0, s6              /* cross_row += dx */
    sw      a0, 68(sp)
    lw      a0, 72(sp)
    add     a0, a0, s7              /* dot_row   += dy */
    sw      a0, 72(sp)
.LYpy_next:
    addi    t6, t6, 1
    j       .LYpy_loop

.LYdraw_ret:
    lw      ra,   0(sp)
    lw      s0,   4(sp)
    lw      s1,   8(sp)
    lw      s2,  12(sp)
    lw      s3,  16(sp)
    lw      s4,  20(sp)
    lw      s5,  24(sp)
    lw      s6,  28(sp)
    lw      s7,  32(sp)
    lw      s8,  36(sp)
    lw      s9,  40(sp)
    lw      s10, 44(sp)
    lw      s11, 48(sp)
    addi    sp, sp, 80
    ret

    .size draw_line_asm_yuv422, .-draw_line_asm_yuv422

/* ════════════════════════════════════════════════════════════════════
 * undraw_line_asm_yuv422 — zero Y bytes in the bounding box.
 *
 * For each pixel (px, py) in the bounding box, writes 0 to the Y byte
 * at fb[(py*fb_w + px)*2].  The UV byte at +1 is never touched.
 *
 * Register map:
 *   s0  = fb
 *   s1  = fb_w
 *   s2  = bx0
 *   s3  = bx1
 *   s4  = by0
 *   s5  = by1
 *
 * Stack frame (48 bytes):
 *   sp+ 0  : saved ra
 *   sp+ 4  : saved s0..s5  (24 bytes)
 *   sp+28  : x0
 *   sp+32  : y0
 *   sp+36  : x1
 *   sp+40  : y1
 *   sp+44  : (pad)
 *   sp+48  : thickness  (9th arg from caller)
 * ════════════════════════════════════════════════════════════════════ */

    .global undraw_line_asm_yuv422
    .type   undraw_line_asm_yuv422, @function
    .align  2

undraw_line_asm_yuv422:
    addi    sp, sp, -48
    sw      ra,  0(sp)
    sw      s0,  4(sp)
    sw      s1,  8(sp)
    sw      s2, 12(sp)
    sw      s3, 16(sp)
    sw      s4, 20(sp)
    sw      s5, 24(sp)

    /* Save x0,y0,x1,y1 before anything clobbers a3..a6 */
    sw      a3, 28(sp)
    sw      a4, 32(sp)
    sw      a5, 36(sp)
    sw      a6, 40(sp)

    mv      s0, a0          /* fb   */
    mv      s1, a1          /* fb_w */

    beqz    s0, .LYundraw_ret
    blez    s1, .LYundraw_ret
    blez    a2, .LYundraw_ret

    la      t0, g_line_glow
    lw      t0, 0(t0)
    bgez    t0, 1f
    li      t0, 0
1:

    lw      t1, 48(sp)
    srai    t1, t1, 1               /* beam_r */
    add     t2, t1, t0              /* R */

    /* ── Bounding box ── */
    lw      a3, 28(sp)
    lw      a5, 36(sp)
    blt     a3, a5, 1f
    mv      a3, a5
1:  sub     a3, a3, t2
    bge     a3, zero, 1f
    li      a3, 0
1:  addi    t3, s1, -1
    blt     a3, t3, 1f
    mv      a3, t3
1:  mv      s2, a3                  /* bx0 */

    lw      a3, 28(sp)
    lw      a5, 36(sp)
    bgt     a3, a5, 1f
    mv      a3, a5
1:  add     a3, a3, t2
    bge     a3, zero, 1f
    li      a3, 0
1:  blt     a3, t3, 1f
    mv      a3, t3
1:  mv      s3, a3                  /* bx1 */

    lw      a4, 32(sp)
    lw      a6, 40(sp)
    blt     a4, a6, 1f
    mv      a4, a6
1:  sub     a4, a4, t2
    bge     a4, zero, 1f
    li      a4, 0
1:  addi    t3, a2, -1
    blt     a4, t3, 1f
    mv      a4, t3
1:  mv      s4, a4                  /* by0 */

    lw      a4, 32(sp)
    lw      a6, 40(sp)
    bgt     a4, a6, 1f
    mv      a4, a6
1:  add     a4, a4, t2
    bge     a4, zero, 1f
    li      a4, 0
1:  blt     a4, t3, 1f
    mv      a4, t3
1:  mv      s5, a4                  /* by1 */

    /* ── Zero Y bytes ── */
    mv      t6, s4                  /* py */
.LYundraw_py:
    bgt     t6, s5, .LYundraw_ret

    /* row_base = fb + py*fb_w*2 */
    mul     t5, t6, s1
    slli    t5, t5, 1
    add     t5, s0, t5

    mv      t4, s2                  /* px */
.LYundraw_px:
    bgt     t4, s3, .LYundraw_py_next

    slli    a0, t4, 1
    add     a0, t5, a0
    sb      zero, 0(a0)             /* Y = 0; UV at +1 untouched */

    addi    t4, t4, 1
    j       .LYundraw_px

.LYundraw_py_next:
    addi    t6, t6, 1
    j       .LYundraw_py

.LYundraw_ret:
    lw      ra,  0(sp)
    lw      s0,  4(sp)
    lw      s1,  8(sp)
    lw      s2, 12(sp)
    lw      s3, 16(sp)
    lw      s4, 20(sp)
    lw      s5, 24(sp)
    addi    sp, sp, 48
    ret

    .size undraw_line_asm_yuv422, .-undraw_line_asm_yuv422
