/* draw_line_asm_rgb888 — RV32IMAC, incremental cross-product distance.
 *
 * g_line_glow and gauss_lut are defined in draw_line_dist_rgb888.c.
 *
 * Key optimisation: instead of __divdi3 per interior pixel, precompute
 *   inv_len2 = (2^32-1) / len2   (one DIVU per draw call)
 * and maintain cross = dx*(py-y0) - dy*(px-x0) incrementally:
 *   per px step: cross -= dy
 *   per py step: cross_row += dx
 * Then d2_q16 = (cross^2 * inv_len2) >> 16 via MULHU — no function call.
 * dot is also tracked incrementally to avoid per-pixel MUL for cap detection.
 *
 * Register map:
 *   s0  = fb              s6  = dx
 *   s1  = fb_w            s7  = dy
 *   s2  = bx0             s8  = len2
 *   s3  = bx1             s9  = brightness
 *   s4  = by0             s10 = beam_r2_q16
 *   s5  = by1             s11 = gs  (gauss_scale)
 *   t3  = cross  (inner-loop, incremental)
 *   t0  = dot    (inner-loop, incremental)
 *   t4  = px     (inner-loop counter)
 *   t5  = row    (fb + py*fb_w*3)
 *   t6  = py     (outer-loop counter)
 *
 * Stack frame (80 bytes, 16-byte aligned):
 *   sp+ 0  : saved ra
 *   sp+ 4  : saved s0..s11  (48 bytes)
 *   sp+52  : x0  local
 *   sp+56  : y0  local
 *   sp+60  : x1  local
 *   sp+64  : y1  local
 *   sp+68  : cross_row  — cross at (bx0, py), updated each outer iteration
 *   sp+72  : dot_row    — dot   at (bx0, py), updated each outer iteration
 *   sp+76  : inv_len2   — (2^32-1)/len2, or undefined when len2==0
 *
 * Constraint: |cross| < 65536 required for cross^2 to fit in uint32.
 * Satisfied when R*sqrt(len2) < 65536, i.e. for R<=16 on 1920-wide screens.
 */

    .extern g_line_glow
    .extern g_line_width
    .extern g_line_Rb

    .section .iram0.text,"ax",@progbits
    .global draw_line_asm_rgb888
    .type   draw_line_asm_rgb888, @function
    .align  2

draw_line_asm_rgb888:
    /* ── Prologue ── */
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

    /* Early exits */
//    beqz    s0, .Ldraw_ret
//    blez    s1, .Ldraw_ret
//    blez    a2, .Ldraw_ret          /* fb_h */
//    blez    s9, .Ldraw_ret

    /* Save coordinate args */
    sw      a3, 52(sp)      /* x0 */
    sw      a4, 56(sp)      /* y0 */
    sw      a5, 60(sp)      /* x1 */
    sw      a6, 64(sp)      /* y1 */

    /* glow = max(g_line_glow, 0) — still needed for gauss scale */
    la      t0, g_line_glow
    lw      t0, 0(t0)
    bgez    t0, .Lglow_ok
    li      t0, 0
.Lglow_ok:

    /* beam_r = g_line_width >> 1 (for beam_r2_q16);  Rb = g_line_Rb (bbox) */
    la      t1, g_line_width
    lw      t1, 0(t1)
    srai    t1, t1, 1               /* t1 = beam_r */
    la      t2, g_line_Rb
    lw      t2, 0(t2)               /* t2 = Rb     */

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
    beqz    t0, .Lgs_zero
    mul     a0, t0, t0
    li      a1, 2899968             /* 177<<14 */
    div     s11, a1, a0
    j       .Lgs_done
.Lgs_zero:
    li      s11, 0
.Lgs_done:

    /* ── Incremental state (len2 > 0 only) ── */
    beqz    s8, .Lsetup_done

    /* inv_len2 ≈ 2^32/len2  via (2^32-1)/len2 (one DIVU, error < 1 LSB) */
    li      a0, -1
    divu    a0, a0, s8
    sw      a0, 76(sp)

    /* apx0 = bx0-x0,  apy0 = by0-y0  (offsets for first pixel of first row) */
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

.Lsetup_done:

    /* ════════════════════════════════════
     * Outer loop: py = by0..by1
     * ════════════════════════════════════ */
    mv      t6, s4
.Lpy_loop:
    bgt     t6, s5, .Ldraw_ret

    /* row = fb + py*fb_w*3  (x*3 = x + x<<1) */
    mul     t5, t6, s1
    slli    a0, t5, 1
    add     t5, t5, a0
    add     t5, s0, t5

    /* Reload incremental cross/dot for this row */
    lw      t3, 68(sp)              /* cross = cross_row */
    lw      t0, 72(sp)              /* dot   = dot_row   */

    /* ────────────────────────────────────
     * Inner loop: px = bx0..bx1
     * ──────────────────────────────────── */
    mv      t4, s2
.Lpx_loop:
    bgt     t4, s3, .Lpx_done

    /* ── Distance → a0=d2_q16 ── */
    beqz    s8, .Lpoint_dist

    blez    t0, .Lstart_cap         /* dot <= 0 → start cap */
    bge     t0, s8, .Lend_cap       /* dot >= len2 → end cap */

    /* Interior: d2_q16 = (cross^2 * inv_len2) >> 16
     * mulhu gives high 32 bits of the 64-bit product:
     *   (hi:lo) >> 16  →  (hi<<16) | (lo>>16) */
    mul     a4, t3, t3              /* cross^2 (fits uint32 for |cross|<65536) */
    lw      a5, 76(sp)              /* inv_len2 */
    mulhu   a6, a4, a5              /* high 32 of product */
    mul     a5, a4, a5              /* low  32 of product */
    slli    a6, a6, 16
    srli    a5, a5, 16
    or      a0, a6, a5              /* a0 = d2_q16 */
    j       .Lbeam_test

.Lstart_cap:
    lw      a0, 52(sp)              /* x0 */
    lw      a1, 56(sp)              /* y0 */
    sub     a2, t4, a0              /* apx = px-x0 */
    sub     a3, t6, a1              /* apy = py-y0 */
    slli    a0, a2, 8
    slli    a1, a3, 8
    mul     a0, a0, a0
    mul     a1, a1, a1
    add     a0, a0, a1
    j       .Lbeam_test

.Lend_cap:
    lw      a0, 60(sp)              /* x1 */
    lw      a1, 64(sp)              /* y1 */
    sub     a2, t4, a0
    sub     a3, t6, a1
    slli    a0, a2, 8
    slli    a1, a3, 8
    mul     a0, a0, a0
    mul     a1, a1, a1
    add     a0, a0, a1
    j       .Lbeam_test

.Lpoint_dist:                       /* len2 == 0: distance to (x0,y0) */
    lw      a0, 52(sp)
    lw      a1, 56(sp)
    sub     a2, t4, a0
    sub     a3, t6, a1
    slli    a0, a2, 8
    slli    a1, a3, 8
    mul     a0, a0, a0
    mul     a1, a1, a1
    add     a0, a0, a1

.Lbeam_test:
    /* a0 = d2_q16 */
    bgt     a0, s10, .Lglow_path
    mv      a0, s9                  /* contrib = brightness */
    j       .Lpixel_write

.Lglow_path:
    beqz    s11, .Lpx_incr          /* glow disabled */

    sub     a1, a0, s10             /* glow_d2_q16 */
    mul     a2, a1, s11
    mulh    a3, a1, s11
    slli    a3, a3, 2
    srli    a2, a2, 30
    or      a0, a3, a2              /* lut_idx */

    li      t1, 178
    bgeu    a0, t1, .Lpx_incr       /* past end of LUT */

    la      t1, gauss_lut
    add     t1, t1, a0
    lbu     a1, 0(t1)               /* gauss_lut[lut_idx] */
    mul     a0, s9, a1
    srai    a0, a0, 8               /* contrib = brightness*lut >> 8 */
    beqz    a0, .Lpx_incr

.Lpixel_write:
    /* t2 = &row[px*3] */
    li      a3, 3
    mul     t2, t4, a3
    add     t2, t5, t2

    lbu     t1, 0(t2)               /* existing pixel value */
    add     t1, t1, a0              /* t1 = v = pixel + contrib */

    li      a5, 255
    ble     t1, a5, .Lwrite_simple

    /* ── Saturation spill ── */
    sub     a2, t1, a5              /* v - 255 */
    addi    a2, a2, 3
    srai    a2, a2, 2               /* a2 = spill = (v-255+3)>>2 */
    blez    a2, .Lno_spill

    /* Left: px > bx0 */
    ble     t4, s2, .Lno_left
    addi    a0, t2, -3
    lbu     a1, 0(a0)
    add     a1, a1, a2
    blt     a1, a5, 1f
    mv      a1, a5
1:  sb      a1, 0(a0)
    sb      a1, 1(a0)
    sb      a1, 2(a0)
.Lno_left:

    /* Right: px < bx1 */
    bge     t4, s3, .Lno_right
    addi    a0, t2, 3
    lbu     a1, 0(a0)
    add     a1, a1, a2
    blt     a1, a5, 1f
    mv      a1, a5
1:  sb      a1, 0(a0)
    sb      a1, 1(a0)
    sb      a1, 2(a0)
.Lno_right:

    /* Top: py > by0  →  row - fb_w*3 */
    ble     t6, s4, .Lno_top
    slli    a1, s1, 1
    add     a1, s1, a1              /* fb_w*3 */
    sub     a0, t5, a1              /* prev_row */
    sub     a1, t2, t5              /* px*3 offset */
    add     a0, a0, a1
    lbu     a1, 0(a0)
    add     a1, a1, a2
    blt     a1, a5, 1f
    mv      a1, a5
1:  sb      a1, 0(a0)
    sb      a1, 1(a0)
    sb      a1, 2(a0)
.Lno_top:

    /* Bottom: py < by1  →  row + fb_w*3 */
    bge     t6, s5, .Lno_bottom
    slli    a1, s1, 1
    add     a1, s1, a1
    add     a0, t5, a1              /* next_row */
    sub     a1, t2, t5              /* px*3 offset */
    add     a0, a0, a1
    lbu     a1, 0(a0)
    add     a1, a1, a2
    blt     a1, a5, 1f
    mv      a1, a5
1:  sb      a1, 0(a0)
    sb      a1, 1(a0)
    sb      a1, 2(a0)
.Lno_bottom:

.Lno_spill:
    mv      t1, a5                  /* v = 255 */

.Lwrite_simple:
    sb      t1, 0(t2)
    sb      t1, 1(t2)
    sb      t1, 2(t2)

    /* ── Increment cross/dot for next px ── */
.Lpx_incr:
    beqz    s8, .Lpx_next
    sub     t3, t3, s7              /* cross -= dy */
    add     t0, t0, s6              /* dot   += dx */
.Lpx_next:
    addi    t4, t4, 1
    j       .Lpx_loop

.Lpx_done:
    /* Advance row-start cross/dot for next py */
    beqz    s8, .Lpy_next
    lw      a0, 68(sp)
    add     a0, a0, s6              /* cross_row += dx */
    sw      a0, 68(sp)
    lw      a0, 72(sp)
    add     a0, a0, s7              /* dot_row   += dy */
    sw      a0, 72(sp)
.Lpy_next:
    addi    t6, t6, 1
    j       .Lpy_loop

.Ldraw_ret:
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

    .size draw_line_asm_rgb888, .-draw_line_asm_rgb888

/* ════════════════════════════════════════════════════════════════════
 * undraw_line_asm_rgb888 — zero the bounding box rows with memset.
 *
 * Register map:
 *   s0  = fb
 *   s1  = fb_w
 *   s2  = bx0
 *   s3  = span_bytes  = (bx1-bx0+1)*3
 *   s4  = by0
 *   s5  = by1
 *   s6  = py  (outer-loop counter; saved so memset call doesn't clobber it)
 *
 * Stack frame (48 bytes):
 *   sp+ 0  : saved ra
 *   sp+ 4  : saved s0
 *   sp+ 8  : saved s1
 *   sp+12  : saved s2
 *   sp+16  : saved s3
 *   sp+20  : saved s4
 *   sp+24  : saved s5
 *   sp+28  : saved s6
 *   sp+32  : x0 local
 *   sp+36  : y0 local
 *   sp+40  : x1 local
 *   sp+44  : y1 local
 *   sp+48  : (unused — thickness removed)
 * ════════════════════════════════════════════════════════════════════ */

    .global undraw_line_asm_rgb888
    .type   undraw_line_asm_rgb888, @function
    .align  2

undraw_line_asm_rgb888:
    addi    sp, sp, -48
    sw      ra,  0(sp)
    sw      s0,  4(sp)
    sw      s1,  8(sp)
    sw      s2, 12(sp)
    sw      s3, 16(sp)
    sw      s4, 20(sp)
    sw      s5, 24(sp)
    sw      s6, 28(sp)

    mv      s0, a0          /* fb    */
    mv      s1, a1          /* fb_w  */

    /* Early exits */
//    beqz    s0, .Lundraw_ret
//    blez    s1, .Lundraw_ret
//    blez    a2, .Lundraw_ret    /* fb_h */

    /* Save x0,y0,x1,y1 */
    sw      a3, 32(sp)
    sw      a4, 36(sp)
    sw      a5, 40(sp)
    sw      a6, 44(sp)

    /* Rb = g_line_Rb — no glow/thickness needed for undraw bbox */
    la      t2, g_line_Rb
    lw      t2, 0(t2)           /* t2 = Rb */

    /* ── Bounding box ── */
    /* bx0 = iclamp(min(x0,x1) - Rb, 0, fb_w-1) */
    lw      a3, 32(sp)
    lw      a5, 40(sp)
    blt     a3, a5, 1f
    mv      a3, a5
1:  sub     a3, a3, t2
    bge     a3, zero, 1f
    li      a3, 0
1:  addi    t3, s1, -1          /* fb_w-1 */
    blt     a3, t3, 1f
    mv      a3, t3
1:  mv      s2, a3              /* s2 = bx0 */

    /* bx1 (kept in t0 only to derive span_bytes) */
    lw      a3, 32(sp)
    lw      a5, 40(sp)
    bgt     a3, a5, 1f
    mv      a3, a5
1:  add     a3, a3, t2
    bge     a3, zero, 1f
    li      a3, 0
1:  blt     a3, t3, 1f
    mv      a3, t3
1:                              /* a3 = bx1 */

    /* span_bytes = (bx1 - bx0 + 1) * 3 */
    sub     t0, a3, s2
    addi    t0, t0, 1
    slli    t1, t0, 1
    add     s3, t0, t1          /* s3 = span_bytes */

    /* by0 = iclamp(min(y0,y1) - R, 0, fb_h-1) */
    lw      a4, 36(sp)
    lw      a6, 44(sp)
    blt     a4, a6, 1f
    mv      a4, a6
1:  sub     a4, a4, t2
    bge     a4, zero, 1f
    li      a4, 0
1:  addi    t3, a2, -1          /* fb_h-1 */
    blt     a4, t3, 1f
    mv      a4, t3
1:  mv      s4, a4              /* s4 = by0 */

    /* by1 = iclamp(max(y0,y1) + R, 0, fb_h-1) */
    lw      a4, 36(sp)
    lw      a6, 44(sp)
    bgt     a4, a6, 1f
    mv      a4, a6
1:  add     a4, a4, t2
    bge     a4, zero, 1f
    li      a4, 0
1:  blt     a4, t3, 1f
    mv      a4, t3
1:  mv      s5, a4              /* s5 = by1 */

    /* ── Loop: memset each row ── */
    mv      s6, s4              /* py = by0 */
.Lundraw_loop:
    bgt     s6, s5, .Lundraw_ret

    /* a0 = fb + (py*fb_w + bx0) * 3 */
    mul     a0, s6, s1
    add     a0, a0, s2
    slli    t0, a0, 1
    add     a0, a0, t0          /* *3 */
    add     a0, s0, a0

    li      a1, 0
    mv      a2, s3              /* span_bytes */
    call    memset

    addi    s6, s6, 1
    j       .Lundraw_loop

.Lundraw_ret:
    lw      ra,  0(sp)
    lw      s0,  4(sp)
    lw      s1,  8(sp)
    lw      s2, 12(sp)
    lw      s3, 16(sp)
    lw      s4, 20(sp)
    lw      s5, 24(sp)
    lw      s6, 28(sp)
    addi    sp, sp, 48
    ret

    .size undraw_line_asm_rgb888, .-undraw_line_asm_rgb888
