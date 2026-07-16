// draw_line_gauss_rgb888.S — ESP32-P4 (RV32IMAC)
//
// void draw_line_asm_rgb888(uint8_t *fb, int fw, int fh,
//                            int x0, int y0, int x1, int y1,
//                            int brightness, int thickness);
//
//   Xiaolin Wu antialiased line for RGB888.  OVERWRITES pixels (no additive
//   accumulation, no flickering).  For thin lines: 2-pixel soft edge per Wu.
//   For thick lines: solid core + 1-pixel AA fringe each side.
//   Single-point degenerate: (lo+hi+1)² pixel block → 1 pixel for thickness=1.
//
// void undraw_line_asm_rgb888(uint8_t *fb, int fw, int fh,
//                              int x0, int y0, int x1, int y1,
//                              int brightness, int thickness);
//
//   Zeroes exactly the pixels draw_line_asm_rgb888 would have written.
//   'brightness' accepted but ignored.
//
// Frame: 60 bytes.  9th arg (thickness) at 60(sp) after callee adjusts sp.
//   56=ra 52=s0 48=s1 44=s2 40=s3 36=s4 32=s5 28=s6
//   24=s7 20=s8 16=s9 12=s10 8=s11 0-7=pad

    .section .iram0.text, "ax"
    .align   4

// ---------------------------------------------------------------------------
// PUT_PIXEL_888  px py alpha ptr tmp
//   Writes (alpha,alpha,alpha) to fb[(py*fw+px)*3].  Bounds-checked.
//   Uses s0=fb, s1=fw, s2=fh.  Clobbers ptr, tmp.
// ---------------------------------------------------------------------------
.macro PUT_PIXEL_888  px, py, alpha, ptr, tmp
    bltz    \px, .Lpp_skip_\@
    bge     \px, s1,  .Lpp_skip_\@
    bltz    \py, .Lpp_skip_\@
    bge     \py, s2,  .Lpp_skip_\@
    mul     \ptr, \py, s1
    add     \ptr, \ptr, \px
    slli    \tmp, \ptr, 1
    add     \ptr, \ptr, \tmp        // *3
    add     \ptr, \ptr, s0
    sb      \alpha, 0(\ptr)
    sb      \alpha, 1(\ptr)
    sb      \alpha, 2(\ptr)
.Lpp_skip_\@:
.endm

// ---------------------------------------------------------------------------
// ZERO_PIXEL_888  px py ptr tmp
//   Writes (0,0,0) to fb[(py*fw+px)*3].  Bounds-checked.
// ---------------------------------------------------------------------------
.macro ZERO_PIXEL_888  px, py, ptr, tmp
    bltz    \px, .Lzp_skip_\@
    bge     \px, s1,  .Lzp_skip_\@
    bltz    \py, .Lzp_skip_\@
    bge     \py, s2,  .Lzp_skip_\@
    mul     \ptr, \py, s1
    add     \ptr, \ptr, \px
    slli    \tmp, \ptr, 1
    add     \ptr, \ptr, \tmp        // *3
    add     \ptr, \ptr, s0
    sb      zero, 0(\ptr)
    sb      zero, 1(\ptr)
    sb      zero, 2(\ptr)
.Lzp_skip_\@:
.endm

// ===========================================================================
// draw_line_asm_rgb888
// Register map:
//   s0=fb  s1=fw  s2=fh  s3=steep  s4=lo  s5=hi  s6=brightness
//   s7=y_fp(Q8)  s8=gradient(Q8)  s9=x  s10=x1  s11=unused
// ===========================================================================
    .global  draw_line_asm_rgb888
    .type    draw_line_asm_rgb888, @function

draw_line_asm_rgb888:
    addi    sp,  sp, -60
    sw      ra,  56(sp)
    sw      s0,  52(sp)
    sw      s1,  48(sp)
    sw      s2,  44(sp)
    sw      s3,  40(sp)
    sw      s4,  36(sp)
    sw      s5,  32(sp)
    sw      s6,  28(sp)
    sw      s7,  24(sp)
    sw      s8,  20(sp)
    sw      s9,  16(sp)
    sw      s10, 12(sp)
    sw      s11,  8(sp)

    mv      s0, a0
    mv      s1, a1
    mv      s2, a2
    mv      s6, a7

    // thickness → lo (s4), hi (s5)
    lw      t0, 60(sp)
    li      s4, 0
    li      s5, 0
    ble     t0, zero, .Ldrw_thk_ok
    addi    t1, t0, -1
    srli    s4, t1, 1
    srli    s5, t0, 1
.Ldrw_thk_ok:

    mv      t1, a3          // x0
    mv      t2, a4          // y0
    mv      t3, a5          // x1
    mv      t4, a6          // y1

    // |dx|, |dy|
    sub     t5, t3, t1
    sub     t6, t4, t2
    mv      a0, t5
    bge     a0, zero, .Ldrw_absdx
    neg     a0, a0
.Ldrw_absdx:
    mv      a1, t6
    bge     a1, zero, .Ldrw_absdy
    neg     a1, a1
.Ldrw_absdy:

    // steep: swap axes when |dy| > |dx|
    li      s3, 0
    ble     a1, a0, .Ldrw_nosteep
    li      s3, 1
    mv      a2, t1;  mv t1, t2;  mv t2, a2
    mv      a2, t3;  mv t3, t4;  mv t4, a2
.Ldrw_nosteep:

    // ensure x0 ≤ x1
    ble     t1, t3, .Ldrw_dir_ok
    mv      a2, t1;  mv t1, t3;  mv t3, a2
    mv      a2, t2;  mv t2, t4;  mv t4, a2
.Ldrw_dir_ok:

    mv      s9,  t1
    mv      s10, t3
    sub     t5, t3, t1      // dx
    sub     t6, t4, t2      // dy

    // degenerate: dx==0 → solid dot block
    bnez    t5, .Ldrw_hasdx

    neg     t5, s4          // oy = -lo
.Ldrw_pt_oy:
    bgt     t5, s5, .Ldrw_done
    neg     t6, s4          // ox = -lo
.Ldrw_pt_ox:
    bgt     t6, s5, .Ldrw_pt_nxtoy
    beqz    s3, .Ldrw_pt_ns
    add     a0, t2, t6      // steep: px = minor+ox
    add     a1, t1, t5      //        py = major+oy
    PUT_PIXEL_888  a0, a1, s6, a2, a3
    j       .Ldrw_pt_nxtox
.Ldrw_pt_ns:
    add     a0, t1, t6      // non-steep: px = major+ox
    add     a1, t2, t5      //            py = minor+oy
    PUT_PIXEL_888  a0, a1, s6, a2, a3
.Ldrw_pt_nxtox:
    addi    t6, t6, 1
    j       .Ldrw_pt_ox
.Ldrw_pt_nxtoy:
    addi    t5, t5, 1
    j       .Ldrw_pt_oy

.Ldrw_hasdx:
    slli    t6, t6, 8
    div     s8, t6, t5      // gradient Q8
    slli    s7, t2, 8       // y_fp = y0 << 8

// =========================================================================
// X LOOP
// =========================================================================
.Ldrw_xlp:
    bgt     s9, s10, .Ldrw_done

    srai    t0, s7, 8       // y_int
    andi    t1, s7, 0xFF    // frac  (0..255)

    bnez    s5, .Ldrw_solid

    // ---- Wu thin (hi==0) ----
    // alpha_top = brightness * (256 - frac) >> 8
    li      t2, 256
    sub     t2, t2, t1
    mul     t3, s6, t2
    srli    t3, t3, 8       // alpha_top

    beqz    s3, .Ldrw_top_ns
    PUT_PIXEL_888  t0, s9, t3, a0, a1
    j       .Ldrw_bot
.Ldrw_top_ns:
    PUT_PIXEL_888  s9, t0, t3, a0, a1

.Ldrw_bot:
    beqz    t1, .Ldrw_xnxt  // frac==0: no bottom pixel
    mul     t3, s6, t1
    srli    t3, t3, 8       // alpha_bot
    beqz    t3, .Ldrw_xnxt
    addi    a2, t0, 1       // y_int + 1
    beqz    s3, .Ldrw_bot_ns
    PUT_PIXEL_888  a2, s9, t3, a0, a1
    j       .Ldrw_xnxt
.Ldrw_bot_ns:
    PUT_PIXEL_888  s9, a2, t3, a0, a1
    j       .Ldrw_xnxt

    // ---- Solid core + 1-pixel AA fringe (hi > 0) ----
.Ldrw_solid:
    sub     a0, t0, s4      // c = y_int - lo
    add     a1, t0, s5      // end = y_int + hi

.Ldrw_corelp:
    bgt     a0, a1, .Ldrw_fringe
    beqz    s3, .Ldrw_core_ns
    PUT_PIXEL_888  a0, s9, s6, a2, a3
    j       .Ldrw_corenxt
.Ldrw_core_ns:
    PUT_PIXEL_888  s9, a0, s6, a2, a3
.Ldrw_corenxt:
    addi    a0, a0, 1
    j       .Ldrw_corelp

.Ldrw_fringe:
    // Fringe above core: y_int-lo-1, alpha = brightness*(256-frac)>>8
    li      t2, 256
    sub     t2, t2, t1
    mul     t3, s6, t2
    srli    t3, t3, 8
    beqz    t3, .Ldrw_frb
    sub     a2, t0, s4
    addi    a2, a2, -1      // y_int - lo - 1
    beqz    s3, .Ldrw_fra_ns
    PUT_PIXEL_888  a2, s9, t3, a0, a1
    j       .Ldrw_frb
.Ldrw_fra_ns:
    PUT_PIXEL_888  s9, a2, t3, a0, a1

.Ldrw_frb:
    // Fringe below core: y_int+hi+1, alpha = brightness*frac>>8
    beqz    t1, .Ldrw_xnxt
    mul     t3, s6, t1
    srli    t3, t3, 8
    beqz    t3, .Ldrw_xnxt
    add     a2, t0, s5
    addi    a2, a2, 1       // y_int + hi + 1
    beqz    s3, .Ldrw_frb_ns
    PUT_PIXEL_888  a2, s9, t3, a0, a1
    j       .Ldrw_xnxt
.Ldrw_frb_ns:
    PUT_PIXEL_888  s9, a2, t3, a0, a1

.Ldrw_xnxt:
    add     s7, s7, s8
    addi    s9, s9, 1
    j       .Ldrw_xlp

.Ldrw_done:
    lw      ra,  56(sp)
    lw      s0,  52(sp)
    lw      s1,  48(sp)
    lw      s2,  44(sp)
    lw      s3,  40(sp)
    lw      s4,  36(sp)
    lw      s5,  32(sp)
    lw      s6,  28(sp)
    lw      s7,  24(sp)
    lw      s8,  20(sp)
    lw      s9,  16(sp)
    lw      s10, 12(sp)
    lw      s11,  8(sp)
    addi    sp,  sp,  60
    ret

    .size draw_line_asm_rgb888, . - draw_line_asm_rgb888

// ===========================================================================
// undraw_line_asm_rgb888
// Follows the exact same gradient walk; zeros every pixel draw would write.
// 'brightness' arg is accepted but ignored.
// ===========================================================================
    .global  undraw_line_asm_rgb888
    .type    undraw_line_asm_rgb888, @function

undraw_line_asm_rgb888:
    addi    sp,  sp, -60
    sw      ra,  56(sp)
    sw      s0,  52(sp)
    sw      s1,  48(sp)
    sw      s2,  44(sp)
    sw      s3,  40(sp)
    sw      s4,  36(sp)
    sw      s5,  32(sp)
    sw      s6,  28(sp)
    sw      s7,  24(sp)
    sw      s8,  20(sp)
    sw      s9,  16(sp)
    sw      s10, 12(sp)
    sw      s11,  8(sp)

    mv      s0, a0
    mv      s1, a1
    mv      s2, a2
    // s6 unused in undraw

    lw      t0, 60(sp)
    li      s4, 0
    li      s5, 0
    ble     t0, zero, .Lund_thk_ok
    addi    t1, t0, -1
    srli    s4, t1, 1
    srli    s5, t0, 1
.Lund_thk_ok:

    mv      t1, a3
    mv      t2, a4
    mv      t3, a5
    mv      t4, a6

    sub     t5, t3, t1
    sub     t6, t4, t2
    mv      a0, t5
    bge     a0, zero, .Lund_absdx
    neg     a0, a0
.Lund_absdx:
    mv      a1, t6
    bge     a1, zero, .Lund_absdy
    neg     a1, a1
.Lund_absdy:

    li      s3, 0
    ble     a1, a0, .Lund_nosteep
    li      s3, 1
    mv      a2, t1;  mv t1, t2;  mv t2, a2
    mv      a2, t3;  mv t3, t4;  mv t4, a2
.Lund_nosteep:

    ble     t1, t3, .Lund_dir_ok
    mv      a2, t1;  mv t1, t3;  mv t3, a2
    mv      a2, t2;  mv t2, t4;  mv t4, a2
.Lund_dir_ok:

    mv      s9,  t1
    mv      s10, t3
    sub     t5, t3, t1
    sub     t6, t4, t2

    // degenerate: dx==0 → zero same dot block
    bnez    t5, .Lund_hasdx

    neg     t5, s4
.Lund_pt_oy:
    bgt     t5, s5, .Lund_done
    neg     t6, s4
.Lund_pt_ox:
    bgt     t6, s5, .Lund_pt_nxtoy
    beqz    s3, .Lund_pt_ns
    add     a0, t2, t6
    add     a1, t1, t5
    ZERO_PIXEL_888  a0, a1, a2, a3
    j       .Lund_pt_nxtox
.Lund_pt_ns:
    add     a0, t1, t6
    add     a1, t2, t5
    ZERO_PIXEL_888  a0, a1, a2, a3
.Lund_pt_nxtox:
    addi    t6, t6, 1
    j       .Lund_pt_ox
.Lund_pt_nxtoy:
    addi    t5, t5, 1
    j       .Lund_pt_oy

.Lund_hasdx:
    slli    t6, t6, 8
    div     s8, t6, t5
    slli    s7, t2, 8

// =========================================================================
// X LOOP — zero beam pixels
// =========================================================================
.Lund_xlp:
    bgt     s9, s10, .Lund_done

    srai    t0, s7, 8       // y_int
    andi    t1, s7, 0xFF    // frac

    bnez    s5, .Lund_solid

    // Thin: zero y_int and y_int+1
    beqz    s3, .Lund_th_ns0
    ZERO_PIXEL_888  t0, s9, a0, a1
    j       .Lund_th_1
.Lund_th_ns0:
    ZERO_PIXEL_888  s9, t0, a0, a1
.Lund_th_1:
    addi    a2, t0, 1
    beqz    s3, .Lund_th_ns1
    ZERO_PIXEL_888  a2, s9, a0, a1
    j       .Lund_xnxt
.Lund_th_ns1:
    ZERO_PIXEL_888  s9, a2, a0, a1
    j       .Lund_xnxt

    // Thick: zero core + 1 fringe pixel each side
.Lund_solid:
    sub     a0, t0, s4
    addi    a0, a0, -1      // y_int - lo - 1 (fringe)
    add     a1, t0, s5
    addi    a1, a1, 1       // y_int + hi + 1 (fringe)

.Lund_corelp:
    bgt     a0, a1, .Lund_xnxt
    beqz    s3, .Lund_core_ns
    ZERO_PIXEL_888  a0, s9, a2, a3
    j       .Lund_corenxt
.Lund_core_ns:
    ZERO_PIXEL_888  s9, a0, a2, a3
.Lund_corenxt:
    addi    a0, a0, 1
    j       .Lund_corelp

.Lund_xnxt:
    add     s7, s7, s8
    addi    s9, s9, 1
    j       .Lund_xlp

.Lund_done:
    lw      ra,  56(sp)
    lw      s0,  52(sp)
    lw      s1,  48(sp)
    lw      s2,  44(sp)
    lw      s3,  40(sp)
    lw      s4,  36(sp)
    lw      s5,  32(sp)
    lw      s6,  28(sp)
    lw      s7,  24(sp)
    lw      s8,  20(sp)
    lw      s9,  16(sp)
    lw      s10, 12(sp)
    lw      s11,  8(sp)
    addi    sp,  sp,  60
    ret

    .size undraw_line_asm_rgb888, . - undraw_line_asm_rgb888
