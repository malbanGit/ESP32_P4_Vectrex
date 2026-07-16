// draw_line_gauss_rgb888.S — ESP32-P4 (RV32IMAC)
//
// Two public functions:
//
//   void draw_line_asm_rgb888(uint8_t *fb, int fw, int fh,
//                              int x0, int y0, int x1, int y1,
//                              int brightness, int thickness);
//
//     Gaussian-glow additive line.  Reads existing pixel values and saturating-
//     adds a 5-pixel σ=0.8 Gaussian kernel centred on the Wu sub-pixel position.
//     Crossing lines accumulate brightness.  Endpoints are drawn as filled circles.
//
//   void undraw_line_asm_rgb888(uint8_t *fb, int fw, int fh,
//                                int x0, int y0, int x1, int y1,
//                                int brightness, int thickness);
//
//     Erases every pixel that draw could have touched (all 3 channels → 0).
//     'brightness' parameter is accepted but ignored.
//
// Algorithm: Wu major-axis walk with steep-detection and Q8 fixed-point gradient.
// For thin lines (hi==0): 5-pixel Gaussian kernel each column.
// For thick lines (hi>0): solid core [y_int-lo .. y_int+hi] + 2-pixel Gaussian fringe.
// Endpoints (and dots): filled circle of radius hi+2.
//
// Stack frame: 72 bytes.
//   68(sp) = ra  64=s0  60=s1  56=s2  52=s3  48=s4  44=s5
//   40=s6   36=s7  32=s8  28=s9  24=s10  20=s11  16=scratch
//   12=y1_orig  8=x1_orig  4=y0_orig  0=x0_orig
//   72(sp) = thickness (9th caller arg)

// ---------------------------------------------------------------------------
// Data
// ---------------------------------------------------------------------------
    .section .rodata
    .align  2

    // gauss_lut_rgb8[fi][k+2]
    // fi = frac>>5 (0..7), k = -2..+2
    // value = gaussian(k - frac/255, σ=0.8) * 255
gauss_lut_rgb8:
    .byte 11, 117, 255, 117, 11
    .byte  7,  95, 252, 140, 16
    .byte  5,  75, 243, 164, 23
    .byte  3,  58, 228, 188, 32
    .byte  2,  44, 210, 210, 44
    .byte  1,  32, 188, 228, 58
    .byte  1,  23, 164, 243, 75
    .byte  0,  16, 140, 252, 95

    // fringe_w_rgb8[0..1]: Gaussian weights at perpendicular offset 1 and 2
    // from the solid-core edge (for thick lines).
fringe_w_rgb8:
    .byte  117, 11

// ---------------------------------------------------------------------------
// Code
// ---------------------------------------------------------------------------
    .section .iram0.text, "ax"
    .align  4

// ---------------------------------------------------------------------------
// Macro: ADD_PX_RGB8  px py contrib ptr v sat
//   Saturating-add 'contrib' to all 3 RGB channels of fb[(py*fw+px)*3].
//   Bounds-checked.  Clobbers: ptr, v, sat.
//   Uses globals: s0=fb, s1=fw, s2=fh.
// ---------------------------------------------------------------------------
.macro ADD_PX_RGB8  px, py, contrib, ptr, v, sat
    bltz    \px, .Ladd_skip_\@
    bge     \px, s1, .Ladd_skip_\@
    bltz    \py, .Ladd_skip_\@
    bge     \py, s2, .Ladd_skip_\@
    mul     \ptr, \py, s1
    add     \ptr, \ptr, \px
    slli    \sat, \ptr, 1
    add     \ptr, \ptr, \sat         // offset * 3
    add     \ptr, \ptr, s0           // byte pointer
    lbu     \v,  0(\ptr)
    add     \v,  \v,  \contrib
    srli    \sat, \v,  8
    beqz    \sat, .Ladd_r_\@
    li      \v,  255
.Ladd_r_\@:
    sb      \v,  0(\ptr)
    lbu     \v,  1(\ptr)
    add     \v,  \v,  \contrib
    srli    \sat, \v,  8
    beqz    \sat, .Ladd_g_\@
    li      \v,  255
.Ladd_g_\@:
    sb      \v,  1(\ptr)
    lbu     \v,  2(\ptr)
    add     \v,  \v,  \contrib
    srli    \sat, \v,  8
    beqz    \sat, .Ladd_b_\@
    li      \v,  255
.Ladd_b_\@:
    sb      \v,  2(\ptr)
.Ladd_skip_\@:
.endm

// ---------------------------------------------------------------------------
// Macro: ZERO_PX_RGB8  px py ptr tmp
//   Writes 0 to all 3 channels of fb[(py*fw+px)*3].  Bounds-checked.
//   Clobbers: ptr, tmp.
// ---------------------------------------------------------------------------
.macro ZERO_PX_RGB8  px, py, ptr, tmp
    bltz    \px, .Lzp_skip_\@
    bge     \px, s1, .Lzp_skip_\@
    bltz    \py, .Lzp_skip_\@
    bge     \py, s2, .Lzp_skip_\@
    mul     \ptr, \py, s1
    add     \ptr, \ptr, \px
    slli    \tmp, \ptr, 1
    add     \ptr, \ptr, \tmp         // offset * 3
    add     \ptr, \ptr, s0
    sb      zero, 0(\ptr)
    sb      zero, 1(\ptr)
    sb      zero, 2(\ptr)
.Lzp_skip_\@:
.endm

// ---------------------------------------------------------------------------
// draw_line_asm_rgb888
// Register map:
//   s0=fb  s1=fw  s2=fh  s3=steep  s4=lo  s5=hi  s6=brightness
//   s7=y_fp(Q8)  s8=gradient  s9=x  s10=x1  s11=&gauss_lut
// ---------------------------------------------------------------------------
    .global  draw_line_asm_rgb888
    .type    draw_line_asm_rgb888, @function

draw_line_asm_rgb888:
    addi    sp,  sp, -72
    sw      ra,  68(sp)
    sw      s0,  64(sp)
    sw      s1,  60(sp)
    sw      s2,  56(sp)
    sw      s3,  52(sp)
    sw      s4,  48(sp)
    sw      s5,  44(sp)
    sw      s6,  40(sp)
    sw      s7,  36(sp)
    sw      s8,  32(sp)
    sw      s9,  28(sp)
    sw      s10, 24(sp)
    sw      s11, 20(sp)

    mv      s0, a0
    mv      s1, a1
    mv      s2, a2
    mv      s6, a7

    blez    s6, .Ldraw_done_rgb8

    sw      a3,  0(sp)       // x0_orig
    sw      a4,  4(sp)       // y0_orig
    sw      a5,  8(sp)       // x1_orig
    sw      a6, 12(sp)       // y1_orig

    la      s11, gauss_lut_rgb8

    // thickness → lo (s4) and hi (s5)
    lw      t0, 72(sp)
    li      s4, 0
    li      s5, 0
    ble     t0, zero, .Ldthk_rgb8
    addi    t1, t0, -1
    srli    s4, t1, 1
    srli    s5, t0, 1
.Ldthk_rgb8:

    mv      t1, a3
    mv      t2, a4
    mv      t3, a5
    mv      t4, a6

    // compute |dx|, |dy|
    sub     t5, t3, t1
    sub     t6, t4, t2
    mv      a0, t5
    bge     a0, zero, .Ldabsdx_rgb8
    neg     a0, a0
.Ldabsdx_rgb8:
    mv      a1, t6
    bge     a1, zero, .Ldabsdy_rgb8
    neg     a1, a1
.Ldabsdy_rgb8:

    // steep: swap axes when |dy| > |dx|
    li      s3, 0
    ble     a1, a0, .Ldnostep_rgb8
    li      s3, 1
    mv      a2, t1;  mv t1, t2;  mv t2, a2
    mv      a2, t3;  mv t3, t4;  mv t4, a2
.Ldnostep_rgb8:

    // ensure x0 ≤ x1 along major axis
    ble     t1, t3, .Lddir_rgb8
    mv      a2, t1;  mv t1, t3;  mv t3, a2
    mv      a2, t2;  mv t2, t4;  mv t4, a2
.Lddir_rgb8:

    mv      s9,  t1
    mv      s10, t3
    sub     t5,  t3, t1
    sub     t6,  t4, t2

    // endpoint circle radius = hi + 2  (stored at 16(sp))
    addi    a2, s5, 2
    sw      a2, 16(sp)

    // degenerate: dx == 0 → draw circle dot
    bnez    t5, .Ldhasdx_rgb8
    lw      a0, 0(sp)
    lw      a1, 4(sp)
    lw      a2, 16(sp)
    jal     ra, .Lcircle_draw_rgb8
    j       .Ldraw_done_rgb8

.Ldhasdx_rgb8:
    slli    t6, t6, 8
    div     s8, t6, t5       // gradient Q8
    slli    s7, t2, 8        // y_fp = y0 << 8

// =========================================================================
// X LOOP
// =========================================================================
.Ldxlp_rgb8:
    bgt     s9, s10, .Ldafter_xlp_rgb8

    srai    t0, s7, 8        // y_int
    andi    t1, s7, 0xFF     // frac
    srli    t2, t1, 5        // fi = frac >> 5

    bnez    s5, .Ldsolid_rgb8

    // ------------------------------------------------------------------
    // THIN: 5-pixel Gaussian kernel, k = -2 .. +2
    // ------------------------------------------------------------------
    slli    t3, t2, 2
    add     t3, t3, t2
    add     t3, t3, s11      // &gauss_lut[fi][0]

    li      t4, -2
.Ldklp_rgb8:
    li      t5, 3
    bge     t4, t5, .Ldxnxt_rgb8

    addi    t5, t4, 2
    add     t5, t5, t3
    lbu     t5, 0(t5)        // lut_val
    beqz    t5, .Ldknxt_rgb8

    mul     t6, s6, t5
    addi    t6, t6, 127
    srli    t6, t6, 8        // contrib
    beqz    t6, .Ldknxt_rgb8

    add     a0, t0, t4       // yk = y_int + k
    beqz    s3, .Ldkns_rgb8
    ADD_PX_RGB8  a0, s9, t6, a1, a2, a3
    j       .Ldknxt_rgb8
.Ldkns_rgb8:
    ADD_PX_RGB8  s9, a0, t6, a1, a2, a3
.Ldknxt_rgb8:
    addi    t4, t4, 1
    j       .Ldklp_rgb8

    // ------------------------------------------------------------------
    // SOLID CORE + FRINGE
    // ------------------------------------------------------------------
.Ldsolid_rgb8:
    sub     a0, t0, s4       // c = y_int - lo
    add     a1, t0, s5       // end = y_int + hi
.Ldcorelp_rgb8:
    bgt     a0, a1, .Ldfringe_rgb8
    beqz    s3, .Ldcorens_rgb8
    ADD_PX_RGB8  a0, s9, s6, a2, a3, a4
    j       .Ldcorenxt_rgb8
.Ldcorens_rgb8:
    ADD_PX_RGB8  s9, a0, s6, a2, a3, a4
.Ldcorenxt_rgb8:
    addi    a0, a0, 1
    j       .Ldcorelp_rgb8

.Ldfringe_rgb8:
    la      a2, fringe_w_rgb8
    li      a3, 0
.Ldfrlp_rgb8:
    li      a4, 2
    bge     a3, a4, .Ldxnxt_rgb8

    add     a4, a2, a3
    lbu     a4, 0(a4)
    beqz    a4, .Ldfrnxt_rgb8

    mul     t6, s6, a4
    addi    t6, t6, 127
    srli    t6, t6, 8
    beqz    t6, .Ldfrnxt_rgb8

    // positive side: yk = y_int + hi + 1 + f
    add     a4, t0, s5
    addi    a4, a4, 1
    add     a4, a4, a3
    beqz    s3, .Ldfrpns_rgb8
    ADD_PX_RGB8  a4, s9, t6, a5, t3, t4
    j       .Ldfrneg_rgb8
.Ldfrpns_rgb8:
    ADD_PX_RGB8  s9, a4, t6, a5, t3, t4

.Ldfrneg_rgb8:
    // negative side: yk = y_int - lo - 1 - f
    sub     a4, t0, s4
    addi    a4, a4, -1
    sub     a4, a4, a3
    beqz    s3, .Ldfrnns_rgb8
    ADD_PX_RGB8  a4, s9, t6, a5, t3, t4
    j       .Ldfrnxt_rgb8
.Ldfrnns_rgb8:
    ADD_PX_RGB8  s9, a4, t6, a5, t3, t4

.Ldfrnxt_rgb8:
    addi    a3, a3, 1
    j       .Ldfrlp_rgb8

.Ldxnxt_rgb8:
    add     s7, s7, s8
    addi    s9, s9, 1
    j       .Ldxlp_rgb8

// =========================================================================
// After X loop: draw endpoint circles
// =========================================================================
.Ldafter_xlp_rgb8:
    lw      a2, 16(sp)       // radius = hi + 2
    lw      a0,  0(sp)
    lw      a1,  4(sp)
    jal     ra, .Lcircle_draw_rgb8
    lw      a0,  8(sp)
    lw      a1, 12(sp)
    jal     ra, .Lcircle_draw_rgb8

.Ldraw_done_rgb8:
    lw      ra,  68(sp)
    lw      s0,  64(sp)
    lw      s1,  60(sp)
    lw      s2,  56(sp)
    lw      s3,  52(sp)
    lw      s4,  48(sp)
    lw      s5,  44(sp)
    lw      s6,  40(sp)
    lw      s7,  36(sp)
    lw      s8,  32(sp)
    lw      s9,  28(sp)
    lw      s10, 24(sp)
    lw      s11, 20(sp)
    addi    sp,  sp,  72
    ret

    .size draw_line_asm_rgb888, . - draw_line_asm_rgb888

// ---------------------------------------------------------------------------
// .Lcircle_draw_rgb8  — additive-fill circle
//   a0=cx  a1=cy  a2=radius  s6=brightness
//   t0=dy  t1=dx  t2=r²  t3=dy²  t4=dx²+dy²  t5=px  t6=py
//   ADD_PX_RGB8 scratch: a3, a4, a5
//   Does NOT touch s0-s11 or a0-a2.
// ---------------------------------------------------------------------------
.Lcircle_draw_rgb8:
    beqz    a2, .Lcdone_rgb8
    mul     t2, a2, a2
    neg     t0, a2
.Lcdy_rgb8:
    bgt     t0, a2, .Lcdone_rgb8
    mul     t3, t0, t0
    neg     t1, a2
.Lcdx_rgb8:
    bgt     t1, a2, .Lcdyne_rgb8
    mul     t4, t1, t1
    add     t4, t4, t3
    bgt     t4, t2, .Lcdxne_rgb8
    add     t5, a0, t1
    add     t6, a1, t0
    ADD_PX_RGB8  t5, t6, s6, a3, a4, a5
.Lcdxne_rgb8:
    addi    t1, t1, 1
    j       .Lcdx_rgb8
.Lcdyne_rgb8:
    addi    t0, t0, 1
    j       .Lcdy_rgb8
.Lcdone_rgb8:
    ret

// ===========================================================================
// undraw_line_asm_rgb888
// Same prologue/structure; zeros every pixel the draw could have touched.
// 'brightness' arg ignored.
// Register map: s0=fb  s1=fw  s2=fh  s3=steep  s4=lo  s5=hi
//               s7=y_fp  s8=gradient  s9=x  s10=x1
// ===========================================================================
    .global  undraw_line_asm_rgb888
    .type    undraw_line_asm_rgb888, @function

undraw_line_asm_rgb888:
    addi    sp,  sp, -72
    sw      ra,  68(sp)
    sw      s0,  64(sp)
    sw      s1,  60(sp)
    sw      s2,  56(sp)
    sw      s3,  52(sp)
    sw      s4,  48(sp)
    sw      s5,  44(sp)
    sw      s6,  40(sp)
    sw      s7,  36(sp)
    sw      s8,  32(sp)
    sw      s9,  28(sp)
    sw      s10, 24(sp)
    sw      s11, 20(sp)

    mv      s0, a0
    mv      s1, a1
    mv      s2, a2
    // s6 unused in undraw

    sw      a3,  0(sp)
    sw      a4,  4(sp)
    sw      a5,  8(sp)
    sw      a6, 12(sp)

    lw      t0, 72(sp)
    li      s4, 0
    li      s5, 0
    ble     t0, zero, .Luthk_rgb8
    addi    t1, t0, -1
    srli    s4, t1, 1
    srli    s5, t0, 1
.Luthk_rgb8:

    mv      t1, a3;  mv t2, a4;  mv t3, a5;  mv t4, a6

    sub     t5, t3, t1
    sub     t6, t4, t2
    mv      a0, t5
    bge     a0, zero, .Luabsdx_rgb8
    neg     a0, a0
.Luabsdx_rgb8:
    mv      a1, t6
    bge     a1, zero, .Luabsdy_rgb8
    neg     a1, a1
.Luabsdy_rgb8:

    li      s3, 0
    ble     a1, a0, .Lunostep_rgb8
    li      s3, 1
    mv      a2, t1;  mv t1, t2;  mv t2, a2
    mv      a2, t3;  mv t3, t4;  mv t4, a2
.Lunostep_rgb8:

    ble     t1, t3, .Ludir_rgb8
    mv      a2, t1;  mv t1, t3;  mv t3, a2
    mv      a2, t2;  mv t2, t4;  mv t4, a2
.Ludir_rgb8:

    mv      s9,  t1
    mv      s10, t3
    sub     t5, t3, t1
    sub     t6, t4, t2

    addi    a2, s5, 2        // circle radius
    sw      a2, 16(sp)

    bnez    t5, .Luhasdx_rgb8
    lw      a0,  0(sp)
    lw      a1,  4(sp)
    lw      a2, 16(sp)
    jal     ra, .Lcircle_zero_rgb8
    j       .Lundraw_done_rgb8

.Luhasdx_rgb8:
    slli    t6, t6, 8
    div     s8, t6, t5
    slli    s7, t2, 8

// =========================================================================
// X LOOP — zero beam extent
// =========================================================================
.Luxlp_rgb8:
    bgt     s9, s10, .Luafter_xlp_rgb8

    srai    t0, s7, 8        // y_int

    beqz    s5, .Luthin_rgb8
    sub     a0, t0, s4
    addi    a0, a0, -2       // from = y_int - lo - 2
    add     a1, t0, s5
    addi    a1, a1, 2        // to   = y_int + hi + 2
    j       .Lucollp_rgb8
.Luthin_rgb8:
    addi    a0, t0, -2       // from = y_int - 2
    addi    a1, t0,  2       // to   = y_int + 2

.Lucollp_rgb8:
    bgt     a0, a1, .Luxnxt_rgb8
    beqz    s3, .Lucolns_rgb8
    ZERO_PX_RGB8  a0, s9, a2, a3
    j       .Lucolnxt_rgb8
.Lucolns_rgb8:
    ZERO_PX_RGB8  s9, a0, a2, a3
.Lucolnxt_rgb8:
    addi    a0, a0, 1
    j       .Lucollp_rgb8

.Luxnxt_rgb8:
    add     s7, s7, s8
    addi    s9, s9, 1
    j       .Luxlp_rgb8

.Luafter_xlp_rgb8:
    lw      a2, 16(sp)
    lw      a0,  0(sp);  lw a1,  4(sp)
    jal     ra, .Lcircle_zero_rgb8
    lw      a0,  8(sp);  lw a1, 12(sp)
    jal     ra, .Lcircle_zero_rgb8

.Lundraw_done_rgb8:
    lw      ra,  68(sp)
    lw      s0,  64(sp)
    lw      s1,  60(sp)
    lw      s2,  56(sp)
    lw      s3,  52(sp)
    lw      s4,  48(sp)
    lw      s5,  44(sp)
    lw      s6,  40(sp)
    lw      s7,  36(sp)
    lw      s8,  32(sp)
    lw      s9,  28(sp)
    lw      s10, 24(sp)
    lw      s11, 20(sp)
    addi    sp,  sp,  72
    ret

    .size undraw_line_asm_rgb888, . - undraw_line_asm_rgb888

// ---------------------------------------------------------------------------
// .Lcircle_zero_rgb8 — zero-fill circle
//   a0=cx  a1=cy  a2=radius
//   t0=dy  t1=dx  t2=r²  t3=dy²  t4=dx²+dy²  t5=px  t6=py
//   ZERO_PX_RGB8 scratch: a3, a4
// ---------------------------------------------------------------------------
.Lcircle_zero_rgb8:
    beqz    a2, .Lczret_rgb8
    mul     t2, a2, a2
    neg     t0, a2
.Lczdy_rgb8:
    bgt     t0, a2, .Lczret_rgb8
    mul     t3, t0, t0
    neg     t1, a2
.Lczdx_rgb8:
    bgt     t1, a2, .Lczdyne_rgb8
    mul     t4, t1, t1
    add     t4, t4, t3
    bgt     t4, t2, .Lczdxne_rgb8
    add     t5, a0, t1
    add     t6, a1, t0
    ZERO_PX_RGB8  t5, t6, a3, a4
.Lczdxne_rgb8:
    addi    t1, t1, 1
    j       .Lczdx_rgb8
.Lczdyne_rgb8:
    addi    t0, t0, 1
    j       .Lczdy_rgb8
.Lczret_rgb8:
    ret
