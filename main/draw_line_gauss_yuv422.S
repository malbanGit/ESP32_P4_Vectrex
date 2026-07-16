// draw_line_gauss_yuv422.S — ESP32-P4 (RV32IMAC)
//
// Two public functions (same algorithm as draw_line_gauss_rgb888.S):
//
//   void draw_line_asm_yuv422(uint8_t *fb, int fw, int fh,
//                              int x0, int y0, int x1, int y1,
//                              int brightness, int thickness);
//
//   void undraw_line_asm_yuv422(uint8_t *fb, int fw, int fh,
//                                int x0, int y0, int x1, int y1,
//                                int brightness, int thickness);
//
// YUV422 (YUYV) layout: pixel (px,py) has Y at (py*fw+px)*2, UV at +1.
// Draw: saturating-adds to Y only; UV stays untouched (neutral chroma=white).
// Undraw: zeros Y only; UV stays at 128 (black luma).
//
// Stack frame / register map identical to draw_line_gauss_rgb888.S.

// ---------------------------------------------------------------------------
// Data
// ---------------------------------------------------------------------------
    .section .rodata
    .align  2

gauss_lut_yuv:
    .byte 11, 117, 255, 117, 11
    .byte  7,  95, 252, 140, 16
    .byte  5,  75, 243, 164, 23
    .byte  3,  58, 228, 188, 32
    .byte  2,  44, 210, 210, 44
    .byte  1,  32, 188, 228, 58
    .byte  1,  23, 164, 243, 75
    .byte  0,  16, 140, 252, 95

fringe_w_yuv:
    .byte  117, 11

// ---------------------------------------------------------------------------
// Code
// ---------------------------------------------------------------------------
    .section .iram0.text, "ax"
    .align  4

// ---------------------------------------------------------------------------
// Macro: ADD_PX_YUV  px py contrib ptr v sat
//   Saturating-add 'contrib' to the Y byte of fb[(py*fw+px)*2].
//   UV byte unchanged.  Bounds-checked.  Clobbers: ptr, v, sat.
// ---------------------------------------------------------------------------
.macro ADD_PX_YUV  px, py, contrib, ptr, v, sat
    bltz    \px, .Ladd_skip_yuv_\@
    bge     \px, s1, .Ladd_skip_yuv_\@
    bltz    \py, .Ladd_skip_yuv_\@
    bge     \py, s2, .Ladd_skip_yuv_\@
    mul     \ptr, \py, s1
    add     \ptr, \ptr, \px
    slli    \ptr, \ptr, 1          // * 2 (2 bytes per pixel in YUV422)
    add     \ptr, \ptr, s0
    lbu     \v,  0(\ptr)           // Y byte
    add     \v,  \v,  \contrib
    srli    \sat, \v,  8
    beqz    \sat, .Ladd_y_yuv_\@
    li      \v,  255
.Ladd_y_yuv_\@:
    sb      \v,  0(\ptr)           // write Y; leave UV at 1(\ptr) unchanged
.Ladd_skip_yuv_\@:
.endm

// ---------------------------------------------------------------------------
// Macro: ZERO_PX_YUV  px py ptr tmp
//   Zeros the Y byte of fb[(py*fw+px)*2].  UV unchanged.
// ---------------------------------------------------------------------------
.macro ZERO_PX_YUV  px, py, ptr, tmp
    bltz    \px, .Lzp_skip_yuv_\@
    bge     \px, s1, .Lzp_skip_yuv_\@
    bltz    \py, .Lzp_skip_yuv_\@
    bge     \py, s2, .Lzp_skip_yuv_\@
    mul     \ptr, \py, s1
    add     \ptr, \ptr, \px
    slli    \ptr, \ptr, 1
    add     \ptr, \ptr, s0
    sb      zero, 0(\ptr)          // zero Y only
.Lzp_skip_yuv_\@:
.endm

// ---------------------------------------------------------------------------
// draw_line_asm_yuv422
// ---------------------------------------------------------------------------
    .global  draw_line_asm_yuv422
    .type    draw_line_asm_yuv422, @function

draw_line_asm_yuv422:
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

    blez    s6, .Ldraw_done_yuv

    sw      a3,  0(sp)
    sw      a4,  4(sp)
    sw      a5,  8(sp)
    sw      a6, 12(sp)

    la      s11, gauss_lut_yuv

    lw      t0, 72(sp)
    li      s4, 0
    li      s5, 0
    ble     t0, zero, .Ldthk_yuv
    addi    t1, t0, -1
    srli    s4, t1, 1
    srli    s5, t0, 1
.Ldthk_yuv:

    mv      t1, a3;  mv t2, a4;  mv t3, a5;  mv t4, a6

    sub     t5, t3, t1
    sub     t6, t4, t2
    mv      a0, t5
    bge     a0, zero, .Ldabsdx_yuv
    neg     a0, a0
.Ldabsdx_yuv:
    mv      a1, t6
    bge     a1, zero, .Ldabsdy_yuv
    neg     a1, a1
.Ldabsdy_yuv:

    li      s3, 0
    ble     a1, a0, .Ldnostep_yuv
    li      s3, 1
    mv      a2, t1;  mv t1, t2;  mv t2, a2
    mv      a2, t3;  mv t3, t4;  mv t4, a2
.Ldnostep_yuv:

    ble     t1, t3, .Lddir_yuv
    mv      a2, t1;  mv t1, t3;  mv t3, a2
    mv      a2, t2;  mv t2, t4;  mv t4, a2
.Lddir_yuv:

    mv      s9,  t1
    mv      s10, t3
    sub     t5, t3, t1
    sub     t6, t4, t2

    addi    a2, s5, 2
    sw      a2, 16(sp)

    bnez    t5, .Ldhasdx_yuv
    lw      a0,  0(sp)
    lw      a1,  4(sp)
    lw      a2, 16(sp)
    jal     ra, .Lcircle_draw_yuv
    j       .Ldraw_done_yuv

.Ldhasdx_yuv:
    slli    t6, t6, 8
    div     s8, t6, t5
    slli    s7, t2, 8

// =========================================================================
// X LOOP
// =========================================================================
.Ldxlp_yuv:
    bgt     s9, s10, .Ldafter_xlp_yuv

    srai    t0, s7, 8
    andi    t1, s7, 0xFF
    srli    t2, t1, 5

    bnez    s5, .Ldsolid_yuv

    // THIN Gaussian
    slli    t3, t2, 2
    add     t3, t3, t2
    add     t3, t3, s11

    li      t4, -2
.Ldklp_yuv:
    li      t5, 3
    bge     t4, t5, .Ldxnxt_yuv

    addi    t5, t4, 2
    add     t5, t5, t3
    lbu     t5, 0(t5)
    beqz    t5, .Ldknxt_yuv

    mul     t6, s6, t5
    addi    t6, t6, 127
    srli    t6, t6, 8
    beqz    t6, .Ldknxt_yuv

    add     a0, t0, t4
    beqz    s3, .Ldkns_yuv
    ADD_PX_YUV  a0, s9, t6, a1, a2, a3
    j       .Ldknxt_yuv
.Ldkns_yuv:
    ADD_PX_YUV  s9, a0, t6, a1, a2, a3
.Ldknxt_yuv:
    addi    t4, t4, 1
    j       .Ldklp_yuv

    // SOLID + FRINGE
.Ldsolid_yuv:
    sub     a0, t0, s4
    add     a1, t0, s5
.Ldcorelp_yuv:
    bgt     a0, a1, .Ldfringe_yuv
    beqz    s3, .Ldcorens_yuv
    ADD_PX_YUV  a0, s9, s6, a2, a3, a4
    j       .Ldcorenxt_yuv
.Ldcorens_yuv:
    ADD_PX_YUV  s9, a0, s6, a2, a3, a4
.Ldcorenxt_yuv:
    addi    a0, a0, 1
    j       .Ldcorelp_yuv

.Ldfringe_yuv:
    la      a2, fringe_w_yuv
    li      a3, 0
.Ldfrlp_yuv:
    li      a4, 2
    bge     a3, a4, .Ldxnxt_yuv

    add     a4, a2, a3
    lbu     a4, 0(a4)
    beqz    a4, .Ldfrnxt_yuv

    mul     t6, s6, a4
    addi    t6, t6, 127
    srli    t6, t6, 8
    beqz    t6, .Ldfrnxt_yuv

    add     a4, t0, s5
    addi    a4, a4, 1
    add     a4, a4, a3
    beqz    s3, .Ldfrpns_yuv
    ADD_PX_YUV  a4, s9, t6, a5, t3, t4
    j       .Ldfrneg_yuv
.Ldfrpns_yuv:
    ADD_PX_YUV  s9, a4, t6, a5, t3, t4

.Ldfrneg_yuv:
    sub     a4, t0, s4
    addi    a4, a4, -1
    sub     a4, a4, a3
    beqz    s3, .Ldfrnns_yuv
    ADD_PX_YUV  a4, s9, t6, a5, t3, t4
    j       .Ldfrnxt_yuv
.Ldfrnns_yuv:
    ADD_PX_YUV  s9, a4, t6, a5, t3, t4

.Ldfrnxt_yuv:
    addi    a3, a3, 1
    j       .Ldfrlp_yuv

.Ldxnxt_yuv:
    add     s7, s7, s8
    addi    s9, s9, 1
    j       .Ldxlp_yuv

.Ldafter_xlp_yuv:
    lw      a2, 16(sp)
    lw      a0,  0(sp);  lw a1,  4(sp)
    jal     ra, .Lcircle_draw_yuv
    lw      a0,  8(sp);  lw a1, 12(sp)
    jal     ra, .Lcircle_draw_yuv

.Ldraw_done_yuv:
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

    .size draw_line_asm_yuv422, . - draw_line_asm_yuv422

// ---------------------------------------------------------------------------
// .Lcircle_draw_yuv — additive-fill circle for YUV422
//   a0=cx  a1=cy  a2=radius  s6=brightness
// ---------------------------------------------------------------------------
.Lcircle_draw_yuv:
    beqz    a2, .Lcdone_yuv
    mul     t2, a2, a2
    neg     t0, a2
.Lcdy_yuv:
    bgt     t0, a2, .Lcdone_yuv
    mul     t3, t0, t0
    neg     t1, a2
.Lcdx_yuv:
    bgt     t1, a2, .Lcdyne_yuv
    mul     t4, t1, t1
    add     t4, t4, t3
    bgt     t4, t2, .Lcdxne_yuv
    add     t5, a0, t1
    add     t6, a1, t0
    ADD_PX_YUV  t5, t6, s6, a3, a4, a5
.Lcdxne_yuv:
    addi    t1, t1, 1
    j       .Lcdx_yuv
.Lcdyne_yuv:
    addi    t0, t0, 1
    j       .Lcdy_yuv
.Lcdone_yuv:
    ret

// ===========================================================================
// undraw_line_asm_yuv422
// ===========================================================================
    .global  undraw_line_asm_yuv422
    .type    undraw_line_asm_yuv422, @function

undraw_line_asm_yuv422:
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

    sw      a3,  0(sp)
    sw      a4,  4(sp)
    sw      a5,  8(sp)
    sw      a6, 12(sp)

    lw      t0, 72(sp)
    li      s4, 0
    li      s5, 0
    ble     t0, zero, .Luthk_yuv
    addi    t1, t0, -1
    srli    s4, t1, 1
    srli    s5, t0, 1
.Luthk_yuv:

    mv      t1, a3;  mv t2, a4;  mv t3, a5;  mv t4, a6

    sub     t5, t3, t1
    sub     t6, t4, t2
    mv      a0, t5
    bge     a0, zero, .Luabsdx_yuv
    neg     a0, a0
.Luabsdx_yuv:
    mv      a1, t6
    bge     a1, zero, .Luabsdy_yuv
    neg     a1, a1
.Luabsdy_yuv:

    li      s3, 0
    ble     a1, a0, .Lunostep_yuv
    li      s3, 1
    mv      a2, t1;  mv t1, t2;  mv t2, a2
    mv      a2, t3;  mv t3, t4;  mv t4, a2
.Lunostep_yuv:

    ble     t1, t3, .Ludir_yuv
    mv      a2, t1;  mv t1, t3;  mv t3, a2
    mv      a2, t2;  mv t2, t4;  mv t4, a2
.Ludir_yuv:

    mv      s9,  t1
    mv      s10, t3
    sub     t5, t3, t1
    sub     t6, t4, t2

    addi    a2, s5, 2
    sw      a2, 16(sp)

    bnez    t5, .Luhasdx_yuv
    lw      a0,  0(sp)
    lw      a1,  4(sp)
    lw      a2, 16(sp)
    jal     ra, .Lcircle_zero_yuv
    j       .Lundraw_done_yuv

.Luhasdx_yuv:
    slli    t6, t6, 8
    div     s8, t6, t5
    slli    s7, t2, 8

.Luxlp_yuv:
    bgt     s9, s10, .Luafter_xlp_yuv

    srai    t0, s7, 8

    beqz    s5, .Luthin_yuv
    sub     a0, t0, s4
    addi    a0, a0, -2
    add     a1, t0, s5
    addi    a1, a1, 2
    j       .Lucollp_yuv
.Luthin_yuv:
    addi    a0, t0, -2
    addi    a1, t0,  2

.Lucollp_yuv:
    bgt     a0, a1, .Luxnxt_yuv
    beqz    s3, .Lucolns_yuv
    ZERO_PX_YUV  a0, s9, a2, a3
    j       .Lucolnxt_yuv
.Lucolns_yuv:
    ZERO_PX_YUV  s9, a0, a2, a3
.Lucolnxt_yuv:
    addi    a0, a0, 1
    j       .Lucollp_yuv

.Luxnxt_yuv:
    add     s7, s7, s8
    addi    s9, s9, 1
    j       .Luxlp_yuv

.Luafter_xlp_yuv:
    lw      a2, 16(sp)
    lw      a0,  0(sp);  lw a1,  4(sp)
    jal     ra, .Lcircle_zero_yuv
    lw      a0,  8(sp);  lw a1, 12(sp)
    jal     ra, .Lcircle_zero_yuv

.Lundraw_done_yuv:
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

    .size undraw_line_asm_yuv422, . - undraw_line_asm_yuv422

// ---------------------------------------------------------------------------
// .Lcircle_zero_yuv — zero-fill circle, YUV422
// ---------------------------------------------------------------------------
.Lcircle_zero_yuv:
    beqz    a2, .Lczret_yuv
    mul     t2, a2, a2
    neg     t0, a2
.Lczdy_yuv:
    bgt     t0, a2, .Lczret_yuv
    mul     t3, t0, t0
    neg     t1, a2
.Lczdx_yuv:
    bgt     t1, a2, .Lczdyne_yuv
    mul     t4, t1, t1
    add     t4, t4, t3
    bgt     t4, t2, .Lczdxne_yuv
    add     t5, a0, t1
    add     t6, a1, t0
    ZERO_PX_YUV  t5, t6, a3, a4
.Lczdxne_yuv:
    addi    t1, t1, 1
    j       .Lczdx_yuv
.Lczdyne_yuv:
    addi    t0, t0, 1
    j       .Lczdy_yuv
.Lczret_yuv:
    ret
