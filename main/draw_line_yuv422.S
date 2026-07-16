// draw_line_yuv422.S  —  ESP32-P4 (RV32IMAC) anti-aliased thick line drawing
//
// YUV422 (YUYV) framebuffer variant — 2 bytes per pixel.
// Byte layout: [Y, UV]  where Y is luma and UV is U (x even) or V (x odd).
// Beam writes Y=alpha (brightness), UV=128 (neutral chroma → white beam).
//
// C signature (9 arguments):
//
//   void draw_line_asm_yuv422(uint8_t *fb,  // a0
//                              int fb_w,    // a1
//                              int fb_h,    // a2
//                              int x0,      // a3
//                              int y0,      // a4
//                              int x1,      // a5
//                              int y1,      // a6
//                              int brightness, // a7
//                              int thickness); // 60(sp) in callee
//
// Placement: .iram0.text

    .section .iram0.text, "ax"
    .align   4
    .global  draw_line_asm_yuv422
    .type    draw_line_asm_yuv422, @function

// ---------------------------------------------------------------------------
// Macro: PUT_PIXEL_YUV  fb fw fh px py alpha t0 t1
//   Writes Y=alpha, UV=128 to the YUYV framebuffer at pixel (px,py).
//   Byte offset = pixel_index * 2.
//   Bounds checks commented out.
// ---------------------------------------------------------------------------
.macro PUT_PIXEL_YUV fb, fw, fh, px, py, alpha, t0, t1
    // bltz    \px, .Lskip_ppyuv_\@
    // bge     \px, \fw, .Lskip_ppyuv_\@
    // bltz    \py, .Lskip_ppyuv_\@
    // bge     \py, \fh, .Lskip_ppyuv_\@
    mul     \t0, \py, \fw
    add     \t0, \t0, \px           // pixel index
    slli    \t0, \t0, 1             // byte offset = index * 2
    add     \t0, \fb, \t0           // ptr into framebuffer
    sb      \alpha, 0(\t0)          // Y = alpha
    li      \t1, 128
    sb      \t1, 1(\t0)             // UV = 128 (neutral chroma → white beam)
.Lskip_ppyuv_\@:
.endm

// ---------------------------------------------------------------------------
// draw_line_asm_yuv422
// ---------------------------------------------------------------------------
draw_line_asm_yuv422:
    // ----- PROLOGUE ---------------------------------------------------------
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

    // s0=fb, s1=fw, s2=fh, s3=steep, s4=half_w, s5=brightness
    // s6=y_fp(Q8), s7=grad, s8=x, s9=x1

    mv      s0, a0
    mv      s1, a1
    mv      s2, a2
    mv      s5, a7

    lw      t0, 60(sp)
    blt     t0, zero, .Lthickyuv_clamp
    srli    s4, t0, 1
    j       .Lthickyuv_ok
.Lthickyuv_clamp:
    li      s4, 0
.Lthickyuv_ok:

    mv      t1, a3
    mv      t2, a4
    mv      t3, a5
    mv      t4, a6

    // ----- STEEP DETECTION --------------------------------------------------
    sub     t5, t3, t1; bge t5, zero, .Labsyuv_dx; neg t5, t5
.Labsyuv_dx:
    sub     t6, t4, t2; bge t6, zero, .Labsyuv_dy; neg t6, t6
.Labsyuv_dy:
    li      s3, 0
    ble     t6, t5, .Lnoyuv_steep
    li      s3, 1
    mv      a2, t1;  mv t1, t2;  mv t2, a2
    mv      a2, t3;  mv t3, t4;  mv t4, a2
.Lnoyuv_steep:

    ble     t1, t3, .Lnoyuv_swap
    mv      a2, t1;  mv t1, t3;  mv t3, a2
    mv      a2, t2;  mv t2, t4;  mv t4, a2
.Lnoyuv_swap:

    mv      s8, t1
    mv      s9, t3

    sub     t5, t3, t1
    sub     t6, t4, t2

    // ----- DEGENERATE: dx == 0 ----------------------------------------------
    bnez    t5, .Lhasyuv_dx

    beqz    s3, .Lpyuv_nosteep
    mv      a0, t2; mv a1, t1
    j       .Lpyuv_draw
.Lpyuv_nosteep:
    mv      a0, t1; mv a1, t2
.Lpyuv_draw:
    neg     t5, s4
.Lpyuv_outer:
    bgt     t5, s4, .Lyuvdone
    neg     t6, s4
.Lpyuv_inner:
    bgt     t6, s4, .Lpyuv_next
    add     a2, a0, t6
    add     a3, a1, t5
    PUT_PIXEL_YUV s0, s1, s2, a2, a3, s5, a4, a5
    addi    t6, t6, 1
    j       .Lpyuv_inner
.Lpyuv_next:
    addi    t5, t5, 1
    j       .Lpyuv_outer

    // ----- MAIN CASE --------------------------------------------------------
.Lhasyuv_dx:
    slli    t6, t6, 8
    div     s7, t6, t5
    slli    s6, t2, 8

.Lxyuv_loop:
    bgt     s8, s9, .Lyuvdone

    srai    t0, s6, 8
    andi    t1, s6, 0xFF

    // --- TOP AA EDGE ---
    li      t2, 255
    sub     t2, t2, t1
    mul     t3, s5, t2
    srli    t3, t3, 8

    sub     a0, t0, s4
    beqz    s3, .Ltopyuv_nosteep
    PUT_PIXEL_YUV s0, s1, s2, a0, s8, t3, a3, a4
    j       .Ltopyuv_done
.Ltopyuv_nosteep:
    PUT_PIXEL_YUV s0, s1, s2, s8, a0, t3, a3, a4
.Ltopyuv_done:

    // --- CORE PIXELS ---
    sub     a0, t0, s4
    addi    a0, a0, 1
    add     a1, t0, s4

.Lcoreyuv_loop:
    bgt     a0, a1, .Lcoreyuv_done
    beqz    s3, .Lcoreyuv_nosteep
    PUT_PIXEL_YUV s0, s1, s2, a0, s8, s5, a4, a5
    j       .Lcoreyuv_next
.Lcoreyuv_nosteep:
    PUT_PIXEL_YUV s0, s1, s2, s8, a0, s5, a4, a5
.Lcoreyuv_next:
    addi    a0, a0, 1
    j       .Lcoreyuv_loop
.Lcoreyuv_done:

    // --- BOTTOM AA EDGE ---
    beqz    t1, .Lbotyuv_skip
    mul     t3, s5, t1
    srli    t3, t3, 8

    add     a0, t0, s4
    addi    a0, a0, 1
    beqz    s3, .Lbotyuv_nosteep
    PUT_PIXEL_YUV s0, s1, s2, a0, s8, t3, a3, a4
    j       .Lbotyuv_skip
.Lbotyuv_nosteep:
    PUT_PIXEL_YUV s0, s1, s2, s8, a0, t3, a3, a4
.Lbotyuv_skip:

    add     s6, s6, s7
    addi    s8, s8, 1
    j       .Lxyuv_loop

.Lyuvdone:
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

    .size draw_line_asm_yuv422, . - draw_line_asm_yuv422
