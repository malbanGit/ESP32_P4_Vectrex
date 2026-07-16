// draw_line_rgb888.S  —  ESP32-P4 (RV32IMAC) anti-aliased thick line drawing
//
// RGB888 framebuffer variant (3 bytes per pixel, packed R,G,B).
// Identical Wu AA + thickness algorithm to draw_line.S (RGB565).
//
// C signature (9 arguments; 9th goes on the stack per the RISC-V ABI):
//
//   void draw_line_asm_rgb888(uint8_t *fb,  // a0  framebuffer base
//                              int fb_w,    // a1  pixels per row
//                              int fb_h,    // a2  rows
//                              int x0,      // a3
//                              int y0,      // a4
//                              int x1,      // a5
//                              int y1,      // a6
//                              int brightness, // a7  0-255
//                              int thickness); // stack[0] → 60(sp) in callee
//
// Beam colour: greyscale white — R=G=B=alpha.  No packing macro required.
// Byte offset  = pixel_index * 3  = (pixel_index << 1) + pixel_index.
//
// Placement: .iram0.text (fast on-chip SRAM).

    .section .iram0.text, "ax"
    .align   4
    .global  draw_line_asm_rgb888
    .type    draw_line_asm_rgb888, @function

// ---------------------------------------------------------------------------
// Macro: PUT_PIXEL_888  fb fw fh px py alpha t0 t1
//   Writes (alpha,alpha,alpha) to fb[py*fw+px]*3.
//   Bounds checks commented out; caller guarantees valid coords.
//   Clobbers: t0, t1.
// ---------------------------------------------------------------------------
.macro PUT_PIXEL_888 fb, fw, fh, px, py, alpha, t0, t1
    // bltz    \px, .Lskip_pp8_\@
    // bge     \px, \fw, .Lskip_pp8_\@
    // bltz    \py, .Lskip_pp8_\@
    // bge     \py, \fh, .Lskip_pp8_\@
    mul     \t0, \py, \fw
    add     \t0, \t0, \px           // pixel index
    slli    \t1, \t0, 1
    add     \t0, \t1, \t0           // byte offset = index * 3
    add     \t0, \fb, \t0           // ptr into framebuffer
    sb      \alpha, 0(\t0)          // R
    sb      \alpha, 1(\t0)          // G
    sb      \alpha, 2(\t0)          // B
.Lskip_pp8_\@:
.endm

// ---------------------------------------------------------------------------
// draw_line_asm_rgb888
// ---------------------------------------------------------------------------
draw_line_asm_rgb888:
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

    // s0 = fb, s1 = fb_w, s2 = fb_h, s3 = steep, s4 = half_w
    // s5 = brightness, s6 = y_fp (Q8), s7 = gradient, s8 = x, s9 = x1

    mv      s0, a0          // fb
    mv      s1, a1          // fb_w
    mv      s2, a2          // fb_h
    mv      s5, a7          // brightness

    lw      t0, 60(sp)      // thickness (9th arg)
    blt     t0, zero, .Lthick8_clamp
    srli    s4, t0, 1       // half_w = thickness >> 1
    j       .Lthick8_ok
.Lthick8_clamp:
    li      s4, 0
.Lthick8_ok:

    // Load x0,y0,x1,y1 (still in argument registers)
    mv      t1, a3          // x0
    mv      t2, a4          // y0
    mv      t3, a5          // x1
    mv      t4, a6          // y1

    // ----- STEEP DETECTION --------------------------------------------------
    sub     t5, t3, t1; bge t5, zero, .Labs8_dx; neg t5, t5
.Labs8_dx:
    sub     t6, t4, t2; bge t6, zero, .Labs8_dy; neg t6, t6
.Labs8_dy:
    li      s3, 0
    ble     t6, t5, .Lno8_steep
    li      s3, 1
    mv      a2, t1;  mv t1, t2;  mv t2, a2
    mv      a2, t3;  mv t3, t4;  mv t4, a2
.Lno8_steep:

    // ----- ENSURE x0 ≤ x1 --------------------------------------------------
    ble     t1, t3, .Lno8_swap
    mv      a2, t1;  mv t1, t3;  mv t3, a2
    mv      a2, t2;  mv t2, t4;  mv t4, a2
.Lno8_swap:

    mv      s8, t1          // x = x0
    mv      s9, t3          // x1

    sub     t5, t3, t1      // dx
    sub     t6, t4, t2      // dy

    // ----- DEGENERATE: dx == 0 → square point -------------------------------
    bnez    t5, .Lhas8_dx

    beqz    s3, .Lpt8_nosteep
    mv      a0, t2; mv a1, t1
    j       .Lpt8_draw
.Lpt8_nosteep:
    mv      a0, t1; mv a1, t2
.Lpt8_draw:
    neg     t5, s4          // oy = -half_w
.Lpt8_outer:
    bgt     t5, s4, .L8done
    neg     t6, s4
.Lpt8_inner:
    bgt     t6, s4, .Lpt8_next
    add     a2, a0, t6
    add     a3, a1, t5
    PUT_PIXEL_888 s0, s1, s2, a2, a3, s5, a4, a5
    addi    t6, t6, 1
    j       .Lpt8_inner
.Lpt8_next:
    addi    t5, t5, 1
    j       .Lpt8_outer

    // ----- MAIN CASE --------------------------------------------------------
.Lhas8_dx:
    slli    t6, t6, 8
    div     s7, t6, t5      // gradient (Q8)
    slli    s6, t2, 8       // y_fp = y0 << 8

    // =========================================================================
    // X LOOP
    // =========================================================================
.Lx8_loop:
    bgt     s8, s9, .L8done

    srai    t0, s6, 8       // y_int
    andi    t1, s6, 0xFF    // frac

    // --- TOP AA EDGE ---
    li      t2, 255
    sub     t2, t2, t1      // alpha_top = 255 - frac
    mul     t3, s5, t2
    srli    t3, t3, 8       // scaled_b_top

    sub     a0, t0, s4      // ytop = y_int - half_w
    beqz    s3, .Ltop8_nosteep
    PUT_PIXEL_888 s0, s1, s2, a0, s8, t3, a3, a4
    j       .Ltop8_done
.Ltop8_nosteep:
    PUT_PIXEL_888 s0, s1, s2, s8, a0, t3, a3, a4
.Ltop8_done:

    // --- CORE PIXELS ---
    sub     a0, t0, s4
    addi    a0, a0, 1       // core_start = y_int - half_w + 1
    add     a1, t0, s4      // core_end   = y_int + half_w

.Lcore8_loop:
    bgt     a0, a1, .Lcore8_done
    beqz    s3, .Lcore8_nosteep
    PUT_PIXEL_888 s0, s1, s2, a0, s8, s5, a4, a5
    j       .Lcore8_next
.Lcore8_nosteep:
    PUT_PIXEL_888 s0, s1, s2, s8, a0, s5, a4, a5
.Lcore8_next:
    addi    a0, a0, 1
    j       .Lcore8_loop
.Lcore8_done:

    // --- BOTTOM AA EDGE ---
    beqz    t1, .Lbot8_skip
    mul     t3, s5, t1
    srli    t3, t3, 8       // scaled_b_bot

    add     a0, t0, s4
    addi    a0, a0, 1       // ybot = y_int + half_w + 1
    beqz    s3, .Lbot8_nosteep
    PUT_PIXEL_888 s0, s1, s2, a0, s8, t3, a3, a4
    j       .Lbot8_skip
.Lbot8_nosteep:
    PUT_PIXEL_888 s0, s1, s2, s8, a0, t3, a3, a4
.Lbot8_skip:

    add     s6, s6, s7      // y_fp += gradient
    addi    s8, s8, 1       // x++
    j       .Lx8_loop

    // =========================================================================
    // EPILOGUE
    // =========================================================================
.L8done:
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
