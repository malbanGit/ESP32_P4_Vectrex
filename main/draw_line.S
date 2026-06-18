// draw_line.S  —  ESP32-P4 (RV32IMAC) anti-aliased thick line drawing
//
// C signature (9 arguments; 9th goes on the stack per the RISC-V ABI):
//
//   void draw_line_asm(uint16_t *fb,   // a0  framebuffer pointer
//                      int       fb_w, // a1  pixels per row  (e.g. 1280)
//                      int       fb_h, // a2  rows            (e.g.  800)
//                      int       x0,   // a3
//                      int       y0,   // a4
//                      int       x1,   // a5
//                      int       y1,   // a6
//                      int  brightness,// a7  0-255
//                      int  thickness);// stack[0] at call site → 60(sp) in callee
//
// Algorithm: Xiaolin Wu anti-aliasing extended to arbitrary thickness.
//   For each step along the major axis:
//     • top AA edge  pixel: alpha = 255 - frac
//     • solid core   pixels: alpha = 255
//     • bottom AA edge pixel: alpha = frac
//   Color format: RGB565 grayscale  (R=G=B proportional to brightness).
//
// Placement: .iram0.text so every byte lives in fast on-chip SRAM.

    .section .iram0.text, "ax"
    .align   4
    .global  draw_line_asm
    .type    draw_line_asm, @function

// ---------------------------------------------------------------------------
// Macro: PUT_PIXEL  fb, fb_w, fb_h, px, py, color, tmp
//   Writes color (RGB565) to fb[py*fb_w + px] after bounds check.
//   Clobbers: tmp only.  All other inputs preserved.
// ---------------------------------------------------------------------------
.macro PUT_PIXEL fb, fb_w, fb_h, px, py, color, tmp
    bltz    \px, .Lskip_pp_\@
    bge     \px, \fb_w, .Lskip_pp_\@
    bltz    \py, .Lskip_pp_\@
    bge     \py, \fb_h, .Lskip_pp_\@
    mul     \tmp, \py, \fb_w
    add     \tmp, \tmp, \px
    slli    \tmp, \tmp, 1           // byte offset (* sizeof(uint16_t))
    add     \tmp, \fb, \tmp
    sh      \color, 0(\tmp)
.Lskip_pp_\@:
.endm

// ---------------------------------------------------------------------------
// Macro: B_TO_RGB565  out, b, tmp
//   Convert grayscale brightness b (0-255) → RGB565.
//   Formula: R=b>>3, G=b>>2, B=b>>3  → (R<<11)|(G<<5)|B
//   Clobbers: out, tmp.  b is also clobbered.
// ---------------------------------------------------------------------------
.macro B_TO_RGB565 out, b, tmp
    beqz    \b, .Lzero_\@
    srli    \tmp, \b, 3             // R = B channel (5-bit)
    srli    \b,   \b, 2             // G channel (6-bit)
    slli    \out, \tmp, 11          // R << 11
    slli    \b,   \b,   5
    andi    \b,   \b,   0x07E0      // G << 5, masked
    or      \out, \out, \b
    andi    \tmp, \tmp, 0x001F      // B, masked
    or      \out, \out, \tmp
    j       .Ldone_\@
.Lzero_\@:
    li      \out, 0
.Ldone_\@:
.endm

// ---------------------------------------------------------------------------
// draw_line_asm
// ---------------------------------------------------------------------------
draw_line_asm:
    // ----- PROLOGUE ---------------------------------------------------------
    // Save all callee-saved registers (s0-s11, ra).
    // We use s0..s10.  s11 saved for ABI correctness.
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
    // sp[0..7] padding

    // ----- REGISTER MAP (saved regs) ----------------------------------------
    // s0  = fb (framebuffer base pointer)
    // s1  = fb_w
    // s2  = fb_h
    // s3  = steep flag  (0 = normal, 1 = axes transposed)
    // s4  = half_w  = max(0, (thickness-1)/2)
    // s5  = brightness (0-255)
    // s6  = y_fp  (Q8 fixed-point y, advances each x step)
    // s7  = gradient (Q8 fixed-point, signed)
    // s8  = x  (major-axis loop counter, x0 → x1)
    // s9  = x1 (major-axis loop limit)
    // s10 = color_full (RGB565 for full brightness)

    // ----- LOAD ARGUMENTS ---------------------------------------------------
    mv      s0, a0          // fb
    mv      s1, a1          // fb_w
    mv      s2, a2          // fb_h
    mv      s5, a7          // brightness

    lw      t0, 60(sp)      // thickness (9th arg, on caller stack)
    // half_w = thickness / 2  (matches existing C: half_w = g_line_width / 2)
    // thickness=1 → half_w=0 (pure Wu 1-px), thickness=3 → half_w=1, etc.
    blt     t0, zero, .Lthick_clamp
    srli    s4, t0, 1       // s4 = thickness >> 1
    j       .Lthick_ok
.Lthick_clamp:
    li      s4, 0
.Lthick_ok:

    // ----- PRECOMPUTE color_full = brightness_to_rgb565(brightness) ---------
    mv      t3, s5
    mv      t4, t3          // keep a copy for B_TO_RGB565 (macro clobbers b)
    B_TO_RGB565 s10, t4, t5
    // s10 = full-brightness RGB565 color

    // ----- LOAD x0,y0,x1,y1 FROM ARGS (still in a3..a6) --------------------
    mv      t1, a3          // t1 = x0
    mv      t2, a4          // t2 = y0
    mv      t3, a5          // t3 = x1
    mv      t4, a6          // t4 = y1

    // ----- DETECT STEEP (|dy| > |dx|) and transpose if necessary -----------
    sub     t5, t3, t1      // dx = x1 - x0
    sub     t6, t4, t2      // dy = y1 - y0

    // |dx|
    mv      a0, t5
    bge     a0, zero, .Labs_dx_done
    neg     a0, a0
.Labs_dx_done:
    // |dy|
    mv      a1, t6
    bge     a1, zero, .Labs_dy_done
    neg     a1, a1
.Labs_dy_done:

    li      s3, 0
    ble     a1, a0, .Lno_steep
    li      s3, 1
    // Swap (x0,y0) and (x1,y1) so major axis becomes x
    mv      a2, t1;  mv t1, t2;  mv t2, a2   // swap x0 ↔ y0
    mv      a2, t3;  mv t3, t4;  mv t4, a2   // swap x1 ↔ y1
.Lno_steep:

    // ----- ENSURE x0 ≤ x1 --------------------------------------------------
    ble     t1, t3, .Lno_swap_pts
    mv      a2, t1;  mv t1, t3;  mv t3, a2
    mv      a2, t2;  mv t2, t4;  mv t4, a2
.Lno_swap_pts:

    // Loop bounds: s8 = x0,  s9 = x1
    mv      s8, t1
    mv      s9, t3

    // dx (always ≥ 0),  dy (signed)
    sub     t5, t3, t1      // dx
    sub     t6, t4, t2      // dy

    // ----- DEGENERATE CASE: dx == 0 → single point -------------------------
    // By construction (steep detection), dx=0 only when both dx_orig=dy_orig=0.
    bnez    t5, .Lhas_dx

    // Single point — determine screen coords
    beqz    s3, .Lpt_nosteep
    mv      a0, t2          // screen_x = t2 (was original y before transpose)
    mv      a1, t1          // screen_y = t1 (was original x before transpose)
    j       .Lpt_draw
.Lpt_nosteep:
    mv      a0, t1          // screen_x
    mv      a1, t2          // screen_y
.Lpt_draw:
    // Draw (2*half_w+1)² square centered on (a0, a1)
    neg     t5, s4          // dy_offset = -half_w
.Lpt_outer:
    bgt     t5, s4, .Ldone
    neg     t6, s4          // dx_offset = -half_w
.Lpt_inner:
    bgt     t6, s4, .Lpt_next_outer
    add     a2, a0, t6      // px
    add     a3, a1, t5      // py
    PUT_PIXEL s0, s1, s2, a2, a3, s10, a4
    addi    t6, t6, 1
    j       .Lpt_inner
.Lpt_next_outer:
    addi    t5, t5, 1
    j       .Lpt_outer

    // ----- MAIN CASE: compute gradient, enter x loop -----------------------
.Lhas_dx:
    // gradient = (dy << 8) / dx   (signed Q8 fixed-point)
    slli    t6, t6, 8
    div     s7, t6, t5      // s7 = gradient

    // y_fp = y0 << 8   (Q8)
    slli    s6, t2, 8

    // =========================================================================
    // X LOOP   for x = s8; x <= s9; ++x
    // =========================================================================
.Lx_loop:
    bgt     s8, s9, .Ldone

    // Extract integer and fractional y from Q8 fixed-point
    srai    t0, s6, 8       // t0 = y_int  (signed arithmetic shift)
    andi    t1, s6, 0xFF    // t1 = frac   (0..255)

    // -----------------------------------------------------------------
    // TOP AA EDGE  at (y_int - half_w)
    //   alpha_top = 255 - frac
    //   color_top = rgb565( brightness * alpha_top >> 8 )
    // -----------------------------------------------------------------
    li      t2, 255
    sub     t2, t2, t1          // t2 = alpha_top

    // scaled brightness (brightness × alpha >> 8)
    mul     t3, s5, t2
    srli    t3, t3, 8           // t3 = scaled_b_top

    // RGB565 conversion (B_TO_RGB565 clobbers t3 and t5; result in t5)
    mv      t4, t3
    B_TO_RGB565 t5, t4, t6

    // Screen coordinates for top edge
    sub     a0, t0, s4          // y_top = y_int - half_w
    beqz    s3, .Ltop_nosteep
    // steep: transposed → screen_x = y_top, screen_y = x
    PUT_PIXEL s0, s1, s2, a0, s8, t5, a3
    j       .Ltop_done
.Ltop_nosteep:
    PUT_PIXEL s0, s1, s2, s8, a0, t5, a3
.Ltop_done:

    // -----------------------------------------------------------------
    // CORE PIXELS  y_int - half_w + 1  to  y_int + half_w
    //   alpha = 255  →  use precomputed s10
    // -----------------------------------------------------------------
    sub     a0, t0, s4
    addi    a0, a0, 1           // a0 = core_y_start
    add     a1, t0, s4          // a1 = core_y_end

.Lcore_loop:
    bgt     a0, a1, .Lcore_done
    beqz    s3, .Lcore_nosteep
    PUT_PIXEL s0, s1, s2, a0, s8, s10, a4
    j       .Lcore_next
.Lcore_nosteep:
    PUT_PIXEL s0, s1, s2, s8, a0, s10, a4
.Lcore_next:
    addi    a0, a0, 1
    j       .Lcore_loop
.Lcore_done:

    // -----------------------------------------------------------------
    // BOTTOM AA EDGE  at (y_int + half_w + 1)
    //   alpha_bot = frac
    //   Skip entirely when frac == 0 (zero coverage)
    // -----------------------------------------------------------------
    beqz    t1, .Lbot_skip

    mul     t3, s5, t1
    srli    t3, t3, 8           // t3 = scaled_b_bot

    mv      t4, t3
    B_TO_RGB565 t5, t4, t6

    add     a0, t0, s4
    addi    a0, a0, 1           // y_bot = y_int + half_w + 1
    beqz    s3, .Lbot_nosteep
    PUT_PIXEL s0, s1, s2, a0, s8, t5, a3
    j       .Lbot_skip
.Lbot_nosteep:
    PUT_PIXEL s0, s1, s2, s8, a0, t5, a3
.Lbot_skip:

    // ----- ADVANCE ----------------------------------------------------------
    add     s6, s6, s7          // y_fp += gradient
    addi    s8, s8, 1           // x++
    j       .Lx_loop

    // =========================================================================
    // EPILOGUE
    // =========================================================================
.Ldone:
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

    .size draw_line_asm, . - draw_line_asm
