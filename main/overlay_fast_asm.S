// overlay_fast_asm.S  —  ESP32-P4 (RV32IMAC) fast overlay line rendering
//
// Asm equivalents of overlay_fast_erase_line() and overlay_fast_draw_line().
// Keep the C versions for comparison; select at runtime to benchmark.
//
// overlay_ctx_t field offsets (must match overlay_fast.h):
//   +0  uint16_t *dimmed
//   +4  uint16_t *overlay
//   +8  int       fb_w
//
// Performance notes vs C:
//   • PSRAM load latency hidden: lhu for dimmed[] and overlay[] are issued
//     before the arithmetic that uses them, overlapping ~20-cycle stalls.
//   • Colour math stays in 5/6-bit RGB565 space — no expand/pack roundtrips.
//   • No function-call overhead inside the hot rasteriser loop.

    .section .iram0.text, "ax"
    .align 4

// ---------------------------------------------------------------------------
// ERASE_PIXEL  a0=px  a1=py
//   s0=fb  s1=fw  s2=dimmed
//   Clobbers: t0 t1 t2
// ---------------------------------------------------------------------------
.macro ERASE_PIXEL
    // Bounds checks — commented out; caller guarantees valid coords.
    // bltz  a0, .Lskip_ep_\@
    // bge   a0, s1, .Lskip_ep_\@
    // bltz  a1, .Lskip_ep_\@
    mul     t0, a1, s1
    add     t0, t0, a0          // offset (uint16_t units)
    slli    t1, t0, 1           // byte offset
    add     t2, s2, t1
    lhu     t0, 0(t2)           // load dimmed[offset]  — PSRAM
    add     t2, s0, t1
    sh      t0, 0(t2)           // fb[offset] = dimmed[offset]
// .Lskip_ep_\@:
.endm

// ---------------------------------------------------------------------------
// DRAW_PIXEL  a0=px  a1=py  a2=alpha(0-255)
//   s0=fb  s1=fw  s2=dimmed  s3=overlay  s11=anti_shine(255-shine_through)
//   Clobbers: t0-t6  a3-a6
//   Preserves: a0 a1 a2 a7 s0-s11
// ---------------------------------------------------------------------------
.macro DRAW_PIXEL
    // Bounds checks — commented out; caller guarantees valid coords.
    // bltz  a0, .Lskip_dp_\@
    // bge   a0, s1, .Lskip_dp_\@
    // bltz  a1, .Lskip_dp_\@

    mul     t0, a1, s1
    add     t0, t0, a0          // t0 = offset
    slli    t1, t0, 1           // t1 = byte offset

    // Issue both PSRAM loads early to hide access latency
    add     t2, s2, t1          // &dimmed[offset]
    add     t3, s3, t1          // &overlay[offset]
    lhu     t4, 0(t2)           // t4 = bg   (PSRAM stall)
    lhu     t5, 0(t3)           // t5 = ov   (pipelined)

    // Extract bg channels in 5/6-bit space
    srli    a3, t4, 11          // a3 = bg_r5
    srli    a4, t4, 5
    andi    a4, a4, 0x3F        // a4 = bg_g6
    andi    a5, t4, 0x1F        // a5 = bg_b5

    // Extract overlay channels (must extract ov_b before overwriting t5)
    srli    t4, t5, 11          // t4 = ov_r5
    srli    t6, t5, 5
    andi    t6, t6, 0x3F        // t6 = ov_g6
    andi    t5, t5, 0x1F        // t5 = ov_b5

    // beam = lerp(white, overlay, shine_through)
    //      = overlay + (white-overlay)*anti_shine/256
    // beam_r5 = ov_r + (31-ov_r)*anti_shine >> 8
    li      a6, 31
    sub     a6, a6, t4
    mul     a6, a6, s11
    srli    a6, a6, 8
    add     t4, t4, a6          // t4 = beam_r5

    li      a6, 63
    sub     a6, a6, t6
    mul     a6, a6, s11
    srli    a6, a6, 8
    add     t6, t6, a6          // t6 = beam_g6

    li      a6, 31
    sub     a6, a6, t5
    mul     a6, a6, s11
    srli    a6, a6, 8
    add     t5, t5, a6          // t5 = beam_b5

    // Scale beam by AA alpha (a2)
    mul     t4, t4, a2
    srli    t4, t4, 8           // t4 = fr5
    mul     t6, t6, a2
    srli    t6, t6, 8           // t6 = fg6
    mul     t5, t5, a2
    srli    t5, t5, 8           // t5 = fb5

    // Saturating add with bg — branchless
    // If bg+beam > channel_max, result overflows into higher bit.
    // srli by channel width gives 1 on overflow; neg gives all-ones mask.
    add     t4, a3, t4          // bg_r + fr (0..62)
    srli    a6, t4, 5
    neg     a6, a6
    or      t4, t4, a6
    andi    t4, t4, 0x1F        // saturated r5

    add     t6, a4, t6          // bg_g + fg (0..126)
    srli    a6, t6, 6
    neg     a6, a6
    or      t6, t6, a6
    andi    t6, t6, 0x3F        // saturated g6

    add     t5, a5, t5          // bg_b + fb (0..62)
    srli    a6, t5, 5
    neg     a6, a6
    or      t5, t5, a6
    andi    t5, t5, 0x1F        // saturated b5

    // Pack RGB565 and write to framebuffer
    slli    t4, t4, 11
    slli    t6, t6, 5
    or      t4, t4, t6
    or      t4, t4, t5
    add     t2, s0, t1          // &fb[offset]
    sh      t4, 0(t2)

// .Lskip_dp_\@:
.endm

// ===========================================================================
// overlay_fast_erase_line_asm
//
//   void overlay_fast_erase_line_asm(
//       const overlay_ctx_t *ctx,   // a0
//       uint16_t            *fb,    // a1
//       int x0, int y0,             // a2, a3
//       int x1, int y1,             // a4, a5
//       int thickness)              // a6
//
// Saved registers:
//   s0=fb  s1=fw  s2=dimmed  s3=steep  s4=half_w
//   s5=x   s6=x1  s7=y_fp   s8=grad
//
// Stack frame (48 bytes):
//   sp+44: ra    sp+40..sp+8: s0-s8
//   sp+4:  scratch sy   sp+0: scratch sx   (for degenerate single-point case)
// ===========================================================================
    .global overlay_fast_erase_line_asm
    .type   overlay_fast_erase_line_asm, @function

overlay_fast_erase_line_asm:
    addi    sp,  sp, -48
    sw      ra,  44(sp)
    sw      s0,  40(sp)
    sw      s1,  36(sp)
    sw      s2,  32(sp)
    sw      s3,  28(sp)
    sw      s4,  24(sp)
    sw      s5,  20(sp)
    sw      s6,  16(sp)
    sw      s7,  12(sp)
    sw      s8,   8(sp)

    mv      s0, a1
    lw      s1, 8(a0)           // fb_w
    lw      s2, 0(a0)           // dimmed
    li      s4, 0
    ble     a6, zero, .Le_hw_ok
    addi    t0, a6, -1
    srli    s4, t0, 1           // half_w = (thickness-1) >> 1
.Le_hw_ok:

    // Steep detection
    sub     t0, a4, a2          // dx
    sub     t1, a5, a3          // dy
    mv      t2, t0;  bge t2, zero, .Le_adx;  neg t2, t2
.Le_adx:
    mv      t3, t1;  bge t3, zero, .Le_ady;  neg t3, t3
.Le_ady:
    li      s3, 0
    ble     t3, t2, .Le_nosteep
    li      s3, 1
    mv      t4, a2;  mv a2, a3;  mv a3, t4   // swap x0↔y0
    mv      t4, a4;  mv a4, a5;  mv a5, t4   // swap x1↔y1
.Le_nosteep:

    // Ensure x0 ≤ x1
    ble     a2, a4, .Le_noswap
    mv      t4, a2;  mv a2, a4;  mv a4, t4
    mv      t4, a3;  mv a3, a5;  mv a5, t4
.Le_noswap:

    mv      s5, a2              // x = x0
    mv      s6, a4              // x1
    sub     t0, a4, a2          // dx
    sub     t1, a5, a3          // dy

    // Degenerate: dx == 0 → restore (2*half_w+1)² square
    bnez    t0, .Le_has_dx
    beqz    s3, .Le_pt_ns
    sw      a3, 0(sp)           // sx = y0 (transposed)
    sw      a2, 4(sp)           // sy = x0
    j       .Le_pt_draw
.Le_pt_ns:
    sw      a2, 0(sp)           // sx = x0
    sw      a3, 4(sp)           // sy = y0
.Le_pt_draw:
    neg     t5, s4              // row_off = -half_w
.Le_pt_outer:
    bgt     t5, s4, .Le_done
    neg     t6, s4
.Le_pt_inner:
    bgt     t6, s4, .Le_pt_next_outer
    lw      a0, 0(sp)
    lw      a1, 4(sp)
    add     a0, a0, t6          // px = sx + col_off
    add     a1, a1, t5          // py = sy + row_off
    ERASE_PIXEL
    addi    t6, t6, 1
    j       .Le_pt_inner
.Le_pt_next_outer:
    addi    t5, t5, 1
    j       .Le_pt_outer

    // Main case: Q8 fixed-point rasteriser
.Le_has_dx:
    slli    t1, t1, 8
    div     s8, t1, t0          // grad = (dy<<8)/dx  (Q8, signed)
    slli    s7, a3, 8           // y_fp = y0 << 8

.Le_x_loop:
    bgt     s5, s6, .Le_done

    srai    t0, s7, 8           // t0 = y_int
    andi    t1, s7, 0xFF        // t1 = frac (not needed for erase, but computes positions)

    // Top AA pixel and core strip all get erased
    // Erase y_int-half_w .. y_int+half_w+1 (the full AA extent)
    sub     a7, t0, s4          // a7 = y_int - half_w  (first pixel row)
    add     t2, t0, s4
    addi    t2, t2, 1           // t2 = y_int + half_w + 1  (last pixel row)

.Le_col_loop:
    bgt     a7, t2, .Le_col_done
    beqz    s3, .Le_col_ns
    mv      a0, a7              // steep: px = row index
    mv      a1, s5              //        py = x
    j       .Le_col_draw
.Le_col_ns:
    mv      a0, s5              // px = x
    mv      a1, a7              // py = row index
.Le_col_draw:
    ERASE_PIXEL
    addi    a7, a7, 1
    j       .Le_col_loop
.Le_col_done:

    add     s7, s7, s8          // y_fp += grad
    addi    s5, s5, 1           // x++
    j       .Le_x_loop

.Le_done:
    lw      ra,  44(sp)
    lw      s0,  40(sp)
    lw      s1,  36(sp)
    lw      s2,  32(sp)
    lw      s3,  28(sp)
    lw      s4,  24(sp)
    lw      s5,  20(sp)
    lw      s6,  16(sp)
    lw      s7,  12(sp)
    lw      s8,   8(sp)
    addi    sp,  sp,  48
    ret

    .size overlay_fast_erase_line_asm, . - overlay_fast_erase_line_asm

// ===========================================================================
// overlay_fast_draw_line_asm
//
//   void overlay_fast_draw_line_asm(
//       const overlay_ctx_t *ctx,   // a0
//       uint16_t            *fb,    // a1
//       int x0, int y0,             // a2, a3
//       int x1, int y1,             // a4, a5
//       int brightness,             // a6
//       int thickness,              // a7
//       int shine_through)          // 64(sp) — 9th arg, on caller stack
//
// Saved registers:
//   s0=fb   s1=fw    s2=dimmed  s3=overlay  s4=steep  s5=half_w
//   s6=bright  s7=y_fp  s8=grad  s9=x  s10=x1  s11=anti_shine
//
// Stack frame (64 bytes):
//   sp+60: ra    sp+56..sp+12: s0-s11
//   sp+8..sp+0:  scratch  (sx=sp+0, sy=sp+4)
//   Caller's 9th arg is at sp+64 after prologue.
// ===========================================================================
    .global overlay_fast_draw_line_asm
    .type   overlay_fast_draw_line_asm, @function

overlay_fast_draw_line_asm:
    addi    sp,  sp, -64
    sw      ra,  60(sp)
    sw      s0,  56(sp)
    sw      s1,  52(sp)
    sw      s2,  48(sp)
    sw      s3,  44(sp)
    sw      s4,  40(sp)
    sw      s5,  36(sp)
    sw      s6,  32(sp)
    sw      s7,  28(sp)
    sw      s8,  24(sp)
    sw      s9,  20(sp)
    sw      s10, 16(sp)
    sw      s11, 12(sp)

    mv      s0, a1              // fb
    lw      s1, 8(a0)           // fb_w
    lw      s2, 0(a0)           // dimmed
    lw      s3, 4(a0)           // overlay
    mv      s6, a6              // brightness
    srli    s5, a7, 1           // half_w = thickness >> 1

    // anti_shine = 255 - shine_through
    lw      t0, 64(sp)          // shine_through (9th arg)
    li      t1, 255
    sub     s11, t1, t0         // s11 = anti_shine

    // Steep detection
    sub     t0, a4, a2          // dx
    sub     t1, a5, a3          // dy
    mv      t2, t0;  bge t2, zero, .Ld_adx;  neg t2, t2
.Ld_adx:
    mv      t3, t1;  bge t3, zero, .Ld_ady;  neg t3, t3
.Ld_ady:
    li      s4, 0
    ble     t3, t2, .Ld_nosteep
    li      s4, 1
    mv      t4, a2;  mv a2, a3;  mv a3, t4
    mv      t4, a4;  mv a4, a5;  mv a5, t4
.Ld_nosteep:

    // Ensure x0 ≤ x1
    ble     a2, a4, .Ld_noswap
    mv      t4, a2;  mv a2, a4;  mv a4, t4
    mv      t4, a3;  mv a3, a5;  mv a5, t4
.Ld_noswap:

    mv      s9,  a2             // x = x0
    mv      s10, a4             // x1
    sub     t0, a4, a2          // dx
    sub     t1, a5, a3          // dy

    // Degenerate: dx == 0 → filled square at full brightness
    bnez    t0, .Ld_has_dx
    beqz    s4, .Ld_pt_ns
    sw      a3, 0(sp)           // sx = y0 (transposed)
    sw      a2, 4(sp)           // sy = x0
    j       .Ld_pt_draw
.Ld_pt_ns:
    sw      a2, 0(sp)           // sx
    sw      a3, 4(sp)           // sy
.Ld_pt_draw:
    neg     t5, s5
.Ld_pt_outer:
    bgt     t5, s5, .Ld_done
    neg     t6, s5
.Ld_pt_inner:
    bgt     t6, s5, .Ld_pt_next_outer
    lw      a0, 0(sp)
    lw      a1, 4(sp)
    add     a0, a0, t6
    add     a1, a1, t5
    mv      a2, s6              // alpha = brightness
    DRAW_PIXEL
    addi    t6, t6, 1
    j       .Ld_pt_inner
.Ld_pt_next_outer:
    addi    t5, t5, 1
    j       .Ld_pt_outer

    // Main Wu AA rasteriser
.Ld_has_dx:
    slli    t1, t1, 8
    div     s8, t1, t0          // grad = (dy<<8)/dx  (Q8)
    slli    s7, a3, 8           // y_fp = y0<<8

.Ld_x_loop:
    bgt     s9, s10, .Ld_done

    srai    t0, s7, 8           // t0 = y_int
    andi    t1, s7, 0xFF        // t1 = frac

    // ----- Top AA edge: alpha = brightness*(255-frac)>>8 -----
    li      a2, 255
    sub     a2, a2, t1
    mul     a2, s6, a2
    srli    a2, a2, 8           // a2 = alpha_top

    sub     a1, t0, s5          // a1 = y_int - half_w  (py for non-steep)
    beqz    s4, .Ld_top_ns
    mv      a0, a1              // steep: px=y_top
    mv      a1, s9              //        py=x
    j       .Ld_top_draw
.Ld_top_ns:
    mv      a0, s9              // px=x
                                // a1 already = y_top
.Ld_top_draw:
    DRAW_PIXEL

    // ----- Core pixels: y_int-half_w+1 .. y_int+half_w, alpha=brightness -----
    // Recompute y_int from s7 (DRAW_PIXEL clobbered t0)
    srai    t0, s7, 8           // t0 = y_int
    sub     a7, t0, s5
    addi    a7, a7, 1           // a7 = core_y_start
    add     t2, t0, s5          // t2 = core_y_end  (recompute after each pixel)
    mv      a2, s6              // alpha = brightness (constant for core)

.Ld_core_loop:
    // Recompute end (t2 clobbered by DRAW_PIXEL)
    srai    t2, s7, 8
    add     t2, t2, s5          // core_y_end = y_int + half_w
    bgt     a7, t2, .Ld_core_done
    beqz    s4, .Ld_core_ns
    mv      a0, a7              // steep: px=c
    mv      a1, s9              //        py=x
    j       .Ld_core_draw
.Ld_core_ns:
    mv      a0, s9              // px=x
    mv      a1, a7              // py=c
.Ld_core_draw:
    DRAW_PIXEL                  // clobbers t0-t6, a3-a6; preserves a7, s*
    addi    a7, a7, 1           // c++
    j       .Ld_core_loop
.Ld_core_done:

    // ----- Bottom AA edge: alpha = brightness*frac>>8, skip if frac==0 -----
    andi    t1, s7, 0xFF        // recompute frac from s7
    beqz    t1, .Ld_bot_skip
    mul     a2, s6, t1
    srli    a2, a2, 8           // a2 = alpha_bot

    srai    t0, s7, 8           // y_int
    add     a1, t0, s5
    addi    a1, a1, 1           // a1 = y_int + half_w + 1
    beqz    s4, .Ld_bot_ns
    mv      a0, a1              // steep: px=y_bot
    mv      a1, s9              //        py=x
    j       .Ld_bot_draw
.Ld_bot_ns:
    mv      a0, s9              // px=x
                                // a1 already = y_bot
.Ld_bot_draw:
    DRAW_PIXEL

.Ld_bot_skip:
    add     s7, s7, s8          // y_fp += grad
    addi    s9, s9, 1           // x++
    j       .Ld_x_loop

.Ld_done:
    lw      ra,  60(sp)
    lw      s0,  56(sp)
    lw      s1,  52(sp)
    lw      s2,  48(sp)
    lw      s3,  44(sp)
    lw      s4,  40(sp)
    lw      s5,  36(sp)
    lw      s6,  32(sp)
    lw      s7,  28(sp)
    lw      s8,  24(sp)
    lw      s9,  20(sp)
    lw      s10, 16(sp)
    lw      s11, 12(sp)
    addi    sp,  sp,  64
    ret

    .size overlay_fast_draw_line_asm, . - overlay_fast_draw_line_asm
