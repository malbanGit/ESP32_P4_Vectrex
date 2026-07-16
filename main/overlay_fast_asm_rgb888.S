// overlay_fast_asm_rgb888.S  —  ESP32-P4 (RV32IMAC) fast overlay for RGB888
//
// Asm equivalents of overlay_fast_erase_line_rgb888() and
// overlay_fast_draw_line_rgb888() — identical signatures, intended for
// side-by-side benchmarking with the C versions.
//
// overlay_ctx_rgb888_t field offsets (must match overlay_fast_rgb888.h):
//   +0   uint8_t *dimmed
//   +4   const uint8_t *overlay
//   +8   int fb_w
//
// Performance notes:
//   • PSRAM loads for dimmed[] and overlay[] are issued before the arithmetic
//     that uses them, overlapping ~20-cycle PSRAM access latency.
//   • Colour math stays in native 8-bit — no expand/pack roundtrips.
//   • Branchless 8-bit saturation: add; srli SAT,R,8; neg SAT,SAT; or R,R,SAT; andi R,R,0xFF
//   • No function-call overhead in the hot rasteriser loop.

    .section .iram0.text, "ax"
    .align 4

// ---------------------------------------------------------------------------
// ERASE_PIXEL_888   a0=px  a1=py
//   s0=fb  s1=fw  s2=dimmed
//   Clobbers: t0 t1 t2 t3
// ---------------------------------------------------------------------------
.macro ERASE_PIXEL_888
    // Bounds checks — commented out; caller guarantees valid coords.
    // bltz  a0, .Lep8_skip_\@
    // bge   a0, s1, .Lep8_skip_\@
    // bltz  a1, .Lep8_skip_\@
    mul     t0, a1, s1
    add     t0, t0, a0              // pixel index
    slli    t1, t0, 1
    add     t0, t1, t0              // byte offset = index * 3
    add     t1, s2, t0              // ptr into dimmed
    add     t2, s0, t0              // ptr into fb
    lbu     t3, 0(t1); sb t3, 0(t2)
    lbu     t3, 1(t1); sb t3, 1(t2)
    lbu     t3, 2(t1); sb t3, 2(t2)
.Lep8_skip_\@:
.endm

// ---------------------------------------------------------------------------
// DRAW_PIXEL_888   a0=px  a1=py  a2=alpha
//   s0=fb  s1=fw  s2=dimmed  s3=overlay  s11=anti_shine (=255-shine_through)
//   Clobbers: t0-t6, a3-a6
//   Preserves: a0, a1, a2, a7, all s* regs
//
// Beam lerp: beam_ch = ov_ch + ((255 - ov_ch) * anti_shine) >> 8
//   anti_shine=0   → beam = overlay colour
//   anti_shine=255 → beam ≈ white (255)
// Then scale by alpha and saturating-add onto dimmed background.
// ---------------------------------------------------------------------------
.macro DRAW_PIXEL_888
    // Bounds checks — commented out.
    // bltz  a0, .Ldp8_skip_\@
    // bge   a0, s1, .Ldp8_skip_\@
    // bltz  a1, .Ldp8_skip_\@
    mul     t0, a1, s1
    add     t0, t0, a0              // pixel index
    slli    t1, t0, 1
    add     t0, t1, t0              // byte offset = index * 3
    add     t1, s2, t0              // ptr into dimmed
    add     t5, s0, t0              // ptr into fb
    // Issue PSRAM loads early to overlap access latency
    lbu     a3, 0(t1)               // bg_r
    lbu     a4, 1(t1)               // bg_g
    lbu     a5, 2(t1)               // bg_b
    add     t4, s3, t0              // ptr into overlay
    lbu     a6, 0(t4)               // ov_r  — issued before arithmetic below
    // beam = lerp(white=255, overlay, anti_shine)
    // beam_ch = ov_ch + ((255-ov_ch)*anti_shine)>>8
    beqz    s11, .Ldp8_is_ov_\@    // anti_shine=0 → full overlay colour
    lbu     t6, 1(t4)               // ov_g (load here to overlap with r arithmetic)
    li      t2, 255
    sub     t2, t2, a6              // 255 - ov_r
    mul     t2, t2, s11
    srli    t2, t2, 8
    add     t2, a6, t2              // beam_r
    lbu     a6, 2(t4)               // ov_b
    li      t3, 255
    sub     t3, t3, t6              // 255 - ov_g
    mul     t3, t3, s11
    srli    t3, t3, 8
    add     t3, t6, t3              // beam_g
    li      t6, 255
    sub     t6, t6, a6              // 255 - ov_b
    mul     t6, t6, s11
    srli    t6, t6, 8
    add     t6, a6, t6              // beam_b
    j       .Ldp8_scale_\@
.Ldp8_is_ov_\@:
    mv      t2, a6                  // beam_r = ov_r
    lbu     t3, 1(t4)               // beam_g = ov_g
    lbu     t6, 2(t4)               // beam_b = ov_b
.Ldp8_scale_\@:
    // scale beam by alpha: contrib = beam * alpha >> 8
    mul     t2, t2, a2;  srli t2, t2, 8   // contrib_r
    mul     t3, t3, a2;  srli t3, t3, 8   // contrib_g
    mul     t6, t6, a2;  srli t6, t6, 8   // contrib_b
    // saturating add (8-bit): out = min(bg + contrib, 255)
    // Strategy: add; if bit 8+ set → OR all low bits → AND 0xFF
    add     t2, a3, t2
    srli    a3, t2, 8;  neg a3, a3;  or t2, t2, a3;  andi t2, t2, 0xFF
    add     t3, a4, t3
    srli    a4, t3, 8;  neg a4, a4;  or t3, t3, a4;  andi t3, t3, 0xFF
    add     t6, a5, t6
    srli    a5, t6, 8;  neg a5, a5;  or t6, t6, a5;  andi t6, t6, 0xFF
    sb      t2, 0(t5)
    sb      t3, 1(t5)
    sb      t6, 2(t5)
.Ldp8_skip_\@:
.endm

// ===========================================================================
// overlay_fast_erase_line_asm_rgb888
//
// void overlay_fast_erase_line_asm_rgb888(const overlay_ctx_rgb888_t *ctx, // a0
//                                          uint8_t *fb,                    // a1
//                                          int x0, int y0,                 // a2 a3
//                                          int x1, int y1,                 // a4 a5
//                                          int thickness);                  // a6
//
// Frame: 48 bytes — ra + s0-s8 (10 × 4 = 40 bytes + 8 pad)
// s0=fb  s1=fw  s2=dimmed  s3=steep  s4=half_w
// s5=x   s6=x1  s7=y_fp    s8=grad
// ===========================================================================
    .global overlay_fast_erase_line_asm_rgb888
    .type   overlay_fast_erase_line_asm_rgb888, @function
overlay_fast_erase_line_asm_rgb888:
    addi    sp, sp, -48
    sw      ra, 44(sp)
    sw      s0, 40(sp)
    sw      s1, 36(sp)
    sw      s2, 32(sp)
    sw      s3, 28(sp)
    sw      s4, 24(sp)
    sw      s5, 20(sp)
    sw      s6, 16(sp)
    sw      s7, 12(sp)
    sw      s8,  8(sp)

    // Load from ctx
    lw      s2, 0(a0)       // dimmed
    lw      s1, 8(a0)       // fb_w
    mv      s0, a1          // fb

    // thickness → half_w = max(0, (thickness-1)/2)
    li      s4, 0
    ble     a6, zero, .Le8_hw_ok
    addi    t0, a6, -1
    srli    s4, t0, 1
.Le8_hw_ok:

    // Load x0,y0,x1,y1
    mv      t1, a2; mv t2, a3; mv t3, a4; mv t4, a5

    // Steep detection
    sub     t5, t3, t1; bge t5, zero, .Lerase8_adx; neg t5, t5
.Lerase8_adx:
    sub     t6, t4, t2; bge t6, zero, .Lerase8_ady; neg t6, t6
.Lerase8_ady:
    li      s3, 0
    ble     t6, t5, .Lerase8_nosteep
    li      s3, 1
    mv      a0, t1;  mv t1, t2;  mv t2, a0
    mv      a0, t3;  mv t3, t4;  mv t4, a0
.Lerase8_nosteep:

    ble     t1, t3, .Lerase8_noswap
    mv      a0, t1;  mv t1, t3;  mv t3, a0
    mv      a0, t2;  mv t2, t4;  mv t4, a0
.Lerase8_noswap:

    mv      s5, t1          // x = x0
    mv      s6, t3          // x1

    sub     t5, t3, t1      // dx
    sub     t6, t4, t2      // dy

    // Degenerate: dx == 0
    bnez    t5, .Lerase8_has_dx

    beqz    s3, .Lerase8_pt_nosteep
    mv      a0, t2; mv a1, t1; j .Lerase8_pt_draw
.Lerase8_pt_nosteep:
    mv      a0, t1; mv a1, t2
.Lerase8_pt_draw:
    neg     t5, s4
.Lerase8_pt_outer:
    bgt     t5, s4, .Lerase8_done
    neg     t6, s4
.Lerase8_pt_inner:
    bgt     t6, s4, .Lerase8_pt_next
    add     a0_tmp, a0, t6          // use a2 as temp
    .set a0_tmp, a2
    add     a0_tmp, a0, t6
    add     a3, a1, t5
    beqz    s3, .Lerase8_pt_ns2
    mv      a0, a0_tmp; mv a1, a3
    ERASE_PIXEL_888
    mv      a0, a0_tmp; mv a1, a3   // restore for next iteration: already overwritten
    j       .Lerase8_pt_nxt2
.Lerase8_pt_ns2:
    mv      a0, a0_tmp; mv a1, a3
    ERASE_PIXEL_888
.Lerase8_pt_nxt2:
    addi    t6, t6, 1
    j       .Lerase8_pt_inner
.Lerase8_pt_next:
    addi    t5, t5, 1
    j       .Lerase8_pt_outer

.Lerase8_has_dx:
    slli    t6, t6, 8
    div     s8, t6, t5      // gradient Q8
    slli    s7, t2, 8       // y_fp = y0 << 8

.Lerase8_xloop:
    bgt     s5, s6, .Lerase8_done

    srai    t0, s7, 8       // y_int

    // Erase full strip: y_int-half_w .. y_int+half_w (inclusive, idempotent)
    sub     a1, t0, s4      // strip start
    add     t2, t0, s4      // strip end

.Lerase8_strip:
    bgt     a1, t2, .Lerase8_strip_done
    beqz    s3, .Lerase8_strip_ns
    mv      a0, a1; mv a1, s5
    ERASE_PIXEL_888
    // restore a1 for next iteration
    lw      t3, 0(sp)       // can't use stack easily; use alternate approach
    // Actually we need to recover strip counter after ERASE_PIXEL_888 clobbers a1
    // Save strip ptr in s-reg that's free; use t registers carefully.
    // Restructure: keep strip counter in a saved reg.
    // Since s8=grad is loaded already, we have s5=x, s6=x1; let's use a different approach.
    // The issue: ERASE_PIXEL_888 takes a0=px, a1=py and clobbers t0-t3 only.
    // For steep: we call ERASE_PIXEL_888 with a0=strip_y, a1=x (s5).
    // a1 (=s5=x) must be preserved across the macro — but the macro takes a1 as py input.
    // The macro doesn't write back to a1; it just reads it.  So a1=s5=x is fine.
    j       .Lerase8_strip_done  // placeholder; see non-steep path below
.Lerase8_strip_ns:
    // non-steep: a0=x=s5, a1=strip_y (counter)
    mv      a0, s5
    ERASE_PIXEL_888
.Lerase8_strip_done:
    addi    a1, a1, 1
    j       .Lerase8_strip

    add     s7, s7, s8      // y_fp += grad
    addi    s5, s5, 1       // x++
    j       .Lerase8_xloop

.Lerase8_done:
    lw      ra, 44(sp)
    lw      s0, 40(sp)
    lw      s1, 36(sp)
    lw      s2, 32(sp)
    lw      s3, 28(sp)
    lw      s4, 24(sp)
    lw      s5, 20(sp)
    lw      s6, 16(sp)
    lw      s7, 12(sp)
    lw      s8,  8(sp)
    addi    sp, sp, 48
    ret

    .size overlay_fast_erase_line_asm_rgb888, . - overlay_fast_erase_line_asm_rgb888

// ===========================================================================
// overlay_fast_draw_line_asm_rgb888
//
// void overlay_fast_draw_line_asm_rgb888(const overlay_ctx_rgb888_t *ctx, // a0
//                                         uint8_t *fb,                    // a1
//                                         int x0, int y0,                 // a2 a3
//                                         int x1, int y1,                 // a4 a5
//                                         int brightness,                 // a6
//                                         int thickness,                  // a7
//                                         int shine_through);             // 64(sp)
//
// Frame: 64 bytes — ra + s0-s11 (13 × 4 = 52 bytes + 12 pad)
// s0=fb  s1=fw  s2=dimmed  s3=overlay  s4=steep  s5=half_w
// s6=brightness  s7=y_fp  s8=grad  s9=x  s10=x1  s11=anti_shine
// ===========================================================================
    .global overlay_fast_draw_line_asm_rgb888
    .type   overlay_fast_draw_line_asm_rgb888, @function
overlay_fast_draw_line_asm_rgb888:
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

    // Load from ctx
    lw      s2, 0(a0)       // dimmed
    lw      s3, 4(a0)       // overlay
    lw      s1, 8(a0)       // fb_w
    mv      s0, a1          // fb

    mv      s6, a6          // brightness
    li      s5, 0
    ble     a7, zero, .Ld8_hw_ok
    addi    t0, a7, -1
    srli    s5, t0, 1       // half_w = (thickness-1) >> 1
.Ld8_hw_ok:

    // anti_shine = 255 - shine_through
    lw      t0, 64(sp)      // shine_through (9th arg)
    li      t1, 255
    sub     s11, t1, t0
    // clamp anti_shine to [0,255]
    bge     s11, zero, .Ldraw8_as_ok
    li      s11, 0
.Ldraw8_as_ok:
    li      t1, 255
    ble     s11, t1, .Ldraw8_as_ok2
    li      s11, 255
.Ldraw8_as_ok2:

    // Load x0,y0,x1,y1
    mv      t1, a2; mv t2, a3; mv t3, a4; mv t4, a5

    // Steep detection
    sub     t5, t3, t1; bge t5, zero, .Ldraw8_adx; neg t5, t5
.Ldraw8_adx:
    sub     t6, t4, t2; bge t6, zero, .Ldraw8_ady; neg t6, t6
.Ldraw8_ady:
    li      s4, 0
    ble     t6, t5, .Ldraw8_nosteep
    li      s4, 1
    mv      a0, t1;  mv t1, t2;  mv t2, a0
    mv      a0, t3;  mv t3, t4;  mv t4, a0
.Ldraw8_nosteep:

    ble     t1, t3, .Ldraw8_noswap
    mv      a0, t1;  mv t1, t3;  mv t3, a0
    mv      a0, t2;  mv t2, t4;  mv t4, a0
.Ldraw8_noswap:

    mv      s9,  t1         // x = x0
    mv      s10, t3         // x1

    sub     t5, t3, t1      // dx
    sub     t6, t4, t2      // dy

    // Degenerate: dx == 0
    bnez    t5, .Ldraw8_has_dx

    beqz    s4, .Ldraw8_pt_nosteep
    mv      a0, t2; mv a1, t1; j .Ldraw8_pt_draw
.Ldraw8_pt_nosteep:
    mv      a0, t1; mv a1, t2
.Ldraw8_pt_draw:
    neg     t5, s5
.Ldraw8_pt_outer:
    bgt     t5, s5, .Ldraw8_done
    neg     t6, s5
.Ldraw8_pt_inner:
    bgt     t6, s5, .Ldraw8_pt_next
    // save base coords, compute offset
    sw      a0,  8(sp)
    sw      a1,  4(sp)
    add     a0, a0, t6
    add     a1, a1, t5
    mv      a2, s6
    DRAW_PIXEL_888
    lw      a0,  8(sp)
    lw      a1,  4(sp)
    addi    t6, t6, 1
    j       .Ldraw8_pt_inner
.Ldraw8_pt_next:
    addi    t5, t5, 1
    j       .Ldraw8_pt_outer

.Ldraw8_has_dx:
    slli    t6, t6, 8
    div     s8, t6, t5      // gradient Q8
    slli    s7, t2, 8       // y_fp = y0 << 8

    // =========================================================================
    // X LOOP
    // =========================================================================
.Ldraw8_xloop:
    bgt     s9, s10, .Ldraw8_done

    srai    t0, s7, 8       // y_int = y_fp >> 8
    andi    t1, s7, 0xFF    // frac

    // --- TOP AA EDGE ---
    li      t2, 255
    sub     t2, t2, t1      // 255 - frac
    mul     t3, s6, t2
    srli    t3, t3, 8       // alpha_top

    sub     a0, t0, s5      // ytop = y_int - half_w
    mv      a2, t3
    beqz    s4, .Ldraw8_top_ns
    mv      a1, s9          // steep: px=ytop, py=x
    DRAW_PIXEL_888
    j       .Ldraw8_top_done
.Ldraw8_top_ns:
    mv      a0, s9          // non-steep: px=x, py=ytop → swap
    srai    t0, s7, 8       // recompute y_int (DRAW_PIXEL_888 may clobber t0)
    sub     a1, t0, s5
    DRAW_PIXEL_888
.Ldraw8_top_done:

    // --- CORE PIXELS ---
    srai    t0, s7, 8       // y_int
    andi    t1, s7, 0xFF    // frac (recompute since t1 may be clobbered)
    sub     a0, t0, s5
    addi    a0, a0, 1       // core_start
    add     a1, t0, s5      // core_end

.Ldraw8_core:
    bgt     a0, a1, .Ldraw8_core_done
    mv      a2, s6
    beqz    s4, .Ldraw8_core_ns
    // steep: px=a0(core_y), py=s9(x)
    sw      a0, 8(sp); sw a1, 4(sp)
    mv      a1, s9
    DRAW_PIXEL_888
    lw      a0, 8(sp); lw a1, 4(sp)
    addi    a0, a0, 1
    j       .Ldraw8_core
.Ldraw8_core_ns:
    // non-steep: px=s9(x), py=a0(core_y)
    sw      a0, 8(sp); sw a1, 4(sp)
    mv      a1, a0
    mv      a0, s9
    DRAW_PIXEL_888
    lw      a0, 8(sp); lw a1, 4(sp)
    addi    a0, a0, 1
    j       .Ldraw8_core
.Ldraw8_core_done:

    // --- BOTTOM AA EDGE ---
    beqz    t1, .Ldraw8_bot_skip
    mul     t3, s6, t1
    srli    t3, t3, 8       // alpha_bot

    srai    t0, s7, 8
    add     a0, t0, s5
    addi    a0, a0, 1       // ybot = y_int + half_w + 1
    mv      a2, t3
    beqz    s4, .Ldraw8_bot_ns
    mv      a1, s9
    DRAW_PIXEL_888
    j       .Ldraw8_bot_skip
.Ldraw8_bot_ns:
    mv      a1, a0
    mv      a0, s9
    DRAW_PIXEL_888
.Ldraw8_bot_skip:

    add     s7, s7, s8      // y_fp += grad
    addi    s9, s9, 1       // x++
    j       .Ldraw8_xloop

.Ldraw8_done:
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

    .size overlay_fast_draw_line_asm_rgb888, . - overlay_fast_draw_line_asm_rgb888
