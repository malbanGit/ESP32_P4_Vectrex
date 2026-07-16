// overlay_fast_asm_yuv422.S  —  ESP32-P4 (RV32IMAC) fast overlay for YUV422
//
// Asm equivalents of overlay_fast_erase_line_yuv422() and
// overlay_fast_draw_line_yuv422() — identical signatures, intended for
// side-by-side benchmarking with the C versions.
//
// overlay_ctx_yuv422_t field offsets (must match overlay_fast_yuv422.h):
//   +0   uint8_t *dimmed
//   +4   const uint8_t *overlay
//   +8   int fb_w
//
// YUYV pixel layout:  byte at (y*fw + x)*2 is Y (luma);
//                     byte at (y*fw + x)*2+1 is U (x even) or V (x odd).
//
// Performance notes:
//   • Both PSRAM loads (dimmed and overlay) issued before arithmetic.
//   • Y: branchless saturating add.
//   • UV: signed chroma blend, then branchless clamp to [0,255].
//   • lhu/sh used for ERASE to copy both bytes in one instruction pair.

    .section .iram0.text, "ax"
    .align 4

// ---------------------------------------------------------------------------
// ERASE_PIXEL_YUV   a0=px  a1=py
//   s0=fb  s1=fw  s2=dimmed
//   Clobbers: t0 t1 t2
// ---------------------------------------------------------------------------
.macro ERASE_PIXEL_YUV
    // Bounds checks — commented out.
    // bltz  a0, .Lepyuv_skip_\@
    // bge   a0, s1, .Lepyuv_skip_\@
    // bltz  a1, .Lepyuv_skip_\@
    mul     t0, a1, s1
    add     t0, t0, a0              // pixel index
    slli    t0, t0, 1               // byte offset = index * 2
    add     t1, s2, t0              // ptr into dimmed
    add     t2, s0, t0              // ptr into fb
    lhu     t0, 0(t1)               // load Y+UV as 16-bit (both bytes, aligned)
    sh      t0, 0(t2)               // store both bytes to fb
.Lepyuv_skip_\@:
.endm

// ---------------------------------------------------------------------------
// DRAW_PIXEL_YUV   a0=px  a1=py  a2=alpha
//   s0=fb  s1=fw  s2=dimmed  s3=overlay  s11=anti_shine (=255-shine_through)
//   Clobbers: t0-t6, a3-a6
//   Preserves: a0, a1, a2, a7, all s* regs
//
// Y  beam: beam_Y  = ov_Y  + ((255-ov_Y) *anti_shine)>>8   (anti_shine=0→ov; =255→white)
// UV beam: beam_UV = ov_UV + ((128-ov_UV)*anti_shine)>>8   (neutral chroma for white)
// contrib_Y  = beam_Y * alpha >> 8
// contrib_UV = (beam_UV - 128) * alpha >> 8  [signed]
// out_Y  = saturate(bg_Y + contrib_Y, 0, 255)
// out_UV = clamp(128 + (bg_UV-128) + contrib_UV, 0, 255)
// ---------------------------------------------------------------------------
.macro DRAW_PIXEL_YUV
    // Bounds checks — commented out.
    // bltz  a0, .Ldpyuv_skip_\@
    // bge   a0, s1, .Ldpyuv_skip_\@
    // bltz  a1, .Ldpyuv_skip_\@
    mul     t0, a1, s1
    add     t0, t0, a0              // pixel index
    slli    t0, t0, 1               // byte offset = index * 2
    add     t1, s2, t0              // ptr into dimmed
    add     t4, s3, t0              // ptr into overlay — issue address early
    add     t5, s0, t0              // ptr into fb
    // Issue PSRAM loads early (overlap ~20-cycle latency)
    lbu     a3, 0(t1)               // bg_Y
    lbu     a4, 1(t1)               // bg_UV
    lbu     a6, 0(t4)               // ov_Y
    lbu     t6, 1(t4)               // ov_UV
    // beam_Y  = ov_Y  + ((255 - ov_Y)  * anti_shine) >> 8
    // beam_UV = ov_UV + ((128 - ov_UV) * anti_shine) >> 8
    beqz    s11, .Ldpyuv_is_ov_\@  // anti_shine=0 → beam = overlay directly
    li      t2, 255
    sub     t2, t2, a6              // 255 - ov_Y
    mul     t2, t2, s11
    srli    t2, t2, 8
    add     t2, a6, t2              // beam_Y
    li      t3, 128
    sub     t3, t3, t6              // 128 - ov_UV
    mul     t3, t3, s11
    srli    t3, t3, 8
    add     t3, t6, t3              // beam_UV
    j       .Ldpyuv_scale_\@
.Ldpyuv_is_ov_\@:
    mv      t2, a6                  // beam_Y  = ov_Y
    mv      t3, t6                  // beam_UV = ov_UV
.Ldpyuv_scale_\@:
    // contrib_Y = beam_Y * alpha >> 8
    mul     t2, t2, a2
    srli    t2, t2, 8
    // contrib_UV = (beam_UV - 128) * alpha >> 8  [signed]
    addi    t3, t3, -128
    mul     t3, t3, a2
    srai    t3, t3, 8               // signed shift
    // out_Y = saturating_add(bg_Y, contrib_Y):
    //   add; if bit 8+ set → OR all low bits → AND 0xFF
    add     t2, a3, t2
    srli    a3, t2, 8;  neg a3, a3;  or t2, t2, a3;  andi t2, t2, 0xFF
    // out_UV = clamp(128 + (bg_UV - 128) + contrib_UV, 0, 255)
    addi    a4, a4, -128            // bg_UV - 128
    add     a4, a4, t3              // + contrib_UV
    addi    a4, a4, 128             // + 128
    // clamp low (negative → 0): srai sign; xori(not); and
    srai    t3, a4, 31
    xori    t3, t3, -1
    and     a4, a4, t3
    // clamp high (>255 → 255): srli/neg/or/andi
    srli    t3, a4, 8;  neg t3, t3;  or a4, a4, t3;  andi a4, a4, 0xFF
    sb      t2, 0(t5)               // Y
    sb      a4, 1(t5)               // UV
.Ldpyuv_skip_\@:
.endm

// ===========================================================================
// overlay_fast_erase_line_asm_yuv422
//
// void overlay_fast_erase_line_asm_yuv422(const overlay_ctx_yuv422_t *ctx, // a0
//                                          uint8_t *fb,                    // a1
//                                          int x0, int y0,                 // a2 a3
//                                          int x1, int y1,                 // a4 a5
//                                          int thickness);                  // a6
//
// Frame: 48 bytes  — ra + s0-s8
// s0=fb  s1=fw  s2=dimmed  s3=steep  s4=half_w
// s5=x   s6=x1  s7=y_fp    s8=grad
// ===========================================================================
    .global overlay_fast_erase_line_asm_yuv422
    .type   overlay_fast_erase_line_asm_yuv422, @function
overlay_fast_erase_line_asm_yuv422:
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

    lw      s2, 0(a0)       // dimmed
    lw      s1, 8(a0)       // fb_w
    mv      s0, a1          // fb
    srli    s4, a6, 1       // half_w

    mv      t1, a2; mv t2, a3; mv t3, a4; mv t4, a5

    sub     t5, t3, t1; bge t5, zero, .Leyuv_adx; neg t5, t5
.Leyuv_adx:
    sub     t6, t4, t2; bge t6, zero, .Leyuv_ady; neg t6, t6
.Leyuv_ady:
    li      s3, 0
    ble     t6, t5, .Leyuv_nosteep
    li      s3, 1
    mv      a0, t1;  mv t1, t2;  mv t2, a0
    mv      a0, t3;  mv t3, t4;  mv t4, a0
.Leyuv_nosteep:

    ble     t1, t3, .Leyuv_noswap
    mv      a0, t1;  mv t1, t3;  mv t3, a0
    mv      a0, t2;  mv t2, t4;  mv t4, a0
.Leyuv_noswap:

    mv      s5, t1; mv s6, t3
    sub     t5, t3, t1; sub t6, t4, t2

    bnez    t5, .Leyuv_has_dx

    beqz    s3, .Leyuv_pt_ns
    mv      a0, t2; mv a1, t1; j .Leyuv_pt_d
.Leyuv_pt_ns:
    mv      a0, t1; mv a1, t2
.Leyuv_pt_d:
    neg     t5, s4
.Leyuv_pt_oy:
    bgt     t5, s4, .Leyuv_done
    neg     t6, s4
.Leyuv_pt_ox:
    bgt     t6, s4, .Leyuv_pt_oy_next
    sw      a0, 4(sp); sw a1, 0(sp)
    add     a0, a0, t6; add a1, a1, t5
    ERASE_PIXEL_YUV
    lw      a0, 4(sp); lw a1, 0(sp)
    addi    t6, t6, 1
    j       .Leyuv_pt_ox
.Leyuv_pt_oy_next:
    addi    t5, t5, 1
    j       .Leyuv_pt_oy

.Leyuv_has_dx:
    slli    t6, t6, 8
    div     s8, t6, t5
    slli    s7, t2, 8       // y_fp = y0 << 8

.Leyuv_xloop:
    bgt     s5, s6, .Leyuv_done

    srai    t0, s7, 8       // y_int

    // Erase strip from y_int-half_w to y_int+half_w
    sub     a1, t0, s4      // strip_y (loop counter; macro uses a1 as py)
    add     t2, t0, s4      // strip end

.Leyuv_strip:
    bgt     a1, t2, .Leyuv_strip_done
    beqz    s3, .Leyuv_strip_ns
    // steep: px=a1(strip_y), py=s5(x)
    mv      a0, a1
    sw      a1, 4(sp)
    mv      a1, s5
    ERASE_PIXEL_YUV
    lw      a1, 4(sp)
    addi    a1, a1, 1
    j       .Leyuv_strip
.Leyuv_strip_ns:
    // non-steep: px=s5(x), py=a1(strip_y)
    mv      a0, s5
    ERASE_PIXEL_YUV
    addi    a1, a1, 1
    j       .Leyuv_strip
.Leyuv_strip_done:

    add     s7, s7, s8
    addi    s5, s5, 1
    j       .Leyuv_xloop

.Leyuv_done:
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

    .size overlay_fast_erase_line_asm_yuv422, . - overlay_fast_erase_line_asm_yuv422

// ===========================================================================
// overlay_fast_draw_line_asm_yuv422
//
// void overlay_fast_draw_line_asm_yuv422(const overlay_ctx_yuv422_t *ctx, // a0
//                                         uint8_t *fb,                    // a1
//                                         int x0, int y0,                 // a2 a3
//                                         int x1, int y1,                 // a4 a5
//                                         int brightness,                 // a6
//                                         int thickness,                  // a7
//                                         int shine_through);             // 64(sp)
//
// Frame: 64 bytes  — ra + s0-s11
// s0=fb  s1=fw  s2=dimmed  s3=overlay  s4=steep  s5=half_w
// s6=brightness  s7=y_fp  s8=grad  s9=x  s10=x1  s11=anti_shine
// ===========================================================================
    .global overlay_fast_draw_line_asm_yuv422
    .type   overlay_fast_draw_line_asm_yuv422, @function
overlay_fast_draw_line_asm_yuv422:
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

    lw      s2, 0(a0)       // dimmed
    lw      s3, 4(a0)       // overlay
    lw      s1, 8(a0)       // fb_w
    mv      s0, a1          // fb
    mv      s6, a6          // brightness
    srli    s5, a7, 1       // half_w

    // anti_shine = clamp(255 - shine_through, 0, 255)
    lw      t0, 64(sp)
    li      t1, 255
    sub     s11, t1, t0
    bge     s11, zero, .Ldyuv_as_lo
    li      s11, 0
.Ldyuv_as_lo:
    li      t1, 255
    ble     s11, t1, .Ldyuv_as_hi
    li      s11, 255
.Ldyuv_as_hi:

    mv      t1, a2; mv t2, a3; mv t3, a4; mv t4, a5

    sub     t5, t3, t1; bge t5, zero, .Ldyuv_adx; neg t5, t5
.Ldyuv_adx:
    sub     t6, t4, t2; bge t6, zero, .Ldyuv_ady; neg t6, t6
.Ldyuv_ady:
    li      s4, 0
    ble     t6, t5, .Ldyuv_nosteep
    li      s4, 1
    mv      a0, t1;  mv t1, t2;  mv t2, a0
    mv      a0, t3;  mv t3, t4;  mv t4, a0
.Ldyuv_nosteep:

    ble     t1, t3, .Ldyuv_noswap
    mv      a0, t1;  mv t1, t3;  mv t3, a0
    mv      a0, t2;  mv t2, t4;  mv t4, a0
.Ldyuv_noswap:

    mv      s9,  t1; mv s10, t3
    sub     t5, t3, t1; sub t6, t4, t2

    bnez    t5, .Ldyuv_has_dx

    beqz    s4, .Ldyuv_pt_ns
    mv      a0, t2; mv a1, t1; j .Ldyuv_pt_d
.Ldyuv_pt_ns:
    mv      a0, t1; mv a1, t2
.Ldyuv_pt_d:
    neg     t5, s5
.Ldyuv_pt_oy:
    bgt     t5, s5, .Ldyuv_done
    neg     t6, s5
.Ldyuv_pt_ox:
    bgt     t6, s5, .Ldyuv_pt_oy_next
    sw      a0, 8(sp); sw a1, 4(sp)
    add     a0, a0, t6; add a1, a1, t5
    mv      a2, s6
    DRAW_PIXEL_YUV
    lw      a0, 8(sp); lw a1, 4(sp)
    addi    t6, t6, 1
    j       .Ldyuv_pt_ox
.Ldyuv_pt_oy_next:
    addi    t5, t5, 1
    j       .Ldyuv_pt_oy

.Ldyuv_has_dx:
    slli    t6, t6, 8
    div     s8, t6, t5
    slli    s7, t2, 8

    // =========================================================================
    // X LOOP
    // =========================================================================
.Ldyuv_xloop:
    bgt     s9, s10, .Ldyuv_done

    srai    t0, s7, 8       // y_int
    andi    t1, s7, 0xFF    // frac

    // --- TOP AA EDGE ---
    li      t2, 255
    sub     t2, t2, t1
    mul     t3, s6, t2
    srli    t3, t3, 8       // alpha_top

    sub     a0, t0, s5      // ytop = y_int - half_w
    mv      a2, t3
    beqz    s4, .Ldyuv_top_ns
    mv      a1, s9
    DRAW_PIXEL_YUV
    j       .Ldyuv_top_done
.Ldyuv_top_ns:
    mv      a1, a0
    mv      a0, s9
    DRAW_PIXEL_YUV
.Ldyuv_top_done:

    // --- CORE PIXELS ---
    srai    t0, s7, 8       // recompute y_int
    andi    t1, s7, 0xFF    // recompute frac
    sub     a0, t0, s5
    addi    a0, a0, 1       // core_start
    add     a1, t0, s5      // core_end

.Ldyuv_core:
    bgt     a0, a1, .Ldyuv_core_done
    mv      a2, s6
    beqz    s4, .Ldyuv_core_ns
    sw      a0, 8(sp); sw a1, 4(sp)
    mv      a1, s9          // steep: px=core_y, py=x
    DRAW_PIXEL_YUV
    lw      a0, 8(sp); lw a1, 4(sp)
    addi    a0, a0, 1
    j       .Ldyuv_core
.Ldyuv_core_ns:
    sw      a0, 8(sp); sw a1, 4(sp)
    mv      a1, a0
    mv      a0, s9          // non-steep: px=x, py=core_y
    DRAW_PIXEL_YUV
    lw      a0, 8(sp); lw a1, 4(sp)
    addi    a0, a0, 1
    j       .Ldyuv_core
.Ldyuv_core_done:

    // --- BOTTOM AA EDGE ---
    beqz    t1, .Ldyuv_bot_skip
    mul     t3, s6, t1
    srli    t3, t3, 8       // alpha_bot

    srai    t0, s7, 8
    add     a0, t0, s5
    addi    a0, a0, 1       // ybot = y_int + half_w + 1
    mv      a2, t3
    beqz    s4, .Ldyuv_bot_ns
    mv      a1, s9
    DRAW_PIXEL_YUV
    j       .Ldyuv_bot_skip
.Ldyuv_bot_ns:
    mv      a1, a0
    mv      a0, s9
    DRAW_PIXEL_YUV
.Ldyuv_bot_skip:

    add     s7, s7, s8
    addi    s9, s9, 1
    j       .Ldyuv_xloop

.Ldyuv_done:
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

    .size overlay_fast_draw_line_asm_yuv422, . - overlay_fast_draw_line_asm_yuv422
