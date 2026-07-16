idf_component_register(
    SRCS "main.c" "draw_line.S" "overlay_line.c" "overlay_fast.c" "overlay_fast_asm.S"
         "draw_line_dist_rgb888.c" "draw_line_dist_rgb888.S" "overlay_line_rgb888.c"
         "overlay_fast_rgb888.c" "overlay_fast_asm_rgb888.S"
         "draw_line_gauss_yuv422.S" "overlay_line_yuv422.c"
         "overlay_fast_yuv422.c" "overlay_fast_asm_yuv422.S"
         "lodepng.c"
         vecx/cart.c vecx/e8910.c vecx/e6809.c vecx/vecx.c
    INCLUDE_DIRS "." vecx
    REQUIRES esp_lcd esp32_p4_nano_custom
    PRIV_REQUIRES esp_timer fatfs esp_psram esp_mm esp_codec_dev
)
