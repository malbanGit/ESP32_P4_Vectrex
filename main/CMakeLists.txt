#vecx/e8910.c
idf_component_register(
    SRCS "draw_line_dist_yuv422_overlay.c" "draw_line_dist_rgb888_overlay.S" "main.c" "libayemu/ay8912.c" "draw_line_dist_yuv422.S" "draw_line_dist_rgb888.S" "draw_line_dist_rgb888.c" "draw_line_dist_rgb888_overlay.c" "lodepng.c" "hdmi.c" "lvds.c" "mire.c" vecx/e6809.c  vecx/vecx.c
    INCLUDE_DIRS "." vecx
    REQUIRES esp_lcd fatfs sd_card
    PRIV_REQUIRES esp_timer esp_psram esp_mm esp_codec_dev esp_driver_i2s esp_driver_gpio
)

set(CONFIG_USB_HOST_ENABLED y)
set(CONFIG_USB_HOST_HID_ENABLED y)
set(CONFIG_USB_HOST_HID_TASK_STACK_SIZE 4096)
