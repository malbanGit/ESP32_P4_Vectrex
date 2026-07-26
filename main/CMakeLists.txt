idf_component_register(
    SRCS "draw_line_yuv422_overlay.S" "draw_line_yuv422_color.S" "draw_line_yuv422_brightness.S" "draw_line_yuv422_brightness.c" "draw_line_yuv422_overlay.c" "draw_line_yuv422_color.c" "lvds.c" "main.c" "lodepng.c" "hdmi.c"
    INCLUDE_DIRS "." 
    REQUIRES esp_lcd fatfs sd_card vecx
    PRIV_REQUIRES esp_timer esp_psram esp_mm esp_codec_dev esp_driver_i2s esp_driver_gpio
)

set(CONFIG_USB_HOST_ENABLED y)
set(CONFIG_USB_HOST_HID_ENABLED y)
set(CONFIG_USB_HOST_HID_TASK_STACK_SIZE 4096)
