idf_component_register(
    SRCS "main.c" vecx/cart.c vecx/e8910.c vecx/e6809.c  vecx/vecx.c
    INCLUDE_DIRS "." vecx
    REQUIRES esp_lcd esp32_p4_nano_custom
    PRIV_REQUIRES esp_timer fatfs esp_psram esp_mm esp_codec_dev
)
