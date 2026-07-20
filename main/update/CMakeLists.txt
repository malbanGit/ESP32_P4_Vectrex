idf_component_register(SRCS "lvds.c" "hdmi.c" "mire.c" "hdmi_lt8912b_main.c"
                       PRIV_REQUIRES esp_driver_gpio esp_driver_i2c esp_lcd
                       INCLUDE_DIRS ".")
