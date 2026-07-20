/*
 * HDMI output bring-up via LT8912B (1280x720@60). The driver enables the HDMI
 * output by default during esp_lcd_panel_init(), so there is nothing to toggle.
 */
#include "hdmi.h"
#include "board.h"
#include "esp_check.h"
#include "esp_lcd_panel_ops.h"

static const char *TAG = "hdmi";

esp_err_t hdmi_start(const esp_lcd_panel_lt8912b_io_t *io, esp_lcd_dsi_bus_handle_t dsi_bus,
                     bool yuv422, esp_lcd_panel_handle_t *out_panel, int *out_w, int *out_h)
{
    static esp_lcd_dpi_panel_config_t dpi = LT8912B_1280x720_PANEL_60HZ_DPI_CONFIG();
    dpi.in_color_format  = yuv422 ? LCD_COLOR_FMT_YUV422 : LCD_COLOR_FMT_RGB888;
    dpi.out_color_format = LCD_COLOR_FMT_RGB888;   // LT8912B accepts RGB888 only

    lt8912b_vendor_config_t vc = {
        .video_timing = ESP_LCD_LT8912B_VIDEO_TIMING_1280x720_60Hz(),
        .mipi_config = { .dsi_bus = dsi_bus, .dpi_config = &dpi, .lane_num = 2 },
    };
    esp_lcd_panel_dev_config_t dev = {
        .reset_gpio_num = -1,   // GPIO23 is the LCD reset, not the bridge; bridge uses power-on reset
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 24,
        .vendor_config = &vc,
    };

    esp_lcd_panel_handle_t panel = NULL;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_lt8912b(io, &dev, &panel), TAG, "new panel");
    if (yuv422) {
        esp_lcd_color_conv_config_t cc = {
            .in_color_range = LCD_COLOR_RANGE_FULL,
            .out_color_range = LCD_COLOR_RANGE_FULL,
            .spec.yuv = {
                .conv_std = LCD_YUV_CONV_STD_BT601,
                .yuv422.in_pack_order = LCD_YUV422_PACK_ORDER_YUYV,
            },
        };
        ESP_RETURN_ON_ERROR(esp_lcd_dpi_panel_set_color_conversion(panel, &cc), TAG, "yuv conv");
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(panel), TAG, "reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(panel), TAG, "init");  // driver enables HDMI

    // Restore the MIPI RX channel mapping default: lvds.c programs 0x49:0x40 to
    // 0x10 for an RGB888 frame buffer (see LVDS_BRINGUP.md 1.4), and the value
    // would otherwise survive an LVDS -> HDMI toggle. HDMI was validated with 0.
    {
        uint8_t swap = 0x00;
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io->cec_dsi, 0x40, &swap, 1), TAG, "rgb swap");
    }

    *out_panel = panel;
    *out_w = 1280;
    *out_h = 720;
    ESP_LOGI(TAG, "HDMI 1280x720@60 up (%s frame buffer)", yuv422 ? "YUV422" : "RGB888");
    return ESP_OK;
}
