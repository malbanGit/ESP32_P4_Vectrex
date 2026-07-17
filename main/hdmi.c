/*
 * HDMI output bring-up via LT8912B (1280x720@60). The driver enables the HDMI
 * output by default during esp_lcd_panel_init(), so there is nothing to toggle.
 */
#include "hdmi.h"
#include "board.h"
#include "esp_check.h"
#include "esp_lcd_panel_ops.h"
#define ESP_LCD_LT8912B_VIDEO_TIMING_1280x720_60Hz_60() \
    {                               \
        .hfp            = 48,      \
        .hs             = 32,       \
        .hbp            = 80,      \
        .hact           = 1280,     \
        .htotal         = 1440,     \
        .vfp            = 21,        \
        .vs             = 5,        \
        .vbp            = 180,       \
        .vact           = 720,      \
        .vtotal         = 926,      \
        .h_polarity     = 1,        \
        .v_polarity     = 0,        \
        .vic            = 0,        \
        .aspect_ratio   = LT8912B_ASPECT_RATION_16_9, \
        .pclk_mhz       = 80,    \
    }

#define LT8912B_1280x720_PANEL_60HZ_DPI_CONFIG() LT8912B_1280x720_PANEL_60HZ_DPI_CONFIG_WITH_FBS(1)
#define LT8912B_1280x720_PANEL_60HZ_DPI_CONFIG_WITH_FBS_60(NUM_FBS) \
{                           \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,       \
        .dpi_clock_freq_mhz = 80,                          \
        .virtual_channel = 0,                              \
        .in_color_format = LCD_COLOR_PIXEL_FORMAT_RGB888,     \
        .num_fbs = NUM_FBS,                                      \
        .video_timing = {                                  \
            .h_size = 1280,                       \
            .v_size = 720,                       \
            .hsync_back_porch = 80,                        \
            .hsync_pulse_width = 32,                      \
            .hsync_front_porch = 48,                       \
            .vsync_back_porch = 180,                        \
            .vsync_pulse_width = 5,                        \
            .vsync_front_porch = 21,                        \
        },                                                 \
        .flags.disable_lp = true,                          \
    }

static const char *TAG = "hdmi";

esp_err_t hdmi_start(const esp_lcd_panel_lt8912b_io_t *io, esp_lcd_dsi_bus_handle_t dsi_bus,
                     bool yuv422, esp_lcd_panel_handle_t *out_panel, int *out_w, int *out_h)
{
//    static esp_lcd_dpi_panel_config_t dpi = LT8912B_1280x720_PANEL_60HZ_DPI_CONFIG_WITH_FBS(2) ; // 75 Hz
//    static esp_lcd_dpi_panel_config_t dpi = LT8912B_800x600_PANEL_60HZ_DPI_CONFIG_WITH_FBS(2) ; // 60Hz
//    static esp_lcd_dpi_panel_config_t dpi = LT8912B_1280x800_PANEL_60HZ_DPI_CONFIG_WITH_FBS(2) ; // 70Hz
//    static esp_lcd_dpi_panel_config_t dpi = LT8912B_1920x1080_PANEL_30HZ_DPI_CONFIG_WITH_FBS(2) ; // 36Hz
    static esp_lcd_dpi_panel_config_t dpi = LT8912B_1280x720_PANEL_60HZ_DPI_CONFIG_WITH_FBS_60(2) ; // 60Hz

    

    dpi.in_color_format  = yuv422 ? LCD_COLOR_FMT_YUV422 : LCD_COLOR_FMT_RGB888;
    dpi.out_color_format = LCD_COLOR_FMT_RGB888;   // LT8912B accepts RGB888 only

    
    lt8912b_vendor_config_t vc = {
//        .video_timing = ESP_LCD_LT8912B_VIDEO_TIMING_1280x720_60Hz(),
//        .video_timing = ESP_LCD_LT8912B_VIDEO_TIMING_800x600_60Hz(),
//        .video_timing = ESP_LCD_LT8912B_VIDEO_TIMING_1280x800_60Hz(),
//        .video_timing = ESP_LCD_LT8912B_VIDEO_TIMING_1920x1080_30Hz(),
        .video_timing = ESP_LCD_LT8912B_VIDEO_TIMING_1280x720_60Hz_60(),
        
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

    *out_panel = panel;
    *out_w = dpi.video_timing.h_size;
    *out_h = dpi.video_timing.v_size;
    ESP_LOGI(TAG, "HDMI %dx%d@60 up (%s frame buffer)", *out_w, *out_h, yuv422 ? "YUV422" : "RGB888");
    return ESP_OK;
}
