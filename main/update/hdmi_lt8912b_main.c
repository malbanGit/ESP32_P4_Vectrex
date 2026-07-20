/*
 * Vectrex P4 board — video bring-up (gate #2 / #3): HDMI or LVDS via LT8912B.
 *
 * The VIDEO_MODE switch (GPIO28) selects the output at boot:
 *   HIGH -> HDMI 1280x720@60      LOW -> LVDS 800x480@60 (ST7262)
 * A self-generated test card (mire.c) proves the active video path. The
 * frame-buffer pixel format (RGB888 / YUV422) is the build option in board.h.
 *
 * This file only does board/common bring-up and dispatch; the per-output setup
 * lives in hdmi.c / lvds.c, the test pattern in mire.c (merge-ready layout).
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_ldo_regulator.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_lt8912b.h"

#include "board.h"
#include "mire.h"
#include "hdmi.h"
#include "lvds.h"

static const char *TAG = "video";

typedef enum { VIDEO_OUT_HDMI, VIDEO_OUT_LVDS } video_out_t;
#define VIDEO_OUT_SELECTED VIDEO_OUT_LVDS

// Diagnostic: 1 = output the P4 DPI's built-in vertical color bars instead of our
// frame buffer. Rules out a black/buggy frame buffer and tests the exact
// P4 DPI -> DSI -> bridge -> LVDS data path with known-good pixels. 0 = normal.
#define VIDEO_DPI_TEST_PATTERN  0   // EXPERIMENT #6: DW-DSI VPG pattern -> does the
                                    // bridge count 496 lines with canonical timing?


// VCV_NOE asserted = SN74AVC4T245 translator on (I2S + HPD). Harmless for video,
// kept consistent and needed later for I2S/HPD.
static void board_assert_vcv_noe(void)
{
    gpio_set_level(BOARD_PIN_VCV_NOE, 0);   // pre-load latch before enabling driver
    gpio_config_t io = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << BOARD_PIN_VCV_NOE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_LOGI(TAG, "VCV_NOE asserted (level translator on)");
}

// VDD_MIPI_DPHY must be 2.5 V; on the P4 it comes from internal LDO_VO3.
static void board_enable_dsi_phy_power(void)
{
    esp_ldo_channel_handle_t phy_ldo = NULL;
    esp_ldo_channel_config_t cfg = {
        .chan_id = 3,            // LDO_VO3 -> VDD_MIPI_DPHY
        .voltage_mv = 2500,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&cfg, &phy_ldo));
    ESP_LOGI(TAG, "MIPI DSI D-PHY power on (2.5 V)");
}

static const char *mode_name(video_out_t m) { return m == VIDEO_OUT_LVDS ? "LVDS" : "HDMI"; }

// HDMI Hot-Plug Detect: LT8912B main bank reg 0xC1 bit7 = sink connected (5 V on HPD).
// Same read the driver's internal get_hpd does; we poll it directly off the main io.
static bool hdmi_hpd_present(const esp_lcd_panel_lt8912b_io_t *io)
{
    uint8_t data = 0;
    if (esp_lcd_panel_io_rx_param(io->main, 0xc1, &data, 1) != ESP_OK) return false;
    return (data & 0x80) != 0;
}

// Bring up the selected output, allocate the frame buffer, draw the test card.
static esp_err_t video_start(video_out_t mode, bool yuv422,
                             const esp_lcd_panel_lt8912b_io_t *io, esp_lcd_dsi_bus_handle_t dsi,
                             esp_lcd_panel_handle_t *panel, uint8_t **fb, int *w, int *h)
{
    esp_err_t err = (mode == VIDEO_OUT_LVDS)
                    ? lvds_start(io, dsi, yuv422, panel, w, h)
                    : hdmi_start(io, dsi, yuv422, panel, w, h);
    if (err != ESP_OK) return err;
    if (mode == VIDEO_OUT_HDMI) lvds_backlight(false);   // LVDS panel unused -> backlight off

    size_t sz = (size_t)(*w) * (*h) * VIDEO_FB_BPP;
    *fb = heap_caps_malloc(sz, MALLOC_CAP_SPIRAM);
    ESP_RETURN_ON_FALSE(*fb, ESP_ERR_NO_MEM, TAG, "fb alloc (%u bytes) failed", (unsigned)sz);
    ESP_RETURN_ON_ERROR(mire_render(*fb, *w, *h, yuv422, mode_name(mode)), TAG, "mire");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_draw_bitmap(*panel, 0, 0, *w, *h, *fb), TAG, "draw");
    ESP_LOGI(TAG, "%s up: test card %dx%d (%s)", mode_name(mode), *w, *h, yuv422 ? "YUV422" : "RGB888");
#if LVDS_SETTLE_SWEEP
    if (mode == VIDEO_OUT_LVDS) lvds_sweep_settle(io);   // diagnostic: find the right settle
#endif
    return ESP_OK;
}

// Tear down the current output so the other one can be brought up cleanly.
static void video_stop(esp_lcd_panel_handle_t panel, uint8_t *fb)
{
    if (panel) esp_lcd_panel_del(panel);   // also deletes the inner DPI panel
    if (fb) heap_caps_free(fb);
}

void app_main(void)
{
    esp_log_level_set("lcd.dsi.dpi", ESP_LOG_DEBUG);   // show the actual DPI pixel clock achieved
    board_assert_vcv_noe();
    board_enable_dsi_phy_power();

    // 1) I2C master bus (same pins/driver as the i2c_tools bring-up project).
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t i2c_bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = BOARD_I2C_PORT,
        .sda_io_num = BOARD_I2C_SDA,
        .scl_io_num = BOARD_I2C_SCL,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus));

    // 2) The three LT8912B I2C register banks (main / cec-dsi / avi).
    esp_lcd_panel_lt8912b_io_t lt_io = {0};
    esp_lcd_panel_io_i2c_config_t io_main_cfg = LT8912B_IO_CFG(LT8912B_I2C_HZ, LT8912B_IO_I2C_MAIN_ADDRESS);
    esp_lcd_panel_io_i2c_config_t io_cec_cfg  = LT8912B_IO_CFG(LT8912B_I2C_HZ, LT8912B_IO_I2C_CEC_ADDRESS);
    esp_lcd_panel_io_i2c_config_t io_avi_cfg  = LT8912B_IO_CFG(LT8912B_I2C_HZ, LT8912B_IO_I2C_AVI_ADDRESS);
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_main_cfg, &lt_io.main));
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_cec_cfg,  &lt_io.cec_dsi));
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_avi_cfg,  &lt_io.avi));

    // 3) MIPI-DSI bus, 2 lanes. FINDING (2026-07-19): at the driver's default
    //    1000 Mbps/lane the LT8912B MIPI RX is marginal (picture stable only ~20%
    //    of the time); at 800 Mbps it is rock stable. Override here in project
    //    code -- do NOT edit the managed component header (hash-checked, gets
    //    overwritten on component updates).
    esp_lcd_dsi_bus_handle_t dsi_bus = NULL;
    esp_lcd_dsi_bus_config_t dsi_bus_cfg = LT8912B_PANEL_BUS_DSI_2CH_CONFIG();
    dsi_bus_cfg.lane_bit_rate_mbps = BOARD_DSI_LANE_MBPS;   // single source: board.h
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&dsi_bus_cfg, &dsi_bus));

    // 4) VIDEO_MODE switch: momentary tactile button, reads 1 when released.
    //    Used as a toggle. Default output at boot = LVDS.
    gpio_config_t btn = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << BOARD_PIN_VIDEO_MODE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&btn));

    const bool yuv422 = VIDEO_FB_YUV422;
    video_out_t mode = VIDEO_OUT_SELECTED;          // default at boot
    esp_lcd_panel_handle_t panel = NULL;
    uint8_t *fb = NULL;
    int w = 0, h = 0;
    ESP_ERROR_CHECK(video_start(mode, yuv422, &lt_io, dsi_bus, &panel, &fb, &w, &h));

    // The DPI HW pattern overrides the frame buffer when set, so force it to the
    // right state explicitly: bars only when diagnosing, otherwise NONE (= our mire).
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_set_pattern(panel,
                    VIDEO_DPI_TEST_PATTERN ? MIPI_DSI_PATTERN_BAR_VERTICAL : MIPI_DSI_PATTERN_NONE));
    ESP_LOGI(TAG, "DPI pattern: %s", VIDEO_DPI_TEST_PATTERN ? "vertical bars (diag)" : "NONE (frame buffer)");
#if VIDEO_DPI_TEST_PATTERN
    // Re-run the DDS/line-count diagnostic now that the DW VPG generates the
    // timing: if V-detect goes 484 -> 496 and the servo settles at 26.667 MHz, the
    // missing vblank HSS come from the P4's internal DPI adapter, not the DW host.
    if (mode == VIDEO_OUT_LVDS) lvds_read_dds(&lt_io,false);
#endif

    // 5) Toggle loop: a button press (level 1->0) switches HDMI <-> LVDS by
    //    tearing down and rebuilding the pipeline for the other output.
    int prev = 1;
    TickType_t last_hpd = xTaskGetTickCount();
    while (1) {
        // HDMI Hot-Plug Detect: poll ~every 5 s, log only while HDMI is the active output.
        if (mode == VIDEO_OUT_HDMI && (xTaskGetTickCount() - last_hpd) >= pdMS_TO_TICKS(5000)) {
            last_hpd = xTaskGetTickCount();
            ESP_LOGI(TAG, "HDMI HPD: %s", hdmi_hpd_present(&lt_io) ? "connected" : "no sink");
        }

        int lvl = gpio_get_level(BOARD_PIN_VIDEO_MODE);
        if (prev == 1 && lvl == 0) {                       // press edge
            vTaskDelay(pdMS_TO_TICKS(30));                 // debounce
            if (gpio_get_level(BOARD_PIN_VIDEO_MODE) == 0) 
            {
                mode = (mode == VIDEO_OUT_LVDS) ? VIDEO_OUT_HDMI : VIDEO_OUT_LVDS;
                ESP_LOGI(TAG, "VIDEO_MODE pressed -> switching to %s", mode_name(mode));
                video_stop(panel, fb);
                panel = NULL; fb = NULL;
                ESP_ERROR_CHECK(video_start(mode, yuv422, &lt_io, dsi_bus, &panel, &fb, &w, &h));
                while (gpio_get_level(BOARD_PIN_VIDEO_MODE) == 0) // wait for release
                    vTaskDelay(pdMS_TO_TICKS(500));
            }
        }
        prev = lvl;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
