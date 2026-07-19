/*
 * LVDS output bring-up via LT8912B -> ST7262 (800x480@60).
 *
 * The esp_lcd_lt8912b driver always enables HDMI / disables LVDS at init and
 * exposes no public selector, so after init we replicate its internal LVDS-on +
 * HDMI-off register writes over the main I2C bank. The LVDS PLL locks onto the
 * DPI pixel clock (scaler bypassed), so we just drive the DPI at 800x480.
 */
#include "lvds.h"
#include "board.h"
#include "esp_check.h"
#include "esp_lcd_panel_ops.h"
#include "esp_clk_tree.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "lvds";

// Diagnostic: set to 1 to drive the LT8912B's INTERNAL pattern generator instead
// of the P4 MIPI feed. It isolates LVDS + panel + timing from the DPI input:
//   - pattern visible & stable -> LVDS/panel/timing OK, problem is the MIPI input.
//   - still black             -> problem is LVDS/panel/timing/electrical.
// note : having it activated does not prevent the HDMI test to display our own pattern.
#define LVDS_INTERNAL_PATTERN  0

// ST7262 800x480 @ ~60 Hz. The P4 MIPI-DSI DPI clock is source / integer_div
// (default source PLL_F240M = 240 MHz), so it can only emit 240/N MHz.
//
// EXPERIMENT (2026-06-06): the bridge's DDS recovered-clock reads ~8/9 of the input
// (24.000 -> ~21.3 MHz, below the panel's 23 MHz min). If that ratio is real, we
// need input >= 25.9 MHz so recovered stays >= 23. The only clean in-range value is
// 26.667 MHz (240/9): recovered ~= 23.7 MHz, just above 23 -> may finally lock.
// So we REQUEST 26 -> 240/9 = 26.667 MHz actual. H/V totals chosen for ~60 Hz inside
// the panel ranges: htotal 896 (808/816/896 max), vtotal 496 (488/496/504) ->
// 26.667e6/(896*496) = 60.0 Hz. The bridge is declared pclk 27 (nearest integer to
// the 26.667 MHz it actually receives). Revert to 24/816 if this does not help.

#define ST7262_DPI_CLK_MHZ      24
#define ST7262_DECLARE_PCLK_MHZ 24
#define ST7262_HTOTAL           896
#define ST7262_VTOTAL           496
#define ST7262_VIDEO_TIMING() {                   \
        .hfp = 44, .hs = 8, .hbp = 44,            \
        .hact = 800, .htotal = ST7262_HTOTAL,     \
        .vfp = 4, .vs = 4, .vbp = 8,              \
        .vact = 480, .vtotal = ST7262_VTOTAL,     \
        .h_polarity = 0, .v_polarity = 0,         \
        .vic = 0,                                 \
        .aspect_ratio = LT8912B_ASPECT_RATION_16_9, \
        .pclk_mhz = ST7262_DECLARE_PCLK_MHZ,      \
    }
#define ST7262_DPI_CONFIG() {                     \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT, \
        .dpi_clock_freq_mhz = ST7262_DPI_CLK_MHZ, \
        .virtual_channel = 0,                     \
        .in_color_format = LCD_COLOR_PIXEL_FORMAT_RGB888, \
        .num_fbs = 2,                             \
        .video_timing = {                         \
            .h_size = 800, .v_size = 480,         \
            .hsync_back_porch = 44, .hsync_pulse_width = 8, .hsync_front_porch = 44, \
            .vsync_back_porch = 8, .vsync_pulse_width = 4, .vsync_front_porch = 4, \
        },                                        \
        .flags.disable_lp = true,                 \
    }

// Log the ACTUAL DPI pixel clock the P4 emits AND the resulting refresh. The DPI
// clock is source / integer_div (default source PLL_F240M = 240 MHz), so a request
// that does not divide the source evenly is truncated: div = src/req; actual = src/div
// (same as the IDF HAL). NOTE: the requested value (e.g. 26) may differ from the real
// emitted clock (240/9 = 26.667), so refresh MUST be computed from the actual.
static void lvds_log_dpi_clock(uint32_t requested_mhz)
{
    uint32_t src_hz = 0;
    if (esp_clk_tree_src_get_freq_hz((soc_module_clk_t)MIPI_DSI_DPI_CLK_SRC_DEFAULT,
                                     ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &src_hz) != ESP_OK) {
        ESP_LOGW(TAG, "could not read DPI clock source frequency");
        return;
    }
    uint32_t src_mhz = src_hz / 1000000;
    uint32_t div = (requested_mhz && src_mhz / requested_mhz) ? src_mhz / requested_mhz : 1;
    double actual = (double)src_mhz / div;
    double refresh = actual * 1e6 / ((double)ST7262_HTOTAL * ST7262_VTOTAL);
    ESP_LOGW(TAG, "DPI clock: source=%u MHz, requested=%u MHz -> div=%u -> ACTUAL=%.3f MHz",
             (unsigned)src_mhz, (unsigned)requested_mhz, (unsigned)div, actual);
    ESP_LOGW(TAG, "Emitted refresh = %.2f Hz (%.3f MHz / %dx%d)",
             refresh, actual, ST7262_HTOTAL, ST7262_VTOTAL);
}

// Override the bridge's stream-DDS center to the EXACT received pixel clock. The
// driver derives the DDS word (regs 0x4e-0x50) from the INTEGER pclk_mhz (we declare
// 27), but we actually feed 240/9 = 26.667 MHz. That ~1.25% mismatch is what shows up
// as a diagonal shear. We rewrite the precise word (freq_mhz * 0x16C16) to null it.
static esp_err_t lvds_set_dds_exact(const esp_lcd_panel_lt8912b_io_t *io)
{
    const uint32_t div = 240u / ST7262_DPI_CLK_MHZ;          // 240/26 = 9 -> 26.667 MHz
    uint32_t dds = (uint32_t)((uint64_t)240u * 0x16C16 / div);
    const uint8_t seq[][2] = {
        {0x4e, dds & 0xff}, {0x4f, (dds >> 8) & 0xff}, {0x50, (dds >> 16) & 0xff},
    };
    for (size_t i = 0; i < sizeof(seq) / sizeof(seq[0]); i++)
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io->cec_dsi, seq[i][0], &seq[i][1], 1),
                            TAG, "dds exact write");
    // Re-pulse the DDS reset (main bank 0x05) so the DDS re-locks from the new center.
    const uint8_t rst[][2] = { {0x05, 0xfb}, {0x05, 0xff} };
    for (size_t i = 0; i < sizeof(rst) / sizeof(rst[0]); i++) {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io->main, rst[i][0], &rst[i][1], 1), TAG, "dds reset");
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGW(TAG, "DDS center overridden to exact %u/%u MHz (word=0x%06x) + DDS reset, to fight shear",
             240u, (unsigned)div, (unsigned)dds);
    return ESP_OK;
}

// Switch the bridge to LVDS: LVDS PLL/TX reset, LVDS output ON, HDMI output OFF.
// (Mirrors the driver's internal _panel_lt8912b_lvds_output(true)/hdmi_output(false).)
static esp_err_t lt8912b_select_lvds(esp_lcd_panel_io_handle_t io_main)
{
    static const uint8_t seq[][2] = {
        {0x02, 0xf7}, {0x02, 0xff},                 // LVDS PLL reset
        {0x03, 0xcb}, {0x03, 0xfb}, {0x03, 0xff},   // LVDS TX module reset
        {0x44, 0x30},                               // LVDS output ON
        {0x33, 0x0c},                               // HDMI output OFF
    };
    for (size_t i = 0; i < sizeof(seq) / sizeof(seq[0]); i++)
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io_main, seq[i][0], &seq[i][1], 1),
                            TAG, "LVDS select write failed");
    ESP_LOGI(TAG, "LT8912B switched to LVDS output (HDMI off)");
    return ESP_OK;
}

// MIPI D-PHY "settle" tuning (per the 8912B debugging guidance). The driver sets
// 0x11=0x08, which targets 720p; our smaller 800x480 input needs a different
// settle, otherwise the DDS / resolution detection is wrong. After changing settle
// we reset the MIPI RX so it re-captures, then read back the detected input vtotal
// (0x9f:0x9e): it MUST read 496 (our configured vtotal) for a clean 800x480 capture
// -- the precondition for the LVDS bypass PLL to lock onto the recovered pixel clock.
// >>> Sweep LVDS_MIPI_SETTLE until the logged "detected input vtotal" == 496. <<<
#define LVDS_MIPI_SETTLE  0x05
static esp_err_t lvds_tune_settle(const esp_lcd_panel_lt8912b_io_t *io)
{
    uint8_t v = LVDS_MIPI_SETTLE;
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io->cec_dsi, 0x11, &v, 1), TAG, "settle");

    // MIPI RX logic + DDS reset so the new settle takes effect on the next capture.
    const uint8_t rst[][2] = { {0x03, 0x7f}, {0x03, 0xff}, {0x05, 0xfb}, {0x05, 0xff} };
    for (size_t i = 0; i < sizeof(rst) / sizeof(rst[0]); i++) {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io->main, rst[i][0], &rst[i][1], 1), TAG, "rxrst");
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(200));

    // Read back the detected input timing. H = 0x9c/0x9d, V = 0x9e/0x9f (same regs
    // the driver logs). V captures fine (~496); H currently reads 0xffff (65535 =
    // saturated, NOT captured) -- that is the remaining blocker, so watch BOTH.
    uint8_t hl = 0, hh = 0, vl = 0, vh = 0;
    esp_lcd_panel_io_rx_param(io->main, 0x9c, &hl, 1);
    esp_lcd_panel_io_rx_param(io->main, 0x9d, &hh, 1);
    esp_lcd_panel_io_rx_param(io->main, 0x9e, &vl, 1);
    esp_lcd_panel_io_rx_param(io->main, 0x9f, &vh, 1);
    ESP_LOGI(TAG, "settle=0x%02x -> detected H=%d (stable count, NOT 65535)  V=%d (want ~496)",
             LVDS_MIPI_SETTLE, (hh << 8) | hl, (vh << 8) | vl);
    return ESP_OK;
}

// Live settle sweep (diagnostic): step through settle values, ~10 s each, logging
// the detected input vtotal (target 502). Run AFTER the mire is drawn so you can
// watch the picture stabilize on screen and read the vtotal in the console.
void lvds_sweep_settle(const esp_lcd_panel_lt8912b_io_t *io)
{
    static const uint8_t values[] = { 0x01, 0x02, 0x04, 0x06, 0x08, 0x0c, 0x10, 0x14, 0x18, 0x20, 0x30 };
    ESP_LOGW(TAG, "=== SETTLE SWEEP start (target H stable & V~496, ~10 s / value) ===");
    for (size_t k = 0; k < sizeof(values); k++) {
        uint8_t v = values[k];
        esp_lcd_panel_io_tx_param(io->cec_dsi, 0x11, &v, 1);   // set settle
        const uint8_t rst[][2] = { {0x03, 0x7f}, {0x03, 0xff}, {0x05, 0xfb}, {0x05, 0xff} };
        for (size_t i = 0; i < sizeof(rst) / sizeof(rst[0]); i++) {
            esp_lcd_panel_io_tx_param(io->main, rst[i][0], &rst[i][1], 1);   // MIPI RX + DDS reset
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        vTaskDelay(pdMS_TO_TICKS(300));
        uint8_t hl = 0, hh = 0, vl = 0, vh = 0;
        esp_lcd_panel_io_rx_param(io->main, 0x9c, &hl, 1);
        esp_lcd_panel_io_rx_param(io->main, 0x9d, &hh, 1);
        esp_lcd_panel_io_rx_param(io->main, 0x9e, &vl, 1);
        esp_lcd_panel_io_rx_param(io->main, 0x9f, &vh, 1);
        ESP_LOGW(TAG, ">>> settle=0x%02x : H=%d (stable count, NOT 65535)  V=%d (want ~496) -- look at the screen <<<",
                 v, (hh << 8) | hl, (vh << 8) | vl);
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
    ESP_LOGW(TAG, "=== SETTLE SWEEP done ===");
}

// Backlight on/off. BL_PWM is ACTIVE-LOW (HIGH cuts the light) so the duty is
// inverted vs brightness: 512/1024 = 50% duty -> ~50% brightness, 1023 ~ off.
// BL_EN (active-high) gates the backlight driver. LEDC is configured once.
void lvds_backlight(bool on)
{
    static bool inited = false;
    if (!inited) {
        gpio_config_t en = { .mode = GPIO_MODE_OUTPUT, .pin_bit_mask = 1ULL << BOARD_PIN_BL_EN };
        ESP_ERROR_CHECK(gpio_config(&en));

        ledc_timer_config_t tcfg = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_10_BIT,   // 0..1023 (full = 1024)
            .timer_num = LEDC_TIMER_0,
            .freq_hz = 1000,
            .clk_cfg = LEDC_AUTO_CLK,
        };
        ESP_ERROR_CHECK(ledc_timer_config(&tcfg));
        ledc_channel_config_t ccfg = {
            .gpio_num = BOARD_PIN_BL_PWM,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_0,
            .timer_sel = LEDC_TIMER_0,
            .duty = 980,   // start 50% dark (active-low) until enabled
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ccfg));
        inited = true;
    }
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, on ? 512 : 1023));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
    gpio_set_level(BOARD_PIN_BL_EN, on ? 1 : 0);
    ESP_LOGI(TAG, "Backlight %s", on ? "ON (50%% PWM duty -> ~50%% brightness)" : "OFF");
}

// ST7262 LCD hardware reset (GPIO23, active-low). The bridge has no CPU reset,
// so this drives ONLY the panel: assert, hold, release, then let the TCON settle
// before the LVDS signal is turned on.
static void lcd_reset(void)
{
    gpio_config_t io = { .mode = GPIO_MODE_OUTPUT, .pin_bit_mask = 1ULL << BOARD_PIN_RESET_DISPLAY };
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(BOARD_PIN_RESET_DISPLAY, 0);   // assert reset
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(BOARD_PIN_RESET_DISPLAY, 1);   // release
    vTaskDelay(pdMS_TO_TICKS(50));                // let the panel TCON come up
    ESP_LOGI(TAG, "ST7262 LCD reset released");
}

#if LVDS_INTERNAL_PATTERN
// Enable the LT8912B built-in pattern generator (CEC/DSI bank), keyed to our
// ST7262 timing. Mirrors the driver's internal _panel_lt8912b_send_test().
static esp_err_t lvds_test_pattern(esp_lcd_panel_io_handle_t io_cec)
{
    const int hs = 8, hbp = 44, vs = 4, vbp = 8;
    const int hact = 800, vact = 480, htotal = ST7262_HTOTAL, vtotal = ST7262_VTOTAL, pclk = ST7262_DECLARE_PCLK_MHZ;
    const uint8_t seq[][2] = {
        {0x72, 0x12},
        {0x73, (hs + hbp) & 0xff}, {0x74, (hs + hbp) >> 8},
        {0x75, (vs + vbp) & 0xff},
        {0x76, hact & 0xff}, {0x77, vact & 0xff},
        {0x78, (uint8_t)(((vact >> 8) << 4) | (hact >> 8))},
        {0x79, htotal & 0xff}, {0x7a, vtotal & 0xff},
        {0x7b, (uint8_t)(((vtotal >> 8) << 4) | (htotal >> 8))},
        {0x7c, hs & 0xff}, {0x7d, (uint8_t)(((hs >> 8) << 6) | (vs & 0xff))},
        {0x70, 0x80}, {0x71, 0x51}, {0x42, 0x12},  // pattern enable
        {0x1e, 0x67},                              // h/v/d pol, hdmi sel, pll sel
    };
    for (size_t i = 0; i < sizeof(seq) / sizeof(seq[0]); i++)
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io_cec, seq[i][0], &seq[i][1], 1),
                            TAG, "pattern write");
    uint32_t dds = (uint32_t)(pclk * 0x16C16);   // pixel-clock DDS word
    const uint8_t dds_seq[][2] = {
        {0x4e, dds & 0xff}, {0x4f, (dds >> 8) & 0xff}, {0x50, (dds >> 16) & 0xff}, {0x51, 0x80},
    };
    for (size_t i = 0; i < sizeof(dds_seq) / sizeof(dds_seq[0]); i++)
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io_cec, dds_seq[i][0], &dds_seq[i][1], 1),
                            TAG, "pattern dds");
    ESP_LOGW(TAG, "LVDS INTERNAL test pattern enabled (bypassing MIPI input)");
    return ESP_OK;
}
#endif

// Diagnostic: read the DDS feedback (CEC/DSI bank 0x0c-0x0f) a few times. This is the
// bridge's recovered-clock measurement; STABLE values mean the DDS is tracking the
// recovered MIPI pixel clock, jumping/0x00/0xff = not locking. The driver's lock
// signature is 0x0e==0xd2 / 0x0d in 0xd1..0xfe (checked after the samples).
static void lvds_read_dds(esp_lcd_panel_io_handle_t io_cec)
{
    uint8_t c = 0, d = 0, e = 0, f = 0;
    for (int i = 0; i < 6; i++) {
        esp_lcd_panel_io_rx_param(io_cec, 0x0c, &c, 1);
        esp_lcd_panel_io_rx_param(io_cec, 0x0d, &d, 1);
        esp_lcd_panel_io_rx_param(io_cec, 0x0e, &e, 1);
        esp_lcd_panel_io_rx_param(io_cec, 0x0f, &f, 1);
        ESP_LOGI(TAG, "DDS 0x0c-0f = 0x%02x 0x%02x 0x%02x 0x%02x  (stable = recovered clock locked)", c, d, e, f);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    // The READ feedback (0x0c-0x0f) is NOT the same scale as the WRITE DDS (0x4e-0x50),
    // so converting it to MHz is meaningless. The driver's own lock criterion is
    // 0x0e == 0xd2 with 0x0d in 0xd1..0xfe. Report against that instead.
    bool locked = (e == 0xd2) && (d > 0xd0) && (d < 0xff);
    ESP_LOGW(TAG, "DDS lock check: 0x0e=0x%02x (want 0xd2)  0x0d=0x%02x  -> %s",
             e, d, locked ? "LOCKED" : "not locked");
}

esp_err_t lvds_start(const esp_lcd_panel_lt8912b_io_t *io, esp_lcd_dsi_bus_handle_t dsi_bus,
                     bool yuv422, esp_lcd_panel_handle_t *out_panel, int *out_w, int *out_h)
{
    lcd_reset();   // ST7262 needs a clean reset before it will lock onto the LVDS
esp_log_level_set("*", ESP_LOG_DEBUG);
    // Native 800x480 MIPI input, LVDS bypass (no scaler). Needs the right D-PHY
    // settle for the input to be captured cleanly (see lvds_tune_settle).
    static esp_lcd_dpi_panel_config_t dpi = ST7262_DPI_CONFIG();
    dpi.in_color_format  = yuv422 ? LCD_COLOR_FMT_YUV422 : LCD_COLOR_FMT_RGB888;
    dpi.out_color_format = LCD_COLOR_FMT_RGB888;

    lvds_log_dpi_clock(dpi.dpi_clock_freq_mhz);   // step 1: confirm the real pixel clock

    lt8912b_vendor_config_t vc = {
        .video_timing = ST7262_VIDEO_TIMING(),
        .mipi_config = { .dsi_bus = dsi_bus, .dpi_config = &dpi, .lane_num = 2 },
    };
    esp_lcd_panel_dev_config_t dev = {
        .reset_gpio_num = -1,   // GPIO23 = LCD reset (done in lcd_reset); bridge uses power-on reset
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
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(panel), TAG, "init");  // driver enables HDMI...
#if LVDS_INTERNAL_PATTERN
    ESP_RETURN_ON_ERROR(lvds_test_pattern(io->cec_dsi), TAG, "test pattern");
#endif

    ESP_RETURN_ON_ERROR(lvds_tune_settle(io), TAG, "settle tune");          // clean 800x480 capture
    ESP_RETURN_ON_ERROR(lt8912b_select_lvds(io->main), TAG, "select lvds"); // driver already set bypass
    ESP_RETURN_ON_ERROR(lvds_set_dds_exact(io), TAG, "dds exact");          // align DDS AFTER LVDS PLL reset
    lvds_backlight(true);   // LCD out of reset + LVDS up -> turn the light on
    lvds_read_dds(io->cec_dsi);   // DIAGNOSTIC: DDS stability (high bytes stable = OK)

    *out_panel = panel;
    *out_w = 800;
    *out_h = 480;
    ESP_LOGI(TAG, "LVDS up: native 800x480 bypass (%s)", yuv422 ? "YUV422" : "RGB888");
    return ESP_OK;
}
