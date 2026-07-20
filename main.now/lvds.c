/*
 * LVDS output via LT8912B -> ST7262 (800x480@60), MIPI bypass path. WORKING.
 *
 * The stable-picture recipe has three required elements (full story, measured
 * evidence and the underlying ESP-IDF bug: see LVDS_BRINGUP.md at repo root):
 *   1. Exact DSI-host horizontal timing rewritten in lvds_start(): IDF stores
 *      the DPI clock as an integer (240/9 -> "26" instead of 26.667 MHz) and
 *      stretches every DSI line by 2.56%, which line-locked sinks cannot take.
 *   2. LT8912B 0x49:0x1E = 0x0F -- bit6=0 selects LVDS-mode sync generation
 *      (the Lontium reference init leaves HDMI-mode sync on the LVDS path).
 *   3. Fixed DDS software frequency word (exact pclk, 0x51 bit7 kept set):
 *      with a line-locked output the per-line realign absorbs crystal ppm.
 * Lane rate (BOARD_DSI_LANE_MBPS, board.h) is NOT constrained: with this recipe
 * the link is stable up to the LT8912B's 1.5 Gbps spec (earlier "RX margin"
 * findings were an artefact of the broken timing).
 *
 * The esp_lcd_lt8912b driver always enables HDMI / disables LVDS at init and
 * exposes no public selector, so after init we replicate its internal LVDS-on +
 * HDMI-off register writes over the main I2C bank.
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
#include "hal/mipi_dsi_host_ll.h"   // exact host H-timing fix (see lvds_start)

static const char *TAG = "lvds";

// Diagnostic: set to 1 to drive the LT8912B's INTERNAL pattern generator instead
// of the P4 MIPI feed. It isolates LVDS + panel + timing from the DPI input:
//   - pattern visible & stable -> LVDS/panel/timing OK, problem is the MIPI input.
//   - still black             -> problem is LVDS/panel/timing/electrical.
#define LVDS_INTERNAL_PATTERN  0

// ST7262 800x480 @ 60.00 Hz. The P4 DPI clock is PLL / integer divider (240 or
// 160 MHz sources), so in the panel's 23-27 MHz DCLK range only 24.000 (240/10)
// and 26.667 (240/9) exist. We request 26 -> 240/9 = 26.667 MHz actual, with
// htotal 896 (panel 808-896) and vtotal 496 (panel 488-504):
// 26.667e6 / (896 x 496) = 60.00 Hz.
#define ST7262_DPI_CLK_MHZ      24    // requested; 240/9 = 26.667 MHz actual
#define ST7262_DECLARE_PCLK_MHZ 24    // bridge-declared integer pclk (nearest)
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

// Log the ACTUAL DPI pixel clock the P4 emits and the resulting refresh. The DPI
// clock is source / integer_div, so a request that does not divide the source
// evenly is truncated: div = src/req; actual = src/div (same as the IDF HAL).
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
    ESP_LOGI(TAG, "DPI clock: source=%u MHz, requested=%u -> ACTUAL=%.3f MHz, refresh=%.2f Hz",
             (unsigned)src_mhz, (unsigned)requested_mhz, actual, refresh);
}

// Bridge DDS + sync-generation config (CEC/DSI bank 0x49):
//  - 0x1E = 0x0F: LVDS-mode output sync generation (bit6=0). The reference init
//    value 0x4F leaves HDMI-mode sync on -> erratic line timing on LVDS.
//  - 0x4E-0x51: stream DDS = FIXED software word for the exact input pixel clock
//    (word = pclk_MHz x 0x16C16), 0x51 bit7 kept set (no tracking servo). The
//    line-locked output realigns on every input HSS, absorbing crystal offset.
//  - then a DDS reset (main bank 0x05) so the DDS restarts on the new word.
static esp_err_t lvds_dds_config(const esp_lcd_panel_lt8912b_io_t *io)
{
    const uint32_t div = 240u / ST7262_DPI_CLK_MHZ;          // 240/26 = 9 -> 26.667 MHz
    uint32_t dds = (uint32_t)((uint64_t)240u * 0x16C16 / div);
    const uint8_t seq[][2] = {
        {0x1e, 0x0f},
        {0x4e, dds & 0xff}, {0x4f, (dds >> 8) & 0xff}, {0x50, (dds >> 16) & 0xff},
        {0x51, 0x80},   // bit7 = SW_FREQ_WORD_EN, kept set
    };
    for (size_t i = 0; i < sizeof(seq) / sizeof(seq[0]); i++)
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io->cec_dsi, seq[i][0], &seq[i][1], 1),
                            TAG, "dds write");
    const uint8_t rst[][2] = { {0x05, 0xfb}, {0x05, 0xff} };
    for (size_t i = 0; i < sizeof(rst) / sizeof(rst[0]); i++) {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io->main, rst[i][0], &rst[i][1], 1), TAG, "dds reset");
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "DDS fixed word 0x%06x (%.4f MHz), 0x1E=0x0F (lvds sync mode)",
             (unsigned)dds, (double)dds / 0x16C16);
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

// MIPI D-PHY "settle" (0x49:0x11). The driver sets 0x10 (reference value for
// high-rate inputs); 0x05 is what captures our 800x480 input cleanly.
// After changing settle, reset the MIPI RX + DDS so it re-captures, then read
// back the detected input total: V (0x9E/0x9F) must read 496.
#define LVDS_MIPI_SETTLE  0x05
static esp_err_t lvds_tune_settle(const esp_lcd_panel_lt8912b_io_t *io)
{
    uint8_t v = LVDS_MIPI_SETTLE;
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io->cec_dsi, 0x11, &v, 1), TAG, "settle");

    const uint8_t rst[][2] = { {0x03, 0x7f}, {0x03, 0xff}, {0x05, 0xfb}, {0x05, 0xff} };
    for (size_t i = 0; i < sizeof(rst) / sizeof(rst[0]); i++) {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io->main, rst[i][0], &rst[i][1], 1), TAG, "rxrst");
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(200));

    uint8_t hl = 0, hh = 0, vl = 0, vh = 0;
    esp_lcd_panel_io_rx_param(io->main, 0x9c, &hl, 1);
    esp_lcd_panel_io_rx_param(io->main, 0x9d, &hh, 1);
    esp_lcd_panel_io_rx_param(io->main, 0x9e, &vl, 1);
    esp_lcd_panel_io_rx_param(io->main, 0x9f, &vh, 1);
    ESP_LOGI(TAG, "settle=0x%02x -> detected H=%d  V=%d (496 = healthy)",
             LVDS_MIPI_SETTLE, (hh << 8) | hl, (vh << 8) | vl);
    return ESP_OK;
}

// Live settle sweep (diagnostic): step through settle values, ~10 s each, logging
// the detected input totals. Watch the screen + console, then bake the best value
// into LVDS_MIPI_SETTLE.
void lvds_sweep_settle(const esp_lcd_panel_lt8912b_io_t *io)
{
    static const uint8_t values[] = { 0x01, 0x02, 0x04, 0x06, 0x08, 0x0c, 0x10, 0x14, 0x18, 0x20, 0x30 };
    ESP_LOGW(TAG, "=== SETTLE SWEEP start (target V=496, ~10 s / value) ===");
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
        ESP_LOGW(TAG, ">>> settle=0x%02x : H=%d  V=%d (want 496) -- look at the screen <<<",
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
        {0x1e, 0x67},                              // pattern-mode protocol config
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

// Diagnostic: log the bridge's health figures (see LVDS_BRINGUP.md "Sanity
// checks"). Healthy values: V-detect = 496; DDS word = the fixed 0x25ECF5
// (26.667 MHz); HT_CNT (0x49:0x01/02, input line length in pixel clocks)
// = 895-896; HSYNC_POS (0x49:0x09/0A) = a SMALL CONSTANT (wandering values
// mean the input line timing is broken). fast=true: 500 ms between samples.
void lvds_read_dds(const esp_lcd_panel_lt8912b_io_t *io, bool fast)
{
    esp_lcd_panel_io_handle_t io_cec = io->cec_dsi;
    uint8_t c = 0, d = 0, e = 0, f = 0, stb = 0;
    uint8_t htl = 0, hth = 0, hpl = 0, hph = 0;
    uint8_t vl = 0, vh = 0;
    esp_lcd_panel_io_rx_param(io->main, 0x9e, &vl, 1);
    esp_lcd_panel_io_rx_param(io->main, 0x9f, &vh, 1);
    ESP_LOGI(TAG, "input V-detect = %d (496 = healthy)", (vh << 8) | vl);
    for (int i = 0; i < 20; i++) {
        esp_lcd_panel_io_rx_param(io_cec, 0x0c, &c, 1);
        esp_lcd_panel_io_rx_param(io_cec, 0x0d, &d, 1);
        esp_lcd_panel_io_rx_param(io_cec, 0x0e, &e, 1);
        esp_lcd_panel_io_rx_param(io_cec, 0x0f, &f, 1);
        esp_lcd_panel_io_rx_param(io_cec, 0x0b, &stb, 1);
        esp_lcd_panel_io_rx_param(io_cec, 0x01, &htl, 1);
        esp_lcd_panel_io_rx_param(io_cec, 0x02, &hth, 1);
        esp_lcd_panel_io_rx_param(io_cec, 0x09, &hpl, 1);
        esp_lcd_panel_io_rx_param(io_cec, 0x0a, &hph, 1);
        uint32_t word = ((uint32_t)(f & 0x03) << 24) | ((uint32_t)e << 16) | ((uint32_t)d << 8) | c;
        ESP_LOGI(TAG, "DDS word=0x%07x -> %.3f MHz  STB=%d  HT_CNT=%u (want %u)  HSYNC_POS=%u",
                 (unsigned)word, (double)word / 0x16C16, stb & 1,
                 (unsigned)((hth << 8) | htl), (unsigned)ST7262_HTOTAL,
                 (unsigned)((hph << 8) | hpl));
        vTaskDelay(pdMS_TO_TICKS(fast ? 100 : 5000));
    }
}

esp_err_t lvds_start(const esp_lcd_panel_lt8912b_io_t *io, esp_lcd_dsi_bus_handle_t dsi_bus,
                     bool yuv422, esp_lcd_panel_handle_t *out_panel, int *out_w, int *out_h)
{
    lcd_reset();   // ST7262 needs a clean reset before it will lock onto the LVDS

    static esp_lcd_dpi_panel_config_t dpi = ST7262_DPI_CONFIG();
    dpi.in_color_format  = yuv422 ? LCD_COLOR_FMT_YUV422 : LCD_COLOR_FMT_RGB888;
    dpi.out_color_format = LCD_COLOR_FMT_RGB888;

    lvds_log_dpi_clock(dpi.dpi_clock_freq_mhz);

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

    // ESP-IDF bug workaround (root cause of the historic skew, see LVDS_BRINGUP.md
    // section 2): the HAL stores the DPI clock as an INTEGER (240/9 -> 26 instead
    // of 26.667 MHz) and computes the DSI host's horizontal byte-clock timing
    // from it, stretching every line by 2.56%. Rewrite the host registers with
    // the EXACT pixel clock. The sum of the four fields is forced to the exact
    // line total (front porch absorbs the rounding) so no residual rate error
    // remains. MUST run before esp_lcd_panel_init(): the DW host latches its
    // video config when video mode starts.
    {
        double actual_pclk = 240.0 / (240 / ST7262_DPI_CLK_MHZ);       // 26.667, exact
        double r = (double)BOARD_DSI_LANE_MBPS / 8.0 / actual_pclk;    // byteclk/pixclk
        uint32_t total_b = (uint32_t)(ST7262_HTOTAL * r + 0.5);
        uint32_t hsw = (uint32_t)(8 * r + 0.5), hbp_b = (uint32_t)(44 * r + 0.5);
        uint32_t hact_b = (uint32_t)(800 * r + 0.5);
        uint32_t hfp_b = total_b - hsw - hbp_b - hact_b;
        mipi_dsi_host_ll_dpi_set_horizontal_timing(&MIPI_DSI_HOST, hsw, hbp_b, hact_b, hfp_b);
        ESP_LOGI(TAG, "DSI H timing set for %.3f MHz: hsa=%u hbp=%u hact=%u hfp=%u byteclk (line=%.2f us)",
                 actual_pclk, (unsigned)hsw, (unsigned)hbp_b, (unsigned)hact_b, (unsigned)hfp_b,
                 (double)total_b / (BOARD_DSI_LANE_MBPS / 8.0));
    }

    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(panel), TAG, "reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(panel), TAG, "init");  // driver enables HDMI...
#if LVDS_INTERNAL_PATTERN
    ESP_RETURN_ON_ERROR(lvds_test_pattern(io->cec_dsi), TAG, "test pattern");
#endif

    ESP_RETURN_ON_ERROR(lvds_tune_settle(io), TAG, "settle tune");          // clean 800x480 capture

    // Color-order fix: the P4's RGB888 passthrough and its YUV->RGB converter put
    // opposite byte orders on the DSI wire (measured: RGB888 FB shows R<->B
    // swapped on LVDS, YUV422 FB is correct). Compensate at the bridge's MIPI RX
    // channel mapping (0x49:0x40 bits[6:4] RGD_MIPIRX_RGB_SWAP, default 000) so
    // both FB formats display identical colors.
    {
        uint8_t swap = yuv422 ? 0x00 : 0x10;   // 001 = alternate RGB mapping for the RGB888 FB path
        swap = 0x00;
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io->cec_dsi, 0x40, &swap, 1), TAG, "rgb swap");
        ESP_LOGI(TAG, "MIPI RX RGB_SWAP (0x49:0x40) = 0x%02x for %s FB", swap, yuv422 ? "YUV422" : "RGB888");
    }

    ESP_RETURN_ON_ERROR(lvds_dds_config(io), TAG, "dds config");            // 0x1E + fixed DDS word
    ESP_RETURN_ON_ERROR(lt8912b_select_lvds(io->main), TAG, "select lvds"); // LVDS resets + output on
    lvds_backlight(true);   // LCD out of reset + LVDS up -> turn the light on
//    lvds_read_dds(io, true);     // health check (see LVDS_BRINGUP.md)

    *out_panel = panel;
    *out_w = 800;
    *out_h = 480;
    ESP_LOGI(TAG, "LVDS up: native 800x480 bypass (%s)", yuv422 ? "YUV422" : "RGB888");
    return ESP_OK;
}
