/*
 * Vectrex P4 board — pin map and build-time video options.
 * Shared by main / hdmi / lvds / mire modules; eventual single source of truth
 * for the merged board self-test.
 */
#pragma once

// --- GPIO map ----------------------------------------------------------------
#define BOARD_PIN_VCV_NOE        52   // active LOW: enables SN74AVC4T245 (I2S + HPD)
#define BOARD_PIN_RESET_DISPLAY  23   // active LOW: ST7262 LCD reset (the CPU does NOT reset the LT8912B)
#define BOARD_I2C_SDA            7
#define BOARD_I2C_SCL            8
#define BOARD_I2C_PORT           0
#define LT8912B_I2C_HZ           100000   // 100 kHz, plenty for register config
#define BOARD_PIN_VIDEO_MODE     28   // input, pull-up: HIGH=HDMI, LOW=LVDS
#define BOARD_PIN_BL_EN          34   // active HIGH: LCD backlight enable
#define BOARD_PIN_BL_PWM         11   // backlight PWM (1 kHz) dimming

// --- Frame-buffer pixel format (build-time) ----------------------------------
// The LT8912B MIPI RX always needs RGB888 on the wire; this only selects how the
// frame buffer is stored in PSRAM (and thus the scanout PSRAM bandwidth):
//   0 = native RGB888 (24 bpp): sharpest colors, full scanout (~166 MB/s @720p).
//   1 = YUV422 (16 bpp): halves scanout, lossless for black & white content
//       (full-res luma); the P4 DPI converts YUV422->RGB888 in hardware.
#define VIDEO_FB_YUV422          0
#define VIDEO_FB_BPP             (VIDEO_FB_YUV422 ? 2 : 3)
