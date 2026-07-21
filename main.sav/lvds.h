/*
 * LVDS output bring-up via LT8912B -> ST7262 panel (800x480@60).
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_lt8912b.h"

// Bring up LVDS 800x480@60 to the ST7262 panel: configures the DPI at the panel's
// native timing, switches the LT8912B from HDMI to LVDS output (no public driver
// API for this -> done over I2C), and turns the backlight on.
//   yuv422   : frame-buffer format (the P4 DPI converts YUV422->RGB888 in HW).
//   out_panel: receives the panel handle (use it for draw_bitmap).
//   out_w/out_h: receive the active resolution (800x480).
esp_err_t lvds_start(const esp_lcd_panel_lt8912b_io_t *io, esp_lcd_dsi_bus_handle_t dsi_bus,
                     bool yuv422, esp_lcd_panel_handle_t *out_panel, int *out_w, int *out_h);

// Diagnostic: 1 = after the mire is drawn, sweep the MIPI D-PHY "settle" (0x11)
// through a range, ~10 s per value, logging the detected input vtotal. Watch the
// screen + console to find the settle that gives vtotal=502 and a stable picture,
// then set LVDS_MIPI_SETTLE (in lvds.c) to it and turn this back to 0.
#define LVDS_SETTLE_SWEEP 0
void lvds_sweep_settle(const esp_lcd_panel_lt8912b_io_t *io);

// Backlight control. NOTE: BL_PWM is ACTIVE-LOW (a HIGH level cuts the backlight),
// so the PWM duty is inverted vs brightness (50% duty -> ~50% brightness). BL_EN
// (active-high) gates the backlight driver. on=true -> BL_EN high + 50% PWM.
void lvds_backlight(bool on, int level);

// Diagnostic: log the bridge's input V-detect (0x9E/0x9F) then 20 samples of the
// live stream-DDS word (-> MHz), STB flag and input-line measurements. Callable
// again at any time (e.g. after enabling the P4 DPI test pattern) to see whether
// the servo equilibrium / line counting changed.
void lvds_read_dds(const esp_lcd_panel_lt8912b_io_t *io, bool fast);
