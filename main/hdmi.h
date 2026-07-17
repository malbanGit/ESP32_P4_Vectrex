/*
 * HDMI output bring-up via LT8912B (1280x720@60).
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_lt8912b.h"

// Bring up HDMI 1280x720@60 on an already-created DSI bus and LT8912B I2C banks.
//   yuv422   : frame-buffer format (the P4 DPI converts YUV422->RGB888 in HW).
//   out_panel: receives the panel handle (use it for draw_bitmap / HPD check).
//   out_w/out_h: receive the active resolution (1280x720).
esp_err_t hdmi_start(const esp_lcd_panel_lt8912b_io_t *io, esp_lcd_dsi_bus_handle_t dsi_bus,
                     bool yuv422, esp_lcd_panel_handle_t *out_panel, int *out_w, int *out_h);
