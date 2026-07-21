/*
 * Self-generated test card (mire) renderer — output-agnostic.
 * Draws SMPTE color bars + geometry helpers + an info panel with generated text.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// Render the test card into fb (w x h pixels).
//   yuv422 == true : fb is YUV422 (YUYV-packed, 2 bytes/px).
//   yuv422 == false: fb is RGB888 (3 bytes/px).
//   title          : short UPPERCASE label shown in the panel (e.g. "HDMI"/"LVDS").
// Returns ESP_ERR_NO_MEM if the (YUV422-only) RGB scratch buffer can't be allocated.
esp_err_t mire_render(uint8_t *fb, int w, int h, bool yuv422, const char *title);
