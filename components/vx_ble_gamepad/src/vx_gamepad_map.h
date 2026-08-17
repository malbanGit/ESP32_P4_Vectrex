/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Internal: parsed HID descriptor -> vx_gamepad_state_t.
 *
 * Scope is deliberately narrow (2 analog axes + 4 buttons), but the SELECTION
 * is not: on an unknown off-the-shelf gamepad we must find the analog stick
 * among several axes and the four face buttons among a dozen.  That choice is
 * the whole job of this file.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "vx_ble_gamepad.h"
#include "vx_hid_desc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t field;      /* index into vx_hid_desc_t.fields, -1 = absent */
    int32_t logical_min;
    int32_t logical_max;
    int32_t center;     /* raw value treated as 0 after calibration     */
} vx_axis_binding_t;

typedef struct {
    const vx_hid_desc_t *desc;      /* borrowed, must outlive this struct */

    uint8_t           report_id;
    bool              valid;
    vx_axis_binding_t axis_x;
    vx_axis_binding_t axis_y;
    int16_t           button_field[VX_GAMEPAD_BUTTONS];
    uint8_t           button_count; /* how many of the 4 were actually found */

    /* Calibration */
    bool     calibrating;
    uint8_t  cal_samples;
    int32_t  cal_sum_x;
    int32_t  cal_sum_y;

    uint8_t  deadzone;
} vx_gamepad_bind_t;

/**
 * @brief Choose the analog stick and the four face buttons from a descriptor.
 *
 * Selection rules, in order:
 *  - axes: Generic Desktop X and Y.  These are the LEFT analog stick on every
 *    gamepad layout in circulation; Z/Rz (right stick) and the Simulation-page
 *    triggers are ignored on purpose.
 *  - a Hat switch is ignored: the Vectrex expects an analog stick, and a
 *    D-pad mapped onto X/Y would feel wrong rather than merely limited.
 *  - buttons: Button page usages 1 to 4.  On a conventional gamepad these are
 *    the four face buttons; higher usages are shoulders, sticks and menu keys.
 *
 * @return true if at least X, Y or one button was found.
 */
bool vx_gamepad_bind_build(const vx_hid_desc_t *desc, uint8_t deadzone,
                           vx_gamepad_bind_t *out);

/**
 * @brief Decode one input report payload into a state.
 *
 * @param payload report bytes WITHOUT the leading report-ID byte
 * @return true if this report ID is the one we bound and decoding succeeded.
 */
bool vx_gamepad_bind_decode(vx_gamepad_bind_t *bind, uint8_t report_id,
                            const uint8_t *payload, size_t len,
                            vx_gamepad_state_t *state);

/** @brief Restart centre calibration; sticks are assumed at rest. */
void vx_gamepad_bind_calibrate(vx_gamepad_bind_t *bind);

/** @brief Log what was selected, and what was seen but left unused. */
void vx_gamepad_bind_dump(const vx_gamepad_bind_t *bind, const char *tag);

#ifdef __cplusplus
}
#endif
