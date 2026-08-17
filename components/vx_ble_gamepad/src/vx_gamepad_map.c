/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Parsed HID descriptor -> vx_gamepad_state_t.  See vx_gamepad_map.h.
 */

#include <string.h>

#include "esp_log.h"

#include "vx_ble_gamepad_cfg.h"
#include "vx_gamepad_map.h"

static const char *TAG = "vx_pad_map";

/* ---- binding -------------------------------------------------------------- */

static void axis_init(vx_axis_binding_t *a)
{
    a->field       = -1;
    a->logical_min = 0;
    a->logical_max = 0;
    a->center      = 0;
}

static void axis_set(vx_axis_binding_t *a, int16_t field_idx,
                     const vx_hid_field_t *f)
{
    a->field       = field_idx;
    a->logical_min = f->logical_min;
    a->logical_max = f->logical_max;
    /* Until calibration runs, assume the electrical centre is the middle of the
     * declared range.  That is right for a well-behaved device and close enough
     * for the first few reports of any other. */
    a->center = f->logical_min + (f->logical_max - f->logical_min) / 2;
}

bool vx_gamepad_bind_build(const vx_hid_desc_t *desc, uint8_t deadzone,
                           vx_gamepad_bind_t *out)
{
    if (!desc || !out) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->desc     = desc;
    out->deadzone = deadzone;
    axis_init(&out->axis_x);
    axis_init(&out->axis_y);
    for (int i = 0; i < VX_GAMEPAD_BUTTONS; ++i) {
        out->button_field[i] = -1;
    }

    /* A composite gamepad exposes several input reports (keyboard, consumer
     * keys, the pad itself).  Take the first one that carries an analog X/Y or
     * a face button, and ignore the rest: mixing controls from several reports
     * would mean tracking several payloads to build one state, for no benefit
     * on any gamepad we have seen. */
    for (uint8_t i = 0; i < desc->report_count; ++i) {
        const vx_hid_report_t *rep = &desc->reports[i];
        vx_gamepad_bind_t      cand;

        if (rep->type != VX_HID_REPORT_INPUT) {
            continue;
        }

        cand = *out;
        cand.report_id = rep->report_id;

        for (uint8_t j = 0; j < rep->field_count; ++j) {
            uint16_t              idx = rep->field_first + j;
            const vx_hid_field_t *f   = &desc->fields[idx];

            if (f->flags & VX_HID_FLAG_CONSTANT) {
                continue; /* padding */
            }
            /* Array items list pressed usages instead of holding one control
             * per field (that is how keyboards report keycodes).  Gamepad
             * buttons are always Variable. */
            if (!(f->flags & VX_HID_FLAG_VARIABLE)) {
                continue;
            }

            if (f->usage_page == VX_HID_PAGE_GENERIC_DESKTOP) {
                if (f->usage == VX_HID_USAGE_X && cand.axis_x.field < 0) {
                    axis_set(&cand.axis_x, (int16_t)idx, f);
                } else if (f->usage == VX_HID_USAGE_Y && cand.axis_y.field < 0) {
                    axis_set(&cand.axis_y, (int16_t)idx, f);
                }
                continue;
            }

            if (f->usage_page == VX_HID_PAGE_BUTTON && f->usage >= 1 &&
                f->usage <= VX_GAMEPAD_BUTTONS) {
                cand.button_field[f->usage - 1] = (int16_t)idx;
            }
        }

        cand.button_count = 0;
        for (int b = 0; b < VX_GAMEPAD_BUTTONS; ++b) {
            if (cand.button_field[b] >= 0) {
                cand.button_count++;
            }
        }

        if (cand.axis_x.field >= 0 || cand.axis_y.field >= 0 ||
            cand.button_count > 0) {
            cand.valid = true;
            *out       = cand;
            break;
        }
    }

    if (out->valid) {
        vx_gamepad_bind_calibrate(out);
    }
    return out->valid;
}

void vx_gamepad_bind_calibrate(vx_gamepad_bind_t *bind)
{
    if (!bind) {
        return;
    }
    bind->calibrating = true;
    bind->cal_samples = 0;
    bind->cal_sum_x   = 0;
    bind->cal_sum_y   = 0;
}

/* ---- decoding ------------------------------------------------------------- */

/* Map a raw axis value onto -128..127, with `center` reading exactly 0.
 *
 * The two halves are scaled independently against their own span.  That matters
 * on a real stick: with a centre at 115 out of 0..255, the negative side has 115
 * counts of travel and the positive side 140.  A single scale factor would make
 * the stick reach -128 early and never reach +127 — or the reverse.  Scaling
 * each side to its own extreme makes full deflection read full scale in both
 * directions, which is what a game expects.
 */
static int8_t scale_axis(int32_t raw, const vx_axis_binding_t *a,
                         uint8_t deadzone)
{
    int32_t span, out;

    if (a->logical_max <= a->logical_min) {
        return 0;
    }
    if (raw < a->logical_min) {
        raw = a->logical_min;
    }
    if (raw > a->logical_max) {
        raw = a->logical_max;
    }

    if (raw >= a->center) {
        span = a->logical_max - a->center;
        out  = span > 0 ? ((raw - a->center) * 127) / span : 0;
    } else {
        span = a->center - a->logical_min;
        out  = span > 0 ? ((raw - a->center) * 128) / span : 0;
    }

    if (out > 127) {
        out = 127;
    }
    if (out < -128) {
        out = -128;
    }

    /* Deadzone: below the threshold the stick reads exactly 0.  Without it, the
     * noise of a potentiometer stick makes a "released" control drift by a few
     * counts forever. */
    if (out > -(int32_t)deadzone && out < (int32_t)deadzone) {
        return 0;
    }
    return (int8_t)out;
}

bool vx_gamepad_bind_decode(vx_gamepad_bind_t *bind, uint8_t report_id,
                            const uint8_t *payload, size_t len,
                            vx_gamepad_state_t *state)
{
    int32_t raw_x = 0, raw_y = 0;
    bool    got_x = false, got_y = false;

    if (!bind || !bind->valid || !bind->desc || !payload || !state) {
        return false;
    }
    if (report_id != bind->report_id) {
        return false;
    }

    if (bind->axis_x.field >= 0) {
        got_x = vx_hid_field_extract(payload, len,
                                     &bind->desc->fields[bind->axis_x.field],
                                     &raw_x);
    }
    if (bind->axis_y.field >= 0) {
        got_y = vx_hid_field_extract(payload, len,
                                     &bind->desc->fields[bind->axis_y.field],
                                     &raw_y);
    }

    /* Calibration: average the first reports and take that as the zero.  The
     * gamepad is assumed untouched, which is true right after connection. */
    if (bind->calibrating) {
        if (got_x) {
            bind->cal_sum_x += raw_x;
        }
        if (got_y) {
            bind->cal_sum_y += raw_y;
        }
        if (++bind->cal_samples >= VX_CAL_SAMPLES) {
            if (got_x) {
                bind->axis_x.center = bind->cal_sum_x / VX_CAL_SAMPLES;
            }
            if (got_y) {
                bind->axis_y.center = bind->cal_sum_y / VX_CAL_SAMPLES;
            }
            bind->calibrating = false;
            ESP_LOGI(TAG, "calibrated: centre x=%ld y=%ld",
                     (long)bind->axis_x.center, (long)bind->axis_y.center);
        }
        /* Report 0 while calibrating rather than a value we know is provisional. */
        state->x = 0;
        state->y = 0;
    } else {
        state->x = got_x ? scale_axis(raw_x, &bind->axis_x, bind->deadzone) : 0;
        state->y = got_y ? scale_axis(raw_y, &bind->axis_y, bind->deadzone) : 0;
    }

    state->buttons = 0;
    for (int b = 0; b < VX_GAMEPAD_BUTTONS; ++b) {
        int32_t v;

        if (bind->button_field[b] < 0) {
            continue;
        }
        if (vx_hid_field_extract(payload, len,
                                 &bind->desc->fields[bind->button_field[b]],
                                 &v) &&
            v != 0) {
            state->buttons |= (uint8_t)(1u << b);
        }
    }

    return true;
}

/* ---- diagnostics ---------------------------------------------------------- */

void vx_gamepad_bind_dump(const vx_gamepad_bind_t *bind, const char *tag)
{
    const char *t = tag ? tag : TAG;

    if (!bind || !bind->desc) {
        return;
    }
    if (!bind->valid) {
        ESP_LOGW(t, "no usable gamepad control found in descriptor");
        return;
    }

    ESP_LOGI(t, "bound report id=%u : X %s, Y %s, %u/%u button(s)",
             bind->report_id, bind->axis_x.field >= 0 ? "yes" : "MISSING",
             bind->axis_y.field >= 0 ? "yes" : "MISSING", bind->button_count,
             VX_GAMEPAD_BUTTONS);

    if (bind->axis_x.field >= 0) {
        ESP_LOGI(t, "  X range %ld..%ld", (long)bind->axis_x.logical_min,
                 (long)bind->axis_x.logical_max);
    }
    if (bind->axis_y.field >= 0) {
        ESP_LOGI(t, "  Y range %ld..%ld", (long)bind->axis_y.logical_min,
                 (long)bind->axis_y.logical_max);
    }

    /* What the gamepad offers that we chose not to use.  On an unknown pad this
     * is the line that explains why a control does nothing — and it is
     * information, not a fault: the scope is 2 axes and 4 buttons. */
    for (uint8_t i = 0; i < bind->desc->report_count; ++i) {
        const vx_hid_report_t *rep = &bind->desc->reports[i];

        if (rep->type != VX_HID_REPORT_INPUT) {
            continue;
        }
        for (uint8_t j = 0; j < rep->field_count; ++j) {
            const vx_hid_field_t *f = &bind->desc->fields[rep->field_first + j];

            if ((f->flags & VX_HID_FLAG_CONSTANT) || f->usage == 0) {
                continue;
            }
            if (rep->report_id == bind->report_id) {
                if (f->usage_page == VX_HID_PAGE_GENERIC_DESKTOP &&
                    (f->usage == VX_HID_USAGE_X || f->usage == VX_HID_USAGE_Y)) {
                    continue;
                }
                if (f->usage_page == VX_HID_PAGE_BUTTON &&
                    f->usage <= VX_GAMEPAD_BUTTONS) {
                    continue;
                }
            }
            ESP_LOGD(t, "  unused: id=%u page %02x usage %02x %s",
                     rep->report_id, f->usage_page, f->usage,
                     vx_hid_usage_name(f->usage_page, f->usage));
        }
    }
}
