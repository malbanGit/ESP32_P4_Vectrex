/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * HID report descriptor parser.
 *
 * Why this exists: IDF's esp_hid_parse_report_map() only returns
 * {report_id, type, usage, value_len} per report (esp_hid_common.h:151-158).
 * It never tells you WHERE a control sits inside the report, so it cannot be
 * used to decode an arbitrary gamepad.  Anything that hardcodes byte offsets
 * (as the ODM test code does: report_id == 2 && len == 3) only ever works with
 * the one device it was written against.
 *
 * This parser walks the raw descriptor and produces, per report ID and report
 * type, the list of fields with their exact bit offset, bit size and logical
 * range.  That is what makes an unknown off-the-shelf gamepad usable.
 *
 * Deliberately free of any BLE/IDF dependency beyond esp_log, so that it can be
 * unit-tested and moved into components/vx_ble_gamepad/ unchanged.
 *
 * Reference: USB Device Class Definition for HID 1.11, section 6.2.2.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Capacity limits.  A generous commercial gamepad (16 buttons, 6 axes, one hat,
 * plus a vendor report) stays well under these. */
#define VX_HID_MAX_REPORTS 12
#define VX_HID_MAX_FIELDS  96

/* HID usage pages we care about. */
#define VX_HID_PAGE_GENERIC_DESKTOP 0x01
#define VX_HID_PAGE_SIMULATION      0x02
#define VX_HID_PAGE_BUTTON          0x09
#define VX_HID_PAGE_CONSUMER        0x0C

/* Generic Desktop usages. */
#define VX_HID_USAGE_X            0x30
#define VX_HID_USAGE_Y            0x31
#define VX_HID_USAGE_Z            0x32
#define VX_HID_USAGE_RX           0x33
#define VX_HID_USAGE_RY           0x34
#define VX_HID_USAGE_RZ           0x35
#define VX_HID_USAGE_SLIDER       0x36
#define VX_HID_USAGE_DIAL         0x37
#define VX_HID_USAGE_WHEEL        0x38
#define VX_HID_USAGE_HAT_SWITCH   0x39

/* Simulation usages seen on gamepad triggers. */
#define VX_HID_USAGE_BRAKE        0xC5
#define VX_HID_USAGE_ACCELERATOR  0xC4

typedef enum {
    VX_HID_REPORT_INPUT = 0,
    VX_HID_REPORT_OUTPUT,
    VX_HID_REPORT_FEATURE,
    VX_HID_REPORT_TYPE_COUNT,
} vx_hid_report_type_t;

/* Flags from the Main item data byte (HID 1.11 section 6.2.2.5). */
#define VX_HID_FLAG_CONSTANT (1u << 0) /* padding: no usage, still takes bits  */
#define VX_HID_FLAG_VARIABLE (1u << 1) /* clear = Array (index), set = Variable */
#define VX_HID_FLAG_RELATIVE (1u << 2) /* clear = Absolute                      */

typedef struct {
    uint16_t usage_page;
    uint16_t usage;       /* 0 for padding / unnamed fields                   */
    uint16_t bit_offset;  /* from the start of the report payload             */
    uint8_t  bit_size;
    uint8_t  flags;       /* VX_HID_FLAG_*                                    */
    int32_t  logical_min;
    int32_t  logical_max;
} vx_hid_field_t;

typedef struct {
    uint8_t  report_id;   /* 0 when the descriptor declares no Report ID      */
    uint8_t  type;        /* vx_hid_report_type_t                             */
    uint16_t bit_len;     /* total payload length in bits                     */
    uint8_t  field_first; /* index into vx_hid_desc_t.fields                  */
    uint8_t  field_count;
} vx_hid_report_t;

typedef struct {
    bool            uses_report_ids; /* false = payload has no leading ID byte */
    uint8_t         report_count;
    uint8_t         field_count;
    vx_hid_report_t reports[VX_HID_MAX_REPORTS];
    vx_hid_field_t  fields[VX_HID_MAX_FIELDS];
    /* Set when the descriptor was larger than our capacity: the parse result is
     * usable but incomplete, and the caller should say so loudly rather than
     * silently mis-decode. */
    bool            truncated;
} vx_hid_desc_t;

/**
 * @brief Parse a raw HID report descriptor.
 *
 * @param raw   descriptor bytes as read from characteristic 0x2A4B
 * @param len   length in bytes
 * @param out   caller-provided, zeroed by this function
 * @return true on success (possibly with out->truncated set), false if the
 *         descriptor is malformed beyond recovery.
 */
bool vx_hid_desc_parse(const uint8_t *raw, size_t len, vx_hid_desc_t *out);

/**
 * @brief Find a parsed report by ID and type.
 * @return NULL if absent.
 */
const vx_hid_report_t *vx_hid_desc_find_report(const vx_hid_desc_t *desc,
                                               uint8_t report_id,
                                               vx_hid_report_type_t type);

/**
 * @brief Extract one field's value from a report payload.
 *
 * Handles arbitrary bit offsets and sizes (a hat switch is typically 4 bits
 * straddling nothing, but trigger axes are routinely 10 or 12 bits and DO
 * straddle byte boundaries).  Sign-extends when logical_min < 0.
 *
 * @param payload  report payload, WITHOUT the leading report-ID byte
 * @param len      payload length in bytes
 * @return true if the field fits inside the payload.
 */
bool vx_hid_field_extract(const uint8_t *payload, size_t len,
                          const vx_hid_field_t *field, int32_t *out);

/**
 * @brief Dump a parsed descriptor to the log, one line per field.
 *
 * This is the first thing to look at when a new gamepad misbehaves: it shows
 * exactly what we understood of it.
 */
void vx_hid_desc_dump(const vx_hid_desc_t *desc, const char *tag);

/** @brief Human-readable usage name, for the dump. */
const char *vx_hid_usage_name(uint16_t page, uint16_t usage);

#ifdef __cplusplus
}
#endif
