// usb.i
//
// USB HID Host for ESP32-P4: Keyboard + Joystick/Gamepad
// Managed component required: espressif/usb_host_hid
//
// Call from app_main():
//     usb_keyboard_init();
//
// ═══════════════════════════════════════════════════════════════════════════
//  HOW HID JOYSTICK DETECTION WORKS (Approach 2 — descriptor parsing)
// ═══════════════════════════════════════════════════════════════════════════
//
// Every USB HID device carries a "Report Descriptor" — a compact byte stream
// that precisely describes the layout of every data packet the device will
// send.  It is structured as a sequence of tagged items, each specifying:
//
//   • Which logical category the data belongs to (Usage Page)
//       0x01 = Generic Desktop (joystick axes, hat switches, etc.)
//       0x09 = Buttons
//   • What the specific value means (Usage)
//       0x30 = X axis,  0x31 = Y axis,  0x32 = Z axis, etc.
//   • The value range  (Logical Minimum / Logical Maximum)
//       e.g. 0..255 for an 8-bit unsigned axis, or -32768..32767 for 16-bit
//   • How many bits each value occupies (Report Size) and how many
//     consecutive values follow (Report Count)
//   • Whether a Report ID byte precedes each packet (Report ID)
//       When non-zero, the first byte of every incoming report is the ID
//       so the host knows which descriptor block applies.
//
// ─── Parsing strategy ────────────────────────────────────────────────────
//
// We walk the descriptor once on device connect and extract three things:
//
//   x_axis   — bit offset + bit count + logical range for the X field
//   y_axis   — same for Y
//   buttons  — bit offset of the first button + number of buttons
//
// "Bit offset" counts from the start of the payload (after the Report ID
// byte, if any).  As we encounter each Input item we accumulate the running
// offset: offset += report_count * report_size.
//
// When a Report ID global item appears the running offset resets to zero,
// because each Report ID defines an independent packet layout.  We record
// which Report ID contains our axes so we can ignore packets belonging to
// other reports (e.g. force-feedback status reports).
//
// ─── Sign handling ────────────────────────────────────────────────────────
//
// HID axis values may be unsigned (Logical Min = 0, Max = 255) or signed
// (Logical Min = -128, Max = 127), and any bit width from 4 to 16 bits is
// common.  We detect the signed case (logical_min < 0) and sign-extend the
// raw bit field before scaling.
//
// ─── Scaling to -128 .. +127 ─────────────────────────────────────────────
//
//   scaled = ((raw - logical_min) * 255 / (logical_max - logical_min)) - 128
//
// This maps the full logical range linearly onto the Vectrex signed-byte
// convention.  A dead zone of ±JOY_DEAD_ZONE counts is applied around zero
// to suppress jitter when the stick rests at center.
//
// ─── Button mapping ───────────────────────────────────────────────────────
//
// HID buttons are typically packed one-bit per button, active-high.
// Vectrex expects zero-active (0 = pressed).  We invert the 8 lowest button
// bits and pack them into g_inputState.buttonState[3:0] (Port 0 buttons 1-4).
//
// ─── Keyboard and joystick coexistence ───────────────────────────────────
//
// Both devices use the same USB host stack and event queue.  When a device
// connects we check its protocol field:
//   HID_PROTOCOL_KEYBOARD  → existing keyboard path (boot protocol, fast)
//   HID_PROTOCOL_NONE      → parse descriptor; accept if X or Y axis found
//
// If a joystick is present its axis values overwrite g_inputState.j0_x/y
// on every incoming report.  The keyboard mappings still update buttonState
// and can also move the axes if no joystick is connected.
//
// Public API (declared in defines.h):
//   bool isUSBJoystickAvailable()  — true while a USB joystick is connected
//   int  getUSBAnalogX()           — last X reading, -128 .. +127
//   int  getUSBAnalogY()           — last Y reading, -128 .. +127
//   bool isKeyDown(uint8_t)     — HID keycode query
//   bool isAsciiDown(char)      — ASCII character query
// ═══════════════════════════════════════════════════════════════════════════

#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_err.h"
#include "esp_log.h"

#include "usb/usb_host.h"
#include "usb/hid_host.h"
#include "usb/hid_usage_keyboard.h"

// Dead zone applied to both axes: values in -JOY_DEAD_ZONE..+JOY_DEAD_ZONE
// are forced to zero.  Prevents integrator drift when stick is at rest.
#define JOY_DEAD_ZONE   8

// Maximum number of HID keys tracked simultaneously
#define KBD_MAX_KEYS    256

// ─── HID Usage constants ──────────────────────────────────────────────────
#define HID_USAGE_PAGE_GENERIC_DESKTOP  0x01u
#define HID_USAGE_PAGE_BUTTON           0x09u

#define HID_USAGE_X     0x30u
#define HID_USAGE_Y     0x31u
#define HID_USAGE_Z     0x32u
#define HID_USAGE_RX    0x33u
#define HID_USAGE_RY    0x34u
#define HID_USAGE_RZ    0x35u

// ═══════════════════════════════════════════════════════════════════════════
//  HID Report Descriptor parser
// ═══════════════════════════════════════════════════════════════════════════

// One parsed field from the descriptor (axis or button block)
typedef struct {
    int     bit_offset;   // bit position within the report payload
    int     bit_count;    // width of the field in bits
    int32_t logical_min;
    int32_t logical_max;
    uint8_t report_id;    // which Report ID packet carries this field (0 = no IDs)
    bool    valid;        // true if this field was found in the descriptor
} hid_field_t;

// Parsed layout for a joystick / gamepad
typedef struct {
    hid_field_t x_axis;
    hid_field_t y_axis;
    hid_field_t buttons;  // bit_offset = first button bit; bit_count = number of buttons
} joy_layout_t;

// ─── Helper: read 1/2/4 bytes as signed ──────────────────────────────────
static int32_t hid_read_signed(const uint8_t *p, int size)
{
    switch (size) {
        case 1: return (int32_t)(int8_t)  p[0];
        case 2: return (int32_t)(int16_t)(p[0] | ((uint16_t)p[1] << 8));
        case 4: return (int32_t)          (p[0] | ((uint32_t)p[1]<<8)
                                                 | ((uint32_t)p[2]<<16)
                                                 | ((uint32_t)p[3]<<24));
        default: return 0;
    }
}

// ─── Helper: read 1/2/4 bytes as unsigned ────────────────────────────────
static uint32_t hid_read_unsigned(const uint8_t *p, int size)
{
    switch (size) {
        case 1: return p[0];
        case 2: return p[0] | ((uint32_t)p[1] << 8);
        case 4: return p[0] | ((uint32_t)p[1]<<8)
                            | ((uint32_t)p[2]<<16)
                            | ((uint32_t)p[3]<<24);
        default: return 0;
    }
}

// ─── Core parser ─────────────────────────────────────────────────────────
//
// Walks every short item in the HID report descriptor.
// Item encoding (HID spec 6.2.2.2):
//
//   Byte 0:  [ tag(7:4) | type(3:2) | size(1:0) ]
//     type 0 = Main   (Input, Output, Feature, Collection, End Collection)
//     type 1 = Global (Usage Page, Logical Min/Max, Report Size/Count/ID)
//     type 2 = Local  (Usage, Usage Min/Max)
//     size:  0→0 bytes, 1→1 byte, 2→2 bytes, 3→4 bytes
//
// Global state persists across items; local state resets after each Main item.
// The running bit_offset resets to 0 whenever the Report ID changes.
//
static void parse_hid_descriptor(const uint8_t *desc, size_t len, joy_layout_t *out)
{
    memset(out, 0, sizeof(*out));

    // Global state (carries forward until overwritten)
    uint32_t usage_page     = 0;
    int32_t  logical_min    = 0;
    int32_t  logical_max    = 0;
    uint32_t report_size    = 0;   // bits per individual value
    uint32_t report_count   = 0;   // number of values in this Input/Output group
    uint8_t  current_rid    = 0;   // current Report ID (0 = none)

    // Local state (resets after each Main item)
    uint32_t usages[32];
    int      usage_count    = 0;
    uint32_t usage_min      = 0;
    uint32_t usage_max      = 0;
    bool     has_usage_range = false;

    // Running bit offset within the current report (resets on Report ID change)
    int bit_offset = 0;

    size_t i = 0;
    while (i < len) {
        uint8_t prefix = desc[i++];

        // Long item marker (0xFE) — skip; we don't need them
        if (prefix == 0xFE) {
            if (i < len) { uint8_t dsz = desc[i++]; i += 1 + dsz; }
            continue;
        }

        // Decode prefix
        int raw_size  = prefix & 0x03;
        int item_size = (raw_size == 3) ? 4 : raw_size;  // size code 3 → 4 bytes
        int item_type = (prefix >> 2) & 0x03;
        int item_tag  = (prefix >> 4) & 0x0F;

        if (i + item_size > (int)len) break;
        const uint8_t *data = &desc[i];
        i += item_size;

        uint32_t uval = item_size ? hid_read_unsigned(data, item_size) : 0;
        int32_t  sval = item_size ? hid_read_signed(data, item_size)   : 0;

        switch (item_type) {

        // ── Global items ─────────────────────────────────────────────────
        case 1:
            switch (item_tag) {
            case 0: usage_page   = uval; break;
            case 1: logical_min  = sval; break;
            case 2: logical_max  = sval; break;
            case 7: report_size  = uval; break;
            case 9: report_count = uval; break;
            case 8: // Report ID
                if ((uint8_t)uval != current_rid) {
                    // New report context — reset the running bit offset.
                    // Each Report ID defines an independent packet layout
                    // starting at bit 0.
                    current_rid = (uint8_t)uval;
                    bit_offset  = 0;
                }
                break;
            }
            break;

        // ── Local items ──────────────────────────────────────────────────
        case 2:
            switch (item_tag) {
            case 0: // Usage
                if (usage_count < 32) usages[usage_count++] = uval;
                break;
            case 1: usage_min = uval; has_usage_range = true; break;
            case 2: usage_max = uval; has_usage_range = true; break;
            }
            break;

        // ── Main items ───────────────────────────────────────────────────
        case 0:
            switch (item_tag) {

            case 8: { // Input
                // Bit 0 of the Input flags: 0 = Data, 1 = Constant (padding)
                bool is_padding = (uval & 0x01) != 0;
                uint32_t total_bits = report_count * report_size;

                if (!is_padding) {
                    // Expand usage list: if Usage Min/Max was given expand the range;
                    // otherwise use the explicit usage array.  When fewer usages than
                    // report_count are listed, the last usage repeats (HID spec rule).
                    uint32_t eff[32];
                    int      eff_n = 0;

                    if (has_usage_range) {
                        for (uint32_t u = usage_min; u <= usage_max && eff_n < 32; u++)
                            eff[eff_n++] = u;
                    } else {
                        for (int u = 0; u < usage_count && u < 32; u++)
                            eff[eff_n++] = usages[u];
                    }

                    if (usage_page == HID_USAGE_PAGE_GENERIC_DESKTOP) {
                        // Walk each field in this Input group
                        for (uint32_t f = 0; f < report_count; f++) {
                            uint32_t usage = (f < (uint32_t)eff_n) ? eff[f]
                                           : (eff_n > 0 ? eff[eff_n-1] : 0);

                            if (usage == HID_USAGE_X && !out->x_axis.valid) {
                                out->x_axis.bit_offset  = bit_offset + (int)(f * report_size);
                                out->x_axis.bit_count   = (int)report_size;
                                out->x_axis.logical_min = logical_min;
                                out->x_axis.logical_max = logical_max;
                                out->x_axis.report_id   = current_rid;
                                out->x_axis.valid       = true;
                                ESP_LOGI(TAG, "JOY: X axis  offset=%d bits=%d range=%d..%d rid=%d",
                                         out->x_axis.bit_offset, out->x_axis.bit_count,
                                         logical_min, logical_max, current_rid);
                            } else if (usage == HID_USAGE_Y && !out->y_axis.valid) {
                                out->y_axis.bit_offset  = bit_offset + (int)(f * report_size);
                                out->y_axis.bit_count   = (int)report_size;
                                out->y_axis.logical_min = logical_min;
                                out->y_axis.logical_max = logical_max;
                                out->y_axis.report_id   = current_rid;
                                out->y_axis.valid       = true;
                                ESP_LOGI(TAG, "JOY: Y axis  offset=%d bits=%d range=%d..%d rid=%d",
                                         out->y_axis.bit_offset, out->y_axis.bit_count,
                                         logical_min, logical_max, current_rid);
                            }
                        }
                    } else if (usage_page == HID_USAGE_PAGE_BUTTON) {
                        // First button block wins; record position and count
                        if (!out->buttons.valid) {
                            out->buttons.bit_offset  = bit_offset;
                            out->buttons.bit_count   = (int)report_count;
                            out->buttons.logical_min = logical_min;
                            out->buttons.logical_max = logical_max;
                            out->buttons.report_id   = current_rid;
                            out->buttons.valid       = true;
                            ESP_LOGI(TAG, "JOY: Buttons offset=%d count=%d rid=%d",
                                     bit_offset, (int)report_count, current_rid);
                        }
                    }
                } // !is_padding

                bit_offset += (int)total_bits;
                break;
            }

            case 9:  // Output — advance offset but we don't use these
                bit_offset += (int)(report_count * report_size);
                break;

            case 10: // Collection — no bit offset effect
            case 12: // End Collection — no bit offset effect
                break;
            }

            // ── Reset local state after every Main item ──────────────────
            usage_count     = 0;
            usage_min       = 0;
            usage_max       = 0;
            has_usage_range = false;
            memset(usages, 0, sizeof(usages));
            break;

        default:
            break; // reserved item type
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Joystick runtime state
// ═══════════════════════════════════════════════════════════════════════════

DRAM_ATTR static joy_layout_t          s_joy_layout       = {0};
DRAM_ATTR static volatile bool         s_joystick_present = false;
DRAM_ATTR static volatile int8_t       s_joy_x            = 0;
DRAM_ATTR static volatile int8_t       s_joy_y            = 0;
DRAM_ATTR static hid_host_device_handle_t s_joy_dev_handle = NULL;

// ─── Extract an arbitrary bit field from a report byte array ─────────────
//
// HID fields are packed LSB-first within bytes.
// Example: a 10-bit field at bit_offset=8 lives in bytes[1] bits[1:0]
//          and bytes[2] bits[7:0].
//
IRAM_ATTR static inline int32_t extract_bits(const uint8_t *report, int bit_offset, int bit_count)
{
    uint32_t value = 0;
    for (int b = 0; b < bit_count; b++) {
        int byte_idx = (bit_offset + b) / 8;
        int bit_idx  = (bit_offset + b) % 8;
        if ((report[byte_idx] >> bit_idx) & 1u)
            value |= (1u << b);
    }
    return (int32_t)value;
}

// ─── Scale a raw HID value to the signed-byte range -128 .. +127 ─────────
//
// Handles both unsigned axes (logical_min = 0) and signed axes
// (logical_min < 0, requiring sign extension of the raw bit field).
//
IRAM_ATTR inline static int8_t scale_axis(int32_t raw, int32_t log_min, int32_t log_max, int bit_count)
{
    // Sign-extend if the descriptor declares a signed range
    if (log_min < 0) {
        int32_t sign_bit = 1 << (bit_count - 1);
        if (raw & sign_bit)
            raw |= ~((sign_bit << 1) - 1);  // fill upper bits with 1s
    }

    // Clamp to declared logical range
    if (raw < log_min) raw = log_min;
    if (raw > log_max) raw = log_max;

    int32_t range = log_max - log_min;
    if (range <= 0) return 0;

    // Linear map:  log_min → -128,  log_max → +127
    int32_t scaled = (int32_t)(((int64_t)(raw - log_min) * 255) / range) - 128;
    if (scaled < -128) scaled = -128;
    if (scaled >  127) scaled =  127;
    return (int8_t)scaled;
}

// ─── Process one incoming HID report from the joystick ───────────────────
IRAM_ATTR static void joy_report_callback(const uint8_t *data, int len)
{
    const joy_layout_t *lay = &s_joy_layout;

    // If the device uses Report IDs the first byte of every packet is the
    // Report ID.  We skip it when accessing bit offsets (which were computed
    // relative to the payload that follows the ID byte), and we ignore
    // packets whose ID doesn't match the axis report.
    const uint8_t *payload = data;
    int            plen    = len;

    if (lay->x_axis.report_id != 0) {
        if (plen < 2) return;
        if (data[0] != lay->x_axis.report_id) return;  // not the axis report
        payload++;
        plen--;
    }

    if (lay->x_axis.valid) {
        int32_t raw = extract_bits(payload, lay->x_axis.bit_offset, lay->x_axis.bit_count);
        int8_t  v   = scale_axis(raw, lay->x_axis.logical_min,
                                  lay->x_axis.logical_max, lay->x_axis.bit_count);
        if (v > -JOY_DEAD_ZONE && v < JOY_DEAD_ZONE) v = 0;
        s_joy_x = v;
    }

    if (lay->y_axis.valid) {
        int32_t raw = extract_bits(payload, lay->y_axis.bit_offset, lay->y_axis.bit_count);
        int8_t  v   = scale_axis(raw, lay->y_axis.logical_min,
                                  lay->y_axis.logical_max, lay->y_axis.bit_count);
        if (v > -JOY_DEAD_ZONE && v < JOY_DEAD_ZONE) v = 0;
        s_joy_y = v;
    }

    if (lay->buttons.valid) {
        // Pack up to 8 buttons into a byte; HID buttons are active-high,
        // Vectrex expects zero-active (0 = pressed) → invert.
        int count = lay->buttons.bit_count;
        if (count > 8) count = 8;
        uint8_t btns = 0;
        for (int b = 0; b < count; b++) {
            if (extract_bits(payload, lay->buttons.bit_offset + b, 1))
                btns |= (uint8_t)(1u << b);
        }
        // Invert so zero = pressed; keep Port-1 buttons in upper nibble intact
        uint8_t vectrex = ~btns;
        miniUSBButton = (vectrex & 0x0F);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Keyboard state (unchanged from original implementation)
// ═══════════════════════════════════════════════════════════════════════════

typedef struct {
    hid_host_device_handle_t handle;
    hid_host_driver_event_t  event;
    void                    *arg;
} kbd_hid_evt_t;

DRAM_ATTR static QueueHandle_t s_kbd_evt_queue = NULL;

typedef struct {
    enum { KEY_STATE_PRESSED = 0x00, KEY_STATE_RELEASED = 0x01 } state;
    uint8_t modifier;
    uint8_t key_code;
} key_event_t;

#define KEYBOARD_ENTER_MAIN_CHAR  '\r'
#define KEYBOARD_ENTER_LF_EXTEND  1

static volatile bool    key_state[KBD_MAX_KEYS];
static volatile uint8_t key_modifiers = 0;

static const uint8_t keycode2ascii[57][2] = {
    {0, 0}, {0, 0}, {0, 0}, {0, 0},
    {'a','A'}, {'b','B'}, {'c','C'}, {'d','D'}, {'e','E'}, {'f','F'},
    {'g','G'}, {'h','H'}, {'i','I'}, {'j','J'}, {'k','K'}, {'l','L'},
    {'m','M'}, {'n','N'}, {'o','O'}, {'p','P'}, {'q','Q'}, {'r','R'},
    {'s','S'}, {'t','T'}, {'u','U'}, {'v','V'}, {'w','W'}, {'x','X'},
    {'y','Y'}, {'z','Z'},
    {'1','!'}, {'2','@'}, {'3','#'}, {'4','$'}, {'5','%'},
    {'6','^'}, {'7','&'}, {'8','*'}, {'9','('}, {'0',')'},
    {KEYBOARD_ENTER_MAIN_CHAR, KEYBOARD_ENTER_MAIN_CHAR},
    {0,0}, {'\b',0}, {0,0}, {' ',' '},
    {'-','_'}, {'=','+'}, {'[','{'}, {']','}'}, {'\\','|'}, {'\\','|'},
    {';',':'}, {'\'','"'}, {'`','~'}, {',','<'}, {'.','>'}, {'/','?'}
};

static inline bool kbd_is_shift(uint8_t modifier)
{
    return ((modifier & HID_LEFT_SHIFT)  == HID_LEFT_SHIFT) ||
           ((modifier & HID_RIGHT_SHIFT) == HID_RIGHT_SHIFT);
}

static inline bool kbd_scancode_to_char(uint8_t modifier, uint8_t key_code,
                                        unsigned char *out_char)
{
    if (key_code < HID_KEY_A || key_code > HID_KEY_SLASH) return false;
    uint8_t mod = kbd_is_shift(modifier) ? 1 : 0;
    *out_char = keycode2ascii[key_code][mod];
    return (*out_char != 0);
}

static inline void kbd_print_char(unsigned char c)
{
    if (!c) return;
    putchar(c);
#if KEYBOARD_ENTER_LF_EXTEND
    if (c == KEYBOARD_ENTER_MAIN_CHAR) putchar('\n');
#endif
    fflush(stdout);
}

static void kbd_report_callback(const uint8_t *data, int length)
{
    if (length < (int)sizeof(hid_keyboard_input_report_boot_t)) return;

    const hid_keyboard_input_report_boot_t *kb =
        (const hid_keyboard_input_report_boot_t *)data;

    static uint8_t prev_keys[HID_KEYBOARD_KEY_MAX] = {0};

    key_modifiers = kb->modifier.val;

    for (int i = 0; i < HID_KEYBOARD_KEY_MAX; ++i) {
        uint8_t old_key = prev_keys[i];
        if (old_key > HID_KEY_ERROR_UNDEFINED) {
            bool still_present = false;
            for (int j = 0; j < HID_KEYBOARD_KEY_MAX; ++j) {
                if (kb->key[j] == old_key) { still_present = true; break; }
            }
            if (!still_present) key_state[old_key] = false;
        }
    }

    for (int i = 0; i < HID_KEYBOARD_KEY_MAX; ++i) {
        uint8_t new_key = kb->key[i];
        if (new_key > HID_KEY_ERROR_UNDEFINED) {
            key_state[new_key] = true;
            unsigned char ch;
            if (kbd_scancode_to_char(kb->modifier.val, new_key, &ch))
                kbd_print_char(ch);
        }
    }

    memcpy(prev_keys, kb->key, HID_KEYBOARD_KEY_MAX);
}

// ═══════════════════════════════════════════════════════════════════════════
//  HID interface callback — called for every incoming report
// ═══════════════════════════════════════════════════════════════════════════
//
// A single function handles all connected HID devices.  We dispatch to the
// correct sub-handler based on the protocol that was recorded at connect time.
//
static void hid_interface_callback(hid_host_device_handle_t hid_dev,
                                   const hid_host_interface_event_t event,
                                   void *arg)
{
    (void)arg;
    uint8_t buf[64];
    size_t  len = 0;
    hid_host_dev_params_t params;

    if (hid_host_device_get_params(hid_dev, &params) != ESP_OK) return;

    switch (event) {
    case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:

        if (params.proto == HID_PROTOCOL_KEYBOARD &&
            params.sub_class == HID_SUBCLASS_BOOT_INTERFACE)
        {
            // ── Keyboard: use the fast boot-protocol path ────────────────
            if (hid_host_device_get_raw_input_report_data(
                    hid_dev, buf, sizeof(buf), &len) == ESP_OK && len > 0)
                kbd_report_callback(buf, (int)len);
        }
        else if (params.proto == HID_PROTOCOL_NONE && s_joystick_present &&
                 hid_dev == s_joy_dev_handle)
        {
            // ── Joystick: dispatch to the descriptor-driven handler ───────
            if (hid_host_device_get_raw_input_report_data(
                    hid_dev, buf, sizeof(buf), &len) == ESP_OK && len > 0)
                joy_report_callback(buf, (int)len);
        }
        break;

    case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
        if (hid_dev == s_joy_dev_handle) {
            ESP_LOGI(TAG, "Joystick DISCONNECTED");
            s_joystick_present = false;
            s_joy_dev_handle   = NULL;
            s_joy_x = 0;
            s_joy_y = 0;
            g_inputState.j0_x = 0;
            g_inputState.j0_y = 0;
        } else {
            ESP_LOGI(TAG, "Keyboard DISCONNECTED");
        }
        hid_host_device_close(hid_dev);
        break;

    case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
        ESP_LOGW(TAG, "HID TRANSFER_ERROR (proto=%d)", params.proto);
        break;

    default:
        ESP_LOGW(TAG, "Unhandled interface event: %d", event);
        break;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Device event handler — runs in the event queue task
// ═══════════════════════════════════════════════════════════════════════════
static void hid_device_event_handler(hid_host_device_handle_t hid_dev,
                                     const hid_host_driver_event_t event,
                                     void *arg)
{
    (void)arg;
    if (!s_kbd_evt_queue) return;
    kbd_hid_evt_t evt = { .handle = hid_dev, .event = event, .arg = NULL };
    xQueueSend(s_kbd_evt_queue, &evt, 0);
}

// ─── Open a newly connected device and parse its descriptor if needed ─────
static void kbd_handle_device_event(const kbd_hid_evt_t *evt)
{
    hid_host_dev_params_t params;
    if (hid_host_device_get_params(evt->handle, &params) != ESP_OK) return;

    switch (evt->event) {
    case HID_HOST_DRIVER_EVENT_CONNECTED: {
        ESP_LOGI(TAG, "HID device connected — proto=%d subclass=%d",
                 params.proto, params.sub_class);

        // Common open config for all devices
        const hid_host_device_config_t dev_cfg = {
            .callback     = hid_interface_callback,
            .callback_arg = NULL,
        };

        if (params.proto == HID_PROTOCOL_KEYBOARD) {
            // ── Keyboard path ────────────────────────────────────────────
            ESP_ERROR_CHECK(hid_host_device_open(evt->handle, &dev_cfg));
            if (params.sub_class == HID_SUBCLASS_BOOT_INTERFACE) {
                ESP_ERROR_CHECK(hid_class_request_set_protocol(
                                    evt->handle, HID_REPORT_PROTOCOL_BOOT));
                ESP_ERROR_CHECK(hid_class_request_set_idle(evt->handle, 0, 0));
            }
            ESP_ERROR_CHECK(hid_host_device_start(evt->handle));
            ESP_LOGI(TAG, "Keyboard ready");

        } else if (params.proto == HID_PROTOCOL_NONE) {
            // ── Generic HID path: attempt joystick descriptor parse ──────
            //
            // The interface must be opened before the driver will fetch the
            // report descriptor from the device.  So we open first, read the
            // descriptor, parse it, and only then decide whether to start
            // receiving reports.  If no axes are found we close again.
            ESP_ERROR_CHECK(hid_host_device_open(evt->handle, &dev_cfg));

            size_t   dlen = 0;
            uint8_t *desc = hid_host_get_report_descriptor(evt->handle, &dlen);
            if (!desc || dlen == 0) {
                ESP_LOGW(TAG, "Could not get HID descriptor — ignoring device");
                hid_host_device_close(evt->handle);
                break;
            }

            ESP_LOGI(TAG, "Parsing HID descriptor (%u bytes)...", (unsigned)dlen);
            joy_layout_t layout = {0};
            parse_hid_descriptor(desc, dlen, &layout);

            if (!layout.x_axis.valid && !layout.y_axis.valid) {
                ESP_LOGI(TAG, "No joystick axes found — ignoring device");
                hid_host_device_close(evt->handle);
                break;
            }

            // Save parsed layout and mark joystick as available
            s_joy_layout       = layout;
            s_joystick_present = true;
            s_joy_dev_handle   = evt->handle;

            ESP_ERROR_CHECK(hid_host_device_start(evt->handle));
            ESP_LOGI(TAG, "Joystick ready (X=%s Y=%s Buttons=%s)",
                     layout.x_axis.valid   ? "yes" : "no",
                     layout.y_axis.valid   ? "yes" : "no",
                     layout.buttons.valid  ? "yes" : "no");
        } else {
            ESP_LOGI(TAG, "Unknown HID protocol %d — ignoring", params.proto);
        }
        break;
    }
    default:
        break;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Tasks
// ═══════════════════════════════════════════════════════════════════════════

static void usb_keyboard_task(void *arg)
{
    (void)arg;
    kbd_hid_evt_t evt;
    ESP_LOGI(TAG, "USB HID event task running");
    while (1) {
        if (xQueueReceive(s_kbd_evt_queue, &evt, portMAX_DELAY) == pdTRUE)
            kbd_handle_device_event(&evt);
    }
}

static void usb_host_event_task(void *arg)
{
    (void)arg;
    uint32_t event_flags;
    while (1) {
        esp_err_t err = usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (err != ESP_OK)
            ESP_LOGE("usb_host", "usb_host_lib_handle_events failed: 0x%x", err);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Public API
// ═══════════════════════════════════════════════════════════════════════════

bool isKeyDown(uint8_t hid_code)
{
    return key_state[hid_code];
}

bool isAsciiDown(char c)
{
    if (c >= 'A' && c <= 'Z') c = c + 32;
    for (uint8_t code = HID_KEY_A; code <= HID_KEY_Z; code++) {
        if (isKeyDown(code)) {
            char ch = 'a' + (code - HID_KEY_A);
            if (c == ch) return true;
        }
    }
    return false;
}

bool isShiftDown(void) { return (key_modifiers & (HID_LEFT_SHIFT  | HID_RIGHT_SHIFT))   != 0; }
bool isCtrlDown(void)  { return (key_modifiers & (HID_LEFT_CONTROL| HID_RIGHT_CONTROL)) != 0; }
bool isAltDown(void)   { return (key_modifiers & (HID_LEFT_ALT    | HID_RIGHT_ALT))     != 0; }

// Returns true while a USB joystick/gamepad with X or Y axes is connected
IRAM_ATTR bool isUSBJoystickAvailable(void)
{
    return s_joystick_present;
}

// Last X axis reading: -128 (full left) .. 0 (center) .. +127 (full right)
IRAM_ATTR int getUSBAnalogX(void)
{
    return (int)s_joy_x;
}

// Last Y axis reading: -128 (full up/back) .. 0 (center) .. +127 (full down/forward)
IRAM_ATTR int getUSBAnalogY(void)
{
    return (int)s_joy_y;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Initialisation
// ═══════════════════════════════════════════════════════════════════════════

esp_err_t usb_keyboard_init(void)
{
    esp_err_t err;

    s_kbd_evt_queue = xQueueCreate(10, sizeof(kbd_hid_evt_t));
    if (!s_kbd_evt_queue) {
        ESP_LOGE(TAG, "Failed to create HID event queue");
        return ESP_FAIL;
    }

    // 1. Install USB Host library
    const usb_host_config_t host_cfg = {
        .skip_phy_setup = false,
        .intr_flags     = ESP_INTR_FLAG_LEVEL1,
    };
    err = usb_host_install(&host_cfg);
    if (err != ESP_OK) { ESP_LOGE(TAG, "usb_host_install: 0x%x", err); return err; }
    ESP_LOGI(TAG, "USB Host installed");

    // 2. Task that pumps usb_host_lib_handle_events() — must run on Core 0
    //    because the USB peripheral DMA controller is attached to Core 0.
    if (xTaskCreatePinnedToCore(usb_host_event_task, "usb_host_evt",
                                4096, NULL, 5, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create usb_host_event_task");
        return ESP_FAIL;
    }

    // 3. Install HID Host driver
    const hid_host_driver_config_t hid_cfg = {
        .create_background_task = true,
        .task_priority          = 5,
        .stack_size             = 4096,
        .core_id                = 0,
        .callback               = hid_device_event_handler,
        .callback_arg           = NULL,
    };
    err = hid_host_install(&hid_cfg);
    if (err != ESP_OK) { ESP_LOGE(TAG, "hid_host_install: 0x%x", err); return err; }
    ESP_LOGI(TAG, "HID Host installed");

    // 4. Task that processes device connect/disconnect events from the queue
    if (xTaskCreatePinnedToCore(usb_keyboard_task, "usb_hid_task",
                                4096, NULL, 4, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create usb_hid_task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "USB HID ready — connect keyboard or joystick");
    return ESP_OK;
}
