// bt_joy.i
// BLE HID joystick host for ESP32-P4 + C6 via esp-hosted SDIO.
// NimBLE runs on the P4; the C6 is the BLE radio controller (HCI over SDIO).
//
// Supports up to 2 simultaneous BLE HID joysticks.
// Parses the HID Report Descriptor exactly (same bit-field logic as usb.i).
// First device to connect → port 0 (j0_x / j0_y).
// Second device          → port 1 (j1_x / j1_y).
// Scanning continues until both slots are filled; rescans on disconnect.
//
// ─── sdkconfig.defaults additions required ───────────────────────────────
//   CONFIG_BT_ENABLED=y
//   CONFIG_BT_NIMBLE_ENABLED=y
//   CONFIG_BT_CONTROLLER_DISABLED=y        // P4 has no BT silicon
//   CONFIG_BT_NIMBLE_ROLE_CENTRAL=y
//   CONFIG_BT_NIMBLE_ROLE_OBSERVER=y
//   CONFIG_BT_NIMBLE_MAX_CONNECTIONS=2
//   CONFIG_ESP_HOSTED_ENABLED=y
//   CONFIG_ESP_HOSTED_SDIO_ENABLED=y
//   CONFIG_ESP_HOSTED_SLAVE_TARGET_ESP32C6=y
//   CONFIG_ESP_HOSTED_SDIO_CLK_GPIO=18
//   CONFIG_ESP_HOSTED_SDIO_CMD_GPIO=19
//   CONFIG_ESP_HOSTED_SDIO_D0_GPIO=14
//   CONFIG_ESP_HOSTED_SDIO_D1_GPIO=15
//   CONFIG_ESP_HOSTED_SDIO_D2_GPIO=16
//   CONFIG_ESP_HOSTED_SDIO_D3_GPIO=17
//   CONFIG_ESP_HOSTED_SLAVE_RESET_GPIO=54
//
// ─── idf_component.yml additions required ────────────────────────────────
//   dependencies:
//     espressif/esp_hosted: ">=2.0.0"
//
// ─── Public API (add to defines.h) ───────────────────────────────────────
//   void bt_joy_init(void);
//   bool isBTJoystickAvailable(int port);  // port 0 or 1
//   int  getBTAnalogX(int port);           // -128 .. +127
//   int  getBTAnalogY(int port);           // -128 .. +127
//
// ─── Call from app_main() ────────────────────────────────────────────────
//   nvs_flash_init();   // must be called before bt_joy_init if not done elsewhere
//   bt_joy_init();
//
// ─── Connecting two joysticks ────────────────────────────────────────────
//   Put both joysticks in pairing/discoverable mode before power-on, or
//   turn them on one at a time.  The driver scans continuously; the first
//   device advertising the HID service UUID (0x1812) gets port 0, the next
//   gets port 1.  Both are hot-plug: disconnect one and it rescans.
//   Assignment is first-come-first-served — there is no MAC pinning by
//   default.  To pin a specific controller to a specific port, compare
//   event->disc.addr in BLE_GAP_EVENT_DISC against a known MAC.
// ═════════════════════════════════════════════════════════════════════════

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_att.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"

#define BTJOY_TAG  "BTJOY"

#define BT_MAX_JOYSTICKS    2
#define BT_JOY_DEAD_ZONE    8
#define MAX_REPORT_HANDLES  8
#define MAX_DESC_LEN        512

// ─── GATT UUIDs ───────────────────────────────────────────────────────────
#define UUID16_HID_SVC      0x1812   // HID Service
#define UUID16_REPORT_MAP   0x2A4B   // Report Map (the descriptor)
#define UUID16_REPORT       0x2A4D   // HID Report (notification source)
#define UUID16_CCCD         0x2902   // Client Characteristic Configuration

// ─── HID Usage constants ──────────────────────────────────────────────────
#define BT_HID_UP_GENERIC_DESKTOP  0x01u
#define BT_HID_UP_BUTTON           0x09u
#define BT_HID_USAGE_X             0x30u
#define BT_HID_USAGE_Y             0x31u

// ═════════════════════════════════════════════════════════════════════════
//  HID Report Descriptor parser
//  Identical algorithm to the one in usb.i but with bt_ prefix on all types
//  and functions to avoid collisions when both files are included together.
// ═════════════════════════════════════════════════════════════════════════

typedef struct {
    int     bit_offset;
    int     bit_count;
    int32_t logical_min;
    int32_t logical_max;
    uint8_t report_id;
    bool    valid;
} bt_hid_field_t;

typedef struct {
    bt_hid_field_t x_axis;
    bt_hid_field_t y_axis;
    bt_hid_field_t buttons;
} bt_joy_layout_t;

static int32_t bt_hid_read_signed(const uint8_t *p, int size)
{
    switch (size) {
    case 1: return (int32_t)(int8_t)p[0];
    case 2: return (int32_t)(int16_t)(p[0] | ((uint16_t)p[1] << 8));
    case 4: return (int32_t)(p[0] | ((uint32_t)p[1]<<8)
                                   | ((uint32_t)p[2]<<16)
                                   | ((uint32_t)p[3]<<24));
    default: return 0;
    }
}

static uint32_t bt_hid_read_unsigned(const uint8_t *p, int size)
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

static void bt_parse_hid_descriptor(const uint8_t *desc, size_t len, bt_joy_layout_t *out)
{
    memset(out, 0, sizeof(*out));

    uint32_t usage_page = 0, report_size = 0, report_count = 0;
    int32_t  logical_min = 0, logical_max = 0;
    uint8_t  current_rid = 0;

    uint32_t usages[32];
    int      usage_count = 0;
    uint32_t usage_min = 0, usage_max = 0;
    bool     has_usage_range = false;

    int    bit_offset = 0;
    size_t i = 0;

    while (i < len) {
        uint8_t prefix = desc[i++];

        if (prefix == 0xFE) {  // long item — skip
            if (i < len) { uint8_t dsz = desc[i++]; i += 1 + dsz; }
            continue;
        }

        int raw_size  = prefix & 0x03;
        int item_size = (raw_size == 3) ? 4 : raw_size;
        int item_type = (prefix >> 2) & 0x03;
        int item_tag  = (prefix >> 4) & 0x0F;

        if ((int)(i + item_size) > (int)len) break;
        const uint8_t *data = &desc[i];
        i += item_size;

        uint32_t uval = item_size ? bt_hid_read_unsigned(data, item_size) : 0;
        int32_t  sval = item_size ? bt_hid_read_signed(data, item_size)   : 0;

        switch (item_type) {

        // ── Global items (state persists) ────────────────────────────────
        case 1:
            switch (item_tag) {
            case 0: usage_page   = uval; break;
            case 1: logical_min  = sval; break;
            case 2: logical_max  = sval; break;
            case 7: report_size  = uval; break;
            case 9: report_count = uval; break;
            case 8:  // Report ID — each ID gets its own bit-offset origin
                if ((uint8_t)uval != current_rid) {
                    current_rid = (uint8_t)uval;
                    bit_offset  = 0;
                }
                break;
            }
            break;

        // ── Local items (reset after Main) ───────────────────────────────
        case 2:
            switch (item_tag) {
            case 0: if (usage_count < 32) usages[usage_count++] = uval; break;
            case 1: usage_min = uval; has_usage_range = true; break;
            case 2: usage_max = uval; has_usage_range = true; break;
            }
            break;

        // ── Main items ───────────────────────────────────────────────────
        case 0:
            if (item_tag == 8) {  // Input
                bool     is_padding  = (uval & 0x01) != 0;
                uint32_t total_bits  = report_count * report_size;

                if (!is_padding) {
                    uint32_t eff[32]; int eff_n = 0;
                    if (has_usage_range) {
                        for (uint32_t u = usage_min; u <= usage_max && eff_n < 32; u++)
                            eff[eff_n++] = u;
                    } else {
                        for (int u = 0; u < usage_count && u < 32; u++)
                            eff[eff_n++] = usages[u];
                    }

                    if (usage_page == BT_HID_UP_GENERIC_DESKTOP) {
                        for (uint32_t f = 0; f < report_count; f++) {
                            uint32_t usage = (f < (uint32_t)eff_n)
                                           ? eff[f]
                                           : (eff_n > 0 ? eff[eff_n-1] : 0);

                            if (usage == BT_HID_USAGE_X && !out->x_axis.valid) {
                                out->x_axis.bit_offset  = bit_offset + (int)(f * report_size);
                                out->x_axis.bit_count   = (int)report_size;
                                out->x_axis.logical_min = logical_min;
                                out->x_axis.logical_max = logical_max;
                                out->x_axis.report_id   = current_rid;
                                out->x_axis.valid       = true;
                                ESP_LOGI(BTJOY_TAG, "  X: off=%d bits=%d range=%d..%d rid=%d",
                                         out->x_axis.bit_offset, (int)report_size,
                                         logical_min, logical_max, current_rid);
                            } else if (usage == BT_HID_USAGE_Y && !out->y_axis.valid) {
                                out->y_axis.bit_offset  = bit_offset + (int)(f * report_size);
                                out->y_axis.bit_count   = (int)report_size;
                                out->y_axis.logical_min = logical_min;
                                out->y_axis.logical_max = logical_max;
                                out->y_axis.report_id   = current_rid;
                                out->y_axis.valid       = true;
                                ESP_LOGI(BTJOY_TAG, "  Y: off=%d bits=%d range=%d..%d rid=%d",
                                         out->y_axis.bit_offset, (int)report_size,
                                         logical_min, logical_max, current_rid);
                            }
                        }
                    } else if (usage_page == BT_HID_UP_BUTTON && !out->buttons.valid) {
                        out->buttons.bit_offset  = bit_offset;
                        out->buttons.bit_count   = (int)report_count;
                        out->buttons.logical_min = logical_min;
                        out->buttons.logical_max = logical_max;
                        out->buttons.report_id   = current_rid;
                        out->buttons.valid       = true;
                        ESP_LOGI(BTJOY_TAG, "  Btns: off=%d count=%d rid=%d",
                                 bit_offset, (int)report_count, current_rid);
                    }
                }
                bit_offset += (int)total_bits;

            } else if (item_tag == 9) {  // Output — advance offset only
                bit_offset += (int)(report_count * report_size);
            }
            // Reset local state after every Main item
            usage_count = 0; usage_min = 0; usage_max = 0;
            has_usage_range = false;
            memset(usages, 0, sizeof(usages));
            break;

        default:
            break;
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════
//  Per-connection state
// ═════════════════════════════════════════════════════════════════════════

typedef struct {
    bool      used;
    uint16_t  conn_handle;
    int       port;                 // 0 or 1

    uint16_t  hid_svc_start;
    uint16_t  hid_svc_end;
    uint16_t  report_map_handle;
    uint16_t  report_val_handles[MAX_REPORT_HANDLES];
    int       report_val_count;
    int       subscribe_idx;

    uint8_t   desc_buf[MAX_DESC_LEN];
    int       desc_len;

    bt_joy_layout_t layout;
    bool            layout_valid;

    volatile int8_t  x;
    volatile int8_t  y;
    volatile uint8_t btns;
} bt_conn_t;

static bt_conn_t s_conns[BT_MAX_JOYSTICKS];
static bt_conn_t *find_conn(uint16_t handle)
{
    for (int i = 0; i < BT_MAX_JOYSTICKS; i++)
        if (s_conns[i].used && s_conns[i].conn_handle == handle)
            return &s_conns[i];
    return NULL;
}

static bt_conn_t *alloc_conn(uint16_t handle)
{
    for (int i = 0; i < BT_MAX_JOYSTICKS; i++) {
        if (!s_conns[i].used) {
            memset(&s_conns[i], 0, sizeof(s_conns[i]));
            s_conns[i].used        = true;
            s_conns[i].conn_handle = handle;
            s_conns[i].port        = i;
            return &s_conns[i];
        }
    }
    return NULL;
}

static void free_conn(bt_conn_t *c)
{
    if (!c) return;
    c->used         = false;
    c->layout_valid = false;
    c->x = 0; c->y = 0; c->btns = 0;
}

static int connected_count(void)
{
    int n = 0;
    for (int i = 0; i < BT_MAX_JOYSTICKS; i++)
        if (s_conns[i].used) n++;
    return n;
}

// ═════════════════════════════════════════════════════════════════════════
//  Public API
// ═════════════════════════════════════════════════════════════════════════

IRAM_ATTR bool isBTJoystickAvailable(int port)
{
    if (port < 0 || port >= BT_MAX_JOYSTICKS) return false;
    return s_conns[port].used && s_conns[port].layout_valid;
}

IRAM_ATTR int getBTAnalogX(int port)
{
    if (port < 0 || port >= BT_MAX_JOYSTICKS) return 0;
    return (int)s_conns[port].x;
}

IRAM_ATTR int getBTAnalogY(int port)
{
    if (port < 0 || port >= BT_MAX_JOYSTICKS) return 0;
    return (int)s_conns[port].y;
}

uint8_t getBTButtons()
{
    miniBTButton = (s_conns[0].btns & 0x0F) | ((s_conns[1].btns & 0x0F) << 4);    
    return miniBTButton;
}

// ═════════════════════════════════════════════════════════════════════════
//  Report processing
// ═════════════════════════════════════════════════════════════════════════

IRAM_ATTR static void bt_joy_report_cb(bt_conn_t *c, const uint8_t *data, uint16_t len)
{
    if (!c->layout_valid) return;
    const bt_joy_layout_t *lay = &c->layout;

    const uint8_t *payload = data;
    int            plen    = (int)len;

    if (lay->x_axis.report_id != 0) {
        if (plen < 2) return;
        if (data[0] != lay->x_axis.report_id) return;
        payload++; plen--;
    }

    if (lay->x_axis.valid) {
        int32_t raw = extract_bits(payload,
                                      lay->x_axis.bit_offset,
                                      lay->x_axis.bit_count);
        int8_t v = scale_axis(raw,
                                 lay->x_axis.logical_min,
                                 lay->x_axis.logical_max,
                                 lay->x_axis.bit_count);
        if (v > -BT_JOY_DEAD_ZONE && v < BT_JOY_DEAD_ZONE) v = 0;
        c->x = v;
    }

    if (lay->y_axis.valid) {
        int32_t raw = extract_bits(payload,
                                      lay->y_axis.bit_offset,
                                      lay->y_axis.bit_count);
        int8_t v = scale_axis(raw,
                                 lay->y_axis.logical_min,
                                 lay->y_axis.logical_max,
                                 lay->y_axis.bit_count);
        if (v > -BT_JOY_DEAD_ZONE && v < BT_JOY_DEAD_ZONE) v = 0;
        c->y = v;
    }

    if (lay->buttons.valid) {
        int     count = lay->buttons.bit_count; if (count > 8) count = 8;
        uint8_t raw_btns = 0;
        for (int b = 0; b < count; b++)
            if (extract_bits(payload, lay->buttons.bit_offset + b, 1))
                raw_btns |= (uint8_t)(1u << b);
        c->btns = ~raw_btns;  // HID active-high → Vectrex zero-active

        if (c->port == 0)
            miniBTButton = (miniBTButton & 0xF0) | (c->btns & 0x0F);
        else
            miniBTButton = (miniBTButton & 0x0F) | ((c->btns & 0x0F) << 4);
    }
}

// ═════════════════════════════════════════════════════════════════════════
//  GATT discovery state machine
// ═════════════════════════════════════════════════════════════════════════

static void bt_joy_scan(void);
static int  subscribe_next(bt_conn_t *c);

static int cccd_write_cb(uint16_t conn_handle,
                         const struct ble_gatt_error *error,
                         struct ble_gatt_attr *attr, void *arg)
{
    bt_conn_t *c = find_conn(conn_handle);
    if (!c) return 0;
    if (error->status != 0)
        ESP_LOGW(BTJOY_TAG, "Port %d: CCCD write error %d at idx %d",
                 c->port, error->status, c->subscribe_idx);
    c->subscribe_idx++;
    subscribe_next(c);
    return 0;
}

// The CCCD lives at val_handle+1 on virtually all BLE HID peripherals.
static int subscribe_next(bt_conn_t *c)
{
    if (c->subscribe_idx >= c->report_val_count) {
        ESP_LOGI(BTJOY_TAG, "Port %d: all notifications enabled — joystick READY", c->port);
        c->layout_valid = true;
        if (connected_count() < BT_MAX_JOYSTICKS) bt_joy_scan();
        return 0;
    }

    uint16_t cccd_handle = c->report_val_handles[c->subscribe_idx] + 1;
    uint8_t  val[2]      = {0x01, 0x00};
    struct os_mbuf *om   = ble_hs_mbuf_from_flat(val, 2);
    if (!om) return BLE_ATT_ERR_INSUFFICIENT_RES;

    ESP_LOGI(BTJOY_TAG, "Port %d: enabling notifications on handle 0x%04x (CCCD 0x%04x)",
             c->port, c->report_val_handles[c->subscribe_idx], cccd_handle);

    return ble_gattc_write(c->conn_handle, cccd_handle, om, cccd_write_cb, NULL);
}

static int report_map_read_cb(uint16_t conn_handle,
                               const struct ble_gatt_error *error,
                               struct ble_gatt_attr *attr, void *arg)
{
    bt_conn_t *c = find_conn(conn_handle);
    if (!c) return 0;

    if (error->status == 0 && attr && attr->om) {
        uint16_t chunk = OS_MBUF_PKTLEN(attr->om);
        if (c->desc_len + (int)chunk <= MAX_DESC_LEN) {
            os_mbuf_copydata(attr->om, 0, chunk, c->desc_buf + c->desc_len);
            c->desc_len += (int)chunk;
        } else {
            ESP_LOGW(BTJOY_TAG, "Port %d: descriptor > %d bytes — truncating",
                     c->port, MAX_DESC_LEN);
        }
        return 0;
    }

    if (error->status == BLE_HS_EDONE || error->status == 0) {
        ESP_LOGI(BTJOY_TAG, "Port %d: report map %d bytes — parsing...",
                 c->port, c->desc_len);
        bt_parse_hid_descriptor(c->desc_buf, (size_t)c->desc_len, &c->layout);

        if (!c->layout.x_axis.valid && !c->layout.y_axis.valid) {
            ESP_LOGW(BTJOY_TAG, "Port %d: no X/Y axes found — rejecting device", c->port);
            ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            return 0;
        }
        c->subscribe_idx = 0;
        subscribe_next(c);
    } else {
        ESP_LOGE(BTJOY_TAG, "Port %d: report map read failed: %d", c->port, error->status);
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    return 0;
}

static int chr_disc_cb(uint16_t conn_handle,
                        const struct ble_gatt_error *error,
                        const struct ble_gatt_chr *chr, void *arg)
{
    bt_conn_t *c = find_conn(conn_handle);
    if (!c) return 0;

    if (error->status == BLE_HS_EDONE) {
        if (c->report_map_handle == 0) {
            ESP_LOGW(BTJOY_TAG, "Port %d: no Report Map characteristic", c->port);
            ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            return 0;
        }
        if (c->report_val_count == 0) {
            ESP_LOGW(BTJOY_TAG, "Port %d: no Report characteristics", c->port);
            ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            return 0;
        }
        ESP_LOGI(BTJOY_TAG, "Port %d: reading report map (handle 0x%04x)...",
                 c->port, c->report_map_handle);
        ble_gattc_read_long(conn_handle, c->report_map_handle, 0,
                            report_map_read_cb, NULL);
        return 0;
    }
    if (error->status != 0 || !chr) return 0;

    uint16_t uuid16 = ble_uuid_u16(&chr->uuid.u);

    if (uuid16 == UUID16_REPORT_MAP) {
        c->report_map_handle = chr->val_handle;
        ESP_LOGI(BTJOY_TAG, "Port %d: Report Map at 0x%04x", c->port, chr->val_handle);
    } else if (uuid16 == UUID16_REPORT) {
        if (c->report_val_count < MAX_REPORT_HANDLES) {
            c->report_val_handles[c->report_val_count++] = chr->val_handle;
            ESP_LOGI(BTJOY_TAG, "Port %d: Report chr at 0x%04x", c->port, chr->val_handle);
        }
    }
    return 0;
}

static int svc_disc_cb(uint16_t conn_handle,
                        const struct ble_gatt_error *error,
                        const struct ble_gatt_svc *svc, void *arg)
{
    bt_conn_t *c = find_conn(conn_handle);
    if (!c) return 0;

    if (error->status == BLE_HS_EDONE) {
        if (c->hid_svc_start == 0) {
            ESP_LOGW(BTJOY_TAG, "Port %d: no HID service — rejecting", c->port);
            ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        } else {
            ble_gattc_disc_all_chrs(conn_handle,
                                    c->hid_svc_start, c->hid_svc_end,
                                    chr_disc_cb, NULL);
        }
        return 0;
    }
    if (error->status != 0 || !svc) return 0;

    if (ble_uuid_u16(&svc->uuid.u) == UUID16_HID_SVC) {
        c->hid_svc_start = svc->start_handle;
        c->hid_svc_end   = svc->end_handle;
        ESP_LOGI(BTJOY_TAG, "Port %d: HID service handles 0x%04x-0x%04x",
                 c->port, svc->start_handle, svc->end_handle);
    }
    return 0;
}

// ═════════════════════════════════════════════════════════════════════════
//  GAP event handler
// ═════════════════════════════════════════════════════════════════════════

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_DISC: {
        struct ble_hs_adv_fields f;
        if (ble_hs_adv_parse_fields(&f,
                event->disc.data, event->disc.length_data) != 0)
            break;
        for (int i = 0; i < f.num_uuids16; i++) {
            if (ble_uuid_u16(&f.uuids16[i].u) != UUID16_HID_SVC) continue;

            const uint8_t *a = event->disc.addr.val;
            ESP_LOGI(BTJOY_TAG,
                     "BLE HID device: %02x:%02x:%02x:%02x:%02x:%02x → connecting (port %d)",
                     a[5], a[4], a[3], a[2], a[1], a[0], connected_count());

            ble_gap_disc_cancel();
            ble_gap_connect(BLE_OWN_ADDR_PUBLIC,
                            &event->disc.addr,
                            5000,
                            NULL,
                            gap_event_cb, NULL);
            break;
        }
        break;
    }

    case BLE_GAP_EVENT_CONNECT: {
        if (event->connect.status != 0) {
            ESP_LOGW(BTJOY_TAG, "Connect failed (status %d) — rescanning",
                     event->connect.status);
            vTaskDelay(pdMS_TO_TICKS(1000));
            if (connected_count() < BT_MAX_JOYSTICKS) bt_joy_scan();
            break;
        }
        uint16_t   handle = event->connect.conn_handle;
        bt_conn_t *c      = alloc_conn(handle);
        if (!c) {
            ESP_LOGW(BTJOY_TAG, "All %d slots full — rejecting extra connection",
                     BT_MAX_JOYSTICKS);
            ble_gap_terminate(handle, BLE_ERR_CONN_LIMIT);
            break;
        }
        ESP_LOGI(BTJOY_TAG, "Connected → port %d (handle 0x%04x)", c->port, handle);
        ble_gattc_disc_all_svcs(handle, svc_disc_cb, NULL);

        struct ble_gap_upd_params params = {
            .itvl_min            = 16,   // 20 ms
            .itvl_max            = 16,
            .latency             = 0,
            .supervision_timeout = 500,
        };
        ble_gap_update_params(handle, &params);

        break;
    }

    case BLE_GAP_EVENT_DISCONNECT: {
        uint16_t   handle = event->disconnect.conn.conn_handle;
        bt_conn_t *c      = find_conn(handle);
        if (c) {
            ESP_LOGI(BTJOY_TAG, "Port %d disconnected (reason %d)",
                     c->port, event->disconnect.reason);
            free_conn(c);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        bt_joy_scan();
        break;
    }

    case BLE_GAP_EVENT_NOTIFY_RX: {
        bt_conn_t *c = find_conn(event->notify_rx.conn_handle);
        if (c && c->layout_valid) {
            uint8_t  buf[64];
            uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
            if (len > sizeof(buf)) len = sizeof(buf);
            os_mbuf_copydata(event->notify_rx.om, 0, len, buf);
            bt_joy_report_cb(c, buf, len);
        }
        break;
    }

    case BLE_GAP_EVENT_DISC_COMPLETE:
        ESP_LOGI(BTJOY_TAG, "Scan complete (reason %d), %d/%d slots filled",
                 event->disc_complete.reason, connected_count(), BT_MAX_JOYSTICKS);
        break;


    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(BTJOY_TAG, "MTU negotiated: conn=0x%04x mtu=%d",
                 event->mtu.conn_handle, event->mtu.value);
        break;

    default:
        break;
    }
    return 0;
}

// ═════════════════════════════════════════════════════════════════════════
//  Scanner
// ═════════════════════════════════════════════════════════════════════════

// Aggressive params: 100% duty cycle for fast detection during active scan.
// duration_ms: how long to scan in total (BLE_HS_FOREVER = indefinite).
static void bt_joy_scan_for(uint32_t duration_ms)
{
    if (connected_count() >= BT_MAX_JOYSTICKS) return;
    if (ble_gap_disc_active()) return;  // already scanning

    struct ble_gap_disc_params dp = {
        .passive           = 0,
        .filter_duplicates = 1,
        .itvl              = 160,  // 100 ms
        .window            = 160,  // 100 ms — 100% duty cycle while scanning
    };
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, (int32_t)duration_ms, &dp,
                          gap_event_cb, NULL);
    if (rc == 0)
        ESP_LOGI(BTJOY_TAG, "Scanning for %lu ms... (%d/%d slots used)",
                 duration_ms, connected_count(), BT_MAX_JOYSTICKS);
    else if (rc != BLE_HS_EALREADY)
        ESP_LOGE(BTJOY_TAG, "ble_gap_disc error: %d", rc);
}
// Public: call this when user wants to pair a controller (e.g. button press).
void bt_joy_start_scan(void)
{
    ESP_LOGI(BTJOY_TAG, "User-initiated scan — 10 seconds");
    bt_joy_scan_for(10000);
}
// Internal alias used by connect/disconnect handlers.
static void bt_joy_scan(void) { bt_joy_scan_for(10000); }

/*
static void bt_joy_scan(void)
{
    if (connected_count() >= BT_MAX_JOYSTICKS) return;

    struct ble_gap_disc_params dp = {
        .passive          = 0,
        .filter_duplicates = 1,
        .itvl             = 160,  // 100 ms  (units: 0.625 ms)
        .window           = 80,   // 50 ms
    };
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &dp,
                          gap_event_cb, NULL);
    if (rc == 0)
        ESP_LOGI(BTJOY_TAG, "Scanning... (%d/%d slots used)",
                 connected_count(), BT_MAX_JOYSTICKS);
    else if (rc != BLE_HS_EALREADY)
        ESP_LOGE(BTJOY_TAG, "ble_gap_disc error: %d", rc);
}
*/
// ═════════════════════════════════════════════════════════════════════════
//  Initialisation
// ═════════════════════════════════════════════════════════════════════════

static void on_ble_sync(void)
{
    ble_hs_id_infer_auto(0, NULL);
    ESP_LOGI(BTJOY_TAG, "BLE ready — call bt_joy_start_scan() to pair controllers");
    // No auto-scan: user must call bt_joy_start_scan() explicitly.
}

static void nimble_host_task(void *arg)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}
/*
void bt_joy_init(void)
{
    memset(s_conns, 0, sizeof(s_conns));
    nimble_port_init();
    ble_hs_cfg.sync_cb  = on_ble_sync;
    ble_hs_cfg.reset_cb = NULL;
    ble_svc_gap_device_name_set("vectrex-p4");
    nimble_port_freertos_init(nimble_host_task);
    ESP_LOGI(BTJOY_TAG, "BLE HID host init — up to %d joysticks", BT_MAX_JOYSTICKS);
}
    */
void bt_joy_init(void)
{
    memset(s_conns, 0, sizeof(s_conns));
    nimble_port_init();
    ble_hs_cfg.sync_cb  = on_ble_sync;
    ble_hs_cfg.reset_cb = NULL;
    ble_svc_gap_device_name_set("vectrex-p4");
    // Use a low-priority task so NimBLE doesn't starve the emulator.
    xTaskCreate(nimble_host_task, "nimble_host", 4096, NULL, 2, NULL);
    ESP_LOGI(BTJOY_TAG, "BLE HID host init — up to %d joysticks", BT_MAX_JOYSTICKS);
}

