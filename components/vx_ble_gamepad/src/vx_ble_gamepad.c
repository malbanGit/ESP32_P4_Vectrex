/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * vx_ble_gamepad — BLE HOGP gamepad host.  See include/vx_ble_gamepad.h.
 *
 * Structure:
 *   - one slot per connected gamepad, indexed by connection handle
 *   - scanning runs whenever a slot is free, INCLUDING while another gamepad is
 *     already connected: that is what makes two-player work
 *   - GATT discovery is done here rather than through the peer cache of IDF's
 *     nimble_central_utils, because a component shipped to integrators must not
 *     depend on a directory under $IDF_PATH/examples.  Restricting discovery to
 *     the HID and Battery services also makes it markedly cheaper.
 */

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "esp_hosted.h"

#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_hs_adv.h"
#include "host/ble_sm.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "os/os_mbuf.h"

#include "vx_ble_gamepad.h"
#include "vx_ble_gamepad_cfg.h"
#include "vx_gamepad_map.h"
#include "vx_hid_desc.h"

#if !CONFIG_BT_NIMBLE_ENABLED
#error "vx_ble_gamepad requires CONFIG_BT_NIMBLE_ENABLED"
#endif
#if !CONFIG_BT_NIMBLE_ROLE_CENTRAL
#error "vx_ble_gamepad requires CONFIG_BT_NIMBLE_ROLE_CENTRAL"
#endif
#if !CONFIG_BT_NIMBLE_NVS_PERSIST
#warning "CONFIG_BT_NIMBLE_NVS_PERSIST is off: bonds will not survive a reset, \
gamepads will re-pair on every boot"
#endif

/* This driver is a pure BLE central, so trimming the peripheral role and the
 * GATT server out of NimBLE looks like an obvious saving.  It is not: with them
 * disabled, ACL buffers are never returned to the transport pool, and after
 * roughly forty received packets the link dies with
 *
 *     E vhci_drv: Rx: alloc_acl_from_ll failed
 *
 * repeating forever, no report ever decoded, and the connection finally dropped
 * by the GATT procedure timeout.  GATT discovery still completes beforehand,
 * which makes it look like a gamepad-specific problem rather than a
 * configuration one.  Cost us most of a day on 2026-08-03.
 *
 * The three symbols below have not been bisected individually; they are the
 * known-good set, matching hw_tests/ble_joystick.  If you have a reason to
 * narrow it down, do it with a sustained notification stream, not with GATT
 * discovery alone.
 */
#if !CONFIG_BT_NIMBLE_ROLE_PERIPHERAL || !CONFIG_BT_NIMBLE_ROLE_BROADCASTER || \
    !CONFIG_BT_NIMBLE_GATT_SERVER
#error "vx_ble_gamepad requires CONFIG_BT_NIMBLE_ROLE_PERIPHERAL, \
CONFIG_BT_NIMBLE_ROLE_BROADCASTER and CONFIG_BT_NIMBLE_GATT_SERVER to be \
enabled, even though the driver is central-only. Disabling them breaks ACL \
buffer recycling on the ESP-Hosted VHCI transport -- see the comment above."
#endif

static const char *TAG = "vx_ble_gamepad";

void ble_store_config_init(void);

/* ---- UUIDs ---------------------------------------------------------------- */

#define UUID_SVC_HID           0x1812
#define UUID_CHR_REPORT_MAP    0x2a4b
#define UUID_CHR_REPORT        0x2a4d
#define UUID_CHR_PROTOCOL_MODE 0x2a4e
#define UUID_DSC_CCCD          0x2902
#define UUID_DSC_REPORT_REF    0x2908
#define UUID_SVC_BATTERY       0x180f
#define UUID_CHR_BATTERY_LEVEL 0x2a19

#define APPEARANCE_HID_MIN 0x03c0
#define APPEARANCE_HID_MAX 0x03cf

/* Build-time settings (MAX_REPORTS_PER_DEV, REPORT_MAP_MAX, DEFAULT_PAIRING_MS,
 * CONNECT_TIMEOUT_MS, VX_BLE_GAMEPAD_LOG_RX) live in vx_ble_gamepad_cfg.h. */

/* No reconnection backoff on purpose: the driver never retries blindly, it only
 * connects when it actually sees the gamepad advertise.  A pad that is off or
 * out of range therefore costs nothing, and one that comes back is picked up on
 * its next advertisement. */

/* ---- slot ----------------------------------------------------------------- */

typedef enum {
    SLOT_FREE = 0,
    SLOT_CONNECTING,
    SLOT_DISCOVERING,
    SLOT_READY,
} slot_state_t;

typedef struct {
    uint16_t def_handle;
    uint16_t val_handle;
    uint16_t end_handle;
    uint16_t cccd_handle;
    uint16_t ref_handle;
    uint8_t  report_id;
    uint8_t  report_type; /* 1 = input, per the Report Reference descriptor */
    bool     notify;
} report_chr_t;

typedef struct {
    slot_state_t state;
    uint16_t     conn_handle;
    ble_addr_t   addr;
    char         name[32];

    uint16_t hid_start, hid_end;
    uint16_t report_map_handle;
    uint16_t protocol_mode_handle;

    report_chr_t reports[MAX_REPORTS_PER_DEV];
    uint8_t      report_count;
    uint8_t      cursor; /* walks reports[] during discovery                */

    /* Heap-allocated: ~2 KB per gamepad, released the moment it disconnects.
     * Keeping them static would cost that much permanently for a slot that is
     * empty most of the time. */
    uint8_t       *report_map;
    uint16_t       report_map_len;
    vx_hid_desc_t *desc;

    /* Battery Service.  Discovered after the gamepad is already usable, so a
     * device without one — or one whose battery discovery fails — still works. */
    uint16_t batt_svc_end;
    uint16_t batt_handle;
    uint16_t batt_cccd;
    uint8_t  batt_level;
    bool     batt_valid;

    vx_gamepad_bind_t  bind;
    vx_gamepad_state_t out;
    bool               out_valid;
} slot_t;

/* ---- module state --------------------------------------------------------- */

static vx_ble_gamepad_cfg_t s_cfg;
static slot_t               s_slots[VX_GAMEPAD_MAX_SLOTS];
static bool                 s_running;
static bool                 s_pairing;
static int64_t              s_pairing_until_us;
static bool                 s_scanning;
static bool                 s_connecting;
static int64_t              s_connect_started_us;
/* The host is only usable once the controller has reported ready.  On the very
 * first init the sync happens before anything asks to scan; on a re-init after
 * deinit it does not, and every call lands on BLE_HS_ENOTSYNCED (rc=22). */
static bool                 s_synced;
static SemaphoreHandle_t    s_host_stopped;
static esp_timer_handle_t   s_batt_timer;
static esp_timer_handle_t   s_scan_timer;
static int64_t              s_scan_hold_until_us;
static portMUX_TYPE         s_state_lock = portMUX_INITIALIZER_UNLOCKED;

static int  gap_event(struct ble_gap_event *event, void *arg);
static void scan_update(void);
static void discovery_start(slot_t *s);
static void disc_next_descriptors(slot_t *s);
static void read_next_reference(slot_t *s);
static void read_report_map(slot_t *s);
static void subscribe_next(slot_t *s);
static void slot_release(slot_t *s);
static void battery_discover(slot_t *s);
static void battery_read(slot_t *s);
static int  on_chr(uint16_t conn_handle, const struct ble_gatt_error *error,
                   const struct ble_gatt_chr *chr, void *arg);

/* ---- helpers -------------------------------------------------------------- */

static slot_t *slot_by_conn(uint16_t conn_handle)
{
    for (int i = 0; i < VX_GAMEPAD_MAX_SLOTS; ++i) {
        if (s_slots[i].state != SLOT_FREE &&
            s_slots[i].conn_handle == conn_handle) {
            return &s_slots[i];
        }
    }
    return NULL;
}

static slot_t *slot_alloc(void)
{
    for (int i = 0; i < VX_GAMEPAD_MAX_SLOTS; ++i) {
        if (s_slots[i].state == SLOT_FREE) {
            return &s_slots[i];
        }
    }
    return NULL;
}

static int slot_index(const slot_t *s) { return (int)(s - s_slots); }

/* Local, because addr_str() lives in IDF's nimble_central_utils example helper
 * and a shipped component must not depend on $IDF_PATH/examples. */
static const char *addr_text(const uint8_t v[6])
{
    static char buf[18];

    snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x", v[5], v[4], v[3],
             v[2], v[1], v[0]);
    return buf;
}

static int slots_free_count(void)
{
    int n = 0;
    for (int i = 0; i < VX_GAMEPAD_MAX_SLOTS; ++i) {
        if (s_slots[i].state == SLOT_FREE) {
            n++;
        }
    }
    return n;
}

static void emit(vx_gamepad_event_id_t id, slot_t *s)
{
    vx_gamepad_event_t ev;

    if (!s_cfg.event_cb) {
        return;
    }
    memset(&ev, 0, sizeof(ev));
    ev.id   = id;
    ev.slot = s ? slot_index(s) : -1;
    if (s) {
        memcpy(ev.addr, s->addr.val, 6);
        ev.addr_type = s->addr.type;
        ev.name  = s->name[0] ? s->name : NULL;
        ev.state = s->out;
    }
    s_cfg.event_cb(&ev, s_cfg.event_arg);
}

static void emit_battery(slot_t *s, uint8_t level)
{
    vx_gamepad_event_t ev;

    /* Report only on change: a gamepad polled every 30 s would otherwise wake
     * the application twice a minute to say nothing. */
    if (s->batt_valid && s->batt_level == level) {
        return;
    }
    s->batt_level = level;
    s->batt_valid = true;

    ESP_LOGI(TAG, "slot %d: battery %u%%", slot_index(s), level);

    if (!s_cfg.event_cb) {
        return;
    }
    memset(&ev, 0, sizeof(ev));
    ev.id      = VX_GP_EVT_BATTERY;
    ev.slot    = slot_index(s);
    ev.battery = level;
    memcpy(ev.addr, s->addr.val, 6);
    ev.addr_type = s->addr.type;
    s_cfg.event_cb(&ev, s_cfg.event_arg);
}

static void emit_disconnect(slot_t *s, int reason)
{
    vx_gamepad_event_t ev;

    if (!s_cfg.event_cb) {
        return;
    }
    memset(&ev, 0, sizeof(ev));
    ev.id     = VX_GP_EVT_DISCONNECTED;
    ev.slot   = slot_index(s);
    ev.reason = reason;
    memcpy(ev.addr, s->addr.val, 6);
    ev.addr_type = s->addr.type;
    s_cfg.event_cb(&ev, s_cfg.event_arg);
}

/* Replace the address we scanned with the peer's IDENTITY address.
 *
 * What an advertisement carries may be a resolvable private address, which a
 * privacy-enabled gamepad regenerates every few minutes: it identifies nothing
 * from one session to the next.  peer_id_addr is the resolved identity -- stable
 * for the life of the device, and the very key the bond is filed under.  That is
 * what an application needs in order to recognise "the same gamepad as last
 * time" and restore whatever it keeps per pad.
 *
 * NimBLE fills peer_id_addr with the identity as soon as it knows it and with
 * the over-the-air address until then, so calling this again after encryption
 * is what makes the value final.
 */
static void slot_refresh_identity(slot_t *s)
{
    struct ble_gap_conn_desc desc;

    if (s->conn_handle == BLE_HS_CONN_HANDLE_NONE ||
        ble_gap_conn_find(s->conn_handle, &desc) != 0) {
        return;
    }
    if (ble_addr_cmp(&s->addr, &desc.peer_id_addr) != 0) {
        /* addr_text() hands back one shared buffer, so the two addresses cannot
         * be formatted in the same call. */
        char advertised[18];

        strlcpy(advertised, addr_text(s->addr.val), sizeof(advertised));
        /* At INFO because it only fires when the two differ, i.e. when the peer
         * advertises under a private address.  That is rare, it never happens
         * with the test emulator, and it is precisely the case where an
         * application keying on the advertised address would be wrong. */
        ESP_LOGI(TAG, "slot %d: identity is %s (advertised as %s)", slot_index(s),
                 addr_text(desc.peer_id_addr.val), advertised);
        s->addr = desc.peer_id_addr;
    }
}

static bool addr_is_bonded(const ble_addr_t *addr)
{
    ble_addr_t peers[CONFIG_BT_NIMBLE_MAX_BONDS];
    int        num = 0;

    if (ble_store_util_bonded_peers(peers, &num,
                                    sizeof(peers) / sizeof(peers[0])) != 0) {
        return false;
    }
    for (int i = 0; i < num; ++i) {
        if (ble_addr_cmp(&peers[i], addr) == 0) {
            return true;
        }
    }
    return false;
}

/* ---- advertising filter --------------------------------------------------- */

/* Advertisement and scan response arrive as two SEPARATE discovery events, and a
 * gamepad routinely splits its identity between them — service UUID in one, name
 * in the other.  Requiring both in a single packet silently rejects perfectly
 * ordinary devices, so evidence is accumulated per address. */
#define ADV_CACHE_LEN 8

typedef struct {
    ble_addr_t addr;
    bool       in_use;
    bool       is_hid;
    bool       connectable;
    char       name[32];
    int64_t    last_us;
#if VX_BLE_GAMEPAD_LOG_ADV
    /* What was last reported for this address, so the log shows a device once
     * and then only when something about it changes. */
    uint8_t    log_sig;
    bool       logged;
#endif
} adv_entry_t;

static adv_entry_t s_adv_cache[ADV_CACHE_LEN];

static adv_entry_t *adv_lookup(const ble_addr_t *addr)
{
    int64_t      now    = esp_timer_get_time();
    adv_entry_t *victim = &s_adv_cache[0];

    for (int i = 0; i < ADV_CACHE_LEN; ++i) {
        if (s_adv_cache[i].in_use && ble_addr_cmp(&s_adv_cache[i].addr, addr) == 0) {
            s_adv_cache[i].last_us = now;
            return &s_adv_cache[i];
        }
        if (!s_adv_cache[i].in_use) {
            victim = &s_adv_cache[i];
            break;
        }
        if (s_adv_cache[i].last_us < victim->last_us) {
            victim = &s_adv_cache[i];
        }
    }
    memset(victim, 0, sizeof(*victim));
    victim->addr    = *addr;
    victim->in_use  = true;
    victim->last_us = now;
    return victim;
}

static bool adv_is_hid(const struct ble_hs_adv_fields *f)
{
    for (int i = 0; i < f->num_uuids16; ++i) {
        if (ble_uuid_u16(&f->uuids16[i].u) == UUID_SVC_HID) {
            return true;
        }
    }
    return f->appearance_is_present && f->appearance >= APPEARANCE_HID_MIN &&
           f->appearance <= APPEARANCE_HID_MAX;
}

/* ---- scanning ------------------------------------------------------------- */

static void scan_update(void)
{
    // Malban changes
    // less agressive scanning parameters!
    struct ble_gap_disc_params params = {
        /* Duplicate filtering would suppress the scan response of a device whose
         * advertisement was already reported — exactly the packet the cache
         * above needs to complete a match. */
        .filter_duplicates = 1,
        .passive           = 1,
        .itvl              = 160,  // 100ms interval
        .window            = 16,   // 10ms window — 10% duty cycle 
    };
    uint8_t own_addr_type;
    int     rc;

    if (!s_running || !s_synced || s_scanning || s_connecting) {
        return;
    }
    if (slots_free_count() == 0) {
        return;
    }

    /* Hold off if a gamepad has just come up: the co-processor has a single
     * radio, and an active scan started right then competes with the fresh
     * link's own setup and first reports.  Re-arm rather than drop, otherwise
     * scanning would only resume on the next unrelated event. */
    if (s_scan_hold_until_us > 0) {
        int64_t now = esp_timer_get_time();

        if (now < s_scan_hold_until_us && s_scan_timer) {
            esp_timer_stop(s_scan_timer);
            esp_timer_start_once(s_scan_timer,
                                 (uint64_t)(s_scan_hold_until_us - now));
            return;
        }
        s_scan_hold_until_us = 0;
    }

    if (ble_hs_id_infer_auto(0, &own_addr_type) != 0) {
        return;
    }

    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &params, gap_event, NULL);
    if (rc == 0) {
        s_scanning = true;
        ESP_LOGD(TAG, "scanning (%d slot(s) free, pairing %s)",
                 slots_free_count(), s_pairing ? "on" : "off");
    } else if (rc != BLE_HS_EALREADY) {
        ESP_LOGW(TAG, "scan start failed; rc=%d", rc);
    }
}

static void scan_stop(void)
{
    if (s_scanning) {
        ble_gap_disc_cancel();
        s_scanning = false;
    }
}

static void try_connect(const ble_addr_t *addr, const char *name)
{
    struct ble_gap_conn_params cp;
    struct ble_gap_conn_params *cpp = NULL;
    slot_t *s;
    uint8_t own_addr_type;
    int     rc;

    /* Self-heal: if a connection attempt has been outstanding well past its own
     * timeout, something swallowed the completion event.  Without this the
     * driver stays wedged forever -- s_connecting true, scan_update() returning
     * early every time -- and simply stops reconnecting. */
    if (s_connecting && s_connect_started_us &&
        esp_timer_get_time() - s_connect_started_us >
            (int64_t)CONNECT_TIMEOUT_MS * 2000) {
        ESP_LOGW(TAG, "connection attempt stuck, resetting");
        ble_gap_conn_cancel();
        for (int i = 0; i < VX_GAMEPAD_MAX_SLOTS; ++i) {
            if (s_slots[i].state == SLOT_CONNECTING) {
                slot_release(&s_slots[i]);
            }
        }
        s_connecting        = false;
        s_connect_started_us = 0;
    }

    if (s_connecting) {
        return;
    }
    s = slot_alloc();
    if (!s) {
        return;
    }
    if (ble_hs_id_infer_auto(0, &own_addr_type) != 0) {
        return;
    }

    /* Claim the slot and raise the guard BEFORE stopping the scan.
     *
     * ble_gap_disc_cancel() can deliver BLE_GAP_EVENT_DISC_COMPLETE re-entrantly
     * into gap_event(), which calls scan_update().  With the guard still down
     * and the slot still free, that restarts a scan, finds the same gamepad
     * again and fires a SECOND ble_gap_connect() at it -- after which nothing
     * ever completes and the driver never reconnects. */
    memset(s, 0, sizeof(*s));
    s->state             = SLOT_CONNECTING;
    s->addr              = *addr;
    s->conn_handle       = BLE_HS_CONN_HANDLE_NONE;
    s_connecting         = true;
    s_connect_started_us = esp_timer_get_time();
    if (name) {
        strlcpy(s->name, name, sizeof(s->name));
    }

    scan_stop();

    if (s_cfg.conn_interval_ms) {
        memset(&cp, 0, sizeof(cp));
        cp.scan_itvl           = 0x0010;
        cp.scan_window         = 0x0010;
        cp.itvl_min            = BLE_GAP_CONN_ITVL_MS(s_cfg.conn_interval_ms);
        cp.itvl_max            = BLE_GAP_CONN_ITVL_MS(s_cfg.conn_interval_ms);
        cp.latency             = 0;
        cp.supervision_timeout = BLE_GAP_INITIAL_SUPERVISION_TIMEOUT;
        cpp                    = &cp;
    }

    /* Opened one at a time on purpose: two GATT discoveries in flight on the
     * same link layer make failures much harder to attribute, and the gain
     * would be a few hundred milliseconds once per session. */
    rc = ble_gap_connect(own_addr_type, addr, CONNECT_TIMEOUT_MS, cpp, gap_event,
                         NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "connect failed to start; rc=%d", rc);
        slot_release(s);
        s_connecting         = false;
        s_connect_started_us = 0;
        scan_update();
    }
}

/* ---- GATT discovery ------------------------------------------------------- */

static int on_svc_hid(uint16_t conn_handle, const struct ble_gatt_error *error,
                      const struct ble_gatt_svc *svc, void *arg)
{
    slot_t *s = slot_by_conn(conn_handle);

    if (!s) {
        return 0;
    }
    if (error->status == 0 && svc) {
        s->hid_start = svc->start_handle;
        s->hid_end   = svc->end_handle;
        return 0;
    }
    if (error->status != BLE_HS_EDONE) {
        ESP_LOGW(TAG, "slot %d: HID service discovery failed; status=%d",
                 slot_index(s), error->status);
    }
    if (s->hid_start == 0) {
        ESP_LOGE(TAG, "slot %d: no HID service (0x1812)", slot_index(s));
        emit(VX_GP_EVT_UNSUPPORTED, s);
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return 0;
    }

    if (ble_gattc_disc_all_chrs(conn_handle, s->hid_start, s->hid_end, on_chr,
                                NULL) != 0) {
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    return 0;
}

static int on_chr(uint16_t conn_handle, const struct ble_gatt_error *error,
                  const struct ble_gatt_chr *chr, void *arg)
{
    slot_t *s = slot_by_conn(conn_handle);

    if (!s) {
        return 0;
    }

    if (error->status == 0 && chr) {
        uint16_t uuid = ble_uuid_u16(&chr->uuid.u);

        if (uuid == UUID_CHR_REPORT_MAP) {
            s->report_map_handle = chr->val_handle;
        } else if (uuid == UUID_CHR_PROTOCOL_MODE) {
            s->protocol_mode_handle = chr->val_handle;
        } else if (uuid == UUID_CHR_REPORT &&
                   s->report_count < MAX_REPORTS_PER_DEV) {
            report_chr_t *r = &s->reports[s->report_count++];

            r->def_handle = chr->def_handle;
            r->val_handle = chr->val_handle;
            r->notify     = (chr->properties & BLE_GATT_CHR_PROP_NOTIFY) != 0;
            /* Provisional: refined below once the next characteristic is known. */
            r->end_handle = s->hid_end;
        }
        /* Characteristics are reported in handle order, so the previous Report's
         * descriptor range ends just before this declaration. */
        if (s->report_count >= 2) {
            report_chr_t *prev = &s->reports[s->report_count - 2];
            if (prev->end_handle > chr->def_handle - 1) {
                prev->end_handle = chr->def_handle - 1;
            }
        }
        return 0;
    }

    if (error->status != BLE_HS_EDONE) {
        ESP_LOGW(TAG, "slot %d: characteristic discovery failed; status=%d",
                 slot_index(s), error->status);
    }

    ESP_LOGI(TAG, "slot %d: %u report characteristic(s), map=%u, protocol=%u",
             slot_index(s), s->report_count, s->report_map_handle,
             s->protocol_mode_handle);

    /* Force Report Protocol.  A device that supports Boot Protocol may come up
     * in it, in which case its reports follow the fixed boot layout instead of
     * its own descriptor and everything decoded from that descriptor is wrong. */
    if (s->protocol_mode_handle) {
        static const uint8_t report_protocol = 0x01;
        ble_gattc_write_no_rsp_flat(conn_handle, s->protocol_mode_handle,
                                    &report_protocol, 1);
    }

    s->cursor = 0;
    disc_next_descriptors(s);
    return 0;
}

static int on_dsc(uint16_t conn_handle, const struct ble_gatt_error *error,
                  uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc,
                  void *arg)
{
    slot_t *s = slot_by_conn(conn_handle);

    if (!s) {
        return 0;
    }

    if (error->status == 0 && dsc) {
        uint16_t uuid = ble_uuid_u16(&dsc->uuid.u);

        for (uint8_t i = 0; i < s->report_count; ++i) {
            if (s->reports[i].val_handle != chr_val_handle) {
                continue;
            }
            if (uuid == UUID_DSC_CCCD) {
                s->reports[i].cccd_handle = dsc->handle;
            } else if (uuid == UUID_DSC_REPORT_REF) {
                s->reports[i].ref_handle = dsc->handle;
            }
        }
        return 0;
    }

    s->cursor++;
    disc_next_descriptors(s);
    return 0;
}

static void disc_next_descriptors(slot_t *s)
{
    while (s->cursor < s->report_count) {
        report_chr_t *r = &s->reports[s->cursor];

        if (ble_gattc_disc_all_dscs(s->conn_handle, r->val_handle, r->end_handle,
                                    on_dsc, NULL) == 0) {
            return;
        }
        s->cursor++;
    }

    /* References BEFORE subscribing: a report whose ID is still unknown when the
     * first notification lands cannot be decoded, and the gamepad's first states
     * would be lost. */
    s->cursor = 0;
    read_next_reference(s);
}

static int on_reference(uint16_t conn_handle, const struct ble_gatt_error *error,
                        struct ble_gatt_attr *attr, void *arg)
{
    slot_t *s = slot_by_conn(conn_handle);
    uint8_t v[2];

    if (!s) {
        return 0;
    }
    if (error->status == 0 && attr && OS_MBUF_PKTLEN(attr->om) >= 2 &&
        os_mbuf_copydata(attr->om, 0, 2, v) == 0) {
        report_chr_t *r = &s->reports[s->cursor];

        r->report_id   = v[0];
        r->report_type = v[1];
        ESP_LOGD(TAG, "slot %d: report handle=%u id=%u type=%u", slot_index(s),
                 r->val_handle, v[0], v[1]);
    }
    s->cursor++;
    read_next_reference(s);
    return 0;
}

static void read_next_reference(slot_t *s)
{
    while (s->cursor < s->report_count) {
        report_chr_t *r = &s->reports[s->cursor];

        if (r->ref_handle &&
            ble_gattc_read(s->conn_handle, r->ref_handle, on_reference, NULL) == 0) {
            return;
        }
        s->cursor++;
    }
    read_report_map(s);
}

static void descriptor_ready(slot_t *s);

static int on_report_map(uint16_t conn_handle, const struct ble_gatt_error *error,
                         struct ble_gatt_attr *attr, void *arg)
{
    slot_t  *s = slot_by_conn(conn_handle);
    uint16_t chunk;

    if (!s) {
        return 0;
    }
    if (error->status == BLE_HS_EDONE) {
        descriptor_ready(s);
        return 0;
    }
    if (error->status != 0) {
        ESP_LOGW(TAG, "slot %d: Report Map read failed; status=%d", slot_index(s),
                 error->status);
        descriptor_ready(s);
        return 0;
    }

    chunk = attr ? OS_MBUF_PKTLEN(attr->om) : 0;
    if (s->report_map && s->report_map_len + chunk <= REPORT_MAP_MAX &&
        os_mbuf_copydata(attr->om, 0, chunk, s->report_map + s->report_map_len) == 0) {
        s->report_map_len += chunk;
    }
    return 0;
}

static void read_report_map(slot_t *s)
{
    s->report_map_len = 0;

    if (!s->report_map_handle) {
        ESP_LOGW(TAG, "slot %d: no Report Map characteristic", slot_index(s));
        descriptor_ready(s);
        return;
    }
    /* A long read, not a simple one: with the default 23-byte MTU a simple read
     * returns 22 bytes, and every real gamepad descriptor is longer than that. */
    if (ble_gattc_read_long(s->conn_handle, s->report_map_handle, 0,
                            on_report_map, NULL) != 0) {
        descriptor_ready(s);
    }
}

static void descriptor_ready(slot_t *s)
{
    if (s->report_map_len > 0 && s->desc &&
        vx_hid_desc_parse(s->report_map, s->report_map_len, s->desc)) {
        vx_hid_desc_dump(s->desc, TAG);

        if (vx_gamepad_bind_build(s->desc, s_cfg.deadzone, &s->bind)) {
            vx_gamepad_bind_dump(&s->bind, TAG);
            if (!s_cfg.auto_calibrate) {
                s->bind.calibrating = false;
            }
        } else {
            ESP_LOGW(TAG, "slot %d: no usable X/Y or buttons", slot_index(s));
            emit(VX_GP_EVT_UNSUPPORTED, s);
        }
    } else {
        ESP_LOGW(TAG, "slot %d: no usable HID descriptor (%u bytes)",
                 slot_index(s), s->report_map_len);
        emit(VX_GP_EVT_UNSUPPORTED, s);
    }

    /* The raw descriptor has done its job; the parsed form is what we keep. */
    free(s->report_map);
    s->report_map = NULL;

    s->cursor = 0;
    subscribe_next(s);
}

static int on_subscribe(uint16_t conn_handle, const struct ble_gatt_error *error,
                        struct ble_gatt_attr *attr, void *arg)
{
    slot_t *s = slot_by_conn(conn_handle);

    if (!s) {
        return 0;
    }
    if (error->status != 0) {
        ESP_LOGW(TAG, "slot %d: subscribe failed; status=%d", slot_index(s),
                 error->status);
    }
    s->cursor++;
    subscribe_next(s);
    return 0;
}

static void subscribe_next(slot_t *s)
{
    static const uint8_t notify_on[2] = {1, 0};

    while (s->cursor < s->report_count) {
        report_chr_t *r = &s->reports[s->cursor];

        if (r->notify && r->cccd_handle &&
            ble_gattc_write_flat(s->conn_handle, r->cccd_handle, notify_on,
                                 sizeof(notify_on), on_subscribe, NULL) == 0) {
            return;
        }
        s->cursor++;
    }

    s->state = SLOT_READY;
    memset(&s->out, 0, sizeof(s->out));
    s->out_valid = true;
    ESP_LOGI(TAG, "slot %d ready: %s", slot_index(s),
             s->bind.valid ? "decoding via HID descriptor" : "no decoder");
    emit(VX_GP_EVT_CONNECTED, s);

    /* One gamepad is up; if a slot is still free, go looking for the next --
     * but not immediately, see scan_resume_delay_ms. */
    s_connecting         = false;
    s_connect_started_us = 0;
    if (s_cfg.scan_resume_delay_ms && slots_free_count() > 0) {
        s_scan_hold_until_us =
            esp_timer_get_time() + (int64_t)s_cfg.scan_resume_delay_ms * 1000;
        ESP_LOGI(TAG, "scan resumes in %u ms", s_cfg.scan_resume_delay_ms);
    }
    scan_update();

    /* Battery comes last, on purpose: the gamepad is already reporting by now,
     * so a device with no Battery Service — or one whose discovery fails — costs
     * nothing but a missing percentage. */
    battery_discover(s);
}

/* ---- battery service ------------------------------------------------------ */

static int on_battery_read(uint16_t conn_handle,
                           const struct ble_gatt_error *error,
                           struct ble_gatt_attr *attr, void *arg)
{
    slot_t *s = slot_by_conn(conn_handle);
    uint8_t level;

    if (!s || error->status != 0 || !attr) {
        return 0;
    }
    if (OS_MBUF_PKTLEN(attr->om) >= 1 &&
        os_mbuf_copydata(attr->om, 0, 1, &level) == 0 && level <= 100) {
        emit_battery(s, level);
    }
    return 0;
}

static void battery_read(slot_t *s)
{
    if (s->state == SLOT_READY && s->batt_handle) {
        ble_gattc_read(s->conn_handle, s->batt_handle, on_battery_read, NULL);
    }
}

static int on_battery_dsc(uint16_t conn_handle,
                          const struct ble_gatt_error *error,
                          uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc,
                          void *arg)
{
    slot_t *s = slot_by_conn(conn_handle);

    if (!s) {
        return 0;
    }
    if (error->status == 0 && dsc &&
        ble_uuid_u16(&dsc->uuid.u) == UUID_DSC_CCCD) {
        s->batt_cccd = dsc->handle;
        return 0;
    }
    if (error->status != BLE_HS_EDONE) {
        return 0;
    }

    /* Subscribing is what makes the level arrive as soon as it changes; the
     * periodic read is only a fallback for devices that do not notify. */
    if (s->batt_cccd) {
        static const uint8_t notify_on[2] = {1, 0};
        ble_gattc_write_flat(conn_handle, s->batt_cccd, notify_on,
                             sizeof(notify_on), NULL, NULL);
    }
    battery_read(s);
    return 0;
}

static int on_battery_chr(uint16_t conn_handle,
                          const struct ble_gatt_error *error,
                          const struct ble_gatt_chr *chr, void *arg)
{
    slot_t *s = slot_by_conn(conn_handle);

    if (!s) {
        return 0;
    }
    if (error->status == 0 && chr) {
        if (ble_uuid_u16(&chr->uuid.u) == UUID_CHR_BATTERY_LEVEL) {
            s->batt_handle = chr->val_handle;
        }
        return 0;
    }
    if (!s->batt_handle) {
        ESP_LOGI(TAG, "slot %d: no Battery Level characteristic", slot_index(s));
        return 0;
    }
    if (ble_gattc_disc_all_dscs(conn_handle, s->batt_handle, s->batt_svc_end,
                                on_battery_dsc, NULL) != 0) {
        battery_read(s); /* no CCCD found: fall back to polling only */
    }
    return 0;
}

static int on_battery_svc(uint16_t conn_handle,
                          const struct ble_gatt_error *error,
                          const struct ble_gatt_svc *svc, void *arg)
{
    slot_t *s = slot_by_conn(conn_handle);

    if (!s) {
        return 0;
    }
    if (error->status == 0 && svc) {
        s->batt_svc_end = svc->end_handle;
        ble_gattc_disc_all_chrs(conn_handle, svc->start_handle, svc->end_handle,
                                on_battery_chr, NULL);
        return 0;
    }
    if (s->batt_svc_end == 0) {
        ESP_LOGI(TAG, "slot %d: no Battery Service", slot_index(s));
    }
    return 0;
}

static void battery_discover(slot_t *s)
{
    ble_gattc_disc_svc_by_uuid(s->conn_handle,
                               BLE_UUID16_DECLARE(UUID_SVC_BATTERY),
                               on_battery_svc, NULL);
}

static void discovery_start(slot_t *s)
{
    s->state = SLOT_DISCOVERING;

    s->report_map = malloc(REPORT_MAP_MAX);
    s->desc       = malloc(sizeof(vx_hid_desc_t));
    if (!s->report_map || !s->desc) {
        ESP_LOGE(TAG, "slot %d: out of memory", slot_index(s));
        ble_gap_terminate(s->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }

    if (ble_gattc_disc_svc_by_uuid(s->conn_handle,
                                   BLE_UUID16_DECLARE(UUID_SVC_HID), on_svc_hid,
                                   NULL) != 0) {
        ble_gap_terminate(s->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
}

/* ---- notifications -------------------------------------------------------- */

#if VX_BLE_GAMEPAD_LOG_RX
/* Interval since the previous frame, per slot.  Printed on every line because a
 * link that degrades shows up as timing jitter long before it shows up as a
 * wrong value. */
static int64_t s_last_rx_us[VX_GAMEPAD_MAX_SLOTS];

static void log_frame(const slot_t *s, uint16_t attr_handle, uint8_t report_id,
                      const uint8_t *data, uint16_t len, bool decoded,
                      const vx_gamepad_state_t *st)
{
    char    hex[3 * 24 + 4];
    size_t  p    = 0;
    int     idx  = slot_index(s);
    int64_t now  = esp_timer_get_time();
    int32_t dt   = s_last_rx_us[idx] ? (int32_t)((now - s_last_rx_us[idx]) / 1000)
                                     : 0;

    s_last_rx_us[idx] = now;

    for (uint16_t i = 0; i < len && p + 4 < sizeof(hex); ++i) {
        p += snprintf(hex + p, sizeof(hex) - p, "%02x ", data[i]);
    }
    if (len > 24) {
        snprintf(hex + p, sizeof(hex) - p, "...");
    }

    if (decoded && st) {
        ESP_LOGI(TAG, "rx s%d +%3" PRId32 "ms h=%u id=%u len=%u [ %s] "
                      "x=%+4d y=%+4d btn=%c%c%c%c",
                 idx, dt, attr_handle, report_id, len, hex, st->x, st->y,
                 (st->buttons & 0x01) ? '1' : '.',
                 (st->buttons & 0x02) ? '2' : '.',
                 (st->buttons & 0x04) ? '3' : '.',
                 (st->buttons & 0x08) ? '4' : '.');
    } else {
        ESP_LOGI(TAG, "rx s%d +%3" PRId32 "ms h=%u len=%u [ %s] (not decoded)",
                 idx, dt, attr_handle, len, hex);
    }
}
#endif /* VX_BLE_GAMEPAD_LOG_RX */

static void on_notify(struct ble_gap_event *event)
{
    slot_t  *s = slot_by_conn(event->notify_rx.conn_handle);
    uint8_t  data[64];
    uint16_t len;
    uint8_t  report_id = 0;
    bool     decoded   = false;

    if (!s || s->state != SLOT_READY) {
        return;
    }

    len = OS_MBUF_PKTLEN(event->notify_rx.om);
    if (len > sizeof(data)) {
        len = sizeof(data);
    }
    if (os_mbuf_copydata(event->notify_rx.om, 0, len, data) != 0) {
        return;
    }

    /* Battery Level notifies on the same connection as the reports. */
    if (s->batt_handle && event->notify_rx.attr_handle == s->batt_handle) {
#if VX_BLE_GAMEPAD_LOG_RX
        log_frame(s, event->notify_rx.attr_handle, 0, data, len, false, NULL);
#endif
        if (len >= 1 && data[0] <= 100) {
            emit_battery(s, data[0]);
        }
        return;
    }

    for (uint8_t i = 0; i < s->report_count; ++i) {
        if (s->reports[i].val_handle == event->notify_rx.attr_handle) {
            report_id = s->reports[i].report_id;
            break;
        }
    }

    vx_gamepad_state_t st = s->out;
    if (s->bind.valid) {
        decoded = vx_gamepad_bind_decode(&s->bind, report_id, data, len, &st);
    }

#if VX_BLE_GAMEPAD_LOG_RX
    /* Logged here, before the "unchanged" early return below: a frame that
     * repeats the previous state is still a frame, and leaving it out would hide
     * exactly the duplicates and gaps this log exists to reveal. */
    log_frame(s, event->notify_rx.attr_handle, report_id, data, len, decoded,
              &st);
#endif

    if (!decoded) {
        return;
    }

    if (memcmp(&st, &s->out, sizeof(st)) == 0) {
        return; /* nothing changed: no event, no wake-up */
    }

    /* Short critical section around the snapshot only: vx_ble_gamepad_get_state()
     * may be called from a rendering task at any moment. */
    portENTER_CRITICAL(&s_state_lock);
    s->out = st;
    portEXIT_CRITICAL(&s_state_lock);

    emit(VX_GP_EVT_STATE, s);
}

/* ---- GAP ------------------------------------------------------------------ */

static void slot_release(slot_t *s)
{
    free(s->report_map);
    free(s->desc);
    memset(s, 0, sizeof(*s));
    s->state = SLOT_FREE;
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_hs_adv_fields fields;
    slot_t                  *s;

    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        adv_entry_t *e;

        if (ble_hs_adv_parse_fields(&fields, event->disc.data,
                                    event->disc.length_data) != 0) {
            return 0;
        }
        e = adv_lookup(&event->disc.addr);
        if (event->disc.event_type == BLE_HCI_ADV_RPT_EVTYPE_ADV_IND ||
            event->disc.event_type == BLE_HCI_ADV_RPT_EVTYPE_DIR_IND) {
            e->connectable = true;
        }
        if (adv_is_hid(&fields)) {
            e->is_hid = true;
        }
        if (fields.name && fields.name_len && e->name[0] == '\0') {
            size_t n = fields.name_len < sizeof(e->name) - 1 ? fields.name_len
                                                             : sizeof(e->name) - 1;
            memcpy(e->name, fields.name, n);
            e->name[n] = '\0';
        }

#if VX_BLE_GAMEPAD_LOG_ADV
        {
            uint8_t sig = (uint8_t)((e->is_hid ? 1 : 0) | (e->connectable ? 2 : 0) |
                                    (e->name[0] ? 4 : 0));

            /* At level 1 only a device the driver would consider is worth a
             * line; everything else is the neighbourhood. */
            if ((VX_BLE_GAMEPAD_LOG_ADV >= 2 || e->is_hid) &&
                (!e->logged || sig != e->log_sig)) {
                e->logged  = true;
                e->log_sig = sig;
                ESP_LOGI(TAG, "adv %s rssi=%d conn=%d hid=%d name=\"%s\"",
                         addr_text(event->disc.addr.val), event->disc.rssi,
                         e->connectable, e->is_hid, e->name);
            }
        }
#endif

        /* Pairing is time-limited: without an expiry the driver would keep
         * adopting any gamepad that walks past, long after the user asked. */
        if (s_pairing && esp_timer_get_time() > s_pairing_until_us) {
            s_pairing = false;
            emit(VX_GP_EVT_SCAN_DONE, NULL);
        }

        if (!e->connectable || !e->is_hid) {
            return 0;
        }
        /* Known gamepads are reconnected always; unknown ones only while the
         * application has explicitly asked to enrol one.  Without that gate a
         * neighbour's gamepad would get grabbed the moment it advertises. */
        if (!s_pairing && !addr_is_bonded(&event->disc.addr)) {
            return 0;
        }
        ESP_LOGI(TAG, "gamepad found: %s rssi=%d name=\"%s\"",
                 addr_text(event->disc.addr.val), event->disc.rssi, e->name);
        try_connect(&event->disc.addr, e->name);
        return 0;
    }

    case BLE_GAP_EVENT_DISC_COMPLETE:
        s_scanning = false;
        scan_update();
        return 0;

    case BLE_GAP_EVENT_CONNECT:
        s_connecting         = false;
        s_connect_started_us = 0;
        if (event->connect.status != 0) {
            ESP_LOGW(TAG, "connection failed; status=%d", event->connect.status);
            for (int i = 0; i < VX_GAMEPAD_MAX_SLOTS; ++i) {
                if (s_slots[i].state == SLOT_CONNECTING) {
                    slot_release(&s_slots[i]);
                }
            }
            scan_update();
            return 0;
        }
        for (int i = 0; i < VX_GAMEPAD_MAX_SLOTS; ++i) {
            if (s_slots[i].state == SLOT_CONNECTING) {
                s = &s_slots[i];
                s->conn_handle = event->connect.conn_handle;
                slot_refresh_identity(s);
                ESP_LOGI(TAG, "slot %d connected", i);
                /* Raise the MTU before reading the descriptor: at 23 bytes a
                 * 150-byte descriptor takes seven round trips. */
                ble_gattc_exchange_mtu(s->conn_handle, NULL, NULL);
                /* The HID service is encrypted by specification, so security has
                 * to come first; discovery starts on ENC_CHANGE. */
                if (ble_gap_security_initiate(s->conn_handle) != 0) {
                    discovery_start(s);
                }
                break;
            }
        }
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        s = slot_by_conn(event->enc_change.conn_handle);
        if (!s) {
            return 0;
        }
        if (event->enc_change.status != 0) {
            ESP_LOGE(TAG, "slot %d: encryption failed; status=%d", slot_index(s),
                     event->enc_change.status);
            ble_gap_terminate(s->conn_handle, BLE_ERR_AUTH_FAIL);
            return 0;
        }
        /* Pairing has just settled the peer's identity, so this is where the
         * address the application will see becomes final. */
        slot_refresh_identity(s);
        if (s->state == SLOT_CONNECTING) {
            discovery_start(s);
        }
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX:
        on_notify(event);
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        s = slot_by_conn(event->disconnect.conn.conn_handle);
        if (s) {
            ESP_LOGI(TAG, "slot %d disconnected; reason=%d", slot_index(s),
                     event->disconnect.reason);
            emit_disconnect(s, event->disconnect.reason);
            slot_release(s);
        }
        s_connecting         = false;
        s_connect_started_us = 0;
        scan_update();
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        struct ble_gap_conn_desc desc;

        /* The peer wants a fresh bond while we still hold an old one — typically
         * because the gamepad was reset or paired elsewhere.  Drop ours and let
         * it re-pair, which is what the user expects to happen. */
        if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
            ble_store_util_delete_peer(&desc.peer_id_addr);
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    case BLE_GAP_EVENT_PASSKEY_ACTION:
        /* We advertise no input and no output, so the specification should have
         * settled on Just Works.  A gamepad that still asks for a passkey cannot
         * be paired with this hardware; say so plainly instead of failing with a
         * bare error code. */
        ESP_LOGE(TAG,
                 "gamepad requires authenticated pairing (action %d); "
                 "not supported on a device with no display or keyboard",
                 event->passkey.params.action);
        return ble_gap_terminate(event->passkey.conn_handle, BLE_ERR_AUTH_FAIL);

    case BLE_GAP_EVENT_MTU:
        ESP_LOGD(TAG, "MTU = %u", event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        ESP_LOGD(TAG, "connection parameters updated; status=%d",
                 event->conn_update.status);
        return 0;

    default:
        return 0;
    }
}

/* ---- NimBLE lifecycle ----------------------------------------------------- */

static void on_sync(void)
{
    ble_hs_util_ensure_addr(0);
    s_synced = true;
    ESP_LOGI(TAG, "controller ready");
    /* Picks up a pairing window opened before the controller was ready, which is
     * the normal case on a re-init: the application calls init() then
     * start_pairing() straight away, well before sync. */
    scan_update();
}

static void on_reset(int reason) { ESP_LOGE(TAG, "NimBLE reset; reason=%d", reason); }

/* Runs in the esp_timer task, not the NimBLE host task.  ble_gattc_read() is
 * safe to call from any task; the result comes back on the host task as usual. */
static void battery_timer_cb(void *arg)
{
    for (int i = 0; i < VX_GAMEPAD_MAX_SLOTS; ++i) {
        battery_read(&s_slots[i]);
    }
}

/* Fires when the post-connection scan hold expires. */
static void scan_timer_cb(void *arg)
{
    s_scan_hold_until_us = 0;
    scan_update();
}

static void host_task(void *param)
{
    nimble_port_run();

    /* Signal BEFORE nimble_port_freertos_deinit(): that call ends up in
     * esp_nimble_disable(), which does vTaskDelete() on this very task and
     * therefore never returns.  Anything after it is dead code -- which is
     * exactly why the shutdown used to time out every single time. */
    if (s_host_stopped) {
        xSemaphoreGive(s_host_stopped);
    }
    nimble_port_freertos_deinit();
}

/* ---- public API ----------------------------------------------------------- */

esp_err_t vx_ble_gamepad_init(const vx_ble_gamepad_cfg_t *cfg)
{
    esp_err_t ret;

    if (s_running) {
        return ESP_ERR_INVALID_STATE;
    }

    s_cfg = cfg ? *cfg : (vx_ble_gamepad_cfg_t)VX_BLE_GAMEPAD_CFG_DEFAULT();

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        return ret;
    }

    /* Idempotent: if Wi-Fi already brought the transport up, this only logs
     * "transport is already up".  The driver never tears it back down — see the
     * note on vx_ble_gamepad_deinit(). */
    if (esp_hosted_connect_to_slave() != 0) {
        ESP_LOGE(TAG, "cannot reach the co-processor over ESP-Hosted");
        return ESP_FAIL;
    }

    if (!s_cfg.coproc_legacy_bt) {
        if (esp_hosted_bt_controller_init() != ESP_OK ||
            esp_hosted_bt_controller_enable() != ESP_OK) {
            ESP_LOGE(TAG, "co-processor BT controller refused to start");
            return ESP_FAIL;
        }
    } else {
        ESP_LOGI(TAG, "legacy co-processor: BT controller assumed already on");
    }

    memset(s_slots, 0, sizeof(s_slots));
    memset(s_adv_cache, 0, sizeof(s_adv_cache));
#if VX_BLE_GAMEPAD_LOG_RX
    /* Otherwise the first frame of this run is timed against the last frame of
     * the PREVIOUS one, and the log opens on an interval of several minutes.
     * Harmless in itself, but a diagnostic that prints a wrong number is worse
     * than no diagnostic at all. */
    memset(s_last_rx_us, 0, sizeof(s_last_rx_us));
#endif
    s_pairing            = false;
    s_scanning           = false;
    s_connecting         = false;
    s_connect_started_us = 0;
    s_synced             = false;

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", ret);
        return ret;
    }

    ble_hs_cfg.reset_cb         = on_reset;
    ble_hs_cfg.sync_cb          = on_sync;
    ble_hs_cfg.store_status_cb  = ble_store_util_status_rr;
    ble_hs_cfg.sm_io_cap        = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding       = 1;
    ble_hs_cfg.sm_mitm          = 0;
    ble_hs_cfg.sm_sc            = 1;
    ble_hs_cfg.sm_our_key_dist  = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    ble_store_config_init();

    if (!s_host_stopped) {
        s_host_stopped = xSemaphoreCreateBinary();
    }

    s_running = true;
    ret = esp_nimble_enable(host_task);
    if (ret != ESP_OK) {
        s_running = false;
        nimble_port_deinit();
        return ret;
    }

    if (s_cfg.battery_poll_s) {
        const esp_timer_create_args_t targs = {
            .callback = battery_timer_cb,
            .name     = "vx_pad_batt",
        };
        if (esp_timer_create(&targs, &s_batt_timer) == ESP_OK) {
            esp_timer_start_periodic(s_batt_timer,
                                     (uint64_t)s_cfg.battery_poll_s * 1000000ULL);
        }
    }

    s_scan_hold_until_us = 0;
    if (s_cfg.scan_resume_delay_ms) {
        const esp_timer_create_args_t targs = {
            .callback = scan_timer_cb,
            .name     = "vx_pad_scan",
        };
        esp_timer_create(&targs, &s_scan_timer);
    }

    ESP_LOGI(TAG, "started (%d slot(s), deadzone %u)", VX_GAMEPAD_MAX_SLOTS,
             s_cfg.deadzone);
    return ESP_OK;
}

static esp_err_t shutdown_common(bool release_coproc)
{
    if (!s_running) {
        return ESP_ERR_INVALID_STATE;
    }

    s_running            = false;
    s_pairing            = false;
    s_synced             = false;
    s_connecting         = false;
    s_connect_started_us = 0;

    /* Stop the poll before tearing anything down, so no read is issued against a
     * connection that is about to disappear. */
    if (s_batt_timer) {
        esp_timer_stop(s_batt_timer);
        esp_timer_delete(s_batt_timer);
        s_batt_timer = NULL;
    }
    if (s_scan_timer) {
        esp_timer_stop(s_scan_timer);
        esp_timer_delete(s_scan_timer);
        s_scan_timer = NULL;
    }
    s_scan_hold_until_us = 0;

    scan_stop();

    /* Connections are NOT terminated here on purpose: ble_hs_stop(), reached
     * through nimble_port_stop(), does it itself and waits for each disconnect
     * to complete.  Doing it first makes its own terminate fail with
     * BLE_HS_EALREADY and logs "ble_hs_stop: failed to terminate connection;
     * rc=2", leaving the shutdown half-sequenced.
     *
     * nimble_port_stop() is fully blocking: it waits on the host-stop procedure
     * and then on the stop event being serviced.  The semaphore below is only a
     * belt-and-braces check that the host task really left nimble_port_run(). */
    if (nimble_port_stop() == 0) {
        if (s_host_stopped &&
            xSemaphoreTake(s_host_stopped, pdMS_TO_TICKS(2000)) != pdTRUE) {
            ESP_LOGW(TAG, "NimBLE host task did not stop in time");
        }
        nimble_port_deinit();
    } else {
        ESP_LOGW(TAG, "nimble_port_stop failed; state may leak");
    }

    for (int i = 0; i < VX_GAMEPAD_MAX_SLOTS; ++i) {
        slot_release(&s_slots[i]);
    }
    memset(s_adv_cache, 0, sizeof(s_adv_cache));

    if (s_cfg.coproc_legacy_bt) {
        if (release_coproc) {
            ESP_LOGW(TAG, "legacy co-processor: cannot release its BT memory "
                          "(needs ESP-Hosted >= 2.5.2 on the co-processor)");
        }
        return release_coproc ? ESP_ERR_NOT_SUPPORTED : ESP_OK;
    }

    esp_hosted_bt_controller_disable();
    esp_hosted_bt_controller_deinit(release_coproc);
    if (release_coproc) {
        ESP_LOGW(TAG, "co-processor BT memory released: BLE is unavailable "
                      "until the next reboot");
    }
    return ESP_OK;
}

esp_err_t vx_ble_gamepad_deinit(void) { return shutdown_common(false); }

esp_err_t vx_ble_gamepad_release(void) { return shutdown_common(true); }

bool vx_ble_gamepad_is_running(void) { return s_running; }

esp_err_t vx_ble_gamepad_start_pairing(uint32_t timeout_ms)
{
    if (!s_running) {
        return ESP_ERR_INVALID_STATE;
    }
    s_pairing          = true;
    s_pairing_until_us = esp_timer_get_time() +
                         (int64_t)(timeout_ms ? timeout_ms : DEFAULT_PAIRING_MS) * 1000;
    memset(s_adv_cache, 0, sizeof(s_adv_cache));
    emit(VX_GP_EVT_SCAN_STARTED, NULL);
    scan_update();
    return ESP_OK;
}

esp_err_t vx_ble_gamepad_stop_pairing(void)
{
    if (!s_running) {
        return ESP_ERR_INVALID_STATE;
    }
    s_pairing = false;
    emit(VX_GP_EVT_SCAN_DONE, NULL);
    return ESP_OK;
}

esp_err_t vx_ble_gamepad_get_state(int slot, vx_gamepad_state_t *out)
{
    if (slot < 0 || slot >= VX_GAMEPAD_MAX_SLOTS || !out) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_slots[slot].state != SLOT_READY || !s_slots[slot].out_valid) {
        return ESP_ERR_NOT_FOUND;
    }
    portENTER_CRITICAL(&s_state_lock);
    *out = s_slots[slot].out;
    portEXIT_CRITICAL(&s_state_lock);
    return ESP_OK;
}

esp_err_t vx_ble_gamepad_get_info(int slot, vx_gamepad_info_t *out)
{
    const slot_t *s;

    if (slot < 0 || slot >= VX_GAMEPAD_MAX_SLOTS || !out) {
        return ESP_ERR_INVALID_ARG;
    }
    s = &s_slots[slot];
    if (s->state != SLOT_READY) {
        return ESP_ERR_NOT_FOUND;
    }

    memset(out, 0, sizeof(*out));
    out->connected = true;
    memcpy(out->addr, s->addr.val, 6);
    out->addr_type     = s->addr.type;
    out->battery       = s->batt_level;
    out->battery_valid = s->batt_valid;
    strlcpy(out->name, s->name, sizeof(out->name));
    return ESP_OK;
}

int vx_ble_gamepad_count(void)
{
    int n = 0;
    for (int i = 0; i < VX_GAMEPAD_MAX_SLOTS; ++i) {
        if (s_slots[i].state == SLOT_READY) {
            n++;
        }
    }
    return n;
}

bool vx_ble_gamepad_connected(int slot)
{
    return slot >= 0 && slot < VX_GAMEPAD_MAX_SLOTS &&
           s_slots[slot].state == SLOT_READY;
}

esp_err_t vx_ble_gamepad_forget(int slot)
{
    if (slot >= VX_GAMEPAD_MAX_SLOTS) {
        return ESP_ERR_INVALID_ARG;
    }

    if (slot < 0) {
        ble_addr_t peers[CONFIG_BT_NIMBLE_MAX_BONDS];
        int        num = 0;

        if (ble_store_util_bonded_peers(peers, &num,
                                        sizeof(peers) / sizeof(peers[0])) == 0) {
            for (int i = 0; i < num; ++i) {
                ble_store_util_delete_peer(&peers[i]);
            }
        }
        for (int i = 0; i < VX_GAMEPAD_MAX_SLOTS; ++i) {
            if (s_slots[i].state != SLOT_FREE) {
                ble_gap_terminate(s_slots[i].conn_handle,
                                  BLE_ERR_REM_USER_CONN_TERM);
            }
        }
        return ESP_OK;
    }

    if (s_slots[slot].state == SLOT_FREE) {
        return ESP_ERR_NOT_FOUND;
    }
    ble_store_util_delete_peer(&s_slots[slot].addr);
    ble_gap_terminate(s_slots[slot].conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    return ESP_OK;
}

esp_err_t vx_ble_gamepad_calibrate(int slot)
{
    if (slot < 0 || slot >= VX_GAMEPAD_MAX_SLOTS) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_slots[slot].state != SLOT_READY || !s_slots[slot].bind.valid) {
        return ESP_ERR_NOT_FOUND;
    }
    vx_gamepad_bind_calibrate(&s_slots[slot].bind);
    return ESP_OK;
}
