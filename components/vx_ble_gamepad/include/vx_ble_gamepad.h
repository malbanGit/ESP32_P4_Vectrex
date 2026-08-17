/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * vx_ble_gamepad — BLE HOGP gamepad host for the ESP32-P4 Vectrex module.
 *
 * The P4 has no radio: all BLE traffic goes to the on-board ESP32-C6 over SDIO
 * through ESP-Hosted, with HCI multiplexed on the same link (VHCI).  The driver
 * hides all of that, plus the HID layer, and hands the application a plain
 * gamepad state.
 *
 * Scope, deliberately narrow: 2 analog axes and 4 push buttons.  The Vectrex
 * reference controller has exactly that, so no game can use more.  Hat
 * switches, second sticks and triggers are parsed but not exposed.
 *
 *
 * Typical use
 * -----------
 *
 *     static void on_event(const vx_gamepad_event_t *ev, void *arg)
 *     {
 *         if (ev->id == VX_GP_EVT_STATE) {
 *             printf("pad %d: x=%d y=%d btn=%02x\n",
 *                    ev->slot, ev->state.x, ev->state.y, ev->state.buttons);
 *         }
 *     }
 *
 *     vx_ble_gamepad_cfg_t cfg = VX_BLE_GAMEPAD_CFG_DEFAULT();
 *     cfg.event_cb = on_event;
 *     vx_ble_gamepad_init(&cfg);
 *     vx_ble_gamepad_start_pairing(30000);   // only to enrol a NEW gamepad
 *
 * An emulator that samples once per frame can ignore the callback entirely and
 * call vx_ble_gamepad_get_state() instead; both paths are always live.
 *
 *
 * Turning BLE off and on
 * ----------------------
 *
 * init() and deinit() are a full enable/disable pair, not just an allocation
 * bracket.  deinit() returns the NimBLE host, its buffer pools and all driver
 * state — tens of kilobytes of internal RAM — and disables the co-processor's
 * BT controller, so an application that needs the memory or the radio time for
 * Wi-Fi can simply cycle the two.  Bonds live in NVS, so gamepads reconnect by
 * themselves after the next init().
 *
 * The ESP-Hosted transport to the ESP32-C6 is shared with Wi-Fi and is never
 * torn down by this driver.  See vx_ble_gamepad_deinit().
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum gamepads connected at once. */
#define VX_GAMEPAD_MAX_SLOTS 2

/** Number of buttons exposed, bit 0 = button 1. */
#define VX_GAMEPAD_BUTTONS 4

/**
 * @brief Normalised gamepad state.
 *
 * Axes are int8 centred on 0.  This is exact rather than convenient: a gamepad
 * declaring an 8-bit axis over 0..255 — which the reference controller and most
 * cheap pads do — maps to -128..127 with no rounding at all, its rest value of
 * 128 landing precisely on 0.  A 16-bit pad is scaled into the same range.
 *
 * The values are already calibrated and deadzoned (see vx_ble_gamepad_cfg_t):
 * a stick at rest reads 0 even on hardware whose electrical centre is not the
 * theoretical one.
 */
typedef struct {
    int8_t  x;        /**< left/right, negative = left            */
    int8_t  y;        /**< up/down, negative = up (screen convention) */
    uint8_t buttons;  /**< bit 0 = button 1 ... bit 3 = button 4  */
} vx_gamepad_state_t;

typedef enum {
    VX_GP_EVT_SCAN_STARTED,   /**< pairing scan began                        */
    VX_GP_EVT_SCAN_DONE,      /**< pairing scan ended (timeout or success)   */
    VX_GP_EVT_CONNECTED,      /**< slot is live; state updates start         */
    VX_GP_EVT_DISCONNECTED,   /**< slot lost; auto-reconnect starts          */
    VX_GP_EVT_STATE,          /**< new state for a slot                      */
    VX_GP_EVT_BATTERY,        /**< battery level, 0-100 %; see cfg.battery_poll_s */
    VX_GP_EVT_UNSUPPORTED,    /**< a gamepad connected but has no usable X/Y or buttons */
} vx_gamepad_event_id_t;

typedef struct {
    vx_gamepad_event_id_t id;
    int                   slot;      /**< 0..VX_GAMEPAD_MAX_SLOTS-1, -1 if n/a */
    vx_gamepad_state_t    state;     /**< VX_GP_EVT_STATE                      */
    uint8_t               battery;   /**< VX_GP_EVT_BATTERY, percent           */
    int                   reason;    /**< VX_GP_EVT_DISCONNECTED, BLE reason    */
    const char           *name;      /**< VX_GP_EVT_CONNECTED, may be NULL      */

    /**
     * Identity address of the gamepad, little-endian as BLE carries it
     * (addr[5] is the most significant byte, the one printed first).
     *
     * THIS IS THE VALUE THAT IDENTIFIES A GAMEPAD ACROSS SESSIONS.  Slot numbers
     * do not: a slot is whoever connected first this time round.  Key anything
     * you keep per gamepad — player number, button remapping, a calibration of
     * your own — on this address plus addr_type.
     *
     * It is the resolved identity, not whatever appeared in the advertisement,
     * so it stays put even for a gamepad that rotates a private address; and it
     * is exactly the address its bond is filed under in NVS.
     *
     * Valid on CONNECTED, DISCONNECTED and BATTERY.
     */
    uint8_t               addr[6];
    uint8_t               addr_type;  /**< 0 = public, 1 = random static       */
} vx_gamepad_event_t;

/** @brief What the driver knows about a slot; see vx_ble_gamepad_get_info(). */
typedef struct {
    bool    connected;
    uint8_t addr[6];        /**< identity address — see vx_gamepad_event_t     */
    uint8_t addr_type;      /**< 0 = public, 1 = random static                 */
    char    name[32];       /**< advertised name, empty string if it had none  */
    uint8_t battery;        /**< percent; meaningless unless battery_valid     */
    bool    battery_valid;  /**< false until a level has actually been read    */
} vx_gamepad_info_t;

typedef void (*vx_gamepad_event_cb_t)(const vx_gamepad_event_t *ev, void *arg);

typedef struct {
    vx_gamepad_event_cb_t event_cb;    /**< may be NULL if polling only        */
    void                 *event_arg;

    /**
     * Deadzone as a fraction of full scale, in 1/128ths.  8 is a sensible
     * starting point for a potentiometer stick.  Applied after centre
     * calibration; inside the deadzone the axis reads exactly 0.
     */
    uint8_t deadzone;

    /**
     * Calibrate the rest position on connection.
     *
     * A real analog stick does not rest at the middle of its declared range —
     * the reference controller sits around 115/135 out of 0..255.  When true,
     * the driver samples the first reports (stick assumed untouched) and uses
     * that as the zero.  Turn off only if the gamepad may be held deflected at
     * connection time.
     */
    bool auto_calibrate;

    /**
     * Connection interval requested from the gamepad, in milliseconds.
     * 0 leaves the peer's own preference alone, which is usually right: a
     * battery-powered gamepad knows its own trade-off better than we do.
     */
    uint16_t conn_interval_ms;

    /**
     * The co-processor runs ESP-Hosted firmware older than v2.5.2.
     *
     * Such firmware has no FeatureControl RPC, so the calls that init and enable
     * the remote BT controller do not exist — but it also comes up with that
     * controller already enabled, so nothing is needed.  Calling them anyway
     * costs one RPC timeout each at start-up and returns an error the driver
     * would have to ignore.
     *
     * Defaults to true, matching the firmware shipped on the Vectrex module
     * (v0.0.6).  Set to false once the co-processor has been upgraded; the
     * driver then manages the controller explicitly and can also release it in
     * vx_ble_gamepad_release().
     */
    bool coproc_legacy_bt;

    /**
     * Battery level polling period, in seconds.  0 disables polling.
     *
     * Most gamepads make Battery Level (0x2A19) notifiable, and the driver
     * subscribes when they do, so a level change is reported the moment it
     * happens and polling never fires anything new.  The poll exists for the
     * ones that only allow a read — and as a safety net, since a gamepad that
     * notifies only on change tells you nothing at all if the level never moves.
     *
     * Each poll is one short GATT read per gamepad; at the default it is
     * negligible against the report traffic.
     */
    uint16_t battery_poll_s;

    /**
     * Delay before scanning resumes after a gamepad becomes ready, in
     * milliseconds.  0 resumes immediately.
     *
     * Scanning is how a second gamepad gets found, so it has to come back — but
     * not instantly.  The co-processor has a single 2.4 GHz radio: an active
     * scan started the moment a link is established competes for airtime with
     * that link's own setup and first reports.  Waiting a few seconds lets the
     * new connection settle first.
     *
     * It also keeps the log readable while bringing a gamepad up, since scanning
     * produces a steady stream of advertising reports.
     */
    uint16_t scan_resume_delay_ms;
} vx_ble_gamepad_cfg_t;

#define VX_BLE_GAMEPAD_CFG_DEFAULT()   \
    (vx_ble_gamepad_cfg_t)             \
    {                                  \
        .event_cb = NULL,              \
        .event_arg = NULL,             \
        .deadzone = 8,                 \
        .auto_calibrate = true,        \
        .conn_interval_ms = 0,         \
        .coproc_legacy_bt = true,      \
        .battery_poll_s = 30,          \
        .scan_resume_delay_ms = 5000,  \
    }

/**
 * @brief Bring up ESP-Hosted, NimBLE and the gamepad host.
 *
 * Reconnection to already-bonded gamepads starts immediately: after a reset the
 * application does not need to do anything for the pads to come back.
 *
 * Safe to call again after vx_ble_gamepad_deinit(), which is the intended way to
 * turn BLE off and on: an application that needs the memory or the co-processor
 * radio time for something else can cycle init/deinit freely.
 *
 * The ESP-Hosted transport is brought up only if nobody has done it already, so
 * the order in which Wi-Fi and this driver start does not matter.
 */
esp_err_t vx_ble_gamepad_init(const vx_ble_gamepad_cfg_t *cfg);

/**
 * @brief Scan for and enrol a NEW gamepad.
 *
 * Only needed to add a gamepad the driver has never seen.  Known ones reconnect
 * on their own.
 *
 * @param timeout_ms scan duration; 0 uses a sensible default.
 */
esp_err_t vx_ble_gamepad_start_pairing(uint32_t timeout_ms);

/** @brief Stop an ongoing pairing scan. */
esp_err_t vx_ble_gamepad_stop_pairing(void);

/**
 * @brief Read the current state of a slot.
 *
 * Safe to call from any task at any rate; the state is a consistent snapshot.
 * Returns ESP_ERR_NOT_FOUND if the slot has no gamepad.
 */
esp_err_t vx_ble_gamepad_get_state(int slot, vx_gamepad_state_t *out);

/**
 * @brief Identify the gamepad sitting in a slot.
 *
 * This is how an application recognises a returning gamepad without having to
 * catch and cache VX_GP_EVT_CONNECTED: read info.addr, look it up in whatever
 * table you keep, and apply your per-gamepad settings.
 *
 * Returns ESP_ERR_NOT_FOUND if the slot is empty.
 */
esp_err_t vx_ble_gamepad_get_info(int slot, vx_gamepad_info_t *out);

/** @brief Number of gamepads currently connected. */
int vx_ble_gamepad_count(void);

/** @brief True if the slot currently holds a connected gamepad. */
bool vx_ble_gamepad_connected(int slot);

/**
 * @brief Forget a gamepad: erase its bond and stop reconnecting to it.
 *
 * Use -1 to forget every bonded gamepad.
 */
esp_err_t vx_ble_gamepad_forget(int slot);

/**
 * @brief Re-run centre calibration on a slot, sticks assumed at rest.
 */
esp_err_t vx_ble_gamepad_calibrate(int slot);

/**
 * @brief Shut BLE down and give the resources back.
 *
 * Goes as far as it can while staying reversible: disconnects every gamepad,
 * stops scanning, stops and destroys the NimBLE host task and its pools,
 * disables the co-processor's BT controller, and frees all driver state.  On
 * the P4 side that returns tens of kilobytes of internal RAM.
 *
 * vx_ble_gamepad_init() works again afterwards, and bonded gamepads reconnect,
 * because bonds live in NVS rather than in RAM.
 *
 * WHAT IT DOES NOT DO, on purpose: it never tears down the ESP-Hosted transport.
 * Wi-Fi runs over the same SDIO link to the ESP32-C6, so releasing the transport
 * would take Wi-Fi down with it — the opposite of the point.  The driver brings
 * the transport up if nobody else has, and leaves it up forever after.
 *
 * @note On a co-processor running ESP-Hosted older than v2.5.2 (the factory
 *       firmware on the Vectrex module is v0.0.6), the calls that disable the
 *       remote BT controller do not exist.  Everything on the P4 side is still
 *       released; the C6's BT controller simply stays powered.  The log says so
 *       explicitly rather than pretending success.
 */
esp_err_t vx_ble_gamepad_deinit(void);

/**
 * @brief Shut BLE down permanently and release the co-processor's BT memory.
 *
 * Same as vx_ble_gamepad_deinit(), plus esp_hosted_bt_controller_deinit(true),
 * which hands the BT controller's RAM on the ESP32-C6 back for other uses —
 * typically to give Wi-Fi more room.
 *
 * This is a ONE-WAY DOOR: the controller cannot reclaim that memory, so BLE is
 * unavailable until the next reboot and vx_ble_gamepad_init() will fail.  Use
 * vx_ble_gamepad_deinit() unless you specifically need the C6-side memory back.
 *
 * @note Requires ESP-Hosted >= v2.5.2 on the co-processor.  Returns
 *       ESP_ERR_NOT_SUPPORTED on older firmware, having still performed the
 *       reversible part.
 */
esp_err_t vx_ble_gamepad_release(void);

/** @brief True between a successful init() and a deinit()/release(). */
bool vx_ble_gamepad_is_running(void);

#ifdef __cplusplus
}
#endif
