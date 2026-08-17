# vx_ble_gamepad — BLE HOGP gamepad host for the ESP32-P4 Vectrex module

Connects BLE gamepads (HID over GATT, "HOGP") to the ESP32-P4 and hands the
application a plain, normalised gamepad state. Scanning, pairing, bonding,
reconnection, HID report-descriptor parsing and the ESP-Hosted plumbing are all
internal.

```
ESP32-P4 (host)                                  ESP32-C6 (co-processor)
┌────────────────────────────────────────┐       ┌────────────────────┐
│ your application                       │       │                    │
│        ↕ vx_ble_gamepad                │       │                    │
│  ├ scan / HOGP filter  ├ bonds in NVS  │       │                    │
│  ├ HID descriptor parser               │       │                    │
│  └ calibration + normalisation         │       │                    │
│        ↕ NimBLE host                   │ SDIO  │  BT controller     │
│        ↕ ESP-Hosted VHCI ──────────────┼───────┼→ (BLE)             │
└────────────────────────────────────────┘       └────────────────────┘
```

The P4 has no radio of its own. HCI is multiplexed over the same SDIO link
ESP-Hosted already uses for Wi-Fi, so BLE costs **no extra pin**.

---

## 1. Requirements

| | |
|---|---|
| ESP-IDF | 5.5.1 |
| Target | `esp32p4` |
| Host BT stack | **NimBLE** (Bluedroid is not supported) |
| Co-processor | ESP32-C6 running ESP-Hosted slave firmware; the factory v0.0.6 works as shipped, no reflash |
| Managed components | `espressif/esp_hosted` (already required for Wi-Fi) |

Pairing is **Just Works**: the driver declares itself `NO_IO`. A gamepad that
insists on an authenticated passkey cannot be used on hardware with no display
or keypad, and the driver says so explicitly instead of failing with a bare
error code.

---

## 2. Integration

**1. Copy the component.** Drop `vx_ble_gamepad/` into your project's
`components/`, or keep it out of tree and point at it from the root
`CMakeLists.txt`:

```cmake
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")
```

**2. Declare it** in the CMakeLists of the component that calls it:

```cmake
idf_component_register(SRCS "main.c"
                       REQUIRES vx_ble_gamepad)
```

**3. Apply the sdkconfig options** in section 3. Several of them fail *silently*
if missed, so the driver checks the critical ones at compile time.

Then:

```c
#include "vx_ble_gamepad.h"

static void on_event(const vx_gamepad_event_t *ev, void *arg)
{
    if (ev->id == VX_GP_EVT_STATE) {
        printf("pad %d: x=%d y=%d btn=%02x\n",
               ev->slot, ev->state.x, ev->state.y, ev->state.buttons);
    }
}

vx_ble_gamepad_cfg_t cfg = VX_BLE_GAMEPAD_CFG_DEFAULT();
cfg.event_cb = on_event;
vx_ble_gamepad_init(&cfg);
vx_ble_gamepad_start_pairing(30000);   /* only to enrol a NEW gamepad */
```

`nvs_flash_init()` must have been called before `vx_ble_gamepad_init()` — bonds
live in NVS.

An emulator that samples once per frame can ignore the callback entirely and
poll `vx_ble_gamepad_get_state()` instead. Both paths are always live.

### A working example

`hw_tests/quick_hw_tests` is delivered alongside this component and integrates it
exactly as described above. Its `gamepad` console command is a complete,
runnable reference:

- root `CMakeLists.txt` — `EXTRA_COMPONENT_DIRS` pointing at the component, and
  the `target_compile_definitions()` override of section 5;
- `main/CMakeLists.txt` — `REQUIRES vx_ble_gamepad`;
- `main/test_gamepad.c` — the callback, the polling loop, an automatic pairing
  window on first run, both exit paths, and the heap measurement of section 7;
- its `sdkconfig` — a working merge of the options in section 3 into a project
  that also does Wi-Fi, audio and USB host.

Start there rather than from this file if you prefer reading code.

---

## 3. Required sdkconfig options

Beyond the usual ESP-Hosted SDIO block (slot, bus width, clock, reset GPIO),
which is the same one Wi-Fi needs:

```ini
CONFIG_BT_ENABLED=y
CONFIG_BT_NIMBLE_ENABLED=y
CONFIG_BT_CONTROLLER_DISABLED=y
CONFIG_BT_NIMBLE_TRANSPORT_UART=n
CONFIG_ESP_HOSTED_ENABLE_BT_NIMBLE=y
CONFIG_ESP_HOSTED_NIMBLE_HCI_VHCI=y

CONFIG_BT_NIMBLE_ROLE_CENTRAL=y
CONFIG_BT_NIMBLE_ROLE_PERIPHERAL=y
CONFIG_BT_NIMBLE_ROLE_BROADCASTER=y
CONFIG_BT_NIMBLE_GATT_SERVER=y
CONFIG_BT_NIMBLE_EXT_ADV=n
CONFIG_BT_NIMBLE_NVS_PERSIST=y

CONFIG_BT_NIMBLE_TRANSPORT_ACL_FROM_LL_COUNT=48
CONFIG_BT_NIMBLE_MSYS_2_BLOCK_COUNT=48
```

| Option | Why |
|---|---|
| `BT_CONTROLLER_DISABLED=y` | The P4 has no radio. The controller is on the C6. |
| `ESP_HOSTED_NIMBLE_HCI_VHCI=y` | HCI travels over the existing SDIO link. No extra wiring. |
| `BT_NIMBLE_ROLE_CENTRAL=y` | A gamepad host is a central. |
| **`ROLE_PERIPHERAL`, `ROLE_BROADCASTER`, `GATT_SERVER`** | **Counter-intuitive but mandatory** — see below. |
| `BT_NIMBLE_EXT_ADV=n` | With extended advertising on, discovery results arrive as `BLE_GAP_EVENT_EXT_DISC`, which the driver does not handle: **scanning finds nothing at all**, with no error anywhere. |
| `BT_NIMBLE_NVS_PERSIST=y` | Default is `n`. Without it bonds vanish on reset and every power-up needs re-pairing. The driver emits a `#warning`. |
| `ACL_FROM_LL_COUNT`, `MSYS_2_BLOCK_COUNT` | Buffer pools sized for two gamepads at 50 Hz. Below these, a busy link runs the pools dry. |

### Why a *host* needs the peripheral role

Building NimBLE as a pure central (peripheral, broadcaster and GATT server all
off) produces a stack that connects, discovers, subscribes — and then floods the
log with `alloc_acl_from_ll failed` as soon as notifications flow, until the link
dies. The ACL buffers received from the controller are never recycled, because
the code path that returns them is compiled out with the peripheral role.

The driver never advertises and exposes no service; the roles are enabled purely
so the buffer machinery is complete. This costs a few kilobytes of code and is
not optional — it is enforced by an `#error`.

> A note on `CONFIG_FREERTOS_HZ`: dedicated BLE examples set it to 1000. That was
> tested here against the symptom above and changed nothing; the default 100 is
> fine for this driver. Raise it only if *your* application needs finer timing.

### Checking a merge

`sdkconfig` never changes an option that already exists in it —
`sdkconfig.defaults` applies only when the file is first created, and a Kconfig
`default` never overrides a value already written. After merging this list into
an existing project, **diff the resulting `sdkconfig` rather than trusting the
defaults file.**

---

## 4. Runtime configuration — `vx_ble_gamepad_cfg_t`

Filled with `VX_BLE_GAMEPAD_CFG_DEFAULT()` and passed to `vx_ble_gamepad_init()`.

| Field | Default | Meaning |
|---|---|---|
| `event_cb` | `NULL` | Event callback. May stay `NULL` if you only poll. |
| `event_arg` | `NULL` | Passed back to the callback. |
| `deadzone` | `8` | Deadzone in 1/128ths of full scale, applied after calibration. Inside it an axis reads exactly `0`. `0` disables it — a potentiometer stick will then jitter at rest. |
| `auto_calibrate` | `true` | Sample the first reports on connection and take that as the zero. A real stick does not rest at the middle of its declared range (the reference controller sits near 115/135 out of 0..255). Turn off only if the gamepad may be held deflected at connection time. |
| `conn_interval_ms` | `0` | Connection interval to request. `0` leaves the gamepad's own preference alone, which is usually right — a battery-powered device knows its trade-off better than we do. Set it (e.g. `20`) only when you need a guaranteed report rate. |
| `coproc_legacy_bt` | `true` | The co-processor runs ESP-Hosted older than v2.5.2. See section 9. |
| `battery_poll_s` | `30` | Battery level polling period in seconds; `0` disables it. The driver also subscribes to notifications when the gamepad offers them. |
| `scan_resume_delay_ms` | `5000` | Delay before scanning resumes after a gamepad becomes ready. The C6 has a single 2.4 GHz radio, and an active scan started the instant a link comes up competes with that link's own setup and first reports. |

Callbacks run on the NimBLE host task. Keep them short and do not call back into
the driver from inside one.

---

## 5. Build-time settings — `src/vx_ble_gamepad_cfg.h`

Compile-time constants rather than Kconfig options, deliberately: changing a
Kconfig symbol regenerates `build/config/sdkconfig.h`, which nearly every IDF
source includes, so a one-line tweak costs a full rebuild. These are values you
change while bringing a gamepad up.

| Macro | Default | Meaning |
|---|---|---|
| `VX_BLE_GAMEPAD_LOG_RX` | `0` | Log **every** frame received, one line each — 50 lines/s per gamepad at a 20 ms report period. Each line carries the interval since the previous frame, the raw bytes and the decoded values. This is the only way to judge link quality: a dropped frame, a duplicate or a timing gap is invisible in a sampled state and invisible in a rate counter that averages over a second. |
| `VX_BLE_GAMEPAD_LOG_ADV` | `1` | Advertising reports seen while scanning. `0` off, `1` gamepad candidates only, `2` every device in radio range. Answers the question a silent driver cannot: *is the gamepad not being connected, or not on the air at all?* Level 2 is noisy — most BLE devices rotate their random address every ~15 min. |
| `MAX_REPORTS_PER_DEV` | `8` | Report characteristics kept per gamepad. A composite pad exposes one per report ID (pad, keyboard, consumer keys, vendor). |
| `REPORT_MAP_MAX` | `512` | Reassembly buffer for the HID report descriptor, freed as soon as it is parsed. Commercial pads sit around 100–150 bytes. A larger descriptor is truncated and the parser says so. |
| `DEFAULT_PAIRING_MS` | `30000` | Pairing window used by `vx_ble_gamepad_start_pairing(0)`. |
| `CONNECT_TIMEOUT_MS` | `10000` | Give-up delay on a connection attempt. Short on purpose: the driver only ever connects to a gamepad it has just seen advertise, so a failure means it went away and rescanning is the right answer. |
| `VX_CAL_SAMPLES` | `10` | Reports averaged to find the rest position. Ten is 200 ms at 50 Hz — long enough to average the noise, short enough that the gamepad is still untouched. |

Parser capacities live in `src/vx_hid_desc.h`: `VX_HID_MAX_REPORTS` (12) and
`VX_HID_MAX_FIELDS` (96). A generous commercial gamepad stays well under both.

The two diagnostic macros can be forced from the project without editing the
component — useful to keep it pristine for redelivery:

```cmake
idf_component_get_property(lib vx_ble_gamepad COMPONENT_LIB)
target_compile_definitions(${lib} PRIVATE VX_BLE_GAMEPAD_LOG_RX=1)
```

---

## 6. API

| Function | Purpose |
|---|---|
| `vx_ble_gamepad_init(cfg)` | Bring up ESP-Hosted, NimBLE and the host. Bonded gamepads start reconnecting immediately. |
| `vx_ble_gamepad_start_pairing(ms)` | Scan for and enrol a **new** gamepad. `0` uses `DEFAULT_PAIRING_MS`. |
| `vx_ble_gamepad_stop_pairing()` | Close the window early. |
| `vx_ble_gamepad_get_state(slot, out)` | Consistent snapshot; safe from any task at any rate. `ESP_ERR_NOT_FOUND` if the slot is empty. |
| `vx_ble_gamepad_get_info(slot, out)` | Who is in the slot: identity address, name, battery. See section 8. |
| `vx_ble_gamepad_count()` | Gamepads currently connected. |
| `vx_ble_gamepad_connected(slot)` | Per-slot version of the above. |
| `vx_ble_gamepad_forget(slot)` | Erase the bond and stop reconnecting. `-1` forgets all. |
| `vx_ble_gamepad_calibrate(slot)` | Re-run centre calibration, sticks assumed at rest. |
| `vx_ble_gamepad_deinit()` | Turn BLE off, reversibly. See section 7. |
| `vx_ble_gamepad_release()` | Turn BLE off permanently and give the C6 its BT RAM back. One-way. |
| `vx_ble_gamepad_is_running()` | True between a successful `init()` and a `deinit()`/`release()`. |

### State

```c
typedef struct {
    int8_t  x;        /* left/right, negative = left                 */
    int8_t  y;        /* up/down, negative = up (screen convention)  */
    uint8_t buttons;  /* bit 0 = button 1 ... bit 3 = button 4       */
} vx_gamepad_state_t;
```

Axes are `int8` centred on 0. That is exact rather than merely convenient: a
gamepad declaring an 8-bit axis over 0..255 — the reference controller and most
cheap pads — maps to -128..127 with **no rounding at all**, its rest value of 128
landing precisely on 0. A 16-bit pad is scaled into the same range.

Values are already calibrated and deadzoned: a stick at rest reads 0 even on
hardware whose electrical centre is not the theoretical one.

### Events

| Event | Fields used |
|---|---|
| `VX_GP_EVT_SCAN_STARTED` | — |
| `VX_GP_EVT_SCAN_DONE` | — (timeout or success) |
| `VX_GP_EVT_CONNECTED` | `slot`, `name` (may be `NULL`), `addr` |
| `VX_GP_EVT_DISCONNECTED` | `slot`, `reason`, `addr` — auto-reconnect starts |
| `VX_GP_EVT_STATE` | `slot`, `state` |
| `VX_GP_EVT_BATTERY` | `slot`, `battery` (0–100 %) |
| `VX_GP_EVT_UNSUPPORTED` | `slot` — connected, but no usable X/Y or buttons |

---

## 7. Turning BLE off and on

`init()` and `deinit()` are a full enable/disable pair, not just an allocation
bracket. `deinit()` disconnects every gamepad, stops scanning, stops and destroys
the NimBLE host task and its pools, disables the co-processor's BT controller and
frees all driver state. `init()` works again afterwards and bonded gamepads
reconnect, because bonds live in NVS rather than RAM.

**What `deinit()` deliberately does not do is tear down the ESP-Hosted
transport.** Wi-Fi runs over the same SDIO link to the C6, so releasing it would
take Wi-Fi down too — the opposite of the point. The driver brings the transport
up if nobody else has, and leaves it up.

`vx_ble_gamepad_release()` additionally calls `esp_hosted_bt_controller_deinit()`,
handing the BT controller's RAM on the C6 back — typically to give Wi-Fi more
room. **This is a one-way door**: the controller cannot reclaim that memory, so
BLE stays unavailable until reboot and `init()` will fail. It needs ESP-Hosted
≥ v2.5.2 on the co-processor and returns `ESP_ERR_NOT_SUPPORTED` otherwise,
having still performed the reversible part.

### Memory

Measured on `hw_tests/quick_hw_tests` with two slots configured:

| | |
|---|---|
| Internal RAM at `init()` | **≈ 41.6 KB** |
| Returned by `deinit()` | all of it |
| Leak per init/deinit cycle | **none** (run-to-run drift 0 bytes) |

The first cycle shows a one-time cost of a few kilobytes (NVS cache populated on
first bonding, lazily created synchronisation objects); it does not repeat.

---

## 8. Non-volatile data — where the pairings live

Everything that survives a power cycle is a **BLE bond**, and bonds are written
by NimBLE, not by this driver. The driver stores nothing of its own: no
calibration, no slot assignment, no gamepad name. A slot number is only valid for
the lifetime of one `init()`.

| | |
|---|---|
| Partition | the **default NVS partition** (label `nvs` in the partition table) |
| Namespace | **`nimble_bond`** |
| Keys | `our_sec_N`, `peer_sec_N`, `cccd_sec_N`, `csfc_sec_N`, `p_dev_rec` — `N` being the record index |
| Written by | `ble_store_config` / `ble_store_nvs.c`, enabled by `CONFIG_BT_NIMBLE_NVS_PERSIST=y` |
| Capacity | `CONFIG_BT_NIMBLE_MAX_BONDS` (default 3) |

A bond holds the peer's identity address, its IRK when it uses a resolvable
private address, and the long-term key that encrypts the link. `cccd_sec_*` also
remembers that the host had subscribed to the gamepad's report notifications, so
a reconnection does not have to renegotiate them.

### Recognising a gamepad that comes back

**The key is the identity address**, and there are two ways to read it:

```c
/* on the event  */
if (ev->id == VX_GP_EVT_CONNECTED) {
    my_pad_t *pad = lookup(ev->addr, ev->addr_type);   /* your own table */
    ...
}

/* or at any time, without having to cache the event */
vx_gamepad_info_t info;
if (vx_ble_gamepad_get_info(slot, &info) == ESP_OK) {
    my_pad_t *pad = lookup(info.addr, info.addr_type);
}
```

`vx_gamepad_info_t` also carries the advertised `name`, the last `battery` level
and a `battery_valid` flag saying whether one has been read at all.

Three things worth being explicit about:

- **A slot number identifies nothing across sessions.** Slot 0 is whoever
  connected first this time round; the same gamepad lands in a different slot
  depending on power-up order. Never persist anything against a slot index.
- **The address reported is the resolved identity**, not whatever appeared in the
  advertisement. A gamepad using a resolvable private address changes its
  advertised address every few minutes, so that one would be worthless as a key;
  the identity address is stable and is the same value the bond is filed under.
  Use `addr` together with `addr_type` — the two together are what BLE considers
  a device identity.
- **You do not need to store calibration.** The driver re-calibrates the rest
  position on every connection (`cfg.auto_calibrate`), which is more accurate
  than a saved value: the electrical centre of a potentiometer stick drifts with
  temperature and wear. Identification is for *your* per-gamepad state — player
  number, colour, button remapping — not for restoring ours.

The name is convenient for a UI but is **not** an identity: several gamepads of
the same model advertise the same string, and an advertisement may carry no name
at all, in which case `name` is an empty string and `ev->name` is `NULL`.

What this means in practice:

- **`nvs_flash_init()` must run before `vx_ble_gamepad_init()`.** With no NVS,
  NimBLE keeps bonds in RAM only and every power-up needs re-pairing.
- **Erasing the NVS partition unpairs every gamepad.** So does an OTA that
  reformats it, or an `idf.py erase-flash`. Nothing else in the system is
  affected, and re-pairing is the only recovery.
- **`vx_ble_gamepad_forget(slot)` deletes one record**, `-1` all of them. That is
  the supported way to unpair, and it takes effect immediately — the driver stops
  reconnecting to that address.
- **Bonds are keyed by identity address, not by name.** A gamepad reset to
  factory state usually generates a new identity and shows up as a new device;
  the stale record stays until it is forgotten or `MAX_BONDS` recycles it. If the
  peer offers a fresh bond while an old one is held, the driver drops the old one
  and lets it re-pair, which is what a user expects.
- If your application uses a **custom partition table**, the NVS partition must
  still be present and large enough. Bonds are small (a few hundred bytes each),
  but a full NVS partition makes pairing fail at the moment the key is written —
  after the link is already up, which reads as a puzzling disconnect.

---

## 9. Co-processor firmware

`cfg.coproc_legacy_bt` selects how the C6's BT controller is handled.

**`true` (default)** — for ESP-Hosted slave firmware older than v2.5.2, which
includes the **v0.0.6 shipped on the Vectrex module**. That firmware has no
`FeatureControl` RPC, so the calls that init and enable the remote BT controller
do not exist — but it also comes up with the controller already enabled, so
nothing is needed. Calling them anyway costs one RPC timeout each at start-up.

**`false`** — for v2.5.2 and later. The driver then manages the controller
explicitly and `vx_ble_gamepad_release()` becomes available.

No reflash of the C6 is required. BLE has been verified end to end against the
factory v0.0.6, over the air.

---

## 10. Gamepad compatibility

The driver reads the gamepad's HID report descriptor (characteristic `0x2A4B`),
parses it, and locates each control by **bit offset**. It does not assume any
byte layout, which is what makes an unknown off-the-shelf pad usable.

It binds the first input report that carries an analog X/Y or a face button, and
takes from it:

- **Generic Desktop X and Y** — the analog stick;
- **Button page, usages 1 to 4** — the four face buttons.

Everything else is parsed and deliberately ignored: hat switch, second stick,
triggers, buttons 5 and up, keyboard and consumer-key reports. The Vectrex scope
is two axes and four buttons; the reference controller has exactly that, so no
game can use more.

> The **hat switch is ignored on purpose**, including on pads whose D-pad is
> reported only as a hat. A gamepad with no analog stick will therefore connect
> and report buttons but no movement.

Supported descriptor traits: descriptors with or without report IDs, signed and
unsigned axes, 8- and 16-bit axes, 1- to 4-byte item data, fields that straddle
byte boundaries, padding, arrays (ignored, as they are not per-control), `Push`
and `Pop`, multiple input reports in one descriptor.

### Diagnosing an unrecognised gamepad

1. Set `VX_BLE_GAMEPAD_LOG_ADV` to `2` and check the pad appears at all, with
   `conn=1 hid=1`.
2. On connection the driver dumps the descriptor it read and the binding it
   derived — which report, which fields, their ranges, and what it chose to
   ignore.
3. Set `VX_BLE_GAMEPAD_LOG_RX` to `1` to see every frame with its raw bytes and
   decoded values.

`hw_tests/ble_hid_device` emulates gamepads of five deliberately different
descriptor shapes on an ESP32-S3, which is how the parser is exercised without
owning the hardware.

---

## 11. Limitations and test status

**By design**

- BLE only; no Bluetooth Classic (the C6 is BLE-only).
- Two gamepads maximum (`VX_GAMEPAD_MAX_SLOTS`).
- Two axes and four buttons exposed; no hat, no second stick, no triggers.
- Input only: no HID output reports, so no rumble and no LED control.
- Just Works pairing only; a gamepad requiring a passkey is rejected with an
  explicit message.
- NimBLE only; Bluedroid is not supported.

**Verified**

- End-to-end operation through the factory C6 v0.0.6 with no reflash.
- 50 reports/s sustained, descriptor parsed generically.
- Disconnect and automatic reconnection, bonds surviving reset.
- init/deinit cycling with no leak.

**Not yet verified** — stated plainly so nobody is surprised:

- **Two gamepads at once.** The slot machinery is written and reviewed but has
  never had two real devices on it.
- **A commercial off-the-shelf gamepad.** Compatibility has been exercised
  against five synthetic descriptor shapes, not against a pad bought in a shop.
- **Battery level against a device that really implements it.** The path is
  written and works against the emulator.
- **A gamepad that advertises under a rotating private address.** Once connected,
  the identity address is resolved and reported correctly. But the *scan* filter
  that decides whether a known gamepad is worth reconnecting to compares the
  advertised address against the bond list, and an unresolved private address
  will not match — a pad using privacy might then only be picked up during an
  explicit pairing window. All devices tested so far advertise a fixed address.
  If this shows up, `CONFIG_BT_NIMBLE_HOST_BASED_PRIVACY=y` makes NimBLE resolve
  advertising addresses against the stored IRKs, which is the intended fix.
- **Wi-Fi/BLE coexistence under load.** Both run, but a reconnection after
  cycling between them was once observed to take ~80 s to obtain a DHCP lease.
  This looks like a Wi-Fi-side matter rather than a driver one, and has not been
  investigated.
