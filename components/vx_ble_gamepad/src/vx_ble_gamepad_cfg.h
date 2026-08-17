/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * vx_ble_gamepad -- internal build-time settings.
 *
 * Everything here is a compile-time constant rather than a Kconfig option, on
 * purpose: changing a Kconfig symbol regenerates build/config/sdkconfig.h, which
 * virtually every IDF source includes, so a one-line tweak costs a full rebuild.
 * These are values you change while bringing a gamepad up, sometimes several
 * times an hour.
 *
 * Runtime behaviour that an integrator is expected to tune -- deadzone,
 * calibration, connection interval, battery polling -- is NOT here: it belongs
 * to vx_ble_gamepad_cfg_t in the public header.
 */

#pragma once

/* -------------------------------------------------------------------------- */
/* Diagnostics                                                                */
/* -------------------------------------------------------------------------- */

/* Log EVERY frame received from the gamepad, one line each -- 50 lines per
 * second at a 20 ms report period.  Each line carries the interval since the
 * previous frame, the raw bytes, and the decoded values when the frame is a
 * decodable input report:
 *
 *   rx s0 + 20ms h=37 id=2 len=3 [ 00 85 1d ] x= +10 y=-105 btn=....
 *
 * This is the only way to judge the quality of the radio link.  A dropped frame,
 * a duplicate or a timing gap is invisible in a decoded state that is merely
 * sampled, and invisible too in a rate counter that averages over a second --
 * the "+NNms" is what exposes them.
 *
 * Driver-specific rather than a log level, so turning it on does not also drown
 * the console in every other component's DEBUG output, and so it needs no change
 * to sdkconfig.  It can also be forced from the project's CMakeLists.txt without
 * editing this file:
 *
 *     idf_component_get_property(lib vx_ble_gamepad COMPONENT_LIB)
 *     target_compile_definitions(${lib} PRIVATE VX_BLE_GAMEPAD_LOG_RX=1)
 *
 * SET BACK TO 0 BEFORE DELIVERY: 50 log lines per second per gamepad is a real
 * load on the console and on the CPU.
 */
#ifndef VX_BLE_GAMEPAD_LOG_RX
#define VX_BLE_GAMEPAD_LOG_RX 0
#endif

/* Log what the scan actually sees:
 *
 *   adv 11:22:33:44:55:66 rssi=-42 conn=1 hid=1 name="VX-PAD-TEST"
 *
 * where conn=1 means the advertisement was connectable, and hid=1 that it
 * carried the 0x1812 UUID or a HID appearance -- i.e. that the driver considers
 * it a candidate at all.
 *
 * One line per device the first time it is seen, and again whenever that picture
 * changes (a scan response adding the name, an advertisement adding the UUID).
 * Not one line per packet: an active scan reports every device every ~30 ms.
 *
 * This answers the one question a silent driver cannot: is the gamepad not being
 * connected, or is it not on the air at all?  Without it, a peripheral that has
 * stopped advertising and a host that has stopped scanning look identical from
 * the console -- which cost a session on 2026-08-04, when the emulator was the
 * one at fault.
 *
 *   0  off
 *   1  gamepad candidates only (hid=1)
 *   2  every device in radio range
 *
 * Level 2 is genuinely noisy and not only because of the neighbours: most BLE
 * devices rotate their random address every ~15 minutes, so the same phone comes
 * back as a new device, and the address cache is small enough (ADV_CACHE_LEN)
 * that an evicted address gets announced again next time it shows up.  Use it
 * when the gamepad does not appear at all, then drop back to 1. */
#ifndef VX_BLE_GAMEPAD_LOG_ADV
#define VX_BLE_GAMEPAD_LOG_ADV 1
#endif

/* -------------------------------------------------------------------------- */
/* Capacities                                                                 */
/* -------------------------------------------------------------------------- */

/* Report characteristics kept per gamepad.  A composite pad exposes one per
 * report ID (pad, keyboard, consumer keys, vendor); eight covers everything
 * seen so far. */
#define MAX_REPORTS_PER_DEV 8

/* Reassembly buffer for the HID report descriptor, read from characteristic
 * 0x2A4B.  Allocated on connection and freed as soon as the descriptor is
 * parsed.  Commercial gamepads sit around 100-150 bytes; our own test device is
 * 53.  A descriptor larger than this is truncated, and the parser says so. */
#define REPORT_MAP_MAX 512

/* -------------------------------------------------------------------------- */
/* Timings                                                                    */
/* -------------------------------------------------------------------------- */

/* Default pairing window when vx_ble_gamepad_start_pairing(0) is called. */
#define DEFAULT_PAIRING_MS 30000

/* Give up on a connection attempt after this long.  Short on purpose: the
 * driver only ever connects to a gamepad it has just seen advertise, so a
 * failure means it went away again and rescanning is the right answer. */
#define CONNECT_TIMEOUT_MS 10000

/* Reports averaged to find the stick's rest position at connection time.
 * Ten reports is 200 ms at 50 Hz: long enough to average out the noise of a
 * potentiometer stick, short enough that the gamepad is still untouched. */
#define VX_CAL_SAMPLES 10
