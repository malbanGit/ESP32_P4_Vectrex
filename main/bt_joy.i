#include "vx_ble_gamepad.h"

DRAM_ATTR static vx_gamepad_state_t usage_gp_state;
DRAM_ATTR static volatile bool s_connected[VX_GAMEPAD_MAX_SLOTS];
DRAM_ATTR static volatile uint8_t s_battery[VX_GAMEPAD_MAX_SLOTS];

/* Counts VX_GP_EVT_STATE per slot.  The report RATE is the number that matters
 * here: a frozen display and a healthy 50 Hz stream look identical otherwise. */
DRAM_ATTR static volatile uint32_t s_reports[VX_GAMEPAD_MAX_SLOTS];

/* Per-slot observed axis extremes for auto-ranging.
 * Initialised to ±1 on connect so the first frame never divides by zero.
 * Widens automatically as the user moves the stick toward its limits.
 * Negative side maps to -128, positive side maps to +127. */
static int8_t s_ax_min[VX_GAMEPAD_MAX_SLOTS][2]; /* [slot][0=X, 1=Y] */
static int8_t s_ax_max[VX_GAMEPAD_MAX_SLOTS][2];

void bt_startPairing(int seconds);

static void on_event(const vx_gamepad_event_t *ev, void *arg)
{
    switch (ev->id) {
    case VX_GP_EVT_CONNECTED: {
        vx_gamepad_info_t info;

        s_connected[ev->slot] = true;

        /* Reset auto-range for this slot so a newly connected gamepad
         * starts fresh rather than inheriting a previous pad's range. */
        s_ax_min[ev->slot][0] = -1;
        s_ax_min[ev->slot][1] = -1;
        s_ax_max[ev->slot][0] =  1;
        s_ax_max[ev->slot][1] =  1;

        ESP_LOGI(TAG, "slot %d connected: %s [%02x:%02x:%02x:%02x:%02x:%02x] %s",
                 ev->slot, ev->name ? ev->name : "(no name)", ev->addr[5],
                 ev->addr[4], ev->addr[3], ev->addr[2], ev->addr[1], ev->addr[0],
                 ev->addr_type ? "random-static" : "public");

        /* The same gamepad read back through the polling API instead of the
         * event.  This is the call an integrator makes to recognise a returning
         * pad and restore its own per-gamepad state, so the bench shows that
         * both paths report the same identity. */
        if (vx_ble_gamepad_get_info(ev->slot, &info) == ESP_OK) {
            ESP_LOGI(TAG,
                     "  get_info: %02x:%02x:%02x:%02x:%02x:%02x %s name=\"%s\" "
                     "battery=%s",
                     info.addr[5], info.addr[4], info.addr[3], info.addr[2],
                     info.addr[1], info.addr[0],
                     info.addr_type ? "random-static" : "public", info.name,
                     info.battery_valid ? "known" : "not read yet");
        } else {
            ESP_LOGW(TAG, "  get_info failed on slot %d", ev->slot);
        }
        break;
    }

    case VX_GP_EVT_DISCONNECTED:
        s_connected[ev->slot] = false;
        ESP_LOGI(TAG, "slot %d disconnected (reason %d)", ev->slot, ev->reason);
        break;

    case VX_GP_EVT_BATTERY:
        s_battery[ev->slot] = ev->battery;
        break;

    case VX_GP_EVT_UNSUPPORTED:
        ESP_LOGW(TAG, "slot %d: gamepad has no usable X/Y or buttons", ev->slot);
        break;

    case VX_GP_EVT_SCAN_DONE:
        ESP_LOGI(TAG, "pairing window closed");
        break;

    case VX_GP_EVT_STATE:
        /* Counted, not printed: the display below polls the state, which is what
         * an emulator does -- it wants the state once per frame, not once per
         * radio packet.  The count is only there to measure the actual rate. */
        s_reports[ev->slot]++;
        break;

    default:
        break;
    }
}
/*
static void render(void)
{
    char line[160];
    size_t pos = 0;

    line[0] = '\0';
    for (int slot = 0; slot < VX_GAMEPAD_MAX_SLOTS; ++slot) {

        if (vx_ble_gamepad_get_state(slot, &usage_gp_state) != ESP_OK) {
            pos += snprintf(line + pos, sizeof(line) - pos, "  P%d --------- ",
                            slot + 1);
            continue;
        }
        pos += snprintf(line + pos, sizeof(line) - pos,
                        "  P%d X%+4d Y%+4d [%c%c%c%c]", slot + 1, usage_gp_state.x, usage_gp_state.y,
                        (usage_gp_state.buttons & 0x01) ? '1' : '.',
                        (usage_gp_state.buttons & 0x02) ? '2' : '.',
                        (usage_gp_state.buttons & 0x04) ? '3' : '.',
                        (usage_gp_state.buttons & 0x08) ? '4' : '.');
        if (s_battery[slot] && pos + 8 < sizeof(line)) {
            pos += snprintf(line + pos, sizeof(line) - pos, " %u%%",
                            s_battery[slot]);
        }
        // Reports received since the previous line: 1 s of a healthy 50 Hz
        // stream reads ~50.  This is the number to look at. 
        pos += snprintf(line + pos, sizeof(line) - pos, " %2ureps",
                        (unsigned)s_reports[slot]);
        s_reports[slot] = 0;
    }

    // One ordinary log line per second, NOT an in-place "\r" rewrite.
     // idf.py monitor prefixes and line-buffers, so a carriage-return display is
     // unreadable through it -- a frozen state and a live one look exactly the
     // same, which cost a wasted test run on 2026-08-03. 
    ESP_LOGI(TAG, "%s", line);
}
*/
void bt_init()
{
    vx_ble_gamepad_cfg_t cfg = VX_BLE_GAMEPAD_CFG_DEFAULT();

    cfg.event_cb = on_event;
    /* The factory ESP32-C6 firmware (v0.0.6) predates the FeatureControl RPC, so
     * the BT controller is already on and must not be poked.  Set to false once
     * the co-processor has been upgraded. */
    cfg.coproc_legacy_bt = true;

    memset((void *)s_connected, 0, sizeof(s_connected));
    memset((void *)s_battery,   0, sizeof(s_battery));
    memset(s_ax_min, 0, sizeof(s_ax_min));
    memset(s_ax_max, 0, sizeof(s_ax_max));

    if (vx_ble_gamepad_init(&cfg) != ESP_OK) {
        ESP_LOGE(TAG, "driver init failed");
        return;
    }
    bt_startPairing(30);
}

void bt_startPairing(int seconds)
{
    /* Known gamepads reconnect by themselves; a pairing window is only needed to
     * enrol one the driver has never seen.  Opening it unconditionally here is
     * what makes a first run work with no extra command. */
    ESP_LOGI(TAG, "pairing window open for %d seconds -- switch the gamepad on now", seconds);
    ESP_LOGI(TAG, "(already-known gamepads reconnect without it)");
    vx_ble_gamepad_start_pairing(seconds * 1000);
}

void bt_deinit(void)
{
    vx_ble_gamepad_deinit();
}

/* Read current state for slot and update the auto-range extremes. */
IRAM_ATTR int readBTData(int slot)
{

    int rc = vx_ble_gamepad_get_state(slot, &usage_gp_state);
	// render(); render output is non normalized!
    if (rc == ESP_OK && slot >= 0 && slot < VX_GAMEPAD_MAX_SLOTS) {
        if (usage_gp_state.x < s_ax_min[slot][0]) s_ax_min[slot][0] = usage_gp_state.x;
        if (usage_gp_state.x > s_ax_max[slot][0]) s_ax_max[slot][0] = usage_gp_state.x;
        if (usage_gp_state.y < s_ax_min[slot][1]) s_ax_min[slot][1] = usage_gp_state.y;
        if (usage_gp_state.y > s_ax_max[slot][1]) s_ax_max[slot][1] = usage_gp_state.y;
    }
    return rc;
}

/* Returns true while a BLE gamepad is connected on this slot. */
IRAM_ATTR bool isBTJoystickAvailable(int slot)
{
    return s_connected[slot];
}

/* Normalize a raw centered axis value to -128..+127 using the observed range.
 * Positive values scale against the positive extreme, negative against the
 * negative extreme, so asymmetric sticks fill the full output range. */
static IRAM_ATTR int normalize_axis(int v, int slot, int axis)
{
    if (slot < 0 || slot >= VX_GAMEPAD_MAX_SLOTS) return v;
    if (v > 0) {
        int mx = s_ax_max[slot][axis];
        if (mx > 0) v = v * 127 / mx;
    } else if (v < 0) {
        int mn = s_ax_min[slot][axis];
        if (mn < 0) v = v * 128 / (-mn);
    }
    if (v >  127) v =  127;
    if (v < -128) v = -128;
    return v;
}

/* Last X axis reading: -128 (full left) .. 0 (center) .. +127 (full right) */
IRAM_ATTR int getBTAnalogX(int slot)
{
    return normalize_axis((int)usage_gp_state.x, slot, 0);
}

/* Last Y axis reading: -128 (full up/back) .. 0 (center) .. +127 (full down/forward) */
IRAM_ATTR int getBTAnalogY(int slot)
{
    return normalize_axis((int)usage_gp_state.y, slot, 1);
}

IRAM_ATTR int getBTButtons(int slot)
{
    miniBTButton = 0xff;
    if (slot == 0)
    {
        miniBTButton = miniBTButton - (usage_gp_state.buttons & 0xf);
    }
    else if (slot == 1)
    {
        miniBTButton = miniBTButton - ((usage_gp_state.buttons & 0xf) << 4);
    }
    return (int)miniBTButton;
}
