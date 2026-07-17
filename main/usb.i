// usb_keyboard.c
//
// Vollständige USB-Keyboard-Initialisierung für ESP32-P4
// Benötigt managed component: espressif/usb_host_hid
//
// Aufruf aus deinem app_main():
//     usb_keyboard_init();
//
// Dann Tastatur anschließen, gedrückte Tasten werden auf der Konsole ausgegeben.

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

// Maximaler HID-Keycode, der interessant ist
#define KBD_MAX_KEYS 256

// ---------------------------------------------------------
// Konfiguration / Logging
// ---------------------------------------------------------
// Queue für HID-Device-Events
typedef struct {
    hid_host_device_handle_t handle;
    hid_host_driver_event_t  event;
    void                    *arg;
} kbd_hid_evt_t;

DRAM_ATTR static QueueHandle_t s_kbd_evt_queue = NULL;

// ---------------------------------------------------------
// Hilfsfunktionen für Keyboard-Decode (ScanCode -> ASCII)
// (aus dem offiziellen HID-Host-Beispiel abgeleitet)
// ---------------------------------------------------------

typedef struct {
    enum {
        KEY_STATE_PRESSED  = 0x00,
        KEY_STATE_RELEASED = 0x01,
    } state;
    uint8_t modifier;
    uint8_t key_code;
} key_event_t;

#define KEYBOARD_ENTER_MAIN_CHAR    '\r'
#define KEYBOARD_ENTER_LF_EXTEND    1


// Speichert den Zustand ALLER HID-Keys (0..255)
DRAM_ATTR static volatile bool key_state[KBD_MAX_KEYS];
// Speichert Modifier-Flags
DRAM_ATTR static volatile uint8_t key_modifiers = 0;

// Scancode -> ASCII (ohne Funktionstasten usw.)
DRAM_ATTR static const uint8_t keycode2ascii[57][2] = {
    {0, 0}, /* HID_KEY_NO_PRESS        */
    {0, 0}, /* HID_KEY_ROLLOVER        */
    {0, 0}, /* HID_KEY_POST_FAIL       */
    {0, 0}, /* HID_KEY_ERROR_UNDEFINED */
    {'a', 'A'}, /* HID_KEY_A               */
    {'b', 'B'}, /* HID_KEY_B               */
    {'c', 'C'}, /* HID_KEY_C               */
    {'d', 'D'}, /* HID_KEY_D               */
    {'e', 'E'}, /* HID_KEY_E               */
    {'f', 'F'}, /* HID_KEY_F               */
    {'g', 'G'}, /* HID_KEY_G               */
    {'h', 'H'}, /* HID_KEY_H               */
    {'i', 'I'}, /* HID_KEY_I               */
    {'j', 'J'}, /* HID_KEY_J               */
    {'k', 'K'}, /* HID_KEY_K               */
    {'l', 'L'}, /* HID_KEY_L               */
    {'m', 'M'}, /* HID_KEY_M               */
    {'n', 'N'}, /* HID_KEY_N               */
    {'o', 'O'}, /* HID_KEY_O               */
    {'p', 'P'}, /* HID_KEY_P               */
    {'q', 'Q'}, /* HID_KEY_Q               */
    {'r', 'R'}, /* HID_KEY_R               */
    {'s', 'S'}, /* HID_KEY_S               */
    {'t', 'T'}, /* HID_KEY_T               */
    {'u', 'U'}, /* HID_KEY_U               */
    {'v', 'V'}, /* HID_KEY_V               */
    {'w', 'W'}, /* HID_KEY_W               */
    {'x', 'X'}, /* HID_KEY_X               */
    {'y', 'Y'}, /* HID_KEY_Y               */
    {'z', 'Z'}, /* HID_KEY_Z               */
    {'1', '!'}, /* HID_KEY_1               */
    {'2', '@'}, /* HID_KEY_2               */
    {'3', '#'}, /* HID_KEY_3               */
    {'4', '$'}, /* HID_KEY_4               */
    {'5', '%'}, /* HID_KEY_5               */
    {'6', '^'}, /* HID_KEY_6               */
    {'7', '&'}, /* HID_KEY_7               */
    {'8', '*'}, /* HID_KEY_8               */
    {'9', '('}, /* HID_KEY_9               */
    {'0', ')'}, /* HID_KEY_0               */
    {KEYBOARD_ENTER_MAIN_CHAR, KEYBOARD_ENTER_MAIN_CHAR}, /* ENTER         */
    {0, 0},          /* ESC                       */
    {'\b', 0},       /* DEL (Backspace)          */
    {0, 0},          /* TAB                       */
    {' ', ' '},      /* SPACE                     */
    {'-', '_'},      /* MINUS                     */
    {'=', '+'},      /* EQUAL                     */
    {'[', '{'},      /* OPEN_BRACKET              */
    {']', '}'},      /* CLOSE_BRACKET             */
    {'\\', '|'},     /* BACK_SLASH                */
    {'\\', '|'},     /* SHARP / Non-US            */
    {';', ':'},      /* COLON                     */
    {'\'', '"'},     /* QUOTE                     */
    {'`', '~'},      /* TILDE                     */
    {',', '<'},      /* LESS                      */
    {'.', '>'},      /* GREATER                   */
    {'/', '?'}       /* SLASH                     */
};

IRAM_ATTR static inline bool kbd_is_shift(uint8_t modifier)
{
    return ((modifier & HID_LEFT_SHIFT)  == HID_LEFT_SHIFT) ||
           ((modifier & HID_RIGHT_SHIFT) == HID_RIGHT_SHIFT);
}

IRAM_ATTR static inline bool kbd_scancode_to_char(uint8_t modifier,
                                        uint8_t key_code,
                                        unsigned char *out_char)
{
    if (key_code < HID_KEY_A || key_code > HID_KEY_SLASH) {
        return false;
    }
    uint8_t mod = kbd_is_shift(modifier) ? 1 : 0;
    *out_char = keycode2ascii[key_code][mod];
    return (*out_char != 0);
}

IRAM_ATTR static inline void kbd_print_char(unsigned char c)
{
    if (!c) return;

    putchar(c);
#if KEYBOARD_ENTER_LF_EXTEND
    if (c == KEYBOARD_ENTER_MAIN_CHAR) {
        putchar('\n');
    }
#endif
    fflush(stdout);
}

// ---------------------------------------------------------
// Keyboard-Report-Callback (pro HID-Report vom Keyboard)
// ---------------------------------------------------------
IRAM_ATTR static void kbd_report_callback(const uint8_t *data, int length)
{
    if (length < (int)sizeof(hid_keyboard_input_report_boot_t)) {
        return;
    }

    const hid_keyboard_input_report_boot_t *kb =
        (const hid_keyboard_input_report_boot_t *)data;

    static uint8_t prev_keys[HID_KEYBOARD_KEY_MAX] = {0};

    // Modifier speichern
    key_modifiers = kb->modifier.val;

    // KEY UP Ereignisse (Tasten, die vorher da waren, jetzt aber fehlen)
    for (int i = 0; i < HID_KEYBOARD_KEY_MAX; ++i) {
        uint8_t old_key = prev_keys[i];
        if (old_key > HID_KEY_ERROR_UNDEFINED) {
            bool still_present = false;

            for (int j = 0; j < HID_KEYBOARD_KEY_MAX; ++j) {
                if (kb->key[j] == old_key) {
                    still_present = true;
                    break;
                }
            }
            if (!still_present) {
                key_state[old_key] = false;
            }
        }
    }

    // KEY DOWN Ereignisse (neue Keys)
    for (int i = 0; i < HID_KEYBOARD_KEY_MAX; ++i) {
        uint8_t new_key = kb->key[i];
        if (new_key > HID_KEY_ERROR_UNDEFINED) {
            key_state[new_key] = true;

// debugging            
unsigned char ch;
if (kbd_scancode_to_char(kb->modifier.val, new_key, &ch))  kbd_print_char(ch);

        }
    }

    // Neues Set als "vorher"
    memcpy(prev_keys, kb->key, HID_KEYBOARD_KEY_MAX);
}

// ---------------------------------------------------------
// HID-Interface-Callback (pro Interface: Keyboard / Mouse / Generic)
// Wir interessieren uns nur für KEYBOARD.
// ---------------------------------------------------------

IRAM_ATTR static void hid_interface_callback(hid_host_device_handle_t hid_dev,
                                   const hid_host_interface_event_t event,
                                   void *arg)
{
    (void)arg;
    uint8_t buf[64];
    size_t  len = 0;
    hid_host_dev_params_t params;

    esp_err_t err = hid_host_device_get_params(hid_dev, &params);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "hid_host_device_get_params failed: 0x%x", err);
        return;
    }

    switch (event) {
    case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
        if (params.proto == HID_PROTOCOL_KEYBOARD &&
            params.sub_class == HID_SUBCLASS_BOOT_INTERFACE) {

            err = hid_host_device_get_raw_input_report_data(
                      hid_dev, buf, sizeof(buf), &len);
            if (err == ESP_OK && len > 0) {
                kbd_report_callback(buf, (int)len);
            }
        }
        break;

    case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Keyboard DISCONNECTED");
        // Gerät schließen
        ESP_ERROR_CHECK(hid_host_device_close(hid_dev));
        break;

    case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
        ESP_LOGW(TAG, "Keyboard TRANSFER_ERROR");
        break;

    default:
        ESP_LOGW(TAG, "Unhandled interface event: %d", event);
        break;
    }
}

// ---------------------------------------------------------
// HID-Device-Callback (Connection / Disconnection)
// ---------------------------------------------------------

IRAM_ATTR static void hid_device_event_handler(hid_host_device_handle_t hid_dev,
                                     const hid_host_driver_event_t event,
                                     void *arg)
{
    (void)arg;

    // Alles in eine Queue schieben, damit die eigentliche Verarbeitung in einem Task läuft
    if (!s_kbd_evt_queue) {
        return;
    }

    kbd_hid_evt_t evt = {
        .handle = hid_dev,
        .event  = event,
        .arg    = NULL,
    };
    xQueueSend(s_kbd_evt_queue, &evt, 0);
}

// Wird im Keyboard-Task aufgerufen
IRAM_ATTR static void kbd_handle_device_event(const kbd_hid_evt_t *evt)
{
    hid_host_dev_params_t params;
    ESP_ERROR_CHECK(hid_host_device_get_params(evt->handle, &params));

    switch (evt->event) {
    case HID_HOST_DRIVER_EVENT_CONNECTED:
        ESP_LOGI(TAG, "HID device connected: proto=%d subclass=%d",
                 params.proto, params.sub_class);

        if (params.proto != HID_PROTOCOL_KEYBOARD) {
            ESP_LOGI(TAG, "Not a keyboard, ignoring.");
            return;
        }

        // Interface konfigurieren (Callback)
        {
            const hid_host_device_config_t dev_cfg = {
                .callback     = hid_interface_callback,
                .callback_arg = NULL,
            };
            ESP_ERROR_CHECK(hid_host_device_open(evt->handle, &dev_cfg));
        }

        // Boot-Protokoll aktivieren (für Keyboard üblich)
        if (params.sub_class == HID_SUBCLASS_BOOT_INTERFACE) {
            ESP_ERROR_CHECK(hid_class_request_set_protocol( evt->handle, HID_REPORT_PROTOCOL_BOOT));
            ESP_ERROR_CHECK(hid_class_request_set_idle( evt->handle, 0, 0));
        }

        // Reports aktivieren
        ESP_ERROR_CHECK(hid_host_device_start(evt->handle));
        ESP_LOGI(TAG, "Keyboard started");
        break;

    default:
        // Andere Device-Events hier ignoriert
        break;
    }
}

// ---------------------------------------------------------
// Keyboard-Event-Task: verarbeitet HID-Device-Events aus der Queue
// ---------------------------------------------------------

IRAM_ATTR static void usb_keyboard_task(void *arg)
{
    (void)arg;
    kbd_hid_evt_t evt;

    ESP_LOGI(TAG, "USB keyboard task running");

    while (1) {
        if (xQueueReceive(s_kbd_evt_queue, &evt, portMAX_DELAY) == pdTRUE) {
            kbd_handle_device_event(&evt);
        }
    }
}

IRAM_ATTR static void usb_host_event_task(void *arg)
{
    (void)arg;
    uint32_t event_flags;

    while (1) {
        esp_err_t err = usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (err != ESP_OK) {
            ESP_LOGE("usb_host", "usb_host_lib_handle_events failed: 0x%x", err);
        }
        // event_flags kannst du bei Bedarf auswerten (NO_CLIENTS, ALL_FREE, ...)
    }
}

IRAM_ATTR bool isKeyDown(uint8_t hid_code)
{
    // if (hid_code >= KBD_MAX_KEYS) return false;
    return key_state[hid_code];
}

IRAM_ATTR bool isAsciiDown(char c)
{
    // case-insensitive
    if (c >= 'A' && c <= 'Z') c = c + 32;

    for (uint8_t code = HID_KEY_A; code <= HID_KEY_Z; code++) {
        if (isKeyDown(code)) {
            char ch = 'a' + (code - HID_KEY_A);
            if (c == ch) return true;
        }
    }
    return false;
}

// Modifier-Abfragen
IRAM_ATTR bool isShiftDown(void)
{
    return (key_modifiers & (HID_LEFT_SHIFT | HID_RIGHT_SHIFT)) != 0;
}
IRAM_ATTR bool isCtrlDown(void)
{
    return (key_modifiers & (HID_LEFT_CONTROL | HID_RIGHT_CONTROL)) != 0;
}
IRAM_ATTR bool isAltDown(void)
{
    return (key_modifiers & (HID_LEFT_ALT | HID_RIGHT_ALT)) != 0;
}

// ---------------------------------------------------------
// Öffentliche Initialisierungsfunktion
// ---------------------------------------------------------
esp_err_t usb_keyboard_init(void)
{
    esp_err_t err;

    // Event-Queue anlegen
    s_kbd_evt_queue = xQueueCreate(10, sizeof(kbd_hid_evt_t));
    if (!s_kbd_evt_queue) {
        ESP_LOGE(TAG, "Failed to create keyboard event queue");
        return ESP_FAIL;
    }

    // 1. USB Host Library INSTALLEN (synchron!)
    const usb_host_config_t host_cfg = {
        .skip_phy_setup = false,
        .intr_flags     = ESP_INTR_FLAG_LEVEL1,
    };
    err = usb_host_install(&host_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "usb_host_install failed: 0x%x", err);
        return err;
    }
    ESP_LOGI(TAG, "USB Host installed");

    // 2. Task starten, der nur usb_host_lib_handle_events() aufruft
    if (xTaskCreatePinnedToCore(
            usb_host_event_task,
            "usb_host_evt",
            4096,
            NULL,
            5,
            NULL,
            0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create usb_host_event_task");
        return ESP_FAIL;
    }

    // 3. HID Host installieren (jetzt ist Host garantiert installiert)
    const hid_host_driver_config_t hid_cfg = {
        .create_background_task = true,
        .task_priority          = 5,
        .stack_size             = 4096,
        .core_id                = 0,
        .callback               = hid_device_event_handler,
        .callback_arg           = NULL,
    };
    err = hid_host_install(&hid_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "hid_host_install failed: 0x%x", err);
        return err;
    }
    ESP_LOGI(TAG, "HID Host installed");

    // 4. Keyboard-Task, der deine Queue verarbeitet
    if (xTaskCreatePinnedToCore(
            usb_keyboard_task,
            "usb_kbd_task",
            4096,
            NULL,
            4,
            NULL,
            0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create usb_kbd_task");
        return ESP_FAIL;
    }

    printf("keyboard task created\n");

    ESP_LOGI(TAG, "USB keyboard initialized, waiting for device...");
    return ESP_OK;
}
