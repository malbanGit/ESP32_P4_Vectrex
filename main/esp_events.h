#pragma once
#include <stdint.h>

/* -----------------------------------------------------------------------
 * ESP32 generic event system
 *
 * Decouples the hardware/environment layer from emulator code.
 * Any module can register a listener and receive events without main.c
 * knowing anything about the application running on top of it.
 * --------------------------------------------------------------------- */

/* --- Event types --- */
typedef enum {
    ESP_EVENT_NONE          = 0,
    ESP_EVENT_INIT          = 1,
    ESP_EVENT_SIZE_CHANGED  = 2,
    ESP_EVENT_KILL          = 3,
    ESP_EVENT_USER          = 100 /* first user-defined event type      */
} ESPEventType;

/* --- Event payload --- */
typedef struct {
    ESPEventType type;

    /* Optional payload — interpret based on type */
    union {
        /* ESP_EVENT_JOYSTICK */
        struct {
            int8_t  x;      /* -127..+127 */
            int8_t  y;
            uint8_t buttons; /* bitmask    */
        } size;

        /* ESP_EVENT_BUTTON */
        struct {
            uint8_t id;     /* button index        */
            uint8_t pressed; /* 1 = down, 0 = up   */
        } button;
        /* ESP_EVENT_KEY_DOWN */

        /* generic integer payload for custom events */
        uint32_t value;
    } data;
} ESPEvent;

/* --- Callback type --- */
typedef void (*ESPEventCallback)(const ESPEvent *event);

/* -----------------------------------------------------------------------
 * API
 * --------------------------------------------------------------------- */

/* Add a listener. No-op if listener is NULL or already registered.
 * The internal list grows dynamically as needed. */
void esp_add_event_listener(ESPEventCallback listener);

/* Remove a previously registered listener. No-op if not found. */
void esp_remove_event_listener(ESPEventCallback listener);

/* Deliver event to every registered listener in registration order. */
void esp_fire_event(const ESPEvent *event);

/* Remove all listeners (e.g. on game unload). */
void esp_clear_event_listeners(void);
