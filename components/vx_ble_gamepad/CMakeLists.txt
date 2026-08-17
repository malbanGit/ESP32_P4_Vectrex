# vx_ble_gamepad — BLE HOGP gamepad host for the ESP32-P4 + ESP32-C6 module.
#
# Self-contained on purpose: an integrator copies this directory into their
# components/ and needs nothing else beyond the esp_hosted / esp_wifi_remote
# managed components that the P4 already requires for Wi-Fi.
#
# Required sdkconfig options are listed in README.md; the driver checks the
# critical ones at compile time in vx_ble_gamepad.c.

idf_component_register(
    SRCS "src/vx_ble_gamepad.c"
         "src/vx_gamepad_map.c"
         "src/vx_hid_desc.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src"
    REQUIRES bt
    # espressif__esp_hosted is the managed-component name of ESP-Hosted: the P4
    # has no radio, so the BT controller lives on the co-processor and is reached
    # through it.  Named explicitly rather than relied upon, so a project that
    # forgets it fails at configure time instead of on a missing header.
    PRIV_REQUIRES nvs_flash esp_timer espressif__esp_hosted
)
