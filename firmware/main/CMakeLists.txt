idf_component_register(
    SRCS "app_main.c" "console.c" "taskhb.c" "config.c" "sensors_hw.c" "valve_hw.c" "control_task.c" "zigbee.c" "zigbee_attrs.c" "zigbee_telem.c" "zigbee_diag.c" "ui.c" "ota.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_driver_usb_serial_jtag esp_timer console nvs_flash ctrl_core onewire_bus esp_system app_update
             espressif__esp-zigbee-lib espressif__esp-zboss-lib)
