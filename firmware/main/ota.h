#pragma once
#include "esp_err.h"

void ota_init(void);
void ota_note_good_sweep(void);
void ota_note_joined(void);

/* Zigbee OTA client, driven from zigbee.c's action handler.
 * Takes the esp_zb_zcl_ota_upgrade_value_message_t as a void* so this header
 * stays free of the Zigbee includes (control_task.c pulls it in too). */
esp_err_t ota_zcl_handle(const void *msg);
