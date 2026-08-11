#pragma once
/* Shared between zigbee.c, zigbee_attrs.c and zigbee_telem.c only. NOT public API --
 * everything callers outside main/ need is in zigbee.h.
 *
 * Deliberately small: the 19 s_attr_* custom-cluster backing variables stay static in
 * zigbee.c because the stack mutates them through registered pointers keyed by attribute
 * id, never by C-level name; and the telemetry half reads join state through the public
 * zigbee_joined() rather than reaching for s_joined. */
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_zigbee_core.h"

/* Defined in zigbee_telem.c. Arms the ZCL reporting engine for every attribute that
 * needs it; called from zigbee.c's mark_joined(). Keeping the four individual
 * configure_reporting_* functions static behind this one entry point is why they do not
 * appear here. */
void zigbee_configure_reporting_on_join(void);

/* Defined in zigbee_telem.c, spawned by zigbee.c's zb_task(). */
void telemetry_task(void *arg);

/* Defined in zigbee_telem.c. Forces the next push to report even if the computed mode is
 * unchanged, so each join gives Z2M an authoritative read; called from mark_joined().
 * This exists so s_last_pushed_running_mode can stay static in zigbee_telem.c. */
void zigbee_reset_running_mode_push(void);

/* Defined in zigbee_attrs.c, called from zigbee.c's action_handler(). */
esp_err_t zb_attr_write_cb(const esp_zb_zcl_set_attr_value_message_t *m);
