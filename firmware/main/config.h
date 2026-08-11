#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "ctrl_core/config_map.h"

#define CONFIG_VERSION 3

/* The tunables live in ctrl_core so their clamp ranges and defaults have exactly one
 * definition (config_map.c's SPEC table). config_t is that struct plus the persistence
 * tag -- deliberately NOT a second copy of the fields.
 *
 * The layout is load-bearing: config_load() discriminates NVS blob versions by size, so
 * sizeof(config_t) must stay 76 with every field at its historical offset or every
 * deployed device silently reverts to defaults. The asserts below enforce that. */
typedef struct {
    tunable_cfg_t t;
    uint32_t      cfg_version;
} config_t;

_Static_assert(sizeof(config_t) == 76, "config_t grew or shrank -- the v3 NVS blob is size-discriminated");
_Static_assert(offsetof(config_t, t) == 0, "tunables must stay at offset 0 (v1 migration memcpys a prefix)");
_Static_assert(offsetof(config_t, cfg_version) == 72, "cfg_version moved");
_Static_assert(offsetof(config_t, t.heat_threshold) == 0,  "field moved");
_Static_assert(offsetof(config_t, t.travel_time_s) == 24,  "field moved");
_Static_assert(offsetof(config_t, t.direction_swap) == 28, "field moved");
_Static_assert(offsetof(config_t, t.kp) == 32,             "field moved");
_Static_assert(offsetof(config_t, t.alarm_dwell_ms) == 48, "field moved");
_Static_assert(offsetof(config_t, t.deadtime_s) == 60,     "field moved");
_Static_assert(offsetof(config_t, t.valve_deadband_pct) == 68, "field moved");

extern config_t g_config;

void      config_load(void);          /* nvs init + load (defaults on miss) */
esp_err_t config_save(void);
void      config_factory_reset(void); /* erase ns, reload defaults */
void      config_apply_custom(uint16_t attr_id, const void *val); /* zigbee custom-cluster tunable write, persists */

/* water_running (commanded regulation enable) is persisted as its OWN small NVS key,
 * deliberately NOT a field in the cfg blob: a new blob field would change
 * sizeof(config_t) and force a CONFIG_VERSION bump + migration for what is commanded
 * state, not a tunable. Absent key -> false (park at park_pos: the safe boot state,
 * and the value the OnOff attribute is built with). */
bool      config_water_running_load(void);
void      config_water_running_save(bool on);
