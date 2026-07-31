#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define CONFIG_VERSION 2

typedef struct {
    float    heat_threshold;   /* 28  */
    float    cool_threshold;   /* 16  */
    float    hysteresis;       /* 2   */
    float    heat_setpoint;    /* 35  */
    float    cool_setpoint;    /* 18  */
    float    park_pos;         /* 50  */
    uint32_t travel_time_s;    /* 120 */
    bool     direction_swap;   /* false */
    float    kp;               /* 2.8 */
    float    ki;               /* 0.9 */
    float    gov_high;         /* 36  */
    float    gov_low;          /* 16  */
    uint32_t alarm_dwell_ms;   /* 300000 */
    uint32_t enter_dwell_ms;   /* 60000  */
    uint32_t leave_dwell_ms;   /* 420000 */
    /* NEW FIELDS GO LAST — v1 NVS blobs are migrated by prefix match in config_load(). */
    float    deadtime_s;       /* 30   */
    float    pi_deadband_k;    /* 0.25 */
    uint32_t cfg_version;
} config_t;

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
