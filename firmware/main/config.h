#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    float    heat_threshold;   /* 28  */
    float    cool_threshold;   /* 16  */
    float    hysteresis;       /* 2   */
    float    heat_setpoint;    /* 35  */
    float    cool_setpoint;    /* 18  */
    float    park_pos;         /* 50  */
    uint32_t travel_time_s;    /* 120 */
    bool     direction_swap;   /* false */
    float    kp;               /* 4.0 */
    float    ki;               /* 0.5 */
    float    gov_high;         /* 36  */
    float    gov_low;          /* 16  */
    uint32_t alarm_dwell_ms;   /* 300000 */
    uint32_t enter_dwell_ms;   /* 60000  */
    uint32_t leave_dwell_ms;   /* 420000 */
} config_t;

extern config_t g_config;

void      config_load(void);          /* nvs init + load (defaults on miss) */
esp_err_t config_save(void);
void      config_factory_reset(void); /* erase ns, reload defaults */
void      config_apply_custom(uint16_t attr_id, const void *val); /* zigbee custom-cluster tunable write, persists */
