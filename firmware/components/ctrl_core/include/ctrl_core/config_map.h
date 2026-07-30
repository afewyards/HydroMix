#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "ctrl_core/types.h"

typedef enum {
    TUNABLE_HEAT_THRESHOLD=0, TUNABLE_COOL_THRESHOLD, TUNABLE_TRAVEL_TIME_S,
    TUNABLE_PARK_POS, TUNABLE_DIRECTION_SWAP, TUNABLE_KP, TUNABLE_KI,
    TUNABLE_GOV_HIGH, TUNABLE_GOV_LOW, TUNABLE_ALARM_DWELL_MS,
    TUNABLE_HEAT_SETPOINT, TUNABLE_COOL_SETPOINT,
    TUNABLE_DEADTIME_S, TUNABLE_PI_DEADBAND
} tunable_id_t;

typedef struct {
    float heat_threshold, cool_threshold, hysteresis, heat_setpoint, cool_setpoint, park_pos;
    uint32_t travel_time_s; bool direction_swap; float kp, ki, gov_high, gov_low;
    uint32_t alarm_dwell_ms, enter_dwell_ms, leave_dwell_ms;
    float deadtime_s, pi_deadband_k;
} tunable_cfg_t;

void tunable_cfg_defaults(tunable_cfg_t *c);
void tunable_apply(tunable_cfg_t *c, tunable_id_t id, const void *val);  /* clamps setpoints <=35/>=17 */
