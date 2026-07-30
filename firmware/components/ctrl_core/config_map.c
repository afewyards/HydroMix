#include "ctrl_core/config_map.h"

void tunable_cfg_defaults(tunable_cfg_t *c){
    c->heat_threshold=28; c->cool_threshold=16; c->hysteresis=2;
    c->heat_setpoint=35; c->cool_setpoint=18; c->park_pos=50;
    c->travel_time_s=120; c->direction_swap=false; c->kp=2.8f; c->ki=0.9f;
    c->gov_high=36; c->gov_low=16; c->alarm_dwell_ms=300000;
    c->enter_dwell_ms=60000; c->leave_dwell_ms=420000;
    c->deadtime_s=30.0f; c->pi_deadband_k=0.25f;
}

void tunable_apply(tunable_cfg_t *c, tunable_id_t id, const void *v){
    switch (id){
    case TUNABLE_HEAT_THRESHOLD: c->heat_threshold = *(const float*)v; break;
    case TUNABLE_COOL_THRESHOLD: c->cool_threshold = *(const float*)v; break;
    case TUNABLE_TRAVEL_TIME_S:  c->travel_time_s  = *(const uint32_t*)v; break;
    case TUNABLE_PARK_POS:       c->park_pos = ctrl_clampf(*(const float*)v, 0.0f, 100.0f); break;
    case TUNABLE_DIRECTION_SWAP: c->direction_swap = *(const bool*)v; break;
    case TUNABLE_KP:             c->kp = ctrl_clampf(*(const float*)v, 0.5f, 15.0f); break;
    case TUNABLE_KI:             c->ki = ctrl_clampf(*(const float*)v, 0.0f, 5.0f); break;
    case TUNABLE_GOV_HIGH:       c->gov_high = *(const float*)v; break;
    case TUNABLE_GOV_LOW:        c->gov_low = *(const float*)v; break;
    case TUNABLE_ALARM_DWELL_MS: c->alarm_dwell_ms = *(const uint32_t*)v; break;
    case TUNABLE_HEAT_SETPOINT:  c->heat_setpoint = ctrl_clampf(*(const float*)v, 17.0f, 35.0f); break;
    case TUNABLE_COOL_SETPOINT:  c->cool_setpoint = ctrl_clampf(*(const float*)v, 17.0f, 35.0f); break;
    case TUNABLE_DEADTIME_S:     c->deadtime_s = ctrl_clampf(*(const float*)v, 0.0f, 120.0f); break;
    case TUNABLE_PI_DEADBAND:    c->pi_deadband_k = ctrl_clampf(*(const float*)v, 0.0f, 1.0f); break;
    }
}
