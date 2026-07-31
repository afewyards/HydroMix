#include "ctrl_core/config_map.h"
#include <math.h>

/* NaN -> reject (keep cur); finite -> clamp to [lo,hi]. Mirrors config.c's
 * sane_f(): a NaN passes both ctrl_clampf() comparisons as false and would
 * otherwise flow through unmodified. */
static float sane_f(float cur, float v, float lo, float hi)
{
    if (isnan(v)) return cur;
    return ctrl_clampf(v, lo, hi);
}

/* Range-only clamp for u32 tunables (NaN is a float-only concept). */
static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

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
    case TUNABLE_HEAT_THRESHOLD: c->heat_threshold = sane_f(c->heat_threshold, *(const float*)v, 10.0f, 60.0f); break;
    case TUNABLE_COOL_THRESHOLD: c->cool_threshold = sane_f(c->cool_threshold, *(const float*)v, 0.0f, 40.0f); break;
    case TUNABLE_TRAVEL_TIME_S:  c->travel_time_s  = clamp_u32(*(const uint32_t*)v, 30, 600); break;
    case TUNABLE_PARK_POS:       c->park_pos = sane_f(c->park_pos, *(const float*)v, 0.0f, 100.0f); break;
    case TUNABLE_DIRECTION_SWAP: c->direction_swap = *(const bool*)v; break;
    case TUNABLE_KP:             c->kp = sane_f(c->kp, *(const float*)v, 0.5f, 15.0f); break;
    case TUNABLE_KI:             c->ki = sane_f(c->ki, *(const float*)v, 0.0f, 5.0f); break;
    /* release band is 35/17 (control_task.c) — trip thresholds must stay outside it or the governor limit-cycles */
    case TUNABLE_GOV_HIGH:       c->gov_high = sane_f(c->gov_high, *(const float*)v, 35.0f, 60.0f); break;
    case TUNABLE_GOV_LOW:        c->gov_low = sane_f(c->gov_low, *(const float*)v, 0.0f, 17.0f); break;
    case TUNABLE_ALARM_DWELL_MS: c->alarm_dwell_ms = clamp_u32(*(const uint32_t*)v, 10000, 3600000); break;
    case TUNABLE_HEAT_SETPOINT:  c->heat_setpoint = sane_f(c->heat_setpoint, *(const float*)v, 17.0f, 35.0f); break;
    case TUNABLE_COOL_SETPOINT:  c->cool_setpoint = sane_f(c->cool_setpoint, *(const float*)v, 17.0f, 35.0f); break;
    case TUNABLE_DEADTIME_S:     c->deadtime_s = sane_f(c->deadtime_s, *(const float*)v, 0.0f, 120.0f); break;
    case TUNABLE_PI_DEADBAND:    c->pi_deadband_k = sane_f(c->pi_deadband_k, *(const float*)v, 0.0f, 1.0f); break;
    }
}
