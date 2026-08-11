#include "ctrl_core/config_map.h"
#include "ctrl_core/interlock.h"
#include <math.h>
#include <stddef.h>

float tunable_sane_f(float cur, float v, float lo, float hi)
{
    if (isnan(v)) return cur;
    return ctrl_clampf(v, lo, hi);
}

uint32_t tunable_clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

float valve_deadband_floor_pct(uint32_t travel_time_s)
{
    if (travel_time_s == 0) return 5.0f;      /* degenerate: clamp to the ceiling */
    float q = 1.2f * (INTERLOCK_MIN_PULSE_MS / 1000.0f) * 100.0f / (2.0f * (float)travel_time_s);
    return q > 0.2f ? q : 0.2f;
}

#define F(field) (uint16_t)offsetof(tunable_cfg_t, field)

/* THE table. Every clamp range in the firmware is defined here exactly once.
 * gov_high/gov_low bounds keep the governor's trip thresholds strictly outside the
 * 35/17 release band (control_task.c) -- inside it, the governor limit-cycles. Observed
 * live with gov_low written as 17, sitting on the band edge. */
static const tunable_spec_t SPEC[TUNABLE_COUNT] = {
    [TUNABLE_HEAT_THRESHOLD] = { TK_F32,  F(heat_threshold),    10.0f, 60.0f,  0, 0 },
    [TUNABLE_COOL_THRESHOLD] = { TK_F32,  F(cool_threshold),     0.0f, 40.0f,  0, 0 },
    [TUNABLE_TRAVEL_TIME_S]  = { TK_U32,  F(travel_time_s),      0.0f,  0.0f, 30, 600 },
    [TUNABLE_PARK_POS]       = { TK_F32,  F(park_pos),           0.0f, 100.0f, 0, 0 },
    [TUNABLE_DIRECTION_SWAP] = { TK_BOOL, F(direction_swap),     0.0f,  0.0f,  0, 0 },
    [TUNABLE_KP]             = { TK_F32,  F(kp),                 0.5f, 15.0f,  0, 0 },
    [TUNABLE_KI]             = { TK_F32,  F(ki),                 0.0f,  5.0f,  0, 0 },
    [TUNABLE_GOV_HIGH]       = { TK_F32,  F(gov_high),          36.0f, 60.0f,  0, 0 },
    [TUNABLE_GOV_LOW]        = { TK_F32,  F(gov_low),            0.0f, 16.0f,  0, 0 },
    [TUNABLE_ALARM_DWELL_MS] = { TK_U32,  F(alarm_dwell_ms),     0.0f,  0.0f, 10000, 3600000 },
    [TUNABLE_HEAT_SETPOINT]  = { TK_F32,  F(heat_setpoint),     17.0f, 35.0f,  0, 0 },
    [TUNABLE_COOL_SETPOINT]  = { TK_F32,  F(cool_setpoint),     17.0f, 35.0f,  0, 0 },
    [TUNABLE_DEADTIME_S]     = { TK_F32,  F(deadtime_s),         0.0f, 120.0f, 0, 0 },
    [TUNABLE_PI_DEADBAND]    = { TK_F32,  F(pi_deadband_k),      0.0f,  1.0f,  0, 0 },
    /* flo is the STATIC floor; the real floor is dynamic, see valve_deadband_floor_pct() */
    [TUNABLE_VALVE_DEADBAND] = { TK_F32,  F(valve_deadband_pct), 0.2f,  5.0f,  0, 0 },
    [TUNABLE_HYSTERESIS]     = { TK_F32,  F(hysteresis),         0.0f, 10.0f,  0, 0 },
    [TUNABLE_ENTER_DWELL_MS] = { TK_U32,  F(enter_dwell_ms),     0.0f,  0.0f, 10000, 3600000 },
    [TUNABLE_LEAVE_DWELL_MS] = { TK_U32,  F(leave_dwell_ms),     0.0f,  0.0f, 10000, 7200000 },
};

#undef F

const tunable_spec_t *tunable_spec(tunable_id_t id)
{
    if ((int)id < 0 || (int)id >= TUNABLE_COUNT) return NULL;
    return &SPEC[id];
}

void tunable_cfg_defaults(tunable_cfg_t *c){
    c->heat_threshold=28; c->cool_threshold=16; c->hysteresis=2;
    c->heat_setpoint=35; c->cool_setpoint=18; c->park_pos=50;
    c->travel_time_s=120; c->direction_swap=false; c->kp=2.8f; c->ki=0.9f;
    c->gov_high=36; c->gov_low=16; c->alarm_dwell_ms=300000;
    c->enter_dwell_ms=60000; c->leave_dwell_ms=420000;
    c->deadtime_s=30.0f; c->pi_deadband_k=0.25f; c->valve_deadband_pct=1.0f;
}

void tunable_apply(tunable_cfg_t *c, tunable_id_t id, const void *v)
{
    const tunable_spec_t *s = tunable_spec(id);
    if (!c || !v || !s) return;
    char *base = (char *)c;
    switch (s->kind) {
    case TK_F32: {
        float *f = (float *)(void *)(base + s->off);
        float lo = (id == TUNABLE_VALVE_DEADBAND)
                 ? valve_deadband_floor_pct(c->travel_time_s) : s->flo;
        *f = tunable_sane_f(*f, *(const float *)v, lo, s->fhi);
        break;
    }
    case TK_U32: {
        uint32_t *u = (uint32_t *)(void *)(base + s->off);
        *u = tunable_clamp_u32(*(const uint32_t *)v, s->ulo, s->uhi);
        break;
    }
    case TK_BOOL: {
        bool *b = (bool *)(void *)(base + s->off);
        *b = *(const bool *)v;
        break;
    }
    }
    /* Shortening travel raises valve_deadband_pct's stability floor, so an already-set
     * deadband must be re-clamped or it is left stranded below the new floor. */
    if (id == TUNABLE_TRAVEL_TIME_S) {
        float lo = valve_deadband_floor_pct(c->travel_time_s);
        c->valve_deadband_pct = ctrl_clampf(c->valve_deadband_pct, lo, 5.0f);
    }
}

void tunable_clamp_all(tunable_cfg_t *c)
{
    if (!c) return;
    tunable_cfg_t d;
    tunable_cfg_defaults(&d);
    char *base = (char *)c;
    const char *dbase = (const char *)&d;

    /* travel_time_s FIRST: valve_deadband_pct's floor is derived from it, so the floor
     * must be computed from the final, in-range travel value. */
    c->travel_time_s = tunable_clamp_u32(c->travel_time_s, 30, 600);

    for (int id = 0; id < TUNABLE_COUNT; ++id) {
        const tunable_spec_t *s = &SPEC[id];
        if (id == TUNABLE_VALVE_DEADBAND) continue;   /* dynamic floor, handled below */
        switch (s->kind) {
        case TK_F32: {
            float *f  = (float *)(void *)(base + s->off);
            float dv  = *(const float *)(const void *)(dbase + s->off);
            *f = tunable_sane_f(dv, *f, s->flo, s->fhi);   /* NaN -> shipped default */
            break;
        }
        case TK_U32: {
            uint32_t *u = (uint32_t *)(void *)(base + s->off);
            *u = tunable_clamp_u32(*u, s->ulo, s->uhi);
            break;
        }
        case TK_BOOL: break;                              /* always in range */
        }
    }

    /* The "cur" fallback is itself clamped into the dynamic range so a NaN-corrupted
     * stored value cannot fall back to a stale 1.0 that is below the floor for a short
     * travel_time_s. */
    float lo = valve_deadband_floor_pct(c->travel_time_s);
    c->valve_deadband_pct = tunable_sane_f(ctrl_clampf(d.valve_deadband_pct, lo, 5.0f),
                                           c->valve_deadband_pct, lo, 5.0f);
}

bool tunable_from_attr(uint16_t attr_id, tunable_id_t *out)
{
    tunable_id_t id;
    switch (attr_id) {
    case 0x0000: id = TUNABLE_HEAT_THRESHOLD; break;
    case 0x0001: id = TUNABLE_COOL_THRESHOLD; break;
    case 0x0002: id = TUNABLE_TRAVEL_TIME_S;  break;
    case 0x0003: id = TUNABLE_PARK_POS;       break;
    case 0x0004: id = TUNABLE_DIRECTION_SWAP; break;
    case 0x0005: id = TUNABLE_KP;             break;
    case 0x0006: id = TUNABLE_KI;             break;
    case 0x0007: id = TUNABLE_GOV_HIGH;       break;
    case 0x0008: id = TUNABLE_GOV_LOW;        break;
    case 0x0009: id = TUNABLE_ALARM_DWELL_MS; break;
    case 0x000E: id = TUNABLE_DEADTIME_S;     break;
    case 0x000F: id = TUNABLE_PI_DEADBAND;    break;
    case 0x0010: id = TUNABLE_HEAT_SETPOINT;  break;
    case 0x0011: id = TUNABLE_COOL_SETPOINT;  break;
    case 0x0012: id = TUNABLE_VALVE_DEADBAND; break;
    /* 0x000A resync, 0x000B alarm bitmap, 0x000C fault bitmap, 0x000D travel-since:
     * read-only or command-like, never a stored tunable. */
    default: return false;
    }
    if (out) *out = id;
    return true;
}
