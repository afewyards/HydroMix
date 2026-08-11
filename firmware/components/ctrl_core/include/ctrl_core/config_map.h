#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "ctrl_core/types.h"

/* New ids are APPENDED -- existing values must not shift, because main/config.c
 * static-asserts the ATTR_* -> TUNABLE_* mapping against them. */
typedef enum {
    TUNABLE_HEAT_THRESHOLD=0, TUNABLE_COOL_THRESHOLD, TUNABLE_TRAVEL_TIME_S,
    TUNABLE_PARK_POS, TUNABLE_DIRECTION_SWAP, TUNABLE_KP, TUNABLE_KI,
    TUNABLE_GOV_HIGH, TUNABLE_GOV_LOW, TUNABLE_ALARM_DWELL_MS,
    TUNABLE_HEAT_SETPOINT, TUNABLE_COOL_SETPOINT,
    TUNABLE_DEADTIME_S, TUNABLE_PI_DEADBAND,
    /* appended in 1.7.0 */
    TUNABLE_VALVE_DEADBAND, TUNABLE_HYSTERESIS,
    TUNABLE_ENTER_DWELL_MS, TUNABLE_LEAVE_DWELL_MS,
    TUNABLE_COUNT
} tunable_id_t;

/* Field order is load-bearing: main/config.h embeds this as config_t's first member and
 * static-asserts that every offset is unchanged, because the v3 NVS blob is discriminated
 * by sizeof(config_t). Add new fields at the END, and only with a CONFIG_VERSION bump. */
typedef struct {
    float heat_threshold, cool_threshold, hysteresis, heat_setpoint, cool_setpoint, park_pos;
    uint32_t travel_time_s; bool direction_swap; float kp, ki, gov_high, gov_low;
    uint32_t alarm_dwell_ms, enter_dwell_ms, leave_dwell_ms;
    float deadtime_s, pi_deadband_k, valve_deadband_pct;
} tunable_cfg_t;

typedef enum { TK_F32 = 0, TK_U32, TK_BOOL } tunable_kind_t;

/* The single definition of every tunable's location, type and legal range. Both
 * tunable_apply() and tunable_clamp_all() read it, so the two cannot disagree. */
typedef struct {
    tunable_kind_t kind;
    uint16_t       off;        /* offsetof(tunable_cfg_t, field) */
    float          flo, fhi;   /* TK_F32 bounds */
    uint32_t       ulo, uhi;   /* TK_U32 bounds */
} tunable_spec_t;

const tunable_spec_t *tunable_spec(tunable_id_t id);   /* NULL if id is out of range */

void tunable_cfg_defaults(tunable_cfg_t *c);
/* Clamp one field from a raw value of the matching type. Unknown id: no-op. */
void tunable_apply(tunable_cfg_t *c, tunable_id_t id, const void *val);
/* Clamp every field at once -- the load path's defensive pass over a stored blob. */
void tunable_clamp_all(tunable_cfg_t *c);

/* NaN -> keep cur; finite -> clamp to [lo,hi]. A NaN passes both ctrl_clampf()
 * comparisons as false and would otherwise flow through unmodified. */
float    tunable_sane_f(float cur, float v, float lo, float hi);
uint32_t tunable_clamp_u32(uint32_t v, uint32_t lo, uint32_t hi);

/* The interlock enforces a minimum drive pulse, so the smallest motion the motor can
 * make is (min_pulse/travel_time)*100 %. The deadband must exceed half that quantum or
 * the position estimator ping-pongs across the band edge on every pulse -- reviewer
 * simulation found a constant floor below this limit-cycles the motor at 16.7 % duty
 * with a resync roughly every 10 min. 1.2x margin on the bare half-quantum. Written off
 * INTERLOCK_MIN_PULSE_MS (not a hardcoded 120) so it tracks any future change to the
 * interlock's minimum pulse; at the shipped default travel_time_s=120 this evaluates to
 * exactly 1.0 %, i.e. the shipped valve_deadband_pct sits right at the floor. */
float valve_deadband_floor_pct(uint32_t travel_time_s);
