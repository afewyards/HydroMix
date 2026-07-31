#pragma once
#include "ctrl_core/types.h"

#define ALARM_SUPPLY_HIGH       36.5f
#define ALARM_SUPPLY_LOW        15.5f
#define ALARM_CLEAR_HIGH        35.5f
#define ALARM_CLEAR_LOW         16.5f
#define LINK_LOSS_COOLING_MS    1800000u   /* 30 min */
#define LINK_LOSS_COOL_SETPOINT 21.0f

typedef struct {
    bool     alarmed;
    bool     out_of_bounds;
    uint32_t oob_since_ms;
} alarm_state_t;

void  alarm_init(alarm_state_t *s);
bool  alarm_supply_step(alarm_state_t *s, float t_supply, uint32_t dwell_ms, uint32_t now_ms);

/* Autonomous dew-point guard. Raises the cooling setpoint toward LINK_LOSS_COOL_SETPOINT
 * once we have not heard from the coordinator for LINK_LOSS_COOLING_MS, so an unattended
 * device cannot keep chilling a floor below dew point. Deliberately NOT gated on a ZDO
 * link-down signal: a Router whose coordinator dies silently never emits one, which is
 * precisely the scenario this exists for. last_seen_ms is refreshed by inbound ZCL
 * traffic AND by join/leave signals (control_task_note_link_activity / _set_link).
 * Only ever raises the setpoint -- never lowers it. */
float cooling_link_guard(float cool_setpoint, ctrl_mode_t mode,
                         uint32_t last_seen_ms, uint32_t now_ms);
