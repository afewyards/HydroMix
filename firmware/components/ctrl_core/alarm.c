#include "ctrl_core/alarm.h"

void alarm_init(alarm_state_t *s){ s->alarmed = false; s->out_of_bounds = false; s->oob_since_ms = 0; }

bool alarm_supply_step(alarm_state_t *s, float t, uint32_t dwell_ms, uint32_t now){
    bool excursion = (t >= ALARM_SUPPLY_HIGH) || (t <= ALARM_SUPPLY_LOW);
    bool in_clear  = (t <= ALARM_CLEAR_HIGH) && (t >= ALARM_CLEAR_LOW);

    if (in_clear){
        s->out_of_bounds = false; s->alarmed = false; s->oob_since_ms = now;
    } else if (excursion){
        if (!s->out_of_bounds){ s->out_of_bounds = true; s->oob_since_ms = now; }
        if (now - s->oob_since_ms >= dwell_ms) s->alarmed = true;
    }
    /* hysteresis band (35.5..36.5 / 15.5..16.5): hold current state, timer unchanged */
    return s->alarmed;
}

float cooling_link_guard(float cool_setpoint, ctrl_mode_t mode,
                         uint32_t last_seen_ms, uint32_t now){
    if (mode == MODE_COOLING && (now - last_seen_ms) >= LINK_LOSS_COOLING_MS)
        return fmaxf(cool_setpoint, LINK_LOSS_COOL_SETPOINT);
    return cool_setpoint;
}
