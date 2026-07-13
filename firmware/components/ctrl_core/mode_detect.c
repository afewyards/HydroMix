#include "ctrl_core/mode_detect.h"

void mode_detect_init(mode_detect_state_t *s){
    s->mode = MODE_IDLE; s->candidate = MODE_IDLE; s->cand_since_ms = 0; s->has_valid = false;
}

static ctrl_mode_t classify(float hx, ctrl_mode_t cur, const mode_cfg_t *c){
    switch (cur){
    case MODE_HEATING:
        if (hx < c->heat_threshold - c->hysteresis)
            return (hx <= c->cool_threshold) ? MODE_COOLING : MODE_IDLE;
        return MODE_HEATING;
    case MODE_COOLING:
        if (hx > c->cool_threshold + c->hysteresis)
            return (hx >= c->heat_threshold) ? MODE_HEATING : MODE_IDLE;
        return MODE_COOLING;
    default:
        if (hx >= c->heat_threshold) return MODE_HEATING;
        if (hx <= c->cool_threshold) return MODE_COOLING;
        return MODE_IDLE;
    }
}

ctrl_mode_t mode_detect_step(mode_detect_state_t *s, float hx, bool valid,
                             const mode_cfg_t *c, uint32_t now){
    if (!valid) return s->mode;                  /* hold last known (boot=IDLE) */
    s->has_valid = true;
    ctrl_mode_t target = classify(hx, s->mode, c);
    if (target == s->mode){ s->candidate = s->mode; return s->mode; }
    if (target != s->candidate){ s->candidate = target; s->cand_since_ms = now; }
    uint32_t dwell = (s->mode == MODE_IDLE) ? c->enter_dwell_ms : c->leave_dwell_ms;
    if (now - s->cand_since_ms >= dwell){ s->mode = target; s->candidate = target; }
    return s->mode;
}
