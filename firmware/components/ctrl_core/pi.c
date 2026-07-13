#include "ctrl_core/pi.h"

void pi_init(pi_state_t *s){ s->integ = 0.0f; s->hold = 0; }
void pi_reset(pi_state_t *s){ s->integ = 0.0f; s->hold = 0; }
void pi_mode_change(pi_state_t *s){ s->integ = 0.0f; s->hold = PI_HOLD_CYCLES; }

float pi_step(pi_state_t *s, float pos_ff, float err_supply, bool cooling, bool freeze,
              const pi_cfg_t *cfg){
    float err = cooling ? -err_supply : err_supply;

    if (freeze) return ctrl_clampf(pos_ff, cfg->out_min, cfg->out_max);
    if (s->hold > 0){ s->hold--; return ctrl_clampf(pos_ff, cfg->out_min, cfg->out_max); }

    float p = cfg->kp * err;
    float cand = s->integ + cfg->ki * err;
    float out = pos_ff + p + cand;
    float clamped = ctrl_clampf(out, cfg->out_min, cfg->out_max);
    bool saturated = (out != clamped);
    bool pushing = (clamped >= cfg->out_max && err > 0.0f) ||
                   (clamped <= cfg->out_min && err < 0.0f);
    if (!saturated || !pushing) s->integ = cand;   /* conditional anti-windup */
    return ctrl_clampf(pos_ff + p + s->integ, cfg->out_min, cfg->out_max);
}
