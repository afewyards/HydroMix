#include "ctrl_core/pi.h"

void pi_init(pi_state_t *s){ s->integ = 0.0f; s->hold = 0; }
void pi_reset(pi_state_t *s){ s->integ = 0.0f; s->hold = 0; }
void pi_mode_change(pi_state_t *s){ s->integ = 0.0f; s->hold = PI_HOLD_CYCLES; }

float pi_step(pi_state_t *s, float pos_ff, float err_supply, bool cooling, bool freeze,
              float dt_s, const pi_cfg_t *cfg){
    float err = cooling ? -err_supply : err_supply;

    if (freeze) return ctrl_clampf(pos_ff, cfg->out_min, cfg->out_max);
    if (s->hold > 0){ s->hold--; return ctrl_clampf(pos_ff, cfg->out_min, cfg->out_max); }

    float e_eff = 0.0f;                      /* gap deadband: 0 inside, ramps in beyond */
    if (err >  cfg->deadband_k)      e_eff = err - cfg->deadband_k;
    else if (err < -cfg->deadband_k) e_eff = err + cfg->deadband_k;

    float p = cfg->kp * e_eff;
    float cand = s->integ + cfg->ki * e_eff * (dt_s / 60.0f);
    if (p + cand >  PI_TRIM_CLAMP_PCT) cand =  PI_TRIM_CLAMP_PCT - p;   /* back-calculate */
    if (p + cand < -PI_TRIM_CLAMP_PCT) cand = -PI_TRIM_CLAMP_PCT - p;
    float out = pos_ff + p + cand;
    float clamped = ctrl_clampf(out, cfg->out_min, cfg->out_max);
    bool saturated = (out != clamped);
    bool pushing = (clamped >= cfg->out_max && e_eff > 0.0f) ||
                   (clamped <= cfg->out_min && e_eff < 0.0f);
    if (!saturated || !pushing) s->integ = cand;   /* conditional anti-windup */
    return ctrl_clampf(pos_ff + p + s->integ, cfg->out_min, cfg->out_max);
}
