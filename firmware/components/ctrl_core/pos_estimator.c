#include "ctrl_core/pos_estimator.h"

void pos_est_init(pos_est_state_t *s){
    s->position_pct = 0.0f; s->accum_travel_pct = 0.0f; s->reversals = 0; s->last_sign = 0;
}

void pos_est_update(pos_est_state_t *s, int8_t sign, uint32_t dt_ms, float travel_time_s){
    if (sign == 0) return;                       /* stopped: last_sign unchanged */
    if (s->last_sign != 0 && sign != s->last_sign) s->reversals++;
    s->last_sign = sign;
    if (travel_time_s <= 0.0f) return;
    float delta = ((float)dt_ms / 1000.0f) / travel_time_s * 100.0f;
    s->accum_travel_pct += delta;
    s->position_pct = ctrl_clampf(s->position_pct + (sign > 0 ? delta : -delta), 0.0f, 100.0f);
}

bool pos_est_needs_resync(const pos_est_state_t *s){
    return s->accum_travel_pct >= POS_RESYNC_TRAVEL_PCT || s->reversals >= POS_RESYNC_REVERSALS;
}

void pos_est_resync_done(pos_est_state_t *s){
    s->position_pct = 0.0f; s->accum_travel_pct = 0.0f; s->reversals = 0; s->last_sign = 0;
}
