#include "ctrl_core/feedforward.h"

void ff_init(ff_state_t *s){ s->last_valid = FF_DEFAULT_PCT; s->has_valid = false; }

ff_result_t ff_step(ff_state_t *s, float t_set, float t_ret, float t_src){
    ff_result_t r;
    float denom = t_src - t_ret;
    if (fabsf(denom) < FF_MIN_AUTHORITY_K){
        r.pos_ff = s->has_valid ? s->last_valid : FF_DEFAULT_PCT;
        r.frozen = true;
        return r;
    }
    float raw = (t_set - t_ret) / denom * 100.0f;
    r.pos_ff = ctrl_clampf(raw, 0.0f, 100.0f);
    r.frozen = false;
    s->last_valid = r.pos_ff; s->has_valid = true;
    return r;
}
