#include "ctrl_core/resync_policy.h"
#include <math.h>

void resync_policy_init(resync_policy_state_t *s){
    s->deferring = false;
    s->defer_since_ms = 0;
}

void resync_gate_eval(float t_src_f, bool src_fault, bool mode_active, float t_set,
                      float gov_low, float gov_high, bool *ok, bool *hard_fail){
    *hard_fail = src_fault || !mode_active || t_src_f < gov_low || t_src_f > gov_high;
    *ok = !*hard_fail
          && fabsf(t_src_f - t_set) <= RESYNC_SRC_GATE_K
          && t_src_f >= gov_low  + RESYNC_GATE_GOV_MARGIN_K
          && t_src_f <= gov_high - RESYNC_GATE_GOV_MARGIN_K;
}

resync_action_t resync_policy_step(resync_policy_state_t *s, bool needs_resync, bool gate_ok,
                                   float pos_pct, uint32_t now_ms){
    if (!needs_resync){ s->deferring = false; return RESYNC_ACT_NONE; }
    if (pos_pct < RESYNC_NEAR_END_PCT){ s->deferring = false; return RESYNC_ACT_START_RECIRC; }
    if (gate_ok){ s->deferring = false; return RESYNC_ACT_START_SOURCE; }
    if (!s->deferring){ s->deferring = true; s->defer_since_ms = now_ms; return RESYNC_ACT_NONE; }
    if ((uint32_t)(now_ms - s->defer_since_ms) >= RESYNC_DEFER_MAX_MS){
        s->deferring = false; return RESYNC_ACT_START_RECIRC;
    }
    return RESYNC_ACT_NONE;
}

bool resync_policy_mid_stroke_abort(bool toward_source, bool gate_hard_fail){
    return toward_source && gate_hard_fail;
}
