#pragma once
#include "ctrl_core/types.h"

#define RESYNC_SRC_GATE_K         2.0f
#define RESYNC_GATE_GOV_MARGIN_K  1.0f
#define RESYNC_DEFER_MAX_MS       1800000u
#define RESYNC_NEAR_END_PCT       50.0f

typedef enum { RESYNC_ACT_NONE, RESYNC_ACT_START_SOURCE, RESYNC_ACT_START_RECIRC } resync_action_t;
typedef struct { bool deferring; uint32_t defer_since_ms; } resync_policy_state_t;

void resync_policy_init(resync_policy_state_t *s);
void resync_gate_eval(float t_src_f, bool src_fault, bool mode_active, float t_set,
                      float gov_low, float gov_high, bool *ok, bool *hard_fail);
resync_action_t resync_policy_step(resync_policy_state_t *s, bool needs_resync, bool gate_ok,
                                   float pos_pct, uint32_t now_ms);
bool resync_policy_mid_stroke_abort(bool toward_source, bool gate_hard_fail);
