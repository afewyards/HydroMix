#pragma once
#include "ctrl_core/types.h"

#define POS_RESYNC_TRAVEL_PCT 300.0f
#define POS_RESYNC_REVERSALS  50u

typedef struct {
    float    position_pct;
    float    accum_travel_pct;
    uint32_t reversals;
    int8_t   last_sign;   /* -1,0,+1 */
} pos_est_state_t;

void pos_est_init(pos_est_state_t *s);
void pos_est_update(pos_est_state_t *s, int8_t travel_sign, uint32_t dt_ms, float travel_time_s);
bool pos_est_needs_resync(const pos_est_state_t *s);
void pos_est_resync_done(pos_est_state_t *s, float seed_pct);
