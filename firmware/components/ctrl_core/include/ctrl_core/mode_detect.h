#pragma once
#include "ctrl_core/types.h"

typedef struct {
    ctrl_mode_t mode;
    ctrl_mode_t candidate;
    uint32_t    cand_since_ms;
    bool        has_valid;
} mode_detect_state_t;

typedef struct {
    float    heat_threshold;
    float    cool_threshold;
    float    hysteresis;
    uint32_t enter_dwell_ms;
    uint32_t leave_dwell_ms;
} mode_cfg_t;

void        mode_detect_init(mode_detect_state_t *s);
ctrl_mode_t mode_detect_step(mode_detect_state_t *s, float hx_a, bool hx_a_valid,
                             const mode_cfg_t *cfg, uint32_t now_ms);
