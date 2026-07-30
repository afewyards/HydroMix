#pragma once
#include "ctrl_core/types.h"

#define PI_HOLD_CYCLES 3

typedef struct { float integ; int hold; } pi_state_t;
typedef struct { float kp; float ki; /* %/K per minute */ float out_min; float out_max;
                 float deadband_k; /* gap-form error deadband, K; 0 = disabled */ } pi_cfg_t;

void  pi_init(pi_state_t *s);
void  pi_reset(pi_state_t *s);        /* integ=0, hold=0 (water_running OFF) */
void  pi_mode_change(pi_state_t *s);  /* integ=0, hold=PI_HOLD_CYCLES */
float pi_step(pi_state_t *s, float pos_ff, float err_supply, bool cooling, bool freeze,
              float dt_s, const pi_cfg_t *cfg);
