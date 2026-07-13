#pragma once
#include "ctrl_core/types.h"

#define PI_HOLD_CYCLES 3

typedef struct { float integ; int hold; } pi_state_t;
typedef struct { float kp; float ki; float out_min; float out_max; } pi_cfg_t;

void  pi_init(pi_state_t *s);
void  pi_reset(pi_state_t *s);        /* integ=0, hold=0 (water_running OFF) */
void  pi_mode_change(pi_state_t *s);  /* integ=0, hold=PI_HOLD_CYCLES */
float pi_step(pi_state_t *s, float pos_ff, float err_supply, bool cooling, bool freeze,
              const pi_cfg_t *cfg);
