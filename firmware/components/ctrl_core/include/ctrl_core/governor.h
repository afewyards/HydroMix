#pragma once
#include "ctrl_core/types.h"

typedef struct { bool active; } gov_state_t;
typedef struct { float gov_high; float gov_low; float rel_high; float rel_low; } gov_cfg_t;

void  gov_init(gov_state_t *s);
float gov_step(gov_state_t *s, float target_in, float t_supply, const gov_cfg_t *cfg);
