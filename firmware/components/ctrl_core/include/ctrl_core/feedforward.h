#pragma once
#include "ctrl_core/types.h"

#define FF_MIN_AUTHORITY_K 2.0f
#define FF_DEFAULT_PCT     50.0f

typedef struct { float last_valid; bool has_valid; } ff_state_t;
typedef struct { float pos_ff; bool frozen; } ff_result_t;

void        ff_init(ff_state_t *s);
ff_result_t ff_step(ff_state_t *s, float t_set, float t_return_f, float t_source_f);
