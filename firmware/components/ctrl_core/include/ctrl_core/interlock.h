#pragma once
#include "ctrl_core/types.h"

#define INTERLOCK_MIN_PULSE_MS    2000u
#define INTERLOCK_DEAD_TIME_MS     500u
#define INTERLOCK_ANTI_DITHER_MS 10000u

typedef struct {
    valve_dir_t cur_dir;
    uint32_t    since_ms;      /* when cur_dir started (moving) */
    valve_dir_t last_dir;      /* last non-stop direction (STOP until first move) */
    uint32_t    last_stop_ms;  /* when we last stopped */
    uint32_t    both_error_count;
} interlock_state_t;

typedef struct { bool open_on; bool close_on; } triac_cmd_t;

void        interlock_init(interlock_state_t *s);
triac_cmd_t interlock_step(interlock_state_t *s, bool open_req, bool close_req, uint32_t now_ms);
