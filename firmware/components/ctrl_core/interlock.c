#include "ctrl_core/interlock.h"

void interlock_init(interlock_state_t *s){
    s->cur_dir = VALVE_STOP; s->since_ms = 0;
    s->last_dir = VALVE_STOP; s->last_stop_ms = 0; s->both_error_count = 0;
}

static triac_cmd_t out(valve_dir_t d){
    triac_cmd_t c = { d == VALVE_OPEN, d == VALVE_CLOSE };
    return c;
}

triac_cmd_t interlock_step(interlock_state_t *s, bool open_req, bool close_req, uint32_t now){
    /* Never both. */
    if (open_req && close_req){ s->both_error_count++; open_req = close_req = false; }
    valve_dir_t want = open_req ? VALVE_OPEN : (close_req ? VALVE_CLOSE : VALVE_STOP);

    if (s->cur_dir != VALVE_STOP){
        /* Moving: enforce min pulse before honoring any change. */
        if (now - s->since_ms < INTERLOCK_MIN_PULSE_MS) return out(s->cur_dir);
        if (want == s->cur_dir) return out(s->cur_dir);
        /* stop */
        s->last_dir = s->cur_dir; s->cur_dir = VALVE_STOP; s->last_stop_ms = now;
        return out(VALVE_STOP);
    }

    /* Stopped. */
    if (want == VALVE_STOP) return out(VALVE_STOP);
    if (s->last_dir != VALVE_STOP){
        if (now - s->last_stop_ms < INTERLOCK_DEAD_TIME_MS) return out(VALVE_STOP);
        if (want != s->last_dir && now - s->last_stop_ms < INTERLOCK_ANTI_DITHER_MS)
            return out(VALVE_STOP);
    }
    s->cur_dir = want; s->since_ms = now; s->last_dir = want;
    return out(want);
}
