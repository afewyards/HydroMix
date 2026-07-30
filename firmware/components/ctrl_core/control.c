#include "ctrl_core/control.h"
#include <math.h>

void control_init(control_state_t *s){
    mode_detect_init(&s->mode);
    ff_init(&s->ff);
    pi_init(&s->pi);
    gov_init(&s->gov);
    alarm_init(&s->alarm);
    s->last_mode = MODE_IDLE;
    s->prev_resync = false;
    s->inited = true;
    s->have_now = false;
    s->have_pos = false;
    s->holding = false;
    s->latched_trim = 0.0f;
    s->last_pos = 0.0f;
    s->last_now_ms = 0;
    s->hold_until_ms = 0;
    s->hold_supply_ref = 0.0f;
}

/* PI wrapped in the transit hold: after real valve movement, latch the trim and
 * freeze the integrator until the pipe answers (deadtime_s or >0.25 K supply move).
 * prev_pos/had_pos are the PREVIOUS cycle's valve position (control_step records the
 * current one into state before calling this). */
static float pi_transit(control_state_t *s, const control_in_t *in, const control_cfg_t *cfg,
                        const pi_cfg_t *pc, float pos_ff, float t_set, bool cooling, bool freeze,
                        float dt_s, float prev_pos, bool had_pos, uint32_t now){
    if (freeze){ s->holding = false; s->latched_trim = 0.0f; }   /* freeze semantics outrank the hold */

    if (s->holding &&
        ((int32_t)(now - s->hold_until_ms) >= 0 ||
         fabsf(in->t_supply - s->hold_supply_ref) > TRANSIT_RELEASE_K))
        s->holding = false;

    float target;
    if (s->holding){
        /* The trim was latched under whatever strategy was active when the hold
         * armed (e.g. CTRL_PI_ONLY, trim_max=100). If the strategy flips back to
         * CTRL_FULL (probe recovery) while still holding, replaying that trim
         * onto the FF baseline must be re-bounded to the CURRENT strategy's
         * authority, not the one that produced it. */
        float tm = (pc->trim_max > 0.0f) ? pc->trim_max : PI_TRIM_CLAMP_PCT;
        target = ctrl_clampf(pos_ff + ctrl_clampf(s->latched_trim, -tm, tm), pc->out_min, pc->out_max);
    } else {
        target = pi_step(&s->pi, pos_ff, t_set - in->t_supply, cooling, freeze, dt_s, pc);
        s->latched_trim = target - pos_ff;
    }

    if (had_pos && fabsf(in->valve_pos - prev_pos) > TRANSIT_MOVE_PCT){
        s->holding = true;                                   /* (re)arm on any movement */
        s->hold_until_ms = now + (uint32_t)(cfg->deadtime_s * 1000.0f);
        s->hold_supply_ref = in->t_supply;
    }
    return target;
}

control_out_t control_step(control_state_t *s, const control_in_t *in,
                           const control_cfg_t *cfg, uint32_t now){
    control_out_t o = {0};

    float dt_s = s->have_now ? ctrl_clampf((now - s->last_now_ms) / 1000.0f, 1.0f, 120.0f) : 10.0f;
    s->last_now_ms = now; s->have_now = true;

    float prev_pos = s->last_pos; bool had_pos = s->have_pos;
    s->last_pos = in->valve_pos; s->have_pos = true;

    /* Mode (HX-A). Fault -> hold last mode (mode_detect handles invalid). */
    ctrl_mode_t mode = mode_detect_step(&s->mode, in->hx_a, !in->faults.hx_a, &cfg->mode_cfg, now);
    o.mode = mode;

    if (mode != s->last_mode){
        /* 3-cycle FF-only hold stretches in wall-clock time while a transit hold is
         * latched (pi_step, which decrements pi.hold, isn't called during a hold). */
        pi_mode_change(&s->pi); s->last_mode = mode;
        s->holding = false; s->latched_trim = 0.0f;
    }

    /* Resync falling edge -> re-seed FF fresh with integrator 0. */
    if (s->prev_resync && !in->resync_active){
        pi_reset(&s->pi);
        s->holding = false; s->latched_trim = 0.0f;
    }
    s->prev_resync = in->resync_active;

    /* Telemetry (always). */
    o.supply_alarm = alarm_supply_step(&s->alarm, in->t_supply, cfg->alarm_dwell_ms, now);
    degradation_out_t deg = degradation_eval(&in->faults, mode, cfg->park_pos);
    o.fault_bits = deg.alarm_bits;
    o.strategy   = deg.strategy;

    /* water_running OFF: park, clear integrator, not regulating. */
    if (!in->water_running){
        pi_reset(&s->pi);
        s->holding = false; s->latched_trim = 0.0f;
        o.valve_target = cfg->park_pos;
        o.regulating = false;
        return o;
    }
    o.regulating = true;

    /* IDLE: park. */
    if (mode == MODE_IDLE){ o.valve_target = cfg->park_pos; return o; }

    /* Effective setpoint (+ autonomous cooling link guard). */
    bool cooling = (mode == MODE_COOLING);
    float eff_cool = cooling_link_guard(cfg->cool_setpoint, mode, in->link_up,
                                        in->link_last_seen_ms, now);
    float t_set = cooling ? ctrl_clampf(eff_cool, 17.0f, 35.0f)
                          : ctrl_clampf(cfg->heat_setpoint, 17.0f, 35.0f);

    float target;
    bool  supply_ok = !in->faults.supply;
    bool  freeze_pi = in->resync_active;

    switch (deg.strategy){
    case CTRL_PARK:
        pi_reset(&s->pi);
        s->holding = false; s->latched_trim = 0.0f;
        o.valve_target = deg.park_pos;
        return o;
    case CTRL_FF_ONLY: {
        ff_result_t ff = ff_step(&s->ff, t_set, in->t_return_f, in->t_source_f);
        target = ctrl_clampf(ff.pos_ff + deg.ff_bias_pct, 0.0f, 100.0f);
        break;                                              /* no supply -> no PI, no gov */
    }
    case CTRL_PI_ONLY: {
        /* No usable source/return -> pure PI around park baseline. */
        pi_cfg_t pc = cfg->pi_cfg;
        pc.trim_max = pc.out_max - pc.out_min;   /* PI is the whole controller here — full authority */
        target = pi_transit(s, in, cfg, &pc, cfg->park_pos, t_set, cooling, freeze_pi, dt_s, prev_pos, had_pos, now);
        break;
    }
    case CTRL_FULL:
    default: {
        ff_result_t ff = ff_step(&s->ff, t_set, in->t_return_f, in->t_source_f);
        bool freeze = freeze_pi || ff.frozen;               /* low authority freezes integrator */
        pi_cfg_t pc = cfg->pi_cfg;                          /* trim_max stays 0 -> default ±20, FF carries the baseline */
        target = pi_transit(s, in, cfg, &pc, ff.pos_ff, t_set, cooling, freeze, dt_s, prev_pos, had_pos, now);
        break;
    }
    }

    /* Governor: outermost, needs supply. */
    if (supply_ok) target = gov_step(&s->gov, target, in->t_supply, &cfg->gov_cfg);

    o.valve_target = ctrl_clampf(target, 0.0f, 100.0f);
    return o;
}
