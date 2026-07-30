#include "ctrl_core/control.h"

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
}

control_out_t control_step(control_state_t *s, const control_in_t *in,
                           const control_cfg_t *cfg, uint32_t now){
    control_out_t o = {0};

    float dt_s = s->have_now ? ctrl_clampf((now - s->last_now_ms) / 1000.0f, 1.0f, 120.0f) : 10.0f;
    s->last_now_ms = now; s->have_now = true;

    /* Mode (HX-A). Fault -> hold last mode (mode_detect handles invalid). */
    ctrl_mode_t mode = mode_detect_step(&s->mode, in->hx_a, !in->faults.hx_a, &cfg->mode_cfg, now);
    o.mode = mode;

    if (mode != s->last_mode){ pi_mode_change(&s->pi); s->last_mode = mode; }

    /* Resync falling edge -> re-seed FF fresh with integrator 0. */
    if (s->prev_resync && !in->resync_active) pi_reset(&s->pi);
    s->prev_resync = in->resync_active;

    /* Telemetry (always). */
    o.supply_alarm = alarm_supply_step(&s->alarm, in->t_supply, cfg->alarm_dwell_ms, now);
    degradation_out_t deg = degradation_eval(&in->faults, mode, cfg->park_pos);
    o.fault_bits = deg.alarm_bits;
    o.strategy   = deg.strategy;

    /* water_running OFF: park, clear integrator, not regulating. */
    if (!in->water_running){
        pi_reset(&s->pi);
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
        o.valve_target = deg.park_pos;
        return o;
    case CTRL_FF_ONLY: {
        ff_result_t ff = ff_step(&s->ff, t_set, in->t_return_f, in->t_source_f);
        target = ctrl_clampf(ff.pos_ff + deg.ff_bias_pct, 0.0f, 100.0f);
        break;                                              /* no supply -> no PI, no gov */
    }
    case CTRL_PI_ONLY:
        /* No usable source/return -> pure PI around park baseline. */
        target = pi_step(&s->pi, cfg->park_pos, t_set - in->t_supply, cooling, freeze_pi, dt_s, &cfg->pi_cfg);
        break;
    case CTRL_FULL:
    default: {
        ff_result_t ff = ff_step(&s->ff, t_set, in->t_return_f, in->t_source_f);
        bool freeze = freeze_pi || ff.frozen;               /* low authority freezes integrator */
        target = pi_step(&s->pi, ff.pos_ff, t_set - in->t_supply, cooling, freeze, dt_s, &cfg->pi_cfg);
        break;
    }
    }

    /* Governor: outermost, needs supply. */
    if (supply_ok) target = gov_step(&s->gov, target, in->t_supply, &cfg->gov_cfg);

    o.valve_target = ctrl_clampf(target, 0.0f, 100.0f);
    return o;
}
