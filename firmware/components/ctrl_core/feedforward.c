#include "ctrl_core/feedforward.h"

void ff_cfg_defaults(ff_cfg_t *c){
    c->coupling_pct_k = FF_COUPLING_PCT_K;
    c->auth_min_k     = FF_AUTHORITY_MIN_K;
    c->auth_max_k     = FF_AUTHORITY_MAX_K;
    c->out_tau_s      = FF_OUT_TAU_S;
}

void ff_init(ff_state_t *s){
    s->out = FF_DEFAULT_PCT; s->has_out = false;
    s->last_ms = 0; s->has_last_ms = false;
}

void ff_reseed(ff_state_t *s){ s->has_out = false; }

void ff_mode_change(ff_state_t *s){ ff_reseed(s); }

/* Smallest |t_src - t_ret| the FF is allowed to divide by.
 *
 * Historically this was derived from the source<->valve coupling, to keep the FF out of a
 * region where it was believed to reinforce its own error:
 *
 *     G = |t_ret - t_set| * 100 * coupling_pct_k / denom^2
 *
 * with the floor placed where G reaches 1. That derivation is unsound: G is a MAGNITUDE,
 * and sign is what decides whether a feedback path diverges. The coupling's measured sign
 * makes this path negative feedback, so |G| > 1 is a fast bounded response, not a runaway
 * (see FF_COUPLING_PCT_K for the measurement and how the original fit was confounded).
 *
 * With FF_COUPLING_PCT_K at 0 this returns auth_min_k, and the floor is simply a guard
 * against dividing by a near-zero denominator. The derivation is kept because it is a
 * small diff to re-enable IF the coupling is ever measured open-loop. */
float ff_authority_floor(const ff_cfg_t *c, float t_set, float t_ret){
    /* A zeroed ff_cfg_t (a caller that brace-initialised control_cfg_t without filling
     * this in) must degrade to the fixed floor, NOT to a floor of zero -- ff_step divides
     * by this value when authority is low, so it has to stay strictly positive. */
    float lo = (c->auth_min_k > 0.0f) ? c->auth_min_k : FF_AUTHORITY_MIN_K;
    float hi = (c->auth_max_k > lo)   ? c->auth_max_k : lo;

    float d = lo;
    if (c->coupling_pct_k > 0.0f)
        d = sqrtf(fabsf(t_ret - t_set) * 100.0f * c->coupling_pct_k);
    return ctrl_clampf(d, lo, hi);
}

ff_result_t ff_step(ff_state_t *s, const ff_cfg_t *c, float t_set, float t_ret,
                    float t_src, uint32_t now_ms){
    ff_result_t r;

    /* Unsigned difference, so the ms tick wrapping at ~49.7 days still yields a sane dt. */
    float dt_s = s->has_last_ms ? (float)(uint32_t)(now_ms - s->last_ms) / 1000.0f : 0.0f;
    s->last_ms = now_ms; s->has_last_ms = true;

    r.authority_floor_k = ff_authority_floor(c, t_set, t_ret);

    float want  = t_set - t_ret;        /* < 0 = cooling demand, > 0 = heating demand */
    float denom = t_src - t_ret;
    r.frozen = fabsf(denom) < r.authority_floor_k;

    /* LIMIT the denominator, and BLEND across its sign flip; never abandon the division.
     *
     * Up to 1.5.0, low authority froze the FF at its last valid output and control.c handed
     * that frozen constant straight to the valve (pi.c discards the entire PI output while
     * frozen, both P and I). That is open loop, and it latched: a pinned valve starves the
     * source branch, a starved branch stops being cooled, t_src drifts toward t_ret, |denom|
     * shrinks further, and the freeze can never end. Live on 2026-08-04 that ran for 92
     * minutes, with |denom| down to 0.07 K and supply 2.7 K above setpoint.
     *
     * Clamping the denominator instead keeps the output bounded and live. But clamping the
     * MAGNITUDE while keeping the measured SIGN -- which is what 1.5.1 did -- is a step
     * discontinuity at denom = 0, and the step is the entire output range. `want` cannot
     * change sign within a mode, so at low authority the ratio's sign, and therefore which
     * rail it clamps to, is decided by sign(denom) alone. At t_ret 20.9 / t_set 18.5,
     * t_src 20.89 gives -2.4/-2.0 = +120 -> 100 %, and t_src 20.91 gives -2.4/+2.0 = -120
     * -> 0 %. Two probe readings 0.02 K apart command opposite ends of the valve, and 0.02 K
     * is far below what a pair of DS18B20s agree to.
     *
     * Live on 2026-08-08 that is what happened: dT sat inside +/-0.05 K for about 4.5 h, the
     * FF chattered rail to rail (the 180 s output EMA turning it into a sawtooth), the plant
     * lost cooling output, and the valve logged 2906 % of travel against ~750 % on a normal
     * day, with 6 end-stop resyncs.
     *
     * So blend the two available answers rather than switching between them:
     *
     *   raw_meas   -- trust the measured sign, magnitude floored. The right answer whenever
     *                 the sign is real.
     *   raw_demand -- put the denominator on the side the DEMAND needs. That is the
     *                 direction which restores authority: the source<->valve coupling is
     *                 negative (see FF_COUPLING_PCT_K, about -0.041 K/% measured open loop),
     *                 so opening toward the source cools it, grows |denom|, and re-earns the
     *                 division. Shutting to 0 % instead pushes t_src further past t_ret on
     *                 the wrong side and entrenches the condition that closed the valve.
     *
     * weighted by w = |denom| / floor, which is exactly the confidence the measured sign has
     * earned: full weight at the floor, none at denom = 0 where the reading has no sign of
     * its own. The blend only exists inside the band; outside it w = 1 identically.
     *
     * WHICH WAY raw_demand POINTS IS A PLANT PROPERTY, NOT A LAW. It is only "open toward the
     * source" because this plant runs a FIXED-SPEED pump, where valve position is the sole
     * modulator of source-branch flow: closing starves the branch, a starved branch stops
     * being cooled at all, and t_src rises toward t_ret and then past it. Flow starvation is
     * what makes the coupling negative here, and it dominates only because nothing else can
     * move the flow.
     *
     * Anyone adding PWM pump control must re-check this rather than inherit it. An
     * independently modulated pump breaks the premise outright -- flow stops being a function
     * of valve position, the pump can supply the flow the valve is withholding, and the
     * textbook term of the OPPOSITE sign (HX approach rising with secondary flow) is free to
     * win. If the coupling turns positive, raw_demand points the wrong way and this fallback
     * drives the valve into the condition it was meant to escape. Re-measure open loop before
     * assuming it carries over -- the standard FF_COUPLING_PCT_K already sets, and for the
     * same reason: a closed-loop fit on this exact path produced the wrong SIGN once already.
     * The blend structure itself is sign-agnostic and would survive; only the direction
     * raw_demand encodes is at stake.
     *
     * The properties this buys, in the order they matter:
     *   - Source on the USEFUL side (colder in cooling, hotter in heating): sgn(denom) ==
     *     sgn(want), so raw_meas and raw_demand are the same expression and the blend is a
     *     no-op at every |denom|. The healthy operating band is bit-for-bit unchanged -- the
     *     one exception being the sign of zero when t_set == t_ret exactly, where 1.5.1
     *     returned -0.0 and this returns +0.0, and the command is 0 % either way.
     *   - |denom| >= floor: w = 1, so a source genuinely on the wrong side -- a 3 K warm
     *     source during cooling, the unseated-probe case of 2026-07-31 -- still clamps to
     *     0 %. A source that cannot help is still shut out. Only the near-zero band, where
     *     the sign is noise, is reinterpreted.
     *   - denom = 0: w = 0, so the answer is the demand direction and the valve opens toward
     *     the source. The saturation IS the correct response here, not an error to suppress.
     *   - Continuous everywhere, including at |denom| = floor, so the command now tracks the
     *     probes instead of snapping between rails on their last significant bit.
     *
     * Note this is a re-weighting of the low-authority answer, not a new authority concept:
     * pi.c and control.c are untouched, and r.frozen keeps its meaning (denominator was
     * limited) for telemetry. */
    float raw;
    if (!r.frozen){
        raw = ctrl_clampf(want / denom * 100.0f, 0.0f, 100.0f);
    } else {
        /* want == 0 has no demand direction either; resolve it the way the sign fallback
         * always has. Both ratios are 0 there, so the choice cannot be observed. */
        float sgn_want  = (want > 0.0f) ? 1.0f : -1.0f;
        float sgn_denom = (denom != 0.0f) ? ((denom > 0.0f) ? 1.0f : -1.0f) : sgn_want;

        /* floor is strictly positive (ff_authority_floor guarantees it) -> never divide by 0 */
        float raw_meas   = ctrl_clampf(want / (sgn_denom * r.authority_floor_k) * 100.0f,
                                       0.0f, 100.0f);
        float raw_demand = ctrl_clampf(want / (sgn_want  * r.authority_floor_k) * 100.0f,
                                       0.0f, 100.0f);
        float w = ctrl_clampf(fabsf(denom) / r.authority_floor_k, 0.0f, 1.0f);

        /* Written as an offset from raw_demand, not as w*a + (1-w)*b, so that the
         * useful-side case (raw_meas == raw_demand bit for bit) returns that value exactly
         * rather than within a rounding of it. */
        raw = raw_demand + w * (raw_meas - raw_demand);
    }

    /* Filter the OUTPUT, not the inputs. The mixing law divides by (t_src - t_ret), so
     * smoothing the inputs and then dividing does not land on the same place as dividing
     * and then smoothing -- and with the denominator swinging several K that gap is not
     * second-order. */
    if (c->out_tau_s > 0.0f && s->has_out && dt_s > 0.0f){
        float alpha = dt_s / (c->out_tau_s + dt_s);
        s->out += alpha * (raw - s->out);
    } else {
        s->out = raw;
    }
    s->has_out = true;

    r.pos_ff = s->out;
    return r;
}
