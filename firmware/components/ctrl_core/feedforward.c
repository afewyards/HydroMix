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

    /* LIMIT the denominator; never abandon the division.
     *
     * Up to 1.5.0, low authority froze the FF at its last valid output and control.c handed
     * that frozen constant straight to the valve (pi.c discards the entire PI output while
     * frozen, both P and I). That is open loop, and it latched: a pinned valve starves the
     * source branch, a starved branch stops being cooled, t_src drifts toward t_ret, |denom|
     * shrinks further, and the freeze can never end. Live on 2026-08-04 that ran for 92
     * minutes, with |denom| down to 0.07 K and supply 2.7 K above setpoint.
     *
     * Clamping the denominator instead keeps the output bounded, continuous and correctly
     * signed -- and because the measured coupling is negative, it keeps pointing the valve
     * the way that RESTORES authority: a converging source in cooling yields a large ratio,
     * so the FF opens toward the source, which cools it, which reopens the gap. The
     * saturation IS the correct response here, not an error to be suppressed.
     *
     * Sign comes from the live denominator, falling back to the demand direction when it is
     * exactly zero (a useful source sits on the same side of t_ret as the setpoint). If the
     * source is on the WRONG side, the ratio goes negative and clamps to 0 -- also correct:
     * a source that cannot help must not be mixed in. */
    float denom_eff = denom;
    if (r.frozen){
        float sgn = (denom != 0.0f) ? ((denom > 0.0f) ? 1.0f : -1.0f)
                                    : ((want  > 0.0f) ? 1.0f : -1.0f);
        denom_eff = sgn * r.authority_floor_k;   /* floor is strictly positive -> never 0 */
    }

    float raw = ctrl_clampf(want / denom_eff * 100.0f, 0.0f, 100.0f);

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
