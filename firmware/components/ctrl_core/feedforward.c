#include "ctrl_core/feedforward.h"

void ff_cfg_defaults(ff_cfg_t *c){
    c->coupling_pct_k = FF_COUPLING_PCT_K;
    c->auth_min_k     = FF_AUTHORITY_MIN_K;
    c->auth_max_k     = FF_AUTHORITY_MAX_K;
    c->out_tau_s      = FF_OUT_TAU_S;
}

void ff_init(ff_state_t *s){
    s->last_valid = FF_DEFAULT_PCT; s->has_valid = false;
    s->freezing = false; s->frozen_since_ms = 0;
    s->out = FF_DEFAULT_PCT; s->has_out = false;
    s->last_ms = 0; s->has_last_ms = false;
}

void ff_reseed(ff_state_t *s){
    s->has_out = false;
}

void ff_mode_change(ff_state_t *s){
    s->has_out = false; s->has_valid = false;
    s->freezing = false; s->frozen_since_ms = 0;
}

/* Smallest |t_src - t_ret| the FF is allowed to divide by.
 *
 * The mixing law assumes t_src is an input. It isn't: the valve the FF commands sets
 * secondary flow, flow sets the HX approach, and the approach sets t_src
 * (coupling_pct_k). Differentiating pos_ff with respect to t_src and chaining through
 * that coupling gives the gain around the source -> valve -> source path:
 *
 *     G = |t_ret - t_set| * 100 * coupling_pct_k / denom^2
 *
 * so G reaches 1 at denom = sqrt(|t_ret - t_set| * 100 * coupling_pct_k). Below that the
 * FF reinforces its own error and walks the valve toward a rail; holding last-valid is
 * the only stable option. Because the numerator carries the setpoint, the floor has to
 * track it -- on GF-HydroMix (t_ret ~21.2) that is 2.80 K at a 20 C setpoint but 3.79 K
 * at 19 C, which is exactly the direction a fixed floor gets wrong. */
float ff_authority_floor(const ff_cfg_t *c, float t_set, float t_ret){
    /* A zeroed ff_cfg_t (a caller that brace-initialised control_cfg_t without filling
     * this in) must degrade to the pre-1.5.0 fixed floor, NOT to a floor of zero — which
     * would disable the low-authority freeze altogether and let the FF divide by an
     * arbitrarily small denominator. Fail towards the old, safe behaviour. */
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

    float denom = t_src - t_ret;
    if (fabsf(denom) < r.authority_floor_k){
        if (!s->freezing){ s->freezing = true; s->frozen_since_ms = now_ms; }
        r.pos_ff = s->has_valid ? s->last_valid : FF_DEFAULT_PCT;
        r.frozen = true;
        r.park_requested = (now_ms - s->frozen_since_ms) >= FF_NO_AUTHORITY_PARK_DWELL_MS;
        /* Deliberately leaves the EMA untouched: on recovery it resumes from where it
         * was, and the dt spanning the freeze makes alpha large enough to catch up. */
        return r;
    }

    float raw = ctrl_clampf((t_set - t_ret) / denom * 100.0f, 0.0f, 100.0f);

    /* Filter the OUTPUT, not the inputs. The mixing law divides by (t_src - t_ret), so
     * smoothing the inputs and then dividing does not land on the same place as dividing
     * and then smoothing -- and with the denominator swinging several K that gap is not
     * second-order. Filtering here also keeps the freeze test above on the live reading,
     * so low authority is still detected the instant it happens. */
    if (c->out_tau_s > 0.0f && s->has_out && dt_s > 0.0f){
        float alpha = dt_s / (c->out_tau_s + dt_s);
        s->out += alpha * (raw - s->out);
    } else {
        s->out = raw;
    }
    s->has_out = true;

    r.pos_ff = s->out;
    r.frozen = false;
    r.park_requested = false;
    s->last_valid = s->out; s->has_valid = true;
    s->freezing = false;
    return r;
}
