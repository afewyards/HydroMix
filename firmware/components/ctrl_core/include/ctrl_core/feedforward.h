#pragma once
#include "ctrl_core/types.h"

#define FF_DEFAULT_PCT              50.0f

/* Bounds on the authority floor (see ff_authority_floor).
 *
 * Since 1.5.1 the floor no longer gates the controller off -- it LIMITS the denominator
 * (see ff_step). So its size is not a stability threshold any more, it is how
 * conservatively the FF is allowed to extrapolate when the source and return converge:
 * a bigger floor means a SMALLER commanded ratio, i.e. less aggressive.
 *
 * It is also the width of the band over which ff_step blends the measured sign of
 * (t_src - t_ret) against the demand direction, so it doubles as the |denom| at which that
 * sign is considered fully trustworthy. 2 K is comfortably above DS18B20 pair-to-pair
 * disagreement and comfortably below the plant's normal |denom| range (measured 0.07..4.75,
 * mean 1.89), which is what keeps the blend confined to the genuinely ambiguous readings. */
#define FF_AUTHORITY_MIN_K          2.0f
#define FF_AUTHORITY_MAX_K          4.0f

/* d(t_source)/d(valve %) through the heat exchanger, K per % of travel. Scales the
 * setpoint-derived floor; 0 disables the derivation so the floor is just auth_min_k.
 *
 * DISABLED (0) since 1.5.1 -- the 1.5.0 value of +0.0654 K/% HAD THE WRONG SIGN.
 *
 * It came from a 20 h closed-loop regression at only r = +0.49, taken while the HX
 * primary (hx_a) itself drifted 14.25 -> 16.93 C. The controller opens the valve BECAUSE
 * the primary warmed, so valve % and t_source both track hx_a and correlate positively
 * with no causal content -- a textbook confound.
 *
 * The open-loop measurement (2026-08-04, valve held manually at full source while the
 * primary stayed flat to within 0.19 K) gives the opposite sign: t_source fell 20.87 ->
 * 19.06 C over ~44 % of travel, about -0.041 K/%. MORE source flow COOLS the source,
 * because the starved branch stops being cooled at all -- the effect that dominates here
 * is flow starvation, not the textbook rise in HX approach with secondary flow.
 *
 * That inversion invalidates the derivation, not just its magnitude. ff_authority_floor
 * computes a loop-gain MAGNITUDE and treats |G| > 1 as runaway, but sign is what decides
 * whether feedback diverges. With the true negative coupling, opening the valve cools the
 * source, GROWS |denom| and backs the valve off: negative feedback, self-limiting. |G| > 1
 * under negative feedback is a fast bounded response, not instability. The floor was
 * guarding a runaway that cannot happen in this plant -- while manufacturing a real latch
 * (2026-08-04: valve pinned 92 min, 87 % of samples below the floor, supply 2.7 K warm).
 *
 * Do NOT re-enable this from another closed-loop fit. It needs an open-loop step test:
 * park the valve at 2-3 positions for >= 20 min each with the controller out of the loop.
 *
 * The measured SIGN is also scoped to a fixed-speed pump, where valve position is the only
 * thing modulating source-branch flow -- that is why starvation dominates and more source
 * flow cools the source. ff_step leans on this sign to pick which way to open when authority
 * runs out. A pump that modulates flow independently of the valve invalidates the premise,
 * so both the sign and this constant have to be re-measured (open loop, as above) before any
 * of it carries over. */
#define FF_COUPLING_PCT_K           0.0f

/* Time constant of the EMA on the FF OUTPUT, s. 0 disables filtering.
 *
 * 1.5.0 used 900 s, which is ~7.5x the 120 s valve travel time and ~26x the 35 s
 * deadtime -- far slower than the plant it was steering, and slower than the disturbances
 * it was meant to reject. It was introduced to cut valve travel, but the travel reduction
 * measured at the time came mostly from the FF freeze pinning the valve, not from the
 * filter (verified: at 180 s the 1.5.0 sim is still frozen 100 % of the time with zero
 * travel). 180 s is ~1.5 valve travels: enough to keep the motor off the sensor ripple,
 * short enough that the loop still answers a real disturbance inside one deadtime. */
#define FF_OUT_TAU_S                180.0f

/* Runtime-settable so the host tests can drive old and new behaviour through the same
 * code path, and so promoting any of these to a Zigbee tunable later is a small diff.
 * Not currently exposed over Zigbee -- control_task.c fills it from ff_cfg_defaults(). */
typedef struct {
    float coupling_pct_k;          /* 0 -> derived floor off, floor = auth_min_k */
    float auth_min_k, auth_max_k;
    float out_tau_s;               /* 0 -> output filter off */
} ff_cfg_t;

typedef struct {
    float    out;                  /* EMA of pos_ff */
    bool     has_out;
    uint32_t last_ms;
    bool     has_last_ms;
} ff_state_t;

typedef struct {
    float pos_ff;
    bool  frozen;                  /* authority below floor -> denominator LIMITED and the
                                    * answer blended toward the demand direction (ff_step) */
    float authority_floor_k;       /* floor actually applied this step (telemetry/tests) */
} ff_result_t;

void  ff_cfg_defaults(ff_cfg_t *c);
void  ff_init(ff_state_t *s);
void  ff_reseed(ff_state_t *s);        /* drop the output filter */
void  ff_mode_change(ff_state_t *s);   /* heating<->cooling inverts pos_ff: drop the filter */

float ff_authority_floor(const ff_cfg_t *c, float t_set, float t_return_f);
ff_result_t ff_step(ff_state_t *s, const ff_cfg_t *c, float t_set, float t_return_f,
                    float t_source_f, uint32_t now_ms);
