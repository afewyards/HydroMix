#pragma once
#include "ctrl_core/types.h"

#define FF_DEFAULT_PCT              50.0f
#define FF_NO_AUTHORITY_PARK_DWELL_MS 600000u

/* Bounds on the authority floor (see ff_authority_floor). FF_AUTHORITY_MIN_K is the
 * fixed floor everything before 1.5.0 used and stays the lower bound, so the derived
 * value can only ever be more conservative. FF_AUTHORITY_MAX_K caps it because the
 * freeze is not free: past ~4 K the FF spends long enough frozen that the 10 min park
 * dwell starts to dominate. Measured against 20 h of GF-HydroMix cooling telemetry
 * (2026-08-03), as a fraction of flowing time spent frozen:
 *   2.0 K -> 3.0 %   2.8 K -> 3.6 %   3.8 K -> 6.4 %   4.6 K -> 17.7 % (20 min runs)
 *
 * The dwell is now a long backstop for a genuinely stuck plant, not a routine trigger:
 * a freeze on its own is stable (the FF holds last_valid) and no longer escalates to an
 * open-loop park -- control.c holds the current valve position instead. */
#define FF_AUTHORITY_MIN_K          2.0f
#define FF_AUTHORITY_MAX_K          4.0f

/* d(t_source)/d(valve %) through the heat exchanger, K per % of travel.
 *
 * Opening the mixing valve raises secondary flow, which collapses the HX approach and
 * WARMS the source temperature -- the same quantity the FF divides by. So the FF moves
 * its own denominator, and the mixing law is not the open-loop map it assumes.
 * Regressed over 20 h of live cooling (2026-08-03): +0.0654 K/%, r = +0.49, with the
 * approach (t_source - t_hx_a) sweeping -0.57..+6.25 K while the primary side stayed
 * within 14.25..16.93 C. Rounded down to 0.065 -- this is a plant constant, not a
 * universal one. Set to 0 to disable the derived floor (pre-1.5.0 behaviour). */
#define FF_COUPLING_PCT_K           0.065f

/* Time constant of the EMA on the FF OUTPUT, s.
 *
 * Secondary to the authority floor: the floor is what keeps the FF out of the region
 * where it fights itself, this only stops the motor chasing the ripple that remains.
 * 900 s passes ~0.27 of a 26-minute limit cycle and ~0.47 of a 50-minute one, at a lag
 * the PI's +/-20 % trim absorbs easily (the genuine, primary-driven part of the source
 * swing is 2.68 K p2p, worth well under 10 % of travel). 0 disables filtering. */
#define FF_OUT_TAU_S                900.0f

/* Runtime-settable so the host tests can drive old and new behaviour through the same
 * code path, and so promoting any of these to a Zigbee tunable later is a small diff.
 * Not currently exposed over Zigbee -- control_task.c fills it from ff_cfg_defaults(). */
typedef struct {
    float coupling_pct_k;          /* 0 -> derived floor off, floor = auth_min_k */
    float auth_min_k, auth_max_k;
    float out_tau_s;               /* 0 -> output filter off */
} ff_cfg_t;

typedef struct {
    float    last_valid;
    bool     has_valid;
    bool     freezing;
    uint32_t frozen_since_ms;
    float    out;                  /* EMA of pos_ff */
    bool     has_out;
    uint32_t last_ms;
    bool     has_last_ms;
} ff_state_t;

typedef struct {
    float pos_ff;
    bool  frozen;
    bool  park_requested;
    float authority_floor_k;       /* floor actually applied this step (telemetry/tests) */
} ff_result_t;

void  ff_cfg_defaults(ff_cfg_t *c);
void  ff_init(ff_state_t *s);
void  ff_reseed(ff_state_t *s);        /* drop the output filter, keep last_valid */
void  ff_mode_change(ff_state_t *s);   /* heating<->cooling inverts pos_ff: drop both */

float ff_authority_floor(const ff_cfg_t *c, float t_set, float t_return_f);
ff_result_t ff_step(ff_state_t *s, const ff_cfg_t *c, float t_set, float t_return_f,
                    float t_source_f, uint32_t now_ms);
