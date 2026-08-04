#include "unity.h"
#include "ctrl_core/control.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* Closed-loop regression: FOPDT plant (dead time + probe lag) + a real 3-point relay
 * actuator + FF model error + probe quantization + a large-signal start, driven by
 * the real controller. Gates the current shipped defaults (kp=2.8, ki=0.9, 0.25K
 * deadband, 30s transit hold) settle where 1.0.x tuning (kp=4, ki=3, no deadband, no
 * hold) sustains a relay limit cycle — across nominal dead-time/lag plus a 4-corner
 * robustness sweep. (1.4.0 considered lowering deadband_k to 0.15 alongside the new
 * valve_deadband_pct tunable; a deadband_k sweep of {0.15, 0.20, 0.22, 0.25} against
 * this exact matrix found only 0.25 clears every corner/gate — the theta=40/tau=60
 * corner in particular is non-monotonic across that range — so 0.25 ships unchanged.)
 *
 * Mechanism (found by exhaustive search, docs/superpowers/plans/2026-07-30 Task 7):
 *   - A REAL 3-point relay actuator: once driving, must commit MIN_RUN_TICKS s before
 *     it may stop/reverse; a direction reversal forces a LOCKOUT_TICKS s dead stop
 *     first (contactor protection). This actuator quantization/dead-time-blind windup
 *     is what drives the bench limit cycle — old tuning has no awareness of it, new
 *     tuning's deadband+transit-hold absorb it.
 *   - Valve starts far from equilibrium (INIT_POS) for a large-signal transient.
 *   - Smoothstep (not linear) valve curve: the FF assumes a linear mixing law, so this
 *     is FF model error the bench also has (valve curve, probe bias).
 *   - Supply reading quantized to 12-bit DS18B20 resolution before reaching the
 *     controller.
 *   - Measurement window: ignore SETTLE_GUARD_S at the start, then open once the probe
 *     first comes within 1.0 K of setpoint; measure peak-to-peak and mean to the end
 *     of the sim. Same rule for both configs.
 *   - Gate 5 (mean-offset) threshold is 0.6 K, not the ~0 K a perfect loop would give:
 *     the deliberately coarse actuator (8 s min-run =~ 6.7 % travel =~ 1.2 K supply
 *     quantum) imposes a genuine ~half-quantum steady-state mean floor at the fastest
 *     (THETA_S=20, TAU_S=30) corner — the expected plant after the probe paste +
 *     insulation hardware pass, so it stays in the robustness matrix. 0.6 K still
 *     discriminates a real wrong-temperature-convergence defect (which manifests at
 *     >=1 K) from this floor.
 */

#define MAX_THETA_S     40
#define T_SRC     45.0f
#define T_RET     27.0f
#define T_SET     35.0f
#define TRAVEL_S  120.0f

#define MIN_RUN_TICKS   8     /* min pulse length, s, before actuator may stop/reverse */
#define LOCKOUT_TICKS   3     /* forced-stop dwell after a direction reversal, s */
#define INIT_POS        4.0f  /* % initial valve position (large-signal transient) */
#define SETTLE_GUARD_S  300   /* s, ignore this much at start before opening the window */

typedef struct {
    float pos, probe;
    float delay[MAX_THETA_S];
    int di;
    int theta;
    int8_t dir;              /* -1, 0, +1 : current actual driven direction */
    int phase_ticks;         /* ticks spent in current dir/stop phase */
    int lockout_ticks_left;  /* >0 => forced stop (post-reversal dead time) */
} plant_t;

static void plant_init(plant_t *p, int theta_s){
    memset(p, 0, sizeof(*p));
    p->theta = theta_s;
    p->pos = INIT_POS;
    float t0 = T_RET + p->pos / 100.0f * (T_SRC - T_RET);
    for (int i = 0; i < theta_s; i++) p->delay[i] = t0;
    p->probe = t0;
}

/* One 1 s tick. Actuator is modelled as a real 3-point relay: once driving, it must
 * commit to MIN_RUN_TICKS before it may stop or reverse; a direction reversal is only
 * permitted after a forced-stop dead time of LOCKOUT_TICKS (contactor protection).
 * The mixed temp then rides a smoothstep valve nonlinearity, the dead-time ring
 * buffer, and a first-order probe lag. */
static void plant_tick(plant_t *p, float target, float tau_s){
    float err = target - p->pos;
    int desired = 0;
    if (err >  2.0f) desired =  1;
    else if (err < -2.0f) desired = -1;

    int actual;
    if (p->lockout_ticks_left > 0){
        p->lockout_ticks_left--;
        actual = 0;                                    /* forced stop, ignore desired */
    } else if (p->dir != 0 && p->phase_ticks < MIN_RUN_TICKS){
        actual = p->dir;                                /* committed to current pulse */
    } else if (desired == p->dir){
        actual = p->dir;                                /* no change requested */
    } else if (p->dir != 0 && desired != 0 && desired == -p->dir){
        actual = 0;                                     /* reversal -> forced stop first */
        p->lockout_ticks_left = LOCKOUT_TICKS;
    } else {
        actual = desired;                               /* simple start or simple stop */
    }

    if (actual != p->dir){ p->phase_ticks = 0; p->dir = actual; }
    else if (p->dir != 0) p->phase_ticks++;

    if (p->dir != 0) p->pos += p->dir * (100.0f / TRAVEL_S);
    p->pos = ctrl_clampf(p->pos, 0.0f, 100.0f);

    float f = p->pos / 100.0f;
    /* Real 3-way valves aren't linear; the FF assumes they are. */
    float f_real = f * f * (3.0f - 2.0f * f);
    float t_mix = T_RET + f_real * (T_SRC - T_RET);

    float delayed = p->delay[p->di];
    p->delay[p->di] = t_mix;
    p->di = (p->di + 1) % p->theta;
    p->probe += (delayed - p->probe) * (1.0f / tau_s);
}

/* Sim run for a given (theta_s, tau_s) corner. 60 min sim, control every 10 s.
 * Measurement window: ignore SETTLE_GUARD_S at the start, then find the first tick
 * where the probe is within 1.0 K of setpoint; measure peak-to-peak (and mean, for
 * the mean-offset gate) from there to the end of the sim. Identical rule for both
 * configs. mean_out (may be NULL) receives the window-mean probe temp. */
static float run_sim(const pi_cfg_t *pi, float deadtime_s, int theta_s, float tau_s,
                      float *mean_out){
    control_state_t st; control_init(&st);
    control_cfg_t cfg = {
        .heat_setpoint = T_SET, .cool_setpoint = 18.0f, .park_pos = 50.0f,
        .mode_cfg = { 28.0f, 16.0f, 2.0f, 60000, 420000 },
        .pi_cfg = *pi,
        .gov_cfg = { 36.0f, 16.0f, 35.0f, 17.0f },
        .alarm_dwell_ms = 300000,
        .deadtime_s = deadtime_s,
    };
    /* Source/return/setpoint are all constant here, so the output EMA reseeds on the
     * first step and then holds — this corner is unaffected by the 1.5.0 FF changes,
     * which is exactly what these gates should keep proving. */
    ff_cfg_defaults(&cfg.ff_cfg);
    plant_t p; plant_init(&p, theta_s);
    control_in_t in = {0};
    in.water_running = true; in.link_up = true;
    in.t_source_f = T_SRC; in.t_return_f = T_RET; in.hx_a = 40.0f;
    float target = 50.0f, tmin = 1000.0f, tmax = -1000.0f;
    double sum = 0.0; long cnt = 0;
    int window_open = 0;
    for (int t = 0; t < 3600; t++){
        plant_tick(&p, target, tau_s);
        if (t % 10 == 0){
            in.t_supply = 0.0625f * roundf(p.probe / 0.0625f);   /* 12-bit DS18B20 */
            in.valve_pos = p.pos;
            control_out_t o = control_step(&st, &in, &cfg, (uint32_t)t * 1000u);
            if (o.regulating) target = o.valve_target;
        }
        if (!window_open && t >= SETTLE_GUARD_S && fabsf(p.probe - T_SET) <= 1.0f)
            window_open = 1;
        if (window_open){
            if (p.probe < tmin) tmin = p.probe;
            if (p.probe > tmax) tmax = p.probe;
            sum += p.probe; cnt++;
        }
    }
    if (!window_open){ tmin = 0.0f; tmax = 0.0f; }
    if (mean_out) *mean_out = (cnt > 0) ? (float)(sum / cnt) : 0.0f;
    return tmax - tmin;
}

void setUp(void){} void tearDown(void){}

/* Nominal dead-time/lag plus a 4-corner robustness sweep (THETA_S in {20,40},
 * TAU_S in {30,60}), each asserting all four gates. */
void test_lag_regression_split(void){
    pi_cfg_t oldcfg = { .kp = 4.0f, .ki = 3.0f, .out_min = 0, .out_max = 100, .deadband_k = 0 };
    pi_cfg_t newcfg = { .kp = 2.8f, .ki = 0.9f, .out_min = 0, .out_max = 100, .deadband_k = 0.25f };
    static const int   thetas[] = { 30, 20, 20, 40, 40 };
    static const float taus[]   = { 45.0f, 30.0f, 60.0f, 30.0f, 60.0f };

    for (int i = 0; i < 5; i++){
        float new_mean;
        float old_pp = run_sim(&oldcfg, 0.0f,  thetas[i], taus[i], NULL);
        float new_pp = run_sim(&newcfg, 30.0f, thetas[i], taus[i], &new_mean);
        printf("lagsim[theta=%d tau=%.0f]: old p-p %.2f K, new p-p %.2f K, new mean %.2f K\n",
               thetas[i], taus[i], old_pp, new_pp, new_mean);
        TEST_ASSERT_TRUE(new_pp < 0.8f);              /* converges */
        TEST_ASSERT_TRUE(old_pp > 0.8f);              /* old behaviour limit-cycles */
        TEST_ASSERT_TRUE(old_pp > 2.0f * new_pp);     /* the fix is material */
        TEST_ASSERT_TRUE(fabsf(new_mean - T_SET) < 0.6f); /* converges to the right temp */
    }
}

int main(void){
    UNITY_BEGIN();
    RUN_TEST(test_lag_regression_split);
    return UNITY_END();
}
