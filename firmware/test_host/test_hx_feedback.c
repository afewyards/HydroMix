#include "unity.h"
#include "ctrl_core/control.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* Closed-loop regression for the HX approach coupling -- the mechanism behind the live
 * valve hunting on GF-HydroMix at a 19 C cooling setpoint (diagnosed 2026-08-03).
 *
 * The mixing law the FF inverts treats t_source as an independent input. On a plate HX
 * fed by a fixed primary it is not: opening the mixing valve raises SECONDARY flow,
 * which collapses the HX approach, which WARMS t_source -- the very quantity the FF
 * divides by. So the FF moves its own denominator, and the closer the setpoint sits to
 * the return temperature the harder that bites, because pos_ff's numerator
 * (t_set - t_ret) shrinks while the denominator swings just as much.
 *
 * Measured over 20 h of live cooling telemetry, flowing only (hx_a < 17 C):
 *   d(t_source)/d(valve %)          = +0.0654 K/%   (r = +0.49)
 *   d(approach)/d(valve %)          = +0.036  K/%   (r = +0.40)
 *   HX primary (hx_a)               = 14.25..16.93 C, 2.68 K p2p  <- steady
 *   t_source                        = 7.81 K p2p                  <- 3x its own primary
 *   valve position                  = sd 10.6 %, 98.5 % p2p
 *
 * The plant below reproduces that: source temp is primary + coupling*position behind a
 * lag, supply is the mix behind dead time + probe lag, and the primary carries the
 * genuine slow swing the real one has. What the fix must do is cut VALVE TRAVEL without
 * giving up supply tracking -- the swing is the complaint, the temperature was already
 * being held (live supply sd was 0.37 K). */

#define T_RET        21.2f    /* floor return: live sd was 0.19 K, so effectively fixed */
#define HX_PRIMARY   15.6f    /* mean of the live 14.25..16.93 primary band */
#define PRIM_AMP     1.34f    /* 2.68 K p2p, the real primary swing */
#define PRIM_PERIOD  3000.0f  /* s (50 min) — the dominant period in the live data */
#define T_SET        19.0f    /* the setpoint that misbehaves */

/* K per valve %, regressed from live telemetry at +0.0654 — but only r = +0.49, and it
 * is a closed-loop estimate, so the gates below sweep it rather than trusting one value. */
static float g_coupling = 0.0654f;
#define COUPLING     g_coupling

#define TRAVEL_S     120.0f
#define VALVE_DB     2.0f     /* valve_deadband_pct as configured on the live device */
#define SRC_TAU_S    120.0f   /* HX approach doesn't shift instantly with flow */
#define THETA_S      30       /* transport dead time, valve -> supply probe */
#define TAU_S        45.0f    /* probe lag */
#define SIM_S        14400    /* 4 h */
#define WARMUP_S     1800     /* ignore the first 30 min */
#define INIT_POS     35.0f    /* live mean valve position */

typedef struct {
    float pos, src, probe;
    float delay[THETA_S];
    int   di;
    double travel;            /* cumulative |dpos|, % of travel */
} plant_t;

static void plant_init(plant_t *p){
    memset(p, 0, sizeof(*p));
    p->pos = INIT_POS;
    p->src = HX_PRIMARY + COUPLING * p->pos;
    float t0 = T_RET + p->pos / 100.0f * (p->src - T_RET);
    for (int i = 0; i < THETA_S; i++) p->delay[i] = t0;
    p->probe = t0;
}

/* One 1 s tick. The valve slews at the real travel rate and ignores commands inside the
 * motor deadband; the source then chases primary + coupling*position through a lag, and
 * the mix rides dead time + probe lag. */
static void plant_tick(plant_t *p, float target, int t){
    float err = target - p->pos;
    if (fabsf(err) > VALVE_DB){
        float step = 100.0f / TRAVEL_S;
        float d = (err > 0.0f) ? step : -step;
        if (fabsf(d) > fabsf(err)) d = err;
        p->pos += d;
        p->travel += fabsf(d);
    }
    p->pos = ctrl_clampf(p->pos, 0.0f, 100.0f);

    float primary = HX_PRIMARY + PRIM_AMP * sinf(6.2831853f * (float)t / PRIM_PERIOD);
    float src_ss  = primary + COUPLING * p->pos;
    p->src += (src_ss - p->src) * (1.0f / SRC_TAU_S);

    float t_mix = T_RET + p->pos / 100.0f * (p->src - T_RET);
    float delayed = p->delay[p->di];
    p->delay[p->di] = t_mix;
    p->di = (p->di + 1) % THETA_S;
    p->probe += (delayed - p->probe) * (1.0f / TAU_S);
}

typedef struct { float travel, sup_pp, sup_mean_err, src_pp; } result_t;

static result_t run_sim(const ff_cfg_t *ff){
    control_state_t st; control_init(&st);
    control_cfg_t cfg = {
        .heat_setpoint = 35.0f, .cool_setpoint = T_SET, .park_pos = 20.0f,
        .mode_cfg = { 30.0f, 20.0f, 2.0f, 60000, 420000 },
        .pi_cfg = { 2.8f, 0.5f, 0.0f, 100.0f, 0.25f, 0.0f },  /* live kp/ki/deadband */
        .ff_cfg = *ff,
        .gov_cfg = { 36.0f, 16.0f, 35.0f, 17.0f },
        .alarm_dwell_ms = 300000,
        .deadtime_s = 20.0f,                                   /* live value */
    };
    plant_t p; plant_init(&p);
    control_in_t in = {0};
    in.water_running = true; in.link_up = true;
    in.t_return_f = T_RET; in.hx_a = 15.0f;                    /* < cool_threshold: cooling */

    float target = INIT_POS;
    float smin = 1e9f, smax = -1e9f, srcmin = 1e9f, srcmax = -1e9f;
    double sum = 0.0; long n = 0;
    double travel0 = 0.0;

    for (int t = 0; t < SIM_S; t++){
        plant_tick(&p, target, t);
        if (t % 10 == 0){
            in.t_supply   = 0.0625f * roundf(p.probe / 0.0625f);  /* 12-bit DS18B20 */
            in.t_source_f = p.src;
            in.valve_pos  = p.pos;
            in.link_last_seen_ms = (uint32_t)t * 1000u;
            control_out_t o = control_step(&st, &in, &cfg, (uint32_t)t * 1000u);
            if (o.regulating) target = o.valve_target;
        }
        if (t == WARMUP_S) travel0 = p.travel;
        if (t >= WARMUP_S){
            if (p.probe < smin) smin = p.probe;
            if (p.probe > smax) smax = p.probe;
            if (p.src   < srcmin) srcmin = p.src;
            if (p.src   > srcmax) srcmax = p.src;
            sum += p.probe; n++;
        }
    }
    result_t r;
    r.travel       = (float)(p.travel - travel0);
    r.sup_pp       = smax - smin;
    r.sup_mean_err = (float)(sum / (double)n) - T_SET;
    r.src_pp       = srcmax - srcmin;
    return r;
}

void setUp(void){} void tearDown(void){}

/* The complaint is valve/source hunting, not supply error — the live loop was already
 * holding supply to sd 0.37 K. So the gates are: travel and source swing must drop
 * materially, and supply tracking must not be traded away to get it. Swept across a
 * coupling range spanning the measurement uncertainty (0.04 is a well-sized HX that
 * barely droops, 0.09 an undersized one that droops harder than the live regression). */
void test_hx_coupling_cuts_valve_travel(void){
    static const float couplings[] = { 0.04f, 0.0654f, 0.09f };
    ff_cfg_t legacy = { .coupling_pct_k = 0.0f, .auth_min_k = 2.0f,
                        .auth_max_k = 2.0f, .out_tau_s = 0.0f };
    ff_cfg_t shipped; ff_cfg_defaults(&shipped);

    for (size_t i = 0; i < sizeof couplings / sizeof couplings[0]; i++){
        g_coupling = couplings[i];
        result_t old = run_sim(&legacy);
        result_t neu = run_sim(&shipped);

        printf("hxfb[coupling=%.4f] legacy: travel %7.1f %%  sup p-p %.2f K  mean err %+.2f K  src p-p %.2f K\n",
               couplings[i], old.travel, old.sup_pp, old.sup_mean_err, old.src_pp);
        printf("hxfb[coupling=%.4f] new   : travel %7.1f %%  sup p-p %.2f K  mean err %+.2f K  src p-p %.2f K\n",
               couplings[i], neu.travel, neu.sup_pp, neu.sup_mean_err, neu.src_pp);
        printf("hxfb[coupling=%.4f] ratio : travel %.2fx  src p-p %.2fx\n",
               couplings[i], neu.travel / old.travel, neu.src_pp / old.src_pp);

        TEST_ASSERT_TRUE(neu.travel < 0.50f * old.travel);   /* the point of the change */
        TEST_ASSERT_TRUE(neu.src_pp < 0.70f * old.src_pp);   /* less flow modulation */
        TEST_ASSERT_TRUE(neu.sup_pp <= old.sup_pp + 0.15f);  /* not bought with ripple */
        /* Mean offset must not meaningfully degrade. 0.20 K rather than ~0, for the same
         * reason lagsim's gate 5 sits at 0.6 K: the 0.25 K PI deadband makes any offset
         * smaller than itself structurally uncorrectable, so sub-deadband drift either
         * way is noise. Observed worst case across this sweep is +0.17 K (at the mildest
         * coupling, where the loop barely struggles); the other two corners IMPROVE by
         * 0.24 and 0.41 K, because less valve thrash means less average flow modulation
         * and so less HX droop to fight. */
        TEST_ASSERT_TRUE(fabsf(neu.sup_mean_err) <= fabsf(old.sup_mean_err) + 0.20f);
    }
    g_coupling = 0.0654f;
}

int main(void){
    UNITY_BEGIN();
    RUN_TEST(test_hx_coupling_cuts_valve_travel);
    return UNITY_END();
}
