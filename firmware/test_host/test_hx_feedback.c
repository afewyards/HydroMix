#include "unity.h"
#include "ctrl_core/control.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* Closed-loop regression for the source<->valve coupling through the heat exchanger, with
 * a SWINGING primary -- disturbance rejection, where test_ff_recovery covers the starved
 * cold start.
 *
 * HISTORY, because this file previously argued the opposite and its gate hid a real bug.
 *
 * Up to 1.5.0 this test asserted `new.travel < 0.50 * old.travel` as "the point of the
 * change", on a plant whose coupling was +0.0654 K/% (opening the valve WARMS the source).
 * Both premises were wrong:
 *
 *  1. THE SIGN. +0.0654 came from a 20 h closed-loop fit at r = +0.49, taken while the HX
 *     primary itself drifted 14.25 -> 16.93 C -- the controller opens the valve because the
 *     primary warmed, so valve % and t_source both track hx_a and correlate positively with
 *     no causal content. The open-loop measurement (2026-08-04, valve held manually at full
 *     source, primary flat to 0.19 K) gives -0.041 K/%: more source flow COOLS the source,
 *     because the starved branch stops being cooled at all.
 *
 *  2. THE GATE. "Valve travel must more than halve" is trivially won by a valve that has
 *     stopped moving. Re-running the 1.5.0 configuration on this file's own plant: travel
 *     0.0 %, position p-p 0.0 %, FF frozen 100 % of the sim, pinned at 48.3 %, supply 1.0 K
 *     warm -- and it PASSED. Its supply-tracking guards only passed because the baseline it
 *     compared against was itself railed at 98.3 % with +2.65 K error. "Stopped working" and
 *     "moves less" are the same measurement, so travel is now bounded on BOTH sides and
 *     tracking error is capped absolutely rather than relative to a broken baseline. */

#define T_RET        21.2f    /* live return: sd 0.19 K over 869 samples, effectively fixed */
#define HX_PRIMARY   15.6f    /* mean of the live 14.43..16.62 primary band */
#define PRIM_AMP     1.34f    /* 2.68 K p2p, the real primary swing */
#define PRIM_PERIOD  3000.0f  /* s (50 min) — the dominant period in the live data */
#define T_SET        18.5f    /* the live cooling setpoint */

/* HX effectiveness at full secondary flow. Calibrated to the measured pair
 * {valve 100 % -> t_src 18.12 C} against a 15.6 C primary and a 21.2 C return, i.e. 0.55.
 * Swept below because it is one operating point, not a characterised curve. */
static float g_eff_max = 0.55f;

#define TRAVEL_S     120.0f
#define VALVE_DB     1.5f     /* live valve_deadband_pct */
#define SRC_TAU_S    120.0f   /* HX approach doesn't shift instantly with flow */
#define THETA_S      30       /* transport dead time, valve -> supply probe */
#define TAU_S        45.0f    /* probe lag */
#define SIM_S        14400    /* 4 h */
#define WARMUP_S     7200     /* judge the last 2 h */
#define INIT_POS     50.0f

typedef struct {
    float pos, src, probe;
    float delay[THETA_S];
    int   di;
    double travel;
} plant_t;

/* Secondary flow tracks valve position and HX effectiveness rises with flow, so the source
 * is cooled toward the primary in proportion to how far the valve is open. Squared rather
 * than linear because the observed starvation at 55.7 % was far worse than linear. */
static float src_ss_of(float primary, float pos){
    float f = pos / 100.0f;
    return T_RET - g_eff_max * f * f * (T_RET - primary);
}

static float primary_at(int t){
    return HX_PRIMARY + PRIM_AMP * sinf(6.2831853f * (float)t / PRIM_PERIOD);
}

static void plant_init(plant_t *p){
    memset(p, 0, sizeof(*p));
    p->pos = INIT_POS;
    p->src = src_ss_of(HX_PRIMARY, p->pos);
    float t0 = T_RET + p->pos / 100.0f * (p->src - T_RET);
    for (int i = 0; i < THETA_S; i++) p->delay[i] = t0;
    p->probe = t0;
}

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

    p->src += (src_ss_of(primary_at(t), p->pos) - p->src) * (1.0f / SRC_TAU_S);

    float t_mix = T_RET + p->pos / 100.0f * (p->src - T_RET);
    float delayed = p->delay[p->di];
    p->delay[p->di] = t_mix;
    p->di = (p->di + 1) % THETA_S;
    p->probe += (delayed - p->probe) * (1.0f / TAU_S);
}

typedef struct { float travel, sup_pp, sup_mean_err, src_pp, pos_min, pos_max; } result_t;

static result_t run_sim(void){
    control_state_t st; control_init(&st);
    control_cfg_t cfg = {
        .heat_setpoint = 35.0f, .cool_setpoint = T_SET, .park_pos = 20.0f,
        .mode_cfg = { 30.0f, 20.0f, 2.0f, 60000, 420000 },
        .pi_cfg = { 2.8f, 0.3f, 0.0f, 100.0f, 0.1f, 0.0f },   /* live kp/ki/deadband */
        .gov_cfg = { 36.0f, 16.0f, 35.0f, 17.0f },
        .alarm_dwell_ms = 300000,
        .deadtime_s = 35.0f,                                  /* live value */
    };
    ff_cfg_defaults(&cfg.ff_cfg);

    plant_t p; plant_init(&p);
    control_in_t in = {0};
    in.water_running = true; in.link_up = true;
    in.t_return_f = T_RET; in.hx_a = 15.6f;                   /* < cool_threshold: cooling */

    float target = INIT_POS;
    float smin = 1e9f, smax = -1e9f, srcmin = 1e9f, srcmax = -1e9f;
    float pmin = 1e9f, pmax = -1e9f;
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
            if (p.pos   < pmin) pmin = p.pos;
            if (p.pos   > pmax) pmax = p.pos;
            sum += p.probe; n++;
        }
    }
    result_t r;
    r.travel       = (float)(p.travel - travel0);
    r.sup_pp       = smax - smin;
    r.sup_mean_err = (float)(sum / (double)n) - T_SET;
    r.src_pp       = srcmax - srcmin;
    r.pos_min = pmin; r.pos_max = pmax;
    return r;
}

void setUp(void){} void tearDown(void){}

void test_hx_coupling_tracks_without_thrash(void){
    static const float effs[] = { 0.45f, 0.55f, 0.70f };

    for (size_t i = 0; i < sizeof effs / sizeof effs[0]; i++){
        g_eff_max = effs[i];
        result_t r = run_sim();

        /* Coldest supply this plant can physically make, at full source against the mean
         * primary. At low effectiveness the setpoint is simply out of reach, which is a
         * plant-capacity fact and must not be charged to the controller. */
        float floor_sup = T_RET - g_eff_max * (T_RET - HX_PRIMARY);
        float unreachable = (floor_sup > T_SET) ? (floor_sup - T_SET) : 0.0f;

        printf("hxfb[eff=%.2f] travel %7.1f %%  pos %.1f..%.1f  sup p-p %.2f K  "
               "mean err %+.2f K (floor %+.2f)  src p-p %.2f K\n",
               effs[i], r.travel, r.pos_min, r.pos_max, r.sup_pp,
               r.sup_mean_err, unreachable, r.src_pp);

        /* NOT LATCHED. The gate that 1.5.0 inverted: a valve that stops moving is a
         * failure, not a win. Over 2 h with a swinging primary it must be working. */
        TEST_ASSERT_TRUE(r.travel > 20.0f);

        /* NOT THRASHING. 120 s full travel, so 2 h of sane regulation is well under this;
         * the 1.1.0 limit cycle swept ~96 % every ~260 s, which would be ~2700 %. */
        TEST_ASSERT_TRUE(r.travel < 1500.0f);

        /* TRACKING, in absolute terms rather than against a broken baseline. */
        TEST_ASSERT_TRUE(fabsf(r.sup_mean_err) < 0.6f + unreachable);
        TEST_ASSERT_TRUE(r.sup_pp < 1.5f);
    }
    g_eff_max = 0.55f;
}

int main(void){
    UNITY_BEGIN();
    RUN_TEST(test_hx_coupling_tracks_without_thrash);
    return UNITY_END();
}
