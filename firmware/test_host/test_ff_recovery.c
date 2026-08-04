#include "unity.h"
#include "ctrl_core/control.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

/* Closed-loop regression for the 2026-08-04 GF-HydroMix latch: the valve sat at 55.7 %
 * for 92 minutes while supply ran 2.6-2.75 K above a 18.5 C cooling setpoint, and the
 * house stopped being cooled.
 *
 * WHAT THE LIVE DATA SAID (869 Z2M samples, 09:00-10:33, all values below are measured):
 *   - cool_setpoint 18.5, t_return 20.87..21.37 (effectively fixed), hx_a 14.43..16.62
 *   - |t_src - t_ret| ran 0.07..4.75 K, mean 1.89 -> BELOW the 1.5.0 derived floor
 *     (sqrt(2.7*100*0.065) = 4.19, clamped to the 4.0 K ceiling) in 87 % of samples
 *   - valve_position moved on exactly 5 occasions in 92 min, ALL of them inside the one
 *     window where |denom| crossed 4.00 -- the loop was gated off the rest of the time
 *   - the source-vs-valve coupling is NEGATIVE, not the +0.0654 K/% that 1.5.0 assumed:
 *     over 10:12-10:26, with the valve manually at full source and the primary flat
 *     (hx_a -0.19 K), t_src fell 20.87 -> 19.06 = -1.81 K over ~44 % of travel,
 *     i.e. about -0.041 K/%. MORE source flow COOLS the source.
 *
 * The plant below is calibrated to those two endpoints. Secondary flow tracks valve
 * position, and HX effectiveness rises with flow, so the source is cooled toward the
 * primary in proportion to how far the valve is open:
 *
 *     t_src_ss = t_ret - eff_max * (pos/100)^2 * (t_ret - primary)
 *
 * which reproduces the measured pair {55.7 % -> 20.2 C starved, 100 % -> 18.1 C} and
 * gives d(t_src)/d(pos) < 0 everywhere -- the measured sign. Squared rather than linear
 * because the observed starvation at 55.7 % was much worse than a linear fall predicts.
 *
 * Under 1.5.0 this test FAILS: the FF freezes, its held baseline becomes the whole valve
 * command (pi.c discards the entire PI output while frozen), the valve never moves off
 * its start position, and supply parks ~2.3 K warm -- reproducing the incident. */

#define T_RET        21.2f
#define HX_PRIMARY   15.6f
#define EFF_MAX      0.55f    /* calibrated: pos=100 -> t_src 18.12 (measured 18.12) */
#define T_SET        18.5f    /* the live cooling setpoint */
#define START_POS    50.0f    /* FF_DEFAULT_PCT: the baseline a freeze holds with no valid */
#define TRAVEL_S     120.0f
#define VALVE_DB     1.5f     /* live valve_deadband_pct */
#define SRC_TAU_S    120.0f
#define THETA_S      30
#define TAU_S        45.0f
#define SIM_S        10800    /* 3 h */
#define SETTLE_S     7200     /* judge the last hour */

typedef struct { float pos, src, probe; float delay[THETA_S]; int di; double travel; } plant_t;

static float src_ss_of(float pos){
    float f = pos / 100.0f;
    return T_RET - EFF_MAX * f * f * (T_RET - HX_PRIMARY);
}

static void plant_init(plant_t *p){
    memset(p, 0, sizeof(*p));
    p->pos = START_POS;
    p->src = src_ss_of(p->pos);
    float t0 = T_RET + p->pos / 100.0f * (p->src - T_RET);
    for (int i = 0; i < THETA_S; i++) p->delay[i] = t0;
    p->probe = t0;
}

static void plant_tick(plant_t *p, float target){
    float err = target - p->pos;
    if (fabsf(err) > VALVE_DB){
        float step = 100.0f / TRAVEL_S;
        float d = (err > 0.0f) ? step : -step;
        if (fabsf(d) > fabsf(err)) d = err;
        p->pos += d;
        p->travel += fabsf(d);
    }
    p->pos = ctrl_clampf(p->pos, 0.0f, 100.0f);

    p->src += (src_ss_of(p->pos) - p->src) * (1.0f / SRC_TAU_S);

    float t_mix = T_RET + p->pos / 100.0f * (p->src - T_RET);
    float delayed = p->delay[p->di];
    p->delay[p->di] = t_mix;
    p->di = (p->di + 1) % THETA_S;
    p->probe += (delayed - p->probe) * (1.0f / TAU_S);
}

typedef struct {
    float sup_mean_err, sup_pp, pos_min, pos_max, pos_final, src_final, den_final;
    double travel;
} result_t;

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

    float target = START_POS;
    float smin = 1e9f, smax = -1e9f, pmin = 1e9f, pmax = -1e9f;
    double sum = 0.0; long n = 0; double travel0 = 0.0;

    for (int t = 0; t < SIM_S; t++){
        plant_tick(&p, target);
        if (t % 10 == 0){
            in.t_supply   = 0.0625f * roundf(p.probe / 0.0625f);   /* 12-bit DS18B20 */
            in.t_source_f = p.src;
            in.valve_pos  = p.pos;
            in.link_last_seen_ms = (uint32_t)t * 1000u;
            control_out_t o = control_step(&st, &in, &cfg, (uint32_t)t * 1000u);
            if (o.regulating) target = o.valve_target;
        }
        if (t == SETTLE_S) travel0 = p.travel;
        if (t >= SETTLE_S){
            if (p.probe < smin) smin = p.probe;
            if (p.probe > smax) smax = p.probe;
            if (p.pos   < pmin) pmin = p.pos;
            if (p.pos   > pmax) pmax = p.pos;
            sum += p.probe; n++;
        }
    }
    result_t r;
    r.sup_mean_err = (float)(sum / (double)n) - T_SET;
    r.sup_pp = smax - smin;
    r.pos_min = pmin; r.pos_max = pmax; r.pos_final = p.pos;
    r.src_final = p.src; r.den_final = fabsf(p.src - T_RET);
    r.travel = p.travel - travel0;
    return r;
}

void setUp(void){} void tearDown(void){}

/* The plant is calibrated so the setpoint IS reachable: supply(pos) = t_ret -
 * eff_max*(pos/100)^3*(t_ret-primary), which hits 18.5 at pos ~= 95.7 %. So a
 * controller that cannot get there has given up authority it actually had. */
void test_starved_source_recovers(void){
    result_t r = run_sim();
    printf("ffrec: final pos %.1f %%  src %.2f  |den| %.2f  mean err %+.2f K  "
           "sup p-p %.2f  pos %.1f..%.1f  travel %.0f %%\n",
           r.pos_final, r.src_final, r.den_final, r.sup_mean_err, r.sup_pp,
           r.pos_min, r.pos_max, r.travel);

    /* 1. THE LATCH: the valve must not still be sitting where it started. This is the
     *    single assertion the 2026-08-04 incident violates -- 55.7 % for 92 minutes. */
    TEST_ASSERT_TRUE(fabsf(r.pos_final - START_POS) > 10.0f);

    /* 2. Authority must be RESTORED, not merely declared lost. Opening the valve is
     *    what re-establishes the denominator the FF needs (measured coupling < 0). */
    TEST_ASSERT_TRUE(r.den_final > 2.0f);

    /* 3. Supply must actually reach the setpoint it is capable of reaching. Live error
     *    while latched was +2.6..2.75 K; the manual full-source override got to 0.25 K. */
    TEST_ASSERT_TRUE(fabsf(r.sup_mean_err) < 0.6f);

    /* 4. ...without trading it for motor thrash. 120 s travel over the judged hour: a
     *    healthy loop nudges, it does not sweep. */
    TEST_ASSERT_TRUE(r.travel < 400.0);
}

int main(void){
    UNITY_BEGIN();
    RUN_TEST(test_starved_source_recovers);
    return UNITY_END();
}
