#include "unity.h"
#include "ctrl_core/feedforward.h"

static ff_state_t s;
static ff_cfg_t   cfg;      /* shipped defaults: fixed 2 K floor + 180 s output EMA */
static ff_cfg_t   legacy;   /* fixed 2 K floor, no output filter */
static ff_cfg_t   derived;  /* the 1.5.0 setpoint-derived floor, kept only to test the math */

void setUp(void){
    ff_init(&s);
    ff_cfg_defaults(&cfg);
    legacy  = (ff_cfg_t){ .coupling_pct_k = 0.0f, .auth_min_k = 2.0f,
                          .auth_max_k = 2.0f, .out_tau_s = 0.0f };
    derived = (ff_cfg_t){ .coupling_pct_k = 0.065f, .auth_min_k = 2.0f,
                          .auth_max_k = 4.0f, .out_tau_s = 0.0f };
}
void tearDown(void){}

/* Heating: set 35, return 30, source 45 -> (35-30)/(45-30)=33.3 %. */
void test_formula_heating(void){
    ff_result_t r = ff_step(&s, &cfg, 35.0f, 30.0f, 45.0f, 0);
    TEST_ASSERT_FALSE(r.frozen);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 33.33f, r.pos_ff);
}

/* Cooling: set 18, return 24, source 8 -> (18-24)/(8-24)=37.5 %. */
void test_formula_cooling(void){
    ff_result_t r = ff_step(&s, &cfg, 18.0f, 24.0f, 8.0f, 0);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 37.5f, r.pos_ff);
}

/* Result clamps 0..100 even with authority to spare: (50-30)/(35-30) = 400 %. */
void test_clamp(void){
    ff_result_t r = ff_step(&s, &cfg, 50.0f, 30.0f, 35.0f, 0);
    TEST_ASSERT_FALSE(r.frozen);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, r.pos_ff);
}

/* ---- low authority: LIMIT the denominator, never hold a stale baseline -------------
 *
 * This is the 1.5.1 behaviour change. Up to 1.5.0 a low-authority step returned the last
 * valid output and control.c drove the valve with that frozen constant, which latched the
 * valve for 92 minutes live on 2026-08-04. The floor is now applied to the DENOMINATOR, so
 * the output stays live, bounded and correctly signed. */

/* Cooling with the source converging on the return: the ratio saturates toward the source,
 * which is the move that RESTORES authority (measured d t_src/d pos < 0). Critically it
 * must NOT be the stale 33.3 % that the previous valid step produced. */
void test_low_authority_drives_toward_source_not_stale_baseline(void){
    ff_step(&s, &cfg, 35.0f, 30.0f, 45.0f, 0);                 /* valid ~33.3 first */
    ff_result_t r = ff_step(&s, &cfg, 35.0f, 30.0f, 31.0f, 0); /* |31-30|=1 < 2 -> limited */
    TEST_ASSERT_TRUE(r.frozen);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, r.pos_ff);                 /* 5/2*100 = 250 -> clamp */
}

/* Same with no prior valid step at all: still a live answer, not FF_DEFAULT_PCT. */
void test_low_authority_no_prior_still_computes(void){
    ff_result_t r = ff_step(&s, &cfg, 18.5f, 21.2f, 20.9f, 0); /* cooling, |denom| 0.3 */
    TEST_ASSERT_TRUE(r.frozen);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, r.pos_ff);                 /* -2.7/-2.0*100 = 135 */
}

/* A source on the WRONG side of the return cannot help, so it must be shut out entirely --
 * the ratio goes negative and clamps to 0. This is the unseated-probe case (2026-07-31)
 * where t_source reads above t_return during cooling. */
void test_low_authority_wrong_side_source_closes(void){
    ff_result_t r = ff_step(&s, &cfg, 18.5f, 21.2f, 21.5f, 0);
    TEST_ASSERT_TRUE(r.frozen);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, r.pos_ff);
}

/* denom exactly 0 has no sign of its own: fall back to the demand direction, so cooling
 * still opens toward the source rather than dividing by zero or picking a rail at random. */
void test_zero_denominator_uses_demand_direction(void){
    ff_result_t r = ff_step(&s, &cfg, 18.5f, 21.2f, 21.2f, 0);
    TEST_ASSERT_TRUE(r.frozen);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, r.pos_ff);
    ff_state_t s2; ff_init(&s2);
    ff_result_t h = ff_step(&s2, &cfg, 35.0f, 30.0f, 30.0f, 0);   /* heating demand */
    TEST_ASSERT_EQUAL_FLOAT(100.0f, h.pos_ff);
}

/* ---- authority floor -------------------------------------------------------------- */

/* The derivation itself still computes what it always did -- it is just not shipped.
 * floor = sqrt(|t_ret - t_set| * 100 * coupling), clamped to [min,max]. */
void test_authority_floor_derivation_math(void){
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 2.79f, ff_authority_floor(&derived, 20.0f, 21.2f));
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 3.79f, ff_authority_floor(&derived, 19.0f, 21.2f));
    /* clamps: a setpoint sitting on the return needs no margin, a far one is capped */
    TEST_ASSERT_EQUAL_FLOAT(FF_AUTHORITY_MIN_K, ff_authority_floor(&derived, 21.2f, 21.2f));
    TEST_ASSERT_EQUAL_FLOAT(FF_AUTHORITY_MAX_K, ff_authority_floor(&derived, 35.0f, 21.2f));
}

/* coupling 0 (the shipped default) disables the derivation -> floor is just auth_min_k. */
void test_shipped_floor_is_fixed(void){
    TEST_ASSERT_EQUAL_FLOAT(2.0f, ff_authority_floor(&cfg, 19.0f, 21.2f));
    TEST_ASSERT_EQUAL_FLOAT(2.0f, ff_authority_floor(&cfg, 18.5f, 21.2f));
    TEST_ASSERT_EQUAL_FLOAT(2.0f, ff_authority_floor(&legacy, 19.0f, 21.2f));
}

/* REGRESSION (2026-08-04): the derived floor reached 4.0 K at the live 18.5 C setpoint and
 * put the trip point ABOVE the plant's normal |denom| range (measured 0.07..4.75, mean
 * 1.89), so 87 % of samples were treated as no-authority. The shipped floor must leave a
 * 3 K denominator alone -- that is a perfectly usable source. */
void test_shipped_floor_leaves_usable_denominator_alone(void){
    ff_result_t r = ff_step(&s, &cfg, 18.5f, 21.2f, 18.2f, 0);   /* |denom| = 3.0 */
    TEST_ASSERT_FALSE(r.frozen);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 4.0f, ff_authority_floor(&derived, 18.5f, 21.2f));
    ff_state_t s2; ff_init(&s2);
    ff_result_t old = ff_step(&s2, &derived, 18.5f, 21.2f, 18.2f, 0);
    TEST_ASSERT_TRUE(old.frozen);                               /* what 1.5.0 did here */
}

/* ---- output filter ---------------------------------------------------------------- */

/* One tau of elapsed time -> alpha = dt/(tau+dt) = 0.5, so the output moves halfway to
 * the new raw value instead of jumping to it. Tau is 180 s since 1.5.1 (was 900). */
void test_output_ema_smooths_step(void){
    ff_result_t a = ff_step(&s, &cfg, 20.0f, 21.2f, 15.34f, 0);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 20.48f, a.pos_ff);             /* first call reseeds */

    ff_result_t b = ff_step(&s, &cfg, 20.0f, 21.2f, 13.2f, 180000);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 17.74f, b.pos_ff);             /* half of 20.48 -> 15.0 */

    ff_state_t s2; ff_init(&s2);
    ff_step(&s2, &legacy, 20.0f, 21.2f, 15.34f, 0);
    ff_result_t c = ff_step(&s2, &legacy, 20.0f, 21.2f, 13.2f, 180000);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 15.0f, c.pos_ff);              /* unfiltered: jumps */
}

/* The filter keeps running through a low-authority step, so the output is continuous
 * across the boundary rather than jumping to a rail the moment the floor is crossed. */
void test_filter_applies_through_low_authority(void){
    ff_step(&s, &cfg, 20.0f, 21.2f, 15.34f, 0);                   /* ~20.48 */
    ff_result_t f = ff_step(&s, &cfg, 20.0f, 21.2f, 21.0f, 180000); /* |0.2| -> limited */
    TEST_ASSERT_TRUE(f.frozen);
    /* denom -0.2 limits to -2.0, so raw = -1.2/-2.0*100 = 60 (no clamping needed);
     * alpha 0.5 -> halfway from 20.48 to 60, i.e. a nudge and not a jump to the rail. */
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 40.24f, f.pos_ff);
}

/* Heating<->cooling inverts pos_ff, so the filter must not carry across the flip. */
void test_mode_change_reseeds_filter(void){
    ff_step(&s, &cfg, 20.0f, 21.2f, 15.34f, 0);                   /* cooling, ~20.5 */
    ff_mode_change(&s);
    ff_result_t r = ff_step(&s, &cfg, 35.0f, 30.0f, 45.0f, 10000); /* heating, 33.3 */
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 33.33f, r.pos_ff);              /* not blended with 20.5 */
}

/* ff_reseed drops the filter, so the next step lands on the raw ratio. */
void test_reseed_drops_filter(void){
    ff_step(&s, &cfg, 20.0f, 21.2f, 15.34f, 0);
    ff_reseed(&s);
    ff_result_t r = ff_step(&s, &cfg, 20.0f, 21.2f, 13.2f, 1000);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 15.0f, r.pos_ff);
}

int main(void){
    UNITY_BEGIN();
    RUN_TEST(test_formula_heating);
    RUN_TEST(test_formula_cooling);
    RUN_TEST(test_clamp);
    RUN_TEST(test_low_authority_drives_toward_source_not_stale_baseline);
    RUN_TEST(test_low_authority_no_prior_still_computes);
    RUN_TEST(test_low_authority_wrong_side_source_closes);
    RUN_TEST(test_zero_denominator_uses_demand_direction);
    RUN_TEST(test_authority_floor_derivation_math);
    RUN_TEST(test_shipped_floor_is_fixed);
    RUN_TEST(test_shipped_floor_leaves_usable_denominator_alone);
    RUN_TEST(test_output_ema_smooths_step);
    RUN_TEST(test_filter_applies_through_low_authority);
    RUN_TEST(test_mode_change_reseeds_filter);
    RUN_TEST(test_reseed_drops_filter);
    return UNITY_END();
}
