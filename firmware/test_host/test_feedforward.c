#include "unity.h"
#include "ctrl_core/feedforward.h"

static ff_state_t s;
static ff_cfg_t   cfg;      /* shipped defaults: derived floor + 900 s output EMA */
static ff_cfg_t   legacy;   /* pre-1.5.0: fixed 2 K floor, no output filter */

void setUp(void){
    ff_init(&s);
    ff_cfg_defaults(&cfg);
    legacy = (ff_cfg_t){ .coupling_pct_k = 0.0f, .auth_min_k = 2.0f,
                         .auth_max_k = 2.0f, .out_tau_s = 0.0f };
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

/* Result clamps 0..100. Source must clear the derived floor (4 K here, since
 * sqrt(20*100*0.065) caps at auth_max_k) for the ratio to be computed at all. */
void test_clamp(void){
    ff_result_t r = ff_step(&s, &cfg, 50.0f, 30.0f, 33.0f, 0); /* |3|<4 -> freeze first */
    TEST_ASSERT_TRUE(r.frozen);
    r = ff_step(&s, &cfg, 50.0f, 30.0f, 35.0f, 0);             /* (50-30)/5=400% -> clamp 100 */
    TEST_ASSERT_EQUAL_FLOAT(100.0f, r.pos_ff);
}

/* Low authority: freeze last valid, do NOT zero. */
void test_freeze_low_authority(void){
    ff_step(&s, &cfg, 35.0f, 30.0f, 45.0f, 0);                 /* valid ~33.3 stored */
    ff_result_t r = ff_step(&s, &cfg, 35.0f, 30.0f, 31.0f, 0); /* |31-30|=1 -> freeze */
    TEST_ASSERT_TRUE(r.frozen);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 33.33f, r.pos_ff);
}

/* Low authority with no prior valid -> safe mid (50 %), frozen. */
void test_freeze_no_prior(void){
    ff_result_t r = ff_step(&s, &cfg, 35.0f, 30.0f, 30.5f, 0);
    TEST_ASSERT_TRUE(r.frozen);
    TEST_ASSERT_EQUAL_FLOAT(50.0f, r.pos_ff);
}

/* Freeze latches at first low-authority cycle; park_requested only fires once
 * continuously frozen for >= FF_NO_AUTHORITY_PARK_DWELL_MS. */
void test_park_requested_after_dwell(void){
    ff_step(&s, &cfg, 35.0f, 30.0f, 30.5f, 0);                  /* freeze begins t=0 */
    ff_result_t r = ff_step(&s, &cfg, 35.0f, 30.0f, 30.5f, FF_NO_AUTHORITY_PARK_DWELL_MS - 30000);
    TEST_ASSERT_TRUE(r.frozen);
    TEST_ASSERT_FALSE(r.park_requested);
    r = ff_step(&s, &cfg, 35.0f, 30.0f, 30.5f, FF_NO_AUTHORITY_PARK_DWELL_MS + 1000);
    TEST_ASSERT_TRUE(r.frozen);
    TEST_ASSERT_TRUE(r.park_requested);
}

/* Authority recovering mid-freeze resets the dwell clock. */
void test_authority_recovery_resets_dwell(void){
    ff_step(&s, &cfg, 35.0f, 30.0f, 30.5f, 0);
    ff_step(&s, &cfg, 35.0f, 30.0f, 30.5f, 40000);
    ff_step(&s, &cfg, 35.0f, 30.0f, 45.0f, 41000);              /* denom=15 >= floor */
    ff_result_t r = ff_step(&s, &cfg, 35.0f, 30.0f, 30.5f, 61000);
    TEST_ASSERT_TRUE(r.frozen);
    TEST_ASSERT_FALSE(r.park_requested);
}

/* A dip shorter than the dwell just holds last valid; no park requested. */
void test_brief_dip_shorter_than_dwell_holds_last_valid(void){
    ff_result_t valid = ff_step(&s, &cfg, 35.0f, 30.0f, 45.0f, 0);
    ff_result_t r = ff_step(&s, &cfg, 35.0f, 30.0f, 30.5f, 10000);
    TEST_ASSERT_TRUE(r.frozen);
    TEST_ASSERT_FALSE(r.park_requested);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, valid.pos_ff, r.pos_ff);
}

/* ---- derived authority floor ------------------------------------------------ */

/* floor = sqrt(|t_ret - t_set| * 100 * coupling), clamped to [min,max]. The whole point
 * is that it TRACKS THE SETPOINT: on GF-HydroMix (t_ret 21.2) dropping the cooling
 * setpoint 20 -> 19 doubles the FF's numerator and needs a wider floor, 2.80 -> 3.79 K. */
void test_authority_floor_tracks_setpoint(void){
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 2.79f, ff_authority_floor(&cfg, 20.0f, 21.2f));
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 3.79f, ff_authority_floor(&cfg, 19.0f, 21.2f));
    /* clamps: a setpoint sitting on the return needs no margin, a far one is capped */
    TEST_ASSERT_EQUAL_FLOAT(FF_AUTHORITY_MIN_K, ff_authority_floor(&cfg, 21.2f, 21.2f));
    TEST_ASSERT_EQUAL_FLOAT(FF_AUTHORITY_MAX_K, ff_authority_floor(&cfg, 35.0f, 21.2f));
}

/* A denominator that clears the old fixed 2 K floor but not the setpoint-derived one is
 * exactly the band where the FF drives its own source warmer and walks to a rail. */
void test_derived_floor_freezes_where_fixed_floor_did_not(void){
    ff_step(&s, &cfg, 19.0f, 21.2f, 14.0f, 0);                    /* seed a valid baseline */
    ff_result_t r = ff_step(&s, &cfg, 19.0f, 21.2f, 18.2f, 1000); /* |denom|=3.0: 2<3<3.79 */
    TEST_ASSERT_TRUE(r.frozen);

    ff_state_t s2; ff_init(&s2);
    ff_step(&s2, &legacy, 19.0f, 21.2f, 14.0f, 0);
    ff_result_t r2 = ff_step(&s2, &legacy, 19.0f, 21.2f, 18.2f, 1000);
    TEST_ASSERT_FALSE(r2.frozen);                                 /* old behaviour: sails on */
}

/* coupling 0 disables the derivation entirely -> floor is just auth_min_k. */
void test_coupling_zero_restores_fixed_floor(void){
    TEST_ASSERT_EQUAL_FLOAT(2.0f, ff_authority_floor(&legacy, 19.0f, 21.2f));
}

/* ---- output filter ---------------------------------------------------------- */

/* One tau of elapsed time -> alpha = dt/(tau+dt) = 0.5, so the output moves halfway to
 * the new raw value instead of jumping to it. */
void test_output_ema_smooths_step(void){
    ff_result_t a = ff_step(&s, &cfg, 20.0f, 21.2f, 15.34f, 0);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 20.48f, a.pos_ff);             /* first call reseeds */

    ff_result_t b = ff_step(&s, &cfg, 20.0f, 21.2f, 13.2f, 900000);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 17.74f, b.pos_ff);             /* half of 20.48 -> 15.0 */

    ff_state_t s2; ff_init(&s2);
    ff_step(&s2, &legacy, 20.0f, 21.2f, 15.34f, 0);
    ff_result_t c = ff_step(&s2, &legacy, 20.0f, 21.2f, 13.2f, 900000);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 15.0f, c.pos_ff);              /* unfiltered: jumps */
}

/* The freeze holds the FILTERED baseline, not the last raw ratio. */
void test_freeze_holds_filtered_value(void){
    ff_step(&s, &cfg, 20.0f, 21.2f, 15.34f, 0);
    ff_result_t b = ff_step(&s, &cfg, 20.0f, 21.2f, 13.2f, 900000);
    ff_result_t f = ff_step(&s, &cfg, 20.0f, 21.2f, 21.0f, 901000);  /* |0.2| -> freeze */
    TEST_ASSERT_TRUE(f.frozen);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, b.pos_ff, f.pos_ff);
}

/* Heating<->cooling inverts pos_ff, so the filter must not carry across the flip. */
void test_mode_change_reseeds_filter(void){
    ff_step(&s, &cfg, 20.0f, 21.2f, 15.34f, 0);                   /* cooling, ~20.5 */
    ff_mode_change(&s);
    ff_result_t r = ff_step(&s, &cfg, 35.0f, 30.0f, 45.0f, 10000); /* heating, 33.3 */
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 33.33f, r.pos_ff);              /* not blended with 20.5 */
}

/* ff_reseed drops the filter but keeps last_valid available for a subsequent freeze. */
void test_reseed_keeps_last_valid(void){
    ff_result_t a = ff_step(&s, &cfg, 20.0f, 21.2f, 15.34f, 0);
    ff_reseed(&s);
    ff_result_t f = ff_step(&s, &cfg, 20.0f, 21.2f, 21.0f, 1000);  /* low authority */
    TEST_ASSERT_TRUE(f.frozen);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, a.pos_ff, f.pos_ff);
}

int main(void){
    UNITY_BEGIN();
    RUN_TEST(test_formula_heating);
    RUN_TEST(test_formula_cooling);
    RUN_TEST(test_clamp);
    RUN_TEST(test_freeze_low_authority);
    RUN_TEST(test_freeze_no_prior);
    RUN_TEST(test_park_requested_after_dwell);
    RUN_TEST(test_authority_recovery_resets_dwell);
    RUN_TEST(test_brief_dip_shorter_than_dwell_holds_last_valid);
    RUN_TEST(test_authority_floor_tracks_setpoint);
    RUN_TEST(test_derived_floor_freezes_where_fixed_floor_did_not);
    RUN_TEST(test_coupling_zero_restores_fixed_floor);
    RUN_TEST(test_output_ema_smooths_step);
    RUN_TEST(test_freeze_holds_filtered_value);
    RUN_TEST(test_mode_change_reseeds_filter);
    RUN_TEST(test_reseed_keeps_last_valid);
    return UNITY_END();
}
