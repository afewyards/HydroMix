#include "unity.h"
#include "ctrl_core/feedforward.h"

static ff_state_t s;
void setUp(void){ ff_init(&s); }
void tearDown(void){}

/* Heating: set 35, return 30, source 45 -> (35-30)/(45-30)=33.3 %. */
void test_formula_heating(void){
    ff_result_t r = ff_step(&s, 35.0f, 30.0f, 45.0f, 0);
    TEST_ASSERT_FALSE(r.frozen);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 33.33f, r.pos_ff);
}

/* Cooling: set 18, return 24, source 8 -> (18-24)/(8-24)=37.5 %. */
void test_formula_cooling(void){
    ff_result_t r = ff_step(&s, 18.0f, 24.0f, 8.0f, 0);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 37.5f, r.pos_ff);
}

/* Result clamps 0..100. */
void test_clamp(void){
    ff_result_t r = ff_step(&s, 50.0f, 30.0f, 31.0f, 0); /* huge -> >100 but authority<2K first */
    (void)r;
    r = ff_step(&s, 50.0f, 30.0f, 33.0f, 0);             /* (50-30)/3=666% -> clamp 100 */
    TEST_ASSERT_EQUAL_FLOAT(100.0f, r.pos_ff);
}

/* Low authority (<2K): freeze last valid, do NOT zero. */
void test_freeze_low_authority(void){
    ff_step(&s, 35.0f, 30.0f, 45.0f, 0);                 /* valid ~33.3 stored */
    ff_result_t r = ff_step(&s, 35.0f, 30.0f, 31.0f, 0); /* |31-30|=1<2 -> freeze */
    TEST_ASSERT_TRUE(r.frozen);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 33.33f, r.pos_ff);
}

/* Low authority with no prior valid -> safe mid (50 %), frozen. */
void test_freeze_no_prior(void){
    ff_result_t r = ff_step(&s, 35.0f, 30.0f, 30.5f, 0);
    TEST_ASSERT_TRUE(r.frozen);
    TEST_ASSERT_EQUAL_FLOAT(50.0f, r.pos_ff);
}

/* Freeze latches at first low-authority cycle; park_requested only fires once
 * continuously frozen for >= FF_NO_AUTHORITY_PARK_DWELL_MS (60 s). */
void test_park_requested_after_dwell(void){
    ff_step(&s, 35.0f, 30.0f, 30.5f, 0);                  /* freeze begins t=0 */
    ff_result_t r = ff_step(&s, 35.0f, 30.0f, 30.5f, 30000);
    TEST_ASSERT_TRUE(r.frozen);
    TEST_ASSERT_FALSE(r.park_requested);
    r = ff_step(&s, 35.0f, 30.0f, 30.5f, 61000);
    TEST_ASSERT_TRUE(r.frozen);
    TEST_ASSERT_TRUE(r.park_requested);
}

/* Authority recovering mid-freeze resets the dwell clock: a subsequent freeze
 * starts counting from zero again, not from the original freeze onset. */
void test_authority_recovery_resets_dwell(void){
    ff_step(&s, 35.0f, 30.0f, 30.5f, 0);                  /* freeze begins t=0 */
    ff_step(&s, 35.0f, 30.0f, 30.5f, 40000);              /* still frozen, 40 s in */
    ff_step(&s, 35.0f, 30.0f, 45.0f, 41000);              /* authority restored, denom=15>=2K */
    ff_result_t r = ff_step(&s, 35.0f, 30.0f, 30.5f, 61000); /* freeze again, fresh onset */
    TEST_ASSERT_TRUE(r.frozen);
    TEST_ASSERT_FALSE(r.park_requested);                  /* would be true if dwell carried over from t=0 */
}

/* A dip shorter than the dwell just holds last valid; no park requested. */
void test_brief_dip_shorter_than_dwell_holds_last_valid(void){
    ff_result_t valid = ff_step(&s, 35.0f, 30.0f, 45.0f, 0);   /* valid ~33.3 stored */
    ff_result_t r = ff_step(&s, 35.0f, 30.0f, 30.5f, 10000);   /* freeze begins, ~10 s dip */
    TEST_ASSERT_TRUE(r.frozen);
    TEST_ASSERT_FALSE(r.park_requested);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, valid.pos_ff, r.pos_ff);
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
    return UNITY_END();
}
