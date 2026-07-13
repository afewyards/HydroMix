#include "unity.h"
#include "ctrl_core/feedforward.h"

static ff_state_t s;
void setUp(void){ ff_init(&s); }
void tearDown(void){}

/* Heating: set 35, return 30, source 45 -> (35-30)/(45-30)=33.3 %. */
void test_formula_heating(void){
    ff_result_t r = ff_step(&s, 35.0f, 30.0f, 45.0f);
    TEST_ASSERT_FALSE(r.frozen);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 33.33f, r.pos_ff);
}

/* Cooling: set 18, return 24, source 8 -> (18-24)/(8-24)=37.5 %. */
void test_formula_cooling(void){
    ff_result_t r = ff_step(&s, 18.0f, 24.0f, 8.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 37.5f, r.pos_ff);
}

/* Result clamps 0..100. */
void test_clamp(void){
    ff_result_t r = ff_step(&s, 50.0f, 30.0f, 31.0f); /* huge -> >100 but authority<2K first */
    (void)r;
    r = ff_step(&s, 50.0f, 30.0f, 33.0f);             /* (50-30)/3=666% -> clamp 100 */
    TEST_ASSERT_EQUAL_FLOAT(100.0f, r.pos_ff);
}

/* Low authority (<2K): freeze last valid, do NOT zero. */
void test_freeze_low_authority(void){
    ff_step(&s, 35.0f, 30.0f, 45.0f);                 /* valid ~33.3 stored */
    ff_result_t r = ff_step(&s, 35.0f, 30.0f, 31.0f); /* |31-30|=1<2 -> freeze */
    TEST_ASSERT_TRUE(r.frozen);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 33.33f, r.pos_ff);
}

/* Low authority with no prior valid -> safe mid (50 %), frozen. */
void test_freeze_no_prior(void){
    ff_result_t r = ff_step(&s, 35.0f, 30.0f, 30.5f);
    TEST_ASSERT_TRUE(r.frozen);
    TEST_ASSERT_EQUAL_FLOAT(50.0f, r.pos_ff);
}

int main(void){
    UNITY_BEGIN();
    RUN_TEST(test_formula_heating);
    RUN_TEST(test_formula_cooling);
    RUN_TEST(test_clamp);
    RUN_TEST(test_freeze_low_authority);
    RUN_TEST(test_freeze_no_prior);
    return UNITY_END();
}
