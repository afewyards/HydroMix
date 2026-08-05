#include "unity.h"
#include "ctrl_core/pos_estimator.h"

static pos_est_state_t s;
void setUp(void){ pos_est_init(&s); }
void tearDown(void){}

/* 120 s travel: 60 s open from 0 -> 50 %. */
void test_integrates_position(void){
    pos_est_update(&s, +1, 60000, 120.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 50.0f, s.position_pct);
}

/* Position clamps at 100 and 0. */
void test_clamps(void){
    pos_est_update(&s, +1, 300000, 120.0f);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, s.position_pct);
    pos_est_update(&s, -1, 300000, 120.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s.position_pct);
}

/* Resync at 3x full travel (300 % accumulated). */
void test_resync_on_travel(void){
    for (int i=0;i<3;i++){ pos_est_update(&s,+1,120000,120.0f); pos_est_update(&s,-1,120000,120.0f); }
    /* accumulated = 6 * 100 %? no: 3 open + guard; ensure >300 */
    TEST_ASSERT_TRUE(pos_est_needs_resync(&s));
}

/* Resync at 50 direction reversals. */
void test_resync_on_reversals(void){
    int8_t sign = 1;
    for (int i=0;i<51;i++){ pos_est_update(&s, sign, 100, 120.0f); sign = -sign; }
    TEST_ASSERT_TRUE(pos_est_needs_resync(&s));
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(50, s.reversals);
}

/* resync_done zeroes position, accum, reversals. */
void test_resync_done_clears(void){
    pos_est_update(&s, +1, 60000, 120.0f);
    pos_est_resync_done(&s, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s.position_pct);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s.accum_travel_pct);
    TEST_ASSERT_EQUAL_UINT32(0, s.reversals);
}

void test_resync_done_seeds_position(void){
    pos_est_update(&s, +1, 60000, 120.0f);
    pos_est_resync_done(&s, 100.0f);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, s.position_pct);
    TEST_ASSERT_EQUAL_FLOAT(0.0f,  s.accum_travel_pct);
    TEST_ASSERT_EQUAL_UINT32(0,    s.reversals);
    pos_est_resync_done(&s, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s.position_pct);
}

int main(void){
    UNITY_BEGIN();
    RUN_TEST(test_integrates_position);
    RUN_TEST(test_clamps);
    RUN_TEST(test_resync_on_travel);
    RUN_TEST(test_resync_on_reversals);
    RUN_TEST(test_resync_done_clears);
    RUN_TEST(test_resync_done_seeds_position);
    return UNITY_END();
}
