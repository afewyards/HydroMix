#include "unity.h"
#include "ctrl_core/resync_policy.h"

static resync_policy_state_t s;
void setUp(void){ resync_policy_init(&s); }
void tearDown(void){}

void test_gate_pass(void){
    bool ok, hard;
    resync_gate_eval(18.6f, false, true, 18.5f, 16.0f, 36.0f, &ok, &hard);
    TEST_ASSERT_TRUE(ok); TEST_ASSERT_FALSE(hard);
}
void test_gate_fail_distance(void){          /* 2.5 K off setpoint */
    bool ok, hard;
    resync_gate_eval(21.0f, false, true, 18.5f, 16.0f, 36.0f, &ok, &hard);
    TEST_ASSERT_FALSE(ok); TEST_ASSERT_FALSE(hard);
}
void test_gate_fail_gov_margin(void){        /* within 2 K but < gov_low+1 */
    bool ok, hard;
    resync_gate_eval(16.5f, false, true, 17.5f, 16.0f, 36.0f, &ok, &hard);
    TEST_ASSERT_FALSE(ok); TEST_ASSERT_FALSE(hard);
}
void test_gate_hard_outside_band(void){      /* cold slug: 14.6 < gov_low */
    bool ok, hard;
    resync_gate_eval(14.6f, false, true, 18.5f, 16.0f, 36.0f, &ok, &hard);
    TEST_ASSERT_FALSE(ok); TEST_ASSERT_TRUE(hard);
}
void test_gate_hard_fault_or_idle(void){
    bool ok, hard;
    resync_gate_eval(18.5f, true,  true,  18.5f, 16.0f, 36.0f, &ok, &hard);
    TEST_ASSERT_TRUE(hard);
    resync_gate_eval(18.5f, false, false, 18.5f, 16.0f, 36.0f, &ok, &hard);
    TEST_ASSERT_TRUE(hard); TEST_ASSERT_FALSE(ok);
}
void test_no_trip_no_action(void){
    TEST_ASSERT_EQUAL(RESYNC_ACT_NONE, resync_policy_step(&s, false, true, 90.0f, 0));
}
void test_low_pos_recirc_now(void){
    TEST_ASSERT_EQUAL(RESYNC_ACT_START_RECIRC, resync_policy_step(&s, true, false, 20.0f, 0));
}
void test_high_pos_gate_ok_source(void){
    TEST_ASSERT_EQUAL(RESYNC_ACT_START_SOURCE, resync_policy_step(&s, true, true, 90.0f, 0));
}
void test_defer_then_gate_pass(void){
    TEST_ASSERT_EQUAL(RESYNC_ACT_NONE,         resync_policy_step(&s, true, false, 90.0f, 0));
    TEST_ASSERT_EQUAL(RESYNC_ACT_NONE,         resync_policy_step(&s, true, false, 90.0f, 600000));
    TEST_ASSERT_EQUAL(RESYNC_ACT_START_SOURCE, resync_policy_step(&s, true, true,  90.0f, 900000));
}
void test_defer_timeout_recirc(void){
    TEST_ASSERT_EQUAL(RESYNC_ACT_NONE,         resync_policy_step(&s, true, false, 90.0f, 0));
    TEST_ASSERT_EQUAL(RESYNC_ACT_START_RECIRC, resync_policy_step(&s, true, false, 90.0f, RESYNC_DEFER_MAX_MS));
}
void test_defer_pos_drop_recirc(void){       /* valve closed below 50 % while deferring */
    TEST_ASSERT_EQUAL(RESYNC_ACT_NONE,         resync_policy_step(&s, true, false, 90.0f, 0));
    TEST_ASSERT_EQUAL(RESYNC_ACT_START_RECIRC, resync_policy_step(&s, true, false, 40.0f, 60000));
}
void test_abort_only_source_stroke(void){
    TEST_ASSERT_TRUE (resync_policy_mid_stroke_abort(true,  true));
    TEST_ASSERT_FALSE(resync_policy_mid_stroke_abort(true,  false));
    TEST_ASSERT_FALSE(resync_policy_mid_stroke_abort(false, true));
}

int main(void){
    UNITY_BEGIN();
    RUN_TEST(test_gate_pass);
    RUN_TEST(test_gate_fail_distance);
    RUN_TEST(test_gate_fail_gov_margin);
    RUN_TEST(test_gate_hard_outside_band);
    RUN_TEST(test_gate_hard_fault_or_idle);
    RUN_TEST(test_no_trip_no_action);
    RUN_TEST(test_low_pos_recirc_now);
    RUN_TEST(test_high_pos_gate_ok_source);
    RUN_TEST(test_defer_then_gate_pass);
    RUN_TEST(test_defer_timeout_recirc);
    RUN_TEST(test_defer_pos_drop_recirc);
    RUN_TEST(test_abort_only_source_stroke);
    return UNITY_END();
}
