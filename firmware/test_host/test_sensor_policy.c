#include "unity.h"
#include "ctrl_core/sensor_policy.h"

void setUp(void){} void tearDown(void){}

/* Exactly 0x0550 (85.0 C) is the DS18B20 POR scratchpad value. */
void test_por_raw_detected(void){
    TEST_ASSERT_TRUE(sensor_fault_is_por_raw(0x0550));
    TEST_ASSERT_FALSE(sensor_fault_is_por_raw(0x0551));
    TEST_ASSERT_FALSE(sensor_fault_is_por_raw(0x0000));
}

void test_latches_after_3_consecutive_fails(void){
    sensor_fault_state_t s = {0};
    TEST_ASSERT_FALSE(sensor_fault_update(&s, false));
    TEST_ASSERT_FALSE(s.faulted);
    TEST_ASSERT_FALSE(sensor_fault_update(&s, false));
    TEST_ASSERT_FALSE(s.faulted);
    TEST_ASSERT_FALSE(sensor_fault_update(&s, false));
    TEST_ASSERT_TRUE(s.faulted);
}

void test_single_good_does_not_clear_latched_fault(void){
    sensor_fault_state_t s = {0};
    sensor_fault_update(&s, false);
    sensor_fault_update(&s, false);
    sensor_fault_update(&s, false);            /* latched */
    TEST_ASSERT_TRUE(s.faulted);
    TEST_ASSERT_FALSE(sensor_fault_update(&s, true));
    TEST_ASSERT_TRUE(s.faulted);
    TEST_ASSERT_EQUAL_INT(3, s.fail_streak);   /* not zeroed by the single good read */
}

void test_clears_after_3_consecutive_goods(void){
    sensor_fault_state_t s = {0};
    sensor_fault_update(&s, false);
    sensor_fault_update(&s, false);
    sensor_fault_update(&s, false);            /* latched */
    TEST_ASSERT_FALSE(sensor_fault_update(&s, true));
    TEST_ASSERT_FALSE(sensor_fault_update(&s, true));
    TEST_ASSERT_TRUE(sensor_fault_update(&s, true));   /* 3rd good clears */
    TEST_ASSERT_FALSE(s.faulted);
    TEST_ASSERT_EQUAL_INT(0, s.fail_streak);
}

/* bad,good,bad,good,... latches on the 5th call and never clears (good_streak
 * never reaches 3 consecutive since every other read is bad). */
void test_alternating_good_bad_latches_and_stays_faulted(void){
    sensor_fault_state_t s = {0};
    bool cleared_any = false;
    for (int i = 0; i < 12; ++i) {
        bool ok = (i % 2) == 1;
        if (sensor_fault_update(&s, ok)) cleared_any = true;
        if (i == 4) TEST_ASSERT_TRUE(s.faulted);
    }
    TEST_ASSERT_TRUE(s.faulted);
    TEST_ASSERT_FALSE(cleared_any);
}

void test_ema_reseed_bypasses_filter(void){
    TEST_ASSERT_EQUAL_FLOAT(85.0f, sensor_ema_step(20.0f, 85.0f, 0.2f, true));
}

void test_ema_normal_step_uses_alpha(void){
    TEST_ASSERT_EQUAL_FLOAT(21.0f, sensor_ema_step(20.0f, 25.0f, 0.2f, false));
}

int main(void){
    UNITY_BEGIN();
    RUN_TEST(test_por_raw_detected);
    RUN_TEST(test_latches_after_3_consecutive_fails);
    RUN_TEST(test_single_good_does_not_clear_latched_fault);
    RUN_TEST(test_clears_after_3_consecutive_goods);
    RUN_TEST(test_alternating_good_bad_latches_and_stays_faulted);
    RUN_TEST(test_ema_reseed_bypasses_filter);
    RUN_TEST(test_ema_normal_step_uses_alpha);
    return UNITY_END();
}
