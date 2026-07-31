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

/* Leaky-bucket regression coverage (1.2.1): fail_streak decays only after
 * TWO CONSECUTIVE good reads (via good_streak, which itself resets to 0 on
 * every bad read). A single isolated good read must not forgive anything,
 * or patterns like alternating bad/good (test_alternating_good_bad_latches_
 * and_stays_faulted below) would never latch. */

/* good,good,fail repeating: per cycle, the first good leaves fail_streak
 * untouched (only 1 consecutive good so far), the second good decays it by
 * 1 (now 2 consecutive), and the following fail raises it by 1 again -- it
 * never reaches SENSOR_FAULT_AFTER (3), no matter how many cycles run. The
 * mid-cycle assertions distinguish this from two wrong fixes: no decay at
 * all (the pre-1.2.1 bug, which would latch by the 3rd fail), and decay on
 * any single good read (which would zero fail_streak after the *first*
 * good instead of the second). */
void test_repeating_good_good_fail_never_latches(void){
    sensor_fault_state_t s = {0};
    for (int cycle = 0; cycle < 20; ++cycle) {
        sensor_fault_update(&s, true);                              /* good #1 */
        if (cycle > 0) TEST_ASSERT_EQUAL_INT(1, s.fail_streak);      /* not yet decayed */
        sensor_fault_update(&s, true);                              /* good #2 */
        TEST_ASSERT_EQUAL_INT(0, s.fail_streak);                    /* decays now */
        sensor_fault_update(&s, false);                             /* fail */
        TEST_ASSERT_EQUAL_INT(1, s.fail_streak);
        TEST_ASSERT_FALSE(s.faulted);
    }
}

/* fail,fail,good,fail: the single good read only reaches good_streak == 1,
 * one short of the 2-consecutive decay requirement, so it forgives nothing.
 * The trailing fail then pushes fail_streak to 3 and latches -- a net-
 * failing sensor (3 bad reads in 4 sweeps) is supposed to latch. */
void test_fail_fail_good_fail_latches(void){
    sensor_fault_state_t s = {0};
    sensor_fault_update(&s, false);
    TEST_ASSERT_EQUAL_INT(1, s.fail_streak);
    sensor_fault_update(&s, false);
    TEST_ASSERT_EQUAL_INT(2, s.fail_streak);
    TEST_ASSERT_FALSE(s.faulted);
    sensor_fault_update(&s, true);
    TEST_ASSERT_EQUAL_INT(2, s.fail_streak);   /* only 1 consecutive good -- no decay */
    TEST_ASSERT_FALSE(s.faulted);
    sensor_fault_update(&s, false);
    TEST_ASSERT_EQUAL_INT(3, s.fail_streak);
    TEST_ASSERT_TRUE(s.faulted);
}

/* 3 consecutive fails must still latch immediately -- the decay only applies
 * to good reads, so this path is unchanged by the fix. */
void test_three_consecutive_fails_still_latches(void){
    sensor_fault_state_t s = {0};
    TEST_ASSERT_FALSE(sensor_fault_update(&s, false));
    TEST_ASSERT_FALSE(sensor_fault_update(&s, false));
    TEST_ASSERT_FALSE(sensor_fault_update(&s, false));
    TEST_ASSERT_TRUE(s.faulted);
}

/* fail,fail,good repeating: the lone good read never reaches 2 consecutive
 * (the next fail resets good_streak to 0 first), so decay never fires at
 * all in this pattern -- fail_streak climbs by 2 every cycle and a mostly-
 * bad sensor latches almost immediately (at the start of the 2nd cycle). */
void test_repeating_fail_fail_good_latches_within_a_few_cycles(void){
    sensor_fault_state_t s = {0};
    bool latched = false;
    for (int cycle = 0; cycle < 3 && !latched; ++cycle) {
        sensor_fault_update(&s, false);
        sensor_fault_update(&s, false);
        sensor_fault_update(&s, true);
        if (s.faulted) latched = true;
    }
    TEST_ASSERT_TRUE(latched);
}

/* After a latched fault clears via 3 consecutive goods, fail_streak must be
 * back at 0 and the state must behave like a fresh sensor -- 3 fresh fails
 * latch again, unaffected by any pre-clear history. */
void test_state_restarts_cleanly_after_clear(void){
    sensor_fault_state_t s = {0};
    sensor_fault_update(&s, false);
    sensor_fault_update(&s, false);
    sensor_fault_update(&s, false);            /* latched */
    sensor_fault_update(&s, true);
    sensor_fault_update(&s, true);
    TEST_ASSERT_TRUE(sensor_fault_update(&s, true));   /* clears */
    TEST_ASSERT_FALSE(s.faulted);
    TEST_ASSERT_EQUAL_INT(0, s.fail_streak);

    TEST_ASSERT_FALSE(sensor_fault_update(&s, false));
    TEST_ASSERT_FALSE(sensor_fault_update(&s, false));
    sensor_fault_update(&s, false);
    TEST_ASSERT_TRUE(s.faulted);
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
    RUN_TEST(test_repeating_good_good_fail_never_latches);
    RUN_TEST(test_fail_fail_good_fail_latches);
    RUN_TEST(test_three_consecutive_fails_still_latches);
    RUN_TEST(test_repeating_fail_fail_good_latches_within_a_few_cycles);
    RUN_TEST(test_state_restarts_cleanly_after_clear);
    RUN_TEST(test_ema_reseed_bypasses_filter);
    RUN_TEST(test_ema_normal_step_uses_alpha);
    return UNITY_END();
}
