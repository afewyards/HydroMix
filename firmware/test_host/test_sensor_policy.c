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
    TEST_ASSERT_EQUAL_INT(SENSOR_FAULT_AFTER, s.fail_streak);  /* not zeroed by the single good read */
}

void test_clears_after_3_consecutive_goods(void){
    sensor_fault_state_t s = {0};
    sensor_fault_update(&s, false);
    sensor_fault_update(&s, false);
    sensor_fault_update(&s, false);            /* latched */
    TEST_ASSERT_FALSE(sensor_fault_update(&s, true));
    TEST_ASSERT_FALSE(sensor_fault_update(&s, true));
    TEST_ASSERT_TRUE(sensor_fault_update(&s, true));   /* SENSOR_CLEAR_AFTER'th good clears */
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

/* Leaky-bucket coverage. Rule under test (sensor_policy.c): fail_streak decays by 1
 * on every consecutive good read from the SENSOR_DECAY_AFTER'th onward; good_streak
 * resets to 0 on any bad read. Break-even is therefore one fail in
 * SENSOR_DECAY_AFTER+1 reads -- 25 % at the shipped value of 3. */

/* good,good,fail repeating (33 % failure) MUST latch: two goods never reach
 * SENSOR_DECAY_AFTER, so nothing is forgiven and fail_streak climbs by exactly 1
 * per cycle. Latches on the 3rd cycle's fail == the 9th sweep. Regression guard for
 * the 1.2.1 bug where the threshold was 2 and this pattern never latched at all. */
void test_repeating_good_good_fail_latches(void){
    sensor_fault_state_t s = {0};
    int cycles_to_latch = -1;
    for (int cycle = 0; cycle < 20 && cycles_to_latch < 0; ++cycle) {
        sensor_fault_update(&s, true);
        sensor_fault_update(&s, true);
        TEST_ASSERT_EQUAL_INT(cycle, s.fail_streak);        /* nothing forgiven */
        sensor_fault_update(&s, false);
        TEST_ASSERT_EQUAL_INT(cycle + 1, s.fail_streak);
        if (s.faulted) cycles_to_latch = cycle + 1;
    }
    TEST_ASSERT_EQUAL_INT(SENSOR_FAULT_AFTER, cycles_to_latch);
}

/* good,good,good,fail repeating (25 % failure, the break-even rate) must NEVER
 * latch: the 3rd consecutive good forgives that cycle's fail exactly, so
 * fail_streak oscillates 0 <-> 1 forever. */
void test_repeating_three_good_one_fail_never_latches(void){
    sensor_fault_state_t s = {0};
    for (int cycle = 0; cycle < 50; ++cycle) {
        sensor_fault_update(&s, true);
        sensor_fault_update(&s, true);
        sensor_fault_update(&s, true);
        TEST_ASSERT_EQUAL_INT(0, s.fail_streak);            /* 3rd good decays */
        sensor_fault_update(&s, false);
        TEST_ASSERT_EQUAL_INT(1, s.fail_streak);
        TEST_ASSERT_FALSE(s.faulted);
    }
}

/* A long good run keeps decaying once per read -- good_streak is not reset by a
 * decay, only by a bad read. Three fails then five goods must fully drain the
 * bucket (goods 3, 4 and 5 each forgive one). */
void test_long_good_run_decays_once_per_read(void){
    sensor_fault_state_t s = {0};
    sensor_fault_update(&s, false);
    sensor_fault_update(&s, false);            /* fail_streak 2, not yet latched */
    TEST_ASSERT_FALSE(s.faulted);
    sensor_fault_update(&s, true);
    sensor_fault_update(&s, true);
    TEST_ASSERT_EQUAL_INT(2, s.fail_streak);
    sensor_fault_update(&s, true);             /* 3rd consecutive good */
    TEST_ASSERT_EQUAL_INT(1, s.fail_streak);
    sensor_fault_update(&s, true);             /* 4th */
    TEST_ASSERT_EQUAL_INT(0, s.fail_streak);
    sensor_fault_update(&s, true);             /* 5th: floor at 0, no underflow */
    TEST_ASSERT_EQUAL_INT(0, s.fail_streak);
}

/* fail,fail,good,fail: the single good read only reaches good_streak == 1, well short
 * of SENSOR_DECAY_AFTER, so it forgives nothing. The trailing fail pushes fail_streak
 * to SENSOR_FAULT_AFTER and latches -- a net-failing sensor (3 bad in 4 sweeps) must
 * latch. */
void test_fail_fail_good_fail_latches(void){
    sensor_fault_state_t s = {0};
    sensor_fault_update(&s, false);
    TEST_ASSERT_EQUAL_INT(1, s.fail_streak);
    sensor_fault_update(&s, false);
    TEST_ASSERT_EQUAL_INT(2, s.fail_streak);
    TEST_ASSERT_FALSE(s.faulted);
    sensor_fault_update(&s, true);
    TEST_ASSERT_EQUAL_INT(2, s.fail_streak);   /* 1 consecutive good < SENSOR_DECAY_AFTER */
    TEST_ASSERT_FALSE(s.faulted);
    sensor_fault_update(&s, false);
    TEST_ASSERT_EQUAL_INT(SENSOR_FAULT_AFTER, s.fail_streak);
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

/* fail,fail,good repeating: the lone good never reaches SENSOR_DECAY_AFTER consecutive
 * (the next fail resets good_streak to 0 first), so decay never fires at all --
 * fail_streak climbs by 2 every cycle and a mostly-bad sensor latches almost
 * immediately (at the start of the 2nd cycle). */
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
    RUN_TEST(test_repeating_good_good_fail_latches);
    RUN_TEST(test_repeating_three_good_one_fail_never_latches);
    RUN_TEST(test_long_good_run_decays_once_per_read);
    RUN_TEST(test_fail_fail_good_fail_latches);
    RUN_TEST(test_three_consecutive_fails_still_latches);
    RUN_TEST(test_repeating_fail_fail_good_latches_within_a_few_cycles);
    RUN_TEST(test_state_restarts_cleanly_after_clear);
    RUN_TEST(test_ema_reseed_bypasses_filter);
    RUN_TEST(test_ema_normal_step_uses_alpha);
    return UNITY_END();
}
