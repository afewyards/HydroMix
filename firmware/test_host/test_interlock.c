#include "unity.h"
#include "ctrl_core/interlock.h"

static interlock_state_t s;
void setUp(void){ interlock_init(&s); }
void tearDown(void){}

/* Never both: both requested -> both off + error counter. */
void test_never_both(void){
    triac_cmd_t c = interlock_step(&s, true, true, 0);
    TEST_ASSERT_FALSE(c.open_on); TEST_ASSERT_FALSE(c.close_on);
    TEST_ASSERT_EQUAL_UINT32(1, s.both_error_count);
}

/* First move starts immediately (no prior stop -> no dead-time gate). */
void test_first_move_immediate(void){
    triac_cmd_t c = interlock_step(&s, true, false, 0);
    TEST_ASSERT_TRUE(c.open_on); TEST_ASSERT_FALSE(c.close_on);
}

/* Min pulse 2 s: request drops at 1 s but output stays until 2 s. */
void test_min_pulse_2s(void){
    interlock_step(&s, true, false, 0);
    triac_cmd_t c = interlock_step(&s, false, false, 1000);  /* asked to stop early */
    TEST_ASSERT_TRUE(c.open_on);                             /* held: <2000 ms */
    c = interlock_step(&s, false, false, 2000);              /* now allowed to stop */
    TEST_ASSERT_FALSE(c.open_on);
}

/* Dead time 500 ms: after stopping, same-dir restart blocked <500 ms. */
void test_dead_time_500ms(void){
    interlock_step(&s, true, false, 0);
    interlock_step(&s, false, false, 2000);                  /* stop at 2000 */
    triac_cmd_t c = interlock_step(&s, true, false, 2300);   /* +300 ms */
    TEST_ASSERT_FALSE(c.open_on);                            /* dead-time */
    c = interlock_step(&s, true, false, 2500);               /* +500 ms */
    TEST_ASSERT_TRUE(c.open_on);
}

/* Anti-dither 10 s: reversal blocked within 10 s of previous move. */
void test_anti_dither_10s(void){
    interlock_step(&s, true, false, 0);
    interlock_step(&s, false, false, 2000);                  /* stop OPEN at 2000 */
    triac_cmd_t c = interlock_step(&s, false, true, 5000);   /* reverse @ +3 s */
    TEST_ASSERT_FALSE(c.close_on);                           /* anti-dither */
    c = interlock_step(&s, false, true, 12001);              /* @ +10.001 s */
    TEST_ASSERT_TRUE(c.close_on);
}

int main(void){
    UNITY_BEGIN();
    RUN_TEST(test_never_both);
    RUN_TEST(test_first_move_immediate);
    RUN_TEST(test_min_pulse_2s);
    RUN_TEST(test_dead_time_500ms);
    RUN_TEST(test_anti_dither_10s);
    return UNITY_END();
}
