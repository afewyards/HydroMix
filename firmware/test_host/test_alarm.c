#include "unity.h"
#include "ctrl_core/alarm.h"

static alarm_state_t s;
void setUp(void){ alarm_init(&s); }
void tearDown(void){}

/* No alarm below the 5-min dwell even while out of bounds. */
void test_no_alarm_before_dwell(void){
    TEST_ASSERT_FALSE(alarm_supply_step(&s, 37.0f, 300000, 0));
    TEST_ASSERT_FALSE(alarm_supply_step(&s, 37.0f, 300000, 299000));
}

/* Alarm after 5 min continuously above 36.5. */
void test_alarm_after_5min(void){
    alarm_supply_step(&s, 37.0f, 300000, 0);
    TEST_ASSERT_TRUE(alarm_supply_step(&s, 37.0f, 300000, 300000));
}

/* Re-entering bounds resets the dwell (no alarm). */
void test_dwell_resets_on_return(void){
    alarm_supply_step(&s, 37.0f, 300000, 0);
    alarm_supply_step(&s, 30.0f, 300000, 100000);            /* back in band -> reset */
    TEST_ASSERT_FALSE(alarm_supply_step(&s, 37.0f, 300000, 350000)); /* only 250 s new */
}

/* Clears at 35.5 hysteresis. */
void test_clear_hysteresis(void){
    alarm_supply_step(&s, 37.0f, 300000, 0);
    alarm_supply_step(&s, 37.0f, 300000, 300000);            /* alarmed */
    TEST_ASSERT_FALSE(alarm_supply_step(&s, 35.0f, 300000, 310000)); /* <=35.5 -> clear */
}

/* Cooling dew guard: 30 min without inbound traffic in COOLING raises setpoint to 21.
 * This is the silently-dead-coordinator case -- there is no ZDO signal to key off, so
 * traffic silence alone must be enough. */
void test_link_guard_raises(void){
    TEST_ASSERT_EQUAL_FLOAT(21.0f, cooling_link_guard(18.0f, MODE_COOLING, 0, 1800000));
}

/* Recent traffic keeps the guard inactive no matter how long the device has been up. */
void test_link_guard_inactive_when_traffic_recent(void){
    TEST_ASSERT_EQUAL_FLOAT(18.0f, cooling_link_guard(18.0f, MODE_COOLING, 86400000u, 86401000u));
}

/* Inactive in heating, inside the window, at boot, and it never LOWERS a setpoint. */
void test_link_guard_inactive(void){
    TEST_ASSERT_EQUAL_FLOAT(35.0f, cooling_link_guard(35.0f, MODE_HEATING, 0, 1800000));
    TEST_ASSERT_EQUAL_FLOAT(18.0f, cooling_link_guard(18.0f, MODE_COOLING, 0, 1799000));
    TEST_ASSERT_EQUAL_FLOAT(18.0f, cooling_link_guard(18.0f, MODE_COOLING, 0, 0));
    TEST_ASSERT_EQUAL_FLOAT(24.0f, cooling_link_guard(24.0f, MODE_COOLING, 0, 1800000));
}

int main(void){
    UNITY_BEGIN();
    RUN_TEST(test_no_alarm_before_dwell);
    RUN_TEST(test_alarm_after_5min);
    RUN_TEST(test_dwell_resets_on_return);
    RUN_TEST(test_clear_hysteresis);
    RUN_TEST(test_link_guard_raises);
    RUN_TEST(test_link_guard_inactive_when_traffic_recent);
    RUN_TEST(test_link_guard_inactive);
    return UNITY_END();
}
