#include "unity.h"
#include <math.h>
#include <stdint.h>
#include "ctrl_core/config_map.h"

void setUp(void){} void tearDown(void){}

void test_heat_threshold_maps(void){
    tunable_cfg_t c; tunable_cfg_defaults(&c);
    float v = 30.0f;
    tunable_apply(&c, TUNABLE_HEAT_THRESHOLD, &v);
    TEST_ASSERT_EQUAL_FLOAT(30.0f, c.heat_threshold);
}
void test_setpoint_clamped(void){
    tunable_cfg_t c; tunable_cfg_defaults(&c);
    float v = 99.0f;
    tunable_apply(&c, TUNABLE_HEAT_SETPOINT, &v);
    TEST_ASSERT_EQUAL_FLOAT(35.0f, c.heat_setpoint);   /* clamp <=35 */
}
void test_new_defaults(void){
    tunable_cfg_t c; tunable_cfg_defaults(&c);
    TEST_ASSERT_EQUAL_FLOAT(2.8f,  c.kp);
    TEST_ASSERT_EQUAL_FLOAT(0.9f,  c.ki);
    TEST_ASSERT_EQUAL_FLOAT(30.0f, c.deadtime_s);
    TEST_ASSERT_EQUAL_FLOAT(0.25f, c.pi_deadband_k);
}

void test_gain_bounds(void){
    tunable_cfg_t c; tunable_cfg_defaults(&c);
    float v;
    v = 99.0f;  tunable_apply(&c, TUNABLE_KP, &v);          TEST_ASSERT_EQUAL_FLOAT(15.0f,  c.kp);
    v = -1.0f;  tunable_apply(&c, TUNABLE_KI, &v);          TEST_ASSERT_EQUAL_FLOAT(0.0f,   c.ki);
    v = 999.0f; tunable_apply(&c, TUNABLE_DEADTIME_S, &v);  TEST_ASSERT_EQUAL_FLOAT(120.0f, c.deadtime_s);
    v = 5.0f;   tunable_apply(&c, TUNABLE_PI_DEADBAND, &v); TEST_ASSERT_EQUAL_FLOAT(1.0f,   c.pi_deadband_k);
}
void test_nan_rejected(void){
    tunable_cfg_t c; tunable_cfg_defaults(&c);
    float nanv = NAN;
    tunable_apply(&c, TUNABLE_KP, &nanv);
    TEST_ASSERT_EQUAL_FLOAT(2.8f, c.kp);              /* unchanged */
    tunable_apply(&c, TUNABLE_HEAT_THRESHOLD, &nanv);
    TEST_ASSERT_EQUAL_FLOAT(28.0f, c.heat_threshold); /* unchanged */
}
void test_threshold_and_travel_bounds(void){
    tunable_cfg_t c; tunable_cfg_defaults(&c);
    float v = 200.0f; tunable_apply(&c, TUNABLE_HEAT_THRESHOLD, &v);
    TEST_ASSERT_EQUAL_FLOAT(60.0f, c.heat_threshold);
    uint32_t t = 100000; tunable_apply(&c, TUNABLE_TRAVEL_TIME_S, &t);
    TEST_ASSERT_EQUAL(600, c.travel_time_s);
    t = 5; tunable_apply(&c, TUNABLE_TRAVEL_TIME_S, &t);
    TEST_ASSERT_EQUAL(30, c.travel_time_s);
}
int main(void){
    UNITY_BEGIN();
    RUN_TEST(test_heat_threshold_maps);
    RUN_TEST(test_setpoint_clamped);
    RUN_TEST(test_new_defaults);
    RUN_TEST(test_gain_bounds);
    RUN_TEST(test_nan_rejected);
    RUN_TEST(test_threshold_and_travel_bounds);
    return UNITY_END();
}
