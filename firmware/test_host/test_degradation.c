#include "unity.h"
#include "ctrl_core/degradation.h"

void setUp(void){} void tearDown(void){}

/* Rung 1: all OK -> FULL. */
void test_all_ok(void){
    sensor_faults_t f = {0};
    degradation_out_t o = degradation_eval(&f, MODE_HEATING, 50.0f);
    TEST_ASSERT_EQUAL(CTRL_FULL, o.strategy);
    TEST_ASSERT_EQUAL_UINT16(0, o.alarm_bits);
}

/* Rung 2: supply faulted -> FF only; cooling adds conservative bias. */
void test_supply_fault_cooling_bias(void){
    sensor_faults_t f = { .supply = true };
    degradation_out_t o = degradation_eval(&f, MODE_COOLING, 50.0f);
    TEST_ASSERT_EQUAL(CTRL_FF_ONLY, o.strategy);
    TEST_ASSERT_EQUAL_FLOAT(-COOLING_FF_BIAS_PCT, o.ff_bias_pct); /* toward recirc */
    TEST_ASSERT_TRUE(o.alarm_bits & FAULT_BIT_SUPPLY);
}

/* Rung 2 heating: FF only, no bias. */
void test_supply_fault_heating(void){
    sensor_faults_t f = { .supply = true };
    degradation_out_t o = degradation_eval(&f, MODE_HEATING, 50.0f);
    TEST_ASSERT_EQUAL(CTRL_FF_ONLY, o.strategy);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, o.ff_bias_pct);
}

/* Rung 3: source faulted -> PI only. */
void test_source_fault(void){
    sensor_faults_t f = { .source = true };
    degradation_out_t o = degradation_eval(&f, MODE_HEATING, 50.0f);
    TEST_ASSERT_EQUAL(CTRL_PI_ONLY, o.strategy);
}

/* Rung 4 heating: supply + return -> park at park_pos. */
void test_park_heating(void){
    sensor_faults_t f = { .supply = true, .ret = true };
    degradation_out_t o = degradation_eval(&f, MODE_HEATING, 50.0f);
    TEST_ASSERT_EQUAL(CTRL_PARK, o.strategy);
    TEST_ASSERT_EQUAL_FLOAT(50.0f, o.park_pos);
}

/* Rung 4 cooling: supply + source -> park at 10 % (dew-point safe). */
void test_park_cooling_10pct(void){
    sensor_faults_t f = { .supply = true, .source = true };
    degradation_out_t o = degradation_eval(&f, MODE_COOLING, 50.0f);
    TEST_ASSERT_EQUAL(CTRL_PARK, o.strategy);
    TEST_ASSERT_EQUAL_FLOAT(10.0f, o.park_pos);
}

/* HX-B fault: alarm only, still FULL. */
void test_hxb_alarm_only(void){
    sensor_faults_t f = { .hx_b = true };
    degradation_out_t o = degradation_eval(&f, MODE_HEATING, 50.0f);
    TEST_ASSERT_EQUAL(CTRL_FULL, o.strategy);
    TEST_ASSERT_TRUE(o.alarm_bits & FAULT_BIT_HX_B);
}

int main(void){
    UNITY_BEGIN();
    RUN_TEST(test_all_ok);
    RUN_TEST(test_supply_fault_cooling_bias);
    RUN_TEST(test_supply_fault_heating);
    RUN_TEST(test_source_fault);
    RUN_TEST(test_park_heating);
    RUN_TEST(test_park_cooling_10pct);
    RUN_TEST(test_hxb_alarm_only);
    return UNITY_END();
}
