#include "unity.h"
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
int main(void){ UNITY_BEGIN(); RUN_TEST(test_heat_threshold_maps); RUN_TEST(test_setpoint_clamped); return UNITY_END(); }
