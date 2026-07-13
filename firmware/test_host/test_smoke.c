#include "unity.h"
#include "ctrl_core/types.h"

void setUp(void){} void tearDown(void){}

void test_clamp(void){
    TEST_ASSERT_EQUAL_FLOAT(0.0f,  ctrl_clampf(-5.0f, 0.0f, 100.0f));
    TEST_ASSERT_EQUAL_FLOAT(100.0f,ctrl_clampf(150.0f,0.0f, 100.0f));
    TEST_ASSERT_EQUAL_FLOAT(50.0f, ctrl_clampf(50.0f, 0.0f, 100.0f));
}

int main(void){ UNITY_BEGIN(); RUN_TEST(test_clamp); return UNITY_END(); }
