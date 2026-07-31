#include "unity.h"
#include "ctrl_core/ota_gate.h"

void setUp(void){} void tearDown(void){}

/* Fires exactly once, on the call that reaches the threshold -- never before. */
void test_fires_exactly_at_threshold(void){
    uint32_t c = 0;
    for (uint32_t i = 1; i < 12; ++i)
        TEST_ASSERT_FALSE(ota_gate_step(&c, 12));
    TEST_ASSERT_TRUE(ota_gate_step(&c, 12));          /* 12th call */
    TEST_ASSERT_EQUAL_UINT32(12, c);
}

/* Never fires again, and the counter saturates rather than wrapping. */
void test_never_fires_again_and_saturates(void){
    uint32_t c = 0;
    for (int i = 0; i < 12; ++i) ota_gate_step(&c, 12);
    for (int i = 0; i < 1000; ++i) TEST_ASSERT_FALSE(ota_gate_step(&c, 12));
    TEST_ASSERT_EQUAL_UINT32(12, c);
}

/* Threshold 1 fires on the very first call, then never again. */
void test_threshold_one_fires_immediately(void){
    uint32_t c = 0;
    TEST_ASSERT_TRUE(ota_gate_step(&c, 1));
    TEST_ASSERT_FALSE(ota_gate_step(&c, 1));
    TEST_ASSERT_EQUAL_UINT32(1, c);
}

/* Threshold 0 must never fire -- a mis-set constant must not validate an image
 * instantly (that would silently disable rollback protection). */
void test_threshold_zero_never_fires(void){
    uint32_t c = 0;
    for (int i = 0; i < 10; ++i) TEST_ASSERT_FALSE(ota_gate_step(&c, 0));
    TEST_ASSERT_EQUAL_UINT32(0, c);
}

int main(void){
    UNITY_BEGIN();
    RUN_TEST(test_fires_exactly_at_threshold);
    RUN_TEST(test_never_fires_again_and_saturates);
    RUN_TEST(test_threshold_one_fires_immediately);
    RUN_TEST(test_threshold_zero_never_fires);
    return UNITY_END();
}
