#include "unity.h"
#include "ctrl_core/governor.h"

static gov_state_t s;
static const gov_cfg_t cfg = { .gov_high = 36.0f, .gov_low = 16.0f, .rel_high = 35.0f, .rel_low = 17.0f };
void setUp(void){ gov_init(&s); }
void tearDown(void){}

/* Normal supply: passthrough. */
void test_passthrough(void){
    TEST_ASSERT_EQUAL_FLOAT(60.0f, gov_step(&s, 60.0f, 30.0f, &cfg));
}

/* Supply > 36: override to 0 %. */
void test_high_override(void){
    TEST_ASSERT_EQUAL_FLOAT(0.0f, gov_step(&s, 60.0f, 36.5f, &cfg));
    TEST_ASSERT_TRUE(s.active);
}

/* Supply < 16: override to 0 %. */
void test_low_override(void){
    TEST_ASSERT_EQUAL_FLOAT(0.0f, gov_step(&s, 60.0f, 15.0f, &cfg));
}

/* Latches until supply re-enters [17,35]: at 35.5 still governing. */
void test_hysteresis_release(void){
    gov_step(&s, 60.0f, 37.0f, &cfg);                        /* active */
    TEST_ASSERT_EQUAL_FLOAT(0.0f, gov_step(&s, 60.0f, 35.5f, &cfg)); /* still active */
    TEST_ASSERT_EQUAL_FLOAT(60.0f, gov_step(&s, 60.0f, 34.0f, &cfg)); /* released */
    TEST_ASSERT_FALSE(s.active);
}

int main(void){
    UNITY_BEGIN();
    RUN_TEST(test_passthrough);
    RUN_TEST(test_high_override);
    RUN_TEST(test_low_override);
    RUN_TEST(test_hysteresis_release);
    return UNITY_END();
}
