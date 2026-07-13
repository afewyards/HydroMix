#include "unity.h"
#include "ctrl_core/mode_detect.h"

static mode_detect_state_t s;
static const mode_cfg_t cfg = {
    .heat_threshold = 28.0f, .cool_threshold = 16.0f, .hysteresis = 2.0f,
    .enter_dwell_ms = 60000, .leave_dwell_ms = 420000,
};
void setUp(void){ mode_detect_init(&s); }
void tearDown(void){}

/* Boot is IDLE; invalid HX-A holds IDLE. */
void test_boot_idle(void){
    TEST_ASSERT_EQUAL(MODE_IDLE, mode_detect_step(&s, 99.0f, false, &cfg, 0));
}

/* Enter HEATING needs hx>=28 held for 60 s. */
void test_enter_heating_dwell_60s(void){
    TEST_ASSERT_EQUAL(MODE_IDLE,    mode_detect_step(&s, 30.0f, true, &cfg, 0));
    TEST_ASSERT_EQUAL(MODE_IDLE,    mode_detect_step(&s, 30.0f, true, &cfg, 59000));
    TEST_ASSERT_EQUAL(MODE_HEATING, mode_detect_step(&s, 30.0f, true, &cfg, 60000));
}

/* A 40 s dip below threshold must NOT leave heating (needs 7 min). */
void test_leave_heating_dwell_7min(void){
    mode_detect_step(&s, 30.0f, true, &cfg, 0);
    mode_detect_step(&s, 30.0f, true, &cfg, 60000);           /* HEATING */
    TEST_ASSERT_EQUAL(MODE_HEATING, mode_detect_step(&s, 20.0f, true, &cfg, 100000)); /* dip */
    TEST_ASSERT_EQUAL(MODE_HEATING, mode_detect_step(&s, 20.0f, true, &cfg, 140000)); /* +40 s */
    TEST_ASSERT_EQUAL(MODE_IDLE,    mode_detect_step(&s, 20.0f, true, &cfg, 520001)); /* >7 min */
}

/* Hysteresis 2K: at 26.5 (26<hx<28) still HEATING (leave band is <26). */
void test_hysteresis(void){
    mode_detect_step(&s, 30.0f, true, &cfg, 0);
    mode_detect_step(&s, 30.0f, true, &cfg, 60000);
    TEST_ASSERT_EQUAL(MODE_HEATING, mode_detect_step(&s, 26.5f, true, &cfg, 600000));
}

/* HX-A fault holds last known mode. */
void test_fault_holds_mode(void){
    mode_detect_step(&s, 30.0f, true, &cfg, 0);
    mode_detect_step(&s, 30.0f, true, &cfg, 60000);           /* HEATING */
    TEST_ASSERT_EQUAL(MODE_HEATING, mode_detect_step(&s, 0.0f, false, &cfg, 100000));
}

int main(void){
    UNITY_BEGIN();
    RUN_TEST(test_boot_idle);
    RUN_TEST(test_enter_heating_dwell_60s);
    RUN_TEST(test_leave_heating_dwell_7min);
    RUN_TEST(test_hysteresis);
    RUN_TEST(test_fault_holds_mode);
    return UNITY_END();
}
