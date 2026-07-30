#include "unity.h"
#include "ctrl_core/pi.h"

static pi_state_t s;
static const pi_cfg_t cfg = { .kp = 2.0f, .ki = 6.0f, .out_min = 0.0f, .out_max = 100.0f };
void setUp(void){ pi_init(&s); }
void tearDown(void){}

/* Heating positive error raises output above FF. */
void test_heating_trim(void){
    float out = pi_step(&s, 50.0f, 3.0f, false, false, 10.0f, &cfg); /* err=+3 */
    TEST_ASSERT_TRUE(out > 50.0f);
}

/* Cooling sign inversion: same raw err flips direction. */
void test_cooling_sign(void){
    float out = pi_step(&s, 50.0f, 3.0f, true, false, 10.0f, &cfg);  /* effective err=-3 */
    TEST_ASSERT_TRUE(out < 50.0f);
}

/* Conditional anti-windup: at max clamp with pushing error, integrator must not grow. */
void test_anti_windup(void){
    for (int i=0;i<20;i++) pi_step(&s, 100.0f, 10.0f, false, false, 10.0f, &cfg); /* saturated high */
    float integ_saturated = s.integ;
    pi_step(&s, 100.0f, 10.0f, false, false, 10.0f, &cfg);
    TEST_ASSERT_EQUAL_FLOAT(integ_saturated, s.integ);       /* frozen */
}

/* Mode change: PI off (FF only) for 3 cycles, integ reset. */
void test_mode_change_hold_3(void){
    pi_step(&s, 50.0f, 5.0f, false, false, 10.0f, &cfg);            /* build integ */
    pi_mode_change(&s);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s.integ);
    TEST_ASSERT_EQUAL_FLOAT(50.0f, pi_step(&s, 50.0f, 5.0f, false, false, 10.0f, &cfg)); /* 1 */
    TEST_ASSERT_EQUAL_FLOAT(50.0f, pi_step(&s, 50.0f, 5.0f, false, false, 10.0f, &cfg)); /* 2 */
    TEST_ASSERT_EQUAL_FLOAT(50.0f, pi_step(&s, 50.0f, 5.0f, false, false, 10.0f, &cfg)); /* 3 */
    TEST_ASSERT_TRUE(pi_step(&s, 50.0f, 5.0f, false, false, 10.0f, &cfg) > 50.0f);       /* 4: active */
}

/* Freeze: output = FF, integrator untouched. */
void test_freeze(void){
    float before = s.integ;
    float out = pi_step(&s, 42.0f, 8.0f, false, true, 10.0f, &cfg);
    TEST_ASSERT_EQUAL_FLOAT(42.0f, out);
    TEST_ASSERT_EQUAL_FLOAT(before, s.integ);
}

/* ki is %/K per MINUTE, scaled by actual dt. */
void test_ki_dt_scaling(void){
    pi_step(&s, 50.0f, 1.0f, false, false, 30.0f, &cfg);  /* integ += 6*1*(30/60) = 3 */
    TEST_ASSERT_EQUAL_FLOAT(3.0f, s.integ);
    pi_step(&s, 50.0f, 1.0f, false, false, 60.0f, &cfg);  /* += 6*1*1 = 6 */
    TEST_ASSERT_EQUAL_FLOAT(9.0f, s.integ);
}

int main(void){
    UNITY_BEGIN();
    RUN_TEST(test_heating_trim);
    RUN_TEST(test_cooling_sign);
    RUN_TEST(test_anti_windup);
    RUN_TEST(test_mode_change_hold_3);
    RUN_TEST(test_freeze);
    RUN_TEST(test_ki_dt_scaling);
    return UNITY_END();
}
