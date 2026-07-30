#include "unity.h"
#include "ctrl_core/control.h"

static control_state_t st;
static control_cfg_t cfg;
static control_in_t in;

void setUp(void){
    control_init(&st);
    cfg.heat_setpoint = 35.0f; cfg.cool_setpoint = 18.0f; cfg.park_pos = 50.0f;
    cfg.mode_cfg = (mode_cfg_t){ 28.0f, 16.0f, 2.0f, 60000, 420000 };
    cfg.pi_cfg   = (pi_cfg_t){ 4.0f, 3.0f, 0.0f, 100.0f };
    cfg.gov_cfg  = (gov_cfg_t){ 36.0f, 16.0f, 35.0f, 17.0f };
    cfg.alarm_dwell_ms = 300000;
    in = (control_in_t){0};
    in.water_running = true; in.link_up = true;
    in.t_supply = 30.0f; in.t_source_f = 45.0f; in.t_return_f = 30.0f; in.hx_a = 30.0f;
}
void tearDown(void){}

/* water_running OFF -> park at park_pos, not regulating. */
void test_water_off_parks(void){
    in.water_running = false;
    control_out_t o = control_step(&st, &in, &cfg, 0);
    TEST_ASSERT_FALSE(o.regulating);
    TEST_ASSERT_EQUAL_FLOAT(50.0f, o.valve_target);
}

/* IDLE mode parks at park_pos while regulating. */
void test_idle_parks(void){
    in.hx_a = 22.0f;                                        /* between thresholds */
    control_out_t o = control_step(&st, &in, &cfg, 0);
    TEST_ASSERT_EQUAL(MODE_IDLE, o.mode);
    TEST_ASSERT_EQUAL_FLOAT(50.0f, o.valve_target);
}

/* HEATING after dwell drives a plausible FF target ~33 %. */
void test_heating_ff(void){
    control_step(&st, &in, &cfg, 0);
    control_out_t o = control_step(&st, &in, &cfg, 60000);  /* HEATING now */
    TEST_ASSERT_EQUAL(MODE_HEATING, o.mode);
    TEST_ASSERT_TRUE(o.valve_target > 25.0f && o.valve_target < 80.0f);
}

/* Governor: supply hot -> target forced to 0 regardless of FF. */
void test_governor_overrides(void){
    control_step(&st, &in, &cfg, 0);
    in.t_supply = 37.0f;
    control_out_t o = control_step(&st, &in, &cfg, 60000);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, o.valve_target);
}

/* Degradation park: supply + return faulted in cooling -> 10 %. */
void test_degradation_park_cooling(void){
    in.hx_a = 10.0f;                                        /* cooling */
    control_step(&st, &in, &cfg, 0);
    control_step(&st, &in, &cfg, 60000);                    /* COOLING committed */
    in.faults.supply = true; in.faults.ret = true;
    control_out_t o = control_step(&st, &in, &cfg, 70000);
    TEST_ASSERT_EQUAL(CTRL_PARK, o.strategy);
    TEST_ASSERT_EQUAL_FLOAT(10.0f, o.valve_target);
}

/* Link guard raises cooling setpoint after 30 min lost link (warmer target -> lower FF). */
void test_link_guard(void){
    in.hx_a = 8.0f; in.t_source_f = 8.0f; in.t_return_f = 24.0f; in.t_supply = 18.0f;
    control_step(&st, &in, &cfg, 0);
    control_step(&st, &in, &cfg, 60000);                    /* COOLING */
    in.link_up = false; in.link_last_seen_ms = 0;
    control_out_t guarded = control_step(&st, &in, &cfg, 1860000); /* >30 min */
    in.link_up = true;
    control_out_t normal  = control_step(&st, &in, &cfg, 1870000);
    TEST_ASSERT_TRUE(guarded.valve_target < normal.valve_target); /* warmer sp = less cold source */
}

int main(void){
    UNITY_BEGIN();
    RUN_TEST(test_water_off_parks);
    RUN_TEST(test_idle_parks);
    RUN_TEST(test_heating_ff);
    RUN_TEST(test_governor_overrides);
    RUN_TEST(test_degradation_park_cooling);
    RUN_TEST(test_link_guard);
    return UNITY_END();
}
