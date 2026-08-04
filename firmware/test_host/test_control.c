#include "unity.h"
#include "ctrl_core/control.h"
#include <math.h>

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
    cfg.deadtime_s = 0.0f;                                  /* 0 = hold disabled (old behavior) */
    in = (control_in_t){0};
    in.water_running = true; in.link_up = true;
    in.t_supply = 30.0f; in.t_source_f = 45.0f; in.t_return_f = 30.0f; in.hx_a = 30.0f;
}
void tearDown(void){}

static uint32_t warmup_heating(void){
    uint32_t t = 0;
    control_step(&st, &in, &cfg, t);
    for (int i = 0; i < 8; i++){ t += 10000; control_step(&st, &in, &cfg, t); }
    return t;   /* HEATING committed, PI active */
}

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

/* Link guard raises cooling setpoint after 30 min of traffic silence (warmer target ->
 * lower FF). Driven by last_seen_ms alone -- link_up is status-only and no longer gates
 * the guard, since a silently dead coordinator never flips it. */
void test_link_guard(void){
    in.hx_a = 8.0f; in.t_source_f = 8.0f; in.t_return_f = 24.0f; in.t_supply = 18.0f;
    control_step(&st, &in, &cfg, 0);
    control_step(&st, &in, &cfg, 60000);                    /* COOLING */
    in.link_up = false; in.link_last_seen_ms = 0;
    control_out_t guarded = control_step(&st, &in, &cfg, 1860000); /* >30 min silent */
    in.link_up = true; in.link_last_seen_ms = 1870000;      /* traffic resumes */
    control_out_t normal  = control_step(&st, &in, &cfg, 1870000);
    TEST_ASSERT_TRUE(guarded.valve_target < normal.valve_target); /* warmer sp = less cold source */
}

/* After the valve moves >0.5 %, integrator freezes and trim latches for deadtime_s. */
void test_transit_hold(void){
    cfg.deadtime_s = 30.0f;
    in.valve_pos = 40.0f; in.t_supply = 32.0f;
    uint32_t t = warmup_heating();
    control_step(&st, &in, &cfg, t += 10000);
    TEST_ASSERT_TRUE(st.pi.integ != 0.0f);            /* PI integrating */
    in.valve_pos = 42.0f;                             /* moved 2 % -> arms hold */
    control_step(&st, &in, &cfg, t += 10000);
    float integ1 = st.pi.integ;
    control_step(&st, &in, &cfg, t += 10000);         /* holding */
    TEST_ASSERT_TRUE(st.holding);
    TEST_ASSERT_EQUAL_FLOAT(integ1, st.pi.integ);     /* frozen */
    control_step(&st, &in, &cfg, t += 20000);         /* deadtime elapsed */
    TEST_ASSERT_FALSE(st.holding);
    TEST_ASSERT_TRUE(st.pi.integ != integ1);          /* integrating again */
}

/* Early release when the pipe answers (>0.25 K supply move since hold start). */
void test_transit_hold_early_release(void){
    cfg.deadtime_s = 60.0f;
    in.valve_pos = 40.0f; in.t_supply = 32.0f;
    uint32_t t = warmup_heating();
    in.valve_pos = 43.0f;                             /* arm */
    control_step(&st, &in, &cfg, t += 10000);
    float integ1 = st.pi.integ;
    in.t_supply = 32.4f;                              /* pipe answered */
    control_step(&st, &in, &cfg, t += 10000);
    TEST_ASSERT_FALSE(st.holding);
    TEST_ASSERT_TRUE(st.pi.integ != integ1);
}

/* Governor overrides regardless of hold state -- and the hold itself stays
 * latched throughout, proving the governor clamp is applied on top of (not
 * instead of) the frozen transit-hold trim. */
void test_governor_bypasses_hold(void){
    cfg.deadtime_s = 60.0f;
    in.valve_pos = 40.0f; in.t_supply = 35.9f;         /* stays under gov_high 36 */
    uint32_t t = warmup_heating();                     /* err = 35-35.9 = -0.9 K, fine for heating */
    in.valve_pos = 43.0f;                              /* moved 3 % -> arms hold */
    control_step(&st, &in, &cfg, t += 10000);
    TEST_ASSERT_TRUE(st.holding);
    in.t_supply = 36.1f;                               /* over gov_high; Delta 0.2 < release 0.25 -> hold stays latched */
    control_out_t o = control_step(&st, &in, &cfg, t += 10000);
    TEST_ASSERT_TRUE(st.holding);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, o.valve_target);
}

/* FF stays live during the hold; only the trim is latched. */
void test_ff_live_during_hold(void){
    cfg.deadtime_s = 120.0f;
    in.valve_pos = 40.0f; in.t_supply = 34.9f;        /* near-zero err */
    uint32_t t = warmup_heating();
    in.valve_pos = 43.0f;                             /* arm */
    control_out_t a = control_step(&st, &in, &cfg, t += 10000);
    in.t_return_f = 25.0f;                            /* FF input shifts while holding */
    control_out_t b = control_step(&st, &in, &cfg, t += 10000);
    TEST_ASSERT_TRUE(st.holding);
    TEST_ASSERT_TRUE(fabsf(b.valve_target - a.valve_target) > 1.0f);
}

/* CTRL_PI_ONLY gets full trim authority (park_pos +/- full 0..100 span), not
 * the default +/-20 % clamp: a sustained error must drive the target well
 * past what a +/-20 clamp around park_pos (50) could ever reach (max 70). */
void test_pi_only_full_authority(void){
    cfg.deadtime_s = 0.0f;                              /* hold releases every cycle -- simplest */
    uint32_t t = warmup_heating();
    in.faults.source = true;                            /* source-or-return fault -> CTRL_PI_ONLY */
    in.t_supply = 30.0f;                                /* err = 35-30 = +5 K, sustained */
    in.valve_pos = cfg.park_pos;
    control_out_t o = {0};
    for (int i = 0; i < 30; i++){
        t += 10000;
        o = control_step(&st, &in, &cfg, t);
        TEST_ASSERT_EQUAL(CTRL_PI_ONLY, o.strategy);
        in.valve_pos = o.valve_target;                  /* feed back -> transit hold arms/releases naturally */
    }
    TEST_ASSERT_TRUE(o.valve_target > 75.0f);
}

/* A trim latched while holding under CTRL_PI_ONLY (trim_max=100, no cap) must
 * be re-bounded to the CURRENT strategy's authority if the strategy flips
 * back to CTRL_FULL (probe recovery) while the hold is still latched --
 * otherwise a PI_ONLY-sized trim gets replayed straight onto the FF baseline,
 * blowing through the CTRL_FULL +/-20 clamp. */
void test_hold_trim_clamped_on_strategy_flip(void){
    cfg.deadtime_s = 0.0f;                              /* hold releases every cycle during buildup */
    uint32_t t = warmup_heating();                      /* CTRL_FULL; FF = (35-30)/(45-30)*100 = 33.33 % */
    in.faults.source = true;                            /* -> CTRL_PI_ONLY, full trim authority */
    in.t_supply = 30.0f;                                /* err = +5 K, sustained */
    in.valve_pos = cfg.park_pos;
    control_out_t o = {0};
    for (int i = 0; i < 6; i++){                         /* build a trim well past +/-20 */
        t += 10000;
        o = control_step(&st, &in, &cfg, t);
        in.valve_pos = o.valve_target;
    }
    TEST_ASSERT_TRUE(st.latched_trim > 20.0f);

    cfg.deadtime_s = 60.0f;                             /* this arm must survive the strategy flip below */
    t += 10000;
    o = control_step(&st, &in, &cfg, t);                /* still CTRL_PI_ONLY */
    in.valve_pos = o.valve_target;
    TEST_ASSERT_TRUE(st.holding);
    TEST_ASSERT_TRUE(st.latched_trim > 20.0f);

    in.faults.source = false;                           /* probe recovers -> back to CTRL_FULL, still holding */
    t += 10000;
    o = control_step(&st, &in, &cfg, t);
    TEST_ASSERT_EQUAL(CTRL_FULL, o.strategy);
    TEST_ASSERT_TRUE(st.holding);                       /* hold survived the strategy flip */
    /* Replayed trim clamped to +/-20 around the FF baseline (33.33 %), not the
     * unclamped PI_ONLY trim (would be ~70 % without the fix). */
    TEST_ASSERT_TRUE(fabsf(o.valve_target - 53.3333f) < 0.05f);
}

/* Low-authority FF (source ~= return) in CTRL_FULL must keep the loop CLOSED: the PI goes
 * on regulating on measured supply error, and the valve is not pinned.
 *
 * 1.5.0 did the opposite -- ff.frozen fed pi_step's freeze path, which discards the whole
 * PI output (both P and I), so the valve took a bare frozen FF constant and the loop ran
 * open. Live on 2026-08-04 that latched the valve at 55.7 % for 92 minutes while supply sat
 * 2.7 K above setpoint. t_supply is a valid measurement whatever the FF denominator does. */
void test_ff_low_authority_keeps_pi_authority(void){
    /* t_supply must move off the fixture default (30, err=+5 K): p alone (kp*err =
     * 4*5 = 20) already equals trim_max (20) there, so pi.c's anti-windup ("pushing")
     * blocks the integrator every cycle regardless of anything else -- same reason
     * test_transit_hold and friends all override t_supply to 32+ instead of leaving
     * it at 30. valve_pos is set to its held value BEFORE warmup and never changes
     * again, so the transit hold (movement > TRANSIT_MOVE_PCT) never arms -- nothing
     * to release, so it can't suppress pi_step during buildup either. */
    in.t_supply = 32.0f;                    /* err = 35-32 = 3 K; p = 12 < trim_max 20 */
    in.valve_pos = 35.0f;                   /* != cfg.park_pos (50) -> held vs parked distinguishable */
    uint32_t t = warmup_heating();          /* CTRL_FULL, PI active, FF has authority */
    control_step(&st, &in, &cfg, t += 10000);   /* one more cycle, same as test_transit_hold */
    TEST_ASSERT_TRUE(st.pi.integ != 0.0f);
    float integ_before = st.pi.integ;

    in.t_source_f = 30.2f;                  /* |30.2-30|=0.2 < 2K -> denominator gets limited */
    control_step(&st, &in, &cfg, t += 10000);
    control_out_t o = control_step(&st, &in, &cfg, t += 10000);

    /* Not pinned to the current position -- that pin IS the bug. Heating demand with the
     * source converged on the return saturates the limited ratio toward the source. */
    TEST_ASSERT_TRUE(fabsf(o.valve_target - in.valve_pos) > 1.0f);
    TEST_ASSERT_TRUE(o.valve_target > in.valve_pos);

    /* THE LOOP MUST STILL BE CLOSED ON MEASURED SUPPLY. This is the precise 1.5.0
     * regression: ff.frozen fed pi_step's freeze path, which returns pos_ff untouched, so
     * the valve took a bare FF constant and supply error had no effect at all.
     *
     * Overshoot the setpoint by 5 K and the target must come DOWN off the FF's rail. Testing
     * it this way rather than by inspecting trim or integ, because here the limited ratio
     * saturates pos_ff at out_max, where trim is structurally 0 and the integrator sits at
     * its anti-windup clamp -- both correct, and both blind to whether the loop is alive. */
    float railed = o.valve_target;
    in.t_supply = 40.0f;                    /* 5 K above the 35 C heat setpoint */
    for (int i = 0; i < 6; i++) o = control_step(&st, &in, &cfg, t += 10000);
    TEST_ASSERT_TRUE(o.valve_target < railed - 1.0f);
    (void)integ_before;
}

/* Supply probe faulted: t_supply is stale or BSS-zero, so the freeze alarm must not
 * latch on it. 0.0 C is <= ALARM_SUPPLY_LOW and would otherwise alarm after dwell. */
void test_supply_fault_blocks_false_freeze_alarm(void){
    in.faults.supply = true;
    in.t_supply = 0.0f;
    control_out_t o = control_step(&st, &in, &cfg, 0);
    TEST_ASSERT_FALSE(o.supply_alarm);
    o = control_step(&st, &in, &cfg, 600000);        /* 10 min, twice the 5 min dwell */
    TEST_ASSERT_FALSE(o.supply_alarm);
}

/* A real alarm must survive the probe subsequently failing -- a frozen in-band value
 * must not clear it. */
void test_supply_fault_holds_existing_alarm(void){
    in.t_supply = 37.0f;
    control_step(&st, &in, &cfg, 0);
    control_out_t o = control_step(&st, &in, &cfg, 300000);
    TEST_ASSERT_TRUE(o.supply_alarm);                /* dwell met -> alarmed */
    in.faults.supply = true; in.t_supply = 30.0f;    /* frozen, in the clear band */
    o = control_step(&st, &in, &cfg, 310000);
    TEST_ASSERT_TRUE(o.supply_alarm);                /* must NOT falsely clear */
}

/* The excursion dwell re-arms clean on fault clear: an out-of-bounds reading right
 * after recovery must serve a fresh full dwell, not inherit stale elapsed time. */
void test_supply_fault_rearms_dwell_on_recovery(void){
    in.faults.supply = true;
    in.t_supply = 37.0f;
    control_step(&st, &in, &cfg, 0);
    control_step(&st, &in, &cfg, 600000);            /* 10 min faulted, no dwell accrues */
    in.faults.supply = false;
    control_out_t o = control_step(&st, &in, &cfg, 610000);
    TEST_ASSERT_FALSE(o.supply_alarm);               /* dwell restarts here */
    o = control_step(&st, &in, &cfg, 890000);        /* +280 s, still < 300 s */
    TEST_ASSERT_FALSE(o.supply_alarm);
    o = control_step(&st, &in, &cfg, 910001);        /* +300.001 s -> alarm */
    TEST_ASSERT_TRUE(o.supply_alarm);
}

int main(void){
    UNITY_BEGIN();
    RUN_TEST(test_water_off_parks);
    RUN_TEST(test_idle_parks);
    RUN_TEST(test_heating_ff);
    RUN_TEST(test_governor_overrides);
    RUN_TEST(test_degradation_park_cooling);
    RUN_TEST(test_link_guard);
    RUN_TEST(test_transit_hold);
    RUN_TEST(test_transit_hold_early_release);
    RUN_TEST(test_governor_bypasses_hold);
    RUN_TEST(test_ff_live_during_hold);
    RUN_TEST(test_pi_only_full_authority);
    RUN_TEST(test_hold_trim_clamped_on_strategy_flip);
    RUN_TEST(test_ff_low_authority_keeps_pi_authority);
    RUN_TEST(test_supply_fault_blocks_false_freeze_alarm);
    RUN_TEST(test_supply_fault_holds_existing_alarm);
    RUN_TEST(test_supply_fault_rearms_dwell_on_recovery);
    return UNITY_END();
}
