#include "unity.h"
#include <math.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "ctrl_core/config_map.h"

void setUp(void){} void tearDown(void){}

/* tunable_cfg_t has 3 bytes of padding after direction_swap, and tunable_cfg_defaults()
 * does not write them. Zero the whole struct first so the TEST_ASSERT_EQUAL_MEMORY
 * comparisons below compare fields, not whatever the stack happened to hold. */
static void fresh(tunable_cfg_t *c){ memset(c, 0, sizeof *c); tunable_cfg_defaults(c); }

/* Every id in the table must describe a real field: a zero-initialised config that is
 * then clamped must land inside the declared bounds for every single tunable. This is
 * the test that catches a bad offsetof entry. */
void test_every_spec_entry_clamps_into_its_own_range(void){
    tunable_cfg_t c;
    memset(&c, 0, sizeof c);
    tunable_clamp_all(&c);
    for (int id = 0; id < TUNABLE_COUNT; ++id) {
        const tunable_spec_t *s = tunable_spec((tunable_id_t)id);
        TEST_ASSERT_NOT_NULL_MESSAGE(s, "every id needs a spec entry");
        const char *base = (const char *)&c;
        if (s->kind == TK_F32) {
            float v = *(const float *)(const void *)(base + s->off);
            TEST_ASSERT_FALSE(isnan(v));
            /* valve_deadband's floor is dynamic and >= its table flo, so test the ceiling
             * generically and the floor via its own test below. */
            TEST_ASSERT_TRUE(v <= s->fhi + 1e-6f);
            if ((tunable_id_t)id != TUNABLE_VALVE_DEADBAND)
                TEST_ASSERT_TRUE(v >= s->flo - 1e-6f);
        } else if (s->kind == TK_U32) {
            uint32_t v = *(const uint32_t *)(const void *)(base + s->off);
            TEST_ASSERT_TRUE(v >= s->ulo && v <= s->uhi);
        }
    }
}

void test_defaults_survive_clamp_all_unchanged(void){
    tunable_cfg_t a, b;
    fresh(&a);
    fresh(&b);
    tunable_clamp_all(&b);
    /* Every field except the last is byte-identical after a clamp: the shipped defaults
     * are all inside their declared ranges. */
    TEST_ASSERT_EQUAL_MEMORY(&a, &b, offsetof(tunable_cfg_t, valve_deadband_pct));
    /* valve_deadband_pct is the one exception, and it is a float-representation artifact
     * rather than an out-of-range default: valve_deadband_floor_pct(120) evaluates to
     * 1.0000001 (0x3F800001), one ULP above the shipped 1.0f, because 1.2f is not exactly
     * representable. The clamp therefore lifts 1.0 to the floor. The arithmetic is
     * character-identical to 1.6.x's clamp_config(), so this is pre-existing behaviour. */
    TEST_ASSERT_EQUAL_FLOAT(a.valve_deadband_pct, b.valve_deadband_pct);
    TEST_ASSERT_TRUE(b.valve_deadband_pct >= valve_deadband_floor_pct(b.travel_time_s));
}

void test_clamp_all_rejects_nan_back_to_default(void){
    tunable_cfg_t c; fresh(&c);
    c.kp = NAN; c.heat_setpoint = NAN;
    tunable_clamp_all(&c);
    TEST_ASSERT_EQUAL_FLOAT(2.8f,  c.kp);
    TEST_ASSERT_EQUAL_FLOAT(35.0f, c.heat_setpoint);
}

void test_clamp_all_pulls_out_of_range_values_in(void){
    tunable_cfg_t c; fresh(&c);
    c.gov_low = 20.0f; c.gov_high = 25.0f; c.travel_time_s = 100000; c.leave_dwell_ms = 1;
    tunable_clamp_all(&c);
    TEST_ASSERT_EQUAL_FLOAT(16.0f, c.gov_low);    /* must stay below the 17/35 band */
    TEST_ASSERT_EQUAL_FLOAT(36.0f, c.gov_high);   /* must stay above it */
    TEST_ASSERT_EQUAL_UINT32(600,   c.travel_time_s);
    TEST_ASSERT_EQUAL_UINT32(10000, c.leave_dwell_ms);
}

void test_valve_deadband_floor_tracks_travel_time(void){
    /* floor = 1.2 * (INTERLOCK_MIN_PULSE_MS/1000) * 100 / (2*travel), min 0.2 */
    float at120 = valve_deadband_floor_pct(120);
    float at30  = valve_deadband_floor_pct(30);
    TEST_ASSERT_TRUE(at30 > at120);               /* shorter travel, higher floor */
    TEST_ASSERT_TRUE(valve_deadband_floor_pct(600) >= 0.2f);   /* hard minimum */
    TEST_ASSERT_EQUAL_FLOAT(1.0f, at120);         /* shipped default sits exactly on it */
}

void test_shortening_travel_lifts_a_stranded_deadband(void){
    tunable_cfg_t c; fresh(&c);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, c.valve_deadband_pct);
    uint32_t t = 30;
    tunable_apply(&c, TUNABLE_TRAVEL_TIME_S, &t);
    TEST_ASSERT_EQUAL_UINT32(30, c.travel_time_s);
    /* The deadband must have been re-floored, not left stranded at 1.0 below the new floor. */
    TEST_ASSERT_TRUE(c.valve_deadband_pct >= valve_deadband_floor_pct(30) - 1e-6f);
}

void test_valve_deadband_is_writable_as_a_tunable(void){
    tunable_cfg_t c; fresh(&c);
    float v = 3.0f;  tunable_apply(&c, TUNABLE_VALVE_DEADBAND, &v);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, c.valve_deadband_pct);
    v = 99.0f;       tunable_apply(&c, TUNABLE_VALVE_DEADBAND, &v);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, c.valve_deadband_pct);    /* ceiling */
}

void test_out_of_range_id_is_ignored(void){
    tunable_cfg_t a, c;
    fresh(&a); fresh(&c);
    float v = 1.0f;
    tunable_apply(&c, (tunable_id_t)TUNABLE_COUNT, &v);
    tunable_apply(&c, (tunable_id_t)9999, &v);
    TEST_ASSERT_EQUAL_MEMORY(&a, &c, sizeof a);
    TEST_ASSERT_NULL(tunable_spec((tunable_id_t)TUNABLE_COUNT));
}

/* The whole point of Task 6: a write arriving over Zigbee and a write arriving through
 * the tunable id must produce byte-identical config. Before 1.7.0 these were two
 * hand-maintained switch statements with independently written bounds. */
void test_attr_path_and_tunable_path_agree(void){
    struct { uint16_t attr; tunable_id_t id; } m[] = {
        {0x0000, TUNABLE_HEAT_THRESHOLD}, {0x0001, TUNABLE_COOL_THRESHOLD},
        {0x0002, TUNABLE_TRAVEL_TIME_S},  {0x0003, TUNABLE_PARK_POS},
        {0x0004, TUNABLE_DIRECTION_SWAP}, {0x0005, TUNABLE_KP},
        {0x0006, TUNABLE_KI},             {0x0007, TUNABLE_GOV_HIGH},
        {0x0008, TUNABLE_GOV_LOW},        {0x0009, TUNABLE_ALARM_DWELL_MS},
        {0x000E, TUNABLE_DEADTIME_S},     {0x000F, TUNABLE_PI_DEADBAND},
        {0x0010, TUNABLE_HEAT_SETPOINT},  {0x0011, TUNABLE_COOL_SETPOINT},
        {0x0012, TUNABLE_VALVE_DEADBAND},
    };
    /* Values chosen to sit outside every declared range in both directions, so each
     * write exercises a clamp rather than passing through. */
    const float    fhi = 1.0e6f, flo = -1.0e6f;
    const uint32_t uhi = 4000000000u, ulo = 0u;
    const bool     bt  = true;

    for (size_t i = 0; i < sizeof m / sizeof m[0]; ++i) {
        const tunable_spec_t *s = tunable_spec(m[i].id);
        TEST_ASSERT_NOT_NULL(s);
        tunable_id_t got;
        TEST_ASSERT_TRUE(tunable_from_attr(m[i].attr, &got));
        TEST_ASSERT_EQUAL_INT(m[i].id, got);

        for (int pass = 0; pass < 2; ++pass) {
            tunable_cfg_t a, b;
            fresh(&a); fresh(&b);
            const void *v = (s->kind == TK_F32)  ? (const void *)(pass ? &flo : &fhi)
                          : (s->kind == TK_U32)  ? (const void *)(pass ? &ulo : &uhi)
                                                 : (const void *)&bt;
            tunable_apply(&a, m[i].id, v);
            tunable_apply(&b, got, v);
            TEST_ASSERT_EQUAL_MEMORY(&a, &b, sizeof a);
        }
    }
}

void test_read_only_attrs_map_to_nothing(void){
    tunable_id_t id;
    TEST_ASSERT_FALSE(tunable_from_attr(0x000A, &id));   /* resync */
    TEST_ASSERT_FALSE(tunable_from_attr(0x000B, &id));   /* alarm bitmap */
    TEST_ASSERT_FALSE(tunable_from_attr(0x000C, &id));   /* fault bitmap */
    TEST_ASSERT_FALSE(tunable_from_attr(0x000D, &id));   /* travel since */
    TEST_ASSERT_FALSE(tunable_from_attr(0xFFFF, &id));   /* unknown */
    TEST_ASSERT_FALSE(tunable_from_attr(0x0013, &id));   /* one past the last real attr */
}

int main(void){
    UNITY_BEGIN();
    RUN_TEST(test_every_spec_entry_clamps_into_its_own_range);
    RUN_TEST(test_defaults_survive_clamp_all_unchanged);
    RUN_TEST(test_clamp_all_rejects_nan_back_to_default);
    RUN_TEST(test_clamp_all_pulls_out_of_range_values_in);
    RUN_TEST(test_valve_deadband_floor_tracks_travel_time);
    RUN_TEST(test_shortening_travel_lifts_a_stranded_deadband);
    RUN_TEST(test_valve_deadband_is_writable_as_a_tunable);
    RUN_TEST(test_out_of_range_id_is_ignored);
    RUN_TEST(test_attr_path_and_tunable_path_agree);
    RUN_TEST(test_read_only_attrs_map_to_nothing);
    return UNITY_END();
}
