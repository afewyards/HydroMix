# Sensor-Lag Compensation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop the supply-temperature limit cycle by making the PI loop wait out the pipe-wall/probe dead time (spec: `docs/superpowers/specs/2026-07-30-sensor-lag-compensation-design.md`).

**Architecture:** All control logic changes live in the pure-C `firmware/components/ctrl_core` component (host-tested via Unity in `firmware/test_host`). `firmware/main` only wires new config fields/attributes through. Z2M converter (`z2m/valvectl.mjs`) gains the two new tunables.

**Tech Stack:** ESP-IDF 5.x (installed at `~/esp/esp-idf`), esp-zigbee-sdk, plain C11, Unity host tests via CMake/ctest.

## Global Constraints

- ki UNIT CHANGE: %/K per 10 s cycle → **%/K per minute**. New defaults kp = 2.8, ki = 0.9. NVS migration ×6. Bounds: kp ∈ [0.5, 15], ki ∈ [0, 5], deadtime_s ∈ [0, 120], pi_deadband_k ∈ [0, 1].
- New tunables: `deadtime_s` (default 30 s, custom attr 0x000E), `pi_deadband_k` (default 0.25 K, attr 0x000F). Trim clamp ±20 % is a compile-time constant, NOT tunable.
- Firmware version → **1.1.0** (Task 8 only).
- Host suite: `cmake -S firmware/test_host -B firmware/test_host/build && cmake --build firmware/test_host/build && ctest --test-dir firmware/test_host/build --output-on-failure`. CMake GLOBs `test_*.c` — adding a NEW test file requires the cmake configure step to rerun (the command above does).
- Target build check: `cd firmware && source ~/esp/esp-idf/export.sh && idf.py build`.
- NEVER run tests/builds directly from an engineer context — delegate to a **test-runner** subagent.
- Commits: Angular convention (committing skill). Commit with `git add -- <files> && git commit -m "msg" -- <same files>` in one shell invocation; never blanket-stage (repo has unrelated dirty PCB files).
- Existing behavior to preserve: freeze semantics (resync/low-authority → output = FF, integrator untouched), 3-cycle FF-only hold after mode change, conditional anti-windup at the 0/100 total clamp, governor outermost.

---

### Task 1: dt-scaled ki (pi.c signature change)

**Files:**
- Modify: `firmware/components/ctrl_core/include/ctrl_core/pi.h`
- Modify: `firmware/components/ctrl_core/pi.c`
- Modify: `firmware/components/ctrl_core/control.c` (call sites + dt tracking)
- Modify: `firmware/components/ctrl_core/include/ctrl_core/control.h` (state fields)
- Test: `firmware/test_host/test_pi.c`, `firmware/test_host/test_control.c`

**Interfaces:**
- Produces: `float pi_step(pi_state_t *s, float pos_ff, float err_supply, bool cooling, bool freeze, float dt_s, const pi_cfg_t *cfg)` — new `dt_s` param (seconds since last call). `cfg->ki` is now %/K **per minute**: each call integrates `ki * err * (dt_s / 60)`.
- Produces: `control_state_t` gains `uint32_t last_now_ms; bool have_now;` — `control_step()` computes `dt_s` internally from its `now` param (clamped [1, 120] s; 10.0f on first call). Tasks 2–4 rely on `dt_s` reaching `pi_step`.

- [ ] **Step 1: Update test_pi.c — all `pi_step` calls gain `10.0f` as the dt argument (before `&cfg`), cfg ki 1.0 → 6.0 (same per-10 s effect), and add the failing scaling test**

```c
/* ki is %/K per MINUTE, scaled by actual dt. */
void test_ki_dt_scaling(void){
    pi_step(&s, 50.0f, 1.0f, false, false, 30.0f, &cfg);  /* integ += 6*1*(30/60) = 3 */
    TEST_ASSERT_EQUAL_FLOAT(3.0f, s.integ);
    pi_step(&s, 50.0f, 1.0f, false, false, 60.0f, &cfg);  /* += 6*1*1 = 6 */
    TEST_ASSERT_EQUAL_FLOAT(9.0f, s.integ);
}
```
Register with `RUN_TEST(test_ki_dt_scaling);`. In `test_control.c` setUp change `cfg.pi_cfg = (pi_cfg_t){ 4.0f, 3.0f, 0.0f, 100.0f };` (0.5/cycle ≡ 3.0/min) and in `test_heating_ff` widen the assert to `o.valve_target > 25.0f && o.valve_target < 80.0f` (the two warm-up calls are 60 s apart, so one dt-scaled step integrates 6× more than one old cycle did).

- [ ] **Step 2: test-runner — expect test_pi/test_control compile FAIL (wrong arg count)**

- [ ] **Step 3: Implement**

`pi.h`: signature + unit comment:
```c
typedef struct { float kp; float ki; /* %/K per minute */ float out_min; float out_max; } pi_cfg_t;
float pi_step(pi_state_t *s, float pos_ff, float err_supply, bool cooling, bool freeze,
              float dt_s, const pi_cfg_t *cfg);
```
`pi.c` line 15: `float cand = s->integ + cfg->ki * err * (dt_s / 60.0f);`

`control.h` `control_state_t`: add `uint32_t last_now_ms; bool have_now;`
`control.c`: in `control_init` add `s->have_now = false;`. In `control_step`, before the mode-detect line:
```c
    float dt_s = s->have_now ? ctrl_clampf((now - s->last_now_ms) / 1000.0f, 1.0f, 120.0f) : 10.0f;
    s->last_now_ms = now; s->have_now = true;
```
Both `pi_step(...)` call sites (CTRL_PI_ONLY, CTRL_FULL): insert `dt_s` before `&cfg->pi_cfg`.

- [ ] **Step 4: test-runner — full host suite green**
- [ ] **Step 5: Commit** `feat(ctrl): scale integral gain by elapsed time (ki now %/K per minute)`

---

### Task 2: Gap-form error deadband

**Files:**
- Modify: `firmware/components/ctrl_core/include/ctrl_core/pi.h`
- Modify: `firmware/components/ctrl_core/pi.c`
- Test: `firmware/test_host/test_pi.c`

**Interfaces:**
- Produces: `pi_cfg_t` gains trailing `float deadband_k;` (0 = disabled; existing initializers keep old behavior). Effective error = 0 inside ±deadband_k, ramps linearly beyond; integrator frozen inside the band.

- [ ] **Step 1: Add failing tests to test_pi.c**

```c
static const pi_cfg_t cfg_db = { .kp = 2.0f, .ki = 6.0f, .out_min = 0.0f, .out_max = 100.0f,
                                 .deadband_k = 0.25f };

/* Inside the band: pure FF out, integrator untouched. */
void test_deadband_inside(void){
    float out = pi_step(&s, 50.0f, 0.2f, false, false, 10.0f, &cfg_db);
    TEST_ASSERT_EQUAL_FLOAT(50.0f, out);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s.integ);
}

/* Gap form: err 1.25 acts as 1.0. p = 2*1 = 2, integ = 6*1*(10/60) = 1 -> 53. */
void test_deadband_gap_ramp(void){
    float out = pi_step(&s, 50.0f, 1.25f, false, false, 10.0f, &cfg_db);
    TEST_ASSERT_EQUAL_FLOAT(53.0f, out);
}
```

- [ ] **Step 2: test-runner — the two new tests FAIL**
- [ ] **Step 3: Implement in pi.c** — after the `err` sign flip, replace direct use of `err`:

```c
    float e_eff = 0.0f;                      /* gap deadband: 0 inside, ramps in beyond */
    if (err >  cfg->deadband_k)      e_eff = err - cfg->deadband_k;
    else if (err < -cfg->deadband_k) e_eff = err + cfg->deadband_k;
```
`p`, the integration term, and the two `pushing` conditions all switch from `err` to `e_eff` (inside the band `e_eff == 0` so `cand == s->integ` — integrator frozen for free).

- [ ] **Step 4: test-runner — full host suite green**
- [ ] **Step 5: Commit** `feat(ctrl): gap-form PI error deadband`

---

### Task 3: Trim clamp ±20 % with back-calculation

**Files:**
- Modify: `firmware/components/ctrl_core/include/ctrl_core/pi.h`
- Modify: `firmware/components/ctrl_core/pi.c`
- Test: `firmware/test_host/test_pi.c`

**Interfaces:**
- Produces: `#define PI_TRIM_CLAMP_PCT 20.0f` in pi.h. Total trim (P+I) never exceeds ±20 %; on clamp the integrator is back-calculated so `p + integ == ±20`.

- [ ] **Step 1: Add failing test to test_pi.c** (uses the file's default `cfg`: kp = 2, ki = 6)

```c
/* Trim (P+I) caps at +20 even though total output never hits the 0/100 clamp. */
void test_trim_clamp_backcalc(void){
    for (int i = 0; i < 40; i++) pi_step(&s, 50.0f, 5.0f, false, false, 10.0f, &cfg);
    float out = pi_step(&s, 50.0f, 5.0f, false, false, 10.0f, &cfg);
    TEST_ASSERT_EQUAL_FLOAT(70.0f, out);          /* 50 + 20 */
    TEST_ASSERT_TRUE(s.integ <= 10.0f + 1e-4f);   /* back-calculated: 20 - p(10) */
}
```

- [ ] **Step 2: test-runner — FAILS (integrator winds far past 20 today)**
- [ ] **Step 3: Implement** — pi.h: `#define PI_TRIM_CLAMP_PCT 20.0f`. pi.c, after computing `p` and `cand`, before the total-output clamp:

```c
    if (p + cand >  PI_TRIM_CLAMP_PCT) cand =  PI_TRIM_CLAMP_PCT - p;   /* back-calculate */
    if (p + cand < -PI_TRIM_CLAMP_PCT) cand = -PI_TRIM_CLAMP_PCT - p;
```

- [ ] **Step 4: test-runner — full host suite green (test_anti_windup must still pass: at the 0/100 clamp the old conditional integration still applies)**
- [ ] **Step 5: Commit** `fix(ctrl): clamp PI trim to ±20 % with integrator back-calculation`

---

### Task 4: Transit hold

**Files:**
- Modify: `firmware/components/ctrl_core/include/ctrl_core/control.h`
- Modify: `firmware/components/ctrl_core/control.c`
- Test: `firmware/test_host/test_control.c`

**Interfaces:**
- Consumes: `pi_step(..., dt_s, ...)` from Task 1.
- Produces: `control_in_t` gains `float valve_pos;` (actual estimated valve position, %). `control_cfg_t` gains `float deadtime_s;`. `control_state_t` gains `float last_pos; bool have_pos; bool holding; uint32_t hold_until_ms; float hold_supply_ref; float latched_trim;`. Constants in control.h: `#define TRANSIT_MOVE_PCT 0.5f`, `#define TRANSIT_RELEASE_K 0.25f`. Task 6 wires `valve_pos`/`deadtime_s` from main.

- [ ] **Step 1: Add failing tests to test_control.c** (also add `#include <math.h>` and this helper above the tests; setUp already runs; note the warm-up burns the 60 s mode dwell AND the 3-cycle post-mode-change FF-only hold)

```c
static uint32_t warmup_heating(void){
    uint32_t t = 0;
    control_step(&st, &in, &cfg, t);
    for (int i = 0; i < 8; i++){ t += 10000; control_step(&st, &in, &cfg, t); }
    return t;   /* HEATING committed, PI active */
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

/* Governor overrides regardless of hold state. */
void test_governor_bypasses_hold(void){
    cfg.deadtime_s = 60.0f;
    in.valve_pos = 40.0f; in.t_supply = 32.0f;
    uint32_t t = warmup_heating();
    in.valve_pos = 43.0f;
    control_step(&st, &in, &cfg, t += 10000);         /* armed */
    in.t_supply = 37.0f;                              /* over gov_high */
    control_out_t o = control_step(&st, &in, &cfg, t += 10000);
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
```
Register all with RUN_TEST. Also set `cfg.deadtime_s = 0.0f;` in setUp so all pre-existing tests keep exact old behavior (0 = hold disabled).

- [ ] **Step 2: test-runner — compile FAIL (missing fields)**
- [ ] **Step 3: Implement**

`control.h`: add the two `#define`s and the struct fields from Interfaces above.

`control.c`: `control_init` adds `s->have_pos = false; s->holding = false; s->latched_trim = 0.0f;`. Add this static helper above `control_step`:

```c
/* PI wrapped in the transit hold: after real valve movement, latch the trim and
 * freeze the integrator until the pipe answers (deadtime_s or >0.25 K supply move).
 * prev_pos/had_pos are the PREVIOUS cycle's valve position (control_step records the
 * current one into state before calling this). */
static float pi_transit(control_state_t *s, const control_in_t *in, const control_cfg_t *cfg,
                        float pos_ff, float t_set, bool cooling, bool freeze,
                        float dt_s, float prev_pos, bool had_pos, uint32_t now){
    if (s->holding &&
        ((int32_t)(now - s->hold_until_ms) >= 0 ||
         fabsf(in->t_supply - s->hold_supply_ref) > TRANSIT_RELEASE_K))
        s->holding = false;

    float target;
    if (s->holding){
        target = ctrl_clampf(pos_ff + s->latched_trim, cfg->pi_cfg.out_min, cfg->pi_cfg.out_max);
    } else {
        target = pi_step(&s->pi, pos_ff, t_set - in->t_supply, cooling, freeze, dt_s, &cfg->pi_cfg);
        s->latched_trim = target - pos_ff;
    }

    if (had_pos && fabsf(in->valve_pos - prev_pos) > TRANSIT_MOVE_PCT){
        s->holding = true;                                   /* (re)arm on any movement */
        s->hold_until_ms = now + (uint32_t)(cfg->deadtime_s * 1000.0f);
        s->hold_supply_ref = in->t_supply;
    }
    return target;
}
```
In `control_step`:
- At the very top, right after the Task 1 `dt_s` block, record position for next cycle (runs on EVERY path, including early returns — that's why it's at the top):
```c
    float prev_pos = s->last_pos; bool had_pos = s->have_pos;
    s->last_pos = in->valve_pos; s->have_pos = true;
```
- CTRL_PI_ONLY branch: `target = pi_transit(s, in, cfg, cfg->park_pos, t_set, cooling, freeze_pi, dt_s, prev_pos, had_pos, now);`
- CTRL_FULL branch: keep the `ff_step` + `freeze` lines, then `target = pi_transit(s, in, cfg, ff.pos_ff, t_set, cooling, freeze, dt_s, prev_pos, had_pos, now);`
- At every existing `pi_reset(...)`/`pi_mode_change(...)` call site (mode change, resync falling edge, water_running OFF, CTRL_PARK) also add `s->holding = false; s->latched_trim = 0.0f;`

- [ ] **Step 4: test-runner — full host suite green**
- [ ] **Step 5: Commit** `feat(ctrl): transit hold — PI waits out the pipe dead time after valve movement`

---

### Task 5: ctrl_core tunable map (mirror + bounds)

**Files:**
- Modify: `firmware/components/ctrl_core/include/ctrl_core/config_map.h`
- Modify: `firmware/components/ctrl_core/config_map.c`
- Test: `firmware/test_host/test_config_map.c`

**Interfaces:**
- Produces: `TUNABLE_DEADTIME_S`, `TUNABLE_PI_DEADBAND` enum values (appended after TUNABLE_COOL_SETPOINT); `tunable_cfg_t` gains `float deadtime_s, pi_deadband_k;`. New defaults: kp = 2.8, ki = 0.9, deadtime_s = 30, pi_deadband_k = 0.25. `tunable_apply` clamps: kp [0.5, 15], ki [0, 5], deadtime_s [0, 120], pi_deadband_k [0, 1].

- [ ] **Step 1: Add failing tests to test_config_map.c** (follow the file's existing pattern for setUp/RUN_TEST)

```c
void test_new_defaults(void){
    tunable_cfg_t c; tunable_cfg_defaults(&c);
    TEST_ASSERT_EQUAL_FLOAT(2.8f,  c.kp);
    TEST_ASSERT_EQUAL_FLOAT(0.9f,  c.ki);
    TEST_ASSERT_EQUAL_FLOAT(30.0f, c.deadtime_s);
    TEST_ASSERT_EQUAL_FLOAT(0.25f, c.pi_deadband_k);
}

void test_gain_bounds(void){
    tunable_cfg_t c; tunable_cfg_defaults(&c);
    float v;
    v = 99.0f;  tunable_apply(&c, TUNABLE_KP, &v);          TEST_ASSERT_EQUAL_FLOAT(15.0f,  c.kp);
    v = -1.0f;  tunable_apply(&c, TUNABLE_KI, &v);          TEST_ASSERT_EQUAL_FLOAT(0.0f,   c.ki);
    v = 999.0f; tunable_apply(&c, TUNABLE_DEADTIME_S, &v);  TEST_ASSERT_EQUAL_FLOAT(120.0f, c.deadtime_s);
    v = 5.0f;   tunable_apply(&c, TUNABLE_PI_DEADBAND, &v); TEST_ASSERT_EQUAL_FLOAT(1.0f,   c.pi_deadband_k);
}
```
If the existing file asserts the OLD kp/ki defaults (4.0/0.5), update those assertions to 2.8/0.9.

- [ ] **Step 2: test-runner — FAIL**
- [ ] **Step 3: Implement** — config_map.h enum + struct fields; config_map.c defaults `c->kp=2.8f; c->ki=0.9f; c->deadtime_s=30.0f; c->pi_deadband_k=0.25f;` and apply cases:

```c
    case TUNABLE_KP:          c->kp = ctrl_clampf(*(const float*)v, 0.5f, 15.0f); break;
    case TUNABLE_KI:          c->ki = ctrl_clampf(*(const float*)v, 0.0f, 5.0f); break;
    case TUNABLE_DEADTIME_S:  c->deadtime_s = ctrl_clampf(*(const float*)v, 0.0f, 120.0f); break;
    case TUNABLE_PI_DEADBAND: c->pi_deadband_k = ctrl_clampf(*(const float*)v, 0.0f, 1.0f); break;
```

- [ ] **Step 4: test-runner — full host suite green**
- [ ] **Step 5: Commit** `feat(ctrl): new tunables deadtime_s/pi_deadband_k, retuned defaults, bounded gains`

---

### Task 6: main/ plumbing — config, NVS migration, Zigbee attrs, task wiring

**Files:**
- Modify: `firmware/main/config.h`
- Modify: `firmware/main/config.c`
- Modify: `firmware/main/zigbee.h`
- Modify: `firmware/main/zigbee.c`
- Modify: `firmware/main/control_task.c`

**Interfaces:**
- Consumes: `control_in_t.valve_pos`, `control_cfg_t.deadtime_s`, `pi_cfg_t.deadband_k` (Tasks 2/4), `valve_get_position()` (exists in valve_hw.h).
- Produces: `config_t` gains trailing `float deadtime_s; float pi_deadband_k; uint32_t cfg_version;` + `#define CONFIG_VERSION 2` in config.h. Zigbee attrs `ATTR_DEADTIME_S 0x000E`, `ATTR_PI_DEADBAND 0x000F` (float, rw) — Task 8's converter keys on these ids.

- [ ] **Step 1: config.h** — append the three fields to `config_t` (order matters: NEW FIELDS GO LAST) and `#define CONFIG_VERSION 2`.

- [ ] **Step 2: config.c** — defaults, migration, bounded writes:

DEFAULTS: `.kp = 2.8f, .ki = 0.9f,` (was 4.0/0.5) and append `.deadtime_s = 30.0f, .pi_deadband_k = 0.25f, .cfg_version = CONFIG_VERSION,`.

Above `config_load`, the frozen v1 layout:
```c
/* v1 blob layout (≤1.0.7): ki was %/K per 10 s cycle, no transit-hold fields.
 * Field order/types must stay byte-identical to the old config_t. */
typedef struct {
    float heat_threshold, cool_threshold, hysteresis, heat_setpoint, cool_setpoint, park_pos;
    uint32_t travel_time_s; bool direction_swap;
    float kp, ki, gov_high, gov_low;
    uint32_t alarm_dwell_ms, enter_dwell_ms, leave_dwell_ms;
} config_v1_t;
```
Replace the load body's blob read with:
```c
    size_t sz = 0;
    if (nvs_get_blob(h, "cfg", NULL, &sz) == ESP_OK) {
        if (sz == sizeof(config_t)) {
            config_t tmp;
            if (nvs_get_blob(h, "cfg", &tmp, &sz) == ESP_OK && tmp.cfg_version == CONFIG_VERSION)
                g_config = tmp;
        } else if (sz == sizeof(config_v1_t)) {
            config_v1_t v1;
            if (nvs_get_blob(h, "cfg", &v1, &sz) == ESP_OK) {
                memcpy(&g_config, &v1, sizeof(v1));       /* common prefix, same layout */
                g_config.kp = ctrl_clampf(v1.kp, 0.5f, 15.0f);
                g_config.ki = ctrl_clampf(v1.ki * 6.0f, 0.0f, 5.0f);  /* per-10s-cycle -> per-min */
                g_config.deadtime_s = DEFAULTS.deadtime_s;
                g_config.pi_deadband_k = DEFAULTS.pi_deadband_k;
                g_config.cfg_version = CONFIG_VERSION;
                nvs_close(h);
                ESP_LOGI(TAG, "cfg migrated v1->v2 (ki %.2f/cycle -> %.2f/min)", v1.ki, g_config.ki);
                config_save();
                return;
            }
        }
    }
    nvs_close(h);
```
`config_apply_custom`: replace the KP/KI cases and add the new ones:
```c
    case ATTR_KP:          g_config.kp = ctrl_clampf(*(const float*)val, 0.5f, 15.0f); break;
    case ATTR_KI:          g_config.ki = ctrl_clampf(*(const float*)val, 0.0f, 5.0f); break;
    case ATTR_DEADTIME_S:  g_config.deadtime_s = ctrl_clampf(*(const float*)val, 0.0f, 120.0f); break;
    case ATTR_PI_DEADBAND: g_config.pi_deadband_k = ctrl_clampf(*(const float*)val, 0.0f, 1.0f); break;
```
(Known pattern quirk, acceptable: the ZCL attribute store keeps the raw written value until reboot while g_config holds the clamped one.)

- [ ] **Step 3: zigbee.h/zigbee.c** — zigbee.h: `#define ATTR_DEADTIME_S 0x000E` `#define ATTR_PI_DEADBAND 0x000F`. zigbee.c: two statics next to the existing ones (`static float s_attr_deadtime_s; static float s_attr_pi_deadband;`), seed them in `build_custom_cluster()` (`= g_config.deadtime_s;` / `= g_config.pi_deadband_k;`), and two attr registrations after the ATTR_ALARM_DWELL line:
```c
    esp_zb_custom_cluster_add_custom_attr(custom, ATTR_DEADTIME_S,  ESP_ZB_ZCL_ATTR_TYPE_SINGLE, rw, &s_attr_deadtime_s);
    esp_zb_custom_cluster_add_custom_attr(custom, ATTR_PI_DEADBAND, ESP_ZB_ZCL_ATTR_TYPE_SINGLE, rw, &s_attr_pi_deadband);
```
(`attr_cb` needs no change — non-RESYNC custom writes already fall through to `config_apply_custom`.)

- [ ] **Step 4: control_task.c** — wire the new inputs:
```c
        in.valve_pos = valve_get_position();
```
(next to the other `in.` lines) and in the `control_cfg_t cfg = {...}`:
```c
            .pi_cfg = { g_config.kp, g_config.ki, 0.0f, 100.0f, g_config.pi_deadband_k },
            .deadtime_s = g_config.deadtime_s,
```

- [ ] **Step 5: test-runner — target build: `cd firmware && source ~/esp/esp-idf/export.sh && idf.py build` must succeed; host suite still green**
- [ ] **Step 6: Commit** `feat(fw): expose deadtime_s/pi_deadband_k over Zigbee, migrate NVS config to v2`

---

### Task 7: FOPDT closed-loop regression test

**Files:**
- Create: `firmware/test_host/test_lagsim.c`

**Interfaces:**
- Consumes: `control_step`/`control_init` public API only (Tasks 1–4 behavior). This is the proof-of-fix test from the spec.

- [ ] **Step 1: Create the test** (new file → the cmake configure in the standard command picks it up via GLOB)

```c
#include "unity.h"
#include "ctrl_core/control.h"
#include <string.h>
#include <stdio.h>

/* Closed-loop regression: FOPDT plant (30 s dead time, 45 s probe lag) + 3-point
 * actuator (2 % deadband, 120 s stroke). Old tuning (hot per-cycle ki, no deadband,
 * no transit hold) sustains a relay limit cycle; 1.1.0 defaults converge. */

#define THETA_S   30
#define TAU_S     45.0f
#define T_SRC     45.0f
#define T_RET     27.0f
#define T_SET     35.0f
#define TRAVEL_S  120.0f

typedef struct { float pos, probe; float delay[THETA_S]; int di; } plant_t;

static void plant_init(plant_t *p){
    memset(p, 0, sizeof(*p));
    p->pos = 50.0f;
    float t0 = T_RET + p->pos / 100.0f * (T_SRC - T_RET);
    for (int i = 0; i < THETA_S; i++) p->delay[i] = t0;
    p->probe = t0;
}

/* One 1 s tick: actuator slews toward target outside its 2 % deadband, then the
 * mixed temp rides through the dead-time ring buffer into a first-order probe. */
static void plant_tick(plant_t *p, float target){
    float err = target - p->pos;
    if (err >  2.0f) p->pos += 100.0f / TRAVEL_S;
    if (err < -2.0f) p->pos -= 100.0f / TRAVEL_S;
    p->pos = ctrl_clampf(p->pos, 0.0f, 100.0f);
    float t_mix = T_RET + p->pos / 100.0f * (T_SRC - T_RET);
    float delayed = p->delay[p->di];
    p->delay[p->di] = t_mix;
    p->di = (p->di + 1) % THETA_S;
    p->probe += (delayed - p->probe) * (1.0f / TAU_S);
}

/* 60 min sim, control every 10 s; returns probe peak-to-peak over the last 15 min. */
static float run_sim(const pi_cfg_t *pi, float deadtime_s){
    control_state_t st; control_init(&st);
    control_cfg_t cfg = {
        .heat_setpoint = T_SET, .cool_setpoint = 18.0f, .park_pos = 50.0f,
        .mode_cfg = { 28.0f, 16.0f, 2.0f, 60000, 420000 },
        .pi_cfg = *pi,
        .gov_cfg = { 36.0f, 16.0f, 35.0f, 17.0f },
        .alarm_dwell_ms = 300000,
        .deadtime_s = deadtime_s,
    };
    plant_t p; plant_init(&p);
    control_in_t in = {0};
    in.water_running = true; in.link_up = true;
    in.t_source_f = T_SRC; in.t_return_f = T_RET; in.hx_a = 40.0f;
    float target = 50.0f, tmin = 1000.0f, tmax = -1000.0f;
    for (int t = 0; t < 3600; t++){
        plant_tick(&p, target);
        if (t % 10 == 0){
            in.t_supply = p.probe;
            in.valve_pos = p.pos;
            control_out_t o = control_step(&st, &in, &cfg, (uint32_t)t * 1000u);
            if (o.regulating) target = o.valve_target;
        }
        if (t >= 3600 - 900){
            if (p.probe < tmin) tmin = p.probe;
            if (p.probe > tmax) tmax = p.probe;
        }
    }
    return tmax - tmin;
}

void setUp(void){} void tearDown(void){}

void test_lag_regression_split(void){
    pi_cfg_t oldcfg = { .kp = 4.0f, .ki = 3.0f, .out_min = 0, .out_max = 100, .deadband_k = 0 };
    pi_cfg_t newcfg = { .kp = 2.8f, .ki = 0.9f, .out_min = 0, .out_max = 100, .deadband_k = 0.25f };
    float old_pp = run_sim(&oldcfg, 0.0f);
    float new_pp = run_sim(&newcfg, 30.0f);
    printf("lagsim: old p-p %.2f K, new p-p %.2f K\n", old_pp, new_pp);
    TEST_ASSERT_TRUE(new_pp < 0.8f);              /* converges */
    TEST_ASSERT_TRUE(old_pp > 0.8f);              /* old behaviour limit-cycles */
    TEST_ASSERT_TRUE(old_pp > 2.0f * new_pp);     /* the fix is material */
}

int main(void){
    UNITY_BEGIN();
    RUN_TEST(test_lag_regression_split);
    return UNITY_END();
}
```

- [ ] **Step 2: test-runner — run it.** If `old_pp > 0.8` fails (sim too benign), do NOT weaken the new-side assert: report the two printed p-p values back for review instead of self-adjusting thresholds — the plant parameters (THETA_S/TAU_S, actuator deadband) are the knobs to revisit, with review.
- [ ] **Step 3: test-runner — full host suite green**
- [ ] **Step 4: Commit** `test(ctrl): FOPDT closed-loop regression proving the limit-cycle fix`

---

### Task 8: Z2M converter, version bump, release prep

**Files:**
- Modify: `z2m/valvectl.mjs`
- Modify: `firmware/version.txt`
- Modify: `firmware/README.md`

**Interfaces:**
- Consumes: attr ids 0x000E/0x000F from Task 6.

- [ ] **Step 1: valvectl.mjs** — in `CUSTOM_ATTRS` add:
```js
    14: {key: 'deadtime_s',    type: T_SINGLE, rw: true},
    15: {key: 'pi_deadband_k', type: T_SINGLE, rw: true},
```
In `exposes` (next to the other tunables):
```js
        e.numeric('deadtime_s', ea.ALL).withUnit('s').withValueMin(0).withValueMax(120)
            .withDescription('Transit hold: PI pauses this long after valve movement'),
        e.numeric('pi_deadband_k', ea.ALL).withUnit('K').withValueMin(0).withValueMax(1)
            .withDescription('PI error deadband (gap form)'),
```
Update the ki expose: `e.numeric('ki', ea.ALL).withDescription('Integral gain, %/K per minute (1.1.0+; was per 10 s cycle)'),`. The `configure()` seed loop iterates `CUSTOM_ATTRS` keys, so the new ids are read automatically.

- [ ] **Step 2: version.txt** — `1.0.7` → `1.1.0`.
- [ ] **Step 3: README** — in `firmware/README.md`, find the tunables/attributes documentation and add the two new attrs (0x000E deadtime_s, 0x000F pi_deadband_k) + a note that ki is %/K per minute since 1.1.0 (NVS migrates automatically, ×6).
- [ ] **Step 4: test-runner — target build (`idf.py build`) with the new version; host suite green.**
- [ ] **Step 5: Commit** `feat(zigbee): expose transit-hold tunables; release 1.1.0`

---

## Owner checkpoints (manual, not tasks)

1. **Stopgap now (on 1.0.7, before any of this ships):** Z2M → device → set `kp = 2.8`, `ki = 0.15` (old per-cycle unit).
2. **Hardware:** thermal paste under all five probe clamps; 25 mm insulation over each probe + ~100 mm of pipe both sides.
3. **Deploy:** copy `z2m/valvectl.mjs` to `/config/zigbee2mqtt/` on the HA box, restart Z2M, then `bridge/request/device/configure` to seed the new tunables. Build `.ota` via `firmware/tools/make_ota.py`, add a `fileVersion` 0x01010000 entry to the OTA override index, push via Z2M. After OTA: verify `ki` reads 0.9 (migrated) and hunting p-p shrinks vs the pre-fix trace.

## Unresolved questions

1. The bench HX-A/HX-B GPIO swap (commit 6ee5f9e) is still in `sensors_hw.c` — should 1.1.0 keep it (probes still cross-plugged) or revert to spec pin order? Plan assumes **keep**; say the word and Task 6 gains the revert.
2. Trim clamp fixed at ±20 % (not tunable) — confirmed OK?
