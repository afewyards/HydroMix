# Gated Bidirectional Resync Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Resync toward the 100 %-source end-stop when position ≥ 50 % and a comfort gate passes; bounded 30-min deferral when it doesn't; recirc-end fallback = today's behavior.

**Architecture:** New pure module `ctrl_core/resync_policy` (gate eval + defer state machine, host-tested). `control_step` computes/publishes gate → `control_task` → `valve_note_resync_gate()` → valve task consults policy instead of auto-requesting recirc. Spec: `docs/superpowers/specs/2026-08-05-gated-bidirectional-resync-design.md`.

**Tech Stack:** ESP-IDF 5.x C, Unity host tests (`firmware/test_host`, GLOB picks up new `test_*.c` — cmake reconfigure needed).

## Global Constraints

- Constants (compile-time, `resync_policy.h`): `RESYNC_SRC_GATE_K 2.0f`, `RESYNC_GATE_GOV_MARGIN_K 1.0f`, `RESYNC_DEFER_MAX_MS 1800000u`, `RESYNC_NEAR_END_PCT 50.0f`.
- No new Zigbee attrs/tunables. No converter changes.
- Boot + manual `valve_resync()` stay forced recirc-end.
- Stroke duration unchanged both directions: `s_travel_latched_s × RESYNC_STALL_MULT`.
- Behavioral assertions only — never gate a test on reduced travel/movement (1.5.0 lesson).
- Host suite fallback runner (no cmake/network): `cc -O1 -o /tmp/t test_host/test_X.c components/ctrl_core/*.c test_host/build-check/_deps/unity-src/src/unity.c -Icomponents/ctrl_core/include -Itest_host/build-check/_deps/unity-src/src -lm` from `firmware/`.
- Engineers: do NOT run tests directly — delegate to test-runner subagent. Commits: Angular convention, stage files by name.

---

### Task 1: resync_policy module

**Files:**
- Create: `firmware/components/ctrl_core/include/ctrl_core/resync_policy.h`
- Create: `firmware/components/ctrl_core/resync_policy.c`
- Test: `firmware/test_host/test_resync_policy.c`

**Interfaces (Produces):**
```c
typedef enum { RESYNC_ACT_NONE, RESYNC_ACT_START_SOURCE, RESYNC_ACT_START_RECIRC } resync_action_t;
typedef struct { bool deferring; uint32_t defer_since_ms; } resync_policy_state_t;
void resync_policy_init(resync_policy_state_t *s);
void resync_gate_eval(float t_src_f, bool src_fault, bool mode_active, float t_set,
                      float gov_low, float gov_high, bool *ok, bool *hard_fail);
resync_action_t resync_policy_step(resync_policy_state_t *s, bool needs_resync, bool gate_ok,
                                   float pos_pct, uint32_t now_ms);
bool resync_policy_mid_stroke_abort(bool toward_source, bool gate_hard_fail);
```

- [ ] **Step 1: failing test** — write `test_resync_policy.c` (match `main()`/RUN_TEST style of `test_pos_estimator.c`):

```c
#include "unity.h"
#include "ctrl_core/resync_policy.h"

static resync_policy_state_t s;
void setUp(void){ resync_policy_init(&s); }
void tearDown(void){}

void test_gate_pass(void){
    bool ok, hard;
    resync_gate_eval(18.6f, false, true, 18.5f, 16.0f, 36.0f, &ok, &hard);
    TEST_ASSERT_TRUE(ok); TEST_ASSERT_FALSE(hard);
}
void test_gate_fail_distance(void){          /* 2.5 K off setpoint */
    bool ok, hard;
    resync_gate_eval(21.0f, false, true, 18.5f, 16.0f, 36.0f, &ok, &hard);
    TEST_ASSERT_FALSE(ok); TEST_ASSERT_FALSE(hard);
}
void test_gate_fail_gov_margin(void){        /* within 2 K but < gov_low+1 */
    bool ok, hard;
    resync_gate_eval(16.5f, false, true, 17.5f, 16.0f, 36.0f, &ok, &hard);
    TEST_ASSERT_FALSE(ok); TEST_ASSERT_FALSE(hard);
}
void test_gate_hard_outside_band(void){      /* cold slug: 14.6 < gov_low */
    bool ok, hard;
    resync_gate_eval(14.6f, false, true, 18.5f, 16.0f, 36.0f, &ok, &hard);
    TEST_ASSERT_FALSE(ok); TEST_ASSERT_TRUE(hard);
}
void test_gate_hard_fault_or_idle(void){
    bool ok, hard;
    resync_gate_eval(18.5f, true,  true,  18.5f, 16.0f, 36.0f, &ok, &hard);
    TEST_ASSERT_TRUE(hard);
    resync_gate_eval(18.5f, false, false, 18.5f, 16.0f, 36.0f, &ok, &hard);
    TEST_ASSERT_TRUE(hard); TEST_ASSERT_FALSE(ok);
}
void test_no_trip_no_action(void){
    TEST_ASSERT_EQUAL(RESYNC_ACT_NONE, resync_policy_step(&s, false, true, 90.0f, 0));
}
void test_low_pos_recirc_now(void){
    TEST_ASSERT_EQUAL(RESYNC_ACT_START_RECIRC, resync_policy_step(&s, true, false, 20.0f, 0));
}
void test_high_pos_gate_ok_source(void){
    TEST_ASSERT_EQUAL(RESYNC_ACT_START_SOURCE, resync_policy_step(&s, true, true, 90.0f, 0));
}
void test_defer_then_gate_pass(void){
    TEST_ASSERT_EQUAL(RESYNC_ACT_NONE,         resync_policy_step(&s, true, false, 90.0f, 0));
    TEST_ASSERT_EQUAL(RESYNC_ACT_NONE,         resync_policy_step(&s, true, false, 90.0f, 600000));
    TEST_ASSERT_EQUAL(RESYNC_ACT_START_SOURCE, resync_policy_step(&s, true, true,  90.0f, 900000));
}
void test_defer_timeout_recirc(void){
    TEST_ASSERT_EQUAL(RESYNC_ACT_NONE,         resync_policy_step(&s, true, false, 90.0f, 0));
    TEST_ASSERT_EQUAL(RESYNC_ACT_START_RECIRC, resync_policy_step(&s, true, false, 90.0f, RESYNC_DEFER_MAX_MS));
}
void test_defer_pos_drop_recirc(void){       /* valve closed below 50 % while deferring */
    TEST_ASSERT_EQUAL(RESYNC_ACT_NONE,         resync_policy_step(&s, true, false, 90.0f, 0));
    TEST_ASSERT_EQUAL(RESYNC_ACT_START_RECIRC, resync_policy_step(&s, true, false, 40.0f, 60000));
}
void test_abort_only_source_stroke(void){
    TEST_ASSERT_TRUE (resync_policy_mid_stroke_abort(true,  true));
    TEST_ASSERT_FALSE(resync_policy_mid_stroke_abort(true,  false));
    TEST_ASSERT_FALSE(resync_policy_mid_stroke_abort(false, true));
}
```

- [ ] **Step 2: verify FAIL** (compile error: header missing) — via test-runner, fallback `cc` one-liner above.
- [ ] **Step 3: implement.** Header per Interfaces block + the 4 constants (Global Constraints). Implementation:

```c
#include "ctrl_core/resync_policy.h"
#include <math.h>

void resync_policy_init(resync_policy_state_t *s){ s->deferring = false; s->defer_since_ms = 0; }

void resync_gate_eval(float t_src_f, bool src_fault, bool mode_active, float t_set,
                      float gov_low, float gov_high, bool *ok, bool *hard_fail){
    *hard_fail = src_fault || !mode_active || t_src_f < gov_low || t_src_f > gov_high;
    *ok = !*hard_fail
          && fabsf(t_src_f - t_set) <= RESYNC_SRC_GATE_K
          && t_src_f >= gov_low  + RESYNC_GATE_GOV_MARGIN_K
          && t_src_f <= gov_high - RESYNC_GATE_GOV_MARGIN_K;
}

resync_action_t resync_policy_step(resync_policy_state_t *s, bool needs_resync, bool gate_ok,
                                   float pos_pct, uint32_t now_ms){
    if (!needs_resync){ s->deferring = false; return RESYNC_ACT_NONE; }
    if (pos_pct < RESYNC_NEAR_END_PCT){ s->deferring = false; return RESYNC_ACT_START_RECIRC; }
    if (gate_ok){ s->deferring = false; return RESYNC_ACT_START_SOURCE; }
    if (!s->deferring){ s->deferring = true; s->defer_since_ms = now_ms; return RESYNC_ACT_NONE; }
    if ((uint32_t)(now_ms - s->defer_since_ms) >= RESYNC_DEFER_MAX_MS){
        s->deferring = false; return RESYNC_ACT_START_RECIRC;
    }
    return RESYNC_ACT_NONE;
}

bool resync_policy_mid_stroke_abort(bool toward_source, bool gate_hard_fail){
    return toward_source && gate_hard_fail;
}
```

- [ ] **Step 4: verify PASS** (test-runner; cmake path needs reconfigure to pick up the new test).
- [ ] **Step 5: commit** `feat(fw): resync_policy — comfort gate + bounded deferral state machine`

---

### Task 2: pos_est_resync_done seed parameter

**Files:**
- Modify: `firmware/components/ctrl_core/include/ctrl_core/pos_estimator.h` (last decl)
- Modify: `firmware/components/ctrl_core/pos_estimator.c` (last fn)
- Modify: `firmware/main/valve_hw.c:72` (sole firmware caller — keep compiling, recirc semantics unchanged)
- Test: `firmware/test_host/test_pos_estimator.c`

**Interfaces (Produces):** `void pos_est_resync_done(pos_est_state_t *s, float seed_pct);`

- [ ] **Step 1: failing test** — append to `test_pos_estimator.c`:

```c
void test_resync_done_seeds_position(void){
    pos_est_update(&s, +1, 60000, 120.0f);
    pos_est_resync_done(&s, 100.0f);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, s.position_pct);
    TEST_ASSERT_EQUAL_FLOAT(0.0f,  s.accum_travel_pct);
    TEST_ASSERT_EQUAL_UINT32(0,    s.reversals);
    pos_est_resync_done(&s, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s.position_pct);
}
```
(add RUN_TEST; grep ALL existing `pos_est_resync_done(` callers in test_host/*.c and update to `(…, 0.0f)`)

- [ ] **Step 2: verify FAIL** (signature mismatch).
- [ ] **Step 3: implement:**

```c
void pos_est_resync_done(pos_est_state_t *s, float seed_pct){
    s->position_pct = ctrl_clampf(seed_pct, 0.0f, 100.0f);
    s->accum_travel_pct = 0.0f; s->reversals = 0; s->last_sign = 0;
}
```
And `valve_hw.c:72`: `pos_est_resync_done(&s_pos, 0.0f);` (bidirectional call comes in Task 4).

- [ ] **Step 4: verify PASS** — full host suite (other suites also call this fn).
- [ ] **Step 5: commit** `feat(fw): pos_est_resync_done takes end-stop seed`

---

### Task 3: gate computation in control_step

**Files:**
- Modify: `firmware/components/ctrl_core/include/ctrl_core/control.h` (control_out_t only — fields are plain bools, no new include)
- Modify: `firmware/components/ctrl_core/control.c` (`#include "ctrl_core/resync_policy.h"`; insert right after `mode_detect_step`, control.c:70-71 — BEFORE all early returns so the gate publishes every cycle incl. water-off/IDLE/park paths)
- Test: `firmware/test_host/test_control.c`

**Interfaces:**
- Consumes: `resync_gate_eval` (Task 1).
- Produces: `control_out_t` fields `bool resync_src_ok; bool resync_src_hard_fail;` (Task 4's control_task reads them).

- [ ] **Step 1: failing test** — in `test_control.c`, reuse the file's existing helper/fixture that reaches MODE_COOLING (it exists — see the cooling tests), then:

```c
/* cooling, src at setpoint -> gate ok; src 10 C (below gov_low 16) -> hard fail */
void test_resync_gate_published(void){
    /* fixture to MODE_COOLING as elsewhere in this file */
    in.t_source_f = /* fixture cool setpoint */;
    control_out_t o = control_step(&st, &in, &cfg, now);
    TEST_ASSERT_TRUE(o.resync_src_ok);
    TEST_ASSERT_FALSE(o.resync_src_hard_fail);
    in.t_source_f = 10.0f;
    o = control_step(&st, &in, &cfg, now + 10000);
    TEST_ASSERT_FALSE(o.resync_src_ok);
    TEST_ASSERT_TRUE(o.resync_src_hard_fail);
}
```
Fixture gotchas (from memory, verify in file): `setUp` leaves `cfg.deadtime_s = 0` (transit hold disabled — irrelevant here); mode needs hx_a + enter dwell to leave IDLE.

- [ ] **Step 2: verify FAIL** (no such fields).
- [ ] **Step 3: implement** — control.h: append the two bools to `control_out_t`. control.c after mode detect:

```c
/* Resync source-end gate: published every cycle, incl. non-regulating paths, so the
 * valve task always has a current verdict for strokes it starts on its own clock. */
{
    float eff_cool_g = cooling_link_guard(cfg->cool_setpoint, mode, in->link_last_seen_ms, now);
    float t_set_g = (mode == MODE_COOLING) ? ctrl_clampf(eff_cool_g, 17.0f, 35.0f)
                                           : ctrl_clampf(cfg->heat_setpoint, 17.0f, 35.0f);
    resync_gate_eval(in->t_source_f, in->faults.source, mode != MODE_IDLE, t_set_g,
                     cfg->gov_cfg.gov_low, cfg->gov_cfg.gov_high,
                     &o.resync_src_ok, &o.resync_src_hard_fail);
}
```

- [ ] **Step 4: verify PASS** — full host suite.
- [ ] **Step 5: commit** `feat(fw): publish resync source-end gate from control_step`

---

### Task 4: valve_hw bidirectional resync + plumbing

**Files:**
- Modify: `firmware/main/valve_hw.h` (add `void valve_note_resync_gate(bool ok, bool hard_fail);`)
- Modify: `firmware/main/valve_hw.c`
- Modify: `firmware/main/control_task.c` (after control_step at :81-82, add `valve_note_resync_gate(o.resync_src_ok, o.resync_src_hard_fail);`)

**Interfaces:** Consumes Task 1 policy fns, Task 2 seed, Task 3 out-fields. No host-testable surface (hardware glue) — logic already covered by Task 1 tests.

- [ ] **Step 1: implement.** In `valve_hw.c`:
  - `#include "ctrl_core/resync_policy.h"`; statics: `static resync_policy_state_t s_rspol;` `static bool s_gate_ok = false, s_gate_hard = true;` (boot default recirc-only until control publishes) `static bool s_rs_toward_src = false;`
  - `valve_start()`: add `resync_policy_init(&s_rspol);` (boot `s_resync_req = true` stays = forced recirc).
  - Setter (mirrors `valve_set_target` locking):
```c
void valve_note_resync_gate(bool ok, bool hard_fail){
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_gate_ok = ok; s_gate_hard = hard_fail;
    xSemaphoreGive(s_lock);
}
```
  - Replace the start-trigger line (`if (s_resync_req && s_rs == RS_IDLE){…}`) with:
```c
if (s_rs == RS_IDLE){
    if (s_resync_req){                       /* boot + manual valve_resync(): forced recirc */
        s_rs = RS_DRIVING; s_rs_toward_src = false; s_rs_start_ms = t; s_resync_req = false;
    } else {
        resync_action_t act = resync_policy_step(&s_rspol, pos_est_needs_resync(&s_pos),
                                                 s_gate_ok, s_pos.position_pct, t);
        if (act != RESYNC_ACT_NONE){
            s_rs = RS_DRIVING;
            s_rs_toward_src = (act == RESYNC_ACT_START_SOURCE);
            s_rs_start_ms = t;
        }
    }
}
```
  - RS_DRIVING block: abort-check first, direction by `s_rs_toward_src`, seed on completion:
```c
if (s_rs == RS_DRIVING){
    if (resync_policy_mid_stroke_abort(s_rs_toward_src, s_gate_hard)){
        s_rs_toward_src = false; s_rs_start_ms = t;   /* fresh full recirc stroke */
    }
    uint32_t stall_ms = (uint32_t)(s_travel_latched_s * 1000.0f * RESYNC_STALL_MULT);
    if (t - s_rs_start_ms >= stall_ms){
        pos_est_resync_done(&s_pos, s_rs_toward_src ? 100.0f : 0.0f);
        s_rs = RS_IDLE;
        want = VALVE_STOP;
    } else {
        want = s_rs_toward_src ? dir_toward_source() : dir_toward_recirc();
    }
} else {
    want = desired_dir(s_target);
}
```
  - DELETE the old `if (pos_est_needs_resync(&s_pos)) s_resync_req = true;` line (policy consumes needs_resync directly) and the old `/* NEVER toward source */` comment — replace with a comment referencing the gate + spec date.
- [ ] **Step 2: verify** — full host suite green (no regressions) AND `idf.py build` clean (device compile is the real test here).
- [ ] **Step 3: commit** `feat(fw): gated bidirectional resync with deferral and mid-stroke abort`

---

### Task 5: release 1.6.0

**Files:**
- Modify: `firmware/version.txt` → `1.6.0`
- Modify: `firmware/README.md` (1.6.0 section: gate rule, deferral, abort, constants, spec link)

- [ ] **Step 1:** version + README.
- [ ] **Step 2:** `idf.py reconfigure && idf.py build` (MANDATORY reconfigure — stale-PROJECT_VER gotcha). Verify embedded version: read `esp_app_desc_t` at offset 0x20 of the .bin (magic 0xABCD5432, version string at +16) — `esptool image_info` does NOT print it.
- [ ] **Step 3:** `python3 tools/make_ota.py --out firmware/build` (`--out` is a DIRECTORY; script reads the built binary itself, run from `firmware/`).
- [ ] **Step 4:** full host suite green; commit `feat(fw): release 1.6.0 — gated bidirectional resync`. USB flash / OTA-index staging = owner step, main session only (subagents must not flash).

---

## Unresolved questions

1. Manual Z2M resync switch stays forced recirc-end (predictable for bench use) — confirm.
2. After merge: 1.6.0 delivery — USB flash again, or stage the .ota in the override index (single-entry, newest only) for a ~1 h OTA?
