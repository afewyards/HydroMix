# Review-Findings Fixes (1.2.1 → 1.3.0) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix all 7 major + 5 minor + 3 nit findings from the 2026-07-31 firmware review of the 1.2.1 tree, ship as 1.3.0.

**Architecture:** Pure logic goes into `firmware/components/ctrl_core/` (host-tested, Unity). `firmware/main/` gets HW/stack wiring only. One new ctrl_core unit (`ota_gate`), one new NVS key (water_running, outside the cfg blob), one z2m converter change. Task 0 first commits the already-shipped 1.2.1 working tree so git history matches the binary on the live device.

**Tech Stack:** ESP-IDF 5.5.4 (`~/esp/esp-idf`), esp-zigbee-lib 1.6.8 / esp-zboss-lib 1.6.4, plain C11, Unity + CMake/ctest host suite, Python 3 (`tools/make_ota.py`).

## Global Constraints

- **Host tests build OUTSIDE the repo.** The repo already has 16 stray `firmware/test_host/build-*` dirs; do not add another. Canonical command (validated, 13/13 green):
  ```bash
  cmake -S firmware/test_host -B /tmp/vc-tests \
    -DFETCHCONTENT_SOURCE_DIR_UNITY=/Users/kleist/Sites/ValveController/firmware/test_host/build-121a/_deps/unity-src \
  && cmake --build /tmp/vc-tests -j8 \
  && ctest --test-dir /tmp/vc-tests --output-on-failure
  ```
  The `FETCHCONTENT_SOURCE_DIR_UNITY` override reuses the vendored Unity v2.6.0 already on disk — without it CMake hits the network.
- `firmware/test_host/CMakeLists.txt` **GLOBs** `test_*.c`. Adding a NEW test file requires re-running the `cmake -S ... -B ...` configure step (the command above does). `firmware/components/ctrl_core/CMakeLists.txt` GLOBs `*.c`, so new ctrl_core sources need no build-file edit.
- **Full target build (`idf.py build`) is a VERIFY step in the FINAL task only**, not per-task. `source ~/esp/esp-idf/export.sh` first.
- `sdkconfig` is gitignored. Source of truth is `firmware/sdkconfig.defaults`: `CONFIG_ESP_TASK_WDT_INIT=y`, `CONFIG_ESP_TASK_WDT_TIMEOUT_S=30`, `CONFIG_ESP_TASK_WDT_PANIC=y`, `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y`.
- **Device is live on a household heating loop.** Every default chosen here is the safety-conservative one. Never leave the tree in a state where a mid-plan flash regresses live behaviour (this dictates the Task 4-before-Task 5 ordering; see Task 5).
- **NEVER run tests/builds directly from an engineer context** — delegate to a **test-runner** subagent.
- Commits: Angular convention. Always path-limited, one shell invocation:
  `git add -- <files> && git commit -m "msg" -- <same files>`. **Never blanket-stage** — `board.step`, `pcb/ValveController.kicad_pcb`, `pcb/ValveController.kicad_sch`, `pcb/3dmodels/`, `top.stl` are the owner's uncommitted work and must NOT be committed by any task.
- Max 5 files per task.
- **All line numbers in this plan are as-of the 1.2.1 tree committed in Task 0** and drift as earlier tasks land. Every edit step quotes the exact text to replace — **anchor on the quoted content, not the line number.** Notably `firmware/main/zigbee.c` is edited by Tasks 4, 6, 7, 9 and 12, and `firmware/main/control_task.c` by Tasks 2, 4 and 8.
- z2m converter lives at `z2m/valvectl.mjs`. **Deploying it to HAOS is out of scope** (see Non-goals).

## Non-goals (explicitly out of scope)

- Bench verifications: reporting `max_interval=0` semantics, thermostat setpoint writes above 30 °C, the Z2M `availability` setting.
- z2m converter climate/setpoint expose gap (`tz.thermostat_occupied_*_setpoint` handlers are wired but no `e.climate()` expose exists) — separate decision, Unresolved Q5.
- Deployment: USB flash, OTA staging to HAOS, Z2M converter/index install.
- `interlock.c:15` `both_error_count` is unreachable — **deliberately left in place** as a zero-cost safety net (Task 12 records the rationale, changes no code).

---

### Task 0: Commit the shipped 1.2.1 tree as-is

The current uncommitted firmware working tree is byte-for-byte what was OTA-delivered to the live device today. Commit it unchanged so git history matches the shipped artifact. **No code edits in this task.**

**Files:**
- Modify (commit only, no edits): `firmware/components/ctrl_core/sensor_policy.c`, `firmware/main/control_task.c`, `firmware/main/ota.c`, `firmware/test_host/test_sensor_policy.c`, `firmware/version.txt`, `firmware/README.md`

- [ ] **Step 1: Confirm exactly these six files, and nothing else, are staged**

```bash
git diff --stat -- firmware/README.md firmware/components/ctrl_core/sensor_policy.c \
  firmware/main/control_task.c firmware/main/ota.c \
  firmware/test_host/test_sensor_policy.c firmware/version.txt
```
Expected: exactly 6 files listed. `firmware/version.txt` must read `1.2.1`:
```bash
cat firmware/version.txt   # -> 1.2.1
```

- [ ] **Step 2: Confirm the owner's non-firmware changes are NOT in the commit set**

```bash
git status --short -- board.step pcb top.stl
```
Expected: `board.step`, `pcb/ValveController.kicad_pcb`, `pcb/ValveController.kicad_sch` still show ` M`, `pcb/3dmodels/`, `top.stl` still `??`. They must remain untouched after Step 3.

- [ ] **Step 3: Run the host suite on the unmodified tree**

Delegate to test-runner:
```bash
cmake -S firmware/test_host -B /tmp/vc-tests \
  -DFETCHCONTENT_SOURCE_DIR_UNITY=/Users/kleist/Sites/ValveController/firmware/test_host/build-121a/_deps/unity-src \
&& cmake --build /tmp/vc-tests -j8 \
&& ctest --test-dir /tmp/vc-tests --output-on-failure
```
Expected: `100% tests passed out of 13`.

- [ ] **Step 4: Commit**

```bash
git add -- firmware/README.md firmware/components/ctrl_core/sensor_policy.c \
  firmware/main/control_task.c firmware/main/ota.c \
  firmware/test_host/test_sensor_policy.c firmware/version.txt \
&& git commit -m "fix(fw): leaky fault-streak decay, OTA gate decoupled from probes; release 1.2.1" \
  -- firmware/README.md firmware/components/ctrl_core/sensor_policy.c \
     firmware/main/control_task.c firmware/main/ota.c \
     firmware/test_host/test_sensor_policy.c firmware/version.txt
```

- [ ] **Step 5: Verify the working tree still has the owner's PCB changes**

```bash
git status --short
```
Expected: `board.step`, `pcb/*` and `top.stl` entries survive; no `firmware/` entries except the untracked `.cache/` and `build-*/` dirs.

---

### Task 1: F5 — fault-streak decay needs 3 consecutive goods

`sensor_policy.c:22` forgives one fail after **2** consecutive goods, so break-even sits at a 33 % failure rate: `good,good,fail` repeating nets zero and a probe failing a third of its sweeps **never latches**. Raise the requirement to 3 consecutive goods (break-even 25 %).

**Files:**
- Modify: `firmware/components/ctrl_core/include/ctrl_core/sensor_policy.h`
- Modify: `firmware/components/ctrl_core/sensor_policy.c:18-22`
- Test: `firmware/test_host/test_sensor_policy.c`

**Interfaces:**
- Produces: `#define SENSOR_DECAY_AFTER 3` in `sensor_policy.h` — consecutive good reads required before each further good forgives one fail. `sensor_fault_update()` signature is unchanged.

- [ ] **Step 1: Rewrite the inverted test and add the 25 % break-even test**

In `firmware/test_host/test_sensor_policy.c`, **delete** the comment block at lines 60-73 and the whole `test_repeating_good_good_fail_never_latches` function (lines 74-85), and insert in their place:

```c
/* Leaky-bucket coverage. Rule under test (sensor_policy.c): fail_streak decays by 1
 * on every consecutive good read from the SENSOR_DECAY_AFTER'th onward; good_streak
 * resets to 0 on any bad read. Break-even is therefore one fail in
 * SENSOR_DECAY_AFTER+1 reads -- 25 % at the shipped value of 3. */

/* good,good,fail repeating (33 % failure) MUST latch: two goods never reach
 * SENSOR_DECAY_AFTER, so nothing is forgiven and fail_streak climbs by exactly 1
 * per cycle. Latches on the 3rd cycle's fail == the 9th sweep. Regression guard for
 * the 1.2.1 bug where the threshold was 2 and this pattern never latched at all. */
void test_repeating_good_good_fail_latches(void){
    sensor_fault_state_t s = {0};
    int cycles_to_latch = -1;
    for (int cycle = 0; cycle < 20 && cycles_to_latch < 0; ++cycle) {
        sensor_fault_update(&s, true);
        sensor_fault_update(&s, true);
        TEST_ASSERT_EQUAL_INT(cycle, s.fail_streak);        /* nothing forgiven */
        sensor_fault_update(&s, false);
        TEST_ASSERT_EQUAL_INT(cycle + 1, s.fail_streak);
        if (s.faulted) cycles_to_latch = cycle + 1;
    }
    TEST_ASSERT_EQUAL_INT(SENSOR_FAULT_AFTER, cycles_to_latch);
}

/* good,good,good,fail repeating (25 % failure, the break-even rate) must NEVER
 * latch: the 3rd consecutive good forgives that cycle's fail exactly, so
 * fail_streak oscillates 0 <-> 1 forever. */
void test_repeating_three_good_one_fail_never_latches(void){
    sensor_fault_state_t s = {0};
    for (int cycle = 0; cycle < 50; ++cycle) {
        sensor_fault_update(&s, true);
        sensor_fault_update(&s, true);
        sensor_fault_update(&s, true);
        TEST_ASSERT_EQUAL_INT(0, s.fail_streak);            /* 3rd good decays */
        sensor_fault_update(&s, false);
        TEST_ASSERT_EQUAL_INT(1, s.fail_streak);
        TEST_ASSERT_FALSE(s.faulted);
    }
}

/* A long good run keeps decaying once per read -- good_streak is not reset by a
 * decay, only by a bad read. Three fails then five goods must fully drain the
 * bucket (goods 3, 4 and 5 each forgive one). */
void test_long_good_run_decays_once_per_read(void){
    sensor_fault_state_t s = {0};
    sensor_fault_update(&s, false);
    sensor_fault_update(&s, false);            /* fail_streak 2, not yet latched */
    TEST_ASSERT_FALSE(s.faulted);
    sensor_fault_update(&s, true);
    sensor_fault_update(&s, true);
    TEST_ASSERT_EQUAL_INT(2, s.fail_streak);
    sensor_fault_update(&s, true);             /* 3rd consecutive good */
    TEST_ASSERT_EQUAL_INT(1, s.fail_streak);
    sensor_fault_update(&s, true);             /* 4th */
    TEST_ASSERT_EQUAL_INT(0, s.fail_streak);
    sensor_fault_update(&s, true);             /* 5th: floor at 0, no underflow */
    TEST_ASSERT_EQUAL_INT(0, s.fail_streak);
}
```

Update the registration in `main()`: replace
`RUN_TEST(test_repeating_good_good_fail_never_latches);` with
```c
    RUN_TEST(test_repeating_good_good_fail_latches);
    RUN_TEST(test_repeating_three_good_one_fail_never_latches);
    RUN_TEST(test_long_good_run_decays_once_per_read);
```

- [ ] **Step 2: Replace the remaining hardcoded 3s and stale comments in the same file (finding 13b + 15)**

In `test_single_good_does_not_clear_latched_fault`, change
`TEST_ASSERT_EQUAL_INT(3, s.fail_streak);   /* not zeroed by the single good read */` to:
```c
    TEST_ASSERT_EQUAL_INT(SENSOR_FAULT_AFTER, s.fail_streak);  /* not zeroed by the single good read */
```

In `test_clears_after_3_consecutive_goods`, change `TEST_ASSERT_TRUE(sensor_fault_update(&s, true));   /* 3rd good clears */` to:
```c
    TEST_ASSERT_TRUE(sensor_fault_update(&s, true));   /* SENSOR_CLEAR_AFTER'th good clears */
```

In `test_fail_fail_good_fail_latches`, replace its leading comment and the two magic 3s:
```c
/* fail,fail,good,fail: the single good read only reaches good_streak == 1, well short
 * of SENSOR_DECAY_AFTER, so it forgives nothing. The trailing fail pushes fail_streak
 * to SENSOR_FAULT_AFTER and latches -- a net-failing sensor (3 bad in 4 sweeps) must
 * latch. */
void test_fail_fail_good_fail_latches(void){
    sensor_fault_state_t s = {0};
    sensor_fault_update(&s, false);
    TEST_ASSERT_EQUAL_INT(1, s.fail_streak);
    sensor_fault_update(&s, false);
    TEST_ASSERT_EQUAL_INT(2, s.fail_streak);
    TEST_ASSERT_FALSE(s.faulted);
    sensor_fault_update(&s, true);
    TEST_ASSERT_EQUAL_INT(2, s.fail_streak);   /* 1 consecutive good < SENSOR_DECAY_AFTER */
    TEST_ASSERT_FALSE(s.faulted);
    sensor_fault_update(&s, false);
    TEST_ASSERT_EQUAL_INT(SENSOR_FAULT_AFTER, s.fail_streak);
    TEST_ASSERT_TRUE(s.faulted);
}
```

In `test_repeating_fail_fail_good_latches_within_a_few_cycles`, replace its leading comment:
```c
/* fail,fail,good repeating: the lone good never reaches SENSOR_DECAY_AFTER consecutive
 * (the next fail resets good_streak to 0 first), so decay never fires at all -- 
 * fail_streak climbs by 2 every cycle and a mostly-bad sensor latches almost
 * immediately (at the start of the 2nd cycle). */
```

- [ ] **Step 3: test-runner — expect `test_sensor_policy` to FAIL**

```bash
cmake -S firmware/test_host -B /tmp/vc-tests \
  -DFETCHCONTENT_SOURCE_DIR_UNITY=/Users/kleist/Sites/ValveController/firmware/test_host/build-121a/_deps/unity-src \
&& cmake --build /tmp/vc-tests -j8 \
&& ctest --test-dir /tmp/vc-tests --output-on-failure -R test_sensor_policy
```
Expected: FAIL — under the shipped 2-good rule the second good of cycle 1 already decays, so `test_repeating_good_good_fail_latches` aborts on its mid-cycle assert with `Expected 1 Was 0` (it never reaches the latch assertion), and `test_long_good_run_decays_once_per_read` fails with `Expected 2 Was 1` at the second good. `test_repeating_three_good_one_fail_never_latches` passes under both rules — it is the guard against over-correcting.

- [ ] **Step 4: Implement**

`firmware/components/ctrl_core/include/ctrl_core/sensor_policy.h` — add below `SENSOR_CLEAR_AFTER`:
```c
#define SENSOR_DECAY_AFTER  3         /* consecutive good reads before each further good forgives one fail */
```

`firmware/components/ctrl_core/sensor_policy.c` — replace lines 18-22 (the comment and the decay line):
```c
        /* Leaky decay of the fail-streak bucket. Precisely:
         *   - the first SENSOR_DECAY_AFTER-1 goods of a run forgive nothing;
         *   - every consecutive good from the SENSOR_DECAY_AFTER'th onward forgives
         *     exactly one earlier fail (fail_streak--), floored at 0;
         *   - good_streak is NOT reset by a decay, so a long good run keeps decaying
         *     once per read; only a bad read resets it to 0.
         * Break-even is therefore one fail in SENSOR_DECAY_AFTER+1 reads: 25 % at 3.
         * A worse-than-25 % failure rate still climbs to the latch threshold; a rarer
         * isolated glitch is forgiven and never accumulates. The 1.2.1 value of 2 put
         * break-even at 33 %, so a probe failing a third of its sweeps never latched. */
        if (s->good_streak >= SENSOR_DECAY_AFTER && s->fail_streak > 0) s->fail_streak--;
```

- [ ] **Step 5: test-runner — full host suite green**

```bash
cmake --build /tmp/vc-tests -j8 && ctest --test-dir /tmp/vc-tests --output-on-failure
```
Expected: `100% tests passed out of 13`.

- [ ] **Step 6: Commit**

```bash
git add -- firmware/components/ctrl_core/include/ctrl_core/sensor_policy.h \
  firmware/components/ctrl_core/sensor_policy.c firmware/test_host/test_sensor_policy.c \
&& git commit -m "fix(fw): require 3 consecutive goods to decay a fault streak (25% break-even)" \
  -- firmware/components/ctrl_core/include/ctrl_core/sensor_policy.h \
     firmware/components/ctrl_core/sensor_policy.c firmware/test_host/test_sensor_policy.c
```

---

### Task 2: F4 — extract the OTA validation gate and move it outside the WDT window

`control_task.c:81-84` validates the OTA image after 3 completed cycles ≈ 20 s — **inside** the 30 s task-WDT window, so the comment at `:25-29` and the README claim ("a hang trips the 30 s task WDT and reboots, which rolls back on its own") are inverted: the gate fires before the watchdog could ever catch a hang. Raise to 12 cycles (~110 s) and extract the one-shot logic into a host-testable ctrl_core function.

**Files:**
- Create: `firmware/components/ctrl_core/include/ctrl_core/ota_gate.h`
- Create: `firmware/components/ctrl_core/ota_gate.c`
- Create: `firmware/test_host/test_ota_gate.c`
- Modify: `firmware/main/control_task.c:25-29,81-84`
- Modify: `firmware/README.md:64-71`

**Interfaces:**
- Produces: `bool ota_gate_step(uint32_t *cycles, uint32_t threshold)` — call once per completed control cycle; returns `true` exactly once, on the call that brings `*cycles` up to `threshold`. `*cycles` saturates at `threshold` (never wraps, never fires twice). `threshold == 0` never fires.
- Produces: `#define OTA_GATE_CYCLES 12` in `control_task.c` (file-local).

- [ ] **Step 1: Write the failing test**

Create `firmware/test_host/test_ota_gate.c`:
```c
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
```

- [ ] **Step 2: test-runner — expect configure/compile FAIL**

New test file ⇒ the CMake **configure** step must re-run to pick it up via the GLOB:
```bash
rm -rf /tmp/vc-tests && cmake -S firmware/test_host -B /tmp/vc-tests \
  -DFETCHCONTENT_SOURCE_DIR_UNITY=/Users/kleist/Sites/ValveController/firmware/test_host/build-121a/_deps/unity-src \
&& cmake --build /tmp/vc-tests -j8
```
Expected: FAIL — `fatal error: ctrl_core/ota_gate.h: No such file or directory`.

- [ ] **Step 3: Implement the ctrl_core unit**

Create `firmware/components/ctrl_core/include/ctrl_core/ota_gate.h`:
```c
#pragma once
#include "ctrl_core/types.h"

/* One-shot cycle counter for the OTA rollback-validation gate.
 *
 * Lives here rather than inline in control_task.c so the "fires exactly once, and
 * only at the threshold" property is host-testable: getting it wrong either
 * validates an image early (silently defeating rollback protection) or never
 * validates it (rollback loop). Neither failure is observable on a bench without
 * performing a real OTA.
 *
 * Call once per completed control cycle. *cycles saturates at threshold, so it
 * never wraps and never fires a second time. threshold == 0 never fires. */
bool ota_gate_step(uint32_t *cycles, uint32_t threshold);
```

Create `firmware/components/ctrl_core/ota_gate.c`:
```c
#include "ctrl_core/ota_gate.h"

bool ota_gate_step(uint32_t *cycles, uint32_t threshold){
    if (*cycles >= threshold) return false;   /* already fired, or threshold 0 */
    (*cycles)++;
    return *cycles == threshold;
}
```

No CMake edits needed: `components/ctrl_core/CMakeLists.txt` GLOBs `*.c`, `test_host/CMakeLists.txt` GLOBs both `test_*.c` and `../components/ctrl_core/*.c`.

- [ ] **Step 4: test-runner — full host suite green (now 14 tests)**

```bash
cmake --build /tmp/vc-tests -j8 && ctest --test-dir /tmp/vc-tests --output-on-failure
```
Expected: `100% tests passed out of 14`.

- [ ] **Step 5: Wire it into control_task.c**

Add the include after `#include "ota.h"` (line 10):
```c
#include "ctrl_core/ota_gate.h"
```

Replace lines 25-29 (the comment block and `static uint8_t s_cycles_completed = 0;`):
```c
/* Completed control cycles, saturating at OTA_GATE_CYCLES (see ctrl_core/ota_gate.h).
 * Sensor faults are a property of the plant wiring, not of the image, so OTA
 * validation must not wait on fault-free probes. What this gate DOES prove is that
 * the control task is still looping -- and only if it takes longer than the task
 * watchdog: OTA_GATE_CYCLES * CYCLE_MS is ~110 s, comfortably OUTSIDE the 30 s window
 * (CONFIG_ESP_TASK_WDT_TIMEOUT_S=30, panic on), so a hung control task trips the WDT
 * -> reset -> rollback long before this gate could validate. At the previous 3 cycles
 * (~20 s) the gate fired INSIDE the watchdog window and therefore proved nothing. */
#define OTA_GATE_CYCLES 12
static uint32_t s_cycles_completed = 0;
```

Replace lines 79-84 (the fast-path comment and the `if (s_cycles_completed < 3)` block):
```c
        /* OTA validation fast path: don't let a missing/faulted probe delay validation
         * all the way out to the 10-minute fallback timer in ota.c. */
        if (ota_gate_step(&s_cycles_completed, OTA_GATE_CYCLES)) ota_note_good_sweep();
```

- [ ] **Step 6: Fix the inverted README claim**

In `firmware/README.md`, replace list item 2 of the OTA validation section (lines 68-71):
```markdown
  2. `OTA_GATE_CYCLES` (12) completed control cycles regardless of sensor faults —
     sensor faults are a plant-wiring property, not an image property. 12 cycles at
     the 10 s control period is ~110 s, deliberately **longer** than the 30 s task
     watchdog timeout (`CONFIG_ESP_TASK_WDT_TIMEOUT_S=30`, panic on): a control task
     that hangs trips the watchdog and reboots — rolling back — *before* this gate
     could ever validate the image. (1.2.1 used 3 cycles ≈ 20 s, which fired inside
     the watchdog window and so proved nothing.)
```

- [ ] **Step 7: test-runner — suite still green**

```bash
ctest --test-dir /tmp/vc-tests --output-on-failure
```
Expected: `100% tests passed out of 14`.

- [ ] **Step 8: Commit**

```bash
git add -- firmware/components/ctrl_core/include/ctrl_core/ota_gate.h \
  firmware/components/ctrl_core/ota_gate.c firmware/test_host/test_ota_gate.c \
  firmware/main/control_task.c firmware/README.md \
&& git commit -m "fix(fw): move OTA validation gate outside the 30 s WDT window (12 cycles), extract to ctrl_core" \
  -- firmware/components/ctrl_core/include/ctrl_core/ota_gate.h \
     firmware/components/ctrl_core/ota_gate.c firmware/test_host/test_ota_gate.c \
     firmware/main/control_task.c firmware/README.md
```

---

### Task 3: F8 — freeze the supply alarm while the supply probe is faulted

`control.c:88` runs `alarm_supply_step()` unconditionally. With `faults.supply` set, `in->t_supply` is the last-good value — or `0.0` straight from BSS if the probe never read once. `0.0 <= ALARM_SUPPLY_LOW (15.5)` latches a **false freeze alarm** after the dwell; a frozen in-band value **falsely clears** a real one.

**Files:**
- Modify: `firmware/components/ctrl_core/control.c:87-88`
- Test: `firmware/test_host/test_control.c`

**Interfaces:**
- Consumes: `alarm_state_t` fields `alarmed`, `out_of_bounds`, `oob_since_ms` (`ctrl_core/alarm.h`), already reachable via `s->alarm`.
- Produces: no signature change. `control_out_t.supply_alarm` now holds its last value while `in->faults.supply` is set.

- [ ] **Step 1: Write the failing tests**

Append to `firmware/test_host/test_control.c`, before `main()`:
```c
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
```

Register in `main()`:
```c
    RUN_TEST(test_supply_fault_blocks_false_freeze_alarm);
    RUN_TEST(test_supply_fault_holds_existing_alarm);
    RUN_TEST(test_supply_fault_rearms_dwell_on_recovery);
```

- [ ] **Step 2: test-runner — expect `test_control` to FAIL**

```bash
ctest --test-dir /tmp/vc-tests --output-on-failure -R test_control
```
Expected: FAIL — `test_supply_fault_blocks_false_freeze_alarm` asserts TRUE where FALSE expected at the 600000 step (0.0 °C latched a freeze alarm).

- [ ] **Step 3: Implement**

In `firmware/components/ctrl_core/control.c`, replace lines 87-88:
```c
    /* Telemetry (always). */
    o.supply_alarm = alarm_supply_step(&s->alarm, in->t_supply, cfg->alarm_dwell_ms, now);
```
with:
```c
    /* Telemetry (always). The supply freeze alarm is only meaningful on a LIVE supply
     * reading: with the probe faulted, in->t_supply is the last-good value, or 0.0
     * straight from BSS if it never read once. 0.0 <= ALARM_SUPPLY_LOW would latch a
     * false freeze alarm after the dwell, and a frozen in-band value would falsely
     * clear a real one. So hold the current alarm state and re-arm the excursion dwell,
     * making it restart clean when the probe recovers. */
    if (in->faults.supply){
        s->alarm.out_of_bounds = false;
        s->alarm.oob_since_ms  = now;
        o.supply_alarm = s->alarm.alarmed;
    } else {
        o.supply_alarm = alarm_supply_step(&s->alarm, in->t_supply, cfg->alarm_dwell_ms, now);
    }
```

- [ ] **Step 4: test-runner — full host suite green**

```bash
cmake --build /tmp/vc-tests -j8 && ctest --test-dir /tmp/vc-tests --output-on-failure
```
Expected: `100% tests passed out of 14`.

- [ ] **Step 5: Commit**

```bash
git add -- firmware/components/ctrl_core/control.c firmware/test_host/test_control.c \
&& git commit -m "fix(fw): hold supply freeze alarm while the supply probe is faulted" \
  -- firmware/components/ctrl_core/control.c firmware/test_host/test_control.c
```

---

### Task 4: F7a + F12 — passive link liveness, and volatile cross-task state

**Ordering note (deliberate deviation from "ctrl_core first"):** this `main/` task MUST land **before** Task 5. Task 5 makes the cooling dew guard fire purely on `link_last_seen_ms`. Today that timestamp is only refreshed by ZDO signals, so shipping Task 5 first would let a long-joined device with no ZDO events look "quiet for 30 min" and spuriously raise the cooling setpoint. This task makes the timestamp truthful first, and on its own is a strict no-op-or-safer change (refreshing the timestamp more often only makes the *existing* guard less likely to fire).

**Files:**
- Modify: `firmware/main/control_task.h`
- Modify: `firmware/main/control_task.c:15-24,99`
- Modify: `firmware/main/zigbee.c:448-453`

**Interfaces:**
- Produces: `void control_task_note_link_activity(void)` — refreshes `s_link_seen` to now. Called from `zigbee.c`'s `action_handler` on every inbound core action callback. Task 5's `cooling_link_guard()` consumes the resulting `control_in_t.link_last_seen_ms`.

- [ ] **Step 1: Declare the new setter**

In `firmware/main/control_task.h`, add after `void control_task_set_link(bool up, uint32_t last_seen_ms);`:
```c
void        control_task_note_link_activity(void);   /* refresh liveness on inbound ZCL traffic */
```

- [ ] **Step 2: Make the cross-task statics volatile (F12) and add the setter (F7a)**

In `firmware/main/control_task.c`, replace lines 15-24:
```c
/* Cross-task state. Written by the Zigbee stack task (attr_cb, signal handler) and the
 * console task, read by the control loop -- and s_mode/s_alarm/s_faults the other way
 * round. volatile matches the ui.c:13 idiom for exactly this pattern. It is NOT
 * atomicity, but each of these is a single naturally-aligned word on RV32, and volatile
 * is what stops the compiler caching them in a register across the 10 s loop. */
static volatile ctrl_mode_t s_mode = MODE_IDLE;
static volatile bool     s_alarm = false;
static volatile uint16_t s_faults = 0;
static volatile bool     s_water_running = false;      /* HA enables regulation */
static volatile bool     s_link_up = false;
static volatile uint32_t s_link_seen = 0;
/* One-shot: set when an AnalogOutput manual override write is accepted (zigbee.c),
 * cleared on any water_running transition. While set, the OFF-park loop below leaves
 * the valve alone (spec §4.5: write supersedes park_pos until water_running ON or reboot). */
static volatile bool s_override_active = false;
```
(`s_ctrl` and `s_cycles_completed` stay non-volatile — both are touched only by the control loop, and `ota_gate_step()` takes a plain `uint32_t *`.)

Replace the one-line `control_task_set_link` definition near the end of the file —
```c
void control_task_set_link(bool up, uint32_t seen){ s_link_up = up; s_link_seen = seen; }
```
— with the two functions below:
```c
void control_task_set_link(bool up, uint32_t seen){
    /* seen BEFORE up: a reader that interleaves then sees a stale timestamp alongside
     * the new link state, never a fresh timestamp alongside the old one. The cooling
     * dew guard keys off the timestamp, and "older than it really is" errs toward
     * raising the cooling setpoint -- less cooling, the safe direction. */
    s_link_seen = seen;
    s_link_up = up;
}

/* Passive liveness. ZDO signals alone are not enough: a Router whose coordinator dies
 * silently gets no LEAVE and no failed STEERING, so s_link_up stays true forever. Any
 * inbound ZCL action is positive proof the coordinator is still talking to us. */
void control_task_note_link_activity(void){ s_link_seen = now_ms(); }
```

- [ ] **Step 3: Call it from the Zigbee action handler**

In `firmware/main/zigbee.c`, replace `action_handler` (lines 448-453):
```c
static esp_err_t action_handler(esp_zb_core_action_callback_id_t id, const void *msg)
{
    /* Any inbound core action -- attribute write, OTA block, reporting-config, command
     * callback -- proves the coordinator is alive. This is the broadest hook the stack
     * offers; plain attribute READS are answered inside the stack and do NOT reach here,
     * so the liveness signal is only as good as the traffic Z2M actually generates
     * (see Unresolved Q3: availability polling). */
    control_task_note_link_activity();
    if (id == ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID)    return attr_cb(msg);
    if (id == ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID) return ota_zcl_handle(msg);
    return ESP_OK;
}
```

- [ ] **Step 4: test-runner — host suite unaffected but must still be green**

`main/` is not host-compiled; this confirms nothing regressed.
```bash
ctest --test-dir /tmp/vc-tests --output-on-failure
```
Expected: `100% tests passed out of 14`.

- [ ] **Step 5: Commit**

```bash
git add -- firmware/main/control_task.h firmware/main/control_task.c firmware/main/zigbee.c \
&& git commit -m "fix(fw): track inbound ZCL traffic as link liveness; mark cross-task state volatile" \
  -- firmware/main/control_task.h firmware/main/control_task.c firmware/main/zigbee.c
```

---

### Task 5: F7 — cooling dew guard triggers on traffic silence, not on `link_up`

`alarm.c:21` requires `!link_up`, but `s_link_up` only changes on ZDO signals. A Router whose coordinator dies silently never gets one — so the guard is unreachable in its single most likely scenario. Drop the `link_up` requirement: silence for `LINK_LOSS_COOLING_MS` is the trigger, and Task 4 made that timestamp reflect real traffic.

**Files:**
- Modify: `firmware/components/ctrl_core/include/ctrl_core/alarm.h`
- Modify: `firmware/components/ctrl_core/alarm.c:19-24`
- Modify: `firmware/components/ctrl_core/control.c:108-109`
- Modify: `firmware/components/ctrl_core/include/ctrl_core/control.h` (comment only)
- Test: `firmware/test_host/test_alarm.c`

**Interfaces:**
- Produces: `float cooling_link_guard(float cool_setpoint, ctrl_mode_t mode, uint32_t last_seen_ms, uint32_t now_ms)` — the `bool link_up` parameter is **removed**. `control_in_t.link_up` is kept (still set by `control_task.c`, still reported) but is no longer consumed by the guard; `control.h` records that explicitly.

- [ ] **Step 1: Update the tests to the new signature and add the silent-coordinator case**

In `firmware/test_host/test_alarm.c`, replace the two link-guard tests:
```c
/* Cooling dew guard: 30 min without inbound traffic in COOLING raises setpoint to 21.
 * This is the silently-dead-coordinator case -- there is no ZDO signal to key off, so
 * traffic silence alone must be enough. */
void test_link_guard_raises(void){
    TEST_ASSERT_EQUAL_FLOAT(21.0f, cooling_link_guard(18.0f, MODE_COOLING, 0, 1800000));
}

/* Recent traffic keeps the guard inactive no matter how long the device has been up. */
void test_link_guard_inactive_when_traffic_recent(void){
    TEST_ASSERT_EQUAL_FLOAT(18.0f, cooling_link_guard(18.0f, MODE_COOLING, 86400000u, 86401000u));
}

/* Inactive in heating, inside the window, at boot, and it never LOWERS a setpoint. */
void test_link_guard_inactive(void){
    TEST_ASSERT_EQUAL_FLOAT(35.0f, cooling_link_guard(35.0f, MODE_HEATING, 0, 1800000));
    TEST_ASSERT_EQUAL_FLOAT(18.0f, cooling_link_guard(18.0f, MODE_COOLING, 0, 1799000));
    TEST_ASSERT_EQUAL_FLOAT(18.0f, cooling_link_guard(18.0f, MODE_COOLING, 0, 0));
    TEST_ASSERT_EQUAL_FLOAT(24.0f, cooling_link_guard(24.0f, MODE_COOLING, 0, 1800000));
}
```

Register the new one in `main()`, after `RUN_TEST(test_link_guard_raises);`:
```c
    RUN_TEST(test_link_guard_inactive_when_traffic_recent);
```

- [ ] **Step 2: test-runner — expect compile FAIL**

```bash
ctest --test-dir /tmp/vc-tests --output-on-failure -R test_alarm
```
Expected: build FAIL — too few arguments to `cooling_link_guard`.

- [ ] **Step 3: Implement**

`firmware/components/ctrl_core/include/ctrl_core/alarm.h` — replace the declaration:
```c
/* Autonomous dew-point guard. Raises the cooling setpoint toward LINK_LOSS_COOL_SETPOINT
 * once we have not heard from the coordinator for LINK_LOSS_COOLING_MS, so an unattended
 * device cannot keep chilling a floor below dew point. Deliberately NOT gated on a ZDO
 * link-down signal: a Router whose coordinator dies silently never emits one, which is
 * precisely the scenario this exists for. last_seen_ms is refreshed by inbound ZCL
 * traffic AND by join/leave signals (control_task_note_link_activity / _set_link).
 * Only ever raises the setpoint -- never lowers it. */
float cooling_link_guard(float cool_setpoint, ctrl_mode_t mode,
                         uint32_t last_seen_ms, uint32_t now_ms);
```

`firmware/components/ctrl_core/alarm.c` — replace lines 19-24:
```c
float cooling_link_guard(float cool_setpoint, ctrl_mode_t mode,
                         uint32_t last_seen_ms, uint32_t now){
    if (mode == MODE_COOLING && (now - last_seen_ms) >= LINK_LOSS_COOLING_MS)
        return fmaxf(cool_setpoint, LINK_LOSS_COOL_SETPOINT);
    return cool_setpoint;
}
```

`firmware/components/ctrl_core/control.c` — replace lines 108-109:
```c
    float eff_cool = cooling_link_guard(cfg->cool_setpoint, mode,
                                        in->link_last_seen_ms, now);
```

`firmware/components/ctrl_core/include/ctrl_core/control.h` — replace the two link fields in `control_in_t`:
```c
    bool  link_up;                  /* ZDO-observed join state; STATUS ONLY -- the cooling
                                       dew guard deliberately does not consume it, since a
                                       silently dead coordinator leaves it true forever */
    uint32_t link_last_seen_ms;     /* last inbound ZCL activity or ZDO signal */
```

- [ ] **Step 4: test-runner — full host suite green**

```bash
cmake --build /tmp/vc-tests -j8 && ctest --test-dir /tmp/vc-tests --output-on-failure
```
Expected: `100% tests passed out of 14`.

- [ ] **Step 5: Commit**

```bash
git add -- firmware/components/ctrl_core/include/ctrl_core/alarm.h \
  firmware/components/ctrl_core/alarm.c firmware/components/ctrl_core/control.c \
  firmware/components/ctrl_core/include/ctrl_core/control.h firmware/test_host/test_alarm.c \
&& git commit -m "fix(fw): trigger cooling dew guard on traffic silence instead of ZDO link state" \
  -- firmware/components/ctrl_core/include/ctrl_core/alarm.h \
     firmware/components/ctrl_core/alarm.c firmware/components/ctrl_core/control.c \
     firmware/components/ctrl_core/include/ctrl_core/control.h firmware/test_host/test_alarm.c
```

---

### Task 6: F1 — persist water_running and keep the OnOff attribute in sync

`app_main.c:45` forces `control_task_set_water_running(true)` ("bench default") at every boot, while `zigbee.c:212` builds the OnOff cluster with `.on_off = 0` and nothing ever syncs them. After a power blip the device regulates a live heating loop while HA shows OFF. Persist the last commanded state in its own small NVS key, restore it before the endpoints are built, and write through on every OnOff write.

**Design:** a **separate NVS key**, not a field in the `cfg` blob — a new blob field would change `sizeof(config_t)` and force a `CONFIG_VERSION` bump plus migration for a value that is commanded state, not a tunable. **Boot default when the key is absent: `false`** (device parks at `park_pos` — safe, and consistent with the OnOff attribute default).

**Files:**
- Modify: `firmware/main/config.h`
- Modify: `firmware/main/config.c`
- Modify: `firmware/main/app_main.c:45`
- Modify: `firmware/main/zigbee.c:212,351-354`

**Interfaces:**
- Produces: `bool config_water_running_load(void)` — reads NVS key `"water"` in namespace `"valvectl"`; returns `false` when absent/unreadable.
- Produces: `void config_water_running_save(bool on)` — writes + commits that key; silently no-ops if NVS is unavailable.
- Requires: `config_load()` (which runs `nvs_flash_init()`) must have been called first — `app_main()` already does this at line 40.

- [ ] **Step 1: Declare the API**

In `firmware/main/config.h`, add after the `config_apply_custom` declaration:
```c
/* water_running (commanded regulation enable) is persisted as its OWN small NVS key,
 * deliberately NOT a field in the cfg blob: a new blob field would change
 * sizeof(config_t) and force a CONFIG_VERSION bump + migration for what is commanded
 * state, not a tunable. Absent key -> false (park at park_pos: the safe boot state,
 * and the value the OnOff attribute is built with). */
bool      config_water_running_load(void);
void      config_water_running_save(bool on);
```

- [ ] **Step 2: Implement it**

In `firmware/main/config.c`, add after the `#define NS "valvectl"` line (line 10):
```c
#define KEY_WATER "water"
```
and append at the end of the file:
```c
bool config_water_running_load(void)
{
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) return false;
    uint8_t v = 0;
    esp_err_t e = nvs_get_u8(h, KEY_WATER, &v);
    nvs_close(h);
    if (e != ESP_OK) {
        ESP_LOGI(TAG, "no persisted water_running, defaulting OFF (park)");
        return false;
    }
    return v != 0;
}

void config_water_running_save(bool on)
{
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READWRITE, &h) != ESP_OK) return;
    if (nvs_set_u8(h, KEY_WATER, on ? 1 : 0) == ESP_OK) nvs_commit(h);
    nvs_close(h);
}
```
(`config_factory_reset()` already calls `nvs_erase_all()` on this namespace, which clears the key too — a factory reset therefore boots OFF. Correct.)

- [ ] **Step 3: Restore at boot instead of forcing true**

In `firmware/main/app_main.c`, replace line 45:
```c
    /* Restore the last state HA commanded. Forcing true here (the old "bench default")
     * meant that after any power blip the device regulated a live loop while the OnOff
     * attribute -- and therefore HA -- still read OFF. Must run BEFORE zigbee_start():
     * build_endpoints() seeds the OnOff attribute from control_task_water_running(). */
    control_task_set_water_running(config_water_running_load());
```

- [ ] **Step 4: Seed the OnOff attribute from the restored state and write through on writes**

In `firmware/main/zigbee.c`, replace line 212:
```c
    /* Seeded from the NVS-restored value (app_main sets it before zigbee_start()), so
     * reads and reports of OnOff always agree with what the control task is doing. */
    esp_zb_on_off_cluster_cfg_t oncfg = { .on_off = control_task_water_running() };
```

Replace the OnOff branch of `attr_cb` (lines 351-354):
```c
    if (m->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
        bool on = *(bool*)m->attribute.data.value;
        /* Write-through, with no-op elision: HA re-sending the same state must not
         * erase/write NVS from the Zigbee stack task on every message. */
        if (on != control_task_water_running()) {
            control_task_set_water_running(on);
            config_water_running_save(on);
        }
        return ESP_OK;
    }
```

- [ ] **Step 5: test-runner — host suite still green**

```bash
ctest --test-dir /tmp/vc-tests --output-on-failure
```
Expected: `100% tests passed out of 14`.

- [ ] **Step 6: Commit**

```bash
git add -- firmware/main/config.h firmware/main/config.c firmware/main/app_main.c firmware/main/zigbee.c \
&& git commit -m "fix(fw): persist water_running in NVS and seed the OnOff attribute from it" \
  -- firmware/main/config.h firmware/main/config.c firmware/main/app_main.c firmware/main/zigbee.c
```

---

### Task 7: F2 — publish the ZCL invalid sentinel for faulted probes

`zigbee.c:506` and `:557` publish `sensors_get(id).value_c` ignoring `.fault`, so a dead probe's last-good value (or BSS `0.0`) is reported forever as a live measurement. Publish `0x8000` — the ZCL invalid sentinel for both Temperature Measurement `MeasuredValue` and Thermostat `LocalTemperature` — and make the z2m converter surface it as `null`.

**Files:**
- Modify: `firmware/main/zigbee.c:498-511,554-561`
- Modify: `z2m/valvectl.mjs:163` + new converter

- [ ] **Step 1: Add the sentinel helper in zigbee.c**

Insert immediately above `void zigbee_report_temps(void)` (line 498):
```c
/* ZCL "invalid" sentinel for the int16 temperature attributes -- Temperature
 * Measurement MeasuredValue and Thermostat LocalTemperature both define 0x8000
 * (== -32768) as "value not available". add_temp_ep() already seeds the cluster
 * with it, so the stack accepts it. */
#define ZCL_TEMP_INVALID ((int16_t)0x8000)

/* A latched-faulted probe holds its last-good value forever -- or 0.0 from BSS if it
 * never produced one. Publishing that as a live measurement is a lie the coordinator
 * cannot detect: HA charts a plausible frozen temperature and no automation notices.
 * Publish the sentinel instead. */
static int16_t temp_centi(sensor_id_t id)
{
    sensor_reading_t r = sensors_get(id);
    if (r.fault) return ZCL_TEMP_INVALID;
    return (int16_t)(r.value_c * 100.0f);
}
```

- [ ] **Step 2: Use it in both publish paths**

In `zigbee_report_temps()`, replace line 506:
```c
        int16_t v = temp_centi(map[i].id);
```

In `set_local_temperature()`, replace line 557:
```c
    int16_t lt = temp_centi(SENS_SUPPLY);
```

- [ ] **Step 3: Make the z2m converter surface the sentinel as null**

In `z2m/valvectl.mjs`, insert above `const fzWaterRunning = {` (line 128):
```js
// The generic fz.temperature has no guard for the ZCL invalid sentinel and would
// publish -327.68 °C as a real reading. The firmware sends 0x8000 (-32768) for any
// probe whose fault is latched (temp_centi() in firmware/main/zigbee.c).
const fzTemperature = {
    cluster: 'msTemperatureMeasurement', type: ['attributeReport', 'readResponse'],
    convert: (model, msg, publish, options, meta) => {
        if (msg.data.measuredValue === undefined) return;
        // multiEndpoint postfix built explicitly rather than via postfixWithEndpointName(),
        // whose signature has changed across zigbee-herdsman-converters versions. The
        // endpoint map below names endpoints '2'..'6' identically to their IDs, so this
        // produces exactly the `temperature_<ep>` properties the exposes declare.
        const property = `temperature_${msg.endpoint.ID}`;
        if (msg.data.measuredValue === -32768) return {[property]: null};
        return {[property]: Math.round(msg.data.measuredValue) / 100};
    },
};
```

Replace the `fromZigbee` line (line 163):
```js
    fromZigbee: [fzWaterRunning, fzTemperature, fz.thermostat, fzRunningMode, fzAnalogOutput, fzCustom],
```

`fz.thermostat` handles `localTemp` and current upstream versions already drop values below −273.15 °C, and `local_temperature` is not in `exposes` anyway — so no change is needed there. Confirm at deploy time against the installed converter version (Unresolved Q6).

- [ ] **Step 4: test-runner — host suite still green**

```bash
ctest --test-dir /tmp/vc-tests --output-on-failure
```
Expected: `100% tests passed out of 14`.

- [ ] **Step 5: Commit**

```bash
git add -- firmware/main/zigbee.c z2m/valvectl.mjs \
&& git commit -m "fix(fw): publish ZCL invalid sentinel 0x8000 for faulted temperature probes" \
  -- firmware/main/zigbee.c z2m/valvectl.mjs
```

---

### Task 8: F3 + F16 + F17a — sweep watchdog, staleness fault, coherent snapshot

`sensors_hw.c:90` `sweep_task` is not registered with the task WDT and `last_ok_ms` (written at `:77`) is never checked, so a wedged sweep leaves every reading frozen with `fault == false` and the control loop keeps integrating on stale data forever. `sensors_fill_faults()` at `:131` also takes `g_lock` five times, so the control loop can get a torn snapshot. Fix all three in one locked pass, and stop discarding the `esp_task_wdt_add()` return in the other two tasks.

**Files:**
- Modify: `firmware/main/sensors_hw.c:1-10,22,90-92,131-138`
- Modify: `firmware/main/control_task.c:34`
- Modify: `firmware/main/valve_hw.c:57`

- [ ] **Step 1: Register the sweep task with the watchdog**

In `firmware/main/sensors_hw.c`, add after `#include "esp_log.h"` (line 7):
```c
#include "esp_task_wdt.h"
```

Replace lines 90-92:
```c
static void sweep_task(void *arg)
{
    /* Subscribed to the task WDT like the control and valve tasks: an RMT/1-Wire wedge
     * here used to be invisible (readings simply froze) rather than causing the reset +
     * rollback the watchdog exists to produce. One iteration is ~10 s (750 ms convert +
     * 9.25 s delay), well inside the 30 s budget. */
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    for (;;) {
        esp_task_wdt_reset();
```

- [ ] **Step 2: Add the staleness constant**

In `firmware/main/sensors_hw.c`, add after `#define EMA_TAU_S 40.0f` (line 25):
```c
/* Belt-and-braces for a sweep that stops producing readings without failing them
 * (task starved, RMT wedged, sweep_task deleted). Independent of the latch state:
 * a reading whose last good sweep is older than this is reported faulted, so control
 * degrades to park instead of silently integrating frozen data. 3 sweep periods plus
 * 5 s of slack. */
#define SENSOR_STALE_MS (3u * SWEEP_PERIOD_MS + 5000u)   /* 35 s */
```

- [ ] **Step 3: One locked pass, with the staleness check folded in**

Replace `sensors_fill_faults()` (lines 131-138) entirely:
```c
/* One locked pass. Taking g_lock five times (once per sensors_get) let the sweep task
 * interleave and hand the control loop a mix of pre- and post-sweep fault state. */
void sensors_fill_faults(sensor_faults_t *o)
{
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    bool f[SENS_COUNT];

    xSemaphoreTake(g_lock, portMAX_DELAY);
    for (int i = 0; i < SENS_COUNT; ++i) {
        /* last_ok_ms == 0 means "never read a good value"; sensors_start() already boots
         * that sensor as latched-faulted, so no staleness test is needed (and applying
         * one would be wrong -- now - 0 is just uptime). */
        bool stale = (g_read[i].last_ok_ms != 0) &&
                     ((now - g_read[i].last_ok_ms) > SENSOR_STALE_MS);
        f[i] = g_read[i].fault || stale;
    }
    xSemaphoreGive(g_lock);

    o->supply = f[SENS_SUPPLY];
    o->ret    = f[SENS_RETURN];
    o->source = f[SENS_SOURCE];
    o->hx_a   = f[SENS_HX_A];
    o->hx_b   = f[SENS_HX_B];
}
```

- [ ] **Step 4: Stop discarding the other two `esp_task_wdt_add()` returns (F17a)**

In `firmware/main/control_task.c`, the first line of `control_loop()` — `    esp_task_wdt_add(NULL);` — becomes:
```c
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
```
(`esp_err.h` is reachable via the existing `esp_task_wdt.h` include.)

In `firmware/main/valve_hw.c`, the first line of `valve_task()` — `    esp_task_wdt_add(NULL);` — becomes:
```c
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
```

- [ ] **Step 5: test-runner — host suite still green**

```bash
ctest --test-dir /tmp/vc-tests --output-on-failure
```
Expected: `100% tests passed out of 14`.

- [ ] **Step 6: Commit**

```bash
git add -- firmware/main/sensors_hw.c firmware/main/control_task.c firmware/main/valve_hw.c \
&& git commit -m "fix(fw): watchdog the sensor sweep, fault stale readings, snapshot faults under one lock" \
  -- firmware/main/sensors_hw.c firmware/main/control_task.c firmware/main/valve_hw.c
```

---

### Task 9: F6 + F9 — echo clamped thermostat setpoints back, elide no-op saves

`zigbee.c:358-365` clamps the heating/cooling setpoint into `g_config` and saves, but never echoes the clamped value into the ZCL attribute store — unlike every custom-cluster tunable at `:396-442`. The stack has already latched the raw write, so a 40 °C write leaves HA reading a setpoint the device will never honour. The same branches also call `config_save()` unconditionally, so every HA re-send erases/writes NVS from the Zigbee stack task.

**Files:**
- Modify: `firmware/main/zigbee.c:355-367`

- [ ] **Step 1: Implement both fixes**

Replace the thermostat branch of `attr_cb` (lines 355-367):
```c
    if (m->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT) {
        if (m->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID)
            return ESP_OK;   /* accepted-but-ignored: mode is auto-detected */
        /* Setpoints are clamped to the same 17..35 C band control_task.c re-clamps with.
         * Two things follow, both previously missing:
         *  - echo the clamped value back UNCONDITIONALLY: the stack already latched the
         *    raw write into the attribute store, so without this a 40 C write leaves the
         *    store at 4000 and HA reads back a setpoint the device will never honour.
         *    Safe to call from here -- this runs in the stack task, same as the
         *    ATTR_RESYNC self-clear below;
         *  - only config_save() when g_config actually changed, so HA re-sending the
         *    same setpoint (or one that clamps back to the current value) does not
         *    erase/write NVS from the stack task on every message. */
        if (m->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID) {
            float c = ctrl_clampf(*(int16_t*)m->attribute.data.value / 100.0f, 17.0f, 35.0f);
            if (c != g_config.heat_setpoint) { g_config.heat_setpoint = c; config_save(); }
            int16_t echo = (int16_t)(c * 100.0f + 0.5f);   /* c >= 17, so +0.5 rounds */
            esp_zb_zcl_set_attribute_val(EP_MAIN, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID, &echo, false);
        }
        if (m->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID) {
            float c = ctrl_clampf(*(int16_t*)m->attribute.data.value / 100.0f, 17.0f, 35.0f);
            if (c != g_config.cool_setpoint) { g_config.cool_setpoint = c; config_save(); }
            int16_t echo = (int16_t)(c * 100.0f + 0.5f);
            esp_zb_zcl_set_attribute_val(EP_MAIN, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID, &echo, false);
        }
        return ESP_OK;
    }
```

- [ ] **Step 2: test-runner — host suite still green**

```bash
ctest --test-dir /tmp/vc-tests --output-on-failure
```
Expected: `100% tests passed out of 14`.

- [ ] **Step 3: Commit**

```bash
git add -- firmware/main/zigbee.c \
&& git commit -m "fix(fw): echo clamped thermostat setpoints back to the attribute store, elide no-op saves" \
  -- firmware/main/zigbee.c
```

---

### Task 10: F11 — latch travel_time_s alongside the direction swap

`valve_hw.c:67` recomputes the resync stall time live from `g_config.travel_time_s` while `s_swap_latched` (`:88`) is frozen. A mid-stroke `travel_time_s` write collapses the stall time and corrupts the position estimator, which reads the same live value at `:90`. Latch both together.

**Files:**
- Modify: `firmware/main/valve_hw.c:27-30,67,86-90,101`

- [ ] **Step 1: Add the latch**

In `firmware/main/valve_hw.c`, replace lines 27-30 (the `s_swap_latched` comment and declaration):
```c
/* Latched copies of the motion-affecting config, refreshed only while idle (see
 * valve_task), so a runtime config change never flips the open/close mapping nor
 * rescales the stall/position math mid-stroke or mid-resync. */
static bool              s_swap_latched = false;
static uint32_t          s_travel_latched_s = 0;
```

- [ ] **Step 2: Read the latch in the resync stall math**

Inside the `if (s_rs == RS_DRIVING)` branch of `valve_task()`, replace
```c
            uint32_t stall_ms = (uint32_t)(g_config.travel_time_s * 1000.0f * RESYNC_STALL_MULT);
```
with
```c
            uint32_t stall_ms = (uint32_t)(s_travel_latched_s * 1000.0f * RESYNC_STALL_MULT);
```

- [ ] **Step 3: Refresh both latches at the same idle point, and feed the estimator from the latch**

Replace the idle-latch comment, its `if`, and the `pos_est_update` call that follows it —
```c
        /* Idle (not driving, not mid-resync): safe point to pick up a direction_swap change
         * for the *next* motion. While actually moving, the latch stays frozen. */
        if (applied == VALVE_STOP && s_rs == RS_IDLE) s_swap_latched = g_config.direction_swap;

        pos_est_update(&s_pos, travel_sign_of(applied), TICK_MS, (float)g_config.travel_time_s);
```
— with:
```c
        /* Idle (not driving, not mid-resync): the only safe point to pick up config
         * changes for the *next* motion. While actually moving, both latches stay
         * frozen -- a travel_time_s write mid-stroke would otherwise rescale the
         * position estimator's %-per-tick under it. */
        if (applied == VALVE_STOP && s_rs == RS_IDLE) {
            s_swap_latched     = g_config.direction_swap;
            s_travel_latched_s = g_config.travel_time_s;
        }

        pos_est_update(&s_pos, travel_sign_of(applied), TICK_MS, (float)s_travel_latched_s);
```

- [ ] **Step 4: Seed the latch at boot**

In `valve_start()`, replace
```c
    s_swap_latched = g_config.direction_swap;              /* initial latch at boot */
```
with
```c
    s_swap_latched     = g_config.direction_swap;          /* initial latch at boot */
    s_travel_latched_s = g_config.travel_time_s;
```

- [ ] **Step 5: test-runner — host suite still green**

```bash
ctest --test-dir /tmp/vc-tests --output-on-failure
```
Expected: `100% tests passed out of 14`.

- [ ] **Step 6: Commit**

```bash
git add -- firmware/main/valve_hw.c \
&& git commit -m "fix(fw): latch travel_time_s with the direction swap so mid-stroke writes can't corrupt position" \
  -- firmware/main/valve_hw.c
```

---

### Task 11: F10 — fail loudly on a short first OTA payload block

`ota.c:180` silently returns `ESP_OK` when the first payload block is `<= OTA_SUBELEMENT_HDR_LEN`, with a comment claiming "wait for more" — but there is no accumulation buffer, so those bytes are simply discarded and the image is corrupted with no diagnostic. Unreachable at `max_data_size = 64` (`zigbee.c:252`), but load-bearing.

**Files:**
- Modify: `firmware/main/ota.c:179-184`

- [ ] **Step 1: Implement**

In `firmware/main/ota.c`, replace lines 179-184:
```c
        if (!s_dl_trimmed) {
            /* The 6-byte sub-element header always arrives whole in the first block:
             * max_data_size is 64 (zigbee.c), ten times this header. A shorter first
             * block means the server framed the image differently than assumed, and
             * there is NO accumulation buffer here to re-assemble it -- the old code
             * returned ESP_OK and silently dropped image bytes, producing a corrupt
             * flash write with no diagnostic. Fail visibly instead. */
            if (len <= OTA_SUBELEMENT_HDR_LEN) {
                ESP_LOGE(TAG, "first OTA payload block is %u bytes, <= the %u-byte "
                              "sub-element header; cannot trim without buffering",
                         (unsigned)len, (unsigned)OTA_SUBELEMENT_HDR_LEN);
                download_abort("short first payload block");
                return ESP_FAIL;
            }
            data += OTA_SUBELEMENT_HDR_LEN;
            len  -= OTA_SUBELEMENT_HDR_LEN;
            s_dl_trimmed = true;
        }
```

- [ ] **Step 2: test-runner — host suite still green**

```bash
ctest --test-dir /tmp/vc-tests --output-on-failure
```
Expected: `100% tests passed out of 14`.

- [ ] **Step 3: Commit**

```bash
git add -- firmware/main/ota.c \
&& git commit -m "fix(fw): abort the OTA transfer on a short first payload block instead of dropping bytes" \
  -- firmware/main/ota.c
```

---

### Task 12: F14 + F17b — nits (stale TODO, OTA size diagnostic)

No test steps: neither change is behavioural. VERIFY is "suite still green + grep clean".

**Correction to the review finding for F17b:** the finding proposed warning when `s_dl_written != s_dl_total`. That would fire on **every successful OTA**. `ota_header.image_size` is the *total .ota file size* — `tools/make_ota.py` builds it as `HEADER_LENGTH (56) + 6 + len(image)` — while `s_dl_written` counts app-binary bytes only, so a healthy transfer differs by exactly 62. Warn only on genuinely impossible accounting, and log the expected delta.

**Interlock note (F17c), no code change:** `interlock.c:15` `both_error_count` is unreachable given the current callers. It is **deliberately retained** — a zero-cost safety net against a future caller asserting both directions, which on this hardware means shorting both triacs.

**Files:**
- Modify: `firmware/main/zigbee.c:75`
- Modify: `firmware/main/ota.c:132,195-198`

- [ ] **Step 1: Delete the stale TODO (F14)**

`valve_travel_since_resync()` has existed at `valve_hw.c:126` since 1.1.x and is used at `zigbee.c:547`. In `firmware/main/zigbee.c`, replace line 75:
```c
static float    s_attr_travel_since;  /* mirrors valve_travel_since_resync(); pushed by zigbee_push_status() */
```

- [ ] **Step 2: Add the OTA size diagnostic (F17b)**

In `firmware/main/ota.c`, add below `#define OTA_SUBELEMENT_HDR_LEN 6` (line 132):
```c
#define OTA_FILE_HDR_LEN       56   /* ZCL OTA file header the stack strips before us */
```

Replace the `STATUS_CHECK` case (lines 195-198):
```c
    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_CHECK:
        /* ota_header.image_size is the TOTAL advertised .ota file size (tools/make_ota.py
         * builds it as OTA_FILE_HDR_LEN + OTA_SUBELEMENT_HDR_LEN + the app binary), while
         * s_dl_written counts app-binary bytes only -- so a healthy transfer differs by
         * exactly 62 and must NOT warn. What is never legitimate is writing zero bytes,
         * or more bytes than the whole file: both mean the trim/accounting is wrong. */
        if (s_dl_written == 0 || s_dl_written > s_dl_total)
            ESP_LOGW(TAG, "OTA accounting looks wrong: wrote %lu app bytes for a %lu-byte file",
                     (unsigned long)s_dl_written, (unsigned long)s_dl_total);
        ESP_LOGI(TAG, "OTA image received: %lu app bytes written (advertised file size %lu, "
                      "expected difference %u)",
                 (unsigned long)s_dl_written, (unsigned long)s_dl_total,
                 (unsigned)(OTA_FILE_HDR_LEN + OTA_SUBELEMENT_HDR_LEN));
        return ESP_OK;
```

- [ ] **Step 3: VERIFY — grep clean and suite green**

```bash
grep -rn "TODO" firmware/main/ firmware/components/   # expect: no hits
ctest --test-dir /tmp/vc-tests --output-on-failure
```
Expected: no TODO hits in `firmware/main/` or `firmware/components/`; `100% tests passed out of 14`.

- [ ] **Step 4: Commit**

```bash
git add -- firmware/main/zigbee.c firmware/main/ota.c \
&& git commit -m "docs(fw): drop stale travel-since TODO, correct the OTA size diagnostic" \
  -- firmware/main/zigbee.c firmware/main/ota.c
```

---

### Task 13: Release 1.3.0

**Files:**
- Modify: `firmware/version.txt`
- Modify: `firmware/README.md`

- [ ] **Step 1: Bump the version**

`firmware/version.txt` — replace the entire contents with:
```
1.3.0
```

- [ ] **Step 2: Add the release notes to the README**

In `firmware/README.md`, immediately **above** the `## Prerequisites` heading, insert:
```markdown
## 1.3.0 — review-findings release

Safety and correctness fixes from the 2026-07-31 firmware review:

- **water_running now survives a reboot.** Persisted in its own NVS key (`valvectl/water`,
  outside the cfg blob, so no `CONFIG_VERSION` bump) and used to seed the OnOff attribute
  at endpoint build. Previously every boot forced regulation ON while HA read OFF. Absent
  key ⇒ OFF (parks at `park_pos`). A factory reset therefore boots OFF.
- **Faulted probes publish the ZCL invalid sentinel `0x8000`** instead of a frozen
  last-good value, on both Temperature Measurement `MeasuredValue` and Thermostat
  `LocalTemperature`. `z2m/valvectl.mjs` maps it to `null`.
- **The sensor sweep is watchdogged**, and any reading older than 35 s (3 sweep periods
  + slack) is reported faulted regardless of latch state — a wedged sweep now degrades
  control to park instead of integrating frozen data. `sensors_fill_faults()` takes one
  coherent locked snapshot.
- **OTA validation gate moved outside the watchdog window:** 12 control cycles (~110 s)
  instead of 3 (~20 s), extracted to `ctrl_core/ota_gate.c` and host-tested.
- **Fault-streak decay requires 3 consecutive good reads** (break-even 25 % failure rate,
  was 33 %) — a probe failing a third of its sweeps now latches.
- **Thermostat setpoint writes echo the clamped value back** to the attribute store, and
  only write NVS when the value actually changed.
- **Cooling dew guard triggers on 30 min of traffic silence** rather than a ZDO link-down
  signal, which a silently dead coordinator never sends.
- **Supply freeze alarm is held while the supply probe is faulted** (no false latch on a
  BSS-zero reading, no false clear on a frozen in-band one).
- `travel_time_s` is latched with `direction_swap`, so a mid-stroke write cannot corrupt
  the position estimate; a short first OTA payload block now aborts loudly.
```

- [ ] **Step 3: VERIFY — full host suite green**

```bash
rm -rf /tmp/vc-tests && cmake -S firmware/test_host -B /tmp/vc-tests \
  -DFETCHCONTENT_SOURCE_DIR_UNITY=/Users/kleist/Sites/ValveController/firmware/test_host/build-121a/_deps/unity-src \
&& cmake --build /tmp/vc-tests -j8 \
&& ctest --test-dir /tmp/vc-tests --output-on-failure
```
Expected: `100% tests passed out of 14`.

- [ ] **Step 4: VERIFY — clean full target build**

```bash
source ~/esp/esp-idf/export.sh && cd firmware && idf.py fullclean && idf.py build
```
Expected: build succeeds, no warnings introduced by this plan's files, and the summary
reports project version `1.3.0`.

- [ ] **Step 5: Build the OTA artifact**

`--out` takes a **DIRECTORY**, not a file path; the script names the file
`valvecontroller-<version>.ota` itself.
```bash
python3 tools/make_ota.py --out /tmp/vc-ota
```
Expected output includes `version : 1.3.0 -> 0x01030000` and
`wrote : /tmp/vc-ota/valvecontroller-1.3.0.ota`.

- [ ] **Step 6: Commit**

```bash
git add -- firmware/version.txt firmware/README.md \
&& git commit -m "docs(fw): release 1.3.0 — review-findings fixes" \
  -- firmware/version.txt firmware/README.md
```

- [ ] **Step 7: Hand off deployment (NOT part of this plan)**

Report to the owner, do not perform: USB flash or OTA staging of
`valvecontroller-1.3.0.ota`, adding the Z2M local OTA index entry (`fileVersion`
`0x01030000`, `imageType 0x0001`, `manufacturerCode 0x1234`, `modelId "HydroMix"`), and
copying `z2m/valvectl.mjs` to `/config/zigbee2mqtt/` on HAOS + restarting Z2M. The
converter change in Task 7 is required for the `0x8000` sentinel to render as
unavailable rather than −327.68 °C.

---

## Unresolved questions

1. F1 boot default when NVS key absent: false (park) chosen — OK? Alternative: true matches old behavior but recreates the mismatch.
2. F5 decay policy: 3-consecutive-goods (break-even 25 %) chosen over windowed failure-rate counter (better diagnostics, more state + new attr). OK?
3. F7 passive liveness needs periodic inbound traffic — is Z2M availability enabled for this device? If not, guard may false-trigger cooling setpoint raise to 21 °C after quiet periods; enable availability or lengthen window?
   *(Added while reading code: plain attribute READS are answered inside the stack and never reach `action_handler`, so the liveness signal depends on writes / OTA / reporting-config traffic specifically. Also note the failure mode is the safe direction — less cooling — so a false trigger is a comfort issue, not a damage one.)*
4. Release: single 1.3.0 with everything, or split safety-critical (F1-F4) into a fast 1.2.2?
5. Z2M converter exposes no climate/setpoint entities (tz handlers exist, no exposes) — add them in this release?
   *(Confirmed while reading `z2m/valvectl.mjs`: `tz.thermostat_occupied_heating_setpoint` / `_cooling_setpoint` are wired into `toZigbee` at line 164-165, but there is no `e.climate()` in `exposes`, so both setpoints are unreachable from HA today.)*
6. **(new) `fz.thermostat` localTemp sentinel guard:** current upstream `zigbee-herdsman-converters` drops `localTemp` values below −273.15 °C, which would filter our `0x8000`. Unverified against the version actually installed on this HAOS. If that guard is absent there, `local_temperature` will publish −327.68 °C — harmless today (not exposed), but it becomes visible the moment Q5 is answered "yes". Verify at deploy time?
7. **(new) `ota_header.image_size` semantics as delivered by ZBOSS:** Task 12 assumes it is the *total* .ota file size (56-byte header + 6-byte sub-element + binary), matching what `tools/make_ota.py` writes. The 1.2.1 OTA performed today logged `OTA image received: <written> bytes written (header size <total>)` — can the owner check that line in the device log to confirm the 62-byte difference? If ZBOSS instead reports the post-header size, the new diagnostic is still correct (it only warns on impossible values) but the logged "expected difference" is misleading.
8. **(new) Task 4-before-Task 5 ordering:** this inverts the requested "ctrl_core first" grouping, because shipping the guard-semantics change before the liveness plumbing would let a long-joined device spuriously raise its cooling setpoint. Confirm the reordering is acceptable.
9. **(new) 16 stray `firmware/test_host/build-*` directories** are untracked in the repo and one of them (`build-121a`) is now load-bearing for this plan's `FETCHCONTENT_SOURCE_DIR_UNITY` override. Delete them and vendor Unity properly (or add a `.gitignore` entry) as a follow-up?
