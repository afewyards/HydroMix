# Firmware 1.7.0 Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove the triplicated RTC diagnostics and triplicated config-clamping left by the 1.6.x debugging work, and split the 984-line `zigbee.c`, so the display/AUX firmware has clean seams to attach to.

**Architecture:** One portable `diag_ring` module in `ctrl_core` absorbs the mechanism shared by three hand-rolled RTC history buffers, while each owner keeps its own typed payload. `config_t` embeds `tunable_cfg_t` so the clamp ranges and defaults have exactly one definition, in the host-tested portable core. `zigbee.c` splits four ways along seams it already has.

**Tech Stack:** ESP-IDF v5.5.4, ESP32-C6, FreeRTOS, C11. Host tests: Unity v2.6.0 via CMake `FetchContent`, one executable per `test_*.c`, each linking all of `ctrl_core`.

## Global Constraints

- **This firmware runs a live household heating plant.** It came back into service 2026-08-10 after a 13-hour outage. Every task must leave the tree building and the host suite green.
- **`sizeof(config_t)` MUST remain 76 bytes and every field offset MUST be unchanged.** The v3 NVS blob is discriminated by size; a change silently rejects every deployed device's stored tuning and reverts it to defaults. Verified offsets: `heat_threshold` 0, `travel_time_s` 24, `direction_swap` 28, `kp` 32, `alarm_dwell_ms` 48, `deadtime_s` 60, `valve_deadband_pct` 68, `cfg_version` 72.
- **`ctrl_core` must have zero ESP-IDF dependencies.** It is compiled by the host suite via `file(GLOB "../components/ctrl_core/*.c")`. Any `#include "esp_*.h"` in that directory breaks all 17 test binaries. Standard C only (`<stdint.h>`, `<stdbool.h>`, `<string.h>`, `<stdio.h>`, `<stdarg.h>`, `<math.h>`).
- **New `.c` files in `ctrl_core/` are picked up automatically** by both the glob above and the IDF component build. New files in `main/` MUST be added to `main/CMakeLists.txt` `SRCS` by hand.
- **Behaviour-preserving.** No control-law, timing, threshold or Zigbee-protocol change. Only Task 5 changes behaviour, and only by making three clamp implementations agree; the equivalence test proves what changed.
- **Never run `git add -A` or `git add .`** — stage files individually by name.
- **Commit message format:** Angular Commit Conventions, scope `fw`, e.g. `refactor(fw): ...`.
- **Do not touch** `pcb/`, `analysis/`, or `z2m/` — those carry the user's own uncommitted work.
- **Build directories:** use `/private/tmp/claude-502/-Users-kleist-Sites-ValveController/c85c92c0-629d-4bf6-aa58-e2352fc544a3/scratchpad/<name>` for host test builds. Never create `build-*/` inside the repo.

### Running the host test suite

Every task uses this. Run it from `/Users/kleist/Sites/ValveController/firmware/test_host`:

```bash
SP=/private/tmp/claude-502/-Users-kleist-Sites-ValveController/c85c92c0-629d-4bf6-aa58-e2352fc544a3/scratchpad
cd /Users/kleist/Sites/ValveController/firmware/test_host
cmake -B $SP/tb -S . >/dev/null && cmake --build $SP/tb -j8 >/dev/null && ctest --test-dir $SP/tb --output-on-failure
```

Full clean cycle is about 9 s (3.4 s configure incl. the Unity fetch, 2 s build, 3.6 s ctest). Reuse `$SP/tb` between tasks so the Unity fetch happens once. Baseline before any change: **17/17 passing**.

### Building for the target

```bash
. $HOME/esp/esp-idf/export.sh          # verify the real path; check `ls ~/esp` if this fails
cd /Users/kleist/Sites/ValveController/firmware
idf.py reconfigure && idf.py build
```

**`idf.py reconfigure` is mandatory before any build whose version matters.** This project has been bitten by a stale cached `PROJECT_VER`: without it the app descriptor keeps the previous version string and the OTA/`update.installed_version` reporting lies.

---

## File Structure

**Created:**
- `firmware/components/ctrl_core/diag_ring.c` — RTC history mechanism: warm/cold detection, boot sequence counter, history shift, reset-reason names, bounded string append. No payload knowledge.
- `firmware/components/ctrl_core/include/ctrl_core/diag_ring.h` — its interface.
- `firmware/test_host/test_diag_ring.c` — Unity tests for the above.
- `firmware/test_host/test_config_clamp.c` — proves the Zigbee attribute write path and the tunable path clamp identically.
- `firmware/main/zigbee_internal.h` — statics shared across the four-way split; not public API.
- `firmware/main/zigbee_attrs.c` — custom cluster `0xFC00` write dispatch + `ATTR_*`→`TUNABLE_*` mapping.
- `firmware/main/zigbee_telem.c` — telemetry task, temperature reporting, `configure_reporting_*`.
- `firmware/main/zigbee_diag.c` — attribute-write tallies, on `diag_ring`.

**Modified:**
- `firmware/components/ctrl_core/include/ctrl_core/config_map.h` — `tunable_cfg_t` absorbs `valve_deadband_pct`; new tunable ids; new `tunable_clamp_all` / `tunable_sane_f` / `tunable_clamp_u32` / `valve_deadband_floor_pct` prototypes.
- `firmware/components/ctrl_core/config_map.c` — becomes the single home for every clamp range and default.
- `firmware/main/config.h` — `config_t` embeds `tunable_cfg_t t`; static asserts pin the layout.
- `firmware/main/config.c` — `clamp_config` and `config_apply_custom` delegate to `ctrl_core`; `DEFAULTS` derived.
- `firmware/main/control_task.c`, `firmware/main/valve_hw.c` — `g_config.X` → `g_config.t.X` (16 references total).
- `firmware/main/taskhb.c` — rebuilt on `diag_ring`.
- `firmware/main/sensors_hw.c` — ring moves to `diag_ring`; roll-on-boot leaves `sensors_start()`.
- `firmware/main/zigbee.c` — reduced to init, endpoint/cluster registration, join/steer signals.
- `firmware/main/CMakeLists.txt` — new `main/` sources added to `SRCS`.
- `firmware/version.txt`, `firmware/README.md` — 1.7.0 release notes.
- `.gitignore` — stop tracking build detritus.

**Deleted:**
- `firmware/test_host/build-*/` — 26 untracked directories, 304 MB.

---

## Task 1: Portable RTC history mechanism (`diag_ring`)

**Files:**
- Create: `firmware/components/ctrl_core/include/ctrl_core/diag_ring.h`
- Create: `firmware/components/ctrl_core/diag_ring.c`
- Test: `firmware/test_host/test_diag_ring.c`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `#define DIAG_RING_DEPTH 4`
  - `typedef struct { uint32_t magic; uint32_t seq; } diag_hdr_t;`
  - `bool diag_ring_warm(void *store, size_t store_size, uint32_t magic);`
  - `void diag_ring_shift(void *hist, size_t elem, uint8_t depth);`
  - `const char *diag_reset_reason_name(int reason);`
  - `size_t diag_appendf(char *o, size_t n, size_t used, const char *fmt, ...);`

  Tasks 2, 3 and 4 rewrite `main/taskhb.c`, `main/sensors_hw.c` and the zigbee diagnostics onto exactly these six.

**Contract notes for the implementer:**

`diag_ring_warm` requires `diag_hdr_t` to be the **first member** of the store struct — it casts `store` to `diag_hdr_t *`. On a warm boot (magic matches) it bumps `seq` and returns `true`, leaving the payload intact. On a cold boot it zeroes the whole store, stamps the magic, sets `seq = 1`, and returns `false`. Each call site asserts the first-member requirement with `_Static_assert`.

`diag_ring_shift` moves `hist[n-1]` into `hist[n]` for n descending, leaving `hist[0]` untouched for the caller to overwrite with its own typed fields. It deliberately does **not** copy `cur` into `hist[0]`: the three owners have different relationships between their live and historical structs (heartbeats add `end_reason`/`valid` fields that `cur` does not have), and a generic copy would force all three into one shape.

- [ ] **Step 1: Confirm the `esp_reset_reason_t` numeric values before hardcoding them**

`ctrl_core` cannot include `esp_system.h`, so `diag_reset_reason_name` uses a numeric table. Read the real enum from the installed IDF rather than trusting memory:

```bash
grep -n -A 20 'typedef enum {' $(find $HOME/esp -name esp_system.h -path '*/esp_system/*' 2>/dev/null | head -1) | grep -E 'ESP_RST_|typedef|}'
```

Write down the integer for each of `ESP_RST_UNKNOWN`, `POWERON`, `EXT`, `SW`, `PANIC`, `INT_WDT`, `TASK_WDT`, `WDT`, `DEEPSLEEP`, `BROWNOUT`, `SDIO`, `USB`, `JTAG`, and any later members. Use those values in Step 3. If the header cannot be found, stop and report — do not guess.

- [ ] **Step 2: Write the failing tests**

Create `firmware/test_host/test_diag_ring.c`:

```c
#include "unity.h"
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include "ctrl_core/diag_ring.h"

void setUp(void){} void tearDown(void){}

typedef struct { uint32_t a, b; } payload_t;
typedef struct {
    diag_hdr_t hdr;
    payload_t  cur;
    payload_t  hist[DIAG_RING_DEPTH];
} store_t;

#define MAGIC 0xABCD1234u

void test_cold_boot_zeroes_and_seeds(void){
    store_t s;
    memset(&s, 0xAA, sizeof s);          /* garbage, as uninitialised RTC RAM would be */
    bool warm = diag_ring_warm(&s, sizeof s, MAGIC);
    TEST_ASSERT_FALSE(warm);
    TEST_ASSERT_EQUAL_UINT32(MAGIC, s.hdr.magic);
    TEST_ASSERT_EQUAL_UINT32(1, s.hdr.seq);
    TEST_ASSERT_EQUAL_UINT32(0, s.cur.a);            /* payload wiped */
    TEST_ASSERT_EQUAL_UINT32(0, s.hist[DIAG_RING_DEPTH-1].b);
}

void test_warm_boot_bumps_seq_and_keeps_payload(void){
    store_t s;
    memset(&s, 0, sizeof s);
    diag_ring_warm(&s, sizeof s, MAGIC);             /* cold: seq = 1 */
    s.cur.a = 42;
    bool warm = diag_ring_warm(&s, sizeof s, MAGIC); /* warm: seq = 2 */
    TEST_ASSERT_TRUE(warm);
    TEST_ASSERT_EQUAL_UINT32(2, s.hdr.seq);
    TEST_ASSERT_EQUAL_UINT32(42, s.cur.a);           /* NOT wiped */
    diag_ring_warm(&s, sizeof s, MAGIC);
    TEST_ASSERT_EQUAL_UINT32(3, s.hdr.seq);
}

void test_wrong_magic_is_treated_as_cold(void){
    store_t s;
    memset(&s, 0, sizeof s);
    s.hdr.magic = 0xDEADBEEFu;
    s.cur.a = 99;
    TEST_ASSERT_FALSE(diag_ring_warm(&s, sizeof s, MAGIC));
    TEST_ASSERT_EQUAL_UINT32(0, s.cur.a);
}

void test_shift_moves_history_down_and_leaves_slot_zero(void){
    payload_t h[4] = { {1,1}, {2,2}, {3,3}, {4,4} };
    diag_ring_shift(h, sizeof h[0], 4);
    TEST_ASSERT_EQUAL_UINT32(1, h[0].a);   /* untouched, caller overwrites */
    TEST_ASSERT_EQUAL_UINT32(1, h[1].a);
    TEST_ASSERT_EQUAL_UINT32(2, h[2].a);
    TEST_ASSERT_EQUAL_UINT32(3, h[3].a);   /* 4 fell off the end */
}

void test_shift_is_a_noop_for_degenerate_inputs(void){
    payload_t h[2] = { {7,7}, {8,8} };
    diag_ring_shift(h, sizeof h[0], 1);    /* depth 1: nothing to shift */
    TEST_ASSERT_EQUAL_UINT32(7, h[0].a);
    TEST_ASSERT_EQUAL_UINT32(8, h[1].a);
    diag_ring_shift(h, 0, 2);              /* zero element size */
    TEST_ASSERT_EQUAL_UINT32(8, h[1].a);
    diag_ring_shift(NULL, sizeof h[0], 2); /* must not crash */
}

void test_appendf_accumulates_and_returns_used(void){
    char b[32]; b[0] = '\0';
    size_t u = diag_appendf(b, sizeof b, 0, "ab");
    TEST_ASSERT_EQUAL_UINT32(2, u);
    u = diag_appendf(b, sizeof b, u, "cd%d", 5);
    TEST_ASSERT_EQUAL_UINT32(5, u);
    TEST_ASSERT_EQUAL_STRING("abcd5", b);
}

void test_appendf_truncates_without_overflowing(void){
    char guard[16];
    memset(guard, 0x7E, sizeof guard);
    char *b = guard;                       /* deliberately hand it only 8 of the 16 */
    b[0] = '\0';
    size_t u = diag_appendf(b, 8, 0, "0123456789ABCDEF");
    TEST_ASSERT_EQUAL_UINT32(7, u);                    /* capped at n-1 */
    TEST_ASSERT_EQUAL_STRING("0123456", b);
    TEST_ASSERT_EQUAL_HEX8(0x7E, (unsigned char)guard[8]);  /* byte past n untouched */
}

void test_appendf_past_capacity_is_idempotent(void){
    char b[8]; b[0] = '\0';
    size_t u = diag_appendf(b, sizeof b, 0, "0123456789");
    size_t v = diag_appendf(b, sizeof b, u, "more");
    size_t w = diag_appendf(b, sizeof b, v, "more");
    TEST_ASSERT_EQUAL_UINT32(7, u);
    TEST_ASSERT_EQUAL_UINT32(7, v);
    TEST_ASSERT_EQUAL_UINT32(7, w);
    TEST_ASSERT_EQUAL_STRING("0123456", b);
}

void test_appendf_degenerate_buffers(void){
    char b[1];
    b[0] = 'x';
    TEST_ASSERT_EQUAL_UINT32(0, diag_appendf(b, 1, 0, "hello"));
    TEST_ASSERT_EQUAL_STRING("", b);                   /* terminated, not left as 'x' */
    TEST_ASSERT_EQUAL_UINT32(0, diag_appendf(NULL, 8, 0, "hello"));
    TEST_ASSERT_EQUAL_UINT32(0, diag_appendf(b, 0, 0, "hello"));
}

void test_reset_reason_names(void){
    /* Values confirmed against the installed IDF in Step 1. */
    TEST_ASSERT_EQUAL_STRING("poweron",  diag_reset_reason_name(1));
    TEST_ASSERT_EQUAL_STRING("TASK_WDT", diag_reset_reason_name(6));
    TEST_ASSERT_EQUAL_STRING("BROWNOUT", diag_reset_reason_name(9));
    TEST_ASSERT_EQUAL_STRING("other",    diag_reset_reason_name(9999));
}

int main(void){
    UNITY_BEGIN();
    RUN_TEST(test_cold_boot_zeroes_and_seeds);
    RUN_TEST(test_warm_boot_bumps_seq_and_keeps_payload);
    RUN_TEST(test_wrong_magic_is_treated_as_cold);
    RUN_TEST(test_shift_moves_history_down_and_leaves_slot_zero);
    RUN_TEST(test_shift_is_a_noop_for_degenerate_inputs);
    RUN_TEST(test_appendf_accumulates_and_returns_used);
    RUN_TEST(test_appendf_truncates_without_overflowing);
    RUN_TEST(test_appendf_past_capacity_is_idempotent);
    RUN_TEST(test_appendf_degenerate_buffers);
    RUN_TEST(test_reset_reason_names);
    return UNITY_END();
}
```

If Step 1 showed different integers for `ESP_RST_POWERON` / `TASK_WDT` / `BROWNOUT`, correct `test_reset_reason_names` to match before running.

- [ ] **Step 3: Run the tests to verify they fail**

```bash
SP=/private/tmp/claude-502/-Users-kleist-Sites-ValveController/c85c92c0-629d-4bf6-aa58-e2352fc544a3/scratchpad
cd /Users/kleist/Sites/ValveController/firmware/test_host
cmake -B $SP/tb -S . && cmake --build $SP/tb -j8
```

Expected: build FAILS with `ctrl_core/diag_ring.h: No such file or directory`.

- [ ] **Step 4: Write the header**

Create `firmware/components/ctrl_core/include/ctrl_core/diag_ring.h`:

```c
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Shared mechanism for the RTC-RAM run histories.
 *
 * Three of these were written independently during the 1.6.x debugging (per-task
 * heartbeats, 1-Wire failure tallies, Zigbee attribute-write tallies) because each was
 * added under pressure while chasing a different hypothesis. The payloads genuinely
 * differ; only the mechanism is shared, and only the mechanism lives here.
 *
 * Depth is 4 because reaching the console on this board costs TWO resets -- plugging the
 * cable in is one, opening the port is another -- so a one-deep record always describes
 * the run your own reset ended, never the run the fault ended. */
#define DIAG_RING_DEPTH 4

/* MUST be the first member of the owning store struct; diag_ring_warm() casts to it.
 * Assert that at each call site with _Static_assert(offsetof(store_t, hdr) == 0, ...). */
typedef struct {
    uint32_t magic;
    uint32_t seq;      /* boot counter, so runs are identifiable across resets */
} diag_hdr_t;

/* Warm (magic matched): bump seq, leave the payload alone, return true -- the caller
 * should now shift its history and record the run that just ended.
 * Cold (magic absent): zero the whole store, stamp magic, seq = 1, return false. */
bool diag_ring_warm(void *store, size_t store_size, uint32_t magic);

/* hist[h] <- hist[h-1] for h descending. hist[0] is left as-is for the caller to
 * overwrite with its own typed fields. No-op if hist is NULL, elem is 0, or depth < 2. */
void diag_ring_shift(void *hist, size_t elem, uint8_t depth);

/* esp_reset_reason_t widened to int -- ctrl_core cannot include esp_system.h. Call sites
 * in main/ pin the mapping with _Static_assert against the real enum. */
const char *diag_reset_reason_name(int reason);

/* Bounded append into o[0..n). Returns the new used length, never more than n-1.
 * Always leaves o NUL-terminated. Safe to call repeatedly after the buffer is full. */
size_t diag_appendf(char *o, size_t n, size_t used, const char *fmt, ...)
    __attribute__((format(printf, 4, 5)));
```

- [ ] **Step 5: Write the implementation**

Create `firmware/components/ctrl_core/diag_ring.c`:

```c
#include "ctrl_core/diag_ring.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

bool diag_ring_warm(void *store, size_t store_size, uint32_t magic)
{
    if (!store || store_size < sizeof(diag_hdr_t)) return false;
    diag_hdr_t *h = (diag_hdr_t *)store;
    if (h->magic == magic) {
        h->seq++;
        return true;
    }
    memset(store, 0, store_size);
    h->magic = magic;
    h->seq   = 1;
    return false;
}

void diag_ring_shift(void *hist, size_t elem, uint8_t depth)
{
    if (!hist || elem == 0 || depth < 2) return;
    char *b = (char *)hist;
    for (int h = (int)depth - 1; h > 0; --h)
        memcpy(b + (size_t)h * elem, b + (size_t)(h - 1) * elem, elem);
}

const char *diag_reset_reason_name(int reason)
{
    /* Numeric because ctrl_core cannot see esp_system.h. Values confirmed against the
     * installed ESP-IDF; main/taskhb.c static-asserts that they still match. */
    switch (reason) {
        case 1:  return "poweron";
        case 2:  return "ext";
        case 3:  return "sw";
        case 4:  return "PANIC";
        case 5:  return "INT_WDT";
        case 6:  return "TASK_WDT";
        case 7:  return "WDT";
        case 8:  return "deepsleep";
        case 9:  return "BROWNOUT";
        case 10: return "sdio";
        case 11: return "usb";
        case 12: return "jtag";
        default: return "other";
    }
}

size_t diag_appendf(char *o, size_t n, size_t used, const char *fmt, ...)
{
    if (!o || n == 0) return 0;
    if (used >= n - 1) { o[n - 1] = '\0'; return n - 1; }
    va_list ap;
    va_start(ap, fmt);
    int k = vsnprintf(o + used, n - used, fmt, ap);
    va_end(ap);
    if (k < 0) return used;                     /* encoding error: keep what we had */
    used += (size_t)k;                          /* vsnprintf returns the WANTED length */
    return used > n - 1 ? n - 1 : used;
}
```

Correct the `diag_reset_reason_name` cases if Step 1 found different integers.

- [ ] **Step 6: Run the tests to verify they pass**

```bash
SP=/private/tmp/claude-502/-Users-kleist-Sites-ValveController/c85c92c0-629d-4bf6-aa58-e2352fc544a3/scratchpad
cd /Users/kleist/Sites/ValveController/firmware/test_host
cmake -B $SP/tb -S . >/dev/null && cmake --build $SP/tb -j8 >/dev/null && ctest --test-dir $SP/tb --output-on-failure
```

Expected: **18/18 passing** (17 existing + `test_diag_ring`). `test_diag_ring` alone must report 10 tests, 0 failures. If any pre-existing test broke, stop — `diag_ring.c` joins every binary via the glob, so a name collision with an existing symbol is the likely cause.

- [ ] **Step 7: Commit**

```bash
cd /Users/kleist/Sites/ValveController
git add firmware/components/ctrl_core/include/ctrl_core/diag_ring.h \
        firmware/components/ctrl_core/diag_ring.c \
        firmware/test_host/test_diag_ring.c
git commit -m "feat(fw): add portable RTC history ring shared by the diagnostics"
```

---

## Task 2: Move `taskhb.c` onto `diag_ring`

**Files:**
- Modify: `firmware/main/taskhb.c` (whole file, currently 118 lines)
- Unchanged: `firmware/main/taskhb.h` — the public API (`hb_note`, `hb_boot_report`, `hb_format`, `hb_id_t`) does not change

**Interfaces:**
- Consumes: `diag_ring_warm`, `diag_ring_shift`, `diag_reset_reason_name`, `diag_appendf`, `DIAG_RING_DEPTH` from Task 1.
- Produces: no API change. `hb_note(hb_id_t)`, `hb_boot_report(void)`, `hb_format(char*, size_t)` keep their signatures; `console.c` and the four task files are untouched.

This task is done first of the three migrations because `taskhb.c` is the smallest and is entirely this pattern — if the `diag_ring` API is wrong, it shows up here cheapest.

- [ ] **Step 1: Replace the file contents**

Rewrite `firmware/main/taskhb.c` as:

```c
#include "taskhb.h"
#include <stddef.h>
#include <string.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "ctrl_core/diag_ring.h"

static const char *TAG = "hb";

static const char *const HB_NAME[HB_COUNT] = { "sensors", "control", "valve", "telem" };

#define HB_MAGIC 0x48425403u   /* bumped: the store layout changed with the hdr */

/* diag_reset_reason_name() is numeric because ctrl_core cannot see esp_system.h.
 * Pin the mapping here, where the real enum IS visible, so an IDF renumbering breaks
 * the build instead of quietly mislabelling every recorded run. */
_Static_assert(ESP_RST_POWERON  == 1, "diag_reset_reason_name table is stale");
_Static_assert(ESP_RST_PANIC    == 4, "diag_reset_reason_name table is stale");
_Static_assert(ESP_RST_TASK_WDT == 6, "diag_reset_reason_name table is stale");
_Static_assert(ESP_RST_BROWNOUT == 9, "diag_reset_reason_name table is stale");

typedef struct {
    uint32_t last_ms[HB_COUNT];
    uint8_t  end_reason;         /* esp_reset_reason_t observed at the NEXT boot */
    uint8_t  valid;
} hb_run_t;

typedef struct {
    diag_hdr_t hdr;                    /* MUST be first: diag_ring_warm() casts to it */
    uint32_t   last_ms[HB_COUNT];      /* live run */
    hb_run_t   hist[DIAG_RING_DEPTH];  /* hist[0] = most recently ended run */
} hb_store_t;

_Static_assert(offsetof(hb_store_t, hdr) == 0, "diag_hdr_t must be the first member");

static RTC_NOINIT_ATTR hb_store_t s_hb;

void hb_note(hb_id_t id)
{
    if (id < HB_COUNT) s_hb.last_ms[id] = (uint32_t)(esp_timer_get_time() / 1000);
}

/* Age measured back from the newest heartbeat, which approximates the instant the run
 * died -- the reset time itself is not recoverable after the fact. */
static uint32_t newest_of(const uint32_t *v)
{
    uint32_t n = 0;
    for (int i = 0; i < HB_COUNT; ++i) if (v[i] > n) n = v[i];
    return n;
}

void hb_boot_report(void)
{
    uint8_t rr = (uint8_t)esp_reset_reason();

    if (diag_ring_warm(&s_hb, sizeof s_hb, HB_MAGIC)) {
        diag_ring_shift(s_hb.hist, sizeof s_hb.hist[0], DIAG_RING_DEPTH);
        memcpy(s_hb.hist[0].last_ms, s_hb.last_ms, sizeof s_hb.last_ms);
        s_hb.hist[0].end_reason = rr;      /* why the run that just ended, ended */
        s_hb.hist[0].valid = 1;

        const hb_run_t *p = &s_hb.hist[0];
        uint32_t newest = newest_of(p->last_ms);
        ESP_LOGW(TAG, "run#%lu ended by %s; heartbeat ages at death (0 = last to run):",
                 (unsigned long)(s_hb.hdr.seq - 1), diag_reset_reason_name(rr));
        for (int i = 0; i < HB_COUNT; ++i)
            ESP_LOGW(TAG, "  %-7s last=%lu ms  age=%lu ms", HB_NAME[i],
                     (unsigned long)p->last_ms[i], (unsigned long)(newest - p->last_ms[i]));
    } else {
        ESP_LOGI(TAG, "no previous heartbeats (cold power-on)");
    }
    memset(s_hb.last_ms, 0, sizeof s_hb.last_ms);
}

void hb_format(char *o, size_t n)
{
    if (!o || n == 0) return;
    o[0] = '\0';
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    size_t u = diag_appendf(o, n, 0, "run#%lu now=%lu ms\n",
                            (unsigned long)s_hb.hdr.seq, (unsigned long)now);
    for (int i = 0; i < HB_COUNT; ++i)
        u = diag_appendf(o, n, u, "  %-7s age=%lu\n", HB_NAME[i],
                         (unsigned long)(now - s_hb.last_ms[i]));
    for (int h = 0; h < DIAG_RING_DEPTH; ++h) {
        if (!s_hb.hist[h].valid) continue;
        const hb_run_t *p = &s_hb.hist[h];
        uint32_t newest = newest_of(p->last_ms);
        u = diag_appendf(o, n, u, "prev-%d (run#%lu) ended by %s:\n", h + 1,
                         (unsigned long)(s_hb.hdr.seq - (uint32_t)(h + 1)),
                         diag_reset_reason_name(p->end_reason));
        for (int i = 0; i < HB_COUNT; ++i)
            u = diag_appendf(o, n, u, "  %-7s last=%lu age=%lu\n", HB_NAME[i],
                             (unsigned long)p->last_ms[i],
                             (unsigned long)(newest - p->last_ms[i]));
    }
}
```

Three things changed beyond the mechanical substitution, all deliberate:

1. **`HB_MAGIC` bumped `0x48425402` → `0x48425403`.** The store gained a `diag_hdr_t` at the front, so old RTC contents no longer match the new layout. Without the bump, the first boot after flashing would read a stale `seq` and stale `hist[]` through the wrong offsets and print nonsense.
2. **`rr_name` deleted** — `diag_reset_reason_name` replaces it. It gains `deepsleep`, `sdio` and `jtag`, which the `taskhb.c` copy lacked.
3. **The `u < n - 1` guards are gone** from `hb_format` — `diag_appendf` enforces the bound itself and is safe to call after the buffer fills.

- [ ] **Step 2: Verify the host suite is still green**

`taskhb.c` is in `main/`, not `ctrl_core`, so it is NOT compiled by the host suite — but `diag_ring.c` is, and this task must not have disturbed it.

```bash
SP=/private/tmp/claude-502/-Users-kleist-Sites-ValveController/c85c92c0-629d-4bf6-aa58-e2352fc544a3/scratchpad
cd /Users/kleist/Sites/ValveController/firmware/test_host
cmake --build $SP/tb -j8 >/dev/null && ctest --test-dir $SP/tb --output-on-failure
```

Expected: **18/18 passing**.

- [ ] **Step 3: Verify it compiles for the target**

```bash
. $HOME/esp/esp-idf/export.sh
cd /Users/kleist/Sites/ValveController/firmware && idf.py build
```

Expected: build succeeds with no new warnings. A `_Static_assert` failure here means the IDF reset-reason enum differs from Task 1's table — fix `diag_ring.c` and its test, do not weaken the assert.

- [ ] **Step 4: Commit**

```bash
cd /Users/kleist/Sites/ValveController
git add firmware/main/taskhb.c
git commit -m "refactor(fw): rebuild task heartbeats on the shared diag ring"
```

---

## Task 3: Move `sensors_hw.c` onto `diag_ring`

**Files:**
- Modify: `firmware/main/sensors_hw.c:72-95` (store definition), `:224-279` (`sensors_start`), `:325-363` (`fmt_run`, `sensors_format_stats`)

**Interfaces:**
- Consumes: `diag_ring_warm`, `diag_ring_shift`, `diag_appendf`, `DIAG_RING_DEPTH` from Task 1.
- Produces: no API change. `sensors_start`, `sensors_format_stats`, `sensors_sweep_count`, `sensors_sweep_dead`, `sensors_get`, `sensors_fill_faults` keep their signatures. The `s_stats` macro still resolves to the live tally, so `sweep_task` is untouched.

- [ ] **Step 1: Replace the store definition**

In `firmware/main/sensors_hw.c`, replace lines 72-95 (from `#define SENSOR_STATS_MAGIC` through `#define s_stats (s_store.cur)`) with:

```c
/* 0x5EA5C0E0, bumped from 0x5EA5C0DF: the store gained a diag_hdr_t at the front, so
 * RTC contents written by 1.6.x no longer describe this layout. */
#define SENSOR_STATS_MAGIC 0x5EA5C0E0u

typedef struct {
    uint32_t sweeps;
    uint32_t ok[SENS_COUNT];
    uint32_t convert_fail[SENS_COUNT];
    uint32_t fail[SENS_COUNT][OW_REASONS];   /* [OW_OK] stays zero */
} sensor_stats_t;

typedef struct {
    diag_hdr_t     hdr;                          /* MUST be first */
    sensor_stats_t cur;
    sensor_stats_t hist[DIAG_RING_DEPTH];        /* hist[0] = most recently ended run */
} sensor_store_t;

_Static_assert(offsetof(sensor_store_t, hdr) == 0, "diag_hdr_t must be the first member");

static RTC_NOINIT_ATTR sensor_store_t s_store;
#define s_stats (s_store.cur)
```

Add `#include <stddef.h>` and `#include "ctrl_core/diag_ring.h"` to the include block at the top of the file. Delete the `#define SENSOR_HISTORY 4` line and its comment — the depth now comes from `DIAG_RING_DEPTH`. Keep the long `---- Failure attribution ----` comment block above it; it explains *why* the tallies exist and is still accurate.

- [ ] **Step 2: Extract the roll-on-boot out of `sensors_start`**

The history roll is currently interleaved with mutex creation and task startup. Split it into its own function. Replace lines 224-257 (from `void sensors_start(void)` through `memset(&s_store.cur, 0, sizeof s_store.cur);`) with:

```c
/* Shift the just-ended run into history and log it. Attaching a console resets this
 * board, so for an operator chasing an intermittent fault the history is the only view
 * of the run they were actually trying to observe. */
static void sensors_roll_history(void)
{
    if (!diag_ring_warm(&s_store, sizeof s_store, SENSOR_STATS_MAGIC)) {
        ESP_LOGI(TAG, "no previous-run sensor stats (cold power-on)");
        return;
    }
    diag_ring_shift(s_store.hist, sizeof s_store.hist[0], DIAG_RING_DEPTH);
    s_store.hist[0] = s_store.cur;

    const sensor_stats_t *p = &s_store.hist[0];
    ESP_LOGW(TAG, "previous run#%lu: %lu sweeps",
             (unsigned long)(s_store.hdr.seq - 1), (unsigned long)p->sweeps);
    for (int i = 0; i < SENS_COUNT; ++i) {
        uint32_t bad = 0;
        for (int r = OW_OK + 1; r < OW_REASONS; ++r) bad += p->fail[i][r];
        if (!bad && !p->convert_fail[i]) continue;
        ESP_LOGW(TAG, "  %s ok=%lu fail=%lu (rmt=%lu reset=%lu write=%lu read=%lu "
                      "crc=%lu por85=%lu) convert_fail=%lu",
                 SENS_NAME[i], (unsigned long)p->ok[i], (unsigned long)bad,
                 (unsigned long)p->fail[i][OW_BUS],
                 (unsigned long)p->fail[i][OW_RESET],
                 (unsigned long)p->fail[i][OW_WRITE],
                 (unsigned long)p->fail[i][OW_READ],
                 (unsigned long)p->fail[i][OW_CRC],
                 (unsigned long)p->fail[i][OW_POR],
                 (unsigned long)p->convert_fail[i]);
    }
}

void sensors_start(void)
{
    sensors_roll_history();
    memset(&s_store.cur, 0, sizeof s_store.cur);
```

The rest of `sensors_start` (the `g_lock` creation, the latched-faulted seeding loop, and the checked `xTaskCreate` with its `abort()`) stays exactly as it is.

Note the ordering requirement: `diag_ring_warm` zeroes the whole store on a cold boot, so the `memset(&s_store.cur, ...)` must stay *after* `sensors_roll_history()`. On a warm boot it clears the live tally after it has been copied into `hist[0]`; on a cold boot it is redundant but harmless.

- [ ] **Step 3: Rewrite the formatters**

Replace lines 325-363 (`fmt_run` and `sensors_format_stats`) with:

```c
static size_t fmt_run(char *o, size_t n, size_t u, const char *label, const sensor_stats_t *s)
{
    u = diag_appendf(o, n, u, "%s: %lu sweeps\n", label, (unsigned long)s->sweeps);
    for (int i = 0; i < SENS_COUNT; ++i)
        u = diag_appendf(o, n, u,
                         "  %-6s ok=%lu rmt=%lu rst=%lu wr=%lu rd=%lu crc=%lu por85=%lu cnv=%lu\n",
                         SENS_NAME[i], (unsigned long)s->ok[i],
                         (unsigned long)s->fail[i][OW_BUS],
                         (unsigned long)s->fail[i][OW_RESET],
                         (unsigned long)s->fail[i][OW_WRITE],
                         (unsigned long)s->fail[i][OW_READ],
                         (unsigned long)s->fail[i][OW_CRC],
                         (unsigned long)s->fail[i][OW_POR],
                         (unsigned long)s->convert_fail[i]);
    return u;
}

/* Console `stats`: this run, plus the previous ones RTC memory carried across resets --
 * including the reset that opening the console just caused. */
void sensors_format_stats(char *o, size_t n)
{
    if (!o || n == 0) return;
    o[0] = '\0';
    char lbl[24];
    snprintf(lbl, sizeof lbl, "run#%lu", (unsigned long)s_store.hdr.seq);
    size_t u = fmt_run(o, n, 0, lbl, &s_store.cur);
    bool any = false;
    for (int h = 0; h < DIAG_RING_DEPTH; ++h) {
        if (!s_store.hist[h].sweeps) continue;
        snprintf(lbl, sizeof lbl, "prev-%d", h + 1);
        u = fmt_run(o, n, u, lbl, &s_store.hist[h]);
        any = true;
    }
    if (!any) diag_appendf(o, n, u, "prev: none (cold power-on)\n");
}
```

- [ ] **Step 4: Build for the target**

```bash
. $HOME/esp/esp-idf/export.sh
cd /Users/kleist/Sites/ValveController/firmware && idf.py build
```

Expected: succeeds, no new warnings. Any remaining reference to `s_store.magic`, `s_store.seq` or `SENSOR_HISTORY` is a compile error — fix it to use `s_store.hdr.magic`, `s_store.hdr.seq`, `DIAG_RING_DEPTH`.

- [ ] **Step 5: Confirm the host suite is unaffected**

```bash
SP=/private/tmp/claude-502/-Users-kleist-Sites-ValveController/c85c92c0-629d-4bf6-aa58-e2352fc544a3/scratchpad
ctest --test-dir $SP/tb --output-on-failure
```

Expected: **18/18 passing**.

- [ ] **Step 6: Commit**

```bash
cd /Users/kleist/Sites/ValveController
git add firmware/main/sensors_hw.c
git commit -m "refactor(fw): move 1-Wire tallies onto the shared diag ring"
```

---

## Task 4: Extract the Zigbee write tally into `zigbee_diag.c`, on `diag_ring`

**Files:**
- Create: `firmware/main/zigbee_diag.c`, `firmware/main/zigbee_diag.h`
- Modify: `firmware/main/zigbee.c` — delete lines 618-669 (tally block) and 715-800 (formatters, `reset_reason_name`, `zb_temp_stats_init`); rewrite `zigbee_report_temps` (671-713) to call the new recorders
- Modify: `firmware/main/CMakeLists.txt` — add `zigbee_diag.c` to `SRCS`

**Interfaces:**
- Consumes: `diag_ring_warm`, `diag_ring_shift`, `diag_reset_reason_name`, `diag_appendf`, `DIAG_RING_DEPTH` from Task 1.
- Produces, in `zigbee_diag.h`:
  - `void zbdiag_boot_init(void);` — replaces the file-static `zb_temp_stats_init`
  - `void zbdiag_note_ok(int i, uint8_t ep, int16_t wrote, int16_t read_back);`
  - `void zbdiag_note_fail(int i, uint8_t ep, int32_t status, int16_t wrote);`
  - `void zbdiag_note_tick(uint32_t sweeps, uint16_t faults);`
  - `zigbee_format_temp_stats(char*, size_t)` moves here unchanged in signature; it stays declared in `zigbee.h` because `console.c` calls it.

This is deliberately the first slice of the four-way split (Task 7): the diagnostics are the most separable bucket, they touch no `esp_zb_*` call, and doing them first proves the internal-header pattern on low-risk code.

- [ ] **Step 1: Create `zigbee_diag.h`**

```c
#pragma once
#include <stddef.h>
#include <stdint.h>

/* Temperature attribute-write tally, kept in RTC RAM across resets.
 *
 * The five measurement endpoints published the ZCL invalid sentinel for 13 h on
 * 2026-08-10 while the thermostat LocalTemperature on EP1 -- fed from the SAME sensor in
 * the same telemetry iteration, microseconds apart -- carried a live, rising value.
 * esp_zb_zcl_set_attribute_val() returns a status and every call site discarded it, so
 * which endpoint failed, and why, was unknowable from outside.
 *
 * Three things are recorded per run, because each rules out a different layer:
 *   ok/fail + status    -- did the write get accepted at all;
 *   mismatch/read       -- did the ZCL table actually TAKE the value. SUCCESS on the
 *                          write is not evidence of that, and the table is what the
 *                          reporting engine transmits from;
 *   ticks/sweeps/faults -- what the plant was doing at the time, so "writes all
 *                          succeeded" does not merely move the question elsewhere. */

/* Roll the previous run into history and log it. Call from zigbee_start(). */
void zbdiag_boot_init(void);

/* i is the 0-based endpoint index (0 = supply .. 4 = hx_b); ep is the ZCL endpoint
 * number, used only for logging. */
void zbdiag_note_ok(int i, uint8_t ep, int16_t wrote, int16_t read_back);
void zbdiag_note_fail(int i, uint8_t ep, int32_t status, int16_t wrote);

/* Once per completed telemetry iteration. */
void zbdiag_note_tick(uint32_t sweeps, uint16_t faults);
```

- [ ] **Step 2: Create `zigbee_diag.c`**

```c
#include "zigbee_diag.h"
#include "zigbee.h"
#include <stddef.h>
#include <string.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "ctrl_core/diag_ring.h"

static const char *TAG = "zigbee";

/* 0x2B7E150B, bumped from 0x2B7E150A: the store gained a diag_hdr_t at the front. */
#define ZB_TEMP_STATS_MAGIC 0x2B7E150Bu
#define ZB_EP_COUNT 5

static const char *const TEMP_EP_NAME[ZB_EP_COUNT] = {
    "supply", "return", "source", "hx_a", "hx_b"
};

typedef struct {
    uint32_t ok[ZB_EP_COUNT];
    uint32_t fail[ZB_EP_COUNT];
    uint32_t mismatch[ZB_EP_COUNT];   /* write returned SUCCESS but the table held other */
    int32_t  last_err[ZB_EP_COUNT];   /* esp_zb_zcl_status_t of the most recent failure */
    int16_t  last_val[ZB_EP_COUNT];   /* value the write attempted */
    int16_t  last_read[ZB_EP_COUNT];  /* what the table held immediately after */
    uint32_t ticks;                   /* telemetry iterations completed this run */
    uint32_t sweeps;                  /* sensor sweeps completed this run */
    uint16_t faults;                  /* control_task_faults() at the last tick */
    /* Why THIS run started -- which is to say why the previous one ended. The console
     * cannot answer it: attaching USB is exactly the intervention that stops the resets. */
    uint8_t  reset_reason;
} zb_run_t;

typedef struct {
    diag_hdr_t hdr;                       /* MUST be first */
    zb_run_t   cur;
    zb_run_t   hist[DIAG_RING_DEPTH];     /* hist[0] = most recently ended run */
} zb_store_t;

_Static_assert(offsetof(zb_store_t, hdr) == 0, "diag_hdr_t must be the first member");

static RTC_NOINIT_ATTR zb_store_t s_zb;

void zbdiag_note_ok(int i, uint8_t ep, int16_t wrote, int16_t read_back)
{
    if (i < 0 || i >= ZB_EP_COUNT) return;
    s_zb.cur.ok[i]++;
    s_zb.cur.last_read[i] = read_back;
    if (read_back != wrote) {
        s_zb.cur.mismatch[i]++;
        if (s_zb.cur.mismatch[i] == 1)
            ESP_LOGE(TAG, "ep%u %s attr MISMATCH: wrote %d, table holds %d",
                     (unsigned)ep, TEMP_EP_NAME[i], (int)wrote, (int)read_back);
    }
}

void zbdiag_note_fail(int i, uint8_t ep, int32_t status, int16_t wrote)
{
    if (i < 0 || i >= ZB_EP_COUNT) return;
    s_zb.cur.fail[i]++;
    s_zb.cur.last_err[i] = status;
    s_zb.cur.last_val[i] = wrote;
    /* First failure per endpoint is loud; after that the tally carries it, so a
     * persistent fault cannot bury the rest of the log at 30 lines a minute. */
    if (s_zb.cur.fail[i] == 1)
        ESP_LOGE(TAG, "set_attribute_val(ep%u %s) failed: status=0x%02x value=%d",
                 (unsigned)ep, TEMP_EP_NAME[i], (unsigned)status, (int)wrote);
}

void zbdiag_note_tick(uint32_t sweeps, uint16_t faults)
{
    s_zb.cur.ticks++;
    s_zb.cur.sweeps = sweeps;
    s_zb.cur.faults = faults;
}

void zbdiag_boot_init(void)
{
    uint8_t rr = (uint8_t)esp_reset_reason();

    if (diag_ring_warm(&s_zb, sizeof s_zb, ZB_TEMP_STATS_MAGIC)) {
        diag_ring_shift(s_zb.hist, sizeof s_zb.hist[0], DIAG_RING_DEPTH);
        s_zb.hist[0] = s_zb.cur;
        const zb_run_t *p = &s_zb.hist[0];
        ESP_LOGW(TAG, "previous run#%lu: ticks=%lu sweeps=%lu faults=0x%02x boot=%s (ended -> %s)",
                 (unsigned long)(s_zb.hdr.seq - 1), (unsigned long)p->ticks,
                 (unsigned long)p->sweeps, (unsigned)p->faults,
                 diag_reset_reason_name(p->reset_reason), diag_reset_reason_name(rr));
        for (int i = 0; i < ZB_EP_COUNT; ++i) {
            if (!p->fail[i] && !p->mismatch[i]) continue;
            ESP_LOGW(TAG, "  ep%d %s ok=%lu fail=%lu mism=%lu err=0x%02lx val=%d read=%d",
                     i + 2, TEMP_EP_NAME[i], (unsigned long)p->ok[i], (unsigned long)p->fail[i],
                     (unsigned long)p->mismatch[i], (unsigned long)(uint32_t)p->last_err[i],
                     (int)p->last_val[i], (int)p->last_read[i]);
        }
    } else {
        ESP_LOGI(TAG, "no previous attr-write history (cold power-on)");
    }
    memset(&s_zb.cur, 0, sizeof s_zb.cur);
    s_zb.cur.reset_reason = rr;
    ESP_LOGW(TAG, "run#%lu started, reset reason: %s (%u)",
             (unsigned long)s_zb.hdr.seq, diag_reset_reason_name(rr), (unsigned)rr);
}

static size_t fmt_run_block(char *o, size_t n, size_t u, const char *label, const zb_run_t *r)
{
    u = diag_appendf(o, n, u, "%s ticks=%lu sweeps=%lu faults=0x%02x boot=%s\n",
                     label, (unsigned long)r->ticks, (unsigned long)r->sweeps,
                     (unsigned)r->faults, diag_reset_reason_name(r->reset_reason));
    for (int i = 0; i < ZB_EP_COUNT; ++i)
        u = diag_appendf(o, n, u,
                         "  %-6s ok=%lu fail=%lu mism=%lu err=0x%02lx val=%d read=%d\n",
                         TEMP_EP_NAME[i], (unsigned long)r->ok[i], (unsigned long)r->fail[i],
                         (unsigned long)r->mismatch[i], (unsigned long)(uint32_t)r->last_err[i],
                         (int)r->last_val[i], (int)r->last_read[i]);
    return u;
}

/* Console `zbtemp`: the live run plus the last DIAG_RING_DEPTH completed ones. Read the
 * history, not `run#` -- by the time you can type this, the run that mattered is two
 * resets back. */
void zigbee_format_temp_stats(char *o, size_t n)
{
    if (!o || n == 0) return;
    o[0] = '\0';
    char lbl[28];
    snprintf(lbl, sizeof lbl, "run#%lu:", (unsigned long)s_zb.hdr.seq);
    size_t u = fmt_run_block(o, n, 0, lbl, &s_zb.cur);
    for (int h = 0; h < DIAG_RING_DEPTH; ++h) {
        if (!s_zb.hist[h].ticks && !s_zb.hist[h].sweeps) continue;   /* slot never used */
        snprintf(lbl, sizeof lbl, "prev-%d (run#%lu):", h + 1,
                 (unsigned long)(s_zb.hdr.seq - (uint32_t)(h + 1)));
        u = fmt_run_block(o, n, u, lbl, &s_zb.hist[h]);
    }
}
```

Add `#include <stdio.h>` for `snprintf`.

- [ ] **Step 3: Strip the tally out of `zigbee.c`**

Delete these regions from `firmware/main/zigbee.c`:
- lines 618-669 — the `---- Temperature attribute write tally ----` comment block, `ZB_TEMP_STATS_MAGIC`, `ZB_RUN_HISTORY`, `zb_run_t`, `zb_temp_stats_t`, `static RTC_NOINIT_ATTR zb_temp_stats_t s_zb;`, and `TEMP_EP_NAME`
- lines 715-800 — the forward declaration of `reset_reason_name`, `fmt_run_block`, `zigbee_format_temp_stats`, `reset_reason_name`, `zb_temp_stats_init`
- line 24 — `static void zb_temp_stats_init(void);` forward declaration

Add `#include "zigbee_diag.h"` to the include block.

Replace `zigbee_report_temps` (lines 671-713) with:

```c
void zigbee_report_temps(void)
{
    struct { uint8_t ep; sensor_id_t id; } map[] = {
        { EP_T_SUPPLY, SENS_SUPPLY }, { EP_T_RETURN, SENS_RETURN },
        { EP_T_SOURCE, SENS_SOURCE }, { EP_T_HXA, SENS_HX_A }, { EP_T_HXB, SENS_HX_B },
    };
    esp_zb_lock_acquire(portMAX_DELAY);
    for (size_t i = 0; i < 5; ++i) {
        int16_t v = temp_centi(map[i].id);
        esp_zb_zcl_status_t st = esp_zb_zcl_set_attribute_val(
            map[i].ep, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &v, false);
        if (st == ESP_ZB_ZCL_STATUS_SUCCESS) {
            /* Read straight back out of the ZCL table -- the same place the reporting
             * engine sources from. SUCCESS on the write is not evidence the table took it. */
            esp_zb_zcl_attr_t *a = esp_zb_zcl_get_attribute(
                map[i].ep, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID);
            int16_t back = (a && a->data_p) ? *(int16_t *)a->data_p : (int16_t)0x7FFF;
            zbdiag_note_ok((int)i, map[i].ep, v, back);
        } else {
            zbdiag_note_fail((int)i, map[i].ep, (int32_t)st, v);
        }
    }
    esp_zb_lock_release();
    zbdiag_note_tick(sensors_sweep_count(), control_task_faults());
}
```

The lock discipline is unchanged: the recorders touch only RTC RAM and `ESP_LOG*`, never `esp_zb_*`, so they are safe inside the locked region — but `zbdiag_note_tick` stays outside it, exactly where the old counter updates were.

Finally, in `zigbee_start()` (line 571), replace the call to `zb_temp_stats_init()` with `zbdiag_boot_init()`.

- [ ] **Step 4: Register the new source file**

In `firmware/main/CMakeLists.txt`, add `"zigbee_diag.c"` to the `SRCS` list, next to `"zigbee.c"`.

- [ ] **Step 5: Build and verify**

```bash
. $HOME/esp/esp-idf/export.sh
cd /Users/kleist/Sites/ValveController/firmware && idf.py build
```

Expected: succeeds. If `TEMP_EP_NAME` is reported undefined, some other function in `zigbee.c` was using it — move that function to `zigbee_diag.c` too, or expose the table via `zigbee_diag.h`; do not duplicate the array.

```bash
SP=/private/tmp/claude-502/-Users-kleist-Sites-ValveController/c85c92c0-629d-4bf6-aa58-e2352fc544a3/scratchpad
ctest --test-dir $SP/tb --output-on-failure
```

Expected: **18/18 passing** (19/19 if Task 5 is already done).

- [ ] **Step 6: Commit**

```bash
cd /Users/kleist/Sites/ValveController
git add firmware/main/zigbee_diag.c firmware/main/zigbee_diag.h \
        firmware/main/zigbee.c firmware/main/CMakeLists.txt
git commit -m "refactor(fw): extract the zigbee write tally onto the shared diag ring"
```

---

## Task 5: One definition of every clamp range and default (`ctrl_core`)

**Files:**
- Modify: `firmware/components/ctrl_core/include/ctrl_core/config_map.h` (whole file, currently 23 lines)
- Modify: `firmware/components/ctrl_core/config_map.c` (whole file, currently 49 lines)
- Test: `firmware/test_host/test_config_clamp.c` (create)
- Test: `firmware/test_host/test_config_map.c` (existing — must keep passing unchanged)

**Interfaces:**
- Consumes: `ctrl_clampf` from `ctrl_core/types.h`, `INTERLOCK_MIN_PULSE_MS` from `ctrl_core/interlock.h`.
- Produces, for Task 6 to consume:
  - `tunable_cfg_t` gains `float valve_deadband_pct;` as its **last** field.
  - `tunable_id_t` gains `TUNABLE_VALVE_DEADBAND`, `TUNABLE_HYSTERESIS`, `TUNABLE_ENTER_DWELL_MS`, `TUNABLE_LEAVE_DWELL_MS`, `TUNABLE_COUNT` — all appended, existing values unchanged.
  - `float tunable_sane_f(float cur, float v, float lo, float hi);`
  - `uint32_t tunable_clamp_u32(uint32_t v, uint32_t lo, uint32_t hi);`
  - `float valve_deadband_floor_pct(uint32_t travel_time_s);`
  - `void tunable_clamp_all(tunable_cfg_t *c);`
  - `const tunable_spec_t *tunable_spec(tunable_id_t id);`
  - `tunable_apply` and `tunable_cfg_defaults` keep their existing signatures.

**Why a table rather than two switch statements.** The point of this task is that each clamp range has exactly one definition. Two `switch` blocks in one file is still two definitions — closer together, but able to drift. A single `TUNABLE_SPEC[]` table that both `tunable_apply` and `tunable_clamp_all` read makes drift impossible, and it makes the display/AUX board's per-channel `input_mode` a one-line addition instead of three edits.

- [ ] **Step 1: Write the failing tests**

Create `firmware/test_host/test_config_clamp.c`:

```c
#include "unity.h"
#include <math.h>
#include <stdint.h>
#include "ctrl_core/config_map.h"

void setUp(void){} void tearDown(void){}

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
            float v = *(const float *)(base + s->off);
            TEST_ASSERT_FALSE(isnan(v));
            /* valve_deadband's floor is dynamic and >= its table flo, so test the ceiling
             * generically and the floor via its own test below. */
            TEST_ASSERT_TRUE(v <= s->fhi + 1e-6f);
            if ((tunable_id_t)id != TUNABLE_VALVE_DEADBAND)
                TEST_ASSERT_TRUE(v >= s->flo - 1e-6f);
        } else if (s->kind == TK_U32) {
            uint32_t v = *(const uint32_t *)(base + s->off);
            TEST_ASSERT_TRUE(v >= s->ulo && v <= s->uhi);
        }
    }
}

void test_defaults_survive_clamp_all_unchanged(void){
    tunable_cfg_t a, b;
    tunable_cfg_defaults(&a);
    tunable_cfg_defaults(&b);
    tunable_clamp_all(&b);
    TEST_ASSERT_EQUAL_MEMORY(&a, &b, sizeof a);   /* shipped defaults are all in range */
}

void test_clamp_all_rejects_nan_back_to_default(void){
    tunable_cfg_t c; tunable_cfg_defaults(&c);
    c.kp = NAN; c.heat_setpoint = NAN;
    tunable_clamp_all(&c);
    TEST_ASSERT_EQUAL_FLOAT(2.8f,  c.kp);
    TEST_ASSERT_EQUAL_FLOAT(35.0f, c.heat_setpoint);
}

void test_clamp_all_pulls_out_of_range_values_in(void){
    tunable_cfg_t c; tunable_cfg_defaults(&c);
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
    tunable_cfg_t c; tunable_cfg_defaults(&c);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, c.valve_deadband_pct);
    uint32_t t = 30;
    tunable_apply(&c, TUNABLE_TRAVEL_TIME_S, &t);
    TEST_ASSERT_EQUAL_UINT32(30, c.travel_time_s);
    /* The deadband must have been re-floored, not left stranded at 1.0 below the new floor. */
    TEST_ASSERT_TRUE(c.valve_deadband_pct >= valve_deadband_floor_pct(30) - 1e-6f);
}

void test_valve_deadband_is_writable_as_a_tunable(void){
    tunable_cfg_t c; tunable_cfg_defaults(&c);
    float v = 3.0f;  tunable_apply(&c, TUNABLE_VALVE_DEADBAND, &v);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, c.valve_deadband_pct);
    v = 99.0f;       tunable_apply(&c, TUNABLE_VALVE_DEADBAND, &v);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, c.valve_deadband_pct);    /* ceiling */
}

void test_out_of_range_id_is_ignored(void){
    tunable_cfg_t a, c;
    tunable_cfg_defaults(&a); tunable_cfg_defaults(&c);
    float v = 1.0f;
    tunable_apply(&c, (tunable_id_t)TUNABLE_COUNT, &v);
    tunable_apply(&c, (tunable_id_t)9999, &v);
    TEST_ASSERT_EQUAL_MEMORY(&a, &c, sizeof a);
    TEST_ASSERT_NULL(tunable_spec((tunable_id_t)TUNABLE_COUNT));
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
    return UNITY_END();
}
```

Add `#include <string.h>` at the top for `memset`/`TEST_ASSERT_EQUAL_MEMORY`.

- [ ] **Step 2: Run the tests to verify they fail**

```bash
SP=/private/tmp/claude-502/-Users-kleist-Sites-ValveController/c85c92c0-629d-4bf6-aa58-e2352fc544a3/scratchpad
cd /Users/kleist/Sites/ValveController/firmware/test_host
cmake -B $SP/tb -S . >/dev/null && cmake --build $SP/tb -j8
```

Expected: build FAILS — `tunable_spec_t`, `TUNABLE_COUNT`, `tunable_clamp_all`, `valve_deadband_floor_pct` and `TUNABLE_VALVE_DEADBAND` do not exist yet. Re-run `cmake -B` (not just `--build`) so the new test file is picked up by the glob.

- [ ] **Step 3: Rewrite the header**

Replace `firmware/components/ctrl_core/include/ctrl_core/config_map.h` with:

```c
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "ctrl_core/types.h"

/* New ids are APPENDED -- existing values must not shift, because main/config.c
 * static-asserts the ATTR_* -> TUNABLE_* mapping against them. */
typedef enum {
    TUNABLE_HEAT_THRESHOLD=0, TUNABLE_COOL_THRESHOLD, TUNABLE_TRAVEL_TIME_S,
    TUNABLE_PARK_POS, TUNABLE_DIRECTION_SWAP, TUNABLE_KP, TUNABLE_KI,
    TUNABLE_GOV_HIGH, TUNABLE_GOV_LOW, TUNABLE_ALARM_DWELL_MS,
    TUNABLE_HEAT_SETPOINT, TUNABLE_COOL_SETPOINT,
    TUNABLE_DEADTIME_S, TUNABLE_PI_DEADBAND,
    /* appended in 1.7.0 */
    TUNABLE_VALVE_DEADBAND, TUNABLE_HYSTERESIS,
    TUNABLE_ENTER_DWELL_MS, TUNABLE_LEAVE_DWELL_MS,
    TUNABLE_COUNT
} tunable_id_t;

/* Field order is load-bearing: main/config.h embeds this as config_t's first member and
 * static-asserts that every offset is unchanged, because the v3 NVS blob is discriminated
 * by sizeof(config_t). Add new fields at the END, and only with a CONFIG_VERSION bump. */
typedef struct {
    float heat_threshold, cool_threshold, hysteresis, heat_setpoint, cool_setpoint, park_pos;
    uint32_t travel_time_s; bool direction_swap; float kp, ki, gov_high, gov_low;
    uint32_t alarm_dwell_ms, enter_dwell_ms, leave_dwell_ms;
    float deadtime_s, pi_deadband_k, valve_deadband_pct;
} tunable_cfg_t;

typedef enum { TK_F32 = 0, TK_U32, TK_BOOL } tunable_kind_t;

/* The single definition of every tunable's location, type and legal range. Both
 * tunable_apply() and tunable_clamp_all() read it, so the two cannot disagree. */
typedef struct {
    tunable_kind_t kind;
    uint16_t       off;        /* offsetof(tunable_cfg_t, field) */
    float          flo, fhi;   /* TK_F32 bounds */
    uint32_t       ulo, uhi;   /* TK_U32 bounds */
} tunable_spec_t;

const tunable_spec_t *tunable_spec(tunable_id_t id);   /* NULL if id is out of range */

void tunable_cfg_defaults(tunable_cfg_t *c);
/* Clamp one field from a raw value of the matching type. Unknown id: no-op. */
void tunable_apply(tunable_cfg_t *c, tunable_id_t id, const void *val);
/* Clamp every field at once -- the load path's defensive pass over a stored blob. */
void tunable_clamp_all(tunable_cfg_t *c);

/* NaN -> keep cur; finite -> clamp to [lo,hi]. A NaN passes both ctrl_clampf()
 * comparisons as false and would otherwise flow through unmodified. */
float    tunable_sane_f(float cur, float v, float lo, float hi);
uint32_t tunable_clamp_u32(uint32_t v, uint32_t lo, uint32_t hi);

/* The interlock enforces a minimum drive pulse, so the smallest motion the motor can
 * make is (min_pulse/travel_time)*100 %. The deadband must exceed half that quantum or
 * the position estimator ping-pongs across the band edge on every pulse -- reviewer
 * simulation found a constant floor below this limit-cycles the motor at 16.7 % duty
 * with a resync roughly every 10 min. 1.2x margin on the bare half-quantum. */
float valve_deadband_floor_pct(uint32_t travel_time_s);
```

- [ ] **Step 4: Rewrite the implementation**

Replace `firmware/components/ctrl_core/config_map.c` with:

```c
#include "ctrl_core/config_map.h"
#include "ctrl_core/interlock.h"
#include <math.h>
#include <stddef.h>

float tunable_sane_f(float cur, float v, float lo, float hi)
{
    if (isnan(v)) return cur;
    return ctrl_clampf(v, lo, hi);
}

uint32_t tunable_clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

float valve_deadband_floor_pct(uint32_t travel_time_s)
{
    if (travel_time_s == 0) return 5.0f;      /* degenerate: clamp to the ceiling */
    float q = 1.2f * (INTERLOCK_MIN_PULSE_MS / 1000.0f) * 100.0f / (2.0f * (float)travel_time_s);
    return q > 0.2f ? q : 0.2f;
}

#define F(field) (uint16_t)offsetof(tunable_cfg_t, field)

/* THE table. Every clamp range in the firmware is defined here exactly once.
 * gov_high/gov_low bounds keep the governor's trip thresholds strictly outside the
 * 35/17 release band (control_task.c) -- inside it, the governor limit-cycles. Observed
 * live with gov_low written as 17, sitting on the band edge. */
static const tunable_spec_t SPEC[TUNABLE_COUNT] = {
    [TUNABLE_HEAT_THRESHOLD] = { TK_F32,  F(heat_threshold),    10.0f, 60.0f,  0, 0 },
    [TUNABLE_COOL_THRESHOLD] = { TK_F32,  F(cool_threshold),     0.0f, 40.0f,  0, 0 },
    [TUNABLE_TRAVEL_TIME_S]  = { TK_U32,  F(travel_time_s),      0.0f,  0.0f, 30, 600 },
    [TUNABLE_PARK_POS]       = { TK_F32,  F(park_pos),           0.0f, 100.0f, 0, 0 },
    [TUNABLE_DIRECTION_SWAP] = { TK_BOOL, F(direction_swap),     0.0f,  0.0f,  0, 0 },
    [TUNABLE_KP]             = { TK_F32,  F(kp),                 0.5f, 15.0f,  0, 0 },
    [TUNABLE_KI]             = { TK_F32,  F(ki),                 0.0f,  5.0f,  0, 0 },
    [TUNABLE_GOV_HIGH]       = { TK_F32,  F(gov_high),          36.0f, 60.0f,  0, 0 },
    [TUNABLE_GOV_LOW]        = { TK_F32,  F(gov_low),            0.0f, 16.0f,  0, 0 },
    [TUNABLE_ALARM_DWELL_MS] = { TK_U32,  F(alarm_dwell_ms),     0.0f,  0.0f, 10000, 3600000 },
    [TUNABLE_HEAT_SETPOINT]  = { TK_F32,  F(heat_setpoint),     17.0f, 35.0f,  0, 0 },
    [TUNABLE_COOL_SETPOINT]  = { TK_F32,  F(cool_setpoint),     17.0f, 35.0f,  0, 0 },
    [TUNABLE_DEADTIME_S]     = { TK_F32,  F(deadtime_s),         0.0f, 120.0f, 0, 0 },
    [TUNABLE_PI_DEADBAND]    = { TK_F32,  F(pi_deadband_k),      0.0f,  1.0f,  0, 0 },
    /* flo is the STATIC floor; the real floor is dynamic, see valve_deadband_floor_pct() */
    [TUNABLE_VALVE_DEADBAND] = { TK_F32,  F(valve_deadband_pct), 0.2f,  5.0f,  0, 0 },
    [TUNABLE_HYSTERESIS]     = { TK_F32,  F(hysteresis),         0.0f, 10.0f,  0, 0 },
    [TUNABLE_ENTER_DWELL_MS] = { TK_U32,  F(enter_dwell_ms),     0.0f,  0.0f, 10000, 3600000 },
    [TUNABLE_LEAVE_DWELL_MS] = { TK_U32,  F(leave_dwell_ms),     0.0f,  0.0f, 10000, 7200000 },
};

#undef F

const tunable_spec_t *tunable_spec(tunable_id_t id)
{
    if ((int)id < 0 || (int)id >= TUNABLE_COUNT) return NULL;
    return &SPEC[id];
}

void tunable_cfg_defaults(tunable_cfg_t *c){
    c->heat_threshold=28; c->cool_threshold=16; c->hysteresis=2;
    c->heat_setpoint=35; c->cool_setpoint=18; c->park_pos=50;
    c->travel_time_s=120; c->direction_swap=false; c->kp=2.8f; c->ki=0.9f;
    c->gov_high=36; c->gov_low=16; c->alarm_dwell_ms=300000;
    c->enter_dwell_ms=60000; c->leave_dwell_ms=420000;
    c->deadtime_s=30.0f; c->pi_deadband_k=0.25f; c->valve_deadband_pct=1.0f;
}

void tunable_apply(tunable_cfg_t *c, tunable_id_t id, const void *v)
{
    const tunable_spec_t *s = tunable_spec(id);
    if (!c || !v || !s) return;
    char *base = (char *)c;
    switch (s->kind) {
    case TK_F32: {
        float *f = (float *)(void *)(base + s->off);
        float lo = (id == TUNABLE_VALVE_DEADBAND)
                 ? valve_deadband_floor_pct(c->travel_time_s) : s->flo;
        *f = tunable_sane_f(*f, *(const float *)v, lo, s->fhi);
        break;
    }
    case TK_U32: {
        uint32_t *u = (uint32_t *)(void *)(base + s->off);
        *u = tunable_clamp_u32(*(const uint32_t *)v, s->ulo, s->uhi);
        break;
    }
    case TK_BOOL: {
        bool *b = (bool *)(void *)(base + s->off);
        *b = *(const bool *)v;
        break;
    }
    }
    /* Shortening travel raises valve_deadband_pct's stability floor, so an already-set
     * deadband must be re-clamped or it is left stranded below the new floor. */
    if (id == TUNABLE_TRAVEL_TIME_S) {
        float lo = valve_deadband_floor_pct(c->travel_time_s);
        c->valve_deadband_pct = ctrl_clampf(c->valve_deadband_pct, lo, 5.0f);
    }
}

void tunable_clamp_all(tunable_cfg_t *c)
{
    if (!c) return;
    tunable_cfg_t d;
    tunable_cfg_defaults(&d);
    char *base = (char *)c;
    const char *dbase = (const char *)&d;

    /* travel_time_s FIRST: valve_deadband_pct's floor is derived from it, so the floor
     * must be computed from the final, in-range travel value. */
    c->travel_time_s = tunable_clamp_u32(c->travel_time_s, 30, 600);

    for (int id = 0; id < TUNABLE_COUNT; ++id) {
        const tunable_spec_t *s = &SPEC[id];
        if (id == TUNABLE_VALVE_DEADBAND) continue;   /* dynamic floor, handled below */
        switch (s->kind) {
        case TK_F32: {
            float *f  = (float *)(void *)(base + s->off);
            float dv  = *(const float *)(const void *)(dbase + s->off);
            *f = tunable_sane_f(dv, *f, s->flo, s->fhi);   /* NaN -> shipped default */
            break;
        }
        case TK_U32: {
            uint32_t *u = (uint32_t *)(void *)(base + s->off);
            *u = tunable_clamp_u32(*u, s->ulo, s->uhi);
            break;
        }
        case TK_BOOL: break;                              /* always in range */
        }
    }

    /* The "cur" fallback is itself clamped into the dynamic range so a NaN-corrupted
     * stored value cannot fall back to a stale 1.0 that is below the floor for a short
     * travel_time_s. */
    float lo = valve_deadband_floor_pct(c->travel_time_s);
    c->valve_deadband_pct = tunable_sane_f(ctrl_clampf(d.valve_deadband_pct, lo, 5.0f),
                                           c->valve_deadband_pct, lo, 5.0f);
}
```

- [ ] **Step 5: Run the tests to verify they pass**

```bash
SP=/private/tmp/claude-502/-Users-kleist-Sites-ValveController/c85c92c0-629d-4bf6-aa58-e2352fc544a3/scratchpad
cd /Users/kleist/Sites/ValveController/firmware/test_host
cmake -B $SP/tb -S . >/dev/null && cmake --build $SP/tb -j8 >/dev/null && ctest --test-dir $SP/tb --output-on-failure
```

Expected: **19/19 passing**. `test_config_map` (the pre-existing one) must still pass **without edits** — it exercises the same clamps through `tunable_apply`, so if the table transcribed a bound wrongly it fails here. That is the point of leaving it untouched.

If `test_defaults_survive_clamp_all_unchanged` fails, a shipped default sits outside its own declared range — report which field rather than widening the range to make the test pass.

- [ ] **Step 6: Commit**

```bash
cd /Users/kleist/Sites/ValveController
git add firmware/components/ctrl_core/include/ctrl_core/config_map.h \
        firmware/components/ctrl_core/config_map.c \
        firmware/test_host/test_config_clamp.c
git commit -m "refactor(fw): define every tunable clamp range once, in a table"
```

---

## Task 6: Rewire `config.c` onto the single source

**Files:**
- Modify: `firmware/main/config.h` (whole file, currently 46 lines)
- Modify: `firmware/main/config.c:16-24` (`DEFAULTS`), `:58-126` (`sane_f`, `clamp_u32`, `valve_deadband_floor_pct`, `clamp_config`), `:128-205` (`config_load`), `:218-267` (`config_factory_reset`, `config_apply_custom`)
- Modify: `firmware/main/control_task.c`, `firmware/main/valve_hw.c` (16 `g_config.X` → `g_config.t.X` references)
- Modify: `firmware/components/ctrl_core/config_map.c`, `include/ctrl_core/config_map.h` — add `tunable_from_attr`
- Test: `firmware/test_host/test_config_clamp.c` (extend)

**Interfaces:**
- Consumes: everything Task 5 produced.
- Produces:
  - `bool tunable_from_attr(uint16_t attr_id, tunable_id_t *out);` in `ctrl_core` — true and `*out` set for a writable tunable attribute, false for read-only/unknown ones.
  - `config_t` becomes `{ tunable_cfg_t t; uint32_t cfg_version; }`. Public functions `config_load`, `config_save`, `config_factory_reset`, `config_apply_custom`, `config_water_running_load`, `config_water_running_save` all keep their signatures.

**The layout constraint, restated because it is the one thing that can damage the plant.** `sizeof(config_t)` must stay **76** with `t` at offset 0. This was verified on the host toolchain: `tunable_cfg_t` with `valve_deadband_pct` appended is 72 bytes, plus `cfg_version` at offset 72 gives 76, with every inner field at its current offset. The `_Static_assert`s in Step 2 prove it on the RISC-V target at build time. If they fail, **stop** — do not adjust `CONFIG_VERSION` to work around it, because bumping the version silently discards every deployed device's stored tuning.

- [ ] **Step 1: Add the attribute mapping to `ctrl_core`**

Append to `firmware/components/ctrl_core/include/ctrl_core/config_map.h`, before the final line:

```c
/* Zigbee custom-cluster attribute id -> tunable id. Numeric because ctrl_core cannot see
 * main/zigbee.h; main/config.c static-asserts that ATTR_* still match these values.
 * Returns false for read-only or unknown attributes (resync, the bitmaps, travel-since). */
bool tunable_from_attr(uint16_t attr_id, tunable_id_t *out);
```

Append to `firmware/components/ctrl_core/config_map.c`:

```c
bool tunable_from_attr(uint16_t attr_id, tunable_id_t *out)
{
    tunable_id_t id;
    switch (attr_id) {
    case 0x0000: id = TUNABLE_HEAT_THRESHOLD; break;
    case 0x0001: id = TUNABLE_COOL_THRESHOLD; break;
    case 0x0002: id = TUNABLE_TRAVEL_TIME_S;  break;
    case 0x0003: id = TUNABLE_PARK_POS;       break;
    case 0x0004: id = TUNABLE_DIRECTION_SWAP; break;
    case 0x0005: id = TUNABLE_KP;             break;
    case 0x0006: id = TUNABLE_KI;             break;
    case 0x0007: id = TUNABLE_GOV_HIGH;       break;
    case 0x0008: id = TUNABLE_GOV_LOW;        break;
    case 0x0009: id = TUNABLE_ALARM_DWELL_MS; break;
    case 0x000E: id = TUNABLE_DEADTIME_S;     break;
    case 0x000F: id = TUNABLE_PI_DEADBAND;    break;
    case 0x0010: id = TUNABLE_HEAT_SETPOINT;  break;
    case 0x0011: id = TUNABLE_COOL_SETPOINT;  break;
    case 0x0012: id = TUNABLE_VALVE_DEADBAND; break;
    /* 0x000A resync, 0x000B alarm bitmap, 0x000C fault bitmap, 0x000D travel-since:
     * read-only or command-like, never a stored tunable. */
    default: return false;
    }
    if (out) *out = id;
    return true;
}
```

- [ ] **Step 2: Add the equivalence test**

Append these two tests to `firmware/test_host/test_config_clamp.c` and register them in `main()`:

```c
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
            tunable_cfg_defaults(&a); tunable_cfg_defaults(&b);
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
```

Run the suite: `cmake -B $SP/tb -S . >/dev/null && cmake --build $SP/tb -j8 >/dev/null && ctest --test-dir $SP/tb --output-on-failure`. Expected: **19/19 passing** with `test_config_clamp` now reporting 10 tests.

- [ ] **Step 3: Rewrite `config.h`**

Replace `firmware/main/config.h` with:

```c
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "ctrl_core/config_map.h"

#define CONFIG_VERSION 3

/* The tunables live in ctrl_core so their clamp ranges and defaults have exactly one
 * definition (config_map.c's SPEC table). config_t is that struct plus the persistence
 * tag -- deliberately NOT a second copy of the fields.
 *
 * The layout is load-bearing: config_load() discriminates NVS blob versions by size, so
 * sizeof(config_t) must stay 76 with every field at its historical offset or every
 * deployed device silently reverts to defaults. The asserts below enforce that. */
typedef struct {
    tunable_cfg_t t;
    uint32_t      cfg_version;
} config_t;

_Static_assert(sizeof(config_t) == 76, "config_t grew or shrank -- the v3 NVS blob is size-discriminated");
_Static_assert(offsetof(config_t, t) == 0, "tunables must stay at offset 0 (v1 migration memcpys a prefix)");
_Static_assert(offsetof(config_t, cfg_version) == 72, "cfg_version moved");
_Static_assert(offsetof(config_t, t.heat_threshold) == 0,  "field moved");
_Static_assert(offsetof(config_t, t.travel_time_s) == 24,  "field moved");
_Static_assert(offsetof(config_t, t.direction_swap) == 28, "field moved");
_Static_assert(offsetof(config_t, t.kp) == 32,             "field moved");
_Static_assert(offsetof(config_t, t.alarm_dwell_ms) == 48, "field moved");
_Static_assert(offsetof(config_t, t.deadtime_s) == 60,     "field moved");
_Static_assert(offsetof(config_t, t.valve_deadband_pct) == 68, "field moved");

extern config_t g_config;

void      config_load(void);          /* nvs init + load (defaults on miss) */
esp_err_t config_save(void);
void      config_factory_reset(void); /* erase ns, reload defaults */
void      config_apply_custom(uint16_t attr_id, const void *val); /* zigbee custom-cluster tunable write, persists */

/* water_running (commanded regulation enable) is persisted as its OWN small NVS key,
 * deliberately NOT a field in the cfg blob: a new blob field would change
 * sizeof(config_t) and force a CONFIG_VERSION bump + migration for what is commanded
 * state, not a tunable. Absent key -> false (park at park_pos: the safe boot state,
 * and the value the OnOff attribute is built with). */
bool      config_water_running_load(void);
void      config_water_running_save(bool on);
```

- [ ] **Step 4: Rewrite the changed parts of `config.c`**

Four edits. Everything not listed stays exactly as it is — in particular **do not touch** the `config_v1_t` / `config_v2_t` structs, their explanatory comments, `CONFIG_VERSION_V2`, `config_save`, `config_water_running_load` or `config_water_running_save`.

**(a)** Replace the `DEFAULTS` block (lines 16-24) with a function, since the defaults now come from `ctrl_core` and cannot be a static initialiser:

```c
static config_t config_defaults(void)
{
    config_t c = { .cfg_version = CONFIG_VERSION };
    tunable_cfg_defaults(&c.t);
    return c;
}
```

**(b)** Delete `sane_f` (lines 62-66), `clamp_u32` (69-74), `valve_deadband_floor_pct` (85-89) and the body of `clamp_config` (96-126) — all four now live in `ctrl_core`. Replace `clamp_config` with:

```c
/* Defensive pass after every successful load path so a stale, hand-edited or bit-flipped
 * NVS blob can never leave g_config outside the bounds every write path enforces. */
static void clamp_config(config_t *c) { tunable_clamp_all(&c->t); }
```

Keep the long comment above `valve_deadband_floor_pct` — move it to the function's new home in `config_map.h` (Step 3 of Task 5 already places it there; delete the copy here rather than leaving both).

**(c)** In `config_load`, replace `g_config = DEFAULTS;` with `g_config = config_defaults();`. In the v2 migration branch, add `.t` to each of the 17 field assignments (`g_config.t.heat_threshold = v2.heat_threshold;` and so on) and change the `valve_deadband_pct` line to:

```c
                g_config.t.valve_deadband_pct = config_defaults().t.valve_deadband_pct;
```

In the v1 branch, `memcpy(&g_config, &v1, sizeof(v1))` stays as-is — `offsetof(config_t, t) == 0` is asserted, so the prefix copy still lands correctly. Change the three following lines to `g_config.t.ki`, `g_config.t.deadtime_s`, `g_config.t.pi_deadband_k`, and `g_config.t.valve_deadband_pct`, each taking its default from `config_defaults()`.

**(d)** Replace `config_factory_reset`'s `g_config = DEFAULTS;` with `g_config = config_defaults();`, and replace the whole of `config_apply_custom` (lines 225-267, including its comment) with:

```c
/* Maps a zigbee custom-cluster attribute write onto the matching tunable and persists.
 * The clamp ranges live in ctrl_core/config_map.c's SPEC table -- this function no longer
 * restates them, which is what stopped the two from drifting. Read-only and unknown
 * attributes are ignored. */
void config_apply_custom(uint16_t attr_id, const void *val)
{
    tunable_id_t id;
    if (!tunable_from_attr(attr_id, &id)) return;

    config_t before = g_config;
    tunable_apply(&g_config.t, id, val);
    /* Skip the flash write when the clamped value didn't actually change g_config (a
     * write clamped back to its current value, or an unchanged re-write) -- avoids
     * wearing the NVS partition on no-op traffic. */
    if (memcmp(&before, &g_config, sizeof g_config) != 0) config_save();
}
```

Finally, add the static asserts pinning the numeric table in `ctrl_core` to the real `ATTR_*` macros. Put them just above `config_apply_custom`:

```c
/* tunable_from_attr() hardcodes these numbers because ctrl_core cannot include zigbee.h.
 * Pin them here, where both are visible, so a renumbering breaks the build. */
_Static_assert(ATTR_HEAT_THRESHOLD == 0x0000, "tunable_from_attr table is stale");
_Static_assert(ATTR_ALARM_DWELL    == 0x0009, "tunable_from_attr table is stale");
_Static_assert(ATTR_DEADTIME_S     == 0x000E, "tunable_from_attr table is stale");
_Static_assert(ATTR_HEAT_SETPOINT  == 0x0010, "tunable_from_attr table is stale");
_Static_assert(ATTR_COOL_SETPOINT  == 0x0011, "tunable_from_attr table is stale");
_Static_assert(ATTR_VALVE_DEADBAND == 0x0012, "tunable_from_attr table is stale");
```

`config.c` already includes `zigbee.h`, so `ATTR_*` is in scope.

- [ ] **Step 5: Fix the call sites**

```bash
. $HOME/esp/esp-idf/export.sh
cd /Users/kleist/Sites/ValveController/firmware && idf.py build 2>&1 | grep -E 'error|has no member' | head -40
```

Every `g_config.<tunable field>` outside `config.c` is now a compile error. There are 16 such references — 10 in `control_task.c`, 6 in `valve_hw.c` — plus 34 in `zigbee.c`. Rewrite each as `g_config.t.<field>`. `g_config.cfg_version` is **not** one of them; it stays at the top level.

This is the safety property of the embed: a missed rename cannot compile, so it cannot reach the plant.

Repeat until `idf.py build` succeeds with no new warnings.

- [ ] **Step 6: Verify nothing moved**

The build already ran the `_Static_assert`s. Confirm explicitly that the persisted layout is intact:

```bash
cd /Users/kleist/Sites/ValveController/firmware
grep -c '_Static_assert' main/config.h     # expect 10
idf.py build 2>&1 | tail -5                # expect a clean binary summary
```

A `_Static_assert` failure here is a **stop condition**, not something to work around. Report the failing assert and the actual `sizeof`/offset.

- [ ] **Step 7: Run the host suite**

```bash
SP=/private/tmp/claude-502/-Users-kleist-Sites-ValveController/c85c92c0-629d-4bf6-aa58-e2352fc544a3/scratchpad
ctest --test-dir $SP/tb --output-on-failure
```

Expected: **19/19 passing**.

- [ ] **Step 8: Commit**

```bash
cd /Users/kleist/Sites/ValveController
git add firmware/main/config.h firmware/main/config.c \
        firmware/main/control_task.c firmware/main/valve_hw.c firmware/main/zigbee.c \
        firmware/components/ctrl_core/include/ctrl_core/config_map.h \
        firmware/components/ctrl_core/config_map.c \
        firmware/test_host/test_config_clamp.c
git commit -m "refactor(fw): delegate config clamping to ctrl_core's single source"
```

---

## Task 7: Split the rest of `zigbee.c`

**Files:**
- Create: `firmware/main/zigbee_internal.h`, `firmware/main/zigbee_attrs.c`, `firmware/main/zigbee_telem.c`
- Modify: `firmware/main/zigbee.c` — retains init, endpoint/cluster registration, join/steer signal handling
- Modify: `firmware/main/CMakeLists.txt`

**Interfaces:**
- Consumes: `zigbee_diag.h` (Task 4), `ctrl_core/config_map.h` (Tasks 5-6).
- Produces: no change to the public API in `zigbee.h`. `zigbee_start`, `zigbee_steer`, `zigbee_leave`, `zigbee_joined`, `zigbee_report_temps`, `zigbee_push_status`, `zigbee_on_join`, `zigbee_format_temp_stats` all keep their signatures and behaviour.

**Bucket assignment** (line numbers are pre-Task-4; after Task 4 the DIAG lines are already gone):

| Stays in `zigbee.c` (CORE) | Moves to `zigbee_attrs.c` | Moves to `zigbee_telem.c` |
|---|---|---|
| `TAG` :22, `s_joined` :25 | `attr_cb` :373-529 | `ZCL_TEMP_INVALID` :605, `temp_centi` :611 |
| `STEER_RETRY_*` :31-33, `s_retry_ms` | | `zigbee_report_temps` :671 |
| `configure_reporting_on_join` :51 | | `push_running_mode_report` :810 |
| `zigbee_on_join` (weak) :65 | | `zigbee_push_status` :835 |
| `MFR`/`MODEL` :68-69 | | `set_local_temperature` :877 |
| `build_custom_cluster` :94 | | `TELEMETRY_PERIOD_MS` :889, `telemetry_task` :891 |
| `add_temp_ep` :157, `ota_file_version` :178 | | `configure_reporting_temp` :903 |
| `fill_zcl_string` :199, `build_endpoints` :208 | | `configure_reporting_position` :924 |
| `schedule_steer_retry`/`steer_retry_cb` :298-316 | | `configure_reporting_running_mode` :944 |
| `mark_joined` :318, `mark_unjoined` :330 | | `configure_reporting_bitmap` :965 |
| `esp_zb_app_signal_handler` :337 | | |
| `action_handler` :530, `zb_task` :543 | | |
| `zigbee_start` :571, `zigbee_steer` :584, `zigbee_leave` :591, `zigbee_joined` :599 | | |

**Cross-bucket symbols — verified by full cross-reference of the file, not estimated:**

Genuinely need `zigbee_internal.h`:

- `s_last_pushed_running_mode` / `RUNNING_MODE_UNSET` :38-39 — reset by CORE (`mark_joined` :325), read and written by TELEM (`push_running_mode_report` :812, :831). No existing accessor. Either expose it, or keep it static in TELEM and give CORE a one-line non-static reset function to call — the latter is cleaner.
- `configure_reporting_*` :903-984 — all four defined in TELEM, all four called by `configure_reporting_on_join` :51-63, which is itself called only from CORE's `mark_joined`. **Cleanest fix: move `configure_reporting_on_join` into TELEM and have CORE call that single entry point**, leaving all four `configure_reporting_*` static.
- `attr_cb` :373-528 — must become non-static so CORE's `action_handler` :530 can dispatch to it. (Moving `action_handler` into ATTRS instead would separate it from `zb_task`, where it is registered with the SDK — don't.)
- `telemetry_task` :891-900 — spawned by CORE's `zb_task` via `xTaskCreate` at :563, so it must become non-static.
- `zb_temp_stats_init` — already handled in Task 4, where it becomes the public `zbdiag_boot_init()`.

Do **not** need sharing, despite looking like they might:

- `s_joined` :25 — read by TELEM, but the public `zigbee_joined()` already exposes it. Have TELEM call that.
- **The 19 `s_attr_*` custom-cluster backing variables** :74-92 — referenced by name *only* inside `build_custom_cluster`. The Zigbee stack mutates them through registered pointers, keyed by attribute id, which is not a C-level reference. They stay static in CORE. This includes `s_attr_travel_since`.
- `now_ms()` :47, `s_retry_ms` :33, `MFR`/`MODEL` :68-69, `SW_BUILD_ID`/`DATE_CODE` :196-197 — all CORE only.
- `s_attr_running_mode` :250 — function-scoped static inside `build_endpoints`, not file-scope.
- `TAG` :22 — used by all buckets, but this is not a sharing problem: give each new file its own `static const char *TAG`. Using a distinct tag per file (`"zigbee_attrs"`, `"zigbee_telem"`) makes the logs more useful, not less.
- `TEMP_EP_NAME` and `s_zb` — already resolved by Task 4, which moved both into `zigbee_diag.c` behind the `zbdiag_note_*` accessors.

`esp_zb_app_signal_handler` :337 is an **SDK-required symbol name** with implicit external linkage — it is declared in no header in this repo. Whichever file keeps it must leave it non-static.

**Lock discipline must not change.** `esp_zb_lock_acquire`/`release` pairs currently sit at :586/:588 (`zigbee_steer`), :593/:595 (`zigbee_leave`), :677/:709 (`zigbee_report_temps`), :837/:874 (`zigbee_push_status`), :879/:883 (`set_local_temperature`). `push_running_mode_report` is called with the lock **already held** by `zigbee_push_status` and must not take it itself. Moving a function between files must not move an acquire or release relative to its callees.

- [ ] **Step 1: Confirm the cross-reference above still holds**

The list above was produced by cross-referencing every file-scope symbol in `zigbee.c` against every use. Tasks 4 and 6 have since edited the file, so re-confirm rather than assume:

```bash
cd /Users/kleist/Sites/ValveController/firmware/main
grep -n '^static [^(]*;$\|^static const [^(]*;$\|^static .*\[\].*=' zigbee.c
grep -c 's_attr_travel_since' zigbee.c    # expect 2: declaration + build_custom_cluster
```

If a symbol appears that is not accounted for above, cross-reference it (`grep -n "\bNAME\b" zigbee.c`) and place it in the bucket that uses it before proceeding. Anything used by exactly one bucket stays `static` in that bucket's file.

- [ ] **Step 2: Create `zigbee_internal.h`**

```c
#pragma once
/* Shared between zigbee.c, zigbee_attrs.c and zigbee_telem.c only. NOT public API --
 * everything callers outside main/ need is in zigbee.h.
 *
 * Deliberately small: the 19 s_attr_* custom-cluster backing variables stay static in
 * zigbee.c because the stack mutates them through registered pointers keyed by attribute
 * id, never by C-level name; and TELEM reads join state through the public
 * zigbee_joined() rather than reaching for s_joined. */
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_zigbee_core.h"

/* Defined in zigbee_telem.c. Arms the ZCL reporting engine for every attribute that
 * needs it; called from zigbee.c's mark_joined(). Keeping the four individual
 * configure_reporting_* functions static behind this one entry point is why they do not
 * appear here. */
void zigbee_configure_reporting_on_join(void);

/* Defined in zigbee_telem.c, spawned by zigbee.c's zb_task(). */
void telemetry_task(void *arg);

/* Defined in zigbee_telem.c. Forces the next push to report even if the computed mode is
 * unchanged, so each join gives Z2M an authoritative read; called from mark_joined().
 * This exists so s_last_pushed_running_mode can stay static in zigbee_telem.c. */
void zigbee_reset_running_mode_push(void);

/* Defined in zigbee_attrs.c, called from zigbee.c's action_handler(). */
esp_err_t zb_attr_write_cb(const esp_zb_zcl_set_attr_value_message_t *m);
```

Each new `.c` file gets its own `static const char *TAG` — use `"zigbee_attrs"` and `"zigbee_telem"` so the logs say which half spoke.

- [ ] **Step 3: Move ATTRS first, build, then move TELEM, build again**

Two separate compile-verified moves, not one big cut. Move `attr_cb` (lines 373-529) into `zigbee_attrs.c`, renaming it `zb_attr_write_cb` and removing `static`. Give the new file the includes it needs (`zigbee.h`, `zigbee_internal.h`, `config.h`, `ctrl_core/config_map.h`, `esp_log.h`, plus whatever the body references). Change `action_handler` in `zigbee.c` to call `zb_attr_write_cb`. Add `zigbee_attrs.c` to `SRCS`. Build:

```bash
. $HOME/esp/esp-idf/export.sh
cd /Users/kleist/Sites/ValveController/firmware && idf.py build
```

Then repeat for the TELEM bucket into `zigbee_telem.c`, add it to `SRCS`, and build again. Doing them separately means a link error names one bucket's problem, not two overlapping ones.

- [ ] **Step 4: Verify the split changed nothing observable**

```bash
cd /Users/kleist/Sites/ValveController/firmware
wc -l main/zigbee.c main/zigbee_attrs.c main/zigbee_telem.c main/zigbee_diag.c
grep -n 'esp_zb_lock_acquire\|esp_zb_lock_release' main/zigbee*.c
```

Expected: `zigbee.c` well under 500 lines, and the acquire/release pairs still balanced within each function exactly as listed above — five pairs total, none in `zigbee_attrs.c` or `zigbee_diag.c`, and none inside `push_running_mode_report`.

```bash
idf.py build && ctest --test-dir /private/tmp/claude-502/-Users-kleist-Sites-ValveController/c85c92c0-629d-4bf6-aa58-e2352fc544a3/scratchpad/tb --output-on-failure
```

Expected: clean build, **19/19 passing**.

- [ ] **Step 5: Commit**

```bash
cd /Users/kleist/Sites/ValveController
git add firmware/main/zigbee.c firmware/main/zigbee_attrs.c firmware/main/zigbee_telem.c \
        firmware/main/zigbee_internal.h firmware/main/CMakeLists.txt
git commit -m "refactor(fw): split zigbee.c into core, attrs, telemetry and diagnostics"
```

---

## Task 8: Housekeeping

**Files:**
- Modify: `.gitignore`
- Delete: `firmware/test_host/build-*/` (26 untracked directories, 304 MB)

- [ ] **Step 1: Confirm the build directories are untracked before deleting anything**

```bash
cd /Users/kleist/Sites/ValveController
git status --porcelain firmware/test_host/ | grep -c '^??'
du -sh firmware/test_host/build-* | tail -1
git ls-files firmware/test_host/ | grep -c 'build-'    # MUST print 0
```

If the last command prints anything other than `0`, **stop** — some build output is tracked and deleting it would be a real change. Report instead.

- [ ] **Step 2: Delete them**

```bash
cd /Users/kleist/Sites/ValveController/firmware/test_host
rm -rf build-*/
ls -d build-*/ 2>/dev/null || echo "all gone"
```

- [ ] **Step 3: Stop them coming back**

Append to `/Users/kleist/Sites/ValveController/.gitignore`:

```gitignore
# host test build trees (each carries its own fetched copy of Unity)
firmware/test_host/build*/
# ESP-IDF component-manager cache
firmware/.cache/
```

Verify: `git status --porcelain firmware/ | grep -v '^ M'` should no longer list `firmware/.cache/` or any `build-*` path.

- [ ] **Step 4: Commit**

```bash
cd /Users/kleist/Sites/ValveController
git add .gitignore
git commit -m "chore(fw): ignore host test build trees and the IDF component cache"
```

---

## Task 9: Release 1.7.0

**Files:**
- Modify: `firmware/version.txt`, `firmware/README.md`

- [ ] **Step 1: Record the size before and after**

The pre-refactor 1.6.4 image was **712,864 bytes**, 37.5% of the 1,900,544-byte OTA slot. Measure the new one:

```bash
. $HOME/esp/esp-idf/export.sh
cd /Users/kleist/Sites/ValveController/firmware
idf.py size | head -20
```

Expect roughly break-even — this refactor removes about 250 lines of duplicated logic from a `main/` that is only 23 KB of a 713 KB image, and adds a table. A swing of more than a few KB in either direction means something was dropped or duplicated by mistake; investigate before continuing.

- [ ] **Step 2: Bump the version**

Write `1.7.0` to `firmware/version.txt` (single line, trailing newline).

- [ ] **Step 3: Build with a forced reconfigure**

```bash
cd /Users/kleist/Sites/ValveController/firmware
idf.py reconfigure && idf.py build
strings build/valvectl.bin | grep -m1 '^1\.7\.0' || echo "VERSION NOT IN IMAGE"
```

`idf.py reconfigure` is not optional: this project has shipped an image whose app descriptor carried a stale `PROJECT_VER`, which then reported the wrong version to OTA and to `update.installed_version` in Home Assistant. If the `strings` check does not find `1.7.0`, stop and fix it before flashing.

- [ ] **Step 4: Write the README section**

Add a `## 1.7.0` section at the top of the version history in `firmware/README.md`, following the format of the existing `## 1.6.4` section. Cover: the three RTC rings collapsed onto `ctrl_core/diag_ring`; the three clamp implementations collapsed onto one `SPEC` table with `config_t` embedding `tunable_cfg_t`; the `sizeof(config_t) == 76` static assert and *why* it exists; the four-way `zigbee.c` split and that it is the seam the display/AUX endpoints attach to. State plainly that the image size is essentially unchanged and that flash was never the constraint — 62% of the OTA slot was already free.

- [ ] **Step 5: Flash over USB**

```bash
cd /Users/kleist/Sites/ValveController/firmware
ls /dev/cu.usbmodem*          # confirm the board is attached
idf.py -p /dev/cu.usbmodem<NN> flash
```

Do **not** flash over OTA for this release. The 1.6.1 incident began at an OTA reboot, and a USB flash keeps the recovery path short if the plant misbehaves.

- [ ] **Step 6: Soak on mains with USB unplugged**

This is the verification that actually matters, and it must be done the hard way.

1. Unplug USB completely.
2. Power-cycle the board on mains alone. A cold boot, not a reset with the cable attached — attaching USB both resets the board and masks the class of fault 1.6.4 fixed.
3. Watch Zigbee2MQTT / Home Assistant for **at least 5 minutes**, well past the ~40 s mark where the 1.6.x watchdog loop reset.

Pass criteria, all of which must hold:
- Temperatures keep updating past 40 s and do not drop to the invalid sentinel.
- `fault_bitmap` is `0`.
- `mode` reports a real state (`heating` / `cooling` / `idle`), not a stalled one.
- The valve position tracks rather than dipping to 0 (a dip to 0 with a ~161 s signature is a deliberate end-stop resync, not a fault — but it should not be happening on every boot).

Note that `update.installed_version` in Home Assistant is a **stale cache** and is not evidence of anything; ignore it.

If the plant misbehaves, the fastest recovery is reflashing 1.6.4 (`git checkout acecb8d -- firmware/` then Steps 3 and 5), not debugging live.

- [ ] **Step 7: Commit**

```bash
cd /Users/kleist/Sites/ValveController
git add firmware/version.txt firmware/README.md
git commit -m "feat(fw): release 1.7.0 — deduplicate diagnostics and config clamping"
```

Do **not** stage `pcb/`, `analysis/`, or `z2m/` — those carry the user's own uncommitted work.

---

## Deferred to its own release: 1.7.1

The build is currently `CONFIG_COMPILER_OPTIMIZATION_LEVEL_DEBUG=y` (`-Og`). Switching to `CONFIG_COMPILER_OPTIMIZATION_SIZE=y` is expected to cut 10–20% of the image, but it changes code generation on a live heating plant, so it ships alone with nothing else changed. Assertions stay enabled and `CONFIG_ESP_ERR_TO_NAME_LOOKUP` stays — both are diagnostics on a device that was recently debugged blind, and they buy back headroom that is already in surplus. Same soak as Task 9 Step 6, plus an `idf.py size` diff recorded in the release notes.

This is one sdkconfig line and does not need a plan of its own.
