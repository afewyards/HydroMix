# Firmware refactor — deduplicate diagnostics, unify config, split zigbee.c

Status: approved 2026-08-11. Two releases: **1.7.0** (refactor, behaviour-preserving) then **1.7.1** (sdkconfig only).

## Motivation

Three goals were named: room for the display/AUX board, maintainability, flash headroom.

**Flash headroom is already satisfied and is not a design driver.** Measured on the 1.6.4 build:

| | bytes | share |
|---|---|---|
| Binary total | 712,864 | 37.5% of OTA slot |
| OTA slot (`ota_0`/`ota_1`, 0x1D0000 each) | 1,900,544 | 1,159.8 KiB free |
| Zigbee stack (`zboss_stack` + `zb_api` + `zboss_port`) | 354,147 | 49.7% of image |
| `libmain.a` — all of `main/` | 23,118 | 3.2% |
| `libconsole.a` (esp_console + argtable3) | 15,734 | 2.2% |
| `libctrl_core.a` | 5,986 | 0.8% |

Half the image is the Zigbee stack; project code is 3%. No refactor of `main/` can move the image size
meaningfully — a perfect rewrite of everything hand-written here saves less than the console library.
Size work is therefore isolated into 1.7.1 as a single sdkconfig change, and 1.7.0 is judged purely on
duplication removed and seams created.

The 1.6.1–1.6.4 debugging session left scar tissue: instrumentation was added three times in three
places under time pressure. That duplication sits exactly where the display/AUX firmware must land.

## Release 1.7.0 — refactor

### 1. Extract the RTC history mechanism

Three independent implementations of the same pattern (magic word + seq + live `cur` + `hist[4]`,
shift-on-boot, reset-reason label, bounded-`snprintf` formatter):

| file | struct | roll-on-boot | formatter |
|---|---|---|---|
| `main/taskhb.c` | 20-31 | `hb_boot_report` 65-90 | `hb_format` 92-118 |
| `main/sensors_hw.c` | 80-92 | inline in `sensors_start()` 226-256 | `fmt_run` 325-343 + `sensors_format_stats` 349-363 |
| `main/zigbee.c` | 642-667 | `zb_temp_stats_init` 771-800 | `fmt_run_block` 720-737 + `zigbee_format_temp_stats` 739-751 |

`esp_reset_reason_t → const char *` exists twice (`taskhb.c:35-49` `rr_name`, `zigbee.c:753-769`
`reset_reason_name`) and the two copies already disagree by two cases.

**Payloads stay typed per owner.** Heartbeat timestamps, 1-Wire tallies and attribute-write tallies are
different data; a generic `void *` ring holding all three would be worse than the three copies. Only the
mechanism is shared.

New portable module `components/ctrl_core/diag_ring.c` + `include/ctrl_core/diag_ring.h`. **Zero ESP
dependencies** — it is picked up automatically by the host suite's `../components/ctrl_core/*.c` glob.
`main/` retains ownership of the `RTC_NOINIT_ATTR` storage and of calling `esp_reset_reason()`, because
neither can cross into the portable core.

```c
#define DIAG_RING_DEPTH 4

typedef struct { uint32_t magic; uint32_t seq; } diag_hdr_t;

/* true  = storage survived the reset (seq bumped, caller should record the ended run)
   false = cold start (hdr zeroed, magic stamped, seq = 1) */
bool diag_ring_warm(diag_hdr_t *hdr, uint32_t magic);

/* hist[n] <- hist[n-1] for n descending; hist[0] is left untouched for the caller
   to overwrite with its own typed fields. */
void diag_ring_shift(void *hist, size_t elem, uint8_t depth);

const char *diag_reset_reason_name(int reason);   /* esp_reset_reason_t widened to int */

/* Bounded append. Returns the new used length. Never writes past n-1, always NUL-terminates. */
size_t diag_appendf(char *o, size_t n, size_t used, const char *fmt, ...);
```

`diag_appendf` replaces `fmt_run`, `fmt_run_block`, and the pattern `hb_format` inlines without
extracting. Each hand-rolls the same `u += k; if (u >= n)` bounds arithmetic — the single most
off-by-one-prone code in the tree, and currently untested. It gets tests.

**Host tests:** warm/cold transition and seq behaviour; shift correctness with sentinel-filled arrays
(including depth 1 and full-depth wrap); `diag_appendf` truncation at exact boundary, zero-length
buffer, and repeated append past capacity.

### 2. Collapse the config duplication

`main/config.c:228-269` (`config_apply_custom`, keyed on `ATTR_*`) and `ctrl_core/config_map.c`
(`tunable_apply`, keyed on `TUNABLE_*`) independently implement the same clamping switch over the same
fields with the same ranges: `heat_threshold` 10–60, `cool_threshold` 0–40, `kp` 0.5–15, `ki` 0–5,
`gov_high` 36–60, `gov_low` 0–16, `alarm_dwell` 10000–3600000, `deadtime_s` 0–120, `pi_deadband` 0–1,
`heat`/`cool_setpoint` 17–35.

Only the `ctrl_core` copy has tests. The untested copy is the one that actually serves Zigbee writes.
They will drift.

Defaults are duplicated the same way: `config.c:16-24` (`DEFAULTS`) and `config_map.c`
(`tunable_cfg_defaults`), same numbers as separate literals.

**Change:** delete `config_apply_custom`'s clamping body; replace with an `ATTR_* → TUNABLE_*` mapping
that delegates to `tunable_apply`. Derive `DEFAULTS` from `tunable_cfg_defaults` rather than restating
it. One definition of each clamp, one definition of each default, both tested.

**No NVS schema change** — the v1→v2→v3 blob migration is untouched.

**To verify during implementation:** `tunable_apply`'s value type must accommodate what the display/AUX
work will add. That feature introduces a per-channel boolean `input_mode`, not a clamped numeric. If the
existing signature is numeric-only, extend it in this pass rather than forcing the next feature to
reintroduce a second apply path — which would undo this whole item.

**Host test:** a Zigbee-side attribute write and the equivalent tunable write produce identical clamped
results across every mapped field, including out-of-range values at both ends.

### 3. Split `zigbee.c`

984 lines doing five jobs. Split along the seams already present:

| file | contents |
|---|---|
| `zigbee.c` | init, endpoint/cluster registration, join/steer signal handling |
| `zigbee_attrs.c` | custom cluster `0xFC00` dispatch (`attr_cb`, ~157 lines) + the `ATTR_*→TUNABLE_*` map from §2 |
| `zigbee_telem.c` | telemetry task, temperature reporting, the four `configure_reporting_*` |
| `zigbee_diag.c` | attribute-write tallies, rebuilt on `diag_ring` |

Statics shared across the split move to an internal `zigbee_internal.h`, not the public `zigbee.h`.

This is the split that pays: the display/AUX board adds two on/off switch endpoints, and its per-channel
`input_mode` config lands in the custom-cluster dispatch.

### 4. Thin `sensors_hw.c`

363 → ~250 lines. The ring moves to `diag_ring`; the roll-on-boot comes out of `sensors_start()`, where
it is currently interleaved with mutex creation and task startup (`226-256`). No change to the 1-Wire
driver, EMA filter, or fault classification.

### 5. Housekeeping

Delete the 26 untracked `firmware/test_host/build-*/` directories (304 MB; each carries its own fetched
Unity copy). Add `.gitignore` entries for `firmware/test_host/build*/` and `firmware/.cache/`.

### Seams for display/AUX — documented, not implemented

No new peripheral files are created in 1.7.0. The refactor leaves these attachment points, recorded here
so the next spec can rely on them:

- **Peripheral tasks** follow the existing `*_hw.c` pattern (`sensors_hw`, `valve_hw`, `ui`): own task,
  own heartbeat id. `display_hw.c` and `aux_hw.c` attach the same way.
- **`hb_id_t`** gains `HB_DISPLAY` / `HB_AUX` by adding enum entries. The heartbeat payload is
  `last_ms[HB_COUNT]`, so it widens automatically; `diag_ring` itself is payload-agnostic and needs no
  change.
- **Portable display logic** — page state machine, scroll/render, digit font — belongs in `ctrl_core`
  so the host suite can test it, as its spec requires. Only I2C transport lives in `main/display_hw.c`.
- **Endpoint registration** for the two on/off channels goes in the post-split `zigbee.c`; their command
  handling in `zigbee_attrs.c`.

### Verification

- Host suite green and grown: 17 binaries today, plus the `diag_ring` and unified-clamp tests. Full
  clean cycle is ~9 s, so it runs on every step.
- `idf.py size` diffed before/after. Expect roughly break-even; a large swing means something was
  dropped or duplicated by mistake.
- Flash over USB, then **soak on mains with USB unplugged past the 40 s mark** — the 1.6.4 fault only
  appears with no host attached, and attaching USB both resets the board and masks it.
- Confirm `fault_bitmap: 0`, temperatures flowing, and the plant regulating in Z2M.

## Release 1.7.1 — sdkconfig only

The build is currently `CONFIG_COMPILER_OPTIMIZATION_LEVEL_DEBUG=y` (`-Og`). Set
`CONFIG_COMPILER_OPTIMIZATION_SIZE=y`. Nothing else changes in this release, so any misbehaviour on the
plant is unambiguously attributable.

**Assertions stay enabled** (`CONFIG_COMPILER_OPTIMIZATION_ASSERTIONS_ENABLE=y`, level 2) and
`CONFIG_ESP_ERR_TO_NAME_LOOKUP` stays. Both are diagnostics on a device that was recently debugged
blind; trading them for bytes buys headroom already in surplus.

Expected saving is 10–20% of the image, unmeasured — no Release-vs-Debug baseline exists yet. Measure,
record in `firmware/README.md`, and keep the change only if the plant soaks clean.

Same verification as 1.7.0, plus `idf.py size` before/after recorded in the release notes.

## Out of scope, and why

- **No `constants.h`.** Timing and threshold `#define`s currently sit at the top of the file that uses
  them (`SWEEP_PERIOD_MS`, `CYCLE_MS`, `TICK_MS`, `EMA_TAU_S`, `SELF_HEAL_*`, `STEER_RETRY_*`,
  `TELEMETRY_PERIOD_MS`, DS18B20 command bytes). Centralising them creates a header the whole tree
  depends on and rebuilds on every tuning tweak. Scattered-but-local beats central-and-coupled here.
- **Test harness untouched.** 17 executables each relinking the whole of `ctrl_core` is wasteful in
  principle and costs 9 s in practice. Not worth the churn.
- **No cuts to console/argtable3, Green Power, or a switch to `ZB_ZED`.** These are the largest
  remaining size levers after the optimisation flip, and every one of them degrades observability or
  capability to buy space that is already free.
- **No NVS schema change.** The `input_mode` fields the display/AUX board needs will force a v4
  migration; that belongs in that feature's own release, not bundled here.

## Risk

Everything in 1.7.0 is mechanical movement rather than logic change, **except the config unification**,
which genuinely replaces two implementations with one — hence the dedicated equivalence test.

The exposure is that this touches the Zigbee path Z2M talks to, on a plant that only came back into
service on 2026-08-10. Mitigation is the soak in Verification, performed the hard way (no USB), because
this codebase has already demonstrated a fault that is invisible whenever it is observable.
