# ValveController Firmware

ESP-IDF firmware for the ValveController PCB (ESP32-C6) — a hydronic 3-way mixing
valve controller regulating supply temperature, exposed over Zigbee (Router role)
with OTA, autonomous when the Zigbee link is down.

## 1.6.0 — resync can now target the source end, gated by comfort

Diagnosed from 19 h of live GF-HydroMix telemetry (16:29Z–01:47Z, 2026-08-05): in the
current cooling regime the valve regulates at 85–100 %, so every position resync —
previously always a drive to the 0 %/recirculation end-stop — was a ~200 % round trip
with ~3 min of forced recirc and a ~2 K supply excursion. Three such events tripped
in that window, preferentially right after source cold slugs, i.e. exactly when the
plant was already disturbed. "Valve goes to 0 for no reason" was the live symptom.

- **A resync may now target the 100 %-source end-stop** when the valve is already near
  it and a comfort gate passes, instead of unconditionally driving to recirc. New pure
  module `ctrl_core/resync_policy.c` decides the end each cycle:
  - position < `RESYNC_NEAR_END_PCT` (50 %) → recirc-end immediately (cheap; the mix
    is mostly recirc already).
  - position ≥ 50 % and the gate passes → source-end.
  - position ≥ 50 % and the gate fails → deferred, re-checked every control cycle:
    source-end the instant the gate passes, or recirc-end after
    `RESYNC_DEFER_MAX_MS` (30 min) — whichever comes first. Position dropping below
    50 % while deferring also forces recirc-end immediately, since it just got cheap.
  - The accumulated-travel / reversal counters that trigger a resync keep accumulating
    through deferral; they only reset on resync completion, unchanged.
- **The comfort gate** (`resync_gate_eval()`, computed once per cycle in `control_step`
  and published to `valve_hw` via `valve_note_resync_gate()`) requires: mode is HEATING
  or COOLING, the source probe is unfaulted, `|t_src − setpoint| ≤ RESYNC_SRC_GATE_K`
  (2.0 K), and `t_src` sits inside the governor band with a `RESYNC_GATE_GOV_MARGIN_K`
  (1.0 K) margin on both sides. A dew-guard-raised setpoint tightens the gate along
  with it automatically. All four thresholds are compile-time constants — no new
  Zigbee tunables.
- **Mid-stroke abort:** resync ignores regulation targets while driving, so a
  source-end stroke that starts at the edge of a cold slug can carry `t_src` below the
  governor band with nothing to stop it. If the source leaves the raw governor band
  during a source-end stroke, the stroke aborts and restarts as a fresh full
  recirc-end stroke — its stall deadline extended by the interlock's reversal
  blackout (`INTERLOCK_MIN_PULSE_MS + INTERLOCK_DEAD_TIME_MS +
  INTERLOCK_ANTI_DITHER_MS`) so the deadline accounts for the pulse-finish,
  dead-time and anti-dither the interlock imposes before the reversed drive actually
  starts. Recirc-end strokes never abort. Worst case equals pre-1.6.0 behavior; never
  worse.
- **Boot resyncs and manual resyncs (console `resync` / Z2M switch) stay forced
  recirc-end**, unconditionally — sensors boot latched-faulted, and bench use wants a
  predictable end-stop. A manual resync requested while an autonomous source-end
  stroke is already driving is **latched, not dropped**: `valve_task` only consumes
  the pending manual-resync flag once the drive returns to idle, so it fires as a
  forced recirc-end stroke the moment the in-flight autonomous stroke completes.
- Stroke duration stays `travel_time_s × RESYNC_STALL_MULT` in both directions (plus
  the abort extension above) — drift is why a resync happens at all, so no
  distance-based shortening. `pos_est_resync_done()` now takes the end-stop seed (0.0
  recirc / 100.0 source) instead of always seeding 0.
- This deliberately supersedes the 2026-07-13 "resync only toward the
  0 %/recirculation end-stop" rule and the `NEVER toward source` comment it left in
  `valve_hw.c`. The original rationale — 100 % source is the dangerous end — now
  holds via the gate, the governor-band bound and the mid-stroke abort, not a blanket
  prohibition.
- Not yet Zigbee-exposed: resync events, the chosen end, and gate state publish
  nowhere — pairs with the still-open `ff.frozen`/authority-floor telemetry gap from
  1.5.1, next release.

New constants (`components/ctrl_core/include/ctrl_core/resync_policy.h`):
`RESYNC_SRC_GATE_K` 2.0 K, `RESYNC_GATE_GOV_MARGIN_K` 1.0 K, `RESYNC_DEFER_MAX_MS`
1 800 000 (30 min), `RESYNC_NEAR_END_PCT` 50.0 %.

Design spec: `docs/superpowers/specs/2026-08-05-gated-bidirectional-resync-design.md`.

Suite 17/17, 119 tests. Build clean, 63 % OTA headroom, embedded app version verified
1.6.0.

## 1.5.1 — the authority floor had the wrong sign, and the freeze latched the valve

**1.5.0 broke cooling on the live plant.** On 2026-08-04 the valve sat at 55.7 % for 92
minutes while supply ran 2.6–2.75 K above an 18.5 °C setpoint and the house stopped being
cooled. Root-caused from 869 Z2M samples (09:00–10:33).

### What went wrong

Three things compounded:

1. **A freeze disconnected the whole controller, not just the integrator.** `pi.c`'s
   `if (freeze) return ctrl_clampf(pos_ff, …)` discards **both** P and I, and `control.c`
   passed `freeze_pi || ff.frozen`. So a low-authority FF handed the valve a bare frozen
   constant and ran open-loop — while `t_supply`, a perfectly valid measurement, was ignored.
2. **The floor was placed inside the plant's normal operating range.** At the live 18.5 °C
   setpoint the derived floor was `sqrt(2.7·100·0.065)` = 4.19, clamped to the 4.0 K ceiling.
   Measured `|t_src − t_ret|` runs **0.07–4.75 K, mean 1.89** → **87 % of samples treated as
   no-authority.** The valve moved on exactly 5 occasions in 92 min, *all* of them inside the
   one window where `|denom|` crossed 4.00.
3. **The escape hatch was a no-op.** `if (ff.park_requested) target = in->valve_pos;` commands
   exactly what the freeze already commands. 1.5.0 correctly identified that the old open-loop
   park to `park_pos` was harmful, but deleted the only thing that broke the latch.

And the latch is self-reinforcing: a pinned valve starves the source branch, a starved branch
stops being cooled, `t_src` drifts toward `t_ret`, `|denom|` shrinks further. It reached
**0.07 K**. There was no exit.

### The coupling sign was inverted

`FF_COUPLING_PCT_K = +0.0654 K/%` came from a 20 h **closed-loop** fit at only r = +0.49,
taken while the HX primary itself drifted 14.25 → 16.93 °C. The controller opens the valve
*because* the primary warmed, so valve % and `t_source` both track `hx_a` and correlate
positively with no causal content — a confound.

The open-loop measurement (valve held manually at full source, primary flat to within
0.19 K) gives the **opposite** sign: `t_src` fell 20.87 → 19.06 °C over ~44 % of travel,
about **−0.041 K/%**. More source flow *cools* the source; flow starvation dominates the
textbook rise in HX approach.

That inverts the derivation, not just its magnitude. `ff_authority_floor()` computes a loop
gain **magnitude** and treats |G| > 1 as runaway — but sign decides whether feedback
diverges. With the true negative coupling, opening the valve cools the source, *grows*
`|denom|` and backs the valve off: negative feedback, self-limiting. **The floor guarded a
runaway that cannot happen in this plant, while manufacturing a real latch.**

### The fix

- **`ff_step` now LIMITS the denominator instead of abandoning the division.** Output stays
  live, bounded, continuous and correctly signed, and — because the coupling is negative — it
  points the valve the way that *restores* authority. Saturating toward the source is the
  correct response to a converging source, not an error to suppress. A source on the wrong
  side of `t_ret` still clamps to 0; `denom == 0` falls back to the demand direction.
- **`ff.frozen` no longer freezes the PI.** Only `resync_active` does, where the valve is
  being driven to an end stop and the output is moot anyway.
- **`FF_COUPLING_PCT_K` → 0**, so the floor is a fixed 2 K numerical guard. Do *not* re-enable
  it from another closed-loop fit — it needs an open-loop step test (park the valve at 2–3
  positions ≥ 20 min each with the controller out of the loop).
- **`FF_OUT_TAU_S` 900 → 180 s.** 900 s was ~7.5× the valve travel time. The travel reduction
  it was credited with came mostly from the freeze pinning the valve: at 180 s the 1.5.0 sim
  is still frozen 100 % of the time with zero travel.
- **`park_requested` / `FF_NO_AUTHORITY_PARK_DWELL_MS` removed**, along with `last_valid` /
  `freezing` / `frozen_since_ms` — all dead once the denominator is limited. `water_running`
  OFF, `MODE_IDLE` and `CTRL_PARK` parks are untouched; those are real safety parks.

### The test that blessed the bug

`test_hx_feedback.c`'s headline gate was `new.travel < 0.50 × old.travel` — *"the valve moved
less."* Re-run on its own plant at the shipped coupling, 1.5.0 gives **travel 0.0 %, frozen
100 %, pinned at 48.3 %, supply 1.0 K warm — and it PASSED.** Its supply guards passed only
because the baseline it compared against was itself railed at 98.3 % with +2.65 K error.

**A latched valve trivially wins a travel-reduction gate.** Travel is now bounded on *both*
sides and tracking error is capped absolutely, not relative to a broken baseline. The plant
also carries the measured negative sign, and `test_ff_recovery.c` is new: it starts in the
observed starved state and fails on 1.5.0 (49.2 %, travel 0, +2.33 K) while 1.5.1 reaches
95.8 % — the analytic optimum is 95.7 % — at −0.01 K.

Suite 16/16, 105 tests. Build clean, 63 % OTA headroom, embedded app version verified 1.5.1.
Flashed over USB 2026-08-04. **Still open:** `ff.frozen` and `authority_floor_k` are published
nowhere, which is why 87 % frozen was invisible in HA and a human had to notice the house was
warm. Exposing them needs a Zigbee attribute plus a converter deploy — next release.

## 1.5.0 — feedforward vs. the HX approach coupling; no more open-loop parks

Diagnosed from 20 h of live GF-HydroMix cooling telemetry (2026-08-03), after the valve
began hunting once the cooling setpoint moved from 20 °C toward 19 °C.

> **Version note.** The 2026-08-03 bench build shipped everything below but still
> self-identified as **1.4.0**, because `version.txt` was never bumped — a different
> failure from the 1.3.x `PROJECT_VER` CMake-cache trap, with the same symptom. When you
> need to know which source a flashed image really is, check for a version-unique symbol
> (`nm build/valvecontroller.elf | grep ff_authority_floor`) rather than trusting the
> reported version, `version.txt`, or Z2M's `installed_version`.

### The 60 s park was worse than the freeze it escalated (2026-08-04)

Live follow-up on the build above: the valve kept dropping to ~22 % for ~90 s at a time.
Not a spike and not a reporting artefact — a *commanded* move, every step exactly
8.3 %/10 s (full slew at `travel_time_s 120`), bottoming just short of `park_pos 20`
because `valve_deadband_pct 2` stops the motor within 2 % of target.

- **Cause: the derived floor made the freeze reachable, and the freeze escalated to a
  park.** The floor tracks the setpoint, so moving from 20 °C to 19 °C raised it from
  2.79 K to **3.51 K** (at `t_ret ≈ 20.9`), while the plant's live `|t_src − t_ret|` runs
  3.31…6.75 K — it dips under 3.51 K but never under the old fixed 2.0 K. Replaying the
  freeze predicate over 3 h 13 m of telemetry: **exactly two freezes ≥ the 60 s dwell,
  exactly two observed parks, zero false positives in 2316 samples.** So the fix for
  hunting at 19 °C is what introduced parking at 19 °C.
- **`park_requested` now holds position instead of driving `park_pos`, and no longer
  resets the PI.** Parking is open-loop, and in cooling it *raises* supply temp (+0.7 K
  measured per event) — it manufactures the disturbance the loop then has to undo. A
  freeze is already stable on its own: the FF holds `last_valid`. Recovery from a hold
  resumes on the integrator it left, rather than rebuilding from zero.
- **`FF_NO_AUTHORITY_PARK_DWELL_MS` 60 s → 600 s**, demoted to a backstop for a genuinely
  stuck plant now that the sustained case no longer parks.
- The `water_running` OFF, `MODE_IDLE`, and `CTRL_PARK` degradation parks are unchanged —
  those are real safety parks. The governor still applies after the hold.

### Investigated and REJECTED: splitting the PI deadband off the integral path

Supply sits at **+0.39 K** against a 19 °C setpoint (user-visible as "19.5 instead of
19"); the parks above account for only +0.025 K of it. `pi_step()` zeroes `e_eff` inside
±`deadband_k` and uses it for **both** the proportional and integral paths, so on paper
every point in `[setpoint − deadband_k, setpoint + deadband_k]` is an equilibrium with a
frozen integrator — a textbook permanent offset. The live minimum over a 3 h capture was
exactly **18.75** = 19 − 0.25, sitting precisely on the deadband edge, with 52 % of
samples inside the band. That looked conclusive.

**It did not survive simulation, and the change was reverted.** `test_lagsim`'s FOPDT
loop, 6 h per run, comparing the shipped code against a variant whose integrator runs on
true error, plus two intermediate variants (integral deadband at 0.4× and 0.6× the
proportional one):

- At **4 of the 5 corners the offset is identical to three decimal places** across every
  variant and every `ki` in 0.9…0.2. The residual offsets there (+0.17, −0.50, +0.17,
  −0.05) are plant and valve-quantisation artifacts the integral path cannot influence.
- At **θ=40 / τ=60 removing the integral deadband is materially worse**: the loop settles
  at **−0.94 K** where the shipped code sits at **+0.06 K**. That is integral windup
  through the 40 s deadtime — the deadband is acting as a brake against it, not merely
  buying the offset. This is the corner closest to the real GF-HydroMix plant.
- It also breaks the existing `new_pp < 0.8` gate at the shipped `ki=0.9` (1.21 K) and
  would have needed a `ki` retune to 0.5 purely to compensate.

Why the live signature misleads: in the lagsim's geometry (`T_SRC` 45 / `T_RET` 27, an
18 K span) 2 % of valve travel is worth ~0.54 K of supply, which swamps the 0.25 K
deadband. On the real plant the span is only ~4.8 K, so 2 % of travel is ~0.13 K and the
deadband edge becomes visible in the data — but visible is not the same as binding, and
no variant of relaxing it actually moved the number.

**If the offset is worth chasing, do it with the existing `pi_deadband_k` tunable
(custom-cluster 0x000F, live-writable over Zigbee) as a reversible experiment on the real
plant — not with a structural change to the controller.** Note the 1.4.0 five-corner
sweep already found ripple *non-monotonic* in `pi_deadband_k` between 0.15 and 0.25
(limit-cycle regime hopping), so do not bisect it; test discrete values and watch.

- **The FF was driving its own denominator.** `ff_step()` inverts a mixing law that treats
  `t_source` as an independent input. On a plate HX fed by a fixed primary it is not:
  opening the mixing valve raises *secondary* flow, collapses the HX approach, and warms
  `t_source` — the quantity `pos_ff = (t_set − t_ret)/(t_src − t_ret)` divides by.
  Regressed live: **d(t_source)/d(valve %) = +0.0654 K/%** (r = +0.49), approach sweeping
  −0.57…+6.25 K while the primary side (`hx_a`) stayed inside 14.25…16.93 °C. Net effect:
  `t_source` swung **7.81 K p2p, 3× its own primary**, and the valve swung 98.5 % p2p.
  Supply itself was never the problem (sd 0.37 K) — valve travel was.
- **Authority floor is now derived from the setpoint** instead of the fixed
  `FF_MIN_AUTHORITY_K 2.0`. Chaining the coupling through the mixing law gives loop gain
  `G = |t_ret − t_set| · 100 · coupling / denom²`, so `G = 1` at
  `denom = sqrt(|t_ret − t_set| · 100 · coupling)`. That tracks the setpoint, which is
  exactly what a constant gets wrong: at `t_ret ≈ 21.2` the floor is **2.80 K at a 20 °C
  setpoint but 3.79 K at 19 °C**. Clamped to `[FF_AUTHORITY_MIN_K 2.0,
  FF_AUTHORITY_MAX_K 4.0]` — past ~4 K the freeze stops being free (measured share of
  flowing time spent frozen: 2.0 K → 3.0 %, 2.8 K → 3.6 %, 3.8 K → 6.4 %, but
  4.6 K → 17.7 % in runs long enough for the 60 s park dwell to dominate).
- **The FF output carries a 900 s EMA** (`FF_OUT_TAU_S`), applied to `pos_ff`, *not* to
  `t_source`/`t_return`. The mixing law divides by `(t_src − t_ret)`, so filtering the
  inputs and then dividing does not land where dividing and then filtering does — and
  with the denominator swinging several K that gap is not second-order. Filtering after
  the division also leaves the low-authority test on the live reading, so a collapse in
  authority is still caught the instant it happens. This is the *secondary* fix: the
  floor is what keeps the FF out of the region where it fights itself; the EMA only stops
  the motor chasing the ripple that remains.
- **A zeroed `ff_cfg_t` degrades to the pre-1.5.0 fixed 2 K floor, not to no floor.**
  `control_cfg_t` is brace-initialised in several places (including `test_control.c`), and
  a zero `auth_max_k` would otherwise disable the low-authority freeze altogether.
- `ff_reseed()` on a resync falling edge drops the output filter but keeps `last_valid`;
  `ff_mode_change()` drops both, since heating and cooling put `pos_ff` on opposite sides
  of `t_ret`.
- Not Zigbee tunables. They live in `ff_cfg_t` (filled by `ff_cfg_defaults()` in
  `control_task.c`) purely so the host tests can drive old and new behaviour through one
  code path — and so promoting any of them to a custom-cluster attribute later is a small
  diff. **No NVS bump**, so no downgrade hazard this time.

`test_hx_feedback.c` is the new closed-loop regression: the plant reproduces the measured
coupling, and the coefficient is swept {0.04, 0.0654, 0.09} because 0.0654 is a
closed-loop estimate at only r = 0.49. Across that sweep, valve travel falls to
**0.28–0.39×** and source p2p to **0.40–0.51×**, while supply p2p and mean offset improve
at the two stiffer couplings. `test_lagsim.c` is unaffected by construction — its source,
return and setpoint are constant, so the EMA reseeds on the first step and then holds.

## 1.4.0 — valve deadband tunable + RunningMode push

- **Motor stop deadband is now a runtime Zigbee tunable** (`0x0012 valve_deadband_pct`,
  float %, custom cluster `0xFC00`, same read/write/NVS-persist machinery as the other
  tunables). Was a compile-time `#define DEADBAND_PCT 2.0f` in `valve_hw.c`.
  **BEHAVIOR CHANGE:** the shipped default is now 1.0 % *at `travel_time_s` ≥ 120 s*
  (was effectively 2.0 % on every device before this release, regardless of
  `travel_time_s`) — halved because at `kp=2.8 %/K` that's ≈±0.36 K of supply error
  before the motor reacts, and the existing 30 s transit hold + 0.25 K supply-move
  release already suppress oscillation at the tighter band. Below `travel_time_s=120`
  this is NOT a straight halving: `clamp_config()` raises the migrated default up to
  the travel-derived floor (below), so a device already tuned to `travel_time_s=60`
  lands at 2.0 % (unchanged) and one at `travel_time_s=30` lands at 4.0 % (doubled).
  Clamped to `[max(0.2, floor), 5.0]`, where `floor` is derived from the interlock's
  minimum drive pulse and the current `travel_time_s` (`1.2 × (INTERLOCK_MIN_PULSE_MS /
  1000) × 100 / (2 × travel_time_s)` — ≈1.0 % at 120 s, 2.0 % at 60 s, 0.2 % at 600 s):
  a constant floor below this lets a short travel time turn every drive pulse into a
  full band crossing, limit-cycling the motor. Writing `travel_time_s` re-clamps
  `valve_deadband_pct` against the new floor as a side effect (shortening travel can
  raise the floor above an already-set deadband) — both the Zigbee attribute store and
  NVS reflect the adjusted value.
- **`pi_deadband_k` (`0x000F`) stays at its 0.25 K default.** A lower value (0.15) was
  considered alongside the valve-deadband work above, but a closed-loop sweep of
  {0.15, 0.20, 0.22, 0.25} against `test_lagsim.c`'s dead-time/lag robustness matrix
  found only 0.25 clears every corner's convergence/ripple gate — 0.25 ships unchanged.
- **NVS config migrates v2 → v3** (`CONFIG_VERSION` 2 → 3) to add `valve_deadband_pct`
  without wiping existing tunables: a 1.3.x-sized blob is read field-by-field, the new
  field defaults to 1.0, and the blob is re-saved at the new size/version. **Downgrading
  to ≤1.3.1 after this release wipes the config** — the old firmware's loader only
  recognizes its own (smaller) blob size and falls back to defaults for anything else.
- **Thermostat `RunningMode` (`0x001E`) now pushes an explicit unsolicited report** on
  every change, and once on every (re)join. This is a belt-and-suspenders addition, NOT
  the root-cause fix for the 6 h-stale `mode` sensor observed live: the actual cause was
  that EP1's `hvacThermostat` cluster was never bound in the first place (live
  binding-table inspection showed EP1 bound for `genOnOff`/`genAnalogOutput` only), so
  neither the passive ZCL reporting-engine config *nor* this new explicit push has
  anywhere to send a report without a binding. The root-cause fix is converter-side —
  `z2m/valvectl.mjs`'s `configure()` now binds `hvacThermostat` — and this firmware push
  only helps once that binding exists.

## 1.3.1 — Zigbee-writable setpoints

- **heat_setpoint / cool_setpoint moved to the custom cluster** (attrs `0x0010`/`0x0011`,
  float °C, clamped 17–35, NaN-rejected, echoed back, NVS-persisted — same machinery as
  the other tunables). The standard Thermostat cluster rejects every setpoint write on
  this device with `INVALID_VALUE`: ZBOSS enforces the ZCL single-zone invariant
  `heat ≤ cool − deadband`, which the device's independent seasonal targets (heat 35 /
  cool 18) deliberately violate. The thermostat cluster's `OccupiedHeating/CoolingSetpoint`
  are effectively static boot defaults: the firmware attempts a mirror write on every
  custom-cluster setpoint change, but ZBOSS vetoes local thermostat setpoint stores under
  the same deadband rule (verified on-air 2026-07-31, `check=false` notwithstanding), so
  reads of the standard attributes do NOT reflect the live targets — use the custom
  attrs. `z2m/valvectl.mjs` exposes both as `heat_setpoint` / `cool_setpoint` numbers
  and no longer attempts thermostat-cluster writes.

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

## Prerequisites

- **ESP-IDF v5.5.4**, installed at `~/esp/esp-idf` on this machine. Activate it in
  every new shell before running `idf.py`:
  ```
  . ~/esp/esp-idf/export.sh
  ```
- Target chip: **esp32c6**. If starting from a clean checkout (or after switching
  IDF versions), set the target once:
  ```
  cd firmware
  idf.py set-target esp32c6
  ```
- USB-C cable to the board's native USB-Serial-JTAG port (GPIO12/13) — this is
  used for flashing, the console, and JTAG. **No separate USB-to-serial adapter
  needed.**

## Build / flash / monitor

```
cd firmware
idf.py build
idf.py -p <PORT> flash monitor
```

The first `idf.py build` (or any build after an `idf_component.yml` change)
fetches managed components (`espressif/onewire_bus`, `espressif/esp-zigbee-lib`,
`espressif/esp-zboss-lib`) — this can take a few minutes and needs network
access. Resolved versions are pinned in `firmware/dependencies.lock` (currently
esp-zigbee-lib 1.6.8, esp-zboss-lib 1.6.4, onewire_bus 1.1.1).

Exit the serial monitor with `Ctrl+]`.

## Partition table and OTA

`firmware/partitions.csv` (4 MB flash, `CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y`):

| Name | Type | SubType | Offset | Size |
|---|---|---|---|---|
| `nvs` | data | nvs | 0x9000 | 0x6000 |
| `otadata` | data | ota | 0xf000 | 0x2000 |
| `phy_init` | data | phy | 0x11000 | 0x1000 |
| `ota_0` | app | ota_0 | 0x20000 | 0x1D0000 (~1.81 MB) |
| `ota_1` | app | ota_1 | 0x1F0000 | 0x1D0000 (~1.81 MB) |
| `zb_storage` | data | fat | 0x3C0000 | 0x38000 |
| `zb_fct` | data | fat | 0x3F8000 | 0x1000 |

(Note: the OTA-data partition is named `otadata`, not `ota_data`.)

Two-slot OTA with `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y`. The actual gating
logic lives in `firmware/main/ota.c`:

- On boot, `ota_init()` checks whether the running image is still
  `ESP_OTA_IMG_PENDING_VERIFY` (i.e. it was just flashed via OTA and hasn't been
  validated yet).
- The image is marked valid — cancelling any pending rollback — once
  `ota_note_joined()` (Zigbee rejoined, called from `zigbee_on_join()` in
  `app_main.c`) **and** `ota_note_good_sweep()` have both fired.
  `ota_note_good_sweep()` is called from `control_task.c` on either of:
  1. a control cycle completing with no sensor faults, checked every cycle
     (fast path), or
  2. `OTA_GATE_CYCLES` (12) completed control cycles regardless of sensor faults —
     sensor faults are a plant-wiring property, not an image property. 12 cycles at
     the 10 s control period is ~110 s, deliberately **longer** than the 30 s task
     watchdog timeout (`CONFIG_ESP_TASK_WDT_TIMEOUT_S=30`, panic on): a control task
     that hangs trips the watchdog and reboots — rolling back — *before* this gate
     could ever validate the image. (1.2.1 used 3 cycles ≈ 20 s, which fired inside
     the watchdog window and so proved nothing.)

  As a last resort, a bounded 10-minute fallback timer (started at rejoin)
  validates unconditionally if neither path above has fired yet.

  If the new image never reaches validation by any of these paths, the
  bootloader rolls back to the previous image on the next reset.

**Publishing an OTA update via Zigbee2MQTT:** there is no cloud OTA index entry
for this private manufacturer code. The process (documented in comments at the
top of `z2m/valvectl.mjs`) is: build `firmware/build/valvecontroller.bin` →
wrap it into a Zigbee OTA image (`.ota`) with the ZCL OTA header (manufacturer
code, image type, file version) → host the `.ota` file somewhere Z2M can read
it → add an entry (`modelId`, `url`, `fileVersion`) to Z2M's local OTA index
config.

## Boot-safety invariant

`triacs_safe_low()` is the **literal first statement** in `app_main()`
(`firmware/main/app_main.c`) — before NVS, before anything else. It configures
GPIO2 and GPIO3 as outputs with pull-downs enabled and immediately drives both
low. `valve_hw.c` is the **sole owner** of GPIO2/3 afterwards, and the
`ctrl_core` interlock (`interlock_step()`) guarantees the two are never driven
high simultaneously. On reset/panic/brownout/watchdog, GPIO2/3 float low
(external pull-downs + MOC3063 input threshold hold the triacs off) until
`app_main` re-establishes the safe-low state, and every boot runs a resync
toward the recirculation end.

## Hardware / GPIO map

| Function | GPIO | Notes |
|---|---|---|
| Valve open triac | GPIO2 | active-high, output-low first, owned by `valve_hw.c` |
| Valve close triac | GPIO3 | active-high, output-low first, owned by `valve_hw.c` |
| TEMP_SUPPLY | GPIO0 | DS18B20, unfiltered (PI input) |
| TEMP_RETURN | GPIO1 | DS18B20, EMA-filtered (τ≈40 s) |
| TEMP_SOURCE | GPIO10 | DS18B20, EMA-filtered (τ≈40 s) |
| TEMP_HX_A | GPIO18 | DS18B20, mode detection |
| TEMP_HX_B | GPIO19 | DS18B20, monitoring only |
| Button | GPIO9 | active-low, pull-up; short = Zigbee steering, hold ≥5 s = leave + factory reset |
| Console / flash / JTAG | GPIO12/13 | native USB-Serial-JTAG (USB-C), no separate config needed |
| *(spare)* | GPIO15 | Unconnected. Nominally the JTAG-source strap, but ignored while `JTAG_SEL_ENABLE` is unburned (the default), so the C6 always uses the USB Serial/JTAG controller. One of only two free non-strapping pins, along with GPIO16. **If you ever burn `JTAG_SEL_ENABLE`, this pin must be pulled high externally** — the C6 has no internal pull here, and a floating strap could divert JTAG to the MTDI/MTCK/MTMS/MTDO pads |

## Display: driver-output → panel mapping

The IS31FL3730's outputs are **not** wired straight through to the KWM-20881AGB.
The panel's pinout interleaves rows and columns down both of its edges, so a
straight-through wiring produced an unroutable ratsnest (40 crossings). The
outputs were permuted to make the fan-out planar (6 crossings). Rows were only
ever swapped with rows and columns with columns — the driver sources on rows and
the panel is common-row-anode, so the two groups cannot be interchanged.

**The display driver must undo this in the framebuffer.** To light the LED at
panel (row *r*, column *c*), set the bit at driver output (R, C):

| Panel row | Driver | | Panel col | Driver |
|---|---|---|---|---|
| ROW1 | **R3** | | COL1 | **C3** |
| ROW2 | **R1** | | COL2 | **C6** |
| ROW3 | **R4** | | COL3 | **C7** |
| ROW4 | **R2** | | COL4 | **C1** |
| ROW5 | **R8** | | COL5 | **C8** |
| ROW6 | **R5** | | COL6 | **C2** |
| ROW7 | **R7** | | COL7 | **C4** |
| ROW8 | **R6** | | COL8 | **C5** |

Net names follow the **driver** side: `DISP_R3` is the driver's R3 output, which
lands on panel ROW8. Two constant lookup tables in the display driver are enough;
there is no runtime cost beyond the indirection.

## Sensor sweep (`firmware/main/sensors_hw.c`)

Pipelined 1-Wire sweep every **10 s** (`SWEEP_PERIOD_MS`), sharing the C6's
limited RMT TX/RX pairs across all 5 GPIOs by creating and deleting a bus per
GPIO per phase:

1. **Phase 1** — for each of the 5 GPIOs: create bus → Skip ROM + Convert T →
   delete bus (line released; DS18B20s are externally powered so conversion
   continues unattended).
2. One shared `750 ms` delay (`CONVERT_MS`) for conversion to finish.
3. **Phase 2** — for each GPIO: create bus → read scratchpad (with CRC-8 check
   via `onewire_crc8()` from the `onewire_bus` component's `onewire_crc.h`) →
   delete bus, up to **3 retries** (`MAX_RETRY`) on failure.
4. Sleep for the remainder of the 10 s period, then repeat.

A sensor is marked **faulted** after **3 consecutive** failed sweeps
(`FAULT_AFTER`). Source and return readings are EMA-filtered with τ≈40 s
(α ≈ 0.2 at a 10 s sample period); supply is left unfiltered for the PI loop.

## Console commands (USB-Serial-JTAG, prompt `valvectl>`)

| Command | Effect |
|---|---|
| `status` | Print supply/return/source/HX-A/HX-B readings and fault flags |
| `valve <0-100>` | Manually set the valve target position (percent) |
| `resync` | Force a valve position resync |
| `mode` | Print detected mode, alarm state, fault bitmap, and position |
| `factory-reset` | Zigbee leave + NVS wipe (defaults reload on next boot) |

## Zigbee join / factory-reset gestures

- **Short button press** (GPIO9) → network steering (join).
- **Hold ≥5 s** → Zigbee leave + factory reset (NVS erased, defaults reload).

**Status indication.** The single status LED was removed from the board once the
8×8 matrix landed; local state now belongs to the display. The matrix driver is
not written yet, so until it is, the USB-Serial-JTAG console is the only local
readout — `status` and `mode` report everything the LED used to encode.

`ui.c` still contains the old `led_task` driving GPIO15. It is inert (the LED is
gone) and harmless — the pin only sees the 10 k strap pull-up, and the strapping
latch is sampled at Chip Reset, so runtime toggling cannot affect JTAG selection.
It gets replaced wholesale by the display work.

## Zigbee endpoints

Router role. Manufacturer `Knife`, model `HydroMix`. Manufacturer code is
currently `VALVECTL_MFR_CODE 0x1234` — a **placeholder/test value**, not a
real Zigbee Alliance-assigned code (see the plan's "Unresolved Q1").

- **EP1**: Basic, Identify, On/Off (`water_running`), Thermostat (local temp =
  supply; setpoints clamped 17–35 °C; `SystemMode` writes accepted but
  ignored — mode is auto-detected from HX-A; `RunningMode` pushes an explicit
  unsolicited report on change and on every (re)join, since **1.4.0**), Analog
  Output (position 0–100 %, writable only while `water_running` is OFF), OTA
  client, plus a manufacturer-specific custom cluster `0xFC00` exposing 15
  read-write tunables — including, since **1.1.0**, `0x000E deadtime_s` (transit
  hold: seconds the PI loop pauses after the valve moves, 0–120, default 30) and
  `0x000F pi_deadband_k` (PI error deadband, K, 0–1, default 0.25) — and since
  **1.4.0**, `0x0012 valve_deadband_pct` (motor stop
  deadband, % of travel, default 1.0, clamped to `[max(0.2, floor), 5.0]` where
  `floor` tracks `travel_time_s` — see the 1.4.0 notes above) — a self-clearing
  `resync` bool, and read-only alarm-bitmap / fault-bitmap / travel-since-resync
  attributes. `ki` is also %/K per **minute** as of
  1.1.0 (was %/K per 10 s cycle in ≤1.0.7); a device upgrading from an older
  build migrates its stored config automatically on first boot (ki ×6,
  clamped to the new bounds — see `config_load()` in `firmware/main/config.c`).
  Alarm and fault bitmaps report **immediately** on change (no periodic cap);
  temperatures report at ±0.2 K or 60 s max; position reports at ±1 % or
  60 s max.
- **EP2–EP6**: Temperature Measurement (supply, return, source, HX-A, HX-B).

## Zigbee2MQTT integration

`z2m/valvectl.mjs` is the external converter. See the comments at the top of
that file for the manual OTA-image publishing process referenced above.

## Host-side control-logic tests

See `firmware/test_host/README.md` — the `ctrl_core` component (pure C, no IDF
includes) is unit-tested on macOS/Linux with plain CMake + Unity before any
hardware is touched.
