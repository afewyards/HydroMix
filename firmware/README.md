# ValveController Firmware

ESP-IDF firmware for the ValveController PCB (ESP32-C6) — a hydronic 3-way mixing
valve controller regulating supply temperature, exposed over Zigbee (Router role)
with OTA, autonomous when the Zigbee link is down.

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
| Status LED | GPIO15 | **active-low** (drive LOW = lit) |
| Console / flash / JTAG | GPIO12/13 | native USB-Serial-JTAG (USB-C), no separate config needed |

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

## Zigbee join / factory-reset gestures and LED legend

- **Short button press** (GPIO9) → network steering (join).
- **Hold ≥5 s** → Zigbee leave + factory reset (NVS erased, defaults reload).

Status LED (GPIO15, active-low) pattern priority, highest first:

1. **Identify** — fast single blink (60 ms on / 60 ms off), repeating.
2. **Alarm or sensor fault** — triple blink (80 ms on / 120 ms off ×3), then a
   600 ms pause.
3. **Not joined** (steering) — single blink (100 ms on / 100 ms off), repeating.
4. **Mode** (once joined, no alarm/fault):
   - HEATING — one 150 ms flash, then ~2.5 s off.
   - COOLING — two 150 ms flashes (200 ms gap), then ~2.5 s off.
   - IDLE — one short 50 ms flash, then ~5 s off.

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
