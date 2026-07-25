# ValveController Bench Validation Checklist

Self-contained, sequential bench procedure covering all 7 MANUAL CHECKPOINT
tasks from the firmware plan (plan Tasks 4, 15, 17, 20, 25, 27, 29). Work
through the checkpoints **in order** — each one gates the next. Ground truth
for every command, string, and behavior below is the as-built firmware code;
where this checklist and the original plan text differ, this checklist
reflects the code.

## 0. Build & flash

Do this once at the start of a bench session (and again any time you need a
fresh build — e.g. between checkpoints, or before Checkpoint 6's OTA test):

```
. ~/esp/esp-idf/export.sh      # activate the ESP-IDF v5.5.4 environment (once per shell)
cd firmware
idf.py build
```

Find the board's port, then flash + open the serial monitor:

```
ls /dev/cu.usbmodem*                          # confirm the device node, e.g. /dev/cu.usbmodem1101
idf.py -p /dev/cu.usbmodemXXXX flash monitor  # replace XXXX with your actual port
```

Exit the monitor with **Ctrl+]**.

Once Checkpoint 6 (OTA round-trip) has passed, **OTA becomes the normal
update path** — for routine firmware updates after that point you don't need
to re-flash over USB-C at all; push a new image through Zigbee2MQTT instead
(see Checkpoint 6).

## ⚠️ SAFETY — read before touching the board

- **USB-C power only for Checkpoints 1, 2, and 3.** Do **not** connect mains
  power at any point before Checkpoint 3 (triac verification) has fully
  passed.
- Checkpoint 3 is **the mains gate**. Its Steps 1–5 must all pass, with the
  valve mechanically free to dry-stroke (no water pressure forcing it), before
  Step 6 (first mains connection) — and before any later checkpoint that
  assumes mains/actuator.
- Checkpoints 4–6 can be run on USB-C power alone (Zigbee radio only needs
  3.3 V). Only Checkpoint 3 Step 6 onward and Checkpoint 7 need mains + the
  ARA661 actuator.
- GPIO2 (valve-open triac) and GPIO3 (valve-close triac) must **never** read
  high simultaneously, at any point, in any checkpoint. If you ever observe
  both high — stop, disconnect mains immediately, and do not proceed until
  the cause is understood.

## Pre-flight: confirm Task #33 fixes are merged

Task #33 (in progress at the time this checklist was written) fixes 5 gaps
in `firmware/main/{zigbee.c, control_task.c, ota.c, valve_hw.c, app_main.c}`
that this checklist assumes are present. **Before starting, confirm Task #33
is merged** — otherwise the checkpoints below will fail for reasons unrelated
to your hardware:

1. **Periodic telemetry push** — a ~10 s timer/task pushing
   `zigbee_report_temps()` + `zigbee_push_status()` (and the thermostat
   `local_temperature`) under the Zigbee lock. Without this, Zigbee
   temperature/position/status attributes never update after boot (they
   stay at their initialization values forever). With it, expect temps,
   position, and mode to visibly refresh in Z2M roughly every ~10 s
   (subject to the ±0.2 K / ±1 % / 60 s-max reporting thresholds in
   Checkpoint 5.3). Directly affects **Checkpoint 5**.
2. **`control_task_set_link()` wired to the Zigbee join/leave/steering
   signal** — without this, the link-up state is never set from `false`,
   so the 30-minute cooling-setpoint-raise guard behaves incorrectly.
   Affects **Checkpoint 7 Step 8**.
3. **`water_running=OFF` actually commands the valve to `park_pos`** —
   without this, turning `water_running` off stops regulation but leaves the
   valve wherever it last was, instead of parking. Affects **Checkpoint 5
   Step 4**.
4. **OTA mark-valid gate excludes HX-B (monitoring-only) faults and has a
   bounded fallback** — without this, a flaky/unplugged HX-B sensor can
   permanently block OTA validation (endless rollback loop). Affects
   **Checkpoint 6**.
5. **`direction_swap` latched at motion/resync start** — without this, a
   runtime write mid-stroke could invert direction unexpectedly. Relevant to
   **Checkpoint 3**.

If you must run this checklist before Task #33 lands, treat any failure in
the affected steps above as an expected/known gap, not a hardware problem.

---

## Checkpoint 1 — First flash, boot, LED, console (plan Task 4)

**Power: USB-C only. No mains.**

- [ ] **1.1** Connect the board via USB-C. Run `ls /dev/cu.usbmodem*` and
      confirm a device node appears (e.g. `/dev/cu.usbmodem1101`).
- [ ] **1.2** Flash and open the monitor:
      ```
      cd firmware
      idf.py -p /dev/cu.usbmodemXXXX flash monitor
      ```
      (replace `XXXX` with your actual port suffix; exit the monitor later
      with `Ctrl+]`).
- [ ] **1.3** In the boot log, confirm the line (tag `app`):
      `ValveController boot: triacs forced low`
      — this must be the very first thing logged, before NVS/config/sensor
      init messages.
- [ ] **1.4** With a multimeter, confirm GPIO2 and GPIO3 both read **≈0 V**
      at idle.
- [ ] **1.5** Watch the status LED (GPIO15, active-low): confirm the "idle"
      breathing-off pattern is running — a brief lit pulse roughly every few
      seconds, not steady-on or steady-off. (Once mode/join state settles,
      the exact pattern is one of those in the LED legend under Checkpoint
      4 — at this very early boot moment, expect the "not joined" single
      blink: 100 ms on / 100 ms off, repeating.) If the LED is dark or
      permanently lit, re-check the active-low wiring (drive LOW = lit).
- [ ] **1.6** At the `valvectl>` prompt, run each of:
      - `status` → prints a line like
        `supply=xx.xx ret=xx.xx src=xx.xx hxa=xx.xx hxb=xx.xx faults=00000`
        (fault digits will be `1` for any sensor not yet connected/settled).
      - `mode` → prints a line like `mode=IDLE alarm=0 faults=0x00 pos=50.0`.
      - `resync` → prints `resync requested` and triggers a real valve
        resync (harmless with the actuator unpowered/disconnected — GPIO2/3
        will pulse per Checkpoint 3's resync behavior, but with no mains
        nothing physically moves).
      None of these should crash or hang the console.

**Result:** [ ] PASS  [ ] FAIL — Notes: ______________________________________

---

## Checkpoint 2 — Sensors on the bench (plan Task 15)

**Power: USB-C only. No mains.**

Connector-to-sensor mapping (GPIO ↔ role):

| GPIO | Sensor |
|---|---|
| GPIO0 | SUPPLY |
| GPIO1 | RETURN |
| GPIO10 | SOURCE |
| GPIO18 | HX-A |
| GPIO19 | HX-B |

- [ ] **2.1** Plug in all 5 DS18B20s. Put SUPPLY/RETURN/SOURCE probes in one
      room-temperature water glass, and HX-A/HX-B in a second glass.
- [ ] **2.2** Flash + monitor (as in Checkpoint 1.2, or reuse the running
      session).
- [ ] **2.3** Run `status`. Confirm all 5 readings are plausible °C values,
      within ≈1 °C of each other within the same glass, and `faults=00000`.
- [ ] **2.4** Warm the SOURCE probe by hand (or warm water) and watch
      `status` over ~1–2 minutes: the **filtered** value used internally for
      feed-forward (not shown directly by `status`, which prints raw
      `value_c`) lags the raw jump with an EMA time constant τ≈40 s (a sweep
      runs every 10 s, so α≈0.2 per sample) — SUPPLY has no filtering and
      would track instantly if you warmed it instead.
- [ ] **2.5** Unplug HX-B. After 3 failed sweeps (~30 s, since a sweep runs
      every 10 s) confirm `status` shows the HX-B fault bit set (the 5th
      digit of `faults=` becomes `1`). Replug it and confirm the bit clears
      on the next good sweep.
- [ ] **2.6** Confirm sweep cadence: the whole sweep (convert-all → 750 ms
      wait → read-all with retries) takes well under 1 s of active bus time,
      and repeats every 10 s (`SWEEP_PERIOD_MS`).

**Result:** [ ] PASS  [ ] FAIL — Notes: ______________________________________

---

## Checkpoint 3 — Triac drive verification (NO MAINS), then dry-stroke (plan Task 17)

**🛑 THIS IS THE MAINS GATE. Do not connect mains until Steps 3.1–3.5 all
pass.** USB-C power only for those steps. Scope or LED probes on GPIO2
(SSR_OPEN) and GPIO3 (SSR_CLOSE).

- [ ] **3.1** Flash + monitor with probes/scope connected to GPIO2 and GPIO3.
- [ ] **3.2** At boot, confirm a resync runs: with the default
      `direction_swap=false`, GPIO3 (toward recirc) is asserted continuously
      for `travel_time_s × 1.15` — **≈138 s** at the default `travel_time_s`
      of 120 s — while GPIO2 stays low the entire time. **Confirm GPIO2 and
      GPIO3 are never both high at any instant.**
- [ ] **3.3** After resync completes, run `valve 80` — confirm only the
      toward-source line (GPIO2 with default `direction_swap`) pulses.
      Then run `valve 20` — confirm that after ≥500 ms dead time and a
      ≥2 s minimum pulse on the first move, the opposite line (GPIO3) drives.
      Confirm no reversal happens within 10 s of the previous move
      (anti-dither).
- [ ] **3.4** Run `valve 50`, then immediately `valve 51` — confirm the 2 %
      deadband suppresses any pulse (no line asserts for a sub-deadband
      delta).
- [ ] **3.5** Run `resync` — confirm only the toward-recirc line (GPIO3 by
      default) ever drives; the toward-source line (GPIO2) never asserts
      during a resync. Record scope traces/photos if possible.
- [ ] **3.6 (mains allowed only after 3.1–3.5 all pass):** Connect mains and
      the ARA661 actuator, with the valve **mechanically disconnected or
      free to dry-stroke**. Repeat `valve 0`, `valve 50`, `valve 100` and
      confirm the actuator strokes the expected direction each time.
      - If the direction is reversed: there is **no console command** to
        change `direction_swap` directly. Either (a) join Zigbee first
        (short button press — Zigbee only needs USB-C power, no mains
        required to join) and write the `direction_swap` custom-cluster
        attribute to `true` via Z2M/a Zigbee tool, or (b) temporarily edit
        the `DEFAULTS` struct in `firmware/main/config.c` and reflash. Then
        re-test.
      - Confirm self-lock: with both triacs off, the actuator holds its
        position (no drift).

**Result:** [ ] PASS  [ ] FAIL — Notes: ______________________________________

---

## Checkpoint 4 — End-to-end regulation on the bench (plan Task 20)

**Power: USB-C (mains optional, only if Checkpoint 3 Step 3.6 passed).**

- [ ] **4.1** Flash + monitor. `status` + `mode` confirm sensors read and
      the detected mode starts `IDLE` (boot mode is always IDLE until HX-A
      classifies), valve parked around 50 % (`park_pos` default).
- [ ] **4.2** Warm the HX-A probe above 28 °C (`heat_threshold`) for >60 s
      (`enter_dwell_ms`) → confirm `mode` becomes `HEATING`. Cool it below
      26 °C (28 − 2 K hysteresis) briefly, under 7 minutes
      (`leave_dwell_ms` = 420 s) → confirm mode does **not** leave yet.
      Hold below threshold for >7 min → confirm it transitions out
      (toward IDLE or COOLING depending on where HX-A settles).
- [ ] **4.3** In HEATING, warm SOURCE and adjust RETURN; confirm the
      commanded valve position tracks the feed-forward formula direction —
      a hotter source drives toward a **lower** percentage for the same
      setpoint (less hot source needed to hit target).
- [ ] **4.4** Force a governor event: warm SUPPLY above 36 °C (`gov_high`)
      → confirm the valve is driven to 0 % immediately, regardless of mode.
      Cool it back below 35 °C (the floor clamp) → confirm normal regulation
      resumes.
- [ ] **4.5** Unplug SUPPLY → after 3 failed sweeps confirm `faults` shows
      the supply bit and the strategy degrades to FF-only (cooling adds a
      conservative bias toward recirc; heating does not). Also unplug
      RETURN → confirm the strategy degrades further to PARK (10 % if in
      COOLING, else `park_pos`).
- [ ] **4.6** Hold SUPPLY above 36.5 °C continuously for 5 minutes
      (`alarm_dwell_ms` = 300000 ms) → confirm `alarm=1` in `mode` output
      appears only after the full dwell, not immediately (the governor
      should already be correcting well before the alarm fires — the alarm
      is telemetry-only and never blocks regulation).

**Result:** [ ] PASS  [ ] FAIL — Notes: ______________________________________

---

## Checkpoint 5 — Zigbee2MQTT join, entities, reporting, writes (plan Task 25)

**Requires a running Zigbee2MQTT coordinator.** USB-C power is sufficient.

- [ ] **5.1** Copy `z2m/valvectl.mjs` into Z2M's `external_converters`
      directory (or reference it from `configuration.yaml`); restart Z2M.
- [ ] **5.2** Flash the board (or power-cycle it). Short-press the button
      (GPIO9) → confirm Z2M pairs the device, showing manufacturer `Knife`
      and model `HydroMix`, and exposes all entities from
      `z2m/valvectl.mjs` (temps, `water_running`, `valve_position`, `mode`,
      the 10 tunables, `alarm`, `fault_bitmap`, `travel_since_resync`,
      `resync`).
- [ ] **5.3** Confirm all 5 temperature entities update: change a probe by
      ±0.2 K → report within seconds; otherwise within 60 s max. Confirm
      `valve_position` updates on a ±1 % change (also within 60 s max).
      *(Depends on the Task #33 periodic telemetry push — see Pre-flight.)*
- [ ] **5.4** Toggle `water_running` OFF → confirm regulation stops **and
      the valve physically parks at `park_pos`**. Write `valve_position`
      (the AnalogOutput attribute) while OFF → confirm it moves to the
      written value (manual override honored only while OFF). Toggle
      `water_running` back ON → confirm the manual write is now ignored and
      normal regulation resumes. *(Park-on-OFF depends on Task #33 — see
      Pre-flight.)*
- [ ] **5.5** Write `heat_threshold` / `cool_threshold` / `park_pos` from
      Z2M → confirm the values persist across a full power-cycle (they're
      saved to NVS via `config_save()`). Write a thermostat `SystemMode`
      from Home Assistant/Z2M → confirm it's accepted (no error) but the
      detected mode is unaffected (mode is always auto-detected from HX-A).
- [ ] **5.6** Trigger the `resync` write-bool entity → confirm the valve
      resyncs and the `resync` attribute self-clears back to 0 shortly
      after. Trigger an alarm (SUPPLY >36.5 °C for 5 min, as in Checkpoint
      4.6) → confirm the `alarm` binary sensor in Z2M turns ON immediately
      once the alarm bit sets (bitmap attrs report on any change, no
      periodic delay).

**Result:** [ ] PASS  [ ] FAIL — Notes: ______________________________________

---

## Checkpoint 6 — OTA round-trip + rollback (plan Task 27)

**Bench + Z2M.**

- [ ] **6.1** Bump the firmware version, then build:
      ```
      cd firmware
      idf.py build
      ```
      Locate `firmware/build/valvecontroller.bin`. Wrap it into a Zigbee OTA
      image (`.ota`) using the Zigbee OTA image tool, with manufacturer code
      `0x1234` and image type `0x0001` (matching `VALVECTL_MFR_CODE` and the
      OTA cluster config in `zigbee.c`). Add an entry (`modelId`, `url`,
      `fileVersion`) to Z2M's local OTA index config — see the comments at
      the top of `z2m/valvectl.mjs` for the exact mechanism.
- [ ] **6.2** In Z2M, trigger "Check for updates" then "Update" on the
      device. Confirm the transfer completes and the device reboots into
      the new OTA slot.
- [ ] **6.3** Confirm the boot log prints `image pending verify: awaiting
      rejoin + good sweep`, then — after Zigbee rejoin and one clean sensor
      sweep (no faults) — `OTA image validated, rollback cancelled`.
      *(If HX-B is faulted/unplugged during this test, confirm it does
      **not** block validation — that exclusion + the bounded fallback are
      part of Task #33; see Pre-flight.)*
- [ ] **6.4 (rollback test):** Build a deliberately broken image (e.g. force
      an early `abort()` before Zigbee starts in `app_main()`), OTA it, and
      confirm the bootloader **rolls back** to the previous good slot once
      the pending image fails to self-validate. Confirm the device returns
      to service on the old (good) image and firmware version.
- [ ] **6.5** Record firmware versions before/after and the observed
      rollback behavior.

**Result:** [ ] PASS  [ ] FAIL — Notes: ______________________________________

---

## Checkpoint 7 — Full bench validation suite (mains + actuator) (plan Task 29)

**Final acceptance. Mains + real ARA661 actuator — only after Checkpoint 3
has fully passed.**

- [ ] **7.1 Boot safety** — power-cycle 5× with a scope on GPIO2/GPIO3;
      confirm both stay low through every reset, the boot resync always
      runs toward recirc (GPIO3 by default), and the two lines are never
      both high.
- [ ] **7.2 Heating regulation** — hot source loop (≥40 °C); confirm supply
      converges to the heating setpoint with no valve dither (EMA filtering
      + interlock anti-dither working together).
- [ ] **7.3 Cooling regulation** — cold source (≤12 °C); confirm supply
      holds at or above the cooling setpoint and is never driven below the
      17 °C floor; confirm the PI sign is correct for cooling (inverted vs.
      heating).
- [ ] **7.4 Governor** — force supply >36 °C and separately <16 °C; confirm
      an immediate ramp to 0 % in both cases, releasing back to normal
      regulation at 35 °C / 17 °C respectively, in both heating and cooling.
- [ ] **7.5 Alarm dwell** — hold supply >36.5 °C for 5 min; confirm the
      alarm bit sets only after the full dwell, while the governor was
      already correcting from the 36 °C threshold onward.
- [ ] **7.6 Degradation ladder** — unplug SUPPLY (→ FF-only; verify the
      cooling conservative bias with a cold source running), then also
      unplug RETURN (→ PARK; 10 % in cooling, `park_pos` in heating).
      Unplug HX-A (→ hold last detected mode + alarm). Reboot with HX-A
      still unplugged (→ boots IDLE). Replug everything → confirm full
      recovery.
- [ ] **7.7 Resync triggers** — force >3× accumulated full-travel distance
      (300 %) and, separately, 50 direction reversals via repeated setpoint
      steps; confirm auto-resync fires in each case and re-seeds bumplessly
      (integrator reset, no sudden valve slam).
- [ ] **7.8 Zigbee-loss cooling guard** — in COOLING mode, power off the
      Zigbee coordinator; after 30 minutes confirm the effective cooling
      setpoint rises to 21 °C (`LINK_LOSS_COOL_SETPOINT`); restore the
      coordinator → confirm the setpoint returns to the configured value.
      *(This entire behavior depends on Task #33 wiring real join/leave
      state into `control_task_set_link()` — see Pre-flight. Without it,
      the link is effectively always considered "down," so this test is
      not meaningful yet.)*
- [ ] **7.9 OTA** — one more full end-to-end OTA + rollback cycle, per
      Checkpoint 6.
- [ ] **7.10** Sign off the acceptance checklist below; record the firmware
      version and date.

**Result:** [ ] PASS  [ ] FAIL — Notes: ______________________________________

---

## Final sign-off

- Firmware version tested: ______________________
- Date: ______________________
- Tester: ______________________
- All 7 checkpoints passed: [ ] YES  [ ] NO (list any open items above)
