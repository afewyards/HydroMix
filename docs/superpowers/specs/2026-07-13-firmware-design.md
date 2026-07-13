# ValveController Firmware — Design Spec

**Date:** 2026-07-13 (rev 2 — post-review: HVAC + embedded + schematic reviews applied)
**Hardware:** ValveController PCB rev A — ESP32-C6-WROOM-1-N4 (4 MB), ESBE ARA661 230 V 3-point actuator via 2× MOC3063S zero-cross opto-triacs, 5× DS18B20 on dedicated 1-Wire GPIOs, BOOT button, status LED, USB-C (native USB-Serial-JTAG).

## 1. System overview

Hydronic mixing controller for a heating/cooling floor system. A 3-way valve mixes **SOURCE** (secondary side of a heat exchanger, either hot ≥40 °C or cold ≤12 °C) with **RETURN** (floor loop return) into **SUPPLY** (into the floor). Firmware regulates supply temperature to a mode-dependent setpoint, hard-bounded by floor limits (max 35 °C, min 17 °C). Zigbee (router) provides monitoring, configuration, the regulation-enable toggle, and OTA updates. The device also regulates fully autonomously if Zigbee is down (with one cooling-specific guard, §4.4).

### Plumbing / position convention

- **0 %** = full recirculation (100 % return port) — thermally harmless end.
- **100 %** = full source — the hot/cold end. **Parking "closed" is dangerous** (floor would run full hot/cold); the safe park position is mid-travel.
- A config flag (`direction_swap`) maps which triac direction moves toward source, since this depends on valve installation orientation.

## 2. Hardware map

| Function | GPIO | Net | Notes |
|---|---|---|---|
| Valve → open triac | GPIO2 | SSR_OPEN | MOC3063S, active high; **10 k pull-down to GND** (schematic change pending) |
| Valve → close triac | GPIO3 | SSR_CLOSE | MOC3063S, active high; **10 k pull-down to GND** (schematic change pending) |
| Supply temp | GPIO0 | TEMP_SUPPLY | DS18B20, own connector, 4.7 k pull-up |
| Return temp | GPIO1 | TEMP_RETURN | DS18B20, own connector |
| Source temp | GPIO10 | TEMP_SOURCE | DS18B20, own connector |
| HX inlet temp | GPIO18 | TEMP_HX_A | DS18B20, own connector — **mode detection** |
| HX exhaust temp | GPIO19 | TEMP_HX_B | DS18B20, own connector — monitoring |
| Button | GPIO9 | BOOT | Also HW bootstrap when held at power-up |
| Status LED | GPIO15 | STAT_LED | **Active-low**: cathode→GPIO15, anode→R4→3V3 (schematic rewire pending — fixes D1 reverse-bias bug AND defines the GPIO15 JTAG strap high at reset) |
| Console + flash + JTAG | GPIO12/13 | USB_D−/USB_D+ | Native USB-Serial-JTAG via USB-C |

UART0/J15 debug header removed from the design (owner decision 2026-07-13); DBG_TX/DBG_RX labels to be deleted from the schematic.

ARA661 facts firmware relies on: 120 s/90° travel, self-locking (hold = both triacs off), safe to stall at end-stops, **never both triacs on**.

**Boot-time triac safety:** GPIO2/3 are high-impedance (floating) at reset — NOT driven low. A floating pin cannot source the ~5 mA a MOC3063 needs, but defense-in-depth: external 10 k pull-downs (above) + `app_main` configures GPIO2/3 as output-low as its **first** action, before any other init.

## 3. Platform

- **ESP-IDF 5.x + esp-zigbee-sdk, plain C, FreeRTOS.**
- Partition table: two OTA app slots (≥1.8 MB each) + `nvs` + `ota_data` + `phy_init` + `zb_storage` + `zb_fct` in 4 MB. Verify real binary size early; expected ~1.1–1.5 MB.
- Zigbee role: **Router** (mains powered).

## 4. Modules

Each module = one .c/.h pair; pure-logic modules take inputs/outputs as plain structs (no FreeRTOS/driver includes) so they compile host-side for unit tests.

### 4.1 `sensors`
- 5 physical 1-Wire connectors, **time-shared over the C6's RMT channels** — the C6 has only 2 RMT TX + 2 RX channels and the `onewire_bus` driver consumes one TX+RX pair per bus, so 5 concurrent buses are impossible (verified against `soc_caps.h` and driver source).
- **Pipelined sweep every 10 s** (~0.85 s total): for each of the 5 GPIOs in turn: create bus → Skip ROM + Convert T → delete bus (~3 ms each; sensors are externally powered, so the line may be released during conversion — the 4.7 k pull-up holds it high). One 750 ms `vTaskDelay` after the last Convert T, then for each GPIO: create bus → read scratchpad → delete bus. 12-bit resolution.
- One DS18B20 per wire → Skip ROM addressing, connector = identity, no ROM enumeration.
- CRC check, up to 3 retries. Per-sensor state: last value, timestamp, fault flag (fault after 3 consecutive failed sweeps).
- **EMA filter (τ ≈ 40 s) on T_source and T_return** before they feed the feed-forward math (kills valve dither from transients/stratification). T_supply stays unfiltered for the PI.

### 4.2 `valve` (sole owner of GPIO2/3)
- **Interlock:** never both outputs high. A request for both → both off + error counter. Enforced in this module and nowhere else.
- ≥500 ms dead time when reversing direction. **Minimum pulse 2 s** (short pulses under-travel due to motor spin-up/backlash and would bias the run-time position estimate; 2 s ≈ 1.7 % travel).
- Anti-dither: no direction reversal within 10 s of the previous move.
- **Position estimator:** 0–100 % integrated from run-time; `travel_time_s` config (default 120 s per ARA661).
- **Resync:** drive toward the 0 % end-stop for `travel_time × 1.15` (safe stall), then position := 0 %. Triggered: at boot; on demand (Zigbee attribute / CLI); when accumulated travel exceeds 3× full travel; **or** after 50 direction reversals (backlash dominates drift); whichever comes first. Never resync toward the 100 %/source end.
- Resync is a **non-blocking state machine** inside the valve task (never busy-waits; task WDT stays fed; telemetry keeps running). Bumpless: control loop freezes its integrator during resync, and on completion re-seeds target from fresh feed-forward with integrator = 0.
- API: `valve_set_target(pct)`, `valve_get_position()`, `valve_resync()`, `valve_stop()`.

### 4.3 `control` (10 s cycle, runs when `water_running` = ON)
- **Mode detect** from **TEMP_HX_A**: ≥ `heat_threshold` (default 28 °C) → HEATING (setpoint default 35 °C); ≤ `cool_threshold` (default 16 °C) → COOLING (setpoint default 18 °C); in between → IDLE (park at `park_pos`, default 50 %). Hysteresis 2 K. **Asymmetric dwell: 60 s to enter a mode, 7 min (config) to leave it** — a boiler off-cycle or heat-pump defrost dip on HX-A must not flip the mode. Setpoints configurable, clamped to ≤ 35 / ≥ 17.
- **Boot mode = IDLE** until HX-A produces a valid reading that classifies the mode (never guess, never trust a stale "last mode" from before power-loss).
- **Feed-forward:** `pos_ff = (T_set − T_return) / (T_source − T_return)` on the filtered inputs, clamped 0–100 %. Exact mixing physics, valid in both modes without sign handling. When |T_source − T_return| < 2 K (no authority): **freeze the last valid target** (do not zero it — zeroing would slam the valve toward recirc) and freeze the integrator.
- **PI trim** on supply error (`T_set − T_supply`, sign inverted in cooling): added to `pos_ff`. **Conditional anti-windup:** integrate only when the *final clamped target* is unsaturated OR the error pushes away from the clamp. **PI disabled (FF-only) for 3 cycles after any mode change** — a wrong-sign PI during changeover is positive feedback.
- **Supply governor — outermost stage, after FF+PI, before `valve_set_target()`:** whenever T_supply > `gov_high` (default 36 °C) or < `gov_low` (default 16 °C), override the computed target and ramp toward 0 % at full slew until supply re-enters [17, 35]. Because 0 % = recirculated return water, this de-escalates any extreme in either mode with no mode knowledge — it survives mode mis-detection and bad FF. This *is* regulation (per the owner's alarms-never-park rule), just with a guaranteed-safe direction.
- **water_running = OFF:** park at `park_pos`, clear integrator, keep measuring and reporting. This toggle is *regulation enable*, not power — Zigbee, sensors, and reporting always run.

### 4.4 `safety` (independent task, always runs)
Alarms are **pure telemetry — they never stop or override regulation.** The control loop (including its governor stage) is the corrective action.
- **Alarm thresholds (owner choice):** supply > 36.5 °C or < 15.5 °C **continuously for `alarm_dwell` (default 5 min)** → alarm bit (hysteresis: clears at 35.5/16.5, dwell resets whenever supply re-enters bounds). The dwell gives the loop — governor included — time to resolve an excursion before it's reported; margins sit outside normal control ripple around the 35/18 setpoints. The governor (§4.3) still engages immediately at 36/16 — only the *reporting* waits, never the correction.
- **Sensor-fault degradation ladder** (alarm at every level below 1):
  1. All sensors OK → feed-forward + PI + governor
  2. Supply faulted → feed-forward only. **In COOLING, bias FF conservative** (offset toward less source, i.e. higher supply) — with no supply feedback, an optimistic FF could push the floor below dew point undetected.
  3. Source or return faulted → pure PI on supply (+ governor)
  4. Supply AND (source or return) faulted → park: `park_pos` in heating/idle, **10 % (toward recirc) in cooling** — a blind 50 % with a 12 °C source mixes to ~15–16 °C = condensation.
  - HX-A faulted → hold last known mode + alarm (no blind mode flips); if faulted at boot → IDLE.
  - HX-B faulted → alarm only.
- **Dew-point guard (autonomous cooling):** firmware has no humidity input — dew-point safety is HA's job via `water_running`/setpoint. If the Zigbee link is lost (no coordinator contact > 30 min) while in COOLING, raise the effective cooling setpoint to 21 °C until the link returns. Heating needs no equivalent.
- Task watchdog covers control + valve tasks. Reset/panic/brownout → GPIO2/3 float but pull-downs + MOC3063 input threshold guarantee triacs off → valve self-locks in place; every boot starts with a resync then normal regulation.

### 4.5 `zigbee`
Endpoints:
- **EP1 (main):**
  - Basic — manufacturer `Kleist`, model `ValveCtl-C6`, fw version.
  - Identify — LED fast-blink.
  - **On/Off — `water_running` regulation enable.** HA flips this from the pump/flow automation.
  - Thermostat — local temperature = supply; occupied heating/cooling setpoints (writable, clamped ≤35/≥17); `ThermostatRunningMode` = detected mode. **Mandatory attributes `ControlSequenceOfOperation` and `SystemMode` populated; host writes to `SystemMode` are accepted-but-ignored** (mode is auto-detected; it must not fight the loop).
  - Analog Output — valve position 0–100 %; **writable** = manual position override, accepted only while `water_running` = OFF. A written value supersedes `park_pos` until `water_running` turns ON or the device reboots.
  - OTA upgrade client.
- **EP2–EP6:** Temperature Measurement × 5 → supply, return, source, HX-A, HX-B.
- **Custom manufacturer-specific cluster (EP1)** — attributes only, no custom ZCL commands (simpler in esp-zigbee-sdk and Z2M): `heat_threshold`, `cool_threshold`, `travel_time_s`, `park_pos`, `direction_swap`, PI gains (`kp`, `ki`), governor thresholds, `alarm_dwell`, **`resync` as a self-clearing writable bool** (write 1 → resync → auto-clear), alarm bitmap, sensor-fault bitmap, travel-since-resync. Converter must pass the manufacturer code on reads/writes.
- **Reporting:** temps on ±0.2 K change or 60 s max; position on ±1 %; alarm/fault bitmaps immediately.
- **Z2M external converter** (JS, lives in this repo, `z2m/valvectl.js`): friendly entity names (`water_running`, `supply_temp`, …), tunables as settings, alarm as binary sensor, OTA index entry.

### 4.6 `ui`
- **Button GPIO9:** short press → network steering (join); hold 5 s → Zigbee leave + NVS factory reset. (Held at power-up = ROM bootloader — hardware behavior, no firmware.)
- **LED GPIO15 (active-low):** joined idle: 1 blink / 5 s · steering: fast blink · heating: slow single blink · cooling: slow double blink · alarm or sensor fault: rapid triple · identify: continuous fast blink.

### 4.7 `console` (USB-Serial-JTAG)
Log output + minimal CLI on the **native USB-Serial-JTAG console** (the USB-C port — the one that's actually accessible): `status`, `valve <pct>`, `resync`, `mode`, `factory-reset`. No auth (physical access only). UART0 is unused (J15 removed).

### 4.8 `config`
All tunables in NVS with the defaults above. Survives OTA; wiped by factory reset. Written via Zigbee custom cluster or CLI. Runtime state (position estimate, alarm bits) is **never** persisted — boot resync re-establishes truth.

## 5. OTA

Standard Zigbee OTA cluster (client — supported for routers). Build produces a `.ota` file; served by Z2M's local OTA index. Two-slot scheme with `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`: new image calls `esp_ota_mark_app_valid_cancel_rollback()` only after Zigbee rejoin + one successful sensor sweep; otherwise the bootloader rolls back.

## 6. Error handling summary

| Failure | Behavior |
|---|---|
| Supply beyond 36/16 | Governor ramps valve toward 0 % (recirc) — regulation, not parking |
| Supply beyond 36.5/15.5 for > 5 min | + alarm bit (telemetry only; dwell lets the loop fix it quietly first) |
| Any sensor fault | Degradation ladder §4.4; alarm bit |
| Zigbee down / not joined | Full autonomous regulation; if COOLING > 30 min without link → setpoint 21 °C; steering retries with backoff |
| Both-triac request | Both off, error counter, log |
| Panic/brownout/WDT | Triacs off via pull-downs (valve holds), reboot, resync, resume |
| OTA image bad | Bootloader rollback to previous slot |

## 7. Testing

- **Host-side unit tests (Unity)** for all pure-logic modules: PI + conditional anti-windup, feed-forward (freeze-on-low-authority, divide guard), governor, mode detect (asymmetric dwell, boot-IDLE), position estimator, resync triggers, degradation ladder (incl. cooling-specific rungs), interlock. TDD per task.
- **On-target bench plan:** USB-C power only (no mains needed thanks to diode-OR), DS18B20s in water glasses, triac drive verified by LED/scope on SSR_OPEN/SSR_CLOSE before any mains connection; then mains + actual actuator dry-stroke; then Z2M join, converter, reporting, OTA round-trip.

## 8. Hardware changes required (schematic, pre-routing)

1. **D1 rewire to active-low** — as drawn D1 is reverse-biased (pad 1 = cathode on the driven net) and can never light. New wiring: 3V3 → R4 (1 k) → D1 anode (pad 2), D1 cathode (pad 1) → GPIO15. Also defines the GPIO15 JTAG-source strap high at reset.
2. **Add 2× 10 k pull-downs** GPIO2→GND and GPIO3→GND (SSR_OPEN/SSR_CLOSE).
3. **Delete dangling DBG_TX/DBG_RX labels** (J15 dropped for good — owner decision).
4. ERC hygiene (optional): PWR_FLAG on MAINS_L/MAINS_N and 5 V; no-connect flags on unused GPIOs 4/5/6/7/11/20/21/22/23.

## 9. Out of scope

- Pump control, humidity/dew-point sensing (HA's job — see §4.4 guard), weather compensation, HA automations, Secure Boot / flash encryption (eFuse plan dropped 2026-07-13), UART0 console (J15 removed).

## 10. Open questions

1. ~~LED polarity~~ RESOLVED: active-low by design (§8.1) — fixes the D1 polarity bug and the strap in one move.
2. ~~Travel time~~ RESOLVED: ESBE ARA661 → 120 s/90°.
3. `heat_threshold`/`cool_threshold` defaults (28/16 °C) — sane for 40+/12− sources, tune on the real system.
