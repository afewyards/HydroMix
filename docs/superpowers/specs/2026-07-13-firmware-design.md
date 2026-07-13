# ValveController Firmware — Design Spec

**Date:** 2026-07-13
**Hardware:** ValveController PCB rev A — ESP32-C6-WROOM-1-N4 (4 MB), ESBE ARA661 230 V 3-point actuator via 2× MOC3063S zero-cross opto-triacs, 5× DS18B20 on dedicated 1-Wire GPIOs, BOOT button, status LED, USB-C (native USB-Serial-JTAG), UART0 debug header.

## 1. System overview

Hydronic mixing controller for a heating/cooling floor system. A 3-way valve mixes **SOURCE** (secondary side of a heat exchanger, either hot ≥40 °C or cold ≤12 °C) with **RETURN** (floor loop return) into **SUPPLY** (into the floor). Firmware regulates supply temperature to a mode-dependent setpoint, hard-bounded by floor limits (max 35 °C, min 17 °C). Zigbee (router) provides monitoring, configuration, the regulation-enable toggle, and OTA updates. The device also regulates fully autonomously if Zigbee is down.

### Plumbing / position convention

- **0 %** = full recirculation (100 % return port) — thermally harmless end.
- **100 %** = full source — the hot/cold end. **Parking "closed" is dangerous** (floor would run full hot/cold); the safe park position is mid-travel.
- A config flag (`direction_swap`) maps which triac direction moves toward source, since this depends on valve installation orientation.

## 2. Hardware map

| Function | GPIO | Net | Notes |
|---|---|---|---|
| Valve → open triac | GPIO2 | SSR_OPEN | MOC3063S, active high |
| Valve → close triac | GPIO3 | SSR_CLOSE | MOC3063S, active high |
| Supply temp | GPIO0 | TEMP_SUPPLY | DS18B20, own bus, 4.7 k pull-up |
| Return temp | GPIO1 | TEMP_RETURN | DS18B20, own bus |
| Source temp | GPIO10 | TEMP_SOURCE | DS18B20, own bus |
| HX inlet temp | GPIO18 | TEMP_HX_A | DS18B20, own bus — **mode detection** |
| HX exhaust temp | GPIO19 | TEMP_HX_B | DS18B20, own bus — monitoring |
| Button | GPIO9 | BOOT | Also HW bootstrap when held at power-up |
| Status LED | GPIO15 | STAT_LED | Check PCB for polarity (active-low suggested) |
| Debug console | U0TXD/U0RXD | DBG_TX/RX | Log + CLI |

ARA661 facts firmware relies on: ~120 s full travel, self-locking (hold = both triacs off), safe to stall at end-stops (≥120 s drive is harmless), **never both triacs on**.

## 3. Platform

- **ESP-IDF 5.x + esp-zigbee-sdk, plain C, FreeRTOS.**
- Partition table: two OTA app slots + `nvs` + `zb_storage` + `zb_fct` + `ota_data` in 4 MB.
- Zigbee role: **Router** (mains powered).

## 4. Modules

Each module = one .c/.h pair; pure-logic modules take inputs/outputs as plain structs (no FreeRTOS/driver includes) so they compile host-side for unit tests.

### 4.1 `sensors`
- 5 independent 1-Wire buses (ESP-IDF `onewire_bus`, RMT-backed). One DS18B20 per bus → connector = identity, no ROM enumeration.
- Poll all 5 concurrently every 10 s, 12-bit resolution (750 ms conversion).
- CRC check, up to 3 retries. Per-sensor state: last value, timestamp, fault flag (fault after 3 consecutive failed polls or bus absence).

### 4.2 `valve` (sole owner of GPIO2/3)
- **Interlock:** never both outputs high. A request for both → both off + error counter. Enforced in this module and nowhere else.
- ≥500 ms dead time when reversing direction. Minimum pulse 1 s.
- **Position estimator:** 0–100 % integrated from run-time; `travel_time_s` config (default 120 s).
- **Resync:** drive toward the 0 % end-stop for `travel_time × 1.15` (safe stall), then position := 0 %. Performed at boot, on demand (Zigbee command / CLI), and automatically when accumulated travel since last resync exceeds 3× full travel (drift control). Never resync toward the 100 %/source end.
- API: `valve_set_target(pct)`, `valve_get_position()`, `valve_resync()`, `valve_stop()`.

### 4.3 `control` (10 s cycle, runs when `water_running` = ON)
- **Mode detect** from **TEMP_HX_A**: ≥ `heat_threshold` (default 28 °C) → HEATING (setpoint default 35 °C); ≤ `cool_threshold` (default 16 °C) → COOLING (setpoint default 18 °C); in between → IDLE (park at `park_pos`, default 50 %). Hysteresis 2 K + 60 s dwell before any mode change. Setpoints configurable, clamped to ≤ 35 / ≥ 17.
- **Feed-forward:** `pos_ff = (T_set − T_return) / (T_source − T_return)`, clamped 0–100 %. Valid in both modes without sign handling. Skipped (weight 0) when |T_source − T_return| < 2 K.
- **PI trim** on supply error (`T_set − T_supply`, sign inverted in cooling): output added to `pos_ff`, integrator anti-windup clamp when valve saturated at 0/100 %. Target position → `valve_set_target()`; valve module pulses only when |target − estimate| > 2 %.
- **water_running = OFF:** park at `park_pos`, clear integrator, keep measuring and reporting. This toggle is *regulation enable*, not power — Zigbee, sensors, and reporting always run.

### 4.4 `safety` (independent task, always runs)
Alarms are **pure telemetry — they never stop or override regulation.** The control loop is itself the corrective action.
- Supply > 36 °C or < 16 °C → alarm bit set (cleared when back within 33/19 with hysteresis). Loop keeps regulating throughout.
- **Sensor-fault degradation ladder** (alarm at every level below 1):
  1. All sensors OK → feed-forward + PI
  2. Supply faulted → feed-forward only (source + return)
  3. Source or return faulted → pure PI on supply
  4. Supply AND (source or return) faulted → park `park_pos` (nothing left to regulate with)
  - HX-A faulted → hold last known mode (no blind mode flips) + alarm. HX-B faulted → alarm only.
- Task watchdog covers control + valve tasks. GPIO2/3 are low at reset/panic/brownout → both triacs off → valve self-locks in place; every boot starts with a resync then normal regulation.

### 4.5 `zigbee`
Endpoints:
- **EP1 (main):**
  - Basic — manufacturer `Kleist`, model `ValveCtl-C6`, fw version.
  - Identify — LED fast-blink.
  - **On/Off — `water_running` regulation enable.** HA flips this from the pump/flow automation.
  - Thermostat — local temperature = supply; occupied heating setpoint (35); occupied cooling setpoint (18); running mode = detected mode (heat/cool/idle).
  - Analog Output — valve position 0–100 %; **writable** = manual position override, accepted only while `water_running` = OFF (commissioning/testing). A written value supersedes `park_pos` until `water_running` turns ON or the device reboots.
  - OTA upgrade client.
- **EP2–EP6:** Temperature Measurement × 5 → supply, return, source, HX-A, HX-B.
- **Custom manufacturer-specific cluster (EP1):** `heat_threshold`, `cool_threshold`, `travel_time_s`, `park_pos`, `direction_swap`, PI gains (`kp`, `ki`), resync command, alarm bitmap, sensor-fault bitmap, position-estimate age since last resync.
- **Reporting:** temps on ±0.2 K change or 60 s max; position on ±1 %; alarm/fault bitmaps immediately.
- **Z2M external converter** (JS, lives in this repo, `z2m/valvectl.js`): friendly entity names (`water_running`, `supply_temp`, …), tunables as settings, alarm as binary sensor, OTA index entry.

### 4.6 `ui`
- **Button GPIO9:** short press → network steering (join); hold 5 s → Zigbee leave + NVS factory reset. (Held at power-up = ROM bootloader — hardware behavior, no firmware.)
- **LED GPIO15:** joined idle: 1 blink / 5 s · steering: fast blink · heating: slow single blink · cooling: slow double blink · alarm or sensor fault: rapid triple · identify: continuous fast blink.

### 4.7 `console` (UART0)
Log output + minimal CLI: `status`, `valve <pct>`, `resync`, `mode`, `factory-reset`. Bench bring-up tool; no auth (physical access only).

### 4.8 `config`
All tunables in NVS with the defaults above. Survives OTA; wiped by factory reset. Written via Zigbee custom cluster or CLI.

## 5. OTA

Standard Zigbee OTA cluster (client). Build produces a `.ota` file; served by Z2M's local OTA index. Two-slot partition scheme with rollback: new image must mark itself valid (after Zigbee rejoin + one successful sensor poll) or the bootloader rolls back.

## 6. Error handling summary

| Failure | Behavior |
|---|---|
| Supply out of bounds | Alarm bit; regulation continues (it is the fix) |
| Any sensor fault | Degradation ladder §4.4; alarm bit |
| Zigbee down / not joined | Full autonomous regulation with stored config; steering retries with backoff |
| Both-triac request | Both off, error counter, log |
| Panic/brownout/WDT | Triacs off (valve holds), reboot, resync, resume |
| OTA image bad | Bootloader rollback to previous slot |

## 7. Testing

- **Host-side unit tests (Unity)** for all pure-logic modules: PI + anti-windup, feed-forward (incl. divide-by-near-zero guard), mode detect (hysteresis/dwell), position estimator, degradation ladder, interlock logic. TDD per task.
- **On-target bench plan:** USB-C power only (no mains needed thanks to diode-OR), DS18B20s in water glasses, triac drive verified by LED/scope on SSR_OPEN/SSR_CLOSE before any mains connection; then mains + actual actuator dry-stroke; then Z2M join, converter, reporting, OTA round-trip.

## 8. Out of scope

- Pump control (no output on the board), humidity/dew-point sensing, weather compensation, Home Assistant automations (HA decides `water_running` and any setpoint tweaks), Secure Boot / flash encryption (eFuse plan dropped 2026-07-13).

## 9. Open questions

1. LED polarity — active-low or active-high — final schematic wiring to be confirmed when routing is done (abstracted behind `ui`, one `#define`).
2. ~~Travel time~~ RESOLVED 2026-07-13: actuator confirmed ESBE ARA661 → 120 s / 90°. Config default stays 120 s.
3. `heat_threshold`/`cool_threshold` defaults (28/16 °C) — sane for 40+/12− sources, tune on the real system.
