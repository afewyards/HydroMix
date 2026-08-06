# ValveController

A mains-powered, Zigbee-connected controller for a hydronic **3-way mixing valve** — it
regulates the supply-water temperature of a floor heating/cooling system and keeps
regulating **autonomously** if the Zigbee link goes down.

One repo, three parts: a custom KiCad PCB (ESP32-C6), ESP-IDF firmware with a
host-testable control core, and a Zigbee2MQTT converter.

> **Status:** Firmware v1 committed; pure-C control core passes host-side unit tests;
> on-target bring-up per [`firmware/BENCH_CHECKLIST.md`](firmware/BENCH_CHECKLIST.md).
> PCB rev A — placement complete, routing in progress. Personal project.

## What it does

A 3-way valve mixes **SOURCE** water (the hot or cold secondary side of a heat exchanger)
with **RETURN** water from the floor loop to produce **SUPPLY** water at a target
temperature. The controller:

- **Auto-detects mode** (heating / cooling / idle) from a heat-exchanger inlet sensor — the
  host never has to tell it which season it is.
- Regulates supply temperature with a **feed-forward mixing model + PI trim**, hard-bounded
  to floor-safe limits (17–35 °C).
- Drives an ESBE ARA661 230 V 3-point actuator through two solid-state (opto-triac) channels.
- Reads **5× DS18B20** probes (supply, return, source, and two heat-exchanger points) over 1-Wire.
- Exposes everything over **Zigbee** (Router role) for monitoring, tuning, the
  regulation-enable toggle, and **OTA** — with a first-class **Home Assistant / Zigbee2MQTT**
  integration.
- **Runs fully standalone** when Zigbee is unavailable, with a supply "governor" that
  de-escalates any temperature excursion toward safe recirculation regardless of mode.

### ⚠️ The counter-intuitive safety rule

Valve position **0 % = full recirculation** (return water only — thermally harmless).
**100 % = full source** (full hot or cold). Parking the valve "closed" (100 %) would run the
floor at full source temperature and is **dangerous** — so the safe park/idle position is
**mid-travel (~50 %)**, and a resync always drives toward the 0 % end, never the source end.
See the [design spec](docs/superpowers/specs/2026-07-13-firmware-design.md) for the full rationale.

## Hardware

Custom single-board design in KiCad (`pcb/`). Rev A highlights:

| Block | Parts |
|---|---|
| MCU | **ESP32-C6-WROOM-1-N4** (RISC-V, 802.15.4 Zigbee radio, 4 MB flash) |
| Power | 230 V AC → **Mean Well IRM-03-5** (5 V) → **TLV75733** LDO (3.3 V); USB-C 5 V is diode-OR'd in (B5819W) so the board bench-powers and flashes over USB-C with **no mains** |
| Valve drive | 2× channel: **MOC3063S** zero-cross opto-triac → **Z0103MN** triac, switching the 230 V 3-point actuator; RC snubbers + X2 safety caps |
| Mains protection | Varistor (B72210S) + fuse |
| Sensing | 5× **DS18B20** on dedicated 1-Wire connectors (4.7 k pull-ups) |
| I/O | Tactile button, 8×8 LED matrix (**KWM-20881AGB** + **IS31FL3730** driver), **USB-C** — native USB-Serial-JTAG (power, flash, console, JTAG in one port) |
| Connectors | Phoenix pluggable terminal blocks (mains in, actuator) + 5 sensor headers |

Full parts list: [`pcb/ValveController.csv`](pcb/ValveController.csv). Custom 3D component models
live in `pcb/3dmodels/`; `board.step` (repo root) is the assembled-board export.

## Firmware

ESP-IDF 5.5.x, plain C on FreeRTOS. See **[`firmware/README.md`](firmware/README.md)** for the
full build/flash/monitor guide, GPIO map, partition/OTA scheme, console commands, and gestures.

The control logic lives in a **pure-C, IDF-free component** (`firmware/components/ctrl_core/`) so
it compiles and unit-tests on the host (macOS/Linux) before any hardware is involved:

- `feedforward` — exact mixing physics, `(T_set − T_return) / (T_source − T_return)`
- `pi` — PI trim with conditional anti-windup
- `governor` — outermost safety stage; ramps toward recirculation on any supply excursion
- `mode_detect` — heating/cooling/idle with asymmetric dwell + hysteresis
- `degradation` — sensor-fault fallback ladder
- `pos_estimator`, `interlock`, `alarm`, `config_map`

The IDF-side glue (`firmware/main/`) wires that core to hardware: `sensors_hw`, `valve_hw`,
`zigbee`, `ota`, `ui`, `console`, `control_task`, `config`.

Quick start:

```bash
. ~/esp/esp-idf/export.sh
cd firmware
idf.py set-target esp32c6       # first time only
idf.py build
idf.py -p <PORT> flash monitor  # over USB-C; Ctrl+] to exit the monitor
```

Host-side control tests (plain CMake + Unity, no hardware): [`firmware/test_host/`](firmware/test_host/).

## Zigbee / Home Assistant

Router role · manufacturer `Knife` · model `HydroMix`.

- **EP1** — Basic, Identify, On/Off (`water_running` = regulation enable, driven by the host's
  pump/flow automation), Thermostat (local temp = supply; setpoints clamped 17–35 °C; mode is
  auto-detected, host `SystemMode` writes accepted-but-ignored), Analog Output (manual position,
  writable only while regulation is off), OTA client, and a manufacturer-specific cluster
  (`0xFC00`) exposing ~10 tunables, a self-clearing `resync`, and read-only alarm/fault bitmaps.
- **EP2–EP6** — Temperature Measurement for supply, return, source, HX-A, HX-B.

[`z2m/valvectl.mjs`](z2m/valvectl.mjs) is the external Zigbee2MQTT converter (friendly entity names,
tunables as settings, alarms as binary sensors, OTA index). The manufacturer OTA-publishing steps
are documented in the comment header of that file.

## Repository layout

```
ValveController/
├── pcb/                        KiCad project (schematic, board, DRC rules, BOM)
│   ├── 3dmodels/               custom STEP models
│   ├── Kleist2.pretty/         project footprint library
│   └── backups/                local pre-edit snapshots (git-ignored)
├── firmware/                   ESP-IDF application
│   ├── main/                   IDF glue (sensors, valve, zigbee, ota, ui, console)
│   ├── components/ctrl_core/   pure-C control logic (host-testable)
│   └── test_host/              Unity unit tests
├── z2m/valvectl.mjs             Zigbee2MQTT external converter
├── docs/superpowers/           design spec + implementation plan
└── board.step                  assembled-board 3D export
```

## Safety

⚠️ This board switches **230 V AC mains**. All bench work — firmware, sensor reads, triac-drive
verification — can be done on **USB-C power alone**; do not connect mains until the drive channels
are verified per [`firmware/BENCH_CHECKLIST.md`](firmware/BENCH_CHECKLIST.md). And keep the position
convention in mind: the safe idle is **mid-travel**, not "closed."

## Documentation

- [Firmware guide](firmware/README.md) — build, GPIO map, OTA, console, gestures
- [Design spec](docs/superpowers/specs/2026-07-13-firmware-design.md) — control theory, safety, Zigbee model
- [Implementation plan](docs/superpowers/plans/2026-07-13-firmware.md)
- [Bench checklist](firmware/BENCH_CHECKLIST.md)

---

Personal project by Thierry Kleist. Released under the [MIT License](LICENSE).
