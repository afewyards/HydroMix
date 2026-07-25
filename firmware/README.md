# ValveController Firmware

ESP-IDF firmware for the ValveController PCB (ESP32-C6) — a hydronic 3-way mixing
valve controller regulating supply temperature, exposed over Zigbee (Router role)
with OTA, autonomous when the Zigbee link is down.

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
- The image is marked valid — cancelling any pending rollback — only once
  **both** conditions are met:
  1. `ota_note_joined()` — Zigbee has rejoined the network (called from
     `zigbee_on_join()` in `app_main.c`).
  2. `ota_note_good_sweep()` — at least one control cycle has completed with
     no sensor faults (called from `control_task.c` once supply/return/source/
     HX-A/HX-B are all fault-free).

  If the new image never reaches both conditions, the bootloader rolls back to
  the previous image on the next reset.

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
  ignored — mode is auto-detected from HX-A), Analog Output (position 0–100 %,
  writable only while `water_running` is OFF), OTA client, plus a
  manufacturer-specific custom cluster `0xFC00` exposing 10 read-write
  tunables, a self-clearing `resync` bool, and read-only alarm-bitmap /
  fault-bitmap / travel-since-resync attributes. Alarm and fault bitmaps report
  **immediately** on change (no periodic cap); temperatures report at ±0.2 K or
  60 s max; position reports at ±1 % or 60 s max.
- **EP2–EP6**: Temperature Measurement (supply, return, source, HX-A, HX-B).

## Zigbee2MQTT integration

`z2m/valvectl.mjs` is the external converter. See the comments at the top of
that file for the manual OTA-image publishing process referenced above.

## Host-side control-logic tests

See `firmware/test_host/README.md` — the `ctrl_core` component (pure C, no IDF
includes) is unit-tested on macOS/Linux with plain CMake + Unity before any
hardware is touched.
