# 8×8 LED matrix display + 2 aux 24 VAC SSR channels — design

**Date:** 2026-08-05
**Branch:** `feat/display-aux` (based on `main` @ 8eff6f9, includes pump iPWM leaf)
**Scope:** Schematic (`pcb/ValveController.kicad_sch`), firmware (display UI + 2 Zigbee switch endpoints), Z2M external converter. PCB placement/routing is a follow-up task.
**Goal:** Local glanceable status display (valve %, state, temps) paged by a button, plus two independently Zigbee-commanded isolated 24 VAC signal pass-through switches.

## Requirements (user-approved)

- **Display:** compact matrix-LED look, read up close (not across the room). Shows valve position %, operating-state icons, temperatures — one page at a time; a button cycles pages. Single 8×8 block with smart rendering: static content wherever it fits, horizontal scroll only for long values.
- **AUX channels:** 2× independent 24 VAC pass-through (IN→OUT), signal-level ≤100 mA, closed/opened **only** by Zigbee command from HA. No input sensing (explicitly dropped — HA already decides when the circuit is live). Galvanic isolation between channel and logic.

## Decisions (user-approved)

1. **Single 8×8 20 mm THT matrix block + I2C driver.** Considered and rejected: 2× 8×8 blocks (16×8 — more space/cost than needed once scrolling UI accepted); discrete 0402 on-PCB matrix (128 hand-placements, alignment risk); 0.91″ OLED (loses matrix look, sourced module). Off-the-shelf SMD 8×8 grids effectively don't exist (~$60 RGB exception) — the THT block is the one deviation from the all-SMD preference.
2. **Driver = IS31FL3730 class (native 2.7–5.5 V)** so the whole display runs from 3V3 with no level shifting. HT16K33 rejected: officially a 4.5–5.5 V part; running it at 3.3 V is out of spec, and at 5 V its VIH exceeds 3.3 V I2C levels.
3. **PhotoMOS (1-Form-A solid-state relay) per AUX channel.** The board's existing MOC3063+Z0103 triac pattern rejected: triac holding current (~5 mA) is unreliable at signal-level loads. Mechanical signal relay rejected: coil power, audible, taller.
4. **Pins (all non-strapping):** IO6 = `I2C_SDA`, IO7 = `I2C_SCL`, IO11 = `BTN_PAGE`, IO22 = `AUX1_EN`, IO23 = `AUX2_EN`. Remaining free after this: IO4, IO5, IO8, IO16, IO17.
5. **One 4-position PTSM terminal (J15)** for both channels: `IN1 | OUT1 | IN2 | OUT2`. No common terminal needed without sensing.
6. **AUX channels default OFF** at boot/reset/crash — 10 k pulldowns on the enable GPIOs guarantee it in hardware; HA re-commands after rejoin.

## Circuit

### Block 1 — display

| Ref | Part | Notes |
|---|---|---|
| DS1 | 8×8 20 mm dot-matrix block, single **green** (KWM-20881AGB) | THT; **must be low-Vf yellow-green/GaP chemistry (Vf ≈ 2.1 V), NOT InGaN pure-green (Vf ≈ 3.2 V — no headroom from 3V3 drive)** |
| U7 | IS31FL3730-QFLS2-TR (QFN-24) | I2C addr strap per datasheet; brightness via current register |
| R18, R19 | 4.7 k 0402 | I2C pull-ups to 3V3 |
| C14 | 100 nF 0402 | U7 decoupling (high-frequency) |
| C15 | 1 µF 0402 | U7 decoupling (bulk) — fitted, required per datasheet Fig. 1 typical application circuit (not optional) |
| SW2 | momentary switch to GND on IO11 | internal pull-up, firmware debounce |

New nets: `I2C_SDA`, `I2C_SCL`, `BTN_PAGE`. Power: 3V3 rail; firmware caps driver current register + scan duty so worst-case display draw ≤ ~40 mA (5 V PSU headroom after pump leaf ≈ 150 mA). U7 pin 3 (SDB, hardware shutdown, active-low) is tied directly to 3V3 — no GPIO is allocated for it (approved pin list is IO6/7/11/22/23 only), so the driver is permanently out of hardware shutdown; firmware controls brightness/on-off entirely via I2C registers.

### Block 2 — AUX SSR channels

| Ref | Part | Notes |
|---|---|---|
| Q6, Q7 | PhotoMOS 1-Form-A, ≥60 V blocking, ≥100 mA load (CPC1017N) | 24 VAC peaks ±34 V → 60 V min |
| RL3, RL4 | 680 Ω 0402 | LED drive ≈ 3 mA from 3.3 V GPIO |
| R20, R21 | 10 k 0402 | pulldowns on IO22/IO23 (board convention; hardware-off at boot) |
| J15 | Phoenix PTSM 4-pos, same family as J8–J14 (1770979) | `AUX1_IN, AUX1_OUT, AUX2_IN, AUX2_OUT` |

New nets: `AUX1_EN`, `AUX2_EN`, `AUX1_IN/OUT`, `AUX2_IN/OUT`. Placement in the LV zone, clear of the mains/triac section per existing `.kicad_dru` rules; the PhotoMOS provides the channel↔logic isolation barrier.

## Firmware

- **Zigbee:** two on/off switch endpoints (numbers assigned from the existing endpoint map at implementation). State default OFF at boot; no persistence of last state.
- **Display engine:** I2C master + 8×8 framebuffer, 3×5 digit font, horizontal scroll routine. Pages: ① valve % (default; 0–99 static two-digit, 100 = full-frame icon), ② state icon (idle/moving/resync/error/Zigbee-lost), ③ supply temp, ④ return temp (temps scroll "21.4°" once, then park integer). Scroll advances in whole-pixel steps at a 30 ms tick (≈33 px/s) — a single tunable constant; I2C runs at 400 kHz so bus load is negligible. Button short-press cycles; auto-return to page ① after 15 s on another page; dim to minimum brightness after 60 s without a button press; wake to full brightness on button or state change.
- **Button:** IO11 active-low, internal pull-up, debounced in firmware.
- **Z2M:** extend the live external converter (ESM) with the two switch entities; display/button are local-only, no Zigbee exposure.
- **Host tests:** page state machine, scroll/render, endpoint command handling in `firmware/test_host`.

## Verification (house rule)

1. Schematic edit: ERC clean + **full netlist diff against a pre-edit export** — only the intended new nets/components may appear.
2. Layout follow-up: DRC with existing custom rules.
3. Bench: AUX channel switching with a real 24 VAC source + load; display current measured against budget.

## Out of scope

PCB placement/routing (follow-up task), enclosure window/light-pipe, display Zigbee exposure, AUX input sensing.

## Open items (settle at implementation)

- ~~Exact MPNs~~ — resolved:
  - DS1 = **KWM-20881AGB** (row-anode, GaP yellow-green/low-Vf chemistry per Device Selection Guide, Vf 2.2 V typ / 2.8 V max — confirms Decision 1's headroom requirement); footprint = custom `Kleist2:LED_Matrix_8x8_20mm`, 2.5 mm pin pitch, 15.0 mm row spacing. Sourced via TME; no verified DigiKey listing exists for this MPN (DigiKey field intentionally left empty in the BOM, MPN populated).
  - U7 = **IS31FL3730-QFLS2-TR**, QFN-24 (only ordering option available — no SOP variant exists).
  - Q6, Q7 = **CPC1017N**, instantiated from the official `Relay_SolidState:CPC1017N` KiCad symbol (no hand-embedded symbol needed; pins 1–4 match datasheet 1:1).
  - J15 = Phoenix **1770979** (4-pos PTSM 0.5/4-2.5-V-THR) — the plan's placeholder candidate "1770967" does not exist as a part; corrected during Task 1 research.
- Zigbee endpoint numbers from the existing endpoint map — still open, belongs to the firmware plan.
- Whether temp pages cover only supply/return or all five sensors (default: two; trivial to extend) — still open, belongs to the firmware plan.
- Carried to firmware plan: IS31FL3730 powers up with its current-register default at 40 mA/row (Lighting Effect Register 0Dh = `0000`), which exceeds the KWM-20881AGB's 25 mA/dot absolute maximum in single-lit-dot cases. Firmware must program the current register to a safe value *before* enabling the display (SDB is hard-wired high, so this is a software gate, not a hardware one).
