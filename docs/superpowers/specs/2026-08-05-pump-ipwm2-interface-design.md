# Wilo Para ST 15-7-50 iPWM2 pump interface — schematic design

**Date:** 2026-08-05
**Scope:** Schematic only (`pcb/ValveController.kicad_sch`). PCB placement/routing, firmware, and Z2M converter are follow-up tasks.
**Goal:** Board can command pump speed (iPWM2 input) and read pump feedback (flow + status) from the ESP32-C6.

## Primary sources

- Wilo-Para/iPWM Technical Guide V1.0 (wilo429191): https://cms.media.wilo.com/cdndoc/wilo429191/4683951/wilo429191.pdf
- WILO Intec "Para ST \*\* 7/iPWM" factory datasheet: https://www.maxiflame.it/public/Files/rif000001/406/datasheet-para_st_7_ipwm.pdf

## Interface facts (verified against both sources)

- **3-wire signal cable**: core 1 (brown) = PWM Input, core 2 (blue/grey) = PWM Common, core 3 (black) = PWM Output. Pump-side connector Facon PR72; Wilo overmoulded signal cables 4530965/4530663/4530764/4530664 (0.5/1/1.5/2 m). Cable < 3 m (EMC), ≥ 0.25 mm².
- **PWM input** (controller → pump): Thevenin ~7 V + 3.15 kΩ at the pump pin. Spec: U_high 4–24.5 V (recommended design ≥ 10 V), U_low ≤ 1 V, 3.5–10 mA, 90–5000 Hz (1 kHz nominal). Wilo reference circuit (fig. 8): 24 V → 2.2 kΩ pull-up (valid 1–2.5 kΩ) → 100 Ω ESD series → pump; NPN/FET open-collector sinks the line.
- **iPWM2 (solar) profile**: < 7 % = stop; 7–12 % = hysteresis (avoid); 12–15 % = min speed; 15–95 % = linear min→max; > 95 % = max. **0 % / wire-break = pump stops.**
- **PWM output** (pump → controller): open-collector behind internal 470 Ω, 75 Hz ± 2. U_oH window 3–25 V (controller supplies pull-up), U_oL ≤ 1 V. Wilo reference circuit (fig. 11): pull-up → node → 100 Ω → pump, filter C at node. Wilo formula with VCC = 3.3 V yields ≈ 5.6 kΩ.
- **Feedback encoding**: 2 % standby; 5–75 % flow (full scale ambiguous: 1.4 m³/h per guide vs 2.1 m³/h per datasheet graph — bench-calibrate); 80 % abnormal running; 85/90 % stopped-but-functional; 95 % permanent failure; 0/100 % = short/open circuit.

## Decisions (user-approved)

1. **Signal only** — pump mains stays off-board. No HV changes.
2. **Reset/crash failsafe = pump STOPS** — drive line held low whenever the GPIO floats.
3. **24 V source = boost leaf; power chain untouched** — IRM-03-5 stays. Considered and rejected: IRM-03-24 + 24→5 buck (native 24 V, but puts a new stage in the whole board's critical path for one 11 mA pull-up); 5 V pull-up (below Wilo's recommended ≥ 10 V).

## Circuit

### Block 1 — 24 V boost leaf (net `24V`)

| Ref | Part | Notes |
|---|---|---|
| U6 | TPS61040DBVR (SOT-23-5) | hysteretic boost, 28 V max |
| L1 | 10 µH, Isat ≥ 500 mA (MPN at implementation, e.g. NR4018T100M) | |
| D6 | B5819W-TP | existing BOM MPN, 40 V Schottky |
| C9 | 4.7 µF/16 V 0805 | Cin, from `5V` net |
| C10, C11 | 2.2 µF/50 V 1206 X7R | Cout |
| R10/R11 | 1 M / 53.6 k (E96) | FB divider → V_out ≈ 24.2 V; feedforward cap only if TI typical app shows one (verify at implementation) |

EN tied to `5V` (always on). Input = `5V` net (post diode-OR) → 24 V also available on USB-only bench power. Load ≤ ~11 mA (~75 mA at 5 V) — inside the IRM's ~230 mA headroom.

### Block 2 — drive (Wilo fig. 8), IO20

`24V` → **R14 2.2 kΩ ERJ-P08J222V (0.5 W 1206)** → line node → **R15 100 Ω ERJ-P08J101V** (existing MPN) → J14.1.
**Q5 BSS138** drain on line node, source GND. Gate: **R12 100 Ω** series from IO20 (net `PUMP_PWM`) + **R13 10 k pull-up to 3V3**.

- GPIO floats (reset/flash/crash) → FET on → line low → pump stopped. Board unpowered → line ~0 V → stopped.
- R14 sized 0.5 W: dissipates 262 mW continuously whenever pump is commanded off.
- Logic inverted: line duty = complement of GPIO duty (LEDC hardware invert).

### Block 3 — feedback (Wilo fig. 11), IO21

3V3 → **R16 5.6 k 0402** → node (net `PUMP_FB`, direct to IO21) → **R17 100 Ω ERJ-P08J101V** → J14.3. **C12 10 nF 0402** node→GND.
Levels: high 3.3 V (≥ 3 V spec min), low ≈ 0.3 V (470 Ω + 100 Ω vs 5.6 k). No level shifter.

### Connector — J14

Phoenix PTSM 0,5/3-2,5-V THR, MPN 1770966 (same as J8–J12; next column slot y = 97.79, x = 240.03).
Pin order **matches Wilo cable-core numbering**, not the sensor GND-on-pin-1 convention:

| Pin | Net | Wilo core |
|---|---|---|
| 1 | PWM to pump (via R15) | 1, brown |
| 2 | GND (PWM Common) | 2, blue/grey |
| 3 | Feedback (via R17) | 3, black |

### GPIO map addition

| GPIO | Net | Direction | Peripheral |
|---|---|---|---|
| IO20 | PUMP_PWM | out | LEDC 1 kHz, inverted |
| IO21 | PUMP_FB | in | 75 Hz duty capture (GPIO ISR or RMT RX) |

IO22/IO23 remain free. IO4/IO5 avoided (strapping).

New references used: U6, Q5, D6, J14, L1, R10–R17, C9–C12 — verified non-colliding.

## Deliberately omitted

TVS diodes (Wilo reference has none; cable ≤ 3 m), switchable boost EN, pump mains switching, level shifters.

## Firmware implications (informational, out of scope)

LEDC 1 kHz inverted output; duty map 15–95 % = min→max, park at 0 % for stop, never command 7–12 %; feedback decode per the table above with per-state reaction times (80 % state lags up to 60 s); flow full-scale needs bench calibration; new Zigbee attrs + Z2M expose for pump speed/flow/status later.

## Verification plan

1. ERC clean after edit.
2. **Full netlist diff** against pre-edit export (house rule since the J14/J2 label incident) — only the new nets/components may appear.
3. All new symbols carry hidden MPN + DigiKey properties; stock verified via DigiKey skill at implementation.
4. TPS61040 pinout/FB network cross-checked against the TI datasheet before wiring the symbol.

## Unresolved questions

1. Flow feedback full scale 1.4 vs 2.1 m³/h — Wilo docs conflict; settle by bench calibration (or Wilo Intec support).
2. Wilo factory article number for Para ST 15-7-50 iPWM2 not published — confirm from the physical pump label when it arrives.
3. Exact inductor MPN + optional FB feedforward cap — settle at implementation against the TI datasheet.
