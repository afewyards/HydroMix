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
| U6 | TPS61040DRVR (WSON-6, U2's footprint) | hysteretic boost, 28 V max |
| L1 | LSXND4040TKL100MDG (Isat 1.3 A; renamed successor of NRS4018/NR4018 — TY 2021 renumbering) | 10 µH |
| D6 | B5819W-TP | existing BOM MPN, 40 V Schottky |
| C9 | 4.7 µF/16 V 0805, GRM21BR71C475KE51K | Cin, from `5V` net |
| C10, C11 | 2.2 µF/50 V 1206 X7R, GCM31CR71H225KA55L | Cout |
| R10/R11 | 249 k / 13.3 k (E96) | FB divider → V_out ≈ 24.3 V, worst-case ≤ 25.5 V incl. I_FB |
| C13 | 82 pF C0G 0402, GRM1555C1H820JA01D | FB feedforward across R10 (TI SLVS413K §8.2.2.2; bench-verify vs double-pulsing) |
| D7 | BZT52B27 (Diotec, SOD-123F, 26.46–27.54 V, 500 mW) | 24 V rail clamp — TPS61040 has no internal OVP; FB-open fault otherwise runs U6+Q5 to their shared 30 V breakdown |

EN tied to `5V` (always on). Input = `5V` net (post diode-OR) → 24 V also available on USB-only bench power. Load ≤ ~11 mA (~75 mA at 5 V) — inside the IRM's ~230 mA headroom.

### Block 2 — drive (Wilo fig. 8), IO20

`24V` → **R14 2.2 kΩ ERJ-P08J222V (0.5 W 1206)** → line node → **R15 100 Ω ERJ-P08J101V** (existing MPN) → J14.1.
**Q5 CSD17313Q2** (TI NexFET, SON 2×2 mm, hand-authored `Kleist2:CSD17313Q2` footprint; G=3, S=4+7/EP, D=1,2,5,6,8; BSS138 SOT-23 documented fallback) drain on line node, source GND. Gate: **R12 100 Ω** series from IO20 (net `PUMP_PWM`) + **R13 10 k pull-up to 3V3**.

- GPIO floats (reset/flash/crash) → FET on → line low → pump stopped. Board unpowered → line floats to the pump's own ~7 V open-circuit level (its wire-break signature, no edges) → stopped. Known uncovered corner (accepted): 3V3 rail dead while 5 V alive → gate pull-up limp → line driven high ~17 V → pump runs at max; benign (over-circulation only, valve outputs are dark in the same failure).
- R14 sized 0.5 W: dissipates 262 mW continuously whenever pump is commanded off.
- Logic inverted: line duty = complement of GPIO duty (LEDC hardware invert).

### Block 3 — feedback (Wilo fig. 11), IO21

3V3 → **R16 10 k 0402** → node (net `PUMP_FB`, direct to IO21) → **R17 100 Ω ERJ-P08J101V** → J14.3. **C12 10 nF 0402, C0402C103K5RECAUTO** node→GND.
Levels: high 3.3 V (≥ 3 V spec min), low ≈ 0.3 V (470 Ω + 100 Ω vs 10 k; worst-case V_low 0.68 V at IO21, +146 mV margin under VIL vs the prior 5.6 k). No level shifter.

### Connector — J14

Phoenix PTSM 0,5/3-2,5-H THR, MPN 1770898 (next column slot y = 97.79, x = 240.03). *(Revised 2026-08-05: owner switched J14 from the vertical 1770966 to the horizontal/angled sibling 1770898 — wire entry parallel to the PCB; same PTSM 0,5 family, 2.5 mm pitch, 0.5 mm² max. The Wilo 4222049 cable's 0.35–0.5 mm² ferrules still fit; a considered upgrade to a ≥1.5 mm² family (PT/SPT/PTS) was dropped in favor of family consistency.)*
Pin order **matches Wilo cable-core numbering**, not the sensor GND-on-pin-1 convention:

| Pin | Net | Wilo core |
|---|---|---|
| 1 | PWM to pump (via R15) | 1, brown |
| 2 | GND (PWM Common) | 2, blue/grey |
| 3 | Feedback (via R17) | 3, black |

Mis-plug consequences (same PTSM housing as J8–J12): pump cable in a sensor port → pump input grounded → pump stops, no damage; sensor cable in J14 → sensor sees the PWM line current-limited to ~10 mA by R14 — no damage path either way; 24 V never reaches an ESP32 pin.
Add a PUMP silk callout at layout.

### GPIO map addition

| GPIO | Net | Direction | Peripheral |
|---|---|---|---|
| IO20 | PUMP_PWM | out | LEDC ≤ 500 Hz, inverted (rise-time margin: 1 kHz leaves only +12 % vs Wilo's T/500 with 3 m cable) |
| IO21 | PUMP_FB | in | 75 Hz duty capture (GPIO ISR or RMT RX) |

IO22/IO23 remain free. IO4/IO5 avoided (strapping).

New references used: U6, Q5, D6, D7, J14, L1, R10–R17, C9–C13 — verified non-colliding.

## Deliberately omitted

TVS diodes (Wilo reference has none; cable ≤ 3 m; field-side TVS on J14 re-examined and re-declined at final review 2026-08-05; the 24 V rail clamp D7 covers the internal FB-open fault, which is a different risk), switchable boost EN, pump mains switching, level shifters.

## Firmware implications (informational, out of scope)

LEDC ≤ 500 Hz inverted output; duty map 15–95 % = min→max, park at 0 % for stop, never command 7–12 %; feedback decode per the table above with per-state reaction times (80 % state lags up to 60 s); flow full-scale needs bench calibration; signal cable ≤ 3 m, unshielded (shielded/5 m fails the rise-time budget); new Zigbee attrs + Z2M expose for pump speed/flow/status later.

## Verification plan

1. ERC clean after edit.
2. **Full netlist diff** against pre-edit export (house rule since the J14/J2 label incident) — only the new nets/components may appear.
3. All new symbols carry hidden MPN + DigiKey properties; stock verified via DigiKey skill at implementation.
4. TPS61040 pinout/FB network cross-checked against the TI datasheet before wiring the symbol.

## Unresolved questions

1. Flow feedback full scale 1.4 vs 2.1 m³/h — Wilo docs conflict; settle by bench calibration (or Wilo Intec support).
2. Wilo factory article number for Para ST 15-7-50 iPWM2 not published — confirm from the physical pump label when it arrives.
3. ~~Exact inductor MPN + optional FB feedforward cap — settle at implementation against the TI datasheet.~~ **Resolved 2026-08-05:** L1 = LSXND4040TKL100MDG (Taiyo Yuden, Isat 1.3 A; renamed successor of NRS4018T100MDGJ/NR4018T100M per Taiyo Yuden's 2021 renumbering — same footprint, no fit/form/function change). Feedforward cap needed per TI SLVS413K §8.2.2.2 (prevents double-pulsing) — added as C13, 22 pF C0G 0402 across R10, TI's own reference-design value; bench-verify against double-pulsing once built.
