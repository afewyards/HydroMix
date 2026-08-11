# ValveController Schematic Review — 2026-08-11

**Project:** pcb/ValveController.kicad_sch (KiCad 9, single sheet) — 117 components, 101 nets, 50 unique parts, 100% MPN coverage
**Scope:** Schematic only (layout excluded at owner's request; PCB is mid-rework — 45 nets unrouted, new parts unplaced). Firmware excluded (adaptation to this board is planned separately).
**Analyzers run:** analyze_schematic.py (run 2026-08-11_1029), analyze_pcb.py --full (context only), cross_analysis.py, analyze_emc.py (out of scope, provisional), analyze_thermal.py (skipped itself — no extraction cache), lifecycle_audit.py (LCSC-only), kicad-cli sch erc, datasheet sync (~30 PDFs in pcb/datasheets/).
**Not run:** SPICE (no simulator installed), gerber analysis (no fab outputs), deep_review_gate.py (findings kept in reviewer format; citations spot-verified manually).
**Deep review:** four datasheet-driven subsystem reviews (power/pump, MCU/USB, AC-I/O, display) — full findings with citations in `analysis/deep_review_{power,mcu,acio,display}.json`.

## Overview

230 VAC-fed controller for an ESBE ARA661 230 V 3-point valve: fused mains (F2 500 mA time-lag) + S10K275 MOV → IRM-03-5 (5 V/600 mA) → TLV75733 3V3 (ESP32-C6-WROOM-1, IS31FL3730 + KWM-20881AGB 8×8 display) and TPS61040 boost → 24.3 V for the Wilo Para iPWM2 pump interface (CSD17313Q2 open-drain + 2.2k pull-up, J14). Two Z0103MN triac valve channels via MOC3063 zero-cross optos (J2), two CPC1035N PhotoMOS AUX pairs with HCPL-354 loop detection and SW3-switched 24 V wetting (J15), five DS18B20 1-Wire probe connectors (J8–J12), USB-C debug port (USB4145 + USBLC6-2).

## Verdict

**One blocker — two diodes mounted backwards (trivial fix). Everything else is in genuinely good shape.** All five custom/critical symbol pinouts (ESP32-C6 module, TPS61040, TLV75733, IS31FL3730, KWM-20881 matrix, plus USB4145/USBLC6/Z0103/MOC3063/CPC1035/HCPL-354) were verified pin-by-pin against manufacturer datasheets with zero mismatches. Fix the blocker, sweep the hygiene list, and the schematic is ready for the layout to catch up.

## Blocker

**B1. D20/D21 (SMF33A) are reversed on the pump interface — interface dead as drawn.**
Cathode (pin 1) to GND, anode (pin 2) to PUMP_PWM_OUT / PUMP_FB_IN. SMF33A is *unidirectional* ("color band denotes cathode" — Littelfuse SMF datasheet); as wired, the 24 V pull-up (R14 2.2k) forward-biases D20 permanently: pump PWM line clamped at ~0.7 V, ~10.3 mA wasted, R14 at ~233 mW (~93% of rating); D21 likewise clamps the 3V3-pulled feedback line so IO23 never reads high. Verified three independent ways: net topology, raw .kicad_sch pin-rotation geometry (D20 @ (355.0, 266.0) rot 90°, D21 @ (355.0, 280.0)), and PCB pad nets. **Fix: rotate both 180° (cathode to line, anode to GND).** Note the sibling parts are fine: D18/D19 are SMF51**CA** (bidirectional) and D8–D12 PEC3205**C**S are bidirectional (internal-circuit diagram, datasheet p.1) — orientation moot for those.

## High

**H1. TLV75733 (U2) thermal ceiling in a display+radio worst case** *(needs bench check)*.
WSON-6 RθJA 100.2 °C/W; full-on display at the IS31FL3730's power-on default 40 mA (~320 mA scanned) + 802.11b TX 382 mA ⇒ 1.19 W ⇒ Tj ≈ 144 °C > 125 °C at 25 °C ambient. Zigbee duty is far lower, so this is an OTA-over-WiFi + full-display corner. Mitigations: set the display current register low at init (firmware plan), and bench-measure U2 under worst case. (The analyzer's power budget misses U7 entirely.)

## Medium

- **M1. SMF33A clamp exceeds Q5/U6 30 V abs-max — accepted, part choice CONFIRMED correct.** VBR 36.7–40.6 V, VC up to 53 V: a hard transient can exceed CSD17313Q2/TPS61040 ratings before clamping. However the Wilo iPWM guide (fig. 11 spec table) allows the feedback line up to **UoH = 32 V in abnormal boiler operation**, so a standoff below 33 V would conduct in-spec — SMF33A is the right part; do NOT downsize. Residual fast-transient exposure is mitigated by the 100 Ω series resistors (R15/R17) and D7's 27 V zener on the rail; accept and document.
- **M2. D7 (BZT52B27, 500 mW) overdissipates in a sustained FB-open fault** — boost at current limit into the clamp ≈ 0.7–0.8 W. Fault energy is bounded (boost ILIM), but the clamp is sacrificial, not continuous. Accept (document) or upsize (SMBJ-class / SOD-123F 1 W+).
- **M3. 5 V budget tight** — worst case ≈ 530 mA of IRM-03-5's 600 mA (WiFi TX 382 + display ~80 + boost ~67). Hiccup-mode OCP means sustained overload = restart loop. Bench-measure real peaks; 121 µF of 3V3 bulk rides ms-scale bursts.
- **M4. U7 C_FILT (pin 5) hard-grounded** — datasheet calls the cap "required" (0.1 µF, Fig. 1); likely benign with audio disabled (A_EN=0, IN grounded) but unqualified. Add 0.1 µF or bench-verify.
- **M5. GPIO15 (JTAG-select strap) floating** — datasheet §4.4 explicitly forbids high-Z on this strap. Inert today (eFuse default ignores it); add a pull for insurance.
- **M6. AUX LED drive margin thin** (found independently by two reviewers) — 680 Ω gives 1.82 mA at the worst corner vs CPC1035N's 2 mA guaranteed turn-on, and Clare recommends 10 mA above 60 °C. Change RL3/RL4 to 470–560 Ω.
- **M7. No local VBUS bypass at J13/U10** (analyzer UC-001 confirmed) — nearest caps sit behind D5; ST's USBLC6 app note wants a local bypass for clamp effectiveness. Add 1–4.7 µF at U10.5/J13. Debug-only port softens urgency.
- **M8. AUX detect ripple in external-24VAC mode** — 100 Hz pulsing vs 15.9 Hz RC corner: IO20/21 see ripple, not DC. Firmware must integrate/debounce (DC-wetting mode via SW3 is clean).
- **M9. 3V3 pin on J8–J12 is unprotected** — probe power pin has no series element or clamp (data pin has both); one field-wiring short resets the whole controller. Add per-connector (or shared) series R / polyfuse on the probe 3V3 feed.

## Low / hygiene

- U7 SDB hard to 3V3 — a wedged display driver can only be reset by power-cycling 3V3; a pull-up + GPIO would restore the datasheet's SDB reset feature. (Optional.)
- C1 100 µF/6.3 V on 3V3 = 52% of rating — just over the 50% X7R guideline; fine functionally (C5/C6/C15 in parallel).
- SW3 contact spec 24 VDC vs 24.3 V rail (+1.3%) — negligible, noted for completeness.
- HCPL-354 wetting current 0.64 mA below the 1 mA characterized CTR point — ~4× margin on the computed floor; scope at temperature extremes.
- SW3-ON + external device combination can leak ≤ ~1.6 mA into external sense inputs — installation/manual note, not a defect.
- J2/J3 "250 V" property vs 253 V (230 V+10%) — confirm Phoenix's working-voltage definition (SPT 2,5 family is standard 230 V kit; almost certainly fine).
- NC-flag cleanup: add flags on IO11 + RXD0; delete stale flags sitting on connected pins IO4, IO5, RS8.2.
- U5 symbol types its 10 GND pins as power-out → 11 spurious ERC errors; retype power_in/passive.
- Delete the 0.0127 mm dangling wire stub at (349.25, 97.79) [schematic units mm×0.01 per ERC: @(3.4925, 0.9779)].
- 16 off-grid endpoints in the J8–J12 / D18–D21 / U10 region — grid-snap sweep.
- D20/D21 use Device:D_Zener; switch to Device:D_TVS when rotating (fixes the analyzer's "J14 unprotected" false negative too).
- Missing PWR_FLAGs on 24V_FB/24V_SW/VBUS (analyzer RS-001) — ERC hygiene only.
- No test points anywhere; given that USB attach resets the board and masks faults (documented debug history), add UART0 TX/RX/GND pads or a 3-pin header — TXD0/RXD0 are free.

## Info / design questions

- **R13 pulls Q5's gate UP to 3V3 — RESOLVED: deliberate and correct.** Pump line actively held low whenever the MCU isn't driving (boot, reset, crash). Per the Wilo iPWM Technical Guide V1.0 (now in pcb/datasheets/) iPWM2 profile: <7 % duty = stop, wire-break = STOP — so line-low = 0 % = STOP. The 2026-08-05 interface spec documents exactly this rationale ("reset/crash/unpowered = pump stopped"). Opposite polarity from the other outputs' pulldowns is intentional safe-state logic, not an error.
- CC1/CC2 have no dedicated ESD device — acceptable for an internal debug port.
- Display power-on defaults are safe: 40 mA scanned ≈ 1/8 duty vs 100 mA/1-10-duty matrix peak rating (~2.5× margin); explicit CS-register init still recommended.
- Display pixel remap for the firmware plan (driver→matrix): R1→ROW2, R2→ROW4, R3→ROW1, R4→ROW3, R5→ROW6, R6→ROW8, R7→ROW7, R8→ROW5; C1→COL4, C2→COL6, C3→COL1, C4→COL7, C5→COL8, C6→COL2, C7→COL3, C8→COL5.

## Verified correct (datasheet-cited highlights)

- **ESP32-C6 module:** all 29 pins verified against Espressif Table 3-1 — no numbering errors; EN RC = Espressif's exact recommended 10k/1µF (~278× margin on tSTBL); BOOT strap textbook, C8 settles 1 ms before the 3 ms sampling window; IO8 strap pulled up, sole load; USB D+/D- polarity correct (IO13=D+/IO12=D-); no auto-reset circuit needed with native USB-JTAG — correct.
- **USB:** USB4145 pin map exact; USBLC6-2 channels correct (D+ on 1&6, D- on 3&4, flow-through); CC pulldowns 5.1k one-per-pin; SBU correctly NC.
- **Boost:** TPS61040 pinout exact; Vout = 1.233 × (1+249/13.3) ≈ 24.31 V; R10/R11 in TI's recommended ranges; C13 feedforward correctly across R10; L1 is literally TI's recommended inductor (Isat 2.9× ILIM); D6 60 V/1 A ample; C9/C10/C11 meet TI minimums.
- **LDO:** TLV75733 pinout exact; EN-to-IN is TI's documented always-on practice; output capacitance ≫ stability minimum.
- **Mains front-end:** F2 = 500 mA time-lag NANO2 (tolerates IRM-03 20 A inrush, ~12× line-current margin); S10K275 correct for 230 VAC; C18/CS1/CS2 genuinely X1/Y2-certified; D4/D5 OR-ing verified by pin names, no backfeed.
- **Triac channels:** Z0103MN pinout verified (onsemi second source); MOC3063 app-note topology exact; LED drive 6.9–11.1 mA vs 5 mA IFT; gate current 3–11× IGT, fault-bounded under MOC ITSM; snubbers correctly across each triac; boot-safe pulldowns everywhere.
- **AUX/detect:** CPC1035N pinout exact; SMF51CA correctly bidirectional across floating pairs; HCPL-354 orientation correct (E→GND, C→DET); detection logic traced end-to-end (HIGH=closed by current diversion — see deep_review_acio.json explainer).
- **1-Wire probes:** all five channels identical and correctly mapped (J8→IO0, J9→IO1, J10→IO10, J11→IO5, J12→IO4); 4.7k pull-ups per DS18B20 convention; PEC3205CS 5 V standoff / ≤20 pF / bidirectional — correct choice; powered (non-parasite) mode.
- **Display:** IS31FL3730 all 25 pins exact; matrix symbol 16/16 against Luckylight series pinout; row-anode ↔ source-side polarity correct; I2C 0x60, no conflicts; C14+C15 decoupling per datasheet Fig. 1.
- **Isolation inventory for the layout stage:** MOC3063 5 kVrms, HCPL-354 3.75 kVrms, CPC1035N 1.5 kVrms, IRM-03 transformer — the only mains↔logic crossings; netlist shows no accidental galvanic path.

## Analyzer false positives (triaged, with corrected math)

| Analyzer claim | Reality |
|---|---|
| PS-001 error: circular enable U6→U6 | EN tied to 5 V (=VIN), not its own output |
| U6 Vout = 11.83 V | Heuristic used 0.6 V Vref; real VFB 1.233 V → 24.3 V |
| VD-003: R11 at 43 mW | Only VFB across R11 → 0.11 mW (380× lower) |
| PS-001: U2 PG unknown | TLV75733P has no PG pin in any package |
| U2/U6 "missing capacitors" / "missing decoupling on FB/SW" | All required caps present; FB/SW pins don't take decoupling |
| EP-AUD: J15/J2/J3/J14 "no ESD" | J15 has SMF51CA differential pairs; J2/J3 have MOV+snubbers (correct class for mains); J14 has D20/D21 (reversed, and invisible to the detector due to D_Zener lib_id) |
| MU-1: TEMP_SOURCE on non-ADC GPIO10 | Probes are DS18B20 1-Wire (title block); no ADC involved |
| AC-2: no filter caps on temp inputs | Correct for 1-Wire — caps would hurt the bus |
| NT-001 H3/H4; RS-001 rails; DS-002 | Logo/hole symbols; missing PWR_FLAGs only; resolved by datasheet sync |
| kicad-cli: 85 lib warnings, 11 U5 pin_to_pin errors | Environment noise + symbol pin-type hygiene (see Low list) |

## Not performed / review limits

- **SPICE:** no simulator installed (`brew install ngspice` would enable boost/filter verification).
- **Thermal analyzer:** skipped itself (no extraction cache); H1 covers the dominant risk manually.
- **Lifecycle:** LCSC-only audit returns "unknown" for all 45 MPNs; needs DigiKey/Mouser keys for real EOL status.
- **Layout / EMC / cross-domain:** excluded per owner request; re-run after routing completes (current EMC output reflects the unfinished layout).
- **Deep-review gate script:** not run (reviewer-format JSONs); key citations spot-verified by the orchestrator instead (D20/D21 three ways, PEC3205CS directionality, 1-Wire reclassification, R13 pull-up).
- **KWM-20881AGB exact datasheet** unpublished — AVA sibling used as pinout proxy (16/16 match; AGB current ratings may differ slightly).
- **Wilo Para iPWM2 guide** was absent during the agent pass (R13/R14 checks initially rested on the approved board spec); the guide has since been fetched to `pcb/datasheets/Wilo_iPWM_Technical_Guide_V1.0_pump_J14_interface.pdf` and closes the R13 polarity question (see Info section). R14=2.2k is Wilo's recommended pull-up (valid 1–2.5 k).

## Fixes applied 2026-08-11 (same day, scripted with netlist-diff gate)

Backup: `pcb/ValveController.kicad_sch.pre-review-fixes.bak`. Netlist diff verified to contain exactly the intended changes; ERC errors 17→4 (only the H3/H4 logo-pin noise remains); analyzer re-run confirms all changes. **Not committed to git. If the project is open in KiCad, use File → Revert before continuing work.** The PCB now needs F8 (Update from Schematic): place C19 (at U10/J13), C20 (at U7), R39, R40 + the D20/D21 pad-net swap.

- **B1 FIXED**: D20/D21 rotated 180° — cathodes now on PUMP_PWM_OUT/PUMP_FB_IN, anodes on GND.
- **M4 FIXED**: U7 pin 5 lifted from GND; C20 100 nF (GRM155R71C104KA88D) added on new net C_FILT.
- **M5 FIXED**: NC flag removed from IO15; R39 10 k pull-up to 3V3 added (strapped high like the old board).
- **M6 FIXED**: RL3/RL4 680 → 470 Ω (RC0402FR-07470RL); worst-corner LED current now 2.64 mA vs 2 mA spec.
- **M7 FIXED**: C19 1 µF (CL05A105KP5NNNC) added directly on VBUS at U10/J13.
- **M9 FIXED**: J8–J12 pin 3 moved to new net 3V3_PROBES behind R40 100 Ω 1206 (ERJ-P08J101V) — a field short now draws ~33 mA instead of dropping the rail. (The schematic analyzer's new RS-001 info on 3V3_PROBES is the expected consequence; native ERC is clean.)
- **Hygiene FIXED**: NC flags added on IO11 + RXD0 (correct pin-12 position re-derived — the review's original coordinate was pin 11); stale NC flags on IO4/IO5 deleted; U5 symbol GND pins retyped power-out → passive (11 ERC errors gone); 1.27 mm dangling wire stub at H4 deleted.
- **Deliberately not changed**: SMF33A stays (Wilo UoH 32 V), D_Zener glyph stays, D7 stays (documented sacrificial), PWR_FLAGs not added, off-grid endpoints left for the GUI, SDB rework not done, H3/H4 logo-pin ERC noise left.
- Follow-up in progress: DigiKey URL properties for the new/changed parts (RL3/RL4's old 680R product link was stale).

## Recommended action order

1. Rotate D20 + D21 (blocker). Keep SMF33A (33 V standoff required by Wilo UoH=32 V abnormal — M1) and keep the polarized D_Zener glyph (semantically correct for a unidirectional TVS; the analyzer's EP-AUD J14 false-negative is an analyzer limitation, not worth a misleading bidirectional symbol).
2. RL3/RL4 680 → 470–560 Ω (M6). Add 1 µF at VBUS/U10 (M7). Add 0.1 µF on C_FILT or bench-clear it (M4). Pull on GPIO15 (M5). Series R/polyfuse on probe-connector 3V3 (M9).
3. Hygiene sweep: NC flags, stale NC removals, U5 GND pin types, PWR_FLAGs, wire stub, off-grid endpoints, UART test pads.
4. Bench checklist for first prototype: U2 thermals under OTA+display (H1), 5 V peak draw (M3), AUX opto margins at temperature (M6/AC-5), DET ripple behavior for firmware debounce design (M8).
5. Decide on D7 sizing or document it as sacrificial (M2).
