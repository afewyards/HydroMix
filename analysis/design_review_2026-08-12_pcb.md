# ValveController PCB Layout Review — 2026-08-12

**Project:** pcb/ValveController.kicad_pcb (KiCad 9, 4-layer, 94.5 × 99.75 × 1.6 mm) — 124 footprints (102 SMD / 15 THT + art), 99 copper nets, 585 vias, routing 100 % complete
**Scope:** Layout-focused full review — the counterpart to the 2026-08-11 schematic review, which explicitly deferred the layout ("re-run after routing completes"). Schematic re-analyzed for regressions only.
**Assembly context (owner-confirmed 2026-08-12):** hand reflow, components placed under microscope — no machine assembly.
**Analyzers run:** analyze_schematic.py, analyze_pcb.py --full --proximity, cross_analysis.py, analyze_emc.py, analyze_gerbers.py (fresh export, same-minute as board save), kicad-cli pcb drc --severity-all (with project .kicad_dru), lifecycle_audit.py. Runs `2026-08-12_1858` / `2026-08-12_1858-2`.
**Not run:** SPICE (no simulator installed), analyze_thermal.py produced no data (no datasheet extraction cache — skipped itself), deep_review_gate.py (layout findings kept in report format; the per-IC datasheet deep review was completed 2026-08-11 and the schematic is unchanged apart from the verified fix sweep).

## Overview

Same system as the schematic review: 230 VAC-fed ESBE ARA661 valve controller (IRM-03-5 → TLV75733 3V3 → ESP32-C6; TPS61040 24 V boost for the Wilo iPWM2 pump; two triac valve channels; two AUX PhotoMOS channels; five DS18B20 probes; USB-C debug; 8×8 display). The board is now fully routed on 4 layers: **In1 = solid GND plane, In2 = 3V3 plane (86 % fill)**, mains section in the top band with a deliberate plane void, logic in the bottom two-thirds.

## Verdict

**One pre-fab blocker: G1's copper logo is parked 55 mm off the board and was exported into the gerbers.** Fix that (plus two cheap thermal-via items on U2 and U7) and this layout is ready to order. The safety-critical part — mains isolation — is genuinely well executed: a documented custom rule file enforces 6.4 mm reinforced HV↔LV spacing through complete netclass coverage, native DRC passes it with zero clearance violations, and a raw-file sample of the In1 plane confirms the mains region contains no logic copper at all. All six fixes from the 2026-08-11 schematic review are confirmed present in copper.

## Critical Findings

| Severity | Issue | Detail |
|---|---|---|
| CRITICAL (pre-fab) | **B1. G1 off-board copper is in the exported gerbers** | G1 (`KleistLabs_Logo_10mm_Copper`) sits at (5, 5) — the outline starts at x = 60.5. F.Cu gerber extents are 154.6 mm vs 94.5 mm Edge.Cuts (gerber analyzer GR-002: "width varies by 60.9 mm"). The 10 mm footprint variant also no longer exists in Kleist2.pretty (DRC `lib_footprint_issues`); the on-board art is now the 8 mm logo + QR pair. **Fix: delete G1 from board and schematic (or place it properly), refill zones, re-export gerbers.** Raw-file + gerber verified. |
| WARNING | **W1. U2 (TLV75733) lost its thermal vias** | The board copy of `WSON-6-1EP…_ThermalVias` has 10 pads / 0 thru-hole; the library copy has 12 with 2 via pads (this is what the two DRC `lib_footprint_mismatch` hits on WSON-6 are). U6's copy kept its 2 vias; U2's has none, so its EP reaches GND only through the F.Cu pour. Last review's H1 (Tj ≈ 144 °C worst case at RθJA 100.2 °C/W) assumed a JEDEC board *with* via stitching — the real θJA is now worse than the H1 math. **Fix: re-sync the footprint from the library or add 2–4 × 0.3 mm vias EP → In1 GND.** Raw-file verified (pad counts), datasheet context from 2026-08-11 review. |
| WARNING | **W2. U7 (IS31FL3730) QFN-24 EP has 0 thermal vias** | Analyzer TV-001 (0/5 minimum). EP is on GND but connects only via F.Cu. U7 sinks the scanned display current (~320 mA at power-on defaults). **Fix: add 4–5 × 0.3 mm vias in the 2.5 × 2.5 mm EP.** Analyzer-derived, EP net raw-file verified. |

Hand-reflow note for W1/W2: open 0.3 mm vias in an EP wick some paste during reflow — expected and acceptable at this size. Apply slightly extra paste on the EP, and treat solder appearing in the via barrels as a useful visual confirmation the EP actually wetted (hard to inspect otherwise). Tenting them from the back keeps solder bumps off B.Cu; don't bother filling/plugging.

## Previous Review Delta (vs design_review_2026-08-11.md)

| Status | Count | Items |
|---|---|---|
| Fixed, now confirmed in copper | 6 | B1 D20/D21 orientation; M4 C20; M5 R39; M6 RL3/RL4 = 470 Ω; M7 C19; M9 R40 + 3V3_PROBES |
| Still open | 2 | Test points 0/98 nets (TE-001 — UART pads were recommended given the USB-reset debug trap); U2 thermal margin H1 (now aggravated → W1) |
| New (this review) | 5 | B1 gerber contamination; W1/W2 thermal vias; REF** annotation; silkscreen trio |
| Accepted/bench items unchanged | — | M1/M2 (SMF33A + D7 sizing documented), M3 (5 V budget bench check), M8 (AUX ripple firmware debounce), H1 bench measurement |

Pad-net checks this pass: **D20 = K→PUMP_PWM_OUT / A→GND, D21 = K→PUMP_FB_IN / A→GND** — last review's blocker is closed at the copper level (raw-file verified).

Schematic re-run shows no regressions: remaining error/warnings are the already-triaged false positives (PS-001 "circular enable" on U6, VD-003 R11 power math) plus PWR_FLAG/single-pin hygiene. MPN coverage still 100 %, `pcb/datasheets/` still synced (~30 PDFs).

## Mains Isolation (verified three ways)

The most safety-critical aspect of this board, and it holds up:

1. **Netclass coverage is complete.** Every net touching the mains section is in HV (`MAINS_L`, `MAINS_L_F`, `MAINS_N`, `PE`, `VALVE_OPEN`, `VALVE_CLOSE`) or HV_LOCAL (`Q1_G`, `Q2_G`, `RS1_m`, `RS2_m`, `U3_o6`, `U4_o6`, triac NC pads) — checked by enumerating pad nets of F2, RV1, C18, CS1/CS2, J2/J3, Q1/Q2, U3/U4, U1 against the `.kicad_pro` patterns. No orphan mains net escapes the rules.
2. **The `.kicad_dru` enforces reinforced spacing.** `hv_lv_isolation` requires 6.4 mm HV↔LV clearance (last rule = highest precedence), with documented rationale: MOC3063 body = 7.47 mm, placement = 8.78 mm. IEC 60664-1 at 250 Vrms / OVC II / PD2 asks ~5.0 mm reinforced creepage — 6.4 mm copper spacing exceeds it. Sub-rules cover N↔HV 2.0 mm, across-the-fuse 2.0 mm (F2 gap 4.5 mm), and valve-to-valve 2.0 mm.
3. **DRC + raw-file confirm.** kicad-cli DRC with these rules: **0 clearance violations** (16 warnings, all library-sync/silk). Point-in-polygon sampling of the In1 GND fill: **0/117 sample points in the mains band** (85–150, 28–48) vs 92 % coverage in logic regions — the isolation void is real, not a bounding-box illusion.

The only mains↔logic crossings remain the certified parts (MOC3063, HCPL-354, CPC1035N, IRM-03, C18/CS1/CS2 Y-caps), matching the schematic review's isolation inventory.

## PCB Verification (analyzer vs raw file)

- **Footprint count:** 124 in analyzer = 124 in raw file. Schematic↔PCB: only PCB-side extras are the two unannotated art footprints (`REF**` 8 mm logo + `REF**` QR at (109–118, 74.7)); no schematic component is missing from the board (cross_analysis XV-001 undercounts this as 1 because both share the ref `REF**`).
- **Board outline:** 94.5 × 99.75 mm from Edge.Cuts, matches gerber extents.
- **Pad-net spot checks:** U1 (mains pins 1/3, GND 14, PSU_5V 16), U2/U6 (WSON pinout incl. EP=GND), U7 EP=GND, D20/D21 as above — all consistent with the datasheet-verified schematic pass of 2026-08-11.
- **Zone fills are current:** all 14 zones `is_filled`, plausible ratios; gerbers exported the same minute as the board save.

## USB (J13 → U10 → U5)

- **Flow-through routing verified at copper level:** USB_D− islands are {J13.A7/B7, U10.3} + {U10.4, U5.13} — signal enters one USBLC6 pad, exits the other, exactly ST's intended flow-through. D+ equivalent per KiCad's own connectivity. The two DRC `unconnected_items` exclusions cover the USBLC6-2's internal pin 1↔6 / 3↔4 bonds — legitimate exclusions, already datasheet-verified 2026-08-11. (The plugin's island math splits D+ into 3 islands — arc-handling gap, disproven by DRC.)
- **Diff pair delta 0.46 mm** (D+ 26.05 mm / D− 25.59 mm) — far inside full-speed USB tolerance. D+ takes 2 vias, D− none; irrelevant at 12 Mb/s. No impedance control needed for FS.
- **ESD path:** C19 1 µF is 2.92 mm from U10 (esd_bypass ≤ 3 mm ✓), nearest GND via 2.0 mm from U10 — EMC ES-002 "no ground via near U10" is a false positive at any reasonable threshold.

## Power & Ground

- **Planes:** In1 solid GND under all logic (92 % sampled coverage incl. display corridor); In2 3V3 at 86 % fill; 464 GND stitching vias.
- **Trace widths:** mains 2.0 mm (HV class floor 1.2 mm) — huge margin at < 100 mA valve/PSU currents; PSU_5V 0.8 mm; 3V3_PROBES uniformly 0.8 mm (the M9 fix routed generously); 24 V / 5 V / VBUS 0.3–0.8 mm with plane assist — all comfortable per IPC-2221 at this board's loads. Minimum 0.2 mm segments on 3V3/GND are short pad necks; bulk distribution is by plane.
- **Decoupling placement (all within 3 mm, same side):** U5←C7 @ 2.4 mm, U2←C5 @ 2.1 mm, U6←C9 @ 1.5 mm, U7←C14 @ 2.85 mm + C20, U10←C19 @ 2.9 mm. EMC DC-001/DC-002 complaints are false positives: U8/U9 are HCPL-354 optos with no VCC pin; U10 has C19; U1 is a self-contained AC/DC module feeding bulk caps downstream.
- **VBUS PS-002 "2 islands":** zone + trace topology, electrically continuous (DRC clean) — benign.

## Thermal

- analyze_thermal.py produced no junction estimates (no extraction cache) — the quantitative reference remains H1 in the 2026-08-11 review. Layout reality check this pass: **U2 0 EP vias (W1), U7 0 EP vias (W2), U6 2 vias (adequate for ~0.1–0.2 W boost losses)**. U2 is the one to fix before the H1 bench check, otherwise the bench numbers will be measured against a worse-than-modeled board.
- Tombstoning: 32 × 0402 flagged for pad thermal asymmetry (info) — moot: hand reflow heats gradually and misbehaving parts get nudged on the spot.

## EMC (pre-compliance; risk score 11.5 before triage)

- **GP-001 (21 findings, 16 of them "0 % plane coverage" errors on DISP_R1-8/C1-8): FALSE POSITIVE.** Raw-file point-in-polygon sampling shows the In1 GND plane covers the U7→DS1 display corridor at 92 % — same as under the ESP32. The EMC tool mis-reads the multi-layer zone's per-layer fills. The remaining GP-001s (U3_A/U4_A 67–80 %, PUMP_* 70–78 %, 24V 87.5 %) are edges of the deliberate mains void / board-edge connector approaches — expected geometry, accept.
- **RP-001** missing stitching vias at DISP_R1-3 F↔B transitions (reference swaps GND↔3V3 plane): real but low-risk at display-scan speeds. Optional: a GND via + 3V3–GND cap near the display corridor transitions.
- **IO-001 "no EMC filtering" on J2/J3/J8-J12/J14/J15:** every port already carries deliberate protection (mains: MOV + snubbers; probes: R40 + PEC3205CS series-R/TVS — shunt caps would break 1-Wire; pump: 100 Ω + SMF33A; AUX: SMF51CA). The analyzer wants L/C structures it can pattern-match. Residual truth: long probe/pump cables are the radiated-emissions antennas — if pre-compliance fails 30–230 MHz, the fix is CM chokes at J8–J12/J14, not board changes. Keep as a contingency, not a change.
- **SW-001** U6 boost harmonics in 30–88 MHz: inherent to TPS61040; the hot loop is tight (C9 1.5 mm from U6, 2 EP vias). Note for the pre-compliance scan.
- **DC-003** C14/C15 far from via — minor, absorbed by W2's via work if done nearby.

## Manufacturing / DFM

- **DFM tier:** analyzer says "advanced" solely for annular ring — but only **2 of 585 vias** are 0.5/0.3 mm (0.1 mm ring; they're U6's footprint-embedded thermal vias). The population is 526 × 0.6/0.3 and 57 × 0.8/0.4. Common standard-tier fab capability covers 0.5/0.3; verify against your fab's standard capability rather than re-designing. Min track 0.2 mm, min drill 0.3 mm — standard everywhere.
- **FD-001 no fiducials** (error-rated): dismissed — owner confirmed hand placement under microscope; fiducials serve pick-and-place vision only.
- **TE-001 test points 0/98:** still open from last review; the UART TX/RX/GND pads remain the cheapest insurance given that USB attach resets the board and masks faults.
- **Gerbers:** all 11 layers + PTH/NPTH present, drill classification clean (585 vias, 93 component holes), alignment fails **only** because of B1's off-board logo. GR-004 paste/copper ratio 32 % is the THT + art copper skew — benign.
- **Silkscreen trio (cosmetic):** '24AC SENSE' text overlaps SW3's outline; U1 outline clipped by a mask polygon; Q4 footprint text self-overlap.
- **REF\*\* art footprints:** give the 8 mm logo and QR real references (G2/G3) so BOM/DRC tooling stops tripping on `REF**`, and confirm their ref texts are hidden.
- **Ordering:** 4-layer, 1.6 mm, copper finish not set in board setup (choose at order time). For hand reflow: **ENIG recommended** — flat pads matter for seating the QFN-24 0.5 mm and WSON parts under the microscope (HASL domes make them swim); **order the stencil** — syringe-pasting a 0.5 mm-pitch QFN and two WSON EPs is the hard way, a frameless stencil is a few euros with the board order.

## Component Lifecycle

Audit ran (45 MPNs) but every distributor returned "unknown" — same as 2026-08-11; the credentials/API path still doesn't produce lifecycle data. No EOL/NRND information available. Review gap, unchanged.

## False Positives / Reviewer Overrides (this pass)

| Claim | Reality |
|---|---|
| EMC GP-001: 16 display nets "0 % plane coverage" | In1 GND solid under display corridor (92 % sampled) — tool mis-parses multi-layer zone fills |
| EMC DC-002: no decoupling near U8/U10/U1 | U8/U9 are optos without VCC; U10 has C19 @ 2.9 mm; U1 is an AC/DC module |
| EMC ES-002: no GND via near U10 | GND via 2.0 mm away |
| DRC unconnected: U10 pads 1↔6, 3↔4 | USBLC6-2 internal bonds; flow-through routing verified; exclusions correct |
| PCB PM-002 error: G1 55 mm overhang | Not a placement error but a parked footprint → escalated to blocker B1 (gerber contamination) |
| PM-002 info: U5 −1.95 mm overhang | ESP32-C6 antenna correctly hangs off the board edge |
| DFM-001/002: annular ring below Class 2 | Affects exactly 2 embedded thermal vias; fab-capability check, not a redesign |
| Sch PS-001/VD-003 | Same false positives triaged 2026-08-11 |

## Positive Findings

1. Reinforced mains isolation enforced by rule, not hope: complete HV netclass coverage + 6.4 mm `.kicad_dru` constraint + clean DRC + verified plane void. The rule file's comments (pad-gap floors, MOC body limit) show the constraints were engineered, not defaulted.
2. All six schematic-review fixes confirmed in copper, including the D20/D21 blocker.
3. 100 % routing completion, zero DRC electrical violations, schematic↔PCB sync clean.
4. Tight decoupling discipline — every active IC's closest cap ≤ 3 mm, same side.
5. Solid In1 GND with 464 stitching vias; dedicated 3V3 plane; USB flow-through done right; antenna correctly overhanging the edge.
6. Sensible width hierarchy per netclass (2.0 mm mains, 1.2 mm HV floor, 0.8 mm probe power).

## Not Performed / Review Limits

- **SPICE:** no simulator installed (`brew install ngspice` would enable it) — unchanged from last review.
- **Thermal analyzer:** no quantitative Tj output (no datasheet extraction cache); relied on 2026-08-11 manual H1 analysis + this pass's via audit.
- **Lifecycle:** all 45 MPNs "unknown" — no usable EOL data.
- **Deep-review gate:** layout findings kept in report format; per-IC datasheet pass not regenerated (schematic unchanged since the verified 2026-08-11 pass).
- **Impedance control:** not analyzed beyond noting FS USB doesn't need it.
- **Creepage via slots/coatings:** review measured copper spacing in 2D; no slot/coating analysis (none appear intended).

## Fixes applied 2026-08-12 (same day, verified 19:36–19:43)

- **B1 FIXED:** off-board G1 (stale 10 mm logo) deleted; on-board art annotated G1 (8 mm logo) + G2 (QR), `REF**` gone. Gerbers + drills re-exported 19:43 (`kicad-cli --board-plot-params`); analyzer confirms alignment clean, F.Cu extents 93.7 × 98.85 mm inside the 94.5 × 99.75 mm outline.
- **W1 FIXED:** 2 × 0.5/0.3 mm free vias in U2's EP (matches the library's 2-via layout).
- **W2 FIXED:** 4 × 0.5/0.3 mm free vias in U7's EP. Drill count 585 → 591 confirms exactly the 6 new vias.
- DRC re-run: 0 electrical violations, warnings 16 → 14 (G1 library noise gone; remaining = WSON/terminal-block lib-sync + the cosmetic silk trio, all accepted).
- **Deliberately not changed:** silk trio, test points, fiducials (hand assembly), stitching vias at DISP transitions.

## Recommended Action Order

1. **B1:** delete or place G1, refill zones, re-export gerbers, confirm gerber extents = 94.5 × 99.75 mm.
2. **W1/W2:** restore U2's 2 thermal vias (library re-sync fixes the DRC mismatch too) and add 4–5 vias under U7's EP.
3. Add UART test pads (TE-001, carried over) while touching the board.
4. Hygiene: annotate the two `REF**` art footprints, silk trio, optionally 2 stitching vias at the DISP_R1-3 transitions.
5. Unchanged bench checklist from 2026-08-11: U2 thermals under OTA+display (after W1), 5 V peak draw, AUX opto margins, DET ripple.
