# Strip status LED · USB-C → USB4145 — Design Spec

**Date:** 2026-08-06
**Scope:** `pcb/ValveController.kicad_sch`, `pcb/ValveController.kicad_pcb`, `tools/`, docs. Firmware untouched.
**Context:** The 8×8 matrix (DS1 + U7 IS31FL3730, merged 2026-08-05) supersedes the single status LED. Separately, J13 moves to a connector that can actually be hand-soldered.

Both changes land while the PCB is unrouted (`HEAD` has 331 segments / 82 vias; working tree has 0), so neither costs a rip-up.

## 1. Status LED removal

D1 (0402 LED, IO15 active-low) is redundant now that the matrix can render state. **Both D1 and R4 go**, and IO15 is left unconnected.

### 1.1 Why R4 goes too

R4 had a second job: the firmware spec (2026-07-13, §8.1) noted it also held the GPIO15 JTAG-source strap high. Per **ESP32-C6 Series Datasheet v1.5**:

- **Table 3-1** — GPIO15 default configuration is **Floating**. Unlike GPIO9, it has no internal weak pull-up.
- **§3.4** — *"This pin does not have any internal pull resistors and the strapping value must be controlled by the external circuit that cannot be in a high impedance state."*
- **Table 3-7** — with the factory-default eFuses (`DIS_PAD_JTAG=0`, `DIS_USB_JTAG=0`, `JTAG_SEL_ENABLE=0`) GPIO15 is **Ignored** and the JTAG source is the USB Serial/JTAG controller.

§3.4's warning describes the case where the strap is *active*. Table 3-7's bolded default row governs this board: the strapping latch still samples GPIO15 at reset, but nothing reads the value. A floating pin there changes nothing.

Keeping a pull-up would insure exactly one scenario — burning `JTAG_SEL_ENABLE`, after which a floating GPIO15 could divert JTAG to the MTDI/MTCK/MTMS/MTDO pads and present as dead hardware. That is a deliberate, knowing act, not an accident.

Against that, GPIO15 is scarce: the display/AUX design left IO4, IO5, IO8, IO16 free, and IO4/IO5/IO8 are all strapping pins. GPIO15 and GPIO16 are the only clean non-strapping spares. A 10 k pull-up would not block reuse, but it would make the pin idle high — awkward for an open-drain bus, a chip select, or an active-high enable that must be off at boot. An unencumbered spare is worth more than insurance against an eFuse burn that is not planned.

**If `JTAG_SEL_ENABLE` is ever burned, GPIO15 must be pulled high externally.** Recorded in the firmware GPIO map.

### 1.2 IO15 disposition

No net. A `no_connect` flag on U5's IO15 pin at `(134.62, 71.12)`, matching the existing no-connect at `(134.62, 66.04)` in the same pin column. In the PCB the pad takes the board's standard unconnected name, `unconnected-(U5-IO15-Pad23)`.

### 1.3 Schematic edits

Delete the whole branch:

| Element | Location |
|---|---|
| Symbol `D1` | `(182.88, 256.54)`, uuid `0385bbdc…` |
| Symbol `R4` | `(182.88, 241.3)`, uuid `4c9767a4…` |
| Wire | `(182.88, 261.62)`–`(182.88, 266.7)` |
| Wire | `(182.88, 251.46)`–`(179.07, 251.46)` |
| Wire | `(182.88, 231.14)`–`(182.88, 236.22)` |
| Wire | `(182.88, 246.38)`–`(182.88, 248.92)` |
| Wire | `(179.07, 248.92)`–`(182.88, 248.92)` |
| Global label `LED_A` | `(182.88, 266.7)` and `(179.07, 248.92)` |
| Global label `STAT_LED` | `(179.07, 251.46)` |
| Global label `3V3` | `(182.88, 231.14)` |

Replace: the `STAT_LED` global label at `(134.62, 71.12)` sits directly on U5's IO15 pin with no stub wire → becomes `(no_connect (at 134.62 71.12))`.

Nets `STAT_LED` and `LED_A` cease to exist.

### 1.4 PCB edits

Delete the `D1` footprint (`LED_SMD:LED_0402_1005Metric`, ~`(91.8, 108.075)`) and the `R4` footprint (`Resistor_SMD:R_0402_1005Metric`, ~`(90.7, 108.075)`). Nothing on either was routed. U5 pad 23 takes `unconnected-(U5-IO15-Pad23)`.

### 1.5 Documentation

- `README.md:51` — drop "status LED (**active-low**)" from the I/O row.
- `firmware/README.md` — remove the Status LED row from the GPIO map; replace the "LED legend" subsection with gestures only.
- `firmware/BENCH_CHECKLIST.md:108` — step 1.5 becomes a USB-console liveness check instead of an LED watch.

### 1.6 Firmware deliberately unchanged

`ui.c` (`PIN_LED`, `led_task`, `blink`) and `app_main.c:17` (`PIN_STAT_LED`) stay. They will be replaced wholesale by the IS31FL3730 display work rather than churned twice.

Consequence until then: `led_task` drives IO15, which now goes nowhere — no LED, no resistor, no net. Inert. Strapping values are latched at Chip Reset and held until power-down (datasheet §3), so runtime toggling cannot affect JTAG selection even if `JTAG_SEL_ENABLE` were burned. Bring-up in the interim relies on the USB-Serial-JTAG console, which reports more than the LED did.

## 2. J13 → GCT USB4145-03-0170-C

### 2.1 Rationale

**Hand-solderability.** On the USB4115-03-C the shell wall stands in front of the contact row, so the pads cannot be reached with an iron. The USB4145 keeps all 16 contacts clear on the surface. This is a build-process decision, not a manufacturability rescue — both parts are vertical and architecturally identical (SMT contacts, 4 THT shell stakes, 2 NPTH pegs, no `Edge.Cuts`), so USB4115 was never a UJ20-style defect.

Secondary gains:

- **Exact pin match.** USB4145's footprint has 16 pads; the symbol in use, `Connector:USB_C_Receptacle_USB2.0_16P`, has 16 pins. KiCad's stock USB4115 footprint carries 24, leaving 8 dangling.
- **Correct stake length.** `0170` = 1.70 mm shell stake, right for a 1.6 mm board. `0070` and `0230` are the wrong-board-thickness variants.
- **Already proven** in `../Ikawa/board`, with a normalised footprint, a TraceParts STEP, and an import script.
- **In stock** — [DigiKey 16649098](https://www.digikey.com/en/products/detail/gct/USB4145-03-0170-C/16649098).

KiCad versions match (both `20260206` / 10.0), so no `kicad-cli fp upgrade` is required.

### 2.2 Licence handling

The footprint is SnapMagic-derived. Their terms permit designing, manufacturing and distributing *boards* built with the model, but prohibit redistributing the *model files*. Ikawa vendors it because that repo is closed source. **HydroMix is public and MIT**, so vendoring it here is exactly the prohibited case.

Therefore:

- Port Ikawa's `tools/import_usb4145.py` into `tools/`, self-contained (inline the ~90 lines of s-expression parse/dump it needs from `kisym`, rather than importing a module nothing else here uses). It drops the spurious `Edge.Cuts` obround around the right locating peg — an export artifact; GCT's drawing calls out a plain Ø0.71 hole, and the NPTH pads are authoritative — and renames stake pads `S1..S4` → `SH`.
- Outputs `pcb/Kleist2.pretty/USB_C_Receptacle_GCT_USB4145-03-0170-C_Vertical_SMT.kicad_mod` and `pcb/3dmodels/usb4145-03-0170-c.stp`, the latter referenced as `${KIPRJMOD}/3dmodels/…` per existing convention.
- **`.gitignore` both outputs**, with a comment naming the script and the reason. Anyone rebuilding runs it against their own SnapMagic download.

Fabrication outputs — Gerbers, drill, CPL, BOM — are board designs and are explicitly permitted, so a normal CM handoff is unaffected.

### 2.3 Schematic edits

`J13` keeps symbol `Connector:USB_C_Receptacle_USB2.0_16P`. Properties change only:

| Property | New value |
|---|---|
| Value / MPN | `USB4145-03-0170-C` |
| Footprint | `Kleist2:USB_C_Receptacle_GCT_USB4145-03-0170-C_Vertical_SMT` |
| Datasheet | `https://gct.co/files/drawings/usb4145.pdf` |
| DigiKey | `https://www.digikey.com/en/products/detail/gct/USB4145-03-0170-C/16649098` |

The existing Datasheet property points at `usb4105.pdf` while the part is a USB4115 — a pre-existing mismatch, corrected by this change.

### 2.4 PCB edits

Swap J13's footprint at its current origin `(104.2, 91.4)`, preserving nets `VBUS`, `GND`, `CC1`, `CC2`, `USB_D+`, `USB_D-` and the two unconnected SBU pads. Nothing is routed, so there is no rip-up.

**The two hand-drawn `peg` keepout zones inside J13 are dropped, not carried over.** They were drawn against the USB4115 body — local Y ±1.8…2.5, which cleared that part's 0.85 mm contacts at Y ±0.835. The USB4145's contact row sits at Y ±1.485 with 1.15 mm pads, so the same zones overlap eight signal pads (A5–A8, B5–B8) and produce 8 `items_not_allowed` DRC errors. They are USB4115-specific and do not transfer. The USB4145's own mechanical features are already real pads in the footprint (NPTH pegs at ±4.0, shell stakes at ±4.0/±1.43); redraw body keepouts against GCT's drawing only if the new part actually needs them.

`descr`/`tags` are set by the import script on the **library** footprint, not edited on the board copy, so the placed footprint matches its library version and KiCad's `lib_footprint_mismatch` check stays quiet.

### 2.5 UJ20 cleanup

The Same Sky UJ20-C-V-C-3-SMT-TR was the previous vertical candidate (added `73ab795`) and was never adopted — schematic and PCB both stayed on USB4115. Remove:

- `pcb/UJ20_C_V_C_3_SMT_TR/` and its `fp-lib-table` entry
- `pcb/Kleist2.pretty/CUI_UJ20-C-V-C-3-SMT-TR.kicad_mod`, `UJ20-C-V-C-3-SMT-TR.kicad_sym`, `UJ20-C-V-C-3-SMT-TR.step`

## 3. Display fan-out: U7 placement and output permutation

Separate from the two changes above, and applied in the same pass.

### 3.1 The problem

U7's outputs were wired straight through to the panel — `R1→ROW1`, `C1→COL1`, and so on. The KWM-20881AGB's pinout interleaves rows and columns arbitrarily down both edges (left edge: ROW1, COL4, COL6, ROW4, COL1, ROW2, COL7, COL8 top-to-bottom), so straight-through wiring guarantees a tangle. Measured **40 crossings** with U7 in its old position below the panel; **57** with U7 moved above it.

The board has only two signal layers (F.Cu, B.Cu — In1/In2 are power planes), so each crossing costs a via pair under the display.

### 3.2 What is free to change

Rows may be permuted among the panel's ROW pins and columns among its COL pins. The two groups cannot be interchanged: the IS31FL3730 sources current on rows and sinks on columns, and the panel is common-row-anode. The display driver is not written yet, so the permutation costs nothing downstream — it becomes two constant lookup tables.

### 3.3 Cost model — and two wrong ones first

This was got wrong twice before it was got right; the model *is* the design decision here.

1. **Straight-line segment crossings.** Wrong. A net from U7's left edge to the panel's bottom-right corner cuts through the panel's empty middle, where no other straight segment lies — so the single worst net to route scored as nearly free. This produced a map with `R8 → ROW5` wrapping the entire chip.
2. **Per-side bus inversions.** Better, but scores each side independently and so never charges a net for crossing from one side of U7 to the other. It just moved the same pathology onto different nets.
3. **One cyclic ring.** Correct. Nets cannot route through the panel, so the fan-out is planar iff the order nets leave U7's perimeter matches the order of pins around the panel. Anchor: U7's panel-facing edge ↔ the panel's top-centre; walking counterclockwise round U7 matches going down the panel's left column, round the bottom, and up its right column. Crossings = inversions between the two orders.

Scored under (3), model (1)'s "optimum" was **worse than doing nothing** — 36 crossings against the identity's 54 at rot 180, versus 12 achievable.

The actionable figure is not crossings but **how many nets need a via pair**, since one net can drop to B.Cu and pass under several others in a single hop. That is `16 − LIS` of the ring sequence, and it is what the final search minimises.

### 3.4 Placement

U7 stays at **rot 0** and moves from `(59.075, 113.775)` (below the panel) to **`(58.825, 83.0)`** — centred on the panel's x-centre, above it.

| U7 rot | nets needing a via | crossings |
|---|---|---|
| **0** | **5** | **12** |
| 90 | 6 | 26 |
| 180 | 6 | 18 |
| 270 | 7 | 38 |

Rot 0 puts pins 1–6 on U7's left edge, facing away from U5. That costs almost nothing: four of those six (SDB→3V3, IN/C_FILT/AD→GND) via straight down to the In1/In2 planes and need no routing, leaving only SDA/SCL — 400 kHz signals that route on B.Cu without penalty.

**DS1's rotation was also searched** and its current 90° is already optimal (rot 0/180/270 give 14–22 with the best U7 rotation). The panel is square, so rotating it would have been visually free with a framebuffer transform — it simply doesn't help.

### 3.5 Permutation

| Driver | → panel | | Driver | → panel |
|---|---|---|---|---|
| R1 | ROW2 | | C1 | COL4 |
| R2 | ROW4 | | C2 | COL6 |
| R3 | ROW1 | | C3 | COL1 |
| R4 | ROW3 | | C4 | COL7 |
| R5 | ROW6 | | C5 | COL8 |
| R6 | ROW8 | | C6 | COL2 |
| R7 | ROW7 | | C7 | COL3 |
| R8 | ROW5 | | C8 | COL5 |

Implemented by relabelling the **DS1-side** global labels only, so each net keeps meaning "driver output *n*" (`DISP_R3` = driver R3, landing on ROW1). U7-side labels are untouched. The inverse table for firmware is in `firmware/README.md`.

The resulting fan-out is two clean groups. Leaving U7 counterclockwise from the panel-facing edge: `R3 R2 R1` then `C1…C5` serve the panel's left column in order; `C6 C7 C8` then `R8 R7 R6 R5 R4` serve the right column. The 5 lifted nets are `R1`/`R2` dipping under `C1–C3` at U7's bottom-left, and `C6–C8` dipping under `R6–R8` at its right — both short local hops at the chip, not long detours.

## 4. Out of scope

- **Enclosure.** `top.stl` and `board.step` have no source in this repo. The USB4145 body and pad origin differ from the USB4115, so the port cutout must be regenerated in CAD after this lands. Flagged, not touched.
- **Display firmware.** Still to be written; unaffected by either change here.

## 5. Verification

1. KiCad ERC shows no new violations against the pre-change baseline — in particular no unconnected-pin error on U5 IO15 (the `no_connect` covers it) and no dangling `LED_A`/`STAT_LED`.
2. PCB has no D1 and no R4; U5 pad 23 carries `unconnected-(U5-IO15-Pad23)`; no `STAT_LED`/`LED_A`/`JTAG_SEL` anywhere.
3. DRC clean on J13's new land pattern, with attention to the 0.6 mm plated slots on the four `SH` stakes — routine on connector footprints, but confirm against JLCPCB's published minimum at order time.
4. 3D viewer: USB4145 STEP seats correctly. The STEP is TraceParts while the footprint is SnapMagic, so alignment is not guaranteed — cosmetic only.
5. `grep -ril "stat_led\|status led" .` must no longer hit `pcb/*.kicad_sch`, `pcb/*.kicad_pcb` or `README.md`. Remaining hits are expected: `firmware/main/` (source deliberately unchanged, §1.6), the two dated 2026-07-13 documents (historical records), this spec, and the passages in `firmware/README.md` / `firmware/BENCH_CHECKLIST.md` that explain the removal to a reader who is looking for the LED.
6. Fresh clone + `python3 tools/import_usb4145.py <vendor.kicad_mod> <vendor.step>` reproduces the ignored footprint.

## 6. Unresolved

None.
