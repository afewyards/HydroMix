# Strip status LED · USB-C → USB4145 — Design Spec

**Date:** 2026-08-06
**Scope:** `pcb/ValveController.kicad_sch`, `pcb/ValveController.kicad_pcb`, `tools/`, docs. Firmware untouched.
**Context:** The 8×8 matrix (DS1 + U7 IS31FL3730, merged 2026-08-05) supersedes the single status LED. Separately, J13 moves to a connector that can actually be hand-soldered.

Both changes land while the PCB is unrouted (`HEAD` has 331 segments / 82 vias; working tree has 0), so neither costs a rip-up.

## 1. Status LED removal

D1 (0402 LED, IO15 active-low) is redundant now that the matrix can render state. It goes. R4 does **not** — it had a second job.

### 1.1 R4 is retained as the GPIO15 strap pull-up

Per **ESP32-C6 Series Datasheet v1.5**:

- **Table 3-1** — GPIO15 default configuration is **Floating**. Unlike GPIO9, it has no internal weak pull-up.
- **§3.4** — *"This pin does not have any internal pull resistors and the strapping value must be controlled by the external circuit that cannot be in a high impedance state."*
- **Table 3-7** — with the factory-default eFuses (`DIS_PAD_JTAG=0`, `DIS_USB_JTAG=0`, `JTAG_SEL_ENABLE=0`) GPIO15 is **ignored** and the JTAG source is the USB Serial/JTAG controller.

So a floating IO15 is harmless *today*, but it contradicts the datasheet's explicit instruction and would silently divert JTAG to the MTDI/MTCK/MTMS/MTDO pads if `JTAG_SEL_ENABLE` were ever burned. R4 stays, rewired as a plain pull-up, value **1 k → 10 k**. Weak enough that IO15 remains usable as an ordinary GPIO later.

Strapping values are latched at Chip Reset and held until power-down (datasheet §3), so nothing driving IO15 at runtime can affect JTAG selection.

### 1.2 Net rename

`STAT_LED` no longer describes the net. Rename to **`JTAG_SEL`**, matching the datasheet's Table 3-7 terminology.

### 1.3 Schematic edits

Delete:

| Element | Location / uuid |
|---|---|
| Symbol `D1` | `(182.88, 256.54)`, uuid `0385bbdc…` |
| Wire | `(182.88, 261.62)`–`(182.88, 266.7)`, uuid `407f079a…` |
| Wire | `(182.88, 251.46)`–`(179.07, 251.46)`, uuid `be735783…` |
| Global label `LED_A` | `(182.88, 266.7)`, uuid `05edf3a0…` |
| Global label `STAT_LED` | `(179.07, 251.46)`, uuid `6535899f…` |

Keep and modify:

- Global label `LED_A` at `(179.07, 248.92)` (uuid `1e0dba8d…`) → rename to `JTAG_SEL`. It already sits on R4's lower pin via the two surviving wires, so R4 lands between `3V3` and `JTAG_SEL` with no rewiring.
- Global label `STAT_LED` at `(134.62, 71.12)` — sits directly on U5's IO15 pin, no stub wire → rename to `JTAG_SEL`.
- `R4` Value `1k` → `10k`, MPN `user stock (1k 0402)` → `user stock (10k 0402)`, Description (currently empty) → GPIO15 JTAG-source strap pull-up, per datasheet §3.4.

Unchanged: the `3V3` label at `(182.88, 231.14)` and wires `(182.88, 231.14)`–`(182.88, 236.22)`, `(182.88, 246.38)`–`(182.88, 248.92)`, `(179.07, 248.92)`–`(182.88, 248.92)`.

No `no_connect` is needed — IO15 keeps a net.

### 1.4 PCB edits

Delete the `D1` footprint (`LED_SMD:LED_0402_1005Metric`, ~`(91.8, 108.075)`). Both its nets were unrouted. `R4` stays in place; `STAT_LED` → `JTAG_SEL` and the value change arrive via netlist update.

### 1.5 Documentation

- `README.md:51` — drop "status LED (**active-low**)" from the I/O row.
- `firmware/README.md` — remove the Status LED row from the GPIO map; replace the "LED legend" subsection with gestures only.
- `firmware/BENCH_CHECKLIST.md:108` — step 1.5 becomes a USB-console liveness check instead of an LED watch.

### 1.6 Firmware deliberately unchanged

`ui.c` (`PIN_LED`, `led_task`, `blink`) and `app_main.c:17` (`PIN_STAT_LED`) stay. They will be replaced wholesale by the IS31FL3730 display work rather than churned twice.

Consequence until then: `led_task` toggles IO15 against the new 10 k pull-up. Sinking 3V3/10 k ≈ **0.33 mA** when driven low — harmless, and it cannot affect the strap, which is already latched. Bring-up in the interim relies on the USB-Serial-JTAG console, which reports more than the LED did.

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

### 2.5 UJ20 cleanup

The Same Sky UJ20-C-V-C-3-SMT-TR was the previous vertical candidate (added `73ab795`) and was never adopted — schematic and PCB both stayed on USB4115. Remove:

- `pcb/UJ20_C_V_C_3_SMT_TR/` and its `fp-lib-table` entry
- `pcb/Kleist2.pretty/CUI_UJ20-C-V-C-3-SMT-TR.kicad_mod`, `UJ20-C-V-C-3-SMT-TR.kicad_sym`, `UJ20-C-V-C-3-SMT-TR.step`

## 3. Out of scope

- **Enclosure.** `top.stl` and `board.step` have no source in this repo. The USB4145 body and pad origin differ from the USB4115, so the port cutout must be regenerated in CAD after this lands. Flagged, not touched.
- **Display firmware.** Still to be written; unaffected by either change here.

## 4. Verification

1. KiCad ERC clean — specifically no unconnected-pin error on U5 IO15 and no dangling `LED_A`/`STAT_LED`.
2. Netlist update into the PCB reports D1 removed, `JTAG_SEL` present on U5 pin 23 and R4, and no `STAT_LED`/`LED_A` remaining.
3. DRC clean on J13's new land pattern, with attention to the 0.6 mm plated slots on the four `SH` stakes — routine on connector footprints, but confirm against JLCPCB's published minimum at order time.
4. 3D viewer: USB4145 STEP seats correctly. The STEP is TraceParts while the footprint is SnapMagic, so alignment is not guaranteed — cosmetic only.
5. `grep -ril "stat_led\|status led" .` hits only `firmware/main/` (source deliberately unchanged, §1.6), the two dated 2026-07-13 documents under `docs/superpowers/` (historical records, not to be rewritten), and this spec. `pcb/*.kicad_sch`, `pcb/*.kicad_pcb`, `README.md`, `firmware/README.md` and `firmware/BENCH_CHECKLIST.md` must all drop out.
6. Fresh clone + `python3 tools/import_usb4145.py <vendor.kicad_mod> <vendor.step>` reproduces the ignored footprint.

## 5. Unresolved

None.
