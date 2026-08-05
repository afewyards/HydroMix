# Wilo Para ST iPWM2 Pump Interface — Schematic Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add the iPWM2 pump interface (24 V boost leaf, open-drain drive, feedback conditioning, J14) to `pcb/ValveController.kicad_sch` per spec `docs/superpowers/specs/2026-08-05-pump-ipwm2-interface-design.md`.

**Architecture:** Three blocks added to the single A3 sheet: TPS61040 5 V→24 V boost; BSS138 open-drain drive with gate pull-up (reset = pump stop); RC-conditioned 75 Hz feedback into IO21. All connectivity via short wire stubs + net labels (robust for text-level edits). Netlist diff is the acceptance test for every edit.

**Tech Stack:** KiCad 10 s-expression text edits; `kicad-cli` for ERC/netlist/BOM; DigiKey skill for stock; git pathspec-scoped commits.

## Global Constraints

- Branch `feat/fw-1.5.0`. NEVER touch `pcb/ValveController.kicad_pcb`, `board.step`, `top.stl` (owner's uncommitted work).
- Commits: single shell invocation, pathspec-scoped: `git add -- <file> && git commit -m "msg" -- <same file>`. Never `git add -A`/`.`. Never reset/rebase/amend.
- **Full netlist diff after EVERY schematic edit** (house rule since the J2 label incident). Baseline = working tree BEFORE Task 2 (tree already differs from HEAD: CS1/CS2 values — handled in Task 0).
- Every new symbol carries hidden `MPN` + `DigiKey` properties, format copied from an existing symbol (e.g. D4).
- New coords on 1.27 mm grid. New refs only from: U6, Q5, D6, J14, L1, R10–R17, C9–C12 (verified non-colliding).
- kicad-cli: `KICAD=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli` (fallback `which kicad-cli`).
- Scratchpad for artifacts: session scratchpad dir (`$SCRATCH` below).

---

### Task 0: Commit pre-existing owner edit + baseline artifacts

The working tree's only sch diff vs HEAD is the finished CS1/CS2 snubber substitution from 2026-07-28 (verified: 12-line diff, values/MPN only). Commit it separately so pump commits stay clean. **Owner approved at plan handoff.**

**Files:** Modify: none. Commit existing diff of `pcb/ValveController.kicad_sch`.

- [ ] **Step 1: Verify the diff is still only CS1/CS2**

Run: `git diff pcb/ValveController.kicad_sch`
Expected: only CS1/CS2 Value/MPN/DigiKey lines (`22nF X2`→`4.7nF X1`, `R463I22204001M`→`ECQ-UBAF472V5`). If anything else appears: STOP, report.

- [ ] **Step 2: Commit**

```bash
git add -- pcb/ValveController.kicad_sch && git commit -m "fix(pcb): CS1/CS2 snubber 22nF X2 -> 4.7nF X1 (ECQ-UBAF472V5, actuator hum)" -- pcb/ValveController.kicad_sch
```

- [ ] **Step 3: Export baseline netlist + ERC**

```bash
$KICAD sch export netlist --output $SCRATCH/baseline.net pcb/ValveController.kicad_sch
$KICAD sch erc --output $SCRATCH/baseline-erc.rpt pcb/ValveController.kicad_sch || true
```

Record baseline ERC violation count/types (pre-existing warnings are NOT ours to fix).

---

### Task 1: Part verification (no schematic edits)

**Files:** Create: `$SCRATCH/parts.md` (MPN table for Tasks 2–3).

**Interfaces — Produces:** confirmed MPN/DigiKey/footprint for every new ref; confirmed TPS61040 DBV pinout.

- [ ] **Step 1: Confirm TPS61040 pinout + FB network from TI datasheet** (use kicad-happy:datasheets or fetch TI PDF). Expected (VERIFY, do not assume): SOT-23-5: 1=SW, 2=GND, 3=FB, 4=EN, 5=VIN; V_FB = 1.233 V; check whether typical app uses a feedforward cap across R_top — if yes add C13 (0402, datasheet value) to Task 2; if no, omit.
- [ ] **Step 2: DigiKey stock check** (kicad-happy:digikey skill) for:

| Ref | Value | Candidate MPN | Footprint |
|---|---|---|---|
| U6 | TPS61040DBVR | TPS61040DBVR | Package_TO_SOT_SMD:SOT-23-5 |
| L1 | 10 µH, Isat ≥ 500 mA | NR4018T100M (alt: LQH44PN100MP0L) | per chosen part |
| Q5 | BSS138 | BSS138 (onsemi) | Package_TO_SOT_SMD:SOT-23 |
| D6 | B5819W | B5819W-TP (copy D4's exact props) | Diode_SMD:D_SOD-123 (copy D4) |
| R10 | 1M 0402 1% | RC0402FR-071ML | R_0402_1005Metric |
| R11 | 53.6k 0402 1% | RC0402FR-0753K6L | R_0402_1005Metric |
| R12 | 100R 0402 | RC0402FR-07100RL | R_0402_1005Metric |
| R13 | 10k 0402 | copy R7's MPN | R_0402_1005Metric |
| R14 | 2.2k 1206 0.5W | ERJ-P08J222V | R_1206_3216Metric |
| R15, R17 | 100R 1206 0.5W | ERJ-P08J101V (existing, copy RS1) | R_1206_3216Metric |
| R16 | 5.6k 0402 | RC0402FR-075K6L | R_0402_1005Metric |
| C9 | 4.7µF 16V X7R 0805 | pick in-stock | C_0805_2012Metric |
| C10, C11 | 2.2µF 50V X7R 1206 | pick in-stock | C_1206_3216Metric |
| C12 | 10nF 50V X7R 0402 | pick in-stock | C_0402_1005Metric |
| J14 | 1770966 | 1770966 (copy J8 symbol+footprint exactly) | TerminalBlock_Phoenix_PTSM-0,5-3-2.5-V-THR |

- [ ] **Step 3: Write `$SCRATCH/parts.md`** — final table incl. DigiKey PNs + inductor footprint lib name. Out-of-stock → substitute same-spec part, note it.

---

### Task 2: 24 V boost block

**Files:** Modify: `pcb/ValveController.kicad_sch`

**Interfaces — Produces:** net `24V` (used by Task 3). New refs U6, L1, D6, C9, C10, C11, R10, R11 (+C13 iff Task 1 says so).

Edit method (both tasks): embed needed lib_symbols from `/Applications/KiCad/KiCad.app/Contents/SharedSupport/symbols/` (Device:L, Device:C, Device:R, Transistor_FET:BSS138, Regulator_Switching TPS61040 if present — else hand-embed 5-pin symbol; D_Schottky + J-symbol: copy the already-embedded D4/J8 defs). New `uuid`s via `uuidgen`. Every pin gets a short wire stub + net label (or power symbol copied from an existing 5V/GND instance). Place block in free canvas (check existing symbol extents first; suggest below J5-area/left of sensor column). Copy hidden-property structure from D4.

- [ ] **Step 1: Write the expected netlist delta (the test)** to `$SCRATCH/task2-expected.md`:

```
New nets:
  24V     = D6.1(K), R14→(Task3), C10.1, C11.1, R10.1   [Task 2 subset: D6.1, C10.1, C11.1, R10.1]
  24V_SW  = L1.2, U6.1(SW), D6.2(A)
  24V_FB  = U6.3(FB), R10.2, R11.1
Grown nets:
  5V  += L1.1, U6.5(VIN), U6.4(EN), C9.1
  GND += U6.2(GND), R11.2, C9.2, C10.2, C11.2
```

- [ ] **Step 2: Apply the edit** (values/MPNs from `$SCRATCH/parts.md`; R10=1M, R11=53.6k → V_out≈24.2 V).
- [ ] **Step 3: ERC** — `$KICAD sch erc ...`; violations must be ⊆ baseline set (no new ones). If "input power pin not driven" appears on 24V/5V, add PWR_FLAG on `24V` and re-run.
- [ ] **Step 4: Netlist diff** — export `$SCRATCH/task2.net`, `diff $SCRATCH/baseline.net $SCRATCH/task2.net`; every added line must map to the Step-1 table; ZERO changes to pre-existing nets/components beyond listed `+=` growth. Any unexplained line: STOP, fix before commit.
- [ ] **Step 5: Commit**

```bash
git add -- pcb/ValveController.kicad_sch && git commit -m "feat(pcb): 24V boost leaf (TPS61040) for pump iPWM interface" -- pcb/ValveController.kicad_sch
```

---

### Task 3: Drive + feedback + J14

**Files:** Modify: `pcb/ValveController.kicad_sch`

**Interfaces — Consumes:** net `24V` (Task 2). **Produces:** nets `PUMP_PWM`/`PUMP_FB` on IO20/IO21 (firmware-facing names, frozen).

- [ ] **Step 1: Write expected netlist delta** to `$SCRATCH/task3-expected.md`:

```
New nets:
  PUMP_PWM      = U5.18(IO20), R12.1
  PUMP_PWM_G    = R12.2, R13.2, Q5.1(G)
  PUMP_PWM_LINE = Q5.3(D), R14.2, R15.1
  PUMP_PWM_OUT  = R15.2, J14.1
  PUMP_FB       = U5.19(IO21), R16.2, C12.1, R17.1
  PUMP_FB_IN    = R17.2, J14.3
Grown nets:
  24V += R14.1
  3V3 += R13.1, R16.1
  GND += Q5.2(S), C12.2, J14.2
```

BSS138 pins: 1=G, 2=S, 3=D. U5 pads: 18=IO20, 19=IO21 — if these pads carry `no_connect` markers, delete those markers.

- [ ] **Step 2: Apply the edit.** J14 = copy of J8 (symbol `Kleist:901361103`, footprint + Value/MPN `1770966`, DigiKey prop from J8) placed at (240.03, 97.79) continuing the sensor column. Pin order is Wilo-cable order (1=PWM out to pump, 2=GND, 3=feedback) — J14's pin1/2/3 nets deliberately differ from J8–J12's GND/DATA/3V3 convention, per spec. Q5/R12–R17/C12 grouped near U5's right side or the J14 area.
- [ ] **Step 3: ERC** — no new violations vs baseline.
- [ ] **Step 4: Netlist diff** — `diff $SCRATCH/task2.net $SCRATCH/task3.net` must match Step-1 table exactly; then `diff $SCRATCH/baseline.net $SCRATCH/task3.net` must equal Task2+Task3 tables combined, nothing else.
- [ ] **Step 5: Commit**

```bash
git add -- pcb/ValveController.kicad_sch && git commit -m "feat(pcb): iPWM2 pump drive, feedback conditioning, J14 (Wilo core order)" -- pcb/ValveController.kicad_sch
```

---

### Task 4: Final verification + spec closeout

**Files:** Modify: `docs/superpowers/specs/2026-08-05-pump-ipwm2-interface-design.md` (unresolved-Q3 only)

- [ ] **Step 1: BOM export sanity**

```bash
$KICAD sch export bom --fields 'Reference,${QUANTITY},Value,MPN,DigiKey,MPN2,DigiKey2,Footprint' --group-by 'Value,MPN' --output $SCRATCH/bom.csv pcb/ValveController.kicad_sch
```

Expected: U6, Q5, D6 (grouped with D4/D5 under B5819W-TP), L1, J14 (grouped with J8–J12 under 1770966), R10–R17, C9–C12 all present with MPN + DigiKey filled.

- [ ] **Step 2: Spec cross-check** — walk the spec's Circuit section against `diff baseline.net task3.net`; every spec component/net accounted for.
- [ ] **Step 3: Update spec Unresolved Q3** with the chosen L1 MPN (+ feedforward-cap outcome). Commit:

```bash
git add -- docs/superpowers/specs/2026-08-05-pump-ipwm2-interface-design.md && git commit -m "docs(pcb): resolve inductor MPN + Cff in pump spec" -- docs/superpowers/specs/2026-08-05-pump-ipwm2-interface-design.md
```

- [ ] **Step 4: Report** — final summary: commits made, ERC state vs baseline, netlist delta table, any part substitutions. PCB F8/placement is explicitly OUT of scope (later task, owner decision).

---

## Unresolved questions (owner)

1. **Task 0 commits your pre-existing uncommitted CS1/CS2 snubber edit** (22nF X2 → 4.7nF X1 ECQ-UBAF472V5) as its own `fix(pcb)` commit — OK? Without it, the first pump commit would silently sweep your edit in.
2. Optional: run a kicad-skill design review over the finished schematic after Task 4 (extra gate beyond ERC + netlist diff)?
