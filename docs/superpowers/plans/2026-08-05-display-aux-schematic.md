# 8×8 Display + AUX SSR Channels — Schematic Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add display block (8×8 matrix DS1 + I2C driver U7 + page button SW2) and 2 isolated 24 VAC PhotoMOS channels (Q6/Q7 + J15) to `pcb/ValveController.kicad_sch` per spec `docs/superpowers/specs/2026-08-05-display-aux-ssr-design.md`. Firmware/Z2M = separate follow-up plan.

**Architecture:** Two blocks on the single A3 sheet. Display: IS31FL3730-class 3V3 I2C driver scanning DS1, bus on IO6/IO7, button on IO11. AUX: per channel GPIO→680 Ω→PhotoMOS LED, 10 k pulldown, outputs to 4-pos PTSM J15. Wire stubs + net labels; netlist diff is the acceptance test for every edit.

**Tech Stack:** KiCad 10 s-expression text edits; `kicad-cli` ERC/netlist/BOM; kicad-happy:datasheets + digikey skills; git pathspec-scoped commits.

## Global Constraints

- Worktree `/Users/kleist/Sites/ValveController/.claude/worktrees/display-ssr`, branch `feat/display-aux`. NEVER touch `pcb/ValveController.kicad_pcb`, `board.step`, `top.stl`. NEVER cd to main checkout.
- Commits: `committing` skill; single invocation, pathspec-scoped: `git add -- <file> && git commit -m "msg" -- <same file>`. Never `git add -A`/`.`. Never reset/rebase/amend.
- **Full netlist diff after EVERY schematic edit.** Baseline = Task 0 (tree verified clean at plan time).
- New symbols: hand-embed as `Kleist:<MPN>` lib_symbols (existing in-file pattern); hidden `MPN` + `DigiKey` properties, format copied from D4. New `uuid`s via `uuidgen`. Coords on 1.27 mm grid, free canvas (check extents first).
- Refs frozen (verified free): DS1, U7, SW2, Q6, Q7, RL3, RL4, R18–R21, C14, J15 (+C15 iff U7 datasheet requires bulk cap).
- Nets frozen (firmware-facing): `I2C_SDA`, `I2C_SCL`, `BTN_PAGE`, `AUX1_EN`, `AUX2_EN`.
- U5 pads: 6=IO6, 7=IO7, 12=IO11, 20=IO22, 21=IO23 — delete any `no_connect` markers on these.
- `KICAD=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli` (fallback `which kicad-cli`). Artifacts → session scratchpad (`$SCRATCH`).

---

### Task 0: Baseline artifacts

**Files:** Create: `$SCRATCH/baseline.net`, `$SCRATCH/baseline-erc.rpt`

- [ ] **Step 1: Verify clean tree** — `git status --short` → empty. Anything else: STOP, report.
- [ ] **Step 2: Export baseline**

```bash
$KICAD sch export netlist --output $SCRATCH/baseline.net pcb/ValveController.kicad_sch
$KICAD sch erc --output $SCRATCH/baseline-erc.rpt pcb/ValveController.kicad_sch || true
```

Record baseline ERC violation count/types (pre-existing warnings NOT ours to fix).

---

### Task 1: Part verification (no schematic edits)

**Files:** Create: `$SCRATCH/parts.md`

**Interfaces — Produces:** confirmed MPN/DigiKey PN/footprint/pinout for every new ref; U7↔DS1 row/col polarity map; final Task 2/3 pin numbers.

- [ ] **Step 1: U7 driver datasheet** (kicad-happy:datasheets; DigiKey API creds absent → web fetch fallback). IS31FL3730: confirm package availability (QFN vs SOP), pin count/names (SDA, SCL, VCC, GND, AD, audio pin, matrix drive pins), I2C addr with AD→GND, current-limit register range, decoupling recommendation (100 nF + bulk? → C15 iff required), whether matrix pins source rows/sink columns or inverse. If IS31FL3730 unbuyable: fall back to another 2.7–5.5 V I2C 8×8 driver (IS31FL3731 subset use) and redo this step for it. Record exact pin↔net table for Task 2.
- [ ] **Step 2: DS1 matrix block** — KWM-20881 class 20 mm single **green**. **Chemistry constraint: yellow-green/GaP, Vf ≈ 2.1 V @ 20 mA. REJECT InGaN pure-green (Vf ≈ 3.0–3.2 V — no headroom from 3V3 drive).** From datasheet: 16-pin map (row/col per pin), common-anode vs common-cathode variant. **Pick the variant whose polarity matches U7's drive orientation from Step 1.** Record pin map + mechanical (pin grid, outline) for footprint.
- [ ] **Step 3: Q6/Q7 PhotoMOS** — CPC1017N: confirm ≥60 V blocking / ≥100 mA / SOP-4 pinout (expected, VERIFY: 1=A, 2=K, 3+4=MOS out), stock; alternates AQY211EH, TLP3406-class if out. Check KiCad official `Relay_SolidState` lib for existing symbol; else hand-embed 4-pin.
- [ ] **Step 4: Remaining parts stock check:**

| Ref | Value | Candidate MPN | Footprint |
|---|---|---|---|
| DS1 | 8×8 20 mm green (low-Vf) | KWM-20881 green variant (per Step 2) | `Kleist2:LED_Matrix_8x8_20mm` (created Task 2) |
| U7 | IS31FL3730 | IS31FL3730-QFLS2-TR (pkg per stock) | per package |
| Q6, Q7 | PhotoMOS 60 V/100 mA | CPC1017N | SOP-4 per datasheet |
| R18, R19 | 4.7 k 0402 | RC0402FR-074K7L | R_0402_1005Metric |
| RL3, RL4 | 680 R 0402 | RC0402FR-07680RL | R_0402_1005Metric |
| R20, R21 | 10 k 0402 | copy R7's MPN | R_0402_1005Metric |
| C14 | 100 nF 0402 | copy C7's MPN | C_0402_1005Metric |
| C15 | bulk per U7 datasheet (iff required) | pick in-stock | per value |
| SW2 | momentary | copy SW1 symbol/footprint/MPN exactly | copy SW1 |
| J15 | PTSM 0,5/4-2,5-V THT | 1770967 (VERIFY 4-pos MPN) | `TerminalBlock_Phoenix:TerminalBlock_Phoenix_PTSM-0,5-4-2.5-V-THR` (VERIFY exists in official lib) |

- [ ] **Step 5: Write `$SCRATCH/parts.md`** — final table incl. DigiKey PNs, U7 pin↔net map, DS1 pin map. Out-of-stock → same-spec substitute, note it.

---

### Task 2: Display block

**Files:** Modify: `pcb/ValveController.kicad_sch`. Create: `pcb/Kleist2.pretty/LED_Matrix_8x8_20mm.kicad_mod`

**Interfaces — Consumes:** `$SCRATCH/parts.md` (U7/DS1 pin maps). **Produces:** nets `I2C_SDA`, `I2C_SCL`, `BTN_PAGE` (frozen for firmware plan).

- [ ] **Step 1: Write expected netlist delta** to `$SCRATCH/task2-expected.md` (matrix pin numbers from parts.md):

```
New nets:
  I2C_SDA   = U5.6(IO6), U7.<SDA>, R18.1
  I2C_SCL   = U5.7(IO7), U7.<SCL>, R19.1
  BTN_PAGE  = U5.12(IO11), SW2.1
  DISP_R1…R8 = U7.<row pin> ↔ DS1.<row pin>   (8 nets)
  DISP_C1…C8 = U7.<col pin> ↔ DS1.<col pin>   (8 nets)
Grown nets:
  3V3 += U7.<VCC>, R18.2, R19.2, C14.1 (+C15.1 iff present)
  GND += U7.<GND>, U7.<AD>, SW2.2, C14.2 (+C15.2, +audio pin per datasheet)
```

- [ ] **Step 2: Create DS1 footprint** in `Kleist2.pretty` from parts.md mechanical data (16 THT pads, outline, pin 1 mark).
- [ ] **Step 3: Apply schematic edit** — embed `Kleist:<DS1-MPN>`, `Kleist:IS31FL3730` (16+... pins per datasheet) symbols; SW2 = copy of SW1; R/C from Device lib as existing. Wire stubs + labels per Step 1. Place block in free canvas.
- [ ] **Step 4: ERC** — violations ⊆ baseline set.
- [ ] **Step 5: Netlist diff** — `diff $SCRATCH/baseline.net $SCRATCH/task2.net`: every added line maps to Step-1 table, zero unexplained changes. Else STOP, fix.
- [ ] **Step 6: Commit** (committing skill)

```bash
git add -- pcb/ValveController.kicad_sch pcb/Kleist2.pretty/LED_Matrix_8x8_20mm.kicad_mod && git commit -m "feat(pcb): 8x8 matrix display, I2C driver, page button" -- pcb/ValveController.kicad_sch pcb/Kleist2.pretty/LED_Matrix_8x8_20mm.kicad_mod
```

---

### Task 3: AUX SSR channels

**Files:** Modify: `pcb/ValveController.kicad_sch`

**Interfaces — Consumes:** `$SCRATCH/parts.md` (Q6/Q7 pinout, J15 MPN). **Produces:** nets `AUX1_EN`, `AUX2_EN` (frozen for firmware plan).

- [ ] **Step 1: Write expected netlist delta** to `$SCRATCH/task3-expected.md`:

```
New nets:
  AUX1_EN  = U5.20(IO22), RL3.1, R20.1
  AUX1_DRV = RL3.2, Q6.1(A)
  AUX1_IN  = Q6.4, J15.1
  AUX1_OUT = Q6.3, J15.2
  AUX2_EN  = U5.21(IO23), RL4.1, R21.1
  AUX2_DRV = RL4.2, Q7.1(A)
  AUX2_IN  = Q7.4, J15.3
  AUX2_OUT = Q7.3, J15.4
Grown nets:
  GND += Q6.2(K), Q7.2(K), R20.2, R21.2
```

(Q6/Q7 pin numbers = expected CPC1017N, correct from parts.md if datasheet differs.)

- [ ] **Step 2: Apply schematic edit** — Q6/Q7 symbol from official Relay_SolidState lib or hand-embedded per Task 1; J15 = 4-pin connector symbol styled after J8, official PTSM 4-pos footprint, MPN/DigiKey props. Place near existing SSR/valve section but in LV zone (isolated side clear of logic pours — final spacing is layout's job).
- [ ] **Step 3: ERC** — violations ⊆ baseline set.
- [ ] **Step 4: Netlist diff** — `diff $SCRATCH/task2.net $SCRATCH/task3.net` = Step-1 table exactly; `diff $SCRATCH/baseline.net $SCRATCH/task3.net` = Task2+Task3 combined, nothing else.
- [ ] **Step 5: Commit** (committing skill)

```bash
git add -- pcb/ValveController.kicad_sch && git commit -m "feat(pcb): 2x isolated 24VAC aux PhotoMOS channels, J15" -- pcb/ValveController.kicad_sch
```

---

### Task 4: BOM sanity + spec closeout

**Files:** Modify: `docs/superpowers/specs/2026-08-05-display-aux-ssr-design.md` (Open-items only)

- [ ] **Step 1: BOM export**

```bash
$KICAD sch export bom --fields 'Reference,${QUANTITY},Value,MPN,DigiKey,MPN2,DigiKey2,Footprint' --group-by 'Value,MPN' --output $SCRATCH/bom.csv pcb/ValveController.kicad_sch
```

Expected: DS1, U7, SW2 (grouped with SW1), Q6/Q7 grouped, J15, R18–R21, RL3/RL4, C14 (+C15) all present with MPN + DigiKey filled.

- [ ] **Step 2: Spec cross-check** — walk spec Circuit section vs `diff baseline.net task3.net`; every component/net accounted for.
- [ ] **Step 3: Resolve spec Open items** (MPNs, PTSM PN; endpoint numbers stay open for firmware plan). Commit (committing skill):

```bash
git add -- docs/superpowers/specs/2026-08-05-display-aux-ssr-design.md && git commit -m "docs(pcb): resolve display/aux MPN open items" -- docs/superpowers/specs/2026-08-05-display-aux-ssr-design.md
```

- [ ] **Step 4: Report** — commits, ERC state vs baseline, netlist delta table, substitutions. PCB placement/routing + firmware explicitly OUT of scope.

---

### Task 5: Design review gate

**Files:** none (fixes, if any, follow house edit+diff+commit rules)

- [ ] **Step 1: Run kicad-happy:kicad full schematic analysis** over `pcb/ValveController.kicad_sch`.
- [ ] **Step 2: Triage findings** — pre-existing issues: report only, do NOT fix. Issues in new display/AUX blocks: fix via the Task 2/3 edit method (expected-delta → edit → ERC → netlist diff → committing-skill commit).
- [ ] **Step 3: Final report** — review verdict, fixes made, remaining known-issues list.

---

## Unresolved questions (owner)

*(all resolved at plan handoff 2026-08-05)*

1. Firmware + Z2M plan: **after** schematic lands. 2. DS1 color: **green, low-Vf chemistry only**. 3. Design review gate: **yes** (Task 5).
