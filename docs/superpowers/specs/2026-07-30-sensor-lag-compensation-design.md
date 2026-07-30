# Sensor-Lag Compensation for the Mixing Control Loop

- **Date:** 2026-07-30
- **Status:** Approved
- **Target firmware:** 1.1.0 (current: 1.0.7; minor bump — `ki` changes unit and new attributes are added)

## Problem

On a real water loop, supply temperature hunts around the setpoint with ~1–1.5 K
peak-to-peak amplitude and a period under 2 minutes. The firmware assumes the
supply probe reads the mixed-water temperature instantly; in reality the
clamp-on DS18B20 sees a change only after it conducts through the pipe wall
(τ ≈ 60–120 s for a bare dry clamp on copper), plus 10 s sampling and a few
seconds of transport from the mix point.

## Root cause

This is **not** linear PI instability — at typical source–return spreads,
kp = 4 %/K is ~4× below the marginal-stability gain. It is a **relay limit
cycle** driven by the discrete 3-point actuator:

1. Supply error reaches ~0.5 K → the 2 % valve deadband is exceeded → a pulse
   fires (min pulse 2 s ≈ 1.7 % travel ≈ 0.3 K of supply effect).
2. The effect is invisible for 3–4 control cycles while it transits the pipe
   wall and probe.
3. The error persists, so 3–4 more pulses stack before the first is seen.
4. Supply overshoots by the stacked amount; the cycle repeats in reverse.

The integrator also winds during transit: conditional anti-windup only engages
at the 0/100 total-output clamps, which FF (sitting mid-range) never lets it
reach — the trim can silently wind to ±40 %.

## Design

### 1. Transit hold (new, `ctrl_core`)

`control_step()` gains an input: valve movement since last cycle, derived from
the position-estimate delta (threshold 0.5 %). On movement, a hold timer starts
(`deadtime_s`, new tunable, default 30 s). While holding:

- integrator frozen, PI trim output latched;
- FF stays live (open-loop, not part of the feedback cycle);
- governor recirc override always bypasses the hold;
- early release when |ΔT_supply since hold start| > 0.25 K.

Effective correction cadence becomes ~1 pulse per dead time — the correct rate
for a 3-point actuator: fire, wait for the pipe to answer, correct.

### 2. PI deadband + trim clamp (`pi.c`)

- **Gap-form error deadband** `pi_deadband_k`, default 0.25 K: effective error
  is 0 inside the band, ramps in linearly beyond it; integrator frozen inside.
  Sized just above one minimum pulse's supply effect (~0.31 K at typical
  spread) so a single pulse cannot re-trigger itself.
- **Trim clamp:** P+I trim clamped to ±20 % (compile-time constant) with
  integrator back-calculation on clamp. Existing conditional integration at the
  0/100 total clamp stays.

### 3. dt-scaled gains + config migration

- `ki` changes unit from %/K **per cycle** to %/K **per minute**, integrated
  with measured elapsed cycle time.
- New defaults are the retune: **kp = 2.8 %/K, ki = 0.9 %/K/min**
  (≡ 0.15 per 10 s cycle).
- NVS config version bump; stored per-cycle ki migrates ×6 to %/K/min.
- Zigbee writes get sanity bounds (fixes existing unbounded-write defect):
  kp ∈ [0.5, 15], ki ∈ [0, 5] %/K/min, deadtime_s ∈ [0, 120],
  pi_deadband_k ∈ [0, 1].
- `deadtime_s` and `pi_deadband_k` exposed as new custom-cluster attributes;
  Z2M external converter updated to match.

### 4. Testing (`test_host/`)

Unit tests: hold enter / early-release / expiry / governor bypass, deadband gap
math, trim clamp back-calculation, dt scaling and ki migration. Plus one
closed-loop regression test against a small simulated first-order-plus-dead-time
plant (θ ≈ 30 s, τ ≈ 45 s, discrete 1.7 % pulses) asserting the old behaviour
sustains a limit cycle and the new logic converges. That test is the proof of
the fix.

### 5. Rollout

1. **Stopgap (tonight, no reflash):** write kp = 2.8, ki = 0.15 (old per-cycle
   unit) to the live device via Z2M.
2. **Hardware pass:** thermal paste under all five clamp probes; 25 mm
   insulation over each probe and ~100 mm of pipe either side. Halves probe lag
   (60–120 s → 20–30 s) and removes the 2–5 K ambient-coupling bias that skews
   both PI and FF.
3. **Firmware 1.1.0** via the existing OTA override index.

## Out of scope (second wave, only if hunting persists)

- Gain-scheduling kp by the source–return spread (kp ≈ 50/ΔT).
- Slew limit on the FF term (~15 %/min) against source-step kicks.
- Daily valve re-reference against position-estimate drift.
- Backlash compensation on direction reversal (3–8 % typical).
- Immersion/thermowell supply probe (τ ≈ 5–15 s) if clamp-on stays inadequate.

## Unresolved questions

None.
