# Gated Bidirectional Resync — Design

**Date:** 2026-08-05
**Target release:** 1.6.0
**Status:** approved (brainstorm 2026-08-05)

## Problem

The position estimator dead-reckons valve position; it re-zeros by driving a timed
full stroke (`travel_time_s × RESYNC_STALL_MULT` = 140 × 1.15 ≈ 161 s) to the
recirculation end-stop whenever accumulated travel ≥ 300 % (`POS_RESYNC_TRAVEL_PCT`)
or reversals ≥ 50 (`POS_RESYNC_REVERSALS`). In the current cooling regime the valve
regulates at 85–100 %, so every resync is a ~200 % round trip with ~3 min of forced
recirc and a ~2 K supply excursion. Three such events in the 19 h to 2026-08-05 05:36Z
(16:29Z, 22:07Z, 01:47Z) prompted a "valve goes to 0 for no reason" investigation.
The counters trip preferentially right after source cold slugs (t_src 18.5 → 14.6 °C,
~5 min dwell), i.e. exactly when the plant is disturbed.

## Decision

Allow resync toward the 100 %-source end-stop when position and temperatures make it
safe and near-free; otherwise defer briefly; fall back to today's recirc-end behavior.

This deliberately supersedes the 2026-07-13 spec rule "resync only toward the
0 %/full-recirculation end-stop" (and the `NEVER toward source` comment in
`valve_hw.c`). The original rationale — 100 % source is the dangerous end — is
preserved by the gate, the governor-band bound, and the mid-stroke abort.

## 1. Decision policy — new pure module `ctrl_core/resync_policy.c`

Counters (`pos_estimator`) and stroke mechanics (`valve_hw`) stay put. The new
state machine decides which end and when; it lives in ctrl_core so it is
host-testable. Inputs per cycle: `needs_resync`, `gate_ok`, `gate_hard_fail`,
position estimate, `now`. Output: `NONE | START_SOURCE | START_RECIRC`.

Rules:
- position < `RESYNC_NEAR_END_PCT` (50 %) → `START_RECIRC` immediately (cheap; the
  mix is mostly recirc already).
- position ≥ 50 % and gate passes → `START_SOURCE`.
- position ≥ 50 % and gate fails → defer, re-check each cycle; `START_SOURCE` the
  moment the gate passes; `START_RECIRC` after `RESYNC_DEFER_MAX_MS` (30 min), or
  immediately if position drops below 50 % while deferring (recirc became cheap).
- Counters keep accumulating during deferral; they reset only on resync completion
  (`pos_est_resync_done`), unchanged.
- Boot resync: recirc-end unconditionally (sensors boot latched-faulted; gate
  defaults to fail until the control task publishes).

## 2. Gate — computed in `control_step`, published like other outputs

`gate_ok` = (mode is HEATING or COOLING)
AND source probe unfaulted
AND `|t_src_f − t_set| ≤ RESYNC_SRC_GATE_K` (2.0 K)
AND `t_src_f` within `[gov_low + RESYNC_GATE_GOV_MARGIN_K, gov_high − RESYNC_GATE_GOV_MARGIN_K]` (margin 1.0 K).

`t_src_f` is the same filtered source the FF uses; `t_set` is the effective
setpoint from `control_step` (a dew-guard-raised setpoint automatically tightens
the gate). `gate_hard_fail` = source faulted OR `t_src_f` outside the raw
`[gov_low, gov_high]` band.

Plumbing: new `control_out_t` fields → `control_task` calls
`valve_note_resync_gate(ok, hard_fail)`; stored atomically; read by the valve task
under its existing lock. All thresholds are compile-time constants (like
`RESYNC_STALL_MULT`); no new Zigbee tunables.

## 3. Mechanics — minimal changes

- `RS_DRIVING` gains a direction (`s_rs_dir`); source-end strokes drive
  `dir_toward_source()` (already exists; `direction_swap` latch already resolves
  physical direction).
- Stroke duration stays `travel × 1.15` in BOTH directions — drift is the reason
  for resyncing, so no distance-based shortening.
- `pos_est_resync_done()` takes the seed: 0.0 (recirc) or 100.0 (source).
- `resync_active` semantics (PI freeze) unchanged. After a top-end resync in the
  current regime the FF baseline is near the rail, so reopen travel ≈ 0.

## 4. Mid-stroke abort

A source-end stroke started at the edge of a cold slug could carry t_src below the
governor band while the governor is powerless (resync ignores targets). If
`gate_hard_fail` becomes true during a source-end stroke, abandon it and start a
fresh full recirc-end stroke. Worst case equals today's behavior; never worse.
Recirc-end strokes never abort.

## 5. Testing

Unit tests on `resync_policy`: nearest-end choice both sides of 50 %, defer → gate
pass, defer → timeout, gate flicker during deferral, mid-stroke abort, boot path.
`pos_est` seed-100 test. Assertions are behavioral (per the 1.5.0
`test_hx_feedback` lesson — never gate on reduced travel/movement). No lagsim
changes; the policy is orthogonal to loop dynamics.

## Constants (new, compile-time)

| Constant | Value |
|---|---|
| `RESYNC_SRC_GATE_K` | 2.0 K |
| `RESYNC_GATE_GOV_MARGIN_K` | 1.0 K |
| `RESYNC_DEFER_MAX_MS` | 1 800 000 (30 min) |
| `RESYNC_NEAR_END_PCT` | 50.0 % |

## Out of scope (deliberate)

- Zigbee/Z2M exposure of resync events, chosen end, or counters — pair with the
  still-open `ff.frozen`/authority-floor telemetry item in a later release.
- Opportunistic resync (top-end micro-resyncs whenever the valve is already ≥97 %) —
  revisit if deferral proves insufficient.
- New runtime tunables.
