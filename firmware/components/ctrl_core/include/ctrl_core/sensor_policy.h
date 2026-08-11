#pragma once
#include "ctrl_core/types.h"

#define SENSOR_POR_RAW      0x0550u   /* DS18B20 power-on-reset scratchpad (85.0 C) */
#define SENSOR_FAULT_AFTER  3         /* bad reads to latch a fault */
#define SENSOR_CLEAR_AFTER  3         /* consecutive good reads to clear a latched fault */
#define SENSOR_DECAY_AFTER  3         /* consecutive good reads before each further good forgives one fail */

typedef struct {
    int  fail_streak;
    int  good_streak;
    bool faulted;
} sensor_fault_state_t;

/* ---- Sweep liveness ------------------------------------------------------
 * The per-sensor fault latch cannot tell "every probe failed" apart from "the sweep
 * that would have read them never ran". sensors_start() deliberately seeds all five
 * latched-faulted, so a sweep task that dies -- or never gets created, since
 * xTaskCreate's return was unchecked -- presents as five simultaneously dead probes,
 * forever, with no failure tally to show for it. Nothing else catches that either: the
 * staleness guard in sensors_fill_faults() is skipped while last_ok_ms == 0, and a task
 * that never ran never subscribed to the task WDT, so the watchdog has nothing to time
 * out on. The plant parks (correct) and says nothing (not correct).
 *
 * Live on 2026-08-10 the plant sat parked for 13 h in exactly this state, reporting the
 * ZCL invalid sentinel on all five endpoints while the 1-Wire bus was provably healthy:
 * a reset restored every probe instantly and the failure tally showed zero read errors.
 *
 * So track the SWEEP, not just its results. */
#define SENSOR_SWEEP_GRACE_MS  35000u   /* boot allowance before "no sweep yet" counts as dead */
#define SENSOR_SWEEP_DEAD_MS   35000u   /* no completed sweep for this long == dead (3 periods + slack) */

typedef struct {
    uint32_t last_sweep_ms;
    bool     any_sweep;
} sensor_sweep_state_t;

bool  sensor_fault_is_por_raw(uint16_t raw);
bool  sensor_fault_update(sensor_fault_state_t *s, bool read_ok);
float sensor_ema_step(float prev, float sample, float alpha, bool reseed);

/* Call once per completed sweep iteration, whatever the reads did. */
void  sensor_sweep_note(sensor_sweep_state_t *s, uint32_t now_ms);
/* now_ms is uptime, so before the first sweep the grace window IS the elapsed time. */
bool  sensor_sweep_is_dead(const sensor_sweep_state_t *s, uint32_t now_ms);
