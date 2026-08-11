#pragma once
#include <stddef.h>
#include <stdint.h>

/* Temperature attribute-write tally, kept in RTC RAM across resets.
 *
 * The five measurement endpoints published the ZCL invalid sentinel for 13 h on
 * 2026-08-10 while the thermostat LocalTemperature on EP1 -- fed from the SAME sensor in
 * the same telemetry iteration, microseconds apart -- carried a live, rising value.
 * esp_zb_zcl_set_attribute_val() returns a status and every call site discarded it, so
 * which endpoint failed, and why, was unknowable from outside.
 *
 * Three things are recorded per run, because each rules out a different layer:
 *   ok/fail + status    -- did the write get accepted at all;
 *   mismatch/read       -- did the ZCL table actually TAKE the value. SUCCESS on the
 *                          write is not evidence of that, and the table is what the
 *                          reporting engine transmits from;
 *   ticks/sweeps/faults -- what the plant was doing at the time, so "writes all
 *                          succeeded" does not merely move the question elsewhere. */

/* Roll the previous run into history and log it. Call from zigbee_start(). */
void zbdiag_boot_init(void);

/* i is the 0-based endpoint index (0 = supply .. 4 = hx_b); ep is the ZCL endpoint
 * number, used only for logging. */
void zbdiag_note_ok(int i, uint8_t ep, int16_t wrote, int16_t read_back);
void zbdiag_note_fail(int i, uint8_t ep, int32_t status, int16_t wrote);

/* Once per completed telemetry iteration. */
void zbdiag_note_tick(uint32_t sweeps, uint16_t faults);
