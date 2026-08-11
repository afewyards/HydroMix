#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Shared mechanism for the RTC-RAM run histories.
 *
 * Three of these were written independently during the 1.6.x debugging (per-task
 * heartbeats, 1-Wire failure tallies, Zigbee attribute-write tallies) because each was
 * added under pressure while chasing a different hypothesis. The payloads genuinely
 * differ; only the mechanism is shared, and only the mechanism lives here.
 *
 * Depth is 4 because reaching the console on this board costs TWO resets -- plugging the
 * cable in is one, opening the port is another -- so a one-deep record always describes
 * the run your own reset ended, never the run the fault ended. */
#define DIAG_RING_DEPTH 4

/* MUST be the first member of the owning store struct; diag_ring_warm() casts to it.
 * Assert that at each call site with _Static_assert(offsetof(store_t, hdr) == 0, ...). */
typedef struct {
    uint32_t magic;
    uint32_t seq;      /* boot counter, so runs are identifiable across resets */
} diag_hdr_t;

/* Warm (magic matched): bump seq, leave the payload alone, return true -- the caller
 * should now shift its history and record the run that just ended.
 * Cold (magic absent): zero the whole store, stamp magic, seq = 1, return false. */
bool diag_ring_warm(void *store, size_t store_size, uint32_t magic);

/* hist[h] <- hist[h-1] for h descending. hist[0] is left as-is for the caller to
 * overwrite with its own typed fields. No-op if hist is NULL, elem is 0, or depth < 2. */
void diag_ring_shift(void *hist, size_t elem, uint8_t depth);

/* esp_reset_reason_t widened to int -- ctrl_core cannot include esp_system.h. Call sites
 * in main/ pin the mapping with _Static_assert against the real enum. */
const char *diag_reset_reason_name(int reason);

/* Bounded append into o[0..n). Returns the new used length, never more than n-1.
 * Always leaves o NUL-terminated. Safe to call repeatedly after the buffer is full. */
size_t diag_appendf(char *o, size_t n, size_t used, const char *fmt, ...)
    __attribute__((format(printf, 4, 5)));
