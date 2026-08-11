#pragma once
#include <stddef.h>
#include <stdint.h>

/* Per-task heartbeats, kept in RTC RAM across a reset.
 *
 * A TASK_WDT reset says only that SOMETHING starved the watchdog; the panic text that
 * names the task goes to the console, and on this board the console is USB — the one
 * thing whose presence stops the resets happening at all. So the name never reaches
 * anyone. Record instead, per subscribed task, the uptime at its last loop iteration.
 * After the reset, the stalest heartbeat is the task that stopped feeding.
 *
 * If every app heartbeat is fresh and equally recent, no app task stalled, and the
 * starved subscriber is the idle task (CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0=y) —
 * i.e. something is monopolising the CPU rather than blocking. The two readings call
 * for opposite fixes, which is why this distinguishes them. */
typedef enum { HB_SENSORS, HB_CONTROL, HB_VALVE, HB_TELEM, HB_COUNT } hb_id_t;

/* Call once per loop iteration, right where the task feeds the watchdog. */
void hb_note(hb_id_t id);
/* Snapshot the just-ended run into history and log it. Call early in app_main. */
void hb_boot_report(void);
/* Console `hb`: live heartbeat ages plus the previous run's final ones. */
void hb_format(char *out, size_t n);
