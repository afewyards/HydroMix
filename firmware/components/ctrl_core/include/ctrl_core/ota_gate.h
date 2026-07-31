#pragma once
#include "ctrl_core/types.h"

/* One-shot cycle counter for the OTA rollback-validation gate.
 *
 * Lives here rather than inline in control_task.c so the "fires exactly once, and
 * only at the threshold" property is host-testable: getting it wrong either
 * validates an image early (silently defeating rollback protection) or never
 * validates it (rollback loop). Neither failure is observable on a bench without
 * performing a real OTA.
 *
 * Call once per completed control cycle. *cycles saturates at threshold, so it
 * never wraps and never fires a second time. threshold == 0 never fires. */
bool ota_gate_step(uint32_t *cycles, uint32_t threshold);
