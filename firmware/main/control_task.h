#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "ctrl_core/types.h"

void        control_task_start(void);
ctrl_mode_t control_task_mode(void);
bool        control_task_alarm(void);
uint16_t    control_task_faults(void);
void        control_task_set_water_running(bool on);
bool        control_task_water_running(void);
void        control_task_set_link(bool up, uint32_t last_seen_ms);
void        control_task_note_manual_override(void);
