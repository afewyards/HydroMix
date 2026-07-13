#pragma once
#include <stdbool.h>

void  valve_start(void);
void  valve_set_target(float pct);
float valve_get_position(void);
void  valve_resync(void);
void  valve_stop(void);
bool  valve_resync_active(void);
float valve_travel_since_resync(void);   /* accumulated travel %, resets to 0 on resync completion */
