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

bool  sensor_fault_is_por_raw(uint16_t raw);
bool  sensor_fault_update(sensor_fault_state_t *s, bool read_ok);
float sensor_ema_step(float prev, float sample, float alpha, bool reseed);
