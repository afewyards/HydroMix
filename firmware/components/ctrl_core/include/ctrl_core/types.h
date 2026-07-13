#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

typedef enum { VALVE_STOP = 0, VALVE_OPEN = 1, VALVE_CLOSE = 2 } valve_dir_t;
typedef enum { MODE_IDLE = 0, MODE_HEATING = 1, MODE_COOLING = 2 } ctrl_mode_t;

static inline float ctrl_clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
