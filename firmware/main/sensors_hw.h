#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "ctrl_core/degradation.h"

typedef enum { SENS_SUPPLY, SENS_RETURN, SENS_SOURCE, SENS_HX_A, SENS_HX_B, SENS_COUNT } sensor_id_t;

typedef struct {
    float    value_c;        /* raw last good */
    float    value_filt_c;   /* EMA (source/return); == value_c for others */
    uint32_t last_ok_ms;
    bool     fault;          /* true after 3 consecutive failed sweeps */
} sensor_reading_t;

void            sensors_start(void);
sensor_reading_t sensors_get(sensor_id_t id);
void            sensors_fill_faults(sensor_faults_t *out);
/* Per-reason 1-Wire failure tallies for this run and the previous one. */
void            sensors_format_stats(char *out, size_t n);
