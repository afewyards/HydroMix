#pragma once
#include "ctrl_core/types.h"

#define FAULT_BIT_SUPPLY (1u<<0)
#define FAULT_BIT_RETURN (1u<<1)
#define FAULT_BIT_SOURCE (1u<<2)
#define FAULT_BIT_HX_A   (1u<<3)
#define FAULT_BIT_HX_B   (1u<<4)
/* Not a probe: the sweep task itself stopped producing iterations. Distinct from the five
 * probe bits because the operator response is opposite -- 0x1f alone means go look at the
 * wiring, 0x3f means the board is at fault and the probes are probably fine. */
#define FAULT_BIT_SWEEP  (1u<<5)

#define COOLING_BLIND_PARK_PCT 10.0f
#define COOLING_FF_BIAS_PCT    10.0f   /* subtract in cooling FF-only (toward recirc) */

typedef enum { CTRL_FULL = 0, CTRL_FF_ONLY = 1, CTRL_PI_ONLY = 2, CTRL_PARK = 3 } ctrl_strategy_t;

typedef struct { bool supply, ret, source, hx_a, hx_b; } sensor_faults_t;

typedef struct {
    ctrl_strategy_t strategy;
    float    park_pos;
    float    ff_bias_pct;   /* added to FF target; negative = toward recirc */
    uint16_t alarm_bits;
} degradation_out_t;

degradation_out_t degradation_eval(const sensor_faults_t *f, ctrl_mode_t mode, float park_pos_cfg);
