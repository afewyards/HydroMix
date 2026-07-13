#include "ctrl_core/degradation.h"

degradation_out_t degradation_eval(const sensor_faults_t *f, ctrl_mode_t mode, float park_pos_cfg){
    degradation_out_t o = { .strategy = CTRL_FULL, .park_pos = park_pos_cfg,
                            .ff_bias_pct = 0.0f, .alarm_bits = 0 };
    if (f->supply) o.alarm_bits |= FAULT_BIT_SUPPLY;
    if (f->ret)    o.alarm_bits |= FAULT_BIT_RETURN;
    if (f->source) o.alarm_bits |= FAULT_BIT_SOURCE;
    if (f->hx_a)   o.alarm_bits |= FAULT_BIT_HX_A;
    if (f->hx_b)   o.alarm_bits |= FAULT_BIT_HX_B;

    bool src_or_ret = f->source || f->ret;

    if (f->supply && src_or_ret){
        o.strategy = CTRL_PARK;
        o.park_pos = (mode == MODE_COOLING) ? COOLING_BLIND_PARK_PCT : park_pos_cfg;
    } else if (f->supply){
        o.strategy = CTRL_FF_ONLY;
        if (mode == MODE_COOLING) o.ff_bias_pct = -COOLING_FF_BIAS_PCT;
    } else if (src_or_ret){
        o.strategy = CTRL_PI_ONLY;
    } else {
        o.strategy = CTRL_FULL;   /* hx_a/hx_b faults are alarm-only */
    }
    return o;
}
