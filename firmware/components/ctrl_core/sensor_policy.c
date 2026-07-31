#include "ctrl_core/sensor_policy.h"

bool sensor_fault_is_por_raw(uint16_t raw){
    return raw == SENSOR_POR_RAW;
}

bool sensor_fault_update(sensor_fault_state_t *s, bool read_ok){
    if (read_ok){
        s->good_streak++;
        if (s->faulted && s->good_streak >= SENSOR_CLEAR_AFTER){
            s->faulted = false;
            s->fail_streak = 0;
            return true;
        }
        return false;
    }
    s->good_streak = 0;
    s->fail_streak++;
    if (s->fail_streak >= SENSOR_FAULT_AFTER) s->faulted = true;
    return false;
}

float sensor_ema_step(float prev, float sample, float alpha, bool reseed){
    return reseed ? sample : prev + alpha * (sample - prev);
}
