#include "ctrl_core/sensor_policy.h"

bool sensor_fault_is_por_raw(uint16_t raw){
    return raw == SENSOR_POR_RAW;
}

bool sensor_fault_update(sensor_fault_state_t *s, bool read_ok){
    if (read_ok){
        s->good_streak++;
        if (s->faulted){
            if (s->good_streak >= SENSOR_CLEAR_AFTER){
                s->faulted = false;
                s->fail_streak = 0;
                return true;
            }
            return false;
        }
        /* Leaky decay of the fail-streak bucket. Precisely:
         *   - the first SENSOR_DECAY_AFTER-1 goods of a run forgive nothing;
         *   - every consecutive good from the SENSOR_DECAY_AFTER'th onward forgives
         *     exactly one earlier fail (fail_streak--), floored at 0;
         *   - good_streak is NOT reset by a decay, so a long good run keeps decaying
         *     once per read; only a bad read resets it to 0.
         * Break-even is therefore one fail in SENSOR_DECAY_AFTER+1 reads: 25 % at 3.
         * A worse-than-25 % failure rate still climbs to the latch threshold; a rarer
         * isolated glitch is forgiven and never accumulates. The 1.2.1 value of 2 put
         * break-even at 33 %, so a probe failing a third of its sweeps never latched. */
        if (s->good_streak >= SENSOR_DECAY_AFTER && s->fail_streak > 0) s->fail_streak--;
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

void sensor_sweep_note(sensor_sweep_state_t *s, uint32_t now_ms){
    s->last_sweep_ms = now_ms;
    s->any_sweep     = true;
}

bool sensor_sweep_is_dead(const sensor_sweep_state_t *s, uint32_t now_ms){
    /* No sweep has ever completed: there is no age to measure, so uptime is the age.
     * This is the branch that catches a sweep task which never started -- the failure
     * every other guard in the system is blind to. */
    if (!s->any_sweep) return now_ms >= SENSOR_SWEEP_GRACE_MS;
    /* Unsigned wraparound is exactly right here: the difference stays correct across the
     * ~49.7-day rollover of the millisecond uptime. */
    return (uint32_t)(now_ms - s->last_sweep_ms) > SENSOR_SWEEP_DEAD_MS;
}
