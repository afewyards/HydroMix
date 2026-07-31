#include "control_task.h"
#include "config.h"
#include "sensors_hw.h"
#include "valve_hw.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "ctrl_core/control.h"
#include "ota.h"
#include "ctrl_core/ota_gate.h"

#define CYCLE_MS 10000

static control_state_t s_ctrl;
/* Cross-task state. Written by the Zigbee stack task (attr_cb, signal handler) and the
 * console task, read by the control loop -- and s_mode/s_alarm/s_faults the other way
 * round. volatile matches the ui.c:13 idiom for exactly this pattern. It is NOT
 * atomicity, but each of these is a single naturally-aligned word on RV32, and volatile
 * is what stops the compiler caching them in a register across the 10 s loop. */
static volatile ctrl_mode_t s_mode = MODE_IDLE;
static volatile bool     s_alarm = false;
static volatile uint16_t s_faults = 0;
static volatile bool     s_water_running = false;      /* HA enables regulation */
static volatile bool     s_link_up = false;
static volatile uint32_t s_link_seen = 0;
/* One-shot: set when an AnalogOutput manual override write is accepted (zigbee.c),
 * cleared on any water_running transition. While set, the OFF-park loop below leaves
 * the valve alone (spec §4.5: write supersedes park_pos until water_running ON or reboot). */
static volatile bool s_override_active = false;
/* Completed control cycles, saturating at OTA_GATE_CYCLES (see ctrl_core/ota_gate.h).
 * Sensor faults are a property of the plant wiring, not of the image, so OTA
 * validation must not wait on fault-free probes. What this gate DOES prove is that
 * the control task is still looping -- and only if it takes longer than the task
 * watchdog: OTA_GATE_CYCLES * CYCLE_MS is ~110 s, comfortably OUTSIDE the 30 s window
 * (CONFIG_ESP_TASK_WDT_TIMEOUT_S=30, panic on), so a hung control task trips the WDT
 * -> reset -> rollback long before this gate could validate. At the previous 3 cycles
 * (~20 s) the gate fired INSIDE the watchdog window and therefore proved nothing. */
#define OTA_GATE_CYCLES 12
static uint32_t s_cycles_completed = 0;

static uint32_t now_ms(void){ return (uint32_t)(esp_timer_get_time() / 1000); }

static void control_loop(void *arg){
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    control_init(&s_ctrl);
    for (;;) {
        esp_task_wdt_reset();

        control_in_t in = {0};
        in.t_source_f = sensors_get(SENS_SOURCE).value_filt_c;
        in.t_return_f = sensors_get(SENS_RETURN).value_filt_c;
        in.t_supply   = sensors_get(SENS_SUPPLY).value_c;
        in.hx_a       = sensors_get(SENS_HX_A).value_c;
        in.valve_pos  = valve_get_position();
        sensors_fill_faults(&in.faults);
        /* HX-B is monitoring-only (spec: fault -> alarm only, never blocks control) —
         * excluded from the OTA good-sweep gate so it can't cause a rollback loop. */
        if (!in.faults.supply && !in.faults.ret && !in.faults.source && !in.faults.hx_a)
            ota_note_good_sweep();
        in.water_running = s_water_running;
        in.resync_active = valve_resync_active();
        in.link_up = s_link_up; in.link_last_seen_ms = s_link_seen;

        control_cfg_t cfg = {
            .heat_setpoint = ctrl_clampf(g_config.heat_setpoint, 17.0f, 35.0f),
            .cool_setpoint = ctrl_clampf(g_config.cool_setpoint, 17.0f, 35.0f),
            .park_pos = g_config.park_pos,
            .mode_cfg = { g_config.heat_threshold, g_config.cool_threshold, g_config.hysteresis,
                          g_config.enter_dwell_ms, g_config.leave_dwell_ms },
            .pi_cfg = { g_config.kp, g_config.ki, 0.0f, 100.0f, g_config.pi_deadband_k,
                        0.0f /* trim_max: 0 = PI_TRIM_CLAMP_PCT; overridden per strategy in control.c */ },
            .gov_cfg = { g_config.gov_high, g_config.gov_low, 35.0f, 17.0f },
            .alarm_dwell_ms = g_config.alarm_dwell_ms,
            .deadtime_s = g_config.deadtime_s,
        };

        control_out_t o = control_step(&s_ctrl, &in, &cfg, now_ms());
        s_mode = o.mode; s_alarm = o.supply_alarm; s_faults = o.fault_bits;

        if (o.regulating) {
            valve_set_target(o.valve_target);
        } else if (!s_water_running && !s_override_active) {
            /* water_running OFF and no manual override since the ON->OFF edge:
               actually park (a manual AnalogOutput write, applied directly by zigbee's
               attr_cb, supersedes this until water_running goes ON again or reboot). */
            valve_set_target(g_config.park_pos);
        }

        /* OTA validation fast path: don't let a missing/faulted probe delay validation
         * all the way out to the 10-minute fallback timer in ota.c. */
        if (ota_gate_step(&s_cycles_completed, OTA_GATE_CYCLES)) ota_note_good_sweep();

        vTaskDelay(pdMS_TO_TICKS(CYCLE_MS));
    }
}

void control_task_start(void){ xTaskCreate(control_loop, "control", 4096, NULL, 5, NULL); }
ctrl_mode_t control_task_mode(void){ return s_mode; }
bool control_task_alarm(void){ return s_alarm; }
uint16_t control_task_faults(void){ return s_faults; }
void control_task_set_water_running(bool on){
    if (on != s_water_running) s_override_active = false;   /* one-shot, cleared on each transition */
    s_water_running = on;
}
bool control_task_water_running(void){ return s_water_running; }
void control_task_set_link(bool up, uint32_t seen){
    /* seen BEFORE up: a reader that interleaves then sees a stale timestamp alongside
     * the new link state, never a fresh timestamp alongside the old one. The cooling
     * dew guard keys off the timestamp, and "older than it really is" errs toward
     * raising the cooling setpoint -- less cooling, the safe direction. */
    s_link_seen = seen;
    s_link_up = up;
}

/* Passive liveness. ZDO signals alone are not enough: a Router whose coordinator dies
 * silently gets no LEAVE and no failed STEERING, so s_link_up stays true forever. Any
 * inbound ZCL action is positive proof the coordinator is still talking to us. */
void control_task_note_link_activity(void){ s_link_seen = now_ms(); }
void control_task_note_manual_override(void){ s_override_active = true; }
