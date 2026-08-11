/* Outbound telemetry: the periodic push of temperatures, valve position and status, plus
 * the ZCL reporting-engine setup that a join arms. Everything here runs OUTSIDE the Zigbee
 * stack task, so each entry point takes the stack lock itself -- see the comments on the
 * individual functions for which ones do and which are called with it already held. */
#include "zigbee.h"
#include "zigbee_internal.h"
#include "zigbee_diag.h"
#include "control_task.h"
#include "sensors_hw.h"
#include "valve_hw.h"
#include "taskhb.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_zigbee_core.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "zcl/esp_zigbee_zcl_thermostat.h"

/* Last RunningMode value explicitly reported via push_running_mode_report() (below).
 * RUNNING_MODE_UNSET is not a valid RunningMode value, so it always counts as "changed"
 * — used both at boot and reset on every (re)join, see zigbee_reset_running_mode_push(). */
#define RUNNING_MODE_UNSET 0xFFu
static uint8_t s_last_pushed_running_mode = RUNNING_MODE_UNSET;

static void configure_reporting_temp(uint8_t ep);
static void configure_reporting_position(void);
static void configure_reporting_bitmap(uint16_t attr_id);
static void configure_reporting_running_mode(void);

/* Reporting setup always runs on join, regardless of what zigbee_on_join()
 * (weak, overridable by e.g. ui.c) does. */
void zigbee_configure_reporting_on_join(void)
{
    configure_reporting_temp(EP_T_SUPPLY);
    configure_reporting_temp(EP_T_RETURN);
    configure_reporting_temp(EP_T_SOURCE);
    configure_reporting_temp(EP_T_HXA);
    configure_reporting_temp(EP_T_HXB);
    configure_reporting_position();
    configure_reporting_running_mode();
    /* spec §4.5: alarm/fault bitmaps report immediately (on any change, no periodic cap). */
    configure_reporting_bitmap(ATTR_ALARM_BITMAP);
    configure_reporting_bitmap(ATTR_FAULT_BITMAP);
}

void zigbee_reset_running_mode_push(void)
{
    s_last_pushed_running_mode = RUNNING_MODE_UNSET;
}

/* ZCL "invalid" sentinel for the int16 temperature attributes -- Temperature
 * Measurement MeasuredValue and Thermostat LocalTemperature both define 0x8000
 * (== -32768) as "value not available". add_temp_ep() already seeds the cluster
 * with it, so the stack accepts it. */
#define ZCL_TEMP_INVALID ((int16_t)0x8000)

/* A latched-faulted probe holds its last-good value forever -- or 0.0 from BSS if it
 * never produced one. Publishing that as a live measurement is a lie the coordinator
 * cannot detect: HA charts a plausible frozen temperature and no automation notices.
 * Publish the sentinel instead. */
static int16_t temp_centi(sensor_id_t id)
{
    sensor_reading_t r = sensors_get(id);
    if (r.fault) return ZCL_TEMP_INVALID;
    return (int16_t)(r.value_c * 100.0f);
}

void zigbee_report_temps(void)
{
    struct { uint8_t ep; sensor_id_t id; } map[] = {
        { EP_T_SUPPLY, SENS_SUPPLY }, { EP_T_RETURN, SENS_RETURN },
        { EP_T_SOURCE, SENS_SOURCE }, { EP_T_HXA, SENS_HX_A }, { EP_T_HXB, SENS_HX_B },
    };
    esp_zb_lock_acquire(portMAX_DELAY);
    for (size_t i = 0; i < 5; ++i) {
        int16_t v = temp_centi(map[i].id);
        esp_zb_zcl_status_t st = esp_zb_zcl_set_attribute_val(
            map[i].ep, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &v, false);
        if (st == ESP_ZB_ZCL_STATUS_SUCCESS) {
            /* Read straight back out of the ZCL table -- the same place the reporting
             * engine sources from. SUCCESS on the write is not evidence the table took it. */
            esp_zb_zcl_attr_t *a = esp_zb_zcl_get_attribute(
                map[i].ep, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID);
            int16_t back = (a && a->data_p) ? *(int16_t *)a->data_p : (int16_t)0x7FFF;
            zbdiag_note_ok((int)i, map[i].ep, v, back);
        } else {
            zbdiag_note_fail((int)i, map[i].ep, (int32_t)st, v);
        }
    }
    esp_zb_lock_release();
    zbdiag_note_tick(sensors_sweep_count(), control_task_faults());
}

/* configure_reporting_running_mode() below only arms the ZCL reporting engine's
 * passive delta/interval check; live testing found Z2M's `mode` sensor going stale for
 * hours regardless (observed: 6 h stale). So on top of that, push an explicit one-shot
 * report the moment the computed value actually changes. RUNNING_MODE_UNSET (not a valid
 * RunningMode value) forces a push on the first zigbee_push_status() after each join —
 * even when the mode itself hasn't changed since the last join — giving Z2M an
 * authoritative read instead of waiting on the passive engine's next sweep; see
 * zigbee_reset_running_mode_push(), called from zigbee.c's mark_joined(). */
static void push_running_mode_report(uint8_t running_mode)
{
    if (!zigbee_joined() || running_mode == s_last_pushed_running_mode) return;

    /* Caller already holds the ZB stack lock (zigbee_push_status(), same discipline as
     * every other esp_zb_* call from outside the stack task in this file). Address mode
     * DST_ADDR_ENDP_NOT_PRESENT means "resolve via the binding table", same destination
     * the passive reporting engine would use — this just sends it now instead of on the
     * engine's next tick. */
    esp_zb_zcl_report_attr_cmd_t cmd = {
        .zcl_basic_cmd = { .src_endpoint = EP_MAIN },
        .address_mode  = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT,
        .direction     = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        .clusterID     = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
        .manuf_code    = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .attributeID   = ESP_ZB_ZCL_ATTR_THERMOSTAT_RUNNING_MODE_ID,
    };
    /* Only latch on a confirmed send. A failed send (e.g. right after join, before the
     * route/binding is ready) must NOT mark the value as pushed -- otherwise it goes
     * quiet until the mode changes again instead of retrying on the next telemetry tick. */
    if (esp_zb_zcl_report_attr_cmd_req(&cmd) == ESP_OK) {
        s_last_pushed_running_mode = running_mode;
    }
}

void zigbee_push_status(void)
{
    esp_zb_lock_acquire(portMAX_DELAY);

    /* Quantise to 0.1 % before publishing: the valve is timing-driven and cannot resolve
     * anything close to that, so advertising the raw float is false precision to every
     * consumer, not just Z2M. Note this does NOT by itself give a clean number on the wire
     * — float32 cannot represent 48.1 exactly (it becomes 48.099998474121094), so the Z2M
     * converter still rounds on receipt. This is about not claiming the resolution. */
    float pos = roundf(valve_get_position() * 10.0f) / 10.0f;
    esp_zb_zcl_set_attribute_val(EP_MAIN, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID, &pos, false);

    /* ZCL Thermostat Running Mode values (spec: Off=0x00, Cool=0x03, Heat=0x04).
     * Not using the ESP_ZB_ZCL_THERMOSTAT_RUNNING_MODE_*_VALUE macros here: they
     * cast to zb_uint8_t, a raw ZBOSS type not transitively available via the
     * esp-zigbee-lib include path used elsewhere in this file. */
    uint8_t running_mode;
    switch (control_task_mode()) {
        case MODE_HEATING: running_mode = 0x04; break;
        case MODE_COOLING: running_mode = 0x03; break;
        default:           running_mode = 0x00; break;
    }
    esp_zb_zcl_set_attribute_val(EP_MAIN, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_THERMOSTAT_RUNNING_MODE_ID, &running_mode, false);
    push_running_mode_report(running_mode);

    uint16_t alarm_bits = control_task_alarm() ? 0x0001 : 0x0000;
    esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_ALARM_BITMAP, &alarm_bits, false);

    uint16_t fault_bits = control_task_faults();
    esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_FAULT_BITMAP, &fault_bits, false);

    float travel_since = roundf(valve_travel_since_resync() * 100.0f) / 100.0f;
    esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_TRAVEL_SINCE, &travel_since, false);

    esp_zb_lock_release();
}

static void set_local_temperature(void)
{
    esp_zb_lock_acquire(portMAX_DELAY);
    int16_t lt = temp_centi(SENS_SUPPLY);
    esp_zb_zcl_set_attribute_val(EP_MAIN, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID, &lt, false);
    esp_zb_lock_release();
}

/* Periodic telemetry: temps + status + thermostat local_temperature, every 10 s
 * regardless of writes/reads from the coordinator (BLOCKER fix — nothing else pushes
 * this data on its own). Each helper below acquires/releases the ZB lock itself. */
#define TELEMETRY_PERIOD_MS 10000

void telemetry_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
        hb_note(HB_TELEM);
        zigbee_report_temps();
        zigbee_push_status();
        set_local_temperature();
    }
}

/* Reporting: temps at ±0.2 K or 60 s max; position at ±1 %. */
static void configure_reporting_temp(uint8_t ep)
{
    esp_zb_zcl_reporting_info_t info = {
        .direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
        .ep = ep,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    /* Without this the report goes out with APS profile 0 (ZDO), which
     * coordinators route to the ZDO parser and reject as a malformed frame. */
    info.dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID;
    info.u.send_info.min_interval = 0;
    info.u.send_info.max_interval = 60;
    info.u.send_info.delta.s16 = 20;   /* 0.2 K in hundredths */
    info.u.send_info.def_min_interval = 0;
    info.u.send_info.def_max_interval = 60;
    esp_zb_zcl_update_reporting_info(&info);
}

static void configure_reporting_position(void)
{
    esp_zb_zcl_reporting_info_t info = {
        .direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
        .ep = EP_MAIN,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    info.dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID;
    info.u.send_info.min_interval = 0;
    info.u.send_info.max_interval = 60;
    info.u.send_info.delta.f32 = 1.0f;   /* ±1 % */
    info.u.send_info.def_min_interval = 0;
    info.u.send_info.def_max_interval = 60;
    esp_zb_zcl_update_reporting_info(&info);
}

/* RunningMode: on change, and at least every 60 s so `mode` survives a coordinator restart. */
static void configure_reporting_running_mode(void)
{
    esp_zb_zcl_reporting_info_t info = {
        .direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
        .ep = EP_MAIN,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_ZCL_ATTR_THERMOSTAT_RUNNING_MODE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    info.dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID;
    info.u.send_info.min_interval = 0;
    info.u.send_info.max_interval = 60;
    info.u.send_info.delta.u8 = 1;
    info.u.send_info.def_min_interval = 0;
    info.u.send_info.def_max_interval = 60;
    esp_zb_zcl_update_reporting_info(&info);
}

/* Alarm/fault bitmaps: immediately on any change, no periodic re-send
 * (max_interval=0, delta=0 -> report as soon as the value differs). */
static void configure_reporting_bitmap(uint16_t attr_id)
{
    esp_zb_zcl_reporting_info_t info = {
        .direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
        .ep = EP_MAIN,
        .cluster_id = VALVECTL_CUSTOM_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = attr_id,
        /* Plain attributes now (see build_custom_cluster) — a VALVECTL_MFR_CODE here would
         * no longer resolve to any registered attribute. */
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    info.dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID;
    info.u.send_info.min_interval = 0;
    info.u.send_info.max_interval = 0;
    info.u.send_info.delta.u16 = 0;
    info.u.send_info.def_min_interval = 0;
    info.u.send_info.def_max_interval = 0;
    esp_zb_zcl_update_reporting_info(&info);
}
