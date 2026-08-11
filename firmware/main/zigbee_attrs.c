/* Inbound ZCL attribute writes: the coordinator's only way to change how this device
 * regulates. Dispatched from zigbee.c's action_handler(), so every function here runs in
 * the Zigbee stack task and must NOT take the stack lock -- it is already held. */
#include "zigbee.h"
#include "zigbee_internal.h"
#include "config.h"
#include "control_task.h"
#include "valve_hw.h"
#include <math.h>
#include "esp_zigbee_core.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "zcl/esp_zigbee_zcl_thermostat.h"

esp_err_t zb_attr_write_cb(const esp_zb_zcl_set_attr_value_message_t *m)
{
    if (m->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
        bool on = *(bool*)m->attribute.data.value;
        /* Write-through, with no-op elision: HA re-sending the same state must not
         * erase/write NVS from the Zigbee stack task on every message. */
        if (on != control_task_water_running()) {
            control_task_set_water_running(on);
            config_water_running_save(on);
        }
        return ESP_OK;
    }
    if (m->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT) {
        if (m->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID)
            return ESP_OK;   /* accepted-but-ignored: mode is auto-detected */
        /* Below is now unreachable in normal operation: ZBOSS enforces heat<=cool-deadband on
         * this cluster, and this device's real targets (heat 35 / cool 18) always violate that,
         * so the stack rejects the write with INVALID_VALUE before this ever runs -- writable
         * ATTR_HEAT_SETPOINT/ATTR_COOL_SETPOINT on the custom cluster (below) are the real path
         * now. Left in place (harmless) in case a future write happens to satisfy the deadband. */
        /* Setpoints are clamped to the same 17..35 C band control_task.c re-clamps with.
         * Two things follow, both previously missing:
         *  - echo the clamped value back UNCONDITIONALLY: the stack already latched the
         *    raw write into the attribute store, so without this a 40 C write leaves the
         *    store at 4000 and HA reads back a setpoint the device will never honour.
         *    Safe to call from here -- this runs in the stack task, same as the
         *    ATTR_RESYNC self-clear below;
         *  - only config_save() when g_config actually changed, so HA re-sending the
         *    same setpoint (or one that clamps back to the current value) does not
         *    erase/write NVS from the stack task on every message. */
        if (m->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID) {
            float c = ctrl_clampf(*(int16_t*)m->attribute.data.value / 100.0f, 17.0f, 35.0f);
            if (c != g_config.t.heat_setpoint) { g_config.t.heat_setpoint = c; config_save(); }
            int16_t echo = (int16_t)(c * 100.0f + 0.5f);   /* c >= 17, so +0.5 rounds */
            esp_zb_zcl_set_attribute_val(EP_MAIN, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID, &echo, false);
        }
        if (m->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID) {
            float c = ctrl_clampf(*(int16_t*)m->attribute.data.value / 100.0f, 17.0f, 35.0f);
            if (c != g_config.t.cool_setpoint) { g_config.t.cool_setpoint = c; config_save(); }
            int16_t echo = (int16_t)(c * 100.0f + 0.5f);
            esp_zb_zcl_set_attribute_val(EP_MAIN, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID, &echo, false);
        }
        return ESP_OK;
    }
    if (m->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT) {
        /* manual position override — only honored while water_running OFF; supersedes
         * the OFF-park loop in control_task.c until water_running goes ON or reboot. */
        float v = *(float*)m->attribute.data.value;
        if (isnan(v)) return ESP_OK;   /* reject: NaN would otherwise freeze the valve */
        if (!control_task_water_running()) {
            valve_set_target(ctrl_clampf(v, 0.0f, 100.0f));
            control_task_note_manual_override();
        }
        return ESP_OK;
    }
    if (m->info.cluster == VALVECTL_CUSTOM_CLUSTER_ID) {
        if (m->attribute.id == ATTR_RESYNC) {
            if (*(uint8_t*)m->attribute.data.value) {
                valve_resync();
                uint8_t zero = 0;   /* self-clear */
                esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_RESYNC, &zero, false);
            }
            return ESP_OK;
        }
        config_apply_custom(m->attribute.id, m->attribute.data.value);   /* config.c, persists */
        /* config_apply_custom() clamps/NaN-rejects every writable float and u32 tunable
         * (heat/cool_threshold, gov_high/low, park_pos, kp, ki, alarm_dwell_ms,
         * travel_time_s, deadtime_s, pi_deadband_k); echo the clamped g_config value
         * back into the ZCL attribute store so a subsequent read reflects the regulated
         * value instead of the raw write the stack already latched into the backing
         * s_attr_* variable. */
        switch (m->attribute.id) {
        case ATTR_HEAT_THRESHOLD:
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_HEAT_THRESHOLD, &g_config.t.heat_threshold, false);
            break;
        case ATTR_COOL_THRESHOLD:
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_COOL_THRESHOLD, &g_config.t.cool_threshold, false);
            break;
        case ATTR_TRAVEL_TIME_S:
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_TRAVEL_TIME_S, &g_config.t.travel_time_s, false);
            /* config_apply_custom() may have just re-clamped valve_deadband_pct upward as a
             * side effect (shortening travel raises its stability floor) -- mirror that into
             * the attribute store too, same as every other clamped-tunable echo here. */
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_VALVE_DEADBAND, &g_config.t.valve_deadband_pct, false);
            break;
        case ATTR_PARK_POS:
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_PARK_POS, &g_config.t.park_pos, false);
            break;
        case ATTR_KP:
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_KP, &g_config.t.kp, false);
            break;
        case ATTR_KI:
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_KI, &g_config.t.ki, false);
            break;
        case ATTR_GOV_HIGH:
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_GOV_HIGH, &g_config.t.gov_high, false);
            break;
        case ATTR_GOV_LOW:
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_GOV_LOW, &g_config.t.gov_low, false);
            break;
        case ATTR_ALARM_DWELL:
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_ALARM_DWELL, &g_config.t.alarm_dwell_ms, false);
            break;
        case ATTR_DEADTIME_S:
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_DEADTIME_S, &g_config.t.deadtime_s, false);
            break;
        case ATTR_PI_DEADBAND:
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_PI_DEADBAND, &g_config.t.pi_deadband_k, false);
            break;
        case ATTR_HEAT_SETPOINT: {
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_HEAT_SETPOINT, &g_config.t.heat_setpoint, false);
            /* Mirror into the standard thermostat cluster's attribute store so a coordinator
             * reading OccupiedHeatingSetpoint sees the value this device actually regulates
             * from — the thermostat-branch write path above is unreachable, so without this
             * the standard cluster's stored value would silently go stale. */
            int16_t echo = (int16_t)(g_config.t.heat_setpoint * 100.0f + 0.5f);
            esp_zb_zcl_set_attribute_val(EP_MAIN, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID, &echo, false);
            break;
        }
        case ATTR_COOL_SETPOINT: {
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_COOL_SETPOINT, &g_config.t.cool_setpoint, false);
            int16_t echo = (int16_t)(g_config.t.cool_setpoint * 100.0f + 0.5f);
            esp_zb_zcl_set_attribute_val(EP_MAIN, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID, &echo, false);
            break;
        }
        case ATTR_VALVE_DEADBAND:
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_VALVE_DEADBAND, &g_config.t.valve_deadband_pct, false);
            break;
        default: break;   /* ATTR_DIRECTION_SWAP (unclamped) or read-only attrs — already match what the stack latched */
        }
        return ESP_OK;
    }
    return ESP_OK;
}
