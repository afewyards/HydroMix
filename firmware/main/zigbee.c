#include "zigbee.h"
#include "control_task.h"
#include "sensors_hw.h"
#include "valve_hw.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_zigbee_core.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "zcl/esp_zigbee_zcl_thermostat.h"

static const char *TAG = "zigbee";
static bool s_joined = false;
static esp_timer_handle_t s_telemetry_timer;

static void configure_reporting_temp(uint8_t ep);
static void configure_reporting_position(void);
static void configure_reporting_bitmap(uint16_t attr_id);
static void telemetry_timer_cb(void *arg);

static uint32_t now_ms(void){ return (uint32_t)(esp_timer_get_time() / 1000); }

/* Reporting setup always runs on join, regardless of what zigbee_on_join()
 * (weak, overridable by e.g. ui.c) does. */
static void configure_reporting_on_join(void)
{
    configure_reporting_temp(EP_T_SUPPLY);
    configure_reporting_temp(EP_T_RETURN);
    configure_reporting_temp(EP_T_SOURCE);
    configure_reporting_temp(EP_T_HXA);
    configure_reporting_temp(EP_T_HXB);
    configure_reporting_position();
    /* spec §4.5: alarm/fault bitmaps report immediately (on any change, no periodic cap). */
    configure_reporting_bitmap(ATTR_ALARM_BITMAP);
    configure_reporting_bitmap(ATTR_FAULT_BITMAP);
}

__attribute__((weak)) void zigbee_on_join(void){}

/* ZCL character strings are length-prefixed. */
static char MFR[]   = "\005Knife";
static char MODEL[] = "\010HydroMix";

/* ---- custom manufacturer cluster (tunables + status) ----
 * Backing storage for custom attributes must outlive build_endpoints(); the
 * Zigbee stack references these by address, not by value. */
static float    s_attr_heat_threshold;
static float    s_attr_cool_threshold;
static uint32_t s_attr_travel_time_s;
static float    s_attr_park_pos;
static bool     s_attr_direction_swap;
static float    s_attr_kp;
static float    s_attr_ki;
static float    s_attr_gov_high;
static float    s_attr_gov_low;
static uint32_t s_attr_alarm_dwell;
static bool     s_attr_resync;
static uint16_t s_attr_alarm_bitmap;
static uint16_t s_attr_fault_bitmap;
static float    s_attr_travel_since;  /* TODO: no travel-since-resync accessor in valve_hw yet; placeholder 0 */

static esp_zb_attribute_list_t *build_custom_cluster(void)
{
    s_attr_heat_threshold = g_config.heat_threshold;
    s_attr_cool_threshold = g_config.cool_threshold;
    s_attr_travel_time_s  = g_config.travel_time_s;
    s_attr_park_pos       = g_config.park_pos;
    s_attr_direction_swap = g_config.direction_swap;
    s_attr_kp             = g_config.kp;
    s_attr_ki             = g_config.ki;
    s_attr_gov_high       = g_config.gov_high;
    s_attr_gov_low        = g_config.gov_low;
    s_attr_alarm_dwell    = g_config.alarm_dwell_ms;
    s_attr_resync         = false;
    s_attr_alarm_bitmap   = 0;
    s_attr_fault_bitmap   = 0;
    s_attr_travel_since   = 0.0f;

    esp_zb_attribute_list_t *custom = esp_zb_zcl_attr_list_create(VALVECTL_CUSTOM_CLUSTER_ID);
    uint8_t rw = ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_MANUF_SPEC;
    uint8_t ro = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING | ESP_ZB_ZCL_ATTR_MANUF_SPEC;

    esp_zb_custom_cluster_add_custom_attr(custom, ATTR_HEAT_THRESHOLD, ESP_ZB_ZCL_ATTR_TYPE_SINGLE, rw, &s_attr_heat_threshold);
    esp_zb_custom_cluster_add_custom_attr(custom, ATTR_COOL_THRESHOLD, ESP_ZB_ZCL_ATTR_TYPE_SINGLE, rw, &s_attr_cool_threshold);
    esp_zb_custom_cluster_add_custom_attr(custom, ATTR_TRAVEL_TIME_S,  ESP_ZB_ZCL_ATTR_TYPE_U32,    rw, &s_attr_travel_time_s);
    esp_zb_custom_cluster_add_custom_attr(custom, ATTR_PARK_POS,       ESP_ZB_ZCL_ATTR_TYPE_SINGLE, rw, &s_attr_park_pos);
    esp_zb_custom_cluster_add_custom_attr(custom, ATTR_DIRECTION_SWAP, ESP_ZB_ZCL_ATTR_TYPE_BOOL,   rw, &s_attr_direction_swap);
    esp_zb_custom_cluster_add_custom_attr(custom, ATTR_KP,             ESP_ZB_ZCL_ATTR_TYPE_SINGLE, rw, &s_attr_kp);
    esp_zb_custom_cluster_add_custom_attr(custom, ATTR_KI,             ESP_ZB_ZCL_ATTR_TYPE_SINGLE, rw, &s_attr_ki);
    esp_zb_custom_cluster_add_custom_attr(custom, ATTR_GOV_HIGH,       ESP_ZB_ZCL_ATTR_TYPE_SINGLE, rw, &s_attr_gov_high);
    esp_zb_custom_cluster_add_custom_attr(custom, ATTR_GOV_LOW,        ESP_ZB_ZCL_ATTR_TYPE_SINGLE, rw, &s_attr_gov_low);
    esp_zb_custom_cluster_add_custom_attr(custom, ATTR_ALARM_DWELL,    ESP_ZB_ZCL_ATTR_TYPE_U32,    rw, &s_attr_alarm_dwell);
    esp_zb_custom_cluster_add_custom_attr(custom, ATTR_RESYNC,         ESP_ZB_ZCL_ATTR_TYPE_BOOL,   rw, &s_attr_resync);
    esp_zb_custom_cluster_add_custom_attr(custom, ATTR_ALARM_BITMAP,   ESP_ZB_ZCL_ATTR_TYPE_16BITMAP, ro, &s_attr_alarm_bitmap);
    esp_zb_custom_cluster_add_custom_attr(custom, ATTR_FAULT_BITMAP,   ESP_ZB_ZCL_ATTR_TYPE_16BITMAP, ro, &s_attr_fault_bitmap);
    esp_zb_custom_cluster_add_custom_attr(custom, ATTR_TRAVEL_SINCE,   ESP_ZB_ZCL_ATTR_TYPE_SINGLE, ro, &s_attr_travel_since);

    return custom;
}

static void add_temp_ep(esp_zb_ep_list_t *eps, uint8_t ep)
{
    esp_zb_temperature_meas_cluster_cfg_t tcfg = {
        .measured_value = 0x8000,          /* invalid until first report */
        .min_value = -4000, .max_value = 12500,
    };
    esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_temperature_meas_cluster(
        cl, esp_zb_temperature_meas_cluster_create(&tcfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_endpoint_config_t epc = {
        .endpoint = ep, .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID, .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(eps, cl, epc);
}

static esp_zb_ep_list_t *build_endpoints(void)
{
    esp_zb_ep_list_t *eps = esp_zb_ep_list_create();

    /* ---- EP1 main ---- */
    esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

    esp_zb_basic_cluster_cfg_t bcfg = { .zcl_version = 8, .power_source = 0x01 };
    esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(&bcfg);
    esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MFR);
    esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL);
    esp_zb_cluster_list_add_basic_cluster(cl, basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_identify_cluster_cfg_t icfg = { .identify_time = 0 };
    esp_zb_cluster_list_add_identify_cluster(cl, esp_zb_identify_cluster_create(&icfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_on_off_cluster_cfg_t oncfg = { .on_off = 0 };   /* water_running = OFF at boot */
    esp_zb_cluster_list_add_on_off_cluster(cl, esp_zb_on_off_cluster_create(&oncfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_thermostat_cluster_cfg_t thcfg = {
        .local_temperature = 0x8000,
        .occupied_cooling_setpoint = 1800,   /* 18.00 C, 0.01 units */
        .occupied_heating_setpoint = 3500,   /* 35.00 C */
        .control_sequence_of_operation = 0x04, /* cooling & heating */
        .system_mode = 0x01,                 /* auto */
    };
    esp_zb_cluster_list_add_thermostat_cluster(cl, esp_zb_thermostat_cluster_create(&thcfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_analog_output_cluster_cfg_t aocfg = {
        .present_value = 50.0f, .out_of_service = 0, .status_flags = 0,
    };
    esp_zb_cluster_list_add_analog_output_cluster(cl, esp_zb_analog_output_cluster_create(&aocfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* OTA upgrade client. */
    esp_zb_ota_cluster_cfg_t otacfg = {
        .ota_upgrade_manufacturer = VALVECTL_MFR_CODE, .ota_upgrade_image_type = 0x0001,
        .ota_upgrade_file_version = 0x01000000,
    };
    esp_zb_cluster_list_add_ota_cluster(cl, esp_zb_ota_cluster_create(&otacfg), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    esp_zb_cluster_list_add_custom_cluster(cl, build_custom_cluster(), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_endpoint_config_t ep1 = {
        .endpoint = EP_MAIN, .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_THERMOSTAT_DEVICE_ID, .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(eps, cl, ep1);

    /* ---- EP2..EP6 temperature ---- */
    add_temp_ep(eps, EP_T_SUPPLY);
    add_temp_ep(eps, EP_T_RETURN);
    add_temp_ep(eps, EP_T_SOURCE);
    add_temp_ep(eps, EP_T_HXA);
    add_temp_ep(eps, EP_T_HXB);
    return eps;
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal)
{
    uint32_t *p = signal->p_app_signal;
    esp_err_t err = signal->esp_err_status;
    esp_zb_app_signal_type_t sig = *p;
    switch (sig) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err == ESP_OK && esp_zb_bdb_is_factory_new())
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err == ESP_OK) {
            s_joined = true; ESP_LOGI(TAG, "joined");
            control_task_set_link(true, now_ms());
            configure_reporting_on_join();
            zigbee_on_join();
        } else {
            s_joined = false;
            control_task_set_link(false, now_ms());
        }
        break;
    default:
        break;
    }
}

static esp_err_t attr_cb(const esp_zb_zcl_set_attr_value_message_t *m)
{
    if (m->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
        control_task_set_water_running(*(bool*)m->attribute.data.value);
        return ESP_OK;
    }
    if (m->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT) {
        if (m->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID)
            return ESP_OK;   /* accepted-but-ignored: mode is auto-detected */
        if (m->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID) {
            float c = *(int16_t*)m->attribute.data.value / 100.0f;
            g_config.heat_setpoint = ctrl_clampf(c, 17.0f, 35.0f); config_save();
        }
        if (m->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID) {
            float c = *(int16_t*)m->attribute.data.value / 100.0f;
            g_config.cool_setpoint = ctrl_clampf(c, 17.0f, 35.0f); config_save();
        }
        return ESP_OK;
    }
    if (m->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT) {
        /* manual position override — only honored while water_running OFF; supersedes
         * the OFF-park loop in control_task.c until water_running goes ON or reboot. */
        if (!control_task_water_running()) {
            valve_set_target(*(float*)m->attribute.data.value);
            control_task_note_manual_override();
        }
        return ESP_OK;
    }
    if (m->info.cluster == VALVECTL_CUSTOM_CLUSTER_ID) {
        if (m->attribute.id == ATTR_RESYNC) {
            if (*(uint8_t*)m->attribute.data.value) {
                valve_resync();
                uint8_t zero = 0;   /* self-clear */
                esp_zb_zcl_set_manufacturer_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, VALVECTL_MFR_CODE, ATTR_RESYNC, &zero, false);
            }
            return ESP_OK;
        }
        config_apply_custom(m->attribute.id, m->attribute.data.value);   /* config.c, persists */
        return ESP_OK;
    }
    return ESP_OK;
}

static esp_err_t action_handler(esp_zb_core_action_callback_id_t id, const void *msg)
{
    if (id == ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID) return attr_cb(msg);
    return ESP_OK;
}

static void zb_task(void *arg)
{
    esp_zb_cfg_t cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,
        .install_code_policy = false,
        .nwk_cfg.zczr_cfg = { .max_children = 10 },
    };
    esp_zb_init(&cfg);
    esp_zb_device_register(build_endpoints());
    esp_zb_core_action_handler_register(action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));

    /* Periodic telemetry push (spec: temps + status keep flowing even without an
     * external trigger). 10 s cadence matches the control cycle / sensor sweep. */
    const esp_timer_create_args_t telem_args = { .callback = telemetry_timer_cb, .name = "zb_telem" };
    ESP_ERROR_CHECK(esp_timer_create(&telem_args, &s_telemetry_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_telemetry_timer, 10000000ULL));

    esp_zb_stack_main_loop();
}

void zigbee_start(void){ xTaskCreate(zb_task, "zigbee", 8192, NULL, 5, NULL); }
void zigbee_steer(void){ esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING); }
void zigbee_leave(void){ esp_zb_bdb_reset_via_local_action(); s_joined = false; control_task_set_link(false, now_ms()); }
bool zigbee_joined(void){ return s_joined; }

void zigbee_report_temps(void)
{
    struct { uint8_t ep; sensor_id_t id; } map[] = {
        { EP_T_SUPPLY, SENS_SUPPLY }, { EP_T_RETURN, SENS_RETURN },
        { EP_T_SOURCE, SENS_SOURCE }, { EP_T_HXA, SENS_HX_A }, { EP_T_HXB, SENS_HX_B },
    };
    esp_zb_lock_acquire(portMAX_DELAY);
    for (size_t i = 0; i < 5; ++i) {
        int16_t v = (int16_t)(sensors_get(map[i].id).value_c * 100.0f);
        esp_zb_zcl_set_attribute_val(map[i].ep, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &v, false);
    }
    esp_zb_lock_release();
}

void zigbee_push_status(void)
{
    esp_zb_lock_acquire(portMAX_DELAY);

    float pos = valve_get_position();
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

    uint16_t alarm_bits = control_task_alarm() ? 0x0001 : 0x0000;
    esp_zb_zcl_set_manufacturer_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, VALVECTL_MFR_CODE, ATTR_ALARM_BITMAP, &alarm_bits, false);

    uint16_t fault_bits = control_task_faults();
    esp_zb_zcl_set_manufacturer_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, VALVECTL_MFR_CODE, ATTR_FAULT_BITMAP, &fault_bits, false);

    float travel_since = valve_travel_since_resync();
    esp_zb_zcl_set_manufacturer_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, VALVECTL_MFR_CODE, ATTR_TRAVEL_SINCE, &travel_since, false);

    esp_zb_lock_release();
}

static void set_local_temperature(void)
{
    esp_zb_lock_acquire(portMAX_DELAY);
    int16_t lt = (int16_t)(sensors_get(SENS_SUPPLY).value_c * 100.0f);
    esp_zb_zcl_set_attribute_val(EP_MAIN, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID, &lt, false);
    esp_zb_lock_release();
}

/* Periodic telemetry: temps + status + thermostat local_temperature, every 10 s
 * regardless of writes/reads from the coordinator (BLOCKER fix — nothing else pushes
 * this data on its own). Each helper below acquires/releases the ZB lock itself. */
static void telemetry_timer_cb(void *arg)
{
    zigbee_report_temps();
    zigbee_push_status();
    set_local_temperature();
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
    info.u.send_info.min_interval = 0;
    info.u.send_info.max_interval = 60;
    info.u.send_info.delta.f32 = 1.0f;   /* ±1 % */
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
        .manuf_code = VALVECTL_MFR_CODE,
    };
    info.u.send_info.min_interval = 0;
    info.u.send_info.max_interval = 0;
    info.u.send_info.delta.u16 = 0;
    info.u.send_info.def_min_interval = 0;
    info.u.send_info.def_max_interval = 0;
    esp_zb_zcl_update_reporting_info(&info);
}
