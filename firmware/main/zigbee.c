#include "zigbee.h"
#include "control_task.h"
#include "sensors_hw.h"
#include "valve_hw.h"
#include "config.h"
#include "ota.h"
#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_app_desc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_zigbee_core.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "zcl/esp_zigbee_zcl_thermostat.h"

static const char *TAG = "zigbee";
static bool s_joined = false;

/* Steering never gives up: while unjoined we re-attempt on a backoff, so a
 * coordinator that is down, out of range, or not permitting join is recovered
 * from without a button press. Regulation is unaffected either way — the
 * control task keeps running on its own when the link is down. */
#define STEER_RETRY_MIN_MS 10000u
#define STEER_RETRY_MAX_MS 60000u
static uint32_t s_retry_ms = STEER_RETRY_MIN_MS;

static void configure_reporting_temp(uint8_t ep);
static void configure_reporting_position(void);
static void configure_reporting_bitmap(uint16_t attr_id);
static void configure_reporting_running_mode(void);
static void telemetry_task(void *arg);

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
    configure_reporting_running_mode();
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
static float    s_attr_deadtime_s;
static float    s_attr_pi_deadband;

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
    s_attr_deadtime_s     = g_config.deadtime_s;
    s_attr_pi_deadband    = g_config.pi_deadband_k;

    esp_zb_attribute_list_t *custom = esp_zb_zcl_attr_list_create(VALVECTL_CUSTOM_CLUSTER_ID);
    /* Plain (non-manufacturer-specific) attributes. 0xFC00 is already in the
     * manufacturer-specific *cluster* range, so the attributes inside it don't also need a
     * manufacturer code — and making them MANUF_SPEC is actively broken here:
     *   - esp_zb_custom_cluster_add_custom_attr() takes no manufacturer code, so the
     *     MANUF_SPEC flag alone registered them under code 0. Z2M reads/writes carrying
     *     code 0x1234 matched nothing -> UNSUPPORTED_ATTRIBUTE, every tunable read null
     *     and no write landed.
     *   - esp_zb_cluster_add_manufacturer_attr() is the API that *does* take the code, but
     *     its cluster_id parameter is documented as an esp_zb_zcl_cluster_id_t (a STANDARD
     *     cluster). Passing a custom 0xFC00 leaves the descriptor inconsistent and the
     *     ZBOSS reporting engine load-faults in zb_zcl_get_next_reporting_info() the
     *     moment it first sweeps the table -> boot loop right after "joined".
     * So: keep the custom-cluster API, drop MANUF_SPEC, and have the Z2M converter talk to
     * 0xFC00 without a manufacturer code. */
    uint8_t rw = ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE;
    uint8_t ro = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING;

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
    esp_zb_custom_cluster_add_custom_attr(custom, ATTR_DEADTIME_S,     ESP_ZB_ZCL_ATTR_TYPE_SINGLE, rw, &s_attr_deadtime_s);
    esp_zb_custom_cluster_add_custom_attr(custom, ATTR_PI_DEADBAND,    ESP_ZB_ZCL_ATTR_TYPE_SINGLE, rw, &s_attr_pi_deadband);

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

/* ZCL OTA file version, derived from the project version (firmware/version.txt)
 * and encoded as 0xMMmmpp00. A hardcoded constant makes every build advertise an
 * identical version, so an OTA server can never tell images apart and updates can
 * never apply. Falls back to 1.0.0 if the version string is not semver (e.g. the
 * git-describe default when version.txt is absent). */
static uint32_t ota_file_version(void)
{
    const esp_app_desc_t *d = esp_app_get_description();
    uint32_t part[3] = {0, 0, 0};
    int i = 0;
    for (const char *p = (d ? d->version : ""); *p && i < 3; ++p) {
        if (*p >= '0' && *p <= '9') part[i] = part[i] * 10u + (uint32_t)(*p - '0');
        else if (*p == '.')         ++i;
        else                        break;   /* stop at any non-semver suffix */
    }
    uint32_t v = ((part[0] & 0xFF) << 24) | ((part[1] & 0xFF) << 16) | ((part[2] & 0xFF) << 8);
    return v ? v : 0x01000000;
}

/* ZCL character strings are length-prefixed (byte 0 = length), same shape as the MFR/MODEL
 * literals above — but these two are built at runtime from the app descriptor so they can
 * never drift from the flashed image. Storage is static: the stack keeps the pointer and
 * build_endpoints() returns. */
static char SW_BUILD_ID[17];
static char DATE_CODE[17];

static void fill_zcl_string(char *dst, size_t cap, const char *src)
{
    size_t n = strlen(src);
    if (n > cap - 2) n = cap - 2;      /* 1 length byte + NUL */
    dst[0] = (char)n;
    memcpy(dst + 1, src, n);
    dst[n + 1] = '\0';
}

static esp_zb_ep_list_t *build_endpoints(void)
{
    esp_zb_ep_list_t *eps = esp_zb_ep_list_create();

    /* ---- EP1 main ---- */
    esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

    esp_zb_basic_cluster_cfg_t bcfg = { .zcl_version = 8, .power_source = 0x01 };
    esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(&bcfg);
    /* SWBuildID (0x4000) is what Z2M surfaces as "Firmware build ID"; without it the field
     * reads "unknown". DateCode (0x0006) backs "Firmware date code". Both are optional, so
     * esp_zb_basic_cluster_create() does not declare them. */
    const esp_app_desc_t *app = esp_app_get_description();
    fill_zcl_string(SW_BUILD_ID, sizeof SW_BUILD_ID, app ? app->version : "0.0.0");
    fill_zcl_string(DATE_CODE,   sizeof DATE_CODE,   app ? app->date    : "");

    esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MFR);
    esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID, SW_BUILD_ID);
    esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, DATE_CODE);
    esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL);
    esp_zb_cluster_list_add_basic_cluster(cl, basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_identify_cluster_cfg_t icfg = { .identify_time = 0 };
    esp_zb_cluster_list_add_identify_cluster(cl, esp_zb_identify_cluster_create(&icfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* Seeded from the NVS-restored value (app_main sets it before zigbee_start()), so
     * reads and reports of OnOff always agree with what the control task is doing. */
    esp_zb_on_off_cluster_cfg_t oncfg = { .on_off = control_task_water_running() };
    esp_zb_cluster_list_add_on_off_cluster(cl, esp_zb_on_off_cluster_create(&oncfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_thermostat_cluster_cfg_t thcfg = {
        .local_temperature = 0x8000,
        .occupied_cooling_setpoint = 1800,   /* 18.00 C, 0.01 units */
        .occupied_heating_setpoint = 3500,   /* 35.00 C */
        .control_sequence_of_operation = 0x04, /* cooling & heating */
        .system_mode = 0x01,                 /* auto */
    };
    /* RunningMode (0x001E) is optional and NOT part of esp_zb_thermostat_cluster_cfg_t, so
     * esp_zb_thermostat_cluster_create() never declares it. Without this explicit add, the
     * zigbee_push_status() write to RUNNING_MODE_ID silently no-ops and a coordinator read
     * gets UNSUPPORTED_ATTRIBUTE — the `mode` property stays null forever. */
    esp_zb_attribute_list_t *therm = esp_zb_thermostat_cluster_create(&thcfg);
    static uint8_t s_attr_running_mode = 0x00;   /* stack keeps the pointer; must outlive this fn */
    esp_zb_cluster_add_attr(therm, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
        ESP_ZB_ZCL_ATTR_THERMOSTAT_RUNNING_MODE_ID, ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &s_attr_running_mode);
    esp_zb_cluster_list_add_thermostat_cluster(cl, therm, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_analog_output_cluster_cfg_t aocfg = {
        .present_value = 50.0f, .out_of_service = 0, .status_flags = 0,
    };
    esp_zb_cluster_list_add_analog_output_cluster(cl, esp_zb_analog_output_cluster_create(&aocfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* OTA upgrade client. The CLIENT_DATA attribute is what actually arms the
     * client state machine: without it the cluster is declared but never queries
     * a server, so no image is ever offered. Transfer is handled by
     * ota_zcl_handle() via ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID. */
    esp_zb_ota_cluster_cfg_t otacfg = {
        .ota_upgrade_manufacturer = VALVECTL_MFR_CODE, .ota_upgrade_image_type = 0x0001,
        .ota_upgrade_file_version = ota_file_version(),
        .ota_upgrade_downloaded_file_ver = ota_file_version(),
    };
    esp_zb_attribute_list_t *ota = esp_zb_ota_cluster_create(&otacfg);
    /* static: the stack keeps a pointer to this, and build_endpoints() returns. */
    static esp_zb_zcl_ota_upgrade_client_variable_t ota_client = {
        .timer_query   = ESP_ZB_ZCL_OTA_UPGRADE_QUERY_TIMER_COUNT_DEF,
        .hw_version    = 1,
        .max_data_size = 64,
    };
    esp_zb_ota_cluster_add_attr(ota, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_DATA_ID, &ota_client);
    esp_zb_cluster_list_add_ota_cluster(cl, ota, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

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

/* Runs in the Zigbee stack task (scheduler callback), so no lock is taken. */
static void steer_retry_cb(uint8_t param);

static void schedule_steer_retry(void)
{
    esp_zb_scheduler_alarm_cancel(steer_retry_cb, 0);   /* never stack alarms */
    esp_zb_scheduler_alarm(steer_retry_cb, 0, s_retry_ms);
    ESP_LOGW(TAG, "not joined — next steering attempt in %u ms", (unsigned)s_retry_ms);
    s_retry_ms = (s_retry_ms * 2 > STEER_RETRY_MAX_MS) ? STEER_RETRY_MAX_MS : s_retry_ms * 2;
}

static void steer_retry_cb(uint8_t param)
{
    (void)param;
    if (s_joined) return;
    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
    /* Re-arm unconditionally: if this attempt yields no STEERING signal at all
     * (silent failure), the loop must still keep trying. Cancelled on join. */
    schedule_steer_retry();
}

static void mark_joined(void)
{
    esp_zb_scheduler_alarm_cancel(steer_retry_cb, 0);
    s_retry_ms = STEER_RETRY_MIN_MS;
    s_joined = true;
    ESP_LOGI(TAG, "joined");
    control_task_set_link(true, now_ms());
    configure_reporting_on_join();
    zigbee_on_join();
}

static void mark_unjoined(void)
{
    s_joined = false;
    control_task_set_link(false, now_ms());
    schedule_steer_retry();
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
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "startup failed (%s)", esp_err_to_name(err));
            mark_unjoined();
        } else if (esp_zb_bdb_is_factory_new()) {
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            schedule_steer_retry();          /* covers a steer that never reports */
        } else {
            /* Already commissioned and back on our network — no STEERING signal
             * is emitted in this path, so the join hooks must fire here too. */
            mark_joined();
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err == ESP_OK) mark_joined();
        else               mark_unjoined();
        break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        ESP_LOGW(TAG, "left the network");
        mark_unjoined();
        break;
    default:
        break;
    }
}

static esp_err_t attr_cb(const esp_zb_zcl_set_attr_value_message_t *m)
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
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_HEAT_THRESHOLD, &g_config.heat_threshold, false);
            break;
        case ATTR_COOL_THRESHOLD:
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_COOL_THRESHOLD, &g_config.cool_threshold, false);
            break;
        case ATTR_TRAVEL_TIME_S:
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_TRAVEL_TIME_S, &g_config.travel_time_s, false);
            break;
        case ATTR_PARK_POS:
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_PARK_POS, &g_config.park_pos, false);
            break;
        case ATTR_KP:
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_KP, &g_config.kp, false);
            break;
        case ATTR_KI:
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_KI, &g_config.ki, false);
            break;
        case ATTR_GOV_HIGH:
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_GOV_HIGH, &g_config.gov_high, false);
            break;
        case ATTR_GOV_LOW:
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_GOV_LOW, &g_config.gov_low, false);
            break;
        case ATTR_ALARM_DWELL:
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_ALARM_DWELL, &g_config.alarm_dwell_ms, false);
            break;
        case ATTR_DEADTIME_S:
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_DEADTIME_S, &g_config.deadtime_s, false);
            break;
        case ATTR_PI_DEADBAND:
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_PI_DEADBAND, &g_config.pi_deadband_k, false);
            break;
        default: break;   /* ATTR_DIRECTION_SWAP (unclamped) or read-only attrs — already match what the stack latched */
        }
        return ESP_OK;
    }
    return ESP_OK;
}

static esp_err_t action_handler(esp_zb_core_action_callback_id_t id, const void *msg)
{
    /* Any inbound core action -- attribute write, OTA block, reporting-config, command
     * callback -- proves the coordinator is alive. This is the broadest hook the stack
     * offers; plain attribute READS are answered inside the stack and do NOT reach here,
     * so the liveness signal is only as good as the traffic Z2M actually generates
     * (see Unresolved Q3: availability polling). */
    control_task_note_link_activity();
    if (id == ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID)    return attr_cb(msg);
    if (id == ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID) return ota_zcl_handle(msg);
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
     * external trigger). 10 s cadence matches the control cycle / sensor sweep.
     * Runs in its own task rather than an esp_timer callback: every call below
     * blocks on the Zigbee stack lock, and esp_timer callbacks share one task, so
     * blocking there stalls every other timer in the system. */
    xTaskCreate(telemetry_task, "zb_telem", 4096, NULL, 4, NULL);

    esp_zb_stack_main_loop();
}

void zigbee_start(void){ xTaskCreate(zb_task, "zigbee", 8192, NULL, 5, NULL); }
/* Called from the button/console tasks, i.e. outside the Zigbee stack task —
 * esp_zb_* APIs need the stack lock held, same as the reporting paths below. */
void zigbee_steer(void)
{
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
    esp_zb_lock_release();
}

void zigbee_leave(void)
{
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_bdb_reset_via_local_action();
    esp_zb_lock_release();
    s_joined = false;
    control_task_set_link(false, now_ms());
}
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
    int16_t lt = (int16_t)(sensors_get(SENS_SUPPLY).value_c * 100.0f);
    esp_zb_zcl_set_attribute_val(EP_MAIN, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID, &lt, false);
    esp_zb_lock_release();
}

/* Periodic telemetry: temps + status + thermostat local_temperature, every 10 s
 * regardless of writes/reads from the coordinator (BLOCKER fix — nothing else pushes
 * this data on its own). Each helper below acquires/releases the ZB lock itself. */
#define TELEMETRY_PERIOD_MS 10000

static void telemetry_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
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
