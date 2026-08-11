#include "zigbee.h"
#include "control_task.h"
#include "sensors_hw.h"
#include "valve_hw.h"
#include "config.h"
#include "ota.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_app_desc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_zigbee_core.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "zcl/esp_zigbee_zcl_thermostat.h"

static const char *TAG = "zigbee";

static void zb_temp_stats_init(void);   /* defined with the tally, below */
static bool s_joined = false;

/* Steering never gives up: while unjoined we re-attempt on a backoff, so a
 * coordinator that is down, out of range, or not permitting join is recovered
 * from without a button press. Regulation is unaffected either way — the
 * control task keeps running on its own when the link is down. */
#define STEER_RETRY_MIN_MS 10000u
#define STEER_RETRY_MAX_MS 60000u
static uint32_t s_retry_ms = STEER_RETRY_MIN_MS;

/* Last RunningMode value explicitly reported via push_running_mode_report() (below).
 * RUNNING_MODE_UNSET is not a valid RunningMode value, so it always counts as "changed"
 * — used both at boot and reset on every (re)join, see mark_joined(). */
#define RUNNING_MODE_UNSET 0xFFu
static uint8_t s_last_pushed_running_mode = RUNNING_MODE_UNSET;

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
static float    s_attr_travel_since;  /* mirrors valve_travel_since_resync(); pushed by zigbee_push_status() */
static float    s_attr_deadtime_s;
static float    s_attr_pi_deadband;
static float    s_attr_heat_setpoint;
static float    s_attr_cool_setpoint;
static float    s_attr_valve_deadband;

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
    s_attr_heat_setpoint  = g_config.heat_setpoint;
    s_attr_cool_setpoint  = g_config.cool_setpoint;
    s_attr_valve_deadband = g_config.valve_deadband_pct;

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
    esp_zb_custom_cluster_add_custom_attr(custom, ATTR_HEAT_SETPOINT,  ESP_ZB_ZCL_ATTR_TYPE_SINGLE, rw, &s_attr_heat_setpoint);
    esp_zb_custom_cluster_add_custom_attr(custom, ATTR_COOL_SETPOINT,  ESP_ZB_ZCL_ATTR_TYPE_SINGLE, rw, &s_attr_cool_setpoint);
    esp_zb_custom_cluster_add_custom_attr(custom, ATTR_VALVE_DEADBAND, ESP_ZB_ZCL_ATTR_TYPE_SINGLE, rw, &s_attr_valve_deadband);

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
    s_last_pushed_running_mode = RUNNING_MODE_UNSET;   /* force a fresh RunningMode push, see above */
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
        /* Below is now unreachable in normal operation: ZBOSS enforces heat<=cool-deadband on
         * this cluster, and this device's real targets (heat 35 / cool 18) always violate that,
         * so the stack rejects the write with INVALID_VALUE before attr_cb ever runs — writable
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
            if (c != g_config.heat_setpoint) { g_config.heat_setpoint = c; config_save(); }
            int16_t echo = (int16_t)(c * 100.0f + 0.5f);   /* c >= 17, so +0.5 rounds */
            esp_zb_zcl_set_attribute_val(EP_MAIN, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID, &echo, false);
        }
        if (m->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID) {
            float c = ctrl_clampf(*(int16_t*)m->attribute.data.value / 100.0f, 17.0f, 35.0f);
            if (c != g_config.cool_setpoint) { g_config.cool_setpoint = c; config_save(); }
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
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_HEAT_THRESHOLD, &g_config.heat_threshold, false);
            break;
        case ATTR_COOL_THRESHOLD:
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_COOL_THRESHOLD, &g_config.cool_threshold, false);
            break;
        case ATTR_TRAVEL_TIME_S:
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_TRAVEL_TIME_S, &g_config.travel_time_s, false);
            /* config_apply_custom() may have just re-clamped valve_deadband_pct upward as a
             * side effect (shortening travel raises its stability floor) -- mirror that into
             * the attribute store too, same as every other clamped-tunable echo here. */
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_VALVE_DEADBAND, &g_config.valve_deadband_pct, false);
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
        case ATTR_HEAT_SETPOINT: {
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_HEAT_SETPOINT, &g_config.heat_setpoint, false);
            /* Mirror into the standard thermostat cluster's attribute store so a coordinator
             * reading OccupiedHeatingSetpoint sees the value this device actually regulates
             * from — the thermostat-branch write path above is unreachable, so without this
             * the standard cluster's stored value would silently go stale. */
            int16_t echo = (int16_t)(g_config.heat_setpoint * 100.0f + 0.5f);
            esp_zb_zcl_set_attribute_val(EP_MAIN, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID, &echo, false);
            break;
        }
        case ATTR_COOL_SETPOINT: {
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_COOL_SETPOINT, &g_config.cool_setpoint, false);
            int16_t echo = (int16_t)(g_config.cool_setpoint * 100.0f + 0.5f);
            esp_zb_zcl_set_attribute_val(EP_MAIN, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID, &echo, false);
            break;
        }
        case ATTR_VALVE_DEADBAND:
            esp_zb_zcl_set_attribute_val(EP_MAIN, VALVECTL_CUSTOM_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_VALVE_DEADBAND, &g_config.valve_deadband_pct, false);
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
    /* Without this task nothing pushes temps or status at all: the device stays joined and
     * answers reads, so it looks alive while every value in HA silently freezes. */
    if (xTaskCreate(telemetry_task, "zb_telem", 4096, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate(zb_telem) failed -- aborting for reset+rollback");
        abort();
    }

    esp_zb_stack_main_loop();
}

void zigbee_start(void){
    /* Before the stack, so the previous run's attribute-write tally is reported even if
     * anything below fails. */
    zb_temp_stats_init();
    /* No Zigbee task means no coordinator link ever forms -- the plant runs on its last
     * stored settings with nobody able to see or command it. */
    if (xTaskCreate(zb_task, "zigbee", 8192, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate(zigbee) failed -- aborting for reset+rollback");
        abort();
    }
}
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

/* ---- Temperature attribute write tally -----------------------------------
 * The five measurement endpoints published the ZCL invalid sentinel for 13 h on
 * 2026-08-10 while the thermostat LocalTemperature on EP1 -- fed by temp_centi() from
 * the SAME sensor, in the same telemetry iteration, microseconds apart -- carried a
 * live, rising value. esp_zb_zcl_set_attribute_val() returns a status and every call
 * site discarded it, so which endpoint failed, and why, was unknowable from outside.
 *
 * Three things are recorded per run, because each rules out a different layer:
 *   ok/fail + status  -- did the write get accepted at all;
 *   mismatch/read     -- did the ZCL table actually TAKE the value. SUCCESS on the write
 *                        is not evidence of that, and the table is what the reporting
 *                        engine transmits from;
 *   ticks/sweeps/faults -- what the plant was doing at the time, so "writes all
 *                        succeeded" does not just move the question somewhere else.
 *
 * RTC_NOINIT, and a HISTORY rather than a single previous run. Reaching this board's
 * console costs two resets -- plugging the cable in is one, opening the port is another
 * -- so a one-deep record is destroyed before it can ever be read. That is precisely how
 * the 1-Wire tally for the 13 h outage was lost, and then how the first cut of this tally
 * lost the 2026-08-11 mains-only reproduction. Depth 4 means two resets cannot reach the
 * run under investigation. */
#define ZB_TEMP_STATS_MAGIC 0x2B7E150Au
#define ZB_RUN_HISTORY 4

typedef struct {
    uint32_t ok[5];
    uint32_t fail[5];
    uint32_t mismatch[5];      /* write returned SUCCESS but the table held something else */
    int32_t  last_err[5];      /* esp_zb_zcl_status_t of the most recent failure */
    int16_t  last_val[5];      /* value the write attempted */
    int16_t  last_read[5];     /* what the table held immediately after */
    uint32_t ticks;            /* telemetry iterations completed this run */
    uint32_t sweeps;           /* sensor sweeps completed this run */
    uint16_t faults;           /* control_task_faults() at the last tick */
} zb_run_t;

typedef struct {
    uint32_t magic;
    uint32_t seq;                         /* boot counter; identifies runs across resets */
    zb_run_t cur;
    zb_run_t hist[ZB_RUN_HISTORY];        /* hist[0] = most recently ended run */
} zb_temp_stats_t;

static RTC_NOINIT_ATTR zb_temp_stats_t s_zb;

static const char *const TEMP_EP_NAME[5] = { "supply", "return", "source", "hx_a", "hx_b" };

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
            s_zb.cur.ok[i]++;
            /* Read straight back out of the ZCL table -- the same place the reporting
             * engine sources from. */
            esp_zb_zcl_attr_t *a = esp_zb_zcl_get_attribute(
                map[i].ep, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID);
            int16_t back = (a && a->data_p) ? *(int16_t *)a->data_p : (int16_t)0x7FFF;
            s_zb.cur.last_read[i] = back;
            if (back != v) {
                s_zb.cur.mismatch[i]++;
                if (s_zb.cur.mismatch[i] == 1)
                    ESP_LOGE(TAG, "ep%u %s attr MISMATCH: wrote %d, table holds %d",
                             (unsigned)map[i].ep, TEMP_EP_NAME[i], (int)v, (int)back);
            }
        } else {
            s_zb.cur.fail[i]++;
            s_zb.cur.last_err[i] = (int32_t)st;
            s_zb.cur.last_val[i] = v;
            /* First failure per endpoint is loud; after that the tally carries it, so a
             * persistent fault cannot bury the rest of the log at 30 lines a minute. */
            if (s_zb.cur.fail[i] == 1)
                ESP_LOGE(TAG, "set_attribute_val(ep%u %s) failed: status=0x%02x value=%d",
                         (unsigned)map[i].ep, TEMP_EP_NAME[i], (unsigned)st, (int)v);
        }
    }
    esp_zb_lock_release();
    s_zb.cur.ticks++;
    s_zb.cur.sweeps = sensors_sweep_count();
    s_zb.cur.faults = control_task_faults();
}

/* Console `zbtemp`: the live run plus the last ZB_RUN_HISTORY completed ones. Read the
 * history, not `run` -- by the time you can type this, the run that mattered is two
 * resets back. */
static size_t fmt_run_block(char *o, size_t n, size_t u, const char *label, const zb_run_t *r)
{
    int k = snprintf(o + u, n - u, "%s ticks=%lu sweeps=%lu faults=0x%02x\n",
                     label, (unsigned long)r->ticks, (unsigned long)r->sweeps,
                     (unsigned)r->faults);
    if (k > 0) u += (size_t)k;
    if (u >= n) return n - 1;
    for (int i = 0; i < 5; ++i) {
        k = snprintf(o + u, n - u,
                     "  %-6s ok=%lu fail=%lu mism=%lu err=0x%02lx val=%d read=%d\n",
                     TEMP_EP_NAME[i], (unsigned long)r->ok[i], (unsigned long)r->fail[i],
                     (unsigned long)r->mismatch[i], (unsigned long)(uint32_t)r->last_err[i],
                     (int)r->last_val[i], (int)r->last_read[i]);
        if (k > 0) u += (size_t)k;
        if (u >= n) return n - 1;
    }
    return u;
}

void zigbee_format_temp_stats(char *o, size_t n)
{
    if (!o || n == 0) return;
    char lbl[28];
    snprintf(lbl, sizeof lbl, "run#%lu:", (unsigned long)s_zb.seq);
    size_t u = fmt_run_block(o, n, 0, lbl, &s_zb.cur);
    for (int h = 0; h < ZB_RUN_HISTORY && u < n - 1; ++h) {
        if (!s_zb.hist[h].ticks && !s_zb.hist[h].sweeps) continue;   /* slot never used */
        snprintf(lbl, sizeof lbl, "prev-%d (run#%lu):", h + 1,
                 (unsigned long)(s_zb.seq - (uint32_t)(h + 1)));
        u = fmt_run_block(o, n, u, lbl, &s_zb.hist[h]);
    }
}

static void zb_temp_stats_init(void)
{
    if (s_zb.magic == ZB_TEMP_STATS_MAGIC) {
        for (int h = ZB_RUN_HISTORY - 1; h > 0; --h) s_zb.hist[h] = s_zb.hist[h - 1];
        s_zb.hist[0] = s_zb.cur;
        s_zb.seq++;
        const zb_run_t *p = &s_zb.hist[0];
        ESP_LOGW(TAG, "previous run#%lu: ticks=%lu sweeps=%lu faults=0x%02x",
                 (unsigned long)(s_zb.seq - 1), (unsigned long)p->ticks,
                 (unsigned long)p->sweeps, (unsigned)p->faults);
        for (int i = 0; i < 5; ++i) {
            if (!p->fail[i] && !p->mismatch[i]) continue;
            ESP_LOGW(TAG, "  ep%d %s ok=%lu fail=%lu mism=%lu err=0x%02lx val=%d read=%d",
                     i + 2, TEMP_EP_NAME[i], (unsigned long)p->ok[i], (unsigned long)p->fail[i],
                     (unsigned long)p->mismatch[i], (unsigned long)(uint32_t)p->last_err[i],
                     (int)p->last_val[i], (int)p->last_read[i]);
        }
    } else {
        memset(&s_zb, 0, sizeof s_zb);
        s_zb.magic = ZB_TEMP_STATS_MAGIC;
        s_zb.seq   = 1;
        ESP_LOGI(TAG, "no previous attr-write history (cold power-on)");
    }
    memset(&s_zb.cur, 0, sizeof s_zb.cur);
}

/* configure_reporting_running_mode() below only arms the ZCL reporting engine's
 * passive delta/interval check; live testing found Z2M's `mode` sensor going stale for
 * hours regardless (observed: 6 h stale). So on top of that, push an explicit one-shot
 * report the moment the computed value actually changes. RUNNING_MODE_UNSET (not a valid
 * RunningMode value) forces a push on the first zigbee_push_status() after each join —
 * even when the mode itself hasn't changed since the last join — giving Z2M an
 * authoritative read instead of waiting on the passive engine's next sweep; see the reset
 * in mark_joined(). */
static void push_running_mode_report(uint8_t running_mode)
{
    if (!s_joined || running_mode == s_last_pushed_running_mode) return;

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
