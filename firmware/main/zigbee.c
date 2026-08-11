#include "zigbee.h"
#include "zigbee_internal.h"
#include "control_task.h"
#include "config.h"
#include "ota.h"
#include "zigbee_diag.h"
#include <stdlib.h>
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

static uint32_t now_ms(void){ return (uint32_t)(esp_timer_get_time() / 1000); }

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
    s_attr_heat_threshold = g_config.t.heat_threshold;
    s_attr_cool_threshold = g_config.t.cool_threshold;
    s_attr_travel_time_s  = g_config.t.travel_time_s;
    s_attr_park_pos       = g_config.t.park_pos;
    s_attr_direction_swap = g_config.t.direction_swap;
    s_attr_kp             = g_config.t.kp;
    s_attr_ki             = g_config.t.ki;
    s_attr_gov_high       = g_config.t.gov_high;
    s_attr_gov_low        = g_config.t.gov_low;
    s_attr_alarm_dwell    = g_config.t.alarm_dwell_ms;
    s_attr_resync         = false;
    s_attr_alarm_bitmap   = 0;
    s_attr_fault_bitmap   = 0;
    s_attr_travel_since   = 0.0f;
    s_attr_deadtime_s     = g_config.t.deadtime_s;
    s_attr_pi_deadband    = g_config.t.pi_deadband_k;
    s_attr_heat_setpoint  = g_config.t.heat_setpoint;
    s_attr_cool_setpoint  = g_config.t.cool_setpoint;
    s_attr_valve_deadband = g_config.t.valve_deadband_pct;

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
    zigbee_reset_running_mode_push();   /* force a fresh RunningMode push on the next status */
    /* Reporting setup always runs on join, regardless of what zigbee_on_join()
     * (weak, overridable by e.g. ui.c) does. */
    zigbee_configure_reporting_on_join();
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

static esp_err_t action_handler(esp_zb_core_action_callback_id_t id, const void *msg)
{
    /* Any inbound core action -- attribute write, OTA block, reporting-config, command
     * callback -- proves the coordinator is alive. This is the broadest hook the stack
     * offers; plain attribute READS are answered inside the stack and do NOT reach here,
     * so the liveness signal is only as good as the traffic Z2M actually generates
     * (see Unresolved Q3: availability polling). */
    control_task_note_link_activity();
    if (id == ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID)    return zb_attr_write_cb(msg);
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
    zbdiag_boot_init();
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
