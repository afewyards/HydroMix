#include "config.h"
#include "zigbee.h"
#include "ctrl_core/types.h"
#include <string.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

#define NS "valvectl"
static const char *TAG = "config";
config_t g_config;

static const config_t DEFAULTS = {
    .heat_threshold = 28.0f, .cool_threshold = 16.0f, .hysteresis = 2.0f,
    .heat_setpoint = 35.0f,  .cool_setpoint = 18.0f,  .park_pos = 50.0f,
    .travel_time_s = 120,    .direction_swap = false,
    .kp = 2.8f, .ki = 0.9f,  .gov_high = 36.0f, .gov_low = 16.0f,
    .alarm_dwell_ms = 300000, .enter_dwell_ms = 60000, .leave_dwell_ms = 420000,
    .deadtime_s = 30.0f, .pi_deadband_k = 0.25f, .cfg_version = CONFIG_VERSION,
};

/* v1 blob layout (≤1.0.7): ki was %/K per 10 s cycle, no transit-hold fields.
 * Field order/types must stay byte-identical to the old config_t. */
typedef struct {
    float heat_threshold, cool_threshold, hysteresis, heat_setpoint, cool_setpoint, park_pos;
    uint32_t travel_time_s; bool direction_swap;
    float kp, ki, gov_high, gov_low;
    uint32_t alarm_dwell_ms, enter_dwell_ms, leave_dwell_ms;
} config_v1_t;

void config_load(void)
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    g_config = DEFAULTS;
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) { ESP_LOGW(TAG,"no ns, using defaults"); return; }

    size_t sz = 0;
    if (nvs_get_blob(h, "cfg", NULL, &sz) == ESP_OK) {
        if (sz == sizeof(config_t)) {
            config_t tmp;
            if (nvs_get_blob(h, "cfg", &tmp, &sz) == ESP_OK && tmp.cfg_version == CONFIG_VERSION)
                g_config = tmp;
        } else if (sz == sizeof(config_v1_t)) {
            config_v1_t v1;
            if (nvs_get_blob(h, "cfg", &v1, &sz) == ESP_OK) {
                memcpy(&g_config, &v1, sizeof(v1));       /* common prefix, same layout */
                g_config.kp = ctrl_clampf(v1.kp, 0.5f, 15.0f);
                g_config.ki = ctrl_clampf(v1.ki * 6.0f, 0.0f, 5.0f);  /* per-10s-cycle -> per-min */
                g_config.deadtime_s = DEFAULTS.deadtime_s;
                g_config.pi_deadband_k = DEFAULTS.pi_deadband_k;
                g_config.cfg_version = CONFIG_VERSION;
                nvs_close(h);
                ESP_LOGI(TAG, "cfg migrated v1->v2 (ki %.2f/cycle -> %.2f/min)", v1.ki, g_config.ki);
                config_save();
                return;
            }
        }
    }
    nvs_close(h);
}

esp_err_t config_save(void)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open(NS, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    e = nvs_set_blob(h, "cfg", &g_config, sizeof(config_t));
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e;
}

void config_factory_reset(void)
{
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READWRITE, &h) == ESP_OK) { nvs_erase_all(h); nvs_commit(h); nvs_close(h); }
    g_config = DEFAULTS;
}

/* Maps a zigbee custom-cluster attribute write (ATTR_* in zigbee.h) onto the
 * matching g_config field, mirroring the clamps in ctrl_core/config_map.c's
 * tunable_apply(), then persists. Read-only/unknown attrs are ignored. */
void config_apply_custom(uint16_t attr_id, const void *val)
{
    switch (attr_id) {
    case ATTR_HEAT_THRESHOLD: g_config.heat_threshold = *(const float*)val; break;
    case ATTR_COOL_THRESHOLD: g_config.cool_threshold = *(const float*)val; break;
    case ATTR_TRAVEL_TIME_S:  g_config.travel_time_s  = *(const uint32_t*)val; break;
    case ATTR_PARK_POS:       g_config.park_pos = ctrl_clampf(*(const float*)val, 0.0f, 100.0f); break;
    case ATTR_DIRECTION_SWAP: g_config.direction_swap = *(const bool*)val; break;
    case ATTR_KP:             g_config.kp = ctrl_clampf(*(const float*)val, 0.5f, 15.0f); break;
    case ATTR_KI:             g_config.ki = ctrl_clampf(*(const float*)val, 0.0f, 5.0f); break;
    case ATTR_GOV_HIGH:       g_config.gov_high = *(const float*)val; break;
    case ATTR_GOV_LOW:        g_config.gov_low = *(const float*)val; break;
    case ATTR_ALARM_DWELL:    g_config.alarm_dwell_ms = *(const uint32_t*)val; break;
    case ATTR_DEADTIME_S:     g_config.deadtime_s = ctrl_clampf(*(const float*)val, 0.0f, 120.0f); break;
    case ATTR_PI_DEADBAND:    g_config.pi_deadband_k = ctrl_clampf(*(const float*)val, 0.0f, 1.0f); break;
    default: return; /* read-only or unknown — nothing to persist */
    }
    config_save();
}
