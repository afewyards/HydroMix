#include "config.h"
#include "zigbee.h"
#include <string.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

#define NS "valvectl"
#define KEY_WATER "water"
static const char *TAG = "config";
config_t g_config;

static config_t config_defaults(void)
{
    config_t c = { .cfg_version = CONFIG_VERSION };
    tunable_cfg_defaults(&c.t);
    return c;
}

/* v1 blob layout (≤1.0.7): ki was %/K per 10 s cycle, no transit-hold fields, no
 * cfg_version. Field order/types must stay byte-identical to the old config_t.
 * Since v1 has nothing after leave_dwell_ms, a raw memcpy of these bytes onto the front
 * of a fresh (DEFAULTS-initialised) config_t correctly leaves every later field at its
 * default -- see the v1 branch in config_load(). */
typedef struct {
    float heat_threshold, cool_threshold, hysteresis, heat_setpoint, cool_setpoint, park_pos;
    uint32_t travel_time_s; bool direction_swap;
    float kp, ki, gov_high, gov_low;
    uint32_t alarm_dwell_ms, enter_dwell_ms, leave_dwell_ms;
} config_v1_t;

/* v2 blob layout (1.3.0-1.3.1): everything 1.4.0 has except valve_deadband_pct, which
 * 1.4.0 inserts BEFORE cfg_version. Unlike v1, v2 already ends in cfg_version -- so the
 * v1 trick (raw memcpy of the whole blob onto a fresh config_t) would land v2's old
 * cfg_version bits in valve_deadband_pct's slot (reinterpreted as a bogus float) instead
 * of at the new, shifted cfg_version offset. Migrated field-by-field instead; see the v2
 * branch in config_load(). */
typedef struct {
    float heat_threshold, cool_threshold, hysteresis, heat_setpoint, cool_setpoint, park_pos;
    uint32_t travel_time_s; bool direction_swap;
    float kp, ki, gov_high, gov_low;
    uint32_t alarm_dwell_ms, enter_dwell_ms, leave_dwell_ms;
    float deadtime_s, pi_deadband_k;
    uint32_t cfg_version;
} config_v2_t;
/* Fixed historical tag for the v2 layout -- NOT CONFIG_VERSION, which now means v3.
 * Guards the v2 branch below the same way tmp.cfg_version==CONFIG_VERSION guards the
 * v3 branch, so a blob that merely happens to be 72 bytes but isn't really a v2 cfg
 * blob doesn't get silently accepted. */
#define CONFIG_VERSION_V2 2u

/* Defensive pass after every successful load path so a stale, hand-edited or bit-flipped
 * NVS blob can never leave g_config outside the bounds every write path enforces. */
static void clamp_config(config_t *c) { tunable_clamp_all(&c->t); }

void config_load(void)
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    g_config = config_defaults();
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) { ESP_LOGW(TAG,"no ns, using defaults"); return; }

    size_t sz = 0;
    if (nvs_get_blob(h, "cfg", NULL, &sz) == ESP_OK) {
        if (sz == sizeof(config_t)) {
            config_t tmp;
            if (nvs_get_blob(h, "cfg", &tmp, &sz) == ESP_OK && tmp.cfg_version == CONFIG_VERSION) {
                g_config = tmp;
                clamp_config(&g_config);
            } else {
                ESP_LOGW(TAG, "cfg blob v3-sized but unreadable or version mismatch, using defaults");
            }
        } else if (sz == sizeof(config_v2_t)) {
            config_v2_t v2;
            if (nvs_get_blob(h, "cfg", &v2, &sz) == ESP_OK && v2.cfg_version == CONFIG_VERSION_V2) {
                /* Field-by-field (not a raw memcpy prefix, see config_v2_t's comment above):
                 * v2's cfg_version sits where v3's valve_deadband_pct now lives. */
                g_config.t.heat_threshold = v2.heat_threshold;
                g_config.t.cool_threshold = v2.cool_threshold;
                g_config.t.hysteresis     = v2.hysteresis;
                g_config.t.heat_setpoint  = v2.heat_setpoint;
                g_config.t.cool_setpoint  = v2.cool_setpoint;
                g_config.t.park_pos       = v2.park_pos;
                g_config.t.travel_time_s  = v2.travel_time_s;
                g_config.t.direction_swap = v2.direction_swap;
                g_config.t.kp             = v2.kp;
                g_config.t.ki             = v2.ki;
                g_config.t.gov_high       = v2.gov_high;
                g_config.t.gov_low        = v2.gov_low;
                g_config.t.alarm_dwell_ms = v2.alarm_dwell_ms;
                g_config.t.enter_dwell_ms = v2.enter_dwell_ms;
                g_config.t.leave_dwell_ms = v2.leave_dwell_ms;
                g_config.t.deadtime_s     = v2.deadtime_s;
                g_config.t.pi_deadband_k  = v2.pi_deadband_k;
                g_config.t.valve_deadband_pct = config_defaults().t.valve_deadband_pct;   /* new in v3, not in v2 blob */
                clamp_config(&g_config);
                g_config.cfg_version = CONFIG_VERSION;
                nvs_close(h);
                ESP_LOGI(TAG, "cfg migrated v2->v3 (valve_deadband_pct defaulted to %.2f%%)",
                         g_config.t.valve_deadband_pct);
                config_save();
                return;
            } else {
                ESP_LOGW(TAG, "cfg v2 blob unreadable or version mismatch, using defaults");
            }
        } else if (sz == sizeof(config_v1_t)) {
            config_v1_t v1;
            if (nvs_get_blob(h, "cfg", &v1, &sz) == ESP_OK) {
                memcpy(&g_config, &v1, sizeof(v1));       /* common prefix, same layout */
                config_t d = config_defaults();
                g_config.t.ki = v1.ki * 6.0f;             /* per-10s-cycle -> per-min */
                g_config.t.deadtime_s = d.t.deadtime_s;
                g_config.t.pi_deadband_k = d.t.pi_deadband_k;
                g_config.t.valve_deadband_pct = d.t.valve_deadband_pct;
                clamp_config(&g_config);
                g_config.cfg_version = CONFIG_VERSION;
                nvs_close(h);
                ESP_LOGI(TAG, "cfg migrated v1->v3 (ki %.2f/cycle -> %.2f/min)", v1.ki, g_config.t.ki);
                config_save();
                return;
            } else {
                ESP_LOGW(TAG, "cfg v1 blob unreadable, using defaults");
            }
        } else {
            ESP_LOGW(TAG, "cfg blob size %u matches neither v1 (%u), v2 (%u) nor v3 (%u) layout, using defaults",
                     (unsigned)sz, (unsigned)sizeof(config_v1_t), (unsigned)sizeof(config_v2_t), (unsigned)sizeof(config_t));
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
    g_config = config_defaults();
}

/* tunable_from_attr() hardcodes these numbers because ctrl_core cannot include zigbee.h.
 * Pin them here, where both are visible, so a renumbering breaks the build. */
_Static_assert(ATTR_HEAT_THRESHOLD == 0x0000, "tunable_from_attr table is stale");
_Static_assert(ATTR_ALARM_DWELL    == 0x0009, "tunable_from_attr table is stale");
_Static_assert(ATTR_DEADTIME_S     == 0x000E, "tunable_from_attr table is stale");
_Static_assert(ATTR_HEAT_SETPOINT  == 0x0010, "tunable_from_attr table is stale");
_Static_assert(ATTR_COOL_SETPOINT  == 0x0011, "tunable_from_attr table is stale");
_Static_assert(ATTR_VALVE_DEADBAND == 0x0012, "tunable_from_attr table is stale");

/* Maps a zigbee custom-cluster attribute write onto the matching tunable and persists.
 * The clamp ranges live in ctrl_core/config_map.c's SPEC table -- this function no longer
 * restates them, which is what stopped the two from drifting. Read-only and unknown
 * attributes are ignored. */
void config_apply_custom(uint16_t attr_id, const void *val)
{
    tunable_id_t id;
    if (!tunable_from_attr(attr_id, &id)) return;

    config_t before = g_config;
    tunable_apply(&g_config.t, id, val);
    /* Skip the flash write when the clamped value didn't actually change g_config (a
     * write clamped back to its current value, or an unchanged re-write) -- avoids
     * wearing the NVS partition on no-op traffic. */
    if (memcmp(&before, &g_config, sizeof g_config) != 0) config_save();
}

bool config_water_running_load(void)
{
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) return false;
    uint8_t v = 0;
    esp_err_t e = nvs_get_u8(h, KEY_WATER, &v);
    nvs_close(h);
    if (e != ESP_OK) {
        ESP_LOGI(TAG, "no persisted water_running, defaulting OFF (park)");
        return false;
    }
    return v != 0;
}

void config_water_running_save(bool on)
{
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READWRITE, &h) != ESP_OK) return;
    if (nvs_set_u8(h, KEY_WATER, on ? 1 : 0) == ESP_OK) nvs_commit(h);
    nvs_close(h);
}
