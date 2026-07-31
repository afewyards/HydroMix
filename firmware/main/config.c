#include "config.h"
#include "zigbee.h"
#include "ctrl_core/types.h"
#include "ctrl_core/interlock.h"
#include <math.h>
#include <string.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

#define NS "valvectl"
#define KEY_WATER "water"
static const char *TAG = "config";
config_t g_config;

static const config_t DEFAULTS = {
    .heat_threshold = 28.0f, .cool_threshold = 16.0f, .hysteresis = 2.0f,
    .heat_setpoint = 35.0f,  .cool_setpoint = 18.0f,  .park_pos = 50.0f,
    .travel_time_s = 120,    .direction_swap = false,
    .kp = 2.8f, .ki = 0.9f,  .gov_high = 36.0f, .gov_low = 16.0f,
    .alarm_dwell_ms = 300000, .enter_dwell_ms = 60000, .leave_dwell_ms = 420000,
    .deadtime_s = 30.0f, .pi_deadband_k = 0.25f, .valve_deadband_pct = 1.0f,
    .cfg_version = CONFIG_VERSION,
};

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

/* NaN -> reject (keep cur); finite -> clamp to [lo,hi]. A NaN written over
 * Zigbee (or surviving in a corrupt NVS blob) makes both ctrl_clampf()
 * comparisons false and passes through unmodified -- this is the gate that
 * stops it before it ever reaches g_config. */
static float sane_f(float cur, float v, float lo, float hi)
{
    if (isnan(v)) return cur;
    return ctrl_clampf(v, lo, hi);
}

/* Range-only clamp for u32 tunables (NaN is a float-only concept). */
static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* The interlock enforces a minimum drive pulse (INTERLOCK_MIN_PULSE_MS): the smallest
 * motion increment the motor can ever make is (min_pulse/travel_time)*100 %. The
 * deadband must exceed half that quantum or the position estimator ping-pongs across
 * the band edge on every single pulse -- reviewer simulation found a constant floor
 * below this limit-cycles the motor at 16.7% duty with a resync roughly every 10 min.
 * 1.2x safety margin on top of the bare half-quantum. Written off
 * INTERLOCK_MIN_PULSE_MS (not a hardcoded 120) so it tracks any future change to the
 * interlock's minimum pulse; at the shipped default travel_time_s=120 this evaluates
 * to exactly 1.0 %, i.e. DEFAULTS.valve_deadband_pct sits right at the floor. */
static float valve_deadband_floor_pct(uint32_t travel_time_s)
{
    float q = 1.2f * (INTERLOCK_MIN_PULSE_MS / 1000.0f) * 100.0f / (2.0f * (float)travel_time_s);
    return q > 0.2f ? q : 0.2f;
}

/* Defensive clamp applied after every successful load path (fresh v2 blob or v1
 * migration) so a stale, hand-edited, or bit-flipped NVS blob can never leave
 * g_config outside the bounds every write path (config_apply_custom / ctrl_core's
 * tunable_apply) already enforces. NaN fields fall back to the shipped default
 * (there is no "current" value to preserve on a fresh load). */
static void clamp_config(config_t *c)
{
    c->heat_threshold = sane_f(DEFAULTS.heat_threshold, c->heat_threshold, 10.0f, 60.0f);
    c->cool_threshold = sane_f(DEFAULTS.cool_threshold, c->cool_threshold, 0.0f, 40.0f);
    /* release band is 35/17 (control_task.c) — trip thresholds must stay strictly outside it or the
     * governor limit-cycles; observed live (gov_low written as 17, sitting on the band edge) */
    c->gov_high       = sane_f(DEFAULTS.gov_high,       c->gov_high,       36.0f, 60.0f);
    c->gov_low        = sane_f(DEFAULTS.gov_low,        c->gov_low,        0.0f, 16.0f);
    c->park_pos       = sane_f(DEFAULTS.park_pos,       c->park_pos,       0.0f, 100.0f);
    c->heat_setpoint  = sane_f(DEFAULTS.heat_setpoint,  c->heat_setpoint,  17.0f, 35.0f);
    c->cool_setpoint  = sane_f(DEFAULTS.cool_setpoint,  c->cool_setpoint,  17.0f, 35.0f);
    c->kp             = sane_f(DEFAULTS.kp,             c->kp,             0.5f, 15.0f);
    c->ki             = sane_f(DEFAULTS.ki,             c->ki,             0.0f, 5.0f);
    c->deadtime_s     = sane_f(DEFAULTS.deadtime_s,     c->deadtime_s,     0.0f, 120.0f);
    c->pi_deadband_k  = sane_f(DEFAULTS.pi_deadband_k,  c->pi_deadband_k,  0.0f, 1.0f);
    c->hysteresis     = sane_f(DEFAULTS.hysteresis,     c->hysteresis,     0.0f, 10.0f);
    /* travel_time_s clamps FIRST: valve_deadband_pct's floor is derived from it, so the
     * floor must be computed from the final, in-range travel value. */
    c->travel_time_s  = clamp_u32(c->travel_time_s,  30,    600);
    /* Ceiling of 5.0 caps how far off-target the valve is allowed to sit before it
     * moves; floor is the stability minimum from valve_deadband_floor_pct() above (or
     * 0.2, whichever is greater). The "cur" fallback passed to sane_f is itself clamped
     * into the same dynamic range so a NaN-corrupted stored value can't fall back to a
     * stale 1.0 that's below the floor for a short travel_time_s. */
    float deadband_lo = valve_deadband_floor_pct(c->travel_time_s);
    c->valve_deadband_pct = sane_f(ctrl_clampf(DEFAULTS.valve_deadband_pct, deadband_lo, 5.0f),
                                   c->valve_deadband_pct, deadband_lo, 5.0f);
    c->alarm_dwell_ms = clamp_u32(c->alarm_dwell_ms, 10000, 3600000);
    c->enter_dwell_ms = clamp_u32(c->enter_dwell_ms, 10000, 3600000);
    c->leave_dwell_ms = clamp_u32(c->leave_dwell_ms, 10000, 7200000);
}

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
                g_config.heat_threshold = v2.heat_threshold;
                g_config.cool_threshold = v2.cool_threshold;
                g_config.hysteresis     = v2.hysteresis;
                g_config.heat_setpoint  = v2.heat_setpoint;
                g_config.cool_setpoint  = v2.cool_setpoint;
                g_config.park_pos       = v2.park_pos;
                g_config.travel_time_s  = v2.travel_time_s;
                g_config.direction_swap = v2.direction_swap;
                g_config.kp             = v2.kp;
                g_config.ki             = v2.ki;
                g_config.gov_high       = v2.gov_high;
                g_config.gov_low        = v2.gov_low;
                g_config.alarm_dwell_ms = v2.alarm_dwell_ms;
                g_config.enter_dwell_ms = v2.enter_dwell_ms;
                g_config.leave_dwell_ms = v2.leave_dwell_ms;
                g_config.deadtime_s     = v2.deadtime_s;
                g_config.pi_deadband_k  = v2.pi_deadband_k;
                g_config.valve_deadband_pct = DEFAULTS.valve_deadband_pct;   /* new in v3, not in v2 blob */
                clamp_config(&g_config);
                g_config.cfg_version = CONFIG_VERSION;
                nvs_close(h);
                ESP_LOGI(TAG, "cfg migrated v2->v3 (valve_deadband_pct defaulted to %.2f%%)",
                         g_config.valve_deadband_pct);
                config_save();
                return;
            } else {
                ESP_LOGW(TAG, "cfg v2 blob unreadable or version mismatch, using defaults");
            }
        } else if (sz == sizeof(config_v1_t)) {
            config_v1_t v1;
            if (nvs_get_blob(h, "cfg", &v1, &sz) == ESP_OK) {
                memcpy(&g_config, &v1, sizeof(v1));       /* common prefix, same layout */
                g_config.ki = v1.ki * 6.0f;               /* per-10s-cycle -> per-min */
                g_config.deadtime_s = DEFAULTS.deadtime_s;
                g_config.pi_deadband_k = DEFAULTS.pi_deadband_k;
                g_config.valve_deadband_pct = DEFAULTS.valve_deadband_pct;
                clamp_config(&g_config);
                g_config.cfg_version = CONFIG_VERSION;
                nvs_close(h);
                ESP_LOGI(TAG, "cfg migrated v1->v3 (ki %.2f/cycle -> %.2f/min)", v1.ki, g_config.ki);
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
    g_config = DEFAULTS;
}

/* Maps a zigbee custom-cluster attribute write (ATTR_* in zigbee.h) onto the
 * matching g_config field, mirroring the clamps in ctrl_core/config_map.c's
 * tunable_apply(), then persists. Read-only/unknown attrs are ignored. */
void config_apply_custom(uint16_t attr_id, const void *val)
{
    config_t before = g_config;
    switch (attr_id) {
    case ATTR_HEAT_THRESHOLD: g_config.heat_threshold = sane_f(g_config.heat_threshold, *(const float*)val, 10.0f, 60.0f); break;
    case ATTR_COOL_THRESHOLD: g_config.cool_threshold = sane_f(g_config.cool_threshold, *(const float*)val, 0.0f, 40.0f); break;
    case ATTR_TRAVEL_TIME_S: {
        g_config.travel_time_s = clamp_u32(*(const uint32_t*)val, 30, 600);
        /* Shortening travel raises valve_deadband_pct's stability floor (see
         * valve_deadband_floor_pct()) -- re-clamp so an already-set deadband can't be
         * left stranded below the new, higher floor. */
        float floor_pct = valve_deadband_floor_pct(g_config.travel_time_s);
        g_config.valve_deadband_pct = ctrl_clampf(g_config.valve_deadband_pct, floor_pct, 5.0f);
        break;
    }
    case ATTR_PARK_POS:       g_config.park_pos = sane_f(g_config.park_pos, *(const float*)val, 0.0f, 100.0f); break;
    case ATTR_DIRECTION_SWAP: g_config.direction_swap = *(const bool*)val; break;
    case ATTR_KP:             g_config.kp = sane_f(g_config.kp, *(const float*)val, 0.5f, 15.0f); break;
    case ATTR_KI:             g_config.ki = sane_f(g_config.ki, *(const float*)val, 0.0f, 5.0f); break;
    /* release band is 35/17 (control_task.c) — trip thresholds must stay strictly outside it or the
     * governor limit-cycles; observed live (gov_low written as 17, sitting on the band edge) */
    case ATTR_GOV_HIGH:       g_config.gov_high = sane_f(g_config.gov_high, *(const float*)val, 36.0f, 60.0f); break;
    case ATTR_GOV_LOW:        g_config.gov_low = sane_f(g_config.gov_low, *(const float*)val, 0.0f, 16.0f); break;
    case ATTR_ALARM_DWELL:    g_config.alarm_dwell_ms = clamp_u32(*(const uint32_t*)val, 10000, 3600000); break;
    case ATTR_DEADTIME_S:     g_config.deadtime_s = sane_f(g_config.deadtime_s, *(const float*)val, 0.0f, 120.0f); break;
    case ATTR_PI_DEADBAND:    g_config.pi_deadband_k = sane_f(g_config.pi_deadband_k, *(const float*)val, 0.0f, 1.0f); break;
    case ATTR_VALVE_DEADBAND: g_config.valve_deadband_pct = sane_f(g_config.valve_deadband_pct, *(const float*)val,
                                  valve_deadband_floor_pct(g_config.travel_time_s), 5.0f); break;
    /* Custom-cluster write path for the two regulation targets — the standard thermostat
     * cluster rejects them (ZBOSS enforces heat<=cool-deadband; this device's independent
     * seasonal targets 35/18 always violate it). Same clamp as clamp_config()/tunable_apply(). */
    case ATTR_HEAT_SETPOINT:  g_config.heat_setpoint = sane_f(g_config.heat_setpoint, *(const float*)val, 17.0f, 35.0f); break;
    case ATTR_COOL_SETPOINT:  g_config.cool_setpoint = sane_f(g_config.cool_setpoint, *(const float*)val, 17.0f, 35.0f); break;
    default: return; /* read-only or unknown — nothing to persist */
    }
    /* Skip the flash write when the clamped/applied value didn't actually change
     * g_config (e.g. a write that got clamped back to its current value, or an
     * unchanged re-write) — avoids wearing the NVS partition on no-op traffic. */
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
