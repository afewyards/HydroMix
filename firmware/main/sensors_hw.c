#include "sensors_hw.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "onewire_bus.h"
#include "onewire_crc.h"
#include "ctrl_core/sensor_policy.h"

static const char *TAG = "sensors";

/* GPIO per sensor id (spec §2): SUPPLY, RETURN, SOURCE, HX_A, HX_B.
 *
 * HX_A and HX_B are intentionally SWAPPED relative to spec §2 (HX_A reads GPIO19,
 * HX_B reads GPIO18) because the PCB layout swapped the HX-A/HX-B connectors
 * relative to the schematic. This mapping is correct for the fabricated board and
 * must NOT be reverted. Matters beyond labelling: mode detection keys off HX_A, so
 * without this swap heating/cooling selection would follow the wrong probe. */
static const int PIN[SENS_COUNT] = { 0, 1, 10, 19, 18 };
#define SWEEP_PERIOD_MS 10000
#define CONVERT_MS       750
#define MAX_RETRY          3
#define EMA_TAU_S       40.0f
/* Belt-and-braces for a sweep that stops producing readings without failing them
 * (task starved, RMT wedged, sweep_task deleted). Independent of the latch state:
 * a reading whose last good sweep is older than this is reported faulted, so control
 * degrades to park instead of silently integrating frozen data. 3 sweep periods plus
 * 5 s of slack. */
#define SENSOR_STALE_MS (3u * SWEEP_PERIOD_MS + 5000u)   /* 35 s */

/* DS18B20 ROM/function commands (Skip ROM: single sensor per wire). */
#define CMD_SKIP_ROM  0xCC
#define CMD_CONVERT_T 0x44
#define CMD_READ_SCR  0xBE

static sensor_reading_t     g_read[SENS_COUNT];
static sensor_fault_state_t g_fault_state[SENS_COUNT];
static SemaphoreHandle_t    g_lock;

static bool onewire_convert(int gpio)
{
    onewire_bus_handle_t bus = NULL;
    onewire_bus_config_t bcfg = { .bus_gpio_num = gpio };
    onewire_bus_rmt_config_t rmt = { .max_rx_bytes = 10 };
    if (onewire_new_bus_rmt(&bcfg, &rmt, &bus) != ESP_OK) return false;
    bool ok = (onewire_bus_reset(bus) == ESP_OK);
    if (ok) { uint8_t cmd[2] = { CMD_SKIP_ROM, CMD_CONVERT_T }; ok = (onewire_bus_write_bytes(bus, cmd, 2) == ESP_OK); }
    onewire_bus_del(bus);      /* release the RMT pair for the next GPIO */
    return ok;
}

static bool onewire_read_temp(int gpio, float *out, uint16_t *raw_out)
{
    onewire_bus_handle_t bus = NULL;
    onewire_bus_config_t bcfg = { .bus_gpio_num = gpio };
    onewire_bus_rmt_config_t rmt = { .max_rx_bytes = 10 };
    if (onewire_new_bus_rmt(&bcfg, &rmt, &bus) != ESP_OK) return false;
    bool ok = (onewire_bus_reset(bus) == ESP_OK);
    uint8_t scr[9] = {0};
    if (ok) { uint8_t cmd[2] = { CMD_SKIP_ROM, CMD_READ_SCR }; ok = (onewire_bus_write_bytes(bus, cmd, 2) == ESP_OK); }
    if (ok) ok = (onewire_bus_read_bytes(bus, scr, 9) == ESP_OK);
    onewire_bus_del(bus);
    if (!ok) return false;
    if (onewire_crc8(0, scr, 8) != scr[8]) return false;   /* CRC */
    uint16_t raw = (uint16_t)((scr[1] << 8) | scr[0]);
    *raw_out = raw;
    *out = (int16_t)raw / 16.0f;                            /* 12-bit */
    return true;
}

static void apply_ema(sensor_id_t id, float v)
{
    float dt = SWEEP_PERIOD_MS / 1000.0f;
    float alpha = dt / (EMA_TAU_S + dt);                   /* ~0.2 at 10 s / 40 s */
    bool filtered = (id == SENS_SOURCE || id == SENS_RETURN);
    xSemaphoreTake(g_lock, portMAX_DELAY);
    bool just_cleared = sensor_fault_update(&g_fault_state[id], true);
    bool reseed = !filtered || just_cleared || g_read[id].last_ok_ms == 0;
    g_read[id].value_c = v;
    g_read[id].value_filt_c = sensor_ema_step(g_read[id].value_filt_c, v, alpha, reseed);
    g_read[id].last_ok_ms = (uint32_t)(esp_timer_get_time() / 1000);
    g_read[id].fault = g_fault_state[id].faulted;
    xSemaphoreGive(g_lock);
}

static void mark_fail(sensor_id_t id)
{
    xSemaphoreTake(g_lock, portMAX_DELAY);
    sensor_fault_update(&g_fault_state[id], false);
    g_read[id].fault = g_fault_state[id].faulted;
    xSemaphoreGive(g_lock);
}

static void sweep_task(void *arg)
{
    /* Subscribed to the task WDT like the control and valve tasks: an RMT/1-Wire wedge
     * here used to be invisible (readings simply froze) rather than causing the reset +
     * rollback the watchdog exists to produce. One iteration is ~10 s (750 ms convert +
     * 9.25 s delay), well inside the 30 s budget. */
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    for (;;) {
        esp_task_wdt_reset();
        /* Phase 1: kick a conversion on each GPIO in turn (line released between). */
        for (int i = 0; i < SENS_COUNT; ++i) onewire_convert(PIN[i]);
        vTaskDelay(pdMS_TO_TICKS(CONVERT_MS));             /* single shared wait */
        /* Phase 2: read each scratchpad, with retries. POR scratchpad (85.0 C,
         * raw 0x0550) means "never converted" and counts as a failed read. */
        for (int i = 0; i < SENS_COUNT; ++i) {
            float v; uint16_t raw; bool ok = false;
            for (int r = 0; r < MAX_RETRY && !ok; ++r)
                ok = onewire_read_temp(PIN[i], &v, &raw) && !sensor_fault_is_por_raw(raw);
            if (ok) apply_ema((sensor_id_t)i, v); else mark_fail((sensor_id_t)i);
        }
        vTaskDelay(pdMS_TO_TICKS(SWEEP_PERIOD_MS - CONVERT_MS));
    }
}

void sensors_start(void)
{
    g_lock = xSemaphoreCreateMutex();
    for (int i = 0; i < SENS_COUNT; ++i) {
        g_read[i].fault = true;
        g_read[i].last_ok_ms = 0;
        /* Boot as already-latched-faulted, not zero-init, so a dead probe stays
         * faulted from t=0 instead of reporting healthy after its first failed
         * read (fail_streak would otherwise start below the latch threshold). */
        g_fault_state[i] = (sensor_fault_state_t){
            .fail_streak = SENSOR_FAULT_AFTER, .good_streak = 0, .faulted = true };
    }
    xTaskCreate(sweep_task, "sensors", 4096, NULL, 5, NULL);
}

sensor_reading_t sensors_get(sensor_id_t id)
{
    xSemaphoreTake(g_lock, portMAX_DELAY);
    sensor_reading_t r = g_read[id];
    xSemaphoreGive(g_lock);
    return r;
}

/* One locked pass. Taking g_lock five times (once per sensors_get) let the sweep task
 * interleave and hand the control loop a mix of pre- and post-sweep fault state. */
void sensors_fill_faults(sensor_faults_t *o)
{
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    bool f[SENS_COUNT];

    xSemaphoreTake(g_lock, portMAX_DELAY);
    for (int i = 0; i < SENS_COUNT; ++i) {
        /* last_ok_ms == 0 means "never read a good value"; sensors_start() already boots
         * that sensor as latched-faulted, so no staleness test is needed (and applying
         * one would be wrong -- now - 0 is just uptime). */
        bool stale = (g_read[i].last_ok_ms != 0) &&
                     ((now - g_read[i].last_ok_ms) > SENSOR_STALE_MS);
        f[i] = g_read[i].fault || stale;
    }
    xSemaphoreGive(g_lock);

    o->supply = f[SENS_SUPPLY];
    o->ret    = f[SENS_RETURN];
    o->source = f[SENS_SOURCE];
    o->hx_a   = f[SENS_HX_A];
    o->hx_b   = f[SENS_HX_B];
}
