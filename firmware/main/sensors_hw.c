#include "sensors_hw.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_attr.h"
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

/* ---- Failure attribution -------------------------------------------------
 * onewire_read_temp() used to collapse six unrelated failures into a bare
 * `return false`, which is what made an intermittent field fault
 * undiagnosable: RMT exhaustion, an electrically absent probe, line noise and
 * a convert window that ran short all look identical from the sweep loop, yet
 * each has a different fix. Count them apart.
 *
 * The tallies live in RTC_NOINIT_ATTR to defeat an observability trap on this
 * board: opening the USB-JTAG port resets the chip (rst:0x15 USB_UART_HPSYS,
 * reproduced 5/5), so attaching a console to inspect a misbehaving run is the
 * one action guaranteed to destroy the run being inspected. RTC RAM survives
 * that warm reset, so sensors_start() can print the PREVIOUS run's tally --
 * plugging the cable in then yields the history you attached it to get. A true
 * power-on leaves the region uninitialised, which the magic detects. */
typedef enum {
    OW_OK = 0, OW_BUS, OW_RESET, OW_WRITE, OW_READ, OW_CRC, OW_POR, OW_REASONS
} ow_reason_t;

/* Indexed by ow_reason_t. "por85" is the DS18B20 power-on scratchpad (85.0 C):
 * the part answered fine but its conversion had not finished, which indicts
 * CONVERT_MS rather than the wiring -- the one failure here that is a firmware
 * timing bug rather than a plant problem. */
static const char *const OW_REASON[OW_REASONS] = {
    "ok", "rmt", "reset", "write", "read", "crc", "por85"
};
static const char *const SENS_NAME[SENS_COUNT] = {
    "supply", "return", "source", "hx_a", "hx_b"
};

#define SENSOR_STATS_MAGIC 0x5EA5C0DEu

typedef struct {
    uint32_t magic;
    uint32_t sweeps;
    uint32_t ok[SENS_COUNT];
    uint32_t convert_fail[SENS_COUNT];
    uint32_t fail[SENS_COUNT][OW_REASONS];   /* [OW_OK] stays zero */
} sensor_stats_t;

static RTC_NOINIT_ATTR sensor_stats_t s_stats;
static sensor_stats_t s_prev;        /* previous run, snapshotted at boot */
static bool           s_prev_valid;

static sensor_reading_t     g_read[SENS_COUNT];
static sensor_fault_state_t g_fault_state[SENS_COUNT];
static sensor_sweep_state_t g_sweep;      /* guarded by g_lock, like g_read */
static SemaphoreHandle_t    g_lock;

static ow_reason_t onewire_convert(int gpio)
{
    onewire_bus_handle_t bus = NULL;
    onewire_bus_config_t bcfg = { .bus_gpio_num = gpio };
    onewire_bus_rmt_config_t rmt = { .max_rx_bytes = 10 };
    if (onewire_new_bus_rmt(&bcfg, &rmt, &bus) != ESP_OK) return OW_BUS;
    ow_reason_t r = OW_OK;
    if (onewire_bus_reset(bus) != ESP_OK) r = OW_RESET;
    if (r == OW_OK) {
        uint8_t cmd[2] = { CMD_SKIP_ROM, CMD_CONVERT_T };
        if (onewire_bus_write_bytes(bus, cmd, 2) != ESP_OK) r = OW_WRITE;
    }
    onewire_bus_del(bus);      /* release the RMT pair for the next GPIO */
    return r;
}

static ow_reason_t onewire_read_temp(int gpio, float *out, uint16_t *raw_out)
{
    onewire_bus_handle_t bus = NULL;
    onewire_bus_config_t bcfg = { .bus_gpio_num = gpio };
    onewire_bus_rmt_config_t rmt = { .max_rx_bytes = 10 };
    if (onewire_new_bus_rmt(&bcfg, &rmt, &bus) != ESP_OK) return OW_BUS;
    ow_reason_t r = OW_OK;
    if (onewire_bus_reset(bus) != ESP_OK) r = OW_RESET;
    uint8_t scr[9] = {0};
    if (r == OW_OK) {
        uint8_t cmd[2] = { CMD_SKIP_ROM, CMD_READ_SCR };
        if (onewire_bus_write_bytes(bus, cmd, 2) != ESP_OK) r = OW_WRITE;
    }
    if (r == OW_OK && onewire_bus_read_bytes(bus, scr, 9) != ESP_OK) r = OW_READ;
    onewire_bus_del(bus);
    if (r != OW_OK) return r;
    if (onewire_crc8(0, scr, 8) != scr[8]) return OW_CRC;
    uint16_t raw = (uint16_t)((scr[1] << 8) | scr[0]);
    /* Folded in from the sweep loop so a never-converted part is attributed like
     * any other failure instead of disappearing into the retry condition. */
    if (sensor_fault_is_por_raw(raw)) return OW_POR;
    *raw_out = raw;
    *out = (int16_t)raw / 16.0f;                            /* 12-bit */
    return OW_OK;
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
        for (int i = 0; i < SENS_COUNT; ++i)
            if (onewire_convert(PIN[i]) != OW_OK) s_stats.convert_fail[i]++;
        vTaskDelay(pdMS_TO_TICKS(CONVERT_MS));             /* single shared wait */
        /* Phase 2: read each scratchpad, with retries. The POR (85.0 C) check now
         * lives inside onewire_read_temp() so it is attributed like every other
         * failure rather than folded silently into the retry condition. */
        char why[160]; size_t used = 0; why[0] = '\0';
        for (int i = 0; i < SENS_COUNT; ++i) {
            float v = 0; uint16_t raw = 0;
            ow_reason_t r = OW_OK;
            for (int t = 0; t < MAX_RETRY; ++t) {
                r = onewire_read_temp(PIN[i], &v, &raw);
                if (r == OW_OK) break;
            }
            if (r == OW_OK) {
                s_stats.ok[i]++;
                apply_ema((sensor_id_t)i, v);
            } else {
                s_stats.fail[i][r]++;
                mark_fail((sensor_id_t)i);
                /* One line per sweep rather than per failure: at the ~70 % failure
                 * rate a 10-minute latch implies, five lines every 10 s would bury
                 * everything else in the log. */
                if (used < sizeof why - 1) {
                    int k = snprintf(why + used, sizeof why - used, "%s%s=%s",
                                     used ? " " : "", SENS_NAME[i], OW_REASON[r]);
                    if (k > 0) {
                        used += (size_t)k;
                        if (used >= sizeof why) used = sizeof why - 1;
                    }
                }
            }
        }
        s_stats.sweeps++;
        /* Liveness is recorded per ITERATION, not per successful read: a sweep that runs
         * and fails everything is a plant problem, one that stops running is a board
         * problem, and only this counter can tell them apart downstream. */
        xSemaphoreTake(g_lock, portMAX_DELAY);
        sensor_sweep_note(&g_sweep, (uint32_t)(esp_timer_get_time() / 1000));
        xSemaphoreGive(g_lock);
        if (why[0])
            ESP_LOGW(TAG, "sweep %lu failed: %s", (unsigned long)s_stats.sweeps, why);
        vTaskDelay(pdMS_TO_TICKS(SWEEP_PERIOD_MS - CONVERT_MS));
    }
}

void sensors_start(void)
{
    /* Report the previous run before zeroing. Attaching a console resets this
     * board, so for an operator chasing an intermittent fault this is the only
     * view of the run they were actually trying to observe. */
    if (s_stats.magic == SENSOR_STATS_MAGIC) {
        s_prev = s_stats;
        s_prev_valid = true;
        ESP_LOGW(TAG, "previous run: %lu sweeps", (unsigned long)s_prev.sweeps);
        for (int i = 0; i < SENS_COUNT; ++i) {
            uint32_t bad = 0;
            for (int r = OW_OK + 1; r < OW_REASONS; ++r) bad += s_prev.fail[i][r];
            if (!bad && !s_prev.convert_fail[i]) continue;
            ESP_LOGW(TAG, "  %s ok=%lu fail=%lu (rmt=%lu reset=%lu write=%lu read=%lu "
                          "crc=%lu por85=%lu) convert_fail=%lu",
                     SENS_NAME[i], (unsigned long)s_prev.ok[i], (unsigned long)bad,
                     (unsigned long)s_prev.fail[i][OW_BUS],
                     (unsigned long)s_prev.fail[i][OW_RESET],
                     (unsigned long)s_prev.fail[i][OW_WRITE],
                     (unsigned long)s_prev.fail[i][OW_READ],
                     (unsigned long)s_prev.fail[i][OW_CRC],
                     (unsigned long)s_prev.fail[i][OW_POR],
                     (unsigned long)s_prev.convert_fail[i]);
        }
    } else {
        ESP_LOGI(TAG, "no previous-run sensor stats (cold power-on)");
    }
    memset(&s_stats, 0, sizeof s_stats);
    s_stats.magic = SENSOR_STATS_MAGIC;

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
    /* Unchecked, this returns pdFAIL and app_main sails on into a controller whose probes
     * are all latched-faulted forever -- and because the task never ran, it never
     * subscribed to the task WDT, so nothing resets either. Abort instead: the panic
     * handler resets, and on the first boot after an OTA the image never validates, so
     * the bootloader rolls back to the last good one. Silence is the one option that
     * cannot be allowed here. */
    if (xTaskCreate(sweep_task, "sensors", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate(sensors) failed -- no sweep task, aborting for reset+rollback");
        abort();
    }
}

uint32_t sensors_sweep_count(void){ return s_stats.sweeps; }

bool sensors_sweep_dead(void)
{
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    xSemaphoreTake(g_lock, portMAX_DELAY);
    bool dead = sensor_sweep_is_dead(&g_sweep, now);
    xSemaphoreGive(g_lock);
    return dead;
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

static size_t fmt_run(char *o, size_t n, size_t u, const char *label, const sensor_stats_t *s)
{
    int k = snprintf(o + u, n - u, "%s: %lu sweeps\n", label, (unsigned long)s->sweeps);
    if (k > 0) u += (size_t)k;
    if (u >= n) return n - 1;
    for (int i = 0; i < SENS_COUNT; ++i) {
        k = snprintf(o + u, n - u,
                     "  %-6s ok=%lu rmt=%lu rst=%lu wr=%lu rd=%lu crc=%lu por85=%lu cnv=%lu\n",
                     SENS_NAME[i], (unsigned long)s->ok[i],
                     (unsigned long)s->fail[i][OW_BUS],
                     (unsigned long)s->fail[i][OW_RESET],
                     (unsigned long)s->fail[i][OW_WRITE],
                     (unsigned long)s->fail[i][OW_READ],
                     (unsigned long)s->fail[i][OW_CRC],
                     (unsigned long)s->fail[i][OW_POR],
                     (unsigned long)s->convert_fail[i]);
        if (k > 0) u += (size_t)k;
        if (u >= n) return n - 1;
    }
    return u;
}

/* Console `stats`: this run, plus the previous one when RTC memory carried it
 * across a reset -- including the reset that opening the console just caused. */
void sensors_format_stats(char *o, size_t n)
{
    if (!o || n == 0) return;
    size_t u = fmt_run(o, n, 0, "run", &s_stats);
    if (s_prev_valid) fmt_run(o, n, u, "prev", &s_prev);
    else if (u < n - 1) snprintf(o + u, n - u, "prev: none (cold power-on)\n");
}
