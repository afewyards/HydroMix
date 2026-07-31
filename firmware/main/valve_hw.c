#include "valve_hw.h"
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "ctrl_core/interlock.h"
#include "ctrl_core/pos_estimator.h"

#define PIN_OPEN  GPIO_NUM_2
#define PIN_CLOSE GPIO_NUM_3
#define TICK_MS   100
#define DEADBAND_PCT 2.0f
#define RESYNC_STALL_MULT 1.15f

typedef enum { RS_IDLE, RS_DRIVING } resync_state_t;

static interlock_state_t s_ilk;
static pos_est_state_t   s_pos;
static SemaphoreHandle_t s_lock;
static float             s_target = 50.0f;
static resync_state_t    s_rs = RS_IDLE;
static uint32_t          s_rs_start_ms = 0;
static bool              s_resync_req = false;
/* Latched copies of the motion-affecting config, refreshed only while idle (see
 * valve_task), so a runtime config change never flips the open/close mapping nor
 * rescales the stall/position math mid-stroke or mid-resync. */
static bool              s_swap_latched = false;
static uint32_t          s_travel_latched_s = 0;

static uint32_t now_ms(void){ return (uint32_t)(esp_timer_get_time() / 1000); }

/* Resolve which physical direction moves toward source(100)/recirc(0) via the latched swap. */
static valve_dir_t dir_toward_source(void){ return s_swap_latched ? VALVE_CLOSE : VALVE_OPEN; }
static valve_dir_t dir_toward_recirc(void){ return s_swap_latched ? VALVE_OPEN : VALVE_CLOSE; }

static int8_t travel_sign_of(valve_dir_t applied){
    if (applied == dir_toward_source()) return +1;
    if (applied == dir_toward_recirc()) return -1;
    return 0;
}

static void apply(triac_cmd_t c){
    /* Interlock guarantees never-both; this is the only writer of GPIO2/3. */
    gpio_set_level(PIN_OPEN,  c.open_on ? 1 : 0);
    gpio_set_level(PIN_CLOSE, c.close_on ? 1 : 0);
}

static valve_dir_t desired_dir(float target){
    float pos = s_pos.position_pct;
    if (target > pos + DEADBAND_PCT) return dir_toward_source();
    if (target < pos - DEADBAND_PCT) return dir_toward_recirc();
    return VALVE_STOP;
}

static void valve_task(void *arg){
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    for (;;) {
        esp_task_wdt_reset();
        uint32_t t = now_ms();
        xSemaphoreTake(s_lock, portMAX_DELAY);

        if (s_resync_req && s_rs == RS_IDLE){ s_rs = RS_DRIVING; s_rs_start_ms = t; s_resync_req = false; }

        valve_dir_t want;
        if (s_rs == RS_DRIVING){
            uint32_t stall_ms = (uint32_t)(s_travel_latched_s * 1000.0f * RESYNC_STALL_MULT);
            if (t - s_rs_start_ms >= stall_ms){
                pos_est_resync_done(&s_pos);              /* position := 0 % */
                s_rs = RS_IDLE;
                want = VALVE_STOP;
            } else {
                want = dir_toward_recirc();               /* NEVER toward source */
            }
        } else {
            want = desired_dir(s_target);
            if (pos_est_needs_resync(&s_pos)) s_resync_req = true;
        }

        triac_cmd_t c = interlock_step(&s_ilk,
                                       want == VALVE_OPEN,  want == VALVE_CLOSE, t);
        apply(c);

        valve_dir_t applied = c.open_on ? VALVE_OPEN : (c.close_on ? VALVE_CLOSE : VALVE_STOP);

        /* Idle (not driving, not mid-resync): the only safe point to pick up config
         * changes for the *next* motion. While actually moving, both latches stay
         * frozen -- a travel_time_s write mid-stroke would otherwise rescale the
         * position estimator's %-per-tick under it. */
        if (applied == VALVE_STOP && s_rs == RS_IDLE) {
            s_swap_latched     = g_config.direction_swap;
            s_travel_latched_s = g_config.travel_time_s;
        }

        pos_est_update(&s_pos, travel_sign_of(applied), TICK_MS, (float)s_travel_latched_s);

        xSemaphoreGive(s_lock);
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
    }
}

void valve_start(void){
    s_lock = xSemaphoreCreateMutex();
    interlock_init(&s_ilk);
    pos_est_init(&s_pos);
    s_swap_latched     = g_config.direction_swap;          /* initial latch at boot */
    s_travel_latched_s = g_config.travel_time_s;
    s_resync_req = true;                                   /* boot resync */
    xTaskCreate(valve_task, "valve", 4096, NULL, 6, NULL);
}

void valve_set_target(float pct){
    if (isnan(pct)) return;   /* defense in depth: also reachable from console, not just Zigbee */
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_target = ctrl_clampf(pct, 0.0f, 100.0f);
    xSemaphoreGive(s_lock);
}
float valve_get_position(void){
    xSemaphoreTake(s_lock, portMAX_DELAY);
    float p = s_pos.position_pct;
    xSemaphoreGive(s_lock);
    return p;
}
void valve_resync(void){
    xSemaphoreTake(s_lock, portMAX_DELAY); s_resync_req = true; xSemaphoreGive(s_lock);
}
void valve_stop(void){ valve_set_target(valve_get_position()); }
bool valve_resync_active(void){
    xSemaphoreTake(s_lock, portMAX_DELAY); bool a = (s_rs == RS_DRIVING); xSemaphoreGive(s_lock);
    return a;
}
float valve_travel_since_resync(void){
    xSemaphoreTake(s_lock, portMAX_DELAY); float t = s_pos.accum_travel_pct; xSemaphoreGive(s_lock);
    return t;
}
