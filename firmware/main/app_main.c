#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "console.h"
#include "config.h"
#include "sensors_hw.h"
#include "valve_hw.h"
#include "control_task.h"
#include "zigbee.h"
#include "ui.h"
#include "ota.h"
#include "taskhb.h"
#include <stdio.h>

#define PIN_SSR_OPEN   GPIO_NUM_2
#define PIN_SSR_CLOSE  GPIO_NUM_3
#define PIN_STAT_LED   GPIO_NUM_15   /* active-low: 0 = lit */

static const char *TAG = "app";

/* FIRST action in app_main: force both triacs off before anything else can run. */
static void triacs_safe_low(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_SSR_OPEN) | (1ULL << PIN_SSR_CLOSE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(PIN_SSR_OPEN, 0);
    gpio_set_level(PIN_SSR_CLOSE, 0);
}

void app_main(void)
{
    triacs_safe_low();                 /* MUST be first */
    hb_boot_report();                  /* before anything can overwrite the heartbeats */
    ESP_LOGI(TAG, "ValveController boot: triacs forced low");
    config_load();
    ota_init();
    sensors_start();
    valve_start();
    control_task_start();
    /* Restore the last state HA commanded. Forcing true here (the old "bench default")
     * meant that after any power blip the device regulated a live loop while the OnOff
     * attribute -- and therefore HA -- still read OFF. Must run BEFORE zigbee_start():
     * build_endpoints() seeds the OnOff attribute from control_task_water_running(). */
    control_task_set_water_running(config_water_running_load());
    zigbee_start();
    ui_start();
    console_start();
}

void console_hook_status(char *o, size_t n){
    sensor_reading_t s = sensors_get(SENS_SUPPLY), r = sensors_get(SENS_RETURN),
                     so = sensors_get(SENS_SOURCE), a = sensors_get(SENS_HX_A), b = sensors_get(SENS_HX_B);
    snprintf(o, n, "supply=%.2f ret=%.2f src=%.2f hxa=%.2f hxb=%.2f faults=%d%d%d%d%d\n",
             s.value_c, r.value_c, so.value_c, a.value_c, b.value_c,
             s.fault, r.fault, so.fault, a.fault, b.fault);
}

void console_hook_valve(int pct){ valve_set_target((float)pct); }
void console_hook_resync(void){ valve_resync(); }
void console_hook_factory_reset(void){ zigbee_leave(); config_factory_reset(); }

void zigbee_on_join(void){ ota_note_joined(); }

void console_hook_stats(char *o, size_t n){ sensors_format_stats(o, n); }

void console_hook_zbtemp(char *o, size_t n){ zigbee_format_temp_stats(o, n); }

void console_hook_hb(char *o, size_t n){ hb_format(o, n); }

void console_hook_mode(char *o, size_t n){
    const char *m = control_task_mode()==MODE_HEATING?"HEATING":
                    control_task_mode()==MODE_COOLING?"COOLING":"IDLE";
    snprintf(o, n, "mode=%s alarm=%d faults=0x%02x pos=%.1f\n",
             m, control_task_alarm(), control_task_faults(), valve_get_position());
}
