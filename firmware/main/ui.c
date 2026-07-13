#include "ui.h"
#include "zigbee.h"
#include "control_task.h"
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define PIN_BTN GPIO_NUM_9
#define PIN_LED GPIO_NUM_15   /* active-low */
#define HOLD_RESET_MS 5000

static volatile bool s_identify = false;
void ui_set_identify(bool on){ s_identify = on; }

static void led(int lit){ gpio_set_level(PIN_LED, lit ? 0 : 1); }   /* active-low */

static void blink(int count, int on_ms, int gap_ms){
    for (int i = 0; i < count; ++i){ led(1); vTaskDelay(pdMS_TO_TICKS(on_ms)); led(0); vTaskDelay(pdMS_TO_TICKS(gap_ms)); }
}

static void led_task(void *arg){
    gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT); led(0);
    for (;;) {
        if (s_identify)               { blink(1, 60, 60); continue; }
        if (control_task_alarm() || control_task_faults()) { blink(3, 80, 120); vTaskDelay(pdMS_TO_TICKS(600)); continue; }
        if (!zigbee_joined())         { blink(1, 100, 100); continue; }          /* steering / not joined */
        switch (control_task_mode()){
            case MODE_HEATING: blink(1, 150, 0); vTaskDelay(pdMS_TO_TICKS(2500)); break; /* slow single */
            case MODE_COOLING: blink(2, 150, 200); vTaskDelay(pdMS_TO_TICKS(2500)); break; /* slow double */
            default:           blink(1, 50, 0); vTaskDelay(pdMS_TO_TICKS(4950)); break;    /* idle 1/5s */
        }
    }
}

static void btn_task(void *arg){
    gpio_config_t b = { .pin_bit_mask = 1ULL<<PIN_BTN, .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE };
    gpio_config(&b);
    for (;;) {
        if (gpio_get_level(PIN_BTN) == 0) {                 /* pressed (active-low) */
            int held = 0;
            while (gpio_get_level(PIN_BTN) == 0 && held < HOLD_RESET_MS + 100) { vTaskDelay(pdMS_TO_TICKS(50)); held += 50; }
            if (held >= HOLD_RESET_MS) { zigbee_leave(); config_factory_reset(); }
            else                       { zigbee_steer(); }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void ui_start(void){
    xTaskCreate(led_task, "led", 2048, NULL, 3, NULL);
    xTaskCreate(btn_task, "btn", 2048, NULL, 3, NULL);
}
