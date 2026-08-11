#include "taskhb.h"
#include <stdio.h>
#include <string.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

static const char *TAG = "hb";

static const char *const HB_NAME[HB_COUNT] = { "sensors", "control", "valve", "telem" };

#define HB_MAGIC 0x48425401u

typedef struct {
    uint32_t magic;
    uint32_t last_ms[HB_COUNT];      /* uptime at each task's most recent iteration */
    uint32_t prev_last_ms[HB_COUNT]; /* the same, for the run that just ended */
    uint32_t prev_valid;
} hb_state_t;

static RTC_NOINIT_ATTR hb_state_t s_hb;

void hb_note(hb_id_t id)
{
    if (id < HB_COUNT) s_hb.last_ms[id] = (uint32_t)(esp_timer_get_time() / 1000);
}

void hb_boot_report(void)
{
    esp_reset_reason_t rr = esp_reset_reason();

    if (s_hb.magic == HB_MAGIC) {
        memcpy(s_hb.prev_last_ms, s_hb.last_ms, sizeof s_hb.prev_last_ms);
        s_hb.prev_valid = 1;

        /* The newest heartbeat approximates when the run died; ages are measured back
         * from it, so the numbers stay meaningful without knowing the reset instant. */
        uint32_t newest = 0;
        for (int i = 0; i < HB_COUNT; ++i)
            if (s_hb.prev_last_ms[i] > newest) newest = s_hb.prev_last_ms[i];

        ESP_LOGW(TAG, "previous run ended by %s; heartbeat ages at death (0 = last to run):",
                 rr == ESP_RST_TASK_WDT ? "TASK_WDT" :
                 rr == ESP_RST_INT_WDT  ? "INT_WDT"  :
                 rr == ESP_RST_PANIC    ? "PANIC"    :
                 rr == ESP_RST_SW       ? "sw"       :
                 rr == ESP_RST_BROWNOUT ? "BROWNOUT" : "other");
        for (int i = 0; i < HB_COUNT; ++i)
            ESP_LOGW(TAG, "  %-7s last=%lu ms  age=%lu ms",
                     HB_NAME[i], (unsigned long)s_hb.prev_last_ms[i],
                     (unsigned long)(newest - s_hb.prev_last_ms[i]));
    } else {
        memset(&s_hb, 0, sizeof s_hb);
        s_hb.magic = HB_MAGIC;
        ESP_LOGI(TAG, "no previous heartbeats (cold power-on)");
    }
    memset(s_hb.last_ms, 0, sizeof s_hb.last_ms);
}

void hb_format(char *o, size_t n)
{
    if (!o || n == 0) return;
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    size_t u = 0;
    int k = snprintf(o, n, "now=%lu ms\nrun:\n", (unsigned long)now);
    if (k > 0) u += (size_t)k;
    for (int i = 0; i < HB_COUNT && u < n - 1; ++i) {
        k = snprintf(o + u, n - u, "  %-7s last=%lu age=%lu\n", HB_NAME[i],
                     (unsigned long)s_hb.last_ms[i],
                     (unsigned long)(now - s_hb.last_ms[i]));
        if (k > 0) u += (size_t)k;
    }
    if (!s_hb.prev_valid) {
        if (u < n - 1) snprintf(o + u, n - u, "prev: none (cold power-on)\n");
        return;
    }
    uint32_t newest = 0;
    for (int i = 0; i < HB_COUNT; ++i)
        if (s_hb.prev_last_ms[i] > newest) newest = s_hb.prev_last_ms[i];
    k = snprintf(o + u, n - u, "prev (age back from last task to run):\n");
    if (k > 0) u += (size_t)k;
    for (int i = 0; i < HB_COUNT && u < n - 1; ++i) {
        k = snprintf(o + u, n - u, "  %-7s last=%lu age=%lu\n", HB_NAME[i],
                     (unsigned long)s_hb.prev_last_ms[i],
                     (unsigned long)(newest - s_hb.prev_last_ms[i]));
        if (k > 0) u += (size_t)k;
    }
}
