#include "taskhb.h"
#include <stdio.h>
#include <string.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

static const char *TAG = "hb";

static const char *const HB_NAME[HB_COUNT] = { "sensors", "control", "valve", "telem" };

#define HB_MAGIC 0x48425402u
/* Depth, for the third time in this investigation. A one-deep record cannot answer the
 * question it exists for: reaching the console costs a reset, so the run you can read is
 * always the one YOUR reset ended, never the one the fault ended. The watchdog run is
 * already two boots back by the time anyone types a command. */
#define HB_HISTORY 4

typedef struct {
    uint32_t last_ms[HB_COUNT];
    uint8_t  end_reason;         /* esp_reset_reason_t observed at the NEXT boot */
    uint8_t  valid;
} hb_run_t;

typedef struct {
    uint32_t magic;
    uint32_t seq;
    uint32_t last_ms[HB_COUNT];  /* live run */
    hb_run_t hist[HB_HISTORY];   /* hist[0] = most recently ended run */
} hb_state_t;

static RTC_NOINIT_ATTR hb_state_t s_hb;

static const char *rr_name(uint8_t r)
{
    switch ((esp_reset_reason_t)r) {
        case ESP_RST_POWERON:   return "poweron";
        case ESP_RST_EXT:       return "ext";
        case ESP_RST_SW:        return "sw";
        case ESP_RST_PANIC:     return "PANIC";
        case ESP_RST_INT_WDT:   return "INT_WDT";
        case ESP_RST_TASK_WDT:  return "TASK_WDT";
        case ESP_RST_WDT:       return "WDT";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";
        case ESP_RST_USB:       return "usb";
        default:                return "other";
    }
}

void hb_note(hb_id_t id)
{
    if (id < HB_COUNT) s_hb.last_ms[id] = (uint32_t)(esp_timer_get_time() / 1000);
}

/* Age measured back from the newest heartbeat, which approximates the instant the run
 * died -- the reset time itself is not recoverable after the fact. */
static uint32_t newest_of(const uint32_t *v)
{
    uint32_t n = 0;
    for (int i = 0; i < HB_COUNT; ++i) if (v[i] > n) n = v[i];
    return n;
}

void hb_boot_report(void)
{
    uint8_t rr = (uint8_t)esp_reset_reason();

    if (s_hb.magic == HB_MAGIC) {
        for (int h = HB_HISTORY - 1; h > 0; --h) s_hb.hist[h] = s_hb.hist[h - 1];
        memcpy(s_hb.hist[0].last_ms, s_hb.last_ms, sizeof s_hb.last_ms);
        s_hb.hist[0].end_reason = rr;      /* why the run that just ended, ended */
        s_hb.hist[0].valid = 1;
        s_hb.seq++;

        const hb_run_t *p = &s_hb.hist[0];
        uint32_t newest = newest_of(p->last_ms);
        ESP_LOGW(TAG, "run#%lu ended by %s; heartbeat ages at death (0 = last to run):",
                 (unsigned long)(s_hb.seq - 1), rr_name(rr));
        for (int i = 0; i < HB_COUNT; ++i)
            ESP_LOGW(TAG, "  %-7s last=%lu ms  age=%lu ms", HB_NAME[i],
                     (unsigned long)p->last_ms[i], (unsigned long)(newest - p->last_ms[i]));
    } else {
        memset(&s_hb, 0, sizeof s_hb);
        s_hb.magic = HB_MAGIC;
        s_hb.seq   = 1;
        ESP_LOGI(TAG, "no previous heartbeats (cold power-on)");
    }
    memset(s_hb.last_ms, 0, sizeof s_hb.last_ms);
}

void hb_format(char *o, size_t n)
{
    if (!o || n == 0) return;
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    size_t u = 0;
    int k = snprintf(o, n, "run#%lu now=%lu ms\n", (unsigned long)s_hb.seq, (unsigned long)now);
    if (k > 0) u += (size_t)k;
    for (int i = 0; i < HB_COUNT && u < n - 1; ++i) {
        k = snprintf(o + u, n - u, "  %-7s age=%lu\n", HB_NAME[i],
                     (unsigned long)(now - s_hb.last_ms[i]));
        if (k > 0) u += (size_t)k;
    }
    for (int h = 0; h < HB_HISTORY && u < n - 1; ++h) {
        if (!s_hb.hist[h].valid) continue;
        const hb_run_t *p = &s_hb.hist[h];
        uint32_t newest = newest_of(p->last_ms);
        k = snprintf(o + u, n - u, "prev-%d (run#%lu) ended by %s:\n", h + 1,
                     (unsigned long)(s_hb.seq - (uint32_t)(h + 1)), rr_name(p->end_reason));
        if (k > 0) u += (size_t)k;
        for (int i = 0; i < HB_COUNT && u < n - 1; ++i) {
            k = snprintf(o + u, n - u, "  %-7s last=%lu age=%lu\n", HB_NAME[i],
                         (unsigned long)p->last_ms[i],
                         (unsigned long)(newest - p->last_ms[i]));
            if (k > 0) u += (size_t)k;
        }
    }
}
