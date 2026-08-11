#include "taskhb.h"
#include <stddef.h>
#include <string.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "ctrl_core/diag_ring.h"

static const char *TAG = "hb";

static const char *const HB_NAME[HB_COUNT] = { "sensors", "control", "valve", "telem" };

#define HB_MAGIC 0x48425403u   /* bumped: the store layout changed with the hdr */

/* diag_reset_reason_name() is numeric because ctrl_core cannot see esp_system.h.
 * Pin the mapping here, where the real enum IS visible, so an IDF renumbering breaks
 * the build instead of quietly mislabelling every recorded run. */
_Static_assert(ESP_RST_POWERON     == 1,  "diag_reset_reason_name table is stale");
_Static_assert(ESP_RST_PANIC       == 4,  "diag_reset_reason_name table is stale");
_Static_assert(ESP_RST_TASK_WDT    == 6,  "diag_reset_reason_name table is stale");
_Static_assert(ESP_RST_BROWNOUT    == 9,  "diag_reset_reason_name table is stale");
_Static_assert(ESP_RST_EFUSE       == 13, "diag_reset_reason_name table is stale");
_Static_assert(ESP_RST_PWR_GLITCH  == 14, "diag_reset_reason_name table is stale");
_Static_assert(ESP_RST_CPU_LOCKUP  == 15, "diag_reset_reason_name table is stale");

typedef struct {
    uint32_t last_ms[HB_COUNT];
    uint8_t  end_reason;         /* esp_reset_reason_t observed at the NEXT boot */
    uint8_t  valid;
} hb_run_t;

typedef struct {
    diag_hdr_t hdr;                    /* MUST be first: diag_ring_warm() casts to it */
    uint32_t   last_ms[HB_COUNT];      /* live run */
    hb_run_t   hist[DIAG_RING_DEPTH];  /* hist[0] = most recently ended run */
} hb_store_t;

_Static_assert(offsetof(hb_store_t, hdr) == 0, "diag_hdr_t must be the first member");

static RTC_NOINIT_ATTR hb_store_t s_hb;

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

    if (diag_ring_warm(&s_hb, sizeof s_hb, HB_MAGIC)) {
        diag_ring_shift(s_hb.hist, sizeof s_hb.hist[0], DIAG_RING_DEPTH);
        memcpy(s_hb.hist[0].last_ms, s_hb.last_ms, sizeof s_hb.last_ms);
        s_hb.hist[0].end_reason = rr;      /* why the run that just ended, ended */
        s_hb.hist[0].valid = 1;

        const hb_run_t *p = &s_hb.hist[0];
        uint32_t newest = newest_of(p->last_ms);
        ESP_LOGW(TAG, "run#%lu ended by %s; heartbeat ages at death (0 = last to run):",
                 (unsigned long)(s_hb.hdr.seq - 1), diag_reset_reason_name(rr));
        for (int i = 0; i < HB_COUNT; ++i)
            ESP_LOGW(TAG, "  %-7s last=%lu ms  age=%lu ms", HB_NAME[i],
                     (unsigned long)p->last_ms[i], (unsigned long)(newest - p->last_ms[i]));
    } else {
        ESP_LOGI(TAG, "no previous heartbeats (cold power-on)");
    }
    memset(s_hb.last_ms, 0, sizeof s_hb.last_ms);
}

void hb_format(char *o, size_t n)
{
    if (!o || n == 0) return;
    o[0] = '\0';
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    size_t u = diag_appendf(o, n, 0, "run#%lu now=%lu ms\n",
                            (unsigned long)s_hb.hdr.seq, (unsigned long)now);
    for (int i = 0; i < HB_COUNT; ++i)
        u = diag_appendf(o, n, u, "  %-7s age=%lu\n", HB_NAME[i],
                         (unsigned long)(now - s_hb.last_ms[i]));
    for (int h = 0; h < DIAG_RING_DEPTH; ++h) {
        if (!s_hb.hist[h].valid) continue;
        const hb_run_t *p = &s_hb.hist[h];
        uint32_t newest = newest_of(p->last_ms);
        u = diag_appendf(o, n, u, "prev-%d (run#%lu) ended by %s:\n", h + 1,
                         (unsigned long)(s_hb.hdr.seq - (uint32_t)(h + 1)),
                         diag_reset_reason_name(p->end_reason));
        for (int i = 0; i < HB_COUNT; ++i)
            u = diag_appendf(o, n, u, "  %-7s last=%lu age=%lu\n", HB_NAME[i],
                             (unsigned long)p->last_ms[i],
                             (unsigned long)(newest - p->last_ms[i]));
    }
}
