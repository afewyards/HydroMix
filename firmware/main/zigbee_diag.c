#include "zigbee_diag.h"
#include "zigbee.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "ctrl_core/diag_ring.h"

static const char *TAG = "zigbee";

/* 0x2B7E150B, bumped from 0x2B7E150A: the store gained a diag_hdr_t at the front. */
#define ZB_TEMP_STATS_MAGIC 0x2B7E150Bu
#define ZB_EP_COUNT 5

static const char *const TEMP_EP_NAME[ZB_EP_COUNT] = {
    "supply", "return", "source", "hx_a", "hx_b"
};

typedef struct {
    uint32_t ok[ZB_EP_COUNT];
    uint32_t fail[ZB_EP_COUNT];
    uint32_t mismatch[ZB_EP_COUNT];   /* write returned SUCCESS but the table held other */
    int32_t  last_err[ZB_EP_COUNT];   /* esp_zb_zcl_status_t of the most recent failure */
    int16_t  last_val[ZB_EP_COUNT];   /* value the write attempted */
    int16_t  last_read[ZB_EP_COUNT];  /* what the table held immediately after */
    uint32_t ticks;                   /* telemetry iterations completed this run */
    uint32_t sweeps;                  /* sensor sweeps completed this run */
    uint16_t faults;                  /* control_task_faults() at the last tick */
    /* Why THIS run started -- which is to say why the previous one ended. The console
     * cannot answer it: attaching USB is exactly the intervention that stops the resets. */
    uint8_t  reset_reason;
} zb_run_t;

typedef struct {
    diag_hdr_t hdr;                       /* MUST be first */
    zb_run_t   cur;
    zb_run_t   hist[DIAG_RING_DEPTH];     /* hist[0] = most recently ended run */
} zb_store_t;

_Static_assert(offsetof(zb_store_t, hdr) == 0, "diag_hdr_t must be the first member");

static RTC_NOINIT_ATTR zb_store_t s_zb;

void zbdiag_note_ok(int i, uint8_t ep, int16_t wrote, int16_t read_back)
{
    if (i < 0 || i >= ZB_EP_COUNT) return;
    s_zb.cur.ok[i]++;
    s_zb.cur.last_read[i] = read_back;
    if (read_back != wrote) {
        s_zb.cur.mismatch[i]++;
        if (s_zb.cur.mismatch[i] == 1)
            ESP_LOGE(TAG, "ep%u %s attr MISMATCH: wrote %d, table holds %d",
                     (unsigned)ep, TEMP_EP_NAME[i], (int)wrote, (int)read_back);
    }
}

void zbdiag_note_fail(int i, uint8_t ep, int32_t status, int16_t wrote)
{
    if (i < 0 || i >= ZB_EP_COUNT) return;
    s_zb.cur.fail[i]++;
    s_zb.cur.last_err[i] = status;
    s_zb.cur.last_val[i] = wrote;
    /* First failure per endpoint is loud; after that the tally carries it, so a
     * persistent fault cannot bury the rest of the log at 30 lines a minute. */
    if (s_zb.cur.fail[i] == 1)
        ESP_LOGE(TAG, "set_attribute_val(ep%u %s) failed: status=0x%02x value=%d",
                 (unsigned)ep, TEMP_EP_NAME[i], (unsigned)status, (int)wrote);
}

void zbdiag_note_tick(uint32_t sweeps, uint16_t faults)
{
    s_zb.cur.ticks++;
    s_zb.cur.sweeps = sweeps;
    s_zb.cur.faults = faults;
}

void zbdiag_boot_init(void)
{
    uint8_t rr = (uint8_t)esp_reset_reason();

    if (diag_ring_warm(&s_zb, sizeof s_zb, ZB_TEMP_STATS_MAGIC)) {
        diag_ring_shift(s_zb.hist, sizeof s_zb.hist[0], DIAG_RING_DEPTH);
        s_zb.hist[0] = s_zb.cur;
        const zb_run_t *p = &s_zb.hist[0];
        ESP_LOGW(TAG, "previous run#%lu: ticks=%lu sweeps=%lu faults=0x%02x boot=%s (ended -> %s)",
                 (unsigned long)(s_zb.hdr.seq - 1), (unsigned long)p->ticks,
                 (unsigned long)p->sweeps, (unsigned)p->faults,
                 diag_reset_reason_name(p->reset_reason), diag_reset_reason_name(rr));
        for (int i = 0; i < ZB_EP_COUNT; ++i) {
            if (!p->fail[i] && !p->mismatch[i]) continue;
            ESP_LOGW(TAG, "  ep%d %s ok=%lu fail=%lu mism=%lu err=0x%02lx val=%d read=%d",
                     i + 2, TEMP_EP_NAME[i], (unsigned long)p->ok[i], (unsigned long)p->fail[i],
                     (unsigned long)p->mismatch[i], (unsigned long)(uint32_t)p->last_err[i],
                     (int)p->last_val[i], (int)p->last_read[i]);
        }
    } else {
        ESP_LOGI(TAG, "no previous attr-write history (cold power-on)");
    }
    memset(&s_zb.cur, 0, sizeof s_zb.cur);
    s_zb.cur.reset_reason = rr;
    ESP_LOGW(TAG, "run#%lu started, reset reason: %s (%u)",
             (unsigned long)s_zb.hdr.seq, diag_reset_reason_name(rr), (unsigned)rr);
}

static size_t fmt_run_block(char *o, size_t n, size_t u, const char *label, const zb_run_t *r)
{
    u = diag_appendf(o, n, u, "%s ticks=%lu sweeps=%lu faults=0x%02x boot=%s\n",
                     label, (unsigned long)r->ticks, (unsigned long)r->sweeps,
                     (unsigned)r->faults, diag_reset_reason_name(r->reset_reason));
    for (int i = 0; i < ZB_EP_COUNT; ++i)
        u = diag_appendf(o, n, u,
                         "  %-6s ok=%lu fail=%lu mism=%lu err=0x%02lx val=%d read=%d\n",
                         TEMP_EP_NAME[i], (unsigned long)r->ok[i], (unsigned long)r->fail[i],
                         (unsigned long)r->mismatch[i], (unsigned long)(uint32_t)r->last_err[i],
                         (int)r->last_val[i], (int)r->last_read[i]);
    return u;
}

/* Console `zbtemp`: the live run plus the last DIAG_RING_DEPTH completed ones. Read the
 * history, not `run#` -- by the time you can type this, the run that mattered is two
 * resets back. */
void zigbee_format_temp_stats(char *o, size_t n)
{
    if (!o || n == 0) return;
    o[0] = '\0';
    char lbl[28];
    snprintf(lbl, sizeof lbl, "run#%lu:", (unsigned long)s_zb.hdr.seq);
    size_t u = fmt_run_block(o, n, 0, lbl, &s_zb.cur);
    for (int h = 0; h < DIAG_RING_DEPTH; ++h) {
        if (!s_zb.hist[h].ticks && !s_zb.hist[h].sweeps) continue;   /* slot never used */
        snprintf(lbl, sizeof lbl, "prev-%d (run#%lu):", h + 1,
                 (unsigned long)(s_zb.hdr.seq - (uint32_t)(h + 1)));
        u = fmt_run_block(o, n, u, lbl, &s_zb.hist[h]);
    }
}
