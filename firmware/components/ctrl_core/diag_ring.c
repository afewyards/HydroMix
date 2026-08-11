#include "ctrl_core/diag_ring.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

bool diag_ring_warm(void *store, size_t store_size, uint32_t magic)
{
    if (!store || store_size < sizeof(diag_hdr_t)) return false;
    diag_hdr_t *h = (diag_hdr_t *)store;
    if (h->magic == magic) {
        h->seq++;
        return true;
    }
    memset(store, 0, store_size);
    h->magic = magic;
    h->seq   = 1;
    return false;
}

void diag_ring_shift(void *hist, size_t elem, uint8_t depth)
{
    if (!hist || elem == 0 || depth < 2) return;
    char *b = (char *)hist;
    for (int h = (int)depth - 1; h > 0; --h)
        memcpy(b + (size_t)h * elem, b + (size_t)(h - 1) * elem, elem);
}

const char *diag_reset_reason_name(int reason)
{
    /* Numeric because ctrl_core cannot see esp_system.h. Values confirmed against the
     * installed ESP-IDF; main/taskhb.c static-asserts that they still match. */
    switch (reason) {
        case 1:  return "poweron";
        case 2:  return "ext";
        case 3:  return "sw";
        case 4:  return "PANIC";
        case 5:  return "INT_WDT";
        case 6:  return "TASK_WDT";
        case 7:  return "WDT";
        case 8:  return "deepsleep";
        case 9:  return "BROWNOUT";
        case 10: return "sdio";
        case 11: return "usb";
        case 12: return "jtag";
        case 13: return "efuse";
        case 14: return "PWR_GLITCH";
        case 15: return "CPU_LOCKUP";
        default: return "other";
    }
}

size_t diag_appendf(char *o, size_t n, size_t used, const char *fmt, ...)
{
    if (!o || n == 0) return 0;
    if (used >= n - 1) { o[n - 1] = '\0'; return n - 1; }
    va_list ap;
    va_start(ap, fmt);
    int k = vsnprintf(o + used, n - used, fmt, ap);
    va_end(ap);
    if (k < 0) return used;                     /* encoding error: keep what we had */
    used += (size_t)k;                          /* vsnprintf returns the WANTED length */
    return used > n - 1 ? n - 1 : used;
}
