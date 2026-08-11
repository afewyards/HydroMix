#include "console.h"
#include <string.h>
#include <stdlib.h>
#include "esp_console.h"
#include "driver/usb_serial_jtag.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

__attribute__((weak)) void console_hook_status(char *o, size_t n){ snprintf(o,n,"no status\n"); }
__attribute__((weak)) void console_hook_valve(int pct){ (void)pct; }
__attribute__((weak)) void console_hook_resync(void){}
__attribute__((weak)) void console_hook_mode(char *o, size_t n){ snprintf(o,n,"IDLE\n"); }
__attribute__((weak)) void console_hook_factory_reset(void){}
__attribute__((weak)) void console_hook_stats(char *o, size_t n){ snprintf(o,n,"no stats\n"); }
__attribute__((weak)) void console_hook_zbtemp(char *o, size_t n){ snprintf(o,n,"no zbtemp\n"); }
__attribute__((weak)) void console_hook_hb(char *o, size_t n){ snprintf(o,n,"no hb\n"); }

static int cmd_status(int c, char **v){ char b[256]; console_hook_status(b,sizeof b); printf("%s",b); return 0; }
static int cmd_valve(int c, char **v){ if(c<2){printf("usage: valve <0-100>\n");return 1;} console_hook_valve(atoi(v[1])); return 0; }
static int cmd_resync(int c, char **v){ console_hook_resync(); printf("resync requested\n"); return 0; }
static int cmd_mode(int c, char **v){ char b[64]; console_hook_mode(b,sizeof b); printf("%s",b); return 0; }
static int cmd_freset(int c, char **v){ console_hook_factory_reset(); printf("factory reset\n"); return 0; }
/* Up to 5 runs x (header + 5 sensors); sized so the whole history prints un-truncated. */
static int cmd_stats(int c, char **v){ char b[2816]; console_hook_stats(b,sizeof b); printf("%s",b); return 0; }
/* Up to 5 runs x (header + 5 endpoints). */
static int cmd_zbtemp(int c, char **v){ char b[2048]; console_hook_zbtemp(b,sizeof b); printf("%s",b); return 0; }
static int cmd_hb(int c, char **v){ char b[1024]; console_hook_hb(b,sizeof b); printf("%s",b); return 0; }

/* The REPL is started only once a USB host is actually attached.
 *
 * Measured 2026-08-11: on mains alone the board resets about every 40 s with
 * ESP_RST_TASK_WDT, and the per-task heartbeats show every watchdog-subscribed app task
 * fed on schedule right up to the reset (sensors 9.9 s stale on a 10 s loop, control
 * 0.1 s, valve 0 s). The only remaining subscriber is the idle task
 * (CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0=y), so nothing BLOCKED -- something spun.
 * A spinner above idle (priority 0) but below the app tasks (4-6) starves idle alone,
 * which is exactly the observed picture, and the REPL sits in that band.
 *
 * With no host attached its console reads return immediately instead of blocking, so
 * the line-editing loop free-runs. Attaching USB makes the reads block, which is why
 * the fault vanishes precisely when it becomes observable.
 *
 * Waiting in a task rather than testing once at boot: enumeration is not complete the
 * instant app_main runs, and the cable can be plugged in at any time. The poll blocks on
 * vTaskDelay, so it costs nothing while it waits. */
static void console_repl_start(void);

static void console_wait_for_host(void *arg)
{
    while (!usb_serial_jtag_is_connected()) vTaskDelay(pdMS_TO_TICKS(500));
    console_repl_start();
    vTaskDelete(NULL);
}

void console_start(void)
{
    if (xTaskCreate(console_wait_for_host, "con_wait", 4096, NULL, 2, NULL) != pdPASS)
        ESP_LOGE("console", "xTaskCreate(con_wait) failed -- console unavailable");
}

static void console_repl_start(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t rc = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    rc.prompt = "valvectl>";
    esp_console_dev_usb_serial_jtag_config_t hw = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw, &rc, &repl));

    const esp_console_cmd_t cmds[] = {
        {"status","print sensor + control status",NULL,cmd_status,NULL},
        {"valve","set valve percent (0-100)",NULL,cmd_valve,NULL},
        {"resync","force valve resync",NULL,cmd_resync,NULL},
        {"mode","print detected mode",NULL,cmd_mode,NULL},
        {"factory-reset","zigbee leave + NVS wipe",NULL,cmd_freset,NULL},
        {"stats","1-Wire failure tallies (this run + previous)",NULL,cmd_stats,NULL},
        {"zbtemp","temperature attribute-write tallies (this run + previous)",NULL,cmd_zbtemp,NULL},
        {"hb","per-task heartbeat ages (this run + previous)",NULL,cmd_hb,NULL},
    };
    for (size_t i=0;i<sizeof(cmds)/sizeof(cmds[0]);++i) ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
