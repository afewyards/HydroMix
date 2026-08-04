#include "console.h"
#include <string.h>
#include <stdlib.h>
#include "esp_console.h"
#include "esp_log.h"

__attribute__((weak)) void console_hook_status(char *o, size_t n){ snprintf(o,n,"no status\n"); }
__attribute__((weak)) void console_hook_valve(int pct){ (void)pct; }
__attribute__((weak)) void console_hook_resync(void){}
__attribute__((weak)) void console_hook_mode(char *o, size_t n){ snprintf(o,n,"IDLE\n"); }
__attribute__((weak)) void console_hook_factory_reset(void){}
__attribute__((weak)) void console_hook_stats(char *o, size_t n){ snprintf(o,n,"no stats\n"); }

static int cmd_status(int c, char **v){ char b[256]; console_hook_status(b,sizeof b); printf("%s",b); return 0; }
static int cmd_valve(int c, char **v){ if(c<2){printf("usage: valve <0-100>\n");return 1;} console_hook_valve(atoi(v[1])); return 0; }
static int cmd_resync(int c, char **v){ console_hook_resync(); printf("resync requested\n"); return 0; }
static int cmd_mode(int c, char **v){ char b[64]; console_hook_mode(b,sizeof b); printf("%s",b); return 0; }
static int cmd_freset(int c, char **v){ console_hook_factory_reset(); printf("factory reset\n"); return 0; }
/* Two runs x 5 sensors x 8 counters; 1280 keeps the whole dump un-truncated. */
static int cmd_stats(int c, char **v){ char b[1280]; console_hook_stats(b,sizeof b); printf("%s",b); return 0; }

void console_start(void)
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
    };
    for (size_t i=0;i<sizeof(cmds)/sizeof(cmds[0]);++i) ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
