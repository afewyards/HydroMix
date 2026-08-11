#pragma once
#include <stddef.h>
void console_start(void);
void console_hook_zbtemp(char *o, size_t n);
/* Weak hooks — later modules override. */
void console_hook_status(char *out, size_t n);
void console_hook_valve(int pct);
void console_hook_resync(void);
void console_hook_mode(char *out, size_t n);
void console_hook_factory_reset(void);
void console_hook_stats(char *out, size_t n);
