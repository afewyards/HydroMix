#pragma once
#include <stdbool.h>

#define VALVECTL_MFR_CODE 0x1234   /* private/test manufacturer code (Unresolved Q1) */
#define EP_MAIN     1
#define EP_T_SUPPLY 2
#define EP_T_RETURN 3
#define EP_T_SOURCE 4
#define EP_T_HXA    5
#define EP_T_HXB    6

#define VALVECTL_CUSTOM_CLUSTER_ID 0xFC00
/* custom attribute ids (manufacturer-specific) */
#define ATTR_HEAT_THRESHOLD  0x0000
#define ATTR_COOL_THRESHOLD  0x0001
#define ATTR_TRAVEL_TIME_S   0x0002
#define ATTR_PARK_POS        0x0003
#define ATTR_DIRECTION_SWAP  0x0004
#define ATTR_KP              0x0005
#define ATTR_KI              0x0006
#define ATTR_GOV_HIGH        0x0007
#define ATTR_GOV_LOW         0x0008
#define ATTR_ALARM_DWELL     0x0009
#define ATTR_RESYNC          0x000A   /* self-clearing bool */
#define ATTR_ALARM_BITMAP    0x000B
#define ATTR_FAULT_BITMAP    0x000C
#define ATTR_TRAVEL_SINCE    0x000D
#define ATTR_DEADTIME_S      0x000E
#define ATTR_PI_DEADBAND     0x000F
/* Regulation targets, writable here because the standard thermostat cluster isn't: ZBOSS
 * enforces heat<=cool-deadband on OccupiedHeating/CoolingSetpoint, and this device keeps
 * independent seasonal targets (heat 35 / cool 18) that always violate that, so every ZCL
 * write to those attrs returns INVALID_VALUE before firmware ever sees it. Mirror
 * g_config.heat_setpoint/cool_setpoint (clamp [17,35], same as config_apply_custom()). */
#define ATTR_HEAT_SETPOINT   0x0010
#define ATTR_COOL_SETPOINT   0x0011

void zigbee_start(void);
void zigbee_steer(void);
void zigbee_leave(void);
bool zigbee_joined(void);
void zigbee_report_temps(void);
void zigbee_push_status(void);
void zigbee_on_join(void);   /* weak */
