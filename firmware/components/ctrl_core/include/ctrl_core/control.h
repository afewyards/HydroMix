#pragma once
#include "ctrl_core/types.h"
#include "ctrl_core/mode_detect.h"
#include "ctrl_core/feedforward.h"
#include "ctrl_core/pi.h"
#include "ctrl_core/governor.h"
#include "ctrl_core/degradation.h"
#include "ctrl_core/alarm.h"

#define TRANSIT_MOVE_PCT    0.5f
#define TRANSIT_RELEASE_K   0.25f

typedef struct {
    mode_detect_state_t mode;
    ff_state_t          ff;
    pi_state_t          pi;
    gov_state_t         gov;
    alarm_state_t       alarm;
    ctrl_mode_t         last_mode;
    bool                prev_resync;
    bool                inited;
    uint32_t            last_now_ms;
    bool                have_now;
    float               last_pos;
    bool                have_pos;
    bool                holding;
    uint32_t            hold_until_ms;
    float               hold_supply_ref;
    float               latched_trim;
} control_state_t;

typedef struct {
    float t_source_f, t_return_f;   /* filtered */
    float t_supply;                 /* raw */
    float hx_a;
    float valve_pos;                /* actual estimated valve position, % */
    sensor_faults_t faults;
    bool  water_running;
    bool  resync_active;
    bool  link_up;                  /* ZDO-observed join state; STATUS ONLY -- the cooling
                                       dew guard deliberately does not consume it, since a
                                       silently dead coordinator leaves it true forever */
    uint32_t link_last_seen_ms;     /* last inbound ZCL activity or ZDO signal */
} control_in_t;

typedef struct {
    float      heat_setpoint;       /* clamped <=35 by caller/config */
    float      cool_setpoint;       /* clamped >=17 */
    float      park_pos;
    mode_cfg_t mode_cfg;
    pi_cfg_t   pi_cfg;
    ff_cfg_t   ff_cfg;
    gov_cfg_t  gov_cfg;
    uint32_t   alarm_dwell_ms;
    float      deadtime_s;
} control_cfg_t;

typedef struct {
    float           valve_target;
    bool            regulating;
    ctrl_mode_t     mode;
    bool            supply_alarm;
    uint16_t        fault_bits;
    ctrl_strategy_t strategy;
} control_out_t;

void          control_init(control_state_t *s);
control_out_t control_step(control_state_t *s, const control_in_t *in,
                           const control_cfg_t *cfg, uint32_t now_ms);
