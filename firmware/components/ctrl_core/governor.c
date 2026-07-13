#include "ctrl_core/governor.h"

void gov_init(gov_state_t *s){ s->active = false; }

float gov_step(gov_state_t *s, float target_in, float t_supply, const gov_cfg_t *cfg){
    if (!s->active){
        if (t_supply > cfg->gov_high || t_supply < cfg->gov_low) s->active = true;
    } else {
        if (t_supply <= cfg->rel_high && t_supply >= cfg->rel_low) s->active = false;
    }
    return s->active ? 0.0f : target_in;   /* 0 % = recirc = de-escalation in either mode */
}
