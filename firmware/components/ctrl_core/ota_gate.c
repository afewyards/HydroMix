#include "ctrl_core/ota_gate.h"

bool ota_gate_step(uint32_t *cycles, uint32_t threshold){
    if (*cycles >= threshold) return false;   /* already fired, or threshold 0 */
    (*cycles)++;
    return *cycles == threshold;
}
