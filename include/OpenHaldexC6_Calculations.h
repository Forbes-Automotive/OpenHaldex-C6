#pragma once

#include <OpenHaldexC6_defs.h>

float get_lock_target_adjustment();
static float get_expert_lock_target();
uint8_t get_lock_target_adjusted_value(uint8_t value, bool invert);
void getLockData(twai_message_t& rx_message_chs);