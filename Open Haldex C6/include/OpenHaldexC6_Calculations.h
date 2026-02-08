#ifndef OPENHALDEXC6_CALCULATIONS_H
#define OPENHALDEXC6_CALCULATIONS_H

#include <OpenHaldexC6_defs.h>

float get_lock_target_adjustment();
uint8_t get_lock_target_adjusted_value(uint8_t value, bool invert);
void getLockData(twai_message_t& rx_message_chs);

#endif // OPENHALDEXC6_CALCULATIONS_H
