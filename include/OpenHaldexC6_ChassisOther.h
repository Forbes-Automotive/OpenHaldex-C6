#pragma once
#include <OpenHaldexC6_defs.h>

// Chassis-bus decoding for the non-VW cars (V10.00.0): vehicle speed, accelerator
// pedal, engine RPM, brake switch and handbrake, so Expert mode, the speed /
// throttle limits and brake / handbrake follow work on these cars too.

bool hasOtherCarChassisDecode(uint8_t generation);        // true for 41 (Insignia), 42 (Focus RS), 43 (Volvo P3), 44 (SAAB 9-3), 21 (Volvo P2 2002)
void decodeOtherCarChassis(const twai_message_t &frame); // read one chassis frame for the active generation
