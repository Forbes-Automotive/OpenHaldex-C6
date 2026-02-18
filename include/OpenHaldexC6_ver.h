#pragma once
#include <OpenHaldexC6_defs.h>

// Current firmware version
#define FW_VERSION "2.01"

/*
Version Control:

*** remember and update FW_VERSION in '_defs.h' ***

V1.00 - basic code for testing
V1.01 - added in reliable mode changing and eeprom saving
V1.02 - confirmed Gen1 (OEM & Standalone), Gen4 (Standalone)
V1.03 - confirmed Gen2, updated general codebase
V1.04 - added feedback for incoming brake/handbrake sensors.  Added options to invert if required
V1.05 - brake out GPIO mapped incorrectly - changed
V1.06 - added 6040 split
V1.07 - added OTA updates (!) Thanks to Sasha!
V1.08 - Sorted crash when long press / WiFi reset
V1.09 - revised throttle AND speed setpoints to work together with enable/disable lock - thanks to Chris!
V1.10 - added custom mode to allow custom lock percentage (based on speed/throttle/body)
V1.11 - added SavvyCAN

V2.00 - changed to PlatformIO / VS Code
V2.01 - added custom UI with:
        > 'on TC, enable 5050' checkbox & 'on ext. button hold, enable 5050' checkbox
        > expert editor with 7x7 array of speed/throttle/lock

** to do **:
        > add 'ota' to match existing layout
        > add reduction in throttle/speed off
        > add ABS stale, use ECU output
        > look into correction_factor (_calculations.cpp) and look to adjust to bring request inline with actual
        > move CAN into interrupt based - ESP_INTR_FLAG_IRAM
*/