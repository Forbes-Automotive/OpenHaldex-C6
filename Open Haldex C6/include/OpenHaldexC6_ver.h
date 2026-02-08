#ifndef OPENHALDEXC6_VER_H
#define OPENHALDEXC6_VER_H

/*
Version Control:

*** remember and update FW_VERSION in '_defs.h' ***

V1.00 - basic code for testing 
V1.01 - added in reliable mode changing and eeprom saving
V1.02 - confirmed Gen1 (OEM & Standalone), Gen4 (Standalone)
V1.03 - confirmed Gen2, updated general codebase
V1.04 - added feedback for incoming brake/handbrake sensors.  Added options to invert if required.
V1.05 - brake out GPIO mapped incorrectly - changed
V1.06 - added 6040 split
V1.07 - added OTA updates (!) Thanks to Sasha!
V1.08 - Sorted crash when long press / WiFi reset
V1.09 - revised throttle AND speed setpoints to work together with enable/disable lock - thanks to Chris!
V1.10 - added custom mode to allow custom lock percentage (based on speed/throttle/body)
V2.00 - changed to PlatformIO / VS Code
*/

// Current firmware version
#define FW_VERSION "1.10"

#endif // OPENHALDEXC6_VER_H
