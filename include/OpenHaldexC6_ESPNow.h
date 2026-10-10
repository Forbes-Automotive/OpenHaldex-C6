#pragma once
#include <OpenHaldexC6_defs.h>

// ---------------------------------------------------------------------------
// OHX - OpenHaldex over ESP-NOW for can2gauge (and any other display). Wire format: include/ohx_proto.h (shared with
// can2gauge, keep the two copies identical).
//   - broadcasts the live state every 100 ms on the AP's channel, so a gauge with power only (no CAN) shows the Haldex
//   - accepts signed commands (mode, generation, learn, controller on / off). The key is the AP password, so a gauge
//     gets exactly the access the web UI gives. Generation and learn commands need the car stopped.
// The ESP-NOW stack follows the WiFi: it is (re)started whenever the AP is up and stopped before WiFi goes off.
// ---------------------------------------------------------------------------
extern bool espNowEnabled; // broadcast the live state (persisted, default on)
extern bool espNowControl; // accept commands from gauges (persisted, default on)

void setupESPNow();
void espNowStop(); // call before WiFi.mode(WIFI_OFF); the task starts it again once the AP is back
