#pragma once

#include <OpenHaldexC6_defs.h>
#include <cstring>

// Legacy WiFi header - now just a stub
// All WiFi functionality has moved to OpenHaldexC6_WebServer.h

// These functions are no longer used - kept for compatibility
void setupWiFi();
void disconnectWifi();
void resetWifiPassword(); // clears WiFi password and restarts AP as open network
void resetWifiSsid();     // restores default SSID and restarts AP
void resetWifi();         // clears password AND restores default SSID, restarts AP
void resetWifiSta();      // clears home-network (bridge mode) credentials, disables STA, restarts AP
void applyWifiMode();     // (re)applies AP (+ STA if configured) using the current stored credentials
void pollWifiSta();       // call periodically from loop() to track STA connect/reconnect state
inline void updateLabels(void *arg) {}