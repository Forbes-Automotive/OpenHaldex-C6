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
inline void updateLabels(void *arg) {}