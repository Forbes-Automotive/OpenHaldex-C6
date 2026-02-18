#include <OpenHaldexC6_WiFi.h>

// Legacy WiFi implementation - now a stub
// All WiFi functionality has moved to OpenHaldexC6_WebServer.cpp

void setupWiFi()
{
  // WiFi setup is now in main.cpp
  WiFi.hostname(wifiHostName);
  DEBUG("Creating Access Point...");
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192, 168, 1, 1), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(wifiHostName);
  WiFi.setSleep(false);
  DEBUG("WiFi AP started: %s", wifiHostName);
  DEBUG("IP address: 192.168.1.1");
}

void disconnectWifi()
{
  rebootWiFi = true;
}