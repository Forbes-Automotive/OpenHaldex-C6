/*
OpenHaldex-C6 - Forbes Automotive
Haldex Controller for Gen1, Gen2 and Gen4 Haldex Controllers.  Supports WiFi.  Version: 1.10.

Codebase derived from OpenHaldex 4.0 - CAN data is the same, just ported to ESP32 C6.
*/


#include <OpenHaldexC6_defs.h>
#include <OpenHaldexC6_can.h>
#include <OpenHaldexC6_EEP.h>
#include <OpenHaldexC6_IO.h>
#include <OpenHaldexC6_OTA.h>
#include <OpenHaldexC6_WiFi.h>
#include <OpenHaldexC6_Analyzer.h>

void setup() {
#if enableDebug || detailedDebug || detailedDebugCAN || detailedDebugWiFi || detailedDebugEEP || detailedDebugIO
  Serial.begin(500000);  // if ANY Serial is required, begin
  DEBUG("OpenHaldex-C6 Launching...");
#endif

  readEEP();       // read EEPROM for stored settings
  setupIO();       // setup gpio for input / output
  setupCAN();      // setup two CAN buses
  setupButtons();  // setup 'buttons' for changing mode (internal and external)
  setupTasks();    // setup tasks for each of the main functions - CAN Chassis/Haldex handling, Serial prints, Standalone, etc
  connectWifi();   // enable / start WiFi
  setupUI();       // setup wifi user interface
  setupOTA();      // setup OTA update server
}

void loop() {
  delay(100);  // literally here to give more CPU time to tasks

  if (rebootWiFi) {
#if enableDebug
    DEBUG("Restarting WiFi...");
#endif

    for (int i = 0; i <= 3; i++) {
      strip.setLedColorData(led_channel, led_brightness, led_brightness, led_brightness);
      strip.show();
      delay(50);
      strip.setLedColorData(led_channel, 0, 0, 0);
      strip.show();
      delay(50);
    }

    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);

    connectWifi();
    rebootWiFi = false;
  }
}

