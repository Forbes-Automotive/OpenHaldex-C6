/*
OpenHaldex-C6 - Forbes Automotive
Haldex Controller for Gen1, Gen2 and Gen4 Haldex Controllers.  Supports WiFi.  Version: 1.06.  Versions in '_ver.ino'

Codebase derived from OpenHaldex 4.0 - CAN data is the same, just ported to ESP32 C6.
*/

#include <OpenHaldexC6_defs.h>

// Current firmware version (must match OpenHaldexC6_ver.h)
#define FW_VERSION "1.10"

void setup() {
#if enableDebug || detailedDebug || detailedDebugCAN || detailedDebugWiFi || detailedDebugEEP || detailedDebugIO
  Serial.begin(500000);  // if ANY Serial is required, begin
  DEBUG("OpenHaldex-C6 Launching...");
#endif

  readEEP();       // read EEPROM for stored settings  - in '_EEP.ino'
  setupIO();       // setup gpio for input / output  - in '_io.ino'
  setupCAN();      // setup two CAN buses  - in '_io.ino'
  setupButtons();  // setup 'buttons' for changing mode (internal and external) - in '_io.ino'
  setupTasks();    // setup tasks for each of the main functions - CAN Chassis/Haldex handling, Serial prints, Standalone, etc - in '_io.ino'
  connectWifi();   // enable / start WiFi - in '_wifi.ino'
  setupUI();       // setup wifi user interface - in '_wifi.ino'
  setupDashboard(); // setup dashboard - in '_Dashboard.ino'
  setupOTA();      // setup OTA update server - in '_OTA.ino'
}

void loop() {
  loopDashboard();
  delay(100);  // literally here to give more CPU time to tasks

  if (rebootWiFi) {
#if enableDebug
    DEBUG("Restarting WiFi...");
#endif

    for (int i = 0; i <= 3; i++) {
      strip.setLedColorData(led_channel, led_brightness, led_brightness, led_brightness);  // red
      strip.show();
      delay(50);
      strip.setLedColorData(led_channel, 0, 0, 0);  // red
      strip.show();
      delay(50);
    }

    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);

    connectWifi();  // enable / start WiFi - in '_wifi.ino'
    //setupUI();      // setup wifi user interface - in '_wifi.ino'
    rebootWiFi = false;
  }
}