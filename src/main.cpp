/*
OpenHaldex C6 Firmware
Copyright (c) 2026 Forbes Automotive

This file is part of the OpenHaldex C6 project.

Licensed under the Forbes Automotive Source-Available License (FASL) v1.0.

Personal, educational, and non-commercial use is permitted.
Commercial use, including selling hardware running this firmware,
is strictly prohibited without written permission from Forbes Automotive.

See the LICENSE file in the root of this repository for full license terms.
Project repository: https://github.com/Forbes-Automotive/OpenHaldex-C6
*/

#include <OpenHaldexC6_defs.h>
#include <OpenHaldexC6_can.h>
#include <OpenHaldexC6_EEP.h>
#include <OpenHaldexC6_IO.h>
#include <OpenHaldexC6_OTA.h>
#include <OpenHaldexC6_WiFi.h>
#include <OpenHaldexC6_Analyzer.h>
#include <OpenHaldexC6_API.h>

void setup()
{
#if enableDebug || detailedDebug || detailedDebugCAN || detailedDebugWiFi || detailedDebugEEP || detailedDebugIO
  Serial.begin(500000);                // start serial at a high baud rate for debugging
  Serial.setTxTimeoutMs(10);           // set a small timeout for Serial writes to prevent blocking if the Serial Monitor is not open
  DEBUG("OpenHaldex-C6 Launching..."); // debug message to indicate startup
#endif

  readEEP();        // read previously stored settings in EEPROM
  setupIO();        // setup IO
  setupCAN();       // bring CAN online
  setupButtons();   // setup mode & external mode buttons
  setupTasks();     // setup tasks
  setupWiFi();      // setup WiFi
  setupWebServer(); // setup WebServer
  setupAPI();       // setup API handling for WebServer
  setupOTA();       // setup Over-the-Air Updates

  if (needsFirmwareConfirmation())
  {
    DEBUG("[OTA SAFETY] New firmware detected - confirming after safety checks...");
    // Small delay to ensure CAN buses are fully initialized
    delay(100);
    confirmFirmwareValidity();
  }
}

void loop()
{
  delay(1); // add a small delay to prevent watchdog resets and allow other tasks to run

  if (rebootWiFi)
  {
#if detailedDebugWiFi
    DEBUG("Restarting WiFi...");
#endif

    for (int i = 0; i <= 3; i++)
    {
      strip.setLedColorData(led_channel, led_brightness, led_brightness, led_brightness);
      strip.show();
      delay(50);
      strip.setLedColorData(led_channel, 0, 0, 0);
      strip.show();
      delay(50);
    }

    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);

    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(IPAddress(192, 168, 1, 1), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
    WiFi.softAP(wifiHostName);
    WiFi.setSleep(false);

    rebootWiFi = false;
  }
}