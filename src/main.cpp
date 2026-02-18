/*
OpenHaldex-C6 - Forbes Automotive
Haldex Controller for Gen1, Gen2 and Gen4 Haldex Controllers. Supports WiFi. Version: 2.01 - now ported to PlatformIO.
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
  Serial.begin(500000);
  DEBUG("OpenHaldex-C6 Launching...");
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
}

void loop()
{
  delay(1);

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