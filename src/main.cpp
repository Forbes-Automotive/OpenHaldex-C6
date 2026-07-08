/*
OpenHaldex-C6 - Forbes Automotive
Haldex Controller for Gen1, Gen2, Gen4 and Gen5 Haldex Controllers. Supports WiFi. Version: 3.10 - now ported to PlatformIO.
*/

#include <OpenHaldexC6_defs.h>
#include <OpenHaldexC6_can.h>
#include <OpenHaldexC6_EEP.h>
#include <OpenHaldexC6_IO.h>
#include <OpenHaldexC6_OTA.h>
#include <OpenHaldexC6_WiFi.h>
#include <OpenHaldexC6_Analyzer.h>
#include <OpenHaldexC6_API.h>
#include <ESPmDNS.h> // for mDNS responder to allow openhaldex.local access to the web UI without needing to know the IP address
#include "esp_pm.h"  // for power management when CAN sleep enabled

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

  // Power management: when CAN sleep is enabled, scale CPU frequency down
  if (canSleepEnabled)
  {
    esp_pm_config_t pm_cfg = {
        .max_freq_mhz = 160,
        // Aggressive: drop CPU floor to 10MHz (XTAL/N) for deeper idle.
        .min_freq_mhz = canSleepAggressive ? 10 : 40,
        .light_sleep_enable = true,
    };
    esp_err_t pm_err = esp_pm_configure(&pm_cfg);
    if (pm_err != ESP_OK)
    {
      DEBUG("ESP Power Management Failed: %d (continuing without CPU frequency scaling)", (int)pm_err);
    }
  }

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

  { // temp counters for debugging, just left in because they can be useful for testing timing of various functions/tasks
    tempCounter++;
    if (tempCounter > 5)
    {
      tempCounter = 0;
      tempCounter1++;
    }

    if (tempCounter1 > 10)
    {
      tempCounter1 = 0;
      tempCounter2++;
    }

    if (tempCounter2 > 254)
    {
      tempCounter2 = 0;
    }
  }

  // Low-power mode: shut down WiFi AP when there has been no CAN traffic for 5+ minutes
  // and no devices are connected. WiFi restarts automatically when CAN traffic returns
  if (lowPowerMode && WiFi.getMode() != WIFI_OFF)
  {
#if detailedDebugWiFi
    DEBUG("Low Power: Disabling WiFi AP");
#endif
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
  }

  if (rebootWiFi)
  {
#if detailedDebugWiFi
    DEBUG("Restarting WiFi...");
#endif

    // just here to show a visual indication of the WiFi reboot
    for (int i = 0; i <= 3; i++)
    {
      strip.setLedColorData(led_channel, ledBrightness, ledBrightness, ledBrightness);
      strip.show();
      delay(50);
      strip.setLedColorData(led_channel, 0, 0, 0);
      strip.show();
      delay(50);
    }

    WiFi.disconnect(true, true); // disconnect and erase AP settings to ensure a clean restart
    WiFi.mode(WIFI_OFF);         // turn off WiFi to reset the state

    WiFi.mode(WIFI_AP); // restart in AP mode
    WiFi.softAPConfig(IPAddress(192, 168, 1, 1), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
    if (strlen(wifiPassword) >= 8)
    {
      WiFi.softAP(wifiHostName, wifiPassword); // password-protected network
    }
    else
    {
      WiFi.softAP(wifiHostName); // open network
    }
    WiFi.setSleep(false);
    // Aggressive sleep: trim AP TX power to reduce active-WiFi current.
    if (canSleepAggressive)
    {
      WiFi.setTxPower(WIFI_POWER_8_5dBm);
    }
    MDNS.end();
    MDNS.begin("openhaldex"); // restart openhaldex.local
    MDNS.addService("http", "tcp", 80);

    rebootWiFi = false;
  }
}