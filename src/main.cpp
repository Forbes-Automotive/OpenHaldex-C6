/*
OpenHaldex-C6 - Forbes Automotive
Haldex Controller for Gen1, Gen2, Gen4 and Gen5 Haldex Controllers
Version: 8.00.5
*/

#include <OpenHaldexC6_defs.h>
#include <OpenHaldexC6_can.h>
#include <OpenHaldexC6_EEP.h>
#include <OpenHaldexC6_IO.h>
#include <OpenHaldexC6_OTA.h>
#include <OpenHaldexC6_WiFi.h>
#include <OpenHaldexC6_Analyzer.h>
#include <OpenHaldexC6_API.h>
#include <OpenHaldexC6_ESPNow.h> // live state to can2gauge (and other displays) over ESP-NOW
#include <OpenHaldexC6_BLE.h>
#include <ESPmDNS.h> // for mDNS responder to allow openhaldex.local access to the web UI without needing to know the IP address
#include "esp_pm.h"  // for power management when CAN sleep enabled
#include "esp_heap_caps.h"

#if debugMemory
// Heap and per-task stack headroom, to size task stacks from evidence. On the
// C6, IRAM code, static RAM, task stacks, WiFi and BLE all come out of the same
// 512 KB SRAM - with too little left, BLE fails to init and the web server
// truncates pages. Stack "free" = high-water mark: the least unused stack that
// task has ever had (bytes on ESP-IDF).
static void printMemoryReport()
{
  DEBUG("MEM: heap free %u, min ever %u, largest block %u",
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
        (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
        (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
  twai_status_info_t s0 = {}, s1 = {};
  twai_get_status_info_v2(twai_bus_0, &s0);
  twai_get_status_info_v2(twai_bus_1, &s1);
  DEBUG("MEM: CAN rx missed/overrun chassis %lu/%lu, haldex %lu/%lu",
        s0.rx_missed_count, s0.rx_overrun_count, s1.rx_missed_count, s1.rx_overrun_count);
  const UBaseType_t n = uxTaskGetNumberOfTasks();
  TaskStatus_t *ts = static_cast<TaskStatus_t *>(malloc(n * sizeof(TaskStatus_t)));
  if (ts == nullptr)
  {
    DEBUG("MEM: no heap for the task list");
    return;
  }
  const UBaseType_t got = uxTaskGetSystemState(ts, n, nullptr);
  for (UBaseType_t i = 0; i < got; i++)
  {
    DEBUG("MEM:   %-20s stack free %5u", ts[i].pcTaskName, (unsigned)ts[i].usStackHighWaterMark);
  }
  free(ts);
}
#endif

void setup()
{
#if enableDebug || detailedDebug || detailedDebugCAN || detailedDebugWiFi || detailedDebugEEP || detailedDebugIO
  Serial.begin(500000);      // start serial at a high baud rate for debugging
  Serial.setTxTimeoutMs(10); // set a small timeout for Serial writes to prevent blocking if the Serial Monitor is not open
  // native USB-CDC: give the host up to 3s to attach so early boot logs (LittleFS/WiFi/mDNS) aren't lost before it connects
  unsigned long serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 3000)
  {
    delay(10);
  }
  delay(250);                          // small grace period after attach before the first message is sent
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
  setupESPNow();    // gauges over ESP-NOW (follows the AP: started / stopped with it)
  setupBLE();       // setup BLE link to the DashCAN app (stack comes up from its own task)

  // Power management: when CAN sleep is enabled, scale CPU frequency down
  if (canSleepEnabled)
  {
    esp_pm_config_t pm_cfg = {
        .max_freq_mhz = 160,
        // Aggressive: drop CPU lower limit to 10MHz (XTAL/N) for deeper idle.
        .min_freq_mhz = canSleepAggressive ? 10 : 40,
        .light_sleep_enable = true,
    };
    esp_err_t pm_err = esp_pm_configure(&pm_cfg);
    if (pm_err != ESP_OK)
    {
      DEBUG("ESP Power Management Failed: %d (continuing without CPU frequency scaling)", (int)pm_err);
    }

    // Automatic light sleep can power down the TWAI domain (sleep_allow_pd in
    // setupCAN). Hold a no-light-sleep lock while the module is awake so that
    // can't happen with the car running; updateTriggers releases it only when
    // the low-power state machine deliberately sleeps, and takes it back on wake.
    esp_pm_lock_handle_t lk = nullptr;
    if (esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "can_awake", &lk) == ESP_OK)
    {
      pmNoLightSleepLock = (void *)lk;
      esp_pm_lock_acquire(lk);
    }
  }

  if (needsFirmwareConfirmation())
  {
    DEBUG("[OTA SAFETY] New firmware detected - confirmation deferred to otaRollbackTick()");
  }
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(100)); // yield the Arduino loop task so other FreeRTOS tasks can run

  otaRollbackTick(); // confirm a freshly-installed OTA image once the device has proven itself

#if debugMemory
  {
    static uint32_t nextMemReport = 15000; // after WiFi, web server, ESP-NOW and BLE have all started
    if (millis() >= nextMemReport)
    {
      nextMemReport = millis() + 60000;
      printMemoryReport();
    }
  }
#endif

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
    espNowStop(); // before the radio goes
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
      vTaskDelay(pdMS_TO_TICKS(50));
      strip.setLedColorData(led_channel, 0, 0, 0);
      strip.show();
      vTaskDelay(pdMS_TO_TICKS(50));
    }

    espNowStop();                // the ESP-NOW task brings it back once the AP is up again
    WiFi.disconnect(true, true); // disconnect and erase AP settings to ensure a clean restart
    WiFi.mode(WIFI_OFF);         // turn off WiFi to reset the state

    startSoftAP(); // restart in AP mode (local-only DHCP, current SSID/password)
    MDNS.end();
    MDNS.begin("openhaldex"); // restart openhaldex.local
    MDNS.addService("http", "tcp", 80);

    rebootWiFi = false;
  }

  pollWifiSta(); // bridge mode: track the home-network connection (no-op when not configured)
}