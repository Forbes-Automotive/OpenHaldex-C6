#include <OpenHaldexC6_OTA.h>
#include <Update.h>
#include <LittleFS.h>
#include <mbedtls/sha256.h>

#define OTA_PASSWORD "haldex"

//static AsyncWebServer *otaServer = nullptr;

// ============================================================================
// SAFETY-CRITICAL: Configuration
// ============================================================================

// OTA password - CHANGE THIS FOR PRODUCTION USE!
#define OTA_PASSWORD "haldex"

// OTA partition labels (must match partition table)
#define OTA_PARTITION_LABEL_0 "ota_0"
#define OTA_PARTITION_LABEL_1 "ota_1"

// Safety check timeout (ms) - how long to wait for safety conditions
#define OTA_SAFETY_CHECK_TIMEOUT_MS 5000

// ============================================================================
// SAFETY-CRITICAL: State Variables
// ============================================================================

static const char *TAG = "OTA";
static AsyncWebServer *otaServer = nullptr;
static bool otaUpdateInProgress = false;
static esp_ota_handle_t otaHandle = 0;
static const esp_partition_t *otaPartition = nullptr;

// Firmware confirmation flag - set to true only after all safety checks pass
static bool firmwareConfirmed = false;

// Deferred rollback confirmation (see otaRollbackTick). The Arduino core
// normally marks a freshly-booted OTA image valid before setup() runs; we
// override verifyRollbackLater() so the image is only confirmed once the
// device has proven itself (web UI reachable, or a clean uptime window).
static bool rollbackPending = false;
static bool webServedOk = false;
#define OTA_CONFIRM_UPTIME_MS 60000UL // confirm after 60s of uptime even if no client connected

// Optional integrity check: the uploader (guided OTA) passes the expected
// SHA-256 as ?sha256=<64 hex>; the handler hashes chunks as they arrive and
// refuses to activate the image on mismatch. Manual uploads with no hash are
// accepted as before.
static mbedtls_sha256_context otaSha;
static bool otaShaActive = false;
static char otaShaExpected[65] = "";

static void otaShaBegin(AsyncWebServerRequest *request) {
  otaShaActive = false;
  otaShaExpected[0] = '\0';
  const AsyncWebParameter *p = request->hasParam("sha256") ? request->getParam("sha256") : nullptr;
  if (p == nullptr || p->value().length() != 64) return;
  strncpy(otaShaExpected, p->value().c_str(), 64);
  otaShaExpected[64] = '\0';
  for (char *c = otaShaExpected; *c; ++c) *c = tolower(*c);
  mbedtls_sha256_init(&otaSha);
  mbedtls_sha256_starts(&otaSha, 0);
  otaShaActive = true;
}

static void otaShaUpdate(const uint8_t *data, size_t len) {
  if (otaShaActive && len) mbedtls_sha256_update(&otaSha, data, len);
}

// Returns true when no hash was requested or the computed hash matches.
static bool otaShaFinishOk() {
  if (!otaShaActive) return true;
  uint8_t digest[32];
  mbedtls_sha256_finish(&otaSha, digest);
  mbedtls_sha256_free(&otaSha);
  otaShaActive = false;
  char hex[65];
  for (int i = 0; i < 32; ++i) sprintf(hex + i * 2, "%02x", digest[i]);
  hex[64] = '\0';
  bool ok = (strcmp(hex, otaShaExpected) == 0);
#if enableDebug || detailedDebugWiFi
  DEBUG("[OTA] SHA-256 %s (got %s)", ok ? "OK" : "MISMATCH", hex);
#endif
  return ok;
}

// Read the web UI's own version string from /version.json on LittleFS
// ("--" if the file is missing, e.g. a pre-8.00.5 filesystem).
static String readFsVersion() {
  File f = LittleFS.open("/version.json", "r");
  if (!f) return "--";
  String body = f.readString();
  f.close();
  int k = body.indexOf("\"fs\"");
  if (k < 0) return "--";
  int q1 = body.indexOf('"', k + 4);
  int q2 = q1 >= 0 ? body.indexOf('"', q1 + 1) : -1;
  if (q1 < 0 || q2 < 0) return "--";
  return body.substring(q1 + 1, q2);
}

// ============================================================================
// SAFETY-CRITICAL: Check if system is in safe state for OTA update
// ============================================================================
// Behavior:
// - If CAN is NOT detected (bench setting): OTA allowed immediately.
// - If CAN IS detected (vehicle): enforce safety AND auto-revert to STOCK.
//
// Vehicle safety conditions:
// 1. Vehicle speed == 0
// 2. CAN buses operational (no bus failure)
// 3. Outputs safe: controller disabled OR mode switched to STOCK automatically
// 4. No active Haldex temp protection
//
// `allowEnforce` gates safety check 3's side effect (forcing state.mode to
// STOCK). Pass true only right before an actual firmware/filesystem write
// begins. Purely informational callers (the /ota/check status poll, which
// the web UI hits every few seconds just to render the OTA page, and the
// legacy /update page) must pass false so merely checking status - or just
// having the page open - doesn't silently kick the unit out of whatever
// standalone mode the user selected.
// ============================================================================
bool isSystemSafeForOTA(bool allowEnforce) {
  // BENCH MODE: No CAN detected -> allow OTA
  bool canDetected = (hasCANChassis || hasCANHaldex);
  if (!canDetected) {
#if enableDebug || detailedDebugWiFi
    DEBUG("[OTA SAFETY] CAN not detected - assuming BENCH mode: OTA allowed");
#endif
    return true;
  }

  // VEHICLE MODE: CAN detected -> enforce safety

  // SAFETY CHECK 1: Vehicle MUST be stationary
  if (received_vehicle_speed > 0) {
#if enableDebug || detailedDebugWiFi
    DEBUG("[OTA SAFETY] Vehicle moving: %d kmh - OTA BLOCKED", received_vehicle_speed);
#endif
    return false;
  }

  // SAFETY CHECK 2: CAN bus health
  if (isBusFailure) {
#if enableDebug || detailedDebugWiFi
    DEBUG("[OTA SAFETY] CAN bus failure detected - OTA BLOCKED");
#endif
    return false;
  }

  // SAFETY CHECK 3: Outputs safe
  // If controller is disabled, we're safe. Otherwise, either force STOCK
  // (real update about to start) or report not-safe without touching mode
  // (a status probe - the mode is left exactly as the user set it).
  if (!disableController) {
    if (state.mode != MODE_STOCK) {
      if (!allowEnforce) {
        return false;
      }
#if enableDebug || detailedDebugWiFi
      DEBUG("[OTA SAFETY] Controller active in non-stock mode - auto-switching to STOCK for OTA safety");
#endif
      state.mode = MODE_STOCK;
    }
  }

  // SAFETY CHECK 4: No active Haldex faults (temp protection)
  if (received_temp_protection) {
#if enableDebug || detailedDebugWiFi
    DEBUG("[OTA SAFETY] Haldex temperature protection active - OTA BLOCKED");
#endif
    return false;
  }

  // All safety checks passed
#if enableDebug || detailedDebugWiFi
  DEBUG("[OTA SAFETY] System safe for OTA update");
#endif
  return true;
}

// ============================================================================
// SAFETY-CRITICAL: Check if firmware needs confirmation on boot
// ============================================================================
// Call this early in setup() to check if we booted from a new OTA partition
// ============================================================================
bool needsFirmwareConfirmation() {
  esp_ota_img_states_t ota_state;
  esp_err_t err = esp_ota_get_state_partition(esp_ota_get_running_partition(), &ota_state);

  if (err != ESP_OK) {
    return false;
  }

  // If state is ESP_OTA_IMG_PENDING_VERIFY, firmware needs confirmation
  return (ota_state == ESP_OTA_IMG_PENDING_VERIFY);
}

// ============================================================================
// OTA Update Handler - SAFETY-CRITICAL: Blocks unsafe updates
// ============================================================================
//        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)

void handleOTAUpdate(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  // SAFETY CHECK: Block update if system is not safe
  if (!isSystemSafeForOTA()) {
    if (index == 0) {
      // First chunk - reject immediately
      request->send(403, "text/plain", "OTA BLOCKED: System not in safe state. Vehicle must be stationary, CAN initialized, outputs safe, no faults.");
#if enableDebug || detailedDebugWiFi
      DEBUG("[OTA SAFETY] Update rejected - system not safe");
#endif
    }
    return;
  }

  // First chunk - initialize OTA
  if (index == 0) {
    otaUpdateInProgress = true;

    // Get next OTA partition
    otaPartition = esp_ota_get_next_update_partition(NULL);
    if (otaPartition == NULL) {
      request->send(500, "text/plain", "OTA ERROR: No OTA partition found. Check partition table.");
      otaUpdateInProgress = false;
      return;
    }

#if enableDebug || detailedDebugWiFi
    DEBUG("[OTA] Starting update to partition: %s", otaPartition->label);
#endif

    // Begin OTA update
    esp_err_t err = esp_ota_begin(otaPartition, OTA_SIZE_UNKNOWN, &otaHandle);
    if (err != ESP_OK) {
      request->send(500, "text/plain", "OTA ERROR: Failed to begin update");
      otaUpdateInProgress = false;
      return;
    }
    otaShaBegin(request);
  }

  // Write data chunk
  otaShaUpdate(data, len);
  esp_err_t err = esp_ota_write(otaHandle, data, len);
  if (err != ESP_OK) {
    request->send(500, "text/plain", "OTA ERROR: Write failed");
    esp_ota_abort(otaHandle);
    otaUpdateInProgress = false;
    return;
  }

  // Final chunk - finish OTA
  if (final) {
    // Integrity: hash must match before the image is allowed to boot
    if (!otaShaFinishOk()) {
      request->send(400, "text/plain", "OTA ERROR: SHA-256 mismatch - image discarded");
      esp_ota_abort(otaHandle);
      otaUpdateInProgress = false;
      return;
    }
    err = esp_ota_end(otaHandle);
    if (err != ESP_OK) {
      if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
        request->send(400, "text/plain", "OTA ERROR: Image validation failed");
      } else {
        request->send(500, "text/plain", "OTA ERROR: End failed");
      }
      esp_ota_abort(otaHandle);
      otaUpdateInProgress = false;
      return;
    }

    // Set boot partition to new firmware
    err = esp_ota_set_boot_partition(otaPartition);
    if (err != ESP_OK) {
      request->send(500, "text/plain", "OTA ERROR: Failed to set boot partition");
      otaUpdateInProgress = false;
      return;
    }

#if enableDebug || detailedDebugWiFi
    DEBUG("[OTA] Update complete. Rebooting...");
    DEBUG("[OTA SAFETY] New firmware will require confirmation on boot");
#endif

    request->send(200, "text/plain", "OTA update complete. Rebooting... Firmware will be confirmed after safety checks pass.");

    // Small delay to allow response to be sent
    delay(1000);

    for (int i = 0; i <= 8; i++) {
      strip.setLedColorData(led_channel, ledBrightness/2, ledBrightness/2, ledBrightness/2);  // red
      strip.show();
      delay(50);
      strip.setLedColorData(led_channel, 0, 0, 0);  // red
      strip.show();
      delay(50);
    }

    // Reboot
    ESP.restart();
  } else {
    // Progress update
    request->send(200, "text/plain", "OK");
  }
}

// ============================================================================
// Filesystem (LittleFS) Update Handler - writes littlefs.bin to the "spiffs"
// data partition via the Arduino Update library (U_SPIFFS). Same safety gate
// as the firmware path. Used by the OTA page "Filesystem (web UI)" option.
// ============================================================================
void handleFSUpdate(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  // SAFETY CHECK: Block update if system is not safe
  if (!isSystemSafeForOTA()) {
    if (index == 0) {
      request->send(403, "text/plain", "OTA BLOCKED: System not in safe state.");
    }
    return;
  }

  // First chunk - begin filesystem update
  if (index == 0) {
    otaUpdateInProgress = true;
#if enableDebug || detailedDebugWiFi
    DEBUG("[OTA] Starting filesystem update: %s", filename.c_str());
#endif
    if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_SPIFFS)) {
      request->send(500, "text/plain", "OTA ERROR: Failed to begin filesystem update");
      otaUpdateInProgress = false;
      return;
    }
    otaShaBegin(request);
  }

  // Write data chunk
  otaShaUpdate(data, len);
  if (len && Update.write(data, len) != len) {
    request->send(500, "text/plain", "OTA ERROR: Filesystem write failed");
    Update.abort();
    otaUpdateInProgress = false;
    return;
  }

  // Final chunk - finish filesystem update
  if (final) {
    if (!Update.end(true)) {
      request->send(500, "text/plain", "OTA ERROR: Filesystem update failed");
      otaUpdateInProgress = false;
      return;
    }
    // Integrity: the partition is already written, but nothing boots from it -
    // report the mismatch so the UI retries step 1 before flashing firmware.
    if (!otaShaFinishOk()) {
      request->send(400, "text/plain", "OTA ERROR: Filesystem SHA-256 mismatch - re-upload the filesystem");
      otaUpdateInProgress = false;
      return;
    }

#if enableDebug || detailedDebugWiFi
    DEBUG("[OTA] Filesystem update complete. No reboot (upload firmware next).");
#endif

    // Filesystem does NOT reboot: the two-step OTA flow uploads the filesystem
    // first (step 1), then the firmware (step 2) reboots at the end.
    otaUpdateInProgress = false;
    request->send(200, "text/plain", "Filesystem update complete.");

    // brief green confirmation flash
    for (int i = 0; i <= 4; i++) {
      strip.setLedColorData(led_channel, 0, ledBrightness / 2, 0);
      strip.show();
      delay(40);
      strip.setLedColorData(led_channel, 0, 0, 0);
      strip.show();
      delay(40);
    }
  } else {
    request->send(200, "text/plain", "OK");
  }
}

// ============================================================================
// Setup OTA Server
// ============================================================================
void setupOTA() {
#if detailedDebugWiFi
  DEBUG("[OTA] Setting up OTA update server...");
#endif

  // Check if firmware needs confirmation
  rollbackPending = needsFirmwareConfirmation();
  if (rollbackPending) {
#if enableDebug
    DEBUG("[OTA SAFETY] New firmware detected - pending confirmation (web UI reached or %lus clean uptime)", OTA_CONFIRM_UPTIME_MS / 1000UL);
#endif
  }

  // Create OTA server

  // Info endpoint
  webServer.on("/ota/info", HTTP_GET, [](AsyncWebServerRequest *request) {
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_app_desc_t app_info;

    if (running != NULL) {
      esp_ota_get_partition_description(running, &app_info);
    }

    webServedOk = true; // a client reached the UI on this image - see otaRollbackTick()

    String json = "{";
    json += "\"version\":\"" + String(FW_VERSION) + "\",";
    json += "\"fsVersion\":\"" + readFsVersion() + "\",";
    json += "\"hostname\":\"" + String(wifiHostName) + "\",";
    json += "\"chipModel\":\"" + String(ESP.getChipModel()) + "\",";
    json += "\"chipRevision\":\"" + String(ESP.getChipRevision()) + "\",";
    json += "\"freeHeap\":\"" + String(ESP.getFreeHeap()) + "\",";
    json += "\"flashSize\":\"" + String(ESP.getFlashChipSize() / 1024) + " KB\",";
    if (running != NULL) {
      json += "\"partition\":\"" + String(running->label) + "\",";
      json += "\"appVersion\":\"" + String(app_info.version) + "\",";
      json += "\"appDate\":\"" + String(app_info.date) + "\",";
      json += "\"appTime\":\"" + String(app_info.time) + "\"";
    }
    json += "}";
    request->send(200, "application/json", json);
  });

  // Filesystem verification: remount LittleFS and report the web UI version it
  // now contains. The guided OTA calls this after the filesystem upload and
  // only proceeds to the firmware step if the version matches the release.
  webServer.on("/ota/fsinfo", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (otaUpdateInProgress) {
      request->send(409, "application/json", "{\"ok\":false,\"error\":\"update in progress\"}");
      return;
    }
    LittleFS.end();
    bool mounted = LittleFS.begin(false);
    String fsVer = mounted ? readFsVersion() : "--";
    bool hasIndex = mounted && LittleFS.exists("/index.html") && LittleFS.exists("/app.js");
    String json = "{";
    json += "\"ok\":" + String((mounted && hasIndex) ? "true" : "false") + ",";
    json += "\"mounted\":" + String(mounted ? "true" : "false") + ",";
    json += "\"fsVersion\":\"" + fsVer + "\",";
    json += "\"fwVersion\":\"" + String(FW_VERSION) + "\"";
    json += "}";
    request->send(200, "application/json", json);
  });

  // Health check endpoint
  webServer.on("/ota/health", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "OK");
  });

  // SAFETY-CRITICAL: Safety check endpoint
  webServer.on("/ota/check", HTTP_GET, [](AsyncWebServerRequest *request) {
    // Status only - never force the mode just because the page polled us.
    bool safe = isSystemSafeForOTA(false);
    String json = "{";
    json += "\"allowed\":" + String(safe ? "true" : "false") + ",";
    json += "\"speed\":" + String(received_vehicle_speed) + ",";
    json += "\"canInitialized\":" + String((hasCANChassis || isStandalone) && (hasCANHaldex || isStandalone) ? "true" : "false") + ",";
    json += "\"busFailure\":" + String(isBusFailure ? "true" : "false") + ",";
    json += "\"controllerDisabled\":" + String(disableController ? "true" : "false") + ",";
    json += "\"mode\":\"" + String(get_openhaldex_mode_string(state.mode)) + "\"";

    if (!safe) {
      json += ",\"reason\":\"";
      if (received_vehicle_speed > 0) json += "Vehicle moving. ";
      if (!hasCANChassis && !isStandalone) json += "Chassis CAN not initialized. ";
      if (!hasCANHaldex) json += "Haldex CAN not initialized. ";
      if (isBusFailure) json += "CAN bus failure. ";
      if (!disableController && state.mode != MODE_STOCK) json += "Controller active. ";
      json += "\"";
    } else {
      json += ",\"reason\":\"System safe for OTA update\"";
    }

    json += "}";
    request->send(200, "application/json", json);
  });

  // SAFETY-CRITICAL: Filesystem (web UI) update endpoint (no auth).
  // NOTE: must be registered BEFORE "/ota/update" - AsyncWebServer matches a
  // handler on "<uri>/..." prefixes too, so the firmware handler would otherwise
  // swallow filesystem uploads (same rule as /api/wifi/ssid in _API.cpp).
  webServer.on(
    "/ota/update/fs", HTTP_POST,
    [](AsyncWebServerRequest *request) {
      if (!isSystemSafeForOTA()) {
        request->send(403, "application/json", "{\"error\":\"OTA BLOCKED: System not in safe state\"}");
        return;
      }
      request->send(200, "text/plain", "Ready for upload");
    },
    handleFSUpdate);

  // SAFETY-CRITICAL: OTA update endpoint (no auth - see note above)
  webServer.on(
    "/ota/update", HTTP_POST,
    [](AsyncWebServerRequest *request) {
      // No password: the safety gate (stationary, CAN healthy) and the SHA-256
      // check are the controls. Runs after the upload completes.
      // SAFETY CHECK: Block if system not safe
      if (!isSystemSafeForOTA()) {
        request->send(403, "application/json", "{\"error\":\"OTA BLOCKED: System not in safe state\"}");
        return;
      }

      request->send(200, "text/plain", "Ready for upload");
    },
    handleOTAUpdate);

  // Legacy endpoint for AsyncElegantOTA compatibility (redirects to new endpoint)
  webServer.on("/update", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!request->authenticate("admin", OTA_PASSWORD)) {
      return request->requestAuthentication();
    }

    // Redirect to info page with instructions
    String html = "<!DOCTYPE html><html><head><title>OTA Update</title></head><body>";
    html += "<h1>OTA Firmware Update</h1>";
    html += "<p>Use the /ota/update endpoint to upload firmware.</p>";
    html += "<p>Current version: " + String(FW_VERSION) + "</p>";

    bool safe = isSystemSafeForOTA(false);
    html += "<p>System status: " + String(safe ? "<span style='color:green'>SAFE</span>" : "<span style='color:red'>NOT SAFE</span>") + "</p>";

    if (!safe) {
      html += "<p style='color:red'><strong>OTA BLOCKED: System not in safe state</strong></p>";
    }

    html += "<form method='POST' action='/ota/update' enctype='multipart/form-data'>";
    html += "<input type='file' name='firmware' accept='.bin'><br><br>";
    html += "<input type='submit' value='Upload Firmware' " + String(safe ? "" : "disabled") + ">";
    html += "</form>";
    html += "</body></html>";

    request->send(200, "text/html", html);
  });
  
  //otaServer->begin();

#if enableDebug || detailedDebugWiFi
  DEBUG("[OTA] OTA server started successfully!");
  DEBUG("[OTA] Update URL: http://192.168.1.1/ota/update");
  DEBUG("[OTA] Version: %s", FW_VERSION);
  DEBUG("[OTA SAFETY] OTA updates require system to be in safe state");
#endif
}

// ============================================================================
// Deferred rollback confirmation
// ============================================================================
// The Arduino core auto-confirms a pending OTA image inside initArduino()
// unless verifyRollbackLater() returns true. We defer so a build that
// crash-loops before proving itself is reverted by the bootloader on the next
// reset. The image is confirmed (from loop(), every ~100ms) as soon as EITHER:
//   - a client has fetched /ota/info (WiFi + LittleFS + HTTP all alive), or
//   - OTA_CONFIRM_UPTIME_MS of uptime (the image runs; a crash-loop never gets here).
// Deliberately independent of CAN state: a wiring/bus fault is not the new
// image's fault and must not silently revert an update on the next power cycle.
// ============================================================================
// The core's weak symbol lives in a C file (esp32-hal-misc.c), so the override
// must have C linkage or it would be name-mangled and silently ignored.
extern "C" bool verifyRollbackLater() {
  return true;
}

void otaRollbackTick() {
  if (!rollbackPending) return;
  bool confirm = webServedOk || (millis() >= OTA_CONFIRM_UPTIME_MS);
  if (!confirm) return;
  esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
  if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) { // INVALID_STATE = already confirmed
    firmwareConfirmed = true;
    rollbackPending = false;
#if enableDebug || detailedDebugWiFi
    DEBUG("[OTA SAFETY] Firmware confirmed as valid (%s)", webServedOk ? "web UI reached" : "clean uptime");
#endif
  } else {
#if enableDebug
    DEBUG("[OTA SAFETY] Failed to confirm firmware: %s", esp_err_to_name(err));
#endif
    rollbackPending = false; // don't spam
  }
}

// ============================================================================
// Check if OTA update is in progress
// ============================================================================
bool isOTAUpdateInProgress() {
  return otaUpdateInProgress;
}

// ============================================================================
// Get current firmware version
// ============================================================================
String getFirmwareVersion() {
  return String(FW_VERSION);
}



