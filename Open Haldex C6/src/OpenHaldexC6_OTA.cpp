#include <OpenHaldexC6_OTA.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "esp_ota_ops.h"

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
// ============================================================================
bool isSystemSafeForOTA() {
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
  // If controller is disabled, we're safe. Otherwise, force STOCK.
  if (!disableController) {
    if (state.mode != MODE_STOCK) {
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
// SAFETY-CRITICAL: Confirm firmware validity after successful boot
// ============================================================================
// This function MUST be called in setup() AFTER:
// 1. CAN buses are initialized
// 2. Outputs are set to safe state
// 3. No faults are detected
//
// If this is not called, ESP-IDF will automatically rollback on next boot
// ============================================================================
void confirmFirmwareValidity() {
  // SAFETY CHECK: Only confirm if system is in safe state
  if (!isSystemSafeForOTA()) {
#if enableDebug
    DEBUG("[OTA SAFETY] System not safe - firmware confirmation BLOCKED");
    DEBUG("[OTA SAFETY] Rollback will occur on next boot");
#endif
    return;
  }

  // SAFETY CHECK: Verify CAN buses are initialized
  // Check if CAN buses have received messages (indicates initialization)
  // In standalone mode, chassis CAN may not be present, so we check accordingly
  if (!isStandalone && !hasCANChassis) {
#if enableDebug
    DEBUG("[OTA SAFETY] CAN buses not initialized - firmware confirmation BLOCKED");
#endif
    return;
  }

  // SAFETY CHECK: Verify no active faults
  if (isBusFailure) {
#if enableDebug
    DEBUG("[OTA SAFETY] CAN bus failure detected - firmware confirmation BLOCKED");
#endif
    return;
  }

  // All safety checks passed - mark firmware as valid
  esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
  if (err == ESP_OK) {
    firmwareConfirmed = true;
#if enableDebug || detailedDebugWiFi
    DEBUG("[OTA SAFETY] Firmware confirmed as valid");
#endif
  } else {
#if enableDebug
    DEBUG("[OTA SAFETY] Failed to confirm firmware: %s", esp_err_to_name(err));
#endif
  }
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
  }

  // Write data chunk
  esp_err_t err = esp_ota_write(otaHandle, data, len);
  if (err != ESP_OK) {
    request->send(500, "text/plain", "OTA ERROR: Write failed");
    esp_ota_abort(otaHandle);
    otaUpdateInProgress = false;
    return;
  }

  // Final chunk - finish OTA
  if (final) {
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
      strip.setLedColorData(led_channel, led_brightness/2, led_brightness/2, led_brightness/2);  // red
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
// Setup OTA Server
// ============================================================================
void setupOTA() {
#if detailedDebugWiFi
  DEBUG("[OTA] Setting up OTA update server...");
#endif

  // Check if firmware needs confirmation
  if (needsFirmwareConfirmation()) {
#if enableDebug
    DEBUG("[OTA SAFETY] New firmware detected - will confirm after safety checks");
#endif
    // Don't confirm yet - wait for confirmFirmwareValidity() to be called
  }

  // Create OTA server
  otaServer = new AsyncWebServer(81);

  // Info endpoint
  otaServer->on("/ota/info", HTTP_GET, [](AsyncWebServerRequest *request) {
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_app_desc_t app_info;

    if (running != NULL) {
      esp_ota_get_partition_description(running, &app_info);
    }

    String json = "{";
    json += "\"version\":\"" + String(FW_VERSION) + "\",";
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

  // Health check endpoint
  otaServer->on("/ota/health", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "OK");
  });

  // SAFETY-CRITICAL: Safety check endpoint
  otaServer->on("/ota/check", HTTP_GET, [](AsyncWebServerRequest *request) {
    bool safe = isSystemSafeForOTA();
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

  // SAFETY-CRITICAL: OTA update endpoint with authentication
  otaServer->on(
    "/ota/update", HTTP_POST,
    [](AsyncWebServerRequest *request) {
      // Check authentication
      if (!request->authenticate("admin", OTA_PASSWORD)) {
        return request->requestAuthentication();
      }

      // SAFETY CHECK: Block if system not safe
      if (!isSystemSafeForOTA()) {
        request->send(403, "application/json", "{\"error\":\"OTA BLOCKED: System not in safe state\"}");
        return;
      }

      request->send(200, "text/plain", "Ready for upload");
    },
    handleOTAUpdate);

  // Legacy endpoint for AsyncElegantOTA compatibility (redirects to new endpoint)
  otaServer->on("/update", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!request->authenticate("admin", OTA_PASSWORD)) {
      return request->requestAuthentication();
    }

    // Redirect to info page with instructions
    String html = "<!DOCTYPE html><html><head><title>OTA Update</title></head><body>";
    html += "<h1>OTA Firmware Update</h1>";
    html += "<p>Use the /ota/update endpoint to upload firmware.</p>";
    html += "<p>Current version: " + String(FW_VERSION) + "</p>";

    bool safe = isSystemSafeForOTA();
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
  
  otaServer->begin();

#if enableDebug || detailedDebugWiFi
  DEBUG("[OTA] OTA server started successfully!");
  DEBUG("[OTA] Update URL: http://192.168.1.1:81/ota/update");
  DEBUG("[OTA] Username: admin");
  DEBUG("[OTA] Password: %s", OTA_PASSWORD);
  DEBUG("[OTA] Version: %s", FW_VERSION);
  DEBUG("[OTA SAFETY] OTA updates require system to be in safe state");
#endif
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



