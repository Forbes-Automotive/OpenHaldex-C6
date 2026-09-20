#include <OpenHaldexC6_OTA.h>
#include <Update.h>
#include <LittleFS.h>
#include <mbedtls/sha256.h>
extern "C" {
#include <esp_littlefs.h> // esp_littlefs_mounted()
}

// Filesystem / upload diagnostics: on with the WiFi debug flags, silent otherwise
#if enableDebug || detailedDebugWiFi
#define OTA_DEBUG(...) DEBUG(__VA_ARGS__)
#else
#define OTA_DEBUG(...)
#endif

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

// Last time any polled UI endpoint was hit (see otaWebClientActive). The
// dashboard polls every 500 ms and the OTA page every 3 s, so 30 s of silence
// means the browser really has gone away, not just paused between polls.
static volatile uint32_t lastWebActivityMs = 0;
#define OTA_WEB_ACTIVE_WINDOW_MS 30000UL

void otaNoteWebActivity() {
  lastWebActivityMs = millis();
}

bool otaWebClientActive() {
  if (otaUpdateInProgress) return true;
  return lastWebActivityMs != 0 && (millis() - lastWebActivityMs) < OTA_WEB_ACTIVE_WINDOW_MS;
}

// No integrity hash on the upload stream: the chip validates a firmware
// image itself (esp_ota_end) and a filesystem image is validated by mounting
// it. A SHA-256 gate against the release index was tried and dropped - a
// rebuilt .bin with a stale index failed every update for no real gain.

// Diagnostic: SHA-256 of the first `len` bytes of a partition as read back
// from flash (what actually got written, not what was received). 720 kB
// takes ~100 ms.
static String partitionSha256(const esp_partition_t *p, size_t len) {
  if (!p) return "";
  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts(&ctx, 0);
  static uint8_t buf[1024];
  for (size_t off = 0; off < len; off += sizeof(buf)) {
    size_t n = min(sizeof(buf), len - off);
    if (esp_partition_read(p, off, buf, n) != ESP_OK) { mbedtls_sha256_free(&ctx); return ""; }
    mbedtls_sha256_update(&ctx, buf, n);
  }
  uint8_t digest[32];
  mbedtls_sha256_finish(&ctx, digest);
  mbedtls_sha256_free(&ctx);
  char hex[65];
  for (int i = 0; i < 32; ++i) sprintf(hex + i * 2, "%02x", digest[i]);
  hex[64] = '\0';
  return String(hex);
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
// Web UI filesystem guards
// ============================================================================
// LittleFS is built with CONFIG_LITTLEFS_ASSERTS=y, and the panic handler
// reboots. Feed lfs a partition that is half one image and half another (a
// filesystem OTA that stopped part-way, or was rejected after the partition
// had already been rewritten) and it can trip an assert while walking the
// directory tree - at boot, every boot: a boot loop with no web server to
// recover from. So:
//   - the partition is only ever handed to lfs after a look at the superblock
//     pair (the "littlefs" magic and a geometry that fits this partition);
//   - a filesystem upload that fails for any reason erases that superblock
//     pair, so what's left can never be mistaken for a filesystem;
//   - the web server always starts; with no usable filesystem "/" serves a
//     built-in recovery page (see setupWebServer) with the two upload forms.
// ============================================================================
static const esp_partition_t *fsPartition() {
  return esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, NULL);
}

static String hex32(const uint8_t *b) {
  char s[65];
  for (int i = 0; i < 32; i++) sprintf(s + i * 2, "%02x", b[i]);
  s[64] = '\0';
  return String(s);
}

// LittleFS superblock: revision(4) tag(4) "littlefs"(8) tag(4) version(4)
// block_size(4) block_count(4) ... - the pair lives in blocks 0 and 1 and
// either one may be the current copy.
static bool fsSuperblockLooksValid(const esp_partition_t *p) {
  if (!p) return false;
  const uint32_t blockSize = 4096;
  for (int b = 0; b < 2; b++) {
    uint8_t head[32];
    if (esp_partition_read(p, (size_t)b * blockSize, head, sizeof(head)) != ESP_OK) continue;
    uint32_t bs, bc;
    memcpy(&bs, head + 24, 4);
    memcpy(&bc, head + 28, 4);
    OTA_DEBUG("[FS] block %d: %s | magic %s block_size %lu block_count %lu (partition %lu bytes)", b, hex32(head).c_str(),
          memcmp(head + 8, "littlefs", 8) == 0 ? "ok" : "MISSING", (unsigned long)bs, (unsigned long)bc, (unsigned long)p->size);
    if (memcmp(head + 8, "littlefs", 8) != 0) continue;
    if (bs == blockSize && bc > 0 && (uint64_t)bc * bs <= p->size) return true;
  }
  return false;
}

bool fsMountSafe() {
  const esp_partition_t *p = fsPartition();
  if (!p) {
    OTA_DEBUG("[FS] no data/spiffs partition in the table");
    return false;
  }
  if (fsMounted()) return true;
  if (!fsSuperblockLooksValid(p)) {
    OTA_DEBUG("[FS] No LittleFS superblock on '%s' - not mounting (upload littlefs.bin from the recovery page)", p->label);
    return false;
  }
  bool ok = LittleFS.begin(false);
  OTA_DEBUG("[FS] LittleFS.begin -> %s; index.html %s, app.js %s, version.json %s", ok ? "mounted" : "FAILED",
        ok && LittleFS.exists("/index.html") ? "present" : "missing",
        ok && LittleFS.exists("/app.js") ? "present" : "missing",
        ok && LittleFS.exists("/version.json") ? "present" : "missing");
  return ok;
}

bool fsMounted() {
  return esp_littlefs_mounted("spiffs");
}

bool fsUiAvailable() {
  return fsMounted() && LittleFS.exists("/index.html") && LittleFS.exists("/app.js");
}

void fsInvalidate() {
  LittleFS.end();
  const esp_partition_t *p = fsPartition();
  if (p) esp_partition_erase_range(p, 0, 2 * 4096);
#if enableDebug || detailedDebugWiFi
  DEBUG("[FS] Superblock pair erased - filesystem partition marked empty");
#endif
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
// Upload results
// ============================================================================
// The upload callbacks below NEVER call request->send(). The body is still
// streaming in while they run; a response written mid-body makes the browser
// treat the upload as finished and drop the connection under the parser -
// AsyncTCP then touches the freed pcb (Guru Meditation in tcp_output) and a
// half-written partition is left behind. The old handlers answered "200 OK"
// on every chunk, so no upload through this UI ever completed. Outcome is
// recorded here and sent by the request handler once the body has ended;
// the first outcome recorded wins, later chunks are just drained.
// ============================================================================
struct OtaResult {
  bool set = false;
  int code = 0;
  String msg;
};
static OtaResult fwResult, fsResult;
static bool fwRebootPending = false;

static void otaSetResult(OtaResult &r, int code, const String &msg) {
  if (r.set) return;
  r.set = true;
  r.code = code;
  r.msg = msg;
  OTA_DEBUG("[OTA] result %d: %s", code, msg.c_str());
}

static void otaSendResult(AsyncWebServerRequest *request, OtaResult &r, const char *noFileMsg) {
  if (!r.set) { r.code = 400; r.msg = noFileMsg; }
  request->send(r.code, "text/plain", r.msg);
  r.set = false;
}

// ============================================================================
// OTA Update Handler - SAFETY-CRITICAL: Blocks unsafe updates
// ============================================================================
void handleOTAUpdate(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  otaNoteWebActivity();
  static bool fwStarted = false;
  if (index == 0) { fwResult.set = false; fwStarted = false; }
  if (fwResult.set) return; // outcome already decided - drain the rest of the body

  // SAFETY CHECK: Block update if system is not safe
  if (!isSystemSafeForOTA()) {
    if (fwStarted) { esp_ota_abort(otaHandle); otaUpdateInProgress = false; fwStarted = false; }
    otaSetResult(fwResult, 403, "OTA BLOCKED: System not in safe state. Vehicle must be stationary, CAN initialized, outputs safe, no faults.");
    return;
  }

  // First chunk - initialize OTA
  if (index == 0) {
    otaPartition = esp_ota_get_next_update_partition(NULL);
    if (otaPartition == NULL) {
      otaSetResult(fwResult, 500, "OTA ERROR: No OTA partition found. Check partition table.");
      return;
    }
    OTA_DEBUG("[OTA] Starting firmware update: %s -> %s", filename.c_str(), otaPartition->label);
    esp_err_t err = esp_ota_begin(otaPartition, OTA_SIZE_UNKNOWN, &otaHandle);
    if (err != ESP_OK) {
      otaSetResult(fwResult, 500, "OTA ERROR: Failed to begin update");
      return;
    }
    fwStarted = true;
    otaUpdateInProgress = true;
  }

  // Write data chunk
  if (len && esp_ota_write(otaHandle, data, len) != ESP_OK) {
    esp_ota_abort(otaHandle);
    otaUpdateInProgress = false;
    fwStarted = false;
    otaSetResult(fwResult, 500, "OTA ERROR: Write failed");
    return;
  }

  // Final chunk - finish OTA (esp_ota_end validates the image before it can boot)
  if (final) {
    fwStarted = false;
    esp_err_t err = esp_ota_end(otaHandle);
    if (err != ESP_OK) {
      esp_ota_abort(otaHandle);
      otaUpdateInProgress = false;
      otaSetResult(fwResult, err == ESP_ERR_OTA_VALIDATE_FAILED ? 400 : 500,
                   err == ESP_ERR_OTA_VALIDATE_FAILED ? "OTA ERROR: Image validation failed - is that a firmware.bin?" : "OTA ERROR: End failed");
      return;
    }
    if (esp_ota_set_boot_partition(otaPartition) != ESP_OK) {
      otaUpdateInProgress = false;
      otaSetResult(fwResult, 500, "OTA ERROR: Failed to set boot partition");
      return;
    }
    OTA_DEBUG("[OTA] Firmware written to %s. Rebooting once the response is out; the new image must confirm itself.", otaPartition->label);
    otaSetResult(fwResult, 200, "OTA update complete. Rebooting... Firmware will be confirmed after safety checks pass.");
    fwRebootPending = true; // otaUpdateInProgress stays set until the reboot
  }
}

// Runs after the body: send the outcome, then reboot if a firmware image was
// just installed.
static void finishFirmwareRequest(AsyncWebServerRequest *request) {
  otaSendResult(request, fwResult, "No file received - pick firmware.bin first.");
  if (!fwRebootPending) return;
  delay(1000); // let the response leave
  for (int i = 0; i <= 8; i++) {
    strip.setLedColorData(led_channel, ledBrightness / 2, ledBrightness / 2, ledBrightness / 2);
    strip.show();
    delay(50);
    strip.setLedColorData(led_channel, 0, 0, 0);
    strip.show();
    delay(50);
  }
  ESP.restart();
}

// ============================================================================
// Filesystem (LittleFS) Update Handler - writes littlefs.bin to the "spiffs"
// data partition via the Arduino Update library (U_SPIFFS). Same safety gate
// as the firmware path. Used by the OTA page "Filesystem (web UI)" option.
//
// The firmware keeps running throughout (it lives in ota_0/ota_1, not here)
// and there is NO reboot at the end. LittleFS is unmounted before the first
// byte is written - it used to stay mounted while the partition was rewritten
// underneath it, so any page load during or after the upload walked the old
// directory tree over new blocks. Any failure (write error, client gone,
// short upload, SHA mismatch, image that won't mount) ends with
// fsInvalidate(), leaving a partition that cannot be mounted rather than a
// hybrid of two images; the web server then serves the recovery page.
// ============================================================================
static size_t fsBytesReceived = 0;
static uint8_t fsHeadRx[32]; // first bytes as received - compared with the flash read-back on failure
static size_t fsHeadRxLen = 0;

// `wipe` = false only when nothing has been written yet (the old image is
// intact, so put it back rather than erase it).
static void fsUpdateFail(int code, const String &msg, bool wipe = true) {
  Update.abort();
  if (wipe) fsInvalidate(); else fsMountSafe();
  otaUpdateInProgress = false;
  otaSetResult(fsResult, code, msg);
}

void handleFSUpdate(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  otaNoteWebActivity();
  if (index == 0) { fsResult.set = false; fsBytesReceived = 0; fsHeadRxLen = 0; }
  if (fsResult.set) return; // outcome already decided - drain the rest of the body

  // SAFETY CHECK: Block update if system is not safe
  if (!isSystemSafeForOTA()) {
    if (index == 0) otaSetResult(fsResult, 403, "OTA BLOCKED: System not in safe state."); // nothing written
    else fsUpdateFail(403, "OTA BLOCKED: system left the safe state mid-upload - re-upload the filesystem");
    return;
  }

  // First chunk - begin filesystem update
  if (index == 0) {
    if (Update.isRunning()) Update.abort(); // an earlier upload that never finished
    otaUpdateInProgress = true;
    OTA_DEBUG("[OTA] Starting filesystem update: %s (%s bytes expected)", filename.c_str(),
          request->hasParam("size") ? request->getParam("size")->value().c_str() : "?");
    LittleFS.end(); // nothing may read the partition while it is being rewritten
    if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_SPIFFS)) {
      fsUpdateFail(500, "OTA ERROR: Failed to begin filesystem update", false);
      return;
    }
    // Phone wandered off mid-upload: don't leave half an image behind.
    request->onDisconnect([]() {
      if (otaUpdateInProgress && Update.isRunning()) fsUpdateFail(0, "client disconnected mid-upload");
    });
  }

  // Write data chunk
  if (fsHeadRxLen < sizeof(fsHeadRx) && len) {
    size_t n = min(len, sizeof(fsHeadRx) - fsHeadRxLen);
    memcpy(fsHeadRx + fsHeadRxLen, data, n);
    fsHeadRxLen += n;
  }
  fsBytesReceived += len;
  if (len && Update.write(data, len) != len) {
    fsUpdateFail(500, "OTA ERROR: Filesystem write failed");
    return;
  }

  // Final chunk - finish filesystem update
  if (final) {
    // Expected length, when the uploader says (?size=): a short body still
    // arrives with final=true, and Update.end(true) would happily accept it.
    size_t expect = 0;
    if (request->hasParam("size")) expect = (size_t)request->getParam("size")->value().toInt();
    if (expect && fsBytesReceived != expect) {
      fsUpdateFail(400, "OTA ERROR: Filesystem upload was short (" + String(fsBytesReceived) + " of " + String(expect) + " bytes) - re-upload the filesystem");
      return;
    }
    if (!Update.end(true)) {
      fsUpdateFail(500, "OTA ERROR: Filesystem update failed");
      return;
    }
    // Mount what was written. If it doesn't come up as a filesystem holding
    // the web UI there's no point keeping it. The read-back in the message
    // lets the uploader compare against the file it sent.
    if (!fsMountSafe() || !fsUiAvailable()) {
      uint8_t flashHead[32] = {0};
      esp_partition_read(fsPartition(), 0, flashHead, sizeof(flashHead));
      String msg = "OTA ERROR: Filesystem written but does not mount (";
      msg += fsMounted() ? "mounted, web UI files missing" : "mount failed";
      msg += "). Received " + String(fsBytesReceived) + " bytes. First 32 bytes received: " + hex32(fsHeadRx) +
             " | in flash: " + hex32(flashHead) + " | read-back sha256: " + partitionSha256(fsPartition(), fsBytesReceived);
      fsUpdateFail(500, msg);
      return;
    }

    OTA_DEBUG("[OTA] Filesystem update complete: %u bytes, web UI v%s. No reboot (upload firmware next).", (unsigned)fsBytesReceived, readFsVersion().c_str());

    // Filesystem does NOT reboot: the two-step OTA flow uploads the filesystem
    // first (step 1), then the firmware (step 2) reboots at the end.
    otaUpdateInProgress = false;
    otaSetResult(fsResult, 200, "Filesystem update complete.");

    // brief green confirmation flash
    for (int i = 0; i <= 4; i++) {
      strip.setLedColorData(led_channel, 0, ledBrightness / 2, 0);
      strip.show();
      delay(40);
      strip.setLedColorData(led_channel, 0, 0, 0);
      strip.show();
      delay(40);
    }
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
    otaNoteWebActivity();

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
    otaNoteWebActivity();
    if (otaUpdateInProgress) {
      request->send(409, "application/json", "{\"ok\":false,\"error\":\"update in progress\"}");
      return;
    }
    // The upload handler already remounted; only try again if it isn't up.
    bool mounted = fsMounted() || fsMountSafe();
    String fsVer = mounted ? readFsVersion() : "--";
    bool hasIndex = mounted && fsUiAvailable();
    String json = "{";
    json += "\"ok\":" + String((mounted && hasIndex) ? "true" : "false") + ",";
    json += "\"mounted\":" + String(mounted ? "true" : "false") + ",";
    json += "\"fsVersion\":\"" + fsVer + "\",";
    json += "\"fwVersion\":\"" + String(FW_VERSION) + "\"";
    json += "}";
    request->send(200, "application/json", json);
  });

  // Diagnostic: what is physically in the filesystem partition. ?len=N hashes
  // the first N bytes (default: whole partition) so it can be compared with
  // `sha256sum littlefs.bin` on the PC. Also on the recovery page.
  webServer.on("/ota/fsdiag", HTTP_GET, [](AsyncWebServerRequest *request) {
    otaNoteWebActivity();
    const esp_partition_t *p = fsPartition();
    if (!p) { request->send(500, "application/json", "{\"error\":\"no spiffs partition\"}"); return; }
    size_t len = p->size;
    if (request->hasParam("len")) { long l = request->getParam("len")->value().toInt(); if (l > 0 && (size_t)l <= p->size) len = (size_t)l; }
    uint8_t head[32];
    String hex = "";
    if (esp_partition_read(p, 0, head, sizeof(head)) == ESP_OK) {
      char h[3];
      for (int i = 0; i < 32; i++) { sprintf(h, "%02x", head[i]); hex += h; }
    }
    uint32_t bs = 0, bc = 0;
    memcpy(&bs, head + 24, 4);
    memcpy(&bc, head + 28, 4);
    String json = "{";
    json += "\"partition\":\"" + String(p->label) + "\",\"address\":" + String(p->address) + ",\"size\":" + String(p->size) + ",";
    json += "\"head\":\"" + hex + "\",\"magic\":" + String(memcmp(head + 8, "littlefs", 8) == 0 ? "true" : "false") + ",";
    json += "\"blockSize\":" + String(bs) + ",\"blockCount\":" + String(bc) + ",";
    json += "\"mounted\":" + String(fsMounted() ? "true" : "false") + ",";
    json += "\"ui\":" + String(fsUiAvailable() ? "true" : "false") + ",";
    json += "\"fsVersion\":\"" + (fsMounted() ? readFsVersion() : String("--")) + "\",";
    json += "\"hashLen\":" + String(len) + ",\"sha256\":\"" + partitionSha256(p, len) + "\"";
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
    otaNoteWebActivity();
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
  // The request lambdas run once the whole body has been parsed; that is the
  // only place a response may be sent (see "Upload results" above).
  webServer.on(
    "/ota/update/fs", HTTP_POST,
    [](AsyncWebServerRequest *request) { otaSendResult(request, fsResult, "No file received - pick littlefs.bin first."); },
    handleFSUpdate);

  // SAFETY-CRITICAL: OTA update endpoint (no auth: the safety gate - stationary,
  // CAN healthy - is the control, and the chip validates the image itself)
  webServer.on(
    "/ota/update", HTTP_POST,
    finishFirmwareRequest,
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



