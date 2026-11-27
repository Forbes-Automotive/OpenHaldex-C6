#include <OpenHaldexC6_API.h>
#include <OpenHaldexC6_UDS.h>
#include <OpenHaldexC6_Calculations.h>
#include <OpenHaldexC6_WiFi.h>
#include <OpenHaldexC6_OTA.h> // otaNoteWebActivity()
#include <OpenHaldexC6_ESPNow.h> // gauges over ESP-NOW settings
#include <OpenHaldexC6_BLE.h> // bleIsConnected(), bleForgetBonds()
#include <OpenHaldexC6_Settings.h> // applyDrivingSetting()

#include <cstring>
#include <vector>    // /api/wifi/scan de-dup
#include <algorithm> // std::sort

#define BACKUP_LAYOUT 1 // /api/backup format (V11.00.14); sent as backupLayout in /api/settings

// ---- Defines -------------------------------------------------------------------

#define JSON_BODY_MAX 4096 // largest POST body we accept (the biggest real one, /api/tune, is ~1 KB)

// ---- Statics -------------------------------------------------------------------

// Served at "/" when the web UI filesystem is missing, broken or empty (a
// filesystem OTA that failed, a fresh chip with only firmware on it). Needs
// nothing from LittleFS: two uploads straight to the OTA endpoints, web UI
// first. Deliberately plain - it has to work from any phone browser.
static const char RECOVERY_HTML[] PROGMEM = R"HTML(<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>OpenHaldex-C6 recovery</title><style>body{font-family:sans-serif;background:#111;color:#eee;margin:0;padding:16px;max-width:520px}
h1{font-size:20px}p{line-height:1.5;color:#bbb}code{color:#fff}section{border:1px solid #333;border-radius:10px;padding:14px;margin:14px 0}
input[type=file]{display:block;margin:10px 0;max-width:100%}button{background:#2a6df4;color:#fff;border:0;border-radius:8px;padding:10px 16px;font-size:15px}
button:disabled{opacity:.5}.s{margin-top:8px;font-size:14px;color:#9c9}.e{color:#f77}.bar{height:6px;background:#333;border-radius:3px;margin-top:8px}.bar i{display:block;height:100%;width:0;background:#2a6df4;border-radius:3px}</style></head>
<body><h1>OpenHaldex-C6 &middot; web UI missing</h1>
<p>The controller is running (firmware <code>%FW%</code>) but its web-interface partition holds no usable filesystem - usually a filesystem update that stopped part-way. Nothing else is affected. Upload the two files from the release folder on GitHub (<code>Releases/V&hellip;/</code>): the web UI first, then the firmware if you were mid-update.</p>
<section><strong>1. Web UI</strong> &mdash; <code>littlefs.bin</code><input type="file" id="fs" accept=".bin"><button id="fsb">Upload web UI</button><div class="bar"><i id="fsp"></i></div><div class="s" id="fss"></div></section>
<section><strong>2. Firmware</strong> &mdash; <code>firmware.bin</code> (optional; reboots when done)<input type="file" id="fw" accept=".bin"><button id="fwb">Upload firmware</button><div class="bar"><i id="fwp"></i></div><div class="s" id="fws"></div></section>
<section><strong>Partition diagnostics</strong> <button id="dgb" style="float:right;padding:6px 10px;font-size:13px">Refresh</button><pre id="dg" style="white-space:pre-wrap;word-break:break-all;font-size:12px;color:#bbb;margin:10px 0 0">loading&hellip;</pre></section>
<script>
function diag(){var x=new XMLHttpRequest();x.open('GET','/ota/fsdiag');x.onload=function(){try{var d=JSON.parse(x.responseText),o='';for(var k in d)o+=k+': '+d[k]+'\n';document.getElementById('dg').textContent=o}catch(e){document.getElementById('dg').textContent=x.responseText}};x.send()}
document.getElementById('dgb').onclick=diag;diag();
function up(k,url,field,done,then){var f=document.getElementById(k).files[0],b=document.getElementById(k+'b'),s=document.getElementById(k+'s'),p=document.getElementById(k+'p');
if(!f){s.textContent='Pick the file first.';s.className='s e';return}b.disabled=true;s.className='s';s.textContent='Uploading…';
var d=new FormData();d.append(field,f,f.name);var x=new XMLHttpRequest();x.open('POST',url+'?size='+f.size);
x.upload.onprogress=function(e){if(e.lengthComputable)p.style.width=Math.round(e.loaded/e.total*100)+'%'};
x.onload=function(){if(x.status===200){s.textContent=done;if(then)then()}else{s.className='s e';s.textContent=x.responseText||('Failed ('+x.status+')');b.disabled=false}};
x.onerror=function(){s.className='s e';s.textContent='Upload failed - check the connection and retry.';b.disabled=false};x.send(d)}
document.getElementById('fsb').onclick=function(){up('fs','/ota/update/fs','filesystem','Web UI installed - opening it…',function(){setTimeout(function(){location.reload()},1500)})};
document.getElementById('fwb').onclick=function(){up('fw','/ota/update','firmware','Firmware installed - rebooting. Reload this page in ~20 s.')};
</script></body></html>)HTML";

// ---- Small helpers -------------------------------------------------------------

// helper function to send JSON to the browser. V11.00.8: serialised straight into
// the response stream - building a String first and letting send() copy it held
// the reply twice (~40 KB at once for the Long Learn status).
static void sendJSON(AsyncWebServerRequest *request, int code, const JsonDocument &data)
{
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    response->setCode(code);
    serializeJson(data, *response);
    request->send(response);
}

// {"ok":false,"error":...} - every POST must answer, even when it refuses: a
// request left without a reply holds a browser connection and a socket until it
// times out (the mode buttons froze the page after ~6 taps).
static void sendError(AsyncWebServerRequest *request, int code, const char *error)
{
    JsonDocument resp;
    resp["ok"] = false;
    resp["error"] = error;
    sendJSON(request, code, resp);
}

// Add the engine coolant and oil temperatures (PQ Kombi_2) to a reply, in whole degC.
// null (shown as "--") when no Kombi_2 has come in for 2 s, or never - so MQB and
// the other makes, which don't send it, show "--".
static void addEngineTemps(JsonDocument &data)
{
    uint32_t lastSeenMs = lastKombi2Ms; // copy first, then read the clock (the CAN task can change it)
    bool isKombi2Fresh = false;
    if (lastSeenMs != 0 && (millis() - lastSeenMs) <= KOMBI2_TIMEOUT_MS)
    {
        isKombi2Fresh = true;
    }

    if (isKombi2Fresh)
    {
        data["coolantTemp"] = (int)lroundf(receivedCoolantTemp); // lroundf = round to the nearest whole number (works below zero too)
        data["oilTemp"] = (int)lroundf(receivedOilTemp);
    }
    else
    {
        data["coolantTemp"] = nullptr;
        data["oilTemp"] = nullptr;
    }
}

// Long Learn start: {"testAll": bool}. Same pre-flight as the manual learn plus
// a check that the generation actually has gated blocks to bisect.
static void longLearnStartIncoming(AsyncWebServerRequest *request, const String &body)
{
    JsonDocument in;
    bool testAll = false;
    if (deserializeJson(in, body) == DeserializationError::Ok && in["testAll"].is<bool>())
        testAll = in["testAll"].as<bool>();

    JsonDocument resp;
    const char *why = nullptr;
    if (!hasCANHaldex)
    {
        resp["ok"] = false;
        resp["error"] = "No Haldex CAN data available";
    }
    else if (!startLongLearn(testAll, &why))
    {
        resp["ok"] = false;
        resp["error"] = why ? why : "Long Learn could not start";
    }
    else
    {
        resp["ok"] = true;
    }
    sendJSON(request, 200, resp);
}

// manage mode (saved from Web, handled here): WebServer sends, this handles
static void modeIncoming(AsyncWebServerRequest *request, const String &body)
{
    JsonDocument data;
    if (deserializeJson(data, body) != DeserializationError::Ok)
    {
        DEBUG("Invalid JSON");
        sendError(request, 400, "Invalid JSON");
        return;
    }

    if (!data["mode"].is<uint8_t>())
    {
        sendError(request, 400, "No mode given");
        return;
    }
    if (!requestMode(data["mode"]))
    {
        sendError(request, 409, "Mode not changed (controller disabled, or not a valid mode)");
        return;
    }
    JsonDocument resp;
    resp["ok"] = true;
    sendJSON(request, 200, resp);
}

// ---- Setup ---------------------------------------------------------------------

// setup webserver function
void setupWebServer()
{
    // The firmware never depends on the filesystem - it only holds the web UI.
    // Mount it if it looks sane (fsMountSafe: a LittleFS superblock that fits
    // the partition, so a half-written image can't trip an lfs assert and
    // boot-loop us), and start the server either way: without a UI, "/" is
    // the recovery page and the /ota/* and /api/* endpoints all still work.
    if (fsMountSafe() && fsUiAvailable())
    {
        DEBUG("LittleFS mounted successfully");
    }
    else
    {
        DEBUG("LittleFS: no usable web UI - serving the recovery page at /");
    }

    // index.html streamed from LittleFS (chunked, low-heap) with no-cache
    // headers; it must never be cached. Decided per request, so a filesystem
    // upload from the recovery page switches straight over to the real UI.
    webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
                 {
        AsyncWebServerResponse *res;
        if (fsUiAvailable()) {
            res = request->beginResponse(LittleFS, "/index.html", "text/html");
        } else {
            String html = FPSTR(RECOVERY_HTML);
            html.replace("%FW%", FW_VERSION);
            res = request->beginResponse(200, "text/html", html);
        }
        res->addHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
        res->addHeader("Pragma", "no-cache");
        request->send(res); });

    // app.js / style.css: "no-cache" means the browser keeps a copy but asks
    // every time (If-None-Match against the ETag the handler derives from the
    // file's LittleFS mtime/size) and gets a 304 unless the file changed.
    // Previously max-age=1y with a hand-bumped ?v= in index.html - which got
    // forgotten, so phones ran a stale app.js against new HTML and buttons
    // on new cards did nothing.
    // The filter keeps the handler out of the way while nothing is mounted -
    // otherwise every /api request first asks LittleFS.exists() and logs an
    // "File system is not mounted" error.
    webServer.serveStatic("/", LittleFS, "/").setDefaultFile("index.html").setCacheControl("no-cache")
        .setFilter([](AsyncWebServerRequest *request) { return fsMounted(); });

    webServer.begin(); // begin the webServer
    DEBUG("Web server started");
    // mDNS is already started in setupWiFi(); don't re-init it here.
}
// helper function to calculate CPU usage percentage based on FreeRTOS task run time stats
static int getCPUUsagePercent() 
{
    static configRUN_TIME_COUNTER_TYPE previousTotalRuntime = 0;
    static configRUN_TIME_COUNTER_TYPE previousIdleRuntime = 0;

    const UBaseType_t taskCount = uxTaskGetNumberOfTasks();
    if (taskCount == 0)
    {
        return -1;
    }

    TaskStatus_t *taskStats = static_cast<TaskStatus_t *>(malloc(taskCount * sizeof(TaskStatus_t)));
    if (taskStats == nullptr)
    {
        return -1;
    }

    configRUN_TIME_COUNTER_TYPE totalRuntime = 0;
    const UBaseType_t collectedTasks = uxTaskGetSystemState(taskStats, taskCount, &totalRuntime);

    if ((collectedTasks == 0) || (totalRuntime == 0))
    {
        free(taskStats);
        return -1;
    }

    configRUN_TIME_COUNTER_TYPE idleRuntime = 0;
    for (UBaseType_t i = 0; i < collectedTasks; i++)
    {
        if ((taskStats[i].pcTaskName != nullptr) && (strncmp(taskStats[i].pcTaskName, "IDLE", 4) == 0))
        {
            idleRuntime += taskStats[i].ulRunTimeCounter;
        }
    }

    free(taskStats);

    if ((previousTotalRuntime == 0) || (totalRuntime <= previousTotalRuntime) || (idleRuntime < previousIdleRuntime))
    {
        previousTotalRuntime = totalRuntime;
        previousIdleRuntime = idleRuntime;
        return -1;
    }

    const configRUN_TIME_COUNTER_TYPE totalRuntimeDelta = totalRuntime - previousTotalRuntime;
    const configRUN_TIME_COUNTER_TYPE idleRuntimeDelta = idleRuntime - previousIdleRuntime;

    previousTotalRuntime = totalRuntime;
    previousIdleRuntime = idleRuntime;

    if (totalRuntimeDelta == 0)
    {
        return -1;
    }

    float idlePercent = (static_cast<float>(idleRuntimeDelta) * 100.0f) / static_cast<float>(totalRuntimeDelta);
    idlePercent = constrain(idlePercent, 0.0f, 100.0f);

    const int cpuPercent = int(100.0f - idlePercent + 0.5f);
    return constrain(cpuPercent, 0, 100);
}

// Collect a POST body, then hand it to `done`. The body is gathered in a
// malloc() buffer because the web server free()s _tempObject when a request
// ends early (phone dropped mid-upload) - a `new String` there leaked its text.
static void parseJSON(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total, void (*done)(AsyncWebServerRequest *, const String &))
{
    if (index == 0)
    {
        if (total > JSON_BODY_MAX)
        {
            sendError(request, 413, "Request too large");
            return;
        }
        request->_tempObject = malloc(total + 1);
        if (request->_tempObject == nullptr)
        {
            sendError(request, 503, "Out of memory");
            return;
        }
    }

    char *body = (char *)request->_tempObject;
    if (body == nullptr || index + len > total)
    {
        return; // refused above, or a chunk that doesn't fit
    }
    memcpy(body + index, data, len);

    if (index + len == total)
    {
        body[total] = 0;
        const String bodyString(body); // the one copy the handlers get
        free(body);
        request->_tempObject = nullptr;
        done(request, bodyString);
    }
}

// GET /api/live (V11.00.4) - only the dashboard's Live Data tiles and the lock.
// Much smaller than /api/dashboard, so the page can ask for it up to 10 times a
// second (Display Options > Live Data Refresh) without loading the controller.
static void liveOutgoing(AsyncWebServerRequest *request)
{
    JsonDocument data;

    if (hasCANChassis) // chassis CAN ok: send the car's values
    {
        data["speed"] = receivedVehicleSpeed;
        data["throttle"] = int(receivedPedalValue);
        data["rpm"] = receivedVehicleRpm;
        data["boost"] = receivedVehicleBoost;
    }
    else // chassis CAN not ok: null (shown as "--")
    {
        data["speed"] = nullptr;
        data["throttle"] = nullptr;
        data["rpm"] = nullptr;
        data["boost"] = nullptr;
    }
    addEngineTemps(data);

    data["lockTarget"] = int(lockTarget);
    data["expertPT"] = expertPtActive(); // V11.00.12: Expert is in a PT area - the car's own strategy has the Haldex
    if (hasCANHaldex) // Haldex CAN ok: what the Haldex reports back
    {
        data["lockActual"] = receivedHaldexEngagement;
    }
    else
    {
        data["lockActual"] = nullptr;
    }

    sendJSON(request, 200, data);
}

// parse the current INCOMING request for status: WebServer asks, this delivers
static void statusOutgoing(AsyncWebServerRequest *request)
{
    JsonDocument data;
    const bool chassisOk = hasCANChassis;
    const bool haldexOk = hasCANHaldex;

    data["mode"] = state.mode;

    // Ext-button force flag is driven by the external button
    data["extButtonActive"] = extButtonForceModeFlag;

    // Drive mode select (MQB Charisma): the profile the car is in, the mode its gateway
    // asks the Haldex / VAQ for, and the OpenHaldex mode that profile is forcing (-1 = none).
    {
        JsonObject driveMode = data["driveMode"].to<JsonObject>(); // nested object: "driveMode": { ... }
        uint32_t lastReadMs = lastDriveProfileMs;                  // copy first, then read the clock (see getDriveModeForcedValue)
        uint32_t nowMs = millis();
        bool isDriveProfileFresh = false;
        if (lastReadMs != 0 && (nowMs - lastReadMs) <= DRIVE_MODE_TIMEOUT_MS)
        {
            isDriveProfileFresh = true;
        }
        driveMode["seen"] = isDriveProfileFresh;    // false = no Charisma frame in the last 3 s
        driveMode["profile"] = receivedDriveProfile; // 1 Comfort, 2 Normal, 3 Sport ... (0 = none)
        driveMode["source"] = driveProfileSource;    // 7 = Charisma_07, 1 = Charisma_01 engine request, 0 = none
        if (hasHaldexModeRequest)
        {
            driveMode["haldexRequest"] = receivedHaldexModeRequest; // Charisma_01 byte 3 bits 0-3
            driveMode["vaqRequest"] = receivedVaqModeRequest;       // Charisma_01 byte 4 bits 4-7
        }
        else
        {
            driveMode["haldexRequest"] = nullptr; // shown as "--"
            driveMode["vaqRequest"] = nullptr;
        }
        driveMode["forcing"] = getDriveModeForcedValue(); // 0-5, or -1 when not forcing
    }

    if (chassisOk) // if chassis CAN ok, set related values
    {
        data["speed"] = receivedVehicleSpeed;
        data["throttle"] = int(receivedPedalValue);
        data["asrOn"] = !asrForceModeFlag;
        data["tcOn"] = !tcForceModeFlag;
        data["hazardActive"] = hazardForceModeFlag;
        data["rpm"] = receivedVehicleRpm;
        data["boost"] = receivedVehicleBoost;
    }
    else // if chassis CAN not ok, set related values to null (displayed as "--" in the UI)
    {
        data["speed"] = nullptr;
        data["throttle"] = nullptr;
        data["asrOn"] = nullptr;
        data["tcOn"] = nullptr;
        data["hazardActive"] = nullptr;
        data["rpm"] = nullptr;
        data["boost"] = nullptr;
    }
    addEngineTemps(data); // coolant / oil temp from PQ Kombi_2 (V11.00.4)

    data["brakeIn"] = brakeSignalActive;
    data["brakeOut"] = brakeActive;
    data["handbrakeIn"] = handbrakeSignalActive;
    data["handbrakeOut"] = handbrakeActive;

    // Steering angle + health. Shown for every gen whose chassis bus carries a decoded
    // steering frame (VW LW_1 / LWI_01, Volvo P3 0x11600002) - the same gens as BLE /
    // dashCAN, not just the ones that use it for the lock scale (2/4/50/52).
    // V11.00.6: Gen1 (and 3/5/51...) were missing here although dashCAN showed them.
    // GM/Ford (41/42) and the other Volvo/SAAB gens stay out: nothing decoded there.
    // Stale/absent/unsupported -> unhealthy + null (shown as "--"). Magnitude only.
    {
        const bool steerSupported = haldexGeneration != 41 && haldexGeneration != 42 &&
                                    haldexGeneration != GEN_LANDROVER_GEN4 && // V11.00.13: no LR steering decode
                                    (!isVolvoOrSaabGeneration(haldexGeneration) ||
                                     haldexGeneration == GEN_VOLVO_GEN4); // V11.00.5: Volvo P3 shown (display only, scale estimated)
        const bool steerHealthy = steerSupported && chassisOk && receivedSteeringMs != 0 &&
                                  (millis() - receivedSteeringMs) <= steeringStaleMs;
        data["steeringHealthy"] = steerHealthy;
        if (steerHealthy)
            data["steeringAngle"] = (int)(fabsf(receivedSteeringAngle) + 0.5f);
        else
            data["steeringAngle"] = nullptr;
    }

    // Per-corner slip [FL, FR, RL, RR] as signed %. Fresh within 500 ms; a -128
    // sentinel (or stale data) reports null so the UI blanks rather than showing 0.
    {
        JsonArray slip = data["slip"].to<JsonArray>();
        const bool slipFresh = lastCornerSlipMs != 0 && (millis() - lastCornerSlipMs) < 500;
        for (uint8_t i = 0; i < 4; i++)
        {
            if (slipFresh && cornerSlip[i] != -128)
                slip.add((int)cornerSlip[i]);
            else
                slip.add(nullptr);
        }
    }

    // Steering-angle lock reduction (engagement-split display). Active only when
    // scaling ran this cycle and actually pulled the request back.
    data["steeringScaleEnabled"] = steeringScaleEnabled;
    if (steeringScaleIsActive())
    {
        data["steeringScaleActive"] = true;
        data["lockRequested"] = steeringScaleRequestedPct();
        data["lockScaled"] = steeringScaleResultPct();
    }
    else
    {
        data["steeringScaleActive"] = false;
        data["lockRequested"] = nullptr;
        data["lockScaled"] = nullptr;
    }
    data["steeringThrottleOverride"] = steeringThrottleOverridePct(); // how much of the steering reduction the throttle is cancelling (0-100 %)

    // CAN-decoded brake / handbrake state.
    //   Brake:     PQ Motor_2 MO2_BLS (Gen2/4/51), MQB ESP_05 ESP_Fahrer_bremst (Gen5).
    //   Handbrake: PQ Kombi_1 KO1_Handbremse (Gen2/4/51), MQB Kombi_01 KBI_Handbremse (Gen5).
    //   Insignia (41) / Focus RS (42): own decode in OpenHaldexC6_CHS.cpp (chsDecodeChassis).
    // Gen1 reads PQ Kombi_1 too since V11.00.0. Volvo P1 (V11.00.1) reads 0x0110483C;
    // the other Volvo generations have no CAN handbrake decode -> null ("--").
    if (chassisOk)
    {
        data["brakeFromCAN"] = brakeFromCAN;
        data["absBraking"] = absBrakingFromCAN; // PQ Bremse_1 b0.2 / MQB ESP_21 b7.0 (V11.00.0)
        const bool hbFromCANSupported = (haldexGeneration == 1 || haldexGeneration == 2 || haldexGeneration == 4 ||
                                         haldexGeneration == 50 || haldexGeneration == 51 ||
                                         haldexGeneration == 52 || haldexGeneration == 41 ||
                                         haldexGeneration == 42 || haldexGeneration == GEN_SAAB_93_XWD ||
                                         haldexGeneration == GEN_VOLVO_P1);
        if (hbFromCANSupported)
        {
            data["handbrakeFromCAN"] = handbrakeFromCAN;
        }
        else
        {
            data["handbrakeFromCAN"] = nullptr; // no CAN handbrake source for this platform (displayed as "--" in the UI)
        }
    }
    else
    {
        data["brakeFromCAN"] = nullptr;     // if chassis CAN not ok, set to null (displayed as "--" in the UI)
        data["handbrakeFromCAN"] = nullptr; // if chassis CAN not ok, set to null (displayed as "--" in the UI)
        data["absBraking"] = nullptr;       // if chassis CAN not ok, set to null (displayed as "--" in the UI)
    }

    data["lockTarget"] = int(lockTarget);
    data["expertPT"] = expertPtActive(); // V11.00.12: Expert is in a PT area - the car's own strategy has the Haldex

    // Why lock is being held at 0%, when it is (V10.00.2, from PR #54). Looks at
    // the mode actually in use: a force trigger (TC / hazards / button / drive
    // mode) wins over the selected mode. Stock and FWD ask for no lock on
    // purpose, so they never report a reason.
    {
        int modeInUse = getForcedModeValue(); // -1 = nothing forcing
        if (modeInUse < 0)
        {
            modeInUse = state.mode; // no force trigger: the selected mode
        }

        uint8_t blocking = 0; // LOCK_GATE_* bits, 0 = nothing blocking
        if (modeInUse != MODE_STOCK && modeInUse != MODE_FWD && !expertPtActive()) // a PT area is Stock: no reason to give
        {
            blocking = lockGateBlocking();
        }

        data["lockGated"] = (blocking != 0);
        JsonArray gateReasons = data["lockGateReasons"].to<JsonArray>(); // list of reasons for the web page
        if (blocking & LOCK_GATE_THROTTLE) // bit 0
        {
            gateReasons.add("throttle");
        }
        if (blocking & LOCK_GATE_UNDER_SPEED) // bit 1
        {
            gateReasons.add("underSpeed");
        }
        if (blocking & LOCK_GATE_ABOVE_SPEED) // bit 2
        {
            gateReasons.add("aboveSpeed");
        }
        if (blocking & LOCK_GATE_HANDBRAKE) // bit 3 (V11.00.0)
        {
            gateReasons.add("handbrake");
        }
        if (blocking & LOCK_GATE_ABS) // bit 4 (V11.00.0)
        {
            gateReasons.add("abs");
        }
    }

    if (haldexOk) // if haldex CAN ok set related values
    {
        data["lockActual"] = receivedHaldexEngagement;
        data["haldexState"] = receivedHaldexState;
        data["haldexEngagement"] = receivedHaldexEngagement;
        data["haldexEngagementRaw"] = receivedHaldexEngagementRaw;
        data["clutch1Report"] = receivedReportClutch1;
        data["clutch2Report"] = receivedReportClutch2;
        data["tempProtection"] = receivedTempProtection;
        data["couplingOpen"] = receivedCouplingOpen;
        data["speedLimit"] = receivedSpeedLimit;

        if (haldexGeneration == 41)
        {
            JsonObject gen41 = data["gen41"].to<JsonObject>();
            JsonObject sec = gen41["secAxle"].to<JsonObject>();
            sec["statusRaw"] = receivedSecAxleStatusRaw;
            sec["torqueNm"] = receivedSecAxleTorqueNm;
            sec["clutchState"] = receivedSecAxleClutchState;
            sec["active"] = receivedSecAxleActive;
            sec["fdcmHealthy"] = receivedSecAxleFdcmHealthy;
            sec["arc"] = receivedSecAxleArc;
            JsonObject rear = gen41["rearAxle"].to<JsonObject>();
            rear["statusFlags"] = receivedRearAxleStatusFlags;
            rear["metricA"] = receivedRearAxleMetricA;
            rear["metricB"] = receivedRearAxleMetricB;
            JsonObject hb = gen41["heartbeat"].to<JsonObject>();
            hb["aliveBus0"] = receivedHaldexAliveBus0;
            hb["drivetrainOk"] = receivedDrivetrainStateOk;
            hb["aliveBus0AgeMs"] = receivedHaldexAliveBus0Ms ? (millis() - receivedHaldexAliveBus0Ms) : 0;
            hb["drivetrainAgeMs"] = receivedDrivetrainStateMs ? (millis() - receivedDrivetrainStateMs) : 0;
        }

        // Haldex temperature: SAAB 0x210, VAG Gen4 / 0AY 0x2C0 byte 4 (V11.00.0), any
        // generation that decodes one. -128 = never received.
        if (receivedHaldexTempC != -128)
        {
            data["haldexTempC"] = receivedHaldexTempC;
        }
        else
        {
            data["haldexTempC"] = nullptr;
        }

        // Haldex health (V11.00.0): what the unit itself reports about its own
        // protections. Only generations whose feedback bits are known from the
        // OEM code report them (healthSupported); the rest show "--".
        data["healthSupported"] = haldexHealthSupported;
        if (haldexGeneration == 50)
        {
            data["leverAccepted"] = receivedLockRequestValid; // 0CQ Allrad_03 b1.6 [KM ALR_Sendestatus] [REA audit_gen5_0cq F3: 0 only for the "no ESP request" pattern - not proof of acceptance]
        }
        data["limpMode"] = receivedLimpMode;
        data["awdWarning"] = receivedAwdWarning;
        data["tempSensorFault"] = receivedTempSensorFault;
        data["faultStored"] = receivedFaultStored;

        // The genuine warnings, as a list the web page turns into a banner
        JsonArray warnings = data["haldexWarnings"].to<JsonArray>();
        if (haldexHealthSupported)
        {
            if (receivedReportClutch1)
            {
                warnings.add("fault"); // fault present (clutch / pump / valve)
            }
            if (receivedReportClutch2)
            {
                warnings.add("plausibility"); // signal plausibility fault
            }
            if (receivedTempProtection)
            {
                warnings.add("overTemp"); // over-temperature protection pulling the clutch back
            }
            if (receivedLimpMode)
            {
                warnings.add("limp"); // limp / fault reaction active
            }
            if (receivedAwdWarning)
            {
                warnings.add("lamp"); // AWD warning lamp requested
            }
            if (receivedTempSensorFault)
            {
                warnings.add("tempSensor"); // temperature sensor fault
            }
            if (receivedFaultStored)
            {
                warnings.add("faultStored"); // fault stored in the unit's memory
            }
        }
    }
    else // if haldex CAN not ok, set related values to null (displayed as "--" in the UI)
    {
        data["lockActual"] = nullptr;
        data["haldexState"] = nullptr;
        data["haldexEngagement"] = nullptr;
        data["haldexEngagementRaw"] = nullptr;
        data["clutch1Report"] = nullptr;
        data["clutch2Report"] = nullptr;
        data["tempProtection"] = nullptr;
        data["couplingOpen"] = nullptr;
        data["speedLimit"] = nullptr;
        data["haldexTempC"] = nullptr;
        data["healthSupported"] = nullptr;
        data["limpMode"] = nullptr;
        data["awdWarning"] = nullptr;
        data["tempSensorFault"] = nullptr;
        data["faultStored"] = nullptr;
    }

    data["chassisCAN"] = chassisOk;
    data["haldexCAN"] = haldexOk;

    // Volvo P1 (V11.00.1): what standalone is sending and what the AWD module
    // answers, for the live line on the Volvo P1 Standalone card
    if (haldexGeneration == GEN_VOLVO_P1)
    {
        JsonObject p1 = data["volvoP1"].to<JsonObject>(); // nested object: "volvoP1": { ... }
        p1["standalone"] = isStandalone;
        p1["lockPercent"] = lockRequestPercent();     // the lock % being shown to the module (0-100)
        p1["torque"] = volvoP1SentTorque;             // torque counts above zero
        p1["pedal"] = volvoP1SentPedal;               // pedal byte, 0-255
        p1["rpm"] = volvoP1SentRpm;
        p1["frontKmh"] = volvoP1SentFrontRaw / 100.0f; // km/h
        p1["rearKmh"] = volvoP1SentRearRaw / 100.0f;   // km/h
        if (haldexOk)
        {
            p1["engagementCount"] = volvoP1EngagementCount; // 0-1023, 540 = full
        }
        else
        {
            p1["engagementCount"] = nullptr; // no reply from the module
        }
    }
    data["busFailure"] = isBusFailure;
    const uint32_t lastChassisTick = lastCANChassisTick; // copies: the CAN tasks write these
    const uint32_t lastHaldexTick = lastCANHaldexTick;
    data["lastChassisMs"] = lastChassisTick != 0 ? (millis() - lastChassisTick) : 0;
    data["lastHaldexMs"] = lastHaldexTick != 0 ? (millis() - lastHaldexTick) : 0;
    data["diagToolActive"] = externalDiagActive(); // external scanner detected -> our live polling auto-paused
    data["udsSession"] = udsSessionMode;           // 0 idle, 1 default session, 3 extended session (Gen5 poller)
    data["canTxDropBus0"] = canTxDropBus0;         // canTransmit() failures since boot (chassis)
    data["canTxDropBus1"] = canTxDropBus1;         // canTransmit() failures since boot (Haldex)

    // UDS live data is Gen5 family (0CQ MQB / 0AY / VAQ) only.
    if (haldexOk && liveDiagEnabled && isGen5Family())
    {
        JsonObject uds = data["uds"].to<JsonObject>();
        uds["terminalVoltage"] = udsTerminalVoltage;
        uds["moduleTemp"] = udsModuleTemp;
        uds["clutchTemp"] = udsClutchTemp;
        uds["coolingFinTemp"] = udsCoolingFinTemp;
        uds["clutchCurrent"] = udsClutchCurrent;
        uds["clutchPWM"] = udsClutchPWM;
        uds["clutchVoltage"] = udsClutchVoltage;
        uds["clutchTorquePct"] = udsClutchTorquePct;
        if (haldexGeneration != 52)
            uds["axleSpeedDiff"] = udsAxleSpeedDiff; // no 0x2BF0 on the VAQ
    }

    // KWP2000/TP2.0 raw measuring-block capture (Gen2/4 PQ Haldex).
    if (haldexOk && liveDiagEnabled && (haldexGeneration == 2 || haldexGeneration == 4))
    {
        JsonObject kwp = data["kwp"].to<JsonObject>();
        kwp["connected"] = kwpTp20Connected;
        kwp["raw"] = kwpTp20RawDump;

        // Gen4 decoded/scaled measuring values (groups 1, 3, 4).
        if (haldexGeneration == 4)
        {
            kwp["oilTemp"] = kwpOilTemp;
            kwp["plateTemp"] = kwpPlateTemp;
            kwp["supplyVoltage"] = kwpSupplyVoltage;
            kwp["oilPressure"] = kwpOilPressure;
            kwp["estTorque"] = kwpEstTorque;
            kwp["clutchDuty"] = kwpClutchDuty;
            kwp["clutchValveCurrent"] = kwpClutchValveCurrent;
            kwp["pumpDuty"] = kwpPumpDuty;
            kwp["pumpCurrent"] = kwpPumpCurrent;
            kwp["pressureIsTarget"] = true; // Gen4 group 3 field 1 is the pressure TARGET, not the sensor
        }
        else
        {
            // Gen2 (V11.00.10): groups 1 / 2 / 7D. Pressure here is the measured sensor.
            kwp["oilTemp"] = kwpOilTemp;
            kwp["supplyVoltage"] = kwpSupplyVoltage;
            kwp["oilPressure"] = kwpOilPressure;
            kwp["pressureIsTarget"] = false;
            kwp["loggedIn"] = kwpGen2LoggedIn;
            if (kwpGen2LoginNrc != 0) // a key was refused and saved - shown as a warning on the card
            {
                JsonObject lf = kwp["loginFault"].to<JsonObject>();
                char hex[12];
                snprintf(hex, sizeof(hex), "%02X", kwpGen2LoginNrc);
                lf["nrc"] = hex;
                snprintf(hex, sizeof(hex), "%08lX", (unsigned long)kwpGen2LoginSeed);
                lf["seed"] = hex;
                snprintf(hex, sizeof(hex), "%08lX", (unsigned long)kwpGen2LoginKey);
                lf["key"] = hex;
            }
            if (kwpGen2LoginBlockedNrc != 0)
            {
                char hex[4];
                snprintf(hex, sizeof(hex), "%02X", kwpGen2LoginBlockedNrc);
                kwp["loginBlockedNrc"] = hex;
            }
            if (kwpGen2LoggedIn) // groups 0x32 / 0x33 behind SecurityAccess level 3
            {
                kwp["pumpCurrent"] = kwpPumpCurrent;
                kwp["clutchDuty"] = kwpClutchDuty;
                kwp["clutchValveCurrent"] = kwpClutchValveCurrent;
            }
            if (kwpBrakeSwitch >= 0)
                kwp["brakeSwitch"] = kwpBrakeSwitch == 1;
            else
                kwp["brakeSwitch"] = nullptr;
            JsonObject cs = kwp["canStatus"].to<JsonObject>();
            static const char *const names[4] = {"engine", "abs", "cluster", "steering"};
            for (uint8_t k = 0; k < 4; k++)
            {
                if (kwpCanStatusValid & (1u << k))
                    cs[names[k]] = (kwpCanStatus & (1u << k)) != 0;
                else
                    cs[names[k]] = nullptr;
            }
        }
    }

    // Ford Kuga (gen 45) live data over Ford KWP2000 (V11.00.10, fordDiagPoll). null = not read /
    // not in this firmware. Bar values only for the 8V41 DID set, and provisional.
    if (haldexOk && liveDiagEnabled && haldexGeneration == GEN_FORD_KUGA)
    {
        JsonObject fd = data["ford"].to<JsonObject>();
        const uint8_t valid = fordDiagValid;
        const bool v8V41 = (fordDiagVariant == 1);
        fd["connected"] = fordDiagConnected;
        fd["variant"] = v8V41 ? "8V41" : (fordDiagVariant == 2 ? "9V4N" : "");
        auto put = [&](const char *key, uint8_t bit, float value) {
            if (valid & bit)
                fd[key] = value;
            else
                fd[key] = nullptr;
        };
        put("supplyV", FORD_DIAG_SUPPLY, fordSupplyV);
        put("oilTemp", FORD_DIAG_TEMP, fordOilTemp);
        put("pressureRaw", FORD_DIAG_PMEAS, fordPressureRaw);
        put("setpointRaw", FORD_DIAG_PCMD, fordSetpointRaw);
        put("pressureBar", v8V41 ? FORD_DIAG_PMEAS : 0, fordPressureBar);
        put("setpointBar", v8V41 ? FORD_DIAG_PCMD : 0, fordSetpointBar);
        put("valveCurrent", FORD_DIAG_VALVE, fordValveCurrentA);
        put("pumpCurrent", FORD_DIAG_PUMP, fordPumpCurrentA);
    }

    data["uptimeMs"] = millis();
    data["freeHeap"] = ESP.getFreeHeap();
    data["bleConnected"] = bleIsConnected();
    data["bleCodeRequired"] = bleCodeIsRequired(); // flips when the first phone pairs
    data["lpChassisFrameCount"] = lpChassisFrameCount;
    data["lpHaldexFrameCount"] = lpHaldexFrameCount;
    {
        uint32_t fps, lostPs, frames, lost;
        espNowRelayStats(fps, lostPs, frames, lost);
        JsonObject relay = data["canRelay"].to<JsonObject>(); // CAN frames to gauges over ESP-NOW
        relay["rate"] = espNowRelayRate();
        relay["fps"] = fps;
        relay["lostPerSec"] = lostPs;
        relay["frames"] = frames;
        relay["lost"] = lost;
    }

    const int cpuUsage = getCPUUsagePercent(); // calculate CPU usage percentage using FreeRTOS task run time stats
    if (cpuUsage >= 0)
    {
        data["cpuUsage"] = cpuUsage;
    }
    else // if CPU usage couldn't be calculated, set to "--"
    {
        data["cpuUsage"] = nullptr;
    }

    sendJSON(request, 200, data);
}

// parse the current INCOMING request for settings: WebServer asks, this delivers
static void settingsOutgoing(AsyncWebServerRequest *request)
{
    JsonDocument data;
    // values
    data["haldexGeneration"] = haldexGeneration;
    data["canSpeedKbps"] = canBusSpeedKbps; // 500, or 250 for Volvo P2 Gen2
    data["tcForceModeValue"] = tcForceModeValue;
    data["hazardForceModeValue"] = hazardForceModeValue;
    data["extBtnForceModeValue"] = extBtnForceModeValue;
    data["disengageUnderSpeed"] = disengageUnderSpeed;
    data["disengageAboveSpeed"] = disengageAboveSpeed;
    data["disableThrottle"] = disableThrottle;
    data["mode"] = lastMode;
    data["lockReleaseRatePerSec"] = lockReleaseRatePerSec;
    data["lockReleaseEnabled"] = lockReleaseEnabled;
    data["steeringScaleEnabled"] = steeringScaleEnabled;
    data["FW_VERSION"] = FW_VERSION;
    data["boardVersion"] = BOARD_VERSION; // "1.0" -> System Info "V1.0"
    data["backupLayout"] = BACKUP_LAYOUT; // V11.00.14: /api/backup exists (the page checks before using it)

    // bools
    data["disableController"] = disableController;
    data["isStandalone"] = isStandalone;
    data["useCANifAvailable"] = useCANifAvailable;
    data["tcForceMode"] = tcForceMode;
    data["extButtonForceMode"] = extBtnForceMode;
    data["hazardForceMode"] = hazardForceMode;
    data["forceModesPriority"] = forceModesPriority; // which button trigger wins (0-5)

    // drive mode select: on/off, and the mode for each drive profile (255 = N/A)
    data["driveModeForce"] = driveModeForce;
    JsonArray driveModeValuesJSON = data["driveModeForceValues"].to<JsonArray>();
    for (uint8_t i = 0; i < DRIVE_PROFILE_COUNT; i++)
    {
        driveModeValuesJSON.add(driveModeForceValue[i]);
    }

    data["disableOnboardButton"] = disableOnboardButton;
    data["disableExternalButton"] = disableExternalButton;
    data["fixHunting"] = fixHunting;
    data["dangerZoneEnabled"] = dangerZoneEnabled;
    data["bpkCeilingNm"] = bpkCeilingNm;
    data["esp14MinFloorPct"] = esp14MinFloorPct;
    data["longLearnNotes"] = longLearnNotes;
    data["canSleepEnabled"] = canSleepEnabled;
    data["bleEnabled"] = bleEnabled;
    data["blePasskey"] = blePasskey;            // shown so a second phone can pair; never in backups (not in BACKUP_GENERAL_KEYS)
    data["bleCodeRequired"] = bleCodeIsRequired(); // false until the first phone has paired
    data["canSleepAggressive"] = canSleepAggressive;
    data["benchMode"] = benchMode;
    data["lpWakeThresholdFps"] = lpWakeThresholdFps;
    data["sleepCalState"] = sleepCalState;
    data["sleepCalAvgFps"] = sleepCalAvgFps;

    data["analyzerMode"] = analyzerMode;
    data["analyzerSerial"] = analyzerSerial;
    data["liveDiagEnabled"] = liveDiagEnabled;

    data["followBrake"] = followBrake;
    data["invertBrake"] = invertBrake;
    data["followHandbrake"] = followHandbrake;
    data["invertHandbrake"] = invertHandbrake;
    data["followABS"] = followABS; // V11.00.0

    // Volvo P1 standalone lock (V11.00.1) - the Volvo P1 Standalone card
    data["volvoP1LockSource"] = volvoP1LockSource;     // 0 engine torque, 1 wheel slip, 2 both
    data["volvoP1RoadSpeedKmh"] = volvoP1RoadSpeedKmh; // km/h
    data["volvoP1Gear"] = volvoP1Gear;                 // 1-3, the gear the rpm is worked out for
    data["volvoP1TorqueFull"] = volvoP1TorqueFull;     // torque counts at 100 %
    data["volvoP1SlipFullKmh"] = volvoP1SlipFullKmh;   // km/h at 100 %

    data["broadcastOpenHaldexOverCAN"] = broadcastOpenHaldexOverCAN;
    data["espNowEnabled"] = espNowEnabled; // gauges over ESP-NOW (can2gauge)
    data["espNowControl"] = espNowControl;
    data["canRelay"] = espNowRelayRate(); // CAN frames to gauges (cannRate_t)

    data["ledBrightness"] = ledBrightness;
    data["slipWheelbaseMm"] = slipWheelbaseMm;
    data["slipTrackFrontMm"] = slipTrackFrontMm;
    data["slipTrackRearMm"] = slipTrackRearMm;
    data["slipSteeringRatio"] = slipSteeringRatio;
    {
        // V11.00.14: this generation's default car, for the Vehicle Geometry card
        const VehicleGeometry &def = vehicleDefaultsForGeneration(haldexGeneration);
        JsonObject vd = data["vehicleDefaults"].to<JsonObject>();
        vd["car"] = def.car;
        vd["wheelbaseMm"] = def.wheelbaseMm;
        vd["trackFrontMm"] = def.trackFrontMm;
        vd["trackRearMm"] = def.trackRearMm;
        vd["steeringRatio"] = def.steeringRatio;
    }

    // throttle/speed/lock array send
    // row array
    JsonArray throttleArrayJSON = data["throttleArray"].to<JsonArray>();
    for (uint8_t i = 0; i < throttleArrayCount; i++)
    {
        throttleArrayJSON.add(throttleArray[i]);
    }

    // column array
    JsonArray speedArrayJSON = data["speedArray"].to<JsonArray>();
    for (uint8_t i = 0; i < speedArrayCount; i++)
    {
        speedArrayJSON.add(speedArray[i]);
    }

    // lock array
    JsonArray lockArrayJSON = data["lockArray"].to<JsonArray>();
    for (uint8_t throttlePos = 0; throttlePos < throttleArrayCount; throttlePos++)
    {
        JsonArray throttleRow = lockArrayJSON.add<JsonArray>();
        for (uint8_t speedPos = 0; speedPos < speedArrayCount; speedPos++)
        {
            throttleRow.add(lockArray[throttlePos][speedPos]);
        }
    }

    // Expert PT cells (V11.00.12): same 7x7 layout as lockArray, 1 = PT
    JsonArray lockPTJSON = data["lockPT"].to<JsonArray>();
    for (uint8_t throttlePos = 0; throttlePos < throttleArrayCount; throttlePos++)
    {
        JsonArray ptRow = lockPTJSON.add<JsonArray>();
        for (uint8_t speedPos = 0; speedPos < speedArrayCount; speedPos++)
        {
            ptRow.add(lockCellPt(throttlePos, speedPos) ? 1 : 0);
        }
    }

    // steering-angle lock-scale curve (breakpoints + 0-100% multipliers)
    JsonArray steeringArrayJSON = data["steeringArray"].to<JsonArray>();
    for (uint8_t i = 0; i < steeringArrayCount; i++)
    {
        steeringArrayJSON.add(steeringArray[i]);
    }
    JsonArray steeringScaleJSON = data["steeringLockScaleArray"].to<JsonArray>();
    for (uint8_t i = 0; i < steeringArrayCount; i++)
    {
        steeringScaleJSON.add(steeringLockScaleArray[i]);
    }
    // throttle override for the steering curve (saved with the curve via /api/tune)
    data["steeringThrottleOverrideEnabled"] = steeringThrottleOverrideEnabled; // on / off
    data["steeringThrottleOverrideStart"] = steeringThrottleOverrideStart;     // pedal % where it starts
    data["steeringThrottleOverrideFull"] = steeringThrottleOverrideFull;       // pedal % where it is complete

    // Frame-edit blocks for the current generation (per-CAN-ID passthrough toggles).
    {
        int gi = frameEditGenIdx(haldexGeneration);
        JsonArray fb = data["frameBlocks"].to<JsonArray>();
        if (gi >= 0)
        {
            for (uint16_t i = 0; i < frameEditBlockCount; i++)
            {
                if (frameEditBlocks[i].genIdx == (uint8_t)gi)
                {
                    JsonObject o = fb.add<JsonObject>();
                    o["bit"] = frameEditBlocks[i].bit;
                    o["name"] = frameEditBlocks[i].name;
                    o["canId"] = frameEditBlocks[i].canId;
                    o["enabled"] = frameEditEnabled((uint8_t)gi, frameEditBlocks[i].bit);
                    o["def"] = (bool)((frameEditMaskDefaults[gi] >> frameEditBlocks[i].bit) & 0x1ULL); // in the normal-mode default set
                }
            }
        }
    }

    sendJSON(request, 200, data);
}

// manage settings (saved from Web, handled here): WebServer sends, this handles
static void settingsIncoming(AsyncWebServerRequest *request, const String &body)
{
    JsonDocument data;
    if (deserializeJson(data, body) != DeserializationError::Ok)
    {
        DEBUG("Invalid JSON");
        sendError(request, 400, "Invalid JSON");
        return;
    }

    // A running Long Learn swaps frame blocks on the live mask: changing the
    // generation, the mode (standalone = the other mask) or a block under it
    // would make every later read wrong without anyone noticing.
    if (longLearnActive && (!data["haldexGeneration"].isNull() || !data["isStandalone"].isNull() ||
                            !data["frameEditBit"].isNull() || !data["frameEditReset"].isNull()))
    {
        sendError(request, 409, "Long Learn is running - cancel it first");
        return;
    }

    if (data["haldexGeneration"].is<uint8_t>())
    {
        int generation = data["haldexGeneration"];
        // Volvo P2 Gen2 runs CAN at 250 kbit/s: picking it (or leaving it) saves and
        // restarts the controller a couple of seconds later (checkCanSpeedChange).
        if (isSupportedGeneration((uint8_t)generation))
        {
            const uint8_t oldGeneration = haldexGeneration;
            haldexGeneration = (uint8_t)generation;
            udsApplyDefaultIds(); // move the UDS pair with the generation (no-op while the serial lab has it pinned)
            vehicleGeometryFollowGeneration(oldGeneration, haldexGeneration); // untouched geometry -> the new generation's car
            // V11.00.14 (2026-11-27): the learn table was learned on the old generation's unit - drop it.
            // keepLearnTable: the config import sets a CAN-speed generation AFTER restoring
            // that file's table (which belongs to the new generation), so it keeps it.
            const bool keepLearnTable = data["keepLearnTable"].is<bool>() && data["keepLearnTable"].as<bool>();
            if (oldGeneration != haldexGeneration && !keepLearnTable && !haldexLearnActive)
            {
                learnTableForget();
            }
        }
    }

    // Driving settings: shared validation with BLE (OpenHaldexC6_Settings.cpp).
    {
        struct
        {
            const char *key;
            uint8_t id;
        } const drivingBools[] = {
            {"tcForceMode", DS_TC_FORCE_MODE},
            {"hazardForceMode", DS_HAZARD_FORCE_MODE},
            {"extButtonForceMode", DS_EXT_BTN_FORCE_MODE},
            {"followBrake", DS_FOLLOW_BRAKE},
            {"followHandbrake", DS_FOLLOW_HANDBRAKE},
            {"steeringScaleEnabled", DS_STEERING_SCALE_ENABLED},
            {"lockReleaseEnabled", DS_LOCK_RELEASE_ENABLED},
            {"liveDiagEnabled", DS_LIVE_DIAG_ENABLED},
        };
        for (const auto &b : drivingBools)
            if (data[b.key].is<bool>())
                applyDrivingSetting(b.id, data[b.key].as<bool>() ? 1 : 0);

        struct
        {
            const char *key;
            uint8_t id;
            int maxValue; // clamp before the u16 conversion (negative / huge JSON numbers)
        } const drivingNumbers[] = {
            {"tcForceModeValue", DS_TC_FORCE_MODE_VALUE, 255},
            {"hazardForceModeValue", DS_HAZARD_FORCE_MODE_VALUE, 255},
            {"extBtnForceModeValue", DS_EXT_BTN_FORCE_MODE_VALUE, 255},
            {"disengageUnderSpeed", DS_DISENGAGE_UNDER_SPEED, 300},
            {"disengageAboveSpeed", DS_DISENGAGE_ABOVE_SPEED, 300},
            {"disableThrottle", DS_DISABLE_THROTTLE, 100},
            {"lockReleaseRatePerSec", DS_LOCK_RELEASE_RATE, 500},
            {"ledBrightness", DS_LED_BRIGHTNESS, 255},
        };
        for (const auto &n : drivingNumbers)
            if (data[n.key].is<float>())
                applyDrivingSetting(n.id, (uint16_t)constrain(lroundf(data[n.key].as<float>()), 0L, (long)n.maxValue));
    }

    // Force Mode Priority (which of TC / hazards / external button wins). The web page
    // has always sent this, but it was never read here, so it didn't save.
    if (data["forceModesPriority"].is<int>())
    {
        int priority = data["forceModesPriority"];
        if (priority >= 0 && priority <= 5)
        {
            forceModesPriority = (uint8_t)priority;
        }
    }

    // Drive mode select: on/off
    if (data["driveModeForce"].is<bool>())
    {
        driveModeForce = data["driveModeForce"];
    }

    // Drive mode select: the whole table, one OpenHaldex mode per drive profile.
    // Anything that isn't a real mode (0-5) is stored as N/A.
    if (data["driveModeForceValues"].is<JsonArray>())
    {
        JsonArray values = data["driveModeForceValues"];
        for (uint8_t i = 0; i < DRIVE_PROFILE_COUNT && i < values.size(); i++)
        {
            int value = -1;
            if (values[i].is<int>())
            {
                value = values[i];
            }
            if (value >= 0 && value < (int)openHaldexMode_t_MAX)
            {
                driveModeForceValue[i] = (uint8_t)value;
            }
            else
            {
                driveModeForceValue[i] = DRIVE_MODE_NOT_SET;
            }
        }
    }

    if (data["disableController"].is<bool>())
    {
        setControllerDisabled(data["disableController"]);
    }

    if (data["isStandalone"].is<bool>())
    {
        setStandaloneMode(data["isStandalone"].as<bool>()); // Stock -> FWD when switching on (V11.00.12)
    }

    if (data["analyzerMode"].is<bool>())
    {
        setAnalyzerMode(data["analyzerMode"]);
    }

    if (data["analyzerSerial"].is<bool>())
    {
        setAnalyzerSerialMode(data["analyzerSerial"]);
    }

    if (data["useCANifAvailable"].is<bool>())
    {
        useCANifAvailable = data["useCANifAvailable"];
    }

    if (data["disableOnboardButton"].is<bool>())
    {
        disableOnboardButton = data["disableOnboardButton"];
    }

    if (data["disableExternalButton"].is<bool>())
    {
        disableExternalButton = data["disableExternalButton"];
    }

    if (data["fixHunting"].is<bool>())
    {
        fixHunting = data["fixHunting"];
    }

    if (data["dangerZoneEnabled"].is<bool>())
    {
        dangerZoneEnabled = data["dangerZoneEnabled"];
    }

    if (data["bpkCeilingNm"].is<uint16_t>())
    {
        bpkCeilingNm = (uint16_t)constrain((int)data["bpkCeilingNm"], 10, 500);
    }

    if (data["esp14MinFloorPct"].is<uint8_t>())
    {
        esp14MinFloorPct = (uint8_t)constrain((int)data["esp14MinFloorPct"], 0, 100);
    }

    if (data["canSleepEnabled"].is<bool>())
    {
        canSleepEnabled = data["canSleepEnabled"];
        // Aggressive depends on the sleep path (esp_pm_configure / light sleep).
        // Disabling the base feature should also disable aggressive mode
        if (!canSleepEnabled)
            canSleepAggressive = false;
    }
    if (data["canSleepAggressive"].is<bool>())
    {
        canSleepAggressive = data["canSleepAggressive"];
        // Enabling aggressive implies the base sleep path is on.
        if (canSleepAggressive)
            canSleepEnabled = true;
    }
    if (data["benchMode"].is<bool>())
    {
        benchMode = data["benchMode"];
    }
    if (data["bleEnabled"].is<bool>())
    {
        bleEnabled = data["bleEnabled"];
    }
    if (data["lpWakeThresholdFps"].is<uint16_t>())
    {
        lpWakeThresholdFps = constrain((uint16_t)data["lpWakeThresholdFps"], 0, 2000);
    }
    if (data["sleepCalArm"].is<bool>())
    {
        sleepCalState = data["sleepCalArm"] ? SLEEP_CAL_ARMED : SLEEP_CAL_DECLINED;
    }
    if (data["invertBrake"].is<bool>())
    {
        invertBrake = data["invertBrake"];
    }

    if (data["invertHandbrake"].is<bool>())
    {
        invertHandbrake = data["invertHandbrake"];
    }

    if (data["followABS"].is<bool>())
    {
        followABS = data["followABS"]; // V11.00.0: off = our lock overrules the ABS
    }

    // Volvo P1 standalone lock (V11.00.1). Each one is kept inside a safe range.
    if (data["volvoP1LockSource"].is<int>())
    {
        volvoP1LockSource = (uint8_t)constrain((int)data["volvoP1LockSource"], VOLVO_P1_SOURCE_TORQUE, VOLVO_P1_SOURCE_BOTH);
    }
    if (data["volvoP1RoadSpeedKmh"].is<int>())
    {
        volvoP1RoadSpeedKmh = (uint8_t)constrain((int)data["volvoP1RoadSpeedKmh"], 5, 120); // never 0: the module won't lock while stopped
    }
    if (data["volvoP1Gear"].is<int>())
    {
        volvoP1Gear = (uint8_t)constrain((int)data["volvoP1Gear"], 1, 3); // only 1st-3rd were in the logs
    }
    if (data["volvoP1TorqueFull"].is<int>())
    {
        volvoP1TorqueFull = (uint16_t)constrain((int)data["volvoP1TorqueFull"], 0, 1000); // 12-bit field, 2048 = 0, so up to +1000 is safe
    }
    if (data["volvoP1SlipFullKmh"].is<int>())
    {
        volvoP1SlipFullKmh = (uint8_t)constrain((int)data["volvoP1SlipFullKmh"], 0, 40);
    }

    if (data["espNowEnabled"].is<bool>())
        espNowEnabled = data["espNowEnabled"];
    if (data["espNowControl"].is<bool>())
        espNowControl = data["espNowControl"];
    if (data["canRelay"].is<int>())
        espNowSetRelayRate((uint8_t)constrain(data["canRelay"].as<int>(), 0, 255)); // out of range = off

    if (data["broadcastOpenHaldexOverCAN"].is<bool>())
    {
        broadcastOpenHaldexOverCAN = data["broadcastOpenHaldexOverCAN"];
    }

    // Vehicle geometry for per-corner slip (computeCornerSlip): mm/ratio,
    // 0 is rejected by the calc itself so a bad value just disables slip calc
    // rather than producing nonsense - safe range is generous, not exact.
    if (data["slipWheelbaseMm"].is<int>())
    {
        slipWheelbaseMm = (uint16_t)constrain((int)data["slipWheelbaseMm"], 0, 4000);
    }
    if (data["slipTrackFrontMm"].is<int>())
    {
        slipTrackFrontMm = (uint16_t)constrain((int)data["slipTrackFrontMm"], 0, 2200);
    }
    if (data["slipTrackRearMm"].is<int>())
    {
        slipTrackRearMm = (uint16_t)constrain((int)data["slipTrackRearMm"], 0, 2200);
    }
    if (data["slipSteeringRatio"].is<float>())
    {
        slipSteeringRatio = constrain((float)data["slipSteeringRatio"], 1.0f, 40.0f);
    }
    // "Use defaults" on the Vehicle Geometry card (V11.00.14): this generation's car
    if (data["vehicleDefaults"].is<bool>() && data["vehicleDefaults"].as<bool>())
    {
        applyVehicleDefaults(haldexGeneration);
    }

    // Long Learn chassis/car notes (free text, exported with the report)
    if (data["longLearnNotes"].is<const char *>())
    {
        const char *n = data["longLearnNotes"];
        memset(longLearnNotes, 0, sizeof(longLearnNotes));
        strncpy(longLearnNotes, n, LL_NOTES_LEN);
    }

    // Frame-edit gating: reset all masks to defaults, or toggle a single block
    // (bit) for the currently-selected generation.
    if (data["frameEditReset"].is<bool>() && data["frameEditReset"].as<bool>())
    {
        resetFrameEditMask();
    }
    if (data["frameEditBit"].is<uint8_t>() && data["frameEditOn"].is<bool>())
    {
        int gi = frameEditGenIdx(haldexGeneration);
        uint8_t bit = data["frameEditBit"];
        if (gi >= 0 && bit < 64)
        {
            // Edit the mask for the mode we're currently in (standalone vs normal).
            if (data["frameEditOn"].as<bool>())
                activeFrameEditMask()[gi] |= (1ULL << bit);
            else
                activeFrameEditMask()[gi] &= ~(1ULL << bit);
            frameEditKeepRequired(); // V11.00.14 (2026-11-27): a block the unit can't run without snaps back on
        }
    }

    JsonDocument resp;
    resp["ok"] = true;
    sendJSON(request, 200, resp);
}

// ---- Config backup extras (V11.00.14): GET / POST /api/backup ----------------
// What /api/settings can't carry back: the frame-edit masks of EVERY generation
// in BOTH modes (settings only shows the current one), the car IDs Long Learn
// holds back from the Haldex, and the learn table.
// Compatibility, both ways:
//  - masks are named per generation slot, not by position, so a file from an
//    older or newer firmware lines up; unknown names are skipped and a slot the
//    file doesn't have is left as it is;
//  - each part (frameEdits / heldBack / learnTable) is optional;
//  - 64-bit masks travel as hex text (a JS number only holds 53 bits);
//  - firmware without this endpoint: the page sees no "backupLayout" in
//    /api/settings and falls back to the old per-bit frame edits.
static const char *const FRAME_EDIT_SLOT_NAMES[FE_GEN_COUNT] = {
    "gen1", "gen2", "gen4", "gen50", "gen51",
    nullptr, // FE_GEN_SPARE_60: kept free, never in a backup
    "volvoP1", "volvoGen4", "fordKuga", "landRoverGen4",
};

static bool parseHex64(const char *text, uint64_t &out)
{
    if (text == nullptr || *text == 0)
        return false;
    char *end = nullptr;
    out = strtoull(text, &end, 16); // "0x" prefix allowed
    return end != nullptr && *end == 0;
}

static void backupOutgoing(AsyncWebServerRequest *request)
{
    JsonDocument data;
    data["layout"] = BACKUP_LAYOUT;

    JsonObject fe = data["frameEdits"].to<JsonObject>();
    for (uint8_t i = 0; i < FE_GEN_COUNT; i++)
    {
        if (FRAME_EDIT_SLOT_NAMES[i] == nullptr)
            continue;
        char normal[19], standalone[19];
        snprintf(normal, sizeof(normal), "0x%016llX", (unsigned long long)frameEditMask[i]);
        snprintf(standalone, sizeof(standalone), "0x%016llX", (unsigned long long)frameEditMaskSA[i]);
        JsonObject slot = fe[FRAME_EDIT_SLOT_NAMES[i]].to<JsonObject>();
        slot["normal"] = normal;
        slot["standalone"] = standalone;
    }

    JsonArray held = data["heldBack"].to<JsonArray>();
    for (uint8_t h = 0; h < HELD_BACK_MAX; h++)
    {
        if (heldBackIds[h] == 0)
            continue;
        char idTxt[12];
        snprintf(idTxt, sizeof(idTxt), "0x%lX", (unsigned long)(heldBackIds[h] & ~HELD_BACK_EXTD));
        JsonObject o = held.add<JsonObject>();
        o["id"] = idTxt;
        o["ext"] = (bool)(heldBackIds[h] & HELD_BACK_EXTD);
    }

    JsonObject learn = data["learnTable"].to<JsonObject>();
    learn["valid"] = haldexLearnTableValid;
    if (haldexLearnTableValid)
    {
        JsonArray v = learn["values"].to<JsonArray>();
        for (uint8_t i = 0; i < sizeof(haldexLearnTable); i++)
            v.add(haldexLearnTable[i]);
    }
    sendJSON(request, 200, data);
}

static void backupIncoming(AsyncWebServerRequest *request, const String &body)
{
    JsonDocument data;
    if (deserializeJson(data, body) != DeserializationError::Ok)
    {
        sendError(request, 400, "Invalid JSON");
        return;
    }
    if (longLearnActive || haldexLearnActive)
    {
        sendError(request, 409, "A learn is running - cancel it first");
        return;
    }

    // Check every part before anything is written (no half restore).
    uint64_t newMask[FE_GEN_COUNT], newMaskSA[FE_GEN_COUNT];
    memcpy(newMask, frameEditMask, sizeof(newMask));
    memcpy(newMaskSA, frameEditMaskSA, sizeof(newMaskSA));
    JsonObject fe = data["frameEdits"].as<JsonObject>();
    if (!fe.isNull())
    {
        for (uint8_t i = 0; i < FE_GEN_COUNT; i++)
        {
            if (FRAME_EDIT_SLOT_NAMES[i] == nullptr)
                continue;
            JsonObject slot = fe[FRAME_EDIT_SLOT_NAMES[i]].as<JsonObject>();
            if (slot.isNull())
                continue; // not in the file (older firmware didn't have this generation): keep
            uint64_t v;
            if (!slot["normal"].isNull())
            {
                if (!parseHex64(slot["normal"].as<const char *>(), v)) { sendError(request, 400, "Bad frame-edit mask"); return; }
                newMask[i] = v;
            }
            if (!slot["standalone"].isNull())
            {
                if (!parseHex64(slot["standalone"].as<const char *>(), v)) { sendError(request, 400, "Bad frame-edit mask"); return; }
                newMaskSA[i] = v;
            }
        }
    }

    uint32_t newHeld[HELD_BACK_MAX] = {0, 0, 0, 0};
    JsonArray held = data["heldBack"].as<JsonArray>();
    if (!held.isNull())
    {
        if (held.size() > HELD_BACK_MAX) { sendError(request, 400, "Too many held-back IDs"); return; }
        uint8_t n = 0;
        for (JsonObject o : held)
        {
            uint64_t id;
            if (!parseHex64(o["id"].as<const char *>(), id) || id == 0 || id > 0x1FFFFFFFULL)
            {
                sendError(request, 400, "Bad held-back ID");
                return;
            }
            const bool ext = o["ext"].as<bool>() || id > 0x7FF;
            newHeld[n++] = (uint32_t)id | (ext ? HELD_BACK_EXTD : 0);
        }
    }

    JsonObject learn = data["learnTable"].as<JsonObject>();
    uint8_t newLearn[sizeof(haldexLearnTable)];
    bool learnGiven = !learn.isNull() && learn["valid"].is<bool>();
    bool learnValid = learnGiven && learn["valid"].as<bool>();
    if (learnValid)
    {
        JsonArray v = learn["values"].as<JsonArray>();
        if (v.isNull() || v.size() != sizeof(newLearn)) { sendError(request, 400, "Bad learn table"); return; }
        uint8_t i = 0;
        for (JsonVariant x : v)
        {
            if (!x.is<int>() || x.as<int>() < 0 || x.as<int>() > 255) { sendError(request, 400, "Bad learn table"); return; }
            newLearn[i++] = (uint8_t)x.as<int>();
        }
    }

    // All good: apply. writeEEP saves it within 2 s.
    if (!fe.isNull())
    {
        memcpy(frameEditMask, newMask, sizeof(newMask));
        memcpy(frameEditMaskSA, newMaskSA, sizeof(newMaskSA));
        frameEditKeepRequired(); // an older file can't switch off what the unit needs
    }
    if (!held.isNull())
        memcpy(heldBackIds, newHeld, sizeof(newHeld));
    if (learnGiven)
    {
        if (learnValid)
            memcpy(haldexLearnTable, newLearn, sizeof(newLearn));
        haldexLearnTableValid = learnValid; // a file with no learn table clears this one, like the unit it came from
    }

    JsonDocument resp;
    resp["ok"] = true;
    sendJSON(request, 200, resp);
}

// manage tune (saved from Web, handled here): WebServer sends, this handles
static void tuneIncoming(AsyncWebServerRequest *request, const String &body)
{
    JsonDocument data;
    if (deserializeJson(data, body) != DeserializationError::Ok)
    {
        DEBUG("Invalid JSON");
        sendError(request, 400, "Invalid JSON");
        return;
    }

    // V11.00.8: every array is checked BEFORE anything is written - a bad lock-map row
    // used to return after the speed / throttle axes were already saved (half a map).
    JsonArray speedArrayJSON = data["speedArray"].as<JsonArray>();
    JsonArray throttleArrayJSON = data["throttleArray"].as<JsonArray>();
    JsonArray lockArrayJSON = data["lockArray"].as<JsonArray>();
    const bool hasMap = !speedArrayJSON.isNull() || !throttleArrayJSON.isNull() || !lockArrayJSON.isNull();
    if (hasMap)
    {
        if (speedArrayJSON.size() != speedArrayCount || throttleArrayJSON.size() != throttleArrayCount ||
            lockArrayJSON.size() != throttleArrayCount)
        {
            sendError(request, 400, "Invalid map array length");
            return;
        }
        for (uint8_t throttle = 0; throttle < throttleArrayCount; throttle++)
        {
            if (lockArrayJSON[throttle].as<JsonArray>().size() != speedArrayCount)
            {
                sendError(request, 400, "Invalid lock map row");
                return;
            }
        }
    }
    JsonArray lockPTJSON = data["lockPT"].as<JsonArray>();
    const bool hasPT = !lockPTJSON.isNull();
    if (hasPT)
    {
        bool ok = (lockPTJSON.size() == throttleArrayCount);
        for (uint8_t throttle = 0; ok && throttle < throttleArrayCount; throttle++)
        {
            ok = (lockPTJSON[throttle].as<JsonArray>().size() == speedArrayCount);
        }
        if (!ok)
        {
            sendError(request, 400, "Invalid PT map");
            return;
        }
    }
    JsonArray steeringArrayJSON = data["steeringArray"].as<JsonArray>();
    JsonArray steeringScaleJSON = data["steeringLockScaleArray"].as<JsonArray>();
    const bool hasSteering = !steeringArrayJSON.isNull() || !steeringScaleJSON.isNull();
    if (hasSteering &&
        (steeringArrayJSON.size() != steeringArrayCount || steeringScaleJSON.size() != steeringArrayCount))
    {
        sendError(request, 400, "Invalid steering array length");
        return;
    }

    // Steering throttle override (optional - only changed when present in the payload).
    // Checked before anything changes, so a bad start/full pair rejects the whole save.
    {
        int newStart = steeringThrottleOverrideStart; // start with the current values
        int newFull = steeringThrottleOverrideFull;
        if (data["steeringThrottleOverrideStart"].is<int>())
        {
            newStart = constrain(data["steeringThrottleOverrideStart"].as<int>(), 0, 100); // pedal % where the override starts
        }
        if (data["steeringThrottleOverrideFull"].is<int>())
        {
            newFull = constrain(data["steeringThrottleOverrideFull"].as<int>(), 0, 100); // pedal % where the override is complete
        }
        if (newFull <= newStart)
        {
            JsonDocument response;
            response["ok"] = false;
            response["error"] = "Throttle override: Full % must be above Start %";
            sendJSON(request, 400, response);
            return;
        }
        steeringThrottleOverrideStart = (uint8_t)newStart;
        steeringThrottleOverrideFull = (uint8_t)newFull;
        if (data["steeringThrottleOverrideEnabled"].is<bool>())
        {
            steeringThrottleOverrideEnabled = data["steeringThrottleOverrideEnabled"].as<bool>(); // on / off
        }
    }

    // Speed/throttle/lock map (optional - only applied when present in the payload; checked above).
    if (hasMap)
    {
        // fill throttle array
        for (uint8_t i = 0; i < throttleArrayCount; i++)
        {
            throttleArray[i] = (uint8_t)(throttleArrayJSON[i] | 0);
        }

        // fill speed array
        for (uint8_t i = 0; i < speedArrayCount; i++)
        {
            speedArray[i] = (uint16_t)(speedArrayJSON[i] | 0);
        }

        // fill lock array
        for (uint8_t throttle = 0; throttle < throttleArrayCount; throttle++)
        {
            JsonArray throttleRow = lockArrayJSON[throttle].as<JsonArray>();
            for (uint8_t speed = 0; speed < speedArrayCount; speed++)
            {
                const int cell = throttleRow[speed] | 0;
                lockArray[throttle][speed] = (uint8_t)constrain(cell, 0, 100); // V11.00.12: was an unchecked cast
            }
        }
    }

    // Expert PT cells (optional - left as they are when the payload has none, so
    // a tool that only sends lockArray doesn't wipe them; checked above).
    if (hasPT)
    {
        uint64_t mask = 0;
        for (uint8_t throttle = 0; throttle < throttleArrayCount; throttle++)
        {
            JsonArray ptRow = lockPTJSON[throttle].as<JsonArray>();
            for (uint8_t speed = 0; speed < speedArrayCount; speed++)
            {
                if ((ptRow[speed] | 0) != 0)
                {
                    mask |= 1ULL << (throttle * speedArrayCount + speed);
                }
            }
        }
        lockPTMask = mask;
    }

    // Steering-angle lock-scale curve (optional - breakpoints + 0-100% multipliers; checked above).
    if (hasSteering)
    {
        for (uint8_t i = 0; i < steeringArrayCount; i++)
        {
            steeringArray[i] = (uint16_t)(steeringArrayJSON[i] | 0);
            steeringLockScaleArray[i] = (uint8_t)constrain((int)(steeringScaleJSON[i] | 0), 0, 100);
        }
    }

    JsonDocument resp;
    resp["ok"] = true;
    sendJSON(request, 200, resp);
}

// setup main section for handling requests
void setupAPI()
{
    // ===== GET ENDPOINTS =====

    // GET /api/settings - retrieve all current settings
    webServer.on("/api/settings", HTTP_GET, [](AsyncWebServerRequest *request)
                 { settingsOutgoing(request); });

    // GET /api/backup - config backup extras: every frame-edit mask, held-back IDs, learn table (V11.00.14)
    webServer.on("/api/backup", HTTP_GET, [](AsyncWebServerRequest *request)
                 { backupOutgoing(request); });

    // GET /api/dashboard - retrieve live status data (polled regularly by JS)
    webServer.on("/api/dashboard", HTTP_GET, [](AsyncWebServerRequest *request)
                 {
                     otaNoteWebActivity(); // a browser is on the UI - hold WiFi up even if it came in via the home router
                     statusOutgoing(request); });

    // GET /api/live - just the Live Data tiles and the lock, for the faster refresh (V11.00.4)
    webServer.on("/api/live", HTTP_GET, liveOutgoing);

    // GET /api/uds/read - UDS read-by-identifier helper
    webServer.on("/api/uds/read", HTTP_GET, [](AsyncWebServerRequest *request)
                 {
                     auto reqP = request->getParam("req", false);
                     auto resP = request->getParam("res", false);
                     auto didP = request->getParam("did", false);

                     if (!reqP || !resP || !didP)
                     {
                         JsonDocument response;
                         response["error"] = "Parameters required: req, res, did";
                         sendJSON(request, 400, response);
                         return;
                     }

                     uint32_t requestId = strtoul(reqP->value().c_str(), nullptr, 16);
                     uint32_t responseId = strtoul(resP->value().c_str(), nullptr, 16);
                     uint16_t did = (uint16_t)strtoul(didP->value().c_str(), nullptr, 16);
                     // Optional bus selector: 0 = chassis (default), 1 = Haldex side.
                     auto busP = request->getParam("bus", false);
                     const uint8_t bus = busP ? (uint8_t)(strtoul(busP->value().c_str(), nullptr, 10) != 0) : 0;

                     // A malformed req/res parses to 0 via strtoul, which would transmit
                     // on CAN ID 0x0 (highest priority on the bus) or wait on an ID that
                     // never comes - reject both.
                     if (requestId == 0 || requestId > 0x1FFFFFFFu || responseId == 0 || responseId > 0x1FFFFFFFu)
                     {
                         JsonDocument response;
                         response["success"] = false;
                         response["error"] = "Invalid request/response ID";
                         sendJSON(request, 400, response);
                         return;
                     }

                     // One read at a time: udsWebRespId doubles as the busy flag, and
                     // two concurrent reads would fight over the tap queue.
                     if (udsWebRespId != 0)
                     {
                         JsonDocument response;
                         response["success"] = false;
                         response["error"] = "UDS read already in progress";
                         sendJSON(request, 429, response);
                         return;
                     }

                     if (udsWebRxQueue == nullptr)
                     {
                         udsWebRxQueue = xQueueCreate(8, sizeof(twai_message_t));
                     }
                     if (udsWebRxQueue == nullptr)
                     {
                         JsonDocument response;
                         response["success"] = false;
                         response["error"] = "Out of memory";
                         sendJSON(request, 500, response);
                         return;
                     }

                     // Responses arrive via the parse-task copy-tap for the chosen bus
                     // (the frame still flows through the gateway), so this never steals
                     // frames from the bridge. The 300 ms timeout bounds how long this
                     // handler holds the async_tcp task; single-frame replies land well
                     // inside it.
                     xQueueReset(udsWebRxQueue);
                     udsManualLastMs = millis(); // background live polls (Ford) back off for a while
                     udsWebBus = bus;
                     udsWebRespId = responseId;

                     OpenHaldexC6::UDS uds(bus ? twaiBus1 : twaiBus0, udsWebRxQueue);
                     uint8_t buffer[256];
                     size_t bufferLen = sizeof(buffer);
                     const bool ok = uds.readDataByIdentifier(requestId, responseId, did, buffer, bufferLen, 300);
                     udsWebRespId = 0;

                     if (!ok)
                     {
                         JsonDocument response;
                         response["success"] = false;
                         response["error"] = "UDS read failed";
                         sendJSON(request, 500, response);
                         return;
                     }

                     JsonDocument response;
                     response["success"] = true;
                     response["did"] = did;
                     JsonArray dataArray = response["data"].to<JsonArray>();
                     for (size_t i = 0; i < bufferLen; i++)
                         dataArray.add(buffer[i]);

                     sendJSON(request, 200, response); });

    // V11.00.5: fault codes - GET /api/dtc = state + last result,
    // POST /api/dtc/read | /api/dtc/clear = start one (runs in dtcTask)
    webServer.on("/api/dtc", HTTP_GET, [](AsyncWebServerRequest *request)
                 {
                     JsonDocument data;
                     const char *proto = dtcProtocolForGeneration(haldexGeneration);
                     data["generation"] = haldexGeneration;
                     data["supported"]  = (proto != nullptr);
                     data["protocol"]   = proto ? proto : "";
                     static const char *states[] = {"idle", "busy", "done", "error"};
                     data["state"]   = states[dtcState & 0x03];
                     data["action"]  = dtcLastAction == 2 ? "clear" : (dtcLastAction == 1 ? "read" : "");
                     data["message"] = dtcMessage;
                     data["ageMs"]   = dtcDoneMs ? (uint32_t)(millis() - dtcDoneMs) : 0;
                     JsonArray codes = data["codes"].to<JsonArray>();
                     if (dtcState == DTC_DONE)
                     {
                         for (uint8_t i = 0; i < dtcCount; i++)
                         {
                             JsonObject c = codes.add<JsonObject>();
                             c["code"]   = dtcList[i].code;
                             c["detail"] = dtcList[i].detail;
                         }
                     }
                     sendJSON(request, 200, data); });

    auto dtcStart = [](AsyncWebServerRequest *request, uint8_t action)
    {
        JsonDocument resp;
        if (dtcState == DTC_BUSY || dtcRequest != 0)
        {
            resp["ok"]    = false;
            resp["error"] = "A fault-code request is already running";
            sendJSON(request, 200, resp);
            return;
        }
        dtcState   = DTC_BUSY;
        dtcRequest = action;
        resp["ok"] = true;
        sendJSON(request, 200, resp);
    };
    webServer.on("/api/dtc/read", HTTP_POST, [dtcStart](AsyncWebServerRequest *request)
                 { dtcStart(request, 1); });
    webServer.on("/api/dtc/clear", HTTP_POST, [dtcStart](AsyncWebServerRequest *request)
                 { dtcStart(request, 2); });

    // V11.00.10: Gen2 SecurityAccess - forget a saved key refusal so the login is tried once more
    // (on the next TP2.0 channel). Only ever pressed on purpose from the warning on the Gen2 card.
    webServer.on("/api/kwp/gen2login/reset", HTTP_POST, [](AsyncWebServerRequest *request)
                 {
                     gen2LoginReset();
                     JsonDocument resp;
                     resp["ok"] = true;
                     sendJSON(request, 200, resp); });

    // GET /api/learn/status - returns current learn progress and table (when valid)
    webServer.on("/api/learn/status", HTTP_GET, [](AsyncWebServerRequest *request)
                 {
                     JsonDocument data;
                     data["active"]     = (bool)haldexLearnActive;
                     data["progress"]   = (uint8_t)haldexLearnStep;
                     data["tableValid"] = haldexLearnTableValid;
                     data["currentCF"]  = (uint8_t)haldexLearnCF;
                     data["currentEng"] = receivedHaldexEngagement;
                     if (haldexLearnTableValid)
                     {
                         JsonArray table = data["table"].to<JsonArray>();
                         for (uint8_t i = 0; i <= 100; i++)
                             table.add(haldexLearnTable[i]);
                     }
                     sendJSON(request, 200, data); });

    // ===== POST ENDPOINTS =====

    // POST /api/settings - save settings from web UI
    webServer.on(
        "/api/settings", HTTP_POST, [](AsyncWebServerRequest *request)
        { (void)request; }, nullptr,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
        {
            parseJSON(request, data, len, index, total, settingsIncoming);
        });

    // POST /api/backup - restore the config backup extras (V11.00.14)
    webServer.on(
        "/api/backup", HTTP_POST, [](AsyncWebServerRequest *request)
        { (void)request; }, nullptr,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
        {
            parseJSON(request, data, len, index, total, backupIncoming);
        });

    // POST /api/mode - change operating mode
    webServer.on(
        "/api/mode", HTTP_POST, [](AsyncWebServerRequest *request)
        { (void)request; }, nullptr,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
        {
            parseJSON(request, data, len, index, total, modeIncoming);
        });

    // POST /api/tune - update throttle/speed/lock arrays
    webServer.on(
        "/api/tune", HTTP_POST, [](AsyncWebServerRequest *request)
        { (void)request; }, nullptr,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
        {
            parseJSON(request, data, len, index, total, tuneIncoming);
        });

    // POST /api/learn/start - begin the learn sweep
    webServer.on("/api/learn/start", HTTP_POST, [](AsyncWebServerRequest *request)
                 {
                     if (!hasCANHaldex)
                     {
                         JsonDocument resp;
                         resp["ok"]    = false;
                         resp["error"] = "No Haldex CAN data available";
                         sendJSON(request, 200, resp);
                         return;
                     }
                     if (haldexGeneration == GEN_VOLVO_P1 && !isStandalone)
                     {
                         // V11.00.1: OEM mode doesn't edit any P1 frames yet, so a learn would
                         // only measure the car - it needs Standalone for now
                         JsonDocument resp;
                         resp["ok"]    = false;
                         resp["error"] = "Volvo P1: Learn needs Standalone for now (OEM mode doesn't edit frames yet)";
                         sendJSON(request, 200, resp);
                         return;
                     }
                     if (longLearnActive || haldexLearnActive)
                     {
                         sendError(request, 409, longLearnActive ? "Long Learn is running" : "A learn is already running");
                         return;
                     }
                     startHaldexLearn();
                     JsonDocument resp;
                     resp["ok"] = true;
                     sendJSON(request, 200, resp); });

    // ---- CAN Test card (V11.00.12) ------------------------------------------
    // GET: what each bus has received (per ID) and the last loop test result.
    webServer.on("/api/cantest", HTTP_GET, [](AsyncWebServerRequest *request)
                 {
                     JsonDocument resp;
                     const uint32_t now = millis();
                     JsonObject loop = resp["loop"].to<JsonObject>();
                     loop["ageMs"] = canLoopTestAgeMs(); // 0 = never run
                     loop["sent"] = canLoopTestSent();
                     loop["seen"] = canLoopTestSeen();
                     JsonArray buses = resp["buses"].to<JsonArray>();
                     const HdxRxStat *tables[2] = {chsRxStats, hdxRxStats};
                     const uint32_t dropped[2] = {chsRxDroppedIds, hdxRxDroppedIds};
                     const uint32_t lastMs[2] = {lastCANChassisTick, lastCANHaldexTick};
                     for (uint8_t b = 0; b < 2; b++)
                     {
                         JsonObject bus = buses.add<JsonObject>();
                         bus["lastMs"] = lastMs[b] != 0 ? (now - lastMs[b]) : 0; // 0 = nothing since boot
                         bus["moreIds"] = dropped[b];                            // frames whose ID didn't fit the table
                         JsonArray ids = bus["ids"].to<JsonArray>();
                         for (uint8_t i = 0; i < HDX_RX_STATS_MAX; i++)
                         {
                             const HdxRxStat &st = tables[b][i];
                             if (st.id == 0)
                                 continue;
                             JsonObject e = ids.add<JsonObject>();
                             char hex[10];
                             snprintf(hex, sizeof(hex), st.extd ? "%08lX" : "%03lX", (unsigned long)st.id);
                             e["id"] = hex;
                             e["count"] = st.count;
                             e["ageMs"] = now - st.lastMs;
                         }
                     }
                     sendJSON(request, 200, resp); });

    // POST: run the loop test (CHS and HDX wired together). Car must be stopped.
    webServer.on("/api/cantest/loop", HTTP_POST, [](AsyncWebServerRequest *request)
                 {
                     if (!canLoopTestStart())
                     {
                         sendError(request, 409, "Car is moving");
                         return;
                     }
                     JsonDocument resp;
                     resp["ok"] = true;
                     sendJSON(request, 200, resp); });

    // POST: forget the per-ID lists, so only what arrives from now on shows.
    webServer.on("/api/cantest/clear", HTTP_POST, [](AsyncWebServerRequest *request)
                 {
                     canRxCensusClear();
                     JsonDocument resp;
                     resp["ok"] = true;
                     sendJSON(request, 200, resp); });

    // POST /api/learn/cancel - abort an in-progress learn sweep
    webServer.on("/api/learn/cancel", HTTP_POST, [](AsyncWebServerRequest *request)
                 {
                     if (longLearnActive)
                     {
                         sendError(request, 409, "Long Learn is running - cancel it from its own card"); // would stop its sweep half-way
                         return;
                     }
                     haldexLearnCancel = true;
                     JsonDocument resp;
                     resp["ok"] = true;
                     sendJSON(request, 200, resp); });

    // POST /api/ble/forget - drop every bonded phone, new pairing code; the next phone pairs without it
    webServer.on("/api/ble/forget", HTTP_POST, [](AsyncWebServerRequest *request)
                 {
                     JsonDocument resp;
                     resp["ok"] = bleForgetBonds();
                     sendJSON(request, 200, resp); });

    // POST /api/learn/clear - discard the stored learn table and revert to formula
    webServer.on("/api/learn/clear", HTTP_POST, [](AsyncWebServerRequest *request)
                 {
                     if (longLearnActive || haldexLearnActive)
                     {
                         sendError(request, 409, "A learn is running");
                         return;
                     }
                     haldexLearnTableValid = false;
                     memset(haldexLearnTable, 0, sizeof(haldexLearnTable));
                     JsonDocument resp;
                     resp["ok"] = true;
                     sendJSON(request, 200, resp); });

    // GET /api/longlearn/status - phase/progress, per-block verdicts, sweep log
    // and live sweep position. Results stay available after the run finishes
    // (until the next run) so the UI can show and export them.
    webServer.on("/api/longlearn/status", HTTP_GET, [](AsyncWebServerRequest *request)
                 {
                     JsonDocument data;
                     data["active"]     = (bool)longLearnActive;
                     data["phase"]      = (uint8_t)longLearnPhase;
                     data["sweepIdx"]   = (uint8_t)longLearnSweepIdx;
                     data["sweepTotal"] = (uint8_t)longLearnSweepTotal;
                     data["currentBit"] = (int)longLearnCurrentBit;
                     data["generation"] = longLearnGeneration;
                     data["testAll"]    = longLearnTestAll;
                     data["reachMin"]   = LL_REACH_MIN;
                     data["fixHunting"]    = fixHunting;
                     data["fixHuntingSet"] = longLearnFixHuntingSet; // the retry turned it on and it was kept
                     data["bpkNow"]        = bpkCeilingNm;             // 0CQ torque ceiling (Nm)
                     data["bpkStart"]      = longLearnBpkStart;
                     data["bpkAdjusted"]   = longLearnBpkAdjusted;     // the retry's ceiling walk changed it and it was kept
                     data["isStandalone"]  = isStandalone;
                     data["runStandalone"] = longLearnWasStandalone;
                     data["failReason"]    = longLearnFailReason;
                     data["interaction"]   = longLearnInteraction;
                     data["followBrake"]     = followBrake;
                     data["followHandbrake"] = followHandbrake;
                     data["followABS"]       = followABS;
                     JsonArray ptCF = data["ptCF"].to<JsonArray>(); // CF of each sweeps[].pts entry
                     JsonArray refPts = data["refPts"].to<JsonArray>(); // all-ours quick read
                     for (uint8_t k = 0; k < LL_NPTS; k++)
                     {
                         ptCF.add(LL_POINT_CF[k]);
                         refPts.add(longLearnRefPts[k]);
                     }
                     auto putState = [](JsonObject o, const LLCarState &st)
                     {
                         o["flags"] = st.flags; // LLST_* bits
                         o["speed"] = st.speed;
                         o["rpm"]   = st.rpm;
                     };
                     putState(data["startState"].to<JsonObject>(), longLearnStartState);
                     auto hex8 = [](const uint8_t *d, uint8_t n, char *out)
                     {
                         // "AA BB CC ..." (out needs 3 x 8 bytes)
                         out[0] = 0;
                         if (n > 8)
                             n = 8;
                         for (uint8_t k = 0; k < n; k++)
                             sprintf(out + k * 3, "%02X ", d[k]);
                         if (n > 0)
                             out[n * 3 - 1] = 0; // drop the last space
                     };
                     const uint32_t endMs = longLearnActive ? millis() : longLearnEndMs;
                     data["elapsedS"]   = (longLearnStartMs && endMs >= longLearnStartMs) ? (endMs - longLearnStartMs) / 1000 : 0;
                     // live sweep position (mirrors /api/learn/status)
                     data["cf"]   = (uint8_t)haldexLearnCF;
                     data["eng"]  = receivedHaldexEngagement;
                     data["step"] = (uint8_t)haldexLearnStep;

                     auto putScore = [](JsonObject o, const LearnScore &sc)
                     {
                         o["reach"]      = sc.reach;
                         o["maxStep"]    = sc.maxStep;
                         o["engageCF"]   = sc.engageCF;
                         o["engageJump"] = sc.engageJump;
                         o["score"]      = sc.score;
                         o["smooth"]     = sc.smooth;
                     };
                     if (longLearnBaselineValid)
                         putScore(data["baseline"].to<JsonObject>(), longLearnBaseline);
                     if (longLearnFinalValid)
                         putScore(data["final"].to<JsonObject>(), longLearnFinal);

                     // Per-block verdicts for the generation the run belongs to
                     // (or the current generation when idle) with live enable state.
                     const int gi = (longLearnPhase != LL_IDLE) ? (int)longLearnGenIdx : frameEditGenIdx(haldexGeneration);
                     char maskHex[20] = "";
                     if (gi >= 0)
                     {
                         const uint64_t m = activeFrameEditMask()[gi];
                         snprintf(maskHex, sizeof(maskHex), "0x%08lX", (unsigned long)(m & 0xFFFFFFFFULL));
                         JsonArray blocks = data["blocks"].to<JsonArray>();
                         for (uint16_t i = 0; i < frameEditBlockCount; i++)
                         {
                             if (frameEditBlocks[i].genIdx != (uint8_t)gi)
                                 continue;
                             JsonObject o = blocks.add<JsonObject>();
                             o["bit"]     = frameEditBlocks[i].bit;
                             o["name"]    = frameEditBlocks[i].name;
                             o["canId"]   = frameEditBlocks[i].canId;
                             o["enabled"] = (bool)((m >> frameEditBlocks[i].bit) & 0x1ULL);
                             o["def"]     = (bool)((frameEditMaskDefaults[gi] >> frameEditBlocks[i].bit) & 0x1ULL);
                             o["core"]    = (bool)((frameEditLockDriven[gi] >> frameEditBlocks[i].bit) & 0x1ULL);
                             const uint8_t bit = frameEditBlocks[i].bit;
                             o["result"]  = longLearnBlockResult[bit];
                             if (bit < CAR_BLOCK_SLOTS && longLearnBlockResult[bit] >= LLB_NEEDED)
                             {
                                 // what the swap test saw: the car's frame vs ours, and the car's state
                                 char hx[26];
                                 o["carSent"] = longLearnCarSent[bit];
                                 if (longLearnCarSent[bit])
                                 {
                                     hex8(longLearnCarBytes[bit], longLearnCarDlc[bit], hx);
                                     o["car"] = hx;
                                 }
                                 hex8(longLearnOurBytes[bit], 8, hx);
                                 o["ours"] = hx;
                                 putState(o["state"].to<JsonObject>(), longLearnTestState[bit]);
                             }
                         }
                     }
                     data["mask"] = maskHex;

                     JsonArray sweeps = data["sweeps"].to<JsonArray>();
                     for (uint8_t i = 0; i < longLearnSweepCount; i++)
                     {
                         const LongLearnSweep &e = longLearnSweeps[i];
                         JsonObject o = sweeps.add<JsonObject>();
                         o["kind"]    = e.kind;
                         o["bit"]     = e.bit;
                         o["verdict"] = e.verdict;
                         o["fixHunting"] = e.fixHunting;
                         o["bpk"]        = e.bpkNm;
                         if (e.kind == LLS_POINTS || e.kind == LLS_BLOCK || e.kind == LLS_UNLISTED)
                         {
                             JsonArray p = o["pts"].to<JsonArray>();
                             for (uint8_t k = 0; k < LL_NPTS; k++)
                                 p.add(e.pts[k]);
                         }
                         putScore(o, e.s);
                     }

                     // Car IDs with no block (step 4) and the ones held back from the Haldex
                     data["unlistedResult"]   = longLearnUnlistedResult; // LLU_*
                     data["unlistedSeen"]     = longLearnUnlistedSeen;
                     data["unlistedOverflow"] = longLearnUnlistedOverflowSeen;
                     JsonArray seen = data["unlistedIds"].to<JsonArray>();
                     const uint8_t nSeen = (longLearnPhase != LL_IDLE) ? longLearnUnlistedSeen : 0;
                     for (uint8_t i = 0; i < nSeen && i < UNLISTED_MAX; i++)
                     {
                         char idTxt[12];
                         snprintf(idTxt, sizeof(idTxt), unlistedFrames[i].extd ? "0x%08lX" : "0x%03lX", (unsigned long)unlistedFrames[i].id);
                         seen.add(idTxt);
                     }
                     JsonArray hits = data["unlistedHits"].to<JsonArray>();
                     for (uint8_t h = 0; h < longLearnUnlistedHitCount; h++)
                     {
                         const LLUnlistedHit &u = longLearnUnlistedHits[h];
                         JsonObject o = hits.add<JsonObject>();
                         if (u.id == 0xFFFFFFFFUL)
                         {
                             o["id"] = "overflow"; // one of the IDs beyond the first UNLISTED_MAX
                             continue;
                         }
                         char idTxt[12], hx[26];
                         snprintf(idTxt, sizeof(idTxt), u.extd ? "0x%08lX" : "0x%03lX", (unsigned long)u.id);
                         o["id"] = idTxt;
                         hex8(u.data, u.dlc, hx);
                         o["car"] = hx;
                     }
                     JsonArray held = data["heldBack"].to<JsonArray>();
                     for (uint8_t h = 0; h < HELD_BACK_MAX; h++)
                     {
                         if (heldBackIds[h] == 0)
                             continue;
                         char idTxt[12];
                         const bool ext = heldBackIds[h] & HELD_BACK_EXTD;
                         snprintf(idTxt, sizeof(idTxt), ext ? "0x%08lX" : "0x%03lX", (unsigned long)(heldBackIds[h] & ~HELD_BACK_EXTD));
                         held.add(idTxt);
                     }
                     sendJSON(request, 200, data); });

    // POST /api/longlearn/clearheld - let every car ID through to the Haldex again
    webServer.on("/api/longlearn/clearheld", HTTP_POST, [](AsyncWebServerRequest *request)
                 {
                     if (longLearnActive)
                     {
                         JsonDocument resp;
                         resp["ok"] = false;
                         resp["error"] = "Long Learn is running";
                         sendJSON(request, 200, resp);
                         return;
                     }
                     memset(heldBackIds, 0, sizeof(heldBackIds)); // saved by writeEEP
                     JsonDocument resp;
                     resp["ok"] = true;
                     sendJSON(request, 200, resp); });

    // POST /api/longlearn/start - optional body {"testAll": bool}
    webServer.on(
        "/api/longlearn/start", HTTP_POST, [](AsyncWebServerRequest *request)
        {
            if (request->contentLength() == 0)
                longLearnStartIncoming(request, String("{}")); // body-less start
        },
        nullptr,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
        {
            parseJSON(request, data, len, index, total, longLearnStartIncoming);
        });

    // POST /api/longlearn/cancel - abort; previous mask/floor/table are restored
    webServer.on("/api/longlearn/cancel", HTTP_POST, [](AsyncWebServerRequest *request)
                 {
                     longLearnCancel = true;
                     haldexLearnCancel = true; // also stops the sweep in progress
                     JsonDocument resp;
                     resp["ok"] = true;
                     sendJSON(request, 200, resp); });

    // NOTE: route registration order matters. ESPAsyncWebServer's URL matcher
    // accepts a registered "/api/wifi" handler for any URL that starts with
    // "/api/wifi/" (see WebHandlerImpl.h canHandle). The more-specific routes
    // must be registered BEFORE "/api/wifi"
    // or POSTs to "/api/wifi/ssid" get missed

    // ---- Bridge mode (PR #39, louij2): home-network STA alongside the AP ----

    // GET /api/wifi/scan - nearby networks for the home-WiFi picker.
    // The scan is ASYNC: the first call starts it and answers {"scanning":true};
    // the page polls until the list comes back. A blocking scan here would sit
    // inside the async_tcp task for 2-3 s, which is exactly the task that has
    // to keep serving everyone else. One radio is shared between AP and STA,
    // so the AP drops off-channel for the scan's duration either way - hence
    // this is a manual action behind the SSID field, never automatic.
    webServer.on("/api/wifi/scan", HTTP_GET, [](AsyncWebServerRequest *request)
                 {
                     JsonDocument resp;
                     int16_t n = WiFi.scanComplete();
                     if (n == WIFI_SCAN_RUNNING)
                     {
                         resp["scanning"] = true;
                     }
                     else if (n == WIFI_SCAN_FAILED || n < 0)
                     {
                         // nothing in progress (or a previous one failed): kick one off
                         WiFi.scanNetworks(true /*async*/, false /*hidden*/);
                         resp["scanning"] = true;
                     }
                     else
                     {
                         resp["scanning"] = false;
                         JsonArray nets = resp["networks"].to<JsonArray>();
                         // de-duplicate by SSID keeping the strongest, then sort strongest first
                         struct Net { String ssid; int32_t rssi; bool secure; };
                         std::vector<Net> list;
                         for (int16_t i = 0; i < n; i++)
                         {
                             String ssid = WiFi.SSID(i);
                             if (ssid.length() == 0) continue; // hidden - type it in instead
                             int32_t rssi = WiFi.RSSI(i);
                             bool merged = false;
                             for (auto &e : list)
                                 if (e.ssid == ssid) { if (rssi > e.rssi) e.rssi = rssi; merged = true; break; }
                             if (!merged) list.push_back({ssid, rssi, WiFi.encryptionType(i) != WIFI_AUTH_OPEN});
                         }
                         std::sort(list.begin(), list.end(), [](const Net &a, const Net &b) { return a.rssi > b.rssi; });
                         for (auto &e : list)
                         {
                             JsonObject o = nets.add<JsonObject>();
                             o["ssid"] = e.ssid;
                             o["rssi"] = e.rssi;
                             o["secure"] = e.secure;
                         }
                         WiFi.scanDelete(); // next GET starts a fresh scan
                     }
                     sendJSON(request, 200, resp); });

    // POST /api/wifi/sta/reset - forget the home network, back to AP only
    webServer.on("/api/wifi/sta/reset", HTTP_POST, [](AsyncWebServerRequest *request)
                 {
                     resetWifiSta();
                     JsonDocument resp;
                     resp["ok"] = true;
                     sendJSON(request, 200, resp); });

    // GET /api/wifi/sta - bridge-mode status. Password is write-only, never returned.
    webServer.on("/api/wifi/sta", HTTP_GET, [](AsyncWebServerRequest *request)
                 {
                     otaNoteWebActivity();
                     JsonDocument resp;
                     resp["ssid"] = wifiStaSsid;
                     resp["passwordSet"] = (strlen(wifiStaPassword) >= 8);
                     resp["connected"] = wifiStaConnected;
                     resp["ip"] = wifiStaIP;
                     if (wifiStaConnected) resp["rssi"] = WiFi.RSSI();
                     else resp["rssi"] = nullptr;
                     sendJSON(request, 200, resp); });

    // POST /api/wifi/sta {ssid, password} - set (empty ssid = disable) and restart AP(+STA)
    webServer.on(
        "/api/wifi/sta", HTTP_POST, [](AsyncWebServerRequest *request)
        { (void)request; }, nullptr,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
        {
            parseJSON(request, data, len, index, total, [](AsyncWebServerRequest *req, const String &body)
                      {
                JsonDocument d;
                JsonDocument resp;
                auto fail = [&](const char *why) { resp["ok"] = false; resp["error"] = why; sendJSON(req, 400, resp); };
                if (deserializeJson(d, body) != DeserializationError::Ok) { fail("Invalid JSON"); return; }
                if (!d["ssid"].is<const char *>()) { fail("Missing 'ssid' field"); return; }
                const char *newSsid = d["ssid"];
                const size_t ssidLen = strlen(newSsid);
                if (ssidLen > 32) { fail("SSID too long (max 32)"); return; }
                for (size_t i = 0; i < ssidLen; ++i)
                {
                    unsigned char c = (unsigned char)newSsid[i];
                    if (c < 0x20 || c > 0x7E) { fail("SSID must be printable ASCII"); return; }
                }
                const char *newPwd = d["password"].is<const char *>() ? d["password"].as<const char *>() : "";
                const size_t pwdLen = strlen(newPwd);
                if (pwdLen > 0 && pwdLen < 8) { fail("Password must be at least 8 characters, or empty for an open network"); return; }
                if (pwdLen >= 65) { fail("Password too long (max 64)"); return; }
                memset(wifiStaSsid, 0, sizeof(wifiStaSsid));
                strncpy(wifiStaSsid, newSsid, sizeof(wifiStaSsid) - 1);
                memset(wifiStaPassword, 0, sizeof(wifiStaPassword));
                if (pwdLen > 0) strncpy(wifiStaPassword, newPwd, sizeof(wifiStaPassword) - 1);
                rebootWiFi = true; // restart AP(+STA) with the new credentials
                resp["ok"] = true;
                resp["ssid"] = wifiStaSsid;
                sendJSON(req, 200, resp); });
        });

    // POST /api/wifi/ssid/reset - restore factory SSID and restart AP
    webServer.on("/api/wifi/ssid/reset", HTTP_POST, [](AsyncWebServerRequest *request)
                 {
                     resetWifiSsid();
                     JsonDocument resp;
                     resp["ok"] = true;
                     resp["ssid"] = wifiSsid;
                     sendJSON(request, 200, resp); });

    // GET /api/wifi/ssid - return current AP SSID
    webServer.on("/api/wifi/ssid", HTTP_GET, [](AsyncWebServerRequest *request)
                 {
                     JsonDocument resp;
                     resp["ssid"] = wifiSsid;
                     resp["default"] = wifiHostNameDefault;
                     sendJSON(request, 200, resp); });

    // POST /api/wifi/ssid - change AP SSID; AP restarts immediately
    webServer.on(
        "/api/wifi/ssid", HTTP_POST, [](AsyncWebServerRequest *request)
        { (void)request; }, nullptr,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
        {
            parseJSON(request, data, len, index, total, [](AsyncWebServerRequest *req, const String &body)
                      {
                JsonDocument d;
                if (deserializeJson(d, body) != DeserializationError::Ok)
                {
                    JsonDocument resp; resp["ok"] = false; resp["error"] = "Invalid JSON";
                    sendJSON(req, 400, resp); return;
                }
                if (!d["ssid"].is<const char *>())
                {
                    JsonDocument resp; resp["ok"] = false; resp["error"] = "Missing 'ssid' field";
                    sendJSON(req, 400, resp); return;
                }
                const char *newSsid = d["ssid"];
                const size_t ssidLen = strlen(newSsid);
                if (ssidLen < 1)
                {
                    JsonDocument resp; resp["ok"] = false; resp["error"] = "SSID must be at least 1 character";
                    sendJSON(req, 400, resp); return;
                }
                if (ssidLen > 32)
                {
                    JsonDocument resp; resp["ok"] = false; resp["error"] = "SSID too long (max 32)";
                    sendJSON(req, 400, resp); return;
                }
                // basic printable-ASCII guard (reject control chars / non-ASCII to keep AP discoverable)
                for (size_t i = 0; i < ssidLen; ++i)
                {
                    unsigned char c = (unsigned char)newSsid[i];
                    if (c < 0x20 || c > 0x7E)
                    {
                        JsonDocument resp; resp["ok"] = false; resp["error"] = "SSID must be printable ASCII";
                        sendJSON(req, 400, resp); return;
                    }
                }
                memset(wifiSsid, 0, sizeof(wifiSsid));
                strncpy(wifiSsid, newSsid, sizeof(wifiSsid) - 1);
                rebootWiFi = true; // restart AP with new SSID
                JsonDocument resp;
                resp["ok"] = true;
                resp["ssid"] = wifiSsid;
                sendJSON(req, 200, resp); });
        });

    // POST /api/wifi/reset - clear password and restart AP as open network
    webServer.on("/api/wifi/reset", HTTP_POST, [](AsyncWebServerRequest *request)
                 {
                     resetWifiPassword();
                     JsonDocument resp;
                     resp["ok"] = true;
                     sendJSON(request, 200, resp); });

    // GET /api/wifi - return whether a WiFi password is currently set
    webServer.on("/api/wifi", HTTP_GET, [](AsyncWebServerRequest *request)
                 {
                     JsonDocument resp;
                     resp["passwordSet"] = (strlen(wifiPassword) >= 8);
                     sendJSON(request, 200, resp); });

    // POST /api/wifi - set or clear WiFi password; AP restarts immediately
    webServer.on(
        "/api/wifi", HTTP_POST, [](AsyncWebServerRequest *request)
        { (void)request; }, nullptr,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
        {
            parseJSON(request, data, len, index, total, [](AsyncWebServerRequest *req, const String &body)
                      {
                JsonDocument d;
                if (deserializeJson(d, body) != DeserializationError::Ok)
                {
                    JsonDocument resp; resp["ok"] = false; resp["error"] = "Invalid JSON";
                    sendJSON(req, 400, resp); return;
                }
                if (!d["password"].is<const char *>())
                {
                    JsonDocument resp; resp["ok"] = false; resp["error"] = "Missing 'password' field";
                    sendJSON(req, 400, resp); return;
                }
                const char *newPwd = d["password"];
                const size_t pwdLen = strlen(newPwd);
                if (pwdLen > 0 && pwdLen < 8)
                {
                    JsonDocument resp; resp["ok"] = false; resp["error"] = "Password must be at least 8 characters or empty";
                    sendJSON(req, 400, resp); return;
                }
                if (pwdLen >= 65)
                {
                    JsonDocument resp; resp["ok"] = false; resp["error"] = "Password too long (max 64)";
                    sendJSON(req, 400, resp); return;
                }
                memset(wifiPassword, 0, sizeof(wifiPassword));
                if (pwdLen > 0) strncpy(wifiPassword, newPwd, sizeof(wifiPassword) - 1);
                rebootWiFi = true; // restart AP with new credentials
                JsonDocument resp;
                resp["ok"] = true;
                resp["passwordSet"] = (pwdLen >= 8);
                sendJSON(req, 200, resp); });
        });
}
