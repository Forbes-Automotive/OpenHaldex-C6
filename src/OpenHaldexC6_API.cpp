#include <OpenHaldexC6_API.h>
#include <OpenHaldexC6_UDS.h>
#include <OpenHaldexC6_Calculations.h>

#include <cstring>

static int getCPUUsagePercent() // helper function to calculate CPU usage percentage based on FreeRTOS task run time stats
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

// helper function to send over JSON data to the ESP
static void sendJSON(AsyncWebServerRequest *request, int code, const JsonDocument &data)
{
    String out;
    serializeJson(data, out);
    request->send(code, "application/json", out);
}

// helper function to reserve space (and delete) for incoming data
static void parseJSON(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total, void (*done)(AsyncWebServerRequest *, const String &))
{
    if (index == 0)
    {
        request->_tempObject = new String();              // create a new String to hold the incoming data, stored in the request's temp object pointer
        ((String *)request->_tempObject)->reserve(total); // reserve space for the incoming data to optimize memory usage and prevent fragmentation
    }

    String *body = (String *)request->_tempObject; // get the pointer to the String object from the request's temp object pointer
    body->concat((const char *)data, len);         // append the incoming data chunk to the String object

    if (index + len == total)
    {
        String bodyString = *body;      // create a copy of the body string to pass to the done callback, since we'll be deleting the original String object to free memory
        delete body;                    // delete the original String object to free memory, since we have a copy of the data in bodyString for the callback
        request->_tempObject = nullptr; // clear the temp object pointer to avoid dangling pointer issues
        done(request, bodyString);      // call the done callback with the request and the complete body string
    }
}

// parse the current INCOMING request for status: WebServer asks, this delivers
static void statusOutgoing(AsyncWebServerRequest *request)
{
    JsonDocument data;
    const bool chassisOk = hasCANChassis;
    const bool haldexOk = hasCANHaldex;

    data["mode"] = state.mode;

    if (chassisOk) // if chassis CAN ok, set related values
    {
        data["speed"] = received_vehicle_speed;
        data["throttle"] = int(received_pedal_value);
        data["asrOn"] = !asrForceModeFlag;
        data["tcOn"] = !tcForceModeFlag;
        data["rpm"] = received_vehicle_rpm;
        data["boost"] = received_vehicle_boost;
    }
    else // if chassis CAN not ok, set related values to
    {
        data["speed"] = nullptr;
        data["throttle"] = nullptr;
        data["asrOn"] = nullptr;
        data["tcOn"] = nullptr;
        data["rpm"] = nullptr;
        data["boost"] = nullptr;
    }

    data["brakeIn"] = brakeSignalActive;
    data["brakeOut"] = brakeActive;
    data["handbrakeIn"] = handbrakeSignalActive;
    data["handbrakeOut"] = handbrakeActive;

    data["lockTarget"] = int(lock_target);

    if (haldexOk) // if haldex CAN ok set related values
    {
        data["lockActual"] = received_haldex_engagement;
        data["haldexState"] = received_haldex_state;
        data["haldexEngagement"] = received_haldex_engagement;
        data["haldexEngagementRaw"] = received_haldex_engagement_raw;
        data["clutch1Report"] = received_report_clutch1;
        data["clutch2Report"] = received_report_clutch2;
        data["tempProtection"] = received_temp_protection;
        data["couplingOpen"] = received_coupling_open;
        data["speedLimit"] = received_speed_limit;
    }
    else // if haldex CAN not ok, set related values to "--"
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
    }

    data["chassisCAN"] = chassisOk;
    data["haldexCAN"] = haldexOk;
    data["busFailure"] = isBusFailure;
    data["lastChassisMs"] = lastCANChassisTick > 0 ? (millis() - lastCANChassisTick) : 0;
    data["lastHaldexMs"] = lastCANHaldexTick > 0 ? (millis() - lastCANHaldexTick) : 0;

    data["uptimeMs"] = millis();
    data["freeHeap"] = ESP.getFreeHeap();

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
    data["forceModeValue"] = forceModeValue;
    data["disengageUnderSpeed"] = disengageUnderSpeed;
    data["disengageAboveSpeed"] = disengageAboveSpeed;
    data["disableThrottle"] = disableThrottle;
    data["mode"] = lastMode;
    data["lockReleaseRatePerSec"] = lockReleaseRatePerSec;
    data["FW_VERSION"] = FW_VERSION;

    // bools
    data["disableController"] = disableController;
    data["isStandalone"] = isStandalone;
    data["tcForceMode"] = tcForceMode;
    data["extButtonForceMode"] = extBtnForceMode;

    data["disableOnboardButton"] = disableOnboardButton;
    data["disableExternalButton"] = disableExternalButton;

    data["followBrake"] = followBrake;
    data["invertBrake"] = invertBrake;
    data["followHandbrake"] = followHandbrake;
    data["invertHandbrake"] = invertHandbrake;

    data["broadcastOpenHaldexOverCAN"] = broadcastOpenHaldexOverCAN;

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

    sendJSON(request, 200, data);
}

// manage settings (saved from Web, handled here): ESP sends, this handles
static void settingsIncoming(AsyncWebServerRequest *request, const String &body)
{
    JsonDocument data;
    if (deserializeJson(data, body) != DeserializationError::Ok)
    {
        DEBUG("Invalid JSON");
        return;
    }

    if (data["haldexGeneration"].is<uint8_t>())
    {
        int generation = data["haldexGeneration"];
        if (generation == 1 || generation == 2 || generation == 4 || generation == 5 || generation == 41)
        {
            haldexGeneration = (uint8_t)generation;
            lastMode = generation;
        }
    }

    if (data["forceModeValue"].is<uint8_t>())
    {
        forceModeValue = (uint8_t)data["forceModeValue"];
        DEBUG("forcemodevalue: %d", forceModeValue);
    }

    if (data["disengageUnderSpeed"].is<uint16_t>())
    {
        uint16_t value = data["disengageUnderSpeed"];
        disengageUnderSpeed = constrain(value, 0, 300);
    }

    if (data["disengageAboveSpeed"].is<uint16_t>())
    {
        uint16_t value = data["disengageAboveSpeed"];
        disengageAboveSpeed = constrain(value, 0, 300);
    }

    if (data["disableThrottle"].is<uint8_t>())
    {
        uint8_t value = data["disableThrottle"];
        disableThrottle = constrain(value, 0, 100);
        state.pedal_threshold = disableThrottle;
    }

    if (data["disableController"].is<bool>())
    {
        disableController = data["disableController"];
        if (disableController)
        {
            state.mode = MODE_STOCK;
            lastMode = 0;
        }
        if (!disableController && analyzerMode)
        {
            setAnalyzerMode(false);
        }
    }

    if (data["isStandalone"].is<bool>())
    {
        isStandalone = data["isStandalone"];

        if (!isStandalone)
        {
            vTaskSuspend(handle_frames1000);
            vTaskSuspend(handle_frames200);
            vTaskSuspend(handle_frames100);
            vTaskSuspend(handle_frames25);
            vTaskSuspend(handle_frames20);
            vTaskSuspend(handle_frames10);
        }
        else
        {
            vTaskResume(handle_frames1000);
            vTaskResume(handle_frames200);
            vTaskResume(handle_frames100);
            vTaskResume(handle_frames25);
            vTaskResume(handle_frames20);
            vTaskResume(handle_frames10);
        }
    }

    if (data["analyzerMode"].is<bool>())
    {
        setAnalyzerMode(data["analyzerMode"]);
    }

    if (data["tcForceMode"].is<bool>())
    {
        tcForceMode = data["tcForceMode"];
    }

    if (data["extButtonForceMode"].is<bool>())
    {
        extBtnForceMode = data["extButtonForceMode"];
    }

    if (data["disableOnboardButton"].is<bool>())
    {
        disableOnboardButton = data["disableOnboardButton"];
    }

    if (data["disableExternalButton"].is<bool>())
    {
        disableExternalButton = data["disableExternalButton"];
    }

    if (data["followBrake"].is<bool>())
    {
        followBrake = data["followBrake"];
    }

    if (data["invertBrake"].is<bool>())
    {
        invertBrake = data["invertBrake"];
    }

    if (data["followHandbrake"].is<bool>())
    {
        followHandbrake = data["followHandbrake"];
    }

    if (data["invertHandbrake"].is<bool>())
    {
        invertHandbrake = data["invertHandbrake"];
    }

    if (data["broadcastOpenHaldexOverCAN"].is<bool>())
    {
        broadcastOpenHaldexOverCAN = data["broadcastOpenHaldexOverCAN"];
    }

    JsonDocument resp;
    resp["ok"] = true;
    sendJSON(request, 200, resp);
}

// manage mode (saved from Web, handled here): ESP sends, this handles
static void modeIncoming(AsyncWebServerRequest *request, const String &body)
{
    JsonDocument data;
    if (deserializeJson(data, body) != DeserializationError::Ok)
    {
        DEBUG("Invalid JSON");
        return;
    }

    if (data["mode"].is<uint8_t>())
    {
        if (!disableController)
        {
            if (isStandalone && (openhaldex_mode_t)data["mode"] == 0)
            {
                state.mode = (openhaldex_mode_t)lastMode;
            }
            else
            {
                state.mode = (openhaldex_mode_t)data["mode"];
            }
            lastMode = state.mode;
        }
    }
}

// manage tune (saved from Web, handled here): ESP sends, this handles
static void tuneIncoming(AsyncWebServerRequest *request, const String &body)
{
    JsonDocument data;
    if (deserializeJson(data, body) != DeserializationError::Ok)
    {
        DEBUG("Invalid JSON");
        return;
    }

    JsonArray speedArrayJSON = data["speedArray"].as<JsonArray>();
    JsonArray throttleArrayJSON = data["throttleArray"].as<JsonArray>();
    JsonArray lockArrayJSON = data["lockArray"].as<JsonArray>();

    if (speedArrayJSON.size() != speedArrayCount || throttleArrayJSON.size() != throttleArrayCount)
    {
        DEBUG("Invalid Array Length");
        return;
    }

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
        if (throttleRow.size() != throttleArrayCount)
        {
            DEBUG("Invalid lock array");
            return;
        }
        for (uint8_t speed = 0; speed < speedArrayCount; speed++)
        {
            lockArray[throttle][speed] = (uint8_t)throttleRow[speed];
        }
    }

    JsonDocument resp;
    resp["ok"] = true;
    sendJSON(request, 200, resp);
}

// setup webserver function
void setupWebServer()
{
    if (!LittleFS.begin(false))
    {
        DEBUG("LittleFS mount failed!"); // littleFS didn't mount
        // add a warning function - flashing LED?
        return;
    }
    DEBUG("LittleFS mounted successfully");

    // when "/" is requested, send index.html page
    webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
                 { request->send(LittleFS, "/index.html", "text/html"); });

    webServer.serveStatic("/", LittleFS, "/").setDefaultFile("index.html"); // dunno - same as above?

    webServer.begin(); // begin the webServer
    DEBUG("Web server started");

    if (MDNS.begin("openhaldex"))
    {
        MDNS.addService("http", "tcp", 80);
        DEBUG("mDNS responder started: openhaldex.local");
    }
}

// setup main section for handling requests
void setupAPI()
{
    // ===== GET ENDPOINTS =====

    // GET /api/settings - retrieve all current settings
    webServer.on("/api/settings", HTTP_GET, [](AsyncWebServerRequest *request)
                 { settingsOutgoing(request); });

    // GET /api/dashboard - retrieve live status data (polled regularly by JS)
    webServer.on("/api/dashboard", HTTP_GET, [](AsyncWebServerRequest *request)
                 { statusOutgoing(request); });

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

                     OpenHaldexC6::UDS uds;
                     uint8_t buffer[256];
                     size_t bufferLen = sizeof(buffer);

                     if (!uds.readDataByIdentifier(requestId, responseId, did, buffer, bufferLen))
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

    // GET /api/learn/status - returns current learn progress and table (when valid)
    webServer.on("/api/learn/status", HTTP_GET, [](AsyncWebServerRequest *request)
                 {
                     JsonDocument data;
                     data["active"]     = (bool)haldexLearnActive;
                     data["progress"]   = (uint8_t)haldexLearnStep;
                     data["tableValid"] = haldexLearnTableValid;
                     data["currentCF"]  = (uint8_t)haldexLearnCF;
                     data["currentEng"] = received_haldex_engagement;
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
                     startHaldexLearn();
                     JsonDocument resp;
                     resp["ok"] = true;
                     sendJSON(request, 200, resp); });

    // POST /api/learn/cancel - abort an in-progress learn sweep
    webServer.on("/api/learn/cancel", HTTP_POST, [](AsyncWebServerRequest *request)
                 {
                     haldexLearnCancel = true;
                     JsonDocument resp;
                     resp["ok"] = true;
                     sendJSON(request, 200, resp); });

    // POST /api/learn/clear - discard the stored learn table and revert to formula
    webServer.on("/api/learn/clear", HTTP_POST, [](AsyncWebServerRequest *request)
                 {
                     haldexLearnTableValid = false;
                     memset(haldexLearnTable, 0, sizeof(haldexLearnTable));
                     JsonDocument resp;
                     resp["ok"] = true;
                     sendJSON(request, 200, resp); });
}
