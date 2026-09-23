#include <OpenHaldexC6_API.h>
#include <OpenHaldexC6_UDS.h>
#include <OpenHaldexC6_Calculations.h>
#include <OpenHaldexC6_WiFi.h>
#include <OpenHaldexC6_OTA.h> // otaNoteWebActivity()

#include <cstring>
#include <vector>    // /api/wifi/scan de-dup
#include <algorithm> // std::sort

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

    // Ext-button force flag is driven by the external button
    data["extButtonActive"] = extButtonForceModeFlag;

    if (chassisOk) // if chassis CAN ok, set related values
    {
        data["speed"] = received_vehicle_speed;
        data["throttle"] = int(received_pedal_value);
        data["asrOn"] = !asrForceModeFlag;
        data["tcOn"] = !tcForceModeFlag;
        data["hazardActive"] = hazardForceModeFlag;
        data["rpm"] = received_vehicle_rpm;
        data["boost"] = received_vehicle_boost;
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

    data["brakeIn"] = brakeSignalActive;
    data["brakeOut"] = brakeActive;
    data["handbrakeIn"] = handbrakeSignalActive;
    data["handbrakeOut"] = handbrakeActive;

    // Steering angle + health. Only gens with a steering source (2/4/50/52).
    // Stale/absent/unsupported -> unhealthy + null (shown as "--"). Magnitude only.
    {
        const bool steerSupported = (haldexGeneration == 2 || haldexGeneration == 4 ||
                                     haldexGeneration == 50 || haldexGeneration == 52);
        const bool steerHealthy = steerSupported && chassisOk && received_steering_ms != 0 &&
                                  (millis() - received_steering_ms) <= steeringStaleMs;
        data["steeringHealthy"] = steerHealthy;
        if (steerHealthy)
            data["steeringAngle"] = (int)(fabsf(received_steering_angle) + 0.5f);
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
    if (steering_scale_is_active())
    {
        data["steeringScaleActive"] = true;
        data["lockRequested"] = steering_scale_requested_pct();
        data["lockScaled"] = steering_scale_result_pct();
    }
    else
    {
        data["steeringScaleActive"] = false;
        data["lockRequested"] = nullptr;
        data["lockScaled"] = nullptr;
    }

    // CAN-decoded brake / handbrake state.
    //   Brake:     PQ Motor_2 MO2_BLS (Gen2/4/51), MQB ESP_05 ESP_Fahrer_bremst (Gen5).
    //   Handbrake: PQ Kombi_1 KO1_Handbremse (Gen2/4/51), MQB Kombi_01 KBI_Handbremse (Gen5).
    // Gen1 (1J0) and the GM/Ford variants have no CAN handbrake decode -> null ("--").
    if (chassisOk)
    {
        data["brakeFromCAN"] = brakeFromCAN;
        const bool hbFromCANSupported = (haldexGeneration == 2 || haldexGeneration == 4 ||
                                         haldexGeneration == 50 || haldexGeneration == 51 ||
                                         haldexGeneration == 52);
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
    }

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

        if (haldexGeneration == 41)
        {
            JsonObject gen41 = data["gen41"].to<JsonObject>();
            JsonObject sec = gen41["secAxle"].to<JsonObject>();
            sec["statusRaw"] = received_sec_axle_status_raw;
            sec["torqueNm"] = received_sec_axle_torque_nm;
            sec["clutchState"] = received_sec_axle_clutch_state;
            sec["active"] = received_sec_axle_active;
            sec["fdcmHealthy"] = received_sec_axle_fdcm_healthy;
            sec["arc"] = received_sec_axle_arc;
            JsonObject rear = gen41["rearAxle"].to<JsonObject>();
            rear["statusFlags"] = received_rear_axle_status_flags;
            rear["metricA"] = received_rear_axle_metric_a;
            rear["metricB"] = received_rear_axle_metric_b;
            JsonObject hb = gen41["heartbeat"].to<JsonObject>();
            hb["aliveBus0"] = received_haldex_alive_bus0;
            hb["drivetrainOk"] = received_drivetrain_state_ok;
            hb["aliveBus0AgeMs"] = received_haldex_alive_bus0_ms ? (millis() - received_haldex_alive_bus0_ms) : 0;
            hb["drivetrainAgeMs"] = received_drivetrain_state_ms ? (millis() - received_drivetrain_state_ms) : 0;
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
    }

    data["chassisCAN"] = chassisOk;
    data["haldexCAN"] = haldexOk;
    data["busFailure"] = isBusFailure;
    data["lastChassisMs"] = lastCANChassisTick > 0 ? (millis() - lastCANChassisTick) : 0;
    data["lastHaldexMs"] = lastCANHaldexTick > 0 ? (millis() - lastCANHaldexTick) : 0;
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
        uds["blockagePct"] = udsBlockagePct;
    }

    // KWP2000/TP2.0 raw measuring-block capture (Gen2/4 PQ Haldex).
    if (haldexOk && liveDiagEnabled && (haldexGeneration == 2 || haldexGeneration == 4))
    {
        JsonObject kwp = data["kwp"].to<JsonObject>();
        kwp["connected"] = kwpTp20Connected;
        kwp["raw"] = kwpTp20RawDump;

        // Gen4 (0AY) decoded/scaled measuring values.
        if (haldexGeneration == 4)
        {
            kwp["oilTemp"] = kwpOilTemp;
            kwp["plateTemp"] = kwpPlateTemp;
            kwp["supplyVoltage"] = kwpSupplyVoltage;
            kwp["oilPressure"] = kwpOilPressure;
            kwp["estTorque"] = kwpEstTorque;
            kwp["clutchDuty"] = kwpClutchDuty;
            kwp["clutchValveCurrent"] = kwpClutchValveCurrent;
        }
    }

    data["uptimeMs"] = millis();
    data["freeHeap"] = ESP.getFreeHeap();
    data["lpChassisFrameCount"] = lpChassisFrameCount;
    data["lpHaldexFrameCount"] = lpHaldexFrameCount;

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
    data["tcForceModeValue"] = tcForceModeValue;
    data["hazardForceModeValue"] = hazardForceModeValue;
    data["extBtnForceModeValue"] = extBtnForceModeValue;
    data["disengageUnderSpeed"] = disengageUnderSpeed;
    data["disengageAboveSpeed"] = disengageAboveSpeed;
    data["disableThrottle"] = disableThrottle;
    data["mode"] = lastMode;
    data["lockReleaseRatePerSec"] = lockReleaseRatePerSec;
    data["steeringScaleEnabled"] = steeringScaleEnabled;
    data["FW_VERSION"] = FW_VERSION;

    // bools
    data["disableController"] = disableController;
    data["isStandalone"] = isStandalone;
    data["useCANifAvailable"] = useCANifAvailable;
    data["tcForceMode"] = tcForceMode;
    data["extButtonForceMode"] = extBtnForceMode;
    data["hazardForceMode"] = hazardForceMode;

    data["disableOnboardButton"] = disableOnboardButton;
    data["disableExternalButton"] = disableExternalButton;
    data["fixHunting"] = fixHunting;
    data["dangerZoneEnabled"] = dangerZoneEnabled;
    data["bpkCeilingNm"] = bpkCeilingNm;
    data["esp14MinFloorPct"] = esp14MinFloorPct;
    data["longLearnNotes"] = longLearnNotes;
    data["canSleepEnabled"] = canSleepEnabled;
    data["canSleepAggressive"] = canSleepAggressive;
    data["benchMode"] = benchMode;
    data["lpWakeThresholdFps"] = lpWakeThresholdFps;

    data["analyzerMode"] = analyzerMode;
    data["analyzerSerial"] = analyzerSerial;
    data["liveDiagEnabled"] = liveDiagEnabled;

    data["followBrake"] = followBrake;
    data["invertBrake"] = invertBrake;
    data["followHandbrake"] = followHandbrake;
    data["invertHandbrake"] = invertHandbrake;

    data["broadcastOpenHaldexOverCAN"] = broadcastOpenHaldexOverCAN;

    data["ledBrightness"] = ledBrightness;
    data["slipWheelbaseMm"] = slipWheelbaseMm;
    data["slipTrackFrontMm"] = slipTrackFrontMm;
    data["slipTrackRearMm"] = slipTrackRearMm;
    data["slipSteeringRatio"] = slipSteeringRatio;

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
        return;
    }

    if (data["haldexGeneration"].is<uint8_t>())
    {
        int generation = data["haldexGeneration"];
        if (generation == 1 || generation == 2 || generation == 4 || generation == 50 || generation == 51 || generation == 52 || generation == 41)
        {
            haldexGeneration = (uint8_t)generation;
            lastMode = generation;
            udsApplyDefaultIds(); // move the UDS pair with the generation (no-op while the serial lab has it pinned)
        }
    }

    if (data["tcForceModeValue"].is<uint8_t>())
    {
        uint8_t v = data["tcForceModeValue"];
        if (v < 6)
            tcForceModeValue = v;
    }

    if (data["hazardForceModeValue"].is<uint8_t>())
    {
        uint8_t v = data["hazardForceModeValue"];
        if (v < 6)
            hazardForceModeValue = v;
    }

    if (data["extBtnForceModeValue"].is<uint8_t>())
    {
        uint8_t v = data["extBtnForceModeValue"];
        if (v < 6)
            extBtnForceModeValue = v;
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
            vTaskSuspend(handle_frames13);
            vTaskSuspend(handle_frames50);
            vTaskSuspend(handle_frames250);
            vTaskSuspend(handle_gen41_dual_bus_rates);
        }
        else
        {
            vTaskResume(handle_frames1000);
            vTaskResume(handle_frames200);
            vTaskResume(handle_frames100);
            vTaskResume(handle_frames25);
            vTaskResume(handle_frames20);
            vTaskResume(handle_frames10);
            vTaskResume(handle_frames13);
            vTaskResume(handle_frames50);
            vTaskResume(handle_frames250);
            vTaskResume(handle_gen41_dual_bus_rates);
        }
    }

    if (data["analyzerMode"].is<bool>())
    {
        setAnalyzerMode(data["analyzerMode"]);
    }

    if (data["analyzerSerial"].is<bool>())
    {
        setAnalyzerSerialMode(data["analyzerSerial"]);
    }

    if (data["liveDiagEnabled"].is<bool>())
    {
        liveDiagEnabled = data["liveDiagEnabled"];
    }

    if (data["useCANifAvailable"].is<bool>())
    {
        useCANifAvailable = data["useCANifAvailable"];
    }

    if (data["tcForceMode"].is<bool>())
    {
        tcForceMode = data["tcForceMode"];
    }

    if (data["extButtonForceMode"].is<bool>())
    {
        extBtnForceMode = data["extButtonForceMode"];
    }

    if (data["hazardForceMode"].is<bool>())
    {
        hazardForceMode = data["hazardForceMode"];
        if (!hazardForceMode)
        {

            hazardForceModeFlag = false; // clear active flag when feature is disabled
        }
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

    if (data["steeringScaleEnabled"].is<bool>())
    {
        steeringScaleEnabled = data["steeringScaleEnabled"];
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
    if (data["lpWakeThresholdFps"].is<uint16_t>())
    {
        lpWakeThresholdFps = constrain((uint16_t)data["lpWakeThresholdFps"], 0, 2000);
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

    if (data["ledBrightness"].is<int>())
    {
        ledBrightness = (uint8_t)constrain((int)data["ledBrightness"], 0, 255);
    }

    // Vehicle geometry for per-corner slip (compute_corner_slip): mm/ratio,
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
        }
    }

    JsonDocument resp;
    resp["ok"] = true;
    sendJSON(request, 200, resp);
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
    if (!hasCANHaldex)
    {
        resp["ok"] = false;
        resp["error"] = "No Haldex CAN data available";
    }
    else if (frameEditGenIdx(haldexGeneration) < 0)
    {
        resp["ok"] = false;
        resp["error"] = "Frame editing is not available for this generation";
    }
    else if (!startLongLearn(testAll))
    {
        resp["ok"] = false;
        resp["error"] = "A learn is already running";
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

// manage tune (saved from Web, handled here): WebServer sends, this handles
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

    // Speed/throttle/lock map (optional - only applied when present in the payload).
    if (!speedArrayJSON.isNull() || !throttleArrayJSON.isNull() || !lockArrayJSON.isNull())
    {
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
    }

    // Steering-angle lock-scale curve (optional - breakpoints + 0-100% multipliers).
    JsonArray steeringArrayJSON = data["steeringArray"].as<JsonArray>();
    JsonArray steeringScaleJSON = data["steeringLockScaleArray"].as<JsonArray>();
    if (!steeringArrayJSON.isNull() || !steeringScaleJSON.isNull())
    {
        if (steeringArrayJSON.size() != steeringArrayCount || steeringScaleJSON.size() != steeringArrayCount)
        {
            DEBUG("Invalid steering array length");
            return;
        }
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

// setup main section for handling requests
void setupAPI()
{
    // ===== GET ENDPOINTS =====

    // GET /api/settings - retrieve all current settings
    webServer.on("/api/settings", HTTP_GET, [](AsyncWebServerRequest *request)
                 { settingsOutgoing(request); });

    // GET /api/dashboard - retrieve live status data (polled regularly by JS)
    webServer.on("/api/dashboard", HTTP_GET, [](AsyncWebServerRequest *request)
                 {
                     otaNoteWebActivity(); // a browser is on the UI - hold WiFi up even if it came in via the home router
                     statusOutgoing(request); });

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
                     udsWebBus = bus;
                     udsWebRespId = responseId;

                     OpenHaldexC6::UDS uds(bus ? twai_bus_1 : twai_bus_0, udsWebRxQueue);
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
                     data["floorNow"]   = esp14MinFloorPct;
                     data["floorStart"] = longLearnFloorStart;
                     data["floorResult"]= longLearnFloorResult;
                     data["bpkNow"]       = bpkCeilingNm;
                     data["bpkStart"]     = longLearnBpkStart;
                     data["bpkAdjusted"]  = longLearnBpkAdjusted;
                     data["fixHunting"]   = fixHunting;
                     data["isStandalone"] = isStandalone;
                     const uint32_t endMs = longLearnActive ? millis() : longLearnEndMs;
                     data["elapsedS"]   = (longLearnStartMs && endMs >= longLearnStartMs) ? (endMs - longLearnStartMs) / 1000 : 0;
                     // live sweep position (mirrors /api/learn/status)
                     data["cf"]   = (uint8_t)haldexLearnCF;
                     data["eng"]  = received_haldex_engagement;
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
                             o["result"]  = longLearnBlockResult[frameEditBlocks[i].bit];
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
                         o["floor"]   = e.floorPct;
                         o["bpk"]     = e.bpkNm;
                         o["verdict"] = e.verdict;
                         putScore(o, e.s);
                     }
                     sendJSON(request, 200, data); });

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
