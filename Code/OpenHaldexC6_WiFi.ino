void connectWifi() {
  WiFi.hostname(wifiHostName);
#if detailedDebugWiFi
  DEBUG("Beginning WiFi...");
  DEBUG("Creating Access Point...");
#endif
  //WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192, 168, 1, 1), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(wifiHostName);
  WiFi.setSleep(false);  // for the ESP32: turn off sleeping to increase UI responsivness (at the cost of power use)
}

void setupUI() {
  ESPUI.setVerbosity(Verbosity::Quiet);  // turn off verbose debugging (Verbose for ON; Quiet for OFF)
  ESPUI.sliderContinuous = true;         // update slider valves constantly disabled.  No need and can cause crashes

  // create mode tab
  auto tabModes = ESPUI.addControl(Tab, "", "Haldex Modes");
  ESPUI.addControl(Separator, "Current Mode", "", Dark, tabModes);
  int16_currentMode = ESPUI.addControl(Select, "Current Mode", "", Dark, tabModes, generalCallback);
  ESPUI.addControl(Option, "Stock", "Stock", Dark, int16_currentMode);
  ESPUI.addControl(Option, "FWD", "FWD", Dark, int16_currentMode);
  ESPUI.addControl(Option, "50/50", "5050", Dark, int16_currentMode);
  ESPUI.addControl(Option, "60/40", "6040", Dark, int16_currentMode);
  ESPUI.addControl(Option, "75/25", "7525", Dark, int16_currentMode);
  ESPUI.addControl(Option, "Custom", "Custom", Dark, int16_currentMode);

  ESPUI.addControl(Separator, "Current Locking", "", Dark, tabModes);
  label_currentLocking = ESPUI.addControl(Label, "", "0", Dark, tabModes, generalCallback);

  int16_disableThrottle = ESPUI.addControl(Slider, "Disable Below Throttle (%)", String(disableThrottle), Dark, tabModes, generalCallback);
  ESPUI.addControl(Min, "", "0", Dark, int16_disableThrottle);
  ESPUI.addControl(Max, "", "100", Dark, int16_disableThrottle);

  int16_disableSpeed = ESPUI.addControl(Slider, "Disable Above Speed (kmh)", String(disableSpeed), Dark, tabModes, generalCallback);
  ESPUI.addControl(Min, "", "0", Dark, int16_disableSpeed);
  ESPUI.addControl(Max, "", "300", Dark, int16_disableSpeed);

  bool_disableControl = ESPUI.addControl(Switcher, "Disable Controller", String(disableController), Dark, tabModes, generalCallback);

  // create General Setup tab
  auto tabSetup = ESPUI.addControl(Tab, "", "Setup");
  ESPUI.addControl(Separator, "Haldex Generation", "", Dark, tabSetup);
  int16_haldexGeneration = ESPUI.addControl(Select, "Haldex Generation", "", Dark, tabSetup, generalCallback);
  ESPUI.addControl(Option, "Generation 1", "Gen1", Dark, int16_haldexGeneration);
  ESPUI.addControl(Option, "Generation 2", "Gen2", Dark, int16_haldexGeneration);
  ESPUI.addControl(Option, "Generation 3 (N/A)", "Gen3", Dark, int16_haldexGeneration);
  ESPUI.addControl(Option, "Generation 4", "Gen4", Dark, int16_haldexGeneration);
  ESPUI.addControl(Option, "Generation 5 (N/A)", "Gen5", Dark, int16_haldexGeneration);

  ESPUI.addControl(Separator, "Follow", "", Dark, tabSetup);
  bool_followHandbrake = ESPUI.addControl(Switcher, "Follow Handbrake", String(followHandbrake), Dark, tabSetup, generalCallback);
  bool_invertHandbrake = ESPUI.addControl(Switcher, "Invert Handbrake", String(invertHandbrake), Dark, tabSetup, generalCallback);

  bool_followBrake = ESPUI.addControl(Switcher, "Follow Brake", String(followBrake), Dark, tabSetup, generalCallback);
  bool_invertBrake = ESPUI.addControl(Switcher, "Invert Brake", String(invertBrake), Dark, tabSetup, generalCallback);

  ESPUI.addControl(Separator, "Haldex Standalone", "", Dark, tabSetup);
  bool_isStandalone = ESPUI.addControl(Switcher, "Standalone", String(isStandalone), Dark, tabSetup, generalCallback);

  ESPUI.addControl(Separator, "Broadcast Haldex Over CAN", "", Dark, tabSetup);
  bool_broadcastHaldex = ESPUI.addControl(Switcher, "Broadcast Haldex", String(broadcastOpenHaldexOverCAN), Dark, tabSetup, generalCallback);

  // create Custom Modes tab
  String clearLabelStyle = "background-color: unset; width: 100%;";
  auto tabCustom = ESPUI.addControl(Tab, "", "Custom");
  ESPUI.addControl(Separator, "Lock on Throttle or Speed", "", Dark, tabCustom);
  int16_customSelect = ESPUI.addControl(Select, "Throttle/Speed", "", Dark, tabCustom, generalCallback);
  ESPUI.addControl(Option, "Speed", "Speed", Dark, int16_customSelect);
  ESPUI.addControl(Option, "Throttle", "Throttle", Dark, int16_customSelect);
  ESPUI.addControl(Option, "Both", "Both", Dark, int16_customSelect);

  //add a 'custom' array for locking.
  customSet_1_throttle = ESPUI.addControl(Slider, "Disable Below Throttle (%)", String(throttleArray[0]), Dark, tabCustom, generalCallback);  // create 'speed' slider
  ESPUI.addControl(Max, "", "100", Dark, customSet_1_throttle);                                                                               // set max valve to 100(%)
  ESPUI.setElementStyle(ESPUI.addControl(Label, "", "Throttle", None, customSet_1_throttle), clearLabelStyle);                                // give it a label

  customSet_1_speed = ESPUI.addControl(Slider, "Disable Above Speed (kmh)", String(speedArray[0]), Dark, tabCustom, generalCallback);  // create 'speed' slider
  ESPUI.addControl(Max, "", "300", Dark, customSet_1_speed);                                                                           // set max value to 300 (kmh)
  ESPUI.setElementStyle(ESPUI.addControl(Label, "", "Speed", None, customSet_1_speed), clearLabelStyle);                               // give it a label

  customSet_1_lock = ESPUI.addControl(Slider, "Amount of Lock (%)", String(lockArray[0]), Dark, tabCustom, generalCallback);  // create 'speed' slider
  ESPUI.setElementStyle(ESPUI.addControl(Label, "", "Lock %", None, customSet_1_lock), clearLabelStyle);                      // give it a label
  ESPUI.addControl(Max, "", "100", Dark, customSet_1_lock);                                                                   // set max value to 100(%)

  // create Diag tab
  auto tabDiag = ESPUI.addControl(Tab, "", "Diag.");
  ESPUI.addControl(Separator, "Chassis CAN", "", Dark, tabDiag);
  label_hasChassisCAN = ESPUI.addControl(Label, "", "0", Dark, tabDiag, generalCallback);
  ESPUI.addControl(Separator, "Haldex CAN", "", Dark, tabDiag);
  label_hasHaldexCAN = ESPUI.addControl(Label, "", "0", Dark, tabDiag, generalCallback);
  ESPUI.addControl(Separator, "Bus Failures", "", Dark, tabDiag);
  label_hasBusFailure = ESPUI.addControl(Label, "", "0", Dark, tabDiag, generalCallback);

  ESPUI.addControl(Separator, "Haldex State", "", Dark, tabDiag);
  label_HaldexState = ESPUI.addControl(Label, "", "0", Dark, tabDiag, generalCallback);
  ESPUI.addControl(Separator, "Haldex Temp", "", Dark, tabDiag);
  label_HaldexTemp = ESPUI.addControl(Label, "", "0", Dark, tabDiag, generalCallback);
  ESPUI.addControl(Separator, "Haldex Clutch 1", "", Dark, tabDiag);
  label_HaldexClutch1 = ESPUI.addControl(Label, "", "0", Dark, tabDiag, generalCallback);
  ESPUI.addControl(Separator, "Haldex Clutch 2", "", Dark, tabDiag);
  label_HaldexClutch2 = ESPUI.addControl(Label, "", "0", Dark, tabDiag, generalCallback);
  ESPUI.addControl(Separator, "Haldex Coupling", "", Dark, tabDiag);
  label_HaldexCoupling = ESPUI.addControl(Label, "", "0", Dark, tabDiag, generalCallback);
  ESPUI.addControl(Separator, "Haldex Speed Limit", "", Dark, tabDiag);
  label_HaldexSpeedLimit = ESPUI.addControl(Label, "", "0", Dark, tabDiag, generalCallback);

  ESPUI.addControl(Separator, "Speed", "", Dark, tabDiag);
  label_currentSpeed = ESPUI.addControl(Label, "", "0", Dark, tabDiag, generalCallback);
  ESPUI.addControl(Separator, "RPM", "", Dark, tabDiag);
  label_currentRPM = ESPUI.addControl(Label, "", "0", Dark, tabDiag, generalCallback);
  ESPUI.addControl(Separator, "Boost", "", Dark, tabDiag);
  label_currentBoost = ESPUI.addControl(Label, "", "0", Dark, tabDiag, generalCallback);

  ESPUI.addControl(Separator, "Brake Signal In", "", Dark, tabDiag);
  label_brakeIn = ESPUI.addControl(Label, "", "0", Dark, tabDiag, generalCallback);

  ESPUI.addControl(Separator, "Brake Signal Out", "", Dark, tabDiag);
  label_brakeOut = ESPUI.addControl(Label, "", "0", Dark, tabDiag, generalCallback);

  ESPUI.addControl(Separator, "Handbrake Signal In", "", Dark, tabDiag);
  label_handbrakeIn = ESPUI.addControl(Label, "", "0", Dark, tabDiag, generalCallback);

  ESPUI.addControl(Separator, "Handbrake Signal Out", "", Dark, tabDiag);
  label_handbrakeOut = ESPUI.addControl(Label, "", "0", Dark, tabDiag, generalCallback);

  // create OTA Update tab
  auto tabOTA = ESPUI.addControl(Tab, "", "Firmware");
  ESPUI.addControl(Separator, "Firmware Information", "", Dark, tabOTA);
  label_firmwareVersion = ESPUI.addControl(Label, "Firmware Version", getFirmwareVersion().c_str(), Dark, tabOTA);
  label_chipModel = ESPUI.addControl(Label, "Chip Model", ESP.getChipModel(), Dark, tabOTA);
  label_freeHeap = ESPUI.addControl(Label, "Free Heap", String(ESP.getFreeHeap()).c_str(), Dark, tabOTA);

  ESPUI.addControl(Separator, "Update Instructions", "", Dark, tabOTA);
  ESPUI.addControl(Label, "", "1. Ensure vehicle is stationary (speed = 0)", Dark, tabOTA);
  ESPUI.addControl(Label, "", "2. Open: 192.168.1.1:81/update", Dark, tabOTA);
  ESPUI.addControl(Label, "", "3. Login: admin / haldex", Dark, tabOTA);
  ESPUI.addControl(Label, "", "4. Upload firmware .bin file", Dark, tabOTA);
  ESPUI.addControl(Label, "", "5. Wait for reboot", Dark, tabOTA);

  ESPUI.addControl(Separator, "Status", "", Dark, tabOTA);
  label_otaStatus = ESPUI.addControl(Label, "OTA Status", "Ready", Dark, tabOTA);

  // Analyzer mode toggle lives on Setup to avoid adding a new tab.
  ESPUI.addControl(Separator, "Analyzer", "", Dark, tabSetup);
  bool_analyzerMode = ESPUI.addControl(Switcher, "Analyzer Mode", String(analyzerMode), Dark, tabSetup, generalCallback);

  //Finally, start up the UI.
  //This should only be called once we are connected to WiFi.
  ESPUI.begin(wifiHostName);
  //WiFi.setTxPower(WIFI_POWER_8_5dBm);  // set a lower power mode (some C3 aerials aren't great and leaving it high causes failures)
}

void disconnectWifi() {
  rebootWiFi = true;
}

void generalCallback(Control *sender, int type) {
#if detailedDebugWiFi
  Serial.print("CB: id(");
  Serial.print(sender->id);
  Serial.print(") Type(");
  Serial.print(type);
  Serial.print(") '");
  Serial.print(sender->label);
  Serial.print("' = ");
  Serial.println(sender->value);
#endif

  // Analyzer toggle: flips pass-through mode and keeps UI in sync.
  if (sender->id == bool_analyzerMode) {
    setAnalyzerMode(sender->value.toInt());
    return;
  }

  uint8_t tempID = int(sender->id);
  switch (tempID) {
    case 3:
      if (sender->value == "Stock") {
        state.mode = MODE_STOCK;
      }
      if (sender->value == "FWD") {
        state.mode = MODE_FWD;
      }
      if (sender->value == "5050") {
        state.mode = MODE_5050;
      }
      if (sender->value == "6040") {
        state.mode = MODE_6040;
      }
      if (sender->value == "7525") {
        state.mode = MODE_7525;
      }
      if (sender->value == "Custom") {
        state.mode = MODE_CUSTOM;
      }
      break;
    case 12:
      disableThrottle = sender->value.toInt();
      state.pedal_threshold = disableThrottle;
      break;
    case 15:
      disableSpeed = sender->value.toInt();
      break;
    case 18:
      // If the user re-enables the controller, ensure analyzer mode is off.
      disableController = sender->value.toInt();
      if (!disableController && analyzerMode) {
        setAnalyzerMode(false);
      }
      break;
    case 21:
      if (sender->value == "Gen1") {
        haldexGeneration = 1;
      }
      if (sender->value == "Gen2") {
        haldexGeneration = 2;
      }
      if (sender->value == "Gen3") {
        haldexGeneration = 2;
      }
      if (sender->value == "Gen4") {
        haldexGeneration = 4;
      }
      if (sender->value == "Gen5") {
        haldexGeneration = 4;
      }
      break;
    case 28:
      followHandbrake = sender->value.toInt();
      break;
    case 29:
      invertHandbrake = sender->value.toInt();
      break;
    case 30:
      followBrake = sender->value.toInt();
      break;
    case 31:
      invertBrake = sender->value.toInt();
      break;
    case 33:
      isStandalone = sender->value.toInt();

      if (!isStandalone) {
        vTaskSuspend(handle_frames1000);
        vTaskSuspend(handle_frames200);
        vTaskSuspend(handle_frames100);
        vTaskSuspend(handle_frames25);
        vTaskSuspend(handle_frames20);
        vTaskSuspend(handle_frames10);
      } else {
        vTaskResume(handle_frames1000);
        vTaskResume(handle_frames200);
        vTaskResume(handle_frames100);
        vTaskResume(handle_frames25);
        vTaskResume(handle_frames20);
        vTaskResume(handle_frames10);
      }
      break;
    case 35:
      broadcastOpenHaldexOverCAN = sender->value.toInt();
      break;

    case 38:
      if (sender->value == "Speed") {
        customSpeed = true;
        customThrottle = false;
      }
      if (sender->value == "Throttle") {
        customSpeed = false;
        customThrottle = true;
      }
      if (sender->value == "Both") {
        customSpeed = true;
        customThrottle = true;
      }
      break;
    case 42:
      throttleArray[0] = sender->value.toInt();  // custom 1 - throttle
      break;
    case 45:
      speedArray[0] = sender->value.toInt();  // custom 1 - speed
      break;
    case 48:
      lockArray[0] = sender->value.toInt();  // custom 1 - lock
      break;
  }
}

void extendedCallback(Control *sender, int type, void *param) {
#if serialDebugWifi
  Serial.print("eCB: id(");
  Serial.print(sender->id);
  Serial.print(") Type(");
  Serial.print(type);
  Serial.print(") '");
  Serial.print(sender->label);
  Serial.print("' = ");
  Serial.println(sender->value);
  Serial.print("param = ");
  Serial.println((long)param);
#endif
}

void updateLabels(void *arg) {
  while (1) {
#if detailedDebugStack
    stackupdateLabels = uxTaskGetStackHighWaterMark(NULL);
#endif

    if ((millis() - lastCANChassisTick) > 500) {
      hasCANChassis = false;
    } else {
      hasCANChassis = true;
    }

    if ((millis() - lastCANHaldexTick) > 500) {
      hasCANHaldex = false;
    } else {
      hasCANHaldex = true;
    }

    hasCANChassis ? ESPUI.updateLabel(label_hasChassisCAN, "Yes") : ESPUI.updateLabel(label_hasChassisCAN, "No");
    hasCANHaldex ? ESPUI.updateLabel(label_hasHaldexCAN, "Yes") : ESPUI.updateLabel(label_hasHaldexCAN, "No");

    isBusFailure ? ESPUI.updateLabel(label_hasBusFailure, "Yes") : ESPUI.updateLabel(label_hasBusFailure, "No");

    disableController ? state.mode = MODE_STOCK : state.mode = state.mode;

    if (customSpeed && !customThrottle) {
      ESPUI.updateSelect(int16_customSelect, "Speed");
    }
    if (customThrottle && !customSpeed) {
      ESPUI.updateLabel(int16_customSelect, "Throttle");
    }
    if (customThrottle && customSpeed) {
      ESPUI.updateLabel(int16_customSelect, "Both");
    }

    switch (haldexGeneration) {
      case 1:
        ESPUI.updateSelect(int16_haldexGeneration, "Gen1");
        isStandalone ? isGen1Standalone = true : isGen1Standalone = false;
        break;
      case 2:
        ESPUI.updateSelect(int16_haldexGeneration, "Gen2");
        isStandalone ? isGen2Standalone = true : isGen2Standalone = false;
        break;
      case 4:
        ESPUI.updateSelect(int16_haldexGeneration, "Gen4");
        isStandalone ? isGen4Standalone = true : isGen4Standalone = false;
        break;
    }

    if (isStandalone) {
      lock_target = get_lock_target_adjustment();
    }

    switch (state.mode) {
      case 0:
        if (isStandalone) {
          state.mode = MODE_FWD;
          lastMode = 1;
          break;
        }

        lastMode = 0;
        ESPUI.updateSelect(int16_currentMode, "Stock");
        strip.setLedColorData(led_channel, led_brightness, 0, 0);  // red
        break;
      case 1:
        lastMode = 1;
        ESPUI.updateSelect(int16_currentMode, "FWD");
        strip.setLedColorData(led_channel, 0, led_brightness, 0);  // green
        break;
      case 2:
        lastMode = 2;
        ESPUI.updateSelect(int16_currentMode, "5050");
        strip.setLedColorData(led_channel, 0, 0, led_brightness);  // blue
        break;
      case 3:
        lastMode = 3;
        ESPUI.updateSelect(int16_currentMode, "6040");
        strip.setLedColorData(led_channel, 255, 16, 240);  // neon pink
        break;
      case 4:
        lastMode = 4;
        ESPUI.updateSelect(int16_currentMode, "7525");
        strip.setLedColorData(led_channel, 0, led_brightness, led_brightness);  // cyan
        break;
      case 5:
        lastMode = 5;
        ESPUI.updateSelect(int16_currentMode, "Custom");
        strip.setLedColorData(led_channel, led_brightness, led_brightness, led_brightness);  // white
        break;
    }
    strip.show();

    char haldexTemp[50];
    sprintf(haldexTemp, "Req:Act: %d:%d", int(lock_target), received_haldex_engagement);  //lock_target appliedTorque
    ESPUI.updateLabel(label_currentLocking, String(haldexTemp));

    haldexTemp[0] = '\0';
    sprintf(haldexTemp, "%d kmh", received_vehicle_speed);
    ESPUI.updateLabel(label_currentSpeed, String(haldexTemp));

    haldexTemp[0] = '\0';
    sprintf(haldexTemp, "%d mbar", received_vehicle_boost);
    ESPUI.updateLabel(label_currentBoost, String(haldexTemp));

    haldexTemp[0] = '\0';
    sprintf(haldexTemp, BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(received_haldex_state));
    ESPUI.updateLabel(label_HaldexState, String(haldexTemp));

    ESPUI.updateLabel(label_HaldexClutch1, received_report_clutch1 ? "Open" : "Closed");
    ESPUI.updateLabel(label_HaldexClutch2, received_report_clutch2 ? "Open" : "Closed");
    ESPUI.updateLabel(label_HaldexCoupling, received_coupling_open ? "Open" : "Closed");
    ESPUI.updateLabel(label_HaldexSpeedLimit, received_speed_limit ? "Active" : "Not Active");

    ESPUI.updateLabel(label_currentRPM, String(received_vehicle_rpm));

    ESPUI.updateLabel(label_brakeIn, brakeSignalActive ? "Active" : "Not Active");
    ESPUI.updateLabel(label_handbrakeIn, handbrakeSignalActive ? "Active" : "Not Active");

    ESPUI.updateLabel(label_brakeOut, brakeActive ? "Active" : "Not Active");
    ESPUI.updateLabel(label_handbrakeOut, handbrakeActive ? "Active" : "Not Active");

    // Update OTA tab labels
    ESPUI.updateLabel(label_firmwareVersion, getFirmwareVersion());
    ESPUI.updateLabel(label_chipModel, ESP.getChipModel());
    ESPUI.updateLabel(label_freeHeap, String(ESP.getFreeHeap()));

    // Update OTA status - warn if vehicle is moving
    if (received_vehicle_speed > 0) {
      ESPUI.updateLabel(label_otaStatus, "Vehicle Moving - OTA Disabled");
    } else {
      ESPUI.updateLabel(label_otaStatus, "Ready for Update");
    }

    vTaskDelay(labelRefresh / portTICK_PERIOD_MS);
  }
}


