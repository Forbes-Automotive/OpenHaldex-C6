void setupIO() {
  // using the TCAN1044 for CAN control
  pinMode(CAN0_RS, OUTPUT);    // gpio for controlling can_0 state - enabled or disabled
  pinMode(CAN1_RS, OUTPUT);    // gpio for controlling can_0 slope - enabled or disabled
  digitalWrite(CAN0_RS, LOW);  // set chip ena
  digitalWrite(CAN1_RS, LOW);  // set chip ena

  pinMode(gpio_hb_in, INPUT);       // gpio for internal mode button
  pinMode(gpio_brake_in, INPUT);    // gpio for internal mode button
  pinMode(gpio_hb_out, OUTPUT);     // gpio for internal mode button
  pinMode(gpio_brake_out, OUTPUT);  // gpio for internal mode button

  strip.begin();                        // begin RGB LED onboard
  strip.setBrightness(led_brightness);  // default brightness
}

void setupCAN() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(gpio_num_t(CAN0_TX), gpio_num_t(CAN0_RX), TWAI_MODE_NO_ACK);  // TWAI_MODE_NORMAL, TWAI_MODE_NO_ACK or TWAI_MODE_LISTEN_ONLY
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();                                                             // default CAN speed
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();                                                           // accept all messages

  //g_config.intr_flags = ESP_INTR_FLAG_LOWMED;  //Optional - move canbus irq to free up the default level 1 IRQ it will take up.  Todo
  g_config.tx_queue_len = 1024;  //<TWAI_GENERAL_CONFIG_DEFAULT default is 5, use this to increase if needed
  g_config.rx_queue_len = 2048;  //<TWAI_GENERAL_CONFIG_DEFAULT default is 5, use this to increase if needed // 4096
  //g_config.intr_flags = ESP_INTR_FLAG_IRAM;

  // setup CAN Controller (0)
  g_config.controller_id = 0;
  ESP_ERROR_CHECK(twai_driver_install_v2(&g_config, &t_config, &f_config, &twai_bus_0));
  DEBUG("CAN - Driver 0 Installed");
  ESP_ERROR_CHECK(twai_start_v2(twai_bus_0));
  DEBUG("CAN - Driver 0 Started");

  // setup CAN Controller (1)
  g_config.controller_id = 1;
  g_config.tx_io = gpio_num_t(CAN1_TX);
  g_config.rx_io = gpio_num_t(CAN1_RX);
  ESP_ERROR_CHECK(twai_driver_install_v2(&g_config, &t_config, &f_config, &twai_bus_1));
  DEBUG("CAN - Driver 1 Installed");
  ESP_ERROR_CHECK(twai_start_v2(twai_bus_1));
  DEBUG("CAN - Driver 1 Started");

  // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    DEBUG("Reconfiguration of CAN alerts");
  } else {
    DEBUG("Failed to reconfigure CAN alerts!");
    return;
  }
}

void setupButtons() {
  //setup buttons / inputs
  btnMode.setMenuCount(0);
  btnMode.setMenuLevel(0);            // Use the functions bound to the first menu associated with the button
  btnMode.setMode(Mode_Synchronous);  // can be caught in the main loop - no rush

  btnMode_ext.setMenuCount(0);
  btnMode_ext.setMenuLevel(0);            // Use the functions bound to the first menu associated with the button
  btnMode_ext.setMode(Mode_Synchronous);  // can be caught in the main loop - no rush

  //InterruptButton::m_RTOSservicerStackDepth = 4096; // Use larger values for more memory intensive functions if using Asynchronous mode.
  btnMode.bind(Event_KeyPress, 0, &modeChange);  // apply to 'mode change'
  btnMode.bind(Event_LongKeyPress, 0, &disconnectWifi);
  btnMode_ext.bind(Event_KeyPress, 0, &modeChangeExt);  // apply to 'mode change ext'
}

void setupTasks() {
  // max task priority = 24
  xTaskCreate(showHaldexState, "showHaldexState", 5000, NULL, 1, NULL);
  xTaskCreate(updateLabels, "updateLabels", 6000, NULL, 2, NULL);
  xTaskCreate(writeEEP, "writeEEP", 2000, NULL, 3, NULL);
  xTaskCreate(updateTriggers, "updateTriggers", 2000, NULL, 4, NULL);
  // Analyzer task stays idle unless analyzerMode is enabled.
  setupAnalyzer();

  xTaskCreate(frames1000, "frames1000", 8000, NULL, 5, &handle_frames1000);
  xTaskCreate(frames200, "frames200", 8000, NULL, 6, &handle_frames200);
  xTaskCreate(frames100, "frames100", 8000, NULL, 7, &handle_frames100);
  xTaskCreate(frames25, "frames25", 8000, NULL, 8, &handle_frames25);
  xTaskCreate(frames20, "frames20", 8000, NULL, 9, &handle_frames20);
  xTaskCreate(frames10, "frames10", 8000, NULL, 10, &handle_frames10);

  if (!isStandalone) {
    vTaskSuspend(handle_frames1000);
    vTaskSuspend(handle_frames200);
    vTaskSuspend(handle_frames100);
    vTaskSuspend(handle_frames25);
    vTaskSuspend(handle_frames20);
    vTaskSuspend(handle_frames10);
  }

  xTaskCreate(broadcastOpenHaldex, "broadcastOpenHaldex", 1000, NULL, 10, NULL);
  xTaskCreate(parseCAN_hdx, "parseHaldex", 2048, NULL, 11, NULL);   // create a task for FreeRTOS for incoming haldex CAN - in '_can.ino'
  xTaskCreate(parseCAN_chs, "parseChassis", 2048, NULL, 12, NULL);  // create a task for FreeRTOS for incoming chassis CAN - in '_can.ino'
}

void updateTriggers(void *arg) {
  while (1) {
    if (digitalRead(gpio_hb_in)) {
      handbrakeSignalActive = true;
    } else {
      handbrakeSignalActive = false;
    }

    if (digitalRead(gpio_brake_in)) {
      brakeSignalActive = true;
    } else {
      brakeSignalActive = false;
    }
    // Analyzer mode: keep buttons + CAN recovery, but skip brake/handbrake IO outputs.
    if (analyzerMode) {
      btnMode.processSyncEvents();      // Only required if using sync events
      btnMode_ext.processSyncEvents();  // Only required if using sync events

      if (isBusFailure) {
        twai_initiate_recovery_v2(twai_bus_0);
        twai_initiate_recovery_v2(twai_bus_1);
      }

      vTaskDelay(updateTriggersRefresh / portTICK_PERIOD_MS);
      continue;
    }



    if (followHandbrake) {
      if (invertHandbrake) {
        digitalWrite(gpio_hb_out, !digitalRead(gpio_hb_in));
        handbrakeActive = false;
      } else {
        digitalWrite(gpio_hb_out, digitalRead(gpio_hb_in));
        handbrakeActive = true;
      }
    } else {
      if (invertHandbrake) {
        digitalWrite(gpio_hb_out, LOW);
        handbrakeActive = false;

      } else {
        digitalWrite(gpio_hb_out, HIGH);
        handbrakeActive = true;
      }
    }

    if (followBrake) {
      if (invertBrake) {
        digitalWrite(gpio_brake_out, !digitalRead(gpio_brake_in));
        brakeActive = false;
      } else {
        digitalWrite(gpio_brake_out, digitalRead(gpio_brake_in));
        brakeActive = true;
      }
    } else {
      if (invertBrake) {
        digitalWrite(gpio_brake_out, LOW);
        brakeActive = false;

      } else {
        digitalWrite(gpio_brake_out, HIGH);
        brakeActive = true;
      }
    }

    btnMode.processSyncEvents();      // Only required if using sync events
    btnMode_ext.processSyncEvents();  // Only required if using sync events

    if (isBusFailure) {
      twai_initiate_recovery_v2(twai_bus_0);
      twai_initiate_recovery_v2(twai_bus_1);
    }

    vTaskDelay(updateTriggersRefresh / portTICK_PERIOD_MS);
  }
}

void showHaldexState(void *arg) {
  while (1) {
    stackshowHaldexState = uxTaskGetStackHighWaterMark(NULL);

    DEBUG("Mode: %s", get_openhaldex_mode_string(state.mode));
    DEBUG("    Req:Act: %d:%d", int(lock_target), received_haldex_engagement);  // this is the lock %

#if detailedDebug
    DEBUG("    Raw haldexEngagement: " BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(received_haldex_state));
    DEBUG("    reportClutch1: %d", received_report_clutch1);  // this means it has a clutch issue
    DEBUG("    reportClutch2: %d", received_report_clutch2);  // this means it also has a clutch issue
    DEBUG("    couplingOpen: %d", received_coupling_open);    // clutch fully disengaged
    DEBUG("    speedLimit: %d", received_speed_limit);        // hit a speed limit...
    DEBUG("    tempCounter2: %d", state.pedal_threshold);     // incrememting value for checking the response to vars...

    DEBUG("    hasChassisCAN: %d", hasCANChassis);          // incrememting value for checking the response to vars...
    DEBUG("    hasHaldexCAN: %d", hasCANHaldex);            // incrememting value for checking the response to vars...
    DEBUG("    lastCANChassis: %d", lastCANChassisTick);    // incrememting value for checking the response to vars...
    DEBUG("    lastHaldexChassis: %d", lastCANHaldexTick);  // incrememting value for checking the response to vars...

    DEBUG("    currentMode: %d", lastMode);        // incrememting value for checking the response to vars...
    DEBUG("    isStandalone: %d", isStandalone);   // incrememting value for checking the response to vars...
    DEBUG("    haldexGen: %d", haldexGeneration);  // incrememting value for checking the response to vars...

    if (isBusFailure) {
      DEBUG("    Bus Failure: True");
    }
#endif

#if detailedDebugIO
    DEBUG("    Brake signal: %s", brakeSignalActive ? "true" : "false");
    DEBUG("    Handbrake signal: %s", handbrakeSignalActive ? "true" : "false");

    DEBUG("    Follow brake: %s", followBrake ? "true" : "false");
    DEBUG("    Invert handbrake: %s", invertHandbrake ? "true" : "false");
    DEBUG("    Invert brake: %s", invertBrake ? "true" : "false");
#endif

#if detailedDebugStack
    DEBUG("Stack Sizes:");

    DEBUG("    stackCHS: %d", stackCHS);  // incrememting value for checking the response to vars...
    DEBUG("    stackHDX: %d", stackHDX);  // incrememting value for checking the response to vars...

    DEBUG("    stackframes10: %d", stackframes10);      // incrememting value for checking the response to vars...
    DEBUG("    stackframes20: %d", stackframes20);      // incrememting value for checking the response to vars...
    DEBUG("    stackframes25: %d", stackframes25);      // incrememting value for checking the response to vars...
    DEBUG("    stackframes100: %d", stackframes100);    // incrememting value for checking the response to vars...
    DEBUG("    stackframes200: %d", stackframes200);    // incrememting value for checking the response to vars...
    DEBUG("    stackframes1000: %d", stackframes1000);  // incrememting value for checking the response to vars...

    DEBUG("    stackbroadcastOpenHaldex: %d", stackbroadcastOpenHaldex);  // incrememting value for checking the response to vars...
    DEBUG("    stackupdateLabels: %d", stackupdateLabels);                // incrememting value for checking the response to vars...
    DEBUG("    stackshowHaldexState: %d", stackshowHaldexState);          // incrememting value for checking the response to vars...
    DEBUG("    stackwriteEEP: %d", stackwriteEEP);                        // incrememting value for checking the response to vars...
#endif

#if detailedDebugRuntimeStats
    char buffer2[2048] = { 0 };
    vTaskGetRunTimeStats(buffer2);
    Serial.println(buffer2);
#endif

#if detailedDebugCAN
    uint32_t alerts_triggered;
    twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(0));
    twai_status_info_t twaistatus;
    twai_get_status_info(&twaistatus);
    DEBUG("");                  // this is the lock %
    DEBUG("CAN-BUS Details:");  // this is the lock %
    DEBUG("    RX buffered: %lu\t", twaistatus.msgs_to_rx);
    DEBUG("    RX missed: %lu\t", twaistatus.rx_missed_count);
    DEBUG("    RX overrun %lu\n", twaistatus.rx_overrun_count);

    if (alerts_triggered & TWAI_ALERT_ERR_ACTIVE) {
      isBusFailure = true;
    }
    if (alerts_triggered & TWAI_ALERT_RECOVERY_IN_PROGRESS) {
      isBusFailure = true;
    }
    if (alerts_triggered & TWAI_ALERT_ABOVE_ERR_WARN) {
      isBusFailure = true;
    }
    if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
      isBusFailure = true;
    }
    if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
      isBusFailure = true;
    }
    if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
      isBusFailure = true;
    }
    if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
      isBusFailure = true;
    }
    if (alerts_triggered & TWAI_ALERT_BUS_OFF) {
      isBusFailure = true;
    }
    if (alerts_triggered & TWAI_ALERT_RX_FIFO_OVERRUN) {
      isBusFailure = true;
    }
#endif

    vTaskDelay(serialMonitorRefresh / portTICK_PERIOD_MS);
  }
}

void frames10(void *arg) {
  while (1) {
    // Analyzer mode runs as a passive bridge; skip standalone frame generation.
    if (analyzerMode) {
      vTaskDelay(10 / portTICK_PERIOD_MS);
      continue;
    }
    if (isStandalone) {
      switch (haldexGeneration) {
        case 1:
          Gen1_frames10();
          break;
        case 2:
          Gen2_frames10();
          break;
        case 4:
          Gen4_frames10();
          break;
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void frames20(void *arg) {
  while (1) {
    // Analyzer mode runs as a passive bridge; skip standalone frame generation.
    if (analyzerMode) {
      vTaskDelay(20 / portTICK_PERIOD_MS);
      continue;
    }
    if (isStandalone) {
      switch (haldexGeneration) {
        case 1:
          Gen1_frames20();
          break;
        case 2:
          Gen2_frames20();
          break;
        case 4:
          Gen4_frames20();
          break;
      }
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void frames25(void *arg) {
  while (1) {
    // Analyzer mode runs as a passive bridge; skip standalone frame generation.
    if (analyzerMode) {
      vTaskDelay(25 / portTICK_PERIOD_MS);
      continue;
    }
    if (isStandalone) {
      switch (haldexGeneration) {
        case 1:
          Gen1_frames25();
          break;
        case 2:
          Gen2_frames25();
          break;
        case 4:
          Gen4_frames25();
          break;
      }
    }
    vTaskDelay(25 / portTICK_PERIOD_MS);
  }
}

void frames100(void *arg) {
  while (1) {
    // Analyzer mode runs as a passive bridge; skip standalone frame generation.
    if (analyzerMode) {
      vTaskDelay(100 / portTICK_PERIOD_MS);
      continue;
    }
    if (isStandalone) {
      lock_target = get_lock_target_adjustment();
      switch (haldexGeneration) {
        case 1:
          Gen1_frames100();
          break;
        case 2:
          Gen2_frames100();
          break;
        case 4:
          Gen4_frames100();
          break;
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void frames200(void *arg) {
  while (1) {
    // Analyzer mode runs as a passive bridge; skip standalone frame generation.
    if (analyzerMode) {
      vTaskDelay(200 / portTICK_PERIOD_MS);
      continue;
    }
    if (isStandalone) {
      switch (haldexGeneration) {
        case 1:
          Gen1_frames200();
          break;
        case 2:
          Gen2_frames200();
          break;
        case 4:
          Gen4_frames200();
          break;
      }
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void frames1000(void *arg) {
  while (1) {
    // Analyzer mode runs as a passive bridge; skip standalone frame generation.
    if (analyzerMode) {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }
    if (isStandalone) {
      switch (haldexGeneration) {
        case 1:
          Gen1_frames1000();
          break;
        case 2:
          Gen2_frames1000();
          break;
        case 4:
          Gen4_frames1000();
          break;
      }
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
