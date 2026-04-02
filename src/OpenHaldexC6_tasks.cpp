#include <OpenHaldexC6_tasks.h>
#include <OpenHaldexC6_IO.h>
#include <OpenHaldexC6_can.h>
#include <OpenHaldexC6_EEP.h>
#include <OpenHaldexC6_Analyzer.h>
#include <OpenHaldexC6_StandaloneCAN.h>

void setupTasks()
{
  // max task priority = 24
  xTaskCreate(showHaldexState, "showHaldexState", 5000, NULL, 1, NULL);
  xTaskCreate(writeEEP, "writeEEP", 2000, NULL, 3, NULL);
  xTaskCreate(updateTriggers, "updateTriggers", 2000, NULL, 4, NULL);

  // Analyzer task stays idle unless analyzerMode is enabled.
  setupAnalyzer();

  // Create tasks for frame generation at various intervals. These will run in the background and can be suspended when not in use (e.g., when not in standalone mode).
  xTaskCreate(frames1000, "frames1000", 8000, NULL, 5, &handle_frames1000);
  xTaskCreate(frames200, "frames200", 8000, NULL, 6, &handle_frames200);
  xTaskCreate(frames100, "frames100", 8000, NULL, 7, &handle_frames100);
  xTaskCreate(frames25, "frames25", 8000, NULL, 8, &handle_frames25);
  xTaskCreate(frames20, "frames20", 8000, NULL, 9, &handle_frames20);
  xTaskCreate(frames10, "frames10", 8000, NULL, 10, &handle_frames10);

  if (!isStandalone)
  {
    vTaskSuspend(handle_frames1000);
    vTaskSuspend(handle_frames200);
    vTaskSuspend(handle_frames100);
    vTaskSuspend(handle_frames25);
    vTaskSuspend(handle_frames20);
    vTaskSuspend(handle_frames10);
  }

  xTaskCreate(broadcastOpenHaldex, "broadcastOpenHaldex", 1000, NULL, 10, NULL); // create a task for FreeRTOS for broadcasting the haldex state
  xTaskCreate(parseCAN_hdx, "parseHaldex", 2048, NULL, 11, NULL);                // create a task for FreeRTOS for incoming haldex CAN - in '_can.ino'
  xTaskCreate(parseCAN_chs, "parseChassis", 2048, NULL, 12, NULL);               // create a task for FreeRTOS for incoming chassis CAN - in '_can.ino'
}

void showHaldexState(void *arg)
{
  while (1)
  {
    stackshowHaldexState = uxTaskGetStackHighWaterMark(NULL);

    DEBUG("Mode: %s", get_openhaldex_mode_string(state.mode));
    DEBUG("    Req:Act: %d:%d", int(lock_target), received_haldex_engagement); // this is the lock %

    if (detailedDebug)
    {
      DEBUG("    Raw haldexEngagement: " BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(received_haldex_state));
      DEBUG("    reportClutch1: %d", received_report_clutch1); // this means it has a clutch issue
      DEBUG("    reportClutch2: %d", received_report_clutch2); // this means it also has a clutch issue
      DEBUG("    couplingOpen: %d", received_coupling_open);   // clutch fully disengaged
      DEBUG("    speedLimit: %d", received_speed_limit);       // hit a speed limit...
      DEBUG("    tempCounter2: %d", state.pedal_threshold);    // incrememting value for checking the response to vars...

      DEBUG("    hasChassisCAN: %d", hasCANChassis);         // incrememting value for checking the response to vars...
      DEBUG("    hasHaldexCAN: %d", hasCANHaldex);           // incrememting value for checking the response to vars...
      DEBUG("    lastCANChassis: %ldms", lastCANChassisTick > 0 ? (long)(millis() - lastCANChassisTick) : -1);   // incrememting value for checking the response to vars...
      DEBUG("    lastHaldex: %ldms", lastCANHaldexTick > 0 ? (long)(millis() - lastCANHaldexTick) : -1); // incrememting value for checking the response to vars...

      DEBUG("    currentMode: %d", lastMode);       // incrememting value for checking the response to vars...
      DEBUG("    isStandalone: %d", isStandalone);  // incrememting value for checking the response to vars...
      DEBUG("    haldexGen: %d", haldexGeneration); // incrememting value for checking the response to vars...
      DEBUG("    Analyser Mode: %d", analyzerMode);
      DEBUG("    Force Mode Value: %d", forceModeValue);

      DEBUG("Free heap: %d bytes", ESP.getFreeHeap());        // for debug - checking ESP available space
      DEBUG("Min free heap: %d bytes", ESP.getMinFreeHeap()); // for debug - checking ESP available space

      if (isBusFailure)
      {
        DEBUG("    Bus Failure: True");
      }
    }

    if (detailedDebugIO)
    {
      DEBUG("    Brake signal: %s", brakeSignalActive ? "true" : "false");
      DEBUG("    Handbrake signal: %s", handbrakeSignalActive ? "true" : "false");

      DEBUG("    Follow brake: %s", followBrake ? "true" : "false");
      DEBUG("    Invert handbrake: %s", invertHandbrake ? "true" : "false");
      DEBUG("    Invert brake: %s", invertBrake ? "true" : "false");
    }

    if (detailedDebugArray)
    {
      DEBUG("%d%d%d%d%d%d", throttleArray[0], throttleArray[1], throttleArray[2], throttleArray[3], throttleArray[4], throttleArray[5], throttleArray[6]);
      DEBUG("%d%d%d%d%d%d", speedArray[0], speedArray[1], speedArray[2], speedArray[3], speedArray[4], speedArray[5], speedArray[6]);
      // DEBUG("%d", speedArray[j]);
      // DEBUG("%d%d%d%d%d%d", lockArray[i][0], lockArray[i][1], lockArray[i][2], lockArray[i][3], lockArray[i][4], lockArray[i][5], lockArray[i][6]);
    }

    if (detailedDebugStack)
    {
      DEBUG("Stack Sizes:");

      DEBUG("    stackCHS: %d", stackCHS); // incrememting value for checking the response to vars...
      DEBUG("    stackHDX: %d", stackHDX); // incrememting value for checking the response to vars...

      DEBUG("    stackframes10: %d", stackframes10);     // incrememting value for checking the response to vars...
      DEBUG("    stackframes20: %d", stackframes20);     // incrememting value for checking the response to vars...
      DEBUG("    stackframes25: %d", stackframes25);     // incrememting value for checking the response to vars...
      DEBUG("    stackframes100: %d", stackframes100);   // incrememting value for checking the response to vars...
      DEBUG("    stackframes200: %d", stackframes200);   // incrememting value for checking the response to vars...
      DEBUG("    stackframes1000: %d", stackframes1000); // incrememting value for checking the response to vars...

      DEBUG("    stackbroadcastOpenHaldex: %d", stackbroadcastOpenHaldex); // incrememting value for checking the response to vars...
      DEBUG("    stackupdateLabels: %d", stackupdateLabels);               // incrememting value for checking the response to vars...
      DEBUG("    stackshowHaldexState: %d", stackshowHaldexState);         // incrememting value for checking the response to vars...
      DEBUG("    stackwriteEEP: %d", stackwriteEEP);                       // incrememting value for checking the response to vars...
    }

    if (detailedDebugRuntimeStats)
    {
      char buffer2[2048] = {0};
      vTaskGetRunTimeStats(buffer2);
      Serial.println(buffer2);
    }

    if (detailedDebugCAN)
    {
      uint32_t alerts_triggered;
      twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(0));
      twai_status_info_t twaistatus;
      twai_get_status_info(&twaistatus);
      DEBUG("");                 // this is the lock %
      DEBUG("CAN-BUS Details:"); // this is the lock %
      DEBUG("    RX buffered: %lu\t", twaistatus.msgs_to_rx);
      DEBUG("    RX missed: %lu\t", twaistatus.rx_missed_count);
      DEBUG("    RX overrun %lu\n", twaistatus.rx_overrun_count);

      if (alerts_triggered & TWAI_ALERT_ERR_ACTIVE)
      {
        isBusFailure = true;
      }
      if (alerts_triggered & TWAI_ALERT_RECOVERY_IN_PROGRESS)
      {
        isBusFailure = true;
      }
      if (alerts_triggered & TWAI_ALERT_ABOVE_ERR_WARN)
      {
        isBusFailure = true;
      }
      if (alerts_triggered & TWAI_ALERT_BUS_ERROR)
      {
        isBusFailure = true;
      }
      if (alerts_triggered & TWAI_ALERT_TX_FAILED)
      {
        isBusFailure = true;
      }
      if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL)
      {
        isBusFailure = true;
      }
      if (alerts_triggered & TWAI_ALERT_ERR_PASS)
      {
        isBusFailure = true;
      }
      if (alerts_triggered & TWAI_ALERT_BUS_OFF)
      {
        isBusFailure = true;
      }
      if (alerts_triggered & TWAI_ALERT_RX_FIFO_OVERRUN)
      {
        isBusFailure = true;
      }
    }

    vTaskDelay(serialMonitorRefresh / portTICK_PERIOD_MS);
  }
}
