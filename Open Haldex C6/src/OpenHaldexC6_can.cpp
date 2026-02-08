#include <OpenHaldexC6_can.h>
#include <OpenHaldexC6_Calculations.h>
#include <OpenHaldexC6_Analyzer.h>

void broadcastOpenHaldex(void *arg) {
  while (1) {
#if detailedDebugStack
    stackbroadcastOpenHaldex = uxTaskGetStackHighWaterMark(NULL);
#endif

    // Analyzer mode keeps the CAN bridge alive but suppresses OpenHaldex broadcast frames.
    if (analyzerMode) {
      vTaskDelay(broadcastRefresh / portTICK_PERIOD_MS);
      continue;
    }

    // build up the 'OpenHaldex' frame for broadcasting back over CAN
    twai_message_t broadcast_frame;
    broadcast_frame.identifier = OPENHALDEX_BROADCAST_ID;
    broadcast_frame.extd = 0;
    broadcast_frame.rtr = 0;
    broadcast_frame.data_length_code = 8;
    broadcast_frame.data[0] = 0;
    broadcast_frame.data[1] = isGen1Standalone + isGen2Standalone + isGen4Standalone;
    broadcast_frame.data[2] = (uint8_t)received_haldex_engagement_raw;
    broadcast_frame.data[3] = (uint8_t)lock_target;
    broadcast_frame.data[4] = received_vehicle_speed;
    broadcast_frame.data[5] = state.mode_override;
    broadcast_frame.data[6] = (uint8_t)state.mode;
    broadcast_frame.data[7] = (uint8_t)received_pedal_value;

    twai_transmit_v2(twai_bus_0, &broadcast_frame, (10 / portTICK_PERIOD_MS));

    vTaskDelay(broadcastRefresh / portTICK_PERIOD_MS);
  }
}

// Helper for analyzer pass-through: forward a frame exactly as received.
static void transmitFrameCopy(twai_handle_t bus, const twai_message_t &src, twai_message_t &scratch) {
  scratch = src;
  scratch.extd = src.extd;
  scratch.rtr = src.rtr;
  scratch.data_length_code = src.data_length_code;
  twai_transmit_v2(bus, &scratch, (10 / portTICK_PERIOD_MS));
}

void parseCAN_chs(void *arg) {
  while (1) {
#if detailedDebugStack
    stackCHS = uxTaskGetStackHighWaterMark(NULL);
#endif

    while (twai_receive_v2(twai_bus_0, &rx_message_chs, 0) == ESP_OK) {
      lastCANChassisTick = millis();

      tx_message_hdx.identifier = rx_message_chs.identifier;

      // Analyzer mode: queue for GVRET/SLCAN and forward untouched, skipping control logic.
      if (analyzerMode) {
        analyzerQueueFrame(rx_message_chs, 0);
        transmitFrameCopy(twai_bus_1, rx_message_chs, tx_message_hdx);
        continue;
      }

      // check to see if we're in standalone - and therefore ignore ALL CAN frames, EXCEPT diag. ones
      if (isGen1Standalone || isGen2Standalone || isGen4Standalone) {
        switch (rx_message_chs.identifier) {
          case diagnostics_1_ID:
          case diagnostics_2_ID:
          case diagnostics_3_ID:
          case diagnostics_4_ID:
          case diagnostics_5_ID:
            tx_message_hdx = rx_message_chs;
            tx_message_hdx.extd = rx_message_chs.extd;
            tx_message_hdx.rtr = rx_message_chs.rtr;
            tx_message_hdx.data_length_code = rx_message_chs.data_length_code;
            twai_transmit_v2(twai_bus_1, &tx_message_hdx, (10 / portTICK_PERIOD_MS));
            break;
        }
      }

      // check to see if we're NOT in standalone - and look to edit the frames if necessary
      if (!isGen1Standalone && !isGen2Standalone && !isGen4Standalone) {
        switch (rx_message_chs.identifier) {
          case MOTOR1_ID:
            received_pedal_value = rx_message_chs.data[5] * 0.4;
            received_vehicle_rpm = ((rx_message_chs.data[3] << 8) | rx_message_chs.data[2]) * 0.25;
            break;

          case MOTOR2_ID:
            received_vehicle_speed = rx_message_chs.data[3] * 128 / 100;
            break;

          case OPENHALDEX_EXTERNAL_CONTROL_ID:
            if (rx_message_chs.data[0] < (uint8_t)openhaldex_mode_t_MAX && rx_message_chs.data[0] != (uint8_t)MODE_CUSTOM) {
              state.mode = (openhaldex_mode_t)rx_message_chs.data[0];
            }
            break;
        }

        // Edit the CAN frame, if not in STOCK mode
        if (state.mode != MODE_STOCK) {
          if (haldexGeneration == 1 || haldexGeneration == 2 || haldexGeneration == 4) {
            getLockData(rx_message_chs);
          }
        } else {
          lock_target = 0;
        }

        tx_message_hdx = rx_message_chs;
        tx_message_hdx.extd = rx_message_chs.extd;
        tx_message_hdx.rtr = rx_message_chs.rtr;
        tx_message_hdx.data_length_code = rx_message_chs.data_length_code;
        twai_transmit_v2(twai_bus_1, &tx_message_hdx, (10 / portTICK_PERIOD_MS));
      }
    }

    vTaskDelay(1);
  }
}

void parseCAN_hdx(void *arg) {
  while (1) {
#if detailedDebugStack
    stackHDX = uxTaskGetStackHighWaterMark(NULL);
#endif

    while (twai_receive_v2(twai_bus_1, &rx_message_hdx, 0) == ESP_OK) {
      lastCANHaldexTick = millis();

      // Analyzer mode: queue for GVRET/SLCAN and forward untouched, skipping control logic.
      if (analyzerMode) {
        analyzerQueueFrame(rx_message_hdx, 1);
        twai_transmit_v2(twai_bus_0, &rx_message_hdx, (10 / portTICK_PERIOD_MS));
        continue;
      }

      // different generations have different ranges of 'applied lock', so scale them to suit
      if (haldexGeneration == 1) {
        received_haldex_engagement_raw = rx_message_hdx.data[1];
        received_haldex_engagement = map(received_haldex_engagement_raw, 128, 198, 0, 100);
      }

      if (haldexGeneration == 2) {
        received_haldex_engagement_raw = rx_message_hdx.data[1] + rx_message_hdx.data[4];
        received_haldex_engagement = map(received_haldex_engagement_raw, 128, 255, 0, 100);
      }

      if (haldexGeneration == 4) {
        received_haldex_engagement_raw = rx_message_hdx.data[1];
        received_haldex_engagement = map(received_haldex_engagement_raw, 128, 255, 0, 100);
      }
      received_haldex_state = rx_message_hdx.data[0];

      // Decode the state byte.
      received_report_clutch1 = (received_haldex_state & (1 << 0));
      received_temp_protection = (received_haldex_state & (1 << 1));
      received_report_clutch2 = (received_haldex_state & (1 << 2));
      received_coupling_open = (received_haldex_state & (1 << 3));
      received_speed_limit = (received_haldex_state & (1 << 6));

      twai_transmit_v2(twai_bus_0, &rx_message_hdx, (10 / portTICK_PERIOD_MS));
    }

    vTaskDelay(1);
  }
}
