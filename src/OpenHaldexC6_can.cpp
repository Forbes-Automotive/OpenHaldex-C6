#include <OpenHaldexC6_can.h>
#include <OpenHaldexC6_Calculations.h>
#include <OpenHaldexC6_Analyzer.h>

void broadcastOpenHaldex(void *arg)
{
  while (1)
  {
#if detailedDebugStack
    stackbroadcastOpenHaldex = uxTaskGetStackHighWaterMark(NULL);
#endif

    // Analyzer mode keeps the CAN bridge alive but suppresses OpenHaldex broadcast frames.
    if (analyzerMode)
    {
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
    broadcast_frame.data[1] = isStandalone;
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
static void transmitFrameCopy(twai_handle_t bus, const twai_message_t &src, twai_message_t &scratch)
{
  scratch = src;
  scratch.extd = src.extd;
  scratch.rtr = src.rtr;
  scratch.data_length_code = src.data_length_code;
  twai_transmit_v2(bus, &scratch, (10 / portTICK_PERIOD_MS));
}

void setupCAN()
{
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(gpio_num_t(CAN0_TX), gpio_num_t(CAN0_RX), TWAI_MODE_NO_ACK); // TWAI_MODE_NORMAL, TWAI_MODE_NO_ACK or TWAI_MODE_LISTEN_ONLY
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();                                                            // default CAN speed
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();                                                          // accept all messages

  // g_config.intr_flags = ESP_INTR_FLAG_LOWMED;  //Optional - move canbus irq to free up the default level 1 IRQ it will take up.  Todo
  g_config.tx_queue_len = 1024; //<TWAI_GENERAL_CONFIG_DEFAULT default is 5, use this to increase if needed
  g_config.rx_queue_len = 2048; //<TWAI_GENERAL_CONFIG_DEFAULT default is 5, use this to increase if needed // 4096
  // g_config.intr_flags = ESP_INTR_FLAG_IRAM;

  // setup CAN Controller (0) - Chassis
  g_config.controller_id = 0;
  ESP_ERROR_CHECK(twai_driver_install_v2(&g_config, &t_config, &f_config, &twai_bus_0));
  DEBUG("CAN - Driver 0 Installed");
  ESP_ERROR_CHECK(twai_start_v2(twai_bus_0));
  DEBUG("CAN - Driver 0 Started");

  // setup CAN Controller (1) - Haldex
  g_config.controller_id = 1;
  g_config.tx_io = gpio_num_t(CAN1_TX);
  g_config.rx_io = gpio_num_t(CAN1_RX);
  ESP_ERROR_CHECK(twai_driver_install_v2(&g_config, &t_config, &f_config, &twai_bus_1));
  DEBUG("CAN - Driver 1 Installed");
  ESP_ERROR_CHECK(twai_start_v2(twai_bus_1));
  DEBUG("CAN - Driver 1 Started");

  // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK)
  {
    DEBUG("Reconfiguration of CAN alerts");
  }
  else
  {
    DEBUG("Failed to reconfigure CAN alerts!");
    return;
  }
}

void parseCAN_chs(void *arg)
{
  while (1)
  {
#if detailedDebugStack
    stackCHS = uxTaskGetStackHighWaterMark(NULL);
#endif

    while (twai_receive_v2(twai_bus_0, &rx_message_chs, 0) == ESP_OK)
    {
      lastCANChassisTick = millis();

      tx_message_hdx.identifier = rx_message_chs.identifier;

      // Analyzer mode: queue for GVRET/SLCAN and forward untouched, skipping control logic.
      if (analyzerMode)
      {
        analyzerQueueFrame(rx_message_chs, 0);
        transmitFrameCopy(twai_bus_1, rx_message_chs, tx_message_hdx);
        continue;
      }

      // check to see if we're in standalone - and therefore ignore ALL CAN frames, EXCEPT diag. ones
      if (isStandalone)
      {
        switch (rx_message_chs.identifier)
        {
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
        continue;
      }

      // check to see if we're NOT in standalone - and look to edit the frames if necessary
      if (!isStandalone)
      {
        switch (rx_message_chs.identifier)
        {
        case MOTOR1_ID:
          // PQ Platform: capture basic pedal / rpm data
          received_pedal_value = rx_message_chs.data[5] * 0.4;
          received_vehicle_rpm = ((rx_message_chs.data[3] << 8) | rx_message_chs.data[2]) * 0.25;
          break;

        case MOTOR2_ID:
          // PQ Platform: only if ABS speed is missing
          if ((!isABSValid || (millis() - lastABSResponse) > absTimeout)) // if we haven't received a valid ABS response within the timeout, use this speed value instead (which is less accurate but better than nothing)
          {
            received_vehicle_speed = rx_message_chs.data[3] * 128 / 100;
          }
          break;

        case MOTOR_04:
        {
          // MQB Motor_04 (0x107) / MO_Ladedruck: start bit 39, length 9, little-endian, factor 0.01 bar (absolute).
          // vw_mqb.dbc: SG_ MO_Ladedruck : 39|9@1+ (0.01,0) [0|5.1] "Unit_Bar"
          // LSB at bit 39 = data[4] bit 7; bits 1-8 = data[5] bits 0-7.
          const uint16_t mo_ladedruck_raw = (uint16_t)(((uint16_t)rx_message_chs.data[5] << 1) | (rx_message_chs.data[4] >> 7)) & 0x01FF;
          const int32_t boost_mbar = (int32_t)(mo_ladedruck_raw * 10.0f + 0.5f) - 1000;
          received_vehicle_boost = (boost_mbar > 0) ? (uint16_t)boost_mbar : 0;
          break;
        }

        case MOTOR_12:
        {
          // MQB Motor_12 (0x0A8) / MO_Drehzahl_01: start bit 48, length 16, little-endian, factor 0.25 RPM/bit.
          // vw_mqb.dbc: SG_ MO_Drehzahl_01 : 48|16@1+ (0.25,0) [0|16383] "Unit_MinutInver"
          // Sent directly to SAK_MQB (Haldex). bytes 6+7 little-endian.
          const uint16_t mo_drehzahl_raw = ((uint16_t)rx_message_chs.data[7] << 8) | rx_message_chs.data[6];
          received_vehicle_rpm = (uint16_t)(mo_drehzahl_raw * 0.25f + 0.5f);
          break;
        }
        
        case MOTOR7_ID:
        {
          // PQ Motor_7 / Ladedruck: start bit 32, length 8, little-endian, factor 0.01 bar (absolute).
          const uint8_t pq_ladedruck_raw = rx_message_chs.data[4];
          const int32_t boost_mbar = (int32_t)(pq_ladedruck_raw * 10.0f + 0.5f) - 1000;
          received_vehicle_boost = (boost_mbar > 0) ? (uint16_t)boost_mbar : 0;
          break;
        }

        case MOTOR_20:
          // MQB platform: MO_Fahrpedalrohwert_01 is bits 12..19 with 0.4%/bit scaling.
          received_pedal_value = ((rx_message_chs.data[1] >> 4) | ((rx_message_chs.data[2] & 0x0F) << 4)) * 0.4f;
          break;

        case MOTOR_14:
          // MQB platform: MO_Kickdown is byte 5 bit 0 in the captured frames.
          received_kickdown = bitRead(rx_message_chs.data[5], 0);
          break;

        case BRAKES1_ID:
        {
          // PQ Platform: capture ASR/ESP disable flags and speed as fallback if ECU speed missing
          asrForceModeFlag = bitRead(rx_message_chs.data[7], 4); // from PQ - Byte 7 (BR1_ASR_passiv) - ASR disabled by driver
          tcForceModeFlag = bitRead(rx_message_chs.data[7], 5);  // from PQ - Byte 7 (BR1_ESPASR_passiv) - ESP disabled by driver

          // PQ BR1_Rad_kmh: start bit 17, length 15, little-endian, factor 0.01 km/h
          const uint16_t br1_speed_raw = (((uint16_t)rx_message_chs.data[3] << 8) | rx_message_chs.data[2]) >> 1;
          received_vehicle_speed = (uint16_t)(br1_speed_raw * 0.01f + 0.5f);
          break;
        }

        case BRAKES3_ID:
        {
          // PQ Platform: capture ASR/ESP disable flags and speed as fallback if ECU speed missing
          // PQ Radgeschw__VL_4_1: start bit 1, length 15, little-endian, factor 0.01 km/h
          const uint16_t br3_speed_raw = (((uint16_t)rx_message_chs.data[1] << 8) | rx_message_chs.data[0]) >> 1;
          received_vehicle_speed = (uint16_t)(br3_speed_raw * 0.01f + 0.5f);

          isABSValid = true;
          lastABSResponse = millis();
          break;
        }

        case ESP_19:
        {
          // MQB platform: four wheel speeds in km/h with 0.0075 km/h per bit.
          const uint16_t wheel_speed_hl_raw = (rx_message_chs.data[1] << 8) | rx_message_chs.data[0];
          const uint16_t wheel_speed_hr_raw = (rx_message_chs.data[3] << 8) | rx_message_chs.data[2];
          const uint16_t wheel_speed_vl_raw = (rx_message_chs.data[5] << 8) | rx_message_chs.data[4];
          const uint16_t wheel_speed_vr_raw = (rx_message_chs.data[7] << 8) | rx_message_chs.data[6];
          const float average_wheel_speed =
              (wheel_speed_hl_raw + wheel_speed_hr_raw + wheel_speed_vl_raw + wheel_speed_vr_raw) * (0.0075f / 4.0f);

          received_vehicle_speed = (uint16_t)(average_wheel_speed + 0.5f);
          isABSValid = true;
          lastABSResponse = millis();
          break;
        }

        case ESP_21:
        {
          // MQB platform: ASR_Tastung_passiv is bit 0 of byte 6, ESP_Keying_passiv is bit 1 of byte 6
          asrForceModeFlag = bitRead(rx_message_chs.data[6], 0); // from MQB - Byte 6 Bit 0 (ASR_Tastung_passiv) - ASR passive
          tcForceModeFlag = bitRead(rx_message_chs.data[6], 1);  // from MQB - Byte 6 (ESP_Keying_passive) - ESP disabled by driver

          // MQB fallback: ESP_v_Signal is a 16-bit little-endian speed value with 0.01 km/h per bit.
          received_vehicle_speed = (uint16_t)((((rx_message_chs.data[5] << 8) | rx_message_chs.data[4]) * 0.01f) + 0.5f);
          isABSValid = true;
          lastABSResponse = millis();
          break;
        }

        case GETRIEBE_17:
        {
          // MQB: GE_TippZustand at bits 16-17 (byte 2 bits 0-1), Getriebe_17 / 0xB1
          // 0=Init_No_Tip, 1=Tip_permanent (sustained/both), 2=Tip_temporaer (single press)
          const uint8_t tippZustand = rx_message_chs.data[2] & 0x03;
          paddleTipActive = (tippZustand > 0);
          paddleTipBoth = (tippZustand == 1); // Tip_permanent: sustained hold or both paddles
          break;
        }

        case GETRIEBE_11:
        {
          // MQB: GE_Zielgang at bits 60-63 (byte 7 bits 4-7), Getriebe_11 / 0xAD
          // Direction inferred: zielgang increase while tip active = upshift (right paddle);
          //                     zielgang decrease while tip active = downshift (left paddle).
          // GRA_Tip_Hoch/Runter (0x12B) and MFL_Tip_Up/Down (0x3DD) are absent from this bus.
          static uint8_t lastZielgang = 0;
          const uint8_t zielgang = (rx_message_chs.data[7] >> 4) & 0x0F;
          if (paddleTipActive)
          {
            paddleTipUp = (zielgang > lastZielgang);
            paddleTipDown = (zielgang < lastZielgang);
          }
          else
          {
            paddleTipUp = false;
            paddleTipDown = false;
          }
          lastZielgang = zielgang;
          break;
        }

        case OPENHALDEX_EXTERNAL_CONTROL_ID:
          if (rx_message_chs.data[0] < (uint8_t)openhaldex_mode_t_MAX)
          {
            state.mode = (openhaldex_mode_t)rx_message_chs.data[0];
          }
          break;
        }

        if (1) // find out what this does(!) rx_message_chs.identifier != diagnostics_5_ID
        {
          // Edit the CAN frame, if not in STOCK mode
          if (state.mode != MODE_STOCK)
          {
            if (haldexGeneration == 1 || haldexGeneration == 2 || haldexGeneration == 4 || haldexGeneration == 5 || haldexGeneration == 41)
            {
              getLockData(rx_message_chs);
            }
          }
          else
          {
            lock_target = received_haldex_engagement;

            if (extBtnForceMode || tcForceMode)
            {
              if (extButtonForceModeFlag || tcForceModeFlag) // if either flag is active, return the forced lock value, otherwise return 0
              {
                getLockData(rx_message_chs);
              }
            }
          }

          tx_message_hdx = rx_message_chs;
          tx_message_hdx.extd = rx_message_chs.extd;
          tx_message_hdx.rtr = rx_message_chs.rtr;
          tx_message_hdx.data_length_code = rx_message_chs.data_length_code;
          twai_transmit_v2(twai_bus_1, &tx_message_hdx, (10 / portTICK_PERIOD_MS));
        }
      }
    }

    vTaskDelay(1);
  }
}

void parseCAN_hdx(void *arg)
{
  while (1)
  {
#if detailedDebugStack
    stackHDX = uxTaskGetStackHighWaterMark(NULL);
#endif

    while (twai_receive_v2(twai_bus_1, &rx_message_hdx, 0) == ESP_OK)
    {
      lastCANHaldexTick = millis();

      // Analyzer mode: queue for GVRET/SLCAN and forward untouched, skipping control logic.
      if (analyzerMode)
      {
        analyzerQueueFrame(rx_message_hdx, 1);
        twai_transmit_v2(twai_bus_0, &rx_message_hdx, (10 / portTICK_PERIOD_MS));
        continue;
      }

      // different generations have different ranges of 'applied lock', so scale them to suit
      if (haldexGeneration == 1)
      {
        received_haldex_engagement_raw = rx_message_hdx.data[1];
        received_haldex_engagement = map(received_haldex_engagement_raw, 128, 198, 0, 100);
      }

      if (haldexGeneration == 2)
      {
        received_haldex_engagement_raw = rx_message_hdx.data[1] + rx_message_hdx.data[4];
        received_haldex_engagement = map(received_haldex_engagement_raw, 128, 255, 0, 100);
      }

      if (haldexGeneration == 4)
      {
        received_haldex_engagement_raw = rx_message_hdx.data[1];
        received_haldex_engagement = map(received_haldex_engagement_raw, 128, 255, 0, 100);
      }

      if (haldexGeneration == 5)
      {
        if (rx_message_hdx.identifier == HALDEX_ID_GEN5) // because it sends TWO types of IDs(!)
        {
          received_haldex_engagement_raw = rx_message_hdx.data[2];
          received_haldex_engagement = map(received_haldex_engagement_raw, 0, 250, 0, 100);
          received_haldex_state = rx_message_hdx.data[3]; // this was out the loop (where the below says 0?)
        }
      }
      else
      {
        received_haldex_state = rx_message_hdx.data[0];
      }

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