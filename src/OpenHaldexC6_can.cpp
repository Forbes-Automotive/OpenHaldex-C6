#include <OpenHaldexC6_can.h>
#include <OpenHaldexC6_Calculations.h>
#include <OpenHaldexC6_Analyzer.h>
#include <OpenHaldexC6_UDS.h>

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

// Brake/handbrake override applied to chassis frames before forwarding to the
// Haldex bus. When `followBrake`/`followHandbrake` is enabled, the bit on the
// outgoing frame is rewritten to mirror (or, with `invertBrake`/`invertHandbrake`,
// invert) the live `brakeFromCAN` / `handbrakeFromCAN` value decoded by the
// switch above. With follow=OFF the frame is forwarded untouched.
//
// Bit positions (verified against vw_pq.dbc / vw_mqb.dbc):
//   PQ Gen2/4: Motor_2  (0x288) byte 2 bit 0  - MO2_BLS         (no CRC)
//   MQB Gen5:  ESP_05   (0x106) byte 3 bit 2  - ESP_Fahrer_bremst (CRC at byte 0 via ID_SEQ_106)
//   MQB Gen5:  Kombi_01 (0x30B) byte 2 bit 7  - KBI_Handbremse   (no CRC)
static void applyBrakeHandbrakeCANOverride(twai_message_t &m)
{
  switch (haldexGeneration)
  {
  case 2:
  case 4:
  case 51:
    if (followBrake && m.identifier == MOTOR2_ID && m.data_length_code >= 3)
    {
      const bool out = invertBrake ? !brakeFromCAN : brakeFromCAN;
      m.data[2] = (uint8_t)((m.data[2] & ~0x01) | (out ? 0x01 : 0x00));
    }
    // Gen51 (0AY) also carries parking brake on Kombi_01 (0x30B) byte 2 bit 7.
    if (m.identifier == KOMBI_01 && m.data_length_code >= 3)
    {
      if (followHandbrake && haldexGeneration == 51)
      {
        const bool out = invertHandbrake ? !handbrakeFromCAN : handbrakeFromCAN;
        m.data[2] = (uint8_t)((m.data[2] & ~0x80) | (out ? 0x80 : 0x00));
      }
    }
    break;

  case 50:
    if (followBrake && m.identifier == ESP_05 && m.data_length_code >= 8)
    {
      const bool out = invertBrake ? !brakeFromCAN : brakeFromCAN;
      m.data[3] = (uint8_t)((m.data[3] & ~0x04) | (out ? 0x04 : 0x00));
      // Recompute AUTOSAR CRC8 at byte 0 - counter nibble at byte 1 already
      // matches the original sender, so the same ID_SEQ_106 lookup applies.
      m.data[0] = calcChecksum(m.data, ID_SEQ_106);
    }
    if (followHandbrake && m.identifier == KOMBI_01 && m.data_length_code >= 3)
    {
      const bool out = invertHandbrake ? !handbrakeFromCAN : handbrakeFromCAN;
      m.data[2] = (uint8_t)((m.data[2] & ~0x80) | (out ? 0x80 : 0x00));
      // Kombi_01 carries no CRC byte (byte 0 is fully populated with KBI lamp flags).
    }
    break;

  default:
    break;
  }
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

  // Allow the TWAI power domain to be powered down during light sleep; the
  // driver auto-saves/restores its registers + RX queue across sleep cycles.
  // Sleep entry itself is gated at runtime by `canSleepEnabled`.
  g_config.general_flags.sleep_allow_pd = 1;

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
  // Interrupt-driven receive: block on the TWAI driver's RX queue (signalled
  // from the TWAI ISR) instead of polling with vTaskDelay(1). The task is
  // unblocked the moment a frame is queued by the ISR, so latency is the queue
  // wakeup time (microseconds) rather than the previous ~1 ms polling tick.
  // CPU usage on an idle bus drops to effectively zero.
  while (1)
  {
#if detailedDebugStack
    stackCHS = uxTaskGetStackHighWaterMark(NULL);
#endif

    // Block until at least one frame is available, then read any further
    // frames already queued by the ISR before blocking again.
    if (twai_receive_v2(twai_bus_0, &rx_message_chs, portMAX_DELAY) != ESP_OK)
    {
      // Driver was stopped/uninstalled (e.g. light-sleep entry); back off and retry.
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    do
    {
      lastCANChassisTick = millis();
      ++lpChassisFrameCount;

      tx_message_hdx.identifier = rx_message_chs.identifier;

      // Gen41 Haldex-originated Bus0 heartbeats - capture presence & timestamp.
      // Done before any mode gating so the API can show liveness even in standalone.
      if (haldexGeneration == 41)
      {
        if (rx_message_chs.identifier == HALDEX_GEN41_SEC_AXLE_GENINFO_ID)
        {
          received_haldex_alive_bus0 = true;
          received_haldex_alive_bus0_ms = millis();
        }
        else if (rx_message_chs.identifier == HALDEX_GEN41_DRIVETRAIN_STATE_ID)
        {
          received_drivetrain_state_ok = true;
          received_drivetrain_state_ms = millis();
        }
      }

      // Analyzer mode: queue for GVRET/SLCAN and forward untouched, skipping control logic.
      if (analyzerMode || analyzerSerial)
      {
        analyzerQueueFrame(rx_message_chs, 0);
        transmitFrameCopy(twai_bus_1, rx_message_chs, tx_message_hdx);
        continue;
      }

      if (!isStandalone || useCANifAvailable)
      { // process frames regardless
        switch (rx_message_chs.identifier)
        {
        case MOTOR1_ID:
          // PQ Motor_1 (0x280) - engine ECU broadcast frame.
          // vw_pq.dbc:
          //   SG_ Motordrehzahl                      : 16|16@1+ (0.25,0)  "U/min"  -> bytes 2..3 LE, engine RPM
          //   SG_ Fahrpedalwert_oder_Drosselklapp    : 40|8@1+  (0.4,0)   "%"     -> byte 5,    accelerator pedal / throttle position
          //   SG_ inneres_Motor_Moment(_ohne_extern) : 8/32|8@1+(0.39,0)  "MDI"   -> bytes 1,4, indicated engine torque
          //   SG_ Fahrerwunschmoment                 : 56|8@1+  (0.39,0)  "MDI"   -> byte 7,    driver-requested torque
          //   SG_ Kickdownschalter                   : 2|1                          -> kickdown switch
          // Capture pedal % (data[5] * 0.4) and engine RPM (LE 16-bit * 0.25).
          received_pedal_value = rx_message_chs.data[5] * 0.4;
          received_vehicle_rpm = ((rx_message_chs.data[3] << 8) | rx_message_chs.data[2]) * 0.25;
          break;

        case MOTOR2_ID:
          // PQ Motor_2 (0x288) - secondary engine ECU broadcast.
          // Used here only as a *fallback* coarse vehicle-speed source if the ABS
          // (Bremse_3 / Bremse_1) is silent. The legacy ESP32 codebase used
          // data[3] as a vehicle-speed proxy with a fudge factor (* 128/100) - this
          // does not exactly match any single DBC signal but produces "close enough"
          // km/h until ABS comes online.
          if ((!isABSValid || (millis() - lastABSResponse) > absTimeout)) // if we haven't received a valid ABS response within the timeout, use this speed value instead (which is less accurate but better than nothing)
          {
            received_vehicle_speed = rx_message_chs.data[3] * 128 / 100;
          }
          // PQ brake-light switch broadcast - vw_pq.dbc:
          //   SG_ MO2_BLS : 16|1@1+   -> byte 2 bit 0 ("Brake light switch - unfiltered raw signal")
          // Captured for diag display and as the source for the optional
          // followBrake / invertBrake CAN override applied before forwarding.
          brakeFromCAN = bitRead(rx_message_chs.data[2], 0);
          break;

        case ESP_05:
          // MQB ESP_05 (0x106) - brake-pressure / brake-pedal broadcast (vw_mqb.dbc):
          //   SG_ ESP_Fahrer_bremst : 26|1@1+   -> byte 3 bit 2, "driver is braking"
          // Captured for diag and used as the source for the optional CAN override.
          if (rx_message_chs.data_length_code >= 4)
          {
            brakeFromCAN = bitRead(rx_message_chs.data[3], 2);
          }
          break;

        case KOMBI_01:
          // MQB Kombi_01 (0x30B) - instrument cluster broadcast (vw_mqb.dbc):
          //   SG_ KBI_Handbremse : 23|1@1+     -> byte 2 bit 7, parking brake engaged
          // PQ handbrake stays on the physical IO path; this case is MQB-only.
          if (rx_message_chs.data_length_code >= 3)
          {
            handbrakeFromCAN = bitRead(rx_message_chs.data[2], 7);
          }
          break;

        case MOTOR_04:
        {
          // MQB Motor_04 (0x107) - engine ECU torque/charge broadcast.
          // vw_mqb.dbc / MQB_FCAN K-matrix:
          //   SG_ MO_Ladedruck : 39|9@1+ (0.01,0) [0|5.1] "Unit_Bar"
          // i.e. absolute boost pressure (Ladedruck = boost pressure) in bar.
          // 9-bit little-endian: LSB at bit 39 = data[4] bit 7; bits 1..8 = data[5] bits 0..7.
          // Convert absolute -> gauge mbar: (raw * 10) - 1000 ; clamp to >=0.
          const uint16_t mo_ladedruck_raw = (uint16_t)(((uint16_t)rx_message_chs.data[5] << 1) | (rx_message_chs.data[4] >> 7)) & 0x01FF;
          const int32_t boost_mbar = (int32_t)(mo_ladedruck_raw * 10.0f + 0.5f) - 1000;
          received_vehicle_boost = (boost_mbar > 0) ? (uint16_t)boost_mbar : 0;
          break;
        }

        case MOTOR_12:
        {
          // MQB Motor_12 (0x0A8) - high-rate engine speed/torque broadcast.
          // vw_mqb.dbc:
          //   SG_ MO_Drehzahl_01 : 48|16@1+ (0.25,0) [0|16383] "Unit_MinutInver"
          // i.e. engine speed (Drehzahl = revs) in RPM, factor 0.25/bit, 16-bit LE.
          // Sent as a receiver to SAK_MQB (Haldex). bytes 6+7 little-endian.
          const uint16_t mo_drehzahl_raw = ((uint16_t)rx_message_chs.data[7] << 8) | rx_message_chs.data[6];
          received_vehicle_rpm = (uint16_t)(mo_drehzahl_raw * 0.25f + 0.5f);
          break;
        }

        case MOTOR7_ID:
        {
          // PQ Motor_7 (0x588) - engine boost/charge broadcast.
          //   SG_ Ladedruck (boost pressure) : 32|8@1+ (0.01,0) bar (absolute).
          // 8-bit absolute: gauge mbar = (raw * 10) - 1000 ; clamp to >=0.
          const uint8_t pq_ladedruck_raw = rx_message_chs.data[4];
          const int32_t boost_mbar = (int32_t)(pq_ladedruck_raw * 10.0f + 0.5f) - 1000;
          received_vehicle_boost = (boost_mbar > 0) ? (uint16_t)boost_mbar : 0;
          break;
        }

        case MOTOR_20:
          // MQB Motor_20 (0x121) - accelerator/raw pedal broadcast.
          //   SG_ MO_Fahrpedalrohwert_01 : 12|8@1+ (0.4,0) "%"
          // (Fahrpedalrohwert = raw accelerator pedal value)
          // 8-bit value straddles bytes 1..2: LSB at bit 12 = data[1] bit 4; bits 1..7 = data[2] bits 0..6.
          received_pedal_value = ((rx_message_chs.data[1] >> 4) | ((rx_message_chs.data[2] & 0x0F) << 4)) * 0.4f;
          break;

        case MOTOR_14:
          // MQB Motor_14 (0x3BE) - low-rate engine state/event broadcast.
          //   SG_ MO_Kickdown : byte 5 bit 0 (in our captured frames)
          // Kickdown = full-throttle floored switch latch.
          received_kickdown = bitRead(rx_message_chs.data[5], 0);
          break;

        case mGate_Komf_1:
        {
          // PQ Gate_Komf_1 (0x390) - gateway body / comfort signals.
          // vw_golf_mk4.dbc:
          //   SG_ GK1_Warnblk_Status : 55|1@1+ -> byte 6 bit 7, hazard warning lights active
          // Used to drive hazard-light force-mode when enabled.
          hazardForceModeFlag = bitRead(rx_message_chs.data[6], 7);
          break;
        }

        case GATEWAY_72:
        {
          // MQB Gateway_72 (0x3DB) - gateway body lighting state.
          // Note: on at least some MQB clusters the gateway frame does NOT
          // reflect hazard state when active, so we now decode hazard from
          // Blinkmodi_02 (0x366) below instead. This case is kept as a stub
          // in case other body signals are needed here later.
          break;
        }

        case BLINKMODI_02:
        {
          // MQB Blinkmodi_02 (0x366) - turn-signal / hazard switch state.
          // vw_mqb.dbc:
          //   SG_ Hazard_Switch : 20|1@1+   -> byte 2 bit 4
          //   CM_ "Four-way flashers active"
          hazardForceModeFlag = bitRead(rx_message_chs.data[2], 4);
          break;
        }

        case BRAKES1_ID:
        {
          // PQ Bremse_1 (0x1A0) - ABS/ESP main broadcast (Bremse = brake).
          // vw_pq.dbc:
          //   SG_ BR1_ASR_passiv      : 60|1   -> byte 7 bit 4, ASR disabled by driver
          //   SG_ BR1_ESPASR_passiv   : 61|1   -> byte 7 bit 5, ESP+ASR disabled by driver
          //   SG_ BR1_Rad_kmh         : 17|15@1+ (0.01,0) "km/h"  -> wheel speed (>>1 of bytes 2..3 LE)
          //   SG_ BR1_Lampe_ABS / BR1_Lampe_ASR / BR1_Lampe_BK    -> warning lamps (bits 8..10)
          asrForceModeFlag = bitRead(rx_message_chs.data[7], 4); // PQ - Byte 7 (BR1_ASR_passiv)    - ASR disabled by driver
          tcForceModeFlag = bitRead(rx_message_chs.data[7], 5);  // PQ - Byte 7 (BR1_ESPASR_passiv) - ESP disabled by driver

          // BR1_Rad_kmh: 15-bit @ start bit 17, factor 0.01 km/h.
          // start bit 17 in LE = data[2] bit 1; therefore raw = ((data[3]<<8)|data[2]) >> 1.
          const uint16_t br1_speed_raw = (((uint16_t)rx_message_chs.data[3] << 8) | rx_message_chs.data[2]) >> 1;
          received_vehicle_speed = (uint16_t)(br1_speed_raw * 0.01f + 0.5f);
          break;
        }

        case BRAKES3_ID:
        {
          // PQ Bremse_3 (0x4A0) - individual wheel speeds broadcast.
          // vw_pq.dbc:
          //   SG_ Radgeschw__VL_4_1 : 1|15@1+ (0.01,0) "km/h"  -> front-left wheel speed (Radgeschwindigkeit = wheel speed, VL = vorne links / front-left)
          //   plus VR / HL / HR equivalents at start bits 17/33/49.
          // We only read the front-left here as a representative speed then
          // mark ABS "valid" so that the Motor_2 fallback is suppressed.
          // start bit 1 in LE = data[0] bit 1; raw = ((data[1]<<8)|data[0]) >> 1.
          const uint16_t br3_speed_raw = (((uint16_t)rx_message_chs.data[1] << 8) | rx_message_chs.data[0]) >> 1;
          received_vehicle_speed = (uint16_t)(br3_speed_raw * 0.01f + 0.5f);

          isABSValid = true;
          lastABSResponse = millis();
          break;
        }

        case ESP_19:
        {
          // MQB ESP_19 (0x0B2) - per-wheel ABS speeds broadcast.
          // MQB FCAN K-matrix: four 16-bit LE wheel speeds (HL/HR/VL/VR) with
          // factor 0.0075 km/h per bit each.
          //   HL = Hinten Links  (rear left)   -> bytes 0..1
          //   HR = Hinten Rechts (rear right)  -> bytes 2..3
          //   VL = Vorne  Links  (front left)  -> bytes 4..5
          //   VR = Vorne  Rechts (front right) -> bytes 6..7
          // Average all four for a stable vehicle speed. ABS is then marked valid
          // so the Motor_2 fallback path stays suppressed.
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
          // MQB ESP_21 (0x0FD) - ESP/ASR mode + reference speed.
          // MQB FCAN K-matrix:
          //   SG_ ASR_Tastung_passiv (Tastung = keying/button)  -> byte 6 bit 0, ASR disabled by driver
          //   SG_ ESP_Tastung_passiv                            -> byte 6 bit 1, ESP disabled by driver
          //   SG_ ESP_v_Signal : 16@1+ (0.01,0) "km/h"          -> bytes 4..5 LE, vehicle reference speed
          asrForceModeFlag = bitRead(rx_message_chs.data[6], 0); // MQB - Byte 6 Bit 0 (ASR_Tastung_passiv) - ASR passive
          tcForceModeFlag = bitRead(rx_message_chs.data[6], 1);  // MQB - Byte 6 Bit 1 (ESP_Tastung_passiv) - ESP disabled by driver

          received_vehicle_speed = (uint16_t)((((rx_message_chs.data[5] << 8) | rx_message_chs.data[4]) * 0.01f) + 0.5f);
          isABSValid = true;
          lastABSResponse = millis();
          break;
        }

        case GETRIEBE_17:
        {
          // MQB Getriebe_17 (0x0B1) - DSG paddle/tip status (Getriebe = transmission).
          // MQB FCAN K-matrix:
          //   SG_ GE_TippZustand : 16|2@1+   -> byte 2 bits 0..1, paddle tip state
          //     0 = Init_No_Tip      (idle / no paddle)
          //     1 = Tip_permanent    (sustained hold or both paddles together)
          //     2 = Tip_temporaer    (single momentary press)
          // We don't get an explicit up/down on this bus, only the activity flag;
          // direction is inferred from Getriebe_11.GE_Zielgang transitions below.
          const uint8_t tippZustand = rx_message_chs.data[2] & 0x03;
          paddleTipActive = (tippZustand > 0);
          paddleTipBoth = (tippZustand == 1); // Tip_permanent: sustained hold or both paddles
          break;
        }

        case GETRIEBE_11:
        {
          // MQB Getriebe_11 (0x0AD) - DSG status broadcast.
          // MQB FCAN K-matrix:
          //   SG_ GE_Zielgang : 60|4@1+   -> byte 7 bits 4..7, target gear of in-progress shift
          // Direction is inferred: zielgang increase while tip active = upshift
          // (right paddle); zielgang decrease while tip active = downshift (left).
          // The dedicated GRA_Tip_Hoch/Runter (Hoch=up, Runter=down) at 0x12B and
          // MFL_Tip_Up/Down at 0x3DD are absent from this bus, hence the inference.
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
          // Only use external mode-change frames when broadcasting is
          // enabled - otherwise a stray frame on the chassis bus matching
          // OPENHALDEX_EXTERNAL_CONTROL_ID (e.g. data[0]==1 -> FWD) can flip
          // the mode unexpectedly.
          if (broadcastOpenHaldexOverCAN &&
              rx_message_chs.data[0] < (uint8_t)openhaldex_mode_t_MAX)
          {
            state.mode = (openhaldex_mode_t)rx_message_chs.data[0];
          }
          break;
        }
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
        if (1) // find out what this does(!) - rx_message_chs.identifier != diagnostics_5_ID
        {
          // Controller disabled = HARD passthrough. When disableController is set the
          // unit acts as a transparent bridge: it READS the chassis frame and forwards
          // it to the Haldex completely untouched - no getLockData(), no brake/handbrake
          // override, no force-flag handling, regardless of mode, learn state, or any
          // enabled/asserted flag. This is the global kill-switch for all frame work.
          if (!disableController)
          {
          // Edit the CAN frame, if not in STOCK mode (or if learn is active - learn must run regardless of mode)
          if (state.mode != MODE_STOCK || haldexLearnActive)
          {
            if (haldexGeneration == 1 || haldexGeneration == 2 || haldexGeneration == 4 || haldexGeneration == 50 || haldexGeneration == 51 || haldexGeneration == 41)
            {
              getLockData(rx_message_chs);
            }
          }
          else
          {
            // in stock mode and not learning
            lock_target = received_haldex_engagement;

            /*
                        if (extBtnForceMode || tcForceMode || hazardForceMode)
            {
              if (extButtonForceModeFlag || tcForceModeFlag || hazardForceModeFlag) // if either flag is active, return the forced lock value, otherwise return 0
              {
                getLockData(rx_message_chs);
              }
            }
          }
            */

            // Only force the lock when an *enabled* feature's own flag is active
            if ((extBtnForceMode && extButtonForceModeFlag) ||
                (tcForceMode && tcForceModeFlag) ||
                (hazardForceMode && hazardForceModeFlag))
            {
              getLockData(rx_message_chs);
            }
          }

          // Optional brake/handbrake override (followBrake/followHandbrake +
          // invertBrake/invertHandbrake). Changes rx_message_chs in place
          // before the copy so the Haldex sees the rewritten bit + valid CRC.
          applyBrakeHandbrakeCANOverride(rx_message_chs);
          }
          else
          {
            // Controller disabled: read-only. Mirror the Haldex's reported
            // engagement for telemetry only; rx_message_chs is left untouched
            // and forwarded verbatim below.
            lock_target = received_haldex_engagement;
          }

          tx_message_hdx = rx_message_chs;
          tx_message_hdx.extd = rx_message_chs.extd;
          tx_message_hdx.rtr = rx_message_chs.rtr;
          tx_message_hdx.data_length_code = rx_message_chs.data_length_code;
          twai_transmit_v2(twai_bus_1, &tx_message_hdx, (10 / portTICK_PERIOD_MS));
        }
      }
    } while (twai_receive_v2(twai_bus_0, &rx_message_chs, 0) == ESP_OK);
  }
}

void parseCAN_hdx(void *arg)
{
  // for all haldex frames received
  while (1)
  {
#if detailedDebugStack
    stackHDX = uxTaskGetStackHighWaterMark(NULL);
#endif

    if (twai_receive_v2(twai_bus_1, &rx_message_hdx, portMAX_DELAY) != ESP_OK)
    {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    do
    {
      lastCANHaldexTick = millis();
      ++lpHaldexFrameCount;

      // Analyzer mode: queue for GVRET/SLCAN and forward untouched, skipping control logic.
      if (analyzerMode || analyzerSerial)
      {
        analyzerQueueFrame(rx_message_hdx, 1);
        twai_transmit_v2(twai_bus_0, &rx_message_hdx, (10 / portTICK_PERIOD_MS));
        continue;
      }

      // Different generations have different ranges of 'applied lock', so scale them to suit.
      // Engagement % AND fault/state flags are decoded per-generation below so that the
      // status byte is always sourced from the correct frame/byte for each platform.
      switch (haldexGeneration)
      {
      case 1:
        // Gen1 Haldex (early pre-PQ): engagement on byte 1, raw range ~128..198.
        received_haldex_engagement_raw = rx_message_hdx.data[1];
        received_haldex_engagement = map(received_haldex_engagement_raw, 128, 198, 0, 100);
        received_haldex_state = rx_message_hdx.data[0];
        break;

      case 2:
        // Gen2 Haldex (PQ Allrad_1 / 0x2C0). As per vw_pq.dbc:
        //   SG_ Kupplungssteifigkeit_Hinten__Ist : 32|8@1+ (0.7874,0) [0|100] "%" -> byte 4
        // Byte 1 is centre-clutch torque-rate (Nm/min), unrelated to engagement.
        received_haldex_engagement_raw = rx_message_hdx.data[4];
        received_haldex_engagement = map(received_haldex_engagement_raw, 0, 127, 0, 100);
        received_haldex_state = rx_message_hdx.data[0];
        break;

      case 4:
        // Gen4 Haldex (later PQ) / 0x2C). As per vw_pq.dbc: engagement on byte 1, raw 128..255.
        received_haldex_engagement_raw = rx_message_hdx.data[1];
        received_haldex_engagement = map(received_haldex_engagement_raw, 128, 255, 0, 100);
        received_haldex_state = rx_message_hdx.data[0];
        break;

      case 41:
        // Gen41 (Vauxhall / GM FDCM) - per GMW8762 PPEI:
        //   0x1CC Secondary Axle Status (DLC=3): clutch torque feedback + state
        //   0x1D1 Rear Axle Status (DLC=4): status flags + diagnostic response
        switch (rx_message_hdx.identifier)
        {
        case HALDEX_GEN41_SEC_AXLE_STATUS_ID:
          if (rx_message_hdx.data_length_code >= 3)
          {
            const uint8_t d0 = rx_message_hdx.data[0];
            const uint8_t d1 = rx_message_hdx.data[1];
            const uint8_t d2 = rx_message_hdx.data[2];
            received_sec_axle_status_raw = d0;
            received_sec_axle_active = (d0 & 0x10) != 0;
            received_sec_axle_fdcm_healthy = (d0 & 0x08) != 0;
            received_sec_axle_arc = d0 & 0x03;
            received_sec_axle_torque_nm = d1;
            received_sec_axle_clutch_state = d2;
            // Feed the existing engagement pipe (used by API + broadcast).
            received_haldex_engagement_raw = d1;
            // Captured OEM peak ~150 Nm during active requests; map 0..200 Nm to 0..100%.
            received_haldex_engagement = (d1 >= 200) ? 100 : (uint8_t)((uint16_t)d1 * 100 / 200);
          }
          break;
        case HALDEX_GEN41_REAR_AXLE_STATUS_ID:
          if (rx_message_hdx.data_length_code >= 4)
          {
            received_rear_axle_status_flags = rx_message_hdx.data[0];
            received_rear_axle_metric_a = rx_message_hdx.data[1];
            received_rear_axle_metric_b = rx_message_hdx.data[2];
            // data[3] is the static 0x28 framing byte (verified across all OEM captures).
          }
          break;
        }
        break;

      case 50:
        // Gen5 Haldex (MQB Allrad_03 / 0x118). Per MQB FCAN K-matrix:
        //   SG_ ALR_Ist_Proz : 16|8@1+ (0.4,0) "%" -> byte 2
        //   SG_ ALR_Charisma_FahrPr / ALR_Charisma_Status -> byte 3
        // Only update on the actual engagement frame (the ECU sends 2 different IDs).
        if (rx_message_hdx.identifier == HALDEX_ID_GEN5)
        {
          received_haldex_engagement_raw = rx_message_hdx.data[2];
          received_haldex_engagement = map(received_haldex_engagement_raw, 0, 250, 0, 100);
          received_haldex_state = rx_message_hdx.data[3]; // Charisma byte (mode/status)
        }
        break;

      case 51:
        // Gen5 (0AY) Haldex: PQ Allrad_1 (0x2C0) format.
        // Byte 0 = fault/state flags (confirmed per PQ K-matrix DBC: bit0=Fehler_Allrad_Kupplung,
        //   bit1=Übertemperaturschutz, bit2=Fehlerstatus_Kupplungssteifigkeit, bit3=Kupplung_komplett_offen,
        //   bit4=Notlauf, bit5=Allrad_Warnlampe, bit6=Geschwindigkeitsbegrenzung).
        // Byte 1 = engagement raw (Gen4-empirical; DBC defines byte1 as Kupplungssteifigkeit_Mitte, byte4 as rear).
        if (rx_message_hdx.identifier == HALDEX_ID)
        {
          received_haldex_engagement_raw = rx_message_hdx.data[1];
          received_haldex_engagement = map(received_haldex_engagement_raw, 128, 255, 0, 100);
          // Snap the top of the curve: the pump audibly keeps ramping after the
          // decoded percentage stalls at 99, so treat >=99% as fully engaged.
          if (received_haldex_engagement >= 99)
          {
            received_haldex_engagement = 100;
          }
          received_haldex_state = rx_message_hdx.data[0]; 
        }
        break;

      default:
        break;
      }

      // Decode the state byte. Bit positions per vw_pq.dbc Allrad_1 byte 0 (Gen1/2/4):
      //   bit 0 = Fehler_Allrad_Kupplung           (clutch fault)            -> clutch1Report
      //   bit 1 = Ubertemperaturschutz__Allrad_1_  (over-temp protection)    -> tempProtection
      //   bit 2 = Fehlerstatus_Kupplungssteifigke  (clutch stiffness fault)  -> clutch2Report
      //   bit 3 = Kupplung_komplett_offen          (coupling fully open)     -> couplingOpen
      //   bit 4 = Notlauf                          (limp mode)               -> limpMode
      //   bit 5 = Allrad_Warnlampe                 (4WD warning lamp)        -> awdWarning
      //   bit 6 = Geschwindigkeitsbegrenzung       (speed limit)             -> speedLimit
      // Gen5 byte 3 holds Charisma_FahrPr/Status, not fault flags - skip for Gen5.
      // Gen41 uses dedicated state vars decoded above - skip for Gen41.
      if (haldexGeneration != 50 && haldexGeneration != 41)
      {
        received_report_clutch1 = (received_haldex_state & (1 << 0));
        received_temp_protection = (received_haldex_state & (1 << 1));
        received_report_clutch2 = (received_haldex_state & (1 << 2));
        received_coupling_open = (received_haldex_state & (1 << 3));
        received_limp_mode = (received_haldex_state & (1 << 4));
        received_awd_warning = (received_haldex_state & (1 << 5));
        received_speed_limit = (received_haldex_state & (1 << 6));
        received_long_lock_state = 0; // not present on PQ Allrad_1
      }
      else if (haldexGeneration == 50)
      {
        received_report_clutch1 = false;
        received_report_clutch2 = false;
        received_temp_protection = false;
        received_coupling_open = false;
        received_limp_mode = false;
        received_awd_warning = false;
        received_speed_limit = false;
        // Gen5 (MQB Allrad_03): ALR_Sta_Laengssperre at start bit 12, length 2 little-endian
        // -> byte 1 bits 4..5. 0=open, 1=partial, 2=closed, 3=undefined.
        if (rx_message_hdx.identifier == HALDEX_ID_GEN5)
        {
          received_long_lock_state = (rx_message_hdx.data[1] >> 4) & 0x03;
        }
      }

      // UDS MQB: mirror 0x779 response frames into the UDS task queue when polling is active.
      // Always fall through so the frame is forwarded to Bus 0 (keeps VCDS / passthrough working).
      if (udsMQBEnabled && udsRxQueue != nullptr && rx_message_hdx.identifier == 0x779)
      {
        xQueueSend(udsRxQueue, &rx_message_hdx, 0);
      }

      twai_transmit_v2(twai_bus_0, &rx_message_hdx, (10 / portTICK_PERIOD_MS));
    } while (twai_receive_v2(twai_bus_1, &rx_message_hdx, 0) == ESP_OK);
  }
}