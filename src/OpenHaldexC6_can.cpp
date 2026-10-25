#include <OpenHaldexC6_can.h>
#include <OpenHaldexC6_Calculations.h>
#include <OpenHaldexC6_Analyzer.h>
#include <OpenHaldexC6_UDS.h>
#include <OpenHaldexC6_ESPNow.h> // espNowTapFrame()
#include <OpenHaldexC6_ChassisOther.h> // Insignia / Focus RS / Volvo P3 chassis decode

// Every bridge / diagnostic transmit goes through this wrapper so a failed send
// (TX queue still full after the 10 ms wait, driver stopped / bus-off) is
// counted instead of vanishing. Counted, not logged per-frame: at bus speed a
// wedged transmitter would flood the log. Failures that reach the wire and error
// out already surface via TWAI_ALERT_TX_FAILED in canBusRecovery(); these
// counters catch the call-site failures the driver never sees. The counters are
// bumped from several tasks without locking, so treat them as an
// order-of-magnitude diagnostic rather than an exact tally.
bool canTransmit(twai_handle_t bus, const twai_message_t *msg)
{
  if (twai_transmit_v2(bus, msg, (10 / portTICK_PERIOD_MS)) == ESP_OK)
  {
    return true;
  }
  const bool isBus0 = (bus == twai_bus_0);
  uint32_t drops;
  if (isBus0)
  {
    drops = canTxDropBus0 + 1;
    canTxDropBus0 = drops;
  }
  else
  {
    drops = canTxDropBus1 + 1;
    canTxDropBus1 = drops;
  }
  // Log the first drop and every 256th after that.
  if ((drops & 0xFF) == 1)
  {
    DEBUG("CAN - TX failed on bus %d (ID 0x%03X, %lu drops total)",
          isBus0 ? 0 : 1, (unsigned)msg->identifier, (unsigned long)drops);
  }
  return false;
}

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

    canTransmit(twai_bus_0, &broadcast_frame);

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
  canTransmit(bus, &scratch);
}

// Brake/handbrake bits on the chassis frames we pass to the Haldex in OEM mode.
// With Follow Brake / Follow Handbrake on, the bit sent to the Haldex is set when
// EITHER the car's own bit is set (passed through as it was) OR a switch wired
// to our brake / handbrake input pin is on (V11.00.0 - for cars where a physical
// brake or handbrake switch is fitted, so it reaches the Haldex over CAN too).
// brakeActive / handbrakeActive are the input pins with "Invert ... Input"
// already applied by IO.cpp; the car's CAN bit is never inverted.
// With Follow off the frame is forwarded untouched.
//
// Bit positions (verified against vw_pq.dbc / PQ35_46 K-matrix / vw_mqb.dbc):
//   PQ Gen1/2/4/0AY: Motor_2  (0x288) byte 2 bit 0  - MO2_BLS         (no CRC)
//   PQ Gen1/2/4/0AY: Kombi_1  (0x320) byte 1 bit 1  - KO1_Handbremse  (no CRC)
//   MQB Gen5:        ESP_05   (0x106) byte 3 bit 2  - ESP_Fahrer_bremst (CRC at byte 0 via ID_SEQ_106)
//   MQB Gen5:        Kombi_01 (0x30B) byte 2 bit 7  - KBI_Handbremse   (no CRC)
static void applyBrakeHandbrakeCANOverride(twai_message_t &m)
{
  // What the Haldex should see: the car's bit OR our input pin
  bool brakeOut = false;
  if (brakeFromCAN || brakeActive)
  {
    brakeOut = true;
  }
  bool handbrakeOut = false;
  if (handbrakeFromCAN || handbrakeActive)
  {
    handbrakeOut = true;
  }

  switch (haldexGeneration)
  {
  case 1:  // Gen1 02D (V11.00.0): same Motor_2 / Kombi_1 bits as the PQ cars
  case 2:  // PQ35 (1K0): brake + handbrake both broadcast on chassis CAN
  case 4:  // PQ35/46 (0BR): brake + handbrake both broadcast on chassis CAN
  case 51: // 0AY - a PQ unit, PQ frame set (see Gen5_0AY_frames*)
    // PQ Motor_2 (0x288) - MO2_BLS brake light switch: byte 2 bit 0. No CRC.
    if (followBrake && m.identifier == MOTOR2_ID && m.data_length_code >= 3)
    {
      if (brakeOut)
      {
        m.data[2] = m.data[2] | 0x01; // byte 2 bit 0 = brake pressed
      }
      else
      {
        m.data[2] = m.data[2] & ~0x01; // byte 2 bit 0 = brake released
      }
    }
    // PQ Kombi_1 (0x320) - KO1_Handbremse: byte 1 bit 1 (PQ35/46 K-matrix
    // "Byte 2 / Bit 1", 1 = angezogen / applied). Byte 1 bit 0 is the
    // brake-fluid warning (KO1_Bremsinfo) and is left untouched. No CRC.
    if (followHandbrake && m.identifier == mKombi_1 && m.data_length_code >= 2)
    {
      if (handbrakeOut)
      {
        m.data[1] = m.data[1] | 0x02; // byte 1 bit 1 = handbrake applied
      }
      else
      {
        m.data[1] = m.data[1] & ~0x02; // byte 1 bit 1 = handbrake off
      }
    }
    // (The old Gen51 MQB Kombi_01 0x30B rewrite is gone: the OEM code shows
    // the 0AY has no receive slot for 0x30B - spec_gen5_0ay #12.)
    break;

  case 50:
  case 52: // Gen5 0CQ VAQ - copies 0CQ brake/handbrake overrides
    // MQB ESP_05 (0x106) - ESP_Fahrer_bremst: byte 3 bit 2, CRC at byte 0
    if (followBrake && m.identifier == ESP_05 && m.data_length_code >= 8)
    {
      if (brakeOut)
      {
        m.data[3] = m.data[3] | 0x04; // byte 3 bit 2 = driver braking
      }
      else
      {
        m.data[3] = m.data[3] & ~0x04; // byte 3 bit 2 = not braking
      }
      // Recompute AUTOSAR CRC8 at byte 0 - counter nibble at byte 1 already
      // matches the original sender, so the same ID_SEQ_106 lookup applies.
      m.data[0] = calcChecksum(m.data, ID_SEQ_106);
    }
    // MQB Kombi_01 (0x30B) - KBI_Handbremse: byte 2 bit 7. No CRC byte
    // (byte 0 is fully populated with KBI lamp flags).
    if (followHandbrake && m.identifier == KOMBI_01 && m.data_length_code >= 3)
    {
      if (handbrakeOut)
      {
        m.data[2] = m.data[2] | 0x80; // byte 2 bit 7 = handbrake applied
      }
      else
      {
        m.data[2] = m.data[2] & ~0x80; // byte 2 bit 7 = handbrake off
      }
    }
    break;

  default:
    break;
  }
}

// Which CAN bus speed a Haldex generation needs. Volvo P2 cars before the 2005
// facelift run their fast CAN bus at 250 kbit/s; everything else is 500 kbit/s.
uint16_t canSpeedForGeneration(uint8_t generation)
{
  if (generation == GEN_VOLVO_P2_GEN2)
  {
    return 250;
  }
  return 500;
}

// Called from loop(). The CAN controllers can only change speed by being
// re-installed, and the receive tasks are sitting inside them, so the safe way
// is to save the new generation and restart. Waits 2 s first so the web page
// (or gauge / serial lab) gets its reply before the controller goes away.
void checkCanSpeedChange()
{
  static uint32_t speedMismatchSinceMs = 0; // when the generation first asked for a different speed (0 = no mismatch)

  if (canSpeedForGeneration(haldexGeneration) == canBusSpeedKbps)
  {
    speedMismatchSinceMs = 0; // speeds match, nothing to do
    return;
  }

  if (speedMismatchSinceMs == 0)
  {
    speedMismatchSinceMs = millis(); // start the 2 s wait
    if (speedMismatchSinceMs == 0)
    {
      speedMismatchSinceMs = 1; // millis() was exactly 0 - keep "waiting" distinct from "no mismatch"
    }
    return;
  }

  if (millis() - speedMismatchSinceMs < 2000)
  {
    return; // still waiting
  }

  DEBUG("CAN - generation %d needs %d kbit/s (running %d) - saving and restarting",
        haldexGeneration, canSpeedForGeneration(haldexGeneration), canBusSpeedKbps);
  pref.putUChar("haldexGen", haldexGeneration); // save now: the periodic writeEEP might not have run yet
  delay(100);                                   // let the flash write and the debug line finish
  ESP.restart();
}

void setupCAN()
{
  // Bus speed comes from the saved Haldex generation (readEEP runs before this).
  canBusSpeedKbps = canSpeedForGeneration(haldexGeneration);
  DEBUG("CAN - Bus speed %d kbit/s (generation %d)", canBusSpeedKbps, haldexGeneration);

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(gpio_num_t(CAN0_TX), gpio_num_t(CAN0_RX), TWAI_MODE_NO_ACK); // TWAI_MODE_NORMAL, TWAI_MODE_NO_ACK or TWAI_MODE_LISTEN_ONLY
  twai_timing_config_t t_config_500 = TWAI_TIMING_CONFIG_500KBITS();                                                        // normal CAN speed
  twai_timing_config_t t_config_250 = TWAI_TIMING_CONFIG_250KBITS();                                                        // Volvo P2 Gen2 CAN speed
  twai_timing_config_t t_config = t_config_500;                                                                             // the one actually used
  if (canBusSpeedKbps == 250)
  {
    t_config = t_config_250;
  }
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); // accept all messages

  // g_config.intr_flags = ESP_INTR_FLAG_LOWMED;  //Optional - move canbus irq to free up the default level 1 IRQ it will take up.  Todo
  // Queues come out of heap at 13 B per frame, per bus. Was 1024/2048 (~80 KB for both buses).
  // 512 RX = ~115 ms of a 100%-loaded 500k bus; the parse tasks are the top app priority.
  // Check the debugMemory "CAN" line for missed/overrun counts if this is ever in doubt.
  g_config.tx_queue_len = 256;
  g_config.rx_queue_len = 512;
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

  // Reconfigure alerts to detect frame receive, error states and the full
  // bus-off / recovery lifecycle so canBusRecovery() can drive a clean restart.
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS |
                              TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL |
                              TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_OFF |
                              TWAI_ALERT_BUS_RECOVERED | TWAI_ALERT_RX_FIFO_OVERRUN;
  // Apply to both controllers explicitly (v2 driver -> per-handle alerts).
  bool alertsOk = (twai_reconfigure_alerts_v2(twai_bus_0, alerts_to_enable, NULL) == ESP_OK) &&
                  (twai_reconfigure_alerts_v2(twai_bus_1, alerts_to_enable, NULL) == ESP_OK);
  if (alertsOk)
  {
    DEBUG("Reconfiguration of CAN alerts");
  }
  else
  {
    DEBUG("Failed to reconfigure CAN alerts!");
    return;
  }
}

// ============================================================================
// CAN-bus fault recovery
// ============================================================================
// Polls both TWAI controllers and drives the bus-off recovery so
// the bus comes back cleanly after a fault:
//   RUNNING     -> normal operation, clear the fault flag.
//   BUS_OFF     -> hardware went bus-off (TEC >= 256); kick off recovery.
//   RECOVERING  -> wait for the 128x11-recessive-bit recovery to complete.
//   STOPPED     -> recovery finished (driver stops here); restart the bus.
//
// A STOPPED controller is always restarted. Nothing in the firmware stops a
// controller on purpose (light sleep saves and restores it in the driver), so a
// stopped one is either finishing our own bus-off recovery or has come back
// stopped from a power-down, and in both cases nothing else will restart it.
// `isBusFailure` is owned here: it is set while any bus is unhealthy and
// cleared once both controllers are running again.
// ============================================================================
void canBusRecovery()
{
  static bool recovering[2] = {false, false}; // two TWAI controllers, track which ones we put into recovery
  twai_handle_t buses[2] = {twai_bus_0, twai_bus_1};
  bool anyFault = false;

  for (int i = 0; i < 2; ++i)
  {
    twai_handle_t bus = buses[i];

    // Non-blocking alert read - latch transient error conditions as a fault.
    uint32_t alerts = 0;
    if (twai_read_alerts_v2(bus, &alerts, 0) == ESP_OK)
    {
      if (alerts & (TWAI_ALERT_BUS_ERROR | TWAI_ALERT_ERR_PASS |
                    TWAI_ALERT_TX_FAILED | TWAI_ALERT_RX_QUEUE_FULL |
                    TWAI_ALERT_RX_FIFO_OVERRUN))
      {
        anyFault = true;
      }
    }

    twai_status_info_t status;
    if (twai_get_status_info_v2(bus, &status) != ESP_OK)
      continue;

    switch (status.state)
    {
    case TWAI_STATE_BUS_OFF:
      // Controller is bus-off; begin recovery (moves it to RECOVERING).
      twai_initiate_recovery_v2(bus);
      recovering[i] = true;
      anyFault = true;
      DEBUG("CAN bus %d: bus-off detected - initiating recovery", i);
      break;

    case TWAI_STATE_RECOVERING:
      // Recovery underway; hold the fault flag until it completes.
      recovering[i] = true;
      anyFault = true;
      break;

    case TWAI_STATE_STOPPED:
      // Restart it whether or not we started the recovery: a controller left
      // stopped means no frames, and in non-aggressive low power no frames
      // also means the module never sees the car wake up.
      if (twai_start_v2(bus) == ESP_OK)
      {
        recovering[i] = false;
        DEBUG("CAN bus %d: controller was stopped - restarted", i);
      }
      anyFault = true;
      break;

    case TWAI_STATE_RUNNING:
    default:
      recovering[i] = false; // not recovering, clear the flag
      break;
    }
  }

  isBusFailure = anyFault;
}

// True while an external diagnostic tool was recently seen addressing the Haldex
// on the chassis bus. Used to auto-pause the live-diagnostic so we
// don't collide with a real scanner (VCDS/ODIS). Resets automatically once the
// scanner tool goes quiet for EXTERNAL_DIAG_TIMEOUT_MS.
bool externalDiagActive()
{
  const uint32_t t = externalDiagLastMs;
  return t != 0 && (millis() - t) < EXTERNAL_DIAG_TIMEOUT_MS;
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
      espNowTapFrame(rx_message_chs); // relay to gauges over ESP-NOW (when on) - the frame as the car sent it

      // External diagnostic-tool detection (auto-pause of live polling).
      // A scanner addresses the Haldex from the chassis side; these request IDs
      // never originate from us (our polling transmits on Bus 1), so seeing one on
      // Bus 0 means a real tool is connected. externalDiagActive() then backs our
      // UDS/TP2.0 tasks off until it goes quiet - so liveDiagEnabled restores on disconnection.
      {
        const uint32_t _did = rx_message_chs.identifier;
        if (_did == KWP_TP20_SETUP_TX_ID ||         // 0x200 TP2.0 channel setup
            _did == 0x7DFu ||                       // OBD functional request (scan/clear)
            (_did >= 0x700u && _did <= 0x71Fu))     // VAG UDS/KWP physical tester requests
        {
          externalDiagLastMs = millis();
        }
      }

      // /api/uds/read tap (Bus 0 reads): COPY the response frame into the web
      // helper's queue - the frame continues down the normal gateway path below,
      // so the helper can never starve the bridge of chassis frames.
      if (udsWebRespId != 0 && udsWebBus == 0 && udsWebRxQueue != nullptr &&
          rx_message_chs.identifier == udsWebRespId)
      {
        if (xQueueSend(udsWebRxQueue, &rx_message_chs, 0) != pdTRUE)
        {
          DEBUG("CAN - UDS web tap queue full, dropped frame 0x%03X", (unsigned)rx_message_chs.identifier);
        }
      }

      tx_message_hdx.identifier = rx_message_chs.identifier;

      // Gen41 Haldex-originated Bus0 heartbeats - capture presence & timestamp.
      // Done before any mode so the API can show liveness even in standalone.
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

      // OpenHaldex supplier-specific UDS responders. A tester on the OBD/chassis
      // bus (e.g. Rokketek gauge) polls the Haldex physical address 0x70F and
      // decodes replies on 0x779; the passive 0x6B0 broadcast does not cross the
      // gateway to the OBD port, so we answer supplier DIDs on the same pair and
      // CONSUME the request (continue) so the real Haldex on Bus 1 never sees it.
      if (rx_message_chs.identifier == diagnostics_5_ID &&
          rx_message_chs.data_length_code >= 4 &&
          (rx_message_chs.data[0] & 0xF0) == 0x00 && // ISO-TP single frame
          (rx_message_chs.data[0] & 0x0F) >= 3 &&    // >= 3 payload bytes (SID + DID)
          rx_message_chs.data[1] == 0x22 &&          // ReadDataByIdentifier
          rx_message_chs.data[2] == (uint8_t)(OPENHALDEX_LOCK_DID >> 8) &&
          rx_message_chs.data[3] == (uint8_t)(OPENHALDEX_LOCK_DID & 0xFF))
      {
        const float lt = lock_target;
        const uint8_t cmd = (uint8_t)(lt < 0.0f ? 0.0f : (lt > 100.0f ? 100.0f : lt + 0.5f));
        twai_message_t resp{};
        resp.identifier = OPENHALDEX_UDS_RESPONSE_ID;
        resp.extd = 0;
        resp.rtr = 0;
        resp.data_length_code = 8;
        resp.data[0] = 0x06; // ISO-TP single frame, 6 payload bytes
        resp.data[1] = 0x62; // positive RDBI response
        resp.data[2] = (uint8_t)(OPENHALDEX_LOCK_DID >> 8);
        resp.data[3] = (uint8_t)(OPENHALDEX_LOCK_DID & 0xFF);
        resp.data[4] = cmd;                                        // commanded lock %, 0-100
        resp.data[5] = (uint8_t)received_haldex_engagement_raw;    // applied engagement, raw
        resp.data[6] = (uint8_t)state.mode;                        // live mode, so a write reads back
        resp.data[7] = 0xAA;                                       // ISO-TP padding
        canTransmit(twai_bus_0, &resp);
        continue; // consume - do NOT forward to Bus 1
      }

      // Per-corner slip DID: 4 signed slip bytes [FL, FR, RL, RR]; -128 = no data
      // (too slow, or stale wheel/steering signal) so the gauge can blank rather
      // than draw a false zero.
      if (rx_message_chs.identifier == diagnostics_5_ID &&
          rx_message_chs.data_length_code >= 4 &&
          (rx_message_chs.data[0] & 0xF0) == 0x00 &&
          (rx_message_chs.data[0] & 0x0F) >= 3 &&
          rx_message_chs.data[1] == 0x22 &&
          rx_message_chs.data[2] == (uint8_t)(OPENHALDEX_SLIP_DID >> 8) &&
          rx_message_chs.data[3] == (uint8_t)(OPENHALDEX_SLIP_DID & 0xFF))
      {
        int8_t slip[4] = {-128, -128, -128, -128};
        if (lastCornerSlipMs != 0 && (millis() - lastCornerSlipMs) < 500)
        {
          slip[0] = cornerSlip[0];
          slip[1] = cornerSlip[1];
          slip[2] = cornerSlip[2];
          slip[3] = cornerSlip[3];
        }
        twai_message_t resp{};
        resp.identifier = OPENHALDEX_UDS_RESPONSE_ID;
        resp.extd = 0;
        resp.rtr = 0;
        resp.data_length_code = 8;
        resp.data[0] = 0x07; // ISO-TP single frame, 7 payload bytes (0x62 + DID + 4)
        resp.data[1] = 0x62;
        resp.data[2] = (uint8_t)(OPENHALDEX_SLIP_DID >> 8);
        resp.data[3] = (uint8_t)(OPENHALDEX_SLIP_DID & 0xFF);
        resp.data[4] = (uint8_t)slip[0];
        resp.data[5] = (uint8_t)slip[1];
        resp.data[6] = (uint8_t)slip[2];
        resp.data[7] = (uint8_t)slip[3];
        canTransmit(twai_bus_0, &resp);
        continue;
      }

      // Drive-mode WRITE (WriteDataByIdentifier 0x2E): [len][2E][DID_hi][DID_lo][mode].
      // Sets the mode as a head unit would; replies 0x6E on success or 0x7F/0x31
      // (requestOutOfRange) for a bad value. Request is consumed.
      if (rx_message_chs.identifier == diagnostics_5_ID &&
          rx_message_chs.data_length_code >= 5 &&
          (rx_message_chs.data[0] & 0xF0) == 0x00 &&
          (rx_message_chs.data[0] & 0x0F) >= 4 &&
          rx_message_chs.data[1] == 0x2E &&
          rx_message_chs.data[2] == (uint8_t)(OPENHALDEX_MODE_DID >> 8) &&
          rx_message_chs.data[3] == (uint8_t)(OPENHALDEX_MODE_DID & 0xFF))
      {
        const uint8_t requested_mode = rx_message_chs.data[4];
        twai_message_t resp{};
        resp.identifier = OPENHALDEX_UDS_RESPONSE_ID;
        resp.extd = 0;
        resp.rtr = 0;
        resp.data_length_code = 8;
        if (requested_mode < (uint8_t)openhaldex_mode_t_MAX)
        {
          state.mode = (openhaldex_mode_t)requested_mode;
          resp.data[0] = 0x03; // ISO-TP single frame, 3 payload bytes
          resp.data[1] = 0x6E; // positive WDBI response
          resp.data[2] = (uint8_t)(OPENHALDEX_MODE_DID >> 8);
          resp.data[3] = (uint8_t)(OPENHALDEX_MODE_DID & 0xFF);
          resp.data[4] = 0xAA;
          resp.data[5] = 0xAA;
          resp.data[6] = 0xAA;
          resp.data[7] = 0xAA;
        }
        else
        {
          resp.data[0] = 0x03;
          resp.data[1] = 0x7F; // negative response
          resp.data[2] = 0x2E; // ...to WriteDataByIdentifier
          resp.data[3] = 0x31; // requestOutOfRange
          resp.data[4] = 0xAA;
          resp.data[5] = 0xAA;
          resp.data[6] = 0xAA;
          resp.data[7] = 0xAA;
        }
        canTransmit(twai_bus_0, &resp);
        continue;
      }

      // Insignia (41), Focus RS (42) and Volvo P3 (43) have their own chassis
      // decoder: speed, pedal, RPM, brake and handbrake (OpenHaldexC6_ChassisOther.cpp).
      if ((!isStandalone || useCANifAvailable) && hasOtherCarChassisDecode(haldexGeneration))
      {
        decodeOtherCarChassis(rx_message_chs);
      }

      // The decoder below reads VW chassis IDs. Other makes reuse some of the same
      // numbers (SAAB 0x280 / 0x288 / 0x320) for other things, so skip it for
      // every non-VW generation.
      if ((!isStandalone || useCANifAvailable) && !isVolvoOrSaabGeneration(haldexGeneration) &&
          !hasOtherCarChassisDecode(haldexGeneration))
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
          // MQB-only ID; PQ platforms carry the handbrake on Kombi_1 (0x320) below.
          if (rx_message_chs.data_length_code >= 3)
          {
            handbrakeFromCAN = bitRead(rx_message_chs.data[2], 7);
          }
          break;

        case CHARISMA_07:
          // MQB Charisma_07 (0x3E8) - drive mode select, sent by the gateway once a second.
          // MQB FCAN K-matrix:
          //   SG_ CHA_Current_Mode : 8|4@1+  -> byte 1 bits 0-3, the drive profile the driver picked
          // Profile numbers (jilleb/MQB-FPA): 1 Comfort, 2 Normal, 3 Sport, 4 Offroad, 5 Eco,
          // 6 Race, 7 Individual, 8 Range/PHEV 1, 9 Lift/PHEV 2, 10 Snow/PHEV 3,
          // 11 Offroad Individual/PHEV 4. 0 = not filled in by this car.
          // Our own MQB logs read 00 B7 00 .. = 7 (Individual) while Charisma_01 asked
          // for a mix of Comfort / Normal / Sport across the systems, which is what an
          // Individual profile looks like.
          if ((haldexGeneration == 50 || haldexGeneration == 51 || haldexGeneration == 52) &&
              rx_message_chs.data_length_code >= 2)
          {
            uint8_t currentMode = rx_message_chs.data[1] & 0x0F; // byte 1 bits 0-3: drive profile
            if (currentMode != 0)                                // 0 = not used on this car, so fall back to Charisma_01
            {
              receivedDriveProfile = currentMode;
              driveProfileSource = DRIVE_SOURCE_CHARISMA_07;
              lastDriveProfileMs = millis();
              lastCharisma07Ms = millis();
            }
          }
          break;

        case CHARISMA_01:
          // MQB Charisma_01 (0x385) - drive mode select, sent by the gateway once a second.
          // MQB FCAN K-matrix: a 4-bit "requested mode" for each system the gateway controls:
          //   SG_ CHA_Ziel_FahrPr_MO  : 16|4@1+ -> byte 2 bits 0-3, engine
          //   SG_ CHA_Ziel_FahrPr_ALR : 24|4@1+ -> byte 3 bits 0-3, Haldex (all-wheel drive)
          //   SG_ CHA_Ziel_FahrPr_VAQ : 36|4@1+ -> byte 4 bits 4-7, front diff lock (VAQ)
          // Mode numbers (jilleb/MQB-FPA): 1 Comfort, 2 Normal, 3 Sport, 4 Offroad, 5 Eco,
          // 6 Race, 7 Individual/Eco+, 8 On, 9 Off, 10 Snow. 0 = no function.
          if ((haldexGeneration == 50 || haldexGeneration == 51 || haldexGeneration == 52) &&
              rx_message_chs.data_length_code >= 5)
          {
            receivedHaldexModeRequest = rx_message_chs.data[3] & 0x0F;     // byte 3 bits 0-3: mode asked of the Haldex
            receivedVaqModeRequest = (rx_message_chs.data[4] >> 4) & 0x0F; // byte 4 bits 4-7: mode asked of the VAQ
            hasHaldexModeRequest = true;

            // The engine's requested mode follows the drive profile on most cars, so use it
            // when Charisma_07 (the profile itself) isn't on this bus. It can't tell Individual
            // apart (it shows the engine setting chosen inside Individual). 8 (On) and 9 (Off)
            // aren't profiles, so they're skipped.
            uint8_t engineModeRequest = rx_message_chs.data[2] & 0x0F; // byte 2 bits 0-3: mode asked of the engine
            bool hasRecentCharisma07 = false;
            if (lastCharisma07Ms != 0 && (millis() - lastCharisma07Ms) < DRIVE_MODE_TIMEOUT_MS)
            {
              hasRecentCharisma07 = true;
            }
            if (!hasRecentCharisma07 && engineModeRequest != 8 && engineModeRequest != 9)
            {
              receivedDriveProfile = engineModeRequest;
              driveProfileSource = DRIVE_SOURCE_CHARISMA_01;
              lastDriveProfileMs = millis();
            }
          }
          break;

        case mKombi_1:
          // PQ Kombi_1 (0x320) - instrument cluster broadcast.
          // PQ35/46 K-matrix (V5.20.6F), message mKombi_1 0x320:
          //   KO1_Bremsinfo  : Byte 2 / Bit 0 (1-based) -> data[1] bit 0, brake-fluid warning
          //   KO1_Handbremse : Byte 2 / Bit 1 (1-based) -> data[1] bit 1, 1 = angezogen (applied)
          // (vw_pq.dbc lumps both as "Bremsinfo : 8|2".) Gen2 (1K0) and Gen4 (0AY)
          // read the handbrake from this frame, not from a physical wire. Gated to
          // the PQ-frame generations so a foreign 0x320 on another platform can't
          // spoof it. Gen1 (02D) added in V11: its OEM code reads this same bit
          // (handbrake pulled at 5-100 km/h -> clutch opens, spec_gen1_02d §2.2).
          if ((haldexGeneration == 1 || haldexGeneration == 2 || haldexGeneration == 4 || haldexGeneration == 51) &&
              rx_message_chs.data_length_code >= 2)
          {
            handbrakeFromCAN = bitRead(rx_message_chs.data[1], 1);
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

        case KLEMMEN_STATUS_01:
          // MQB Klemmen_Status_01 (0x3C0), MQB_ACAN_KMatrix:
          //   SG_ ZAS_Kl_15 : 17|1@1+ -> byte 2 bit 1, ignition (terminal 15) on
          received_kl15 = (rx_message_chs.data[2] >> 1) & 0x01;
          lastKl15Ms = millis();
          break;

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
          // SG_ GK1_Warnblk_Status : 55|1@1+ -> byte 6 bit 7, hazard warning lights active
          // Used to drive hazard-light force-mode when enabled.
          // MK4 Golf doesn't actually have this on CAN despite DBC saying it does(!)
          hazardForceModeFlag = bitRead(rx_message_chs.data[6], 7);
          break;
        }

        case GATEWAY_72:
        {
          // MQB Gateway_72 (0x3DB) - gateway body lighting state.
          // Note: on at least some MQB clusters the gateway frame does NOT
          // reflect hazard state when active, so decode hazard from
          // Blinkmodi_02 (0x366) below instead. This case is kept
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

          // BR1_ABS_Brems (vw_pq.dbc 2|1) - byte 0 bit 2, the ABS module is braking.
          // The Haldex reads this bit itself and opens the clutch (OEM code: Gen4
          // 0BR "state 4"). Our lock request outranks that, so lock_gate_blocking()
          // releases lock while it is set (V11.00.0).
          absBrakingFromCAN = bitRead(rx_message_chs.data[0], 2);

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

          // Promote all four corners for the per-corner slip DID and dashboard.
          // Order [FL, FR, RL, RR] = [VL, VR, HL, HR].
          wheelSpeedRaw[0] = wheel_speed_vl_raw;
          wheelSpeedRaw[1] = wheel_speed_vr_raw;
          wheelSpeedRaw[2] = wheel_speed_hl_raw;
          wheelSpeedRaw[3] = wheel_speed_hr_raw;
          lastWheelSpeedResponse = millis();

          // Geometry-compensated slip: feed steering only while fresh, else 0
          // (straight) so it falls back to plain relative slip rather than
          // mis-compensating on a stale angle.
          int16_t steerTenths = 0;
          if (received_steering_ms != 0 && (millis() - received_steering_ms) < 500)
          {
            int16_t mag = (int16_t)(received_steering_angle * 10.0f + 0.5f);
            steerTenths = received_steering_negative ? (int16_t)-mag : mag;
          }
          if (!compute_corner_slip(wheelSpeedRaw, steerTenths, slipSteeringRatio,
                                   slipWheelbaseMm, slipTrackFrontMm, slipTrackRearMm,
                                   slipMinSpeedRaw, cornerSlip))
          {
            cornerSlip[0] = cornerSlip[1] = cornerSlip[2] = cornerSlip[3] = -128;
          }
          lastCornerSlipMs = millis();
          break;
        }

        case mLW_1:
        {
          // PQ LW_1 (0x0C2) - steering-angle broadcast (vw_pq.dbc):
          //   SG_ LW1_LRW : 0|15@1+ (0.04375,0) "deg" -> data[0] + data[1] bits 0..6
          //   SG_ LW1_LRW_Sign : 15|1 -> data[1] bit 7 (ignored; we track magnitude only)
          const uint16_t lrw_raw = (((uint16_t)(rx_message_chs.data[1] & 0x7F) << 8) | rx_message_chs.data[0]);
          received_steering_angle = lrw_raw * 0.04375f;
          received_steering_negative = bitRead(rx_message_chs.data[1], 7); // LW1_LRW_Sign
          received_steering_ms = millis();
          break;
        }

        case LWI_01:
        {
          // MQB LWI_01 (0x086) - steering-angle sensor (vw_mqb.dbc):
          //   SG_ LWI_Lenkradwinkel : 16|13@1+ (0.1,0) "deg" -> data[2] + data[3] bits 0..4
          //   SG_ LWI_VZ_Lenkradwinkel : 29|1 -> data[3] bit 5 (ignored; magnitude only)
          const uint16_t lwi_raw = (((uint16_t)(rx_message_chs.data[3] & 0x1F) << 8) | rx_message_chs.data[2]);
          received_steering_angle = lwi_raw * 0.1f;
          received_steering_negative = bitRead(rx_message_chs.data[3], 5); // LWI_VZ_Lenkradwinkel
          received_steering_ms = millis();
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

          // ABS_Bremsung (MQB FCAN K-matrix 56|1) - byte 7 bit 0, the ABS module is
          // braking. The K-matrix lists Allrad_Haldex as a receiver of this signal.
          // Released by lock_gate_blocking() while set (V11.00.0).
          if (rx_message_chs.data_length_code >= 8)
          {
            absBrakingFromCAN = bitRead(rx_message_chs.data[7], 0);
          }

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
          canTransmit(twai_bus_1, &tx_message_hdx);
          break;
        }
        continue;
      }

      // check to see if we're NOT in standalone - and look to edit the frames if necessary
      if (!isStandalone)
      {
        if (1) // find out what this does(!) affects MQB: rx_message_chs.identifier != diagnostics_5_ID
        {
          // Controller disabled = HARD passthrough. When disableController is set the
          // unit acts as a transparent bridge: it READS the chassis frame and forwards
          // it to the Haldex completely untouched - no getLockData(), no brake/handbrake
          // override, no force-flag handling, regardless of mode, learn state, or any
          // enabled/asserted flag. Global kill-switch for all frame work!
          if (!disableController)
          {
          // Effective mode: an active force trigger's value overrides the base
          // mode (get_forced_mode_value pairs each feature with its own flag, so
          // a stray tcForceModeFlag with the TC feature disabled cannot trigger).
          // When the EFFECTIVE mode is Stock - base mode Stock with no trigger,
          // or a trigger whose configured value is Stock - the only correct
          // behaviour is untouched passthrough. Editing frames in Stock used to
          // mirror received_haldex_engagement back as the command, closing a
          // feedback loop that latched the clutch at 100% until FWD/power cycle.
          const int forcedMode = get_forced_mode_value();
          const bool stockEffective = (forcedMode >= 0) ? (forcedMode == 0)
                                                        : (state.mode == MODE_STOCK);

          // Edit the CAN frame unless effective-Stock (learn must run regardless of mode)
          if (!stockEffective || haldexLearnActive)
          {
            if (haldexGeneration == 1 || haldexGeneration == 2 || haldexGeneration == 4 || haldexGeneration == 50 || haldexGeneration == 51 || haldexGeneration == 52 || haldexGeneration == 41)
            {
              // this is the main function that edits the CAN frame to control the Haldex.
              getLockData(rx_message_chs);
            }
          }
          else
          {
            // Stock passthrough: mirror the Haldex's reported engagement for
            // telemetry only; the frame itself is forwarded untouched below.
            lock_target = received_haldex_engagement;
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
            // and forwarded below - purely so that 'Requested' doesn't show 0 (looks unclean IMO)
            lock_target = received_haldex_engagement;
          }

          tx_message_hdx = rx_message_chs;
          tx_message_hdx.extd = rx_message_chs.extd;
          tx_message_hdx.rtr = rx_message_chs.rtr;
          tx_message_hdx.data_length_code = rx_message_chs.data_length_code;
          canTransmit(twai_bus_1, &tx_message_hdx);
        }
      }
    } while (twai_receive_v2(twai_bus_0, &rx_message_chs, 0) == ESP_OK);
  }
}

// Tally every Bus 1 frame per ID (serial lab RXIDS). Linear scan of a 16-slot
// table: Bus 1 carries only what the module sends (our own TX is not echoed),
// so a handful of IDs is the norm. Called from the parse task only.
static void hdxRxCensus(const twai_message_t &m)
{
  const uint32_t now = millis();
  int slot = -1;
  for (int i = 0; i < HDX_RX_STATS_MAX; i++)
  {
    if (hdxRxStats[i].id == m.identifier && hdxRxStats[i].extd == (m.extd ? 1 : 0))
    {
      slot = i;
      break;
    }
    if (slot < 0 && hdxRxStats[i].id == 0)
      slot = i; // first free slot, used only if no match follows
  }
  if (slot < 0)
  {
    hdxRxDroppedIds++;
    return;
  }
  HdxRxStat &s = hdxRxStats[slot];
  if (s.id != m.identifier || s.extd != (m.extd ? 1 : 0))
  {
    s.id = m.identifier;
    s.extd = m.extd ? 1 : 0;
    s.count = 0;
    s.lastMs = now;
    s.periodMs = 0;
  }
  else
  {
    const uint32_t dt = now - s.lastMs;
    s.periodMs = (uint16_t)(dt > 0xFFFF ? 0xFFFF : dt);
    s.lastMs = now;
  }
  s.count++;
  s.dlc = m.data_length_code;
  for (uint8_t i = 0; i < 8; i++)
    s.data[i] = (i < m.data_length_code) ? m.data[i] : 0;
}

// Remember the raw frame the engagement decoder just accepted (serial lab FB line).
static inline void hdxFbCapture(const twai_message_t &m)
{
  hdxFbId = m.identifier;
  hdxFbDlc = m.data_length_code;
  for (uint8_t i = 0; i < 8; i++)
    hdxFbData[i] = (i < m.data_length_code) ? m.data[i] : 0;
  hdxFbMs = millis();
}

// Clear every Haldex health flag (V11.00.0). Each generation's feedback decode
// in parseCAN_hdx() calls this first, then sets only the bits its own frame
// really carries - so a flag can never be left over from another generation.
static void clearHaldexHealth()
{
  received_report_clutch1 = false;    // fault present
  received_report_clutch2 = false;    // plausibility fault
  received_temp_protection = false;   // over-temperature protection
  received_coupling_open = false;     // clutch commanded fully open (status)
  received_limp_mode = false;         // limp / fault reaction
  received_awd_warning = false;       // AWD warning lamp
  received_speed_limit = false;       // speed limit request
  received_temp_sensor_fault = false; // temperature sensor fault
  received_fault_stored = false;      // fault stored in the unit's memory
}

// PQ Allrad_1 (0x2C0) byte 0 with the vw_pq.dbc names was decoded in one shared
// block for Gen1/2/4/0AY before V11 (bit 0 clutch fault, 1 over-temp, 2 stiffness
// fault, 3 coupling open, 4 limp, 5 lamp, 6 speed limit). The OEM code shows each
// unit uses these bits differently, so each generation's case decodes its own.

// Gen5 0CQ (V11.00.0): the "fault reaction active" bit from the unit's own status
// frame 0x17F0000F (byte 0 bit 0, OEM code: spec_gen5_0cq §6). That frame comes
// far less often than Allrad_03, so the last value is kept here and merged into
// the health flags on every Allrad_03. 'static' keeps these private to this file.
static bool gen5StatusFaultReaction = false; // 0x17F0000F byte 0 bit 0 as last received
static uint32_t gen5StatusFrameMs = 0;       // millis() of the last 0x17F0000F (0 = never)

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
      hdxRxCensus(rx_message_hdx);

      // Analyzer mode: queue for GVRET/SLCAN and forward untouched, skipping control logic.
      if (analyzerMode || analyzerSerial)
      {
        analyzerQueueFrame(rx_message_hdx, 1);
        canTransmit(twai_bus_0, &rx_message_hdx);
        continue;
      }

      // Different generations have different ranges of 'applied lock', so scale them to suit.
      // Engagement % AND fault/state flags are decoded per-generation below so that the
      // status byte is always sourced from the correct frame/byte for each platform.
      switch (haldexGeneration)
      {
      case 1:
        // Gen1 Haldex (02D900554B) Allrad_1 / 0x2C0, DLC 2 (OEM code: spec_gen1_02d §6).
        // Only decode 0x2C0 - before V11 every frame on the Haldex bus was decoded (D2).
        if (rx_message_hdx.identifier == HALDEX_ID)
        {
          // Byte 1 = 0x80 + measured clutch torque. 0x7F = released (very common - it
          // used to wrap round to 255 %, D1), 0x80 = no pressure, OEM top 0xDF.
          // 198 (0xC6) is kept as "100 %": the most our bench unit ever reported.
          received_haldex_engagement_raw = rx_message_hdx.data[1];
          int32_t engagementPercent = 0; // worked out in a signed number so it can't wrap
          if (received_haldex_engagement_raw >= 0x80)
          {
            engagementPercent = map(received_haldex_engagement_raw, 128, 198, 0, 100);
          }
          if (engagementPercent < 0)
          {
            engagementPercent = 0;
          }
          if (engagementPercent > 100)
          {
            engagementPercent = 100; // 199..223 is real extra clamping force - still shown as 100 %
          }
          received_haldex_engagement = (uint8_t)engagementPercent;
          received_haldex_state = rx_message_hdx.data[0];

          // Byte 0 status bits as the 02D's OEM code really sets them (spec §6).
          // Not the vw_pq.dbc names: bit 2 and bit 4 are NOT faults on this unit.
          clearHaldexHealth();
          if (received_haldex_state & (1 << 0))
          {
            received_report_clutch1 = true; // bit 0: unit is in its fault-reaction (suspended) mode...
            received_limp_mode = true;      // ...with the clutch held open = limp
          }
          if (received_haldex_state & (1 << 1))
          {
            received_temp_protection = true; // bit 1: oil over-temperature (or oil temperature sensor problem)
          }
          // bit 2: our Bremse_4 ceiling is active and the clutch is still ramping down to it - status, not a fault
          if (received_haldex_state & (1 << 3))
          {
            received_coupling_open = true; // bit 3: coupling fully open (no clutch pressure)
          }
          // bit 4: "input signals incomplete" (a monitored frame missing / invalid) - status only,
          //        the 02D takes no action on it, so it is NOT limp mode (D3, D4)
          if (received_haldex_state & (1 << 5))
          {
            received_awd_warning = true; // bit 5: AWD warning lamp (inputs implausible for a long time, or fault mode)
          }
          // bits 6-7: never set by the 02D (no speed-limit bit on Gen1)
          received_haldex_temp_c = -128; // Gen1 does not report its temperature on CAN
          haldexHealthSupported = true;
          received_long_lock_state = 0; // not present on PQ Allrad_1
        }
        break;

      case 2:
        // Gen2 Haldex 7H0907554 / 1K0907554B (Allrad_1 / 0x2C0), V11.00.0 from the
        // OEM code (documents/haldex-binaries/leaf_trace/spec_gen2_7h0.md §6):
        //   byte 1 = engagement: min(measured clutch value / 28 + 0x80, 0xFE).
        //            0x80 = open, about 0xFD = full - the same scale as the
        //            Bremse_4 lever we send. (1K0 identical.)
        //   byte 4 / byte 6 are NEVER written by the unit (always 0) - before V11
        //            we read byte 4 (vw_pq.dbc name), so the UI always showed 0 %.
        //   byte 0 = status bits, see below. byte 7 bits 4-7 = message counter.
        // Only decode the Allrad_1 frame; the module also sends 0x081 and diagnostic IDs.
        if (rx_message_hdx.identifier == HALDEX_ID && rx_message_hdx.data_length_code >= 2)
        {
          received_haldex_engagement_raw = rx_message_hdx.data[1];
          int gen2Engagement = 0;                                 // worked out in int so it can't wrap below 0
          if (received_haldex_engagement_raw > 0x80)
          {
            gen2Engagement = ((int)received_haldex_engagement_raw - 0x80) * 100 / 0x7D; // 0x81..0xFD -> 0..100 %
          }
          if (gen2Engagement > 100)
          {
            gen2Engagement = 100; // 0xFE (the unit's own top value) reads 100 %
          }
          received_haldex_state = rx_message_hdx.data[0];

          // Byte 0 status bits, as the OEM code sets them (spec_gen2_7h0 §6). The vw_pq.dbc
          // names (clutch fault, limp, AWD lamp, speed limit ...) do NOT match this unit:
          //   bit 0 = controller active / engaged (status, not a fault)
          //   bit 1 = over-temperature derate - the unit forces its request to 0 (genuine protection)
          //   bit 2 = fault / warning-lamp composite (genuine fault)
          //   bit 3 = coupling fully open (no clutch pressure)
          //   bit 4, bit 5 = status, meaning not decoded yet (kept raw in received_haldex_state)
          //   bit 6, bit 7 = never written on Gen2 (no speed-limit bit)
          clearHaldexHealth();
          received_temp_protection = (received_haldex_state & (1 << 1)); // bit 1: over-temperature derate
          received_report_clutch1 = (received_haldex_state & (1 << 2));  // bit 2: fault present ...
          received_awd_warning = (received_haldex_state & (1 << 2));     // ... and it is what lights the AWD lamp
          received_coupling_open = (received_haldex_state & (1 << 3));   // bit 3: coupling fully open
          received_haldex_temp_c = -128;                                 // Gen2 does not send a temperature
          haldexHealthSupported = true;

          // At rest a real 1K0 sends byte 1 = 0x88 with bit 3 (coupling open) set (from log
          // "1K0 554A - clear codes, brake on,off ..."): the measured value has a small floor.
          // When the unit itself says the coupling is open, show 0 %.
          if (received_coupling_open)
          {
            gen2Engagement = 0;
          }
          received_haldex_engagement = (uint8_t)gen2Engagement;
          received_long_lock_state = 0; // not present on PQ Allrad_1
        }
        break;

      case 4:
        // Gen4 Haldex (later PQ) Allrad_1 / 0x2C0: engagement on byte 1, raw 128..255.
        // Like Gen5, the module broadcasts more than one CAN ID; only decode the
        // engagement frame. Without this guard a stray frame's byte 1 (< 128) makes
        // map() go negative and wrap in the uint8_t result -> the learn procedure
        // saw engagement jumping between 0 and >150%.
        // V11.00.0: decode per the 0BR's own code (spec_gen4_0br §6).
        if (rx_message_hdx.identifier == HALDEX_ID)
        {
          // Byte 1 = the clutch torque the unit is applying, on the SAME scale
          // as our Bremse_4 lever: 0x7F / 0x80 = none, 0x80 + n = n x 112 units,
          // 0xFD = top of the linear range, 0xFE = at full capacity. The unit
          // never sends 0xFF. V11: was mapped 0x80..0xFF (§7 #14).
          received_haldex_engagement_raw = rx_message_hdx.data[1];
          int engagementPercent = 0; // worked out in int so it can't wrap round
          if (received_haldex_engagement_raw >= 0xFE)
          {
            engagementPercent = 100; // at the unit's present maximum capacity
          }
          else if (received_haldex_engagement_raw > 0x80)
          {
            engagementPercent = ((int)received_haldex_engagement_raw - 0x80) * 100 / 125; // 0x81..0xFD -> 0..100 %
          }
          if (engagementPercent > 100)
          {
            engagementPercent = 100;
          }
          received_haldex_engagement = (uint8_t)engagementPercent;

          received_haldex_state = rx_message_hdx.data[0];
          clearHaldexHealth();
          // Byte 0 flags the 0BR really writes (b0.6 speed limit is never set
          // by Gen4, so it stays false - §7 #16; b0.7 is always 0):
          received_report_clutch1 = (received_haldex_state & (1 << 0));  // b0.0 fault present (fault memory active)
          received_temp_protection = (received_haldex_state & (1 << 1)); // b0.1 over-temperature protection active
          received_report_clutch2 = (received_haldex_state & (1 << 2));  // b0.2 clutch / sensor plausibility fault
          received_coupling_open = (received_haldex_state & (1 << 3));   // b0.3 clutch commanded fully open (status)
          received_limp_mode = (received_haldex_state & (1 << 4));       // b0.4 limp / fault reaction (also set if Motor_5 is missing)
          received_awd_warning = (received_haldex_state & (1 << 5));     // b0.5 AWD warning lamp

          if (rx_message_hdx.data_length_code >= 5)
          {
            received_temp_sensor_fault = (rx_message_hdx.data[3] & (1 << 3)); // b3.3 the unit's temperature sensor has failed
            received_fault_stored = (rx_message_hdx.data[3] & (1 << 7));      // b3.7 fault-memory entry stored
            // b4 = Haldex temperature, degC = b4 - 60; 0xFF = sensor invalid
            if (rx_message_hdx.data[4] == 0xFF)
            {
              received_haldex_temp_c = -128; // no valid temperature
            }
            else
            {
              received_haldex_temp_c = (int16_t)rx_message_hdx.data[4] - 60;
            }
          }
          // b7 high nibble is an alive counter (0-15); not decoded - a silent
          // unit already shows as "no Haldex" through hasCANHaldex.
          haldexHealthSupported = true;
          received_long_lock_state = 0; // not present on PQ Allrad_1
        }
        break;

      case 41:
        // Gen41 (Vauxhall / GM FDCM) - per GMW8762 PPEI:
        //   0x1CC Secondary Axle Status (DLC=3): clutch torque feedback + state
        //   0x1D1 Rear Axle Status (DLC=4): status flags + diagnostic response
        // No OEM code held for the GM unit, so no health bits are decoded.
        clearHaldexHealth();
        haldexHealthSupported = false;
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
      case 52:
        // Gen5 Haldex 0CQ (MQB Allrad_03 / 0x118, DLC 4). V11.00.0: decoded per the
        // 0CQ907554D's own code (spec_gen5_0cq §6):
        //   b0       always 0x00 (never written)
        //   b1[0..3] rolling counter 0-15
        //   b1[4..5] status: 0 = normal, 2 = fault reaction - clutch released and
        //            b2 forced to 0. 1 and 3 are never sent. (Was read as
        //            "0 open / 1 partial / 2 closed" - wrong, spec D11.)
        //   b1[6]    1 = the unit accepts our ESP_14 request (0 = it sees no valid
        //            request and runs its own strategy)
        //   b2       actual clutch torque, 0-250 = 0-100 % (0.4 %/step, the same
        //            scale as our ESP_14 lever); 0xFE until the first value after
        //            power-up (was shown as 101 %, spec D13)
        //   b3       never written by the 0CQ907554D (always 0x00). Other 0CQ units
        //            DO send their drive-mode reply here - our 554C bench log and the
        //            car logs show 0x12 - so it is still kept raw in
        //            received_haldex_state for the Drive Mode display, but nothing
        //            is decided from it (spec D12).
        // Only Gen 50: the VAQ (52) sends Quersperre_03 0x137 instead, never 0x118
        // (OEM code: its TX table, spec §8.8).
        if (haldexGeneration == 50 && rx_message_hdx.identifier == HALDEX_ID_GEN5)
        {
          received_haldex_engagement_raw = rx_message_hdx.data[2];
          int engagementPercent = 0; // worked out in int so it can't wrap round
          if (received_haldex_engagement_raw <= 250)
          {
            engagementPercent = (int)received_haldex_engagement_raw * 100 / 250; // 0..250 -> 0..100 %
          }
          // 251 and up (0xFE = no value yet after power-up) = not available -> 0 %
          if (engagementPercent > 100)
          {
            engagementPercent = 100;
          }
          received_haldex_engagement = (uint8_t)engagementPercent;
          received_haldex_state = rx_message_hdx.data[3]; // drive-mode reply on units that send it (0 on 554D), display only
          hdxFbCapture(rx_message_hdx);

          // Health (spec §6). Status 2 = the unit is in a fault-reaction state and has
          // released the clutch. 0x17F0000F byte 0 bit 0 says the same thing (kept
          // from the last status frame, forgotten if that frame stops).
          clearHaldexHealth();
          uint8_t allradStatus = (rx_message_hdx.data[1] >> 4) & 0x03; // b1 bits 4-5
          bool isFaultReaction = false;
          if (allradStatus == 2)
          {
            isFaultReaction = true; // ALR_Sta = 2: fault reaction, clutch released
          }
          if (gen5StatusFaultReaction && gen5StatusFrameMs != 0 &&
              (millis() - gen5StatusFrameMs) < HALDEX_GEN5_STATUS_TIMEOUT_MS)
          {
            isFaultReaction = true; // 0x17F0000F byte 0 bit 0
          }
          if (isFaultReaction)
          {
            received_report_clutch1 = true; // a genuine fault is active - read the codes over UDS
            received_limp_mode = true;      // fault reaction
            received_coupling_open = true;  // the unit has released the clutch
          }
          received_lever_request_valid = (rx_message_hdx.data[1] & (1 << 6)); // b1 bit 6: our ESP_14 request accepted
          received_haldex_temp_c = -128;                                      // the 0CQ does not send a temperature
          haldexHealthSupported = true;

          // Same 0..3 slot the UI / serial lab already read: 0 = normal, 2 = fault -
          // clutch released (1 and 3 are never sent by the 0CQ).
          received_long_lock_state = allradStatus;
        }
        else if (haldexGeneration == 50 && rx_message_hdx.identifier == HALDEX_GEN5_STATUS_ID && rx_message_hdx.extd)
        {
          // 0CQ status frame 0x17F0000F (29-bit, DLC 8). OEM code (spec §6): byte 0
          // bit 0 = 1 while the unit is in a fault-reaction state. Byte 7 bit 7 is
          // set by its fault-memory loop but is ALSO set on healthy cars in our logs,
          // so it is not decoded (spec §8 item 5).
          if (rx_message_hdx.data_length_code >= 1)
          {
            gen5StatusFaultReaction = (rx_message_hdx.data[0] & (1 << 0)); // byte 0 bit 0
            gen5StatusFrameMs = millis();
          }
        }
        else if (haldexGeneration == 52 && rx_message_hdx.identifier == QUERSPERRE_03)
        {
          // VAQ / Quersperre_03 (0x137, DLC 4) per MQB FCAN K-matrix - the
          // front-lock counterpart of Allrad_03, same byte plan:
          //   b1[0..3] Quersperre_03_BZ   rolling counter
          //   b1[4..6] QUER_Sta_Quersperre 0 rule mode / 1 driver / 2 err open /
          //                                3 err closed / 4 temp shutdown / 5 comms
          //   b1[7]    QUER_Gleichlauf     1 = synchronised
          //   b2       QUER_Ist_Proz       0.4 %/bit (250 = 100 %); 252 SafeState
          //                                sensor unverified, 253 error+closed,
          //                                254 init, 255 error -> read as 0 %
          //   b3       Sendestatus b0, VAQ_Charisma_FahrPr b1..4, Status b5..6
          received_haldex_engagement_raw = rx_message_hdx.data[2];
          if (received_haldex_engagement_raw >= 252)
            received_haldex_engagement = 0;
          else
            received_haldex_engagement = (uint8_t)map(received_haldex_engagement_raw > 250 ? 250 : received_haldex_engagement_raw, 0, 250, 0, 100);
          received_haldex_state = rx_message_hdx.data[3];
          received_quer_state = (rx_message_hdx.data[1] >> 4) & 0x07;
          received_quer_sync = (rx_message_hdx.data[1] >> 7) & 0x01;
          hdxFbCapture(rx_message_hdx);
          clearHaldexHealth();
          // V11 TODO (bench): the VAQ file has its app code but not its 0x137
          // builder traced yet (spec_gen5_0cq §8.8), so these K-matrix states are
          // not confirmed from the OEM code - no health flags are set from them.
          haldexHealthSupported = false;
          received_lever_request_valid = false; // 0CQ-only bit
          // Same slot for the VAQ: QUER_Sta_Quersperre shares the first four
          // meanings with ALR_Sta_Laengssperre (0 rule, 1 driver, 2/3 error).
          received_long_lock_state = received_quer_state;
        }
        break;

      case 51:
        // Gen5 (0AY, 0AY907554F) Haldex - a PQ unit: feedback is PQ Allrad_1
        // (0x2C0, DLC 8). V11.00.0: decoded per the 0AY's own code
        // (spec_gen5_0ay §6) - same layout as Gen4 0BR.
        if (rx_message_hdx.identifier == HALDEX_ID)
        {
          // Byte 1 = the clutch torque the unit is applying, on the SAME scale
          // as our Bremse_4 lever: 0x7F / 0x80 = none, 0x80 + n = n x 112 units,
          // 0xFD = our 100 % request (0xFD on the lever), 0xFE = at / over 0xFD.
          // The unit never sends 0xFF. Before V11 this was mapped 0x80..0xFF
          // with a ">= 99 -> 100" snap, because 0xFD only read 98-99 % on that
          // scale (spec §7 #10). The correct scale makes the snap unnecessary.
          received_haldex_engagement_raw = rx_message_hdx.data[1];
          int engagementPercent = 0; // worked out in int so it can't wrap round
          if (received_haldex_engagement_raw >= 0xFD)
          {
            engagementPercent = 100; // full request reached (0xFD) or above it (0xFE)
          }
          else if (received_haldex_engagement_raw > 0x80)
          {
            engagementPercent = ((int)received_haldex_engagement_raw - 0x80) * 100 / 125; // 0x81..0xFC -> 0..99 %
          }
          if (engagementPercent > 100)
          {
            engagementPercent = 100;
          }
          received_haldex_engagement = (uint8_t)engagementPercent;

          received_haldex_state = rx_message_hdx.data[0];
          clearHaldexHealth();
          // Byte 0 flags the 0AY really writes (spec_gen5_0ay §6). b0.5 (warning
          // lamp) and b0.6 (speed limit) are never set by the 0AY - always 0 -
          // so they stay false; b0.7 is always 0 too.
          received_report_clutch1 = (received_haldex_state & (1 << 0));  // b0.0 clutch-system fault present
          received_temp_protection = (received_haldex_state & (1 << 1)); // b0.1 over-temperature protection
          received_report_clutch2 = (received_haldex_state & (1 << 2));  // b0.2 clutch stiffness / actuator fault
          received_coupling_open = (received_haldex_state & (1 << 3));   // b0.3 clutch commanded fully open (status)
          received_limp_mode = (received_haldex_state & (1 << 4));       // b0.4 limp / emergency (also set if Motor_5 is missing)

          if (rx_message_hdx.data_length_code >= 5)
          {
            received_temp_sensor_fault = (rx_message_hdx.data[3] & (1 << 3)); // b3.3 the unit's temperature sensor is invalid
            // b3.7 = Fehlerspeichereintrag_Allrad_1 (vw_pq.dbc); the 0AY writes it
            // from its background memory self-test (spec §6, meaning inferred).
            received_fault_stored = (rx_message_hdx.data[3] & (1 << 7));
            // b4 = Haldex temperature, degC = b4 - 60; 0xFF = sensor invalid
            if (rx_message_hdx.data[4] == 0xFF)
            {
              received_haldex_temp_c = -128; // no valid temperature
            }
            else
            {
              received_haldex_temp_c = (int16_t)rx_message_hdx.data[4] - 60;
            }
          }
          // b7 high nibble is an alive counter (0-15); not decoded - a silent
          // unit already shows as "no Haldex" through hasCANHaldex.
          haldexHealthSupported = true;
          received_long_lock_state = 0; // not present on PQ Allrad_1
        }
        break;

      // Volvo / SAAB (V10.00.0). What the status number means is not known yet,
      // so it is shown as 0-255 -> 0-100 % until logs give the real scaling.
      case GEN_VOLVO_P2_GEN2:
        if (rx_message_hdx.identifier == VOLVO_P2G2_HALDEX_ID)
        {
          received_haldex_engagement_raw = rx_message_hdx.data[6]; // status byte
          received_haldex_engagement = map(received_haldex_engagement_raw, 0, 255, 0, 100);
          received_haldex_state = rx_message_hdx.data[0]; // byte 0 kept raw for the logs (not decoded)
          hdxFbCapture(rx_message_hdx);
          clearHaldexHealth(); // no OEM code held for the pre-facelift unit: no health bits known
          haldexHealthSupported = false;
          received_long_lock_state = 0;
        }
        break;

      case GEN_VOLVO_P2_GEN3:
        // OEM code (S60R DEM signal table, leaf_trace/volvo_bitrate.md goal 4):
        //   bytes 0-4      never written by the unit
        //   byte 5 b0-2    "updated" bits for the fields below
        //   byte 6         measured clutch value (analog input, most likely clutch
        //                  pressure = real engagement), ~0-150 at sensor full scale
        //   byte 7 b6-7    fault / limp: 0 normal, 1 mode-selector condition,
        //                  2 not ready or stored input fault (clutch held at 0),
        //                  3 a diagnostic monitor reports a fault
        //   byte 7 b2-3    3 = input monitors clear, anything else = an input the
        //                  unit watches is missing / flagged (comms)
        //   byte 7 b0-1    internal state (2 or 3), not decoded
        if (rx_message_hdx.identifier == VOLVO_P2G3_HALDEX_ID)
        {
          received_haldex_engagement_raw = rx_message_hdx.data[6]; // byte 6 = measured clutch value
          int engagementPercent = (int)received_haldex_engagement_raw * 100 / VOLVO_P2G3_ENGAGEMENT_FULL_SCALE;
          if (engagementPercent > 100)
          {
            engagementPercent = 100; // clamp - full scale is a bench calibration value
          }
          received_haldex_engagement = (uint8_t)engagementPercent;
          received_haldex_state = rx_message_hdx.data[7]; // byte 7 = status bits (shown as binary on the dashboard)
          hdxFbCapture(rx_message_hdx);

          clearHaldexHealth();
          haldexHealthSupported = true;
          received_long_lock_state = 0;

          uint8_t faultLimpValue = (rx_message_hdx.data[7] >> 6) & 0x03; // byte 7 bits 6-7
          if (faultLimpValue == 2)
          {
            received_limp_mode = true;     // unit not ready / stored input fault - clutch held at 0
            received_coupling_open = true; // ...so the clutch is open
          }
          else if (faultLimpValue == 3)
          {
            received_report_clutch1 = true; // a diagnostic monitor reports a fault
          }
          // faultLimpValue 1 = mode-selector condition: status, not a fault

          uint8_t inputMonitorValue = (rx_message_hdx.data[7] >> 2) & 0x03; // byte 7 bits 2-3
          if (inputMonitorValue != 3)
          {
            // An input the unit watches is missing or flagged - shown as a
            // "signal plausibility fault". In standalone this points straight at
            // a frame we aren't sending (yet).
            received_report_clutch2 = true;
          }
          // V11 TODO (bench): confirm byte 6 full scale (VOLVO_P2G3_ENGAGEMENT_FULL_SCALE)
          // at 100 % lock, and which situations set byte 7 bits 6-7 = 1 / 2 / 3.
        }
        break;

      case GEN_VOLVO_GEN4:
        if (rx_message_hdx.identifier == VOLVO_G4_HALDEX_ID)
        {
          received_haldex_engagement_raw = rx_message_hdx.data[7]; // status byte
          received_haldex_engagement = map(received_haldex_engagement_raw, 0, 255, 0, 100);
          received_haldex_state = rx_message_hdx.data[0]; // byte 0 kept raw for the logs (not decoded)
          hdxFbCapture(rx_message_hdx);
          clearHaldexHealth(); // no OEM code held for the P3 unit: no health bits known
          haldexHealthSupported = false;
          received_long_lock_state = 0;
        }
        break;

      case GEN_VOLVO_SPC_GEN5:
        // Volvo Gen5 "SPC" 31380867 (V11.00.0). Reply 0x126, 8 bytes, high byte
        // first, no checksum or counter (OEM code: spec_ppc_volvo_spc_bmw.md section 6):
        //   bytes 0-1 (15 bits) = clutch torque the unit is really delivering
        //                         (forced to 0 when the unit is in a fault / off state)
        //   byte 3 bits 3-4      = status: 0 normal, 1 fault / torque inhibited,
        //                         2 special state (strategy state 4, meaning unknown)
        // No warning-lamp bit is sent on 0x126.
        if (rx_message_hdx.identifier == VOLVO_SPC_HALDEX_ID && rx_message_hdx.data_length_code >= 4)
        {
          uint16_t torqueCount = (uint16_t)(rx_message_hdx.data[0] & 0x7F); // byte 0 bits 0-6 = top 7 bits
          torqueCount = (uint16_t)((torqueCount << 8) | rx_message_hdx.data[1]); // byte 1 = bottom 8 bits

          // engagement % = delivered torque against full scale, worked out in a
          // whole number first and clamped so it can never wrap past 100
          int engagementPercent = (int)((uint32_t)torqueCount * 100 / VOLVO_SPC_FEEDBACK_FULL_SCALE);
          if (engagementPercent > 100)
          {
            engagementPercent = 100;
          }
          if (engagementPercent < 0)
          {
            engagementPercent = 0;
          }
          // V11 TODO (bench): full scale is the OEM code's largest possible count,
          // not a measured full lock - set VOLVO_SPC_FEEDBACK_FULL_SCALE on the bench.
          received_haldex_engagement = (uint8_t)engagementPercent;

          uint16_t rawScaled = torqueCount >> 5; // 15-bit torque / 32 so it fits one byte (6710 -> 209)
          if (rawScaled > 255)
          {
            rawScaled = 255;
          }
          received_haldex_engagement_raw = (uint8_t)rawScaled;
          received_haldex_state = rx_message_hdx.data[3]; // byte 3 kept raw (status in bits 3-4)

          const uint8_t unitStatus = (rx_message_hdx.data[3] >> 3) & 0x03; // byte 3 bits 3-4
          clearHaldexHealth();
          if (unitStatus == 1)
          {
            received_report_clutch1 = true; // status 1: fault - the unit has withdrawn clutch torque
            received_limp_mode = true;      // ... and is running its fault reaction (clutch held open)
          }
          if (torqueCount == 0)
          {
            received_coupling_open = true; // delivering no torque: clutch fully open (status, not a fault)
          }
          haldexHealthSupported = true;
          received_long_lock_state = 0;
          hdxFbCapture(rx_message_hdx);
        }
        break;

      case GEN_VOLVO_P1:
        // Volvo P1 AWD module (V11.00.1), 0x1960000A every 20 ms - its only frame.
        // From Adam's V50 T5 AWD logs (P1_V50_T5_AWD_DECODE.md):
        //   bytes 0-3   always 00          byte 4   always 05
        //   byte 5      status: 2E ignition on / engine off, 2C or 3C running
        //   byte 6      bits 2-7 always 111000, bits 0-1 = top 2 bits of engagement
        //   byte 7      bottom 8 bits of engagement (10-bit: 0 open .. 540 full lock)
        if (rx_message_hdx.identifier == VOLVO_P1_HALDEX_ID && rx_message_hdx.extd &&
            rx_message_hdx.data_length_code >= 8)
        {
          uint16_t engagementCount = (rx_message_hdx.data[6] & 0x03) << 8; // byte 6 bits 0-1 = top 2 bits
          engagementCount = engagementCount | rx_message_hdx.data[7];      // byte 7 = bottom 8 bits
          volvoP1EngagementCount = engagementCount;                        // kept for the Volvo P1 card's live readout

          // engagement % = count against full lock, worked out in a whole number and clamped
          uint32_t engagementPercent = (uint32_t)engagementCount * 100 / VOLVO_P1_ENGAGEMENT_FULL_SCALE;
          if (engagementPercent > 100)
          {
            engagementPercent = 100;
          }
          received_haldex_engagement = (uint8_t)engagementPercent;

          // raw value for the dashboard / logs: the 10-bit count / 4 so it fits one byte (540 -> 135)
          received_haldex_engagement_raw = (uint8_t)(engagementCount >> 2);
          received_haldex_state = rx_message_hdx.data[5]; // byte 5 = status (2C / 3C / 2E)
          hdxFbCapture(rx_message_hdx);
          clearHaldexHealth(); // no fault bits found in the logs yet
          haldexHealthSupported = false;
          received_long_lock_state = 0;
        }
        break;

      case GEN_SAAB_93_XWD:
        if (rx_message_hdx.identifier == SAAB_93_HALDEX_ID)
        {
          // 11-bit value: low 3 bits of byte 0 then all of byte 1, cut down to 0-255
          uint16_t statusValue = (rx_message_hdx.data[0] & 0x07); // byte 0 bits 0-2 = top 3 bits
          statusValue = (statusValue << 8) | rx_message_hdx.data[1]; // byte 1 = bottom 8 bits
          received_haldex_engagement_raw = statusValue >> 3;          // 0-2047 -> 0-255
          received_haldex_engagement = map(received_haldex_engagement_raw, 0, 255, 0, 100);
          received_haldex_temp_c = (int16_t)rx_message_hdx.data[2] - 40; // byte 2 = temperature + 40 degC (GM style)
          received_haldex_state = rx_message_hdx.data[0];                // byte 0 kept raw for the logs (not decoded)
          hdxFbCapture(rx_message_hdx);
          clearHaldexHealth(); // no OEM code held for the 9-3 unit: only the temperature is known
          haldexHealthSupported = false;
          received_long_lock_state = 0;
        }
        break;

      default:
        break;
      }

      // Haldex health flags (fault / over-temp / limp / lamp ...) are decoded
      // inside each generation's case above since V11.00.0 - the bit layout is
      // different on every unit (OEM code findings, CONTROL_AND_DTC_SPEC.md).

      // UDS MQB: mirror 0x779 response frames into the UDS task queue when polling is active.
      // While our poller owns the diagnostic channel and no external tool is on
      // the chassis side, every 0x779 is a reply to a request the car never made
      // - keep it off Bus 0 so the gateway/OBD side sees no diagnostic traffic it
      // did not ask for. The moment a real tool is seen on Bus 0 the poller backs
      // off and replies are forwarded again (VCDS / gauge passthrough intact).
      bool udsSuppressForward = false;
      if (liveDiagEnabled && udsRxQueue != nullptr && rx_message_hdx.identifier == udsHaldexRespId)
      {
        if (xQueueSend(udsRxQueue, &rx_message_hdx, 0) != pdTRUE)
        {
          // One missed UDS response (the poller re-requests) - log, don't block.
          DEBUG("CAN - UDS MQB tap queue full, dropped frame 0x779");
        }
        udsSuppressForward = udsPollActive && !externalDiagActive();
      }

      // /api/uds/read tap (Bus 1 reads): copy, the frame still forwards below.
      if (udsWebRespId != 0 && udsWebBus == 1 && udsWebRxQueue != nullptr &&
          rx_message_hdx.identifier == udsWebRespId)
      {
        if (xQueueSend(udsWebRxQueue, &rx_message_hdx, 0) != pdTRUE)
        {
          DEBUG("CAN - UDS web tap queue full, dropped frame 0x%03X", (unsigned)rx_message_hdx.identifier);
        }
        udsSuppressForward = true; // reply to our own web request - same rule as the poller
      }

      // KWP2000-over-TP2.0 (Gen2/4): mirror channel-setup + negotiated data-channel
      // frames into the TP2.0 task queue. Frame is still forwarded to Bus 0 below.
      if (liveDiagEnabled && tp20RxQueue != nullptr && !rx_message_hdx.extd)
      {
        const uint32_t tpId = rx_message_hdx.identifier;
        if (tpId == KWP_TP20_SETUP_RX_ID || tpId == KWP_TP20_TESTER_RX_ID ||
            (kwpTp20EcuTxId != 0 && tpId == kwpTp20EcuTxId))
        {
          xQueueSend(tp20RxQueue, &rx_message_hdx, 0);
        }
      }

      if (!udsSuppressForward)
      {
        canTransmit(twai_bus_0, &rx_message_hdx);
      }
    } while (twai_receive_v2(twai_bus_1, &rx_message_hdx, 0) == ESP_OK);
  }
}