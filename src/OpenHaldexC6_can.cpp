#include <OpenHaldexC6_can.h>
#include <OpenHaldexC6_Calculations.h>
#include <OpenHaldexC6_Analyzer.h>
#include <OpenHaldexC6_UDS.h>

// =============================================================================
// CAN - what both buses share (V11.00.14 split)
// =============================================================================
//   OpenHaldexC6_can.cpp  driver set-up, bus-off recovery, counted transmit,
//                         RX census, loop test, 0x6B0 broadcast, bus speed
//   OpenHaldexC6_CHS.cpp  Bus 0 (chassis): setupCHS(), per-generation chassis
//                         decode, frames to the Haldex, parseCANChs()
//   OpenHaldexC6_HDX.cpp  Bus 1 (Haldex): setupHDX(), per-generation feedback
//                         decode, diagnostic taps, parseCANHdx()
// =============================================================================

// ---- Loop test state (V11.00.12) --------------------------------------------
// The old "link CHS and HDX and look for two ticks" check relied on the 0x6B0
// broadcast going out on Bus 0 all the time. It no longer does (off unless the
// setting is on AND the Haldex is talking), so a linked pair showed one tick.
// This sends one marked frame from each bus and reports which bus got which.
static volatile uint32_t loopTestStartMs = 0;
static volatile uint8_t loopTestSeq = 0;
static volatile uint8_t loopTestSeenBits = 0;
static volatile uint8_t loopTestSentBits = 0;

// ---- Small helpers -----------------------------------------------------------

static void loopTestBuild(twai_message_t &f, uint8_t originBus)
{
  f = {};
  f.identifier = CAN_LOOP_TEST_ID;
  f.extd = 1;
  f.data_length_code = 8;
  f.data[0] = 'O';
  f.data[1] = 'H';
  f.data[2] = 'X';
  f.data[3] = 'L';
  f.data[4] = 'P';
  f.data[5] = loopTestSeq;
  f.data[6] = originBus;
}

// Tally every Bus 1 frame per ID (serial lab RXIDS). Linear scan of a 16-slot
// table: Bus 1 carries only what the module sends (our own TX is not echoed),
// so a handful of IDs is the norm. Called from the parse task only.
static void rxCensus(HdxRxStat *table, volatile uint32_t &dropped, const twai_message_t &m)
{
  const uint32_t now = millis();
  int slot = -1;
  for (int i = 0; i < HDX_RX_STATS_MAX; i++)
  {
    if (table[i].id == m.identifier && table[i].extd == (m.extd ? 1 : 0))
    {
      slot = i;
      break;
    }
    if (slot < 0 && table[i].id == 0)
      slot = i; // first free slot, used only if no match follows
  }
  if (slot < 0)
  {
    dropped = dropped + 1;
    return;
  }
  HdxRxStat &s = table[slot];
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

// ---- Setup -------------------------------------------------------------------

// Which CAN bus speed a Haldex generation needs. Volvo P2 cars before the 2005
// facelift run their fast CAN bus at 250 kbit/s; everything else is 500 kbit/s.
uint16_t canSpeedForGeneration(uint8_t generation)
{
  if (generation == GEN_VOLVO_P2_GEN2)
  {
    return 250; // [REA none for gen 21: 2003 car log + S60R bootloader inference only (audit_volvo_c166 #24/#85)]
  }
  return 500; // Volvo gen 3 [REA C1BTR 0x1641 #19-21], gen 53 [REA ppc #29], gen 43 / 31 [bench / LOG]
              // gen 46 Land Rover (V11.00.13) [REA audit_landrover #4/E1: bootloader BTR 0x3E81 = 500k @ 20 MHz; Evoque CAN1 0x1643 also 500k]
}

// Install and start one TWAI controller with the shared settings (queue sizes,
// IRAM interrupt, sleep power-down, alerts). setupCHS() / setupHDX() call this
// for their own controller; canBusSpeedKbps must already be set (setupCAN).
void canInstallBus(uint8_t controllerId, int txPin, int rxPin, twai_handle_t *bus)
{
  twai_general_config_t generalConfig = TWAI_GENERAL_CONFIG_DEFAULT(gpio_num_t(txPin), gpio_num_t(rxPin), TWAI_MODE_NO_ACK); // TWAI_MODE_NORMAL, TWAI_MODE_NO_ACK or TWAI_MODE_LISTEN_ONLY
  twai_timing_config_t timing500 = TWAI_TIMING_CONFIG_500KBITS();                                                        // normal CAN speed
  twai_timing_config_t timing250 = TWAI_TIMING_CONFIG_250KBITS();                                                        // Volvo P2 Gen2 CAN speed
  twai_timing_config_t timingConfig = timing500;                                                                             // the one actually used
  if (canBusSpeedKbps == 250)
  {
    timingConfig = timing250;
  }
  twai_filter_config_t filterConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL(); // accept all messages

  // generalConfig.intr_flags = ESP_INTR_FLAG_LOWMED;  //Optional - move canbus irq to free up the default level 1 IRQ it will take up.  Todo
  // Queues come out of heap at 13 B per frame, per bus. Was 1024/2048 (~80 KB for both buses).
  // 512 RX = ~115 ms of a 100%-loaded 500k bus; the parse tasks are the top app priority.
  // Check the debugMemory "CAN" line for missed/overrun counts if this is ever in doubt.
  generalConfig.tx_queue_len = 256;
  generalConfig.rx_queue_len = 512;

  // V11.00.6: run the CAN interrupt from IRAM (fast internal RAM) so it keeps working while
  // flash is being written (settings saves, OTA). Without this the interrupt is held off for
  // the whole flash write, the CAN chip's own 5-frame buffer overflows and frames are lost.
  // With it, frames wait in the 512-frame queue and go out a few ms late instead.
  // LEVEL1 = the normal interrupt level (the driver's default); the | adds the IRAM flag to it.
  // Needs CONFIG_TWAI_ISR_IN_IRAM=y in platformio.ini (custom_sdkconfig), or the driver
  // refuses to install (ESP_ERR_INVALID_ARG) and the unit reboots at start-up.
#if CONFIG_TWAI_ISR_IN_IRAM
  generalConfig.intr_flags = ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM;
#else
  // V11.00.9: the framework libs this build linked were made WITHOUT CONFIG_TWAI_ISR_IN_IRAM
  // (a PC whose C:\.platformio-openhaldex libs are older than the synced sdkconfig.defaults hash -
  // pioarduino then skips the rebuild). Asking for the IRAM flag anyway made twai_driver_install
  // fail and the unit abort() at every boot - no CAN frames at all. Fall back to the normal
  // interrupt so it still runs; frames can be dropped during flash writes until the libs are rebuilt.
#warning "CONFIG_TWAI_ISR_IN_IRAM is not set in the linked framework libs - CAN interrupt NOT in IRAM. Rebuild the custom IDF libs before a release build."
  generalConfig.intr_flags = ESP_INTR_FLAG_LEVEL1;
#endif

  // Allow the TWAI power domain to be powered down during light sleep; the
  // driver auto-saves/restores its registers + RX queue across sleep cycles.
  // Sleep entry itself is gated at runtime by `canSleepEnabled`.
  generalConfig.general_flags.sleep_allow_pd = 1;

  generalConfig.controller_id = controllerId;
  ESP_ERROR_CHECK(twai_driver_install_v2(&generalConfig, &timingConfig, &filterConfig, bus));
  DEBUG("CAN - Driver %d Installed", controllerId);
  ESP_ERROR_CHECK(twai_start_v2(*bus));
  DEBUG("CAN - Driver %d Started", controllerId);

  // Alerts for frame receive, error states and the full bus-off / recovery
  // lifecycle so canBusRecovery() can drive a clean restart (v2 driver -> per-handle alerts).
  const uint32_t alertsToEnable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS |
                                  TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL |
                                  TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_OFF |
                                  TWAI_ALERT_BUS_RECOVERED | TWAI_ALERT_RX_FIFO_OVERRUN;
  if (twai_reconfigure_alerts_v2(*bus, alertsToEnable, NULL) == ESP_OK)
  {
    DEBUG("CAN - Driver %d alerts configured", controllerId);
  }
  else
  {
    DEBUG("CAN - Driver %d: failed to configure alerts!", controllerId);
  }
}

void setupCAN()
{
  // Bus speed comes from the saved Haldex generation (readEEP runs before this).
  canBusSpeedKbps = canSpeedForGeneration(haldexGeneration);
  DEBUG("CAN - Bus speed %d kbit/s (generation %d)", canBusSpeedKbps, haldexGeneration);
  setupCHS(); // Bus 0 - chassis (OpenHaldexC6_CHS.cpp)
  setupHDX(); // Bus 1 - Haldex  (OpenHaldexC6_HDX.cpp)
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

// ---- Main code ---------------------------------------------------------------

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
  const bool isBus0 = (bus == twaiBus0);
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
  twai_handle_t buses[2] = {twaiBus0, twaiBus1};
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

void chsRxCensus(const twai_message_t &m) { rxCensus(chsRxStats, chsRxDroppedIds, m); }
void hdxRxCensus(const twai_message_t &m) { rxCensus(hdxRxStats, hdxRxDroppedIds, m); }

void canRxCensusClear()
{
  for (uint8_t i = 0; i < HDX_RX_STATS_MAX; i++)
  {
    hdxRxStats[i].id = 0;
    chsRxStats[i].id = 0;
  }
  hdxRxDroppedIds = 0;
  chsRxDroppedIds = 0;
}

bool canLoopTestStart()
{
  if (receivedVehicleSpeed > 0)
  {
    return false; // never on a moving car
  }
  loopTestSeq = loopTestSeq + 1;
  loopTestSeenBits = 0;
  loopTestSentBits = 0;
  uint32_t now = millis();
  loopTestStartMs = now ? now : 1;
  twai_message_t f;
  loopTestBuild(f, 0);
  if (canTransmit(twaiBus0, &f))
    loopTestSentBits = loopTestSentBits | 0x01;
  loopTestBuild(f, 1);
  if (canTransmit(twaiBus1, &f))
    loopTestSentBits = loopTestSentBits | 0x02;
  return true;
}

uint8_t canLoopTestSeen() { return loopTestSeenBits; }
uint8_t canLoopTestSent() { return loopTestSentBits; }
uint32_t canLoopTestAgeMs() { return loopTestStartMs ? (millis() - loopTestStartMs) : 0; }

// true = a loop-test frame: noted here and swallowed (no tick, no parse, no
// forwarding - forwarding it across linked buses would loop it forever).
bool canLoopTestCatch(const twai_message_t &m, uint8_t rxBus)
{
  if (!m.extd || m.identifier != CAN_LOOP_TEST_ID || m.data_length_code < 7 ||
      m.data[0] != 'O' || m.data[1] != 'H' || m.data[2] != 'X' || m.data[3] != 'L' || m.data[4] != 'P')
  {
    return false;
  }
  if (m.data[5] == loopTestSeq)
  {
    const uint8_t origin = m.data[6];
    uint8_t bit = 0;
    if (origin == 0 && rxBus == 1) bit = LOOP_CHS_TO_HDX;
    else if (origin == 1 && rxBus == 0) bit = LOOP_HDX_TO_CHS;
    else if (origin == 0 && rxBus == 0) bit = LOOP_ECHO_CHS;
    else if (origin == 1 && rxBus == 1) bit = LOOP_ECHO_HDX;
    loopTestSeenBits = loopTestSeenBits | bit;
  }
  return true;
}

void broadcastOpenHaldex(void *arg)
{
  while (1)
  {
#if detailedDebugStack
    stackbroadcastOpenHaldex = uxTaskGetStackHighWaterMark(NULL);
#endif

    // Analyzer mode keeps the CAN bridge alive but suppresses OpenHaldex broadcast frames.
    // The "Broadcast OpenHaldex over CAN" setting was never checked here, so
    // 0x6B0 went out on the chassis bus every 200 ms on every unit, forever -
    // including with the car locked: the drivetrain ECUs saw traffic and never
    // slept, so the bus stayed above the low-power threshold and the controller
    // never slept either. Only transmit when the setting is on AND the Haldex
    // side is talking (it goes quiet within seconds of terminal 15 dropping).
    const uint32_t lastHaldexMs = lastCANHaldexTick; // copy: the Haldex task writes it
    const bool haldexAwake = lastHaldexMs != 0 &&
                             (millis() - lastHaldexMs) < BROADCAST_HALDEX_QUIET_MS;
    if (analyzerMode || !broadcastOpenHaldexOverCAN || !haldexAwake)
    {
      vTaskDelay(broadcastRefresh / portTICK_PERIOD_MS);
      continue;
    }

    // build up the 'OpenHaldex' frame for broadcasting back over CAN
    twai_message_t broadcastFrame = {}; // V11.00.12: zeroed - a stray "self" bit made the controller receive its own frame
    broadcastFrame.identifier = OPENHALDEX_BROADCAST_ID;
    broadcastFrame.extd = 0;
    broadcastFrame.rtr = 0;
    broadcastFrame.data_length_code = 8;
    broadcastFrame.data[0] = 0;
    broadcastFrame.data[1] = isStandalone;
    broadcastFrame.data[2] = (uint8_t)receivedHaldexEngagementRaw;
    broadcastFrame.data[3] = (uint8_t)lockTarget;
    broadcastFrame.data[4] = receivedVehicleSpeed;
    broadcastFrame.data[5] = state.modeOverride;
    broadcastFrame.data[6] = (uint8_t)state.mode;
    broadcastFrame.data[7] = (uint8_t)receivedPedalValue;

    canTransmit(twaiBus0, &broadcastFrame);

    vTaskDelay(broadcastRefresh / portTICK_PERIOD_MS);
  }
}
