#include <OpenHaldexC6_IO.h>
#include <OpenHaldexC6_can.h>
#include <OpenHaldexC6_WiFi.h>

// Low-power state: 
//   WATCHING = WiFi Active, Normal IO
//   SLEEPING = WiFi Off, LED off, and (if canSleepAggressive) CAN transceivers in standby
//   using GPIO ISR Wake on the CAN_RX pins.
#define LP_WATCHING 0
#define LP_SLEEPING 1

static uint8_t  lpState              = LP_WATCHING;
static uint32_t lpNoClientsSince     = 0;
static uint32_t lpLastFpsCheck       = 0;
static uint32_t lpLastChassisSnap    = 0;
static uint32_t lpLastHaldexSnap     = 0;
static uint32_t lpChassisFps         = 0;
static uint32_t lpHaldexFps          = 0;

// Aggressive-mode state.
static bool     lpTransceiversStandby = false; // CAN_RS pins driven high (TCAN1044 standby)
static bool     lpWakeIsrAttached     = false; // GPIO ISRs currently armed on CAN_RX
static bool     lpTasksSuspended      = false; // background periodic tasks parked

// Background tasks paused while LP_SLEEPING + aggressive. None of these have
// anything useful to do without bus traffic - frame generators TX into a
// dead bus, broadcastOpenHaldex sends state nobody is listening to,
// showHaldexState logs to a serial port that is almost certainly closed.
// Frame generators are only resumed in standalone mode (they were never
// running in OEM mode - see setupTasks()).
struct LpManagedTask { TaskHandle_t *handle; bool standaloneOnly; };
static const LpManagedTask lpManagedTasks[] = {
  { &handle_broadcastOpenHaldex,   false },
  { &handle_showHaldexState,       false },
  { &handle_frames1000,            true  },
  { &handle_frames250,             true  },
  { &handle_frames200,             true  },
  { &handle_frames100,             true  },
  { &handle_frames50,              true  },
  { &handle_frames25,              true  },
  { &handle_frames20,              true  },
  { &handle_frames13,              true  },
  { &handle_frames10,              true  },
  { &handle_gen41_dual_bus_rates,  true  },
};

// GPIO ISRs - fire on the first falling edge of CAN_RX while the
// transceivers are parked in standby. The TCAN1044 in standby still drives
// RXD low on any valid bus activity, so this gives us a microsecond-scale
// wake without ever having to power the transceiver up to look. We flag the
// request and notify updateTriggers so it can act immediately. This is the
// only wake path in aggressive mode - there is no periodic probe.
static void IRAM_ATTR onCanWakeEdge()
{
  canWakeRequest = true;
  if (handle_updateTriggers)
  {
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(handle_updateTriggers, &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
  }
}

static void lpAttachWakeIsrs()
{
  if (lpWakeIsrAttached) return;
  attachInterrupt(digitalPinToInterrupt(CAN0_RX), onCanWakeEdge, FALLING);
  attachInterrupt(digitalPinToInterrupt(CAN1_RX), onCanWakeEdge, FALLING);
  lpWakeIsrAttached = true;
}

static void lpDetachWakeIsrs()
{
  if (!lpWakeIsrAttached) return;
  detachInterrupt(digitalPinToInterrupt(CAN0_RX));
  detachInterrupt(digitalPinToInterrupt(CAN1_RX));
  lpWakeIsrAttached = false;
}

// TCAN1044: RS pin HIGH = standby (RXD reflects wake events, very low current draw);
//           RS pin LOW  = normal operation. ISRs are armed BEFORE entering
// standby so we never miss the first SOF edge, and torn down AFTER leaving
// standby so the TWAI driver has uncontested ownership of the RX pin.
static void lpSetTransceiverStandby(bool standby)
{
  if (standby == lpTransceiversStandby) return;
  if (standby)
  {
    canWakeRequest = false;
    lpAttachWakeIsrs();
    digitalWrite(CAN0_RS, HIGH);
    digitalWrite(CAN1_RS, HIGH);
  }
  else
  {
    digitalWrite(CAN0_RS, LOW);
    digitalWrite(CAN1_RS, LOW);
    lpDetachWakeIsrs();
    canWakeRequest = false;
  }
  lpTransceiversStandby = standby;
}

static void lpSuspendBackgroundTasks()
{
  if (lpTasksSuspended) return;
  for (uint8_t i = 0; i < sizeof(lpManagedTasks) / sizeof(lpManagedTasks[0]); i++)
  {
    TaskHandle_t handle = *lpManagedTasks[i].handle;
    if (handle && eTaskGetState(handle) != eSuspended)
    {
      vTaskSuspend(handle);
    }
  }
  lpTasksSuspended = true;
}

static void lpResumeBackgroundTasks()
{
  if (!lpTasksSuspended) return;
  for (uint8_t i = 0; i < sizeof(lpManagedTasks) / sizeof(lpManagedTasks[0]); i++)
  {
    if (lpManagedTasks[i].standaloneOnly && !isStandalone) continue;
    
    TaskHandle_t handle = *lpManagedTasks[i].handle;
    if (handle && eTaskGetState(handle) == eSuspended)
    {
      vTaskResume(handle);
    }
  }
  lpTasksSuspended = false;
}

void setupIO()
{
  // using the TCAN1044 for CAN control
  pinMode(CAN0_RS, OUTPUT);   // gpio for controlling can_0 state - enabled or disabled
  pinMode(CAN1_RS, OUTPUT);   // gpio for controlling can_0 slope - enabled or disabled
  digitalWrite(CAN0_RS, LOW); // set chip enable
  digitalWrite(CAN1_RS, LOW); // set chip enable

  pinMode(gpio_hb_in, INPUT_PULLDOWN);    // gpio for handbrake in signal (pulldown so floating harness draws 0mA)
  pinMode(gpio_brake_in, INPUT_PULLDOWN); // gpio for brake in signal (pulldown so floating harness draws 0mA)
  pinMode(gpio_hb_out, OUTPUT);    // gpio for handbrake out signal
  pinMode(gpio_brake_out, OUTPUT); // gpio for brake out signal

  strip.begin();            // begin RGB LED onboard
  strip.setBrightness(255); // always full library scale; brightness controlled via color values
}

void modeChange(void)
{
#if enableDebug
  DEBUG("Internal mode button pressed!");
#endif
  if (disableOnboardButton)
    return; // onboard button disabled, ignore press

  uint8_t next_mode = (uint8_t)state.mode + 1; // Determine the next mode in the sequence.

  if (isStandalone)
  {
    // In standalone mode, skip MODE_EXPERT when cycling through modes as it's not used.
    if (next_mode == MODE_EXPERT)
    {
      next_mode++;
    }
  }

  if (next_mode < (uint8_t)openhaldex_mode_t_MAX)
  {
    state.mode = (openhaldex_mode_t)next_mode; // If the next mode is valid, change the current mode to it.
  }
  // On overflow, start over from MODE_STOCK.
  else
  {
    if (!isStandalone)
    {
      state.mode = MODE_STOCK; // If in non-standalone mode, loop back to MODE_STOCK after the last mode.
    }
    else
    {
      state.mode = MODE_FWD; // If in standalone mode, loop back to MODE_FWD after the last mode.
    }
  }
  lastMode = state.mode;
}

void modeChangeExt(void)
{
#if enableDebug
  DEBUG("External mode button pressed!");
#endif
  if (disableExternalButton)
    return; // external button disabled, ignore press
  if (extBtnForceMode)
    return; // Hold mode: short press has no effect; only hold triggers force mode

  uint8_t next_mode = (uint8_t)state.mode + 1; // Determine the next mode in the sequence.

  if (isStandalone)
  {
    // In standalone mode, skip MODE_EXPERT when cycling through modes, as it's not used.
    if (next_mode == MODE_EXPERT)
    {
      next_mode++;
    }
  }

  if (next_mode < (uint8_t)openhaldex_mode_t_MAX)
  {
    state.mode = (openhaldex_mode_t)next_mode; // If the next mode is valid, change the current mode to it
  }
  // On overflow, start over from MODE_STOCK.
  else
  {
    if (!isStandalone)
    {
      state.mode = MODE_STOCK;
    }
    else
    {
      state.mode = MODE_FWD;
    }
  }
  lastMode = state.mode;
}

void modeChangeExtLong(void)
{
#if enableDebug
  DEBUG("External mode button long pressed!");
#endif

  extButtonForceModeFlag = true;
}

void modeChangeExtLongOff(void)
{
#if enableDebug
  DEBUG("External mode button long pressed!");
#endif

  extButtonForceModeFlag = false;
}

void setupButtons()
{
  // setup buttons / inputs
  btnMode.setMenuCount(0);
  btnMode.setMenuLevel(0);           // Use the functions bound to the first menu associated with the button
  btnMode.setMode(Mode_Synchronous); // can be caught in the main loop - no rush

  btnMode_ext.setMenuCount(0);
  btnMode_ext.setMenuLevel(0);           // Use the functions bound to the first menu associated with the button
  btnMode_ext.setMode(Mode_Synchronous); // can be caught in the main loop - no rush

  // InterruptButton::m_RTOSservicerStackDepth = 4096; // Use larger values for more memory intensive functions if using Asynchronous mode.
  btnMode.bind(Event_KeyPress, 0, &modeChange);            // short press: cycle mode
  btnMode.bind(Event_LongKeyPress, 0, &resetWifi); // long press: clear WiFi password + restore default SSID, restart AP open

  btnMode_ext.bind(Event_KeyPress, 0, &modeChangeExt);         // short press: cycle mode (external button)
  btnMode_ext.bind(Event_LongKeyPress, 0, &modeChangeExtLong); // long press: force mode (external button)
  // btnMode.bind(Event_KeyDown, 0, &modeChangeExtLongOff);
}

void updateTriggers(void *arg)
{
  while (1)
  {
    const uint32_t now = millis();
    hasCANChassis = (lastCANChassisTick > 0) && ((now - (uint32_t)lastCANChassisTick) <= canHealthTimeoutMs); // 1000ms timeout for CAN health - if we haven't received a message in 1000ms, consider the CAN connection unhealthy
    hasCANHaldex = (lastCANHaldexTick > 0) && ((now - (uint32_t)lastCANHaldexTick) <= canHealthTimeoutMs);    // 1000ms timeout for CAN health - if we haven't received a message in 1000ms, consider the CAN connection unhealthy

    // Low-power WiFi management
    // Standalone: Haldex bus fps. OEM: chassis bus fps.
    //
    // LP_WATCHING : no clients + canActive false for watchMs -> shut WiFi+LED -> LP_SLEEPING
    // LP_SLEEPING : canActive -> restore WiFi -> LP_WATCHING
    //               CPU auto-sleeps via esp_pm_configure in main.cpp when FreeRTOS is idle.
    //
    // Aggressive add-on (canSleepAggressive=true): while LP_SLEEPING we also
    // put the CAN transceivers in standby and suspend the background
    // periodic tasks. A GPIO ISR on CAN_RX is the only wake path - the moment
    // bus activity returns, the falling edge on CAN_RX fires the ISR and brings
    // everything back.
    {
      // Compute frames-per-second once per second from the running frame counters.
      if ((now - lpLastFpsCheck) >= 1000UL)
      {
        lpChassisFps = lpChassisFrameCount - lpLastChassisSnap;
        lpHaldexFps = lpHaldexFrameCount - lpLastHaldexSnap;
        lpLastChassisSnap = lpChassisFrameCount;
        lpLastHaldexSnap = lpHaldexFrameCount;
        lpLastFpsCheck = now;
        if (lpState == LP_SLEEPING)
        {
          DEBUG("LP_SLEEPING: chassis=%lu fps  haldex=%lu fps  threshold=%u  standalone=%d  clients=%d",
                (unsigned long)lpChassisFps, (unsigned long)lpHaldexFps,
                (unsigned)lpWakeThresholdFps, (int)isStandalone,
                (int)WiFi.softAPgetStationNum());
        }
        else
        {
          DEBUG("LP_WATCHING: chassis=%lu fps  haldex=%lu fps  threshold=%u  standalone=%d  clients=%d  noClientSince=%lu",
                (unsigned long)lpChassisFps, (unsigned long)lpHaldexFps,
                (unsigned)lpWakeThresholdFps, (int)isStandalone,
                (int)WiFi.softAPgetStationNum(),
                lpNoClientsSince ? (unsigned long)(now - lpNoClientsSince) : 0UL);
        }
      }

      const bool noClients = (WiFi.softAPgetStationNum() == 0) && (WiFi.getMode() != WIFI_OFF);
      // Standalone: Haldex fps >= fixed 50 fps threshold.
      // OEM: chassis fps >= lpWakeThresholdFps (UI slider, default 50).
      const bool canActive = isStandalone ? (lpHaldexFps >= 50U)
                                          : (lpChassisFps >= lpWakeThresholdFps);

      switch (lpState)
      {
      case LP_WATCHING:
        if (noClients && !canActive)
        {
          if (lpNoClientsSince == 0)
            lpNoClientsSince = now;
#if debugCANSleep
          if ((now - lpNoClientsSince) >= 2000UL)
#else
          if ((now - lpNoClientsSince) >= lowPowerWatchMs)
#endif
          {
            lowPowerMode = true;
            lpState = LP_SLEEPING;
            DEBUG("Low power: no clients + CAN idle (%lu fps) - shutting down WiFi+LED%s",
                  (unsigned long)(isStandalone ? lpHaldexFps : lpChassisFps),
                  canSleepAggressive ? " + transceivers standby (ISR wake)" : "");
            strip.setLedColorData(led_channel, 0, 0, 0);
            strip.show();
            // Aggressive: shutdown transceivers immediately and suspend background
            // periodic tasks. Wake is purely ISR-driven on the CAN_RX pins.
            if (canSleepAggressive)
            {
              lpSetTransceiverStandby(true);
              lpSuspendBackgroundTasks();
            }
          }
        }
        else
        {
          lpNoClientsSince = 0;
        }
        break;

      case LP_SLEEPING:
        // Aggressive: ISR-driven wake. The CAN_RX GPIO ISR sets
        // canWakeRequest on the first falling edge from the (standby)
        // transceiver; we then bring the transceivers live so the fps path
        // can confirm real traffic and exit to LP_WATCHING.
        if (canSleepAggressive)
        {
          if (lpTransceiversStandby && canWakeRequest)
          {
            lpSetTransceiverStandby(false);
            DEBUG("LP aggressive: RX edge detected - transceivers live");
          }
        }
        else if (lpTransceiversStandby)
        {
          // Aggressive was disabled while sleeping - restore normal mode.
          lpSetTransceiverStandby(false);
          lpResumeBackgroundTasks();
        }

        if (canActive)
        {
          // Ensure transceivers are back to normal before we re-enable WiFi/IO.
          lpSetTransceiverStandby(false);
          lpResumeBackgroundTasks();
          lowPowerMode = false;
          lpNoClientsSince = 0;
          lpState = LP_WATCHING;
          DEBUG("Low power: CAN active (%lu fps, threshold %u) - restoring WiFi",
                (unsigned long)(isStandalone ? lpHaldexFps : lpChassisFps),
                (unsigned)lpWakeThresholdFps);
          rebootWiFi = true;
        }
        // CPU auto-sleeps via esp_pm_configure(light_sleep_enable=true) in main.cpp.
        break;
      }
    }

    // Analyzer mode: keep buttons + CAN recovery, but skip brake/handbrake IO outputs.
    if (analyzerMode)
    {
      canBusRecovery();

      vTaskDelay(updateTriggersRefresh / portTICK_PERIOD_MS);
      continue;
    }

    btnMode.processSyncEvents();     // Only required if using sync events
    btnMode_ext.processSyncEvents(); // Only required if using sync events

    handbrakeSignalActive = digitalRead(gpio_hb_in);
    brakeSignalActive = digitalRead(gpio_brake_in);

    if (followHandbrake)
    {
      bool hbOut = invertHandbrake ? !handbrakeSignalActive : handbrakeSignalActive;
      digitalWrite(gpio_hb_out, hbOut);
      handbrakeActive = hbOut;
    }
    else
    {
      digitalWrite(gpio_hb_out, LOW);
      handbrakeActive = false;
    }

    if (followBrake)
    {
      bool brakeOut = invertBrake ? !brakeSignalActive : brakeSignalActive;
      digitalWrite(gpio_brake_out, brakeOut);
      brakeActive = brakeOut;
    }
    else
    {
      digitalWrite(gpio_brake_out, LOW);
      brakeActive = false;
    }

    // Poll both controllers and drive the bus-off recovery state machine.
    canBusRecovery();

    if (!lowPowerMode)
    {
      switch (state.mode)
      {
      case 0:
        strip.setLedColorData(led_channel, ledBrightness, 0, 0); // red
        break;
      case 1:
        strip.setLedColorData(led_channel, 0, ledBrightness, 0); // green
        break;
      case 2:
        strip.setLedColorData(led_channel, 0, 0, ledBrightness); // blue
        break;
      case 3:
      {
        // neon pink
        uint8_t r = (uint8_t)((255 * ledBrightness) / 255); // 255
        uint8_t g = (uint8_t)((16 * ledBrightness) / 255);
        uint8_t b = (uint8_t)((240 * ledBrightness) / 255);
        strip.setLedColorData(led_channel, r, g, b); // scaled with brightness
        break;
      }
      case 4:
        strip.setLedColorData(led_channel, 0, ledBrightness, ledBrightness); // cyan
        break;
      case 5:
        strip.setLedColorData(led_channel, ledBrightness, ledBrightness, ledBrightness); // white
        break;
      }
      strip.show(); // Update the LED strip to reflect the new colour settings
    }

    // Aggressive sleep: block on a task notification with a long timeout
    // instead of polling every 500ms. The CAN_RX wake ISRs give a
    // notification on any bus activity, so wake latency stays ~ISR + 1
    // tick. The timeout caps how long we sit idle if no edge ever fires
    // (safety-net probe + sanity check).
    if (canSleepAggressive && lowPowerMode)
    {
      // 2s timeout aligns with the safety-net probe cadence.
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000));
    }
    else
    {
      vTaskDelay(updateTriggersRefresh / portTICK_PERIOD_MS);
    }
  }
}
