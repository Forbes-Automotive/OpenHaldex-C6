#include <OpenHaldexC6_IO.h>
#include <OpenHaldexC6_can.h>
#include <OpenHaldexC6_WiFi.h>
#include <OpenHaldexC6_OTA.h> // otaWebClientActive(): bridge-mode browsers hold WiFi up
#include <OpenHaldexC6_Analyzer.h> // setAnalyzerMode(): enabling the controller leaves analyzer mode
#include "esp_pm.h"                // release/take the no-light-sleep lock around deliberate sleep

// Light sleep is only allowed while the low-power state machine has
// deliberately put the module to sleep. No-op when CAN sleep is off (no lock).
static void pmAllowLightSleep(bool allow)
{
  if (pmNoLightSleepLock == nullptr)
    return;
  esp_pm_lock_handle_t lk = (esp_pm_lock_handle_t)pmNoLightSleepLock;
  if (allow)
    esp_pm_lock_release(lk);
  else
    esp_pm_lock_acquire(lk);
}

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

static uint32_t sleepCalSum          = 0; // chassis fps summed over the current window
static uint16_t sleepCalSecs         = 0; // seconds in the current window

// Bench mode support (PR #39): latches true the first time real CAN traffic is
// seen on either bus this power cycle. Once set, benchMode stops suppressing
// sleep - the unit is demonstrably harnessed - so a forgotten toggle can't
// weaken the parked-car battery protection. Not persisted; resets on boot.
static bool everSawCANThisSession = false;

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
  { &handle_standaloneFrames,      true  },
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

// One second of the sleep auto-setup. Counts only while nobody is connected,
// the car is parked with the ignition off (KL15, where the car sends it) and
// CAN has been seen this session (not on the bench). Anything else restarts
// the 15 min window.
static void sleepCalTick(bool noClients)
{
  const bool parked = received_vehicle_speed == 0 && (received_vehicle_rpm == 0 || !hasCANChassis) && !ignitionOn();
  if (!noClients || !parked || !everSawCANThisSession)
  {
    sleepCalSum = 0;
    sleepCalSecs = 0;
    return;
  }

  if (sleepCalSecs == 0)
  {
    canSleepEnabled = true;
    canSleepAggressive = true;
    DEBUG("Sleep auto-setup: measuring parked bus for %u s", (unsigned)SLEEP_CAL_WINDOW_S);
  }
  sleepCalSum += lpChassisFps;
  sleepCalSecs++;
  if (sleepCalSecs < SLEEP_CAL_WINDOW_S)
    return;

  sleepCalAvgFps = (uint16_t)(sleepCalSum / sleepCalSecs);
  uint32_t t = sleepCalAvgFps + SLEEP_CAL_MARGIN_FPS;
  t = ((t + 9) / 10) * 10; // slider step
  lpWakeThresholdFps = (uint16_t)constrain(t, 100UL, 2000UL);
  sleepCalState = SLEEP_CAL_DONE;
  DEBUG("Sleep auto-setup: parked avg %u fps -> wake threshold %u fps",
        (unsigned)sleepCalAvgFps, (unsigned)lpWakeThresholdFps);
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

bool requestMode(uint8_t mode)
{
  if (mode >= (uint8_t)openhaldex_mode_t_MAX || disableController)
  {
    return false;
  }

  if (isStandalone && mode == MODE_STOCK)
  {
    state.mode = (openhaldex_mode_t)lastMode; // Stock is passthrough-only: standalone keeps the last driving mode
  }
  else
  {
    state.mode = (openhaldex_mode_t)mode;
  }
  lastMode = state.mode;
  return true;
}

void setControllerDisabled(bool disabled)
{
  disableController = disabled;
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

    // Fail safe on signal loss: parseCAN only ever overwrites these on frame
    // arrival, so on bus loss they latch their last value forever. In
    // standalone that means synthesizing lock from stale speed/throttle (and a
    // stale ESP/hazard force flag holding a force mode on) indefinitely. Zero
    // them once the health timeout declares the bus dead so the speed/throttle
    // gates and force triggers see "no input" instead of the last frame.
    // lastCANChassisTick > 0 keeps the boot state (never seen a frame) as-is.
    if (!hasCANChassis && lastCANChassisTick > 0)
    {
      received_vehicle_speed = 0;
      received_pedal_value = 0;
      tcForceModeFlag = false;
      hazardForceModeFlag = false;
      isABSValid = false;
    }
    if (!hasCANHaldex && lastCANHaldexTick > 0)
    {
      received_haldex_engagement = 0; // stale engagement would freeze telemetry and poison a running learn
    }

    if (hasCANChassis || hasCANHaldex)
      everSawCANThisSession = true; // real bus seen: bench mode (if on) stops holding WiFi up from here on

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
      bool fpsUpdated = false;
      if ((now - lpLastFpsCheck) >= 1000UL)
      {
        fpsUpdated = true;
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
          DEBUG("LP_WATCHING: chassis=%lu fps  haldex=%lu fps  threshold=%u  standalone=%d  clients=%d  noClientSince=%lu  kl15=%d  sleepCal=%u %us",
                (unsigned long)lpChassisFps, (unsigned long)lpHaldexFps,
                (unsigned)lpWakeThresholdFps, (int)isStandalone,
                (int)WiFi.softAPgetStationNum(),
                lpNoClientsSince ? (unsigned long)(now - lpNoClientsSince) : 0UL,
                (int)ignitionOn(), (unsigned)sleepCalState, (unsigned)sleepCalSecs);
        }
      }

      // "No clients" = nobody joined to our AP AND no browser polling the UI.
      // The second half covers a phone/laptop reaching us through the home
      // router (bridge mode), which softAPgetStationNum() can't see - without
      // it the controller could switch WiFi off in the middle of an OTA update
      // done over the bridge on the bench. An upload in progress always holds.
      const bool noClients = (WiFi.softAPgetStationNum() == 0) && (WiFi.getMode() != WIFI_OFF) && !otaWebClientActive();
      // Standalone: Haldex fps >= fixed 50 fps threshold.
      // OEM: chassis fps >= lpWakeThresholdFps (UI slider, default 50).
      const bool canActive = isStandalone ? (lpHaldexFps >= 50U)
                                          : (lpChassisFps >= lpWakeThresholdFps);
      // Bench mode: with no real CAN seen this session, behave as if the bus
      // were active so LP_WATCHING never sleeps. The instant traffic appears
      // (everSawCANThisSession latches) this stops applying.
      const bool benchHold = benchMode && !everSawCANThisSession;

      // Sleep auto-setup: hold sleep off until the parked bus has been measured once.
      const bool sleepCalHold = (sleepCalState == SLEEP_CAL_ARMED) && !isStandalone;
      if (sleepCalHold && fpsUpdated)
        sleepCalTick(noClients);

      switch (lpState)
      {
      case LP_WATCHING:
        // canSleepEnabled is the UI toggle: it used to gate only the CPU
        // frequency scaling in main.cpp, never this WiFi shutdown, so turning
        // it off did nothing visible (PR #39 fix).
        if (canSleepEnabled && noClients && !canActive && !benchHold && !sleepCalHold)
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
            pmAllowLightSleep(true);
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
          pmAllowLightSleep(false);
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
