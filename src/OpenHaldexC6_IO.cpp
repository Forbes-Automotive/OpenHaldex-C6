#include <OpenHaldexC6_IO.h>
#include <OpenHaldexC6_can.h>
#include <OpenHaldexC6_WiFi.h>

void setupIO()
{
  // using the TCAN1044 for CAN control
  pinMode(CAN0_RS, OUTPUT);   // gpio for controlling can_0 state - enabled or disabled
  pinMode(CAN1_RS, OUTPUT);   // gpio for controlling can_0 slope - enabled or disabled
  digitalWrite(CAN0_RS, LOW); // set chip enable
  digitalWrite(CAN1_RS, LOW); // set chip enable

  pinMode(gpio_hb_in, INPUT);      // gpio for handbrake in signal
  pinMode(gpio_brake_in, INPUT);   // gpio for brake in signal
  pinMode(gpio_hb_out, OUTPUT);    // gpio for handbrake out signal
  pinMode(gpio_brake_out, OUTPUT); // gpio for brake out signal

  strip.begin();                       // begin RGB LED onboard
  strip.setBrightness(led_brightness); // default brightness
}

void modeChange(void)
{
#if enableDebug
  DEBUG("Internal mode button pressed!");
#endif
  if (disableOnboardButton) return; // onboard button disabled, ignore press

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
  if (disableExternalButton) return; // external button disabled, ignore press

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
  btnMode.bind(Event_KeyPress, 0, &modeChange); // apply to 'mode change'
  btnMode.bind(Event_LongKeyPress, 0, &disconnectWifi);

  btnMode_ext.bind(Event_KeyPress, 0, &modeChangeExt);     // apply to 'mode change ext'
  btnMode.bind(Event_LongKeyPress, 0, &modeChangeExtLong); // apply to 'mode change ext'
  // btnMode.bind(Event_KeyDown, 0, &modeChangeExtLongOff);
}

void updateTriggers(void *arg)
{
  while (1)
  {
    const uint32_t now = millis();
    hasCANChassis = (lastCANChassisTick > 0) && ((now - (uint32_t)lastCANChassisTick) <= canHealthTimeoutMs); // 1000ms timeout for CAN health - if we haven't received a message in 1000ms, consider the CAN connection unhealthy
    hasCANHaldex = (lastCANHaldexTick > 0) && ((now - (uint32_t)lastCANHaldexTick) <= canHealthTimeoutMs);    // 1000ms timeout for CAN health - if we haven't received a message in 1000ms, consider the CAN connection unhealthy

    handbrakeSignalActive = digitalRead(gpio_hb_in);
    brakeSignalActive = digitalRead(gpio_brake_in);

    // Analyzer mode: keep buttons + CAN recovery, but skip brake/handbrake IO outputs.
    if (analyzerMode)
    {
      if (isBusFailure)
      {
        twai_initiate_recovery_v2(twai_bus_0);
        twai_initiate_recovery_v2(twai_bus_1);
      }

      vTaskDelay(updateTriggersRefresh / portTICK_PERIOD_MS);
      continue;
    }

    btnMode.processSyncEvents();     // Only required if using sync events
    btnMode_ext.processSyncEvents(); // Only required if using sync events

    if (followHandbrake)
    {
      if (invertHandbrake)
      {
        digitalWrite(gpio_hb_out, !digitalRead(gpio_hb_in));
        handbrakeActive = false;
      }
      else
      {
        digitalWrite(gpio_hb_out, digitalRead(gpio_hb_in));
        handbrakeActive = true;
      }
    }
    else
    {
      if (invertHandbrake)
      {
        digitalWrite(gpio_hb_out, LOW);
        handbrakeActive = false;
      }
      else
      {
        digitalWrite(gpio_hb_out, HIGH);
        handbrakeActive = true;
      }
    }

    if (followBrake)
    {
      if (invertBrake)
      {
        digitalWrite(gpio_brake_out, !digitalRead(gpio_brake_in));
        brakeActive = false;
      }
      else
      {
        digitalWrite(gpio_brake_out, digitalRead(gpio_brake_in));
        brakeActive = true;
      }
    }
    else
    {
      if (invertBrake)
      {
        digitalWrite(gpio_brake_out, LOW);
        brakeActive = false;
      }
      else
      {
        digitalWrite(gpio_brake_out, HIGH);
        brakeActive = true;
      }
    }

    if (isBusFailure)
    {
      twai_initiate_recovery_v2(twai_bus_0);
      twai_initiate_recovery_v2(twai_bus_1);
    }

    switch (state.mode)
    {
    case 0:
      strip.setLedColorData(led_channel, led_brightness, 0, 0); // red
      break;
    case 1:
      strip.setLedColorData(led_channel, 0, led_brightness, 0); // green
      break;
    case 2:
      strip.setLedColorData(led_channel, 0, 0, led_brightness); // blue
      break;
    case 3:
      strip.setLedColorData(led_channel, 255, 16, 240); // neon pink
      break;
    case 4:
      strip.setLedColorData(led_channel, 0, led_brightness, led_brightness); // cyan
      break;
    case 5:
      strip.setLedColorData(led_channel, led_brightness, led_brightness, led_brightness); // white
      break;
    }
    strip.show(); // Update the LED strip to reflect the new colour settings

    vTaskDelay(updateTriggersRefresh / portTICK_PERIOD_MS);
  }
}
