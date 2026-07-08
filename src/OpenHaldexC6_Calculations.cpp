#include <OpenHaldexC6_Calculations.h>
#include <OpenHaldexC6_tasks.h>

// Only executed when in MODE_FWD/MODE_5050/MODE_Expert
static inline bool lock_enabled()
{
  bool throttle_ok = false;
  bool speed_ok = false;

  if (state.mode != MODE_EXPERT)
  {
    throttle_ok = (state.pedal_threshold == 0) || (int(received_pedal_value) >= state.pedal_threshold);
    speed_ok = (disengageUnderSpeed == 0) || (received_vehicle_speed <= disengageUnderSpeed) || (received_vehicle_speed >= disengageAboveSpeed);
    return throttle_ok && speed_ok;
  }

  if (state.mode == MODE_EXPERT)
  {
    // todo - add in override functions?
    throttle_ok = true;
    speed_ok = true;
    return throttle_ok && speed_ok;
  }

  return false;
}

static float get_expert_lock_target()
{
  // 2D throttle/speed map - interpolation between arrays.  Result is an (int) of float of requested lock percentage
  float throttle = received_pedal_value;
  throttle = constrain(throttle, 0, 100); // fix throttle from 0-100

  float speed = received_vehicle_speed; // fix speed from 0-300 kmh (should be plenty)
  speed = constrain(speed, 0, 300);

  uint8_t t0 = 0; // index for throttle array
  uint8_t t1 = 0; // index for throttle array
  float t_ratio = 0;

  if (throttle >= throttleArray[throttleArrayCount - 1])
  {
    t0 = throttleArrayCount - 1; // if throttle is above top value, use top 2 values for interpolation (will just return top value)
    t1 = t0;                     // if throttle is above top value, use top 2 values for interpolation (will just return top value)
  }
  else
  {
    for (uint8_t i = 0; i < throttleArrayCount - 1; i++) // find throttle indexes for interpolation
    {
      if (throttle <= throttleArray[i + 1])
      {
        t0 = i;
        t1 = i + 1;
        float denom = (float)throttleArray[t1] - (float)throttleArray[t0];    // calculate ratio for interpolation - handle divide by zero just in case
        t_ratio = (denom > 0) ? ((throttle - throttleArray[t0]) / denom) : 0; // if denom is zero, just use t0 value (ratio of 0), otherwise calculate ratio between t0 and t1
        break;
      }
    }
  }

  uint8_t s0 = 0; // index for speed array
  uint8_t s1 = 0; // index for speed array
  float s_ratio = 0;

  if (speed >= speedArray[speedArrayCount - 1])
  {
    s0 = speedArrayCount - 1; // if speed is above top value, use top 2 values for interpolation (will just return top value)
    s1 = s0;                  // if speed is above top value, use top 2 values for interpolation (will just return top value)
  }
  else
  {
    for (uint8_t i = 0; i < speedArrayCount - 1; i++)
    {
      if (speed <= speedArray[i + 1])
      {
        s0 = i;
        s1 = i + 1;
        float denom = (float)speedArray[s1] - (float)speedArray[s0];    // calculate ratio for interpolation - handle divide by zero just in case
        s_ratio = (denom > 0) ? ((speed - speedArray[s0]) / denom) : 0; // if denom is zero, just use s0 value (ratio of 0), otherwise calculate ratio between s0 and s1
        break;
      }
    }
  }

  float v00 = lockArray[t0][s0];
  float v01 = lockArray[t0][s1];
  float v10 = lockArray[t1][s0];
  float v11 = lockArray[t1][s1];

  float v0 = v00 + ((v01 - v00) * s_ratio);
  float v1 = v10 + ((v11 - v10) * s_ratio);
  float v = v0 + ((v1 - v0) * t_ratio);

  v = constrain(v, 0, 100); // ensure lock target is between 0 and 100
  return int(v);            // return lock target as an integer percentage (0-100)
}

float get_lock_target_adjustment()
{
  // const MODE_NAMES = ['Stock', 'FWD', '50:50', '60:40', '75:25', 'Expert']; // mode names as Strings
  if (extBtnForceMode || tcForceMode || hazardForceMode) // if any force-mode trigger is enabled
  {
    // Determine which trigger is active and pick its configured mode value.
    // Priority is user-configurable via forceModesPriority (0-5 covering all 6 orderings of TC/Hazards/External).
    struct TriggerCheck
    {
      bool enabled;
      bool flag;
      uint8_t value;
    };
    const TriggerCheck triggers[3] = {
        {tcForceMode, tcForceModeFlag, tcForceModeValue},                // index 0 = TC
        {hazardForceMode, hazardForceModeFlag, hazardForceModeValue},    // index 1 = Hazard
        {extBtnForceMode, extButtonForceModeFlag, extBtnForceModeValue}, // index 2 = Ext
    };

    // Each row is [first, second, third] trigger index in priority order. 0=TC, 1=Hazard, 2=Ext. 6 rows cover all 6 options.
    static const uint8_t priorityOrders[6][3] = {
        {1, 0, 2}, // 0: Hazard > TC > Ext (default)
        {0, 1, 2}, // 1: TC > Hazard > Ext
        {1, 2, 0}, // 2: Hazard > Ext > TC
        {0, 2, 1}, // 3: TC > Ext > Hazard
        {2, 0, 1}, // 4: Ext > TC > Hazard
        {2, 1, 0}, // 5: Ext > Hazard > TC
    };

    uint8_t fmv = 0; // force mode value (0=Stock, 1=FWD, 2=50:50, 3=60:40, 4=75:25, 5=Expert)
    bool anyActive = false;
    const uint8_t pri = (forceModesPriority < 6) ? forceModesPriority : 0;

    for (uint8_t i = 0; i < 3; i++)
    {
      const uint8_t idx = priorityOrders[pri][i];
      if (triggers[idx].enabled && triggers[idx].flag)
      {
        fmv = triggers[idx].value;
        anyActive = true;
        break;
      }
    }

    if (anyActive)
    {
      switch (fmv)
      {
      case 0:
        return received_haldex_engagement; // stock -> passthrough (mirror actual engagement, don't force open)
      case 1:
        return 0; // FWD
      case 2:
        return 100; // 50:50
      case 3:
        return 40; // 60:40
      case 4:
        return 30; // 75:25
      case 5:
        return get_expert_lock_target(); // Expert
      default:
        return 0; // error - zero lock
      }
    }
  }

  // getting here means no external influences - handle each mode and calculate lock
  switch (state.mode)
  {
  case MODE_STOCK:
    return received_haldex_engagement; // stock -> passthrough (mirror actual engagement, never force FWD)

  case MODE_FWD:
    return 0; // zero lock

  case MODE_5050:
    if (lock_enabled())
    {
      return 100; // 100% lock
    }
    return 0; // lock not enabled, zero lock

  case MODE_6040:
    if (lock_enabled())
    {
      return 40; // 40% lock
    }
    return 0; // lock not enabled, zero lock

  case MODE_7525:
    if (lock_enabled())
    {
      return 30; // 30% lock
    }
    return 0; // lock not enabled, zero lock

  case MODE_EXPERT:
    if (lock_enabled())
    {
      return get_expert_lock_target();
    }
    return 0; // lock not enabled, zero lock

  default:
    return 0; // error - zero lock
  }
}

uint8_t get_lock_target_adjusted_value(uint8_t value, bool invert)
{
  // during learn, use the current learn correction factor
  if (haldexLearnActive)
  {
    uint8_t corrected_value = (uint16_t)value * haldexLearnCF / 100;
    return (invert ? (0xFE - corrected_value) : corrected_value);
  }

  // handle 5050 mode
  if (lock_target == 100)
  {
    if (lock_enabled())
    {
      return (invert ? (0xFE - value) : value); // if lock enabled, return full value (or inverted), otherwise return 0 (or inverted)
    }
    return (invert ? 0xFE : 0x00);
  }

  // handle FWD mode
  if (lock_target == 0)
  {
    return (invert ? 0xFE : 0x00);
  }

  uint8_t correction_factor = 0; // calculate correction factor based on learn table if valid, otherwise use a default formula to determine correction factor (which is not very accurate, but better than nothing)
  if (haldexLearnTableValid)
  {
    // find the smallest correction factor where the learned engagement meets or exceeds lock_target
    for (uint8_t i = 0; i <= 100; i++)
    {
      if (haldexLearnTable[i] >= (uint8_t)lock_target)
      {
        correction_factor = i;
        break;
      }
    }
  }
  else if (haldexGeneration == 41)
  {
    // Gen41 (Vauxhall/Opel/Buick FDCM) defaults — observed mapping CF→engagement is
    // steeper than the VAG Haldex curve.
    // Linear fit: engagement = 2.2 * CF - 24  →  CF = (target + 24) / 2.2
    correction_factor = (uint8_t)constrain(((float)lock_target + 24.0f) / 2.2f, 0, 100);
  }
  else
  {
    // VAG Haldex default — linear fit: engagement = 2 * CF - 20  →  CF = (target + 20) / 2
    correction_factor = (uint8_t)constrain(((float)lock_target / 2) + 20, 0, 100);
  }

  uint8_t corrected_value = (uint16_t)value * correction_factor / 100;
  if (lock_enabled())
  {
    return (invert ? (0xFE - corrected_value) : corrected_value); // if lock enabled, return corrected value (or inverted), otherwise return 0 (or inverted)
  }
  return (invert ? 0xFE : 0x00); // if lock not enabled, return 0 (or inverted)
}

void startHaldexLearn()
{
  if (haldexLearnActive)
  {
    return; // already running
  }

  memset(haldexLearnTable, 0, sizeof(haldexLearnTable));
  haldexLearnCancel = false;
  haldexLearnStep = 0;
  haldexLearnCF = 0;
  haldexLearnActive = true;

  xTaskCreate(haldexLearnTask, "haldexLearn", 4096, nullptr, 1, nullptr);
}

void getLockData(twai_message_t &rx_message_chs)
{
  // Calculate raw lock target then (optionally) apply rate-limited release decay.
  // When lockReleaseEnabled is false, all transitions to new lock % are instantaneous.
  // When enabled, rising transitions are always instantaneous; falling
  // transitions are limited to `lockReleaseRatePerSec` %/s so the clutch
  // releases gradually rather than snapping open.
  static float smoothed_lock_target = 0.0f;
  static uint32_t last_lock_ms = 0;

  const float raw_target = get_lock_target_adjustment(); // calculate raw lock target based on mode, overrides, and learn table

  if (!lockReleaseEnabled)
  {
    smoothed_lock_target = raw_target; // instant: bypass rate limit
    last_lock_ms = millis();
  }
  else
  {
    const uint32_t now_ms = millis();
    const float dt_s = (last_lock_ms == 0) ? 0.0f : (float)(now_ms - last_lock_ms) / 1000.0f;
    last_lock_ms = now_ms;

    if (raw_target >= smoothed_lock_target)
    {
      smoothed_lock_target = raw_target; // rise: immediate
    }
    else
    {
      const float max_drop = lockReleaseRatePerSec * dt_s;
      smoothed_lock_target = (raw_target > smoothed_lock_target - max_drop)
                                 ? raw_target
                                 : smoothed_lock_target - max_drop;
    }
  }
  lock_target = smoothed_lock_target;

  // begin frame parsing / editting
  // edit the frames if configured as Gen1...
  if (haldexGeneration == 1)
  {
    switch (rx_message_chs.identifier)
    {
    case MOTOR1_ID:
      rx_message_chs.data[0] = 0x00;
      rx_message_chs.data[1] = get_lock_target_adjusted_value(0xFE, false);
      rx_message_chs.data[2] = 0x21;
      rx_message_chs.data[3] = get_lock_target_adjusted_value(0x4E, false);
      rx_message_chs.data[4] = get_lock_target_adjusted_value(0xFE, false);
      rx_message_chs.data[5] = get_lock_target_adjusted_value(0xFE, false);
      appliedTorque = rx_message_chs.data[6];

      switch (state.mode)
      {
      case MODE_FWD:
        appliedTorque = get_lock_target_adjusted_value(0xFE, true);
        break;
      case MODE_5050:
        appliedTorque = get_lock_target_adjusted_value(0x16, false);
        break;
      case MODE_6040:
        appliedTorque = get_lock_target_adjusted_value(0x22, false);
        break;
      case MODE_7525:
        appliedTorque = get_lock_target_adjusted_value(0x50, false);
        break;
      default:
        break;
      }

      rx_message_chs.data[6] = appliedTorque;
      rx_message_chs.data[7] = 0x00;
      break;
    case MOTOR3_ID:
      rx_message_chs.data[2] = get_lock_target_adjusted_value(0xFE, false);
      rx_message_chs.data[7] = get_lock_target_adjusted_value(0xFE, false);
      break;
    case BRAKES1_ID:
      rx_message_chs.data[1] = get_lock_target_adjusted_value(0x00, false);
      rx_message_chs.data[2] = 0x00;
      rx_message_chs.data[3] = get_lock_target_adjusted_value(0x0A, false);
      break;
    case BRAKES3_ID:
      rx_message_chs.data[0] = get_lock_target_adjusted_value(0xFE, false);
      rx_message_chs.data[1] = 0x0A;
      rx_message_chs.data[2] = get_lock_target_adjusted_value(0xFE, false);
      rx_message_chs.data[3] = 0x0A;
      rx_message_chs.data[4] = 0x00;
      rx_message_chs.data[5] = 0x0A;
      rx_message_chs.data[6] = 0x00;
      rx_message_chs.data[7] = 0x0A;
      break;
    }
  }

  // edit the frames if configured as Gen2...
  if (haldexGeneration == 2)
  {
    switch (rx_message_chs.identifier)
    {
    case MOTOR1_ID:
      rx_message_chs.data[1] = get_lock_target_adjusted_value(0xFE, false);
      rx_message_chs.data[2] = 0x21;
      rx_message_chs.data[3] = get_lock_target_adjusted_value(0x4E, false);
      rx_message_chs.data[6] = get_lock_target_adjusted_value(0xFE, false); // 0x20 in standalone - same as gen1?
      break;
    case MOTOR3_ID:
      rx_message_chs.data[2] = get_lock_target_adjusted_value(0xFE, false);
      rx_message_chs.data[7] = get_lock_target_adjusted_value(0x01, false);
      break;
    case BRAKES1_ID:
      rx_message_chs.data[0] = get_lock_target_adjusted_value(0x80, false);
      rx_message_chs.data[1] = get_lock_target_adjusted_value(0x41, false);
      rx_message_chs.data[2] = get_lock_target_adjusted_value(0xFE, false);
      rx_message_chs.data[3] = 0x0A;
      break;
    case BRAKES2_ID:
      rx_message_chs.data[4] = get_lock_target_adjusted_value(0x7F, false);
      rx_message_chs.data[5] = get_lock_target_adjusted_value(0xFE, false);
      break;
    case BRAKES3_ID:
      rx_message_chs.data[0] = get_lock_target_adjusted_value(0xFE, false);
      rx_message_chs.data[1] = 0x0A;
      rx_message_chs.data[2] = get_lock_target_adjusted_value(0xFE, false);
      rx_message_chs.data[3] = 0x0A;
      rx_message_chs.data[4] = 0x00;
      rx_message_chs.data[5] = 0x0A;
      rx_message_chs.data[6] = 0x00;
      rx_message_chs.data[7] = 0x0A;
      break;
    }
  }

  // edit the frames if configured as Gen4...
  if (haldexGeneration == 4)
  {
    switch (rx_message_chs.identifier)
    {
    case mLW_1:
      rx_message_chs.data[0] = lws_2[mLW_1_counter][0];
      rx_message_chs.data[1] = lws_2[mLW_1_counter][1];
      rx_message_chs.data[2] = lws_2[mLW_1_counter][2];
      rx_message_chs.data[3] = lws_2[mLW_1_counter][3];
      rx_message_chs.data[4] = lws_2[mLW_1_counter][4];
      rx_message_chs.data[5] = lws_2[mLW_1_counter][5];
      rx_message_chs.data[6] = lws_2[mLW_1_counter][6];
      rx_message_chs.data[7] = lws_2[mLW_1_counter][7];
      mLW_1_counter++;
      if (mLW_1_counter > 15)
      {
        mLW_1_counter = 0;
      }
      break;
    case MOTOR1_ID:
      rx_message_chs.data[1] = get_lock_target_adjusted_value(0xFE, false);
      rx_message_chs.data[2] = get_lock_target_adjusted_value(0x20, false);
      rx_message_chs.data[3] = get_lock_target_adjusted_value(0x4E, false);
      rx_message_chs.data[4] = get_lock_target_adjusted_value(0xFE, false);
      rx_message_chs.data[5] = get_lock_target_adjusted_value(0xFE, false);
      rx_message_chs.data[6] = get_lock_target_adjusted_value(0x16, false);
      rx_message_chs.data[7] = get_lock_target_adjusted_value(0xFE, false);
      break;
    case BRAKES1_ID:
      rx_message_chs.data[0] = 0x20;
      rx_message_chs.data[1] = 0x40;
      rx_message_chs.data[4] = get_lock_target_adjusted_value(0xFE, false);
      rx_message_chs.data[5] = get_lock_target_adjusted_value(0xFE, false);
      break;
    case BRAKES2_ID:
      rx_message_chs.data[4] = get_lock_target_adjusted_value(0x7F, false);
      break;
    case BRAKES3_ID:
      rx_message_chs.data[0] = get_lock_target_adjusted_value(0xB6, false);
      rx_message_chs.data[1] = 0x07;
      rx_message_chs.data[2] = get_lock_target_adjusted_value(0xCC, false);
      rx_message_chs.data[3] = 0x07;
      rx_message_chs.data[4] = get_lock_target_adjusted_value(0xD2, false);
      rx_message_chs.data[5] = 0x07;
      rx_message_chs.data[6] = get_lock_target_adjusted_value(0xD2, false);
      rx_message_chs.data[7] = 0x07;
      break;

    case BRAKES4_ID:
      rx_message_chs.data[0] = get_lock_target_adjusted_value(0xFE, false);
      rx_message_chs.data[1] = 0x00;
      rx_message_chs.data[2] = 0x00;
      rx_message_chs.data[3] = 0x64;
      rx_message_chs.data[4] = 0x00;
      rx_message_chs.data[5] = 0x00;
      rx_message_chs.data[6] = BRAKES4_counter;
      BRAKES4_crc = 0;
      for (uint8_t i = 0; i < 7; i++)
      {
        BRAKES4_crc ^= rx_message_chs.data[i];
      }
      rx_message_chs.data[7] = BRAKES4_crc;

      BRAKES4_counter = BRAKES4_counter + 16;
      if (BRAKES4_counter > 0xF0)
      {
        BRAKES4_counter = 0x00;
      }
      break;
    }
  }

  // edit the frames if configured as Gen5 (0AY) - frames left
  // commented-out are so they can be re-enabled later if a required
  if (haldexGeneration == 51)
  {
    switch (rx_message_chs.identifier)
    {
    // ---- Active (lock-modulated) -------------------------------------------
    case MOTOR1_ID:
      // PQ Motor_1 (0x280) - engine ECU broadcast.  Bytes 1..7 lock-adjusted to
      // bias the haldex; byte 0 left at 0 (Fahrerwunschmoment).
      rx_message_chs.data[0] = 0x00;                                        // Fahrerwunschmoment
      rx_message_chs.data[1] = get_lock_target_adjusted_value(0xFE, false); // Fahrerwunschmoment
      rx_message_chs.data[2] = get_lock_target_adjusted_value(0x20, false); // Motordrehzahl low
      rx_message_chs.data[3] = get_lock_target_adjusted_value(0x4E, false); // Motordrehzahl high
      rx_message_chs.data[4] = get_lock_target_adjusted_value(0xFE, false); // Fahrpedalwert
      rx_message_chs.data[5] = get_lock_target_adjusted_value(0xFE, false); // inneres_Motor_Moment_ohne_extern
      rx_message_chs.data[6] = get_lock_target_adjusted_value(0x16, false); // mechanisches_Motor_Verlustmoment
      rx_message_chs.data[7] = get_lock_target_adjusted_value(0xFE, false); // inneres_Motor_Moment
      break;

    case BRAKES3_ID:
      // PQ Bremse_3 (0x4A0) - per-wheel speeds (Radgeschw_VL/VR/HL/HR).
      // Low bytes lock-adjusted, high bytes fixed at 0x07.
      rx_message_chs.data[0] = get_lock_target_adjusted_value(0xB6, false); // Radgeschw_VL low
      rx_message_chs.data[1] = 0x07;                                        // Radgeschw_VL high
      rx_message_chs.data[2] = get_lock_target_adjusted_value(0xCC, false); // Radgeschw_VR low
      rx_message_chs.data[3] = 0x07;                                        // Radgeschw_VR high
      rx_message_chs.data[4] = get_lock_target_adjusted_value(0xD2, false); // Radgeschw_HL low
      rx_message_chs.data[5] = 0x07;                                        // Radgeschw_HL high
      rx_message_chs.data[6] = get_lock_target_adjusted_value(0xD2, false); // Radgeschw_HR low
      rx_message_chs.data[7] = 0x07;                                        // Radgeschw_HR high
      break;

    case BRAKES4_ID:
      // PQ Bremse_4 (0x2A0) - ABS coupling moment.  Byte 0 is a signed offset
      // around 0x7F (matches standalone: get_lock_target_adjusted_value(0x7F) - 0x7F).
      // data[6] is rolling counter (16-step), data[7] is XOR-CRC over data[0..6].
      rx_message_chs.data[0] = get_lock_target_adjusted_value(0x7F, false) - 0x7F; // ABS_Vorgabewert_hinten_Kupplung
      rx_message_chs.data[1] = 0x00;
      rx_message_chs.data[2] = 0x00;
      rx_message_chs.data[3] = 0x64;
      rx_message_chs.data[4] = 0x00;
      rx_message_chs.data[5] = 0x00;
      rx_message_chs.data[6] = BRAKES4_counter;
      BRAKES4_crc = 0;
      for (uint8_t i = 0; i < 7; i++)
      {
        BRAKES4_crc ^= rx_message_chs.data[i];
      }
      rx_message_chs.data[7] = BRAKES4_crc;

      BRAKES4_counter = BRAKES4_counter + 16;
      if (BRAKES4_counter > 0xF0)
      {
        BRAKES4_counter = 0x00;
      }
      break;

    case mLW_1:
      // PQ LW_1 (0x0C2) - steering-angle replay table; not lock-modulated.
      rx_message_chs.data[0] = lws_2[mLW_1_counter][0];
      rx_message_chs.data[1] = lws_2[mLW_1_counter][1];
      rx_message_chs.data[2] = lws_2[mLW_1_counter][2];
      rx_message_chs.data[3] = lws_2[mLW_1_counter][3];
      rx_message_chs.data[4] = lws_2[mLW_1_counter][4];
      rx_message_chs.data[5] = lws_2[mLW_1_counter][5];
      rx_message_chs.data[6] = lws_2[mLW_1_counter][6];
      rx_message_chs.data[7] = lws_2[mLW_1_counter][7];
      mLW_1_counter++;
      if (mLW_1_counter > 15)
        mLW_1_counter = 0;
      break;

      // ---- Inactive (kept for future use, no lock adjustment in standalone) ---
      /*

      case BRAKES1_ID:
        // PQ Bremse_1 (0x1A0) - ABS/ESP main broadcast.  Static in standalone.
        rx_message_chs.data[0] = 0x20;
        rx_message_chs.data[1] = 0x40;
        rx_message_chs.data[2] = 0xF0;
        rx_message_chs.data[3] = 0x07;
        rx_message_chs.data[4] = 0xFE;
        rx_message_chs.data[5] = 0xFE;
        rx_message_chs.data[6] = 0x00;
        rx_message_chs.data[7] = BRAKES1_counter;
        if (++BRAKES1_counter > 0x1F)
          BRAKES1_counter = 10;
        break;

      case mGetriebe_2:
        // PQ Getriebe_2 (0x540) - transmission status; needed by DTC 17497 but static.
        rx_message_chs.data[0] = (mGetriebe_2_counter << 4) & 0xF0;
        rx_message_chs.data[1] = 0x50;
        rx_message_chs.data[2] = 0xFF;
        rx_message_chs.data[3] = 0x00;
        rx_message_chs.data[4] = 0xFF;
        rx_message_chs.data[5] = 0x00;
        rx_message_chs.data[6] = 0x00;
        rx_message_chs.data[7] = 0xFF;
        mGetriebe_2_counter = (mGetriebe_2_counter + 1) & 0x0F;
        break;

      case BRAKES5_ID:
        // PQ Bremse_5 (0x4A8) - ESP brake-event broadcast.  Static in standalone.
        rx_message_chs.data[0] = 0xFE;
        rx_message_chs.data[1] = 0x7F;
        rx_message_chs.data[2] = 0x03;
        rx_message_chs.data[3] = 0x00;
        rx_message_chs.data[4] = 0x00;
        rx_message_chs.data[5] = 0x00;
        rx_message_chs.data[6] = BRAKES5_counter;
        rx_message_chs.data[7] = BRAKES5_counter2;
        BRAKES5_counter = BRAKES5_counter + 10;
        if (BRAKES5_counter > 0xF0) BRAKES5_counter = 0;
        BRAKES5_counter2 = BRAKES5_counter2 + 10;
        if (BRAKES5_counter2 > 0xF3) BRAKES5_counter2 = 3;
        break;

      case BRAKES8_ID:
        // PQ Bremse_8 (0x1AC) - ESP supplemental broadcast; dual rolling counters.
        rx_message_chs.data[0] = BRAKES8_counter;
        rx_message_chs.data[1] = BRAKES8_counter1;
        rx_message_chs.data[2] = 0x00;
        rx_message_chs.data[3] = 0x00;
        rx_message_chs.data[4] = 0x69;
        rx_message_chs.data[5] = 0x21;
        rx_message_chs.data[6] = 0x00;
        rx_message_chs.data[7] = 0xC1;
        if (++BRAKES8_counter > 0x8F) BRAKES8_counter = 0x80;
        if (++BRAKES8_counter1 > 0x0F) BRAKES8_counter1 = 0x00;
        break;

      case BRAKES2_ID:
        // PQ Bremse_2 (0x5A0) - ESP/ABS sensor broadcast.  Now static in standalone
        // (Querbeschleunigung was lock-adjusted historically; now 0x7F literal).
        rx_message_chs.data[0] = 0x80;
        rx_message_chs.data[1] = 0x7A;
        rx_message_chs.data[2] = 0x05;
        rx_message_chs.data[3] = BRAKES2_counter;
        rx_message_chs.data[4] = 0x7F;
        rx_message_chs.data[5] = 0xCA;
        rx_message_chs.data[6] = 0x1B;
        rx_message_chs.data[7] = 0xAB;
        BRAKES2_counter = BRAKES2_counter + 16;
        if (BRAKES2_counter > 0xF0) BRAKES2_counter = 0;
        break;

      case MOTOR2_ID:
        // PQ Motor_2 (0x288) - secondary engine broadcast.  Static in standalone.
        rx_message_chs.data[0] = 0x00;
        rx_message_chs.data[1] = 0x30;
        rx_message_chs.data[2] = 0x00;
        rx_message_chs.data[3] = 0x0A;
        rx_message_chs.data[4] = 0x0A;
        rx_message_chs.data[5] = 0x10;
        rx_message_chs.data[6] = 0xFE;
        rx_message_chs.data[7] = 0xFE;
        break;

      case MOTOR5_ID:
        // PQ Motor_5 (0x480) - tertiary engine broadcast.  Static in standalone.
        rx_message_chs.data[0] = 0xFE;
        rx_message_chs.data[1] = 0x00;
        rx_message_chs.data[2] = 0x00;
        rx_message_chs.data[3] = 0x00;
        rx_message_chs.data[4] = 0x00;
        rx_message_chs.data[5] = 0x00;
        rx_message_chs.data[6] = 0x00;
        rx_message_chs.data[7] = MOTOR5_counter;
        break;

      case mKombi_1:
        // PQ Kombi_1 (0x320) - instrument-cluster broadcast.  Static in standalone.
        rx_message_chs.data[0] = 0x24;
        rx_message_chs.data[1] = 0x00;
        rx_message_chs.data[2] = 0x1D;
        rx_message_chs.data[3] = 0xB9;
        rx_message_chs.data[4] = 0x07;
        rx_message_chs.data[5] = 0x42;
        rx_message_chs.data[6] = 0x09;
        rx_message_chs.data[7] = 0x81;
        break;

      case mKombi_3:
        // PQ Kombi_3 (0x520) - cluster odometer/keys.  Static in standalone.
        rx_message_chs.data[0] = 0x60;
        rx_message_chs.data[1] = 0x43;
        rx_message_chs.data[2] = 0x01;
        rx_message_chs.data[3] = 0x10;
        rx_message_chs.data[4] = 0x66;
        rx_message_chs.data[5] = 0xF1;
        rx_message_chs.data[6] = 0x03;
        rx_message_chs.data[7] = 0x02;
        break;

      case mGate_Komf_1:
        // PQ Gate_Komf_1 (0x390) - gateway-comfort broadcast.  Static in standalone.
        rx_message_chs.data[0] = 0x03;
        rx_message_chs.data[1] = 0x11;
        rx_message_chs.data[2] = 0x58;
        rx_message_chs.data[3] = 0x00;
        rx_message_chs.data[4] = 0x40;
        rx_message_chs.data[5] = 0x00;
        rx_message_chs.data[6] = 0x01;
        rx_message_chs.data[7] = 0x08;
        break;

      case BRAKES11_ID:
        // PQ Bremse_11 (0x5B7) - extended brake frame.  Static in standalone.
        rx_message_chs.data[0] = 0x00;
        rx_message_chs.data[1] = 0xC0;
        rx_message_chs.data[2] = 0x00;
        rx_message_chs.data[3] = 0x00;
        rx_message_chs.data[4] = 0x00;
        rx_message_chs.data[5] = 0x00;
        rx_message_chs.data[6] = 0x00;
        rx_message_chs.data[7] = 0x00;
        break;

      case mSysteminfo_1:
        // PQ Systeminfo_1 (0x5D0) - gateway vehicle-identity broadcast.  Static.
        rx_message_chs.data[0] = 0x00;
        rx_message_chs.data[1] = 0x24;
        rx_message_chs.data[2] = 0x35;
        rx_message_chs.data[3] = 0x0F;
        rx_message_chs.data[4] = 0x39;
        rx_message_chs.data[5] = 0x59;
        rx_message_chs.data[6] = 0x00;
        rx_message_chs.data[7] = 0x00;
        break;

      case mKombi_2:
        // PQ Kombi_2 (0x420) - cluster temps.  Static in standalone.
        rx_message_chs.data[0] = 0x4C;
        rx_message_chs.data[1] = 0x86;
        rx_message_chs.data[2] = 0x85;
        rx_message_chs.data[3] = 0x00;
        rx_message_chs.data[4] = 0x00;
        rx_message_chs.data[5] = 0x30;
        rx_message_chs.data[6] = 0xFF;
        rx_message_chs.data[7] = 0x04;
        break;

      case mDiagnose_1:
        // PQ Diagnose_1 (0x7D0) - diagnostic timestamp broadcast.  Static.
        rx_message_chs.data[0] = 0x26;
        rx_message_chs.data[1] = 0xF2;
        rx_message_chs.data[2] = 0x03;
        rx_message_chs.data[3] = 0x12;
        rx_message_chs.data[4] = 0x70;
        rx_message_chs.data[5] = 0x19;
        rx_message_chs.data[6] = 0x25;
        rx_message_chs.data[7] = mDiagnose_1_counter;
        mDiagnose_1_counter++;
        if (mDiagnose_1_counter > 0x1F) mDiagnose_1_counter = 0;
        break;
      */
    }
  }

  // edit the frames if configured as Gen5 (0CQ) - frames left
  // commented-out are so they can be re-enabled later if a required
  if (haldexGeneration == 50)
  {
    switch (rx_message_chs.identifier)
    {
      /*
            twai_message_t frame;
            frame.identifier = ESP_18; // 0x135.  Fixed response, no changes
            frame.extd = 0;
            frame.rtr = 0;
            frame.data_length_code = 8;
            frame.data[0] = 0x00; // supposed to have CRC? doesn't affect
            frame.data[1] = 0xC0; // always 0xC0, never changes
            frame.data[2] = 0x00; // doesn't affect
            frame.data[3] = 0x00; // doesn't affect
            frame.data[4] = 0x00; // doesn't affect
            frame.data[5] = 0x00; // doesn't affect
            frame.data[6] = 0x00; // doesn't affect
            frame.data[7] = 0x00; // doesn't affect
            twai_transmit_v2(twai_bus_1, &frame, 0);
      */
    case ESP_19:
      rx_message_chs.data[0] = get_lock_target_adjusted_value(ESP_19_counter2, false);        // HL - wheel speed
      rx_message_chs.data[1] = get_lock_target_adjusted_value(ESP_19_counter, false);         // HL - wheel speed
      rx_message_chs.data[2] = get_lock_target_adjusted_value(ESP_19_counter2, false);        // HR - wheel speed
      rx_message_chs.data[3] = get_lock_target_adjusted_value(ESP_19_counter, false);         // HR - wheel speed
      rx_message_chs.data[4] = get_lock_target_adjusted_value(ESP_19_counter2 + 0xCA, false); // VL - wheel speed 0xDB
      rx_message_chs.data[5] = get_lock_target_adjusted_value(ESP_19_counter, false);         // VL - wheel speed -- affects if =0x0B
      rx_message_chs.data[6] = get_lock_target_adjusted_value(ESP_19_counter2 + 0xCA, false); // VR - wheel speed 0xDB
      rx_message_chs.data[7] = get_lock_target_adjusted_value(ESP_19_counter, false);         // VR - wheel speed -- affects if =0x0B
      ESP_19_counter++;
      ESP_19_counter2++;
      if (ESP_19_counter > 0x1A) // 0x1e
      {
        ESP_19_counter = 0x01; // 0x10
      }
      if (ESP_19_counter2 > 0x0E) // 0x0a
      {
        ESP_19_counter2 = 0x00; // 0x00
      }
      break;

    case GETRIEBE_11:
      rx_message_chs.data[0] = 0x00;                // checksum placeholder none affect
      rx_message_chs.data[1] = GETRIEBE_11_counter; // rolling - 0x00>0x0F
      rx_message_chs.data[2] = 0x00;                // Was 0xFE Torque intervention at the engine. Requests a short-term reduction or increase in torque from the ECU. This signal is only valid in combination with GE_MMom_Status (for MQB) or GE_MMom_Status_02 (for MLBevo).
      rx_message_chs.data[3] = 0xFE;                // Pre-control torque (anticipatory torque request) (GE_MMom_Vorhalt_02)
      rx_message_chs.data[4] = 0x00;                // Actual gear/range selected (5=P, 6=R, 7=N, 8=D, 9=S, 10=E, 13/14=T)
      rx_message_chs.data[5] = 0x00;                // Shift sequence state (0=idle, 1=shift in progress, etc.) - does not affect (>0x00)
      rx_message_chs.data[6] = 0x00;                // Power transmission status / clutch lock-up state - does not affect (>0x00)
      rx_message_chs.data[7] = 0x00;                // Target gear of current shift

      rx_message_chs.data[0] = calcChecksum(rx_message_chs.data, ID_SEQ_0AD); // for 0x0AD

      GETRIEBE_11_counter++;
      if (GETRIEBE_11_counter > 0x0F)
      {
        GETRIEBE_11_counter = 0;
      }
      break;

    case MOTOR_12:
      rx_message_chs.data[0] = 0x00;                                                    // checksum placeholder
      rx_message_chs.data[1] = MOTOR_12_counter;                                        // rolling - 0x70>0x7F
      rx_message_chs.data[2] = 0x00;                                                    // doesn't affect Negative available torque (maximum engine braking) (MO_Mom_neg_verfuegbar) - does not affect
      rx_message_chs.data[3] = 0x00;                                                    // doesn't affect sometimes 0xC0, sometimes 0x00 Static torque limit
      rx_message_chs.data[4] = 0x00;                                                    // doesn't affect sometimes 0x3A, somtimes 0x39 Dynamic torque limit
      rx_message_chs.data[5] = 0x64;                                                    // doesn't affect Vehicle speed signal quality bit (0x64=good, 0x00=bad).  True?
      rx_message_chs.data[6] = 0x0F;                                                    // Engine speed signal quality bit. Was 0xD4 - does affect.  Bool
      rx_message_chs.data[7] = get_lock_target_adjusted_value(MOTOR_12_counter, false); // Engine speed / RPM was 0xAE affects.  >30 slows down.  Only adds 5%. Was 0x10 - does affect

      rx_message_chs.data[0] = calcChecksum(rx_message_chs.data, ID_SEQ_0A8); // for 0x0A8

      MOTOR_12_counter++;
      if (MOTOR_12_counter > 0x7F)
      {
        MOTOR_12_counter = 0x70;
      }
      break;

    case MOTOR_11:
      // MQB Motor_11 (0x0A7) - dual packing selected by user "Fix Hunting" toggle.
      //   fixHunting == false : V3 packing - works on 554C/D/H and 554K @ 100% lock.
      //   fixHunting == true  : DBC-correct BPK packing - needed on 554K at partial
      //                         lock (60/40, 70/30) where V3 packing causes hunting.
      if (!fixHunting)
      {
        rx_message_chs.data[0] = 0x00;                                        // checksum placeholder
        rx_message_chs.data[1] = MOTOR_11_counter;                            // rolling - 0x40>0x4F
        rx_message_chs.data[2] = 0xFA;                                        // Raw target torque (unfiltered driver demand) (MO_Mom_Soll_Roh)
        rx_message_chs.data[3] = 0xFA;                                        // Actual total torque output (Actual total torque output)
        rx_message_chs.data[4] = 0x00;                                        // doesn't affect Total inertia torque component (MO_Mom_Traegheit_Summe)
        rx_message_chs.data[5] = 0xFA;                                        // Filtered target torque (MO_Mom_Soll_gefiltert)
        rx_message_chs.data[6] = get_lock_target_adjusted_value(0xFA, false); // (MO_Mom_Soll_01?).  Massive effect.  Was 0x78
        rx_message_chs.data[7] = get_lock_target_adjusted_value(0xFA, false); // massive effect.  Was 0x3E
      }
      else
      {
        // ---- BPK packing (Fix Hunting on; needed for 554K @ partial lock) ----
        const uint8_t BPK_CEIL = 220;     // Nm at 100% lock
        const uint8_t BPK_SLEW_IST = 8;   // Nm/cycle, MO_Mom_Ist_Summe ramp
        const uint8_t BPK_SLEW_SOLF = 32; // Nm/cycle, MO_Mom_Soll_gefiltert ramp
        const uint8_t BPK_FLOOR = 10;

        uint8_t torqueNm = get_lock_target_adjusted_value(0xFE, false);
        torqueNm = (uint8_t)(BPK_FLOOR +
                             ((uint16_t)torqueNm * (BPK_CEIL - BPK_FLOOR)) / 0xFE);

        static uint8_t prevIstNm = 0, prevSolfNm = 0;
        auto slew = [](uint8_t cur, uint8_t target, uint8_t step) -> uint8_t
        {
          if (target > cur)
            return ((uint16_t)cur + step >= target) ? target : (uint8_t)(cur + step);
          if (target < cur)
            return (cur <= step || cur - step <= target) ? target : (uint8_t)(cur - step);
          return cur;
        };
        uint8_t istNm = slew(prevIstNm, torqueNm, BPK_SLEW_IST);
        uint8_t solfNm = slew(prevSolfNm, torqueNm, BPK_SLEW_SOLF);
        prevIstNm = istNm;
        prevSolfNm = solfNm;

        uint16_t rawSollRoh = (uint16_t)(torqueNm + 509) & 0x3FF;
        uint16_t rawIst = (uint16_t)(istNm + 509) & 0x3FF;
        uint16_t rawSolf = (uint16_t)(solfNm + 509) & 0x3FF;

        const uint8_t TRAEG_LO = 0xFD;
        const uint8_t TRAEG_HI = 0x01;
        const uint8_t SCHUB_LO = 0x07;
        const uint8_t SCHUB_HI = 0x1E;
        const uint8_t STATUS_FL = 0x20; // Normalbetrieb=1, QBit=valid

        rx_message_chs.data[0] = 0x00; // CRC placeholder
        rx_message_chs.data[1] = (MOTOR_11_counter & 0x0F) | ((rawSollRoh & 0x000F) << 4);
        rx_message_chs.data[2] = ((rawSollRoh >> 4) & 0x3F) | ((rawIst & 0x0003) << 6);
        rx_message_chs.data[3] = (rawIst >> 2) & 0xFF;
        rx_message_chs.data[4] = TRAEG_LO;
        rx_message_chs.data[5] = (TRAEG_HI & 0x03) | ((rawSolf & 0x3F) << 2);
        rx_message_chs.data[6] = ((rawSolf >> 6) & 0x0F) | ((SCHUB_LO & 0x0F) << 4);
        rx_message_chs.data[7] = (SCHUB_HI & 0x1F) | STATUS_FL;
      }

      rx_message_chs.data[0] = calcChecksum(rx_message_chs.data, ID_SEQ_0A7); // for 0x0A7

      MOTOR_11_counter++;
      if (MOTOR_11_counter > 0x4F)
      {
        MOTOR_11_counter = 0x40;
      }
      break;

    case ESP_14:                               // ESP_14 0x08A
      rx_message_chs.data[0] = 0x00;           // checksum placeholder
      rx_message_chs.data[1] = ESP_14_counter; // rolling - 0x10>0x1F
      rx_message_chs.data[2] = 0x00;           // doesn't affect
      rx_message_chs.data[3] = 0x00;           // doesn't affect sometimes 0xC0, sometimes 0x00
      rx_message_chs.data[4] = 0x00;           // doesn't affect BR_Vorg_Quer_Min Minimum specified limit value of the clutch's operating range by the ESP MQB Haldex: 100% torque corresponds to 2000 Nm.
      rx_message_chs.data[6] = 0x00;           // doesn't affect BR_Vorg_Allrad_Min Minimum specified limit value of the clutch's operating range by the ESP MQB Haldex: 100% torque corresponds to 2000 Nm

      appliedTorque = get_lock_target_adjusted_value(0xFE, false);

      rx_message_chs.data[5] = appliedTorque; // BR_Vorg_Quer_Max - lock-modulated (massive effect, ported from standalone)
      rx_message_chs.data[7] = appliedTorque; // BR_Vorg_Allrad_Max - lock-modulated (massive effect)
      // massive effects (4>7)

      rx_message_chs.data[0] = calcChecksum(rx_message_chs.data, ID_SEQ_08A); // for 0x08A

      ESP_14_counter++;
      if (ESP_14_counter > 0x1F)
      {
        ESP_14_counter = 0x10;
      }
      break;

      /*case LWI_01:
        rx_message_chs.data[0] = 0x00;           // checksum placeholder
        rx_message_chs.data[1] = LWI_01_counter; // rolling - 0x10>0x1F
        rx_message_chs.data[2] = 0x01;           // LWI_SensorStatus
        rx_message_chs.data[3] = 0x00;           // LWI_Qbit_sub_daten
        rx_message_chs.data[4] = 0x00;           // LWI_Qbit_Lendradwiken
        rx_message_chs.data[5] = 0x00;           // LWI_lendradwinken
        rx_message_chs.data[6] = 0x00;           // LWI_lendradw_geschw
        rx_message_chs.data[7] = 0x00;           // LWI_lendradw_geschw Unit Degress of Arc per Second

        rx_message_chs.data[0] = calcChecksum(rx_message_chs.data, ID_SEQ_086); // for 0x086

        LWI_01_counter++;
        if (LWI_01_counter > 0x1F)
        {
          LWI_01_counter = 0x10;
        }
        break;


    case MOTOR_20:
      rx_message_chs.data[0] = 0x00;             // checksum
      rx_message_chs.data[1] = MOTOR_20_counter; // rolling - 0x00>0x0F && MO_Accelerator_Raw_Value_01!
      rx_message_chs.data[2] = 0x40;             // no affect MO_Accelerator_Raw_Value_01 sss
      rx_message_chs.data[3] = 0x40;             // no affect sometimes 0xC0, sometimes 0x00
      rx_message_chs.data[4] = 0x19;             // no affect sometimes 0x3A, somtimes 0x39
      rx_message_chs.data[5] = 0x59;             // no affect
      rx_message_chs.data[6] = 0x7E;             // no affect
      rx_message_chs.data[7] = 0xFE;             // no affect

      rx_message_chs.data[0] = calcChecksum(rx_message_chs.data, ID_SEQ_121); // for 0x121

      MOTOR_20_counter++;
      if (MOTOR_20_counter > 0x0F)
      {
        MOTOR_20_counter = 0x00;
      }
      break;
      */

    case ESP_10:
      rx_message_chs.data_length_code = 8;                                    // DLC 8
      rx_message_chs.data[0] = 0x00;                                          // checksum placeholder
      rx_message_chs.data[1] = ESP_10_counter;                                // rolling - 0x00>0x0F
      rx_message_chs.data[2] = 0x01;                                          // no affect all these affect, find which one
      rx_message_chs.data[3] = 0x04;                                          // no effect sometimes 0xC0, sometimes 0x00
      rx_message_chs.data[4] = 0x00;                                          // no effect sometimes 0x3A, somtimes 0x39
      rx_message_chs.data[5] = 0x40;                                          // no effect
      rx_message_chs.data[6] = 0x00;                                          // no effect
      rx_message_chs.data[7] = 0xFF;                                          // this affects(!) - a good 40%.  Was 0xFF
      rx_message_chs.data[0] = calcChecksum(rx_message_chs.data, ID_SEQ_116); // for 0x116

      ESP_10_counter++;
      if (ESP_10_counter > 0x0F)
      {
        ESP_10_counter = 0x00;
      }
      break;

    case ESP_05:                                                              // ESP_05 0x106
      rx_message_chs.data_length_code = 8;                                    // DLC 8
      rx_message_chs.data[0] = 0x00;                                          // checksum
      rx_message_chs.data[1] = ESP_05_counter;                                // rolling - 0x80>0x8F
      rx_message_chs.data[2] = 0x64;                                          // no effect
      rx_message_chs.data[3] = 0xC0;                                          // this affects(!) sometimes 0xC0, sometimes 0x00
      rx_message_chs.data[4] = 0x00;                                          // no effect sometimes 0x3A, somtimes 0x39
      rx_message_chs.data[5] = 0x00;                                          // no effect
      rx_message_chs.data[6] = 0xFD;                                          // no effect
      rx_message_chs.data[7] = 0x00;                                          // this affects(!) - on/off.  Was 0x10.  0x00 doesn't hurt
      rx_message_chs.data[0] = calcChecksum(rx_message_chs.data, ID_SEQ_106); // for 0x106

      ESP_05_counter++;
      if (ESP_05_counter > 0x8F)
      {
        ESP_05_counter = 0x80;
      }
      break;

    case EPB_01:                               // EPB_01 0x104
      rx_message_chs.data_length_code = 8;     // DLC 8
      rx_message_chs.data[0] = 0x00;           // checksum
      rx_message_chs.data[1] = EPB_01_counter; // rolling - 0x30>0x3F - none affect
      rx_message_chs.data[2] = 0xA6;
      rx_message_chs.data[3] = 0x00; // sometimes 0xC0, sometimes 0x00
      rx_message_chs.data[4] = 0xE6; // sometimes 0x3A, somtimes 0x39
      rx_message_chs.data[5] = 0x00;
      rx_message_chs.data[6] = 0x00;
      rx_message_chs.data[7] = 0x31;
      rx_message_chs.data[0] = calcChecksum(rx_message_chs.data, ID_SEQ_104); // for 0x104

      EPB_01_counter++;
      if (EPB_01_counter > 0x3F)
      {
        EPB_01_counter = 0x30;
      }
      break;

      /*

          case ESP_02:                                                          // ESP_02 0x10B
        rx_message_chs.data[0] = 0x00;                                          // checksum
        rx_message_chs.data[1] = ESP_02_counter;                                // rolling - 0x00>0x1F
        rx_message_chs.data[2] = 0x7E;                                          // doesn't effect one of these affects, find which one - doesn't affect
        rx_message_chs.data[3] = 0x0F;                                          // doesn't effect sometimes 0xC0, sometimes 0x00
        rx_message_chs.data[4] = 0x82;                                          // doesn't effect sometimes 0x3A, somtimes 0x39
        rx_message_chs.data[5] = 0x0C;                                          // doesn't effect rolling?
        rx_message_chs.data[6] = 0x40;                                          // doesn't efffect
        rx_message_chs.data[7] = 0x00;                                          // doesn't effect
        rx_message_chs.data[0] = calcChecksum(rx_message_chs.data, ID_SEQ_101); // for 0x101

        ESP_02_counter++;
        if (ESP_02_counter > 0x1F)
        {
          ESP_02_counter = 0x00;
        }
        break;

      case ESP_21:
        rx_message_chs.data[0] = 0x00;           // checksum
        rx_message_chs.data[1] = ESP_21_counter; // rolling - 0x00>0x1F
        rx_message_chs.data[2] = 0x1F;           // in diagnosis? none affect
        rx_message_chs.data[3] = 0x80;           // sometimes 0xC0, sometimes 0x00
        rx_message_chs.data[4] = 0x00;           // sometimes 0x3A, somtimes 0x39
        rx_message_chs.data[5] = 0x00;
        rx_message_chs.data[6] = 0x00;
        rx_message_chs.data[7] = 0x00;
        rx_message_chs.data[0] = calcChecksum(rx_message_chs.data, ID_SEQ_0fd); // for 0x0fd

        ESP_21_counter++;
        if (ESP_21_counter > 0x1F)
        {
          ESP_21_counter = 0x00;
        }
        break;

      case KOMBI_01:
        rx_message_chs.data[0] = 0x10; // angle of turn (block 011) low byte
        rx_message_chs.data[1] = 0x20; // checksum (0x20>0x2F)
        rx_message_chs.data[2] = 0x02; //
        rx_message_chs.data[3] = 0x00; //
        rx_message_chs.data[4] = 0x0C; //
        rx_message_chs.data[5] = 0x00; //
        rx_message_chs.data[6] = 0x00; //
        rx_message_chs.data[7] = 0x24; //
        break;

      case ESP_23:
        rx_message_chs.data[0] = 0x00;                                          // checksum placeholder no effect
        rx_message_chs.data[1] = ESP_23_counter;                                // ESP_23_counter;           // no effect B high byte
        rx_message_chs.data[2] = 0xBF;                                          // no effect C
        rx_message_chs.data[3] = 0x7F;                                          // no effect D
        rx_message_chs.data[4] = 0x00;                                          // rate of change (block 010)
        rx_message_chs.data[5] = 0x00;                                          // rate of change (block 010)
        rx_message_chs.data[6] = 0x7C;                                          // rate of change (block 010)
        rx_message_chs.data[7] = 0x78;                                          // rate of change (block 010)
        rx_message_chs.data[0] = calcChecksum(rx_message_chs.data, ID_SEQ_5be); // for 0x5be

        ESP_23_counter++;
        if (ESP_23_counter > 0x1F)
        {
          ESP_23_counter = 0x00;
        }
        break;

      case Parkhilfe_04:
        rx_message_chs.data[0] = 0x00; // angle of turn (block 011) low byte
        rx_message_chs.data[1] = 0x00; // no effect B high byte
        rx_message_chs.data[2] = 0x00; // no effect C
        rx_message_chs.data[3] = 0x00; // no effect D
        rx_message_chs.data[4] = 0x00; // rate of change (block 010)
        rx_message_chs.data[5] = 0x00; // rate of change (block 010)
        rx_message_chs.data[6] = 0x00; // rate of change (block 010)
        rx_message_chs.data[7] = 0x24; // rate of change (block 010)
        break;

      case GATEWAY_72:
        rx_message_chs.data[0] = 0x50; //
        rx_message_chs.data[1] = 0x80; //
        rx_message_chs.data[2] = 0x00; //
        rx_message_chs.data[3] = 0x00; //
        rx_message_chs.data[4] = 0x05; //
        rx_message_chs.data[5] = 0x10; //
        rx_message_chs.data[6] = 0x01; //
        rx_message_chs.data[7] = 0x78; //
        break;

      case GETRIEBE_14:
        rx_message_chs.data[0] = 0x00; // Maximum possible acceleration (limited by gear/clutch)
        rx_message_chs.data[1] = 0x00; // Charisma drive programme selected (affects shift mapping)
        rx_message_chs.data[2] = 0x54; // Charisma system status
        rx_message_chs.data[3] = 0x24; // Drag/friction loss torque in transmission
        rx_message_chs.data[4] = 0x00; // Launch control active
        rx_message_chs.data[5] = 0x60; //
        rx_message_chs.data[6] = 0x01; //
        rx_message_chs.data[7] = 0x51; //
        break;

      */

    case MOTOR_14:
      rx_message_chs.data[0] = 0x00;                                          // checksum
      rx_message_chs.data[1] = MOTOR_14_counter;                              // 0x10 to 0x1F
      rx_message_chs.data[2] = 0xE6;                                          // doesn't effect Start/stop system state (0=inactive, 1=stopping, 2=stopped, 3=restarting)
      rx_message_chs.data[3] = 0x01;                                          // this affects(!) on/off Restart event flag
      rx_message_chs.data[4] = 0xC8;                                          // doesn't effect Engine stop event flag
      rx_message_chs.data[5] = 0x80;                                          // doesn't effect
      rx_message_chs.data[6] = 0x00;                                          // doesn't effect
      rx_message_chs.data[7] = 0x80;                                          // doesn't effect
      rx_message_chs.data[0] = calcChecksum(rx_message_chs.data, ID_SEQ_3be); // for 0x3be

      MOTOR_14_counter++;
      if (MOTOR_14_counter > 0x1F)
      {
        MOTOR_14_counter = 0x10;
      }
      break;

    case ESP_07:
      rx_message_chs.data[0] = 0x00;                                          // checksum
      rx_message_chs.data[1] = ESP_07_counter;                                // 0x20>0x2F
      rx_message_chs.data[2] = 0x00;                                          // one of these affects, find which one
      rx_message_chs.data[3] = 0x00;                                          // no effect
      rx_message_chs.data[4] = 0x00;                                          // no effect
      rx_message_chs.data[5] = 0x00;                                          // no efefct
      rx_message_chs.data[6] = 0x00;                                          // no effect
      rx_message_chs.data[7] = 0x00;                                          // no effect
      rx_message_chs.data[0] = calcChecksum(rx_message_chs.data, ID_SEQ_392); // for 0x392

      ESP_07_counter++;
      if (ESP_07_counter > 0x1F)
      {
        ESP_07_counter = 0x00;
      }
      break;
      /*

    case ESP_29:
      rx_message_chs.data[0] = 0x00; //
      rx_message_chs.data[1] = 0x20; // checksum (0x20>0x2F)?  Not in Savvy
      rx_message_chs.data[2] = 0x59; //
      rx_message_chs.data[3] = 0x00; //
      rx_message_chs.data[4] = 0x00; //
      rx_message_chs.data[5] = 0x00; //
      rx_message_chs.data[6] = 0x00; //
      rx_message_chs.data[7] = 0x00; //
      break;

    case MOTOR_07:
      rx_message_chs.data[0] = 0xA0; // no effect from any
      rx_message_chs.data[1] = 0x5A; //
      rx_message_chs.data[2] = 0x56; //
      rx_message_chs.data[3] = 0xA3; //
      rx_message_chs.data[4] = 0x80; //
      rx_message_chs.data[5] = 0xA0; //
      rx_message_chs.data[6] = 0x59; //
      rx_message_chs.data[7] = 0x01; //
      break;

    case CHARISMA_01:
      rx_message_chs.data[0] = 0x00; // CHA_Target_Driving_Program_AGA & CHA_Target_Driving_Prior_ESP
      rx_message_chs.data[1] = 0x00; // CHA_Target_Driving_Pri_Freewheel & void
      rx_message_chs.data[2] = 0x22; // CHA_Target_Driving_Program_MO & CHA_Target_Driving_Program_GE
      rx_message_chs.data[3] = 0x02; // CHA_Target_Driving_PR_ALR (inc. AWD) & CHA_Target_Driving_Program_MO_BZS
      rx_message_chs.data[4] = 0x02; // CHA_Target_Driving_Project_DR & CHA_Target_Driving_Prior_VAQ
      rx_message_chs.data[5] = 0x20; // CHA_Target_Driving_PR_AFS & CHA_Target_Driving_Program_RGS
      rx_message_chs.data[6] = 0x02; // CHA_Target_Driving_Price_EPS & CHA_Target_Driving_Principal_ACC
      rx_message_chs.data[7] = 0x02; // CHA_Target_Driving_Prior_SAK & CHA_Target_Driving_Program_MO_StSt
      break;

    case SYSTEMINFO_01:
      rx_message_chs.data[0] = 0x84; //
      rx_message_chs.data[1] = 0x3C; //
      rx_message_chs.data[2] = 0x00; //
      rx_message_chs.data[3] = 0x7F; //
      rx_message_chs.data[4] = 0x14; //
      rx_message_chs.data[5] = 0x00; //
      rx_message_chs.data[6] = 0x00; //
      rx_message_chs.data[7] = 0x00; //
      break;

    case MOTOR_CODE_01:
      rx_message_chs.data[0] = 0x00;                                          // checksum
      rx_message_chs.data[1] = MOTOR_CODE_01_counter;                         // rolling (10>1F)
      rx_message_chs.data[2] = 0x2B;                                          //
      rx_message_chs.data[3] = 0x53;                                          //
      rx_message_chs.data[4] = 0x14;                                          //
      rx_message_chs.data[5] = 0x14;                                          //
      rx_message_chs.data[6] = 0xD7;                                          //
      rx_message_chs.data[7] = 0x24;                                          //
      rx_message_chs.data[0] = calcChecksum(rx_message_chs.data, ID_SEQ_641); // for 0x641

      MOTOR_CODE_01_counter++;
      if (MOTOR_CODE_01_counter > 0x1F)
      {
        MOTOR_CODE_01_counter = 0x10;
      }
      break;

    case ESP_20:
      rx_message_chs.data[0] = 0x00;                                          // checksum
      rx_message_chs.data[1] = ESP_20_counter;                                // rolling (30>3F)
      rx_message_chs.data[2] = 0x2B;                                          // no effect C
      rx_message_chs.data[3] = 0x10;                                          // no effect D
      rx_message_chs.data[4] = 0x00;                                          //
      rx_message_chs.data[5] = 0x00;                                          //
      rx_message_chs.data[6] = 0xE2;                                          //
      rx_message_chs.data[7] = 0x79;                                          // BR_Tire circumference
      rx_message_chs.data[0] = calcChecksum(rx_message_chs.data, ID_SEQ_65d); // for 0x65d

      ESP_20_counter++;
      if (ESP_20_counter > 0x3F)
      {
        ESP_20_counter = 0x30;
      }
      break;

    case DIAGNOSE_01:
      rx_message_chs.data[0] = 0x30; //
      rx_message_chs.data[1] = 0x4D; //
      rx_message_chs.data[2] = 0x58; //
      rx_message_chs.data[3] = 0xA2; //
      rx_message_chs.data[4] = 0x89; //
      rx_message_chs.data[5] = 0x85; //
      rx_message_chs.data[6] = 0x3F; // 0x3F OR 0xBF? (3F, then BF, then 3F, then BF...)
      rx_message_chs.data[7] = 0x30; // 2D, then 2D, then 2E, then 2E, then 2F, then 2F... roll over? When?
      break;

    case KOMBI_02:
      rx_message_chs.data[0] = 0x4D; // no effect from any
      rx_message_chs.data[1] = 0x58; //
      rx_message_chs.data[2] = 0xF2; //
      rx_message_chs.data[3] = 0xEE; //
      rx_message_chs.data[4] = 0x04; //
      rx_message_chs.data[5] = 0x2B; //
      rx_message_chs.data[6] = 0x00; //
      rx_message_chs.data[7] = 0x78; //
      break;
      //...
      */
    }
  }
}
