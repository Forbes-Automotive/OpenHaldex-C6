#include <OpenHaldexC6_Calculations.h>

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
  throttle = constrain(throttle, 0, 100);

  float speed = received_vehicle_speed;
  if (speed < 0)
  {
    speed = 0;
  }

  uint8_t t0 = 0;
  uint8_t t1 = 0;
  float t_ratio = 0;

  if (throttle >= throttleArray[throttleArrayCount - 1])
  {
    t0 = throttleArrayCount - 1;
    t1 = t0;
  }
  else
  {
    for (uint8_t i = 0; i < throttleArrayCount - 1; i++)
    {
      if (throttle <= throttleArray[i + 1])
      {
        t0 = i;
        t1 = i + 1;
        float denom = (float)throttleArray[t1] - (float)throttleArray[t0];
        t_ratio = (denom > 0) ? ((throttle - throttleArray[t0]) / denom) : 0;
        break;
      }
    }
  }

  uint8_t s0 = 0;
  uint8_t s1 = 0;
  float s_ratio = 0;

  if (speed >= speedArray[speedArrayCount - 1])
  {
    s0 = speedArrayCount - 1;
    s1 = s0;
  }
  else
  {
    for (uint8_t i = 0; i < speedArrayCount - 1; i++)
    {
      if (speed <= speedArray[i + 1])
      {
        s0 = i;
        s1 = i + 1;
        float denom = (float)speedArray[s1] - (float)speedArray[s0];
        s_ratio = (denom > 0) ? ((speed - speedArray[s0]) / denom) : 0;
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

  if (v < 0)
  {
    v = 0;
  }
  if (v > 100)
  {
    v = 100;
  }
  return int(v);
}

float get_lock_target_adjustment()
{
  if (extButtonForce5050 || tcForce5050)
  {
    if (extButtonForce5050Flag || tcForce5050Flag)
    {
      return 100;
    }
  }

  // handle each mode and calculate lock
  switch (state.mode)
  {
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
  // handle 5050 mode
  if (lock_target == 100)
  {
    if (lock_enabled())
    {
      return (invert ? (0xFE - value) : value);
    }
    return (invert ? 0xFE : 0x00);
  }

  // handle FWD mode
  if (lock_target == 0)
  {
    return (invert ? 0xFE : 0x00);
  }

  // handle all other modes and apply 'correction' factor - could iterate this
  float correction_factor = ((float)lock_target / 2) + 20;
  uint8_t corrected_value = value * (correction_factor / 100);
  if (lock_enabled())
  {
    return (invert ? (0xFE - corrected_value) : corrected_value);
  }
  return (invert ? 0xFE : 0x00);
}

void getLockData(twai_message_t &rx_message_chs)
{
  // get the initial lock target
  lock_target = get_lock_target_adjustment();

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
      rx_message_chs.data[6] = get_lock_target_adjusted_value(0xFE, false);
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
}
