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
  if (extBtnForceMode || tcForceMode) // if external button or tc force mode enabled, override all and return force mode value (0, 40, 50, 60, 100 or expert)
  {
    if (extButtonForceModeFlag || tcForceModeFlag) // if either flag is active, return the forced lock value, otherwise return 0
    {
      switch (forceModeValue)
      {
      case 0:
        return 0; // stock
      case 1:
        return 0; // FWD
      case 2:
        return 100; // 50:50
      case 3:
        return 40; // 60:40
      case 4:
        return 30; // 75:25
      case 5:
        return get_expert_lock_target(); // expert - return calculated expert lock target
      default:
        return 0; // default to 0 if invalid value
      }
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
  else
  {
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
  haldexLearnStep   = 0;
  haldexLearnCF     = 0;
  haldexLearnActive = true;

  xTaskCreate(haldexLearnTask, "haldexLearn", 4096, nullptr, 1, nullptr);
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

  if (haldexGeneration == 5)
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

      /*case GETRIEBE_11:
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
        */

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
      rx_message_chs.data[0] = 0x00;                                        // checksum placeholder
      rx_message_chs.data[1] = MOTOR_11_counter;                            // rolling - 0x40>0x4F
      rx_message_chs.data[2] = 0xFA;                                        // Raw target torque (unfiltered driver demand) (MO_Mom_Soll_Roh)
      rx_message_chs.data[3] = 0xFA;                                        // Actual total torque output (Actual total torque output)
      rx_message_chs.data[4] = 0x00;                                        // doesn't affect Total inertia torque component (MO_Mom_Traegheit_Summe)
      rx_message_chs.data[5] = 0xFA;                                        // Filtered target torque (MO_Mom_Soll_gefiltert)
      rx_message_chs.data[6] = get_lock_target_adjusted_value(0xFA, false); // (MO_Mom_Soll_01?).  Massive effect.  Was 0x78
      rx_message_chs.data[7] = get_lock_target_adjusted_value(0xFA, false); // massive effect.  Was 0x3E

      rx_message_chs.data[0] = calcChecksum(rx_message_chs.data, ID_SEQ_0A7); // for 0x0A7

      MOTOR_11_counter++;
      if (MOTOR_11_counter > 0x4F)
      {
        MOTOR_11_counter = 0x40;
      }
      break;

    case ESP_14:                                                            // ESP_14 0x08A
      rx_message_chs.data[0] = 0x00;                                        // checksum placeholder
      rx_message_chs.data[1] = ESP_14_counter;                              // rolling - 0x10>0x1F
      rx_message_chs.data[2] = 0x00;                                        // doesn't affect
      rx_message_chs.data[3] = 0x00;                                        // doesn't affect sometimes 0xC0, sometimes 0x00
      rx_message_chs.data[4] = 0x00;                                        // doesn't affect BR_Vorg_Quer_Min Minimum specified limit value of the clutch's operating range by the ESP MQB Haldex: 100% torque corresponds to 2000 Nm.
      rx_message_chs.data[5] = 0x00;                                        // BR_Vorg_Quer_Max Maximum predefined limit of the clutch's operating range by the ESP MQB Haldex: 100% torque corresponds to 2000 Nm.
      rx_message_chs.data[6] = 0x00;                                        // doesn't affect BR_Vorg_Allrad_Min Minimum specified limit value of the clutch's operating range by the ESP MQB Haldex: 100% torque corresponds to 2000 Nm
      rx_message_chs.data[7] = get_lock_target_adjusted_value(0xFE, false); // BR_Vorg_Allrad_Max Maximum specified limit of the clutch's operating range by the ESP MQB Haldex: 100% torque corresponds to 2000 Nm.
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
      rx_message_chs.data_length_code = 8;                                            // DLC 8
      rx_message_chs.data[0] = 0x00;                                                  // checksum placeholder
      rx_message_chs.data[1] = ESP_10_counter;                                        // rolling - 0x00>0x0F
      rx_message_chs.data[2] = 0x01;                                                  // no affect all these affect, find which one
      rx_message_chs.data[3] = 0x04;                                                  // no effect sometimes 0xC0, sometimes 0x00
      rx_message_chs.data[4] = 0x00;                                                  // no effect sometimes 0x3A, somtimes 0x39
      rx_message_chs.data[5] = 0x40;                                                  // no effect
      rx_message_chs.data[6] = 0x00;                                                  // no effect
      rx_message_chs.data[7] = get_lock_target_adjusted_value(ESP_10_counter, false); // this affects(!) - a good 40%.  Was 0xFF
      rx_message_chs.data[0] = calcChecksum(rx_message_chs.data, ID_SEQ_116);         // for 0x116

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

      /*case EPB_01:                               // EPB_01 0x104
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


      case ESP_02:                                                              // ESP_02 0x10B
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
