#include <OpenHaldexC6_Calculations.h>

// Only executed when in MODE_FWD/MODE_5050/MODE_CUSTOM
static inline bool lock_enabled() {
  bool throttle_ok = false;
  bool speed_ok = false;

  if (state.mode != MODE_CUSTOM) {
    throttle_ok = (state.pedal_threshold == 0) || (int(received_pedal_value) >= state.pedal_threshold);
    speed_ok = (disableSpeed == 0) || (received_vehicle_speed <= disableSpeed);
    return throttle_ok && speed_ok;
  }

  if (state.mode == MODE_CUSTOM) {
    if (customSpeed && !customThrottle) {
      speed_ok = (speedArray[0] == 0) || (received_vehicle_speed <= speedArray[0]);
      throttle_ok = true;
    }
    if (customThrottle && !customSpeed) {
      throttle_ok = (throttleArray[0] == 0) || (int(received_pedal_value) >= throttleArray[0]);
      speed_ok = true;
    }
    if (customThrottle && customSpeed) {
      speed_ok = (speedArray[0] == 0) || (received_vehicle_speed <= speedArray[0]);
      throttle_ok = (throttleArray[0] == 0) || (int(received_pedal_value) >= throttleArray[0]);
    }

    return throttle_ok && speed_ok;
  }
  
  return false;
}

// Only executed when in MODE_FWD/MODE_5050/MODE_CUSTOM
float get_lock_target_adjustment() {
  // Handle FWD and 5050 modes.
  switch (state.mode) {
    case MODE_FWD:
      return 0;

    case MODE_5050:
      if (lock_enabled()) {
        return 100;
      }
      return 0;

    case MODE_6040:
      if (lock_enabled()) {
        return 40;
      }
      return 0;

    case MODE_7525:
      if (lock_enabled()) {
        return 30;
      }
      return 0;

    case MODE_CUSTOM:
      if (lock_enabled()) {
        return lockArray[0];
      }
      return 0;

    default:
      return 0;
  }

  // Getting here means it's not FWD or 5050/7525.

  // Check if locking is necessary.
  if (!lock_enabled()) {
    return 0;
  }

  // Find the pair of lockpoints between which the vehicle speed falls.
  lockpoint_t lp_lower = state.custom_mode.lockpoints[0];
  lockpoint_t lp_upper = state.custom_mode.lockpoints[state.custom_mode.lockpoint_count - 1];

  // Look for the lockpoint above the current vehicle speed.
  for (uint8_t i = 0; i < state.custom_mode.lockpoint_count; i++) {
    if (received_vehicle_speed <= state.custom_mode.lockpoints[i].speed) {
      lp_upper = state.custom_mode.lockpoints[i];
      lp_lower = state.custom_mode.lockpoints[(i == 0) ? 0 : (i - 1)];
      break;
    }
  }

  // Handle the case where the vehicle speed is lower than the lowest lockpoint.
  if (received_vehicle_speed <= lp_lower.speed) {
    return lp_lower.lock;
  }

  // Handle the case where the vehicle speed is higher than the highest lockpoint.
  if (received_vehicle_speed >= lp_upper.speed) {
    return lp_upper.lock;
  }

  // In all other cases, interpolation is necessary.
  float inter = (float)(lp_upper.speed - lp_lower.speed) / (float)(received_vehicle_speed - lp_lower.speed);

  // Calculate the target.
  float target = lp_lower.lock + ((float)(lp_upper.lock - lp_lower.lock) / inter);
  return target;
}

// Only executed when in MODE_FWD/MODE_5050/MODE_CUSTOM
uint8_t get_lock_target_adjusted_value(uint8_t value, bool invert) {
  // Handle 5050 mode.
  if (lock_target == 100) {
    if (lock_enabled()) {
      return (invert ? (0xFE - value) : value);
    }
    return (invert ? 0xFE : 0x00);
  }

  // Handle FWD and CUSTOM modes.
  // No correction is necessary if the target is already 0.
  if (lock_target == 0) {
    return (invert ? 0xFE : 0x00);
  }

  float correction_factor = ((float)lock_target / 2) + 20;
  uint8_t corrected_value = value * (correction_factor / 100);
  if (lock_enabled()) {
    return (invert ? (0xFE - corrected_value) : corrected_value);
  }
  return (invert ? 0xFE : 0x00);
}

// Only executed when in MODE_FWD/MODE_5050/MODE_CUSTOM
void getLockData(twai_message_t& rx_message_chs) {
  // Get the initial lock target.
  lock_target = get_lock_target_adjustment();

  // Edit the frames if configured as Gen1...
  if (haldexGeneration == 1) {
    switch (rx_message_chs.identifier) {
      case MOTOR1_ID:
        rx_message_chs.data[0] = 0x00;
        rx_message_chs.data[1] = get_lock_target_adjusted_value(0xFE, false);
        rx_message_chs.data[2] = 0x21;
        rx_message_chs.data[3] = get_lock_target_adjusted_value(0x4E, false);
        rx_message_chs.data[4] = get_lock_target_adjusted_value(0xFE, false);
        rx_message_chs.data[5] = get_lock_target_adjusted_value(0xFE, false);
        appliedTorque = rx_message_chs.data[6];

        switch (state.mode) {
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

  // Edit the frames if configured as Gen2...
  if (haldexGeneration == 2) {
    switch (rx_message_chs.identifier) {
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
  
  // Edit the frames if configured as Gen4...
  if (haldexGeneration == 4) {
    switch (rx_message_chs.identifier) {
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
        if (mLW_1_counter > 15) {
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
        for (uint8_t i = 0; i < 7; i++) {
          BRAKES4_crc ^= rx_message_chs.data[i];
        }
        rx_message_chs.data[7] = BRAKES4_crc;

        BRAKES4_counter = BRAKES4_counter + 16;
        if (BRAKES4_counter > 0xF0) {
          BRAKES4_counter = 0x00;
        }
        break;
    }
  }
}
