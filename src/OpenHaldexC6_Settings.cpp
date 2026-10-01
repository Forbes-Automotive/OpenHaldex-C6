#include <OpenHaldexC6_Settings.h>

// Driving settings reachable from both the web UI and the DashCAN app (BLE).
// The web API used to validate these inline in settingsIncoming; the rules are
// unchanged, only shared now.

#define DS_LOCK_RELEASE_MIN 5   // %/s, same range as the web UI slider
#define DS_LOCK_RELEASE_MAX 500

bool applyDrivingSetting(uint8_t id, uint16_t value)
{
  const bool isBoolValue = value <= 1;
  const bool on = value == 1;

  switch (id)
  {
  case DS_TC_FORCE_MODE:
    if (!isBoolValue)
      return false;
    tcForceMode = on;
    return true;
  case DS_TC_FORCE_MODE_VALUE:
    if (value >= (uint16_t)openhaldex_mode_t_MAX)
      return false;
    tcForceModeValue = (uint8_t)value;
    return true;
  case DS_HAZARD_FORCE_MODE:
    if (!isBoolValue)
      return false;
    hazardForceMode = on;
    if (!hazardForceMode)
      hazardForceModeFlag = false; // clear active flag when feature is disabled
    return true;
  case DS_HAZARD_FORCE_MODE_VALUE:
    if (value >= (uint16_t)openhaldex_mode_t_MAX)
      return false;
    hazardForceModeValue = (uint8_t)value;
    return true;
  case DS_EXT_BTN_FORCE_MODE:
    if (!isBoolValue)
      return false;
    extBtnForceMode = on;
    return true;
  case DS_EXT_BTN_FORCE_MODE_VALUE:
    if (value >= (uint16_t)openhaldex_mode_t_MAX)
      return false;
    extBtnForceModeValue = (uint8_t)value;
    return true;
  case DS_FOLLOW_BRAKE:
    if (!isBoolValue)
      return false;
    followBrake = on;
    return true;
  case DS_FOLLOW_HANDBRAKE:
    if (!isBoolValue)
      return false;
    followHandbrake = on;
    return true;
  case DS_DISENGAGE_UNDER_SPEED: // 0 = off
    disengageUnderSpeed = constrain(value, 0, 300);
    return true;
  case DS_DISENGAGE_ABOVE_SPEED: // 0 = off
    disengageAboveSpeed = constrain(value, 0, 300);
    return true;
  case DS_DISABLE_THROTTLE:
    disableThrottle = (uint8_t)constrain(value, 0, 100);
    state.pedal_threshold = disableThrottle;
    return true;
  case DS_STEERING_SCALE_ENABLED:
    if (!isBoolValue)
      return false;
    steeringScaleEnabled = on;
    return true;
  case DS_LOCK_RELEASE_ENABLED:
    if (!isBoolValue)
      return false;
    lockReleaseEnabled = on;
    return true;
  case DS_LOCK_RELEASE_RATE:
    lockReleaseRatePerSec = (float)constrain(value, DS_LOCK_RELEASE_MIN, DS_LOCK_RELEASE_MAX);
    return true;
  case DS_LIVE_DIAG_ENABLED:
    if (!isBoolValue)
      return false;
    liveDiagEnabled = on;
    return true;
  case DS_LED_BRIGHTNESS:
    ledBrightness = (uint8_t)constrain(value, 0, 255);
    return true;
  default:
    return false;
  }
}

uint8_t drivingSettingValueLen(uint8_t id)
{
  switch (id)
  {
  case DS_DISENGAGE_UNDER_SPEED:
  case DS_DISENGAGE_ABOVE_SPEED:
  case DS_LOCK_RELEASE_RATE:
    return 2;
  default:
    return (id >= DS_TC_FORCE_MODE && id <= DS_LED_BRIGHTNESS) ? 1 : 0;
  }
}

static void put16le(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)(v >> 8);
}

void buildDrivingSettings(uint8_t *out)
{
  uint8_t flags = 0;
  if (tcForceMode)
    flags |= 1 << 0;
  if (hazardForceMode)
    flags |= 1 << 1;
  if (extBtnForceMode)
    flags |= 1 << 2;
  if (followBrake)
    flags |= 1 << 3;
  if (followHandbrake)
    flags |= 1 << 4;
  if (steeringScaleEnabled)
    flags |= 1 << 5;
  if (lockReleaseEnabled)
    flags |= 1 << 6;
  if (liveDiagEnabled)
    flags |= 1 << 7;

  out[0] = DRIVING_SETTINGS_LAYOUT;
  out[1] = flags;
  out[2] = tcForceModeValue;
  out[3] = hazardForceModeValue;
  out[4] = extBtnForceModeValue;
  put16le(&out[5], disengageUnderSpeed);
  put16le(&out[7], disengageAboveSpeed);
  out[9] = disableThrottle;
  put16le(&out[10], (uint16_t)(lockReleaseRatePerSec + 0.5f));
  out[12] = ledBrightness;
  out[13] = 0; // reserved flags
}
