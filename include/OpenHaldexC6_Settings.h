#pragma once

#include <OpenHaldexC6_defs.h>

// Driving settings shared by the web API (/api/settings) and BLE (Settings
// characteristic): one validation path so both behave the same. Ids are the
// BLE wire ids (documents/MOBILE_APP_OPENHALDEX.md) - never reuse or renumber them.
enum DrivingSettingId : uint8_t
{
  DS_TC_FORCE_MODE = 0x01,
  DS_TC_FORCE_MODE_VALUE = 0x02,
  DS_HAZARD_FORCE_MODE = 0x03,
  DS_HAZARD_FORCE_MODE_VALUE = 0x04,
  DS_EXT_BTN_FORCE_MODE = 0x05,
  DS_EXT_BTN_FORCE_MODE_VALUE = 0x06,
  DS_FOLLOW_BRAKE = 0x07,
  DS_FOLLOW_HANDBRAKE = 0x08,
  DS_DISENGAGE_UNDER_SPEED = 0x09,
  DS_DISENGAGE_ABOVE_SPEED = 0x0A,
  DS_DISABLE_THROTTLE = 0x0B,
  DS_STEERING_SCALE_ENABLED = 0x0C,
  DS_LOCK_RELEASE_ENABLED = 0x0D,
  DS_LOCK_RELEASE_RATE = 0x0E,
  DS_LIVE_DIAG_ENABLED = 0x0F,
  DS_LED_BRIGHTNESS = 0x10,
};

#define DRIVING_SETTINGS_LAYOUT 1
#define DRIVING_SETTINGS_LEN 14

// Validates and applies one setting. Mode values >= 6, bools > 1 and unknown
// ids are rejected (false, nothing changed); speeds, throttle, release rate and
// LED brightness are clamped to their range, as the web API always did.
// Persisted by the periodic writeEEP task like every other setting.
bool applyDrivingSetting(uint8_t id, uint16_t value);

// Wire size of a setting's value: 1 or 2 bytes, 0 for an unknown id.
uint8_t drivingSettingValueLen(uint8_t id);

// The 14-byte Settings payload (layout in documents/MOBILE_APP_OPENHALDEX.md).
void buildDrivingSettings(uint8_t *out);
