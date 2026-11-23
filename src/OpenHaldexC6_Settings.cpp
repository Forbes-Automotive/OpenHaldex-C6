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
    if (value >= (uint16_t)openHaldexMode_t_MAX)
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
    if (value >= (uint16_t)openHaldexMode_t_MAX)
      return false;
    hazardForceModeValue = (uint8_t)value;
    return true;
  case DS_EXT_BTN_FORCE_MODE:
    if (!isBoolValue)
      return false;
    extBtnForceMode = on;
    return true;
  case DS_EXT_BTN_FORCE_MODE_VALUE:
    if (value >= (uint16_t)openHaldexMode_t_MAX)
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
    state.pedalThreshold = disableThrottle;
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

// ---- Vehicle geometry defaults per generation (V11.00.14) -----------------
// One common Haldex car per generation, factory wheels. Track moves 5-10 mm
// with wheel offset; 1-2 % is plenty for the slip maths. "est" = no published
// ratio: from lock-to-lock turns / the platform's usual value.
//   Gen1  Golf Mk4 R32           auto-data; ratio cars.com (3 turns)
//   Gen2  Golf Mk5 R32           auto-data; ratio est (Golf 6 R rack family)
//   Gen4  Golf Mk6 R             VW US 2012 Golf R spec sheet
//   Gen5  Audi TT 8S quattro     Audi TTS technical data (progressive rack, nominal 14.6)
//   0AY   Tiguan Mk2 4Motion     auto motor und sport / auto-data; ratio est (2.25 turns)
//   P2    Volvo XC90 Mk1         Volvo 2003 technical data; ratio est (2.7 turns)
//   P1    Volvo V50 AWD          Volvo Nov-2003 technical spec; ratio Volvo club spec pages
//   GM    Insignia A 2.0T 4x4    auto-data; ratio est
//   P3    Volvo XC60 Mk1         Volvo MY2017 specs; ratio ultimatespecs
//   SAAB  9-3 XWD                cars.com / guideautoweb (about 3 turns)
//   Ford  Kuga Mk1 AWD           cars-data / encycarpedia; ratio est
//   LR    Freelander 2           auto-data; ratio LR workshop text (2.6 turns). Evoque: 2660 / 1625 / 1630
//   SPA   Volvo XC90 Mk2         Volvo MY2022 specs
static const VehicleGeometry GEOMETRY_GEN1 = {2518, 1513, 1487, 16.0f, "VW Golf Mk4 R32"};
static const VehicleGeometry GEOMETRY_GEN2 = {2578, 1533, 1515, 15.6f, "VW Golf Mk5 R32"};
static const VehicleGeometry GEOMETRY_GEN4 = {2578, 1533, 1514, 15.6f, "VW Golf Mk6 R"};
static const VehicleGeometry GEOMETRY_GEN5 = {2505, 1568, 1547, 14.6f, "Audi TT Mk3 (8S)"};
static const VehicleGeometry GEOMETRY_0AY = {2677, 1576, 1566, 14.5f, "VW Tiguan Mk2"};
static const VehicleGeometry GEOMETRY_VOLVO_P2 = {2857, 1634, 1624, 16.0f, "Volvo XC90 Mk1"};
static const VehicleGeometry GEOMETRY_VOLVO_P1 = {2640, 1535, 1531, 15.4f, "Volvo V50 AWD"};
static const VehicleGeometry GEOMETRY_GM = {2737, 1585, 1587, 16.0f, "Vauxhall / Opel Insignia A 4x4"};
static const VehicleGeometry GEOMETRY_VOLVO_P3 = {2774, 1632, 1586, 16.0f, "Volvo XC60 Mk1"};
static const VehicleGeometry GEOMETRY_SAAB = {2675, 1524, 1506, 16.0f, "SAAB 9-3 XWD"};
static const VehicleGeometry GEOMETRY_KUGA = {2690, 1574, 1584, 15.0f, "Ford Kuga Mk1"};
static const VehicleGeometry GEOMETRY_LAND_ROVER = {2660, 1611, 1624, 16.7f, "Land Rover Freelander 2"};
static const VehicleGeometry GEOMETRY_VOLVO_SPA = {2984, 1665, 1667, 16.8f, "Volvo XC90 Mk2"};

// The single default every generation had before V11.00.14 (Audi TT Mk3): a
// unit still on these figures hasn't had its car typed in either.
static const VehicleGeometry GEOMETRY_LEGACY = {2505, 1572, 1543, 15.0f, "Audi TT Mk3"};

const VehicleGeometry &vehicleDefaultsForGeneration(uint8_t gen)
{
  switch (gen)
  {
  case 1:
    return GEOMETRY_GEN1;
  case 2:
    return GEOMETRY_GEN2;
  case 4:
    return GEOMETRY_GEN4;
  case 50:
  case 52:
    return GEOMETRY_GEN5;
  case 51:
    return GEOMETRY_0AY;
  case 21:
  case 3:
    return GEOMETRY_VOLVO_P2;
  case 31:
    return GEOMETRY_VOLVO_P1;
  case 41:
    return GEOMETRY_GM;
  case 43:
    return GEOMETRY_VOLVO_P3;
  case 44:
    return GEOMETRY_SAAB;
  case 45:
    return GEOMETRY_KUGA;
  case 46:
    return GEOMETRY_LAND_ROVER;
  case 53:
    return GEOMETRY_VOLVO_SPA;
  default:
    return GEOMETRY_GEN1;
  }
}

void applyVehicleDefaults(uint8_t gen)
{
  const VehicleGeometry &g = vehicleDefaultsForGeneration(gen);
  slipWheelbaseMm = g.wheelbaseMm;
  slipTrackFrontMm = g.trackFrontMm;
  slipTrackRearMm = g.trackRearMm;
  slipSteeringRatio = g.steeringRatio;
}

static bool geometryMatches(const VehicleGeometry &g)
{
  return slipWheelbaseMm == g.wheelbaseMm && slipTrackFrontMm == g.trackFrontMm &&
         slipTrackRearMm == g.trackRearMm && fabsf(slipSteeringRatio - g.steeringRatio) < 0.05f;
}

void vehicleGeometryFollowGeneration(uint8_t oldGen, uint8_t newGen)
{
  if (oldGen == newGen)
    return;
  if (geometryMatches(vehicleDefaultsForGeneration(oldGen)) || geometryMatches(GEOMETRY_LEGACY))
    applyVehicleDefaults(newGen);
}
