#include <OpenHaldexC6_Calculations.h>
#include <OpenHaldexC6_tasks.h>
#include <math.h> // tanf/sqrtf/fabsf for the per-corner slip geometry

// Geometry-compensated per-corner slip. Adopted from OpenHaldex-Edge by Rekt
// (Kile Thomson) - https://github.com/Kile-Thomson/OpenHaldex-Edge - see
// THIRD_PARTY_NOTICES.md. At steering road-wheel angle delta,
// pure Ackermann puts the turn centre on the rear-axle line R = wheelbase /
// tan(delta) from the centreline. Each wheel traces its own radius, so with
// zero real slip the wheel speeds are proportional to those radii. We build the
// four geometric radii, turn them into expected speeds sharing the measured
// mean, and report each corner's fractional excess as slip. On a straight this
// reduces to "speed vs. the average of the others" with no special-casing.
bool compute_corner_slip(const uint16_t wheel_raw[4], int16_t steer_wheel_tenths,
                         float steering_ratio, uint16_t wheelbase_mm,
                         uint16_t track_front_mm, uint16_t track_rear_mm,
                         uint16_t min_speed_raw, int8_t slip_out[4])
{
  slip_out[0] = slip_out[1] = slip_out[2] = slip_out[3] = 0;

  if (wheelbase_mm == 0 || track_front_mm == 0 || track_rear_mm == 0 ||
      steering_ratio < 1.0f)
    return false;

  const uint32_t sum_raw =
      (uint32_t)wheel_raw[0] + wheel_raw[1] + wheel_raw[2] + wheel_raw[3];
  const float mean_actual = sum_raw * 0.25f;
  if (mean_actual < (float)min_speed_raw || mean_actual <= 0.0f)
    return false;

  const float L = (float)wheelbase_mm;
  const float half_tf = (float)track_front_mm * 0.5f;
  const float half_tr = (float)track_rear_mm * 0.5f;

  const float delta_deg = ((float)steer_wheel_tenths * 0.1f) / steering_ratio;
  const float delta = fabsf(delta_deg) * (float)M_PI / 180.0f;

  float rad[4]; // [FL, FR, RL, RR]
  const float kStraight = 0.0005f;
  if (delta < kStraight)
  {
    rad[0] = rad[1] = rad[2] = rad[3] = 1.0f;
  }
  else
  {
    float R = L / tanf(delta);
    const float min_R = (half_tf > half_tr ? half_tf : half_tr) + 1.0f;
    if (R < min_R)
      R = min_R;

    const float r_rear_inner = R - half_tr;
    const float r_rear_outer = R + half_tr;
    const float r_front_inner = sqrtf(L * L + (R - half_tf) * (R - half_tf));
    const float r_front_outer = sqrtf(L * L + (R + half_tf) * (R + half_tf));

    if (delta_deg > 0.0f)
    {
      // Turning right: right-side wheels are inner (shorter radius, slower).
      rad[0] = r_front_outer; // FL
      rad[1] = r_front_inner; // FR
      rad[2] = r_rear_outer;  // RL
      rad[3] = r_rear_inner;  // RR
    }
    else
    {
      rad[0] = r_front_inner; // FL
      rad[1] = r_front_outer; // FR
      rad[2] = r_rear_inner;  // RL
      rad[3] = r_rear_outer;  // RR
    }
  }

  const float mean_rad = (rad[0] + rad[1] + rad[2] + rad[3]) * 0.25f;
  if (mean_rad <= 0.0f)
    return false;

  for (int i = 0; i < 4; i++)
  {
    const float expected = mean_actual * (rad[i] / mean_rad);
    if (expected <= 0.0f)
    {
      slip_out[i] = 0;
      continue;
    }
    float slip_pct = ((float)wheel_raw[i] / expected - 1.0f) * 100.0f;
    if (slip_pct > 127.0f)
      slip_pct = 127.0f;
    else if (slip_pct < -100.0f)
      slip_pct = -100.0f;
    slip_out[i] = (int8_t)(slip_pct < 0.0f ? slip_pct - 0.5f : slip_pct + 0.5f);
  }
  return true;
}

// Global lock gate: the "Disengage Under/Above Speed" and "Minimum Throttle
// Before Lock" settings. Applies to EVERY lock-producing path - the selected
// mode (50:50 / 60:40 / 75:25 / Expert) AND any force-mode trigger (TC,
// hazards, external button) - so a forced 50:50 in a car park still obeys the
// under-speed cut-off. A bound of 0 means that side of the window is disabled.
// Expert mode used to bypass this entirely (its map has its own speed axis);
// it is now gated too, matching the UI hint "disable ANY lock below...".
//
// lock_gate_blocking() says WHICH of the three settings is holding lock off
// right now (LOCK_GATE_* bits, 0 = none), so the dashboard can name it.
// lock_enabled() is built on it, so the reason shown can never disagree with
// what the lock maths actually did. Same tests as before (V10.00.2, from PR #54
// by Luca C: a Gen5 0AY owner saw "FWD in every mode" with only "Target: 0%"
// on screen, caused by an old Disengage Under Speed setting).
uint8_t lock_gate_blocking()
{
  uint8_t blocking = 0; // no gate blocking yet

  // Minimum Throttle Before Lock: 0 = off, otherwise throttle must reach it
  if (state.pedal_threshold != 0 && int(received_pedal_value) < state.pedal_threshold)
  {
    blocking = blocking | LOCK_GATE_THROTTLE;
  }

  // Disengage Under Speed: 0 = off, otherwise no lock below it (car parks)
  if (disengageUnderSpeed != 0 && received_vehicle_speed < disengageUnderSpeed)
  {
    blocking = blocking | LOCK_GATE_UNDER_SPEED;
  }

  // Disengage Above Speed: 0 = off, otherwise no lock above it (motorways)
  if (disengageAboveSpeed != 0 && received_vehicle_speed > disengageAboveSpeed)
  {
    blocking = blocking | LOCK_GATE_ABOVE_SPEED;
  }

  // Follow Handbrake (V11.00.0): handbrake pulled -> clutch open, as the Haldex
  // does on its own. Needed because our lock request outranks the unit's own
  // brake + handbrake handling on the direct-lever units.
  if (handbrakeForHaldex())
  {
    blocking = blocking | LOCK_GATE_HANDBRAKE;
  }

  // Follow ABS (V11.00.0, default on): ABS braking -> clutch open, the Haldex's own ABS protection
  if (absBrakingForHaldex())
  {
    blocking = blocking | LOCK_GATE_ABS;
  }

  return blocking;
}

// True when none of the three settings is holding lock off.
static inline bool lock_enabled()
{
  return (lock_gate_blocking() == 0);
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

// Steering-angle third axis (FWD bias): returns a 0-100% multiplier for the
// lock target based on |steering-wheel angle|. Max lock at low angle, reduced
// lock as angle grows. Only gens with a steering source (2/4/50/52) use it;
// unsupported gens or stale/absent steering data return 100 (no reduction).
static float get_steering_lock_scale()
{
  const bool supported = (haldexGeneration == 2 || haldexGeneration == 4 ||
                          haldexGeneration == 50 || haldexGeneration == 52);
  if (!supported)
    return 100.0f;
  if (!steeringScaleEnabled)
    return 100.0f; // feature disabled -> full lock, no reduction
  if (received_steering_ms == 0 || (millis() - received_steering_ms) > steeringStaleMs)
    return 100.0f; // no/stale steering -> full lock, bias off

  float angle = fabsf(received_steering_angle);
  angle = constrain(angle, 0, (float)steeringArray[steeringArrayCount - 1]);

  if (angle >= steeringArray[steeringArrayCount - 1])
    return constrain((float)steeringLockScaleArray[steeringArrayCount - 1], 0, 100);

  for (uint8_t i = 0; i < steeringArrayCount - 1; i++)
  {
    if (angle <= steeringArray[i + 1])
    {
      const float denom = (float)steeringArray[i + 1] - (float)steeringArray[i];
      const float ratio = (denom > 0) ? ((angle - steeringArray[i]) / denom) : 0;
      const float v0 = steeringLockScaleArray[i];
      const float v1 = steeringLockScaleArray[i + 1];
      return constrain(v0 + ((v1 - v0) * ratio), 0, 100);
    }
  }
  return 100.0f;
}

// Scale a lock target by the steering-angle curve. Applied to lock-producing
// modes only (not Stock passthrough or FWD).
// Telemetry captured for the UI engagement-split display: last requested
// (pre-scale) and applied (post-scale) lock, and whether scaling ran this cycle.
static float s_steer_requested = 0.0f;
static float s_steer_applied = 0.0f;
static bool s_steer_applied_flag = false;

static inline float apply_steering_scale(float lock)
{
  const float scaled = lock * get_steering_lock_scale() / 100.0f;
  s_steer_requested = lock;
  s_steer_applied = scaled;
  s_steer_applied_flag = true;
  return scaled;
}

// Steering-scale telemetry accessors (used by the API live-status endpoint).
bool steering_scale_is_active()
{
  return s_steer_applied_flag && (s_steer_applied + 0.5f < s_steer_requested);
}
uint8_t steering_scale_requested_pct()
{
  return (uint8_t)constrain((int)(s_steer_requested + 0.5f), 0, 100);
}
uint8_t steering_scale_result_pct()
{
  return (uint8_t)constrain((int)(s_steer_applied + 0.5f), 0, 100);
}

// The mode the car's drive mode select (MQB Charisma) is forcing, or -1 for none.
// The drive profile is read from Charisma_07 / Charisma_01 in parseCAN_chs.
int get_drive_mode_forced_value()
{
  if (!driveModeForce)
  {
    return -1; // feature off
  }
  // Copy the time first, then read the clock: the CAN task can update lastDriveProfileMs at
  // any moment, and reading it after millis() could make "now - then" wrap round to a huge number.
  uint32_t lastReadMs = lastDriveProfileMs; // when the drive profile was last read
  uint32_t nowMs = millis();                // time now
  if (lastReadMs == 0 || (nowMs - lastReadMs) > DRIVE_MODE_TIMEOUT_MS)
  {
    return -1; // no drive profile read recently (not an MQB car, or the gateway has stopped sending)
  }
  if (receivedDriveProfile >= DRIVE_PROFILE_COUNT)
  {
    return -1; // a profile number we don't have a table entry for
  }

  uint8_t forcedMode = driveModeForceValue[receivedDriveProfile]; // the mode set for this profile in the table
  if (forcedMode >= (uint8_t)openhaldex_mode_t_MAX)
  {
    return -1; // N/A (or a bad saved value): leave the mode alone
  }
  return forcedMode;
}

// Which force-mode value applies right now: 0..5 (Stock/FWD/5050/6040/7525/Expert)
// when an enabled trigger's flag is active, or -1 when no force mode applies.
// Priority between simultaneous triggers is user-configurable via
// forceModesPriority (0-5 covering all 6 orderings of TC/Hazards/External).
// The drive mode select comes after all three: it's a standing choice, while
// the others are something the driver does on purpose right now.
// Exposed so the inline gateway (parseCAN_chs) can detect "effective mode is
// Stock" and pass frames through untouched instead of editing them.
int get_forced_mode_value()
{
  if (!(extBtnForceMode || tcForceMode || hazardForceMode))
  {
    return get_drive_mode_forced_value(); // no button triggers turned on: only the drive mode can force
  }

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

  const uint8_t priority = (forceModesPriority < 6) ? forceModesPriority : 0;

  for (uint8_t i = 0; i < 3; i++)
  {
    const uint8_t idx = priorityOrders[priority][i];
    if (triggers[idx].enabled && triggers[idx].flag)
    {
      return triggers[idx].value;
    }
  }
  return get_drive_mode_forced_value(); // no button trigger active: the drive mode select comes last
}

float get_lock_target_adjustment()
{
  s_steer_applied_flag = false; // reset; apply_steering_scale sets it when used this cycle

  // The return value is a torque COMMAND: standalone packs it into the frames it
  // synthesizes, and the inline gateway packs it into edited chassis frames. It
  // must never mirror received_haldex_engagement back - engagement 100
  // commanding 100 closes a positive feedback loop that latches the clutch shut
  // until FWD or a power cycle. Stock as a command is zero forced lock; true
  // stock PASSTHROUGH is the caller's job (parseCAN_chs skips frame edits
  // entirely when the effective mode is Stock).
  // const MODE_NAMES = ['Stock', 'FWD', '50:50', '60:40', '75:25', 'Expert'];
  const int forced = get_forced_mode_value();
  if (forced >= 0)
  {
    // Forced lock modes go through the same speed/throttle gate as the
    // selected mode below, so lock_target (and the dashboard "Requested"
    // figure) never reads 100% below the under-speed cut-off.
    switch (forced)
    {
    case 0:
      return 0; // Stock -> no forced lock (passthrough is handled by the caller)
    case 1:
      return 0; // FWD
    case 2:
      return lock_enabled() ? apply_steering_scale(100) : 0; // 50:50
    case 3:
      return lock_enabled() ? apply_steering_scale(40) : 0; // 60:40
    case 4:
      return lock_enabled() ? apply_steering_scale(30) : 0; // 75:25
    case 5:
      return lock_enabled() ? apply_steering_scale(get_expert_lock_target()) : 0; // Expert
    default:
      return 0; // error - zero lock
    }
  }

  // getting here means no external influences - handle each mode and calculate lock
  switch (state.mode)
  {
  case MODE_STOCK:
    return 0; // Stock -> no forced lock. Standalone has nothing to pass through, so Stock is an open clutch; inline Stock never reaches here (parseCAN_chs forwards frames untouched).

  case MODE_FWD:
    return 0; // zero lock

  case MODE_5050:
    if (lock_enabled())
    {
      return apply_steering_scale(100); // 100% lock
    }
    return 0; // lock not enabled, zero lock

  case MODE_6040:
    if (lock_enabled())
    {
      return apply_steering_scale(40); // 40% lock
    }
    return 0; // lock not enabled, zero lock

  case MODE_7525:
    if (lock_enabled())
    {
      return apply_steering_scale(30); // 30% lock
    }
    return 0; // lock not enabled, zero lock

  case MODE_EXPERT:
    if (lock_enabled())
    {
      return apply_steering_scale(get_expert_lock_target());
    }
    return 0; // lock not enabled, zero lock

  default:
    return 0; // error - zero lock
  }
}

// Look up the learn table: the smallest correction factor (0-100) whose learned
// engagement reaches the wanted engagement. If nothing the sweep learned gets
// there, use the entry that gave the most engagement.
static uint8_t learn_table_correction_factor(uint8_t wantedEngagement)
{
  for (uint8_t i = 0; i <= 100; i++)
  {
    if (haldexLearnTable[i] >= wantedEngagement)
    {
      return i; // first step that reaches it
    }
  }

  // requested lock exceeds anything the sweep learned - clamp to the highest learned entry
  uint8_t correctionFactor = 0;
  uint8_t bestEngagement = 0;
  for (uint8_t i = 0; i <= 100; i++)
  {
    if (haldexLearnTable[i] >= bestEngagement)
    {
      bestEngagement = haldexLearnTable[i];
      correctionFactor = i;
    }
  }
  return correctionFactor;
}

// ===== Direct-lever units (V11.00.0) =====
// The OEM code (documents/haldex-binaries/CONTROL_AND_DTC_SPEC.md) shows most
// VAG units take a straight clutch-torque request in one byte - Bremse_4
// (0x2A0) byte 0 on Gen1 / Gen2 / Gen4 / 0AY, ESP_14 (0x08A) bytes 6-7 on 0CQ.
// Before V11 we scaled 0x7F by a correction factor, which lands BELOW 0x7F
// ("invalid" - the unit ignores it and runs its own strategy) and at 100 %
// sent 0x7F itself ("zero request" - clutch OPEN when full lock was asked for).

// How much lock (0-100 %) to ask a direct-lever unit for right now.
// Learn: the sweep's correction factor IS the lever position, so the learn
// table measures the lever directly. Otherwise lock_target, corrected through
// the learn table if one has been saved. 0 whenever the lock gate is closed
// (speed / throttle settings, handbrake, ABS braking).
uint8_t lever_lock_percent()
{
  if (haldexLearnActive)
  {
    return haldexLearnCF; // learn sweeps the lever itself, 0..100
  }

  if (lock_target <= 0)
  {
    return 0; // FWD / Stock / nothing asked for
  }

  if (!lock_enabled())
  {
    return 0; // a lock gate is holding lock off
  }

  if (haldexLearnTableValid)
  {
    return learn_table_correction_factor((uint8_t)lock_target); // measured lever -> engagement curve
  }

  if (lock_target >= 100)
  {
    return 100;
  }
  return (uint8_t)lock_target; // the lever is linear, so no default fudge curve is needed
}

// Turn a lock % into the Bremse_4 byte 0 lever value used by the PQ units:
//   0 %       -> 0x7F  valid "no torque" request, the clutch is commanded fully open
//   1 - 99 %  -> 0x80 + (percent x 125 / 100), so 0x81 .. 0xFC (linear torque request)
//   100 %     -> fullLockByte (0xFE on Gen4 = "present max capacity", 0xFD on 0AY
//                where 0xFE means HOLD - each generation passes its own)
// Never 0x00-0x7E or 0xFF: those are "invalid" and the unit ignores the byte.
uint8_t vag_lever_byte(uint8_t fullLockByte)
{
  uint8_t lockPercent = lever_lock_percent();

  if (lockPercent == 0)
  {
    return 0x7F; // valid zero request: clutch open
  }
  if (lockPercent >= 100)
  {
    return fullLockByte; // top of the range for this unit
  }

  uint16_t step = ((uint16_t)lockPercent * 125 + 50) / 100; // 1..99 % -> 1..124 steps (rounded)
  return (uint8_t)(0x80 + step);
}

// ===== Brake / handbrake / ABS for the Haldex (V11.00.0) =====
// "Follow Brake" and "Follow Handbrake" (Output Options) decide whether the
// Haldex gets to see the brake pedal and the handbrake. In OEM mode the car's
// own frames already carry them (and our frame edits must not wipe them); in
// standalone mode the frame builders put them in with these two functions.

// True when Follow Brake is on and the brake pedal is pressed. Sources: the
// brake input wire (brakeActive - IO.cpp sets it from the wire with "Invert
// Brake Input" applied) or the car's CAN (brakeFromCAN).
bool brakeForHaldex()
{
  if (!followBrake)
  {
    return false; // Follow Brake off: the Haldex is not told about the brake
  }
  if (brakeActive)
  {
    return true; // brake input wire
  }
  if (hasCANChassis && brakeFromCAN)
  {
    return true; // car's CAN (Motor_2 / ESP_05 ...)
  }
  return false;
}

// True when Follow Handbrake is on and the handbrake is pulled. Same two sources
// as the brake: the handbrake input wire or the car's CAN (Kombi_1 / Kombi_01 ...).
bool handbrakeForHaldex()
{
  if (!followHandbrake)
  {
    return false; // Follow Handbrake off
  }
  if (handbrakeActive)
  {
    return true; // handbrake input wire
  }
  if (hasCANChassis && handbrakeFromCAN)
  {
    return true; // car's CAN
  }
  return false;
}

// True when "Follow ABS" is on and the ABS module says it is braking. Every
// Haldex opens its clutch for this on its own, but our lock request outranks
// the unit's own ABS handling (OEM code, Gen4 0BR: "any valid Bremse_4 byte
// wins"), so we do the release ourselves. Follow ABS is on by default; turning
// it off lets our lock overrule the ABS (and standalone sends "not braking").
bool absBrakingForHaldex()
{
  if (!followABS)
  {
    return false; // Follow ABS off: our lock overrules the ABS
  }
  if (hasCANChassis && absBrakingFromCAN)
  {
    return true;
  }
  return false;
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
    correction_factor = learn_table_correction_factor((uint8_t)lock_target);
  }
  else if (haldexGeneration == 41)
  {
    // Gen41 (Vauxhall/Opel/Buick FDCM) defaults — observed mapping CF→engagement is
    // steeper than the VAG Haldex curve.
    // Linear fit: engagement = 2.2 * CF - 24  →  CF = (target + 24) / 2.2
    correction_factor = (uint8_t)constrain(((float)lock_target + 24.0f) / 2.2f, 0, 100);
  }
  else if (isVolvoOrSaabGeneration(haldexGeneration))
  {
    // Volvo / SAAB (V10.00.0): no measured curve yet, so straight through -
    // 100 % lock sends the strongest known lock value. Run Learn
    // Haldex once a unit is on the bench to replace this with a real table.
    correction_factor = (uint8_t)constrain((float)lock_target, 0, 100);
  }
  else
  {
    // VAG Haldex default — linear fit: engagement = 2 * CF - 20  →  CF = (target + 20) / 2
    correction_factor = (uint8_t)constrain(((float)lock_target + 20.0f) / 2.0f, 0, 100);
  }

  uint8_t corrected_value = (uint16_t)value * correction_factor / 100;
  if (lock_enabled())
  {
    return (invert ? (0xFE - corrected_value) : corrected_value); // if lock enabled, return corrected value (or inverted), otherwise return 0 (or inverted)
  }
  return (invert ? 0xFE : 0x00); // if lock not enabled, return 0 (or inverted)
}

void fill_esp19_wheel_speeds(uint8_t data[8])
{
  // Wheel speed MUST keep changing or the Haldex slowly disengages - a
  // static value fades over time and eventually stops. This is the original,
  // proven keep-alive dither (all 4 corners share the same small swing).
  //
  // A lock_target-proportional front/rear delta (simulating real slip, like
  // the Gen42/Ford wheel-speed code does) was tried here and made things
  // worse on real hardware - reported lock still faded from 100% and then
  // collapsed to 0%, rather than holding. Likely reading to the Haldex as
  // excessive/implausible sustained slip and triggering a separate
  // protection cutoff. Reverted; do not reintroduce without confirming on
  // the car first.
  if (wsBaseRaw == 0)
  {
    // Legacy behaviour: all four corners driven from the free-running counters,
    // scaled by the lock (0 km/h at 0 %, ~20-30 km/h at 100 %) - the bench-proven
    // wheel-speed method. V11.00.0 (Adam): kept ON alongside the ESP_14 lever in
    // every non-Stock mode. The OEM code (spec_gen5_0cq D6) shows the 0CQ's own
    // strategy reads these, so the unit never sees real slip while forced; Stock
    // passes the car's real speeds, and the ABS / handbrake gates drop the lock
    // (and so these speeds) to 0.
    const uint8_t hlLo = get_lock_target_adjusted_value(ESP_19_counter2, false);        // rear wheel speed low byte
    const uint8_t hlHi = get_lock_target_adjusted_value(ESP_19_counter, false);         // all corners high byte
    const uint8_t vlLo = get_lock_target_adjusted_value(ESP_19_counter2 + 0xBA, false); // front wheel speed low byte

    data[0] = hlLo; // HL (rear left) low
    data[1] = hlHi; // HL (rear left) high
    data[2] = hlLo; // HR (rear right) low
    data[3] = hlHi; // HR (rear right) high
    data[4] = vlLo; // VL (front left) low
    data[5] = hlHi; // VL (front left) high
    data[6] = vlLo; // VR (front right) low
    data[7] = hlHi; // VR (front right) high
    if (wsLeftRightDeltaRaw != 0)
    {
      // VAQ lever: split the front axle left vs right around the legacy value.
      const int32_t base = (int32_t)((uint16_t)hlHi << 8 | vlLo);
      int32_t vl = base + wsLeftRightDeltaRaw / 2;
      int32_t vr = base - wsLeftRightDeltaRaw / 2;
      if (vl < 0) vl = 0; if (vl > 0xFFFF) vl = 0xFFFF;
      if (vr < 0) vr = 0; if (vr > 0xFFFF) vr = 0xFFFF;
      data[4] = (uint8_t)(vl & 0xFF); data[5] = (uint8_t)(vl >> 8);
      data[6] = (uint8_t)(vr & 0xFF); data[7] = (uint8_t)(vr >> 8);
    }
  }
  else
  {
    // Explicit mode: a fixed base speed per corner, optionally dithered, with
    // an optional front-axle offset. Lets the "does the Haldex hunt because
    // the simulated wheel speed keeps moving?" question be tested directly -
    // set wsDitherRaw 0 for a genuinely static speed.
    static bool phase = false;
    phase = !phase;
    const int32_t dither = wsDitherRaw ? (phase ? (int32_t)wsDitherRaw : -(int32_t)wsDitherRaw) : 0;
    int32_t rear = (int32_t)wsBaseRaw + dither;
    int32_t front = rear + wsFrontDeltaRaw;
    // VAQ lever: front left-vs-right split (a transverse lock reacts to VL vs VR).
    int32_t vl = front + wsLeftRightDeltaRaw / 2;
    int32_t vr = front - wsLeftRightDeltaRaw / 2;
    if (rear < 0) rear = 0;
    if (vl < 0) vl = 0;
    if (vr < 0) vr = 0;
    const uint16_t r = (uint16_t)(rear > 0xFFFF ? 0xFFFF : rear);
    const uint16_t l16 = (uint16_t)(vl > 0xFFFF ? 0xFFFF : vl);
    const uint16_t r16 = (uint16_t)(vr > 0xFFFF ? 0xFFFF : vr);

    data[0] = (uint8_t)(r & 0xFF);   // HL low
    data[1] = (uint8_t)(r >> 8);     // HL high
    data[2] = (uint8_t)(r & 0xFF);   // HR low
    data[3] = (uint8_t)(r >> 8);     // HR high
    data[4] = (uint8_t)(l16 & 0xFF); // VL low
    data[5] = (uint8_t)(l16 >> 8);   // VL high
    data[6] = (uint8_t)(r16 & 0xFF); // VR low
    data[7] = (uint8_t)(r16 >> 8);   // VR high
  }

  if (!wsFreeze)
  {
    ESP_19_counter++;
    ESP_19_counter2++;
    if (ESP_19_counter > 0x10)
      ESP_19_counter = 0x0A;
    if (ESP_19_counter2 > 0x2F)
      ESP_19_counter2 = 0x2E;
  }
}

bool dangerZoneActive()
{
  return dangerZoneEnabled && !haldexLearnActive && lock_target > 99.0f;
}

void fill_motor11_bpk(uint8_t data[8], uint8_t counter)
{
  // DBC-correct bit packing for Motor_11 (0x0A7). Every field below is a
  // runtime tunable (see defs.h) so the serial lab can massage the wire
  // format live; the defaults are the values that were hardcoded here.
  // Signals are 10-bit with offset -509, i.e. raw = Nm + 509.
  //
  // Danger Zone: the pump duty on the 0CQ follows the torque this frame
  // claims, not ESP_14 (bench 2026-09-20: 220 Nm -> 58 % / 9 A, 320 Nm ->
  // 90 % / 11 A, 400 Nm -> 95 % / 11.5 A, relief valve pegged, reported
  // engagement in the 80s). At a full lock request, raise the ceiling to
  // dangerZoneNm unless the user's own calibration is already higher.
  // V11.00.0 (OEM code, spec_gen5_0cq D1): the pump only followed Motor_11 in
  // that test because full lock sent ESP_14 = 0xFE, which the 0CQ treats as "no
  // ESP request" and falls back to its own Motor_11 strategy. With the V11 lever
  // (Min = Max = 0..250) the 0CQ follows ESP_14 and this torque is not used for
  // the target - it only feeds the unit's clutch-load model.
  uint16_t ceilNm = bpkCeilingNm;
  if (dangerZoneActive() && dangerZoneNm > ceilNm)
    ceilNm = dangerZoneNm;
  const uint16_t floorNm = (bpkFloorNm < ceilNm) ? bpkFloorNm : 0;

  uint16_t torqueNm = get_lock_target_adjusted_value(0xFE, false);
  torqueNm = (uint16_t)(floorNm + ((uint32_t)torqueNm * (ceilNm - floorNm)) / 0xFE);

  static uint16_t prevIstNm = 0, prevSolfNm = 0;
  auto slew = [](uint16_t cur, uint16_t target, uint16_t step) -> uint16_t
  {
    if (step == 0)
      return target; // 0 = no rate limit
    if (target > cur)
      return ((uint32_t)cur + step >= target) ? target : (uint16_t)(cur + step);
    if (target < cur)
      return (cur <= step || cur - step <= target) ? target : (uint16_t)(cur - step);
    return cur;
  };
  uint16_t istNm = slew(prevIstNm, torqueNm, bpkSlewIst);
  uint16_t solfNm = slew(prevSolfNm, torqueNm, bpkSlewSolf);
  prevIstNm = istNm;
  prevSolfNm = solfNm;

  // Optional overrides, to test which field the Haldex actually keys off.
  if (bpkForceIstNm >= 0)
    istNm = (uint16_t)bpkForceIstNm;
  if (bpkForceSolfNm >= 0)
    solfNm = (uint16_t)bpkForceSolfNm;

  bpkLogSample(torqueNm, istNm, solfNm);

  const uint16_t rawSollRoh = (uint16_t)(torqueNm + 509) & 0x3FF;
  const uint16_t rawIst = (uint16_t)(istNm + 509) & 0x3FF;
  const uint16_t rawSolf = (uint16_t)(solfNm + 509) & 0x3FF;
  const uint16_t rawTraeg = bpkTraegRaw & 0x3FF;
  const uint16_t rawSchub = bpkSchubRaw & 0x1FF;

  data[0] = 0x00; // CRC placeholder - caller fills it
  data[1] = (counter & 0x0F) | ((rawSollRoh & 0x000F) << 4);
  data[2] = ((rawSollRoh >> 4) & 0x3F) | ((rawIst & 0x0003) << 6);
  data[3] = (rawIst >> 2) & 0xFF;
  data[4] = rawTraeg & 0xFF;
  data[5] = ((rawTraeg >> 8) & 0x03) | ((rawSolf & 0x3F) << 2);
  data[6] = ((rawSolf >> 6) & 0x0F) | ((rawSchub & 0x0F) << 4);
  data[7] = ((rawSchub >> 4) & 0x1F) | bpkStatusFl;

  for (uint8_t i = 0; i < 8; i++)
    bpkLastFrame[i] = data[i];
}

void bpkLogSample(uint16_t torqueNm, uint16_t istNm, uint16_t solfNm)
{
  // Runs at the Motor_11 rate (~100 Hz) on both BPK paths, so it stays a
  // plain store - the serial lab task does its own timing when it streams.
  bpkLastTorqueNm = torqueNm;
  bpkLastIstNm = istNm;
  bpkLastSolfNm = solfNm;
}

// Gen5 0CQ (V11.00.0): the ESP_14 lever value for bytes 6 and 7
// (BR_Vorg_Allrad_Min / BR_Vorg_Allrad_Max). OEM code (spec_gen5_0cq §2.2):
// sending Min = Max = V puts the 0CQ in its "exact" mode and the clutch target
// becomes V x 128, which is the same 0-250 scale (0.4 % per step) the unit
// reports back in Allrad_03 byte 2. 0 = clutch open, 250 (0xFA) = 100 %.
// 251 and above are NOT requests: the 0CQ then ignores ESP_14 and runs its own
// Motor_11 strategy, which is why the old 0..0xFE scale lost control at full
// lock (spec D1). So this never returns more than 250.
uint8_t gen5_esp14_lever_byte()
{
  uint16_t leverValue = (uint16_t)lever_lock_percent() * 250; // 0..25000
  leverValue = (leverValue + 50) / 100;                        // 0..250, rounded to the nearest step
  if (leverValue > 250)
  {
    leverValue = 250; // top of the valid range (0xFA)
  }
  return (uint8_t)leverValue;
}

void startHaldexLearn()
{
  if (haldexLearnActive || longLearnActive)
  {
    return; // already running (Long Learn drives its own sweeps)
  }

  memset(haldexLearnTable, 0, sizeof(haldexLearnTable));
  haldexLearnCancel = false;
  haldexLearnStep = 0;
  haldexLearnCF = 0;
  haldexLearnActive = true;

  xTaskCreate(haldexLearnTask, "haldexLearn", 4096, nullptr, 1, nullptr);
}

bool runLearnSweep(uint32_t preHoldMs)
{
  // Bench-measured (Gen5 0CQ): engagement needs several hundred ms to settle
  // after a step, and a single instantaneous sample lands mid-transient - which
  // is what made learned mid-points wander. Settle first, then AVERAGE over a
  // short observation window, so each entry is the value actually held rather
  // than whatever the reading was passing through. Held steady, this hardware
  // tracks the request 1:1 with no jitter at all, so a clean sweep should come
  // out close to an identity table.
  uint32_t settleMs = 400;
  if (haldexGeneration == GEN_VOLVO_P1)
  {
    // Volvo P1 (V11.00.1): engagement trails the torque we send by 100-400 ms
    // in the logs, so give each step longer before reading it
    settleMs = VOLVO_P1_LEARN_SETTLE_MS;
  }
  const uint32_t observeMs = 200;
  const uint32_t sampleMs = 50;
  uint8_t peak = 0; // highest engagement recorded so far (monotonic hold)

  // The ESP_14 launch floor pins BR_Vorg_*_Min to Max, leaving the Haldex no
  // room to modulate - it drives the pump to full duty and corrupts the top of
  // the sweep. Always learn with it at 0 and restore afterwards. NOTE: the
  // Motor_11 packing (fixHunting) is deliberately NOT forced here: which
  // packing a unit needs is a per-unit trait (554K needs BPK, this 0CQ does
  // not), and the table must describe how the car will actually be driven.
  const uint8_t floorBeforeLearn = esp14MinFloorPct;
  esp14MinFloorPct = 0;

  memset(haldexLearnTable, 0, sizeof(haldexLearnTable));
  haldexLearnCancel = false;
  haldexLearnStep = 0;
  haldexLearnCF = 0;
  haldexLearnActive = true; // frames now carry haldexLearnCF regardless of mode

  // Optional pre-hold at CF 0: wait for the clutch to release from the previous
  // sweep (engagement <= 2 % for five consecutive 100 ms ticks) or time out.
  if (preHoldMs > 0)
  {
    uint8_t releasedTicks = 0;
    for (uint32_t held = 0; held < preHoldMs && !haldexLearnCancel; held += 100)
    {
      vTaskDelay(100 / portTICK_PERIOD_MS);
      if (received_haldex_engagement <= 2)
      {
        if (++releasedTicks >= 5)
          break;
      }
      else
      {
        releasedTicks = 0;
      }
    }
  }

  for (uint16_t cf = 0; cf <= 100; cf++)
  {
    if (haldexLearnCancel)
    {
      break;
    }

    haldexLearnStep = (uint8_t)cf;
    haldexLearnCF = (uint8_t)cf;

    vTaskDelay(settleMs / portTICK_PERIOD_MS);

    // Average the settled window rather than taking one instantaneous read.
    uint32_t sum = 0;
    uint16_t n = 0;
    for (uint32_t o = 0; o < observeMs && !haldexLearnCancel; o += sampleMs)
    {
      vTaskDelay(sampleMs / portTICK_PERIOD_MS);
      sum += received_haldex_engagement;
      n++;
    }
    uint8_t eng = n ? (uint8_t)(sum / n) : received_haldex_engagement;

    // Engagement must rise (or plateau) as the requested lock climbs - it can
    // never physically fall. A reading below the running peak is a data fault
    // (e.g. a bad byte at the top end that returns 0 or a lower value), so hold
    // the highest lock achieved so far instead of saving the drop. This keeps
    // "the last available highest lock" as the learned value for higher requests.
    if (eng < peak)
    {
      eng = peak; // last available highest lock remains
    }
    else
    {
      peak = eng;
    }

    haldexLearnTable[cf] = eng;
  }

  bool anyNonZero = false;
  if (!haldexLearnCancel)
  {
    // only mark valid if at least one non-zero engagement was recorded
    for (uint8_t i = 0; i <= 100; i++)
    {
      if (haldexLearnTable[i] > 0)
      {
        anyNonZero = true;
        break;
      }
    }
    haldexLearnTableValid = anyNonZero;
    haldexLearnStep = anyNonZero ? 101 : 102; // 101 = complete OK, 102 = complete but no data
  }

  const bool ok = !haldexLearnCancel && anyNonZero;
  haldexLearnActive = false;
  haldexLearnCF = 0;
  esp14MinFloorPct = floorBeforeLearn; // restore whatever the user had set
  return ok;
}

void scoreLearnTable(const uint8_t *table, LearnScore &out)
{
  out.reach = table[100];
  out.engageCF = 101;
  for (uint8_t i = 0; i <= 100; i++)
  {
    if (table[i] > 0)
    {
      out.engageCF = i;
      break;
    }
  }
  out.engageJump = (out.engageCF <= 100) ? table[out.engageCF] : 0;

  // Largest rise between consecutive steps after the first engage step. The
  // table is monotonic (runLearnSweep holds the peak) so the delta is >= 0.
  out.maxStep = 0;
  for (uint16_t i = (uint16_t)out.engageCF + 1; i <= 100; i++)
  {
    const uint8_t d = table[i] - table[i - 1];
    if (d > out.maxStep)
      out.maxStep = d;
  }

  out.smooth = (out.engageCF <= LL_ENGAGE_MAX_CF) &&
               (out.reach >= LL_REACH_MIN) &&
               (out.maxStep <= LL_STEP_MAX);

  // Composite rank: start from reach, penalise discontinuities hard and late
  // engagement gently. A table that never engages scores 0.
  int s = out.reach;
  if (out.maxStep > 3)
    s -= 4 * (out.maxStep - 3);
  if (out.engageCF > 20)
    s -= (out.engageCF - 20) / 2;
  if (out.engageCF > 100)
    s = 0;
  out.score = (uint8_t)constrain(s, 0, 100);
}

bool startLongLearn(bool testAll)
{
  if (longLearnActive || haldexLearnActive)
    return false;
  const int gi = frameEditGenIdx(haldexGeneration);
  if (gi < 0)
    return false; // gen41/42 etc. have no gated blocks to bisect

  longLearnGenIdx = (uint8_t)gi;
  longLearnGeneration = haldexGeneration;
  longLearnTestAll = testAll;
  longLearnCancel = false;
  longLearnPhase = LL_SWEEP;
  longLearnSweepIdx = 0;
  longLearnSweepTotal = 0;
  longLearnSweepCount = 0;
  longLearnCurrentBit = -1;
  longLearnBaselineValid = false;
  longLearnFinalValid = false;
  longLearnBpkAdjusted = false;
  longLearnFailReason = LLF_NONE;
  longLearnNoise = 0;
  longLearnTol = LL_TOLERANCE;
  longLearnInteraction = false;
  longLearnStartMs = millis();
  longLearnEndMs = 0;
  memset(longLearnBlockResult, 0, sizeof(longLearnBlockResult));
  longLearnActive = true;

  xTaskCreate(longLearnTask, "longLearn", 6144, nullptr, 1, nullptr);
  return true;
}

// ===== Gen2 (7H0907554 / 1K0907554B) demand bytes (V11.00.0) =====
// OEM code (documents/haldex-binaries/leaf_trace/spec_gen2_7h0.md §2.3): Bremse_4
// (0x2A0) byte 0 is a CEILING on the clutch request - it can only pull the
// request DOWN. So the rest of the bus has to ask for at least as much lock as
// the lever: Motor_1 inner torque (b1) and pedal (b5) at maximum, loss torque
// (b6) at 0, and in standalone a little front-wheel slip on Bremse_3. Then the
// lever byte alone sets how much the clutch closes (0x7F = open .. 0xFD = full).
// 1K0 note: same frames and same lever; the 1K0 only uses 0x540 instead of
// 0x440 as its final-request scale inside the unit, so no separate code.
#define GEN2_BREMSE4_BLOCK_BIT 5      // Bremse_4 (0x2A0) bit in the Gen2 frameEditBlocks rows
#define GEN2_BREMSE4_SEEN_MS 500      // car's 0x2A0 counts as "present" if seen this recently (OEM mode)
static uint32_t gen2LastBremse4Ms = 0; // millis() when the car's Bremse_4 last passed through our edit (OEM mode). static = only this file uses it

// True when the Haldex is getting our Bremse_4 lever byte right now: the
// Bremse_4 block is on, and either we build the frame (standalone) or the car's
// own 0x2A0 is passing through us to be edited (OEM mode).
static bool gen2LeverOnBus()
{
  if (!frameEditEnabled(FE_GEN_2, GEN2_BREMSE4_BLOCK_BIT))
  {
    return false; // Bremse_4 block turned off - no lever
  }
  if (isStandalone)
  {
    return true; // standalone builds Bremse_4 itself
  }
  if (gen2LastBremse4Ms == 0)
  {
    return false; // car has not sent a 0x2A0 yet
  }
  if ((millis() - gen2LastBremse4Ms) > GEN2_BREMSE4_SEEN_MS)
  {
    return false; // car's 0x2A0 has stopped
  }
  return true;
}

// Value for one Gen2 "demand" byte (Motor_1 b1 / b5, Bremse_3 front-wheel low bytes).
//   Lever on the bus: fullValue whenever any lock is wanted, 0 when none is
//                     (0x7F on the lever opens the clutch anyway).
//   No lever:         the pre-V11 behaviour - fullValue scaled by the lock target -
//                     so an old saved mask without Bremse_4 never jumps to full lock.
uint8_t gen2DemandValue(uint8_t fullValue)
{
  if (gen2LeverOnBus())
  {
    if (lever_lock_percent() > 0)
    {
      return fullValue; // ask for everything, the Bremse_4 ceiling sets the level
    }
    return 0x00; // no lock wanted
  }
  return get_lock_target_adjusted_value(fullValue, false); // no lever: demand follows the lock target (pre-V11)
}

// ===== Gen1 (02D900554B) lock lever (V11.00.0) =====
// OEM code (documents/haldex-binaries/leaf_trace/spec_gen1_02d.md §2): the 02D
// works out its own clutch request from Motor_1 (torque, pedal, rpm), its
// tables and the wheel speeds. Bremse_4 (0x2A0) byte 0 is then a CEILING on that
// request, applied at the very end:  ceiling = (byte0 - 0x80) x 192.
//   0x7F        -> ceiling 0, clutch forced open straight away (no ramp)
//   0x81..0xC9  -> straight-line partial lock, 0xA5 = about 50 %
//   0xCA..0xFF  -> above the biggest request the unit ever makes = full lock
//   0x00..0x7E  -> no ceiling at all
// So the ceiling only works as a lever while Motor_1 asks for MAXIMUM demand.
#define GEN1_BREMSE4_BLOCK_BIT 4   // frameEditBlocks Gen1 bit for Bremse_4 (0x2A0)
#define GEN1_FULL_SCALE 0x3700     // biggest request the 02D makes in normal driving (OEM code: the E2A0 reset value)
#define GEN1_CEILING_STEP 192      // one Bremse_4 byte-0 step = 192 request units (OEM code)
#define GEN1_FULL_LOCK_BYTE 0xFD   // 100 %: ceiling 24000, well above 0x3700, so no limit (never 0xFE/0xFF)
#define GEN1_BREMSE4_TIMEOUT_MS 500 // OEM mode: the car's 0x2A0 counts as "present" for this long after the last one

volatile uint32_t gen1CarBremse4Ms = 0; // millis() of the last car Bremse_4 we edited (0 = never). volatile: written by the chassis CAN task

// Gen1: Bremse_4 byte 0 for the lock asked for right now. The general
// vag_lever_byte() spreads 1-99 % over 0x81..0xFC, but on the 02D everything
// from 0xCA up is already full lock, so 60-99 % would all come out as 100 %.
// This uses the 02D's own scale instead: 1-99 % -> 0x81..0xC9.
uint8_t gen1_lever_byte()
{
  uint8_t lockPercent = lever_lock_percent(); // learn sweep / learn table / lock gates already applied

  if (lockPercent == 0)
  {
    return 0x7F; // ceiling 0: clutch open
  }
  if (lockPercent >= 100)
  {
    return GEN1_FULL_LOCK_BYTE; // no ceiling: the unit's own maximum
  }

  uint32_t wantedCeiling = (uint32_t)lockPercent * GEN1_FULL_SCALE / 100;           // ceiling in the unit's own units
  uint32_t ceilingSteps = (wantedCeiling + (GEN1_CEILING_STEP / 2)) / GEN1_CEILING_STEP; // rounded to whole byte steps
  if (ceilingSteps < 1)
  {
    ceilingSteps = 1; // 1 % must still be a (small) request, not "open"
  }
  return (uint8_t)(0x80 + ceilingSteps); // 50 % -> 0xA5, 99 % -> 0xC9 (matches the OEM table)
}

// Gen1: is Bremse_4 byte 0 carrying our lock lever right now?
// Standalone: whenever its frame-edit block is on (we send the frame ourselves).
// OEM mode: only when its block is on AND the car is really sending 0x2A0 -
// many Gen1 cars don't, and then Motor_1 has to stay the lever (old method).
bool gen1_ceiling_lever_active()
{
  if (!frameEditEnabled(FE_GEN_1, GEN1_BREMSE4_BLOCK_BIT))
  {
    return false; // Bremse_4 block turned off
  }
  if (isStandalone)
  {
    return true; // we generate 0x2A0 ourselves
  }

  uint32_t lastSeenMs = gen1CarBremse4Ms; // copy first: the CAN task can change it at any moment
  if (lastSeenMs == 0)
  {
    return false; // the car has never sent 0x2A0
  }
  uint32_t nowMs = millis(); // time now
  if ((nowMs - lastSeenMs) > GEN1_BREMSE4_TIMEOUT_MS)
  {
    return false; // the car has stopped sending 0x2A0
  }
  return true;
}

// Gen1: value for a "demand" byte (Motor_1 torque / pedal, the Bremse_3 slip).
// With the Bremse_4 ceiling as the lever, the demand is held at maximum and the
// ceiling does the partial lock (OEM code: spec_gen1_02d §2.3); 0 % drops the
// demand too. Without it (old method), the demand byte itself is scaled.
uint8_t gen1_demand_value(uint8_t fullValue)
{
  if (!gen1_ceiling_lever_active())
  {
    return get_lock_target_adjusted_value(fullValue, false); // old method: demand scaled by the lock
  }
  if (lever_lock_percent() == 0)
  {
    return 0x00; // no lock asked for: no demand
  }
  return fullValue; // maximum demand, Bremse_4 sets how much of it the clutch gets
}

// Gen1: Motor_1 byte 6 (mechanical loss torque - the 02D SUBTRACTS it from
// byte 1, OEM code §2.1). 0x00 = maximum demand, 0xFE = no drive torque.
// Never 0xFF (= invalid, the unit then uses a 30 Nm default).
// oemValue = the car's own byte 6 (OEM mode) or 0x00 (standalone), kept for
// Stock / Expert like before.
uint8_t gen1_loss_torque_byte(uint8_t oemValue)
{
  if (gen1_ceiling_lever_active())
  {
    if (lever_lock_percent() == 0)
    {
      return 0xFE; // no lock: no drive torque
    }
    return 0x00; // maximum demand
  }

  switch (state.mode)
  {
  case MODE_FWD:
    return get_lock_target_adjusted_value(0xFE, true); // 0xFE = no drive torque
  case MODE_5050:
    return 0x00; // was 0x16 (took ~22 Nm off); 0x00 is the real maximum (OEM code: spec_gen1_02d D11)
  case MODE_6040:
    return get_lock_target_adjusted_value(0x22, false); // bench-tuned constant
  case MODE_7525:
    return get_lock_target_adjusted_value(0x50, false); // bench-tuned constant
  default:
    return oemValue; // Stock / Expert: unchanged
  }
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

  // If the incoming frame is a known frame for this generation and its bit is
  // cleared, skip all editing.  If its bit is set, allow editing
  {
    int _feGen = frameEditGenIdx(haldexGeneration);
    if (_feGen >= 0)
    {
      for (uint16_t _i = 0; _i < frameEditBlockCount; _i++)
      {
        if (frameEditBlocks[_i].genIdx == (uint8_t)_feGen &&
            frameEditBlocks[_i].canId == rx_message_chs.identifier)
        {
          if (!frameEditEnabled((uint8_t)_feGen, frameEditBlocks[_i].bit))
            return; // block disabled -> pass the car's real frame through untouched
          break;
        }
      }
    }
  }

  // begin frame parsing / editting
  // edit the frames if configured as Gen1...
  if (haldexGeneration == 1)
  {
    switch (rx_message_chs.identifier)
    {
    case MOTOR1_ID:
      // PQ Motor_1 (0x280). What the 02D reads (OEM code: spec_gen1_02d §3):
      // b0 bit 3, b1 torque, b2-3 rpm, b5 pedal, b6 loss torque. b4 and b7 are never read.
      rx_message_chs.data[0] = 0x00;                       // b0 status bits: 0 stops the clutch-switch (Kupplungsschalter, bit 3) wheel-pattern release from arming (spec §4: harmless)
      rx_message_chs.data[1] = gen1_demand_value(0xFE);    // b1 inneres_Motor_Moment (engine torque) - the demand
      // b2-3 Motordrehzahl (rpm): the car's REAL rpm is passed through (V11). It used to be
      // scaled with the lock, which dropped it below 400 rpm at low targets and the 02D
      // switches its pump off below 400 rpm = no lock at all (spec_gen1_02d D5).
      // b4 inneres_Motor_Moment_ohne_extern: not read by the 02D - left as the car sent it (D10)
      rx_message_chs.data[5] = gen1_demand_value(0xFE);    // b5 Fahrpedalwert (accelerator pedal) - the demand
      appliedTorque = gen1_loss_torque_byte(rx_message_chs.data[6]); // b6 mechanisches_Motor_Verlustmoment (subtracted from b1 by the unit)
      rx_message_chs.data[6] = appliedTorque;
      // b7 Fahrerwunschmoment (driver-wish torque): not read by the 02D - left as the car sent it (D10)
      break;
    case MOTOR3_ID:
      // Motor_3 (0x380): the 02D has NO receive slot for this ID (OEM code: spec_gen1_02d §1),
      // so this edit does nothing. Block is off by default since V11 (D8).
      rx_message_chs.data[2] = get_lock_target_adjusted_value(0xFE, false);
      rx_message_chs.data[7] = get_lock_target_adjusted_value(0xFE, false);
      break;
    case BRAKES1_ID:
    {
      // PQ Bremse_1 (0x1A0). Off by default since V11 (spec_gen1_02d D6). The old edit
      // zeroed bytes 1-2, which cleared the brake-light / ABS bits the 02D reads - most
      // importantly b1 bit 7, which makes the 02D OPEN the clutch and stop its pump.
      // Now every bit the 02D reads is kept exactly as the car sent it; only the
      // lamp / other bits it never reads are cleared.
      uint8_t carByte1 = rx_message_chs.data[1]; // car's byte 1
      rx_message_chs.data[1] = carByte1 & 0x98;  // b1: keep bit 3 (BR1_Lichtschalt, brake light), bit 4 (BR1_StaDruckschw), bit 7 (clutch release + pump off); clear the lamp bits
      // b0 (bit 2 BR1_ABS_Brems), b2 (bit 0 brake-source select + speed) and b3 are passed through untouched
      break;
    }
    case BRAKES3_ID:
      // PQ Bremse_3 (0x4A0) wheel speeds. ON by default in every non-Stock mode
      // alongside the lever (V11.00.0, Adam - the bench-proven method). Replaces the
      // real wheel speeds with ~13 km/h, which makes lock independent of road speed
      // but hides real wheel slip from the 02D's slip hold and its speed-dependent
      // over-temperature limit while forced (spec_gen1_02d D7). Stock never gets here.
      rx_message_chs.data[0] = gen1_demand_value(0xFE); // Radgeschw_VL low (front-left) - small fake front slip
      rx_message_chs.data[1] = 0x0A;                    // Radgeschw_VL high
      rx_message_chs.data[2] = gen1_demand_value(0xFE); // Radgeschw_VR low (front-right) - small fake front slip
      rx_message_chs.data[3] = 0x0A;                    // Radgeschw_VR high
      rx_message_chs.data[4] = 0x00;                    // Radgeschw_HL low (rear-left)
      rx_message_chs.data[5] = 0x0A;                    // Radgeschw_HL high
      rx_message_chs.data[6] = 0x00;                    // Radgeschw_HR low (rear-right)
      rx_message_chs.data[7] = 0x0A;                    // Radgeschw_HR high
      break;
    case BRAKES4_ID:
    {
      // PQ Bremse_4 (0x2A0) byte 0 = torque CEILING on the 02D's own request (OEM code:
      // spec_gen1_02d §2.3). Only some Gen1 cars send it; while they do, this is the lever
      // and Motor_1 above is held at maximum demand (gen1_ceiling_lever_active()).
      gen1CarBremse4Ms = millis(); // the car is sending 0x2A0
      if (rx_message_chs.data_length_code < 1)
      {
        break; // empty frame - nothing to edit
      }
      uint8_t carCeilingByte = rx_message_chs.data[0]; // the ABS module's own ceiling (0x7F+ = a limit, below = none)
      uint8_t ourCeilingByte = gen1_lever_byte();       // the lock we want, 0x7F..0xFD
      if (carCeilingByte >= 0x7F && carCeilingByte < ourCeilingByte)
      {
        // The ABS is asking for LESS clutch than we are (e.g. during an ABS / ESP
        // event): keep the car's value. We may only ever lower the ceiling.
        rx_message_chs.data[0] = carCeilingByte;
      }
      else
      {
        rx_message_chs.data[0] = ourCeilingByte; // our lock lever
      }
      // bytes 1-7: not read by the 02D - left as the car sent them
      break;
    }
    }
  }

  // edit the frames if configured as Gen2...
  // V11.00.0 (OEM code: spec_gen2_7h0.md). The lever is Bremse_4 (0x2A0) byte 0;
  // Motor_1 is held at full demand so the lever's ceiling decides the lock (see
  // gen2DemandValue above). Default OEM blocks (V11.00.0, Adam: achieve the lock)
  // are Motor_1 + Bremse_3 (lock-scaled wheel speeds) + Bremse_4 = 0x31; ABS / ESP,
  // brake and handbrake bits pass through.
  // The car has one chassis bus and one Haldex bus on Gen2 (Adam, 2026-10-22).
  // The OEM code sets up a second CAN controller in the chip (spec_gen2_7h0 §1),
  // but only one bus is wired to the unit, so the spec's CAN1/CAN2 split doesn't
  // apply to us - Gen2 has always locked from this one bus.
  if (haldexGeneration == 2)
  {
    uint8_t editedByte = 0; // our value for a byte, before the car's safety bits are put back

    switch (rx_message_chs.identifier)
    {
    case MOTOR1_ID:
      // Motor_1 bytes the unit reads (spec_gen2_7h0 §2.2): b1 inner torque (ADDS to the
      // estimate), b6 loss torque (is SUBTRACTED from b1), b5 pedal, b2-3 rpm, b0 bit 3.
      // Before V11 b1 and b6 were both set to the same value, so they cancelled out, and
      // b2-3 (rpm) were overwritten. Now: rpm and b0 pass through from the car.
      rx_message_chs.data[1] = gen2DemandValue(0xFE); // inneres_Motor_Moment (inner torque) - full demand when lock wanted
      rx_message_chs.data[5] = gen2DemandValue(0xFE); // Fahrpedalwert (pedal) - full when lock wanted
      rx_message_chs.data[6] = 0x00;                  // mechanisches_Motor_Verlustmoment (loss torque) - 0, never 0xFF (0xFF = invalid -> 30 Nm default)
      break;
    case MOTOR3_ID:
      // Not received by the 7H0 / 1K0 (OEM code: no receive object for 0x380) - this edit does
      // nothing to the Haldex. Block is off by default.
      rx_message_chs.data[2] = get_lock_target_adjusted_value(0xFE, false);
      rx_message_chs.data[7] = get_lock_target_adjusted_value(0x01, false);
      break;
    case BRAKES1_ID:
      // Safety (V11): the unit reads these Bremse_1 bits to open the clutch for ABS / ESP
      // braking (spec_gen2_7h0 §3): b0 bit2 BR1_ABS_Brems, b0 bit4 BR1_ESP_Ongr, b1 bits 0/3/4/7
      // (ABS lamp, light switch, pressure-switch status, diagnosis), b2 bit0 BR1_BKV_active.
      // Those are always the car's own; we only edit the other bits. Block is off by default.
      editedByte = get_lock_target_adjusted_value(0x80, false); // pre-V11 edit value
      editedByte = editedByte & 0xEB;                            // drop our bits 2 and 4
      rx_message_chs.data[0] = (rx_message_chs.data[0] & 0x14) | editedByte; // car's b0 bit2 + bit4 kept
      editedByte = get_lock_target_adjusted_value(0x41, false);  // pre-V11 edit value
      editedByte = editedByte & 0x66;                            // drop our bits 0, 3, 4, 7
      rx_message_chs.data[1] = (rx_message_chs.data[1] & 0x99) | editedByte; // car's b1 bits 0/3/4/7 kept
      editedByte = get_lock_target_adjusted_value(0xFE, false);  // pre-V11 edit value
      editedByte = editedByte & 0xFE;                            // drop our bit 0
      rx_message_chs.data[2] = (rx_message_chs.data[2] & 0x01) | editedByte; // car's b2 bit0 kept
      rx_message_chs.data[3] = 0x0A;                             // BR1_Rad_kmh high (not read by the unit)
      break;                                                     // b6 / b7 (alive counter) untouched
    case BRAKES2_ID:
      // The unit reads Bremse_2 b0, b1 bit0 and b7 bit7 only - this edit writes b4/b5, which it
      // ignores (spec_gen2_7h0 D8). Harmless; block is off by default (pass the car's frame).
      rx_message_chs.data[4] = get_lock_target_adjusted_value(0x7F, false);
      rx_message_chs.data[5] = get_lock_target_adjusted_value(0xFE, false);
      break;
    case BRAKES3_ID:
      // ON by default in every non-Stock mode alongside the Bremse_4 lever (V11.00.0,
      // Adam - the bench-proven method). This REPLACES the car's real wheel speeds, so
      // the unit's own slip and over-speed protection can't see real slip while forced
      // (spec_gen2_7h0 D9); Stock never gets here. Front wheels a little faster than
      // the rear = fake slip, full whenever any lock is wanted.
      rx_message_chs.data[0] = gen2DemandValue(0xFE); // Radgeschw_VL low (front left)
      rx_message_chs.data[1] = 0x0A;                  // Radgeschw_VL high
      rx_message_chs.data[2] = gen2DemandValue(0xFE); // Radgeschw_VR low (front right)
      rx_message_chs.data[3] = 0x0A;                  // Radgeschw_VR high
      rx_message_chs.data[4] = 0x00;                  // Radgeschw_HL low (rear left)
      rx_message_chs.data[5] = 0x0A;                  // Radgeschw_HL high
      rx_message_chs.data[6] = 0x00;                  // Radgeschw_HR low (rear right)
      rx_message_chs.data[7] = 0x0A;                  // Radgeschw_HR high
      break;

    // ---- Transferred from standalone (gated off by default) ----------------
    case BRAKES4_ID:
      // THE LEVER (OEM code: spec_gen2_7h0 §2.3). Byte 0 is a ceiling on the clutch request
      // in the same 0x80-offset units as the engagement feedback (0x2C0 byte 1):
      //   0x7F = 0 % (clutch open), 0x81..0xFD = (byte - 0x80) / 0x7D x 100 %, 0xFD = 100 %.
      //   0xFE / 0xFF / below 0x7F = "no limit" - never sent as a lock value.
      // Bytes 1-2 (read by the unit, role not decoded) pass through from the car.
      // V11 TODO (bench): if the car does not send 0x2A0 at all, OpenHaldex would have to
      // inject it - not done yet; Motor_1 then falls back to the pre-V11 scaling.
      // Safety: the car's ABS module uses this same byte to LIMIT the clutch (e.g. during an
      // ABS / ESP event). If its value is a valid limit (0x7F..0xFD) lower than ours, keep the
      // car's - we may only ever lower the ceiling, never lift the ABS module's own limit.
      // (Log "CAN_log_ONLY_ABS_30s-30mph_60s-70mph": a PQ ABS sends b0-b2 = 0x00 all the time
      // = "no limit", so normally our lever goes straight in.)
      gen2LastBremse4Ms = millis();                        // the car's 0x2A0 is here: the lever is on the bus
      editedByte = vag_lever_byte(0xFD);                     // our lock request: 0x7F / 0x81..0xFC / 0xFD
      if (rx_message_chs.data[0] >= 0x7F && rx_message_chs.data[0] <= 0xFD && rx_message_chs.data[0] < editedByte)
      {
        break; // the car's ABS is asking for less clutch than we are - leave its byte 0 alone
      }
      rx_message_chs.data[0] = editedByte;                   // our lock lever
      break;
    case BRAKES5_ID:
      // 0x4A8: the unit checks b6 bits 4-7 as an alive counter (must change every frame) and
      // reads a 12-bit value in b2-3 (spec_gen2_7h0 §1/§3). No checksum is checked.
      // Off by default - better to pass the car's frame (its own counter).
      rx_message_chs.data[0] = 0xFE;
      rx_message_chs.data[1] = 0x7F;
      rx_message_chs.data[2] = 0x03;
      rx_message_chs.data[3] = 0x00;
      rx_message_chs.data[4] = 0x00;
      rx_message_chs.data[5] = 0x00;
      BRAKES5_counter = BRAKES5_counter + 0x10;              // next alive count in the top 4 bits (wraps 0xF? -> 0x0? on its own)
      rx_message_chs.data[6] = BRAKES5_counter & 0xF0;       // b6 bits 4-7 = alive counter, bits 0-3 = 0
      rx_message_chs.data[7] = BRAKES5_counter2;             // checksum byte - the unit does not check it
      BRAKES5_counter2 = BRAKES5_counter2 + 10;
      if (BRAKES5_counter2 > 0xF3)
        BRAKES5_counter2 = 3;
      break;
    case BRAKES9_ID:
      // Bremse_9 (0x0AE) and Bremse_10 (0x3A0) are NOT received by the 7H0 / 1K0 (OEM code:
      // no receive object) - these edits do nothing to the Haldex. Blocks off by default.
      rx_message_chs.data[0] = BRAKES9_counter;
      rx_message_chs.data[1] = BRAKES9_counter2;
      rx_message_chs.data[2] = 0x00;
      rx_message_chs.data[3] = 0x00;
      rx_message_chs.data[4] = 0x00;
      rx_message_chs.data[5] = 0x00;
      rx_message_chs.data[6] = 0x02;
      rx_message_chs.data[7] = 0x00;
      BRAKES9_counter = BRAKES9_counter + 10;
      if (BRAKES9_counter > 0xF1)
        BRAKES9_counter = 0x11;
      BRAKES9_counter2 = BRAKES9_counter2 + 10;
      if (BRAKES9_counter2 > 0xF0)
        BRAKES9_counter2 = 0x00;
      break;
    case BRAKES10_ID:
      rx_message_chs.data[0] = 0xA6;
      rx_message_chs.data[1] = BRAKES10_counter;
      rx_message_chs.data[2] = 0x75;
      rx_message_chs.data[3] = 0xD4;
      rx_message_chs.data[4] = 0x51;
      rx_message_chs.data[5] = 0x47;
      rx_message_chs.data[6] = 0x1D;
      rx_message_chs.data[7] = 0x0F;
      BRAKES10_counter = BRAKES10_counter + 1;
      if (BRAKES10_counter > 0xF)
        BRAKES10_counter = 0;
      break;
    case MOTOR5_ID:
      rx_message_chs.data[0] = 0xFE;
      rx_message_chs.data[1] = 0x00;
      rx_message_chs.data[2] = 0x00;
      rx_message_chs.data[3] = 0x00;
      rx_message_chs.data[4] = 0x00;
      rx_message_chs.data[5] = 0x00;
      rx_message_chs.data[6] = 0x00;
      rx_message_chs.data[7] = MOTOR5_counter;
      MOTOR5_counter++;
      break;
    case MOTOR2_ID:
      // Unit reads b0 (mux 3 = max engine torque, else keeps 300 Nm) and b2 bits 0/1
      // (MO2_BLS brake light / MO2_BTS brake pedal switch - its brake vote). Safety (V11):
      // b2 bits 0-1 stay the car's own. Off by default (pass the car's real torque).
      rx_message_chs.data[0] = 0x00;                                   // mux 0 - unit keeps its 300 Nm torque reference
      rx_message_chs.data[1] = 0x30;                                   // MO2_Kuehlm_T (coolant temp)
      rx_message_chs.data[2] = rx_message_chs.data[2] & 0x03;          // car's MO2_BLS (bit0) + MO2_BTS (bit1) kept, rest 0
      rx_message_chs.data[3] = 0x0A;
      rx_message_chs.data[4] = 0x0A;
      rx_message_chs.data[5] = 0x10;
      rx_message_chs.data[6] = 0xFE;
      rx_message_chs.data[7] = 0xFE;
      break;
    case mLW_1:
      // Steering angle. The unit checks b5 bits 4-7 as an alive counter (steps of 0x10, must
      // change every frame); b7 is not checked (no checksums on Gen2). Off by default.
      rx_message_chs.data[0] = 0x20;
      rx_message_chs.data[1] = 0x00;
      rx_message_chs.data[2] = 0x00;
      rx_message_chs.data[3] = 0x00;
      rx_message_chs.data[4] = 0x80;
      rx_message_chs.data[5] = mLW_1_counter;
      rx_message_chs.data[6] = 0x00;
      mLW_1_crc = 255 - (rx_message_chs.data[0] + rx_message_chs.data[1] + rx_message_chs.data[2] + rx_message_chs.data[3] + rx_message_chs.data[5]);
      rx_message_chs.data[7] = mLW_1_crc;
      mLW_1_counter = mLW_1_counter + 16;
      if (mLW_1_counter >= 0xF0)
        mLW_1_counter = 0;
      break;
    case mKombi_1:
      // Unit reads b1 bit1 (KO1_Handbremse - its handbrake vote). Safety (V11): before V11
      // this wrote 0x02 = handbrake PULLED. Now b1 bits 0-1 (brake-fluid warning, handbrake)
      // stay the car's own. Off by default.
      rx_message_chs.data[0] = 0x00;
      rx_message_chs.data[1] = rx_message_chs.data[1] & 0x03; // car's KO1_Bremsinfo (bit0) + KO1_Handbremse (bit1) kept, rest 0
      rx_message_chs.data[2] = 0x00;
      rx_message_chs.data[3] = 0x00;
      rx_message_chs.data[4] = 0x36;
      rx_message_chs.data[5] = 0x00;
      rx_message_chs.data[6] = 0x00;
      rx_message_chs.data[7] = 0x00;
      break;
    }
  }

  // edit the frames if configured as Gen4...
  // V11.00.0 (OEM code: spec_gen4_0br): Bremse_4 byte 0 is the only lever the
  // 0BR needs - any valid byte there outranks the unit's own strategy. The OEM
  // default (V11.00.0, Adam) is every block that drives lock: Bremse_4 (lever) +
  // Motor_1 + Bremse_3 (the bench-proven lock-scaled method) = 0x32. Steering and
  // the ABS / brake / handbrake flags pass through; the Bremse_1 / Kombi_1 edits,
  // if turned on, keep those bits.
  // Motor_2 / Motor_5 / Bremse_5 / Bremse_8 / Getriebe_1 (blocks 12-16) have no
  // case here on purpose: in OEM mode the car's own frame always passes through.
  if (haldexGeneration == 4)
  {
    switch (rx_message_chs.identifier)
    {
    case mLW_1:
      // LW_1 replay: replaces the real steering angle with a 16-step capture.
      // Not needed for control (spec_gen4_0br §7 #10) - block OFF by default.
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
      // Motor_1: lock-scaled (the bench-proven method), ON by default alongside the
      // Bremse_4 lever. It feeds the unit's own strategy, which a valid Bremse_4
      // byte overrides (spec_gen4_0br §2.3), so the two agree.
      // Byte 0 (clutch pedal switch b0.3) is left as the car sent it.
      rx_message_chs.data[1] = get_lock_target_adjusted_value(0xFE, false); // inneres_Motor_Moment (inner torque)
      rx_message_chs.data[2] = get_lock_target_adjusted_value(0x20, false); // Motordrehzahl low (rpm x 0.25)
      rx_message_chs.data[3] = get_lock_target_adjusted_value(0x4E, false); // Motordrehzahl high
      rx_message_chs.data[4] = get_lock_target_adjusted_value(0xFE, false); // inner torque without external (not read by Gen4)
      rx_message_chs.data[5] = get_lock_target_adjusted_value(0xFE, false); // Fahrpedalwert (pedal x 0.4 %)
      rx_message_chs.data[6] = get_lock_target_adjusted_value(0x16, false); // mechanisches_Motor_Verlustmoment (loss torque)
      rx_message_chs.data[7] = get_lock_target_adjusted_value(0xFE, false); // Fahrerwunschmoment (driver demand)
      break;
    case BRAKES1_ID:
    {
      // Bremse_1: the 0BR reads b0.0 ASR request, b0.2 ABS braking, b0.3 EDS,
      // b0.4 ESP and b1.3 / b1.4 (spec_gen4_0br §3). V11: keep those exactly as
      // the car sent them so the unit still sees real ABS braking (§7 #8).
      // b4-5 used to be lock-scaled here, but the unit never reads them - dropped.
      uint8_t carByte0 = rx_message_chs.data[0];                 // car's real ABS / ESP / EDS / ASR flags
      uint8_t carByte1 = rx_message_chs.data[1];                 // car's real b1 flags
      rx_message_chs.data[0] = 0x20 | (carByte0 & 0x1D);         // 0x20 as before + real bits 0, 2, 3, 4
      rx_message_chs.data[1] = 0x40 | (carByte1 & 0x18);         // 0x40 as before + real bits 3, 4
      break;
    }
    case BRAKES2_ID:
      // Bremse_2 byte 4 (lateral acceleration) is not read by the 0BR - no
      // effect on Gen4 (spec_gen4_0br §7 #11). Block OFF by default.
      rx_message_chs.data[4] = get_lock_target_adjusted_value(0x7F, false);
      break;
    case BRAKES3_ID:
      // Bremse_3: replaces the REAL wheel speeds with ~9-10 km/h figures scaled
      // by lock - the bench-proven method, ON by default in every non-Stock mode
      // alongside the Bremse_4 lever (V11.00.0, Adam). Hides real slip from the
      // unit while forced (spec_gen4_0br §7 #9); Stock never gets here.
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
      // Bremse_4 byte 0 = the 0BR's direct clutch-torque lever (OEM code:
      // spec_gen4_0br §2.2 / §2.6). V11: built from the lock %, not from the mode:
      //   0 %      -> 0x7F  valid zero request, clutch commanded fully open
      //   1-99 %   -> 0x80 + % x 1.25 (linear, 112 internal torque units per step)
      //   100 %    -> 0xFE  "present maximum clutch capacity"
      // Before V11 this sent 0x7F x CF/100 (below 0x7F = ignored by the unit) and
      // 0x7F at 100 % outside 50:50 mode = clutch OPEN (§7 #1 / #2). The learn
      // sweep now moves this byte directly too (§7 #3, via lever_lock_percent()).
      appliedTorque = vag_lever_byte(0xFE); // Gen4: 0xFE = full lock

      rx_message_chs.data[0] = appliedTorque; // lever (see above)
      rx_message_chs.data[1] = 0x00;          // not read by Gen4
      rx_message_chs.data[2] = 0x00;          // not read by Gen4
      rx_message_chs.data[3] = 0x64;          // not read by Gen4
      rx_message_chs.data[4] = 0x00;          // not read by Gen4
      rx_message_chs.data[5] = 0x00;          // not read by Gen4
      rx_message_chs.data[6] = BRAKES4_counter; // counter in the high nibble - the 0BR never checks it (§7 #20)
      BRAKES4_crc = 0;
      for (uint8_t i = 0; i < 7; i++)
      {
        BRAKES4_crc ^= rx_message_chs.data[i];
      }
      rx_message_chs.data[7] = BRAKES4_crc; // XOR checksum - the 0BR does not even copy byte 7 (§7 #20)

      BRAKES4_counter = BRAKES4_counter + 16;
      if (BRAKES4_counter > 0xF0)
      {
        BRAKES4_counter = 0x00;
      }
      break;

    // ---- Transferred from standalone (gated off by default) ----------------
    case mKombi_1:
    {
      // Kombi_1: the 0BR reads b1.1 handbrake (spec_gen4_0br §3). V11: keep the
      // car's real b1 bits 0-1 (brake info + handbrake) - Follow Handbrake is
      // applied afterwards by applyBrakeHandbrakeCANOverride().
      uint8_t carHandbrakeBits = rx_message_chs.data[1] & 0x03; // car's real Bremsinfo + handbrake bits
      rx_message_chs.data[0] = 0x24;
      rx_message_chs.data[1] = carHandbrakeBits;
      rx_message_chs.data[2] = 0x1D;
      rx_message_chs.data[3] = 0xB9;
      rx_message_chs.data[4] = 0x07;
      rx_message_chs.data[5] = 0x42;
      rx_message_chs.data[6] = 0x09;
      rx_message_chs.data[7] = 0x81;
      break;
    }
    case mKombi_3:
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
      rx_message_chs.data[0] = 0x00;
      rx_message_chs.data[1] = 0xC0;
      rx_message_chs.data[2] = 0x00;
      rx_message_chs.data[3] = 0x00;
      rx_message_chs.data[4] = 0x00;
      rx_message_chs.data[5] = 0x00;
      rx_message_chs.data[6] = 0x00;
      rx_message_chs.data[7] = 0x00;
      break;
    case mKombi_2:
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
      rx_message_chs.data[0] = 0x26;
      rx_message_chs.data[1] = 0xF2;
      rx_message_chs.data[2] = 0x03;
      rx_message_chs.data[3] = 0x12;
      rx_message_chs.data[4] = 0x70;
      rx_message_chs.data[5] = 0x19;
      rx_message_chs.data[6] = 0x25;
      rx_message_chs.data[7] = mDiagnose_1_counter;
      mDiagnose_1_counter++;
      if (mDiagnose_1_counter > 0x1F)
        mDiagnose_1_counter = 0;
      break;
    }
  }

  // edit the frames if configured as Gen5 (0AY, 0AY907554F).
  // V11.00.0: the 0AY is a PQ unit (PQ frame set, feedback Allrad_1 0x2C0) and
  // its lock lever is Bremse_4 (0x2A0) byte 0 - OEM code: spec_gen5_0ay §2.
  // OEM default 0x06 (V11.00.0, Adam: achieve the lock) = Bremse_4 (the lever) +
  // Bremse_3 (lock-scaled wheel speeds, the bench-proven method). Every other
  // block is OFF by default; if switched on it sends steady values and still keeps
  // the car's real ABS / brake-light / handbrake bits.
  if (haldexGeneration == 51)
  {
    switch (rx_message_chs.identifier)
    {
    // ---- The lever --------------------------------------------------------
    case BRAKES4_ID:
      // PQ Bremse_4 (0x2A0) byte 0 = clutch torque request (spec_gen5_0ay §2):
      //   0x7F = open, 0x80 + n = n x 112 (linear), 0xFD = maximum request,
      //   0xFE = HOLD the last target on the 0AY (not "full" as on Gen4),
      //   0x00-0x7E / 0xFF = ignored (unit runs its own strategy).
      // So 100 % is 0xFD here. vag_lever_byte() already sends 0x7F while a lock
      // gate is closed (ABS braking, handbrake with Follow Handbrake, speed /
      // throttle settings) - needed because a valid Bremse_4 outranks the
      // unit's own brake + ABS and handbrake clutch-open.
      // The 0AY only stores bytes 0-2 and never reads the counter / XOR in
      // bytes 6-7, so bytes 1-7 stay exactly as the car sent them.
      rx_message_chs.data[0] = vag_lever_byte(0xFD); // ABS_Vorgabewert (lever)
      break;

    // ---- Optional edits (OFF by default) ----------------------------------
    case MOTOR1_ID:
      // PQ Motor_1 (0x280) - engine ECU broadcast. Only feeds the unit's own
      // strategy, which a valid Bremse_4 overrides, so it is no longer a lever
      // (was lock-scaled before V11: rpm 0 at FWD). Steady plausible values
      // from spec_gen5_0ay §5. Byte names from vw_pq.dbc / spec §3.
      rx_message_chs.data[0] = 0x00; // flags: b0.3 Kupplungsschalter (clutch pedal) = 0 released
      rx_message_chs.data[1] = 0x20; // inneres_Motor_Moment (inner torque) small, not 0xFF
      rx_message_chs.data[2] = 0x80; // Motordrehzahl low  (0x0C80 x 0.25 = 800 rpm idle)
      rx_message_chs.data[3] = 0x0C; // Motordrehzahl high
      rx_message_chs.data[4] = 0x20; // inneres_Motor_Moment_ohne_extern - never read by the 0AY
      rx_message_chs.data[5] = 0x00; // Fahrpedalwert (accelerator pedal) 0 %
      rx_message_chs.data[6] = 0x10; // mechanisches_Motor_Verlustmoment (loss torque), not 0xFF
      rx_message_chs.data[7] = 0x00; // Fahrerwunschmoment (driver-wish torque)
      break;

    case BRAKES3_ID:
      // PQ Bremse_3 (0x4A0) - per-wheel speeds (Radgeschw_VL/VR/HL/HR), lock-scaled.
      // The bench-proven wheel-speed method, kept ON by default in every non-Stock
      // mode alongside the Bremse_4 lever (V11.00.0, Adam). The OEM code (spec_gen5_0ay
      // §4 / §7 #4) shows this hides real slip from the unit while forced; Stock never
      // gets here (untouched passthrough). Switch this block off for the lever alone.
      // Low bytes lock-adjusted, high bytes fixed at 0x07 (as 10.00.2).
      rx_message_chs.data[0] = get_lock_target_adjusted_value(0xB6, false); // Radgeschw_VL low
      rx_message_chs.data[1] = 0x07;                                        // Radgeschw_VL high
      rx_message_chs.data[2] = get_lock_target_adjusted_value(0xCC, false); // Radgeschw_VR low
      rx_message_chs.data[3] = 0x07;                                        // Radgeschw_VR high
      rx_message_chs.data[4] = get_lock_target_adjusted_value(0xD2, false); // Radgeschw_HL low
      rx_message_chs.data[5] = 0x07;                                        // Radgeschw_HL high
      rx_message_chs.data[6] = get_lock_target_adjusted_value(0xD2, false); // Radgeschw_HR low
      rx_message_chs.data[7] = 0x07;                                        // Radgeschw_HR high
      break;

    case mLW_1:
      // PQ LW_1 (0x0C2) - steering-angle replay table (recorded from a real
      // car: about 1.5 deg, counter in byte 5); not lock-modulated.
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

    // ---- Inactive frames restored (gated off by default via frameEditMask) --
    case BRAKES1_ID:
      // PQ Bremse_1 (0x1A0) - ABS/ESP main broadcast.
      // Bytes 0-1 are left as the car sent them: the 0AY reads b0.0 ASR
      // request, b0.2 ABS braking, b0.3 EDS, b0.4 ESP and b1.3 brake-light
      // switch (spec_gen5_0ay §3) - its own ABS clutch-open needs them.
      // Before V11 this wrote b0 = 0x20 / b1 = 0x40 and wiped the ABS flag.
      rx_message_chs.data[2] = 0xF0;            // BR1_Rad_kmh low (not read by the 0AY)
      rx_message_chs.data[3] = 0x07;            // BR1_Rad_kmh high (0x07F0 >> 1 = 10.16 km/h)
      rx_message_chs.data[4] = 0xFE;            // BR1_ASRMo_sl (ASR torque, no reduction)
      rx_message_chs.data[5] = 0xFE;            // BR1_ASRMo_fa (ASR torque, no reduction)
      rx_message_chs.data[6] = 0x00;            // BR1_MSR_Mo
      rx_message_chs.data[7] = BRAKES1_counter; // BR1_Zaehler - counter, not checked by the 0AY
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
      // PQ Bremse_5 (0x4A8) - ESP broadcast. The 0AY reads only the yaw rate
      // (b0 + b1 bits 0-5, b1.7 sign) - spec_gen5_0ay §3/§5: yaw 0.
      // Was 0xFE / 0x7F (near-maximum yaw) before V11.
      rx_message_chs.data[0] = 0x00; // yaw rate low = 0
      rx_message_chs.data[1] = 0x00; // yaw rate high bits 0-5 = 0, b1.7 sign = 0
      rx_message_chs.data[2] = 0x03;
      rx_message_chs.data[3] = 0x00;
      rx_message_chs.data[4] = 0x00;
      rx_message_chs.data[5] = 0x00;
      rx_message_chs.data[6] = BRAKES5_counter;
      rx_message_chs.data[7] = BRAKES5_counter2;
      BRAKES5_counter = BRAKES5_counter + 10;
      if (BRAKES5_counter > 0xF0)
        BRAKES5_counter = 0;
      BRAKES5_counter2 = BRAKES5_counter2 + 10;
      if (BRAKES5_counter2 > 0xF3)
        BRAKES5_counter2 = 3;
      break;

    case BRAKES8_ID:
      // PQ Bremse_8 (0x1AC) - ESP supplemental broadcast; dual rolling counters.
      // NOT received by the 0AY (no message object - spec_gen5_0ay §1), so this
      // edit has no effect on the Haldex; block OFF in both modes.
      rx_message_chs.data[0] = BRAKES8_counter;
      rx_message_chs.data[1] = BRAKES8_counter1;
      rx_message_chs.data[2] = 0x00;
      rx_message_chs.data[3] = 0x00;
      rx_message_chs.data[4] = 0x69;
      rx_message_chs.data[5] = 0x21;
      rx_message_chs.data[6] = 0x00;
      rx_message_chs.data[7] = 0xC1;
      if (++BRAKES8_counter > 0x8F)
        BRAKES8_counter = 0x80;
      if (++BRAKES8_counter1 > 0x0F)
        BRAKES8_counter1 = 0x00;
      break;

    case BRAKES2_ID:
      // PQ Bremse_2 (0x5A0) - ESP/ABS sensor broadcast. The 0AY reads b0
      // (lateral accel, 0x7F = 0 g), b1.0 (mux) and b7.7 - spec_gen5_0ay §5:
      // b0 = 0x7F, b1.0 = 0, b7.7 = 0. Before V11: b0 0x80, b7 0xAB (b7.7 = 1).
      rx_message_chs.data[0] = 0x7F;            // Querbeschleunigung = 0 g
      rx_message_chs.data[1] = 0x7A;            // b1.0 = 0 (lateral-accel mux), rest mean wheel speed
      rx_message_chs.data[2] = 0x05;            // mittlere_Raddrehzahl high
      rx_message_chs.data[3] = BRAKES2_counter; // Zeitstempel low (not read)
      rx_message_chs.data[4] = 0x7F;            // Zeitstempel high (not read)
      rx_message_chs.data[5] = 0xCA;            // Wegimpulse (not read)
      rx_message_chs.data[6] = 0x1B;            // Wegimpulse / status (not read)
      rx_message_chs.data[7] = 0x2B;            // Impulszahl; b7.7 gemessene_Querbeschleunigung = 0
      BRAKES2_counter = BRAKES2_counter + 16;
      if (BRAKES2_counter > 0xF0)
        BRAKES2_counter = 0;
      break;

    case MOTOR2_ID:
      // PQ Motor_2 (0x288) - engine broadcast. Byte 2 is left as the car sent
      // it: b2.0 MO2_BLS is the brake-light switch the 0AY uses for its own
      // brake clutch-open, b2.4 is also read (spec_gen5_0ay §3). Before V11
      // this wrote b2 = 0x00 and wiped the brake light.
      rx_message_chs.data[0] = 0x00; // mux 0 (MO2_CAN_Vers) - not the max-torque mux 3
      rx_message_chs.data[1] = 0x30; // MO2_Kuehlm_T coolant temp (not read by the 0AY)
      rx_message_chs.data[3] = 0x0A;
      rx_message_chs.data[4] = 0x0A;
      rx_message_chs.data[5] = 0x10;
      rx_message_chs.data[6] = 0xFE;
      rx_message_chs.data[7] = 0xFE;
      break;

    case MOTOR5_ID:
      // PQ Motor_5 (0x480) - engine broadcast. The 0AY only latches b0 once when
      // mux (bits 6-7) = 2; 0xFE is mux 3 so nothing is latched (spec_gen5_0ay §3).
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
      // PQ Kombi_1 (0x320) - instrument-cluster broadcast. Byte 1 is left as the
      // car sent it: b1.1 KO1_Handbremse is the handbrake the 0AY uses for its
      // own clutch-open (spec_gen5_0ay §3). Before V11 this wrote b1 = 0x00.
      rx_message_chs.data[0] = 0x24; // status bits (not read by the 0AY)
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
      // PQ Bremse_11 (0x5B7) - extended brake frame. NOT received by the 0AY
      // (spec_gen5_0ay §1) - no effect; block OFF in both modes.
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
      // PQ Kombi_2 (0x420) - cluster temps. NOT received by the 0AY
      // (spec_gen5_0ay §1) - no effect; block OFF in both modes.
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
      if (mDiagnose_1_counter > 0x1F)
        mDiagnose_1_counter = 0;
      break;
    }
  }

  // edit the frames if configured as Gen5 (0CQ) - frames left
  // commented-out are so they can be re-enabled later if a required
  if (haldexGeneration == 50 || haldexGeneration == 52) // 0CQ + VAQ (clone base)
  {
    switch (rx_message_chs.identifier)
    {
    case ESP_18: // 0x135 - fixed response, static (transferred; gated off by default)
      rx_message_chs.data[0] = 0x00;
      rx_message_chs.data[1] = 0xC0;
      rx_message_chs.data[2] = 0x00;
      rx_message_chs.data[3] = 0x00;
      rx_message_chs.data[4] = 0x00;
      rx_message_chs.data[5] = 0x00;
      rx_message_chs.data[6] = 0x00;
      rx_message_chs.data[7] = 0x00;
      break;
    case ESP_19:
      // Lock-scaled wheel speeds - the bench-proven method, kept ON by default in
      // every non-Stock mode alongside the ESP_14 lever (V11.00.0, Adam). The 0CQ
      // reads all four for its strategy and clutch-temperature model (OEM code:
      // spec_gen5_0cq §3 / D6), so it doesn't see real slip while forced. Stock
      // never gets here (untouched passthrough); switch block 0 off to pass the
      // car's real speeds with the lever alone.
      fill_esp19_wheel_speeds(rx_message_chs.data);
      break;

    case GETRIEBE_11:
      rx_message_chs.data[0] = 0x00;                // checksum placeholder (the 0CQ checks no CRC - spec §1)
      rx_message_chs.data[1] = GETRIEBE_11_counter; // rolling - 0x00>0x0F
      rx_message_chs.data[2] = 0x00;                // Was 0xFE Torque intervention at the engine. Requests a short-term reduction or increase in torque from the ECU. This signal is only valid in combination with GE_MMom_Status (for MQB) or GE_MMom_Status_02 (for MLBevo).
      rx_message_chs.data[3] = 0xFE;                // Pre-control torque (anticipatory torque request) (GE_MMom_Vorhalt_02)
      rx_message_chs.data[4] = 0x00;                // Actual gear/range selected (5=P, 6=R, 7=N, 8=D, 9=S, 10=E, 13/14=T)
      rx_message_chs.data[5] = 0x00;                // Shift sequence state (0=idle, 1=shift in progress, etc.) - does not affect (>0x00)
      rx_message_chs.data[6] = 0x00;                // Power transmission status / clutch lock-up state - does not affect (>0x00)
      // b7 bits 4-7 = target gear (GE_Zielgang), the only part the 0CQ reads (OEM code:
      // spec_gen5_0cq §3). V11.00.0: the car's real gear is kept - it used to be forced
      // to 0 here (spec D8). Bits 0-3 are not read and stay 0.
      rx_message_chs.data[7] = rx_message_chs.data[7] & 0xF0; // keep the car's target gear (b7 bits 4-7)

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
      // b6-b7 = engine speed (MO_Drehzahl_01, 0.25 rpm per step), read by the 0CQ's
      // strategy and monitors (OEM code: spec_gen5_0cq §3). V11.00.0: it used to be
      // (lock value << 8 | 0x0F) - about 4 rpm at 0 % up to ~16000 rpm at 100 %
      // (spec D5). Now a steady 2000 rpm on the 0CQ. Block 2 is ON by default (the
      // OEM default is every lock-driving block; the VAQ's Motor_12 still follows lock).
      rx_message_chs.data[6] = 0x40; // engine speed low byte  (0x1F40 = 8000 x 0.25 = 2000 rpm)
      rx_message_chs.data[7] = 0x1F; // engine speed high byte

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
      //   Danger Zone live    : BPK packing regardless (raised ceiling), see fill_motor11_bpk.
      // V11.00.0 (OEM code, spec_gen5_0cq §2.1): the 0CQ only uses this torque for its
      // OWN strategy, and that strategy is ignored while ESP_14 sends Min = Max (the
      // exact request we now always send). So Motor_11 can no longer fight the ESP_14
      // lever - it only feeds the unit's clutch-load model. Block 3 is ON by default
      // (the bench-proven set, kept alongside the ESP_14 lever).
      if (!fixHunting && !dangerZoneActive())
      {
        rx_message_chs.data[0] = 0x00;             // checksum placeholder (the 0CQ checks no CRC - spec §1)
        rx_message_chs.data[1] = MOTOR_11_counter; // rolling - 0x40>0x4F; high nibble is also the low bits of MO_Mom_Soll_Roh
        rx_message_chs.data[2] = 0xFA;             // MO_Mom_Soll_Roh + MO_Mom_Ist_Summe bits: Soll_Roh = 423 Nm, Ist = 494 Nm (spec D4)
        rx_message_chs.data[3] = 0xFA;             // MO_Mom_Ist_Summe (actual engine torque) high bits
        rx_message_chs.data[4] = 0x00;             // MO_Mom_Traegheit_Summe - not read by the 0CQ
        rx_message_chs.data[5] = 0xFA;             // MO_Mom_Soll_gefiltert - not read by the 0CQ
        // b6/b7 only carry MO_Mom_Schub (never used by the 0CQ) and b7 bit 5
        // MO_Status_Normalbetrieb (fault-monitor enable only). They used to follow
        // the lock, which flipped bit 61 with the lock value - the "massive effect"
        // seen on the bench came from ESP_14 changing in the same cycle (spec D3).
        // Now fixed at the values that were sent at 100 % lock.
        rx_message_chs.data[6] = 0xFA; // MO_Mom_Schub low bits (not used by the 0CQ)
        rx_message_chs.data[7] = 0xFA; // MO_Mom_Schub high bits + b7.5 Normalbetrieb = 1
      }
      else
      {
        // ---- BPK packing (Fix Hunting on; needed for 554K @ partial lock) ----
        fill_motor11_bpk(rx_message_chs.data, MOTOR_11_counter);
      }

      rx_message_chs.data[0] = calcChecksum(rx_message_chs.data, ID_SEQ_0A7); // for 0x0A7

      MOTOR_11_counter++;
      if (MOTOR_11_counter > 0x4F)
      {
        MOTOR_11_counter = 0x40;
      }
      break;

    case ESP_14:                               // ESP_14 0x08A - THE lock lever on the 0CQ (OEM code: spec_gen5_0cq §2)
      rx_message_chs.data[0] = 0x00;           // checksum placeholder (the 0CQ checks no CRC - spec §1 - but the car's other modules may)
      rx_message_chs.data[1] = ESP_14_counter; // rolling - 0x10>0x1F (not read by the 0CQ)
      rx_message_chs.data[2] = 0x00;           // not read by the 0CQ
      rx_message_chs.data[3] = 0x00;           // BR_Status_Quer_ESP (b3 bits 3-5) - VAQ only, not read by the 0CQ

      // V11.00.0 (OEM code, spec_gen5_0cq §2.2, D1/D2): byte 6 (Allrad_Min) =
      // byte 7 (Allrad_Max) = V, 0..250. Min = Max is the 0CQ's "exact" mode: the
      // clutch target is exactly V (0.4 % per step) and the unit's own Motor_11
      // strategy is ignored. 0 = clutch open. The old 0..0xFE scale with Min = 0
      // only set a ceiling, and at >= 98 % (0xFB..0xFE) the 0CQ read it as "no
      // ESP request" and fell back to its own strategy.
      // The Launch PWM floor (esp14MinFloorPct) is no longer used here: Min is
      // already pinned to Max, which is the most a floor could ever do.
      // (Danger Zone used to pin Min here too. Its Motor_11 ceiling still works,
      // but in exact mode the 0CQ follows ESP_14 - see fill_motor11_bpk.)
      appliedTorque = gen5_esp14_lever_byte();

      rx_message_chs.data[4] = appliedTorque; // BR_Vorg_Quer_Min - not read by the 0CQ (VAQ sets its own below)
      rx_message_chs.data[5] = appliedTorque; // BR_Vorg_Quer_Max - not read by the 0CQ
      rx_message_chs.data[6] = appliedTorque; // BR_Vorg_Allrad_Min = V (exact mode)
      rx_message_chs.data[7] = appliedTorque; // BR_Vorg_Allrad_Max = V

      if (haldexGeneration == 52)
      {
        // VAQ (bench 2026-09-17, see Gen5_0CQ_VAQ_frames10): the front lock
        // follows BR_Vorg_Quer_Min 1:1 in plain percent (0.4 %/bit) and only
        // while BR_Status_Quer_ESP >= 3. The Allrad bytes (b6/b7) are not read
        // (OEM code, spec_gen5_0cq §8.8: the VAQ only stores ESP_14 bytes 0-5).
        // V11 TODO (bench): the bench says "plain percent 1:1", but this still goes
        // through the old 0CQ correction curve. Try quer = gen5_esp14_lever_byte()
        // (same 0..250 scale, linear) once a VAQ is back on the bench.
        const uint8_t quer = get_lock_target_adjusted_value(250, false);
        rx_message_chs.data[3] = quer ? 0x20 : 0x00; // 4 = ESP requests cross lock / 0 deactivated
        rx_message_chs.data[4] = quer;               // BR_Vorg_Quer_Min
        rx_message_chs.data[5] = quer;               // BR_Vorg_Quer_Max = Min
      }

      rx_message_chs.data[0] = calcChecksum(rx_message_chs.data, ID_SEQ_08A); // for 0x08A

      ESP_14_counter++;
      if (ESP_14_counter > 0x1F)
      {
        ESP_14_counter = 0x10;
      }
      break;

    case LWI_01:
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

    case ESP_10:
      rx_message_chs.data_length_code = 8;                                    // DLC 8
      rx_message_chs.data[0] = 0x00;                                          // checksum placeholder
      rx_message_chs.data[1] = ESP_10_counter;                                // rolling - 0x00>0x0F
      rx_message_chs.data[2] = 0x01;                                          // no affect all these affect, find which one
      rx_message_chs.data[3] = 0x04;                                          // no effect sometimes 0xC0, sometimes 0x00
      rx_message_chs.data[4] = 0x00;                                          // no effect sometimes 0x3A, somtimes 0x39
      rx_message_chs.data[5] = 0x40;                                          // no effect
      rx_message_chs.data[6] = 0x00;                                          // no effect
      rx_message_chs.data[7] = 0xFF;                                          // old bench note said "affects a good 40%" - but the 0CQ does not receive ESP_10 at all (OEM code: spec_gen5_0cq D9), so that was something else changing
      rx_message_chs.data[0] = calcChecksum(rx_message_chs.data, ID_SEQ_116); // for 0x116

      ESP_10_counter++;
      if (ESP_10_counter > 0x0F)
      {
        ESP_10_counter = 0x00;
      }
      break;

    case ESP_05: // ESP_05 0x106
      // The 0CQ reads the brake pressure here (bits 16-25 = b2 + b3 bits 0-1, OEM
      // code: spec_gen5_0cq §3). V11.00.0: the car's real brake pressure, the
      // driver-braking bit (b3.2) and b7 are kept as the car sent them - this edit
      // used to overwrite them with a fixed "not braking" value (spec D7,
      // protective). Block 6 is OFF by default since V11. Follow Brake / Invert
      // Brake still rewrite b3.2 afterwards (applyBrakeHandbrakeCANOverride).
      rx_message_chs.data_length_code = 8;     // DLC 8
      rx_message_chs.data[0] = 0x00;           // checksum
      rx_message_chs.data[1] = ESP_05_counter; // rolling - 0x80>0x8F
      // data[2] and data[3] left as the car sent them: brake pressure + ESP_Fahrer_bremst
      rx_message_chs.data[4] = 0x00; // not read by the 0CQ
      rx_message_chs.data[5] = 0x00; // not read by the 0CQ
      rx_message_chs.data[6] = 0xFD; // not read by the 0CQ
      // data[7] left as the car sent it (ESP_Verz_EPB_aktiv b7.2 etc.)
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
      // b7 bits 0/1/4/5 = ABS braking / ASR / EDS / ESP intervention, read by the 0CQ
      // (OEM code: spec_gen5_0cq §3). V11.00.0: kept as the car sent them - this
      // edit used to clear them, hiding a real ABS stop from the Haldex.
      // data[7] left as the car sent it
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
      // b2 bit 7 = KBI_Handbremse, read by the 0CQ (spec_gen5_0cq §3). V11.00.0: the
      // car's real handbrake bit is kept (it used to be cleared); the rest of b2 is 0x02.
      rx_message_chs.data[2] = (rx_message_chs.data[2] & 0x80) | 0x02; // keep the car's handbrake bit (b2.7)
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
    }
  }
}
