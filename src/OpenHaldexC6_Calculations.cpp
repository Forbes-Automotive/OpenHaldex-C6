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
static inline bool lock_enabled()
{
  const bool throttle_ok = (state.pedal_threshold == 0) || (int(received_pedal_value) >= state.pedal_threshold);
  // Allow lock only within the window [disengageUnderSpeed, disengageAboveSpeed].
  const bool under_ok = (disengageUnderSpeed == 0) || (received_vehicle_speed >= disengageUnderSpeed);
  const bool above_ok = (disengageAboveSpeed == 0) || (received_vehicle_speed <= disengageAboveSpeed);
  return throttle_ok && under_ok && above_ok;
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

float get_lock_target_adjustment()
{
  s_steer_applied_flag = false; // reset; apply_steering_scale sets it when used this cycle
  // const MODE_NAMES = ['Stock', 'FWD', '50:50', '60:40', '75:25', 'Expert']; // mode names as Strings - just to note here
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
    const uint8_t priority = (forceModesPriority < 6) ? forceModesPriority : 0;

    for (uint8_t i = 0; i < 3; i++)
    {
      const uint8_t idx = priorityOrders[priority][i];
      if (triggers[idx].enabled && triggers[idx].flag)
      {
        fmv = triggers[idx].value;
        anyActive = true;
        break;
      }
    }

    if (anyActive)
    {
      // Forced lock modes go through the same speed/throttle gate as the
      // selected mode below. Previously they returned the raw lock here, so
      // lock_target (and the dashboard "Requested" figure) read 100% below the
      // under-speed cut-off even though the frame values were being zeroed.
      switch (fmv)
      {
      case 0:
        return received_haldex_engagement; // stock -> passthrough (mirror actual engagement, don't force open)
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
    bool found = false;
    for (uint8_t i = 0; i <= 100; i++)
    {
      if (haldexLearnTable[i] >= (uint8_t)lock_target)
      {
        correction_factor = i;
        found = true;
        break;
      }
    }
    if (!found)
    {
      // requested lock exceeds anything the sweep learned - clamp to the highest learned entry
      uint8_t best_engagement = 0;
      for (uint8_t i = 0; i <= 100; i++)
      {
        if (haldexLearnTable[i] >= best_engagement)
        {
          best_engagement = haldexLearnTable[i];
          correction_factor = i;
        }
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
    // Legacy behaviour: all four corners driven from the free-running counters.
    const uint8_t hlLo = get_lock_target_adjusted_value(ESP_19_counter2, false);
    const uint8_t hlHi = get_lock_target_adjusted_value(ESP_19_counter, false);
    const uint8_t vlLo = get_lock_target_adjusted_value(ESP_19_counter2 + 0xBA, false);

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

void fill_motor11_bpk(uint8_t data[8], uint8_t counter)
{
  // DBC-correct bit packing for Motor_11 (0x0A7). Every field below is a
  // runtime tunable (see defs.h) so the serial lab can massage the wire
  // format live; the defaults are the values that were hardcoded here.
  // Signals are 10-bit with offset -509, i.e. raw = Nm + 509.
  const uint16_t ceilNm = bpkCeilingNm;
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
  const uint32_t settleMs = 400;
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
  longLearnStartMs = millis();
  longLearnEndMs = 0;
  memset(longLearnBlockResult, 0, sizeof(longLearnBlockResult));
  longLearnActive = true;

  xTaskCreate(longLearnTask, "longLearn", 6144, nullptr, 1, nullptr);
  return true;
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

    // ---- Transferred from standalone (gated off by default) ----------------
    case BRAKES4_ID:
      rx_message_chs.data[0] = 0x00;
      rx_message_chs.data[1] = 0x00;
      rx_message_chs.data[2] = 0x00;
      rx_message_chs.data[3] = 0x00;
      rx_message_chs.data[4] = 0x00;
      rx_message_chs.data[5] = 0x00;
      rx_message_chs.data[6] = BRAKES4_counter;
      rx_message_chs.data[7] = BRAKES4_counter;
      BRAKES4_counter = BRAKES4_counter + 10;
      if (BRAKES4_counter > 0xF0)
        BRAKES4_counter = 0;
      break;
    case BRAKES5_ID:
      rx_message_chs.data[0] = 0xFE;
      rx_message_chs.data[1] = 0x7F;
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
    case BRAKES9_ID:
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
      rx_message_chs.data[0] = 0x00;
      rx_message_chs.data[1] = 0x30;
      rx_message_chs.data[2] = 0x00;
      rx_message_chs.data[3] = 0x0A;
      rx_message_chs.data[4] = 0x0A;
      rx_message_chs.data[5] = 0x10;
      rx_message_chs.data[6] = 0xFE;
      rx_message_chs.data[7] = 0xFE;
      break;
    case mLW_1:
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
      rx_message_chs.data[0] = 0x00;
      rx_message_chs.data[1] = 0x02;
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
      if (haldexLearnActive || state.mode != MODE_5050)
      {
        appliedTorque = get_lock_target_adjusted_value(0x7F, false); // regulated clamp
      }
      else
      {
        appliedTorque = get_lock_target_adjusted_value(0xFE, false); // full clamp (27 bar)
      }

      rx_message_chs.data[0] = appliedTorque;
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

    // ---- Transferred from standalone (gated off by default) ----------------
    case mKombi_1:
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

    // ---- Inactive frames restored (gated off by default via frameEditMask) --
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
      if (BRAKES5_counter > 0xF0)
        BRAKES5_counter = 0;
      BRAKES5_counter2 = BRAKES5_counter2 + 10;
      if (BRAKES5_counter2 > 0xF3)
        BRAKES5_counter2 = 3;
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
      if (++BRAKES8_counter > 0x8F)
        BRAKES8_counter = 0x80;
      if (++BRAKES8_counter1 > 0x0F)
        BRAKES8_counter1 = 0x00;
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
      if (BRAKES2_counter > 0xF0)
        BRAKES2_counter = 0;
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
      fill_esp19_wheel_speeds(rx_message_chs.data);
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
        fill_motor11_bpk(rx_message_chs.data, MOTOR_11_counter);
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

      appliedTorque = get_lock_target_adjusted_value(0xFE, false);

      // Launch PWM floor: raise BR_Vorg_*_Min while lock is commanded, clamped
      // strictly below Max so the Haldex keeps room to modulate. 0% = unchanged,
      // and it collapses to 0 whenever Max does (off-throttle, FWD, coasting).
      // Adopted from OpenHaldex-Edge by Rekt (Kile Thomson) - see THIRD_PARTY_NOTICES.md.
      {
        uint8_t esp14Floor = 0;
        if (esp14MinFloorPct > 0 && appliedTorque > 1)
        {
          uint16_t f = ((uint16_t)appliedTorque * esp14MinFloorPct) / 100;
          if (f > (uint16_t)(appliedTorque - 1))
            f = (uint16_t)(appliedTorque - 1);
          esp14Floor = (uint8_t)f;
        }
        // Danger Zone: at a full 50:50 request only, pin Min to Max so the
        // Haldex has no modulation room and goes to full pump duty.
        if (dangerZoneEnabled && lock_target >= 100 && appliedTorque > 1)
          esp14Floor = (uint8_t)(appliedTorque - 1);
        rx_message_chs.data[4] = esp14Floor; // BR_Vorg_Quer_Min
        rx_message_chs.data[6] = esp14Floor; // BR_Vorg_Allrad_Min
      }

      rx_message_chs.data[5] = appliedTorque; // BR_Vorg_Quer_Max - lock-modulated (massive effect, ported from standalone)
      rx_message_chs.data[7] = appliedTorque; // BR_Vorg_Allrad_Max - lock-modulated (massive effect)
      // massive effects (4>7)

      if (haldexGeneration == 52)
      {
        // VAQ (bench 2026-09-17, see Gen5_0CQ_VAQ_frames10): the front lock
        // follows BR_Vorg_Quer_Min 1:1 in plain percent (0.4 %/bit) and only
        // while BR_Status_Quer_ESP >= 3. The Allrad bytes (b6/b7) are not read.
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
    }
  }
}
