#include <OpenHaldexC6_Calculations.h>
#include <OpenHaldexC6_tasks.h>
#include <math.h> // tanf/sqrtf/fabsf for the per-corner slip geometry

// ---- Defines -------------------------------------------------------------------

// PT on/off with hysteresis (V11.00.12): on at >= 55 % PT share, off at <= 45 %,
// and at least 250 ms in a state before it may change again - so a car sitting on
// the boundary between a PT cell and a number cell doesn't flick between the
// car's frames and ours every cycle. Called from stockEffective(), which runs in
// the CAN task, the frame task and the web task, so the state sits behind a lock.
#define EXPERT_PT_ON_SHARE 0.55f
#define EXPERT_PT_OFF_SHARE 0.45f
#define EXPERT_PT_DWELL_MS 250
// ---- Gen2 (7H0907554 / 1K0907554B) demand bytes (V11.00.0) ----
// OEM code (documents/haldex-binaries/leaf_trace/spec_gen2_7h0.md §2.3): Bremse_4
// (0x2A0) byte 0 is a CEILING on the clutch request - it can only pull the
// request DOWN. So the rest of the bus has to ask for at least as much lock as
// the lock request: Motor_1 inner torque (b1) and pedal (b5) at maximum, loss torque
// (b6) at 0, and in standalone a little front-wheel slip on Bremse_3. Then the
// lock request byte alone sets how much the clutch closes (0x7F = open .. 0xFD = full).
// 1K0 note: same frames we send; but the 1K0 uses 0x540 instead of 0x440 as its final-request
// scale, so its full-valve byte (0xEA vs 0xD6), its read-back top (~0xEC vs ~0xD7) and its 0x2C0 DLC
// (4 vs 8) differ [REA audit_gen2_7h0 #60/#61/F3] - covered by one shared range (gen2LockRequestByte, GEN2_REQUEST_FULL).
// Demand bytes [KM Motor_1.inneres_Motor_Moment / Fahrpedalwert_oder_Drosselklapp / Bremse_3.Radgeschw__VL/VR]
// [REA audit_gen2_7h0 #34/#36/#45/#52: the request E3F6 = min(base, E2A4, E34A ceiling, E2D2) - demand must exceed the ceiling].
#define GEN2_BREMSE4_BLOCK_BIT 5 // Bremse_4 (0x2A0) bit in the Gen2 frameEditBlocks rows
// ---- Gen1 (02D900554B) lock request (V11.00.0) ----
// OEM code (documents/haldex-binaries/leaf_trace/spec_gen1_02d.md §2): the 02D
// works out its own clutch request from Motor_1 (torque, pedal, rpm), its
// tables and the wheel speeds. Bremse_4 (0x2A0) byte 0 is then a CEILING on that
// request, applied at the very end:  ceiling = (byte0 - 0x80) x 192.
//   0x7F        -> ceiling 0, clutch forced open straight away (no ramp)
//   0x81..0xC9  -> straight-line partial lock, 0xA5 = about 50 %
//   0xCA..0xFF  -> above the biggest request the unit ever makes = full lock
//   0x00..0x7E  -> no ceiling at all
// So the ceiling only works as a lock request while Motor_1 asks for MAXIMUM demand.
// [KM Bremse_4.ABS_Vorgabewert_mitte_Kupplungs (3,-381) Nm/min - DBC scale not what the unit does]
// [REA audit_gen1_02d #57/#58/#68: 0x7F -> 0, 0x80 -> [0x6356] = 19, >0x80 -> (b0-0x80) x 192, <0x7F none; no timeout (#12)]
#define GEN1_BREMSE4_BLOCK_BIT 4   // frameEditBlocks Gen1 bit for Bremse_4 (0x2A0)

// ---- Statics -------------------------------------------------------------------

static portMUX_TYPE expertPtMux = portMUX_INITIALIZER_UNLOCKED;
static bool expertPt = false;
static uint32_t expertPtChangedMs = 0;

// Last throttle override worked out (0-100 %), kept for the web page's live display.
static float steerThrottleOverride = 0.0f;

// Scale a lock target by the steering-angle curve. Applied to lock-producing
// modes only (not Stock passthrough or FWD).
// Telemetry captured for the UI engagement-split display: last requested
// (pre-scale) and applied (post-scale) lock, and whether scaling ran this cycle.
static float steerRequested = 0.0f;
static float steerApplied = 0.0f;
static bool steerAppliedFlag = false;

// ---- Small helpers -------------------------------------------------------------

// True when none of the three settings is holding lock off.
static bool lockEnabled()
{
  return (lockGateBlocking() == 0);
}

// The mode actually in use right now: a force trigger wins over the selected mode.
static int modeInUse()
{
  const int forced = getForcedModeValue();
  return (forced >= 0) ? forced : (int)state.mode;
}

// How much of the steering-angle lock reduction the throttle cancels (0-100 %).
// 0 = full steering reduction (light throttle, turn-in agility).
// 100 = no steering reduction (hard acceleration or counter-steering - hold full lock).
// In between it rises in a straight line from the start % to the full %.
static float getSteeringThrottleOverride()
{
  if (!steeringThrottleOverrideEnabled)
  {
    return 0.0f; // feature off: the steering curve works as before
  }

  float throttle = receivedPedalValue;                      // pedal position, % (0 when chassis CAN is lost)
  float startPercent = (float)steeringThrottleOverrideStart; // pedal % where the override starts
  float fullPercent = (float)steeringThrottleOverrideFull;   // pedal % where the override is complete

  if (throttle <= startPercent)
  {
    return 0.0f; // light throttle: no override, keep the steering reduction
  }
  if (throttle >= fullPercent)
  {
    return 100.0f; // hard on the throttle: cancel the steering reduction completely
  }

  float range = fullPercent - startPercent; // always above 0 here (throttle is between the two)
  return (throttle - startPercent) * 100.0f / range;
}

// Look up the learn table: the smallest correction factor (0-100) whose learned
// engagement reaches the wanted engagement. If nothing the sweep learned gets
// there, use the entry that gave the most engagement.
static uint8_t learnTableCorrectionFactor(uint8_t wantedEngagement)
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

// True when the Haldex is getting our Bremse_4 lock request byte: the Bremse_4 block
// is on, so we send the whole frame ourselves (both modes since V11.00.7 - the
// lock request no longer depends on the car sending a 0x2A0 for us to edit).
static bool gen2LockRequestOnBus()
{
  return frameEditEnabled(FE_GEN_2, GEN2_BREMSE4_BLOCK_BIT);
}
// Geometry-compensated per-corner slip. Adopted from OpenHaldex-Edge by Rekt
// (Kile Thomson) - https://github.com/Kile-Thomson/OpenHaldex-Edge - see
// THIRD_PARTY_NOTICES.md. At steering road-wheel angle delta,
// pure Ackermann puts the turn centre on the rear-axle line R = wheelbase /
// tan(delta) from the centreline. Each wheel traces its own radius, so with
// zero real slip the wheel speeds are proportional to those radii. We build the
// four geometric radii, turn them into expected speeds sharing the measured
// mean, and report each corner's fractional excess as slip. On a straight this
// reduces to "speed vs. the average of the others" with no special-casing.
bool computeCornerSlip(const uint16_t wheelRaw[4], int16_t steerWheelTenths,
                         float steeringRatio, uint16_t wheelbaseMm,
                         uint16_t trackFrontMm, uint16_t trackRearMm,
                         uint16_t minSpeedRaw, int8_t slipOut[4])
{
  slipOut[0] = slipOut[1] = slipOut[2] = slipOut[3] = 0;

  if (wheelbaseMm == 0 || trackFrontMm == 0 || trackRearMm == 0 ||
      steeringRatio < 1.0f)
    return false;

  const uint32_t sumRaw =
      (uint32_t)wheelRaw[0] + wheelRaw[1] + wheelRaw[2] + wheelRaw[3];
  const float meanActual = sumRaw * 0.25f;
  if (meanActual < (float)minSpeedRaw || meanActual <= 0.0f)
    return false;

  const float L = (float)wheelbaseMm;
  const float halfTf = (float)trackFrontMm * 0.5f;
  const float halfTr = (float)trackRearMm * 0.5f;

  const float deltaDeg = ((float)steerWheelTenths * 0.1f) / steeringRatio;
  const float delta = fabsf(deltaDeg) * (float)M_PI / 180.0f;

  float rad[4]; // [FL, FR, RL, RR]
  const float kStraight = 0.0005f;
  if (delta < kStraight)
  {
    rad[0] = rad[1] = rad[2] = rad[3] = 1.0f;
  }
  else
  {
    float R = L / tanf(delta);
    const float minRadius = (halfTf > halfTr ? halfTf : halfTr) + 1.0f;
    if (R < minRadius)
      R = minRadius;

    const float rRearInner = R - halfTr;
    const float rRearOuter = R + halfTr;
    const float rFrontInner = sqrtf(L * L + (R - halfTf) * (R - halfTf));
    const float rFrontOuter = sqrtf(L * L + (R + halfTf) * (R + halfTf));

    if (deltaDeg > 0.0f)
    {
      // Turning right: right-side wheels are inner (shorter radius, slower).
      rad[0] = rFrontOuter; // FL
      rad[1] = rFrontInner; // FR
      rad[2] = rRearOuter;  // RL
      rad[3] = rRearInner;  // RR
    }
    else
    {
      rad[0] = rFrontInner; // FL
      rad[1] = rFrontOuter; // FR
      rad[2] = rRearInner;  // RL
      rad[3] = rRearOuter;  // RR
    }
  }

  const float meanRad = (rad[0] + rad[1] + rad[2] + rad[3]) * 0.25f;
  if (meanRad <= 0.0f)
    return false;

  for (int i = 0; i < 4; i++)
  {
    const float expected = meanActual * (rad[i] / meanRad);
    if (expected <= 0.0f)
    {
      slipOut[i] = 0;
      continue;
    }
    float slipPct = ((float)wheelRaw[i] / expected - 1.0f) * 100.0f;
    if (slipPct > 127.0f)
      slipPct = 127.0f;
    else if (slipPct < -100.0f)
      slipPct = -100.0f;
    slipOut[i] = (int8_t)(slipPct < 0.0f ? slipPct - 0.5f : slipPct + 0.5f);
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
// lockGateBlocking() says WHICH of the three settings is holding lock off
// right now (LOCK_GATE_* bits, 0 = none), so the dashboard can name it.
// lockEnabled() is built on it, so the reason shown can never disagree with
// what the lock maths actually did. Same tests as before (V10.00.2, from PR #54
// by Luca C: a Gen5 0AY owner saw "FWD in every mode" with only "Target: 0%"
// on screen, caused by an old Disengage Under Speed setting).
uint8_t lockGateBlocking()
{
  uint8_t blocking = 0; // no gate blocking yet

  // Minimum Throttle Before Lock: 0 = off, otherwise throttle must reach it
  if (state.pedalThreshold != 0 && int(receivedPedalValue) < state.pedalThreshold)
  {
    blocking = blocking | LOCK_GATE_THROTTLE;
  }

  // Disengage Under Speed: 0 = off, otherwise no lock below it (car parks)
  if (disengageUnderSpeed != 0 && receivedVehicleSpeed < disengageUnderSpeed)
  {
    blocking = blocking | LOCK_GATE_UNDER_SPEED;
  }

  // Disengage Above Speed: 0 = off, otherwise no lock above it (motorways)
  if (disengageAboveSpeed != 0 && receivedVehicleSpeed > disengageAboveSpeed)
  {
    blocking = blocking | LOCK_GATE_ABOVE_SPEED;
  }

  // Follow Handbrake (V11.00.0): handbrake pulled -> clutch open, as the Haldex
  // does on its own. Needed because our lock request outranks the unit's own
  // brake + handbrake handling on the direct-lock request units.
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

// 2D throttle/speed map lookup at the live point. lockOut = the blended lock %
// from the NUMBER cells only (PT cells left out and the rest re-weighted, so the
// lock still fades smoothly up to the edge of a PT area). ptShareOut = how much
// of the blend (0..1) sits on PT cells - expertPtActive() decides PT from it.
static void expertMapLookup(float *lockOut, float *ptShareOut)
{
  float throttle = receivedPedalValue;
  throttle = constrain(throttle, 0, 100); // fix throttle from 0-100

  float speed = receivedVehicleSpeed; // fix speed from 0-300 kmh (should be plenty)
  speed = constrain(speed, 0, 300);

  uint8_t t0 = 0; // index for throttle array
  uint8_t t1 = 0; // index for throttle array
  float tRatio = 0;

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
        tRatio = (denom > 0) ? ((throttle - throttleArray[t0]) / denom) : 0; // if denom is zero, just use t0 value (ratio of 0), otherwise calculate ratio between t0 and t1
        break;
      }
    }
  }

  uint8_t s0 = 0; // index for speed array
  uint8_t s1 = 0; // index for speed array
  float sRatio = 0;

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
        sRatio = (denom > 0) ? ((speed - speedArray[s0]) / denom) : 0; // if denom is zero, just use s0 value (ratio of 0), otherwise calculate ratio between s0 and s1
        break;
      }
    }
  }

  // Bilinear weights of the four cells around the point (they add up to 1; at
  // the top of an axis the same cell appears twice, which is still correct).
  const uint8_t ct[4] = {t0, t0, t1, t1};
  const uint8_t cs[4] = {s0, s1, s0, s1};
  const float w[4] = {(1.0f - sRatio) * (1.0f - tRatio), sRatio * (1.0f - tRatio),
                      (1.0f - sRatio) * tRatio, sRatio * tRatio};

  float ptWeight = 0.0f;
  float numWeight = 0.0f;
  float numSum = 0.0f;
  for (uint8_t i = 0; i < 4; i++)
  {
    if (lockCellPt(ct[i], cs[i]))
    {
      ptWeight += w[i];
    }
    else
    {
      numWeight += w[i];
      const uint8_t cell = lockArray[ct[i]][cs[i]];
      numSum += w[i] * (float)((cell > 100) ? 100 : cell);
    }
  }

  float v = (numWeight > 0.0f) ? (numSum / numWeight) : 0.0f; // all four PT: no number to give
  *lockOut = constrain(v, 0, 100);
  *ptShareOut = ptWeight; // weights add up to 1, so this is already the share
}

static bool expertPtUpdate(bool expertInUse)
{
  float lock = 0.0f;
  float share = 0.0f;
  const bool anyPt = expertInUse && ((lockPTMask & LOCK_PT_ALL) != 0);
  if (anyPt)
  {
    expertMapLookup(&lock, &share);
  }

  const uint32_t now = millis();
  portENTER_CRITICAL(&expertPtMux);
  if (!anyPt)
  {
    expertPt = false; // not Expert, or no PT cells: off straight away
  }
  else if ((now - expertPtChangedMs) >= EXPERT_PT_DWELL_MS)
  {
    const bool want = expertPt ? (share > EXPERT_PT_OFF_SHARE) : (share >= EXPERT_PT_ON_SHARE);
    if (want != expertPt)
    {
      expertPt = want;
      expertPtChangedMs = now;
    }
  }
  const bool result = expertPt;
  portEXIT_CRITICAL(&expertPtMux);
  return result;
}

bool expertPtActive()
{
  return expertPtUpdate(modeInUse() == MODE_EXPERT);
}

static float getExpertLockTarget()
{
  if (expertPtActive())
  {
    return 0; // PT area: the car's own strategy (normal mode) / no forced lock (standalone)
  }
  float lock = 0.0f;
  float share = 0.0f;
  expertMapLookup(&lock, &share);
  return int(lock); // lock target as an integer percentage (0-100)
}

// Steering-angle third axis (FWD bias): returns a 0-100% multiplier for the
// lock target based on |steering-wheel angle|. Max lock at low angle, reduced
// lock as angle grows. Only gens with a steering source (2/4/50/52) use it;
// unsupported gens or stale/absent steering data return 100 (no reduction).
// This is the curve on its own - getSteeringLockScale() below adds the
// throttle override on top.
static float getSteeringCurveScale()
{
  const bool supported = (haldexGeneration == 2 || haldexGeneration == 4 ||
                          haldexGeneration == 50 || haldexGeneration == 52);
  if (!supported)
    return 100.0f;
  if (!steeringScaleEnabled)
    return 100.0f; // feature disabled -> full lock, no reduction
  if (receivedSteeringMs == 0 || (millis() - receivedSteeringMs) > steeringStaleMs)
    return 100.0f; // no/stale steering -> full lock, bias off

  float angle = fabsf(receivedSteeringAngle);
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

// The steering-angle multiplier actually used (0-100 %): the steering curve,
// with the throttle override fading its reduction back out towards 100 %.
// Example: curve says 50 % at 180 deg, throttle override 50 % -> 75 %.
static float getSteeringLockScale()
{
  float curveScale = getSteeringCurveScale();        // what the steering curve alone asks for
  float throttleOverride = getSteeringThrottleOverride(); // how much of the reduction the throttle cancels
  steerThrottleOverride = throttleOverride;          // keep for the live display

  float reduction = 100.0f - curveScale;                              // how much lock the curve takes away
  float finalScale = curveScale + (reduction * throttleOverride / 100.0f); // give back the throttle's share of it
  return constrain(finalScale, 0, 100);
}

static float applySteeringScale(float lock)
{
  const float scaled = lock * getSteeringLockScale() / 100.0f;
  steerRequested = lock;
  steerApplied = scaled;
  steerAppliedFlag = true;
  return scaled;
}

// Steering-scale telemetry accessors (used by the API live-status endpoint).
bool steeringScaleIsActive()
{
  return steerAppliedFlag && (steerApplied + 0.5f < steerRequested);
}
uint8_t steeringScaleRequestedPct()
{
  return (uint8_t)constrain((int)(steerRequested + 0.5f), 0, 100);
}
uint8_t steeringScaleResultPct()
{
  return (uint8_t)constrain((int)(steerApplied + 0.5f), 0, 100);
}

// How much of the steering reduction the throttle is cancelling right now (0-100 %).
// 0 when no lock mode is running the steering scale this cycle.
uint8_t steeringThrottleOverridePct()
{
  if (!steerAppliedFlag)
  {
    return 0; // steering scale not in use this cycle (Stock, FWD or lock gated off)
  }
  return (uint8_t)constrain((int)(steerThrottleOverride + 0.5f), 0, 100);
}

// The mode the car's drive mode select (MQB Charisma) is forcing, or -1 for none.
// The drive profile is read from Charisma_07 / Charisma_01 in parseCANChs.
int getDriveModeForcedValue()
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
  if (forcedMode >= (uint8_t)openHaldexMode_t_MAX)
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
// Exposed so the inline gateway (parseCANChs) can detect "effective mode is
// Stock" and pass frames through untouched instead of editing them.
int getForcedModeValue()
{
  if (!(extBtnForceMode || tcForceMode || hazardForceMode))
  {
    return getDriveModeForcedValue(); // no button triggers turned on: only the drive mode can force
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
  return getDriveModeForcedValue(); // no button trigger active: the drive mode select comes last
}

float getLockTargetAdjustment()
{
  steerAppliedFlag = false; // reset; applySteeringScale sets it when used this cycle

  // The return value is a torque COMMAND: standalone packs it into the frames it
  // synthesizes, and the inline gateway packs it into edited chassis frames. It
  // must never mirror receivedHaldexEngagement back - engagement 100
  // commanding 100 closes a positive feedback loop that latches the clutch shut
  // until FWD or a power cycle. Stock as a command is zero forced lock; true
  // stock PASSTHROUGH is the caller's job (parseCANChs skips frame edits
  // entirely when the effective mode is Stock).
  // const MODE_NAMES = ['Stock', 'FWD', '50:50', '60:40', '75:25', 'Expert'];
  const int forced = getForcedModeValue();
  if (forced >= 0)
  {
    // Forced lock modes go through the same speed/throttle gate as the
    // selected mode below, so lockTarget (and the dashboard "Requested"
    // figure) never reads 100% below the under-speed cut-off.
    switch (forced)
    {
    case 0:
      return 0; // Stock -> no forced lock (passthrough is handled by the caller)
    case 1:
      return 0; // FWD
    case 2:
      return lockEnabled() ? applySteeringScale(100) : 0; // 50:50
    case 3:
      return lockEnabled() ? applySteeringScale(40) : 0; // 60:40
    case 4:
      return lockEnabled() ? applySteeringScale(30) : 0; // 75:25
    case 5:
      return lockEnabled() ? applySteeringScale(getExpertLockTarget()) : 0; // Expert
    default:
      return 0; // error - zero lock
    }
  }

  // getting here means no external influences - handle each mode and calculate lock
  switch (state.mode)
  {
  case MODE_STOCK:
    return 0; // Stock -> no forced lock. Standalone has nothing to pass through, so Stock is an open clutch; inline Stock never reaches here (parseCANChs forwards frames untouched).

  case MODE_FWD:
    return 0; // zero lock

  case MODE_5050:
    if (lockEnabled())
    {
      return applySteeringScale(100); // 100% lock
    }
    return 0; // lock not enabled, zero lock

  case MODE_6040:
    if (lockEnabled())
    {
      return applySteeringScale(40); // 40% lock
    }
    return 0; // lock not enabled, zero lock

  case MODE_7525:
    if (lockEnabled())
    {
      return applySteeringScale(30); // 30% lock
    }
    return 0; // lock not enabled, zero lock

  case MODE_EXPERT:
    if (lockEnabled())
    {
      return applySteeringScale(getExpertLockTarget());
    }
    return 0; // lock not enabled, zero lock

  default:
    return 0; // error - zero lock
  }
}

// ===== Direct-lock request units (V11.00.0) =====
// The OEM code (documents/haldex-binaries/CONTROL_AND_DTC_SPEC.md) shows most
// VAG units take a straight clutch-torque request in one byte - Bremse_4
// (0x2A0) byte 0 on Gen1 / Gen2 / Gen4 / 0AY, ESP_14 (0x08A) bytes 6-7 on 0CQ.
// Before V11 we scaled 0x7F by a correction factor, which lands BELOW 0x7F
// ("invalid" - the unit ignores it and runs its own strategy) and at 100 %
// sent 0x7F itself ("zero request" - clutch OPEN when full lock was asked for).

// How much lock (0-100 %) to ask a direct-lock request unit for right now.
// Learn: the sweep's correction factor IS the lock request position, so the learn
// table measures the lock request directly. Otherwise lockTarget, corrected through
// the learn table if one has been saved. 0 whenever the lock gate is closed
// (speed / throttle settings, handbrake, ABS braking).
uint8_t lockRequestPercent()
{
  if (haldexLearnActive)
  {
    return haldexLearnCF; // learn sweeps the lock request itself, 0..100
  }

  if (lockTarget <= 0)
  {
    return 0; // FWD / Stock / nothing asked for
  }

  if (!lockEnabled())
  {
    return 0; // a lock gate is holding lock off
  }

  if (haldexLearnTableValid)
  {
    return learnTableCorrectionFactor((uint8_t)lockTarget); // measured lock request -> engagement curve
  }

  if (lockTarget >= 100)
  {
    return 100;
  }
  return (uint8_t)lockTarget; // the lock request is linear, so no default fudge curve is needed
}

// Turn a lock % into the Bremse_4 byte 0 lock request value used by the PQ units
// [KM Bremse_4.ABS_Vorgabewert_mitte_Kupplungs, byte 0]:
//   0 %       -> 0x7F  valid "no torque" request, the clutch is commanded fully open
//                [REA audit_gen4_0br #14/#21, audit_gen5_0ay #11: 0x7F -> mode 0, target 0]
//   1 - 99 %  -> 0x80 + (percent x fullSteps / 100), so 0x81 .. 0xFC for 125 (linear torque request)
//                [REA both: (b - 0x80) x 112, K1/K2 = 0xE000/0x200; 0BR 1/16 Nm = 7 Nm/step, INF]
//   100 %     -> fullLockByte, each generation passes its own:
//                Gen4 0xFE = "mode 4": last cycle's ramped capacity F1B00 (target 32000),
//                always >= 0xFD's 14000 [REA audit_gen4_0br §8 / #43];
//                0AY 0xFD = top request 14000, because 0xFE = HOLD there [REA audit_gen5_0ay #20/#23]
// Never 0x00-0x7E or 0xFF: those are "invalid" and the unit ignores the byte.
// The same builder runs in OEM and standalone mode (an ON block = our whole frame,
// carFrameToHaldex), so the range is identical in both modes.
uint8_t vagLockRequestByte(uint8_t fullSteps, uint8_t fullLockByte)
{
  uint8_t lockPercent = lockRequestPercent();

  if (lockPercent == 0)
  {
    return 0x7F; // valid zero request: clutch open
  }
  if (lockPercent >= 100)
  {
    return fullLockByte; // top of the range for this unit
  }

  uint16_t step = ((uint16_t)lockPercent * fullSteps + 50) / 100; // 1..99 % -> 1..124 steps for 125 (rounded)
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
// the unit's own ABS handling (OEM code, Gen4 0BR + 0AY: a valid Bremse_4 byte
// puts the unit in "follow the request" state 3, above its own ABS / brake +
// handbrake states 4 / 2; only its own states 5 / 6 (fault reaction) outrank
// it, and the result is still capped by its limits - on the 0BR by the
// Motor_1-rpm-gated F1AFE [REA audit_gen4_0br #26/#28, audit_gen5_0ay #15]),
// so we do the release ourselves. Follow ABS is on by default; turning
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

uint8_t getLockTargetAdjustedValue(uint8_t value, bool invert)
{
  // during learn, use the current learn correction factor
  if (haldexLearnActive)
  {
    uint8_t correctedValue = (uint16_t)value * haldexLearnCF / 100;
    return (invert ? (0xFE - correctedValue) : correctedValue);
  }

  // handle 5050 mode
  if (lockTarget == 100)
  {
    if (lockEnabled())
    {
      return (invert ? (0xFE - value) : value); // if lock enabled, return full value (or inverted), otherwise return 0 (or inverted)
    }
    return (invert ? 0xFE : 0x00);
  }

  // handle FWD mode
  if (lockTarget == 0)
  {
    return (invert ? 0xFE : 0x00);
  }

  uint8_t correctionFactor = 0; // calculate correction factor based on learn table if valid, otherwise use a default formula to determine correction factor (which is not very accurate, but better than nothing)
  if (haldexLearnTableValid)
  {
    correctionFactor = learnTableCorrectionFactor((uint8_t)lockTarget);
  }
  else if (haldexGeneration == 41)
  {
    // Gen41 (Vauxhall/Opel/Buick FDCM) defaults — observed mapping CF→engagement is
    // steeper than the VAG Haldex curve.
    // Linear fit: engagement = 2.2 * CF - 24  →  CF = (target + 24) / 2.2
    correctionFactor = (uint8_t)constrain(((float)lockTarget + 24.0f) / 2.2f, 0, 100);
  }
  else if (isVolvoOrSaabGeneration(haldexGeneration) || haldexGeneration == GEN_FORD_KUGA ||
           haldexGeneration == GEN_LANDROVER_GEN4) // V11.00.13: LR lock request = direct torque T, linear in % (defs.h LR_*)
  {
    // Volvo / SAAB (V10.00.0), Ford Kuga (V11.00.11 - lock % = ESP torque ceiling, linear in torque): no measured curve yet, so straight through -
    // 100 % lock sends the strongest known lock value. Run Learn
    // Haldex once a unit is on the bench to replace this with a real table.
    correctionFactor = (uint8_t)constrain((float)lockTarget, 0, 100);
  }
  else
  {
    // VAG Haldex default — linear fit: engagement = 2 * CF - 20  →  CF = (target + 20) / 2
    correctionFactor = (uint8_t)constrain(((float)lockTarget + 20.0f) / 2.0f, 0, 100);
  }

  uint8_t correctedValue = (uint16_t)value * correctionFactor / 100;
  if (lockEnabled())
  {
    return (invert ? (0xFE - correctedValue) : correctedValue); // if lock enabled, return corrected value (or inverted), otherwise return 0 (or inverted)
  }
  return (invert ? 0xFE : 0x00); // if lock not enabled, return 0 (or inverted)
}

void fillEsp19WheelSpeeds(uint8_t data[8])
{
  // Wheel speed MUST keep changing or the Haldex slowly disengages - a
  // static value fades over time and eventually stops. This is the original,
  // proven keep-alive dither (all 4 corners share the same small swing).
  //
  // A lockTarget-proportional front/rear delta (simulating real slip, like
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
    // wheel-speed method. V11.00.0 (Adam): kept ON alongside the ESP_14 lock request in
    // every non-Stock mode. The OEM code (spec_gen5_0cq D6) shows the 0CQ's own
    // strategy reads these, so the unit never sees real slip while forced; Stock
    // passes the car's real speeds, and the ABS / handbrake gates drop the lock
    // (and so these speeds) to 0. This is the DEFAULT for gen 50 / 52 (spec D6
    // "lock-independent speeds" was deliberately not applied - audit_gen5_0cq F5).
    // The scaling goes through getLockTargetAdjustedValue(), i.e. the old VAG
    // (target + 20) / 2 curve or the learn table - not the linear ESP_14 lock request.
    const uint8_t hlLo = getLockTargetAdjustedValue(ESP_19_counter2, false);        // rear wheel speed low byte
    const uint8_t hlHi = getLockTargetAdjustedValue(ESP_19_counter, false);         // all corners high byte
    const uint8_t vlLo = getLockTargetAdjustedValue(ESP_19_counter2 + 0xBA, false); // front wheel speed low byte

    data[0] = hlLo; // [KM ESP_19.ESP_HL_Radgeschw_02 low, 0.0075 km/h] [REA audit F5 / diag G15: sig118 (RL), read; strategy + 0x2BF0 + clutch-load inputs]
    data[1] = hlHi; // [KM ESP_HL_Radgeschw_02 high] [REA: as b0]
    data[2] = hlLo; // [KM ESP_HR_Radgeschw_02 low] [REA diag G15: sig119 (RR), read]
    data[3] = hlHi; // [KM ESP_HR_Radgeschw_02 high] [REA: as b2]
    data[4] = vlLo; // [KM ESP_VL_Radgeschw_02 low] [REA diag G15: sig116 (FL), read]
    data[5] = hlHi; // [KM ESP_VL_Radgeschw_02 high] [REA: as b4]
    data[6] = vlLo; // [KM ESP_VR_Radgeschw_02 low] [REA diag G15: sig117 (FR), read]
    data[7] = hlHi; // [KM ESP_VR_Radgeschw_02 high] [REA: as b6]
    if (wsLeftRightDeltaRaw != 0)
    {
      // VAQ lock request: split the front axle left vs right around the legacy value.
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
    // VAQ lock request: front left-vs-right split (a transverse lock reacts to VL vs VR).
    int32_t vl = front + wsLeftRightDeltaRaw / 2;
    int32_t vr = front - wsLeftRightDeltaRaw / 2;
    if (rear < 0) rear = 0;
    if (vl < 0) vl = 0;
    if (vr < 0) vr = 0;
    const uint16_t r = (uint16_t)(rear > 0xFFFF ? 0xFFFF : rear);
    const uint16_t l16 = (uint16_t)(vl > 0xFFFF ? 0xFFFF : vl);
    const uint16_t r16 = (uint16_t)(vr > 0xFFFF ? 0xFFFF : vr);

    data[0] = (uint8_t)(r & 0xFF);   // [KM ESP_HL_Radgeschw_02 low, 0.0075 km/h] [REA diag G15: sig118, read]
    data[1] = (uint8_t)(r >> 8);     // [KM ESP_HL_Radgeschw_02 high]
    data[2] = (uint8_t)(r & 0xFF);   // [KM ESP_HR_Radgeschw_02 low] [REA diag G15: sig119, read]
    data[3] = (uint8_t)(r >> 8);     // [KM ESP_HR_Radgeschw_02 high]
    data[4] = (uint8_t)(l16 & 0xFF); // [KM ESP_VL_Radgeschw_02 low] [REA diag G15: sig116, read]
    data[5] = (uint8_t)(l16 >> 8);   // [KM ESP_VL_Radgeschw_02 high]
    data[6] = (uint8_t)(r16 & 0xFF); // [KM ESP_VR_Radgeschw_02 low] [REA diag G15: sig117, read]
    data[7] = (uint8_t)(r16 >> 8);   // [KM ESP_VR_Radgeschw_02 high]
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
  return dangerZoneEnabled && !haldexLearnActive && lockTarget > 99.0f;
}

void fillMotor11Bpk(uint8_t data[8], uint8_t counter)
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
  // ESP request" and falls back to its own Motor_11 strategy. With the V11 lock request
  // (Min = Max = 0..250) the 0CQ follows ESP_14 and this torque is not used for
  // the target - it only feeds the unit's clutch-load model.
  uint16_t ceilNm = bpkCeilingNm;
  if (dangerZoneActive() && dangerZoneNm > ceilNm)
    ceilNm = dangerZoneNm;
  const uint16_t floorNm = (bpkFloorNm < ceilNm) ? bpkFloorNm : 0;

  uint16_t torqueNm = getLockTargetAdjustedValue(0xFE, false);
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

  data[0] = 0x00; // caller fills it [KM Motor_11.Motor_11_CRC] [REA audit_gen5_0cq A14/S6: b0 never read, no CRC check]
  data[1] = (counter & 0x0F) | ((rawSollRoh & 0x000F) << 4); // [KM Motor_11_BZ b1[0-3], MO_Mom_Soll_Roh low b1[4-7] (raw-509 Nm)] [REA audit A14: BZ not read; Soll_Roh sig152 bits 12-21 -> 0xCEE6]
  data[2] = ((rawSollRoh >> 4) & 0x3F) | ((rawIst & 0x0003) << 6); // [KM MO_Mom_Soll_Roh high b2[0-5], MO_Mom_Ist_Summe low b2[6-7]] [REA audit A14 (VAQ: positions hold, audit §4)]
  data[3] = (rawIst >> 2) & 0xFF; // [KM MO_Mom_Ist_Summe high (raw-509 Nm)] [REA audit A14/A20: Ist sig153 bits 22-31, x4 x MO_Faktor -> 0xCEE4, raw >= 0x3FB invalid; strategy only, not the exact target]
  data[4] = rawTraeg & 0xFF; // [KM MO_Mom_Traegheit_Summe low] [REA audit A21: not read]
  data[5] = ((rawTraeg >> 8) & 0x03) | ((rawSolf & 0x3F) << 2); // [KM MO_Mom_Traegheit_Summe high b5[0-1], MO_Mom_Soll_gefiltert low b5[2-7]] [REA audit A21: not read (Solf is never seen by the 0CQ)]
  data[6] = ((rawSolf >> 6) & 0x0F) | ((rawSchub & 0x0F) << 4); // [KM MO_Mom_Soll_gefiltert high b6[0-3], MO_Mom_Schub low b6[4-7]] [REA audit A27: Schub getter never used; VAQ reads one b6-b7 bit (FUN_c25856)]
  data[7] = ((rawSchub >> 4) & 0x1F) | bpkStatusFl; // [KM MO_Mom_Schub high b7[0-4], MO_Status_Normalbetrieb_01 b7.5, MO_erste_Ungenauschwelle b7.6, MO_QBit_Motormomente b7.7] [REA audit A27: only b7.5 (bit 61), monitor-enable only]

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

// Gen5 0CQ (V11.00.0): the ESP_14 lock request value for bytes 6 and 7
// (BR_Vorg_Allrad_Min / BR_Vorg_Allrad_Max). OEM code (spec_gen5_0cq §2.2):
// sending Min = Max = V puts the 0CQ in its "exact" mode and the clutch target
// becomes V x 128, which is the same 0-250 scale (0.4 % per step) the unit
// reports back in Allrad_03 byte 2. 0 = clutch open, 250 (0xFA) = 100 %.
// [KM ESP_14.BR_Vorg_Allrad_Min/Max 0.4 %] [REA audit_gen5_0cq A17/A18/A25/S13]
// Above 250 nothing is a usable request: 251 and 253-255 (with b6 = b7) make the
// 0CQ ignore ESP_14 and run its own Motor_11 strategy, which is why the old
// 0..0xFE scale lost control at full lock (spec D1); 252 (0xFC) in Max OPENS the
// clutch - it is NOT "hold" (audit S13). So this never returns more than 250.
// "Exact" is V x 128 limited by the unit's falling-ramp and run-time capacity
// cap (audit A25). Linear in lockRequestPercent() - unlike the VAQ path, ESP_19
// speeds and BPK torque, which use getLockTargetAdjustedValue().
uint8_t gen5Esp14LockRequestByte()
{
  uint16_t lockRequestValue = (uint16_t)lockRequestPercent() * GEN50_REQUEST_FULL; // 0..25000
  lockRequestValue = (lockRequestValue + 50) / 100;                        // 0..250, rounded to the nearest step
  if (lockRequestValue > GEN50_REQUEST_FULL)
  {
    lockRequestValue = GEN50_REQUEST_FULL; // top of the valid range (0xFA)
  }
  return (uint8_t)lockRequestValue;
}

void startHaldexLearn()
{
  if (haldexLearnActive || longLearnActive)
  {
    return; // already running (Long Learn drives its own sweeps)
  }

  // The table is cleared by runLearnSweep (which keeps a copy to put back on cancel)
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

  // V11.00.8: keep the table we had, to put back if this sweep is cancelled - a
  // half-built table marked valid used to cap the car at part lock (the
  // unfinished entries stay 0). It isn't used while learning (the lock request follows
  // haldexLearnCF), and writeEEP doesn't save it until the run is over.
  static uint8_t tableBefore[101]; // static: off the task stack (one sweep at a time)
  memcpy(tableBefore, haldexLearnTable, sizeof(tableBefore));
  const bool tableBeforeValid = haldexLearnTableValid;
  memset(haldexLearnTable, 0, sizeof(haldexLearnTable));
  haldexLearnTableValid = false;
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
      if (receivedHaldexEngagement <= 2)
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
    longLearnStopIfMoving();

    // Average the settled window rather than taking one instantaneous read.
    uint32_t sum = 0;
    uint16_t n = 0;
    for (uint32_t o = 0; o < observeMs && !haldexLearnCancel; o += sampleMs)
    {
      vTaskDelay(sampleMs / portTICK_PERIOD_MS);
      sum += receivedHaldexEngagement;
      n++;
    }
    uint8_t eng = n ? (uint8_t)(sum / n) : receivedHaldexEngagement;

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
  else
  {
    memcpy(haldexLearnTable, tableBefore, sizeof(tableBefore)); // cancelled: the table we had
    haldexLearnTableValid = tableBeforeValid;
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

bool startLongLearn(bool testAll, const char **why)
{
  const char *reason = nullptr;
  const int gi = frameEditGenIdx(haldexGeneration);
  if (longLearnActive || haldexLearnActive)
  {
    reason = "A learn is already running";
  }
  else if (gi < 0)
  {
    reason = "Frame editing is not available for this generation"; // gen41/42 etc. have no blocks
  }
  else if (hasCANChassis && receivedVehicleSpeed > 0)
  {
    reason = "Stop the car first - during Long Learn the Haldex doesn't see the car's brakes / ABS";
  }
  if (reason)
  {
    if (why)
      *why = reason;
    return false;
  }

  longLearnGenIdx = (uint8_t)gi;
  longLearnGeneration = haldexGeneration;
  longLearnTestAll = testAll;
  longLearnWasStandalone = isStandalone;
  longLearnCancel = false;
  longLearnPhase = LL_SWEEP;
  longLearnSweepIdx = 0;
  longLearnSweepTotal = 0;
  longLearnSweepCount = 0;
  longLearnCurrentBit = -1;
  longLearnBaselineValid = false;
  longLearnFinalValid = false;
  longLearnFixHuntingSet = false;
  longLearnBpkStart = bpkCeilingNm;
  longLearnBpkAdjusted = false;
  longLearnMoved = false;
  longLearnFailReason = LLF_NONE;
  memset(longLearnRefPts, 0, sizeof(longLearnRefPts));
  longLearnInteraction = false;
  memset(longLearnCarSent, 0, sizeof(longLearnCarSent));
  memset(longLearnCarDlc, 0, sizeof(longLearnCarDlc));
  memset(longLearnCarBytes, 0, sizeof(longLearnCarBytes));
  memset(longLearnOurBytes, 0, sizeof(longLearnOurBytes));
  memset(longLearnTestState, 0, sizeof(longLearnTestState));
  longLearnUnlistedResult = LLU_NOT_RUN;
  longLearnUnlistedSeen = 0;
  longLearnUnlistedOverflowSeen = false;
  longLearnUnlistedHitCount = 0;
  longLearnStartMs = millis();
  longLearnEndMs = 0;
  memset(longLearnBlockResult, 0, sizeof(longLearnBlockResult));
  longLearnActive = true;

  xTaskCreate(longLearnTask, "longLearn", 6144, nullptr, 1, nullptr);
  return true;
}

// Long Learn feeds the Haldex our frames only (no real brakes / ABS), so it
// must never run with the car moving. Called from every wait in the run.
void longLearnStopIfMoving()
{
  if (longLearnActive && hasCANChassis && receivedVehicleSpeed > LL_MOVING_KMH)
  {
    longLearnMoved = true;
    longLearnCancel = true;
    haldexLearnCancel = true; // stops a sweep in progress
  }
}

// ===== Gen2 (7H0907554 / 1K0907554B) demand bytes (V11.00.0) =====

// Value for one Gen2 "demand" byte (Motor_1 b1 / b5, Bremse_3 front-wheel low bytes).
//   Lock request on the bus: fullValue whenever any lock is wanted, 0 when none is
//                     (0x7F on the lock request opens the clutch anyway).
//   No lock request:         the pre-V11 behaviour - fullValue scaled by the lock target -
//                     so an old saved mask without Bremse_4 never jumps to full lock.
uint8_t gen2DemandValue(uint8_t fullValue)
{
  if (gen2LockRequestOnBus())
  {
    if (lockRequestPercent() > 0)
    {
      return fullValue; // ask for everything, the Bremse_4 ceiling sets the level
    }
    return 0x00; // no lock wanted
  }
  return getLockTargetAdjustedValue(fullValue, false); // no lock request: demand follows the lock target (pre-V11)
}

// Gen2: Bremse_4 byte 0 for the lock asked for right now (V11.00.13, audit_gen2_7h0).
// OEM code: ceiling = (b0 - 0x80) x 28 request units; above a 38.8 bar target the unit drives its
// valve straight to maximum - from b0 = 0xD6 on a 7H0 and from 0xEA on a 1K0 (final-request scale
// 0x440 vs 0x540). We don't identify the controller, so 1-99 % is spread over 0x81..0xE9 (exact on a
// 1K0; a 7H0 is already at full lock from ~82 %). 0x7F = 0 % (valid zero = clutch open), 0xFD = 100 %.
// The engagement read-back (0x2C0 byte 1, can.cpp) uses the same 0x80..0xEA = 0..100 % range.
// [KM Bremse_4.ABS_Vorgabewert_mitte_Kupplungs (3,-381) Nm/min - DBC scale not what the unit does]
// [REA audit_gen2_7h0 #48/#49/#50/#51/#55: 0x7F -> 0 at once; 0x80 = ramp to open (-40/cycle);
//  0x81..0xFD -> (b0-0x80) x 28; 0xFE/0xFF/<0x7F -> default 8000 = no limit]
uint8_t gen2LockRequestByte()
{
  const uint8_t lockPercent = lockRequestPercent();
  if (lockPercent == 0)
  {
    return 0x7F; // valid zero request: clutch open
  }
  if (lockPercent >= 100)
  {
    return GEN2_REQUEST_FULL_BYTE; // top request: full valve on both 7H0 and 1K0
  }
  const uint16_t step = ((uint16_t)lockPercent * GEN2_REQUEST_FULL + 50) / 100; // 1 % -> 1 .. 99 % -> 105
  return (uint8_t)(0x80 + step); // 0x81 .. 0xE9
}

// ===== Gen1 (02D900554B) lock request (V11.00.0) =====

// Gen1: Bremse_4 byte 0 for the lock asked for right now. The general
// vagLockRequestByte() spreads 1-99 % over 0x81..0xFC, but on the 02D everything
// from 0xCA up is already full lock, so 60-99 % would all come out as 100 %.
// This uses the 02D's own scale instead: 1-99 % -> 0x81..0xC9.
uint8_t gen1LockRequestByte()
{
  uint8_t lockPercent = lockRequestPercent(); // learn sweep / learn table / lock gates already applied

  if (lockPercent == 0)
  {
    return 0x7F; // ceiling 0: clutch open
  }
  if (lockPercent >= 100)
  {
    return GEN1_REQUEST_FULL_BYTE; // no ceiling: the unit's own maximum
  }

  uint32_t wantedCeiling = (uint32_t)lockPercent * GEN1_REQUEST_FULL / 100;           // ceiling in the unit's own units
  uint32_t ceilingSteps = (wantedCeiling + (GEN1_REQUEST_STEP / 2)) / GEN1_REQUEST_STEP; // rounded to whole byte steps
  if (ceilingSteps < 1)
  {
    ceilingSteps = 1; // 1 % must still be a (small) request, not "open"
  }
  return (uint8_t)(0x80 + ceilingSteps); // 50 % -> 0xA5, 99 % -> 0xC9 (matches the OEM table)
}

// Gen1: is Bremse_4 byte 0 carrying our lock request right now? Whenever its
// frame-edit block is on: we send the whole 0x2A0 ourselves in both modes
// (V11.00.7 - before, normal mode could only edit a 0x2A0 the car sent, and
// many Gen1 cars don't send one). Block off: Motor_1 is the lock request (old method).
bool gen1CeilingRequestActive()
{
  return frameEditEnabled(FE_GEN_1, GEN1_BREMSE4_BLOCK_BIT);
}

// Gen1: value for a "demand" byte (Motor_1 torque / pedal, the Bremse_3 slip).
// With the Bremse_4 ceiling as the lock request, the demand is held at maximum and the
// ceiling does the partial lock (OEM code: spec_gen1_02d §2.3); 0 % drops the
// demand too. Without it (old method), the demand byte itself is scaled.
// [KM Motor_1.inneres_Motor_Moment / Fahrpedalwert_oder_Drosselklapp, Bremse_3.Radgeschw__VL/VR low bytes]
// [REA audit_gen1_02d #36/#68 + §5: request ~15 900 at max demand > 0x3700, so the ceiling sets E258; F5/F6: b5 + fake slip not needed]
uint8_t gen1DemandValue(uint8_t fullValue)
{
  if (!gen1CeilingRequestActive())
  {
    return getLockTargetAdjustedValue(fullValue, false); // old method: demand scaled by the lock
  }
  if (lockRequestPercent() == 0)
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
// [KM Motor_1.mechanisches_Motor_Verlustmomen 0.39 % MDI] [REA audit_gen1_02d #18/#25: E1A0 = E108 x b6/255, E1BA = b1 - b6 torque; 0xFF -> 0x780]
uint8_t gen1LossTorqueByte(uint8_t oemValue)
{
  if (gen1CeilingRequestActive())
  {
    if (lockRequestPercent() == 0)
    {
      return 0xFE; // no lock: no drive torque
    }
    return 0x00; // maximum demand
  }

  switch (state.mode)
  {
  case MODE_FWD:
    return getLockTargetAdjustedValue(0xFE, true); // 0xFE = no drive torque
  case MODE_5050:
    return 0x00; // was 0x16 (took ~22 Nm off); 0x00 is the real maximum (OEM code: spec_gen1_02d D11)
  case MODE_6040:
    return getLockTargetAdjustedValue(0x22, false); // bench-tuned constant
  case MODE_7525:
    return getLockTargetAdjustedValue(0x50, false); // bench-tuned constant
  default:
    return oemValue; // Stock / Expert: unchanged
  }
}

// ===== Our frames vs the car's (V11.00.7) =====
// One rule for every frame-edit generation, in both modes: a block ON = the
// Haldex gets OUR whole frame for that ID (built by the standalone builders,
// exactly what the bench sends) and the car's copy is held back; OFF = the
// car's own frame goes through untouched. Before V11.00.7 normal mode instead
// edited a few bytes of the car's frame, and some blocks (Gen4 Motor_2 /
// Getriebe_1 ...) had no edit at all, so "on" and "off" were the same there and
// the car's brake / handbrake / gear bits always reached the unit - which is why
// Long Learn could reach 100 % on the bench and not in the car.

// Effective mode is Stock: base mode Stock with no force trigger, or a trigger
// whose configured value is Stock. Stock = untouched passthrough, always.
// V11.00.12: Expert counts as Stock while the live point sits in a PT area of the
// map (forced Expert from TC / hazards / button / drive mode included). The PT
// state is worked out here, not in updateLockTarget(), because the frame task
// stops calling that while the car's frames are going through.
bool stockEffective()
{
  const int inUse = modeInUse();
  if (inUse == MODE_STOCK)
  {
    return true;
  }
  return expertPtUpdate(inUse == MODE_EXPERT);
}

// Are our frames going to the Haldex right now?
bool ownFramesActive()
{
  if (analyzerMode)
  {
    return false; // passive bridge
  }
  if (isStandalone)
  {
    return true; // standalone: our frames are the bus
  }
  if (analyzerSerial)
  {
    return false; // serial analyzer in normal mode forwards every car frame untouched (can.cpp)
  }
  if (frameEditGenIdx(haldexGeneration) < 0)
  {
    return false; // no frame-edit blocks (Gen4.1 / Ford ...): normal mode passes the car's frames
  }
  if (disableController)
  {
    return false; // controller disabled = hard passthrough
  }
  if (!hasCANChassis)
  {
    return false; // the car's bus is quiet (parked): send nothing, so the Haldex can sleep
  }
  if (haldexLearnActive || longLearnActive)
  {
    return true; // Learn and Long Learn run whatever the mode
  }
  return !stockEffective();
}

// lockTarget from the mode, force triggers, lock gates and learn table, with
// the optional release rate limit: rising is instant, falling is limited to
// lockReleaseRatePerSec %/s when lockReleaseEnabled (dt-based, so any call rate
// works). Called by the 100 ms standalone bucket (both modes) and, on the
// generations without frame-edit blocks, for every car frame.
void updateLockTarget()
{
  static float smoothedLockTarget = 0.0f;
  static uint32_t lastLockMs = 0;

  const float rawTarget = getLockTargetAdjustment();
  const uint32_t nowMs = millis();

  if (!lockReleaseEnabled)
  {
    smoothedLockTarget = rawTarget; // instant: no rate limit
  }
  else
  {
    const float dtS = (lastLockMs == 0) ? 0.0f : (float)(nowMs - lastLockMs) / 1000.0f;
    if (rawTarget >= smoothedLockTarget)
    {
      smoothedLockTarget = rawTarget; // rise: immediate
    }
    else
    {
      const float maxDrop = lockReleaseRatePerSec * dtS;
      if (rawTarget > smoothedLockTarget - maxDrop)
      {
        smoothedLockTarget = rawTarget;
      }
      else
      {
        smoothedLockTarget = smoothedLockTarget - maxDrop;
      }
    }
  }
  lastLockMs = nowMs;
  lockTarget = smoothedLockTarget;
}
