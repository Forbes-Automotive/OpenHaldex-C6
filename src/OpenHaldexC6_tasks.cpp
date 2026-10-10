#include <OpenHaldexC6_tasks.h>
#include <OpenHaldexC6_IO.h>
#include <OpenHaldexC6_can.h>
#include <OpenHaldexC6_EEP.h>
#include <OpenHaldexC6_Analyzer.h>
#include <OpenHaldexC6_StandaloneCAN.h>
#include <OpenHaldexC6_Calculations.h>
#include <OpenHaldexC6_UDS.h>

void haldexLearnTask(void *arg)
{
  runLearnSweep(); // startHaldexLearn() has already primed the learn flags
  vTaskDelete(NULL);
}

// ---- Long Learn ------------------------------------------------------------
// Finds which STATIC frame blocks the Haldex actually needs. The lock-driven
// blocks (frameEditLockDriven - the ones we edit to have control) are always
// sent; everything else is a candidate:
//   1. Reference: every block on - as standalone sends the bus - one full
//      0-100 sweep on the unit's own settings.
//   2. BPK Adjust (Gen5 only, only when that sweep hunts - not smooth - or is
//      short of LL_BPK_ACCEPT): switch Motor_11 to BPK packing ("Fix
//      Hunting") and keep it only if a real sweep is better; if still short
//      on level, walk the torque ceiling up to the lowest value that reaches
//      100%. Whatever wins is KEPT when the run completes.
//   3. Prove it: the all-on sweep on the settled settings must be smooth (re-
//      swept if BPK moved the ceiling). Then two quick reads at each
//      LL_POINT_CF (20/40/70/100) with everything on; their spread is the
//      noise floor blocks are judged against. Not smooth, or spread >
//      LL_NOISE_MAX, stops the run - no trustworthy reference to compare with.
//   4. Testing Blocks: each candidate is turned off ON ITS OWN (everything else
//      on), read at the same points (release-to-0 before each), then turned
//      back on. A change beyond the noise floor at any point means the block
//      matters; otherwise "no effect". Points can't see a step between them -
//      that is left to the full confirmation sweep.
//   5. Confirmation: every "no effect" block off together, full sweep compared
//      CF-by-CF with the step-3 curve. If it still matches it becomes the
//      final set and stored table; if not (a step between the test points, or
//      blocks covering for each other) every block is left on, the reference
//      table is stored and the run is flagged as an interaction.
// In standalone "off" means the frame is not sent at all; in normal mode it
// means the car's own frame passes through untouched. Cancel / failure
// restores the mask, floor, torque ceiling, Fix Hunting and learn table.
static void longLearnLog(uint8_t kind, uint8_t bit, uint8_t verdict, const LearnScore &s,
                         uint8_t maxDev = 0, int8_t meanDelta = 0, const uint8_t *pts = nullptr)
{
  if (longLearnSweepCount < LL_MAX_SWEEPS)
  {
    LongLearnSweep &e = longLearnSweeps[longLearnSweepCount++];
    e.kind = kind;
    e.bit = bit;
    e.floorPct = esp14MinFloorPct;
    e.bpkNm = bpkCeilingNm;
    e.verdict = verdict;
    e.maxDev = maxDev;
    e.meanDelta = meanDelta;
    for (uint8_t k = 0; k < LL_NPTS; k++)
      e.pts[k] = pts ? pts[k] : 0;
    e.s = s;
  }
  longLearnSweepIdx = longLearnSweepCount;
}

static inline bool longLearnImproved(const LearnScore &trial, const LearnScore &base)
{
  return (!base.smooth && trial.smooth) || (trial.score > base.score + LL_TOLERANCE);
}

// Worst |t - ref| over CF 0-100, and the mean signed difference (direction).
static void compareCurves(const uint8_t *t, const uint8_t *ref, uint8_t &maxDev, int8_t &meanDelta)
{
  int worst = 0;
  int32_t sum = 0;
  for (uint8_t i = 0; i <= 100; i++)
  {
    const int d = (int)t[i] - (int)ref[i];
    sum += d;
    if (d > worst)
      worst = d;
    else if (-d > worst)
      worst = -d;
  }
  int32_t m = sum / 101;
  if (m > 127)
    m = 127;
  if (m < -127)
    m = -127;
  maxDev = (uint8_t)worst;
  meanDelta = (int8_t)m;
}

// Verdict for a read with block(s) off, against the proven (smooth) reference.
// smooth = the ramp stayed smooth (always true for a point read).
static uint8_t judgeAgainstReference(bool smooth, uint8_t maxDev, int8_t meanDelta)
{
  if (smooth && maxDev <= longLearnTol)
    return LLB_REMOVED; // no effect
  if (!smooth || meanDelta <= 0)
    return LLB_NEEDED;  // lower or lost smoothness without it
  return LLB_HARMFUL;   // moved up without it - still an effect, kept on
}

// One full 0-100 sweep, scored. False only on cancel - a sweep with no Haldex
// feedback is a valid (all-zero, score 0) result, e.g. a block the Haldex
// cannot work without.
static bool longLearnSweep(LearnScore &s)
{
  if (longLearnCancel)
    return false;
  runLearnSweep(4000); // pre-hold: let the clutch release from the previous sweep
  if (haldexLearnCancel || longLearnCancel)
    return false;
  scoreLearnTable(haldexLearnTable, s);
  return true;
}

// Quick single-point read: command CF directly (no ramp from 0) and hold long
// enough for the Haldex to settle. Used by the BPK ceiling walk, where only
// the level AT full lock matters. Judges the SUSTAINED value: only the last
// ~500 ms is scored, taking the MINIMUM seen there, so a fleeting spike to
// 100% over a reading that actually fluctuates at 80-85% does not pass.
static uint8_t quickHoldEngagement(uint8_t cf, uint32_t settleMs)
{
  haldexLearnActive = true;
  haldexLearnCancel = false;
  haldexLearnCF = cf;
  haldexLearnStep = cf;

  const uint32_t observeMs = (settleMs > 500) ? 500 : settleMs;
  uint8_t tailMin = 255;
  bool haveTail = false;
  for (uint32_t held = 0; held < settleMs && !longLearnCancel; held += 100)
  {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    const uint8_t eng = received_haldex_engagement;
    if (held >= settleMs - observeMs)
    {
      if (!haveTail || eng < tailMin)
        tailMin = eng;
      haveTail = true;
    }
  }
  return haveTail ? tailMin : 0;
}

// Release to 0 - waiting for the Haldex to actually let go, same detection
// runLearnSweep uses - and only THEN command cf and take a settled read. A
// steady hold at 100% with just the mask bit flipped proved unreliable on
// hardware (every block looked "needed": the controller doesn't cleanly
// re-evaluate a step change while already at max); it needs a real
// release-then-reapply edge.
static uint8_t cycleAndRead(uint8_t cf)
{
  haldexLearnActive = true;
  haldexLearnCancel = false;
  haldexLearnCF = 0;
  haldexLearnStep = 0;
  uint8_t releasedTicks = 0;
  for (uint32_t held = 0; held < 2000 && !longLearnCancel; held += 100)
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
  return quickHoldEngagement(cf, 1000);
}

// Read every LL_POINT_CF point (~1.5-3 s each). False only on cancel.
static bool quickPoints(uint8_t *out)
{
  for (uint8_t k = 0; k < LL_NPTS; k++)
  {
    out[k] = cycleAndRead(LL_POINT_CF[k]);
    if (longLearnCancel)
      return false;
  }
  return true;
}

// Sweep-log score for a point read: reach = the 100% point, engage = the
// first point that read non-zero.
static LearnScore pointScore(const uint8_t *p)
{
  LearnScore s = {};
  s.reach = p[LL_NPTS - 1];
  s.score = s.reach;
  s.engageCF = 101;
  for (uint8_t k = 0; k < LL_NPTS; k++)
  {
    if (p[k] > 0)
    {
      s.engageCF = LL_POINT_CF[k];
      s.engageJump = p[k];
      break;
    }
  }
  s.smooth = (s.reach >= LL_REACH_MIN);
  return s;
}

// Worst |a - ref| over the points, and their mean signed difference.
static void comparePoints(const uint8_t *a, const uint8_t *ref, uint8_t &maxDev, int8_t &meanDelta)
{
  int worst = 0, sum = 0;
  for (uint8_t k = 0; k < LL_NPTS; k++)
  {
    const int d = (int)a[k] - (int)ref[k];
    sum += d;
    if (d > worst)
      worst = d;
    else if (-d > worst)
      worst = -d;
  }
  maxDev = (uint8_t)worst;
  meanDelta = (int8_t)(sum / LL_NPTS);
}

void longLearnTask(void *arg)
{
  const uint8_t gi = longLearnGenIdx;
  uint64_t *mask = activeFrameEditMask(); // normal or standalone mask, whichever is live

  // Only Gen5 (0CQ/VAQ) packs Motor_11 with the Fix Hunting / BPK toggle.
  const bool isGen5 = (longLearnGeneration == 50 || longLearnGeneration == 52);

  // Everything needed to put the unit back exactly as it was on cancel/failure.
  longLearnMaskStart = mask[gi];
  longLearnFloorStart = esp14MinFloorPct;
  longLearnFloorResult = esp14MinFloorPct; // not tuned: runLearnSweep learns at floor 0 anyway
  longLearnBpkStart = bpkCeilingNm;
  const bool fixHuntingStart = fixHunting;
  uint8_t savedTable[101];
  memcpy(savedTable, haldexLearnTable, sizeof(savedTable));
  const bool savedTableValid = haldexLearnTableValid;

  // Reference curve - static to keep it off the task stack (one run at a time).
  static uint8_t refA[101];

  // All function-scope state is declared here, before the first goto.
  uint8_t outcome = LL_FAILED;
  LearnScore s = {}, sA = {};
  bool refAValid = false; // refA was swept on the settings the run has settled on
  bool anyRemoved = false;
  uint8_t dev = 0;
  int8_t md = 0;
  uint8_t pRef[2][LL_NPTS] = {}; // two all-on point reference reads
  uint8_t refPts[LL_NPTS] = {};  // their mean - what each block is read against

  // Start from the unit's OWN settings - do NOT force BPK packing here.
  // Forcing it made Long Learn's table disagree wildly with a normal learn:
  // which packing a unit needs is a per-unit trait, and on a 0CQ that doesn't
  // need BPK the same request reads about HALF under BPK (bench: 15.9% vs
  // 30.0%) with heavy jitter. Phase 2 may still turn BPK on, but only as
  // remediation when the first sweep hunts or can't reach target.

  // Block list for this generation + candidate marking.
  uint8_t bits[64];
  uint8_t nBits = 0;
  uint64_t allMask = 0;
  for (uint16_t i = 0; i < frameEditBlockCount && nBits < 64; i++)
  {
    if (frameEditBlocks[i].genIdx != gi)
      continue;
    const uint8_t b = frameEditBlocks[i].bit;
    bits[nBits++] = b;
    allMask |= (1ULL << b);
    const bool isCore = (frameEditLockDriven[gi] >> b) & 0x1ULL;
    longLearnBlockResult[b] = (isCore && !longLearnTestAll) ? LLB_CORE : LLB_UNTESTED;
  }
  uint8_t nCand = 0;
  for (uint8_t i = 0; i < nBits; i++)
    if (longLearnBlockResult[bits[i]] == LLB_UNTESTED)
      nCand++;

  longLearnSweepTotal = 1 + 1 + nCand + 1; // first + second reference + blocks + confirm (refined below)
  mask[gi] = allMask;                      // every block on, as standalone sends them

  // ---- Phase 1: first all-on sweep on the unit's own settings ------------
  longLearnPhase = LL_SWEEP;
  if (!longLearnSweep(sA))
  {
    outcome = LL_CANCELLED;
    goto restore;
  }
  longLearnLog(LLS_BASELINE, 0xFF, sA.smooth ? 1 : 0, sA);
  memcpy(refA, haldexLearnTable, sizeof(refA));
  refAValid = true;
  longLearnBaseline = sA;
  longLearnBaselineValid = true;
  if (sA.engageCF > 100 && !isGen5)
  {
    longLearnFailReason = LLF_NO_DATA; // nothing BPK could rescue on this generation
    goto restore;
  }

  // ---- Phase 2: BPK Adjust (Gen5 only) ------------------------------------
  // Remediation only, entered when the all-on sweep on the unit's OWN settings
  // hunts (not smooth) or is short of target. `smooth` already encodes reach
  // >= LL_REACH_MIN, engaged by LL_ENGAGE_MAX_CF and no step > LL_STEP_MAX.
  // The packing decision is judged with a REAL sweep - jumpiness is a
  // property of the ramp. The ceiling walk afterwards is a level question, so
  // quick holds are fine there; the proof sweeps below then judge the ramp at
  // whatever ceiling it settled on.
  if (isGen5 && (!sA.smooth || sA.reach < LL_BPK_ACCEPT))
  {
    longLearnPhase = LL_BPK;
    const uint16_t stepNm = 40;
    const uint16_t maxNm = 500;
    const uint16_t quickSettleMs = 1200;
    const bool fixHuntBefore = fixHunting;
    const uint16_t ceilBefore = bpkCeilingNm;

    LearnScore best = sA;
    uint8_t bestReach = sA.reach;
    uint16_t bestNm = bpkCeilingNm;
    bool bestFixHunt = fixHunting;

    // Step 1: if BPK is off, enable it and re-sweep. Keep it only if the sweep
    // is genuinely better (smooth when the first wasn't, or a better score).
    if (!fixHunting)
    {
      fixHunting = true;
      LearnScore trial = {};
      if (!longLearnSweep(trial))
      {
        outcome = LL_CANCELLED;
        goto restore;
      }
      longLearnLog(LLS_BPK, 0xFF, trial.smooth ? 1 : 0, trial);
      if (longLearnImproved(trial, best))
      {
        best = trial;
        bestReach = trial.reach;
        bestFixHunt = true;
        sA = trial; // this sweep is the reference on the new packing
        memcpy(refA, haldexLearnTable, sizeof(refA));
        longLearnBaseline = trial;
      }
      else
      {
        fixHunting = false; // did not help - back to how the unit was (refA still valid)
      }
    }

    // Step 2: only if still short on LEVEL, walk the ceiling to the lowest
    // value that reaches 100%. Skipped when already at target, so a unit that
    // just needed the packing change is not dragged up the ceiling range too.
    if (fixHunting && bestReach < LL_BPK_ACCEPT)
    {
      for (uint16_t nm = (uint16_t)(ceilBefore + stepNm); nm <= maxNm; nm += stepNm)
      {
        if (longLearnCancel)
        {
          outcome = LL_CANCELLED;
          goto restore;
        }
        bpkCeilingNm = nm;
        const uint8_t reach = quickHoldEngagement(100, quickSettleMs);
        LearnScore bs = {};
        bs.reach = reach;
        bs.score = reach;
        bs.engageCF = reach > 0 ? 0 : 101;
        bs.smooth = (reach >= LL_BPK_ACCEPT);
        longLearnLog(LLS_BPK, 0xFF, bs.smooth ? 1 : 0, bs);
        if (reach > bestReach)
        {
          bestReach = reach;
          bestNm = nm;
        }
        if (reach >= 100)
          break;
      }
    }

    // Keep the winning combination - which may be the unit's original
    // settings if none of the BPK variants beat them.
    fixHunting = bestFixHunt;
    bpkCeilingNm = bestNm;
    longLearnBpkAdjusted = (bestFixHunt != fixHuntBefore) || (bestNm != ceilBefore);
    if (bestNm != ceilBefore)
      refAValid = false; // ceiling moved since refA was swept - re-reference
    haldexLearnActive = false;
  }

  // ---- Phase 3: prove all-on gives a smooth 100%, then point reference -----
  longLearnPhase = LL_SWEEP;
  longLearnSweepTotal = longLearnSweepCount + (refAValid ? 0 : 1) + 2 + nCand + 1;
  if (!refAValid)
  {
    if (!longLearnSweep(sA))
    {
      outcome = LL_CANCELLED;
      goto restore;
    }
    longLearnLog(LLS_BASELINE, 0xFF, sA.smooth ? 1 : 0, sA);
    memcpy(refA, haldexLearnTable, sizeof(refA));
  }
  longLearnBaseline = sA;
  if (sA.engageCF > 100)
  {
    longLearnFailReason = LLF_NO_DATA;
    goto restore;
  }
  if (!sA.smooth)
  {
    longLearnFailReason = LLF_NOT_SMOOTH;
    goto restore;
  }

  // Two quick point reads with everything on - the reference the blocks are
  // read against, and their spread is the noise floor.
  for (uint8_t k = 0; k < 2; k++)
  {
    if (!quickPoints(pRef[k]))
    {
      outcome = LL_CANCELLED;
      goto restore;
    }
    longLearnLog(LLS_POINTS, 0xFF, 1, pointScore(pRef[k]), 0, 0, pRef[k]);
  }
  comparePoints(pRef[0], pRef[1], dev, md);
  longLearnNoise = dev;
  if (dev > LL_NOISE_MAX)
  {
    longLearnFailReason = LLF_NOISY;
    goto restore;
  }
  longLearnTol = (dev > LL_TOLERANCE) ? dev : LL_TOLERANCE;
  for (uint8_t k = 0; k < LL_NPTS; k++)
    refPts[k] = (uint8_t)(((uint16_t)pRef[0][k] + pRef[1][k] + 1) / 2);

  // ---- Phase 4: each candidate off on its own, point read, back on -------
  longLearnPhase = LL_BLOCKS;
  for (uint8_t i = 0; i < nBits; i++)
  {
    const uint8_t b = bits[i];
    if (longLearnBlockResult[b] != LLB_UNTESTED)
      continue;
    longLearnCurrentBit = b;
    mask[gi] &= ~(1ULL << b);
    uint8_t a[LL_NPTS] = {};
    const bool ok = quickPoints(a);
    mask[gi] |= (1ULL << b); // back on - every block is judged against all-on, not a shrinking set
    if (!ok)
    {
      outcome = LL_CANCELLED;
      goto restore;
    }
    comparePoints(a, refPts, dev, md);
    const uint8_t verdict = judgeAgainstReference(true, dev, md);
    longLearnBlockResult[b] = verdict;
    longLearnLog(LLS_BLOCK, b, verdict, pointScore(a), dev, md, a);
  }
  longLearnCurrentBit = -1;
  haldexLearnActive = false; // release the held point before the confirmation sweep

  // ---- Phase 5: confirmation with every "no effect" block off together ----
  longLearnPhase = LL_FINAL;
  for (uint8_t i = 0; i < nBits; i++)
  {
    if (longLearnBlockResult[bits[i]] == LLB_REMOVED)
    {
      mask[gi] &= ~(1ULL << bits[i]);
      anyRemoved = true;
    }
  }
  if (!longLearnSweep(s))
  {
    outcome = LL_CANCELLED;
    goto restore;
  }
  compareCurves(haldexLearnTable, refA, dev, md);
  longLearnLog(LLS_FINAL, 0xFF, s.smooth ? 1 : 0, s, dev, md);
  longLearnFinal = s;
  if (anyRemoved && judgeAgainstReference(s.smooth, dev, md) != LLB_REMOVED)
  {
    // Each block was harmless at the test points on its own but the reduced set is
    // not - a step between the test points, or two frames carrying the same
    // signal. Don't guess which: leave everything on and store the all-on
    // reference curve, which matches that set.
    longLearnInteraction = true;
    mask[gi] = allMask;
    memcpy(haldexLearnTable, refA, sizeof(refA));
    haldexLearnTableValid = true;
    haldexLearnStep = 101;
    longLearnFinal = longLearnBaseline;
  }
  longLearnFinalValid = true;

  // On success KEEP whatever packing / ceiling Phase 2 settled on - the stored
  // table was learned with it. (Phase 2 only turns BPK on when a real sweep
  // proved it better, so an unchanged unit ends up exactly as it started.)
  longLearnPhase = LL_DONE;
  longLearnEndMs = millis();
  longLearnActive = false;
  vTaskDelete(NULL);
  return;

restore:
  // Put back the mask, floor, torque ceiling, Fix Hunting toggle and learn
  // table from before the run so a cancelled/failed run never leaves a
  // half-tested configuration - or a changed torque model - behind.
  mask[gi] = longLearnMaskStart;
  esp14MinFloorPct = longLearnFloorStart;
  bpkCeilingNm = longLearnBpkStart;
  fixHunting = fixHuntingStart;
  memcpy(haldexLearnTable, savedTable, sizeof(savedTable));
  haldexLearnTableValid = savedTableValid;
  haldexLearnStep = savedTableValid ? 101 : 0;
  haldexLearnActive = false;
  longLearnCurrentBit = -1;
  longLearnPhase = outcome;
  longLearnEndMs = millis();
  longLearnActive = false;
  vTaskDelete(NULL);
}

void setupTasks()
{
  // max task priority = 24
  xTaskCreate(showHaldexState, "showHaldexState", 5000, NULL, 1, &handle_showHaldexState);
  xTaskCreate(writeEEP, "writeEEP", 2000, NULL, 3, NULL);
  xTaskCreate(updateTriggers, "updateTriggers", 3072, NULL, 4, &handle_updateTriggers); // was 2000: measured 432 B left (9.00.6 debugMemory)

  // Analyzer task stays idle unless analyzerMode is enabled.
  setupAnalyzer();

  // USB serial diagnostic harness (idle until a host talks to it).
  setupSerialLab();

  // Standalone frame generation: one scheduler task for every rate (was ten tasks of 2560 B each,
  // ~250-450 B used per debugMemory). Suspended whenever the car's own chassis bus is in use.
  xTaskCreate(standaloneFramesTask, "standaloneFrames", 3072, NULL, 10, &handle_standaloneFrames);
  if (!isStandalone)
  {
    vTaskSuspend(handle_standaloneFrames);
  }

  xTaskCreate(broadcastOpenHaldex, "broadcastOpenHaldex", 2048, NULL, 10, &handle_broadcastOpenHaldex); // was 1000: measured 644 B left // create a task for FreeRTOS for broadcasting the haldex state
  xTaskCreate(parseCAN_hdx, "parseHaldex", 2048, NULL, 11, NULL);                // create a task for FreeRTOS for incoming haldex CAN - in '_can.ino'
  xTaskCreate(parseCAN_chs, "parseChassis", 2048, NULL, 12, NULL);               // create a task for FreeRTOS for incoming chassis CAN - in '_can.ino'
  xTaskCreate(udsMQBTask, "udsMQBTask", 2048, NULL, 5, NULL);                    // UDS MQB diagnostic polling task (Gen 5 only)
  xTaskCreate(kwpTp20Task, "kwpTp20Task", 3072, NULL, 5, NULL);                  // KWP2000/TP2.0 diagnostic task (Gen2/4 PQ Haldex)
}

void showHaldexState(void *arg)
{
  while (1)
  {
    stackshowHaldexState = uxTaskGetStackHighWaterMark(NULL);

    DEBUG("Mode: %s", get_openhaldex_mode_string(state.mode));
    DEBUG("    Req:Act: %d:%d", int(lock_target), received_haldex_engagement); // this is the lock %

    if (detailedDebug)
    {
      DEBUG("    Raw haldexState: " BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(received_haldex_state));
      DEBUG("    reportClutch1: %d", received_report_clutch1); // this means it has a clutch issue
      DEBUG("    reportClutch2: %d", received_report_clutch2); // this means it also has a clutch issue
      DEBUG("    couplingOpen: %d", received_coupling_open);   // clutch fully disengaged
      DEBUG("    speedLimit: %d", received_speed_limit);       // hit a speed limit...
      DEBUG("    tempCounter2: %d", tempCounter2);    // incrememting value for checking the response to vars...

      DEBUG("    hasChassisCAN: %d", hasCANChassis);         // incrememting value for checking the response to vars...
      DEBUG("    hasHaldexCAN: %d", hasCANHaldex);           // incrememting value for checking the response to vars...
      DEBUG("    lastCANChassis: %ldms", lastCANChassisTick > 0 ? (long)(millis() - lastCANChassisTick) : -1);   // incrememting value for checking the response to vars...
      DEBUG("    lastHaldex: %ldms", lastCANHaldexTick > 0 ? (long)(millis() - lastCANHaldexTick) : -1); // incrememting value for checking the response to vars...

      DEBUG("    currentMode: %d", lastMode);       // incrememting value for checking the response to vars...
      DEBUG("    isStandalone: %d", isStandalone);  // incrememting value for checking the response to vars...
      DEBUG("    haldexGen: %d", haldexGeneration); // incrememting value for checking the response to vars...
      DEBUG("    Analyser Mode: %d", analyzerMode);
      DEBUG("    Force Mode Value TC/Haz/Ext: %d/%d/%d", tcForceModeValue, hazardForceModeValue, extBtnForceModeValue);

      DEBUG("Free heap: %d bytes", ESP.getFreeHeap());        // for debug - checking ESP available space
      DEBUG("Min free heap: %d bytes", ESP.getMinFreeHeap()); // for debug - checking ESP available space

      if (isBusFailure)
      {
        DEBUG("    Bus Failure: True");
      }
    }

    if (detailedDebugIO)
    {
      DEBUG("    Brake signal: %s", brakeSignalActive ? "true" : "false");
      DEBUG("    Handbrake signal: %s", handbrakeSignalActive ? "true" : "false");

      DEBUG("    Follow brake: %s", followBrake ? "true" : "false");
      DEBUG("    Invert handbrake: %s", invertHandbrake ? "true" : "false");
      DEBUG("    Invert brake: %s", invertBrake ? "true" : "false");
    }

    if (detailedDebugArray)
    {
      DEBUG("%d%d%d%d%d%d", throttleArray[0], throttleArray[1], throttleArray[2], throttleArray[3], throttleArray[4], throttleArray[5], throttleArray[6]);
      DEBUG("%d%d%d%d%d%d", speedArray[0], speedArray[1], speedArray[2], speedArray[3], speedArray[4], speedArray[5], speedArray[6]);
      // DEBUG("%d", speedArray[j]);
      // DEBUG("%d%d%d%d%d%d", lockArray[i][0], lockArray[i][1], lockArray[i][2], lockArray[i][3], lockArray[i][4], lockArray[i][5], lockArray[i][6]);
    }

    if (detailedDebugStack)
    {
      DEBUG("Stack Sizes:");

      DEBUG("    stackCHS: %d", stackCHS); // incrememting value for checking the response to vars...
      DEBUG("    stackHDX: %d", stackHDX); // incrememting value for checking the response to vars...

      DEBUG("    stackStandaloneFrames: %d", (int)uxTaskGetStackHighWaterMark(handle_standaloneFrames));

      DEBUG("    stackbroadcastOpenHaldex: %d", stackbroadcastOpenHaldex); // incrememting value for checking the response to vars...
      DEBUG("    stackupdateLabels: %d", stackupdateLabels);               // incrememting value for checking the response to vars...
      DEBUG("    stackshowHaldexState: %d", stackshowHaldexState);         // incrememting value for checking the response to vars...
      DEBUG("    stackwriteEEP: %d", stackwriteEEP);                       // incrememting value for checking the response to vars...
    }

    if (detailedDebugRuntimeStats)
    {
      char buffer2[2048] = {0};
      vTaskGetRunTimeStats(buffer2);
      Serial.println(buffer2);
    }

    if (detailedDebugCAN)
    {
      // Diagnostic reporting only. Fault detection/recovery and ownership of
      // isBusFailure live in canBusRecovery() - don't read/drain alerts here
      // (that would consume them before the recovery poll can see them).
      twai_status_info_t twaistatus0;
      twai_status_info_t twaistatus1;
      twai_get_status_info_v2(twai_bus_0, &twaistatus0);
      twai_get_status_info_v2(twai_bus_1, &twaistatus1);
      DEBUG("");
      DEBUG("CAN-BUS Details:");
      DEBUG("    Bus0 state: %d  RX buffered: %lu  RX missed: %lu  RX overrun: %lu",
            (int)twaistatus0.state, twaistatus0.msgs_to_rx,
            twaistatus0.rx_missed_count, twaistatus0.rx_overrun_count);
      DEBUG("    Bus1 state: %d  RX buffered: %lu  RX missed: %lu  RX overrun: %lu",
            (int)twaistatus1.state, twaistatus1.msgs_to_rx,
            twaistatus1.rx_missed_count, twaistatus1.rx_overrun_count);
      DEBUG("    Bus failure: %s", isBusFailure ? "true" : "false");
    }

    vTaskDelay(serialMonitorRefresh / portTICK_PERIOD_MS);
  }
}
