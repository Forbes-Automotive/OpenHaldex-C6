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
// Automates the manual "learn, look, add/remove a block, learn again" loop:
//   1. Initial Sweep: turn EVERY frame block for the generation on and run
//      ONE sweep at whatever Launch PWM Floor is currently configured - the
//      floor governs ramp shape (how fast the clutch takes up/releases
//      lock), not how far it can ultimately go, so it is NOT hunted through
//      candidate values here (that used to burn sweeps chasing a knob that
//      can't fix a "won't reach 100%" result). This just establishes the
//      baseline shape and whether 100% is already reachable as configured.
//   2. BPK Adjust (Gen5 only, only if step 1 didn't reach true 100%): the
//      torque model (Motor_11 "Fix Hunting"/BPK packing) is forced on for
//      the whole run so the user's configured torque ceiling (bpkCeilingNm)
//      actually gates the sweep instead of being silently bypassed by the
//      default V3 packing. Steps the ceiling up with quick single-point
//      checks at CF=100 (the ceiling doesn't affect ramp shape, only the
//      top-end value, so there's no need for a full 0-100 sweep per step)
//      until 100% is reached or the ceiling hits its safe maximum.
//   3. Sweeping Blocks: for each candidate, a quick release-to-0-then-
//      back-to-100% cycle (not a full 101-step sweep, but a real cycle -
//      holding steady at 100% and just flipping the mask bit was found on
//      real hardware to make every block look "needed", since the
//      controller doesn't cleanly re-evaluate a step change while already
//      at max). A block whose removal changes the settled result (worse OR
//      better) goes back on - NEEDED, or flagged HARMFUL when it was better
//      without; one that makes no difference stays off.
//   4. Confirmation sweep on the final set so the stored learn table (used
//      operationally to interpolate every lock_target, not just 100%)
//      matches the blocks that are actually enabled.
// Candidates are the blocks outside frameEditMaskDefaults (the historically
// edited V7 set is kept as-is) unless Test All was requested. Cancel / failure
// restores the mask, floor, torque ceiling, Fix Hunting toggle and learn
// table that were in place before the run. Fix Hunting itself is always
// reverted at the end (success or not) - only the calibrated ceiling value
// is kept, so a car needs Fix Hunting turned on by hand afterward to
// actually use it.
static void longLearnLog(uint8_t kind, uint8_t bit, uint8_t verdict, const LearnScore &s)
{
  if (longLearnSweepCount < LL_MAX_SWEEPS)
  {
    LongLearnSweep &e = longLearnSweeps[longLearnSweepCount++];
    e.kind = kind;
    e.bit = bit;
    e.floorPct = esp14MinFloorPct;
    e.bpkNm = bpkCeilingNm;
    e.verdict = verdict;
    e.s = s;
  }
  longLearnSweepIdx = longLearnSweepCount;
}

static inline bool longLearnDegraded(const LearnScore &trial, const LearnScore &base)
{
  return (base.smooth && !trial.smooth) || (trial.score + LL_TOLERANCE < base.score);
}
static inline bool longLearnImproved(const LearnScore &trial, const LearnScore &base)
{
  return (!base.smooth && trial.smooth) || (trial.score > base.score + LL_TOLERANCE);
}

// Quick single-point read: command CF directly (no ramp from 0) and hold long
// enough for the Haldex to settle, returning the highest engagement seen
// (same peak-hold guard as runLearnSweep, against a transient bad reading).
// Used wherever only the value AT one CF matters - BPK ceiling search, block
// on/off checks - which is much faster than a full 0-100 sweep. A full sweep
// is only needed where the SHAPE of the ramp itself is being judged (floor
// tuning, the final confirmation table).
static uint8_t quickHoldEngagement(uint8_t cf, uint32_t settleMs)
{
  haldexLearnActive = true;
  haldexLearnCancel = false;
  haldexLearnCF = cf;
  haldexLearnStep = cf;

  // Judge the SUSTAINED value, not a transient spike: sample the whole
  // window but only score the last ~500 ms (the "observe" tail, after the
  // initial ramp/settle), taking the MINIMUM seen there. A plain running
  // peak over the whole window was found (on real hardware) to wrongly pass
  // a ceiling that only produced a fleeting spike to 100% while the actual
  // sustained reading fluctuated well below it (e.g. 80-85%) - a held
  // target that fluctuates is not the same as one that's actually reached.
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

// Two-point test for the block phase: release to 0 (waiting for the Haldex
// to actually let go, same detection runLearnSweep uses) and only THEN
// command straight to 100% and hold for a settled read. A steady hold at
// 100% with just the mask bit flipped turned out to be unreliable in
// practice - every block came back "needed" because the controller doesn't
// cleanly re-evaluate a step change while already sitting at max; it needs
// to see a genuine release-then-reapply edge to give a trustworthy reading.
// Still far faster than a full 0-100 sweep (two commanded points instead of
// 101), just not a frozen hold.
// As cycleAndReadEngagement(), but commands an arbitrary CF instead of full
// lock - used to check candidate blocks at part lock as well as at 100%.
static uint8_t cycleAndReadEngagementAt(uint8_t cf, uint32_t releaseMs, uint32_t settleMs)
{
  haldexLearnActive = true;
  haldexLearnCancel = false;
  haldexLearnCF = 0;
  haldexLearnStep = 0;
  uint8_t releasedTicks = 0;
  for (uint32_t held = 0; held < releaseMs && !longLearnCancel; held += 100)
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
  return quickHoldEngagement(cf, settleMs);
}

static uint8_t cycleAndReadEngagement(uint32_t releaseMs, uint32_t settleMs)
{
  haldexLearnActive = true;
  haldexLearnCancel = false;
  haldexLearnCF = 0;
  haldexLearnStep = 0;
  uint8_t releasedTicks = 0;
  for (uint32_t held = 0; held < releaseMs && !longLearnCancel; held += 100)
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
  return quickHoldEngagement(100, settleMs);
}

void longLearnTask(void *arg)
{
  const uint8_t gi = longLearnGenIdx;
  uint64_t *mask = activeFrameEditMask(); // normal or standalone mask, whichever is live
  const uint32_t preHoldMs = 4000;        // let the clutch release between full sweeps

  // Only Gen5 (0CQ/VAQ) packs Motor_11 with the Fix Hunting / BPK toggle.
  const bool isGen5 = (longLearnGeneration == 50 || longLearnGeneration == 52);

  // Everything needed to put the unit back exactly as it was on cancel/failure.
  longLearnMaskStart = mask[gi];
  longLearnFloorStart = esp14MinFloorPct;
  longLearnBpkStart = bpkCeilingNm;
  const bool fixHuntingStart = fixHunting;
  uint8_t savedTable[101];
  memcpy(savedTable, haldexLearnTable, sizeof(savedTable));
  const bool savedTableValid = haldexLearnTableValid;

  // Start from the unit's OWN settings - do NOT force BPK packing here.
  // Forcing it made Long Learn's table disagree wildly with a normal learn:
  // which packing a unit needs is a per-unit trait, and on a 0CQ that doesn't
  // need BPK the same request reads about HALF under BPK (bench: 15.9% vs
  // 30.0%) with heavy jitter. Phase 2 below may still turn BPK on, but only
  // as remediation when the baseline can't reach target - never by default.

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
    const bool isCore = (frameEditMaskDefaults[gi] >> b) & 0x1ULL;
    longLearnBlockResult[b] = (isCore && !longLearnTestAll) ? LLB_CORE : LLB_UNTESTED;
  }
  uint8_t nCand = 0;
  for (uint8_t i = 0; i < nBits; i++)
    if (longLearnBlockResult[bits[i]] == LLB_UNTESTED)
      nCand++;

  // The Launch PWM Floor governs the clutch's ramp rate, not how much lock
  // is ultimately reachable - hunting through floor candidates burns sweeps
  // without ever being able to fix a "can't reach 100%" result, since that's
  // gated by the torque ceiling instead (Phase 2, Gen5). So Phase 1 now just
  // runs once at whatever floor is currently configured.
  uint8_t floors[1];
  uint8_t nFloors = 0;
  floors[nFloors++] = esp14MinFloorPct;
  longLearnSweepTotal = nFloors + nCand + 1;

  uint8_t outcome = LL_FAILED;
  LearnScore s;

  mask[gi] = allMask; // phase 1 runs with every block on

  // ---- Phase 1: Initial Sweep (baseline / floor tuning) ------------------
  longLearnPhase = LL_SWEEP;
  LearnScore best = {};
  uint8_t bestFloor = floors[0];
  bool haveBest = false;
  for (uint8_t fi = 0; fi < nFloors; fi++)
  {
    if (longLearnCancel)
    {
      outcome = LL_CANCELLED;
      goto restore;
    }
    esp14MinFloorPct = floors[fi];
    if (!runLearnSweep(preHoldMs))
    {
      if (haldexLearnCancel)
      {
        outcome = LL_CANCELLED;
        goto restore;
      }
      // Zero Haldex feedback at this one floor candidate is not a run
      // failure - it just means this particular floor doesn't work. Score
      // the (all-zero) table normally (reach 0, lowest possible score) and
      // try the next floor candidate; if none of them reach 100%, Phase 2
      // (BPK Adjust) picks up from there on Gen5 and raises the torque
      // ceiling instead.
    }
    scoreLearnTable(haldexLearnTable, s);
    longLearnLog(fi == 0 ? LLS_BASELINE : LLS_FLOOR, 0xFF, s.smooth ? 1 : 0, s);
    if (!haveBest || s.score > best.score)
    {
      best = s;
      bestFloor = floors[fi];
      haveBest = true;
    }
    // Every floor already commands full lock (CF climbs to 100) - what
    // changes is whether that reaches true 100% engagement. Keep trying
    // floor candidates until one actually gets there; only "smooth" (close
    // but short of 100) is not good enough to stop early on. If nothing
    // reaches 100%, the best-scoring floor tried is used as the fallback -
    // Phase 2 (BPK Adjust) picks up from there if this is Gen5.
    if (s.reach >= 100)
      break;
  }
  esp14MinFloorPct = bestFloor;
  longLearnFloorResult = bestFloor;
  longLearnBaseline = best;
  longLearnBaselineValid = true;
  longLearnSweepTotal = longLearnSweepCount + nCand + 1; // exact from here on

  // ---- Phase 2: BPK Adjust (Gen5 only) ----------------------------------
  // Remediation only, entered when the baseline on the unit's OWN settings is
  // either jumpy or short of target. Jumpiness matters as much as level here:
  // Fix Hunting exists to cure hunting, so a sweep that reaches 95% but climbs
  // in steps is exactly the case it is for. `smooth` already encodes all three
  // criteria (reach >= LL_REACH_MIN, engaged by LL_ENGAGE_MAX_CF, no step >
  // LL_STEP_MAX), so use it as the trigger rather than level alone.
  //
  // Judge the packing decision with a REAL SWEEP - jumpiness is a property of
  // the ramp and cannot be seen from a single-point reading. The ceiling walk
  // afterwards is a level question, so quick holds are fine there.
  if (isGen5 && (!best.smooth || best.reach < LL_BPK_ACCEPT))
  {
    longLearnPhase = LL_BPK;
    const uint16_t stepNm = 40;
    const uint16_t maxNm = 500;
    const uint16_t quickSettleMs = 1200;
    const bool fixHuntBefore = fixHunting;
    const uint16_t ceilBefore = bpkCeilingNm;

    uint8_t bestReach = best.reach;   // what the unit's own settings managed
    uint16_t bestNm = bpkCeilingNm;
    bool bestFixHunt = fixHunting;

    // Step 1: if BPK is off, enable it and re-sweep. Keep it only if the sweep
    // is genuinely better (smooth when the baseline wasn't, or a better score).
    if (!fixHunting)
    {
      fixHunting = true;
      if (runLearnSweep(preHoldMs))
      {
        LearnScore trial;
        scoreLearnTable(haldexLearnTable, trial);
        longLearnLog(LLS_BPK, 0xFF, trial.smooth ? 1 : 0, trial);
        if (longLearnImproved(trial, best))
        {
          best = trial;
          bestReach = trial.reach;
          bestFixHunt = true;
        }
        else
        {
          fixHunting = false; // did not help - back to how the unit was
        }
      }
      else if (haldexLearnCancel)
      {
        fixHunting = fixHuntBefore;
        outcome = LL_CANCELLED;
        goto restore;
      }
      else
      {
        fixHunting = false; // no data under BPK - not the answer for this unit
      }
    }

    // Step 2: only if still short on LEVEL, walk the ceiling. Skipped entirely
    // when the sweep is already smooth and at target, so a unit that just
    // needed the packing change is not dragged up the ceiling range as well.
    if (fixHunting && bestReach < LL_BPK_ACCEPT)
    {
      for (uint16_t nm = (uint16_t)(ceilBefore + stepNm); nm <= maxNm; nm += stepNm)
      {
        if (longLearnCancel)
        {
          fixHunting = fixHuntBefore;
          bpkCeilingNm = ceilBefore;
          outcome = LL_CANCELLED;
          goto restore;
        }
        bpkCeilingNm = nm;
        const uint8_t reach = quickHoldEngagement(100, quickSettleMs);
        LearnScore bs = {};
        bs.reach = reach; bs.score = reach;
        bs.engageCF = reach > 0 ? 0 : 101;
        bs.smooth = (reach >= LL_BPK_ACCEPT);
        longLearnLog(LLS_BPK, 0xFF, bs.smooth ? 1 : 0, bs);
        if (reach > bestReach) { bestReach = reach; bestNm = nm; }
        if (reach >= 100)
          break;
      }
    }

    // Keep the winning combination - which may be the unit's original
    // settings if none of the BPK variants beat them.
    fixHunting = bestFixHunt;
    bpkCeilingNm = bestNm;
    longLearnBpkAdjusted = (bestFixHunt != fixHuntBefore) || (bestNm != ceilBefore);
    if (bestReach > longLearnBaseline.reach)
    {
      longLearnBaseline = best;
      longLearnBaseline.reach = bestReach;
    }
    longLearnSweepTotal = longLearnSweepCount + nCand + 1; // exact again
  }

  // ---- Phase 3: Sweeping Blocks (release/reapply check per block) --------
  // For each candidate: release to 0, command back up to 100%, and compare
  // the settled reading to baseline - a fresh two-point cycle rather than a
  // frozen hold, per the block-phase-only issue found in testing (every
  // block was coming back "needed" under a steady hold+flip). A block that
  // drives the result to (near) zero is simply the strongest possible "this
  // block matters" reading, not a run failure. Still far faster than a full
  // sweep per block.
  longLearnPhase = LL_BLOCKS;
  mask[gi] = allMask; // clean baseline with every block back on
  {
    const uint32_t releaseMs = 2000, settleMs = 1000;
    // Confirm good at BOTH ends of the range before touching any block. Testing
    // only at full lock was leaving Long Learn with a good top end and a wrecked
    // middle: a block can be irrelevant at 100% yet matter at part lock, get
    // declared "not needed", and then the confirmation sweep runs without it.
    const uint8_t baseline100 = cycleAndReadEngagement(releaseMs, settleMs);
    const uint8_t baselineMid = cycleAndReadEngagementAt(LL_MID_CF, releaseMs, settleMs);

    for (uint8_t i = 0; i < nBits; i++)
    {
      const uint8_t b = bits[i];
      if (longLearnBlockResult[b] != LLB_UNTESTED)
        continue;
      if (longLearnCancel)
      {
        outcome = LL_CANCELLED;
        goto restore;
      }
      longLearnCurrentBit = b;
      mask[gi] &= ~(1ULL << b); // remove it
      const uint8_t reach = cycleAndReadEngagement(releaseMs, settleMs);           // 0 -> 100% with it off
      const uint8_t reachMid = cycleAndReadEngagementAt(LL_MID_CF, releaseMs, settleMs); // and at part lock

      LearnScore bs = {};
      bs.reach = reach;
      bs.score = reach;
      bs.engageCF = reach > 0 ? 0 : 101;
      bs.smooth = (reach >= 100);

      // A block only stays off if it makes no measurable difference at BOTH
      // full and part lock. Judge the worst of the two deltas so a mid-range
      // regression can't be hidden by a clean 100% reading.
      const int d100 = (int)reach - (int)baseline100;
      const int dMid = (int)reachMid - (int)baselineMid;
      const int worstDelta = (d100 < dMid) ? d100 : dMid;
      const int bestDelta = (d100 > dMid) ? d100 : dMid;

      // Rule: if removing the block AFFECTS either operating point in either
      // direction it goes back on. Only a block that makes no measurable
      // difference anywhere stays off.
      uint8_t verdict;
      if (worstDelta + LL_TOLERANCE < 0)
      {
        verdict = LLB_NEEDED;
        mask[gi] |= (1ULL << b); // worse without it - put it back
        if (longLearnCancel)
        {
          outcome = LL_CANCELLED;
          goto restore;
        }
        cycleAndReadEngagement(releaseMs, settleMs); // 0 -> 100% again with it restored, clean state before the next candidate
      }
      else if (bestDelta > LL_TOLERANCE)
      {
        verdict = LLB_HARMFUL; // better without it - still kept on, flagged for the user
        mask[gi] |= (1ULL << b);
        if (longLearnCancel)
        {
          outcome = LL_CANCELLED;
          goto restore;
        }
        cycleAndReadEngagement(releaseMs, settleMs); // 0 -> 100% again with it restored, clean state before the next candidate
      }
      else
      {
        verdict = LLB_REMOVED; // no measurable difference - leave it off
      }
      longLearnBlockResult[b] = verdict;
      longLearnLog(LLS_BLOCK, b, verdict, bs);
    }
  }
  longLearnCurrentBit = -1;
  haldexLearnActive = false; // release the held 100% command before the final sweep

  // ---- Phase 4: confirmation sweep on the final set ----------------------
  longLearnPhase = LL_FINAL;
  if (longLearnCancel)
  {
    outcome = LL_CANCELLED;
    goto restore;
  }
  if (!runLearnSweep(preHoldMs))
  {
    outcome = haldexLearnCancel ? LL_CANCELLED : LL_FAILED;
    goto restore;
  }
  scoreLearnTable(haldexLearnTable, longLearnFinal);
  longLearnFinalValid = true;
  longLearnLog(LLS_FINAL, 0xFF, longLearnFinal.smooth ? 1 : 0, longLearnFinal);

  // On success KEEP whatever packing Phase 2 settled on - the stored table was
  // learned with it, and reverting here would leave the car driving on a
  // different torque model to the one the table describes. (Phase 2 only turns
  // BPK on when a real sweep proved it better, and puts it back otherwise, so
  // an unchanged unit still ends up exactly as it started.) Cancel/failure
  // still reverts everything at `restore:` below.
  longLearnPhase = LL_DONE;
  longLearnEndMs = millis();
  longLearnActive = false;
  vTaskDelete(NULL);
  return;

restore:
  // Put back the mask, floor, torque ceiling, Fix Hunting toggle and learn
  // table from before the run so a cancelled/failed run never leaves a
  // half-bisected configuration - or a changed torque model - behind.
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
  xTaskCreate(updateTriggers, "updateTriggers", 2000, NULL, 4, &handle_updateTriggers);

  // Analyzer task stays idle unless analyzerMode is enabled.
  setupAnalyzer();

  // USB serial diagnostic harness (idle until a host talks to it).
  setupSerialLab();

  // Create tasks for frame generation at various intervals. These will run in the background and can be suspended when not in use (e.g., when not in standalone mode).
  xTaskCreate(frames1000, "frames1000", 8000, NULL, 5, &handle_frames1000);
  xTaskCreate(frames200, "frames200", 8000, NULL, 6, &handle_frames200);
  xTaskCreate(frames100, "frames100", 8000, NULL, 7, &handle_frames100);
  xTaskCreate(frames25, "frames25", 8000, NULL, 8, &handle_frames25);
  xTaskCreate(frames20, "frames20", 8000, NULL, 9, &handle_frames20);
  xTaskCreate(frames10, "frames10", 8000, NULL, 10, &handle_frames10);
  xTaskCreate(frames13, "frames13", 4000, NULL, 10, &handle_frames13);
  xTaskCreate(frames50, "frames50", 4000, NULL, 8, &handle_frames50);
  xTaskCreate(frames250, "frames250", 4000, NULL, 5, &handle_frames250);
  xTaskCreate(gen41DualBusRatesTask, "gen41DualBusRates", 4000, NULL, 10, &handle_gen41_dual_bus_rates);

  if (!isStandalone)
  {
    vTaskSuspend(handle_frames1000);
    vTaskSuspend(handle_frames200);
    vTaskSuspend(handle_frames100);
    vTaskSuspend(handle_frames25);
    vTaskSuspend(handle_frames20);
    vTaskSuspend(handle_frames10);
    vTaskSuspend(handle_frames13);
    vTaskSuspend(handle_frames50);
    vTaskSuspend(handle_frames250);
    vTaskSuspend(handle_gen41_dual_bus_rates);
  }

  xTaskCreate(broadcastOpenHaldex, "broadcastOpenHaldex", 1000, NULL, 10, &handle_broadcastOpenHaldex); // create a task for FreeRTOS for broadcasting the haldex state
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

      DEBUG("    stackframes10: %d", stackframes10);     // incrememting value for checking the response to vars...
      DEBUG("    stackframes20: %d", stackframes20);     // incrememting value for checking the response to vars...
      DEBUG("    stackframes25: %d", stackframes25);     // incrememting value for checking the response to vars...
      DEBUG("    stackframes100: %d", stackframes100);   // incrememting value for checking the response to vars...
      DEBUG("    stackframes200: %d", stackframes200);   // incrememting value for checking the response to vars...
      DEBUG("    stackframes1000: %d", stackframes1000); // incrememting value for checking the response to vars...
      DEBUG("    stackframes13: %d", stackframes13);     // incrememting value for checking the response to vars...
      DEBUG("    stackframes50: %d", stackframes50);     // incrememting value for checking the response to vars...
      DEBUG("    stackframes250: %d", stackframes250);   // incrememting value for checking the response to vars...

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
