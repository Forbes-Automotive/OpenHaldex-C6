#include <OpenHaldexC6_tasks.h>
#include <OpenHaldexC6_IO.h>
#include <OpenHaldexC6_can.h>
#include <OpenHaldexC6_EEP.h>
#include <OpenHaldexC6_Analyzer.h>
#include <OpenHaldexC6_StandaloneCAN.h>
#include <OpenHaldexC6_Calculations.h>
#include <OpenHaldexC6_UDS.h>

// ---- Small helpers -------------------------------------------------------------

// The car right now, for the report
static void longLearnCarState(LLCarState &st)
{
  uint8_t f = 0;
  if (brakeFromCAN)
    f |= LLST_BRAKE_CAN;
  if (handbrakeFromCAN)
    f |= LLST_HANDBRAKE_CAN;
  if (absBrakingFromCAN)
    f |= LLST_ABS_CAN;
  if (brakeActive)
    f |= LLST_BRAKE_WIRE;
  if (handbrakeActive)
    f |= LLST_HANDBRAKE_WIRE;
  if (hasCANChassis)
    f |= LLST_CHASSIS_CAN;
  st.flags = f;
  st.speed = receivedVehicleSpeed;
  st.rpm = receivedVehicleRpm;
}

// One full 0-100 sweep, scored. False only on cancel - a sweep with no Haldex
// feedback is a valid (all-zero, score 0) result.
static bool longLearnSweep(LearnScore &s)
{
  if (longLearnCancel)
    return false;
  runLearnSweep(4000); // pre-hold: let the clutch release from the previous read
  if (haldexLearnCancel || longLearnCancel)
    return false;
  scoreLearnTable(haldexLearnTable, s);
  return true;
}

// Command cf directly and hold. Judges the SUSTAINED value: the minimum over
// the last ~500 ms, so a fleeting spike over a fluctuating reading doesn't pass.
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
    longLearnStopIfMoving();
    const uint8_t eng = receivedHaldexEngagement;
    if (held >= settleMs - observeMs)
    {
      if (!haveTail || eng < tailMin)
        tailMin = eng;
      haveTail = true;
    }
  }
  return haveTail ? tailMin : 0;
}

// Release to 0 - waiting for the Haldex to actually let go - and only THEN
// command cf and take a settled read. A steady hold at 100 % with just a frame
// swapped proved unreliable on hardware: the unit doesn't cleanly re-evaluate
// a change while already at max, it needs a release-then-reapply edge.
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
    longLearnStopIfMoving();
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

// Score for a point read: reach = the 100 % point, engage = the first point that read non-zero
static LearnScore pointScore(const uint8_t *p)
{
  LearnScore s = {};
  s.reach = p[LL_NPTS - 1];
  s.score = s.reach;
  s.engageCF = 101;
  for (uint8_t k = 0; k < LL_NPTS; k++)
  {
    if (p[k] > 0 && p[k] != 0xFF)
    {
      s.engageCF = LL_POINT_CF[k];
      s.engageJump = p[k];
      break;
    }
  }
  s.smooth = (s.reach >= LL_REACH_MIN);
  return s;
}

// Let only these unknown-ID slots through (slot UNLISTED_MAX = the overflow group)
static void longLearnPassUnlisted(const uint8_t *slots, uint8_t n)
{
  uint64_t set = 0;
  bool overflow = false;
  for (uint8_t i = 0; i < n; i++)
  {
    if (slots[i] >= UNLISTED_MAX)
      overflow = true;
    else
      set |= (1ULL << slots[i]);
  }
  unlistedPassSet = set;
  unlistedPassOverflow = overflow;
  unlistedPassMode = UNL_PASS_SET;
}

// ---- Setup ---------------------------------------------------------------------

void setupTasks()
{
  // max task priority = 24
#if enableDebug
  // Serial state logger: every line in it is a DEBUG(), so without enableDebug it
  // would only hold a 5000 B stack and sleep (V11.00.8: not created then).
  xTaskCreate(showHaldexState, "showHaldexState", 5000, NULL, 1, &handleShowHaldexState);
#endif
  xTaskCreate(writeEEP, "writeEEP", 2000, NULL, 3, NULL);
  xTaskCreate(updateTriggers, "updateTriggers", 3072, NULL, 4, &handleUpdateTriggers); // was 2000: measured 432 B left (9.00.6 debugMemory)

  // Analyzer task stays idle unless analyzerMode is enabled.
  setupAnalyzer();

  // USB serial diagnostic harness (idle until a host talks to it).
  setupSerialLab();

  // Our frames: one scheduler task for every rate (was ten tasks of 2560 B each,
  // ~250-450 B used per debugMemory). V11.00.7: runs in both modes - it only
  // sends while ownFramesActive() (standalone, or normal mode outside Stock).
  xTaskCreate(standaloneFramesTask, "standaloneFrames", 3072, NULL, 10, &handleStandaloneFrames);

  xTaskCreate(broadcastOpenHaldex, "broadcastOpenHaldex", 2048, NULL, 10, &handleBroadcastOpenHaldex); // was 1000: measured 644 B left // create a task for FreeRTOS for broadcasting the haldex state
  xTaskCreate(parseCANHdx, "parseHaldex", 2048, NULL, 11, NULL);                // create a task for FreeRTOS for incoming haldex CAN - in '_can.ino'
  xTaskCreate(parseCANChs, "parseChassis", 2048, NULL, 12, NULL);               // create a task for FreeRTOS for incoming chassis CAN - in '_can.ino'
  xTaskCreate(udsMQBTask, "udsMQBTask", 2048, NULL, 5, NULL);                    // UDS MQB diagnostic polling task (Gen 5 only)
  xTaskCreate(kwpTp20Task, "kwpTp20Task", 3072, NULL, 5, NULL);                  // KWP2000/TP2.0 diagnostic task (Gen2/4 PQ Haldex)
  xTaskCreate(dtcTask, "dtcTask", 4096, NULL, 4, NULL);                          // V11.00.5: fault-code read / clear (UDS, KWP/TP2.0, Volvo D2)
}

// ---- Long Learn (V11.00.7) ---------------------------------------------------
// Finds which frame the Haldex needs from US in this car - core or not - and
// whether the car sends something we don't know about that cuts the lock.
// "Ours" = our whole standalone frame for that ID (what the bench sends);
// "the car's" = the car's own frame passed through untouched, or nothing if the
// car doesn't send that ID. Same in both modes (on the bench there are simply
// no car frames).
//   1. ALL OURS: every block ours, every car ID held back - the bench bus. One
//      full 0-100 sweep must reach LL_REACH_MIN smoothly. If not, a retry: on
//      the 0CQ Fix Hunting (BPK packing) on, and if it is still short on level
//      the torque ceiling is walked up to the lowest value that reaches full
//      lock (units need different ceilings) - whatever works is kept. Other
//      generations: the same sweep once more. Still not -> stop and report: the
//      bench bus can't lock this unit, so it isn't a missing frame.
//   2. A quick all-ours read at CF 20/40/70/100 - the reference in the report.
//   3. Each block (non-core; core too with Test All) swapped to the car's frame
//      on its own, read at the same points, then back to ours. Full lock below
//      LL_REACH_MIN -> KEEP OURS. Otherwise the car's frame is fine (or, if the
//      car doesn't send it, ours isn't needed) -> block off. The car's bytes and
//      the car's state (brake / handbrake / ABS / speed) are logged per test:
//      a run started with the brake on or in gear reads differently.
//   4. Car IDs with no block: all let through together. Still full lock ->
//      nothing missing. Lower -> find which one: let half of them through, keep
//      the half that drops it, until one ID is left. It is held back from the
//      Haldex (heldBackIds) and the rest re-checked, up to HELD_BACK_MAX IDs.
//   5. Confirmation: kept blocks ours, the rest the car's, found IDs held back -
//      one full sweep. Full lock and smooth -> that set and its table are kept.
//      Otherwise every block is left ours with the step-1 table and the run is
//      flagged as an interaction (two frames covering for each other).
// The run stops (and restores) the moment the car moves (longLearnStopIfMoving).
// Cancel / failure puts back the mask, Fix Hunting, torque ceiling, held-back IDs and table.

// Log one read (sweep or point read). Always counts toward progress; stored
// while there is room in the log.
static void longLearnLog(uint8_t kind, uint8_t bit, uint8_t verdict, const LearnScore &s,
                         const uint8_t *pts = nullptr)
{
  if (longLearnSweepCount < LL_MAX_SWEEPS)
  {
    LongLearnSweep &e = longLearnSweeps[longLearnSweepCount++];
    e.kind = kind;
    e.bit = bit;
    e.verdict = verdict;
    e.fixHunting = fixHunting;
    e.bpkNm = bpkCeilingNm;
    for (uint8_t k = 0; k < LL_NPTS; k++)
    {
      e.pts[k] = pts ? pts[k] : 0;
    }
    e.s = s;
  }
  if (longLearnSweepIdx < 255)
  {
    longLearnSweepIdx = longLearnSweepIdx + 1;
  }
  if (longLearnSweepTotal <= longLearnSweepIdx)
  {
    longLearnSweepTotal = longLearnSweepIdx + 1; // the unknown-ID search adds reads as it goes
  }
}

// Full-lock read with only these unknown IDs through, logged. -1 = cancelled.
static int longLearnUnlistedRead(const uint8_t *slots, uint8_t n)
{
  longLearnPassUnlisted(slots, n);
  const uint8_t r = cycleAndRead(100);
  if (longLearnCancel)
    return -1;
  uint8_t pts[LL_NPTS];
  for (uint8_t k = 0; k < LL_NPTS; k++)
    pts[k] = 0xFF; // not read
  pts[LL_NPTS - 1] = r;
  longLearnLog(LLS_UNLISTED, n, r >= LL_REACH_MIN ? 1 : 0, pointScore(pts), pts);
  return r;
}

// The one unknown ID among slots[0..n) that cuts the lock on its own (the whole
// group is already known to). Halves until one is left. Returns its slot,
// -1 if neither half alone cuts it (a combination), -2 on cancel.
static int longLearnFindUnlisted(const uint8_t *slots, uint8_t n)
{
  uint8_t lo = 0, hi = n;
  while (hi - lo > 1)
  {
    const uint8_t mid = (uint8_t)((lo + hi) / 2);
    int r = longLearnUnlistedRead(slots + lo, mid - lo);
    if (r < 0)
      return -2;
    if (r < LL_REACH_MIN)
    {
      hi = mid; // the first half alone cuts it
      continue;
    }
    r = longLearnUnlistedRead(slots + mid, hi - mid);
    if (r < 0)
      return -2;
    if (r < LL_REACH_MIN)
    {
      lo = mid; // the second half alone cuts it
      continue;
    }
    return -1; // only together
  }
  return slots[lo];
}

// ---- Main code -----------------------------------------------------------------

void haldexLearnTask(void *arg)
{
  runLearnSweep(); // startHaldexLearn() has already primed the learn flags
  vTaskDelete(NULL);
}

void longLearnTask(void *arg)
{
  const uint8_t gi = longLearnGenIdx;
  uint64_t *mask = activeFrameEditMask(); // normal or standalone mask, whichever is live
  const bool isGen5 = (longLearnGeneration == 50 || longLearnGeneration == 52); // Fix Hunting exists on the 0CQ only

  // Everything needed to put the unit back exactly as it was on cancel/failure.
  longLearnMaskStart = mask[gi];
  const bool fixHuntingStart = fixHunting;
  uint32_t heldStart[HELD_BACK_MAX];
  memcpy(heldStart, heldBackIds, sizeof(heldStart));
  uint8_t savedTable[101];
  memcpy(savedTable, haldexLearnTable, sizeof(savedTable));
  const bool savedTableValid = haldexLearnTableValid;

  static uint8_t refA[101]; // the all-ours table - static to keep it off the task stack (one run at a time)
  static uint8_t slots[UNLISTED_MAX + 1];

  // All function-scope state is declared here, before the first goto.
  uint8_t outcome = LL_FAILED;
  LearnScore s = {}, sA = {};
  uint8_t a[LL_NPTS] = {};
  uint8_t nSlots = 0;
  uint8_t heldSlot[HELD_BACK_MAX] = {};
  uint64_t keepMask = 0;
  int r = 0;
  bool fixHuntingTurnedOn = false;

  // Block list for this generation: core blocks stay ours unless Test All.
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

  memset(heldBackIds, 0, sizeof(heldBackIds)); // this run decides them
  resetFrameCapture();                         // fresh car / our captures and unknown-ID list
  longLearnCarState(longLearnStartState);
  mask[gi] = allMask;              // every block ours
  unlistedPassMode = UNL_HOLD_ALL; // every other car ID held back = the bench bus
  longLearnSweepTotal = 1 + 1 + nCand + 1 + 1; // all ours + reference + blocks + unknown IDs + confirm

  // ---- 1. All ours: must reach full lock smoothly --------------------------
  longLearnPhase = LL_SWEEP;
  if (!longLearnSweep(sA))
  {
    outcome = LL_CANCELLED;
    goto restore;
  }
  longLearnLog(LLS_BASELINE, 0xFF, sA.smooth ? 1 : 0, sA);
  longLearnBaseline = sA;
  longLearnBaselineValid = true;
  if (!sA.smooth)
  {
    longLearnPhase = LL_BPK;
    if (isGen5)
    {
      // 0CQ: Fix Hunting (BPK packing) on - the torque ceiling only applies with it
      fixHuntingTurnedOn = !fixHunting;
      fixHunting = true;
    }
    if (!longLearnSweep(sA))
    {
      outcome = LL_CANCELLED;
      goto restore;
    }
    longLearnLog(LLS_BPK, 0xFF, sA.smooth ? 1 : 0, sA);

    if (isGen5 && !sA.smooth && sA.reach < LL_REACH_MIN)
    {
      // Still short on level: walk the torque ceiling up to the lowest value that
      // reaches full lock (a held read at 100 % per step), then prove it with a sweep.
      uint8_t bestReach = sA.reach;
      uint16_t bestNm = longLearnBpkStart;
      for (uint16_t nm = longLearnBpkStart + LL_BPK_STEP_NM; nm <= LL_BPK_MAX_NM; nm += LL_BPK_STEP_NM)
      {
        bpkCeilingNm = nm;
        const uint8_t reach = quickHoldEngagement(100, 1200);
        if (longLearnCancel)
        {
          outcome = LL_CANCELLED;
          goto restore;
        }
        LearnScore hold = {};
        hold.reach = reach;
        hold.score = reach;
        hold.engageCF = (reach > 0) ? 100 : 101;
        hold.smooth = (reach >= LL_REACH_MIN);
        longLearnLog(LLS_BPK, 0xFE, hold.smooth ? 1 : 0, hold);
        if (reach > bestReach)
        {
          bestReach = reach;
          bestNm = nm;
        }
        if (reach >= 100)
          break;
      }
      bpkCeilingNm = bestNm;
      if (bestNm != longLearnBpkStart)
      {
        if (!longLearnSweep(sA))
        {
          outcome = LL_CANCELLED;
          goto restore;
        }
        longLearnLog(LLS_BPK, 0xFF, sA.smooth ? 1 : 0, sA);
      }
    }
    longLearnBaseline = sA;
    if (!sA.smooth)
    {
      longLearnFailReason = (sA.engageCF > 100) ? LLF_NO_DATA : LLF_NOT_SMOOTH;
      goto restore;
    }
    longLearnFixHuntingSet = fixHuntingTurnedOn;                 // it worked with Fix Hunting: keep it
    longLearnBpkAdjusted = (bpkCeilingNm != longLearnBpkStart); // and with this ceiling
  }
  memcpy(refA, haldexLearnTable, sizeof(refA));

  // ---- 2. Quick all-ours read (the reference) -------------------------------
  longLearnPhase = LL_SWEEP;
  if (!quickPoints(longLearnRefPts))
  {
    outcome = LL_CANCELLED;
    goto restore;
  }
  longLearnLog(LLS_POINTS, 0xFF, 1, pointScore(longLearnRefPts), longLearnRefPts);
  if (longLearnRefPts[LL_NPTS - 1] < LL_REACH_MIN)
  {
    longLearnFailReason = LLF_REF_LOW; // the sweep made it, the held read didn't - not stable enough to judge swaps
    goto restore;
  }

  // ---- 3. Each block swapped to the car's frame on its own -----------------
  longLearnPhase = LL_BLOCKS;
  for (uint8_t i = 0; i < nBits; i++)
  {
    const uint8_t b = bits[i];
    if (longLearnBlockResult[b] != LLB_UNTESTED)
      continue;
    longLearnCurrentBit = b;
    const uint32_t testStartMs = millis();
    if (b < CAR_BLOCK_SLOTS)
      memcpy(longLearnOurBytes[b], ourBlockData[b], 8); // ours, as sent until now
    mask[gi] &= ~(1ULL << b);                           // the car's frame (or none) from here
    const bool ok = quickPoints(a);
    if (b < CAR_BLOCK_SLOTS)
    {
      const CarFrameSeen cf = carBlockFrame[b]; // copy: the chassis task keeps writing it
      longLearnCarSent[b] = (cf.lastMs != 0) && ((int32_t)(cf.lastMs - testStartMs) >= 0);
      longLearnCarDlc[b] = cf.dlc;
      memcpy(longLearnCarBytes[b], cf.data, 8);
      longLearnCarState(longLearnTestState[b]);
    }
    mask[gi] |= (1ULL << b); // back to ours - every block is judged against all ours
    if (!ok)
    {
      outcome = LL_CANCELLED;
      goto restore;
    }
    const uint8_t verdict = (a[LL_NPTS - 1] < LL_REACH_MIN) ? LLB_NEEDED : LLB_REMOVED;
    longLearnBlockResult[b] = verdict;
    longLearnLog(LLS_BLOCK, b, verdict, pointScore(a), a);
  }
  longLearnCurrentBit = -1;

  // ---- 4. Car IDs we have no block for -------------------------------------
  longLearnPhase = LL_UNLISTED;
  nSlots = unlistedCount;
  longLearnUnlistedSeen = nSlots;
  longLearnUnlistedOverflowSeen = unlistedOverflow;
  for (uint8_t i = 0; i < nSlots; i++)
    slots[i] = i;
  if (unlistedOverflow)
    slots[nSlots++] = UNLISTED_MAX; // the IDs beyond the table, as one group
  if (nSlots == 0)
  {
    longLearnUnlistedResult = LLU_NONE_SEEN;
  }
  else
  {
    longLearnPassUnlisted(slots, nSlots); // all of them through
    if (!quickPoints(a))
    {
      outcome = LL_CANCELLED;
      goto restore;
    }
    longLearnLog(LLS_UNLISTED, nSlots, a[LL_NPTS - 1] >= LL_REACH_MIN ? 1 : 0, pointScore(a), a);
    if (a[LL_NPTS - 1] >= LL_REACH_MIN)
    {
      longLearnUnlistedResult = LLU_NO_EFFECT;
    }
    else
    {
      longLearnUnlistedResult = LLU_FOUND;
      while (longLearnUnlistedHitCount < HELD_BACK_MAX)
      {
        // Candidates: every unknown ID not already found
        uint8_t cand[UNLISTED_MAX + 1];
        uint8_t nc = 0;
        for (uint8_t i = 0; i < nSlots; i++)
        {
          bool taken = false;
          for (uint8_t h = 0; h < longLearnUnlistedHitCount; h++)
            if (heldSlot[h] == slots[i])
              taken = true;
          if (!taken)
            cand[nc++] = slots[i];
        }
        if (nc == 0)
          break;
        const int found = longLearnFindUnlisted(cand, nc);
        if (found == -2)
        {
          outcome = LL_CANCELLED;
          goto restore;
        }
        if (found < 0)
        {
          if (longLearnUnlistedHitCount == 0)
            longLearnUnlistedResult = LLU_COMBINATION;
          break;
        }
        LLUnlistedHit &hit = longLearnUnlistedHits[longLearnUnlistedHitCount];
        if (found >= UNLISTED_MAX)
        {
          hit.id = 0xFFFFFFFFUL; // the overflow group - can't be held back by ID
          hit.extd = false;
          hit.dlc = 0;
          memset(hit.data, 0, sizeof(hit.data));
        }
        else
        {
          const CarFrameSeen cf = unlistedFrames[found];
          hit.id = cf.id;
          hit.extd = cf.extd;
          hit.dlc = cf.dlc;
          memcpy(hit.data, cf.data, 8);
        }
        heldSlot[longLearnUnlistedHitCount] = (uint8_t)found;
        longLearnUnlistedHitCount++;

        // Everything else through: still cut?
        nc = 0;
        for (uint8_t i = 0; i < nSlots; i++)
        {
          bool taken = false;
          for (uint8_t h = 0; h < longLearnUnlistedHitCount; h++)
            if (heldSlot[h] == slots[i])
              taken = true;
          if (!taken)
            cand[nc++] = slots[i];
        }
        if (nc == 0)
          break;
        r = longLearnUnlistedRead(cand, nc);
        if (r < 0)
        {
          outcome = LL_CANCELLED;
          goto restore;
        }
        if (r >= LL_REACH_MIN)
          break; // nothing else cuts it
      }
    }
  }
  unlistedPassMode = UNL_HOLD_ALL;

  // ---- 5. Confirmation on the final set -------------------------------------
  longLearnPhase = LL_FINAL;
  keepMask = allMask;
  for (uint8_t i = 0; i < nBits; i++)
  {
    if (longLearnBlockResult[bits[i]] == LLB_REMOVED)
      keepMask &= ~(1ULL << bits[i]);
  }
  mask[gi] = keepMask;
  for (uint8_t h = 0, n = 0; h < longLearnUnlistedHitCount; h++)
  {
    if (longLearnUnlistedHits[h].id != 0xFFFFFFFFUL)
      heldBackIds[n++] = longLearnUnlistedHits[h].id | (longLearnUnlistedHits[h].extd ? HELD_BACK_EXTD : 0);
  }
  unlistedPassMode = UNL_FOLLOW_MODE; // as it will be driven: normal mode passes the other car IDs
  haldexLearnActive = false;
  if (!longLearnSweep(s))
  {
    outcome = LL_CANCELLED;
    goto restore;
  }
  longLearnLog(LLS_FINAL, 0xFF, s.smooth ? 1 : 0, s);
  longLearnFinal = s;
  if (!s.smooth)
  {
    // Each swap was fine on its own but the final set together isn't - two
    // frames covering for each other. Don't guess: every block ours, and the
    // all-ours table that matches it.
    longLearnInteraction = true;
    mask[gi] = allMask;
    memcpy(haldexLearnTable, refA, sizeof(refA));
    haldexLearnTableValid = true;
    haldexLearnStep = 101;
    longLearnFinal = longLearnBaseline;
  }
  longLearnFinalValid = true;

  longLearnPhase = LL_DONE;
  longLearnEndMs = millis();
  longLearnActive = false;
  vTaskDelete(NULL);
  return;

restore:
  // Put back the mask, Fix Hunting, held-back IDs and learn table from before
  // the run so a cancelled / failed run never leaves a half-tested set behind.
  mask[gi] = longLearnMaskStart;
  fixHunting = fixHuntingStart;
  longLearnFixHuntingSet = false;
  bpkCeilingNm = longLearnBpkStart;
  longLearnBpkAdjusted = false;
  if (longLearnMoved)
  {
    outcome = LL_FAILED; // not a user cancel: say why
    longLearnFailReason = LLF_MOVED;
  }
  memcpy(heldBackIds, heldStart, sizeof(heldBackIds));
  unlistedPassMode = UNL_FOLLOW_MODE;
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

void showHaldexState(void *arg)
{
  while (1)
  {
    stackshowHaldexState = uxTaskGetStackHighWaterMark(NULL);

    DEBUG("Mode: %s", getOpenhaldexModeString(state.mode));
    DEBUG("    Req:Act: %d:%d", int(lockTarget), receivedHaldexEngagement); // this is the lock %

    if (detailedDebug)
    {
      DEBUG("    Raw haldexState: " BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(receivedHaldexState));
      DEBUG("    reportClutch1: %d", receivedReportClutch1); // this means it has a clutch issue
      DEBUG("    reportClutch2: %d", receivedReportClutch2); // this means it also has a clutch issue
      DEBUG("    couplingOpen: %d", receivedCouplingOpen);   // clutch fully disengaged
      DEBUG("    speedLimit: %d", receivedSpeedLimit);       // hit a speed limit...

      DEBUG("    hasChassisCAN: %d", hasCANChassis);         // incrememting value for checking the response to vars...
      DEBUG("    hasHaldexCAN: %d", hasCANHaldex);           // incrememting value for checking the response to vars...
      DEBUG("    lastCANChassis: %ldms", lastCANChassisTick != 0 ? (long)(millis() - lastCANChassisTick) : -1);   // incrememting value for checking the response to vars...
      DEBUG("    lastHaldex: %ldms", lastCANHaldexTick != 0 ? (long)(millis() - lastCANHaldexTick) : -1); // incrememting value for checking the response to vars...

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

      DEBUG("    stackStandaloneFrames: %d", (int)uxTaskGetStackHighWaterMark(handleStandaloneFrames));

      DEBUG("    stackbroadcastOpenHaldex: %d", stackbroadcastOpenHaldex); // incrememting value for checking the response to vars...
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
      twai_get_status_info_v2(twaiBus0, &twaistatus0);
      twai_get_status_info_v2(twaiBus1, &twaistatus1);
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
