#pragma once

#include <OpenHaldexC6_defs.h>

// Learn Haldex table
extern uint8_t haldexLearnTable[101];
extern bool haldexLearnTableValid;
extern volatile bool haldexLearnActive;
extern volatile bool haldexLearnCancel;
extern volatile uint8_t haldexLearnStep;
extern volatile uint8_t haldexLearnCF;
void learnTableForget(); // V11.00.15: drop the learn table (generation changed - it was learned on another unit)

float getLockTargetAdjustment();

// Which of the speed / throttle settings is holding lock at 0% right now, as
// LOCK_GATE_* bits (see _defs.h); 0 = nothing is blocking.
uint8_t lockGateBlocking();

// Which force-mode value applies right now: 0..5 (Stock/FWD/5050/6040/7525/
// Expert) when an enabled force trigger's flag is active (priority picked by
// forceModesPriority), or -1 when no force mode applies. Used by
// getLockTargetAdjustment and by the inline gateway to detect "effective
// mode is Stock", which must mean untouched passthrough - never frame edits
// built from mirrored engagement (the stuck-at-100% feedback loop).
int getForcedModeValue();

// The mode the car's drive mode select is forcing right now (0..5), or -1 when the
// feature is off, the drive profile hasn't been read in the last 3 s, or that
// profile is set to N/A. Lowest priority: TC / hazards / external button win.
int getDriveModeForcedValue();
static float getExpertLockTarget();
uint8_t getLockTargetAdjustedValue(uint8_t value, bool invert);

// Direct-lock request units (V11.00.0) - see OpenHaldexC6_Calculations.cpp
uint8_t lockRequestPercent();               // 0-100 % to ask for now (learn sweep, learn table, lock gates applied)
uint8_t vagLockRequestByte(uint8_t fullSteps, uint8_t fullLockByte); // Bremse_4 byte 0: 0x7F at 0 %, 0x81..0xFC, fullLockByte at 100 %

// Brake / handbrake / ABS for the Haldex (V11.00.0)
bool brakeForHaldex();      // Follow Brake on and the brake pedal is pressed (wire or CAN)
bool handbrakeForHaldex();  // Follow Handbrake on and the handbrake is pulled (wire or CAN)
bool absBrakingForHaldex(); // Follow ABS on and the ABS module is braking

// Gen1 02D lock request (V11.00.0) - see OpenHaldexC6_Calculations.cpp
uint8_t gen2LockRequestByte();                       // V11.00.13 Bremse_4 byte 0 ceiling: 0x7F at 0 %, 0x81..0xE9, 0xFD at 100 %
uint8_t gen1LockRequestByte();                      // Bremse_4 byte 0 ceiling: 0x7F at 0 %, 0x81..0xC9, 0xFD at 100 %
bool gen1CeilingRequestActive();                // true when Bremse_4 byte 0 is the lock request (standalone, or the car sends 0x2A0)
uint8_t gen1DemandValue(uint8_t fullValue);    // Motor_1 / Bremse_3 demand byte (max while the ceiling is the lock request)
uint8_t gen1LossTorqueByte(uint8_t oemValue); // Motor_1 byte 6 loss torque (subtracted by the unit)

// Gen2 7H0 / 1K0 (V11.00.0) - see OpenHaldexC6_Calculations.cpp
uint8_t gen2DemandValue(uint8_t fullValue);      // Motor_1 b1/b5, Bremse_3 slip byte: full while the Bremse_4 lock request is on the bus and lock is wanted

// ---- Our frames vs the car's (V11.00.7) -------------------------------------
// Every frame-edit generation, both modes: a block ON = the Haldex gets OUR
// whole frame for that ID (the standalone builders) and the car's copy is held
// back; OFF = the car's own frame goes through untouched. Car IDs that aren't
// in the block table go through in normal mode and are held back in
// standalone, except during Long Learn (unlistedPassMode) and the IDs Long
// Learn found cut the lock (heldBackIds - never passed).
bool stockEffective();    // effective mode is Stock (force triggers applied), or Expert sitting in a PT area
bool expertPtActive();   // V11.00.12: Expert is the mode in use and the live point is in a PT area of the map
bool ownFramesActive();  // our frames are going to the Haldex right now
void updateLockTarget(); // lockTarget from mode / triggers / gates, with the release rate limit

#define CAR_BLOCK_SLOTS 32 // per-bit frame capture: every generation's blocks fit in bits 0-31
#define UNLISTED_MAX 64    // car IDs outside the block table we keep track of (more = one "overflow" group)
#define HELD_BACK_MAX 4    // car IDs Long Learn can keep away from the Haldex

struct CarFrameSeen
{
    uint32_t id;     // CAN ID
    uint32_t lastMs; // millis() when last seen (0 = never)
    uint8_t data[8];
    uint8_t dlc;
    bool extd;       // 29-bit ID
};
extern CarFrameSeen carBlockFrame[CAR_BLOCK_SLOTS]; // last car frame per block bit (active generation)
extern uint8_t ourBlockData[CAR_BLOCK_SLOTS][8];    // last frame WE sent per block bit
extern uint32_t ourBlockMs[CAR_BLOCK_SLOTS];        // millis() of that send (0 = never)
extern CarFrameSeen unlistedFrames[UNLISTED_MAX];   // car IDs outside the block table, in order first seen
extern volatile uint8_t unlistedCount;
extern volatile bool unlistedOverflow;              // more unlisted IDs than UNLISTED_MAX were seen

enum
{
    UNL_FOLLOW_MODE = 0, // normal mode: pass; standalone: hold back (the bench bus)
    UNL_HOLD_ALL,        // Long Learn "all ours": every unlisted car ID held back
    UNL_PASS_SET         // Long Learn: only unlistedPassSet (+ the overflow group if unlistedPassOverflow) go through
};
extern volatile uint8_t unlistedPassMode;
extern volatile uint64_t unlistedPassSet;  // bit i = unlistedFrames[i]
extern volatile bool unlistedPassOverflow;
extern uint32_t heldBackIds[HELD_BACK_MAX]; // ID | HELD_BACK_EXTD for 29-bit; 0 = empty slot. Saved.
#define HELD_BACK_EXTD 0x80000000UL

void noteOurFrame(const twai_message_t &f);     // standaloneTx(): remember what we sent per block
bool carFrameToHaldex(const twai_message_t &f); // can.cpp: should this car frame go to the Haldex?
void resetFrameCapture();                       // forget captured car / our frames and unlisted IDs

void startHaldexLearn();

// Blocking learn sweep (CF 0..100, 300 ms/step) - the body of the manual Learn
// task, shared with Long Learn. Must be called from a task. preHoldMs > 0 holds
// CF=0 first until the Haldex has released (or the hold times out) so residual
// engagement from a previous sweep does not lift the bottom of the table.
// Returns true when the sweep completed and recorded at least one non-zero
// engagement (i.e. haldexLearnTableValid was set).
bool runLearnSweep(uint32_t preHoldMs = 0);

// Gen5 (0CQ/VAQ) ESP_19 wheel-speed simulation, shared by standalone frame
// generation and normal-mode in-place editing (both call this so the two
// copies can't drift). Wheel speed MUST keep changing or the Haldex slowly
// disengages (found by trial and error on real hardware). A lockTarget-
// proportional front/rear delta was tried here and made things worse on the
// car (lock faded then collapsed to 0%), so this stays the flat, proven
// dither. Fills data[0..7] and advances the shared counters.
// V11.00.0: the speeds no longer scale with the lock (OEM code, spec_gen5_0cq
// D6) - they are always the values that used to be sent at 100 % lock.
void fillEsp19WheelSpeeds(uint8_t data[8]);

// Gen5 0CQ ESP_14 lock request (V11.00.0) - see OpenHaldexC6_Calculations.cpp.
// 0..250 for ESP_14 bytes 6 AND 7 (Min = Max = exact clutch target, 0.4 %/step).
uint8_t gen5Esp14LockRequestByte();

// Motor_11 (0x0A7) BPK packing, shared by standalone generation and normal-mode
// in-place editing so the two can't drift. Fills data[0..7] (byte 0 is left as
// a CRC placeholder for the caller) from the runtime BPK tunables in defs.h.
void fillMotor11Bpk(uint8_t data[8], uint8_t counter);

// Danger Zone is live this cycle: toggle on, full lock requested (> 99 %), not
// in a learn sweep. When true the Motor_11 packers use BPK packing with the
// ceiling raised to dangerZoneNm, whatever the Fix Hunting toggle says.
bool dangerZoneActive();

// Stores what the Motor_11 BPK packer computed this cycle, for the serial lab
// task to stream out. Called from both BPK code paths at the Motor_11 rate.
void bpkLogSample(uint16_t torqueNm, uint16_t istNm, uint16_t solfNm);

// ---- Serial lab (USB diagnostic harness) ------------------------------------
// Line-based control + telemetry over USB serial so a host script can drive
// ceiling/floor/lock/packing values and watch the Haldex respond in real time,
// instead of rebuilding firmware per experiment. See OpenHaldexC6_SerialLab.cpp.
void setupSerialLab();

// ---- Long Learn (V11.00.7: our frames vs the car's) -------------------------
// Finds which frame the Haldex needs from US in this car. All ours (= the
// bench bus) must reach LL_REACH_MIN smoothly; then each block is swapped to
// the car's frame on its own - a drop below LL_REACH_MIN at full lock means
// KEEP OURS - and the car IDs we don't know are let through to find one we're
// missing. See longLearnTask() in tasks.cpp for the flow.
// Scores a learn table for "smoothness": the Haldex may jump on its first
// engage step (e.g. 0 -> 30 %) but after that must climb without steps larger
// than LL_STEP_MAX and reach LL_REACH_MIN by CF 100.
struct LearnScore
{
    uint8_t reach;      // engagement at CF 100
    uint8_t maxStep;    // largest single-step rise AFTER the first engage step
    uint8_t engageCF;   // first CF with non-zero engagement (101 = never)
    uint8_t engageJump; // engagement recorded at engageCF
    uint8_t score;      // 0-100 composite used to rank configurations
    bool smooth;        // passes all three smoothness criteria
};
#define LL_REACH_MIN 90     // % engagement at CF 100 that counts as "full lock" (all ours must reach it; a swap below it = keep ours)
#define LL_STEP_MAX 8       // largest tolerated single-step rise after engage
#define LL_ENGAGE_MAX_CF 60 // must have started engaging by this CF
#define LL_BPK_STEP_NM 40   // 0CQ retry: torque-ceiling walk step (units differ in the ceiling they need)
#define LL_BPK_MAX_NM 500   // 0CQ retry: highest ceiling the walk tries
#define LL_MOVING_KMH 5     // Long Learn stops (and restores) above this road speed
// CF points each swap is read at. Only the 100 % point decides; the others are
// logged so a part-lock change can still be seen in the report.
#define LL_NPTS 4
static const uint8_t LL_POINT_CF[LL_NPTS] = {20, 40, 70, 100};
void scoreLearnTable(const uint8_t *table, LearnScore &out);

// Phase numbers are also sent over ESP-NOW (ohx status longPhase) - append only.
enum
{
    LL_IDLE = 0,
    LL_SWEEP,     // "All ours": every block ours, every car ID held back, full sweep
    LL_BPK,       // "Retry": all ours not smooth / short - again; 0CQ: Fix Hunting on, then the torque-ceiling walk
    LL_BLOCKS,    // "Car's frames": each block swapped to the car's on its own
    LL_FINAL,     // confirmation sweep on the final set
    LL_DONE,
    LL_CANCELLED,
    LL_FAILED,    // see longLearnFailReason - previous state restored
    LL_UNLISTED   // "Unknown car IDs": the car IDs we don't have a block for
};
enum
{
    LLF_NONE = 0,
    LLF_NO_DATA,    // all ours got no Haldex feedback at all
    LLF_NOT_SMOOTH, // all ours did not give a smooth LL_REACH_MIN (after the retry)
    LLF_REF_LOW,    // the all-ours quick read at 100 % was below LL_REACH_MIN
    LLF_MOVED       // the car moved during the run
};
enum
{
    LLB_UNTESTED = 0, // not tested (yet)
    LLB_CORE,         // core block - always ours, not tested (unless Test All)
    LLB_NEEDED,       // KEEP OURS: with the car's frame (or none) full lock dropped below LL_REACH_MIN
    LLB_REMOVED,      // the car's frame is fine (or the car doesn't send it and ours isn't needed) -> off
    LLB_HARMFUL       // (no longer produced - kept so old numbers read the same)
};
enum
{
    LLS_BASELINE = 0, // all ours, full sweep
    LLS_POINTS,       // all ours, LL_POINT_CF quick read (the reference)
    LLS_BLOCK,        // one block swapped to the car's, LL_POINT_CF read
    LLS_FINAL,        // confirmation sweep
    LLS_BPK,          // retry: a full sweep (bit 0xFF) or a torque-ceiling hold at 100 % (bit 0xFE, 0CQ)
    LLS_UNLISTED      // unknown car IDs let through: bit = how many IDs, pts = the read
};
struct LongLearnSweep
{
    uint8_t kind;     // LLS_*
    uint8_t bit;      // LLS_BLOCK: block bit; LLS_UNLISTED: number of IDs let through; else 0xFF
    uint8_t verdict;  // LLB_* for block reads, 1 = reached full lock for unlisted reads, 1/0 smooth for sweeps
    bool fixHunting;  // Fix Hunting during this read (0CQ)
    uint16_t bpkNm;   // torque ceiling (bpkCeilingNm) during this read (0CQ)
    uint8_t pts[LL_NPTS]; // point reads (LLS_POINTS / LLS_BLOCK / LLS_UNLISTED), else 0
    LearnScore s;     // full sweeps: the score; point reads: reach = the 100 % point
};
#define LL_MAX_SWEEPS 80
#define LL_NOTES_LEN 200

// The car while a test ran (from the car's CAN and our input wires)
struct LLCarState
{
    uint8_t flags;  // LLST_*
    uint16_t speed; // km/h
    uint16_t rpm;
};
#define LLST_BRAKE_CAN 0x01     // brake pedal (car CAN)
#define LLST_HANDBRAKE_CAN 0x02 // handbrake (car CAN)
#define LLST_ABS_CAN 0x04       // ABS braking (car CAN)
#define LLST_BRAKE_WIRE 0x08    // brake input wire
#define LLST_HANDBRAKE_WIRE 0x10 // handbrake input wire
#define LLST_CHASSIS_CAN 0x80   // the car's CAN was being received

// Unknown car IDs (no block in our table)
enum
{
    LLU_NOT_RUN = 0,  // not reached
    LLU_NONE_SEEN,    // the car sent nothing we don't know (or no car - bench)
    LLU_NO_EFFECT,    // all of them through together: still full lock
    LLU_FOUND,        // the ID(s) in longLearnUnlistedHits cut the lock - now held back
    LLU_COMBINATION   // they cut the lock together but no single half of them does - not narrowed down
};
struct LLUnlistedHit
{
    uint32_t id;     // CAN ID (0xFFFFFFFF = the overflow group of IDs beyond UNLISTED_MAX)
    bool extd;
    uint8_t dlc;
    uint8_t data[8]; // the car's bytes
};

extern volatile bool longLearnActive;
extern volatile bool longLearnCancel;
extern volatile uint8_t longLearnPhase;      // LL_*
extern volatile uint8_t longLearnSweepIdx;   // reads completed so far
extern volatile uint8_t longLearnSweepTotal; // estimated total
extern volatile int16_t longLearnCurrentBit; // block being tested (-1 = none)
extern uint8_t longLearnGenIdx;              // FE_GEN_* the run belongs to
extern uint8_t longLearnGeneration;          // haldexGeneration the run belongs to
extern bool longLearnTestAll;                // also swap the core blocks
extern bool longLearnWasStandalone;          // mode the run was made in
extern uint8_t longLearnBlockResult[64];     // LLB_* per bit
extern LearnScore longLearnBaseline;         // all ours
extern LearnScore longLearnFinal;
extern bool longLearnBaselineValid;
extern bool longLearnFinalValid;
extern uint64_t longLearnMaskStart;          // active mask before the run (restored on cancel)
extern bool longLearnFixHuntingSet;          // the retry turned Fix Hunting on and it was kept
extern uint16_t longLearnBpkStart;           // bpkCeilingNm before the run (restored on cancel / failure)
extern bool longLearnBpkAdjusted;            // the retry's ceiling walk changed bpkCeilingNm and it was kept
extern volatile bool longLearnMoved;         // the car moved: the run was stopped
extern uint8_t longLearnFailReason;          // LLF_* when longLearnPhase == LL_FAILED
extern uint8_t longLearnRefPts[LL_NPTS];     // all-ours quick read
extern bool longLearnInteraction;            // the confirmation failed -> every block left ours
extern LLCarState longLearnStartState;       // the car when the run started
// Per block bit, captured during that block's swap test:
extern bool longLearnCarSent[CAR_BLOCK_SLOTS];        // the car sent this ID during the test
extern uint8_t longLearnCarDlc[CAR_BLOCK_SLOTS];
extern uint8_t longLearnCarBytes[CAR_BLOCK_SLOTS][8]; // the car's frame
extern uint8_t longLearnOurBytes[CAR_BLOCK_SLOTS][8]; // ours (all ours, just before the swap)
extern LLCarState longLearnTestState[CAR_BLOCK_SLOTS]; // the car during the test
// Unknown car IDs
extern uint8_t longLearnUnlistedResult;               // LLU_*
extern uint8_t longLearnUnlistedSeen;                 // how many were seen by that step
extern bool longLearnUnlistedOverflowSeen;
extern LLUnlistedHit longLearnUnlistedHits[HELD_BACK_MAX];
extern uint8_t longLearnUnlistedHitCount;
extern LongLearnSweep longLearnSweeps[LL_MAX_SWEEPS];
extern uint8_t longLearnSweepCount;
extern uint32_t longLearnStartMs;
extern uint32_t longLearnEndMs;
extern char longLearnNotes[LL_NOTES_LEN + 1]; // user chassis/car notes (exported with the report)

// false if already running / not a frame-edit generation. *why gets a reason.
bool startLongLearn(bool testAll, const char **why = nullptr);
void longLearnStopIfMoving(); // called from every Long Learn wait: stops the run if the car moves

// Steering-angle lock-scale telemetry (for the engagement-split display).
bool steeringScaleIsActive();
uint8_t steeringScaleRequestedPct();
uint8_t steeringScaleResultPct();
uint8_t steeringThrottleOverridePct(); // how much of the steering reduction the throttle is cancelling (0-100 %)

// Geometry-compensated per-corner slip (adopted from OpenHaldex-Edge by Rekt /
// Kile Thomson - see THIRD_PARTY_NOTICES.md). wheelRaw is [FL, FR, RL, RR] in ESP_19
// units; steerWheelTenths is signed steering-wheel angle in 0.1 deg. Returns
// false (and all-zero slipOut) when the car/geometry is degenerate or too slow;
// otherwise fills slipOut[4] with signed slip % per corner. Pure math.
bool computeCornerSlip(const uint16_t wheelRaw[4], int16_t steerWheelTenths,
                         float steeringRatio, uint16_t wheelbaseMm,
                         uint16_t trackFrontMm, uint16_t trackRearMm,
                         uint16_t minSpeedRaw, int8_t slipOut[4]);