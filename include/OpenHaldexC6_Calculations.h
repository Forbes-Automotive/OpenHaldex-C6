#pragma once

#include <OpenHaldexC6_defs.h>

// Learn Haldex table
extern uint8_t haldexLearnTable[101];
extern bool haldexLearnTableValid;
extern volatile bool haldexLearnActive;
extern volatile bool haldexLearnCancel;
extern volatile uint8_t haldexLearnStep;
extern volatile uint8_t haldexLearnCF;

float get_lock_target_adjustment();

// Which force-mode value applies right now: 0..5 (Stock/FWD/5050/6040/7525/
// Expert) when an enabled force trigger's flag is active (priority picked by
// forceModesPriority), or -1 when no force mode applies. Used by
// get_lock_target_adjustment and by the inline gateway to detect "effective
// mode is Stock", which must mean untouched passthrough - never frame edits
// built from mirrored engagement (the stuck-at-100% feedback loop).
int get_forced_mode_value();
static float get_expert_lock_target();
uint8_t get_lock_target_adjusted_value(uint8_t value, bool invert);
void getLockData(twai_message_t& rx_message_chs);
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
// disengages (found by trial and error on real hardware). A lock_target-
// proportional front/rear delta was tried here and made things worse on the
// car (lock faded then collapsed to 0%), so this stays the flat, proven
// dither. Fills data[0..7] and advances the shared counters.
void fill_esp19_wheel_speeds(uint8_t data[8]);

// Motor_11 (0x0A7) BPK packing, shared by standalone generation and normal-mode
// in-place editing so the two can't drift. Fills data[0..7] (byte 0 is left as
// a CRC placeholder for the caller) from the runtime BPK tunables in defs.h.
void fill_motor11_bpk(uint8_t data[8], uint8_t counter);

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

// ---- Long Learn (automated frame-block bisection) --------------------------
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
#define LL_REACH_MIN 90     // % engagement required at CF 100
#define LL_STEP_MAX 8       // largest tolerated single-step rise after engage
#define LL_ENGAGE_MAX_CF 60 // must have started engaging by this CF
#define LL_TOLERANCE 4      // minimum deviation (%) treated as "no change";
                            // raised to the measured reference-to-reference noise
#define LL_NOISE_MAX 10     // two all-on point reference reads further apart than
                            // this (worst point) are too noisy to judge blocks by
// CF points each block is read at - spread over the ramp so a block that only
// matters at part lock (a step at 20%, say) isn't missed on a clean 100%.
#define LL_NPTS 4
static const uint8_t LL_POINT_CF[LL_NPTS] = {20, 40, 70, 100};
#define LL_BPK_ACCEPT 90    // return% Long Learn treats as good enough before it
                            // starts changing BPK settings. 100% is the aim, but
                            // 90+ is accepted rather than chasing the last few
                            // points into the pressure-relief regime.
void scoreLearnTable(const uint8_t *table, LearnScore &out);

// Phase numbers are also sent over ESP-NOW (ohx status longPhase) - append only.
enum
{
    LL_IDLE = 0,
    LL_SWEEP,     // "Reference": all-on sweeps that must prove a smooth 100% first
    LL_BPK,       // "BPK Adjust" (Gen5 only): hunting/short -> BPK packing + torque ceiling
    LL_BLOCKS,    // "Testing Blocks": each candidate off on its own, full sweep, back on
    LL_FINAL,     // confirmation sweep on the final set
    LL_DONE,
    LL_CANCELLED,
    LL_FAILED     // see longLearnFailReason - previous state restored
};
enum
{
    LLF_NONE = 0,
    LLF_NO_DATA,    // a reference sweep got no Haldex feedback at all
    LLF_NOT_SMOOTH, // all blocks on did not give a smooth 100% - nothing to compare against
    LLF_NOISY       // the two point reference reads disagree by more than LL_NOISE_MAX
};
enum
{
    LLB_UNTESTED = 0, // candidate, not yet tested
    LLB_CORE,         // lock-driven block - always sent, never tested (unless Test All)
    LLB_NEEDED,       // off on its own read lower at the test points -> kept on
    LLB_REMOVED,      // off on its own made no difference -> left off
    LLB_HARMFUL       // off on its own read higher -> still kept on, flagged
};
enum
{
    LLS_BASELINE = 0, // all-on full sweep (first sweep / reference curve)
    LLS_POINTS,       // all-on LL_POINT_CF reference read (was LLS_FLOOR, unused)
    LLS_BLOCK,        // one candidate block off, LL_POINT_CF read
    LLS_FINAL,        // confirmation sweep
    LLS_BPK           // BPK packing / torque-ceiling candidate (Gen5 only)
};
struct LongLearnSweep
{
    uint8_t kind;     // LLS_*
    uint8_t bit;      // block bit under test (0xFF = n/a)
    uint8_t floorPct; // esp14MinFloorPct during the sweep
    uint16_t bpkNm;   // bpkCeilingNm during the sweep (Gen5 only; 0 elsewhere)
    uint8_t verdict;  // LLB_* for block sweeps, 1/0 smooth/reached-100 for the rest
    uint8_t maxDev;   // block: worst |read - ref| over the points; final: over CF 0-100
    int8_t meanDelta; // block/final: mean (read - reference), sign = direction
    uint8_t pts[LL_NPTS]; // point reads (LLS_POINTS / LLS_BLOCK), else 0
    LearnScore s;     // point reads: reach = 100% point, engage = first non-zero point
};
#define LL_MAX_SWEEPS 80
#define LL_NOTES_LEN 200

extern volatile bool longLearnActive;
extern volatile bool longLearnCancel;
extern volatile uint8_t longLearnPhase;      // LL_*
extern volatile uint8_t longLearnSweepIdx;   // sweeps completed so far
extern volatile uint8_t longLearnSweepTotal; // estimated total (exact after the floor phase)
extern volatile int16_t longLearnCurrentBit; // block being tested (-1 = none)
extern uint8_t longLearnGenIdx;              // FE_GEN_* the run belongs to
extern uint8_t longLearnGeneration;          // haldexGeneration the run belongs to
extern bool longLearnTestAll;                // also bisect the default (core) blocks
extern uint8_t longLearnBlockResult[64];     // LLB_* per bit
extern LearnScore longLearnBaseline;
extern LearnScore longLearnFinal;
extern bool longLearnBaselineValid;
extern bool longLearnFinalValid;
extern uint8_t longLearnFloorStart;  // esp14MinFloorPct before the run
extern uint8_t longLearnFloorResult; // esp14MinFloorPct chosen by the run
extern uint64_t longLearnMaskStart;  // active mask before the run (restored on cancel)
extern uint16_t longLearnBpkStart;   // bpkCeilingNm before the run (Gen5; restored on cancel/failure)
extern bool longLearnBpkAdjusted;    // true if the BPK-adjust phase changed packing or ceiling
extern uint8_t longLearnFailReason;  // LLF_* when longLearnPhase == LL_FAILED
extern uint8_t longLearnNoise;       // worst |refA - refB| over CF 0-100
extern uint8_t longLearnTol;         // deviation threshold actually used (max(LL_TOLERANCE, noise))
extern bool longLearnInteraction;    // the "no effect" blocks off TOGETHER changed the curve -> all left on
extern LongLearnSweep longLearnSweeps[LL_MAX_SWEEPS];
extern uint8_t longLearnSweepCount;
extern uint32_t longLearnStartMs;
extern uint32_t longLearnEndMs;
extern char longLearnNotes[LL_NOTES_LEN + 1]; // user chassis/car notes (exported with the report)

bool startLongLearn(bool testAll); // false if already running / not a gated generation

// Steering-angle lock-scale telemetry (for the engagement-split display).
bool steering_scale_is_active();
uint8_t steering_scale_requested_pct();
uint8_t steering_scale_result_pct();

// Geometry-compensated per-corner slip (adopted from OpenHaldex-Edge by Rekt /
// Kile Thomson - see THIRD_PARTY_NOTICES.md). wheel_raw is [FL, FR, RL, RR] in ESP_19
// units; steer_wheel_tenths is signed steering-wheel angle in 0.1 deg. Returns
// false (and all-zero slip_out) when the car/geometry is degenerate or too slow;
// otherwise fills slip_out[4] with signed slip % per corner. Pure math.
bool compute_corner_slip(const uint16_t wheel_raw[4], int16_t steer_wheel_tenths,
                         float steering_ratio, uint16_t wheelbase_mm,
                         uint16_t track_front_mm, uint16_t track_rear_mm,
                         uint16_t min_speed_raw, int8_t slip_out[4]);