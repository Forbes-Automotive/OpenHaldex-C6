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
#define LL_TOLERANCE 4      // score band treated as "no change" (sweep-to-sweep noise)
#define LL_MID_CF 40        // part-lock point blocks are also checked at, so a
                            // block that only matters mid-range isn't dropped
                            // on the strength of a clean 100% reading alone
#define LL_BPK_ACCEPT 90    // return% Long Learn treats as good enough before it
                            // starts changing BPK settings. 100% is the aim, but
                            // 90+ is accepted rather than chasing the last few
                            // points into the pressure-relief regime.
void scoreLearnTable(const uint8_t *table, LearnScore &out);

enum
{
    LL_IDLE = 0,
    LL_SWEEP,     // "Initial Sweep": baseline / (Gen5) PWM-floor tuning with every block on
    LL_BPK,       // "BPK Adjust" (Gen5 only): raise the torque ceiling until 100% is reachable
    LL_BLOCKS,    // "Sweeping Blocks": quick on/off check of each candidate block at 100%
    LL_FINAL,     // confirmation sweep on the final set
    LL_DONE,
    LL_CANCELLED,
    LL_FAILED     // no Haldex data / sweep aborted - previous state restored
};
enum
{
    LLB_UNTESTED = 0, // candidate, not yet tested
    LLB_CORE,         // default block - kept on, never tested (unless Test All)
    LLB_NEEDED,       // removing it degraded the learn -> kept on
    LLB_REMOVED,      // removing it made no difference -> left off
    LLB_HARMFUL       // removing it improved the learn -> still kept on (any effect = keep), flagged
};
enum
{
    LLS_BASELINE = 0, // first all-on sweep
    LLS_FLOOR,        // further all-on sweep at a different PWM floor
    LLS_BLOCK,        // one candidate block removed (quick on/off check)
    LLS_FINAL,        // confirmation sweep
    LLS_BPK           // BPK torque-ceiling candidate (Gen5 only, quick check)
};
struct LongLearnSweep
{
    uint8_t kind;     // LLS_*
    uint8_t bit;      // block bit under test (0xFF = n/a)
    uint8_t floorPct; // esp14MinFloorPct during the sweep
    uint16_t bpkNm;   // bpkCeilingNm during the sweep (Gen5 only; 0 elsewhere)
    uint8_t verdict;  // LLB_* for block sweeps, 1/0 smooth/reached-100 for the rest
    LearnScore s;
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
extern bool longLearnBpkAdjusted;    // true if the BPK-adjust phase actually ran this run
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