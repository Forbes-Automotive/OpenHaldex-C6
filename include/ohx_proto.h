#pragma once
// ---------------------------------------------------------------------------
// OHX - OpenHaldex over ESP-NOW (wire format v1). SHARED FILE: identical copies live in
//   can2gauge/include/ohx_proto.h  and  OpenHaldex-C6/include/ohx_proto.h
// Change both together and bump OHX_PROTO on any layout change.
//
// OpenHaldex -> broadcast   OHX_STATUS every 100 ms (and straight after a command), on its AP's WiFi channel.
//                            Anything in range on that channel hears it (no pairing, no association): a gauge with
//                            power only - no CAN - still gets the Haldex lock, mode, speed, rpm, boost ...
// gauge -> OpenHaldex       OHX_COMMAND, unicast to the MAC the status came from (ESP-NOW acks + retries it at the
//                            MAC layer). Signed: hmac = HMAC-SHA256(key, every byte before hmac), first 8 bytes, where
//                            key = OpenHaldex's WiFi AP password ("" for an open AP - the same access its web UI gives).
//                            nonce must be the one in the latest status: OpenHaldex picks a new random nonce at boot
//                            and after every command it accepts, so a recorded command can never be played again.
//                            The result comes back in the next status (ackSrc / ackSeq / ackResult).
// Both ends must be on the same WiFi channel: in the car both APs default to channel 1; if either joins a home network
// (bridge mode) the other must join the same one.
// ---------------------------------------------------------------------------
#include <stdint.h>

#define OHX_MAGIC0 'O'
#define OHX_MAGIC1 'H'
#define OHX_PROTO 1
#define OHX_STATUS_MS 100 // broadcast period
#define OHX_NONE8 0xFF    // "no value" for uint8 fields
#define OHX_NONE16 0xFFFF // ... uint16 fields
#define OHX_NONE_T (-32768) // ... int16 temperatures

enum ohxType_t : uint8_t
{
  OHX_T_STATUS = 1,
  OHX_T_COMMAND = 2,
};

// flags[0]
#define OHX_F0_STANDALONE 0x01
#define OHX_F0_CHASSIS_CAN 0x02 // chassis CAN alive
#define OHX_F0_HALDEX_CAN 0x04  // Haldex CAN alive (lockActual valid)
#define OHX_F0_CTRL_OFF 0x08    // controller disabled (stock pass-through)
#define OHX_F0_BUS_FAIL 0x10
#define OHX_F0_OVERRIDE 0x20    // a force mode (TC / hazard / external button) is overriding the mode
#define OHX_F0_CAN_BCAST 0x40   // 0x6B0 broadcast / 0x6A0 control enabled
#define OHX_F0_LEARNED 0x80     // a learned engagement table is in use
// flags[1]
#define OHX_F1_TC_FORCE 0x01    // traction control off -> force mode
#define OHX_F1_HAZARD 0x02      // hazards on -> force mode
#define OHX_F1_EXT_BTN 0x04     // external button -> force mode
#define OHX_F1_BRAKE 0x08       // brake active (as sent to the Haldex)
#define OHX_F1_HANDBRAKE 0x10
#define OHX_F1_TEMP_PROT 0x20   // Haldex temperature protection
#define OHX_F1_COUPLING_OPEN 0x40
#define OHX_F1_SPEED_LIMIT 0x80
// flags[2]
#define OHX_F2_LEARN 0x01       // learn sweep running (learnStep = progress)
#define OHX_F2_LONG_LEARN 0x02  // long learn running (longPhase / longSweep / longTotal)
#define OHX_F2_CONTROL 0x04     // commands over ESP-NOW are accepted
#define OHX_F2_KEY 0x08         // its AP has a password: commands must be signed with it
#define OHX_F2_FIX_HUNTING 0x10
#define OHX_F2_STEER_SCALE 0x20 // steering-angle lock scaling pulled the request back this cycle
#define OHX_F2_ANALYZER 0x40    // analyzer mode (no control)
#define OHX_F2_LIVE_DIAG 0x80   // live diagnostics on (clutch / oil temperatures, supply)
// flags[3]
#define OHX_F3_ASR_OFF 0x01

enum ohxCmd_t : uint8_t
{
  OHX_C_MODE = 1,           // arg0 = mode (0 Stock, 1 FWD, 2 50:50, 3 60:40, 4 75:25, 5 Expert)
  OHX_C_GENERATION = 2,     // arg0 = generation (1, 2, 4, 41, 50, 51, 52 ...) - car stopped only
  OHX_C_LEARN_START = 3,    // car stopped only
  OHX_C_LEARN_CANCEL = 4,
  OHX_C_LEARN_CLEAR = 5,    // car stopped only
  OHX_C_LONG_START = 6,     // arg0 = 1: also test the default blocks - car stopped only
  OHX_C_LONG_CANCEL = 7,
  OHX_C_CONTROLLER = 8,     // arg0 = 1 enable, 0 disable (stock)
};

enum ohxResult_t : uint8_t
{
  OHX_R_NONE = 0,     // no command yet
  OHX_R_OK = 1,
  OHX_R_STALE = 2,    // nonce out of date (another command got in first) - resend with the new one
  OHX_R_AUTH = 3,     // signature wrong: the gauge has the wrong OpenHaldex WiFi password
  OHX_R_DISABLED = 4, // control over ESP-NOW is switched off on the OpenHaldex
  OHX_R_MOVING = 5,   // needs the car stopped
  OHX_R_NO_HALDEX = 6,// no Haldex CAN data
  OHX_R_BUSY = 7,     // a learn is already running
  OHX_R_INVALID = 8,  // bad argument / unknown command
  OHX_R_UNSUPPORTED = 9, // not available for this generation
  OHX_R_CTRL_OFF = 10,   // mode change while the controller is disabled
};

struct __attribute__((packed)) OhxHdr
{
  uint8_t magic[2];
  uint8_t type;  // ohxType_t
  uint8_t proto; // OHX_PROTO
};

struct __attribute__((packed)) OhxStatus
{
  OhxHdr h;
  uint32_t nonce;
  uint8_t seq;            // +1 per packet
  uint8_t fw[3];          // major, minor, patch
  uint8_t generation;     // haldexGeneration
  uint8_t mode;           // the mode in force (openhaldex_mode_t)
  uint8_t flags[4];       // OHX_F*
  uint8_t lockTarget;     // % requested
  uint8_t lockActual;     // % engagement reported by the Haldex, OHX_NONE8 without Haldex CAN
  uint8_t engagementRaw;  // raw engagement byte (as in 0x6B0 byte 2)
  uint8_t haldexState;    // raw Haldex status byte
  uint16_t speed;         // km/h, OHX_NONE16 without chassis CAN
  uint16_t rpm;
  uint16_t boost;         // mbar above ambient (clamped at 0)
  uint8_t throttle;       // %, OHX_NONE8
  uint8_t disableThrottle;      // setting: lock released under this pedal %
  uint16_t disengageUnderSpeed; // setting, km/h
  uint16_t disengageAboveSpeed; // setting, km/h
  uint16_t steering;      // |angle| deg, OHX_NONE16 when stale / not supported
  int8_t slip[4];         // FL FR RL RR %, -128 = none
  uint8_t learnStep;      // 0..100 learn sweep progress
  uint8_t longPhase;      // long learn phase (OpenHaldex LL_*)
  uint8_t longSweep;      // sweeps done
  uint8_t longTotal;      // sweeps expected
  int16_t clutchTemp;     // 0.1 C: Gen5 clutch (UDS) / Gen4 plate (KWP), OHX_NONE_T
  int16_t oilTemp;        // 0.1 C: Gen4 oil (KWP), OHX_NONE_T
  int16_t moduleTemp;     // 0.1 C: Gen5 module (UDS), OHX_NONE_T
  uint16_t supplyMv;      // terminal / supply voltage mV, OHX_NONE16
  uint8_t ackSrc[2];      // last command: the sender's MAC bytes 4, 5
  uint8_t ackSeq;         // its seq
  uint8_t ackResult;      // ohxResult_t
  uint8_t channel;        // the OpenHaldex's WiFi channel
  uint8_t spare[3];
  char name[24];          // its AP SSID
};

struct __attribute__((packed)) OhxCommand
{
  OhxHdr h;
  uint32_t nonce; // from the latest OhxStatus
  uint8_t seq;    // the sender's own counter (matched against ackSeq)
  uint8_t cmd;    // ohxCmd_t
  uint8_t arg[4];
  uint8_t hmac[8];
};

static_assert(sizeof(OhxStatus) <= 250, "OHX status must fit one ESP-NOW v1 packet");
static_assert(sizeof(OhxCommand) == 22, "OHX command layout");
