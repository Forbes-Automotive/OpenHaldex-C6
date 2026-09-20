#pragma once

// general ESP activities
#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "driver/gpio.h"

// for CAN IDs - to make code easier
#include <OpenHaldexC6_canID.h>
#include <OpenHaldexC6_ver.h>

#include "Freenove_WS2812_Lib_for_ESP32.h" // for RGB LED
#include <Preferences.h>                   // for eeprom/remember settings

#include <WiFi.h>    // included for WiFi pages
#include <ESPmDNS.h> // included for WiFi pages
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "esp_ota_ops.h"
#include <ArduinoJson.h>

#include <LittleFS.h>

#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_partition.h"
#include "esp_https_ota.h"
#include "esp_flash_partitions.h"
#include "esp_log.h"

#include "InterruptButton.h" // for mode button (internal & external)

// debug options
#define enableDebug 0               // set to 1 to enable debug messages over Serial; set to 0 to disable
#define detailedDebug 0             // set to 1 to enable more detailed debug messages (only recommended when debugging specific issues, as it can be very verbose)
#define detailedDebugStack 0        // set to 1 to print stack traces on errors (only useful if enableDebug is also 1)
#define detailedDebugRuntimeStats 0 // set to 1 to print FreeRTOS runtime stats periodically (only useful if enableDebug is also 1)
#define detailedDebugCAN 0          // set to 1 to enable detailed CAN debug messages (only recommended when debugging CAN-related issues, as it can be very verbose)
#define detailedDebugWiFi 0         // set to 1 to enable detailed WiFi debug messages (only recommended when debugging WiFi-related issues, as it can be very verbose)
#define detailedDebugEEP 0          // set to 1 to enable detailed EEPROM debug messages (only recommended when debugging EEPROM-related issues, as it can be very verbose)
#define detailedDebugIO 0           // set to 1 to enable detailed IO debug messages (only recommended when debugging IO-related issues, as it can be very verbose)
#define detailedDebugArray 0        // set to 1 to enable detailed debug messages for arrays (like throttle/speed/lock curves) - only recommended when debugging issues related to those, as it can be very verbose
#define debugCANSleep 0             // set to 1 to skip the 5-min idle/60-s count and sleep after ~2 s with no clients

// refresh rates
#define eepRefresh 2000           // EEPROM save in ms
#define broadcastRefresh 200      // broadcast data over CAN refresh rate in ms
#define serialMonitorRefresh 1000 // Serial Monitor refresh rate in ms
#define updateTriggersRefresh 500 // change IO refresh rate in ms

// debugging macros
#ifdef enableDebug
#define DEBUG(x, ...) Serial.printf(x "\n", ##__VA_ARGS__)
#define DEBUG_(x, ...) Serial.printf(x, ##__VA_ARGS__)
#else
#define DEBUG(x, ...)
#define DEBUG_(x, ...)
#endif

// helpers to format a number as a binary string for printf
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)       \
  ((byte) & 0x80 ? '1' : '0'),     \
      ((byte) & 0x40 ? '1' : '0'), \
      ((byte) & 0x20 ? '1' : '0'), \
      ((byte) & 0x10 ? '1' : '0'), \
      ((byte) & 0x08 ? '1' : '0'), \
      ((byte) & 0x04 ? '1' : '0'), \
      ((byte) & 0x02 ? '1' : '0'), \
      ((byte) & 0x01 ? '1' : '0')

// GPIO
#define CAN0_RS 2  // can_0 slope control
#define CAN0_RX 23 // can_0 rx
#define CAN0_TX 3  // can_0 tx
#define CAN1_RS 22 // can_1 slope control
#define CAN1_RX 20 // can_1 tx
#define CAN1_TX 21 // can_1 rx

#define gpio_led 8       // gpio for led
#define gpio_mode 19     // gpio mode button internal
#define gpio_mode_ext 18 // gpio mode button external

#define gpio_hb_in 14    // gpio for handbrake signal in
#define gpio_hb_out 15   // gpio for handbrake signal out
#define gpio_brake_in 0  // gpio for brake signal in
#define gpio_brake_out 1 // gpio for brake signal out

// Board revision ID. One GPIO, read once at boot by readBoardRev() before
// anything else is configured:
//   rev 1 (original TWAI PCB) : pin unconnected -> reads HIGH against the
//                               internal pull-up
//   rev 2 (CAN FD / SPI PCB)  : 4k7 pull-down to GND -> reads LOW (0.3 V
//                               against the ~45 k internal pull-up)
// GPIO4 was chosen because it is free on the current PCB, is not a strapping
// pin (an external pull-down on 8/9/15 would change boot mode) and is
// ADC-capable, so a later revision can move to a resistor-divider ladder read
// as voltage bands without changing the concept. The pull-up is released
// straight after the read so a 4k7 to ground does not sit across it (0.7 mA,
// which low-power mode would notice). Placeholder pin - move it when the PCB
// pin-out is final.
#define BOARD_ID_PIN 4
#define BOARD_REV_TWAI 1  // native TWAI transceivers on CAN0/CAN1 pins
#define BOARD_REV_CANFD 2 // SPI CAN FD controllers
extern uint8_t boardRev;
void readBoardRev();

// led settings
#define led_channel 0              // channel for led
#define led_brightness_default 255 // compile-time default
extern uint8_t ledBrightness;      // runtime LED brightness (0–255, persisted)

// wifi settings
#define wifiHostNameDefault "OpenHaldex-C6" // factory default AP SSID
#define wifiHostName wifiSsid              // legacy alias - all call sites now use runtime SSID
extern char wifiSsid[33];                  // runtime AP SSID (max 32 chars + NUL)

extern twai_handle_t twai_bus_0; // for ESP32-C6 CANBUS 0
extern twai_handle_t twai_bus_1; // for ESP32-C6 CANBUS 1

extern twai_message_t rx_message_hdx; // incoming haldex message
extern twai_message_t rx_message_chs; // incoming chassis message

extern twai_message_t tx_message_hdx; // outgoing haldex message
extern twai_message_t tx_message_chs; // outgoing chassis message

extern TaskHandle_t handle_frames1000;           // for enabling/disabling 1000ms frames
extern TaskHandle_t handle_frames250;            // for enabling/disabling 250ms frames
extern TaskHandle_t handle_frames200;            // for enabling/disabling 200ms frames
extern TaskHandle_t handle_frames100;            // for enabling/disabling 100ms frames
extern TaskHandle_t handle_frames50;             // for enabling/disabling 50ms frames
extern TaskHandle_t handle_frames25;             // for enabling/disabling 25ms frames
extern TaskHandle_t handle_frames20;             // for enabling/disabling 20ms frames
extern TaskHandle_t handle_frames13;             // for enabling/disabling 13ms frames
extern TaskHandle_t handle_frames10;             // for enabling/disabling 10ms frames
extern TaskHandle_t handle_gen41_dual_bus_rates; // dedicated Gen41 dual-bus cadence task
extern TaskHandle_t handle_broadcastOpenHaldex;  // OpenHaldex CAN broadcast task (suspended in aggressive sleep)
extern TaskHandle_t handle_showHaldexState;      // serial state logger (suspended in aggressive sleep)
extern TaskHandle_t handle_updateTriggers;       // notified by CAN_RX wake ISRs in aggressive sleep

// setup - main inputs
extern bool isMPH;       // 0 = kph, 1 = mph
#define mphFactor 621371 // to convert from kmh > mph

// for EEP
extern Preferences pref; // for EEPROM / storing settings

// for LED
extern Freenove_ESP32_WS2812 strip; // 1 led, gpio pin, channel, type of LED

// for mode changing (buttons & external inputs)
extern InterruptButton btnMode;     // pin, GPIO_MODE_INPUT, state when pressed, long press, autorepeat, double-click, debounce
extern InterruptButton btnMode_ext; // pin, GPIO_MODE_INPUT, state when pressed, long press, autorepeat, double-click, debounce

extern AsyncWebServer webServer;

// functions
void frames10(void *arg);
void frames20(void *arg);
void frames25(void *arg);
void frames100(void *arg);
void frames200(void *arg);
void frames1000(void *arg);

void parseCAN_chs(void *arg);
void parseCAN_hdx(void *arg);
void broadcastOpenHaldex(void *arg);
void showHaldexState(void *arg);

void setupButtons();
void setupCAN();
void canBusRecovery();
void setupTasks();

void updateTriggers(void *arg);
void modeChange();
void modeChangeExt();

extern void getLockData(twai_message_t &rx_message_chs);
extern uint8_t get_lock_target_adjusted_value(uint8_t value, bool invert);

// for EEP
extern void readEEP();
extern void writeEEP();

extern void connectWifi();
extern void disconnectWifi();
extern void setupAnalyzer();
extern void setAnalyzerMode(bool enable);
extern void setAnalyzerSerialMode(bool enable);
extern void analyzerQueueFrame(const twai_message_t &frame, uint8_t bus);

// const MODE_NAMES = ['Stock', 'FWD', '50:50', '60:40', '75:25', 'Expert'];
enum openhaldex_mode_t
{
  MODE_STOCK,
  MODE_FWD,
  MODE_5050,
  MODE_6040,
  MODE_7525,
  MODE_EXPERT,
  openhaldex_mode_t_MAX // for validation
};

struct openhaldex_state_t
{
  openhaldex_mode_t mode;
  uint8_t pedal_threshold;
  bool mode_override;
};

// Values received from Haldex CAN
extern uint8_t received_haldex_state;
extern uint8_t received_haldex_engagement_raw;
extern uint8_t received_haldex_engagement;
extern uint8_t appliedTorque;

// Gen41 Haldex feedback (decoded from FDCM-originated frames per GMW8762 PPEI).
// 0x1CC Secondary Axle Status (Bus1, DLC=3):
//   D0 status byte: bit 0x10 = secondary axle engagement active flag,
//                   bit 0x08 = FDCM operational/healthy (always set when alive),
//                   bits[1:0] = 2-bit Alive Rolling Counter
//   D1 = applied secondary axle torque feedback (Nm, 1 Nm/LSB) - tracks 0x1CE D1 request
//   D2 = clutch sub-state (1=idle, 3=arming, 4=ramping, 5=engaged, 6=holding, 7=ramping-down)
extern uint8_t received_sec_axle_status_raw;   // 0x1CC D0
extern uint8_t received_sec_axle_torque_nm;    // 0x1CC D1 - applied rear-axle torque (Nm)
extern uint8_t received_sec_axle_clutch_state; // 0x1CC D2 - clutch sub-state machine
extern bool received_sec_axle_active;          // 0x1CC D0 bit 0x10
extern bool received_sec_axle_fdcm_healthy;    // 0x1CC D0 bit 0x08
extern uint8_t received_sec_axle_arc;          // 0x1CC D0 bits[1:0]

// 0x1D1 Rear Axle Status (Bus1, DLC=4):
//   D0 = status flags (mostly 0 in healthy operation)
//   D1, D2 = rear-axle diagnostic metrics (vary with activity)
//   D3 = static framing byte (0x28 in all OEM captures)
extern uint8_t received_rear_axle_status_flags; // 0x1D1 D0
extern uint8_t received_rear_axle_metric_a;     // 0x1D1 D1
extern uint8_t received_rear_axle_metric_b;     // 0x1D1 D2

// 0x1CF / 0x331 Bus0 heartbeats (presence indicators only)
extern bool received_haldex_alive_bus0;        // 0x1CF (D0=0x03) seen recently
extern bool received_drivetrain_state_ok;      // 0x331 ([F8 FF]) seen recently
extern uint32_t received_haldex_alive_bus0_ms; // millis() of last 0x1CF
extern uint32_t received_drivetrain_state_ms;  // millis() of last 0x331

extern bool received_report_clutch1;
extern bool received_report_clutch2;
extern bool received_temp_protection;
extern bool received_coupling_open;
extern bool received_speed_limit;
extern bool received_limp_mode;
extern bool received_awd_warning;
extern uint8_t received_long_lock_state;
extern bool received_kickdown;

// values received from Chassis CAN
extern float received_pedal_value;
extern uint16_t received_vehicle_speed;
extern uint16_t received_vehicle_rpm;
extern uint16_t received_vehicle_boost;
extern uint8_t haldexGeneration;
// Steering-wheel angle magnitude (deg, abs) decoded from chassis CAN + last-seen time.
extern float received_steering_angle;
extern uint32_t received_steering_ms;
// Steering direction sign (true = negative/left, from the LWI_01/LW_1 sign bit) for slip geometry.
extern bool received_steering_negative;
// Per-corner raw wheel speeds [FL, FR, RL, RR] (ESP_19 units, 0.0075 km/h/bit) + freshness.
extern uint16_t wheelSpeedRaw[4];
extern uint32_t lastWheelSpeedResponse;
// Geometry-compensated per-corner slip [FL, FR, RL, RR] as signed % (-128 = no data) + freshness.
extern int8_t cornerSlip[4];
extern uint32_t lastCornerSlipMs;
// Per-car slip geometry (Audi TT Mk3 defaults; want on-car calibration).
// Slip geometry model adopted from OpenHaldex-Edge by Rekt (Kile Thomson).
extern float slipSteeringRatio;
extern uint16_t slipWheelbaseMm;
extern uint16_t slipTrackFrontMm;
extern uint16_t slipTrackRearMm;
extern uint16_t slipMinSpeedRaw;
extern uint8_t tcForceModeValue;
extern uint8_t hazardForceModeValue;
extern uint8_t extBtnForceModeValue;

// extern bools for global features and states
extern bool isStandalone;
extern bool useCANifAvailable;

extern bool isBusFailure;
extern bool hasCANChassis;
extern bool hasCANHaldex;
extern bool broadcastOpenHaldexOverCAN;
extern bool disableController;
extern bool followBrake;
extern bool invertBrake;
extern bool followHandbrake;
extern bool invertHandbrake;
extern bool tcForceMode;
extern bool asrForceModeFlag;
extern bool tcForceModeFlag;

extern bool paddleTipActive;
extern bool paddleTipUp;
extern bool paddleTipDown;
extern bool paddleTipBoth;

extern bool extBtnForceMode;
extern bool extButtonForceModeFlag;

extern bool disableOnboardButton;
extern bool disableExternalButton;

extern bool fixHunting; // Motor_11: false=V3 packing, true=BPK packing

// ---- Motor_11 BPK packing tunables ------------------------------------------
// Every field the BPK packer puts on the wire, exposed at runtime so the serial
// lab can massage them live instead of needing a reflash per experiment. All
// default to the values that were previously hardcoded, so an untouched build
// behaves exactly as before. None are persisted - they reset on boot.
extern uint16_t bpkFloorNm;  // 10   - Nm claimed at zero command
extern uint16_t bpkSlewIst;  // 8    - Nm/cycle ramp on MO_Mom_Ist_Summe
extern uint16_t bpkSlewSolf; // 32   - Nm/cycle ramp on MO_Mom_Soll_gefiltert
extern uint16_t bpkTraegRaw; // 509  - MO_Mom_Traegheit_Summe raw (509 = 0 Nm)
extern uint16_t bpkSchubRaw; // 487  - MO_Mom_Schub raw (487 = -22 Nm)
// Byte 7 status bits. 0x20 = MO_Status_Normalbetrieb_01 (bit 61) only. Per the
// MQB DBC, MO_QBit_Motormomente - the quality bit qualifying the whole Motor_11
// torque set - is bit 63 = 0x80, so the historical 0x20 has always sent these
// values as NOT qualified despite the old comment claiming otherwise.
extern uint8_t bpkStatusFl;  // 0x20
// Force Ist / Soll_gefiltert to a fixed Nm instead of the slewed target, to
// test whether the Haldex keys off one field rather than another. -1 = auto.
extern int32_t bpkForceIstNm;
extern int32_t bpkForceSolfNm;
// Last Motor_11 payload actually transmitted, for telemetry.
extern volatile uint8_t bpkLastFrame[8];

// Danger Zone: at a full 50:50 request, pin the ESP_14 coupling-range minimum
// to the maximum so the Haldex is given no room to modulate and drives the pump
// to full duty. Measured on the bench: ~99% PWM and ~10.5 A, versus ~56% PWM at
// the same request with this off. Maximum clamping force, but the Haldex's
// REPORTED engagement reads LOWER (80s rather than ~98%) because its estimate
// backs off once the pressure relief valve opens. Off by default; persisted.
extern bool dangerZoneEnabled;

// ---- Serial lab byte overrides ----------------------------------------------
// Force an individual byte of any generated standalone frame, so each byte's
// effect can be measured one at a time. Applied in standaloneTx() just before
// transmit; the frame's checksum is recomputed afterwards for the IDs that
// carry one, so an overridden frame is still accepted by the Haldex.
// Test-only: not persisted, cleared on boot.
#define LAB_OVR_MAX 12
struct LabOverride
{
  uint16_t canId;  // 0 = slot unused
  uint8_t byteIdx; // 0..7
  uint8_t value;
};
extern LabOverride labOverrides[LAB_OVR_MAX];

// ---- ESP_19 wheel-speed tunables --------------------------------------------
// Also runtime, to test whether the Haldex's hunting tracks the simulated wheel
// speed changing. Defaults reproduce the legacy free-running counter exactly.
extern bool wsFreeze;            // true = stop advancing the counters (static speed)
extern uint16_t wsBaseRaw;       // 0 = legacy counter behaviour; else fixed raw per corner
extern uint16_t wsDitherRaw;     // alternating +/- dither applied when wsBaseRaw > 0
extern int32_t wsFrontDeltaRaw;  // extra raw counts on the front axle (VL/VR); 0 = none
extern int32_t wsLeftRightDeltaRaw; // front left-vs-right split in raw counts (VL - VR); 0 = none. VAQ lever.

// Per-car Gen5 BPK lock calibration: Nm the spoof frame claims at full command.
// Not a strength dial - shifts the calibration so commanded lock matches delivered.
extern uint16_t bpkCeilingNm;
// ESP_14 launch PWM floor (0-100%): raises BR_Vorg_*_Min while lock is commanded,
// clamped strictly below Max. 0 = off (upstream behaviour).
// Adopted from OpenHaldex-Edge by Rekt (Kile Thomson) - see THIRD_PARTY_NOTICES.md.
extern uint8_t esp14MinFloorPct;

// ---- BPK serial lab (diagnostic harness) ------------------------------------
// Last values the Motor_11 BPK packer computed, captured each cycle by
// bpkLogSample() from both BPK code paths (standalone generation and
// normal-mode in-place editing). Read by the serial lab task, which streams
// them out over USB alongside what the Haldex is reporting back so a host
// script can correlate "what we sent" against "what came back" in real time.
// Zero when Fix Hunting is off (V3 packing computes no Nm values).
extern volatile uint16_t bpkLastTorqueNm; // raw feedforward target (Nm, unslewed)
extern volatile uint16_t bpkLastIstNm;    // slewed MO_Mom_Ist value sent (Nm)
extern volatile uint16_t bpkLastSolfNm;   // slewed MO_Mom_Soll_gefiltert sent (Nm)

// ---- Frame-edit gating (per-CAN-ID passthrough toggles) --------------------
// In normal (non-standalone) mode getLockData() overwrites specific bytes of
// selected passthrough frames. Each editable frame ("block") can be enabled or
// disabled at runtime so users can bisect which edited frame upsets the learn
// procedure. A disabled block leaves the car's real frame untouched (clean
// passthrough). Only gens 1/2/4/50/51 are gated (gen41/42 are dual-bus and are
// not handled in normal mode).
enum
{
    FE_GEN_1 = 0,
    FE_GEN_2,
    FE_GEN_4,
    FE_GEN_50,
    FE_GEN_51,
    FE_GEN_COUNT
};

struct FrameEditBlock
{
    uint8_t genIdx;   // FE_GEN_*
    uint8_t bit;      // bit position within that generation's mask
    uint16_t canId;   // CAN identifier of the frame
    const char *name; // human-readable frame name (shown in UI)
};

// Two independent masks: the passthrough (normal-mode) mask only enables the
// historically-edited frames by default, while the standalone mask enables ALL
// frames by default - in standalone the module synthesises the whole bus, so
// every editable frame must be generated. frameEditEnabled()/activeFrameEditMask()
// select the correct mask for the current mode via isStandalone.
extern uint64_t frameEditMask[FE_GEN_COUNT];                 // passthrough (normal-mode) enable bits per generation
extern uint64_t frameEditMaskSA[FE_GEN_COUNT];               // standalone enable bits per generation
extern const uint64_t frameEditMaskDefaults[FE_GEN_COUNT];   // normal-mode defaults (historically-edited frames on)
extern const uint64_t frameEditMaskDefaultsSA[FE_GEN_COUNT]; // standalone defaults (all frames on)
extern const FrameEditBlock frameEditBlocks[];            // descriptor table for UI/API
extern const uint16_t frameEditBlockCount;                // number of entries in frameEditBlocks[]
int frameEditGenIdx(uint8_t generation);                  // haldexGeneration -> FE_GEN_* (-1 if not gated)
bool frameEditEnabled(uint8_t genIdx, uint8_t bit);       // test a block's enable bit for the active mode
uint64_t *activeFrameEditMask();                          // mask for the current mode (isStandalone ? SA : normal)
void resetFrameEditMask();                                // restore defaults for both masks / all generations

extern bool canSleepEnabled;    // runtime toggle for CAN-wake light sleep (UI/EEP)
extern bool canSleepAggressive; // aggressive add-on: transceiver standby + DFS 10MHz + low WiFi TX power
extern volatile bool canWakeRequest; // set by CAN_RX GPIO ISR when transceivers in standby see bus activity

extern bool rebootWiFi;
extern bool lowPowerMode;
extern char wifiPassword[65]; // WiFi AP password - empty = open network

extern bool hazardForceMode;     // setting: use hazard lights to activate force mode
extern bool hazardForceModeFlag; // runtime: hazard lights are currently on

extern bool brakeActive;
extern bool brakeSignalActive;

extern bool handbrakeActive;
extern bool handbrakeSignalActive;

extern bool brakeFromCAN;
extern bool handbrakeFromCAN;

extern bool otaUpdate;

// Analyzer mode = passive bridge + external CAN analyzer interface.
extern bool analyzerMode;   // WiFi GVRET/SLCAN (TCP port 23)
extern bool analyzerSerial; // Serial GVRET (1 Mbaud, SavvyCAN serial connection)

#define lowPowerWatchMs 300000UL // 5 min with no WiFi clients → sleep

// --- CAN-wake light sleep --------------------------------------------------
// When `canSleepEnabled` is set, after WiFi has been shut down AND both CAN
// buses have been silent for `lowPowerDeepIdleMs`, the SoC enters light sleep.
// Wake source = falling edge on either CAN_RX pin (SOF of any frame).
#define lowPowerDeepIdleMs 10000UL // CAN-silent dwell before light sleep
#define lowPowerProbeMs    5000UL  // probe window length (ms)

// Aggressive CAN sleep: while LP_SLEEPING + aggressive, transceivers are
// parked in standby (~10mA saved). Wake is purely interrupt-driven: a GPIO
// ISR on each CAN_RX pin fires on the first falling edge (TCAN1044 pulls
// RXD low on any bus activity, even in standby) and immediately brings the
// transceivers back to normal mode. There is no periodic probe.
// Wake threshold is configurable at runtime via lpWakeThresholdFps (UI slider, stored in EEP).
// Default 1100 fps on chassis bus; Standalone uses a hardcoded 50 fps threshold.
// Frames required = lpWakeThresholdFps * lowPowerProbeMs / 1000
extern uint16_t lpWakeThresholdFps; // runtime wake threshold (fps), adjustable via UI

// Analyzer protocol for the TCP bridge (GVRET for SavvyCAN, Lawicel/SLCAN for CANHacker).
#define ANALYZER_PROTOCOL_GVRET 0
#define ANALYZER_PROTOCOL_LAWICEL 1
extern uint8_t analyzerProtocol;

// UDS MQB diagnostic polling (Gen 5 family only)
// Requests go to 0x70F on Bus 1; responses come from 0x779 on Bus 1.
// The whole Gen5 family - 0CQ MQB (50), 0AY PQ-derived (51) and VAQ (52) -
// shares the same UDS stack and DIDs, so every UDS gate uses this predicate.
inline bool isGen5Family() { return haldexGeneration == 50 || haldexGeneration == 51 || haldexGeneration == 52; }
extern bool liveDiagEnabled;     // master live-diagnostics enable (persisted, default off);
                                 // gates UDS (Gen5) and TP2.0 (Gen2/4) so it never blocks real tools unless opted in
// Auto-pause: while liveDiagEnabled, if an external scanner (VCDS/ODIS) is seen
// addressing the Haldex from the chassis side (Bus 0), our polling backs off and
// resumes once the tool goes quiet. externalDiagLastMs is the last time such a
// request frame was observed; the persisted liveDiagEnabled setting is untouched.
#define EXTERNAL_DIAG_TIMEOUT_MS 4000u
extern volatile uint32_t externalDiagLastMs;
bool externalDiagActive();
extern QueueHandle_t udsRxQueue; // parseCAN_hdx pushes 0x779 frames here
// True while udsMQBTask owns the Haldex diagnostic channel (probe/session open
// through to loop exit). parseCAN_hdx uses it to keep the Haldex's 0x779 replies
// to OUR requests off Bus 0 - the car never asked, so the gateway/OBD side
// should not see them. Replies are still forwarded whenever an external tool is
// active, so VCDS / gauges keep working.
extern volatile bool udsPollActive;
// Session the poller is currently using: 0 = none, 0x01 = default (plain RDBI,
// no TesterPresent), 0x03 = extended (fallback when the module refuses RDBI in
// default session). Reported on the API for diagnosis.
extern volatile uint8_t udsSessionMode;

// /api/uds/read helper: the parse task for the selected bus COPIES frames
// matching udsWebRespId into udsWebRxQueue (the frame still flows down the
// normal gateway path). The endpoint sets udsWebRespId/udsWebBus for the
// duration of one read, then clears udsWebRespId. Never read the TWAI driver
// directly from the web task - it would steal bridge frames.
extern QueueHandle_t udsWebRxQueue;
extern volatile uint32_t udsWebRespId; // 0 = no read in flight
extern volatile uint8_t udsWebBus;     // 0 = chassis (Bus 0), 1 = Haldex (Bus 1)

// ---- Haldex-bus diagnostic addressing ---------------------------------------
// The Gen5 family answers UDS on different ISO-TP pairs: Haldex/Allrad on
// 0x70F -> 0x779, the VAQ/Quersperre on 0x71E -> 0x788 (MQB FCAN K-matrix). The
// live-diag poller, the parse-task tap and the serial lab all address the module
// through these two so one switch moves everything. udsApplyDefaultIds() picks
// the pair from haldexGeneration unless the serial lab has pinned one manually
// (DIAGID) - the bench is where "which pair does this unit really use" gets
// answered, so it must be changeable without a reflash.
extern uint32_t udsHaldexReqId;  // UDS physical request ID on Bus 1
extern uint32_t udsHaldexRespId; // matching response ID on Bus 1
extern bool udsIdsManual;        // true = pinned by the serial lab, auto-pick disabled
void udsApplyDefaultIds();       // set the pair from haldexGeneration (no-op when pinned)

// ---- Haldex-bus RX census (serial lab RXIDS) --------------------------------
// Every frame received on Bus 1 is tallied per CAN ID: count, last payload, and
// the interval between the last two. In standalone the only other node on Bus 1
// is the module under test, so this table IS the list of what the unit transmits
// (feedback frames, network management, diagnostic replies) - the first thing to
// establish on an unknown unit like the VAQ. Small and fixed so it costs nothing.
#define HDX_RX_STATS_MAX 16
struct HdxRxStat
{
  uint32_t id;       // 0 = slot unused
  uint32_t count;
  uint32_t lastMs;
  uint16_t periodMs; // interval between the last two frames of this ID
  uint8_t dlc;
  uint8_t extd;
  uint8_t data[8];
};
extern HdxRxStat hdxRxStats[HDX_RX_STATS_MAX];
extern volatile uint32_t hdxRxDroppedIds; // frames whose ID found no free slot

// Last engagement/feedback frame the per-generation decoder accepted (0x118
// Allrad_03, 0x137 Quersperre_03, 0x2C0 Allrad_1, ...), raw, for the serial
// lab's FB telemetry line: what the module said, next to what we sent.
extern volatile uint32_t hdxFbMs;
extern volatile uint32_t hdxFbId;
extern volatile uint8_t hdxFbDlc;
extern volatile uint8_t hdxFbData[8];
// VAQ Quersperre_03 (0x137) state fields, K-matrix names:
//   QUER_Sta_Quersperre b1[4..6]: 0 rule mode, 1 driver-activated, 2 error open,
//                                 3 error closed, 4 temporary shutdown, 5 comms disturbed
//   QUER_Gleichlauf     b1[7]   : 0 no synchronism, 1 synchronised
extern uint8_t received_quer_state;
extern uint8_t received_quer_sync;


// Transmit-failure counters maintained by canTransmit() (see OpenHaldexC6_can.h).
extern volatile uint32_t canTxDropBus0;
extern volatile uint32_t canTxDropBus1;
extern float udsTerminalVoltage; // 0x0286: raw × 0.1 V
extern float udsModuleTemp;      // 0x028D: raw − 55 °C  (1 byte, offset 55)
extern float udsClutchTemp;      // 0x2BF1: LE16 (D6×256+D5 − 22767)/100 °C
extern float udsCoolingFinTemp;  // 0x2BE4: LE16 (D6×256+D5 − 22767)/100 °C
extern float udsClutchCurrent;   // 0x2BE6: BE16 × 0.001 A
extern uint8_t udsClutchPWM;     // 0x2BE7: raw % (1 byte, 0–100)
extern float udsClutchVoltage;   // 0x2BE9: BE16 × 0.001 V
extern uint8_t udsBlockagePct;   // unconfirmed DID — always 0

// --- KWP2000 over VW TP2.0 diagnostics: Gen2 / Gen4 (PQ) Haldex ------------
// The PQ-platform AWD/Haldex controller is diagnosed with KWP2000 tunnelled
// over VW TP2.0 (not raw UDS/ISO-TP). Channel setup is broadcast to 0x200 with
// the target logical address in byte0; the module answers on 0x200 + address.
// Confirmed from a SavvyCAN/VCDS capture of a 1K0 Gen2 Haldex: the module
// enumerates at logical address 0x0A and answers on 0x20A. (Address 0x22 /
// "22-AWD" carried over from the S3 sources did NOT respond on this platform —
// that reference was misleading.) If your module enumerates elsewhere, change
// KWP_TP20_HALDEX_ADDR (confirm from a VCDS / SavvyCAN capture) and rebuild —
// every derived ID below tracks it automatically.
#define KWP_TP20_HALDEX_ADDR   0x0Au                             // logical (diagnostic) address (confirmed 1K0 Gen2)
#define KWP_TP20_SETUP_TX_ID   0x200u                             // tester -> broadcast channel setup
#define KWP_TP20_SETUP_RX_ID   (0x200u + KWP_TP20_HALDEX_ADDR)    // ECU setup response (0x20A)
#define KWP_TP20_TESTER_RX_ID  0x300u                             // ID we ask the ECU to transmit data on

// TP2.0 polling shares the master liveDiagEnabled flag (declared above); it only
// acts when haldexGeneration == 2 || 4.
extern QueueHandle_t tp20RxQueue;        // parseCAN_hdx pushes TP2.0 diag frames here
extern volatile uint32_t kwpTp20EcuTxId; // negotiated ECU->tester data CAN id (0 until channel open)
extern bool kwpTp20Connected;            // TP2.0 channel + KWP session currently up
extern char kwpTp20RawDump[512];         // raw measuring-block capture (formula-id + a/b, no scaling yet)

// Gen4 (0AY) scaled measuring values, decoded via VAG formulas confirmed from
// VCDS captures. Group 0x01 = oil/plate temp + supply voltage; group 0x03 =
// oil pressure, estimated torque, clutch valve duty and current.
extern float kwpOilTemp;         // G01[0] formula 0x1A: b - a  (°C)
extern float kwpPlateTemp;       // G01[1] formula 0x1A: b - a  (°C, heated in capture)
extern float kwpSupplyVoltage;   // G01[2] formula 0x06: 0.001*a*b  (V)
extern float kwpOilPressure;     // G03[0] formula 0x0E: 0.01*a*(b-100)  (bar)
extern float kwpEstTorque;       // G03[1] formula 0x5E: 0.1*a*(b-128)  (Nm)
extern float kwpClutchDuty;      // G03[2] formula 0x21: 0.01*a*b  (%)
extern float kwpClutchValveCurrent; // G03[3] formula 0x18: 0.001*a*b  (A)


extern uint32_t alerts_to_enable;

extern long lastCANChassisTick;
extern long lastCANHaldexTick;
extern volatile uint32_t lpChassisFrameCount; // chassis CAN frame counter for probe window activity check
extern volatile uint32_t lpHaldexFrameCount;  // haldex CAN frame counter for standalone probe window activity check
extern uint32_t canHealthTimeoutMs;

extern uint8_t lastMode;
extern uint8_t disableThrottle;
extern uint16_t disengageUnderSpeed;
extern uint16_t disengageAboveSpeed;

extern uint32_t rxtxcount; // frame counter
extern uint32_t stackCHS;
extern uint32_t stackHDX;

extern uint32_t stackframes10;
extern uint32_t stackframes13;
extern uint32_t stackframes20;
extern uint32_t stackframes25;
extern uint32_t stackframes50;
extern uint32_t stackframes100;
extern uint32_t stackframes200;
extern uint32_t stackframes250;
extern uint32_t stackframes1000;

extern uint32_t stackbroadcastOpenHaldex;
extern uint32_t stackupdateLabels;
extern uint32_t stackshowHaldexState;
extern uint32_t stackwriteEEP;

// internal variables
extern openhaldex_state_t state;
extern float lock_target;

// Settings
extern float lockReleaseRatePerSec;
extern bool lockReleaseEnabled;  // when false, lock target changes are instantaneous
extern bool steeringScaleEnabled; // when false, steering-angle lock scaling is bypassed
extern uint8_t forceModesPriority; // 0=Haz>TC>Ext, 1=TC>Haz>Ext, 2=Haz>Ext>TC, 3=TC>Ext>Haz, 4=Ext>TC>Haz, 5=Ext>Haz>TC
extern uint32_t lastABSResponse;
extern bool isABSValid;
extern uint32_t absTimeout;

// Expert mode tables
#define speedArrayCount 7    // 0, 30, 60, 90, 120, 160, 180
#define throttleArrayCount 7 // 0, 15, 30, 45, 60, 75, 90

extern uint16_t speedArray[speedArrayCount];
extern uint8_t throttleArray[throttleArrayCount];
extern uint8_t lockArray[throttleArrayCount][speedArrayCount];

// Steering-angle third axis (FWD-bias): 1D scaling curve applied to the
// computed lock target. steeringArray holds steering-wheel angle breakpoints
// (deg, magnitude), steeringLockScaleArray the matching 0-100% lock multiplier.
// Only gens with a steering source (2/4/50/52) use it; others = no reduction.
#define steeringArrayCount 5   // 0, 45, 90, 180, 360 deg
#define steeringStaleMs 500    // steering considered stale/unhealthy after this (ms)
extern uint16_t steeringArray[steeringArrayCount];
extern uint8_t steeringLockScaleArray[steeringArrayCount];

// for running through vars to see effects
extern uint8_t tempCounter;
extern uint8_t tempCounter1;
extern uint16_t tempCounter2;

// checksum values (for calculating module checksums in standalone mode)
extern uint8_t MOTOR5_counter; // starting counter for Motor5 is 0
extern uint8_t MOTOR6_counter; // starting counter for Motor6 is 254
extern uint8_t MOTOR6_counter2;

extern uint8_t BRAKES1_counter;  // starting counter for Brakes1 is 10
extern uint8_t BRAKES2_counter;  // starting counter for Brakes2 is 3
extern uint8_t BRAKES4_counter;  // starting counter for Brakes4 is 0
extern uint8_t BRAKES4_counter2; // starting counter for Brakes4 is 0
extern uint8_t BRAKES4_crc;      // crc for Brakes4

extern uint8_t BRAKES5_counter;  // starting counter for Brakes5 is 0
extern uint8_t BRAKES5_counter2; // starting counter for Brake5 is 3

extern uint8_t BRAKES8_counter;  // Bremse_8 high-nibble rolling counter (0x80..0x8F)
extern uint8_t BRAKES8_counter1; // Bremse_8 low-nibble rolling counter (0x00..0x0F)

extern uint8_t BRAKES9_counter;  // starting counter for Brakes9 is 11
extern uint8_t BRAKES9_counter2; // starting counter for Brakes9 is 11

extern uint8_t BRAKES10_counter; // starting counter for Brakes10 is 0

extern uint8_t mLW_1_counter;  // was 0
extern uint8_t mLW_1_counter2; // was 0
extern uint8_t mLW_1_crc;      // crc for mLW_1

extern uint8_t mDiagnose_1_counter; // starting counter for mDiagnose_1 is 0

// PQ ACAN Gen5 0AY synthesized broadcasts
extern uint8_t mGetriebe_2_counter; // 4-bit rolling counter (Zaehler_Getriebe_2)

extern const uint8_t lws_2[16][8]; // lookup table for calculating lws_2 crc

extern const uint8_t GETRIEBE_1_array[16][8]; // lookup table for calculating lws_2 crc
extern uint8_t GETRIEBE_11_counter;           // starting counter for mDiagnose_1 is 0

extern uint8_t MOTOR_11_counter; // starting counter for Motor_11 is 0
extern uint8_t MOTOR_12_counter; // starting counter for Motor_12 is 0
extern uint8_t ESP_02_counter;   // starts at zero
extern uint8_t ESP_02_crc;       // starts at zero
extern uint8_t LWI_01_counter;
extern uint8_t ESP_14_counter;
extern uint8_t MOTOR_20_counter;
extern uint8_t ESP_10_counter;
extern uint8_t ESP_05_counter;
extern uint8_t EPB_01_counter;
extern uint8_t ESP_23_counter;
extern uint8_t ESP_21_counter;
extern uint8_t ESP_07_counter;
extern uint8_t MOTOR_CODE_01_counter;
extern uint8_t ESP_20_counter;
extern uint8_t MOTOR_14_counter;
extern uint8_t ESP_19_counter;
extern uint8_t ESP_19_counter2;

extern uint8_t Gen41_1CE234_counter;

// Gen42 (Ford) standalone rolling counters
extern uint8_t gen42_main_counter;   // 8-bit: shared by 0x080 D7, 0x20F D8, 0x211 D8
extern uint8_t gen42_090_nibble;     // 4-bit (0-15): 0x090 D1 high nibble
extern uint8_t gen42_20F_d6;         // 0x20F D6 (+0x10 per frame, mod 256)
extern uint8_t gen42_190_counter;    // 4-bit (0-15): 0x190 D6
extern uint8_t gen42_275_counter;    // 0x275 D1 (+0x20 per 100 ms, mod 256)

// Secondary Axle Torque Request to encode into 0x1CE (Nm, 0 = no request).
// 4 Nm/LSB empirical scaling; set via API or mode handler.
extern float g_1ce_torque_request_nm;

// gen5 checksums
extern const uint8_t ID_SEQ_0A8[16];
extern const uint8_t ID_SEQ_0AD[16];
extern const uint8_t ID_SEQ_0A7[16];
extern const uint8_t ID_SEQ_08A[16];
extern const uint8_t ID_SEQ_086[16];
extern const uint8_t ID_SEQ_121[16];
extern const uint8_t ID_SEQ_110[16];
extern const uint8_t ID_SEQ_106[16];
extern const uint8_t ID_SEQ_104[16];
extern const uint8_t ID_SEQ_116[16];
extern const uint8_t ID_SEQ_101[16];
extern const uint8_t ID_SEQ_0fd[16];
extern const uint8_t ID_SEQ_5be[16];
extern const uint8_t ID_SEQ_3be[16];
extern const uint8_t ID_SEQ_641[16];
extern const uint8_t ID_SEQ_645[16];
extern const uint8_t ID_SEQ_65d[16];
extern const uint8_t ID_SEQ_392[16];

// sum calculation functions
extern uint8_t crc8_autosar(uint8_t *data, uint8_t len);
extern uint8_t calcChecksum(uint8_t *frame, const uint8_t *idSeq);

// Convert a value of type openhaldex_mode_t to a string.
// const MODE_NAMES = ['Stock', 'FWD', '50:50', '60:40', '75:25', 'Expert'];
inline const char *get_openhaldex_mode_string(openhaldex_mode_t mode)
{
  switch (mode)
  {
  case MODE_STOCK:
    return "STOCK";
  case MODE_FWD:
    return "FWD";
  case MODE_5050:
    return "5050";
  case MODE_6040:
    return "6040";
  case MODE_7525:
    return "7525";
  case MODE_EXPERT:
    return "EXPERT";
  default:
    return "UNKNOWN";
    break;
  }
}