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

// UDS MQB diagnostic polling (Gen 5 only)
// Requests go to 0x771 on Bus 1; responses come from 0x779 on Bus 1.
extern bool udsMQBEnabled;       // enable flag (persisted)
extern QueueHandle_t udsRxQueue; // parseCAN_hdx pushes 0x779 frames here
extern float udsTerminalVoltage; // 0x0286: raw × 0.1 V
extern float udsModuleTemp;      // 0x028D: raw − 55 °C  (1 byte, offset 55)
extern float udsClutchTemp;      // 0x2BF1: LE16 (D6×256+D5 − 22767)/100 °C
extern float udsCoolingFinTemp;  // 0x2BE4: LE16 (D6×256+D5 − 22767)/100 °C
extern float udsClutchCurrent;   // 0x2BE6: BE16 × 0.001 A
extern uint8_t udsClutchPWM;     // 0x2BE7: raw % (1 byte, 0–100)
extern float udsClutchVoltage;   // 0x2BE9: BE16 × 0.001 V
extern uint8_t udsBlockagePct;   // unconfirmed DID — always 0

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