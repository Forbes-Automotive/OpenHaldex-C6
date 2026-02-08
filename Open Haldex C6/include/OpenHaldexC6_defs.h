#ifndef OPENHALDEXC6_DEFS_H
#define OPENHALDEXC6_DEFS_H

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

#include "Freenove_WS2812_Lib_for_ESP32.h"  // for RGB LED
#include <Preferences.h>                    // for eeprom/remember settings

#include <ESPUI.h>    // included for WiFi pages
#include <WiFi.h>     // included for WiFi pages
#include <ESPmDNS.h>  // included for WiFi pages

#include "InterruptButton.h"  // for mode button (internal & external)

// debug options
#define enableDebug 1
#define detailedDebug 1
#define detailedDebugStack 1
#define detailedDebugRuntimeStats 0
#define detailedDebugCAN 0
#define detailedDebugWiFi 1
#define detailedDebugEEP 0
#define detailedDebugIO 0

// refresh rates
#define eepRefresh 2000            // EEPROM save in ms
#define broadcastRefresh 200       // broadcast refresh rate in ms
#define serialMonitorRefresh 1000  // Serial Monitor refresh rate in ms
#define labelRefresh 500           // broadcast refresh rate in ms
#define updateTriggersRefresh 500

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
#define BYTE_TO_BINARY(byte) \
  ((byte)&0x80 ? '1' : '0'), \
    ((byte)&0x40 ? '1' : '0'), \
    ((byte)&0x20 ? '1' : '0'), \
    ((byte)&0x10 ? '1' : '0'), \
    ((byte)&0x08 ? '1' : '0'), \
    ((byte)&0x04 ? '1' : '0'), \
    ((byte)&0x02 ? '1' : '0'), \
    ((byte)&0x01 ? '1' : '0')

// GPIO
#define CAN0_RS 2   // can_0 slope control
#define CAN0_RX 23  // can_0 rx
#define CAN0_TX 3   // can_0 tx
#define CAN1_RS 22  // can_1 slope control
#define CAN1_RX 20  // can_1 tx
#define CAN1_TX 21  // can_1 rx

#define gpio_led 8        // gpio for led
#define gpio_mode 19      // gpio mode button internal
#define gpio_mode_ext 18  // gpio mode button external

#define gpio_hb_in 14     // gpio for handbrake signal in
#define gpio_hb_out 15    // gpio for handbrake signal out
#define gpio_brake_in 0   // gpio for brake signal in
#define gpio_brake_out 1  // gpio for brake signal out

// led settings
#define led_channel 0       // channel for led
#define led_brightness 255  // default brightness

// wifi settings
#define wifiHostName "OpenHaldex-C6"  // the WiFi name

extern twai_handle_t twai_bus_0;  // for ESP32 C6 CANBUS 0
extern twai_handle_t twai_bus_1;  // for ESP32 C6 CANBUS 1

extern twai_message_t rx_message_hdx;  // incoming haldex message
extern twai_message_t rx_message_chs;  // incoming chassis message

extern twai_message_t tx_message_hdx;  // outgoing haldex message
extern twai_message_t tx_message_chs;  // outgoing chassis message

extern TaskHandle_t handle_frames1000;  // for enabling/disabling 1000ms frames
extern TaskHandle_t handle_frames200;   // for enabling/disabling 200ms frames
extern TaskHandle_t handle_frames100;   // for enabling/disabling 100ms frames
extern TaskHandle_t handle_frames25;    // for enabling/disabling 25ms frames
extern TaskHandle_t handle_frames20;    // for enabling/disabling 20ms frames
extern TaskHandle_t handle_frames10;    // for enabling/disabling 10ms frames

// setup - main inputs
extern bool isMPH;       // 0 = kph, 1 = mph
#define mphFactor 621371  // to convert from kmh > mph

// for EEP
extern Preferences pref;  // for EEPROM / storing settings

// for LED
extern Freenove_ESP32_WS2812 strip;  // 1 led, gpio pin, channel, type of LED

// for mode changing (buttons & external inputs)
extern InterruptButton btnMode;          // pin, GPIO_MODE_INPUT, state when pressed, long press, autorepeat, double-click, debounce
extern InterruptButton btnMode_ext;  // pin, GPIO_MODE_INPUT, state when pressed, long press, autorepeat, double-click, debounce

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

// for WiFi Function Prototypes
extern void connectWifi();
extern void disconnectWifi();
extern void setupUI();
extern void generalCallback(Control *sender, int type);
extern void extendedCallback(Control *sender, int type, void *param);
extern void updateLabels(void *arg);
extern void setupAnalyzer();
extern void setAnalyzerMode(bool enable);
extern void analyzerQueueFrame(const twai_message_t &frame, uint8_t bus);

enum openhaldex_mode_t {
  MODE_STOCK,
  MODE_FWD,
  MODE_5050,
  MODE_6040,
  MODE_7525,
  MODE_CUSTOM,
  MODE_MAP,
  openhaldex_mode_t_MAX
};

struct lockpoint_t {
  uint8_t speed;
  uint8_t lock;
  uint8_t intensity;
};

#define CUSTOM_LOCK_POINTS_MAX_COUNT 10
struct openhaldex_custom_mode_t {
  lockpoint_t lockpoints[CUSTOM_LOCK_POINTS_MAX_COUNT];
  uint8_t lockpoint_bitfield_high_byte;
  uint8_t lockpoint_bitfield_low_byte;
  uint8_t lockpoint_count;
};

struct openhaldex_state_t {
  openhaldex_mode_t mode;
  openhaldex_custom_mode_t custom_mode;
  uint8_t pedal_threshold;
  bool mode_override;
};

// Values received from Haldex CAN
extern uint8_t received_haldex_state;
extern uint8_t received_haldex_engagement_raw;
extern uint8_t received_haldex_engagement;
extern uint8_t appliedTorque;

extern bool received_report_clutch1;
extern bool received_report_clutch2;
extern bool received_temp_protection;
extern bool received_coupling_open;
extern bool received_speed_limit;

// values received from Chassis CAN
extern float received_pedal_value;
extern uint16_t received_vehicle_speed;
extern uint16_t received_vehicle_rpm;
extern uint16_t received_vehicle_boost;
extern uint8_t haldexGeneration;

extern bool isStandalone;
extern bool isGen1Standalone;
extern bool isGen2Standalone;
extern bool isGen3Standalone;
extern bool isGen4Standalone;
extern bool isGen5Standalone;

extern bool isBusFailure;
extern bool hasCANChassis;
extern bool hasCANHaldex;
extern bool broadcastOpenHaldexOverCAN;
extern bool disableController;
extern bool followBrake;
extern bool invertBrake;
extern bool followHandbrake;
extern bool invertHandbrake;

extern bool rebootWiFi;

extern bool brakeActive;
extern bool brakeSignalActive;

extern bool handbrakeActive;
extern bool handbrakeSignalActive;

extern bool otaUpdate;
extern bool customSpeed;
extern bool customThrottle;
// Analyzer mode = passive bridge + external CAN analyzer interface.
extern bool analyzerMode;

// Analyzer protocol for the TCP bridge (GVRET for SavvyCAN, Lawicel/SLCAN for CANHacker).
#define ANALYZER_PROTOCOL_GVRET 0
#define ANALYZER_PROTOCOL_LAWICEL 1
extern uint8_t analyzerProtocol;

extern uint32_t alerts_to_enable;

extern long lastCANChassisTick;
extern long lastCANHaldexTick;

extern uint8_t lastMode;
extern uint8_t disableThrottle;
extern uint16_t disableSpeed;

extern uint32_t rxtxcount;  // frame counter
extern uint32_t stackCHS;
extern uint32_t stackHDX;

extern uint32_t stackframes10;
extern uint32_t stackframes20;
extern uint32_t stackframes25;
extern uint32_t stackframes100;
extern uint32_t stackframes200;
extern uint32_t stackframes1000;

extern uint32_t stackbroadcastOpenHaldex;
extern uint32_t stackupdateLabels;
extern uint32_t stackshowHaldexState;
extern uint32_t stackwriteEEP;

// internal variables
extern openhaldex_state_t state;
extern float lock_target;

extern uint16_t speedArray[5];
extern uint8_t throttleArray[5];
extern uint8_t lockArray[5];

// for running through vars to see effects
extern uint8_t tempCounter;
extern uint8_t tempCounter1;
extern uint16_t tempCounter2;

// checksum values (for calculating module checksums in standalone mode)
extern uint8_t MOTOR5_counter;
extern uint8_t MOTOR6_counter;
extern uint8_t MOTOR6_counter2;

extern uint8_t BRAKES1_counter;  // 0
extern uint8_t BRAKES2_counter;   // starting counter for Brakes2 is 3
extern uint8_t BRAKES4_counter;
extern uint8_t BRAKES4_counter2;
extern uint8_t BRAKES4_crc;

extern uint8_t BRAKES5_counter;
extern uint8_t BRAKES5_counter2;  // starting counter for Brake5 is 3

extern uint8_t BRAKES8_counter;  // starting counter for Brakes9 is 11

extern uint8_t BRAKES9_counter;  // starting counter for Brakes9 is 11
extern uint8_t BRAKES9_counter2;

extern uint8_t BRAKES10_counter;

extern uint8_t mLW_1_counter;  // was 0
extern uint8_t mLW_1_counter2;
extern uint8_t mLW_1_crc;

extern uint8_t mDiagnose_1_counter;

extern const uint8_t lws_2[16][8];

// WiFi UI handles
extern uint16_t int16_currentMode, label_currentLocking, int16_disableThrottle, int16_disableSpeed, int16_haldexGeneration, int16_customSelect;
extern uint16_t customSet_1, customSet_2, customSet_3, customSet_4, customSet_5;

extern uint16_t customSet_1_speed, customSet_1_throttle, customSet_1_lock;

extern uint16_t bool_broadcastHaldex, bool_disableControl, bool_followHandbrake, bool_followBrake, bool_invertBrake, bool_invertHandbrake, bool_isStandalone, bool_analyzerMode;

extern int label_hasChassisCAN, label_hasHaldexCAN, label_hasBusFailure, label_HaldexState, label_HaldexTemp, label_HaldexClutch1, label_HaldexClutch2, label_HaldexCoupling, label_HaldexSpeedLimit, label_currentSpeed, label_currentRPM, label_currentBoost, label_brakeIn, label_brakeOut, label_handbrakeIn, label_handbrakeOut, label_firmwareVersion, label_chipModel, label_freeHeap, label_otaStatus;

// Convert a value of type openhaldex_mode_t to a string.
inline const char *get_openhaldex_mode_string(openhaldex_mode_t mode) {
  switch (mode) {
    case MODE_STOCK:
      return "STOCK";
    case MODE_FWD:
      return "FWD";
    case MODE_5050:
      return "5050";
    case MODE_7525:
      return "7525";
    case MODE_6040:
      return "6040";
    case MODE_CUSTOM:
      return "CUSTOM";
    case MODE_MAP:
      return "MAP";
    default:
      break;
  }
  return "?";
}

#endif // OPENHALDEXC6_DEFS_H
