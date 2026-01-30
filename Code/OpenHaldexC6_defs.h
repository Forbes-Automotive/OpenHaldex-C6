#pragma once
// general ESP activities
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

#include "Freenove_WS2812_Lib_for_ESP32.h"  // for RGB LED
#include <Preferences.h>                    // for eeprom/remember settings

#include <ESPUI.h>    // included for WiFi pages
#include <WiFi.h>     // included for WiFi pages
#include <ESPmDNS.h>  // included for WiFi pages

#include "InterruptButton.h"  // for mode button (internal & external)

// debug options
#define enableDebug 0
#define detailedDebug 0
#define detailedDebugStack 0
#define detailedDebugRuntimeStats 0
#define detailedDebugCAN 0
#define detailedDebugWiFi 0
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

static twai_handle_t twai_bus_0;  // for ESP32 C6 CANBUS 0
static twai_handle_t twai_bus_1;  // for ESP32 C6 CANBUS 1

twai_message_t rx_message_hdx;  // incoming haldex message
twai_message_t rx_message_chs;  // incoming chassis message

twai_message_t tx_message_hdx;  // outgoing haldex message
twai_message_t tx_message_chs;  // outgoing chassis message

TaskHandle_t handle_frames1000;  // for enabling/disabling 1000ms frames
TaskHandle_t handle_frames200;   // for enabling/disabling 200ms frames
TaskHandle_t handle_frames100;   // for enabling/disabling 100ms frames
TaskHandle_t handle_frames25;    // for enabling/disabling 25ms frames
TaskHandle_t handle_frames20;    // for enabling/disabling 20ms frames
TaskHandle_t handle_frames10;    // for enabling/disabling 10ms frames

// setup - main inputs
bool isMPH = false;       // 0 = kph, 1 = mph
#define mphFactor 621371  // to convert from kmh > mph

// for EEP
Preferences pref;  // for EEPROM / storing settings

// for LED
Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(1, gpio_led, led_channel, TYPE_RGB);  // 1 led, gpio pin, channel, type of LED

// for mode changing (buttons & external inputs)
InterruptButton btnMode(gpio_mode, HIGH, GPIO_MODE_INPUT, 1000, 500, 750, 80000);          // pin, GPIO_MODE_INPUT, state when pressed, long press, autorepeat, double-click, debounce
InterruptButton btnMode_ext(gpio_mode_ext, HIGH, GPIO_MODE_INPUT, 1000, 500, 750, 80000);  // pin, GPIO_MODE_INPUT, state when pressed, long press, autorepeat, double-click, debounce

// functions
void frames10();
void frames20();
void frames25();
void frames100();
void frames200();
void frames1000();

void parseCAN_chs();
void parseCAN_hdx();
void broadcastOpenHaldex();
void showHaldexState();

void setupButtons();
void setupCAN();
void setupTasks();

void updateTriggers();
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
extern void updateLabels();
extern void setupAnalyzer();
extern void setAnalyzerMode(bool enable);
extern void analyzerQueueFrame(const twai_message_t &frame, uint8_t bus);

// Values received from Haldex CAN
uint8_t received_haldex_state = 0;
uint8_t received_haldex_engagement_raw = 0;
uint8_t received_haldex_engagement = 0;
uint8_t appliedTorque = 0;

bool received_report_clutch1;
bool received_report_clutch2;
bool received_temp_protection;
bool received_coupling_open;
bool received_speed_limit;

// values received from Chassis CAN
float received_pedal_value;
uint16_t received_vehicle_speed;
uint16_t received_vehicle_rpm;
uint16_t received_vehicle_boost;
uint8_t haldexGeneration;

bool isStandalone = false;
bool isGen1Standalone = false;
bool isGen2Standalone = false;
bool isGen3Standalone = false;
bool isGen4Standalone = false;
bool isGen5Standalone = false;

bool isBusFailure = false;
bool hasCANChassis = false;
bool hasCANHaldex = false;
bool broadcastOpenHaldexOverCAN = true;
bool disableController = false;
bool followBrake = false;
bool invertBrake = false;
bool followHandbrake = false;
bool invertHandbrake = false;

bool rebootWiFi = false;

bool brakeActive = false;
bool brakeSignalActive = false;

bool handbrakeActive = false;
bool handbrakeSignalActive = false;

bool otaUpdate = false;
bool customSpeed = true;
bool customThrottle = false;
// Analyzer mode = passive bridge + external CAN analyzer interface.
bool analyzerMode = false;

// Analyzer protocol for the TCP bridge (GVRET for SavvyCAN, Lawicel/SLCAN for CANHacker).
#define ANALYZER_PROTOCOL_GVRET 0
#define ANALYZER_PROTOCOL_LAWICEL 1
uint8_t analyzerProtocol = ANALYZER_PROTOCOL_GVRET;

uint32_t alerts_to_enable = 0;

long lastCANChassisTick;
long lastCANHaldexTick;

uint8_t lastMode = 0;
uint8_t disableThrottle = 0;
uint16_t disableSpeed = 0;

uint32_t rxtxcount = 0;  // frame counter
uint32_t stackCHS = 0;
uint32_t stackHDX = 0;

uint32_t stackframes10 = 0;
uint32_t stackframes20 = 0;
uint32_t stackframes25 = 0;
uint32_t stackframes100 = 0;
uint32_t stackframes200 = 0;
uint32_t stackframes1000 = 0;

uint32_t stackbroadcastOpenHaldex = 0;
uint32_t stackupdateLabels = 0;
uint32_t stackshowHaldexState = 0;
uint32_t stackwriteEEP = 0;

enum openhaldex_mode_t {
  MODE_STOCK,
  MODE_FWD,
  MODE_5050,
  MODE_6040,
  MODE_7525,
  MODE_CUSTOM,
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

// internal variables
openhaldex_state_t state;
float lock_target = 0;

// Convert a value of type openhaldex_mode_t to a string.
const char *get_openhaldex_mode_string(openhaldex_mode_t mode) {
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
    default:
      break;
  }
  return "?";
}

uint16_t speedArray[] = { 0, 0, 0, 0, 0 };
uint8_t throttleArray[] = { 0, 0, 0, 0, 0 };
uint8_t lockArray[] = { 0, 0, 0, 0, 0 };

// for running through vars to see effects
uint8_t tempCounter;
uint8_t tempCounter1;
uint16_t tempCounter2;

// checksum values (for calculating module checksums in standalone mode)
uint8_t MOTOR5_counter = 0;
uint8_t MOTOR6_counter = 254;
uint8_t MOTOR6_counter2 = 0;

uint8_t BRAKES1_counter = 10;  // 0
uint8_t BRAKES2_counter = 0;   // starting counter for Brakes2 is 3
uint8_t BRAKES4_counter = 0;
uint8_t BRAKES4_counter2 = 0x64;
uint8_t BRAKES4_crc = 0;

uint8_t BRAKES5_counter = 0x04;
uint8_t BRAKES5_counter2 = 3;  // starting counter for Brake5 is 3

uint8_t BRAKES8_counter = 0x00;  // starting counter for Brakes9 is 11

uint8_t BRAKES9_counter = 0x03;  // starting counter for Brakes9 is 11
uint8_t BRAKES9_counter2 = 0x00;

uint8_t BRAKES10_counter = 0;

uint8_t mLW_1_counter = 0;  // was 0
uint8_t mLW_1_counter2 = 0x77;
uint8_t mLW_1_crc = 0;

uint8_t mDiagnose_1_counter = 0;

/*const uint8_t lws_1[16][8] = {
  { 0x88, 0x00, 0x00, 0x00, 0x80, 0x00, 0xA3, 0x77 },
  { 0x88, 0x00, 0x00, 0x00, 0x80, 0x10, 0x86, 0x67 },
  { 0x88, 0x00, 0x00, 0x00, 0x80, 0x20, 0xE9, 0x57 },
  { 0x88, 0x00, 0x00, 0x00, 0x80, 0x30, 0xCC, 0x47 },
  { 0x88, 0x00, 0x00, 0x00, 0x80, 0x40, 0x37, 0x37 },
  { 0x88, 0x00, 0x00, 0x00, 0x80, 0x50, 0x12, 0x27 },
  { 0x88, 0x00, 0x00, 0x00, 0x80, 0x60, 0x7D, 0x17 },
  { 0x88, 0x00, 0x00, 0x00, 0x80, 0x70, 0x58, 0x07 },
  { 0x88, 0x00, 0x00, 0x00, 0x80, 0x80, 0xA4, 0xF7 },
  { 0x88, 0x00, 0x00, 0x00, 0x80, 0x90, 0x81, 0xE7 },
  { 0x88, 0x00, 0x00, 0x00, 0x80, 0xA0, 0xEE, 0xD7 },
  { 0x88, 0x00, 0x00, 0x00, 0x80, 0xB0, 0xCB, 0xC7 },
  { 0x88, 0x00, 0x00, 0x00, 0x80, 0xC0, 0x30, 0xB7 },
  { 0x88, 0x00, 0x00, 0x00, 0x80, 0xD0, 0x15, 0xA7 },
  { 0x88, 0x00, 0x00, 0x00, 0x80, 0xE0, 0x7A, 0x97 },
  { 0x88, 0x00, 0x00, 0x00, 0x80, 0xF0, 0x5F, 0x87 }
  // ... add as many as you have; 16 fills the LUT fully
};*/
const uint8_t lws_2[16][8] = {
  { 0x22, 0x00, 0x00, 0x00, 0x80, 0x00, 0xA0, 0xDD },
  { 0x22, 0x00, 0x00, 0x00, 0x80, 0x10, 0x85, 0xCD },
  { 0x22, 0x00, 0x00, 0x00, 0x80, 0x20, 0xEA, 0xBD },
  { 0x22, 0x00, 0x00, 0x00, 0x80, 0x30, 0xCF, 0xAD },
  { 0x22, 0x00, 0x00, 0x00, 0x80, 0x40, 0x34, 0x9D },
  { 0x22, 0x00, 0x00, 0x00, 0x80, 0x50, 0x11, 0x8D },
  { 0x22, 0x00, 0x00, 0x00, 0x80, 0x60, 0x7E, 0x7D },
  { 0x22, 0x00, 0x00, 0x00, 0x80, 0x70, 0x5B, 0x6D },
  { 0x22, 0x00, 0x00, 0x00, 0x80, 0x80, 0xA7, 0x5D },
  { 0x22, 0x00, 0x00, 0x00, 0x80, 0x90, 0x82, 0x4D },
  { 0x22, 0x00, 0x00, 0x00, 0x80, 0xA0, 0xED, 0x3D },
  { 0x22, 0x00, 0x00, 0x00, 0x80, 0xB0, 0xC8, 0x2D },
  { 0x22, 0x00, 0x00, 0x00, 0x80, 0xC0, 0x33, 0x1D },
  { 0x22, 0x00, 0x00, 0x00, 0x80, 0xD0, 0x16, 0x0D },
  { 0x22, 0x00, 0x00, 0x00, 0x80, 0xE0, 0x79, 0xFD },
  { 0x22, 0x00, 0x00, 0x00, 0x80, 0xF0, 0x5C, 0xED }
};

// WiFi UI handles
uint16_t int16_currentMode, label_currentLocking, int16_disableThrottle, int16_disableSpeed, int16_haldexGeneration, int16_customSelect;
uint16_t customSet_1, customSet_2, customSet_3, customSet_4, customSet_5;

uint16_t customSet_1_speed, customSet_1_throttle, customSet_1_lock;

uint16_t bool_broadcastHaldex, bool_disableControl, bool_followHandbrake, bool_followBrake, bool_invertBrake, bool_invertHandbrake, bool_isStandalone, bool_analyzerMode;

int label_hasChassisCAN, label_hasHaldexCAN, label_hasBusFailure, label_HaldexState, label_HaldexTemp, label_HaldexClutch1, label_HaldexClutch2, label_HaldexCoupling, label_HaldexSpeedLimit, label_currentSpeed, label_currentRPM, label_currentBoost, label_brakeIn, label_brakeOut, label_handbrakeIn, label_handbrakeOut, label_firmwareVersion, label_chipModel, label_freeHeap, label_otaStatus;;
