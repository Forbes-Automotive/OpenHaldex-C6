#include <OpenHaldexC6_defs.h>

// CAN bus handles
twai_handle_t twai_bus_0;  // for ESP32 C6 CANBUS 0
twai_handle_t twai_bus_1;  // for ESP32 C6 CANBUS 1

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

// for EEP
Preferences pref;  // for EEPROM / storing settings

// for LED - will be initialized in setupIO()
Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(1, gpio_led, led_channel, TYPE_RGB);  // 1 led, gpio pin, channel, type of LED

// for mode changing (buttons & external inputs) - will be initialized in setupButtons()
InterruptButton btnMode(gpio_mode, HIGH, GPIO_MODE_INPUT, 1000, 500, 750, 80000);          // pin, GPIO_MODE_INPUT, state when pressed, long press, autorepeat, double-click, debounce
InterruptButton btnMode_ext(gpio_mode_ext, HIGH, GPIO_MODE_INPUT, 1000, 500, 750, 80000);  // pin, GPIO_MODE_INPUT, state when pressed, long press, autorepeat, double-click, debounce

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

// internal variables
openhaldex_state_t state;
float lock_target = 0;

uint16_t speedArray[5] = { 0, 0, 0, 0, 0 };
uint8_t throttleArray[5] = { 0, 0, 0, 0, 0 };
uint8_t lockArray[5] = { 0, 0, 0, 0, 0 };

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

int label_hasChassisCAN, label_hasHaldexCAN, label_hasBusFailure, label_HaldexState, label_HaldexTemp, label_HaldexClutch1, label_HaldexClutch2, label_HaldexCoupling, label_HaldexSpeedLimit, label_currentSpeed, label_currentRPM, label_currentBoost, label_brakeIn, label_brakeOut, label_handbrakeIn, label_handbrakeOut, label_firmwareVersion, label_chipModel, label_freeHeap, label_otaStatus;
