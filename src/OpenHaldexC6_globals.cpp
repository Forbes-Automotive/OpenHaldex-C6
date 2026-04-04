#include <OpenHaldexC6_defs.h>

// TWAI handles
twai_handle_t twai_bus_0; // for ESP32 C6 CANBUS 0
twai_handle_t twai_bus_1; // for ESP32 C6 CANBUS 1

twai_message_t rx_message_hdx; // incoming haldex message
twai_message_t rx_message_chs; // incoming chassis message

twai_message_t tx_message_hdx; // outgoing haldex message
twai_message_t tx_message_chs; // outgoing chassis message

TaskHandle_t handle_frames1000; // for enabling/disabling 1000ms frames
TaskHandle_t handle_frames200;  // for enabling/disabling 200ms frames
TaskHandle_t handle_frames100;  // for enabling/disabling 100ms frames
TaskHandle_t handle_frames25;   // for enabling/disabling 25ms frames
TaskHandle_t handle_frames20;   // for enabling/disabling 20ms frames
TaskHandle_t handle_frames10;   // for enabling/disabling 10ms frames

// for EEP
Preferences pref; // for EEPROM / storing settings

// for LED - will be initialized in setupIO()
Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(1, gpio_led, led_channel, TYPE_RGB); // 1 led, gpio pin, channel, type of LED

// for mode changing (buttons & external inputs) - will be initialized in setupButtons()
InterruptButton btnMode(gpio_mode, HIGH, GPIO_MODE_INPUT, 1000, 500, 750, 80000);         // pin, GPIO_MODE_INPUT, state when pressed, long press, autorepeat, double-click, debounce
InterruptButton btnMode_ext(gpio_mode_ext, HIGH, GPIO_MODE_INPUT, 1000, 500, 750, 80000); // pin, GPIO_MODE_INPUT, state when pressed, long press, autorepeat, double-click, debounce

// for webServer
AsyncWebServer webServer(80);

// Values received from Haldex CAN
uint8_t received_haldex_state = 0;
uint8_t received_haldex_engagement_raw = 0;
uint8_t received_haldex_engagement = 0;
uint8_t appliedTorque = 0;

// values received from Haldex CAN
bool received_report_clutch1;
bool received_report_clutch2;
bool received_temp_protection;
bool received_coupling_open;
bool received_speed_limit;
bool received_kickdown = false;

// values received from Chassis CAN
float received_pedal_value = 0;
uint16_t received_vehicle_speed = 0;
uint16_t received_vehicle_rpm;
uint16_t received_vehicle_boost;
uint8_t haldexGeneration;
uint8_t forceModeValue;

uint32_t lastABSResponse = 0; // for tracking ABS responses to determine if ABS is valid
bool isABSValid = false;      // set to true if valid ABS response received within timeout, otherwise false
uint32_t absTimeout = 1000;    // timeout in ms for determining if ABS is valid - if no valid ABS response received within this time, isABSValid will be set to false

bool isStandalone = false;

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

bool tcForceMode = false;
bool tcForceModeFlag = false;
bool asrForceModeFlag = false;

bool paddleTipActive = false;
bool paddleTipUp     = false;
bool paddleTipDown   = false;
bool paddleTipBoth   = false;

bool extBtnForceMode = false;
bool extButtonForceModeFlag = false;

bool disableOnboardButton = false;
bool disableExternalButton = false;

bool otaUpdate = false;

// Analyzer mode = passive bridge + external CAN analyzer interface.
bool analyzerMode = false;

// Analyzer protocol for the TCP bridge (GVRET for SavvyCAN, Lawicel/SLCAN for CANHacker).
uint8_t analyzerProtocol = ANALYZER_PROTOCOL_GVRET;

uint32_t alerts_to_enable = 0;

long lastCANChassisTick = 0;
long lastCANHaldexTick = 0;
uint32_t canHealthTimeoutMs = 1000;

uint8_t lastMode = 0;
uint8_t disableThrottle = 0;
uint16_t disengageUnderSpeed = 0;
uint16_t disengageAboveSpeed = 0;

uint32_t rxtxcount = 0; // frame counter
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

float lockReleaseRatePerSec = 120.0f;

// setup - main inputs
bool isMPH = false; // 0 = kph, 1 = mph

uint16_t speedArray[speedArrayCount] = {0, 30, 60, 90, 120, 160, 180};
uint8_t throttleArray[throttleArrayCount] = {0, 15, 30, 45, 60, 75, 90};
uint8_t lockArray[throttleArrayCount][speedArrayCount] = {
    {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},
    {5, 5, 5, 5, 5, 40, 40},
    {40, 40, 40, 40, 40, 40, 40},
    {80, 80, 80, 80, 80, 80, 80},
    {80, 80, 80, 80, 80, 80, 80},
    {80, 80, 80, 80, 80, 80, 80}};

// for running through vars to see effects
uint8_t tempCounter;

// Haldex learn table: index = correction factor (0-100%), value = observed engagement (0-100%)
uint8_t haldexLearnTable[101];
bool haldexLearnTableValid = false;
volatile bool haldexLearnActive = false;
volatile bool haldexLearnCancel = false;
volatile uint8_t haldexLearnStep = 0;   // 0-100 = current step, 101 = complete
volatile uint8_t haldexLearnCF = 0;     // current correction factor override during learn
uint8_t tempCounter1;
uint16_t tempCounter2;

// checksum values (for calculating module checksums in standalone mode)
uint8_t MOTOR5_counter = 0;
uint8_t MOTOR6_counter = 254;
uint8_t MOTOR6_counter2 = 0;

uint8_t BRAKES1_counter = 10; // 0
uint8_t BRAKES2_counter = 0;  // starting counter for Brakes2 is 3
uint8_t BRAKES4_counter = 0;
uint8_t BRAKES4_counter2 = 0x64;
uint8_t BRAKES4_crc = 0;

uint8_t BRAKES5_counter = 0x04;
uint8_t BRAKES5_counter2 = 3; // starting counter for Brake5 is 3

uint8_t BRAKES8_counter = 0x00; // starting counter for Brakes9 is 11

uint8_t BRAKES9_counter = 0x03; // starting counter for Brakes9 is 11
uint8_t BRAKES9_counter2 = 0x00;

uint8_t BRAKES10_counter = 0;

uint8_t mLW_1_counter = 0; // was 0
uint8_t mLW_1_counter2 = 0x77;
uint8_t mLW_1_crc = 0;

uint8_t mDiagnose_1_counter = 0;

// gen5 sums
uint8_t ESP_02_counter = 0; // starts at zero
uint8_t ESP_02_crc = 0;     // starts at zero

uint8_t GETRIEBE_11_counter = 0x00;   // starting counter for GETRIEBE_11_counter is 0x00
uint8_t MOTOR_11_counter = 0x40;      // starting counter for Motor_11 is 0x40
uint8_t MOTOR_12_counter = 0x70;      // starting counter for Motor_12 is 0x70
uint8_t LWI_01_counter = 0x10;        // starting counter for LWI_01 is 0x10
uint8_t ESP_14_counter = 0x10;        // starting counter for ESP_14 is 0x10
uint8_t MOTOR_20_counter = 0x00;      // starting counter for Motor_20 is 0x00
uint8_t ESP_10_counter = 0x00;        // starting counter for ESP_10 is 0x00
uint8_t ESP_05_counter = 0x80;        // starting counter for ESP_05 is 0x80
uint8_t EPB_01_counter = 0x30;        // starting counter for EPB_01 is 0x30
uint8_t ESP_23_counter = 0x00;        // starting counter for ESP_23 is 0x00
uint8_t ESP_21_counter = 0x00;        // starting counter for ESP_21 is 0x00
uint8_t ESP_07_counter = 0x20;        // starting counter for ESP_07 is 0x20
uint8_t MOTOR_CODE_01_counter = 0x10; // starting counter for MOTOR_CODE_01 is 0x10
uint8_t ESP_20_counter = 0x30;        // starting counter for ESP_20 is 0x30
uint8_t MOTOR_14_counter = 0x10;      // starting counter for MOTOR_14 is 0x10
uint8_t ESP_19_counter = 0x20;        // starting counter for ESP_19 is 0x20
uint8_t ESP_19_counter2 = 0x00;       // starting counter for ESP_19_counter2 is 0x00

const uint8_t lws_2[16][8] = {
    {0x22, 0x00, 0x00, 0x00, 0x80, 0x00, 0xA0, 0xDD},
    {0x22, 0x00, 0x00, 0x00, 0x80, 0x10, 0x85, 0xCD},
    {0x22, 0x00, 0x00, 0x00, 0x80, 0x20, 0xEA, 0xBD},
    {0x22, 0x00, 0x00, 0x00, 0x80, 0x30, 0xCF, 0xAD},
    {0x22, 0x00, 0x00, 0x00, 0x80, 0x40, 0x34, 0x9D},
    {0x22, 0x00, 0x00, 0x00, 0x80, 0x50, 0x11, 0x8D},
    {0x22, 0x00, 0x00, 0x00, 0x80, 0x60, 0x7E, 0x7D},
    {0x22, 0x00, 0x00, 0x00, 0x80, 0x70, 0x5B, 0x6D},
    {0x22, 0x00, 0x00, 0x00, 0x80, 0x80, 0xA7, 0x5D},
    {0x22, 0x00, 0x00, 0x00, 0x80, 0x90, 0x82, 0x4D},
    {0x22, 0x00, 0x00, 0x00, 0x80, 0xA0, 0xED, 0x3D},
    {0x22, 0x00, 0x00, 0x00, 0x80, 0xB0, 0xC8, 0x2D},
    {0x22, 0x00, 0x00, 0x00, 0x80, 0xC0, 0x33, 0x1D},
    {0x22, 0x00, 0x00, 0x00, 0x80, 0xD0, 0x16, 0x0D},
    {0x22, 0x00, 0x00, 0x00, 0x80, 0xE0, 0x79, 0xFD},
    {0x22, 0x00, 0x00, 0x00, 0x80, 0xF0, 0x5C, 0xED}};

    /*
These are all for Gen5 checksum:
Calculation see specifications 'Communication Security for FlexRay and CAN'
From MQB and MLBevo: "Calculation see specifications 'End-to-End Communication Security'"
Final values see accompanying document "S-PDU Identification Sequences"
 */
// CAN ID 0x0A8 - Motor_12
const uint8_t ID_SEQ_0A8[16] = {
    0x52, 0x8C, 0x50, 0xEE, 0x4F, 0xA6, 0xCC, 0xCF,
    0x7D, 0x2F, 0x98, 0x6B, 0x27, 0x41, 0x9F, 0x93};

// CAN ID 0x0AD - Getriebe_11
const uint8_t ID_SEQ_0AD[16] = {
    0x3F, 0x69, 0x39, 0xDC, 0x94, 0xF9, 0x14, 0x64,
    0xD8, 0x6A, 0x34, 0xCE, 0xA2, 0x55, 0xB5, 0x2C};

// CAN ID 0x0A7 - Motor_11
const uint8_t ID_SEQ_0A7[16] = {
    0xd2, 0x3d, 0xcd, 0x28, 0x4c, 0x14, 0x22, 0x4b,
    0x24, 0xac, 0xfa, 0x55, 0x66, 0x80, 0x0d, 0x6c};

// CAN ID 0x08A - ESP_14
const uint8_t ID_SEQ_08A[16] = {
    0xd4, 0xd4, 0xd4, 0xd4, 0xd4, 0xd4, 0xd4, 0xd4,
    0xd4, 0xd4, 0xd4, 0xd4, 0xd4, 0xd4, 0xd4, 0xd4};

// CAN ID 0x086 - LWI_01
const uint8_t ID_SEQ_086[16] = {
    0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86,
    0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86};

// CAN ID 0x121 - MOTOR_20
const uint8_t ID_SEQ_121[16] = {
    0xe9, 0x65, 0xae, 0x6b, 0x7b, 0x35, 0xe5, 0x5f,
    0x4e, 0xc7, 0x86, 0xa2, 0xbb, 0xdd, 0xeb, 0xb4};

// CAN ID 0x110 - ESP_10
const uint8_t ID_SEQ_110[16] = {
    0xac, 0xac, 0xac, 0xac, 0xac, 0xac, 0xac, 0xac,
    0xac, 0xac, 0xac, 0xac, 0xac, 0xac, 0xac, 0xac};

// CAN ID 0x106 - ESP_05
const uint8_t ID_SEQ_106[16] = {
    0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
    0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07};

// CAN ID 0x104 - EPB_01
const uint8_t ID_SEQ_104[16] = {
    0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
    0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05};

// CAN ID 0x116 - ESP_10
const uint8_t ID_SEQ_116[16] = {
    0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
    0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05};

// CAN ID 0x101 - ESP_02
const uint8_t ID_SEQ_101[16] = {
    0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
    0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa};

// CAN ID 0x0fd - ESP_21
const uint8_t ID_SEQ_0fd[16] = {
    0xb4, 0xef, 0xf8, 0x49, 0x1e, 0xe5, 0xc2, 0xc0,
    0x97, 0x19, 0x3c, 0xc9, 0xf1, 0x98, 0xd6, 0x61};

// CAN ID 0x5be - ESP_23
const uint8_t ID_SEQ_5be[16] = {
    0xc9, 0x21, 0x6f, 0x63, 0xd2, 0x42, 0x6a, 0x77,
    0x4a, 0x3d, 0xb0, 0x62, 0x9f, 0x38, 0xcd, 0x5c};

// CAN ID 0x3be - MOTOR_14
const uint8_t ID_SEQ_3be[16] = {
    0x1f, 0x28, 0xc6, 0x85, 0xe6, 0xf8, 0xb0, 0x19,
    0x5b, 0x64, 0x35, 0x21, 0xe4, 0xf7, 0x9c, 0x24};

// CAN ID MOTOR_CODE_01 - 0x641
const uint8_t ID_SEQ_641[16] = {
    0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47,
    0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47};

// CAN ID ESP_20 - 0x645
const uint8_t ID_SEQ_645[16] = {
    0xac, 0xb3, 0xab, 0xeb, 0x7a, 0xe1, 0x3b, 0xf7,
    0x73, 0xba, 0x7c, 0x9e, 0x06, 0x5f, 0x02, 0xd9};

// CAN ID ESP_20 - 0x65d
const uint8_t ID_SEQ_65d[16] = {
    0xac, 0xb3, 0xab, 0xeb, 0x7a, 0xe1, 0x3b, 0xf7,
    0x73, 0xba, 0x7c, 0x9e, 0x06, 0x5f, 0x02, 0xd9};

// CAN ID 0x392 - ESP_07
const uint8_t ID_SEQ_392[16] = {
    0x91, 0x91, 0x91, 0x91, 0x91, 0x91, 0x91, 0x91,
    0x91, 0x91, 0x91, 0x91, 0x91, 0x91, 0x91, 0x91};

// ============================================================
//  Core CRC8/AUTOSAR function
// ============================================================
extern uint8_t crc8_autosar(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0x2F;
            }
            else
            {
                crc = (crc << 1);
            }
        }
    }
    return crc ^ 0xFF;
}

// ============================================================
//  Checksum calculator - pass 8-byte frame, returns new B0
//  frame[0] = existing/placeholder checksum (will be replaced)
//  frame[1] = counter byte (lower nibble = alive counter 0-15)
//  frame[2..7] = frame data
// ============================================================
extern uint8_t calcChecksum(uint8_t *frame, const uint8_t *idSeq)
{
    uint8_t counter = frame[1] & 0x0F; // extract alive counter from B1
    uint8_t crcInput[8];

    crcInput[0] = idSeq[counter]; // prepend DataID byte
    for (uint8_t i = 1; i < 8; i++)
    {
        crcInput[i] = frame[i]; // B1..B7
    }

    return crc8_autosar(crcInput, 8);
}

// ============================================================
//  Convenience wrappers per message
// ============================================================
extern uint8_t checksum_0A8(uint8_t *frame)
{
    return calcChecksum(frame, ID_SEQ_0A8);
}

extern uint8_t checksum_0AD(uint8_t *frame)
{
    return calcChecksum(frame, ID_SEQ_0AD);
}

extern uint8_t checksum_0A7(uint8_t *frame)
{
    return calcChecksum(frame, ID_SEQ_0A7);
}

extern uint8_t checksum_08A(uint8_t *frame)
{
    return calcChecksum(frame, ID_SEQ_08A);
}

extern uint8_t checksum_086(uint8_t *frame)
{
    return calcChecksum(frame, ID_SEQ_086);
}

extern uint8_t checksum_121(uint8_t *frame)
{
    return calcChecksum(frame, ID_SEQ_121);
}

extern uint8_t checksum_110(uint8_t *frame)
{
    return calcChecksum(frame, ID_SEQ_110);
}

extern uint8_t checksum_106(uint8_t *frame)
{
    return calcChecksum(frame, ID_SEQ_106);
}

extern uint8_t checksum_104(uint8_t *frame)
{
    return calcChecksum(frame, ID_SEQ_104);
}

extern uint8_t checksum_116(uint8_t *frame)
{
    return calcChecksum(frame, ID_SEQ_116);
}

extern uint8_t checksum_101(uint8_t *frame)
{
    return calcChecksum(frame, ID_SEQ_101);
}

extern uint8_t checksum_0fd(uint8_t *frame)
{
    return calcChecksum(frame, ID_SEQ_0fd);
}

extern uint8_t checksum_5be(uint8_t *frame)
{
    return calcChecksum(frame, ID_SEQ_5be);
}

extern uint8_t checksum_3be(uint8_t *frame)
{
    return calcChecksum(frame, ID_SEQ_3be);
}

extern uint8_t checksum_641(uint8_t *frame)
{
    return calcChecksum(frame, ID_SEQ_641);
}

extern uint8_t checksum_645(uint8_t *frame)
{
    return calcChecksum(frame, ID_SEQ_645);
}

extern uint8_t checksum_65d(uint8_t *frame)
{
    return calcChecksum(frame, ID_SEQ_65d);
}

extern uint8_t checksum_392(uint8_t *frame)
{
    return calcChecksum(frame, ID_SEQ_392);
}