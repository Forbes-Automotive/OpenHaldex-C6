#include <OpenHaldexC6_defs.h>
#include <OpenHaldexC6_Calculations.h> // LearnScore / LongLearnSweep types for the Long Learn globals

// TWAI handles
twai_handle_t twai_bus_0; // for ESP32 C6 CANBUS 0
twai_handle_t twai_bus_1; // for ESP32 C6 CANBUS 1

twai_message_t rx_message_hdx; // incoming haldex message
twai_message_t rx_message_chs; // incoming chassis message

twai_message_t tx_message_hdx; // outgoing haldex message
twai_message_t tx_message_chs; // outgoing chassis message

TaskHandle_t handle_standaloneFrames = nullptr; // standalone frame scheduler (runs only while standalone)
TaskHandle_t handle_broadcastOpenHaldex = nullptr; // OpenHaldex CAN broadcast task (suspended in aggressive sleep)
TaskHandle_t handle_showHaldexState     = nullptr; // serial state logger (suspended in aggressive sleep)
TaskHandle_t handle_updateTriggers      = nullptr; // notified by CAN_RX wake ISRs in aggressive sleep

// for EEP
Preferences pref; // for EEPROM / storing settings

// for LED - will be initialized in setupIO()
OneLed strip(gpio_led, LED_COLOR_ORDER_RGB); // 1 led, gpio pin, colour order (was Freenove TYPE_RGB)

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

// Gen41 Haldex feedback (FDCM-originated, decoded per GMW8762 PPEI)
uint8_t  received_sec_axle_status_raw = 0;
uint8_t  received_sec_axle_torque_nm = 0;
uint8_t  received_sec_axle_clutch_state = 0;
bool     received_sec_axle_active = false;
bool     received_sec_axle_fdcm_healthy = false;
uint8_t  received_sec_axle_arc = 0;

uint8_t  received_rear_axle_status_flags = 0;
uint8_t  received_rear_axle_metric_a = 0;
uint8_t  received_rear_axle_metric_b = 0;

bool     received_haldex_alive_bus0 = false;
bool     received_drivetrain_state_ok = false;
uint32_t received_haldex_alive_bus0_ms = 0;
uint32_t received_drivetrain_state_ms = 0;

// values received from Haldex CAN
bool received_report_clutch1;
bool received_report_clutch2;
bool received_temp_protection;
bool received_coupling_open;
bool received_speed_limit;
bool received_limp_mode;                // Gen1/2/4: Allrad_1 byte 0 bit 4 (Notlauf - limp mode)
bool received_awd_warning;             // Gen1/2/4: Allrad_1 byte 0 bit 5 (Allrad_Warnlampe - AWD warning lamp)
uint8_t received_long_lock_state = 0;  // Gen5: 0x118 byte 1 bits 4..5 (0 = normal, 2 = fault reaction / clutch released; 0CQ OEM code never sends 1 or 3). VAQ: QUER_Sta 0..7
bool received_kickdown = false;

// Haldex health (V11.00.0) - see the "Haldex health" notes in _defs.h
bool received_temp_sensor_fault = false; // the unit's own temperature sensor has failed (Gen4/0AY 0x2C0 b3.3)
bool received_fault_stored = false;      // the unit has a fault stored in its memory (Gen4/0AY 0x2C0 b3.7)
bool haldexHealthSupported = false;      // set by the feedback decode when this generation's health bits are known
bool received_lever_request_valid = false; // Gen5 0CQ: 0x118 b1 bit 6 = the unit accepts the ESP_14 request (OEM code: spec_gen5_0cq §6)

// values received from Chassis CAN
float received_pedal_value = 0;
uint16_t received_vehicle_speed = 0;
float received_steering_angle = 0; // steering-wheel angle magnitude (deg, abs)
uint32_t received_steering_ms = 0; // millis() of last decoded steering frame (0 = never)
bool received_steering_negative = false; // steering direction sign for slip geometry

// Per-corner slip (Phase F): raw wheel speeds, computed slip, and per-car geometry.
uint16_t wheelSpeedRaw[4] = {0, 0, 0, 0}; // [FL, FR, RL, RR], ESP_19 units (0.0075 km/h/bit)
uint32_t lastWheelSpeedResponse = 0;       // millis() of last ESP_19 frame (0 = never)
int8_t cornerSlip[4] = {-128, -128, -128, -128}; // [FL, FR, RL, RR] signed %, -128 = no data
uint32_t lastCornerSlipMs = 0;             // millis() of last slip computation (0 = never)
float slipSteeringRatio = 15.0f;           // steering-wheel:road-wheel ratio (Audi TT Mk3)
uint16_t slipWheelbaseMm = 2505;           // wheelbase
uint16_t slipTrackFrontMm = 1572;          // front track
uint16_t slipTrackRearMm = 1543;           // rear track
uint16_t slipMinSpeedRaw = 667;            // ~5 km/h floor below which slip is untrustworthy
uint16_t received_vehicle_rpm;
bool received_kl15 = false;
uint32_t lastKl15Ms = 0;

// Klemmen_Status_01 is sent every 100 ms and stops when the bus sleeps,
// so an old value means "unknown", not "on".
bool ignitionOn()
{
  return lastKl15Ms != 0 && (millis() - lastKl15Ms) < 2000UL && received_kl15;
}
uint16_t received_vehicle_boost;
uint8_t haldexGeneration;
uint8_t tcForceModeValue     = 2; // default 50:50
uint8_t hazardForceModeValue = 2; // default 50:50
uint8_t extBtnForceModeValue = 2; // default 50:50

uint32_t lastABSResponse = 0; // for tracking ABS responses to determine if ABS is valid
bool isABSValid = false;      // set to true if valid ABS response received within timeout, otherwise false
uint32_t absTimeout = 1000;    // timeout in ms for determining if ABS is valid - if no valid ABS response received within this time, isABSValid will be set to false

bool isStandalone = false;
bool useCANifAvailable = false;

bool isBusFailure = false;
bool hasCANChassis = false;
bool hasCANHaldex = false;
bool broadcastOpenHaldexOverCAN = true;
bool disableController = false;
bool followBrake = false;
bool invertBrake = false;
bool followHandbrake = false;
bool followABS = true; // V11.00.0: open the clutch while the ABS is braking (the Haldex's own safety); off = our lock overrules the ABS

// Volvo P1 standalone lock settings (V11.00.1) - see defs.h
uint8_t volvoP1LockSource = VOLVO_P1_SOURCE_TORQUE; // lock shown as engine torque (what the module follows in the logs)
uint8_t volvoP1RoadSpeedKmh = 30;                   // standalone road speed, km/h
uint16_t volvoP1TorqueFull = 600;                   // torque counts at 100 % lock (most seen on full-throttle launches)
uint8_t volvoP1SlipFullKmh = 10;                    // front wheels this much faster at 100 % lock, km/h (the wheelspin log had ~10)
uint16_t volvoP1SentTorque = 0;                     // last torque sent
uint8_t volvoP1SentPedal = 0;                       // last pedal byte sent
uint16_t volvoP1SentRpm = 0;                        // last rpm sent
uint16_t volvoP1SentFrontRaw = 0;                   // last front wheel speed sent, 0.01 km/h
uint16_t volvoP1SentRearRaw = 0;                    // last rear wheel speed sent, 0.01 km/h
uint16_t volvoP1EngagementCount = 0;                // last 0x1960000A engagement count
bool invertHandbrake = false;

bool rebootWiFi = false;
bool lowPowerMode = false;
void *pmNoLightSleepLock = nullptr; // created in setup() when CAN sleep is enabled
volatile uint32_t lpChassisFrameCount = 0; // incremented by chassis CAN task; used to measure bus activity during probe window
volatile uint32_t lpHaldexFrameCount = 0;  // incremented by haldex CAN task; used for standalone probe window
char wifiPassword[65] = ""; // WiFi AP password - empty string = open network
char wifiSsid[33] = wifiHostNameDefault; // runtime AP SSID (factory default, overridden from EEPROM)

char wifiStaSsid[33] = "";     // bridge mode: home-network SSID - empty = disabled, AP only
char wifiStaPassword[65] = ""; // bridge mode: home-network password - empty = open network
bool wifiStaConnected = false; // runtime: associated to the home network with an IP
char wifiStaIP[16] = "";       // runtime: dotted quad once connected, "" otherwise

bool hazardForceMode = false;     // setting: use hazard lights to activate force mode
bool hazardForceModeFlag = false; // runtime: hazard lights are currently on

bool brakeActive = false;
bool brakeSignalActive = false;

bool handbrakeActive = false;
bool handbrakeSignalActive = false;

bool brakeFromCAN = false;
bool handbrakeFromCAN = false;
bool absBrakingFromCAN = false; // ABS module is braking (PQ Bremse_1 b0.2, MQB ESP_21 b7.0); cleared when the chassis bus goes quiet

bool tcForceMode = false;
bool tcForceModeFlag = false;
bool asrForceModeFlag = false;

bool paddleTipActive = false;
bool paddleTipUp     = false;
bool paddleTipDown   = false;
bool paddleTipBoth   = false;

bool extBtnForceMode = false;
bool extButtonForceModeFlag = false;

// Drive mode select (MQB "Charisma"): which OpenHaldex mode each drive profile forces.
// 0 Stock, 1 FWD, 2 50:50, 3 60:40, 4 75:25, 5 Expert, DRIVE_MODE_NOT_SET = leave the mode alone.
bool driveModeForce = false; // setting: off until the user turns it on
uint8_t driveModeForceValue[DRIVE_PROFILE_COUNT] = {
    DRIVE_MODE_NOT_SET, // 0  none
    MODE_STOCK,         // 1  Comfort    -> Stock
    DRIVE_MODE_NOT_SET, // 2  Normal
    MODE_6040,          // 3  Sport      -> 60:40
    DRIVE_MODE_NOT_SET, // 4  Offroad
    DRIVE_MODE_NOT_SET, // 5  Eco
    MODE_5050,          // 6  Race       -> 50:50
    MODE_EXPERT,        // 7  Individual -> Expert
    DRIVE_MODE_NOT_SET, // 8  Range / PHEV 1
    DRIVE_MODE_NOT_SET, // 9  Lift / PHEV 2
    DRIVE_MODE_NOT_SET, // 10 Snow / PHEV 3
    DRIVE_MODE_NOT_SET, // 11 Offroad Individual / PHEV 4
};
uint8_t receivedDriveProfile = 0;               // drive profile the car is in (0 = none)
uint8_t driveProfileSource = DRIVE_SOURCE_NONE; // which frame the profile came from
uint32_t lastDriveProfileMs = 0;                // when the profile was last read
uint32_t lastCharisma07Ms = 0;                  // when Charisma_07 was last seen (0 = never)
uint8_t receivedHaldexModeRequest = 0;          // mode the car's gateway asks the Haldex for
uint8_t receivedVaqModeRequest = 0;             // mode the car's gateway asks the VAQ for
bool hasHaldexModeRequest = false;              // true once a Charisma_01 frame has been read

bool disableOnboardButton = false;
bool disableExternalButton = false;

bool fixHunting = false; // when true, Motor_11 uses BPK packing instead of V3

uint16_t bpkCeilingNm = 220; // per-car Gen5 BPK ceiling (Nm at 100% command); default preserves prior fixed value
uint8_t esp14MinFloorPct = 0; // ESP_14 launch PWM floor (%); 0 = unchanged

// BPK packing tunables - defaults are the previously hardcoded values.
uint16_t bpkFloorNm = 10;
uint16_t bpkSlewIst = 8;
uint16_t bpkSlewSolf = 32;
uint16_t bpkTraegRaw = 509; // = 0 Nm
uint16_t bpkSchubRaw = 487; // = -22 Nm
uint8_t bpkStatusFl = 0x20; // Normalbetrieb only; 0x80 would add MO_QBit_Motormomente
int32_t bpkForceIstNm = -1;
int32_t bpkForceSolfNm = -1;
volatile uint8_t bpkLastFrame[8] = {0};

bool dangerZoneEnabled = false; // full-duty 50:50 (see defs.h) - off by default
uint16_t dangerZoneNm = 320;    // Motor_11 BPK ceiling used while Danger Zone is live (bench: 285 pegs the PRV, 320 -> 90 % / 11 A)

LabOverride labOverrides[LAB_OVR_MAX] = {};

// ESP_19 wheel-speed tunables - defaults reproduce the legacy counter exactly.
bool wsFreeze = false;
uint16_t wsBaseRaw = 0;
uint16_t wsDitherRaw = 0;
int32_t wsFrontDeltaRaw = 0;
int32_t wsLeftRightDeltaRaw = 0; // VAQ bench: +n raises VL and lowers VR by n/2 each (front L/R slip)

volatile uint16_t bpkLastTorqueNm = 0;
volatile uint16_t bpkLastIstNm = 0;
volatile uint16_t bpkLastSolfNm = 0;

// ---- Frame-edit gating (per-CAN-ID passthrough toggles) --------------------
// Bit layout per generation matches the order the cases appear in getLockData().
// Defaults: currently-active (compiled) frames ON; frames transferred from
// standalone / previously commented-out are OFF (=> clean passthrough).
// V11.00.0 (Adam): in every non-Stock mode the job is to ACHIEVE the lock %. The
// OEM-code work confirmed which bytes do that, so the OEM defaults are exactly the
// blocks that drive lock: the lever byte the unit reads + the bench-proven
// lock-scaled wheel speeds (+ Motor_1 demand where the unit uses it). Blocks that
// only overwrite things the unit doesn't need for lock (Bremse_1 ABS flags,
// steering, gear ...) stay off. Stock = untouched passthrough, always.
const uint64_t frameEditMaskDefaults[FE_GEN_COUNT] = {
    0x00000019ULL, // FE_GEN_1 : Motor_1 (0, demand), Bremse_3 (3, wheel speeds), Bremse_4 (4, ceiling lever - only edited when the car sends 0x2A0) (spec_gen1_02d §2, was 0x0F)
    0x00000031ULL, // FE_GEN_2 : Motor_1 (0, demand), Bremse_3 (4, wheel speeds), Bremse_4 (5, the lever) (spec_gen2_7h0 §2, was 0x1F)
    0x00000032ULL, // FE_GEN_4 : Motor_1 (1, lock-scaled), Bremse_3 (4, wheel speeds), Bremse_4 (5, the lever) (spec_gen4_0br §2.6, was 0x3F)
    0x0000001DULL, // FE_GEN_50: ESP_19 (0, wheel speeds), Motor_12 (2), Motor_11 (3), ESP_14 (4, the lever Min = Max = V) (spec_gen5_0cq §2, was 0xFF). Also gen 52 (VAQ), whose Motor_12 still follows lock
    0x00000006ULL, // FE_GEN_51: Bremse_3 (1, wheel speeds), Bremse_4 (2, the lever) (spec_gen5_0ay §2, was 0x0F)
    0x000000FFULL, // FE_GEN_SPARE_60: not used in V11 - V12's Gen6 (Magma) default, so a mask saved here reads right after moving to V12
    0x00000000ULL, // FE_GEN_VOLVO_P1: nothing edited in OEM mode yet (V11.00.1: OEM mode is untouched passthrough for the P1)
};

uint64_t frameEditMask[FE_GEN_COUNT] = {
    0x00000019ULL, 0x00000031ULL, 0x00000032ULL, 0x0000001DULL, 0x00000006ULL, 0x000000FFULL, 0x00000000ULL};

// Standalone frame-edit defaults: EVERYTHING on. In standalone the module
// synthesises the whole bus, so all editable frames must be generated by
// default (the passthrough defaults above only enable the historically-edited
// subset). A separate mask keeps the two modes fully independent.
const uint64_t frameEditMaskDefaultsSA[FE_GEN_COUNT] = {
    0x0000003DULL, // FE_GEN_1 : Motor_1, Bremse_1, Bremse_3, Bremse_4, Motor_2 (V11 spec_gen1_02d §5; Motor_3 + Kombi_1 off)
    0x00001E7DULL, // FE_GEN_2 : V11 - all on except Motor_3 (1), Bremse_9 (7), Bremse_10 (8): not received by 7H0/1K0 (OEM code: spec_gen2_7h0 §5, was 0x1FFF)
    0x0001F9FFULL, // FE_GEN_4 : V11 - bits 0-8, 11, 12-16 on; Bremse_11 (9) + Kombi_2 (10) off - Gen4 does not read them (OEM code: spec_gen4_0br §5, was 0xFFF)
    0x0FFFFFFFULL, // FE_GEN_50: bits 0-27 all on. The 0CQ alone only needs 0x0FE0FB5F (ESP_10 (5), EPB_01 (7), ESP_18 (10), bits 16-20 not received - spec_gen5_0cq §5) and ignores the rest, but gen 52 VAQ shares this mask and its receive set is not traced, so its bench-proven frames stay on (V11.00.0)
    0x0006BF7FULL, // FE_GEN_51: V11 - bits 0-6, 8-13, 15, 17, 18 (Getriebe_1) on; Bremse_8 (7), Bremse_11 (14), Kombi_2 (16) off - not received by the 0AY (OEM code: spec_gen5_0ay §5, was 0x3FFFF)
    0x1FFFFFFFFFULL, // FE_GEN_SPARE_60: not used in V11 - V12's Gen6 (Magma) standalone default (bits 0-36 on)
    0x003FFFFFULL,   // FE_GEN_VOLVO_P1: all 22 car frames on (bits 0-21) - turn off the ones the AWD module turns out not to need
};

uint64_t frameEditMaskSA[FE_GEN_COUNT] = {
    0x0000003DULL, 0x00001E7DULL, 0x0001F9FFULL, 0x0FFFFFFFULL, 0x0006BF7FULL, 0x1FFFFFFFFFULL, 0x003FFFFFULL};

// Blocks whose payload is built from lock_target (get_lock_target_adjusted_value)
// - the ones we MUST edit to have control. Long Learn always sends these and
// only tests the rest (the static payloads). Keep in step with getLockData().
const uint64_t frameEditLockDriven[FE_GEN_COUNT] = {
    0x00000019ULL, // FE_GEN_1 : Motor_1, Bremse_3, Bremse_4 (V11: Motor_3 not received, Bremse_1 carries no lock)
    0x00000031ULL, // FE_GEN_2 : V11 - Motor_1 (demand), Bremse_3 (slip), Bremse_4 (the lever). Motor_3 not received, Bremse_1/2 carry no lock (spec_gen2_7h0, was 0x1F)
    0x00000032ULL, // FE_GEN_4 : V11 - Motor_1 (lock-scaled), Bremse_3 (wheel speeds), Bremse_4 (the lever) (spec_gen4_0br §2.6, was 0x3E)
    0x0000001DULL, // FE_GEN_50: ESP_19 (wheel speeds), Motor_12, Motor_11, ESP_14 (the lever) - Motor_12 still follows lock on the VAQ (spec_gen5_0cq §2)
    0x00000006ULL, // FE_GEN_51: V11 - Bremse_3 (wheel speeds), Bremse_4 (the lever) (spec_gen5_0ay §2, was 0x07)
    0x1000001DULL, // FE_GEN_SPARE_60: not used in V11 - V12's Gen6 value
    0x00000003ULL, // FE_GEN_VOLVO_P1: engine torque + pedal (0, 0x0300410E) and RPM (1, 0x0090411E) follow the lock
};

// Descriptor table used by the API/UI. Order per generation defines the bit index.
const FrameEditBlock frameEditBlocks[] = {
    // Gen1
    {FE_GEN_1, 0, MOTOR1_ID, "Motor_1 (0x280)"},
    {FE_GEN_1, 1, MOTOR3_ID, "Motor_3 (0x380)"},
    {FE_GEN_1, 2, BRAKES1_ID, "Bremse_1 (0x1A0)"},
    {FE_GEN_1, 3, BRAKES3_ID, "Bremse_3 (0x4A0)"},
    {FE_GEN_1, 4, BRAKES4_ID, "Bremse_4 (0x2A0)"}, // V11: torque-ceiling lock lever (spec_gen1_02d §2.3)
    {FE_GEN_1, 5, MOTOR2_ID, "Motor_2 (0x288)"},   // V11: standalone only - the 02D needs it for "all inputs valid"
    {FE_GEN_1, 6, mKombi_1, "Kombi_1 (0x320)"},    // V11: standalone handbrake (off by default: the 02D's own handbrake wire is used)
    // Gen2
    {FE_GEN_2, 0, MOTOR1_ID, "Motor_1 (0x280)"},
    {FE_GEN_2, 1, MOTOR3_ID, "Motor_3 (0x380)"},
    {FE_GEN_2, 2, BRAKES1_ID, "Bremse_1 (0x1A0)"},
    {FE_GEN_2, 3, BRAKES2_ID, "Bremse_2 (0x5A0)"},
    {FE_GEN_2, 4, BRAKES3_ID, "Bremse_3 (0x4A0)"},
    {FE_GEN_2, 5, BRAKES4_ID, "Bremse_4 (0x2A0)"},
    {FE_GEN_2, 6, BRAKES5_ID, "Bremse_5 (0x4A8)"},
    {FE_GEN_2, 7, BRAKES9_ID, "Bremse_9 (0x0AE)"},
    {FE_GEN_2, 8, BRAKES10_ID, "Bremse_10 (0x3A0)"},
    {FE_GEN_2, 9, MOTOR5_ID, "Motor_5 (0x480)"},
    {FE_GEN_2, 10, mLW_1, "LW_1 (0x0C2)"},
    {FE_GEN_2, 11, MOTOR2_ID, "Motor_2 (0x288)"},
    {FE_GEN_2, 12, mKombi_1, "Kombi_1 (0x320)"},
    // Gen4
    {FE_GEN_4, 0, mLW_1, "LW_1 (0x0C2)"},
    {FE_GEN_4, 1, MOTOR1_ID, "Motor_1 (0x280)"},
    {FE_GEN_4, 2, BRAKES1_ID, "Bremse_1 (0x1A0)"},
    {FE_GEN_4, 3, BRAKES2_ID, "Bremse_2 (0x5A0)"},
    {FE_GEN_4, 4, BRAKES3_ID, "Bremse_3 (0x4A0)"},
    {FE_GEN_4, 5, BRAKES4_ID, "Bremse_4 (0x2A0)"},
    {FE_GEN_4, 6, mKombi_1, "Kombi_1 (0x320)"},
    {FE_GEN_4, 7, mKombi_3, "Kombi_3 (0x520)"},
    {FE_GEN_4, 8, mGate_Komf_1, "Gate_Komf_1 (0x390)"},
    {FE_GEN_4, 9, BRAKES11_ID, "Bremse_11 (0x5B7)"},
    {FE_GEN_4, 10, mKombi_2, "Kombi_2 (0x420)"},
    {FE_GEN_4, 11, mDiagnose_1, "Diagnose_1 (0x7D0)"},
    // Gen4 V11.00.0: frames the 0BR monitors that we never sent (OEM code: spec_gen4_0br §3).
    // Standalone builds them (Gen4_frames20); in OEM mode the car's own frame always passes through.
    {FE_GEN_4, 12, MOTOR2_ID, "Motor_2 (0x288)"},      // brake light b2.0, max torque mux
    {FE_GEN_4, 13, MOTOR5_ID, "Motor_5 (0x480)"},      // missing = unit sets its own limp bit
    {FE_GEN_4, 14, BRAKES5_ID, "Bremse_5 (0x4A8)"},    // yaw rate
    {FE_GEN_4, 15, BRAKES8_ID, "Bremse_8 (0x1AC)"},    // ESP status / counter
    {FE_GEN_4, 16, mGetriebe_1, "Getriebe_1 (0x440)"}, // gear / torque ratio
    // Gen50 (0CQ MQB)
    {FE_GEN_50, 0, ESP_19, "ESP_19 (0x0B2)"},
    {FE_GEN_50, 1, GETRIEBE_11, "Getriebe_11 (0x0AD)"},
    {FE_GEN_50, 2, MOTOR_12, "Motor_12 (0x0A8)"},
    {FE_GEN_50, 3, MOTOR_11, "Motor_11 (0x0A7)"},
    {FE_GEN_50, 4, ESP_14, "ESP_14 (0x08A)"},
    {FE_GEN_50, 5, ESP_10, "ESP_10 (0x116)"},
    {FE_GEN_50, 6, ESP_05, "ESP_05 (0x106)"},
    {FE_GEN_50, 7, EPB_01, "EPB_01 (0x104)"},
    {FE_GEN_50, 8, MOTOR_14, "Motor_14 (0x3BE)"},
    {FE_GEN_50, 9, ESP_07, "ESP_07 (0x392)"},
    {FE_GEN_50, 10, ESP_18, "ESP_18 (0x135)"},
    {FE_GEN_50, 11, LWI_01, "LWI_01 (0x086)"},
    {FE_GEN_50, 12, MOTOR_20, "Motor_20 (0x121)"},
    {FE_GEN_50, 13, ESP_02, "ESP_02 (0x101)"},
    {FE_GEN_50, 14, ESP_21, "ESP_21 (0x0FD)"},
    {FE_GEN_50, 15, KOMBI_01, "Kombi_01 (0x30B)"},
    {FE_GEN_50, 16, ESP_23, "ESP_23 (0x5BE)"},
    {FE_GEN_50, 17, Parkhilfe_04, "Parkhilfe_04 (0x54B)"},
    {FE_GEN_50, 18, GATEWAY_72, "Gateway_72 (0x3DB)"},
    {FE_GEN_50, 19, GETRIEBE_14, "Getriebe_14 (0x3C8)"},
    {FE_GEN_50, 20, ESP_29, "ESP_29 (0x18C)"},
    {FE_GEN_50, 21, MOTOR_07, "Motor_07 (0x640)"},
    {FE_GEN_50, 22, CHARISMA_01, "Charisma_01 (0x385)"},
    {FE_GEN_50, 23, SYSTEMINFO_01, "Systeminfo_01 (0x585)"},
    {FE_GEN_50, 24, MOTOR_CODE_01, "Motor_Code_01 (0x641)"},
    {FE_GEN_50, 25, ESP_20, "ESP_20 (0x65D)"},
    {FE_GEN_50, 26, DIAGNOSE_01, "Diagnose_01 (0x6B2)"},
    {FE_GEN_50, 27, KOMBI_02, "Kombi_02 (0x6B7)"},
    // Gen51 (0AY - a PQ unit: PQ frame set, Bremse_4 lever, Allrad_1 0x2C0 feedback - spec_gen5_0ay §1)
    {FE_GEN_51, 0, MOTOR1_ID, "Motor_1 (0x280)"},
    {FE_GEN_51, 1, BRAKES3_ID, "Bremse_3 (0x4A0)"},
    {FE_GEN_51, 2, BRAKES4_ID, "Bremse_4 (0x2A0)"},
    {FE_GEN_51, 3, mLW_1, "LW_1 (0x0C2)"},
    {FE_GEN_51, 4, BRAKES1_ID, "Bremse_1 (0x1A0)"},
    {FE_GEN_51, 5, mGetriebe_2, "Getriebe_2 (0x540)"},
    {FE_GEN_51, 6, BRAKES5_ID, "Bremse_5 (0x4A8)"},
    {FE_GEN_51, 7, BRAKES8_ID, "Bremse_8 (0x1AC)"},
    {FE_GEN_51, 8, BRAKES2_ID, "Bremse_2 (0x5A0)"},
    {FE_GEN_51, 9, MOTOR2_ID, "Motor_2 (0x288)"},
    {FE_GEN_51, 10, MOTOR5_ID, "Motor_5 (0x480)"},
    {FE_GEN_51, 11, mKombi_1, "Kombi_1 (0x320)"},
    {FE_GEN_51, 12, mKombi_3, "Kombi_3 (0x520)"},
    {FE_GEN_51, 13, mGate_Komf_1, "Gate_Komf_1 (0x390)"},
    {FE_GEN_51, 14, BRAKES11_ID, "Bremse_11 (0x5B7)"},
    {FE_GEN_51, 15, mSysteminfo_1, "Systeminfo_1 (0x5D0)"},
    {FE_GEN_51, 16, mKombi_2, "Kombi_2 (0x420)"},
    {FE_GEN_51, 17, mDiagnose_1, "Diagnose_1 (0x7D0)"},
    // Gen51 V11.00.0: frame the 0AY monitors that we never sent (OEM code: spec_gen5_0ay §3).
    // Standalone builds it (Gen5_0AY_frames20); in OEM mode the car's own frame always passes through.
    {FE_GEN_51, 18, mGetriebe_1, "Getriebe_1 (0x440)"}, // gear (b1 low nibble)
    // Volvo P1 (generation 31, V11.00.1): every frame on the V50's high-speed bus,
    // all sent in standalone. Bits 0-1 follow the lock; the rest are steady values
    // from the logs (plus their counters). Names say what's known about each one.
    {FE_GEN_VOLVO_P1, 0, VOLVO_P1_ENGINE_TORQUE_ID, "Engine Torque / Pedal / Brake (0x0300410E)"},
    {FE_GEN_VOLVO_P1, 1, VOLVO_P1_ENGINE_RPM_ID, "Engine RPM (0x0090411E)"},
    {FE_GEN_VOLVO_P1, 2, VOLVO_P1_WHEELS_ID, "Wheel Speeds (0x01C04026)"},
    {FE_GEN_VOLVO_P1, 3, VOLVO_P1_SPEED_ID, "Vehicle Speed (0x02104136)"},
    {FE_GEN_VOLVO_P1, 4, VOLVO_P1_BRAKE_ID, "Brake Pressure / DSTC (0x02000026)"},
    {FE_GEN_VOLVO_P1, 5, VOLVO_P1_HANDBRAKE_ID, "Handbrake (0x0110483C)"},
    {FE_GEN_VOLVO_P1, 6, VOLVO_P1_STEERING_ID, "Steering Angle (0x02804026)"},
    {FE_GEN_VOLVO_P1, 7, VOLVO_P1_0100082C_ID, "Unknown (0x0100082C)"},
    {FE_GEN_VOLVO_P1, 8, VOLVO_P1_STEER_SENSOR_ID, "Steering Angle Sensor (0x0340412E)"},
    {FE_GEN_VOLVO_P1, 9, VOLVO_P1_02C00020_ID, "Unknown (0x02C00020)"},
    {FE_GEN_VOLVO_P1, 10, VOLVO_P1_00800006_ID, "Unknown (0x00800006)"},
    {FE_GEN_VOLVO_P1, 11, VOLVO_P1_03800006_ID, "Unknown (0x03800006)"},
    {FE_GEN_VOLVO_P1, 12, VOLVO_P1_19000026_ID, "Unknown (0x19000026)"},
    {FE_GEN_VOLVO_P1, 13, VOLVO_P1_PULSES_ID, "Wheel Pulse Counters (0x19A00002)"},
    {FE_GEN_VOLVO_P1, 14, VOLVO_P1_19E00006_ID, "Unknown (0x19E00006)"},
    {FE_GEN_VOLVO_P1, 15, VOLVO_P1_1B200002_ID, "Unknown (0x1B200002)"},
    {FE_GEN_VOLVO_P1, 16, VOLVO_P1_1B600002_ID, "Unknown (0x1B600002)"},
    {FE_GEN_VOLVO_P1, 17, VOLVO_P1_1B700030_ID, "Unknown (0x1B700030)"},
    {FE_GEN_VOLVO_P1, 18, VOLVO_P1_1A200020_ID, "Unknown (0x1A200020)"},
    {FE_GEN_VOLVO_P1, 19, VOLVO_P1_1AE0092C_ID, "Unknown (0x1AE0092C)"},
    {FE_GEN_VOLVO_P1, 20, VOLVO_P1_1BA0493C_ID, "Unknown, 20 Pages (0x1BA0493C)"},
    {FE_GEN_VOLVO_P1, 21, VOLVO_P1_NETWORK_ID, "Network Heartbeat (0x1BE0493C)"},
};
const uint16_t frameEditBlockCount = sizeof(frameEditBlocks) / sizeof(frameEditBlocks[0]);

int frameEditGenIdx(uint8_t generation)
{
    switch (generation)
    {
    case 1:
        return FE_GEN_1;
    case 2:
        return FE_GEN_2;
    case 4:
        return FE_GEN_4;
    case 50:
        return FE_GEN_50;
    case 52:
        return FE_GEN_50; // Gen5 0CQ VAQ shares 0CQ frame-edit toggles (clone base)
    case 51:
        return FE_GEN_51;
    case GEN_VOLVO_P1:
        return FE_GEN_VOLVO_P1; // Volvo P1: the standalone frames can be turned on / off
    default:
        return -1; // not gated (e.g. gen41/42)
    }
}

uint64_t *activeFrameEditMask()
{
    // Standalone generates the whole bus (frameEditMaskSA, all-on default);
    // normal mode edits real passthrough frames (frameEditMask). Each consumer
    // (standaloneTx / passthrough editor / API) runs in the matching mode, so
    // routing through this keeps every one of them on the correct mask.
    return isStandalone ? frameEditMaskSA : frameEditMask;
}

bool frameEditEnabled(uint8_t genIdx, uint8_t bit)
{
    if (genIdx >= FE_GEN_COUNT)
        return true;
    return (activeFrameEditMask()[genIdx] >> bit) & 0x1ULL;
}

void resetFrameEditMask()
{
    for (uint8_t i = 0; i < FE_GEN_COUNT; i++)
    {
        frameEditMask[i]   = frameEditMaskDefaults[i];
        frameEditMaskSA[i] = frameEditMaskDefaultsSA[i];
    }
}

bool canSleepEnabled = true;
bool canSleepAggressive = false; // opt-in: transceiver standby + DFS floor 10MHz + low WiFi TX power
bool benchMode = false;          // opt-in: hold WiFi up on the bench until real CAN traffic is seen
bool bleEnabled = true;          // BLE link to the DashCAN app
uint32_t blePasskey = 0;         // 0 = not made yet; setupBLE() generates one
volatile bool canWakeRequest = false; // ISR-set wake flag when transceivers in standby see bus activity
uint16_t lpWakeThresholdFps = 1100; // wake threshold fps; default 1100 — user adjustable via UI
uint8_t sleepCalState = SLEEP_CAL_NONE;
uint16_t sleepCalAvgFps = 0;

bool otaUpdate = false;

// Analyzer mode (external CAN analyzer interface)
bool analyzerMode = false;   // WiFi GVRET/SLCAN (TCP Port 23)
bool analyzerSerial = false; // Serial GVRET (SavvyCAN Serial Connection)

// UDS MQB diagnostic polling (Gen 5 only)
bool liveDiagEnabled = false;             // master live-diagnostics enable; gates UDS (Gen5) + TP2.0 (Gen2/4)
volatile uint32_t externalDiagLastMs = 0; // last time an external scanner request was seen on Bus 0 (0 = never)
QueueHandle_t udsRxQueue = nullptr;
volatile bool udsPollActive = false;      // poller owns the Haldex diag channel -> 0x779 replies stay off Bus 0
volatile uint8_t udsSessionMode = 0;      // 0 none, 0x01 default session, 0x03 extended session
QueueHandle_t udsWebRxQueue = nullptr;    // /api/uds/read responses, tapped (copied) by the parse task
volatile uint32_t udsWebRespId = 0;       // response ID /api/uds/read waits for; 0 = no read in flight
volatile uint8_t udsWebBus = 0;           // bus the in-flight /api/uds/read is using

// Haldex-bus diagnostic addressing (see defs.h). Defaults are the Haldex pair;
// udsApplyDefaultIds() swaps to the VAQ pair for generation 52.
uint32_t udsHaldexReqId = ISO_ALLRAD_REQ;
uint32_t udsHaldexRespId = ISO_ALLRAD_RESP;
bool udsIdsManual = false;

// Haldex-bus RX census + last feedback frame (serial lab RXIDS / FB).
HdxRxStat hdxRxStats[HDX_RX_STATS_MAX] = {};
volatile uint32_t hdxRxDroppedIds = 0;
volatile uint32_t hdxFbMs = 0;
volatile uint32_t hdxFbId = 0;
volatile uint8_t hdxFbDlc = 0;
volatile uint8_t hdxFbData[8] = {0};
uint8_t received_quer_state = 0;
uint8_t received_quer_sync = 0;
volatile uint32_t canTxDropBus0 = 0;      // canTransmit() failures on Bus 0 (chassis)
volatile uint32_t canTxDropBus1 = 0;      // canTransmit() failures on Bus 1 (Haldex)
float udsTerminalVoltage = 0.0f;
float udsModuleTemp = 0.0f;
float udsClutchTemp = 0.0f;
float udsCoolingFinTemp = 0.0f;
float udsClutchCurrent = 0.0f;
uint8_t udsClutchPWM = 0;
float udsClutchVoltage = 0.0f;
uint8_t udsBlockagePct = 0;
volatile uint32_t udsLastDecodeMs = 0;

// KWP2000 over VW TP2.0 diagnostics (Gen2 / Gen4 PQ Haldex)
QueueHandle_t tp20RxQueue = nullptr;
volatile uint32_t kwpTp20EcuTxId = 0;
bool kwpTp20Connected = false;
char kwpTp20RawDump[512] = {0};

// Gen4 (0AY) scaled measuring values (see OpenHaldexC6_defs.h for formulas)
float kwpOilTemp = 0.0f;
float kwpPlateTemp = 0.0f;
float kwpSupplyVoltage = 0.0f;
float kwpOilPressure = 0.0f;
float kwpEstTorque = 0.0f;
float kwpClutchDuty = 0.0f;
float kwpClutchValveCurrent = 0.0f;


// LED
uint8_t ledBrightness = led_brightness_default;

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

uint32_t stackbroadcastOpenHaldex = 0;
uint32_t stackupdateLabels = 0;
uint32_t stackshowHaldexState = 0;
uint32_t stackwriteEEP = 0;

// internal variables
openhaldex_state_t state;
float lock_target = 0;

float lockReleaseRatePerSec = 120.0f;
bool lockReleaseEnabled = true;  // when false, lock target changes are instantaneous
bool steeringScaleEnabled = true; // when false, steering-angle lock scaling is bypassed (full lock)
uint8_t forceModesPriority = 0; // 0=Haz>TC>Ext, 1=TC>Haz>Ext, 2=Haz>Ext>TC, 3=TC>Ext>Haz, 4=Ext>TC>Haz, 5=Ext>Haz>TC

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

// Steering-angle lock-scale curve: angle breakpoints (deg) -> lock multiplier (%).
uint16_t steeringArray[steeringArrayCount] = {0, 45, 90, 180, 360};
uint8_t steeringLockScaleArray[steeringArrayCount] = {100, 100, 80, 50, 20};

// for running through vars to see effects
uint8_t tempCounter;

// Haldex learn table: index = correction factor (0-100%), value = observed engagement (0-100%)
uint8_t haldexLearnTable[101];
bool haldexLearnTableValid = false;
volatile bool haldexLearnActive = false;
volatile bool haldexLearnCancel = false;
volatile uint8_t haldexLearnStep = 0;   // 0-100 = current step, 101 = complete
volatile uint8_t haldexLearnCF = 0;     // current correction factor override during learn

// Long Learn state (see Calculations.h). Results persist in RAM until the next
// run so the UI can show / export them after completion.
volatile bool longLearnActive = false;
volatile bool longLearnCancel = false;
volatile uint8_t longLearnPhase = LL_IDLE;
volatile uint8_t longLearnSweepIdx = 0;
volatile uint8_t longLearnSweepTotal = 0;
volatile int16_t longLearnCurrentBit = -1;
uint8_t longLearnGenIdx = 0;
uint8_t longLearnGeneration = 0;
bool longLearnTestAll = false;
uint8_t longLearnBlockResult[64];
LearnScore longLearnBaseline;
LearnScore longLearnFinal;
bool longLearnBaselineValid = false;
bool longLearnFinalValid = false;
uint8_t longLearnFloorStart = 0;
uint8_t longLearnFloorResult = 0;
uint64_t longLearnMaskStart = 0;
uint16_t longLearnBpkStart = 0;
bool longLearnBpkAdjusted = false;
uint8_t longLearnFailReason = 0;
uint8_t longLearnNoise = 0;
uint8_t longLearnTol = 0;
bool longLearnInteraction = false;
LongLearnSweep longLearnSweeps[LL_MAX_SWEEPS];
uint8_t longLearnSweepCount = 0;
uint32_t longLearnStartMs = 0;
uint32_t longLearnEndMs = 0;
char longLearnNotes[LL_NOTES_LEN + 1] = "";
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

uint8_t BRAKES8_counter = 0x80;  // Bremse_8 byte1 rolling counter (0x80..0x8F)
uint8_t BRAKES8_counter1 = 0x00; // Bremse_8 byte2 rolling counter (0x00..0x0F)

uint8_t BRAKES9_counter = 0x03; // starting counter for Brakes9 is 11
uint8_t BRAKES9_counter2 = 0x00;

uint8_t BRAKES10_counter = 0;

uint8_t mLW_1_counter = 0; // was 0
uint8_t mLW_1_counter2 = 0x77;
uint8_t mLW_1_crc = 0;

uint8_t mDiagnose_1_counter = 0;

// PQ ACAN Gen5 0AY synthesized broadcasts
uint8_t mGetriebe_2_counter = 0; // 4-bit rolling counter (Zaehler_Getriebe_2) in data[0] bits 4..7

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
    0xac, 0xac, 0xac, 0xac, 0xac, 0xac, 0xac, 0xac,
    0xac, 0xac, 0xac, 0xac, 0xac, 0xac, 0xac, 0xac};

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

    // VW E2E profile: CRC-8 (0x2F) over B1..B7 with the counter-indexed
    // DataID byte APPENDED last. Verified 2026-09-17 against a real MQB
    // capture: 4,400+ frames on 13 IDs (086 0A7 0A8 0AD 0FD 101 104 106 116
    // 121 392 3BE 641 65D) match 100 % with the DataID last and 0 % with it
    // first. The Gen5 Haldex never rejected the old (wrong) checksum; the
    // VAQ does check at least some frames.
    for (uint8_t i = 1; i < 8; i++)
    {
        crcInput[i - 1] = frame[i]; // B1..B7
    }
    crcInput[7] = idSeq[counter]; // DataID byte last

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

uint8_t Gen41_1CE234_counter = 0;

// Gen42 (Ford) standalone rolling counters
uint8_t gen42_main_counter  = 0x51; // 8-bit: shared by 0x080 D7, 0x20F D8, 0x211 D8
uint8_t gen42_090_nibble    = 0x00; // 4-bit (0-15): 0x090 D1 high nibble
uint8_t gen42_20F_d6        = 0x00; // 0x20F D6 (+0x10 per frame, mod 256)
uint8_t gen42_190_counter   = 0x00; // 4-bit (0-15): 0x190 D6
uint8_t gen42_275_counter   = 0x80; // 0x275 D1 (+0x20 per 100 ms, mod 256)

// Volvo / SAAB standalone counters and heartbeats (V10.00.0)
bool volvoHeartbeatFlip = false; // Volvo 100 ms network frame: false = send the first byte 6-7 pair, true = its opposite
uint8_t saab140Counter = 0;      // SAAB 0x140 byte 0 bits 0-1, counts 0-3
uint8_t saab160Step = 0;         // SAAB 0x160 bytes 2/3 pair to send next, 0-3

// Volvo / SAAB Haldex feedback (V10.00.0)
int16_t received_haldex_temp_c = -128; // Haldex temperature in degC (SAAB 0x210 b2 - 40, VAG Gen4/0AY 0x2C0 b4 - 60); -128 = not received yet

// CAN bus speed the two controllers were started at (set in setupCAN from the generation)
uint16_t canBusSpeedKbps = 500;

// True for the Volvo and SAAB generations added in V10.00.0. They share a lot:
// no VW chassis decode, no VW checksums, linear lock scaling until learned.
bool isVolvoOrSaabGeneration(uint8_t generation)
{
  if (generation == GEN_VOLVO_P2_GEN2) return true; // Volvo P2 Gen2
  if (generation == GEN_VOLVO_P2_GEN3) return true; // Volvo P2 Gen3
  if (generation == GEN_VOLVO_GEN4) return true;    // Volvo Gen4
  if (generation == GEN_SAAB_93_XWD) return true;   // SAAB 9-3 XWD
  if (generation == GEN_VOLVO_SPC_GEN5) return true; // Volvo Gen5 SPC 31380867 (V11.00.0)
  if (generation == GEN_VOLVO_P1) return true;       // Volvo P1 S40 / V50 AWD (V11.00.1)
  return false;
}

// 0x1CE Secondary Axle Torque Request (Nm). 0.0 = no request (open clutch).
// Set from the API/mode handler. Encoding: 4 Nm/LSB (empirical; 0xFF = 1020 Nm max).
float g_1ce_torque_request_nm = 0.0f;