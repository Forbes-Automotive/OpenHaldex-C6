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

#include <OpenHaldexC6_led.h>              // for RGB LED (single WS2812, replaces Freenove lib - see header)
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
#define debugMemory 1               // set to 1 to print heap + every task's unused stack 15 s after boot, then every 60 s

// refresh rates
#define eepRefresh 2000           // EEPROM save in ms
#define broadcastRefresh 200      // broadcast data over CAN refresh rate in ms
#define BROADCAST_HALDEX_QUIET_MS 5000 // stop the 0x6B0 broadcast once the Haldex side has been silent this long
#define serialMonitorRefresh 1000 // Serial Monitor refresh rate in ms
#define updateTriggersRefresh 500 // change IO refresh rate in ms

// debugging macros
// "#if", not "#ifdef": enableDebug is always defined (as 0 or 1), so "#ifdef" was
// always true and every DEBUG() message was built in and printed even with it at 0
// (fixed in V11.00.1 - saves flash and stops the constant serial output)
#if enableDebug
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

#define gpioLed 8       // gpio for led
#define gpioMode 19     // gpio mode button internal
#define gpioModeExt 18 // gpio mode button external

#define gpioHbIn 14    // gpio for handbrake signal in
#define gpioHbOut 15   // gpio for handbrake signal out
#define gpioBrakeIn 0  // gpio for brake signal in
#define gpioBrakeOut 1 // gpio for brake signal out

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
#define ledChannel 0              // channel for led
#define ledBrightnessDefault 255 // compile-time default
extern uint8_t ledBrightness;      // runtime LED brightness (0–255, persisted)

// wifi settings
#define wifiHostNameDefault "OpenHaldex-C6" // factory default AP SSID
#define wifiHostName wifiSsid              // legacy alias - all call sites now use runtime SSID
extern char wifiSsid[33];                  // runtime AP SSID (max 32 chars + NUL)

// WiFi bridge mode (PR #39, louij2): optionally ALSO join a home/garage network
// as a station (WIFI_AP_STA) so the controller is reachable over that LAN
// without anyone leaving their normal WiFi. The AP keeps running regardless.
// Empty SSID = disabled = the AP-only behaviour every install has today.
extern char wifiStaSsid[33];     // home network SSID; empty = bridge mode off
extern char wifiStaPassword[65]; // home network password; empty = open network
extern bool wifiStaConnected;    // runtime: associated and holding an IP
extern char wifiStaIP[16];       // runtime: dotted quad once connected, "" otherwise

extern twai_handle_t twaiBus0; // for ESP32-C6 CANBUS 0
extern twai_handle_t twaiBus1; // for ESP32-C6 CANBUS 1

extern twai_message_t rxMessageHdx; // incoming haldex message
extern twai_message_t rxMessageChs; // incoming chassis message

extern twai_message_t txMessageHdx; // outgoing haldex message

extern TaskHandle_t handleStandaloneFrames;     // our-frames scheduler (both modes; sends while ownFramesActive())
extern TaskHandle_t handleBroadcastOpenHaldex;  // OpenHaldex CAN broadcast task (suspended in aggressive sleep)
extern TaskHandle_t handleShowHaldexState;      // serial state logger (suspended in aggressive sleep)
extern TaskHandle_t handleUpdateTriggers;       // notified by CAN_RX wake ISRs in aggressive sleep

// setup - main inputs

// for EEP
extern Preferences pref; // for EEPROM / storing settings

// for LED
extern OneLed strip; // 1 led, gpio pin, colour order

// for mode changing (buttons & external inputs)
extern InterruptButton btnMode;     // pin, GPIO_MODE_INPUT, state when pressed, long press, autorepeat, double-click, debounce
extern InterruptButton btnModeExt; // pin, GPIO_MODE_INPUT, state when pressed, long press, autorepeat, double-click, debounce

extern AsyncWebServer webServer;

// functions
void standaloneFramesTask(void *arg);

void parseCANChs(void *arg);
void parseCANHdx(void *arg);
void broadcastOpenHaldex(void *arg);
void showHaldexState(void *arg);

void setupButtons();
void setupCAN();
void canBusRecovery();
void setupTasks();

void updateTriggers(void *arg);
void modeChange();
void modeChangeExt();

// Mode / controller changes requested from the web UI or BLE (shared so both
// apply the same rules). requestMode() returns false when the request was
// rejected (out of range, or the controller is disabled).
bool requestMode(uint8_t mode);
void setControllerDisabled(bool disabled);
void setStandaloneMode(bool on); // V11.00.12: standalone on while in Stock -> FWD

extern uint8_t getLockTargetAdjustedValue(uint8_t value, bool invert);

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
enum openHaldexMode_t
{
  MODE_STOCK,
  MODE_FWD,
  MODE_5050,
  MODE_6040,
  MODE_7525,
  MODE_EXPERT,
  openHaldexMode_t_MAX // for validation
};

struct openHaldexState_t
{
  openHaldexMode_t mode;
  uint8_t pedalThreshold;
  bool modeOverride;
};

// Values received from Haldex CAN
extern uint8_t receivedHaldexState;
extern uint8_t receivedHaldexEngagementRaw;
extern uint8_t receivedHaldexEngagement;
extern uint8_t appliedTorque;

// Gen41 Haldex feedback (decoded from FDCM-originated frames per GMW8762 PPEI).
// 0x1CC Secondary Axle Status (Bus1, DLC=3):
//   D0 status byte: bit 0x10 = secondary axle engagement active flag,
//                   bit 0x08 = FDCM operational/healthy (always set when alive),
//                   bits[1:0] = 2-bit Alive Rolling Counter
//   D1 = applied secondary axle torque feedback (Nm, 1 Nm/LSB) - tracks 0x1CE D1 request
//   D2 = clutch sub-state (1=idle, 3=arming, 4=ramping, 5=engaged, 6=holding, 7=ramping-down)
extern uint8_t receivedSecAxleStatusRaw;   // 0x1CC D0
extern uint8_t receivedSecAxleTorqueNm;    // 0x1CC D1 - applied rear-axle torque (Nm)
extern uint8_t receivedSecAxleClutchState; // 0x1CC D2 - clutch sub-state machine
extern bool receivedSecAxleActive;          // 0x1CC D0 bit 0x10
extern bool receivedSecAxleFdcmHealthy;    // 0x1CC D0 bit 0x08
extern uint8_t receivedSecAxleArc;          // 0x1CC D0 bits[1:0]

// 0x1D1 Rear Axle Status (Bus1, DLC=4):
//   D0 = status flags (mostly 0 in healthy operation)
//   D1, D2 = rear-axle diagnostic metrics (vary with activity)
//   D3 = static framing byte (0x28 in all OEM captures)
extern uint8_t receivedRearAxleStatusFlags; // 0x1D1 D0
extern uint8_t receivedRearAxleMetricA;     // 0x1D1 D1
extern uint8_t receivedRearAxleMetricB;     // 0x1D1 D2

// 0x1CF / 0x331 Bus0 heartbeats (presence indicators only)
extern bool receivedHaldexAliveBus0;        // 0x1CF (D0=0x03) seen recently
extern bool receivedDrivetrainStateOk;      // 0x331 ([F8 FF]) seen recently
extern uint32_t receivedHaldexAliveBus0Ms; // millis() of last 0x1CF
extern uint32_t receivedDrivetrainStateMs;  // millis() of last 0x331

extern bool receivedReportClutch1;
extern bool receivedReportClutch2;
extern bool receivedTempProtection;
extern bool receivedCouplingOpen;
extern bool receivedSpeedLimit;
extern bool receivedLimpMode;
extern bool receivedAwdWarning;
extern uint8_t receivedLongLockState;
extern bool receivedKickdown;

// ===== Haldex health (V11.00.0) =====
// What the Haldex itself says about its own health, decoded per generation in
// parseCANHdx() from the OEM code findings (documents/haldex-binaries/
// CONTROL_AND_DTC_SPEC.md). These are the unit's GENUINE protections - we
// report them to the driver, we never hide them. Each generation only sets
// the ones its feedback frame really carries; the rest stay false.
//   receivedReportClutch1   = the unit has a fault present (clutch / pump / valve)
//   receivedReportClutch2   = signal plausibility fault (clutch / sensor)
//   receivedTempProtection  = over-temperature protection active (clutch pulled back)
//   receivedLimpMode        = limp / fault reaction active
//   receivedAwdWarning      = AWD warning lamp requested on the cluster
//   receivedCouplingOpen    = clutch commanded fully open (status only, not a fault)
extern bool receivedTempSensorFault; // the unit's own temperature sensor has failed
extern bool receivedFaultStored;      // the unit has stored a fault in its memory (scan it with VCDS / ODIS); on the 0AY (gen 51) 0x2C0 b3.7 is a self-test flag instead (audit_gen5_0ay #53/F1)
extern bool haldexHealthSupported;      // true when this generation's feedback carries health bits we decode
extern bool receivedLockRequestValid; // Gen5 0CQ: Allrad_03 (0x118) b1 bit 6 [KM ALR_Sendestatus] [REA audit_gen5_0cq A19/F3: 0 only for the "no ESP request" pattern Max {FA,FE,FF} + Min {FE,FF,00}; not proof the request is accepted]

// values received from Chassis CAN
extern float receivedPedalValue;
extern uint16_t receivedVehicleSpeed;
extern uint16_t receivedVehicleRpm;
extern bool receivedKl15;          // MQB ZAS_Kl_15 (ignition on), from Klemmen_Status_01
extern uint32_t lastKl15Ms;         // millis() of the last Klemmen_Status_01; 0 = never seen
bool ignitionOn();                  // KL15 on and fresh; false when unknown
extern uint16_t receivedVehicleBoost;
extern float receivedCoolantTemp;   // engine coolant temperature in degC, from PQ Kombi_2 (0x420) byte 4 (V11.00.4)
extern float receivedOilTemp;       // engine oil temperature in degC, from PQ Kombi_2 (0x420) byte 3 (V11.00.4)
extern uint32_t lastKombi2Ms;       // millis() of the last Kombi_2 frame; 0 = never seen
#define KOMBI2_TIMEOUT_MS 2000      // no Kombi_2 for this long = temperatures unknown, shown as "--"
extern uint8_t haldexGeneration;

// ===== Volvo / SAAB Haldex generations (added V10.00.0) =====
// Our numbering: first digit = Haldex generation, second = which car (as 41 = GM, 42 = Ford).
#define GEN_VOLVO_P2_GEN2 21 // Volvo P2 pre-facelift (S60/V70/XC70/XC90 to ~2004), Haldex Gen2, CAN at 250 kbit/s [REA none: no gen-21 dump; 250k = 2003 car log + a bootloader inference only, audit_volvo_c166 #24/#85]
#define GEN_VOLVO_P2_GEN3 3  // Volvo P2 facelift (S60/S60R/V70/XC70/XC90 2005+), Haldex Gen3, CAN at 500 kbit/s [REA audit_volvo_c166 #19-21: S60R C1BTR 0x1641 @ 20 MHz]
#define GEN_VOLVO_GEN4 43    // Volvo Gen4 (P3 XC60/V70/S80), 29-bit IDs (V11.00.5 bench: same message set as the P2 facelift DEM) [bench; no P3 dump - only the S60R field layouts + D2 IDs back it]
#define GEN_SAAB_93_XWD 44   // SAAB 9-3 XWD (GM family), Haldex Gen4
#define GEN_VOLVO_SPC_GEN5 53 // Volvo Haldex Gen5 "SPC" unit 31380867 (PowerPC, 11-bit IDs, 500 kbit/s assumed) - added V11.00.0 from its OEM code [REA audit_ppc_spc_bmw #29: 500k confirmed, FlexCAN CTRL 0x005A0047]
#define GEN_VOLVO_P1 31       // Volvo P1 (S40 / V50 AWD 2004-2012), Haldex Gen3, 500 kbit/s, 29-bit IDs - added V11.00.1 from Adam's 2005 V50 T5 AWD logs [REA none: no P1 dump; logs only]
#define GEN_FORD_KUGA 45      // Ford Kuga Mk1 (C1 platform) AWD module 8V41-14C251 / 9V4N, 500 kbit/s, 11-bit - added V11.00.9
                              // from the 2026-11-18/19 bench + the unit's own firmware (leaf_trace/trackF_kuga_8v41.md)
#define GEN_LANDROVER_GEN4 46 // Land Rover Haldex Gen4: Freelander 2 BH52-4C045-AC AND Evoque BJ32-4C045-AC (one generation for
                              // both - we don't identify the controller: the union of both receive sets is sent, whichever
                              // feedback arrives is decoded), 500 kbit/s, 11-bit - added V11.00.13. UNTESTED: no car log, built
                              // from the binary trace (leaf_trace/audit_landrover.md over trackG_landrover.md)
                              // [REA audit_landrover #4/#5/E1: 500k proven from the unit's own bootloader (FL2 BTR 0x3E81, Evoque
                              //  CAN1 0x1643 = still 500k), every ID 11-bit DLC 8]

// Land Rover Gen4 (generation 46) lock request + feedback (V11.00.13, from the binary trace - no bench yet)
// Lock request = the ABS / DSC "direct torque request", field 4 of the unit's ABS request decode - the normal
// path the car's DSC uses, NOT the UDS DID 0xFD00 end-of-line override [REA audit_landrover #29/#49/#55]:
//   FL2: mode 0x20A b6 bits 5-7 = 4 with qualifier 0x20A b2 bits 4-6 in {2,3}; torque T = 0x20A b6[4:0]:b7
//   EVQ: mode 0x2B5 b1 bits 2-4 = 4 with qualifier 0x250 b0 bits 0-2 in {2,3}; torque T = 0x20A b4[4:0]:b5
//   -> F157B = 1, F158C = 2, F15A0 = T << 4 (16-bit shift: T bit 12 is lost, so T must stay <= 0x0FFF)
//   -> mode manager mode 3 (ABS request beats everything but a fault, mode 6) -> FUN_03fc18 field 4 (runs
//      with r12 = 0 AND 1, so in both strategies FUN_040000 / FUN_03f930): F257E = min(F15A0, F1548)
//   -> output FUN_041e22: commanded torque F2612 = min(F257E, F2608), F2608 = the limit that ramps up to
//      F1548 while the base demand F25FE is enabled and down to 0 otherwise.
//   F1548 (= F260A) = min(ramp to cal 0x6532 = 24000 [0x653A = 14400 when F1547 = -1, i.e. 0x160 b1 bit 4 set],
//      FUN_044fb8(cal 0x62A8 = 5371 pressure) = about 20300..>24000 by temperature / speed tables) - same cals
//      on the Evoque (0x63BE / 0x63C6 / 0x613E). So 100 % = T 1500 (24000 >> 4); above that nothing changes.
// Gates the strategy needs for F25FE (FUN_0424a6) [REA audit_landrover #22/#53]: rpm F1564 > 2688 internal
// (> ~401 rpm on 0x12A / EVQ 0x100), F158E = 0 (0x378 b7 bit 3 / EVQ 0x143 b3 bit 7 clear, + 70 cycles),
// F2606 = 0 and F260C = 0 (the unit's own temperature limits - nothing on CAN), F1570 = 0 (only while the
// unit's operating state E682 == 1 = normal, set by FUN_01c0a0 when no monitor asks for a fault reaction -
// jump table 0x26244; i.e. a complete, plausible receive set) [REA: E682 state meaning INF].
#define LR_RPM_RUNNING 800       // 0x12A / EVQ 0x100 engine rpm (1 rpm/bit, DID F40C) while lock is asked for (standalone; gate > ~401)
// Feedback: FL2 0x310 / EVQ 0x301 b6[4:0]:b7 = F2612 >> 4 = the COMMANDED clutch torque (not measured - no
// pressure is on CAN) in the same unit as T, so lock % = feedback / LR_REQUEST_FULL in both modes
// [REA audit_landrover #44: F2612 -> F15AE (0x2C6E8) -> >>4 -> sig 0x46].

// Gen2 (7H0 / 1K0) lock request + read-back range (V11.00.13, audit_gen2_7h0): Bremse_4 b0 and 0x2C0 b1 share the
// (byte - 0x80) x 28 unit; full valve from 0xD6 (7H0) / 0xEA (1K0). No controller identification, so the
// range is the 1K0's: 0x80..0xEA = 0..100 % in both directions (lock request 1-99 % = 0x81..0xE9, 100 % = 0xFD).

// Ford Kuga (generation 45) lock request + feedback (V11.00.11: the unit's own torque strategy, no 0x6F1)
// The module's normal control (trackF "Strategy enable" / "Normal-path lock request"): base demand needs
// 0x201 rpm > ~401 and its pressure-ready latch; pump demand needs 0x201 b6 (pedal) > 5; torque demand
// = 0x205 engine torque x gear ratio x accel / speed factors, then CAPPED by the ABS / ESP torque
// ceiling 0x211 b6 field 3 (bits 1-2 = 3) with b2[3:0]:b3 = C (demand <= 8 x C). It turns the demand
// into a pressure target, runs the pump from target - measured and closes the loop on the valve.
// Lock request = that ceiling: C = lock % x KUGA_REQUEST_FULL, with engine torque high enough that the
// ceiling (not the torque) sets the demand - exactly how the car's ESP limits the AWD torque.
#define KUGA_RPM_RUNNING 800       // 0x201 engine rpm while lock is asked for (standalone; the pump gate is > ~401)
#define KUGA_IDLE_TORQUE_NM 25     // 0x205 engine torque with no lock asked for (Focus Mk2 idle value in the trace)
#define KUGA_DRIVE_TORQUE_NM 7000  // 0x205 engine torque while lock is asked for (13-bit field, max 7679 Nm). REA (audit_ford_kuga A / #92):
                                   // above ~620 Nm the unit's acceleration estimate F1692 overflows and the strategy factor drops to its
                                   // minimum 820, so parked the torque demand tops out at F16E4 = 10251 = C 1281 for ANY torque > ~620 Nm
                                   // (500 Nm -> C 689, 100 Nm -> C 461). 7000 keeps the demand at that top; the ceiling below then sets it.
#define KUGA_PEDAL_ACTIVE 0x50     // 0x201 b6 (0.5 %/bit) = 40 % while lock is asked for: > 5 enables the strategy pump demand (F156B)
// Feedback: 0x260 byte 2 bits 0-3 : byte 3 = clutch torque from the MEASURED pressure / 8 (trackF) -
// the same unit as the ceiling C, so lock % = feedback / KUGA_REQUEST_FULL.
// [REA audit_ford_kuga #13/#54/#61, FI-3: 8V41 only and not pure: +2 counts once the pressure-ready latch F1048
//  is set, oil-temperature / slip factors (sensor full scale reads 2438 at -40 C, 2294 at 8 C, 1958 at 120 C),
//  0x0FF = pressure-sensor / monitor-2 fault marker; 9V4N sends its torque REQUEST (F13EE>>3), not a measurement]
bool isVolvoOrSaabGeneration(uint8_t generation); // true for any of the six above
bool isSupportedGeneration(uint8_t generation);   // can be picked from the web UI / ESP-NOW (one list, V11.00.8)

// =============================================================================
// Lock request + feedback scaling - every generation in one place (V11.00.14)
// =============================================================================
// The same names for every unit:
//   <GEN>_REQUEST_FULL       lock request value we send for 100 %
//   <GEN>_REQUEST_STEP       unit's internal units per step of that value (only where it scales it)
//   <GEN>_REQUEST_FULL_BYTE  byte sent at exactly 100 % (where 100 % is a special code)
//   <GEN>_FEEDBACK_FULL      feedback value read back as 100 % engagement
// (+ _OPEN / _ZERO / _MAX where a unit has them). Where the request and the feedback share one
// unit, _FEEDBACK_FULL is defined from _REQUEST_FULL so the two can't drift apart.
// The why of every number (KMatrix / REA refs) is next to it below; the frame layouts are in canID.h.

// ----- Gen1 02D (1) - Bremse_4 byte 0 = a CEILING on the unit's own request: (b0 - 0x80) x STEP
#define GEN1_REQUEST_FULL 0x3700     // biggest request the 02D makes in normal driving (OEM code: the E2A0 reset value) [REA audit_gen1_02d #47: E2A0 only 0x3700 or lower; valve 785 of 791]
#define GEN1_REQUEST_STEP 192      // one Bremse_4 byte-0 step = 192 request units (OEM code) [REA audit_gen1_02d #57: 3 x b0 x 64 - 0x6000]
#define GEN1_REQUEST_FULL_BYTE 0xFD   // 100 %: ceiling 24000, well above 0x3700, so no limit (0xFE/0xFF have no special meaning on the 02D - audit F1 - but avoided as on other PQ units)
#define GEN1_FEEDBACK_FULL 198 // 0x2C0 b1 at 100 % = 0xC6: MEASURED full lock (bench + car 2026-11-27). V11.00.14 (2026-11-27): back from 201 (0xC9), which
                               // was only modelled (0x80 + 0x3700/192, audit F9/#66, round-trip error 3-9 %) - full lock read 95 % and Learn never reached 100

// ----- Gen2 7H0 / 1K0 (2) - Bremse_4 byte 0, (b0 - 0x80) x 28; one range for both units (the 1K0's)
#define GEN2_REQUEST_FULL 106          // lock request: step = lock % x 106 / 100 -> 99 % = 0xE9 [KM Bremse_4.ABS_Vorgabewert_mitte_Kupplungs] [REA audit_gen2_7h0 #55 §5: 1K0 full > 0xE9 (106 steps), 7H0 > 0xD5 (85)]
#define GEN2_REQUEST_FULL_BYTE 0xFD // 100 %: full valve on both 7H0 and 1K0 [REA audit_gen2_7h0 #55]
#define GEN2_FEEDBACK_FULL 106 // read-back: % = (0x2C0 b1 - 0x80) x 100 / 106, clamped to 100 [KM Allrad_1.Kupplungssteifigkeit_Mitte__Ist] [REA audit_gen2_7h0 #57: top ~0xEC 1K0 / ~0xD7 7H0 -> a 7H0 at full lock reads ~82 %]

// ----- Gen4 0BR (4) - Bremse_4 byte 0, (b0 - 0x80) x 112; 1-99 % = 0x81..0xFC
#define GEN4_REQUEST_FULL 125      // steps above 0x80 for 100 % (1-99 % -> 1..124) [REA audit_gen4_0br #14/#21]
#define GEN4_REQUEST_FULL_BYTE 0xFE // 100 %: 'mode 4' = the unit's ramped capacity, always >= 0xFD [REA audit_gen4_0br §8 / #43]
#define GEN4_FEEDBACK_FULL 125     // 0x2C0 b1 - 0x80 at 100 % (0xFE = at full capacity) [REA audit_gen4_0br #22]

// ----- Gen4.1 GM FDCM (41) - 0x1CE secondary axle torque request in Nm
#define GEN41_REQUEST_STEP 4       // Nm per bit of 0x1CE byte 1 (empirical; 0xFF = 1020 Nm)
#define GEN41_FEEDBACK_FULL 200    // 0x1CC byte 1 Nm read as 100 % (captured OEM peak ~150 Nm)

// ----- Gen5 0CQ (50) / VAQ (52) - ESP_14 request byte, 0.4 %/step; feedback the same scale
#define GEN50_REQUEST_FULL 250     // ESP_14 value at 100 % (0xFA); above it the unit ignores ESP_14 [REA audit_gen5_0cq S13, spec D1]
#define GEN50_FEEDBACK_FULL GEN50_REQUEST_FULL // Allrad_03 b2 ALR_Ist_Proz 0..250 [KM 0.4 %] [REA audit A23]
#define GEN52_FEEDBACK_FULL GEN50_REQUEST_FULL // Quersperre_03 b2 QUER_Ist_Proz 0..250 [KM 0.4 %] [REA audit §4: scale not in the dump - bench]

// ----- Gen5 0AY (51) - a PQ unit: Bremse_4 byte 0 like Gen4, but 100 % = 0xFD (0xFE = HOLD on the 0AY)
#define GEN51_REQUEST_FULL 125      // steps above 0x80 for 100 % [REA audit_gen5_0ay #11/#12]
#define GEN51_REQUEST_FULL_BYTE 0xFD // 100 %: top request 14000 [REA audit_gen5_0ay #20/#23]
#define GEN51_FEEDBACK_FULL 125     // 0x2C0 b1 - 0x80 at 100 % (0xFD / 0xFE read 100 %) [REA audit_gen5_0ay #26/#48]

// ----- Ford Kuga (45) - 0x211 ESP torque ceiling C; feedback 0x260 in the same unit
#define KUGA_REQUEST_FULL 1281     // 0x211 b2[3:0]:b3 at 100 % (V11.00.13, was 2294). REA model (audit_ford_kuga #92, d03_demand.txt):
                                   // the unit's own demand can't go above C 1281 parked, so a ceiling above it cut nothing and every lock
                                   // % above ~56 % did the same. 2294 is the 0x260 reading at sensor FULL SCALE (shorted sensor, bench
                                   // 2026-11-21), not a reachable demand. Bench check: 0x260 should plateau near 1281 for C >= 1281.
#define KUGA_REQUEST_OPEN 8        // REA (audit_ford_kuga): C >= 8 already asks for pressure; C < 8 = no demand. 1 % = C 13 is a real lock [#53; not used in code]
#define KUGA_FEEDBACK_ZERO 0             // trackF: 0 at zero measured torque (the old 0x0E3 was a stuck-sensor artefact) [REA #13: 2 once latched - reads 0 %]
#define KUGA_FEEDBACK_FULL KUGA_REQUEST_FULL // same scale as the lock request in both modes; fb above 1281 (up to sensor full scale) clamps to 100 %

// ----- Land Rover FL2 / Evoque (46) - ABS/DSC direct torque request T; feedback = commanded torque, same unit
#define LR_REQUEST_FULL 1500 // 100 % = T 1500 -> F15A0 24000 = the unit's own cap F1548 (cal 0x6532) [REA audit_landrover #52/#53]
#define LR_REQUEST_MAX 0x0FFF // field width after the unit's 16-bit << 4 (audit #29a) - never send more
#define LR_FEEDBACK_FULL LR_REQUEST_FULL

// ----- Volvo P2 pre-facelift (21) - feedback only (request is via the faked inputs)
#define VOLVO_P2G2_FEEDBACK_FULL 255 // status byte 6 read 0-255 = 0-100 % until a log gives the real scale [no OEM code]

// ----- Volvo P2 facelift (3)
#define VOLVO_P2G3_FEEDBACK_FULL 135 // byte 6 value read as 100 % engagement (V11.00.13, was 150) [KM -][REA audit_volvo_c166 #53/#74: byte 6 tops out at 135 = 0x87 (fixed zero 0x30D, 0 below ~0.9 V)]

// ----- Volvo P3 Gen4 (43) - request via rpm / wheel-speed inputs (VolvoGen4_lockRequestInputs)
#define VOLVO_G4_FEEDBACK_FULL 255 // byte 6 at full lock on the bench unit [BENCH][no OEM code; the S60R tops out at 135]

// ----- Volvo P1 S40 / V50 (31) - request via the faked pedal / torque inputs
#define VOLVO_P1_REQUEST_PEDAL_FULL 231       // ** CAN CHANGE THIS ** pedal byte at 100 % lock (full throttle in the pull log; 0 at 0 %)
#define VOLVO_P1_FEEDBACK_FULL 540   // ** CAN CHANGE THIS ** engagement count shown as 100 % (the ceiling in the full pull and the wheelspin logs)

// ----- Volvo SPC Gen5 (53) - 0x041 torque ceiling; feedback 0x126 setpoint
#define VOLVO_SPC_REQUEST_FULL 0x0C80   // ** CAN CHANGE THIS ** 0x041 bytes 6-7 at 100 % (V11.00.13, was 0x7FFF): just under the ~3222-count limit where the unit stops following the ceiling (audit_ppc_spc_bmw #39, variants 0-2). Bench: confirm lock still rises up to here
#define VOLVO_SPC_FEEDBACK_FULL 2033  // 0x126 bytes 0-1 count shown as 100 % (V11.00.13, was 6710 = 0xFFF x 1024 / 625): the setpoint our full ceiling allows - 0x0C80 x 1024 / 0xA4F x 16 = 19866 -> (>>4) x 1024 / 625 = 2033 [INF: assumes Ram40001030 is in the ceiling's unit]. Bench: set to the count at real full lock

// ----- SAAB 9-3 XWD (44) - feedback only
#define SAAB_93_FEEDBACK_FULL 255  // 11-bit status >> 3 read 0-255 = 0-100 % [no OEM code]
// =============================================================================

// ===== Volvo P1 standalone lock (V11.00.1) =====
// How the lock % is shown to the P1 AWD module in standalone. Set from the
// Settings page (Volvo P1 Standalone card), saved, and in backups.
#define VOLVO_P1_SOURCE_TORQUE 0 // engine torque + pedal rise with the lock (like a full-throttle pull - what the module follows in the logs)
#define VOLVO_P1_SOURCE_SLIP 1   // front wheels run faster than the rears as the lock rises (like wheelspin)
#define VOLVO_P1_SOURCE_BOTH 2   // both at once
extern uint8_t volvoP1LockSource;   // setting: one of the three above (default torque)
extern uint8_t volvoP1RoadSpeedKmh; // setting: road / rear wheel speed sent in standalone, km/h (default 30; the module never locked while stopped)
extern uint8_t volvoP1Gear;         // setting: gear the engine rpm is worked out for, 1-3 (default 2: 30 km/h = ~2000 rpm)
extern uint16_t volvoP1TorqueFull;  // setting: engine torque counts above zero sent at 100 % lock (default 600)
extern uint8_t volvoP1SlipFullKmh;  // setting: how much faster the front wheels run at 100 % lock, km/h (default 10)
// What standalone sent last and what came back - shown live on the settings card
extern uint16_t volvoP1SentTorque;      // engine torque counts above zero
extern uint8_t volvoP1SentPedal;        // pedal byte, 0-255
extern uint16_t volvoP1SentRpm;         // rpm
extern uint16_t volvoP1SentFrontRaw;    // front wheel speed, 0.01 km/h
extern uint16_t volvoP1SentRearRaw;     // rear wheel / road speed, 0.01 km/h
extern uint16_t volvoP1EngagementCount; // last engagement from 0x1960000A, 0-1023 (540 = full)

// CAN bus speed. Both controllers run at the same speed, picked from the Haldex
// generation at boot (setupCAN). Changing to/from a generation with a different
// speed saves the setting and restarts the controller (checkCanSpeedChange).
extern uint16_t canBusSpeedKbps;                     // speed the CAN controllers were started at (500 or 250)
uint16_t canSpeedForGeneration(uint8_t generation);  // 250 for Volvo P2 Gen2, otherwise 500
void checkCanSpeedChange();                          // called from loop(): restart if the generation needs a new speed
// Steering-wheel angle magnitude (deg, abs) decoded from chassis CAN + last-seen time.
extern float receivedSteeringAngle;
extern uint32_t receivedSteeringMs;
// Steering direction sign (true = negative/left, from the LWI_01/LW_1 sign bit) for slip geometry.
extern bool receivedSteeringNegative;
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
extern bool followABS; // setting: open the clutch while the ABS is braking (default on); off = our lock overrules the ABS
extern bool tcForceMode;
extern bool asrForceModeFlag;
extern bool tcForceModeFlag;

extern bool paddleTipActive;
extern bool paddleTipUp;
extern bool paddleTipDown;
extern bool paddleTipBoth;

extern bool extBtnForceMode;
extern bool extButtonForceModeFlag;

// ===== Drive mode select (MQB "Charisma") =====
// The car's drive mode select (Comfort / Normal / Sport / Race / Individual ...) can force an
// OpenHaldex mode. Profile numbers are the gateway's own (from the MQB K-matrix + jilleb/MQB-FPA).
#define DRIVE_PROFILE_COUNT 12     // drive profiles 0-11 (0 = none, 1 Comfort ... 11 Offroad Individual)
#define DRIVE_MODE_NOT_SET 255     // "N/A" in the drive mode table: this profile doesn't force a mode
#define DRIVE_MODE_TIMEOUT_MS 3000 // Charisma frames come once a second; after 3 s without one the drive mode is unknown
#define DRIVE_SOURCE_NONE 0        // no drive mode seen yet
#define DRIVE_SOURCE_CHARISMA_07 7 // read from Charisma_07 (0x3E8) CHA_Current_Mode: the profile itself
#define DRIVE_SOURCE_CHARISMA_01 1 // read from Charisma_01 (0x385) engine request: used when 0x3E8 isn't on this bus

extern bool driveModeForce;                                // setting: use the car's drive mode select to force a mode
extern uint8_t driveModeForceValue[DRIVE_PROFILE_COUNT];   // setting: OpenHaldex mode for each drive profile, or DRIVE_MODE_NOT_SET
extern uint8_t receivedDriveProfile;                       // runtime: drive profile the car is in (0 = none)
extern uint8_t driveProfileSource;                         // runtime: which frame it came from (DRIVE_SOURCE_...)
extern uint32_t lastDriveProfileMs;                        // runtime: when the drive profile was last read
extern uint32_t lastCharisma07Ms;                          // runtime: when Charisma_07 was last seen (0 = never)
extern uint8_t receivedHaldexModeRequest;                  // runtime: mode the car's gateway asks the Haldex for (Charisma_01 byte 3 bits 0-3)
extern uint8_t receivedVaqModeRequest;                     // runtime: mode the car's gateway asks the VAQ for (Charisma_01 byte 4 bits 4-7)
extern bool hasHaldexModeRequest;                          // runtime: true once a Charisma_01 frame has been read

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

// Danger Zone: at a full lock request (50:50, or Expert with lockTarget > 99),
// send Motor_11 in BPK packing with the torque ceiling raised to dangerZoneNm.
// The 0CQ's pump duty follows the engine torque this frame claims, so the pump
// goes past its normal ~60 % / 9-10 A and pegs the pressure relief valve.
// Bench 2026-09-20 (0CQ, BPK): 220 Nm -> 58 % / 9 A; 285 Nm pegs the PRV;
// 320 Nm -> 90 % / 11.0 A; 400 Nm -> 95 % / 11.5 A. Maximum clamping force,
// but the Haldex's REPORTED engagement reads LOWER (80s rather than ~98%)
// because its estimate backs off once the relief valve opens. ESP_14 Min/Max
// pinning (the original implementation) was measured to do nothing on the 0CQ.
// REA (audit_gen5_0cq A29 / spec D1): that bench ran with ESP_14 b7 = 0xFE, which
// the 0CQ treats as "no ESP request" - that is why the pump followed Motor_11.
// With the V11 lock request (Min = Max <= 250) the 0CQ follows ESP_14 and this torque
// only feeds its strategy / clutch-load model.
// Off by default; the toggle is persisted, the Nm value is a lab tunable.
extern bool dangerZoneEnabled;
extern uint16_t dangerZoneNm; // 320 - used only when it exceeds bpkCeilingNm

// ---- Serial lab byte overrides ----------------------------------------------
// Force an individual byte of any generated standalone frame, so each byte's
// effect can be measured one at a time. Applied in standaloneTx() just before
// transmit; the frame's checksum is recomputed afterwards for the IDs that
// carry one, so an overridden frame is still accepted by the Haldex.
// Test-only: not persisted, cleared on boot.
#define LAB_OVR_MAX 12
struct LabOverride
{
  uint32_t canId;  // 0 = slot unused (32-bit so Volvo's 29-bit IDs fit)
  uint8_t byteIdx; // 0..7
  uint8_t value;
};
extern LabOverride labOverrides[LAB_OVR_MAX];

// Serial-lab frame mutes (TXMUTE): drop individual generated standalone frames,
// or all of them, so a frame set can be bisected or replaced wholesale by TXADD
// frames (which go straight to the bus and are never muted). Test-only, not
// persisted.
#define LAB_MUTE_MAX 16
extern uint32_t labMuteIds[LAB_MUTE_MAX]; // 0 = slot unused
extern bool labMuteAll;

// ---- ESP_19 wheel-speed tunables --------------------------------------------
// Also runtime, to test whether the Haldex's hunting tracks the simulated wheel
// speed changing. Defaults reproduce the legacy free-running counter exactly.
extern bool wsFreeze;            // true = stop advancing the counters (static speed)
extern uint16_t wsBaseRaw;       // 0 = legacy counter behaviour; else fixed raw per corner
extern uint16_t wsDitherRaw;     // alternating +/- dither applied when wsBaseRaw > 0
extern int32_t wsFrontDeltaRaw;  // extra raw counts on the front axle (VL/VR); 0 = none
extern int32_t wsLeftRightDeltaRaw; // front left-vs-right split in raw counts (VL - VR); 0 = none. VAQ lock request.

// Per-car Gen5 BPK lock calibration: Nm the spoof frame claims at full command.
// Not a strength dial - shifts the calibration so commanded lock matches delivered.
extern uint16_t bpkCeilingNm;
// ESP_14 launch PWM floor (0-100%): raises BR_Vorg_*_Min while lock is commanded,
// clamped strictly below Max. 0 = off (upstream behaviour).
// NOTE: since V11.00.0 no ESP_14 builder reads this (Min is pinned to Max, the
// 0CQ exact mode); only the learn sweep zeroes / restores it.
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

// ---- Frame-edit blocks (one per CAN ID) -------------------------------------
// V11.00.7, both modes: a block ON = the Haldex gets OUR whole frame for that ID
// (the standalone builders) and the car's copy is held back; OFF = the car's own
// frame goes through untouched. Long Learn swaps them one at a time to find the
// frame the Haldex needs from us. Gated: gens 1/2/4/50/51/52, the Volvo P1 (31)
// and Volvo Gen4 (43). Gen41/42 are dual-bus and have no blocks.
enum
{
    FE_GEN_1 = 0,
    FE_GEN_2,
    FE_GEN_4,
    FE_GEN_50,
    FE_GEN_51,
    FE_GEN_SPARE_60, // V11.00.1: kept free - V12 puts Gen6 (Magma) here, so saved masks line up when moving to V12
    FE_GEN_VOLVO_P1, // V11.00.1: Volvo P1 (generation 31) standalone frames
    FE_GEN_VOLVO_GEN4, // V11.00.5: Volvo Gen4 P3 (generation 43) - standalone frames + OEM lock edits
    FE_GEN_FORD_KUGA,  // V11.00.9: Ford Kuga (generation 45) - standalone frames + OEM lock edits (V11.00.11: 0x211 ceiling lock request)
    FE_GEN_LANDROVER_GEN4, // V11.00.13: Land Rover FL2 / Evoque (generation 46). LAST on purpose: a save from an older version
                           // has one number fewer, so EEP.cpp loads every older slot unchanged and gives this one its default
    FE_GEN_COUNT
};

// Volvo Gen4 (FE_GEN_VOLVO_GEN4) block bits that are gated by hand: the two pages
// of 0x10A2407C share one CAN ID, so their blocks carry no ID (V11.00.5)
#define VOLVO_G4_BLOCK_NETWORK 9 // 0x10A2407C page 1 (heartbeat)
#define VOLVO_G4_BLOCK_CONFIG 11 // 0x10A2407C page 0x14 (configuration)

struct FrameEditBlock
{
    uint8_t genIdx;   // FE_GEN_*
    uint8_t bit;      // bit position within that generation's mask
    uint32_t canId;   // CAN identifier of the frame (32-bit since V11.00.1 so Volvo's 29-bit IDs fit)
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
extern const uint64_t frameEditLockDriven[FE_GEN_COUNT];     // blocks whose payload follows lockTarget (Long Learn core)
extern const FrameEditBlock frameEditBlocks[];            // descriptor table for UI/API
extern const uint16_t frameEditBlockCount;                // number of entries in frameEditBlocks[]
int frameEditGenIdx(uint8_t generation);                  // haldexGeneration -> FE_GEN_* (-1 if not gated)
bool frameEditEnabled(uint8_t genIdx, uint8_t bit);       // test a block's enable bit for the active mode
uint64_t *activeFrameEditMask();                          // mask for the current mode (isStandalone ? SA : normal)
void resetFrameEditMask();                                // restore defaults for both masks / all generations
bool frameEditKeepRequired();                             // switch back on the blocks a unit can't run without (true = changed)

extern bool canSleepEnabled;    // runtime toggle for CAN-wake light sleep (UI/EEP)
extern bool canSleepAggressive; // aggressive add-on: transceiver standby + DFS 10MHz + low WiFi TX power

// Bench mode (PR #39, louij2): suppresses the CAN-wake WiFi sleep above while
// bench testing over USB with no harness, so the AP doesn't drop mid-session.
// The board can't tell "harnessed to a sleeping car" from "no harness at all"
// (both are zero CAN traffic), so it's an explicit toggle - but it self-clears
// the moment either bus shows real traffic this power cycle, so a forgotten
// toggle can't weaken parked-car battery protection once installed. See
// everSawCANThisSession in OpenHaldexC6_IO.cpp.
extern bool benchMode;               // persisted (UI/EEP), default off

// Bluetooth LE link to the DashCAN mobile app (see OpenHaldexC6_BLE.cpp).
extern bool bleEnabled;      // persisted (UI/EEP), default on
extern uint32_t blePasskey;  // 6-digit pairing code, required once a phone is bonded; random, persisted, never in backups
extern volatile bool canWakeRequest; // set by CAN_RX GPIO ISR when transceivers in standby see bus activity

extern bool rebootWiFi;
extern bool lowPowerMode;
extern void *pmNoLightSleepLock; // esp_pm_lock_handle_t: held while awake so light sleep cannot power down the CAN controllers mid-drive
extern char wifiPassword[65]; // WiFi AP password - empty = open network

extern bool hazardForceMode;     // setting: use hazard lights to activate force mode
extern bool hazardForceModeFlag; // runtime: hazard lights are currently on

extern bool brakeActive;
extern bool brakeSignalActive;

extern bool handbrakeActive;
extern bool handbrakeSignalActive;

extern bool brakeFromCAN;
extern bool handbrakeFromCAN;
extern bool absBrakingFromCAN; // the ABS module says it is braking (PQ Bremse_1 b0.2 / MQB ESP_21 b7.0) - the Haldex opens the clutch for this
extern uint8_t kugaCar211[8];  // V11.00.13 Ford Kuga OEM: the car's last 0x211
extern bool kugaCar211Valid;    // V11.00.13: kugaCar211 holds a real car frame
extern uint16_t kugaCarTorqueCeiling; // V11.00.11 Ford Kuga OEM: the car's own ESP torque ceiling (0x211 field 3, b2[3:0]:b3); 0xFFFF = none
// V11.00.13 Land Rover Gen4 OEM: the car's last copy of each lock request frame - ours is built on it (only the lock request bits change)
extern uint8_t lrCar20A[8], lrCar2B5[8], lrCar250[8];
extern bool lrCar20AValid, lrCar2B5Valid, lrCar250Valid; // a car frame has been seen since the chassis bus woke
// Which feedback frame the Land Rover unit sends (set in parseCANHdx: 0x310 = FL2, 0x301 = Evoque). Used ONLY
// to pick the chassis-decode layout (rpm / speed / pedal for the dashboard and the lock map) - the frames we
// send are always the union of both, whatever this says.
#define LR_LAYOUT_UNKNOWN 0
#define LR_LAYOUT_FL2 1
#define LR_LAYOUT_EVQ 2
extern volatile uint8_t lrFeedbackLayout;

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

// One-shot sleep auto-setup (offered for Gen5): measure the parked chassis bus
// for 15 min, then wake threshold = average + SLEEP_CAL_MARGIN_FPS.
#define SLEEP_CAL_NONE 0     // never offered
#define SLEEP_CAL_ARMED 1    // waiting for the car to be left parked
#define SLEEP_CAL_DONE 2     // measured; threshold set
#define SLEEP_CAL_DECLINED 3 // user said no; don't ask again
#define SLEEP_CAL_WINDOW_S 900
#define SLEEP_CAL_MARGIN_FPS 300
extern uint8_t sleepCalState;    // persisted
extern uint16_t sleepCalAvgFps;  // parked-bus average measured by the auto-setup (persisted, for display)

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
extern QueueHandle_t udsRxQueue; // parseCANHdx pushes 0x779 frames here
// True while udsMQBTask owns the Haldex diagnostic channel (probe/session open
// through to loop exit). parseCANHdx uses it to keep the Haldex's 0x779 replies
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
extern volatile uint32_t udsManualLastMs; // millis() of the last web / serial-lab UDS request (0 = none); background pollers back off (V11.00.10)

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
// V11.00.12: the same census for Bus 0 (chassis), for the Diagnostics "CAN Test"
// card - so a tick can always be traced to the IDs that caused it. A car has more
// than 16 IDs; the rest are only counted (chsRxDroppedIds).
extern HdxRxStat chsRxStats[HDX_RX_STATS_MAX];
extern volatile uint32_t chsRxDroppedIds;
void canRxCensusClear();

// CAN loop test (V11.00.12): with CHS and HDX wired together, each bus sends one
// test frame and the other must receive it. Test frames never reach the parsers,
// the ticks or the bridge (no echo storm). Seen bits: LOOP_CHS_TO_HDX = Bus 1 got
// Bus 0's frame, LOOP_HDX_TO_CHS = Bus 0 got Bus 1's, LOOP_ECHO_* = a bus got its
// OWN frame back (should never happen - it would explain a tick with nothing wired).
#define CAN_LOOP_TEST_ID 0x1FFFFE5A // extended, unused by any unit we talk to
#define LOOP_CHS_TO_HDX 0x01
#define LOOP_HDX_TO_CHS 0x02
#define LOOP_ECHO_CHS 0x04
#define LOOP_ECHO_HDX 0x08
bool canLoopTestStart();              // false = car moving
uint8_t canLoopTestSeen();            // LOOP_* bits so far
uint32_t canLoopTestAgeMs();          // since the last start (0 = never run)
uint8_t canLoopTestSent();            // bit 0 / bit 1: the frame was accepted for transmit on Bus 0 / 1

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
extern uint8_t receivedQuerState;
extern uint8_t receivedQuerSync;


// Transmit-failure counters maintained by canTransmit() (see OpenHaldexC6_can.h).
extern volatile uint32_t canTxDropBus0;
extern volatile uint32_t canTxDropBus1;
// Gen5 UDS live data - meanings from the units' own firmware (V11.00.10, leaf_trace/diag_gen5_0cq.md,
// diag_gen5_0ay.md). No pressure, valve or oil-temperature DID exists on any Gen5.
extern float udsTerminalVoltage; // 0x0286: raw × 0.1 V - supply (ADC ch10)
extern float udsModuleTemp;      // 0x028D: byte 0 − 55 °C - control-unit temp SENSOR 1 (ADC ch15)
extern float udsClutchTemp;      // 0x2BF1: LE16 (raw − 22767)/100 °C - control-unit temp SENSOR 2 (ch13; VCDS "clutch")
extern float udsCoolingFinTemp;  // 0x2BE4: LE16 (raw − 22757)/100 °C - thermal MODEL
extern float udsClutchCurrent;   // 0x2BE6: BE16 mA − 5 → A - pump motor current, measured (mA unit inferred, audit_gen5_0ay #66/F2)
extern uint8_t udsClutchPWM;     // 0x2BE7: raw % (1 byte, 0–99) - pump PWM duty, commanded
extern float udsClutchVoltage;   // 0x2BE9: 1 byte × 0.1 V - pump motor voltage, CALCULATED (~back-EMF)
extern float udsClutchTorquePct; // 0x2BF3: 1 byte - clutch torque applied, % (scale per generation)
extern float udsAxleSpeedDiff;   // 0x2BF0: BE16 × 0.1 km/h - front/rear axle speed difference (not VAQ)
extern volatile uint32_t udsLastDecodeMs; // millis() of the last decoded UDS value (0 = none); freshness for BLE Diag

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
extern QueueHandle_t tp20RxQueue;        // parseCANHdx pushes TP2.0 diag frames here
extern volatile uint32_t kwpTp20EcuTxId; // negotiated ECU->tester data CAN id (0 until channel open)
extern bool kwpTp20Connected;            // TP2.0 channel + KWP session currently up
extern char kwpTp20RawDump[512];         // raw measuring-block capture (formula-id + a/b, no scaling yet)

// Gen4 (0BR / 0BS / 2K0 / 0AY907554C) scaled measuring values, decoded via VAG formulas
// confirmed from VCDS captures and the unit's own group table (leaf_trace/diag_gen4_0br.md).
extern float kwpOilTemp;         // G01[0] formula 0x1A: b - a  (°C) - oil temperature sensor
extern float kwpPlateTemp;       // G01[1] formula 0x1A: b - a  (°C) - plate temperature MODEL, no sensor
extern float kwpSupplyVoltage;   // G01[2] formula 0x06: 0.001*a*b  (V)
extern float kwpOilPressure;     // G03[0] formula 0x0E: 0.005*a*b  (bar) - pressure TARGET, not the sensor
extern float kwpEstTorque;       // G03[1] formula 0x5E: 0.1*a*(b-128)  (Nm) - commanded clutch torque (F1B08, 1/16 Nm; lock request step = 7 Nm, audit_gen4_0br #41)
extern float kwpClutchDuty;      // G03[2] formula 0x21: 0.01*a*b  (%) - valve PWM duty (command)
extern float kwpClutchValveCurrent; // G03[3] formula 0x18: 0.001*a*b  (A) - valve current, measured
extern float kwpPumpDuty;        // G04[2] formula 0x21: 0.01*a*b  (%) - pump motor duty (V11.00.10)
extern float kwpPumpCurrent;     // G04[3] formula 0x18: 0.001*a*b  (A) - pump motor current (V11.00.10)
// Gen2 (7H0 / 1K0) reuses kwpSupplyVoltage (G01[1]), kwpOilTemp (G02[0]) and kwpOilPressure
// (G02[1] - MEASURED on Gen2, unlike Gen4's target), plus (V11.00.10, leaf_trace/diag_gen2_7h0.md):
extern int8_t kwpBrakeSwitch;    // G01[0] text code: 1 = pressed (00A7), 0 = not (00C1), -1 = unknown
extern uint8_t kwpCanStatus;     // G7D bits 0-3 = engine / ABS / cluster / steering received OK
extern uint8_t kwpCanStatusValid; // which kwpCanStatus bits have been read
extern bool kwpGen2LoggedIn;     // SecurityAccess level 3 open -> G32/G33 polled: kwpPumpCurrent, kwpClutchDuty
                                 // (valve duty), kwpClutchValveCurrent (V11.00.10)
extern uint8_t kwpGen2LoginNrc;  // SAVED: NRC of a refused key (0 = none) - no more login tries until reset
extern uint32_t kwpGen2LoginSeed; // SAVED: the seed / key of that refusal (for checking the algorithm)
extern uint32_t kwpGen2LoginKey;
extern uint8_t kwpGen2LoginBlockedNrc; // last seed refusal (7F 27 36 / 37 = the unit's own lockout delay), 0 = none

// Ford Kuga (gen 45) live data (V11.00.10): Ford KWP2000 22 HH LL on 0x761 / 0x769 in the
// default session (fordDiagPoll in OpenHaldexC6_UDS.cpp). Variant is detected from the DID set.
enum
{
    FORD_DIAG_SUPPLY = 0x01,   // 0x0042 supply, mV
    FORD_DIAG_TEMP = 0x02,     // 8V41 0x1943 (s16, 0.1 degC) / 9V4N 0x19BA (s8, degC)
    FORD_DIAG_PMEAS = 0x04,    // measured clutch pressure: 8V41 0x1944 [REA #33] / 9V4N 0x1945 - WRONG: 9V4N 0x1945 = valve current, 9V4N has no pressure read-back [REA #63, audit D]
    FORD_DIAG_PCMD = 0x08,     // commanded pressure setpoint: 8V41 0x1947 / 9V4N 0xEE04 [REA #43: 8V41 0x1947 = valve command in F1058 units; 9V4N 0xEE04 = E554 setpoint]
    FORD_DIAG_VALVE = 0x10,    // 8V41 0x1945 valve (solenoid) current [REA #46; 9V4N 0x1945 is the same signal, polled as PMEAS]
    FORD_DIAG_PUMP = 0x20,     // 0x1946 pump motor current
};
extern uint8_t fordDiagVariant;     // 0 = not identified yet, 1 = 8V41 DID set, 2 = 9V4N DID set
extern bool fordDiagConnected;      // the module answered recently
extern uint8_t fordDiagValid;       // FORD_DIAG_* bits that hold a value
extern float fordSupplyV;           // V
extern float fordOilTemp;           // degC
extern uint16_t fordPressureRaw;    // measured pressure, raw DID value
extern uint16_t fordSetpointRaw;    // commanded setpoint, raw DID value
extern float fordPressureBar;       // 8V41 only, PROVISIONAL scale (see fordDiagPoll) [REA #6/#45: 0x1944 x 0.01953 bar is an assumption - no unit in the code; the older 0.05 bar/count (40 bar) is one too]
extern float fordSetpointBar;       // 8V41 only, PROVISIONAL scale [REA #43: 0x1947 / 1000 - valve command, same assumed scale]
extern float fordValveCurrentA;     // A (8V41)
extern float fordPumpCurrentA;      // A



extern volatile uint32_t lastCANChassisTick; // millis() of the last chassis frame, 0 = never
extern volatile uint32_t lastCANHaldexTick;  // millis() of the last Haldex frame, 0 = never
extern volatile uint32_t lpChassisFrameCount; // chassis CAN frame counter for probe window activity check
extern volatile uint32_t lpHaldexFrameCount;  // haldex CAN frame counter for standalone probe window activity check
extern uint32_t canHealthTimeoutMs;

extern uint8_t lastMode;
extern uint8_t disableThrottle;
extern uint16_t disengageUnderSpeed;
extern uint16_t disengageAboveSpeed;

// Which speed / throttle setting is holding lock at 0% (bits, can be several at
// once). Sent on the dashboard so the web page can say why, instead of a bare
// "Target: 0%". See lockGateBlocking() in OpenHaldexC6_Calculations.cpp.
#define LOCK_GATE_THROTTLE 0x01    // bit 0: throttle below "Minimum Throttle Before Lock"
#define LOCK_GATE_UNDER_SPEED 0x02 // bit 1: speed below "Disengage Under Speed"
#define LOCK_GATE_ABOVE_SPEED 0x04 // bit 2: speed above "Disengage Above Speed"
#define LOCK_GATE_HANDBRAKE 0x08   // bit 3: handbrake pulled with "Follow Handbrake" on (V11.00.0)
#define LOCK_GATE_ABS 0x10         // bit 4: ABS is braking with "Follow ABS" on - the Haldex's own safety release (V11.00.0)

extern uint32_t stackCHS;
extern uint32_t stackHDX;


extern uint32_t stackbroadcastOpenHaldex;
extern uint32_t stackshowHaldexState;
extern uint32_t stackwriteEEP;

// internal variables
extern openHaldexState_t state;
extern float lockTarget;

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

// Expert PT (pass-through) cells (V11.00.12): one bit per lockArray cell,
// bit = throttle * speedArrayCount + speed. A PT cell hands that area of the map
// back to the car's own Haldex strategy (same as Stock: car frames untouched).
// lockArray keeps the cell's number underneath, so clearing PT brings it back.
// Standalone has no car frames to pass, so PT there = 0 % lock.
#define LOCK_PT_CELLS (throttleArrayCount * speedArrayCount)
#define LOCK_PT_ALL ((1ULL << LOCK_PT_CELLS) - 1ULL)
extern uint64_t lockPTMask;
static inline bool lockCellPt(uint8_t throttle, uint8_t speed)
{
  return (lockPTMask >> (throttle * speedArrayCount + speed)) & 1ULL;
}

// Steering-angle third axis (FWD-bias): 1D scaling curve applied to the
// computed lock target. steeringArray holds steering-wheel angle breakpoints
// (deg, magnitude), steeringLockScaleArray the matching 0-100% lock multiplier.
// Only gens with a steering source (2/4/50/52) use it; others = no reduction.
#define steeringArrayCount 5   // 0, 45, 90, 180, 360 deg
#define steeringStaleMs 500    // steering considered stale/unhealthy after this (ms)
extern uint16_t steeringArray[steeringArrayCount];
extern uint8_t steeringLockScaleArray[steeringArrayCount];

// Throttle override for the steering-angle lock scale (rally use, V11.00.6).
// Below the start % the steering curve works as normal (turn-in agility).
// Between start and full it fades the steering reduction out in a straight line.
// At or above full % the steering reduction is ignored, so the requested lock
// holds through hard acceleration and counter-steering.
#define STEERING_THROTTLE_START_DEFAULT 40 // pedal % where the override starts (default)
#define STEERING_THROTTLE_FULL_DEFAULT 75  // pedal % where the override is complete (default)
extern bool steeringThrottleOverrideEnabled; // true = throttle can cancel the steering-angle reduction (default off)
extern uint8_t steeringThrottleOverrideStart; // pedal % where the override starts (0-99)
extern uint8_t steeringThrottleOverrideFull;  // pedal % where the steering reduction is fully cancelled (1-100, above start)

// for running through vars to see effects
extern uint8_t tempCounter;

// checksum values (for calculating module checksums in standalone mode)
extern uint8_t MOTOR5_counter; // starting counter for Motor5 is 0

extern uint8_t BRAKES1_counter;  // starting counter for Brakes1 is 10
extern uint8_t BRAKES2_counter;  // starting counter for Brakes2 is 3
extern uint8_t BRAKES4_counter;  // starting counter for Brakes4 is 0
extern uint8_t BRAKES4_crc;      // crc for Brakes4

extern uint8_t BRAKES5_counter;  // starting counter for Brakes5 is 0
extern uint8_t BRAKES5_counter2; // starting counter for Brake5 is 3

extern uint8_t BRAKES8_counter;  // Bremse_8 high-nibble rolling counter (0x80..0x8F)
extern uint8_t BRAKES8_counter1; // Bremse_8 low-nibble rolling counter (0x00..0x0F)

extern uint8_t BRAKES9_counter;  // starting counter for Brakes9 is 11
extern uint8_t BRAKES9_counter2; // starting counter for Brakes9 is 11

extern uint8_t BRAKES10_counter; // starting counter for Brakes10 is 0

extern uint8_t mLW_1_counter;  // was 0
extern uint8_t mLW_1_crc;      // crc for mLW_1

extern uint8_t mDiagnose_1_counter; // starting counter for mDiagnose_1 is 0

// PQ ACAN Gen5 0AY synthesized broadcasts
extern uint8_t mGetriebe_2_counter; // 4-bit rolling counter (Zaehler_Getriebe_2)

extern const uint8_t lws2[16][8]; // lookup table for calculating lws2 crc

extern uint8_t GETRIEBE_11_counter;           // starting counter for mDiagnose_1 is 0

extern uint8_t MOTOR_11_counter; // starting counter for Motor_11 is 0
extern uint8_t MOTOR_12_counter; // starting counter for Motor_12 is 0
extern uint8_t ESP_02_counter;   // starts at zero
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
extern uint8_t gen42MainCounter;   // 8-bit: shared by 0x080 D7, 0x20F D8, 0x211 D8
extern uint8_t gen42Frame090Nibble;     // 4-bit (0-15): 0x090 D1 high nibble
extern uint8_t gen42Frame20FD6;         // 0x20F D6 (+0x10 per frame, mod 256)
extern uint8_t gen42Frame190Counter;    // 4-bit (0-15): 0x190 D6
extern uint8_t gen42Frame275Counter;    // 0x275 D1 (+0x20 per 100 ms, mod 256)

// Volvo / SAAB standalone counters and heartbeats (V10.00.0)
extern bool volvoHeartbeatFlip;     // Volvo 100 ms network frame: bytes 6-7 swap to their opposite each send
extern uint8_t saab140Counter;      // SAAB 0x140 byte 0 bits 0-1: 2-bit rolling counter
extern uint8_t saab160Step;         // SAAB 0x160 bytes 2/3: which of the 4 counter/protection pairs is next

// Volvo / SAAB Haldex feedback (V10.00.0, provisional until logs confirm the scaling)
extern int16_t receivedHaldexTempC; // Haldex temperature in degC (SAAB 0x210 b2 - 40, VAG Gen4/0AY 0x2C0 b4 - 60); -128 = not received

// Secondary Axle Torque Request to encode into 0x1CE (Nm, 0 = no request).
// 4 Nm/LSB empirical scaling; set via API or mode handler.
extern float gen41TorqueRequestNm;

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
extern uint8_t crc8Autosar(uint8_t *data, uint8_t len);
extern uint8_t calcChecksum(uint8_t *frame, const uint8_t *idSeq);

// Convert a value of type openHaldexMode_t to a string.
// const MODE_NAMES = ['Stock', 'FWD', '50:50', '60:40', '75:25', 'Expert'];
inline const char *getOpenhaldexModeString(openHaldexMode_t mode)
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

// ---- Volvo Gen4 (P3) standalone constants (moved from StandaloneCAN.cpp, V11.00.7) ----
// The lock request's fixed inputs (VolvoGen4_lockRequestInputs / volvoGen4PutTorque).
// Tags: [bench] = 2026-11-07 P3 bench; [no OEM code] = no P3 dump exists; "S60R" = the P2 facelift DEM dump
// (audit_volvo_c166), a different unit whose field layouts the P3 shares.
#define VOLVO_G4_RPM_RUNNING 900   // engine rpm sent whenever lock is wanted (bench: 300 and up = pump on) [bench][no OEM code; S60R ready gate is ~401 on / ~200 off, #34/#56]
#define VOLVO_G4_CRUISE_SPEED 4000 // wheel-speed value for 1-99 % (bench: lowest steady pre-load, ~0x36) [bench; 4000 x 1/64 = 62.5 km/h]
#define VOLVO_G4_SLIP_MAX 600      // most slip ever sent - the unit opens the clutch at ~650 (excess-slip protection) [bench][no OEM code]
#define VOLVO_G4_TORQUE_RAW 0 // what the lock request was bench-measured with (0 = far below the car's idle 512) [bench; S60R s09 terms: C030 = 4 x 0 - 2048 = -2048]
#define VOLVO_G4_PEDAL_RAW 0  // foot off [bench; S60R s01: C038 = (2 x 0 + 5) / 10 = 0]
