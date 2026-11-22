#include <OpenHaldexC6_StandaloneCAN.h>
#include <OpenHaldexC6_Calculations.h>

// The frames built here are "ours" (V11.00.7, both modes): a frame-edit block
// ON sends our whole frame and can.cpp holds the car's copy of that ID back;
// OFF sends nothing here and the car's own frame (if it sends one) goes
// through untouched. Frames that are not listed as blocks for the active
// generation always send. Every Haldex-bus transmit goes through
// standaloneTx() instead of calling twai_transmit_v2 directly.

// =============================================================================
// Volvo / SAAB standalone tables and state (V11.00.7: moved up from the middle
// of each vehicle's section). Scalar settings for these builders are in
// OpenHaldexC6_defs.h, their CAN IDs in OpenHaldexC6_canID.h.
// 'static' = private to this file.
// =============================================================================

// ---- Statics -------------------------------------------------------------------

// ---- Volvo P2 Gen2 ----
// Static frames: starting values, byte meanings not known yet
// V11.00.13 per-byte tags: NO binary backing and no KMatrix for any gen-21 byte - every byte of every
// table below is [KM -][REA none] (no gen-21 ID in any dump, audit_volvo_c166 #86). [LOG] =
// documents/VOLVO_GEN2_VS_GEN4_LOGS_2026-11-11.md:
//   0x00224024  b0-b7  capture replay, meaning unknown
//   0x00C24000  b0[7:6] = 1, b0[5:0]:b1 = 0  | b2[7:6] = 0, b2[5:0]:b3 = 0     [LOG: 14-bit wheel speeds +
//               b4[7:6] = 3, b4[5:0]:b5 = 322 | b6[7:6] = 3, b6[5:0]:b7 = 322   2-bit qualifiers, as 0x10004002]
//               -> a FIXED front-faster slip (~5 km/h at 1/64), sent whatever the lock %
//   0x01E2402C  b0-b7  immobiliser response CEM -> ECM [LOG], not for the Haldex
//   0x03200002 / 0x03400002  b0-b7  capture replays, meaning unknown
static const uint8_t VOLVO_P2G2_224024_DATA[8] = {0xE9, 0xC9, 0x00, 0x1F, 0x01, 0xFF, 0x08, 0x00};  // 0x00224024
static const uint8_t VOLVO_P2G2_C24000_DATA[8] = {0x40, 0x00, 0x00, 0x00, 0xC1, 0x42, 0xC1, 0x42};  // 0x00C24000
static const uint8_t VOLVO_P2G2_1E2402C_DATA[8] = {0xC4, 0xF9, 0x20, 0xC6, 0x44, 0xF6, 0xAE, 0x4D}; // 0x01E2402C
static const uint8_t VOLVO_P2G2_3200002_DATA[8] = {0x04, 0xC3, 0x62, 0xAE, 0x01, 0xF4, 0x01, 0xF4}; // 0x03200002
static const uint8_t VOLVO_P2G2_3400002_DATA[8] = {0x01, 0x00, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00}; // 0x03400002

// ---- Volvo P2 Gen3 (the counters / toggle are shared with Volvo Gen4) ----
// Static frames: starting values, byte meanings not known yet
// V11.00.13 per-byte tags. [KM -] for every byte (no KMatrix for this 29-bit set). [REA sNN] = S60R DEM
// signal-table entry (audit_volvo_c166 a03_lanes): ctl = used for control, mon = signal-error table only,
// nr = defined, referenced nowhere; "-" = no signal on that byte. For gen 43 the same tables are sent, but
// the S60R is a different unit: there these tags are LAYOUT only [no OEM code for the P3].
//   0x0042406C  b0 counter bits 6-7 [REA #47: RX drops a frame whose byte 0 repeats - required] | b1 0x88 [-]
//               b2 0x00 [s1B ctl, bit 2 -> C018 latch; 0 sent] | b3 0x71 [s14 mon, bit 7; 0 sent]
//               b4 0x60 [s13 ctl bit 7 -> C00C (0 sent); s0E mon bits 5-6] | b5 [-] | b6 0xC1 [s0D mon bits 5-7] | b7 [-]
//   0x00224024  b0 bit 2 toggle [-: alive toggle, not read] | b1-b5 [-] | b6 0xC0 [s00 ctl bit 2 -> C027 mode logic; 0 sent] | b7 [-]
//   0x00E24026  b0-b3 [-] | b4 0x9F [s02 ctl bit 5: set = no brake vote C02A; 0 sent] | b5-b7 [-]
//   0x00C0402A  b0 0x1E [s1A nr bits 5-6] | b1, b2 [-] | b3 0x00 [s19 mon, 8-bit] | b4-b7 [-] (b4/b6 = gear lever [LOG])
//   0x10E24000  b0, b1 [-] | b2[3:0]:b3 [s24 mon 12-bit] | b4[3:0]:b5 [s22 mon 12-bit] | b6 bits 4-5 [s23 mon],
//               bits 2-3 [s10 mon], b6[1:0]:b7 [s0F mon 10-bit] - all 0 sent
//   0x11220028  b0-b2 [-] | b3[1:0]:b4 [s0B nr 10-bit, = 1] | b5-b7 [-]
//   0x11420006  b0-b5 [-] | b6 bits 2-3 [s12 nr], b6[1:0]:b7 [s11 nr 10-bit]
static const uint8_t VOLVO_P2G3_42406C_DATA[8] = {0x80, 0x88, 0x00, 0x71, 0x60, 0x00, 0xC1, 0xF3}; // 0x0042406C (byte 0 is replaced by the counter)
static const uint8_t VOLVO_P2G3_224024_DATA[8] = {0x3F, 0xF8, 0x00, 0x00, 0x13, 0xFF, 0xC0, 0x00}; // 0x00224024 (byte 0 bit 2 is toggled)
static const uint8_t VOLVO_P2G3_E24026_DATA[8] = {0x00, 0x01, 0xE0, 0xE4, 0x9F, 0x08, 0x00, 0x00}; // 0x00E24026

// V11.00.0: frames the DEM subscribes to (OEM code: MO table) that we never sent.
// Steady values from the 500k S60 log - what the bytes mean is not known.
static const uint8_t VOLVO_P2G3_C0402A_DATA[8] = {0x1E, 0xE4, 0x00, 0x00, 0x27, 0xFF, 0xD3, 0xFF};   // 0x00C0402A (bytes 4/6 move with the gear lever in the log)
static const uint8_t VOLVO_P2G3_10E24000_DATA[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 0x10E24000 (all zero for the whole log)
static const uint8_t VOLVO_P2G3_11220028_DATA[8] = {0x23, 0x60, 0x0F, 0x3C, 0x01, 0x00, 0x35, 0x2E}; // 0x11220028 (bytes 6-7 drift slowly in the log)
static const uint8_t VOLVO_P2G3_11420006_DATA[8] = {0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0xC0, 0x00}; // 0x11420006

static uint8_t volvoP2G3Counter42406C = 0; // 0x0042406C byte 0 bits 6-7: counts 0-3, one step per send (500k S60 log: 00 40 80 C0)
static bool volvoP2G3Toggle = false;       // flips every 10 ms: 0x00224024 byte 0 bit 2 and 0x0080401E byte 0 (1E <-> 06)
static uint8_t volvoP2G3Slow = 0;          // counts 10 ms sends, 0-4, so the 56/60 ms frames go out every 50 ms

// ---- Volvo Gen4 (P3) ----
// 0x10A2407C page 0x14 (CEM configuration) - from the 500k S60 log. Without it the
// unit stores fault 0x0015 ~32 s after start and goes into limp (clutch open, lock
// requests ignored). Only byte 7 (0x59) matters - all zero is not accepted, any
// other byte can be 0. Once a second is enough (bench: every 2 s still fine).
// Bytes [KM -]: b0 = page 0x14 | b1-b6 = log values, [bench] not needed | b7 = 0x59 [bench][no OEM code for the
// P3 fault; REA S60R #46: page 0x14 b7 (with page 0x10 b1) selects the calibration set, 0x59 = set 0 = default]
static const uint8_t VOLVO_G4_CONFIG_PAGE_DATA[8] = {0x14, 0x04, 0x02, 0x02, 0x01, 0x02, 0x03, 0x59};

// 0x10004002 top two bits of bytes 0 / 2 / 4 / 6, as the real car sends them (500k S60 log)
// [KM -] b0 = 2 [REA: no signal on b0[7:6]][bench: small effect] | b2 = 0 [REA s05 mode: 1/2 = mode 4][bench: 1/2 open
// the clutch] | b4 = 3, b6 = 3 [REA s1F/s1E mon, parameter 3][bench: anything else = fault 0x0012]
static const uint8_t VOLVO_G4_WHEEL_QUALIFIER[4] = {2, 0, 3, 3};

// Bench map (2026-11-07): slip on the bytes 4-7 pair at VOLVO_G4_CRUISE_SPEED ->
// measured pressure (feedback byte 6), held steady for 5 s at each point
static const uint16_t VOLVO_G4_SLIP_RAW[6] = {0, 150, 250, 300, 500, 600};
static const uint8_t VOLVO_G4_SLIP_PRESSURE[6] = {54, 58, 99, 119, 199, 239};

static uint8_t volvoGen4Slow = 0;   // counts 10 ms sends, 0-4, for the 50 ms frames
static uint8_t volvoGen4Config = 0; // counts 100 ms sends, 0-9, for the 1 s config page

// ---- Volvo Gen5 SPC ----
static const uint8_t VOLVO_SPC_ZERO_DATA[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // starting value for the unknown frames [KM -][REA: COM update bits clear -> 39 of the unit's 40 plain signals are never taken]

// ---- SAAB 9-3 XWD ----
// Static frames: starting values, byte meanings not known yet
static const uint8_t SAAB_93_180_DATA[8] = {0x20, 0x0F, 0xFF, 0x68, 0x98, 0x00, 0x00, 0x00}; // 0x180
static const uint8_t SAAB_93_288_DATA[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 0x288
static const uint8_t SAAB_93_280_DATA[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 0x280
static const uint8_t SAAB_93_300_DATA[8] = {0x10, 0x08, 0x00, 0x00, 0x7D, 0xFD, 0x00, 0x20}; // 0x300
static const uint8_t SAAB_93_301_DATA[8] = {0x00, 0x2E, 0xFF, 0x00, 0x00, 0x00, 0x62, 0xC2}; // 0x301
static const uint8_t SAAB_93_410_DATA[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 0x410
static const uint8_t SAAB_93_3ED_DATA[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00}; // 0x3ED
static const uint8_t SAAB_93_320_DATA[8] = {0x00, 0x0A, 0x0A, 0x04, 0x31, 0x28, 0x00, 0x00}; // 0x320
static const uint8_t SAAB_93_510_DATA[8] = {0x00, 0x86, 0x6A, 0x00, 0x00, 0x00, 0x00, 0x00}; // 0x510
static const uint8_t SAAB_93_398_DATA[8] = {0x80, 0x00, 0x50, 0xC3, 0x48, 0x02, 0x00, 0x00}; // 0x398

// 0x160 bytes 2/3 step through these four pairs, one per send
static const uint8_t SAAB_93_160_BYTE2[4] = {0x00, 0x5F, 0x9F, 0xDF}; // counter in bits 6-7 (step 0 sends 00)
static const uint8_t SAAB_93_160_BYTE3[4] = {0x00, 0xFF, 0xFE, 0xFD}; // protection: 0 minus the counter

// ---- Volvo P1 ----
// V11.00.13 per-byte tags: EVERY byte of every P1 table and builder is [KM -][REA none] - no KMatrix
// covers the P1 29-bit set and no P1 AWD-module dump exists. Byte meanings are [LOG] only
// (documents/vehicle-logs/gen3/P1_V50_T5_AWD_DECODE.md, V50 + C30 traces); "steady" = most common logged value.
// Steady values (Drive.csv, 56.9 s). Bytes that are worked out each send are
// marked in the send functions below.
static const uint8_t VOLVO_P1_0300410E_DATA[8] = {0x98, 0x8C, 0xB8, 0x17, 0xA8, 0x86, 0xF5, 0x23}; // torque / pedal / brake switch
static const uint8_t VOLVO_P1_0090411E_DATA[8] = {0x42, 0x07, 0xE7, 0xD0, 0x9F, 0x00, 0x9C, 0x1D}; // rpm (2000 here)
static const uint8_t VOLVO_P1_02104136_DATA[8] = {0xBB, 0x03, 0x3F, 0xFF, 0x24, 0xE0, 0x0B, 0xCD}; // vehicle speed in bytes 6-7
static const uint8_t VOLVO_P1_02000026_DATA[8] = {0x7F, 0x34, 0x00, 0xFC, 0x52, 0x00, 0xE0, 0xEF}; // brake released, DSTC on
static const uint8_t VOLVO_P1_0110483C_DATA[8] = {0x08, 0xE7, 0x22, 0x00, 0x44, 0x7A, 0xFD, 0x0F}; // handbrake off
static const uint8_t VOLVO_P1_02804026_DATA[8] = {0xF8, 0x00, 0x78, 0x00, 0xF2, 0x24, 0xFD, 0xFF}; // steering + yaw set to straight (0x800)
static const uint8_t VOLVO_P1_0100082C_DATA[8] = {0x0F, 0x40, 0xB9, 0xA8, 0x66, 0x11, 0x00, 0x00};
static const uint8_t VOLVO_P1_02C00020_DATA[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x34};
static const uint8_t VOLVO_P1_00800006_DATA[8] = {0x0A, 0x8F, 0x46, 0x95, 0x41, 0xED, 0xAF, 0x3F}; // byte 0 bits 6-7 = counter
static const uint8_t VOLVO_P1_03800006_DATA[8] = {0x1A, 0x8F, 0x9D, 0x13, 0xC1, 0xAD, 0x02, 0x85};
static const uint8_t VOLVO_P1_19000026_DATA[8] = {0x00, 0xFF, 0x01, 0x40, 0x30, 0x11, 0x86, 0x00};
static const uint8_t VOLVO_P1_19E00006_DATA[8] = {0x0A, 0x00, 0x19, 0xC6, 0x98, 0x66, 0x40, 0x00};
static const uint8_t VOLVO_P1_1B200002_DATA[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x73};
static const uint8_t VOLVO_P1_1B600002_DATA[8] = {0x00, 0x00, 0x00, 0x00, 0x87, 0x26, 0x4D, 0xCA};
static const uint8_t VOLVO_P1_1B700030_DATA[8] = {0x6F, 0xA8, 0x26, 0x1E, 0x45, 0xC0, 0x03, 0x01};
static const uint8_t VOLVO_P1_1A200020_DATA[8] = {0x0F, 0x00, 0xA2, 0x29, 0x00, 0x9D, 0x02, 0x2A}; // byte 0 bits 6-7 = counter
static const uint8_t VOLVO_P1_1AE0092C_DATA[8] = {0x00, 0x00, 0x22, 0xE0, 0x41, 0xCA, 0x00, 0x00}; // byte 0 bits 6-7 = counter

// 0x1B600002 byte 1: a 6-step pattern on the real car (C30 trace, V50 values)
static const uint8_t VOLVO_P1_1B600002_BYTE1[6] = {0x40, 0x00, 0x80, 0x00, 0x00, 0x00};

// 0x1BA0493C: 20 pages, byte 0 = page number 1-20, each with its own steady data
// (the most common value of each page across the V50 driving logs).
static const uint8_t VOLVO_P1_1BA0493C_PAGES[20][8] = {
    {0x01, 0x03, 0x02, 0x08, 0x01, 0x03, 0x04, 0x02},
    {0x02, 0x01, 0x05, 0x01, 0x03, 0x01, 0x01, 0x02},
    {0x03, 0x03, 0x02, 0x01, 0x02, 0x02, 0x01, 0x02},
    {0x04, 0x12, 0x10, 0x02, 0x01, 0x02, 0x01, 0x02},
    {0x05, 0x03, 0x01, 0x05, 0x01, 0x03, 0x01, 0x03},
    {0x06, 0x02, 0x02, 0x07, 0x01, 0x03, 0x02, 0x01},
    {0x07, 0x01, 0x03, 0x00, 0x02, 0x01, 0x01, 0x01},
    {0x08, 0x02, 0x01, 0x12, 0x02, 0x03, 0x01, 0x02},
    {0x09, 0x00, 0x00, 0x02, 0x01, 0x01, 0x01, 0x01},
    {0x0A, 0x01, 0x01, 0x01, 0x01, 0x02, 0x06, 0x01},
    {0x0B, 0x01, 0x01, 0x01, 0x01, 0x03, 0x01, 0x01},
    {0x0C, 0x02, 0x01, 0x01, 0x02, 0x02, 0x01, 0x08},
    {0x0D, 0x04, 0x02, 0x01, 0x01, 0x01, 0x01, 0x01},
    {0x0E, 0x01, 0x02, 0x02, 0x02, 0x02, 0x01, 0x02},
    {0x0F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
};

// 0x1BE0493C pages (byte 0 = page). Page 01 is built each time (clock + heartbeat).
// The car also sends its VIN on pages 10, 11 and 12 - not needed, so not sent.
static const uint8_t VOLVO_P1_NETWORK_PAGE02[8] = {0x02, 0x56, 0x8A, 0x00, 0x4A, 0x03, 0xAE, 0xA2}; // status
static const uint8_t VOLVO_P1_NETWORK_PAGE05[8] = {0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x8F};

// Counters and toggles ('static' = they belong to this file only)
static uint32_t volvoP1Tick10 = 0;         // counts 10 ms sends, for the every-Nth bits and the slow frames
static uint8_t volvoP1SteerSensorCount = 0; // 0x0340412E byte 0 low nibble: 0-15, one step per send
static uint8_t volvoP1Count20 = 0;         // counts 20 ms sends: 0x00800006 counter, 0x0090411E / 0x03800006 every-Nth bits
static bool volvoP1Toggle13 = false;       // 0x19000026 byte 6 bit 3, flips every send
static uint8_t volvoP1PulsePage = 0;       // 0x19A00002 byte 0 page: 0-3 -> 02, 12, 6E, 12
static uint8_t volvoP1PulseCounter[4] = {0x52, 0x50, 0xFA, 0xAF}; // 0x19A00002 bytes 1-4: four wheel pulse counters (start as logged)
static uint8_t volvoP1Distance = 0x30;     // 0x19A00002 byte 5: distance count (start as logged)
static uint16_t volvoP1PulseHundredths = 0; // rear pulses not sent yet, in hundredths of a pulse
static uint16_t volvoP1FrontPulseHundredths = 0; // front pulses not sent yet, in hundredths of a pulse
static uint8_t volvoP1DistancePulses = 0;  // pulses since byte 5 last went up
static uint8_t volvoP1Count1B6 = 0;        // 0x1B600002 sends, 0-5
static uint8_t volvoP1Count1B7 = 0;        // 0x1B700030 sends, 0-2
static uint8_t volvoP1Count1A2 = 0;        // 0x1A200020 byte 0 bits 6-7 counter, 0-3
static uint8_t volvoP1Count1AE = 0;        // 0x1AE0092C byte 0 bits 6-7 counter, 0-3
static uint8_t volvoP1Page1BA = 0;         // 0x1BA0493C page to send next, 0-19
static uint8_t volvoP1NetworkSlot = 0;     // 0x1BE0493C 250 ms slot, 0-3
static uint32_t volvoP1NetworkClock = 0x94DECF27; // 0x1BE0493C page 01 bytes 2-5: goes up 10 a second (start as logged)
static bool volvoP1NetworkHeartbeat = false; // 0x1BE0493C page 01 bytes 6-7: 02 04, then FD FB on the next page 01

// Gen1 Standalone Frames
// Synthesises the minimum PQ messages the haldex needs while no donor chassis bus is
// present. All frames go out on the haldex bus (twaiBus1).

// V11.00.13 (audit_gen1_02d F4, raw scan): the 02D keeps the LAST Bremse_4 byte 0 it received for ever -
// no timeout, no reset path until power-off. So when our ceiling stops (Stock, controller disabled,
// Bremse_4 block turned off, generation changed) a car that sends no 0x2A0 of its own would keep our
// last ceiling (0x7F = clutch held open). Gen1_releaseCeiling() sends 3 x 0x2A0 with byte 0 = 0x00
// (0x00..0x7E = "no ceiling", OEM code) 20 ms apart, then stays quiet. A car's own 0x2A0 overrides it.
static bool gen1CeilingOnBus = false;   // our Bremse_4 ceiling has been sent since the last release
static uint8_t gen1ReleaseLeft = 0;      // release frames still to send
static uint32_t gen1ReleaseLastMs = 0;

// Gen4 Standalone Frames (0BR / 0BS / 2K, PQ frame set).
// V11.00.0: rebuilt from the OEM code (documents/haldex-binaries/leaf_trace/
// spec_gen4_0br.md). The lock request is Bremse_4 byte 0 (linear, no counter/CRC
// check). Every other frame the unit monitors is sent with steady, plausible
// content so it never logs a false "missing message" fault - its own genuine
// protections (over-temperature, limp, lamp) are left alone and reported.
// Replays steering (LW_1) from a 16-step capture (lws2[]).
// All frames go out on the haldex bus (twaiBus1).
// V11.00.13 per-byte tags: [KM msg.signal scale] = vw_pq.dbc (PQ35/46 KMatrix
// naming), [REA ...] = what the 0BR's own code does with the byte - sources
// leaf_trace/audit_gen4_0br.md (overrules) + spec_gen4_0br.md §3 (bytes read,
// audit #11/#45 CONFIRMED in sw3016/3017/4013/5010). "not copied" = beyond the
// unit's RX copy length (min DLC, spec §1), so it can never be read.

// Gen4-only rolling counters. "static" here = private to this file.
static uint8_t gen4Bremse5Counter = 0; // Bremse_5 b6 high nibble, 0..15 (not checked by the 0BR)
static uint8_t gen4Bremse8Counter = 0; // Bremse_8 b1 low nibble, 0..15 (not checked by the 0BR)

// Gen4.1 (GM/SAAB) Standalone Frames
// VIN: W0LGS8EF2B1064055  (digits 10..17 = "B1064055")
//
// Simulation scenario: bench test simulating a running car at high load.
//   Driven (front) wheels:     ~20 km/h
//   Non-driven (rear) wheels:  ~15 km/h
//   Engine:                    2000 rpm, running, high torque, ~80% pedal override
//   Brakes / handbrake:        OFF
//   Steering angle:            0 deg (straight), Calibrated
//   Ignition:                  Run (Run/Crank active)
//   Transmission:              D, 3rd gear, TCC Locked, Engaged Forward
//   Validity bits:             0 = Valid  (REMINDER: 1 = Invalid)
//   Virtual Device Avail bits: 1 = Available
//
// Wheel pulse model: 30 mm per pulse, 2 us per timestamp tick.
//   Driven     ~20 km/h -> +4 pulses, +10800 ticks per 10 ms
//   Non-driven ~15 km/h -> +3 pulses, +10800 ticks per 10 ms
//   Sequence Number shared L/R per packet (spec: samples within 0.5 ms).
//   Timestamp Rollover Counter increments each time the 16-bit timestamp wraps.
//   Reset Occurred held True for ~200 ms after power-up, then cleared.
//
// Bucket mapping (spec rate -> scheduler bucket):
//   10 ms:   0x0C1, 0x0C5, 0x0F1, 0x180 (Bus1 SAS), 0x0C9 (spec 12.5ms)
//   20 ms:   0x1CE, 0x1E9, 0x1F5 (spec 25ms)
//   25 ms:   0x2F9 (spec 50ms)
//   100 ms:  0x1F1
//   200 ms:  0x3F1 (spec 250ms), 0x4C1 (spec 500ms)
//   1000 ms: 0x4E1, 0x120 Bus0 (Vehicle_Odometer_HS)
//
// RX (received from Haldex / other modules, not transmitted):
//   0x1CC - Secondary Axle Status
//   0x1CF - Secondary Axle General Information
//   0x1D0 - Front Axle Status
//   0x1D1 - Rear Axle Status

// --- Gen41 simulation state (shared across the 10 ms frames) ---
static uint8_t gen41Seq = 0;          // 2-bit sequence number, same L/R per packet
static uint16_t gen41Frame10Cnt = 0; // frames since power-up (10 ms bucket)

// Driven (front) wheels - ~20 km/h
static uint16_t gen41DrvLPulse = 0;
static uint16_t gen41DrvRPulse = 0;
static uint16_t gen41DrvLTs = 0;
static uint16_t gen41DrvRTs = 0;
static uint8_t gen41DrvLRoll = 0;
static uint8_t gen41DrvRRoll = 0;

// Non-driven (rear) wheels - ~15 km/h
static uint16_t gen41NdrvLPulse = 0;
static uint16_t gen41NdrvRPulse = 0;
static uint16_t gen41NdrvLTs = 0;
static uint16_t gen41NdrvRTs = 0;
static uint8_t gen41NdrvLRoll = 0;
static uint8_t gen41NdrvRRoll = 0;

// Alive Rolling Counts (2-bit) per message
static uint8_t gen41ThrottleArc = 0; // 0x0C9 Driver Throttle Override Detection ARC
static uint8_t gen41BrakeArc = 0;    // 0x0F1 Brake Pedal Position ARC
static uint8_t gen41Frame1e9Counter = 0;  // 0x1E9 4-step rolling counter: B6=(ctr<<5)

struct Gen41ReplayState
{
  uint8_t step;
  uint8_t mask;
};

static Gen41ReplayState gen41Frame130State = {0U, 0x03U};
static Gen41ReplayState gen41Frame330State = {0U, 0x07U};
static Gen41ReplayState gen41Frame2c3State = {0U, 0x03U};
static Gen41ReplayState gen41Frame230State = {0U, 0x07U};

// 0x230 (Bus0) — 8-state synchronized replay table covering 86% of OEM frames.
static const uint8_t gen41Frame230B1[8] = {0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10};
static const uint8_t gen41Frame230B2[8] = {0x00, 0x01, 0x10, 0x11, 0x20, 0x21, 0x30, 0x31};
static const uint8_t gen41Frame230B3[8] = {0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00};
static const uint8_t gen41Frame230B6[8] = {0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F};
static const uint8_t gen41Frame230B7[8] = {0x00, 0x00, 0xFF, 0xFF, 0xFE, 0xFE, 0xFD, 0xFD};

// 0x130 (Bus1) - Multi-axis inertial sensor. Checksum = sum(D0..D5) + 0x26 (D6 excluded).
static const uint8_t gen41Frame130Bus1Tbl[4][8] = {
    {0x00, 0x0E, 0x10, 0x03, 0xFF, 0x20, 0x09, 0x6E},
    {0x20, 0x0F, 0x10, 0x03, 0xFF, 0x40, 0x09, 0xAF},
    {0x40, 0x10, 0x10, 0x03, 0xFF, 0xC0, 0x0A, 0x50},
    {0x60, 0x11, 0x10, 0x00, 0x00, 0x20, 0x08, 0xCF},
};

// 0x330 Bus1: two 12-bit signed acceleration-like values in D0:D1 and D2:D3.
static const uint8_t gen41Frame330Bus1Tbl[8][4] = {
    {0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x0F, 0xFD},
    {0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x03},
    {0x00, 0x00, 0x00, 0x00},
    {0x00, 0x03, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00},
    {0x0F, 0xFD, 0x00, 0x00},
};

static const uint16_t gen41Frame2c3WheelSpeedTbl[4] = {
    0x0A8C,
    0x0A8A,
    0x0A8E,
    0x0A8C,
};

static volatile uint16_t gen41LastWheelSpeed2c3 = 0x0A8C;

// Bus0 mirrors these from Bus1 payload generation with dedicated per-ID timing.
portMUX_TYPE gen41Bus0CacheMux = portMUX_INITIALIZER_UNLOCKED;
twai_message_t gen41Bus0CacheC1;
twai_message_t gen41Bus0CacheC5;
bool gen41Bus0CacheValidC1 = false;
bool gen41Bus0CacheValidC5 = false;

// 0x1E5 PPEI Steering Wheel Angle (GMW8762, DLC=8, 10 ms cadence)
static int16_t gen41SwaAngleRaw = 1;
static uint8_t gen41SwaArc = 0;
static uint8_t gen41SwaPhase = 0;

static constexpr int16_t kSwaRateWobble[8] = {0, +2, 0, -3, 0, +2, +4, -5};
static constexpr uint8_t kSwaAgeBus0[4] = {0x02, 0x02, 0x03, 0x01};
static constexpr uint8_t kSwaAgeBus1[4] = {0xFB, 0xFB, 0xFC, 0xFA};

// --- Ford Kuga Mk1 AWD module (generation 45, 500 kbit/s, 11-bit) -------------
// V11.00.9, from the 2026-11-18/19 bench (documents/FORD_KUGA_GEN4_BENCH_2026-11-18.md) and the
// unit's own firmware (8V41-4C045-AC, documents/haldex-binaries/leaf_trace/trackF_kuga_8v41.md).
// The frame set is exactly the unit's receive table; bytes / periods are a real 2007 Focus Mk2
// (Ford C1 = Kuga Mk1 platform) idling, tools/ford_kuga_kwp/c1_traces/parked.trc. The unit checks
// no checksum and no counter, but the counters are kept as the real car sends them.
// Lock (V11.00.11): the unit's OWN torque strategy, as in the car - no test channel. trackF
// "Strategy enable" / "Normal-path lock request": 0x201 rpm > ~401 enables the base demand, 0x201 b6 pedal
// > 5 enables the pump demand, 0x205 engine torque (w0 b0[4:0]:b1, w3 b6[4:0]:b7) drives the torque
// demand, and the ABS / ESP frame 0x211 b6 field 3 caps it at 8 x C (C = b2[3:0]:b3). The unit makes
// a pressure target from it, runs the pump from target - measured and closes the loop on the valve.
// We send the torque high and set C from the lock % - the same way the car's ESP limits the AWD torque.
// 0x260 b2[3:0]:b3 reports the clutch torque from the measured pressure in the same unit as C.
// [REA audit #13/#54/#61: 8V41 only, and +2 once the pressure-ready latch is set, full scale moves with oil
//  temperature (2438 at -40 C .. 2294 at 8 C .. 1958 at 120 C), 0x0FF = pressure-sensor fault; 9V4N = request echo]
// (V11.00.9 / 10 used the 0x6F1 test / EOL override instead: it skips the pump demand, so the pump sat
// on its standby floor - see FORD_KUGA_GEN4_BENCH_2026-11-18.md. 0x6F1 is now only released, once.)
// Standalone engine rpm is sent only while lock is asked for, so in FWD the pump stays off.
// Each frame is its own frame-edit block (FE_GEN_FORD_KUGA, bits in the comments).
// Per-byte map (V11.00.13 comment pass). [KM ..] = CAN database: Ford C1 has no KMatrix, so the Focus RS
// (focus_rs_understood_signals.dbc) / CGEA1.2 (ford_cgea1_2_ptcan_2011.dbc) / VehicleCAN.dbc name is given where
// the ID exists - "CGEA" = a later platform's layout, C1 unconfirmed. [REA ..] = the unit's own firmware:
// audit_ford_kuga.md (#n = claim ledger) overrules trackF (8V41) / trackH (9V4N). "copies n" = RX copy length.
// 0x080 (copies 6, U0121 group):
//   b0-1  0x2714 [KM RS ACCEL_BRAKE_STATUS.Accelerator_Pedal_Position 6|10 0.1 % / Brake_Pedal_Status 4|2 - overlap only]
//                [REA #25: 16-bit F1030 = raw x 28595/16000 - 17872 -> 0x2714 = ~0]
//   b2-b4 0x75 0x30 0x01 [KM -] [REA: no signal, not read]
//   b5    0x33   [KM -] [REA trackF sig 0x72: b5 bits 0-1, no reader]
//   b6    counter [KM -] [REA audit: not copied (copy length 6) - never reaches the unit]
// 0x090 (copies 8, U0121):
//   b0[7:4] counter [KM -] [REA: no signal, not read]
//   b0[3:0]:b1 0x704 [KM - (RS ENGINE_DATA.Engine_Speed 35|13 2 rpm does not fit these bytes)]
//                [REA #25: 12-bit F102C = raw x 28595/15999 - 3216 -> ~0]
//   b2-b3 0xD7 0x04 [KM -] [REA trackF: b2 bits 4-7 no reader (sig 0x6C), rest no signal]
//   b4[3:0]:b5 0x7E3 [KM -] [REA #25: 12-bit F105C = raw x 5120/2000 - 5120 -> ~48 = ~0]; b4[7:6] no reader (0x6D)
//   b6-b7 0x17 0xD0 [KM -] [REA trackF/H: no reader (8V41 0x6F/0x70, 9V4N 12-bit 0x73)]
// 0x211 (copies 7, U0121) - THE LOCK REQUEST, built in FordKuga_frames10:
//   b0-1  0xFFFE [KM CGEA DesiredTorqBrk_CG1.PrplWhlTot_Tq_RqMx 7|16 4 Nm -131072 (0xFFFE = no limit)] [REA: no signal, not read]
//   b2[7:4] 0x8  [KM -] [REA #79: bit 7 read by nothing (sig 0x5E); bits 6-4 no signal]
//   b2[3:0]:b3 C [KM - (CGEA RearDiffLck_Tq_RqMx 23|12 1 Nm sits at b2:b3[7:4] - differs)]
//                [REA #22: field 3 only, F103C = C<<3, demand F156C = unsigned min(F156C, 8 x C) @0x369EA;
//                 same unit as 0x260 b2[3:0]:b3; 9V4N: the ceiling can't fall faster than a ramp (trackH FUN_0315f4)]
//   b4    0x48   [KM -] [REA #24: bits 4 / 5 no reader (sigs 0x5F / 0x60)]
//   b5    0x48   [KM -] [REA #24: bit 0 no reader (sig 0x61) - yet on the bench b5.0 = 1 released the clutch, unexplained]
//   b6[2:1] field [KM - (CGEA bits 49/50 = AccBrkDeny/AccBrkDis - differs)] [REA #21/#36: 0 none, 1 open (F156C = 0),
//                 2 ramp-down, 3 ceiling = C; 9V4N same (trackH)]; other b6 bits no signal
//   b7    counter [KM -] [REA: not copied (copy length 7)]
// 0x231 (copies 7, U0100): all bytes [KM CGEA/VehicleCAN TransGearData_2 - C1 unconfirmed]
//                [REA trackF/H: b0[7:4] presence check only (8V41 sig 0x5A / 9V4N 0x5B); sigs 0x59/0x5B/0x5C no reader]
// 0x433 (copies 4, not presence-checked):
//   b0-b2 0x00 0x01 0x6A [KM - (CGEA Cluster_Information_3_CG1 = fuel level etc., differs)] [REA trackF: not decoded]
//   b3    0x8C   bit 0 = "handbrake" (Focus RS log) [KM -] [REA #26/#35: F103A -> mode 5 latch (8V41) / mode 4,
//                 alive-gated (9V4N): demand ramped to 0]; bit 1 [REA: 8V41 no reader; 9V4N F1399 thermal-limit
//                 ramp, 1 = down / 0 = up (trackH)] - we send 0
//   b4-b7 [KM -] [REA: not copied (copy length 4)]
// 0x4F0 (copies 6, U0100): all bytes [KM -]
//   b0 0x12 [REA: no signal]; b1 0x24 [REA 8V41: b1[2:0] presence only (sig 0x4C); 9V4N: b1[7:3] = 4 -> cal-set pick]
//   b2 0x12 [REA 9V4N: b2[3:0] = 2 -> (2,4) is no known combo -> cal set 0 = default, harmless (audit sibling table)]
//   b3-b5 0x2A 0x00 0x41 [REA: sigs 0x4D-0x52 no reader]
// 0x4F1 (copies 8, U0100): all bytes [KM -]
//   b0 0x41 [REA: b0[3:0] presence (sig 0x49); 9V4N: = 1 -> F139C = 1 -> cal set 0 (default)]
//   b1-b5 [REA: no signal]; b6-b7 0x5E 0x5E [REA: sigs 0x4A / 0x4B no reader]
static const uint8_t KUGA_080_DATA[7] = {0x27, 0x14, 0x75, 0x30, 0x01, 0x33, 0x00};       // b6 = counter
static const uint8_t KUGA_090_DATA[8] = {0x07, 0x04, 0xD7, 0x04, 0x07, 0xE3, 0x17, 0xD0}; // b0 bits 4-7 = counter
static const uint8_t KUGA_211_DATA[8] = {0xFF, 0xFE, 0x80, 0x00, 0x48, 0x48, 0x00, 0x00}; // b6 bits 1-2 ABS request, b7 = counter
static const uint8_t KUGA_231_DATA[7] = {0x01, 0x66, 0x64, 0x00, 0x01, 0x32, 0x80};
static const uint8_t KUGA_433_DATA[8] = {0x00, 0x01, 0x6A, 0x8C, 0x00, 0xD0, 0x00, 0x00}; // b3 bit 0 = handbrake
static const uint8_t KUGA_4F0_DATA[6] = {0x12, 0x24, 0x12, 0x2A, 0x00, 0x41};
static const uint8_t KUGA_4F1_DATA[8] = {0x41, 0x01, 0x95, 0x00, 0x00, 0x64, 0x5E, 0x5E};
static uint8_t kugaCounter8 = 0;      // 0x080 b6 / 0x211 b7 (the Focus counts them together)
static uint8_t kugaCounter090 = 0;    // 0x090 b0 high nibble
static uint8_t kugaTick = 0;          // 10 ms ticks for the 20 ms / 100 ms / 500 ms / 1000 ms frames
static uint16_t kugaSlowTick = 0;
static uint8_t kugaReleaseLeft = 3;   // V11.00.11: one-shot 0x6F1 release (00s) when our frames start - clears an

// --- Land Rover Gen4 Haldex: Freelander 2 + Evoque (generation 46, 500 kbit/s, 11-bit) -------------
// V11.00.13, UNTESTED (no car log, no bench unit): from the unit's own firmware - FL2 BH52-4C045-AC and Evoque
// BJ32-4C045-AC (leaf_trace/audit_landrover.md, which overrules trackG_landrover.md). One generation for both:
// we never identify the controller, we send the UNION of both receive sets (each unit ignores the other's IDs;
// where both read the same ID the bytes satisfy both layouts - conflicts noted per byte) and decode whichever
// feedback arrives (0x310 FL2 / 0x301 Evoque, parseCANHdx).
// Lock request = the DSC / ABS "direct torque request" (ABS request field 4), the normal path the car uses - NOT the
// UDS DID 0xFD00 end-of-line override (defs.h LR_* has the whole chain + every gate). T = lock % x 1500 / 100.
// Periods: not known (audit #17) - every frame 20 ms, the CCF pages 0x400 / 0x405 100 ms [GUESS - bench].
// Standalone engine rpm only while lock is asked for (as the Kuga), so in FWD the base demand is off.
// Each frame is its own frame-edit block (FE_GEN_LANDROVER_GEN4, bit = "block n" below).
static uint8_t lrTick = 0;   // 20 ms ticks for the 100 ms CCF frames
static uint8_t lrPage = 0;   // 0x400 page index / 0x405 page toggle
static const uint8_t LR_400_PAGES[4] = {0x01, 0x02, 0x09, 0x13}; // 0x400 pages the unit decodes [REA trackG / audit #42]

// ---- Small helpers -------------------------------------------------------------

// Wheel / vehicle speed word for Bremse_3 (and Bremse_1 b2-3) on Gen4.
// Bremse_3 carries each wheel as a 15-bit 0.01 km/h value in bits 1-15 of a
// 16-bit word (bit 0 is a spare flag), so word = km/h x 100 x 2.
// Real vehicle speed when the car's CAN is connected; otherwise a steady
// 10 km/h, close to the ~9.9 km/h (0x07B6-0x07D2) the pre-V11 frames sent.
// All four wheels get the same value: no fake slip (spec_gen4_0br §5).
static uint16_t gen4SpeedWord()
{
  uint16_t speedKmh = 10; // fallback when no real speed is known
  if (hasCANChassis)
  {
    speedKmh = receivedVehicleSpeed; // real speed from the car's ABS / engine frames
  }
  if (speedKmh > 326)
  {
    speedKmh = 326; // top of the 15-bit 0.01 km/h range
  }
  uint16_t speedWord = (uint16_t)(speedKmh * 100) << 1; // 0.01 km/h steps, shifted past spare bit 0
  return speedWord;
}

static uint8_t gen41TakeReplayIndex(Gen41ReplayState &state)
{
  const uint8_t idx = static_cast<uint8_t>(state.step & state.mask);
  state.step = static_cast<uint8_t>((state.step + 1U) & state.mask);
  return idx;
}

static void gen41SwaAdvanceWobble()
{
  gen41SwaPhase++;
  if ((gen41SwaPhase & 0x07U) == 0U)
  {
    if (gen41SwaPhase & 0x08U)
      gen41SwaAngleRaw++;
    else
      gen41SwaAngleRaw--;
  }
  gen41SwaArc = static_cast<uint8_t>((gen41SwaArc + 1U) & 0x03U);
}

static void gen41Build1e5(twai_message_t &frame, bool bus1)
{
  const uint16_t angleBits = static_cast<uint16_t>(gen41SwaAngleRaw);
  const int16_t rate = kSwaRateWobble[gen41SwaPhase & 0x07U];
  const uint16_t rateBits = static_cast<uint16_t>(rate) & 0x0FFFU;

  const uint8_t status = bus1 ? 0x4FU : 0x4CU;
  const uint8_t marker = bus1 ? 0xFFU : 0x00U;
  const uint8_t epvK = bus1 ? 0x5BU : 0x5DU;
  const uint8_t age = bus1 ? kSwaAgeBus1[gen41SwaPhase & 0x03U]
                           : kSwaAgeBus0[gen41SwaPhase & 0x03U];

  frame.identifier = 0x1E5;
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;

  frame.data[0] = status;
  frame.data[1] = static_cast<uint8_t>((angleBits >> 8) & 0xFFU);
  frame.data[2] = static_cast<uint8_t>(angleBits & 0xFFU);
  frame.data[3] = static_cast<uint8_t>(((gen41SwaArc & 0x03U) << 5) | 0x10U | ((rateBits >> 8) & 0x0FU));
  frame.data[4] = static_cast<uint8_t>(rateBits & 0xFFU);
  frame.data[5] = marker;
  frame.data[6] = age;
  frame.data[7] = static_cast<uint8_t>(frame.data[0] + frame.data[1] + frame.data[2] + frame.data[3] + frame.data[4] + frame.data[5] + epvK);
}

static void gen41Build180(twai_message_t &frame)
{
  const uint16_t angleBits = static_cast<uint16_t>(gen41SwaAngleRaw);
  const int16_t rate = kSwaRateWobble[gen41SwaPhase & 0x07U];
  const uint16_t rateBits = static_cast<uint16_t>(rate) & 0x0FFFU;
  const uint8_t arc = gen41SwaArc & 0x03U;
  const uint8_t d2 = static_cast<uint8_t>(angleBits & 0xFFU);

  frame.identifier = 0x180;
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;

  frame.data[0] = 0x4EU;
  frame.data[1] = static_cast<uint8_t>((angleBits >> 8) & 0xFFU);
  frame.data[2] = d2;
  frame.data[3] = static_cast<uint8_t>(0x80U | (arc << 5) | ((rateBits >> 8) & 0x0FU));
  frame.data[4] = static_cast<uint8_t>(rateBits & 0xFFU);
  frame.data[5] = 0xFFU;
  frame.data[6] = static_cast<uint8_t>((0U - d2 - arc) & 0xFFU);
  frame.data[7] = 0x00U;
}

static bool gen41PeriodicDue(uint32_t &lastMs,
                                      uint32_t nowMs,
                                      uint32_t periodMs,
                                      uint32_t firstDelayMs)
{
  if (lastMs == 0U)
  {
    if (nowMs < firstDelayMs)
    {
      return false;
    }
    lastMs = nowMs;
    return true;
  }

  if ((nowMs - lastMs) < periodMs)
  {
    return false;
  }

  lastMs = nowMs;
  return true;
}

static uint8_t gen41WheelStatusByte(uint8_t roll, uint8_t seq,
                                              uint8_t resetOccurred,
                                              uint16_t pulse)
{
  return (uint8_t)(((roll & 0x3) << 6) |
                   ((seq & 0x3) << 4) |
                   ((resetOccurred & 0x1) << 3) |
                   ((pulse >> 8) & 0x3));
}

// --- Volvo Gen4 (P3, 500 kbit/s, 29-bit IDs) ---------------------------------
// V11.00.13 tags: [KM -] on every byte (no KMatrix covers this 29-bit set). There is NO P3 dump:
// [bench] = proven on the 2026-11-07 unit, [no OEM code] = nothing backs it, [REA S60R sNN] = the
// field LAYOUT from the P2 facelift DEM (a different unit, audit_volvo_c166) - layout only.
// V11.00.5, rewritten from the 2026-11-07 bench session on a real P3 unit
// (31219215 AA / software 31219186 AE - documents/VOLVO_GEN4_BENCH_2026-11-07.md).
// The old 11-bit frames (0x12A / 0x20A / 0x220 / 0x405 from the V60 DBC) got no
// answer at all. The unit uses the same 29-bit message set as the P2 facelift
// DEM: it replies on 0x1020000A and is happy with our Gen3 frames. So this sends
// the Gen3 frames with their idle content, and makes lock from the two inputs the
// bench proved the unit really acts on (see VolvoGen4_lockRequestInputs below).
// Every frame is its own frame-edit block (FE_GEN_VOLVO_GEN4, bits in the
// comments), so any one can be switched off in Frame Editing / serial lab BLOCK.
// 0x10A2407C carries two blocks (page 1 and page 0x14), so those two are gated
// here by hand - their block entries have no CAN ID.

// Config page data, wheel qualifiers, the bench slip map and the counters: top
// of this file. VOLVO_G4_RPM_RUNNING / _CRUISE_SPEED / _SLIP_MAX: OpenHaldexC6_defs.h.

// The slip that gives a lock % (pressure = percent of 0xFF), straight lines
// between the bench points. Under the lowest point (~21 %) it stays at 0 slip,
// which is the unit's own pre-load at speed (~0x36). Run Learn to fine-tune.
static uint16_t volvoGen4SlipForPercent(uint8_t lockPercent)
{
  const uint16_t wantedPressure = (uint16_t)((uint32_t)lockPercent * 255 / 100);
  if (wantedPressure <= VOLVO_G4_SLIP_PRESSURE[0])
  {
    return 0;
  }
  for (uint8_t i = 1; i < 6; i++)
  {
    if (wantedPressure <= VOLVO_G4_SLIP_PRESSURE[i])
    {
      const uint16_t slipSpan = VOLVO_G4_SLIP_RAW[i] - VOLVO_G4_SLIP_RAW[i - 1];
      const uint16_t pressureSpan = VOLVO_G4_SLIP_PRESSURE[i] - VOLVO_G4_SLIP_PRESSURE[i - 1];
      const uint16_t pressureInto = wantedPressure - VOLVO_G4_SLIP_PRESSURE[i - 1];
      return (uint16_t)(VOLVO_G4_SLIP_RAW[i - 1] + (uint32_t)slipSpan * pressureInto / pressureSpan);
    }
  }
  return VOLVO_G4_SLIP_MAX;
}

// The Gen4 lock request - what the lock inputs should be for a lock %. Shared by
// standalone and OEM mode, so a Learn table fits both. lockPercent comes from
// lockRequestPercent() (mode buttons / lock target, learn table, lock gates,
// Learn's own sweep).
//   0 %     engine rpm 0          -> pump off, clutch open
//   1-99 %  rpm 900, rear pair 4000, front pair 4000 + 150..600 (front wheels
//           spinning faster than the rear) -> pressure 0x3A..0xEF
//   100 %   rpm 900, all four wheels 0 (car stood still) -> full pull-away pre-lock, 0xFF
// Which pair is which: in the S60 log's one turn and its one wheelspin (traction
// control flag set) the bytes 4-7 pair runs faster, so bytes 4-7 = front axle,
// bytes 0-3 = rear. That also fits the bench: front-faster slip gives steady lock,
// rear-faster hunts.
// [bench][no OEM code for any threshold / pressure / pair order here; REA S60R #36: the code pairs b0-3 and
// b4-7 with separate divisors but does not say which is front]
static void VolvoGen4_lockRequestInputs(uint8_t lockPercent, uint16_t &engineRpm, uint16_t &rearSpeed, uint16_t &frontSpeed)
{
  engineRpm = 0;
  rearSpeed = 0;
  frontSpeed = 0;
  if (lockPercent >= 100)
  {
    engineRpm = VOLVO_G4_RPM_RUNNING;
  }
  else if (lockPercent > 0)
  {
    engineRpm = VOLVO_G4_RPM_RUNNING;
    rearSpeed = VOLVO_G4_CRUISE_SPEED;
    frontSpeed = VOLVO_G4_CRUISE_SPEED + volvoGen4SlipForPercent(lockPercent);
  }
}

// 0x0062401E engine torque / pedal - the values the lock request is measured with.
// Bench: any positive torque or pedal here PULLS THE LOCK DOWN (often to 0) and
// once stored fault 0x0012, and with the frame missing lock drops ~40 %. So it is
// a lock input too: with its block on we always send these (both modes since
// V11.00.7), so the car's throttle can't fight the lock request (and a Learn table stays
// true).
//   b0[1:0]:b1  engine torque, 10-bit, ~512 = 0 Nm (S60 log 502-629, follows rpm r=0.88)
//   b4[1:0]:b5  second engine torque (requested / indicated?), 10-bit, ~512 = 0 Nm
//   b6[1:0]:b7  accelerator pedal, 10-bit, 0 = foot off (S60 log 0-223)
// VOLVO_G4_TORQUE_RAW / VOLVO_G4_PEDAL_RAW (the values sent): OpenHaldexC6_defs.h

// Put the torque / pedal fields into 0x0062401E, keeping every other bit of the
// frame being built (the brake switch in byte 4 bit 4, the 1 Hz toggle in byte 2, ...)
static void volvoGen4PutTorque(uint8_t *data)
{
  data[0] = (uint8_t)((data[0] & 0xFC) | ((VOLVO_G4_TORQUE_RAW >> 8) & 0x03)); // [KM -][REA S60R s09 torque bits 9-8 (layout)][bench value]
  data[1] = (uint8_t)(VOLVO_G4_TORQUE_RAW & 0xFF);                             // [KM -][REA S60R s09 torque bits 7-0, C030 = 4 x raw - 2048][bench value]
  data[4] = (uint8_t)((data[4] & 0xFC) | ((VOLVO_G4_TORQUE_RAW >> 8) & 0x03)); // [KM -][REA S60R s06 2nd torque bits 9-8][bench value]
  data[5] = (uint8_t)(VOLVO_G4_TORQUE_RAW & 0xFF);                             // [KM -][REA S60R s06 bits 7-0, C016 = 4 x raw - 2048][bench value]
  data[6] = (uint8_t)((data[6] & 0xFC) | ((VOLVO_G4_PEDAL_RAW >> 8) & 0x03));  // [KM -][REA S60R s01 pedal bits 9-8][bench value]
  data[7] = (uint8_t)(VOLVO_G4_PEDAL_RAW & 0xFF);                              // [KM -][REA S60R s01 pedal bits 7-0, C038 = (2 x raw + 5) / 10][bench value]
}

// Put one 14-bit wheel speed into 0x10004002, keeping the given qualifier bits
static void volvoGen4PutWheel(uint8_t *data, uint8_t wheel, uint8_t qualifier, uint16_t speed)
{
  data[wheel * 2] = (uint8_t)((qualifier << 6) | ((speed >> 8) & 0x3F)); // b0/b2/b4/b6 [KM -]: [7:6] qualifier (see VOLVO_G4_WHEEL_QUALIFIER), [5:0] speed bits 13-8 [REA S60R s21/s20/s1D/s1C layout]
  data[wheel * 2 + 1] = (uint8_t)(speed & 0xFF);                         // b1/b3/b5/b7 [KM -]: speed bits 7-0, 1/64 km/h [bench + vtl/volvo-ddd; no OEM code for the scale]
}

// The lock request: lock % -> ESP torque ceiling C (0x211 b2[3:0]:b3). 0 = open; 1..100 % = 13..1281 (V11.00.13:
// the unit's top parked demand, audit_ford_kuga #92; C < KUGA_REQUEST_OPEN = no demand). In OEM the car's own ESP ceiling is never
// exceeded - its traction / stability limit stays in charge.
static uint16_t kugaCeilingForPercent(uint8_t lockPercent)
{
  if (lockPercent > 100)
  {
    lockPercent = 100;
  }
  uint16_t c = (uint16_t)((uint32_t)lockPercent * KUGA_REQUEST_FULL / 100);
  if (!isStandalone && kugaCarTorqueCeiling != 0xFFFF && kugaCarTorqueCeiling < c)
  {
    c = kugaCarTorqueCeiling;
  }
  return c;
}

// One-shot release of the unit's test channel (00 00 00 00 00 = its own control), 3 frames 100 ms
// apart when our frames start. Sent directly (not a frame-edit block): it must go out in both modes.
static void kugaReleaseTestChannel()
{
  if (kugaReleaseLeft == 0)
  {
    return;
  }
  twai_message_t f = {};
  f.identifier = FORD_KUGA_OVERRIDE_ID;
  f.extd = 0;
  f.data_length_code = 5; // b0-b4 = 00 [KM -] [REA #28: 8V41 b0 bits 0-1 = 0 -> F106F = 0 (normal), F1042 = 0, b0[5:2] = 0 no sub-mode;
                          //  9V4N #66: cmd b0[5:2] = 0 -> F13F2 = 0, and it auto-releases after ~100 cycles without 0x6F1 anyway]
  twai_transmit_v2(twaiBus1, &f, 0);
  kugaReleaseLeft--;
}

// lock % -> field-4 torque T. 0 % = T 0 (F257E = 0 -> clutch open through the normal path); 100 % = 1500
// (F15A0 = 24000 = the unit's cap F1548). Never above 0x0FFF: the unit's << 4 is 16-bit (audit #29a).
static uint16_t lrTorqueForPercent(uint8_t lockPercent)
{
  if (lockPercent > 100)
  {
    lockPercent = 100;
  }
  uint32_t t = ((uint32_t)lockPercent * LR_REQUEST_FULL + 50) / 100;
  if (t > LR_REQUEST_MAX)
  {
    t = LR_REQUEST_MAX;
  }
  return (uint16_t)t;
}
// ---- Volvo P1 (S40 / V50 AWD, generation 31) Standalone Frames (V11.00.1) ----

// Steady frame data, page tables and counters: top of this file.
// Tags below: every byte [KM -][REA none] (no P1 dump, no P1 KMatrix) - [LOG] = from the V50 / C30 traces.

// A value part-way along a straight line: atZero at 0 % lock, atFull at 100 %
static uint16_t volvoP1Scale(uint16_t atZero, uint16_t atFull, uint8_t lockPercent)
{
  int32_t change = (int32_t)atFull - (int32_t)atZero;     // how far the value moves from 0 % to 100 %
  int32_t value = (int32_t)atZero + change * lockPercent / 100;
  return (uint16_t)value;
}

// Copy a steady 8-byte frame into data[]
static void volvoP1Copy(uint8_t data[8], const uint8_t steady[8])
{
  for (uint8_t i = 0; i < 8; i++)
  {
    data[i] = steady[i];
  }
}

// The lock % that goes into the engine frames (torque, pedal, rpm): the lock
// when Lock Source is Engine Torque or Both, otherwise 0 (engine at a light cruise)
static uint8_t volvoP1EngineLockPercent(uint8_t lockPercent)
{
  if (volvoP1LockSource == VOLVO_P1_SOURCE_SLIP)
  {
    return 0;
  }
  return lockPercent;
}

// The lock % that goes into the wheel speeds: the lock when Lock Source is Wheel
// Slip or Both, otherwise 0 (all four wheels the same)
static uint8_t volvoP1SlipLockPercent(uint8_t lockPercent)
{
  if (volvoP1LockSource == VOLVO_P1_SOURCE_TORQUE)
  {
    return 0;
  }
  return lockPercent;
}

// Rear wheel / road speed to send, 0.01 km/h per count (Road Speed setting)
static uint16_t volvoP1RearSpeedRaw()
{
  return (uint16_t)volvoP1RoadSpeedKmh * 100;
}

// Engine rpm to send: Road Speed x the Gear setting's ratio (rpm per km/h, from
// the V50 logs), so the rpm and the road speed always match a real gear. A pair
// that matches no gear (4500 rpm at 30 km/h, as before) looks like a slipping
// clutch, and the module may not pass torque to the rear with the clutch slipping.
static uint16_t volvoP1GearRpm()
{
  uint16_t rpmPerKmh = VOLVO_P1_RPM_PER_KMH_GEAR2; // 2nd gear (the default)
  if (volvoP1Gear == 1)
  {
    rpmPerKmh = VOLVO_P1_RPM_PER_KMH_GEAR1; // 1st gear
  }
  else if (volvoP1Gear == 3)
  {
    rpmPerKmh = VOLVO_P1_RPM_PER_KMH_GEAR3; // 3rd gear
  }
  uint32_t rpm = (uint32_t)volvoP1RoadSpeedKmh * rpmPerKmh;
  if (rpm > VOLVO_P1_RPM_MAX)
  {
    rpm = VOLVO_P1_RPM_MAX; // e.g. 120 km/h in 1st: hold at the limit rather than send a silly number
  }
  return (uint16_t)rpm;
}

// Front wheel speed to send, 0.01 km/h: the rears plus the Wheel Slip share of the lock
static uint16_t volvoP1FrontSpeedRaw(uint8_t lockPercent)
{
  uint16_t slipRaw = volvoP1Scale(0, (uint16_t)volvoP1SlipFullKmh * 100, volvoP1SlipLockPercent(lockPercent));
  return volvoP1RearSpeedRaw() + slipRaw;
}

// 0x0340412E byte 3, the steering angle sensor's check byte. From all 26,820
// logged frames (V50 and C30, every one fits):
//   byte 3 = 0x02 - (byte 0 with its two nibbles swapped + byte 1 bit 7
//                    + byte 4 + byte 5 + byte 6 + byte 7)       (wraps at 256)
// with 0x04 instead of 0x02 when byte 0 bit 6 (the angle direction) is set.
// Bytes 1-2 (the fine steering rate) are not in it, apart from byte 1's sign bit.
static uint8_t volvoP1SteerSensorCheck(const uint8_t data[8])
{
  uint8_t byte0Swapped = (uint8_t)(((data[0] & 0x0F) << 4) | (data[0] >> 4)); // byte 0 with its nibbles swapped
  uint8_t base = VOLVO_P1_STEER_CHECK_BASE;                                  // 0x02
  if (data[0] & (1 << 6))
  {
    base = VOLVO_P1_STEER_CHECK_BASE + 2; // byte 0 bit 6 set (the other steering direction): 0x04
  }
  uint8_t total = byte0Swapped;  // adds up in a byte, so it wraps at 256 by itself
  total = total + (data[1] & 0x80); // byte 1 sign bit only
  total = total + data[4];
  total = total + data[5];
  total = total + data[6];
  total = total + data[7];
  return (uint8_t)(base - total);
}

// Checksummed frames: map a CAN ID to its ID sequence so an overridden byte can
// have its CRC recomputed. IDs absent here carry no checksum.
static const uint8_t *idSeqFor(uint32_t id)
{
  switch (id)
  {
  case 0x0A8:
    return ID_SEQ_0A8;
  case 0x0AD:
    return ID_SEQ_0AD;
  case 0x0A7:
    return ID_SEQ_0A7;
  case 0x08A:
    return ID_SEQ_08A;
  case 0x086:
    return ID_SEQ_086;
  case 0x121:
    return ID_SEQ_121;
  case 0x110:
    return ID_SEQ_110;
  case 0x106:
    return ID_SEQ_106;
  case 0x104:
    return ID_SEQ_104;
  case 0x116:
    return ID_SEQ_116;
  case 0x101:
    return ID_SEQ_101;
  case 0x0FD:
    return ID_SEQ_0fd;
  case 0x5BE:
    return ID_SEQ_5be;
  case 0x3BE:
    return ID_SEQ_3be;
  case 0x641:
    return ID_SEQ_641;
  case 0x645:
    return ID_SEQ_645;
  case 0x65D:
    return ID_SEQ_65d;
  case 0x392:
    return ID_SEQ_392;
  default:
    return nullptr;
  }
}

static void standaloneTx(twai_message_t &f)
{
  f.self = 0; // a self-received frame reads as a live Haldex (flicker to ✓ with lock 0)
  // Serial-lab mutes (TXMUTE): bench bisecting / whole-set replacement by TXADD.
  if (labMuteAll)
    return;
  for (uint8_t i = 0; i < LAB_MUTE_MAX; i++)
    if (labMuteIds[i] != 0 && labMuteIds[i] == f.identifier)
      return;
  int genid = frameEditGenIdx(haldexGeneration);
  if (genid >= 0)
  {
    for (uint16_t i = 0; i < frameEditBlockCount; i++)
    {
      if (frameEditBlocks[i].genIdx == (uint8_t)genid &&
          frameEditBlocks[i].canId == f.identifier)
      {
        if (!frameEditEnabled((uint8_t)genid, frameEditBlocks[i].bit))
          return; // block disabled -> do not generate this frame
        break;
      }
    }
  }

  // Serial-lab byte overrides, applied last so they beat whatever built the
  // frame. CRC is recomputed after, or the Haldex would reject the frame and
  // the measurement would be of a dropped frame rather than a changed byte.
  bool overridden = false;
  for (uint8_t i = 0; i < LAB_OVR_MAX; i++)
  {
    if (labOverrides[i].canId == f.identifier && labOverrides[i].byteIdx < 8)
    {
      f.data[labOverrides[i].byteIdx] = labOverrides[i].value;
      overridden = true;
    }
  }
  if (overridden)
  {
    const uint8_t *seq = idSeqFor(f.identifier);
    // Volvo / SAAB frames carry no VW checksum: SAAB's 0x110 shares an ID with
    // a checksummed VW frame, so only recompute the CRC on the VW generations.
    if (seq && !isVolvoOrSaabGeneration(haldexGeneration) && haldexGeneration != GEN_FORD_KUGA &&
        haldexGeneration != GEN_LANDROVER_GEN4) // V11.00.13: LR 0x101 / 0x116 share IDs with checksummed VW frames - no checksum here
      f.data[0] = calcChecksum(f.data, seq);
  }

  noteOurFrame(f); // V11.00.7: Long Learn reports our bytes next to the car's
  twai_transmit_v2(twaiBus1, &f, 0);
}

void Gen1_releaseCeiling(bool ceilingStillOurs)
{
  if (ceilingStillOurs)
  {
    gen1ReleaseLeft = 0;
    return;
  }
  if (gen1CeilingOnBus)
  {
    gen1CeilingOnBus = false;
    gen1ReleaseLeft = 3;
    gen1ReleaseLastMs = millis() - 20;
  }
  if (gen1ReleaseLeft == 0 || (uint32_t)(millis() - gen1ReleaseLastMs) < 20)
  {
    return;
  }
  twai_message_t f = {};
  f.identifier = BRAKES4_ID; // PQ Bremse_4 0x2A0 (KMatrix vw_pq), DLC 3
  f.extd = 0;
  f.data_length_code = 3;
  f.data[0] = 0x00; // [KM Bremse_4.ABS_Vorgabewert_mitte_Kupplungs 3 Nm/min -381] [REA audit_gen1_02d #57: b0 < 0x7F = no ceiling]
  f.data[1] = 0x00; // [KM Bremse_4.ABS_Vorgabewert_hinten_Kupplung 0.7874 %] [REA spec_gen1_02d §3: not read]
  f.data[2] = 0x00; // [KM Bremse_4.Einheit_Kupplungssteifigkeit b2.0, rest Frei] [REA spec_gen1_02d §3: not read]
  twai_transmit_v2(twaiBus1, &f, 0); // direct: the Bremse_4 block may be off, and Stock sends nothing else
  gen1ReleaseLastMs = millis();
  gen1ReleaseLeft--;
}

void Gen1_frames10()
{
  // not used in Gen1
}

void Gen1_frames20()
{
  twai_message_t frame = {};
  // PQ Motor_1 (0x280, DLC 8) - main engine ECU broadcast (vw_pq.dbc: Motor_1).
  // Byte layout and what the 02D reads: OEM code, spec_gen1_02d §3 / §5.
  frame.identifier = MOTOR1_ID;
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;                     // status bits all clear [KM Motor_1.Leergasinformation b0.0 .. Kupplungsschalter b0.3 .. Momentenangaben_ungenau b0.7] [REA audit_gen1_02d #15: only b0.3 read -> E1E2.3 status]
  frame.data[1] = gen1DemandValue(0xFE);  // the demand [KM Motor_1.inneres_Motor_Moment 0.39 % MDI] [REA audit_gen1_02d #19/#25: E11A = E108 x b1/255, PLUS side of (b1 - b6); 0xFF invalid]
  // Motordrehzahl (KMatrix vw_pq Motor_1 b2-3, 0.25 rpm/bit). REA (audit_gen1_02d F2): the 02D's rpm term asks for
  // lock by itself - it needs >~4000 rpm for full lock (pump enable > 400 rpm, off < 200 rpm), so 0x4E21 = 4992 rpm
  // while lock is wanted. V11.00.13 (audit F3): with the Bremse_4 block OFF that rpm alone made the 02D ask ~53 %
  // in FWD, so at 0 % without the ceiling lock request the rpm drops to 0x0021 (8 rpm = pump off = open, as V10).
  const bool gen1RpmLow = !gen1CeilingRequestActive() && lockRequestPercent() == 0;
  frame.data[2] = 0x21;                     // same in both cases [KM Motor_1.Motordrehzahl low, 0.25 rpm] [REA audit_gen1_02d #21: rpm E1BC = b2-3, >= 0xFF00 -> 8000]
  frame.data[3] = gen1RpmLow ? 0x00 : 0x4E; // 0x4E21 = 4992 rpm (lock wanted) / 0x0021 (FWD, no lock request) [KM Motor_1.Motordrehzahl high] [REA audit_gen1_02d #52/F2/F3: pump on > raw 0x640, off <= 0x320; rpm term E1FE + ratio E188]
  frame.data[4] = 0x00;                     // (spec D10, was mislabelled "pedal") [KM Motor_1.inneres_Motor_Moment_ohne_exter 0.39 % MDI] [REA audit_gen1_02d #23: not read]
  frame.data[5] = gen1DemandValue(0xFE);  // the demand [KM Motor_1.Fahrpedalwert_oder_Drosselklapp 0.4 %] [REA audit_gen1_02d #22/F5: used only once "pedal learned" (<= 30 % for 30 cycles) - harmless]
  appliedTorque = gen1LossTorqueByte(0x00); // mechanisches_Motor_Verlustmoment: the 02D subtracts it from byte 1 (0x00 = max demand)
  frame.data[6] = appliedTorque;            // loss torque [KM Motor_1.mechanisches_Motor_Verlustmomen 0.39 % MDI] [REA audit_gen1_02d #18/#25: E1A0 = E108 x b6/255, SUBTRACTED from b1; 0xFF -> 30 Nm]
  frame.data[7] = 0x00;                     // driver-wish torque [KM Motor_1.Fahrerwunschmoment 0.39 % MDI] [REA audit_gen1_02d #20: E112 computed, never used]
  standaloneTx(frame);

  // PQ Motor_2 (0x288, DLC 8) - NEW in V11 (spec_gen1_02d D4). The 02D watches for
  // this frame: without it its "all inputs valid" check never passes and 0x2C0
  // byte 0 bit 4 stays set for ever (our UI used to show that as "limp mode").
  frame.identifier = MOTOR2_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0xDA; // mux 3 + 26 x 10 Nm = 260 Nm [KM Motor_2.MO2_Mp_Code b0.6-7 / MO2_max_Mo b0.0-5 10 Nm] [REA audit_gen1_02d #17: E108 = bits0-5 x 640 when mux 3; own init 0x4100 = 260 Nm]
  frame.data[1] = 0x00; // [KM Motor_2.MO2_Kuehlm_T 0.75 -48 degC] [REA spec_gen1_02d §3: not read]
  frame.data[2] = 0x00; // status bits, set below [KM Motor_2.MO2_BLS b2.0, MO2_BTS b2.1, MO2_Sta_No_Bet b2.4] [REA audit_gen1_02d #15/#40: b2.0 -> E1EC.6, b2.1 -> E1EC.8 brake votes (brake path, ~11 % cap #41); b2.4 -> E1E4.0 monitor]
  if (brakeForHaldex())
  {
    frame.data[2] = frame.data[2] | (1 << 0); // bit 0 MO2_BLS: brake light switch (Follow Brake on + brake pressed)
  }
  frame.data[3] = 0x00; // [KM Motor_2.TSK_Limiter / MO2_GRAregelt / MO2_Sport_Error / MO2_OffRoad / MO2_RME_Gehalt] [REA spec_gen1_02d §3: not read]
  frame.data[4] = 0x00; // [KM Motor_2.MO2_GRA_Soll 1.28 km/h] [REA spec_gen1_02d §3: not read]
  frame.data[5] = 0x00; // [KM Motor_2.MO2_LL_Solldz 10 rpm] [REA spec_gen1_02d §3: not read]
  frame.data[6] = 0x00; // [KM Motor_2.MO2_Begr_Mo 0.39 %] [REA spec_gen1_02d §3: not read]
  frame.data[7] = 0x00; // [KM Motor_2.MO2_Mo_ZWR 0.39 %] [REA spec_gen1_02d §3: not read]
  standaloneTx(frame);

  // PQ Bremse_4 (0x2A0, DLC 3 per K-matrix vw_pq.dbc) - NEW in V11. Byte 0 is a
  // torque CEILING on the 02D's own request = our lock request (spec_gen1_02d §2.3).
  // Motor_1 above is held at maximum demand while this block is on.
  frame.identifier = BRAKES4_ID;
  frame.data_length_code = 3;
  frame.data[0] = gen1LockRequestByte(); // 0x7F = open, 0x81..0xC9 = partial (0xA5 ~ 50 %), 0xFD = full [KM Bremse_4.ABS_Vorgabewert_mitte_Kupplungs 3 Nm/min -381] [REA audit_gen1_02d #57/#68: ceiling (b0-0x80) x 192 on E258, 0x7F = 0 at once, sticky #12]
  frame.data[1] = 0x00;              // [KM Bremse_4.ABS_Vorgabewert_hinten_Kupplung 0.7874 %] [REA spec_gen1_02d §3: not read]
  frame.data[2] = 0x00;              // [KM Bremse_4.Einheit_Kupplungssteifigkeit b2.0, rest Frei] [REA spec_gen1_02d §3: not read]
  if (gen1CeilingRequestActive())
  {
    gen1CeilingOnBus = true; // the 02D now holds our ceiling until it sees another 0x2A0 (see Gen1_releaseCeiling)
  }
  standaloneTx(frame);

  // PQ Motor_3 (0x380, DLC 8) - secondary engine ECU broadcast (vw_pq.dbc: Motor_3).
  // The 02D has NO receive slot for 0x380 (OEM code, spec_gen1_02d §1), so this
  // frame does nothing; its block is off by default since V11 (D8).
  frame.identifier = MOTOR3_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // status flags [KM Motor_3.MO3_Vorgluehen b0.0 .. MO3_Sta_Pedal b0.4 .. MO3_Sta_Temp b0.7] [REA audit_gen1_02d #2: 0x380 not received]
  frame.data[1] = 0x50; // [KM Motor_3.MO3_Offsentemp 0.75 -48 degC] [REA audit_gen1_02d #2: not received]
  frame.data[2] = 0x00; // [KM Motor_3.MO3_Pedalwert 0.4 %] [REA audit_gen1_02d #2: not received]
  frame.data[3] = 0x00; // [KM Motor_3.MO3_Rad_Wu_Mo low 8 of 12 bits, 0.39 %] [REA audit_gen1_02d #2: not received]
  frame.data[4] = 0x00; // [KM Motor_3.MO3_Rad_Wu_Mo high nibble, MO3_Vorz_RadWu b4.4, MO3_Freigabe_Segeln b4.5] [REA audit_gen1_02d #2: not received]
  frame.data[5] = 0x00; // [KM Motor_3.MO3_Dz_Beeinfl 0.39 %] [REA audit_gen1_02d #2: not received]
  frame.data[6] = 0x00; // [KM Motor_3.MO3_WunschDz 25 rpm] [REA audit_gen1_02d #2: not received]
  frame.data[7] = 0xFE; // [KM Motor_3.MO3_DKW 0.4 % (m0) / MO3_FPGradient (m1)] [REA audit_gen1_02d #2: not received]
  standaloneTx(frame);

  // PQ Bremse_1 (0x1A0, DLC 8) - ABS/ESP main broadcast.
  // Bremse = brake. The 02D reads bytes 0-2 only (OEM code, spec_gen1_02d §3);
  // it does not check the counter.
  frame.identifier = BRAKES1_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x80; // bit 7 as before; bit 2 set below [KM Bremse_1.BR1_EBV_Ongr b0.7, BR1_ABS_Brems b0.2] [REA audit_gen1_02d #15/#42: only b0.2 read -> E1E2.6 / E276 = 0 (brake-path request 0)]
  if (absBrakingForHaldex())
  {
    frame.data[0] = frame.data[0] | (1 << 2); // bit 2 BR1_ABS_Brems: the car's ABS is braking (only with a chassis bus)
  }
  frame.data[1] = 0x00; // all clear (brake goes in Motor_2) [KM Bremse_1.BR1_Lichtschalt b1.3, BR1_StaDruckschw b1.4, BR1_Diagnose b1.7] [REA audit_gen1_02d #15/#40/#50/F7: b1.3 -> E1EC.7 / b1.4 -> E1EC.9 brake votes, b1.7 -> request 0 + pump off]
  frame.data[2] = 0x00; // [KM Bremse_1.BR1_BKV_active b2.0 + BR1_Rad_kmh low bits] [REA audit_gen1_02d #15: b2.0 -> E1E4.13 brake-source select; speed not read]
  frame.data[3] = 0x0A; // [KM Bremse_1.BR1_Rad_kmh high, 0.01 km/h] [REA spec_gen1_02d §3: not read]
  frame.data[4] = 0xFE; // [KM Bremse_1.BR1_ASRMo_sl 0.39 %] [REA spec_gen1_02d §3: not read]
  frame.data[5] = 0xFE; // [KM Bremse_1.BR1_ASRMo_fa 0.39 % (m0) / BR1_MSR_Mo_inv (m1)] [REA spec_gen1_02d §3: not read]
  frame.data[6] = 0x00; // [KM Bremse_1.BR1_MSR_Mo 0.39 %] [REA spec_gen1_02d §3: not read]
  frame.data[7] = BRAKES1_counter; // [KM Bremse_1.BR1_Zaehler b7.0-3, BR1_ASR_ESP..BR1_Ersatz_Kmh b7.4-7] [REA trackE/audit_gen1_02d #11: no counter check, presence only]
  if (++BRAKES1_counter > 0xF)
    BRAKES1_counter = 0;
  standaloneTx(frame);

  // PQ Bremse_3 (0x4A0, DLC 8) - per-wheel speeds (vw_pq.dbc: Bremse_3, word >> 1 = 0.01 km/h).
  // Without this frame the 02D's request is 0 (OEM code §2.1). 0x0A00 = 12.8 km/h on
  // every wheel; the fronts get up to 1.27 km/h of fake slip to add demand
  // (fine: the 02D's slip hold only starts at ~3.7 km/h front-over-rear, spec §5).
  frame.identifier = BRAKES3_ID;
  frame.data_length_code = 8;
  frame.data[0] = gen1DemandValue(0xFE); // front-left low [KM Bremse_3.Radgeschw__VL bits1-15 0.01 km/h, b0.0 Frei] [REA audit_gen1_02d #32/#33/F6: (word>>1); fronts - rears = slip E1BE; fake slip not needed, slows rise ~4x]
  frame.data[1] = 0x0A;                    // [KM Bremse_3.Radgeschw__VL high] [REA audit_gen1_02d #13: wheel valid while word>>1 < 0x7F80]
  frame.data[2] = gen1DemandValue(0xFE); // front-right low [KM Bremse_3.Radgeschw__VR bits17-31 0.01 km/h] [REA audit_gen1_02d #33: front avg]
  frame.data[3] = 0x0A;                    // [KM Bremse_3.Radgeschw__VR high] [REA audit_gen1_02d #13: valid < 0x7F80]
  frame.data[4] = 0x00;                                        // rear-left low [KM Bremse_3.Radgeschw__HL bits33-47 0.01 km/h] [REA audit_gen1_02d #33: rear avg]
  frame.data[5] = 0x0A;                                        // [KM Bremse_3.Radgeschw__HL high] [REA audit_gen1_02d #13: valid < 0x7F80]
  frame.data[6] = 0x00;                                        // rear-right low [KM Bremse_3.Radgeschw__HR bits49-63 0.01 km/h] [REA audit_gen1_02d #33: rear avg]
  frame.data[7] = 0x0A;                                        // [KM Bremse_3.Radgeschw__HR high] [REA audit_gen1_02d #13/#43: valid < 0x7F80; no Bremse_3 = request 0]
  standaloneTx(frame);
}

void Gen1_frames25()
{
  // not used in Gen1
}
void Gen1_frames100()
{
  // PQ Kombi_1 (0x320, DLC 8) - NEW in V11, block OFF by default. The 02D reads
  // byte 1 bit 1 (KO1_Handbremse): pulling the handbrake between 5 and 100 km/h
  // opens the clutch (OEM code, spec_gen1_02d §2.2). While this frame is NOT sent
  // the 02D uses its own handbrake wire instead, which is why it is off by default.
  twai_message_t frame = {};
  frame.identifier = mKombi_1;
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // lamp bits [KM Kombi_1.Fahrertuer_4_1 b0.0 .. Oeldruck b0.2 .. Vorgluehlampe b0.7] [REA spec_gen1_02d §3: not read]
  frame.data[1] = 0x00; // bit 1 set below; bit 0 clear [KM Kombi_1.Bremsinfo b1.0-1 = KMatrix V5.20.6F KO1_Bremsinfo b1.0 / KO1_Handbremse b1.1] [REA audit_gen1_02d #15: b1.1 -> E1E2.7 handbrake, rising edge at 5-100 km/h opens the clutch]
  if (handbrakeForHaldex())
  {
    frame.data[1] = frame.data[1] | (1 << 1); // bit 1: handbrake pulled (Follow Handbrake on + handbrake pulled)
  }
  frame.data[2] = 0x00; // [KM Kombi_1.Tankinhalt b2.0-6 1 l, Tankwarnung b2.7] [REA spec_gen1_02d §3: not read]
  frame.data[3] = 0x00; // [KM Kombi_1.Signalquelle_Geschwindigkeit b3.0 + Geschwindigkeit__Kombi_1_ low (0.01 km/h)] [REA spec_gen1_02d §3: not read]
  frame.data[4] = 0x00; // [KM Kombi_1.Geschwindigkeit__Kombi_1_ high] [REA spec_gen1_02d §3: not read]
  frame.data[5] = 0x00; // [KM Kombi_1.ADR_Summer / Zeitluecke / Blinker b5.2-5 + Angezeigte_Geschwindigkeit low b5.6-7] [REA spec_gen1_02d §3: not read]
  frame.data[6] = 0x00; // [KM Kombi_1.Angezeigte_Geschwindigkeit high, 0.32 km/h] [REA spec_gen1_02d §3: not read]
  frame.data[7] = 0x00; // [KM Kombi_1.Frei_Kombi_1_3] [REA spec_gen1_02d §3: not read]
  standaloneTx(frame);
}
void Gen1_frames200()
{
  // not used in Gen1
}
void Gen1_frames1000()
{
  // not used in Gen1
}

// Gen2 Standalone Frames (PQ - VW Touran/early Audi etc.)
// Richer than Gen1: needs steering-angle (LW_1), Kombi_1, and several brake-side frames.
// All transmitted on the haldex bus (twaiBus1).
// V11.00.0 (OEM code: documents/haldex-binaries/leaf_trace/spec_gen2_7h0.md §5):
//   - lock request = Bremse_4 (0x2A0) byte 0 (0x7F open .. 0xFD full), with Motor_1 held
//     at full demand and a little front-wheel slip while lock is wanted (gen2DemandValue).
//   - the unit checks NO checksums; alive counters only on Bremse_1 b7 bits 0-3,
//     LW_1 b5 bits 4-7 and 0x4A8 b6 bits 4-7 - those must change every frame.
//   - Motor_3 (0x380), Bremse_9 (0x0AE), Bremse_10 (0x3A0) are not received by the
//     7H0 / 1K0 - their blocks are off by default (frameEditMaskDefaultsSA).
//   - 1K0907554B: same frames we send; but its final-request scale is 0x540 (7H0 0x440), so full valve
//     comes at Bremse_4 b0 0xEA (7H0 0xD6), its 0x2C0 read-back tops at ~0xEC (7H0 ~0xD7) and its
//     0x2C0 is DLC 4 (no byte 5/7, no counter) [REA audit_gen2_7h0 #55/#57/#61/#4].
//   - both CAN controllers of the OEM code sit on the same pins = one bus [REA audit_gen2_7h0 #9/F9],
//     so every frame here reaches the unit on that bus.
void Gen2_frames10()
{
  twai_message_t frame = {};
  // PQ Bremse_1 (0x1A0, DLC 8) - ABS/ESP main broadcast (vw_pq.dbc: Bremse_1).
  // The unit reads (audit_gen2_7h0 #24, corrects spec §3): b0 bit2/bit4, b1 bits 0/3/4/7, b2 bit0,
  // b7 bit5, and b7 bits 0-3 as an alive counter - NOT b6. "ABS/ESP bits clear" = normal car.
  // No Bremse_1 also sets 0x2C0 b0 bit2 (fault/lamp) [REA audit_gen2_7h0 #63].
  frame.identifier = BRAKES1_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // all off [KM Bremse_1.BR1_ASR_Anf b0.0 .. BR1_ABS_Brems b0.2 .. BR1_ESP_Ongr b0.4 .. BR1_EBV_Ongr b0.7] [REA audit_gen2_7h0 #24: b0.2 + b0.4 read (ABS/ESP votes)]
  if (absBrakingForHaldex())
  {
    frame.data[0] = frame.data[0] | (1 << 2); // bit2 BR1_ABS_Brems: car's ABS is braking - the unit opens the clutch
  }
  frame.data[1] = 0x40;                   // bit0 ABS lamp now OFF (was 0x41) - V11 TODO (bench): confirm [KM Bremse_1.BR1_Lampe_ABS b1.0, BR1_Lichtschalt b1.3, BR1_StaDruckschw b1.4, BR1_Sta_MAD b1.6, BR1_Diagnose b1.7] [REA audit_gen2_7h0 #24: b1.0/.3/.4/.7 read, b1.6 not]
  frame.data[2] = 0x00;                   // [KM Bremse_1.BR1_BKV_active b2.0 + BR1_Rad_kmh low bits1-7, 0.01 km/h] [REA audit_gen2_7h0 #24: b2.0 read, speed not]
  frame.data[3] = 0xFE;                   // [KM Bremse_1.BR1_Rad_kmh high] [REA audit_gen2_7h0 #24: not read]
  frame.data[4] = 0xFE;                   // [KM Bremse_1.BR1_ASRMo_sl 0.39 %] [REA audit_gen2_7h0 #24: not read]
  frame.data[5] = 0xFE;                   // [KM Bremse_1.BR1_ASRMo_fa 0.39 % (m0) / BR1_MSR_Mo_inv (m1)] [REA audit_gen2_7h0 #24: not read]
  frame.data[6] = 0x00;                   // [KM Bremse_1.BR1_MSR_Mo 0.39 %] [REA audit_gen2_7h0 #24/F7: NOT read (old "low nibble read" was wrong)]
  frame.data[7] = BRAKES1_counter & 0x0F; // [KM Bremse_1.BR1_Zaehler b7.0-3, BR1_ESPASR_passive b7.5 = 0] [REA audit_gen2_7h0 #17/#18/#24: b7.0-3 alive counter (repeat = frame absent), b7.5 read]
  if (++BRAKES1_counter > 0xF)
    BRAKES1_counter = 0;
  standaloneTx(frame);

  // PQ Bremse_2 (0x5A0, DLC 8) - ESP/ABS sensor broadcast (vw_pq.dbc: Bremse_2).
  // Gen2 (OEM code, spec_gen2_7h0 §3) only reads b0, b1 bit0 and b7 bit7 - the
  // lock-adjusted b4/b5 below are ignored by the unit (harmless, left as they were).
  frame.identifier = BRAKES2_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x7F;                                        // 0x7F = 0 g [KM Bremse_2.Querbeschleunigung (m0) 0.01 g -1.27 / Timer (m1) 0.04 us] [REA audit_gen2_7h0 #32: (b0-0x7F) -> E3D0 only if b1.0 set]
  frame.data[1] = 0xAE;                                        // b1.0 = 0 [KM Bremse_2.Querbeschl__TimerTic b1.0 (mux) + mittlere_Raddrehzahl__Bremse_2 low bits9-15 0.002 U/s] [REA audit_gen2_7h0 #32: b1.0 read (gates b0)]
  frame.data[2] = 0x3D;                                        // [KM Bremse_2.mittlere_Raddrehzahl__Bremse_2 high] [REA spec_gen2_7h0 §3: not read]
  frame.data[3] = BRAKES2_counter;                             // rolling counter [KM Bremse_2.Zeitstempel low, 1 tic] [REA spec_gen2_7h0 §3: not read]
  frame.data[4] = getLockTargetAdjustedValue(0x7F, false); // [KM Bremse_2.Zeitstempel high] [REA spec_gen2_7h0 §3: not read - the lock scaling here does nothing]
  frame.data[5] = getLockTargetAdjustedValue(0xFE, false); // [KM Bremse_2.Wegimpulse_Vorderachse low 8 of 11 bits] [REA spec_gen2_7h0 §3: not read - the lock scaling here does nothing]
  frame.data[6] = 0x5E;                                        // [KM Bremse_2.Wegimpulse_Vorderachse high b6.0-2, Wegimpulszaehlerstatus b6.3, Fehlerspeichereintrag b6.4, Warnlampe_DDS b6.5, Fehlerstatus_Wegimpulse b6.7] [REA spec_gen2_7h0 §3: not read]
  frame.data[7] = 0x2B;                                        // b7.7 = 0 [KM Bremse_2.Impulszahl b7.0-5, gemessene_Querbeschleunigung b7.7] [REA audit_gen2_7h0 #32: b7.7 read (role not decoded)]
  BRAKES2_counter = BRAKES2_counter + 10;
  if (BRAKES2_counter > 0xF7)
    BRAKES2_counter = 7;
  standaloneTx(frame);

  // PQ Bremse_3 (0x4A0, DLC 8) - per-wheel speeds (see Gen1 for signal layout).
  // The unit works out slip from these (front average - rear average). Front wheels a
  // little faster than the rear = fake slip, so the unit's own request is above the
  // Bremse_4 ceiling and the lock request decides. Full whenever lock is wanted (gen2DemandValue).
  // Safety: synthetic speeds - the unit's slip protection never sees real slip in standalone.
  frame.identifier = BRAKES3_ID;
  frame.data_length_code = 8;
  frame.data[0] = gen2DemandValue(0xFE); // front left low [KM Bremse_3.Radgeschw__VL bits1-15 0.01 km/h, b0.0 Frei] [REA audit_gen2_7h0 #25/#45: (hi<<7)|(lo>>1); front avg - rear avg = slip E420]
  frame.data[1] = 0x0A;                  // [KM Bremse_3.Radgeschw__VL high] [REA audit_gen2_7h0 #25: valid < 0x7F80]
  frame.data[2] = gen2DemandValue(0xFE); // front right low [KM Bremse_3.Radgeschw__VR bits17-31 0.01 km/h] [REA audit_gen2_7h0 #45: front avg]
  frame.data[3] = 0x0A;                  // [KM Bremse_3.Radgeschw__VR high] [REA audit_gen2_7h0 #25: valid < 0x7F80]
  frame.data[4] = 0x00;                  // rear left low [KM Bremse_3.Radgeschw__HL bits33-47 0.01 km/h] [REA audit_gen2_7h0 #45: rear avg]
  frame.data[5] = 0x0A;                  // [KM Bremse_3.Radgeschw__HL high] [REA audit_gen2_7h0 #25: valid < 0x7F80]
  frame.data[6] = 0x00;                  // rear right low [KM Bremse_3.Radgeschw__HR bits49-63 0.01 km/h] [REA audit_gen2_7h0 #45: rear avg]
  frame.data[7] = 0x0A;                  // [KM Bremse_3.Radgeschw__HR high] [REA audit_gen2_7h0 #25/#46: valid < 0x7F80; no Bremse_3 = request forced to 0]
  standaloneTx(frame);

  // PQ Bremse_4 (0x2A0) - THE LOCK LOCK REQUEST (OEM code: spec_gen2_7h0 §2.3, audit_gen2_7h0).
  // The unit copies 3 bytes and uses only b0: a ceiling (b0 - 0x80) x 28 on its clutch
  // request, in the same 0x80-offset units as the engagement it sends back (0x2C0 byte 1).
  //   0x7F = 0 % (clutch open), 0x80 = ramp down to open (-40/cycle, never sent),
  //   0x81..0xE9 = 1..99 % (gen2LockRequestByte), 0xFD = 100 %; full valve from 0xD6 (7H0) / 0xEA (1K0).
  //   0xFE / 0xFF / below 0x7F = "no limit" (pre-V11 sent 0x00 here = no limit).
  frame.identifier = BRAKES4_ID;
  frame.data_length_code = 8;                    // unit needs at least 3
  frame.data[0] = gen2LockRequestByte();             // lock ceiling 0x7F / 0x81..0xE9 / 0xFD [KM Bremse_4.ABS_Vorgabewert_mitte_Kupplungs 3 Nm/min -381] [REA audit_gen2_7h0 #48/#49/#55: E34A = (b0-0x80) x 28; full valve from 0xD6 7H0 / 0xEA 1K0]
  frame.data[1] = 0x00;                          // 0 as before [KM Bremse_4.ABS_Vorgabewert_hinten_Kupplung 0.7874 %] [REA audit_gen2_7h0 #30: copied, getter has no caller]
  frame.data[2] = 0x00;                          // 0 as before [KM Bremse_4.Einheit_Kupplungssteifigkeit b2.0, rest Frei] [REA audit_gen2_7h0 #30: copied, never used]
  frame.data[3] = 0x00;                          // [KM - (Bremse_4 is DLC 3)] [REA spec_gen2_7h0 §1: not copied (RX DLC 3)]
  frame.data[4] = 0x00;                          // [KM -] [REA spec_gen2_7h0 §1: not copied]
  frame.data[5] = 0x00;                          // [KM -] [REA spec_gen2_7h0 §1: not copied]
  frame.data[6] = BRAKES4_counter;               // kept as before [KM -] [REA spec_gen2_7h0 §1: not copied]
  frame.data[7] = BRAKES4_counter;               // kept as before [KM -] [REA spec_gen2_7h0 §1: not copied]
  BRAKES4_counter = BRAKES4_counter + 10;
  if (BRAKES4_counter > 0xF0)
  {
    BRAKES4_counter = 0;
  }
  standaloneTx(frame);

  // PQ Bremse_5 (0x4A8, DLC 8) - ESP brake-event broadcast.
  // vw_pq.dbc: BR5_Giergeschw / BR5_Gierrate (yaw rate * 0.01 deg/s),
  //            BR5_Bremsdruck (brake pressure * 0.1 bar), BR5_Bremslicht,
  //            BR5_Notbremsung (emergency-brake), BR5_HDC_bereit (hill-descent-ready).
  // The unit reads a 12-bit value in b2-3 plus flags, and checks b6 bits 4-7 as an
  // alive counter - it must change every frame (spec_gen2_7h0 §1). No checksum checked.
  frame.identifier = BRAKES5_ID;
  frame.data_length_code = 8;                    // unit needs at least 7
  frame.data[0] = 0xFE;                          // yaw rate low [KM Bremse_5.BR5_Giergeschw low of 14 bits, 0.01 deg/s] [REA audit_gen2_7h0 #29: not read]
  frame.data[1] = 0x7F;                          // [KM Bremse_5.BR5_Giergeschw high b1.0-5, BR5_Sta_Gierrate b1.6, BR5_Vorzeichen b1.7] [REA audit_gen2_7h0 #29: not read]
  frame.data[2] = 0x03;                          // 0.3 bar [KM Bremse_5.BR5_Bremsdruck low of 12 bits, 0.1 bar] [REA audit_gen2_7h0 #29: sig85 12-bit b2-3 read]
  frame.data[3] = 0x00;                          // [KM Bremse_5.BR5_Bremsdruck high b3.0-3, BR5_Stillstand b3.4, BR5_Druckvalid b3.5, BR5_Sta_Druck b3.6, BR5_Sign_Druck b3.7] [REA audit_gen2_7h0 #29: b3.0-3, b3.5, b3.7 read]
  frame.data[4] = 0x00;                          // [KM Bremse_5.ESP_Rollenmodus b4.2, ESP_Stat_FallBack_eBKV b4.3, ESP_Anforderung_EPB b4.4-5, ESP_Autohold b4.6-7] [REA audit_gen2_7h0 #29: not read]
  frame.data[5] = 0x00;                          // [KM Bremse_5.BR5_Anhi_Sta b5.0 .. BR5_Bremslicht b5.3, BR5_Notbremsung b5.4 .. BR5_AWV2_Fehler b5.7] [REA audit_gen2_7h0 #29: not read]
  BRAKES5_counter = BRAKES5_counter + 0x10;      // next alive count in the top 4 bits (wraps 0xF? -> 0x0? on its own)
  frame.data[6] = BRAKES5_counter & 0xF0;        // (was +10 per frame, which repeated values) [KM Bremse_5.COUNTER b6.4-7; BR5_ZT_Rueckk / ANB_CM / HDC_bereit / ECD_Lampe b6.0-3] [REA audit_gen2_7h0 #17/#18: b6.4-7 alive counter, repeat = frame absent]
  frame.data[7] = BRAKES5_counter2;              // [KM Bremse_5.CHECKSUM] [REA audit_gen2_7h0 #19: not copied (RX DLC 7), no checksum]
  BRAKES5_counter2 = BRAKES5_counter2 + 10;
  if (BRAKES5_counter2 > 0xF3)
  {
    BRAKES5_counter2 = 3;
  }
  standaloneTx(frame);

  // PQ Bremse_9 (0x0AE, DLC 8) - extended ABS/ESP frame (NOT in vw_pq.dbc).
  // NOT received by the 7H0 / 1K0 (OEM code: no receive object) - block off by default.
  // Older bench note: "only data[6] really matters (0x02 OK, 0x01 changes haldex behaviour)".
  frame.identifier = BRAKES9_ID;    // 0x0AE - not received by Gen2
  frame.data_length_code = 8;       // DLC 8
  frame.data[0] = BRAKES9_counter;  // checksum / rolling counter A [KM - (0x0AE not in vw_pq.dbc)] [REA audit_gen2_7h0 F15: 0x0AE not received]
  frame.data[1] = BRAKES9_counter2; // checksum / rolling counter B [KM -] [REA audit_gen2_7h0 F15: not received]
  frame.data[2] = 0x00;             // [KM -] [REA audit_gen2_7h0 F15: not received]
  frame.data[3] = 0x00;             // [KM -] [REA audit_gen2_7h0 F15: not received]
  frame.data[4] = 0x00;             // [KM -] [REA audit_gen2_7h0 F15: not received]
  frame.data[5] = 0x00;             // [KM -] [REA audit_gen2_7h0 F15: not received]
  frame.data[6] = 0x02;             // old bench "mode-control byte" [KM -] [REA audit_gen2_7h0 F15: not received - the old note cannot be a 7H0/1K0 effect]
  frame.data[7] = 0x00;             // [KM -] [REA audit_gen2_7h0 F15: not received]
  standaloneTx(frame);
  BRAKES9_counter = BRAKES9_counter + 10;
  if (BRAKES9_counter > 0xF1)
  {
    BRAKES9_counter = 0x11;
  }
  BRAKES9_counter2 = BRAKES9_counter2 + 10;
  if (BRAKES9_counter2 > 0xF0)
  {
    BRAKES9_counter2 = 0x00;
  }

  // PQ LW_1 / mLW_1 (0x0C2, DLC 8) - steering-angle broadcast (vw_pq.dbc: LW_1).
  // Lenkrad = steering wheel; LRW = Lenkradwinkel.
  frame.identifier = mLW_1;
  frame.data_length_code = 8;
  frame.data[0] = 0x20;          // [KM Lenkwinkel_1.LW1_LRW low of 15 bits, 0.04375 deg] [REA audit_gen2_7h0 #28: sig118 angle read]
  frame.data[1] = 0x00;          // [KM Lenkwinkel_1.LW1_LRW high b1.0-6, LW1_LRW_Sign b1.7] [REA audit_gen2_7h0 #28: angle + sign read]
  frame.data[2] = 0x00;          // [KM Lenkwinkel_1.LW1_Lenk_Gesch low of 15 bits, 0.04375 deg/s] [REA audit_gen2_7h0 #28: not read]
  frame.data[3] = 0x00;          // [KM Lenkwinkel_1.LW1_Lenk_Gesch high b3.0-6, LW1_Gesch_Sign b3.7] [REA audit_gen2_7h0 #28: not read]
  frame.data[4] = 0x80;          // [KM Lenkwinkel_1.LW1_ID] [REA audit_gen2_7h0 #28: not read]
  frame.data[5] = mLW_1_counter; // steps of 0x10, low bits 0 [KM Lenkwinkel_1.LW1_Initquelle b5.0, LW1_Status b5.1-2, LW1_Sta_KL30 b5.3, LW1_Zaehler b5.4-7] [REA audit_gen2_7h0 #17/#28: status <= 1 = valid; b5.4-7 alive counter, repeat = frame absent]
  frame.data[6] = 0x00;          // [KM Lenkwinkel_1.LW1_Kodier] [REA spec_gen2_7h0 §1: not copied (RX DLC 6)]
  mLW_1_crc = 255 - (frame.data[0] + frame.data[1] + frame.data[2] + frame.data[3] + frame.data[5]);
  frame.data[7] = mLW_1_crc; // inverted sum over data[0..3]+data[5] [KM Lenkwinkel_1.LW1_CRC] [REA audit_gen2_7h0 #19: not copied (RX DLC 6), no checksum]
  mLW_1_counter = mLW_1_counter + 16;
  if (mLW_1_counter >= 0xF0)
    mLW_1_counter = 0;
  standaloneTx(frame);
}

void Gen2_frames20()
{
  twai_message_t frame = {};
  // PQ Motor_1 (0x280, DLC 8) - engine ECU broadcast (vw_pq.dbc: Motor_1).
  // V11 (OEM code, spec_gen2_7h0 §2.2 / §5): the unit reads b1 inner torque (ADDED to its
  // torque estimate), b6 loss torque (SUBTRACTED from b1), b5 pedal, b2-3 rpm and b0 bit3.
  // Before V11 b6 rose with lock and cancelled b1, and the rpm high byte was scaled with
  // lock (low lock = rpm near 0 = pump off). Now: rpm fixed, b6 = 0, b1/b5 full while lock
  // is wanted - the Bremse_4 lock request (frames10) sets the actual lock.
  frame.identifier = MOTOR1_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x08;                  // as before [KM Motor_1.Kupplungsschalter b0.3 (+ Leergasinformation b0.0 .. Momentenangaben_ungenau b0.7)] [REA audit_gen2_7h0 #22: only b0.3 read -> E472.2 status copy]
  frame.data[1] = gen2DemandValue(0xFE); // full while lock wanted [KM Motor_1.inneres_Motor_Moment 0.39 % MDI] [REA audit_gen2_7h0 #34/#41: E26E = E280 x b1/255, PLUS side of (b1 - b6); 0xFF -> uses b6 value]
  frame.data[2] = 0x20;                  // [KM Motor_1.Motordrehzahl low, 0.25 rpm] [REA audit_gen2_7h0 #35: raw rpm E41E; >= 0xFF00 -> 0x1F40]
  frame.data[3] = 0x4E;                  // 0x4E20 = 5000 rpm, fixed (above pump-on) [KM Motor_1.Motordrehzahl high] [REA audit_gen2_7h0 #27/#42: raw thresholds 0x640/0x960; - rpm/64 term; ratio E396]
  frame.data[4] = 0xFA;                  // [KM Motor_1.inneres_Motor_Moment_ohne_exter 0.39 % MDI] [REA audit_gen2_7h0 #23: not read]
  frame.data[5] = gen2DemandValue(0xFE); // full while lock wanted [KM Motor_1.Fahrpedalwert_oder_Drosselklapp 0.4 %] [REA audit_gen2_7h0 #36: E354 = b5 x 0x6600/255 (slew + pedal table)]
  frame.data[6] = 0x00;                  // 0, never 0xFF [KM Motor_1.mechanisches_Motor_Verlustmomen 0.39 % MDI] [REA audit_gen2_7h0 #33/#41: E3BE = E280 x b6/255, SUBTRACTED from b1; 0xFF -> 30 Nm + error]
  frame.data[7] = 0xFA;                  // [KM Motor_1.Fahrerwunschmoment 0.39 % MDI] [REA audit_gen2_7h0 #23: never read]
  standaloneTx(frame);

  // PQ Motor_2 (0x288, DLC 8) - secondary engine broadcast (vw_pq.dbc: Motor_2).
  // Unit reads b0 (mux 3 = max engine torque; mux 0 here leaves its 300 Nm default)
  // and b2 bits 0/1 (brake vote).
  frame.identifier = MOTOR2_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // mux 0 [KM Motor_2.MO2_Mp_Code b0.6-7 / MO2_CAN_Vers (m0) b0.0-5] [REA audit_gen2_7h0 #38/#39: E280 only set at mux 3 -> stays 0x4B00 = 300 Nm]
  frame.data[1] = 0x30; // [KM Motor_2.MO2_Kuehlm_T 0.75 -48 degC] [REA spec_gen2_7h0 §3: not read]
  frame.data[2] = 0x00; // all off [KM Motor_2.MO2_BLS b2.0, MO2_BTS b2.1, MO2_Sta_Kuehlm .. MO2_Sta_GRA b2.2-7] [REA audit_gen2_7h0 #32 / spec §3: b2.0 + b2.1 read (brake vote)]
  if (brakeForHaldex())
  {
    frame.data[2] = frame.data[2] | (1 << 0); // bit0 MO2_BLS: Follow Brake on and the brake is pressed
  }
  frame.data[3] = 0x0A; // [KM Motor_2.TSK_Limiter b3.1, MO2_GRAregelt b3.2, MO2_Sport_Error b3.3, MO2_OffRoad b3.4, MO2_RME_Gehalt b3.5-7] [REA spec_gen2_7h0 §1: not copied (RX DLC 3)]
  frame.data[4] = 0x0A; // [KM Motor_2.MO2_GRA_Soll 1.28 km/h] [REA: not copied (RX DLC 3)]
  frame.data[5] = 0x10; // [KM Motor_2.MO2_LL_Solldz 10 rpm] [REA: not copied (RX DLC 3)]
  frame.data[6] = 0xFE; // [KM Motor_2.MO2_Begr_Mo 0.39 %] [REA: not copied (RX DLC 3)]
  frame.data[7] = 0xFE; // [KM Motor_2.MO2_Mo_ZWR 0.39 %] [REA: not copied (RX DLC 3)]
  standaloneTx(frame);

  // PQ Motor_5 (0x480, DLC 8) - tertiary engine broadcast (vw_pq.dbc: Motor_5, multiplexed).
  // The unit only takes b0 (bits 0-5 value, bits 6-7 mux) - the rest is ignored.
  frame.identifier = MOTOR5_ID;   // 0x480
  frame.data_length_code = 8;     // DLC 8
  frame.data[0] = 0xFE;           // mux 3; old car-test note: ASR 0x04 sets bit 4, 0x08 removes it (coupling open/closed) [KM Motor_5.MO5_Mp_Code b0.6-7, m3 = MO5_Abgastyp/OBD/MO5_DPF_reg b0.0-5] [REA spec_gen2_7h0 §3: sig93 b0.0-5 + sig94 b0.6-7 read, role not decoded]
  frame.data[1] = 0x00;           // (old note: "can use to disable (>130 dec)") [KM Motor_5.MO5_Vorgluehen b1.1, MO5_E_Gas b1.2, MO5_OBD_2 b1.3 .. MO5_KliKo_Red b1.7] [REA spec_gen2_7h0 §1: not copied (RX DLC 1)]
  frame.data[2] = 0x00;           // [KM Motor_5.MO5_Verbrauch low, 1 ul] [REA: not copied (RX DLC 1)]
  frame.data[3] = 0x00;           // [KM Motor_5.MO5_Verbrauch high b3.0-6, MO5_UeberlVerb b3.7] [REA: not copied (RX DLC 1)]
  frame.data[4] = 0x00;           // [KM Motor_5.MO5_Luefter 0.4 %] [REA: not copied (RX DLC 1)]
  frame.data[5] = 0x00;           // [KM Motor_5.MO5_HLeuchte b5.0 .. MO5_Interlock b5.7] [REA: not copied (RX DLC 1)]
  frame.data[6] = 0x00;           // [KM Motor_5.MO5_Start b6.0 .. MO5_Motortext4 b6.7] [REA: not copied (RX DLC 1)]
  frame.data[7] = MOTOR5_counter; // rolling value [KM Motor_5.CHECKSUM] [REA: not copied (RX DLC 1), no checksum]
  MOTOR5_counter++;               // V11: was ++BRAKES1_counter (copy-paste bug that bumped Bremse_1's alive counter)
  standaloneTx(frame);

  // PQ Bremse_10 (0x3A0, DLC 8) - extended ABS/ESP frame (in "vw_pq (1).dbc" / vw_golf_mk4.dbc, not in opendbc vw_pq.dbc).
  // NOT received by the 7H0 / 1K0 (OEM code: no receive object) - block off by default.
  frame.identifier = BRAKES10_ID;   // 0x3A0 - not received by Gen2
  frame.data_length_code = 8;       // DLC 8
  frame.data[0] = 0xA6;             // static value [KM Bremse_10.B10_Checksumme] [REA audit_gen2_7h0 F15: 0x3A0 not received]
  frame.data[1] = BRAKES10_counter; // rolling counter [KM Bremse_10.B10_Zaehler b1.0-3, B10_QB_Wegimp_VL..HR b1.4-7] [REA: not received]
  frame.data[2] = 0x75;             // static value [KM Bremse_10.B10_Wegimp_VL low of 10 bits] [REA: not received]
  frame.data[3] = 0xD4;             // static value [KM Bremse_10.B10_Wegimp_VL high b3.0-1, B10_Wegimp_VR low b3.2-7] [REA: not received]
  frame.data[4] = 0x51;             // static value [KM Bremse_10.B10_Wegimp_VR high b4.0-3, B10_Wegimp_HL low b4.4-7] [REA: not received]
  frame.data[5] = 0x47;             // static value [KM Bremse_10.B10_Wegimp_HL high b5.0-5, B10_Wegimp_HR low b5.6-7] [REA: not received]
  frame.data[6] = 0x1D;             // static value [KM Bremse_10.B10_Wegimp_HR high] [REA: not received]
  frame.data[7] = 0x0F;             // fixed - NOT a checksum per the DBC [KM Bremse_10.B10_QB_Fahrtr_* b7.0-3, B10_Fahrtr_* b7.4-7] [REA: not received]
  BRAKES10_counter = BRAKES10_counter + 1;
  if (BRAKES10_counter > 0xF)
  {
    BRAKES10_counter = 0;
  }
  standaloneTx(frame);
}

void Gen2_frames25()
{
  twai_message_t frame = {};
  // PQ Kombi_1 (0x320, DLC 8) - instrument-cluster broadcast (vw_pq.dbc: Kombi_1).
  frame.identifier = mKombi_1;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // lamp bits [KM Kombi_1.Fahrertuer_4_1 b0.0 .. Oeldruck b0.2, Dynamische_Oeldruckwarnung b0.3 .. Vorgluehlampe b0.7] [REA spec_gen2_7h0 §3: not read]
  frame.data[1] = 0x00; // bit0 fluid warn off, bit1 released. V11: was 0x02 = handbrake PULLED [KM Kombi_1.Bremsinfo b1.0-1 = KMatrix V5.20.6F KO1_Bremsinfo b1.0 / KO1_Handbremse b1.1] [REA audit_gen2_7h0 #32 / spec §3: sig95 b1.1 handbrake vote]
  if (handbrakeForHaldex())
  {
    frame.data[1] = frame.data[1] | (1 << 1); // bit1 KO1_Handbremse: Follow Handbrake on and the handbrake is pulled
  }
  frame.data[2] = 0x00; // [KM Kombi_1.Tankinhalt b2.0-6 1 l, Tankwarnung b2.7] (old label "speed low" was wrong) [REA spec_gen2_7h0 §1: not copied (RX DLC 2)]
  frame.data[3] = 0x00; // [KM Kombi_1.Signalquelle_Geschwindigkeit b3.0 + Geschwindigkeit__Kombi_1_ low, 0.01 km/h] [REA: not copied (RX DLC 2)]
  frame.data[4] = 0x36; // (old label "Tankinhalt" was wrong) [KM Kombi_1.Geschwindigkeit__Kombi_1_ high] [REA: not copied (RX DLC 2)]
  frame.data[5] = 0x00; // [KM Kombi_1.ADR_Summer / Zeitluecke / Blinker b5.2-5 + Angezeigte_Geschwindigkeit low b5.6-7] [REA: not copied (RX DLC 2)]
  frame.data[6] = 0x00; // [KM Kombi_1.Angezeigte_Geschwindigkeit high, 0.32 km/h] [REA: not copied (RX DLC 2)]
  frame.data[7] = 0x00; // [KM Kombi_1.Frei_Kombi_1_3] [REA: not copied (RX DLC 2)]
  standaloneTx(frame);
}

void Gen2_frames100() {}
void Gen2_frames200() {}
void Gen2_frames1000() {}

void Gen4_frames10()
{
  twai_message_t frame = {};
  // PQ LW_1 / mLW_1 (0x0C2, DLC 8) - steering-angle replay (vw_pq.dbc: LW_1).
  frame.identifier = mLW_1;
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  // Byte layout as the 0BR reads it (spec_gen4_0br §3); replay table is fine (§5).
  // mLW_1_counter is shared with Gen2, which steps it in 0x10s: mask it to the 16-row
  // table so a generation change at runtime can't read past lws2 (V11.00.8)
  mLW_1_counter = mLW_1_counter & 0x0F;
  frame.data[0] = lws2[mLW_1_counter][0]; // [KM Lenkwinkel_1.LW1_LRW low, 15 bit 0.04375 deg] [REA 0BR: steering angle]
  frame.data[1] = lws2[mLW_1_counter][1]; // [KM LW1_LRW bits 8-14 + b1.7 LW1_LRW_Sign] [REA 0BR: angle + sign]
  frame.data[2] = lws2[mLW_1_counter][2]; // [KM LW1_Lenk_Gesch low, 0.04375 deg/s] [REA 0BR: not read]
  frame.data[3] = lws2[mLW_1_counter][3]; // [KM LW1_Lenk_Gesch high + b3.7 LW1_Gesch_Sign] [REA 0BR: not read]
  frame.data[4] = lws2[mLW_1_counter][4]; // [KM LW1_ID] [REA 0BR: read]
  frame.data[5] = lws2[mLW_1_counter][5]; // [KM b5.0 LW1_Initquelle, b5.1-2 LW1_Status, b5.3 LW1_Sta_KL30, b5.4-7 LW1_Zaehler] [REA 0BR: status b5.1-2 read, counter not checked]
  frame.data[6] = lws2[mLW_1_counter][6]; // [KM LW1_Kodier] [REA 0BR: not copied (RX len 6)]
  frame.data[7] = lws2[mLW_1_counter][7]; // [KM LW1_CRC] [REA 0BR: not copied (RX len 6)]

  mLW_1_counter++;
  if (mLW_1_counter > 15)
    mLW_1_counter = 0;
  standaloneTx(frame);

  // Speed shared by Bremse_1 and Bremse_3 this cycle (real, or a steady 10 km/h)
  uint16_t speedWord = gen4SpeedWord();

  // PQ Bremse_1 (0x1A0, DLC 8) - ABS/ESP main broadcast (vw_pq.dbc: Bremse_1).
  // The 0BR reads b0.0 ASR request, b0.2 ABS braking, b0.3 EDS, b0.4 ESP,
  // b1.3 / b1.4 and the b7 counter (not checked) - spec_gen4_0br §3. It does
  // NOT read the vehicle speed (b2-3) or b4-6.
  uint8_t bremse1Byte0 = 0x00; // no ASR / ABS / EDS / ESP intervention
  if (absBrakingForHaldex())
  {
    bremse1Byte0 = bremse1Byte0 | (1 << 2); // b0.2 BR1_ABS_Brems: the car's ABS is braking
  }
  frame.identifier = BRAKES1_ID;
  frame.data_length_code = 8;
  frame.data[0] = bremse1Byte0;                     // [KM Bremse_1 b0.0 BR1_ASR_Anf, b0.1 MSR_Anf, b0.2 BR1_ABS_Brems, b0.3 EDS_Ongr, b0.4 ESP_Ongr, b0.5-6 ASR_Ongr, b0.7 EBV] [REA 0BR: b0.0/0.2/0.3/0.4 read; b0.2 = ABS-braking latch -> state 4] (V11: was 0x20)
  frame.data[1] = 0x00;                             // [KM b1.0-2 lamps ABS/ASR/BK, b1.3 BR1_Lichtschalt, b1.4 BR1_StaDruckschw, b1.5-7] [REA 0BR: b1.3 + b1.4 read] (V11: was 0x40)
  frame.data[2] = (uint8_t)(speedWord & 0xFF);      // [KM b2.0 BR1_BKV_active, b2.1-7 BR1_Rad_kmh low 0.01 km/h] [REA 0BR: not read]
  frame.data[3] = (uint8_t)(speedWord >> 8);        // [KM BR1_Rad_kmh high] [REA 0BR: not read]
  frame.data[4] = 0xFE;                             // [KM BR1_ASRMo_sl 0.39 %] no ASR cut [REA 0BR: not read]
  frame.data[5] = 0xFE;                             // [KM BR1_ASRMo_fa (mux 0) / BR1_MSR_Mo_inv (mux 1) 0.39 %] [REA 0BR: not read]
  frame.data[6] = 0x00;                             // [KM BR1_MSR_Mo 0.39 %] no drag-torque request [REA 0BR: not read]
  frame.data[7] = BRAKES1_counter & 0x0F;           // [KM b7.0-3 BR1_Zaehler, b7.4 ASR_ESP, b7.5 ESPASR_passive, b7.6 Sta_ESP, b7.7 Ersatz_Kmh] [REA 0BR: counter read, not checked]
  BRAKES1_counter++;
  if (BRAKES1_counter > 0x0F)
    BRAKES1_counter = 0; // V11: was 10..0x1F, which jumped F -> A in the nibble (§7 #21)
  standaloneTx(frame);

  // PQ Bremse_3 (0x4A0, DLC 8) - per-wheel speeds (vw_pq.dbc: Bremse_3).
  // V11: four EQUAL, steady speeds (real vehicle speed when we have it). Before
  // V11 these were lock-scaled to fake slip; the lock request is Bremse_4 now (§7 #9).
  frame.identifier = BRAKES3_ID;
  frame.data_length_code = 8;
  frame.data[0] = (uint8_t)(speedWord & 0xFF); // [KM Bremse_3 b0.0 Frei_Bremse_3_1, b0.1-7 Radgeschw__VL_4_1 low 0.01 km/h] [REA 0BR audit #12: word>>1 = wheel speed (own strategy / slip)]
  frame.data[1] = (uint8_t)(speedWord >> 8);   // [KM Radgeschw__VL_4_1 high] [REA 0BR: VL]
  frame.data[2] = (uint8_t)(speedWord & 0xFF); // [KM b2.0 Frei_Bremse_3_2, b2.1-7 Radgeschw__VR_4_1 low] [REA 0BR: VR, bit 0 ignored]
  frame.data[3] = (uint8_t)(speedWord >> 8);   // [KM Radgeschw__VR_4_1 high] [REA 0BR: VR]
  frame.data[4] = (uint8_t)(speedWord & 0xFF); // [KM b4.0 Frei_Bremse_3_3, b4.1-7 Radgeschw__HL_4_1 low] [REA 0BR: HL, bit 0 ignored]
  frame.data[5] = (uint8_t)(speedWord >> 8);   // [KM Radgeschw__HL_4_1 high] [REA 0BR: HL]
  frame.data[6] = (uint8_t)(speedWord & 0xFF); // [KM b6.0 Frei_Bremse_3_4, b6.1-7 Radgeschw__HR_4_1 low] [REA 0BR: HR, bit 0 ignored]
  frame.data[7] = (uint8_t)(speedWord >> 8);   // [KM Radgeschw__HR_4_1 high] [REA 0BR: HR]
  standaloneTx(frame);

  // Bremse_4 byte 0 = the 0BR's direct clutch-torque lock request (OEM code:
  // spec_gen4_0br §2.2 / §2.6):
  //   0 %     -> 0x7F  valid zero request, clutch commanded fully open
  //   1-99 %  -> 0x80 + % x 1.25 (linear, 112 internal torque units per step,
  //              1/16 Nm units = 7 Nm per step [REA audit_gen4_0br #41, INF])
  //   100 %   -> 0xFE  "mode 4": request = last cycle's capacity F1B00 (ramped,
  //              target 32000) - always >= 0xFD's fixed 14000, so 0xFE is right
  //              at 100 % in OEM and standalone alike [REA audit_gen4_0br §8, #43]
  // The unit still caps this: final = min(request, F1AFE limit), and F1AFE only
  // ramps up while Motor_1 rpm > ~401 rpm (off < ~200) and no over-temperature
  // [REA audit_gen4_0br #28]; states 5 / 6 (own fault reaction) outrank it (#26).
  // V11: was 0x7F x CF/100 (below 0x7F = ignored) and 0x7F at 100 % outside
  // 50:50 = clutch open (§7 #1 / #2). Learn sweeps this byte too (§7 #3).
  appliedTorque = vagLockRequestByte(GEN4_REQUEST_FULL, GEN4_REQUEST_FULL_BYTE); // Gen4: 0xFE = full lock

  // PQ Bremse_4 (0x2A0, DLC 8) - only byte 0 is read; byte 6 counter and byte
  // 7 checksum are ignored by the 0BR (§7 #20), kept as before.
  frame.identifier = BRAKES4_ID;
  frame.data_length_code = 8;
  frame.data[0] = appliedTorque;   // [KM Bremse_4.ABS_Vorgabewert_mitte_Kupplungs (3,-381) Nm/min] [REA audit_gen4_0br #14: 0x7F open, 0x80-0xFD (b-0x80)x112, 0xFE mode 4, else invalid]
  frame.data[1] = 0x00;            // [KM ABS_Vorgabewert_hinten_Kupplung 0.7874 %] [REA 0BR: not read]
  frame.data[2] = 0x00;            // [KM b2.0 Einheit_Kupplungssteifigkeit, b2.1-7 Frei_Bremse_4_1] [REA 0BR: not read]
  frame.data[3] = 0x64;            // [KM -] [REA 0BR: not read]
  frame.data[4] = 0x00;            // [KM -] [REA 0BR: not read]
  frame.data[5] = 0x00;            // [KM -] [REA 0BR: not read]
  frame.data[6] = BRAKES4_counter; // [KM -] rolling counter, high nibble [REA audit_gen4_0br #16: sig 0x6D defined, never loaded]
  BRAKES4_crc = 0;
  for (uint8_t i = 0; i < 7; i++)
  {
    BRAKES4_crc ^= frame.data[i];
  }
  frame.data[7] = BRAKES4_crc; // [KM -] XOR over data[0..6] [REA 0BR: not copied (RX len 7)]
  BRAKES4_counter = BRAKES4_counter + 16;
  if (BRAKES4_counter > 0xF0)
    BRAKES4_counter = 0x00;
  standaloneTx(frame);

  // PQ Motor_1 (0x280, DLC 8) - engine ECU broadcast (vw_pq.dbc: Motor_1).
  // V11: steady values, no longer lock-scaled (§5). Not only the own strategy:
  // Motor_1 rpm (b2-3) is the 0BR's engine-running switch for ALL lock - its
  // torque limit F1AFE ramps up only while rpm > ~401 rpm and ramps out below
  // ~200 rpm, final = min(Bremse_4 request, F1AFE) [REA audit_gen4_0br #28 /
  // FI-1]. So keep the steady 2000 rpm here (> 401). Other bytes: own strategy.
  // Light-cruise figures chosen as plausible - not from a log.
  frame.identifier = MOTOR1_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // [KM Motor_1 b0.0 Leergasinformation .. b0.3 Kupplungsschalter .. b0.7 Momentenangaben_ungenau] [REA 0BR audit #19: b0.3 read (F13D3 = !bit, clutch pedal); other bits not read] (V11: was left over from Bremse_4 = "pressed", §7 #4)
  frame.data[1] = 0x20; // [KM inneres_Motor_Moment 0.39 % MDI] [REA 0BR: inner torque x F13C0/256, own strategy]
  frame.data[2] = 0x40; // [KM Motordrehzahl low, 16 bit 0.25 rpm] 0x1F40 = 2000 rpm [REA audit_gen4_0br #17/#28: rpm -> F13BE; > 2688 (~401 rpm) enables lock, < 1344 (~200) ramps it out]
  frame.data[3] = 0x1F; // [KM Motordrehzahl high] [REA 0BR: rpm, as b2]
  frame.data[4] = 0x20; // [KM inneres_Motor_Moment_ohne_exter 0.39 % MDI] [REA 0BR: not read]
  frame.data[5] = 0x20; // [KM Fahrpedalwert_oder_Drosselklapp 0.4 %] 12.8 % [REA 0BR: pedal F13D4, own strategy]
  frame.data[6] = 0x08; // [KM mechanisches_Motor_Verlustmomen 0.39 % MDI] [REA 0BR: loss torque, own strategy]
  frame.data[7] = 0x20; // [KM Fahrerwunschmoment 0.39 % MDI] [REA 0BR: driver demand F13C8, own strategy]
  standaloneTx(frame);
}

void Gen4_frames20()
{
  twai_message_t frame = {};
  // PQ Bremse_2 (0x5A0, DLC 8) - ESP/ABS sensor broadcast (vw_pq.dbc: Bremse_2).
  // Gen4 reads b0, b1.0, b3 bits 4-7, b6.6, b7.7 - not b4 (spec_gen4_0br §3).
  frame.identifier = BRAKES2_ID;
  frame.data_length_code = 8;
  // V11.00.13: byte names below corrected from vw_pq.dbc (the old ones were shifted).
  frame.data[0] = 0x80;                                        // [KM Bremse_2.Querbeschleunigung (mux 0) 0.01 g -1.27 / Timer (mux 1)] 0x80 = +0.01 g [REA 0BR: read (lat accel / timer per mux)]
  frame.data[1] = 0x7A;                                        // [KM b1.0 Querbeschl__TimerTic (mux), b1.1-7 mittlere_Raddrehzahl__Bremse_2 low 0.002 U/s] [REA 0BR: b1.0 read, rest not read]
  frame.data[2] = 0x05;                                        // [KM mittlere_Raddrehzahl__Bremse_2 high] [REA 0BR: not read]
  frame.data[3] = BRAKES2_counter;                             // [KM Zeitstempel low, 16 bit tics] counter in bits 4-7 [REA 0BR: b3 bits 4-7 read]
  frame.data[4] = getLockTargetAdjustedValue(0x7F, false); // [KM Zeitstempel high] (lock-scaled leftover, no effect) [REA 0BR: not read]
  frame.data[5] = 0xCA;                                        // [KM Wegimpulse_Vorderachse low, 11 bit] [REA 0BR: not read]
  frame.data[6] = 0x1B;                                        // [KM b6.0-2 Wegimpulse high, b6.3 Wegimpulszaehlerstatus, b6.4 Fehlerspeichereintrag_Bremse, b6.5 Warnlampe_DDS, b6.6 Frei_Bremse_2_5, b6.7 Fehlerstatus_Wegimpulse] [REA 0BR: b6.6 read (= 0 here)]
  frame.data[7] = 0xAB;                                        // [KM b7.0-5 Impulszahl, b7.6 Frei, b7.7 gemessene_Querbeschleunigung] b7.7 = 1 here [REA 0BR: b7.7 read, use not traced]
  standaloneTx(frame);
  BRAKES2_counter = BRAKES2_counter + 16;
  if (BRAKES2_counter > 0xF0)
    BRAKES2_counter = 0;

  // ---- V11.00.0: frames the 0BR monitors that we never sent (spec_gen4_0br §3 / §7 #5).
  // Each has a 50-tick alive window in the unit; 20 ms is well inside it.
  // Content: K-matrix value tables (vw_pq.dbc) with steady, plausible values -
  // no PQ log of these frames was available.

  // PQ Motor_2 (0x288, DLC 8). Gen4 reads b0 (mux), b2.0 brake light, b2.1, b2.4.
  uint8_t motor2Byte2 = 0x10; // b2.4 MO2_Sta_No_Bet = 1: engine in normal operation
  if (brakeForHaldex())
  {
    motor2Byte2 = motor2Byte2 | (1 << 0); // b2.0 MO2_BLS: brake light on (Follow Brake)
  }
  frame.identifier = MOTOR2_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0xDE;        // [KM Motor_2 b0.6-7 mux, b0.0-5 MO2_max_Mo (mux 3) 10 Nm] 0x1E = 300 Nm [REA 0BR: mux 3 -> max torque = b0[5:0] x 40 (F13C0, own-strategy torque scale)]
  frame.data[1] = 0xB8;        // [KM MO2_Kuehlm_T 0.75 -48 degC] 90 degC [REA 0BR: not read]
  frame.data[2] = motor2Byte2; // [KM b2.0 MO2_BLS, b2.1 MO2_BTS, b2.2-3 Sta_Kuehlm/Klima, b2.4 MO2_Sta_No_Bet, b2.5 TSK, b2.6-7 Sta_GRA] [REA 0BR: b2.0 brake light (state 2), b2.1, b2.4 read]
  frame.data[3] = 0x00;        // [KM b3.1 TSK_Limiter, b3.2 GRAregelt, b3.3 Sport_Error, b3.4 OffRoad, b3.5-7 RME_Gehalt] [REA 0BR: not copied (RX len 3)]
  frame.data[4] = 0x00;        // [KM MO2_GRA_Soll 1.28 km/h] [REA 0BR: not copied]
  frame.data[5] = 0x50;        // [KM MO2_LL_Solldz 10 rpm] 800 rpm [REA 0BR: not copied]
  frame.data[6] = 0xFE;        // [KM MO2_Begr_Mo 0.39 %] [REA 0BR: not copied]
  frame.data[7] = 0x00;        // [KM MO2_Mo_ZWR 0.39 %] [REA 0BR: not copied]
  standaloneTx(frame);

  // PQ Motor_5 (0x480, DLC 8). Must be present: without it the 0BR sets its own
  // limp bit (0x2C0 b0.4) - OEM code FUN_032880. Gen4 reads b0 (mux) and b6.3.
  frame.identifier = MOTOR5_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x1E; // [KM Motor_5 b0.6-7 MO5_Mp_Code, b0.0-5 MO5_max_Moment (mux 0) 10 Nm] 300 Nm [REA 0BR: mux data + code read]
  frame.data[1] = 0x00; // [KM b1.1 Vorgluehen, b1.2 E_Gas, b1.3 OBD_2, b1.4 Heissl, b1.5-7 Klima] [REA 0BR: not read]
  frame.data[2] = 0x00; // [KM MO5_Verbrauch low, 15 bit ul] [REA 0BR: not read]
  frame.data[3] = 0x00; // [KM MO5_Verbrauch high + b3.7 UeberlVerb] [REA 0BR: not read]
  frame.data[4] = 0x00; // [KM MO5_Luefter 0.4 %] [REA 0BR: not read]
  frame.data[5] = 0x00; // [KM b5.0 HLeuchte .. b5.7 Interlock] [REA 0BR: not read]
  frame.data[6] = 0x00; // [KM b6.0 Start, b6.1 Anlasser, b6.2 GRA_Hauptsch, b6.3 MO5_Momente, b6.4-7 Motortext] [REA 0BR: b6.3 read (torque reference x2)]
  frame.data[7] = 0x00; // [KM Motor_5.CHECKSUM] worked out below [REA 0BR: not copied (RX len 7)]
  for (uint8_t i = 0; i < 7; i++)
  {
    frame.data[7] ^= frame.data[i]; // XOR of bytes 0-6 (the 0BR copies only 7 bytes, so it never checks it)
  }
  standaloneTx(frame);

  // PQ Bremse_5 (0x4A8, DLC 8). Gen4 reads the yaw rate b0 + b1 bits 0-5
  // (0.01 deg/s), b1.6 status, b1.7 sign and the b6 counter (not checked).
  frame.identifier = BRAKES5_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;                            // [KM Bremse_5.BR5_Giergeschw low, 14 bit 0.01 deg/s] yaw 0 [REA 0BR: yaw rate]
  frame.data[1] = 0x00;                            // [KM b1.0-5 BR5_Giergeschw high, b1.6 BR5_Sta_Gierrate, b1.7 BR5_Vorzeichen] [REA 0BR: yaw + status + sign read]
  frame.data[2] = 0x00;                            // [KM BR5_Bremsdruck low, 12 bit 0.1 bar] [REA 0BR: not read]
  frame.data[3] = 0x00;                            // [KM b3.0-3 Bremsdruck high, b3.4 Stillstand, b3.5 Druckvalid, b3.6 Sta_Druck, b3.7 Sign_Druck] [REA 0BR: not read]
  frame.data[4] = 0x00;                            // [KM b4.2-7 ESP_Rollenmodus / FallBack_eBKV / Anforderung_EPB / Autohold] [REA 0BR: not read]
  frame.data[5] = 0x00;                            // [KM b5.0 Anhi_Sta .. b5.3 BR5_Bremslicht, b5.4 Notbremsung ..] [REA 0BR: not read]
  frame.data[6] = (uint8_t)(gen4Bremse5Counter << 4); // [KM b6.0-3 flags (ZT/ANB/HDC/ECD), b6.4-7 Bremse_5.COUNTER] [REA 0BR: counter read, not checked]
  frame.data[7] = 0x00;                            // [KM Bremse_5.CHECKSUM] worked out below [REA 0BR: not copied (RX len 7)]
  for (uint8_t i = 0; i < 7; i++)
  {
    frame.data[7] ^= frame.data[i]; // XOR of bytes 0-6 (not copied by the 0BR, DLC 7)
  }
  standaloneTx(frame);
  gen4Bremse5Counter++;
  if (gen4Bremse5Counter > 0x0F)
    gen4Bremse5Counter = 0;

  // PQ Bremse_8 (0x1AC, DLC 8). Gen4 reads b1 bits 0-3 counter (not checked),
  // b1.4, b1.7 and b4.4; b0 checksum is not checked (spec_gen4_0br §3).
  frame.identifier = BRAKES8_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;                       // [KM Bremse_8.BR8_Checksumme] worked out below [REA 0BR: not checked]
  frame.data[1] = gen4Bremse8Counter & 0x0F;  // [KM b1.0-3 BR8_Zaehler, b1.4 BR8_Sta_ACC_Anf, b1.5 Verz_EPB_akt, b1.6 Sta_Br_temp, b1.7 BR8_Sta_Br_Druck] [REA 0BR: counter (not checked), b1.4, b1.7 read]
  frame.data[2] = 0x7F;                       // [KM BR8_TolAbgl_HL 0.0488 % -6.2] 0x7F = 0 % [REA 0BR: not read]
  frame.data[3] = 0x7F;                       // [KM BR8_TolAbgl_HR 0.0488 % -6.2] 0 % [REA 0BR: not read]
  frame.data[4] = 0x69;                       // [KM BR8_Istbeschl low, 9 bit 0.02 m/s2 -7.22] 0x169 = 0 m/s2 [REA 0BR: b4.4 read (= 0)]
  frame.data[5] = 0x01;                       // [KM b5.0 Istbeschl bit 8, b5.1 Sta_HW_BLS, b5.2 QB_LBeschl, b5.3-4 ESC_Mode, b5.5-7] [REA 0BR: not copied (RX len 5)]
  frame.data[6] = 0x00;                       // [KM BR8_Laengsbeschl low, 10 bit 0.03125 m/s2 -16] 0x200 = 0 [REA 0BR: not copied]
  frame.data[7] = 0x02;                       // [KM b7.0-1 Laengsbeschl high, b7.2 Sta_ADR_BR, b7.3 BR8_Quattro, b7.4-7 VerzReg/BLS/EPB] [REA 0BR: not copied]
  for (uint8_t i = 1; i < 8; i++)
  {
    frame.data[0] ^= frame.data[i]; // XOR of bytes 1-7 (not checked by the 0BR)
  }
  standaloneTx(frame);
  gen4Bremse8Counter++;
  if (gen4Bremse8Counter > 0x0F)
    gen4Bremse8Counter = 0;

  // PQ Getriebe_1 (0x440, DLC 8). Gen4 reads b1 bits 0-3 target gear, b2 wheel /
  // crank torque ratio (0.1 per bit) and b6 bits 4-7 (spec_gen4_0br §3).
  frame.identifier = mGetriebe_1;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // [KM Getriebe_1 b0.0 GE1_Schaltung, b0.1 Sta_Schutz, b0.2 Klimakompr, b0.3-4 GE1_WK, b0.5-6 StSt_Info, b0.7 EGS_Anf] [REA 0BR: not read]
  frame.data[1] = 0x55; // [KM b1.0-3 GE1_Zielgang, b1.4-7 GE1_Wahl_Pos] 5th gear, D [REA 0BR: Zielgang read (gear)]
  frame.data[2] = 0x0A; // [KM GE1_Mrad_Mkurb 0.1] ratio 1.0 [REA 0BR: read (wheel / crank torque ratio)]
  frame.data[3] = 0x00; // [KM GE1_Soll_Mo 0.39 %] [REA 0BR: not read]
  frame.data[4] = 0x7F; // [KM GE1_Fahrwistd 0.249 -31.6] 0x7F = 0 [REA 0BR: not read]
  frame.data[5] = 0x00; // [KM b5.0-3 GE1_Notlauf, b5.4-5 GE1_Kuehlung, b5.7 Sta_OBD] [REA 0BR: not read]
  frame.data[6] = 0x00; // [KM b6.0 LaunchControl, b6.1 Infobit, b6.2 SleepInd, b6.3-6 Getriebe_1.COUNTER, b6.7 GE1_Fehlereintr] [REA 0BR: b6 bits 4-7 read] V11 TODO (bench): sent as 0 = no gearbox fault; try a counter here if the unit logs a Getriebe_1 fault
  frame.data[7] = 0x00; // [KM GE1_WaVerl_Mo 0.39 %] [REA 0BR: not copied (RX len 7)]
  standaloneTx(frame);
}

void Gen4_frames25()
{
  twai_message_t frame = {};
  // PQ Kombi_1 (0x320, DLC 8) - instrument-cluster broadcast (vw_pq.dbc: Kombi_1).
  // Gen4 reads b1.1 handbrake and b5 bits 0-3 (spec_gen4_0br §3).
  uint8_t kombi1Byte1 = 0x00; // b1.0 brake-fluid warning off, b1.1 handbrake released
  if (handbrakeForHaldex())
  {
    kombi1Byte1 = kombi1Byte1 | (1 << 1); // b1.1 KO1_Handbremse: handbrake pulled (Follow Handbrake)
  }
  frame.identifier = mKombi_1;
  frame.data_length_code = 8;
  frame.data[0] = 0x24;        // [KM Kombi_1 b0.0 Fahrertuer, b0.1 Fehlerstatus_Tank, b0.2 Oeldruck, b0.3 Dyn_Oeldruckwarnung, b0.4 Kuehlmittelmangel, b0.5 Heissleuchten_Vorwarnung, b0.6-7] [REA 0BR: not read]
  frame.data[1] = kombi1Byte1; // [KM b1.0-1 Bremsinfo (KMatrix: b1.1 KO1_Handbremse), b1.2 Ladekontroll, b1.3 Stellgliedtest, b1.7 Tankstop] [REA 0BR: b1.1 handbrake -> state 2 with brake light (cal 0x316C7)] (V11: was always 0)
  frame.data[2] = 0x1D;        // [KM b2.0-6 Tankinhalt 1 l, b2.7 Tankwarnung] 29 l [REA 0BR: not read]
  frame.data[3] = 0xB9;        // [KM b3.0 Signalquelle_Geschwindigkeit, b3.1-7 Geschwindigkeit__Kombi_1_ low 0.01 km/h] [REA 0BR: not read]
  frame.data[4] = 0x07;        // [KM Geschwindigkeit__Kombi_1_ high] ~9.9 km/h [REA 0BR: not read]
  frame.data[5] = 0x42;        // [KM b5.0-1 Frei_Kombi_1_2, b5.2 ADR_Summer, b5.3 Zeitluecke, b5.4-5 Blinker, b5.6-7 Angezeigte_Geschwindigkeit low 0.32 km/h] [REA 0BR: b5 bits 0-3 read]
  frame.data[6] = 0x09;        // [KM Angezeigte_Geschwindigkeit high] [REA 0BR: not copied (RX len 6)]
  frame.data[7] = 0x81;        // [KM Frei_Kombi_1_3] [REA 0BR: not copied]
  standaloneTx(frame);

  // PQ Kombi_3 (0x520, DLC 8) - cluster mux data / stand time / odometer (vw_pq.dbc: Kombi_3).
  // V11.00.13: byte names corrected from vw_pq.dbc (the old ones were shifted).
  frame.identifier = mKombi_3;
  frame.data_length_code = 8;
  frame.data[0] = 0x60; // [KM Kombi_3 b0 mux data (mux 1: b0.0-3 Derivat, b0.4-7 Generation)] [REA 0BR: mux data read]
  frame.data[1] = 0x43; // [KM b1.0-5 mux data (mux 1: Baureihe / Marke), b1.6-7 Kombi_Multiplex_Code] code 1 [REA 0BR: mux data bits 0-5 + code read]
  frame.data[2] = 0x01; // [KM b2.0-3 Schluesselinfo, b2.4-7 Frei_Kombi_3_1] [REA 0BR: not read]
  frame.data[3] = 0x10; // [KM Standzeit low, 15 bit x 4 s] [REA 0BR: stand time]
  frame.data[4] = 0x66; // [KM b4.0-6 Standzeit high, b4.7 Fehlerstatus_Standzeit] [REA 0BR: bits 0-6 stand time]
  frame.data[5] = 0xF1; // [KM Kilometerstand low, 20 bit km] [REA 0BR: not copied (RX len 5)]
  frame.data[6] = 0x03; // [KM Kilometerstand mid] [REA 0BR: not copied]
  frame.data[7] = 0x02; // [KM b7.0-3 Kilometerstand high, b7.4-7 Frei_Kombi_3_2] [REA 0BR: not copied]
  standaloneTx(frame);
}

void Gen4_frames100()
{
  twai_message_t frame = {};
  // PQ Gate_Komf_1 (0x390, DLC 8) - gateway-comfort broadcast (vw_pq.dbc: Gate_Komf_1).
  // Gen4 reads only b3.4 GK1_Rueckfahr (reverse) - spec_gen4_0br §3.
  frame.identifier = mGate_Komf_1;
  frame.data_length_code = 8;
  frame.data[0] = 0x03; // [KM Gate_Komf_1 b0.0 GK1_Sta_RDK_Warn, b0.1 GK1_Sta_Anhaen, b0.2-5 light / door status, b0.7 SleepAckn] [REA 0BR: not read]
  frame.data[1] = 0x11; // [KM b1.0-3 GK1_CharismaModus (mux 1), b1.4-7 GK1_SamFktNr (mux)] [REA 0BR: not read]
  frame.data[2] = 0x58; // [KM b2.0 Fa_Tuerkont, b2.1 GK1_RueckfahrSch, b2.2 ELV, b2.3 Kessy_2, b2.4 Stdhzg, b2.5 SH_Verbau, b2.6 ParkFrontWi, b2.7 KW_Warm] [REA 0BR: not read]
  frame.data[3] = 0x00; // [KM b3.4 GK1_Rueckfahr; b3.0/3.2-3 remote start / boot, b3.5-7 GK1_BrLi] not reversing [REA 0BR: b3.4 read (reverse)]
  frame.data[4] = 0x40; // [KM b4.0-7 BLS_ILM / EDC / Blinker_li/re / def_P_verr / Fernlicht / b4.6 GK1_Sta_Licht2 / Sta_LSM] [REA 0BR: not copied (RX len 4)]
  frame.data[5] = 0x00; // [KM b5 GK1_Count_Anhaen / BSK / BLS_AAG / Anhaenger bits] [REA 0BR: not copied]
  frame.data[6] = 0x01; // [KM b6.0 GK1_Abblendlicht .. b6.7 Warnblk_Status] [REA 0BR: not copied]
  frame.data[7] = 0x08; // [KM b7.0 SH_laeuft .. b7.3 GK1_Bremslicht (= 1 here) .. b7.7] [REA 0BR: not copied]
  standaloneTx(frame);

  // PQ Bremse_11 (0x5B7, DLC 8) - extended brake frame (vw_pq.dbc: Bremse_11).
  // Not in the Gen4 0BR receive table at all - the unit ignores it (spec_gen4_0br
  // §3), so its block is OFF in the V11 standalone default (same for Kombi_2).
  frame.identifier = BRAKES11_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // [KM Bremse_11.CHECKSUM] [REA 0BR: not received]
  frame.data[1] = 0XC0; // [KM b1.0-3 COUNTER, b1.5 B11_HydHalten, b1.6-7 B11_Br_StSt_Info] StSt_Info 3 [REA 0BR: not received]
  frame.data[2] = 0x00; // [KM B11_OBD_Nib_VL / _VR] [REA 0BR: not received]
  frame.data[3] = 0x00; // [KM B11_OBD_Nib_HL / _HR] [REA 0BR: not received]
  frame.data[4] = 0x00; // [KM b4.0 B11_EPB_Steller_akt, b4.1 _gue] [REA 0BR: not received]
  frame.data[5] = 0x00; // [KM -] [REA 0BR: not received]
  frame.data[6] = 0x00; // [KM -] [REA 0BR: not received]
  frame.data[7] = 0x00; // [KM -] [REA 0BR: not received]
  standaloneTx(frame);
}

void Gen4_frames200()
{
  twai_message_t frame = {};
  // PQ Kombi_2 (0x420, DLC 8) - cluster temps (vw_pq.dbc: Kombi_2). Not in the
  // 0BR receive table (spec_gen4_0br §3) - block OFF in the standalone default.
  // V11.00.13: byte names corrected from vw_pq.dbc (the old ones were shifted).
  frame.identifier = mKombi_2;
  frame.data_length_code = 8;
  frame.data[0] = 0x4C; // [KM Kombi_2 b0.0-2 Fehlerstatus Aussen/Oel/Kuehlm, b0.3 Anhaenger_erkannt, b0.7 Fehlerspeichereintrag] [REA 0BR: not received]
  frame.data[1] = 0x86; // [KM Aussentemperatur_gefiltert 0.5 -50 degC] [REA 0BR: not received]
  frame.data[2] = 0x85; // [KM Aussentemp__ungefiltert 0.5 -50 degC] [REA 0BR: not received]
  frame.data[3] = 0x00; // [KM Oeltemperatur_4_1 1 -60 degC] [REA 0BR: not received]
  frame.data[4] = 0x00; // [KM Kuehlmitteltemp__4_1 0.75 -48 degC] [REA 0BR: not received]
  frame.data[5] = 0x30; // [KM b5.0-6 Klemme_58d %, b5.7 Fehlerstatus] [REA 0BR: not received]
  frame.data[6] = 0xFF; // [KM b6.0-6 Klemme_58s %, b6.7 Fehlerstatus] [REA 0BR: not received]
  frame.data[7] = 0x04; // [KM Frei_Kombi_2_2] [REA 0BR: not received]
  standaloneTx(frame);
}

void Gen4_frames1000()
{
  twai_message_t frame = {};
  // PQ Diagnose_1 (0x7D0, DLC 8) - diagnostic timestamp broadcast (vw_pq.dbc: Diagnose_1).
  // REA 0BR: relearn count / km / date fields read (spec_gen4_0br §3), 500-tick window.
  frame.identifier = mDiagnose_1;
  frame.data_length_code = 8;
  frame.data[0] = 0x26;                // [KM Diagnose_1.DI1_VerlernZaehl] [REA 0BR: relearn count]
  frame.data[1] = 0xF2;                // [KM DI1_km_Stand low, 20 bit km] [REA 0BR: km]
  frame.data[2] = 0x03;                // [KM DI1_km_Stand mid] [REA 0BR: km]
  frame.data[3] = 0x12;                // [KM b3.0-3 DI1_km_Stand high, b3.4-7 DI1_Jahr low (+2000)] [REA 0BR: km / date]
  frame.data[4] = 0x70;                // [KM b4.0-2 DI1_Jahr high, b4.3-6 DI1_Monat, b4.7 DI1_Tag low] [REA 0BR: date]
  frame.data[5] = 0x19;                // [KM b5.0-3 DI1_Tag high, b5.4-7 DI1_Stunde low] [REA 0BR: date / time]
  frame.data[6] = 0x25;                // [KM b6.0 DI1_Stunde high, b6.1-6 DI1_Minute, b6.7 DI1_Sekunde low] [REA 0BR: time]
  frame.data[7] = mDiagnose_1_counter; // [KM b7.0-4 DI1_Sekunde high, b7.6 KM_Stand_alt, b7.7 Zeit_alt] rolling 0..31 [REA 0BR: time]
  standaloneTx(frame);
  mDiagnose_1_counter++;
  if (mDiagnose_1_counter > 0x1F)
    mDiagnose_1_counter = 0;
}

void gen41StepWheel(uint16_t &pulse, uint16_t &ts, uint8_t &roll,
                      uint16_t pulseInc, uint16_t tsInc)
{
  pulse = (uint16_t)((pulse + pulseInc) & 0x3FF);
  uint32_t newTs = (uint32_t)ts + (uint32_t)tsInc;
  if (newTs > 0xFFFF)
  {
    roll = (uint8_t)((roll + 1) & 0x3);
  }
  ts = (uint16_t)(newTs & 0xFFFF);
}

void Gen41_frames10()
{
  twai_message_t frame = {};
  frame.extd = 0;
  frame.rtr = 0;

  const uint16_t spdMod = 15;
  gen41StepWheel(gen41DrvLPulse, gen41DrvLTs, gen41DrvLRoll, 2, (uint16_t)(13000 + spdMod * 100));
  gen41StepWheel(gen41DrvRPulse, gen41DrvRTs, gen41DrvRRoll, 2, (uint16_t)(13000 + spdMod * 100));
  gen41StepWheel(gen41NdrvLPulse, gen41NdrvLTs, gen41NdrvLRoll, 2, (uint16_t)(12000 + spdMod * 100));
  gen41StepWheel(gen41NdrvRPulse, gen41NdrvRTs, gen41NdrvRRoll, 2, (uint16_t)(12000 + spdMod * 100));

  const uint8_t resetOcc = 0;
  const uint8_t seq = gen41Seq;

  // 0x0C1 - PPEI Driven Wheel Rotational Status (10 ms), EBCM TX
  frame.identifier = 0x0C1;
  frame.data_length_code = 8;
  frame.data[0] = gen41WheelStatusByte(gen41DrvLRoll, seq, resetOcc, gen41DrvLPulse);
  frame.data[1] = (uint8_t)(gen41DrvLPulse & 0xFF);
  frame.data[2] = (uint8_t)((gen41DrvLTs >> 8) & 0xFF);
  frame.data[3] = (uint8_t)(gen41DrvLTs & 0xFF);
  frame.data[4] = gen41WheelStatusByte(gen41DrvRRoll, seq, resetOcc, gen41DrvRPulse);
  frame.data[5] = (uint8_t)(gen41DrvRPulse & 0xFF);
  frame.data[6] = (uint8_t)((gen41DrvRTs >> 8) & 0xFF);
  frame.data[7] = (uint8_t)(gen41DrvRTs & 0xFF);
  taskENTER_CRITICAL(&gen41Bus0CacheMux);
  gen41Bus0CacheC1 = frame;
  gen41Bus0CacheValidC1 = true;
  taskEXIT_CRITICAL(&gen41Bus0CacheMux);
  standaloneTx(frame);

  // 0x0C5 - PPEI Non Driven Wheel Rotational Status (10 ms), EBCM TX
  frame.identifier = 0x0C5;
  frame.data_length_code = 8;
  frame.data[0] = gen41WheelStatusByte(gen41NdrvLRoll, seq, resetOcc, gen41NdrvLPulse);
  frame.data[1] = (uint8_t)(gen41NdrvLPulse & 0xFF);
  frame.data[2] = (uint8_t)((gen41NdrvLTs >> 8) & 0xFF);
  frame.data[3] = (uint8_t)(gen41NdrvLTs & 0xFF);
  frame.data[4] = gen41WheelStatusByte(gen41NdrvRRoll, seq, resetOcc, gen41NdrvRPulse);
  frame.data[5] = (uint8_t)(gen41NdrvRPulse & 0xFF);
  frame.data[6] = (uint8_t)((gen41NdrvRTs >> 8) & 0xFF);
  frame.data[7] = (uint8_t)(gen41NdrvRTs & 0xFF);
  taskENTER_CRITICAL(&gen41Bus0CacheMux);
  gen41Bus0CacheC5 = frame;
  gen41Bus0CacheValidC5 = true;
  taskEXIT_CRITICAL(&gen41Bus0CacheMux);
  standaloneTx(frame);

  // 0x0C9 - PPEI Engine General Status 1 (spec 12.5 ms), ECM TX
  frame.identifier = 0x0C9;
  frame.data_length_code = 8;
  frame.data[0] = 0x80;
  frame.data[1] = getLockTargetAdjustedValue(0x1F, false);
  frame.data[2] = getLockTargetAdjustedValue(0x40, false);
  {
    const uint8_t tArc = gen41ThrottleArc & 0x3U;
    const uint8_t tSig = 1U;
    const uint8_t tPv = static_cast<uint8_t>((~(tSig + tArc) + 1U) & 0x3U);
    frame.data[3] = static_cast<uint8_t>(0x10U | (tArc << 2U) | tPv);
  }
  frame.data[4] = 0xFE;
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x64;
  twai_transmit_v2(twaiBus0, &frame, 0);
  gen41ThrottleArc = (uint8_t)((gen41ThrottleArc + 1) & 0x3);

  // 0x0F1 - PPEI Brake Apply Status (10 ms), BSM TX
  frame.identifier = 0x0F1;
  frame.data_length_code = 6;
  {
    const uint8_t bArc = gen41BrakeArc & 0x3U;
    const uint8_t bSig = 0U;
    const uint8_t bPv = static_cast<uint8_t>((~(bSig + bArc) + 1U) & 0x3U);
    frame.data[0] = static_cast<uint8_t>((bArc << 4U) | (bPv << 2U));
  }
  frame.data[1] = 0x00;
  frame.data[2] = 0x00;
  frame.data[3] = 0x40;
  frame.data[4] = 0x00;
  frame.data[5] = 0x00;
  twai_transmit_v2(twaiBus0, &frame, 0);
  gen41BrakeArc = (uint8_t)((gen41BrakeArc + 1) & 0x3);

  gen41Seq = (uint8_t)((gen41Seq + 1) & 0x3);
  if (gen41Frame10Cnt < 0xFFFF)
    gen41Frame10Cnt++;

  // 0x130 (Bus1) - Multi-axis inertial sensor (~10-13 ms cadence)
  {
    frame.identifier = 0x130;
    frame.extd = 0;
    frame.rtr = 0;
    frame.data_length_code = 8;
    const uint8_t idx = gen41TakeReplayIndex(gen41Frame130State);
    const uint8_t *r130 = gen41Frame130Bus1Tbl[idx];
    for (uint8_t i = 0U; i < 8U; i++)
    {
      frame.data[i] = r130[i];
    }
    standaloneTx(frame);
  }

  // 0x140 (Bus1) - Chassis Inertial Sensor (Yaw Rate), ~10 ms cadence
  {
    static uint8_t gen41Frame140Counter = 0U;
    const uint8_t d0 = (uint8_t)(0x80U | ((gen41Frame140Counter & 0x03U) << 2));
    const uint8_t d5 = 0xDAU;
    const uint8_t d7 = (uint8_t)((d0 + d5 + 0x37U) & 0xFFU);
    frame.identifier = 0x140;
    frame.extd = 0;
    frame.rtr = 0;
    frame.data_length_code = 8;
    frame.data[0] = d0;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x0F;
    frame.data[5] = d5;
    frame.data[6] = 0x01;
    frame.data[7] = d7;
    standaloneTx(frame);
    gen41Frame140Counter = (gen41Frame140Counter + 1U) & 0x03U;
  }
}

void Gen41_frames20()
{
  uint8_t step = static_cast<uint8_t>(Gen41_1CE234_counter & 0x03);
  uint8_t d4Common = (step == 0) ? 0x00 : static_cast<uint8_t>(0x100 - step);

  twai_message_t frame = {};
  frame.extd = 0;
  frame.rtr = 0;

  // 0x1CE (Bus1) - PPEI Secondary Axle Torque Request (GMW8762, EBCM TX, 20 ms)
  {
    const uint8_t arc = static_cast<uint8_t>(step & 0x03U);
    const float reqNm = gen41TorqueRequestNm;
    const bool requesting = (reqNm > 0.0f);
    const float scaled = reqNm / (float)GEN41_REQUEST_STEP + 0.5f;
    const uint8_t d1 = requesting
                           ? static_cast<uint8_t>(scaled >= 255.0f ? 255U : static_cast<uint8_t>(scaled))
                           : 0x00U;
    const uint8_t d0 = static_cast<uint8_t>((arc << 3) | (requesting ? 0x04U : 0x00U));
    const uint8_t d2 = requesting ? 0x03U : (arc == 0U ? 0x00U : 0x07U);
    const uint8_t d3 = static_cast<uint8_t>((0U - arc - d1) & 0xFFU);
    frame.identifier = 0x1CE;
    frame.data_length_code = 8;
    frame.data[0] = d0;
    frame.data[1] = d1;
    frame.data[2] = d2;
    frame.data[3] = d3;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    standaloneTx(frame);
  }

  // 0x1E9 - PPEI Chassis General Status 1 (20 ms), EBCM TX
  frame.identifier = 0x1E9;
  frame.data_length_code = 8;
  frame.data[0] = 0x0F;
  frame.data[1] = 0xFC;
  frame.data[2] = 0x00;
  frame.data[3] = 0x0E;
  frame.data[4] = 0x80;
  frame.data[5] = 0x06;
  frame.data[6] = static_cast<uint8_t>(gen41Frame1e9Counter << 5);
  frame.data[7] = 0x00;
  twai_transmit_v2(twaiBus0, &frame, 0);
  gen41Frame1e9Counter = static_cast<uint8_t>((gen41Frame1e9Counter + 1U) & 0x03U);

  // 0x1F5 - PPEI Transmission General Status 2 (spec 25 ms), TCM TX
  frame.identifier = 0x1F5;
  frame.data_length_code = 8;
  frame.data[0] = 0xE2;
  frame.data[1] = 0x03;
  frame.data[2] = 0x00;
  frame.data[3] = 0x0F;
  frame.data[4] = 0x00;
  frame.data[5] = 0x00;
  frame.data[6] = 0x01;
  frame.data[7] = 0x0F;
  twai_transmit_v2(twaiBus0, &frame, 0);

  // 0x234 (Bus1) - Insignia-specific rolling counter frame
  frame.identifier = 0x234;
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 4;
  {
    uint8_t byte0Value = static_cast<uint8_t>(0x04 + (step << 4));
    frame.data[0] = byte0Value;
    frame.data[1] = 0x00;
    frame.data[2] = static_cast<uint8_t>(step == 0 ? 0x0C : 0x0B);
    frame.data[3] = d4Common;
  }
  {
    Gen41_1CE234_counter = (Gen41_1CE234_counter + 1) & 0x03;
  }
  standaloneTx(frame);
}

void Gen41_frames25()
{
  twai_message_t frame = {};
  frame.extd = 0;
  frame.rtr = 0;

  // 0x2F9 - PPEI Chassis General Status 2 (spec 50 ms), EBCM TX
  {
    static uint8_t gen41Frame2f9Counter = 0;
    frame.identifier = 0x2F9;
    frame.data_length_code = 7;
    frame.data[0] = static_cast<uint8_t>((gen41Frame2f9Counter << 3) | (gen41Frame2f9Counter < 16U ? 0x03U : 0x00U));
    frame.data[1] = 0x01;
    frame.data[2] = 0x10;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    twai_transmit_v2(twaiBus0, &frame, 0);
    gen41Frame2f9Counter = static_cast<uint8_t>((gen41Frame2f9Counter + 1U) & 0x1FU);
  }

  // 0x1C3 (Bus0) — GMW8762 "PPEI Engine Torque Status 2" (25ms), ECM TX
  frame.identifier = 0x1C3;
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = getLockTargetAdjustedValue(0x2E, true) + 0x1A;
  frame.data[1] = 0x51;
  frame.data[2] = 0x06;
  frame.data[3] = 0x51;
  frame.data[4] = 0x00;
  frame.data[5] = 0x00;
  frame.data[6] = 0xFE;
  frame.data[7] = tempCounter;
  twai_transmit_v2(twaiBus0, &frame, 0);

  // 0x191 (Bus0) - EngineData, OEM 23ms cadence
  frame.identifier = 0x191;
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x06;
  frame.data[1] = 0xAA;
  frame.data[2] = 0x08;
  frame.data[3] = 0xCA;
  frame.data[4] = 0x06;
  frame.data[5] = 0xAA;
  frame.data[6] = 0x00;
  frame.data[7] = 0x17;
  twai_transmit_v2(twaiBus0, &frame, 0);

  // 0x1C5 (Bus0) - PPEI Driver Intended Axle Torque Status (spec 25ms), ECM TX
  {
    frame.identifier = 0x1C5;
    frame.extd = 0;
    frame.rtr = 0;
    frame.data_length_code = 8;
    frame.data[0] = 0x2E;
    frame.data[1] = 0x91;
    frame.data[2] = 0x2B;
    frame.data[3] = 0x66;
    frame.data[4] = 0x2F;
    frame.data[5] = 0xAF;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    twai_transmit_v2(twaiBus0, &frame, 0);
  }

  // 0x330 (Bus1) - Insignia inertial/accel sensor, OEM ~25-30ms cadence
  {
    frame.identifier = 0x330;
    frame.extd = 0;
    frame.rtr = 0;
    frame.data_length_code = 8;
    const uint8_t idx = gen41TakeReplayIndex(gen41Frame330State);
    const uint8_t *r330 = gen41Frame330Bus1Tbl[idx];
    frame.data[0] = r330[0];
    frame.data[1] = r330[1];
    frame.data[2] = r330[2];
    frame.data[3] = r330[3];
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    standaloneTx(frame);
  }
}

void Gen41_frames100()
{
  twai_message_t frame = {};
  frame.extd = 0;
  frame.rtr = 0;

  // 0x230 (Bus0) — OEM 96 ms cadence. 8-state synchronized replay table.
  {
    const uint8_t idx = gen41TakeReplayIndex(gen41Frame230State);
    frame.identifier = 0x230;
    frame.data_length_code = 8;
    frame.data[0] = 0x00;
    frame.data[1] = gen41Frame230B1[idx];
    frame.data[2] = gen41Frame230B2[idx];
    frame.data[3] = gen41Frame230B3[idx];
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = gen41Frame230B6[idx];
    frame.data[7] = gen41Frame230B7[idx];
    twai_transmit_v2(twaiBus0, &frame, 0);
  }

  // 0x348 (Bus0) — OEM 101 ms cadence. DLC=5.
  frame.identifier = 0x348;
  frame.data_length_code = 5;
  frame.data[0] = 0x00;
  frame.data[1] = 0x00;
  frame.data[2] = 0x00;
  frame.data[3] = 0x00;
  frame.data[4] = 0x1B;
  twai_transmit_v2(twaiBus0, &frame, 0);

  // 0x34A (Bus0) — OEM 103 ms cadence. DLC=5.
  frame.identifier = 0x34A;
  frame.data_length_code = 5;
  frame.data[0] = 0x00;
  frame.data[1] = 0x00;
  frame.data[2] = 0x00;
  frame.data[3] = 0x00;
  frame.data[4] = 0x1B;
  twai_transmit_v2(twaiBus0, &frame, 0);

  // 0x1F1 - PPEI Platform General Status (100 ms), BCM/GW TX
  frame.identifier = 0x1F1;
  frame.data_length_code = 8;
  frame.data[0] = 0xAE;
  frame.data[1] = 0x0A;
  frame.data[2] = 0x00;
  frame.data[3] = 0x00;
  frame.data[4] = 0x08;
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x7A;
  twai_transmit_v2(twaiBus0, &frame, 0);

  // 0x17D (Bus0) — GMW8762 "Antilock_Brake_and_TC_Status_HS" (100ms), EBCM TX
  frame.identifier = 0x17D;
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 6;
  frame.data[0] = 0x22;
  frame.data[1] = 0x24;
  frame.data[2] = 0x42;
  frame.data[3] = 0xFF;
  frame.data[4] = 0x00;
  frame.data[5] = 0x00;
  twai_transmit_v2(twaiBus0, &frame, 0);
}

void Gen41_frames200()
{
  twai_message_t frame = {};
  frame.extd = 0;
  frame.rtr = 0;

  // 0x12A (Bus0) - BCMDoorBeltStatus
  frame.identifier = 0x12A;
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;
  frame.data[1] = 0x20;
  frame.data[2] = 0x60;
  frame.data[3] = 0x75;
  frame.data[4] = 0x00;
  frame.data[5] = 0x30;
  frame.data[6] = 0x00;
  frame.data[7] = 0x80;
  twai_transmit_v2(twaiBus0, &frame, 0);

  // 0x135 (Bus0) - ECMPRDNL: Transmission PRNDL gear position & ESP button
  frame.identifier = 0x135;
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x02;
  frame.data[1] = 0x00;
  frame.data[2] = 0x1D;
  frame.data[3] = 0x00;
  frame.data[4] = 0x03;
  frame.data[5] = 0x1A;
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;
  twai_transmit_v2(twaiBus0, &frame, 0);

  // 0x4C1 - PPEI Engine General Status 4 (spec 500 ms), ECM TX
  frame.identifier = 0x4C1;
  frame.data_length_code = 8;
  frame.data[0] = 0x10;
  frame.data[1] = 0xC6;
  frame.data[2] = 0x5F;
  frame.data[3] = 0x3B;
  frame.data[4] = 0x60;
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;
  twai_transmit_v2(twaiBus0, &frame, 0);
}

void Gen41_frames1000()
{
  twai_message_t frame = {};
  frame.extd = 0;
  frame.rtr = 0;
  const uint32_t nowMs = millis();

  // 0x4F1 (Bus0) — GMW8762 "PPEI Powertrain Configuration Data" (1000ms), ECM TX
  frame.identifier = 0x4F1;
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  static const uint16_t TYRE_CIRC_4F1_F = 420U;
  static const uint16_t TYRE_CIRC_4F1_R = 420U;
  frame.data[0] = static_cast<uint8_t>((TYRE_CIRC_4F1_F >> 8) & 0xFFU);
  frame.data[1] = static_cast<uint8_t>(TYRE_CIRC_4F1_F & 0xFFU);
  frame.data[2] = static_cast<uint8_t>((TYRE_CIRC_4F1_R >> 8) & 0xFFU);
  frame.data[3] = static_cast<uint8_t>(TYRE_CIRC_4F1_R & 0xFFU);
  frame.data[4] = 0x00;
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;
  twai_transmit_v2(twaiBus0, &frame, 0);

  // 0x500 (Bus0) - GMLAN NM frame, ~2 s heartbeat
  {
    static uint32_t gen41Frame500LastMs = 0U;
    if (gen41PeriodicDue(gen41Frame500LastMs, nowMs, 2000U, 2000U))
    {
      frame.identifier = 0x500;
      frame.extd = 0;
      frame.rtr = 0;
      frame.data_length_code = 4;
      frame.data[0] = 0x30;
      frame.data[1] = 0x30;
      frame.data[2] = 0x01;
      frame.data[3] = 0xF4;
      twai_transmit_v2(twaiBus0, &frame, 0);
    }
  }
}

void Gen41_frames13()
{
  twai_message_t frame = {};
  frame.extd = 0;
  frame.rtr = 0;

  // 0x0F9 (Bus0) — GMW8762 "PPEI Transmission General Status 1" (12.5ms), TCM/ECM TX
  frame.identifier = 0x0F9;
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x07;
  frame.data[1] = 0xF3;
  frame.data[2] = 0x00;
  frame.data[3] = 0x00;
  frame.data[4] = 0x00;
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;
  twai_transmit_v2(twaiBus0, &frame, 0);

  // 0x180 (Bus1) - GM SAS, 10 ms cadence
  gen41Build180(frame);
  standaloneTx(frame);

  gen41SwaAdvanceWobble();
}

void Gen41_frames50()
{
  twai_message_t frame = {};
  frame.extd = 0;
  frame.rtr = 0;

  // 0x2C3 (Bus0) — GMW8762 "PPEI Engine Torque Status 3" (50ms), ECM TX
  frame.identifier = 0x2C3;
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  const uint8_t idx2c3 = gen41TakeReplayIndex(gen41Frame2c3State);
  const uint16_t speed2c3 = gen41Frame2c3WheelSpeedTbl[idx2c3];
  gen41LastWheelSpeed2c3 = speed2c3;
  frame.data[0] = static_cast<uint8_t>((speed2c3 >> 8) & 0xFFU);
  frame.data[1] = static_cast<uint8_t>(speed2c3 & 0xFFU);
  frame.data[2] = static_cast<uint8_t>((speed2c3 >> 8) & 0xFFU);
  frame.data[3] = static_cast<uint8_t>(speed2c3 & 0xFFU);
  frame.data[4] = 0x00;
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;
  twai_transmit_v2(twaiBus0, &frame, 0);
}

void Gen41_frames250()
{
  twai_message_t frame = {};
  frame.extd = 0;
  frame.rtr = 0;

  // 0x3F1 - PPEI Platform Engine Control Request (spec 250 ms), GW TX
  frame.identifier = 0x3F1;
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;
  frame.data[1] = 0xC5;
  frame.data[2] = 0x7E;
  frame.data[3] = 0x08;
  frame.data[4] = 0x11;
  frame.data[5] = 0xFA;
  frame.data[6] = 0x0A;
  frame.data[7] = 0x60;
  twai_transmit_v2(twaiBus0, &frame, 0);
}

// Measured frame roles (bench, Gen5 0CQ, standalone, V3 packing, CF 30, floor 0,
// one frame disabled at a time; control = 30.0% reported / 30.7% PWM / 3.88 A):
//   ESP_19  0x0B2  MANDATORY - off: 0% engagement, pump dead (0% PWM, 0.01 A)
//   Motor_11 0x0A7 MANDATORY - off: 0% engagement, pump dead (0% PWM, 0.01 A)
//   ESP_05  0x106  MANDATORY - off: 0% engagement, but pump still runs (17% PWM,
//                  1.6 A) - gates committing to/reporting lock, not actuation
//   ESP_14  0x08A  THE LIMITER - off: 98.6% engagement (peak 100) on a 30%
//                  request. BR_Vorg_*_Max is what caps delivered lock
//   Motor_12 0x0A8 no measurable effect at this operating point
//   ESP_02  0x101  no measurable effect at this operating point
//   ESP_21  0x0FD  no measurable effect at this operating point
// Command, pump duty and reported value track 1:1 in V3. Note "no effect" was
// only established at steady 30% on a bench - not during transitions or at speed.
// V11.00.0 (OEM code, documents/haldex-binaries/leaf_trace/spec_gen5_0cq.md):
// the roles above were measured with ESP_14 Min = 0, i.e. ESP_14 as a ceiling over
// the unit's own Motor_11 strategy - which is why Motor_11 looked mandatory. ESP_14
// now sends Min = Max = V (0..250), the 0CQ's "exact" mode: the clutch target is V
// itself and the Motor_11 strategy is not used. The 0CQ receives 18 frames
// (spec §3) and checks no CRC or counter on any of them; ESP_10, EPB_01, ESP_18,
// ESP_23, Parkhilfe_04, Gateway_72, Getriebe_14 and ESP_29 are not received at
// all, so their blocks are OFF in the standalone defaults (globals.cpp).
void Gen5_0CQ_frames10()
{
  twai_message_t frame = {};
  // MQB ESP_18 (0x135, DLC 8) - ESP minor broadcast. Fixed response, no changes.
  // [REA audit_gen5_0cq R6 / spec §1: 0x135 is NOT in the 0CQ RX table - the whole frame is ignored]
  frame.identifier = ESP_18; // ESP_18 0x135
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // [KM ESP_18.ESP_18_CRC] [REA: not received]
  frame.data[1] = 0xC0; // [KM ESP_18_BZ b1[0-3]=0, ESP_St_V_fzg_ref b1[4-5]=0, ESP_v_fzg_ref_FR b1[6-7]=3] [REA: not received]
  frame.data[2] = 0x00; // [KM ESP_Allrad_Moment_Anforderung 8 Nm/bit] [REA: not received]
  frame.data[3] = 0x00; // [KM ESP_Allrad_Anforderung b3[0-4]] [REA: not received]
  frame.data[4] = 0x00; // [KM -] [REA: not received]
  frame.data[5] = 0x00; // [KM -] [REA: not received]
  frame.data[6] = 0x00; // [KM ESP_V_fzg_ref low, 0.01 km/h] [REA: not received]
  frame.data[7] = 0x00; // [KM ESP_V_fzg_ref high] [REA: not received]
  standaloneTx(frame);

  // MQB ESP_19 (0x0B2, DLC 8) - wheel speeds (ESP_VL/VR/HL/HR_Radgeschw_02,
  // LE, 0.0075 km/h/bit - see the real decode in OpenHaldexC6_can.cpp).
  // Wheel speed MUST keep changing or haldex slowly disengages - static
  // makes it fade over time and eventually stop. The speeds DO follow the
  // lock by default (wsBaseRaw = 0: 0 km/h at 0 %, ~19-31 km/h at 100 %,
  // through the old VAG correction curve) - Adam's V11.00.0 choice, see
  // fillEsp19WheelSpeeds() in Calculations.cpp. Only the serial-lab
  // explicit mode (wsBaseRaw > 0) sends lock-independent speeds. The 0CQ's
  // strategy and clutch-load model read all four (spec D6 is NOT applied -
  // audit_gen5_0cq F5). Byte tags: see fillEsp19WheelSpeeds().
  frame.identifier = ESP_19; // ESP_19 0x0b2
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  fillEsp19WheelSpeeds(frame.data);
  standaloneTx(frame);

  // MQB Getriebe_11 (0x0AD, DLC 8) - transmission ECU broadcast.
  // GE_MMom_Anf_02 (engine torque request), GE_MMom_Vorhalt_02 (anticipatory torque),
  // GE_Fahrstufe (selected gear), GE_Schalt_Sequenz (shift state), GE_Zielgang.
  // The 0CQ reads only b7 bits 4-7, the target gear (OEM code: spec_gen5_0cq §3):
  // 1-7 are used as gears; 0 becomes the unit's internal "no gear" code (0xFE).
  // (Motor_14's gear is only used when this frame is missing.)
  // V11 TODO (bench): try a real gear here (e.g. b7 = 0x20 = 2nd) and compare -
  // the spec cannot say what gear 0 does to the strategy (spec §5).
  frame.identifier = GETRIEBE_11; // 0x0AD
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;                // [KM Getriebe_11.Getriebe_11_CRC] [REA audit S6/A16: no CRC check]
  frame.data[1] = GETRIEBE_11_counter; // 0x00..0x0F [KM Getriebe_11_BZ b1[0-3], GE_MMom_Soll_02 low b1[4-7]=0] [REA spec §3: not read]
  frame.data[2] = 0x00;                // (was 0xFE) [KM GE_MMom_Soll_02 high b2[0-5] (raw-509 Nm), GE_MMom_Vorhalt_02 low b2[6-7]] [REA spec §3: not read]
  frame.data[3] = 0xFE;                // [KM GE_MMom_Vorhalt_02 high (raw-509 Nm)] [REA spec §3: not read]
  frame.data[4] = 0x00;                // [KM GE_Uefkt low, 0.1/bit] [REA spec §3: not read]
  frame.data[5] = 0x00;                // [KM GE_Uefkt high b5[0-1], GE_Fahrstufe b5[2-5], GE_Ursache_Fahrstufe_N b5.6, GE_Schaltablauf b5.7] [REA spec §3: not read]
  frame.data[6] = 0x00;                // [KM GE_Uefkt_unplausibel b6.1, GE_MMom_Status_02 b6[2-4], GE_Status_Kraftschluss b6[5-7]] [REA spec §3: not read]
  frame.data[7] = 0x00;                // gear 0, see the TODO above [KM GE_MMom_Status b7[0-1], GE_Zielgang b7[4-7]] [REA spec §3: b7[4-7] only - 1-7 gear, 8->0xFF, 0xB/0xC->8/9, else (incl. 0) 0xFE]

  frame.data[0] = calcChecksum(frame.data, ID_SEQ_0AD); // for 0x0AD

  GETRIEBE_11_counter++;
  if (GETRIEBE_11_counter > 0x0F)
  {
    GETRIEBE_11_counter = 0;
  }
  standaloneTx(frame);

  // MQB Motor_12 (0x0A8, DLC 8) - engine torque limits / RPM (Motor_12).
  // MO_Mom_neg_verfuegbar (max engine braking), Mom_Statisch_Limit (static limit),
  // Mom_Dynamisch_Limit (dynamic limit), MO_Drehzahl_01 (engine RPM).
  // The 0CQ reads only b6-b7 = engine speed (OEM code: spec_gen5_0cq §3).
  frame.identifier = MOTOR_12; // Motor_12 0x0A8
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;             // [KM Motor_12.Motor_12_CRC] [REA audit S6/A16: no CRC check]
  frame.data[1] = MOTOR_12_counter; // 0x70..0x7F [KM Motor_12_BZ b1[0-3], MO_Mom_neg_verfuegbar low b1[4-7]=7] [REA spec §3: not read]
  frame.data[2] = 0x00;             // [KM MO_Mom_neg_verfuegbar high b2[0-4] (raw-509 Nm), MO_Mom_Begr_Fahr low b2[5-7]] [REA spec §3: not read]
  frame.data[3] = 0x00;             // [KM MO_Mom_Begr_Fahr high b3[0-5] (Nm), MO_Mom_Begr_Schalt low b3[6-7]] [REA spec §3: not read]
  frame.data[4] = 0x00;             // [KM MO_Mom_Begr_Schalt high (raw-509 Nm)] [REA spec §3: not read]
  frame.data[5] = 0x64;             // [KM MO_Momentenintegral_02 b5[0-6]=100 %, MO_QBit_Drehzahl_01 b5.7=0 valid] [REA spec §3: not read]
  // V11.00.0: b6-b7 are ONE 16-bit engine speed (MO_Drehzahl_01, 0.25 rpm per
  // step), not a quality bit + rpm. b7 used to follow the lock, so the rpm ran
  // from ~4 rpm at 0 % to ~16000 rpm at 100 % (spec D5). Now a steady 2000 rpm.
  frame.data[6] = 0x40; // 0x1F40 = 2000 rpm [KM MO_Drehzahl_01 low, 0.25 rpm] [REA audit S15/F7: b6-b7 one u16, raw x0x6487/0x3C00 -> 0xCEE2; strategy/monitor input; "0.25 rpm" is KM only]
  frame.data[7] = 0x1F; // [KM MO_Drehzahl_01 high] [REA audit S15: see b6]

  frame.data[0] = calcChecksum(frame.data, ID_SEQ_0A8); // for 0x0A8

  MOTOR_12_counter++;
  if (MOTOR_12_counter > 0x7F)
  {
    MOTOR_12_counter = 0x70;
  }
  standaloneTx(frame);

  // MQB Motor_11 (0x0A7, DLC 8) - engine torque demand/output broadcast.
  // V11.00.0 (OEM code, spec_gen5_0cq §2.1): the 0CQ reads only MO_Mom_Soll_Roh
  // (bits 12-21) and MO_Mom_Ist_Summe (bits 22-31), raw = Nm + 509, plus b7 bit 5.
  // That torque feeds its OWN strategy, which is ignored while ESP_14 sends
  // Min = Max (the exact request below), so Motor_11 can't fight the ESP_14 lock request.
  // It still feeds the unit's clutch-load model, so it keeps a steady value.
  //
  // Two packings, selected by the user-facing "Fix Hunting" toggle (fixHunting):
  //   fixHunting == false (default): empirical V3 packing, b6/b7 now fixed at
  //                                  0xFA (they used to follow the lock).
  //                                  Confirmed working on 554C, 554D,
  //                                  554H, and 554K @ 100% lock.
  //   fixHunting == true            : DBC-correct bit-packed Soll_Roh/Ist/Solf
  //                                  with fixed BPK defaults. Required on 554K
  //                                  at partial lock (60/40, 70/30) where the
  //                                  V3 packing causes the controller to hunt.
  //
  // The previous auto-detect logic was removed because the engagement-crossing
  // heuristic falsely flagged 554D as hunting. Mode is now strictly user-set
  // and persisted to EEPROM.
  frame.identifier = MOTOR_11; // MOTOR_11 0x0A7
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;

  // Danger Zone rides on the BPK packing (raised torque ceiling), so it forces
  // that path for the cycles it is live even with Fix Hunting off.
  if (!fixHunting && !dangerZoneActive())
  {
    // ---- V3 packing (default, working on 554C/D/H and 554K@100%) ----
    frame.data[0] = 0x00;             // [KM Motor_11.Motor_11_CRC] [REA audit A14/S6: b0 never read, no CRC check]
    frame.data[1] = MOTOR_11_counter; // 0x40..0x4F [KM Motor_11_BZ b1[0-3], MO_Mom_Soll_Roh low b1[4-7]] [REA audit A14: BZ not read; Soll_Roh sig152 bits 12-21]
    frame.data[2] = 0xFA;             // Soll_Roh raw 0x3A4 = 423 Nm (spec D4) [KM MO_Mom_Soll_Roh high b2[0-5], MO_Mom_Ist_Summe low b2[6-7] (raw-509 Nm)] [REA audit A14/A20: Soll -> 0xCEE6, strategy only]
    frame.data[3] = 0xFA;             // Ist raw 0x3EB = 494 Nm [KM MO_Mom_Ist_Summe high] [REA audit A14/A20: Ist sig153 bits 22-31, x4 x MO_Faktor -> 0xCEE4; raw >= 0x3FB invalid]
    frame.data[4] = 0x00;             // [KM MO_Mom_Traegheit_Summe low] [REA audit A21: not read]
    frame.data[5] = 0xFA;             // [KM MO_Mom_Traegheit_Summe high b5[0-1], MO_Mom_Soll_gefiltert low b5[2-7]] [REA audit A21: not read]
    // b6/b7 only hold MO_Mom_Schub (never used by the 0CQ) and b7 bit 5
    // MO_Status_Normalbetrieb (fault-monitor enable only). They used to follow
    // the lock, flipping bit 61 with the lock value; the "massive effect" seen on
    // the bench came from ESP_14 changing in the same cycle (spec D3). V11.00.0:
    // fixed at the values that were sent at 100 % lock.
    frame.data[6] = 0xFA; // [KM MO_Mom_Soll_gefiltert high b6[0-3], MO_Mom_Schub low b6[4-7]] [REA audit A27: Schub getter exists, never used]
    frame.data[7] = 0xFA; // [KM MO_Mom_Schub high b7[0-4], MO_Status_Normalbetrieb_01 b7.5=1, MO_erste_Ungenauschwelle b7.6, MO_QBit_Motormomente b7.7] [REA audit A27: only b7.5 (bit 61), monitor-enable callbacks only]
  }
  else
  {
    // ---- BPK packing (Fix Hunting toggle on, or Danger Zone live) ----
    // Shared with the normal-mode packer so the two can never drift, and so the
    // serial lab tunables apply identically in both. See fillMotor11Bpk().
    // The torque here still follows the lock (the "torque-style" lock request), but the
    // 0CQ only uses it when ESP_14 is missing - ESP_14 below is what it follows.
    fillMotor11Bpk(frame.data, MOTOR_11_counter);
  }

  frame.data[0] = calcChecksum(frame.data, ID_SEQ_0A7); // for 0x0A7

  MOTOR_11_counter++;
  if (MOTOR_11_counter > 0x4F)
  {
    MOTOR_11_counter = 0x40;
  }

  standaloneTx(frame);
  /*
  0xd2,0x3d,0xcd,0x28,0x4c,0x14,0x22,0x4b,0x24,0xac,0xfa,0x55,0x66,0x80,0x0d,0x6c
  */

  // MQB ESP_14 (0x08A, DLC 8) - THE lock request on the 0CQ.
  // BR_Vorg_Quer_Min/Max (b4/b5, front lock - not read by the 0CQ),
  // BR_Vorg_Allrad_Min/Max (b6/b7 - the only bytes the 0CQ reads, spec_gen5_0cq §2).
  // V11.00.0 (spec D1/D2): b6 = b7 = V, 0..250. Min = Max is the 0CQ's "exact"
  // mode: clutch target = V (0.4 % per step, same scale as Allrad_03 byte 2),
  // and its own Motor_11 strategy is ignored. V = 0 = clutch open. Never above
  // 250: 251 / 253-255 mean "no ESP request" and the unit runs its own strategy
  // (that is what happened at full lock with the old 0..0xFE scale); 252 (0xFC)
  // in b7 OPENS the clutch - it is NOT "hold" (audit_gen5_0cq S13: b7 == 0xFC is
  // tested first; "hold" is only b6 = 0xFC with b7 != 0xFC, and only in one of
  // the two target tasks). The Launch PWM floor is not used here any more - Min
  // is already pinned to Max. "Exact" = V x 128, then the unit's falling-ramp
  // (only while flag 0xCFDC is set) and its run-time capacity cap 0xCDB2, plus
  // its fault reactions, still apply on top (audit A25 / S14).
  frame.identifier = ESP_14; // ESP_14 0x08A
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;           // [KM ESP_14.ESP_14_CRC] [REA audit S6/A13: b0 never read, no CRC check]
  frame.data[1] = ESP_14_counter; // 0x10..0x1F [KM ESP_14_BZ b1[0-3], ESP_14_Status_DSR b1[4-7]=1] [REA audit A13: not read]
  frame.data[2] = 0x00;           // [KM ESP_14_Anf_Lenkmomoffset_DSR low, 0.01 Nm] [REA audit A13: not read]
  frame.data[3] = 0x00;           // [KM Lenkmomoffset high b3.0, VZ b3.1, ESP_14_Sendestatus b3.2, BR_Status_Quer_ESP b3[3-5] (VAQ)] [REA audit A13: not read by the 0CQ]

  appliedTorque = gen5Esp14LockRequestByte(); // V, 0..250

  frame.data[4] = appliedTorque; // mirrored V [KM BR_Vorg_Quer_Min 0.4 %] [REA audit A13: not read by the 0CQ]
  frame.data[5] = appliedTorque; // mirrored V [KM BR_Vorg_Quer_Max 0.4 %] [REA audit A13: not read by the 0CQ]
  frame.data[6] = appliedTorque; // Min = V [KM BR_Vorg_Allrad_Min 0.4 %, 251 Ueberanpressung/252 schnell_zu/254 Init/255 Fehler] [REA audit A13/A17/A18: id114, Min x128; b6 = b7 <= 0xFA -> exact target]
  frame.data[7] = appliedTorque; // Max = V [KM BR_Vorg_Allrad_Max 0.4 %, 252 schnell_offen] [REA audit A13/A17/S13: id115, Max x128; 0xFC = OPEN, 0xFB/0xFD-0xFF with b6 = b7 = no request]

  frame.data[0] = calcChecksum(frame.data, ID_SEQ_08A); // for 0x08A

  ESP_14_counter++;
  if (ESP_14_counter > 0x1F)
  {
    ESP_14_counter = 0x10;
  }
  standaloneTx(frame);

  /*
  0xd4,0xd4,0xd4,0xd4,0xd4,0xd4,0xd4,0xd4,0xd4,0xd4,0xd4,0xd4,0xd4,0xd4
  */

  // MQB LWI_01 (0x086, DLC 8) - steering-angle sensor (Lenkwinkelinformation).
  // LWI_Lenkradwinkel (steering wheel angle * 0.1 deg), LWI_Lenkradw_Geschw (rate * 5 deg/s),
  // LWI_Sensorstatus, plus quality bits.
  frame.identifier = LWI_01; // LWI_01 0x086
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;           // [KM LWI_01.LWI_01_CRC] [REA audit S6: no CRC check]
  frame.data[1] = LWI_01_counter; // 0x10..0x1F [KM LWI_01_BZ b1[0-3], LWI_Sensorstatus b1.4=1 'nicht kalibriert', LWI_QBit_Lenkradwinkel b1.7] [REA spec §3/audit S6: counter + b1 bits 4/7 COM only, never read]
  // The 0CQ reads the steering angle from bits 16-28 (b2 + b3 bits 0-4, 0.1 deg
  // per step) and its sign from bit 29 (OEM code: spec_gen5_0cq §3). b2 is the
  // angle low byte, not a sensor status. V11.00.0: angle 0 (was 0x01 = 0.1 deg).
  frame.data[2] = 0x00;           // 0 deg [KM LWI_Lenkradwinkel low, 0.1 deg] [REA spec §3: bits 16-28 -> 0xCECC]
  frame.data[3] = 0x00;           // [KM LWI_Lenkradwinkel high b3[0-4], LWI_VZ_Lenkradwinkel b3.5, LWI_VZ_Lenkradw_Geschw b3.6, LWI_Lenkradw_Geschw low b3.7] [REA spec §3: angle + sign bit 29 read]
  frame.data[4] = 0x00;           // [KM LWI_Lenkradw_Geschw high, 5 deg/s] [REA spec §3: not read]
  frame.data[5] = 0x00;           // [KM LWI_Sub_Daten low] [REA spec §3: not read]
  frame.data[6] = 0x00;           // [KM LWI_Sub_Daten high] [REA spec §3: not read]
  frame.data[7] = 0x00;           // [KM -] [REA spec §3: not read]

  frame.data[0] = calcChecksum(frame.data, ID_SEQ_086); // for 0x086

  LWI_01_counter++;
  if (LWI_01_counter > 0x1F)
  {
    LWI_01_counter = 0x10;
  }

  standaloneTx(frame);
  /*
  0x86,0x86,0x86,0x86,0x86,0x86,0x86,0x86,0x86,0x86,0x86,0x86,0x86,0x86
  */
}

void Gen5_0CQ_frames20()
{
  twai_message_t frame = {};
  // MQB Motor_20 (0x121, DLC 8) - accelerator pedal raw + gradient + idle info.
  // The 0CQ reads bits 28-35 (b3[4-7] + b4[0-3], raw 0x94 here) as its "pedal"
  // (spec_gen5_0cq §3, name inferred) - those bits are NOT MO_Fahrpedalrohwert_01
  // in the K-matrix (that is bits 12-19), they straddle MO_Fahrpedalgradient /
  // MO_Sig_Fahrpedalgradient / MO_rel_Saugrohrdruck.
  frame.identifier = MOTOR_20;      // MOTOR_20 0x121
  frame.data_length_code = 8;       // DLC 8
  frame.data[0] = 0x00;             // [KM Motor_20.Motor_20_CRC] [REA audit S6: no CRC check]
  frame.data[1] = MOTOR_20_counter; // 0x00..0x0F [KM Motor_20_BZ b1[0-3], MO_Fahrpedalrohwert_01 low b1[4-7]] [REA spec §3: not read]
  frame.data[2] = 0x40;             // [KM MO_Fahrpedalrohwert_01 high b2[0-3] (0.4 %, = 0 %), MO_QBit_Fahrpedalwerte_01 b2.4, MO_Fahrpedalgradient low b2[5-7]] [REA spec §3: not read]
  frame.data[3] = 0x40;             // [KM MO_Fahrpedalgradient high b3[0-4] (25 %/s), MO_Sig_Fahrpedalgradient b3.5, MO_rel_Saugrohrdruck low b3[6-7]] [REA spec §3: b3[4-7] = low nibble of the 0CQ "pedal" (bits 28-35) -> 0xCFEF = raw x0xCC/0xFF]
  frame.data[4] = 0x19;             // [KM MO_rel_Saugrohrdruck high b4[0-3] (18 mbar), _gem_err b4.4, MO_Moment_im_Leerlauf b4.5, MO_Schubabschaltung b4.6, StoppVorbereitung b4.7] [REA spec §3: b4[0-3] = high nibble of the "pedal"]
  frame.data[5] = 0x59;             // [KM MO_Solldrehz_Leerlauf 10 rpm (890 rpm)] [REA spec §3: not read]
  frame.data[6] = 0x7E;             // [KM MO_Entkopplung_Sollschlupf b6[0-6] 20 rpm, MO_temporaere_Fahrerabwesenheit b6.7] [REA spec §3: not read]
  frame.data[7] = 0xFE;             // [KM TSK_a_Soll_gradientenbegrenzt b7[1-7] 0.1-7.2 m/s2] [REA spec §3: not read]

  frame.data[0] = calcChecksum(frame.data, ID_SEQ_121); // for 0x121

  MOTOR_20_counter++;
  if (MOTOR_20_counter > 0x0F)
  {
    MOTOR_20_counter = 0x00;
  }

  standaloneTx(frame);
  /*
  0xe9,0x65,0xae,0x6b,0x7b,0x35,0xe5,0x5f,0x4e,0xc7,0x86,0xa2,0xbb,0xdd,0xeb,0xb4
  */

  // MQB ESP_10 (0x116, DLC 8) - lateral dynamics / yaw + lateral accel.
  // ESP_Gierrate (yaw), ESP_Querbeschleunigung (lateral acceleration), status bits.
  // V11.00.0: the 0CQ does NOT receive ESP_10 (OEM code: spec_gen5_0cq §1 RX table,
  // D9), so block 5 is OFF in the standalone defaults. The old "affects 40 %" note
  // below must have been something else changing at the same time.
  // K-matrix: ESP_10 is the wheel-PULSE frame (ESP_Wegimpuls_*), not yaw /
  // lateral G - the old per-byte names were wrong.
  frame.identifier = ESP_10;                            // ESP_10 0x116
  frame.data_length_code = 8;                           // DLC 8
  frame.data[0] = 0x00;                                 // [KM ESP_10.ESP_10_CRC] [REA spec §1: 0x116 not received]
  frame.data[1] = ESP_10_counter;                       // 0x00..0x0F [KM ESP_10_BZ b1[0-3], ESP_QBit_Wegimpuls_VL/VR/HL/HR b1[4-7]] [REA: not received]
  frame.data[2] = 0x01;                                 // [KM ESP_Wegimpuls_VL low] [REA: not received]
  frame.data[3] = 0x04;                                 // [KM ESP_Wegimpuls_VL high b3[0-1], ESP_Wegimpuls_VR low b3[2-7]] [REA: not received]
  frame.data[4] = 0x00;                                 // [KM ESP_Wegimpuls_VR high b4[0-3], ESP_Wegimpuls_HL low b4[4-7]] [REA: not received]
  frame.data[5] = 0x40;                                 // [KM ESP_Wegimpuls_HL high b5[0-5], ESP_Wegimpuls_HR low b5[6-7]] [REA: not received]
  frame.data[6] = 0x00;                                 // [KM ESP_Wegimpuls_HR high] [REA: not received]
  frame.data[7] = 0xFF;                                 // old note "affects 40 %" - not a 0CQ effect [KM ESP_VL/VR/HL/HR_Fahrtrichtung, 2 bits each] [REA: not received]
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_116); // for 0x116

  ESP_10_counter++;
  if (ESP_10_counter > 0x0F)
  {
    ESP_10_counter = 0x00;
  }

  standaloneTx(frame);
  /*
  0xac,0xac,0xac,0xac,0xac,0xac,0xac,0xac,0xac,0xac,0xac,0xac,0xac,0xac,0xac
  */

  // MQB ESP_05 (0x106, DLC 8) - brake pressure + brake-light/brake-pedal flags.
  // BR_Bremsdruck (master cylinder pressure), BR_Bremslicht (brake-light), BR_Bremspedal (pedal active).
  // The 0CQ reads the brake pressure, bits 16-25 (b2 + b3 bits 0-1), and sets an
  // internal "braking" flag above a threshold (OEM code: spec_gen5_0cq §3).
  frame.identifier = ESP_05;      // ESP_05 0x106
  frame.data_length_code = 8;     // DLC 8
  frame.data[0] = 0x00;           // [KM ESP_05.ESP_05_CRC] [REA audit S6: no CRC check]
  frame.data[1] = ESP_05_counter; // 0x80..0x8F [KM ESP_05_BZ b1[0-3], ESP_QBit_Bremsdruck b1.4, ESP_QBit_Fahrer_bremst b1.5, ESP_Schwelle_Unterdruck b1[6-7]=2] [REA audit S6: counter sig110 never read]
  frame.data[2] = 0x64;           // raw 0x064 = 0 bar, not braking [KM ESP_Bremsdruck low, 0.3 bar -30] [REA audit S15: bits 16-25 -> "braking" flag 0xCFE1 above a threshold]
  // b3: bits 0-1 = brake pressure high bits (0), bit 2 = ESP_Fahrer_bremst
  // (driver braking), bits 6-7 = 0xC0 as before. V11.00.0: bit 2 follows
  // brakeForHaldex() ("Follow Brake" + brake input wire or the car's CAN). The
  // 0CQ itself does not read bit 2 (spec §3) but the car's other modules do -
  // so on the 0CQ "Follow Brake" never reaches the unit (pressure stays 0 bar).
  frame.data[3] = 0xC0; // [KM ESP_Bremsdruck high b3[0-1], ESP_Fahrer_bremst b3.2, ESP_Verz_TSK_aktiv b3.3, ADS/TSK/AWV2 flags b3[4-7]] [REA spec §3: b3[0-1] read (pressure), b3.2 not read]
  if (brakeForHaldex())
  {
    frame.data[3] = frame.data[3] | (1 << 2); // [KM ESP_Fahrer_bremst] [REA spec §3: not read by the 0CQ]
  }
  frame.data[4] = 0x00;                                 // [KM ECD/HDC/Autohold/HBA/Prefill flags] [REA spec §3: not read]
  frame.data[5] = 0x00;                                 // [KM ESP_Status_Anfahrhilfe, HDC_aktiv, StartStopp_Info, ESP_Eingr_* flags] [REA spec §3: not read]
  frame.data[6] = 0xFD;                                 // [KM ESP_BKV_Unterdruck 4 mbar (1012)] [REA spec §3: not read]
  frame.data[7] = 0x00;                                 // old "affects on/off" not a 0CQ effect [KM Autohold_aktiv b7.0, ESP_Verz_EPB_aktiv b7.2, ECD_Bremslicht b7.3, ...] [REA spec §3: not read]
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_106); // CRC over the finished bytes (incl. the brake bit), like applyBrakeHandbrakeCANOverride does

  ESP_05_counter++;
  if (ESP_05_counter > 0x8F)
  {
    ESP_05_counter = 0x80;
  }

  standaloneTx(frame);
  /*
  0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07
  */

  // MQB EPB_01 (0x104, DLC 8) - electronic parking brake state.
  // Not received by the 0CQ (spec_gen5_0cq §1) - block 7 is OFF in the standalone defaults.
  frame.identifier = EPB_01;                            // EPB_01 0x104
  frame.data_length_code = 8;                           // DLC 8
  frame.data[0] = 0x00;                                 // [KM EPB_01.EPB_01_CRC] [REA spec §1: 0x104 not received]
  frame.data[1] = EPB_01_counter;                       // 0x30..0x3F [KM EPB_01_BZ b1[0-3], EPB_QBit_Laengsbeschleunigung b1.4, EPB_QBit_Pedalweg_Kuppl b1.5, EPB_Freig_Verzoeg_Anf b1.7] [REA: not received]
  frame.data[2] = 0xA6;                                 // [KM EPB_Verzoeg_Anf 0.048-7.968 m/s2] [REA: not received]
  frame.data[3] = 0x00;                                 // [KM -] [REA: not received]
  frame.data[4] = 0xE6;                                 // [KM EPB_Pedalweg_Kuppl 0.4 %] [REA: not received]
  frame.data[5] = 0x00;                                 // [KM -] [REA: not received]
  frame.data[6] = 0x00;                                 // [KM EPB_DAA_Randbed_erf b6.1, EPB_Fehlerstatus b6[2-3], EPB_Schalterposition b6[4-5], QBit b6.6] [REA: not received]
  frame.data[7] = 0x31;                                 // [KM EPB_Spannkraft b7[0-4] kN (17), EPB_Status b7[5-6] (1)] [REA: not received]
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_104); // for 0x104

  EPB_01_counter++;
  if (EPB_01_counter > 0x3F)
  {
    EPB_01_counter = 0x30;
  }

  standaloneTx(frame);
  /*
  0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05
  */

  // MQB ESP_02 (0x101, DLC 8) - ESP brake/yaw status broadcast.
  // The 0CQ reads the lateral acceleration (b2) and the yaw rate (bits 40-53)
  // with its sign (bit 54) (OEM code: spec_gen5_0cq §3). V11.00.0: 0 g and no
  // yaw (was -0.01 g and 0.12 deg/s) - a car driving straight.
  frame.identifier = ESP_02;                            // ESP_02 0x101
  frame.data_length_code = 8;                           // DLC 8
  frame.data[0] = 0x00;                                 // [KM ESP_02.ESP_02_CRC] [REA audit S6: no CRC check]
  frame.data[1] = ESP_02_counter;                       // 0x00..0x1F [KM ESP_02_BZ b1[0-3], ESP_QBit_Gierrate b1.4 (toggles with the counter), QBit_Laengsbeschl b1.5, QBit_Querb b1.6] [REA spec §3/audit S6: b1 not read, counter sig102 never read]
  frame.data[2] = 0x7F;                                 // 0 g [KM ESP_Querbeschleunigung 0.01 g -1.27] [REA spec §3: b2-0x7F signed -> 0xCED0]
  frame.data[3] = 0x0F;                                 // [KM ESP_Laengsbeschl low, 0.03125 m/s2 -16] [REA spec §3: bits 24-33 COM only, not read]
  frame.data[4] = 0x82;                                 // [KM ESP_Laengsbeschl high b4[0-1], ESC/Anschleppstart flags b4[2-6], ESP_QBit_Anf_Vert_Wank b4.7] [REA spec §3: not read]
  frame.data[5] = 0x00;                                 // 0 deg/s [KM ESP_Gierrate low, 0.01 deg/s] [REA spec §3: bits 40-53 -> 0xCECE]
  frame.data[6] = 0x00;                                 // [KM ESP_Gierrate high b6[0-5], ESP_VZ_Gierrate b6.6, ESP_Notbremsanzeige b6.7] [REA spec §3: yaw + sign bit 54 read]
  frame.data[7] = 0x00;                                 // [KM ESP_SpannungsAnf b7.0, ESP_PLA_Abbruch b7[1-3], ESP_Status_ESP_PLA b7[4-7]] [REA spec §3: not read]
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_101); // for 0x101

  ESP_02_counter++;
  if (ESP_02_counter > 0x1F)
  {
    ESP_02_counter = 0x00;
  }

  standaloneTx(frame);
  /*
  0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa
  */

  // MQB ESP_21 (0x0FD, DLC 8) - ESP wheel-speed/dynamics diagnostics.
  // The 0CQ reads only b7 bits 0, 1, 4, 5 = ABS_Bremsung, ASR_Anf, EDS_Eingriff,
  // ESP_Eingriff (OEM code: spec_gen5_0cq §3; K-matrix lists Allrad_Haldex as a
  // receiver). V11.00.0: ABS_Bremsung carries absBrakingForHaldex() (the car's ABS
  // module braking, from the chassis CAN); the other three stay 0 (no intervention).
  frame.identifier = ESP_21;      // ESP_21 0x0fd
  frame.data_length_code = 8;     // DLC 8
  frame.data[0] = 0x00;           // [KM ESP_21.ESP_21_CRC] [REA audit S6: no CRC check]
  frame.data[1] = ESP_21_counter; // 0x00..0x1F [KM ESP_21_BZ b1[0-3], BR_Eingriffsmoment low b1[4-7]] [REA audit S6: counter sig123 never read]
  frame.data[2] = 0x1F;           // [KM BR_Eingriffsmoment high b2[0-5] (raw-509 Nm), ESP_PLA_Bremseingriff b2.6, ESP_Diagnose b2.7] [REA spec §3: not read]
  frame.data[3] = 0x80;           // [KM ESC_Reku_Freigabe b3.0, v_Signal_Qualifier b3[1-3], ESP_Vorsteuerung b3.4, OBD_QBit_Schlechtweg b3.7=1] [REA spec §3: not read]
  frame.data[4] = 0x00;           // [KM ESP_v_Signal low, 0.01 km/h] [REA spec §3: not read]
  frame.data[5] = 0x00;           // [KM ESP_v_Signal high] [REA spec §3: not read]
  frame.data[6] = 0x00;           // [KM ASR/ESP_Tastung_passiv, ESP_Systemstatus, ASR_Schalteingriff, ..., ESP_QBit_v_Signal b6.7] [REA spec §3: not read]
  frame.data[7] = 0x00;           // no intervention [KM ABS_Bremsung b7.0, ASR_Anf b7.1, MSR_Anf b7.2, EBV b7.3, EDS_Eingriff b7.4, ESP_Eingriff b7.5] [REA spec §3: b7 bits 0/1/4/5 -> 0xCFDE/0xCFE3/0xCFE4/0xCFE0]
  if (absBrakingForHaldex())
  {
    frame.data[7] = frame.data[7] | (1 << 0); // [KM ESP_21.ABS_Bremsung] [REA spec §3: read]
  }
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_0fd); // for 0x0fd (over the finished bytes)

  ESP_21_counter++;
  if (ESP_21_counter > 0x1F)
  {
    ESP_21_counter = 0x00;
  }

  standaloneTx(frame);

  /*
  0xb4,0xef,0xf8,0x49,0x1e,0xe5,0xc2,0xc0,0x97,0x19,0x3c,0xc9,0xf1,0x98,0xd6,0x61
  */
}

void Gen5_0CQ_frames25()
{
  twai_message_t frame = {};
  // MQB Kombi_01 (0x30B, DLC 8) - instrument cluster broadcast.
  // KBI_Kilometerstand (odometer), KBI_Geschw_Anzeige (displayed speed), warning lamps.
  // The 0CQ reads only b2 bit 7 = KBI_Handbremse (OEM code: spec_gen5_0cq §3,
  // and only while ESP_20 b1 bits 4-5 are not 3). V11.00.0: it carries
  // handbrakeForHaldex() ("Follow Handbrake" + wire or the car's CAN); 0 = released.
  // NOTE: our ESP_20 below sends b1[4-5] = 3 (the counter runs 0x30..0x3F =
  // BR_Systemart "ESP_mit_integrierter_EPB"), and spec §3 says the 0CQ only uses
  // KBI_Handbremse while ESP_20 b1[4-5] != 3 (and cal 0xC0B6CF = 1) - so with our
  // own ESP_20 this bit is probably ignored by the unit (row not re-audited).
  frame.identifier = KOMBI_01; // Kombi_01 0x30B
  frame.data_length_code = 8;  // DLC 8
  frame.data[0] = 0x10;        // [KM KBI_ABS/ESP/BKL/Airbag_Lampe b0[0-3]=0, KBI_SILA_gueltig b0.4=1, ...] [REA spec §3: not read]
  frame.data[1] = 0x20;        // BZ held at 0 [KM Kombi_01_BZ b1[0-3], KBI_Anzeigestatus_GRA b1[5-6]=1] [REA spec §3: not read]
  frame.data[2] = 0x02;        // [KM KBI_MFA_v_Einheit_01 b2.1=1, KBI_Handbremse b2.7=0] [REA spec §3: only b2.7 -> 0xCFE2, gated by ESP_20 b1[4-5] != 3]
  if (handbrakeForHaldex())
  {
    frame.data[2] = frame.data[2] | (1 << 7); // [KM Kombi_01.KBI_Handbremse] [REA spec §3: read, see gate above]
  }
  frame.data[3] = 0x00;        // (not odometer) [KM KBI_V_Digital low] [REA spec §3: not read]
  frame.data[4] = 0x0C;        // [KM KBI_V_Digital high b4.0, PLA/NV/LIM display flags b4[1-5]] [REA spec §3: not read]
  frame.data[5] = 0x00;        // [KM KBI_Audioabsenkung_MIB b5[0-1]] [REA spec §3: not read]
  frame.data[6] = 0x00;        // [KM KBI_angez_Geschw low, 0.32 km/h] [REA spec §3: not read]
  frame.data[7] = 0x24;        // [KM KBI_angez_Geschw high b7[0-1], KBI_Einheit_Tacho b7.2, ACC/SWA flags, KBI_Systemstatus b7.7] [REA spec §3: not read]
  standaloneTx(frame);
}

void Gen5_0CQ_frames100()
{
  twai_message_t frame = {};
  // MQB ESP_23 (0x5BE, DLC 8) - longitudinal/lateral acceleration, tyre data.
  // BR_Laengsbeschleunigung (longitudinal G), BR_Querbeschleunigung (lateral G), BR_Tire_Circumference.
  frame.identifier = ESP_23;                            // ESP_23 0x5be - this is fixed in Savvy but CHKS in Kmatrix?
  frame.data_length_code = 8;                           // DLC 8
  frame.data[0] = 0x00;                                 // [KM ESP_23.ESP_23_CRC] [REA audit R6: 0x5BE not in the 0CQ or VAQ RX table]
  frame.data[1] = ESP_23_counter;                       // [KM ESP_23_BZ b1[0-3], ESP_Gierrate_Offset_2 low b1[4-7]] [REA: not received]
  frame.data[2] = 0xBF;                                 // [KM ESP_Gierrate_Offset_2 high b2[0-6] (0.01-10.23 deg/s), ESP_Lenkwinkel_Offset low b2.7] [REA: not received]
  frame.data[3] = 0x7F;                                 // [KM ESP_Lenkwinkel_Offset high (0.1-25.5 deg)] [REA: not received]
  frame.data[4] = 0x00;                                 // [KM BR_Reifenumfang_sekundaer low, mm] [REA: not received]
  frame.data[5] = 0x00;                                 // [KM BR_Reifenumfang_sekundaer high b5[0-3], BR_QBit_Reifenumfang_sekundaer b5.4] [REA: not received]
  frame.data[6] = 0x7C;                                 // [KM ESP_Querbeschleunigung_Offset b6[2-7] (0.01-0.3 g)] [REA: not received]
  frame.data[7] = 0x78;                                 // [KM ESP_Laengsbeschleunigung_Offset (0.03125-3.9375 m/s2)] [REA: not received]
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_5be); // for 0x5be

  ESP_23_counter++;
  if (ESP_23_counter > 0x1F)
  {
    ESP_23_counter = 0x00;
  }

  standaloneTx(frame);
  /*
    0xc9,0x21,0x6f,0x63,0xd2,0x42,0x6a,0x77,0x4a,0x3d,0xb0,0x62,0x9f,0x38,0xcd,0x5c
    */

  // MQB Parkhilfe_04 (0x54B, DLC 8) - park-assist module state broadcast.
  frame.identifier = Parkhilfe_04; // Parkhilfe_04 0x54B
  frame.data_length_code = 8;      // DLC 8
  frame.data[0] = 0x00;            // [KM -] [REA audit R6: 0x54B not in the 0CQ or VAQ RX table]
  frame.data[1] = 0x00;            // [KM PH_Verschmutzungsmeldung b1[4-7]] [REA: not received]
  frame.data[2] = 0x00;            // [KM PH_Aufschaltursache b2[0-4], PH_Ton_Ausgabe b2[5-7]] [REA: not received]
  frame.data[3] = 0x00;            // [KM PH_Ton_Pausenlaenge] [REA: not received]
  frame.data[4] = 0x00;            // [KM -] [REA: not received]
  frame.data[5] = 0x00;            // [KM -] [REA: not received]
  frame.data[6] = 0x00;            // [KM PH_Ton_Richtung b6[1-3], PH_Ton_Lautstaerke b6[4-6], BCM_WAH_Meldung low b6.7] [REA: not received]
  frame.data[7] = 0x24;            // [KM BCM_WAH_Meldung high b7[0-1], PDC_Charisma_Status b7[2-3], PDC_Charisma_FahrPr b7[4-7]] [REA: not received]
  standaloneTx(frame);

  // MQB Gateway_72 (0x3DB, DLC 8) - gateway routing/diagnostic broadcast.
  frame.identifier = GATEWAY_72; // Gateway_72 0x3DB
  frame.data_length_code = 8;    // DLC 8
  frame.data[0] = 0x50;          // [KM Gateway_72 *_alt timeout flags (BCM_01/SMLS_01/ZV_02/Wischer_01/...)] [REA audit R6: 0x3DB not in the 0CQ or VAQ RX table]
  frame.data[1] = 0x80;          // [KM WFS/Licht/ZV flags, BCM1_LDS_Stellung low b1.7] [REA: not received]
  frame.data[2] = 0x00;          // [KM BCM1_LDS_Stellung high, Verdeck/door flags] [REA: not received]
  frame.data[3] = 0x00;          // [KM door/wiper flags, AAG_Anhaenger_erkannt b3.4] [REA: not received]
  frame.data[4] = 0x05;          // [KM KL_Thermomanagement b4[0-1], WFS_Schluessel_Fahrberecht b4[2-5], ...] [REA: not received]
  frame.data[5] = 0x10;          // [KM ZV_verriegelt_soll b5[2-3], ZV_Schluessel_Zugang b5[4-7], ...] [REA: not received]
  frame.data[6] = 0x01;          // [KM WFS_ID_Geb_autorisiert b6.0, BH_Fernlicht/Blinker b6[1-3], BCM1_OBD_FStatus_ATemp b6[4-7]] [REA: not received]
  frame.data[7] = 0x78;          // [KM BCM1_Aussen_Temp_ungef 0.5-50 degC (10 degC)] [REA: not received]
  standaloneTx(frame);

  // MQB Getriebe_14 (0x3C8, DLC 8) - transmission slow broadcast (Charisma, drag torque, launch).
  frame.identifier = GETRIEBE_14; // Getriebe_14 0x3C8
  frame.data_length_code = 8;     // DLC 8
  frame.data[0] = 0x00;           // [KM -] [REA audit R6: 0x3C8 not in the 0CQ or VAQ RX table]
  frame.data[1] = 0x00;           // [KM GE_OBD_AbsperrVent b1[4-7]] [REA: not received]
  frame.data[2] = 0x54;           // [KM GE_amax_moeglich low, 0.024-2.016 m/s2] [REA: not received]
  frame.data[3] = 0x24;           // [KM GE_amax_moeglich high b3.0, GE_Charisma_FahrPr b3[1-4], GE_Charisma_Status b3[5-6], WFS b3.7] [REA: not received]
  frame.data[4] = 0x00;           // [KM GE_Verlustmoment, Nm] [REA: not received]
  frame.data[5] = 0x60;           // [KM Freilauf_Charisma_FahrPr b5[0-3], Freilauf_Charisma_Status b5[4-5], WFS b5[6-7]] [REA: not received]
  frame.data[6] = 0x01;           // [KM GE_Zustand_Kuppltemp b6[0-1], GE_LaunchControl b6.3, GE_Heizwunsch b6[4-5], ...] [REA: not received]
  frame.data[7] = 0x51;           // [KM GE_Sumpftemperatur -58 degC (23 degC)] [REA: not received]
  standaloneTx(frame);

  // MQB Motor_14 (0x3BE, DLC 8) - start/stop subsystem state broadcast.
  // MO_StSt_Status (state machine), MO_StSt_Restart (restart event), MO_StSt_Stop (stop event).
  frame.identifier = MOTOR_14;                          // Motor_14 0x3BE
  frame.data_length_code = 8;                           // DLC 8
  frame.data[0] = 0x00;                                 // [KM Motor_14.Motor_14_CRC] [REA 0CQ audit S6: no CRC check]
  frame.data[1] = MOTOR_14_counter;                     // 0x10..0x1F [KM Motor_14_BZ b1[0-3], MO_StartStopp_Status b1[4-5]=1, _Wiederstart b1.6, _Motorstopp b1.7] [REA spec §3: not read]
  frame.data[2] = 0xE6;                                 // [KM MO_Freig_Reku b2[0-1], MO_Kl_75 b2.2, MO_Kl_50 b2.3, MO_Gangposition b2[4-7]=14 'undefined'] [REA spec §3: b2[4-7] = gear, used only if Getriebe_11 is missing (0xD->0xFF, >8->0); VAQ getter not traced]
  frame.data[3] = 0x01;                                 // old "affects on/off" [KM MO_StartStopp_Fahrerwunsch b3[0-1]=1, MO_Fahrer_bremst b3.4, MO_QBit_Fahrer_bremst b3.5, MO_BLS b3.6] [REA spec §3: b3 bits 4/5 COM only, not read by the 0CQ]
  frame.data[4] = 0xC8;                                 // [KM MO_Timeout_ESP b4.0, MO_Aussp_Anlass b4.3, MO_Kuppl_schalter b4.5=0, MO_Interlock b4.6, MO_Motor_laeuft b4.7] [REA spec §3: b4.5 -> 0xCFF3 = (bit != 1)]
  frame.data[5] = 0x80;                                 // [KM MO_Kickdown b5.0, ..., MO_PTC_Status low b5.7] [REA spec §3: not read]
  frame.data[6] = 0x00;                                 // [KM MO_PTC_Status high, MO_QBit_Gangposition b6.2, MO_Signalquelle_Gangposition b6.3, ...] [REA spec §3: not read]
  frame.data[7] = 0x80;                                 // [KM MO_FMAus_Startvariante, MO_Zylinderabschaltung, MO_StartVorauss_erfuellt b7[6-7]] [REA spec §3: not read]
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_3be); // for 0x3be

  MOTOR_14_counter++;
  if (MOTOR_14_counter > 0x1F)
  {
    MOTOR_14_counter = 0x10;
  }

  standaloneTx(frame);
  /*
  0x1f,0x28,0xc6,0x85,0xe6,0xf8,0xb0,0x19,0x5b,0x64,0x35,0x21,0xe4,0xf7,0x9c,0x24
  */

  // MQB ESP_07 (0x392, DLC 8) - ESP brake-event/intervention diagnostics.
  frame.identifier = ESP_07;                            // ESP_07 0x392
  frame.data_length_code = 8;                           // DLC 8
  frame.data[0] = 0x00;                                 // [KM ESP_07.ESP_07_CRC] [REA 0CQ audit S6: no CRC check]
  frame.data[1] = ESP_07_counter;                       // 0x20..0x2F [KM ESP_07_BZ b1[0-3], ESP_ACC_LDE b1.4, ESP_Allrad_Antrieb b1.5=0, ESP_Codierung_ADS b1[6-7]] [REA spec §3: not read]
  frame.data[2] = 0x00;                                 // [KM ESP_RTA_HL 0.0488 % -6.2] [REA spec §3: not read]
  frame.data[3] = 0x00;                                 // [KM ESP_RTA_HR] [REA spec §3: not read]
  frame.data[4] = 0x00;                                 // [KM ESP_RTA_VR] [REA spec §3: not read]
  frame.data[5] = 0x00;                                 // [KM OBD_Fehler_Radsensor_HL b5[0-3], _HR b5[4-7]] [REA spec §3: not read]
  frame.data[6] = 0x00;                                 // [KM OBD_Fehler_Radsensor_VL b6[0-3], _VR b6[4-7]] [REA spec §3: not read]
  frame.data[7] = 0x00;                                 // [KM ESP_Qualifizierung_Antriebsart b7.0, ..., ESP_Rollenmodus_Deaktivieren b7.6=0] [REA spec §3: b7.6 only, used with engine speed in FUN_c4320c; VAQ getter not traced]
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_392); // for 0x392

  ESP_07_counter++;
  if (ESP_07_counter > 0x2F) // V11.00.0: was "> 0x20 -> 0x2F", which ran backwards and stuck at 0x2F (spec_gen5_0cq D15; harmless, the 0CQ doesn't read it)
  {
    ESP_07_counter = 0x20; // counter runs 0x20..0x2F
  }
  standaloneTx(frame);
  /*
   0x91,0x91,0x91,0x91,0x91,0x91,0x91,0x91,0x91,0x91,0x91,0x91,0x91,0x91
   */

  // MQB ESP_29 (0x18C, DLC 8) - ESP slow-rate diagnostic broadcast.
  frame.identifier = ESP_29;  // ESP_29 0x18C
  frame.data_length_code = 8; // DLC 8
  frame.data[0] = 0x00;       // [KM - (ESP_29 has no CRC in the K-matrix)] [REA audit R6: 0x18C not in the 0CQ or VAQ RX table]
  frame.data[1] = 0x20;       // held (no roll seen in SavvyCAN) [KM - b1[0-3] (no BZ), ESP_Monitoring_Gierrate b1[4-6]=2, ESP_Monitoring_Laengsb low b1.7] [REA: not received]
  frame.data[2] = 0x59;       // [KM ESP_Monitoring_Laengsb high b2[0-1], _Lenkwinkel b2[2-4], _Querb b2[5-7]] [REA: not received]
  frame.data[3] = 0x00;       // [KM OBD_Fehler_Bremsdruck b3[0-3]] [REA: not received]
  frame.data[4] = 0x00;       // [KM -] [REA: not received]
  frame.data[5] = 0x00;       // [KM -] [REA: not received]
  frame.data[6] = 0x00;       // [KM -] [REA: not received]
  frame.data[7] = 0x00;       // [KM -] [REA: not received]
  standaloneTx(frame);
}

void Gen5_0CQ_frames200()
{
  twai_message_t frame = {};
  // PQ Kombi_2 (0x420, DLC 8) - cluster temps (vw_pq.dbc: Kombi_2), same bytes as the
  // Gen4 / 0AY copy. Transmit currently disabled (commented out below).
  // [KM MQB: - (PQ-only ID)] [REA audit R6: 0x420 not in the 0CQ RX table] - byte names are vw_pq Kombi_2.
  frame.identifier = mKombi_2; // Kombi_2 0x420
  frame.data_length_code = 8;  // DLC 8
  frame.data[0] = 0x4C;        // Kuehlmitteltemp (coolant * 0.75 - 48 degC)
  frame.data[1] = 0x86;        // Oeltemperatur (oil temp)
  frame.data[2] = 0x85;        // Aussentemperatur_gefiltert (outside-air filtered)
  frame.data[3] = 0x00;        // Aussentemperatur_ungefiltert (outside-air raw)
  frame.data[4] = 0x00;        // reserved
  frame.data[5] = 0x30;        // status
  frame.data[6] = 0xFF;        // reserved
  frame.data[7] = 0x04;        // reserved / counter
  // standaloneTx(frame);
}

void Gen5_0CQ_frames1000()
{
  twai_message_t frame = {};
  // MQB Motor_07 (0x640, DLC 8) - engine slow-rate diagnostics broadcast.
  frame.identifier = MOTOR_07; // Motor_07 0x640, 1000 ms
  frame.data_length_code = 8;  // DLC 8
  frame.data[0] = 0xA0;        // (no CRC in KM) [KM Motor_07 QBit_Ansaugluft/Oel/Kuehlmittel_Temp b0[0-2], KL15_Abschaltempfehlung b0.3, ...] [REA: 0CQ not received (audit R6); VAQ in RX table (audit §4), getter not traced]
  frame.data[1] = 0x5A;        // [KM MO_Ansaugluft_Temp 0.75-48 degC] [REA: as b0]
  frame.data[2] = 0x56;        // [KM MO_Oel_Temp -60 degC] [REA: as b0]
  frame.data[3] = 0xA3;        // [KM MO_Kuehlmittel_Temp 0.75-48 degC] [REA: as b0]
  frame.data[4] = 0x80;        // [KM MO_Hoeheninfo 0.00781] [REA: as b0]
  frame.data[5] = 0xA0;        // [KM MO_Kennfeldk b5.0, MO_Versionsinfo_02 b5[1-5], QBit_Hoeheninfo b5.6, Getriebe_kuehlen b5.7] [REA: as b0]
  frame.data[6] = 0x59;        // [KM MO_Mom_Traegheit_02 b6[0-4], MO_Heizungspumpenansteuerung b6[5-7]] [REA: as b0]
  frame.data[7] = 0x01;        // [KM Heizungspumpe high b7.0, MO_SpannungsAnf b7.1, MO_Nachlaufzeit_Heizungspumpe b7[2-7]] [REA: as b0]
  standaloneTx(frame);

  frame.identifier = CHARISMA_01; // Charisma_01 0x385
  frame.data_length_code = 8;     // DLC 8 no effect from any
  frame.data[0] = 0x00;           // (no CRC in KM) [KM Charisma_01 CHA_Ziel_FahrPr_AGA b0[0-3], _ESP b0[4-7]] [REA: 0CQ not received (audit R6); VAQ in RX table (audit §4), getter not traced]
  frame.data[1] = 0x00;           // [KM CHA_Ziel_FahrPr_Freilauf b1[0-3], CHA_Fahrer_Umschaltung b1.6, _BMA b1.7] [REA: as b0]
  frame.data[2] = 0x22;           // [KM CHA_Ziel_FahrPr_MO b2[0-3]=2, _GE b2[4-7]=2] [REA: as b0]
  frame.data[3] = 0x02;           // [KM CHA_Ziel_FahrPr_ALR b3[0-3]=2 (Haldex), _MO_BZS b3[4-7]] [REA: as b0]
  frame.data[4] = 0x02;           // [KM CHA_Ziel_FahrPr_DR b4[0-3]=2, CHA_Ziel_FahrPr_VAQ b4[4-7]=0 'keine_Funktion'] [REA: as b0]
  frame.data[5] = 0x20;           // [KM CHA_Ziel_FahrPr_AFS b5[0-3], _RGS b5[4-7]] [REA: as b0]
  frame.data[6] = 0x02;           // [KM CHA_Ziel_FahrPr_EPS b6[0-3], _ACC b6[4-7]] [REA: as b0]
  frame.data[7] = 0x02;           // [KM CHA_Ziel_FahrPr_SAK b7[0-3], _MO_StSt b7[4-7]] [REA: as b0]
  standaloneTx(frame);

  // MQB Systeminfo_01 (0x585, DLC 8) - system identification/info broadcast.
  frame.identifier = SYSTEMINFO_01; // Systeminfo_01 0x585
  frame.data_length_code = 8;       // DLC 8
  frame.data[0] = 0x84;             // (no CRC in KM) [KM Systeminfo_01 SI_Sammel_SG_Fehler b0[0-5], SI_P_Mode_gueltig b0.6, SI_Diagnose_Aktiv b0.7] [REA spec §3: not read]
  frame.data[1] = 0x3C;             // [KM SI_QRS_Mode b1.0, SI_T_Mode b1.1, SI_NWDF b1.2=1, SI_NWDF_gueltig b1.3=1, SI_Sammelfehler b1.4=1, GW_KD_Fehler b1.5=1, ...] [REA spec §3: b1 bits 2/3 only, monitor-enable (FUN_c20236/c4b482); VAQ getter not traced]
  frame.data[2] = 0x00;             // [KM SI_BUS_01..08] [REA spec §3: not read]
  frame.data[3] = 0x7F;             // [KM SI_BUS_09..15] [REA spec §3: not read]
  frame.data[4] = 0x14;             // [KM SI_Bus_Identifikation] [REA spec §3: not read]
  frame.data[5] = 0x00;             // [KM -] [REA spec §3: not read]
  frame.data[6] = 0x00;             // [KM -] [REA spec §3: not read]
  frame.data[7] = 0x00;             // [KM -] [REA spec §3: not read]
  standaloneTx(frame);

  // MQB Motor_Code_01 (0x641, DLC 8) - engine code / identification broadcast.
  // The 0CQ reads b1 bits 4-5 = MO_Faktor_Momente (1/2/3), which MULTIPLIES the
  // Motor_11 torque, and b4 bits 4-7 once at start-up (must not be 0) (OEM code:
  // spec_gen5_0cq §3). V11.00.0: the factor is now set on purpose (= 1) instead
  // of only being 1 because the counter happened to run 0x10..0x1F (spec D18),
  // and the duplicated header lines are gone (D16).
  frame.identifier = MOTOR_CODE_01; // Motor_Code_01 0x641
  frame.data_length_code = 8;       // DLC 8
  frame.data[0] = 0x00;             // [KM Motor_Code_01.Motor_Code_01_CRC] [REA audit S6: no CRC check]
  frame.data[1] = (1 << 4) | (MOTOR_CODE_01_counter & 0x0F); // [KM Motor_Code_01_BZ b1[0-3], MO_Faktor_Momente_02 b1[4-5]=1, MO_Hybridfahrzeug b1[6-7]=0] [REA audit A20/S15: b1[4-5] multiplies the Motor_11 torque]
  frame.data[2] = 0x2B;             // [KM MO_Code] [REA spec §3: not read]
  frame.data[3] = 0x53;             // [KM MO_Getriebe_Code b3[0-5]=19 DQ250, MO_StartStopp_Codiert b3.6=1, MO_Anzeige_StSt_gueltig b3.7] [REA spec §3: not read]
  frame.data[4] = 0x14;             // [KM MO_Anzahl_Zyl b4[0-3]=4, MO_Kraftstoffart b4[4-7]=1 Benzin] [REA audit S15: b4[4-7] latched once -> 0xCEF8 = 1 (!=0) / 2 (=0)]
  frame.data[5] = 0x14;             // [KM MO_Hubraum b5[0-6] 0.1 l (2.0 l), MO_Ansaugsystem b5.7=0 Turbo] [REA spec §3: not read]
  frame.data[6] = 0xD7;             // [KM MO_Leistung low (kW)] [REA spec §3: not read]
  frame.data[7] = 0x24;             // [KM MO_Leistung high b7.0 (215 kW), MO_OBD_Typ b7[1-2], MO_DPF_verbaut b7.3, TSK_Codierung b7[4-6]] [REA spec §3: not read]
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_641); // V11.00.0: real CRC (was left 0). The 0CQ checks none, the car's other modules may

  MOTOR_CODE_01_counter++;
  if (MOTOR_CODE_01_counter > 0x1F)
  {
    MOTOR_CODE_01_counter = 0x10;
  }

  standaloneTx(frame);
  /*
  0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47
  */

  // MQB ESP_20 (0x65D, DLC 8) - ESP slow diagnostic broadcast incl. tyre circumference.
  frame.identifier = ESP_20;                            // ESP_20 0x65D
  frame.data_length_code = 8;                           // DLC 8 (no effect from any byte)
  frame.data[0] = 0x00;                                 // [KM ESP_20.ESP_20_CRC] [REA 0CQ audit S6: no CRC check]
  frame.data[1] = ESP_20_counter;                       // 0x30..0x3F [KM ESP_20_BZ b1[0-3], BR_Systemart b1[4-5]=3 'ESP_mit_integrierter_EPB', ESP_SpannungsAnf_02 b1[6-7]] [REA spec §3/§8.8: b1[4-5] read (0CQ + VAQ); 3 = gate that disables the Kombi_01 handbrake flag]
  frame.data[2] = 0x2B;                                 // [KM ESP_Zaehnezahl (43)] [REA spec §3: not read]
  frame.data[3] = 0x10;                                 // [KM ESP_Charisma_FahrPr b3[0-3], ESP_Charisma_Status b3[4-5]=1, Wiederstart_Anz b3[6-7]] [REA spec §3: not read]
  frame.data[4] = 0x00;                                 // [KM ESP_Wiederstart/Stoppverbot_Anz flags] [REA spec §3: not read]
  frame.data[5] = 0x00;                                 // [KM Stoppverbot_Anz, ESP_Dachrelingsensor b5[2-3], HDC_Charisma_FahrPr low b5[5-7]] [REA spec §3: not read]
  frame.data[6] = 0xE2;                                 // [KM HDC_Charisma_Status b6[1-2], BR_QBit_Reifenumfang b6.3=0, BR_Reifenumfang low b6[4-7]] [REA spec §3: bits 52-63 read; b6.3 COM only]
  frame.data[7] = 0x79;                                 // 0x79E = 1950 mm [KM BR_Reifenumfang high, mm] [REA spec §3: tyre circumference -> rolling radius 0xCEBE = mm x1024/6283]
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_65d); // for 0x65d

  ESP_20_counter++;
  if (ESP_20_counter > 0x3F)
  {
    ESP_20_counter = 0x30;
  }

  standaloneTx(frame);
  /*
  0xac,0xb3,0xab,0xeb,0x7a,0xe1,0x3b,0xf7,0x73,0xba,0x7c,0x9e,0x06,0x5f,0x02,0xd9
  */

  // MQB Diagnose_01 (0x6B2, DLC 8) - generic diagnostic broadcast.
  frame.identifier = DIAGNOSE_01; // Diagnose_01 0x6B2
  frame.data_length_code = 8;     // DLC 8
  frame.data[0] = 0x30;           // (no CRC in KM) [KM Diagnose_01.DGN_Verlernzaehler] [REA spec §3: b0 drive-cycle counter (FUN_c4c198), fault memory only - not control]
  frame.data[1] = 0x4D;           // [KM KBI_Kilometerstand bits 8-27, km] [REA spec §3: mileage stamp for fault memory (FUN_c1c582)]
  frame.data[2] = 0x58;           // [KM KBI_Kilometerstand] [REA spec §3: as b1]
  frame.data[3] = 0xA2;           // [KM KBI_Kilometerstand high b3[0-3], UH_Jahr low b3[4-7] (+2000)] [REA spec §3: fault-memory time stamp]
  frame.data[4] = 0x89;           // [KM UH_Jahr high b4[0-2], UH_Monat b4[3-6], UH_Tag low b4.7] [REA spec §3: time stamp]
  frame.data[5] = 0x85;           // [KM UH_Tag high b5[0-3], UH_Stunde low b5[4-7]] [REA spec §3: time stamp]
  frame.data[6] = 0x3F;           // car toggles 3F/BF [KM UH_Stunde high b6.0, UH_Minute b6[1-6], UH_Sekunde low b6.7] [REA spec §3: time stamp]
  frame.data[7] = 0x30;           // car steps 2D,2D,2E,2E.. [KM UH_Sekunde high b7[0-4], Vehicle_Driving_Cycle b7.5, Kombi_02_alt b7.6, Uhrzeit_01_alt b7.7] [REA spec §3: time stamp]
  standaloneTx(frame);

  // MQB Kombi_02 (0x6B7, DLC 8) - instrument cluster slow-rate broadcast.
  frame.identifier = KOMBI_02; // Kombi_02 0x6B7
  frame.data_length_code = 8;  // DLC 8
  frame.data[0] = 0x4D;        // (no CRC in KM) [KM Kombi_02.KBI_Kilometerstand bits 0-19, km] [REA spec §3: not read]
  frame.data[1] = 0x58;        // [KM KBI_Kilometerstand] [REA spec §3: not read]
  frame.data[2] = 0xF2;        // [KM KBI_Kilometerstand high b2[0-3], KBI_Standzeit_02 low b2[4-7], s] [REA spec §3: bits 20-36 read ONCE at start, <= 0x1FFFC seeds the clutch-temperature model 0xCEF6]
  frame.data[3] = 0xEE;        // [KM KBI_Standzeit_02] [REA spec §3: as b2]
  frame.data[4] = 0x04;        // standing time 20207 s [KM KBI_Standzeit_02 high b4.0, KBI_Reserve_Warnung_Sek b4.6, KBI_RDK_Kalibrier_Anf b4.7] [REA spec §3: bit 36 = last Standzeit bit]
  frame.data[5] = 0x2B;        // [KM KBI_Inhalt_Tank b5[0-6] l, KBI_FStatus_Tank b5.7] [REA spec §3: not read]
  frame.data[6] = 0x00;        // [KM KBI_aktive_Laengsfunktion b6[0-2], ..., KBI_QBit_Aussen_Temp_gef b6.7] [REA spec §3: not read]
  frame.data[7] = 0x78;        // [KM KBI_Aussen_Temp_gef 0.5-50 degC (10 degC)] [REA spec §3: not read]
  standaloneTx(frame);
}

// =============================================================================
// Gen5 (0CQ) VAQ Standalone Frames - generation 52
// A copy of the Gen5 0CQ set (same IDs, bytes, counters, CRCs and
// standaloneTx gating - gen 52 shares the FE_GEN_50 frame-edit blocks). 
// Each frame carries a 'VAQ K-matrix' note saying which signals
// the VAQ node is listed as receiving (MQB FCAN K-matrix, node 'VAQ'); frames
// marked NOT in its receive list are the first candidates to drop. 
// Feedback comes back on Quersperre_03 (0x137) rather than Allrad_03 (0x118);
// diagnostics on 0x71E/0x788 rather than 0x70F/0x779 (both decoded/selectable).
// REA (audit_gen5_0cq §4, VAQ 0CQ907554A/E dump 0xC10000-0xC4FFFF): the VAQ RX
// table = the 0CQ's 18 operational IDs + 0x640 Motor_07 + 0x385 Charisma_01, so
// ESP_18, ESP_10, EPB_01, ESP_23, Parkhilfe_04, Gateway_72, Getriebe_14 and
// ESP_29 are NOT received. Traced VAQ getters: ESP_14 b3[3-5]/b4/b5 and Motor_11
// Soll/Ist + one b6-b7 bit only; every other "[REA spec §3 ...]" tag in this set
// is the 0CQ's reading (same COM table address, getters not traced on the VAQ).
// All VAQ scales (ESP_14 0.4 %, 0x137 b2, 0x2BF3) live in the calibration
// block 0xC00000-0xC0FFFF that is MISSING from the dump - bench-only, not REA.
// =============================================================================
void Gen5_0CQ_VAQ_frames10()
{
  twai_message_t frame = {};
  // MQB ESP_18 (0x135, DLC 8) - ESP minor broadcast. Fixed response, no changes.
  // VAQ K-matrix: NOT in the VAQ receive list (no VAQ signal on ESP_18). Kept
  // like-for-like; block 10 can drop it to prove it is dead weight on this unit.
  frame.identifier = ESP_18; // ESP_18 0x135
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // [KM ESP_18.ESP_18_CRC] [REA audit §4: 0x135 not in the VAQ RX table]
  frame.data[1] = 0xC0; // [KM ESP_18_BZ b1[0-3]=0, ESP_St_V_fzg_ref b1[4-5], ESP_v_fzg_ref_FR b1[6-7]=3] [REA: not received]
  frame.data[2] = 0x00; // [KM ESP_Allrad_Moment_Anforderung 8 Nm/bit] [REA: not received]
  frame.data[3] = 0x00; // [KM ESP_Allrad_Anforderung b3[0-4]] [REA: not received]
  frame.data[4] = 0x00; // [KM -] [REA: not received]
  frame.data[5] = 0x00; // [KM -] [REA: not received]
  frame.data[6] = 0x00; // [KM ESP_V_fzg_ref low, 0.01 km/h] [REA: not received]
  frame.data[7] = 0x00; // [KM ESP_V_fzg_ref high] [REA: not received]
  standaloneTx(frame);

  // MQB ESP_19 (0x0B2, DLC 8) - wheel speeds (ESP_VL/VR/HL/HR_Radgeschw_02,
  // LE, 0.0075 km/h/bit - see the real decode in OpenHaldexC6_can.cpp).
  // Wheel speed MUST keep changing or haldex slowly disengages - static
  // makes it fade over time and eventually stop. All four corners share the
  // same keep-alive dither (a front/rear slip delta was tried and reverted) -
  // see fillEsp19WheelSpeeds(). By default (wsBaseRaw = 0) the speeds
  // follow the lock (0 km/h at 0 %), same as the 0CQ set.
  // [REA audit §4: 0x0B2 in the VAQ RX table; getter not traced]
  // VAQ K-matrix: reads all four ESP_*_Radgeschw_02 (LE16, 0.0075 km/h/bit):
  // b0-1 HL, b2-3 HR, b4-5 VL, b6-7 VR. For a FRONT transverse lock the
  // relevant slip is left vs right (VL vs VR), not front vs rear - the 0CQ
  // keep-alive dither is kept as-is, a VL/VR delta is the VAQ-specific lock request.
  frame.identifier = ESP_19; // ESP_19 0x0b2
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  fillEsp19WheelSpeeds(frame.data);
  standaloneTx(frame);

  // MQB Getriebe_11 (0x0AD, DLC 8) - transmission ECU broadcast.
  // GE_MMom_Anf_02 (engine torque request), GE_MMom_Vorhalt_02 (anticipatory torque),
  // GE_Fahrstufe (selected gear), GE_Schalt_Sequenz (shift state), GE_Zielgang.
  // VAQ K-matrix: reads only GE_Zielgang (b7 bits 4-7: 0 P/N, 1-8 gear).
  frame.identifier = GETRIEBE_11; // 0x0AD
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;                // [KM Getriebe_11.Getriebe_11_CRC] [REA audit §4: no CRC check]
  frame.data[1] = GETRIEBE_11_counter; // 0x00..0x0F [KM Getriebe_11_BZ b1[0-3], GE_MMom_Soll_02 low b1[4-7]] [REA: not traced]
  frame.data[2] = 0x00;                // (was 0xFE) [KM GE_MMom_Soll_02 high b2[0-5] (raw-509 Nm), GE_MMom_Vorhalt_02 low b2[6-7]] [REA: not traced]
  frame.data[3] = 0xFE;                // [KM GE_MMom_Vorhalt_02 high (raw-509 Nm)] [REA: not traced]
  frame.data[4] = 0x00;                // [KM GE_Uefkt low, 0.1/bit] [REA: not traced]
  frame.data[5] = 0x00;                // [KM GE_Uefkt high b5[0-1], GE_Fahrstufe b5[2-5], GE_Ursache_Fahrstufe_N b5.6, GE_Schaltablauf b5.7] [REA: not traced]
  frame.data[6] = 0x00;                // [KM GE_Uefkt_unplausibel b6.1, GE_MMom_Status_02 b6[2-4], GE_Status_Kraftschluss b6[5-7]] [REA: not traced]
  frame.data[7] = 0x00;                // gear 0 [KM GE_MMom_Status b7[0-1], GE_Zielgang b7[4-7]] [REA spec §8.8: VAQ COM table has GE_Zielgang at the same b7[4-7]]

  frame.data[0] = calcChecksum(frame.data, ID_SEQ_0AD); // for 0x0AD

  GETRIEBE_11_counter++;
  if (GETRIEBE_11_counter > 0x0F)
  {
    GETRIEBE_11_counter = 0;
  }
  standaloneTx(frame);

  // MQB Motor_12 (0x0A8, DLC 8) - engine torque limits / RPM (Motor_12).
  // MO_Mom_neg_verfuegbar (max engine braking), Mom_Statisch_Limit (static limit),
  // Mom_Dynamisch_Limit (dynamic limit), MO_Drehzahl_01 (engine RPM).
  // VAQ K-matrix: reads only MO_Drehzahl_01 (b6-7 LE16, 0.25 rpm/bit).
  // VAQ bench 2026-09-17 (see ESP_14 below): rpm has NO effect on the VAQ's lock -
  // ESP_14 alone is the lock request. The "does affect" notes that were on b6/b7 here
  // were 0CQ findings copied over, and b7 was the frame counter scaled by the
  // lock (a 7170-8130 rpm saw-tooth at 100 %). V11.00.8: a steady 2000 rpm, the
  // same as the 0CQ set (spec_gen5_0cq D5) - a plausible "engine running" value.
  frame.identifier = MOTOR_12; // Motor_12 0x0A8
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;                                                    // [KM Motor_12.Motor_12_CRC] [REA audit §4: no CRC check]
  frame.data[1] = MOTOR_12_counter;                                        // 0x70..0x7F [KM Motor_12_BZ b1[0-3], MO_Mom_neg_verfuegbar low b1[4-7]] [REA: not traced]
  frame.data[2] = 0x00;                                                    // [KM MO_Mom_neg_verfuegbar high b2[0-4], MO_Mom_Begr_Fahr low b2[5-7]] [REA: not traced]
  frame.data[3] = 0x00;                                                    // [KM MO_Mom_Begr_Fahr high b3[0-5], MO_Mom_Begr_Schalt low b3[6-7]] [REA: not traced]
  frame.data[4] = 0x00;                                                    // [KM MO_Mom_Begr_Schalt high] [REA: not traced]
  frame.data[5] = 0x64;                                                    // [KM MO_Momentenintegral_02 b5[0-6]=100 %, MO_QBit_Drehzahl_01 b5.7=0] [REA: not traced]
  frame.data[6] = 0x40; // 0x1F40 = 2000 rpm [KM MO_Drehzahl_01 low, 0.25 rpm] [REA: not traced on the VAQ (0CQ: b6-b7 u16, audit S15)]
  frame.data[7] = 0x1F; // [KM MO_Drehzahl_01 high] [REA: not traced]

  frame.data[0] = calcChecksum(frame.data, ID_SEQ_0A8); // for 0x0A8

  MOTOR_12_counter++;
  if (MOTOR_12_counter > 0x7F)
  {
    MOTOR_12_counter = 0x70;
  }
  standaloneTx(frame);

  // MQB Motor_11 (0x0A7, DLC 8) - engine torque demand/output broadcast.
  //
  // Two packings, selected by the user-facing "Fix Hunting" toggle (fixHunting):
  //   fixHunting == false (default): empirical V3 packing, b6/b7 lock-modulated
  //                                  0..0xFA. Confirmed working on 554C, 554D,
  //                                  554H, and 554K @ 100% lock.
  //   fixHunting == true            : DBC-correct bit-packed Soll_Roh/Ist/Solf
  //                                  with fixed BPK defaults. Required on 554K
  //                                  at partial lock (60/40, 70/30) where the
  //                                  V3 packing causes the controller to hunt.
  //
  // The previous auto-detect logic was removed because the engagement-crossing
  // heuristic falsely flagged 554D as hunting. Mode is now strictly user-set
  // and persisted to EEPROM.
  // VAQ K-matrix: reads MO_Mom_Soll_Roh (b1[4-7]+b2[0-5]), MO_Mom_Ist_Summe
  // (b2[6-7]+b3), MO_Mom_Schub (b6[4-7]+b7[0-4]) and MO_Status_Normalbetrieb_01
  // (b7 bit 5). It does NOT list MO_Mom_Soll_gefiltert (Haldex-only). V3 packing
  // puts appliedTorque in b6/b7, so Normalbetrieb follows bit 5 of the lock
  // value (clear for 0x40-0x5F, 0x80-0x9F, 0xC0-0xDF). FIXHUNT 1 (BPK packing)
  // is DBC-correct and holds Normalbetrieb = 1 - test it early on the VAQ.
  frame.identifier = MOTOR_11; // MOTOR_11 0x0A7
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;

  if (!fixHunting && !dangerZoneActive())
  {
    // ---- V3 packing (default, working on 554C/D/H and 554K@100%) ----
    appliedTorque = getLockTargetAdjustedValue(0xFA, false);

    frame.data[0] = 0x00;             // [KM Motor_11.Motor_11_CRC] [REA audit §4: no CRC check]
    frame.data[1] = MOTOR_11_counter; // 0x40..0x4F [KM Motor_11_BZ b1[0-3], MO_Mom_Soll_Roh low b1[4-7]] [REA audit §4: Soll_Roh position holds (FUN_c2143a, buffer 0xCAD4)]
    frame.data[2] = 0xFA;             // Soll_Roh 423 Nm [KM MO_Mom_Soll_Roh high b2[0-5], MO_Mom_Ist_Summe low b2[6-7] (raw-509 Nm)] [REA audit §4: Soll/Ist positions hold]
    frame.data[3] = 0xFA;             // Ist 494 Nm [KM MO_Mom_Ist_Summe high] [REA audit §4: Ist position holds (FUN_c21456)]
    frame.data[4] = 0x00;             // [KM MO_Mom_Traegheit_Summe low] [REA: not traced]
    frame.data[5] = 0xFA;             // [KM MO_Mom_Traegheit_Summe high b5[0-1], MO_Mom_Soll_gefiltert low b5[2-7]] [REA: not traced]
    frame.data[6] = appliedTorque;    // lock-modulated, (t+20)/2 curve [KM MO_Mom_Soll_gefiltert high b6[0-3], MO_Mom_Schub low b6[4-7]] [REA audit §4/F8: one b6-b7 bit read (FUN_c25856 <- 0xCADA); bench: no effect]
    frame.data[7] = appliedTorque;    // [KM MO_Mom_Schub high b7[0-4], MO_Status_Normalbetrieb_01 b7.5 (flips with the lock), b7.6, MO_QBit_Motormomente b7.7] [REA audit F8: hold constant like the 0CQ set (spec D3)]
  }
  else
  {
    // ---- BPK packing (Fix Hunting toggle on; needed for 554K @ partial lock) ----
    // Shared with the normal-mode packer so the two can never drift, and so the
    // serial lab tunables apply identically in both. See fillMotor11Bpk().
    // (appliedTorque is set by the ESP_14 frame below.)
    fillMotor11Bpk(frame.data, MOTOR_11_counter);
  }

  frame.data[0] = calcChecksum(frame.data, ID_SEQ_0A7); // for 0x0A7

  MOTOR_11_counter++;
  if (MOTOR_11_counter > 0x4F)
  {
    MOTOR_11_counter = 0x40;
  }

  standaloneTx(frame);
  /*
  0xd2,0x3d,0xcd,0x28,0x4c,0x14,0x22,0x4b,0x24,0xac,0xfa,0x55,0x66,0x80,0x0d,0x6c
  */

  // MQB ESP_14 (0x08A, DLC 8) - ESP-to-AWD coupling-range limits.
  // BR_Vorg_Quer_Min/Max (lateral stability range), BR_Vorg_Allrad_Min/Max (AWD range).
  // Old 0CQ note: 100 % torque = 2000 Nm, data[7] has a massive effect on engagement.
  // VAQ K-matrix: reads BR_Status_Quer_ESP (b3 bits 3-5: 0 cross lock
  // deactivated, 1 ESP error, 2 comms disturbed, 3 ESP ready, 4 ESP requests
  // cross lock, 5 AYC support), BR_Vorg_Quer_Min (b4) and BR_Vorg_Quer_Max (b5),
  // 0.4 %/bit, 252 = quick close/open, 254 init, 255 error. The Allrad bytes
  // b6/b7 are NOT read by the VAQ. 0CQ sends b3 = 0x00 = 'cross lock
  // deactivated' - PRIME SUSPECT if the VAQ never engages: try b3 = 0x18
  // (ready) or 0x20 (ESP requests cross lock) with OVR 08A 3 0x18 / 0x20.
  //
  // BENCH RESULT 2026-09-17 (real 0CQ907554E VAQ): this frame IS the lock request.
  //   - BR_Status_Quer_ESP must be >= 3 (0x18 ready / 0x20 request / 0x28 AYC);
  //     0 "deactivated" -> always 0 %, whatever Min/Max say.
  //   - QUER_Ist_Proz follows BR_Vorg_Quer_Min 1:1 (0.4 %/bit): 0x0A -> 4 %,
  //     0x32 -> 20 %, 0x7D -> 50 %, 0xC8/0xE1/0xFA -> 80 % (cap at standstill).
  //   - Min > Max is rejected (0 %, pump off). Min 0 with Max > 0 -> ~11 %
  //     rule-mode preload. Min = Max = 0 -> 0 %, pump idles at 15 % PWM.
  //   - Motor_11 packing (FIXHUNT), Charisma, rpm, wheel-speed splits: no effect.
  //   Both bytes have to change together, which is why single-byte sweeps
  //   never found it. The torque-ceiling scaling of the 0CQ path is wrong
  //   here: the VAQ wants plain percent.
  frame.identifier = ESP_14; // ESP_14 0x08A
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;           // [KM ESP_14.ESP_14_CRC] [REA audit §4: no CRC check]
  frame.data[1] = ESP_14_counter; // 0x10..0x1F [KM ESP_14_BZ b1[0-3], ESP_14_Status_DSR b1[4-7]=1] [REA audit §4: not read (only b3-b5 decoded)]
  frame.data[2] = 0x00;           // [KM ESP_14_Anf_Lenkmomoffset_DSR low, 0.01 Nm] [REA audit §4: not read]

  // Cross-lock request in plain percent: 0..250 = 0..100 % (0.4 %/bit).
  // OEM code (spec_gen5_0cq §8.8, D20): the VAQ reads b3 bits 3-5, b4 and b5 -
  // this layout matches. V11 TODO (bench): the bench says the VAQ follows Quer_Min
  // "1:1 in plain percent", but this still goes through the old 0CQ correction
  // curve. Try quer = gen5Esp14LockRequestByte() (linear 0..250) on the bench.
  // The 0.4 %/bit and 250 = 100 % scale is K-matrix + bench only: the VAQ's own
  // scale lives in the calibration block missing from the dump (audit §4).
  const uint8_t quer = getLockTargetAdjustedValue(250, false);
  frame.data[3] = quer ? 0x20 : 0x00; // 4 / 0 [KM BR_Status_Quer_ESP b3[3-5]: 0 deaktiviert, 3 ESP_bereit, 4 ESP_Anf_Quersperre, 5 AYC] [REA audit §4/F9: FUN_c24ab6; status 0 or 2 -> open branch]
  frame.data[4] = quer;               // [KM BR_Vorg_Quer_Min 0.4 %, 252 schnell_zu, 254 Init, 255 Fehler] [REA audit §4: Min (FUN_c257ee), same 5-branch decode as the 0CQ (FUN_c171da); scale not in dump]
  frame.data[5] = quer;               // Max = Min = exact [KM BR_Vorg_Quer_Max 0.4 %, 252 schnell_offen] [REA audit §4/F9: Max (FUN_c25108); Min = Max -> exact; 0xFC presumably OPEN as on the 0CQ]

  // Allrad bytes (b6/b7) are not stored by the VAQ (OEM code: its ESP_14 buffer
  // is only 6 bytes long, spec §8.8). V11.00.0: same packing as the 0CQ set
  // (Min = Max = V, 0..250) so the two stay comparable.
  appliedTorque = gen5Esp14LockRequestByte();
  frame.data[6] = appliedTorque; // [KM BR_Vorg_Allrad_Min 0.4 %] [REA audit §4: not stored (6-byte buffer, len table 0xC16350)]
  frame.data[7] = appliedTorque; // [KM BR_Vorg_Allrad_Max 0.4 %] [REA audit §4: not stored]

  frame.data[0] = calcChecksum(frame.data, ID_SEQ_08A); // for 0x08A

  ESP_14_counter++;
  if (ESP_14_counter > 0x1F)
  {
    ESP_14_counter = 0x10;
  }
  standaloneTx(frame);

  /*
  0xd4,0xd4,0xd4,0xd4,0xd4,0xd4,0xd4,0xd4,0xd4,0xd4,0xd4,0xd4,0xd4,0xd4
  */

  // MQB LWI_01 (0x086, DLC 8) - steering-angle sensor (Lenkwinkelinformation).
  // LWI_Lenkradwinkel (steering wheel angle * 0.1 deg), LWI_Lenkradw_Geschw (rate * 5 deg/s),
  // LWI_Sensorstatus, plus quality bits.
  // VAQ K-matrix: reads BZ, LWI_Sensorstatus (b1 bit 4), QBit (b1 bit 7),
  // LWI_Lenkradwinkel (b2 + b3[0-4], 0.1 deg/bit) and VZ (b3 bit 5). The 0CQ
  // b2 = 0x01 is really Lenkradwinkel low byte = 0.1 deg, harmless.
  // Bench 2026-09-17: the VAQ logged the LWI_01 "missing" DTC (0x00410D) with
  // the 0CQ packing on. Cause: the 0CQ counter runs 0x10..0x1F, which keeps
  // b1 bit 4 = LWI_Sensorstatus = 1 "not calibrated" (K-matrix VAL_ 134). The
  // Haldex never checked it; the VAQ does. Low nibble only -> status 0 "OK".
  frame.identifier = LWI_01; // LWI_01 0x086
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;                  // [KM LWI_01.LWI_01_CRC] [REA audit §4: no CRC check]
  frame.data[1] = LWI_01_counter & 0x0F; // [KM LWI_01_BZ b1[0-3], LWI_Sensorstatus b1.4=0 iO, LWI_QBit_Lenkradwinkel b1.7=0] [REA: getter not traced; bench DTC 0x00410D cured by b1.4 = 0]
  frame.data[2] = 0x01;                  // 0.1 deg [KM LWI_Lenkradwinkel low, 0.1 deg] [REA: not traced]
  frame.data[3] = 0x00;                  // [KM LWI_Lenkradwinkel high b3[0-4], LWI_VZ_Lenkradwinkel b3.5, VZ_Lenkradw_Geschw b3.6, Lenkradw_Geschw low b3.7] [REA: not traced]
  frame.data[4] = 0x00;                  // [KM LWI_Lenkradw_Geschw high, 5 deg/s] [REA: not traced]
  frame.data[5] = 0x00;                  // [KM LWI_Sub_Daten low] [REA: not traced]
  frame.data[6] = 0x00;                  // [KM LWI_Sub_Daten high] [REA: not traced]
  frame.data[7] = 0x00;                  // [KM -] [REA: not traced]

  frame.data[0] = calcChecksum(frame.data, ID_SEQ_086); // for 0x086

  LWI_01_counter++;
  if (LWI_01_counter > 0x1F)
  {
    LWI_01_counter = 0x10;
  }

  standaloneTx(frame);
  /*
  0x86,0x86,0x86,0x86,0x86,0x86,0x86,0x86,0x86,0x86,0x86,0x86,0x86,0x86
  */
}

void Gen5_0CQ_VAQ_frames20()
{
  twai_message_t frame = {};
  // MQB Motor_20 (0x121, DLC 8) - accelerator pedal raw/filtered + status.
  // MO_Accelerator_Raw_Value_01 (raw pedal), MO_Fahrpedal_Roh, MO_Pedal_Filt.
  // VAQ K-matrix: reads MO_Fahrpedalrohwert_01 (b1[4-7]+b2[0-3], 0.4 %/bit)
  // and MO_QBit_Fahrpedalwerte_01 (b2 bit 4). Counter 0x0X + b2 0x40 -> pedal
  // raw 0 %, QBit 0 (valid).
  // Bench 2026-09-17: the VAQ logged the Motor_20 "missing" DTC (0x004157,
  // plus 0x004457) with the 0CQ packing on. Bytes 2..5 now match a real MQB
  // idle capture (b2 0x00, b3 0xC0, b4 0x39, b5 0x50) instead of the 0CQ
  // guesses (0x40 0x40 0x19 0x59); the 0CQ never checked them, the VAQ does.
  frame.identifier = MOTOR_20;      // MOTOR_20 0x121
  frame.data_length_code = 8;       // DLC 8
  frame.data[0] = 0x00;             // [KM Motor_20.Motor_20_CRC] [REA audit §4: no CRC check]
  frame.data[1] = MOTOR_20_counter; // [KM Motor_20_BZ b1[0-3], MO_Fahrpedalrohwert_01 low b1[4-7]=0] [REA: not traced]
  frame.data[2] = 0x00;             // [KM MO_Fahrpedalrohwert_01 high b2[0-3] (0.4 %, = 0 %), MO_QBit_Fahrpedalwerte_01 b2.4=0, MO_Fahrpedalgradient low b2[5-7]] [REA: not traced]
  frame.data[3] = 0xC0;             // real-car idle [KM MO_Fahrpedalgradient high b3[0-4], MO_Sig_Fahrpedalgradient b3.5, MO_rel_Saugrohrdruck low b3[6-7]] [REA: not traced (0CQ reads bits 28-35 here, spec §3)]
  frame.data[4] = 0x39;             // real-car idle [KM MO_rel_Saugrohrdruck high b4[0-3], _gem_err b4.4, MO_Moment_im_Leerlauf b4.5, ...] [REA: not traced]
  frame.data[5] = 0x50;             // real-car idle [KM MO_Solldrehz_Leerlauf 10 rpm (800 rpm)] [REA: not traced]
  frame.data[6] = 0x7E;             // real-car idle [KM MO_Entkopplung_Sollschlupf b6[0-6], MO_temporaere_Fahrerabwesenheit b6.7] [REA: not traced]
  frame.data[7] = 0xFE;             // real-car idle [KM TSK_a_Soll_gradientenbegrenzt b7[1-7]] [REA: not traced]

  frame.data[0] = calcChecksum(frame.data, ID_SEQ_121); // for 0x121

  MOTOR_20_counter++;
  if (MOTOR_20_counter > 0x0F)
  {
    MOTOR_20_counter = 0x00;
  }

  standaloneTx(frame);
  /*
  0xe9,0x65,0xae,0x6b,0x7b,0x35,0xe5,0x5f,0x4e,0xc7,0x86,0xa2,0xbb,0xdd,0xeb,0xb4
  */

  // MQB ESP_10 (0x116, DLC 8) - lateral dynamics / yaw + lateral accel.
  // ESP_Gierrate (yaw), ESP_Querbeschleunigung (lateral acceleration), status bits.
  // VAQ K-matrix: NOT in the VAQ receive list (only the Haldex reads
  // ESP_*_Fahrtrichtung from it). Kept like-for-like; candidate to drop.
  frame.identifier = ESP_10;                            // ESP_10 0x116
  frame.data_length_code = 8;                           // DLC 8
  frame.data[0] = 0x00;                                 // [KM ESP_10.ESP_10_CRC] [REA audit §4: 0x116 not in the VAQ RX table]
  frame.data[1] = ESP_10_counter;                       // [KM ESP_10_BZ b1[0-3], ESP_QBit_Wegimpuls_VL/VR/HL/HR b1[4-7]] [REA: not received]
  frame.data[2] = 0x01;                                 // [KM ESP_Wegimpuls_VL low] [REA: not received]
  frame.data[3] = 0x04;                                 // [KM ESP_Wegimpuls_VL high b3[0-1], _VR low b3[2-7]] [REA: not received]
  frame.data[4] = 0x00;                                 // [KM ESP_Wegimpuls_VR high b4[0-3], _HL low b4[4-7]] [REA: not received]
  frame.data[5] = 0x40;                                 // [KM ESP_Wegimpuls_HL high b5[0-5], _HR low b5[6-7]] [REA: not received]
  frame.data[6] = 0x00;                                 // [KM ESP_Wegimpuls_HR high] [REA: not received]
  frame.data[7] = 0xFF;                                 // old "affects 40 %" note was a 0CQ copy, impossible [KM ESP_*_Fahrtrichtung 2 bits each] [REA: not received]
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_116); // for 0x116

  ESP_10_counter++;
  if (ESP_10_counter > 0x0F)
  {
    ESP_10_counter = 0x00;
  }

  standaloneTx(frame);
  /*
  0xac,0xac,0xac,0xac,0xac,0xac,0xac,0xac,0xac,0xac,0xac,0xac,0xac,0xac,0xac
  */

  // MQB ESP_05 (0x106, DLC 8) - brake pressure + brake-light/brake-pedal flags.
  // BR_Bremsdruck (master cylinder pressure), BR_Bremslicht (brake-light), BR_Bremspedal (pedal active).
  // VAQ K-matrix: reads BZ, ESP_QBit_Fahrer_bremst (b1 bit 5), ESP_Fahrer_bremst
  // (b3 bit 2), ESP_Verz_TSK_aktiv (b3 bit 3), ESP_Verz_EPB_aktiv (b7 bit 2).
  // The 0CQ 'b3 affects' finding is ESP_Fahrer_bremst (brake threshold
  // exceeded); 0xC0 leaves it clear.
  // V11.00.0: b3 bit 2 follows brakeForHaldex() ("Follow Brake"); off = 0 as before.
  frame.identifier = ESP_05;                            // ESP_05 0x106
  frame.data_length_code = 8;                           // DLC 8
  frame.data[0] = 0x00;                                 // [KM ESP_05.ESP_05_CRC] [REA audit §4: no CRC check]
  frame.data[1] = ESP_05_counter;                       // 0x80..0x8F [KM ESP_05_BZ b1[0-3], ESP_QBit_Bremsdruck b1.4, ESP_QBit_Fahrer_bremst b1.5, ESP_Schwelle_Unterdruck b1[6-7]=2] [REA: not traced]
  frame.data[2] = 0x64;                                 // 0 bar [KM ESP_Bremsdruck low, 0.3 bar -30] [REA: not traced (0CQ reads bits 16-25)]
  frame.data[3] = 0xC0;                                 // [KM ESP_Bremsdruck high b3[0-1], ESP_Fahrer_bremst b3.2=0, ESP_Verz_TSK_aktiv b3.3, AWV2 flags b3[6-7]] [REA: not traced]
  if (brakeForHaldex())
  {
    frame.data[3] = frame.data[3] | (1 << 2); // [KM ESP_05.ESP_Fahrer_bremst] [REA: not traced]
  }
  frame.data[4] = 0x00;                                 // [KM ECD/HDC/Autohold/HBA/Prefill flags] [REA: not traced]
  frame.data[5] = 0x00;                                 // [KM ESP_Status_Anfahrhilfe, HDC_aktiv, StartStopp_Info, ESP_Eingr_* flags] [REA: not traced]
  frame.data[6] = 0xFD;                                 // [KM ESP_BKV_Unterdruck 4 mbar] [REA: not traced]
  frame.data[7] = 0x00;                                 // old "affects on/off" (was 0x10 = b7.4 ESP_Verzoeg_EPB_verf) [KM Autohold_aktiv b7.0, ESP_Verz_EPB_aktiv b7.2, ECD_Bremslicht b7.3] [REA: not traced]
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_106); // for 0x106

  ESP_05_counter++;
  if (ESP_05_counter > 0x8F)
  {
    ESP_05_counter = 0x80;
  }

  standaloneTx(frame);
  /*
  0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07
  */

  // MQB EPB_01 (0x104, DLC 8) - electronic parking brake state.
  // VAQ K-matrix: NOT in the VAQ receive list. Kept like-for-like.
  frame.identifier = EPB_01;                            // EPB_01 0x104
  frame.data_length_code = 8;                           // DLC 8
  frame.data[0] = 0x00;                                 // [KM EPB_01.EPB_01_CRC] [REA audit §4: 0x104 not in the VAQ RX table]
  frame.data[1] = EPB_01_counter;                       // 0x30..0x3F [KM EPB_01_BZ b1[0-3], EPB_QBit_* b1[4-5], EPB_Freig_Verzoeg_Anf b1.7] [REA: not received]
  frame.data[2] = 0xA6;                                 // [KM EPB_Verzoeg_Anf 0.048-7.968 m/s2] [REA: not received]
  frame.data[3] = 0x00;                                 // [KM -] [REA: not received]
  frame.data[4] = 0xE6;                                 // [KM EPB_Pedalweg_Kuppl 0.4 %] [REA: not received]
  frame.data[5] = 0x00;                                 // [KM -] [REA: not received]
  frame.data[6] = 0x00;                                 // [KM EPB_DAA_Randbed_erf, EPB_Fehlerstatus, EPB_Schalterposition, QBit] [REA: not received]
  frame.data[7] = 0x31;                                 // [KM EPB_Spannkraft b7[0-4] kN, EPB_Status b7[5-6]] [REA: not received]
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_104); // for 0x104

  EPB_01_counter++;
  if (EPB_01_counter > 0x3F)
  {
    EPB_01_counter = 0x30;
  }

  standaloneTx(frame);
  /*
  0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05
  */

  // MQB ESP_02 (0x101, DLC 8) - ESP brake/yaw status broadcast.
  // VAQ K-matrix: reads BZ, the three QBits (b1 bits 4-6), Querbeschleunigung
  // (b2, 0.01 g - 1.27), Laengsbeschl (b3 + b4[0-1], 0.03125 m/s2 - 16),
  // Gierrate (b5 + b6[0-5], 0.01 deg/s) and VZ_Gierrate (b6 bit 6). 0CQ values
  // decode to -0.01 g lateral, +0.47 m/s2 longitudinal, 0.12 deg/s yaw.
  // Bench 2026-09-17: the VAQ logged the ESP_02 "missing" DTC (0x00410F) with
  // the 0CQ packing on. Cause: the 0CQ counter runs 0x00..0x1F, so for half
  // of every cycle b1 bit 4 = 1 - per the K-matrix b1.4 is ESP_QBit_Gierrate
  // (b1.5 QBit_Laengsbeschl, b1.6 QBit_Querb), "replacement/error value"
  // (K-matrix VAL_ 257). Low nibble only -> all three QBits (b1[4-6]) = 0 valid.
  frame.identifier = ESP_02;                            // ESP_02 0x101
  frame.data_length_code = 8;                           // DLC 8
  frame.data[0] = 0x00;                                 // [KM ESP_02.ESP_02_CRC] [REA audit §4: no CRC check]
  frame.data[1] = ESP_02_counter & 0x0F;                // [KM ESP_02_BZ b1[0-3], ESP_QBit_Gierrate b1.4, QBit_Laengsbeschl b1.5, QBit_Querb b1.6 = 0] [REA: not traced; bench DTC 0x00410F cured by QBits = 0]
  frame.data[2] = 0x7E;                                 // -0.01 g [KM ESP_Querbeschleunigung 0.01 g -1.27] [REA: not traced]
  frame.data[3] = 0x0F;                                 // [KM ESP_Laengsbeschl low, 0.03125 m/s2 -16] [REA: not traced]
  frame.data[4] = 0x82;                                 // +0.47 m/s2 [KM ESP_Laengsbeschl high b4[0-1], ESC flags b4[2-6], ESP_QBit_Anf_Vert_Wank b4.7=1] [REA: not traced]
  frame.data[5] = 0x0C;                                 // 0.12 deg/s [KM ESP_Gierrate low, 0.01 deg/s] [REA: not traced]
  frame.data[6] = 0x40;                                 // [KM ESP_Gierrate high b6[0-5], ESP_VZ_Gierrate b6.6=1 negativ, ESP_Notbremsanzeige b6.7] [REA: not traced]
  frame.data[7] = 0x00;                                 // [KM ESP_SpannungsAnf b7.0, ESP_PLA_Abbruch b7[1-3], ESP_Status_ESP_PLA b7[4-7]] [REA: not traced]
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_101); // for 0x101

  ESP_02_counter++;
  if (ESP_02_counter > 0x1F)
  {
    ESP_02_counter = 0x00;
  }

  standaloneTx(frame);
  /*
  0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa
  */

  // MQB ESP_21 (0x0FD, DLC 8) - ESP wheel-speed/dynamics diagnostics.
  // VAQ K-matrix: reads BZ and b7 bits 0 ABS_Bremsung, 1 ASR_Anf, 4
  // EDS_Eingriff, 5 ESP_Eingriff. 0CQ b7 = 0x00 -> no interventions.
  // V11.00.0: ABS_Bremsung (b7 bit 0) carries absBrakingForHaldex() (the car's ABS).
  frame.identifier = ESP_21;                            // ESP_21 0x0fd
  frame.data_length_code = 8;                           // DLC 8
  frame.data[0] = 0x00;                                 // [KM ESP_21.ESP_21_CRC] [REA audit §4: no CRC check]
  frame.data[1] = ESP_21_counter;                       // 0x00..0x1F [KM ESP_21_BZ b1[0-3], BR_Eingriffsmoment low b1[4-7]] [REA: not traced]
  frame.data[2] = 0x1F;                                 // [KM BR_Eingriffsmoment high b2[0-5], ESP_PLA_Bremseingriff b2.6, ESP_Diagnose b2.7] [REA: not traced]
  frame.data[3] = 0x80;                                 // [KM ESC_Reku_Freigabe, v_Signal_Qualifier, ..., OBD_QBit_Schlechtweg b3.7=1] [REA: not traced]
  frame.data[4] = 0x00;                                 // [KM ESP_v_Signal low, 0.01 km/h] [REA: not traced]
  frame.data[5] = 0x00;                                 // [KM ESP_v_Signal high] [REA: not traced]
  frame.data[6] = 0x00;                                 // [KM ASR/ESP_Tastung_passiv, ESP_Systemstatus, ..., ESP_QBit_v_Signal b6.7] [REA: not traced]
  frame.data[7] = 0x00;                                 // no intervention [KM ABS_Bremsung b7.0, ASR_Anf b7.1, EDS_Eingriff b7.4, ESP_Eingriff b7.5] [REA: not traced]
  if (absBrakingForHaldex())
  {
    frame.data[7] = frame.data[7] | (1 << 0); // [KM ESP_21.ABS_Bremsung] [REA: not traced]
  }
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_0fd); // for 0x0fd (over the finished bytes)

  ESP_21_counter++;
  if (ESP_21_counter > 0x1F)
  {
    ESP_21_counter = 0x00;
  }

  standaloneTx(frame);

  /*
  0xb4,0xef,0xf8,0x49,0x1e,0xe5,0xc2,0xc0,0x97,0x19,0x3c,0xc9,0xf1,0x98,0xd6,0x61
  */
}

void Gen5_0CQ_VAQ_frames25()
{
  twai_message_t frame = {};
  // MQB Kombi_01 (0x30B, DLC 8) - instrument cluster broadcast.
  // KBI_Kilometerstand (odometer), KBI_Geschw_Anzeige (displayed speed), warning lamps.
  // VAQ K-matrix: reads only KBI_Handbremse (b2 bit 7). 0CQ b2 = 0x02 -> off.
  // Handbrake-follow rewrites this bit for gen 52 too (see can.cpp).
  // Bench 2026-09-17: the VAQ logged the Kombi_01 "missing" DTC (0x00410E)
  // with the static 0CQ frame on. K-matrix: Kombi_01_BZ is b1[0-3], a rolling
  // counter the 0CQ never needed advanced. Roll it for the VAQ.
  // REA: audit §4 found no counter check in the VAQ's RX copy routine, so what
  // the static BZ tripped (a timeout/plausibility monitor?) is not traced.
  static uint8_t kombi01Bz = 0;
  frame.identifier = KOMBI_01;                          // Kombi_01 0x30B
  frame.data_length_code = 8;                           // DLC 8
  frame.data[0] = 0x10;                                 // [KM KBI_*_Lampe b0[0-3]=0, KBI_SILA_gueltig b0.4=1] [REA: not traced]
  frame.data[1] = (uint8_t)(0x20 | (kombi01Bz & 0x0F)); // rolling [KM Kombi_01_BZ b1[0-3], KBI_Anzeigestatus_GRA b1[5-6]=1] [REA: not traced; bench DTC 0x00410E cured by rolling it]
  kombi01Bz++;
  frame.data[2] = 0x02; // [KM KBI_MFA_v_Einheit_01 b2.1=1, KBI_Handbremse b2.7=0] [REA: not traced]
  if (handbrakeForHaldex())
  {
    frame.data[2] = frame.data[2] | (1 << 7); // [KM Kombi_01.KBI_Handbremse] [REA: not traced]
  }
  frame.data[3] = 0x00; // (not odometer) [KM KBI_V_Digital low] [REA: not traced]
  frame.data[4] = 0x0C; // [KM KBI_V_Digital high b4.0, PLA/NV/LIM display flags] [REA: not traced]
  frame.data[5] = 0x00; // [KM KBI_Audioabsenkung_MIB b5[0-1]] [REA: not traced]
  frame.data[6] = 0x00; // [KM KBI_angez_Geschw low, 0.32 km/h] [REA: not traced]
  frame.data[7] = 0x24; // [KM KBI_angez_Geschw high b7[0-1], KBI_Einheit_Tacho b7.2, ACC/SWA flags] [REA: not traced]
  standaloneTx(frame);
}

void Gen5_0CQ_VAQ_frames100()
{
  twai_message_t frame = {};
  // MQB ESP_23 (0x5BE, DLC 8) - longitudinal/lateral acceleration, tyre data.
  // BR_Laengsbeschleunigung (longitudinal G), BR_Querbeschleunigung (lateral G), BR_Tire_Circumference.
  // VAQ K-matrix: NOT in the VAQ receive list. Kept like-for-like.
  frame.identifier = ESP_23;                            // ESP_23 0x5be - this is fixed in Savvy but CHKS in Kmatrix?
  frame.data_length_code = 8;                           // DLC 8
  frame.data[0] = 0x00;                                 // [KM ESP_23.ESP_23_CRC] [REA audit R6: 0x5BE not in the 0CQ or VAQ RX table]
  frame.data[1] = ESP_23_counter;                       // [KM ESP_23_BZ b1[0-3], ESP_Gierrate_Offset_2 low b1[4-7]] [REA: not received]
  frame.data[2] = 0xBF;                                 // [KM ESP_Gierrate_Offset_2 high b2[0-6] (0.01-10.23 deg/s), ESP_Lenkwinkel_Offset low b2.7] [REA: not received]
  frame.data[3] = 0x7F;                                 // [KM ESP_Lenkwinkel_Offset high (0.1-25.5 deg)] [REA: not received]
  frame.data[4] = 0x00;                                 // [KM BR_Reifenumfang_sekundaer low, mm] [REA: not received]
  frame.data[5] = 0x00;                                 // [KM BR_Reifenumfang_sekundaer high b5[0-3], BR_QBit_Reifenumfang_sekundaer b5.4] [REA: not received]
  frame.data[6] = 0x7C;                                 // [KM ESP_Querbeschleunigung_Offset b6[2-7] (0.01-0.3 g)] [REA: not received]
  frame.data[7] = 0x78;                                 // [KM ESP_Laengsbeschleunigung_Offset (0.03125-3.9375 m/s2)] [REA: not received]
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_5be); // for 0x5be

  ESP_23_counter++;
  if (ESP_23_counter > 0x1F)
  {
    ESP_23_counter = 0x00;
  }

  standaloneTx(frame);
  /*
    0xc9,0x21,0x6f,0x63,0xd2,0x42,0x6a,0x77,0x4a,0x3d,0xb0,0x62,0x9f,0x38,0xcd,0x5c
    */

  // MQB Parkhilfe_04 (0x54B, DLC 8) - park-assist module state broadcast.
  // VAQ K-matrix: NOT in the VAQ receive list. Kept like-for-like.
  frame.identifier = Parkhilfe_04; // Parkhilfe_04 0x54B
  frame.data_length_code = 8;      // DLC 8
  frame.data[0] = 0x00;            // [KM -] [REA audit R6: 0x54B not in the 0CQ or VAQ RX table]
  frame.data[1] = 0x00;            // [KM PH_Verschmutzungsmeldung b1[4-7]] [REA: not received]
  frame.data[2] = 0x00;            // [KM PH_Aufschaltursache b2[0-4], PH_Ton_Ausgabe b2[5-7]] [REA: not received]
  frame.data[3] = 0x00;            // [KM PH_Ton_Pausenlaenge] [REA: not received]
  frame.data[4] = 0x00;            // [KM -] [REA: not received]
  frame.data[5] = 0x00;            // [KM -] [REA: not received]
  frame.data[6] = 0x00;            // [KM PH_Ton_Richtung b6[1-3], PH_Ton_Lautstaerke b6[4-6], BCM_WAH_Meldung low b6.7] [REA: not received]
  frame.data[7] = 0x24;            // [KM BCM_WAH_Meldung high b7[0-1], PDC_Charisma_Status b7[2-3], PDC_Charisma_FahrPr b7[4-7]] [REA: not received]
  standaloneTx(frame);

  // MQB Gateway_72 (0x3DB, DLC 8) - gateway routing/diagnostic broadcast.
  // VAQ K-matrix: NOT in the VAQ receive list. Kept like-for-like.
  frame.identifier = GATEWAY_72; // Gateway_72 0x3DB
  frame.data_length_code = 8;    // DLC 8
  frame.data[0] = 0x50;          // [KM Gateway_72 *_alt timeout flags (BCM_01/SMLS_01/ZV_02/Wischer_01/...)] [REA audit R6: 0x3DB not in the 0CQ or VAQ RX table]
  frame.data[1] = 0x80;          // [KM WFS/Licht/ZV flags, BCM1_LDS_Stellung low b1.7] [REA: not received]
  frame.data[2] = 0x00;          // [KM BCM1_LDS_Stellung high, Verdeck/door flags] [REA: not received]
  frame.data[3] = 0x00;          // [KM door/wiper flags, AAG_Anhaenger_erkannt b3.4] [REA: not received]
  frame.data[4] = 0x05;          // [KM KL_Thermomanagement b4[0-1], WFS_Schluessel_Fahrberecht b4[2-5], ...] [REA: not received]
  frame.data[5] = 0x10;          // [KM ZV_verriegelt_soll b5[2-3], ZV_Schluessel_Zugang b5[4-7], ...] [REA: not received]
  frame.data[6] = 0x01;          // [KM WFS_ID_Geb_autorisiert b6.0, BH_Fernlicht/Blinker b6[1-3], BCM1_OBD_FStatus_ATemp b6[4-7]] [REA: not received]
  frame.data[7] = 0x78;          // [KM BCM1_Aussen_Temp_ungef 0.5-50 degC (10 degC)] [REA: not received]
  standaloneTx(frame);

  // MQB Getriebe_14 (0x3C8, DLC 8) - transmission slow broadcast (Charisma, drag torque, launch).
  // VAQ K-matrix: NOT in the VAQ receive list. Kept like-for-like.
  frame.identifier = GETRIEBE_14; // Getriebe_14 0x3C8
  frame.data_length_code = 8;     // DLC 8
  frame.data[0] = 0x00;           // [KM -] [REA audit R6: 0x3C8 not in the 0CQ or VAQ RX table]
  frame.data[1] = 0x00;           // [KM GE_OBD_AbsperrVent b1[4-7]] [REA: not received]
  frame.data[2] = 0x54;           // [KM GE_amax_moeglich low, 0.024-2.016 m/s2] [REA: not received]
  frame.data[3] = 0x24;           // [KM GE_amax_moeglich high b3.0, GE_Charisma_FahrPr b3[1-4], GE_Charisma_Status b3[5-6], WFS b3.7] [REA: not received]
  frame.data[4] = 0x00;           // [KM GE_Verlustmoment, Nm] [REA: not received]
  frame.data[5] = 0x60;           // [KM Freilauf_Charisma_FahrPr b5[0-3], Freilauf_Charisma_Status b5[4-5], WFS b5[6-7]] [REA: not received]
  frame.data[6] = 0x01;           // [KM GE_Zustand_Kuppltemp b6[0-1], GE_LaunchControl b6.3, GE_Heizwunsch b6[4-5], ...] [REA: not received]
  frame.data[7] = 0x51;           // [KM GE_Sumpftemperatur -58 degC (23 degC)] [REA: not received]
  standaloneTx(frame);

  // MQB Motor_14 (0x3BE, DLC 8) - start/stop subsystem state broadcast.
  // MO_StSt_Status (state machine), MO_StSt_Restart (restart event), MO_StSt_Stop (stop event).
  // VAQ K-matrix: reads MO_Gangposition (b2 bits 4-7: 0 N, 1-8 gear, 9 P,
  // 10 S, 11 D/E, 13 R, 14 undefined), MO_Fahrer_bremst (b3 bit 4), its QBit
  // (b3 bit 5) and MO_Kuppl_schalter (b4 bit 5). 0CQ b2 = 0xE6 -> gear 14
  // 'not defined'; try 0x26 (gear 2) if the VAQ wants a real gear.
  frame.identifier = MOTOR_14;                          // Motor_14 0x3BE
  frame.data_length_code = 8;                           // DLC 8
  frame.data[0] = 0x00;                                 // [KM Motor_14.Motor_14_CRC] [REA 0CQ audit S6: no CRC check]
  frame.data[1] = MOTOR_14_counter;                     // 0x10..0x1F [KM Motor_14_BZ b1[0-3], MO_StartStopp_Status b1[4-5]=1, _Wiederstart b1.6, _Motorstopp b1.7] [REA spec §3: not read]
  frame.data[2] = 0xE6;                                 // [KM MO_Freig_Reku b2[0-1], MO_Kl_75 b2.2, MO_Kl_50 b2.3, MO_Gangposition b2[4-7]=14 'undefined'] [REA spec §3: b2[4-7] = gear, used only if Getriebe_11 is missing (0xD->0xFF, >8->0); VAQ getter not traced]
  frame.data[3] = 0x01;                                 // old "affects on/off" [KM MO_StartStopp_Fahrerwunsch b3[0-1]=1, MO_Fahrer_bremst b3.4, MO_QBit_Fahrer_bremst b3.5, MO_BLS b3.6] [REA spec §3: b3 bits 4/5 COM only, not read by the 0CQ]
  frame.data[4] = 0xC8;                                 // [KM MO_Timeout_ESP b4.0, MO_Aussp_Anlass b4.3, MO_Kuppl_schalter b4.5=0, MO_Interlock b4.6, MO_Motor_laeuft b4.7] [REA spec §3: b4.5 -> 0xCFF3 = (bit != 1)]
  frame.data[5] = 0x80;                                 // [KM MO_Kickdown b5.0, ..., MO_PTC_Status low b5.7] [REA spec §3: not read]
  frame.data[6] = 0x00;                                 // [KM MO_PTC_Status high, MO_QBit_Gangposition b6.2, MO_Signalquelle_Gangposition b6.3, ...] [REA spec §3: not read]
  frame.data[7] = 0x80;                                 // [KM MO_FMAus_Startvariante, MO_Zylinderabschaltung, MO_StartVorauss_erfuellt b7[6-7]] [REA spec §3: not read]
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_3be); // for 0x3be

  MOTOR_14_counter++;
  if (MOTOR_14_counter > 0x1F)
  {
    MOTOR_14_counter = 0x10;
  }

  standaloneTx(frame);
  /*
  0x1f,0x28,0xc6,0x85,0xe6,0xf8,0xb0,0x19,0x5b,0x64,0x35,0x21,0xe4,0xf7,0x9c,0x24
  */

  // MQB ESP_07 (0x392, DLC 8) - ESP brake-event/intervention diagnostics.
  // VAQ K-matrix: reads only ESP_Rollenmodus_Deaktivieren (b7 bit 6).
  frame.identifier = ESP_07;                            // ESP_07 0x392
  frame.data_length_code = 8;                           // DLC 8
  frame.data[0] = 0x00;                                 // [KM ESP_07.ESP_07_CRC] [REA 0CQ audit S6: no CRC check]
  frame.data[1] = ESP_07_counter;                       // 0x20..0x2F [KM ESP_07_BZ b1[0-3], ESP_ACC_LDE b1.4, ESP_Allrad_Antrieb b1.5=0, ESP_Codierung_ADS b1[6-7]] [REA spec §3: not read]
  frame.data[2] = 0x00;                                 // [KM ESP_RTA_HL 0.0488 % -6.2] [REA spec §3: not read]
  frame.data[3] = 0x00;                                 // [KM ESP_RTA_HR] [REA spec §3: not read]
  frame.data[4] = 0x00;                                 // [KM ESP_RTA_VR] [REA spec §3: not read]
  frame.data[5] = 0x00;                                 // [KM OBD_Fehler_Radsensor_HL b5[0-3], _HR b5[4-7]] [REA spec §3: not read]
  frame.data[6] = 0x00;                                 // [KM OBD_Fehler_Radsensor_VL b6[0-3], _VR b6[4-7]] [REA spec §3: not read]
  frame.data[7] = 0x00;                                 // [KM ESP_Qualifizierung_Antriebsart b7.0, ..., ESP_Rollenmodus_Deaktivieren b7.6=0] [REA spec §3: b7.6 only, used with engine speed in FUN_c4320c; VAQ getter not traced]
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_392); // for 0x392

  ESP_07_counter++;
  if (ESP_07_counter > 0x2F) // V11.00.0: was "> 0x20 -> 0x2F", which ran backwards and stuck at 0x2F (spec_gen5_0cq D15; harmless, the 0CQ doesn't read it)
  {
    ESP_07_counter = 0x20; // counter runs 0x20..0x2F
  }
  standaloneTx(frame);
  /*
   0x91,0x91,0x91,0x91,0x91,0x91,0x91,0x91,0x91,0x91,0x91,0x91,0x91,0x91
   */

  // MQB ESP_29 (0x18C, DLC 8) - ESP slow-rate diagnostic broadcast.
  // VAQ K-matrix: NOT in the VAQ receive list. Kept like-for-like.
  frame.identifier = ESP_29;  // ESP_29 0x18C
  frame.data_length_code = 8; // DLC 8
  frame.data[0] = 0x00;       // [KM - (ESP_29 has no CRC in the K-matrix)] [REA audit R6: 0x18C not in the 0CQ or VAQ RX table]
  frame.data[1] = 0x20;       // held (no roll seen in SavvyCAN) [KM - b1[0-3] (no BZ), ESP_Monitoring_Gierrate b1[4-6]=2, ESP_Monitoring_Laengsb low b1.7] [REA: not received]
  frame.data[2] = 0x59;       // [KM ESP_Monitoring_Laengsb high b2[0-1], _Lenkwinkel b2[2-4], _Querb b2[5-7]] [REA: not received]
  frame.data[3] = 0x00;       // [KM OBD_Fehler_Bremsdruck b3[0-3]] [REA: not received]
  frame.data[4] = 0x00;       // [KM -] [REA: not received]
  frame.data[5] = 0x00;       // [KM -] [REA: not received]
  frame.data[6] = 0x00;       // [KM -] [REA: not received]
  frame.data[7] = 0x00;       // [KM -] [REA: not received]
  standaloneTx(frame);
}

void Gen5_0CQ_VAQ_frames200()
{
  twai_message_t frame = {};
  // PQ Kombi_2 (0x420, DLC 8) - cluster temps (vw_pq.dbc: Kombi_2), same bytes as the
  // Gen4 / 0AY copy. Transmit currently disabled (commented out below).
  // VAQ K-matrix: PQ-era ID, not an MQB FCAN message at all. Transmit stays
  // disabled, exactly as in the 0CQ set. [KM -] [REA audit §4: 0x420 not in the VAQ RX table]
  frame.identifier = mKombi_2; // Kombi_2 0x420
  frame.data_length_code = 8;  // DLC 8
  frame.data[0] = 0x4C;        // Kuehlmitteltemp (coolant * 0.75 - 48 degC)
  frame.data[1] = 0x86;        // Oeltemperatur (oil temp)
  frame.data[2] = 0x85;        // Aussentemperatur_gefiltert (outside-air filtered)
  frame.data[3] = 0x00;        // Aussentemperatur_ungefiltert (outside-air raw)
  frame.data[4] = 0x00;        // reserved
  frame.data[5] = 0x30;        // status
  frame.data[6] = 0xFF;        // reserved
  frame.data[7] = 0x04;        // reserved / counter
  // standaloneTx(frame);
}

void Gen5_0CQ_VAQ_frames1000()
{
  twai_message_t frame = {};
  // MQB Motor_07 (0x640, DLC 8) - engine slow-rate diagnostics broadcast.
  // VAQ K-matrix: NOT in the VAQ receive list. Kept like-for-like.
  frame.identifier = MOTOR_07; // Motor_07 0x640, 1000 ms
  frame.data_length_code = 8;  // DLC 8
  frame.data[0] = 0xA0;        // (no CRC in KM) [KM Motor_07 QBit_Ansaugluft/Oel/Kuehlmittel_Temp b0[0-2], KL15_Abschaltempfehlung b0.3, ...] [REA: 0CQ not received (audit R6); VAQ in RX table (audit §4), getter not traced]
  frame.data[1] = 0x5A;        // [KM MO_Ansaugluft_Temp 0.75-48 degC] [REA: as b0]
  frame.data[2] = 0x56;        // [KM MO_Oel_Temp -60 degC] [REA: as b0]
  frame.data[3] = 0xA3;        // [KM MO_Kuehlmittel_Temp 0.75-48 degC] [REA: as b0]
  frame.data[4] = 0x80;        // [KM MO_Hoeheninfo 0.00781] [REA: as b0]
  frame.data[5] = 0xA0;        // [KM MO_Kennfeldk b5.0, MO_Versionsinfo_02 b5[1-5], QBit_Hoeheninfo b5.6, Getriebe_kuehlen b5.7] [REA: as b0]
  frame.data[6] = 0x59;        // [KM MO_Mom_Traegheit_02 b6[0-4], MO_Heizungspumpenansteuerung b6[5-7]] [REA: as b0]
  frame.data[7] = 0x01;        // [KM Heizungspumpe high b7.0, MO_SpannungsAnf b7.1, MO_Nachlaufzeit_Heizungspumpe b7[2-7]] [REA: as b0]
  standaloneTx(frame);

  // MQB Charisma_01 (0x385, DLC 8) - drive-mode (Charisma) target per system.
  // VAQ K-matrix: reads CHA_Ziel_FahrPr_VAQ (b4 bits 4-7). 0CQ b4 = 0x02 ->
  // VAQ programme 0 'no function'. Candidates: 0x12 / 0x22 / 0x32 (programme
  // 1 / 2 / 3) via OVR 385 4 0x12 etc.
  frame.identifier = CHARISMA_01; // Charisma_01 0x385
  frame.data_length_code = 8;     // DLC 8 no effect from any
  frame.data[0] = 0x00;           // (no CRC in KM) [KM Charisma_01 CHA_Ziel_FahrPr_AGA b0[0-3], _ESP b0[4-7]] [REA: 0CQ not received (audit R6); VAQ in RX table (audit §4), getter not traced]
  frame.data[1] = 0x00;           // [KM CHA_Ziel_FahrPr_Freilauf b1[0-3], CHA_Fahrer_Umschaltung b1.6, _BMA b1.7] [REA: as b0]
  frame.data[2] = 0x22;           // [KM CHA_Ziel_FahrPr_MO b2[0-3]=2, _GE b2[4-7]=2] [REA: as b0]
  frame.data[3] = 0x02;           // [KM CHA_Ziel_FahrPr_ALR b3[0-3]=2 (Haldex), _MO_BZS b3[4-7]] [REA: as b0]
  frame.data[4] = 0x02;           // [KM CHA_Ziel_FahrPr_DR b4[0-3]=2, CHA_Ziel_FahrPr_VAQ b4[4-7]=0 'keine_Funktion'] [REA: as b0]
  frame.data[5] = 0x20;           // [KM CHA_Ziel_FahrPr_AFS b5[0-3], _RGS b5[4-7]] [REA: as b0]
  frame.data[6] = 0x02;           // [KM CHA_Ziel_FahrPr_EPS b6[0-3], _ACC b6[4-7]] [REA: as b0]
  frame.data[7] = 0x02;           // [KM CHA_Ziel_FahrPr_SAK b7[0-3], _MO_StSt b7[4-7]] [REA: as b0]
  standaloneTx(frame);

  // MQB Systeminfo_01 (0x585, DLC 8) - system identification/info broadcast.
  // VAQ K-matrix: reads SI_NWDF (b1 bit 2) and SI_NWDF_gueltig (b1 bit 3).
  // 0CQ b1 = 0x3C sets both (network diagnostics released and valid).
  frame.identifier = SYSTEMINFO_01; // Systeminfo_01 0x585
  frame.data_length_code = 8;       // DLC 8
  frame.data[0] = 0x84;             // (no CRC in KM) [KM Systeminfo_01 SI_Sammel_SG_Fehler b0[0-5], SI_P_Mode_gueltig b0.6, SI_Diagnose_Aktiv b0.7] [REA spec §3: not read]
  frame.data[1] = 0x3C;             // [KM SI_QRS_Mode b1.0, SI_T_Mode b1.1, SI_NWDF b1.2=1, SI_NWDF_gueltig b1.3=1, SI_Sammelfehler b1.4=1, GW_KD_Fehler b1.5=1, ...] [REA spec §3: b1 bits 2/3 only, monitor-enable (FUN_c20236/c4b482); VAQ getter not traced]
  frame.data[2] = 0x00;             // [KM SI_BUS_01..08] [REA spec §3: not read]
  frame.data[3] = 0x7F;             // [KM SI_BUS_09..15] [REA spec §3: not read]
  frame.data[4] = 0x14;             // [KM SI_Bus_Identifikation] [REA spec §3: not read]
  frame.data[5] = 0x00;             // [KM -] [REA spec §3: not read]
  frame.data[6] = 0x00;             // [KM -] [REA spec §3: not read]
  frame.data[7] = 0x00;             // [KM -] [REA spec §3: not read]
  standaloneTx(frame);

  // MQB Motor_Code_01 (0x641, DLC 8) - engine code / identification broadcast.
  // VAQ K-matrix: reads MO_Faktor_Momente_02 (b1 bits 4-5) and MO_Kraftstoffart
  // (b4 bits 4-7). b4 = 0x14 -> petrol. OEM code (spec_gen5_0cq §8.8): the VAQ
  // reads Motor_Code_01 b1 bits 4-5 at the same place as the 0CQ.
  // V11.00.0: Faktor = 1 set on purpose (spec D18), duplicate header lines gone
  // (D16). Same bytes as before, so nothing changes on the wire. The checksum is
  // still left at 0 here (it always was on the VAQ set).
  frame.identifier = MOTOR_CODE_01; // Motor_Code_01 0x641
  frame.data_length_code = 8;       // DLC 8
  frame.data[0] = 0x00;             // left 0 [KM Motor_Code_01.Motor_Code_01_CRC] [REA audit §4: no CRC check]
  frame.data[1] = (1 << 4) | (MOTOR_CODE_01_counter & 0x0F); // [KM Motor_Code_01_BZ b1[0-3], MO_Faktor_Momente_02 b1[4-5]=1, MO_Hybridfahrzeug b1[6-7]] [REA spec §8.8: VAQ COM table has b1[4-5] at the same place; use not re-checked (audit §4)]
  frame.data[2] = 0x2B;                  // [KM MO_Code] [REA: not traced]
  frame.data[3] = 0x53;                  // [KM MO_Getriebe_Code b3[0-5]=19 DQ250, MO_StartStopp_Codiert b3.6, MO_Anzeige_StSt_gueltig b3.7] [REA: not traced]
  frame.data[4] = 0x14;                  // [KM MO_Anzahl_Zyl b4[0-3]=4, MO_Kraftstoffart b4[4-7]=1 Benzin] [REA: not traced]
  frame.data[5] = 0x14;                  // [KM MO_Hubraum b5[0-6] 0.1 l, MO_Ansaugsystem b5.7] [REA: not traced]
  frame.data[6] = 0xD7;                  // [KM MO_Leistung low, kW] [REA: not traced]
  frame.data[7] = 0x24;                  // [KM MO_Leistung high b7.0, MO_OBD_Typ, MO_DPF_verbaut, TSK_Codierung] [REA: not traced]

  MOTOR_CODE_01_counter++;
  if (MOTOR_CODE_01_counter > 0x1F)
  {
    MOTOR_CODE_01_counter = 0x10;
  }

  standaloneTx(frame);
  /*
  0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47
  */

  // MQB ESP_20 (0x65D, DLC 8) - ESP slow diagnostic broadcast incl. tyre circumference.
  // VAQ K-matrix: reads BR_Systemart (b1 bits 4-5), BR_QBit_Reifenumfang (b6
  // bit 3) and BR_Reifenumfang (b6[4-7] + b7, mm). Counter 0x30-0x3F ->
  // Systemart 3 'ESP with EPB'; b6/b7 = 0xE2/0x79 -> 1950 mm, QBit 0.
  frame.identifier = ESP_20;                            // ESP_20 0x65D
  frame.data_length_code = 8;                           // DLC 8 (no effect from any byte)
  frame.data[0] = 0x00;                                 // [KM ESP_20.ESP_20_CRC] [REA 0CQ audit S6: no CRC check]
  frame.data[1] = ESP_20_counter;                       // 0x30..0x3F [KM ESP_20_BZ b1[0-3], BR_Systemart b1[4-5]=3 'ESP_mit_integrierter_EPB', ESP_SpannungsAnf_02 b1[6-7]] [REA spec §3/§8.8: b1[4-5] read (0CQ + VAQ); 3 = gate that disables the Kombi_01 handbrake flag]
  frame.data[2] = 0x2B;                                 // [KM ESP_Zaehnezahl (43)] [REA spec §3: not read]
  frame.data[3] = 0x10;                                 // [KM ESP_Charisma_FahrPr b3[0-3], ESP_Charisma_Status b3[4-5]=1, Wiederstart_Anz b3[6-7]] [REA spec §3: not read]
  frame.data[4] = 0x00;                                 // [KM ESP_Wiederstart/Stoppverbot_Anz flags] [REA spec §3: not read]
  frame.data[5] = 0x00;                                 // [KM Stoppverbot_Anz, ESP_Dachrelingsensor b5[2-3], HDC_Charisma_FahrPr low b5[5-7]] [REA spec §3: not read]
  frame.data[6] = 0xE2;                                 // [KM HDC_Charisma_Status b6[1-2], BR_QBit_Reifenumfang b6.3=0, BR_Reifenumfang low b6[4-7]] [REA spec §3: bits 52-63 read; b6.3 COM only]
  frame.data[7] = 0x79;                                 // 0x79E = 1950 mm [KM BR_Reifenumfang high, mm] [REA spec §3: tyre circumference -> rolling radius 0xCEBE = mm x1024/6283]
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_65d); // for 0x65d

  ESP_20_counter++;
  if (ESP_20_counter > 0x3F)
  {
    ESP_20_counter = 0x30;
  }

  standaloneTx(frame);
  /*
  0xac,0xb3,0xab,0xeb,0x7a,0xe1,0x3b,0xf7,0x73,0xba,0x7c,0x9e,0x06,0x5f,0x02,0xd9
  */

  // MQB Diagnose_01 (0x6B2, DLC 8) - generic diagnostic broadcast.
  // VAQ K-matrix: reads DGN_Verlernzaehler (b0), KBI_Kilometerstand (b1-b3[0-3])
  // and the date/time fields. All static in this set.
  frame.identifier = DIAGNOSE_01; // Diagnose_01 0x6B2
  frame.data_length_code = 8;     // DLC 8
  frame.data[0] = 0x30;           // (no CRC in KM) [KM Diagnose_01.DGN_Verlernzaehler] [REA spec §3: b0 drive-cycle counter (FUN_c4c198), fault memory only - not control]
  frame.data[1] = 0x4D;           // [KM KBI_Kilometerstand bits 8-27, km] [REA spec §3: mileage stamp for fault memory (FUN_c1c582)]
  frame.data[2] = 0x58;           // [KM KBI_Kilometerstand] [REA spec §3: as b1]
  frame.data[3] = 0xA2;           // [KM KBI_Kilometerstand high b3[0-3], UH_Jahr low b3[4-7] (+2000)] [REA spec §3: fault-memory time stamp]
  frame.data[4] = 0x89;           // [KM UH_Jahr high b4[0-2], UH_Monat b4[3-6], UH_Tag low b4.7] [REA spec §3: time stamp]
  frame.data[5] = 0x85;           // [KM UH_Tag high b5[0-3], UH_Stunde low b5[4-7]] [REA spec §3: time stamp]
  frame.data[6] = 0x3F;           // car toggles 3F/BF [KM UH_Stunde high b6.0, UH_Minute b6[1-6], UH_Sekunde low b6.7] [REA spec §3: time stamp]
  frame.data[7] = 0x30;           // car steps 2D,2D,2E,2E.. [KM UH_Sekunde high b7[0-4], Vehicle_Driving_Cycle b7.5, Kombi_02_alt b7.6, Uhrzeit_01_alt b7.7] [REA spec §3: time stamp]
  standaloneTx(frame);

  // MQB Kombi_02 (0x6B7, DLC 8) - instrument cluster slow-rate broadcast.
  // VAQ K-matrix: reads KBI_Standzeit_02 (b2[4-7] .. b4[0-4], seconds).
  frame.identifier = KOMBI_02; // Kombi_02 0x6B7
  frame.data_length_code = 8;  // DLC 8
  frame.data[0] = 0x4D;        // (no CRC in KM) [KM Kombi_02.KBI_Kilometerstand bits 0-19, km] [REA spec §3: not read]
  frame.data[1] = 0x58;        // [KM KBI_Kilometerstand] [REA spec §3: not read]
  frame.data[2] = 0xF2;        // [KM KBI_Kilometerstand high b2[0-3], KBI_Standzeit_02 low b2[4-7], s] [REA spec §3: bits 20-36 read ONCE at start, <= 0x1FFFC seeds the clutch-temperature model 0xCEF6]
  frame.data[3] = 0xEE;        // [KM KBI_Standzeit_02] [REA spec §3: as b2]
  frame.data[4] = 0x04;        // standing time 20207 s [KM KBI_Standzeit_02 high b4.0, KBI_Reserve_Warnung_Sek b4.6, KBI_RDK_Kalibrier_Anf b4.7] [REA spec §3: bit 36 = last Standzeit bit]
  frame.data[5] = 0x2B;        // [KM KBI_Inhalt_Tank b5[0-6] l, KBI_FStatus_Tank b5.7] [REA spec §3: not read]
  frame.data[6] = 0x00;        // [KM KBI_aktive_Laengsfunktion b6[0-2], ..., KBI_QBit_Aussen_Temp_gef b6.7] [REA spec §3: not read]
  frame.data[7] = 0x78;        // [KM KBI_Aussen_Temp_gef 0.5-50 degC (10 degC)] [REA spec §3: not read]
  standaloneTx(frame);
}

// =============================================================================
// Gen5 (0AY) Standalone Frames
// The 0AY (0AY907554F) is a PQ unit - PQ frame set, feedback Allrad_1 0x2C0 -
// with the same lock request as Gen4 0BR: Bremse_4 (0x2A0) byte 0 (OEM code:
// spec_gen5_0ay §1/§2). It receives 16 broadcast frames (spec §3); none has a
// counter or checksum check. Bremse_8 0x1AC, Bremse_11 0x5B7 and Kombi_2 0x420
// are NOT received by the 0AY - their blocks default OFF (V11.00.0).
// V11.00.13 per-byte tags: [KM msg.signal scale] = vw_pq.dbc (PQ35/46 KMatrix
// naming), [REA ...] = what the 0AY's own code does with the byte - sources
// leaf_trace/audit_gen5_0ay.md (overrules) + spec_gen5_0ay.md §1/§3 (getters,
// audit #31/#32 CONFIRMED for sw7521). "not stored" = beyond the bytes the unit
// copies (spec §1 table), so it can never be read. sw7543 (0AY907554F->M) also
// stores Motor_5 b1-6 / Bremse_1 b2-7 and reads Motor_5 b6.3 + Bremse_1 b7.5 (#8).
// =============================================================================
void Gen5_0AY_frames10()
{
  twai_message_t frame = {};
  // PQ LW_1 / mLW_1 (0x0C2, DLC 8) - steering-angle replay recorded from a real
  // car (about 1.5 deg, straight ahead). The 0AY reads b0-b1 (angle, b1.7 sign)
  // and b4; a static frame would also do (spec_gen5_0ay §5).
  frame.identifier = mLW_1;
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  // mLW_1_counter is shared with Gen2, which steps it in 0x10s: mask it to the 16-row
  // table so a generation change at runtime can't read past lws2 (V11.00.8)
  mLW_1_counter = mLW_1_counter & 0x0F;
  frame.data[0] = lws2[mLW_1_counter][0]; // [KM Lenkwinkel_1.LW1_LRW low, 15 bit 0.04375 deg] [REA 0AY: steering angle (sig 120) -> 0xCE4C]
  frame.data[1] = lws2[mLW_1_counter][1]; // [KM LW1_LRW bits 8-14 + b1.7 LW1_LRW_Sign] [REA 0AY: angle + sign (sig 121)]
  frame.data[2] = lws2[mLW_1_counter][2]; // [KM LW1_Lenk_Gesch low, 0.04375 deg/s] [REA 0AY: not read]
  frame.data[3] = lws2[mLW_1_counter][3]; // [KM LW1_Lenk_Gesch high + b3.7 LW1_Gesch_Sign] [REA 0AY: not read]
  frame.data[4] = lws2[mLW_1_counter][4]; // [KM LW1_ID] [REA 0AY: read in FUN_c4a69c (sig 123)]
  frame.data[5] = lws2[mLW_1_counter][5]; // [KM b5.0 LW1_Initquelle, b5.1-2 LW1_Status, b5.3 Sta_KL30, b5.4-7 LW1_Zaehler] [REA 0AY: status getter, unused; counter not read]
  frame.data[6] = lws2[mLW_1_counter][6]; // [KM LW1_Kodier] [REA 0AY: not stored (6 bytes)]
  frame.data[7] = lws2[mLW_1_counter][7]; // [KM LW1_CRC] [REA 0AY: not stored]

  mLW_1_counter++;
  if (mLW_1_counter > 15)
    mLW_1_counter = 0;
  standaloneTx(frame);

  // Steady wheel / vehicle speed for Bremse_1 and Bremse_3, in 0.01 km/h.
  // The car's real speed when OpenHaldex has a chassis bus, otherwise a fixed
  // 10.00 km/h (about what the pre-V11 frames sent). Same on all four wheels:
  // no fake slip - the lock request is Bremse_4 (spec_gen5_0ay §5 / §7 #4).
  uint16_t wheelSpeed = 1000; // 10.00 km/h when no chassis speed is available
  if (hasCANChassis)
  {
    uint16_t chassisSpeed = receivedVehicleSpeed; // km/h from the car
    if (chassisSpeed > 326)
    {
      chassisSpeed = 326; // top of the 15-bit 0.01 km/h range
    }
    wheelSpeed = chassisSpeed * 100;
  }
  uint16_t wheelSpeedWord = wheelSpeed << 1; // 15-bit value sits above bit 0 of each word

  // PQ Bremse_1 (0x1A0, DLC 8) - ABS/ESP main broadcast (vw_pq.dbc: Bremse_1).
  // The 0AY reads b0.0, b0.2, b0.3, b0.4 and b1.3 (spec_gen5_0ay §3/§5).
  // Before V11 b0 was 0x20 (BR1_ASR_Ongr) and b1 0x40 - now all intervention
  // flags off, except the real ABS-braking bit carried from the car.
  frame.identifier = BRAKES1_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // [KM Bremse_1 b0.0 BR1_ASR_Anf, b0.2 BR1_ABS_Brems, b0.3 EDS_Ongr, b0.4 ESP_Ongr] [REA 0AY audit #10: b0.2 -> 0xCF50 (state 2 with BLS), b0.4 -> 0xCF52, b0.0/b0.3 -> 0xCF55/56]
  if (absBrakingForHaldex())
  {
    frame.data[0] = frame.data[0] | (1 << 2); // b0.2 BR1_ABS_Brems - ABS module is braking
  }
  frame.data[1] = 0x00; // [KM b1.0-2 lamps, b1.3 BR1_Lichtschalt, b1.4 StaDruckschw] [REA 0AY: b1.3 getter only (sig 119)]
  if (brakeForHaldex())
  {
    frame.data[1] = frame.data[1] | (1 << 3); // b1.3 BR1_Lichtschalt - brake-light switch
  }
  frame.data[2] = (uint8_t)(wheelSpeedWord & 0xFF); // [KM b2.0 BR1_BKV_active, b2.1-7 BR1_Rad_kmh low 0.01 km/h] [REA 0AY: not stored (2 bytes; sw7543 stores, no reader)]
  frame.data[3] = (uint8_t)(wheelSpeedWord >> 8);   // [KM BR1_Rad_kmh high] [REA 0AY: not stored]
  frame.data[4] = 0xFE;                             // [KM BR1_ASRMo_sl 0.39 %] no reduction [REA 0AY: not stored]
  frame.data[5] = 0xFE;                             // [KM BR1_ASRMo_fa / BR1_MSR_Mo_inv 0.39 %] [REA 0AY: not stored]
  frame.data[6] = 0x00;                             // [KM BR1_MSR_Mo 0.39 %] [REA 0AY: not stored]
  frame.data[7] = BRAKES1_counter;                  // [KM b7.0-3 BR1_Zaehler, b7.4 BR1_ASR_ESP, b7.5 ESPASR_passive, b7.6-7] 10..31 -> b7.4 toggles, b7.5 never set [REA 0AY: not stored; sw7543 reads b7.5 (FUN_c4b31c, audit F6)]
  if (++BRAKES1_counter > 0x1F)
    BRAKES1_counter = 10;
  standaloneTx(frame);

  // PQ Bremse_3 (0x4A0, DLC 8) - per-wheel speeds (vw_pq.dbc: Bremse_3).
  // Each wheel is a 16-bit little-endian word, speed in bits 1-15 (0.01 km/h).
  // Was lock-scaled before V11 (fake rear>front slip that changed with lock).
  frame.identifier = BRAKES3_ID;
  frame.data_length_code = 8;
  frame.data[0] = (uint8_t)(wheelSpeedWord & 0xFF); // [KM Bremse_3 b0.0 Frei, b0.1-7 Radgeschw__VL_4_1 low 0.01 km/h] [REA 0AY: word>>1 (sig 93) -> vehicle speed 0xCDA0, DID 0x2BF0]
  frame.data[1] = (uint8_t)(wheelSpeedWord >> 8);   // [KM Radgeschw__VL_4_1 high] [REA 0AY: VL]
  frame.data[2] = (uint8_t)(wheelSpeedWord & 0xFF); // [KM b2.0 Frei, b2.1-7 Radgeschw__VR_4_1 low] [REA 0AY: VR (sig 94)]
  frame.data[3] = (uint8_t)(wheelSpeedWord >> 8);   // [KM Radgeschw__VR_4_1 high] [REA 0AY: VR]
  frame.data[4] = (uint8_t)(wheelSpeedWord & 0xFF); // [KM b4.0 Frei, b4.1-7 Radgeschw__HL_4_1 low] [REA 0AY: HL (sig 95)]
  frame.data[5] = (uint8_t)(wheelSpeedWord >> 8);   // [KM Radgeschw__HL_4_1 high] [REA 0AY: HL]
  frame.data[6] = (uint8_t)(wheelSpeedWord & 0xFF); // [KM b6.0 Frei, b6.1-7 Radgeschw__HR_4_1 low] [REA 0AY: HR (sig 96)]
  frame.data[7] = (uint8_t)(wheelSpeedWord >> 8);   // [KM Radgeschw__HR_4_1 high] [REA 0AY: HR; avoid init marker 0xFF88]
  standaloneTx(frame);

  // PQ Bremse_4 (0x2A0, DLC 8) - THE LOCK REQUEST (OEM code: spec_gen5_0ay §2).
  // Byte 0: 0x7F = clutch open, 0x80 + n = n x 112 torque request, 0xFD = max.
  // 0xFE means HOLD on the 0AY and 0x00-0x7E / 0xFF are ignored, so 100 % is
  // 0xFD. vagLockRequestByte() sends 0x7F whenever a lock gate is closed - ABS
  // braking, handbrake (Follow Handbrake), speed / throttle settings - which
  // a valid Bremse_4 needs, because it outranks the unit's own clutch-open.
  // Before V11 this was adj(0x7F) - 0x7F: 100 % gave 0x00 (ignored!).
  frame.identifier = BRAKES4_ID;
  frame.data_length_code = 8;
  frame.data[0] = vagLockRequestByte(GEN51_REQUEST_FULL, GEN51_REQUEST_FULL_BYTE); // [KM Bremse_4.ABS_Vorgabewert_mitte_Kupplungs (3,-381) Nm/min] [REA audit_gen5_0ay #11/#12: 0x7F open, 0x80-0xFD (b-0x80)x112 capped by 0xCD34, 0xFE hold, else invalid]
  frame.data[1] = 0x00;                 // [KM ABS_Vorgabewert_hinten_Kupplung 0.7874 %] [REA 0AY: not read]
  frame.data[2] = 0x00;                 // [KM b2.0 Einheit_Kupplungssteifigkeit, b2.1-7 Frei_Bremse_4_1] [REA 0AY: b2 bits 0-1 getter (sig 103), no consumer]
  frame.data[3] = 0x00;                 // [KM -] [REA 0AY: not stored (3 bytes)]
  frame.data[4] = 0x00;                 // [KM -] [REA 0AY: not stored]
  frame.data[5] = 0x00;                 // [KM -] [REA 0AY: not stored]
  frame.data[6] = BRAKES4_counter;      // [KM -] rolling counter [REA 0AY: not stored]
  BRAKES4_crc = 0;
  for (uint8_t i = 0; i < 7; i++)
  {
    BRAKES4_crc ^= frame.data[i];
  }
  frame.data[7] = BRAKES4_crc; // [KM -] XOR over data[0..6] [REA 0AY: not stored]
  BRAKES4_counter = BRAKES4_counter + 16;
  if (BRAKES4_counter > 0xF0)
    BRAKES4_counter = 0x00;
  standaloneTx(frame);

  // PQ Motor_1 (0x280, DLC 8) - engine ECU broadcast (vw_pq.dbc: Motor_1).
  // Only feeds the unit's own strategy (which a valid Bremse_4 overrides), so
  // steady plausible values from spec_gen5_0ay §5. Was lock-scaled before V11
  // (rpm 0 at FWD, torque / pedal 0xFE at 100 %).
  frame.identifier = MOTOR1_ID;
  frame.data_length_code = 8;
  // (audit_gen5_0ay F3: the 0xCD34 limiter inputs were not traced, so whether
  // rpm also gates lock on the 0AY, as on the 0BR, is open - 800 rpm is > 401.)
  frame.data[0] = 0x00; // [KM Motor_1 b0.0 Leergasinformation .. b0.3 Kupplungsschalter .. b0.7] [REA 0AY: b0.3 (sig 109) -> 0xCF67 = !bit; other bits not read]
  frame.data[1] = 0x20; // [KM inneres_Motor_Moment 0.39 % MDI] [REA 0AY: with b6 -> torque estimate 0xCE62 (skipped if 0xFF)]
  frame.data[2] = 0x80; // [KM Motordrehzahl low, 16 bit 0.25 rpm] 0x0C80 = 800 rpm [REA 0AY: rpm (sig 111) -> 0xCE60]
  frame.data[3] = 0x0C; // [KM Motordrehzahl high] [REA 0AY: rpm]
  frame.data[4] = 0x20; // [KM inneres_Motor_Moment_ohne_exter 0.39 % MDI] [REA 0AY: never read]
  frame.data[5] = 0x00; // [KM Fahrpedalwert_oder_Drosselklapp 0.4 %] 0 % [REA 0AY: pedal (sig 112) -> 0xCF61, state machine]
  frame.data[6] = 0x10; // [KM mechanisches_Motor_Verlustmomen 0.39 % MDI] [REA 0AY: loss torque -> 0xCE62 (skipped if 0xFF)]
  frame.data[7] = 0x00; // [KM Fahrerwunschmoment 0.39 % MDI] [REA 0AY: used only if Motor_5 latch 0xCE78 == 1]
  standaloneTx(frame);

  // ---------------------------------------------------------------------------
  // PQ Getriebe_2 / mGetriebe_2 (0x540, DLC 8) - transmission status broadcast.
  // Source: PQ35_46 ACAN KMatrix V5.20.6F, cycle 10 ms, timeout 500 ms.
  // Required by haldex DTC 17497 (mGetriebe_2 timeout) on 0AY.
  // Layout (KMatrix bytes are 1-based; byte 1 = data[0]):
  //   data[0]: bit0 LFR_Adap_Freig, bit1 Schub_AbsUnterst, bit2 ECO/4_1,
  //            bit3 ZwGasFlag, bits4..7 = Zaehler_Getriebe_2 (rolling 0..F)
  //   data[1]: Leerlaufsolldrehzahl (idle target rpm, scale 10)
  //   data[2]: Gradientenbegrenzung (torque-gradient limit, init 0xFF = no limit)
  //   data[3]: Synchronisations_Wunschdrehzahl (sync rpm wish, scale 25)
  //   data[4]: invertierte_Synchronisations_Wu (init 0xFF)
  //   data[5]: Synchronisationszeit (sync time ms, scale 20)
  //   data[6]: status bit-field (Hochschaltlampe, Starter, Gong, Unterdrueck,
  //            ShiftLock, ECO, KriechAdapt, Fehler_Kupp)
  //   data[7]: low nibble Ganganzeige_Kombi (display gear, 0xF = ---),
  //            high nibble eingelegte_Fahrstufe (PRNDL position)
  // ---------------------------------------------------------------------------
  frame.identifier = mGetriebe_2;
  frame.data_length_code = 8;
  frame.data[0] = (mGetriebe_2_counter << 4) & 0xF0; // [KM Getriebe_2 b0.0-3 flags, b0.4-7 Zahler_Getriebe_2] [REA 0AY: not read]
  frame.data[1] = 0x50;                              // [KM Leerlaufsolldrehzahl__Getriebe 10 rpm] 800 rpm [REA 0AY: not read]
  frame.data[2] = 0xFF;                              // [KM Gradientenbegrenzung 10 Nm/s] [REA 0AY: not read]
  frame.data[3] = 0x00;                              // [KM Synchronisations_Wunschdrehzahl 25 rpm] [REA 0AY: not read]
  frame.data[4] = 0xFF;                              // [KM invertierte_Synchronisations_Wu 25 rpm] [REA 0AY: not read]
  frame.data[5] = 0x00;                              // [KM Synchronisationszeit 20 ms] [REA 0AY: not read]
  frame.data[6] = 0x00;                              // [KM b6.0 Hochschaltlampe .. b6.5 ECO_Anzeige .. b6.7 Fehlerlampe_Kupplung] [REA 0AY: b6.5 (sig 124) = use b7 gear when Getriebe_1 absent]
  frame.data[7] = 0xFF;                              // [KM b7.0-3 Ganganzeige_Kombi, b7.4-7 eingelegte_Fahrstufe] [REA 0AY: b7.0-3 gear (sig 125), only if b6.5 = 1]
  mGetriebe_2_counter = (mGetriebe_2_counter + 1) & 0x0F;
  standaloneTx(frame);
  // The 0AY reads Getriebe_2 b6.5 and b7 bits 0-3 (gear) only when Getriebe_1
  // is missing and b6.5 = 1 (spec_gen5_0ay §3). b6 = 0 here, so the gear comes
  // from Getriebe_1 (sent in Gen5_0AY_frames20).

  // PQ Bremse_5 (0x4A8, DLC 8) - ESP brake-event broadcast.
  // vw_pq.dbc: BR5_Giergeschw / BR5_Gierrate (yaw rate * 0.01 deg/s),
  //            BR5_Bremsdruck (brake pressure * 0.1 bar), BR5_Bremslicht,
  //            BR5_Notbremsung (emergency-brake), BR5_HDC_bereit (hill-descent-ready).
  // The 0AY reads only b0 + b1 bits 0-5 (yaw rate) and b1.7 (sign) - spec_gen5_0ay
  // §3/§5: yaw 0. Before V11 b0/b1 were 0xFE/0x7F (near-maximum yaw), noted on
  // the car as "affects estimated torque AND vehicle mode" - that was the unit's
  // own strategy, which only ran because Bremse_4 was being ignored.
  frame.identifier = BRAKES5_ID;
  frame.data_length_code = 8;       // DLC 8 (the 0AY stores 2 bytes)
  // V11.00.13: b3-b7 names corrected from vw_pq.dbc (no "Gierrate" signal at b4-5).
  frame.data[0] = 0x00;             // [KM Bremse_5.BR5_Giergeschw low, 14 bit 0.01 deg/s] yaw 0 [REA 0AY: yaw (sig 90) -> 0xCE4E]
  frame.data[1] = 0x00;             // [KM b1.0-5 Giergeschw high, b1.6 BR5_Sta_Gierrate, b1.7 BR5_Vorzeichen] [REA 0AY: yaw + sign (sig 92); b1.6 never used]
  frame.data[2] = 0x03;             // [KM BR5_Bremsdruck low, 12 bit 0.1 bar] [REA 0AY: not stored (2 bytes)]
  frame.data[3] = 0x00;             // [KM b3.0-3 Bremsdruck high, b3.4 Stillstand, b3.5 Druckvalid, b3.6 Sta_Druck, b3.7 Sign_Druck] [REA 0AY: not stored]
  frame.data[4] = 0x00;             // [KM b4.2-7 ESP_Rollenmodus / FallBack_eBKV / Anforderung_EPB / Autohold] [REA 0AY: not stored]
  frame.data[5] = 0x00;             // [KM b5.0 Anhi_Sta .. b5.3 BR5_Bremslicht, b5.4 Notbremsung ..] [REA 0AY: not stored]
  frame.data[6] = BRAKES5_counter;  // [KM b6.0-3 ZT/ANB/HDC_bereit/ECD_Lampe flags, b6.4-7 Bremse_5.COUNTER] (+10 steps also toggle b6.0-3) [REA 0AY: not stored]
  frame.data[7] = BRAKES5_counter2; // [KM Bremse_5.CHECKSUM] (not a real XOR) [REA 0AY: not stored]
  BRAKES5_counter = BRAKES5_counter + 10;
  if (BRAKES5_counter > 0xF0)
  {
    BRAKES5_counter = 0;
  }
  BRAKES5_counter2 = BRAKES5_counter2 + 10;
  if (BRAKES5_counter2 > 0xF3)
  {
    BRAKES5_counter2 = 3;
  }
  standaloneTx(frame);

  // PQ Bremse_8 / mBremse_8 (0x1AC, DLC 8) - ESP supplemental broadcast.
  // NOT received by the 0AY (no message object - spec_gen5_0ay §1): its block
  // defaults OFF in standalone (V11.00.0), so standaloneTx() skips it.
  // data[0] rolls 0x80..0x8F, data[1] rolls 0x00..0x0F (independent counters).
  frame.identifier = BRAKES8_ID;
  frame.data_length_code = 8;
  frame.data[0] = BRAKES8_counter;  // [KM Bremse_8.BR8_Checksumme] 0x80..0x8F [REA 0AY: not received]
  frame.data[1] = BRAKES8_counter1; // [KM b1.0-3 BR8_Zaehler, b1.4-7 status flags] 0x00..0x0F [REA 0AY: not received]
  frame.data[2] = 0x00;             // [KM BR8_TolAbgl_HL 0.0488 % -6.2] [REA 0AY: not received]
  frame.data[3] = 0x00;             // [KM BR8_TolAbgl_HR 0.0488 % -6.2] [REA 0AY: not received]
  frame.data[4] = 0x69;             // [KM BR8_Istbeschl low, 9 bit 0.02 m/s2 -7.22] [REA 0AY: not received]
  frame.data[5] = 0x21;             // [KM b5.0 Istbeschl high, b5.1 Sta_HW_BLS .. b5.7 StaBrSyst] [REA 0AY: not received]
  frame.data[6] = 0x00;             // [KM BR8_Laengsbeschl low, 10 bit 0.03125 m/s2 -16] [REA 0AY: not received]
  frame.data[7] = 0xC1;             // [KM b7.0-1 Laengsbeschl high, b7.2-7 ADR/Quattro/VerzReg/BLS/EPB flags] [REA 0AY: not received]
  standaloneTx(frame);
  if (++BRAKES8_counter > 0x8F)
    BRAKES8_counter = 0x80;
  if (++BRAKES8_counter1 > 0x0F)
    BRAKES8_counter1 = 0x00;
}

void Gen5_0AY_frames20()
{
  twai_message_t frame = {};
  // PQ Bremse_2 (0x5A0, DLC 8) - ESP/ABS sensor broadcast (vw_pq.dbc: Bremse_2).
  // The 0AY reads b0 (lateral accel, 0x7F = 0 g), b1.0 (mux) and b7.7 -
  // spec_gen5_0ay §5: b0 = 0x7F, b1.0 = 0, b7.7 = 0. Before V11: b0 0x80
  // (+0.01 g) and b7 0xAB (b7.7 = 1). Byte names corrected from vw_pq.dbc.
  frame.identifier = BRAKES2_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x7F;            // [KM Bremse_2.Querbeschleunigung (mux 0) 0.01 g -1.27] 0 g [REA 0AY: lat accel centred 0x7F (sig 83) -> 0xCE50]
  frame.data[1] = 0x7A;            // [KM b1.0 Querbeschl__TimerTic (mux), b1.1-7 mittlere_Raddrehzahl low 0.002 U/s] [REA 0AY: b1.0 read (sig 84, FUN_c46198); rest not read]
  frame.data[2] = 0x05;            // [KM mittlere_Raddrehzahl__Bremse_2 high] [REA 0AY: not read]
  frame.data[3] = BRAKES2_counter; // [KM Zeitstempel low, 16 bit tics] [REA 0AY: not read]
  frame.data[4] = 0x7F;            // [KM Zeitstempel high] [REA 0AY: not read]
  frame.data[5] = 0xCA;            // [KM Wegimpulse_Vorderachse low, 11 bit] [REA 0AY: not read]
  frame.data[6] = 0x1B;            // [KM b6.0-2 Wegimpulse high, b6.3 Wegimpulszaehlerstatus, b6.4 Fehlerspeichereintrag_Bremse, b6.5 Warnlampe_DDS, b6.6 Frei, b6.7 Fehlerstatus_Wegimpulse] [REA 0AY: b6.6 getter (sig 85), unused]
  frame.data[7] = 0x2B;            // [KM b7.0-5 Impulszahl, b7.6 Frei, b7.7 gemessene_Querbeschleunigung] b7.7 = 0 [REA 0AY: b7.7 read (sig 86, FUN_c46198)]
  standaloneTx(frame);
  BRAKES2_counter = BRAKES2_counter + 16;
  if (BRAKES2_counter > 0xF0)
  {
    BRAKES2_counter = 0;
  }

  // PQ Motor_2 (0x288, DLC 8) - engine broadcast (vw_pq.dbc: Motor_2).
  // The 0AY reads b0 (mux: bits 6-7 = 3 -> max torque), b2.0 MO2_BLS
  // (brake-light switch, used for its own brake clutch-open) and b2.4.
  frame.identifier = MOTOR2_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // [KM Motor_2 b0.6-7 mux, b0.0-5 MO2_CAN_Vers (mux 0)] - not mux 3 [REA 0AY: mux 3 -> b0[5:0] x 40 = max torque 0xCE6E; other mux ignored]
  frame.data[1] = 0x30; // [KM MO2_Kuehlm_T 0.75 -48 degC] [REA 0AY: never read]
  frame.data[2] = 0x00; // [KM b2.0 MO2_BLS, b2.1 MO2_BTS, b2.4 MO2_Sta_No_Bet] [REA 0AY: b2.0 -> 0xCF53 (state 2), b2.1 getter (sig 107), b2.4 read in FUN_c4d13a]
  if (brakeForHaldex())
  {
    frame.data[2] = frame.data[2] | (1 << 0); // b2.0 MO2_BLS - brake pedal pressed
  }
  frame.data[3] = 0x0A; // [KM b3.1 TSK_Limiter_ausgewaehlt, b3.3 MO2_Sport_Error (both = 1 here), b3.2/3.4-7] [REA 0AY: not stored (3 bytes)]
  frame.data[4] = 0x0A; // [KM MO2_GRA_Soll 1.28 km/h] [REA 0AY: not stored]
  frame.data[5] = 0x10; // [KM MO2_LL_Solldz 10 rpm] 160 rpm [REA 0AY: not stored]
  frame.data[6] = 0xFE; // [KM MO2_Begr_Mo 0.39 %] [REA 0AY: not stored]
  frame.data[7] = 0xFE; // [KM MO2_Mo_ZWR 0.39 %] [REA 0AY: not stored]
  standaloneTx(frame);

  // PQ Motor_5 (0x480, DLC 8) - engine broadcast (vw_pq.dbc: Motor_5, multiplexed).
  // The 0AY only latches b0 once, when the mux (bits 6-7) = 2 (spec_gen5_0ay §3).
  // 0xFE = mux 3, so nothing is latched. Must be present: if Motor_5 is missing
  // the unit sets the limp bit 0x2C0 b0.4 by itself (spec §6).
  frame.identifier = MOTOR5_ID;   // 0x480
  frame.data_length_code = 8;     // DLC 8
  frame.data[0] = 0xFE;           // [KM Motor_5 b0.6-7 MO5_Mp_Code = 3, b0.0-5 Abgastyp / OBD / DPF_reg flags (mux 3)] [REA 0AY: latch only when mux == 2 (0xCE78 = bit5 + 1)] from car testing: ASR 0x04 sets bit 4, 0x08 removes it
  frame.data[1] = 0x00;           // [KM b1.1 Vorgluehen, b1.2 E_Gas, b1.3 OBD_2, b1.4 Heissl, b1.5-7 Klima] [REA 0AY: not stored (1 byte; sw7543 stores 7, no reader)] was 0x00; 0x41? 0x43? tried
  frame.data[2] = 0x00;           // [KM MO5_Verbrauch low, 15 bit ul] [REA 0AY: not stored] was 0x00 no effect
  frame.data[3] = 0x00;           // [KM MO5_Verbrauch high + b3.7 UeberlVerb] [REA 0AY: not stored] was 0xFE no effect
  frame.data[4] = 0x00;           // [KM MO5_Luefter 0.4 %] [REA 0AY: not stored] was 0xFE no effect
  frame.data[5] = 0x00;           // [KM b5.0 HLeuchte .. b5.7 Interlock] [REA 0AY: not stored] was 0xFE no effect
  frame.data[6] = 0x00;           // [KM b6.0 Start, b6.1 Anlasser, b6.2 GRA_Hauptsch, b6.3 MO5_Momente, b6.4-7 Motortext] [REA 0AY: not stored; sw7543 reads b6.3 (scales Motor_2 max torque, audit F7)]
  frame.data[7] = MOTOR5_counter; // [KM Motor_5.CHECKSUM] (rolling counter, not a real XOR) [REA 0AY: not stored]
  MOTOR5_counter++;               // V11: was BRAKES1_counter++ (copy-paste bug, spec_gen5_0ay §7 #14)
  standaloneTx(frame);

  // PQ Getriebe_1 (0x440, DLC 8) - gearbox broadcast (vw_pq.dbc: Getriebe_1).
  // V11.00.0: NEW. The 0AY receives it (PDU 8, stores 2 bytes) and prefers it
  // for the gear: b1 bits 0-3 GE1_Zielgang (spec_gen5_0ay §3/§5, §7 #8). It
  // was never sent before, so the unit was missing a frame it supervises.
  // Values: K-matrix / DBC meanings, gear 0 = "P / disengaged" (no gear).
  // No counter / checksum check in the 0AY.
  frame.identifier = mGetriebe_1;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // [KM Getriebe_1 b0.0 GE1_Schaltung, b0.1 Sta_Schutz, b0.2 Klimakompr, b0.3-4 GE1_WK, b0.5-6 StSt_Info, b0.7 EGS_Anf] [REA 0AY: b0.3-4 getter (sig 100), unused]
  frame.data[1] = 0x00; // [KM b1.0-3 GE1_Zielgang, b1.4-7 GE1_Wahl_Pos] gear 0 [REA 0AY: Zielgang (sig 99) = gear 0xCF66 (7 -> 0xFF, 8/9 mapped)]
  frame.data[2] = 0x0A; // [KM GE1_Mrad_Mkurb 0.1] 1.0 [REA 0AY: not stored (2 bytes)]
  frame.data[3] = 0x00; // [KM GE1_Soll_Mo 0.39 %] [REA 0AY: not stored]
  frame.data[4] = 0x7F; // [KM GE1_Fahrwistd 0.249 -31.6] ~0 [REA 0AY: not stored]
  frame.data[5] = 0x00; // [KM b5.0-3 GE1_Notlauf, b5.4-5 GE1_Kuehlung, b5.7 Sta_OBD] [REA 0AY: not stored]
  frame.data[6] = 0x00; // [KM b6.0-2 Launch / Infobit / SleepInd, b6.3-6 COUNTER, b6.7 GE1_Fehlereintr] [REA 0AY: not stored]
  frame.data[7] = 0x00; // [KM GE1_WaVerl_Mo 0.39 %] [REA 0AY: not stored]
  standaloneTx(frame);
}

void Gen5_0AY_frames25()
{
  twai_message_t frame = {};
  // PQ Kombi_1 (0x320, DLC 8) - instrument-cluster broadcast (vw_pq.dbc: Kombi_1).
  // The 0AY reads b1.1 KO1_Handbremse (handbrake) only - spec_gen5_0ay §3.
  frame.identifier = mKombi_1;
  frame.data_length_code = 8;
  // V11.00.13: b2-b7 names corrected from vw_pq.dbc (the old ones were shifted).
  frame.data[0] = 0x24; // [KM Kombi_1 b0.0 Fahrertuer, b0.1 Fehlerstatus_Tank, b0.2 Oeldruck, b0.3 Dyn_Oeldruckwarnung, b0.4 Kuehlmittelmangel, b0.5 Heissleuchten_Vorwarnung, b0.6-7] [REA 0AY: not read]
  frame.data[1] = 0x00; // [KM b1.0-1 Bremsinfo (KMatrix: b1.1 KO1_Handbremse), b1.2 Ladekontroll, b1.3 Stellgliedtest, b1.7 Tankstop] [REA 0AY: b1.1 (sig 101) -> 0xCF54 (cal 0xC0B6CC = 1) -> state 4]
  if (handbrakeForHaldex())
  {
    frame.data[1] = frame.data[1] | (1 << 1); // b1.1 KO1_Handbremse - handbrake pulled
  }
  frame.data[2] = 0x1D; // [KM b2.0-6 Tankinhalt 1 l, b2.7 Tankwarnung] 29 l [REA 0AY: not stored (2 bytes)]
  frame.data[3] = 0xB9; // [KM b3.0 Signalquelle_Geschwindigkeit, b3.1-7 Geschwindigkeit__Kombi_1_ low 0.01 km/h] [REA 0AY: not stored]
  frame.data[4] = 0x07; // [KM Geschwindigkeit__Kombi_1_ high] ~9.9 km/h [REA 0AY: not stored]
  frame.data[5] = 0x42; // [KM b5.0-1 Frei, b5.2 ADR_Summer, b5.3 Zeitluecke, b5.4-5 Blinker, b5.6-7 Angezeigte_Geschwindigkeit low 0.32 km/h] [REA 0AY: not stored]
  frame.data[6] = 0x09; // [KM Angezeigte_Geschwindigkeit high] [REA 0AY: not stored]
  frame.data[7] = 0x81; // [KM Frei_Kombi_1_3] [REA 0AY: not stored]
  standaloneTx(frame);

  // PQ Kombi_3 (0x520, DLC 8) - cluster mux data / stand time / odometer (vw_pq.dbc: Kombi_3).
  // V11.00.13: byte names corrected from vw_pq.dbc (the old ones were shifted).
  frame.identifier = mKombi_3;
  frame.data_length_code = 8;
  frame.data[0] = 0x60; // [KM Kombi_3 b0 mux data (mux 1: b0.0-3 Derivat, b0.4-7 Generation)] [REA 0AY: (b0 + b1 bits 0-5) & 0xFFF used only when mux == 2 (sig 88)]
  frame.data[1] = 0x43; // [KM b1.0-5 mux data, b1.6-7 Kombi_Multiplex_Code] code 1 [REA 0AY: mux code (sig 87) - 1 here, so ignored]
  frame.data[2] = 0x01; // [KM b2.0-3 Schluesselinfo, b2.4-7 Frei_Kombi_3_1] [REA 0AY: not read]
  frame.data[3] = 0x10; // [KM Standzeit low, 15 bit x 4 s] [REA 0AY: stand time (sig 89) -> 0xCE74, latched once at start-up]
  frame.data[4] = 0x66; // [KM b4.0-6 Standzeit high, b4.7 Fehlerstatus_Standzeit] [REA 0AY: bits 0-6 stand time]
  frame.data[5] = 0xF1; // [KM Kilometerstand low, 20 bit km] [REA 0AY: not stored (5 bytes)]
  frame.data[6] = 0x03; // [KM Kilometerstand mid] [REA 0AY: not stored]
  frame.data[7] = 0x02; // [KM b7.0-3 Kilometerstand high, b7.4-7 Frei_Kombi_3_2] [REA 0AY: not stored]
  standaloneTx(frame);
}

void Gen5_0AY_frames100()
{
  twai_message_t frame = {};
  // PQ Gate_Komf_1 (0x390, DLC 8) - gateway-comfort broadcast (vw_pq.dbc: Gate_Komf_1).
  frame.identifier = mGate_Komf_1;
  frame.data_length_code = 8;
  // V11.00.13: byte names corrected from vw_pq.dbc (b3.4 is GK1_Rueckfahr).
  frame.data[0] = 0x03; // [KM Gate_Komf_1 b0.0 GK1_Sta_RDK_Warn, b0.1 GK1_Sta_Anhaen, b0.2-5 light / door status, b0.7 SleepAckn] [REA 0AY: not read]
  frame.data[1] = 0x11; // [KM b1.0-3 GK1_CharismaModus (mux 1), b1.4-7 GK1_SamFktNr (mux)] [REA 0AY: not read]
  frame.data[2] = 0x58; // [KM b2.0 Fa_Tuerkont, b2.1 GK1_RueckfahrSch, b2.2 ELV, b2.3 Kessy_2, b2.4 Stdhzg, b2.5 SH_Verbau, b2.6 ParkFrontWi, b2.7 KW_Warm] [REA 0AY: not read]
  frame.data[3] = 0x00; // [KM b3.4 GK1_Rueckfahr; b3.0/3.2-3 remote start / boot, b3.5-7 GK1_BrLi] not reversing [REA 0AY: b3.4 (sig 128) = 1 -> gear forced 0xFF]
  frame.data[4] = 0x40; // [KM b4.0-7 BLS_ILM / EDC / Blinker_li/re / def_P_verr / Fernlicht / b4.6 GK1_Sta_Licht2 / Sta_LSM] [REA 0AY: not stored (4 bytes)]
  frame.data[5] = 0x00; // [KM b5 GK1_Count_Anhaen / BSK / BLS_AAG / Anhaenger bits] [REA 0AY: not stored]
  frame.data[6] = 0x01; // [KM b6.0 GK1_Abblendlicht .. b6.7 Warnblk_Status] [REA 0AY: not stored]
  frame.data[7] = 0x08; // [KM b7.0 SH_laeuft .. b7.3 GK1_Bremslicht (= 1 here) .. b7.7] [REA 0AY: not stored]
  standaloneTx(frame);

  // PQ Bremse_11 (0x5B7, DLC 8) - extended brake frame (vw_pq.dbc: Bremse_11).
  // NOT received by the 0AY (no message object - spec_gen5_0ay §1): its block
  // defaults OFF in standalone (V11.00.0), so standaloneTx() skips it.
  frame.identifier = BRAKES11_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // [KM Bremse_11.CHECKSUM] [REA 0AY: not received]
  frame.data[1] = 0XC0; // [KM b1.0-3 COUNTER, b1.5 B11_HydHalten, b1.6-7 B11_Br_StSt_Info] [REA 0AY: not received]
  frame.data[2] = 0x00; // [KM B11_OBD_Nib_VL / _VR] [REA 0AY: not received]
  frame.data[3] = 0x00; // [KM B11_OBD_Nib_HL / _HR] [REA 0AY: not received]
  frame.data[4] = 0x00; // [KM b4.0 B11_EPB_Steller_akt, b4.1 _gue] [REA 0AY: not received]
  frame.data[5] = 0x00; // [KM -] [REA 0AY: not received]
  frame.data[6] = 0x00; // [KM -] [REA 0AY: not received]
  frame.data[7] = 0x00; // [KM -] [REA 0AY: not received]
  standaloneTx(frame);

  // ---------------------------------------------------------------------------
  // PQ Systeminfo_1 / mSysteminfo_1 (0x5D0, DLC 8) - gateway vehicle-identity
  // broadcast.  Source: PQ35_46 ACAN KMatrix V5.20.6F, cycle 100 ms, TO 1000 ms.
  // Required by haldex DTC 16659 (Sys_01 / Systeminfo_1 timeout) on 0AY.
  // The 0AY reads only b6.6 / b6.7 (not in the clutch path) - b6 = 0 here.
  // Layout (KMatrix bytes 1-based; byte 1 = data[0]):
  //   data[0]: status bits (Verbauinfo, Lenker, Tueren, ...) - 0 = defaults
  //   data[1]: low nibble Fzg_Klasse, high nibble Fzg_Marke (Audi/VW/...)
  //   data[2]: low nibble Fzg_Derivat, high nibble Fzg_Gen (PQ35=3)
  //   data[3]: low nibble Fzg_Index (init 0xF), high nibble misc status
  //   data[4]: low nibble Komf_NV (init 9), high nibble Komf_HV (init 3)
  //   data[5]: low nibble Ant_NV (init 9), high nibble Ant_HV (init 5)
  //   data[6]: misc status (Relais_FAS, ELV_Lock, NWDF...) - 0
  //   data[7]: misc status (Notbrems_Status...) - 0
  // No checksum, no rolling counter in this message.
  // ---------------------------------------------------------------------------
  frame.identifier = mSysteminfo_1;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // [KM Systeminfo_1 b0.0-7 CAN_Extern / Diagnose / Sleep flags, Verbauinformation_gueltig] [REA 0AY: not read]
  frame.data[1] = 0x24; // [KM b1.0-3 Fahrzeug_Klasse, b1.4-7 Fahrzeug_Marke_2] [REA 0AY: not read]
  frame.data[2] = 0x35; // [KM b2.0-3 Fahrzeug_Derivat, b2.4-7 Fahrzeug_Generation] [REA 0AY: not read]
  frame.data[3] = 0x0F; // [KM b3.0-3 Fahrzeug_Index, b3.4 Rechtslenker, b3.5 Viertuerer] [REA 0AY: not read]
  frame.data[4] = 0x39; // [KM CAN_Stand_4_1_Komfort_Daten_Neb / _Hau] [REA 0AY: not read]
  frame.data[5] = 0x59; // [KM CAN_Stand_4_1_Antrieb_Daten_Neb / _Hau] [REA 0AY: not read]
  frame.data[6] = 0x00; // [KM -] (KMatrix: FAS/ELV/QRS/NWDF flags) [REA 0AY: b6.6 / b6.7 read (sig 127/126, FUN_c4a780 / FUN_c4d13a, not the clutch path)]
  frame.data[7] = 0x00; // [KM -] [REA 0AY: not stored (7 bytes)]
  standaloneTx(frame);
}

void Gen5_0AY_frames200()
{
  twai_message_t frame = {};
  // PQ Kombi_2 (0x420, DLC 8) - cluster temps (vw_pq.dbc: Kombi_2).
  // NOT received by the 0AY (no message object - spec_gen5_0ay §1): its block
  // defaults OFF in standalone (V11.00.0), so standaloneTx() skips it.
  // V11.00.13: byte names corrected from vw_pq.dbc (the old ones were shifted).
  frame.identifier = mKombi_2;
  frame.data_length_code = 8;
  frame.data[0] = 0x4C; // [KM Kombi_2 b0.0-2 Fehlerstatus Aussen/Oel/Kuehlm, b0.3 Anhaenger_erkannt, b0.7 Fehlerspeichereintrag] [REA 0AY: not received]
  frame.data[1] = 0x86; // [KM Aussentemperatur_gefiltert 0.5 -50 degC] [REA 0AY: not received]
  frame.data[2] = 0x85; // [KM Aussentemp__ungefiltert 0.5 -50 degC] [REA 0AY: not received]
  frame.data[3] = 0x00; // [KM Oeltemperatur_4_1 1 -60 degC] [REA 0AY: not received]
  frame.data[4] = 0x00; // [KM Kuehlmitteltemp__4_1 0.75 -48 degC] [REA 0AY: not received]
  frame.data[5] = 0x30; // [KM b5.0-6 Klemme_58d %, b5.7 Fehlerstatus] [REA 0AY: not received]
  frame.data[6] = 0xFF; // [KM b6.0-6 Klemme_58s %, b6.7 Fehlerstatus] [REA 0AY: not received]
  frame.data[7] = 0x04; // [KM Frei_Kombi_2_2] [REA 0AY: not received]
  standaloneTx(frame);
}

void Gen5_0AY_frames1000()
{
  twai_message_t frame = {};
  // PQ Diagnose_1 (0x7D0, DLC 8) - diagnostic timestamp broadcast (vw_pq.dbc: Diagnose_1).
  // REA 0AY: km (20 bit) + date / time fields 70-77 read only by FUN_c1c3b0
  // (fault-memory environment record, INF) - spec_gen5_0ay §3, stores 8 bytes.
  frame.identifier = mDiagnose_1;
  frame.data_length_code = 8;
  frame.data[0] = 0x26;                // [KM Diagnose_1.DI1_VerlernZaehl] [REA 0AY: fault-memory record]
  frame.data[1] = 0xF2;                // [KM DI1_km_Stand low, 20 bit km] [REA 0AY: km]
  frame.data[2] = 0x03;                // [KM DI1_km_Stand mid] [REA 0AY: km]
  frame.data[3] = 0x12;                // [KM b3.0-3 DI1_km_Stand high, b3.4-7 DI1_Jahr low (+2000)] [REA 0AY: km / date]
  frame.data[4] = 0x70;                // [KM b4.0-2 DI1_Jahr high, b4.3-6 DI1_Monat, b4.7 DI1_Tag low] [REA 0AY: date]
  frame.data[5] = 0x19;                // [KM b5.0-3 DI1_Tag high, b5.4-7 DI1_Stunde low] [REA 0AY: date / time]
  frame.data[6] = 0x25;                // [KM b6.0 DI1_Stunde high, b6.1-6 DI1_Minute, b6.7 DI1_Sekunde low] [REA 0AY: time]
  frame.data[7] = mDiagnose_1_counter; // [KM b7.0-4 DI1_Sekunde high, b7.6 KM_Stand_alt, b7.7 Zeit_alt] rolling 0..31 [REA 0AY: time]
  standaloneTx(frame);
  mDiagnose_1_counter++;
  if (mDiagnose_1_counter > 0x1F)
  {
    mDiagnose_1_counter = 0;
  }
}

// =============================================================================
// Gen42 (Ford Focus RS / CGEA1.2 PT-CAN) Standalone Frames
// =============================================================================
// Simulates the minimum Ford vehicle PT-CAN traffic required to keep the Haldex
// AWD module (TCCM) operational and to modulate clutch engagement.
//
// Engagement control (dual signal):
//   1. FORD_GEN42_WHEEL_SPEEDS_ID (0x4B0) — Four wheel speeds sent by ABS/ESC.
//      Front wheels are inflated above rear by a lock-target-proportional delta.
//      Encoding: raw=0x2710 (10 000) = 0 km/h; (raw − 10 000) / 100 km/h.
//      At 100 % lock target the front delta reaches ~1 000 counts (~10 km/h
//      simulated slip), requesting maximum engagement. [WHEEL_SPEEDS]
//   2. FORD_GEN42_DRIVE_MODE_ID (0x190) D3 — drive-mode byte from GWM/PCM.
//      0x60 = power/demand (engagement requested), 0x20 = idle/off.
//      The Haldex is proactive/predictive and engages before wheel slip occurs.
//      [PROVISIONAL_0x190]
//
// Bus usage: all frames transmitted on twaiBus1 (vehicle PT-CAN → Haldex).
// Haldex responds with FORD_GEN42_AWD_STATUS_ID (0x260) on twaiBus0:
//   AwdLck_Tq_Actl D0:D1 b15|12 = actual clutch torque (1 Nm/LSB).
//   [Information4x4_CG1] — parsed later in parseCANChs (deferred).
// =============================================================================

void Gen42_frames10()
{
  twai_message_t frame = {};
  frame.extd = 0;
  frame.rtr = 0;

  // FORD_GEN42_WHEEL_SPEEDS_ID 0x4B0 (Bus1, 10 ms) — ABS/ESC wheel speeds.
  // FL / FR / RL / RR, each 16-bit big-endian. Encoding: (raw − 10 000) / 100 km/h.
  // Raw=0x2710 (10 000) = 0 km/h standing reference.
  // Inflate front wheels above rear to request proportional AWD engagement.
  // lockTarget is refreshed every 100 ms by the frames100 dispatcher.
  // At 0 % lock: all four wheels at 0x2710 (balanced, no engagement requested).
  // At 100 % lock: front +~1 000 counts (~10 km/h simulated slip). [WHEEL_SPEEDS]
  {
    const uint8_t spdAdj = getLockTargetAdjustedValue(0xFA, false);
    const uint16_t frontDelta = (uint16_t)spdAdj * 4U; // 0 to ~1 000 counts
    const uint16_t frontSpd = 0x2710U + frontDelta;    // FL / FR speed word
    const uint16_t rearSpd = 0x2710U;                   // RL / RR at reference
    frame.identifier = FORD_GEN42_WHEEL_SPEEDS_ID;
    frame.data_length_code = 8;
    frame.data[0] = (uint8_t)(frontSpd >> 8);    // FL high byte
    frame.data[1] = (uint8_t)(frontSpd & 0xFFU); // FL low byte
    frame.data[2] = (uint8_t)(frontSpd >> 8);    // FR high byte
    frame.data[3] = (uint8_t)(frontSpd & 0xFFU); // FR low byte
    frame.data[4] = (uint8_t)(rearSpd >> 8);     // RL high byte
    frame.data[5] = (uint8_t)(rearSpd & 0xFFU);  // RL low byte
    frame.data[6] = (uint8_t)(rearSpd >> 8);     // RR high byte
    frame.data[7] = (uint8_t)(rearSpd & 0xFFU);  // RR low byte
    standaloneTx(frame);
  }

  // FORD_GEN42_TORQUE_FLAGS_ID 0x200 (Bus1, 10 ms) — PCM torque and flags.
  // D0:D1 = PrplWhlTot_Tq_Actl (4 Nm/LSB, offset −131072 Nm, Motorola b7|16).
  // D2:D3 = PrplWhlTot_Tq_LimMn (minimum propulsion torque limit).
  // D4:D5 = PrplWhlTot_Tq_Rq    (requested propulsion torque).
  // D6 b6:b7 = BrkOnOffSwtch_D_Actl, D7 b0:b6 = ACCompressorDisp (%). [TorqueDataEngFlags]
  frame.identifier = FORD_GEN42_TORQUE_FLAGS_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x28U + getLockTargetAdjustedValue(0xD7, false); // Tq_Actl high byte: 0x28→0xFF (proportional to lockTarget)
  frame.data[1] = 0xE0;                                                // Tq_Actl low byte (static)
  frame.data[2] = 0x27U + getLockTargetAdjustedValue(0xD8, false); // Tq_LimMn high byte: 0x27→0xFF
  frame.data[3] = 0x10;                                                // Tq_LimMn low byte (static)
  frame.data[4] = 0x27U + getLockTargetAdjustedValue(0xD8, false); // Tq_Rq high byte: 0x27→0xFF
  frame.data[5] = 0x10;                                                // Tq_Rq low byte (static)
  frame.data[6] = 0x88;
  frame.data[7] = 0x00; // flags / AC displacement
  standaloneTx(frame);

  // FORD_GEN42_ENG_SPD_THROTTLE_ID 0x201 (Bus1, 10 ms) — PCM engine/vehicle speed.
  // D0:D1 = EngAout_N_Actl (2 rpm/LSB, Motorola b7|13).
  // D2:D3 = Veh_V_ActlEng (0.01 kph/LSB, vehicle speed from engine estimate).
  // D4:D5 = ApedPos_Pc_ActlArb (0.1 %/LSB, arbitrated accelerator position).
  // D6 b4:b5 = ApedPosPcActl_D_Qf, D6 b2 = Autostart_B_Stat. [EngVehicleSpThrottle_CG1]
  frame.identifier = FORD_GEN42_ENG_SPD_THROTTLE_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;
  frame.data[1] = 0x00; // EngAout_N = 0 rpm
  frame.data[2] = 0x40;
  frame.data[3] = 0x00; // Veh_V_ActlEng = 0 kph
  frame.data[4] = 0x00;
  frame.data[5] = 0x00; // ApedPos = 0 %
  frame.data[6] = 0x00;
  frame.data[7] = 0x80; // quality flags / static
  standaloneTx(frame);

  // FORD_GEN42_POWERTRAIN_DATA6_ID 0x205 (Bus1, 10 ms) — PCM fuel/torque model data.
  // D4:D5 = provisional fuel-related word (rapidly varying in logs, not a tank sensor).
  // D6 = provisional fuel-related byte. [PowertrainData_6 / PROVISIONAL_0x205]
  frame.identifier = FORD_GEN42_POWERTRAIN_DATA6_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x02;
  frame.data[1] = 0x8B;
  frame.data[2] = 0x01;
  frame.data[3] = 0xFF;
  frame.data[4] = 0x02;
  frame.data[5] = 0x18; // provisional fuel-related word
  frame.data[6] = 0x01;
  frame.data[7] = 0xFF;
  standaloneTx(frame);

  // FORD_GEN42_TRANS_GEAR_DATA2_ID 0x231 (Bus1, 10 ms) — TCM torque/gear status.
  // byte[0]        = MtrGen1AoutTqRq_No_Cs  (8-bit checksum).  Fixed at 0x01.
  // byte[1] b7:b4  = MtrGen1AoutTqRq_No_Cnt (4-bit counter). Fixed at 0x5.
  // Both are frozen because the Focus RS has a MANUAL gearbox — the TCM payload
  // never changes, so the Ford CGEA No_Cnt counter never increments.
  // byte[1] b3:b0 / byte[2-5] = MtrGen1Aout_Tq_Rq (b53|14, 0.1 Nm/LSB, −800 Nm).
  // byte[6] = TrnMil_D_Rq / EngExhBrkTq_Pc_Rq. DLC=7. [TransGearData_2]
  // Log-verified payload: 01 52 64 00 01 00 80
  frame.identifier = FORD_GEN42_TRANS_GEAR_DATA2_ID;
  frame.data_length_code = 7;
  frame.data[0] = 0x01;
  frame.data[1] = 0x52; // No_Cs=0x01; No_Cnt=5 | 0x02
  frame.data[2] = 0x64;
  frame.data[3] = 0x00; // MtrGen1Aout_Tq_Rq
  frame.data[4] = 0x01;
  frame.data[5] = 0x00;                                                          // TrnMsgTxt_D_Rq / static
  frame.data[6] = 0x80U | (0x1FU + getLockTargetAdjustedValue(0x20, false)); // MtrGen1Aout_Tq_Rq upper 6 bits: 0x1F (0 Nm) → 0x3F (~+813 Nm)
  standaloneTx(frame);

  // FORD_GEN42_BODY_INFO4_ID 0x240 (Bus1, 10 ms) — BCM body information (2-byte variant).
  // DBC superset (8 B) includes EngOff_T_Actl (seconds), AmbTempImpr (0.25 °C/LSB, −128 °C).
  // Focus RS logs consistently show only 2 bytes: 0x00 0x40. [Body_Information_4_CG1]
  frame.identifier = FORD_GEN42_BODY_INFO4_ID;
  frame.data_length_code = 2;
  frame.data[0] = 0x00;
  frame.data[1] = 0x40;
  standaloneTx(frame);
}

void Gen42_frames20()
{
  twai_message_t frame = {};
  frame.extd = 0;
  frame.rtr = 0;

  // FORD_GEN42_ACCEL_BRAKE_STATUS_ID 0x080 (Bus1, 20 ms) — PCM accel/brake status.
  // D0:D1 = AccelPedal_Pc (10-bit Motorola b6|10, 0.1 %/LSB): D0[6:5]=raw[9:8], D1[7:0]=raw[7:0].
  //         Scaled 0–100 % proportional to lockTarget. D0[2:0]=0x7 constant, BrakePedal_D=0.
  // D2:D3 = static 0x7530.  D4:D5 = static 0x0130.
  // D7 = 8-bit rolling counter (shared with 0x20F D8 and 0x211 D8).
  // DBC signals: AccelPedal_Pc b6|10 (0.1 %/LSB Motorola), BrakePedal_D b4|2. DLC=7. [ACCEL_BRAKE_STATUS]
  frame.identifier = FORD_GEN42_ACCEL_BRAKE_STATUS_ID;
  frame.data_length_code = 7;
  {
    const uint16_t rawPedal = (uint16_t)getLockTargetAdjustedValue(0xFA, false) * 4U; // 0–1000 = 0–100.0 %
    frame.data[0] = 0x07U | (uint8_t)(((rawPedal >> 8) & 0x03U) << 5);                    // AccelPedal[9:8]; BrakePedal=0; bit2:0=0x7
    frame.data[1] = (uint8_t)(rawPedal & 0xFFU);                                          // AccelPedal[7:0]
  }
  frame.data[2] = 0x75;
  frame.data[3] = 0x30; // D3:D4 = 0x7530 (constant)
  frame.data[4] = 0x01;
  frame.data[5] = 0x30;               // D5:D6 = 0x0130 (constant)
  frame.data[6] = gen42MainCounter; // D7 = 8-bit rolling counter
  standaloneTx(frame);

  // FORD_GEN42_ENGINE_DATA_ID 0x090 (Bus1, 20 ms) — PCM engine speed data.
  // D1 high nibble = 4-bit rolling counter 0x0-0xF (D1 = (counter << 4) | 0x07).
  // DBC signal: EngAout_N_Actl b35|13 (2 rpm/LSB, Motorola). D7:D8 = 0x07D0 (constant). [ENGINE_DATA]
  frame.identifier = FORD_GEN42_ENGINE_DATA_ID;
  frame.data_length_code = 8;
  frame.data[0] = (uint8_t)((gen42Frame090Nibble << 4) | 0x07U); // D1: 4-bit counter | 0x07
  frame.data[1] = 0x05;                                       // D2 (constant)
  frame.data[2] = 0x57;                                       // D3 (constant — EngAout_N upper bits)
  frame.data[3] = 0x07;                                       // D4 (constant)
  {
    const uint16_t V_rpm = 400U + (uint16_t)getLockTargetAdjustedValue(0xFA, false) * 10U; // 400–2900 → 800–5800 RPM
    frame.data[4] = (uint8_t)(0xC0U | ((V_rpm >> 8) & 0x1FU));                                 // D5: upper 5 bits of V; flags 0b110 preserved
    frame.data[5] = (uint8_t)(V_rpm & 0xFFU);                                                  // D6: lower 8 bits of V
  }
  frame.data[6] = 0x07;
  frame.data[7] = 0xD0; // D7:D8 = 0x07D0 (constant)
  standaloneTx(frame);

  // FORD_GEN42_BRAKE_DATA_ID 0x20F (Bus1, 20 ms) — ABS provisional brake data.
  // D1:D2 = 0x7530 (constant). D3:D4 = 0x2710 (wheel speed reference, constant).
  // D5 = 0x40 (constant). D6 = gen42Frame20FD6 (+0x10 per frame, mod 256).
  // D7 = 0xC8 − D6 (complement; D6 + D7 always = 0xC8 = 200 decimal).
  // D8 = shared 8-bit rolling counter (same as 0x080 D7 and 0x211 D8).
  // D2:D3 = provisional brake pressure: 0x0FB0 observed at full brake.
  frame.identifier = FORD_GEN42_BRAKE_DATA_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x75;
  frame.data[1] = 0x30; // D1:D2 = 0x7530 (constant)
  frame.data[2] = 0x27;
  frame.data[3] = 0x10;                            // D3:D4 = 0x2710 (constant)
  frame.data[4] = 0x40;                            // D5 (constant)
  frame.data[5] = gen42Frame20FD6;                    // D6: +0x10 per frame, mod 256
  frame.data[6] = (uint8_t)(0xC8U - gen42Frame20FD6); // D7: complement (D6 + D7 = 0xC8)
  frame.data[7] = gen42MainCounter;              // D8: shared 8-bit rolling counter
  standaloneTx(frame);

  // FORD_GEN42_DESIRED_TORQ_BRK_ID 0x211 (Bus1, 20 ms) — ABS desired torque/brake.
  // D0:D1 = PrplWhlTot_Tq_RqMx (4 Nm/LSB, offset −131072 Nm, Motorola b7|16).
  // D2:D3 b23|12 = RearDiffLck_Tq_RqMx (rear diff lock max torque request, Nm).
  // D3 b3 = AbsActv_B_Actl (ABS active flag). D4:D5 = static 0x4848.
  // D5:D6 b47|10 = VehLongOvrGnd_A_Est (0.035 m/s², offset −17.9 m/s²).
  // D8 = shared 8-bit rolling counter. [DesiredTorqBrk_CG1]
  frame.identifier = FORD_GEN42_DESIRED_TORQ_BRK_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0xFF;
  frame.data[1] = 0xFE;                                        // PrplWhlTot_Tq_RqMx
  frame.data[2] = getLockTargetAdjustedValue(0xFF, false); // RearDiffLck_Tq_RqMx upper 8 bits: 0→255 Nm (proportional to lockTarget)
  frame.data[3] = 0x00;                                        // RearDiffLck_Tq_RqMx lower 4 bits = 0; AbsActv_B_Actl = 0
  frame.data[4] = 0x48;
  frame.data[5] = 0x48;               // VehLongOvrGnd_A_Est (static)
  frame.data[6] = 0x00;               // constant
  frame.data[7] = gen42MainCounter; // D8: shared 8-bit rolling counter
  standaloneTx(frame);

  // FORD_GEN42_DRIVE_MODE_ID 0x190 (Bus1, 20 ms) — GWM/PCM drive mode status.
  // D3 = drive-mode byte: 0x60 when engagement is requested, 0x20 at idle/off.
  //      This is the primary proactive signal — Haldex engages BEFORE slip occurs.
  // D2 = 0x51 (engine speed proxy, ~1000 rpm idle; varies with throttle in logs).
  // D6 = 4-bit nibble counter 0x00-0x0F (gen42Frame190Counter).
  // D7:D8 = provisional steering angle word (0x8000 = 0°, per guide). [PROVISIONAL_0x190]
  {
    const uint8_t driveMode = (lockTarget > 0.0f) ? 0x60U : 0x20U;
    frame.identifier = FORD_GEN42_DRIVE_MODE_ID;
    frame.data_length_code = 8;
    frame.data[0] = 0x00;                      // D1 (constant)
    frame.data[1] = 0x51;                      // D2: engine speed proxy (~1000 rpm idle)
    frame.data[2] = driveMode;                // D3: drive-mode (0x20=idle, 0x60=demand)
    frame.data[3] = 0x80;                      // D4 (constant)
    frame.data[4] = 0x04;                      // D5 (constant — drive-engaged status)
    frame.data[5] = gen42Frame190Counter & 0x0FU; // D6: 4-bit nibble counter 0x00-0x0F
    frame.data[6] = 0x00;                      // D7 (constant)
    frame.data[7] = 0x3C;                      // D8 (constant)
    standaloneTx(frame);
  }

  // Advance counters after all 20 ms frames are sent.
  gen42MainCounter++;                                            // free-running 8-bit (0x080/0x20F/0x211 shared)
  gen42Frame090Nibble = (uint8_t)((gen42Frame090Nibble + 1U) & 0x0FU);   // 4-bit 0-15
  gen42Frame20FD6 = (uint8_t)((gen42Frame20FD6 + 0x10U) & 0xFFU);        // +0x10, mod 256
  gen42Frame190Counter = (uint8_t)((gen42Frame190Counter + 1U) & 0x0FU); // 4-bit 0-15
}

void Gen42_frames100()
{
  twai_message_t frame = {};
  frame.extd = 0;
  frame.rtr = 0;

  // FORD_GEN42_WARN_STATUS_ID 0x212 (Bus1, 100 ms) — IPC/ABS warning status.
  // Provisional: D3/D5 interact for DSC/TCS warning; D4 b5=brake warn, D4 b3=ABS warn.
  // Log-verified payload: F8 00 38 80 15 00 00 00 [PROVISIONAL_0x212]
  frame.identifier = FORD_GEN42_WARN_STATUS_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0xF8;
  frame.data[1] = 0x00; // D1: status byte, D2 (zero)
  frame.data[2] = 0x38;
  frame.data[3] = 0x80; // D3: DSC/TCS candidate, D4: brake/ABS candidates
  frame.data[4] = 0x15;
  frame.data[5] = 0x00; // D5: TCS candidate
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;
  standaloneTx(frame);

  // FORD_GEN42_COUNTER_275_ID 0x275 (Bus1, 100 ms) — unknown rolling counter frame.
  // D1 steps +0x20 per 100 ms cycle: 0x80, 0xA0, 0xC0, 0xE0, 0x00, 0x20, 0x40, 0x60, (wrap).
  // Not matched in available DBCs. DLC=3.
  frame.identifier = FORD_GEN42_COUNTER_275_ID;
  frame.data_length_code = 3;
  frame.data[0] = gen42Frame275Counter; // D1: rolling counter (+0x20 per frame)
  frame.data[1] = 0x00;              // D2 (constant)
  frame.data[2] = 0xFF;              // D3 (constant)
  standaloneTx(frame);

  gen42Frame275Counter = (uint8_t)((gen42Frame275Counter + 0x20U) & 0xFFU);
}

void Gen42_frames200()
{
  twai_message_t frame = {};
  frame.extd = 0;
  frame.rtr = 0;

  // FORD_GEN42_UNKNOWN_270_ID 0x270 (Bus1, 200 ms) — unknown 200 ms heartbeat.
  // D1 = 0x81/0x82 (slow-varying in log); D2 = 0x01 constant. Not in available DBCs.
  // Log-verified most common payload: 81 01 00 00 00 00 00 00
  frame.identifier = FORD_GEN42_UNKNOWN_270_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x81;
  frame.data[1] = 0x01; // D1: status/counter, D2: 0x01
  frame.data[2] = 0x00;
  frame.data[3] = 0x00;
  frame.data[4] = 0x00;
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;
  standaloneTx(frame);

  // FORD_GEN42_UNKNOWN_280_ID 0x280 (Bus1, 200 ms) — unknown 200 ms heartbeat.
  // D1:D2 = 0x00 0x00; D3 = 0xB0 (most common observed; varies slowly in log);
  // D4-D8 = 0x00. Not matched in available DBCs.
  // Log-verified most common payload: 00 00 B0 00 00 00 00 00
  frame.identifier = FORD_GEN42_UNKNOWN_280_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;
  frame.data[1] = 0x00;
  frame.data[2] = 0xB0;
  frame.data[3] = 0x00; // D3: 0xB0 most common; slowly varies
  frame.data[4] = 0x00;
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;
  standaloneTx(frame);

  // FORD_GEN42_FOUR_BY_FOUR_SW_ID 0x460 (Bus1, 200 ms) — GWM 4WD switch status.
  // D1=0x00, D2=0x50 (AWD-auto / selector mode), D3-D8=0x00. [FourByFourSwitchData_FD1]
  // Log-verified payload: 00 50 00 00 00 00 00 00
  frame.identifier = FORD_GEN42_FOUR_BY_FOUR_SW_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;
  frame.data[1] = 0x50; // D2=0x50: AWD auto selector position
  frame.data[2] = 0x00;
  frame.data[3] = 0x00;
  frame.data[4] = 0x00;
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;
  standaloneTx(frame);
}

void Gen42_frames1000()
{
  twai_message_t frame = {};
  frame.extd = 0;
  frame.rtr = 0;

  // FORD_GEN42_PTRAIN_DATA1_ID 0x420 (Bus1, 1000 ms) — PCM powertrain data 1.
  // D1 alternates 0x62/0x63 each second.
  // D7:D8 cluster drive-mode pattern (from guide): Normal=0x10C4, Sport=0x11CC, Drift=0x12C4.
  // DBC signals: CluPdlPos_Pc_Meas D0:D1 (0.1 %/LSB), TrnTotLss_Tq_Est D3 (0.5 Nm/LSB),
  //              FuelAlcohol_Pc D7 (0.39 %/LSB). Launch Control Status: D7 b5 per guide.
  // [PowertrainData_1_CG1 / PROVISIONAL_0x420]
  {
    static uint8_t gen42Frame420Toggle = 0U;
    frame.identifier = FORD_GEN42_PTRAIN_DATA1_ID;
    frame.data_length_code = 8;
    frame.data[0] = (gen42Frame420Toggle & 1U) ? 0x63U : 0x62U; // D1 toggle
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x5D;
    frame.data[4] = 0x43; // TrnTotLss_Tq_Est / static
    frame.data[5] = 0x10;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    standaloneTx(frame);
    gen42Frame420Toggle++;
  }

  // FORD_GEN42_UNKNOWN_428_ID 0x428 (Bus1, 1000 ms) — provisional StrgWheel_PolicePkg.
  // Possible police-package steering wheel data (CGEA2011); not confirmed for Focus RS.
  // Static payload. DLC=4.
  frame.identifier = FORD_GEN42_UNKNOWN_428_ID;
  frame.data_length_code = 4;
  frame.data[0] = 0xFF;
  frame.data[1] = 0x7D;
  frame.data[2] = 0x6A;
  frame.data[3] = 0x31;
  standaloneTx(frame);

  // FORD_GEN42_CLUSTER_INFO_ID 0x430 (Bus1, 1000 ms) — IPC cluster information fragment.
  // DBC 8-byte superset includes OdometerMasterValue D2:D4 (km), DrvSlipCtlMde_D_Rq.
  // Focus RS logs show only DLC=2: 0xBC 0x12 (static). [Cluster_Information]
  frame.identifier = FORD_GEN42_CLUSTER_INFO_ID;
  frame.data_length_code = 2;
  frame.data[0] = 0xBC;
  frame.data[1] = 0x12;
  standaloneTx(frame);

  // FORD_GEN42_CLUSTER_INFO3_ID 0x433 (Bus1, 1000 ms) — IPC cluster information 3.
  // D0:D1 = FuelLvl_Pc_Dsply b7|10 (0.109 %/LSB, −5.22 % offset).
  // D4 b6 = HILL_DESC_SW (hill-descent switch). D7 b0:b3 = camera config bits.
  // Gear-state bitmask in D0-D3 per guide (gear selector position bits). [Cluster_Information_3_CG1]
  frame.identifier = FORD_GEN42_CLUSTER_INFO3_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;
  frame.data[1] = 0x01;
  frame.data[2] = 0x34;
  frame.data[3] = 0x01; // FuelLvl_Pc / gear state bits
  frame.data[4] = 0x00;
  frame.data[5] = 0x20; // HILL_DESC_SW, camera config
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;
  standaloneTx(frame);

  // FORD_GEN42_UNKNOWN_4F0_ID 0x4F0 (Bus1, 1000 ms) — unknown slow heartbeat.
  // Static content in all logs. Not matched in available DBCs. DLC=6.
  // Log-verified payload: 19 2C 22 50 23 45
  frame.identifier = FORD_GEN42_UNKNOWN_4F0_ID;
  frame.data_length_code = 6;
  frame.data[0] = 0x19;
  frame.data[1] = 0x2C;
  frame.data[2] = 0x22;
  frame.data[3] = 0x50;
  frame.data[4] = 0x23;
  frame.data[5] = 0x45;
  standaloneTx(frame);

  // FORD_GEN42_UNKNOWN_4F1_ID 0x4F1 (Bus1, 1000 ms) — unknown slow heartbeat.
  // Static content in all logs. Not matched in available DBCs.
  // Log-verified payload: 41 01 90 01 90 64 61 61
  frame.identifier = FORD_GEN42_UNKNOWN_4F1_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x41;
  frame.data[1] = 0x01;
  frame.data[2] = 0x90;
  frame.data[3] = 0x01;
  frame.data[4] = 0x90;
  frame.data[5] = 0x64;
  frame.data[6] = 0x61;
  frame.data[7] = 0x61;
  standaloneTx(frame);

  // FORD_GEN42_VIN_ASCII_ID 0x4F3 (Bus1, 1000 ms) — static ASCII build-ID fragment.
  // Content is vehicle-specific (likely a build/part code). Varies between cars.
  // Log reference: "DAM42365" (44 41 4D 34 32 33 36 35).
  // Not matched in available DBCs.
  frame.identifier = FORD_GEN42_VIN_ASCII_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x44;
  frame.data[1] = 0x41; // 'D' 'A'
  frame.data[2] = 0x4D;
  frame.data[3] = 0x34; // 'M' '4'
  frame.data[4] = 0x32;
  frame.data[5] = 0x33; // '2' '3'
  frame.data[6] = 0x36;
  frame.data[7] = 0x35; // '6' '5'
  standaloneTx(frame);
}

// =============================================================================
// Volvo / SAAB Standalone Frames (V10.00.0)
// =============================================================================
// Starting point for four new Haldex generations: IDs, send rates, counters
// and the bytes that carry the lock request. The Volvo network heartbeat
// matches a real Volvo PCAN trace.
//
// What each static byte MEANS is not known yet: those frames are sent with
// fixed starting values, untouched. Adjust any byte live with the serial lab
// (OVR <id> <byte> <value>) and watch the result in SavvyCAN, then replace these
// with decoded values once we have our own logs.
//
// Lock request: the strongest known lock value for each car is used as "100 %",
// scaled down by getLockTargetAdjustedValue() (0 when lock is off).
// All frames go out on the Haldex bus through standaloneTx().
// =============================================================================

// Send one 8-byte Volvo / SAAB frame. isExtended = true for Volvo's 29-bit IDs.
static void sendVolvoSaabFrame(uint32_t id, bool isExtended, const uint8_t data[8])
{
  twai_message_t frame = {};
  frame.identifier = id;
  frame.extd = 0; // 11-bit ID
  if (isExtended)
  {
    frame.extd = 1; // 29-bit ID
  }
  frame.rtr = 0;
  frame.data_length_code = 8;
  for (uint8_t i = 0; i < 8; i++)
  {
    frame.data[i] = data[i];
  }
  standaloneTx(frame);
}

// --- Volvo P2 Gen2 (pre-facelift, 250 kbit/s, 29-bit IDs) -------------------
// Haldex replies on VOLVO_P2G2_HALDEX_ID (0x0120000A), decoded in parseCANHdx.

// Static frame data: top of this file.

void VolvoP2Gen2_frames10()
{
  uint8_t data[8];

  // lock request: 0 = no lock, 0xF4 = the strongest value seen (100 %)
  const uint8_t lockByte = getLockTargetAdjustedValue(0xF4, false);
  const bool isLockActive = (lockByte > 0);

  sendVolvoSaabFrame(VOLVO_P2G2_224024_ID, true, VOLVO_P2G2_224024_DATA);

  // 0x0042401E - carries the lock request. Every byte: [KM -][REA none - no gen-21 dump]
  data[0] = 0xC4; // static [LOG: bits 7-6 roll (counter) on the 2003 car - we send it frozen]
  data[1] = 0x00; // static [meaning unknown]
  data[2] = 0x20; // 0x20 normally, 0x21 only at full lock [LOG: b2[4:0] = rpm bits 12-8, so 0x21 = +256 rpm]
  if (isLockActive && lockTarget >= 95.0f)
  {
    data[2] = 0x21;
  }
  data[3] = lockByte; // "lock request" [LOG: = rpm bits 7-0 -> we really send 0-244 rpm (500 at >= 95 %), below the ~300 gate seen on gen 43]
  data[4] = 0x45;     // static [meaning unknown]
  data[5] = 0xF4;     // static [meaning unknown]
  data[6] = 0xAE;     // static [meaning unknown]
  data[7] = 0x4C;     // static [meaning unknown]
  sendVolvoSaabFrame(VOLVO_P2G2_LOCK_ID, true, data);

  // 0x0062401E - "lock active" flags in bytes 6-7. Every byte: [KM -][REA none]; [LOG] layout does NOT match gen 3's
  data[0] = 0xC0; // static [meaning unknown]
  data[1] = 0x00; // static [meaning unknown]
  data[2] = 0xD8; // static [meaning unknown]
  data[3] = 0x00; // static [meaning unknown]
  data[4] = 0x68; // static [meaning unknown]
  data[5] = 0x11; // static [meaning unknown]
  data[6] = 0x60; // 0x60 idle, 0x67 lock active [LOG: 67 D0 = a 13-bit 2000, like gen 3's old rpm trick - meaning here unknown]
  data[7] = 0x00; // 0x00 idle, 0xD0 lock active [as byte 6]
  if (isLockActive)
  {
    data[6] = 0x67;
    data[7] = 0xD0;
  }
  sendVolvoSaabFrame(VOLVO_P2G2_ACTIVE_ID, true, data);

  sendVolvoSaabFrame(VOLVO_P2G2_C24000_ID, true, VOLVO_P2G2_C24000_DATA);
}

void VolvoP2Gen2_frames20()
{
  sendVolvoSaabFrame(VOLVO_P2G2_1E2402C_ID, true, VOLVO_P2G2_1E2402C_DATA);
}

void VolvoP2Gen2_frames100()
{
  uint8_t data[8];

  // 0x0242C07C - network heartbeat: bytes 6-7 swap to their exact opposite on each send
  // Every byte [KM -][REA none]; [LOG] same pages as gen 3's 0x10A2407C (01, 02, 05, 0x10-0x23) - only page 1 is sent
  data[0] = 0x01; // page 1 (the real car multiplexes this frame by byte 0)
  data[1] = 0x1F; // static [unknown]
  data[2] = 0x49; // static [unknown]
  data[3] = 0xDE; // static [LOG: b3-5 = a running clock on the car - frozen here]
  data[4] = 0x38; // static [clock]
  data[5] = 0x08; // static [clock]
  data[6] = 0xFE; // FE F8, then 01 07 on the next send [LOG heartbeat]
  data[7] = 0xF8;
  if (volvoHeartbeatFlip)
  {
    data[6] = 0x01;
    data[7] = 0x07;
  }
  volvoHeartbeatFlip = !volvoHeartbeatFlip;
  sendVolvoSaabFrame(VOLVO_P2G2_NETWORK_ID, true, data);

  sendVolvoSaabFrame(VOLVO_P2G2_3200002_ID, true, VOLVO_P2G2_3200002_DATA);
  sendVolvoSaabFrame(VOLVO_P2G2_3400002_ID, true, VOLVO_P2G2_3400002_DATA);
}

// --- Volvo P2 Gen3 (facelift, 500 kbit/s, 29-bit IDs) ------------------------
// Haldex replies on VOLVO_P2G3_HALDEX_ID (0x1020000A), decoded in parseCANHdx.

// V11.00.0 (OEM code: spec_volvo_p2_s60r.md): the S60R DEM's own message table
// confirms every ID here (29-bit, all DLC 8). It runs a closed-loop slip +
// engine-torque strategy (same family as the VAG Gen2 unit), so there is no
// single "ceiling" byte: our lock request is LOCK 0x0062401E byte 1 plus the
// ACTIVE / MODE / LOCK "active" bytes, all switched together (bench-proven).
// V11.00.13 (audit_volvo_c166 #30/#31/F3): byte 1 is NOT a lock request - it is the low byte of the
// 10-bit engine torque. At any lock > 0 the bundle switches rpm 0 -> 2000, pedal 0 -> 100 %, slip 0 -> 322
// on the b4-7 wheel pair and torque-2 -2048 -> -48 at once (all normal-path OEM inputs, no EOL hack);
// only engine torque C030 = 4 x byte 1 (0..752) scales with the %. So > 0 % is a STEP and the straight
// line 0..0xBC is not backed by the code [REA]. Per-byte tags: see the table notes at the top of the file.
// "500k S60 log" = the public log volvo-CAN-exploration/0000286.TXT (500 kbit/s,
// model year not stated, carries these facelift IDs). It gave us the counters
// below and the content of four frames the DEM listens for that we never sent.
// The 2002/2003 P2 logs are pre-facelift (250 kbit/s, other IDs), not used here.

// Static frame data and counters: top of this file.
// 0x01A2402A is also in the DEM's table but never appears in the log, so it is not sent.
// V11 TODO (bench): if a scan shows a "no communication" fault for it, capture a car.

void VolvoP2Gen3_frames10()
{
  uint8_t data[8];

  // lock request: 0 = no lock, 0xBC = the strongest value seen on the bench (100 %).
  // lockRequestPercent() already gives 0 while a lock gate holds lock off, and
  // the learn sweep position while Learn runs. Straight line 0..0xBC (spec section 2).
  // V11 TODO (bench): the OEM code does not prove 0xBC is exactly full scale.
  // [REA #31/#32: PARTLY / UNVERIFIABLE - nothing in the code marks a full scale for C030; see above]
  const uint8_t lockPercent = lockRequestPercent();
  const uint8_t lockByte = (uint8_t)(((uint16_t)0xBC * lockPercent + 50) / 100); // round to nearest
  const bool isLockActive = (lockByte > 0);

  // the toggle bit flips once per 10 ms send, like the real car
  volvoP2G3Toggle = !volvoP2G3Toggle;

  // 0x10004002 - "lock active" flags in bytes 4-7. The 500k S60 log shows this
  // frame is most likely four 10-bit wheel speeds (2 bits in bytes 0/2/4/6 +
  // bytes 1/3/5/7, all near-equal when driving), so "lock active" = 0x142 on
  // the last two wheels with the first two at 0 - probably a speed difference
  // the DEM's slip strategy answers with lock. Kept exactly as bench-proven.
  // V11 TODO (bench): the log has byte 0 = 0x80, we send 0xC0 (bench capture).
  // [REA #36/#77: four 14-bit wheel speeds, not 10-bit; b0[7:6] carries no signal, so 0xC0 vs 0x80 is harmless]
  data[0] = 0xC0; // static [KM -][REA: b0[7:6] no signal; b0[5:0] = s21 wheel speed bits 13-8 (-> C034, b0-3 pair)]
  data[1] = 0x00; // static [KM -][REA s21 ctl bits 7-0: 0]
  data[2] = 0x00; // static [KM -][REA s05 ctl b2[7:6] mode selector: 0 = normal (1/2 -> C026 mode 4); b2[5:0] = s20 speed (-> C032)]
  data[3] = 0x00; // static [KM -][REA s20 ctl: 0]
  data[4] = 0xC0; // C0 00 C0 00 idle, C1 42 C1 42 lock active [KM -][REA s1F mon b4[7:6] = 3; s1D ctl b4[5:0]:b5 (-> C012, b4-7 pair)]
  data[5] = 0x00; // [KM -][REA s1D ctl: 0 idle, 0x142 = 322 active (~5 km/h at 1/64, [LOG][BENCH] scale)]
  data[6] = 0xC0; // [KM -][REA s1E mon b6[7:6] = 3; s1C ctl b6[5:0]:b7 (-> C010)]
  data[7] = 0x00; // [KM -][REA s1C ctl: 0 idle, 322 active - b4-7 faster than b0-3 = slip]
  if (isLockActive)
  {
    data[4] = 0xC1;
    data[5] = 0x42;
    data[6] = 0xC1;
    data[7] = 0x42;
  }
  sendVolvoSaabFrame(VOLVO_P2G3_ACTIVE_ID, true, data);

  // 0x0042406C - bench-captured bytes, plus the 2-bit rolling counter the
  // real car has in byte 0 bits 6-7 (500k S60 log: 00 40 80 C0 00 ... every
  // 10 ms). V11.00.0: was a frozen 0x80.
  for (uint8_t i = 0; i < 8; i++)
  {
    data[i] = VOLVO_P2G3_42406C_DATA[i]; // copy the starting values
  }
  data[0] = (uint8_t)((VOLVO_P2G3_42406C_DATA[0] & 0x3F) | (volvoP2G3Counter42406C << 6)); // byte 0: counter in bits 6-7, low bits as captured [KM -][REA #47: REQUIRED - the DEM drops a frame whose byte 0 repeats]
  sendVolvoSaabFrame(VOLVO_P2G3_42406C_ID, true, data); // b1-b7: see the table note at the top of the file
  volvoP2G3Counter42406C = (volvoP2G3Counter42406C + 1) & 0x03; // next count, 0-3

  // 0x00C0402A - V11.00.0 new: the DEM listens for it (OEM code: MO table),
  // 10 ms in the 500k S60 log, sent with that log's steady bytes
  // [REA #67: only b3 (s19, monitored for errors) and b0 bits 5-6 (s1A, referenced nowhere) are defined - no control use]
  sendVolvoSaabFrame(VOLVO_P2G3_C0402A_ID, true, VOLVO_P2G3_C0402A_DATA);

  // 0x0062401E - carries the lock request.
  // V11 TODO (bench): in the 500k S60 log this frame comes from the engine side
  // (byte 0 = 0x02 most of the time, byte 1 moves a little either side of 0,
  // so byte 1 is probably an engine-torque signal the DEM feeds forward).
  // [REA #27/#30 CONFIRMED: it is - s09 10-bit engine torque b0[1:0]:b1, C030 = 4 x raw - 2048]
  data[0] = 0x00;     // 0x00 idle, 0x02 lock active [KM -][REA s09 ctl: b0[1:0] = torque bits 9-8; 0x02 = bit 9 -> raw 512 + byte 1; b0[7:2] no signal]
  data[1] = lockByte; // lock request byte [KM -][REA s09 ctl: torque bits 7-0 - NOT a lock request; idle raw 0 -> C030 -2048, active C030 = 4 x lockByte]
  data[2] = 0x42;     // static [KM -][REA: no signal]
  data[3] = 0xEE;     // static [KM -][REA: no signal]
  data[4] = 0xE8;     // E8 00 64 00 idle, E9 F4 65 F4 lock active [KM -][REA s06 ctl: b4[1:0]:b5 second torque C016 = 4 x raw - 2048; other bits no signal]
  data[5] = 0x00;     // [KM -][REA s06 ctl: idle raw 0 -> C016 -2048, active 0x1F4 = 500 -> -48]
  data[6] = 0x64;     // [KM -][REA s01 ctl: b6[1:0]:b7 pedal C038 = (2 x raw + 5) / 10; s0A mon b6 bits 2-3 (= 1)]
  data[7] = 0x00;     // [KM -][REA s01 ctl: idle 0 -> C038 0, active 0x1F4 = 500 -> C038 100]
  if (isLockActive)
  {
    data[0] = 0x02;
    data[4] = 0xE9;
    data[5] = 0xF4;
    data[6] = 0x65;
    data[7] = 0xF4;
  }
  sendVolvoSaabFrame(VOLVO_P2G3_LOCK_ID, true, data);

  // 0x0080401E - mode flags in bytes 2-3. In the 500k S60 log bytes 2-3 follow
  // the engine (0x0366 just after start, about 0x0290 at idle, rising when driving), so
  // they are most likely engine rpm with a flag in bit 7: "lock active" 87 D0 =
  // 2000 rpm, the same trick as our Volvo P3 / SAAB frames. Kept as bench-proven.
  // [REA #33/#34 CONFIRMED: s07 13-bit engine speed; the "ready" ramp needs > ~401 rpm, so idle 0 rpm =
  // feedback b7[7:6] = 2 "not ready" (#56) - normal here, not a fault]
  data[0] = 0x1E; // 1E and 06 every other send (500k S60 log). V11.00.0: was a frozen 1E [KM -][REA: no signal]
  if (volvoP2G3Toggle)
  {
    data[0] = 0x06;
  }
  data[1] = 0x03; // static [KM -][REA: no signal]
  data[2] = 0x80; // 80 00 idle, 87 D0 lock active [KM -][REA s07 ctl: b2[4:0] = rpm bits 12-8; bits 5-7 no signal]
  data[3] = 0x00; // [KM -][REA s07 ctl: rpm bits 7-0 - idle 0 rpm, active 0x07D0 = 2000 rpm]
  data[4] = 0x00; // static [KM -][REA: no signal]
  data[5] = 0x19; // static [KM -][REA: no signal]
  data[6] = 0x72; // static [KM -][REA s03 ctl bit 3 brake switch = 0 (-> C02A); s08 mon bits 4-5 = 3; s04 mon bits 6-7 = 1]
  data[7] = 0x00; // static [KM -][REA: no signal]
  if (isLockActive)
  {
    data[2] = 0x87;
    data[3] = 0xD0;
  }
  sendVolvoSaabFrame(VOLVO_P2G3_MODE_ID, true, data);

  // 0x00224024 - bench-captured bytes; byte 0 bit 2 flips every send like the
  // real car (500k S60 log: 1C 18 1C 18 ...). V11.00.0: was a frozen 0x3F.
  // [REA: only b6 bit 2 (s00 -> C027 mode logic) is read; the b0 toggle carries no signal - cosmetic]
  for (uint8_t i = 0; i < 8; i++)
  {
    data[i] = VOLVO_P2G3_224024_DATA[i]; // copy the starting values
  }
  if (volvoP2G3Toggle)
  {
    data[0] = (uint8_t)(VOLVO_P2G3_224024_DATA[0] & ~(1 << 2)); // byte 0 bit 2 = 0 this time [KM -][REA: no signal]
  }
  else
  {
    data[0] = (uint8_t)(VOLVO_P2G3_224024_DATA[0] | (1 << 2)); // byte 0 bit 2 = 1 this time [KM -][REA: no signal]
  }
  sendVolvoSaabFrame(VOLVO_P2G3_224024_ID, true, data);

  sendVolvoSaabFrame(VOLVO_P2G3_E24026_ID, true, VOLVO_P2G3_E24026_DATA); // [REA s02 ctl: b4 bit 5 only, 0 = brake vote allowed]

  // V11.00.0 new: 0x10E24000 (56 ms in the log) and 0x11220028 (60 ms in the
  // log) go out on every 5th 10 ms send = 50 ms - a bit faster than the car,
  // which can never trip the DEM's "frame missing" timer.
  volvoP2G3Slow++;
  if (volvoP2G3Slow >= 5)
  {
    volvoP2G3Slow = 0; // start counting again
    sendVolvoSaabFrame(VOLVO_P2G3_10E24000_ID, true, VOLVO_P2G3_10E24000_DATA); // [REA #69: monitored for errors only]
    sendVolvoSaabFrame(VOLVO_P2G3_11220028_ID, true, VOLVO_P2G3_11220028_DATA); // [REA #70: its one field is referenced nowhere]
  }
}

void VolvoP2Gen3_frames100()
{
  uint8_t data[8];

  // 0x10A2407C - network heartbeat: bytes 6-7 swap to their exact opposite on each send
  // [REA s0C, #44/#46/#48/F9: only page 1 is sent - page 5 (axle-speed divisors) and pages 0x10 / 0x14
  // (calibration set) are not; without 0x10 / 0x14 the DEM stays on set 0 = the S60 log car's set]
  data[0] = 0x01; // page 1 [KM -][REA s0C page select]
  data[1] = 0x1F; // static [KM -][REA #48: bit 4 read by the network thread]
  data[2] = 0xB8; // static [KM -][REA #48: b2-b5 read by the network thread]
  data[3] = 0xFA; // static [KM -][REA #48]
  data[4] = 0xC2; // static [KM -][REA #48]
  data[5] = 0x7A; // static [KM -][REA #48]
  data[6] = 0xFE; // FE F5, then 01 0A on the next send [KM -][REA #48: b6-7 heartbeat read on no path found]
  data[7] = 0xF5;
  if (volvoHeartbeatFlip)
  {
    data[6] = 0x01;
    data[7] = 0x0A;
  }
  volvoHeartbeatFlip = !volvoHeartbeatFlip;
  sendVolvoSaabFrame(VOLVO_P2G3_NETWORK_ID, true, data);

  // 0x11420006 - V11.00.0 new: the DEM listens for it (OEM code: MO table).
  // 120 ms in the 500k S60 log; sent every 100 ms with that log's steady bytes.
  sendVolvoSaabFrame(VOLVO_P2G3_11420006_ID, true, VOLVO_P2G3_11420006_DATA); // [REA #70: its fields are referenced nowhere]
}

void VolvoGen4_frames10()
{
  uint8_t data[8];

  uint16_t engineRpm;
  uint16_t rearSpeed;
  uint16_t frontSpeed;
  VolvoGen4_lockRequestInputs(lockRequestPercent(), engineRpm, rearSpeed, frontSpeed);

  volvoP2G3Toggle = !volvoP2G3Toggle; // flips once per 10 ms send, like the real car

  // block 0 - 0x10004002 wheel speeds: LOCK INPUT (slip / stood still).
  // Wheels 0-1 (bytes 0-3) = rear axle, wheels 2-3 (bytes 4-7) = front axle.
  for (uint8_t wheel = 0; wheel < 4; wheel++)
  {
    uint16_t speed = rearSpeed;
    if (wheel >= 2)
    {
      speed = frontSpeed;
    }
    volvoGen4PutWheel(data, wheel, VOLVO_G4_WHEEL_QUALIFIER[wheel], speed);
  }
  sendVolvoSaabFrame(VOLVO_G4_WHEELS_ID, true, data);

  // block 1 - 0x0042406C: captured bytes + 2-bit rolling counter in byte 0 bits 6-7
  for (uint8_t i = 0; i < 8; i++)
  {
    data[i] = VOLVO_P2G3_42406C_DATA[i];
  }
  data[0] = (uint8_t)((VOLVO_P2G3_42406C_DATA[0] & 0x3F) | (volvoP2G3Counter42406C << 6)); // [KM -][REA S60R #47: repeat-drop on b0 - kept; bench: block not needed on the P3]
  sendVolvoSaabFrame(VOLVO_P2G3_42406C_ID, true, data); // b1-b7 as the gen 3 table note (layout only on the P3)
  volvoP2G3Counter42406C = (volvoP2G3Counter42406C + 1) & 0x03;

  // block 2 - 0x00C0402A: steady bytes from the S60 log
  sendVolvoSaabFrame(VOLVO_P2G3_C0402A_ID, true, VOLVO_P2G3_C0402A_DATA); // [KM -][REA S60R: b3 mon, b0 bits 5-6 nr][bench: not needed]

  // block 3 - 0x0062401E engine torque / pedal: LOCK INPUT (held at the values
  // the lock request was measured with, see volvoGen4PutTorque). Byte 4 bit 4 = brake
  // switch (ECM copy): set only with Follow Brake on and the brake pressed
  // (V11.00.8 - our frame replaces the car's, so this is the only way the unit
  // sees the pedal; Follow Brake off keeps it clear to achieve the lock).
  data[0] = 0x00; // [KM -] b0[7:2] no signal [REA S60R]; b0[1:0] set by volvoGen4PutTorque
  data[1] = 0x00; // [KM -] torque low byte, set by volvoGen4PutTorque
  data[2] = 0x42; // S60 log: 0x43/0x44 with bit 5 toggling once a second - not read by the DEM [KM -][REA S60R: no signal]
  data[3] = 0xEE; // S60 log: moves 1-253, meaning unknown - not read by the DEM [KM -][REA S60R: no signal]
  data[4] = 0xE8; // bit 4 = brake switch (0 = off) [KM -][LOG][bench; REA S60R: bit 4 is NOT in its signal table - no OEM code]
  if (brakeForHaldex())
  {
    data[4] = (uint8_t)(data[4] | 0x10); // brake pressed (Follow Brake)
  }
  data[5] = 0x00; // [KM -] 2nd torque low byte, set by volvoGen4PutTorque
  data[6] = 0x64; // [KM -][REA S60R s0A mon bits 2-3 (= 1)]; b6[1:0] pedal set by volvoGen4PutTorque
  data[7] = 0x00; // [KM -] pedal low byte, set by volvoGen4PutTorque
  volvoGen4PutTorque(data);
  sendVolvoSaabFrame(VOLVO_G4_TORQUE_ID, true, data);

  // block 4 - 0x0080401E engine rpm in bytes 2-3: LOCK INPUT (pump on / off)
  data[0] = 0x1E; // 1E and 06 every other send (500k S60 log) [KM -][REA S60R: no signal]
  if (volvoP2G3Toggle)
  {
    data[0] = 0x06;
  }
  data[1] = 0x03; // [KM -][REA S60R: no signal]
  data[2] = (uint8_t)(0x80 | ((engineRpm >> 8) & 0x1F)); // bit 7 always set (log) [KM -][REA S60R s07 rpm bits 12-8 (bits 5-7 no signal)]
  data[3] = (uint8_t)(engineRpm & 0xFF);                 // [KM -][REA S60R s07 rpm bits 7-0, 1 rpm/bit][bench: < ~300 = pump off; no OEM code for the P3 gate]
  data[4] = 0x00; // [KM -][REA S60R: no signal]
  data[5] = 0x19; // [KM -][REA S60R: no signal]
  data[6] = 0x72; // bit 3 = brake switch (bench: cuts lock to 0 while moving) [KM -][REA S60R s03 ctl brake vote; s08 / s04 mon bits 4-7]
  if (brakeForHaldex())
  {
    data[6] = (uint8_t)(data[6] | 0x08); // brake pressed (Follow Brake), as the torque frame above
  }
  data[7] = 0x00; // [KM -][REA S60R: no signal]
  sendVolvoSaabFrame(VOLVO_G4_ENGINE_ID, true, data);

  // block 5 - 0x00224024: captured bytes, byte 0 bit 2 flips every send
  // [KM -][REA S60R: only b6 bit 2 (s00) read; b0 toggle no signal][bench: block not needed]
  for (uint8_t i = 0; i < 8; i++)
  {
    data[i] = VOLVO_P2G3_224024_DATA[i];
  }
  if (volvoP2G3Toggle)
  {
    data[0] = (uint8_t)(VOLVO_P2G3_224024_DATA[0] & ~(1 << 2));
  }
  else
  {
    data[0] = (uint8_t)(VOLVO_P2G3_224024_DATA[0] | (1 << 2));
  }
  sendVolvoSaabFrame(VOLVO_P2G3_224024_ID, true, data);

  // block 6 - 0x00E24026: captured bytes
  sendVolvoSaabFrame(VOLVO_P2G3_E24026_ID, true, VOLVO_P2G3_E24026_DATA); // [KM -][REA S60R: b4 bit 5 (s02) only][bench: not needed]

  // blocks 7 / 8 - 0x10E24000 and 0x11220028 every 50 ms
  volvoGen4Slow++;
  if (volvoGen4Slow >= 5)
  {
    volvoGen4Slow = 0;
    sendVolvoSaabFrame(VOLVO_P2G3_10E24000_ID, true, VOLVO_P2G3_10E24000_DATA); // [KM -][REA S60R: mon only][bench: not needed]
    sendVolvoSaabFrame(VOLVO_P2G3_11220028_ID, true, VOLVO_P2G3_11220028_DATA); // [KM -][REA S60R: not read][bench: not needed]
  }
}

void VolvoGen4_frames100()
{
  uint8_t data[8];

  // block 9 - 0x10A2407C page 1, network heartbeat: bytes 6-7 swap to their exact opposite each send
  if (frameEditEnabled(FE_GEN_VOLVO_GEN4, VOLVO_G4_BLOCK_NETWORK))
  {
    data[0] = 0x01; // page 1 [KM -][bench: block needed (heartbeat); no OEM code for the P3]
    data[1] = 0x1F; // [KM -][REA S60R #48: bit 4 read by the network thread (layout only)]
    data[2] = 0xB8; // [KM -][REA S60R #48: b2-b5 read by the network thread]
    data[3] = 0xFA; // [KM -][REA S60R #48]
    data[4] = 0xC2; // [KM -][REA S60R #48]
    data[5] = 0x7A; // [KM -][REA S60R #48]
    data[6] = 0xFE; // FE F5, then 01 0A on the next send [KM -][REA S60R: read on no path found][LOG]
    data[7] = 0xF5; // [KM -][as byte 6]
    if (volvoHeartbeatFlip)
    {
      data[6] = 0x01;
      data[7] = 0x0A;
    }
    volvoHeartbeatFlip = !volvoHeartbeatFlip;
    sendVolvoSaabFrame(VOLVO_G4_NETWORK_ID, true, data);
  }

  // block 10 - 0x11420006: steady bytes from the S60 log
  sendVolvoSaabFrame(VOLVO_P2G3_11420006_ID, true, VOLVO_P2G3_11420006_DATA); // [KM -][REA S60R: not read][bench: not needed]

  // block 11 - 0x10A2407C page 0x14 (configuration) once a second: stops fault 0x0015 / limp
  volvoGen4Config++;
  if (volvoGen4Config >= 10)
  {
    volvoGen4Config = 0;
    if (frameEditEnabled(FE_GEN_VOLVO_GEN4, VOLVO_G4_BLOCK_CONFIG))
    {
      sendVolvoSaabFrame(VOLVO_G4_NETWORK_ID, true, VOLVO_G4_CONFIG_PAGE_DATA); // [KM -][bench: b7 0x59 stops 0x0015; bytes: see the table note]
    }
  }
}
                                      // override latched by V11.00.9 / 10 (or a tool); nothing is sent on 0x6F1 after that

static void kugaSend(uint32_t id, uint8_t dlc, const uint8_t *data)
{
  twai_message_t f = {};
  f.identifier = id;
  f.extd = 0;
  f.data_length_code = dlc;
  for (uint8_t i = 0; i < dlc; i++)
  {
    f.data[i] = data[i];
  }
  standaloneTx(f);
}

// V11.00.11: nothing on 0x6F1 any more. When our frames stop (Stock, controller off, generation
// change) the next start sends the one-shot release again. Called by standaloneFramesTask.
void FordKuga_releaseOverride()
{
  kugaReleaseLeft = 3;
}

void FordKuga_frames10()
{
  const uint8_t lockPercent = lockRequestPercent();
  const bool lockAsked = lockPercent > 0;
  uint8_t d[8];

  // block 3 - 0x201 engine: rpm b0-1 (base demand gate > ~401), pedal b6 (pump demand gate > 5).
  // The unit reads nothing else of this frame (RX signal table, trackF). Standalone: rpm + pedal only
  // while lock is asked for (FWD = pump off). OEM: the car's rpm and pedal, the pedal raised to
  // KUGA_PEDAL_ACTIVE while lock is asked for so the pump follows the demand foot-off too.
  uint16_t rpm = lockAsked ? KUGA_RPM_RUNNING : 0;
  uint8_t pedal = lockAsked ? KUGA_PEDAL_ACTIVE : 0;
  uint16_t speedRaw = 0;
  if (!isStandalone)
  {
    rpm = receivedVehicleRpm;
    const float carPedal = receivedPedalValue * 2.0f; // 0.5 %/bit
    pedal = (uint8_t)constrain(carPedal, 0.0f, 250.0f);
    if (lockAsked && pedal < KUGA_PEDAL_ACTIVE)
    {
      pedal = KUGA_PEDAL_ACTIVE;
    }
    speedRaw = (uint16_t)constrain((uint32_t)receivedVehicleSpeed * 100, 0, 0xFFFF);
  }
  d[0] = (uint8_t)(rpm >> 8);      // b0[6:0] rpm bits 14-8 [KM - (CGEA EngAout_N_Actl 7|13 x2 rpm - differs)] [REA #18/#38: 15-bit b0[6:0]:b1 F1050 = raw x 6.702, gate > 2688 / 1344 = raw > ~401 / ~200; b0 bit 7 no reader (#19)]
  d[1] = (uint8_t)(rpm & 0xFF);    // b1 rpm bits 7-0, 1 rpm/bit (Focus RS log) [KM -] [REA: see b0]
  d[2] = 0x40; // torque word 0x4000 = 0 (not read by the unit) [KM -] [REA: no signal]
  d[3] = 0x00;                     // [KM -] [REA: no signal]
  d[4] = (uint8_t)(speedRaw >> 8); // road speed 0.01 km/h (not read by the unit) [KM - (CGEA Veh_V_ActlEng is b2-3)] [REA: no signal]
  d[5] = (uint8_t)(speedRaw & 0xFF); // [KM -] [REA: no signal]
  d[6] = pedal;                    // b6 pedal 0.5 %/bit (Focus RS log) [KM -] [REA #41/#35/#91: F106A; > 5 enables pump demand F156B, < 40 allows mode 2, preload factor; 9V4N also IIR -> 0x205 torque floor (trackH)]
  d[7] = 0x7F;                     // [KM -] [REA: no signal]
  kugaSend(FORD_KUGA_ENGINE_ID, 8, d);

  // block 1 - 0x205 engine torque: w0 (b0[4:0]:b1) and w3 (b6[4:0]:b7) are the ones the unit reads
  // (+ b2 bit 6 brake switch = Follow Brake [REA audit C: effect = mode 2 + preload off - it reduces the
  // demand, it never opens; "brake switch" is INF from the Focus RS log]). High while lock is asked for, so the 0x211 ceiling is
  // what sets the demand; idle otherwise. w1 / w2 (min / max available) are not read - kept as the trace.
  const uint16_t tq = (uint16_t)((lockAsked ? KUGA_DRIVE_TORQUE_NM : KUGA_IDLE_TORQUE_NM) + 512);
  d[0] = (uint8_t)((tq >> 8) & 0x1F); // b0[4:0] w0 bits 12-8 [KM -] [REA #20: 13-bit b0[4:0]:b1 F105E = (raw-512) x 4 (1 Nm/bit +512, INF); > ~620 Nm caps the demand at C 1281 (#92); b0 bit 5 no reader]
  d[1] = (uint8_t)(tq & 0xFF);        // b1 w0 bits 7-0 [KM -] [REA: see b0]
  d[2] = 0x02; // minimum torque word high byte (0x0219 in the trace); bit 6 = brake [KM -] [REA #20: bit 6 -> F1056 (mode 2 + preload off), bit 7 no reader, b2[4:0]:b3 no signal]
  d[3] = 0x19;                        // [KM -] [REA: no signal]
  d[4] = 0x02; // maximum torque available (0x0267) [KM -] [REA trackF: w2 not read]
  d[5] = 0x67;                        // [KM -] [REA: not read]
  d[6] = (uint8_t)((tq >> 8) & 0x1F); // b6[4:0] w3 bits 12-8 [KM -] [REA #20: 13-bit b6[4:0]:b7 F1044 = (raw-512) x 4, 2nd torque; 9V4N floors it from pedal / rpm (trackH)]
  d[7] = (uint8_t)(tq & 0xFF);        // b7 w3 bits 7-0 [KM -] [REA: see b6]
  if (brakeForHaldex())
  {
    d[2] = (uint8_t)(d[2] | 0x40); // b2 bit 6 = Follow Brake [REA audit C: mode 2 + preload off - reduces, never opens]
  }
  kugaSend(FORD_KUGA_TORQUE_ID, 8, d);

  // block 4 - 0x211 ABS / ESP: THE LOCK REQUEST. The unit reads b2[3:0]:b3 (ceiling C) and b6 bits 1-2 (request
  // field: 1 = zero torque / open, 2 = ramp down, 3 = ceiling) - REA trackF + audit_ford_kuga (b7 not copied,
  // b2 bit 7 read by nothing). b0-b1, b4-b5: Focus Mk2 parked trace values (bench: b5 bit 0 = 1 released the
  // clutch, reader not found in the code - audit_ford_kuga unresolved 1).
  // V11.00.13 (audit FI-2): in OEM the car's own 0x211 is the base - only C and the field are ours, and
  // whenever the car's ESP itself asks for open / ramp-down (field 1 / 2) its whole frame goes through, so
  // the car's ABS / ESP release always wins. Standalone: the trace frame.
  const bool useCar211 = !isStandalone && kugaCar211Valid;
  for (uint8_t i = 0; i < 8; i++)
  {
    d[i] = useCar211 ? kugaCar211[i] : KUGA_211_DATA[i];
  }
  const uint8_t carField = useCar211 ? (uint8_t)((kugaCar211[6] >> 1) & 0x03) : 0;
  if (useCar211 && (carField == 1 || carField == 2))
  {
    // the car's ESP is releasing the clutch: its frame, untouched
  }
  else if (absBrakingForHaldex())
  {
    d[6] = (uint8_t)((d[6] & ~0x06) | 0x02); // Follow ABS: field 1 (bits 1-2 = 01) = zero torque, clutch open [KM -] [REA #36: mode 4, F156C = 0; 0x260 b1 = 0x40]
  }
  else
  {
    const uint16_t c = kugaCeilingForPercent(lockPercent); // never above the car's own field-3 ceiling (OEM)
    d[2] = (uint8_t)((d[2] & 0xF0) | ((c >> 8) & 0x0F)); // b2[3:0] = C bits 11-8 (b2[7:4] kept: car's / trace 0x8) [KM -] [REA #22: C<<3 = demand ceiling, unsigned min @0x369EA; b2 bit 7 read by nothing (#79)]
    d[3] = (uint8_t)(c & 0xFF);                           // b3 = C bits 7-0 [KM -] [REA: see b2]
    d[6] = (uint8_t)((d[6] & ~0x06) | 0x06);              // b6 bits 1-2 = 3: torque ceiling = C [KM -] [REA #21: F1032 = 2, F103C = C<<3; 8V41 sets 0x260 b1 bit 2]
  }
  if (!useCar211)
  {
    d[7] = kugaCounter8; // b7 counter (not read by the unit; kept as the Focus sends it). OEM: the car's own counter [KM -] [REA: not copied, copy length 7]
  }
  // b0-1, b4, b5 = trace / car bytes: [REA #24: b0-1 no signal, b4 bits 4-5 and b5 bit 0 no reader (bench: b5.0 = 1 released, unexplained)]
  kugaSend(FORD_KUGA_ABS_ID, 8, d);

  // block 2 - 0x4B0 wheel speeds: stood still (raw 10000 = 0 km/h)
  for (uint8_t w = 0; w < 4; w++)
  {
    d[w * 2] = 0x27;     // wheel w high byte (FL FR RL RR) [KM RS WHEEL_SPEEDS.Wheel_Speed_1..4 0.01 km/h -100] [REA #17: |raw - 10000| x 11378/7999 - abs, direction not read]
    d[w * 2 + 1] = 0x10; // low byte: 0x2710 = 10000 = 0 km/h [KM: same signal] [REA: same]
  }
  kugaSend(FORD_KUGA_WHEELS_ID, 8, d);

  // block 7 - 0x090, counter in b0 bits 4-7
  for (uint8_t i = 0; i < 8; i++)
  {
    d[i] = KUGA_090_DATA[i];
  }
  d[0] = (uint8_t)((kugaCounter090 << 4) | (KUGA_090_DATA[0] & 0x0F)); // b0[7:4] counter [KM -] [REA: not read]; b0[3:0] = F102C high nibble [REA #25]
  kugaCounter090 = (kugaCounter090 + 1) & 0x0F;
  kugaSend(FORD_KUGA_DYNAMICS_ID, 8, d); // other bytes: see the per-byte map above KUGA_080_DATA

  // block 8 - 0x231 gear: presence
  kugaSend(FORD_KUGA_GEAR_ID, 7, KUGA_231_DATA); // [KM CGEA TransGearData_2 - C1 unconfirmed] [REA: presence only, b0[7:4] checked]

  kugaTick++;
  // block 6 - 0x080 every 20 ms, counter in b6
  if (kugaTick % 2 == 0)
  {
    for (uint8_t i = 0; i < 7; i++)
    {
      d[i] = KUGA_080_DATA[i];
    }
    d[6] = kugaCounter8; // b6 counter [KM -] [REA audit: not copied (copy length 6) - cosmetic only]
    kugaSend(FORD_KUGA_ACCEL_ID, 7, d);
  }
  kugaCounter8++;

  if (kugaTick >= 10)
  {
    kugaTick = 0;
    // block 5 - 0x433 every 100 ms: handbrake b3 bit 0 (Follow Handbrake)
    for (uint8_t i = 0; i < 8; i++)
    {
      d[i] = KUGA_433_DATA[i];
    }
    if (handbrakeForHaldex())
    {
      d[3] = (uint8_t)(d[3] | 0x01); // b3 bit 0 [KM -] [REA #26/#35: F103A -> mode 5 latch (8V41) / mode 4, alive-gated (9V4N): demand ramped to 0]
    }
    kugaSend(FORD_KUGA_HANDBRAKE_ID, 8, d); // unit copies b0-b3 only (copy length 4)

    // 0x6F1 - one-shot release only (V11.00.11), every 100 ms until done
    kugaReleaseTestChannel();

    // blocks 9 / 10 - 0x4F1 every 500 ms, 0x4F0 every 1000 ms: presence
    kugaSlowTick++;
    if (kugaSlowTick % 5 == 0)
    {
      kugaSend(FORD_KUGA_CONFIG_4F1_ID, 8, KUGA_4F1_DATA); // [KM -] [REA: 8V41 presence only; 9V4N cal-set select -> set 0 default]
    }
    if (kugaSlowTick >= 10)
    {
      kugaSlowTick = 0;
      kugaSend(FORD_KUGA_CONFIG_4F0_ID, 6, KUGA_4F0_DATA); // [KM -] [REA: 8V41 presence only; 9V4N cal-set select -> set 0 default]
    }
  }
}

static void lrSend(uint32_t id, const uint8_t *data)
{
  twai_message_t f = {};
  f.identifier = id;
  f.extd = 0;
  f.data_length_code = 8; // every LR RX object is DLC 8 [REA audit_landrover #5]
  for (uint8_t i = 0; i < 8; i++)
  {
    f.data[i] = data[i];
  }
  standaloneTx(f);
}

void LandRover_frames20()
{
  const uint8_t lockPercent = lockRequestPercent();
  const bool lockAsked = lockPercent > 0;
  const uint16_t T = lrTorqueForPercent(lockPercent);
  const uint8_t tHi = (uint8_t)((T >> 8) & 0x1F);
  const uint8_t tLo = (uint8_t)(T & 0xFF);
  uint8_t d[8];

  // OEM: whenever the car's own DSC / ABS asks to open or ramp down (field 1 / 2) its lock request frames go
  // through untouched - the car's release always wins. Which field to look at follows the feedback seen
  // (FL2 0x20A b6, Evoque 0x2B5 b1); both while it is not known yet.
  bool carRelease = false;
  if (!isStandalone)
  {
    const uint8_t fl2Field = lrCar20AValid ? (uint8_t)((lrCar20A[6] >> 5) & 0x07) : 0;
    const uint8_t evqField = lrCar2B5Valid ? (uint8_t)((lrCar2B5[1] >> 2) & 0x07) : 0;
    if (lrFeedbackLayout != LR_LAYOUT_EVQ && (fl2Field == 1 || fl2Field == 2))
    {
      carRelease = true;
    }
    if (lrFeedbackLayout != LR_LAYOUT_FL2 && (evqField == 1 || evqField == 2))
    {
      carRelease = true;
    }
  }
  // Request field we send: 4 = direct torque T; Follow ABS (our ABS input says braking) = 1 = zero torque, open.
  const uint8_t field = absBrakingForHaldex() ? 1 : 4; // [REA audit_landrover #29 / E6: 1 open, 2 ramp, 3 full, 4 direct]

  // block 0 - 0x20A: THE LOCK REQUEST for both units.
  const bool car20A = !isStandalone && lrCar20AValid;
  if (car20A)
  {
    memcpy(d, lrCar20A, 8); // OEM: the car's frame is the base - only the lock request bits below change
  }
  else
  {
    d[0] = 0x00; // [KM -] [REA #40: FL2 b0 bit 1 (sig 0x16) read by nothing; EVQ: not read]
    d[1] = 0x00; // [KM -] [REA #40: FL2 b1 bits 6-7 DID only, b1 bit 4 read by nothing; EVQ: not read]
    d[2] = 0x37; // (also set below) qualifier 3 + EVQ dynamics nibble
    d[3] = 0xFF; // [KM -] [REA E5: EVQ b2[3:0]:b3 = dynamics (FL2 0x155 b6:b7 role F15A6 = raw x 10474/15999 - 1340): 0x7FF = 0; FL2: not read]
    d[4] = 0x00; // (set below)
    d[5] = 0x00;
    d[6] = 0x00; // field 0 = no request until set below
    d[7] = 0x00;
  }
  if (!carRelease)
  {
    d[2] = car20A ? (uint8_t)((d[2] & 0x8F) | 0x30)                      // OEM: car's b2 bit 7 + EVQ dynamics nibble kept
                  : 0x37; // [KM -] [REA #29: FL2 b2 bits 4-6 = 3 = field-4 qualifier (2 or 3 accepted); EVQ E9: b2 bits 4-5 quality != 0; b2[3:0] = EVQ dynamics high nibble 7]
    d[4] = (uint8_t)((car20A ? (d[4] & 0xE0) : 0x00) | tHi); // [KM -] [REA E6: EVQ torque T b4[4:0]; b4 bits 5-6 = EVQ sig 0x05 (FL2's sig 0x05 is DID only) -> 0 / OEM the car's; FL2: not read]
    d[5] = tLo;                                              // [KM -] [REA E6: EVQ torque T b5 = bits 7-0; FL2: not read]
    d[6] = (uint8_t)((field << 5) | (field == 4 ? tHi : 0)); // [KM -] [REA #29: FL2 b6 bits 5-7 = request field (4 -> F157B 1, F158C 2), b6[4:0] = T bits 12-8 (<< 4 -> F15A0)]
    d[7] = (field == 4) ? tLo : 0x00;                        // [KM -] [REA #29/#57: FL2 T bits 7-0 (DID D953 = raw T); EVQ: not read]
  }
  lrSend(LR_REQUEST_20A_ID, d);

  // block 1 - 0x2B5: FL2 wheel speeds 1-2; Evoque ABS request field (lock request).
  const bool car2B5 = !isStandalone && lrCar2B5Valid;
  if (car2B5)
  {
    memcpy(d, lrCar2B5, 8); // OEM: the car's wheel speeds / quality untouched
  }
  else
  {
    d[0] = 0x04; // [KM -] [REA #20: FL2 b0 bits 2-3 quality (sig 0x31) != 0; b0[1:0]:b1 (sig 0x13) read by nothing; EVQ b0 bit 0 (sig 0x18) role INF -> 0]
    d[1] = 0x00; // set below [KM -] [REA E6: EVQ b1 bits 2-4 request field; b1 bit 7 (sig 0x2F) role INF -> 0; FL2: not read]
    d[2] = 0x40; // [KM -] [REA #20: FL2 b2 bits 6-7 quality (sig 0x2F) != 0; bits 4-5 (sig 0x14) read by nothing; EVQ: not read]
    d[3] = 0x00; // [KM -] [REA E5: EVQ wheel speed A b3[6:0] = 0 (standstill); FL2: not read]
    d[4] = 0x00; // [KM -] [REA #19: FL2 wheel speed 2 b4[6:0]; EVQ speed A low byte = 0]
    d[5] = 0x00; // [KM -] [REA #19: FL2 speed 2 low byte; EVQ speed B b5[6:0] = 0]
    d[6] = 0x00; // [KM -] [REA #19: FL2 wheel speed 1 b6[6:0]; EVQ speed B low byte = 0]
    d[7] = 0x09; // [KM -] [REA E9: EVQ b7 bits 3-4 / 0-1 quality != 0; CONFLICT: FL2 speed 1 low byte -> 9 = ~0.09 km/h, the others 0]
  }
  if (!carRelease)
  {
    d[1] = (uint8_t)((d[1] & 0xE3) | (field << 2)); // [KM -] [REA E6: EVQ b1 bits 2-4 = request field (4 = direct torque); FL2 never reads b1 (#40 sig 0x13)]
  }
  lrSend(LR_REQUEST_2B5_ID, d);

  // block 2 - 0x250: FL2 wheel speeds 3-4; Evoque field-4 qualifier (lock request).
  const bool car250 = !isStandalone && lrCar250Valid;
  if (car250)
  {
    memcpy(d, lrCar250, 8); // OEM: the car's wheel speeds / quality untouched
  }
  else
  {
    d[0] = 0x00; // set below
    d[1] = 0x00; // [KM -] [REA: not read by either (EVQ b1 bit 6 = an update bit, never read - #41)]
    d[2] = 0x20; // [KM -] [REA #20: FL2 b2 bits 5-6 quality (sig 0x33) != 0; EVQ: not read]
    d[3] = 0x00; // [KM -] [REA: not read]
    d[4] = 0x00; // [KM -] [REA #19 / E5: FL2 wheel speed 4 b4[6:0] = 0; CONFLICT: EVQ b4[1:0]:b5 dynamics (FL2 0x155 b2:b3 role) raw 0 = its minimum - [GUESS - bench] (its zero ~0x200 would read as 5 km/h on FL2 wheel 4)]
    d[5] = 0x00; // [KM -] [REA #19: FL2 speed 4 low byte; EVQ dynamics low byte]
    d[6] = 0x80; // [KM -] [REA E9: EVQ b6 bits 6-7 quality = 2 != 0; FL2 wheel speed 3 b6[6:0] = 0 (bit 7 is outside its 15 bits)]
    d[7] = 0x00; // [KM -] [REA #19: FL2 speed 3 low byte; EVQ: not read]
  }
  if (!carRelease && (d[0] & 0x07) != 2 && (d[0] & 0x07) != 3)
  {
    d[0] = (uint8_t)((d[0] & 0xF8) | 0x03); // [KM -] [REA E6: EVQ b0 bits 0-2 in {2,3} = field-4 qualifier; FL2 #20: b0 bits 0-1 quality != 0 - both met by 3. OEM: a car value already 2 / 3 is kept]
  }
  lrSend(LR_REQUEST_250_ID, d);

  // Engine rpm: standalone 800 while lock is asked for, 0 otherwise; OEM (block turned on by hand) the car's.
  const uint16_t rpm = isStandalone ? (lockAsked ? LR_RPM_RUNNING : 0) : receivedVehicleRpm;

  // block 3 - 0x12A (FL2): rpm gate + engine torque
  d[0] = 0x00;                              // [KM -] [REA #24: b0 bit 2 -> F1554 = 0 (no mode 1 / 2); b0 bit 3 = update bit, not read]
  d[1] = 0x00;                              // [KM -] [REA: not read]
  d[2] = 0x00;                              // [KM -] [REA: not read (b2 bit 6 = update bit, #41)]
  d[3] = 0x00;                              // [KM -] [REA: not read]
  d[4] = 0x07;                              // [KM -] [REA #23: b4[3:0]:b5 engine torque, 0x7FF = 0 Nm (offset 2047) -> F156E]
  d[5] = 0xFF;                              // [KM -] [REA #23: torque low byte]
  d[6] = (uint8_t)(0x20 | ((rpm >> 8) & 0x1F)); // [KM -] [REA #24: b6 bits 5-6 quality = 1 != 0; #21/#22: rpm b6[4:0] - F1564 = raw x 6.7 > 2688 (~401 rpm) = base-demand gate]
  d[7] = (uint8_t)(rpm & 0xFF);             // [KM -] [REA #21: rpm low byte, 1 rpm/bit (DID F40C)]
  lrSend(LR_FL2_ENGINE_ID, d);

  // block 4 - 0x100 (EVQ): rpm gate
  d[0] = 0x00;                              // [KM -] [REA E5: b0 bit 4 -> F1554 = 0]
  d[1] = 0x00;                              // [KM -] [REA: not read]
  d[2] = 0x40;                              // [KM -] [REA E9: b2 bits 6-7 quality = 1 != 0]
  d[3] = 0x00;                              // [KM -] [REA: not read]
  d[4] = (uint8_t)((rpm >> 8) & 0x1F);      // [KM -] [REA E5: rpm b4[4:0]:b5, same 2688 / 1344 gate (cal 0x638E)]
  d[5] = (uint8_t)(rpm & 0xFF);             // [KM -] [REA E5: rpm low byte]
  d[6] = 0x00;                              // [KM -] [REA: not read]
  d[7] = 0x00;                              // [KM -] [REA: not read]
  lrSend(LR_EVQ_ENGINE_ID, d);

  // blocks 5 / 6 - 0x125 (FL2) and 0x115 (EVQ), same layout: torque 0, pedal 0, quality valid
  d[0] = 0x00;                              // [KM -] [REA: not read]
  d[1] = 0x00;                              // [KM -] [REA #26: b1 = factor for the 'E'-variant torque path (x 0x143 term = 0)]
  d[2] = 0x00;                              // [KM -] [REA #25: pedal b2[1:0]:b3, 0.1 %/bit -> F157D = 0]
  d[3] = 0x00;                              // [KM -] [REA #25: pedal low byte]
  d[4] = 0x47;                              // [KM -] [REA #26: b4 bits 6-7 quality = 1 != 0; bits 4-5 DID only (#40) = 0; #23: torque b4[3:0] = 7]
  d[5] = 0xFF;                              // [KM -] [REA #23: torque 0x7FF = 0 Nm -> F1592]
  d[6] = 0x20;                              // [KM -] [REA #26: b6 bits 5-6 quality = 1 != 0]
  d[7] = 0x00;                              // [KM -] [REA: not read]
  lrSend(LR_FL2_TORQUE_ID, d);
  lrSend(LR_EVQ_TORQUE_ID, d);              // [REA E5: 0x115 = 0x125 bit for bit (sigs 0x01 / 0x02 / 0x1C / 0x1D / 0x2D / 0x2E)]

  // block 7 - 0x116 (EVQ): second torque path, neutral
  d[0] = 0x00;                              // [KM -] [REA: not read]
  d[1] = 0x00;                              // [KM -] [REA: not read]
  d[2] = 0x02;                              // [KM -] [REA E5 sig 0x0A: b2[1:0]:b3 (FL2 0x143 role, raw - 512) = 0x200 = 0]
  d[3] = 0x00;                              // [KM -] [REA: low byte of 0x200]
  d[4] = 0x00;                              // [KM -] [REA: not read]
  d[5] = 0x00;                              // [KM -] [REA: not read]
  d[6] = 0x07;                              // [KM -] [REA E5 sig 0x0B: b6[3:0]:b7 (FL2 0x12A torque role, offset 2047) = 0x7FF = 0]
  d[7] = 0xFF;                              // [KM -] [REA: low byte of 0x7FF]
  lrSend(LR_EVQ_116_ID, d);

  // block 8 - 0x143 (both)
  d[0] = 0x00;                              // [KM -] [REA: not read]
  d[1] = 0x00;                              // [KM -] [REA: not read]
  d[2] = 0x02;                              // [KM -] [REA #26: FL2 b2[1:0]:b3 = 0x200 -> (raw - 512) = 0; EVQ: not read]
  d[3] = 0x00;                              // [KM -] [REA E5: EVQ b3 bit 7 = 0 = inhibit F158E clear (REQUIRED for the base demand), b3 bits 2-3 = 0, b3[1:0]:b4 = 0 -> F157C 0]
  d[4] = 0x00;                              // [KM -] [REA E5: EVQ b4 = low byte of the 0x315-role field]
  d[5] = 0x00;                              // [KM -] [REA: not read]
  d[6] = 0x00;                              // [KM -] [REA: not read]
  d[7] = 0x00;                              // [KM -] [REA: not read]
  lrSend(LR_143_ID, d);

  // block 9 - 0x150 (both)
  d[0] = 0x00;                              // [KM -] [REA #30: FL2 b0 bit 0 F158F = 0 (no mode 2), b0 bit 4 DID only]
  d[1] = 0x00;                              // [KM -] [REA #30: FL2 b1 bit 2 F1588 = 0, b1 bit 7 nothing; EVQ b1 bits 7/5/3 + b1[1:0]:b2 roles INF -> 0]
  d[2] = 0x00;                              // [KM -] [REA: EVQ low byte of the b1[1:0]:b2 field]
  d[3] = 0x00;                              // [KM -] [REA E5: EVQ b3 bit 7 F158F = 0; wheel speed 3 b3[6:0] = 0]
  d[4] = 0x00;                              // [KM -] [REA E5: EVQ speed 3 low byte]
  d[5] = 0x09;                              // [KM -] [REA E9: EVQ b5 bits 0-1 / 3-4 quality != 0; bits 5-6 (sig 0x16) role INF -> 0]
  d[6] = 0x00;                              // [KM -] [REA E5: EVQ wheel speed 4 b6[6:0] = 0]
  d[7] = 0x00;                              // [KM -] [REA E5: EVQ speed 4 low byte]
  lrSend(LR_150_ID, d);

  // block 10 - 0x155 (FL2): dynamics at zero, quality valid
  d[0] = 0x00;                              // [KM -] [REA: not read]
  d[1] = 0x00;                              // [KM -] [REA: not read]
  d[2] = 0x02;                              // [KM -] [REA #27: b2[1:0]:b3 -> F1596 = raw x 17920/2000 - 4582: 0x200 = +5.5 ~ 0]
  d[3] = 0x00;                              // [KM -] [REA: low byte of 0x200]
  d[4] = 0x00;                              // [KM -] [REA #40: b4[3:0]:b5 (sig 0x38) DID only]
  d[5] = 0x00;                              // [KM -] [REA #40: DID only]
  d[6] = 0x57;                              // [KM -] [REA #27: b6 bits 4-5 / 6-7 quality = 1 / 1 != 0; b6[3:0]:b7 -> F15A6 = raw x 10474/15999 - 1340: 0x7FF = 0]
  d[7] = 0xFF;                              // [KM -] [REA #27: low byte of 0x7FF]
  lrSend(LR_FL2_DYN_ID, d);

  // blocks 11 / 12 - power mode, 0x160 (FL2) and 0x101 (EVQ)
  d[0] = 0x00;                              // [KM -] [REA #33: FL2 b0 bit 6 = 0 -> F15A2 = 0 (0x405 temperature table off) [GUESS - bench: what F15A2 drives]; EVQ: not read]
  d[1] = 0x03;                              // [KM -] [REA #31: b1[1:0] = 3 = power mode valid; #32: FL2 b1 bit 4 = 0 -> F1547 from 0x083 (else -1 = 14400 cap); b1 bits 2-3 nothing]
  d[2] = 0x00;                              // [KM -] [REA: not read]
  d[3] = 0x00;                              // [KM -] [REA: not read]
  d[4] = 0x00;                              // [KM -] [REA: not read]
  d[5] = 0x1C;                              // [KM -] [REA #31: b5[6:2] = 7 = power mode; monitors run in 6 / 7 [GUESS - bench: 7 = running is INF]]
  d[6] = 0x00;                              // [KM -] [REA: not read]
  d[7] = 0x00;                              // [KM -] [REA: not read]
  lrSend(LR_FL2_POWER_ID, d);
  lrSend(LR_EVQ_POWER_ID, d);               // [REA E5: 0x101 b1[1:0] = 3, b5[6:2] = power mode - same bits]

  // block 13 - 0x083 terrain / drive mode (both): 0 -> F1547 = 1
  d[0] = 0x00;                              // [KM -] [REA: FL2 not read; EVQ b0[3:0]:b1 (sig 0x2B) / b0 bits 4-5 (sig 0x2C) role INF -> 0]
  d[1] = 0x00;                              // [KM -] [REA: EVQ low byte of sig 0x2B]
  d[2] = 0x00;                              // [KM -] [REA: not read]
  d[3] = 0x00;                              // [KM -] [REA: not read]
  d[4] = 0x00;                              // [KM -] [REA: not read]
  d[5] = 0x00;                              // [KM -] [REA #32 / E5: FL2 b5 bits 0-2, EVQ b5 bits 5-7 = 0 -> F1547 = 0 + 1 (cap target 24000)]
  d[6] = 0x00;                              // [KM -] [REA: not read]
  d[7] = 0x00;                              // [KM -] [REA: not read]
  lrSend(LR_TERRAIN_ID, d);

  // blocks 14 / 15 / 18 / 19 / 20 / 21 - all-zero frames (each zero byte = the safe value, see the comment)
  memset(d, 0, 8);
  lrSend(LR_FL2_315_ID, d);     // [KM -] [REA #34: b3[1:0]:b4 x 25 -> F157C = 0; b0 bits 5-6 DID only]
  lrSend(LR_FL2_INHIBIT_ID, d); // [KM -] [REA #35: b7 bit 3 = 0 -> F158E = 0 - REQUIRED: set = no base demand + monitors held]
  lrSend(LR_EVQ_34F_ID, d);     // [KM -] [REA E5: b7 bits 1-3 mode request = 0]
  lrSend(LR_EVQ_369_ID, d);     // [KM -] [REA E6/E7: b5[3:0] = 0 - NOT 6 / 10 (ramp request that beats the lock request), 0xC, 3; b0 bit 6 = 0]
  lrSend(LR_EVQ_496_ID, d);     // [KM -] [REA trackG: b2 bit 6 only, role INF -> 0]
  lrSend(LR_094_ID, d);         // [KM -] [REA #28: FL2 b6:b7 -> F155E = 0 (sign b0 bit 6); EVQ b6:b7 / b3:b4 = 0, sign b5 bit 5]

  // block 16 - 0x39B (FL2) and block 17 - 0x3EE (both)
  d[6] = 0x00;                  // [KM -] [REA #36: b6:b7 -> F1578 = (raw - 1) x 256 / 10, F1532 = raw != 0: 1 = "0, valid" [GUESS - bench: meaning]]
  d[7] = 0x01;
  lrSend(LR_FL2_39B_ID, d);
  memset(d, 0, 8);
  d[1] = 0x00;                  // [KM -] [REA #37: FL2 b1 bit 7 = 0 (F15A3, no mode 4), b1 bits 3-5 = 0 (mode request)]
  d[5] = 0x00;                  // [KM -] [REA E5: EVQ b5:b6 = FL2 0x39B role -> 1 as above [GUESS - bench]; EVQ b4 bit 5 / b2 bits 4-5 role INF -> 0]
  d[6] = 0x01;
  lrSend(LR_3EE_ID, d);

  // blocks 22 / 23 - CCF 0x400 / 0x405 every 100 ms (alive needed once rpm != 0 - FUN_029f24, #42)
  if (++lrTick >= 5)
  {
    lrTick = 0;
    memset(d, 0, 8);
    d[0] = LR_400_PAGES[lrPage & 0x03]; // [KM -] [REA #42: b0 = mux page 1 / 2 / 9 / 0x13 -> FUN_02f53c..; page content not traced - zeros [GUESS - bench]]
    lrSend(LR_CCF_400_ID, d);
    memset(d, 0, 8);
    if (lrPage & 0x01)
    {
      d[0] = 0x05;              // [KM -] [REA #38: page 5]
      d[6] = 0x08;              // [KM -] [REA #38: b6:b7 big-endian -> F1582 = (v x 1024 + 3141) / 6283: 0x08C6 = 2246 -> 366 = the unit's own default radius]
      d[7] = 0xC6;
    }
    else
    {
      d[0] = 0x02;              // [KM -] [REA #39: page 2]
      d[2] = 0xFF;              // [KM -] [REA #39: b2 - 40 -> F1536; 0xFF = no value -> cal 0x6100 = 30 (the unit's own default)]
      // b4 x 1000 / 4 -> FUN_02f536 (#39): 0 [GUESS - bench: content not traced]
    }
    lrSend(LR_CCF_405_ID, d);
    lrPage++;
  }
}

// --- Volvo Gen5 "SPC" 31380867 (generation 53, 500 kbit/s assumed, 11-bit) ---
// Added V11.00.0 from the unit's OEM code (spec_ppc_volvo_spc_bmw.md, Volvo part).
// Haldex replies on VOLVO_SPC_HALDEX_ID (0x126), decoded in parseCANHdx.
//
// The lock request: 0x041 bytes 6-7, signed 16-bit, high byte first, taken only with byte 5 bit 2 (update
// bit) set. V11.00.13 (audit_ppc_spc_bmw): it is a CEILING on the unit's own torque demand (only in
// its system state 3; 0 = no ceiling), proportional below ~3222 counts. Where the unit's own demand
// comes from (Ram40000D62/D60) is not traced yet, so the ceiling may only reach what that demand asks.
// 500 kbit/s confirmed (FlexCAN CTRL 0x005A0047). No checksum or counter is checked on anything it receives.
//
// What is NOT known yet: the content of the other 23 frames it listens to
// (ignition / status / variant inputs that gate the strategy - if one is
// missing or wrong the unit may stay open) and every frame's real send rate.
// They are sent as all-zero starting values so each byte can be tried live with
// the serial lab (OVR <id> <byte> <value>); set VOLVO_SPC_SEND_PLACEHOLDERS to 0
// to send only the lock request and the network frame.
// V11 TODO (bench): rates below are guesses (0x0xx 10 ms, 0x1xx/0x2xx 20 ms,
// 0x5xx 100 ms); confirm content + rates from a log of a car with this unit.
// No brake / handbrake bit is known in these frames, so none is sent.

// VOLVO_SPC_ZERO_DATA (starting value for the unknown frames): top of this file.

void VolvoSpc_frames10()
{
  uint8_t data[8];

  // lock request 0-100 % (0 while a lock gate holds lock off, the sweep position while Learn runs).
  // V11.00.13 (audit_ppc_spc_bmw #35/#39/#40): the value is a torque CEILING on the unit's own demand,
  // applied only in its system state 3; 0 = NO ceiling (not "open"), so 0 % sends 1 (a ceiling of ~0).
  // Proportional only below ~3222 counts (engage = v x 1024 / 0xA4F x 16 < CAL_c5fc 20000, variants 0-2);
  // above that the unit holds a value from its own torque or drops the ceiling - so 100 % = just under.
  const uint8_t lockPercent = lockRequestPercent();
  uint32_t lockRequestValue = ((uint32_t)VOLVO_SPC_REQUEST_FULL * lockPercent + 50) / 100; // round to nearest
  if (lockRequestValue == 0)
  {
    lockRequestValue = 1; // 0 would release the ceiling = the unit's own demand
  }

  // 0x041 - clutch torque ceiling. REA (v_com_041.txt): one COM signal (12) in this frame, DLC must be 8.
  data[0] = 0x00; // no signal (only signal 12 is mapped in 0x041) [KM -][REA v_com_041: not read]
  data[1] = 0x00; // no signal [KM -][REA: not read]
  data[2] = 0x00; // no signal [KM -][REA: not read]
  data[3] = 0x00; // no signal [KM -][REA: not read]
  data[4] = 0x00; // no signal [KM -][REA: not read]
  data[5] = 0x04; // bit 2 = COM update bit of signal 12: without it the unit never takes bytes 6-7 (REA #35) [KM -][REA sigref 9 mask 0xA190[1] = 0x04]
  data[6] = (uint8_t)((lockRequestValue >> 8) & 0xFF); // signal 12 high byte (s16 Motorola; the unit uses |value|) [KM -][REA #35/#38-#40: -> Ram40000F42 -> port 46; ceiling, engage = |v| x 1024 / 0xA4F x 16, cap at CAL_c5fc]
  data[7] = (uint8_t)(lockRequestValue & 0xFF);        // signal 12 low byte [KM -][REA #35: b6 << 8 | b7]
  sendVolvoSaabFrame(VOLVO_SPC_REQUEST_ID, false, data);

  if (VOLVO_SPC_SEND_PLACEHOLDERS)
  {
    sendVolvoSaabFrame(VOLVO_SPC_04A_ID, false, VOLVO_SPC_ZERO_DATA); // starting value, content unknown
    sendVolvoSaabFrame(VOLVO_SPC_050_ID, false, VOLVO_SPC_ZERO_DATA); // starting value, content unknown
    sendVolvoSaabFrame(VOLVO_SPC_056_ID, false, VOLVO_SPC_ZERO_DATA); // starting value, content unknown
    sendVolvoSaabFrame(VOLVO_SPC_066_ID, false, VOLVO_SPC_ZERO_DATA); // starting value, content unknown
    sendVolvoSaabFrame(VOLVO_SPC_071_ID, false, VOLVO_SPC_ZERO_DATA); // starting value, content unknown
    sendVolvoSaabFrame(VOLVO_SPC_076_ID, false, VOLVO_SPC_ZERO_DATA); // starting value, content unknown
    sendVolvoSaabFrame(VOLVO_SPC_081_ID, false, VOLVO_SPC_ZERO_DATA); // starting value, content unknown
    sendVolvoSaabFrame(VOLVO_SPC_086_ID, false, VOLVO_SPC_ZERO_DATA); // starting value, content unknown
  }
}

void VolvoSpc_frames20()
{
  if (VOLVO_SPC_SEND_PLACEHOLDERS)
  {
    sendVolvoSaabFrame(VOLVO_SPC_131_ID, false, VOLVO_SPC_ZERO_DATA); // starting value, content unknown
    sendVolvoSaabFrame(VOLVO_SPC_155_ID, false, VOLVO_SPC_ZERO_DATA); // starting value, content unknown
    sendVolvoSaabFrame(VOLVO_SPC_158_ID, false, VOLVO_SPC_ZERO_DATA); // starting value, content unknown
    sendVolvoSaabFrame(VOLVO_SPC_161_ID, false, VOLVO_SPC_ZERO_DATA); // starting value, content unknown
    sendVolvoSaabFrame(VOLVO_SPC_165_ID, false, VOLVO_SPC_ZERO_DATA); // starting value, content unknown
    sendVolvoSaabFrame(VOLVO_SPC_171_ID, false, VOLVO_SPC_ZERO_DATA); // starting value, content unknown
    sendVolvoSaabFrame(VOLVO_SPC_200_ID, false, VOLVO_SPC_ZERO_DATA); // starting value, content unknown
    sendVolvoSaabFrame(VOLVO_SPC_211_ID, false, VOLVO_SPC_ZERO_DATA); // starting value, content unknown
    sendVolvoSaabFrame(VOLVO_SPC_231_ID, false, VOLVO_SPC_ZERO_DATA); // starting value, content unknown
    sendVolvoSaabFrame(VOLVO_SPC_235_ID, false, VOLVO_SPC_ZERO_DATA); // starting value, content unknown [KM V60 wheelspeed0 b4-5 RF / b6-7 LF (ID match only) - zeros = -327.68 km/h there]
    sendVolvoSaabFrame(VOLVO_SPC_281_ID, false, VOLVO_SPC_ZERO_DATA); // starting value, content unknown
    sendVolvoSaabFrame(VOLVO_SPC_285_ID, false, VOLVO_SPC_ZERO_DATA); // starting value, content unknown
  }
}

void VolvoSpc_frames100()
{
  uint8_t data[8];

  // 0x501 - network management. The unit's CanNm listens to 0x500-0x53F (its
  // own frame is 0x51E); any frame in that range keeps its network awake.
  // Byte layout assumed from the usual AUTOSAR NM frame (byte 0 = sender
  // node number, byte 1 = control bits). V11 TODO (bench): confirm.
  // Every byte [KM -][REA #48: 0x500-0x53F accepted by CanIf range + masked MB; CanNm FUN_000328f4 byte layout NOT read]
  data[0] = 0x01; // sender node number (ours, made up - matches the ID 0x500 + 1)
  data[1] = 0x00; // control bits: no "repeat message" request
  data[2] = 0x00; // user data
  data[3] = 0x00; // user data
  data[4] = 0x00; // user data
  data[5] = 0x00; // user data
  data[6] = 0x00; // user data
  data[7] = 0x00; // user data
  sendVolvoSaabFrame(VOLVO_SPC_NM_ID, false, data);

  if (VOLVO_SPC_SEND_PLACEHOLDERS)
  {
    sendVolvoSaabFrame(VOLVO_SPC_57E_ID, false, VOLVO_SPC_ZERO_DATA); // starting value, content unknown
    sendVolvoSaabFrame(VOLVO_SPC_57F_ID, false, VOLVO_SPC_ZERO_DATA); // starting value, content unknown
    sendVolvoSaabFrame(VOLVO_SPC_587_ID, false, VOLVO_SPC_ZERO_DATA); // starting value, content unknown
  }
}

// --- SAAB 9-3 XWD (GM family, 500 kbit/s, 11-bit IDs) ------------------------
// Haldex replies on SAAB_93_HALDEX_ID (0x210), decoded in parseCANHdx.
// Uses the GM "alive rolling count" idea, like our Gen41: a 2-bit counter, and
// on 0x160 a protection byte that is 0 minus the counter.

// Static frame data and the 0x160 counter / protection tables: top of this file.

void Saab93_frames10()
{
  uint8_t data[8];

  // lock request: 0 = no lock, 0x89 = the strongest value seen (100 %)
  const uint8_t lockByte = getLockTargetAdjustedValue(0x89, false);
  const bool isLockActive = (lockByte > 0);

  // 0x140 - 2-bit rolling counter in byte 0 bits 0-1
  saab140Counter = (saab140Counter + 1) & 0x03; // count 0-3, moved on before sending
  data[0] = 0xC0 | saab140Counter;               // 0xC0 + counter
  data[1] = 0x03; // static
  data[2] = 0x20; // static
  data[3] = 0xFC; // static
  data[4] = 0xDD; // static
  data[5] = 0x00; // static
  data[6] = 0x00; // static
  data[7] = 0x00; // static
  sendVolvoSaabFrame(SAAB_93_140_ID, false, data);

  // 0x110 - bytes 1-2 are engine RPM x 4 on the 9-3 P-bus, so "lock active"
  // here is really 0 rpm / 0x1F40 = 2000 rpm. Behaviour unchanged for now.
  data[0] = 0x00; // static
  data[1] = 0x00; // 00 00 = 0 rpm (off), 1F 40 = 2000 rpm (lock active)
  data[2] = 0x00;
  data[3] = 0x00; // static
  data[4] = 0x00; // static
  data[5] = 0x08; // static
  data[6] = 0x00; // static
  data[7] = 0x00; // static
  if (isLockActive)
  {
    data[1] = 0x1F;
    data[2] = 0x40;
  }
  sendVolvoSaabFrame(SAAB_93_110_ID, false, data);

  // 0x120 - carries the lock request
  data[0] = 0x00;     // static
  data[1] = lockByte; // lock request
  data[2] = 0x00;     // 0x00 off, 0x83 lock active
  data[3] = 0x2D;     // static
  data[4] = 0x00;     // static
  data[5] = 0x5B;     // static
  data[6] = 0xC0;     // static
  data[7] = 0x00;     // static
  if (isLockActive)
  {
    data[2] = 0x83;
  }
  sendVolvoSaabFrame(SAAB_93_120_ID, false, data);

  // 0x160 - counter / protection pair in bytes 2-3
  data[0] = 0x00;                           // static
  data[1] = 0x00;                           // static
  data[2] = SAAB_93_160_BYTE2[saab160Step]; // counter
  data[3] = SAAB_93_160_BYTE3[saab160Step]; // protection value
  data[4] = 0x00;                           // static
  data[5] = 0x00;                           // static
  data[6] = 0x00;                           // static
  data[7] = 0x00;                           // static
  sendVolvoSaabFrame(SAAB_93_160_ID, false, data);
  saab160Step = (saab160Step + 1) & 0x03; // next pair, 0-3

  sendVolvoSaabFrame(SAAB_93_288_ID, false, SAAB_93_288_DATA);
  sendVolvoSaabFrame(SAAB_93_180_ID, false, SAAB_93_180_DATA);
}

void Saab93_frames20()
{
  sendVolvoSaabFrame(SAAB_93_280_ID, false, SAAB_93_280_DATA);
  sendVolvoSaabFrame(SAAB_93_300_ID, false, SAAB_93_300_DATA);
}

void Saab93_frames100()
{
  sendVolvoSaabFrame(SAAB_93_301_ID, false, SAAB_93_301_DATA);
  sendVolvoSaabFrame(SAAB_93_410_ID, false, SAAB_93_410_DATA);
  sendVolvoSaabFrame(SAAB_93_3ED_ID, false, SAAB_93_3ED_DATA);
  sendVolvoSaabFrame(SAAB_93_320_ID, false, SAAB_93_320_DATA);
}

void Saab93_frames1000()
{
  sendVolvoSaabFrame(SAAB_93_510_ID, false, SAAB_93_510_DATA);
  sendVolvoSaabFrame(SAAB_93_398_ID, false, SAAB_93_398_DATA);
}

// =============================================================================
// Volvo P1 (S40 / V50 AWD, generation 31) Standalone Frames (V11.00.1)
// =============================================================================
// Built from Adam's 2005 V50 T5 AWD logs (documents/vehicle-logs/gen3/
// P1_V50_T5_AWD_DECODE.md). Standalone sends EVERY frame the car puts on its
// high-speed bus (22 of them), so the AWD module sees a whole car. Each one is a
// frame-edit block. A block switched off doesn't go quiet: the CAR's own frame
// with that ID is passed through to the module instead (parseCANChs). So with
// every block off the module sees the real car, and switching our frames on one
// at a time on a drive shows which of ours it won't accept.
//
// What goes in them:
//   - Steady bytes: a real frame of each, taken from Drive.csv at 56.9 s
//     (30 km/h, light throttle, no brake, wheel straight).
//   - Counters / toggles: copied from the full C30 PCAN trace (idle.trc), which
//     caught every frame, so the patterns are exact.
//   - The lock: chosen on the Settings page (Volvo P1 Standalone card, saved):
//       Engine Torque - torque + pedal rise with the lock. In the logs the
//                       module follows torque almost 1:1, so this is the default.
//       Wheel Slip    - the front wheels run faster than the rears (wheelspin).
//       Both          - the two together.
//     lockRequestPercent() gives the lock % (Learn sweep, learn table and every
//     lock gate already applied), exactly as on the other direct-lock request units.
//   - Speed: the Road Speed setting on the rear wheels and the road speed frame;
//     the fronts the same unless Wheel Slip is in use.
//   - Engine rpm: Road Speed x the Gear setting's ratio (1st-3rd, from the logs),
//     so rpm and speed always match a real gear.
//   - Steering: straight ahead and still, with the sensor's own check byte.
//   - Brake / handbrake: from Follow Brake / Follow Handbrake.
// The AWD module answers on 0x1960000A, decoded in parseCANHdx.
//
// Send times: 10 ms frames go out every 10 ms, the car's 14-15 ms frames every
// 13 ms, 20 ms every 20 ms, 28 ms every 25 ms, 50 ms every 50 ms - never slower
// than the car. The slow ones (80 ms and up) count 10 ms steps.
// =============================================================================

// 0x0300410E - engine: two torques, brake switch, accelerator pedal (follow the lock)
static void volvoP1SendEngineTorque(uint8_t lockPercent)
{
  uint8_t data[8];
  volvoP1Copy(data, VOLVO_P1_0300410E_DATA);

  // engine torque, 12-bit with 2048 = 0 (scale not known): 0 at 0 % lock,
  // Torque at 100 % (setting) at full lock
  uint8_t enginePercent = volvoP1EngineLockPercent(lockPercent);
  uint16_t torqueAboveZero = volvoP1Scale(0, volvoP1TorqueFull, enginePercent);
  uint16_t torque = 2048 + torqueAboveZero;
  uint8_t pedal = (uint8_t)volvoP1Scale(0, VOLVO_P1_REQUEST_PEDAL_FULL, enginePercent);

  data[0] = 0x90 | ((torque >> 8) & 0x0F); // top nibble steady (9), bottom = torque top 4 bits [KM -][REA none][LOG]
  data[1] = torque & 0xFF;                 // torque bottom 8 bits [KM -][REA none][LOG: module follows it ~1:1]
  data[4] = 0xA0 | ((torque >> 8) & 0x0F); // top nibble A = brake off, bottom = second torque top 4 bits [KM -][REA none][LOG]
  data[5] = torque & 0xFF;                 // second torque bottom 8 bits (the car keeps the two within a few counts) [KM -][REA none][LOG]
  data[7] = pedal;                         // accelerator pedal, 0-255 [KM -][REA none][LOG: 231 = full throttle]
  // bytes 2, 3, 6: steady table values [KM -][REA none]

  volvoP1SentTorque = torqueAboveZero; // for the live readout
  volvoP1SentPedal = pedal;

  if (brakeForHaldex())
  {
    data[4] = data[4] | (1 << 4); // byte 4 bit 4 = brake switch
    data[4] = data[4] | (1 << 6); // byte 4 bit 6 = second brake switch (always together in the logs)
  }
  sendVolvoSaabFrame(VOLVO_P1_ENGINE_TORQUE_ID, true, data);
}

// 0x0110483C - handbrake in byte 2 bit 6; byte 2 bit 7 set on every 5th send
static void volvoP1SendHandbrake()
{
  uint8_t data[8];
  volvoP1Copy(data, VOLVO_P1_0110483C_DATA);
  if (volvoP1Tick10 % 5 == 0)
  {
    data[2] = data[2] | (1 << 7); // every 5th send, like the car
  }
  if (handbrakeForHaldex())
  {
    data[2] = data[2] | (1 << 6); // handbrake on
  }
  sendVolvoSaabFrame(VOLVO_P1_HANDBRAKE_ID, true, data);
}

// 0x0100082C - steady, byte 4 bit 4 set on every 10th send
static void volvoP1Send0100082C()
{
  uint8_t data[8];
  volvoP1Copy(data, VOLVO_P1_0100082C_DATA);
  if (volvoP1Tick10 % 10 == 0)
  {
    data[4] = data[4] | (1 << 4); // every 10th send, like the car
  }
  sendVolvoSaabFrame(VOLVO_P1_0100082C_ID, true, data);
}

// 0x0340412E - steering angle sensor (it was called the steering pump frame
// before the decode). Sent as "straight ahead and not turning":
//   byte 0   0x30 + 0-15 counter (bits 4-5 = 11 as on the car, bit 6 = direction, 0)
//   bytes 1-2 fine steering rate (bit 7 of byte 1 = direction): 0
//   byte 3   check byte (above)
//   bytes 4-5 coarse steering rate (bit 7 of byte 4 = direction): 0x80 00 = 0, as the V50 sends when still
//   bytes 6-7 steering angle size: 0 = straight
static void volvoP1SendSteerSensor()
{
  uint8_t data[8];
  data[0] = 0x30 | volvoP1SteerSensorCount; // counter 0-15
  data[1] = 0x00;                           // fine rate, top byte
  data[2] = 0x00;                           // fine rate, bottom byte
  data[3] = 0x00;                           // check byte - worked out below, once the rest is filled in
  data[4] = 0x80;                           // coarse rate, top byte (sign bit set, value 0, as logged)
  data[5] = 0x00;                           // coarse rate, bottom byte
  data[6] = 0x00;                           // angle, top byte
  data[7] = 0x00;                           // angle, bottom byte
  data[3] = volvoP1SteerSensorCheck(data);
  sendVolvoSaabFrame(VOLVO_P1_STEER_SENSOR_ID, true, data);
  volvoP1SteerSensorCount = (volvoP1SteerSensorCount + 1) & 0x0F; // next count, 0-15
}

// 0x1BE0493C - one page every 250 ms: page 05, then page 01 (clock and
// heartbeat), then page 02, then a gap. Each page comes once a second.
static void volvoP1SendNetwork()
{
  uint8_t data[8];

  if (volvoP1NetworkSlot == 0)
  {
    sendVolvoSaabFrame(VOLVO_P1_NETWORK_ID, true, VOLVO_P1_NETWORK_PAGE05); // page 05
  }
  else if (volvoP1NetworkSlot == 1)
  {
    // page 01: a clock in bytes 2-5 (+10 a second) and the heartbeat in bytes 6-7
    volvoP1NetworkClock = volvoP1NetworkClock + 10;
    data[0] = 0x01;                                     // page 01
    data[1] = 0x10;                                     // steady
    data[2] = (volvoP1NetworkClock >> 24) & 0xFF;       // clock, top byte
    data[3] = (volvoP1NetworkClock >> 16) & 0xFF;       // clock
    data[4] = (volvoP1NetworkClock >> 8) & 0xFF;        // clock
    data[5] = volvoP1NetworkClock & 0xFF;               // clock, bottom byte
    data[6] = 0x02;                                     // heartbeat: 02 04 ...
    data[7] = 0x04;
    if (volvoP1NetworkHeartbeat)
    {
      data[6] = 0xFD;                                   // ... then its exact opposite, FD FB
      data[7] = 0xFB;
    }
    volvoP1NetworkHeartbeat = !volvoP1NetworkHeartbeat;
    sendVolvoSaabFrame(VOLVO_P1_NETWORK_ID, true, data);
  }
  else if (volvoP1NetworkSlot == 2)
  {
    sendVolvoSaabFrame(VOLVO_P1_NETWORK_ID, true, VOLVO_P1_NETWORK_PAGE02); // page 02, status
  }
  // slot 3: nothing, like the car

  volvoP1NetworkSlot = (volvoP1NetworkSlot + 1) & 0x03; // next slot, 0-3
}

// The slow frames (80 ms and up): counted in 10 ms steps
static void volvoP1SlowFrames()
{
  uint8_t data[8];

  // 0x1B200002 - steady, every 80 ms (85 ms on the car)
  if (volvoP1Tick10 % 8 == 0)
  {
    sendVolvoSaabFrame(VOLVO_P1_1B200002_ID, true, VOLVO_P1_1B200002_DATA);
  }

  // 0x1B600002 - every 170 ms: byte 0 bit 5 on every 6th send, byte 1 6-step pattern
  if (volvoP1Tick10 % 17 == 0)
  {
    volvoP1Copy(data, VOLVO_P1_1B600002_DATA);
    data[1] = VOLVO_P1_1B600002_BYTE1[volvoP1Count1B6]; // byte 1 pattern step
    if (volvoP1Count1B6 == 0)
    {
      data[0] = data[0] | (1 << 5); // byte 0 bit 5, every 6th send
    }
    sendVolvoSaabFrame(VOLVO_P1_1B600002_ID, true, data);
    volvoP1Count1B6++;
    if (volvoP1Count1B6 >= 6)
    {
      volvoP1Count1B6 = 0; // pattern starts again
    }
  }

  // 0x1B700030 - every 210 ms: byte 0 bit 4 on every 3rd send
  if (volvoP1Tick10 % 21 == 0)
  {
    volvoP1Copy(data, VOLVO_P1_1B700030_DATA);
    if (volvoP1Count1B7 == 0)
    {
      data[0] = data[0] | (1 << 4); // 6F -> 7F every 3rd send
    }
    sendVolvoSaabFrame(VOLVO_P1_1B700030_ID, true, data);
    volvoP1Count1B7++;
    if (volvoP1Count1B7 >= 3)
    {
      volvoP1Count1B7 = 0;
    }
  }

  // 0x1A200020 - every 310 ms (315 on the car): byte 0 bits 6-7 = 2-bit counter
  if (volvoP1Tick10 % 31 == 0)
  {
    volvoP1Copy(data, VOLVO_P1_1A200020_DATA);
    data[0] = (uint8_t)((VOLVO_P1_1A200020_DATA[0] & 0x3F) | (volvoP1Count1A2 << 6)); // counter in bits 6-7
    sendVolvoSaabFrame(VOLVO_P1_1A200020_ID, true, data);
    volvoP1Count1A2 = (volvoP1Count1A2 + 1) & 0x03; // next count, 0-3
  }

  // 0x1AE0092C - every 420 ms: byte 0 bits 6-7 = 2-bit counter
  if (volvoP1Tick10 % 42 == 0)
  {
    volvoP1Copy(data, VOLVO_P1_1AE0092C_DATA);
    data[0] = (uint8_t)((VOLVO_P1_1AE0092C_DATA[0] & 0x3F) | (volvoP1Count1AE << 6)); // counter in bits 6-7
    sendVolvoSaabFrame(VOLVO_P1_1AE0092C_ID, true, data);
    volvoP1Count1AE = (volvoP1Count1AE + 1) & 0x03; // next count, 0-3
  }

  // 0x1BA0493C - every 250 ms, the 20 pages in turn
  if (volvoP1Tick10 % 25 == 0)
  {
    sendVolvoSaabFrame(VOLVO_P1_1BA0493C_ID, true, VOLVO_P1_1BA0493C_PAGES[volvoP1Page1BA]);
    volvoP1Page1BA++;
    if (volvoP1Page1BA >= 20)
    {
      volvoP1Page1BA = 0; // back to page 1
    }
  }

  // 0x1BE0493C - every 250 ms, network heartbeat / VIN pages
  if (volvoP1Tick10 % 25 == 0)
  {
    volvoP1SendNetwork();
  }
}

void VolvoP1_frames10()
{
  // lock % to ask for: 0-100, with the learn table and every lock gate applied
  const uint8_t lockPercent = lockRequestPercent();

  volvoP1SendEngineTorque(lockPercent); // 0x0300410E - torque + pedal follow the lock
  volvoP1SendHandbrake();               // 0x0110483C
  volvoP1Send0100082C();                // 0x0100082C
  volvoP1SendSteerSensor();             // 0x0340412E
  volvoP1SlowFrames();                  // 80 ms and slower

  volvoP1Tick10++;
}

// The car's 14-15 ms frames, sent every 13 ms
void VolvoP1_frames13()
{
  uint8_t data[8];
  const uint8_t lockPercent = lockRequestPercent();
  uint16_t rearRaw = volvoP1RearSpeedRaw();             // Road Speed setting
  uint16_t frontRaw = volvoP1FrontSpeedRaw(lockPercent); // the same, plus any Wheel Slip
  volvoP1SentRearRaw = rearRaw;   // for the live readout
  volvoP1SentFrontRaw = frontRaw;

  // 0x01C04026 - four wheel speeds, 16-bit, 0x8000 = stopped, order RL RR FL FR
  // bytes 0-7 [KM -][REA none][LOG: 0.01 km/h, order from the V50 traces]
  uint16_t rearWord = 0x8000 + rearRaw;
  uint16_t frontWord = 0x8000 + frontRaw;
  data[0] = (rearWord >> 8) & 0xFF;  // rear left, high byte
  data[1] = rearWord & 0xFF;         // rear left, low byte
  data[2] = (rearWord >> 8) & 0xFF;  // rear right, high byte
  data[3] = rearWord & 0xFF;         // rear right, low byte
  data[4] = (frontWord >> 8) & 0xFF; // front left, high byte
  data[5] = frontWord & 0xFF;        // front left, low byte
  data[6] = (frontWord >> 8) & 0xFF; // front right, high byte
  data[7] = frontWord & 0xFF;        // front right, low byte
  sendVolvoSaabFrame(VOLVO_P1_WHEELS_ID, true, data);

  // 0x02104136 - road speed in bytes 6-7 (follows the rear wheels, which don't spin)
  volvoP1Copy(data, VOLVO_P1_02104136_DATA);
  data[6] = (rearRaw >> 8) & 0xFF; // speed high byte [KM -][REA none][LOG]
  data[7] = rearRaw & 0xFF;        // speed low byte [KM -][REA none][LOG]
  sendVolvoSaabFrame(VOLVO_P1_SPEED_ID, true, data);

  // 0x02000026 - brake pressure (byte 5 + byte 4 bits 6-7). Released = as logged.
  volvoP1Copy(data, VOLVO_P1_02000026_DATA);
  if (brakeForHaldex())
  {
    data[2] = 0xAD; // braking: a real pair of values from the wheelspin log [KM -][REA none][LOG]
    data[5] = 0x35; // brake pressure byte (released = 0x00) [KM -][REA none][LOG]
  }
  sendVolvoSaabFrame(VOLVO_P1_BRAKE_ID, true, data);

  // 0x02804026 - steering angle and yaw, both straight ahead
  sendVolvoSaabFrame(VOLVO_P1_STEERING_ID, true, VOLVO_P1_02804026_DATA);

  // 0x02C00020 - steady
  sendVolvoSaabFrame(VOLVO_P1_02C00020_ID, true, VOLVO_P1_02C00020_DATA);

  // 0x19000026 - byte 6 bit 3 flips every send (15 ms on the car)
  volvoP1Copy(data, VOLVO_P1_19000026_DATA);
  if (volvoP1Toggle13)
  {
    data[6] = data[6] | (1 << 3); // 86 -> 8E
  }
  volvoP1Toggle13 = !volvoP1Toggle13;
  sendVolvoSaabFrame(VOLVO_P1_19000026_ID, true, data);
}

void VolvoP1_frames20()
{
  uint8_t data[8];

  // 0x0090411E - engine rpm from Road Speed and Gear (not the lock, so it always
  // matches a real gear); byte 0 bit 7 on every 10th send
  volvoP1Copy(data, VOLVO_P1_0090411E_DATA);
  uint16_t rpm = volvoP1GearRpm();
  volvoP1SentRpm = rpm; // for the live readout
  data[2] = 0xE0 | ((rpm >> 8) & 0x1F); // top 3 bits steady, bottom 5 = rpm top bits [KM -][REA none][LOG]
  data[3] = rpm & 0xFF;                 // rpm bottom 8 bits [KM -][REA none][LOG: 1 rpm/bit]
  if (volvoP1Count20 % 10 == 0)
  {
    data[0] = data[0] | (1 << 7); // every 10th send, like the car
  }
  sendVolvoSaabFrame(VOLVO_P1_ENGINE_RPM_ID, true, data);

  // 0x00800006 - byte 0 bits 6-7 = 2-bit counter
  volvoP1Copy(data, VOLVO_P1_00800006_DATA);
  data[0] = (uint8_t)((VOLVO_P1_00800006_DATA[0] & 0x3F) | ((volvoP1Count20 & 0x03) << 6)); // counter in bits 6-7
  sendVolvoSaabFrame(VOLVO_P1_00800006_ID, true, data);

  // 0x03800006 - byte 0 bit 6 on every 5th send
  volvoP1Copy(data, VOLVO_P1_03800006_DATA);
  if (volvoP1Count20 % 5 == 0)
  {
    data[0] = data[0] | (1 << 6); // 1A -> 5A every 5th send
  }
  sendVolvoSaabFrame(VOLVO_P1_03800006_ID, true, data);

  volvoP1Count20++;
  if (volvoP1Count20 >= 100)
  {
    volvoP1Count20 = 0; // 100 is a multiple of 4, 5 and 10, so every pattern carries on smoothly
  }
}

// 0x19A00002 - wheel pulse counters, every 25 ms (28 ms on the car). Byte 0 goes
// 02, 12, 6E, 12; the counters move on the 6E page (every 100 ms here).
void VolvoP1_frames25()
{
  uint8_t data[8];
  const uint8_t pages[4] = {0x02, 0x12, 0x6E, 0x12}; // byte 0, in order

  if (volvoP1PulsePage == 2)
  {
    // pulses in 100 ms at the speeds being sent. Logs: about speed / 150 pulses
    // every 112 ms, so speed / 168 every 100 ms - kept in hundredths so none get lost.
    volvoP1PulseHundredths = volvoP1PulseHundredths + (uint16_t)((uint32_t)volvoP1SentRearRaw * 100 / 168);
    uint8_t newPulses = volvoP1PulseHundredths / 100;                            // whole rear pulses to add now
    volvoP1PulseHundredths = volvoP1PulseHundredths - (uint16_t)newPulses * 100; // keep the part-pulse for next time

    volvoP1FrontPulseHundredths = volvoP1FrontPulseHundredths + (uint16_t)((uint32_t)volvoP1SentFrontRaw * 100 / 168);
    uint8_t newFrontPulses = volvoP1FrontPulseHundredths / 100;                               // whole front pulses
    volvoP1FrontPulseHundredths = volvoP1FrontPulseHundredths - (uint16_t)newFrontPulses * 100; // keep the part-pulse

    // counters 1-2 = rear, 3-4 = front (same order as the wheel speeds; wraps at 255)
    volvoP1PulseCounter[0] = volvoP1PulseCounter[0] + newPulses;
    volvoP1PulseCounter[1] = volvoP1PulseCounter[1] + newPulses;
    volvoP1PulseCounter[2] = volvoP1PulseCounter[2] + newFrontPulses;
    volvoP1PulseCounter[3] = volvoP1PulseCounter[3] + newFrontPulses;

    // byte 5 goes up by one about every 48 pulses (from the logs)
    volvoP1DistancePulses = volvoP1DistancePulses + newPulses;
    while (volvoP1DistancePulses >= 48)
    {
      volvoP1DistancePulses = volvoP1DistancePulses - 48;
      volvoP1Distance++;
    }
  }

  data[0] = pages[volvoP1PulsePage];  // page
  data[1] = volvoP1PulseCounter[0];   // wheel pulse counter 1
  data[2] = volvoP1PulseCounter[1];   // wheel pulse counter 2
  data[3] = volvoP1PulseCounter[2];   // wheel pulse counter 3
  data[4] = volvoP1PulseCounter[3];   // wheel pulse counter 4
  data[5] = volvoP1Distance;          // distance
  data[6] = 0x00;                     // steady
  data[7] = 0xFF;                     // 0xFF moving (0xF7 stopped)
  sendVolvoSaabFrame(VOLVO_P1_PULSES_ID, true, data);

  volvoP1PulsePage = (volvoP1PulsePage + 1) & 0x03; // next page, 0-3
}

void VolvoP1_frames50()
{
  sendVolvoSaabFrame(VOLVO_P1_19E00006_ID, true, VOLVO_P1_19E00006_DATA); // 0x19E00006 - steady
}

// ---- Main code -----------------------------------------------------------------

// Periodic frame tasks
static void frames10()
{
  switch (haldexGeneration)
  {
  case 1:
    Gen1_frames10();
    break;
  case 2:
    Gen2_frames10();
    break;
  case 4:
    Gen4_frames10();
    break;
  case 41:
    Gen41_frames10();
    break;
  case 42:
    Gen42_frames10();
    break;
  case GEN_VOLVO_P2_GEN2:
    VolvoP2Gen2_frames10();
    break;
  case GEN_VOLVO_P2_GEN3:
    VolvoP2Gen3_frames10();
    break;
  case GEN_VOLVO_GEN4:
    VolvoGen4_frames10();
    break;
  case GEN_FORD_KUGA:
    FordKuga_frames10();
    break;
  case GEN_SAAB_93_XWD:
    Saab93_frames10();
    break;
  case GEN_VOLVO_SPC_GEN5:
    VolvoSpc_frames10();
    break;
  case GEN_VOLVO_P1:
    VolvoP1_frames10();
    break;
  case 50:
    Gen5_0CQ_frames10();
    break;
  case 51:
    Gen5_0AY_frames10();
    break;
  case 52:
    Gen5_0CQ_VAQ_frames10();
    break;
  }
}

static void frames20()
{
  switch (haldexGeneration)
  {
  case 1:
    Gen1_frames20();
    break;
  case 2:
    Gen2_frames20();
    break;
  case 4:
    Gen4_frames20();
    break;
  case 41:
    Gen41_frames20();
    break;
  case 42:
    Gen42_frames20();
    break;
  case GEN_VOLVO_P2_GEN2:
    VolvoP2Gen2_frames20();
    break;
  case GEN_SAAB_93_XWD:
    Saab93_frames20();
    break;
  case GEN_LANDROVER_GEN4:
    LandRover_frames20(); // V11.00.13: Land Rover FL2 / Evoque - whole union set (20 ms, CCF 100 ms inside)
    break;
  case GEN_VOLVO_SPC_GEN5:
    VolvoSpc_frames20();
    break;
  case GEN_VOLVO_P1:
    VolvoP1_frames20();
    break;
  case 50:
    Gen5_0CQ_frames20();
    break;
  case 51:
    Gen5_0AY_frames20();
    break;
  case 52:
    Gen5_0CQ_VAQ_frames20();
    break;
  }
}

static void frames25()
{
  switch (haldexGeneration)
  {
  case 1:
    Gen1_frames25();
    break;
  case 2:
    Gen2_frames25();
    break;
  case 4:
    Gen4_frames25();
    break;
  case 41:
    Gen41_frames25();
    break;
  case 50:
    Gen5_0CQ_frames25();
    break;
  case 51:
    Gen5_0AY_frames25();
    break;
  case 52:
    Gen5_0CQ_VAQ_frames25();
    break;
  case GEN_VOLVO_P1:
    VolvoP1_frames25();
    break;
  }
}

static void frames100()
{
  updateLockTarget(); // mode / triggers / release rate limit -> lockTarget (both modes, V11.00.7)
  switch (haldexGeneration)
  {
  case 1:
    Gen1_frames100();
    break;
  case 2:
    Gen2_frames100();
    break;
  case 4:
    Gen4_frames100();
    break;
  case 41:
    Gen41_frames100();
    break;
  case 42:
    Gen42_frames100();
    break;
  case GEN_VOLVO_P2_GEN2:
    VolvoP2Gen2_frames100();
    break;
  case GEN_VOLVO_P2_GEN3:
    VolvoP2Gen3_frames100();
    break;
  case GEN_VOLVO_GEN4:
    VolvoGen4_frames100();
    break;
  case GEN_SAAB_93_XWD:
    Saab93_frames100();
    break;
  case GEN_VOLVO_SPC_GEN5:
    VolvoSpc_frames100();
    break;
  case 50:
    Gen5_0CQ_frames100();
    break;
  case 51:
    Gen5_0AY_frames100();
    break;
  case 52:
    Gen5_0CQ_VAQ_frames100();
    break;
  }
}

static void frames200()
{
  switch (haldexGeneration)
  {
  case 1:
    Gen1_frames200();
    break;
  case 2:
    Gen2_frames200();
    break;
  case 4:
    Gen4_frames200();
    break;
  case 41:
    Gen41_frames200();
    break;
  case 42:
    Gen42_frames200();
    break;
  case 50:
    Gen5_0CQ_frames200();
    break;
  case 51:
    Gen5_0AY_frames200();
    break;
  case 52:
    Gen5_0CQ_VAQ_frames200();
    break;
  }
}

static void frames1000()
{
  switch (haldexGeneration)
  {
  case 1:
    Gen1_frames1000();
    break;
  case 2:
    Gen2_frames1000();
    break;
  case 4:
    Gen4_frames1000();
    break;
  case 41:
    Gen41_frames1000();
    break;
  case 42:
    Gen42_frames1000();
    break;
  case GEN_SAAB_93_XWD:
    Saab93_frames1000();
    break;
  case 50:
    Gen5_0CQ_frames1000();
    break;
  case 51:
    Gen5_0AY_frames1000();
    break;
  case 52:
    Gen5_0CQ_VAQ_frames1000();
    break;
  }
}

static void frames13()
{
  switch (haldexGeneration)
  {
  case 41:
    Gen41_frames13();
    break;
  case GEN_VOLVO_P1:
    VolvoP1_frames13(); // the P1's 14-15 ms frames
    break;
  }
}

static void frames50()
{
  switch (haldexGeneration)
  {
  case 41:
    Gen41_frames50();
    break;
  case GEN_VOLVO_P1:
    VolvoP1_frames50();
    break;
  }
}

static void frames250()
{
  switch (haldexGeneration)
  {
  case 41:
    Gen41_frames250();
    break;
  }
}

// Gen4.1 dual-bus: repeat the cached chassis 0x0C1 / 0x0C5 frames onto bus 0.
static void gen41Bus0Frames20()
{
  if (haldexGeneration != 41)
    return;

  extern twai_message_t gen41Bus0CacheC1;
  extern twai_message_t gen41Bus0CacheC5;
  extern bool gen41Bus0CacheValidC1;
  extern bool gen41Bus0CacheValidC5;
  extern portMUX_TYPE gen41Bus0CacheMux;

  twai_message_t c1, c5;
  bool hasC1, hasC5;
  taskENTER_CRITICAL(&gen41Bus0CacheMux);
  hasC1 = gen41Bus0CacheValidC1;
  hasC5 = gen41Bus0CacheValidC5;
  if (hasC1)
    c1 = gen41Bus0CacheC1;
  if (hasC5)
    c5 = gen41Bus0CacheC5;
  taskEXIT_CRITICAL(&gen41Bus0CacheMux);

  if (hasC1)
    twai_transmit_v2(twaiBus0, &c1, 0);
  if (hasC5)
    twai_transmit_v2(twaiBus0, &c5, 0);
}

// One task drives every standalone rate. These used to be ten tasks (one per
// rate, plus the Gen4.1 bus-0 repeater), each with its own stack and all of them
// held suspended for an entire OEM-mode drive; as one task they give ~23 KB back
// to the C6's shared SRAM (WiFi + BLE + web server). Buckets are listed in the
// old task-priority order, so a tick due for several rates sends them in the
// same order as before. Every TX is non-blocking, so no bucket stalls the rest.
void standaloneFramesTask(void *arg)
{
  struct Bucket
  {
    uint16_t periodMs;
    void (*run)();
    uint32_t dueMs;
  };
  Bucket buckets[] = {
      {10, frames10, 0},
      {13, frames13, 0},
      {20, gen41Bus0Frames20, 0},
      {20, frames20, 0},
      {25, frames25, 0},
      {50, frames50, 0},
      {100, frames100, 0},
      {200, frames200, 0},
      {250, frames250, 0},
      {1000, frames1000, 0},
  };

  while (1)
  {
    // V11.00.7: our frames go out whenever they are the Haldex's frames - in
    // standalone, and in normal mode outside Stock (a block ON = our frame, the
    // car's copy is held back in can.cpp). Never in analyzer mode (passive
    // bridge), Stock, or with the controller disabled.
    // V11.00.13: Gen1 - hand the 02D's sticky Bremse_4 ceiling back when we stop sending it
    Gen1_releaseCeiling(ownFramesActive() && haldexGeneration == 1 && gen1CeilingRequestActive());
    if (!ownFramesActive())
    {
      FordKuga_releaseOverride(); // V11.00.11: re-arms the Kuga's one-shot 0x6F1 release for the next start
      vTaskDelay(10 / portTICK_PERIOD_MS);
      continue;
    }
    if (haldexGeneration != GEN_FORD_KUGA)
    {
      FordKuga_releaseOverride(); // generation changed away from the Kuga: release again when it comes back
    }

    const uint32_t now = millis();
    for (Bucket &b : buckets)
    {
      if ((int32_t)(now - b.dueMs) < 0)
        continue;
      b.run();
      b.dueMs += b.periodMs;
      if ((int32_t)(now - b.dueMs) >= 0) // first run, or back from suspend / analyzer: restart the cadence, don't burst
        b.dueMs = now + b.periodMs;
    }

    vTaskDelay(1); // one tick
  }
}
