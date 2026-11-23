#include <OpenHaldexC6_can.h>
#include <OpenHaldexC6_Calculations.h>
#include <OpenHaldexC6_Analyzer.h>
#include <OpenHaldexC6_UDS.h>
#include <OpenHaldexC6_ESPNow.h> // espNowTapFrame()

// =============================================================================
// Bus 0 - chassis (V11.00.14: split out of OpenHaldexC6_can.cpp; the non-VW
// decoders came in from OpenHaldexC6_ChassisOther.cpp, V10.00.0)
// =============================================================================
// Every frame the car sends: read what we need from it for the active generation
// (chsDecodeChassis - one case per generation), answer our own diagnostic IDs,
// then decide what the Haldex gets (car's frame, edited frame, or ours).
// Fills: receivedVehicleSpeed (km/h), receivedPedalValue (%), receivedVehicleRpm,
//        brakeFromCAN, handbrakeFromCAN, absBrakingFromCAN, tc/asr force flags, ...
// Bytes are numbered 0-7 (data[0] is the first byte on the wire).
//
//   41 Vauxhall / Opel Insignia (GM)  - checked against our labelled Insignia logs
//   42 Ford Focus RS                  - checked against our Focus RS drive log and
//                                       the labelled brake / handbrake / throttle logs
//   43 Volvo P3                       - from volvo_v60_2015_pt.dbc, NOT checked on a car yet
//   44 SAAB 9-3 (P-bus)               - from a published decode of a 2007 9-3 Aero (real car)
//   21 Volvo P2 pre-facelift          - 2002 model-year ID set only (Volvo moves IDs
//                                       between model years); from a 2002 S60 drive log
//   31 Volvo P1 (S40 / V50 AWD)       - from Adam's 2005 V50 T5 AWD logs (V11.00.1)
//   45 Ford Kuga Mk1                  - same Ford C1 IDs as the Focus RS decode + ABS from 0x211
//                                       (the AWD module's own firmware, trackF) - V11.00.9
//   46 Land Rover FL2 / Evoque       - from the Haldex unit's own firmware (audit_landrover), NOT checked
//                                       on a car; layout picked by the feedback ID (0x310 FL2 / 0x301 EVQ)
//   1 / 2 / 4 / 50 / 51 / 52 VW PQ + MQB  - vw_pq.dbc / vw_mqb.dbc / K-matrix (chsDecodeVag)
//   3 Volvo P2 facelift, 53 Volvo SPC  - no chassis decode yet
// =============================================================================

// ---- Non-VW chassis IDs (VW IDs are in canID.h) -------------------------------

// ----- Vauxhall / Opel Insignia (generation 41) -----
#define INSIGNIA_ENGINE_ID 0x0C9      // ECM: engine RPM (bytes 1-2) and accelerator pedal (byte 4)
#define INSIGNIA_BRAKE_ID 0x0F1       // brake pedal: byte 0 bit 1 = brake switch
#define INSIGNIA_HANDBRAKE_ID 0x1F1   // byte 4 bit 4 = electric parking brake applied
#define INSIGNIA_REAR_WHEELS_ID 0x34A // rear wheel speeds: left bytes 0-1, right bytes 2-3, 1/32 km/h per bit

// ----- Ford Focus RS (generation 42) -----
#define FOCUS_ENGINE_ID 0x201    // PCM: RPM bytes 0-1, vehicle speed bytes 4-5 (0.01 km/h), pedal byte 6 (0.5 %)
#define FOCUS_BRAKE_ID 0x205     // byte 2 bit 6 = brake switch, 1 = pressed (labelled brake log)
#define FOCUS_HANDBRAKE_ID 0x433 // byte 3 bit 0 = handbrake, 1 = applied (labelled handbrake log)

// ----- Volvo P3 (generation 43) -----
#define VOLVO_P3_PEDAL_ID 0x020  // AccPedal: 10 bits from byte 2 bits 1-0 and byte 3, 0.1 % per bit [KM V60 AccPedal.AccPedal 17|10 0.1 %][REA none]
#define VOLVO_P3_ENGINE_ID 0x12A // EngineData: brake switch byte 4 bit 6, RPM 13 bits from byte 6 bits 4-0 and byte 7 [KM V60 EngineData.BrakePressed 38|1, .EngineRpm 52|13 1 rpm][REA none]
#define VOLVO_P3_SPEED_ID 0x148  // VehicleSpeed1: bytes 6-7, 0.01 km/h per bit [KM V60 VehicleSpeed1.VehicleSpeed 55|16 0.01][REA none]
// V11.00.5: the real P3 bus is 29-bit (bench, documents/VOLVO_GEN4_BENCH_2026-11-07.md;
// full decode in canID.h next to VOLVO_G4_*). The 11-bit IDs above came from the V60 DBC
// and are kept only in case a car does carry them.
// The 29-bit frames read here use the canID.h names (V11.00.8 - they had a second set of names here):
//   VOLVO_G4_ENGINE_ID 0x0080401E  engine rpm b2[4:0]:b3 (idles ~650-870 in the S60 log); brake switch b6 bit 3
//   VOLVO_G4_TORQUE_ID 0x0062401E  accelerator pedal b6[1:0]:b7 (0 = foot off, 223 on a hard pull)
//   VOLVO_G4_WHEELS_ID 0x10004002  RR b0-1, RL b2-3, FR b4-5, FL b6-7: 14-bit, 1/64 km/h; RL b2 bits 7-6 = 1 / 2 while ABS / stability control brakes
#define VOLVO_P3_STEER29_ID 0x11600002  // steering angle b2-3 (centre ~0x62A0, ~0.4 deg per count - ESTIMATE from one turn in the S60 log)
#define VOLVO_P3_STEER_CENTRE 0x62A0    // straight-ahead value in the S60 log - check on a P3 car
#define VOLVO_P3_STEER_DEG_PER_COUNT 0.4f

// ----- SAAB 9-3 XWD, P-bus (generation 44) -----
#define SAAB_93_ENGINE_ID 0x110    // bytes 1-2 = engine RPM x 4
#define SAAB_93_WHEELS_ID 0x280    // four wheel speeds, 16-bit each (bytes 0-1, 2-3, 4-5, 6-7), raw / 50 = mph
#define SAAB_93_HANDBRAKE_ID 0x380 // byte 5 bit 6 = handbrake applied (0x46 on, 0x06 off)

// ----- Volvo P2 pre-facelift, 2002 model-year IDs (generation 21, 29-bit) -----
// [KM -][REA none] for all four: 2002 S60 logs only (no KMatrix, no dump). Note the 2002 set differs from
// the MY2003 set gen 21's standalone frames use (VOLVO_GEN2_VS_GEN4_LOGS_2026-11-11.md).
#define VOLVO_P2_2002_SPEED_ID 0x00D28026  // bytes 6-7 = vehicle speed, 0.01 km/h per bit
#define VOLVO_P2_2002_ENGINE_ID 0x0032C01E // engine RPM = 13 bits from byte 6 bits 4-0 and byte 7
#define VOLVO_P2_2002_BRAKE_ID 0x0012C024  // byte 4 bit 5 = brake switch
#define VOLVO_P2_2002_PEDAL_ID 0x00E00020  // byte 3 = accelerator pedal: 19 at rest, about 187 floored
#define VOLVO_P2_2002_PEDAL_REST 19        // pedal byte with the foot off (2002 S60 logs)
#define VOLVO_P2_2002_PEDAL_FULL 187       // pedal byte floored (2002 S60 logs)

// ---- Statics -----------------------------------------------------------------

// ===== Our frames vs the car's (V11.00.7) =====
// Every frame-edit generation, both modes: a block ON = we send our whole frame
// for that ID (standalone builders) and the car's copy is held back here; OFF =
// the car's own frame goes to the Haldex untouched. This used to be the Volvo
// P1 / Gen4 standalone "frame bisect" (V11.00.1); the VAG units only edited a
// few bytes of the car's frame in normal mode, so a block "on" could leave the
// car's brake / handbrake / gear bits reaching the unit.
// Car IDs not in the block table ("unlisted"): normal mode passes them,
// standalone holds them back (the bench bus has only our frames); Long Learn
// steers them with unlistedPassMode; heldBackIds never pass.
// Everything is captured on the way so Long Learn can report the car's bytes
// next to ours.
CarFrameSeen carBlockFrame[CAR_BLOCK_SLOTS];
uint8_t ourBlockData[CAR_BLOCK_SLOTS][8];
uint32_t ourBlockMs[CAR_BLOCK_SLOTS];
CarFrameSeen unlistedFrames[UNLISTED_MAX];
volatile uint8_t unlistedCount = 0;
volatile bool unlistedOverflow = false;
volatile uint8_t unlistedPassMode = UNL_FOLLOW_MODE;
volatile uint64_t unlistedPassSet = 0;
volatile bool unlistedPassOverflow = false;
uint32_t heldBackIds[HELD_BACK_MAX] = {0, 0, 0, 0};
static uint16_t lrSpeedA = 0, lrSpeedB = 0; // Land Rover: max wheel speed of each wheel-speed frame, 0.01 km/h [INF: DID 2B06 = raw / 100]
static uint8_t lastZielgang = 0;            // MQB Getriebe_11 GE_Zielgang last seen (paddle direction)

// ---- Small helpers -----------------------------------------------------------

// The VW generations whose chassis frames are PQ (Kombi_1 / Kombi_2 are read only
// on these - other platforms use 0x320 / 0x420 for other things).
static bool isPqChassisGeneration()
{
  return haldexGeneration == 1 || haldexGeneration == 2 || haldexGeneration == 4 || haldexGeneration == 51;
}

// The VW generations that read the MQB drive-mode frames (Charisma_01 / _07).
static bool isMqbDriveModeGeneration()
{
  return haldexGeneration == 50 || haldexGeneration == 51 || haldexGeneration == 52;
}

static void copyFrameSeen(CarFrameSeen &dst, const twai_message_t &f)
{
  dst.id = f.identifier;
  dst.extd = f.extd;
  dst.dlc = (f.data_length_code > 8) ? 8 : f.data_length_code;
  memcpy(dst.data, f.data, 8);
  uint32_t now = millis();
  if (now == 0)
  {
    now = 1; // 0 means "never seen"
  }
  dst.lastMs = now;
}

// The block bit for a CAN ID in the active generation, or -1. The Volvo Gen4
// heartbeat / configuration pages share 0x10A2407C, so their table rows carry
// ID 0 and are matched separately (see carFrameToHaldex).
static int blockBitForId(int genIdx, const twai_message_t &f)
{
  if (f.extd != isVolvoOrSaabGeneration(haldexGeneration))
  {
    return -1; // VAG blocks are 11-bit, Volvo blocks 29-bit
  }
  for (uint16_t i = 0; i < frameEditBlockCount; i++)
  {
    if (frameEditBlocks[i].genIdx == (uint8_t)genIdx && frameEditBlocks[i].canId != 0 &&
        frameEditBlocks[i].canId == f.identifier)
    {
      return frameEditBlocks[i].bit;
    }
  }
  return -1;
}

static bool isDiagnosticFrame(const twai_message_t &f)
{
  if (f.extd)
  {
    return false;
  }
  switch (f.identifier)
  {
  case diagnostics_1_ID:
  case diagnostics_2_ID:
  case diagnostics_3_ID:
  case diagnostics_4_ID:
  case ISO_ALLRAD_REQ:
    return true;
  default:
    return false;
  }
}

static bool isHeldBack(const twai_message_t &f)
{
  const uint32_t key = f.identifier | (f.extd ? HELD_BACK_EXTD : 0);
  for (uint8_t i = 0; i < HELD_BACK_MAX; i++)
  {
    if (heldBackIds[i] != 0 && heldBackIds[i] == key)
    {
      return true;
    }
  }
  return false;
}

// Slot of an unlisted car ID (added the first time it's seen), or -1 when the
// table is full (the ID then belongs to the "overflow" group).
static int noteUnlisted(const twai_message_t &f)
{
  const uint8_t n = unlistedCount;
  for (uint8_t i = 0; i < n; i++)
  {
    if (unlistedFrames[i].id == f.identifier && unlistedFrames[i].extd == (bool)f.extd)
    {
      copyFrameSeen(unlistedFrames[i], f);
      return i;
    }
  }
  if (n >= UNLISTED_MAX)
  {
    unlistedOverflow = true;
    return -1;
  }
  copyFrameSeen(unlistedFrames[n], f);
  unlistedCount = n + 1; // only the chassis task writes the table
  return n;
}

// Helper for analyzer pass-through: forward a frame exactly as received.
static void transmitFrameCopy(twai_handle_t bus, const twai_message_t &src, twai_message_t &scratch)
{
  scratch = src;
  scratch.extd = src.extd;
  scratch.rtr = src.rtr;
  scratch.data_length_code = src.data_length_code;
  canTransmit(bus, &scratch);
}

// Per-corner slip from wheelSpeedRaw [FL, FR, RL, RR] (ESP_19 units). Steering is
// fed only while fresh, else 0 (straight), so it falls back to plain relative slip
// rather than mis-compensating on a stale angle. V11.00.12: shared by ESP_19 (MQB)
// and Bremse_3 (PQ) - slip used to be MQB only.
static void updateCornerSlip()
{
  int16_t steerTenths = 0;
  if (receivedSteeringMs != 0 && (millis() - receivedSteeringMs) < 500)
  {
    int16_t mag = (int16_t)(receivedSteeringAngle * 10.0f + 0.5f);
    steerTenths = receivedSteeringNegative ? (int16_t)-mag : mag;
  }
  if (!computeCornerSlip(wheelSpeedRaw, steerTenths, slipSteeringRatio,
                           slipWheelbaseMm, slipTrackFrontMm, slipTrackRearMm,
                           slipMinSpeedRaw, cornerSlip))
  {
    cornerSlip[0] = cornerSlip[1] = cornerSlip[2] = cornerSlip[3] = -128;
  }
  lastCornerSlipMs = millis();
}

// Land Rover helpers: copy a whole car frame, read a 15-bit wheel speed, larger of two
static void lrCopy(uint8_t *dst, bool &valid, const twai_message_t &frame)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    dst[i] = (i < frame.data_length_code) ? frame.data[i] : 0x00;
  }
  valid = true;
}
static uint16_t lr15(const uint8_t *d, uint8_t hi) { return (uint16_t)(((d[hi] & 0x7F) << 8) | d[hi + 1]); }
static uint16_t lrMax(uint16_t a, uint16_t b) { return a > b ? a : b; }

// ---- Setup -------------------------------------------------------------------

void setupCHS()
{
  canInstallBus(0, CAN0_TX, CAN0_RX, &twaiBus0); // controller 0 = chassis
}

// ---- Chassis decode, one function per car (called from chsDecodeChassis) ------

// VW PQ + MQB (generations 1, 2, 4, 50, 51, 52) - both frame sets are read,
// a car only sends one of them.
static void chsDecodeVag(const twai_message_t &m)
{
  switch (m.identifier)
  {
  case MOTOR1_ID:
    // PQ Motor_1 (0x280) - engine ECU broadcast frame.
    // vw_pq.dbc:
    //   SG_ Motordrehzahl                      : 16|16@1+ (0.25,0)  "U/min"  -> bytes 2..3 LE, engine RPM
    //   SG_ Fahrpedalwert_oder_Drosselklapp    : 40|8@1+  (0.4,0)   "%"     -> byte 5,    accelerator pedal / throttle position
    //   SG_ inneres_Motor_Moment(_ohne_extern) : 8/32|8@1+(0.39,0)  "MDI"   -> bytes 1,4, indicated engine torque
    //   SG_ Fahrerwunschmoment                 : 56|8@1+  (0.39,0)  "MDI"   -> byte 7,    driver-requested torque
    //   SG_ Kickdownschalter                   : 2|1                          -> kickdown switch
    // Capture pedal % (data[5] * 0.4) and engine RPM (LE 16-bit * 0.25).
    receivedPedalValue = m.data[5] * 0.4;
    receivedVehicleRpm = ((m.data[3] << 8) | m.data[2]) * 0.25;
    break;

  case MOTOR2_ID:
    // PQ Motor_2 (0x288) - secondary engine ECU broadcast.
    // Used here only as a *fallback* coarse vehicle-speed source if the ABS
    // (Bremse_3 / Bremse_1) is silent. The legacy ESP32 codebase used
    // data[3] as a vehicle-speed proxy with a fudge factor (* 128/100) - this
    // does not exactly match any single DBC signal but produces "close enough"
    // km/h until ABS comes online.
    if ((!isABSValid || (millis() - lastABSResponse) > absTimeout)) // if we haven't received a valid ABS response within the timeout, use this speed value instead (which is less accurate but better than nothing)
    {
      receivedVehicleSpeed = m.data[3] * 128 / 100;
    }
    // PQ brake-light switch broadcast - vw_pq.dbc:
    //   SG_ MO2_BLS : 16|1@1+   -> byte 2 bit 0 ("Brake light switch - unfiltered raw signal")
    // Captured for diag display and as the source for the optional
    // followBrake / invertBrake CAN override applied before forwarding.
    brakeFromCAN = bitRead(m.data[2], 0);
    break;

  case ESP_05:
    // MQB ESP_05 (0x106) - brake-pressure / brake-pedal broadcast (vw_mqb.dbc):
    //   SG_ ESP_Fahrer_bremst : 26|1@1+   -> byte 3 bit 2, "driver is braking"
    // Captured for diag and used as the source for the optional CAN override.
    if (m.data_length_code >= 4)
    {
      brakeFromCAN = bitRead(m.data[3], 2);
    }
    break;

  case KOMBI_01:
    // MQB Kombi_01 (0x30B) - instrument cluster broadcast (vw_mqb.dbc):
    //   SG_ KBI_Handbremse : 23|1@1+     -> byte 2 bit 7, parking brake engaged
    // MQB-only ID; PQ platforms carry the handbrake on Kombi_1 (0x320) below.
    if (m.data_length_code >= 3)
    {
      handbrakeFromCAN = bitRead(m.data[2], 7);
    }
    break;

  case CHARISMA_07:
    // MQB Charisma_07 (0x3E8) - drive mode select, sent by the gateway once a second.
    // MQB FCAN K-matrix:
    //   SG_ CHA_Current_Mode : 8|4@1+  -> byte 1 bits 0-3, the drive profile the driver picked
    // Profile numbers (jilleb/MQB-FPA): 1 Comfort, 2 Normal, 3 Sport, 4 Offroad, 5 Eco,
    // 6 Race, 7 Individual, 8 Range/PHEV 1, 9 Lift/PHEV 2, 10 Snow/PHEV 3,
    // 11 Offroad Individual/PHEV 4. 0 = not filled in by this car.
    // Our own MQB logs read 00 B7 00 .. = 7 (Individual) while Charisma_01 asked
    // for a mix of Comfort / Normal / Sport across the systems, which is what an
    // Individual profile looks like.
    if (isMqbDriveModeGeneration() && m.data_length_code >= 2)
    {
      uint8_t currentMode = m.data[1] & 0x0F; // byte 1 bits 0-3: drive profile
      if (currentMode != 0)                                // 0 = not used on this car, so fall back to Charisma_01
      {
        receivedDriveProfile = currentMode;
        driveProfileSource = DRIVE_SOURCE_CHARISMA_07;
        lastDriveProfileMs = millis();
        lastCharisma07Ms = millis();
      }
    }
    break;

  case CHARISMA_01:
    // MQB Charisma_01 (0x385) - drive mode select, sent by the gateway once a second.
    // MQB FCAN K-matrix: a 4-bit "requested mode" for each system the gateway controls:
    //   SG_ CHA_Ziel_FahrPr_MO  : 16|4@1+ -> byte 2 bits 0-3, engine
    //   SG_ CHA_Ziel_FahrPr_ALR : 24|4@1+ -> byte 3 bits 0-3, Haldex (all-wheel drive)
    //   SG_ CHA_Ziel_FahrPr_VAQ : 36|4@1+ -> byte 4 bits 4-7, front diff lock (VAQ)
    // Mode numbers (jilleb/MQB-FPA): 1 Comfort, 2 Normal, 3 Sport, 4 Offroad, 5 Eco,
    // 6 Race, 7 Individual/Eco+, 8 On, 9 Off, 10 Snow. 0 = no function.
    if (isMqbDriveModeGeneration() && m.data_length_code >= 5)
    {
      receivedHaldexModeRequest = m.data[3] & 0x0F;     // byte 3 bits 0-3: mode asked of the Haldex
      receivedVaqModeRequest = (m.data[4] >> 4) & 0x0F; // byte 4 bits 4-7: mode asked of the VAQ
      hasHaldexModeRequest = true;

      // The engine's requested mode follows the drive profile on most cars, so use it
      // when Charisma_07 (the profile itself) isn't on this bus. It can't tell Individual
      // apart (it shows the engine setting chosen inside Individual). 8 (On) and 9 (Off)
      // aren't profiles, so they're skipped.
      uint8_t engineModeRequest = m.data[2] & 0x0F; // byte 2 bits 0-3: mode asked of the engine
      bool hasRecentCharisma07 = false;
      if (lastCharisma07Ms != 0 && (millis() - lastCharisma07Ms) < DRIVE_MODE_TIMEOUT_MS)
      {
        hasRecentCharisma07 = true;
      }
      if (!hasRecentCharisma07 && engineModeRequest != 8 && engineModeRequest != 9)
      {
        receivedDriveProfile = engineModeRequest;
        driveProfileSource = DRIVE_SOURCE_CHARISMA_01;
        lastDriveProfileMs = millis();
      }
    }
    break;

  case mKombi_2:
    // PQ Kombi_2 (0x420) - instrument cluster broadcast (vw_pq.dbc):
    //   SG_ Oeltemperatur_4_1                 : 24|8@1+ (1,-60)    -> byte 3, oil temp = raw - 60 degC
    //   SG_ Kuehlmitteltemp__4_1__Kombi_2_    : 32|8@1+ (0.75,-48) -> byte 4, coolant = raw x 0.75 - 48 degC
    // Shown on the dashboard's Live Data card (V11.00.4). Gated to the PQ
    // generations like Kombi_1 below: other platforms use 0x420 for other things.
    if (isPqChassisGeneration() && m.data_length_code >= 5)
    {
      receivedOilTemp = m.data[3] - 60.0f;             // byte 3: oil temp
      receivedCoolantTemp = m.data[4] * 0.75f - 48.0f; // byte 4: coolant temp
      lastKombi2Ms = millis();                                      // remember when, so old values go to "--"
    }
    break;

  case mKombi_1:
    // PQ Kombi_1 (0x320) - instrument cluster broadcast.
    // PQ35/46 K-matrix (V5.20.6F), message mKombi_1 0x320:
    //   KO1_Bremsinfo  : Byte 2 / Bit 0 (1-based) -> data[1] bit 0, brake-fluid warning
    //   KO1_Handbremse : Byte 2 / Bit 1 (1-based) -> data[1] bit 1, 1 = angezogen (applied)
    // (vw_pq.dbc lumps both as "Bremsinfo : 8|2".) Gen2 (1K0) and Gen4 (0AY)
    // read the handbrake from this frame, not from a physical wire. Gated to
    // the PQ-frame generations so a foreign 0x320 on another platform can't
    // spoof it. Gen1 (02D) added in V11: its OEM code reads this same bit
    // (handbrake pulled at 5-100 km/h -> clutch opens, spec_gen1_02d §2.2).
    if (isPqChassisGeneration() && m.data_length_code >= 2)
    {
      handbrakeFromCAN = bitRead(m.data[1], 1);
    }
    break;

  case MOTOR_04:
  {
    // MQB Motor_04 (0x107) - engine ECU torque/charge broadcast.
    // vw_mqb.dbc / MQB_FCAN K-matrix:
    //   SG_ MO_Ladedruck : 39|9@1+ (0.01,0) [0|5.1] "Unit_Bar"
    // i.e. absolute boost pressure (Ladedruck = boost pressure) in bar.
    // 9-bit little-endian: LSB at bit 39 = data[4] bit 7; bits 1..8 = data[5] bits 0..7.
    // Convert absolute -> gauge mbar: (raw * 10) - 1000 ; clamp to >=0.
    const uint16_t moLadedruckRaw = (uint16_t)(((uint16_t)m.data[5] << 1) | (m.data[4] >> 7)) & 0x01FF;
    const int32_t boostMbar = (int32_t)(moLadedruckRaw * 10.0f + 0.5f) - 1000;
    receivedVehicleBoost = (boostMbar > 0) ? (uint16_t)boostMbar : 0;
    break;
  }

  case MOTOR_12:
  {
    // MQB Motor_12 (0x0A8) - high-rate engine speed/torque broadcast.
    // vw_mqb.dbc:
    //   SG_ MO_Drehzahl_01 : 48|16@1+ (0.25,0) [0|16383] "Unit_MinutInver"
    // i.e. engine speed (Drehzahl = revs) in RPM, factor 0.25/bit, 16-bit LE.
    // Sent as a receiver to SAK_MQB (Haldex). bytes 6+7 little-endian.
    const uint16_t moDrehzahlRaw = ((uint16_t)m.data[7] << 8) | m.data[6];
    receivedVehicleRpm = (uint16_t)(moDrehzahlRaw * 0.25f + 0.5f);
    break;
  }

  case MOTOR7_ID:
  {
    // PQ Motor_7 (0x588) - engine boost/charge broadcast.
    //   SG_ Ladedruck (boost pressure) : 32|8@1+ (0.01,0) bar (absolute).
    // 8-bit absolute: gauge mbar = (raw * 10) - 1000 ; clamp to >=0.
    const uint8_t pqLadedruckRaw = m.data[4];
    const int32_t boostMbar = (int32_t)(pqLadedruckRaw * 10.0f + 0.5f) - 1000;
    receivedVehicleBoost = (boostMbar > 0) ? (uint16_t)boostMbar : 0;
    break;
  }

  case KLEMMEN_STATUS_01:
    // MQB Klemmen_Status_01 (0x3C0), MQB_ACAN_KMatrix:
    //   SG_ ZAS_Kl_15 : 17|1@1+ -> byte 2 bit 1, ignition (terminal 15) on
    receivedKl15 = (m.data[2] >> 1) & 0x01;
    lastKl15Ms = millis();
    break;

  case MOTOR_20:
    // MQB Motor_20 (0x121) - accelerator/raw pedal broadcast.
    //   SG_ MO_Fahrpedalrohwert_01 : 12|8@1+ (0.4,0) "%"
    // (Fahrpedalrohwert = raw accelerator pedal value)
    // 8-bit value straddles bytes 1..2: LSB at bit 12 = data[1] bit 4; bits 1..7 = data[2] bits 0..6.
    receivedPedalValue = ((m.data[1] >> 4) | ((m.data[2] & 0x0F) << 4)) * 0.4f;
    break;

  case MOTOR_14:
    // MQB Motor_14 (0x3BE) - low-rate engine state/event broadcast.
    //   SG_ MO_Kickdown : byte 5 bit 0 (in our captured frames)
    // Kickdown = full-throttle floored switch latch.
    receivedKickdown = bitRead(m.data[5], 0);
    break;

  case mGate_Komf_1:
  {
    // PQ Gate_Komf_1 (0x390) - gateway body / comfort signals.
    // vw_golf_mk4.dbc:
    // SG_ GK1_Warnblk_Status : 55|1@1+ -> byte 6 bit 7, hazard warning lights active
    // Used to drive hazard-light force-mode when enabled.
    // MK4 Golf doesn't actually have this on CAN despite DBC saying it does(!)
    hazardForceModeFlag = bitRead(m.data[6], 7);
    break;
  }

  case GATEWAY_72:
  {
    // MQB Gateway_72 (0x3DB) - gateway body lighting state.
    // Note: on at least some MQB clusters the gateway frame does NOT
    // reflect hazard state when active, so decode hazard from
    // Blinkmodi_02 (0x366) below instead. This case is kept
    // in case other body signals are needed here later.
    break;
  }

  case BLINKMODI_02:
  {
    // MQB Blinkmodi_02 (0x366) - turn-signal / hazard switch state.
    // vw_mqb.dbc:
    //   SG_ Hazard_Switch : 20|1@1+   -> byte 2 bit 4
    //   CM_ "Four-way flashers active"
    hazardForceModeFlag = bitRead(m.data[2], 4);
    break;
  }

  case BRAKES1_ID:
  {
    // PQ Bremse_1 (0x1A0) - ABS/ESP main broadcast (Bremse = brake).
    // vw_pq.dbc:
    //   SG_ BR1_ASR_passiv      : 60|1   -> byte 7 bit 4, ASR disabled by driver
    //   SG_ BR1_ESPASR_passiv   : 61|1   -> byte 7 bit 5, ESP+ASR disabled by driver
    //   SG_ BR1_Rad_kmh         : 17|15@1+ (0.01,0) "km/h"  -> wheel speed (>>1 of bytes 2..3 LE)
    //   SG_ BR1_Lampe_ABS / BR1_Lampe_ASR / BR1_Lampe_BK    -> warning lamps (bits 8..10)
    asrForceModeFlag = bitRead(m.data[7], 4); // PQ - Byte 7 (BR1_ASR_passiv)    - ASR disabled by driver
    tcForceModeFlag = bitRead(m.data[7], 5);  // PQ - Byte 7 (BR1_ESPASR_passiv) - ESP disabled by driver

    // BR1_ABS_Brems (vw_pq.dbc 2|1) - byte 0 bit 2, the ABS module is braking.
    // The Haldex reads this bit itself and opens the clutch (OEM code: Gen4
    // 0BR "state 4"). Our lock request outranks that, so lockGateBlocking()
    // releases lock while it is set (V11.00.0).
    absBrakingFromCAN = bitRead(m.data[0], 2);

    // BR1_Rad_kmh: 15-bit @ start bit 17, factor 0.01 km/h.
    // start bit 17 in LE = data[2] bit 1; therefore raw = ((data[3]<<8)|data[2]) >> 1.
    const uint16_t br1SpeedRaw = (((uint16_t)m.data[3] << 8) | m.data[2]) >> 1;
    receivedVehicleSpeed = (uint16_t)(br1SpeedRaw * 0.01f + 0.5f);
    break;
  }

  case BRAKES3_ID:
  {
    // PQ Bremse_3 (0x4A0) - individual wheel speeds broadcast.
    // vw_pq.dbc:
    //   SG_ Radgeschw__VL_4_1 : 1|15@1+ (0.01,0) "km/h"  -> front-left wheel speed (Radgeschwindigkeit = wheel speed, VL = vorne links / front-left)
    //   plus VR / HL / HR equivalents at start bits 17/33/49.
    // We only read the front-left here as a representative speed then
    // mark ABS "valid" so that the Motor_2 fallback is suppressed.
    // start bit 1 in LE = data[0] bit 1; raw = ((data[1]<<8)|data[0]) >> 1.
    const uint16_t br3SpeedRaw = (((uint16_t)m.data[1] << 8) | m.data[0]) >> 1;
    receivedVehicleSpeed = (uint16_t)(br3SpeedRaw * 0.01f + 0.5f);

    isABSValid = true;
    lastABSResponse = millis();

    // V11.00.12: all four corners -> per-corner slip on the PQ cars (Gen1 / 2 / 4 ...),
    // as ESP_19 does on MQB. VL / VR / HL / HR at start bits 1 / 17 / 33 / 49, 15 bits,
    // 0.01 km/h; converted to ESP_19 units (0.0075 km/h) so the slip maths is shared.
    for (uint8_t corner = 0; corner < 4; corner++)
    {
      const uint8_t b = corner * 2;
      const uint32_t raw = ((((uint32_t)m.data[b + 1] << 8) | m.data[b]) >> 1);
      wheelSpeedRaw[corner] = (uint16_t)((raw * 4u + 1u) / 3u); // 0.01 / 0.0075 = 4/3
    }
    lastWheelSpeedResponse = millis();
    updateCornerSlip();
    break;
  }

  case ESP_19:
  {
    // MQB ESP_19 (0x0B2) - per-wheel ABS speeds broadcast.
    // MQB FCAN K-matrix: four 16-bit LE wheel speeds (HL/HR/VL/VR) with
    // factor 0.0075 km/h per bit each.
    //   HL = Hinten Links  (rear left)   -> bytes 0..1
    //   HR = Hinten Rechts (rear right)  -> bytes 2..3
    //   VL = Vorne  Links  (front left)  -> bytes 4..5
    //   VR = Vorne  Rechts (front right) -> bytes 6..7
    // Average all four for a stable vehicle speed. ABS is then marked valid
    // so the Motor_2 fallback path stays suppressed.
    const uint16_t wheelSpeedHlRaw = (m.data[1] << 8) | m.data[0];
    const uint16_t wheelSpeedHrRaw = (m.data[3] << 8) | m.data[2];
    const uint16_t wheelSpeedVlRaw = (m.data[5] << 8) | m.data[4];
    const uint16_t wheelSpeedVrRaw = (m.data[7] << 8) | m.data[6];
    const float averageWheelSpeed =
        (wheelSpeedHlRaw + wheelSpeedHrRaw + wheelSpeedVlRaw + wheelSpeedVrRaw) * (0.0075f / 4.0f);

    receivedVehicleSpeed = (uint16_t)(averageWheelSpeed + 0.5f);
    isABSValid = true;
    lastABSResponse = millis();

    // Promote all four corners for the per-corner slip DID and dashboard.
    // Order [FL, FR, RL, RR] = [VL, VR, HL, HR].
    wheelSpeedRaw[0] = wheelSpeedVlRaw;
    wheelSpeedRaw[1] = wheelSpeedVrRaw;
    wheelSpeedRaw[2] = wheelSpeedHlRaw;
    wheelSpeedRaw[3] = wheelSpeedHrRaw;
    lastWheelSpeedResponse = millis();
    updateCornerSlip();
    break;
  }

  case mLW_1:
  {
    // PQ LW_1 (0x0C2) - steering-angle broadcast (vw_pq.dbc):
    //   SG_ LW1_LRW : 0|15@1+ (0.04375,0) "deg" -> data[0] + data[1] bits 0..6
    //   SG_ LW1_LRW_Sign : 15|1 -> data[1] bit 7 (ignored; we track magnitude only)
    const uint16_t lrwRaw = (((uint16_t)(m.data[1] & 0x7F) << 8) | m.data[0]);
    receivedSteeringAngle = lrwRaw * 0.04375f;
    receivedSteeringNegative = bitRead(m.data[1], 7); // LW1_LRW_Sign
    receivedSteeringMs = millis();
    break;
  }

  case LWI_01:
  {
    // MQB LWI_01 (0x086) - steering-angle sensor (vw_mqb.dbc):
    //   SG_ LWI_Lenkradwinkel : 16|13@1+ (0.1,0) "deg" -> data[2] + data[3] bits 0..4
    //   SG_ LWI_VZ_Lenkradwinkel : 29|1 -> data[3] bit 5 (ignored; magnitude only)
    const uint16_t lwiRaw = (((uint16_t)(m.data[3] & 0x1F) << 8) | m.data[2]);
    receivedSteeringAngle = lwiRaw * 0.1f;
    receivedSteeringNegative = bitRead(m.data[3], 5); // LWI_VZ_Lenkradwinkel
    receivedSteeringMs = millis();
    break;
  }

  case ESP_21:
  {
    // MQB ESP_21 (0x0FD) - ESP/ASR mode + reference speed.
    // MQB FCAN K-matrix:
    //   SG_ ASR_Tastung_passiv (Tastung = keying/button)  -> byte 6 bit 0, ASR disabled by driver
    //   SG_ ESP_Tastung_passiv                            -> byte 6 bit 1, ESP disabled by driver
    //   SG_ ESP_v_Signal : 16@1+ (0.01,0) "km/h"          -> bytes 4..5 LE, vehicle reference speed
    asrForceModeFlag = bitRead(m.data[6], 0); // MQB - Byte 6 Bit 0 (ASR_Tastung_passiv) - ASR passive
    tcForceModeFlag = bitRead(m.data[6], 1);  // MQB - Byte 6 Bit 1 (ESP_Tastung_passiv) - ESP disabled by driver

    // ABS_Bremsung (MQB FCAN K-matrix 56|1) - byte 7 bit 0, the ABS module is
    // braking. The K-matrix lists Allrad_Haldex as a receiver of this signal.
    // Released by lockGateBlocking() while set (V11.00.0).
    if (m.data_length_code >= 8)
    {
      absBrakingFromCAN = bitRead(m.data[7], 0);
    }

    receivedVehicleSpeed = (uint16_t)((((m.data[5] << 8) | m.data[4]) * 0.01f) + 0.5f);
    isABSValid = true;
    lastABSResponse = millis();
    break;
  }

  case GETRIEBE_17:
  {
    // MQB Getriebe_17 (0x0B1) - DSG paddle/tip status (Getriebe = transmission).
    // MQB FCAN K-matrix:
    //   SG_ GE_TippZustand : 16|2@1+   -> byte 2 bits 0..1, paddle tip state
    //     0 = Init_No_Tip      (idle / no paddle)
    //     1 = Tip_permanent    (sustained hold or both paddles together)
    //     2 = Tip_temporaer    (single momentary press)
    // We don't get an explicit up/down on this bus, only the activity flag;
    // direction is inferred from Getriebe_11.GE_Zielgang transitions below.
    const uint8_t tippZustand = m.data[2] & 0x03;
    paddleTipActive = (tippZustand > 0);
    paddleTipBoth = (tippZustand == 1); // Tip_permanent: sustained hold or both paddles
    break;
  }

  case GETRIEBE_11:
  {
    // MQB Getriebe_11 (0x0AD) - DSG status broadcast.
    // MQB FCAN K-matrix:
    //   SG_ GE_Zielgang : 60|4@1+   -> byte 7 bits 4..7, target gear of in-progress shift
    // Direction is inferred: zielgang increase while tip active = upshift
    // (right paddle); zielgang decrease while tip active = downshift (left).
    // The dedicated GRA_Tip_Hoch/Runter (Hoch=up, Runter=down) at 0x12B and
    // MFL_Tip_Up/Down at 0x3DD are absent from this bus, hence the inference.
    const uint8_t zielgang = (m.data[7] >> 4) & 0x0F;
    if (paddleTipActive)
    {
      paddleTipUp = (zielgang > lastZielgang);
      paddleTipDown = (zielgang < lastZielgang);
    }
    else
    {
      paddleTipUp = false;
      paddleTipDown = false;
    }
    lastZielgang = zielgang;
    break;
  }

  case OPENHALDEX_EXTERNAL_CONTROL_ID:
    // Only use external mode-change frames when broadcasting is
    // enabled - otherwise a stray frame on the chassis bus matching
    // OPENHALDEX_EXTERNAL_CONTROL_ID (e.g. data[0]==1 -> FWD) can flip
    // the mode unexpectedly.
    if (broadcastOpenHaldexOverCAN)
    {
      requestMode(m.data[0]); // the shared rules (IO.cpp) - range checked there
    }
    break;
  }
}

// Vauxhall / Opel Insignia (GM Global A, 500 kbit/s, 11-bit)
static void chsDecodeInsignia(const twai_message_t &frame)
{
  if (frame.data_length_code < 5)
  {
    return; // every frame used here has at least 5 bytes
  }

  if (frame.identifier == INSIGNIA_ENGINE_ID)
  {
    uint16_t rpmRaw = (frame.data[1] << 8) | frame.data[2]; // bytes 1-2, 0.25 rpm per bit
    receivedVehicleRpm = rpmRaw / 4;
    receivedPedalValue = frame.data[4] * 100.0f / 255.0f; // byte 4, 0-255 = 0-100 %
  }
  else if (frame.identifier == INSIGNIA_BRAKE_ID)
  {
    brakeFromCAN = bitRead(frame.data[0], 1); // byte 0 bit 1 = brake pressed
  }
  else if (frame.identifier == INSIGNIA_HANDBRAKE_ID)
  {
    handbrakeFromCAN = bitRead(frame.data[4], 4); // byte 4 bit 4 = parking brake applied
  }
  else if (frame.identifier == INSIGNIA_REAR_WHEELS_ID)
  {
    // Rear wheels are the undriven pair on this FWD-based car, so they give the
    // true road speed even while the fronts slip.
    uint16_t rearLeft = (frame.data[0] << 8) | frame.data[1];  // bytes 0-1
    uint16_t rearRight = (frame.data[2] << 8) | frame.data[3]; // bytes 2-3
    uint32_t rearAverage = ((uint32_t)rearLeft + rearRight) / 2;
    receivedVehicleSpeed = rearAverage / 32; // 1/32 km/h per bit
  }
}

// Ford Focus RS (500 kbit/s, 11-bit)
static void chsDecodeFocusRS(const twai_message_t &frame)
{
  if (frame.identifier == FOCUS_ENGINE_ID && frame.data_length_code >= 7)
  {
    receivedVehicleRpm = (frame.data[0] << 8) | frame.data[1];     // bytes 0-1, 1 rpm per bit (845 at idle) [KM - (CGEA EngAout_N_Actl 7|13 x2 rpm - differs)] [REA Kuga #18/#19: 15-bit b0[6:0]:b1, b0 bit 7 no reader]
    uint16_t speedRaw = (frame.data[4] << 8) | frame.data[5];        // bytes 4-5, 0.01 km/h per bit [KM - (CGEA Veh_V_ActlEng is b2-3)] [REA Kuga: not read]
    receivedVehicleSpeed = speedRaw / 100;
    receivedPedalValue = frame.data[6] * 0.5f;                     // byte 6, 0.5 % per bit (0-199) [KM -] [REA Kuga #41: F106A pedal]
  }
  else if (frame.identifier == FOCUS_BRAKE_ID && frame.data_length_code >= 3)
  {
    brakeFromCAN = bitRead(frame.data[2], 6); // byte 2 bit 6, 1 = pressed [KM -] [REA Kuga #20 / audit C: F1056 = mode 2 + preload off; "brake" INF from the RS log]
  }
  else if (frame.identifier == FOCUS_HANDBRAKE_ID && frame.data_length_code >= 4)
  {
    handbrakeFromCAN = bitRead(frame.data[3], 0); // byte 3 bit 0, 1 = applied [KM - (CGEA Cluster_Information_3_CG1 differs)] [REA Kuga #26: F103A -> reduced mode]
  }
}

// Ford Kuga Mk1 (V11.00.9). Same frames as the Focus RS for rpm / road speed / pedal (0x201),
// brake switch (0x205 b2 bit 6) and handbrake (0x433 b3 bit 0) - the Kuga AWD firmware reads
// exactly these two bits (trackF). Plus the ABS / ESP request the AWD module itself obeys:
// 0x211 b6 bits 1-2 = 1 (open the clutch) or 2 (ramp it down) while the ABS / ESP brakes;
// 3 = torque ceiling (normal driving with a cap, not braking).
static void chsDecodeFordKuga(const twai_message_t &frame)
{
  chsDecodeFocusRS(frame);
  if (frame.identifier == FORD_KUGA_ABS_ID && frame.data_length_code >= 7)
  {
    const uint8_t absField = (frame.data[6] >> 1) & 0x03; // b6[2:1] [KM -] [REA #21/#36 sig 0x62: 1 open, 2 ramp-down, 3 ceiling]
    absBrakingFromCAN = (absField == 1 || absField == 2);
    // b2[3:0]:b3 [KM -] [REA #22 sig 0x5D: ceiling C, demand <= 8 x C; same unit as our lock request]
    // V11.00.11: field 3 = the car's ESP torque ceiling b2[3:0]:b3 - our OEM lock request never goes above it
    kugaCarTorqueCeiling = (absField == 3) ? (uint16_t)(((frame.data[2] & 0x0F) << 8) | frame.data[3]) : 0xFFFF;
    // V11.00.13: keep the car's whole 0x211 - in OEM our lock request frame is built on it (StandaloneCAN FordKuga_frames10)
    for (uint8_t i = 0; i < 8; i++)
    {
      kugaCar211[i] = (i < frame.data_length_code) ? frame.data[i] : 0x00; // incl. b5 bit 0 (bench release, REA #24 unresolved) - kept for OEM
    }
    kugaCar211Valid = true;
  }
}

// Land Rover Gen4 FL2 / Evoque (V11.00.13, binary trace only - not checked on a car). The two cars use
// different layouts on the same IDs, so rpm / speed / pedal are decoded only once the unit's feedback says
// which one it is (lrFeedbackLayout: 0x310 = FL2, 0x301 = Evoque). The car's ABS / DSC request and the
// lock request frames are kept whatever the layout. Brake / handbrake: no bit the unit reads is proven to be the
// brake (0x12A b0.2 / 0x150 b0.0 -> mode 2 are INF), so Follow Brake / Handbrake have nothing to edit here.
static void chsDecodeLandRover(const twai_message_t &frame)
{
  if (frame.data_length_code < 8)
  {
    return; // every frame the unit reads is DLC 8 [REA audit_landrover #5]
  }
  const uint8_t *d = frame.data;
  const bool fl2 = lrFeedbackLayout == LR_LAYOUT_FL2;
  const bool evq = lrFeedbackLayout == LR_LAYOUT_EVQ;
  switch (frame.identifier)
  {
  case LR_REQUEST_20A_ID:
    lrCopy(lrCar20A, lrCar20AValid, frame); // OEM: our 0x20A is built on it (StandaloneCAN LandRover_frames20)
    if (fl2)
    {
      const uint8_t field = (d[6] >> 5) & 0x07; // FL2 b6 bits 5-7 [KM -] [REA audit_landrover #29: 1 open, 2 ramp, 3 full, 4 direct]
      absBrakingFromCAN = (field == 1 || field == 2);
    }
    break;
  case LR_REQUEST_2B5_ID:
    lrCopy(lrCar2B5, lrCar2B5Valid, frame);
    if (evq)
    {
      const uint8_t field = (d[1] >> 2) & 0x07; // EVQ b1 bits 2-4 [KM -] [REA audit E6: same field codes as FL2 0x20A b6]
      absBrakingFromCAN = (field == 1 || field == 2);
      lrSpeedA = lrMax(lr15(d, 3), lr15(d, 5)); // EVQ b3[6:0]:b4, b5[6:0]:b6 [KM -] [REA E5: DIDs 2B06-2B09]
    }
    else if (fl2)
    {
      lrSpeedA = lrMax(lr15(d, 4), lr15(d, 6)); // FL2 b4[6:0]:b5, b6[6:0]:b7 [KM -] [REA #19: x11377/8000 internally]
    }
    break;
  case LR_REQUEST_250_ID:
    lrCopy(lrCar250, lrCar250Valid, frame);
    if (fl2)
    {
      lrSpeedB = lrMax(lr15(d, 4), lr15(d, 6)); // FL2 b4[6:0]:b5, b6[6:0]:b7 [KM -] [REA #19]
    }
    break;
  case LR_150_ID:
    if (evq)
    {
      lrSpeedB = lrMax(lr15(d, 3), lr15(d, 6)); // EVQ b3[6:0]:b4, b6[6:0]:b7 [KM -] [REA E5]
    }
    break;
  case LR_FL2_ENGINE_ID:
    if (fl2)
    {
      receivedVehicleRpm = (uint16_t)(((d[6] & 0x1F) << 8) | d[7]); // b6[4:0]:b7, 1 rpm/bit [KM -] [REA #21: DID F40C = raw x 4 = OBD PID 0C]
    }
    break;
  case LR_EVQ_ENGINE_ID:
    if (evq)
    {
      receivedVehicleRpm = (uint16_t)(((d[4] & 0x1F) << 8) | d[5]); // b4[4:0]:b5 [KM -] [REA E5: DID F40C]
    }
    break;
  case LR_FL2_TORQUE_ID:
  case LR_EVQ_TORQUE_ID:
    if ((fl2 && frame.identifier == LR_FL2_TORQUE_ID) || (evq && frame.identifier == LR_EVQ_TORQUE_ID))
    {
      receivedPedalValue = (float)(((d[2] & 0x03) << 8) | d[3]) * 0.1f; // b2[1:0]:b3, 0.1 %/bit [KM -] [REA #25: DID F44A = raw x 255 / 1000]
    }
    break;
  case LR_EVQ_369_ID:
    if (evq && (((d[5] & 0x0F) == 6) || ((d[5] & 0x0F) == 10)))
    {
      absBrakingFromCAN = true; // b5[3:0] 6 / 10 = ramp-down request [KM -] [REA audit E6: beats field 3 / 4]
    }
    break;
  default:
    break;
  }
  if (fl2 || evq)
  {
    receivedVehicleSpeed = lrMax(lrSpeedA, lrSpeedB) / 100; // fastest wheel, km/h
  }
}

// Volvo P3 (500 kbit/s, 11-bit). Handbrake (0x2EE) is not decoded: the meaning
// of its state values isn't known yet.
static void chsDecodeVolvoGen4(const twai_message_t &frame)
{
  if (frame.data_length_code < 8)
  {
    return; // all Volvo P3 frames used here are 8 bytes
  }

  if (frame.extd)
  {
    // V11.00.5: the real P3 bus is 29-bit. Read from the car's frames as they
    // arrive (before any are held back for ours). Road speed comes from the rear
    // wheel speeds in 0x10004002 (scale from the module's own live data, below).
    // V11.00.8: this only started running now (it used to be skipped), so the
    // speed / throttle lock settings and Follow ABS act on a Gen4 Volvo from here.
    if (frame.identifier == VOLVO_G4_ENGINE_ID)
    {
      receivedVehicleRpm = (uint16_t)(((frame.data[2] & 0x1F) << 8) | frame.data[3]); // 1 rpm per bit [KM -][REA S60R s07 layout][LOG]
      brakeFromCAN = bitRead(frame.data[6], 3);                                         // byte 6 bit 3 = brake switch [KM -][REA S60R s03][LOG]
    }
    else if (frame.identifier == VOLVO_G4_TORQUE_ID)
    {
      const uint16_t pedalRaw = (uint16_t)(((frame.data[6] & 0x03) << 8) | frame.data[7]); // b6[1:0]:b7 [KM -][REA S60R s01 layout]
      receivedPedalValue = pedalRaw * 0.4f; // UNCONFIRMED scale: 0.4 %/bit assumed (log: 0 foot off, 223 on a hard pull) [no OEM code; REA S60R F6 hints 0.2 %/bit - C038 = (2 x raw + 5) / 10]
    }
    else if (frame.identifier == VOLVO_G4_WHEELS_ID)
    {
      // Wheel order + scale: the AWD module's own live data (D2 A6 00 06) echoes these
      // four values unchanged, and vtl/volvo-ddd reads that as FR / FL / RR / RL x 0.0156
      // km/h; 0x11600002 / 0x11800002 carry the same speeds in 0.01 km/h (S60 log).
      const uint16_t rr = (uint16_t)(((frame.data[0] & 0x3F) << 8) | frame.data[1]); // [KM -][REA S60R s21 14-bit layout][bench/WEB corner + scale]
      const uint16_t rl = (uint16_t)(((frame.data[2] & 0x3F) << 8) | frame.data[3]); // [KM -][REA S60R s20][bench/WEB]
      receivedVehicleSpeed = (uint16_t)(((uint32_t)rr + rl) / 128); // rear (undriven) wheels: (rr + rl) / 2 / 64 km/h
      // RL qualifier = 1 or 2: the AWD module opens its clutch for both (bench) - the
      // ABS / stability control is braking. 0 = normal, 3 = no reaction.
      const uint8_t absMode = (uint8_t)(frame.data[2] >> 6); // b2[7:6] [KM -][REA S60R s05 mode selector: 1/2 -> mode 4][bench: 1/2 open the clutch]
      absBrakingFromCAN = (absMode == 1 || absMode == 2);
    }
    else if (frame.identifier == VOLVO_P3_STEER29_ID)
    {
      // Display only (gen 43 is not in the steering lock-scale list until the scale
      // and centre are checked on a car)
      const int32_t raw = (int32_t)((frame.data[2] << 8) | frame.data[3]); // b2-3 [KM -][REA: not an AWD-module frame][LOG estimate]
      const int32_t fromCentre = raw - VOLVO_P3_STEER_CENTRE; // + = right in the S60 log
      receivedSteeringAngle = fabsf((float)fromCentre * VOLVO_P3_STEER_DEG_PER_COUNT);
      receivedSteeringNegative = (fromCentre < 0);
      receivedSteeringMs = millis();
    }
    return;
  }

  if (frame.identifier == VOLVO_P3_PEDAL_ID)
  {
    uint16_t pedalRaw = (frame.data[2] & 0x03) << 8; // byte 2 bits 1-0 = top 2 bits
    pedalRaw = pedalRaw | frame.data[3];             // byte 3 = bottom 8 bits
    receivedPedalValue = pedalRaw * 0.1f;          // 0.1 % per bit
  }
  else if (frame.identifier == VOLVO_P3_ENGINE_ID)
  {
    brakeFromCAN = bitRead(frame.data[4], 6); // byte 4 bit 6 = brake pressed
    uint16_t rpmRaw = (frame.data[6] & 0x1F) << 8; // byte 6 bits 4-0 = top 5 bits
    rpmRaw = rpmRaw | frame.data[7];               // byte 7 = bottom 8 bits
    receivedVehicleRpm = rpmRaw;                 // 1 rpm per bit
  }
  else if (frame.identifier == VOLVO_P3_SPEED_ID)
  {
    uint16_t speedRaw = (frame.data[6] << 8) | frame.data[7]; // bytes 6-7, 0.01 km/h per bit
    receivedVehicleSpeed = speedRaw / 100;
  }
}

// SAAB 9-3 P-bus (500 kbit/s, 11-bit). Brake and throttle are not decoded:
// their positions are only guesses so far.
static void chsDecodeSaab93(const twai_message_t &frame)
{
  if (frame.data_length_code < 8)
  {
    return; // all SAAB frames used here are 8 bytes
  }

  if (frame.identifier == SAAB_93_ENGINE_ID)
  {
    uint16_t rpmRaw = (frame.data[1] << 8) | frame.data[2]; // bytes 1-2, 4 counts per rpm
    receivedVehicleRpm = rpmRaw / 4;
  }
  else if (frame.identifier == SAAB_93_WHEELS_ID)
  {
    // Which wheel is which isn't known yet, so use the average of all four
    uint32_t total = 0;
    for (uint8_t i = 0; i < 8; i = i + 2)
    {
      total = total + ((frame.data[i] << 8) | frame.data[i + 1]); // one 16-bit wheel speed
    }
    uint32_t averageRaw = total / 4;
    receivedVehicleSpeed = averageRaw * 1609 / 50000; // raw / 50 = mph, x 1.609 = km/h
  }
  else if (frame.identifier == SAAB_93_HANDBRAKE_ID)
  {
    handbrakeFromCAN = bitRead(frame.data[5], 6); // byte 5 bit 6 = handbrake applied
  }
}

// Volvo P2 pre-facelift, 2002 model-year ID set (250 kbit/s, 29-bit). Other
// model years use different IDs, so these simply won't appear on them.
// Handbrake has not been found yet.
static void chsDecodeVolvoP2Gen2(const twai_message_t &frame)
{
  if (frame.data_length_code < 8)
  {
    return; // all P2 frames used here are 8 bytes
  }

  if (frame.identifier == VOLVO_P2_2002_SPEED_ID)
  {
    uint16_t speedRaw = (frame.data[6] << 8) | frame.data[7]; // bytes 6-7, 0.01 km/h per bit
    receivedVehicleSpeed = speedRaw / 100;
  }
  else if (frame.identifier == VOLVO_P2_2002_ENGINE_ID)
  {
    uint16_t rpmRaw = (frame.data[6] & 0x1F) << 8; // byte 6 bits 4-0 = top 5 bits
    rpmRaw = rpmRaw | frame.data[7];               // byte 7 = bottom 8 bits
    receivedVehicleRpm = rpmRaw;                 // 1 rpm per bit
  }
  else if (frame.identifier == VOLVO_P2_2002_BRAKE_ID)
  {
    brakeFromCAN = bitRead(frame.data[4], 5); // byte 4 bit 5 = brake switch
  }
  else if (frame.identifier == VOLVO_P2_2002_PEDAL_ID)
  {
    // byte 3 runs from 19 (foot off) to about 187 (floored): turn that into 0-100 %
    float pedalRaw = frame.data[3];
    float pedalPercent = (pedalRaw - VOLVO_P2_2002_PEDAL_REST) * 100.0f /
                         (VOLVO_P2_2002_PEDAL_FULL - VOLVO_P2_2002_PEDAL_REST);
    if (pedalPercent < 0.0f)
    {
      pedalPercent = 0.0f; // below the rest value (e.g. key off reads 0)
    }
    if (pedalPercent > 100.0f)
    {
      pedalPercent = 100.0f;
    }
    receivedPedalValue = pedalPercent;
  }
}

// Volvo P1 S40 / V50 (500 kbit/s, 29-bit). IDs are in canID.h (VOLVO_P1_*);
// byte positions from Adam's 2005 V50 T5 AWD logs. Every byte read here: [KM -][REA none][LOG].
static void chsDecodeVolvoP1(const twai_message_t &frame)
{
  if (frame.data_length_code < 8)
  {
    return; // every P1 frame is 8 bytes
  }

  if (frame.identifier == VOLVO_P1_SPEED_ID)
  {
    uint16_t speedRaw = (frame.data[6] << 8) | frame.data[7]; // bytes 6-7, 0.01 km/h per bit (assumed - check against the speedo)
    receivedVehicleSpeed = speedRaw / 100;
  }
  else if (frame.identifier == VOLVO_P1_ENGINE_RPM_ID)
  {
    uint16_t rpmRaw = (frame.data[2] & 0x1F) << 8; // byte 2 bits 4-0 = top 5 bits
    rpmRaw = rpmRaw | frame.data[3];               // byte 3 = bottom 8 bits
    receivedVehicleRpm = rpmRaw;                 // 1 rpm per bit (about 770 at idle)
  }
  else if (frame.identifier == VOLVO_P1_ENGINE_TORQUE_ID)
  {
    receivedPedalValue = frame.data[7] * 100.0f / 255.0f; // byte 7 = pedal, 0-255 (about 250 floored)
    brakeFromCAN = bitRead(frame.data[4], 4);               // byte 4 bit 4 = brake pressed
  }
  else if (frame.identifier == VOLVO_P1_HANDBRAKE_ID)
  {
    handbrakeFromCAN = bitRead(frame.data[2], 6); // byte 2 bit 6 = handbrake on
  }
  else if (frame.identifier == VOLVO_P1_BRAKE_ID)
  {
    // byte 6 bit 4 = DSTC switched off by the driver (1 for as long as it's off:
    // the dstc-off and wheelspin logs). Same job as the VW "ESP passive" bit, so
    // "Use TC to Force Mode" forces its chosen mode while DSTC is off.
    tcForceModeFlag = bitRead(frame.data[6], 4);
    asrForceModeFlag = tcForceModeFlag; // the P1 has one DSTC switch - show it on the ASR pill too
  }
}

// Read one chassis frame for the active generation. Each car only reads its own
// IDs, so another make reusing a VW ID number (SAAB 0x280 / 0x288 / 0x320) can't
// be misread.
static void chsDecodeChassis(const twai_message_t &m)
{
  switch (haldexGeneration)
  {
  case 1:
  case 2:
  case 4:
  case 50:
  case 51:
  case 52:
    chsDecodeVag(m);
    break;

  case 41:
    if (!m.extd)
    {
      chsDecodeInsignia(m); // 11-bit
    }
    break;

  case 42:
    if (!m.extd)
    {
      chsDecodeFocusRS(m); // 11-bit
    }
    break;

  case GEN_FORD_KUGA:
    if (!m.extd)
    {
      chsDecodeFordKuga(m); // 11-bit
    }
    break;

  case GEN_LANDROVER_GEN4:
    if (!m.extd)
    {
      chsDecodeLandRover(m); // 11-bit (V11.00.13)
    }
    break;

  case GEN_SAAB_93_XWD:
    if (!m.extd)
    {
      chsDecodeSaab93(m); // 11-bit
    }
    break;

  case GEN_VOLVO_GEN4:
    chsDecodeVolvoGen4(m); // 11-bit and 29-bit (the real P3 bus is 29-bit)
    break;

  case GEN_VOLVO_P2_GEN2:
    if (m.extd)
    {
      chsDecodeVolvoP2Gen2(m); // 29-bit
    }
    break;

  case GEN_VOLVO_P1:
    if (m.extd)
    {
      chsDecodeVolvoP1(m); // 29-bit
    }
    break;

  default: // GEN_VOLVO_P2_GEN3 (3), GEN_VOLVO_SPC_GEN5 (53): no chassis decode yet
    break;
  }
}

// ---- Frames to the Haldex --------------------------------------------------------

// Brake/handbrake bits on the chassis frames we pass to the Haldex in OEM mode.
// With Follow Brake / Follow Handbrake on, the bit sent to the Haldex is set when
// EITHER the car's own bit is set (passed through as it was) OR a switch wired
// to our brake / handbrake input pin is on (V11.00.0 - for cars where a physical
// brake or handbrake switch is fitted, so it reaches the Haldex over CAN too).
// brakeActive / handbrakeActive are the input pins with "Invert ... Input"
// already applied by IO.cpp; the car's CAN bit is never inverted.
// With Follow off the frame is forwarded untouched.
//
// Bit positions (verified against vw_pq.dbc / PQ35_46 K-matrix / vw_mqb.dbc):
//   PQ Gen1/2/4/0AY: Motor_2  (0x288) byte 2 bit 0  - MO2_BLS         (no CRC)
//   PQ Gen1/2/4/0AY: Kombi_1  (0x320) byte 1 bit 1  - KO1_Handbremse  (no CRC)
//   MQB Gen5:        ESP_05   (0x106) byte 3 bit 2  - ESP_Fahrer_bremst (CRC at byte 0 via ID_SEQ_106)
//   MQB Gen5:        Kombi_01 (0x30B) byte 2 bit 7  - KBI_Handbremse   (no CRC)
static void applyBrakeHandbrakeCANOverride(twai_message_t &m)
{
  // What the Haldex should see: the car's bit OR our input pin
  bool brakeOut = false;
  if (brakeFromCAN || brakeActive)
  {
    brakeOut = true;
  }
  bool handbrakeOut = false;
  if (handbrakeFromCAN || handbrakeActive)
  {
    handbrakeOut = true;
  }

  switch (haldexGeneration)
  {
  case 1:  // Gen1 02D (V11.00.0): same Motor_2 / Kombi_1 bits as the PQ cars
  case 2:  // PQ35 (1K0): brake + handbrake both broadcast on chassis CAN
  case 4:  // PQ35/46 (0BR): brake + handbrake both broadcast on chassis CAN
  case 51: // 0AY - a PQ unit, PQ frame set (see Gen5_0AY_frames*)
    // PQ Motor_2 (0x288) - MO2_BLS brake light switch: byte 2 bit 0. No CRC.
    // [KM Motor_2.MO2_BLS b2.0] [REA Gen1 audit_gen1_02d #15/#40/#41: b2.0 -> E1EC.6 brake vote -> brake path ~11 % cap;
    //  Gen2 audit_gen2_7h0 #32: b2.0/b2.1 brake vote]. b2.1 MO2_BTS (also a vote on both) is passed as the car sent it.
    if (followBrake && m.identifier == MOTOR2_ID && m.data_length_code >= 3)
    {
      if (brakeOut)
      {
        m.data[2] = m.data[2] | 0x01; // byte 2 bit 0 = brake pressed
      }
      else
      {
        m.data[2] = m.data[2] & ~0x01; // byte 2 bit 0 = brake released
      }
    }
    // PQ Kombi_1 (0x320) - KO1_Handbremse: byte 1 bit 1 (PQ35/46 K-matrix
    // "Byte 2 / Bit 1", 1 = angezogen / applied). Byte 1 bit 0 is the
    // brake-fluid warning (KO1_Bremsinfo) and is left untouched. No CRC.
    // [KM Kombi_1.Bremsinfo b1.0-1 (vw_pq.dbc) = KO1_Bremsinfo / KO1_Handbremse] [REA Gen1 audit_gen1_02d #15: b1.1 -> E1E2.7
    //  (rising edge at 5-100 km/h opens the clutch); Gen2 audit_gen2_7h0 #32: sig95 b1.1 handbrake vote]
    if (followHandbrake && m.identifier == mKombi_1 && m.data_length_code >= 2)
    {
      if (handbrakeOut)
      {
        m.data[1] = m.data[1] | 0x02; // byte 1 bit 1 = handbrake applied
      }
      else
      {
        m.data[1] = m.data[1] & ~0x02; // byte 1 bit 1 = handbrake off
      }
    }
    // (The old Gen51 MQB Kombi_01 0x30B rewrite is gone: the OEM code shows
    // the 0AY has no receive slot for 0x30B - spec_gen5_0ay #12.)
    break;

  case 50:
  case 52: // Gen5 0CQ VAQ - copies 0CQ brake/handbrake overrides
    // MQB ESP_05 (0x106) - ESP_Fahrer_bremst: byte 3 bit 2, CRC at byte 0
    // REA (spec_gen5_0cq §3): the 0CQ reads the brake PRESSURE (b2 + b3[0-1]),
    // not this bit, so this edit does not reach the 0CQ's own logic (VAQ: not traced).
    if (followBrake && m.identifier == ESP_05 && m.data_length_code >= 8)
    {
      if (brakeOut)
      {
        m.data[3] = m.data[3] | 0x04; // [KM ESP_05.ESP_Fahrer_bremst b3.2] [REA spec §3: not read by the 0CQ]
      }
      else
      {
        m.data[3] = m.data[3] & ~0x04; // [KM ESP_05.ESP_Fahrer_bremst b3.2 = 0] [REA spec §3: not read by the 0CQ]
      }
      // Recompute AUTOSAR CRC8 at byte 0 - counter nibble at byte 1 already
      // matches the original sender, so the same ID_SEQ_106 lookup applies.
      m.data[0] = calcChecksum(m.data, ID_SEQ_106); // [KM ESP_05.ESP_05_CRC] [REA audit_gen5_0cq S6: the 0CQ/VAQ check none - kept for the car's other modules]
    }
    // MQB Kombi_01 (0x30B) - KBI_Handbremse: byte 2 bit 7. No CRC byte
    // (byte 0 is fully populated with KBI lamp flags).
    // REA (spec §3): the 0CQ only uses this bit while ESP_20 b1[4-5] != 3 (car
    // without integrated EPB) and cal 0xC0B6CF = 1.
    if (followHandbrake && m.identifier == KOMBI_01 && m.data_length_code >= 3)
    {
      if (handbrakeOut)
      {
        m.data[2] = m.data[2] | 0x80; // [KM Kombi_01.KBI_Handbremse b2.7 = 1] [REA spec §3: -> 0xCFE2, gated as above]
      }
      else
      {
        m.data[2] = m.data[2] & ~0x80; // [KM Kombi_01.KBI_Handbremse b2.7 = 0] [REA spec §3: as above]
      }
    }
    break;

  case GEN_VOLVO_GEN4:
    // Volvo Gen4 P3 (V11.00.5): the two ECM brake switches the unit reads - 0x0080401E
    // byte 6 bit 3 (bench: cuts lock while moving) and 0x0062401E byte 4 bit 4. No CRC.
    // Handbrake: not found on this bus yet.
    if (followBrake && m.extd && m.data_length_code >= 8)
    {
      if (m.identifier == VOLVO_G4_ENGINE_ID)
      {
        if (brakeOut)
        {
          m.data[6] = m.data[6] | 0x08; // byte 6 bit 3 = brake pressed [KM -][REA S60R s03 brake vote (layout; no P3 dump)][bench]
        }
        else
        {
          m.data[6] = m.data[6] & ~0x08; // byte 6 bit 3 = brake released
        }
      }
      else if (m.identifier == VOLVO_G4_TORQUE_ID)
      {
        if (brakeOut)
        {
          m.data[4] = m.data[4] | 0x10; // byte 4 bit 4 = brake pressed [KM -][LOG][REA S60R: not in its signal table - no OEM code]
        }
        else
        {
          m.data[4] = m.data[4] & ~0x10; // byte 4 bit 4 = brake released
        }
      }
    }
    break;

  case GEN_FORD_KUGA:
    // Ford Kuga (V11.00.9): the brake switch the AWD module reads is 0x205 byte 2 bit 6 and the
    // handbrake 0x433 byte 3 bit 0 (its own firmware, trackF; same bits as the Focus RS logs). No CRC.
    if (followBrake && !m.extd && m.identifier == FORD_KUGA_TORQUE_ID && m.data_length_code >= 3)
    {
      if (brakeOut)
      {
        m.data[2] = m.data[2] | 0x40; // byte 2 bit 6 = brake pressed [KM -] [REA audit C/#20: F1056 = mode 2 + preload off - reduces, never opens; "brake" INF]
      }
      else
      {
        m.data[2] = m.data[2] & ~0x40; // byte 2 bit 6 = brake released [KM -] [REA: F1056 = 0]
      }
    }
    if (followHandbrake && !m.extd && m.identifier == FORD_KUGA_HANDBRAKE_ID && m.data_length_code >= 4)
    {
      if (handbrakeOut)
      {
        m.data[3] = m.data[3] | 0x01; // byte 3 bit 0 = handbrake applied [KM -] [REA #26/#35: F103A -> mode 5 latch (8V41) / mode 4 alive-gated (9V4N)]
      }
      else
      {
        m.data[3] = m.data[3] & ~0x01; // byte 3 bit 0 = handbrake off [KM -] [REA: F103A = 0]
      }
    }
    break;

  default:
    break;
  }
}

void resetFrameCapture()
{
  memset(carBlockFrame, 0, sizeof(carBlockFrame));
  memset(ourBlockData, 0, sizeof(ourBlockData));
  memset(ourBlockMs, 0, sizeof(ourBlockMs));
  memset(unlistedFrames, 0, sizeof(unlistedFrames));
  unlistedCount = 0;
  unlistedOverflow = false;
}

// standaloneTx(): remember what we sent for each block (Long Learn report)
void noteOurFrame(const twai_message_t &f)
{
  const int gi = frameEditGenIdx(haldexGeneration);
  if (gi < 0)
  {
    return;
  }
  const int bit = blockBitForId(gi, f);
  if (bit < 0 || bit >= CAR_BLOCK_SLOTS)
  {
    return;
  }
  memcpy(ourBlockData[bit], f.data, 8);
  ourBlockMs[bit] = millis();
}

// Should this car frame go to the Haldex? Only for frame-edit generations while
// our frames are active (standalone, or normal mode outside Stock).
bool carFrameToHaldex(const twai_message_t &f)
{
  if (isDiagnosticFrame(f))
  {
    return true; // tester traffic always goes through
  }
  const int gi = frameEditGenIdx(haldexGeneration);
  if (gi < 0)
  {
    return !isStandalone; // no blocks: normal mode passes, standalone sends its own
  }

  // Volvo Gen4: the CEM's 0x10A2407C pages are ours while either page block is on
  if (haldexGeneration == GEN_VOLVO_GEN4 && f.extd && f.identifier == VOLVO_G4_NETWORK_ID)
  {
    return !frameEditEnabled((uint8_t)gi, VOLVO_G4_BLOCK_NETWORK) &&
           !frameEditEnabled((uint8_t)gi, VOLVO_G4_BLOCK_CONFIG);
  }

  const int bit = blockBitForId(gi, f);
  if (bit >= 0)
  {
    if (bit < CAR_BLOCK_SLOTS)
    {
      copyFrameSeen(carBlockFrame[bit], f);
    }
    return !frameEditEnabled((uint8_t)gi, (uint8_t)bit); // ON = ours goes instead
  }

  // A car ID we have no block for
  const int slot = noteUnlisted(f);
  if (isHeldBack(f))
  {
    return false; // Long Learn found this one cuts the lock
  }
  switch (unlistedPassMode)
  {
  case UNL_HOLD_ALL:
    return false;
  case UNL_PASS_SET:
    if (slot < 0)
    {
      return unlistedPassOverflow;
    }
    return (unlistedPassSet >> slot) & 0x1ULL;
  default:
    return !isStandalone; // normal mode: the car's other frames go through; standalone: bench bus only
  }
}

// ---- Our own diagnostic IDs on the chassis bus ----------------------------------

// External diagnostic-tool detection (auto-pause of live polling).
// A scanner addresses the Haldex from the chassis side; these request IDs
// never originate from us (our polling transmits on Bus 1), so seeing one on
// Bus 0 means a real tool is connected. externalDiagActive() then backs our
// UDS/TP2.0 tasks off until it goes quiet - so liveDiagEnabled restores on disconnection.
static void chsNoteExternalTester(const twai_message_t &m)
{
  const uint32_t id = m.identifier;
  if (id == KWP_TP20_SETUP_TX_ID ||         // 0x200 TP2.0 channel setup
      id == 0x7DFu ||                       // OBD functional request (scan/clear)
      (id >= 0x700u && id <= 0x71Fu) ||   // VAG UDS/KWP physical tester requests
      (haldexGeneration == GEN_FORD_KUGA && id == FORD_KUGA_DIAG_REQ) || // V11.00.10: Ford tool -> AWD module (0x761)
      (haldexGeneration == GEN_LANDROVER_GEN4 && id == LR_DIAG_REQ))     // V11.00.13: LR tool -> Haldex (0x795)
  {
    externalDiagLastMs = millis();
  }
}

// Answers the OpenHaldex DIDs a tester asks for on 0x70F (lock, slip, mode write).
// true = it was one of ours: answered and consumed (never forwarded to Bus 1).
static bool chsAnswerOpenHaldexRequest(const twai_message_t &m)
{
  // OpenHaldex supplier-specific UDS responders. A tester on the OBD/chassis
  // bus (e.g. Rokketek gauge) polls the Haldex physical address 0x70F and
  // decodes replies on 0x779; the passive 0x6B0 broadcast does not cross the
  // gateway to the OBD port, so we answer supplier DIDs on the same pair and
  // CONSUME the request (return true) so the real Haldex on Bus 1 never sees it.
  if (m.identifier == ISO_ALLRAD_REQ &&
      m.data_length_code >= 4 &&
      (m.data[0] & 0xF0) == 0x00 && // ISO-TP single frame
      (m.data[0] & 0x0F) >= 3 &&    // >= 3 payload bytes (SID + DID)
      m.data[1] == 0x22 &&          // ReadDataByIdentifier
      m.data[2] == (uint8_t)(OPENHALDEX_LOCK_DID >> 8) &&
      m.data[3] == (uint8_t)(OPENHALDEX_LOCK_DID & 0xFF))
  {
    const float lt = lockTarget;
    const uint8_t cmd = (uint8_t)(lt < 0.0f ? 0.0f : (lt > 100.0f ? 100.0f : lt + 0.5f));
    twai_message_t resp{};
    resp.identifier = ISO_ALLRAD_RESP;
    resp.extd = 0;
    resp.rtr = 0;
    resp.data_length_code = 8;
    resp.data[0] = 0x06; // ISO-TP single frame, 6 payload bytes
    resp.data[1] = 0x62; // positive RDBI response
    resp.data[2] = (uint8_t)(OPENHALDEX_LOCK_DID >> 8);
    resp.data[3] = (uint8_t)(OPENHALDEX_LOCK_DID & 0xFF);
    resp.data[4] = cmd;                                        // commanded lock %, 0-100
    resp.data[5] = (uint8_t)receivedHaldexEngagementRaw;    // applied engagement, raw
    resp.data[6] = (uint8_t)state.mode;                        // live mode, so a write reads back
    resp.data[7] = 0xAA;                                       // ISO-TP padding
    canTransmit(twaiBus0, &resp);
    return true; // consume - do NOT forward to Bus 1
  }

  // Per-corner slip DID: 4 signed slip bytes [FL, FR, RL, RR]; -128 = no data
  // (too slow, or stale wheel/steering signal) so the gauge can blank rather
  // than draw a false zero.
  if (m.identifier == ISO_ALLRAD_REQ &&
      m.data_length_code >= 4 &&
      (m.data[0] & 0xF0) == 0x00 &&
      (m.data[0] & 0x0F) >= 3 &&
      m.data[1] == 0x22 &&
      m.data[2] == (uint8_t)(OPENHALDEX_SLIP_DID >> 8) &&
      m.data[3] == (uint8_t)(OPENHALDEX_SLIP_DID & 0xFF))
  {
    int8_t slip[4] = {-128, -128, -128, -128};
    if (lastCornerSlipMs != 0 && (millis() - lastCornerSlipMs) < 500)
    {
      slip[0] = cornerSlip[0];
      slip[1] = cornerSlip[1];
      slip[2] = cornerSlip[2];
      slip[3] = cornerSlip[3];
    }
    twai_message_t resp{};
    resp.identifier = ISO_ALLRAD_RESP;
    resp.extd = 0;
    resp.rtr = 0;
    resp.data_length_code = 8;
    resp.data[0] = 0x07; // ISO-TP single frame, 7 payload bytes (0x62 + DID + 4)
    resp.data[1] = 0x62;
    resp.data[2] = (uint8_t)(OPENHALDEX_SLIP_DID >> 8);
    resp.data[3] = (uint8_t)(OPENHALDEX_SLIP_DID & 0xFF);
    resp.data[4] = (uint8_t)slip[0];
    resp.data[5] = (uint8_t)slip[1];
    resp.data[6] = (uint8_t)slip[2];
    resp.data[7] = (uint8_t)slip[3];
    canTransmit(twaiBus0, &resp);
    return true;
  }

  // Drive-mode WRITE (WriteDataByIdentifier 0x2E): [len][2E][DID_hi][DID_lo][mode].
  // Sets the mode as a head unit would; replies 0x6E on success or 0x7F/0x31
  // (requestOutOfRange) for a bad value. Request is consumed.
  if (m.identifier == ISO_ALLRAD_REQ &&
      m.data_length_code >= 5 &&
      (m.data[0] & 0xF0) == 0x00 &&
      (m.data[0] & 0x0F) >= 4 &&
      m.data[1] == 0x2E &&
      m.data[2] == (uint8_t)(OPENHALDEX_MODE_DID >> 8) &&
      m.data[3] == (uint8_t)(OPENHALDEX_MODE_DID & 0xFF))
  {
    const uint8_t requestedMode = m.data[4];
    twai_message_t resp{};
    resp.identifier = ISO_ALLRAD_RESP;
    resp.extd = 0;
    resp.rtr = 0;
    resp.data_length_code = 8;
    if (requestMode(requestedMode)) // the shared rules (IO.cpp): refused while disabled / out of range
    {
      resp.data[0] = 0x03; // ISO-TP single frame, 3 payload bytes
      resp.data[1] = 0x6E; // positive WDBI response
      resp.data[2] = (uint8_t)(OPENHALDEX_MODE_DID >> 8);
      resp.data[3] = (uint8_t)(OPENHALDEX_MODE_DID & 0xFF);
      resp.data[4] = 0xAA;
      resp.data[5] = 0xAA;
      resp.data[6] = 0xAA;
      resp.data[7] = 0xAA;
    }
    else
    {
      resp.data[0] = 0x03;
      resp.data[1] = 0x7F; // negative response
      resp.data[2] = 0x2E; // ...to WriteDataByIdentifier
      resp.data[3] = 0x31; // requestOutOfRange
      resp.data[4] = 0xAA;
      resp.data[5] = 0xAA;
      resp.data[6] = 0xAA;
      resp.data[7] = 0xAA;
    }
    canTransmit(twaiBus0, &resp);
    return true;
  }
  return false;
}

// ---- Main code ---------------------------------------------------------------

void parseCANChs(void *arg)
{
  // Interrupt-driven receive: block on the TWAI driver's RX queue (signalled
  // from the TWAI ISR) instead of polling with vTaskDelay(1). The task is
  // unblocked the moment a frame is queued by the ISR, so latency is the queue
  // wakeup time (microseconds) rather than the previous ~1 ms polling tick.
  // CPU usage on an idle bus drops to effectively zero.
  while (1)
  {
#if detailedDebugStack
    stackCHS = uxTaskGetStackHighWaterMark(NULL);
#endif

    // Block until at least one frame is available, then read any further
    // frames already queued by the ISR before blocking again.
    if (twai_receive_v2(twaiBus0, &rxMessageChs, portMAX_DELAY) != ESP_OK)
    {
      // Driver was stopped/uninstalled (e.g. light-sleep entry); back off and retry.
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    do
    {
      if (canLoopTestCatch(rxMessageChs, 0))
      {
        continue; // CAN loop test frame (V11.00.12): not car traffic
      }
      chsRxCensus(rxMessageChs); // V11.00.12: CAN Test card - what Bus 0 is receiving
      uint32_t chassisNowMs = millis();
      if (chassisNowMs == 0)
      {
        chassisNowMs = 1; // 0 means "never seen" (millis() wraps to 0 every 49.7 days)
      }
      lastCANChassisTick = chassisNowMs;
      ++lpChassisFrameCount;
      espNowTapFrame(rxMessageChs); // relay to gauges over ESP-NOW (when on) - the frame as the car sent it

      chsNoteExternalTester(rxMessageChs); // a real scanner on the car's side pauses our polling

      // /api/uds/read tap (Bus 0 reads): COPY the response frame into the web
      // helper's queue - the frame continues down the normal gateway path below,
      // so the helper can never starve the bridge of chassis frames.
      // V11.00.13 Land Rover (gen 46): the unit takes 0x795 on one of its CAN controllers and answers 0x79D
      // from the other; one physical bus is not proven (audit_landrover #13), so a 0x79D is taken from
      // EITHER of our buses while a Bus-1 read waits for it.
      if (udsWebRespId != 0 && udsWebRxQueue != nullptr && rxMessageChs.identifier == udsWebRespId &&
          (udsWebBus == 0 || (haldexGeneration == GEN_LANDROVER_GEN4 && udsWebRespId == LR_DIAG_RESP)))
      {
        if (xQueueSend(udsWebRxQueue, &rxMessageChs, 0) != pdTRUE)
        {
          DEBUG("CAN - UDS web tap queue full, dropped frame 0x%03X", (unsigned)rxMessageChs.identifier);
        }
      }

      txMessageHdx.identifier = rxMessageChs.identifier;

      // Gen41 Haldex-originated Bus0 heartbeats - capture presence & timestamp.
      // Done before any mode so the API can show liveness even in standalone.
      if (haldexGeneration == 41)
      {
        if (rxMessageChs.identifier == HALDEX_GEN41_SEC_AXLE_GENINFO_ID)
        {
          receivedHaldexAliveBus0 = true;
          receivedHaldexAliveBus0Ms = millis();
        }
        else if (rxMessageChs.identifier == HALDEX_GEN41_DRIVETRAIN_STATE_ID)
        {
          receivedDrivetrainStateOk = true;
          receivedDrivetrainStateMs = millis();
        }
      }

      // Analyzer mode: queue for GVRET/SLCAN and forward untouched, skipping control logic.
      if (analyzerMode || analyzerSerial)
      {
        analyzerQueueFrame(rxMessageChs, 0);
        transmitFrameCopy(twaiBus1, rxMessageChs, txMessageHdx);
        continue;
      }

      // OpenHaldex supplier-specific UDS responders (lock / slip DIDs, mode write)
      if (chsAnswerOpenHaldexRequest(rxMessageChs))
      {
        continue; // answered and consumed - the real Haldex on Bus 1 never sees it
      }

      // Speed, pedal, rpm, brake, handbrake ... for the active generation
      if (!isStandalone || useCANifAvailable)
      {
        chsDecodeChassis(rxMessageChs);
      }

      // ---- To the Haldex (V11.00.7) ---------------------------------------
      // Frame-edit generations, while our frames are active (standalone, or
      // normal mode outside Stock): a block ON = our frame is the one the
      // Haldex gets (sent by the standalone task) and the car's is held back;
      // OFF = the car's frame goes through untouched (Follow Brake / Follow
      // Handbrake can still OR our input wires into it). Car IDs without a
      // block: normal mode passes them, standalone holds them back.
      if (ownFramesActive() && frameEditGenIdx(haldexGeneration) >= 0)
      {
        if (!carFrameToHaldex(rxMessageChs))
        {
          continue; // ours goes instead, or held back
        }
        if (!isStandalone)
        {
          applyBrakeHandbrakeCANOverride(rxMessageChs);
        }
        txMessageHdx = rxMessageChs;
        canTransmit(twaiBus1, &txMessageHdx);
        continue;
      }

      // Standalone on a generation without frame-edit blocks: only tester traffic
      // reaches the Haldex - the standalone task builds the rest.
      if (isStandalone)
      {
        if (carFrameToHaldex(rxMessageChs)) // true only for the diagnostic IDs here
        {
          txMessageHdx = rxMessageChs;
          canTransmit(twaiBus1, &txMessageHdx);
        }
        continue;
      }

      // Normal mode with the car's frames: Stock (untouched passthrough), the
      // controller disabled (hard passthrough), or a generation without blocks.
      if (disableController)
      {
        // Read-only bridge: no edits, no brake/handbrake override. Mirror the
        // engagement so 'Requested' doesn't show 0.
        lockTarget = receivedHaldexEngagement;
      }
      else
      {
        if (stockEffective() && !haldexLearnActive)
        {
          // Stock: mirror the engagement for telemetry only. Editing in Stock used
          // to send the engagement back as the request - a feedback loop that
          // latched the clutch at 100 % until FWD / power cycle.
          lockTarget = receivedHaldexEngagement;
        }
        else if (haldexGeneration == 41)
        {
          updateLockTarget(); // Gen4.1: lockTarget for its own outputs (frames pass untouched)
        }
        if (frameEditGenIdx(haldexGeneration) >= 0)
        {
          carFrameToHaldex(rxMessageChs); // capture only (Stock passes everything)
        }
        // V11.00.13 (Adam): Stock changes NOTHING - not even the Follow Brake / Handbrake bits.
        // The override only runs for the generations without blocks outside Stock.
        if (!(stockEffective() && !haldexLearnActive))
        {
          applyBrakeHandbrakeCANOverride(rxMessageChs);
        }
      }
      txMessageHdx = rxMessageChs;
      canTransmit(twaiBus1, &txMessageHdx);
    } while (twai_receive_v2(twaiBus0, &rxMessageChs, 0) == ESP_OK);
  }
}
