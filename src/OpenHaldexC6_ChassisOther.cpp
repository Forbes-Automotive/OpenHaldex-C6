#include <OpenHaldexC6_ChassisOther.h>

// =============================================================================
// Chassis decoding for the non-VW cars (V10.00.0)
// =============================================================================
// Fills the same values the VW decoder in parseCAN_chs does:
//   received_vehicle_speed (km/h), received_pedal_value (%), received_vehicle_rpm,
//   brakeFromCAN, handbrakeFromCAN
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
// =============================================================================

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
#define VOLVO_P3_PEDAL_ID 0x020  // AccPedal: 10 bits from byte 2 bits 1-0 and byte 3, 0.1 % per bit
#define VOLVO_P3_ENGINE_ID 0x12A // EngineData: brake switch byte 4 bit 6, RPM 13 bits from byte 6 bits 4-0 and byte 7
#define VOLVO_P3_SPEED_ID 0x148  // VehicleSpeed1: bytes 6-7, 0.01 km/h per bit

// ----- SAAB 9-3 XWD, P-bus (generation 44) -----
#define SAAB_93_ENGINE_ID 0x110    // bytes 1-2 = engine RPM x 4
#define SAAB_93_WHEELS_ID 0x280    // four wheel speeds, 16-bit each (bytes 0-1, 2-3, 4-5, 6-7), raw / 50 = mph
#define SAAB_93_HANDBRAKE_ID 0x380 // byte 5 bit 6 = handbrake applied (0x46 on, 0x06 off)

// ----- Volvo P2 pre-facelift, 2002 model-year IDs (generation 21, 29-bit) -----
#define VOLVO_P2_2002_SPEED_ID 0x00D28026  // bytes 6-7 = vehicle speed, 0.01 km/h per bit
#define VOLVO_P2_2002_ENGINE_ID 0x0032C01E // engine RPM = 13 bits from byte 6 bits 4-0 and byte 7
#define VOLVO_P2_2002_BRAKE_ID 0x0012C024  // byte 4 bit 5 = brake switch
#define VOLVO_P2_2002_PEDAL_ID 0x00E00020  // byte 3 = accelerator pedal: 19 at rest, about 187 floored
#define VOLVO_P2_2002_PEDAL_REST 19        // pedal byte with the foot off (2002 S60 logs)
#define VOLVO_P2_2002_PEDAL_FULL 187       // pedal byte floored (2002 S60 logs)

// Which generations this file decodes
bool hasOtherCarChassisDecode(uint8_t generation)
{
  if (generation == 41) return true;                // Vauxhall / Opel Insignia
  if (generation == 42) return true;                // Ford Focus RS
  if (generation == GEN_VOLVO_GEN4) return true;    // Volvo P3
  if (generation == GEN_SAAB_93_XWD) return true;   // SAAB 9-3
  if (generation == GEN_VOLVO_P2_GEN2) return true; // Volvo P2 pre-facelift (2002 IDs)
  if (generation == GEN_VOLVO_P1) return true;      // Volvo P1 S40 / V50
  return false;
}

// Vauxhall / Opel Insignia (GM Global A, 500 kbit/s, 11-bit)
static void decodeInsignia(const twai_message_t &frame)
{
  if (frame.data_length_code < 5)
  {
    return; // every frame used here has at least 5 bytes
  }

  if (frame.identifier == INSIGNIA_ENGINE_ID)
  {
    uint16_t rpmRaw = (frame.data[1] << 8) | frame.data[2]; // bytes 1-2, 0.25 rpm per bit
    received_vehicle_rpm = rpmRaw / 4;
    received_pedal_value = frame.data[4] * 100.0f / 255.0f; // byte 4, 0-255 = 0-100 %
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
    received_vehicle_speed = rearAverage / 32; // 1/32 km/h per bit
  }
}

// Ford Focus RS (500 kbit/s, 11-bit)
static void decodeFocusRS(const twai_message_t &frame)
{
  if (frame.identifier == FOCUS_ENGINE_ID && frame.data_length_code >= 7)
  {
    received_vehicle_rpm = (frame.data[0] << 8) | frame.data[1];     // bytes 0-1, 1 rpm per bit (845 at idle)
    uint16_t speedRaw = (frame.data[4] << 8) | frame.data[5];        // bytes 4-5, 0.01 km/h per bit
    received_vehicle_speed = speedRaw / 100;
    received_pedal_value = frame.data[6] * 0.5f;                     // byte 6, 0.5 % per bit (0-199)
  }
  else if (frame.identifier == FOCUS_BRAKE_ID && frame.data_length_code >= 3)
  {
    brakeFromCAN = bitRead(frame.data[2], 6); // byte 2 bit 6, 1 = pressed
  }
  else if (frame.identifier == FOCUS_HANDBRAKE_ID && frame.data_length_code >= 4)
  {
    handbrakeFromCAN = bitRead(frame.data[3], 0); // byte 3 bit 0, 1 = applied
  }
}

// Volvo P3 (500 kbit/s, 11-bit). Handbrake (0x2EE) is not decoded: the meaning
// of its state values isn't known yet.
static void decodeVolvoP3(const twai_message_t &frame)
{
  if (frame.data_length_code < 8)
  {
    return; // all Volvo P3 frames used here are 8 bytes
  }

  if (frame.identifier == VOLVO_P3_PEDAL_ID)
  {
    uint16_t pedalRaw = (frame.data[2] & 0x03) << 8; // byte 2 bits 1-0 = top 2 bits
    pedalRaw = pedalRaw | frame.data[3];             // byte 3 = bottom 8 bits
    received_pedal_value = pedalRaw * 0.1f;          // 0.1 % per bit
  }
  else if (frame.identifier == VOLVO_P3_ENGINE_ID)
  {
    brakeFromCAN = bitRead(frame.data[4], 6); // byte 4 bit 6 = brake pressed
    uint16_t rpmRaw = (frame.data[6] & 0x1F) << 8; // byte 6 bits 4-0 = top 5 bits
    rpmRaw = rpmRaw | frame.data[7];               // byte 7 = bottom 8 bits
    received_vehicle_rpm = rpmRaw;                 // 1 rpm per bit
  }
  else if (frame.identifier == VOLVO_P3_SPEED_ID)
  {
    uint16_t speedRaw = (frame.data[6] << 8) | frame.data[7]; // bytes 6-7, 0.01 km/h per bit
    received_vehicle_speed = speedRaw / 100;
  }
}

// SAAB 9-3 P-bus (500 kbit/s, 11-bit). Brake and throttle are not decoded:
// their positions are only guesses so far.
static void decodeSaab93(const twai_message_t &frame)
{
  if (frame.data_length_code < 8)
  {
    return; // all SAAB frames used here are 8 bytes
  }

  if (frame.identifier == SAAB_93_ENGINE_ID)
  {
    uint16_t rpmRaw = (frame.data[1] << 8) | frame.data[2]; // bytes 1-2, 4 counts per rpm
    received_vehicle_rpm = rpmRaw / 4;
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
    received_vehicle_speed = averageRaw * 1609 / 50000; // raw / 50 = mph, x 1.609 = km/h
  }
  else if (frame.identifier == SAAB_93_HANDBRAKE_ID)
  {
    handbrakeFromCAN = bitRead(frame.data[5], 6); // byte 5 bit 6 = handbrake applied
  }
}

// Volvo P2 pre-facelift, 2002 model-year ID set (250 kbit/s, 29-bit). Other
// model years use different IDs, so these simply won't appear on them.
// Handbrake has not been found yet.
static void decodeVolvoP2Gen2(const twai_message_t &frame)
{
  if (frame.data_length_code < 8)
  {
    return; // all P2 frames used here are 8 bytes
  }

  if (frame.identifier == VOLVO_P2_2002_SPEED_ID)
  {
    uint16_t speedRaw = (frame.data[6] << 8) | frame.data[7]; // bytes 6-7, 0.01 km/h per bit
    received_vehicle_speed = speedRaw / 100;
  }
  else if (frame.identifier == VOLVO_P2_2002_ENGINE_ID)
  {
    uint16_t rpmRaw = (frame.data[6] & 0x1F) << 8; // byte 6 bits 4-0 = top 5 bits
    rpmRaw = rpmRaw | frame.data[7];               // byte 7 = bottom 8 bits
    received_vehicle_rpm = rpmRaw;                 // 1 rpm per bit
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
    received_pedal_value = pedalPercent;
  }
}

// Volvo P1 S40 / V50 (500 kbit/s, 29-bit). IDs are in canID.h (VOLVO_P1_*);
// byte positions from Adam's 2005 V50 T5 AWD logs.
static void decodeVolvoP1(const twai_message_t &frame)
{
  if (frame.data_length_code < 8)
  {
    return; // every P1 frame is 8 bytes
  }

  if (frame.identifier == VOLVO_P1_SPEED_ID)
  {
    uint16_t speedRaw = (frame.data[6] << 8) | frame.data[7]; // bytes 6-7, 0.01 km/h per bit (assumed - check against the speedo)
    received_vehicle_speed = speedRaw / 100;
  }
  else if (frame.identifier == VOLVO_P1_ENGINE_RPM_ID)
  {
    uint16_t rpmRaw = (frame.data[2] & 0x1F) << 8; // byte 2 bits 4-0 = top 5 bits
    rpmRaw = rpmRaw | frame.data[3];               // byte 3 = bottom 8 bits
    received_vehicle_rpm = rpmRaw;                 // 1 rpm per bit (about 770 at idle)
  }
  else if (frame.identifier == VOLVO_P1_ENGINE_TORQUE_ID)
  {
    received_pedal_value = frame.data[7] * 100.0f / 255.0f; // byte 7 = pedal, 0-255 (about 250 floored)
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

// Read one chassis frame for whichever of these cars is selected
void decodeOtherCarChassis(const twai_message_t &frame)
{
  if (haldexGeneration == GEN_VOLVO_P2_GEN2)
  {
    if (frame.extd)
    {
      decodeVolvoP2Gen2(frame); // Volvo P2 uses 29-bit IDs
    }
    return;
  }

  if (haldexGeneration == GEN_VOLVO_P1)
  {
    if (frame.extd)
    {
      decodeVolvoP1(frame); // Volvo P1 uses 29-bit IDs
    }
    return;
  }

  if (frame.extd)
  {
    return; // the other cars use 11-bit IDs for these signals
  }

  if (haldexGeneration == GEN_SAAB_93_XWD)
  {
    decodeSaab93(frame);
  }
  else if (haldexGeneration == 41)
  {
    decodeInsignia(frame);
  }
  else if (haldexGeneration == 42)
  {
    decodeFocusRS(frame);
  }
  else if (haldexGeneration == GEN_VOLVO_GEN4)
  {
    decodeVolvoP3(frame);
  }
}
