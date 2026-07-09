#include <OpenHaldexC6_StandaloneCAN.h>
#include <OpenHaldexC6_Calculations.h>

// Standalone frames honour the same frame-edit mask as the passthrough path: a
// block turned OFF in the UI is simply not generated (mirrors the passthrough
// "leave the frame untouched" behaviour). Frames that are not listed as editable
// blocks for the active generation always send. All standalone Haldex-frame
// transmits go through here instead of calling twai_transmit_v2 directly.
static inline void standaloneTx(twai_message_t &f)
{
  int gi = frameEditGenIdx(haldexGeneration);
  if (gi >= 0)
  {
    for (uint16_t i = 0; i < frameEditBlockCount; i++)
    {
      if (frameEditBlocks[i].genIdx == (uint8_t)gi &&
          frameEditBlocks[i].canId == f.identifier)
      {
        if (!frameEditEnabled((uint8_t)gi, frameEditBlocks[i].bit))
          return; // block disabled -> do not generate this frame
        break;
      }
    }
  }
  twai_transmit_v2(twai_bus_1, &f, 0);
}

// Periodic frame tasks
void frames10(void *arg)
{
  while (1)
  {
    // Analyzer mode runs as a passive bridge; skip standalone frame generation.
    if (analyzerMode)
    {
      vTaskDelay(10 / portTICK_PERIOD_MS);
      continue;
    }
    if (isStandalone)
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
      case 50:
        Gen5_0CQ_frames10();
        break;
      case 51:
        Gen5_0AY_frames10();
        break;
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void frames20(void *arg)
{
  while (1)
  {
    // Analyzer mode runs as a passive bridge; skip standalone frame generation.
    if (analyzerMode)
    {
      vTaskDelay(20 / portTICK_PERIOD_MS);
      continue;
    }
    if (isStandalone)
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
      case 50:
        Gen5_0CQ_frames20();
        break;
      case 51:
        Gen5_0AY_frames20();
        break;
      }
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void frames25(void *arg)
{
  while (1)
  {
    // Analyzer mode runs as a passive bridge; skip standalone frame generation.
    if (analyzerMode)
    {
      vTaskDelay(25 / portTICK_PERIOD_MS);
      continue;
    }
    if (isStandalone)
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
      }
    }
    vTaskDelay(25 / portTICK_PERIOD_MS);
  }
}

void frames100(void *arg)
{
  while (1)
  {
    // Analyzer mode runs as a passive bridge; skip standalone frame generation.
    if (analyzerMode)
    {
      vTaskDelay(100 / portTICK_PERIOD_MS);
      continue;
    }
    if (isStandalone)
    {
      lock_target = get_lock_target_adjustment();
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
      case 50:
        Gen5_0CQ_frames100();
        break;
      case 51:
        Gen5_0AY_frames100();
        break;
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void frames200(void *arg)
{
  while (1)
  {
    // Analyzer mode runs as a passive bridge; skip standalone frame generation.
    if (analyzerMode)
    {
      vTaskDelay(200 / portTICK_PERIOD_MS);
      continue;
    }
    if (isStandalone)
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
      }
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void frames1000(void *arg)
{
  while (1)
  {
    // Analyzer mode runs as a passive bridge; skip standalone frame generation.
    if (analyzerMode)
    {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }
    if (isStandalone)
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
      case 50:
        Gen5_0CQ_frames1000();
        break;
      case 51:
        Gen5_0AY_frames1000();
        break;
      }
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// Gen1 Standalone Frames
// Synthesises the minimum PQ messages the haldex needs while no donor chassis bus is
// present. All frames go out on the haldex bus (twai_bus_1).
void Gen1_frames10()
{
  // not used in Gen1
}

void Gen1_frames20()
{
  twai_message_t frame;
  // PQ Motor_1 (0x280, DLC 8) - main engine ECU broadcast (vw_pq.dbc: Motor_1).
  frame.identifier = MOTOR1_ID;
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;                                        // status / Momentenangaben_ungenau (torque-inaccurate) flag bits
  frame.data[1] = get_lock_target_adjusted_value(0xFE, false); // Fahrerwunschmoment (driver-requested torque)
  frame.data[2] = 0x21;                                        // Motordrehzahl low byte (engine RPM * 0.25)
  frame.data[3] = get_lock_target_adjusted_value(0x4E, false); // Motordrehzahl high byte
  frame.data[4] = get_lock_target_adjusted_value(0xFE, false); // Fahrpedalwert (accelerator pedal %)
  frame.data[5] = get_lock_target_adjusted_value(0xFE, false); // inneres_Motor_Moment_ohne_extern (inner torque w/o external)

  switch (state.mode)
  {
  case MODE_FWD:
    appliedTorque = get_lock_target_adjusted_value(0xFE, true);
    break;
  case MODE_5050:
    appliedTorque = get_lock_target_adjusted_value(0x16, false);
    break;
  case MODE_6040:
    appliedTorque = get_lock_target_adjusted_value(0x22, false);
    break;
  case MODE_7525:
    appliedTorque = get_lock_target_adjusted_value(0x50, false);
    break;
  default:
    appliedTorque = 0;
    break;
  }

  frame.data[6] = appliedTorque; // mechanisches_Motor_Verlustmoment - openHaldex applied-torque demand
  frame.data[7] = 0x00;          // inneres_Motor_Moment (actual inner torque) - left zero in standalone
  standaloneTx(frame);

  // PQ Motor_3 (0x380, DLC 8) - secondary engine ECU broadcast (vw_pq.dbc: Motor_3).
  frame.identifier = MOTOR3_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // status flags (MO3_Sta_Pedal etc.)
  frame.data[1] = 0x50; // MO3_Pedalwert / MO3_Vorgluehen mux
  frame.data[2] = 0x00; // MO3_Rad_Wu_Mo low (wheel-torque request)
  frame.data[3] = 0x00; // MO3_Rad_Wu_Mo high
  frame.data[4] = 0x00; // MO3_Offsentemp (intake-air offset)
  frame.data[5] = 0x00; // reserved / Winterprg / Freigabe_Segeln
  frame.data[6] = 0x00; // reserved
  frame.data[7] = 0xFE; // reserved (kept non-zero for haldex sanity)
  standaloneTx(frame);

  // PQ Bremse_1 (0x1A0, DLC 8) - ABS/ESP main broadcast.
  // Bremse = brake. vw_pq.dbc signals include BR1_ASR_passiv, BR1_ESPASR_passiv,
  // BR1_Rad_kmh (wheel speed) and warning lamps.
  // data[7] is the frame counter (BR1_BZ).
  frame.identifier = BRAKES1_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x80;
  frame.data[1] = get_lock_target_adjusted_value(0x00, false);
  frame.data[2] = 0x00;
  frame.data[3] = 0x0A;
  frame.data[4] = 0xFE;
  frame.data[5] = 0xFE;
  frame.data[6] = 0x00;
  frame.data[7] = BRAKES1_counter;
  if (++BRAKES1_counter > 0xF)
    BRAKES1_counter = 0;
  standaloneTx(frame);

  // PQ Bremse_3 (0x4A0, DLC 8) - per-wheel speeds (vw_pq.dbc: Bremse_3, * 0.01 km/h).
  // Lock-adjusted low bytes "fake" wheel slip to nudge haldex engagement.
  frame.identifier = BRAKES3_ID;
  frame.data_length_code = 8;
  frame.data[0] = get_lock_target_adjusted_value(0xFE, false); // Radgeschw_VL low (front-left wheel)
  frame.data[1] = 0x0A;                                        // Radgeschw_VL high
  frame.data[2] = get_lock_target_adjusted_value(0xFE, false); // Radgeschw_VR low (front-right)
  frame.data[3] = 0x0A;                                        // Radgeschw_VR high
  frame.data[4] = 0x00;                                        // Radgeschw_HL low (rear-left)
  frame.data[5] = 0x0A;                                        // Radgeschw_HL high
  frame.data[6] = 0x00;                                        // Radgeschw_HR low (rear-right)
  frame.data[7] = 0x0A;                                        // Radgeschw_HR high
  standaloneTx(frame);
}

void Gen1_frames25()
{
  // not used in Gen1
}
void Gen1_frames100()
{
  // not used in Gen1
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
// All transmitted on the haldex bus (twai_bus_1).
void Gen2_frames10()
{
  twai_message_t frame;
  // PQ Bremse_1 (0x1A0, DLC 8) - ABS/ESP main broadcast (vw_pq.dbc: Bremse_1).
  frame.identifier = BRAKES1_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;            // BR1_ASR_Anf / MSR_Anf engine-torque request bits
  frame.data[1] = 0x41;            // BR1_ABS_Brems / EDS_Ongr / ESP_Ongr (intervention flags)
  frame.data[2] = 0x00;            // BR1_Lampe_ABS / BR1_Lampe_ASR
  frame.data[3] = 0xFE;            // status / ABS active flags
  frame.data[4] = 0xFE;            // BR1_Rad_kmh low (vehicle speed)
  frame.data[5] = 0xFE;            // BR1_Rad_kmh high
  frame.data[6] = 0x00;            // reserved
  frame.data[7] = BRAKES1_counter; // BR1_BZ - rolling 4-bit counter
  if (++BRAKES1_counter > 0xF)
    BRAKES1_counter = 0;
  standaloneTx(frame);

  // PQ Bremse_2 (0x5A0, DLC 8) - ESP/ABS sensor broadcast (vw_pq.dbc: Bremse_2).
  frame.identifier = BRAKES2_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x7F;                                        // Impulszahl (pulse count, low)
  frame.data[1] = 0xAE;                                        // Wegimpulse_Vorderachse (front-axle path pulses)
  frame.data[2] = 0x3D;                                        // mittlere_Raddrehzahl (mean wheel speed * 0.002 U/sec)
  frame.data[3] = BRAKES2_counter;                             // rolling counter (steps of 0x10)
  frame.data[4] = get_lock_target_adjusted_value(0x7F, false); // Querbeschleunigung (lateral G * 0.01)
  frame.data[5] = get_lock_target_adjusted_value(0xFE, false); // Zeitstempel low (timestamp tics)
  frame.data[6] = 0x5E;                                        // Zeitstempel high
  frame.data[7] = 0x2B;                                        // status / reserved
  BRAKES2_counter = BRAKES2_counter + 10;
  if (BRAKES2_counter > 0xF7)
    BRAKES2_counter = 7;
  standaloneTx(frame);

  // PQ Bremse_3 (0x4A0, DLC 8) - per-wheel speeds (see Gen1 for signal layout).
  frame.identifier = BRAKES3_ID;
  frame.data_length_code = 8;
  frame.data[0] = get_lock_target_adjusted_value(0xFE, false); // Radgeschw_VL low
  frame.data[1] = 0x0A;                                        // Radgeschw_VL high
  frame.data[2] = get_lock_target_adjusted_value(0xFE, false); // Radgeschw_VR low
  frame.data[3] = 0x0A;                                        // Radgeschw_VR high
  frame.data[4] = 0x00;                                        // Radgeschw_HL low
  frame.data[5] = 0x0A;                                        // Radgeschw_HL high
  frame.data[6] = 0x00;                                        // Radgeschw_HR low
  frame.data[7] = 0x0A;                                        // Radgeschw_HR high
  standaloneTx(frame);

  // PQ Bremse_4 (0x2A0, DLC 8 here / 3 per dbc) - ABS coupling-moment broadcast.
  // vw_pq.dbc: ABS_Vorgabewert_hinten_Kupplung (rear coupling % 0..100),
  //            ABS_Vorgabewert_mitte_Kupplungs (centre coupling Nm/min, -381..378).
  frame.identifier = BRAKES4_ID;   // 0x2A0 includes coupling moment? Affects vehicle mode(!) do not have!
  frame.data_length_code = 8;      // DLC 8 (/4)
  frame.data[0] = 0x00;            // affects estimated torque AND vehicle mode(!) - ABS_Vorgabewert_hinten_Kupplung
  frame.data[1] = 0x00;            // ABS_Vorgabewert_mitte_Kupplungs low
  frame.data[2] = 0x00;            // ABS_Vorgabewert_mitte_Kupplungs high / status
  frame.data[3] = 0x00;            // 32605 - reserved/status
  frame.data[4] = 0x00;            // reserved
  frame.data[5] = 0x00;            // reserved
  frame.data[6] = BRAKES4_counter; // checksum / rolling counter
  frame.data[7] = BRAKES4_counter; // checksum (mirror)
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
  frame.identifier = BRAKES5_ID;    // 0x2A0 includes coupling moment? Affects vehicle mode(!) do not have!
  frame.data_length_code = 8;       // DLC 8 (/4)
  frame.data[0] = 0xFE;             // affects estimated torque AND vehicle mode(!) - Giergeschw low (yaw rate)
  frame.data[1] = 0x7F;             // Giergeschw high
  frame.data[2] = 0x03;             // BR5_Bremsdruck (brake pressure)
  frame.data[3] = 0x00;             // 32605 - status / Bremslicht / Notbremsung bits
  frame.data[4] = 0x00;             // BR5_Gierrate low (signed yaw rate)
  frame.data[5] = 0x00;             // BR5_Gierrate high
  frame.data[6] = BRAKES5_counter;  // checksum / rolling counter
  frame.data[7] = BRAKES5_counter2; // checksum / second rolling counter
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

  // PQ Bremse_9 (0x0AE, DLC 8) - extended ABS/ESP frame (NOT in vw_pq.dbc).
  // Reverse-engineered: only data[6] really matters (0x02 OK, 0x01 changes haldex behaviour).
  frame.identifier = BRAKES9_ID;    // 0x0AE do not have!
  frame.data_length_code = 8;       // DLC 8
  frame.data[0] = BRAKES9_counter;  // checksum / rolling counter A
  frame.data[1] = BRAKES9_counter2; // checksum / rolling counter B
  frame.data[2] = 0x00;             // no effect
  frame.data[3] = 0x00;             // no effect
  frame.data[4] = 0x00;             // no effect
  frame.data[5] = 0x00;             // no effect
  frame.data[6] = 0x02;             // 0x01 - mode-control byte (only one that changes haldex behaviour)
  frame.data[7] = 0x00;             // no effect
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
  frame.data[0] = 0x20;          // LW1_LRW low (steering angle * 0.04375 deg)
  frame.data[1] = 0x00;          // LW1_LRW high
  frame.data[2] = 0x00;          // LW1_LRW_Sign (0=left, 1=right) / status
  frame.data[3] = 0x00;          // LW1_Lenk_Gesch low (angular velocity * 0.04375 deg/s)
  frame.data[4] = 0x80;          // LW1_Lenk_Gesch high / LW1_Gesch_Sign
  frame.data[5] = mLW_1_counter; // LW1_Status / rolling counter
  frame.data[6] = 0x00;          // LW1_ID (calibration ID) / Initquelle
  mLW_1_crc = 255 - (frame.data[0] + frame.data[1] + frame.data[2] + frame.data[3] + frame.data[5]);
  frame.data[7] = mLW_1_crc; // inverted-sum CRC over data[0..3]+data[5]
  mLW_1_counter = mLW_1_counter + 16;
  if (mLW_1_counter >= 0xF0)
    mLW_1_counter = 0;
  standaloneTx(frame);
}

void Gen2_frames20()
{
  twai_message_t frame;
  // PQ Motor_1 (0x280, DLC 8) - engine ECU broadcast (vw_pq.dbc: Motor_1).
  // Lock-adjusted bytes bias inneres_Motor_Moment / mechanisches_Verlustmoment so the
  // haldex thinks the engine is producing more torque than it actually is.
  frame.identifier = MOTOR1_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x08;                                        // status / Momentenangaben_ungenau bits
  frame.data[1] = 0xFA;                                        // Fahrerwunschmoment (driver-requested torque)
  frame.data[2] = 0x20;                                        // Motordrehzahl low (engine RPM * 0.25)
  frame.data[3] = get_lock_target_adjusted_value(0x4E, false); // Motordrehzahl high (lock-biased)
  frame.data[4] = 0xFA;                                        // Fahrpedalwert (accelerator pedal %)
  frame.data[5] = 0xFA;                                        // inneres_Motor_Moment_ohne_extern
  frame.data[6] = get_lock_target_adjusted_value(0x20, false); // mechanisches_Motor_Verlustmoment (lock-biased)
  frame.data[7] = 0xFA;                                        // inneres_Motor_Moment (actual)
  standaloneTx(frame);

  // PQ Motor_2 (0x288, DLC 8) - secondary engine broadcast (vw_pq.dbc: Motor_2).
  frame.identifier = MOTOR2_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // MO2_BLS / MO2_BTS / MO2_Sta_Klima (brake-light, brake-test, A/C)
  frame.data[1] = 0x30; // MO2_Kuehlm_T (coolant temp * 0.75 - 48 degC)
  frame.data[2] = 0x00; // MO2_Sta_Kuehlm / MO2_Sta_No_Bet (status flags)
  frame.data[3] = 0x0A; // MO2_GRA_Soll low (cruise-control target speed)
  frame.data[4] = 0x0A; // MO2_GRA_Soll high
  frame.data[5] = 0x10; // GRA / cruise status bits
  frame.data[6] = 0xFE; // reserved
  frame.data[7] = 0xFE; // reserved
  standaloneTx(frame);

  // PQ Motor_5 (0x480, DLC 8) - tertiary engine broadcast (vw_pq.dbc: Motor_5, multiplexed).
  // NOTE: the if(++BRAKES1_counter > 255) below increments BRAKES1_counter (not MOTOR5_counter)
  // - looks like a copy-paste bug, left as-is per "comment-only" pass.
  frame.identifier = MOTOR5_ID;   // 0x1A0
  frame.data_length_code = 8;     // DLC 8
  frame.data[0] = 0xFE;           // ASR 0x04 sets bit 4.  0x08 removes set.  Coupling open/closed - MO5_Mp_Code mux
  frame.data[1] = 0x00;           // can use to disable (>130 dec).  Was 0x00; 0x41?  0x43? - MO5_max_Moment / Drehzahl (mux)
  frame.data[2] = 0x00;           // was 0x00 no effect - MO5_Vorgluehen / E_Gas / OBD_2 lamps
  frame.data[3] = 0x00;           // was 0xFE no effect - MO5_Luefter (cooling-fan PWM)
  frame.data[4] = 0x00;           // was 0xFE miasrl no effect - reserved
  frame.data[5] = 0x00;           // was 0xFE miasrs no effect - reserved
  frame.data[6] = 0x00;           // was 0x00 - reserved
  frame.data[7] = MOTOR5_counter; // checksum / rolling counter
  if (++BRAKES1_counter > 255)
  {                      // 0xF (NOTE: increments BRAKES1_counter - copy-paste artefact)
    BRAKES1_counter = 0; // 0
  }
  standaloneTx(frame);

  // PQ Bremse_10 (0x3A0, DLC 8) - extended ABS/ESP frame (NOT in vw_pq.dbc).
  // Inline notes are pre-existing reverse-engineering observations.
  frame.identifier = BRAKES10_ID;   // 0x1A0
  frame.data_length_code = 8;       // DLC 8
  frame.data[0] = 0xA6;             // ASR 0x04 sets bit 4.  0x08 removes set.  Coupling open/closed
  frame.data[1] = BRAKES10_counter; // can use to disable (>130 dec).  Was 0x00; 0x41?  0x43? - rolling counter
  frame.data[2] = 0x75;             // was 0x00 no effect
  frame.data[3] = 0xD4;             // was 0xFE no effect
  frame.data[4] = 0x51;             // was 0xFE miasrl no effect
  frame.data[5] = 0x47;             // was 0xFE miasrs no effect
  frame.data[6] = 0x1D;             // was 0x00
  frame.data[7] = 0x0F;             // checksum (fixed)
  BRAKES10_counter = BRAKES10_counter + 1;
  if (BRAKES10_counter > 0xF)
  {
    BRAKES10_counter = 0;
  }
  standaloneTx(frame);
}

void Gen2_frames25()
{
  twai_message_t frame;
  // PQ Kombi_1 (0x320, DLC 8) - instrument-cluster broadcast (vw_pq.dbc: Kombi_1).
  frame.identifier = mKombi_1;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // Bremsinfo / Oeldruck status bits
  frame.data[1] = 0x02; // Angezeigte_Geschwindigkeit (displayed speed * 0.32 km/h)
  frame.data[2] = 0x00; // Geschwindigkeit_Kombi_1 low (raw vehicle speed * 0.01 km/h)
  frame.data[3] = 0x00; // Geschwindigkeit_Kombi_1 high
  frame.data[4] = 0x36; // Tankinhalt (fuel level, 1..126 l)
  frame.data[5] = 0x00; // Dynamische_Oeldruckwarnung / status
  frame.data[6] = 0x00; // reserved
  frame.data[7] = 0x00; // reserved
  standaloneTx(frame);
}

void Gen2_frames100() {}
void Gen2_frames200() {}
void Gen2_frames1000() {}

// Gen4 Standalone Frames (PQ-derived, late VAQ).
// Replays steering (LW_1) from a 16-step capture (lws_2[]) and synthesises engine/brake
// frames. BRAKES4 carries an XOR CRC over data[0..6] in data[7].
// All frames go out on the haldex bus (twai_bus_1).
void Gen4_frames10()
{
  twai_message_t frame;
  // PQ LW_1 / mLW_1 (0x0C2, DLC 8) - steering-angle replay (vw_pq.dbc: LW_1).
  frame.identifier = mLW_1;
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = lws_2[mLW_1_counter][0]; // LW1_LRW low (steering-wheel angle * 0.04375 deg)
  frame.data[1] = lws_2[mLW_1_counter][1]; // LW1_LRW high
  frame.data[2] = lws_2[mLW_1_counter][2]; // LW1_LRW_Sign / status
  frame.data[3] = lws_2[mLW_1_counter][3]; // LW1_Lenk_Gesch low (angular velocity)
  frame.data[4] = lws_2[mLW_1_counter][4]; // LW1_Lenk_Gesch high / Gesch_Sign
  frame.data[5] = lws_2[mLW_1_counter][5]; // LW1_Status / counter
  frame.data[6] = lws_2[mLW_1_counter][6]; // LW1_ID / Initquelle
  frame.data[7] = lws_2[mLW_1_counter][7]; // CRC (replayed verbatim)

  mLW_1_counter++;
  if (mLW_1_counter > 15)
    mLW_1_counter = 0;
  standaloneTx(frame);

  // PQ Bremse_1 (0x1A0, DLC 8) - ABS/ESP main broadcast (vw_pq.dbc: Bremse_1).
  frame.identifier = BRAKES1_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x20;                                        // BR1_ASR_Anf / MSR_Anf flags
  frame.data[1] = 0x40;                                        // ABS_Brems / EDS_Ongr / ESP_Ongr intervention bits
  frame.data[2] = 0xF0;                                        // Lampe_ABS / Lampe_ASR
  frame.data[3] = 0x07;                                        // status / ABS active flags
  frame.data[4] = get_lock_target_adjusted_value(0xFE, false); // BR1_Rad_kmh low (vehicle speed)
  frame.data[5] = get_lock_target_adjusted_value(0xFE, false); // BR1_Rad_kmh high
  frame.data[6] = 0x00;                                        // reserved
  frame.data[7] = BRAKES1_counter;                             // BR1_BZ - rolling 5-bit counter (10..0x1F)
  if (++BRAKES1_counter > 0x1F)
    BRAKES1_counter = 10;
  standaloneTx(frame);

  // PQ Bremse_3 (0x4A0, DLC 8) - per-wheel speeds (vw_pq.dbc: Bremse_3).
  frame.identifier = BRAKES3_ID;
  frame.data_length_code = 8;
  frame.data[0] = get_lock_target_adjusted_value(0xB6, false); // Radgeschw_VL low (front-left)
  frame.data[1] = 0x07;                                        // Radgeschw_VL high
  frame.data[2] = get_lock_target_adjusted_value(0xCC, false); // Radgeschw_VR low (front-right)
  frame.data[3] = 0x07;                                        // Radgeschw_VR high
  frame.data[4] = get_lock_target_adjusted_value(0xD2, false); // Radgeschw_HL low (rear-left)
  frame.data[5] = 0x07;                                        // Radgeschw_HL high
  frame.data[6] = get_lock_target_adjusted_value(0xD2, false); // Radgeschw_HR low (rear-right)
  frame.data[7] = 0x07;                                        // Radgeschw_HR high
  standaloneTx(frame);

  if(haldexLearnActive || state.mode != MODE_5050)
  {
    appliedTorque = get_lock_target_adjusted_value(0x7F, false); // regulated clamp
  }
  else
  {
    appliedTorque = get_lock_target_adjusted_value(0xFE, false); // full clamp (27 bar)
  }

  // PQ Bremse_4 (0x2A0, DLC 8) - ABS coupling moment (vw_pq.dbc: Bremse_4).
  // data[7] = XOR-CRC over data[0..6]; data[6] is rolling counter (steps of 16).
  frame.identifier = BRAKES4_ID;
  frame.data_length_code = 8;
  frame.data[0] = appliedTorque; // ABS_Vorgabewert_hinten_Kupplung (rear-clutch %)
  frame.data[1] = 0x00;                                        // ABS_Vorgabewert_mitte_Kupplungs low (centre stiffness Nm/min)
  frame.data[2] = 0x00;                                        // ABS_Vorgabewert_mitte_Kupplungs high
  frame.data[3] = 0x64;                                        // status / reserved
  frame.data[4] = 0x00;                                        // reserved
  frame.data[5] = 0x00;                                        // reserved
  frame.data[6] = BRAKES4_counter;                             // rolling counter (16-step)
  BRAKES4_crc = 0;
  for (uint8_t i = 0; i < 7; i++)
  {
    BRAKES4_crc ^= frame.data[i];
  }
  frame.data[7] = BRAKES4_crc; // XOR CRC over data[0..6]
  BRAKES4_counter = BRAKES4_counter + 16;
  if (BRAKES4_counter > 0xF0)
    BRAKES4_counter = 0x00;
  standaloneTx(frame);

  // PQ Motor_1 (0x280, DLC 8) - engine ECU broadcast (vw_pq.dbc: Motor_1).
  // Every byte except data[0] is lock-target-adjusted to bias the haldex.
  frame.identifier = MOTOR1_ID;
  frame.data_length_code = 8;
  // data[0] left untouched (whatever frame held previously) - status/Momentenangaben_ungenau
  frame.data[1] = get_lock_target_adjusted_value(0xFE, false); // Fahrerwunschmoment (driver-requested torque)
  frame.data[2] = get_lock_target_adjusted_value(0x20, false); // Motordrehzahl low (engine RPM * 0.25)
  frame.data[3] = get_lock_target_adjusted_value(0x4E, false); // Motordrehzahl high
  frame.data[4] = get_lock_target_adjusted_value(0xFE, false); // Fahrpedalwert (accelerator pedal %)
  frame.data[5] = get_lock_target_adjusted_value(0xFE, false); // inneres_Motor_Moment_ohne_extern
  frame.data[6] = get_lock_target_adjusted_value(0x16, false); // mechanisches_Motor_Verlustmoment
  frame.data[7] = get_lock_target_adjusted_value(0xFE, false); // inneres_Motor_Moment (actual)
  standaloneTx(frame);
}

void Gen4_frames20()
{
  twai_message_t frame;
  // PQ Bremse_2 (0x5A0, DLC 8) - ESP/ABS sensor broadcast (vw_pq.dbc: Bremse_2).
  frame.identifier = BRAKES2_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x80;                                        // Impulszahl (pulse count)
  frame.data[1] = 0x7A;                                        // Wegimpulse_Vorderachse (front-axle path pulses)
  frame.data[2] = 0x05;                                        // mittlere_Raddrehzahl (mean wheel speed)
  frame.data[3] = BRAKES2_counter;                             // rolling counter (16-step)
  frame.data[4] = get_lock_target_adjusted_value(0x7F, false); // Querbeschleunigung (lateral G)
  frame.data[5] = 0xCA;                                        // Zeitstempel low (timestamp)
  frame.data[6] = 0x1B;                                        // Zeitstempel high
  frame.data[7] = 0xAB;                                        // status / reserved
  standaloneTx(frame);
  BRAKES2_counter = BRAKES2_counter + 16;
  if (BRAKES2_counter > 0xF0)
    BRAKES2_counter = 0;
}

void Gen4_frames25()
{
  twai_message_t frame;
  // PQ Kombi_1 (0x320, DLC 8) - instrument-cluster broadcast (vw_pq.dbc: Kombi_1).
  frame.identifier = mKombi_1;
  frame.data_length_code = 8;
  frame.data[0] = 0x24; // Bremsinfo / Oeldruck status bits
  frame.data[1] = 0x00; // Angezeigte_Geschwindigkeit (displayed speed)
  frame.data[2] = 0x1D; // Geschwindigkeit_Kombi_1 low (vehicle speed)
  frame.data[3] = 0xB9; // Geschwindigkeit_Kombi_1 high
  frame.data[4] = 0x07; // Tankinhalt (fuel level)
  frame.data[5] = 0x42; // Dynamische_Oeldruckwarnung / status
  frame.data[6] = 0x09; // reserved / multiplex
  frame.data[7] = 0x81; // reserved
  standaloneTx(frame);

  // PQ Kombi_3 (0x520, DLC 8) - cluster odometer/keys (vw_pq.dbc: Kombi_3).
  frame.identifier = mKombi_3;
  frame.data_length_code = 8;
  frame.data[0] = 0x60; // Kilometerstand low (odometer, km)
  frame.data[1] = 0x43; // Kilometerstand mid
  frame.data[2] = 0x01; // Kilometerstand high
  frame.data[3] = 0x10; // Standzeit low (stand time, sec)
  frame.data[4] = 0x66; // Standzeit high
  frame.data[5] = 0xF1; // Schluesselinfo (key info)
  frame.data[6] = 0x03; // Kombi_Multiplex_Code
  frame.data[7] = 0x02; // Kombi_Multiplex_Generation
  standaloneTx(frame);
}

void Gen4_frames100()
{
  twai_message_t frame;
  // PQ Gate_Komf_1 (0x390, DLC 8) - gateway-comfort broadcast (vw_pq.dbc: Gate_Komf_1).
  frame.identifier = mGate_Komf_1;
  frame.data_length_code = 8;
  frame.data[0] = 0x03; // GK1_Sta_Anhaen / door & light status bits
  frame.data[1] = 0x11; // GK1_Blinker_li / re (indicators)
  frame.data[2] = 0x58; // Rueckfahrscheinwerfer (reverse light) / alarm bits
  frame.data[3] = 0x00; // GK1_Sta_RDK_Warn (TPMS warning)
  frame.data[4] = 0x40; // seat-memory / sunroof bits
  frame.data[5] = 0x00; // alarm / lock bits
  frame.data[6] = 0x01; // status
  frame.data[7] = 0x08; // status / counter
  standaloneTx(frame);

  // PQ Bremse_11 (0x5B7, DLC 8) - extended brake frame (NOT in vw_pq.dbc).
  // Static placeholder; only data[1]=0xC0 is significant for haldex sanity.
  frame.identifier = BRAKES11_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // no effect
  frame.data[1] = 0XC0; // mode/status (only byte that matters)
  frame.data[2] = 0x00; // no effect
  frame.data[3] = 0x00; // no effect
  frame.data[4] = 0x00; // no effect
  frame.data[5] = 0x00; // no effect
  frame.data[6] = 0x00; // no effect
  frame.data[7] = 0x00; // no effect
  standaloneTx(frame);
}

void Gen4_frames200()
{
  twai_message_t frame;
  // PQ Kombi_2 (0x420, DLC 8) - cluster temps (vw_pq.dbc: Kombi_2).
  frame.identifier = mKombi_2;
  frame.data_length_code = 8;
  frame.data[0] = 0x4C; // Kuehlmitteltemp (coolant * 0.75 - 48 degC)
  frame.data[1] = 0x86; // Oeltemperatur (oil temp)
  frame.data[2] = 0x85; // Aussentemperatur_gefiltert (outside-air filtered)
  frame.data[3] = 0x00; // Aussentemperatur_ungefiltert (outside-air raw)
  frame.data[4] = 0x00; // reserved
  frame.data[5] = 0x30; // status
  frame.data[6] = 0xFF; // reserved
  frame.data[7] = 0x04; // reserved / counter
  standaloneTx(frame);
}

void Gen4_frames1000()
{
  twai_message_t frame;
  // PQ Diagnose_1 (0x7D0, DLC 8) - diagnostic timestamp broadcast (vw_pq.dbc: Diagnose_1).
  frame.identifier = mDiagnose_1;
  frame.data_length_code = 8;
  frame.data[0] = 0x26;                // DI1_VerlernZaehl (relearn counter)
  frame.data[1] = 0xF2;                // DI1_km_Stand low (odometer)
  frame.data[2] = 0x03;                // DI1_km_Stand mid
  frame.data[3] = 0x12;                // DI1_km_Stand high / DI1_Jahr (year)
  frame.data[4] = 0x70;                // DI1_Monat (month) / DI1_Tag (day)
  frame.data[5] = 0x19;                // DI1_Stunde (hour)
  frame.data[6] = 0x25;                // DI1_Minute (minute)
  frame.data[7] = mDiagnose_1_counter; // DI1_Sekunde (sec) / rolling counter
  standaloneTx(frame);
  mDiagnose_1_counter++;
  if (mDiagnose_1_counter > 0x1F)
    mDiagnose_1_counter = 0;
}

void frames13(void *arg)
{
  while (1)
  {
    // Analyzer mode runs as a passive bridge; skip standalone frame generation.
    if (analyzerMode)
    {
      vTaskDelay(1 / portTICK_PERIOD_MS);
      continue;
    }
    if (isStandalone)
    {
#if detailedDebugStack
      stackframes13 = uxTaskGetStackHighWaterMark(NULL);
#endif
      switch (haldexGeneration)
      {
      case 41:
        Gen41_frames13();
        break;
      }
    }
    vTaskDelay(13 / portTICK_PERIOD_MS);
  }
}

void frames50(void *arg)
{
  while (1)
  {
    // Analyzer mode runs as a passive bridge; skip standalone frame generation.
    if (analyzerMode)
    {
      vTaskDelay(50 / portTICK_PERIOD_MS);
      continue;
    }
    if (isStandalone)
    {
#if detailedDebugStack
      stackframes50 = uxTaskGetStackHighWaterMark(NULL);
#endif
      switch (haldexGeneration)
      {
      case 41:
        Gen41_frames50();
        break;
      }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void frames250(void *arg)
{
  while (1)
  {
    // Analyzer mode runs as a passive bridge; skip standalone frame generation.
    if (analyzerMode)
    {
      vTaskDelay(250 / portTICK_PERIOD_MS);
      continue;
    }
    if (isStandalone)
    {
#if detailedDebugStack
      stackframes250 = uxTaskGetStackHighWaterMark(NULL);
#endif
      switch (haldexGeneration)
      {
      case 41:
        Gen41_frames250();
        break;
      }
    }
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}

void gen41DualBusRatesTask(void *arg)
{
  static const uint32_t GEN41_BUS0_C1_MS = 20;
  static const uint32_t GEN41_BUS0_C5_MS = 20;

  uint32_t last_c1_ms = 0;
  uint32_t last_c5_ms = 0;

  while (1)
  {
    if (analyzerMode || !isStandalone || haldexGeneration != 41)
    {
      vTaskDelay(10 / portTICK_PERIOD_MS);
      continue;
    }

    const uint32_t now_ms = millis();

    if ((now_ms - last_c1_ms) >= GEN41_BUS0_C1_MS)
    {
      extern twai_message_t gen41_bus0_cache_c1;
      extern bool gen41_bus0_cache_valid_c1;
      extern portMUX_TYPE gen41_bus0_cache_mux;
      twai_message_t tx;
      bool has_frame = false;
      taskENTER_CRITICAL(&gen41_bus0_cache_mux);
      if (gen41_bus0_cache_valid_c1)
      {
        tx = gen41_bus0_cache_c1;
        has_frame = true;
      }
      taskEXIT_CRITICAL(&gen41_bus0_cache_mux);
      if (has_frame)
      {
        twai_transmit_v2(twai_bus_0, &tx, 0);
      }
      last_c1_ms = now_ms;
    }

    if ((now_ms - last_c5_ms) >= GEN41_BUS0_C5_MS)
    {
      extern twai_message_t gen41_bus0_cache_c5;
      extern bool gen41_bus0_cache_valid_c5;
      extern portMUX_TYPE gen41_bus0_cache_mux;
      twai_message_t tx;
      bool has_frame = false;
      taskENTER_CRITICAL(&gen41_bus0_cache_mux);
      if (gen41_bus0_cache_valid_c5)
      {
        tx = gen41_bus0_cache_c5;
        has_frame = true;
      }
      taskEXIT_CRITICAL(&gen41_bus0_cache_mux);
      if (has_frame)
      {
        twai_transmit_v2(twai_bus_0, &tx, 0);
      }
      last_c5_ms = now_ms;
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

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
static uint8_t gen41_seq = 0;          // 2-bit sequence number, same L/R per packet
static uint16_t gen41_frame10_cnt = 0; // frames since power-up (10 ms bucket)

// Driven (front) wheels - ~20 km/h
static uint16_t gen41_drv_l_pulse = 0;
static uint16_t gen41_drv_r_pulse = 0;
static uint16_t gen41_drv_l_ts = 0;
static uint16_t gen41_drv_r_ts = 0;
static uint8_t gen41_drv_l_roll = 0;
static uint8_t gen41_drv_r_roll = 0;

// Non-driven (rear) wheels - ~15 km/h
static uint16_t gen41_ndrv_l_pulse = 0;
static uint16_t gen41_ndrv_r_pulse = 0;
static uint16_t gen41_ndrv_l_ts = 0;
static uint16_t gen41_ndrv_r_ts = 0;
static uint8_t gen41_ndrv_l_roll = 0;
static uint8_t gen41_ndrv_r_roll = 0;

// Alive Rolling Counts (2-bit) per message
static uint8_t gen41_throttle_arc = 0; // 0x0C9 Driver Throttle Override Detection ARC
static uint8_t gen41_brake_arc = 0;    // 0x0F1 Brake Pedal Position ARC
static uint8_t gen41_1e9_counter = 0;  // 0x1E9 4-step rolling counter: B6=(ctr<<5)

struct Gen41ReplayState
{
  uint8_t step;
  uint8_t mask;
};

static inline uint8_t gen41_take_replay_index(Gen41ReplayState &state)
{
  const uint8_t idx = static_cast<uint8_t>(state.step & state.mask);
  state.step = static_cast<uint8_t>((state.step + 1U) & state.mask);
  return idx;
}

static Gen41ReplayState gen41_130_state = {0U, 0x03U};
static Gen41ReplayState gen41_330_state = {0U, 0x07U};
static Gen41ReplayState gen41_2c3_state = {0U, 0x03U};
static Gen41ReplayState gen41_230_state = {0U, 0x07U};

// 0x230 (Bus0) — 8-state synchronized replay table covering 86% of OEM frames.
static const uint8_t gen41_230_b1[8] = {0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10};
static const uint8_t gen41_230_b2[8] = {0x00, 0x01, 0x10, 0x11, 0x20, 0x21, 0x30, 0x31};
static const uint8_t gen41_230_b3[8] = {0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00};
static const uint8_t gen41_230_b6[8] = {0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F};
static const uint8_t gen41_230_b7[8] = {0x00, 0x00, 0xFF, 0xFF, 0xFE, 0xFE, 0xFD, 0xFD};

// 0x130 (Bus1) - Multi-axis inertial sensor. Checksum = sum(D0..D5) + 0x26 (D6 excluded).
static const uint8_t gen41_130_bus1_tbl[4][8] = {
    {0x00, 0x0E, 0x10, 0x03, 0xFF, 0x20, 0x09, 0x6E},
    {0x20, 0x0F, 0x10, 0x03, 0xFF, 0x40, 0x09, 0xAF},
    {0x40, 0x10, 0x10, 0x03, 0xFF, 0xC0, 0x0A, 0x50},
    {0x60, 0x11, 0x10, 0x00, 0x00, 0x20, 0x08, 0xCF},
};

// 0x330 Bus1: two 12-bit signed acceleration-like values in D0:D1 and D2:D3.
static const uint8_t gen41_330_bus1_tbl[8][4] = {
    {0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x0F, 0xFD},
    {0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x03},
    {0x00, 0x00, 0x00, 0x00},
    {0x00, 0x03, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00},
    {0x0F, 0xFD, 0x00, 0x00},
};

static const uint16_t gen41_2c3_wheel_speed_tbl[4] = {
    0x0A8C,
    0x0A8A,
    0x0A8E,
    0x0A8C,
};

static volatile uint16_t gen41_last_wheel_speed_2c3 = 0x0A8C;

// Bus0 mirrors these from Bus1 payload generation with dedicated per-ID timing.
portMUX_TYPE gen41_bus0_cache_mux = portMUX_INITIALIZER_UNLOCKED;
twai_message_t gen41_bus0_cache_c1;
twai_message_t gen41_bus0_cache_c5;
bool gen41_bus0_cache_valid_c1 = false;
bool gen41_bus0_cache_valid_c5 = false;

// 0x1E5 PPEI Steering Wheel Angle (GMW8762, DLC=8, 10 ms cadence)
static int16_t gen41_swa_angle_raw = 1;
static uint8_t gen41_swa_arc = 0;
static uint8_t gen41_swa_phase = 0;

static constexpr int16_t kSwaRateWobble[8] = {0, +2, 0, -3, 0, +2, +4, -5};
static constexpr uint8_t kSwaAgeBus0[4] = {0x02, 0x02, 0x03, 0x01};
static constexpr uint8_t kSwaAgeBus1[4] = {0xFB, 0xFB, 0xFC, 0xFA};

static inline void gen41_swa_advance_wobble()
{
  gen41_swa_phase++;
  if ((gen41_swa_phase & 0x07U) == 0U)
  {
    if (gen41_swa_phase & 0x08U)
      gen41_swa_angle_raw++;
    else
      gen41_swa_angle_raw--;
  }
  gen41_swa_arc = static_cast<uint8_t>((gen41_swa_arc + 1U) & 0x03U);
}

static void gen41_build_1e5(twai_message_t &frame, bool bus1)
{
  const uint16_t angle_bits = static_cast<uint16_t>(gen41_swa_angle_raw);
  const int16_t rate = kSwaRateWobble[gen41_swa_phase & 0x07U];
  const uint16_t rate_bits = static_cast<uint16_t>(rate) & 0x0FFFU;

  const uint8_t status = bus1 ? 0x4FU : 0x4CU;
  const uint8_t marker = bus1 ? 0xFFU : 0x00U;
  const uint8_t epv_k = bus1 ? 0x5BU : 0x5DU;
  const uint8_t age = bus1 ? kSwaAgeBus1[gen41_swa_phase & 0x03U]
                           : kSwaAgeBus0[gen41_swa_phase & 0x03U];

  frame.identifier = 0x1E5;
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;

  frame.data[0] = status;
  frame.data[1] = static_cast<uint8_t>((angle_bits >> 8) & 0xFFU);
  frame.data[2] = static_cast<uint8_t>(angle_bits & 0xFFU);
  frame.data[3] = static_cast<uint8_t>(((gen41_swa_arc & 0x03U) << 5) | 0x10U | ((rate_bits >> 8) & 0x0FU));
  frame.data[4] = static_cast<uint8_t>(rate_bits & 0xFFU);
  frame.data[5] = marker;
  frame.data[6] = age;
  frame.data[7] = static_cast<uint8_t>(frame.data[0] + frame.data[1] + frame.data[2] + frame.data[3] + frame.data[4] + frame.data[5] + epv_k);
}

static void gen41_build_180(twai_message_t &frame)
{
  const uint16_t angle_bits = static_cast<uint16_t>(gen41_swa_angle_raw);
  const int16_t rate = kSwaRateWobble[gen41_swa_phase & 0x07U];
  const uint16_t rate_bits = static_cast<uint16_t>(rate) & 0x0FFFU;
  const uint8_t arc = gen41_swa_arc & 0x03U;
  const uint8_t d2 = static_cast<uint8_t>(angle_bits & 0xFFU);

  frame.identifier = 0x180;
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;

  frame.data[0] = 0x4EU;
  frame.data[1] = static_cast<uint8_t>((angle_bits >> 8) & 0xFFU);
  frame.data[2] = d2;
  frame.data[3] = static_cast<uint8_t>(0x80U | (arc << 5) | ((rate_bits >> 8) & 0x0FU));
  frame.data[4] = static_cast<uint8_t>(rate_bits & 0xFFU);
  frame.data[5] = 0xFFU;
  frame.data[6] = static_cast<uint8_t>((0U - d2 - arc) & 0xFFU);
  frame.data[7] = 0x00U;
}

static inline bool gen41_periodic_due(uint32_t &last_ms,
                                      uint32_t now_ms,
                                      uint32_t period_ms,
                                      uint32_t first_delay_ms)
{
  if (last_ms == 0U)
  {
    if (now_ms < first_delay_ms)
    {
      return false;
    }
    last_ms = now_ms;
    return true;
  }

  if ((now_ms - last_ms) < period_ms)
  {
    return false;
  }

  last_ms = now_ms;
  return true;
}

void gen41_step_wheel(uint16_t &pulse, uint16_t &ts, uint8_t &roll,
                      uint16_t pulse_inc, uint16_t ts_inc)
{
  pulse = (uint16_t)((pulse + pulse_inc) & 0x3FF);
  uint32_t new_ts = (uint32_t)ts + (uint32_t)ts_inc;
  if (new_ts > 0xFFFF)
  {
    roll = (uint8_t)((roll + 1) & 0x3);
  }
  ts = (uint16_t)(new_ts & 0xFFFF);
}

static inline uint8_t gen41_wheel_status_byte(uint8_t roll, uint8_t seq,
                                              uint8_t reset_occurred,
                                              uint16_t pulse)
{
  return (uint8_t)(((roll & 0x3) << 6) |
                   ((seq & 0x3) << 4) |
                   ((reset_occurred & 0x1) << 3) |
                   ((pulse >> 8) & 0x3));
}

void Gen41_frames10()
{
  twai_message_t frame;
  frame.extd = 0;
  frame.rtr = 0;

  const uint16_t spd_mod = 15;
  gen41_step_wheel(gen41_drv_l_pulse, gen41_drv_l_ts, gen41_drv_l_roll, 2, (uint16_t)(13000 + spd_mod * 100));
  gen41_step_wheel(gen41_drv_r_pulse, gen41_drv_r_ts, gen41_drv_r_roll, 2, (uint16_t)(13000 + spd_mod * 100));
  gen41_step_wheel(gen41_ndrv_l_pulse, gen41_ndrv_l_ts, gen41_ndrv_l_roll, 2, (uint16_t)(12000 + spd_mod * 100));
  gen41_step_wheel(gen41_ndrv_r_pulse, gen41_ndrv_r_ts, gen41_ndrv_r_roll, 2, (uint16_t)(12000 + spd_mod * 100));

  const uint8_t reset_occ = 0;
  const uint8_t seq = gen41_seq;

  // 0x0C1 - PPEI Driven Wheel Rotational Status (10 ms), EBCM TX
  frame.identifier = 0x0C1;
  frame.data_length_code = 8;
  frame.data[0] = gen41_wheel_status_byte(gen41_drv_l_roll, seq, reset_occ, gen41_drv_l_pulse);
  frame.data[1] = (uint8_t)(gen41_drv_l_pulse & 0xFF);
  frame.data[2] = (uint8_t)((gen41_drv_l_ts >> 8) & 0xFF);
  frame.data[3] = (uint8_t)(gen41_drv_l_ts & 0xFF);
  frame.data[4] = gen41_wheel_status_byte(gen41_drv_r_roll, seq, reset_occ, gen41_drv_r_pulse);
  frame.data[5] = (uint8_t)(gen41_drv_r_pulse & 0xFF);
  frame.data[6] = (uint8_t)((gen41_drv_r_ts >> 8) & 0xFF);
  frame.data[7] = (uint8_t)(gen41_drv_r_ts & 0xFF);
  taskENTER_CRITICAL(&gen41_bus0_cache_mux);
  gen41_bus0_cache_c1 = frame;
  gen41_bus0_cache_valid_c1 = true;
  taskEXIT_CRITICAL(&gen41_bus0_cache_mux);
  standaloneTx(frame);

  // 0x0C5 - PPEI Non Driven Wheel Rotational Status (10 ms), EBCM TX
  frame.identifier = 0x0C5;
  frame.data_length_code = 8;
  frame.data[0] = gen41_wheel_status_byte(gen41_ndrv_l_roll, seq, reset_occ, gen41_ndrv_l_pulse);
  frame.data[1] = (uint8_t)(gen41_ndrv_l_pulse & 0xFF);
  frame.data[2] = (uint8_t)((gen41_ndrv_l_ts >> 8) & 0xFF);
  frame.data[3] = (uint8_t)(gen41_ndrv_l_ts & 0xFF);
  frame.data[4] = gen41_wheel_status_byte(gen41_ndrv_r_roll, seq, reset_occ, gen41_ndrv_r_pulse);
  frame.data[5] = (uint8_t)(gen41_ndrv_r_pulse & 0xFF);
  frame.data[6] = (uint8_t)((gen41_ndrv_r_ts >> 8) & 0xFF);
  frame.data[7] = (uint8_t)(gen41_ndrv_r_ts & 0xFF);
  taskENTER_CRITICAL(&gen41_bus0_cache_mux);
  gen41_bus0_cache_c5 = frame;
  gen41_bus0_cache_valid_c5 = true;
  taskEXIT_CRITICAL(&gen41_bus0_cache_mux);
  standaloneTx(frame);

  // 0x0C9 - PPEI Engine General Status 1 (spec 12.5 ms), ECM TX
  frame.identifier = 0x0C9;
  frame.data_length_code = 8;
  frame.data[0] = 0x80;
  frame.data[1] = get_lock_target_adjusted_value(0x1F, false);
  frame.data[2] = get_lock_target_adjusted_value(0x40, false);
  {
    const uint8_t t_arc = gen41_throttle_arc & 0x3U;
    const uint8_t t_sig = 1U;
    const uint8_t t_pv = static_cast<uint8_t>((~(t_sig + t_arc) + 1U) & 0x3U);
    frame.data[3] = static_cast<uint8_t>(0x10U | (t_arc << 2U) | t_pv);
  }
  frame.data[4] = 0xFE;
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x64;
  twai_transmit_v2(twai_bus_0, &frame, 0);
  gen41_throttle_arc = (uint8_t)((gen41_throttle_arc + 1) & 0x3);

  // 0x0F1 - PPEI Brake Apply Status (10 ms), BSM TX
  frame.identifier = 0x0F1;
  frame.data_length_code = 6;
  {
    const uint8_t b_arc = gen41_brake_arc & 0x3U;
    const uint8_t b_sig = 0U;
    const uint8_t b_pv = static_cast<uint8_t>((~(b_sig + b_arc) + 1U) & 0x3U);
    frame.data[0] = static_cast<uint8_t>((b_arc << 4U) | (b_pv << 2U));
  }
  frame.data[1] = 0x00;
  frame.data[2] = 0x00;
  frame.data[3] = 0x40;
  frame.data[4] = 0x00;
  frame.data[5] = 0x00;
  twai_transmit_v2(twai_bus_0, &frame, 0);
  gen41_brake_arc = (uint8_t)((gen41_brake_arc + 1) & 0x3);

  gen41_seq = (uint8_t)((gen41_seq + 1) & 0x3);
  if (gen41_frame10_cnt < 0xFFFF)
    gen41_frame10_cnt++;

  // 0x130 (Bus1) - Multi-axis inertial sensor (~10-13 ms cadence)
  {
    frame.identifier = 0x130;
    frame.extd = 0;
    frame.rtr = 0;
    frame.data_length_code = 8;
    const uint8_t idx = gen41_take_replay_index(gen41_130_state);
    const uint8_t *r130 = gen41_130_bus1_tbl[idx];
    for (uint8_t i = 0U; i < 8U; i++)
    {
      frame.data[i] = r130[i];
    }
    standaloneTx(frame);
  }

  // 0x140 (Bus1) - Chassis Inertial Sensor (Yaw Rate), ~10 ms cadence
  {
    static uint8_t gen41_140_counter = 0U;
    const uint8_t d0 = (uint8_t)(0x80U | ((gen41_140_counter & 0x03U) << 2));
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
    gen41_140_counter = (gen41_140_counter + 1U) & 0x03U;
  }
}

void Gen41_frames20()
{
  uint8_t step = static_cast<uint8_t>(Gen41_1CE234_counter & 0x03);
  uint8_t d4_common = (step == 0) ? 0x00 : static_cast<uint8_t>(0x100 - step);

  twai_message_t frame;
  frame.extd = 0;
  frame.rtr = 0;

  // 0x1CE (Bus1) - PPEI Secondary Axle Torque Request (GMW8762, EBCM TX, 20 ms)
  {
    const uint8_t arc = static_cast<uint8_t>(step & 0x03U);
    const float req_nm = g_1ce_torque_request_nm;
    const bool requesting = (req_nm > 0.0f);
    const float scaled = req_nm / 4.0f + 0.5f;
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
  frame.data[6] = static_cast<uint8_t>(gen41_1e9_counter << 5);
  frame.data[7] = 0x00;
  twai_transmit_v2(twai_bus_0, &frame, 0);
  gen41_1e9_counter = static_cast<uint8_t>((gen41_1e9_counter + 1U) & 0x03U);

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
  twai_transmit_v2(twai_bus_0, &frame, 0);

  // 0x234 (Bus1) - Insignia-specific rolling counter frame
  frame.identifier = 0x234;
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 4;
  {
    uint8_t d1_234 = static_cast<uint8_t>(0x04 + (step << 4));
    frame.data[0] = d1_234;
    frame.data[1] = 0x00;
    frame.data[2] = static_cast<uint8_t>(step == 0 ? 0x0C : 0x0B);
    frame.data[3] = d4_common;
  }
  {
    Gen41_1CE234_counter = (Gen41_1CE234_counter + 1) & 0x03;
  }
  standaloneTx(frame);
}

void Gen41_frames25()
{
  twai_message_t frame;
  frame.extd = 0;
  frame.rtr = 0;

  // 0x2F9 - PPEI Chassis General Status 2 (spec 50 ms), EBCM TX
  {
    static uint8_t gen41_2f9_counter = 0;
    frame.identifier = 0x2F9;
    frame.data_length_code = 7;
    frame.data[0] = static_cast<uint8_t>((gen41_2f9_counter << 3) | (gen41_2f9_counter < 16U ? 0x03U : 0x00U));
    frame.data[1] = 0x01;
    frame.data[2] = 0x10;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    twai_transmit_v2(twai_bus_0, &frame, 0);
    gen41_2f9_counter = static_cast<uint8_t>((gen41_2f9_counter + 1U) & 0x1FU);
  }

  // 0x1C3 (Bus0) — GMW8762 "PPEI Engine Torque Status 2" (25ms), ECM TX
  frame.identifier = 0x1C3;
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = get_lock_target_adjusted_value(0x2E, true) + 0x1A;
  frame.data[1] = 0x51;
  frame.data[2] = 0x06;
  frame.data[3] = 0x51;
  frame.data[4] = 0x00;
  frame.data[5] = 0x00;
  frame.data[6] = 0xFE;
  frame.data[7] = tempCounter;
  twai_transmit_v2(twai_bus_0, &frame, 0);

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
  twai_transmit_v2(twai_bus_0, &frame, 0);

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
    twai_transmit_v2(twai_bus_0, &frame, 0);
  }

  // 0x330 (Bus1) - Insignia inertial/accel sensor, OEM ~25-30ms cadence
  {
    frame.identifier = 0x330;
    frame.extd = 0;
    frame.rtr = 0;
    frame.data_length_code = 8;
    const uint8_t idx = gen41_take_replay_index(gen41_330_state);
    const uint8_t *r330 = gen41_330_bus1_tbl[idx];
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
  twai_message_t frame;
  frame.extd = 0;
  frame.rtr = 0;

  // 0x230 (Bus0) — OEM 96 ms cadence. 8-state synchronized replay table.
  {
    const uint8_t idx = gen41_take_replay_index(gen41_230_state);
    frame.identifier = 0x230;
    frame.data_length_code = 8;
    frame.data[0] = 0x00;
    frame.data[1] = gen41_230_b1[idx];
    frame.data[2] = gen41_230_b2[idx];
    frame.data[3] = gen41_230_b3[idx];
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = gen41_230_b6[idx];
    frame.data[7] = gen41_230_b7[idx];
    twai_transmit_v2(twai_bus_0, &frame, 0);
  }

  // 0x348 (Bus0) — OEM 101 ms cadence. DLC=5.
  frame.identifier = 0x348;
  frame.data_length_code = 5;
  frame.data[0] = 0x00;
  frame.data[1] = 0x00;
  frame.data[2] = 0x00;
  frame.data[3] = 0x00;
  frame.data[4] = 0x1B;
  twai_transmit_v2(twai_bus_0, &frame, 0);

  // 0x34A (Bus0) — OEM 103 ms cadence. DLC=5.
  frame.identifier = 0x34A;
  frame.data_length_code = 5;
  frame.data[0] = 0x00;
  frame.data[1] = 0x00;
  frame.data[2] = 0x00;
  frame.data[3] = 0x00;
  frame.data[4] = 0x1B;
  twai_transmit_v2(twai_bus_0, &frame, 0);

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
  twai_transmit_v2(twai_bus_0, &frame, 0);

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
  twai_transmit_v2(twai_bus_0, &frame, 0);
}

void Gen41_frames200()
{
  twai_message_t frame;
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
  twai_transmit_v2(twai_bus_0, &frame, 0);

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
  twai_transmit_v2(twai_bus_0, &frame, 0);

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
  twai_transmit_v2(twai_bus_0, &frame, 0);
}

void Gen41_frames1000()
{
  twai_message_t frame;
  frame.extd = 0;
  frame.rtr = 0;
  const uint32_t now_ms = millis();

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
  twai_transmit_v2(twai_bus_0, &frame, 0);

  // 0x500 (Bus0) - GMLAN NM frame, ~2 s heartbeat
  {
    static uint32_t gen41_500_last_ms = 0U;
    if (gen41_periodic_due(gen41_500_last_ms, now_ms, 2000U, 2000U))
    {
      frame.identifier = 0x500;
      frame.extd = 0;
      frame.rtr = 0;
      frame.data_length_code = 4;
      frame.data[0] = 0x30;
      frame.data[1] = 0x30;
      frame.data[2] = 0x01;
      frame.data[3] = 0xF4;
      twai_transmit_v2(twai_bus_0, &frame, 0);
    }
  }
}

void Gen41_frames13()
{
  twai_message_t frame;
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
  twai_transmit_v2(twai_bus_0, &frame, 0);

  // 0x180 (Bus1) - GM SAS, 10 ms cadence
  gen41_build_180(frame);
  standaloneTx(frame);

  gen41_swa_advance_wobble();
}

void Gen41_frames50()
{
  twai_message_t frame;
  frame.extd = 0;
  frame.rtr = 0;

  // 0x2C3 (Bus0) — GMW8762 "PPEI Engine Torque Status 3" (50ms), ECM TX
  frame.identifier = 0x2C3;
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  const uint8_t idx_2c3 = gen41_take_replay_index(gen41_2c3_state);
  const uint16_t speed_2c3 = gen41_2c3_wheel_speed_tbl[idx_2c3];
  gen41_last_wheel_speed_2c3 = speed_2c3;
  frame.data[0] = static_cast<uint8_t>((speed_2c3 >> 8) & 0xFFU);
  frame.data[1] = static_cast<uint8_t>(speed_2c3 & 0xFFU);
  frame.data[2] = static_cast<uint8_t>((speed_2c3 >> 8) & 0xFFU);
  frame.data[3] = static_cast<uint8_t>(speed_2c3 & 0xFFU);
  frame.data[4] = 0x00;
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;
  twai_transmit_v2(twai_bus_0, &frame, 0);
}

void Gen41_frames250()
{
  twai_message_t frame;
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
  twai_transmit_v2(twai_bus_0, &frame, 0);
}

void Gen5_0CQ_frames10()
{
  twai_message_t frame;
  // MQB ESP_18 (0x135, DLC 8) - ESP minor broadcast. Fixed response, no changes.
  frame.identifier = ESP_18; // 0x135.  Fixed response, no changes
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // ESP_18_CHK / checksum (supposed to have CRC? doesn't affect)
  frame.data[1] = 0xC0; // ESP_18 BZ counter / status (always 0xC0, never changes)
  frame.data[2] = 0x00; // ESP_18 reserved (doesn't affect)
  frame.data[3] = 0x00; // ESP_18 reserved (doesn't affect)
  frame.data[4] = 0x00; // ESP_18 reserved (doesn't affect)
  frame.data[5] = 0x00; // ESP_18 reserved (doesn't affect)
  frame.data[6] = 0x00; // ESP_18 reserved (doesn't affect)
  frame.data[7] = 0x00; // ESP_18 reserved (doesn't affect)
  standaloneTx(frame);

  // MQB ESP_19 (0x0B2, DLC 8) - wheel speeds (ESP_VL/VR/HL/HR_Radgeschw_02, LE * 1/64 km/h).
  // Wheel speed MUST keep changing or haldex slowly disengages.
  frame.identifier = ESP_19; // ESP_19 0x0b2
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8; // mad, but wheel speed MUST change - static makes it reduce over time and eventually stop.  Changing it to 0x0B makes it stop immediately.  Counter from 0x20 to 0x2F keeps it going.
  // moved to a slow incrememter (1000ms)
  frame.data[0] = get_lock_target_adjusted_value(ESP_19_counter2, false);        // ESP_HL_Radgeschw_02 low (HL - wheel speed)
  frame.data[1] = get_lock_target_adjusted_value(ESP_19_counter, false);         // ESP_HL_Radgeschw_02 high (HL - wheel speed)
  frame.data[2] = get_lock_target_adjusted_value(ESP_19_counter2, false);        // ESP_HR_Radgeschw_02 low (HR - wheel speed)
  frame.data[3] = get_lock_target_adjusted_value(ESP_19_counter, false);         // ESP_HR_Radgeschw_02 high (HR - wheel speed)
  frame.data[4] = get_lock_target_adjusted_value(ESP_19_counter2 + 0xBA, false); // ESP_VL_Radgeschw_02 low (VL - wheel speed 0xCA)
  frame.data[5] = get_lock_target_adjusted_value(ESP_19_counter, false);         // ESP_VL_Radgeschw_02 high (VL - wheel speed -- affects if =0x0B)
  frame.data[6] = get_lock_target_adjusted_value(ESP_19_counter2 + 0xBA, false); // ESP_VR_Radgeschw_02 low (VR - wheel speed 0xCa)
  frame.data[7] = get_lock_target_adjusted_value(ESP_19_counter, false);         // ESP_VR_Radgeschw_02 high (VR - wheel speed -- affects if =0x0B)
  standaloneTx(frame);

  ESP_19_counter++;
  ESP_19_counter2++;
  if (ESP_19_counter > 0x10) // 0x1A
  {
    ESP_19_counter = 0x0A; // 0x10
  }
  if (ESP_19_counter2 > 0x2F) // 0x0E
  {
    ESP_19_counter2 = 0x2E; // 0x00
  }

  // MQB Getriebe_11 (0x0AD, DLC 8) - transmission ECU broadcast.
  // GE_MMom_Anf_02 (engine torque request), GE_MMom_Vorhalt_02 (anticipatory torque),
  // GE_Fahrstufe (selected gear), GE_Schalt_Sequenz (shift state), GE_Zielgang.
  frame.identifier = GETRIEBE_11; // 0x0AD
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;                // GE_CHK_02 - checksum placeholder (none affect)
  frame.data[1] = GETRIEBE_11_counter; // GE_BZ_02 - rolling 4-bit counter (0x00>0x0F)
  frame.data[2] = 0x00;                // GE_MMom_Anf_02 (engine torque-intervention request) - Was 0xFE Torque intervention at the engine. Requests a short-term reduction or increase in torque from the ECU. This signal is only valid in combination with GE_MMom_Status (for MQB) or GE_MMom_Status_02 (for MLBevo).
  frame.data[3] = 0xFE;                // GE_MMom_Vorhalt_02 (Pre-control torque - anticipatory torque request)
  frame.data[4] = 0x00;                // GE_Fahrstufe (Actual gear/range selected: 5=P, 6=R, 7=N, 8=D, 9=S, 10=E, 13/14=T)
  frame.data[5] = 0x00;                // GE_Schalt_Sequenz (Shift sequence state: 0=idle, 1=shift in progress, etc.) - does not affect (>0x00)
  frame.data[6] = 0x00;                // GE_Kraftschluss / clutch lock-up (Power transmission status / clutch lock-up state) - does not affect (>0x00)
  frame.data[7] = 0x00;                // GE_Zielgang (Target gear of current shift)

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
  frame.identifier = MOTOR_12; // Motor_12 0x0A8
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;                                                    // MO_CHK_02 - checksum placeholder
  frame.data[1] = MOTOR_12_counter;                                        // MO_BZ_02 - rolling counter (0x70>0x7F)
  frame.data[2] = 0x00;                                                    // MO_Mom_neg_verfuegbar (Negative available torque / maximum engine braking) - does not affect (doesn't affect)
  frame.data[3] = 0x00;                                                    // Static torque limit (sometimes 0xC0, sometimes 0x00) - doesn't affect
  frame.data[4] = 0x00;                                                    // Dynamic torque limit (sometimes 0x3A, sometimes 0x39) - doesn't affect
  frame.data[5] = 0x64;                                                    // Vehicle speed signal quality bit (0x64=good, 0x00=bad). True? - doesn't affect
  frame.data[6] = 0x0F;                                                    // Engine speed signal quality bit (Bool). Was 0xD4 - does affect.
  frame.data[7] = get_lock_target_adjusted_value(MOTOR_12_counter, false); // MO_Drehzahl_01 (Engine speed / RPM, was 0xAE affects.  >30 slows down.  Only adds 5%. Was 0x10 - does affect)

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
  frame.identifier = MOTOR_11; // MOTOR_11 0x0A7
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;

  if (!fixHunting)
  {
    // ---- V3 packing (default, working on 554C/D/H and 554K@100%) ----
    appliedTorque = get_lock_target_adjusted_value(0xFA, false);

    frame.data[0] = 0x00;             // MO_CHK_01 - checksum placeholder
    frame.data[1] = MOTOR_11_counter; // MO_BZ_01 - rolling counter (0x40..0x4F)
    frame.data[2] = 0xFA;             // MO_Mom_Soll_Roh
    frame.data[3] = 0xFA;             // MO_Mom_Ist
    frame.data[4] = 0x00;             // MO_Mom_Traegheit_Summe
    frame.data[5] = 0xFA;             // MO_Mom_Soll_gefiltert
    frame.data[6] = appliedTorque;    // lock-modulated (massive effect)
    frame.data[7] = appliedTorque;    // lock-modulated (massive effect)
  }
  else
  {
    // ---- BPK packing (Fix Hunting toggle on; needed for 554K @ partial lock) ----
    // DBC-correct bit-packed Soll_Roh/Ist/Solf with fixed defaults and slew limits.
    const uint8_t BPK_CEIL = 220;     // Nm at 100% lock
    const uint8_t BPK_SLEW_IST = 8;   // Nm/cycle, MO_Mom_Ist_Summe ramp
    const uint8_t BPK_SLEW_SOLF = 32; // Nm/cycle, MO_Mom_Soll_gefiltert ramp
    const uint8_t BPK_FLOOR = 10;

    uint8_t torqueNm = get_lock_target_adjusted_value(0xFE, false);
    appliedTorque = torqueNm;
    torqueNm = (uint8_t)(BPK_FLOOR +
                         ((uint16_t)torqueNm * (BPK_CEIL - BPK_FLOOR)) / 0xFE);

    // Slew limits on Ist and Soll_gefiltert.
    static uint8_t prevIstNm = 0, prevSolfNm = 0;
    auto slew = [](uint8_t cur, uint8_t target, uint8_t step) -> uint8_t
    {
      if (target > cur)
        return ((uint16_t)cur + step >= target) ? target : (uint8_t)(cur + step);
      if (target < cur)
        return (cur <= step || cur - step <= target) ? target : (uint8_t)(cur - step);
      return cur;
    };
    uint8_t istNm = slew(prevIstNm, torqueNm, BPK_SLEW_IST);
    uint8_t solfNm = slew(prevSolfNm, torqueNm, BPK_SLEW_SOLF);
    prevIstNm = istNm;
    prevSolfNm = solfNm;

    uint16_t rawSollRoh = (uint16_t)(torqueNm + 509) & 0x3FF;
    uint16_t rawIst = (uint16_t)(istNm + 509) & 0x3FF;
    uint16_t rawSolf = (uint16_t)(solfNm + 509) & 0x3FF;

    // Idle baselines for non-driven signals.
    const uint8_t TRAEG_LO = 0xFD;
    const uint8_t TRAEG_HI = 0x01;
    const uint8_t SCHUB_LO = 0x07;
    const uint8_t SCHUB_HI = 0x1E;
    const uint8_t STATUS_FL = 0x20; // Normalbetrieb=1, QBit=valid

    frame.data[0] = 0x00; // CRC placeholder
    frame.data[1] = (MOTOR_11_counter & 0x0F) | ((rawSollRoh & 0x000F) << 4);
    frame.data[2] = ((rawSollRoh >> 4) & 0x3F) | ((rawIst & 0x0003) << 6);
    frame.data[3] = (rawIst >> 2) & 0xFF;
    frame.data[4] = TRAEG_LO;
    frame.data[5] = (TRAEG_HI & 0x03) | ((rawSolf & 0x3F) << 2);
    frame.data[6] = ((rawSolf >> 6) & 0x0F) | ((SCHUB_LO & 0x0F) << 4);
    frame.data[7] = (SCHUB_HI & 0x1F) | STATUS_FL;
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
  // 100% torque = 2000 Nm. data[7] has massive effect on engagement.
  frame.identifier = ESP_14; // ESP_14 0x08A
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;           // ESP_CHK_14 - checksum placeholder
  frame.data[1] = ESP_14_counter; // ESP_BZ_14 - rolling counter (0x10>0x1F)
  frame.data[2] = 0x00;           // ESP_14 reserved/status (doesn't affect)
  frame.data[3] = 0x00;           // ESP_14 reserved/status (doesn't affect, sometimes 0xC0, sometimes 0x00)
  frame.data[4] = 0x00;           // BR_Vorg_Quer_Min (Minimum specified limit value of the clutch's operating range by the ESP MQB Haldex: 100% torque corresponds to 2000 Nm.) - doesn't affect
  frame.data[6] = 0x00;           // BR_Vorg_Allrad_Min (Minimum specified limit value of the clutch's operating range by the ESP MQB Haldex: 100% torque corresponds to 2000 Nm) - doesn't affect

  appliedTorque = get_lock_target_adjusted_value(0xFE, false);

  frame.data[5] = appliedTorque; // BR_Vorg_Quer_Max (Maximum predefined limit of the clutch's operating range by the ESP MQB Haldex: 100% torque corresponds to 2000 Nm.)
  frame.data[7] = appliedTorque; // BR_Vorg_Allrad_Max (Maximum specified limit of the clutch's operating range by the ESP MQB Haldex: 100% torque corresponds to 2000 Nm.)
  // massive effects (4>7)

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
  frame.data[0] = 0x00;           // LWI_CHK - checksum placeholder
  frame.data[1] = LWI_01_counter; // LWI_BZ - rolling counter (0x10>0x1F)
  frame.data[2] = 0x01;           // LWI_Sensorstatus (sensor health/status)
  frame.data[3] = 0x00;           // LWI_Qbit_sub_daten (sub-data quality bit)
  frame.data[4] = 0x00;           // LWI_Qbit_Lenkradwinkel (steering-angle quality bit)
  frame.data[5] = 0x00;           // LWI_Lenkradwinkel (steering-wheel angle, * 0.1 deg)
  frame.data[6] = 0x00;           // LWI_Lenkradw_Geschw low (angular velocity, * 5 deg/s)
  frame.data[7] = 0x00;           // LWI_Lenkradw_Geschw high (Unit Degrees of Arc per Second)

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
  twai_message_t frame;
  // MQB Motor_20 (0x121, DLC 8) - accelerator pedal raw/filtered + status.
  // MO_Accelerator_Raw_Value_01 (raw pedal), MO_Fahrpedal_Roh, MO_Pedal_Filt.
  frame.identifier = MOTOR_20;      // MOTOR_20 0x121
  frame.data_length_code = 8;       // DLC 8
  frame.data[0] = 0x00;             // MO_CHK_20 - checksum
  frame.data[1] = MOTOR_20_counter; // MO_BZ_20 - rolling counter (0x00>0x0F && MO_Accelerator_Raw_Value_01!)
  frame.data[2] = 0x40;             // MO_Accelerator_Raw_Value_01 (raw accelerator pedal value) (no affect MO_Accelerator_Raw_Value_01 sss)
  frame.data[3] = 0x40;             // MO_Fahrpedal_Roh (raw pedal position) (no affect sometimes 0xC0, sometimes 0x00)
  frame.data[4] = 0x19;             // MO_Pedal_Filt (filtered pedal) (no affect sometimes 0x3A, somtimes 0x39)
  frame.data[5] = 0x59;             // MO_Status_Pedal / kickdown flags (no affect)
  frame.data[6] = 0x7E;             // MO_Pedal status / driver-demand (no affect)
  frame.data[7] = 0xFE;             // MO_Pedal status (no affect)

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
  frame.identifier = ESP_10;                            // ESP_10 0x116
  frame.data_length_code = 8;                           // DLC 8
  frame.data[0] = 0x00;                                 // BR_CHK_10 - checksum
  frame.data[1] = ESP_10_counter;                       // BR_BZ_10 - rolling counter (0x00>0x0F)
  frame.data[2] = 0x01;                                 // ESP_QBit / status flags (no affect all these affect, find which one)
  frame.data[3] = 0x04;                                 // ESP_Gierrate low (yaw rate) (no effect sometimes 0xC0, sometimes 0x00)
  frame.data[4] = 0x00;                                 // ESP_Gierrate high (no effect sometimes 0x3A, somtimes 0x39)
  frame.data[5] = 0x40;                                 // ESP_VZ_Gierrate (yaw sign) (no effect)
  frame.data[6] = 0x00;                                 // ESP_Querbeschleunigung low (lateral G) (no effect)
  frame.data[7] = 0xFF;                                 // ESP_Querbeschleunigung high (lateral G - this affects(!) - a good 40%.  Was 0xFF)
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
  frame.identifier = ESP_05;                            // ESP_05 0x106
  frame.data_length_code = 8;                           // DLC 8
  frame.data[0] = 0x00;                                 // BR_CHK_05 - checksum
  frame.data[1] = ESP_05_counter;                       // BR_BZ_05 - rolling counter (0x80>0x8F)
  frame.data[2] = 0x64;                                 // BR_Bremsdruck (master-cylinder brake pressure) (no effect)
  frame.data[3] = 0xC0;                                 // BR_Bremslicht / brake-light status (this affects(!) sometimes 0xC0, sometimes 0x00)
  frame.data[4] = 0x00;                                 // BR_Status / brake event flags (no effect sometimes 0x3A, somtimes 0x39)
  frame.data[5] = 0x00;                                 // BR_Status reserved (no effect)
  frame.data[6] = 0xFD;                                 // BR_Status reserved (no effect)
  frame.data[7] = 0x00;                                 // BR_Bremspedal (brake pedal active flag) (this affects(!) - on/off.  Was 0x10.  0x00 doesn't hurt)
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
  frame.identifier = EPB_01;                            // EPB_01 0x104
  frame.data_length_code = 8;                           // DLC 8
  frame.data[0] = 0x00;                                 // EPB_CHK - checksum
  frame.data[1] = EPB_01_counter;                       // EPB_BZ - rolling counter (0x30>0x3F - none affect)
  frame.data[2] = 0xA6;                                 // EPB_Status (parking brake apply/release state)
  frame.data[3] = 0x00;                                 // EPB_Status reserved (sometimes 0xC0, sometimes 0x00)
  frame.data[4] = 0xE6;                                 // EPB_Status reserved (sometimes 0x3A, somtimes 0x39)
  frame.data[5] = 0x00;                                 // EPB_Status reserved
  frame.data[6] = 0x00;                                 // EPB_Status reserved
  frame.data[7] = 0x31;                                 // EPB_Status reserved
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
  frame.identifier = ESP_02;                            // ESP_02 0x101
  frame.data_length_code = 8;                           // DLC 8
  frame.data[0] = 0x00;                                 // BR_CHK_02 - checksum
  frame.data[1] = ESP_02_counter;                       // BR_BZ_02 - rolling counter (0x00>0x1F)
  frame.data[2] = 0x7E;                                 // BR_Bremsdruck / status (doesn't effect one of these affects, find which one - doesn't affect)
  frame.data[3] = 0x0F;                                 // BR_Status / event flags (doesn't effect sometimes 0xC0, sometimes 0x00)
  frame.data[4] = 0x82;                                 // ESP_Gierrate / yaw (doesn't effect sometimes 0x3A, somtimes 0x39)
  frame.data[5] = 0x0C;                                 // ESP_Status (doesn't effect rolling?)
  frame.data[6] = 0x40;                                 // ESP_Status reserved (doesn't efffect)
  frame.data[7] = 0x00;                                 // ESP_Status reserved (doesn't effect)
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
  frame.identifier = ESP_21;                            // ESP_21 0x0fd
  frame.data_length_code = 8;                           // DLC 8
  frame.data[0] = 0x00;                                 // BR_CHK_21 - checksum
  frame.data[1] = ESP_21_counter;                       // BR_BZ_21 - rolling counter (0x00>0x1F)
  frame.data[2] = 0x1F;                                 // ESP_21 status (in diagnosis? none affect)
  frame.data[3] = 0x80;                                 // ESP_21 status (sometimes 0xC0, sometimes 0x00)
  frame.data[4] = 0x00;                                 // ESP_21 status (sometimes 0x3A, somtimes 0x39)
  frame.data[5] = 0x00;                                 // ESP_21 reserved
  frame.data[6] = 0x00;                                 // ESP_21 reserved
  frame.data[7] = 0x00;                                 // ESP_21 reserved
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_0fd); // for 0x0fd

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
  twai_message_t frame;
  // MQB Kombi_01 (0x30B, DLC 8) - instrument cluster broadcast.
  // KBI_Kilometerstand (odometer), KBI_Geschw_Anzeige (displayed speed), warning lamps.
  frame.identifier = KOMBI_01; // kombi 1 0x30b
  frame.data_length_code = 8;  // DLC 8
  frame.data[0] = 0x10;        // KBI status / KBI_Geschw_Anzeige low (angle of turn (block 011) low byte)
  frame.data[1] = 0x20;        // KBI counter / KBI_Geschw_Anzeige high (checksum (0x20>0x2F))
  frame.data[2] = 0x02;        // KBI_Kilometerstand byte 0 (odometer)
  frame.data[3] = 0x00;        // KBI_Kilometerstand byte 1
  frame.data[4] = 0x0C;        // KBI_Kilometerstand byte 2
  frame.data[5] = 0x00;        // KBI status
  frame.data[6] = 0x00;        // KBI warning lamps
  frame.data[7] = 0x24;        // KBI status reserved
  standaloneTx(frame);
}

void Gen5_0CQ_frames100()
{
  twai_message_t frame;
  // MQB ESP_23 (0x5BE, DLC 8) - longitudinal/lateral acceleration, tyre data.
  // BR_Laengsbeschleunigung (longitudinal G), BR_Querbeschleunigung (lateral G), BR_Tire_Circumference.
  frame.identifier = ESP_23;                            // ESP_23 0x5be - this is fixed in Savvy but CHKS in Kmatrix?
  frame.data_length_code = 8;                           // DLC 8
  frame.data[0] = 0x00;                                 // BR_CHK_23 - checksum placeholder (no effect)
  frame.data[1] = ESP_23_counter;                       // BR_BZ_23 - rolling counter (no effect B high byte)
  frame.data[2] = 0xBF;                                 // BR_Laengsbeschleunigung low (longitudinal G) (no effect C)
  frame.data[3] = 0x7F;                                 // BR_Laengsbeschleunigung high (no effect D)
  frame.data[4] = 0x00;                                 // BR_Querbeschleunigung low (lateral G - rate of change (block 010))
  frame.data[5] = 0x00;                                 // BR_Querbeschleunigung high (rate of change (block 010))
  frame.data[6] = 0x7C;                                 // BR_Tire_Circumference / status (rate of change (block 010))
  frame.data[7] = 0x78;                                 // BR_Tire_Circumference / status (rate of change (block 010))
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
  frame.data[0] = 0x00;            // PH_Status low (angle of turn (block 011) low byte)
  frame.data[1] = 0x00;            // PH_Status high (no effect B high byte)
  frame.data[2] = 0x00;            // PH_Status reserved (no effect C)
  frame.data[3] = 0x00;            // PH_Status reserved (no effect D)
  frame.data[4] = 0x00;            // PH_Status reserved (rate of change (block 010))
  frame.data[5] = 0x00;            // PH_Status reserved (rate of change (block 010))
  frame.data[6] = 0x00;            // PH_Status reserved (rate of change (block 010))
  frame.data[7] = 0x24;            // PH_Status reserved (rate of change (block 010))
  standaloneTx(frame);

  // MQB Gateway_72 (0x3DB, DLC 8) - gateway routing/diagnostic broadcast.
  frame.identifier = GATEWAY_72; // gateway 72 0x3db
  frame.data_length_code = 8;    // DLC 8
  frame.data[0] = 0x50;          // GW_Status / routing flags
  frame.data[1] = 0x80;          // GW_Status
  frame.data[2] = 0x00;          // GW_Status reserved
  frame.data[3] = 0x00;          // GW_Status reserved
  frame.data[4] = 0x05;          // GW_Status reserved
  frame.data[5] = 0x10;          // GW_Status reserved
  frame.data[6] = 0x01;          // GW_Status reserved
  frame.data[7] = 0x78;          // GW_Status reserved
  standaloneTx(frame);

  // MQB Getriebe_14 (0x3C8, DLC 8) - transmission slow broadcast (Charisma, drag torque, launch).
  frame.identifier = GETRIEBE_14; // getriebe 14 0x3c8
  frame.data_length_code = 8;     // DLC 8
  frame.data[0] = 0x00;           // GE_Beschl_max (Maximum possible acceleration - limited by gear/clutch)
  frame.data[1] = 0x00;           // GE_Charisma_Fahrprogramm (Charisma drive programme selected - affects shift mapping)
  frame.data[2] = 0x54;           // GE_Charisma_Status (Charisma system status)
  frame.data[3] = 0x24;           // GE_M_Verlust (Drag/friction loss torque in transmission)
  frame.data[4] = 0x00;           // GE_Launch_Control (Launch control active)
  frame.data[5] = 0x60;           // GE_Status reserved
  frame.data[6] = 0x01;           // GE_Status reserved
  frame.data[7] = 0x51;           // GE_Status reserved
  standaloneTx(frame);

  // MQB Motor_14 (0x3BE, DLC 8) - start/stop subsystem state broadcast.
  // MO_StSt_Status (state machine), MO_StSt_Restart (restart event), MO_StSt_Stop (stop event).
  frame.identifier = MOTOR_14;                          // motor 14 0x3be
  frame.data_length_code = 8;                           // DLC 8
  frame.data[0] = 0x00;                                 // MO_CHK_14 - checksum
  frame.data[1] = MOTOR_14_counter;                     // MO_BZ_14 - rolling counter (0x10 to 0x1F)
  frame.data[2] = 0xE6;                                 // MO_StSt_Status (Start/stop system state - 0=inactive, 1=stopping, 2=stopped, 3=restarting) (doesn't effect)
  frame.data[3] = 0x01;                                 // MO_StSt_Restart (Restart event flag) (this affects(!) on/off)
  frame.data[4] = 0xC8;                                 // MO_StSt_Stop (Engine stop event flag) (doesn't effect)
  frame.data[5] = 0x80;                                 // MO_StSt status reserved (doesn't effect)
  frame.data[6] = 0x00;                                 // MO_StSt status reserved (doesn't effect)
  frame.data[7] = 0x80;                                 // MO_StSt status reserved (doesn't effect)
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
  frame.data[0] = 0x00;                                 // BR_CHK_07 - checksum
  frame.data[1] = ESP_07_counter;                       // BR_BZ_07 - rolling counter (0x20>0x2F)
  frame.data[2] = 0x00;                                 // ESP_07 status (one of these affects, find which one)
  frame.data[3] = 0x00;                                 // ESP_07 status (no effect)
  frame.data[4] = 0x00;                                 // ESP_07 status (no effect)
  frame.data[5] = 0x00;                                 // ESP_07 status (no efefct)
  frame.data[6] = 0x00;                                 // ESP_07 status (no effect)
  frame.data[7] = 0x00;                                 // ESP_07 status (no effect)
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_392); // for 0x392

  ESP_07_counter++;
  if (ESP_07_counter > 0x1F)
  {
    ESP_07_counter = 0x00;
  }
  standaloneTx(frame);
  /*
   0x91,0x91,0x91,0x91,0x91,0x91,0x91,0x91,0x91,0x91,0x91,0x91,0x91,0x91
   */

  // MQB ESP_29 (0x18C, DLC 8) - ESP slow-rate diagnostic broadcast.
  frame.identifier = ESP_29;  // esp_29 0x18c
  frame.data_length_code = 8; // DLC 8
  frame.data[0] = 0x00;       // BR_CHK_29 - checksum
  frame.data[1] = 0x20;       // BR_BZ_29 - counter (checksum (0x20>0x2F)?  Not in Savvy)
  frame.data[2] = 0x59;       // ESP_29 status
  frame.data[3] = 0x00;       // ESP_29 status
  frame.data[4] = 0x00;       // ESP_29 status
  frame.data[5] = 0x00;       // ESP_29 status
  frame.data[6] = 0x00;       // ESP_29 status
  frame.data[7] = 0x00;       // ESP_29 status
  standaloneTx(frame);
}

void Gen5_0CQ_frames200()
{
  twai_message_t frame;
  // PQ mKombi_2 (0x0C2, DLC 8) - electronic power steering / instrument-cluster slow.
  // Transmit currently disabled (commented out below).
  frame.identifier = mKombi_2; // electronic power steering 0x0C2
  frame.data_length_code = 8;  // DLC 8
  frame.data[0] = 0x4C;        // EPS status / steering torque (angle of turn (block 011) low byte)
  frame.data[1] = 0x86;        // EPS status (no effect B high byte)
  frame.data[2] = 0x85;        // EPS status (no effect C)
  frame.data[3] = 0x00;        // EPS status (no effect D)
  frame.data[4] = 0x00;        // EPS reserved (rate of change (block 010))
  frame.data[5] = 0x30;        // EPS reserved (rate of change (block 010))
  frame.data[6] = 0xFF;        // EPS reserved (rate of change (block 010))
  frame.data[7] = 0x04;        // EPS reserved (rate of change (block 010))
  // standaloneTx(frame);
}

void Gen5_0CQ_frames1000()
{
  twai_message_t frame;
  // MQB Motor_07 (0x640, DLC 8) - engine slow-rate diagnostics broadcast.
  frame.identifier = MOTOR_07; // motor 07, 1000ms
  frame.data_length_code = 8;  // DLC 8
  frame.data[0] = 0xA0;        // MO_07 diagnostic byte (no effect from any)
  frame.data[1] = 0x5A;        // MO_07 diagnostic byte
  frame.data[2] = 0x56;        // MO_07 diagnostic byte
  frame.data[3] = 0xA3;        // MO_07 diagnostic byte
  frame.data[4] = 0x80;        // MO_07 diagnostic byte
  frame.data[5] = 0xA0;        // MO_07 diagnostic byte
  frame.data[6] = 0x59;        // MO_07 diagnostic byte
  frame.data[7] = 0x01;        // MO_07 diagnostic byte
  standaloneTx(frame);

  frame.identifier = CHARISMA_01; // charisma_01 0x385
  frame.data_length_code = 8;     // DLC 8 no effect from any
  frame.data[0] = 0x00;           // CHA_Target_Driving_Program_AGA & CHA_Target_Driving_Prior_ESP
  frame.data[1] = 0x00;           // CHA_Target_Driving_Pri_Freewheel & void
  frame.data[2] = 0x22;           // CHA_Target_Driving_Program_MO & CHA_Target_Driving_Program_GE
  frame.data[3] = 0x02;           // CHA_Target_Driving_PR_ALR (inc. AWD) & CHA_Target_Driving_Program_MO_BZS
  frame.data[4] = 0x02;           // CHA_Target_Driving_Project_DR & CHA_Target_Driving_Prior_VAQ
  frame.data[5] = 0x20;           // CHA_Target_Driving_PR_AFS & CHA_Target_Driving_Program_RGS
  frame.data[6] = 0x02;           // CHA_Target_Driving_Price_EPS & CHA_Target_Driving_Principal_ACC
  frame.data[7] = 0x02;           // CHA_Target_Driving_Prior_SAK & CHA_Target_Driving_Program_MO_StSt
  standaloneTx(frame);

  // MQB Systeminfo_01 (0x585, DLC 8) - system identification/info broadcast.
  frame.identifier = SYSTEMINFO_01; // systeminfo_01 0x585
  frame.data_length_code = 8;       // DLC 8
  frame.data[0] = 0x84;             // SI status / system info
  frame.data[1] = 0x3C;             // SI info
  frame.data[2] = 0x00;             // SI info
  frame.data[3] = 0x7F;             // SI info
  frame.data[4] = 0x14;             // SI info
  frame.data[5] = 0x00;             // SI info
  frame.data[6] = 0x00;             // SI info
  frame.data[7] = 0x00;             // SI info
  standaloneTx(frame);

  frame.identifier = MOTOR_CODE_01;      // motor_code_01 0x641
  frame.data_length_code = 8;            // DLC 8 no affect from any
  frame.data[0] = 0x00;                  // checksum
  frame.data[1] = MOTOR_CODE_01_counter; // rolling (10>1F)
  // MQB Motor_Code_01 (0x641, DLC 8) - engine code / identification broadcast.
  frame.identifier = MOTOR_CODE_01;      // motor_code_01 0x641
  frame.data_length_code = 8;            // DLC 8
  frame.data[0] = 0x00;                  // MO_CHK_Code - checksum placeholder
  frame.data[1] = MOTOR_CODE_01_counter; // MO_BZ_Code - rolling counter (10>1F)
  frame.data[2] = 0x2B;                  // MO_Code byte (engine identification ASCII/code)
  frame.data[3] = 0x53;                  // MO_Code byte
  frame.data[4] = 0x14;                  // MO_Code byte
  frame.data[5] = 0x14;                  // MO_Code byte
  frame.data[6] = 0xD7;                  // MO_Code byte
  frame.data[7] = 0x24;                  // MO_Code byte

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
  frame.identifier = ESP_20;                            // esp_20 0x65d
  frame.data_length_code = 8;                           // DLC 8 no affect from any
  frame.data[0] = 0x00;                                 // BR_CHK_20 - checksum
  frame.data[1] = ESP_20_counter;                       // BR_BZ_20 - rolling counter (rolling (30>3F))
  frame.data[2] = 0x2B;                                 // ESP_20 status (no effect C)
  frame.data[3] = 0x10;                                 // ESP_20 status (no effect D)
  frame.data[4] = 0x00;                                 // ESP_20 status
  frame.data[5] = 0x00;                                 // ESP_20 status
  frame.data[6] = 0xE2;                                 // ESP_20 status
  frame.data[7] = 0x79;                                 // BR_Tire_Circumference (BR_Tire circumference)
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
  frame.identifier = DIAGNOSE_01; // diagnose_01 0x6b2
  frame.data_length_code = 8;     // DLC 8
  frame.data[0] = 0x30;           // DG_Status
  frame.data[1] = 0x4D;           // DG_Status
  frame.data[2] = 0x58;           // DG_Status
  frame.data[3] = 0xA2;           // DG_Status
  frame.data[4] = 0x89;           // DG_Status
  frame.data[5] = 0x85;           // DG_Status
  frame.data[6] = 0x3F;           // DG_Status (0x3F OR 0xBF? (3F, then BF, then 3F, then BF...))
  frame.data[7] = 0x30;           // DG_Status (2D, then 2D, then 2E, then 2E, then 2F, then 2F... roll over? When?)
  standaloneTx(frame);

  // MQB Kombi_02 (0x6B7, DLC 8) - instrument cluster slow-rate broadcast.
  frame.identifier = KOMBI_02; // kombi2 0x6b7
  frame.data_length_code = 8;  // DLC 8
  frame.data[0] = 0x4D;        // KBI_02 status (no effect from any)
  frame.data[1] = 0x58;        // KBI_02 status
  frame.data[2] = 0xF2;        // KBI_02 status
  frame.data[3] = 0xEE;        // KBI_02 status
  frame.data[4] = 0x04;        // KBI_02 status
  frame.data[5] = 0x2B;        // KBI_02 status
  frame.data[6] = 0x00;        // KBI_02 status
  frame.data[7] = 0x78;        // KBI_02 status
  standaloneTx(frame);
}

// =============================================================================
// Gen5 (0AY) Standalone Frames
// 0AY is a different MQB-era controller variant; the frames below are an
// initial copy of the Gen4 (PQ-derived) standalone set
// =============================================================================
void Gen5_0AY_frames10()
{
  twai_message_t frame;
  // PQ LW_1 / mLW_1 (0x0C2, DLC 8) - steering-angle replay (vw_pq.dbc: LW_1).
  frame.identifier = mLW_1;
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = lws_2[mLW_1_counter][0]; // LW1_LRW low (steering-wheel angle * 0.04375 deg)
  frame.data[1] = lws_2[mLW_1_counter][1]; // LW1_LRW high
  frame.data[2] = lws_2[mLW_1_counter][2]; // LW1_LRW_Sign / status
  frame.data[3] = lws_2[mLW_1_counter][3]; // LW1_Lenk_Gesch low (angular velocity)
  frame.data[4] = lws_2[mLW_1_counter][4]; // LW1_Lenk_Gesch high / Gesch_Sign
  frame.data[5] = lws_2[mLW_1_counter][5]; // LW1_Status / counter
  frame.data[6] = lws_2[mLW_1_counter][6]; // LW1_ID / Initquelle
  frame.data[7] = lws_2[mLW_1_counter][7]; // CRC (replayed verbatim)

  mLW_1_counter++;
  if (mLW_1_counter > 15)
    mLW_1_counter = 0;
  standaloneTx(frame);

  // PQ Bremse_1 (0x1A0, DLC 8) - ABS/ESP main broadcast (vw_pq.dbc: Bremse_1).
  frame.identifier = BRAKES1_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x20;            // BR1_ASR_Anf / MSR_Anf flags
  frame.data[1] = 0x40;            // ABS_Brems / EDS_Ongr / ESP_Ongr intervention bits
  frame.data[2] = 0xF0;            // Lampe_ABS / Lampe_ASR
  frame.data[3] = 0x07;            // status / ABS active flags
  frame.data[4] = 0xFE;            // BR1_Rad_kmh low (vehicle speed)
  frame.data[5] = 0xFE;            // BR1_Rad_kmh high
  frame.data[6] = 0x00;            // reserved
  frame.data[7] = BRAKES1_counter; // BR1_BZ - rolling 5-bit counter (10..0x1F)
  if (++BRAKES1_counter > 0x1F)
    BRAKES1_counter = 10;
  standaloneTx(frame);

  // PQ Bremse_3 (0x4A0, DLC 8) - per-wheel speeds (vw_pq.dbc: Bremse_3).
  frame.identifier = BRAKES3_ID;
  frame.data_length_code = 8;
  frame.data[0] = get_lock_target_adjusted_value(0xB6, false); // Radgeschw_VL low (front-left)
  frame.data[1] = 0x07;                                        // Radgeschw_VL high
  frame.data[2] = get_lock_target_adjusted_value(0xCC, false); // Radgeschw_VR low (front-right)
  frame.data[3] = 0x07;                                        // Radgeschw_VR high
  frame.data[4] = get_lock_target_adjusted_value(0xD2, false); // Radgeschw_HL low (rear-left)
  frame.data[5] = 0x07;                                        // Radgeschw_HL high
  frame.data[6] = get_lock_target_adjusted_value(0xD2, false); // Radgeschw_HR low (rear-right)
  frame.data[7] = 0x07;                                        // Radgeschw_HR high
  standaloneTx(frame);

  // PQ Bremse_4 (0x2A0, DLC 8) - ABS coupling moment (vw_pq.dbc: Bremse_4).
  // data[7] = XOR-CRC over data[0..6]; data[6] is rolling counter (steps of 16).
  frame.identifier = BRAKES4_ID;
  frame.data_length_code = 8;
  frame.data[0] = get_lock_target_adjusted_value(0x7F, false) - 0x7F; // ABS_Vorgabewert_hinten_Kupplung (rear-clutch %)
  frame.data[1] = 0x00;                                        // ABS_Vorgabewert_mitte_Kupplungs low (centre stiffness Nm/min)
  frame.data[2] = 0x00;                                        // ABS_Vorgabewert_mitte_Kupplungs high
  frame.data[3] = 0x64;                                        // status / reserved
  frame.data[4] = 0x00;                                        // reserved
  frame.data[5] = 0x00;                                        // reserved
  frame.data[6] = BRAKES4_counter;                             // rolling counter (16-step)
  BRAKES4_crc = 0;
  for (uint8_t i = 0; i < 7; i++)
  {
    BRAKES4_crc ^= frame.data[i];
  }
  frame.data[7] = BRAKES4_crc; // XOR CRC over data[0..6]
  BRAKES4_counter = BRAKES4_counter + 16;
  if (BRAKES4_counter > 0xF0)
    BRAKES4_counter = 0x00;
  standaloneTx(frame);

  // PQ Motor_1 (0x280, DLC 8) - engine ECU broadcast (vw_pq.dbc: Motor_1).
  // Every byte except data[0] is lock-target-adjusted to bias the haldex.
  frame.identifier = MOTOR1_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;                                        // Fahrerwunschmoment (driver-requested torque)
  frame.data[1] = get_lock_target_adjusted_value(0xFE, false); // Fahrerwunschmoment (driver-requested torque)
  frame.data[2] = get_lock_target_adjusted_value(0x20, false); // Motordrehzahl low (engine RPM * 0.25)
  frame.data[3] = get_lock_target_adjusted_value(0x4E, false); // Motordrehzahl high
  frame.data[4] = get_lock_target_adjusted_value(0xFE, false); // Fahrpedalwert (accelerator pedal %)
  frame.data[5] = get_lock_target_adjusted_value(0xFE, false); // inneres_Motor_Moment_ohne_extern
  frame.data[6] = get_lock_target_adjusted_value(0x16, false); // mechanisches_Motor_Verlustmoment
  frame.data[7] = get_lock_target_adjusted_value(0xFE, false); // inneres_Motor_Moment (actual)
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
  frame.data[0] = (mGetriebe_2_counter << 4) & 0xF0; // status flags = 0, counter in high nibble
  frame.data[1] = 0x50;                              // Leerlaufsolldrehzahl ~800 rpm (0x50 * 10)
  frame.data[2] = 0xFF;                              // Gradientenbegrenzung neutral (init/no limit)
  frame.data[3] = 0x00;                              // Synchronisations_Wunschdrehzahl = 0 (no sync)
  frame.data[4] = 0xFF;                              // invertierte Synchronisations_Wu (init)
  frame.data[5] = 0x00;                              // Synchronisationszeit = 0
  frame.data[6] = 0x00;                              // all status flags off
  frame.data[7] = 0xFF;                              // gear display = ---, Fahrstufe = N
  mGetriebe_2_counter = (mGetriebe_2_counter + 1) & 0x0F;
  standaloneTx(frame);

  // PQ Bremse_5 (0x4A8, DLC 8) - ESP brake-event broadcast.
  // vw_pq.dbc: BR5_Giergeschw / BR5_Gierrate (yaw rate * 0.01 deg/s),
  //            BR5_Bremsdruck (brake pressure * 0.1 bar), BR5_Bremslicht,
  //            BR5_Notbremsung (emergency-brake), BR5_HDC_bereit (hill-descent-ready).
  frame.identifier = BRAKES5_ID;    // 0x2A0 includes coupling moment? Affects vehicle mode(!) do not have!
  frame.data_length_code = 8;       // DLC 8 (/4)
  frame.data[0] = 0xFE;             // affects estimated torque AND vehicle mode(!) - Giergeschw low (yaw rate)
  frame.data[1] = 0x7F;             // Giergeschw high
  frame.data[2] = 0x03;             // BR5_Bremsdruck (brake pressure)
  frame.data[3] = 0x00;             // 32605 - status / Bremslicht / Notbremsung bits
  frame.data[4] = 0x00;             // BR5_Gierrate low (signed yaw rate)
  frame.data[5] = 0x00;             // BR5_Gierrate high
  frame.data[6] = BRAKES5_counter;  // checksum / rolling counter
  frame.data[7] = BRAKES5_counter2; // checksum / second rolling counter
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
  // data[0] rolls 0x80..0x8F, data[1] rolls 0x00..0x0F (independent counters).
  frame.identifier = BRAKES8_ID;
  frame.data_length_code = 8;
  frame.data[0] = BRAKES8_counter;  // byte 1: 0x80..0x8F
  frame.data[1] = BRAKES8_counter1; // byte 2: 0x00..0x0F
  frame.data[2] = 0x00;             // byte 3
  frame.data[3] = 0x00;             // byte 4
  frame.data[4] = 0x69;             // byte 5
  frame.data[5] = 0x21;             // byte 6
  frame.data[6] = 0x00;             // byte 7
  frame.data[7] = 0xC1;             // byte 8
  standaloneTx(frame);
  if (++BRAKES8_counter > 0x8F)
    BRAKES8_counter = 0x80;
  if (++BRAKES8_counter1 > 0x0F)
    BRAKES8_counter1 = 0x00;
}

void Gen5_0AY_frames20()
{
  twai_message_t frame;
  // PQ Bremse_2 (0x5A0, DLC 8) - ESP/ABS sensor broadcast (vw_pq.dbc: Bremse_2).
  frame.identifier = BRAKES2_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x80;            // Impulszahl (pulse count)
  frame.data[1] = 0x7A;            // Wegimpulse_Vorderachse (front-axle path pulses)
  frame.data[2] = 0x05;            // mittlere_Raddrehzahl (mean wheel speed)
  frame.data[3] = BRAKES2_counter; // rolling counter (16-step)
  frame.data[4] = 0x7F;            // Querbeschleunigung (lateral G) was 0x7f with lock calc
  frame.data[5] = 0xCA;            // Zeitstempel low (timestamp)
  frame.data[6] = 0x1B;            // Zeitstempel high
  frame.data[7] = 0xAB;            // status / reserved
  standaloneTx(frame);
  BRAKES2_counter = BRAKES2_counter + 16;
  if (BRAKES2_counter > 0xF0)
  {
    BRAKES2_counter = 0;
  }

  // PQ Motor_2 (0x288, DLC 8) - secondary engine broadcast (vw_pq.dbc: Motor_2).
  frame.identifier = MOTOR2_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // MO2_BLS / MO2_BTS / MO2_Sta_Klima (brake-light, brake-test, A/C)
  frame.data[1] = 0x30; // MO2_Kuehlm_T (coolant temp * 0.75 - 48 degC)
  frame.data[2] = 0x00; // MO2_Sta_Kuehlm / MO2_Sta_No_Bet (status flags)
  frame.data[3] = 0x0A; // MO2_GRA_Soll low (cruise-control target speed)
  frame.data[4] = 0x0A; // MO2_GRA_Soll high
  frame.data[5] = 0x10; // GRA / cruise status bits
  frame.data[6] = 0xFE; // reserved
  frame.data[7] = 0xFE; // reserved
  standaloneTx(frame);

  // PQ Motor_5 (0x480, DLC 8) - tertiary engine broadcast (vw_pq.dbc: Motor_5, multiplexed).
  // NOTE: the if(++BRAKES1_counter > 255) below increments BRAKES1_counter (not MOTOR5_counter)
  // - looks like a copy-paste bug, left as-is per "comment-only" pass.
  frame.identifier = MOTOR5_ID;   // 0x1A0
  frame.data_length_code = 8;     // DLC 8
  frame.data[0] = 0xFE;           // ASR 0x04 sets bit 4.  0x08 removes set.  Coupling open/closed - MO5_Mp_Code mux
  frame.data[1] = 0x00;           // can use to disable (>130 dec).  Was 0x00; 0x41?  0x43? - MO5_max_Moment / Drehzahl (mux)
  frame.data[2] = 0x00;           // was 0x00 no effect - MO5_Vorgluehen / E_Gas / OBD_2 lamps
  frame.data[3] = 0x00;           // was 0xFE no effect - MO5_Luefter (cooling-fan PWM)
  frame.data[4] = 0x00;           // was 0xFE miasrl no effect - reserved
  frame.data[5] = 0x00;           // was 0xFE miasrs no effect - reserved
  frame.data[6] = 0x00;           // was 0x00 - reserved
  frame.data[7] = MOTOR5_counter; // checksum / rolling counter
  if (++BRAKES1_counter > 255)
  {                      // 0xF (NOTE: increments BRAKES1_counter - copy-paste artefact)
    BRAKES1_counter = 0; // 0
  }
  standaloneTx(frame);
}

void Gen5_0AY_frames25()
{
  twai_message_t frame;
  // PQ Kombi_1 (0x320, DLC 8) - instrument-cluster broadcast (vw_pq.dbc: Kombi_1).
  frame.identifier = mKombi_1;
  frame.data_length_code = 8;
  frame.data[0] = 0x24; // Bremsinfo / Oeldruck status bits
  frame.data[1] = 0x00; // Angezeigte_Geschwindigkeit (displayed speed)
  frame.data[2] = 0x1D; // Geschwindigkeit_Kombi_1 low (vehicle speed)
  frame.data[3] = 0xB9; // Geschwindigkeit_Kombi_1 high
  frame.data[4] = 0x07; // Tankinhalt (fuel level)
  frame.data[5] = 0x42; // Dynamische_Oeldruckwarnung / status
  frame.data[6] = 0x09; // reserved / multiplex
  frame.data[7] = 0x81; // reserved
  standaloneTx(frame);

  // PQ Kombi_3 (0x520, DLC 8) - cluster odometer/keys (vw_pq.dbc: Kombi_3).
  frame.identifier = mKombi_3;
  frame.data_length_code = 8;
  frame.data[0] = 0x60; // Kilometerstand low (odometer, km)
  frame.data[1] = 0x43; // Kilometerstand mid
  frame.data[2] = 0x01; // Kilometerstand high
  frame.data[3] = 0x10; // Standzeit low (stand time, sec)
  frame.data[4] = 0x66; // Standzeit high
  frame.data[5] = 0xF1; // Schluesselinfo (key info)
  frame.data[6] = 0x03; // Kombi_Multiplex_Code
  frame.data[7] = 0x02; // Kombi_Multiplex_Generation
  standaloneTx(frame);
}

void Gen5_0AY_frames100()
{
  twai_message_t frame;
  // PQ Gate_Komf_1 (0x390, DLC 8) - gateway-comfort broadcast (vw_pq.dbc: Gate_Komf_1).
  frame.identifier = mGate_Komf_1;
  frame.data_length_code = 8;
  frame.data[0] = 0x03; // GK1_Sta_Anhaen / door & light status bits
  frame.data[1] = 0x11; // GK1_Blinker_li / re (indicators)
  frame.data[2] = 0x58; // Rueckfahrscheinwerfer (reverse light) / alarm bits
  frame.data[3] = 0x00; // GK1_Sta_RDK_Warn (TPMS warning)
  frame.data[4] = 0x40; // seat-memory / sunroof bits
  frame.data[5] = 0x00; // alarm / lock bits
  frame.data[6] = 0x01; // status
  frame.data[7] = 0x08; // status / counter
  standaloneTx(frame);

  // PQ Bremse_11 (0x5B7, DLC 8) - extended brake frame (NOT in vw_pq.dbc).
  // Static placeholder; only data[1]=0xC0 is significant for haldex sanity.
  frame.identifier = BRAKES11_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // no effect
  frame.data[1] = 0XC0; // mode/status (only byte that matters)
  frame.data[2] = 0x00; // no effect
  frame.data[3] = 0x00; // no effect
  frame.data[4] = 0x00; // no effect
  frame.data[5] = 0x00; // no effect
  frame.data[6] = 0x00; // no effect
  frame.data[7] = 0x00; // no effect
  standaloneTx(frame);

  // ---------------------------------------------------------------------------
  // PQ Systeminfo_1 / mSysteminfo_1 (0x5D0, DLC 8) - gateway vehicle-identity
  // broadcast.  Source: PQ35_46 ACAN KMatrix V5.20.6F, cycle 100 ms, TO 1000 ms.
  // Required by haldex DTC 16659 (Sys_01 / Systeminfo_1 timeout) on 0AY.
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
  frame.data[0] = 0x00; // status flags = defaults
  frame.data[1] = 0x24; // Fzg_Klasse=4 (compact), Fzg_Marke=2 (Audi)
  frame.data[2] = 0x35; // Fzg_Derivat=5, Fzg_Gen=3 (PQ35)
  frame.data[3] = 0x0F; // Fzg_Index = 0xF (init), other status = 0
  frame.data[4] = 0x39; // Komf_NV=9, Komf_HV=3 (KMatrix init values)
  frame.data[5] = 0x59; // Ant_NV=9,  Ant_HV=5 (KMatrix init values)
  frame.data[6] = 0x00; // FAS/ELV/QRS/NWDF flags = 0
  frame.data[7] = 0x00; // Notbrems_Status etc = 0
  standaloneTx(frame);
}

void Gen5_0AY_frames200()
{
  twai_message_t frame;
  // PQ Kombi_2 (0x420, DLC 8) - cluster temps (vw_pq.dbc: Kombi_2).
  frame.identifier = mKombi_2;
  frame.data_length_code = 8;
  frame.data[0] = 0x4C; // Kuehlmitteltemp (coolant * 0.75 - 48 degC)
  frame.data[1] = 0x86; // Oeltemperatur (oil temp)
  frame.data[2] = 0x85; // Aussentemperatur_gefiltert (outside-air filtered)
  frame.data[3] = 0x00; // Aussentemperatur_ungefiltert (outside-air raw)
  frame.data[4] = 0x00; // reserved
  frame.data[5] = 0x30; // status
  frame.data[6] = 0xFF; // reserved
  frame.data[7] = 0x04; // reserved / counter
  standaloneTx(frame);
}

void Gen5_0AY_frames1000()
{
  twai_message_t frame;
  // PQ Diagnose_1 (0x7D0, DLC 8) - diagnostic timestamp broadcast (vw_pq.dbc: Diagnose_1).
  frame.identifier = mDiagnose_1;
  frame.data_length_code = 8;
  frame.data[0] = 0x26;                // DI1_VerlernZaehl (relearn counter)
  frame.data[1] = 0xF2;                // DI1_km_Stand low (odometer)
  frame.data[2] = 0x03;                // DI1_km_Stand mid
  frame.data[3] = 0x12;                // DI1_km_Stand high / DI1_Jahr (year)
  frame.data[4] = 0x70;                // DI1_Monat (month) / DI1_Tag (day)
  frame.data[5] = 0x19;                // DI1_Stunde (hour)
  frame.data[6] = 0x25;                // DI1_Minute (minute)
  frame.data[7] = mDiagnose_1_counter; // DI1_Sekunde (sec) / rolling counter
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
// Bus usage: all frames transmitted on twai_bus_1 (vehicle PT-CAN → Haldex).
// Haldex responds with FORD_GEN42_AWD_STATUS_ID (0x260) on twai_bus_0:
//   AwdLck_Tq_Actl D0:D1 b15|12 = actual clutch torque (1 Nm/LSB).
//   [Information4x4_CG1] — parsed later in parseCAN_chs (deferred).
// =============================================================================

void Gen42_frames10()
{
  twai_message_t frame;
  frame.extd = 0;
  frame.rtr  = 0;

  // FORD_GEN42_WHEEL_SPEEDS_ID 0x4B0 (Bus1, 10 ms) — ABS/ESC wheel speeds.
  // FL / FR / RL / RR, each 16-bit big-endian. Encoding: (raw − 10 000) / 100 km/h.
  // Raw=0x2710 (10 000) = 0 km/h standing reference.
  // Inflate front wheels above rear to request proportional AWD engagement.
  // lock_target is refreshed every 100 ms by the frames100 dispatcher.
  // At 0 % lock: all four wheels at 0x2710 (balanced, no engagement requested).
  // At 100 % lock: front +~1 000 counts (~10 km/h simulated slip). [WHEEL_SPEEDS]
  {
    const uint8_t  spd_adj     = get_lock_target_adjusted_value(0xFA, false);
    const uint16_t front_delta = (uint16_t)spd_adj * 4U; // 0 to ~1 000 counts
    const uint16_t front_spd   = 0x2710U + front_delta;  // FL / FR speed word
    const uint16_t rear_spd    = 0x2710U;                 // RL / RR at reference
    frame.identifier       = FORD_GEN42_WHEEL_SPEEDS_ID;
    frame.data_length_code = 8;
    frame.data[0] = (uint8_t)(front_spd >> 8);    // FL high byte
    frame.data[1] = (uint8_t)(front_spd & 0xFFU); // FL low byte
    frame.data[2] = (uint8_t)(front_spd >> 8);    // FR high byte
    frame.data[3] = (uint8_t)(front_spd & 0xFFU); // FR low byte
    frame.data[4] = (uint8_t)(rear_spd  >> 8);    // RL high byte
    frame.data[5] = (uint8_t)(rear_spd  & 0xFFU); // RL low byte
    frame.data[6] = (uint8_t)(rear_spd  >> 8);    // RR high byte
    frame.data[7] = (uint8_t)(rear_spd  & 0xFFU); // RR low byte
    standaloneTx(frame);
  }

  // FORD_GEN42_TORQUE_FLAGS_ID 0x200 (Bus1, 10 ms) — PCM torque and flags.
  // D0:D1 = PrplWhlTot_Tq_Actl (4 Nm/LSB, offset −131072 Nm, Motorola b7|16).
  // D2:D3 = PrplWhlTot_Tq_LimMn (minimum propulsion torque limit).
  // D4:D5 = PrplWhlTot_Tq_Rq    (requested propulsion torque).
  // D6 b6:b7 = BrkOnOffSwtch_D_Actl, D7 b0:b6 = ACCompressorDisp (%). [TorqueDataEngFlags]
  frame.identifier       = FORD_GEN42_TORQUE_FLAGS_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x28U + get_lock_target_adjusted_value(0xD7, false); // Tq_Actl high byte: 0x28→0xFF (proportional to lock_target)
  frame.data[1] = 0xE0;                                                   // Tq_Actl low byte (static)
  frame.data[2] = 0x27U + get_lock_target_adjusted_value(0xD8, false); // Tq_LimMn high byte: 0x27→0xFF
  frame.data[3] = 0x10;                                                   // Tq_LimMn low byte (static)
  frame.data[4] = 0x27U + get_lock_target_adjusted_value(0xD8, false); // Tq_Rq high byte: 0x27→0xFF
  frame.data[5] = 0x10;                                                   // Tq_Rq low byte (static)
  frame.data[6] = 0x88; frame.data[7] = 0x00;                           // flags / AC displacement
  standaloneTx(frame);

  // FORD_GEN42_ENG_SPD_THROTTLE_ID 0x201 (Bus1, 10 ms) — PCM engine/vehicle speed.
  // D0:D1 = EngAout_N_Actl (2 rpm/LSB, Motorola b7|13).
  // D2:D3 = Veh_V_ActlEng (0.01 kph/LSB, vehicle speed from engine estimate).
  // D4:D5 = ApedPos_Pc_ActlArb (0.1 %/LSB, arbitrated accelerator position).
  // D6 b4:b5 = ApedPosPcActl_D_Qf, D6 b2 = Autostart_B_Stat. [EngVehicleSpThrottle_CG1]
  frame.identifier       = FORD_GEN42_ENG_SPD_THROTTLE_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; frame.data[1] = 0x00; // EngAout_N = 0 rpm
  frame.data[2] = 0x40; frame.data[3] = 0x00; // Veh_V_ActlEng = 0 kph
  frame.data[4] = 0x00; frame.data[5] = 0x00; // ApedPos = 0 %
  frame.data[6] = 0x00; frame.data[7] = 0x80; // quality flags / static
  standaloneTx(frame);

  // FORD_GEN42_POWERTRAIN_DATA6_ID 0x205 (Bus1, 10 ms) — PCM fuel/torque model data.
  // D4:D5 = provisional fuel-related word (rapidly varying in logs, not a tank sensor).
  // D6 = provisional fuel-related byte. [PowertrainData_6 / PROVISIONAL_0x205]
  frame.identifier       = FORD_GEN42_POWERTRAIN_DATA6_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x02; frame.data[1] = 0x8B;
  frame.data[2] = 0x01; frame.data[3] = 0xFF;
  frame.data[4] = 0x02; frame.data[5] = 0x18; // provisional fuel-related word
  frame.data[6] = 0x01; frame.data[7] = 0xFF;
  standaloneTx(frame);

  // FORD_GEN42_TRANS_GEAR_DATA2_ID 0x231 (Bus1, 10 ms) — TCM torque/gear status.
  // byte[0]        = MtrGen1AoutTqRq_No_Cs  (8-bit checksum).  Fixed at 0x01.
  // byte[1] b7:b4  = MtrGen1AoutTqRq_No_Cnt (4-bit counter). Fixed at 0x5.
  // Both are frozen because the Focus RS has a MANUAL gearbox — the TCM payload
  // never changes, so the Ford CGEA No_Cnt counter never increments.
  // byte[1] b3:b0 / byte[2-5] = MtrGen1Aout_Tq_Rq (b53|14, 0.1 Nm/LSB, −800 Nm).
  // byte[6] = TrnMil_D_Rq / EngExhBrkTq_Pc_Rq. DLC=7. [TransGearData_2]
  // Log-verified payload: 01 52 64 00 01 00 80
  frame.identifier       = FORD_GEN42_TRANS_GEAR_DATA2_ID;
  frame.data_length_code = 7;
  frame.data[0] = 0x01; frame.data[1] = 0x52; // No_Cs=0x01; No_Cnt=5 | 0x02
  frame.data[2] = 0x64; frame.data[3] = 0x00; // MtrGen1Aout_Tq_Rq
  frame.data[4] = 0x01; frame.data[5] = 0x00; // TrnMsgTxt_D_Rq / static
  frame.data[6] = 0x80U | (0x1FU + get_lock_target_adjusted_value(0x20, false)); // MtrGen1Aout_Tq_Rq upper 6 bits: 0x1F (0 Nm) → 0x3F (~+813 Nm)
  standaloneTx(frame);

  // FORD_GEN42_BODY_INFO4_ID 0x240 (Bus1, 10 ms) — BCM body information (2-byte variant).
  // DBC superset (8 B) includes EngOff_T_Actl (seconds), AmbTempImpr (0.25 °C/LSB, −128 °C).
  // Focus RS logs consistently show only 2 bytes: 0x00 0x40. [Body_Information_4_CG1]
  frame.identifier       = FORD_GEN42_BODY_INFO4_ID;
  frame.data_length_code = 2;
  frame.data[0] = 0x00;
  frame.data[1] = 0x40;
  standaloneTx(frame);
}

void Gen42_frames20()
{
  twai_message_t frame;
  frame.extd = 0;
  frame.rtr  = 0;

  // FORD_GEN42_ACCEL_BRAKE_STATUS_ID 0x080 (Bus1, 20 ms) — PCM accel/brake status.
  // D0:D1 = AccelPedal_Pc (10-bit Motorola b6|10, 0.1 %/LSB): D0[6:5]=raw[9:8], D1[7:0]=raw[7:0].
  //         Scaled 0–100 % proportional to lock_target. D0[2:0]=0x7 constant, BrakePedal_D=0.
  // D2:D3 = static 0x7530.  D4:D5 = static 0x0130.
  // D7 = 8-bit rolling counter (shared with 0x20F D8 and 0x211 D8).
  // DBC signals: AccelPedal_Pc b6|10 (0.1 %/LSB Motorola), BrakePedal_D b4|2. DLC=7. [ACCEL_BRAKE_STATUS]
  frame.identifier       = FORD_GEN42_ACCEL_BRAKE_STATUS_ID;
  frame.data_length_code = 7;
  {
    const uint16_t raw_pedal = (uint16_t)get_lock_target_adjusted_value(0xFA, false) * 4U; // 0–1000 = 0–100.0 %
    frame.data[0] = 0x07U | (uint8_t)(((raw_pedal >> 8) & 0x03U) << 5); // AccelPedal[9:8]; BrakePedal=0; bit2:0=0x7
    frame.data[1] = (uint8_t)(raw_pedal & 0xFFU);                        // AccelPedal[7:0]
  }
  frame.data[2] = 0x75; frame.data[3] = 0x30; // D3:D4 = 0x7530 (constant)
  frame.data[4] = 0x01; frame.data[5] = 0x30; // D5:D6 = 0x0130 (constant)
  frame.data[6] = gen42_main_counter;           // D7 = 8-bit rolling counter
  standaloneTx(frame);

  // FORD_GEN42_ENGINE_DATA_ID 0x090 (Bus1, 20 ms) — PCM engine speed data.
  // D1 high nibble = 4-bit rolling counter 0x0-0xF (D1 = (counter << 4) | 0x07).
  // DBC signal: EngAout_N_Actl b35|13 (2 rpm/LSB, Motorola). D7:D8 = 0x07D0 (constant). [ENGINE_DATA]
  frame.identifier       = FORD_GEN42_ENGINE_DATA_ID;
  frame.data_length_code = 8;
  frame.data[0] = (uint8_t)((gen42_090_nibble << 4) | 0x07U); // D1: 4-bit counter | 0x07
  frame.data[1] = 0x05;                                         // D2 (constant)
  frame.data[2] = 0x57;                                         // D3 (constant — EngAout_N upper bits)
  frame.data[3] = 0x07;                                         // D4 (constant)
  {
    const uint16_t V_rpm = 400U + (uint16_t)get_lock_target_adjusted_value(0xFA, false) * 10U; // 400–2900 → 800–5800 RPM
    frame.data[4] = (uint8_t)(0xC0U | ((V_rpm >> 8) & 0x1FU)); // D5: upper 5 bits of V; flags 0b110 preserved
    frame.data[5] = (uint8_t)(V_rpm & 0xFFU);                   // D6: lower 8 bits of V
  }
  frame.data[6] = 0x07; frame.data[7] = 0xD0;                  // D7:D8 = 0x07D0 (constant)
  standaloneTx(frame);

  // FORD_GEN42_BRAKE_DATA_ID 0x20F (Bus1, 20 ms) — ABS provisional brake data.
  // D1:D2 = 0x7530 (constant). D3:D4 = 0x2710 (wheel speed reference, constant).
  // D5 = 0x40 (constant). D6 = gen42_20F_d6 (+0x10 per frame, mod 256).
  // D7 = 0xC8 − D6 (complement; D6 + D7 always = 0xC8 = 200 decimal).
  // D8 = shared 8-bit rolling counter (same as 0x080 D7 and 0x211 D8).
  // D2:D3 = provisional brake pressure: 0x0FB0 observed at full brake.
  frame.identifier       = FORD_GEN42_BRAKE_DATA_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x75; frame.data[1] = 0x30; // D1:D2 = 0x7530 (constant)
  frame.data[2] = 0x27; frame.data[3] = 0x10; // D3:D4 = 0x2710 (constant)
  frame.data[4] = 0x40;                         // D5 (constant)
  frame.data[5] = gen42_20F_d6;                 // D6: +0x10 per frame, mod 256
  frame.data[6] = (uint8_t)(0xC8U - gen42_20F_d6); // D7: complement (D6 + D7 = 0xC8)
  frame.data[7] = gen42_main_counter;            // D8: shared 8-bit rolling counter
  standaloneTx(frame);

  // FORD_GEN42_DESIRED_TORQ_BRK_ID 0x211 (Bus1, 20 ms) — ABS desired torque/brake.
  // D0:D1 = PrplWhlTot_Tq_RqMx (4 Nm/LSB, offset −131072 Nm, Motorola b7|16).
  // D2:D3 b23|12 = RearDiffLck_Tq_RqMx (rear diff lock max torque request, Nm).
  // D3 b3 = AbsActv_B_Actl (ABS active flag). D4:D5 = static 0x4848.
  // D5:D6 b47|10 = VehLongOvrGnd_A_Est (0.035 m/s², offset −17.9 m/s²).
  // D8 = shared 8-bit rolling counter. [DesiredTorqBrk_CG1]
  frame.identifier       = FORD_GEN42_DESIRED_TORQ_BRK_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0xFF; frame.data[1] = 0xFE; // PrplWhlTot_Tq_RqMx
  frame.data[2] = get_lock_target_adjusted_value(0xFF, false); // RearDiffLck_Tq_RqMx upper 8 bits: 0→255 Nm (proportional to lock_target)
  frame.data[3] = 0x00;                                         // RearDiffLck_Tq_RqMx lower 4 bits = 0; AbsActv_B_Actl = 0
  frame.data[4] = 0x48; frame.data[5] = 0x48; // VehLongOvrGnd_A_Est (static)
  frame.data[6] = 0x00;                         // constant
  frame.data[7] = gen42_main_counter;            // D8: shared 8-bit rolling counter
  standaloneTx(frame);

  // FORD_GEN42_DRIVE_MODE_ID 0x190 (Bus1, 20 ms) — GWM/PCM drive mode status.
  // D3 = drive-mode byte: 0x60 when engagement is requested, 0x20 at idle/off.
  //      This is the primary proactive signal — Haldex engages BEFORE slip occurs.
  // D2 = 0x51 (engine speed proxy, ~1000 rpm idle; varies with throttle in logs).
  // D6 = 4-bit nibble counter 0x00-0x0F (gen42_190_counter).
  // D7:D8 = provisional steering angle word (0x8000 = 0°, per guide). [PROVISIONAL_0x190]
  {
    const uint8_t drive_mode = (lock_target > 0.0f) ? 0x60U : 0x20U;
    frame.identifier       = FORD_GEN42_DRIVE_MODE_ID;
    frame.data_length_code = 8;
    frame.data[0] = 0x00;                        // D1 (constant)
    frame.data[1] = 0x51;                        // D2: engine speed proxy (~1000 rpm idle)
    frame.data[2] = drive_mode;                  // D3: drive-mode (0x20=idle, 0x60=demand)
    frame.data[3] = 0x80;                        // D4 (constant)
    frame.data[4] = 0x04;                        // D5 (constant — drive-engaged status)
    frame.data[5] = gen42_190_counter & 0x0FU;   // D6: 4-bit nibble counter 0x00-0x0F
    frame.data[6] = 0x00;                        // D7 (constant)
    frame.data[7] = 0x3C;                        // D8 (constant)
    standaloneTx(frame);
  }

  // Advance counters after all 20 ms frames are sent.
  gen42_main_counter++;                                              // free-running 8-bit (0x080/0x20F/0x211 shared)
  gen42_090_nibble  = (uint8_t)((gen42_090_nibble  + 1U) & 0x0FU); // 4-bit 0-15
  gen42_20F_d6      = (uint8_t)((gen42_20F_d6      + 0x10U) & 0xFFU); // +0x10, mod 256
  gen42_190_counter = (uint8_t)((gen42_190_counter + 1U) & 0x0FU); // 4-bit 0-15
}

void Gen42_frames100()
{
  twai_message_t frame;
  frame.extd = 0;
  frame.rtr  = 0;

  // FORD_GEN42_WARN_STATUS_ID 0x212 (Bus1, 100 ms) — IPC/ABS warning status.
  // Provisional: D3/D5 interact for DSC/TCS warning; D4 b5=brake warn, D4 b3=ABS warn.
  // Log-verified payload: F8 00 38 80 15 00 00 00 [PROVISIONAL_0x212]
  frame.identifier       = FORD_GEN42_WARN_STATUS_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0xF8; frame.data[1] = 0x00; // D1: status byte, D2 (zero)
  frame.data[2] = 0x38; frame.data[3] = 0x80; // D3: DSC/TCS candidate, D4: brake/ABS candidates
  frame.data[4] = 0x15; frame.data[5] = 0x00; // D5: TCS candidate
  frame.data[6] = 0x00; frame.data[7] = 0x00;
  standaloneTx(frame);

  // FORD_GEN42_COUNTER_275_ID 0x275 (Bus1, 100 ms) — unknown rolling counter frame.
  // D1 steps +0x20 per 100 ms cycle: 0x80, 0xA0, 0xC0, 0xE0, 0x00, 0x20, 0x40, 0x60, (wrap).
  // Not matched in available DBCs. DLC=3.
  frame.identifier       = FORD_GEN42_COUNTER_275_ID;
  frame.data_length_code = 3;
  frame.data[0] = gen42_275_counter; // D1: rolling counter (+0x20 per frame)
  frame.data[1] = 0x00;              // D2 (constant)
  frame.data[2] = 0xFF;              // D3 (constant)
  standaloneTx(frame);

  gen42_275_counter = (uint8_t)((gen42_275_counter + 0x20U) & 0xFFU);
}

void Gen42_frames200()
{
  twai_message_t frame;
  frame.extd = 0;
  frame.rtr  = 0;

  // FORD_GEN42_UNKNOWN_270_ID 0x270 (Bus1, 200 ms) — unknown 200 ms heartbeat.
  // D1 = 0x81/0x82 (slow-varying in log); D2 = 0x01 constant. Not in available DBCs.
  // Log-verified most common payload: 81 01 00 00 00 00 00 00
  frame.identifier       = FORD_GEN42_UNKNOWN_270_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x81; frame.data[1] = 0x01; // D1: status/counter, D2: 0x01
  frame.data[2] = 0x00; frame.data[3] = 0x00;
  frame.data[4] = 0x00; frame.data[5] = 0x00;
  frame.data[6] = 0x00; frame.data[7] = 0x00;
  standaloneTx(frame);

  // FORD_GEN42_UNKNOWN_280_ID 0x280 (Bus1, 200 ms) — unknown 200 ms heartbeat.
  // D1:D2 = 0x00 0x00; D3 = 0xB0 (most common observed; varies slowly in log);
  // D4-D8 = 0x00. Not matched in available DBCs.
  // Log-verified most common payload: 00 00 B0 00 00 00 00 00
  frame.identifier       = FORD_GEN42_UNKNOWN_280_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; frame.data[1] = 0x00;
  frame.data[2] = 0xB0; frame.data[3] = 0x00; // D3: 0xB0 most common; slowly varies
  frame.data[4] = 0x00; frame.data[5] = 0x00;
  frame.data[6] = 0x00; frame.data[7] = 0x00;
  standaloneTx(frame);

  // FORD_GEN42_FOUR_BY_FOUR_SW_ID 0x460 (Bus1, 200 ms) — GWM 4WD switch status.
  // D1=0x00, D2=0x50 (AWD-auto / selector mode), D3-D8=0x00. [FourByFourSwitchData_FD1]
  // Log-verified payload: 00 50 00 00 00 00 00 00
  frame.identifier       = FORD_GEN42_FOUR_BY_FOUR_SW_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; frame.data[1] = 0x50; // D2=0x50: AWD auto selector position
  frame.data[2] = 0x00; frame.data[3] = 0x00;
  frame.data[4] = 0x00; frame.data[5] = 0x00;
  frame.data[6] = 0x00; frame.data[7] = 0x00;
  standaloneTx(frame);
}

void Gen42_frames1000()
{
  twai_message_t frame;
  frame.extd = 0;
  frame.rtr  = 0;

  // FORD_GEN42_PTRAIN_DATA1_ID 0x420 (Bus1, 1000 ms) — PCM powertrain data 1.
  // D1 alternates 0x62/0x63 each second.
  // D7:D8 cluster drive-mode pattern (from guide): Normal=0x10C4, Sport=0x11CC, Drift=0x12C4.
  // DBC signals: CluPdlPos_Pc_Meas D0:D1 (0.1 %/LSB), TrnTotLss_Tq_Est D3 (0.5 Nm/LSB),
  //              FuelAlcohol_Pc D7 (0.39 %/LSB). Launch Control Status: D7 b5 per guide.
  // [PowertrainData_1_CG1 / PROVISIONAL_0x420]
  {
    static uint8_t gen42_420_toggle = 0U;
    frame.identifier       = FORD_GEN42_PTRAIN_DATA1_ID;
    frame.data_length_code = 8;
    frame.data[0] = (gen42_420_toggle & 1U) ? 0x63U : 0x62U; // D1 toggle
    frame.data[1] = 0x00; frame.data[2] = 0x00;
    frame.data[3] = 0x5D; frame.data[4] = 0x43; // TrnTotLss_Tq_Est / static
    frame.data[5] = 0x10; frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    standaloneTx(frame);
    gen42_420_toggle++;
  }

  // FORD_GEN42_UNKNOWN_428_ID 0x428 (Bus1, 1000 ms) — provisional StrgWheel_PolicePkg.
  // Possible police-package steering wheel data (CGEA2011); not confirmed for Focus RS.
  // Static payload. DLC=4.
  frame.identifier       = FORD_GEN42_UNKNOWN_428_ID;
  frame.data_length_code = 4;
  frame.data[0] = 0xFF; frame.data[1] = 0x7D;
  frame.data[2] = 0x6A; frame.data[3] = 0x31;
  standaloneTx(frame);

  // FORD_GEN42_CLUSTER_INFO_ID 0x430 (Bus1, 1000 ms) — IPC cluster information fragment.
  // DBC 8-byte superset includes OdometerMasterValue D2:D4 (km), DrvSlipCtlMde_D_Rq.
  // Focus RS logs show only DLC=2: 0xBC 0x12 (static). [Cluster_Information]
  frame.identifier       = FORD_GEN42_CLUSTER_INFO_ID;
  frame.data_length_code = 2;
  frame.data[0] = 0xBC; frame.data[1] = 0x12;
  standaloneTx(frame);

  // FORD_GEN42_CLUSTER_INFO3_ID 0x433 (Bus1, 1000 ms) — IPC cluster information 3.
  // D0:D1 = FuelLvl_Pc_Dsply b7|10 (0.109 %/LSB, −5.22 % offset).
  // D4 b6 = HILL_DESC_SW (hill-descent switch). D7 b0:b3 = camera config bits.
  // Gear-state bitmask in D0-D3 per guide (gear selector position bits). [Cluster_Information_3_CG1]
  frame.identifier       = FORD_GEN42_CLUSTER_INFO3_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; frame.data[1] = 0x01;
  frame.data[2] = 0x34; frame.data[3] = 0x01; // FuelLvl_Pc / gear state bits
  frame.data[4] = 0x00; frame.data[5] = 0x20; // HILL_DESC_SW, camera config
  frame.data[6] = 0x00; frame.data[7] = 0x00;
  standaloneTx(frame);

  // FORD_GEN42_UNKNOWN_4F0_ID 0x4F0 (Bus1, 1000 ms) — unknown slow heartbeat.
  // Static content in all logs. Not matched in available DBCs. DLC=6.
  // Log-verified payload: 19 2C 22 50 23 45
  frame.identifier       = FORD_GEN42_UNKNOWN_4F0_ID;
  frame.data_length_code = 6;
  frame.data[0] = 0x19; frame.data[1] = 0x2C;
  frame.data[2] = 0x22; frame.data[3] = 0x50;
  frame.data[4] = 0x23; frame.data[5] = 0x45;
  standaloneTx(frame);

  // FORD_GEN42_UNKNOWN_4F1_ID 0x4F1 (Bus1, 1000 ms) — unknown slow heartbeat.
  // Static content in all logs. Not matched in available DBCs.
  // Log-verified payload: 41 01 90 01 90 64 61 61
  frame.identifier       = FORD_GEN42_UNKNOWN_4F1_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x41; frame.data[1] = 0x01;
  frame.data[2] = 0x90; frame.data[3] = 0x01;
  frame.data[4] = 0x90; frame.data[5] = 0x64;
  frame.data[6] = 0x61; frame.data[7] = 0x61;
  standaloneTx(frame);

  // FORD_GEN42_VIN_ASCII_ID 0x4F3 (Bus1, 1000 ms) — static ASCII build-ID fragment.
  // Content is vehicle-specific (likely a build/part code). Varies between cars.
  // Log reference: "DAM42365" (44 41 4D 34 32 33 36 35).
  // Not matched in available DBCs.
  frame.identifier       = FORD_GEN42_VIN_ASCII_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x44; frame.data[1] = 0x41; // 'D' 'A'
  frame.data[2] = 0x4D; frame.data[3] = 0x34; // 'M' '4'
  frame.data[4] = 0x32; frame.data[5] = 0x33; // '2' '3'
  frame.data[6] = 0x36; frame.data[7] = 0x35; // '6' '5'
  standaloneTx(frame);
}