#include <OpenHaldexC6_StandaloneCAN.h>
#include <OpenHaldexC6_Calculations.h>

// Standalone frames are the same frame-edit type as the passthrough path: a
// block turned OFF in the UI is simply not generated (mirrors the passthrough
// "leave the frame untouched" behaviour). Frames that are not listed as editable
// blocks for the active generation always send. All standalone Haldex-frame
// transmits go through here instead of calling twai_transmit_v2 directly.
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

static inline void standaloneTx(twai_message_t &f)
{
  f.self = 0; // a self-received frame reads as a live Haldex (flicker to ✓ with lock 0)
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
    if (seq && !isVolvoOrSaabGeneration(haldexGeneration))
      f.data[0] = calcChecksum(f.data, seq);
  }

  twai_transmit_v2(twai_bus_1, &f, 0);
}

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
  case GEN_VOLVO_GEN4:
    VolvoGen4_frames20();
    break;
  case GEN_SAAB_93_XWD:
    Saab93_frames20();
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

// Gen1 Standalone Frames
// Synthesises the minimum PQ messages the haldex needs while no donor chassis bus is
// present. All frames go out on the haldex bus (twai_bus_1).
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
  frame.data[0] = 0x00;                     // status bits (bit 3 Kupplungsschalter = clutch switch) - all clear
  frame.data[1] = gen1_demand_value(0xFE);  // inneres_Motor_Moment (engine torque, % of max) - the demand
  frame.data[2] = 0x21;                     // Motordrehzahl low byte (engine rpm x 0.25)
  frame.data[3] = 0x4E;                     // Motordrehzahl high byte: FIXED 0x4E21 = 4872 rpm (V11). It used to be scaled with
                                            // the lock, which went below 400 rpm at low targets and the 02D turns its pump off
                                            // below 400 rpm = no lock (spec_gen1_02d D5). 0x4E21 is the bench-proven full-lock value.
  frame.data[4] = 0x00;                     // inneres_Motor_Moment_ohne_extern - not read by the 02D (spec D10, was mislabelled "pedal")
  frame.data[5] = gen1_demand_value(0xFE);  // Fahrpedalwert (accelerator pedal x 0.4 %) - the demand
  appliedTorque = gen1_loss_torque_byte(0x00); // mechanisches_Motor_Verlustmoment: the 02D subtracts it from byte 1 (0x00 = max demand)
  frame.data[6] = appliedTorque;            // loss torque, see the line above
  frame.data[7] = 0x00;                     // Fahrerwunschmoment (driver-wish torque) - not read by the 02D
  standaloneTx(frame);

  // PQ Motor_2 (0x288, DLC 8) - NEW in V11 (spec_gen1_02d D4). The 02D watches for
  // this frame: without it its "all inputs valid" check never passes and 0x2C0
  // byte 0 bit 4 stays set for ever (our UI used to show that as "limp mode").
  frame.identifier = MOTOR2_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0xDA; // MO2_Mp_Code 3 (bits 6-7) + MO2_max_Mo 26 x 10 Nm = 260 Nm (bits 0-5) - same as the 02D's own default (OEM code §2.1)
  frame.data[1] = 0x00; // MO2_Kuehlm_T (coolant temperature) - not read by the 02D
  frame.data[2] = 0x00; // status bits, set below
  if (brakeForHaldex())
  {
    frame.data[2] = frame.data[2] | (1 << 0); // bit 0 MO2_BLS: brake light switch (Follow Brake on + brake pressed)
  }
  frame.data[3] = 0x00; // not read by the 02D
  frame.data[4] = 0x00; // not read by the 02D
  frame.data[5] = 0x00; // not read by the 02D
  frame.data[6] = 0x00; // not read by the 02D
  frame.data[7] = 0x00; // not read by the 02D
  standaloneTx(frame);

  // PQ Bremse_4 (0x2A0, DLC 3 per K-matrix vw_pq.dbc) - NEW in V11. Byte 0 is a
  // torque CEILING on the 02D's own request = our lock lever (spec_gen1_02d §2.3).
  // Motor_1 above is held at maximum demand while this block is on.
  frame.identifier = BRAKES4_ID;
  frame.data_length_code = 3;
  frame.data[0] = gen1_lever_byte(); // 0x7F = open, 0x81..0xC9 = partial (0xA5 ~ 50 %), 0xFD = full
  frame.data[1] = 0x00;              // not read by the 02D
  frame.data[2] = 0x00;              // not read by the 02D
  standaloneTx(frame);

  // PQ Motor_3 (0x380, DLC 8) - secondary engine ECU broadcast (vw_pq.dbc: Motor_3).
  // The 02D has NO receive slot for 0x380 (OEM code, spec_gen1_02d §1), so this
  // frame does nothing; its block is off by default since V11 (D8).
  frame.identifier = MOTOR3_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // status flags (MO3_Sta_Pedal etc.)
  frame.data[1] = 0x50; // MO3_Pedalwert / MO3_Vorgluehen mux
  frame.data[2] = 0x00; // MO3_Rad_Wu_Mo low (wheel-torque request)
  frame.data[3] = 0x00; // MO3_Rad_Wu_Mo high
  frame.data[4] = 0x00; // MO3_Offsentemp (intake-air offset)
  frame.data[5] = 0x00; // reserved / Winterprg / Freigabe_Segeln
  frame.data[6] = 0x00; // reserved
  frame.data[7] = 0xFE; // reserved
  standaloneTx(frame);

  // PQ Bremse_1 (0x1A0, DLC 8) - ABS/ESP main broadcast.
  // Bremse = brake. The 02D reads bytes 0-2 only (OEM code, spec_gen1_02d §3);
  // it does not check the counter.
  frame.identifier = BRAKES1_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x80; // bit 7 BR1_EBV_Ongr as before; bit 2 BR1_ABS_Brems set below
  if (absBrakingForHaldex())
  {
    frame.data[0] = frame.data[0] | (1 << 2); // bit 2 BR1_ABS_Brems: the car's ABS is braking (only with a chassis bus)
  }
  frame.data[1] = 0x00; // bit 3 brake light, bit 4 pressure switch, bit 7 clutch release + pump off: all clear (brake goes in Motor_2)
  frame.data[2] = 0x00; // bit 0 brake-source select clear; rest = speed, not read by the 02D
  frame.data[3] = 0x0A; // not read by the 02D
  frame.data[4] = 0xFE; // not read by the 02D
  frame.data[5] = 0xFE; // not read by the 02D
  frame.data[6] = 0x00; // not read by the 02D
  frame.data[7] = BRAKES1_counter; // BR1_Zaehler counter (not checked by the 02D)
  if (++BRAKES1_counter > 0xF)
    BRAKES1_counter = 0;
  standaloneTx(frame);

  // PQ Bremse_3 (0x4A0, DLC 8) - per-wheel speeds (vw_pq.dbc: Bremse_3, word >> 1 = 0.01 km/h).
  // Without this frame the 02D's request is 0 (OEM code §2.1). 0x0A00 = 12.8 km/h on
  // every wheel; the fronts get up to 1.27 km/h of fake slip to add demand
  // (fine: the 02D's slip hold only starts at ~3.7 km/h front-over-rear, spec §5).
  frame.identifier = BRAKES3_ID;
  frame.data_length_code = 8;
  frame.data[0] = gen1_demand_value(0xFE); // Radgeschw_VL low (front-left wheel)
  frame.data[1] = 0x0A;                    // Radgeschw_VL high
  frame.data[2] = gen1_demand_value(0xFE); // Radgeschw_VR low (front-right)
  frame.data[3] = 0x0A;                    // Radgeschw_VR high
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
  // PQ Kombi_1 (0x320, DLC 8) - NEW in V11, block OFF by default. The 02D reads
  // byte 1 bit 1 (KO1_Handbremse): pulling the handbrake between 5 and 100 km/h
  // opens the clutch (OEM code, spec_gen1_02d §2.2). While this frame is NOT sent
  // the 02D uses its own handbrake wire instead, which is why it is off by default.
  twai_message_t frame = {};
  frame.identifier = mKombi_1;
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // lamp bits - not read by the 02D
  frame.data[1] = 0x00; // bit 1 KO1_Handbremse set below; bit 0 Bremsinfo (brake fluid) clear
  if (handbrakeForHaldex())
  {
    frame.data[1] = frame.data[1] | (1 << 1); // bit 1: handbrake pulled (Follow Handbrake on + handbrake pulled)
  }
  frame.data[2] = 0x00; // not read by the 02D
  frame.data[3] = 0x00; // not read by the 02D
  frame.data[4] = 0x00; // not read by the 02D
  frame.data[5] = 0x00; // not read by the 02D
  frame.data[6] = 0x00; // not read by the 02D
  frame.data[7] = 0x00; // not read by the 02D
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
// All transmitted on the haldex bus (twai_bus_1).
// V11.00.0 (OEM code: documents/haldex-binaries/leaf_trace/spec_gen2_7h0.md §5):
//   - lock lever = Bremse_4 (0x2A0) byte 0 (0x7F open .. 0xFD full), with Motor_1 held
//     at full demand and a little front-wheel slip while lock is wanted (gen2DemandValue).
//   - the unit checks NO checksums; alive counters only on Bremse_1 b7 bits 0-3,
//     LW_1 b5 bits 4-7 and 0x4A8 b6 bits 4-7 - those must change every frame.
//   - Motor_3 (0x380), Bremse_9 (0x0AE), Bremse_10 (0x3A0) are not received by the
//     7H0 / 1K0 - their blocks are off by default (frameEditMaskDefaultsSA).
//   - 1K0907554B: identical frames and lever (only its internal final-request scale differs).
//   - one Haldex bus on the car (the second CAN controller the OEM code sets up is
//     not wired), so every frame here reaches the unit on that bus.
void Gen2_frames10()
{
  twai_message_t frame = {};
  // PQ Bremse_1 (0x1A0, DLC 8) - ABS/ESP main broadcast (vw_pq.dbc: Bremse_1).
  // The unit reads (spec_gen2_7h0 §3): b0 bit2/bit4, b1 bits 0/3/4/7, b2 bit0, b6 low
  // nibble, b7 bit5, and b7 bits 0-3 as an alive counter. "ABS/ESP bits clear" = normal car.
  frame.identifier = BRAKES1_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // b0: BR1_ASR_Anf, MSR_Anf, bit2 BR1_ABS_Brems, bit4 BR1_ESP_Ongr - all off
  if (absBrakingForHaldex())
  {
    frame.data[0] = frame.data[0] | (1 << 2); // bit2 BR1_ABS_Brems: car's ABS is braking - the unit opens the clutch
  }
  frame.data[1] = 0x40;                   // b1: bit6 BR1_Sta_MAD. bit0 BR1_Lampe_ABS now OFF (was 0x41 = ABS lamp on) - V11 TODO (bench): confirm
  frame.data[2] = 0x00;                   // b2: bit0 BR1_BKV_active (brake booster) off, bits1-7 BR1_Rad_kmh low (not read)
  frame.data[3] = 0xFE;                   // b3: BR1_Rad_kmh high (not read by the unit)
  frame.data[4] = 0xFE;                   // b4: BR1_ASRMo_sl (not read)
  frame.data[5] = 0xFE;                   // b5: BR1_ASRMo_fa (not read)
  frame.data[6] = 0x00;                   // b6: BR1_MSR_Mo (low nibble read - 0 = no engine drag request)
  frame.data[7] = BRAKES1_counter & 0x0F; // b7: bits0-3 BR1_Zaehler alive counter, bit5 BR1_ESPASR_passive = 0
  if (++BRAKES1_counter > 0xF)
    BRAKES1_counter = 0;
  standaloneTx(frame);

  // PQ Bremse_2 (0x5A0, DLC 8) - ESP/ABS sensor broadcast (vw_pq.dbc: Bremse_2).
  // Gen2 (OEM code, spec_gen2_7h0 §3) only reads b0, b1 bit0 and b7 bit7 - the
  // lock-adjusted b4/b5 below are ignored by the unit (harmless, left as they were).
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
  // The unit works out slip from these (front average - rear average). Front wheels a
  // little faster than the rear = fake slip, so the unit's own request is above the
  // Bremse_4 ceiling and the lever decides. Full whenever lock is wanted (gen2DemandValue).
  // Safety: synthetic speeds - the unit's slip protection never sees real slip in standalone.
  frame.identifier = BRAKES3_ID;
  frame.data_length_code = 8;
  frame.data[0] = gen2DemandValue(0xFE); // Radgeschw_VL low (front left)
  frame.data[1] = 0x0A;                  // Radgeschw_VL high
  frame.data[2] = gen2DemandValue(0xFE); // Radgeschw_VR low (front right)
  frame.data[3] = 0x0A;                  // Radgeschw_VR high
  frame.data[4] = 0x00;                  // Radgeschw_HL low (rear left)
  frame.data[5] = 0x0A;                  // Radgeschw_HL high
  frame.data[6] = 0x00;                  // Radgeschw_HR low (rear right)
  frame.data[7] = 0x0A;                  // Radgeschw_HR high
  standaloneTx(frame);

  // PQ Bremse_4 (0x2A0) - THE LOCK LEVER (OEM code: spec_gen2_7h0 §2.3).
  // The unit reads b0 (a ceiling on its clutch request), b1 and b2 bit0 (role not
  // decoded); b3-7 are ignored (it only takes 3 bytes). b0 uses the same 0x80-offset
  // units as the engagement it sends back (0x2C0 byte 1):
  //   0x7F = 0 % (clutch open), 0x81..0xFD = (b0 - 0x80) / 0x7D x 100 %, 0xFD = 100 %.
  //   0xFE / 0xFF / below 0x7F = "no limit" (pre-V11 sent 0x00 here = no limit).
  frame.identifier = BRAKES4_ID;
  frame.data_length_code = 8;                    // unit needs at least 3
  frame.data[0] = vag_lever_byte(0xFD);          // lock request: 0x7F / 0x81..0xFC / 0xFD
  frame.data[1] = 0x00;                          // read by the unit, role not decoded - 0 as before
  frame.data[2] = 0x00;                          // bit0 read by the unit, role not decoded - 0 as before
  frame.data[3] = 0x00;                          // not read
  frame.data[4] = 0x00;                          // not read
  frame.data[5] = 0x00;                          // not read
  frame.data[6] = BRAKES4_counter;               // not read (kept as before)
  frame.data[7] = BRAKES4_counter;               // not read (kept as before)
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
  frame.data[0] = 0xFE;                          // Giergeschw low (yaw rate)
  frame.data[1] = 0x7F;                          // Giergeschw high
  frame.data[2] = 0x03;                          // BR5_Bremsdruck (brake pressure) low
  frame.data[3] = 0x00;                          // BR5_Bremsdruck high / Stillstand / Druckvalid bits
  frame.data[4] = 0x00;                          // ESP / Autohold status bits
  frame.data[5] = 0x00;                          // BR5_Anhi / Bremslicht / Notbremsung bits
  BRAKES5_counter = BRAKES5_counter + 0x10;      // next alive count in the top 4 bits (wraps 0xF? -> 0x0? on its own)
  frame.data[6] = BRAKES5_counter & 0xF0;        // b6 bits 4-7 = alive counter (was +10 per frame, which repeated values)
  frame.data[7] = BRAKES5_counter2;              // checksum byte - the unit does not check it
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
  frame.data[5] = mLW_1_counter; // LW1_Zaehler: bits 4-7 alive counter (steps of 0x10) - the unit checks it changes every frame
  frame.data[6] = 0x00;          // LW1_ID (calibration ID) / Initquelle
  mLW_1_crc = 255 - (frame.data[0] + frame.data[1] + frame.data[2] + frame.data[3] + frame.data[5]);
  frame.data[7] = mLW_1_crc; // inverted-sum CRC over data[0..3]+data[5] (Gen2 does not check it - harmless)
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
  // is wanted - the Bremse_4 lever (frames10) sets the actual lock.
  frame.identifier = MOTOR1_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x08;                  // b0: bit3 Kupplungsschalter (read as a status bit) - as before
  frame.data[1] = gen2DemandValue(0xFE); // b1: inneres_Motor_Moment (inner torque) - full while lock wanted
  frame.data[2] = 0x20;                  // b2: Motordrehzahl low (engine rpm x 0.25)
  frame.data[3] = 0x4E;                  // b3: Motordrehzahl high - 0x4E20 = 5000 rpm, fixed (above pump-on)
  frame.data[4] = 0xFA;                  // b4: inneres_Motor_Moment_ohne_extern (not read by the unit)
  frame.data[5] = gen2DemandValue(0xFE); // b5: Fahrpedalwert (pedal) - full while lock wanted
  frame.data[6] = 0x00;                  // b6: mechanisches_Motor_Verlustmoment (loss torque) - 0, never 0xFF (= invalid -> 30 Nm)
  frame.data[7] = 0xFA;                  // b7: Fahrerwunschmoment (not read by the unit)
  standaloneTx(frame);

  // PQ Motor_2 (0x288, DLC 8) - secondary engine broadcast (vw_pq.dbc: Motor_2).
  // Unit reads b0 (mux 3 = max engine torque; mux 0 here leaves its 300 Nm default)
  // and b2 bits 0/1 (brake vote).
  frame.identifier = MOTOR2_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // MO2_Mp_Code mux 0 (MO2_CAN_Vers) - the unit keeps its 300 Nm torque reference
  frame.data[1] = 0x30; // MO2_Kuehlm_T (coolant temp * 0.75 - 48 degC)
  frame.data[2] = 0x00; // MO2_BLS (bit0) brake light, MO2_BTS (bit1), status flags - all off
  if (brakeForHaldex())
  {
    frame.data[2] = frame.data[2] | (1 << 0); // bit0 MO2_BLS: Follow Brake on and the brake is pressed
  }
  frame.data[3] = 0x0A; // MO2_GRA_Soll low (cruise-control target speed)
  frame.data[4] = 0x0A; // MO2_GRA_Soll high
  frame.data[5] = 0x10; // GRA / cruise status bits
  frame.data[6] = 0xFE; // reserved
  frame.data[7] = 0xFE; // reserved
  standaloneTx(frame);

  // PQ Motor_5 (0x480, DLC 8) - tertiary engine broadcast (vw_pq.dbc: Motor_5, multiplexed).
  // The unit only takes b0 (bits 0-5 value, bits 6-7 mux) - the rest is ignored.
  frame.identifier = MOTOR5_ID;   // 0x480
  frame.data_length_code = 8;     // DLC 8
  frame.data[0] = 0xFE;           // ASR 0x04 sets bit 4.  0x08 removes set.  Coupling open/closed - MO5_Mp_Code mux
  frame.data[1] = 0x00;           // can use to disable (>130 dec).  Was 0x00; 0x41?  0x43? - MO5_max_Moment / Drehzahl (mux)
  frame.data[2] = 0x00;           // was 0x00 no effect - MO5_Vorgluehen / E_Gas / OBD_2 lamps
  frame.data[3] = 0x00;           // was 0xFE no effect - MO5_Luefter (cooling-fan PWM)
  frame.data[4] = 0x00;           // was 0xFE miasrl no effect - reserved
  frame.data[5] = 0x00;           // was 0xFE miasrs no effect - reserved
  frame.data[6] = 0x00;           // was 0x00 - reserved
  frame.data[7] = MOTOR5_counter; // checksum / rolling counter (not read by the unit)
  MOTOR5_counter++;               // V11: was ++BRAKES1_counter (copy-paste bug that bumped Bremse_1's alive counter)
  standaloneTx(frame);

  // PQ Bremse_10 (0x3A0, DLC 8) - extended ABS/ESP frame (NOT in vw_pq.dbc).
  // NOT received by the 7H0 / 1K0 (OEM code: no receive object) - block off by default.
  // Inline notes are pre-existing reverse-engineering observations.
  frame.identifier = BRAKES10_ID;   // 0x3A0 - not received by Gen2
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
  twai_message_t frame = {};
  // PQ Kombi_1 (0x320, DLC 8) - instrument-cluster broadcast (vw_pq.dbc: Kombi_1).
  frame.identifier = mKombi_1;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // Oeldruck / Tankwarn / Vorglueh lamp bits
  frame.data[1] = 0x00; // bit0 KO1_Bremsinfo (fluid warn) off, bit1 KO1_Handbremse released. V11: was 0x02 = handbrake PULLED (unit reads b1 bit1)
  if (handbrakeForHaldex())
  {
    frame.data[1] = frame.data[1] | (1 << 1); // bit1 KO1_Handbremse: Follow Handbrake on and the handbrake is pulled
  }
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

// Gen4 Standalone Frames (0BR / 0BS / 2K, PQ frame set).
// V11.00.0: rebuilt from the OEM code (documents/haldex-binaries/leaf_trace/
// spec_gen4_0br.md). The lever is Bremse_4 byte 0 (linear, no counter/CRC
// check). Every other frame the unit monitors is sent with steady, plausible
// content so it never logs a false "missing message" fault - its own genuine
// protections (over-temperature, limp, lamp) are left alone and reported.
// Replays steering (LW_1) from a 16-step capture (lws_2[]).
// All frames go out on the haldex bus (twai_bus_1).

// Gen4-only rolling counters. "static" here = private to this file.
static uint8_t gen4Bremse5Counter = 0; // Bremse_5 b6 high nibble, 0..15 (not checked by the 0BR)
static uint8_t gen4Bremse8Counter = 0; // Bremse_8 b1 low nibble, 0..15 (not checked by the 0BR)

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
    speedKmh = received_vehicle_speed; // real speed from the car's ABS / engine frames
  }
  if (speedKmh > 326)
  {
    speedKmh = 326; // top of the 15-bit 0.01 km/h range
  }
  uint16_t speedWord = (uint16_t)(speedKmh * 100) << 1; // 0.01 km/h steps, shifted past spare bit 0
  return speedWord;
}

void Gen4_frames10()
{
  twai_message_t frame = {};
  // PQ LW_1 / mLW_1 (0x0C2, DLC 8) - steering-angle replay (vw_pq.dbc: LW_1).
  frame.identifier = mLW_1;
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  // Byte layout as the 0BR reads it (spec_gen4_0br §3); replay table is fine (§5).
  frame.data[0] = lws_2[mLW_1_counter][0]; // LW1_LRW low (steering-wheel angle * 0.04375 deg)
  frame.data[1] = lws_2[mLW_1_counter][1]; // LW1_LRW high bits 0-6, b1.7 = sign
  frame.data[2] = lws_2[mLW_1_counter][2]; // LW1_Lenk_Gesch low (steering rate - not read by Gen4)
  frame.data[3] = lws_2[mLW_1_counter][3]; // LW1_Lenk_Gesch high (not read by Gen4)
  frame.data[4] = lws_2[mLW_1_counter][4]; // LW1_ID (read by Gen4)
  frame.data[5] = lws_2[mLW_1_counter][5]; // b5 bits 1-2 status, bits 4-7 counter (not checked)
  frame.data[6] = lws_2[mLW_1_counter][6]; // not read by Gen4
  frame.data[7] = lws_2[mLW_1_counter][7]; // CRC (replayed verbatim; Gen4 copies only 6 bytes)

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
  frame.data[0] = bremse1Byte0;                     // BR1_ASR_Anf b0.0 / ABS_Brems b0.2 / EDS b0.3 / ESP b0.4 (V11: was 0x20)
  frame.data[1] = 0x00;                             // BR1 lamps / status, b1.3 + b1.4 read by Gen4 = 0 (V11: was 0x40)
  frame.data[2] = (uint8_t)(speedWord & 0xFF);      // BR1_Rad_kmh low (bit 0 = BR1_BKV_active, 0) - not read by Gen4
  frame.data[3] = (uint8_t)(speedWord >> 8);        // BR1_Rad_kmh high
  frame.data[4] = 0xFE;                             // BR1_ASRMo_sl: ASR torque limit at max = no ASR cut (not read by Gen4)
  frame.data[5] = 0xFE;                             // BR1_ASRMo_fa: same (not read by Gen4)
  frame.data[6] = 0x00;                             // BR1_MSR_Mo: no drag-torque request (not read by Gen4)
  frame.data[7] = BRAKES1_counter & 0x0F;           // BR1_Zaehler b7 bits 0-3 (0..15); b7.4/b7.5 ASR/ESP off = 0
  BRAKES1_counter++;
  if (BRAKES1_counter > 0x0F)
    BRAKES1_counter = 0; // V11: was 10..0x1F, which jumped F -> A in the nibble (§7 #21)
  standaloneTx(frame);

  // PQ Bremse_3 (0x4A0, DLC 8) - per-wheel speeds (vw_pq.dbc: Bremse_3).
  // V11: four EQUAL, steady speeds (real vehicle speed when we have it). Before
  // V11 these were lock-scaled to fake slip; the lever is Bremse_4 now (§7 #9).
  frame.identifier = BRAKES3_ID;
  frame.data_length_code = 8;
  frame.data[0] = (uint8_t)(speedWord & 0xFF); // Radgeschw_VL low (front-left)
  frame.data[1] = (uint8_t)(speedWord >> 8);   // Radgeschw_VL high
  frame.data[2] = (uint8_t)(speedWord & 0xFF); // Radgeschw_VR low (front-right)
  frame.data[3] = (uint8_t)(speedWord >> 8);   // Radgeschw_VR high
  frame.data[4] = (uint8_t)(speedWord & 0xFF); // Radgeschw_HL low (rear-left)
  frame.data[5] = (uint8_t)(speedWord >> 8);   // Radgeschw_HL high
  frame.data[6] = (uint8_t)(speedWord & 0xFF); // Radgeschw_HR low (rear-right)
  frame.data[7] = (uint8_t)(speedWord >> 8);   // Radgeschw_HR high
  standaloneTx(frame);

  // Bremse_4 byte 0 = the 0BR's direct clutch-torque lever (OEM code:
  // spec_gen4_0br §2.2 / §2.6):
  //   0 %     -> 0x7F  valid zero request, clutch commanded fully open
  //   1-99 %  -> 0x80 + % x 1.25 (linear, 112 internal torque units per step)
  //   100 %   -> 0xFE  the unit's present maximum clutch capacity
  // The unit still caps this with its own limits (over-temperature etc).
  // V11: was 0x7F x CF/100 (below 0x7F = ignored) and 0x7F at 100 % outside
  // 50:50 = clutch open (§7 #1 / #2). Learn sweeps this byte too (§7 #3).
  appliedTorque = vag_lever_byte(0xFE); // Gen4: 0xFE = full lock

  // PQ Bremse_4 (0x2A0, DLC 8) - only byte 0 is read; byte 6 counter and byte
  // 7 checksum are ignored by the 0BR (§7 #20), kept as before.
  frame.identifier = BRAKES4_ID;
  frame.data_length_code = 8;
  frame.data[0] = appliedTorque;   // lever (see above)
  frame.data[1] = 0x00;            // not read by Gen4
  frame.data[2] = 0x00;            // not read by Gen4
  frame.data[3] = 0x64;            // not read by Gen4
  frame.data[4] = 0x00;            // not read by Gen4
  frame.data[5] = 0x00;            // not read by Gen4
  frame.data[6] = BRAKES4_counter; // rolling counter, high nibble (not checked)
  BRAKES4_crc = 0;
  for (uint8_t i = 0; i < 7; i++)
  {
    BRAKES4_crc ^= frame.data[i];
  }
  frame.data[7] = BRAKES4_crc; // XOR checksum over data[0..6] (not even copied by the 0BR)
  BRAKES4_counter = BRAKES4_counter + 16;
  if (BRAKES4_counter > 0xF0)
    BRAKES4_counter = 0x00;
  standaloneTx(frame);

  // PQ Motor_1 (0x280, DLC 8) - engine ECU broadcast (vw_pq.dbc: Motor_1).
  // V11: steady values. Motor_1 only feeds the unit's own strategy, which a
  // valid Bremse_4 byte overrides, so it is no longer lock-scaled (§5).
  // Light-cruise figures chosen as plausible - not from a log.
  frame.identifier = MOTOR1_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // flags: b0.3 Kupplungsschalter 0 = clutch pedal released (V11: was left over from Bremse_4 = "pressed", §7 #4)
  frame.data[1] = 0x20; // inneres_Motor_Moment (inner torque, 0.39 % per bit)
  frame.data[2] = 0x40; // Motordrehzahl low (rpm x 0.25): 0x1F40 = 8000 -> 2000 rpm
  frame.data[3] = 0x1F; // Motordrehzahl high
  frame.data[4] = 0x20; // inneres_Motor_Moment_ohne_extern (not read by Gen4)
  frame.data[5] = 0x20; // Fahrpedalwert (pedal, 0.4 % per bit): 12.8 %
  frame.data[6] = 0x08; // mechanisches_Motor_Verlustmoment (loss torque)
  frame.data[7] = 0x20; // Fahrerwunschmoment (driver demand torque)
  standaloneTx(frame);
}

void Gen4_frames20()
{
  twai_message_t frame = {};
  // PQ Bremse_2 (0x5A0, DLC 8) - ESP/ABS sensor broadcast (vw_pq.dbc: Bremse_2).
  // Gen4 reads b0, b1.0, b3 bits 4-7, b6.6, b7.7 - not b4 (spec_gen4_0br §3).
  frame.identifier = BRAKES2_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x80;                                        // Impulszahl (pulse count)
  frame.data[1] = 0x7A;                                        // Wegimpulse_Vorderachse (front-axle path pulses)
  frame.data[2] = 0x05;                                        // mittlere_Raddrehzahl (mean wheel speed)
  frame.data[3] = BRAKES2_counter;                             // rolling counter (16-step)
  frame.data[4] = get_lock_target_adjusted_value(0x7F, false); // Querbeschleunigung (lateral G) - not read by Gen4, no effect
  frame.data[5] = 0xCA;                                        // Zeitstempel low (timestamp)
  frame.data[6] = 0x1B;                                        // Zeitstempel high
  frame.data[7] = 0xAB;                                        // status / reserved
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
  frame.data[0] = 0xDE;        // mux code 3 (bits 6-7) = MO2_max_Mo, data 0x1E x 10 = 300 Nm max engine torque
  frame.data[1] = 0xB8;        // MO2_Kuehlm_T coolant: 0xB8 x 0.75 - 48 = 90 degC (not read by Gen4)
  frame.data[2] = motor2Byte2; // b2.0 brake light, b2.1 brake test switch = 0 (spec §5), b2.4 normal operation
  frame.data[3] = 0x00;        // cruise control flags off
  frame.data[4] = 0x00;        // MO2_GRA_Soll: no cruise set speed
  frame.data[5] = 0x50;        // MO2_LL_Solldz: idle target 0x50 x 10 = 800 rpm
  frame.data[6] = 0xFE;        // MO2_Begr_Mo: no torque limit
  frame.data[7] = 0x00;        // MO2_Mo_ZWR
  standaloneTx(frame);

  // PQ Motor_5 (0x480, DLC 8). Must be present: without it the 0BR sets its own
  // limp bit (0x2C0 b0.4) - OEM code FUN_032880. Gen4 reads b0 (mux) and b6.3.
  frame.identifier = MOTOR5_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x1E; // mux code 0 (bits 6-7) = MO5_max_Moment, data 0x1E x 10 = 300 Nm (matches Motor_2)
  frame.data[1] = 0x00; // lamps off (glow, EPC, MIL, hot)
  frame.data[2] = 0x00; // MO5_Verbrauch low (fuel used)
  frame.data[3] = 0x00; // MO5_Verbrauch high
  frame.data[4] = 0x00; // MO5_Luefter: cooling fan 0 %
  frame.data[5] = 0x00; // lamps / start flags off
  frame.data[6] = 0x00; // b6.3 MO5_Momente = 0: torque figures single scale (spec §5)
  frame.data[7] = 0x00; // checksum (worked out below)
  for (uint8_t i = 0; i < 7; i++)
  {
    frame.data[7] ^= frame.data[i]; // XOR of bytes 0-6 (the 0BR copies only 7 bytes, so it never checks it)
  }
  standaloneTx(frame);

  // PQ Bremse_5 (0x4A8, DLC 8). Gen4 reads the yaw rate b0 + b1 bits 0-5
  // (0.01 deg/s), b1.6 status, b1.7 sign and the b6 counter (not checked).
  frame.identifier = BRAKES5_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;                            // BR5_Giergeschw low: yaw rate 0 (driving straight)
  frame.data[1] = 0x00;                            // yaw high bits 0-5 = 0, b1.6 status 0, b1.7 sign 0
  frame.data[2] = 0x00;                            // BR5_Bremsdruck low: brake pressure 0 (not read by Gen4)
  frame.data[3] = 0x00;                            // brake pressure high / status bits (not read by Gen4)
  frame.data[4] = 0x00;                            // hill-hold / brake light flags (not read by Gen4)
  frame.data[5] = 0x00;                            // not read by Gen4
  frame.data[6] = (uint8_t)(gen4Bremse5Counter << 4); // COUNTER in bits 4-7 (vw_pq.dbc 52|4)
  frame.data[7] = 0x00;                            // checksum (worked out below)
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
  frame.data[0] = 0x00;                       // BR8_Checksumme (worked out below)
  frame.data[1] = gen4Bremse8Counter & 0x0F;  // BR8_Zaehler bits 0-3; b1.4 / b1.7 status flags = 0
  frame.data[2] = 0x7F;                       // BR8_TolAbgl_HL: 0x7F = 0 % tyre tolerance
  frame.data[3] = 0x7F;                       // BR8_TolAbgl_HR: 0x7F = 0 %
  frame.data[4] = 0x69;                       // BR8_Istbeschl low: 0x169 = 0 m/s2 (b4.4 = 0)
  frame.data[5] = 0x01;                       // BR8_Istbeschl high bit; brake-light / ESC flags 0
  frame.data[6] = 0x00;                       // BR8_Laengsbeschl low: 0x200 = 0 m/s2
  frame.data[7] = 0x02;                       // BR8_Laengsbeschl high bits; Quattro / EPB flags 0
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
  frame.data[0] = 0x00; // no shift running, no protection request
  frame.data[1] = 0x55; // GE1_Zielgang bits 0-3 = 5 (5th gear), GE1_Wahl_Pos bits 4-7 = 5 (D)
  frame.data[2] = 0x0A; // GE1_Mrad_Mkurb: ratio 0x0A x 0.1 = 1.0 (spec §5)
  frame.data[3] = 0x00; // GE1_Soll_Mo
  frame.data[4] = 0x7F; // GE1_Fahrwistd: 0x7F = 0 (neutral driving resistance)
  frame.data[5] = 0x00; // GE1_Notlauf 0 = no gearbox limp
  frame.data[6] = 0x00; // V11 TODO (bench): Gen4 reads b6 bits 4-7 (vw_pq.dbc: counter bits 3-6 + GE1_Fehlereintr bit 7) - sent as 0 = no gearbox fault; try a counter here if the unit logs a Getriebe_1 fault
  frame.data[7] = 0x00; // GE1_WaVerl_Mo
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
  frame.data[0] = 0x24;        // lamp bits (oil pressure / hot warning etc)
  frame.data[1] = kombi1Byte1; // Bremsinfo b1.0 + Handbremse b1.1 (V11: was always 0)
  frame.data[2] = 0x1D;        // Tankinhalt (fuel level, 29 l)
  frame.data[3] = 0xB9;        // b3.0 speed source, bits 1-7 Geschwindigkeit_Kombi_1 low (~9.9 km/h)
  frame.data[4] = 0x07;        // Geschwindigkeit_Kombi_1 high
  frame.data[5] = 0x42;        // b5 bits 0-3 read by Gen4; bits 6-7 = displayed speed low bits
  frame.data[6] = 0x09;        // Angezeigte_Geschwindigkeit (displayed speed) high bits
  frame.data[7] = 0x81;        // spare
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
  twai_message_t frame = {};
  // PQ Gate_Komf_1 (0x390, DLC 8) - gateway-comfort broadcast (vw_pq.dbc: Gate_Komf_1).
  // Gen4 reads only b3.4 GK1_Rueckfahr (reverse) - spec_gen4_0br §3.
  frame.identifier = mGate_Komf_1;
  frame.data_length_code = 8;
  frame.data[0] = 0x03; // GK1_Sta_Anhaen / door & light status bits
  frame.data[1] = 0x11; // GK1_Blinker_li / re (indicators)
  frame.data[2] = 0x58; // alarm / light bits
  frame.data[3] = 0x00; // b3.4 GK1_Rueckfahr = 0: not reversing (the only bit Gen4 reads)
  frame.data[4] = 0x40; // seat-memory / sunroof bits
  frame.data[5] = 0x00; // alarm / lock bits
  frame.data[6] = 0x01; // status
  frame.data[7] = 0x08; // status / counter
  standaloneTx(frame);

  // PQ Bremse_11 (0x5B7, DLC 8) - extended brake frame (NOT in vw_pq.dbc).
  // Not in the Gen4 0BR receive table at all - the unit ignores it (spec_gen4_0br
  // §3), so its block is OFF in the V11 standalone default (same for Kombi_2).
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
  twai_message_t frame = {};
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
  twai_message_t frame = {};
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

  extern twai_message_t gen41_bus0_cache_c1;
  extern twai_message_t gen41_bus0_cache_c5;
  extern bool gen41_bus0_cache_valid_c1;
  extern bool gen41_bus0_cache_valid_c5;
  extern portMUX_TYPE gen41_bus0_cache_mux;

  twai_message_t c1, c5;
  bool has_c1, has_c5;
  taskENTER_CRITICAL(&gen41_bus0_cache_mux);
  has_c1 = gen41_bus0_cache_valid_c1;
  has_c5 = gen41_bus0_cache_valid_c5;
  if (has_c1)
    c1 = gen41_bus0_cache_c1;
  if (has_c5)
    c5 = gen41_bus0_cache_c5;
  taskEXIT_CRITICAL(&gen41_bus0_cache_mux);

  if (has_c1)
    twai_transmit_v2(twai_bus_0, &c1, 0);
  if (has_c5)
    twai_transmit_v2(twai_bus_0, &c5, 0);
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
    // Analyzer mode runs as a passive bridge; skip standalone frame generation.
    if (analyzerMode || !isStandalone)
    {
      vTaskDelay(10 / portTICK_PERIOD_MS);
      continue;
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
  twai_message_t frame = {};
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

  twai_message_t frame = {};
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
  twai_message_t frame = {};
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
  twai_message_t frame = {};
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
  twai_message_t frame = {};
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
  twai_transmit_v2(twai_bus_0, &frame, 0);

  // 0x180 (Bus1) - GM SAS, 10 ms cadence
  gen41_build_180(frame);
  standaloneTx(frame);

  gen41_swa_advance_wobble();
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
  twai_transmit_v2(twai_bus_0, &frame, 0);
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

  // MQB ESP_19 (0x0B2, DLC 8) - wheel speeds (ESP_VL/VR/HL/HR_Radgeschw_02,
  // LE, 0.0075 km/h/bit - see the real decode in OpenHaldexC6_can.cpp).
  // Wheel speed MUST keep changing or haldex slowly disengages - static
  // makes it fade over time and eventually stop. V11.00.0: the speeds no
  // longer follow the lock - the 0CQ's strategy and clutch-temperature model
  // read them (OEM code: spec_gen5_0cq D6), so they are always the gently
  // varying ~19-31 km/h that used to be sent at 100 % lock. See
  // fill_esp19_wheel_speeds().
  frame.identifier = ESP_19; // ESP_19 0x0b2
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  fill_esp19_wheel_speeds(frame.data);
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
  frame.data[0] = 0x00;                // GE_CHK_02 - checksum (the 0CQ checks no CRC - spec §1)
  frame.data[1] = GETRIEBE_11_counter; // GE_BZ_02 - rolling 4-bit counter (0x00>0x0F)
  frame.data[2] = 0x00;                // GE_MMom_Anf_02 (engine torque-intervention request) - Was 0xFE Torque intervention at the engine. Requests a short-term reduction or increase in torque from the ECU. This signal is only valid in combination with GE_MMom_Status (for MQB) or GE_MMom_Status_02 (for MLBevo).
  frame.data[3] = 0xFE;                // GE_MMom_Vorhalt_02 (Pre-control torque - anticipatory torque request)
  frame.data[4] = 0x00;                // GE_Fahrstufe (Actual gear/range selected: 5=P, 6=R, 7=N, 8=D, 9=S, 10=E, 13/14=T)
  frame.data[5] = 0x00;                // GE_Schalt_Sequenz (Shift sequence state: 0=idle, 1=shift in progress, etc.) - does not affect (>0x00)
  frame.data[6] = 0x00;                // GE_Kraftschluss / clutch lock-up (Power transmission status / clutch lock-up state) - does not affect (>0x00)
  frame.data[7] = 0x00;                // GE_Zielgang (b7 bits 4-7, target gear) = 0, see the TODO above

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
  frame.data[0] = 0x00;             // MO_CHK_02 - checksum placeholder
  frame.data[1] = MOTOR_12_counter; // MO_BZ_02 - rolling counter (0x70>0x7F)
  frame.data[2] = 0x00;             // MO_Mom_neg_verfuegbar (Negative available torque / maximum engine braking) - not read by the 0CQ
  frame.data[3] = 0x00;             // Static torque limit - not read by the 0CQ
  frame.data[4] = 0x00;             // Dynamic torque limit - not read by the 0CQ
  frame.data[5] = 0x64;             // not read by the 0CQ
  // V11.00.0: b6-b7 are ONE 16-bit engine speed (MO_Drehzahl_01, 0.25 rpm per
  // step), not a quality bit + rpm. b7 used to follow the lock, so the rpm ran
  // from ~4 rpm at 0 % to ~16000 rpm at 100 % (spec D5). Now a steady 2000 rpm.
  frame.data[6] = 0x40; // engine speed low byte  (0x1F40 = 8000 x 0.25 = 2000 rpm)
  frame.data[7] = 0x1F; // engine speed high byte

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
  // Min = Max (the exact request below), so Motor_11 can't fight the ESP_14 lever.
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
    frame.data[0] = 0x00;             // MO_CHK_01 - checksum (the 0CQ checks no CRC - spec §1)
    frame.data[1] = MOTOR_11_counter; // MO_BZ_01 - rolling counter (0x40..0x4F); high nibble = MO_Mom_Soll_Roh low bits
    frame.data[2] = 0xFA;             // MO_Mom_Soll_Roh / MO_Mom_Ist_Summe bits: Soll_Roh = 423 Nm, Ist = 494 Nm (spec D4)
    frame.data[3] = 0xFA;             // MO_Mom_Ist_Summe high bits (raw 0x3EB, below the 0x3FB "invalid" codes)
    frame.data[4] = 0x00;             // MO_Mom_Traegheit_Summe - not read by the 0CQ
    frame.data[5] = 0xFA;             // MO_Mom_Soll_gefiltert - not read by the 0CQ
    // b6/b7 only hold MO_Mom_Schub (never used by the 0CQ) and b7 bit 5
    // MO_Status_Normalbetrieb (fault-monitor enable only). They used to follow
    // the lock, flipping bit 61 with the lock value; the "massive effect" seen on
    // the bench came from ESP_14 changing in the same cycle (spec D3). V11.00.0:
    // fixed at the values that were sent at 100 % lock.
    frame.data[6] = 0xFA; // MO_Mom_Schub low bits (not used by the 0CQ)
    frame.data[7] = 0xFA; // MO_Mom_Schub high bits + b7.5 Normalbetrieb = 1
  }
  else
  {
    // ---- BPK packing (Fix Hunting toggle on, or Danger Zone live) ----
    // Shared with the normal-mode packer so the two can never drift, and so the
    // serial lab tunables apply identically in both. See fill_motor11_bpk().
    // The torque here still follows the lock (the "torque-style" lever), but the
    // 0CQ only uses it when ESP_14 is missing - ESP_14 below is what it follows.
    fill_motor11_bpk(frame.data, MOTOR_11_counter);
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

  // MQB ESP_14 (0x08A, DLC 8) - THE lock lever on the 0CQ.
  // BR_Vorg_Quer_Min/Max (b4/b5, front lock - not read by the 0CQ),
  // BR_Vorg_Allrad_Min/Max (b6/b7 - the only bytes the 0CQ reads, spec_gen5_0cq §2).
  // V11.00.0 (spec D1/D2): b6 = b7 = V, 0..250. Min = Max is the 0CQ's "exact"
  // mode: clutch target = V (0.4 % per step, same scale as Allrad_03 byte 2),
  // and its own Motor_11 strategy is ignored. V = 0 = clutch open. Never above
  // 250: 251+ means "no ESP request" and the unit runs its own strategy (that
  // is what happened at full lock with the old 0..0xFE scale). The Launch PWM
  // floor is not used here any more - Min is already pinned to Max. The unit's
  // own ramp, capacity limit and fault reactions still apply on top (spec §2.2).
  frame.identifier = ESP_14; // ESP_14 0x08A
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;           // ESP_CHK_14 - checksum (the 0CQ checks no CRC - spec §1)
  frame.data[1] = ESP_14_counter; // ESP_BZ_14 - rolling counter (0x10>0x1F), not read by the 0CQ
  frame.data[2] = 0x00;           // not read by the 0CQ
  frame.data[3] = 0x00;           // BR_Status_Quer_ESP (VAQ only) - not read by the 0CQ

  appliedTorque = gen5_esp14_lever_byte(); // V, 0..250

  frame.data[4] = appliedTorque; // BR_Vorg_Quer_Min - not read by the 0CQ, mirrored
  frame.data[5] = appliedTorque; // BR_Vorg_Quer_Max - not read by the 0CQ, mirrored
  frame.data[6] = appliedTorque; // BR_Vorg_Allrad_Min = V (exact mode)
  frame.data[7] = appliedTorque; // BR_Vorg_Allrad_Max = V

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
  // The 0CQ reads the steering angle from bits 16-28 (b2 + b3 bits 0-4, 0.1 deg
  // per step) and its sign from bit 29 (OEM code: spec_gen5_0cq §3). b2 is the
  // angle low byte, not a sensor status. V11.00.0: angle 0 (was 0x01 = 0.1 deg).
  frame.data[2] = 0x00;           // LWI_Lenkradwinkel low byte = 0 (straight ahead)
  frame.data[3] = 0x00;           // LWI_Lenkradwinkel high bits + sign (b3.5) = 0
  frame.data[4] = 0x00;           // not read by the 0CQ
  frame.data[5] = 0x00;           // not read by the 0CQ
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
  twai_message_t frame = {};
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
  // V11.00.0: the 0CQ does NOT receive ESP_10 (OEM code: spec_gen5_0cq §1 RX table,
  // D9), so block 5 is OFF in the standalone defaults. The old "affects 40 %" note
  // below must have been something else changing at the same time.
  frame.identifier = ESP_10;                            // ESP_10 0x116
  frame.data_length_code = 8;                           // DLC 8
  frame.data[0] = 0x00;                                 // BR_CHK_10 - checksum
  frame.data[1] = ESP_10_counter;                       // BR_BZ_10 - rolling counter (0x00>0x0F)
  frame.data[2] = 0x01;                                 // ESP_QBit / status flags
  frame.data[3] = 0x04;                                 // ESP_Gierrate low (yaw rate)
  frame.data[4] = 0x00;                                 // ESP_Gierrate high
  frame.data[5] = 0x40;                                 // ESP_VZ_Gierrate (yaw sign)
  frame.data[6] = 0x00;                                 // ESP_Querbeschleunigung low (lateral G)
  frame.data[7] = 0xFF;                                 // ESP_Querbeschleunigung high (old note "affects 40 %" - not a 0CQ effect, see above)
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
  frame.data[0] = 0x00;           // BR_CHK_05 - checksum (worked out below)
  frame.data[1] = ESP_05_counter; // BR_BZ_05 - rolling counter (0x80>0x8F)
  frame.data[2] = 0x64;           // ESP_Bremsdruck low byte: raw 0x064 = 100 = 0 bar (K-matrix 0.3 bar/step, -30 bar offset) - not braking
  // b3: bits 0-1 = brake pressure high bits (0), bit 2 = ESP_Fahrer_bremst
  // (driver braking), bits 6-7 = 0xC0 as before. V11.00.0: bit 2 follows
  // brakeForHaldex() ("Follow Brake" + brake input wire or the car's CAN). The
  // 0CQ itself does not read bit 2 (spec §3) but the car's other modules do.
  frame.data[3] = 0xC0; // brake pressure high bits 0, not braking
  if (brakeForHaldex())
  {
    frame.data[3] = frame.data[3] | (1 << 2); // b3 bit 2 = ESP_Fahrer_bremst (driver is braking)
  }
  frame.data[4] = 0x00;                                 // not read by the 0CQ
  frame.data[5] = 0x00;                                 // not read by the 0CQ
  frame.data[6] = 0xFD;                                 // not read by the 0CQ
  frame.data[7] = 0x00;                                 // not read by the 0CQ (old bench note "affects on/off" was not a 0CQ effect)
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
  // The 0CQ reads the lateral acceleration (b2) and the yaw rate (bits 40-53)
  // with its sign (bit 54) (OEM code: spec_gen5_0cq §3). V11.00.0: 0 g and no
  // yaw (was -0.01 g and 0.12 deg/s) - a car driving straight.
  frame.identifier = ESP_02;                            // ESP_02 0x101
  frame.data_length_code = 8;                           // DLC 8
  frame.data[0] = 0x00;                                 // BR_CHK_02 - checksum
  frame.data[1] = ESP_02_counter;                       // BR_BZ_02 - rolling counter (0x00>0x1F)
  frame.data[2] = 0x7F;                                 // ESP_Querbeschleunigung (0.01 g/step, -1.27 g offset): 0x7F = 0 g
  frame.data[3] = 0x0F;                                 // not read by the 0CQ
  frame.data[4] = 0x82;                                 // not read by the 0CQ
  frame.data[5] = 0x00;                                 // ESP_Gierrate low byte (0.01 deg/s per step) = 0
  frame.data[6] = 0x00;                                 // ESP_Gierrate high bits (b6 bits 0-5) = 0, ESP_VZ_Gierrate sign (b6 bit 6) = 0
  frame.data[7] = 0x00;                                 // not read by the 0CQ
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
  frame.data[0] = 0x00;           // BR_CHK_21 - checksum (worked out below)
  frame.data[1] = ESP_21_counter; // BR_BZ_21 - rolling counter (0x00>0x1F)
  frame.data[2] = 0x1F;           // not read by the 0CQ
  frame.data[3] = 0x80;           // not read by the 0CQ
  frame.data[4] = 0x00;           // not read by the 0CQ
  frame.data[5] = 0x00;           // not read by the 0CQ
  frame.data[6] = 0x00;           // not read by the 0CQ
  frame.data[7] = 0x00;           // b7: no ABS / ASR / EDS / ESP intervention
  if (absBrakingForHaldex())
  {
    frame.data[7] = frame.data[7] | (1 << 0); // b7 bit 0 = ABS_Bremsung (ABS is braking)
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
  frame.identifier = KOMBI_01; // kombi 1 0x30b
  frame.data_length_code = 8;  // DLC 8
  frame.data[0] = 0x10;        // KBI status / KBI_Geschw_Anzeige low (angle of turn (block 011) low byte)
  frame.data[1] = 0x20;        // KBI counter / KBI_Geschw_Anzeige high (checksum (0x20>0x2F))
  frame.data[2] = 0x02;        // b2: bit 7 KBI_Handbremse = 0 (released), rest 0x02 as before
  if (handbrakeForHaldex())
  {
    frame.data[2] = frame.data[2] | (1 << 7); // b2 bit 7 = KBI_Handbremse (handbrake pulled)
  }
  frame.data[3] = 0x00;        // KBI_Kilometerstand byte 1
  frame.data[4] = 0x0C;        // KBI_Kilometerstand byte 2
  frame.data[5] = 0x00;        // KBI status
  frame.data[6] = 0x00;        // KBI warning lamps
  frame.data[7] = 0x24;        // KBI status reserved
  standaloneTx(frame);
}

void Gen5_0CQ_frames100()
{
  twai_message_t frame = {};
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
  if (ESP_07_counter > 0x2F) // V11.00.0: was "> 0x20 -> 0x2F", which ran backwards and stuck at 0x2F (spec_gen5_0cq D15; harmless, the 0CQ doesn't read it)
  {
    ESP_07_counter = 0x20; // counter runs 0x20..0x2F
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
  twai_message_t frame = {};
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
  twai_message_t frame = {};
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

  // MQB Motor_Code_01 (0x641, DLC 8) - engine code / identification broadcast.
  // The 0CQ reads b1 bits 4-5 = MO_Faktor_Momente (1/2/3), which MULTIPLIES the
  // Motor_11 torque, and b4 bits 4-7 once at start-up (must not be 0) (OEM code:
  // spec_gen5_0cq §3). V11.00.0: the factor is now set on purpose (= 1) instead
  // of only being 1 because the counter happened to run 0x10..0x1F (spec D18),
  // and the duplicated header lines are gone (D16).
  frame.identifier = MOTOR_CODE_01; // motor_code_01 0x641
  frame.data_length_code = 8;       // DLC 8
  frame.data[0] = 0x00;             // MO_CHK_Code - checksum (worked out below)
  frame.data[1] = (1 << 4) | (MOTOR_CODE_01_counter & 0x0F); // b1 bits 0-3 rolling counter, bits 4-5 MO_Faktor_Momente = 1
  frame.data[2] = 0x2B;             // MO_Code byte (engine identification ASCII/code)
  frame.data[3] = 0x53;             // MO_Code byte
  frame.data[4] = 0x14;             // b4 bits 4-7 = 1 (not 0, read once at start-up by the 0CQ)
  frame.data[5] = 0x14;             // MO_Code byte
  frame.data[6] = 0xD7;             // MO_Code byte
  frame.data[7] = 0x24;             // MO_Code byte
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
// Gen5 (0CQ) VAQ Standalone Frames - generation 52
// A copy of the Gen5 0CQ set (same IDs, bytes, counters, CRCs and
// standaloneTx gating - gen 52 shares the FE_GEN_50 frame-edit blocks). 
// Each frame carries a 'VAQ K-matrix' note saying which signals
// the VAQ node is listed as receiving (MQB FCAN K-matrix, node 'VAQ'); frames
// marked NOT in its receive list are the first candidates to drop. 
// Feedback comes back on Quersperre_03 (0x137) rather than Allrad_03 (0x118);
// diagnostics on 0x71E/0x788 rather than 0x70F/0x779 (both decoded/selectable).
// =============================================================================
void Gen5_0CQ_VAQ_frames10()
{
  twai_message_t frame = {};
  // MQB ESP_18 (0x135, DLC 8) - ESP minor broadcast. Fixed response, no changes.
  // VAQ K-matrix: NOT in the VAQ receive list (no VAQ signal on ESP_18). Kept
  // like-for-like; block 10 can drop it to prove it is dead weight on this unit.
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

  // MQB ESP_19 (0x0B2, DLC 8) - wheel speeds (ESP_VL/VR/HL/HR_Radgeschw_02,
  // LE, 0.0075 km/h/bit - see the real decode in OpenHaldexC6_can.cpp).
  // Wheel speed MUST keep changing or haldex slowly disengages - static
  // makes it fade over time and eventually stop. Front wheels are also
  // inflated above rear by a lock_target-proportional delta (simulated
  // slip requesting engagement) - see fill_esp19_wheel_speeds().
  // VAQ K-matrix: reads all four ESP_*_Radgeschw_02 (LE16, 0.0075 km/h/bit):
  // b0-1 HL, b2-3 HR, b4-5 VL, b6-7 VR. For a FRONT transverse lock the
  // relevant slip is left vs right (VL vs VR), not front vs rear - the 0CQ
  // keep-alive dither is kept as-is, a VL/VR delta is the VAQ-specific lever.
  frame.identifier = ESP_19; // ESP_19 0x0b2
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  fill_esp19_wheel_speeds(frame.data);
  standaloneTx(frame);

  // MQB Getriebe_11 (0x0AD, DLC 8) - transmission ECU broadcast.
  // GE_MMom_Anf_02 (engine torque request), GE_MMom_Vorhalt_02 (anticipatory torque),
  // GE_Fahrstufe (selected gear), GE_Schalt_Sequenz (shift state), GE_Zielgang.
  // VAQ K-matrix: reads only GE_Zielgang (b7 bits 4-7: 0 P/N, 1-8 gear).
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
  // VAQ K-matrix: reads only MO_Drehzahl_01 (b6-7 LE16, 0.25 rpm/bit).
  // 0CQ lock-modulates b7 ('does affect'): to the VAQ that is the rpm high
  // byte, 0x00..0xFA -> 0..16000 rpm. An 'engine running' gate is a likely
  // reason it mattered - test a fixed b7 (e.g. 0x0C = ~800 rpm) on the VAQ.
  // V11 TODO (bench): the 0CQ set now sends a steady 2000 rpm (b6 0x40, b7 0x1F,
  // spec_gen5_0cq D5). The VAQ's use of rpm isn't in the spec (its calibration
  // block is missing), so this copy still follows the lock until benched.
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
    // Shared with the normal-mode packer so the two can never drift, and so the
    // serial lab tunables apply identically in both. See fill_motor11_bpk().
    // (appliedTorque is set by the ESP_14 frame below.)
    fill_motor11_bpk(frame.data, MOTOR_11_counter);
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
  // VAQ K-matrix: reads BR_Status_Quer_ESP (b3 bits 3-5: 0 cross lock
  // deactivated, 1 ESP error, 2 comms disturbed, 3 ESP ready, 4 ESP requests
  // cross lock, 5 AYC support), BR_Vorg_Quer_Min (b4) and BR_Vorg_Quer_Max (b5),
  // 0.4 %/bit, 252 = quick close/open, 254 init, 255 error. The Allrad bytes
  // b6/b7 are NOT read by the VAQ. 0CQ sends b3 = 0x00 = 'cross lock
  // deactivated' - PRIME SUSPECT if the VAQ never engages: try b3 = 0x18
  // (ready) or 0x20 (ESP requests cross lock) with OVR 08A 3 0x18 / 0x20.
  //
  // BENCH RESULT 2026-09-17 (real 0CQ907554E VAQ): this frame IS the lever.
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
  frame.data[0] = 0x00;           // ESP_CHK_14 - checksum placeholder
  frame.data[1] = ESP_14_counter; // ESP_BZ_14 - rolling counter (0x10>0x1F)
  frame.data[2] = 0x00;           // DSR steering-torque offset (not read by the VAQ)

  // Cross-lock request in plain percent: 0..250 = 0..100 % (0.4 %/bit).
  // OEM code (spec_gen5_0cq §8.8, D20): the VAQ reads b3 bits 3-5, b4 and b5 -
  // this layout matches. V11 TODO (bench): the bench says the VAQ follows Quer_Min
  // "1:1 in plain percent", but this still goes through the old 0CQ correction
  // curve. Try quer = gen5_esp14_lever_byte() (linear 0..250) on the bench.
  const uint8_t quer = get_lock_target_adjusted_value(250, false);
  frame.data[3] = quer ? 0x20 : 0x00; // BR_Status_Quer_ESP (b3 bits 3-5): 4 "ESP requests cross lock" / 0 deactivated
  frame.data[4] = quer;               // BR_Vorg_Quer_Min - the VAQ follows this one
  frame.data[5] = quer;               // BR_Vorg_Quer_Max = Min: exact command, no rule-mode headroom

  // Allrad bytes (b6/b7) are not stored by the VAQ (OEM code: its ESP_14 buffer
  // is only 6 bytes long, spec §8.8). V11.00.0: same packing as the 0CQ set
  // (Min = Max = V, 0..250) so the two stay comparable.
  appliedTorque = gen5_esp14_lever_byte();
  frame.data[6] = appliedTorque; // BR_Vorg_Allrad_Min (not read by the VAQ)
  frame.data[7] = appliedTorque; // BR_Vorg_Allrad_Max (not read by the VAQ)

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
  frame.data[0] = 0x00;                  // LWI_CHK - checksum placeholder
  frame.data[1] = LWI_01_counter & 0x0F; // LWI_BZ (b1[0-3]); b1.4 Sensorstatus = 0 OK; b1.7 QBit = 0 valid
  frame.data[2] = 0x01;                  // LWI_Lenkradwinkel low byte (0.1 deg)
  frame.data[3] = 0x00;                  // LWI_Lenkradwinkel high bits + VZ (b3.5) = 0
  frame.data[4] = 0x00;                  // LWI_Lenkradw_Geschw low
  frame.data[5] = 0x00;                  // LWI_Lenkradw_Geschw high / VZ
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;

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
  frame.data[0] = 0x00;             // MO_CHK_20 - checksum
  frame.data[1] = MOTOR_20_counter; // MO_BZ_20 (b1[0-3]) + Fahrpedalrohwert low nibble (b1[4-7]) = 0
  frame.data[2] = 0x00;             // Fahrpedalrohwert high nibble = 0 %, MO_QBit_Fahrpedalwerte_01 (b2.4) = 0 valid
  frame.data[3] = 0xC0;             // real-car idle value
  frame.data[4] = 0x39;             // real-car idle value
  frame.data[5] = 0x50;             // real-car idle value
  frame.data[6] = 0x7E;             // real-car idle value
  frame.data[7] = 0xFE;             // real-car idle value

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
  // VAQ K-matrix: reads BZ, ESP_QBit_Fahrer_bremst (b1 bit 5), ESP_Fahrer_bremst
  // (b3 bit 2), ESP_Verz_TSK_aktiv (b3 bit 3), ESP_Verz_EPB_aktiv (b7 bit 2).
  // The 0CQ 'b3 affects' finding is ESP_Fahrer_bremst (brake threshold
  // exceeded); 0xC0 leaves it clear.
  // V11.00.0: b3 bit 2 follows brakeForHaldex() ("Follow Brake"); off = 0 as before.
  frame.identifier = ESP_05;                            // ESP_05 0x106
  frame.data_length_code = 8;                           // DLC 8
  frame.data[0] = 0x00;                                 // BR_CHK_05 - checksum
  frame.data[1] = ESP_05_counter;                       // BR_BZ_05 - rolling counter (0x80>0x8F)
  frame.data[2] = 0x64;                                 // BR_Bremsdruck low byte: raw 100 = 0 bar (not braking)
  frame.data[3] = 0xC0;                                 // b3: pressure high bits 0, ESP_Fahrer_bremst (bit 2) = 0
  if (brakeForHaldex())
  {
    frame.data[3] = frame.data[3] | (1 << 2); // b3 bit 2 = ESP_Fahrer_bremst (driver is braking)
  }
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
  // VAQ K-matrix: NOT in the VAQ receive list. Kept like-for-like.
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
  // VAQ K-matrix: reads BZ, the three QBits (b1 bits 4-6), Querbeschleunigung
  // (b2, 0.01 g - 1.27), Laengsbeschl (b3 + b4[0-1], 0.03125 m/s2 - 16),
  // Gierrate (b5 + b6[0-5], 0.01 deg/s) and VZ_Gierrate (b6 bit 6). 0CQ values
  // decode to -0.01 g lateral, +0.47 m/s2 longitudinal, 0.12 deg/s yaw.
  // Bench 2026-09-17: the VAQ logged the ESP_02 "missing" DTC (0x00410F) with
  // the 0CQ packing on. Cause: the 0CQ counter runs 0x00..0x1F, so for half
  // of every cycle b1 bit 4 = ESP_QBit_Querb = 1 "replacement/error value"
  // (K-matrix VAL_ 257). Low nibble only -> all three QBits (b1[4-6]) = 0 valid.
  frame.identifier = ESP_02;                            // ESP_02 0x101
  frame.data_length_code = 8;                           // DLC 8
  frame.data[0] = 0x00;                                 // BR_CHK_02 - checksum
  frame.data[1] = ESP_02_counter & 0x0F;                // BR_BZ_02 (b1[0-3]); QBits (b1[4-6]) = 0 valid
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
  // VAQ K-matrix: reads BZ and b7 bits 0 ABS_Bremsung, 1 ASR_Anf, 4
  // EDS_Eingriff, 5 ESP_Eingriff. 0CQ b7 = 0x00 -> no interventions.
  // V11.00.0: ABS_Bremsung (b7 bit 0) carries absBrakingForHaldex() (the car's ABS).
  frame.identifier = ESP_21;                            // ESP_21 0x0fd
  frame.data_length_code = 8;                           // DLC 8
  frame.data[0] = 0x00;                                 // BR_CHK_21 - checksum
  frame.data[1] = ESP_21_counter;                       // BR_BZ_21 - rolling counter (0x00>0x1F)
  frame.data[2] = 0x1F;                                 // ESP_21 status (in diagnosis? none affect)
  frame.data[3] = 0x80;                                 // ESP_21 status (sometimes 0xC0, sometimes 0x00)
  frame.data[4] = 0x00;                                 // ESP_21 status (sometimes 0x3A, somtimes 0x39)
  frame.data[5] = 0x00;                                 // ESP_21 reserved
  frame.data[6] = 0x00;                                 // ESP_21 reserved
  frame.data[7] = 0x00;                                 // b7: no ABS / ASR / EDS / ESP intervention
  if (absBrakingForHaldex())
  {
    frame.data[7] = frame.data[7] | (1 << 0); // b7 bit 0 = ABS_Bremsung (ABS is braking)
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
  static uint8_t kombi01Bz = 0;
  frame.identifier = KOMBI_01;                          // kombi 1 0x30b
  frame.data_length_code = 8;                           // DLC 8
  frame.data[0] = 0x10;                                 // KBI lamps (b0) - all off except bit 4 SILA_gueltig
  frame.data[1] = (uint8_t)(0x20 | (kombi01Bz & 0x0F)); // Kombi_01_BZ (b1[0-3]) rolling + 0CQ high nibble
  kombi01Bz++;
  frame.data[2] = 0x02; // b2: bit 7 KBI_Handbremse = 0 (released), rest 0x02 as before
  if (handbrakeForHaldex())
  {
    frame.data[2] = frame.data[2] | (1 << 7); // V11.00.0: b2 bit 7 = KBI_Handbremse from handbrakeForHaldex()
  }
  frame.data[3] = 0x00; // KBI_Kilometerstand byte 1
  frame.data[4] = 0x0C; // KBI_Kilometerstand byte 2
  frame.data[5] = 0x00; // KBI status
  frame.data[6] = 0x00; // KBI warning lamps
  frame.data[7] = 0x24; // KBI status reserved
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
  // VAQ K-matrix: NOT in the VAQ receive list. Kept like-for-like.
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
  // VAQ K-matrix: NOT in the VAQ receive list. Kept like-for-like.
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
  // VAQ K-matrix: NOT in the VAQ receive list. Kept like-for-like.
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
  // VAQ K-matrix: reads MO_Gangposition (b2 bits 4-7: 0 N, 1-8 gear, 9 P,
  // 10 S, 11 D/E, 13 R, 14 undefined), MO_Fahrer_bremst (b3 bit 4), its QBit
  // (b3 bit 5) and MO_Kuppl_schalter (b4 bit 5). 0CQ b2 = 0xE6 -> gear 14
  // 'not defined'; try 0x26 (gear 2) if the VAQ wants a real gear.
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
  // VAQ K-matrix: reads only ESP_Rollenmodus_Deaktivieren (b7 bit 6).
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

void Gen5_0CQ_VAQ_frames200()
{
  twai_message_t frame = {};
  // PQ mKombi_2 (0x0C2, DLC 8) - electronic power steering / instrument-cluster slow.
  // Transmit currently disabled (commented out below).
  // VAQ K-matrix: PQ-era ID, not an MQB FCAN message at all. Transmit stays
  // disabled, exactly as in the 0CQ set.
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

void Gen5_0CQ_VAQ_frames1000()
{
  twai_message_t frame = {};
  // MQB Motor_07 (0x640, DLC 8) - engine slow-rate diagnostics broadcast.
  // VAQ K-matrix: NOT in the VAQ receive list. Kept like-for-like.
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

  // VAQ K-matrix: reads CHA_Ziel_FahrPr_VAQ (b4 bits 4-7). 0CQ b4 = 0x02 ->

  // VAQ programme 0 'no function'. Candidates: 0x12 / 0x22 / 0x32 (programme

  // 1 / 2 / 3) via OVR 385 4 0x12 etc.

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
  // VAQ K-matrix: reads SI_NWDF (b1 bit 2) and SI_NWDF_gueltig (b1 bit 3).
  // 0CQ b1 = 0x3C sets both (network diagnostics released and valid).
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

  // VAQ K-matrix: reads MO_Faktor_Momente_02 (b1 bits 4-5) and MO_Kraftstoffart

  // (b4 bits 4-7). b4 = 0x14 -> petrol. OEM code (spec_gen5_0cq §8.8): the VAQ
  // reads Motor_Code_01 b1 bits 4-5 at the same place as the 0CQ.
  // V11.00.0: Faktor = 1 set on purpose (spec D18), duplicate header lines gone
  // (D16). Same bytes as before, so nothing changes on the wire. The checksum is
  // still left at 0 here (it always was on the VAQ set).
  // MQB Motor_Code_01 (0x641, DLC 8) - engine code / identification broadcast.
  frame.identifier = MOTOR_CODE_01; // motor_code_01 0x641
  frame.data_length_code = 8;       // DLC 8
  frame.data[0] = 0x00;             // MO_CHK_Code - checksum placeholder
  frame.data[1] = (1 << 4) | (MOTOR_CODE_01_counter & 0x0F); // b1 bits 0-3 rolling counter, bits 4-5 MO_Faktor_Momente = 1
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
  // VAQ K-matrix: reads BR_Systemart (b1 bits 4-5), BR_QBit_Reifenumfang (b6
  // bit 3) and BR_Reifenumfang (b6[4-7] + b7, mm). Counter 0x30-0x3F ->
  // Systemart 3 'ESP with EPB'; b6/b7 = 0xE2/0x79 -> 1950 mm, QBit 0.
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
  // VAQ K-matrix: reads DGN_Verlernzaehler (b0), KBI_Kilometerstand (b1-b3[0-3])
  // and the date/time fields. All static in this set.
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
  // VAQ K-matrix: reads KBI_Standzeit_02 (b2[4-7] .. b4[0-4], seconds).
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
// The 0AY (0AY907554F) is a PQ unit - PQ frame set, feedback Allrad_1 0x2C0 -
// with the same lever as Gen4 0BR: Bremse_4 (0x2A0) byte 0 (OEM code:
// spec_gen5_0ay §1/§2). It receives 16 broadcast frames (spec §3); none has a
// counter or checksum check. Bremse_8 0x1AC, Bremse_11 0x5B7 and Kombi_2 0x420
// are NOT received by the 0AY - their blocks default OFF (V11.00.0).
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
  frame.data[0] = lws_2[mLW_1_counter][0]; // LW1_LRW low (steering-wheel angle * 0.04375 deg)
  frame.data[1] = lws_2[mLW_1_counter][1]; // LW1_LRW high bits 0-6, b1.7 = sign
  frame.data[2] = lws_2[mLW_1_counter][2]; // LW1_Lenk_Gesch low (steering rate, not read)
  frame.data[3] = lws_2[mLW_1_counter][3]; // LW1_Lenk_Gesch high (not read)
  frame.data[4] = lws_2[mLW_1_counter][4]; // LW1_ID (read by the 0AY)
  frame.data[5] = lws_2[mLW_1_counter][5]; // LW1_Status / counter (counter not checked)
  frame.data[6] = lws_2[mLW_1_counter][6]; // not stored by the 0AY (DLC 6)
  frame.data[7] = lws_2[mLW_1_counter][7]; // CRC (replayed verbatim, not stored)

  mLW_1_counter++;
  if (mLW_1_counter > 15)
    mLW_1_counter = 0;
  standaloneTx(frame);

  // Steady wheel / vehicle speed for Bremse_1 and Bremse_3, in 0.01 km/h.
  // The car's real speed when OpenHaldex has a chassis bus, otherwise a fixed
  // 10.00 km/h (about what the pre-V11 frames sent). Same on all four wheels:
  // no fake slip - the lever is Bremse_4 (spec_gen5_0ay §5 / §7 #4).
  uint16_t wheelSpeed = 1000; // 10.00 km/h when no chassis speed is available
  if (hasCANChassis)
  {
    uint16_t chassisSpeed = received_vehicle_speed; // km/h from the car
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
  frame.data[0] = 0x00; // b0.0 ASR_Anf, b0.3 EDS, b0.4 ESP intervention = 0
  if (absBrakingForHaldex())
  {
    frame.data[0] = frame.data[0] | (1 << 2); // b0.2 BR1_ABS_Brems - ABS module is braking
  }
  frame.data[1] = 0x00; // lamps off
  if (brakeForHaldex())
  {
    frame.data[1] = frame.data[1] | (1 << 3); // b1.3 BR1_Lichtschalt - brake-light switch
  }
  frame.data[2] = (uint8_t)(wheelSpeedWord & 0xFF); // b2.0 BKV = 0, bits 1-7 BR1_Rad_kmh low (not read by the 0AY)
  frame.data[3] = (uint8_t)(wheelSpeedWord >> 8);   // BR1_Rad_kmh high
  frame.data[4] = 0xFE;                             // BR1_ASRMo_sl (ASR torque, no reduction)
  frame.data[5] = 0xFE;                             // BR1_ASRMo_fa (ASR torque, no reduction)
  frame.data[6] = 0x00;                             // BR1_MSR_Mo
  frame.data[7] = BRAKES1_counter;                  // BR1_Zaehler - counter, not checked by the 0AY
  if (++BRAKES1_counter > 0x1F)
    BRAKES1_counter = 10;
  standaloneTx(frame);

  // PQ Bremse_3 (0x4A0, DLC 8) - per-wheel speeds (vw_pq.dbc: Bremse_3).
  // Each wheel is a 16-bit little-endian word, speed in bits 1-15 (0.01 km/h).
  // Was lock-scaled before V11 (fake rear>front slip that changed with lock).
  frame.identifier = BRAKES3_ID;
  frame.data_length_code = 8;
  frame.data[0] = (uint8_t)(wheelSpeedWord & 0xFF); // Radgeschw_VL low (front-left)
  frame.data[1] = (uint8_t)(wheelSpeedWord >> 8);   // Radgeschw_VL high
  frame.data[2] = (uint8_t)(wheelSpeedWord & 0xFF); // Radgeschw_VR low (front-right)
  frame.data[3] = (uint8_t)(wheelSpeedWord >> 8);   // Radgeschw_VR high
  frame.data[4] = (uint8_t)(wheelSpeedWord & 0xFF); // Radgeschw_HL low (rear-left)
  frame.data[5] = (uint8_t)(wheelSpeedWord >> 8);   // Radgeschw_HL high
  frame.data[6] = (uint8_t)(wheelSpeedWord & 0xFF); // Radgeschw_HR low (rear-right)
  frame.data[7] = (uint8_t)(wheelSpeedWord >> 8);   // Radgeschw_HR high
  standaloneTx(frame);

  // PQ Bremse_4 (0x2A0, DLC 8) - THE LEVER (OEM code: spec_gen5_0ay §2).
  // Byte 0: 0x7F = clutch open, 0x80 + n = n x 112 torque request, 0xFD = max.
  // 0xFE means HOLD on the 0AY and 0x00-0x7E / 0xFF are ignored, so 100 % is
  // 0xFD. vag_lever_byte() sends 0x7F whenever a lock gate is closed - ABS
  // braking, handbrake (Follow Handbrake), speed / throttle settings - which
  // a valid Bremse_4 needs, because it outranks the unit's own clutch-open.
  // Before V11 this was adj(0x7F) - 0x7F: 100 % gave 0x00 (ignored!).
  frame.identifier = BRAKES4_ID;
  frame.data_length_code = 8;
  frame.data[0] = vag_lever_byte(0xFD); // lever byte
  frame.data[1] = 0x00;                 // not read by the 0AY
  frame.data[2] = 0x00;                 // b2 bits 0-1 have a getter but no user in the 0AY
  frame.data[3] = 0x00;                 // not stored by the 0AY (it keeps bytes 0-2 only)
  frame.data[4] = 0x00;                 // not stored
  frame.data[5] = 0x00;                 // not stored
  frame.data[6] = BRAKES4_counter;      // rolling counter - harmless, never read by the 0AY
  BRAKES4_crc = 0;
  for (uint8_t i = 0; i < 7; i++)
  {
    BRAKES4_crc ^= frame.data[i];
  }
  frame.data[7] = BRAKES4_crc; // XOR over data[0..6] - harmless, never read by the 0AY
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
  frame.data[0] = 0x00; // flags: b0.3 Kupplungsschalter (clutch pedal) = 0 released
  frame.data[1] = 0x20; // inneres_Motor_Moment (inner torque) small, not 0xFF
  frame.data[2] = 0x80; // Motordrehzahl low  (0x0C80 x 0.25 = 800 rpm idle)
  frame.data[3] = 0x0C; // Motordrehzahl high
  frame.data[4] = 0x20; // inneres_Motor_Moment_ohne_extern - never read by the 0AY
  frame.data[5] = 0x00; // Fahrpedalwert (accelerator pedal) 0 %
  frame.data[6] = 0x10; // mechanisches_Motor_Verlustmoment (loss torque), not 0xFF
  frame.data[7] = 0x00; // Fahrerwunschmoment (driver-wish torque)
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
  frame.data[0] = 0x00;             // Giergeschw low (yaw rate) = 0
  frame.data[1] = 0x00;             // Giergeschw high bits 0-5 = 0, b1.7 sign = 0
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
  // NOT received by the 0AY (no message object - spec_gen5_0ay §1): its block
  // defaults OFF in standalone (V11.00.0), so standaloneTx() skips it.
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
  twai_message_t frame = {};
  // PQ Bremse_2 (0x5A0, DLC 8) - ESP/ABS sensor broadcast (vw_pq.dbc: Bremse_2).
  // The 0AY reads b0 (lateral accel, 0x7F = 0 g), b1.0 (mux) and b7.7 -
  // spec_gen5_0ay §5: b0 = 0x7F, b1.0 = 0, b7.7 = 0. Before V11: b0 0x80
  // (+0.01 g) and b7 0xAB (b7.7 = 1). Byte names corrected from vw_pq.dbc.
  frame.identifier = BRAKES2_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x7F;            // Querbeschleunigung (lateral accel) = 0 g
  frame.data[1] = 0x7A;            // b1.0 = 0 (lateral-accel mux), bits 1-7 mittlere_Raddrehzahl low
  frame.data[2] = 0x05;            // mittlere_Raddrehzahl high (mean wheel speed, not read)
  frame.data[3] = BRAKES2_counter; // Zeitstempel low (timestamp, not read)
  frame.data[4] = 0x7F;            // Zeitstempel high (not read)
  frame.data[5] = 0xCA;            // Wegimpulse_Vorderachse low (not read)
  frame.data[6] = 0x1B;            // Wegimpulse high / status bits (not read)
  frame.data[7] = 0x2B;            // Impulszahl bits 0-5; b7.7 gemessene_Querbeschleunigung = 0
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
  frame.data[0] = 0x00; // mux 0 (MO2_CAN_Vers) - not the max-torque mux 3
  frame.data[1] = 0x30; // MO2_Kuehlm_T (coolant temp * 0.75 - 48 degC)
  frame.data[2] = 0x00; // b2.0 MO2_BLS = 0 (set below), b2.1 BTS, b2.4 MO2_Sta_No_Bet = 0
  if (brakeForHaldex())
  {
    frame.data[2] = frame.data[2] | (1 << 0); // b2.0 MO2_BLS - brake pedal pressed
  }
  frame.data[3] = 0x0A; // MO2_Sta_GRA / TSK bits
  frame.data[4] = 0x0A; // MO2_GRA_Soll (cruise target speed)
  frame.data[5] = 0x10; // MO2_LL_Solldz (idle target, 0x10 x 10 rpm)
  frame.data[6] = 0xFE; // MO2_Begr_Mo
  frame.data[7] = 0xFE; // MO2_Mo_ZWR
  standaloneTx(frame);

  // PQ Motor_5 (0x480, DLC 8) - engine broadcast (vw_pq.dbc: Motor_5, multiplexed).
  // The 0AY only latches b0 once, when the mux (bits 6-7) = 2 (spec_gen5_0ay §3).
  // 0xFE = mux 3, so nothing is latched. Must be present: if Motor_5 is missing
  // the unit sets the limp bit 0x2C0 b0.4 by itself (spec §6).
  frame.identifier = MOTOR5_ID;   // 0x480
  frame.data_length_code = 8;     // DLC 8
  frame.data[0] = 0xFE;           // MO5_Mp_Code mux 3 (bits 6-7); from car testing: ASR 0x04 sets bit 4, 0x08 removes it
  frame.data[1] = 0x00;           // was 0x00; 0x41? 0x43? tried - MO5 lamps (Vorgluehen / E_Gas / OBD_2)
  frame.data[2] = 0x00;           // was 0x00 no effect - MO5_Verbrauch low
  frame.data[3] = 0x00;           // was 0xFE no effect - MO5_Verbrauch high
  frame.data[4] = 0x00;           // was 0xFE no effect - MO5_Luefter (cooling-fan PWM)
  frame.data[5] = 0x00;           // was 0xFE no effect - lamp / status bits
  frame.data[6] = 0x00;           // was 0x00 - status bits
  frame.data[7] = MOTOR5_counter; // checksum / rolling counter (not read by the 0AY)
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
  frame.data[0] = 0x00; // GE1 flags (Schaltung, Sta_Schutz, WK...) all 0 - b0 bits 3-4 not used by the 0AY
  frame.data[1] = 0x00; // GE1_Zielgang (bits 0-3) = 0 disengaged; GE1_Wahl_Pos (bits 4-7) not read
  frame.data[2] = 0x0A; // GE1_Mrad_Mkurb wheel/crank torque ratio 1.0 (not read by the 0AY)
  frame.data[3] = 0x00; // GE1_Soll_Mo (not read)
  frame.data[4] = 0x7F; // GE1_Fahrwistd about 0 (not read)
  frame.data[5] = 0x00; // GE1_Notlauf = 0 no emergency, GE1_Kuehlung = 0 (not read)
  frame.data[6] = 0x00; // LaunchControl / Infobit / counter = 0 (not read)
  frame.data[7] = 0x00; // GE1_WaVerl_Mo (not read)
  standaloneTx(frame);
}

void Gen5_0AY_frames25()
{
  twai_message_t frame = {};
  // PQ Kombi_1 (0x320, DLC 8) - instrument-cluster broadcast (vw_pq.dbc: Kombi_1).
  // The 0AY reads b1.1 KO1_Handbremse (handbrake) only - spec_gen5_0ay §3.
  frame.identifier = mKombi_1;
  frame.data_length_code = 8;
  frame.data[0] = 0x24; // status bits (not read by the 0AY)
  frame.data[1] = 0x00; // b1.0 brake-fluid warning = 0, b1.1 handbrake = 0 (set below)
  if (handbrakeForHaldex())
  {
    frame.data[1] = frame.data[1] | (1 << 1); // b1.1 KO1_Handbremse - handbrake pulled
  }
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
  twai_message_t frame = {};
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
  // NOT received by the 0AY (no message object - spec_gen5_0ay §1): its block
  // defaults OFF in standalone (V11.00.0), so standaloneTx() skips it.
  frame.identifier = BRAKES11_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // no effect
  frame.data[1] = 0XC0; // mode/status
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
  twai_message_t frame = {};
  // PQ Kombi_2 (0x420, DLC 8) - cluster temps (vw_pq.dbc: Kombi_2).
  // NOT received by the 0AY (no message object - spec_gen5_0ay §1): its block
  // defaults OFF in standalone (V11.00.0), so standaloneTx() skips it.
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
  twai_message_t frame = {};
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
  twai_message_t frame = {};
  frame.extd = 0;
  frame.rtr = 0;

  // FORD_GEN42_WHEEL_SPEEDS_ID 0x4B0 (Bus1, 10 ms) — ABS/ESC wheel speeds.
  // FL / FR / RL / RR, each 16-bit big-endian. Encoding: (raw − 10 000) / 100 km/h.
  // Raw=0x2710 (10 000) = 0 km/h standing reference.
  // Inflate front wheels above rear to request proportional AWD engagement.
  // lock_target is refreshed every 100 ms by the frames100 dispatcher.
  // At 0 % lock: all four wheels at 0x2710 (balanced, no engagement requested).
  // At 100 % lock: front +~1 000 counts (~10 km/h simulated slip). [WHEEL_SPEEDS]
  {
    const uint8_t spd_adj = get_lock_target_adjusted_value(0xFA, false);
    const uint16_t front_delta = (uint16_t)spd_adj * 4U; // 0 to ~1 000 counts
    const uint16_t front_spd = 0x2710U + front_delta;    // FL / FR speed word
    const uint16_t rear_spd = 0x2710U;                   // RL / RR at reference
    frame.identifier = FORD_GEN42_WHEEL_SPEEDS_ID;
    frame.data_length_code = 8;
    frame.data[0] = (uint8_t)(front_spd >> 8);    // FL high byte
    frame.data[1] = (uint8_t)(front_spd & 0xFFU); // FL low byte
    frame.data[2] = (uint8_t)(front_spd >> 8);    // FR high byte
    frame.data[3] = (uint8_t)(front_spd & 0xFFU); // FR low byte
    frame.data[4] = (uint8_t)(rear_spd >> 8);     // RL high byte
    frame.data[5] = (uint8_t)(rear_spd & 0xFFU);  // RL low byte
    frame.data[6] = (uint8_t)(rear_spd >> 8);     // RR high byte
    frame.data[7] = (uint8_t)(rear_spd & 0xFFU);  // RR low byte
    standaloneTx(frame);
  }

  // FORD_GEN42_TORQUE_FLAGS_ID 0x200 (Bus1, 10 ms) — PCM torque and flags.
  // D0:D1 = PrplWhlTot_Tq_Actl (4 Nm/LSB, offset −131072 Nm, Motorola b7|16).
  // D2:D3 = PrplWhlTot_Tq_LimMn (minimum propulsion torque limit).
  // D4:D5 = PrplWhlTot_Tq_Rq    (requested propulsion torque).
  // D6 b6:b7 = BrkOnOffSwtch_D_Actl, D7 b0:b6 = ACCompressorDisp (%). [TorqueDataEngFlags]
  frame.identifier = FORD_GEN42_TORQUE_FLAGS_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x28U + get_lock_target_adjusted_value(0xD7, false); // Tq_Actl high byte: 0x28→0xFF (proportional to lock_target)
  frame.data[1] = 0xE0;                                                // Tq_Actl low byte (static)
  frame.data[2] = 0x27U + get_lock_target_adjusted_value(0xD8, false); // Tq_LimMn high byte: 0x27→0xFF
  frame.data[3] = 0x10;                                                // Tq_LimMn low byte (static)
  frame.data[4] = 0x27U + get_lock_target_adjusted_value(0xD8, false); // Tq_Rq high byte: 0x27→0xFF
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
  frame.data[6] = 0x80U | (0x1FU + get_lock_target_adjusted_value(0x20, false)); // MtrGen1Aout_Tq_Rq upper 6 bits: 0x1F (0 Nm) → 0x3F (~+813 Nm)
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
  //         Scaled 0–100 % proportional to lock_target. D0[2:0]=0x7 constant, BrakePedal_D=0.
  // D2:D3 = static 0x7530.  D4:D5 = static 0x0130.
  // D7 = 8-bit rolling counter (shared with 0x20F D8 and 0x211 D8).
  // DBC signals: AccelPedal_Pc b6|10 (0.1 %/LSB Motorola), BrakePedal_D b4|2. DLC=7. [ACCEL_BRAKE_STATUS]
  frame.identifier = FORD_GEN42_ACCEL_BRAKE_STATUS_ID;
  frame.data_length_code = 7;
  {
    const uint16_t raw_pedal = (uint16_t)get_lock_target_adjusted_value(0xFA, false) * 4U; // 0–1000 = 0–100.0 %
    frame.data[0] = 0x07U | (uint8_t)(((raw_pedal >> 8) & 0x03U) << 5);                    // AccelPedal[9:8]; BrakePedal=0; bit2:0=0x7
    frame.data[1] = (uint8_t)(raw_pedal & 0xFFU);                                          // AccelPedal[7:0]
  }
  frame.data[2] = 0x75;
  frame.data[3] = 0x30; // D3:D4 = 0x7530 (constant)
  frame.data[4] = 0x01;
  frame.data[5] = 0x30;               // D5:D6 = 0x0130 (constant)
  frame.data[6] = gen42_main_counter; // D7 = 8-bit rolling counter
  standaloneTx(frame);

  // FORD_GEN42_ENGINE_DATA_ID 0x090 (Bus1, 20 ms) — PCM engine speed data.
  // D1 high nibble = 4-bit rolling counter 0x0-0xF (D1 = (counter << 4) | 0x07).
  // DBC signal: EngAout_N_Actl b35|13 (2 rpm/LSB, Motorola). D7:D8 = 0x07D0 (constant). [ENGINE_DATA]
  frame.identifier = FORD_GEN42_ENGINE_DATA_ID;
  frame.data_length_code = 8;
  frame.data[0] = (uint8_t)((gen42_090_nibble << 4) | 0x07U); // D1: 4-bit counter | 0x07
  frame.data[1] = 0x05;                                       // D2 (constant)
  frame.data[2] = 0x57;                                       // D3 (constant — EngAout_N upper bits)
  frame.data[3] = 0x07;                                       // D4 (constant)
  {
    const uint16_t V_rpm = 400U + (uint16_t)get_lock_target_adjusted_value(0xFA, false) * 10U; // 400–2900 → 800–5800 RPM
    frame.data[4] = (uint8_t)(0xC0U | ((V_rpm >> 8) & 0x1FU));                                 // D5: upper 5 bits of V; flags 0b110 preserved
    frame.data[5] = (uint8_t)(V_rpm & 0xFFU);                                                  // D6: lower 8 bits of V
  }
  frame.data[6] = 0x07;
  frame.data[7] = 0xD0; // D7:D8 = 0x07D0 (constant)
  standaloneTx(frame);

  // FORD_GEN42_BRAKE_DATA_ID 0x20F (Bus1, 20 ms) — ABS provisional brake data.
  // D1:D2 = 0x7530 (constant). D3:D4 = 0x2710 (wheel speed reference, constant).
  // D5 = 0x40 (constant). D6 = gen42_20F_d6 (+0x10 per frame, mod 256).
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
  frame.data[5] = gen42_20F_d6;                    // D6: +0x10 per frame, mod 256
  frame.data[6] = (uint8_t)(0xC8U - gen42_20F_d6); // D7: complement (D6 + D7 = 0xC8)
  frame.data[7] = gen42_main_counter;              // D8: shared 8-bit rolling counter
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
  frame.data[2] = get_lock_target_adjusted_value(0xFF, false); // RearDiffLck_Tq_RqMx upper 8 bits: 0→255 Nm (proportional to lock_target)
  frame.data[3] = 0x00;                                        // RearDiffLck_Tq_RqMx lower 4 bits = 0; AbsActv_B_Actl = 0
  frame.data[4] = 0x48;
  frame.data[5] = 0x48;               // VehLongOvrGnd_A_Est (static)
  frame.data[6] = 0x00;               // constant
  frame.data[7] = gen42_main_counter; // D8: shared 8-bit rolling counter
  standaloneTx(frame);

  // FORD_GEN42_DRIVE_MODE_ID 0x190 (Bus1, 20 ms) — GWM/PCM drive mode status.
  // D3 = drive-mode byte: 0x60 when engagement is requested, 0x20 at idle/off.
  //      This is the primary proactive signal — Haldex engages BEFORE slip occurs.
  // D2 = 0x51 (engine speed proxy, ~1000 rpm idle; varies with throttle in logs).
  // D6 = 4-bit nibble counter 0x00-0x0F (gen42_190_counter).
  // D7:D8 = provisional steering angle word (0x8000 = 0°, per guide). [PROVISIONAL_0x190]
  {
    const uint8_t drive_mode = (lock_target > 0.0f) ? 0x60U : 0x20U;
    frame.identifier = FORD_GEN42_DRIVE_MODE_ID;
    frame.data_length_code = 8;
    frame.data[0] = 0x00;                      // D1 (constant)
    frame.data[1] = 0x51;                      // D2: engine speed proxy (~1000 rpm idle)
    frame.data[2] = drive_mode;                // D3: drive-mode (0x20=idle, 0x60=demand)
    frame.data[3] = 0x80;                      // D4 (constant)
    frame.data[4] = 0x04;                      // D5 (constant — drive-engaged status)
    frame.data[5] = gen42_190_counter & 0x0FU; // D6: 4-bit nibble counter 0x00-0x0F
    frame.data[6] = 0x00;                      // D7 (constant)
    frame.data[7] = 0x3C;                      // D8 (constant)
    standaloneTx(frame);
  }

  // Advance counters after all 20 ms frames are sent.
  gen42_main_counter++;                                            // free-running 8-bit (0x080/0x20F/0x211 shared)
  gen42_090_nibble = (uint8_t)((gen42_090_nibble + 1U) & 0x0FU);   // 4-bit 0-15
  gen42_20F_d6 = (uint8_t)((gen42_20F_d6 + 0x10U) & 0xFFU);        // +0x10, mod 256
  gen42_190_counter = (uint8_t)((gen42_190_counter + 1U) & 0x0FU); // 4-bit 0-15
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
  frame.data[0] = gen42_275_counter; // D1: rolling counter (+0x20 per frame)
  frame.data[1] = 0x00;              // D2 (constant)
  frame.data[2] = 0xFF;              // D3 (constant)
  standaloneTx(frame);

  gen42_275_counter = (uint8_t)((gen42_275_counter + 0x20U) & 0xFFU);
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
    static uint8_t gen42_420_toggle = 0U;
    frame.identifier = FORD_GEN42_PTRAIN_DATA1_ID;
    frame.data_length_code = 8;
    frame.data[0] = (gen42_420_toggle & 1U) ? 0x63U : 0x62U; // D1 toggle
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x5D;
    frame.data[4] = 0x43; // TrnTotLss_Tq_Est / static
    frame.data[5] = 0x10;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    standaloneTx(frame);
    gen42_420_toggle++;
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
// scaled down by get_lock_target_adjusted_value() (0 when lock is off).
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
// Haldex replies on VOLVO_P2G2_HALDEX_ID (0x0120000A), decoded in parseCAN_hdx.

// Static frames: starting values, byte meanings not known yet
static const uint8_t VOLVO_P2G2_224024_DATA[8] = {0xE9, 0xC9, 0x00, 0x1F, 0x01, 0xFF, 0x08, 0x00};  // 0x00224024
static const uint8_t VOLVO_P2G2_C24000_DATA[8] = {0x40, 0x00, 0x00, 0x00, 0xC1, 0x42, 0xC1, 0x42};  // 0x00C24000
static const uint8_t VOLVO_P2G2_1E2402C_DATA[8] = {0xC4, 0xF9, 0x20, 0xC6, 0x44, 0xF6, 0xAE, 0x4D}; // 0x01E2402C
static const uint8_t VOLVO_P2G2_3200002_DATA[8] = {0x04, 0xC3, 0x62, 0xAE, 0x01, 0xF4, 0x01, 0xF4}; // 0x03200002
static const uint8_t VOLVO_P2G2_3400002_DATA[8] = {0x01, 0x00, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00}; // 0x03400002

void VolvoP2Gen2_frames10()
{
  uint8_t data[8];

  // lock request: 0 = no lock, 0xF4 = the strongest value seen (100 %)
  const uint8_t lockByte = get_lock_target_adjusted_value(0xF4, false);
  const bool isLockActive = (lockByte > 0);

  sendVolvoSaabFrame(VOLVO_P2G2_224024_ID, true, VOLVO_P2G2_224024_DATA);

  // 0x0042401E - carries the lock request
  data[0] = 0xC4; // static
  data[1] = 0x00; // static
  data[2] = 0x20; // 0x20 normally, 0x21 only at full lock
  if (isLockActive && lock_target >= 95.0f)
  {
    data[2] = 0x21;
  }
  data[3] = lockByte; // lock request
  data[4] = 0x45;     // static
  data[5] = 0xF4;     // static
  data[6] = 0xAE;     // static
  data[7] = 0x4C;     // static
  sendVolvoSaabFrame(VOLVO_P2G2_LOCK_ID, true, data);

  // 0x0062401E - "lock active" flags in bytes 6-7
  data[0] = 0xC0; // static
  data[1] = 0x00; // static
  data[2] = 0xD8; // static
  data[3] = 0x00; // static
  data[4] = 0x68; // static
  data[5] = 0x11; // static
  data[6] = 0x60; // 0x60 idle, 0x67 lock active
  data[7] = 0x00; // 0x00 idle, 0xD0 lock active
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
  data[0] = 0x01; // page 1 (the real car multiplexes this frame by byte 0)
  data[1] = 0x1F; // static
  data[2] = 0x49; // static
  data[3] = 0xDE; // static
  data[4] = 0x38; // static
  data[5] = 0x08; // static
  data[6] = 0xFE; // FE F8, then 01 07 on the next send
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
// Haldex replies on VOLVO_P2G3_HALDEX_ID (0x1020000A), decoded in parseCAN_hdx.

// V11.00.0 (OEM code: spec_volvo_p2_s60r.md): the S60R DEM's own message table
// confirms every ID here (29-bit, all DLC 8). It runs a closed-loop slip +
// engine-torque strategy (same family as the VAG Gen2 unit), so there is no
// single "ceiling" byte: our lever is LOCK 0x0062401E byte 1 plus the
// ACTIVE / MODE / LOCK "active" bytes, all switched together (bench-proven).
// "500k S60 log" = the public log volvo-CAN-exploration/0000286.TXT (500 kbit/s,
// model year not stated, carries these facelift IDs). It gave us the counters
// below and the content of four frames the DEM listens for that we never sent.
// The 2002/2003 P2 logs are pre-facelift (250 kbit/s, other IDs), not used here.

// Static frames: starting values, byte meanings not known yet
static const uint8_t VOLVO_P2G3_42406C_DATA[8] = {0x80, 0x88, 0x00, 0x71, 0x60, 0x00, 0xC1, 0xF3}; // 0x0042406C (byte 0 is replaced by the counter)
static const uint8_t VOLVO_P2G3_224024_DATA[8] = {0x3F, 0xF8, 0x00, 0x00, 0x13, 0xFF, 0xC0, 0x00}; // 0x00224024 (byte 0 bit 2 is toggled)
static const uint8_t VOLVO_P2G3_E24026_DATA[8] = {0x00, 0x01, 0xE0, 0xE4, 0x9F, 0x08, 0x00, 0x00}; // 0x00E24026

// V11.00.0: frames the DEM subscribes to (OEM code: MO table) that we never sent.
// Steady values from the 500k S60 log - what the bytes mean is not known.
static const uint8_t VOLVO_P2G3_C0402A_DATA[8] = {0x1E, 0xE4, 0x00, 0x00, 0x27, 0xFF, 0xD3, 0xFF};   // 0x00C0402A (bytes 4/6 move with the gear lever in the log)
static const uint8_t VOLVO_P2G3_10E24000_DATA[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 0x10E24000 (all zero for the whole log)
static const uint8_t VOLVO_P2G3_11220028_DATA[8] = {0x23, 0x60, 0x0F, 0x3C, 0x01, 0x00, 0x35, 0x2E}; // 0x11220028 (bytes 6-7 drift slowly in the log)
static const uint8_t VOLVO_P2G3_11420006_DATA[8] = {0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0xC0, 0x00}; // 0x11420006
// 0x01A2402A is also in the DEM's table but never appears in the log, so it is not sent.
// V11 TODO (bench): if a scan shows a "no communication" fault for it, capture a car.

// static = these counters belong to this file only
static uint8_t volvoP2G3Counter42406C = 0; // 0x0042406C byte 0 bits 6-7: counts 0-3, one step per send (500k S60 log: 00 40 80 C0)
static bool volvoP2G3Toggle = false;       // flips every 10 ms: 0x00224024 byte 0 bit 2 and 0x0080401E byte 0 (1E <-> 06)
static uint8_t volvoP2G3Slow = 0;          // counts 10 ms sends, 0-4, so the 56/60 ms frames go out every 50 ms

void VolvoP2Gen3_frames10()
{
  uint8_t data[8];

  // lock request: 0 = no lock, 0xBC = the strongest value seen on the bench (100 %).
  // lever_lock_percent() already gives 0 while a lock gate holds lock off, and
  // the learn sweep position while Learn runs. Straight line 0..0xBC (spec section 2).
  // V11 TODO (bench): the OEM code does not prove 0xBC is exactly full scale.
  const uint8_t lockPercent = lever_lock_percent();
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
  data[0] = 0xC0; // static
  data[1] = 0x00; // static
  data[2] = 0x00; // static
  data[3] = 0x00; // static
  data[4] = 0xC0; // C0 00 C0 00 idle, C1 42 C1 42 lock active
  data[5] = 0x00;
  data[6] = 0xC0;
  data[7] = 0x00;
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
  data[0] = (uint8_t)((VOLVO_P2G3_42406C_DATA[0] & 0x3F) | (volvoP2G3Counter42406C << 6)); // byte 0: counter in bits 6-7, low bits as captured
  sendVolvoSaabFrame(VOLVO_P2G3_42406C_ID, true, data);
  volvoP2G3Counter42406C = (volvoP2G3Counter42406C + 1) & 0x03; // next count, 0-3

  // 0x00C0402A - V11.00.0 new: the DEM listens for it (OEM code: MO table),
  // 10 ms in the 500k S60 log, sent with that log's steady bytes
  sendVolvoSaabFrame(VOLVO_P2G3_C0402A_ID, true, VOLVO_P2G3_C0402A_DATA);

  // 0x0062401E - carries the lock request.
  // V11 TODO (bench): in the 500k S60 log this frame comes from the engine side
  // (byte 0 = 0x02 most of the time, byte 1 moves a little either side of 0,
  // so byte 1 is probably an engine-torque signal the DEM feeds forward).
  data[0] = 0x00;     // 0x00 idle, 0x02 lock active
  data[1] = lockByte; // lock request
  data[2] = 0x42;     // static
  data[3] = 0xEE;     // static
  data[4] = 0xE8;     // E8 00 64 00 idle, E9 F4 65 F4 lock active
  data[5] = 0x00;
  data[6] = 0x64;
  data[7] = 0x00;
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
  data[0] = 0x1E; // 1E and 06 every other send (500k S60 log). V11.00.0: was a frozen 1E
  if (volvoP2G3Toggle)
  {
    data[0] = 0x06;
  }
  data[1] = 0x03; // static
  data[2] = 0x80; // 80 00 idle, 87 D0 lock active
  data[3] = 0x00;
  data[4] = 0x00; // static
  data[5] = 0x19; // static
  data[6] = 0x72; // static
  data[7] = 0x00; // static
  if (isLockActive)
  {
    data[2] = 0x87;
    data[3] = 0xD0;
  }
  sendVolvoSaabFrame(VOLVO_P2G3_MODE_ID, true, data);

  // 0x00224024 - bench-captured bytes; byte 0 bit 2 flips every send like the
  // real car (500k S60 log: 1C 18 1C 18 ...). V11.00.0: was a frozen 0x3F.
  for (uint8_t i = 0; i < 8; i++)
  {
    data[i] = VOLVO_P2G3_224024_DATA[i]; // copy the starting values
  }
  if (volvoP2G3Toggle)
  {
    data[0] = (uint8_t)(VOLVO_P2G3_224024_DATA[0] & ~(1 << 2)); // byte 0 bit 2 = 0 this time
  }
  else
  {
    data[0] = (uint8_t)(VOLVO_P2G3_224024_DATA[0] | (1 << 2)); // byte 0 bit 2 = 1 this time
  }
  sendVolvoSaabFrame(VOLVO_P2G3_224024_ID, true, data);

  sendVolvoSaabFrame(VOLVO_P2G3_E24026_ID, true, VOLVO_P2G3_E24026_DATA);

  // V11.00.0 new: 0x10E24000 (56 ms in the log) and 0x11220028 (60 ms in the
  // log) go out on every 5th 10 ms send = 50 ms - a bit faster than the car,
  // which can never trip the DEM's "frame missing" timer.
  volvoP2G3Slow++;
  if (volvoP2G3Slow >= 5)
  {
    volvoP2G3Slow = 0; // start counting again
    sendVolvoSaabFrame(VOLVO_P2G3_10E24000_ID, true, VOLVO_P2G3_10E24000_DATA);
    sendVolvoSaabFrame(VOLVO_P2G3_11220028_ID, true, VOLVO_P2G3_11220028_DATA);
  }
}

void VolvoP2Gen3_frames100()
{
  uint8_t data[8];

  // 0x10A2407C - network heartbeat: bytes 6-7 swap to their exact opposite on each send
  data[0] = 0x01; // page 1
  data[1] = 0x1F; // static
  data[2] = 0xB8; // static
  data[3] = 0xFA; // static
  data[4] = 0xC2; // static
  data[5] = 0x7A; // static
  data[6] = 0xFE; // FE F5, then 01 0A on the next send
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
  sendVolvoSaabFrame(VOLVO_P2G3_11420006_ID, true, VOLVO_P2G3_11420006_DATA);
}

// --- Volvo Gen4 (P3 family, 500 kbit/s, 11-bit IDs) --------------------------
// Haldex replies on VOLVO_G4_HALDEX_ID (0x243), decoded in parseCAN_hdx.
// Only three lock steps are known here, so the request is stepped.

// Static frames: starting values, byte meanings not known yet
static const uint8_t VOLVO_G4_20A_DATA[8] = {0xFD, 0x00, 0x94, 0x00, 0x02, 0x00, 0x00, 0x00}; // 0x20A
static const uint8_t VOLVO_G4_220_DATA[8] = {0x8B, 0x11, 0xF0, 0x00, 0x80, 0x00, 0x80, 0x00}; // 0x220

void VolvoGen4_frames10()
{
  uint8_t data[8];

  // lock request as 0-100 %, after the learn table / lock-enable rules
  const uint8_t lockPercent = get_lock_target_adjusted_value(100, false);

  // 0x12A - EngineData (volvo_v60_2015_pt.dbc): byte 4 bit 6 = brake pressed,
  // engine RPM = 13 bits from byte 6 bits 4-0 and byte 7. The three lock steps
  // below are really three engine-speed values (842 / 700 / 1980 rpm), with
  // 0 rpm for no lock. Next step: drive lock from the pedal (0x020) instead.
  data[0] = 0xE9; // static
  data[1] = 0xFE; // static
  data[2] = 0xC7; // static
  data[3] = 0xF6; // static
  data[4] = 0x98; // static (bit 6 = 0: brake not pressed)
  data[5] = 0x04; // static
  if (lockPercent == 0)
  {
    data[6] = 0xE0; // 0 rpm
    data[7] = 0x00;
  }
  else if (lockPercent >= 67)
  {
    data[6] = 0xE3; // 0x34A = 842 rpm (strongest step)
    data[7] = 0x4A;
  }
  else if (lockPercent >= 34)
  {
    data[6] = 0xE2; // 0x2BC = 700 rpm (middle step)
    data[7] = 0xBC;
  }
  else
  {
    data[6] = 0xE7; // 0x7BC = 1980 rpm (lightest step)
    data[7] = 0xBC;
  }
  sendVolvoSaabFrame(VOLVO_G4_MODE_ID, false, data);
}

void VolvoGen4_frames20()
{
  sendVolvoSaabFrame(VOLVO_G4_20A_ID, false, VOLVO_G4_20A_DATA);
  sendVolvoSaabFrame(VOLVO_G4_220_ID, false, VOLVO_G4_220_DATA);
}

void VolvoGen4_frames100()
{
  uint8_t data[8];

  // 0x405 - network heartbeat: bytes 6-7 swap to their exact opposite on each send
  data[0] = 0x01; // page 1
  data[1] = 0x67; // static
  data[2] = 0x79; // static
  data[3] = 0x04; // static
  data[4] = 0x6C; // static
  data[5] = 0xB7; // static
  data[6] = 0xF4; // F4 45, then 0B BA on the next send
  data[7] = 0x45;
  if (volvoHeartbeatFlip)
  {
    data[6] = 0x0B;
    data[7] = 0xBA;
  }
  volvoHeartbeatFlip = !volvoHeartbeatFlip;
  sendVolvoSaabFrame(VOLVO_G4_NETWORK_ID, false, data);
}

// --- Volvo Gen5 "SPC" 31380867 (generation 53, 500 kbit/s assumed, 11-bit) ---
// Added V11.00.0 from the unit's OEM code (spec_ppc_volvo_spc_bmw.md, Volvo part).
// Haldex replies on VOLVO_SPC_HALDEX_ID (0x126), decoded in parseCAN_hdx.
//
// The lever is known exactly: 0x041 bytes 6-7, signed 16-bit, high byte first.
// The unit takes the size of the number (ignores the sign): 0 = clutch fully
// open, bigger = more lock, and past its calibrated full scale = full lock.
// No checksum or counter is checked on anything it receives.
//
// What is NOT known yet: the content of the other 23 frames it listens to
// (ignition / status / variant inputs that gate the strategy - if one is
// missing or wrong the unit may stay open) and every frame's real send rate.
// They are sent as all-zero starting values so each byte can be tried live with
// the serial lab (OVR <id> <byte> <value>); set VOLVO_SPC_SEND_PLACEHOLDERS to 0
// to send only the lever and the network frame.
// V11 TODO (bench): rates below are guesses (0x0xx 10 ms, 0x1xx/0x2xx 20 ms,
// 0x5xx 100 ms); confirm content + rates from a log of a car with this unit.
// No brake / handbrake bit is known in these frames, so none is sent.

static const uint8_t VOLVO_SPC_ZERO_DATA[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // starting value for the unknown frames

void VolvoSpc_frames10()
{
  uint8_t data[8];

  // lock request 0-100 % (0 while a lock gate holds lock off, the sweep
  // position while Learn runs) -> 0 .. VOLVO_SPC_LEVER_FULL_SCALE
  const uint8_t lockPercent = lever_lock_percent();
  const uint32_t leverValue = ((uint32_t)VOLVO_SPC_LEVER_FULL_SCALE * lockPercent + 50) / 100; // round to nearest

  // 0x041 - clutch demand (OEM code: COM signal 12, bytes 6-7, signed 16-bit high byte first)
  data[0] = 0x00; // not read by the unit
  data[1] = 0x00; // not read by the unit
  data[2] = 0x00; // not read by the unit
  data[3] = 0x00; // not read by the unit
  data[4] = 0x00; // not read by the unit
  data[5] = 0x00; // not read by the unit
  data[6] = (uint8_t)((leverValue >> 8) & 0xFF); // clutch demand high byte (positive, so the top bit stays 0)
  data[7] = (uint8_t)(leverValue & 0xFF);        // clutch demand low byte
  sendVolvoSaabFrame(VOLVO_SPC_LEVER_ID, false, data);

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
    sendVolvoSaabFrame(VOLVO_SPC_235_ID, false, VOLVO_SPC_ZERO_DATA); // starting value, content unknown
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
// Haldex replies on SAAB_93_HALDEX_ID (0x210), decoded in parseCAN_hdx.
// Uses the GM "alive rolling count" idea, like our Gen41: a 2-bit counter, and
// on 0x160 a protection byte that is 0 minus the counter.

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

void Saab93_frames10()
{
  uint8_t data[8];

  // lock request: 0 = no lock, 0x89 = the strongest value seen (100 %)
  const uint8_t lockByte = get_lock_target_adjusted_value(0x89, false);
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
// frame-edit block, so the ones it turns out not to need can be switched off.
//
// What goes in them:
//   - Steady bytes: a real frame of each, taken from Drive.csv at 56.9 s
//     (30 km/h, light throttle, no brake, wheel straight).
//   - Counters / toggles: copied from the full C30 PCAN trace (idle.trc), which
//     caught every frame, so the patterns are exact.
//   - The lock: chosen on the Settings page (Volvo P1 Standalone card, saved):
//       Engine Torque - torque + pedal + rpm rise with the lock. In the logs the
//                       module follows torque almost 1:1, so this is the default.
//       Wheel Slip    - the front wheels run faster than the rears (wheelspin).
//       Both          - the two together.
//     lever_lock_percent() gives the lock % (Learn sweep, learn table and every
//     lock gate already applied), exactly as on the other direct-lever units.
//   - Speed: the Road Speed setting on the rear wheels and the road speed frame;
//     the fronts the same unless Wheel Slip is in use.
//   - Steering: straight ahead and still, with the sensor's own check byte.
//   - Brake / handbrake: from Follow Brake / Follow Handbrake.
// The AWD module answers on 0x1960000A, decoded in parseCAN_hdx.
//
// Send times: 10 ms frames go out every 10 ms, the car's 14-15 ms frames every
// 13 ms, 20 ms every 20 ms, 28 ms every 25 ms, 50 ms every 50 ms - never slower
// than the car. The slow ones (80 ms and up) count 10 ms steps.
// =============================================================================

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

// Front wheel speed to send, 0.01 km/h: the rears plus the Wheel Slip share of the lock
static uint16_t volvoP1FrontSpeedRaw(uint8_t lockPercent)
{
  uint16_t slipRaw = volvoP1Scale(0, (uint16_t)volvoP1SlipFullKmh * 100, volvoP1SlipLockPercent(lockPercent));
  return volvoP1RearSpeedRaw() + slipRaw;
}

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
  uint8_t pedal = (uint8_t)volvoP1Scale(0, VOLVO_P1_PEDAL_FULL, enginePercent);

  data[0] = 0x90 | ((torque >> 8) & 0x0F); // top nibble steady (9), bottom = torque top 4 bits
  data[1] = torque & 0xFF;                 // torque bottom 8 bits
  data[4] = 0xA0 | ((torque >> 8) & 0x0F); // top nibble A = brake off, bottom = second torque top 4 bits
  data[5] = torque & 0xFF;                 // second torque bottom 8 bits (the car keeps the two within a few counts)
  data[7] = pedal;                         // accelerator pedal, 0-255

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
  const uint8_t lockPercent = lever_lock_percent();

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
  const uint8_t lockPercent = lever_lock_percent();
  uint16_t rearRaw = volvoP1RearSpeedRaw();             // Road Speed setting
  uint16_t frontRaw = volvoP1FrontSpeedRaw(lockPercent); // the same, plus any Wheel Slip
  volvoP1SentRearRaw = rearRaw;   // for the live readout
  volvoP1SentFrontRaw = frontRaw;

  // 0x01C04026 - four wheel speeds, 16-bit, 0x8000 = stopped, order RL RR FL FR
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
  data[6] = (rearRaw >> 8) & 0xFF; // speed high byte
  data[7] = rearRaw & 0xFF;        // speed low byte
  sendVolvoSaabFrame(VOLVO_P1_SPEED_ID, true, data);

  // 0x02000026 - brake pressure (byte 5 + byte 4 bits 6-7). Released = as logged.
  volvoP1Copy(data, VOLVO_P1_02000026_DATA);
  if (brakeForHaldex())
  {
    data[2] = 0xAD; // braking: a real pair of values from the wheelspin log
    data[5] = 0x35; // brake pressure byte (released = 0x00)
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
  const uint8_t lockPercent = lever_lock_percent();

  // 0x0090411E - engine rpm follows the lock (Engine Torque / Both); byte 0 bit 7 on every 10th send
  volvoP1Copy(data, VOLVO_P1_0090411E_DATA);
  uint16_t rpm = volvoP1Scale(VOLVO_P1_RPM_AT_ZERO, VOLVO_P1_RPM_FULL, volvoP1EngineLockPercent(lockPercent));
  volvoP1SentRpm = rpm; // for the live readout
  data[2] = 0xE0 | ((rpm >> 8) & 0x1F); // top 3 bits steady, bottom 5 = rpm top bits
  data[3] = rpm & 0xFF;                 // rpm bottom 8 bits
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
