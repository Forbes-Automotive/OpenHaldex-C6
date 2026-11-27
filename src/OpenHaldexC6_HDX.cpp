#include <OpenHaldexC6_can.h>
#include <OpenHaldexC6_Calculations.h>
#include <OpenHaldexC6_Analyzer.h>
#include <OpenHaldexC6_UDS.h>

// =============================================================================
// Bus 1 - Haldex (V11.00.14: split out of OpenHaldexC6_can.cpp)
// =============================================================================
// Every frame the Haldex sends: decode its feedback for the active generation
// (hdxDecodeFeedback - one case per generation, one function per unit), hand
// diagnostic replies to our UDS / TP2.0 tasks, and pass the rest to Bus 0.
// Different generations have different ranges of 'applied lock', so each one
// scales its own. Health flags (fault / over-temp / limp / lamp ...) are decoded
// per generation too - the bit layout is different on every unit (OEM code
// findings, CONTROL_AND_DTC_SPEC.md).
// =============================================================================

// ---- Statics -----------------------------------------------------------------

// Gen5 0CQ (V11.00.0): the "fault reaction active" bit from the unit's own status
// frame 0x17F0000F (byte 0 bit 0, OEM code: spec_gen5_0cq §6). That frame comes
// far less often than Allrad_03, so the last value is kept here and merged into
// the health flags on every Allrad_03. 'static' keeps these private to this file.
static bool gen5StatusFaultReaction = false; // 0x17F0000F byte 0 bit 0 as last received
static uint32_t gen5StatusFrameMs = 0;       // millis() of the last 0x17F0000F (0 = never)

// ---- Small helpers -----------------------------------------------------------

// Remember the raw frame the engagement decoder just accepted (serial lab FB line).
static void hdxFbCapture(const twai_message_t &m)
{
  hdxFbId = m.identifier;
  hdxFbDlc = m.data_length_code;
  for (uint8_t i = 0; i < 8; i++)
    hdxFbData[i] = (i < m.data_length_code) ? m.data[i] : 0;
  hdxFbMs = millis();
}

// Clear every Haldex health flag (V11.00.0). Each generation's feedback decode
// in parseCANHdx() calls this first, then sets only the bits its own frame
// really carries - so a flag can never be left over from another generation.
static void clearHaldexHealth()
{
  receivedReportClutch1 = false;    // fault present
  receivedReportClutch2 = false;    // plausibility fault
  receivedTempProtection = false;   // over-temperature protection
  receivedCouplingOpen = false;     // clutch commanded fully open (status)
  receivedLimpMode = false;         // limp / fault reaction
  receivedAwdWarning = false;       // AWD warning lamp
  receivedSpeedLimit = false;       // speed limit request
  receivedTempSensorFault = false; // temperature sensor fault
  receivedFaultStored = false;      // fault stored in the unit's memory
}

// ---- Setup -------------------------------------------------------------------

void setupHDX()
{
  canInstallBus(1, CAN1_TX, CAN1_RX, &twaiBus1); // controller 1 = Haldex
}

// ---- Feedback decode, one function per unit (called from hdxDecodeFeedback) -----

// PQ Allrad_1 (0x2C0) byte 0 with the vw_pq.dbc names was decoded in one shared
// block for Gen1/2/4/0AY before V11 (bit 0 clutch fault, 1 over-temp, 2 stiffness
// fault, 3 coupling open, 4 limp, 5 lamp, 6 speed limit). The OEM code shows each
// unit uses these bits differently, so each generation's case decodes its own.

static void hdxDecodeGen1(const twai_message_t &m)
{
  // Gen1 Haldex (02D900554B) Allrad_1 / 0x2C0, DLC 2 (OEM code: spec_gen1_02d §6).
  // Only decode 0x2C0 - before V11 every frame on the Haldex bus was decoded (D2).
  if (m.identifier == HALDEX_ID)
  {
    // Byte 1 = 0x80 + measured clutch pressure (E038, valve-command coding). 0x7F = released (very common - it
    // used to wrap round to 255 %, D1), 0x80 = no pressure, OEM top 0xDF.
    // 100 % = 198 (0xC6) = the measured full lock (bench and car). V11.00.13 had moved it to the modelled 201 (0xC9, audit F9),
    // which showed full lock as 95 % and kept Learn below 100 - back to 198 in V11.00.14 (2026-11-27).
    // [KM Allrad_1.Kupplungssteifigkeit_Mitte__Ist (3,-381) Nm/min] [REA audit_gen1_02d #62/#65/#66: 0x7F during release delay, else
    //  0x80 + E038 x 280/3000, linear in E258 with the lock request's 192 step; byte1 - 0x80 ~ E258/192]
    // Byte 2-4 [KM Allrad_1.PNG_* / Schaltung_* / Kupplungssteifigkeit_Hinten__Is] [REA audit_gen1_02d #9: not sent, the 02D's 0x2C0 is DLC 2]
    receivedHaldexEngagementRaw = m.data[1];
    int32_t engagementPercent = 0; // worked out in a signed number so it can't wrap
    if (receivedHaldexEngagementRaw >= 0x80)
    {
      engagementPercent = map(receivedHaldexEngagementRaw, 0x80, GEN1_FEEDBACK_FULL, 0, 100); // 0x80..0xC6 (198)
    }
    if (engagementPercent < 0)
    {
      engagementPercent = 0;
    }
    if (engagementPercent > 100)
    {
      engagementPercent = 100; // 199..223 is real extra clamping force - still shown as 100 %
    }
    receivedHaldexEngagement = (uint8_t)engagementPercent;
    receivedHaldexState = m.data[0];

    // Byte 0 status bits as the 02D's OEM code really sets them (spec §6).
    // Not the vw_pq.dbc names: bit 2 and bit 4 are NOT faults on this unit.
    clearHaldexHealth();
    if (receivedHaldexState & (1 << 0))
    {
      receivedReportClutch1 = true; // bit 0: unit is in its fault-reaction (suspended) mode... [KM Allrad_1.Fehler_Allrad_Kupplung b0.0] [REA audit_gen1_02d #60/#61: only the suspended builder FUN_013678 sets it]
      receivedLimpMode = true;      // ...with the clutch held open = limp
    }
    if (receivedHaldexState & (1 << 1))
    {
      receivedTempProtection = true; // bit 1: oil over-temperature (or sensor problem) [KM Allrad_1.Ubertemperaturschutz__Allrad_1_ b0.1] [REA audit_gen1_02d #55/#60: E1E2.5 && request 0]
    }
    // bit 2: our Bremse_4 ceiling active, request still above it - status, not a fault [KM Allrad_1.Fehlerstatus_Kupplungssteifigke b0.2] [REA audit_gen1_02d #59: 0x2A0 b0 >= 0x7F && E258 > E286]
    if (receivedHaldexState & (1 << 3))
    {
      receivedCouplingOpen = true; // bit 3: coupling fully open (no clutch pressure) [KM Allrad_1.Kupplung_komplett_offen b0.3] [REA spec_gen1_02d §6 / audit #60: E038 == 0]
    }
    // bit 4: "input signals incomplete" (a monitored frame missing / invalid) - status only,
    //        the 02D takes no action on it, so it is NOT limp mode (D3, D4) [KM Allrad_1.Notlauf b0.4] [REA audit_gen1_02d #60/#77: E1E4.1, no control reader]
    if (receivedHaldexState & (1 << 5))
    {
      receivedAwdWarning = true; // bit 5: AWD warning lamp [KM Allrad_1.Allrad_Warnlampe b0.5] [REA audit_gen1_02d #14/#60: E1E4.2, implausible > 1000 cycles, or suspended]
    }
    // bits 6-7: never set by the 02D (no speed-limit bit on Gen1) [KM Allrad_1.Geschwindigkeitsbegrenzung b0.6 / Einheit_der_Kupplungssteifigkei b0.7] [REA audit_gen1_02d #60: never written]
    receivedHaldexTempC = -128; // Gen1 does not report its temperature on CAN
    haldexHealthSupported = true;
    receivedLongLockState = 0; // not present on PQ Allrad_1
  }
}

static void hdxDecodeGen2(const twai_message_t &m)
{
  // Gen2 Haldex 7H0907554 / 1K0907554B (Allrad_1 / 0x2C0), V11.00.0 from the
  // OEM code (documents/haldex-binaries/leaf_trace/spec_gen2_7h0.md §6):
  //   byte 1 = engagement: min(E43C / 28 + 0x80, 0xFE), E43C = measured clutch-oil pressure - 3.8 bar.
  //            0x80 = open; the 40 bar sensor tops it at ~0xD7 (7H0) / ~0xEC (1K0), NOT ~0xFD
  //            [REA audit_gen2_7h0 #56/#57/#59 corrects the old "about 0xFD = full"]. Same 28-unit step
  //            as the Bremse_4 lock request. [KM Allrad_1.Kupplungssteifigkeit_Mitte__Ist (3,-381) Nm/min - DBC scale not used]
  //   byte 2, byte 3 bits0-1: no writer; byte 3 bit7 = sig47 (meaning not traced) [KM Allrad_1.PNG_* / Schaltung_*] [REA audit_gen2_7h0 #66/#67]
  //   byte 4 / byte 6 are NEVER written by the unit (always 0) - before V11
  //            we read byte 4 (vw_pq.dbc name), so the UI always showed 0 %.
  //            [KM Allrad_1.Kupplungssteifigkeit_Hinten__Is 0.7874 % (b4); b6 -] [REA audit_gen2_7h0 #68]
  //   byte 0 = status bits, see below. byte 5 bits 6-7 / byte 7 bits 0-2 = E480 status bits, byte 7 bits 4-7 =
  //            message counter - 7H0 only: the 1K0's 0x2C0 is DLC 4 (bytes 0-3) [KM -] [REA audit_gen2_7h0 #4/#70/#71/F6].
  // Only decode the Allrad_1 frame; the module also sends 0x081 and diagnostic IDs.
  if (m.identifier == HALDEX_ID && m.data_length_code >= 2)
  {
    receivedHaldexEngagementRaw = m.data[1];
    int gen2Engagement = 0;                                 // worked out in int so it can't wrap below 0
    if (receivedHaldexEngagementRaw > 0x80)
    {
      // V11.00.13 (audit_gen2_7h0): byte 1 = measured clutch pressure - 3.8 bar in the lock request's units; the 40 bar
      // sensor tops it at ~0xD7 (7H0) / ~0xEC (1K0). Same 0x80..0xEA = 0..100 % range as gen2LockRequestByte().
      gen2Engagement = ((int)receivedHaldexEngagementRaw - 0x80) * 100 / GEN2_FEEDBACK_FULL;
    }
    if (gen2Engagement > 100)
    {
      gen2Engagement = 100; // 0xFE (the unit's own top value) reads 100 %
    }
    receivedHaldexState = m.data[0];

    // Byte 0 status bits, as the OEM code sets them (spec_gen2_7h0 §6). The vw_pq.dbc
    // names (clutch fault, limp, AWD lamp, speed limit ...) do NOT match this unit:
    //   bit 0 = controller active / engaged (status, not a fault) [KM Allrad_1.Fehler_Allrad_Kupplung] [REA audit_gen2_7h0 #62: E480.1]
    //   bit 1 = over-temperature derate - the unit forces its request to 0 (genuine protection) [KM Allrad_1.Ubertemperaturschutz__Allrad_1_] [REA #62: E47C.5 && E3F6 == 0]
    //   bit 2 = fault / warning-lamp composite (genuine fault) - ALSO set whenever Bremse_1 (0x1A0) is not received
    //           [KM Allrad_1.Fehlerstatus_Kupplungssteifigke] [REA audit_gen2_7h0 #63/F4]
    //   bit 3 = commanded clutch set-point is 0 (output side, not a pressure reading) [KM Allrad_1.Kupplung_komplett_offen] [REA audit_gen2_7h0 #64/F5]
    //   bit 4, bit 5 = status, meaning not decoded yet (kept raw in receivedHaldexState) [KM Allrad_1.Notlauf / Allrad_Warnlampe] [REA #62: E47E.1 / E47E.2]
    //   bit 6, bit 7 = never written on Gen2 (no speed-limit bit) [KM Allrad_1.Geschwindigkeitsbegrenzung / Einheit_der_Kupplungssteifigkei] [REA #65]
    clearHaldexHealth();
    receivedTempProtection = (receivedHaldexState & (1 << 1)); // bit 1: over-temperature derate
    receivedReportClutch1 = (receivedHaldexState & (1 << 2));  // bit 2: fault present ...
    receivedAwdWarning = (receivedHaldexState & (1 << 2));     // ... and it is what lights the AWD lamp
    receivedCouplingOpen = (receivedHaldexState & (1 << 3));   // bit 3: commanded set-point 0 = coupling open (audit #64)
    receivedHaldexTempC = -128;                                 // Gen2 does not send a temperature
    haldexHealthSupported = true;

    // At rest a real 1K0 sends byte 1 = 0x88 with bit 3 (coupling open) set (from log
    // "1K0 554A - clear codes, brake on,off ..."): the measured value has a small floor.
    // When the unit itself says the coupling is open, show 0 %.
    if (receivedCouplingOpen)
    {
      gen2Engagement = 0;
    }
    receivedHaldexEngagement = (uint8_t)gen2Engagement;
    receivedLongLockState = 0; // not present on PQ Allrad_1
  }
}

static void hdxDecodeGen4(const twai_message_t &m)
{
  // Gen4 Haldex (later PQ) Allrad_1 / 0x2C0: engagement on byte 1, raw 128..255.
  // Like Gen5, the module broadcasts more than one CAN ID; only decode the
  // engagement frame. Without this guard a stray frame's byte 1 (< 128) makes
  // map() go negative and wrap in the uint8_t result -> the learn procedure
  // saw engagement jumping between 0 and >150%.
  // V11.00.0: decode per the 0BR's own code (spec_gen4_0br §6).
  if (m.identifier == HALDEX_ID)
  {
    // Byte 1 = the clutch torque the unit is applying, on the SAME scale
    // as our Bremse_4 lock request: 0x7F / 0x80 = none, 0x80 + n = n x 112 units,
    // 0xFD = top of the linear range, 0xFE = at full capacity. The unit
    // never sends 0xFF. V11: was mapped 0x80..0xFF (§7 #14).
    // [KM Allrad_1.Kupplungssteifigkeit_Mitte__Ist (3,-381) Nm/min]
    // [REA audit_gen4_0br #22: FUN_031580(F1B08): 0x7F if Bremse_4 = 0x7F and
    // target 0, 0x80 if target 0, else clamp(0x80 + F1B08/112, 0x81..0xFE);
    // F1B08 = COMMANDED target min(request, F1AFE), not a measurement (#25)]
    // Read-back range = lock request range (both modes): 0x81..0xFD -> 0..100 %, 0xFE = 100 %.
    receivedHaldexEngagementRaw = m.data[1];
    int engagementPercent = 0; // worked out in int so it can't wrap round
    if (receivedHaldexEngagementRaw >= 0xFE)
    {
      engagementPercent = 100; // at the unit's present maximum capacity
    }
    else if (receivedHaldexEngagementRaw > 0x80)
    {
      engagementPercent = ((int)receivedHaldexEngagementRaw - 0x80) * 100 / GEN4_FEEDBACK_FULL; // 0x81..0xFD -> 0..100 %
    }
    if (engagementPercent > 100)
    {
      engagementPercent = 100;
    }
    receivedHaldexEngagement = (uint8_t)engagementPercent;

    receivedHaldexState = m.data[0];
    clearHaldexHealth();
    // Byte 0 flags the 0BR really writes (b0.6 speed limit is never set
    // by Gen4, so it stays false - §7 #16; b0.7 is always 0):
    receivedReportClutch1 = (receivedHaldexState & (1 << 0));  // [KM Allrad_1.Fehler_Allrad_Kupplung] [REA audit_gen4_0br #20: fault-memory manager state E57E in {2,3,4}]
    receivedTempProtection = (receivedHaldexState & (1 << 1)); // [KM Ubertemperaturschutz__Allrad_1_] [REA #20/#40: over-temp latch F1AFC (also ramps F1AFE to 0)]
    receivedReportClutch2 = (receivedHaldexState & (1 << 2));  // [KM Fehlerstatus_Kupplungssteifigke] [REA #20: F146A != 0, signal-plausibility group]
    receivedCouplingOpen = (receivedHaldexState & (1 << 3));   // [KM Kupplung_komplett_offen] [REA #21/FI-3: commanded open OR zero request (0x7F and 0x80) OR torque limit F1AFE ramped out (e.g. rpm < ~200)] - status, not a fault
    receivedLimpMode = (receivedHaldexState & (1 << 4));       // [KM Notlauf] [REA #20: reaction level F1B9C != 0 OR Motor_5 not received]
    receivedAwdWarning = (receivedHaldexState & (1 << 5));     // [KM Allrad_Warnlampe] [REA #20: FUN_032924 true > 499 builds]
    // b0.6 [KM Geschwindigkeitsbegrenzung] [REA: no writer]; b0.7 [KM Einheit_der_Kupplungssteifigkei] [REA: always 0]
    // b2 [KM PNG_Status / PNG_Anzeige / Ganginfo] [REA #24: no writer, stays at init]

    if (m.data_length_code >= 5)
    {
      receivedTempSensorFault = (m.data[3] & (1 << 3)); // [KM b3.3 Frei_Allrad_1_1] [REA #23: temperature diagnosis FUN_019cce == -7 (sensor failed)]
      receivedFaultStored = (m.data[3] & (1 << 7));      // [KM Fehlerspeichereintrag_Allrad_1] [REA spec_gen4_0br §6: sig 0x39, set by the fault-memory writers]
      // b4 = Haldex temperature, degC = b4 - 60; 0xFF = sensor invalid
      // [KM Kupplungssteifigkeit_Hinten__Is 0.7874 % - the 0BR reuses the byte]
      // [REA #23/#47: oil temp E48A/10 + 60, clamp <= 0xFD unsigned, so < -60 degC also gives 0xFD (FI-4)]
      if (m.data[4] == 0xFF)
      {
        receivedHaldexTempC = -128; // no valid temperature
      }
      else
      {
        receivedHaldexTempC = (int16_t)m.data[4] - 60;
      }
    }
    // b5 [KM -] [REA #24: constant 0xC0]; b6 [KM -] [REA: constant 0xFE];
    // b7 [KM -] [REA #24: counter E7B2 0..15 << 4 | 3]. Not decoded - a silent
    // unit already shows as "no Haldex" through hasCANHaldex.
    haldexHealthSupported = true;
    receivedLongLockState = 0; // not present on PQ Allrad_1
  }
}

static void hdxDecodeGen41(const twai_message_t &m)
{
  // Gen41 (Vauxhall / GM FDCM) - per GMW8762 PPEI:
  //   0x1CC Secondary Axle Status (DLC=3): clutch torque feedback + state
  //   0x1D1 Rear Axle Status (DLC=4): status flags + diagnostic response
  // No OEM code held for the GM unit, so no health bits are decoded.
  clearHaldexHealth();
  haldexHealthSupported = false;
  switch (m.identifier)
  {
  case HALDEX_GEN41_SEC_AXLE_STATUS_ID:
    if (m.data_length_code >= 3)
    {
      const uint8_t d0 = m.data[0];
      const uint8_t d1 = m.data[1];
      const uint8_t d2 = m.data[2];
      receivedSecAxleStatusRaw = d0;
      receivedSecAxleActive = (d0 & 0x10) != 0;
      receivedSecAxleFdcmHealthy = (d0 & 0x08) != 0;
      receivedSecAxleArc = d0 & 0x03;
      receivedSecAxleTorqueNm = d1;
      receivedSecAxleClutchState = d2;
      // Feed the existing engagement pipe (used by API + broadcast).
      receivedHaldexEngagementRaw = d1;
      // Captured OEM peak ~150 Nm during active requests; map 0..200 Nm to 0..100%.
      receivedHaldexEngagement = (d1 >= GEN41_FEEDBACK_FULL) ? 100 : (uint8_t)((uint16_t)d1 * 100 / GEN41_FEEDBACK_FULL);
    }
    break;

  case HALDEX_GEN41_REAR_AXLE_STATUS_ID:
    if (m.data_length_code >= 4)
    {
      receivedRearAxleStatusFlags = m.data[0];
      receivedRearAxleMetricA = m.data[1];
      receivedRearAxleMetricB = m.data[2];
      // data[3] is the static 0x28 framing byte (verified across all OEM captures).
    }
    break;
  }
}

static void hdxDecodeGen50(const twai_message_t &m)
{
  // Gen5 Haldex 0CQ (MQB Allrad_03 / 0x118, DLC 4). V11.00.0: decoded per the
  // 0CQ907554D's own code (spec_gen5_0cq §6):
  //   b0       always 0x00 (never written) [KM - (no Allrad_03 signal)] [REA audit_gen5_0cq A24: no writer, init image 00 00 FE 00]
  //   b1[0..3] rolling counter 0-15 [KM Allrad_03_BZ] [REA audit A15: FUN_c205e6]
  //   b1[4..5] status: 0 = normal, 2 = the unit reports no torque (b2 forced
  //            to 0). 1 and 3 are never sent. (Was read as "0 open /
  //            1 partial / 2 closed" - wrong, spec D11.) [KM ALR_Sta_Laengssperre:
  //            0 Regelmodus, 2 Fehler_Sperre_offen] [REA audit A23/A32/F2: 2 is
  //            sent in fault-reaction states 0xCC10 in {2,3,4} AND by a second
  //            routine FUN_c21398 (task FUN_c4c776, when it runs is unknown);
  //            that the clutch is "released" is NOT shown in the code]
  //   b1[6]    [KM ALR_Sendestatus (0 = sends at 100 ms, 1 = 10 ms)] [REA audit
  //            A19/F3: NOT a clean "request accepted" flag - it is 0 only for
  //            the "no request" pattern Max in {FA,FE,FF} with Min in {FE,FF,00}
  //            (+ ALR_Sta unchanged 5 cycles, PDU-status test) and is not updated
  //            outside the supply-voltage window. Our frames always give 1.]
  //   b2       actual clutch torque, 0-250 = 0-100 % (0.4 %/step, the same
  //            scale as our ESP_14 lock request); 0xFE until the first value after
  //            power-up (was shown as 101 %, spec D13) [KM ALR_Ist_Proz 0.4 %,
  //            253 Fehler+zu, 254 Init, 255 Fehler] [REA audit A23: min(round(
  //            0xCDBC/128), 250); 0 in states 2/3/4; 0xFA in state 5 + 0xD02A = 0xC2]
  //   b3       [KM ALR_Charisma_FahrPr b3[0-3], ALR_Charisma_Status b3[4-5]]
  //            [REA audit A24: no writer on 554D]
  //            never written by the 0CQ907554D (always 0x00). Other 0CQ units
  //            DO send their drive-mode reply here - our 554C bench log and the
  //            car logs show 0x12 - so it is still kept raw in
  //            receivedHaldexState for the Drive Mode display, but nothing
  //            is decided from it (spec D12).
  // The VAQ (52) sends Quersperre_03 0x137 instead, never 0x118 (hdxDecodeGen52)
  // (OEM code: its TX table, spec §8.8).
  if (m.identifier == HALDEX_ID_GEN5)
  {
    receivedHaldexEngagementRaw = m.data[2]; // [KM Allrad_03.ALR_Ist_Proz 0.4 %] [REA audit A23: actual torque 0xCDBC/128, <= 250]
    int engagementPercent = 0; // worked out in int so it can't wrap round
    if (receivedHaldexEngagementRaw <= GEN50_FEEDBACK_FULL)
    {
      engagementPercent = (int)receivedHaldexEngagementRaw * 100 / GEN50_FEEDBACK_FULL; // 0..250 -> 0..100 %
    }
    // 251 and up (0xFE = no value yet after power-up) = not available -> 0 %
    if (engagementPercent > 100)
    {
      engagementPercent = 100;
    }
    receivedHaldexEngagement = (uint8_t)engagementPercent;
    receivedHaldexState = m.data[3]; // drive-mode reply on units that send it (0 on 554D), display only [KM ALR_Charisma_FahrPr/_Status] [REA audit A24: no writer on 554D]
    hdxFbCapture(m);

    // Health (spec §6). Status 2 = the unit reports no torque - from its
    // fault-reaction states OR the second routine FUN_c21398 (audit A23/F2), so
    // it is not proof of a fault; "clutch released" is not traced. 0x17F0000F
    // byte 0 bit 0 is the clean fault-reaction flag (kept from the last status
    // frame, forgotten if that frame stops). Audit F2 suggests calling it a
    // fault only with b0.0 or a stored DTC - behaviour left as is (report).
    clearHaldexHealth();
    uint8_t allradStatus = (m.data[1] >> 4) & 0x03; // [KM Allrad_03.ALR_Sta_Laengssperre b1[4-5]] [REA audit A15/A23: sig83, values {0,2}]
    bool isFaultReaction = false;
    if (allradStatus == 2)
    {
      // ALR_Sta = 2 [KM Fehler_Sperre_offen] [REA audit A23/F2: states 2/3/4 or FUN_c21398 - not necessarily a fault].
      // V11.00.13: shown as "coupling open" only; limp / fault come from 0x17F0000F b0.0 (the unit's own fault-reaction bit).
      receivedCouplingOpen = true;
    }
    if (gen5StatusFaultReaction && gen5StatusFrameMs != 0 &&
        (millis() - gen5StatusFrameMs) < HALDEX_GEN5_STATUS_TIMEOUT_MS)
    {
      isFaultReaction = true; // 0x17F0000F byte 0 bit 0
    }
    if (isFaultReaction)
    {
      receivedReportClutch1 = true; // a genuine fault is active - read the codes over UDS
      receivedLimpMode = true;      // fault reaction
      receivedCouplingOpen = true;  // b2 = 0 is reported; "released" not shown in the OEM code (audit A32)
    }
    receivedLockRequestValid = (m.data[1] & (1 << 6)); // [KM ALR_Sendestatus b1.6] [REA audit A19/F3: 0 only for the "no ESP request" pattern - not proof of acceptance]
    receivedHaldexTempC = -128;                                      // the 0CQ does not send a temperature
    haldexHealthSupported = true;

    // Same 0..3 slot the UI / serial lab already read: 0 = normal, 2 = "no
    // torque" status (fault reaction or FUN_c21398 - audit A23; 1 and 3 are
    // never sent by the 0CQ).
    receivedLongLockState = allradStatus;
  }
  else if (m.identifier == HALDEX_GEN5_STATUS_ID && m.extd)
  {
    // 0CQ status frame 0x17F0000F (29-bit, DLC 8). OEM code (spec §6): byte 0
    // bit 0 = 1 while the unit is in a fault-reaction state. Byte 7 bit 7 is
    // set by its fault-memory loop but is ALSO set on healthy cars in our logs,
    // so it is not decoded (spec §8 item 5).
    // [KM KN_Allrad: b0.0 Allrad_KompSchutz, b0.1 Abschaltstufe, b0.2 Transport_Mode,
    // b0[4-7] Nachlauftyp, b1 Allrad_SNI, b5 BusKnockOutTimer, b7.4 KN_ALR_DiagPfad,
    // b7.7 ALR_KD_Fehler] [REA audit S18/S19: b0.0 = 1 iff 0xCC10 in {2,3,4}
    // (FUN_c410f2), b1 = 0x0F const, b7.7 = flag 0xD014 (writers FUN_c483ee /
    // FUN_c4041e / code @0xC4053C)]. The VAQ sends 0x17F0001E instead (not decoded).
    if (m.data_length_code >= 1)
    {
      gen5StatusFaultReaction = (m.data[0] & (1 << 0)); // [KM KN_Allrad.Allrad_KompSchutz b0.0] [REA audit S18/F4: fault-reaction state, correct]
      gen5StatusFrameMs = millis();
    }
  }
}

// Gen5 VAQ (front-axle lock, MQB) - Quersperre_03 0x137, never Allrad_03 0x118
static void hdxDecodeGen52(const twai_message_t &m)
{
  if (m.identifier == QUERSPERRE_03)
  {
    // VAQ / Quersperre_03 (0x137, DLC 4) per MQB FCAN K-matrix - the
    // front-lock counterpart of Allrad_03, same byte plan:
    //   b1[0..3] Quersperre_03_BZ   rolling counter
    //   b1[4..6] QUER_Sta_Quersperre 0 rule mode / 1 driver / 2 err open /
    //                                3 err closed / 4 temp shutdown / 5 comms
    //   b1[7]    QUER_Gleichlauf     1 = synchronised
    //   b2       QUER_Ist_Proz       0.4 %/bit (250 = 100 %); 252 SafeState
    //                                sensor unverified, 253 error+closed,
    //                                254 init, 255 error -> read as 0 %
    //   b3       Sendestatus b0, VAQ_Charisma_FahrPr b1..4, Status b5..6
    // REA (audit_gen5_0cq §4, builder FUN_c1fb42, no CRC; counter writer
    // FUN_c4d834): b0 [KM -] not written by the builder; b1[4-6] = 4 if
    // FUN_c21168() else 0 (only 0 or 4 seen - 4 = KM "Temporaere_Funktions-
    // abschaltung"); b1.7 = FUN_c4c916(); b2 = min(x / scale, 250), 0 in states
    // 2/3/4, 0xFA in state 5 + 0xC2 - the scale is in the MISSING calibration
    // block, so "0.4 %/bit" is K-matrix + bench only; b3.0 ESP_14-related flag,
    // b3[1-4] in {0,2,3}, b3[5-6] in {0,1,2}. Meanings of the values not traced.
    receivedHaldexEngagementRaw = m.data[2]; // [KM Quersperre_03.QUER_Ist_Proz 0.4 %] [REA audit §4: min(x/scale,250), scale not in dump - bench only]
    if (receivedHaldexEngagementRaw >= 252)
      receivedHaldexEngagement = 0;
    else
      receivedHaldexEngagement = (uint8_t)map(receivedHaldexEngagementRaw > GEN52_FEEDBACK_FULL ? GEN52_FEEDBACK_FULL : receivedHaldexEngagementRaw, 0, GEN52_FEEDBACK_FULL, 0, 100);
    receivedHaldexState = m.data[3]; // [KM Quersperre_03_Sendestatus b3.0, VAQ_Charisma_FahrPr b3[1-4], VAQ_Charisma_Status b3[5-6]] [REA audit §4: b3.0 ESP_14 flag, b3[1-4] {0,2,3}, b3[5-6] {0,1,2}]
    receivedQuerState = (m.data[1] >> 4) & 0x07; // [KM QUER_Sta_Quersperre b1[4-6]] [REA audit §4: 4 if FUN_c21168() else 0]
    receivedQuerSync = (m.data[1] >> 7) & 0x01;  // [KM QUER_Gleichlauf b1.7] [REA audit §4: FUN_c4c916(), meaning not traced]
    hdxFbCapture(m);
    clearHaldexHealth();
    // V11 TODO (bench): the 0x137 builder IS now traced (audit §4, above) but
    // the meanings of its values are not, so these K-matrix states are not
    // confirmed from the OEM code - no health flags are set from them.
    haldexHealthSupported = false;
    receivedLockRequestValid = false; // 0CQ-only bit
    // Same slot for the VAQ: QUER_Sta_Quersperre shares the first four
    // meanings with ALR_Sta_Laengssperre (0 rule, 1 driver, 2/3 error).
    // REA (audit §4): the VAQ builder only ever sends 0 or 4 here.
    receivedLongLockState = receivedQuerState;
  }
}

static void hdxDecodeGen51(const twai_message_t &m)
{
  // Gen5 (0AY, 0AY907554F) Haldex - a PQ unit: feedback is PQ Allrad_1
  // (0x2C0, DLC 8). V11.00.0: decoded per the 0AY's own code
  // (spec_gen5_0ay §6) - same layout as Gen4 0BR.
  if (m.identifier == HALDEX_ID)
  {
    // Byte 1 = the clutch torque the unit is applying, on the SAME scale
    // as our Bremse_4 lock request: 0x7F / 0x80 = none, 0x80 + n = n x 112 units,
    // 0xFD = our 100 % request (0xFD on the lock request), 0xFE = at / over 0xFD.
    // The unit never sends 0xFF. Before V11 this was mapped 0x80..0xFF
    // with a ">= 99 -> 100" snap, because 0xFD only read 98-99 % on that
    // scale (spec §7 #10). The correct scale makes the snap unnecessary.
    // [KM Allrad_1.Kupplungssteifigkeit_Mitte__Ist (3,-381) Nm/min]
    // [REA audit_gen5_0ay #26/#48/#50: FUN_c4b22c: 0x7F if Bremse_4 valid, mode 0, torque 0;
    // 0x80 if torque 0; else clamp(0x80 + T/112, 0x81..0xFE), T = 0xCD3E = clutch-layer
    // command min(target, 0xCD32 protection ceiling) - reads LOWER than our request while
    // a protection limit is active (F4); 0 when 0xCC86 in {1,6,7} (#49)]
    // Read-back range = lock request range (both modes): 0x81..0xFC -> 0..99 %, 0xFD/0xFE = 100 %.
    receivedHaldexEngagementRaw = m.data[1];
    int engagementPercent = 0; // worked out in int so it can't wrap round
    if (receivedHaldexEngagementRaw >= GEN51_REQUEST_FULL_BYTE)
    {
      engagementPercent = 100; // full request reached (0xFD) or above it (0xFE)
    }
    else if (receivedHaldexEngagementRaw > 0x80)
    {
      engagementPercent = ((int)receivedHaldexEngagementRaw - 0x80) * 100 / GEN51_FEEDBACK_FULL; // 0x81..0xFC -> 0..99 %
    }
    if (engagementPercent > 100)
    {
      engagementPercent = 100;
    }
    receivedHaldexEngagement = (uint8_t)engagementPercent;

    receivedHaldexState = m.data[0];
    clearHaldexHealth();
    // Byte 0 flags the 0AY really writes (spec_gen5_0ay §6). b0.5 (warning
    // lamp) and b0.6 (speed limit) are never set by the 0AY - always 0 -
    // so they stay false; b0.7 is always 0 too.
    receivedReportClutch1 = (receivedHaldexState & (1 << 0));  // [KM Allrad_1.Fehler_Allrad_Kupplung] [REA audit_gen5_0ay #41: system state 0xCB9E in {2,3,4}; "clutch-system fault" INF]
    receivedTempProtection = (receivedHaldexState & (1 << 1)); // [KM Ubertemperaturschutz__Allrad_1_] [REA #42: 0xCF1F = r9 > 0xCD38 (hyst 0xCD3A); "over-temperature" INF, r9 not traced]
    receivedReportClutch2 = (receivedHaldexState & (1 << 2));  // [KM Fehlerstatus_Kupplungssteifigke] [REA #43: 0x7AB0 != 0; meaning INF]
    receivedCouplingOpen = (receivedHaldexState & (1 << 3));   // [KM Kupplung_komplett_offen] [REA #44: forced open 0xCD5A, OR valid Bremse_4 with request 0 and target 0 (0x7F and 0x80), OR 0x7AAA/7AB0/7ABC with target 0, OR 0xCF24 = 0] - status
    receivedLimpMode = (receivedHaldexState & (1 << 4));       // [KM Notlauf] [REA #45: 0xCD08 != 0 OR Motor_5 0x480 not received in its time-out]
    // b0.5 [KM Allrad_Warnlampe], b0.7 [KM Einheit_der_Kupplungssteifigkei] [REA #46: always 0];
    // b0.6 [KM Geschwindigkeitsbegrenzung] [REA #47: always 0]; b2 [KM PNG_*] [REA #51: always 0x00]

    if (m.data_length_code >= 5)
    {
      receivedTempSensorFault = (m.data[3] & (1 << 3)); // [KM b3.3 Frei_Allrad_1_1] [REA #52: status 0xFA of the temperature check chain FUN_c4ce46 (which sensor state, not traced)]
      // b3.7 [KM Fehlerspeichereintrag_Allrad_1] - but the 0AY does NOT write a
      // fault-memory flag here: [REA audit_gen5_0ay #53/F1: 0xCF8A from the background
      // self-test FUN_c41e12 / FUN_c1fdf4 (also DID 0x1000 bit 1)]. So on gen 51
      // V11.00.13: so it is no longer shown as "Fault Stored" / the dashboard fault warning on gen 51
      // (use Read Fault Codes for the real fault memory).
      receivedFaultStored = false;
      // b4 = Haldex temperature, degC = b4 - 60; 0xFF = sensor invalid
      // [KM Kupplungssteifigkeit_Hinten__Is 0.7874 % - the 0AY reuses the byte]
      // [REA #54: 0xFF if invalid, else min(T/10 + 60, 0xFD), T = 0xCCD8 (ch15) else 0xCCDA (ch13) - 2731]
      if (m.data[4] == 0xFF)
      {
        receivedHaldexTempC = -128; // no valid temperature
      }
      else
      {
        receivedHaldexTempC = (int16_t)m.data[4] - 60;
      }
    }
    // b5 [KM -] [REA #55: always 0x00]; b6 + b7 bits 0-3 [KM -] [REA #56: constant 0x3FE];
    // b7 bits 4-7 [KM -] [REA #40: TX callout counter 0..15]. Not decoded - a silent
    // unit already shows as "no Haldex" through hasCANHaldex.
    haldexHealthSupported = true;
    receivedLongLockState = 0; // not present on PQ Allrad_1
  }
}

// Volvo / SAAB (V10.00.0). What the status number means is not known yet,
// so it is shown as 0-255 -> 0-100 % until logs give the real scaling.
static void hdxDecodeVolvoP2Gen2(const twai_message_t &m)
{
  if (m.identifier == VOLVO_P2G2_HALDEX_ID)
  {
    // gen 21: [KM -][REA none] - no dump has this unit or its IDs (audit_volvo_c166 #86); no log has it awake
    receivedHaldexEngagementRaw = m.data[6]; // status byte [KM -][REA none: assumed by analogy with gen 3's b6]
    receivedHaldexEngagement = map(receivedHaldexEngagementRaw, 0, VOLVO_P2G2_FEEDBACK_FULL, 0, 100); // 0-255 scale [no OEM code]
    receivedHaldexState = m.data[0]; // byte 0 kept raw for the logs (not decoded) [KM -][REA none]
    hdxFbCapture(m);
    clearHaldexHealth(); // no OEM code held for the pre-facelift unit: no health bits known
    haldexHealthSupported = false;
    receivedLongLockState = 0;
  }
}

static void hdxDecodeVolvoP2Gen3(const twai_message_t &m)
{
  // OEM code (S60R DEM signal table, leaf_trace/volvo_bitrate.md goal 4; re-checked
  // audit_volvo_c166 #49-#60, V11.00.13 tags). [KM -] for every byte (no KMatrix for this set).
  //   bytes 0-4      never written by the unit [REA #59]
  //   byte 5 b0-2    "updated" bits for b6 / b7[7:6] / b7[1:0]; b5 bits 3-4 = sig 0x2E, always 0 [REA #59]
  //   byte 6         measured clutch value (analog input, most likely clutch
  //                  pressure = real engagement) [REA s29 #49-#53: = C01E >> 6, C01E = (ADC5 - 0x30D)
  //                  x 15 x 512 / 6250; the zero is a FIXED constant 0x30D, not measured; 0 below
  //                  ~0.9 V; full scale 135 (0x87), NOT ~150 - see VOLVO_P2G3_FEEDBACK_FULL]
  //   byte 7 b6-7    [REA s2C #55/#56]: 0 normal, 1 = mode selector set (0x10004002 b2[7:6] = 1/2),
  //                  2 = NOT READY - mainly ENGINE NOT RUNNING (ready ramp needs > ~401 rpm, drops
  //                  < ~200; also a stored input fault with clutch value 0) - normal with rpm 0,
  //                  3 = monitor 0xE reports a fault
  //   byte 7 b2-3    [REA s2A #57]: 3 = monitors 5 and 2 clear, else 0 - an input the
  //                  unit watches is missing / flagged (comms)
  //   byte 7 b0-1    [REA s2B #58]: S60R 2 if CF8C == 0 else 3; the 2005 DEM.bin differs - not decoded
  if (m.identifier == VOLVO_P2G3_HALDEX_ID)
  {
    receivedHaldexEngagementRaw = m.data[6]; // byte 6 = measured clutch value [KM -][REA s29: 0..135]
    int engagementPercent = (int)receivedHaldexEngagementRaw * 100 / VOLVO_P2G3_FEEDBACK_FULL; // [REA #53/F1: full scale 135 (V11.00.13)]
    if (engagementPercent > 100)
    {
      engagementPercent = 100; // clamp - full scale is a bench calibration value
    }
    receivedHaldexEngagement = (uint8_t)engagementPercent;
    receivedHaldexState = m.data[7]; // byte 7 = status bits (shown as binary on the dashboard)
    hdxFbCapture(m);

    clearHaldexHealth();
    haldexHealthSupported = true;
    receivedLongLockState = 0;

    uint8_t faultLimpValue = (m.data[7] >> 6) & 0x03; // byte 7 bits 6-7 [KM -][REA s2C]
    if (faultLimpValue == 2)
    {
      // [REA #56/#76/F2: 2 is mostly "not ready / engine not running" (ready ramp needs > ~400 rpm), so
      // V11.00.13 no longer shows it as limp mode - only as coupling open]
      receivedCouplingOpen = true; // unit not ready - clutch held at 0
    }
    else if (faultLimpValue == 3)
    {
      receivedReportClutch1 = true; // a diagnostic monitor reports a fault
    }
    // faultLimpValue 1 = mode-selector condition: status, not a fault [REA #55: C026 set (C00E is always 0)]

    uint8_t inputMonitorValue = (m.data[7] >> 2) & 0x03; // byte 7 bits 2-3 [KM -][REA s2A #57]
    if (inputMonitorValue != 3)
    {
      // An input the unit watches is missing or flagged - shown as a
      // "signal plausibility fault". In standalone this points straight at
      // a frame we aren't sending (yet).
      receivedReportClutch2 = true;
    }
    // V11 TODO (bench): confirm byte 6 full scale (VOLVO_P2G3_FEEDBACK_FULL)
    // at 100 % lock, and which situations set byte 7 bits 6-7 = 1 / 2 / 3.
  }
}

static void hdxDecodeVolvoGen4(const twai_message_t &m)
{
  // V11.00.5 bench (31219215 / 31219186, documents/VOLVO_GEN4_BENCH_2026-11-07.md):
  // replies on 0x1020000A every 20 ms, like the P2 DEM, but read differently:
  //   byte 6       measured clutch pressure: 0 = open, 0xFF = full lock
  //   byte 7 b0    1 = fault stored -> limp (clutch dumped, lock requests
  //                ignored). 0x0E = locking, 0x8E = idle (no demand) - both
  //                normal; 0x8F / 0x0F = fault (e.g. 0x0015, no config page)
  //   byte 7 b2-3  3 = the unit's input monitors are happy
  //   byte 7 b6-7  2 = no clutch demand (normal at rest, NOT limp on this unit)
  // V11.00.13 tags: every byte [KM -]; meanings [bench] only - NO P3 dump, so [no OEM code] for the
  // scale (255) and the bit meanings. The S60R layout (b6 = s29, b7 s2C / s2A / s2B, b5 bits 0-2 update
  // bits, b0-b4 never written) fits what the bench saw; its b6 full scale is 135, the P3 reached 0xFF.
  if (m.identifier == VOLVO_G4_HALDEX_ID && m.data_length_code >= 8)
  {
    receivedHaldexEngagementRaw = m.data[6]; // byte 6 = measured clutch pressure [KM -][bench 0..0xFF][no OEM code]
    int engagementPercent = (int)receivedHaldexEngagementRaw * 100 / VOLVO_G4_FEEDBACK_FULL;
    if (engagementPercent > 100)
    {
      engagementPercent = 100;
    }
    receivedHaldexEngagement = (uint8_t)engagementPercent;
    receivedHaldexState = m.data[7]; // byte 7 = status bits (shown as binary on the dashboard)
    hdxFbCapture(m);

    clearHaldexHealth();
    haldexHealthSupported = true;
    receivedLongLockState = 0;

    if (m.data[7] & 0x01)
    {
      receivedLimpMode = true;     // fault stored: the unit has gone into limp [KM -][bench: b7 bit 0][no OEM code; S60R b7[1:0] = s2B]
      receivedCouplingOpen = true; // ...and dumped the clutch
    }
    if (((m.data[7] >> 2) & 0x03) != 3)
    {
      receivedReportClutch2 = true; // an input the unit watches is missing / flagged [KM -][bench; S60R s2A layout]
    }
  }
}

static void hdxDecodeFordKuga(const twai_message_t &m)
{
  // Ford Kuga (V11.00.9) - 0x260 every 20 ms, Motorola (firmware traces trackF 8V41 / trackH 9V4N):
  //   byte 1 bits 6-7  status: 0 active, 1 (0x40) ABS release, 2 (0x80) no demand / rpm gate off /
  //                    test override on (8V41) / monitor, 3 (0xC0) monitor fault
  //                    [KM - (CGEA Information4x4_CG1 has AwdLck_Tq_Actl 15|12 1 Nm on b1:b2[7:4] - C1 layout differs)]
  //                    [REA #9/#10/#39 sig 0x36: 0x80 also when oil > 106 C (F1604) or base demand F15F8 = 0;
  //                     0x40 = 0x211 field 1, or field 2 only while the command E7C8 = 0; 9V4N: no 0x80 for 0x6F1]
  //   byte 1 bit 2     8V41: ABS torque ceiling (0x211 field 3) active [KM -] [REA #8 sig 0x35; 9V4N never written]
  //   byte 1 bits 0-1, byte 0  [KM -] [REA: 8V41 constant 0, 9V4N never written] - not read here
  //   byte 2 bits 4-5  [KM -] [REA #12 sig 0x38: 2 pump / monitor state, 1 over-temp, 0 normal] - not read here
  //   byte 2 bits 0-3 : byte 3   clutch torque / 8 - 8V41: from the MEASURED pressure;
  //                    9V4N: the unit's final torque request
  //                    [KM -] [REA #11/#13/#61 sig 0x37: 8V41 F168A>>3 = torque(measured pressure, slip, oil temp):
  //                     +2 once the pressure-ready latch F1048 is set, full scale 2438 (-40 C) / 2294 (8 C) /
  //                     1958 (120 C), 0x0FF = pressure-sensor / monitor-2 fault marker; 9V4N F13EE>>3 = request echo]
  //   bytes 4-7        8V41 hard-codes 0F FF 0F FF, 9V4N never writes them: not data [KM -] [REA #7/#60]
  // V11.00.11: same unit as the 0x211 torque ceiling we send (C), so % = feedback / KUGA_REQUEST_FULL.
  // REA caveats (display only): 0x0FF fault marker -> V11.00.13 shows 0 % + clutch fault (below); the +2 latch offset
  // reads 0 %; on 9V4N the % is the request, not a measurement.
  if (m.identifier == FORD_KUGA_HALDEX_ID && !m.extd && m.data_length_code >= 4)
  {
    const uint16_t fb = (uint16_t)(((m.data[2] & 0x0F) << 8) | m.data[3]); // b2[3:0]:b3 [KM -] [REA sig 0x37, see above]
    receivedHaldexEngagementRaw = (uint8_t)(fb >> 3); // 12-bit field shown as 8 bits on the dashboard
    int engagementPercent = 0;
    if (fb > KUGA_FEEDBACK_ZERO)
    {
      engagementPercent = (int)((uint32_t)(fb - KUGA_FEEDBACK_ZERO) * 100 / (KUGA_FEEDBACK_FULL - KUGA_FEEDBACK_ZERO));
    }
    if (engagementPercent > 100)
    {
      engagementPercent = 100;
    }
    receivedHaldexEngagement = (uint8_t)engagementPercent;
    receivedHaldexState = m.data[1]; // byte 1 = status bits (shown as binary on the dashboard)
    hdxFbCapture(m);

    clearHaldexHealth();
    haldexHealthSupported = true;
    receivedLongLockState = 0;

    const uint8_t status = (m.data[1] >> 6) & 0x03; // b1[7:6] [KM -] [REA sig 0x36, see above]
    if (fb == 0x0FF && status != 0)
    {
      // V11.00.13 (audit_ford_kuga FI-3): 0x0FF with a non-active status = pressure-sensor / monitor fault
      // marker, not 19 % lock (bench log 65 shows exactly 255)
      receivedHaldexEngagement = 0;
      receivedReportClutch1 = true;
    }
    if (status == 3)
    {
      receivedLimpMode = true;     // a monitor fault: the unit holds the clutch off
      receivedCouplingOpen = true;
    }
    else if (status == 1)
    {
      receivedCouplingOpen = true; // released for the ABS / ESP
    }
  }
}

static void hdxDecodeLandRover(const twai_message_t &m)
{
  // Land Rover Gen4 (V11.00.13, binary trace only - untested). FL2 sends 0x310, Evoque 0x301: same layout bit
  // for bit, Motorola, DLC 8 [REA audit_landrover #43-#46, E3/E4]. Whichever arrives is decoded:
  //   b0-b2            constant 0xFF [KM -] [REA #43: default image, no writer] - not read here
  //   b3 bits 0-1      sig 0x45, no writer (stays 3) [KM -] [REA #43] - not read here
  //   b4 bits 0-1      3 normally, 0 when monitor 15 = 0xE5 [KM -] [REA #46 sig 0x47]
  //   b4 bits 2-3      bit 2 = a DTC record has status bit 3; bit 3 = FUN_03283a() == 7 [KM -] [REA #46 sig 0x4C; lamp INF]
  //   b4 bits 4-7      update bits of sigs 0x4B / 0x46 / 0x48 / 0x4C [KM -] [REA #46] - not read here
  //   b5 bits 0-5      accepted mode request + reason code (0x3EE / EVQ 0x34F echo) [KM -] [REA #37] - not read here
  //   b5 bits 6-7      2 = unit state 4 / monitor 6 = 0xE3, 1 = temperature margin < 5 [KM -] [REA #46 sig 0x4B]
  //   b6 bits 5-7      status: 0 active, 1 ABS open / ramp with F257E = 0, 2 not ready (base demand off),
  //                    3 fault (cal 0x60D5 / monitor 15 = 0xE5), 4 temperature margin < 5, 5 limit flag F2606 /
  //                    monitor 3 = 0xFF, 6 ABS full lock [KM -] [REA #45 sig 0x48; Evoque E4: 5 also needs monitor
  //                    3 != 0xE8 on the temperature branch, 1 also needs FUN_02b3ca() == 2 - same codes]
  //   b6[4:0]:b7       F2612 >> 4 = COMMANDED clutch torque (not measured) [KM -] [REA #44 sig 0x46]
  // % = feedback / LR_FEEDBACK_FULL - the same scale as the lock request T (standalone and OEM identical).
  if ((m.identifier == LR_FL2_HALDEX_ID || m.identifier == LR_EVQ_HALDEX_ID) &&
      !m.extd && m.data_length_code >= 8)
  {
    lrFeedbackLayout = (m.identifier == LR_FL2_HALDEX_ID) ? LR_LAYOUT_FL2 : LR_LAYOUT_EVQ; // chassis-decode layout only
    const uint16_t fb = (uint16_t)(((m.data[6] & 0x1F) << 8) | m.data[7]); // b6[4:0]:b7 [KM -] [REA #44]
    receivedHaldexEngagementRaw = (uint8_t)((fb >> 3) > 255 ? 255 : (fb >> 3)); // 13-bit field shown as 8 bits (1500 -> 187)
    uint32_t pct = (uint32_t)fb * 100 / LR_FEEDBACK_FULL;
    receivedHaldexEngagement = (uint8_t)(pct > 100 ? 100 : pct);
    receivedHaldexState = m.data[6]; // byte 6 = status bits 5-7 (shown as binary on the dashboard)
    hdxFbCapture(m);

    clearHaldexHealth();
    haldexHealthSupported = true;
    receivedLongLockState = 0;

    const uint8_t status = (m.data[6] >> 5) & 0x07; // b6[7:5] [KM -] [REA #45]
    if (status == 3)
    {
      receivedLimpMode = true;     // fault reaction: commanded torque withheld
      receivedCouplingOpen = true;
    }
    else if (status == 4 || status == 5)
    {
      receivedTempProtection = true; // temperature margin / limit flag (5's monitor-3 branch is INF)
    }
    else if (status == 1)
    {
      receivedCouplingOpen = true; // opened / ramped down for the ABS / DSC (or our Follow ABS)
    }
    if ((m.data[4] & 0x03) == 0)
    {
      receivedReportClutch1 = true; // b4 bits 0-1 = 0: monitor 15 = 0xE5 [REA #46 sig 0x47; fault meaning INF]
    }
    // b4 bits 2-3 only once sig 0x4C has been written (its update bit b4.7 set) - the default image 0x0F has both set
    const bool sig4CWritten = (m.data[4] & 0x80) != 0; // [KM -] [REA #46: pB bit of sig 0x4C; clearing INF]
    if (sig4CWritten && (m.data[4] & 0x04))
    {
      receivedFaultStored = true; // b4 bit 2: a stored DTC with status bit 3 [REA #46 sig 0x4C bit 0]
    }
    if (sig4CWritten && (m.data[4] & 0x08))
    {
      receivedAwdWarning = true; // b4 bit 3: FUN_03283a() == 7 [REA #46 sig 0x4C bit 1; lamp request INF]
    }
    if (((m.data[5] >> 6) & 0x03) == 1)
    {
      receivedTempProtection = true; // b5 bits 6-7 = 1: temperature margin < 5 [REA #46 sig 0x4B]
    }
  }
}

static void hdxDecodeVolvoSpc(const twai_message_t &m)
{
  // Volvo Gen5 "SPC" 31380867 (V11.00.0). Reply 0x126, 8 bytes, high byte
  // first, no checksum or counter (OEM code: spec_ppc_volvo_spc_bmw.md section 6):
  //   bytes 0-1 (15 bits) = the unit's own final torque SETPOINT (audit_ppc_spc_bmw #27; b0 bit 7 = COM update bit)
  //                         (forced to 0 when the unit is in a fault / off state)
  //   byte 3 bits 3-4      = status: 0 normal, 1 torque withheld (faults and normal states),
  //                         2 special state (strategy state 4, meaning unknown); b3 bits 5-7 = update bits
  // No warning-lamp bit is sent on 0x126.
  if (m.identifier == VOLVO_SPC_HALDEX_ID && m.data_length_code >= 4)
  {
    // [KM -] for every byte (no 0x126 in volvo_v60_2015_pt / v40_2017_pt). b2, b4, b5-7: not read here
    // [REA #19/#23: b2 and b4 stay at their init value - no writer; b5-7 unused; b4 bits 4-6 = update bits]
    uint16_t torqueCount = (uint16_t)(m.data[0] & 0x7F); // byte 0 bits 0-6 = top 7 bits [KM -][REA tx7 #20: min(((Ram40001030 >> 4) & 0xFFF) x 1024 / 625, 0xFFFF); b0 bit 7 = update bit]
    torqueCount = (uint16_t)((torqueCount << 8) | m.data[1]); // byte 1 = bottom 8 bits [KM -][REA tx7]

    // engagement % = the unit's torque SETPOINT (audit #27, not a delivered / measured torque) against
    // full scale, worked out in a whole number first and clamped so it can never wrap past 100
    int engagementPercent = (int)((uint32_t)torqueCount * 100 / VOLVO_SPC_FEEDBACK_FULL);
    if (engagementPercent > 100)
    {
      engagementPercent = 100;
    }
    if (engagementPercent < 0)
    {
      engagementPercent = 0;
    }
    // V11 TODO (bench): full scale (V11.00.13: 2033) is worked out from our lock request ceiling, [INF],
    // not a measured full lock - set VOLVO_SPC_FEEDBACK_FULL on the bench.
    receivedHaldexEngagement = (uint8_t)engagementPercent;

    uint16_t rawScaled = torqueCount >> 5; // 15-bit torque / 32 so it fits one byte (6710 max -> 209; 2033 -> 63)
    if (rawScaled > 255)
    {
      rawScaled = 255;
    }
    receivedHaldexEngagementRaw = (uint8_t)rawScaled;
    receivedHaldexState = m.data[3]; // byte 3 kept raw (status in bits 3-4) [KM -][REA #19/#24: b3[2:0] = tx1 FUN_00051414(); bits 5-7 update bits]

    const uint8_t unitStatus = (m.data[3] >> 3) & 0x03; // byte 3 bits 3-4 [KM -][REA tx5 #21: 0 normal, 1 withheld (fault or normal-state causes), 2 = Ram40001038 == 4]
    clearHaldexHealth();
    if (unitStatus == 1)
    {
      // V11.00.13 (audit_ppc_spc_bmw #21/#22): status 1 = torque withheld - set by faults AND by normal
      // states (e.g. FUN_000214cc() == 0), so it is no longer shown as limp mode until benched.
      receivedReportClutch1 = true; // torque withheld
    }
    if (torqueCount == 0)
    {
      receivedCouplingOpen = true; // delivering no torque: clutch fully open (status, not a fault)
    }
    haldexHealthSupported = true;
    receivedLongLockState = 0;
    hdxFbCapture(m);
  }
}

static void hdxDecodeVolvoP1(const twai_message_t &m)
{
  // Volvo P1 AWD module (V11.00.1), 0x1960000A every 20 ms - its only frame.
  // From Adam's V50 T5 AWD logs (P1_V50_T5_AWD_DECODE.md). Every byte [KM -][REA none] - no P1 dump,
  // no P1 KMatrix; all [LOG], incl. the 540 full scale:
  //   bytes 0-3   always 00          byte 4   always 05
  //   byte 5      status: 2E ignition on / engine off, 2C or 3C running
  //   byte 6      bits 2-7 always 111000, bits 0-1 = top 2 bits of engagement
  //   byte 7      bottom 8 bits of engagement (10-bit: 0 open .. 540 full lock)
  if (m.identifier == VOLVO_P1_HALDEX_ID && m.extd &&
      m.data_length_code >= 8)
  {
    uint16_t engagementCount = (m.data[6] & 0x03) << 8; // byte 6 bits 0-1 = top 2 bits
    engagementCount = engagementCount | m.data[7];      // byte 7 = bottom 8 bits
    volvoP1EngagementCount = engagementCount;                        // kept for the Volvo P1 card's live readout

    // engagement % = count against full lock, worked out in a whole number and clamped
    uint32_t engagementPercent = (uint32_t)engagementCount * 100 / VOLVO_P1_FEEDBACK_FULL;
    if (engagementPercent > 100)
    {
      engagementPercent = 100;
    }
    receivedHaldexEngagement = (uint8_t)engagementPercent;

    // raw value for the dashboard / logs: the 10-bit count / 4 so it fits one byte (540 -> 135)
    receivedHaldexEngagementRaw = (uint8_t)(engagementCount >> 2);
    receivedHaldexState = m.data[5]; // byte 5 = status (2C / 3C / 2E)
    hdxFbCapture(m);
    clearHaldexHealth(); // no fault bits found in the logs yet
    haldexHealthSupported = false;
    receivedLongLockState = 0;
  }
}

static void hdxDecodeSaab93(const twai_message_t &m)
{
  if (m.identifier == SAAB_93_HALDEX_ID)
  {
    // 11-bit value: low 3 bits of byte 0 then all of byte 1, cut down to 0-255
    uint16_t statusValue = (m.data[0] & 0x07); // byte 0 bits 0-2 = top 3 bits
    statusValue = (statusValue << 8) | m.data[1]; // byte 1 = bottom 8 bits
    receivedHaldexEngagementRaw = statusValue >> 3;          // 0-2047 -> 0-255
    receivedHaldexEngagement = map(receivedHaldexEngagementRaw, 0, SAAB_93_FEEDBACK_FULL, 0, 100);
    receivedHaldexTempC = (int16_t)m.data[2] - 40; // byte 2 = temperature + 40 degC (GM style)
    receivedHaldexState = m.data[0];                // byte 0 kept raw for the logs (not decoded)
    hdxFbCapture(m);
    clearHaldexHealth(); // no OEM code held for the 9-3 unit: only the temperature is known
    haldexHealthSupported = false;
    receivedLongLockState = 0;
  }
}

// Decode one Haldex frame for the active generation
static void hdxDecodeFeedback(const twai_message_t &m)
{
  switch (haldexGeneration)
  {
  case 1:
    hdxDecodeGen1(m);
    break;
  case 2:
    hdxDecodeGen2(m);
    break;
  case 4:
    hdxDecodeGen4(m);
    break;
  case 41:
    hdxDecodeGen41(m);
    break;
  case 50:
    hdxDecodeGen50(m);
    break;
  case 51:
    hdxDecodeGen51(m);
    break;
  case 52:
    hdxDecodeGen52(m);
    break;
  case GEN_VOLVO_P2_GEN2:
    hdxDecodeVolvoP2Gen2(m);
    break;
  case GEN_VOLVO_P2_GEN3:
    hdxDecodeVolvoP2Gen3(m);
    break;
  case GEN_VOLVO_GEN4:
    hdxDecodeVolvoGen4(m);
    break;
  case GEN_FORD_KUGA:
    hdxDecodeFordKuga(m);
    break;
  case GEN_LANDROVER_GEN4:
    hdxDecodeLandRover(m);
    break;
  case GEN_VOLVO_SPC_GEN5:
    hdxDecodeVolvoSpc(m);
    break;
  case GEN_VOLVO_P1:
    hdxDecodeVolvoP1(m);
    break;
  case GEN_SAAB_93_XWD:
    hdxDecodeSaab93(m);
    break;
  default: // 42 (Focus RS): no feedback decode
    break;
  }
}

// ---- Diagnostic replies --------------------------------------------------------

// Diagnostic replies to OUR requests (UDS poller, web read, TP2.0 poller /
// fault-code task) stay off Bus 0: the car never asked for them.
// UDS MQB: mirror 0x779 response frames into the UDS task queue when polling is active.
// While our poller owns the diagnostic channel and no external tool is on
// the chassis side, every 0x779 is a reply to a request the car never made
// - keep it off Bus 0 so the gateway/OBD side sees no diagnostic traffic it
// did not ask for. The moment a real tool is seen on Bus 0 the poller backs
// off and replies are forwarded again (VCDS / gauge passthrough intact).
// Returns true when the frame is a reply to our own request (keep it off Bus 0).
static bool hdxTapDiagnosticReply(const twai_message_t &m)
{
  bool udsSuppressForward = false;
  if (liveDiagEnabled && udsRxQueue != nullptr && m.identifier == udsHaldexRespId)
  {
    if (xQueueSend(udsRxQueue, &m, 0) != pdTRUE)
    {
      // One missed UDS response (the poller re-requests) - log, don't block.
      DEBUG("CAN - UDS MQB tap queue full, dropped frame 0x779");
    }
    udsSuppressForward = udsPollActive && !externalDiagActive();
  }

  // /api/uds/read tap (Bus 1 reads): copy, the frame still forwards below.
  if (udsWebRespId != 0 && udsWebBus == 1 && udsWebRxQueue != nullptr &&
      m.identifier == udsWebRespId)
  {
    if (xQueueSend(udsWebRxQueue, &m, 0) != pdTRUE)
    {
      DEBUG("CAN - UDS web tap queue full, dropped frame 0x%03X", (unsigned)m.identifier);
    }
    udsSuppressForward = true; // reply to our own web request - same rule as the poller
  }

  // KWP2000-over-TP2.0 (Gen2/4): mirror channel-setup + negotiated data-channel
  // frames into the TP2.0 task queue. V11.00.7: and keep them off Bus 0 unless a
  // real tool is on the car's side - before, the Haldex's 0x20A setup reply and
  // its 0x300 data channel reached the car's bus as answers to requests the car
  // never sent (0x300 is also the channel a real tester usually gets, and it
  // outranks most chassis frames on the bus).
  if ((liveDiagEnabled || dtcBusy) && tp20RxQueue != nullptr && !m.extd) // V11.00.5: dtcBusy = fault-code read / clear
  {
    const uint32_t tpId = m.identifier;
    if (tpId == KWP_TP20_SETUP_RX_ID || tpId == KWP_TP20_TESTER_RX_ID ||
        (kwpTp20EcuTxId != 0 && tpId == kwpTp20EcuTxId))
    {
      xQueueSend(tp20RxQueue, &m, 0);
      if (!externalDiagActive())
      {
        udsSuppressForward = true; // reply to our own TP2.0 channel
      }
    }
  }
  return udsSuppressForward;
}

// ---- Main code ---------------------------------------------------------------

void parseCANHdx(void *arg)
{
  // for all haldex frames received
  while (1)
  {
#if detailedDebugStack
    stackHDX = uxTaskGetStackHighWaterMark(NULL);
#endif

    if (twai_receive_v2(twaiBus1, &rxMessageHdx, portMAX_DELAY) != ESP_OK)
    {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    do
    {
      if (canLoopTestCatch(rxMessageHdx, 1))
      {
        continue; // CAN loop test frame (V11.00.12): not Haldex traffic
      }
      uint32_t haldexNowMs = millis();
      if (haldexNowMs == 0)
      {
        haldexNowMs = 1; // 0 means "never seen" (millis() wraps to 0 every 49.7 days)
      }
      lastCANHaldexTick = haldexNowMs;
      ++lpHaldexFrameCount;
      hdxRxCensus(rxMessageHdx);

      // Analyzer mode: queue for GVRET/SLCAN and forward untouched, skipping control logic.
      if (analyzerMode || analyzerSerial)
      {
        analyzerQueueFrame(rxMessageHdx, 1);
        canTransmit(twaiBus0, &rxMessageHdx);
        continue;
      }

      hdxDecodeFeedback(rxMessageHdx); // engagement % + health flags for the active generation

      if (!hdxTapDiagnosticReply(rxMessageHdx))
      {
        canTransmit(twaiBus0, &rxMessageHdx);
      }
    } while (twai_receive_v2(twaiBus1, &rxMessageHdx, 0) == ESP_OK);
  }
}
