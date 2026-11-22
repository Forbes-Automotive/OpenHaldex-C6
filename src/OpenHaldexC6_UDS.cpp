#include <OpenHaldexC6_UDS.h>
#include <OpenHaldexC6_can.h> // canTransmit

// ---- Statics -------------------------------------------------------------------

using namespace OpenHaldexC6;

// Outcome of one ReadDataByIdentifier exchange (see udsPollDid).
enum UdsPollResult : uint8_t
{
    UDS_POLL_OK = 0,      // positive 0x62 response for the requested DID
    UDS_POLL_NRC_SESSION, // 0x7F with a session/security NRC - needs extended session
    UDS_POLL_NRC_OTHER,   // 0x7F with any other NRC (DID unknown, busy, ...)
    UDS_POLL_TIMEOUT,     // nothing matching inside the window
};

static bool gen2LoginLoaded = false;
static uint32_t gen2LoginBlockedMs = 0;  // millis() of the last 7F 27 36 / 37, 0 = none
// ---- Fault codes - read / clear for every generation we know how to talk to ----

volatile uint8_t dtcState = DTC_IDLE;
volatile uint8_t dtcRequest = 0;
volatile bool dtcBusy = false;
uint8_t dtcLastAction = 0;
uint8_t dtcCount = 0;
DtcEntry dtcList[DTC_MAX];
char dtcProtocol[24] = "";
char dtcMessage[96] = "";
uint32_t dtcDoneMs = 0;

enum
{
    DTC_PROTO_NONE = 0,
    DTC_PROTO_UDS,
    DTC_PROTO_KWP,
    DTC_PROTO_VOLVO_D2,
    DTC_PROTO_FORD_KWP, // V11.00.9: Ford Kuga
};

// Volvo D2 live data (V11.00.5): the P2 / P3 AWD module's CAN reply has no
// temperature, so with Live Diagnostics on it is read over D2 every 2 s:
// A6 00 02 byte 0 = oil temperature, signed degC (vtl/volvo-ddd "DEM oil
// temperature"; bench 10-13 degC in a cold garage) -> the Haldex temperature tile.
// [KM -][WEB vtl/volvo-ddd][bench P3][REA: S60R diag handlers FUN_01f122 / 01f17a not decoded for the A6
// local IDs - no OEM code for this value; gen 3 / 21 use it untested]
static uint32_t volvoDiagLastPollMs = 0;
static uint32_t volvoDiagLastGoodMs = 0;

// Ford Kuga live data (V11.00.10): Ford KWP2000 ReadDataByCommonIdentifier 22 HH LL on
// 0x761 / 0x769, in the module's default session - NO session change is ever sent (10 FB
// stops the unit's normal control). DIDs from the unit's own firmware (leaf_trace/
// trackF_kuga_8v41.md, trackH_kuga_9v4n.md) and the bench (FORD_KUGA_GEN4_BENCH_2026-11-18.md),
// big-endian (Ford = Motorola):
//   8V41 (bench unit):
//   (KWP DIDs are in no CAN database: [KM -] for all of them. REA = audit_ford_kuga.md #n.)
//     0x0042 supply, mV                         [bench: 13968 = 13.97 V on a 13.97 V supply] [REA #72: E430 x 24 mV]
//     0x1943 oil temperature, s16 0.1 degC      [bench: 0x4D = 7.7 C at ~5 C ambient; 0xFDDB = open]
//                                               [REA #33/#73: E3FA = E40C (0.1 K) - 0xAAB]
//     0x1944 measured clutch pressure: filtered sensor (E422:E424)/100, 0..800 [REA #33/#44: ADC clamped
//            0x66..0x39A (10..90 % ratiometric) x 0x619A/256 - 0x26E3; "0.5 V .. 4.5 V" and any bar unit
//            are NOT in the code (#45 unverifiable) - the bar scale below is PROVISIONAL]
//     0x1945 valve (solenoid) current, E442>>4 [follows the raw actuator value 1:1, 920..1411 when
//            regulating] - mA by inference [REA #46: filtered measured coil current, ADC ch 11, EEPROM gain]
//     0x1946 pump motor current: supply-side ADC / duty [INF mA; the unit's limit is 4.0-6.5 A]
//            [REA #85: G(duty) x (ADC10 or 2 x ADC15) >> 5, unit INF]
//     0x1947 VALVE COMMAND, not a pressure reading [REA #32/#43, audit FI-8: FUN_01f8ec(E4A8) = inverse
//            map of the actuator command = closed-loop output, in F1058 units (= 0x1944 x 19.53).
//            The old "0.1 kPa" (idle 4008 = 0.40 MPa) was a fit to Volvo's Gen4 spec, not from the code]
//   The firmware converts setpoint -> sensor units with x512/10000 (FUN_019cea), so one 0x1944
//   count = 19.53 setpoint counts: both pressures are shown in the same (provisional) bar scale,
//   0x1947 / 1000 and 0x1944 x 0.01953. Absolute bar still needs a gauge on a working coupling.
//   PROVISIONAL [REA #6/#70, audit FI-4]: no bar/mbar unit exists in the firmware; 0.01953 bar/count
//   (here) and the older "0.05 bar/count = 40 bar full scale" (ver.h history) are both assumptions.
//   9V4N (static only, never on the bench): no 0x1943 / 0x1944 / 0x1947 [REA #62]. Temperature 0x19BA
//   (s8 degC, E48A/10 INF), setpoint 0xEE04 (E554 = commanded pressure setpoint, units unknown - shown
//   raw), pump 0x1946 (FUN_01a582), supply 0x0042.
//   0x1945 on 9V4N is the VALVE COIL CURRENT, not pressure [REA #63/#64, audit FI-1]: same ADC ch 11
//   path as 8V41's 0x1945. 9V4N has NO pressure read-back at all (its converted sensor value E4A6/E4AC
//   is never read, audit D) - V11.00.13: kFord9V4N maps it to the valve current; no pressure is shown on 9V4N.
// Stands down for 15 s after any serial-lab / web UDS request so bench scripts own the channel.
static uint32_t fordDiagLastPollMs = 0;
static uint32_t fordDiagLastGoodMs = 0;
static uint8_t fordDiagIdx = 0;
static uint8_t fordDiagMisses = 0;

struct FordDid
{
    uint16_t did;
    uint8_t field;  // FORD_DIAG_*
    bool oneByte;   // signed 8-bit value instead of a 16-bit word
};
static const FordDid kFord8V41[] = {
    {0x1944, FORD_DIAG_PMEAS, false}, {0x1947, FORD_DIAG_PCMD, false}, {0x1945, FORD_DIAG_VALVE, false},
    {0x1946, FORD_DIAG_PUMP, false},  {0x1943, FORD_DIAG_TEMP, false}, {0x0042, FORD_DIAG_SUPPLY, false},
};
static const FordDid kFord9V4N[] = {
    {0x1945, FORD_DIAG_VALVE, false}, {0xEE04, FORD_DIAG_PCMD, false}, {0x1946, FORD_DIAG_PUMP, false}, // V11.00.13: 0x1945 = valve coil current (was shown as pressure) - 9V4N has no pressure read-back [REA #63, audit D]
    {0x19BA, FORD_DIAG_TEMP, true},   {0x0042, FORD_DIAG_SUPPLY, false},
};

// ---- Small helpers -------------------------------------------------------------

// ---------------------------------------------------------------------------
// UDS MQB polling task (Gen 5 only)
// Requests are sent on Bus 1 to udsHaldexReqId: 0x70F (ISO_ALLRAD_REQ, Haldex
// physical address - was wrongly noted as 0x771) or 0x71E on the VAQ.
// Responses come from 0x779 (VAQ 0x788) on Bus 1, routed here via udsRxQueue by
// parseCANHdx so there is no TWAI RX queue contention.
// [KM -] UDS DIDs are not K-matrix signals. REA = diag_gen5_0cq §2 / §3, re-checked
// in audit_gen5_0cq G1-G19 (DID tables 0xC14B14 0CQ / 0xC14BD4 VAQ).
// ---------------------------------------------------------------------------

static bool udsSendFrame(uint32_t canId, const uint8_t *payload, uint8_t payloadLen)
{
    if (payloadLen > 7) return false;
    twai_message_t msg{};
    msg.identifier = canId;
    msg.extd       = 0;
    msg.rtr        = 0;
    msg.data_length_code = 8;
    msg.data[0] = uint8_t(0x00 | payloadLen); // SF PCI byte
    memcpy(&msg.data[1], payload, payloadLen);
    for (uint8_t i = payloadLen + 1; i < 8; i++) msg.data[i] = 0xAA; // ISO-TP padding
    return canTransmit(twaiBus1, &msg);
}

// DiagnosticSessionControl. Waits for the 0x50 ack (or NRC) so the queue is
// clean before polling starts. Returns true on a positive response.
static bool udsSessionControl(uint32_t reqId, uint8_t session, uint32_t windowMs)
{
    const uint8_t sessReq[] = {0x10, session};
    if (!udsSendFrame(reqId, sessReq, sizeof(sessReq))) return false;

    const uint32_t deadline = millis() + windowMs;
    twai_message_t rsp;
    for (;;)
    {
        const uint32_t now = millis();
        if ((int32_t)(deadline - now) <= 0) return false;
        const uint32_t remaining = deadline - now;
        if (xQueueReceive(udsRxQueue, &rsp, pdMS_TO_TICKS(remaining < 50 ? remaining : 50)) != pdTRUE)
            continue;
        if ((rsp.data[0] & 0xF0) != 0x00 || rsp.data_length_code < 3) continue;
        if (rsp.data[1] == 0x50 && rsp.data[2] == session) return true;
        if (rsp.data[1] == 0x7F && rsp.data[2] == 0x10) return false;
    }
}

static void udsClearValues()
{
    udsTerminalVoltage = 0.0f;
    udsModuleTemp      = 0.0f;
    udsClutchTemp      = 0.0f;
    udsCoolingFinTemp  = 0.0f;
    udsClutchCurrent   = 0.0f;
    udsClutchPWM       = 0;
    udsClutchVoltage   = 0.0f;
    udsClutchTorquePct = 0.0f;
    udsAxleSpeedDiff   = 0.0f;
}
// ---- KWP2000-over-VW-TP2.0 diagnostics — Gen2 / Gen4 (PQ) Haldex ----

// A 16-bit CAN id is carried in the setup frame as [low8][ (id>>8)&0x0F | valid-flag ],
// where bit 4 (0x10) of the prefix means "invalid / not specified".
static void tp20EncodeId(uint32_t id, bool valid, uint8_t &low, uint8_t &prefix)
{
    low = (uint8_t)(id & 0xFF);
    prefix = (uint8_t)((id >> 8) & 0x0F);
    if (!valid) prefix |= 0x10;
}

static uint32_t tp20DecodeId(uint8_t low, uint8_t prefix, bool &valid)
{
    valid = (prefix & 0xF0) == 0;
    return (((uint32_t)(prefix & 0x0F)) << 8) | low;
}

static bool tp20SendRaw(uint32_t canId, const uint8_t *data, uint8_t len)
{
    if (len == 0 || len > 8) return false;
    twai_message_t msg{};
    msg.identifier = canId;
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = len;
    memcpy(msg.data, data, len);
    return canTransmit(twaiBus1, &msg);
}

// Receive the next TP2.0 frame matching expectedId, waiting until the absolute
// deadline (millis timebase). Returns false on timeout.
static bool tp20RecvId(uint32_t expectedId, twai_message_t &out, uint32_t deadlineMs)
{
    while ((int32_t)(deadlineMs - millis()) > 0)
    {
        uint32_t remaining = deadlineMs - millis();
        twai_message_t f{};
        if (xQueueReceive(tp20RxQueue, &f, pdMS_TO_TICKS(remaining < 20 ? (remaining ? remaining : 1) : 20)) != pdTRUE)
            continue;
        if (!f.extd && (f.identifier & 0x1FFFFFFF) == expectedId)
        {
            out = f;
            return true;
        }
    }
    return false;
}

static bool tp20SendAck(uint32_t testerTxId, uint8_t recvSeq)
{
    const uint8_t ack = (uint8_t)(0xB0 | ((recvSeq + 1) & 0x0F));
    return tp20SendRaw(testerTxId, &ack, 1);
}

static void tp20Disconnect(uint32_t testerTxId)
{
    if (testerTxId != 0)
    {
        const uint8_t disc = 0xA8; // channel disconnect
        tp20SendRaw(testerTxId, &disc, 1);
    }
    kwpTp20EcuTxId = 0;
}

// VW/VAG KWP2000 measuring-block scaling. Each measured value is a 3-byte
// group [formula][a][b]; the formula index selects the conversion. Only the
// formulas the Gen4 ST10 Haldex (0BR / 0BS / 2K0 / 0AY907554C) serves for the
// polled groups are implemented; unknown formulas fall back to raw b. Confirmed
// against VCDS-displayed values and the unit's own group table (0BR sw3016 @0x269A2,
// leaf_trace/diag_gen4_0br.md):
//   0x06: 0.001*a*b      V   (voltage)      0x0E: 0.005*a*b       bar (pressure; the unit always sends b = 200)
//   0x18: 0.001*a*b      A   (current)      0x1A: b - a           °C  (temperature)
//   0x21: 0.01*a*b       %   (duty)         0x5E: 0.1*a*(b-128)   Nm  (torque)
//   0x02: 0.002*a*b      %   (Gen2 valve duty, group 0x33 - after the level-3 login)
// Gen2 (7H0 / 1K0) also uses 0x25 = text code a*256+b (switch states / CAN status - decoded
// from b in tp20DecodeGen2, never through here). leaf_trace/diag_gen2_7h0.md.
static float vagScale(uint8_t formula, uint8_t a, uint8_t b)
{
    switch (formula)
    {
    case 0x06: return 0.001f * a * b;                    // supply voltage: a=100,b=2 -> 0.2 V
    case 0x0E: return 0.005f * a * b;                    // pressure:       a=27,b=200 -> 27 bar (standard VAG 0x0E)
    case 0x18: return 0.001f * a * b;                    // valve current:  a=10,b=191 -> 1.910 A
    case 0x1A: return (float)((int)b - (int)a);          // temperature:    a=0,b=24  -> 24 °C
    case 0x21: return 0.01f * a * b;                     // clutch duty:    a=100,b=40 -> 40 %
    case 0x02: return 0.002f * a * b;                    // duty (Gen2):    a=200,b=100 -> 40 %
    case 0x5E: return 0.1f * a * ((int)b - 128);         // est. torque:    a=160,b=255 -> 2032 Nm
    default:   return (float)b;                          // unknown formula - raw high byte
    }
}

// Gen2 SecurityAccess level 3 (V11.00.10, leaf_trace/diag_gen2_7h0.md §6): 27 03 -> 67 03 + 4-byte
// [REA audit_gen2_7h0 #84/#85/F11: confirmed exactly; sendKey without a fresh 67 03 -> 7F 27 12]
// seed (BE), 27 04 + key (BE) -> 67 04 34. key = seed + 0x762B, fixed in 7H0 and 1K0 code (checked
// against a real seed/key pair in a 1K0 EEPROM dump) - no login code. It unlocks measuring groups
// 0x32-0x37 (pump current, valve duty / current) for as long as the TP2.0 channel stays open.
// The unit allows 3 wrong keys, then LOCKS ALL LEVELS and stores the lockout in EEPROM (it survives
// a power cycle; 7F 27 37 = delay running, ~10 min, every request restarts it). So: one key per
// channel, and after a single wrong key we NEVER try again - the refusal (NRC, seed, key) is SAVED
// in our settings (Preferences "g2LoginNrc" / "g2LoginSeed" / "g2LoginKey") so it survives reboots
// and is shown as a warning on the Gen2 live-data card, until someone presses "Allow one more try"
// (POST /api/kwp/gen2login/reset -> gen2LoginReset). On 36 / 37 we wait 15 min before asking again.
static uint32_t gen2Key(uint32_t seed) { return seed + 0x0000762BUL; }

static void gen2LoginLoad()
{
    if (gen2LoginLoaded) return;
    gen2LoginLoaded = true;
    kwpGen2LoginNrc = pref.getUChar("g2LoginNrc", 0);
    kwpGen2LoginSeed = pref.getUInt("g2LoginSeed", 0);
    kwpGen2LoginKey = pref.getUInt("g2LoginKey", 0);
}

static void gen2LoginRefused(uint8_t nrc, uint32_t seed, uint32_t key)
{
    kwpGen2LoginSeed = seed;
    kwpGen2LoginKey = key;
    kwpGen2LoginNrc = nrc; // last: the API reads this as "refusal recorded"
    pref.putUInt("g2LoginSeed", seed);
    pref.putUInt("g2LoginKey", key);
    pref.putUChar("g2LoginNrc", nrc);
}

static void tp20ClearValues()
{
    kwpOilTemp = kwpPlateTemp = kwpSupplyVoltage = 0.0f;
    kwpOilPressure = kwpEstTorque = kwpClutchDuty = kwpClutchValveCurrent = 0.0f;
    kwpPumpDuty = kwpPumpCurrent = 0.0f;
    kwpBrakeSwitch = -1;
    kwpCanStatus = kwpCanStatusValid = 0;
}

static uint8_t dtcProtoFor(uint8_t gen)
{
    switch (gen)
    {
    case 50:
    case 51:
    case 52:
    case GEN_VOLVO_SPC_GEN5: // V11.00.13 (dtc_check_volvo): plain UDS 0x738 -> 0x638, 19 02 / 14 FF FF FF, no security
    case GEN_LANDROVER_GEN4: // V11.00.13 (dtc_check_ford_lr.md): UDS 0x795 -> 0x79D, default session, no security;
                             // 19 02 <mask> -> 59 02 FF + 4-byte records (<= 32 FL2 / 34 Evoque = 139 B, fits rsp[256]);
                             // 14 FF FF FF -> 54 (another group = NRC 0x31). Same on FL2 and Evoque. Untested on a unit.
        return DTC_PROTO_UDS;
    case 2:
    case 4:
        return DTC_PROTO_KWP;
    case GEN_VOLVO_P2_GEN3:
    case GEN_VOLVO_P2_GEN2:
    case GEN_VOLVO_GEN4:
        return DTC_PROTO_VOLVO_D2;
    case GEN_FORD_KUGA:
        return DTC_PROTO_FORD_KWP;
    default:
        return DTC_PROTO_NONE;
    }
}

static void dtcAdd(const char *code, const char *detail)
{
    if (dtcCount >= DTC_MAX) return;
    snprintf(dtcList[dtcCount].code, sizeof(dtcList[dtcCount].code), "%s", code);
    snprintf(dtcList[dtcCount].detail, sizeof(dtcList[dtcCount].detail), "%s", detail);
    dtcCount++;
}

// Borrow the /api/uds/read copy-tap on Bus 1 for one reply ID
static bool dtcTapBegin(uint32_t respId)
{
    if (udsWebRespId != 0) return false; // a web UDS read / serial-lab request is in flight
    if (udsWebRxQueue == nullptr) udsWebRxQueue = xQueueCreate(8, sizeof(twai_message_t));
    if (udsWebRxQueue == nullptr) return false;
    xQueueReset(udsWebRxQueue);
    udsWebBus = 1;
    udsWebRespId = respId;
    return true;
}

static void dtcTapEnd()
{
    udsWebRespId = 0;
}

static void dtcUdsStatusText(uint8_t st, char *out, size_t cap)
{
    snprintf(out, cap, "status %02X%s%s%s%s%s", st,
             (st & 0x01) ? ", active" : "",
             (st & 0x08) ? ", stored" : "",
             (st & 0x04) ? ", pending" : "",
             (st & 0x10) ? ", not tested since clear" : "",
             (st & 0x80) ? ", warning lamp" : "");
}

// What the P3 bench taught us about its codes (not Volvo's own texts) [bench][no OEM code - no P3 dump]
static const char *volvoG4DtcNote(uint16_t code)
{
    switch (code)
    {
    case 0x0015: return "no CEM configuration (0x10A2407C page 0x14) - unit limps";
    case 0xE003: return "network frames missing (logged before they started)";
    case 0x0012: return "wheel-speed signal invalid (bench: a front wheel-speed qualifier not 3)";
    default: return "";
    }
}

static void fordDiagClear()
{
    fordDiagConnected = false;
    fordDiagValid = 0;
    fordDiagVariant = 0;
    fordDiagIdx = 0;
    fordDiagMisses = 0;
    fordDiagLastGoodMs = 0;
}

static void udsDecodeDID(uint16_t did, const twai_message_t &frame)
{
    // Validate: single-frame (PCI high nibble 0), positive RDBI response (0x62), DID echo matches.
    if ((frame.data[0] & 0xF0) != 0x00) return;
    const uint8_t payloadLen = frame.data[0] & 0x0F;
    if (payloadLen < 4) return;                                      // need SID(1) + DID(2) + data(1+)
    if (frame.data[1] != 0x62) return;
    const uint16_t respDID = ((uint16_t)frame.data[2] << 8) | frame.data[3];
    if (respDID != did) return;

    udsLastDecodeMs = millis(); // freshness for the BLE Diag characteristic (values persist after polling stops)

    // Actual measurement data starts at frame.data[4]
    switch (did)
    {
    case 0x0286: // Terminal Voltage: 1 byte, × 0.1 V  (0x8F=143 → 14.3 V confirmed)
                 // [KM -] [REA 0AY audit_gen5_0ay #61: sat_add(0xCBB6 (ch10) x 248/10, 50) mV / 100, 1 byte]
                 // [REA 0CQ audit_gen5_0cq G6: (CC28 x 248/10 + 50)/100, CC28 = IIR(ADC0 ch10, 9/64); VAQ same from CC66 (diag §3)]
        if (payloadLen >= 4)
            udsTerminalVoltage = frame.data[4] * 0.1f;
        break;

    case 0x028D: // Control Module Temperature: byte 0 = T + 55 °C (whole °C, -55..150), byte 1 always 0.
                 // V11.00.10 firmware trace (0CQ sw7082, 0AY sw7521/7543, VAQ): a REAL sensor of its own -
                 // ADC ch15, linear 10 mV/°C IC sensor (T = adc x 0.488 - 50), own filter + range checks.
                 // 0x2BF1 is a second sensor of the same type on ch13 (the firmware's backup when this one
                 // fails), which is why the two track each other. Neither is in the clutch pack.
                 // Heat-sweep confirmed (VCDS, both modules):
                 //   0CQ 554C: 0x4F=79 → 24 °C … 0x76=118 → 63 °C (probe ~25→68)
                 //   0CQ 554D: 0x52=82 → 27 °C … 0x7F=127 → 72 °C (probe ~27→73)
                 // Offset 55 cross-checked against clutch temp at all 4 anchors → mean 55.0.
                 // [KM -] [REA 0AY audit_gen5_0ay #62: (0xCCD8 - 2731)/10 clamp -55..150 -> [T+55, 0x00];
                 // "real ch15 sensor" = #74 UNVERIFIABLE beyond 0xCCD8 (diag_gen5_0ay §3)]
                 // [REA 0CQ audit_gen5_0cq G5/G7: len 2, b0 = clamp((CD52-2731)/10, -55..150)+55, b1 = 0; CD52 =
                 // conv(IIR ADC0 ch15), the unit checks CHNR = 15 in the ADC result. VAQ: CD92, same (audit §4)]
        if (payloadLen >= 4)
            udsModuleTemp = (float)frame.data[4] - 55.0f;
        break;

    case 0x2BE6: // PUMP MOTOR current (VCDS "clutch current"): BE16 mA + a fixed 5 the firmware adds
                 // (0x0005 = 0 A). Measured: current ADC x EEPROM gain/offset, filtered (V11.00.10 trace).
                 // [KM -] [REA 0AY audit_gen5_0ay #66/F2: I + 5, BE16, I = (0xCADE:CAE0 x 0xCCCC (ch0)) >> 14;
                 // encoding CONFIRMED, the mA unit is INFERRED, not proven]
                 // [REA 0CQ audit_gen5_0cq G10: BE16 = I + 5, I = (CB46:48 x CD46 + CB4A:4C) >> 14, mA INF;
                 // VAQ FUN_c20f6a, same Q14 (diag §3)]
        if (payloadLen >= 5)
        {
            const uint16_t raw = ((uint16_t)frame.data[4] << 8) | frame.data[5];
            udsClutchCurrent = (raw > 5 ? raw - 5 : 0) * 0.001f;
        }
        break;

    case 0x2BE7: // PUMP PWM duty as commanded: 1 byte, raw %  (0x00=0 → 0 % confirmed)
                 // [KM -] [REA 0AY audit_gen5_0ay #67: (0xCB9C + 5)/10, 0xCB9C = PWM duty 0.1 % clamp 0..990]
                 // [REA 0CQ audit_gen5_0cq G11: 1 B = (CC0E + 5)/10, CC0E duty 0.1 % clamp 0..990; VAQ from CC4C]
        if (payloadLen >= 4)
            udsClutchPWM = frame.data[4];
        break;

    case 0x2BF1: // "Clutch Temperature" (VCDS) = control-unit temperature SENSOR 2 (ADC ch13, same sensor type
                 // and curve as 0x028D) - V11.00.10 trace. LE16, (D6×256+D5 − 22767)/100 °C
                 // Heat-sweep confirmed (VCDS, both modules); tracks 0x028D within ~1 °C:
                 //   0CQ 554C: 25227 → 24.6 °C … 28987 → 62.2 °C
                 //   0CQ 554D: 25477 → 27.1 °C … 29967 → 72.0 °C
                 // [KM -] [REA 0AY audit_gen5_0ay #63: 10 x 0xCCDA (ch13, 0.1 K) - 4543, LE16, limits 455 / 7007]
                 // [REA 0CQ audit_gen5_0cq G8: LE16 = CD54 x 10 - 4543 (CD54 < 455 -> 0, > 7007 -> 0xFFF1), ADC0 ch13;
                 // VAQ CD94, same (audit §4)]
        if (payloadLen >= 5)
            udsClutchTemp = ((int32_t)((uint16_t)frame.data[5] * 256 + frame.data[4]) - 22767) / 100.0f;
        break;

    case 0x2BE4: // Cooling Fin Temperature - a MODEL (node of the unit's 5-node thermal model), whole °C:
                 // LE16, raw = 100 x T + 22757 (V11.00.10 trace; was decoded with 22767 = 0.1 °C low).
                 // Fins shed heat so they stay cooler:
                 //   0CQ 554C: 25057 → 22.9 °C … 26957 → 41.9 °C
                 //   0CQ 554D: 25357 → 25.9 °C … 25657 → 28.9 °C (barely moved, as observed)
                 // [KM -] [REA 0AY audit_gen5_0ay #65: (0xCE6A + 273) x 10 -> 0x2BF1 encoding = 100T + 22757, LE16]
                 // [REA 0CQ audit_gen5_0cq G9/G18: (CEEE + 273) x 10 then the 0x2BF1 encoding; CEEE = thermal-model
                 // node 3 (0xC82C), node meaning ("fin") UNVERIFIED; VAQ CF34 (audit §4)]
        if (payloadLen >= 5)
            udsCoolingFinTemp = ((int32_t)((uint16_t)frame.data[5] * 256 + frame.data[4]) - 22757) / 100.0f;
        break;

    case 0x2BE9: // PUMP MOTOR voltage, CALCULATED (not measured): duty x supply - current x R_winding(T)
                 // ~= back-EMF, a pump-speed proxy. ONE byte x 0.1 V - the second byte is never written
                 // (V11.00.10 trace, 0CQ / 0AY / VAQ): the old LE16 read turned a leftover 0x57 there into
                 // ~2227 V (554C sw7754 log). Bench 2026-09-20 (0CQ, 61 % PWM, 13.4 V): 50 00 -> 8.0 V.
                 // [KM -] [REA 0AY audit_gen5_0ay #68: (V + 50)/100, one byte, V = duty x Vsup - I x R(T) - K(T); back-EMF INF]
                 // [REA 0CQ audit_gen5_0cq G12: 1 B = (V + 50)/100, R(T) 28..41 @ -30..100 degC, U0 = 0; VAQ 1 byte too]
        if (payloadLen >= 4)
            udsClutchVoltage = (float)frame.data[4] * 0.1f;
        break;

    case 0x2BF3: // Clutch torque APPLIED, 1 byte (V11.00.10 trace) - the same value the unit sends on CAN:
                 // 0CQ / VAQ = Allrad_03 (0x118) byte 2, 0..250 = 0..100 %; 0AY = 0x2C0 byte 1 without its
                 // 0x80 offset, torque / 112, 0..125 = 0..100 %. 0 when open or in a fault state.
                 // [KM -] [REA 0AY audit_gen5_0ay #69: round(0xCE70 / 112) cap 0xFA, 0 when 0xCB9E in {2,3,4};
                 // 0xCE70 = the 0x2C0 b1 torque, so x 0.8 = the same % scale as the lock request / b1 read-back]
                 // [REA 0CQ audit_gen5_0cq G13: 1 B = min(round(CEF2/128), 250), 0 in CC10 {2,3,4} - same value as
                 // 0x118 b2 (record len 2, 1 byte written)]
                 // [REA VAQ audit G19/F10: CF38 via cal 0xC0B67E/80, which is in the MISSING calibration block -
                 // the "/128, 0.4 %/bit" VAQ scale is an assumption, bench-checked only]
        if (payloadLen >= 4)
        {
            const float pct = (haldexGeneration == 51) ? frame.data[4] * 0.8f : frame.data[4] * 0.4f;
            udsClutchTorquePct = pct > 100.0f ? 100.0f : pct;
        }
        break;

    case 0x2BF0: // Axle speed difference |front avg - rear avg| from the wheel speeds, BE16 x 0.1 km/h
                 // (0CQ / 0AY only - the VAQ has no 0x2BF0). V11.00.10 trace.
                 // [KM -] [REA 0AY audit_gen5_0ay #70: |avg(Bremse_3 VL,VR)/10 - avg(HL,HR)/10| clamp 0xCBC, BE16;
                 // NRC without Bremse_3; 0.1 km/h assumes Bremse_3 0.01 km/h/bit (INF)]
                 // [REA 0CQ audit_gen5_0cq G14/G15: BE16 = min(|avg(FL,FR) - avg(RL,RR)|, 3260) of
                 // (ESP_19 raw x 75 + 500)/1000, NRC if PDU 0x0C bad; VAQ: absent, has 0x2F05 (L/R diff) instead]
        if (payloadLen >= 5)
            udsAxleSpeedDiff = (float)(((uint16_t)frame.data[4] << 8) | frame.data[5]) * 0.1f;
        break;

    default:
        break;
    }
}

// Send one RDBI for `did` and drain udsRxQueue until the matching positive
// response, a negative response for service 0x22, or the window expires.
// Positive responses for OTHER DIDs that land meanwhile are still decoded, so
// an out-of-order / stale reply is never wasted. TesterPresent acks and any
// other frames are discarded.
static UdsPollResult udsPollDid(uint32_t reqId, uint16_t did, uint32_t windowMs)
{
    const uint8_t rdbiReq[] = {0x22, (uint8_t)(did >> 8), (uint8_t)(did & 0xFF)};
    if (!udsSendFrame(reqId, rdbiReq, sizeof(rdbiReq))) return UDS_POLL_TIMEOUT;

    const uint32_t deadline = millis() + windowMs;
    twai_message_t rsp;
    for (;;)
    {
        const uint32_t now = millis();
        if ((int32_t)(deadline - now) <= 0) break;
        const uint32_t remaining = deadline - now;
        if (xQueueReceive(udsRxQueue, &rsp, pdMS_TO_TICKS(remaining < 50 ? remaining : 50)) != pdTRUE)
            continue;
        if ((rsp.data[0] & 0xF0) != 0x00 || rsp.data_length_code < 4) continue; // not a single frame we care about

        if (rsp.data[1] == 0x62)
        {
            const uint16_t respDID = ((uint16_t)rsp.data[2] << 8) | rsp.data[3];
            udsDecodeDID(respDID, rsp);
            if (respDID == did) return UDS_POLL_OK;
            continue; // reply to an earlier request - keep waiting for ours
        }
        if (rsp.data[1] == 0x7F && rsp.data[2] == 0x22)
        {
            // ISO 14229 NRCs that mean "not in this session": 0x7E subFunction /
            // 0x7F serviceNotSupportedInActiveSession, 0x33 securityAccessDenied.
            // 0x22 conditionsNotCorrect is about vehicle state (speed etc.), not
            // the session, so it must NOT push us into extended. 0x78 (response
            // pending) is not final - keep waiting for the real answer.
            const uint8_t nrc = rsp.data[3];
            if (nrc == 0x78) continue;
            if (nrc == 0x7E || nrc == 0x7F || nrc == 0x33) return UDS_POLL_NRC_SESSION;
            return UDS_POLL_NRC_OTHER;
        }
        // 0x7E TesterPresent ack, 0x50 session ack, anything else: discard
    }
    return UDS_POLL_TIMEOUT;
}

void udsApplyDefaultIds()
{
    if (udsIdsManual) return; // pinned from the serial lab (DIAGID) - leave alone
    if (haldexGeneration == 52)
    {
        udsHaldexReqId  = ISO_QUERSPERRE_REQ;  // 0x71E
        udsHaldexRespId = ISO_QUERSPERRE_RESP; // 0x788
    }
    else if (haldexGeneration == GEN_FORD_KUGA)
    {
        udsHaldexReqId  = FORD_KUGA_DIAG_REQ;  // 0x761 (V11.00.9, bench)
        udsHaldexRespId = FORD_KUGA_DIAG_RESP; // 0x769
    }
    else if (haldexGeneration == GEN_LANDROVER_GEN4)
    {
        udsHaldexReqId  = LR_DIAG_REQ;  // 0x795 (V11.00.13, dtc_check_ford_lr.md - FL2 + Evoque the same)
        udsHaldexRespId = LR_DIAG_RESP; // 0x79D (answered from the unit's other CAN controller - can.cpp taps both buses)
    }
    else if (haldexGeneration == GEN_VOLVO_SPC_GEN5)
    {
        udsHaldexReqId  = VOLVO_SPC_DIAG_REQ;  // 0x738 (V11.00.13, OEM code: dtc_check_volvo - CanIf RX PDU 1)
        udsHaldexRespId = VOLVO_SPC_DIAG_RESP; // 0x638 (CanIf TX PDU 2)
    }
    else
    {
        udsHaldexReqId  = ISO_ALLRAD_REQ;  // 0x70F
        udsHaldexRespId = ISO_ALLRAD_RESP; // 0x779
    }
}

// ===========================================================================
// KWP2000-over-VW-TP2.0 diagnostics — Gen2 / Gen4 (PQ) Haldex
// ===========================================================================
// The PQ-platform Haldex (AWD) controller is diagnosed with KWP2000 tunnelled
// over VW TP2.0, not raw UDS/ISO-TP. Sequence:
//   1. Broadcast a TP2.0 channel-setup (0xC0) to KWP_TP20_SETUP_TX_ID (0x200)
//      targeting logical address KWP_TP20_HALDEX_ADDR (0x0A). The ECU answers
//      on KWP_TP20_SETUP_RX_ID (0x20A) with 0xD0 and the negotiated data IDs
//      (confirmed 1K0 Gen2 capture: tester TX 0x764, ECU TX 0x300).
//   2. Negotiate channel timing parameters (0xA0 / 0xA1).
//   3. Open a KWP session: StartDiagnosticSession 0x10 0x89.
//   4. Poll measuring blocks: ReadDataByLocalIdentifier 0x21, group N.
//
// This build performs a RAW capture only: each group's response bytes
// (formula-id + a/b per value) are dumped into kwpTp20RawDump so real scaling
// — especially temperatures — can be confirmed from a bench unit before it is
// hard-coded. All frames go on Bus 1 (Haldex) and are routed here by
// parseCANHdx via tp20RxQueue.
// ===========================================================================

// Open the TP2.0 channel and negotiate timing. On success testerTxId/ecuTxId
// hold the data-channel IDs and kwpTp20EcuTxId is set so RX routing is live.
static bool tp20OpenChannel(uint32_t &testerTxId, uint32_t &ecuTxId, uint32_t timeoutMs)
{
    xQueueReset(tp20RxQueue);
    kwpTp20EcuTxId = 0;

    uint8_t rxLow, rxPre, txLow, txPre;
    tp20EncodeId(0, false, rxLow, rxPre);                    // let the ECU assign the tester->ECU id
    tp20EncodeId(KWP_TP20_TESTER_RX_ID, true, txLow, txPre); // ask the ECU to transmit on 0x300

    const uint8_t setup[7] = {
        KWP_TP20_HALDEX_ADDR, 0xC0, rxLow, rxPre, txLow, txPre, 0x01 /* application = KWP2000 */
    };
    if (!tp20SendRaw(KWP_TP20_SETUP_TX_ID, setup, sizeof(setup))) return false;

    twai_message_t resp{};
    if (!tp20RecvId(KWP_TP20_SETUP_RX_ID, resp, millis() + timeoutMs)) return false;
    if (resp.data_length_code < 7 || resp.data[1] != 0xD0) return false; // 0xD0 = setup positive

    bool ecuValid = false, testerValid = false;
    ecuTxId    = tp20DecodeId(resp.data[2], resp.data[3], ecuValid);
    testerTxId = tp20DecodeId(resp.data[4], resp.data[5], testerValid);
    if (!ecuValid || !testerValid || ecuTxId == 0 || testerTxId == 0) return false;
    kwpTp20EcuTxId = ecuTxId; // enable RX routing for the negotiated data channel

    // Channel timing parameters (0xA0 request / 0xA1 response). Byte values from
    // confirmed VW captures: 0F 8A FF 32 FF.
    const uint8_t params[6] = {0xA0, 0x0F, 0x8A, 0xFF, 0x32, 0xFF};
    if (!tp20SendRaw(testerTxId, params, sizeof(params))) return false;
    twai_message_t pr{};
    if (!tp20RecvId(ecuTxId, pr, millis() + 500)) return false;
    if (pr.data_length_code < 1 || (pr.data[0] & 0xF0) != 0xA0) return false; // 0xA1 = param ack

    return true;
}

// Send a single-telegram KWP request and collect the response. Returns the KWP
// payload length (>=1) on success, 0 on failure/timeout. txSeq is the running
// TP2.0 send sequence and is advanced across calls.
static uint8_t kwpTp20Transact(uint32_t testerTxId, uint32_t ecuTxId, const uint8_t *req,
                               uint8_t reqLen, uint8_t &txSeq, uint8_t *out, uint8_t outCap,
                               uint32_t timeoutMs)
{
    if (reqLen == 0 || reqLen > 5) return 0;

    // Single first+last data frame: [0x1|seq][len_hi=0][len_lo=reqLen][payload...]
    uint8_t frame[8] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
    frame[0] = (uint8_t)(0x10 | (txSeq & 0x0F)); // high nibble 1 = last frame, ACK expected
    frame[1] = 0x00;
    frame[2] = reqLen;
    memcpy(&frame[3], req, reqLen);
    if (!tp20SendRaw(testerTxId, frame, (uint8_t)(reqLen + 3))) return 0;

    const uint8_t expectAck = (uint8_t)((txSeq + 1) & 0x0F);
    txSeq = expectAck;
    uint32_t deadline = millis() + timeoutMs; // V11.00.13: not const - a "response pending" pushes it out

    // Wait for the ECU ACK (0xBx). The ECU may skip it and start the response
    // telegram directly, so requeue any data frame we see instead.
    while ((int32_t)(deadline - millis()) > 0)
    {
        twai_message_t f{};
        if (!tp20RecvId(ecuTxId, f, deadline)) return 0;
        if (f.data_length_code == 0) continue;
        const uint8_t op = f.data[0] >> 4;
        if (op == 0x0B) { if ((f.data[0] & 0x0F) == expectAck) break; continue; }
        if (op <= 0x03) { (void)xQueueSendToFront(tp20RxQueue, &f, 0); break; }
        // 0x09 (not-ready ACK) / channel mgmt: ignore
    }

    // Collect the response telegram, ACKing frames that request it.
    uint16_t total = 0, off = 0;
    bool haveFirst = false;
    while ((int32_t)(deadline - millis()) > 0)
    {
        twai_message_t f{};
        if (!tp20RecvId(ecuTxId, f, deadline)) return 0;
        if (f.data_length_code == 0) continue;
        const uint8_t op  = f.data[0] >> 4;
        const uint8_t seq = f.data[0] & 0x0F;
        if (op == 0x0B || op == 0x09) continue; // stray ACK
        if (op > 0x03) continue;                // channel management

        if (!haveFirst)
        {
            if (f.data_length_code < 3) return 0;
            total = ((uint16_t)f.data[1] << 8) | f.data[2];
            if (total == 0 || total > outCap) return 0;
            for (uint8_t i = 3; i < f.data_length_code && off < total; i++) out[off++] = f.data[i];
            haveFirst = true;
        }
        else
        {
            for (uint8_t i = 1; i < f.data_length_code && off < total; i++) out[off++] = f.data[i];
        }

        if (op == 0x00 || op == 0x01) (void)tp20SendAck(testerTxId, seq); // ECU is waiting for our ACK

        if (haveFirst && off >= total)
        {
            // V11.00.13 (dtc_check_vag_kwp): the 7H0 answers a clear with 7F 14 78 ("response pending")
            // and only then 54 FF 00 - keep listening for the real answer instead of reporting a refusal.
            if (total >= 3 && out[0] == 0x7F && out[2] == 0x78)
            {
                haveFirst = false;
                off = 0;
                total = 0;
                deadline = millis() + 5000;
                continue;
            }
            return (uint8_t)total;
        }
    }
    return 0;
}

// Decode the Gen4 measuring groups into the named kwp* globals (V11.00.10: checked against
// the 0BR sw3016 firmware, leaf_trace/diag_gen4_0br.md - same layout on 0BS sw4013, 0BR sw3017,
// 2K0 sw5010 and the PQ 0AY907554C, which is a Gen4 KWP unit).
// buf = [61][GG][f a b][f a b]... ; measured value k starts at buf[2 + 3k].
static void tp20DecodeGen4(uint8_t group, const uint8_t *buf, uint8_t n)
{
    auto value = [&](uint8_t k) -> float {
        uint8_t off = (uint8_t)(2 + 3 * k);
        if ((uint16_t)off + 3 > n) return 0.0f;
        return vagScale(buf[off], buf[off + 1], buf[off + 2]);
    };
    switch (group)
    {
    // Per field: [KM -] (KWP measuring groups have no KMatrix signal) + [REA diag_gen4_0br /
    // audit_gen4_0br #] = what the 0BR's group handler puts there (all CONFIRMED in the audit).
    case 0x01: // [REA handler 0x3588E]
        kwpOilTemp       = value(0); // [KM -] [REA #53: f 0x1A, E48A oil temperature SENSOR (ADC ch2 + NTC curve), whole degC]
        kwpPlateTemp     = value(1); // [KM -] [REA #54 UNVERIFIABLE: f 0x1A, plate temperature MODEL FUN_0408cc (oil temp + torque x slip heat) - no sensor]
        kwpSupplyVoltage = value(2); // [KM -] [REA #55: f 0x06, a=100, b=(mV+200)/100 - ADC ch8, unit adds +0.2 V]; field 4 = 0x25 empty
        break;
    case 0x03: // [REA handler 0x35A7E]
        kwpOilPressure        = value(0); // [KM -] [REA #56/#62: f 0x0E, a = F1B0C/195, b = 200 -> whole bar; pressure TARGET
                                          // FUN_040614(F1B08), NOT the sensor: the ch5 sensor is computed but never read]
        kwpEstTorque          = value(1); // [KM -] [REA #41/#57: f 0x5E, F1B08 in 1/16 Nm = COMMANDED min(F1A74, F1AFE), saturates at 2032 Nm]
        kwpClutchDuty         = value(2); // [KM -] [REA #58: f 0x21, a=100, b = E4FC/10 - valve PWM duty (command), clamp 0..90 %]
        kwpClutchValveCurrent = value(3); // [KM -] [REA #59: f 0x18, a=10, b = mA/10 - valve current MEASURED (ADC ch11), 0.01 A steps]
        break;
    case 0x04: // [REA handler 0x35B62] fields 0/1 = pump state text "HI"/"LO" (formula 0x11, not decoded)
        kwpPumpDuty    = value(2);        // [KM -] [REA #60: f 0x21, pump motor PWM duty E5CE (command)]
        kwpPumpCurrent = value(3);        // [KM -] [REA #61/#63: f 0x18, a = 20 (sw3016/4013) or 40 (sw3017/5010), G(duty) x ch10/ch15 current sense; mA INF]
        break;
    default:
        break;
    }
}

// Gen2 (7H0 / 1K0907554B, identical handlers) measuring groups, from the unit's own group table
// (7H0 @0x163D8, 1K0 @0x1620A - leaf_trace/diag_gen2_7h0.md, V11.00.10). Session 0x89 serves
// 01 02 03 50 5A 62 7D; the pump / valve current groups 0x32-0x37 need SecurityAccess (27 03/04,
// gen2SecurityLogin) and answer NRC 0x31 without it. The 1K0 capture matches byte for byte.
static void tp20DecodeGen2(uint8_t group, const uint8_t *buf, uint8_t n)
{
    auto field = [&](uint8_t k, uint8_t &f, uint8_t &a, uint8_t &b) -> bool {
        const uint8_t off = (uint8_t)(2 + 3 * k);
        if ((uint16_t)off + 3 > n) return false;
        f = buf[off];
        a = buf[off + 1];
        b = buf[off + 2];
        return true;
    };
    uint8_t f, a, b;
    switch (group)
    {
    case 0x01: // brake-light switch (ADC ch7, text code 00C1 / 00A7), supply voltage (ch8, +0.2 V)
        if (field(0, f, a, b) && f == 0x25) // 01.1 [KM -] [REA audit_gen2_7h0 #79 / diag §3: 25 00 A7 (E4B0, ch7 low = braking) / C1]
            kwpBrakeSwitch = (b == 0xA7) ? 1 : (b == 0xC1 ? 0 : -1);
        if (field(1, f, a, b)) // 01.2 [KM -] [REA audit_gen2_7h0 #79: 06 64 b, b = (D7FA x 24 + 200)/100 -> V = b/10 incl. +0.2 V]
            kwpSupplyVoltage = vagScale(f, a, b);
        break;               // 01.3/01.4 = 25 00 00 empty [REA diag_gen2_7h0 §3]
    case 0x02: // oil / clutch temperature (NTC ch4), MEASURED clutch oil pressure (ch5, 0.5-4.5 V, 1 bar steps)
        if (field(0, f, a, b)) // 02.1 [KM -] [REA diag_gen2_7h0 §3: 1A a b, b - a = D76C/10 degC (NTC ch4)]
            kwpOilTemp = vagScale(f, a, b);
        if (field(1, f, a, b)) // 02.2 [KM -] [REA audit_gen2_7h0 #80: 0E C8 b, b = filtered ch5 pressure E14C:E14E/1000, whole bar]
            kwpOilPressure = vagScale(f, a, b);
        break;               // 02.3/02.4 = empty [REA diag_gen2_7h0 §3]
    case 0x32: // (logged in) oil temp, commanded set-point (model), PUMP MOTOR current (ADC ch11, measured)
               // 32.1 1A temp, 32.2 36 hi lo = FUN_027acc(D7F8) x 1000/1907 set-point - both not decoded [REA audit_gen2_7h0 #81]
        if (field(2, f, a, b)) // 32.3 [KM -] [REA audit_gen2_7h0 #81: 18 0A b, b = (E010>>4)/10 -> A = b/100; "pump" label INF]
            kwpPumpCurrent = vagScale(f, a, b);
        break;
    case 0x33: // (logged in) valve driver HI/LO, VALVE duty (formula 0x02), VALVE current (ch10), pressure
               // 33.1 11 'H''I'/'L''O' (D751 valve driver enable) and 33.4 0E C8 pressure (= 02.2) not decoded [REA audit_gen2_7h0 #82]
        if (field(1, f, a, b)) // 33.2 [KM -] [REA audit_gen2_7h0 #82: 02 C8 b, b = D75A>>2 -> % = 0.4 x b]
            kwpClutchDuty = vagScale(f, a, b);
        if (field(2, f, a, b)) // 33.3 [KM -] [REA audit_gen2_7h0 #82: 18 14 b, b = valve mA/20 -> A = b/50]
            kwpClutchValveCurrent = vagScale(f, a, b);
        break;
    case 0x7D: // CAN status: engine (DTC 01314), ABS (01316), cluster (01317), steering sensor 0x0C2.
               // Formula 0x25, odd b = received OK, even b = no communication.
        for (uint8_t k = 0; k < 4; k++)
        {
            if (!field(k, f, a, b) || f != 0x25) continue; // 7D.1-4 [KM -] [REA audit_gen2_7h0 #83: 25 01 72/73 engine (cond 0x3C), 74/75 ABS (0x3D),
                                                            //  78/79 cluster (0x3F), 7C/7D 0x0C2 present (RX idx 0x13 7H0 / 0x0C 1K0)]
            kwpCanStatusValid |= (uint8_t)(1u << k);
            if (b & 1)
                kwpCanStatus |= (uint8_t)(1u << k);
            else
                kwpCanStatus &= (uint8_t)~(1u << k);
        }
        break;
    default:
        break;
    }
}

void gen2LoginReset()
{
    pref.remove("g2LoginNrc");
    pref.remove("g2LoginSeed");
    pref.remove("g2LoginKey");
    kwpGen2LoginNrc = 0;
    kwpGen2LoginSeed = kwpGen2LoginKey = 0;
    gen2LoginBlockedMs = 0;
    gen2LoginLoaded = true;
}

static bool gen2SecurityLogin(uint32_t testerTxId, uint32_t ecuTxId, uint8_t &seq)
{
    gen2LoginLoad();
    if (kwpGen2LoginNrc != 0) return false; // a key was refused once - never risk the unit's lockout
    if (gen2LoginBlockedMs != 0 && millis() - gen2LoginBlockedMs < 15UL * 60UL * 1000UL) return false;
    uint8_t buf[16];
    const uint8_t reqSeed[] = {0x27, 0x03};
    uint8_t n = kwpTp20Transact(testerTxId, ecuTxId, reqSeed, sizeof(reqSeed), seq, buf, sizeof(buf), 1000);
    if (n >= 3 && buf[0] == 0x7F && buf[1] == 0x27)
    {
        if (buf[2] == 0x36 || buf[2] == 0x37) gen2LoginBlockedMs = millis() ? millis() : 1;
        kwpGen2LoginBlockedNrc = buf[2]; // shown on the card (not saved - the unit's own delay)
        DEBUG("KWP Gen2 - seed refused (NRC %02X)", buf[2]);
        return false;
    }
    kwpGen2LoginBlockedNrc = 0;
    if (n < 6 || buf[0] != 0x67 || buf[1] != 0x03) return false;
    const uint32_t seed = ((uint32_t)buf[2] << 24) | ((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 8) | buf[5];
    if (seed == 0)
    {
        return true; // a zero seed = level already unlocked (KWP convention) - no key needed [REA audit_gen2_7h0 F10: 7H0/1K0 never send it (answer 7F 27 12) - dead path, harmless]
    }
    const uint32_t key = gen2Key(seed);
    const uint8_t sendKey[] = {0x27, 0x04, (uint8_t)(key >> 24), (uint8_t)(key >> 16), (uint8_t)(key >> 8), (uint8_t)key};
    n = kwpTp20Transact(testerTxId, ecuTxId, sendKey, sizeof(sendKey), seq, buf, sizeof(buf), 1000);
    if (n >= 2 && buf[0] == 0x67 && buf[1] == 0x04)
    {
        DEBUG("KWP Gen2 - security level 3 unlocked");
        return true;
    }
    if (n >= 3 && buf[0] == 0x7F && buf[1] == 0x27)
    {
        if (buf[2] == 0x36 || buf[2] == 0x37)
            gen2LoginBlockedMs = millis() ? millis() : 1;
        else
            gen2LoginRefused(buf[2], seed, key); // 0x35 invalidKey (or anything else): stop for good, saved
        DEBUG("KWP Gen2 - key %08lX for seed %08lX refused (NRC %02X)", (unsigned long)key, (unsigned long)seed, buf[2]);
    }
    return false;
}

UDS::UDS(twai_handle_t canBus, QueueHandle_t rxQueue)
    : _canBus(canBus), _rxQueue(rxQueue)
{
}

bool UDS::sendSingleFrame(uint32_t canId, const uint8_t *payload, uint8_t length)
{
    if (length > 7)
        return false;

    twai_message_t msg{};
    msg.identifier = canId;
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = length + 1;
    msg.data[0] = uint8_t(0x00 | length);
    memcpy(&msg.data[1], payload, length);

    return canTransmit(_canBus, &msg);
}

bool UDS::sendFirstFrame(uint32_t canId, const uint8_t *payload, uint16_t length)
{
    if (length <= 7 || length > 4095)
        return false;

    twai_message_t msg{};
    msg.identifier = canId;
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = 8;
    msg.data[0] = uint8_t(0x10 | ((length >> 8) & 0x0F));
    msg.data[1] = uint8_t(length & 0xFF);
    memcpy(&msg.data[2], payload, 6);

    return canTransmit(_canBus, &msg);
}

bool UDS::sendConsecutiveFrame(uint32_t canId, const uint8_t *payload, uint8_t sequenceCounter, uint8_t length)
{
    if (length > 7 || sequenceCounter > 0x0F)
        return false;

    twai_message_t msg{};
    msg.identifier = canId;
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = length + 1;
    msg.data[0] = uint8_t(0x20 | (sequenceCounter & 0x0F));
    memcpy(&msg.data[1], payload, length);

    return canTransmit(_canBus, &msg);
}

bool UDS::sendFlowControl(uint32_t canId, uint8_t flowStatus, uint8_t blockSize, uint8_t stMin)
{
    twai_message_t msg{};
    msg.identifier = canId;
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = 3;
    msg.data[0] = uint8_t(0x30 | (flowStatus & 0x0F));
    msg.data[1] = blockSize;
    msg.data[2] = stMin;

    return canTransmit(_canBus, &msg);
}

bool UDS::receiveFrame(twai_message_t &frame, uint32_t timeoutMs)
{
    // Queue mode: frames are copied here by the parse task for the selected bus
    // (already filtered to the response ID). Reading the TWAI driver directly
    // would make this a second consumer racing the gateway parse task - every
    // frame this side won was consumed and never forwarded to the Haldex.
    if (_rxQueue != nullptr)
    {
        return xQueueReceive(_rxQueue, &frame, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
    }

    uint32_t start = millis();
    while ((millis() - start) < timeoutMs)
    {
        if (twai_receive_v2(_canBus, &frame, 10 / portTICK_PERIOD_MS) == ESP_OK)
            return true;
        delay(1);
    }
    return false;
}

bool UDS::sendRequest(uint32_t requestId,
                      uint32_t responseId,
                      const uint8_t *requestData,
                      size_t requestLen,
                      uint8_t *responseBuf,
                      size_t &responseLen,
                      uint32_t timeoutMs)
{
    if (requestData == nullptr || responseBuf == nullptr || responseLen == 0)
        return false;

    // responseLen carries the buffer capacity on entry and the received length
    // on return; capture the capacity before responseLen is reused as the
    // output below, otherwise every copy is bounded by 0 and the read returns
    // no data.
    const size_t capacity = responseLen;

    // Send the request (single or multi-frame)
    if (requestLen <= 7)
    {
        if (!sendSingleFrame(requestId, requestData, requestLen))
            return false;
    }
    else
    {
        if (!sendFirstFrame(requestId, requestData, requestLen))
            return false;

        // Wait for Flow Control frame from ECU (on responseId) before sending Consecutive frames
        twai_message_t fc;
        if (!receiveFrame(fc, timeoutMs))
            return false;

        if (fc.identifier != responseId || (fc.data_length_code < 3) || ((fc.data[0] & 0xF0) != 0x30))
            return false;

        uint8_t blockSize = fc.data[1];
        uint8_t stMin = fc.data[2];

        size_t sent = 6;
        uint8_t seq = 1;
        size_t remaining = requestLen - sent;

        while (remaining > 0)
        {
            size_t chunk = (remaining > 7) ? 7 : remaining;
            if (!sendConsecutiveFrame(requestId, &requestData[sent], seq, chunk))
                return false;

            sent += chunk;
            remaining -= chunk;
            seq = (seq + 1) & 0x0F;

            if (blockSize != 0 && (seq % blockSize == 0))
            {
                // wait for next flow control from ECU
                if (!receiveFrame(fc, timeoutMs))
                    return false;
                if (fc.identifier != responseId || (fc.data_length_code < 3) || ((fc.data[0] & 0xF0) != 0x30))
                    return false;
                blockSize = fc.data[1];
                stMin = fc.data[2];
            }

            if (stMin != 0)
            {
                uint16_t sleepMs = (stMin <= 0x7F) ? stMin : 25; // 0x00-0x7F ms; 0xF1..0xF9 for 100..900ms
                delay(sleepMs);
            }
        }
    }

    // Collect response from ECU
    responseLen = 0;
    twai_message_t frame;

    if (!receiveFrame(frame, timeoutMs))
        return false;

    if (frame.identifier != responseId)
        return false;

    auto pci = frame.data[0] & 0xF0;
    if (pci == 0x00)
    {
        // Single Frame
        uint8_t dataLen = frame.data[0] & 0x0F;
        size_t copyLen = min<size_t>(dataLen, capacity);
        memcpy(responseBuf, &frame.data[1], copyLen);
        responseLen = copyLen;
        return true;
    }

    if (pci == 0x10)
    {
        // First Frame
        uint16_t totalLength = ((frame.data[0] & 0x0F) << 8) | frame.data[1];
        size_t bytesCopied = min<size_t>(6, totalLength);
        if (bytesCopied > capacity)
            return false; // buffer too small

        memcpy(responseBuf, &frame.data[2], bytesCopied);
        size_t receivedTotal = bytesCopied;

        if (!sendFlowControl(requestId, 0x00, 0x00, 0x00)) // CTS
            return false;

        uint8_t seq = 1;
        while (receivedTotal < totalLength)
        {
            if (!receiveFrame(frame, timeoutMs))
                return false;
            if (frame.identifier != responseId)
                continue;
            if ((frame.data[0] & 0xF0) != 0x20)
                return false;
            uint8_t payloadLen = min<uint8_t>(7, totalLength - receivedTotal);
            if (payloadLen > capacity - receivedTotal)
                return false;

            memcpy(responseBuf + receivedTotal, &frame.data[1], payloadLen);
            receivedTotal += payloadLen;
            seq = (seq + 1) & 0x0F;
        }

        responseLen = receivedTotal;
        return true;
    }

    return false;
}

bool UDS::readDataByIdentifier(uint32_t requestId,
                               uint32_t responseId,
                               uint16_t did,
                               uint8_t *dataOut,
                               size_t &dataOutLen,
                               uint32_t timeoutMs)
{
    if (dataOut == nullptr || dataOutLen == 0)
        return false;

    uint8_t req[3];
    req[0] = 0x22;
    req[1] = uint8_t((did >> 8) & 0xFF);
    req[2] = uint8_t(did & 0xFF);

    size_t rawRespLen = dataOutLen;
    if (!sendRequest(requestId, responseId, req, sizeof(req), dataOut, rawRespLen, timeoutMs))
        return false;

    if (rawRespLen < 1)
        return false;

    if (dataOut[0] == 0x62)
    {
        // positive response, payload begins at index 1
        size_t payloadLen = rawRespLen - 1;
        memmove(dataOut, dataOut + 1, payloadLen);
        dataOutLen = payloadLen;
        return true;
    }

    // negative response is 0x7F 0x22 <NRC>
    if (rawRespLen >= 3 && dataOut[0] == 0x7F && dataOut[1] == 0x22)
    {
        dataOutLen = 0;
        return false;
    }

    dataOutLen = 0;
    return false;
}

bool UDS::diagnosticSessionControl(uint32_t requestId,
                                   uint32_t responseId,
                                   uint8_t sessionType,
                                   uint32_t timeoutMs)
{
    uint8_t req[2] = {0x10, sessionType};
    uint8_t resp[4];
    size_t respLen = sizeof(resp);

    if (!sendRequest(requestId, responseId, req, sizeof(req), resp, respLen, timeoutMs))
        return false;

    if (respLen >= 2 && resp[0] == 0x50 && resp[1] == sessionType)
        return true;

    return false;
}

// ===========================================================================
// Fault codes - read / clear for every generation we know how to talk to
// (V11.00.5). One request at a time, run by dtcTask so the web handler never
// blocks. While a request runs (dtcBusy) both live-diag pollers stand down, so
// two testers never talk to the module at once.
//   UDS (ISO-TP)          gens 50 / 51 / 52 (0CQ, 0AY, VAQ) and 46 (Land Rover, 0x795 -> 0x79D) on udsHaldexReqId/RespId:
//                         0x19 02 <mask> read, 0x14 FF FF FF clear
//   KWP2000 over TP2.0    gens 2 / 4 (PQ 7H0 / 1K0, 0BR): 0x18 02 FF 00 read,
//                         0x14 FF 00 clear, VAG 5-digit codes
//   Volvo D2 over CAN     gens 3 / 21 / 43 (P2 DEM, P3 Gen4), module 0x1A:
//                         AE 11 read, AF 11 clear (bench-proven on the P3 unit)
//   Ford KWP2000 on CAN   gen 45 (Kuga), 0x761 -> 0x769 ISO-TP: 0x18 00 FF 00 read (2-byte SAE
//                         codes + status), 0x14 FF 00 clear (bench-proven 2026-11-19, V11.00.9)
// Gen 1 is K-line only. 41 / 42 / 31 / 44 / 53: diagnostic address not known yet.
// ===========================================================================

const char *dtcProtocolForGeneration(uint8_t gen)
{
    switch (dtcProtoFor(gen))
    {
    case DTC_PROTO_UDS:
        return "UDS";
    case DTC_PROTO_KWP:
        return "KWP2000 / TP2.0";
    case DTC_PROTO_VOLVO_D2:
        return "Volvo D2";
    case DTC_PROTO_FORD_KWP:
        return "Ford KWP2000";
    default:
        return nullptr;
    }
}

// ---- UDS over ISO-TP (single-frame request, SF / FF+CF reply) ----
// Returns 0 on a reply (rsp / n filled), -1 timeout, -2 send failed.
static int dtcIsoTp(const uint8_t *req, uint8_t reqLen, uint8_t *rsp, size_t cap, size_t &n, uint32_t timeoutMs)
{
    n = 0;
    xQueueReset(udsWebRxQueue);
    twai_message_t m{};
    m.identifier = udsHaldexReqId;
    m.data_length_code = 8;
    m.data[0] = reqLen;
    memcpy(&m.data[1], req, reqLen);
    for (uint8_t i = reqLen + 1; i < 8; i++) m.data[i] = 0xAA;
    if (!canTransmit(twaiBus1, &m)) return -2;

    uint32_t deadline = millis() + timeoutMs;
    uint16_t total = 0;
    bool multi = false;
    twai_message_t f{};
    while ((int32_t)(deadline - millis()) > 0)
    {
        if (xQueueReceive(udsWebRxQueue, &f, pdMS_TO_TICKS(50)) != pdTRUE) continue;
        const uint8_t pci = f.data[0] & 0xF0;
        if (!multi)
        {
            if (pci == 0x00)
            {
                size_t len = f.data[0] & 0x0F;
                if (len >= 3 && f.data[1] == 0x7F && f.data[3] == 0x78)
                {
                    // responsePending - the answer follows. V11.00.13 (dtc_check_vag_uds): the Gen5 units allow up to
                    // 5000 ms per pending period (P2* @0xC44732), so wait at least 5.5 s after a 0x78.
                    deadline = millis() + (timeoutMs > 5500 ? timeoutMs : 5500);
                    continue;
                }
                if (len > 7) len = 7;
                if (len > cap) len = cap;
                memcpy(rsp, &f.data[1], len);
                n = len;
                return 0;
            }
            if (pci == 0x10)
            {
                total = (uint16_t)(((f.data[0] & 0x0F) << 8) | f.data[1]);
                size_t len = total < 6 ? total : 6;
                if (len > cap) len = cap;
                memcpy(rsp, &f.data[2], len);
                n = len;
                multi = true;
                twai_message_t fc{};
                fc.identifier = udsHaldexReqId;
                fc.data_length_code = 8;
                fc.data[0] = 0x30; // clear to send, no block limit
                fc.data[1] = 0x00; // block size 0 = send everything
                fc.data[2] = 0x02; // V11.00.13: STmin 2 ms - a full Gen5 code list is ~36 frames into an 8-deep queue
                for (uint8_t i = 3; i < 8; i++) fc.data[i] = 0xAA;
                canTransmit(twaiBus1, &fc);
                if (n >= total || n >= cap) return 0;
            }
            continue;
        }
        if (pci != 0x20) continue;
        size_t len = (size_t)total - n;
        if (len > 7) len = 7;
        if (n + len > cap) len = cap - n;
        memcpy(rsp + n, &f.data[1], len);
        n += len;
        if (n >= total || n >= cap) return 0;
    }
    return -1;
}

// ---- Ford KWP2000 on CAN (gen 45, Kuga) - V11.00.9 ----
// Read: 18 00 FF 00 -> 58 <n> then n x {code hi, code lo, status}. Clear: 14 FF 00 -> 54 FF 00.
// Codes are SAE 2-byte: bits 15-14 = P / C / B / U, then four hex digits (C1 21 -> U0121).
static bool dtcRunFordKwp(uint8_t action)
{
    if (!dtcTapBegin(udsHaldexRespId))
    {
        snprintf(dtcMessage, sizeof(dtcMessage), "Diagnostic channel busy - try again");
        return false;
    }
    static uint8_t rsp[128];
    size_t n = 0;
    bool ok = false;
    if (action == 1)
    {
        const uint8_t req[4] = {0x18, 0x00, 0xFF, 0x00};
        const int rc = dtcIsoTp(req, 4, rsp, sizeof(rsp), n, 1500);
        if (rc < 0)
            snprintf(dtcMessage, sizeof(dtcMessage), "No answer from the module (0x%03lX)", (unsigned long)udsHaldexReqId);
        else if (n >= 3 && rsp[0] == 0x7F)
            snprintf(dtcMessage, sizeof(dtcMessage), "Module refused the read (NRC 0x%02X)", rsp[2]);
        else if (n >= 2 && rsp[0] == 0x58)
        {
            const uint8_t count = rsp[1];
            for (uint8_t i = 0; i < count && (size_t)(2 + i * 3 + 2) < n + 1; i++)
            {
                const uint8_t hi = rsp[2 + i * 3];
                const uint8_t lo = rsp[3 + i * 3];
                const uint8_t st = (size_t)(4 + i * 3) < n ? rsp[4 + i * 3] : 0;
                static const char letter[4] = {'P', 'C', 'B', 'U'};
                char code[12];
                snprintf(code, sizeof(code), "%c%X%X%02X", letter[hi >> 6], (hi >> 4) & 0x03, hi & 0x0F, lo);
                char detail[64];
                // V11.00.13 (dtc_check_ford_lr): Kuga status bits 0x20 stored, 0x40 active now, 0x10 not tested yet, 0x80 lamp
                snprintf(detail, sizeof(detail), "status %02X%s%s%s%s", st, (st & 0x40) ? ", active" : "", (st & 0x20) ? ", stored" : "",
                         (st & 0x10) ? ", not tested yet" : "", (st & 0x80) ? ", warning lamp" : "");
                dtcAdd(code, detail);
            }
            ok = true;
        }
        else
            snprintf(dtcMessage, sizeof(dtcMessage), "Unexpected reply from the module");
    }
    else
    {
        const uint8_t req[3] = {0x14, 0xFF, 0x00};
        const int rc = dtcIsoTp(req, 3, rsp, sizeof(rsp), n, 1500);
        if (rc < 0)
            snprintf(dtcMessage, sizeof(dtcMessage), "No answer from the module (0x%03lX)", (unsigned long)udsHaldexReqId);
        else if (n >= 1 && rsp[0] == 0x54)
            ok = true;
        else if (n >= 3 && rsp[0] == 0x7F)
            snprintf(dtcMessage, sizeof(dtcMessage), "Module refused the clear (NRC 0x%02X)", rsp[2]);
        else
            snprintf(dtcMessage, sizeof(dtcMessage), "Unexpected reply from the module");
    }
    dtcTapEnd();
    return ok;
}

static bool dtcRunUds(uint8_t action)
{
    if (!dtcTapBegin(udsHaldexRespId))
    {
        snprintf(dtcMessage, sizeof(dtcMessage), "Diagnostic channel busy - try again");
        return false;
    }
    static uint8_t rsp[256];
    size_t n = 0;
    bool ok = false;
    bool extendedOpened = false;

    if (action == 1)
    {
        // Implementations differ in which status masks they accept
        static const uint8_t masks[] = {0xFF, 0xAE, 0x09, 0x08};
        for (uint8_t mi = 0; mi < sizeof(masks); mi++)
        {
            const uint8_t req[3] = {0x19, 0x02, masks[mi]};
            const int rc = dtcIsoTp(req, 3, rsp, sizeof(rsp), n, 1500);
            if (rc < 0)
            {
                snprintf(dtcMessage, sizeof(dtcMessage), "No answer from the module (UDS 0x%03lX)", (unsigned long)udsHaldexReqId);
                break;
            }
            if (n >= 3 && rsp[0] == 0x7F)
            {
                if ((rsp[2] == 0x31 || rsp[2] == 0x12) && mi + 1 < sizeof(masks)) continue; // try the next mask
                snprintf(dtcMessage, sizeof(dtcMessage), "Module refused the read (NRC 0x%02X)", rsp[2]);
                break;
            }
            if (n < 3 || rsp[0] != 0x59 || rsp[1] != 0x02)
            {
                snprintf(dtcMessage, sizeof(dtcMessage), "Unexpected reply 0x%02X", n ? rsp[0] : 0);
                break;
            }
            static const char letters[4] = {'P', 'C', 'B', 'U'};
            for (size_t i = 3; i + 4 <= n; i += 4)
            {
                char code[14];
                char detail[64];
                const uint8_t b0 = rsp[i], b1 = rsp[i + 1], b2 = rsp[i + 2];
                // V11.00.13 (dtc_check_vag_uds): after a clear every code in the unit's table reads status 0x10
                // ("not tested since clear") and matches mask 0xFF - 60-65 non-faults that pushed real codes out
                // of our list. Only codes that are active (bit 0) or stored (bit 3) are faults.
                if ((rsp[i + 3] & 0x09) == 0)
                {
                    continue;
                }
                snprintf(code, sizeof(code), "%c%u%X%02X-%02X", letters[(b0 >> 6) & 0x03],
                         (unsigned)((b0 >> 4) & 0x03), (unsigned)(b0 & 0x0F), b1, b2);
                dtcUdsStatusText(rsp[i + 3], detail, sizeof(detail));
                dtcAdd(code, detail);
            }
            ok = true;
            break;
        }
    }
    else
    {
        const uint8_t req[4] = {0x14, 0xFF, 0xFF, 0xFF};
        int rc = dtcIsoTp(req, 4, rsp, sizeof(rsp), n, 2000);
        if (rc == 0 && n >= 3 && rsp[0] == 0x7F && (rsp[2] == 0x7F || rsp[2] == 0x22 || rsp[2] == 0x33))
        {
            // Not allowed in the default session: open the extended session and try once more
            const uint8_t sess[2] = {0x10, 0x03};
            if (dtcIsoTp(sess, 2, rsp, sizeof(rsp), n, 1500) == 0 && n >= 2 && rsp[0] == 0x50)
            {
                extendedOpened = true;
                rc = dtcIsoTp(req, 4, rsp, sizeof(rsp), n, 2000);
            }
        }
        if (rc < 0)
            snprintf(dtcMessage, sizeof(dtcMessage), "No answer from the module (UDS 0x%03lX)", (unsigned long)udsHaldexReqId);
        else if (n >= 1 && rsp[0] == 0x54)
            ok = true;
        else if (n >= 3 && rsp[0] == 0x7F)
            snprintf(dtcMessage, sizeof(dtcMessage), "Module refused the clear (NRC 0x%02X)", rsp[2]);
        else
            snprintf(dtcMessage, sizeof(dtcMessage), "Unexpected reply 0x%02X", n ? rsp[0] : 0);
        if (extendedOpened)
        {
            const uint8_t back[2] = {0x10, 0x01};
            dtcIsoTp(back, 2, rsp, sizeof(rsp), n, 500); // hand the module back to its default session
        }
    }
    dtcTapEnd();
    return ok;
}

// ---- KWP2000 over VW TP2.0 (PQ Gen2 / Gen4) ----
static bool dtcRunKwp(uint8_t action)
{
    if (tp20RxQueue == nullptr)
    {
        snprintf(dtcMessage, sizeof(dtcMessage), "TP2.0 not ready");
        return false;
    }
    uint32_t testerTxId = 0, ecuTxId = 0;
    bool opened = tp20OpenChannel(testerTxId, ecuTxId, 1000);
    if (!opened)
    {
        // V11.00.13 (dtc_check_vag_kwp): a channel left open by an earlier tester makes the module ignore the
        // setup silently until it times out - wait ~1 s and try once more before reporting "no answer".
        vTaskDelay(pdMS_TO_TICKS(1000));
        opened = tp20OpenChannel(testerTxId, ecuTxId, 1000);
    }
    if (!opened)
    {
        snprintf(dtcMessage, sizeof(dtcMessage), "No answer from the module (TP2.0 address 0x%02X)", KWP_TP20_HALDEX_ADDR);
        kwpTp20EcuTxId = 0;
        return false;
    }
    uint8_t seq = 0;
    static uint8_t buf[128];
    bool ok = false;
    const uint8_t sess[] = {0x10, 0x89};
    uint8_t n = kwpTp20Transact(testerTxId, ecuTxId, sess, sizeof(sess), seq, buf, sizeof(buf), 1000);
    if (n < 1 || buf[0] != 0x50)
    {
        snprintf(dtcMessage, sizeof(dtcMessage), "Module refused the diagnostic session");
    }
    else if (action == 1)
    {
        const uint8_t req[] = {0x18, 0x02, 0xFF, 0x00}; // ReadDiagnosticTroubleCodesByStatus, all groups
        n = kwpTp20Transact(testerTxId, ecuTxId, req, sizeof(req), seq, buf, sizeof(buf), 2000);
        if (n >= 2 && buf[0] == 0x58)
        {
            // 0x58 <count> then 3 bytes per code: code high, code low, status
            for (uint8_t i = 2; i + 3 <= n; i += 3)
            {
                const uint16_t vag = (uint16_t)((buf[i] << 8) | buf[i + 1]);
                if (vag == 0xFFFF) continue; // "no fault" filler some units send
                char code[14];
                char detail[64];
                snprintf(code, sizeof(code), "%05u", (unsigned)vag);
                // V11.00.13 (dtc_check_vag_kwp): status 0x6n = active now, 0x2n = stored, not active (bit 0x20 is
                // always set) - so "intermittent" only when bit 6 is clear. Low nibble = fault type.
                snprintf(detail, sizeof(detail), "status %02X (type %02u)%s", buf[i + 2], (unsigned)(buf[i + 2] & 0x0F),
                         ((buf[i + 2] & 0x60) == 0x20) ? ", intermittent" : ((buf[i + 2] & 0x40) ? ", active" : ""));
                dtcAdd(code, detail);
            }
            ok = true;
        }
        else if (n >= 3 && buf[0] == 0x7F)
            snprintf(dtcMessage, sizeof(dtcMessage), "Module refused the read (NRC 0x%02X)", buf[2]);
        else
            snprintf(dtcMessage, sizeof(dtcMessage), "No reply to the read");
    }
    else
    {
        const uint8_t req[] = {0x14, 0xFF, 0x00}; // ClearDiagnosticInformation, all groups
        n = kwpTp20Transact(testerTxId, ecuTxId, req, sizeof(req), seq, buf, sizeof(buf), 3000);
        if (n >= 1 && buf[0] == 0x54)
            ok = true;
        else if (n >= 3 && buf[0] == 0x7F)
            snprintf(dtcMessage, sizeof(dtcMessage), "Module refused the clear (NRC 0x%02X)", buf[2]);
        else
            snprintf(dtcMessage, sizeof(dtcMessage), "No reply to the clear");
    }
    tp20Disconnect(testerTxId);
    return ok;
}

// ---- Volvo D2 over CAN (29-bit) ----
// Request 0x000FFFFE [0xC8 + n, module, service, params]; reply on 0x01204001.
// Reply header byte: bit 7 = first frame, bit 6 = last frame, bits 2-0 = data
// bytes in this frame (first / last / single); middle frames carry 7 bytes.
// Frames come ~20 ms apart. Bench-proven on the P3 unit, 2026-11-07.
// [KM -][REA audit_volvo_c166 #15/#45/#71: S60R MO03 = exact-match RX of 0x000FFFFE, module byte 0x1A,
// reply MO01 0x01204001. Headers F7 / EF on the same ID are a TEST-COMMAND channel that can force the clutch
// setpoint - never send them; our 0xC8+n headers never match. Gen 21: the reply ID is NOT known (facelift
// value assumed); gen 43: no P3 dump, IDs [bench]]
static int dtcVolvoD2(uint8_t service, uint8_t param, uint8_t *out, size_t cap, size_t &n)
{
    n = 0;
    xQueueReset(udsWebRxQueue);
    twai_message_t m{};
    m.identifier = VOLVO_G4_DIAG_REQUEST_ID;
    m.extd = 1;
    m.data_length_code = 8;
    m.data[0] = 0xCB; // single frame, 3 bytes: module, service, param [bench][REA: not the F7/EF test header]
    m.data[1] = VOLVO_G4_DIAG_MODULE; // [REA S60R #45/#71: byte 1 == 0x1A checked]
    m.data[2] = service; // AE / AF [bench][no OEM code for the service table]
    m.data[3] = param;   // 0x11 [bench]
    if (!canTransmit(twaiBus1, &m)) return -2;

    uint32_t deadline = millis() + 1500; // V11.00.13: not const - a "still working" reply pushes it out
    bool started = false;
    twai_message_t f{};
    while ((int32_t)(deadline - millis()) > 0)
    {
        if (xQueueReceive(udsWebRxQueue, &f, pdMS_TO_TICKS(50)) != pdTRUE) continue;
        if (!f.extd || f.data_length_code < 2) continue;
        const uint8_t h = f.data[0];          // reply byte 0 = header [bench][no OEM code for the framing]
        const bool first = (h & 0x80) != 0;   // bit 7 first frame [bench]
        const bool last = (h & 0x40) != 0;    // bit 6 last frame [bench]
        if (first)
        {
            n = 0;
            started = true;
        }
        if (!started) continue;
        uint8_t len = 7;
        if (first || last) len = (uint8_t)(h & 0x07);
        for (uint8_t i = 0; i < len && n < cap; i++) out[n++] = f.data[1 + i];
        if (last)
        {
            // V11.00.13 (dtc_check_volvo): "1A 7E <svc> 23" = the DEM is still working on the answer (sent every
            // COM tick, FUN_01f242) - throw it away and keep waiting for the real reply
            if (n >= 4 && out[1] == 0x7E && out[3] == 0x23)
            {
                n = 0;
                started = false;
                deadline = millis() + 3000;
                continue;
            }
            return 0;
        }
    }
    return -1;
}

static bool dtcRunVolvoD2(uint8_t action)
{
    if (!dtcTapBegin(VOLVO_G4_DIAG_RESPONSE_ID))
    {
        snprintf(dtcMessage, sizeof(dtcMessage), "Diagnostic channel busy - try again");
        return false;
    }
    static uint8_t buf[128];
    size_t n = 0;
    bool ok = false;
    const uint8_t service = (action == 1) ? 0xAE : 0xAF; // read / clear stored faults
    if (action != 1)
    {
        // V11.00.13 (dtc_check_volvo): the DEM only accepts AF 11 after an AE 11 read since power-up
        // (flag 0xCF18, S60R FUN_021b46) - otherwise 7F AF 12 (DEM.bin: 7F AF 83). Read first, ignore the result.
        dtcVolvoD2(0xAE, 0x11, buf, sizeof(buf), n);
        n = 0;
    }
    const int rc = dtcVolvoD2(service, 0x11, buf, sizeof(buf), n);
    if (rc < 0)
        snprintf(dtcMessage, sizeof(dtcMessage), "No answer from the AWD module (Volvo D2, module 0x%02X)", VOLVO_G4_DIAG_MODULE);
    else if (n >= 4 && buf[0] == VOLVO_G4_DIAG_MODULE && buf[1] == 0x7F)
        snprintf(dtcMessage, sizeof(dtcMessage), "Module refused (code 0x%02X)", buf[3]);
    else if (n >= 3 && buf[0] == VOLVO_G4_DIAG_MODULE && buf[1] == (uint8_t)(service + 0x40) && buf[2] == 0x11)
    {
        ok = true;
        if (action == 1)
        {
            for (size_t i = 3; i + 2 <= n; i += 2)
            {
                const uint16_t code = (uint16_t)((buf[i] << 8) | buf[i + 1]);
                if (code == 0x0000) continue; // (the DEM never pads - dtc_check_volvo #3 - kept as a harmless guard)
                char text[14];
                snprintf(text, sizeof(text), "AWD-%04X", code);
                dtcAdd(text, volvoG4DtcNote(code));
            }
        }
    }
    else
        snprintf(dtcMessage, sizeof(dtcMessage), "Unexpected reply");
    dtcTapEnd();
    return ok;
}

static void volvoDiagPoll()
{
    const bool allowed = liveDiagEnabled && hasCANHaldex && dtcProtoFor(haldexGeneration) == DTC_PROTO_VOLVO_D2 &&
                         !externalDiagActive() && !analyzerMode && !analyzerSerial;
    if (!allowed)
    {
        if (volvoDiagLastGoodMs != 0 && dtcProtoFor(haldexGeneration) == DTC_PROTO_VOLVO_D2)
        {
            receivedHaldexTempC = -128; // stopped reading: show "--", not an old value
        }
        volvoDiagLastGoodMs = 0;
        return;
    }
    if (millis() - volvoDiagLastPollMs < 2000) return;
    volvoDiagLastPollMs = millis();
    if (!dtcTapBegin(VOLVO_G4_DIAG_RESPONSE_ID)) return; // channel busy (web / serial-lab read) - next time
    // read data by local identifier 0x0002: CD 1A A6 00 02 01 -> 1A E6 00 02 <temp> ...
    xQueueReset(udsWebRxQueue);
    twai_message_t m{};
    m.identifier = VOLVO_G4_DIAG_REQUEST_ID;
    m.extd = 1;
    m.data_length_code = 8;
    m.data[0] = 0xCD; // single frame, 5 bytes [bench]
    m.data[1] = VOLVO_G4_DIAG_MODULE; // 0x1A [REA S60R #71]
    m.data[2] = 0xA6; // read data by local ID [bench][WEB]
    m.data[3] = 0x00; // local ID high [bench]
    m.data[4] = 0x02; // local ID 0x0002 = DEM oil temperature [WEB vtl/volvo-ddd]
    m.data[5] = 0x01; // [bench]
    if (canTransmit(twaiBus1, &m))
    {
        const uint32_t deadline = millis() + 300;
        twai_message_t f{};
        while ((int32_t)(deadline - millis()) > 0)
        {
            if (xQueueReceive(udsWebRxQueue, &f, pdMS_TO_TICKS(30)) != pdTRUE) continue;
            // first (or single) frame: [hdr][1A][E6][00][02][temp] ...
            if (f.extd && (f.data[0] & 0x80) && f.data[1] == VOLVO_G4_DIAG_MODULE && f.data[2] == 0xE6 &&
                f.data[3] == 0x00 && f.data[4] == 0x02)
            {
                receivedHaldexTempC = (int16_t)(int8_t)f.data[5]; // reply b5 = oil temp, s8 degC [KM -][WEB][bench][no OEM code]
                volvoDiagLastGoodMs = millis();
                break;
            }
        }
    }
    dtcTapEnd();
    if (volvoDiagLastGoodMs != 0 && millis() - volvoDiagLastGoodMs > 6000)
    {
        receivedHaldexTempC = -128; // no answer for a while
    }
}

// One 22 HH LL read. 1 = value in out, 0 = negative response (DID not in this firmware),
// -1 = no answer, -2 = channel busy (another request in flight - not the module's fault).
static int fordReadDid(uint16_t did, bool oneByte, int32_t &out)
{
    if (!dtcTapBegin(udsHaldexRespId)) return -2;
    static uint8_t rsp[16];
    size_t n = 0;
    const uint8_t req[3] = {0x22, (uint8_t)(did >> 8), (uint8_t)(did & 0xFF)};
    const int rc = dtcIsoTp(req, sizeof(req), rsp, sizeof(rsp), n, 250);
    dtcTapEnd();
    if (rc < 0) return -1;
    if (n >= 2 && rsp[0] == 0x7F && rsp[1] == 0x22) return 0;
    if (n >= 4 && rsp[0] == 0x62 && rsp[1] == (uint8_t)(did >> 8) && rsp[2] == (uint8_t)(did & 0xFF))
    {
        if (oneByte)
        {
            out = (int8_t)rsp[3];
            return 1;
        }
        if (n >= 5)
        {
            out = (int32_t)(((uint16_t)rsp[3] << 8) | rsp[4]);
            return 1;
        }
    }
    return -1;
}

static void fordDiagStore(uint8_t field, int32_t raw)
{
    const bool v8V41 = (fordDiagVariant == 1);
    switch (field)
    {
    case FORD_DIAG_SUPPLY:
        fordSupplyV = (float)raw / 1000.0f; // 0x0042 mV [REA #72: E430 x 24]
        break;
    case FORD_DIAG_TEMP:
        fordOilTemp = v8V41 ? (float)(int16_t)(uint16_t)raw / 10.0f : (float)raw; // 8V41 0x1943 0.1 C [REA #73]; 9V4N 0x19BA s8 C [REA #62, INF]
        // the dashboard's Haldex temperature tile too; below -50 C = open sensor
        receivedHaldexTempC = fordOilTemp < -50.0f ? -128 : (int16_t)lroundf(fordOilTemp);
        break;
    case FORD_DIAG_PMEAS:
        fordPressureRaw = (uint16_t)raw;
        fordPressureBar = (float)raw * 0.01953f; // 8V41 only - provisional (see above) [REA #6/#45: no unit in the code - assumption; 9V4N raw = valve current #63]
        break;
    case FORD_DIAG_PCMD:
        fordSetpointRaw = (uint16_t)raw;
        fordSetpointBar = (float)raw / 1000.0f; // 8V41 only - provisional [REA #43/FI-8: 0x1947 = valve command in F1058 units (0x1944 x 19.53), not 0.1 kPa; /1000 = same assumed scale as 0x1944 x 0.01953]
        break;
    case FORD_DIAG_VALVE:
        fordValveCurrentA = (float)raw / 1000.0f; // 0x1945 on 8V41 and 9V4N (same ADC ch 11) [REA #46/#63: measured coil current; mA = INF]
        break;
    case FORD_DIAG_PUMP:
        fordPumpCurrentA = (float)raw / 1000.0f; // 0x1946 [REA #85: G(duty) x ADC >> 5; mA = INF]
        break;
    default:
        return;
    }
    fordDiagValid |= field;
}

static void fordDiagPoll()
{
    const bool allowed = liveDiagEnabled && hasCANHaldex && haldexGeneration == GEN_FORD_KUGA &&
                         !externalDiagActive() && !analyzerMode && !analyzerSerial &&
                         (udsManualLastMs == 0 || millis() - udsManualLastMs > 15000);
    if (!allowed)
    {
        if (fordDiagLastGoodMs != 0 && haldexGeneration == GEN_FORD_KUGA)
        {
            receivedHaldexTempC = -128; // stopped reading: show "--", not an old value
        }
        if (fordDiagConnected || fordDiagValid) fordDiagClear();
        return;
    }
    const uint32_t gap = fordDiagConnected ? 250 : 2000;
    if (millis() - fordDiagLastPollMs < gap) return;
    fordDiagLastPollMs = millis();
    udsApplyDefaultIds(); // 0x761 / 0x769 unless pinned in the serial lab

    int32_t raw = 0;
    if (fordDiagVariant == 0)
    {
        // Identify the firmware by its DID set: 0x1943 exists only in 8V41, 0x19BA only in 9V4N.
        int r = fordReadDid(0x1943, false, raw);
        if (r == 1)
        {
            fordDiagVariant = 1;
            fordDiagStore(FORD_DIAG_TEMP, raw);
        }
        else if (r == 0)
        {
            r = fordReadDid(0x19BA, true, raw);
            if (r == 1)
            {
                fordDiagVariant = 2;
                fordDiagStore(FORD_DIAG_TEMP, raw);
            }
            else if (r == 0)
            {
                fordDiagVariant = 2; // neither: treat as the newer set, its DIDs answer or NRC on their own
            }
        }
        if (r >= 0)
        {
            fordDiagConnected = true;
            fordDiagLastGoodMs = millis();
            fordDiagMisses = 0;
            fordDiagIdx = 0;
        }
        return;
    }

    const FordDid *set = (fordDiagVariant == 1) ? kFord8V41 : kFord9V4N;
    const uint8_t count = (fordDiagVariant == 1) ? (uint8_t)(sizeof(kFord8V41) / sizeof(kFord8V41[0]))
                                                 : (uint8_t)(sizeof(kFord9V4N) / sizeof(kFord9V4N[0]));
    if (fordDiagIdx >= count) fordDiagIdx = 0;
    const FordDid &d = set[fordDiagIdx];
    const int r = fordReadDid(d.did, d.oneByte, raw);
    if (r == -2) return; // busy - same DID next time
    fordDiagIdx = (uint8_t)((fordDiagIdx + 1) % count);
    if (r == 1)
    {
        fordDiagStore(d.field, raw);
    }
    else if (r == 0)
    {
        fordDiagValid &= (uint8_t)~d.field; // this firmware hasn't got it
    }
    if (r >= 0)
    {
        fordDiagLastGoodMs = millis();
        fordDiagMisses = 0;
    }
    else if (++fordDiagMisses >= 6)
    {
        // a whole cycle without an answer: module off / unplugged - show "--" and re-identify later
        receivedHaldexTempC = -128;
        fordDiagClear();
    }
}

// ---- Main code -----------------------------------------------------------------

void udsMQBTask(void *arg)
{
    // Request/response pair comes from udsHaldexReqId / udsHaldexRespId (defs.h):
    // Haldex 0x70F -> 0x779 (VCDS uses 0x70F on Bus 0, bridged to Bus 1; response
    // confirmed from a SavvyCAN capture), VAQ 0x71E -> 0x788 per the K-matrix.
    // parseCANHdx routes udsHaldexRespId frames into udsRxQueue.

    static constexpr uint16_t kDIDs[] = {
        0x0286, // Terminal Voltage
        0x028D, // Control Module Temperature
        0x2BE6, // Haldex Clutch Current
        0x2BE7, // Haldex Clutch PWM
        0x2BF1, // Clutch Temperature
        0x2BE4, // Cooling Fin Temperature
        0x2BE9, // Pump motor voltage (calculated)
        0x2BF3, // Clutch torque applied (V11.00.10)
        0x2BF0, // Axle speed difference (V11.00.10; not on the VAQ - skipped there)
    };
    static constexpr uint8_t kDIDCount = sizeof(kDIDs) / sizeof(kDIDs[0]);

    // Timing. Temperatures / voltages move slowly, so one DID every 500 ms
    // (full cycle ~3.5 s) is plenty and cuts diagnostic traffic ~3x versus the
    // old 150 ms cadence. TesterPresent is only needed in extended session and
    // just has to beat the 5 s S3 timer.
    static constexpr uint32_t kPollGapMs   = 500;  // idle gap between DID requests
    static constexpr uint32_t kRespWindow  = 500;  // max wait for one response
    static constexpr uint32_t kTesterMs    = 2000; // TesterPresent period (extended session only)

    // Session strategy. Try plain RDBI in the default session first - no
    // session change, no TesterPresent, nothing for the module to time out of.
    // Only if the module answers with a session-related NRC do we open the
    // extended session (0x03), and then we hand it back (0x01) on exit instead
    // of leaving the module to drop it on S3 timeout. Once extended is known to
    // be required it is used directly on later reconnects.
    static bool preferExtended = false;

    udsRxQueue = xQueueCreate(8, sizeof(twai_message_t));

    auto pollAllowed = []() -> bool {
        // Gen5 family: MQB 0CQ (50), PQ-derived 0AY (51), VAQ (52) - same UDS stack.
        return liveDiagEnabled && !externalDiagActive() && hasCANHaldex &&
               isGen5Family() && !analyzerMode && !analyzerSerial &&
               !dtcBusy; // V11.00.5: stand down while a fault-code read / clear runs
    };

    while (1)
    {
        if (!pollAllowed())
        {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        xQueueReset(udsRxQueue); // flush any stale frames before opening the channel
        udsApplyDefaultIds();    // generation may have changed since the last connect
        const uint32_t kReqId = udsHaldexReqId;
        udsPollActive = true;    // from here on 0x779 replies are ours - keep them off Bus 0

        uint8_t session = preferExtended ? 0x03 : 0x01;
        bool sessionOk = false;

        if (session == 0x01)
        {
            // Probe: one RDBI in the default session.
            const UdsPollResult r = udsPollDid(kReqId, kDIDs[0], kRespWindow);
            if (r == UDS_POLL_OK || r == UDS_POLL_NRC_OTHER)
            {
                sessionOk = true; // module talks to us without a session change
            }
            else if (r == UDS_POLL_NRC_SESSION)
            {
                DEBUG("UDS - RDBI refused in default session (NRC), falling back to extended");
                preferExtended = true;
                session = 0x03;
            }
            // TIMEOUT: module silent - retry below
        }

        if (session == 0x03)
        {
            sessionOk = udsSessionControl(kReqId, 0x03, 2000);
        }

        if (!sessionOk)
        {
            udsPollActive = false;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue; // no usable channel yet - retry
        }

        udsSessionMode = session;
        DEBUG("UDS - polling Haldex in %s session", session == 0x03 ? "extended" : "default");

        uint32_t lastTP = millis();
        uint8_t  didIdx = (session == 0x01) ? 1 : 0; // default-session probe already read kDIDs[0]
        bool     needExtended = false;

        while (pollAllowed())
        {
            if (udsHaldexReqId != kReqId)
            {
                break; // generation changed (e.g. 0CQ -> VAQ): reconnect on the new ID pair (V11.00.8)
            }
            if (session == 0x03 && (millis() - lastTP) >= kTesterMs)
            {
                // TesterPresent with suppress-positive-response bit set (0x80) - no reply expected
                const uint8_t tpReq[] = {0x3E, 0x80};
                udsSendFrame(kReqId, tpReq, sizeof(tpReq));
                lastTP = millis();
            }

            const UdsPollResult r = udsPollDid(kReqId, kDIDs[didIdx], kRespWindow);
            if (r == UDS_POLL_NRC_SESSION)
            {
                // Default session: a DID the probe did not cover needs the extended
                // session after all. Extended session: the module has dropped back to
                // default (e.g. reset while cranking) - V11.00.8: reconnect instead of
                // polling on with frozen values.
                needExtended = true;
                break;
            }

            didIdx = (didIdx + 1) % kDIDCount;
            if (kDIDs[didIdx] == 0x2BF0 && haldexGeneration == 52)
                didIdx = (didIdx + 1) % kDIDCount; // the VAQ has no axle-difference DID
            vTaskDelay(pdMS_TO_TICKS(kPollGapMs));
        }

        // Leaving the channel. Return the module to its default session
        // ourselves rather than letting S3 expire - but not if a real tool has
        // just appeared on Bus 0: it may already own a session of its own.
        if (session == 0x03 && !externalDiagActive())
        {
            udsSessionControl(kReqId, 0x01, 300);
        }
        udsSessionMode = 0;
        udsPollActive = false;

        if (needExtended)
        {
            DEBUG("UDS - DID refused in default session, switching to extended");
            preferExtended = true;
            continue; // reconnect straight away in the extended session
        }

        // Session ended or conditions changed - clear all UDS data
        udsClearValues();
    }
}

void kwpTp20Task(void *arg)
{
    (void)arg;
    // Measuring-block groups differ by generation (confirmed from VCDS captures):
    //   Gen2 (7H0 / 1K0): groups 0x01 (brake switch, supply), 0x02 (oil temp, measured oil
    //               pressure), 0x7D (CAN status); after the level-3 login also 0x32 (pump current)
    //               and 0x33 (valve duty, valve current) - V11.00.10
    //   Gen4 (0BR/0BS/2K0, also 0AY907554C): groups 0x01 (oil temp, plate temp model, voltage),
    //               0x03 (pressure target, torque command, valve duty, valve current),
    //               0x04 (pump state, pump duty, pump current) - V11.00.10
    // Response bytes are dumped raw into kwpTp20RawDump; known Gen4 fields are
    // additionally scaled into the kwp* globals via vagScale().
    static const uint8_t kGroupsGen2[] = {0x01, 0x02, 0x7D};
    static const uint8_t kGroupsGen2Login[] = {0x01, 0x02, 0x7D, 0x32, 0x33}; // after SecurityAccess level 3
    static const uint8_t kGroupsGen4[] = {0x01, 0x03, 0x04};
    static char lineBuf[8][56];

    tp20RxQueue = xQueueCreate(16, sizeof(twai_message_t));

    while (1)
    {
        if (!liveDiagEnabled || externalDiagActive() || !hasCANHaldex || (haldexGeneration != 2 && haldexGeneration != 4) ||
            analyzerMode || analyzerSerial || dtcBusy) // V11.00.5: dtcBusy = a fault-code read / clear owns the channel
        {
            kwpTp20Connected = false;
            kwpTp20EcuTxId = 0;
            tp20ClearValues();
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        uint32_t testerTxId = 0, ecuTxId = 0;
        if (!tp20OpenChannel(testerTxId, ecuTxId, 1000))
        {
            kwpTp20Connected = false;
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        // Open the KWP session (StartDiagnosticSession, VAG development mode 0x89).
        uint8_t seq = 0;
        uint8_t buf[64];
        const uint8_t sess[] = {0x10, 0x89};
        uint8_t n = kwpTp20Transact(testerTxId, ecuTxId, sess, sizeof(sess), seq, buf, sizeof(buf), 1000);
        if (n < 2 || buf[0] != 0x50)
        {
            tp20Disconnect(testerTxId);
            kwpTp20Connected = false;
            vTaskDelay(pdMS_TO_TICKS(1500));
            continue;
        }

        kwpTp20Connected = true;

        // Select the measuring groups for this generation and seed the raw dump.
        const bool isGen4 = (haldexGeneration == 4);
        // Gen2: log in (level 3) for the pump / valve groups 0x32 / 0x33 - V11.00.10.
        kwpGen2LoggedIn = !isGen4 && gen2SecurityLogin(testerTxId, ecuTxId, seq);
        const uint8_t *kGroups = isGen4 ? kGroupsGen4 : (kwpGen2LoggedIn ? kGroupsGen2Login : kGroupsGen2);
        const uint8_t kGroupCount = isGen4
            ? (uint8_t)(sizeof(kGroupsGen4) / sizeof(kGroupsGen4[0]))
            : (kwpGen2LoggedIn ? (uint8_t)(sizeof(kGroupsGen2Login) / sizeof(kGroupsGen2Login[0]))
                               : (uint8_t)(sizeof(kGroupsGen2) / sizeof(kGroupsGen2[0])));
        for (uint8_t i = 0; i < kGroupCount; i++)
            snprintf(lineBuf[i], sizeof(lineBuf[i]), "G%02X: --", kGroups[i]);

        uint8_t gi = 0;
        uint8_t consecutiveFails = 0;

        while (liveDiagEnabled && !externalDiagActive() && hasCANHaldex && (haldexGeneration == 2 || haldexGeneration == 4) &&
               !analyzerMode && !analyzerSerial && !dtcBusy)
        {
            const uint8_t group = kGroups[gi];
            const uint8_t req[] = {0x21, group}; // ReadDataByLocalIdentifier
            n = kwpTp20Transact(testerTxId, ecuTxId, req, sizeof(req), seq, buf, sizeof(buf), 600);

            char *ln = lineBuf[gi];
            if (n >= 2 && buf[0] == 0x61 && buf[1] == group)
            {
                consecutiveFails = 0;
                int p = snprintf(ln, 56, "G%02X:", group);
                for (uint8_t i = 2; i < n && p > 0 && p < 54; i++)
                    p += snprintf(ln + p, 56 - p, " %02X", buf[i]);

                // Scale the confirmed Gen4 fields into the named globals.
                if (isGen4)
                    tp20DecodeGen4(group, buf, n);
                else
                    tp20DecodeGen2(group, buf, n);
            }
            else if (n >= 3 && buf[0] == 0x7F)
            {
                consecutiveFails = 0;
                snprintf(ln, 56, "G%02X: NRC %02X", group, buf[2]);
            }
            else
            {
                snprintf(ln, 56, "G%02X: no data", group);
                consecutiveFails++;
            }

            // Rebuild the shared raw-dump buffer (bounded).
            size_t pos = 0;
            for (uint8_t i = 0; i < kGroupCount && pos < sizeof(kwpTp20RawDump) - 1; i++)
            {
                int w = snprintf(kwpTp20RawDump + pos, sizeof(kwpTp20RawDump) - pos, "%s%s", i ? "\n" : "", lineBuf[i]);
                if (w < 0) break;
                pos += (size_t)w;
            }

            gi = (uint8_t)((gi + 1) % kGroupCount);
            if (gi == 0) DEBUG("KWP TP2.0 blocks:\n%s", kwpTp20RawDump);

            // The whole channel is dead (e.g. ECU closed it) — bail out and reopen.
            if (consecutiveFails >= (uint8_t)(kGroupCount * 2)) break;

            vTaskDelay(pdMS_TO_TICKS(100)); // keeps the TP2.0 channel alive between polls
        }

        tp20Disconnect(testerTxId);
        kwpTp20Connected = false;
        kwpGen2LoggedIn = false; // the unlock ends with the channel
        tp20ClearValues();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void dtcTask(void *arg)
{
    (void)arg;
    while (1)
    {
        if (dtcRequest == 0)
        {
            volvoDiagPoll();
            fordDiagPoll();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        const uint8_t action = dtcRequest; // 1 = read, 2 = clear
        dtcRequest = 0;
        dtcLastAction = action;
        dtcCount = 0;
        dtcMessage[0] = '\0';
        const uint8_t gen = haldexGeneration;
        const uint8_t proto = dtcProtoFor(gen);
        const char *name = dtcProtocolForGeneration(gen);
        snprintf(dtcProtocol, sizeof(dtcProtocol), "%s", name ? name : "");

        bool ok = false;
        if (proto == DTC_PROTO_NONE)
            snprintf(dtcMessage, sizeof(dtcMessage), "%s", gen == 1 ? "Gen1 is diagnosed over K-line - use VCDS" : "Not available for this generation yet");
        else if (!hasCANHaldex)
            snprintf(dtcMessage, sizeof(dtcMessage), "The Haldex is not on the bus");
        else if (analyzerMode || analyzerSerial)
            snprintf(dtcMessage, sizeof(dtcMessage), "Turn the CAN analyser off first");
        else if (externalDiagActive())
            snprintf(dtcMessage, sizeof(dtcMessage), "A scan tool is connected - use that");
        else
        {
            dtcBusy = true;
            // let the live-diag pollers finish their current request and let go
            const uint32_t waitUntil = millis() + 3000;
            while ((udsPollActive || kwpTp20Connected) && (int32_t)(waitUntil - millis()) > 0)
                vTaskDelay(pdMS_TO_TICKS(50));
            vTaskDelay(pdMS_TO_TICKS(100));
            if (proto == DTC_PROTO_UDS)
            {
                udsApplyDefaultIds(); // V11.00.13: this generation's pair (not whatever the live poller last used)
                ok = dtcRunUds(action);
            }
            else if (proto == DTC_PROTO_KWP)
                ok = dtcRunKwp(action);
            else if (proto == DTC_PROTO_FORD_KWP)
            {
                udsApplyDefaultIds(); // 0x761 / 0x769 unless pinned in the serial lab
                ok = dtcRunFordKwp(action);
            }
            else
                ok = dtcRunVolvoD2(action);
            dtcBusy = false;
        }
        if (ok && action == 2) snprintf(dtcMessage, sizeof(dtcMessage), "Fault codes cleared");
        if (ok && action == 1 && dtcCount == 0) snprintf(dtcMessage, sizeof(dtcMessage), "No fault codes stored");
        dtcDoneMs = millis();
        dtcState = ok ? DTC_DONE : DTC_ERROR;
    }
}
