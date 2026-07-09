#include <OpenHaldexC6_UDS.h>

using namespace OpenHaldexC6;

// ---------------------------------------------------------------------------
// UDS MQB polling task (Gen 5 only)
// Requests are sent on Bus 1 to CAN ID 0x771 (Haldex physical address).
// Responses come from 0x779 on Bus 1, routed here via udsRxQueue by
// parseCAN_hdx so there is no TWAI RX queue contention.
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
    return (twai_transmit_v2(twai_bus_1, &msg, pdMS_TO_TICKS(10)) == ESP_OK);
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

    // Actual measurement data starts at frame.data[4]
    switch (did)
    {
    case 0x0286: // Terminal Voltage: 1 byte, × 0.1 V  (0x8F=143 → 14.3 V confirmed)
        if (payloadLen >= 4)
            udsTerminalVoltage = frame.data[4] * 0.1f;
        break;

    case 0x028D: // Control Module Temperature: 1 byte, temp = raw − 55 °C
                 // Heat-sweep confirmed (VCDS, both modules):
                 //   0CQ 554C: 0x4F=79 → 24 °C … 0x76=118 → 63 °C (probe ~25→68)
                 //   0CQ 554D: 0x52=82 → 27 °C … 0x7F=127 → 72 °C (probe ~27→73)
                 // Offset 55 cross-checked against clutch temp at all 4 anchors → mean 55.0.
        if (payloadLen >= 4)
            udsModuleTemp = (float)frame.data[4] - 55.0f;
        break;

    case 0x2BE6: // Haldex Clutch Current: 2 bytes BE, × 0.001 A  (0x000F=15 → 0.015 A confirmed)
        if (payloadLen >= 5)
            udsClutchCurrent = (((uint16_t)frame.data[4] << 8) | frame.data[5]) * 0.001f;
        break;

    case 0x2BE7: // Haldex Clutch PWM: 1 byte, raw %  (0x00=0 → 0 % confirmed)
        if (payloadLen >= 4)
            udsClutchPWM = frame.data[4];
        break;

    case 0x2BF1: // Clutch Temperature: 2 bytes LE16, (D6×256+D5 − 22767)/100 °C
                 // Heat-sweep confirmed (VCDS, both modules); tracks 0x028D within ~1 °C:
                 //   0CQ 554C: 25227 → 24.6 °C … 28987 → 62.2 °C
                 //   0CQ 554D: 25477 → 27.1 °C … 29967 → 72.0 °C
        if (payloadLen >= 5)
            udsClutchTemp = ((int32_t)((uint16_t)frame.data[5] * 256 + frame.data[4]) - 22767) / 100.0f;
        break;

    case 0x2BE4: // Cooling Fin Temperature: 2 bytes LE16, (D6×256+D5 − 22767)/100 °C
                 // Same scaling as clutch (0x2BF1); fins shed heat so they stay cooler:
                 //   0CQ 554C: 25057 → 22.9 °C … 26957 → 41.9 °C
                 //   0CQ 554D: 25357 → 25.9 °C … 25657 → 28.9 °C (barely moved, as observed)
        if (payloadLen >= 5)
            udsCoolingFinTemp = ((int32_t)((uint16_t)frame.data[5] * 256 + frame.data[4]) - 22767) / 100.0f;
        break;

    case 0x2BE9: // Haldex Clutch Voltage: 2 bytes BE, × 0.001 V  (0x001F=31 → 0.031 V ≈ 0 V at rest confirmed)
        if (payloadLen >= 5)
            udsClutchVoltage = (((uint16_t)frame.data[4] << 8) | frame.data[5]) * 0.001f;
        break;

    default:
        break;
    }
}

void udsMQBTask(void *arg)
{
    static constexpr uint32_t kReqId  = 0x70FU; // physical request to Haldex ECU on Bus 1
    // VCDS uses 0x70F on Bus 0 which OpenHaldex bridges to Bus 1; the Haldex ECU
    // listens on 0x70F and responds at 0x779 — confirmed from SavvyCAN capture.
    // Response 0x779 is routed from parseCAN_hdx into udsRxQueue

    static constexpr uint16_t kDIDs[] = {
        0x0286, // Terminal Voltage
        0x028D, // Control Module Temperature
        0x2BE6, // Haldex Clutch Current
        0x2BE7, // Haldex Clutch PWM
        0x2BF1, // Clutch Temperature
        0x2BE4, // Cooling Fin Temperature
        0x2BE9, // Haldex Clutch Voltage
    };
    static constexpr uint8_t kDIDCount = sizeof(kDIDs) / sizeof(kDIDs[0]);

    udsRxQueue = xQueueCreate(8, sizeof(twai_message_t));

    while (1)
    {
        // Gen5 covers both MQB (50) and 0AY PQ-derived 0AY (51) - both use the same UDS stack.
        if (!liveDiagEnabled || externalDiagActive() || !hasCANHaldex || (haldexGeneration != 50 && haldexGeneration != 51) || analyzerMode || analyzerSerial)
        {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        xQueueReset(udsRxQueue); // flush any stale frames before opening session

        // Open extended diagnostic session (DiagnosticSessionControl 0x03)
        const uint8_t sessReq[] = {0x10, 0x03};
        udsSendFrame(kReqId, sessReq, sizeof(sessReq));

        twai_message_t sessResp;
        if (xQueueReceive(udsRxQueue, &sessResp, pdMS_TO_TICKS(2000)) != pdTRUE)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue; // no response - retry
        }
        // Verify positive session response [xx 50 03 ...]
        if (sessResp.data_length_code < 3 || sessResp.data[1] != 0x50 || sessResp.data[2] != 0x03)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Session open - poll DIDs round-robin, sending TesterPresent every 375 ms
        uint32_t lastTP = millis();
        uint8_t  didIdx = 0;

        while (liveDiagEnabled && !externalDiagActive() && hasCANHaldex && (haldexGeneration == 50 || haldexGeneration == 51) && !analyzerMode && !analyzerSerial)
        {
            if ((millis() - lastTP) >= 375U)
            {
                // TesterPresent with suppress-positive-response bit set (0x80) - no reply expected
                const uint8_t tpReq[] = {0x3E, 0x80};
                udsSendFrame(kReqId, tpReq, sizeof(tpReq));
                lastTP = millis();
            }

            const uint16_t did = kDIDs[didIdx];
            const uint8_t rdbiReq[] = {0x22, (uint8_t)(did >> 8), (uint8_t)(did & 0xFF)};
            udsSendFrame(kReqId, rdbiReq, sizeof(rdbiReq));

            // Drain-and-match: keep consuming frames until we get a positive RDBI
            // response or the window expires. This discards TesterPresent acks
            // (0x7E) and any other stale frames that can land before the DID reply.
            {
                const uint32_t kWindow = 500; // ms total receive window per DID
                uint32_t deadline = millis() + kWindow;
                twai_message_t rsp;
                for (;;)
                {
                    uint32_t elapsed = millis();
                    if (elapsed >= deadline) break;
                    uint32_t remaining = deadline - elapsed;
                    if (xQueueReceive(udsRxQueue, &rsp, pdMS_TO_TICKS(remaining < 50 ? remaining : 50)) == pdTRUE)
                    {
                        // Only process positive single-frame RDBI responses
                        if ((rsp.data[0] & 0xF0) == 0x00 && rsp.data[1] == 0x62 && rsp.data_length_code >= 4)
                        {
                            // Decode by the DID actually echoed in the response, not by `did`.
                            // This handles out-of-order or stale frames: if the response belongs
                            // to a DIFFERENT DID we still decode it, but keep waiting for `did`.
                            const uint16_t respDID = ((uint16_t)rsp.data[2] << 8) | rsp.data[3];
                            udsDecodeDID(respDID, rsp);
                            if (respDID == did) break; // got the one we were waiting for
                        }
                        // Else: TP ack, NRC, or other frame — discard and keep waiting
                    }
                }
            }

            didIdx = (didIdx + 1) % kDIDCount;
            vTaskDelay(pdMS_TO_TICKS(150)); // ~150 ms between polls -> full 9-DID cycle ≈ 1.35 s
        }

        // Session ended or conditions changed - clear all UDS data
        udsTerminalVoltage = 0.0f;
        udsModuleTemp      = 0.0f;
        udsClutchTemp      = 0.0f;
        udsCoolingFinTemp = 0.0f;
        udsClutchCurrent   = 0.0f;
        udsClutchPWM       = 0;
        udsClutchVoltage   = 0.0f;
        udsBlockagePct     = 0;
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
// parseCAN_hdx via tp20RxQueue.
// ===========================================================================

// A 16-bit CAN id is carried in the setup frame as [low8][ (id>>8)&0x0F | valid-flag ],
// where bit 4 (0x10) of the prefix means "invalid / not specified".
static inline void tp20EncodeId(uint32_t id, bool valid, uint8_t &low, uint8_t &prefix)
{
    low = (uint8_t)(id & 0xFF);
    prefix = (uint8_t)((id >> 8) & 0x0F);
    if (!valid) prefix |= 0x10;
}

static inline uint32_t tp20DecodeId(uint8_t low, uint8_t prefix, bool &valid)
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
    return (twai_transmit_v2(twai_bus_1, &msg, pdMS_TO_TICKS(20)) == ESP_OK);
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

static void tp20Disconnect(uint32_t testerTxId)
{
    if (testerTxId != 0)
    {
        const uint8_t disc = 0xA8; // channel disconnect
        tp20SendRaw(testerTxId, &disc, 1);
    }
    kwpTp20EcuTxId = 0;
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
    const uint32_t deadline = millis() + timeoutMs;

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

        if (haveFirst && off >= total) return (uint8_t)total;
    }
    return 0;
}

// VW/VAG KWP2000 measuring-block scaling. Each measured value is a 3-byte
// group [formula][a][b]; the formula index selects the conversion. Only the
// formulas observed on the Gen4 (0AY) Haldex captures are implemented; unknown
// formulas fall back to raw b. Confirmed against VCDS-displayed values:
//   0x06: 0.001*a*b      V   (voltage)      0x0E: 0.01*a*(b-100)  bar (pressure)
//   0x18: 0.001*a*b      A   (current)      0x1A: b - a           °C  (temperature)
//   0x21: 0.01*a*b       %   (duty)         0x5E: 0.1*a*(b-128)   Nm  (est. torque)
static float vagScale(uint8_t formula, uint8_t a, uint8_t b)
{
    switch (formula)
    {
    case 0x06: return 0.001f * a * b;                    // supply voltage: a=100,b=2 -> 0.2 V
    case 0x0E: return 0.01f * a * ((int)b - 100);        // oil pressure:   a=27,b=200 -> 27 bar
    case 0x18: return 0.001f * a * b;                    // valve current:  a=10,b=191 -> 1.910 A
    case 0x1A: return (float)((int)b - (int)a);          // temperature:    a=0,b=24  -> 24 °C
    case 0x21: return 0.01f * a * b;                     // clutch duty:    a=100,b=40 -> 40 %
    case 0x5E: return 0.1f * a * ((int)b - 128);         // est. torque:    a=160,b=255 -> 2032 Nm
    default:   return (float)b;                          // unknown formula - raw high byte
    }
}

// Decode the Gen4 (0AY) measuring groups into the named kwp* globals.
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
    case 0x01: // oil temp, plate temp, supply voltage
        kwpOilTemp       = value(0);
        kwpPlateTemp     = value(1);
        kwpSupplyVoltage = value(2);
        break;
    case 0x03: // oil pressure, est. torque, clutch duty, clutch valve current
        kwpOilPressure        = value(0);
        kwpEstTorque          = value(1);
        kwpClutchDuty         = value(2);
        kwpClutchValveCurrent = value(3);
        break;
    default:
        break;
    }
}

static void tp20ClearValues()
{
    kwpOilTemp = kwpPlateTemp = kwpSupplyVoltage = 0.0f;
    kwpOilPressure = kwpEstTorque = kwpClutchDuty = kwpClutchValveCurrent = 0.0f;
}

void kwpTp20Task(void *arg)
{
    (void)arg;
    // Measuring-block groups differ by generation (confirmed from VCDS captures):
    //   Gen2 (1K0): groups 0x01, 0x02, 0x7D
    //   Gen4 (0AY): groups 0x01 (oil/plate temp, voltage), 0x03 (pressure,
    //               est. torque, clutch duty, clutch valve current)
    // Response bytes are dumped raw into kwpTp20RawDump; known Gen4 fields are
    // additionally scaled into the kwp* globals via vagScale().
    static const uint8_t kGroupsGen2[] = {0x01, 0x02, 0x7D};
    static const uint8_t kGroupsGen4[] = {0x01, 0x03};
    static char lineBuf[8][56];

    tp20RxQueue = xQueueCreate(16, sizeof(twai_message_t));

    while (1)
    {
        if (!liveDiagEnabled || externalDiagActive() || !hasCANHaldex || (haldexGeneration != 2 && haldexGeneration != 4) ||
            analyzerMode || analyzerSerial)
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
        const uint8_t *kGroups = isGen4 ? kGroupsGen4 : kGroupsGen2;
        const uint8_t kGroupCount = isGen4
            ? (uint8_t)(sizeof(kGroupsGen4) / sizeof(kGroupsGen4[0]))
            : (uint8_t)(sizeof(kGroupsGen2) / sizeof(kGroupsGen2[0]));
        for (uint8_t i = 0; i < kGroupCount; i++)
            snprintf(lineBuf[i], sizeof(lineBuf[i]), "G%02X: --", kGroups[i]);

        uint8_t gi = 0;
        uint8_t consecutiveFails = 0;

        while (liveDiagEnabled && !externalDiagActive() && hasCANHaldex && (haldexGeneration == 2 || haldexGeneration == 4) &&
               !analyzerMode && !analyzerSerial)
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
        tp20ClearValues();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

UDS::UDS(twai_handle_t canBus)
    : _canBus(canBus)
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

    return (twai_transmit_v2(_canBus, &msg, 10 / portTICK_PERIOD_MS) == ESP_OK);
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

    return (twai_transmit_v2(_canBus, &msg, 10 / portTICK_PERIOD_MS) == ESP_OK);
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

    return (twai_transmit_v2(_canBus, &msg, 10 / portTICK_PERIOD_MS) == ESP_OK);
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

    return (twai_transmit_v2(_canBus, &msg, 10 / portTICK_PERIOD_MS) == ESP_OK);
}

bool UDS::receiveFrame(twai_message_t &frame, uint32_t timeoutMs)
{
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
        size_t copyLen = min<size_t>(dataLen, responseLen);
        memcpy(responseBuf, &frame.data[1], copyLen);
        responseLen = copyLen;
        return true;
    }

    if (pci == 0x10)
    {
        // First Frame
        uint16_t totalLength = ((frame.data[0] & 0x0F) << 8) | frame.data[1];
        size_t bytesCopied = min<size_t>(6, totalLength);
        if (bytesCopied > responseLen)
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
            if (payloadLen > responseLen - receivedTotal)
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
