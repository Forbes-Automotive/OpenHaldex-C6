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

    case 0x028D: // Control Module Temperature: 1 byte, temp = raw − 55 °C  (0x52=82 → 27 °C confirmed)
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
                 // D5=0x53,D6=0x63 → LE16=25427 → 26.60 °C confirmed
        if (payloadLen >= 5)
            udsClutchTemp = ((int32_t)((uint16_t)frame.data[5] * 256 + frame.data[4]) - 22767) / 100.0f;
        break;

    case 0x2BE4: // Cooling Fin Temperature: 2 bytes LE16, (D6×256+D5 − 22767)/100 °C
                 // D5=0x71,D6=0x63 → LE16=25457 → 26.90 °C confirmed
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
        if (!udsMQBEnabled || !hasCANHaldex || (haldexGeneration != 50 && haldexGeneration != 51) || analyzerMode || analyzerSerial)
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

        while (udsMQBEnabled && hasCANHaldex && (haldexGeneration == 50 || haldexGeneration == 51) && !analyzerMode && !analyzerSerial)
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
