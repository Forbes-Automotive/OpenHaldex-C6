#include <OpenHaldexC6_UDS.h>

using namespace OpenHaldexC6;

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
