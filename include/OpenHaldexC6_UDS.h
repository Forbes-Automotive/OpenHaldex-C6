#pragma once

#include <OpenHaldexC6_defs.h>

namespace OpenHaldexC6
{
    class UDS
    {
    public:
        explicit UDS(twai_handle_t canBus = twai_bus_0);

        // requestId: ECU request listener, usually 0x7E0 (physical) for VW
        // responseId: ECU response source, usually 0x7E8 for VW
        bool sendRequest(uint32_t requestId,
                         uint32_t responseId,
                         const uint8_t *requestData,
                         size_t requestLen,
                         uint8_t *responseBuf,
                         size_t &responseLen,
                         uint32_t timeoutMs = 2000);

        bool readDataByIdentifier(uint32_t requestId,
                                  uint32_t responseId,
                                  uint16_t did,
                                  uint8_t *dataOut,
                                  size_t &dataOutLen,
                                  uint32_t timeoutMs = 2000);

        bool diagnosticSessionControl(uint32_t requestId,
                                      uint32_t responseId,
                                      uint8_t sessionType,
                                      uint32_t timeoutMs = 1500);

    private:
        bool sendSingleFrame(uint32_t canId, const uint8_t *payload, uint8_t length);
        bool sendFirstFrame(uint32_t canId, const uint8_t *payload, uint16_t length);
        bool sendConsecutiveFrame(uint32_t canId, const uint8_t *payload, uint8_t sequenceCounter, uint8_t length);
        bool sendFlowControl(uint32_t canId, uint8_t flowStatus, uint8_t blockSize, uint8_t stMin);
        bool receiveFrame(twai_message_t &frame, uint32_t timeoutMs);

        twai_handle_t _canBus;
    };

    // Known VW-DSG and Haldex USAGE constants (for reference).
    static constexpr uint32_t kVWRequestId = 0x7E0;
    static constexpr uint32_t kVWResponseId = 0x7E8;
    static constexpr uint32_t kHaldexRequestId = 0x7E0;
    static constexpr uint32_t kHaldexResponseId = 0x7E8;
}
