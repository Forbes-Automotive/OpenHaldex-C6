#pragma once

#include <OpenHaldexC6_defs.h>

namespace OpenHaldexC6
{
    class UDS
    {
    public:
        // rxQueue: when given, responses are read from this queue (filled by a
        // copy-tap in the parse task for that bus) instead of the TWAI driver,
        // so the helper never competes with the gateway for frames. Only the
        // bench/no-bridge case should leave it null.
        explicit UDS(twai_handle_t canBus = twaiBus0, QueueHandle_t rxQueue = nullptr);

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
        QueueHandle_t _rxQueue;
    };

    // Known VW-DSG and Haldex USAGE constants (for reference).
    static constexpr uint32_t kVWRequestId = 0x7E0;
    static constexpr uint32_t kVWResponseId = 0x7E8;
    static constexpr uint32_t kHaldexRequestId = 0x7E0;
    static constexpr uint32_t kHaldexResponseId = 0x7E8;
}

// UDS MQB diagnostic polling task (Gen 5 only).
// Sends requests to 0x771 on Bus 1; reads responses from udsRxQueue
// (parseCANHdx routes 0x779 frames there when udsRxQueue != nullptr).
void udsMQBTask(void *arg);

// KWP2000-over-VW-TP2.0 diagnostic task (Gen2 / Gen4 PQ Haldex).
// Opens a TP2.0 channel to the AWD module (see KWP_TP20_* in defs.h), starts a
// KWP session and sweeps measuring-block groups, dumping the raw response bytes
// into kwpTp20RawDump for capture. Frames are routed via tp20RxQueue.
void kwpTp20Task(void *arg);

// Gen2 SecurityAccess (V11.00.10): forget a saved key refusal so the login is tried once more.
void gen2LoginReset();

// Fault codes - read / clear (V11.00.5). The API sets dtcRequest (1 = read,
// 2 = clear); dtcTask runs it with the protocol for the live generation (UDS,
// KWP2000 over TP2.0, or Volvo D2) and fills the results below. While dtcBusy
// the live-diag pollers stand down.
enum
{
    DTC_IDLE = 0,
    DTC_BUSY,
    DTC_DONE,
    DTC_ERROR,
};
#define DTC_MAX 24
struct DtcEntry
{
    char code[14];   // e.g. "U0121-00", "01312", "AWD-0015"
    char detail[64]; // status / note
};
extern volatile uint8_t dtcState;
extern volatile uint8_t dtcRequest;
extern volatile bool dtcBusy;
extern uint8_t dtcLastAction; // 1 = read, 2 = clear
extern uint8_t dtcCount;
extern DtcEntry dtcList[DTC_MAX];
extern char dtcProtocol[24];
extern char dtcMessage[96];
extern uint32_t dtcDoneMs;
const char *dtcProtocolForGeneration(uint8_t gen); // nullptr = not available for that generation
void dtcTask(void *arg);
