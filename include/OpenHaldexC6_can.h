#pragma once

#include <OpenHaldexC6_defs.h>

// CAN is three files (V11.00.14):
//   OpenHaldexC6_can.cpp  shared: driver set-up, recovery, transmit, census, loop test, 0x6B0 broadcast
//   OpenHaldexC6_CHS.cpp  Bus 0 - chassis: setupCHS(), chassis decode per generation, parseCANChs()
//   OpenHaldexC6_HDX.cpp  Bus 1 - Haldex:  setupHDX(), feedback decode per generation, parseCANHdx()

// ---- Setup ----
void setupCAN(); // sets the bus speed for the generation, then setupCHS() + setupHDX()
void setupCHS(); // install + start controller 0 (chassis)
void setupHDX(); // install + start controller 1 (Haldex)
void canInstallBus(uint8_t controllerId, int txPin, int rxPin, twai_handle_t *bus); // shared driver settings

// ---- Tasks ----
void broadcastOpenHaldex(void *arg);
void parseCANChs(void *arg);
void parseCANHdx(void *arg);
void canBusRecovery();

// Counted transmit: firmware transmits on the bridge and diagnostic paths go
// through here so a failed send (TX queue still full after the 10 ms wait,
// driver stopped / bus-off) is tallied in canTxDropBus0/1 instead of vanishing.
// Returns true when the frame was queued.
bool canTransmit(twai_handle_t bus, const twai_message_t *msg);

// ---- Shared by the CHS and HDX receive tasks ----
bool canLoopTestCatch(const twai_message_t &m, uint8_t rxBus); // true = loop-test frame, swallow it
void chsRxCensus(const twai_message_t &m);                      // Bus 0 per-ID tally (CAN Test card)
void hdxRxCensus(const twai_message_t &m);                      // Bus 1 per-ID tally (serial lab RXIDS)
