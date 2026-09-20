#pragma once

#include <OpenHaldexC6_defs.h>

void broadcastOpenHaldex(void *arg);
void parseCAN_chs(void *arg);
void parseCAN_hdx(void *arg);
void setupCAN();
void canBusRecovery();

// Counted transmit: firmware transmits on the bridge and diagnostic paths go
// through here so a failed send (TX queue still full after the 10 ms wait,
// driver stopped / bus-off) is tallied in canTxDropBus0/1 instead of vanishing.
// Returns true when the frame was queued.
bool canTransmit(twai_handle_t bus, const twai_message_t *msg);