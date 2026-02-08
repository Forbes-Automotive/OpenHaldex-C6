#ifndef OPENHALDEXC6_CAN_H
#define OPENHALDEXC6_CAN_H

#include <OpenHaldexC6_defs.h>

void broadcastOpenHaldex(void *arg);
void parseCAN_chs(void *arg);
void parseCAN_hdx(void *arg);
void setupCAN();

#endif // OPENHALDEXC6_CAN_H
