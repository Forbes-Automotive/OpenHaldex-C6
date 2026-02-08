#ifndef OPENHALDEXC6_WIFI_H
#define OPENHALDEXC6_WIFI_H

#include <OpenHaldexC6_defs.h>

String getFirmwareVersion();
void connectWifi();
void disconnectWifi();
void setupUI();
void generalCallback(Control *sender, int type);
void extendedCallback(Control *sender, int type, void *param);
void updateLabels(void *arg);

#endif // OPENHALDEXC6_WIFI_H
