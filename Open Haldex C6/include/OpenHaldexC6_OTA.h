#ifndef OPENHALDEXC6_OTA_H
#define OPENHALDEXC6_OTA_H

#include <OpenHaldexC6_defs.h>

void setupOTA();
bool isSystemSafeForOTA();
void confirmFirmwareValidity();
bool needsFirmwareConfirmation();
bool isOTAUpdateInProgress();
String getFirmwareVersion();

#endif // OPENHALDEXC6_OTA_H
