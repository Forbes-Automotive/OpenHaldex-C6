#pragma once

#include <OpenHaldexC6_defs.h>

void setupOTA();
bool isSystemSafeForOTA(bool allowEnforce = true);
bool needsFirmwareConfirmation();
void otaRollbackTick();     // call periodically from loop(): confirms a pending OTA image once the device has proven itself
extern "C" bool verifyRollbackLater(); // arduino-esp32 hook (C linkage): defer the core's automatic image confirmation to otaRollbackTick()
bool isOTAUpdateInProgress();
String getFirmwareVersion();