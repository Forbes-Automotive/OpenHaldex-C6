#pragma once

#include <OpenHaldexC6_defs.h>

void setupOTA();
bool isSystemSafeForOTA(bool allowEnforce = true);
bool needsFirmwareConfirmation();
void otaRollbackTick();     // call periodically from loop(): confirms a pending OTA image once the device has proven itself
extern "C" bool verifyRollbackLater(); // arduino-esp32 hook (C linkage): defer the core's automatic image confirmation to otaRollbackTick()
bool isOTAUpdateInProgress();
String getFirmwareVersion();

// Web-client hold for the low-power AP shutdown. The idle check in
// updateTriggers() only counts stations joined to our own AP, so a phone
// reaching the UI through the home router (bridge mode) looked like nobody
// was there and the controller could switch WiFi off mid-update. Handlers the
// UI polls call otaNoteWebActivity(); otaWebClientActive() is true while a
// request has been seen recently or an upload is being written.
void otaNoteWebActivity();
bool otaWebClientActive();