#pragma once

#include <OpenHaldexC6_defs.h>

void setupOTA();
bool isSystemSafeForOTA();
void confirmFirmwareValidity();
bool needsFirmwareConfirmation();
bool isOTAUpdateInProgress();
String getFirmwareVersion();