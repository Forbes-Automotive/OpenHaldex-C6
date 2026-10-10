#pragma once

#include <OpenHaldexC6_defs.h>

// Bluetooth LE GATT server for the DashCAN mobile app. Contract: documents/MOBILE_APP_OPENHALDEX.md.
void setupBLE();       // creates the BLE task; the task brings the stack up/down (bleEnabled, low power)
bool bleIsConnected(); // a phone is connected (shown on /api/status)
bool bleCodeIsRequired(); // a phone is bonded, so new phones need the pairing code (blePasskey)
bool bleForgetBonds(); // drop every bonded phone; false when BLE is not running (nothing done)
