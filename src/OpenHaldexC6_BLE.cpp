#include <OpenHaldexC6_BLE.h>
#include <OpenHaldexC6_OTA.h> // isOTAUpdateInProgress()
#include <OpenHaldexC6_Settings.h> // applyDrivingSetting(), buildDrivingSettings()
#include <NimBLEDevice.h>
#include "esp_mac.h"

// =============================================================================
// BLE link to the DashCAN mobile app
// =============================================================================
// GATT-native contract (documents/MOBILE_APP_OPENHALDEX.md): every setting is its own
// characteristic (read / write / notify) and live data is one packed binary
// Status characteristic, so there is no command parser and no reassembly.
//
// Threading: NimBLE callbacks run on the NimBLE host task and only queue the
// written byte. bleTask (low priority, below every CAN/frame task) owns the
// stack: it brings it up/down, applies writes through the same entry points as
// the web UI (requestMode / setControllerDisabled) and sends every notify.
//
// A write is a request; the notified value is the truth. After each write the
// characteristic is set back to the value actually in effect and notified, so
// a rejected write (out of range, controller disabled, wrong length) needs no
// separate error channel.
//
// Low power: BLE follows the WiFi AP - the stack is shut down (deinit) while
// lowPowerMode is set so the controller can light-sleep, and comes back with
// WiFi. A connected phone does NOT hold the device awake: phones auto-reconnect
// indefinitely, which would keep a parked car from ever sleeping.

#define BLE_SERVICE_UUID "6254A001-C7B7-494F-A2FB-76FB74A7DDF0"
#define BLE_MODE_UUID "6254A002-C7B7-494F-A2FB-76FB74A7DDF0"
#define BLE_CONTROLLER_UUID "6254A003-C7B7-494F-A2FB-76FB74A7DDF0"
#define BLE_STATUS_UUID "6254A004-C7B7-494F-A2FB-76FB74A7DDF0"
#define BLE_INFO_UUID "6254A005-C7B7-494F-A2FB-76FB74A7DDF0"
#define BLE_SETTINGS_UUID "6254A006-C7B7-494F-A2FB-76FB74A7DDF0" // driving settings (documents/MOBILE_APP_OPENHALDEX.md "Settings")
#define BLE_DIAG_UUID "6254A007-C7B7-494F-A2FB-76FB74A7DDF0"     // Haldex live diagnostics, 1 Hz
#define BLE_PAIRING_UUID "6254A008-C7B7-494F-A2FB-76FB74A7DDF0"  // pairing code, readable by paired phones only
#define BLE_PAIRING_LEN 5                                          // u8 codeRequired, u32 passkey LE

#define BLE_PROTO_VERSION 1
#define BLE_STATUS_LAYOUT 1
#define BLE_STATUS_LEN 17
#define BLE_TASK_TICK_MS 50
#define BLE_STATUS_PERIOD_MS 100 // 10 Hz
#define BLE_STEER_UNKNOWN 0x8000  // INT16_MIN
#define BLE_DIAG_LAYOUT 1
#define BLE_DIAG_LEN 20 // exactly the default ATT payload: no MTU request needed
#define BLE_DIAG_PERIOD_MS 1000
#define BLE_DIAG_UDS_STALE_MS 5000 // UDS values persist after polling stops; older than this = unknown
#define BLE_DIAG_I16_UNKNOWN 0x8000
#define BLE_DIAG_U16_UNKNOWN 0xFFFF

// Diag source byte
#define BLE_DIAG_SRC_NONE 0 // live diag off, Haldex CAN down, scan tool active, or a generation without decoded values
#define BLE_DIAG_SRC_UDS 1  // Gen5 family (UDS ReadDataByIdentifier)
#define BLE_DIAG_SRC_KWP 2  // Gen4 0AY (KWP2000 over TP2.0 measuring blocks)

// Status flags (u16 at offset 15)
#define BLE_FLAG_CHASSIS_CAN (1u << 0)
#define BLE_FLAG_HALDEX_CAN (1u << 1)
#define BLE_FLAG_BUS_FAILURE (1u << 2)
#define BLE_FLAG_TC_OFF (1u << 3)  // driver switched traction control off (TC force mode trigger)
#define BLE_FLAG_ASR_OFF (1u << 4) // ASR off
#define BLE_FLAG_HAZARD (1u << 5)
#define BLE_FLAG_EXT_BUTTON (1u << 6)
#define BLE_FLAG_BRAKE_IN (1u << 7)
#define BLE_FLAG_HANDBRAKE_IN (1u << 8)
#define BLE_FLAG_TEMP_PROTECTION (1u << 9)
#define BLE_FLAG_COUPLING_OPEN (1u << 10)
#define BLE_FLAG_SPEED_LIMIT (1u << 11)
#define BLE_FLAG_STANDALONE (1u << 12)
#define BLE_FLAG_DIAG_TOOL (1u << 13)

enum : uint8_t
{
  BLE_WRITE_MODE,
  BLE_WRITE_CONTROLLER,
  BLE_WRITE_SETTING
};

struct BleWrite
{
  uint8_t target; // BLE_WRITE_*
  uint8_t len;    // written length (Mode / Controller: only 1 is valid)
  uint8_t value;  // first byte (Settings: the setting id)
  uint16_t arg;   // Settings only: the value after the id, little-endian (1 or 2 bytes)
};

static QueueHandle_t bleWriteQueue = nullptr;
static bool bleRunning = false;
static volatile bool bleConnected = false;
static volatile bool bleForgetBondsRequest = false;

// Pairing ("trust on first use"): while no phone is bonded, pairing is
// "Just Works" (no code). Once the first phone has bonded, new phones must
// enter the 6-digit pairing code (blePasskey, shown on the web UI and readable
// by paired phones over the Pairing characteristic). Phones bonded before the
// code was required keep working. Forget Paired Phones clears the bonds,
// makes a new code and opens pairing again.
// NimBLE does not reliably refuse a Just Works pairing when we ask for MITM,
// so onAuthenticationComplete enforces it: a new, unauthenticated bond while
// the code is required is deleted and the link dropped.
static volatile bool bleCodeRequired = false;
static std::vector<NimBLEAddress> trustedPeers; // bonds that may stay unauthenticated (made while pairing was open)
static SemaphoreHandle_t bleSecMutex = nullptr;

static NimBLECharacteristic *chrMode = nullptr;
static NimBLECharacteristic *chrController = nullptr;
static NimBLECharacteristic *chrStatus = nullptr;
static NimBLECharacteristic *chrInfo = nullptr;
static NimBLECharacteristic *chrSettings = nullptr;
static NimBLECharacteristic *chrDiag = nullptr;
static NimBLECharacteristic *chrPairing = nullptr;

static bool isTrustedPeer(const NimBLEAddress &addr)
{
  for (const auto &a : trustedPeers)
    if (a == addr)
      return true;
  return false;
}

static void updatePairingValue()
{
  if (!chrPairing)
    return;
  const uint32_t k = blePasskey;
  const uint8_t v[BLE_PAIRING_LEN] = {(uint8_t)(bleCodeRequired ? 1 : 0), (uint8_t)(k & 0xFF), (uint8_t)((k >> 8) & 0xFF),
                                      (uint8_t)((k >> 16) & 0xFF), (uint8_t)((k >> 24) & 0xFF)};
  chrPairing->setValue(v, sizeof(v));
}

// Applies to pairings that start after the call; existing bonds are unaffected.
static void applyPairingMode(bool codeRequired)
{
  bleCodeRequired = codeRequired;
  if (codeRequired)
  {
    NimBLEDevice::setSecurityAuth(true, true, true); // bonding, MITM (passkey), LE Secure Connections
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);
    NimBLEDevice::setSecurityPasskey(blePasskey);
  }
  else
  {
    NimBLEDevice::setSecurityAuth(true, false, true); // bonding, no MITM ("Just Works"), LE Secure Connections
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  }
  updatePairingValue();
}

class BleServerCallbacks : public NimBLEServerCallbacks
{
  // A phone whose app was closed or killed can keep its old link open (the
  // phone OS still holds it), so the device never sees a disconnect. With one
  // connection slot it then stopped advertising and the restarted app could
  // not find it until the OpenHaldex was rebooted. Now there are two slots,
  // advertising continues while one is free, and a new link from the same
  // phone (identity address) drops that phone's older, stale link.
  void onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo) override
  {
    for (uint16_t handle : server->getPeerDevices())
    {
      if (handle != connInfo.getConnHandle() &&
          server->getPeerInfoByHandle(handle).getIdAddress() == connInfo.getIdAddress())
      {
        server->disconnect(handle);
        DEBUG("BLE - dropped a stale link from the same phone");
      }
    }
    bleConnected = true;
    if (server->getConnectedCount() < CONFIG_BT_NIMBLE_MAX_CONNECTIONS)
    {
      NimBLEDevice::startAdvertising(); // stays findable while a slot is free
    }
  }

  void onDisconnect(NimBLEServer *server, NimBLEConnInfo &connInfo, int reason) override
  {
    // The peer list is already updated here; advertising restarts by itself
    // (advertiseOnDisconnect).
    bleConnected = server->getConnectedCount() > 0;
  }

  uint32_t onPassKeyDisplay() override
  {
    return blePasskey;
  }

  void onAuthenticationComplete(NimBLEConnInfo &connInfo) override
  {
    if (!connInfo.isEncrypted())
      return;
    const NimBLEAddress peer = connInfo.getIdAddress();
    xSemaphoreTake(bleSecMutex, portMAX_DELAY);
    if (!bleCodeRequired)
    {
      if (connInfo.isBonded()) // the first phone: from now on new phones need the code
      {
        if (!isTrustedPeer(peer))
          trustedPeers.push_back(peer);
        applyPairingMode(true);
        DEBUG("BLE - first phone paired, pairing code required from now on");
      }
    }
    else if (!connInfo.isAuthenticated() && !isTrustedPeer(peer))
    {
      // Paired without the code while it is required: undo it.
      NimBLEDevice::deleteBond(peer);
      NimBLEDevice::getServer()->disconnect(connInfo.getConnHandle());
      DEBUG("BLE - refused a pairing without the code");
    }
    xSemaphoreGive(bleSecMutex);
  }
};

class BleWriteCallbacks : public NimBLECharacteristicCallbacks
{
  void onWrite(NimBLECharacteristic *chr, NimBLEConnInfo &connInfo) override
  {
    // Belt and braces for the pairing-code rule: a link that paired without
    // the code while it is required never gets a write through (it is being
    // disconnected by onAuthenticationComplete anyway).
    if (bleCodeRequired && !connInfo.isAuthenticated())
    {
      xSemaphoreTake(bleSecMutex, portMAX_DELAY);
      const bool trusted = isTrustedPeer(connInfo.getIdAddress());
      xSemaphoreGive(bleSecMutex);
      if (!trusted)
        return;
    }
    NimBLEAttValue value = chr->getValue();
    BleWrite w;
    w.target = (chr == chrMode) ? BLE_WRITE_MODE : (chr == chrController) ? BLE_WRITE_CONTROLLER : BLE_WRITE_SETTING;
    w.len = (uint8_t)(value.length() > 255 ? 255 : value.length());
    w.value = value.length() ? value[0] : 0;
    w.arg = value.length() >= 2 ? value[1] : 0;
    if (value.length() >= 3)
      w.arg |= (uint16_t)value[2] << 8;
    if (xQueueSend(bleWriteQueue, &w, 0) != pdTRUE)
    {
      DEBUG("BLE - write queue full, dropped write");
    }
  }
};

static BleServerCallbacks serverCallbacks;
static BleWriteCallbacks writeCallbacks;

static void put16(uint8_t *p, uint32_t v)
{
  if (v > 0xFFFE)
    v = 0xFFFE; // 0xFFFF is the "unknown" sentinel
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)(v >> 8);
}

static uint8_t clampPct(float v)
{
  if (v < 0.0f)
    return 0;
  if (v > 100.0f)
    return 100;
  return (uint8_t)(v + 0.5f);
}

// Signed steering-wheel angle in 0.1 deg (the web UI shows the magnitude only).
// Negative when the VW sign bit (LW1_LRW_Sign / LWI_VZ_Lenkradwinkel) is set.
// Unlike the web view this is not limited to the gens that use steering for lock
// scaling: every gen whose chassis bus carries LW_1 / LWI_01 reports it. GM/Ford
// (41/42) are excluded because foreign frames on those IDs would be misdecoded.
static bool bleSteeringDeci(int16_t &out)
{
  const bool fresh = hasCANChassis && received_steering_ms != 0 &&
                     (millis() - received_steering_ms) <= steeringStaleMs &&
                     haldexGeneration != 41 && haldexGeneration != 42;
  if (!fresh)
  {
    return false;
  }
  float deci = fabsf(received_steering_angle) * 10.0f + 0.5f;
  if (deci > 32767.0f)
    deci = 32767.0f;
  out = received_steering_negative ? -(int16_t)deci : (int16_t)deci;
  return true;
}

// Same sources and null rules as /api/status (statusOutgoing in _API.cpp),
// except the steering angle (see bleSteeringDeci).
static void buildStatus(uint8_t *p)
{
  static uint8_t seq = 0;
  const bool chassisOk = hasCANChassis;
  const bool haldexOk = hasCANHaldex;

  p[0] = BLE_STATUS_LAYOUT;
  p[1] = seq++;
  p[2] = (uint8_t)state.mode;
  p[3] = disableController ? 0 : 1;
  p[4] = clampPct(lock_target);
  p[5] = haldexOk ? (received_haldex_engagement > 0xFE ? 0xFE : received_haldex_engagement) : 0xFF;

  if (chassisOk)
  {
    put16(&p[6], received_vehicle_speed);
    p[8] = clampPct(received_pedal_value);
    put16(&p[9], received_vehicle_rpm);
    put16(&p[11], received_vehicle_boost);
  }
  else
  {
    p[6] = p[7] = 0xFF;
    p[8] = 0xFF;
    p[9] = p[10] = 0xFF;
    p[11] = p[12] = 0xFF;
  }

  int16_t steerDeci = 0;
  const uint16_t steerRaw = bleSteeringDeci(steerDeci) ? (uint16_t)steerDeci : BLE_STEER_UNKNOWN;
  p[13] = (uint8_t)(steerRaw & 0xFF);
  p[14] = (uint8_t)(steerRaw >> 8);

  uint16_t flags = 0;
  if (chassisOk)
  {
    flags |= BLE_FLAG_CHASSIS_CAN;
    if (tcForceModeFlag)
      flags |= BLE_FLAG_TC_OFF;
    if (asrForceModeFlag)
      flags |= BLE_FLAG_ASR_OFF;
    if (hazardForceModeFlag)
      flags |= BLE_FLAG_HAZARD;
  }
  if (haldexOk)
  {
    flags |= BLE_FLAG_HALDEX_CAN;
    if (received_temp_protection)
      flags |= BLE_FLAG_TEMP_PROTECTION;
    if (received_coupling_open)
      flags |= BLE_FLAG_COUPLING_OPEN;
    if (received_speed_limit)
      flags |= BLE_FLAG_SPEED_LIMIT;
  }
  if (isBusFailure)
    flags |= BLE_FLAG_BUS_FAILURE;
  if (extButtonForceModeFlag)
    flags |= BLE_FLAG_EXT_BUTTON;
  if (brakeSignalActive)
    flags |= BLE_FLAG_BRAKE_IN;
  if (handbrakeSignalActive)
    flags |= BLE_FLAG_HANDBRAKE_IN;
  if (isStandalone)
    flags |= BLE_FLAG_STANDALONE;
  if (externalDiagActive())
    flags |= BLE_FLAG_DIAG_TOOL;
  p[15] = (uint8_t)(flags & 0xFF);
  p[16] = (uint8_t)(flags >> 8);
}

static void putI16(uint8_t *p, float v, float scale)
{
  float x = v * scale;
  x = x > 32767.0f ? 32767.0f : (x < -32767.0f ? -32767.0f : x); // -32768 is the sentinel
  const int16_t i = (int16_t)(x < 0 ? x - 0.5f : x + 0.5f);
  p[0] = (uint8_t)(i & 0xFF);
  p[1] = (uint8_t)((uint16_t)i >> 8);
}

static void putU16Scaled(uint8_t *p, float v, float scale)
{
  float x = v * scale;
  x = x < 0.0f ? 0.0f : (x > 65534.0f ? 65534.0f : x); // 0xFFFF is the sentinel
  put16(p, (uint32_t)(x + 0.5f));
}

static void putUnknown16(uint8_t *p, uint16_t sentinel)
{
  p[0] = (uint8_t)(sentinel & 0xFF);
  p[1] = (uint8_t)(sentinel >> 8);
}

// Haldex live diagnostics, the same values as /api/status "uds" / "kwp"
// (documents/MOBILE_APP_OPENHALDEX.md "Diag"). Fields a source does not provide are
// unknown sentinels.
static void buildDiag(uint8_t *p)
{
  const bool common = liveDiagEnabled && hasCANHaldex && !externalDiagActive() && !analyzerMode && !analyzerSerial;
  const bool uds = common && isGen5Family() && udsLastDecodeMs != 0 &&
                   (millis() - udsLastDecodeMs) <= BLE_DIAG_UDS_STALE_MS;
  const bool kwp = common && haldexGeneration == 4 && kwpTp20Connected;

  p[0] = BLE_DIAG_LAYOUT;
  p[1] = uds ? BLE_DIAG_SRC_UDS : kwp ? BLE_DIAG_SRC_KWP : BLE_DIAG_SRC_NONE;
  for (int o = 2; o <= 8; o += 2)
    putUnknown16(&p[o], BLE_DIAG_I16_UNKNOWN); // temps
  for (int o = 10; o <= 14; o += 2)
    putUnknown16(&p[o], BLE_DIAG_U16_UNKNOWN); // voltage, current, duty
  putUnknown16(&p[16], BLE_DIAG_I16_UNKNOWN);  // oil pressure
  putUnknown16(&p[18], BLE_DIAG_I16_UNKNOWN);  // torque

  if (uds)
  {
    putI16(&p[2], udsClutchTemp, 10.0f);              // clutch temp, 0.1 °C
    putI16(&p[6], udsModuleTemp, 10.0f);              // module temp, 0.1 °C
    putI16(&p[8], udsCoolingFinTemp, 10.0f);          // cooling fin temp, 0.1 °C
    putU16Scaled(&p[10], udsTerminalVoltage, 100.0f); // supply, 0.01 V
    putU16Scaled(&p[12], udsClutchCurrent, 1000.0f);  // clutch current, mA
    putU16Scaled(&p[14], (float)udsClutchPWM, 10.0f); // clutch duty, 0.1 %
  }
  else if (kwp)
  {
    putI16(&p[2], kwpPlateTemp, 10.0f); // clutch plate temp, 0.1 °C
    putI16(&p[4], kwpOilTemp, 10.0f);   // oil temp, 0.1 °C
    putU16Scaled(&p[10], kwpSupplyVoltage, 100.0f);
    putU16Scaled(&p[12], kwpClutchValveCurrent, 1000.0f);
    putU16Scaled(&p[14], kwpClutchDuty, 10.0f);
    putI16(&p[16], kwpOilPressure, 100.0f); // oil pressure, 0.01 bar
    putI16(&p[18], kwpEstTorque, 1.0f);     // estimated torque, Nm
  }
}

static void updateInfo()
{
  const uint8_t info[3] = {BLE_PROTO_VERSION, haldexGeneration, (uint8_t)(isStandalone ? 1 : 0)};
  chrInfo->setValue(info, sizeof(info));
}

static void bleStart()
{
  uint8_t mac[6] = {0};
  esp_read_mac(mac, ESP_MAC_BT);
  char name[20];
  snprintf(name, sizeof(name), "OpenHaldex-%02X%02X", mac[4], mac[5]);

  NimBLEDevice::init(name);
  // "Just Works" pairing: bonding + LE Secure Connections, no MITM, so no
  // passkey to type - the phone at most asks to confirm. Writes still need an
  // encrypted (bonded) link; anyone in range can pair, a deliberate trade-off
  // for an easy first connection (was passkey / DisplayOnly before).
  // Phones bonded so far stay valid without the code; pairing is open only
  // while there are none (see applyPairingMode).
  xSemaphoreTake(bleSecMutex, portMAX_DELAY);
  trustedPeers.clear();
  for (int i = 0; i < NimBLEDevice::getNumBonds(); i++)
    trustedPeers.push_back(NimBLEDevice::getBondedAddress(i));
  applyPairingMode(!trustedPeers.empty());
  xSemaphoreGive(bleSecMutex);

  NimBLEServer *server = NimBLEDevice::createServer();
  server->setCallbacks(&serverCallbacks, false);

  NimBLEService *service = server->createService(BLE_SERVICE_UUID);
  chrMode = service->createCharacteristic(
      BLE_MODE_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC | NIMBLE_PROPERTY::NOTIFY,
      1);
  chrController = service->createCharacteristic(
      BLE_CONTROLLER_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC | NIMBLE_PROPERTY::NOTIFY,
      1);
  chrStatus = service->createCharacteristic(BLE_STATUS_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY, BLE_STATUS_LEN);
  chrInfo = service->createCharacteristic(BLE_INFO_UUID, NIMBLE_PROPERTY::READ, 3);
  chrSettings = service->createCharacteristic(
      BLE_SETTINGS_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC | NIMBLE_PROPERTY::NOTIFY,
      DRIVING_SETTINGS_LEN);
  chrDiag = service->createCharacteristic(BLE_DIAG_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY, BLE_DIAG_LEN);
  // Encrypted read: an unpaired phone has to pair first (with the code, once one is required).
  chrPairing = service->createCharacteristic(BLE_PAIRING_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC, BLE_PAIRING_LEN);
  updatePairingValue();
  chrMode->setCallbacks(&writeCallbacks);
  chrController->setCallbacks(&writeCallbacks);
  chrSettings->setCallbacks(&writeCallbacks);

  const uint8_t mode = (uint8_t)state.mode;
  const uint8_t ctrl = disableController ? 0 : 1;
  chrMode->setValue(&mode, 1);
  chrController->setValue(&ctrl, 1);
  uint8_t status[BLE_STATUS_LEN];
  buildStatus(status);
  chrStatus->setValue(status, sizeof(status));
  updateInfo();
  uint8_t settings[DRIVING_SETTINGS_LEN];
  buildDrivingSettings(settings);
  chrSettings->setValue(settings, sizeof(settings));
  uint8_t diag[BLE_DIAG_LEN];
  buildDiag(diag);
  chrDiag->setValue(diag, sizeof(diag));

  // Standard Device Information Service
  NimBLEService *dis = server->createService("180A");
  dis->createCharacteristic("2A29", NIMBLE_PROPERTY::READ)->setValue("Forbes Automotive");
  dis->createCharacteristic("2A24", NIMBLE_PROPERTY::READ)->setValue("OpenHaldex-C6");
  dis->createCharacteristic("2A26", NIMBLE_PROPERTY::READ)->setValue(FW_VERSION);

  // Services start with the server when advertising starts.
  // Flags + 128-bit service UUID fill the advertising packet; the name goes in the scan response.
  NimBLEAdvertisementData advData;
  advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
  advData.setCompleteServices(NimBLEUUID(BLE_SERVICE_UUID));
  NimBLEAdvertisementData scanData;
  scanData.setName(name);
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->setAdvertisementData(advData);
  adv->setScanResponseData(scanData);
  adv->start(); // also starts the GATT server

  // Bonded phones cache the GATT table (iOS and Android). After a firmware
  // update that adds a characteristic (Settings, Diag) they would keep using
  // the old table and never see it. A Service Changed indication on every
  // start makes them re-discover; NimBLE stores it for bonded phones that are
  // not connected and sends it when they reconnect. Cost: one extra service
  // discovery per connection after a reboot.
  server->sendServiceChangedIndication();

  bleRunning = true;
  DEBUG("BLE - advertising as %s", name);
}

static void bleStop()
{
  NimBLEDevice::deinit(true); // frees server, services and characteristics
  chrMode = chrController = chrStatus = chrInfo = chrSettings = chrDiag = chrPairing = nullptr;
  bleConnected = false;
  bleRunning = false;
  xQueueReset(bleWriteQueue);
  DEBUG("BLE - stopped");
}

// Set Mode/Controller back to the values in effect and notify them (write echo / change from another source).
static void notifySettings(uint8_t mode, uint8_t ctrl)
{
  chrMode->setValue(&mode, 1);
  chrMode->notify();
  chrController->setValue(&ctrl, 1);
  chrController->notify();
}

static void bleTask(void *arg)
{
  vTaskDelay(pdMS_TO_TICKS(1000)); // let setup() finish (WiFi, web server) before the radio is shared

  uint8_t notifiedMode = 0xFF;
  uint8_t notifiedCtrl = 0xFF;
  uint8_t notifiedSettings[DRIVING_SETTINGS_LEN] = {0}; // layout byte 0 never matches: first tick notifies
  TickType_t lastStatus = 0;
  TickType_t lastDiag = 0;

  while (1)
  {
    const bool wanted = bleEnabled && !lowPowerMode;
    if (wanted && !bleRunning)
    {
      bleStart();
      notifiedMode = notifiedCtrl = 0xFF;
      memset(notifiedSettings, 0, sizeof(notifiedSettings));
    }
    else if (!wanted && bleRunning)
    {
      bleStop();
    }

    if (bleRunning)
    {
      if (bleForgetBondsRequest)
      {
        // Back to "first phone pairs without a code", with a new code for later.
        NimBLEDevice::deleteAllBonds();
        blePasskey = 100000 + (esp_random() % 900000); // persisted by writeEEP
        xSemaphoreTake(bleSecMutex, portMAX_DELAY);
        trustedPeers.clear();
        applyPairingMode(false);
        xSemaphoreGive(bleSecMutex);
        bleForgetBondsRequest = false;
        DEBUG("BLE - bonds deleted, new pairing code");
      }

      bool echo = false;
      bool settingsEcho = false;
      BleWrite w;
      while (xQueueReceive(bleWriteQueue, &w, 0) == pdTRUE)
      {
        if (w.target == BLE_WRITE_SETTING)
        {
          // [id][value LE]; wrong length / unknown id / out of range: unchanged, still echoed
          const uint8_t valueLen = drivingSettingValueLen(w.value);
          if (valueLen && w.len == 1 + valueLen)
          {
            applyDrivingSetting(w.value, w.arg);
          }
          settingsEcho = true;
          continue;
        }
        if (w.len == 1)
        {
          if (w.target == BLE_WRITE_MODE)
          {
            requestMode(w.value);
          }
          else if (w.value <= 1)
          {
            setControllerDisabled(w.value == 0);
          }
        }
        echo = true;
      }

      const uint8_t mode = (uint8_t)state.mode;
      const uint8_t ctrl = disableController ? 0 : 1;
      if (echo || mode != notifiedMode || ctrl != notifiedCtrl)
      {
        notifySettings(mode, ctrl);
        notifiedMode = mode;
        notifiedCtrl = ctrl;
      }

      // Settings: echo after a write, and notify changes from the web UI too.
      uint8_t settings[DRIVING_SETTINGS_LEN];
      buildDrivingSettings(settings);
      if (settingsEcho || memcmp(settings, notifiedSettings, sizeof(settings)) != 0)
      {
        chrSettings->setValue(settings, sizeof(settings));
        chrSettings->notify();
        memcpy(notifiedSettings, settings, sizeof(settings));
      }

      const TickType_t now = xTaskGetTickCount();
      if ((now - lastDiag) >= pdMS_TO_TICKS(BLE_DIAG_PERIOD_MS) && !isOTAUpdateInProgress())
      {
        lastDiag = now;
        uint8_t diag[BLE_DIAG_LEN];
        buildDiag(diag);
        chrDiag->setValue(diag, sizeof(diag));
        if (bleConnected)
        {
          chrDiag->notify();
        }
      }

      if ((now - lastStatus) >= pdMS_TO_TICKS(BLE_STATUS_PERIOD_MS))
      {
        lastStatus = now;
        updateInfo();
        if (!isOTAUpdateInProgress()) // keep the radio and CPU for the upload
        {
          uint8_t status[BLE_STATUS_LEN];
          buildStatus(status);
          chrStatus->setValue(status, sizeof(status));
          if (bleConnected)
          {
            chrStatus->notify();
          }
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(BLE_TASK_TICK_MS));
  }
}

void setupBLE()
{
  if (blePasskey < 100000 || blePasskey > 999999)
  {
    blePasskey = 100000 + (esp_random() % 900000); // first boot: random pairing code, persisted by writeEEP
  }
  bleSecMutex = xSemaphoreCreateMutex();
  bleWriteQueue = xQueueCreate(8, sizeof(BleWrite));
  xTaskCreate(bleTask, "bleTask", 4096, NULL, 2, NULL);
}

bool bleCodeIsRequired()
{
  return bleCodeRequired;
}

bool bleIsConnected()
{
  return bleConnected;
}

bool bleForgetBonds()
{
  if (!bleRunning)
  {
    return false;
  }
  bleForgetBondsRequest = true;
  return true;
}
