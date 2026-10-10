#include <OpenHaldexC6_ESPNow.h>
#include <OpenHaldexC6_Calculations.h> // learn / long learn state
#include <ohx_proto.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_random.h>
#include "mbedtls/md.h"

bool espNowEnabled = true;
bool espNowControl = true;

static SemaphoreHandle_t nowMutex = nullptr;
static QueueHandle_t cmdQ = nullptr;
static volatile bool nowUp = false;
static volatile bool statusNow = false; // send a status straight away (command answered)
static uint32_t nonce = 0;
static uint8_t statusSeq = 0;
static uint8_t ackSrc[2] = {0, 0}, ackSeq = 0, ackResult = OHX_R_NONE;
static uint8_t lastOkSrc[2] = {0, 0}, lastOkSeq = 0; // a retry of the command just accepted is ignored, not "stale"

struct RxCmd
{
  uint8_t mac[6];
  OhxCommand c;
};

static const uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ---------------------------------------------------------------------------
// ESP-NOW up / down (follows the WiFi)
// ---------------------------------------------------------------------------
// WiFi task - keep it short: copy the command, the ESP-NOW task checks and runs it
static void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
  if (!info || len != (int)sizeof(OhxCommand) || data[0] != OHX_MAGIC0 || data[1] != OHX_MAGIC1 ||
      data[2] != OHX_T_COMMAND || data[3] != OHX_PROTO)
    return;
  RxCmd r;
  memcpy(r.mac, info->src_addr, 6);
  memcpy(&r.c, data, sizeof(OhxCommand));
  xQueueSend(cmdQ, &r, 0);
}

static void nowStart()
{
  xSemaphoreTake(nowMutex, portMAX_DELAY);
  if (!nowUp && esp_now_init() == ESP_OK)
  {
    esp_now_register_recv_cb(onRecv);
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, bcast, 6);
    peer.channel = 0; // the AP's current channel
    peer.ifidx = WIFI_IF_AP;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) == ESP_OK)
      nowUp = true;
    else
      esp_now_deinit();
  }
  xSemaphoreGive(nowMutex);
  DEBUG("ESP-NOW (gauges): %s", nowUp ? "up" : "FAILED");
}

void espNowStop()
{
  if (!nowMutex)
    return;
  xSemaphoreTake(nowMutex, portMAX_DELAY);
  if (nowUp)
  {
    nowUp = false;
    esp_now_unregister_recv_cb();
    esp_now_deinit();
  }
  xSemaphoreGive(nowMutex);
}

// ---------------------------------------------------------------------------
// status
// ---------------------------------------------------------------------------
static int16_t tenths(float c) { return (int16_t)lroundf(c * 10.0f); }

static void sendStatus()
{
  OhxStatus s;
  memset(&s, 0, sizeof(s));
  s.h.magic[0] = OHX_MAGIC0;
  s.h.magic[1] = OHX_MAGIC1;
  s.h.type = OHX_T_STATUS;
  s.h.proto = OHX_PROTO;
  s.nonce = nonce;
  s.seq = statusSeq++;
  unsigned ma = 0, mi = 0, pa = 0;
  sscanf(FW_VERSION, "%u.%u.%u", &ma, &mi, &pa);
  s.fw[0] = ma;
  s.fw[1] = mi;
  s.fw[2] = pa;
  s.generation = haldexGeneration;
  s.mode = (uint8_t)state.mode;

  const bool chassisOk = hasCANChassis, haldexOk = hasCANHaldex;
  s.flags[0] = (isStandalone ? OHX_F0_STANDALONE : 0) | (chassisOk ? OHX_F0_CHASSIS_CAN : 0) |
               (haldexOk ? OHX_F0_HALDEX_CAN : 0) | (disableController ? OHX_F0_CTRL_OFF : 0) |
               (isBusFailure ? OHX_F0_BUS_FAIL : 0) | (state.mode_override ? OHX_F0_OVERRIDE : 0) |
               (broadcastOpenHaldexOverCAN ? OHX_F0_CAN_BCAST : 0) | (haldexLearnTableValid ? OHX_F0_LEARNED : 0);
  s.flags[1] = (tcForceModeFlag ? OHX_F1_TC_FORCE : 0) | (hazardForceModeFlag ? OHX_F1_HAZARD : 0) |
               (extButtonForceModeFlag ? OHX_F1_EXT_BTN : 0) | (brakeActive ? OHX_F1_BRAKE : 0) |
               (handbrakeActive ? OHX_F1_HANDBRAKE : 0) | (haldexOk && received_temp_protection ? OHX_F1_TEMP_PROT : 0) |
               (haldexOk && received_coupling_open ? OHX_F1_COUPLING_OPEN : 0) |
               (haldexOk && received_speed_limit ? OHX_F1_SPEED_LIMIT : 0);
  s.flags[2] = (haldexLearnActive ? OHX_F2_LEARN : 0) | (longLearnActive ? OHX_F2_LONG_LEARN : 0) |
               (espNowControl ? OHX_F2_CONTROL : 0) | (wifiPassword[0] ? OHX_F2_KEY : 0) |
               (fixHunting ? OHX_F2_FIX_HUNTING : 0) | (steering_scale_is_active() ? OHX_F2_STEER_SCALE : 0) |
               (analyzerMode ? OHX_F2_ANALYZER : 0) | (liveDiagEnabled ? OHX_F2_LIVE_DIAG : 0);
  s.flags[3] = (chassisOk && asrForceModeFlag) ? OHX_F3_ASR_OFF : 0;

  s.lockTarget = (uint8_t)constrain((int)lock_target, 0, 100);
  s.lockActual = haldexOk ? received_haldex_engagement : OHX_NONE8;
  s.engagementRaw = received_haldex_engagement_raw;
  s.haldexState = received_haldex_state;
  s.speed = chassisOk ? received_vehicle_speed : OHX_NONE16;
  s.rpm = chassisOk ? received_vehicle_rpm : OHX_NONE16;
  s.boost = chassisOk ? received_vehicle_boost : OHX_NONE16;
  s.throttle = chassisOk ? (uint8_t)constrain((int)received_pedal_value, 0, 100) : OHX_NONE8;
  s.disableThrottle = disableThrottle;
  s.disengageUnderSpeed = disengageUnderSpeed;
  s.disengageAboveSpeed = disengageAboveSpeed;
  const uint32_t now = millis();
  const bool steerFresh = chassisOk && received_steering_ms && now - received_steering_ms <= steeringStaleMs;
  s.steering = steerFresh ? (uint16_t)(fabsf(received_steering_angle) + 0.5f) : OHX_NONE16;
  const bool slipFresh = lastCornerSlipMs && now - lastCornerSlipMs < 500;
  for (int i = 0; i < 4; i++)
    s.slip[i] = slipFresh ? cornerSlip[i] : -128;
  s.learnStep = haldexLearnStep;
  s.longPhase = longLearnPhase;
  s.longSweep = longLearnSweepIdx;
  s.longTotal = longLearnSweepTotal;

  // live diagnostics: Gen5 family over UDS, Gen4 (0AY / PQ) over KWP2000 / TP2.0
  s.clutchTemp = s.oilTemp = s.moduleTemp = OHX_NONE_T;
  s.supplyMv = OHX_NONE16;
  const uint8_t g = haldexGeneration;
  if (haldexOk && liveDiagEnabled && (g == 50 || g == 51 || g == 52))
  {
    s.clutchTemp = tenths(udsClutchTemp);
    s.moduleTemp = tenths(udsModuleTemp);
    s.supplyMv = (uint16_t)lroundf(udsTerminalVoltage * 1000.0f);
  }
  else if (haldexOk && liveDiagEnabled && g == 4 && kwpTp20Connected)
  {
    s.clutchTemp = tenths(kwpPlateTemp);
    s.oilTemp = tenths(kwpOilTemp);
    s.supplyMv = (uint16_t)lroundf(kwpSupplyVoltage * 1000.0f);
  }

  s.ackSrc[0] = ackSrc[0];
  s.ackSrc[1] = ackSrc[1];
  s.ackSeq = ackSeq;
  s.ackResult = ackResult;
  uint8_t ch = 0;
  wifi_second_chan_t ch2;
  if (esp_wifi_get_channel(&ch, &ch2) == ESP_OK)
    s.channel = ch;
  strncpy(s.name, wifiSsid, sizeof(s.name) - 1);

  if (xSemaphoreTake(nowMutex, pdMS_TO_TICKS(5)) == pdTRUE)
  {
    if (nowUp)
      esp_now_send(bcast, (const uint8_t *)&s, sizeof(s));
    xSemaphoreGive(nowMutex);
  }
}

// ---------------------------------------------------------------------------
// commands
// ---------------------------------------------------------------------------
static bool hmacOk(const OhxCommand &c)
{
  uint8_t full[32];
  const mbedtls_md_info_t *md = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  mbedtls_md_hmac(md, (const uint8_t *)wifiPassword, strlen(wifiPassword), (const uint8_t *)&c,
                  offsetof(OhxCommand, hmac), full);
  uint8_t diff = 0;
  for (int i = 0; i < 8; i++)
    diff |= full[i] ^ c.hmac[i];
  return diff == 0;
}

// generation, learn: only with the car stopped (or on the bench, with no chassis CAN at all)
static bool stopped() { return !hasCANChassis || received_vehicle_speed == 0; }

static uint8_t execute(const OhxCommand &c)
{
  switch (c.cmd)
  {
  case OHX_C_MODE: // as POST /api/mode
    if (c.arg[0] >= (uint8_t)openhaldex_mode_t_MAX)
      return OHX_R_INVALID;
    if (disableController)
      return OHX_R_CTRL_OFF;
    if (isStandalone && c.arg[0] == MODE_STOCK)
      state.mode = (openhaldex_mode_t)lastMode;
    else
      state.mode = (openhaldex_mode_t)c.arg[0];
    lastMode = state.mode;
    return OHX_R_OK;
  case OHX_C_GENERATION: // as POST /api/settings {haldexGeneration}
  {
    const uint8_t g = c.arg[0];
    if (!(g == 1 || g == 2 || g == 4 || g == 50 || g == 51 || g == 52 || g == 41))
      return OHX_R_INVALID;
    if (!stopped())
      return OHX_R_MOVING;
    if (haldexLearnActive || longLearnActive)
      return OHX_R_BUSY;
    haldexGeneration = g;
    udsApplyDefaultIds();
    return OHX_R_OK;
  }
  case OHX_C_LEARN_START: // as POST /api/learn/start
    if (!stopped())
      return OHX_R_MOVING;
    if (!hasCANHaldex)
      return OHX_R_NO_HALDEX;
    if (haldexLearnActive || longLearnActive)
      return OHX_R_BUSY;
    startHaldexLearn();
    return OHX_R_OK;
  case OHX_C_LEARN_CANCEL:
    haldexLearnCancel = true;
    return OHX_R_OK;
  case OHX_C_LEARN_CLEAR: // as POST /api/learn/clear
    if (!stopped())
      return OHX_R_MOVING;
    if (haldexLearnActive || longLearnActive)
      return OHX_R_BUSY;
    haldexLearnTableValid = false;
    memset(haldexLearnTable, 0, sizeof(haldexLearnTable));
    return OHX_R_OK;
  case OHX_C_LONG_START: // as POST /api/longlearn/start
    if (!stopped())
      return OHX_R_MOVING;
    if (!hasCANHaldex)
      return OHX_R_NO_HALDEX;
    if (frameEditGenIdx(haldexGeneration) < 0)
      return OHX_R_UNSUPPORTED;
    return startLongLearn(c.arg[0] != 0) ? OHX_R_OK : OHX_R_BUSY;
  case OHX_C_LONG_CANCEL:
    longLearnCancel = true;
    haldexLearnCancel = true;
    return OHX_R_OK;
  case OHX_C_CONTROLLER: // as POST /api/settings {disableController}
    disableController = c.arg[0] == 0;
    if (disableController)
    {
      state.mode = MODE_STOCK;
      lastMode = 0;
    }
    return OHX_R_OK;
  default:
    return OHX_R_INVALID;
  }
}

static void handle(const RxCmd &r)
{
  const OhxCommand &c = r.c;
  uint8_t result;
  if (!hmacOk(c))
    result = OHX_R_AUTH;
  else if (c.nonce != nonce)
  {
    if (r.mac[4] == lastOkSrc[0] && r.mac[5] == lastOkSrc[1] && c.seq == lastOkSeq)
      return; // a retry of the command just carried out - its OK stands
    result = OHX_R_STALE;
  }
  else if (!espNowControl || analyzerMode)
    result = OHX_R_DISABLED;
  else
  {
    result = execute(c);
    nonce = esp_random() | 1; // used up: nothing signed with the old one is accepted again
    if (result == OHX_R_OK)
    {
      lastOkSrc[0] = r.mac[4];
      lastOkSrc[1] = r.mac[5];
      lastOkSeq = c.seq;
    }
    DEBUG("ESP-NOW: command %u from %02X:%02X -> %u", c.cmd, r.mac[4], r.mac[5], result);
  }
  ackSrc[0] = r.mac[4];
  ackSrc[1] = r.mac[5];
  ackSeq = c.seq;
  ackResult = result;
  statusNow = true;
}

// ---------------------------------------------------------------------------
// task
// ---------------------------------------------------------------------------
static void espNowTask(void *)
{
  uint32_t lastStatus = 0, lastCheck = 0;
  for (;;)
  {
    const uint32_t now = millis();
    if (now - lastCheck >= 500) // follow the WiFi: AP up -> ESP-NOW up
    {
      lastCheck = now;
      const bool wifiOn = WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA;
      if (espNowEnabled && wifiOn && !nowUp)
        nowStart();
      else if ((!espNowEnabled || !wifiOn) && nowUp)
        espNowStop();
    }
    RxCmd r;
    while (xQueueReceive(cmdQ, &r, 0) == pdTRUE)
      handle(r);
    if (nowUp && (statusNow || now - lastStatus >= OHX_STATUS_MS))
    {
      statusNow = false;
      lastStatus = now;
      sendStatus();
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setupESPNow()
{
  nowMutex = xSemaphoreCreateMutex();
  cmdQ = xQueueCreate(4, sizeof(RxCmd));
  nonce = esp_random() | 1;
  xTaskCreate(espNowTask, "espNow", 4096, nullptr, 2, nullptr);
}
