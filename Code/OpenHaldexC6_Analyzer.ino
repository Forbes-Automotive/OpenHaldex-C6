#include <OpenHaldexC6_defs.h>

// Analyzer mode: pure CAN pass-through plus a TCP interface for external tools.
// - GVRET binary protocol for SavvyCAN
// - SLCAN (Lawicel) text protocol for CANHacker
// All Haldex control logic is bypassed elsewhere when analyzerMode == true.

// ### Analyzer mode (inline CAN bridge + SavvyCAN)
// Adds a lightweight analyzer mode that turns the controller into a passive CAN bridge and exposes a TCP analyzer interface.

// - Analyzer mode disables OpenHaldex control logic (no injected control frames).
// - The bridge remains active, so the Haldex ECU still sits on the chassis bus.
// - Works with SavvyCAN using GVRET over TCP (port 23).
// - Optional Lawicel/SLCAN is supported for tools like CANHacker (limited to a single bus).

// #### Enable analyzer mode
// 1. Open the OpenHaldex web UI.
// 2. Go to **Setup** and enable **Analyzer Mode**.
//    - This also forces **Disable Controller** for clarity.

// #### Connect SavvyCAN (GVRET)
// 1. Keep the OpenHaldex Wi-Fi AP connected.
// 2. In SavvyCAN: **Add New Device Connection** -> **Network Connection (GVRET)**.
// 3. IP address: `192.168.1.1` (port 23 is implicit in SavvyCAN's GVRET UI).
// 4. Set CAN speed to `500000`.

// ### Notes
// - Analyzer mode is intended for inline sniffing; it does not generate any control frames.
// - When analyzer mode is disabled, the controller works normally.

// Suspected bug fixes for analyzer mode, so far:
// - GVRET control replies use a short blocking write (with a small post-connect delay) so SavvyCAN always completes the handshake.
// - Frame streaming is best-effort: we attempt a single TCP write and drop if it can't fully send (avoids stalls and sidesteps availableForWrite()).
// - It is recommended to use SavvyCAN filters as the busy chassis bus will likely overload the TCP connection quickly.

namespace {
static const uint16_t kAnalyzerPort = 23;
static const uint32_t kAnalyzerPollDelayMs = 1;
static const size_t kAnalyzerQueueDepth = 8;

struct AnalyzerFrame {
  twai_message_t frame;
  uint8_t bus;       // 0 = chassis, 1 = haldex
  uint32_t timestamp;
};

static QueueHandle_t analyzerQueue = nullptr;
static WiFiServer analyzerServer(kAnalyzerPort);
static WiFiClient analyzerClient;
static uint8_t analyzerActiveProtocol = ANALYZER_PROTOCOL_GVRET;
static bool analyzerServerStarted = false;
// Allow a little more time after TCP connect so control replies aren't dropped.
static const uint32_t kGvretControlWriteTimeoutMs = 250;

// -------------------------------
// GVRET protocol (SavvyCAN)
// -------------------------------
enum GvretState {
  GVRET_WAIT,
  GVRET_CMD,
  GVRET_PAYLOAD
};

static GvretState gvretState = GVRET_WAIT;
static uint8_t gvretCommand = 0;
static uint8_t gvretPayload[32];
static size_t gvretIndex = 0;
static size_t gvretExpected = 0;
static bool gvretBinaryEnabled = false;
static uint8_t gvretE7Count = 0;

static void resetGvretParser() {
  gvretState = GVRET_WAIT;
  gvretCommand = 0;
  gvretIndex = 0;
  gvretExpected = 0;
  gvretBinaryEnabled = false;
  gvretE7Count = 0;
}

// Control replies are part of the GVRET handshake; deliver them reliably.
static bool gvretWriteBlocking(const uint8_t *data, size_t len, uint32_t timeoutMs) {
  if (!analyzerClient || !analyzerClient.connected()) {
    return false;
  }

  uint32_t start = millis();
  size_t sent = 0;
  while (sent < len) {
    if (!analyzerClient.connected()) {
      return false;
    }
    size_t written = analyzerClient.write(data + sent, len - sent);
    if (written > 0) {
      sent += written;
      continue;
    }
    if (millis() - start > timeoutMs) {
      return false;
    }
    vTaskDelay(1);
  }

  return true;
}

// Frame streaming is best-effort; drop if TCP can't keep up.
static bool gvretWriteNonBlocking(const uint8_t *data, size_t len) {
  if (!analyzerClient || !analyzerClient.connected()) {
    return false;
  }

  // Some ESP32 builds report 0 from availableForWrite() even when a write succeeds.
  // Try a single write and drop if it doesn't fully send to avoid blocking CAN tasks.
  size_t written = analyzerClient.write(data, len);
  return written == len;
}
static void gvretSendNumBuses() {
  const uint8_t payload[] = { 0xF1, 0x0C, 0x02 };
  gvretWriteBlocking(payload, sizeof(payload), kGvretControlWriteTimeoutMs);
}

static void gvretSendBusInfo() {
  // Minimal bus info response (10 bytes payload) for SavvyCAN compatibility.
  // Speeds are expressed in kbit (500 = 0x01F4).
  const uint16_t speedKbit = 500;
  const uint8_t payload[] = {
    0xF1, 0x06,
    0x01, 0x00,  // can0 enabled, not listen-only
    (uint8_t)(speedKbit & 0xFF), (uint8_t)((speedKbit >> 8) & 0xFF),
    0x01, 0x00, 0x00,  // can1 enabled, not listen-only, not singlewire
    (uint8_t)(speedKbit & 0xFF), (uint8_t)((speedKbit >> 8) & 0xFF),
    0x00  // reserved
  };
  gvretWriteBlocking(payload, sizeof(payload), kGvretControlWriteTimeoutMs);
}

static void gvretSendExtendedBusInfo() {
  uint8_t payload[17] = { 0 };
  payload[0] = 0xF1;
  payload[1] = 0x0D;
  gvretWriteBlocking(payload, sizeof(payload), kGvretControlWriteTimeoutMs);
}

static void gvretSendDeviceInfo() {
  // Build/version fields are placeholders; SavvyCAN only needs a reply.
  const uint8_t payload[] = { 0xF1, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
  gvretWriteBlocking(payload, sizeof(payload), kGvretControlWriteTimeoutMs);
}

static void gvretSendValidation() {
  const uint8_t payload[] = { 0xF1, 0x09 };
  gvretWriteBlocking(payload, sizeof(payload), kGvretControlWriteTimeoutMs);
}

static void gvretSendTimeSync() {
  uint32_t now = micros();
  uint8_t payload[6];
  payload[0] = 0xF1;
  payload[1] = 0x01;
  payload[2] = (uint8_t)(now & 0xFF);
  payload[3] = (uint8_t)((now >> 8) & 0xFF);
  payload[4] = (uint8_t)((now >> 16) & 0xFF);
  payload[5] = (uint8_t)((now >> 24) & 0xFF);
  gvretWriteBlocking(payload, sizeof(payload), kGvretControlWriteTimeoutMs);
}

static void gvretSendFrame(const AnalyzerFrame &entry) {
  uint8_t payload[22];
  uint32_t id = entry.frame.identifier & 0x1FFFFFFF;
  if (entry.frame.extd) {
    id |= 0x80000000;
  }

  payload[0] = 0xF1;
  payload[1] = 0x00;
  payload[2] = (uint8_t)(entry.timestamp & 0xFF);
  payload[3] = (uint8_t)((entry.timestamp >> 8) & 0xFF);
  payload[4] = (uint8_t)((entry.timestamp >> 16) & 0xFF);
  payload[5] = (uint8_t)((entry.timestamp >> 24) & 0xFF);
  payload[6] = (uint8_t)(id & 0xFF);
  payload[7] = (uint8_t)((id >> 8) & 0xFF);
  payload[8] = (uint8_t)((id >> 16) & 0xFF);
  payload[9] = (uint8_t)((id >> 24) & 0xFF);
  payload[10] = (uint8_t)(((entry.bus & 0x0F) << 4) | (entry.frame.data_length_code & 0x0F));

  for (uint8_t i = 0; i < entry.frame.data_length_code && i < 8; ++i) {
    payload[11 + i] = entry.frame.data[i];
  }

  gvretWriteNonBlocking(payload, 11 + entry.frame.data_length_code);
}

static void gvretTransmitFrameFromHost() {
  if (gvretIndex < 7) {
    return;
  }

  uint32_t rawId = (uint32_t)gvretPayload[0] |
                   ((uint32_t)gvretPayload[1] << 8) |
                   ((uint32_t)gvretPayload[2] << 16) |
                   ((uint32_t)gvretPayload[3] << 24);

  uint8_t bus = gvretPayload[4];
  uint8_t dlc = gvretPayload[5];

  twai_message_t msg = { 0 };
  msg.extd = (rawId & 0x80000000UL) != 0;
  msg.rtr = 0;
  msg.data_length_code = dlc > 8 ? 8 : dlc;
  msg.identifier = msg.extd ? (rawId & 0x1FFFFFFF) : (rawId & 0x7FF);

  for (uint8_t i = 0; i < msg.data_length_code; ++i) {
    msg.data[i] = gvretPayload[6 + i];
  }

  if (bus == 0) {
    twai_transmit_v2(twai_bus_0, &msg, (5 / portTICK_PERIOD_MS));
  } else if (bus == 1) {
    twai_transmit_v2(twai_bus_1, &msg, (5 / portTICK_PERIOD_MS));
  }
}

static void gvretProcessCommand() {
  switch (gvretCommand) {
    case 0x00:  // CAN frame from host -> device
      gvretTransmitFrameFromHost();
      break;
    case 0x01:  // time sync request
      gvretSendTimeSync();
      break;
    case 0x06:  // bus info request
      gvretSendBusInfo();
      break;
    case 0x07:  // device info request
      gvretSendDeviceInfo();
      break;
    case 0x09:  // validation request
      gvretSendValidation();
      break;
    case 0x0C:  // number of buses
      gvretSendNumBuses();
      break;
    case 0x0D:  // extended bus info
      gvretSendExtendedBusInfo();
      break;
    case 0x05:  // set CAN params (ignored)
    case 0x0E:  // set extended buses (ignored)
    default:
      break;
  }
}

static void gvretHandleByte(uint8_t byteIn) {
  if (!gvretBinaryEnabled) {
    // SavvyCAN sends 0xE7 0xE7 to enter binary GVRET mode.
    if (byteIn == 0xE7) {
      gvretE7Count++;
      if (gvretE7Count >= 2) {
        gvretBinaryEnabled = true;
        gvretE7Count = 0;
      }
    } else {
      gvretE7Count = 0;
    }
    return;
  }

  switch (gvretState) {
    case GVRET_WAIT:
      if (byteIn == 0xF1) {
        gvretState = GVRET_CMD;
      }
      break;
    case GVRET_CMD:
      gvretCommand = byteIn;
      gvretIndex = 0;
      gvretExpected = 0;

      if (gvretCommand == 0x00) {
        gvretExpected = 6;  // id(4) + bus + len, data follows
        gvretState = GVRET_PAYLOAD;
      } else if (gvretCommand == 0x05) {
        gvretExpected = 9;  // fixed payload + terminator
        gvretState = GVRET_PAYLOAD;
      } else if (gvretCommand == 0x0E) {
        gvretExpected = 13;  // fixed payload + terminator
        gvretState = GVRET_PAYLOAD;
      } else {
        gvretProcessCommand();
        gvretState = GVRET_WAIT;
      }
      break;
    case GVRET_PAYLOAD:
      if (gvretIndex < sizeof(gvretPayload)) {
        gvretPayload[gvretIndex++] = byteIn;
      } else {
        gvretState = GVRET_WAIT;
        break;
      }

      if (gvretCommand == 0x00 && gvretIndex == 6) {
        uint8_t dlc = gvretPayload[5];
        gvretExpected = 6 + dlc + 1;  // include trailing 0 byte
      }

      if (gvretIndex >= gvretExpected) {
        gvretProcessCommand();
        gvretState = GVRET_WAIT;
      }
      break;
  }
}

// -------------------------------
// SLCAN protocol (CANHacker)
// -------------------------------
static char slcanLine[64];
static size_t slcanLen = 0;

static void resetSlcanParser() {
  slcanLen = 0;
}

static int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

static bool parseHexByte(const char *text, uint8_t &out) {
  int hi = hexNibble(text[0]);
  int lo = hexNibble(text[1]);
  if (hi < 0 || lo < 0) {
    return false;
  }
  out = (uint8_t)((hi << 4) | lo);
  return true;
}

static uint32_t parseHexId(const char *text, size_t len, bool &ok) {
  ok = true;
  uint32_t value = 0;
  for (size_t i = 0; i < len; ++i) {
    int nibble = hexNibble(text[i]);
    if (nibble < 0) {
      ok = false;
      return 0;
    }
    value = (value << 4) | (uint32_t)nibble;
  }
  return value;
}

static void slcanSendAck() {
  if (analyzerClient && analyzerClient.connected()) {
    analyzerClient.write("\r");
  }
}

static void slcanHandleLine() {
  if (slcanLen == 0) {
    return;
  }

  char cmd = slcanLine[0];
  if (cmd == 'O' || cmd == 'C' || cmd == 'S') {
    slcanSendAck();
    return;
  }
  if (cmd == 'V') {
    if (analyzerClient && analyzerClient.connected()) {
      analyzerClient.print("V0101\r");
    }
    return;
  }
  if (cmd == 'N') {
    if (analyzerClient && analyzerClient.connected()) {
      analyzerClient.print("N0000\r");
    }
    return;
  }

  bool extended = (cmd == 'T' || cmd == 'R');
  bool remote = (cmd == 'r' || cmd == 'R');
  if (cmd != 't' && cmd != 'T' && cmd != 'r' && cmd != 'R') {
    slcanSendAck();
    return;
  }

  size_t idLen = extended ? 8 : 3;
  if (slcanLen < 1 + idLen + 1) {
    slcanSendAck();
    return;
  }

  bool ok = false;
  uint32_t id = parseHexId(&slcanLine[1], idLen, ok);
  if (!ok) {
    slcanSendAck();
    return;
  }

  int dlcNibble = hexNibble(slcanLine[1 + idLen]);
  if (dlcNibble < 0) {
    slcanSendAck();
    return;
  }
  uint8_t dlc = (uint8_t)dlcNibble;
  if (dlc > 8) {
    dlc = 8;
  }

  size_t dataIndex = 1 + idLen + 1;
  if (!remote && slcanLen < dataIndex + (dlc * 2)) {
    slcanSendAck();
    return;
  }

  twai_message_t msg = { 0 };
  msg.extd = extended;
  msg.rtr = remote ? 1 : 0;
  msg.identifier = extended ? (id & 0x1FFFFFFF) : (id & 0x7FF);
  msg.data_length_code = dlc;

  if (!remote) {
    for (uint8_t i = 0; i < dlc; ++i) {
      if (!parseHexByte(&slcanLine[dataIndex + (i * 2)], msg.data[i])) {
        slcanSendAck();
        return;
      }
    }
  }

  twai_transmit_v2(twai_bus_0, &msg, (5 / portTICK_PERIOD_MS));
  slcanSendAck();
}

static void slcanHandleByte(uint8_t byteIn) {
  if (byteIn == '\r') {
    slcanHandleLine();
    resetSlcanParser();
    return;
  }

  if (slcanLen + 1 < sizeof(slcanLine)) {
    slcanLine[slcanLen++] = (char)byteIn;
    slcanLine[slcanLen] = '\0';
  } else {
    resetSlcanParser();
  }
}

static void slcanSendFrame(const AnalyzerFrame &entry) {
  // CANHacker expects a single bus; use chassis (bus 0) frames only.
  if (entry.bus != 0) {
    return;
  }

  char buffer[40];
  char *ptr = buffer;

  if (entry.frame.extd) {
    ptr += sprintf(ptr, "T%08X%1X", entry.frame.identifier, entry.frame.data_length_code & 0xF);
  } else {
    ptr += sprintf(ptr, "t%03X%1X", entry.frame.identifier, entry.frame.data_length_code & 0xF);
  }

  for (uint8_t i = 0; i < entry.frame.data_length_code; ++i) {
    ptr += sprintf(ptr, "%02X", entry.frame.data[i]);
  }

  *ptr++ = '\r';
  *ptr = '\0';

  if (analyzerClient && analyzerClient.connected()) {
    size_t len = (size_t)(ptr - buffer);
    // Best-effort send; attempt once and drop if it doesn't fully send.
    size_t written = analyzerClient.write((const uint8_t *)buffer, len);
    (void)written;
  }
}

// -------------------------------
// Analyzer task + helpers
// -------------------------------
static void resetAnalyzerClientState() {
  resetGvretParser();
  resetSlcanParser();
}

static void analyzerCloseClient() {
  if (analyzerClient) {
    analyzerClient.stop();
  }
  resetAnalyzerClientState();
}

static void analyzerTask(void *arg) {
  // Handles TCP IO and dequeues CAN frames when analyzerMode is active.
  (void)arg;

  while (1) {
    if (!analyzerMode) {
      analyzerCloseClient();
      analyzerServerStarted = false;
      vTaskDelay(kAnalyzerPollDelayMs / portTICK_PERIOD_MS);
      continue;
    }

    if (analyzerActiveProtocol != analyzerProtocol) {
      analyzerActiveProtocol = analyzerProtocol;
      analyzerCloseClient();
    }

    if (WiFi.getMode() == WIFI_OFF) {
      analyzerCloseClient();
      analyzerServerStarted = false;
      vTaskDelay(kAnalyzerPollDelayMs / portTICK_PERIOD_MS);
      continue;
    }

    if (!analyzerQueue) {
      vTaskDelay(kAnalyzerPollDelayMs / portTICK_PERIOD_MS);
      continue;
    }

    if (!analyzerServerStarted) {
      analyzerServer.begin();
      analyzerServer.setNoDelay(true);
      analyzerServerStarted = true;
    }

    if (!analyzerClient || !analyzerClient.connected()) {
      WiFiClient pending = analyzerServer.available();
      if (pending) {
        analyzerClient = pending;
        analyzerClient.setNoDelay(true);
        // Give TCP a moment to finish setup so control replies can flush.
        vTaskDelay(10 / portTICK_PERIOD_MS);
        resetAnalyzerClientState();
      } else {
        vTaskDelay(kAnalyzerPollDelayMs / portTICK_PERIOD_MS);
        continue;
      }
    }

    while (analyzerClient && analyzerClient.connected() && analyzerClient.available()) {
      uint8_t byteIn = (uint8_t)analyzerClient.read();
      if (analyzerProtocol == ANALYZER_PROTOCOL_LAWICEL) {
        slcanHandleByte(byteIn);
      } else {
        gvretHandleByte(byteIn);
      }
    }

    AnalyzerFrame entry;
    while (xQueueReceive(analyzerQueue, &entry, 0) == pdTRUE) {
      if (analyzerProtocol == ANALYZER_PROTOCOL_LAWICEL) {
        slcanSendFrame(entry);
      } else {
        gvretSendFrame(entry);
      }
    }

    vTaskDelay(kAnalyzerPollDelayMs / portTICK_PERIOD_MS);
  }
}
}  // namespace

// Create analyzer queue + task once; the task self-idles when analyzerMode is false.
void setupAnalyzer() {
  if (!analyzerQueue) {
    analyzerQueue = xQueueCreate(kAnalyzerQueueDepth, sizeof(AnalyzerFrame));
  }
  xTaskCreate(analyzerTask, "analyzer", 4096, NULL, 3, NULL);
}

void setAnalyzerMode(bool enable) {
  analyzerMode = enable;
  if (analyzerMode) {
    // Analyzer mode should not transmit control frames; force controller off.
    if (!disableController) {
      disableController = true;
      if (bool_disableControl) {
        ESPUI.updateSwitcher(bool_disableControl, true);
      }
    }
  }
  if (bool_analyzerMode) {
    ESPUI.updateSwitcher(bool_analyzerMode, analyzerMode);
  }
  if (!analyzerMode && analyzerQueue) {
    xQueueReset(analyzerQueue);
  }
}

void analyzerQueueFrame(const twai_message_t &frame, uint8_t bus) {
  if (!analyzerMode || !analyzerQueue) {
    return;
  }

  // Lawicel/SLCAN is single-bus; expose chassis (bus 0) only.
  if (analyzerProtocol == ANALYZER_PROTOCOL_LAWICEL && bus != 0) {
    return;
  }

  AnalyzerFrame entry;
  entry.frame = frame;
  entry.bus = bus;
  entry.timestamp = micros();

  // Best-effort enqueue; drop when busy to avoid blocking CAN tasks.
  xQueueSend(analyzerQueue, &entry, 0);
}