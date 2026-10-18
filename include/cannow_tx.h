#pragma once
// ---------------------------------------------------------------------------
// CANN sender - relays the car's CAN frames to gauges over ESP-NOW (wire format include/cannow_proto.h). SHARED FILE:
// identical copies live in OpenHaldex-C6/include/cannow_tx.h and adc2can/include/cannow_tx.h, so both relay the same
// way and show the same setting. Include it in one .cpp only (it holds the state).
//
//   cannBegin(kind)    once at boot (cannKind_t): creates the queue
//   cannLink(up)       true straight after ESP-NOW's broadcast peer is added, false before ESP-NOW is shut down
//   cannRate           the setting (cannRate_t), persisted by the device; cannSetRate() to change it
//   cannTap(m)         every frame received on the car's CAN, from the CAN receive task: cheap, never blocks
//   cannPump(send)     every few ms (well under CANN_FLUSH_MS) from the task that owns ESP-NOW, while it is up: packs
//                      the queued frames, sends them through send(), and sets the PHY rate (6 Mbit/s while relaying)
//   cannStat           frames relayed / lost, per second - for the web UI
// ---------------------------------------------------------------------------
#include <Arduino.h>
#include <esp_now.h>
#include "driver/twai.h"
#include <cannow_proto.h>

#define CANN_QUEUE 192 // frames: ~80 ms of a full 500 kbit/s bus, for a pump held up by the web server

typedef bool (*cannSend_t)(const uint8_t *data, size_t len); // broadcast one packet, false = refused

struct CannStat
{
  uint32_t frames, lost, packets; // since boot
  uint32_t framesPerSec, lostPerSec;
};

static volatile uint8_t cannRate = CANN_R_OFF;
static CannStat cannStat = {};

struct CannQFrame
{
  uint32_t id;
  uint8_t len, ext;
  uint8_t d[8];
};
static QueueHandle_t cannQ = nullptr;
static uint8_t cannKind = 0;
static volatile bool cannUp = false;      // ESP-NOW is up: frames are queued
static volatile uint32_t cannTapLost = 0; // queue full (written by the receive task only)
static uint32_t cannSendLost = 0;         // in packets ESP-NOW refused (pump task only)
static uint16_t cannLastMs[2048];         // per 11-bit id: millis() (low 16 bits) of the last frame let through
static uint8_t cannBuf[CANN_BYTES];
static size_t cannLen = sizeof(CannHdr);
static uint8_t cannCount = 0, cannSeq = 0;
static bool cannFast = false; // the broadcast peer is set to 6 Mbit/s
static uint32_t cannFirstMs = 0, cannLostSent = 0, cannSecMs = 0, cannSecFrames = 0, cannSecLost = 0;

static inline void cannBegin(uint8_t kind)
{
  cannKind = kind;
  if (!cannQ)
    cannQ = xQueueCreate(CANN_QUEUE, sizeof(CannQFrame));
}

static inline void cannLink(bool up)
{
  cannUp = up;
  cannFast = false; // a freshly added peer sends at the default 1 Mbit/s
  if (!up && cannQ)
    xQueueReset(cannQ);
}

static inline void cannSetRate(uint8_t r) { cannRate = r < CANN_R_MAX ? r : CANN_R_OFF; }

static inline void cannTap(const twai_message_t &m)
{
  const uint8_t r = cannRate;
  if (r == CANN_R_OFF || r >= CANN_R_MAX || !cannUp || !cannQ || m.rtr)
    return;
  if (r != CANN_R_ALL && !m.extd) // 29-bit ids are rare on a car's bus: never capped
  {
    static const uint8_t gapMs[CANN_R_MAX] = {0, 0, 18, 45, 95}; // a little under 20 / 50 / 100 ms, so frame jitter
                                                                 // does not halve the rate
    const uint16_t now = (uint16_t)millis();
    uint16_t &last = cannLastMs[m.identifier & CANN_TAG_ID];
    if ((uint16_t)(now - last) < gapMs[r])
      return;
    last = now;
  }
  CannQFrame f;
  f.id = m.identifier;
  f.ext = m.extd ? 1 : 0;
  f.len = m.data_length_code > 8 ? 8 : m.data_length_code;
  memcpy(f.d, m.data, 8);
  if (xQueueSend(cannQ, &f, 0) != pdTRUE)
    cannTapLost = cannTapLost + 1; // only this task writes it
}

static inline void cannFlush(cannSend_t send)
{
  if (!cannCount)
    return;
  CannHdr *h = (CannHdr *)cannBuf;
  h->magic[0] = CANN_MAGIC0;
  h->magic[1] = CANN_MAGIC1;
  h->proto = CANN_PROTO;
  h->kind = cannKind;
  h->seq = cannSeq++;
  h->bus = 0;
  h->count = cannCount;
  const uint32_t pending = cannTapLost + cannSendLost - cannLostSent;
  h->lost = pending > 255 ? 255 : (uint8_t)pending;
  cannLostSent += h->lost;
  if (send(cannBuf, cannLen))
  {
    cannStat.packets++;
    cannStat.frames += cannCount;
    cannSecFrames += cannCount;
  }
  else
    cannSendLost += cannCount;
  cannLen = sizeof(CannHdr);
  cannCount = 0;
}

static inline void cannPump(cannSend_t send)
{
  const uint32_t now = millis();
  const bool fast = cannRate != CANN_R_OFF;
  if (fast != cannFast) // relaying: 6 Mbit/s OFDM for every packet on the broadcast peer, back to 1 Mbit/s when off
  {
    static const uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_rate_config_t rc = {};
    rc.phymode = fast ? WIFI_PHY_MODE_11G : WIFI_PHY_MODE_11B;
    rc.rate = fast ? WIFI_PHY_RATE_6M : WIFI_PHY_RATE_1M_L;
    if (esp_now_set_peer_rate_config(bcast, &rc) == ESP_OK)
      cannFast = fast;
  }
  CannQFrame f;
  while (cannQ && xQueueReceive(cannQ, &f, 0) == pdTRUE)
  {
    const size_t need = 2 + (f.ext ? 4 : 0) + f.len;
    if (cannLen + need > CANN_BYTES)
      cannFlush(send);
    if (!cannCount)
      cannFirstMs = now;
    const uint16_t tag = (f.ext ? CANN_TAG_EXT : (uint16_t)(f.id & CANN_TAG_ID)) | (uint16_t)(f.len << CANN_TAG_LEN_SHIFT);
    memcpy(cannBuf + cannLen, &tag, 2);
    cannLen += 2;
    if (f.ext)
    {
      const uint32_t id = f.id & 0x1FFFFFFF;
      memcpy(cannBuf + cannLen, &id, 4);
      cannLen += 4;
    }
    memcpy(cannBuf + cannLen, f.d, f.len);
    cannLen += f.len;
    cannCount++;
  }
  if (cannCount && (!fast || now - cannFirstMs >= CANN_FLUSH_MS))
    cannFlush(send);
  if (now - cannSecMs >= 1000)
  {
    const uint32_t lost = cannTapLost + cannSendLost;
    cannStat.framesPerSec = cannSecFrames;
    cannStat.lostPerSec = lost - cannSecLost;
    cannStat.lost = lost;
    cannSecFrames = 0;
    cannSecLost = lost;
    cannSecMs = now;
  }
}
