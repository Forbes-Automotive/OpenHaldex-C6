#pragma once
// ---------------------------------------------------------------------------
// CANN - the car's CAN frames relayed over ESP-NOW (wire format v1). SHARED FILE: identical copies live in
//   can2gauge/include/cannow_proto.h  OpenHaldex-C6/include/cannow_proto.h  adc2can/include/cannow_proto.h
// Change all three together and bump CANN_PROTO on any layout change. The sender side is include/cannow_tx.h (shared
// by OpenHaldex and adc2can, identical copies too).
//
// A device on the car's CAN (OpenHaldex, adc2can) broadcasts the frames it receives, so a can2gauge with power only -
// no CAN wire - decodes them exactly as if heard on its own bus: the VAG PQ / MQB / MaxxECU decoders, the CAN monitor,
// the DBC and user signals. Off by default; the sender's web UI setting "Relay CAN frames to gauges" (cannRate_t) also
// caps each id's rate if the full bus is too much.
//   - Broadcast, no pairing: every gauge in range on the sender's WiFi channel hears it (both APs default to channel 1).
//   - One-way: a gauge never sends anything back this way (no OBD requests, no gearbox diagnostics over the air).
//   - Batched: frames are packed back to back; a packet goes out when full, or CANN_FLUSH_MS after its first frame.
//   - While relaying, the sender sends all its ESP-NOW packets at 6 Mbit/s (OFDM) instead of 1 Mbit/s: a full 500 kbit/s
//     bus is ~100 packets a second, ~6 % of the air at 6 Mbit/s against ~30 % at 1 Mbit/s. Range in a car is unaffected.
//   - The frames the sender itself transmits are not in it (a CAN controller does not receive its own frames): the gauge
//     gets those from the sender's own packets (OpenHaldex: ohx_proto.h, adc2can: adcnow_proto.h).
//   - A gauge that hears an id on its own wire ignores the relayed copy of that id; with two senders on the same bus, the
//     first one heard keeps each id until it goes quiet.
// ---------------------------------------------------------------------------
#include <stdint.h>

#define CANN_MAGIC0 'C'
#define CANN_MAGIC1 'N'
#define CANN_PROTO 1
#define CANN_BYTES 250   // packet size limit (ESP-NOW v1)
#define CANN_FLUSH_MS 10 // a part-filled packet goes out this long after its first frame

enum cannKind_t : uint8_t // who is sending
{
  CANN_K_OPENHALDEX = 1,
  CANN_K_ADC2CAN = 2,
};

enum cannRate_t : uint8_t // the sender's setting: which frames go out
{
  CANN_R_OFF = 0,
  CANN_R_ALL = 1, // every frame received
  CANN_R_50 = 2,  // each id at most 50 times a second (frames in between are skipped, the next one goes)
  CANN_R_20 = 3,
  CANN_R_10 = 4,
  CANN_R_MAX
};

// After the header, `count` frames packed back to back, each:
//   uint16_t tag  bits 0-10 = the 11-bit id, bits 11-14 = data length 0-8, bit 15 = 29-bit id: bits 0-10 are then zero
//                 and a uint32_t id follows
//   the data bytes (as many as the length)
// An 8-byte 11-bit frame takes 10 bytes, so 24 fit one packet.
#define CANN_TAG_EXT 0x8000
#define CANN_TAG_ID 0x07FF
#define CANN_TAG_LEN_SHIFT 11

struct __attribute__((packed)) CannHdr
{
  uint8_t magic[2]; // CANN_MAGIC0, CANN_MAGIC1
  uint8_t proto;    // CANN_PROTO
  uint8_t kind;     // cannKind_t
  uint8_t seq;      // +1 per packet: a gap = packets lost in the air
  uint8_t bus;      // the sender's bus: 0 = the car's CAN
  uint8_t count;    // frames in this packet
  uint8_t lost;     // frames the sender could not relay since its previous packet (queue full / send refused), max 255
};

static_assert(sizeof(CannHdr) == 8, "CANN header layout");
