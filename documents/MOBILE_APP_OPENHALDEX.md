# OpenHaldex-C6 BLE contract (DashCAN mobile app)

Canonical copy: this file in the OpenHaldex-C6 repo. The DashCAN app keeps a mirror in its `docs/`. Change this file
first and copy it over.

Contract version: `protoVersion` 1, Status layout 1. Implementation: `src/OpenHaldexC6_BLE.cpp`.

## Overview

The OpenHaldex-C6 is a BLE peripheral that the DashCAN app connects to **in addition to** a DashCAN unit. The
OpenHaldex has its own UUIDs and name, and does not use the DashCAN text protocol (`GFW`, `CAN<ids>.`, ...).

The contract is GATT-native: every setting is a characteristic (read / write / notify), and live data is a single
packed binary Status characteristic. There are no text commands and no message reassembly.

Scope:
- change the mode;
- switch the controller on/off;
- read the live data shown on the web UI dashboard.

Everything else (Expert map, learn, force modes, generation, WiFi, OTA) stays on the web UI. There is no firmware
update over BLE.

## Advertising

| Part | Content |
|---|---|
| Advertising packet | flags (LE General Discoverable, BR/EDR not supported) + complete list of 128-bit services: the OpenHaldex service UUID |
| Scan response | complete local name `OpenHaldex-XXXX` (`XXXX` = last 2 bytes of the BT MAC, upper-case hex) |

- **Identify the device by the service UUID.** The name never contains `DashCAN`, so the DashCAN auto-connect ignores
  it.
- **Up to two links.** The device keeps advertising while a slot is free. A phone whose app was closed or killed can
  leave its old link open at the OS level; a new link from the same phone (identity address) drops the older one, so a
  restarted app can always reconnect without rebooting the OpenHaldex.
- **BLE is off when:**
  - the user disables it (web UI → Settings → Bluetooth);
  - the controller is in low-power mode (car parked, WiFi AP off). It comes back when CAN traffic returns. A
    connected phone does not keep the controller awake.

## GATT

Service `6254A001-C7B7-494F-A2FB-76FB74A7DDF0`:

| Characteristic | UUID | Properties | Value |
|---|---|---|---|
| Mode | `6254A002-C7B7-494F-A2FB-76FB74A7DDF0` | read, write (with response), notify | `u8` mode |
| Controller | `6254A003-C7B7-494F-A2FB-76FB74A7DDF0` | read, write (with response), notify | `u8` 1 = enabled, 0 = disabled |
| Status | `6254A004-C7B7-494F-A2FB-76FB74A7DDF0` | read, notify | 17 bytes, see [Status](#status) |
| Info | `6254A005-C7B7-494F-A2FB-76FB74A7DDF0` | read | `u8 protoVersion`, `u8 haldexGeneration`, `u8 isStandalone` |
| Settings | `6254A006-C7B7-494F-A2FB-76FB74A7DDF0` | read, write (with response), notify | 14 bytes, see [Settings](#settings); write `[u8 id][value]` |
| Diag | `6254A007-C7B7-494F-A2FB-76FB74A7DDF0` | read, notify | 20 bytes, 1 Hz, see [Diag](#diag) |

**GATT cache:** phones cache the attribute table of bonded devices. The firmware sends a Service Changed indication
every time BLE starts, so after a firmware update that adds characteristics a bonded phone re-discovers on its next
connection. If a phone still reads wrong values (e.g. it was bonded to a pre-release build without that indication),
forget the device in the phone's Bluetooth settings once. On Android the app can also call `refreshCache` before
discovery.

Mode values: `0` Stock, `1` FWD, `2` 50:50, `3` 60:40, `4` 75:25, `5` Expert.

Info `haldexGeneration` values:

| Value | Platform |
|---|---|
| `1`, `2`, `4` | VW PQ |
| `41` | GM/SAAB Gen4 |
| `42` | Ford |
| `50` | Gen5 MQB 0CQ |
| `51` | Gen5 PQ 0AY |
| `52` | Gen5 MQB 0CQ VAQ |

Device Information Service `0x180A` (read):

| Characteristic | Value |
|---|---|
| `0x2A29` Manufacturer Name | `Forbes Automotive` |
| `0x2A24` Model Number | `OpenHaldex-C6` |
| `0x2A26` Firmware Revision | firmware version, e.g. `9.00.0` |

## Security

- **Pairing: "Just Works".** LE Secure Connections with bonding, no MITM protection, IO capability
  `NoInputNoOutput`. There is **no passkey**: the phone at most asks the user to confirm the pairing.
  - Trade-off, chosen for an easy first connection: anyone in range can pair and change the mode.
- **Writes need an encrypted (bonded) link.** The first write to Mode or Controller from an unpaired phone fails with
  an insufficient-encryption/authentication error, which makes iOS and Android pair (with a confirm dialog at most).
  The app should retry the write after pairing succeeds; on Android it can call `createBond` first.
- **Reads and notifications are open** (Status, Info, Mode, Controller, DIS), so live data works without pairing.
- **Forget Paired Phones** in the web UI deletes all bonds; phones must pair again (the phone should also forget
  the device in its Bluetooth settings).

## Writes: request, then echo

A write to Mode or Controller is a **request**. After processing it the device sets the characteristic to the value
actually in effect and **notifies** it (Mode and Controller are both notified). The app shows the notified value, not
the value it wrote.

| Write | Result |
|---|---|
| Mode `n`, 0 ≤ n ≤ 5, controller enabled | mode = `n` |
| Mode `0` (Stock) while standalone | the mode stays at the last driving mode (Stock is passthrough-only) |
| Mode while the controller is disabled | rejected, mode stays Stock |
| Mode > 5 | rejected, unchanged |
| Controller `0` | controller disabled, mode forced to Stock |
| Controller `1` | controller enabled (also leaves SavvyCAN analyzer mode, as the web UI does) |
| Controller > 1, or any write whose length ≠ 1 byte | rejected, unchanged |

- **Timing:** writes are applied within about 50 ms. The echo notify arrives right after.
- **No error codes:** the write response is always success once the link is paired. A rejection is visible only
  because the echoed value differs from the value written.
- **Changes from other sources are notified too:** the web UI, the physical buttons, CAN `0x6A0` and force modes all
  produce a Mode/Controller notify within about 50 ms.

## Status

Notified at 10 Hz while the phone has notifications enabled on Status. It can also be read at any time.
Little-endian:

| Offset | Type | Field | Unknown |
|---|---|---|---|
| 0 | u8 | layout version (= 1) | - |
| 1 | u8 | sequence counter (wraps; lets the app detect dropped notifies) | - |
| 2 | u8 | mode (0..5) | - |
| 3 | u8 | controller enabled (0/1) | - |
| 4 | u8 | lock target % (0..100) | - |
| 5 | u8 | lock actual %: Haldex engagement as reported by the Haldex | `0xFF` |
| 6 | u16 | vehicle speed, km/h | `0xFFFF` |
| 8 | u8 | throttle / pedal %, 0..100 | `0xFF` |
| 9 | u16 | engine rpm | `0xFFFF` |
| 11 | u16 | boost, the raw value the web UI shows | `0xFFFF` |
| 13 | i16 | steering-wheel angle, 0.1° units, signed | `0x8000` (INT16_MIN) |
| 15 | u16 | flags, see below | - |

**When values are unknown:**
- Speed, throttle, rpm and boost are unknown while the chassis CAN is not healthy.
- Lock actual is unknown while the Haldex CAN is not healthy.
- Real values never use a sentinel: the device clamps them to `0xFE` / `0xFFFE`.

**Steering angle:**
- The value is negative when the VW sign bit is set (PQ `LW_1` 0x0C2 `LW1_LRW_Sign`, MQB `LWI_01` 0x086
  `LWI_VZ_Lenkradwinkel`).
  - **Direction (left/right) of the sign is not yet verified on a car.**
- It is unknown when:
  - the frame is older than 500 ms or missing;
  - the chassis CAN is unhealthy;
  - the platform is GM/Ford (generation 41/42).
- The web UI shows only the magnitude in whole degrees.

**Flags:**

| Bit | Meaning |
|---|---|
| 0 | chassis CAN healthy |
| 1 | Haldex CAN healthy |
| 2 | CAN bus failure |
| 3 | traction control switched off by the driver (TC force-mode trigger); chassis CAN only |
| 4 | ASR switched off; chassis CAN only |
| 5 | hazard lights on (hazard force mode); chassis CAN only |
| 6 | external button force active |
| 7 | brake input active |
| 8 | handbrake input active |
| 9 | Haldex temperature protection; Haldex CAN only |
| 10 | Haldex coupling open; Haldex CAN only |
| 11 | Haldex speed limit; Haldex CAN only |
| 12 | controller in standalone mode |
| 13 | external diagnostic tool detected (live diagnostics paused) |
| 14-15 | reserved, 0 |

**Compatibility rules for the app:**
- Drop a payload shorter than 17 bytes.
- Ignore bytes beyond the ones you know: new fields are only appended.
- A different layout version byte means a breaking change. Show "update the app" rather than decoding.

## Settings

Driving settings that are also on the web UI. An app should check that the characteristic is present after
`retrieveServices` before using it (optional characteristics do not change Info `protoVersion`). Writes need a bonded
link, like Mode.

**Read / notify payload**, little-endian, 14 bytes:

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | layout version (= 1) |
| 1 | u8 | bool flags, below |
| 2 | u8 | TC-off force mode value (0..5) |
| 3 | u8 | hazard force mode value (0..5) |
| 4 | u8 | external-button force mode value (0..5) |
| 5 | u16 | no lock below this speed, km/h (0 = off) |
| 7 | u16 | no lock above this speed, km/h (0 = off) |
| 9 | u8 | no lock below this throttle, % |
| 10 | u16 | lock release rate, %/s |
| 12 | u8 | LED brightness, 0..255 |
| 13 | u8 | reserved flags (0) |

Flags at offset 1: bit 0 TC-off force mode on, 1 hazard force mode on, 2 external-button force mode on,
3 release lock on brake, 4 release lock on handbrake, 5 steering-angle lock scaling, 6 rate-limited lock release,
7 live Haldex diagnostics.

**Write:** one setting per write, `[u8 id][value]`, value little-endian, 1 byte for bool / u8 settings, 2 bytes for
u16 settings:

| Id | Setting | Value |
|---|---|---|
| `0x01` | TC-off force mode on | u8 0/1 |
| `0x02` | TC-off force mode value | u8 0..5 |
| `0x03` | hazard force mode on | u8 0/1 |
| `0x04` | hazard force mode value | u8 0..5 |
| `0x05` | external-button force mode on | u8 0/1 |
| `0x06` | external-button force mode value | u8 0..5 |
| `0x07` | release lock on brake | u8 0/1 |
| `0x08` | release lock on handbrake | u8 0/1 |
| `0x09` | no lock below speed | u16 km/h, clamped 0..300 |
| `0x0A` | no lock above speed | u16 km/h, clamped 0..300 |
| `0x0B` | no lock below throttle | u8 %, clamped 0..100 |
| `0x0C` | steering-angle lock scaling | u8 0/1 |
| `0x0D` | rate-limited lock release | u8 0/1 |
| `0x0E` | lock release rate | u16 %/s, clamped 5..500 |
| `0x0F` | live Haldex diagnostics | u8 0/1 |
| `0x10` | LED brightness | u8 |

- **Request, then echo**, as for Mode: after every Settings write the device notifies the full payload with the
  values in effect. Numbers out of range are clamped; an unknown id, a wrong length, a bool > 1 or a mode value ≥ 6
  is rejected (unchanged, still notified).
- **Changes from the web UI** are notified within about 50 ms.
- **Compatibility:** ids are never reused; new settings get new ids and their fields are appended. Ignore trailing
  bytes and unknown flag bits; a different layout version means "update the app".

## Diag

Haldex live diagnostics, the same values as the web UI diagnostics card. Notified once per second while subscribed;
open (no pairing). Optional like Settings: check that the characteristic is present. Little-endian, 20 bytes:

| Offset | Type | Field | Unit |
|---|---|---|---|
| 0 | u8 | layout version (= 1) | - |
| 1 | u8 | source: 0 none, 1 UDS (Gen5 family), 2 KWP2000 (Gen4 0AY) | - |
| 2 | i16 | clutch temperature (Gen5 clutch, Gen4 plate) | 0.1 °C |
| 4 | i16 | oil temperature (Gen4) | 0.1 °C |
| 6 | i16 | control module temperature (Gen5) | 0.1 °C |
| 8 | i16 | cooling fin temperature (Gen5) | 0.1 °C |
| 10 | u16 | supply voltage | 0.01 V |
| 12 | u16 | clutch (valve) current | mA |
| 14 | u16 | clutch duty / PWM | 0.1 % |
| 16 | i16 | oil pressure (Gen4) | 0.01 bar |
| 18 | i16 | estimated torque (Gen4) | Nm |

- **Unknown:** `0x8000` (i16) / `0xFFFF` (u16) for every field the source does not provide, and for all fields when
  the source is 0.
- **Source 0** when live diagnostics are off (Settings flag bit 7 / web UI), the Haldex CAN is down, an external scan
  tool is detected, analyzer mode is on, or the generation has no decoded values (Gen1/2, GM/Ford). UDS values older
  than 5 s also count as unknown.
- **Compatibility:** fields are only appended; drop shorter payloads, ignore extra bytes, a different layout version
  means "update the app".

## Recommended app connection sequence

1. Scan: pick peripherals whose advertised service UUIDs contain `6254A001-…`. Remember the peripheral id for
   reconnects.
2. Connect, then `retrieveServices`.
3. Read Info (check `protoVersion` == 1), Firmware Revision, Mode and Controller.
4. `startNotification` on Mode, Controller and Status (and Settings / Diag, when present). Do this sequentially, awaiting
   each one: Android rejects parallel CCCD writes.
5. Writes use write-with-response (Mode / Controller: 1 byte; Settings: `[id][value]`). If a write fails with an authentication error, wait for
   pairing to complete and retry once.
