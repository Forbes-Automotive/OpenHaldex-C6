<p align="center">
  <a href="https://forbes-automotive.com/?utm_source=github&utm_medium=readme&utm_campaign=openhaldex" target="_blank">
    <picture>
      <source media="(prefers-color-scheme: dark)" srcset="/Images/FA-logo-white.png">
      <source media="(prefers-color-scheme: light)" srcset="/Images/FA-logo.png">
      <img src="/Images/FA-logo.png" width="250" alt="Forbes Automotive">
    </picture>
  </a>
</p>

# OpenHaldex — ESP32‑C6 Haldex Controller

<p align="center">

![Version](https://img.shields.io/github/v/release/Forbes-Automotive/OpenHaldex-C6)
![GitHub stars](https://img.shields.io/github/stars/Forbes-Automotive/OpenHaldex-C6)
![Last commit](https://img.shields.io/github/last-commit/Forbes-Automotive/OpenHaldex-C6)
![Platform](https://img.shields.io/badge/platform-ESP32--C6-blue)
![Hardware](https://img.shields.io/badge/hardware-Haldex%20Gen1%20%7C%20Gen2%20%7C%20Gen4-green)
[![Install Firmware](https://img.shields.io/badge/Install%20Firmware-Click%20Here-success?style=for-the-badge)](https://forbes-automotive.com/pages/module-software-updater)

</p>

OpenHaldex is an **open-source Haldex AWD controller** for Volkswagen and Audi Group vehicles using Haldex Generation 1, 2, 4 (PQ Chassis) and 5 (MQB) differentials. The full source and hardware files are published under the permissive **MIT License**, so anyone is free to use, modify, and redistribute them, and it incorporates MIT‑licensed upstream work — see [Licensing](#licensing).

Install is easy with the new harnesses for later PQ & MQB chassis:

**Lift seat -> Plug in -> Drive it**

**Performance doesn't need to be expensive or complicated!**

Starting from the codebase from A-Banging-Donk for Generation 1 differentials; OpenHaldexC6 has grown to adapt Generation 2, 4 and 5.  

The firmware runs on an **ESP32‑C6** and reads CAN bus messages from the vehicle, allowing the controller to modify or generate commands so the Haldex differential behaves exactly as you've configured.  

It can operate using OEM CAN signals or it is able to run in Standalone mode - which makes it perfect for conversions!

![OpenHaldex-C6](/Images/openHaldexUI.png)

## Contents

- [Features](#features)
- [Purchase](#purchase)
- [Overview](#overview)
- [Hardware](#hardware)
- [Supported Platforms](#supported-platforms)
- [Modes](#modes)
- [Expert Mode](#expert-mode)
- [Installation](#installation)
- [Firmware Installation](#firmware-installation-esp-web-tools)
- [OTA Updates (Wi‑Fi)](#ota-updates-wi-fi)
- [Status Indicators](#status-indicators)
- [Low Power Mode](#low-power-mode)
- [CAN Sniffing](#can-sniffing-savvycan--gvret)
- [Live Diagnostics](#live-diagnostics)
- [Frame Editing](#frame-editing-addingremoving-can-signals)
- [PCB & Enclosure](#pcb--enclosure)
- [Acknowledgements](#acknowledgements)
- [Licensing](#licensing)
- [Disclaimer](#disclaimer)

## Features

- Two TWAI (CAN) interfaces for reading and bridging CAN traffic
- Built‑in Wi‑Fi for on‑device configuration and diagnostics
- Multiple preset modes plus customisable mode profiles
- Tune your Haldex system directly from your phone via Wi-Fi
- Configurable inputs features onboard and external mode switching options 
- Configurable outputs include two high-side drivers for handbrake and brake outputs (or PWM control for optional oil cooling!)
- SavvyCAN support via Wi‑Fi or Serial
- Wi‑Fi access point password protection, with long-press reset
- Force mode via CAN using hazard lights or TC button
- Adjustable LED brightness
- Live diagnostics: UDS (Gen5) and KWP2000‑over‑TP2.0 (Gen2/Gen4) with measurement scaling confirmed against VCDS
- Selectable CAN frames — enable/disable CAN messages per Generation from the Web UI
- Wireless (OTA) firmware and web‑UI updates over Wi‑Fi
- Colour‑coded status indicators throughout the Web UI (green / orange / red — see [Status Indicators](#status-indicators))

![OpenHaldex-C6](/Images/BoardOverview.png)

---

## Purchase

Assembled modules are available from Forbes Automotive if you do not wish to build one yourself:

➡ **[OpenHaldex C6 Controller – Forbes Automotive](https://forbes-automotive.com/products/openhaldex-controller?utm_source=github&utm_medium=readme&utm_campaign=openhaldex)**

---

## Overview

OpenHaldex-C6 sits between your vehicle and the OEM Haldex controller. It can operate as a passthrough (OEM behaviour), and modify messages to request different amounts of differential lock.  

This is the original source of Generation 2, 4 (including GM) and 5 logic - any forks or code copied from this project is NOT the work of Forbes Automotive and therefore we cannot support other work unless it is remains part of this project.

**Supported generations:** Gen1, Gen2, Gen4 and Gen5

> Gen3 is currently unsupported.

---

## Hardware

The PCB is based around an **ESP32‑C6 Mini** (with Wi‑Fi) and has superior protection against ESD and transient voltages.  It has a built-in fuse and uses quality automotive based components for a reliable system.

Two TWAI/CAN controllers are built into the PCB along with external IO control:

- External mode button
- On‑board RGB LED
- Brake / handbrake management (for Generation 1 systems)
  *Note: some brake/handbrake outputs may require a 10k pulldown resistor on certain platforms(!)

These two high‑side drivers for brake/handbrake could be repurposed for other functions like oil coolers!

> This platform replaces the earlier Teensy (OpenHaldex T4) design to provide wireless support and on‑device configuration.

## Supported Platforms

- Generation 1 - PQ 
- Generation 2 - PQ
- Generation 4 - PQ
- Generation 4 - GM
- Generation 4 - Ford (ongoing)
- Generation 5 - MQB (0CQ)
- Generation 5 - PQ (0AY)
- Generation 5 - MQB (VAQ)

---

## Modes

The controller provides several preset modes along with a fully customisable 'Expert' mode':

| Mode | Behaviour | LED Colour |
|-----|-----------|-----------|
| Stock | OEM behaviour | Red |
| FWD | Zero lock | Green | 
| 7525 | 25% lock | Cyan |
| 6040 | 40% lock | Magenta |
| 5050 | 100% lock | Blue |
| Expert | User‑defined lock | White |

---

## Expert Mode

Expert mode allows lock targets to be configured based on **speed and throttle setpoints** using a table inside the Web UI. This is true **full control** over your Haldex system. No guesswork. You tune it and it'll do exactly what you want it to do, every time.  It requires OEM CAN messages to be present for throttle/speed inputs.

![ExpertMode](/Images/expertmode.png)

*Expert mode grid configuration interface within the OpenHaldex-C6 UI.*

---

## Haldex Learning

Allow the controller to learn *your* Haldex by replacing the original methodology of approximating a lock percentage by cycling through all of the available lock percentages.  

Use the 'Learn Haldex' in the Settings page and within one minute the controller will learn how to get EXACTLY the lock percentage you request.  No more approximations, just exact values.

### Long Learn (automated block bisection)

If a normal learn isn't clean (jumps, plateaus, never reaches 100%), **Long Learn** on the Settings page automates the manual "add/remove a frame, learn again" loop, in four phases:

1. **Initial Sweep** — every editable frame block for the selected generation is switched **on** and one learn is run at the **Launch PWM Floor** currently configured. The floor governs the *ramp* (how fast the clutch takes up and releases lock), not how far it can ultimately go, so it isn't hunted through candidate values here — this step just establishes the baseline shape and whether 100% is already reachable as configured.
2. **BPK Adjust** *(Gen5 only, only runs if step 1 didn't reach 100%)* — if the floor alone can't get to 100%, the torque ceiling itself (**Lock Calibration**, `bpkCeilingNm`) is what's capping it. This step forces **Fix Hunting** on for the rest of the run (the default packing has no ceiling concept at all and would otherwise let the sweep silently ignore whatever ceiling is set) and walks the ceiling up in fixed steps, using quick single-point checks at full lock rather than a full sweep, until 100% is reached or the ceiling hits its safe maximum. Fix Hunting itself is **always reverted** to whatever it was before at the end of the run, win or lose — only the calibrated ceiling *value* is kept, so it needs turning on by hand afterward to actually take effect.
3. **Sweeping Blocks** — with the calibrated floor/ceiling locked in, each *additional* block (anything outside the generation's default set) is removed one at a time and checked with a quick **release-to-0-then-back-to-100%** cycle (not a full 101-step re-sweep, but a real cycle rather than a frozen hold — holding steady at 100% and just flipping the mask bit turned out to be unreliable, since the controller doesn't cleanly re-evaluate a step change while already sitting at max). Any block whose removal *affects* the result goes back on — **needed** if it got worse (including feedback dropping to nothing — that's the strongest possible "needed" signal, not a failure), flagged **affects (better without)** if it improved; one that makes no difference is **not needed** and stays off. Tick *Also test the default (core) blocks* to bisect everything.
4. **Confirmation** — a full sweep is stored against the final block set (this is the one place a full 0–100% sweep still matters, since the stored table is used to interpolate every lock target, not just 100%).

Expect 10–20 minutes with the car running and Haldex CAN active. The tracker shows the current phase, sweep count, the block under test, the current floor and torque ceiling, the reference score and the live Sent/Returned bars. Cancelling (or losing Haldex data during the Initial Sweep or Confirmation phases) restores the blocks, floor, torque ceiling, Fix Hunting toggle and learn table that were in place before the run.

Below the tracker, a **Chassis / car notes** box (saved on the unit) and **Export report (.txt)** produce a plain-text record of the car, calibration values, the recommended block set (as a checklist plus the raw mask), the full sweep log and the stored learn table — handy for sharing a known-good layout for a given chassis.

### Gen5 'Fix Hunting'

Certain Generation 5 controllers — specifically those running the **554K** variant — use a different torque model to calculate the lock request. On these units, the standard learning process will not produce accurate results. Use the **Fix Hunt** option in the Settings page to ensure the correct torque model is used.  

---

## Changing Modes

Modes can be changed using multiple interfaces:

**On‑board button**

Press the `Mode` button to cycle through modes.

**Wi‑Fi**

Use the Web UI at:

```
192.168.1.1 or openhaldex.local
```

**CAN**

See [CAN Broadcast / Change Mode](#can-broadcast--change-mode) for the required **Broadcast over CAN** setting, CAN IDs and mode byte.

**CAN — Force Mode (Hazards / TC Button)**

Two optional force mode triggers are available via. CAN signals already present on the bus:

- **Hazard lights** — activating the hazard switch will trigger a force mode command
- **TC button** — pressing the traction control button can also trigger force mode

Both are optional and can be enabled in the Settings page. They allow mode changes from OEM controls without any additional wiring.

---

## CAN Broadcast / Change Mode

The same **Broadcast over CAN** setting enables both:

- External mode-change commands received by the controller on `0x6A0`
- OpenHaldex state broadcast from the controller on `0x6B0`

> [!NOTE]
> Enabling these features will broadcast NEW CAN IDs the system might already look for(!).  Caution should be exercised when using this feature(!). 

### Change Mode Request

**Broadcast over CAN must be enabled** in Settings, otherwise the controller will ignore the mode-change frame.

Send a standard 11-bit CAN frame to `0x6A0`. Put the required mode value in **Byte 0 / data[0]**. If sending an 8-byte frame, set unused bytes to `0x00`.

| CAN ID | Byte | Value | Mode |
|----|----|----|----|
| `0x6A0` | `0` / `data[0]` | `0x01` | Stock |
| `0x6A0` | `0` / `data[0]` | `0x02` | FWD |
| `0x6A0` | `0` / `data[0]` | `0x03` | 50:50 |
| `0x6A0` | `0` / `data[0]` | `0x04` | 60:40 |
| `0x6A0` | `0` / `data[0]` | `0x05` | 75:25 |
| `0x6A0` | `0` / `data[0]` | `0x06` | Expert |

### Broadcasted State

Default CAN ID:

```
0x6B0
```
The module broadcasts its current state on the CAN bus.  This can be used by aftermarket ECUs or FIS displays to show current status

```
data[0] = reserved 
data[1] = standalone flags (bitmask for Gen1/Gen2/Gen4/Gen5)
data[2] = processed haldex engagement (requested by firmware)
data[3] = lock target percent (actual lock, returned by differential)
data[4] = vehicle speed (kmh)
data[5] = mode_override_flag (legacy)
data[6] = current_mode_number (0...5)
data[7] = driver's pedal value (percentage: 0...100%)
```

---

## Wi‑Fi Setup

1. Connect to the access point **OpenHaldex‑C6** (open by default).
2. Open a browser and navigate to

```
192.168.1.1 or openhaldex.local
```

3. Access the Web UI

### Connecting from a phone

**Use your phone's Wi‑Fi settings — not its Personal Hotspot / hotspot feature.** The controller is the access point: it broadcasts its own Wi‑Fi network (**OpenHaldex‑C6** by default), the same way a home router does. Your phone just joins that network like any other:

1. Phone **Settings → Wi‑Fi** → select **OpenHaldex‑C6** and connect (open network by default, no password).
2. Browse to `192.168.1.1` or `openhaldex.local`.

Do **not** turn on your phone's own Hotspot/Personal Hotspot for this — that shares *your* phone's data with other devices, which is the opposite of what's needed here and won't let the phone reach the controller.

> [!NOTE]
> The controller has no internet access and doesn't need any. Your phone will show *"Wi‑Fi has no internet access"* while connected — that's expected; dismiss it. If the phone keeps dropping the network (some Android versions do this to a Wi‑Fi with no internet), tap the warning and choose **"Stay connected"**. 

If you get stuck mid-way (e.g. the phone auto‑connects to OpenHaldex‑C6 in the background and you didn't mean to), just forget the network or reconnect — the controller keeps running normally whether or not anything is connected to it.

From the **Diagnostics** page you can:

- **Change the WiFi Name (SSID)** — set any 1–32 character printable name (e.g. `MyHaldex`, `MK7-R`). The AP restarts immediately on save.
- **Set / change the WiFi Password** — WPA2, minimum 8 characters. Leave blank for an open network.

If you are locked out or forget your password, **long-pressing the `Mode` button** will clear the WiFi password and restore the open AP. The SSID is preserved (use **Reset to Default** in the UI if you want to revert the name to `OpenHaldex-C6`).

<p align="center">
  <img src="/Images/UIDemo.png" alt="OpenHaldex C6 Web UI" width="900" style="max-width:100%;">
</p>

If the Wi‑Fi interface becomes unresponsive:

- Long‑press the `Mode` button to clear the WiFi password and restart the AP to an open state.

> The Web UI is built on Forbes Automotive's shared dark theme (the same look used across the Can2Cluster, SpeedPulser, SpeedPulserPro, can2rpm, MQB Steering Wheel Controller and AirLift Controller firmware), so the interface and status conventions stay consistent across the whole product line. Current firmware version: **9.00.0** (shown in the UI footer and at `/ota/info`).

---

## Low Power Mode

OpenHaldex C6 is designed to live on a **permanent +12 V** feed without flattening your battery. With Low Power Mode configured correctly the controller draws approximately:

| State | Current |
|---|---|
| Sleeping (car off, WiFi off, CAN quiet) | **~14 mA** |
| Awake (CAN activity detected *or* WiFi client connected) | **~50 mA** |

There are three layers of power saving. Each layer builds on the previous one and all are configured from the **Settings** page.

---

### Layer 1 — Idle AP Shutdown *(always active, no setup needed)*

After **5 minutes** with no WiFi clients connected and no CAN activity, the controller automatically shuts down the WiFi AP and turns off the LED. This runs regardless of any other Low Power settings.

As soon as CAN traffic resumes — typically the instant the car wakes up — WiFi is restored automatically. No user action required.

---

### Layer 2 — CAN Sleep *(optional)*

Enable the **CAN Sleep** toggle in Settings. When active:

- The ESP32-C6 CPU enters **light sleep** whenever FreeRTOS is idle, cutting CPU power consumption significantly.
- The TWAI (CAN) peripheral powers down during sleep but **preserves its registers and receive queue** across cycles, so no messages are lost on wake-up.
- The CAN transceiver chips remain powered and continue listening on the bus, so the first incoming frame wakes the controller instantly.

This mode gives the majority of power savings for most installs and is the recommended starting point.

---

### Layer 3 — CAN Sleep (Aggressive) *(optional, builds on Layer 2)*

Enable **CAN Sleep (Aggressive)** in Settings — everything from Layer 2 remains active, plus:

- **The CAN transceiver chips are completely shut down** (standby pin asserted). This eliminates the ~10 mA standby draw per transceiver.
- The CPU minimum clock drops to **10 MHz**.
- WiFi AP transmit power is trimmed to reduce active-client current.
- Wake is **interrupt-driven**: a GPIO ISR on each CAN_RX line fires on the first bus edge and re-enables the transceivers within microseconds. 

Use Aggressive mode when the car sits unused for days at a time and you want the absolute lowest standby current (~14 mA).

---

### How to set it up

Low Power Mode requires **one short calibration step** the first time, because every car idles its CAN bus at a slightly different rate.

**Step 1 — Measure your Sleeping CAN rate**

1. Park and lock the car. Wait 15 minutes or until the Chassis bus goes fully quiet.
2. Stay connected to the OpenHaldex WiFi AP — the controller will remain awake while a client is connected.
3. Open the Web UI → **Settings** and watch the **Chassis fps** and **Haldex fps** counters.
4. Note the highest reading you see while the car is asleep. Most cars show **800 fps**; some show a handful of fps from a periodic gateway heartbeat.

**Step 2 — Set the wake threshold**

5. Set the **LP Wake Threshold (fps)** slider to a value slightly **above** the parked-bus reading — e.g. if the bus is quiet at 0 fps, set the slider to **5**; if it idles at 8 fps, set the slider to **15**.

**Step 3 — Enable Sleep and Disconnect**

6. Enable **CAN Sleep** (and optionally **CAN Sleep (Aggressive)**) in Settings.
7. **Disconnect from the WiFi AP** (close the browser / forget the network).

The controller will now sleep, drawing ~14 mA, and **wake automatically whenever the CAN frame rate rises above the threshold** — i.e. when you unlock or start the car.  The LED, CAN chips and CPU are all off/reduced.

> [!NOTE]
> **Standalone mode:** with no chassis bus, the Haldex ECU itself sleeps fully, so any Haldex-bus CAN traffic wakes the module regardless of the slider value.

> [!NOTE]
> **Switched-ignition installs:** if the module is already powered off with the ignition, Low Power Mode saves little and is optional.  
---

## Installation

>[!TIP]
> ### Optional Plug & Play Harness (Recommended)
>
> Recommended for quick installation (and removal) — typically **<10 minutes** on Generation 1 Controllers.
> The latest harnesses for Generation 5 are even simpler and you'll be experiencing your Haldex controller in less than 30 seconds:

- Lift the rear seat
- Split the factory 6-pin Haldex connector
- Install your new harness & OpenHaldexC6 Controller
- Drive it (you could put the seat back down too, if you want!)

> For Generation 1 Controllers:

- Remove original connector and install the long end of the harness onto the differential.
- Route the long end along with the original connector back into the boot floor via. the OEM grommet
- Install and secure the OpenHaldexC6 Controller to the new harness, pairing it with the original plug

For full step‑by‑step instructions see the **[OpenHaldex Installation Guide](https://openhaldex.com/docs/OpenHaldex_Installation_Guide.pdf)**.

▶ **Installation demo (YouTube Short):** https://youtube.com/shorts/iUkNh9NbyKY?si=IhgqLIi0WM8wXqe9

▶ **Installation demo (YouTube Short):** https://youtu.be/Wu-u-Dz1444

> [!WARNING]
> ### Manual Wiring (No Harness)
>
> Modules sold without a harness include connector pins for manual wiring. This is a little harder and more involved than using the optional harness, but following the installation guide above it can still be completed easily. Give us a shout if you need a hand.

Gen1:
- Chassis Connector: `1J0‑973‑714`
- Haldex Conneector: `1J0‑973‑814`

Gen4>:
- Haldex Connector — `VW 1J0‑973‑713`
- Vehicle Connector — `VW 1J0‑973‑813`

Build this as a **Y-branch** harness between the two VW 8 or 6-pin connectors, with a long tail to the MX plug.

Routing summary:

- **Permanent power (Term30) and ground must also be branched to the OpenHaldex controller**:
  - Term30 -> MX Pin 1
  - Ground -> MX Pin 2
  
- Chassis CAN is taken from the Vehicle side and sent to the controller:
  - Vehicle Pin 5 -> MX Pin 3 (Chassis CAN Low)
  - Vehicle Pin 6 -> MX Pin 4 (Chassis CAN High)

- Returned CAN from the controller then goes to the Haldex side:
  - MX Pin 5 -> Haldex Pin 5 (Haldex CAN Low)
  - MX Pin 6 -> Haldex Pin 6 (Haldex CAN High)

Generation 1 > 4:
- Vehicle Connector — `VW 1J0‑973‑714`:

| Pin | Signal | Notes |
|----|------|------|
| 1 | Term15 | Pass-through: Vehicle → Haldex |
| 2 | Ground | Pass-through: Vehicle → Haldex and branch to MX Pin 2 |
| 3 | Brake Light | Pass-through: Vehicle → Haldex |
| 4 | Handbrake | Pass-through: Vehicle → Haldex |
| 5 | K-Line | Pass-through: Vehicle → Haldex |
| 6 | N/A | Not Used |
| 7 | Chassis Low | To MX Pin 3 (chassis side) |
| 8 | Chassis High | To MX Pin 4 (chassis side) |

- Haldex Connector — `VW 1J0‑973‑814`:

| Pin | Signal | Notes |
|----|------|------|
| 1 | Term15 | Pass-through: Vehicle → Haldex |
| 2 | Ground | Pass-through: Vehicle → Haldex and branch to MX Pin 2 |
| 3 | Brake Light | Pass-through: Vehicle → Haldex |
| 4 | Handbrake | Pass-through: Vehicle → Haldex |
| 5 | K-Line | Pass-through: Vehicle → Haldex |
| 6 | N/A | Not Used |
| 7 | Chassis Low | To MX Pin 5 (Haldex side) |
| 8 | Chassis High | To MX Pin 6 (Haldex side) |

Generation 5:
- Vehicle Connector — `VW 1J0‑973‑813`:

| Pin | Signal | Notes |
|----|------|------|
| 1 | Term15 | Pass-through: Vehicle → Haldex |
| 2 | Ground | Pass-through: Vehicle → Haldex and branch to MX Pin 2 |
| 3 | Term30 | Pass-through: Vehicle → Haldex and branch to MX Pin 1 |
| 4 | N/A | Not used |
| 5 | Chassis Low | To MX Pin 3 (chassis side) |
| 6 | Chassis High | To MX Pin 4 (chassis side) |

- Haldex Connector — `VW 1J0‑973‑713`:

| Pin | Signal | Notes |
|----|------|------|
| 1 | Term15 | Pass-through: Vehicle → Haldex |
| 2 | Ground/MALT | Pass-through from Vehicle side |
| 3 | Term30 | Pass-through from Vehicle side |
| 4 | N/A | Not used |
| 5 | Haldex Low | From MX Pin 5 (Haldex side) |
| 6 | Haldex High | From MX Pin 6 (Haldex side) |

![Gen4/Gen5 Y-Branch Harness Diagram](/Images/Gen4_Gen5_Y_Branch_Harness.png)

### MX23A12NF Connector Pinout

| Pin | Signal | Notes |
|----|------|------|
| 1 | Vbatt | +12 V |
| 2 | Ground/MALT | Ground |
| 3 | Chassis CAN Low | To chassis/ECU side |
| 4 | Chassis CAN High | To chassis/ECU side |
| 5 | Haldex CAN Low | To Haldex differential |
| 6 | Haldex CAN High | To Haldex differential |
| 7 | Switch Mode External | +12 V to activate |
| 8 | Brake Switch In | +12 V input |
| 9 | Brake Switch Out | Gen1 / 2 differentials only |
|10 | Handbrake Switch In | +12 V input |
|11 | Handbrake Switch Out | Gen1 differentials only |

---

## Firmware Installation (ESP Web Tools)

Firmware can be installed directly from your browser using **ESP Web-Tools**.  
This is the recommended method for most users.

1. Connect the OpenHaldex-C6 controller to your computer using a **data-capable USB-C cable**.
2. Open the firmware installer page: **[Module Software Updater](https://forbes-automotive.com/pages/module-software-updater?utm_source=github&utm_medium=readme&utm_campaign=openhaldex)**
3. Click **Connect** and select the OpenHaldex serial port.
4. Click **Install** and follow the prompts.
5. Wait for the firmware to flash and the device to reboot.

> [!NOTE]
> Some USB-C cables are **power-only** and will not work for flashing.  
> If the device does not appear, try a different cable or USB port.

---

## OTA Updates (Wi‑Fi)

Once a controller is already running firmware **8.00.0 or later**, it can also be updated wirelessly over its own Wi‑Fi AP instead of by USB. This is a **two‑step process**, because the filesystem (web UI) and the firmware (application code) live on separate flash partitions and are updated independently:

1. **Step 1 — Filesystem update** (`POST /ota/update/fs`, uploads `littlefs.bin`)  
   Updates `index.html`, `app.js`, `style.css` and other web assets. The device does **not** reboot after this step — it's meant to be followed immediately by Step 2.
2. **Step 2 — Firmware update** (`POST /ota/update`, uploads `firmware.bin`)  
   Updates the application itself. The device reboots automatically once the upload finishes.

Both endpoints accept an optional `?sha256=<hex>` query parameter; when present the device hashes the upload as it arrives and refuses to activate an image that doesn't match. They need no password, but are **safety‑gated**:

- On the **bench** (no CAN detected), updates are allowed at any time.
- In the **vehicle** (CAN detected), an update is only allowed when the vehicle is stationary, both CAN buses are healthy, the controller is disabled or forced to Stock, and no Haldex temperature‑protection fault is active. `GET /ota/check` reports live pass/fail plus the reason if blocked.

After a firmware update the new image boots in a **pending‑verify** state (ESP‑IDF's rollback mechanism). It is only marked valid — cancelling the automatic rollback — once the web UI has been reached on the new image, or after 60 s of uptime; a build that crash‑loops before either point is reverted to the previous working firmware by the bootloader on the next reset, so a bad flash can't strand the car.

Other useful endpoints: `GET /ota/info` (current version, web‑UI version, chip, free heap, running partition), `GET /ota/fsinfo` (remounts the filesystem and reports the web‑UI version it holds — used to verify Step 1 before Step 2) and `GET /ota/health` (simple liveness check).

### Doing it from the OTA page

1. On a phone or laptop with internet, download `littlefs.bin` and `firmware.bin` from the newest `V…` folder under [`Releases/`](Releases/) (use GitHub's **Download raw file** button).
2. Connect to the **OpenHaldex‑C6** Wi‑Fi and open the **OTA** tab.
3. **Update Type → Filesystem (web UI)**, pick `littlefs.bin`, **Upload**. The step tracker marks it done and switches to Firmware.
4. **Update Type → Firmware (application)**, pick `firmware.bin`, **Upload**. The device reboots by itself; reconnect and reload the page.

Nothing on the controller ever fetches from the internet — the phone needs a connection only for step 1, which can happen anywhere. Keep the page open and the screen on during each upload.

> [!NOTE]
> USB via ESP Web‑Tools (above) remains the recommended method for a controller's **first** flash or for recovering from a failed update; OTA is for updating a controller that's already running.

---

## Status Indicators

Throughout the Web UI — Diagnostics, CAN health, force‑mode sources, and similar live readouts — status is shown as a coloured pill or dot using a consistent convention:

| Colour | Meaning |
|---|---|
| 🟢 Green | Healthy / On / Available — the signal is present and in the expected state |
| 🔴 Red | Unhealthy / Off / Inactive — the signal is present but reporting a bad or inactive state |
| 🟠 Orange | Unavailable — no value has been received for this field (e.g. not applicable to the selected Haldex generation, or data hasn't arrived yet) |

This applies to items such as Chassis/Haldex CAN health, steering health, ASR/TC status, hazard and brake/handbrake inputs, and the OTA safety banner (`SAFE` in green / `NOT SAFE` in red).

---

## CAN Sniffing (SavvyCAN / GVRET)

Adding more functionality couldn't be easier - you just need to know the feature you want and grab some CAN data to help implement it.  Using the SavvyCAN interface, you can listen to the CAN messages to see what the car is talking about...

> [Thanks to Maetro for this feature!]

Enable **Analyzer Mode** in the setup menu to capture CAN frames.

> [!WARNING]
> Enabling Analyzer Mode disables active Haldex control and returns the device to OEM 'pass-through' behaviour.

### SavvyCAN Connection

1. Connect to the OpenHaldex Wi‑Fi AP
2. In Settings, enable SavvyCAN via WiFi

3. In SavvyCAN:

``` 
Open 'Connection'
```

```
Add New Device Connection → Network Connection (GVRET)
```

4. Enter:

```
IP: 192.168.1.1
Port: 23
```

5. Set CAN speed:

```
500000
```

### Serial Connection

SavvyCAN can also connect directly over USB without needing Wi‑Fi:

1. Connect the OpenHaldex controller via USB-C
2. In Settings, enable SavvyCAN via Serial
3. In SavvyCAN, add a new device connection → **Serial Connection (GVRET)**
4. Select the correct COM port and set the speed to **500000**

---

## Live Diagnostics

OpenHaldex can request data from the Haldex ECU for **live measurement data** and display it in the Web UI - just like a scan tool would. This occupies the module's diagnostic channel, it is controlled by a single **Enable Live Diagnostics** toggle in Settings that is **off by default** — however if you use VCDS, ODIS or another diagnostic tool, it will automatically turn itself off until the scan tool is removed.

When enabled, the controller automatically uses the correct protocol for the configured generation — there is nothing else to select.

### Gen5 (MQB `0CQ` / PQ `0AY`) — UDS

Gets data from the Haldex ECU over ISO‑TP / UDS and decodes:

- Terminal voltage
- Control module temperature
- Clutch temperature
- Cooling‑fin temperature
- Clutch current, PWM and voltage

All scaling has been **confirmed against VCDS** by heat‑soaking a bench module and matching the reported values across the full temperature range.  

### Gen2 (`1K0`) / Gen4 (`0AY`) — KWP2000 over VW TP2.0

The PQ‑platform controllers are diagnosed with **KWP2000 tunnelled over VW TP2.0** rather than UDS. OpenHaldex opens the TP2.0 channel, starts a diagnostic session and reads the measuring blocks. Gen4 values are decoded and **confirmed against VCDS**:

- Oil temperature & clutch plate temperature
- Supply voltage
- Oil pressure
- Estimated torque
- Clutch valve duty (%) and current (A)

Raw measuring‑block bytes are also exposed so any remaining values can be characterised.

> [!NOTE]
> If VCDS or other scan tools will not connect, turn this setting off as it may not cleanly pick up the VCDS request so it does not block the tool.  Excessive CAN traffic can cause spurious dash errors.

---

## Frame Editing (Adding/Removing CAN Signals)

From the **Diagnostics** page you can enable or disable OpenHaldex's editing of **individual CAN frames** for the selected Haldex generation.

- Each editable frame for the current generation is listed as a checkbox.
- **Unchecking** a frame leaves the car's original message untouched (clean pass‑through); **checking** it lets OpenHaldex modify or generate that frame.
- Changes apply immediately and are saved per generation.
- A **Reset to Defaults** button restores the generation's standard set.

This is useful for **understanding** which edited frame upsets a Haldex learn procedure or causes fault codes, for tailoring behaviour on unusual vehicles, or for adding/removing specific signals during development.

> [!NOTE]
> Frame editing can apply to both 'Normal' and 'Standalone' modes and only to the currently selected generation.

---

## PCB & Enclosure

Gerber files and enclosure designs are available in the **PCB** folder.  You can use these to get your own units if you wish.

Pinout and functionality remain consistent across supported enclosure versions.

![OpenHaldex-C6](/Images/BoardTop.png)
![OpenHaldex-C6](/Images/BoardBottom.png)

---

## Nice to Haves

- Flashing LED indicator when CAN message transmission fails

---

## Acknowledgements

- **A Banging Donk** — [Original OpenHaldex project](https://github.com/ABangingDonk/OpenHaldexT4) for Gen1 vehicles
- **Chris (meatro) — OpenHaldex‑S3** — [OpenHaldex‑S3](https://github.com/meatro/OpenHaldex-S3) (MIT). Portions of the map editor / Expert Mode, CAN View, web UI, PlatformIO structure and API control derive from this project
- **RktBox (Kile Thomson) — OpenHaldex‑Edge** — [OpenHaldex‑Edge](https://github.com/Kile-Thomson/OpenHaldex-Edge). The geometry‑compensated per‑corner slip calculation (Ackermann wheel‑radius model) and the ESP_14 **Launch PWM Floor** (`BR_Vorg_*_Min` raise) were adopted from this project
- **Arwid Vasilev** — PCB redesign (V1.02)
- **LVT Technologies** — OTA update integration (now deprecated, but still appreciated)

---

## Licensing

OpenHaldex‑C6 is **open source** under the permissive [MIT License](https://opensource.org/licenses/MIT). The firmware source and the hardware design files (Gerbers, schematics, PCB layouts and enclosure files) are published so you can freely build, inspect, modify, redistribute and fabricate a controller for any purpose, including commercial use, subject only to preserving the copyright and license notices.

- **Forbes Automotive original code and hardware design files** (Gen2 / Gen4 / Gen5 work, PCB Gerbers, schematics and enclosure files) — **MIT License**. See [LICENSE.md](LICENSE.md).
- **OpenHaldex‑S3 derived portions** (Chris / meatro) — **MIT**. These remain under MIT; the MIT notice must be preserved.
- **OpenHaldex‑Edge derived portions** (Rekt / Kile Thomson) — per‑corner slip geometry and Launch PWM Floor. Attribution is retained in the source and in the third‑party notices.
- **Original OpenHaldex (Gen1, ABangingDonk)**.

Full attribution and upstream license texts are recorded in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md). If you redistribute source or binaries, keep the third‑party notices and license texts with the distribution.

> **In short:** use it, build it, modify it, redistribute it — that's all encouraged. Just keep the copyright and license notices intact.

---

## Disclaimer

> [!CAUTION]
> This device modifies Haldex behaviour and should only be used **off‑road or on a closed course**.
>
> The unit may behave unpredictably and could increase drivetrain wear.
>
> **Use at your own risk.** Forbes Automotive is not responsible for damages resulting from the use of this device or software.
