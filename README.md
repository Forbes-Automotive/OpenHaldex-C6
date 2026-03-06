<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="/Images/FA-logo-white.png">
    <source media="(prefers-color-scheme: light)" srcset="/Images/FA-logo.png">
    <img src="/Images/FA-logo.png" width="250" alt="Forbes Automotive">
  </picture>
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


OpenHaldex is an open‑source **Haldex AWD controller** for Volkswagen and Audi Group vehicles using Haldex Generation 1, 2 and 4 differentials.

The firmware runs on an **ESP32‑C6 microcontroller** and reads CAN bus messages from the vehicle, allowing the controller to modify or generate commands so the Haldex differential behaves exactly as configured.

## Contents

- [Features](#features)
- [Purchase](#purchase)
- [Overview](#overview)
- [Hardware](#hardware)
- [Modes](#modes)
- [Expert Mode](#expert-mode)
- [Installation](#installation)
- [Firmware Installation](#firmware-installation-esp-web-tools)
- [CAN Sniffing](#can-sniffing-savvycan--gvret)
- [PCB & Enclosure](#pcb--enclosure)
- [Acknowledgements](#acknowledgements)
- [Disclaimer](#disclaimer)

## Features

- Two TWAI (CAN) interfaces for reading and bridging CAN traffic
- Built‑in Wi‑Fi for on‑device configuration and diagnostics
- Multiple preset modes plus customisable mode profiles
- Tune your Haldex system directly from your phone via Wi-Fi

![OpenHaldex-C6](/Images/BoardOverview.png)

---

## Purchase

Assembled modules are available from Forbes Automotive if you do not wish to build one yourself:

➡ **[OpenHaldex C6 Controller – Forbes Automotive](https://forbes-automotive.com/products/openhaldex-controller?utm_source=github&utm_medium=readme&utm_campaign=openhaldex)**

---

## Overview

OpenHaldex sits between your vehicle and the OEM Haldex controller. It can operate as a passthrough (OEM behaviour), or modify messages to request different amounts of differential lock.

**Supported generations:** Gen1, Gen2 and Gen4

> Gen3 and Gen5 are currently unsupported.

---

## Hardware

The PCB is based around an **ESP32‑C6 Mini** (with Wi‑Fi)

Two TWAI/CAN controllers are built into the PCB along with external IO control:

- External mode button
- On‑board RGB LED
- Brake / handbrake signals

There are also two high‑side drivers for brake/handbrake but these could be repurposed for other functions.

> This platform replaces the earlier Teensy (OpenHaldex T4) design to provide better wireless support and on‑device configuration.

---

## Modes

The controller provides several preset modes along with a fully customisable mode:

| Mode | Behaviour |
|-----|-----------|
| Stock | OEM behaviour |
| FWD | Zero lock |
| 7525 | 25% lock |
| 6040 | 40% lock |
| 5050 | 100% lock |
| Expert | User‑defined lock profile |

---

## LED Indicators

5 mm onboard LED colour indicates the active mode:

| Colour | Mode |
|------|------|
| Red | Stock |
| Green | FWD |
| Cyan | 7525 |
| Magenta | 6040 |
| Blue | 5050 |
| White | Expert |

---

## Expert Mode

Expert mode allows lock targets to be configured based on **speed and throttle setpoints** using a table inside the Web UI. This is true **full control** over your Haldex system. No guesswork. You tune it and it'll do exactly what you want it to do, every time.

![ExpertMode](/Images/expertmode.jpg)

*Expert mode grid configuration interface within the OpenHaldex C6 UI.*

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

Send a CAN message containing the mode number in **Byte 0**.

---

## Mode Numbers (CAN Byte 0)

```
Stock = 0
FWD = 1
5050 = 2
6040 = 3
7525 = 4
Expert = 5
```

---

## Broadcasted State

Default CAN ID:

```
0x6B0
```

> [!NOTE]
> Broadcasting can conflict with other devices. The CAN ID can be adjusted if required.

The module broadcasts its current state on the CAN bus.

```
data[1] = standalone_flags (bitmask for Gen1/Gen2/Gen4)
data[2] = processed_haldex_engagement (mapped by firmware)
data[3] = lock_target_percent
data[4] = vehicle_speed
data[5] = mode_override_flag
data[6] = current_mode_number
data[7] = pedal_value
```

---

## Wi‑Fi Setup

1. Connect to the access point **OpenHaldex‑C6**
2. Open a browser and navigate to

```
192.168.1.1 or openhaldex.local
```

3. Access the Web UI

<p align="center">
  <img src="/Images/UIDemo.png" alt="OpenHaldex C6 Web UI" width="900" style="max-width:100%;">
</p>

If the Wi‑Fi interface becomes unresponsive:

- Long‑press the `Mode` button to reset Wi‑Fi.

---

## Installation

>[!TIP]
> ### Optional Plug & Play Harness (Recommended)
>
> Recommended for quick installation (and removal) — typically **2 minutes**.

Route cables as required and connect harness ends as described in the harness instructions.

For full step‑by‑step instructions see the **[OpenHaldex Installation Guide](https://openhaldex.com/docs/OpenHaldex_Installation_Guide.pdf)**.

▶ **Installation demo (YouTube Short):** https://youtube.com/shorts/iUkNh9NbyKY?si=IhgqLIi0WM8wXqe9

> [!WARNING]
> ### Manual Wiring (No Harness)
>
> Modules sold without a harness include connector pins for manual wiring. This is a little harder and more involved than using the optional harness, but with following the installation guide above it can still be completed easily. Give us a shout if you need a hand.

- Haldex Connector — `VW 1J0‑973‑713`
- Vehicle Connector — `VW 1J0‑973‑813`

![InstallationPins-C6](/Images/InstallationPins.png)

### MX23A12NF Connector Pinout

| Pin | Signal | Notes |
|----:|:------|:------|
| 1 | Vbatt | +12 V |
| 2 | Ground/MALT | Ground |
| 3 | Chassis CAN Low | To chassis/ECU side |
| 4 | Chassis CAN High | To chassis/ECU side |
| 5 | Haldex CAN Low | To Haldex differential |
| 6 | Haldex CAN High | To Haldex differential |
| 7 | Switch Mode External | +12 V to activate |
| 8 | Brake Switch In | +12 V input |
| 9 | Brake Switch Out | Gen1 differentials only |
|10 | Handbrake Switch In | +12 V input |
|11 | Handbrake Switch Out | Gen1 differentials only |

---

## Firmware Installation (ESP Web Tools)

Firmware can be installed directly from your browser using **ESP Web Tools**.  
This is the recommended method for most users.

1. Connect the OpenHaldex controller to your computer using a **data-capable USB-C cable**.
2. Open the firmware installer page: **[Module Software Updater](https://forbes-automotive.com/pages/module-software-updater?utm_source=github&utm_medium=readme&utm_campaign=openhaldex)**
3. Click **Connect** and select the OpenHaldex serial port.
4. Click **Install** and follow the prompts.
5. Wait for the firmware to flash and the device to reboot.

> [!NOTE]
> Some USB-C cables are **power-only** and will not work for flashing.  
> If the device does not appear, try a different cable or USB port.


---

## CAN Sniffing (SavvyCAN / GVRET)

Enable **Analyzer Mode** in the setup menu to capture CAN frames.

> [!WARNING]
> Enabling Analyzer Mode disables active Haldex control and returns the device to OEM behaviour.

### SavvyCAN Connection

1. Connect to the OpenHaldex Wi‑Fi AP
2. In SavvyCAN select:

```
Add New Device Connection → Network Connection (GVRET)
```

3. Enter:

```
IP: 192.168.1.1
Port: 23
```

4. Set CAN speed:

```
500000
```

---

## PCB & Enclosure

Gerber files and enclosure designs are available in the **PCB** folder.

Pinout and functionality remain consistent across supported enclosure versions.

![OpenHaldex-C6](/Images/BoardTop.png)
![OpenHaldex-C6](/Images/BoardBottom.png)

---

## Nice to Haves

- Flashing LED indicator when CAN message transmission fails

---

## Acknowledgements

- **Forbes Automotive** — Lead development of the OpenHaldex C6 platform, including reverse‑engineering and open‑source implementation for **Gen2 and Gen4 Haldex systems**, along with ongoing maintenance of the project
- **A Banging Donk** — [Original OpenHaldex project](https://github.com/ABangingDonk/OpenHaldexT4) for Gen1 vehicles
- **Arwid Vasilev** — PCB redesign (V1.02)
- **LVT Technologies** — OTA update integration (now deprecated, but still appreciated)

---

## Disclaimer

> [!CAUTION]
> This device modifies Haldex behaviour and should only be used **off‑road or on a closed course**.
>
> The unit may behave unpredictably and could increase drivetrain wear.
>
> **Use at your own risk.** Forbes Automotive is not responsible for damages resulting from the use of this device or software.
