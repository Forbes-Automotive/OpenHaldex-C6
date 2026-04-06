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

OpenHaldex is an open‑source **Haldex AWD controller** for Volkswagen and Audi Group vehicles using Haldex Generation 1, 2, 4 (PQ Chassis) and 5 (MQB) differentials.

Install is easy with the new harnesses for later PQ & MQB chassis:

**Lift seat -> Plug in -> Drive it**

**Performance doesn't need to be expensive or complicated!**

Starting from the codebase from A-Banging-Donk for Generation 1 differentials; OpenHaldexC6 has grown to adapt Generation 2, 4 and 5.  

The firmware runs on an **ESP32‑C6** and reads CAN bus messages from the vehicle, allowing the controller to modify or generate commands so the Haldex differential behaves exactly as you've configured.  

It can operate using OEM CAN signals or it is also able to run in Standalone mode - this makes it perfect for conversions!

![OpenHaldex-C6](/Images/openHaldexUI.png)

## Contents

- [Features](#features)
- [Purchase](#purchase)
- [Overview](#overview)
- [Hardware](#hardware)
- [Advantages Over Other 'OpenHaldex' Solutions](#advantages-over-other-openhaldex-solutions)
- [Supported Platforms](#supported-platforms)
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
- Configurable inputs features onboard and external mode switching options 
- Configurable outputs include two high-side drivers for handbrake and brake outputs (or PWM control for optional oil cooling!)
- SavvyCAN support

![OpenHaldex-C6](/Images/BoardOverview.png)

---

## Purchase

Assembled modules are available from Forbes Automotive if you do not wish to build one yourself:

➡ **[OpenHaldex C6 Controller – Forbes Automotive](https://forbes-automotive.com/products/openhaldex-controller?utm_source=github&utm_medium=readme&utm_campaign=openhaldex)**

---

## Overview

OpenHaldex sits between your vehicle and the OEM Haldex controller. It can operate as a passthrough (OEM behaviour), or modify messages to request different amounts of differential lock.  

This is the original source of Generation 2, 4 and 5 logic - any forks or code copied from this project is NOT the work of Forbes Automotive and therefore we cannot support other work unless it is remains part of this project.

**Supported generations:** Gen1, Gen2, Gen4 and Gen5

> Gen3 is currently unsupported.

---

## Hardware

The PCB is based around an **ESP32‑C6 Mini** (with Wi‑Fi) but has superior protection against ESD and transient voltages.  It has a built-in fuse and uses quality automotive based components for a reliable system.

Two TWAI/CAN controllers are built into the PCB along with external IO control:

- External mode button
- On‑board RGB LED
- Brake / handbrake management (for Generation 1 systems)

These two high‑side drivers for brake/handbrake could be repurposed for other functions like oil coolers!

> This platform replaces the earlier Teensy (OpenHaldex T4) design to provide better wireless support and on‑device configuration.

## Advantages Over Other 'OpenHaldex' Solutions

- The 'proprietary PCB' is equally as proprietary as the LILYGO T-2Can
- The LILYGO T-2Can does not feature any suitable means of over-voltage or fused protections
- The LILYGO T-2Can does not feature an onboard mode LED - so no immediate user feedback of current mode
- The LILYGO T-2Can does not offer external mode changing for quick 'on-the-fly' changes
- The LILYGO T-2Can does not offer additional high-sided drivers for brake/handbrake control on Gen1 or Gen2 systems
- The LILYGO T-2Can uses MS4553S as CAN interface chips - these have lower ESD protection and in high-voltage or short circuit situations will destroy their irreplaceable protection fuses
- No optional JTAG break-out (which could be repurposed for other features - like GPS, for example)
- No pre-designed enclosure
- Terminations are screw-type - which can lead to poor user installation / connections - leading to sporadic communication faults or failures within the vehicles CAN network - this could include no-starts or steering failures
- It's **still** a custom made piece of hardware including the custom wiring harness YOU'D need to make: collect connectors, crimp/solder or twist & tape, it won't be pretty
- The cost is comparative
---

## Supported Platforms

- Generation 1 - PQ 
- Generation 2 - PQ
- Generation 4 - PQ
- Generation 5 - MQB

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

5mm onboard LED colour indicates the active mode:

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

Expert mode allows lock targets to be configured based on **speed and throttle setpoints** using a table inside the Web UI. This is true **full control** over your Haldex system. No guesswork. You tune it and it'll do exactly what you want it to do, every time.  It requires OEM CAN messages to be present for throttle/speed inputs.

![ExpertMode](/Images/expertmode.jpg)

*Expert mode grid configuration interface within the OpenHaldex C6 UI.*

---

## Haldex Learning

Allow the controller to learn *your* Haldex by replacing the original methodology of approximating a lock percentage by cycling through all of the available lock percentages.  

Use the 'Learn Haldex' in the Settings page and within one minute the controller will learn how to get EXACTLY the lock percentage you request.  No more guess-work, just exactly values.

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
> Broadcasting can conflict with other devices. The CAN ID can be adjusted in code if required.

The module broadcasts its current state on the CAN bus.  This can be used by aftermarket ECUs or FIS displays to show current status

```
data[1] = standalone_flags (bitmask for Gen1/Gen2/Gen4/Gen5)
data[2] = processed_haldex_engagement (requested by firmware)
data[3] = lock_target_percent (actual lock, returned by differential)
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
> Recommended for quick installation (and removal) — typically **<10 minutes** on Generation 1 Controllers.
> The latest harnesses for Generation 2, 4 and 5 are even simpler and you'll be experiencing your Haldex controller in less than 30 seconds:

- Lift the rear seat
- Split the factory 6-pin Haldex connector
- Install your new harness & OpenHaldexC6 Controller
- Drive it (you can put the seat back down too, if you want!)

> For Generation 1 Controllers:
Route cables as required and connect harness ends as described in the harness instructions.

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

Build this as a **Y-branch** harness between the two VW 6-pin connectors, with a long tail to the MX plug.

Routing summary:

- Pins 1, 2, 3 are pass-through between Vehicle and Haldex.
- **Permanent power (Term30) and ground (MALT) must also be branched to the OpenHaldex controller**:
  - Term30 -> MX Pin 1
  - Ground/MALT -> MX Pin 2
- Chassis CAN is taken from the Vehicle side and sent to the controller:
  - Vehicle Pin 5 -> MX Pin 3 (Chassis CAN Low)
  - Vehicle Pin 6 -> MX Pin 4 (Chassis CAN High)
- Returned CAN from the controller then goes to the Haldex side:
  - MX Pin 5 -> Haldex Pin 5 (Haldex CAN Low)
  - MX Pin 6 -> Haldex Pin 6 (Haldex CAN High)

- Vehicle Connector — `VW 1J0‑973‑813`:

| Pin | Signal | Notes |
|----|------|------|
| 1 | Term15 | Pass-through: Vehicle → Haldex |
| 2 | Ground/MALT | Pass-through: Vehicle → Haldex and branch to MX Pin 2 |
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
| 5 | Haldex Low | From MX Pin 5 (returned CAN) |
| 6 | Haldex High | From MX Pin 6 (returned CAN) |

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

Adding more functionality couldn't be easier - you just need to know the feature you want and grab some CAN data to help implement it.  Using the SavvyCAN interface, you can listen to the CAN messages to see what the car is talking about...

Enable **Analyzer Mode** in the setup menu to capture CAN frames.

> [!WARNING]
> Enabling Analyzer Mode disables active Haldex control and returns the device to OEM 'pass-through' behaviour.

### SavvyCAN Connection

1. Connect to the OpenHaldex Wi‑Fi AP
2. In Settings, enable SavvyCAN

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

- **Forbes Automotive** — Lead development of the OpenHaldex C6 platform, including reverse‑engineering and open‑source implementation for **Gen2, Gen4 and Gen5 Haldex systems**, along with ongoing maintenance of the project
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
