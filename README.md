# OpenHaldex — ESP32-C6

An open-source controller for Haldex systems (Generation 1, 2 & 4). OpenHaldex reads CAN frames from your vehicle and can modify or generate messages so the Haldex differential behaves as configured.

Features:
- Two TWAI (CAN) interfaces for reading and bridging CAN traffic
- Built-in Wi‑Fi and Bluetooth for on‑device configuration and diagnostics
- Multiple preset modes plus customisable mode profiles
- OTA firmware updates

![OpenHaldex-C6](/Images/BoardOverview.png)

Purchase
- Assembled modules are available: [OpenHaldex C6 Controller - Forbes Automotive](https://forbes-automotive.com/products/openhaldex-controller)

Overview
OpenHaldex sits between your vehicle and the OEM Haldex controller. It can operate as a passthrough (OEM behaviour), or modify messages to request different amounts of differential lock.

Supported generations: Gen1, Gen2 and Gen4 (Gen3 and Gen5 are currently unsupported).

Hardware
The PCB is based around an ESP32‑C6 Mini (with Wi‑Fi, Bluetooth).  Two TWAI/CAN controllers are built into the PCB, along with external IO control (external mode button/onboard RGB LED and brake/handbrake signals).  There are two high-side drivers for brake/handbrake but these could be repurposed for other functions.
- This platform was chosen to replace the earlier Teensy design for better wireless support and on‑device configuration.

Modes
The controller provides preset modes and a custom mode:
- Stock (OEM behavior)
- FWD (zero lock)
- 7525 (25% lock)
- 6040 (40% lock)
- 5050 (100% lock)
- Expert (user-defined lock profile)

LED indicators (5mm onboard LED):
- Red — Stock
- Green — FWD
- Cyan — 7525
- Magenta — 6040
- Blue — 5050
- White — Expert

Expert Mode
The Expert modes let you set lock targets based on speed and throttle setpoints and is available as a table. 

Changing modes
- Onboard: press the `Mode` button
- Wi‑Fi: use the Web UI (at 192.168.1.1)
- CAN: send a message with the Mode number in Byte 0 (other bytes unused)

Mode numbers (byte 0, uint8_t):
```text
Stock = 0
FWD = 1
5050 = 2
6040 = 3
7525 = 4
Expert = 5
```

Broadcasted state (default CAN ID: `0x6B0`)
> Note: Broadcasting can conflict with other devices — the ID can be adjusted.

The module broadcasts its state on the CAN bus. The layout (data[1]..data[7]) is:
```text
data[1] = standalone_flags (bitmask for Gen1/Gen2/Gen4)
data[2] = processed_haldex_engagement (mapped by firmware)
data[3] = lock_target_percent
data[4] = vehicle_speed
data[5] = mode_override_flag
data[6] = current_mode_number
data[7] = pedal_value
```

Wi‑Fi Setup
1. Connect to the access point `OpenHaldex-C6`.
2. Open a browser at `http://192.168.1.1` to access the web UI.
3. If the Wi‑Fi page becomes unresponsive, long‑press the `Mode` button to reset the Wi‑Fi.

Installation
Optional Plug & Play Harness (recommended)
- Quick install — typically 2 minutes. Route cables as needed and connect harness ends as shown in the harness instructions.

Manual Wiring (no harness)
- Modules sold without a harness include the connector pins for manual wiring.
- Haldex Connector (VW 1J0-973-713)
- Vehicle Connector (VW 1J0-973-813)

- Image: ![InstallationPins-C6](/Images/InstallationPins.png)

MX23A12NF connector pinout:
| Pin | Signal | Notes |
|-----:|:-------|:------|
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

Uploading Code
- Connect the module via a data USB‑C cable (some cables are power‑only — confirm data support).
- Source is open — check the repository for updates, bug reports and feature requests.

Over‑The‑Air (OTA) Updates
1. Download the latest release from GitHub (in 'Releases') (`.bin`).
2. Connect to OpenHaldex Wi‑Fi.
3. Search: `http://192.168.1.1:81/update`
   - Username: `admin`
   - Password: `haldex`
4. Upload the `.bin` file and wait for the device to reboot.
5. Reconnect and verify the firmware version in the OTA tab.

CAN Sniffing (SavvyCAN / GVRET)
- Enable `Analyzer Mode` in the Setup menu to capture frames from either CAN bus. Note: enabling analyzer mode disables all active Haldex control and returns the device to OEM mode.

SavvyCAN (GVRET) connection steps:
1. Keep the OpenHaldex Wi‑Fi AP connected.
2. In SavvyCAN: Add New Device Connection → Network Connection (GVRET).
3. IP: `192.168.1.1` (port 23 is used by GVRET by default).
4. Set CAN speed to `500000`.

PCB & Enclosure
- Gerber files and enclosure designs are in the `PCB` folder in the repo.
- Pinout and functionality are consistent across supported enclosure versions.

![OpenHaldex-C6](/Images/BoardTop.png)
![OpenHaldex-C6](/Images/BoardBottom.png)

Nice to Have
- Flashing LED if there is an issue writing CAN messages.

Acknowledgements
- Thanks to Arwid Vasilev for the PCB redesign (now V1.02).
- Thanks to LVT Technologies for integrating OTA updates.

Disclaimer
> This device modifies Haldex behavior and should only be used off‑road or on a closed course. The unit may behave unpredictably and could increase drivetrain wear. Use at your own risk. Forbes Automotive is not responsible for damages resulting from use of this device or software.
