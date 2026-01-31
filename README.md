# OpenHaldex - ESP32 C6
An open-source Generation 1, 2 & 4 Haldex Controller which originates/is a fork from ABangingDonk's 'OpenHaldex T4'.  It has been extended into Gen2 and Gen4 variants, with Gen3 and Gen5 currently unsupported - CAN reads of these systems would be awesome!

Originally based on the Teensy 4.0; the ESP32 C6 features two TWAI (CAN) interfaces as well as WiFi and Bluetooth support; which makes it an ideal candidate for easier interfacing.

![OpenHaldex-C6](/Images/BoardOverview.png)

### Purchase
If you want to purchase an assembled OpenHaldex C6 controller, you can do so here: [OpenHaldex C6 Controller - Forbes Automotive](https://forbes-automotive.com/products/openhaldex-controller)

### Concept
The basis of the module is to act as a middle man - read the incoming CAN frames destined for the OEM Haldex controller, and, if required, adjust these messages to 'trick' the Haldex into thinking more (or less) lock is required.  

Generation 1, 2 and 4 are all available as 'standalone' systems - which means that no other ECUs have to be present and OpenHaldex will create the necessary frames for Haldex operation.

### ESP32 C6
The ESP32 C6 features two TWAI controllers - which allows CANBUS messages to be read, processed and re-transmitted towards the Haldex.  It also supports WiFi and Bluetooth - which makes on-the-fly configuration changes possible.  

The original Teensy had an external HC-05 Bluetooth chip which is limited support wise - hence the decision to move to the ESP32.

### The Modes
The controller allows for 5 main modes: Stock (act as OEM), FWD (zero lock), 7525 (25% lock), 6040 (40% lock) and 5050 (100% lock) at the Haldex differential.  Generation 1, 2 and 4 have been tested on the bench to allow for a full understanding of what the stock CAN messages look like & therefore what messages need to be editted / created. 

These modes are displayed as colours using the 5mm PCB LED: Red (Stock), Green (FWD), Cyan (7525), Hot Pink (6040) and Blue (5050).  Custom modes are white.

#### Custom Modes
Custom modes allows the user to set up 1x specific lock points at either speed, throttle or both ranges.  Future additions will include multiple set points for varying degrees of lock at specific speed/throttle points.

The modes can be toggled with the onboard 'Mode' button, changed via. WiFi or via. CAN.

Disabling at low throttle inputs or high speed inputs are also configurable.

#### Changing Modes via. CAN
The modes can be changed via. CAN if required.  This is always enabled.  

Byte 0 should be sent with the required mode number (noting numbers below).  The remaining 7 bytes are unused (for just now...).

#### Mode Numbers
Current mode (Byte 0) assumes (uint8_t type):
```
  Stock = 0
  FWD = 1,
  5050 = 2,
  6040 = 3,
  7525 = 4,
  Custom = 5,
```
#### Receiving Modes via. CAN
The default CAN address for broadcasting current state is on 0x6B0.  If you enable 'Broadcast OpenHaldex' you may find there are clashes on the CAN network if an existing module is also broadcasting on this access.  This ID can be adjusted to suit.  It is deliberately a 'high' value because the default Bosch IDs have non-important devices set with high value IDs.  

The following data is broadcast onto the CAN network.  This could be used for aftermarket ECUs / clusters where there is a requirement to view current Haldex state:
```
    data[1] = isGen1Standalone + isGen2Standalone + isGen4Standalone;  // if is standalone.  Not used, but here for something to do
    data[2] = (uint8_t)received_haldex_engagement_raw;                 // processed/mapped by FISCuntrol
    data[3] = (uint8_t)lock_target;                                    // the lock % requested
    data[4] = received_vehicle_speed;                                  // the vehicle speed
    data[5] = state.mode_override;                                     // if is in 'override' mode
    data[6] = (uint8_t)state.mode;                                     // current mode (as above)
    data[7] = (uint8_t)received_pedal_value;                           // vehicle pedal value
```
### WiFi Setup
WiFi setup and configuration is always active.  Connect to 'OpenHaldex-C6' by searching in WiFi devices and searching for 192.168.1.1 in a browser.  All settings are available for editing.  Should the WiFi page hang, a long press on the 'mode' button will reset the WiFi connection.

## Installation
> [!TIP]
> ### Optional Plug & Play Harness
> 🟢 **RECOMMENDED — EASY 2 MINUTE INSTALL**
>
> For users choosing the optional plug & play harness, installation is quick and easy.
>
> - Remove the existing 6 pin connector at the differential and route the OEM cable back into the boot space.
> - Install the long end of the harness through the boot floor and connect to the differential. The remaining small end connects back onto the OEM harness (previously fed through the boot floor) and the OpenHaldex module connects to its connector.
>
> Removal is the reverse of installation.

> [!WARNING]
> ### ⚠️ No Harness - Manual Wiring Required
> Note that all OpenHaldex controllers ordered without harnesses come with the pins required for installation.
>
> Users without the optional plug & play harness will be required to make the following connections to Power/Ground/CAN:
>
> #### Pinouts
> The pins for installation (without the plug'n'play harness) is as follows:
> ![InstallationPins-C6](/Images/InstallationPins.png)
>
> The MX23A12NF connector pinout is:
>
> | Pin/ | Signal | Notes |
> |-----|--------|-------|
> | 1 | Vbatt | 12 V |
> | 2 | Ground/MALT | — |
> | 3 | Chassis CAN Low | to Chassis/ECU side |
> | 4 | Chassis CAN High | to Chassis/ECU side |
> | 5 | Haldex CAN Low | to Haldex side |
> | 6 | Haldex CAN High | to Haldex side |
> | 7 | Switch Mode External | +12v to activate |
> | 8 | Brake Switch In | +12v to activate |
> | 9 | Brake Switch Out | Gen1 differentials ONLY |
> | 10 | Handbrake Switch In | +12v to activate |
> | 11 | Handbrake Switch Out | Gen1 differentials ONLY |

### Uploading Code
For users wishing to customise or edit the code, it is released here for free use.  Connect the Haldex controller via. a data USB-C cable (note some are ONLY power, so this needs to be checked).

It is recommended to check back here regularly to find updates, submit bugs or feature requests.

### Over The Air Updates
Over the air updates have been implemented and the process is as follows:

> Download the most recent 'Release' - this will be saved locally on your phone/laptop/etc as a '.bin'.
> Connect to OpenHaldex on WiFi
> Go to 192.168.1.1:81/update
>> Username: admin
>> Password: haldex
> Select file - locate the new '*.bin'
> Wait for ESP to reboot
> Re-connect and check the OTA tab for firmware version - this should match the downloaded Releases file

### CAN Sniffing / SavvyCAN
For a dedicated SavvyCAN interface, users can choose 'Analyzer Mode' in 'Setup'.  This will allow the Haldex Controller to act as a CAN sniffer so that CAN frames can be captured from either bus.  Once Analyzer mode is enabled, ALL Haldex control will be disabled and the system will operate in 'OEM' mode.

#### Connecting SavvyCAN (GVRET)
> 1. Keep the OpenHaldex Wi-Fi AP connected.
> 2. In SavvyCAN: **Add New Device Connection** -> **Network Connection (GVRET)**.
> 3. IP address: `192.168.1.1` (port 23 is implicit in SavvyCAN's GVRET UI).
> 4. Set CAN speed to `500000`.

### The PCB & Enclosure
The Gerber files for the PCB, should anyone wish to build their own, is under the "PCB" folder.  This is the latest board.
Similarly, the enclosures are also here.

>Pinout & functionality remains the same for ALL generations of enclosure.

![OpenHaldex-C6](/Images/BoardTop.png)

![OpenHaldex-C6](/Images/BoardBottom.png)

### Nice to Haves
Flashing LED if there is an issue with writing CAN messages.

### Shout Outs
Massive thanks to Arwid Vasilev for re-designing the PCB!  Now on V1.02!
Massive thanks to LVT Technologies for adding in OTA updates - it's been something I've wanted to do for ages and this guy cracks it without question!  This'll make updating so much easier!  Cheers dude!

> [!CAUTION]
> ## Disclaimer
> ⚠️ **WARNING / DISCLAIMER**
>
> It should be noted that this will modify Haldex operation and therefore can only be operated off-road and on a closed course.
>
> It should always be assumed that the unit may crash/hang or cause the Haldex to operate unpredictably and caution should be exercised when in use.
>
> Using this unit in any way will exert more strain on drivetrain components, and while the OEM safety features are still in place, it should be understood that having the Haldex unit locked up permanently may cause accelerated wear.
>
> **Forbes Automotive takes no responsibility for damages as a result of using this unit or code in any form.**
