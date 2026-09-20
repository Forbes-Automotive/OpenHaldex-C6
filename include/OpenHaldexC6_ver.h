#pragma once
#include <OpenHaldexC6_defs.h>

// Current firmware version
#define FW_VERSION "9.00.0" // web UI now auto cache-busts via %FW_VERSION% (no manual .html edit needed)

/*
Version Control:

*** remember and update FW_VERSION in '_defs.h' ***

V1.00.0 - basic code for testing
V1.01.0 - added in reliable mode changing and eeprom saving
V1.02.0 - confirmed Gen1 (OEM & Standalone), Gen4 (Standalone)
V1.03.0 - confirmed Gen2, updated general codebase
V1.04.0 - added feedback for incoming brake/handbrake sensors.  Added options to invert if required
V1.05.0 - brake out GPIO mapped incorrectly - changed
V1.06.0 - added 6040 split
V1.07.0 - added OTA updates (!) Thanks to Sasha!
V1.08.0 - Sorted crash when long press / WiFi reset
V1.09.0 - revised throttle AND speed setpoints to work together with enable/disable lock - thanks to Chris!
V1.10.0 - added custom mode to allow custom lock percentage (based on speed/throttle/body)
V1.11.0 - added SavvyCAN

V2.00.0 - changed to PlatformIO / VS Code
V2.01.0 - added custom UI with:
        > 'on TC, enable 5050' checkbox & 'on ext. button hold, enable 5050' checkbox
        > expert editor with 7x7 array of speed/throttle/lock
V2.02.0 - added ECU speed stale, use ABS, confirmed TC flag (Bremse_1)
V2.03.0 - added speed at Bremse_3 and 'Restore Defaults' clearing onhaldex binary state Expert without refresh.  
V2.04.0 - added Force Mode (rather than fixing lock at 100% when TC active), changed Speed/Throttle/Lock Axis text
V2.05.0 - added Lock Target to Stock - this ensures that a % is achieved when TC/ExtBtn is enabled
V2.06.0 - added Haldex State in binary to better understand the state of the Haldex 
V2.07.0 - copied actual lock to requested lock in stock mode so it looks 'cleaner'
V2.10.0 - release
V2.10.1 - fixed the CAN health - always showed Unhealthy because it wasn't updating the flag

V3.00.0 - added support for Gen5 and Gen4 GM/SAAB Gen4 (but logged under 41 as in 4.1 since it's a variant of Gen4)
V3.00.1 - added 'Disable Onboard Button' and 'Disable External Button' options to allow disabling of the onboard button & added Learn Haldex function

V4.00.0 - added support for Gen41 dual-bus (late Insignia) - standalone frames, plus feedback decoding per GMW8762 PPEI
V4.00.1 - added low power mode to disable WiFi AP when no CAN traffic for 5+ minutes and no clients connected, to reduce power consumption when used in a parked car. WiFi restarts automatically when CAN traffic returns.
V4.00.2 - added Password Protection

V5.00.0 - added 'Fix Hunting' option to switch Motor_11 to BPK packing, which can help with hunting at partial lock on 554K controllers

V6.00.0 - added interrupt-based CAN handling, revised Hazard force mode handling, added LED brightness control, adjusted sleep configuration for low power mode 
V6.01.0 - added WiFi naming and better sleep configuration for low power mode, added 'Follow Hazard' force mode option, added 'Disable Hazard Force Mode' option to allow disabling of hazard force mode when the hazard switch is active, added 'Hazard Force Mode Source' option to select which CAN signal is used to trigger hazard force mode (Blinkmodi_02 vs GATEWAY_72)

V7.00.0 - added support for Gen5 (0AY) Haldex - a mix of Gen2 and Gen4 (but ultimately Gen5 chassis).

V8.00.0 - added support for Ford (based on example CAN data, totally untested!)
V8.00.1 - added fix for password and SSID change

V8.00.2 - added fix for TC/hazards not re-enabling stock mode

V8.00.3 - added drop-down options for adding/removing CAN signals if learn isn't 'clean'
        - when this was first developed the frames that 'changed' the Haldex response were ported to the non-standalone version
        - but there could be room for some 'additional'.  This allows the user to add additional frames to mirror standalone 
        - fixed bus recovery (would not recover...)
        - minor UI tweak so that force modes display better (single line)
        - added TP2.0 (ported from Can2Cluster) / VCDS logged (1K0 554C)
        - added new scaling for UDS - 0CQ 554C/D - proven on bench and logged with VCDS
        - minor lock tweaks on 0CQ to target 100% cleaner

        V8.00.4 - shared Forbes Automotive UI theme; added an OTA tab (safety-gated
          /ota endpoints kept); automatic product web-asset cache-nosave.

        V8.00.5 - Long Learn (Settings): automated frame-block learning:
          all blocks on, (Gen5) Launch PWM Floor stepped until the learn is smooth, then each
          additional block removed one at a time (any effect = kept on, else off),
          confirmation learn on the final set, live tracker, chassis notes and a
          .txt report export. Manual Learn now shares the same sweep code.
        - fixed Gen5 (0CQ VAQ, gen 52) being rejected by the settings API and
          skipped by the normal-mode frame editor (frames never ran for VAQ).
        - Reset-to-Defaults confirmed (0CQ default keeps the
          8.00.3 Motor_14/ESP_07 opt-in, not the V7 10-block set).
        - OTA tab: the GitHub "Check for Updates" flow was tried and dropped.
          Phones won't reliably keep mobile data while joined to a WiFi with no
          internet, so a page-driven download from GitHub can't be relied on.
          OTA is the plain Can2Cluster-style card: user downloads littlefs.bin +
          firmware.bin from Releases/ themselves, uploads filesystem then
          firmware. Releases/releases.json and the index half of
          tools/make_release.py removed; /ota endpoints unchanged.
        - AP no longer hands out a default gateway/DNS (local-only network).
        - startSoftAP() now reports the address the AP actually came up on
          instead of a hardcoded "192.168.1.1", and logs a rejected softAPConfig.
        - OTA rollback protection now real: verifyRollbackLater() defers the
          core's auto-confirm; image confirmed once the web UI is reached or
          after 60s uptime, else the bootloader reverts on next reset.
        - fixed /ota/update/fs being captured by the /ota/update handler (route prefix
          match) - filesystem uploads went to the firmware handler; removed the
          (never-enforced) OTA basic-auth.
        - Gen2/Gen4 (PQ) handbrake now decoded from CAN: Kombi_1 (0x320) byte 1
          bit 1 (KO1_Handbremse per PQ35/46 K-matrix). Diag "Handbrake (CAN)"
          reports it for Gen2/4/51 (was Gen5 only) and Follow/Invert Handbrake
          rewrites that bit on the forwarded frame. Brake stays on Motor_2 MO2_BLS.
        - "Disengage Under/Above Speed" + "Minimum Throttle" now gate EVERY lock
          path: force-mode triggers (TC/hazard/ext button) previously bypassed
          the gate in get_lock_target_adjustment() so lock_target read 100% below
          the cut-off; Expert mode previously bypassed it entirely.
        - Ported GitHub PR #39 (louij2, "Add WiFi bridge mode, config
          backup/restore, and bench mode", written against 8.00.3):
          > Backup & Restore (Diagnostics tab + tools/openhaldex_config.py):
            export/import the Expert tune, steering scale, frame edits, all
            settings and the WiFi names as JSON. Passwords are write-only on
            the device and never in the file; import asks for them once.
            Key list widened to everything the 9.x /api/settings accepts.
          > WiFi bridge mode: optionally join a home/garage network as a STA
            alongside the AP (WIFI_AP_STA) so the controller is reachable on
            that LAN. GET/POST /api/wifi/sta, /api/wifi/sta/reset, GET
            /api/wifi/scan. Changed from the PR: the scan is asynchronous
            (a blocking scan sat inside the async_tcp task), and STA retries
            back off - 20 s of auto-reconnect after start, then one attempt
            per 5 min - so a saved home SSID can't keep pulling the single
            radio off the AP's channel while the car is away from home.
          > Bench Mode (Settings): holds WiFi up with no harness connected;
            self-clears the moment either bus shows traffic this power cycle,
            and the UI locks the toggle while CAN is detected.
          > Bug fix from the PR: "Enable CAN Sleep" only ever gated the CPU
            frequency scaling, never the WiFi shutdown in updateTriggers(),
            so switching it off did nothing visible. Now gates both.

*/


/*

** to do **:
        > add throttle/speed axis refresh
        > add 'ota' to match existing layout
        > add reduction in throttle/speed off
        > move CAN into interrupt based - ESP_INTR_FLAG_IRAM
*/