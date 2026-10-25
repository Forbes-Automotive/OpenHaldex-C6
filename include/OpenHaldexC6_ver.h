#pragma once
#include <OpenHaldexC6_defs.h>

// Current firmware version
#define FW_VERSION "11.00.1" // also bump data/version.json ("fs")

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

V9.00.0 - shared Forbes Automotive UI theme; automatic product web-asset
          cache-nosave; OTA tab (see below).
        - Long Learn (Settings): automated frame-block learning:
          all blocks on, (Gen5) Launch PWM Floor stepped until the learn is smooth, then each
          additional block removed one at a time (any effect = kept on, else off),
          confirmation learn on the final set, live tracker, chassis notes and a
          .txt report export. Manual Learn now shares the same sweep code.
        - fixed Gen5 (0CQ VAQ, gen 52) being rejected by the settings API and
          skipped by the normal-mode frame editor (frames never ran for VAQ).
        - Reset-to-Defaults confirmed (0CQ default keeps the
          8.00.3 Motor_14/ESP_07 opt-in, not the V7 10-block set).
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
        - OTA tab (safety-gated /ota endpoints kept): "Check for updates" from
          GitHub. An earlier attempt was dropped because phones won't keep
          mobile data on a WiFi with no internet; bridge mode fixes that by
          giving the phone a route to the internet AND the controller at the
          same time (controller on the home router, phone on the same network).
          Every check pings /ota/info first - no GitHub fetch unless the
          controller is reachable - and a no-internet result reads
          /api/wifi/sta to say exactly what to do: join the network the
          controller is already on (with its address), fix a configured-but-
          dropped link, or set one up in the Home WiFi card, which is
          duplicated on the OTA tab (initWifiSta() takes an id prefix). Retry
          button + "Set up Home WiFi" jump.
        - Release list = Releases/releases.json 
        - Rollback allowed (tick "Show
          beta / older versions"); the confirm warns that older releases may
          not have this page. Two routes only: GitHub, or "Update from Files"
          (littlefs.bin then firmware.bin, the original safety-gated upload
          card). A content-sniffing multi-file picker was tried and dropped
          as noise.
        - Check button reports each stage with elapsed time (1/2 controller,
          2/2 GitHub) so a hang is distinguishable from a dead link.
        - ROOT CAUSE of OTA never completing (firmware or filesystem): both
          upload callbacks answered "200 OK" on every non-final chunk. The
          browser took the first one as the end of the upload and closed the
          connection while the body was still streaming - AsyncTCP then used
          the freed pcb (Guru Meditation in tcp_output, Load access fault)
          and a partly-written partition was left behind. Upload callbacks
          now never send(); the outcome is recorded (first one wins) and the
          request handler sends it once the body has ended. Firmware reboot
          moved there too, after the response.
        - Filesystem OTA hardened after a failed update boot-looped a unit:
          LittleFS is unmounted before the partition is rewritten (it used to
          stay mounted underneath the write); any failure - write error,
          client gone, short body (?size= from the uploader), SHA mismatch,
          image that won't mount - erases the superblock pair so a hybrid of
          two images can never be handed to lfs (CONFIG_LITTLEFS_ASSERTS=y
          + panic-reboot = boot loop). Boot only mounts after a superblock
          sanity check, and the web server ALWAYS starts: with no usable UI
          "/" is a built-in recovery page with the two upload forms. A
          filesystem update never reboots; the firmware runs from ota_x and
          does not need the filesystem.
        - Upload-stream SHA-256 gate removed: the chip validates firmware
          (esp_ota_end) and the filesystem is validated by mounting; the gate
          only ever failed updates when a .bin was rebuilt in place without
          re-running make_release.py. GET /ota/fsdiag reports what is
          physically in the filesystem partition (superblock fields, mounted,
          files, read-back sha256) - also shown on the recovery page.
        - Low-power AP shutdown now also holds off while a browser is polling
          the UI (otaWebClientActive(): /api/dashboard, /api/wifi/sta, /ota/*
          within 30 s, or an upload in progress). Before, only stations joined
          to our own AP counted, so a phone working through the home router on
          the bench (Bench Mode off) could have WiFi cut from under it mid-OTA.
        - app.js/style.css now served with Cache-Control: no-cache (ETag
          revalidation, 304 when unchanged) instead of max-age=1y + a hand-
          bumped ?v= that kept being forgotten - phones were running a stale
          app.js against new HTML, so buttons on new cards did nothing.
        - fixed saving the Haldex generation (dropdown or backup restore)
          overwriting the saved drive mode with the generation number
          (lastMode = generation) - Gen1/41/5x then booted in FWD.

V9.00.5 - 9.00.4 (VAQ + Themes + ESP-NOW) merged with the GitHub contributor PRs:
        - #39 WiFi bridge mode / config backup tool (already ported in 9.00.4)
        - #40 CLI: accept the old _fw_version backup key (not needed by our tool)
        - #43 OTA tab: Stable / Latest build update channel (web UI only)
        - #44 Bluetooth LE for the DashCAN app (NimBLE) + lock release
          switch/rate now actually save. PARTITION CHANGE: app slots
          0x1A0000 -> 0x1C0000, LittleFS 0xB0000 -> 0x70000 (at 0x390000).
          Needs ONE USB flash from any older build - cannot go over OTA.
        - #42 Vehicle Geometry card (wheelbase / tracks / steering ratio)
          for the per-corner slip calc, persisted, in backups. Defaults TT Mk3.
        - #41 favicons, PWA manifest, home-screen icons. The six iOS splash
          screens were left out: with them the UI (~145 blocks) no longer
          fits the 0x70000 LittleFS (112 blocks) that #44 introduced.
        - ESP-NOW and BLE now run side by side on the one radio - untested
          together on hardware.

V9.00.6 - GitHub main after 9.00.5, plus a toolchain update:
        - #45 Stock no longer feeds the reported Haldex engagement back as
          the lock command (positive feedback loop that latched high lock
          until FWD / power cycle). Stock now commands zero forced lock, and
          inline Stock (base mode or force trigger) forwards frames untouched.
        - #46 when a bus goes silent (1 s health timeout), zero the latched
          inputs it feeds: speed, pedal, ESP/hazard force flags, ABS valid
          (chassis) and reported engagement (Haldex). Boot state unchanged.
        - #47 default VAG correction factor (no learn table) now matches its
          comment: CF = (target + 20) / 2, was target / 2 + 20 (+10 CF).
        - #49 line endings normalised to LF (no code change).
        - platform pioarduino 54.03.20 -> 55.03.312; "Async TCP" added to
          lib_ignore so only mathieucarbou/AsyncTCP is linked. 55.03.312
          needs PlatformIO Core >= 6.2.0 (older Core refuses the platform).
          Arduino core 3.3.12 / IDF 5.5.5. Flash 92.5% of the 0x1C0000 app
          slot; littlefs.bin fills the 0x70000 partition exactly.
        - Boot-loop fix: if NimBLEDevice::init() fails (C6 controller logs
          "ble ll env init error code:-13"), bleStart() now bails out and BLE
          stays off until reboot. Previously it carried on into getNumBonds()
          on an un-started host -> NULL mutex -> panic ~1 s after WiFi up.
          Free internal heap is logged just before BLE init.
        - Root cause of both the BLE -13 and the web UI serving broken pages:
          internal heap. Measured 23.7 KB free / 14 KB largest block before
          BLE init (IRAM +28 KB vs the pre-BLE build on the newer core, plus
          static RAM growth). Freenove WS2812 lib dropped - it hard-codes a
          256-LED RMT buffer (24 KB static) for our one LED; replaced by
          OneLed (include/OpenHaldexC6_led.h) on the core's
          rgbLedWriteOrdered(), same API/colour order/brightness behaviour.
        - debugMemory (defs.h): heap + every task's stack high-water mark,
          15 s after boot then every 60 s, to right-size task stacks.
        - Task stacks sized from debugMemory: 10 frame tasks 8000/4000 ->
          2560 (used ~250-450 B), AsyncTCP 16 KB -> 8 KB; updateTriggers
          2000 -> 3072 and broadcastOpenHaldex 1000 -> 2048 (432 / 644 B
          left - near overflow). With the LED change, BLE now starts (48 KB
          free at init) but left only 10 KB, so the web UI still died on
          page load; this frees ~44 KB more.
        - Long Learn reworked around "prove all-on, then test each static block":
          core = lock-driven blocks (frameEditLockDriven, always sent); all-on
          must give a smooth 100% sweep or the run stops; two all-on reads at
          CF 20/40/70/100 give the reference + noise threshold; each static
          block is then tested OFF ON ITS OWN at those points (was: cumulative
          removal, each block judged against a shrinking set); full
          confirmation sweep with all no-effect blocks off together, else
          everything left on (interaction flag). ~6-8 min on Gen5. Gen5
          BPK Adjust kept (hunting -> Fix Hunting + ceiling walk), result kept.
        - #51 canBusRecovery() now restarts ANY stopped controller (was only
          one it had put into bus-off recovery itself). Light sleep can power
          down the TWAI domain (sleep_allow_pd), leaving a controller stopped:
          no frames to the Haldex, and in non-aggressive low power the wake
          check never saw the car come back.
        - #51 with CAN sleep enabled, an ESP_PM_NO_LIGHT_SLEEP lock is held
          while awake; updateTriggers releases it only when the low-power
          state machine deliberately sleeps and takes it back on wake, so
          automatic light sleep can't power down CAN with the car running.

V9.00.7 - relay the car's CAN frames to gauges over ESP-NOW:
        - New "Relay CAN Frames to Gauges" setting (canRelay, default Off) under
          Controller Options: Off / every frame / each ID up to 50, 20 or 10
          per second (29-bit IDs never capped). In the backup keys.
        - Every bus-0 frame is tapped in parseCAN_chs before any brake /
          handbrake edit, so gauges see the frame as the car sent it; a
          can2gauge with power only (no CAN wire) then decodes the car.
        - Up to 24 frames per 250-byte packet, sent when full or 10 ms after
          its first frame; broadcast peer at 6 Mbit/s while relaying.
        - Shared files include/cannow_proto.h + cannow_tx.h (identical in
          can2gauge / adc2can). Relay stats in the status JSON (canRelay).
          Not yet tested on hardware.

V10.00.0 - (beta, 2026-10-18) new market: Volvo and SAAB. Every new market gets a
          new major number (V11 = MQB Evo).
        - Four new Haldex generations, standalone only for now:
            21  Generation 2 - Volvo P2 (Pre-Facelift)  250 kbit/s, 29-bit IDs
             3  Generation 3 - Volvo P2 (Facelift)      500 kbit/s, 29-bit IDs
            43  Generation 4 - Volvo (P3)               500 kbit/s, 11-bit
            44  Generation 4 - SAAB 9-3 XWD             500 kbit/s, 11-bit
          IDs, send rates, counters and the lock / "lock active" bytes are a
          starting point; the Volvo network heartbeat (bytes 6-7 swap to their
          opposite every 100 ms) matches a real Volvo trace. Static bytes are
          fixed starting values, NOT decoded yet: tune with the serial lab OVR
          command + SavvyCAN, then decode from our own logs.
        - Haldex replies decoded for all four (provisional 0-255 -> 0-100 %
          until logs give the scaling); SAAB also gives a temperature
          (0x210 byte 2 - 40) shown as haldexTempC in /api/status.
        - CAN bus speed now follows the generation: Volvo P2 Gen2 runs both
          buses at 250 kbit/s (why a P2 user saw no Haldex health at 500k).
          Changing to or from it saves the generation and restarts the
          controller ~2 s later; /api/settings reports canSpeedKbps.
        - Lock scaling for Volvo / SAAB is straight-through (100 % = the
          strongest known lock value) until Learn is run.
        - The VW chassis decoder (speed / pedal / brake from 0x280, 0x288,
          0x320 ...) is skipped on Volvo / SAAB, where those IDs mean other
          things. Inline (non-standalone) Volvo / SAAB passes frames through
          untouched.
        - Chassis decode for the non-VW cars (new OpenHaldexC6_ChassisOther.cpp),
          so Expert mode, the speed / throttle limits and brake / handbrake
          follow work on them:
            41 Insignia  speed 0x34A rear wheels /32, pedal 0x0C9 b4, RPM 0x0C9,
                         brake 0x0F1 b0 bit1, handbrake 0x1F1 b4 bit4
            42 Focus RS  speed / RPM / pedal 0x201, brake 0x205 b2 bit6,
                         handbrake 0x433 b3 bit0
            43 Volvo P3  speed 0x148, pedal 0x020, brake + RPM 0x12A (no
                         handbrake yet); from the V60 DBC, not checked on a car
            44 SAAB 9-3  RPM 0x110, speed 0x280 (four wheels), handbrake 0x380
                         (no brake / pedal yet)
            21 Volvo P2  2002 model-year IDs only: speed 0x00D28026, RPM
                         0x0032C01E, brake 0x0012C024, pedal 0x00E00020 b3
                         (19 rest .. 187 floored); checked against public 2002
                         S60 logs (IDs change by model year)
        - SAAB 0x110 bytes 1-2 are engine RPM: the "lock active" value sent
          there is 2000 rpm (comment corrected, behaviour unchanged).
          41 / 42 replayed against our own labelled logs. The VW decoder no
          longer runs on 41 / 42, where foreign frames could be misread.
        - Volvo Gen4 0x12A is EngineData: the "lock steps" are engine RPM
          values (comments corrected, behaviour unchanged).
        - Serial lab OVR now takes 29-bit IDs (override slot ID widened to
          32-bit), and no longer writes a VW CRC into SAAB's 0x110.
        - Web UI: "Generation 3 - Volvo" option (was there but rejected by the
          firmware) now works; GM option renamed "Generation 4 - GM (Vauxhall /
          Opel)" so it isn't confused with the 9-3.

V10.00.1 - (beta, 2026-10-19) the car's drive mode select can force a mode (MQB):
        - New "Use Drive Mode to Force Mode" setting (driveModeForce, default
          Off) under Mode Options, with one drop-down per drive profile.
          Defaults: Comfort = Stock, Sport = 60:40, Race = 50:50,
          Individual = Expert, everything else N/A (left alone).
        - The profile is read from Charisma_07 (0x3E8) byte 1 bits 0-3
          (CHA_Current_Mode); if a car doesn't send that, the engine request
          in Charisma_01 (0x385) byte 2 is used instead (can't see Individual).
          Our own MQB logs read 7 = Individual. Gen5 0CQ / 0AY MQB / VAQ only.
        - Lowest priority: TC / hazards / external button still win. No
          Charisma frame for 3 s = no drive mode force.
        - Profile and mode numbers from the MQB K-matrix and jilleb/MQB-FPA
          (credited in the README).
        - The header banner shows a drive mode force, e.g. "Expert (Sport>60:40)".
        - New Drive Mode card on Diagnostics: the car's profile, where it was
          read from, what the gateway asks the Haldex / VAQ for, what the
          Haldex reports back, and what's being forced.
        - Haldex state legend fixed: the web page read the state byte as hex
          text, so 0x12 showed as 0x18 on every generation. Gen5 now decodes
          Allrad_03 byte 3 as mode (bits 0-3) + status (bits 4-5) with names,
          and the VAQ (Gen52) gets its own (Quersperre_03 byte 3, bits 1-4 / 5-6).
        - Force Mode Priority now saves: the web page sent it, but the firmware
          never read it, so it was always Hazard > TC > Ext after a reload.
          Also in the backup file now.

V10.00.2 - (beta, 2026-10-21) parts of PR #54 (Luca C):
        - Dashboard says why lock is held at 0%: when Disengage Under / Above
          Speed or Minimum Throttle Before Lock is cutting lock, a line under
          Haldex Engagement names the setting and its value. Lock behaviour
          unchanged; lock_enabled() is now built on lock_gate_blocking(), and
          the dashboard sends lockGated + lockGateReasons. Uses the mode in
          use (force triggers / drive mode win), so Stock / FWD never show it.
        - OTA tab no longer offers releases that can't fit this partition
          layout (9.00.0 and older need a USB flash since 9.00.5).
        - tools/can_capture.py: record and compare CAN traffic from a laptop
          through analyzer mode (GVRET, port 23).

(2026-10-22: renumbered again. V11 = this OEM-spec rework; the MQB Evo / CAN FD
 line that was 11.00.x becomes V12.)

V11.00.0 - (beta, 2026-10-22) every generation brought in line with what the OEM
          Haldex code actually reads and reports (documents/haldex-binaries/
          CONTROL_AND_DTC_SPEC.md + leaf_trace/spec_<unit>.md), OEM and Standalone.
          Built on 10.00.2.
        - Shared:
          > OEM mode, every non-Stock mode: the job is to achieve the lock %. The
            OEM-code work confirmed which bytes do that, so the OEM defaults are
            the lock-driving blocks only: the unit's lever byte + the bench-proven
            lock-scaled wheel speeds (+ Motor_1 demand where used). Stock is
            untouched passthrough. Frames that only carry flags the unit doesn't
            need for lock (Bremse_1 ABS bits, steering, gear) pass through.
          > Direct-lever helpers lever_lock_percent() / vag_lever_byte(): Bremse_4
            byte 0 = 0x7F at 0 % (valid "open"), 0x80 + p x 1.25, the unit's own
            full value at 100 %. Before, 100 % sent 0x7F (clutch OPEN) on Gen4 and
            partial lock sent < 0x7F, which the units ignore.
          > ABS braking decoded (PQ Bremse_1 b0.2, MQB ESP_21 b7.0). New "Follow ABS"
            setting (default ON, Output Options, in backups): opens the clutch while
            the ABS is braking (new lock gate "abs") - our lock request outranks the
            unit's own ABS release, so we do it for it. Off = our lock overrules the
            ABS, and standalone sends "not braking".
          > OEM mode, Gen1/2/4/5: a brake / handbrake switch wired to our input pin
            is ORed into the car's own bit (Motor_2 / Kombi_1, ESP_05 + CRC /
            Kombi_01) when Follow Brake / Follow Handbrake is on - for cars with
            a physical switch fitted.
          > Follow Brake / Follow Handbrake (renamed from "... Input"): the bits are
            passed through in OEM mode and now put into the Standalone frames;
            Follow Handbrake also opens the clutch while the handbrake is on (new
            lock gate "handbrake"). Brake / handbrake / ABS from CAN cleared when
            the chassis bus goes quiet.
          > Haldex health decoded per generation (fault, plausibility, over-temp,
            limp, AWD lamp, temperature, temp-sensor fault, fault stored) in each
            generation's own case of parseCAN_hdx - the old shared PQ bit decode
            gave Gen1 a false "limp". Dashboard: red "Haldex Warning" card + new
            Haldex Data tiles; /api/dashboard: limpMode, awdWarning,
            tempSensorFault, faultStored, haldexTempC (all gens), healthSupported,
            haldexWarnings, absBraking. Diagnostics: "ABS Braking (CAN)".
          > "Invert Brake / Handbrake Input" now only flips the input WIRE. In OEM
            mode it used to flip the car's own CAN bit too, so the Haldex saw the
            handbrake on while it was off.
          > One-off settings upgrade (EEP.cpp migrateSettingsToV11, key "v11Mig"):
            OEM masks still on the old default move to the new one, and every
            lock-driving block is switched on in customised masks (a VAQ keeps
            its saved mask); standalone masks get the newly-needed frames switched on
            and unread frames off; saved learn table cleared on Gen1/2/4/0CQ/0AY
            because the lever changed - RUN LEARN AGAIN after updating.
        - Gen1 (02D):
          > Bremse_4 0x2A0 byte 0 torque ceiling is the lever on its own scale
            (gen1_lever_byte: 0x7F open, 0x81-0xC9, 0xFD full - 0xCA+ is already
            full on the 02D) with Motor_1 held at max demand.
          > Standalone now sends Motor_2 0x288 (fixes the permanent false "limp").
          > rpm no longer scaled with lock (pump cut off below 400 rpm): OEM passes
            the real rpm, standalone fixed 0x4E21. 50:50 loss torque 0x00.
          > 0x2C0 decoded only on ID 0x2C0, no more 255 % at raw 0x7F; bit 2 / 4
            are status (ceiling active / inputs incomplete), not faults.
          > OEM Bremse_1 edit keeps the ABS clutch-release and brake bits.
          > New blocks 4 Bremse_4, 5 Motor_2, 6 Kombi_1. Defaults OEM 0x19 (Motor_1,
            Bremse_3, Bremse_4), SA 0x3D.
            Kombi_1 handbrake now decoded from CAN on Gen1.
        - Gen2 (7H0 / 1K0):
          > Engagement read from 0x2C0 byte 1 (byte 4 is never written - it always
            showed 0 %, so Learn could never work on Gen2). Health per OEM code:
            over-temp, fault + lamp, coupling open.
          > Bremse_4 byte 0 is now the linear lever (0x7F open .. 0xFD full) - it
            was written as all zeros. Motor_1 held at full demand with loss torque
            0 (they used to cancel); OEM rpm passed through.
          > OEM edits keep the car's ABS/ESP, brake and handbrake bits and never
            lift the ABS module's own 0x2A0 limit.
          > Standalone Kombi_1 sent "handbrake pulled" all the time - now released
            (or the real handbrake with Follow Handbrake). 0x4A8 alive counter
            changes every frame; Motor_5 no longer bumps Bremse_1's counter.
          > Defaults OEM 0x31 (Motor_1 + Bremse_3 + Bremse_4), SA 0x1E7D.
        - Gen4 (0BR / 0BS / 2K):
          > Bremse_4 lever 0x7F / 0x81-0xFC / 0xFE in OEM and standalone - Expert
            100 % and forced 50:50 sent 0x7F = clutch OPEN. Learn sweeps the lever.
          > Standalone adds Motor_2, Motor_5, Bremse_5, Bremse_8, Getriebe_1 (blocks
            12-16) - a missing Motor_5 made the unit report limp by itself.
            Motor_1 clutch-pedal bit fixed; steady Motor_1 and four equal steady
            wheel speeds (real speed when chassis CAN is there).
          > Feedback: engagement 0x80-0xFD = 0-100 %, temperature (b4 - 60),
            temp-sensor fault, fault stored, real health bits.
          > Defaults OEM 0x32 (Motor_1 + Bremse_3 + Bremse_4), SA 0x1F9FF
            (Bremse_11 / Kombi_2 off).
        - Gen5 0CQ (+ VAQ where the spec allows):
          > ESP_14 Min = Max = 0-250 (exact request). Full lock used to send
            0xFB-0xFE = "no request", so the unit fell back to Motor_11.
          > OEM edits keep the car's real gear, brake pressure, ABS and handbrake
            bits; Motor_12 is a steady 2000 rpm and Motor_11 V3 b6/b7 fixed.
          > Standalone: steady 2000 rpm Motor_12, wheel speeds no longer follow
            lock, Motor_11 V3 b6/b7 fixed, ESP_02 0 g, LWI angle 0, Motor_Code_01
            factor + CRC fixed, ESP_07 counter wrap fixed; brake (ESP_05), handbrake
            (Kombi_01) and ABS (ESP_21) bits from the Follow settings / car ABS.
          > Allrad_03 status 2 and 0x17F0000F report a Haldex fault (clutch
            released) - was shown as "closed". ESP_14-accepted bit decoded
            (/api/dashboard leverAccepted). 0xFE no longer shows 101 %.
          > Defaults OEM 0x1D (ESP_19 lock-scaled wheel speeds, Motor_12, Motor_11,
            ESP_14; a VAQ keeps its saved OEM mask on upgrade). SA stays all-on: the
            0CQ only needs 0x0FE0FB5F but the VAQ shares the mask. Launch PWM Floor
            no longer used.
        - Gen5 0AY (a PQ unit, not MQB):
          > Bremse_4 lever 0x7F / 0x81-0xFC / 0xFD (0xFE = HOLD on the 0AY);
            100 % used to send 0x00, which the unit ignores.
          > Engagement on the unit's own scale (99 -> 100 snap removed);
            temperature, temp-sensor fault, fault stored decoded.
          > Standalone: steady Motor_1, equal wheel speeds, new Getriebe_1 0x440
            (block 18); Bremse_8 / Bremse_11 / Kombi_2 off (not received);
            Bremse_2 / Bremse_5 values corrected; Motor_5 counter fixed.
          > Defaults OEM 0x06 (Bremse_3 lock-scaled + Bremse_4), SA 0x6BF7F. Kombi_01 0x30B rewrite
            removed (the 0AY doesn't receive it).
        - Volvo:
          > P2 Gen3: lock via lever_lock_percent() (0..0xBC, all gates applied);
            real-car rolling counters on 0x0042406C / 0x00224024 / 0x0080401E;
            standalone adds 0x00C0402A, 0x10E24000, 0x11220028, 0x11420006 from a
            500 kbit/s S60 log (vehicle-logs-public/volvo-CAN-exploration).
            Feedback 0x1020000A decoded from the OEM signal table: byte 6 =
            measured clutch value (100 % = VOLVO_P2G3_ENGAGEMENT_FULL_SCALE, 150 -
            bench-check), byte 7 bits 6-7 fault / limp, bits 2-3 input monitors.
            Bus confirmed 500 kbit/s from the code (C1BTR 0x1641 at 20 MHz); the
            Volvo SPC (gen 53) is also 500k (FlexCAN_0, 8 MHz crystal).
            0x01A2402A content still needs a bench log.
          > NEW generation 53 - Volvo Gen5 SPC 31380867 (standalone only): lever
            0x041 bytes 6-7, feedback 0x126 torque + fault status decoded; the
            other 23 monitored frames sent as zero placeholders for OVR tuning.
          > Volvo P2 XC166 31431022: diagnostic IDs noted only - needs a capture.

V11.00.1 - (beta, 2026-10-24) Volvo P1 (S40 / V50 AWD), generation 31 - "Generation 3 -
          Volvo P1 (S40 / V50)". From Adam's 2005 V50 T5 AWD logs
          (documents/vehicle-logs/gen3/P1_V50_T5_AWD_DECODE.md). 500 kbit/s, 29-bit IDs.
        - Why the P2 choices showed no Haldex: the P1 uses none of the P2 IDs. Its AWD
          module sends one frame, 0x1960000A; the P2 decode only looked for 0x1020000A.
        - Haldex feedback: 0x1960000A byte 6 bits 0-1 + byte 7 = 10-bit engagement,
          540 = 100 % (VOLVO_P1_ENGAGEMENT_FULL_SCALE); byte 5 = status, shown on the
          dashboard legend. No fault bits found yet (health tiles show "--").
        - Standalone: all 22 frames the car puts on its high-speed bus, each a
          frame-edit block (all on), at the car's own rates. Steady bytes from a real
          30 km/h cruise frame; counters, toggles, the network heartbeat / clock
          (0x1BE0493C) and the four wheel pulse counters (0x19A00002) run like the
          car (patterns from the full C30 trace). Brake / handbrake from Follow
          Brake / Follow Handbrake. The car's VIN pages aren't sent.
        - 0x0340412E is the steering angle sensor (not the steering pump): byte 0 =
          counter + direction bit, bytes 1-2 fine rate, bytes 4-5 coarse rate, bytes
          6-7 angle, byte 3 a check byte = 0x02 - (byte 0 nibbles swapped + byte 1
          bit 7 + bytes 4-7), 0x04 with the direction bit set. Fits all 26,820
          logged frames from both cars. Standalone now works it out (wheel straight,
          not turning) instead of replaying the idle sequence. No other P1 frame has
          a check byte.
        - What drives the module (from the logs): engine torque, almost 1:1 (about
          one engagement count per torque count, 100-400 ms behind, saturating near
          540); big front wheelspin sends it to full; it never locks while stopped.
        - New "Volvo P1 Standalone" card (Settings, generation 31 only, saved and in
          backups): Lock Source (engine torque - default / wheel slip / both), Road
          Speed (5-120 km/h, default 30), Torque at 100 % (0-1000, default 600),
          Front Wheel Slip at 100 % (0-40 km/h, default 10), Reset to Defaults, and a
          live line showing the lock asked for, torque / pedal / rpm / wheel speeds
          sent and the module's engagement count. The mode buttons, Expert and Learn
          all drive it through lever_lock_percent(), as on the other units.
        - Learn / Long Learn on the P1: 600 ms per step (VOLVO_P1_LEARN_SETTLE_MS,
          other generations 400). In OEM mode they say it needs Standalone for now.
        - Wheel pulse counters (0x19A00002) follow the front / rear speeds; counters
          3-4 confirmed as the fronts from the wheelspin log.
        - Flash space: DEBUG() messages were always built in and printed - the switch
          used "#ifdef enableDebug", which is true even when enableDebug is 0. Now
          "#if enableDebug": saves flash (firmware was at 97.5 % of its slot) and
          stops the constant serial output. No partition change, OTA unaffected.
        - Flash space: Espressif's libraries rebuilt smaller with custom_sdkconfig in
          platformio.ini - no library log text, silent asserts, no error-name table,
          no crash dump (there was never a partition for one), no WiFi FTM / WPA3 /
          OWE (the home WiFi bridge can't join a WPA3-ONLY router). OpenHaldex now
          has its own PlatformIO core folder (C:/.platformio-openhaldex) because
          that rebuild happens inside the packages folder, which the other ESP32
          projects share. Same partitions, so OTA keeps working.
        - Chassis decode (OEM mode, or standalone with the car connected): speed
          0x02104136, rpm 0x0090411E, pedal + brake switch 0x0300410E, handbrake
          0x0110483C - so Expert mode and the speed / throttle limits work. DSTC off
          (0x02000026 byte 6 bit 4) drives "Use TC to Force Mode", like the VW ESP
          button, and shows on the Diagnostics ASR / TC pills.
        - OEM mode is untouched passthrough for now: Stock and the lock feedback
          work; editing frames for 50:50 / 60:40 etc. comes next.
        - Frame-edit masks: slot 5 kept spare (V12's Gen6 goes there) and P1 in slot
          6, so saved settings line up with V12. Older saves keep their choices -
          the loader now reads saves with fewer generations. Block CAN IDs are
          32-bit now (Volvo's 29-bit IDs didn't fit).

*/


/*

** to do **:
        > add throttle/speed axis refresh
        > add 'ota' to match existing layout
        > add reduction in throttle/speed off
        > move CAN into interrupt based - ESP_INTR_FLAG_IRAM
*/