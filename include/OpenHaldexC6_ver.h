#pragma once
#include <OpenHaldexC6_defs.h>

// Current firmware version
#define FW_VERSION "9.00.6" // also bump data/version.json ("fs")

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

*/


/*

** to do **:
        > add throttle/speed axis refresh
        > add 'ota' to match existing layout
        > add reduction in throttle/speed off
        > move CAN into interrupt based - ESP_INTR_FLAG_IRAM
*/