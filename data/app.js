let currentEditCell = null; // global flag for current editted cell

const MODE_NAMES = ["Stock", "FWD", "50:50", "60:40", "75:25", "Expert"]; // mode names as Strings

// Cached settings for banner force-mode display and state legend (loaded once from /api/settings)
let _tcForceModeValue     = 2;
let _hazardForceModeValue = 2;
let _extBtnForceModeValue = 2;
let _tcForceMode = false;
let _hazardForceMode = false;
let _extBtnForceMode = false;
let _haldexGeneration = 1;
let _isStandalone = false;
let _useCANifAvailable = false;
let _disableController = false;
const setIntervalDuration = 500; // set refresh duration (for quickly to poll ESP for data)

var speedHeader = [0, 30, 60, 90, 120, 160, 180]; // default speed header (for x-axis)
var throttleHeader = [0, 15, 30, 45, 60, 75, 90]; // default throttle header (for y-axis)

const arrayColumns = speedHeader.length; // var for number of columns
const arrayRows = throttleHeader.length; // var for number of rows

var defaultSpeedHeader = [0, 30, 60, 90, 120, 160, 180]; // default speed header (for x-axis)
var defaultThrottleHeader = [0, 15, 30, 45, 60, 75, 90]; // default throttle header (for y-axis)

// constant for default lock (for restoring settings)
const defaultLock = [
  [0, 0, 0, 0, 0, 0, 0],
  [100, 50, 20, 15, 10, 5, 0],
  [100, 60, 30, 20, 15, 10, 0],
  [100, 70, 40, 30, 15, 15, 10],
  [100, 90, 60, 60, 30, 20, 10],
  [100, 100, 80, 70, 50, 30, 15],
  [100, 100, 100, 80, 60, 50, 40],
];

// variable for current lock (to be used / traced)
var currentLock = [
  [0, 0, 0, 0, 0, 0, 0],
  [0, 0, 0, 0, 0, 0, 0],
  [0, 0, 5, 5, 5, 5, 5],
  [0, 5, 10, 80, 80, 80, 80],
  [10, 20, 30, 40, 40, 40, 40],
  [10, 20, 30, 40, 40, 40, 40],
  [10, 20, 30, 40, 40, 40, 40],
];

// once the document is loaded, start running the script - start with getting settings
// await doesn't seem to work well - so if settings aren't captured first, the page draws WHILE settings are being grabbed.
// this is possible a hack, but I can't think of a better way to do it(!)
document.addEventListener("DOMContentLoaded", initStoredSettings);

// once settings are stored, start applying data where required
function initApp() {
  //initStoredSettings(); // old
  initNavigation();
  initDashboard();
  initModeButtons();
  initSettings();
  initExpertEditor();
  initLearn();
  initWifiSsid();
  initWifi();
  //initOtaPage(); todo - currently just old OTA page, would like to incorporate the styling across
}

// global function for getting data from the ESP
async function fetchJson(url, options) {
  try {
    const request = await fetch(url, options); // request data from ESP
    const result = await request.json(); // wait for response from ESP
    return result;
  } catch (error) {
    console.log("Error:" + error); // Catches and logs any errors
  }
}

// initialise stored settings (async function)
async function initStoredSettings() {
  // initialise stored settings and parse them
  try {
    const data = await fetchJson("/api/settings");
    // values
    document.getElementById("haldexGeneration").value =
      data.haldexGeneration || 1;
    document.getElementById("tcForceModeValue").value     = data.tcForceModeValue     ?? 2;
    document.getElementById("hazardForceModeValue").value  = data.hazardForceModeValue  ?? 2;
    document.getElementById("extBtnForceModeValue").value  = data.extBtnForceModeValue  ?? 2;

    document.getElementById("disengageUnderSpeedRange").value =
      data.disengageUnderSpeed || 0;
    document.getElementById("disengageUnderSpeed").textContent =
      data.disengageUnderSpeed || 0;

    document.getElementById("disengageAboveSpeedRange").value =
      data.disengageAboveSpeed || 0;
    document.getElementById("disengageAboveSpeed").textContent =
      data.disengageAboveSpeed || 0;

    document.getElementById("disableThrottleRange").value =
      data.disableThrottle || 0;
    document.getElementById("disableThrottle").textContent =
      data.disableThrottle || 0;

    const ledBrightPct = Math.round((data.ledBrightness !== undefined ? data.ledBrightness : 255) / 2.55);
    document.getElementById("ledBrightnessRange").value = ledBrightPct;
    document.getElementById("ledBrightnessValue").textContent = ledBrightPct;

    const lockRateRange = document.getElementById("lockReleaseRateRange");
    const lockRateVal   = document.getElementById("lockReleaseRateValue");
    if (lockRateRange && data.lockReleaseRatePerSec !== undefined) {
      lockRateRange.value = data.lockReleaseRatePerSec;
      if (lockRateVal) lockRateVal.textContent = data.lockReleaseRatePerSec;
    }

    const lockReleaseEnabledElem = document.getElementById("lockReleaseEnabled");
    if (lockReleaseEnabledElem) {
      lockReleaseEnabledElem.checked = data.lockReleaseEnabled !== undefined ? data.lockReleaseEnabled : true;
      const container = document.getElementById("lockReleaseRateContainer");
      if (container) container.style.opacity = lockReleaseEnabledElem.checked ? "" : "0.4";
      if (lockRateRange) lockRateRange.disabled = !lockReleaseEnabledElem.checked;
    }

    if (data.forceModesPriority !== undefined) {
      document.getElementById("forceModesPriority").value = data.forceModesPriority;
    }

    document.getElementById("FW_VERSION").textContent = data.FW_VERSION || "--";

    //document.getElementById('mode').value = data.mode || 1;

    // bools
    document.getElementById("disableController").checked =
      data.disableController || false;
    document.getElementById("isStandalone").checked =
      data.isStandalone || false;
    document.getElementById("useCANifAvailable").checked =
      data.useCANifAvailable || false;
    document.getElementById("tcForceMode").checked = data.tcForceMode || false;
    {
      const extBtnCtrlElem = document.getElementById("extButtonForceMode");
      if (extBtnCtrlElem) {
        extBtnCtrlElem.value = data.extButtonForceMode ? "1" : "0";
        const fmvRow = document.getElementById("extBtnForceModeValueRow");
        if (fmvRow) fmvRow.style.display = data.extButtonForceMode ? "" : "none";
      }
    }
    document.getElementById("hazardForceMode").checked =
      data.hazardForceMode || false;

    document.getElementById("followBrake").checked = data.followBrake || false;
    document.getElementById("invertBrake").checked = data.invertBrake || false;
    document.getElementById("followHandbrake").checked =
      data.followHandbrake || false;
    document.getElementById("invertHandbrake").checked =
      data.invertHandbrake || false;

    document.getElementById("broadcastOpenHaldexOverCAN").checked =
      data.broadcastOpenHaldexOverCAN || false;

    document.getElementById("disableOnboardButton").checked =
      data.disableOnboardButton || false;
    document.getElementById("disableExternalButton").checked =
      data.disableExternalButton || false;

    const fixHuntingElem = document.getElementById("fixHunting");
    if (fixHuntingElem) fixHuntingElem.checked = data.fixHunting || false;

    const canSleepElem = document.getElementById("canSleepEnabled");
    if (canSleepElem) canSleepElem.checked = data.canSleepEnabled || false;

    const canSleepAggrElem = document.getElementById("canSleepAggressive");
    if (canSleepAggrElem) canSleepAggrElem.checked = data.canSleepAggressive || false;

    // Aggressive implies basic - lock the basic checkbox while aggressive is on.
    if (canSleepElem && canSleepAggrElem) {
      canSleepElem.disabled = canSleepAggrElem.checked;
    }

    const lpWakeRange = document.getElementById("lpWakeThresholdFpsRange");
    const lpWakeVal   = document.getElementById("lpWakeThresholdFpsValue");
    if (lpWakeRange && data.lpWakeThresholdFps !== undefined) {
      lpWakeRange.value = data.lpWakeThresholdFps;
      if (lpWakeVal) lpWakeVal.textContent = data.lpWakeThresholdFps;
    }

    const analyzerModeElem = document.getElementById("analyzerMode");
    if (analyzerModeElem) analyzerModeElem.checked = data.analyzerMode || false;

    const analyzerSerialElem = document.getElementById("analyzerSerial");
    if (analyzerSerialElem) analyzerSerialElem.checked = data.analyzerSerial || false;

    const udsMqbElem = document.getElementById("udsMQBEnabled");
    if (udsMqbElem) udsMqbElem.checked = data.udsMQBEnabled || false;

    // cache for banner / legend
    _tcForceModeValue     = data.tcForceModeValue     ?? 2;
    _hazardForceModeValue = data.hazardForceModeValue  ?? 2;
    _extBtnForceModeValue = data.extBtnForceModeValue  ?? 2;
    _tcForceMode = data.tcForceMode || false;
    _hazardForceMode = data.hazardForceMode || false;
    _extBtnForceMode = data.extButtonForceMode || false;
    _haldexGeneration = data.haldexGeneration || 1;
    _isStandalone = data.isStandalone || false;
    _useCANifAvailable = data.useCANifAvailable || false;
    _disableController = data.disableController || false;

    // parse speed/throttle/lock array from the ESP
    speedHeader = data.speedArray;
    throttleHeader = data.throttleArray;
    currentLock = data.lockArray;

    // parse the mode button
    if (data.mode !== undefined) {
      modeButton(data.mode);
    }
  } catch (error) {
    console.log("Status failed: " + error.message);
  }

  initApp(); // settings captured, now start filling up the interface
}

// refresh ongoing data
async function refreshStatus() {
  try {
    const data = await fetchJson("/api/dashboard"); // send request for basic data

    // Live frame-rate monitor for LP wake threshold tuning
    if (data.lpChassisFrameCount !== undefined && data.lpHaldexFrameCount !== undefined) {
      const now = Date.now();
      if (refreshStatus._lastFrameTime) {
        const dt = (now - refreshStatus._lastFrameTime) / 1000;
        const cFps = Math.round((data.lpChassisFrameCount - refreshStatus._lastChassis) / dt);
        const hFps = Math.round((data.lpHaldexFrameCount  - refreshStatus._lastHaldex)  / dt);
        const cEl = document.getElementById("lpChassisFrameRate");
        const hEl = document.getElementById("lpHaldexFrameRate");
        if (cEl) cEl.textContent = cFps;
        if (hEl) hEl.textContent = hFps;
      }
      refreshStatus._lastFrameTime = now;
      refreshStatus._lastChassis   = data.lpChassisFrameCount;
      refreshStatus._lastHaldex    = data.lpHaldexFrameCount;
    }

    const displayValue = (value) => (value ?? "--");
    const displayOnOff = (value) =>
      value === undefined || value === null ? "--" : value ? "On" : "Off";

    document.getElementById("speed").textContent = displayValue(data.speed);
    document.getElementById("throttle").textContent = displayValue(
      data.throttle,
    );
    document.getElementById("rpm").textContent = displayValue(data.rpm);
    document.getElementById("boost").textContent = displayValue(data.boost);

    document.getElementById("lockTarget").textContent = displayValue(
      data.lockTarget,
    );
    document.getElementById("lockActual").textContent = displayValue(
      data.lockActual,
    );
    document.getElementById("engagementFill").style.width =
      `${data.lockActual ?? 0}%`;

    // Haldex Data card (Gen 1–4, non-UDS)
    document.getElementById("haldexEngagement").textContent = displayValue(data.haldexEngagement);
    document.getElementById("clutch1Report").textContent   = displayValue(data.clutch1Report);
    document.getElementById("clutch2Report").textContent   = displayValue(data.clutch2Report);
    document.getElementById("tempProtection").textContent  = displayOnOff(data.tempProtection);
    document.getElementById("couplingOpen").textContent    = displayOnOff(data.couplingOpen);
    document.getElementById("speedLimit").textContent      = displayOnOff(data.speedLimit);

    if (data.mode !== undefined) {
      modeButton(data.mode); // set the mode button - there may be external influences
    }

    const canStatus = document.getElementById("canStatus");
    const chassisOk = data.chassisCAN;
    const haldexOk = data.haldexCAN;
    canStatus.textContent = `CAN: ${chassisOk ? "✓" : "X"} Chassis | ${haldexOk ? "✓" : "X"} Haldex`;

    document.getElementById("diagChassisCAN").textContent = chassisOk
      ? "✓ Healthy"
      : "X Unhealthy";
    document.getElementById("diagHaldexCAN").textContent = haldexOk
      ? "✓ Healthy"
      : "X Unhealthy";
    document.getElementById("diagThrottle").textContent = displayValue(
      data.throttle,
    );
    document.getElementById("diagSpeed").textContent = displayValue(data.speed);
    document.getElementById("diagAsrStatus").textContent = displayOnOff(
      data.asrOn,
    );
    document.getElementById("diagTcStatus").textContent = displayOnOff(
      data.tcOn,
    );
    document.getElementById("diagHazardActive").textContent = displayOnOff(
      data.hazardActive,
    );
    document.getElementById("diagBrakeIn").textContent = displayOnOff(
      data.brakeIn,
    );
    document.getElementById("diagBrakeOut").textContent = displayOnOff(
      data.brakeOut,
    );
    document.getElementById("diagHandbrakeIn").textContent =
      displayOnOff(data.handbrakeIn);
    document.getElementById("diagHandbrakeOut").textContent =
      displayOnOff(data.handbrakeOut);
    document.getElementById("diagBrakeFromCAN").textContent = displayOnOff(
      data.brakeFromCAN,
    );
    document.getElementById("diagHandbrakeFromCAN").textContent = displayOnOff(
      data.handbrakeFromCAN,
    );
    document.getElementById("diagCpuUsage").textContent = displayValue(
      data.cpuUsage,
    );
    document.getElementById("diagFreeHeap").textContent = Math.round(
      data.freeHeap / 1024,
    );

    document.getElementById("haldexState").textContent =
      data.haldexState === undefined || data.haldexState === null
        ? "--"
        : hex2bin(data.haldexState);

    updateBannerSubtitle(data);
    // Gen50 (0CQ/MQB): state byte is Allrad_03 byte 3 (Charisma) — use gen=50 legend.
    // Gen51 (0AY) and all PQ gens (1/2/4): state byte is Allrad_1 byte 0 (PQ fault flags) — use gen-specific PQ legend.
    const legendGen = _haldexGeneration;
    renderHaldexStateLegend(data.haldexState, legendGen);

    // Gen 4.1 (GM/SAAB) specific data
    const gen41Card = document.getElementById("gen41Card");
    if (data.gen41) {
      gen41Card.style.display = "";
      const sa = data.gen41.secAxle;
      const ra = data.gen41.rearAxle;

      document.getElementById("g41SecTorqueNm").textContent = displayValue(sa?.torqueNm);
      document.getElementById("g41SecClutchState").textContent = displayValue(sa?.clutchState);

      document.getElementById("g41RearMetricA").textContent = displayValue(ra?.metricA);
      document.getElementById("g41RearMetricB").textContent = displayValue(ra?.metricB);
    } else {
      gen41Card.style.display = "none";
    }

    // UDS MQB diagnostic data — replaces the standard Haldex Data card when active
    const haldexDataCard = document.getElementById("haldexDataCard");
    const udsCard = document.getElementById("udsDataCard");
    if (data.uds) {
      if (haldexDataCard) haldexDataCard.style.display = "none";
      if (udsCard) {
        udsCard.style.display = "";
        document.getElementById("udsTerminalVoltage").textContent = data.uds.terminalVoltage?.toFixed(1) ?? "--";
        document.getElementById("udsModuleTemp").textContent = data.uds.moduleTemp?.toFixed(1) ?? "--";
        document.getElementById("udsClutchTemp").textContent = data.uds.clutchTemp?.toFixed(1) ?? "--";
        document.getElementById("udsCoolingFinTemp").textContent = data.uds.coolingFinTemp?.toFixed(1) ?? "--";
        document.getElementById("udsClutchCurrent").textContent = data.uds.clutchCurrent?.toFixed(3) ?? "--";
        document.getElementById("udsClutchPWM").textContent = displayValue(data.uds.clutchPWM);
        document.getElementById("udsClutchVoltage").textContent = data.uds.clutchVoltage?.toFixed(3) ?? "--";
        document.getElementById("udsBlockagePct").textContent = displayValue(data.uds.blockagePct);
      }
    } else {
      if (haldexDataCard) haldexDataCard.style.display = "";
      if (udsCard) udsCard.style.display = "none";
    }

    refreshTrace(data); // update the live trace
  } catch (error) {
    console.log("Status failed: " + error.message);
  }
}

function hex2bin(hex) {
  return ("00000000" + parseInt(hex, 16).toString(2)).substr(-8);
}

// Update the header subtitle with current mode and any active force mode.
function updateBannerSubtitle(data) {
  const el = document.getElementById("modeStatus");
  if (!el) return;
  const modeName = MODE_NAMES[data.mode] ?? "Unknown";
  let text = `Mode: ${modeName}`;

  // Surface EVERY active force trigger so a flag can never be silently active.
  // Each trigger must be BOTH enabled and have its live flag asserted - matching
  // the firmware gate in OpenHaldexC6_can.cpp. tcForceModeFlag/hazardForceModeFlag
  // are CAN-driven and extButtonActive is button-driven, so any of them can fire
  // without the user having touched the UI.
  const active = [];
  if (_tcForceMode     && data.tcOn === false)            active.push({ trig: "TC",     fmv: _tcForceModeValue });
  if (_hazardForceMode && data.hazardActive === true)     active.push({ trig: "Hazard", fmv: _hazardForceModeValue });
  if (_extBtnForceMode && data.extButtonActive === true)  active.push({ trig: "Ext",    fmv: _extBtnForceModeValue });

  if (active.length) {
    const parts = active.map((a) => `${MODE_NAMES[a.fmv] ?? "Unknown"} (${a.trig})`);
    text += ` | Force: ${parts.join(", ")}`;
  }
  el.textContent = text;
}

// Render a bit-by-bit legend for the Haldex state byte, generation-aware.
function renderHaldexStateLegend(rawHex, gen) {
  const el = document.getElementById("haldexStateLegend");
  if (!el) return;
  if (rawHex === undefined || rawHex === null) { el.innerHTML = ""; return; }
  const val = parseInt(rawHex, 16);

  if (gen === 1 || gen === 2 || gen === 4 || gen === 51) {
    // PQ (Gen 1/2/4): Allrad_1 byte 0 — fault/status flags
    const bits = [
      { bit: 0, label: "Clutch Fault",           desc: "Fehler_Allrad_Kupplung" },
      { bit: 1, label: "Over-Temp Protection",    desc: "Übertemperaturschutz" },
      { bit: 2, label: "Clutch Stiffness Fault",  desc: "Fehlerstatus_Kupplungssteifigkeit" },
      { bit: 3, label: "Coupling Fully Open",     desc: "Kupplung_komplett_offen" },
      { bit: 4, label: "Limp Mode",               desc: "Notlauf" },
      { bit: 5, label: "AWD Warning Lamp",        desc: "Allrad_Warnlampe" },
      { bit: 6, label: "Speed Limit",             desc: "Geschwindigkeitsbegrenzung" },
    ];
    let html = `<table><tr><th>Bit</th><th>Name</th><th>State</th></tr>`;
    bits.forEach(({ bit, label }) => {
      const active = (val >> bit) & 1;
      const cls = active ? "bit-active" : "bit-inactive";
      html += `<tr><td>${bit}</td><td>${label}</td><td class="${cls}">${active ? "SET" : "ok"}</td></tr>`;
    });
    html += `</table>`;
    el.innerHTML = html;
  } else if (gen === 50) {
    // MQB (Gen 5): Allrad_03 byte 3 = ALR_Charisma_FahrPr / ALR_Charisma_Status
    const prog = val & 0x0F;
    const flags = (val >> 4) & 0x0F;
    el.innerHTML =
      `<table><tr><th>Field</th><th>Value</th></tr>` +
      `<tr><td>Driving Programme (bits 0–3)</td><td>${prog}</td></tr>` +
      `<tr><td>Status Flags (bits 4–7)</td><td>0x${flags.toString(16).toUpperCase()}</td></tr>` +
      `</table><p style="margin:4px 0 0;">Note: bit 4–5 of byte 1 = longitudinal lock state (0=open, 1=partial, 2=closed) — separate from this byte.</p>`;
  } else if (gen === 41) {
    el.innerHTML = `<em>Gen 4.1: dedicated status variables used — see Gen41 card above.</em>`;
  } else {
    el.innerHTML = "";
  }
}

// save current lock table
async function saveLockTable() {
  // send the new map data back to the ESP
  try {
    const response = await fetch("/api/tune", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        speedArray: speedHeader,
        throttleArray: throttleHeader,
        lockArray: currentLock.map((r) => r),
      }),
    });

    if (response.ok) {
      showNotification("Map Saved");
    } else {
      showNotification("Failed to Save", "error");
    }
  } catch (error) {
    console.error("Error saving map:", error);
  }
}

// save individual setting immediately
async function saveSetting(key, value) {
  const settings = { [key]: value };

  try {
    const response = await fetchJson("/api/settings", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(settings),
    });

    if (!response.ok) {
      showNotification("Failed to save setting", "error");
    }
  } catch (error) {
    console.log("Saving setting failed: " + error.message);
    showNotification("Error saving setting", "error");
  }
}

// initialise navigation:
function initNavigation() {
  // setup tabs
  const tabs = document.querySelectorAll(".nav-tab");
  const pages = document.querySelectorAll(".page");

  tabs.forEach((tab) => {
    tab.addEventListener("click", () => {
      const page = tab.dataset.page;

      tabs.forEach((t) => t.classList.remove("active"));
      tab.classList.add("active");

      pages.forEach((p) => p.classList.remove("active"));
      document.getElementById(`${page}-page`).classList.add("active");
    });
  });

  // setup 'when slider changes', do this...
  const disengageUnderSpeedRange = document.getElementById(
    "disengageUnderSpeedRange",
  );
  const disengageUnderSpeed = document.getElementById("disengageUnderSpeed");
  disengageUnderSpeedRange.addEventListener("input", () => {
    disengageUnderSpeed.textContent = disengageUnderSpeedRange.value;
  });

  const disengageAboveSpeedRange = document.getElementById(
    "disengageAboveSpeedRange",
  );
  const disengageAboveSpeed = document.getElementById("disengageAboveSpeed");
  disengageAboveSpeedRange.addEventListener("input", () => {
    disengageAboveSpeed.textContent = disengageAboveSpeedRange.value;
  });
  const disableThrottleRange = document.getElementById("disableThrottleRange");
  const disableThrottle = document.getElementById("disableThrottle");
  disableThrottleRange.addEventListener("input", () => {
    disableThrottle.textContent = disableThrottleRange.value;
  });

  const ledBrightnessRange = document.getElementById("ledBrightnessRange");
  const ledBrightnessValue = document.getElementById("ledBrightnessValue");
  if (ledBrightnessRange) {
    ledBrightnessRange.addEventListener("input", () => {
      ledBrightnessValue.textContent = ledBrightnessRange.value;
    });
  }

  // LP wake threshold slider
  const lpWakeRange = document.getElementById("lpWakeThresholdFpsRange");
  const lpWakeVal   = document.getElementById("lpWakeThresholdFpsValue");
  if (lpWakeRange) {
    lpWakeRange.addEventListener("input", () => {
      if (lpWakeVal) lpWakeVal.textContent = lpWakeRange.value;
    });
    lpWakeRange.addEventListener("change", () => {
      saveSetting("lpWakeThresholdFps", parseInt(lpWakeRange.value));
    });
  }

  // Lock release rate slider (display update only — save handled in initSettings)
  const lockRateRange = document.getElementById("lockReleaseRateRange");
  const lockRateVal   = document.getElementById("lockReleaseRateValue");
  if (lockRateRange) {
    lockRateRange.addEventListener("input", () => {
      if (lockRateVal) lockRateVal.textContent = lockRateRange.value;
    });
  }
}

// initialise dashboard:
function initDashboard() {
  refreshStatus(); //
  setInterval(refreshStatus, setIntervalDuration); // request for new data every xms
}

// initialise mode buttons
function initModeButtons() {
  const buttons = document.querySelectorAll(".mode-btn");
  buttons.forEach((btn) => {
    btn.addEventListener("click", () => {
      const mode = parseInt(btn.dataset.mode);

      // Guard: controller disabled
      if (_disableController) {
        showNotification("Controller is disabled — enable it in Controller Options before changing mode", "error");
        return;
      }

      // Guard: Stock unavailable in standalone (no chassis CAN to read from)
      if (mode === 0 && _isStandalone) {
        showNotification("Stock mode is unavailable in Standalone — no chassis CAN to read from", "error");
        return;
      }

      // Guard: Expert requires CAN data (not standalone without 'Use CAN if Available')
      if (mode === 5 && _isStandalone && !_useCANifAvailable) {
        showNotification("Expert mode requires 'Use CAN if Available' to be enabled when in Standalone", "error");
        return;
      }

      modeButton(mode); // change highlighted mode
      sendMode(mode); // send new mode to ESP
    });
  });

  async function sendMode(mode) {
    const sendData = {
      mode: mode, // send just the mode change
    };

    try {
      await fetchJson("/api/mode", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(sendData),
      });
    } catch (error) {
      console.log("Save failed: " + error.message);
    }
  }
}

// initialise settings with immediate save on change
function initSettings() {
  // Selects (dropdown menus)
  const selectIds = ["haldexGeneration", "tcForceModeValue", "hazardForceModeValue", "extBtnForceModeValue", "forceModesPriority"];
  selectIds.forEach((id) => {
    const elem = document.getElementById(id);
    if (elem) {
      elem.addEventListener("change", () => {
        saveSetting(id, parseInt(elem.value));
      });
    }
  });

  // Range sliders
  const rangeSliders = [
    { element: "disengageUnderSpeedRange", key: "disengageUnderSpeed", parse: parseInt },
    { element: "disengageAboveSpeedRange", key: "disengageAboveSpeed", parse: parseInt },
    { element: "disableThrottleRange",     key: "disableThrottle",     parse: parseInt },
    { element: "lockReleaseRateRange",     key: "lockReleaseRatePerSec", parse: parseFloat },
  ];
  rangeSliders.forEach(({ element, key, parse }) => {
    const elem = document.getElementById(element);
    if (elem) {
      elem.addEventListener("input", () => {
        saveSetting(key, (parse || parseInt)(elem.value));
      });
    }
  });

  // LED brightness: UI is 0-100%, firmware stores 0-255
  const ledBrightElem = document.getElementById("ledBrightnessRange");
  if (ledBrightElem) {
    ledBrightElem.addEventListener("input", () => {
      saveSetting("ledBrightness", Math.round(parseInt(ledBrightElem.value) * 2.55));
    });
  }

  // Checkboxes — keep cached state in sync for mode-button guards
  const checkboxCacheMap = {
    disableController: (v) => { _disableController = v; },
    isStandalone:      (v) => { _isStandalone = v; },
    useCANifAvailable: (v) => { _useCANifAvailable = v; },
  };

  const checkboxIds = [
    "disableController",
    "isStandalone",
    "useCANifAvailable",
    "tcForceMode",
    "hazardForceMode",
    "followBrake",
    "invertBrake",
    "followHandbrake",
    "invertHandbrake",
    "broadcastOpenHaldexOverCAN",
    "disableOnboardButton",
    "disableExternalButton",
    "fixHunting",
    "canSleepEnabled",
    "canSleepAggressive",
    "udsMQBEnabled",
    "lockReleaseEnabled",
  ];
  checkboxIds.forEach((id) => {
    const elem = document.getElementById(id);
    if (elem) {
      elem.addEventListener("change", () => {
        if (checkboxCacheMap[id]) checkboxCacheMap[id](elem.checked);
        saveSetting(id, elem.checked);
      });
    }
  });

  // Ext. Control select (Press: Advance Mode / Hold: Force Mode)
  {
    const extBtnCtrlElem = document.getElementById("extButtonForceMode");
    if (extBtnCtrlElem) {
      const fmvRow = document.getElementById("extBtnForceModeValueRow");
      extBtnCtrlElem.addEventListener("change", () => {
        const isHold = extBtnCtrlElem.value === "1";
        saveSetting("extButtonForceMode", isHold);
        if (fmvRow) fmvRow.style.display = isHold ? "" : "none";
      });
    }
  }

  // Lock release: toggle slider enabled state and opacity when checkbox changes.
  const lockReleaseEnabledElem = document.getElementById("lockReleaseEnabled");
  const lockReleaseRateElem    = document.getElementById("lockReleaseRateRange");
  const lockReleaseContainer   = document.getElementById("lockReleaseRateContainer");
  if (lockReleaseEnabledElem) {
    lockReleaseEnabledElem.addEventListener("change", () => {
      const en = lockReleaseEnabledElem.checked;
      if (lockReleaseRateElem) lockReleaseRateElem.disabled = !en;
      if (lockReleaseContainer) lockReleaseContainer.style.opacity = en ? "" : "0.4";
    });
  }

  // CAN sleep: Aggressive implies Basic. Keep the two checkboxes in sync.
  const canSleepElem = document.getElementById("canSleepEnabled");
  const canSleepAggrElem = document.getElementById("canSleepAggressive");
  if (canSleepElem && canSleepAggrElem) {
    canSleepAggrElem.addEventListener("change", () => {
      if (canSleepAggrElem.checked && !canSleepElem.checked) {
        canSleepElem.checked = true;
        saveSetting("canSleepEnabled", true);
      }
      canSleepElem.disabled = canSleepAggrElem.checked;
    });
    canSleepElem.addEventListener("change", () => {
      if (!canSleepElem.checked && canSleepAggrElem.checked) {
        canSleepAggrElem.checked = false;
        saveSetting("canSleepAggressive", false);
        canSleepElem.disabled = false;
      }
    });
  }

  // SavvyCAN mode - mutually exclusive: WiFi and Serial cannot both be active.
  const analyzerModeElem = document.getElementById("analyzerMode");
  const analyzerSerialElem = document.getElementById("analyzerSerial");
  if (analyzerModeElem && analyzerSerialElem) {
    analyzerModeElem.addEventListener("change", () => {
      if (analyzerModeElem.checked) {
        analyzerSerialElem.checked = false;
        saveSetting("analyzerSerial", false);
      }
      saveSetting("analyzerMode", analyzerModeElem.checked);
    });
    analyzerSerialElem.addEventListener("change", () => {
      if (analyzerSerialElem.checked) {
        analyzerModeElem.checked = false;
        saveSetting("analyzerMode", false);
      }
      saveSetting("analyzerSerial", analyzerSerialElem.checked);
    });
  }
}

function modeButton(mode) {
  const buttons = document.querySelectorAll(".mode-btn");
  buttons.forEach((btn) => {
    btn.classList.toggle("active", parseInt(btn.dataset.mode) === mode);
  });

  //document.getElementById('currentMode').textContent = MODE_NAMES[mode] || 'Unknown';
}

// initialise expert editor
function initExpertEditor() {
  const mapGrid = document.getElementById("mapGrid"); // find the 'map grid'

  const cellMarker = document.createElement("div"); // create the first element (which will be a dead cell)
  cellMarker.className = "map-cellHeader";
  cellMarker.setAttribute("speed", String("100")); // make a new attribuate for column
  cellMarker.setAttribute("throttle", String("100")); // make a new attribute for row
  cellMarker.setAttribute("lock", String("na")); // make a new attribute for row
  cellMarker.textContent = String("/"); // marker for showing row/col
  mapGrid.appendChild(cellMarker); // add in the first cell

  // fill the speed axis
  for (let speed = 0; speed < arrayColumns; speed++) {
    const cellSpeedAxis = document.createElement("div");
    cellSpeedAxis.className = "map-cellHeader";
    cellSpeedAxis.min = 0;
    cellSpeedAxis.max = 300;
    cellSpeedAxis.setAttribute("speed", String(speed)); // make a new attribuate for column
    cellSpeedAxis.setAttribute("throttle", String("100")); // make a new attribute for row
    cellSpeedAxis.setAttribute("lock", String("false")); // make a new attribute for row
    cellSpeedAxis.textContent = String(speedHeader[speed]); // update the cell text with the value in currentLock

    cellSpeedAxis.addEventListener("click", () => {
      openEditValue(cellSpeedAxis);
    });

    mapGrid.appendChild(cellSpeedAxis);
  }

  for (let throttle = 0; throttle < arrayRows; throttle++) {
    // fill the throttle axis
    const cellThrottleAxis = document.createElement("div");
    cellThrottleAxis.className = "map-cellHeader";
    cellThrottleAxis.min = 0;
    cellThrottleAxis.max = 100;
    cellThrottleAxis.setAttribute("speed", String("100")); // make a new attribuate for column
    cellThrottleAxis.setAttribute("throttle", String(throttle)); // make a new attribute for row
    cellThrottleAxis.setAttribute("lock", String("false")); // make a new attribute for row
    cellThrottleAxis.textContent = String(throttleHeader[throttle]); // update the cell text with the value in currentLock

    cellThrottleAxis.addEventListener("click", () => {
      openEditValue(cellThrottleAxis);
    });

    mapGrid.appendChild(cellThrottleAxis);
    //end fill

    // fill the contents
    for (let speed = 0; speed < arrayColumns; speed++) {
      const cell = document.createElement("div");
      cell.className = "map-cell";

      cell.setAttribute("speed", String(speed)); // make a new attribuate for column
      cell.setAttribute("throttle", String(throttle)); // make a new attribute for row
      cell.setAttribute("lock", String("true")); // make a new attribute for row
      cell.min = 0;
      cell.max = 100;
      cell.textContent = String(currentLock[throttle][speed]); // update the cell text with the value in currentLock

      cell.addEventListener("click", () => {
        openEditValue(cell);
      });

      updateCellColor(cell, currentLock[throttle][speed]); // update the colour (low/medium/high)

      mapGrid.appendChild(cell);
    }
  }

  document.getElementById("cancelEdit").addEventListener("click", cancelEdit);
  document.getElementById("confirmEdit").addEventListener("click", confirmEdit);
  document.getElementById("saveMap").addEventListener("click", saveLockTable);
  document
    .getElementById("restoreDefaults")
    .addEventListener("click", restoreDefaults);
}

// find array position from a value
function arrayIndex(value, array) {
  let position = 0;
  for (let i = 0; i < array.length; i++) {
    if (value >= array[i]) position = i;
  }
  return position;
}

// refresh live trace
function refreshTrace(data) {
  const cell = [...document.querySelectorAll(".map-cell")]; // find all the cells in the grid (as an array)

  for (i in cell) {
    cell[i].classList.remove("activeTrace"); // remove the active trace from all cells (so only the current one is highlighted)
  }

  if (
    data.speed === undefined ||
    data.speed === null ||
    data.throttle === undefined ||
    data.throttle === null
  ) {
    return;
  }

  const speed = Number(data.speed); // get the current speed from the data
  const throttle = Number(data.throttle); // get the current throttle from the data

  if (Number.isNaN(speed) || Number.isNaN(throttle)) {
    return;
  }

  const throttlePos = arrayIndex(throttle, throttleHeader); // find the position in the grid for the current throttle
  const speedPos = arrayIndex(speed, speedHeader); // find the position in the grid for the current speed

  cell[speedPos + throttlePos * throttleHeader.length].classList.add(
    "activeTrace",
  ); // find the cell in the grid and update the class to activeTrace
}

// update the cell colours
function updateCellColor(cell, value) {
  cell.classList.remove("low", "medium", "high"); // remove all colour classes

  if (value < 30) {
    cell.classList.add("low"); // if the value is less than 30, add the 'low' class
  } else if (value < 60) {
    cell.classList.add("medium"); // if the value is between 30 and 60, add the 'medium' class
  } else {
    cell.classList.add("high"); // if the value is above 60, add the 'high' class
  }
}

// tune edit
function openEditValue(cell) {
  currentEditCell = cell;
  const modal = document.getElementById("editModal");
  const modalTitle = document.getElementById("editModalTitle");
  const input = document.getElementById("editValue");

  if (cell.getAttribute("speed") === "100") {
    modalTitle.textContent = "Edit Throttle Axis";
  }

  if (cell.getAttribute("throttle") === "100") {
    modalTitle.textContent = "Edit Speed Axis";
  }

  if (cell.getAttribute("lock") === "true") {
    modalTitle.textContent = "Edit Lock %";
  }

  input.value = cell.textContent;
  input.focus();
  input.select();

  modal.classList.add("active");
}

function cancelEdit() {
  document.getElementById("editModal").classList.remove("active");
  currentEditCell = null;
}

function confirmEdit() {
  if (!currentEditCell) return;

  const currentCell = document.getElementById("editValue"); // find the current edit cell

  const throttlePos = parseInt(currentEditCell.getAttribute("throttle")); // get the throttle var (position in grid)
  const speedPos = parseInt(currentEditCell.getAttribute("speed")); // get the speed var (position in grid)
  const lockTrue = currentEditCell.getAttribute("lock"); // get the lock var (position in grid)

  const value = parseInt(document.getElementById("editValue").value);

  if (speedPos === 100 && lockTrue === "false") {
    if (isNaN(value) || value < 0 || value > 100) {
      showNotification("Value must be between 0 and 100", "error");
      return;
    }
    throttleHeader[throttlePos] = Number(value);
  }
  if (throttlePos === 100 && lockTrue === "false") {
    if (isNaN(value) || value < 0) {
      showNotification("Value must be greater than 0", "error");
      return;
    }
    speedHeader[speedPos] = Number(value);
  }

  if (lockTrue === "true") {
    if (isNaN(value) || value < 0 || value > 100) {
      showNotification("Value must be between 0 and 100", "error");
      return;
    }
    currentLock[throttlePos][speedPos] = Number(value);
    updateCellColor(currentEditCell, value);
  }

  currentEditCell.textContent = value;
  cancelEdit();
}
// end tune edit

function restoreDefaults() {
  // todo - redraw the map-grid
  for (let throttle = 0; throttle < arrayRows; throttle++) {
    throttleHeader[throttle] = defaultThrottleHeader[throttle];
  }

  for (let speed = 0; speed < arrayColumns; speed++) {
    speedHeader[speed] = defaultSpeedHeader[speed];
  }

  const cell = [...document.querySelectorAll(".map-cell")];
  let i = 0;
  for (let throttle = 0; throttle < arrayRows; throttle++) {
    for (let speed = 0; speed < arrayColumns; speed++) {
      currentLock[throttle][speed] = defaultLock[throttle][speed];
      cell[i].textContent = String(currentLock[throttle][speed]);
      updateCellColor(cell[i], currentLock[throttle][speed]);
      i++;
    }
  }
}

// initialise Learn Haldex UI
function initLearn() {
  let learnPollInterval = null;

  const statusText    = document.getElementById("learnStatusText");
  const progressWrap  = document.getElementById("learnProgressWrap");
  const progressFill  = document.getElementById("learnProgressFill");
  const progressLabel = document.getElementById("learnProgressLabel");
  const learnTrack    = document.getElementById("learnTrack");
  const learnCFFill   = document.getElementById("learnCFFill");
  const learnEngFill  = document.getElementById("learnEngFill");
  const learnCFValue  = document.getElementById("learnCFValue");
  const learnEngValue = document.getElementById("learnEngValue");
  const btnStart      = document.getElementById("learnStart");
  const btnCancel     = document.getElementById("learnCancel");
  const btnClear      = document.getElementById("learnClear");

  function setProgress(pct) {
    progressFill.style.width = pct + "%";
    progressLabel.textContent = pct + "%";
  }

  function setTracking(cf, eng) {
    learnCFFill.style.width   = cf  + "%";
    learnEngFill.style.width  = eng + "%";
    learnCFValue.textContent  = cf;
    learnEngValue.textContent = eng;
  }

  function startPolling() {
    btnStart.style.display     = "none";
    btnCancel.style.display    = "";
    progressWrap.style.display = "";
    learnTrack.style.display   = "";
    setProgress(0);
    setTracking(0, 0);
    learnPollInterval = setInterval(pollStatus, 100);
  }

  function stopPolling() {
    clearInterval(learnPollInterval);
    learnPollInterval          = null;
    btnStart.style.display     = "";
    btnCancel.style.display    = "none";
    progressWrap.style.display = "none";
    learnTrack.style.display   = "none";
  }

  async function pollStatus() {
    const data = await fetchJson("/api/learn/status");
    if (!data) return;

    const pct = Math.min(100, Math.round(data.progress));
    setProgress(pct);
    setTracking(data.currentCF ?? 0, data.currentEng ?? 0);

    if (!data.active) {
      stopPolling();
      if (data.progress === 102) {
        statusText.textContent = "No Haldex CAN data recorded - check connection";
        statusText.style.color = "var(--danger)";
      } else if (data.tableValid) {
        statusText.textContent = "Learn complete \u2713 - calibration table active";
        statusText.style.color = "var(--success)";
      } else {
        statusText.textContent = "Learn cancelled or failed";
        statusText.style.color = "var(--warning)";
      }
    }
  }

  btnStart.addEventListener("click", async () => {
    statusText.textContent = "Learning\u2026";
    statusText.style.color = "var(--text-dim)";
    const resp = await fetchJson("/api/learn/start", { method: "POST" });
    if (!resp || !resp.ok) {
      statusText.textContent = resp && resp.error ? resp.error : "Failed to start learn";
      statusText.style.color = "var(--danger)";
      return;
    }
    startPolling();
  });

  btnCancel.addEventListener("click", async () => {
    await fetchJson("/api/learn/cancel", { method: "POST" });
    stopPolling();
    statusText.textContent = "Learn cancelled";
    statusText.style.color = "var(--warning)";
  });

  btnClear.addEventListener("click", async () => {
    await fetchJson("/api/learn/clear", { method: "POST" });
    statusText.textContent = "Learn data cleared - static factor active";
    statusText.style.color = "var(--text-dim)";
  });

  // check initial state on page load
  fetchJson("/api/learn/status").then((data) => {
    if (!data) return;
    if (data.active) {
      statusText.textContent = "Learning\u2026";
      statusText.style.color = "var(--text-dim)";
      startPolling();
    } else if (data.tableValid) {
      statusText.textContent = "Learn complete \u2713 - calibration table active";
      statusText.style.color = "var(--success)";
    } else {
      statusText.textContent = "No learn data - static factor active";
      statusText.style.color = "var(--text-dim)";
    }
  });
}

// initialise WiFi SSID section
function initWifiSsid() {
  const input    = document.getElementById("wifiSsidInput");
  const status   = document.getElementById("wifiSsidStatus");
  const btnSave  = document.getElementById("wifiSsidSave");
  const btnReset = document.getElementById("wifiSsidReset");
  if (!input || !status || !btnSave || !btnReset) return;

  let defaultSsid = "OpenHaldex-C6";

  function renderStatus(ssid) {
    if (!ssid) {
      status.textContent = "--";
      status.style.color = "var(--text-dim)";
      return;
    }
    if (ssid === defaultSsid) {
      status.textContent = "Default SSID: " + ssid;
      status.style.color = "var(--text-dim)";
    } else {
      status.textContent = "\u2713 Custom SSID: " + ssid;
      status.style.color = "var(--success)";
    }
  }

  // load current SSID
  fetchJson("/api/wifi/ssid").then((data) => {
    if (!data) return;
    if (data.default) defaultSsid = data.default;
    if (data.ssid) {
      input.value = data.ssid;
      input.placeholder = data.ssid;
      renderStatus(data.ssid);
    }
  });

  // save SSID
  btnSave.addEventListener("click", async () => {
    const ssid = input.value.trim();
    if (ssid.length < 1) {
      showNotification("SSID cannot be empty", "error");
      return;
    }
    if (ssid.length > 32) {
      showNotification("SSID too long (max 32)", "error");
      return;
    }
    if (!/^[\x20-\x7E]+$/.test(ssid)) {
      showNotification("SSID must be printable ASCII", "error");
      return;
    }
    const resp = await fetchJson("/api/wifi/ssid", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ ssid: ssid }),
    });
    if (!resp) { showNotification("Failed to reach device", "error"); return; }
    if (!resp.ok) {
      showNotification(resp.error || "Failed to save SSID", "error");
      return;
    }
    status.textContent = "AP restarting as \"" + resp.ssid + "\"\u2026";
    status.style.color = "var(--success)";
    showNotification("WiFi SSID saved - reconnect to AP");
  });

  // reset to factory SSID
  btnReset.addEventListener("click", async () => {
    const resp = await fetchJson("/api/wifi/ssid/reset", { method: "POST" });
    if (!resp || !resp.ok) { showNotification("Reset failed", "error"); return; }
    input.value = resp.ssid || defaultSsid;
    status.textContent = "AP restarting as \"" + (resp.ssid || defaultSsid) + "\"\u2026";
    status.style.color = "var(--text-dim)";
    showNotification("WiFi SSID reset to default - reconnect to AP");
  });
}

// initialise WiFi password section
function initWifi() {
  const input   = document.getElementById("wifiPasswordInput");
  const toggle  = document.getElementById("wifiPasswordToggle");
  const status  = document.getElementById("wifiPasswordStatus");
  const btnSave = document.getElementById("wifiPasswordSave");
  const btnReset= document.getElementById("wifiPasswordReset");

  // show / hide password toggle
  toggle.addEventListener("click", () => {
    const isHidden = input.type === "password";
    input.type = isHidden ? "text" : "password";
    toggle.textContent = isHidden ? "\uD83D\uDE48" : "\uD83D\uDC41";
  });

  // load current status (just whether a password is set; never reveal the value)
  fetchJson("/api/wifi").then((data) => {
    if (!data) return;
    if (data.passwordSet) {
      status.textContent = "\u2713 Password set - AP is secured";
      status.style.color = "var(--success)";
    } else {
      status.textContent = "No password - AP is open";
      status.style.color = "var(--text-dim)";
    }
  });

  // save password
  btnSave.addEventListener("click", async () => {
    const pwd = input.value.trim();
    const resp = await fetchJson("/api/wifi", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ password: pwd }),
    });
    if (!resp) { showNotification("Failed to reach device", "error"); return; }
    if (!resp.ok) {
      showNotification(resp.error || "Failed to save password", "error");
      return;
    }
    input.value = "";
    if (resp.passwordSet) {
      status.textContent = "\u2713 Password set - AP restarting\u2026";
      status.style.color = "var(--success)";
      showNotification("WiFi password saved - reconnect to AP");
    } else {
      status.textContent = "No password - AP restarting as open\u2026";
      status.style.color = "var(--text-dim)";
      showNotification("WiFi password cleared");
    }
  });

  // reset to open network
  btnReset.addEventListener("click", async () => {
    const resp = await fetchJson("/api/wifi/reset", { method: "POST" });
    if (!resp || !resp.ok) { showNotification("Reset failed", "error"); return; }
    input.value = "";
    status.textContent = "No password - AP restarting as open\u2026";
    status.style.color = "var(--text-dim)";
    showNotification("WiFi reset to open network - reconnect to AP");
  });
}

function showNotification(message, type = "success") {
  const notification = document.createElement("div");
  notification.textContent = message;
  notification.style.cssText = `
        position: fixed;
        top: 20px;
        left: 50%;
        transform: translateX(-50%);
        padding: 1rem 2rem;
        background: ${type === "error" ? "var(--danger)" : "var(--success)"};
        color: white;
        border-radius: 8px;
        z-index: 10000;
        font-weight: 600;
        box-shadow: 0 4px 12px rgba(0,0,0,0.3);
    `;

  document.body.appendChild(notification);

  setTimeout(() => {
    notification.style.transition = "opacity 0.3s";
    notification.style.opacity = "0";
    setTimeout(() => notification.remove(), 300);
  }, 3000);
}
