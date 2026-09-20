let currentEditCell = null; // global flag for current editted cell

const MODE_NAMES = ["Stock", "FWD", "50:50", "60:40", "75:25", "Expert"]; // mode names as Strings

// Cached settings for banner force-mode display and state legend (loaded once from /api/settings)
let _tcForceModeValue     = 2;
let _hazardForceModeValue = 2;
let _extBtnForceModeValue = 2;
let _tcForceMode = false;
let _hazardForceMode = false;
let _extBtnForceMode = false;
let _forceModesPriority = 0;
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

// Steering-angle lock scale (third axis): breakpoints (deg) -> lock multiplier (%)
var steeringHeader = [0, 45, 90, 180, 360]; // steering-wheel angle breakpoints (deg)
var steeringScale = [100, 100, 80, 50, 20]; // lock multiplier at each breakpoint (%)
var defaultSteeringHeader = [0, 45, 90, 180, 360];
var defaultSteeringScale = [100, 100, 80, 50, 20];

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
  initLongLearn();
  initWifiSsid();
  initWifi();
  initOtaPage();
  initCollapsibleCards();
  initGaugeUI();
  drawTuneChart();
  startIntroSweep(); // last: everything above is drawn; sweeps once the window has loaded and the first poll is in
}

// Collapse configuration-style cards by default so pages open compact and
// tidy (like Can2Cluster). Tap a card heading to expand/collapse it. Add the
// "no-collapse" class to any card that must stay permanently open.
function initCollapsibleCards() {
  ["basic-page", "expert-page", "settings-page", "diagnostics-page"].forEach((pageId) => {
    const page = document.getElementById(pageId);
    if (!page) return;
    page.querySelectorAll(".card").forEach((card) => {
      if (card.classList.contains("no-collapse")) return;
      const h2 = card.querySelector("h2");
      if (!h2) return;
      card.classList.add("collapsible", "collapsed");
      h2.addEventListener("click", () => card.classList.toggle("collapsed"));
    });
  });
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

    const bpkRange = document.getElementById("bpkCeilingRange");
    const bpkVal   = document.getElementById("bpkCeilingValue");
    if (bpkRange && data.bpkCeilingNm !== undefined) {
      bpkRange.value = data.bpkCeilingNm;
      if (bpkVal) bpkVal.textContent = data.bpkCeilingNm;
    }

    const esp14Range = document.getElementById("esp14MinFloorRange");
    const esp14Val   = document.getElementById("esp14MinFloorValue");
    if (esp14Range && data.esp14MinFloorPct !== undefined) {
      esp14Range.value = data.esp14MinFloorPct;
      if (esp14Val) esp14Val.textContent = data.esp14MinFloorPct;
    }

    const llNotes = document.getElementById("longLearnNotes");
    if (llNotes && typeof data.longLearnNotes === "string") llNotes.value = data.longLearnNotes;

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
    const boardRevEl = document.getElementById("BOARD_REV");
    if (boardRevEl) boardRevEl.textContent = data.boardRev ? `rev ${data.boardRev}` : "--";

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

    const dangerZoneElem = document.getElementById("dangerZoneEnabled");
    if (dangerZoneElem) dangerZoneElem.checked = data.dangerZoneEnabled || false;

    const steeringScaleEnabledElem = document.getElementById("steeringScaleEnabled");
    if (steeringScaleEnabledElem) {
      steeringScaleEnabledElem.checked =
        data.steeringScaleEnabled !== undefined ? data.steeringScaleEnabled : true;
      const stbl = document.getElementById("steeringScaleTable");
      if (stbl) stbl.style.opacity = steeringScaleEnabledElem.checked ? "" : "0.4";
    }

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

    const udsMqbElem = document.getElementById("liveDiagEnabled");
    if (udsMqbElem) udsMqbElem.checked = data.liveDiagEnabled || false;

    // Frame-edit gating checkboxes (per-generation)
    renderFrameBlocks(data.frameBlocks);

    // cache for banner / legend
    _tcForceModeValue     = data.tcForceModeValue     ?? 2;
    _hazardForceModeValue = data.hazardForceModeValue  ?? 2;
    _extBtnForceModeValue = data.extBtnForceModeValue  ?? 2;
    _tcForceMode = data.tcForceMode || false;
    _hazardForceMode = data.hazardForceMode || false;
    _extBtnForceMode = data.extButtonForceMode || false;
    _forceModesPriority = data.forceModesPriority ?? 0;
    _haldexGeneration = data.haldexGeneration || 1;
    _isStandalone = data.isStandalone || false;
    _useCANifAvailable = data.useCANifAvailable || false;
    _disableController = data.disableController || false;

    // parse speed/throttle/lock array from the ESP
    speedHeader = data.speedArray;
    throttleHeader = data.throttleArray;
    currentLock = data.lockArray;

    // parse steering-angle lock-scale curve from the ESP
    if (Array.isArray(data.steeringArray)) steeringHeader = data.steeringArray;
    if (Array.isArray(data.steeringLockScaleArray)) steeringScale = data.steeringLockScaleArray;

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

    // Steering-angle reduction band on the engagement bar: shows the portion of
    // the requested lock that the steering angle is holding back (cap -> request).
    {
      const redEl = document.getElementById("engagementReduction");
      const noteEl = document.getElementById("steeringReductionNote");
      const active = data.steeringScaleActive && data.lockRequested != null &&
        data.lockScaled != null && data.lockRequested > data.lockScaled;
      if (active) {
        const req = Math.min(100, Math.max(0, data.lockRequested));
        const cap = Math.min(100, Math.max(0, data.lockScaled));
        if (redEl) {
          redEl.style.display = "";
          redEl.style.left = `${cap}%`;
          redEl.style.width = `${req - cap}%`;
        }
        if (noteEl) {
          noteEl.style.display = "";
          noteEl.textContent = `Steering reduction: ${req}% \u2192 ${cap}% (\u2212${req - cap}%)`;
        }
      } else {
        if (redEl) redEl.style.display = "none";
        if (noteEl) noteEl.style.display = "none";
      }
    }

    // Haldex Data card (Gen 1–4, non-UDS)
    document.getElementById("haldexEngagement").textContent = displayValue(data.haldexEngagement);
    document.getElementById("clutch1Report").textContent   = displayValue(data.clutch1Report);
    document.getElementById("clutch2Report").textContent   = displayValue(data.clutch2Report);
    document.getElementById("tempProtection").textContent  = displayOnOff(data.tempProtection);
    document.getElementById("couplingOpen").textContent    = displayOnOff(data.couplingOpen);
    document.getElementById("speedLimit").textContent      = displayOnOff(data.speedLimit);

    // Per-corner slip [FL, FR, RL, RR]; null (no data / stale) shows as "--".
    {
      const slip = Array.isArray(data.slip) ? data.slip : [null, null, null, null];
      ["slipFL", "slipFR", "slipRL", "slipRR"].forEach((id, i) => {
        const el = document.getElementById(id);
        if (el) el.textContent = (slip[i] === null || slip[i] === undefined) ? "--" : slip[i];
      });
    }

    if (data.mode !== undefined) {
      modeButton(data.mode); // set the mode button - there may be external influences
    }

    const canStatus = document.getElementById("canStatus");
    const chassisOk = data.chassisCAN;
    const haldexOk = data.haldexCAN;
    canStatus.textContent = `CAN: ${chassisOk ? "✓" : "X"} Chassis | ${haldexOk ? "✓" : "X"} Haldex`;

    setStatusPill("diagChassisCAN", chassisOk, "Healthy", "Unhealthy");
    setStatusPill("diagHaldexCAN", haldexOk, "Healthy", "Unhealthy");
    document.getElementById("diagThrottle").textContent = displayValue(
      data.throttle,
    );
    document.getElementById("diagSpeed").textContent = displayValue(data.speed);
    setStatusPill("diagSteeringHealth", data.steeringHealthy, "Healthy", "Unhealthy");
    const steerAngleEl = document.getElementById("diagSteeringAngle");
    if (steerAngleEl) steerAngleEl.textContent = displayValue(data.steeringAngle);
    setStatusPill("diagAsrStatus", data.asrOn, "On", "Off");
    setStatusPill("diagTcStatus", data.tcOn, "On", "Off");
    setStatusPill("diagHazardActive", data.hazardActive, "On", "Off");
    setStatusPill("diagBrakeIn", data.brakeIn, "On", "Off");
    setStatusPill("diagBrakeOut", data.brakeOut, "On", "Off");
    setStatusPill("diagHandbrakeIn", data.handbrakeIn, "On", "Off");
    setStatusPill("diagHandbrakeOut", data.handbrakeOut, "On", "Off");
    setStatusPill("diagBrakeFromCAN", data.brakeFromCAN, "On", "Off");
    setStatusPill("diagHandbrakeFromCAN", data.handbrakeFromCAN, "On", "Off");
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
    const kwpCard = document.getElementById("kwpDataCard");
    if (data.uds) {
      if (haldexDataCard) haldexDataCard.style.display = "none";
      if (kwpCard) kwpCard.style.display = "none";
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
        const udsStatus = document.getElementById("udsStatus");
        if (udsStatus) udsStatus.textContent = data.diagToolActive ? "Paused — external diagnostic tool detected" : "";
      }
    } else if (data.kwp) {
      // Gen2/Gen4 KWP2000-over-TP2.0 live data.
      if (haldexDataCard) haldexDataCard.style.display = "none";
      if (udsCard) udsCard.style.display = "none";
      if (kwpCard) {
        kwpCard.style.display = "";
        document.getElementById("kwpOilTemp").textContent = data.kwp.oilTemp?.toFixed(1) ?? "--";
        document.getElementById("kwpPlateTemp").textContent = data.kwp.plateTemp?.toFixed(1) ?? "--";
        document.getElementById("kwpSupplyVoltage").textContent = data.kwp.supplyVoltage?.toFixed(2) ?? "--";
        document.getElementById("kwpOilPressure").textContent = data.kwp.oilPressure?.toFixed(0) ?? "--";
        document.getElementById("kwpEstTorque").textContent = data.kwp.estTorque?.toFixed(0) ?? "--";
        document.getElementById("kwpClutchDuty").textContent = data.kwp.clutchDuty?.toFixed(0) ?? "--";
        document.getElementById("kwpClutchValveCurrent").textContent = data.kwp.clutchValveCurrent?.toFixed(3) ?? "--";
        const kwpStatus = document.getElementById("kwpStatus");
        if (kwpStatus) {
          kwpStatus.textContent = data.diagToolActive
            ? "Paused — external diagnostic tool detected"
            : (data.kwp.connected ? "Connected" : "Connecting…");
        }
      }
    } else {
      if (haldexDataCard) haldexDataCard.style.display = "";
      if (udsCard) udsCard.style.display = "none";
      if (kwpCard) kwpCard.style.display = "none";
    }

    // Gauge & graph views (all optional, per-tile toggled in Display Options).
    // While the intro sweep owns the strokes, park the latest values for it to
    // land on instead of drawing over the animation.
    if (introSweep.active) {
      introSweep.pending = data;
    } else {
      updateEngagementGauge(data.lockTarget ?? null, data.lockActual ?? null);
    }
    const traceNow = Date.now();
    pushLockSample(data.lockTarget, data.lockActual, traceNow);
    if (gaugePrefs.trace) renderLockTrace(traceNow);
    if (!introSweep.active) updateTileGauges();
    lastDashData = data;
    if (firstStatusResolve) { firstStatusResolve(data); firstStatusResolve = null; }
    updateChartMarker();

    refreshTrace(data); // update the live trace
  } catch (error) {
    console.log("Status failed: " + error.message);
  }
}

function hex2bin(hex) {
  return ("00000000" + parseInt(hex, 16).toString(2)).substr(-8);
}

// Set a diagnostics value as a coloured status pill.
// null/undefined -> orange "unavailable"; truthy -> green; falsy -> red.
function setStatusPill(id, value, okText, badText, naText) {
  const el = document.getElementById(id);
  if (!el) return;
  el.classList.add("pill");
  el.classList.remove("ok", "bad", "warn");
  if (value === undefined || value === null) {
    el.classList.add("warn");
    el.textContent = naText ?? "Unavailable";
  } else if (value) {
    el.classList.add("ok");
    el.textContent = okText;
  } else {
    el.classList.add("bad");
    el.textContent = badText;
  }
}

// Update the header subtitle with current mode and any active force mode.
function updateBannerSubtitle(data) {
  const el = document.getElementById("modeStatus");
  if (!el) return;
  const modeName = MODE_NAMES[data.mode] ?? "Unknown";
  let text = modeName;

  const force = getActiveForceMode(data);
  if (force) text += ` (${force.source}>${MODE_NAMES[force.mode] ?? "Unknown"})`;
  el.textContent = text;
}

function getActiveForceMode(data) {
  const triggers = [
    { source: "TC",  enabled: _tcForceMode,     active: data.tcOn === false,           mode: _tcForceModeValue },
    { source: "Haz", enabled: _hazardForceMode, active: data.hazardActive === true,    mode: _hazardForceModeValue },
    { source: "Ext", enabled: _extBtnForceMode, active: data.extButtonActive === true, mode: _extBtnForceModeValue },
  ];
  const priorityOrders = [
    [1, 0, 2],
    [0, 1, 2],
    [1, 2, 0],
    [0, 2, 1],
    [2, 0, 1],
    [2, 1, 0],
  ];
  const order = priorityOrders[_forceModesPriority] || priorityOrders[0];
  for (const index of order) {
    const trigger = triggers[index];
    if (trigger.enabled && trigger.active) return trigger;
  }
  return null;
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

// render the steering-angle lock-scale editor (angle breakpoints + % multipliers)
function renderSteeringTable() {
  const table = document.getElementById("steeringScaleTable");
  if (!table) return;
  let angleRow = "<tr><th>Angle °</th>";
  let scaleRow = "<tr><th>Lock %</th>";
  for (let i = 0; i < steeringHeader.length; i++) {
    angleRow += `<td><input type="number" min="0" max="1000" class="steering-angle-input" data-idx="${i}" value="${steeringHeader[i]}"></td>`;
    scaleRow += `<td><input type="number" min="0" max="100" class="steering-scale-input" data-idx="${i}" value="${steeringScale[i]}"></td>`;
  }
  table.innerHTML = angleRow + "</tr>" + scaleRow + "</tr>";
}

// save the steering-angle lock-scale curve back to the ESP
async function saveSteeringTable() {
  document.querySelectorAll(".steering-angle-input").forEach((el) => {
    steeringHeader[Number(el.dataset.idx)] = Math.max(0, Number(el.value) || 0);
  });
  document.querySelectorAll(".steering-scale-input").forEach((el) => {
    steeringScale[Number(el.dataset.idx)] = Math.max(0, Math.min(100, Number(el.value) || 0));
  });

  try {
    const response = await fetch("/api/tune", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        steeringArray: steeringHeader,
        steeringLockScaleArray: steeringScale,
      }),
    });

    if (response.ok) {
      showNotification("Steering Scale Saved");
    } else {
      showNotification("Failed to Save", "error");
    }
  } catch (error) {
    console.error("Error saving steering scale:", error);
  }
}

// restore the steering-angle lock-scale curve to defaults (not yet saved)
function restoreSteeringDefaults() {
  steeringHeader = [...defaultSteeringHeader];
  steeringScale = [...defaultSteeringScale];
  renderSteeringTable();
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
    } else {
      updateCachedSetting(key, value);
    }
  } catch (error) {
    console.log("Saving setting failed: " + error.message);
    showNotification("Error saving setting", "error");
  }
}

function updateCachedSetting(key, value) {
  if (key === "tcForceModeValue") _tcForceModeValue = value;
  if (key === "hazardForceModeValue") _hazardForceModeValue = value;
  if (key === "extBtnForceModeValue") _extBtnForceModeValue = value;
  if (key === "forceModesPriority") _forceModesPriority = value;
  if (key === "tcForceMode") _tcForceMode = value;
  if (key === "hazardForceMode") _hazardForceMode = value;
  if (key === "extButtonForceMode") _extBtnForceMode = value;
}

// ---- Frame-edit gating (Diagnostics > Frame Editing) ----------------------
// Render the per-generation editable-frame checkboxes from /api/settings data.
function renderFrameBlocks(blocks) {
  const list = document.getElementById("frameEditList");
  if (!list) return;
  if (!Array.isArray(blocks) || blocks.length === 0) {
    list.innerHTML = '<p class="hint">Not available for this generation.</p>';
    return;
  }
  list.innerHTML = "";
  blocks.forEach((b) => {
    const row = document.createElement("label");
    row.className = "toggle";
    const cb = document.createElement("input");
    cb.type = "checkbox";
    cb.checked = !!b.enabled;
    cb.dataset.bit = b.bit;
    cb.addEventListener("change", () => saveFrameEdit(b.bit, cb.checked));
    const slider = document.createElement("span");
    slider.className = "toggle-slider";
    const span = document.createElement("span");
    span.className = "toggle-label";
    span.textContent = b.name;
    row.appendChild(cb);
    row.appendChild(slider);
    row.appendChild(span);
    list.appendChild(row);
  });
}

// Toggle a single frame-edit block for the current generation.
async function saveFrameEdit(bit, on) {
  try {
    const response = await fetchJson("/api/settings", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ frameEditBit: bit, frameEditOn: on }),
    });
    if (!response.ok) showNotification("Failed to save frame setting", "error");
  } catch (error) {
    showNotification("Error saving frame setting", "error");
  }
}

// Re-fetch settings and re-render the frame checkboxes (e.g. after a generation
// change or a reset-to-defaults).
async function refreshFrameBlocks() {
  try {
    const data = await fetchJson("/api/settings");
    renderFrameBlocks(data.frameBlocks);
  } catch (error) {
    /* leave existing list in place on error */
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
      elem.addEventListener("change", async () => {
        await saveSetting(id, parseInt(elem.value));
        // Generation change alters which frames are editable — refresh the list.
        if (id === "haldexGeneration") refreshFrameBlocks();
      });
    }
  });

  // Frame-edit reset-to-defaults button (Diagnostics > Frame Editing)
  {
    const feReset = document.getElementById("frameEditReset");
    if (feReset) {
      feReset.addEventListener("click", async () => {
        await saveSetting("frameEditReset", true);
        refreshFrameBlocks();
      });
    }
  }

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

  // Gen5 calibration sliders: update the label live while dragging, but only
  // save on release (change) so a live car isn't streamed calibration changes.
  // Reset buttons put the slider back to the firmware default and save it,
  // in case a page scroll nudged the value.
  const bpkRange = document.getElementById("bpkCeilingRange");
  const bpkVal = document.getElementById("bpkCeilingValue");
  const bpkReset = document.getElementById("bpkCeilingResetBtn");
  if (bpkRange) {
    bpkRange.addEventListener("input", () => { if (bpkVal) bpkVal.textContent = bpkRange.value; });
    bpkRange.addEventListener("change", () => { saveSetting("bpkCeilingNm", parseInt(bpkRange.value)); });
    if (bpkReset) {
      bpkReset.addEventListener("click", () => {
        bpkRange.value = 220;
        if (bpkVal) bpkVal.textContent = "220";
        saveSetting("bpkCeilingNm", 220);
      });
    }
  }
  const esp14Range = document.getElementById("esp14MinFloorRange");
  const esp14Val = document.getElementById("esp14MinFloorValue");
  const esp14Reset = document.getElementById("esp14MinFloorResetBtn");
  if (esp14Range) {
    esp14Range.addEventListener("input", () => { if (esp14Val) esp14Val.textContent = esp14Range.value; });
    esp14Range.addEventListener("change", () => { saveSetting("esp14MinFloorPct", parseInt(esp14Range.value)); });
    if (esp14Reset) {
      esp14Reset.addEventListener("click", () => {
        esp14Range.value = 0;
        if (esp14Val) esp14Val.textContent = "0";
        saveSetting("esp14MinFloorPct", 0);
      });
    }
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
    "dangerZoneEnabled",
    "canSleepEnabled",
    "canSleepAggressive",
    "liveDiagEnabled",
    "lockReleaseEnabled",
    "steeringScaleEnabled",
  ];
  checkboxIds.forEach((id) => {
    const elem = document.getElementById(id);
    if (elem) {
      elem.addEventListener("change", async () => {
        if (checkboxCacheMap[id]) checkboxCacheMap[id](elem.checked);
        await saveSetting(id, elem.checked);
        if (id === "isStandalone") refreshFrameBlocks();
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

  // Steering-angle scaling: dim the curve table when the feature is disabled.
  const steeringScaleEnabledElem = document.getElementById("steeringScaleEnabled");
  if (steeringScaleEnabledElem) {
    steeringScaleEnabledElem.addEventListener("change", () => {
      const stbl = document.getElementById("steeringScaleTable");
      if (stbl) stbl.style.opacity = steeringScaleEnabledElem.checked ? "" : "0.4";
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

  // Steering-angle lock-scale editor
  renderSteeringTable();
  const saveSteerBtn = document.getElementById("saveSteeringScale");
  if (saveSteerBtn) saveSteerBtn.addEventListener("click", saveSteeringTable);
  const restoreSteerBtn = document.getElementById("restoreSteeringScale");
  if (restoreSteerBtn)
    restoreSteerBtn.addEventListener("click", restoreSteeringDefaults);
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
  drawTuneChart(); // keep the 3D surface in sync with the edited cell/axis
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
  drawTuneChart();
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
        renderLearnChart(data.table);
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
      renderLearnChart(data.table);
    } else {
      statusText.textContent = "No learn data - static factor active";
      statusText.style.color = "var(--text-dim)";
    }
  });
}

// ---- Long Learn (Settings > Long Learn) -----------------------------------
// Drives /api/longlearn/*: polls status while a run is active, renders the
// tracker + per-block verdicts, keeps the chassis notes on the unit, and
// exports a plain-text report of the car, calibration, block set and sweeps.
const LL_PHASE_NAMES = ["Idle", "Initial Sweep (all blocks on)", "BPK Adjust (torque ceiling)",
                        "Sweeping Blocks", "Confirmation learn on final set", "Complete", "Cancelled", "Failed"];
const LL_RESULT = { 0: ["untested", ""], 1: ["core", "core"], 2: ["needed", "needed"],
                    3: ["not needed", "removed"], 4: ["affects (better without)", "harmful"] };
const LL_SWEEP_KIND = ["baseline", "floor", "block", "final", "bpk ceiling"];

function llScoreText(sc) {
  if (!sc) return "--";
  const eng = sc.engageCF > 100 ? "never engaged" : `engage @CF${sc.engageCF} \u2192 ${sc.engageJump}%`;
  return `${sc.smooth ? "smooth" : "NOT smooth"} \u00b7 reach ${sc.reach}% \u00b7 max step ${sc.maxStep}% \u00b7 ${eng} \u00b7 score ${sc.score}`;
}

function llFmtElapsed(sec) {
  sec = Math.max(0, Math.floor(sec || 0));
  const m = Math.floor(sec / 60), s = sec % 60;
  return `${m}m ${String(s).padStart(2, "0")}s`;
}

function initLongLearn() {
  const statusText   = document.getElementById("longLearnStatusText");
  const progressWrap = document.getElementById("longLearnProgressWrap");
  const progressFill = document.getElementById("longLearnProgressFill");
  const progressLbl  = document.getElementById("longLearnProgressLabel");
  const tracker      = document.getElementById("longLearnTracker");
  const results      = document.getElementById("longLearnResults");
  const btnStart     = document.getElementById("longLearnStart");
  const btnCancel    = document.getElementById("longLearnCancel");
  const btnExport    = document.getElementById("longLearnExport");
  const testAll      = document.getElementById("longLearnTestAll");
  const notes        = document.getElementById("longLearnNotes");
  if (!statusText || !btnStart) return;

  let pollTimer = null;
  let lastStatus = null; // last /api/longlearn/status payload (used by export)

  function setText(id, v) { const e = document.getElementById(id); if (e) e.textContent = v; }

  function renderBlocks(data) {
    if (!results) return;
    const blocks = Array.isArray(data.blocks) ? data.blocks : [];
    if (blocks.length === 0) { results.innerHTML = ""; return; }
    results.innerHTML = "";
    blocks.forEach((b) => {
      const row = document.createElement("div");
      row.className = "ll-block";
      const name = document.createElement("span");
      name.className = "ll-block-name" + (b.enabled ? "" : " off");
      name.textContent = b.name;
      const tag = document.createElement("span");
      let cls = "", txt = "";
      if (data.active && data.currentBit === b.bit) { cls = "testing"; txt = "testing\u2026"; }
      else {
        const r = LL_RESULT[b.result] || LL_RESULT[0];
        txt = r[0]; cls = r[1];
        if (b.result === 0) txt = b.def ? "default" : (data.phase === 0 ? (b.enabled ? "on" : "off") : "queued");
      }
      tag.className = "ll-tag " + cls;
      tag.textContent = txt;
      row.appendChild(name);
      row.appendChild(tag);
      results.appendChild(row);
    });
  }

  function render(data) {
    lastStatus = data;
    const running = !!data.active;
    btnStart.style.display  = running ? "none" : "";
    btnCancel.style.display = running ? "" : "none";
    if (testAll) testAll.disabled = running;

    const showTracker = running || data.phase >= 5;
    progressWrap.style.display = running ? "" : "none";
    tracker.style.display = showTracker ? "" : "none";

    const total = data.sweepTotal || 0, idx = data.sweepIdx || 0;
    const pct = total ? Math.min(100, Math.round((idx / total) * 100)) : 0;
    progressFill.style.width = pct + "%";
    progressLbl.textContent = `${idx}/${total}`;

    setText("llPhase", LL_PHASE_NAMES[data.phase] || "--");
    setText("llSweep", total ? `${Math.min(idx + (running ? 1 : 0), total)} of ${total}` : "--");
    let testing = "--";
    if (running && data.currentBit >= 0 && Array.isArray(data.blocks)) {
      const b = data.blocks.find((x) => x.bit === data.currentBit);
      testing = b ? `without ${b.name}` : `bit ${data.currentBit}`;
    } else if (running && data.phase === 1) {
      testing = "all blocks on";
    } else if (running && data.phase === 2) {
      testing = `raising torque ceiling (${data.bpkNow} Nm)`;
    } else if (running && data.phase === 4) {
      testing = "final block set";
    }
    setText("llTesting", testing);
    const isGen5 = data.generation === 50 || data.generation === 52;
    setText("llFloor", isGen5 ? `${data.floorNow}%` + (data.phase >= 2 ? ` (was ${data.floorStart}%)` : "") : "n/a (Gen5 only)");
    setText("llBpk", isGen5 ? `${data.bpkNow} Nm` + (data.bpkAdjusted ? ` (was ${data.bpkStart} Nm)` : "") +
      (data.phase >= 3 && !running ? ` — Fix Hunting reverted to ${data.fixHunting ? "on" : "off"}, turn it on to use this` : "") : "n/a (Gen5 only)");
    setText("llBaseline", llScoreText(data.baseline));
    setText("llElapsed", llFmtElapsed(data.elapsedS));

    const cf = data.cf ?? 0, eng = data.eng ?? 0;
    const cfFill = document.getElementById("llCFFill"), engFill = document.getElementById("llEngFill");
    if (cfFill) cfFill.style.width = cf + "%";
    if (engFill) engFill.style.width = eng + "%";
    setText("llCFValue", running ? cf + "%" : "--");
    setText("llEngValue", running ? eng + "%" : "--");

    if (running) {
      statusText.textContent = `Long Learn running \u2014 ${LL_PHASE_NAMES[data.phase] || ""}`;
      statusText.style.color = "var(--text-dim)";
    } else if (data.phase === 5) {
      const f = data.final;
      const kept = (data.blocks || []).filter((b) => b.enabled).length;
      statusText.textContent = `Long Learn complete \u2713 \u2014 ${kept} of ${(data.blocks || []).length} blocks enabled, ` +
        (isGen5 ? `PWM floor ${data.floorResult}%, ` : "") +
        (isGen5 && data.bpkAdjusted ? `torque ceiling ${data.bpkNow} Nm (turn Fix Hunting on to use it), ` : "") +
        `final: ${llScoreText(f)}`;
      statusText.style.color = f && f.smooth ? "var(--success)" : "var(--warning)";
    } else if (data.phase === 6) {
      statusText.textContent = "Long Learn cancelled \u2014 previous blocks, floor, torque ceiling and learn table restored";
      statusText.style.color = "var(--warning)";
    } else if (data.phase === 7) {
      statusText.textContent = "Long Learn failed \u2014 no Haldex data during a sweep. Previous settings restored";
      statusText.style.color = "var(--danger)";
    } else {
      statusText.textContent = "Not run yet";
      statusText.style.color = "var(--text-dim)";
    }
    renderBlocks(data);
  }

  async function poll() {
    const data = await fetchJson("/api/longlearn/status");
    if (!data) return;
    const wasRunning = pollTimer !== null;
    render(data);
    if (!data.active && wasRunning) {
      clearInterval(pollTimer);
      pollTimer = null;
      // The run changed the mask / floor / learn table - refresh dependents.
      refreshFrameBlocks();
      const esp14Range = document.getElementById("esp14MinFloorRange");
      const esp14Val = document.getElementById("esp14MinFloorValue");
      if (esp14Range) { esp14Range.value = data.floorNow; if (esp14Val) esp14Val.textContent = data.floorNow; }
      fetchJson("/api/learn/status").then((ls) => { if (ls && ls.tableValid) renderLearnChart(ls.table); });
    }
  }

  function startPolling() {
    if (pollTimer) return;
    pollTimer = setInterval(poll, 1000);
    poll();
  }

  btnStart.addEventListener("click", async () => {
    statusText.textContent = "Starting Long Learn\u2026";
    statusText.style.color = "var(--text-dim)";
    const resp = await fetchJson("/api/longlearn/start", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ testAll: !!(testAll && testAll.checked) }),
    });
    if (!resp || !resp.ok) {
      statusText.textContent = resp && resp.error ? resp.error : "Failed to start Long Learn";
      statusText.style.color = "var(--danger)";
      return;
    }
    startPolling();
  });

  btnCancel.addEventListener("click", async () => {
    await fetchJson("/api/longlearn/cancel", { method: "POST" });
    statusText.textContent = "Cancelling\u2026";
    statusText.style.color = "var(--warning)";
  });

  if (notes) {
    notes.addEventListener("change", () => saveSetting("longLearnNotes", notes.value.slice(0, 200)));
  }

  if (btnExport) {
    btnExport.addEventListener("click", async () => {
      const [settings, ll, learn] = await Promise.all([
        fetchJson("/api/settings"), fetchJson("/api/longlearn/status"), fetchJson("/api/learn/status"),
      ]);
      if (!settings || !ll) { showNotification("Could not read data for export", "error"); return; }
      const txt = buildLongLearnReport(settings, ll, learn, notes ? notes.value : "");
      const gen = settings.haldexGeneration;
      const stamp = new Date().toISOString().slice(0, 19).replace(/[:T]/g, "-");
      const blob = new Blob([txt], { type: "text/plain" });
      const a = document.createElement("a");
      a.href = URL.createObjectURL(blob);
      a.download = `openhaldex-longlearn-gen${gen}-${stamp}.txt`;
      document.body.appendChild(a);
      a.click();
      setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 1000);
    });
  }

  // initial state (and resume polling if a run is already in progress)
  fetchJson("/api/longlearn/status").then((data) => {
    if (!data) return;
    render(data);
    if (data.active) startPolling();
  });
}

// Plain-text report: car notes, calibration, recommended block set (with a
// ready-to-edit checklist + mask), the sweep log and the stored learn table.
function buildLongLearnReport(settings, ll, learn, notesText) {
  const genSel = document.getElementById("haldexGeneration");
  const genName = genSel && genSel.selectedOptions[0] ? genSel.selectedOptions[0].textContent : String(settings.haldexGeneration);
  const isGen5 = settings.haldexGeneration === 50 || settings.haldexGeneration === 52;
  const pad = (v, n) => String(v).padEnd(n);
  const L = [];
  L.push("OpenHaldex-C6 Long Learn Report");
  L.push("=".repeat(60));
  L.push(`Firmware:   ${settings.FW_VERSION || "--"}`);
  L.push(`Board:      rev ${settings.boardRev || "?"}`);
  L.push(`Exported:   ${new Date().toISOString()}`);
  L.push(`Generation: ${settings.haldexGeneration} (${genName})`);
  L.push(`Mode:       ${settings.isStandalone ? "Standalone" : "Normal (passthrough)"}`);
  L.push("");
  L.push("Chassis / car notes:");
  L.push("  " + ((notesText || "").trim() || "(none entered)").replace(/\n/g, "\n  "));
  L.push("");
  L.push("Calibration:");
  L.push(`  Fix Hunting (BPK packing): ${settings.fixHunting ? "on" : "off"}`);
  L.push(`  Lock Calibration (BPK ceiling): ${settings.bpkCeilingNm} Nm`);
  L.push(`  Launch PWM Floor: ${settings.esp14MinFloorPct} %` +
         (ll.phase === 5 && isGen5 ? `  (Long Learn: ${ll.floorStart} % -> ${ll.floorResult} %)` : ""));
  if (ll.phase === 5 && isGen5 && ll.bpkAdjusted) {
    L.push(`  Torque ceiling raised by Long Learn: ${ll.bpkStart} Nm -> ${ll.bpkNow} Nm (Fix Hunting reverted - turn it on to use this)`);
  }
  L.push("");
  L.push(`Long Learn: ${LL_PHASE_NAMES[ll.phase] || "--"}` +
         (ll.phase >= 5 ? `  (${ll.sweepIdx} sweeps, ${llFmtElapsed(ll.elapsedS)}, test-all ${ll.testAll ? "on" : "off"})` : ""));
  if (ll.baseline) L.push(`  Reference (all on): ${llScoreText(ll.baseline)}`);
  if (ll.final)    L.push(`  Final (kept set):   ${llScoreText(ll.final)}`);
  L.push("");
  L.push(`Frame blocks (mask ${ll.mask || "--"}) - [x] = enabled. Edit and re-apply under Diagnostics > Frame Editing:`);
  (ll.blocks || []).forEach((b) => {
    const r = LL_RESULT[b.result] || LL_RESULT[0];
    const verdict = b.result === 0 ? (ll.phase === 0 ? "" : "untested") : r[0];
    L.push(`  [${b.enabled ? "x" : " "}] ${pad(b.name, 22)} bit ${pad(b.bit, 3)} ${pad(b.def ? "default" : "added", 8)} ${verdict}`);
  });
  L.push("");
  if (Array.isArray(ll.sweeps) && ll.sweeps.length) {
    L.push("Sweep log:");
    ll.sweeps.forEach((sw, i) => {
      let what = LL_SWEEP_KIND[sw.kind] || "?";
      if (sw.kind === 2) {
        const b = (ll.blocks || []).find((x) => x.bit === sw.bit);
        what = `without ${b ? b.name : "bit " + sw.bit}`;
      } else if (sw.kind === 4) {
        what = `ceiling ${sw.bpk} Nm`;
      }
      const verdict = sw.kind === 2 ? ((LL_RESULT[sw.verdict] || ["?"])[0]) : (sw.verdict ? "smooth/100%" : "not smooth/100%");
      L.push(`  #${pad(i + 1, 3)} ${pad(what, 28)} floor ${pad(sw.floor + "%", 5)} reach ${pad(sw.reach, 4)} step ${pad(sw.maxStep, 3)} ` +
             `engage@${pad(sw.engageCF > 100 ? "--" : sw.engageCF, 3)}->${pad(sw.engageJump, 3)} score ${pad(sw.score, 3)} => ${verdict}`);
    });
    L.push("");
  }
  if (learn && learn.tableValid && Array.isArray(learn.table)) {
    L.push("Stored learn table (CF% -> engagement%):");
    for (let i = 0; i < learn.table.length; i += 10) {
      L.push("  " + learn.table.slice(i, i + 10).map((v, j) => `${pad(i + j, 3)}:${pad(v, 3)}`).join(" "));
    }
  } else {
    L.push("Stored learn table: none (static factor active)");
  }
  L.push("");
  return L.join("\n");
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

// initialise OTA page (safety-gated /ota endpoints; firmware + filesystem)
function initOtaPage() {
  const chip = (k, v, cls) => `<div class="chip ${cls || ""}"><div class="k">${k}</div><div class="v">${v}</div></div>`;

  // OTA sequence tracking: filesystem first, then firmware. Completed steps
  // persist in localStorage so the highlight survives the auto-reboot.
  const OTA_STEPS_KEY = "oh_ota_steps";
  function otaLoadDone() {
    try {
      const raw = JSON.parse(localStorage.getItem(OTA_STEPS_KEY) || "{}");
      if (!raw.ts || Date.now() - raw.ts > 15 * 60 * 1000) return [];
      return Array.isArray(raw.done) ? raw.done : [];
    } catch (e) { return []; }
  }
  function otaSaveDone(done) {
    localStorage.setItem(OTA_STEPS_KEY, JSON.stringify({ done, ts: Date.now() }));
  }
  function renderOtaSteps() {
    const sel = document.getElementById("otaType");
    const cur = sel ? sel.value : "filesystem";
    const done = otaLoadDone();
    document.querySelectorAll("#otaSteps .ota-step").forEach((el) => {
      const s = el.dataset.step;
      el.classList.toggle("done", done.includes(s));
      el.classList.toggle("active", s === cur && !done.includes(s));
    });
  }
  function otaMarkDone(type) {
    const done = otaLoadDone();
    if (!done.includes(type)) done.push(type);
    otaSaveDone(done);
    renderOtaSteps();
  }

  function loadInfo() {
    fetchJson("/ota/info").then((i) => {
      if (!i) return;
      const set = (id, v) => { const e = document.getElementById(id); if (e) e.textContent = v || "--"; };
      set("otaFwVersion", i.version + (i.fsVersion && i.fsVersion !== "--" && i.fsVersion !== i.version ? " (web UI " + i.fsVersion + ")" : ""));
      set("otaChip", (i.chipModel || "") + (i.chipRevision ? " rev " + i.chipRevision : ""));
      set("otaPartition", i.partition);
    });
  }

  function loadSafety() {
    const wrap = document.getElementById("otaSafety");
    if (!wrap) return;
    fetchJson("/ota/check").then((s) => {
      if (!s) { wrap.innerHTML = chip("Update", "Offline", "off"); return; }
      wrap.innerHTML =
        chip("Update", s.allowed ? "Allowed" : "Blocked", s.allowed ? "on" : "off") +
        chip("Speed", (s.speed ?? 0) + " km/h", s.speed > 0 ? "off" : "on") +
        chip("CAN", s.canInitialized ? "Ready" : "Not ready", s.canInitialized ? "on" : "off") +
        chip("Reason", s.reason || "--", s.allowed ? "" : "warn");
    });
  }

  // Upload core: POSTs a Blob to the safety-gated OTA endpoint and resolves on
  // 200 / rejects with a user-facing message otherwise. `sha256` (optional) is
  // passed to the device, which hashes the stream and refuses to activate an
  // image that doesn't match.
  function otaUploadBlob(type, blob, filename, opts) {
    opts = opts || {};
    const isFs = type === "filesystem";
    let url = isFs ? "/ota/update/fs" : "/ota/update";
    if (opts.sha256) url += "?sha256=" + encodeURIComponent(opts.sha256);
    const data = new FormData();
    data.append(isFs ? "filesystem" : "firmware", blob, filename);
    return new Promise((resolve, reject) => {
      const xhr = new XMLHttpRequest();
      xhr.open("POST", url);
      xhr.withCredentials = true;
      xhr.upload.addEventListener("progress", (e) => {
        if (e.lengthComputable && opts.onProgress) opts.onProgress(e.loaded / e.total);
      });
      xhr.addEventListener("load", () => {
        if (xhr.status === 200) resolve(xhr.responseText);
        else if (xhr.status === 403) reject(new Error("Blocked: system not safe for update."));
        else if (xhr.status === 401) reject(new Error("Authentication required or failed."));
        else if (xhr.status === 400) reject(new Error(xhr.responseText || "Image rejected by device."));
        else reject(new Error("Update failed (" + xhr.status + ")."));
      });
      xhr.addEventListener("error", () => reject(new Error("Upload failed. Check connection and retry.")));
      xhr.send(data);
    });
  }
  function upload() {
    const fileInput = document.getElementById("otaBin");
    const status = document.getElementById("otaStatus");
    const btn = document.getElementById("otaUploadBtn");
    const type = (document.getElementById("otaType") || {}).value || "firmware";
    if (!fileInput || !fileInput.files.length) { status.textContent = "Pick a .bin file first."; return; }
    const file = fileInput.files[0];
    if (!file.name.toLowerCase().endsWith(".bin")) { status.textContent = "Please choose a .bin file."; return; }
    const wrap = document.getElementById("otaProgressWrap");
    const bar = document.getElementById("otaProgressBar");
    const label = document.getElementById("otaProgressLabel");
    const isFs = type === "filesystem";
    const setPct = (f) => {
      const p = Math.round(f * 100);
      if (bar) bar.style.width = p + "%";
      if (label) label.textContent = p + "%";
    };
    if (btn) btn.disabled = true;
    if (wrap) wrap.hidden = false;
    setPct(0);
    status.textContent = "Uploading " + type + "\u2026";
    otaUploadBlob(type, file, file.name, { onProgress: setPct }).then(() => {
      setPct(1);
      otaMarkDone(type);
      if (isFs) {
        // filesystem does not reboot — advance to the firmware step
        const sel = document.getElementById("otaType");
        if (sel) sel.value = "firmware";
        renderOtaSteps();
        status.textContent = "Filesystem updated. Now upload the firmware.";
        if (wrap) wrap.hidden = true;
        if (btn) btn.disabled = false;
        if (fileInput) fileInput.value = "";
      } else {
        status.textContent = "Update complete. Device rebooting\u2026";
      }
    }).catch((err) => {
      status.textContent = err.message;
      if (wrap) wrap.hidden = true;
      if (btn) btn.disabled = false;
    });
  }

  const btn = document.getElementById("otaUploadBtn");
  if (btn) btn.addEventListener("click", upload);
  const typeSel = document.getElementById("otaType");
  if (typeSel) {
    // after a filesystem upload + reboot, resume on the firmware step
    const done = otaLoadDone();
    if (done.includes("filesystem") && !done.includes("firmware")) typeSel.value = "firmware";
    typeSel.addEventListener("change", renderOtaSteps);
  }
  renderOtaSteps();
  loadInfo();
  loadSafety();
  setInterval(loadSafety, 3000);
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

/* ============================================================================
   Gauges & graphs
   ----------------------------------------------------------------------------
   Optional dial-gauge and chart views for the dashboard. Per-tile numeric vs.
   gauge choice, the engagement arc gauge, a rolling lock-response strip chart,
   the learn calibration chart and the read-only expert-map 3D surface. All
   drawn as hand-rolled SVG (no libraries) so the UI still ships from flash.
   ========================================================================= */

// Resolve the theme's CSS custom properties at runtime so string-built SVG
// (which does not honour var() in presentation attributes) still tracks the
// active theme instead of hardcoding hex.
let _themeCache = null;
function themeColors() {
  if (_themeCache) return _themeCache;
  const cs = getComputedStyle(document.documentElement);
  const v = (name, fb) => (cs.getPropertyValue(name).trim() || fb);
  _themeCache = {
    accent:  v("--accent",   "#24d1c4"),
    accent2: v("--accent-2", "#ffb02e"),
    danger:  v("--danger",   "#ff4d5e"),
    ok:      v("--ok",       "#3fd07f"),
    line:    v("--line",     "#24303b"),
    muted:   v("--muted",    "#7d8ea0"),
    text:    v("--text",     "#e6edf3"),
    panel:   v("--panel",    "#131a22"),
    panel2:  v("--panel-2",  "#182129"),
    bg:      v("--bg",       "#0b0f14"),
  };
  return _themeCache;
}

// Catalogue of tiles that can render as a dial gauge: id -> range/unit. The id
// matches the .gauge-value element the poll already writes, so the gauge reads
// its value straight from that text and needs no extra plumbing.
const GAUGE_TILES = [
  { id: "speed",            label: "Speed",        min: 0, max: 260,  unit: "km/h" },
  { id: "throttle",         label: "Throttle",     min: 0, max: 100,  unit: "%" },
  { id: "rpm",              label: "RPM",          min: 0, max: 8000, unit: "rpm" },
  { id: "boost",            label: "Boost",        min: 0, max: 2500, unit: "mbar" },
  { id: "haldexEngagement", label: "Engagement",   min: 0, max: 100,  unit: "%" },
  { id: "clutch1Report",    label: "Clutch 1",     min: 0, max: 255,  unit: "" },
  { id: "clutch2Report",    label: "Clutch 2",     min: 0, max: 255,  unit: "" },
  { id: "udsClutchTemp",    label: "Clutch Temp",  min: 0, max: 150,  unit: "\u00b0C" },
  { id: "udsModuleTemp",    label: "Module Temp",  min: 0, max: 150,  unit: "\u00b0C" },
  { id: "udsCoolingFinTemp",label: "Fin Temp",     min: 0, max: 150,  unit: "\u00b0C" },
  { id: "udsClutchCurrent", label: "Clutch Cur.",  min: 0, max: 5,    unit: "A" },
  { id: "udsClutchPWM",     label: "PWM",          min: 0, max: 100,  unit: "%" },
  { id: "udsBlockagePct",   label: "Blockage",     min: 0, max: 100,  unit: "%" },
  { id: "slipFL",           label: "Slip FL",      min: -50, max: 50, unit: "%" },
  { id: "slipFR",           label: "Slip FR",      min: -50, max: 50, unit: "%" },
  { id: "slipRL",           label: "Slip RL",      min: -50, max: 50, unit: "%" },
  { id: "slipRR",           label: "Slip RR",      min: -50, max: 50, unit: "%" },
];

// 270-degree dial geometry: a circle stroked over three quarters of its
// circumference, rotated so the gap sits at the bottom.
const TG_R = 40;
const TG_CIRC = 2 * Math.PI * TG_R;
const TG_ARC = TG_CIRC * 0.75;
const TG_GAP = TG_CIRC - TG_ARC;

const GAUGE_PREFS_KEY = "ohGaugePrefs";
// Showcase the feature on first load: live-data tiles and the engagement gauge
// default on, numbers everywhere else. All of it is per-tile revertible.
const GAUGE_DEFAULTS = {
  tiles: ["speed", "throttle", "rpm", "boost"],
  engagement: true,
  trace: true,
};

let gaugePrefs = loadGaugePrefs();

function loadGaugePrefs() {
  try {
    const raw = localStorage.getItem(GAUGE_PREFS_KEY);
    if (raw) {
      const p = JSON.parse(raw);
      return {
        tiles: Array.isArray(p.tiles) ? p.tiles : GAUGE_DEFAULTS.tiles.slice(),
        engagement: p.engagement !== undefined ? !!p.engagement : GAUGE_DEFAULTS.engagement,
        trace: p.trace !== undefined ? !!p.trace : GAUGE_DEFAULTS.trace,
      };
    }
  } catch (e) {
    /* fall through to defaults */
  }
  return {
    tiles: GAUGE_DEFAULTS.tiles.slice(),
    engagement: GAUGE_DEFAULTS.engagement,
    trace: GAUGE_DEFAULTS.trace,
  };
}

function saveGaugePrefs() {
  try {
    localStorage.setItem(GAUGE_PREFS_KEY, JSON.stringify(gaugePrefs));
  } catch (e) {
    /* storage blocked - prefs just won't persist */
  }
}

// Inject the dial SVG (once) into a tile and remember its parts. Colours come
// from CSS classes so the DOM SVG tracks the theme via var() directly.
function ensureTileGauge(tile) {
  if (tile.querySelector(".tile-gauge")) return;
  const wrap = document.createElement("div");
  wrap.className = "tile-gauge";
  wrap.innerHTML =
    `<svg viewBox="0 0 100 100" aria-hidden="true">` +
    `<circle class="tg-track" cx="50" cy="50" r="${TG_R}" transform="rotate(135 50 50)" ` +
    `stroke-dasharray="${TG_ARC.toFixed(2)} ${TG_GAP.toFixed(2)}"/>` +
    `<circle class="tg-fill" cx="50" cy="50" r="${TG_R}" transform="rotate(135 50 50)" ` +
    `stroke-dasharray="0 ${TG_CIRC.toFixed(2)}"/>` +
    `<text class="tg-val" x="50" y="52" text-anchor="middle">--</text>` +
    `<text class="tg-unit" x="50" y="66" text-anchor="middle"></text>` +
    `<text class="tg-min" x="24" y="92" text-anchor="middle">0</text>` +
    `<text class="tg-max" x="76" y="92" text-anchor="middle">0</text>` +
    `</svg>`;
  tile.appendChild(wrap);
}

// Reflect gaugePrefs onto the DOM: toggle the .as-gauge class per tile and the
// engagement gauge / bar / trace visibility.
function applyGaugePrefs() {
  GAUGE_TILES.forEach((t) => {
    const el = document.getElementById(t.id);
    if (!el) return;
    const tile = el.closest(".gauge");
    if (!tile) return;
    ensureTileGauge(tile);
    const on = gaugePrefs.tiles.includes(t.id);
    tile.classList.toggle("as-gauge", on);
    if (on) {
      const unitEl = tile.querySelector(".tg-unit");
      if (unitEl) unitEl.textContent = t.unit;
      const minEl = tile.querySelector(".tg-min");
      const maxEl = tile.querySelector(".tg-max");
      if (minEl) minEl.textContent = t.min;
      if (maxEl) maxEl.textContent = t.max;
    }
  });

  const gWrap = document.getElementById("engagementGaugeWrap");
  const bWrap = document.getElementById("engagementBarWrap");
  if (gWrap) gWrap.style.display = gaugePrefs.engagement ? "" : "none";
  if (bWrap) bWrap.style.display = gaugePrefs.engagement ? "none" : "";

  const tWrap = document.getElementById("lockTraceWrap");
  if (tWrap) tWrap.style.display = gaugePrefs.trace ? "" : "none";
}

// Redraw every tile currently in gauge mode from the value the poll just wrote.
function updateTileGauges() {
  GAUGE_TILES.forEach((t) => {
    const el = document.getElementById(t.id);
    if (!el) return;
    const tile = el.closest(".gauge");
    if (!tile || !tile.classList.contains("as-gauge")) return;
    const raw = parseFloat(el.textContent);
    const valEl = tile.querySelector(".tg-val");
    const fillEl = tile.querySelector(".tg-fill");
    if (!valEl || !fillEl) return;
    // Mirror any numeric test/warn highlight onto the gauge.
    const gaugeWrap = tile.querySelector(".tile-gauge");
    if (gaugeWrap) gaugeWrap.classList.toggle("warn", el.style.color === "orange");
    if (Number.isNaN(raw)) {
      valEl.textContent = "--";
      fillEl.style.strokeDasharray = `0 ${TG_CIRC.toFixed(2)}`;
      return;
    }
    valEl.textContent = el.textContent;
    // Grow a single arc from the start up to the value (do NOT animate offset,
    // which would just rotate the whole 3/4 ring instead of filling it).
    const frac = Math.max(0, Math.min(1, (raw - t.min) / (t.max - t.min || 1)));
    fillEl.style.strokeDasharray = `${(TG_ARC * frac).toFixed(2)} ${TG_CIRC.toFixed(2)}`;
  });
}

// Semi-circular engagement gauge: the fill sweeps with ACTUAL engagement, the
// tick marks the TARGET, so the lag between them reads at a glance.
const EG_ARC_LEN = Math.PI * 80; // 180-degree track length (radius 80)
function updateEngagementGauge(target, actual, instant) {
  const fill = document.getElementById("gaugeArcFill");
  const tick = document.getElementById("gaugeTargetTick");
  const val = document.getElementById("gaugeCenterValue");
  if (!fill) return;
  const a = actual === null || actual === undefined ? 0 : Math.max(0, Math.min(100, Number(actual) || 0));
  // `instant` writes the arc without the CSS transition - used for the very
  // first draw, where the path is otherwise fully stroked and animates to
  // empty before anything has even loaded.
  if (instant) fill.style.transition = "none";
  fill.style.strokeDasharray = EG_ARC_LEN.toFixed(2);
  fill.style.strokeDashoffset = (EG_ARC_LEN * (1 - a / 100)).toFixed(2);
  if (instant) { void fill.getBoundingClientRect(); fill.style.transition = ""; }
  if (val) val.textContent = actual === null || actual === undefined ? "--" : Math.round(a);
  if (tick) {
    if (target === null || target === undefined || Number.isNaN(Number(target))) {
      tick.setAttribute("visibility", "hidden");
    } else {
      const tgt = Math.max(0, Math.min(100, Number(target)));
      tick.setAttribute("transform", `rotate(${tgt * 1.8} 100 100)`);
      tick.setAttribute("visibility", "visible");
    }
  }
}

// ---- Intro needle sweep -------------------------------------------------
// One slow, deliberate 0 -> 100 -> live sweep of the engagement arc and any
// tiles in gauge mode, like a cluster on ignition. It is the LAST thing to
// happen on page load: it waits for the window to finish loading and for the
// first dashboard poll to land, so it settles on the real value rather than
// an empty arc. While it runs, live polls are held back (the latest one is
// kept and applied at the end) so the two never fight over the same stroke.
const SWEEP_UP_MS = 900;    // 0 -> 100
const SWEEP_DOWN_MS = 800;  // 100 -> live value
const SWEEP_WAIT_MS = 4000; // give up waiting for the first poll after this
const introSweep = { active: false, pending: null };
let firstStatusResolve = null;
const firstStatus = new Promise((resolve) => { firstStatusResolve = resolve; });

function startIntroSweep() {
  introSweep.active = true; // hold the gauges still until we sweep
  const loaded = new Promise((resolve) => {
    if (document.readyState === "complete") resolve();
    else window.addEventListener("load", resolve, { once: true });
  });
  const firstOrTimeout = Promise.race([firstStatus, new Promise((r) => setTimeout(() => r(null), SWEEP_WAIT_MS))]);
  Promise.all([loaded, firstOrTimeout]).then(([, data]) => runIntroSweep(data));
}

function runIntroSweep(data) {
  const fill = document.getElementById("gaugeArcFill");
  const tick = document.getElementById("gaugeTargetTick");
  const arcOn = !!fill && gaugePrefs.engagement;

  // Tiles currently drawn as gauges, with the fraction each should land on.
  const tiles = [];
  GAUGE_TILES.forEach((t) => {
    const el = document.getElementById(t.id);
    const tile = el && el.closest(".gauge");
    if (!tile || !tile.classList.contains("as-gauge")) return;
    const fillEl = tile.querySelector(".tg-fill");
    if (!fillEl) return;
    const raw = parseFloat(el.textContent);
    const end = Number.isNaN(raw) ? 0 : Math.max(0, Math.min(1, (raw - t.min) / (t.max - t.min || 1)));
    tiles.push({ fillEl, end });
  });

  const finish = () => {
    introSweep.active = false;
    if (fill) fill.style.transition = "";
    tiles.forEach((t) => { t.fillEl.style.transition = ""; });
    const d = introSweep.pending || data || {};
    introSweep.pending = null;
    updateEngagementGauge(d.lockTarget ?? null, d.lockActual ?? null);
    updateTileGauges();
  };
  if (!arcOn && !tiles.length) { finish(); return; }

  const live = introSweep.pending || data || {};
  const a = live.lockActual === null || live.lockActual === undefined ? 0 : Math.max(0, Math.min(100, Number(live.lockActual) || 0));
  const arcEnd = a / 100;

  // Drive the strokes by hand: the CSS transition would smear every frame.
  if (arcOn) { fill.style.transition = "none"; fill.style.strokeDasharray = EG_ARC_LEN.toFixed(2); }
  if (tick) tick.setAttribute("visibility", "hidden");
  tiles.forEach((t) => { t.fillEl.style.transition = "none"; });

  const easeInOut = (x) => (x < 0.5 ? 4 * x * x * x : 1 - Math.pow(-2 * x + 2, 3) / 2);
  const paint = (frac, tileScale) => {
    if (arcOn) fill.style.strokeDashoffset = (EG_ARC_LEN * (1 - frac)).toFixed(2);
    tiles.forEach((t) => {
      const f = tileScale === null ? frac : t.end + (1 - t.end) * tileScale;
      t.fillEl.style.strokeDasharray = `${(TG_ARC * f).toFixed(2)} ${TG_CIRC.toFixed(2)}`;
    });
  };

  const t0 = performance.now();
  const frame = (now) => {
    const t = now - t0;
    if (t < SWEEP_UP_MS) {
      paint(easeInOut(t / SWEEP_UP_MS), null);
    } else if (t < SWEEP_UP_MS + SWEEP_DOWN_MS) {
      // 1 -> end, each gauge to its own value
      const k = 1 - easeInOut((t - SWEEP_UP_MS) / SWEEP_DOWN_MS);
      paint(arcEnd + (1 - arcEnd) * k, k);
    } else {
      paint(arcEnd, 0);
      finish();
      return;
    }
    requestAnimationFrame(frame);
  };
  requestAnimationFrame(frame);
}

// ---- Live lock-response trace ----------------------------------------------
// Rolling time-history of lock target vs actual engagement; poll-fed so no
// extra device load. A missing value breaks its line rather than plotting 0.
const TRACE_WINDOW_MS = 15000;
const lockTrace = [];

function resetLockTrace() {
  lockTrace.length = 0;
  renderLockTrace(Date.now());
}

function pushLockSample(target, actual, now) {
  const hasT = target !== undefined && target !== null && Number.isFinite(Number(target));
  const hasA = actual !== undefined && actual !== null && Number.isFinite(Number(actual));
  if (!hasT && !hasA) return;
  const t = hasT ? Math.max(0, Math.min(100, Number(target))) : null;
  const a = hasA ? Math.max(0, Math.min(100, Number(actual))) : null;
  lockTrace.push({ t: now, target: t, actual: a });
  const cutoff = now - TRACE_WINDOW_MS;
  let firstKept = 0;
  while (firstKept < lockTrace.length - 1 && lockTrace[firstKept + 1].t < cutoff) firstKept++;
  if (firstKept > 0) lockTrace.splice(0, firstKept);
}

function renderLockTrace(now) {
  const svg = document.getElementById("lockTraceSvg");
  if (!svg) return;
  const c = themeColors();
  const W = 320, H = 150;
  const padL = 26, padR = 6, padT = 8, padB = 18;
  const plotW = W - padL - padR;
  const plotH = H - padT - padB;
  const baseY = padT + plotH;
  const windowStart = now - TRACE_WINDOW_MS;
  const xPix = (t) => Math.max(padL, Math.min(W - padR, padL + ((t - windowStart) / TRACE_WINDOW_MS) * plotW));
  const yPix = (v) => padT + (1 - Math.max(0, Math.min(100, Number(v) || 0)) / 100) * plotH;

  let out = "";
  for (let g = 0; g <= 100; g += 25) {
    const y = yPix(g);
    out += `<line x1="${padL}" y1="${y.toFixed(1)}" x2="${W - padR}" y2="${y.toFixed(1)}" stroke="${c.line}" stroke-width="0.5"/>`;
    out += `<text x="${padL - 4}" y="${(y + 3).toFixed(1)}" fill="${c.muted}" font-size="8" text-anchor="end">${g}</text>`;
  }
  out += `<text x="${padL}" y="${H - 5}" fill="${c.muted}" font-size="8" text-anchor="start">-15s</text>`;
  out += `<text x="${W - padR}" y="${H - 5}" fill="${c.muted}" font-size="8" text-anchor="end">now</text>`;

  const pts = lockTrace;
  if (pts.length >= 2) {
    let aSeg = [];
    const flushActual = () => {
      if (aSeg.length >= 2) {
        const line = aSeg.map((p) => `${xPix(p.t).toFixed(1)},${yPix(p.actual).toFixed(1)}`).join(" ");
        const x0 = xPix(aSeg[0].t).toFixed(1);
        const xN = xPix(aSeg[aSeg.length - 1].t).toFixed(1);
        out += `<polygon points="${x0},${baseY.toFixed(1)} ${line} ${xN},${baseY.toFixed(1)}" fill="${c.accent}" fill-opacity="0.15" stroke="none"/>`;
        out += `<polyline points="${line}" fill="none" stroke="${c.accent}" stroke-width="2" stroke-linejoin="round" stroke-linecap="round"/>`;
      }
      aSeg = [];
    };
    for (const p of pts) {
      if (p.actual === null) { flushActual(); continue; }
      aSeg.push(p);
    }
    flushActual();

    let seg = "";
    for (const p of pts) {
      if (p.target === null) {
        if (seg.trim()) { out += `<polyline points="${seg.trim()}" fill="none" stroke="${c.accent2}" stroke-width="1.5" stroke-dasharray="4 3" stroke-linejoin="round"/>`; seg = ""; }
        continue;
      }
      seg += `${xPix(p.t).toFixed(1)},${yPix(p.target).toFixed(1)} `;
    }
    if (seg.trim()) out += `<polyline points="${seg.trim()}" fill="none" stroke="${c.accent2}" stroke-width="1.5" stroke-dasharray="4 3" stroke-linejoin="round"/>`;
  }
  svg.innerHTML = out;
}

// ---- Learn Haldex calibration chart ----------------------------------------
// X = commanded correction factor (0..100%), Y = measured engagement (0..100%),
// with a 1:1 reference diagonal. Render-only from the learn table; hidden until
// the device returns one.
function renderLearnChart(table) {
  const wrap = document.getElementById("learnChartWrap");
  const svg = document.getElementById("learnChartSvg");
  if (!svg || !wrap) return;
  if (!Array.isArray(table) || table.length < 2) {
    wrap.style.display = "none";
    svg.innerHTML = "";
    return;
  }
  wrap.style.display = "";
  const c = themeColors();
  const W = 320, H = 200;
  const padL = 26, padR = 8, padT = 8, padB = 20;
  const plotW = W - padL - padR;
  const plotH = H - padT - padB;
  const n = table.length;
  const xOf = (cf) => padL + (Math.max(0, Math.min(100, cf)) / 100) * plotW;
  const yOf = (eng) => padT + (1 - Math.max(0, Math.min(100, eng)) / 100) * plotH;

  let out = "";
  for (let g = 0; g <= 100; g += 25) {
    const y = yOf(g);
    out += `<line x1="${padL}" y1="${y.toFixed(1)}" x2="${(W - padR).toFixed(1)}" y2="${y.toFixed(1)}" stroke="${c.line}" stroke-width="0.5"/>`;
    out += `<text x="${(padL - 4).toFixed(1)}" y="${(y + 3).toFixed(1)}" fill="${c.muted}" font-size="8" text-anchor="end">${g}</text>`;
    const x = xOf(g);
    out += `<text x="${x.toFixed(1)}" y="${H - 6}" fill="${c.muted}" font-size="8" text-anchor="middle">${g}</text>`;
  }
  out += `<line x1="${xOf(0).toFixed(1)}" y1="${yOf(0).toFixed(1)}" x2="${xOf(100).toFixed(1)}" y2="${yOf(100).toFixed(1)}" stroke="${c.muted}" stroke-width="1" stroke-dasharray="4 3"/>`;

  let line = "";
  for (let i = 0; i < n; i++) {
    const cf = (i / (n - 1)) * 100;
    const eng = Number(table[i]) || 0;
    line += `${xOf(cf).toFixed(1)},${yOf(eng).toFixed(1)} `;
  }
  out += `<polyline points="${line.trim()}" fill="none" stroke="${c.accent}" stroke-width="2" stroke-linejoin="round" stroke-linecap="round"/>`;
  svg.innerHTML = out;
}

// ---- Expert map 3D surface (read-only) -------------------------------------
// Isometric render of currentLock: speed/throttle on the ground, lock % as
// height, shaded on the editor heat ramp, with a live operating-point dot.
let tuneProject = null;

function heatRGB(frac) {
  const stops = [
    [16, 185, 129],
    [245, 158, 11],
    [220, 38, 38],
  ];
  const f = Math.min(1, Math.max(0, frac));
  const pos = f * (stops.length - 1);
  const i = Math.min(stops.length - 2, Math.floor(pos));
  const t = pos - i;
  const mix = stops[i].map((cc, k) => Math.round(cc + (stops[i + 1][k] - cc) * t));
  return `${mix[0]},${mix[1]},${mix[2]}`;
}

function fracIndex(value, header) {
  if (value <= header[0]) return 0;
  const last = header.length - 1;
  if (value >= header[last]) return last;
  for (let i = 0; i < last; i++) {
    if (value < header[i + 1]) return i + (value - header[i]) / (header[i + 1] - header[i]);
  }
  return last;
}

function drawTuneChart() {
  const host = document.getElementById("tuneChart");
  if (!host) return;
  if (!Array.isArray(currentLock) || !Array.isArray(speedHeader) || !Array.isArray(throttleHeader)) return;
  const c = themeColors();
  const cols = speedHeader.length;
  const rows = throttleHeader.length;
  const W = 340, H = 250;
  const yaw = -30 * Math.PI / 180;
  const pitch = 26 * Math.PI / 180;
  const sinYaw = Math.sin(yaw), cosYaw = Math.cos(yaw);
  const sinPit = Math.sin(pitch), cosPit = Math.cos(pitch);
  const midC = (cols - 1) / 2, midR = (rows - 1) / 2;
  const heightUnits = (cols - 1) * 0.6;
  const camDist = (cols - 1) * 2.4;

  const world = (col, row, lock) => {
    const gx = col - midC;
    const gy = row - midR;
    const gz = (lock / 100) * heightUnits;
    const x = gx * cosYaw + gy * sinYaw;
    const y = -gx * sinYaw + gy * cosYaw;
    const up = gz * cosPit + y * sinPit;
    const depth = y * cosPit - gz * sinPit;
    const persp = camDist / (camDist + depth);
    return { x: x * persp, y: -up * persp, depth };
  };

  const pts = [];
  for (let r = 0; r < rows; r++) for (let col = 0; col < cols; col++) pts.push(world(col, r, currentLock[r][col]));
  pts.push(world(0, 0, 0), world(cols - 1, 0, 0), world(0, rows - 1, 0), world(cols - 1, rows - 1, 0));
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  pts.forEach((p) => {
    if (p.x < minX) minX = p.x;
    if (p.x > maxX) maxX = p.x;
    if (p.y < minY) minY = p.y;
    if (p.y > maxY) maxY = p.y;
  });
  const mX = 16, mTop = 12, mBot = 30;
  const scale = Math.min((W - 2 * mX) / (maxX - minX || 1), (H - mTop - mBot) / (maxY - minY || 1));
  const ox = (W - (maxX - minX) * scale) / 2 - minX * scale;
  const oy = mTop - minY * scale;
  tuneProject = (col, r, lock) => {
    const w = world(col, r, lock);
    return { x: ox + w.x * scale, y: oy + w.y * scale };
  };

  let svg = `<svg viewBox="0 0 ${W} ${H}" role="img" aria-label="Lock map surface">`;
  const facets = [];
  for (let r = 0; r < rows - 1; r++) {
    for (let col = 0; col < cols - 1; col++) {
      const depth = (world(col, r, currentLock[r][col]).depth +
        world(col + 1, r, currentLock[r][col + 1]).depth +
        world(col, r + 1, currentLock[r + 1][col]).depth +
        world(col + 1, r + 1, currentLock[r + 1][col + 1]).depth) / 4;
      facets.push({ r, col, depth });
    }
  }
  facets.sort((a, b) => b.depth - a.depth);
  facets.forEach(({ r, col }) => {
    const corners = [[r, col], [r, col + 1], [r + 1, col + 1], [r + 1, col]];
    const poly = corners.map(([rr, cc]) => {
      const p = tuneProject(cc, rr, currentLock[rr][cc]);
      return `${p.x.toFixed(1)},${p.y.toFixed(1)}`;
    }).join(" ");
    const avg = (currentLock[r][col] + currentLock[r][col + 1] + currentLock[r + 1][col] + currentLock[r + 1][col + 1]) / 4;
    svg += `<polygon points="${poly}" fill="rgb(${heatRGB(avg / 100)})" fill-opacity="0.55" stroke="rgba(0,0,0,0.7)" stroke-width="0.5" stroke-linejoin="round"/>`;
  });

  const floorPt = (col, r) => tuneProject(col, r, 0);
  const fc = {
    near: floorPt(0, rows - 1),
    right: floorPt(cols - 1, rows - 1),
    back: floorPt(cols - 1, 0),
    left: floorPt(0, 0),
  };
  const floorLine = (pa, pb) => `<line x1="${pa.x.toFixed(1)}" y1="${pa.y.toFixed(1)}" x2="${pb.x.toFixed(1)}" y2="${pb.y.toFixed(1)}" stroke="${c.muted}" stroke-width="1"/>`;
  svg += floorLine(fc.left, fc.back) + floorLine(fc.back, fc.right) + floorLine(fc.right, fc.near) + floorLine(fc.near, fc.left);

  const speedMid = tuneProject((cols - 1) / 2, rows - 1, 0);
  const throttleMid = tuneProject(cols - 1, (rows - 1) / 2, 0);
  svg += `<text x="${speedMid.x.toFixed(1)}" y="${(speedMid.y + 16).toFixed(1)}" text-anchor="middle" font-size="8" font-weight="700" fill="${c.muted}">SPEED (KM/H)</text>`;
  svg += `<text x="${(throttleMid.x + 6).toFixed(1)}" y="${(throttleMid.y + 14).toFixed(1)}" text-anchor="start" font-size="8" font-weight="700" fill="${c.muted}">THROTTLE (%)</text>`;

  svg += `<line id="chartMarkerStem" stroke="${c.text}" stroke-width="1" stroke-dasharray="2 2" visibility="hidden"/>`;
  svg += `<circle id="chartMarker" r="4" fill="${c.danger}" stroke="${c.text}" stroke-width="1.5" visibility="hidden"/>`;
  svg += `</svg>`;
  host.innerHTML = svg;

  const legend = document.getElementById("tuneChartLegend");
  if (legend) {
    legend.innerHTML =
      `<div class="chart-legend-caption">Lock % (surface height &amp; colour)</div>` +
      `<div class="chart-legend-bar" style="background:linear-gradient(90deg, rgb(${heatRGB(0)}), rgb(${heatRGB(0.5)}), rgb(${heatRGB(1)}))"></div>` +
      `<div class="chart-legend-scale"><span>0</span><span>50</span><span>100</span></div>`;
  }
  updateChartMarker();
}

function updateChartMarker() {
  const marker = document.getElementById("chartMarker");
  const stem = document.getElementById("chartMarkerStem");
  if (!marker || !tuneProject || !lastDashData) return;
  const speed = Number(lastDashData.speed);
  const throttle = Number(lastDashData.throttle);
  if (Number.isNaN(speed) || Number.isNaN(throttle)) {
    marker.setAttribute("visibility", "hidden");
    if (stem) stem.setAttribute("visibility", "hidden");
    return;
  }
  const col = fracIndex(speed, speedHeader);
  const r = fracIndex(throttle, throttleHeader);
  const c0 = Math.floor(col), c1 = Math.min(speedHeader.length - 1, c0 + 1), tc = col - c0;
  const r0 = Math.floor(r), r1 = Math.min(throttleHeader.length - 1, r0 + 1), tr = r - r0;
  const lerp = (a, b, t) => a + (b - a) * t;
  const blend = (q00, q10, q01, q11) => ({
    x: lerp(lerp(q00.x, q10.x, tc), lerp(q01.x, q11.x, tc), tr),
    y: lerp(lerp(q00.y, q10.y, tc), lerp(q01.y, q11.y, tc), tr),
  });
  const corner = (cc, rr) => tuneProject(cc, rr, currentLock[rr][cc]);
  const top = blend(corner(c0, r0), corner(c1, r0), corner(c0, r1), corner(c1, r1));
  const floor = (cc, rr) => tuneProject(cc, rr, 0);
  const foot = blend(floor(c0, r0), floor(c1, r0), floor(c0, r1), floor(c1, r1));
  marker.setAttribute("cx", top.x.toFixed(1));
  marker.setAttribute("cy", top.y.toFixed(1));
  marker.setAttribute("visibility", "visible");
  if (stem) {
    stem.setAttribute("x1", foot.x.toFixed(1));
    stem.setAttribute("y1", foot.y.toFixed(1));
    stem.setAttribute("x2", top.x.toFixed(1));
    stem.setAttribute("y2", top.y.toFixed(1));
    stem.setAttribute("visibility", "visible");
  }
}

// Snapshot of the last poll so the 3D surface dot can move without a full poll.
let lastDashData = null;

// Build the per-tile gauge customizer and apply the saved preferences.
function initGaugeUI() {
  const host = document.getElementById("gaugeCustomizer");
  if (host) {
    host.innerHTML = "";
    GAUGE_TILES.forEach((t) => {
      const label = document.createElement("label");
      label.className = "tile-opt";
      const cb = document.createElement("input");
      cb.type = "checkbox";
      cb.checked = gaugePrefs.tiles.includes(t.id);
      cb.addEventListener("change", () => {
        const set = new Set(gaugePrefs.tiles);
        if (cb.checked) set.add(t.id); else set.delete(t.id);
        gaugePrefs.tiles = [...set];
        saveGaugePrefs();
        applyGaugePrefs();
      });
      const span = document.createElement("span");
      span.textContent = t.label;
      label.appendChild(cb);
      label.appendChild(span);
      host.appendChild(label);
    });
  }

  const engCb = document.getElementById("optEngagementGauge");
  if (engCb) {
    engCb.checked = gaugePrefs.engagement;
    engCb.addEventListener("change", () => {
      gaugePrefs.engagement = engCb.checked;
      saveGaugePrefs();
      applyGaugePrefs();
    });
  }
  const traceCb = document.getElementById("optLockTrace");
  if (traceCb) {
    traceCb.checked = gaugePrefs.trace;
    traceCb.addEventListener("change", () => {
      gaugePrefs.trace = traceCb.checked;
      saveGaugePrefs();
      applyGaugePrefs();
    });
  }

  applyGaugePrefs();
  renderLockTrace(Date.now());
  updateEngagementGauge(null, null, true); // empty arc, drawn instantly - the intro sweep starts from here
}
