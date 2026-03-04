/*
OpenHaldex C6 Firmware
Copyright (c) 2026 Forbes Automotive

This file is part of the OpenHaldex C6 project.

Licensed under the Forbes Automotive Source-Available License (FASL) v1.0.

Personal, educational, and non-commercial use is permitted.
Commercial use, including selling hardware running this firmware,
is strictly prohibited without written permission from Forbes Automotive.

See the LICENSE file in the root of this repository for full license terms.
Project repository: https://github.com/Forbes-Automotive/OpenHaldex-C6
*/

let currentEditCell = null; // global flag for current editted cell

const MODE_NAMES = ["Stock", "FWD", "50:50", "60:40", "75:25", "Expert"]; // mode names as Strings
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
    document.getElementById("forceModeValue").value = data.forceModeValue || 0;

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

    document.getElementById("FW_VERSION").textContent = data.FW_VERSION || "--";

    //document.getElementById('mode').value = data.mode || 1;
    //document.getElementById('lockReleaseRatePerSec').value = data.lockReleaseRatePerSec || 1;

    // bools
    document.getElementById("disableController").checked =
      data.disableController || false;
    document.getElementById("isStandalone").checked =
      data.isStandalone || false;
    document.getElementById("tcForceMode").checked = data.tcForceMode || false;
    document.getElementById("extButtonForceMode").checked =
      data.extButtonForceMode || false;

    document.getElementById("followBrake").checked = data.followBrake || false;
    document.getElementById("invertBrake").checked = data.invertBrake || false;
    document.getElementById("followHandbrake").checked =
      data.followHandbrake || false;
    document.getElementById("invertHandbrake").checked =
      data.invertHandbrake || false;

    document.getElementById("broadcastOpenHaldexOverCAN").checked =
      data.broadcastOpenHaldexOverCAN || false;

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

    document.getElementById("speed").textContent = data.speed || "--";
    document.getElementById("throttle").textContent = data.throttle || "--";
    document.getElementById("rpm").textContent = data.rpm || "--";
    document.getElementById("boost").textContent = data.boost || "--";

    document.getElementById("lockTarget").textContent = data.lockTarget || "0";
    document.getElementById("lockActual").textContent = data.lockActual || "0";
    document.getElementById("engagementFill").style.width =
      `${data.lockActual || 0}%`;

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
    document.getElementById("diagFreeHeap").textContent = Math.round(
      data.freeHeap / 1024,
    );

    document.getElementById("haldexState").textContent = hex2bin(
      data.haldexState,
    );

    refreshTrace(data); // update the live trace
  } catch (error) {
    console.log("Status failed: " + error.message);
  }
}

function hex2bin(hex) {
  return ("00000000" + parseInt(hex, 16).toString(2)).substr(-8);
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

// save current settings
async function saveSettings() {
  const settings = {
    haldexGeneration: parseInt(
      document.getElementById("haldexGeneration").value,
    ),
    forceModeValue: parseInt(document.getElementById("forceModeValue").value),
    disengageUnderSpeed: parseInt(
      document.getElementById("disengageUnderSpeedRange").value,
    ),
    disengageAboveSpeed: parseInt(
      document.getElementById("disengageAboveSpeedRange").value,
    ),
    disableThrottle: parseInt(
      document.getElementById("disableThrottleRange").value,
    ),

    disableController: document.getElementById("disableController").checked,
    isStandalone: document.getElementById("isStandalone").checked,
    analyzerMode: document.getElementById("analyzerMode").checked,

    tcForceMode: document.getElementById("tcForceMode").checked,
    extButtonForceMode: document.getElementById("extButtonForceMode").checked,

    followBrake: document.getElementById("followBrake").checked,
    invertBrake: document.getElementById("invertBrake").checked,
    followHandbrake: document.getElementById("followHandbrake").checked,
    invertHandbrake: document.getElementById("invertHandbrake").checked,

    broadcastOpenHaldexOverCAN: document.getElementById(
      "broadcastOpenHaldexOverCAN",
    ).checked,
  };

  try {
    const response = await fetchJson("/api/settings", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(settings),
    });

    if (response.ok) {
      showNotification("Settings Saved");
    } else {
      showNotification("Failed to Save", "error");
    }
  } catch (error) {
    console.log("Save failed: " + error.message);
  }
  refreshStatus();
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

// initialise initSettings
function initSettings() {
  document
    .getElementById("saveSettings")
    .addEventListener("click", saveSettings); // onclick, do this
  document
    .getElementById("saveSettingsBasic")
    .addEventListener("click", saveSettings); // onclick, do this
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
  const speed = Number(data.speed); // get the current speed from the data
  const throttle = Number(data.throttle); // get the current throttle from the data
  const throttlePos = arrayIndex(throttle, throttleHeader); // find the position in the grid for the current throttle
  const speedPos = arrayIndex(speed, speedHeader); // find the position in the grid for the current speed

  const cell = [...document.querySelectorAll(".map-cell")]; // find all the cells in the grid (as an array)

  for (i in cell) {
    cell[i].classList.remove("activeTrace"); // remove the active trace from all cells (so only the current one is highlighted)
  }

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
