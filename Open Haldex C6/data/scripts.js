function initTheme() {
  const stored = localStorage.getItem("ohTheme");
  const prefersDark =
    window.matchMedia && window.matchMedia("(prefers-color-scheme: dark)").matches;
  const dark = stored ? stored === "dark" : prefersDark;
  document.body.classList.toggle("dark", dark);
  const toggle = document.getElementById("themeSwitch");
  const label = document.getElementById("themeLabel");
  if (toggle && label) {
    toggle.checked = dark;
    label.textContent = dark ? "Dark" : "Light";
    toggle.addEventListener("change", () => {
      const on = toggle.checked;
      document.body.classList.toggle("dark", on);
      localStorage.setItem("ohTheme", on ? "dark" : "light");
      label.textContent = on ? "Dark" : "Light";
    });
  }
}

// Entrypoint: selects per-page initializer using body data-page attribute.
function initApp() {
  initTheme();
  const page = document.body.dataset.page;
  if (page === "home") initHomePage();
  if (page === "map") initMapPage();
  if (page === "canview") initCanviewPage();
  if (page === "diag") initDiagPage();
  if (page === "ota") initOtaPage();
}

document.addEventListener("DOMContentLoaded", initApp);

// Utility: normalize map labels from filesystem paths.
function nameFromPath(path) {
  if (!path) return "";
  const base = path.split("/").pop() || path;
  const dot = base.lastIndexOf(".");
  return dot > 0 ? base.slice(0, dot) : base;
}

// Utility: quick extension extraction for map list display.
function formatFromPath(path) {
  if (!path) return "";
  const dot = path.lastIndexOf(".");
  return dot > 0 ? path.slice(dot + 1) : "";
}

// Shared fetch helper that enforces HTTP status checks before JSON parse.
async function fetchJson(url, options) {
  const res = await fetch(url, options);
  if (!res.ok) throw new Error("HTTP " + res.status);
  return res.json();
}

// Home page controller:
// - controller/broadcast toggles
// - live speed/throttle gauges
// - engagement bar + map quick load
function initHomePage() {
  const statusEl = document.getElementById("status");
  const mapSelect = document.getElementById("mapSelect");
  const haldexGenSelect = document.getElementById("haldexGen");
  const modeStatus = document.getElementById("modeStatus");
  const toggleController = document.getElementById("toggleController");
  const toggleBroadcast = document.getElementById("toggleBroadcast");
  const speedNeedle = document.getElementById("gaugeSpeedNeedle");
  const throttleNeedle = document.getElementById("gaugeThrottleNeedle");
  const speedValueEl = document.getElementById("gaugeSpeedValue");
  const throttleValueEl = document.getElementById("gaugeThrottleValue");
  const engagementBar = document.getElementById("engagementBar");
  const engagementValueEl = document.getElementById("engagementValue");

  function setStatus(msg) {
    statusEl.textContent = msg || "";
  }

  function setGauge(needle, valueEl, value, max) {
    if (!needle || !valueEl) return;
    const v = Math.max(0, Math.min(max, Number(value) || 0));
    const ratio = max > 0 ? v / max : 0;
    const deg = -120 + ratio * 240;
    needle.style.transform = `translateX(-50%) rotate(${deg}deg)`;
    valueEl.textContent = Math.round(v);
  }

  function setEngagement(value) {
    if (!engagementBar) return;
    const min = 30;
    const max = 170;
    const raw = Number(value);
    const clamped = Math.max(min, Math.min(max, Number.isFinite(raw) ? raw : min));
    const pct = max > min ? ((clamped - min) / (max - min)) * 100 : 0;
    engagementBar.style.width = pct.toFixed(1) + "%";
    if (engagementValueEl) engagementValueEl.textContent = "";
  }

  async function refreshStatus() {
    try {
      const data = await fetchJson("/api/status");
      toggleController.checked = !data.disableController;
      toggleBroadcast.checked = !!data.broadcastOpenHaldexOverCAN;
      if (haldexGenSelect && data.haldexGeneration) {
        haldexGenSelect.value = String(data.haldexGeneration);
      }
      const mode = toggleController.checked ? "MAP" : "STOCK";
      modeStatus.textContent = `Mode: ${mode}`;
      const tel = data.telemetry || {};
      setGauge(speedNeedle, speedValueEl, tel.speed, 200);
      setGauge(throttleNeedle, throttleValueEl, tel.throttle, 100);
      // Prefer the same engagement source shown on diagnostics; fall back only if needed.
      const engagementCandidate = [tel.haldexEngagement, tel.act, tel.haldexEngagementRaw]
        .map((v) => Number(v))
        .find((v) => Number.isFinite(v) && v >= 0);
      setEngagement(engagementCandidate);
    } catch (e) {
      modeStatus.textContent = "Status failed: " + e.message;
    }
  }

  async function updateSettings() {
    try {
      await fetchJson("/api/settings", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          disableController: !toggleController.checked,
          broadcastOpenHaldexOverCAN: toggleBroadcast.checked,
        }),
      });
    } catch (e) {
      modeStatus.textContent = "Update failed: " + e.message;
    }
  }

  async function saveHaldexGen() {
    if (!haldexGenSelect) return;
    const gen = parseInt(haldexGenSelect.value, 10);
    if (!gen) return;
    try {
      await fetchJson("/api/settings", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ haldexGeneration: gen }),
      });
    } catch (e) {
      modeStatus.textContent = "Haldex gen update failed: " + e.message;
    }
  }

  async function refreshMapList(selectPath) {
    try {
      const data = await fetchJson("/api/maps");
      const current = data.current || "";
      mapSelect.innerHTML = "";
      (data.maps || []).forEach((entry) => {
        const opt = document.createElement("option");
        opt.value = entry.path;
        opt.textContent = `${entry.name} (${entry.path})`;
        mapSelect.appendChild(opt);
      });
      if (current && !Array.from(mapSelect.options).some((o) => o.value === current)) {
        const opt = document.createElement("option");
        const name = nameFromPath(current) || "current";
        opt.value = current;
        opt.textContent = `${name} (${current})`;
        mapSelect.appendChild(opt);
      }
      const target = selectPath || current;
      if (target) mapSelect.value = target;
    } catch (e) {
      setStatus("Map list failed: " + e.message);
    }
  }

  async function loadSelectedMap() {
    const path = mapSelect.value;
    if (!path) return;
    setStatus("Loading map...");
    try {
      await fetchJson("/api/maps/load", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ path }),
      });
      setStatus("Loaded: " + path);
      refreshMapList(path);
    } catch (e) {
      setStatus("Load failed: " + e.message);
    }
  }

  document.getElementById("btnRefresh").onclick = () => refreshMapList(mapSelect.value);
  document.getElementById("btnLoad").onclick = loadSelectedMap;
  document.getElementById("btnHaldexGen").onclick = saveHaldexGen;
  toggleController.onchange = updateSettings;
  toggleBroadcast.onchange = updateSettings;

  refreshMapList("");
  refreshStatus();
  setInterval(refreshStatus, 1000);
}

// Map editor controller:
// - bin editors + lock table
// - active-cell tracer from live telemetry
// - map list/load/save/import/export operations
function initMapPage() {
  const defaultSpeed = [0, 5, 10, 20, 40, 60, 80, 100, 140];
  const defaultThrottle = [0, 5, 10, 20, 40, 60, 80];
  const defaultLock = [
    [0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 5, 5, 5, 5, 0, 0, 0],
    [0, 5, 10, 15, 15, 10, 5, 0, 0],
    [5, 10, 20, 25, 25, 20, 15, 10, 5],
    [10, 20, 30, 40, 40, 30, 25, 20, 15],
    [20, 30, 45, 60, 60, 50, 40, 30, 20],
  ];

  const state = {
    speed: [...defaultSpeed],
    throttle: [...defaultThrottle],
    lock: defaultLock.map((r) => [...r]),
  };

  let cellInputs = [];
  let activeCell = null;
  const statusEl = document.getElementById("status");
  const speedBinsEl = document.getElementById("speedBins");
  const throttleBinsEl = document.getElementById("throttleBins");
  const tableEl = document.getElementById("mapTable");
  const mapSelect = document.getElementById("mapSelect");

  function setStatus(msg) {
    statusEl.textContent = msg || "";
  }

  function applyCellColor(input, value) {
    const v = Math.max(0, Math.min(100, parseInt(value || 0, 10)));
    const hue = 120 - v * 1.2;
    input.style.backgroundColor = `hsl(${hue}, 70%, 45%)`;
    input.style.color = "#0b0f12";
  }

  function buildBins() {
    speedBinsEl.innerHTML = "";
    throttleBinsEl.innerHTML = "";
    state.speed.forEach((v, i) => {
      const input = document.createElement("input");
      input.type = "number";
      input.value = v;
      input.min = 0;
      input.max = 255;
      input.onchange = () => {
        state.speed[i] = parseInt(input.value || 0, 10);
      };
      speedBinsEl.appendChild(input);
    });
    state.throttle.forEach((v, i) => {
      const input = document.createElement("input");
      input.type = "number";
      input.value = v;
      input.min = 0;
      input.max = 100;
      input.onchange = () => {
        state.throttle[i] = parseInt(input.value || 0, 10);
      };
      throttleBinsEl.appendChild(input);
    });
  }

  function buildTable() {
    tableEl.innerHTML = "";
    const thead = document.createElement("thead");
    const hrow = document.createElement("tr");
    hrow.appendChild(th("Throttle"));
    state.speed.forEach((s) => hrow.appendChild(th("S" + s)));
    thead.appendChild(hrow);
    tableEl.appendChild(thead);

    const tbody = document.createElement("tbody");
    cellInputs = [];
    state.throttle.forEach((t, r) => {
      const row = document.createElement("tr");
      row.appendChild(th("T" + t));
      state.speed.forEach((_, c) => {
        const cell = document.createElement("td");
        const input = document.createElement("input");
        input.type = "number";
        input.min = 0;
        input.max = 100;
        input.value = state.lock[r][c];
        applyCellColor(input, input.value);
        input.onchange = () => {
          const v = parseInt(input.value || 0, 10);
          state.lock[r][c] = v;
          applyCellColor(input, v);
        };
        cell.appendChild(input);
        row.appendChild(cell);
        if (!cellInputs[r]) cellInputs[r] = [];
        cellInputs[r][c] = cell;
      });
      tbody.appendChild(row);
    });
    tableEl.appendChild(tbody);
  }

  function th(text) {
    const el = document.createElement("th");
    el.textContent = text;
    return el;
  }

  function binIndex(value, bins) {
    let idx = 0;
    for (let i = 0; i < bins.length; i++) {
      if (value >= bins[i]) idx = i;
    }
    return idx;
  }

  function setActiveCell(r, c) {
    const cell = (cellInputs[r] || [])[c];
    if (!cell) return;
    if (activeCell && activeCell !== cell) activeCell.classList.remove("map-active");
    activeCell = cell;
    activeCell.classList.add("map-active");
  }

  async function refreshTrace() {
    try {
      const data = await fetchJson("/api/status");
      const telem = data.telemetry || {};
      const speed = Number(telem.speed || 0);
      const throttle = Number(telem.throttle || 0);
      const r = binIndex(throttle, state.throttle);
      const c = binIndex(speed, state.speed);
      setActiveCell(r, c);
    } catch (e) {
      // ignore trace errors
    }
  }

  async function refreshMapList(selectPath) {
    try {
      const data = await fetchJson("/api/maps");
      const current = data.current || "";
      mapSelect.innerHTML = "";
      (data.maps || []).forEach((entry) => {
        const opt = document.createElement("option");
        opt.value = entry.path;
        opt.textContent = `${entry.name} (${entry.path})`;
        mapSelect.appendChild(opt);
      });
      if (current && !Array.from(mapSelect.options).some((o) => o.value === current)) {
        const opt = document.createElement("option");
        const name = nameFromPath(current) || "current";
        opt.value = current;
        opt.textContent = `${name} (${current})`;
        mapSelect.appendChild(opt);
      }
      const target = selectPath || current;
      if (target) mapSelect.value = target;
    } catch (e) {
      setStatus("Map list failed: " + e.message);
    }
  }

  async function loadSelectedMap() {
    const path = mapSelect.value;
    if (!path) return;
    setStatus("Loading map...");
    try {
      await fetchJson("/api/maps/load", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ path }),
      });
      setStatus("Loaded: " + path);
      await loadFromDevice();
      refreshMapList(path);
    } catch (e) {
      setStatus("Load failed: " + e.message);
    }
  }

  async function loadFromDevice() {
    try {
      const data = await fetchJson("/api/map");
      state.speed = data.speedBins || state.speed;
      state.throttle = data.throttleBins || state.throttle;
      state.lock = data.lockTable || state.lock;
      buildBins();
      buildTable();
      setStatus("Loaded current map");
    } catch (e) {
      setStatus("Load failed: " + e.message);
    }
  }

  async function saveCurrent() {
    setStatus("Saving current map...");
    try {
      await fetchJson("/api/map", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          speedBins: state.speed,
          throttleBins: state.throttle,
          lockTable: state.lock,
        }),
      });
      setStatus("Saved current map");
    } catch (e) {
      setStatus("Save failed: " + e.message);
    }
  }

  async function saveMapAs() {
    const name = document.getElementById("mapName").value.trim();
    if (!name) {
      setStatus("Enter a map name");
      return;
    }
    try {
      await fetchJson("/api/map", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          speedBins: state.speed,
          throttleBins: state.throttle,
          lockTable: state.lock,
        }),
      });
      const res = await fetchJson("/api/maps/save", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ name }),
      });
      setStatus("Saved: " + (res.path || name));
      refreshMapList(res.path || "");
    } catch (e) {
      setStatus("Save failed: " + e.message);
    }
  }

  async function deleteSelectedMap() {
    const path = mapSelect.value;
    if (!path) return;
    if (!confirm("Delete map: " + path + "?")) return;
    try {
      await fetchJson("/api/maps/delete", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ path }),
      });
      setStatus("Deleted: " + path);
      refreshMapList("");
    } catch (e) {
      setStatus("Delete failed: " + e.message);
    }
  }

  function exportTxt() {
    const header = ["T", "Throttle", ...state.speed.map((s) => "S" + s)].join("\t");
    const rows = state.throttle.map((t, i) => ["T" + t, t, ...state.lock[i]].join("\t"));
    const text = [header, ...rows].join("\n");
    const blob = new Blob([text], { type: "text/plain" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = "openhaldex-map.txt";
    document.body.appendChild(a);
    a.click();
    a.remove();
    URL.revokeObjectURL(url);
  }

  function fromTxt(text) {
    const lines = text.split(/\r?\n/).filter((l) => l.trim().length > 0);
    if (lines.length < 2) throw new Error("Invalid map file");
    const header = lines[0].split("\t");
    if (header.length < 2 + defaultSpeed.length) throw new Error("Invalid map header");
    state.speed = header.slice(2).map((v) => parseInt(v.replace(/[ST]/g, ""), 10));
    const throttles = [];
    const lock = [];
    lines.slice(1).forEach((line) => {
      const parts = line.split("\t");
      const t = parseInt(parts[0].replace(/[ST]/g, ""), 10) || parseInt(parts[1], 10);
      throttles.push(t);
      lock.push(parts.slice(2).map((v) => parseInt(v, 10)));
    });
    state.throttle = throttles;
    state.lock = lock;
    buildBins();
    buildTable();
    setStatus("Loaded TXT map");
  }

  document.getElementById("btnRefresh").onclick = () => refreshMapList(mapSelect.value);
  document.getElementById("btnLoadMap").onclick = loadSelectedMap;
  document.getElementById("btnLoad").onclick = loadFromDevice;
  document.getElementById("btnSave").onclick = saveCurrent;
  document.getElementById("btnSaveMap").onclick = saveMapAs;
  document.getElementById("btnDeleteMap").onclick = deleteSelectedMap;
  document.getElementById("btnDownload").onclick = exportTxt;
  document.getElementById("fileInput").onchange = async (e) => {
    const file = e.target.files[0];
    if (!file) return;
    try {
      const txt = await file.text();
      fromTxt(txt);
      await saveCurrent();
      await refreshMapList("");
      setStatus("Loaded TXT map and saved current map");
    } catch (err) {
      setStatus(err.message || "TXT load failed");
    }
  };

  refreshMapList("");
  loadFromDevice();
  setInterval(refreshTrace, 250);
}

// CAN view controller:
// - decoded/raw polling with bus + token filters
// - optional safe capture mode toggle
// - one-shot 30s text dump download
function initCanviewPage() {
  const nameAliases = {
    Motordrehzahl: "Engine speed",
    Lastsignal: "Engine load",
    Fahrpedalwert_oder_Drosselklapp: "Throttle position",
    Ladedruck: "Boost pressure",
    Kuehlmitteltemp__4_1__Kombi_2_: "Coolant temp",
    Oeltemperatur_4_1: "Oil temp",
    Aussentemperatur_gefiltert: "Outside temp (filtered)",
    Aussentemp__ungefiltert_4_1__Ko: "Outside temp (raw)",
    Kupplungssteifigkeit_Hinten__Is: "Rear clutch stiffness",
    Kupplungssteifigkeit_Mitte__Ist: "Center clutch stiffness",
    Kupplung_komplett_offen: "Clutch fully open",
    Geschwindigkeitsbegrenzung: "Speed limit active",
    Allrad_Warnlampe: "AWD warning lamp",
    Notlauf: "Limp mode",
    Ubertemperaturschutz__Allrad_1_: "AWD overtemp protection",
    Fehler_Allrad_Kupplung: "AWD clutch fault",
    Fehlerstatus_Kupplungssteifigke: "Clutch stiffness fault",
    Fehlerspeichereintrag_Allrad_1: "AWD fault stored",
  };

  function formatName(name) {
    if (!name) return "";
    const alias = nameAliases[name];
    return alias || name.replace(/_/g, " ");
  }

  let timer = null;
  let captureActive = false;
  let lastData = { decoded: [], raw: [] };

  const statusEl = document.getElementById("status");
  const captureStatusEl = document.getElementById("captureStatus");
  const decodedBody = document.querySelector("#decodedTable tbody");
  const rawBody = document.querySelector("#rawTable tbody");
  const busFilterEl = document.getElementById("busFilter");
  const filterEl = document.getElementById("filter");
  const presetEl = document.getElementById("diagPreset");
  const captureBtn = document.getElementById("btnCapture");

  function setStatus(msg) {
    statusEl.textContent = msg || "";
  }

  function setCaptureStatus(msg) {
    if (captureStatusEl) captureStatusEl.textContent = msg || "";
  }

  function updateCaptureUi() {
    if (captureBtn) {
      captureBtn.textContent = captureActive ? "Diagnostic Capture: On" : "Diagnostic Capture: Off";
    }
    setCaptureStatus(
      captureActive ? "Capture mode active: Controller OFF + Broadcast ON (settings locked)" : ""
    );
  }

  function busMatches(item, busFilter) {
    if (!busFilter || busFilter === "all") return true;
    const bus = String(item.bus || "").toLowerCase();
    if (!bus) return false;
    if (bus === busFilter) return true;
    if (busFilter === "chassis" && (bus.includes("chassis") || bus.includes("chs"))) return true;
    if (busFilter === "haldex" && (bus.includes("haldex") || bus.includes("hdx"))) return true;
    return false;
  }

  function normalizeToken(token) {
    const t = String(token || "")
      .trim()
      .toLowerCase();
    if (!t) return "";
    return t.startsWith("0x") ? t.slice(2) : t;
  }

  function splitTokens(text) {
    const input = String(text || "").trim();
    if (!input) return [];
    return input
      .split(/\s+/)
      .map(normalizeToken)
      .filter((t) => t.length > 0);
  }

  function mergeTokens(existing, extra) {
    const seen = new Set();
    const out = [];
    [...splitTokens(existing), ...splitTokens(extra)].forEach((t) => {
      if (!seen.has(t)) {
        seen.add(t);
        out.push(t);
      }
    });
    return out.join(" ");
  }

  function addSelectedPreset() {
    if (!filterEl || !presetEl || !presetEl.value) return;
    filterEl.value = mergeTokens(filterEl.value, presetEl.value);
    presetEl.value = "";
    redraw();
  }

  function decodedMatches(item, tokens) {
    if (!tokens || !tokens.length) return true;
    const base = String(item.name || "");
    const pretty = formatName(base);
    const idNum = Number(item.id);
    const idDec = String(item.id || "").toLowerCase();
    const idHex = Number.isFinite(idNum) ? idNum.toString(16).toLowerCase() : "";
    const fields = [
      idDec,
      idHex,
      base.toLowerCase(),
      pretty.toLowerCase(),
      String(item.value || "").toLowerCase(),
    ];
    return tokens.some((t) => fields.some((field) => field.includes(t)));
  }

  function rawMatches(item, tokens) {
    if (!tokens || !tokens.length) return true;
    const idNum = Number(item.id);
    const idDec = String(item.id || "").toLowerCase();
    const idHex = Number.isFinite(idNum) ? idNum.toString(16).toLowerCase() : "";
    const fields = [idDec, idHex, String(item.data || "").toLowerCase()];
    return tokens.some((t) => fields.some((field) => field.includes(t)));
  }

  function redraw() {
    const filterTokens = splitTokens(filterEl ? filterEl.value : "");
    const busFilter = busFilterEl ? busFilterEl.value : "all";
    renderDecoded(decodedBody, lastData.decoded || [], filterTokens, busFilter);
    renderRaw(rawBody, lastData.raw || [], busFilter, filterTokens);
  }

  async function poll() {
    const decodedLimit = parseInt(document.getElementById("decodedLimit").value || "48", 10);
    const rawLimit = parseInt(document.getElementById("rawLimit").value || "20", 10);
    const busFilter = busFilterEl ? busFilterEl.value : "all";
    try {
      const data = await fetchJson(
        "/api/canview?decoded=" +
          decodedLimit +
          "&raw=" +
          rawLimit +
          "&bus=" +
          encodeURIComponent(busFilter)
      );
      lastData.decoded = data.decoded || [];
      lastData.raw = data.raw || [];
      redraw();
      setStatus("Updated " + new Date().toLocaleTimeString());
    } catch (e) {
      setStatus("Fetch failed: " + e.message);
      redraw();
    }
  }

  async function downloadDump() {
    const busFilter = busFilterEl ? busFilterEl.value : "all";
    try {
      setStatus("Building 30s dump...");
      const res = await fetch("/api/canview/dump?seconds=30&bus=" + encodeURIComponent(busFilter));
      if (!res.ok) {
        throw new Error("HTTP " + res.status);
      }
      const blob = await res.blob();
      const url = URL.createObjectURL(blob);
      const a = document.createElement("a");
      const stamp = new Date().toISOString().replace(/[.:]/g, "-");
      a.href = url;
      a.download = `openhaldex-can-dump-${busFilter}-${stamp}.txt`;
      document.body.appendChild(a);
      a.click();
      a.remove();
      URL.revokeObjectURL(url);
      setStatus("Dump downloaded");
    } catch (e) {
      setStatus("Dump failed: " + e.message);
    }
  }

  async function refreshCaptureState() {
    try {
      const data = await fetchJson("/api/canview/capture");
      captureActive = !!data.active;
      updateCaptureUi();
    } catch (e) {
      setCaptureStatus("Capture status failed: " + e.message);
    }
  }

  async function toggleCaptureMode() {
    try {
      setCaptureStatus("Switching capture mode...");
      const data = await fetchJson("/api/canview/capture", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ active: !captureActive }),
      });
      captureActive = !!data.active;
      updateCaptureUi();
      setStatus(captureActive ? "Capture mode enabled" : "Capture mode disabled");
      await poll();
    } catch (e) {
      setCaptureStatus("Capture switch failed: " + e.message);
    }
  }

  function renderDecoded(body, list, filterTokens, busFilter) {
    body.innerHTML = "";
    list
      .filter((item) => busMatches(item, busFilter) && decodedMatches(item, filterTokens))
      .forEach((item) => {
        const tr = document.createElement("tr");
        const baseName = item.name || "";
        const dir = String(item.dir || "").toLowerCase();
        if (dir === "rx" || dir === "tx") tr.classList.add("dir-" + dir);
        if (item.generated) tr.classList.add("generated");
        tr.innerHTML = `<td>${item.bus || ""}</td><td>${item.dir || ""}</td><td>${item.id}</td><td title="${baseName}">${formatName(baseName)}</td><td>${item.value}</td><td>${item.unit || ""}</td><td>${ageMs(item.ts)}</td>`;
        body.appendChild(tr);
      });
  }

  function renderRaw(body, list, busFilter, filterTokens) {
    body.innerHTML = "";
    list
      .filter((item) => busMatches(item, busFilter) && rawMatches(item, filterTokens))
      .forEach((item) => {
        const tr = document.createElement("tr");
        const dir = String(item.dir || "").toLowerCase();
        if (dir === "rx" || dir === "tx") tr.classList.add("dir-" + dir);
        if (item.generated) tr.classList.add("generated");
        tr.innerHTML = `<td>${item.bus || ""}</td><td>${item.dir || ""}</td><td>${item.id}</td><td>${item.dlc}</td><td>${item.data}</td><td>${ageMs(item.ts)}</td>`;
        body.appendChild(tr);
      });
  }

  function ageMs(ts) {
    if (!ts) return "";
    const now = Date.now();
    return Math.max(0, now - ts) + " ms";
  }

  document.getElementById("btnStart").onclick = () => {
    const interval = parseInt(document.getElementById("interval").value || "500", 10);
    if (timer) clearInterval(timer);
    timer = setInterval(poll, interval);
    poll();
  };

  if (busFilterEl) {
    busFilterEl.onchange = () => {
      redraw();
      poll();
    };
  }
  if (filterEl) filterEl.oninput = redraw;
  if (presetEl) presetEl.onchange = addSelectedPreset;

  document.getElementById("btnStop").onclick = () => {
    if (timer) clearInterval(timer);
    timer = null;
  };

  const dumpBtn = document.getElementById("btnDump30");
  if (dumpBtn) dumpBtn.onclick = downloadDump;
  if (captureBtn) captureBtn.onclick = toggleCaptureMode;

  refreshCaptureState();
}
// Diagnostics page:
// combines status, telemetry, frame diagnostics, and network state.
function initDiagPage() {
  const el = (id) => document.getElementById(id);
  const statusEl = el("status");

  function pill(value) {
    if (value === true || value === 1) return '<span class="pill good">Yes</span>';
    if (value === false || value === 0) return '<span class="pill bad">No</span>';
    return `<span class="muted">${value ?? "-"}</span>`;
  }

  function fmt(val, suffix = "") {
    if (val === null || val === undefined) return "-";
    if (typeof val === "number") return `${val}${suffix}`;
    return `${val}${suffix}`;
  }

  function linkOrDash(value) {
    const v = String(value || "").trim();
    if (!v) return "-";
    return `<a href="http://${v}">${v}</a>`;
  }
  function haldexStateLabel(value) {
    if (value === null || value === undefined) return "-";
    const v = Number(value) || 0;
    const labels = [];
    if (v & (1 << 0)) labels.push("Clutch 1 report");
    if (v & (1 << 1)) labels.push("Temp protection");
    if (v & (1 << 2)) labels.push("Clutch 2 report");
    if (v & (1 << 3)) labels.push("Coupling open");
    if (v & (1 << 6)) labels.push("Speed limit");
    if (!labels.length) labels.push("None");
    return `${v} (${labels.join(", ")})`;
  }

  function fmtMs(val) {
    if (val === null || val === undefined) return "-";
    return `${val} ms`;
  }

  function fmtFrame(frame) {
    if (!frame || !frame.ok) return "-";
    const kind = frame.generated ? "GEN" : "BRIDGE";
    const age = frame.ageMs === undefined ? "-" : `${frame.ageMs} ms`;
    return `${frame.data || ""} (${kind}, ${age})`;
  }

  function setText(id, value) {
    const node = el(id);
    if (node) node.textContent = value;
  }
  function fmtUptime(ms) {
    if (!ms && ms !== 0) return "-";
    const sec = Math.floor(ms / 1000);
    const h = Math.floor(sec / 3600);
    const m = Math.floor((sec % 3600) / 60);
    const s = sec % 60;
    return `${h}h ${m}m ${s}s`;
  }

  async function poll() {
    try {
      const data = await fetchJson("/api/status");

      el("mode").textContent = data.mode || "-";
      el("controllerEnabled").innerHTML = pill(!data.disableController);
      el("broadcast").innerHTML = pill(data.broadcastOpenHaldexOverCAN);
      el("haldexGen").textContent = data.haldexGeneration ?? "-";

      const tel = data.telemetry || {};
      el("spec").textContent = fmt(tel.spec);
      el("act").textContent = fmt(tel.act);
      el("haldexState").textContent = haldexStateLabel(tel.haldexState);
      el("haldexEngagement").textContent = fmt(tel.haldexEngagement);

      el("speed").textContent = fmt(tel.speed, " km/h");
      el("rpm").textContent = fmt(tel.rpm, " rpm");
      el("throttle").textContent = fmt(tel.throttle, " %");
      el("boost").textContent = fmt(tel.boost, " kPa");
      el("clutch1").innerHTML = pill(tel.clutch1Report);
      el("clutch2").innerHTML = pill(tel.clutch2Report);

      const can = data.can || {};
      el("canReady").innerHTML = pill(can.ready);
      el("canChassis").innerHTML = pill(can.chassis);
      el("canHaldex").innerHTML = pill(can.haldex);
      el("canFailure").innerHTML = can.busFailure
        ? '<span class="pill bad">Failure</span>'
        : '<span class="pill good">OK</span>';
      el("lastChassis").textContent = fmtMs(can.lastChassisMs);
      el("lastHaldex").textContent = fmtMs(can.lastHaldexMs);

      el("tempProtection").innerHTML = pill(tel.tempProtection);
      el("couplingOpen").innerHTML = pill(tel.couplingOpen);

      const frameDiag = data.frameDiag || {};
      setText("frameTarget", fmt(frameDiag.lockTarget));
      setText("frameM1", fmtFrame(frameDiag.motor1));
      setText("frameM3", fmtFrame(frameDiag.motor3));
      setText("frameB1", fmtFrame(frameDiag.brakes1));
      setText("frameB2", fmtFrame(frameDiag.brakes2));
      setText("frameB3", fmtFrame(frameDiag.brakes3));

      el("uptime").textContent = fmtUptime(data.uptimeMs);

      try {
        const net = await fetchJson("/api/network");
        const mode = net.staConnected ? "STA+AP" : net.ap ? "AP" : "STA";
        el("netMode").textContent = mode;
        el("netStaIp").innerHTML = linkOrDash(net.staIp);
        el("netApIp").innerHTML = linkOrDash(net.apIp);
        el("netHost").innerHTML = linkOrDash(net.hostname);
        el("netInternet").innerHTML = pill(!!net.staConnected && !!net.internet);
      } catch (__) {
        el("netMode").textContent = "-";
        el("netStaIp").textContent = "-";
        el("netApIp").textContent = "-";
        el("netHost").textContent = "-";
        el("netInternet").textContent = "-";
      }

      statusEl.textContent = "Last update: " + new Date().toLocaleTimeString();
    } catch (e) {
      statusEl.textContent = "Status fetch failed: " + e.message;
    }
  }

  poll();
  setInterval(poll, 1000);
}

// OTA page:
// - local file upload flow
// - hotspot credential management
// - online update check/install polling UI
function initOtaPage() {
  const form = document.getElementById("otaForm");
  const statusEl = document.getElementById("status");
  const wifiStatus = document.getElementById("wifiStatus");
  const updateStatus = document.getElementById("updateStatus");
  const wifiStaEnable = document.getElementById("wifiStaEnable");
  const wifiSsid = document.getElementById("wifiSsid");
  const wifiPass = document.getElementById("wifiPass");
  const btnWifiSave = document.getElementById("btnWifiSave");
  const btnWifiClear = document.getElementById("btnWifiClear");

  form.addEventListener("submit", async (e) => {
    e.preventDefault();
    const file = document.getElementById("bin").files[0];
    if (!file) return;
    statusEl.textContent = "Uploading...";

    const data = new FormData();
    data.append("update", file, file.name);

    try {
      const res = await fetch("/ota/update", { method: "POST", body: data });
      const text = await res.text();
      statusEl.textContent = res.ok ? "Update OK, rebooting..." : "Update failed: " + text;
    } catch (err) {
      statusEl.textContent = "Upload error: " + err;
    }
  });
  async function loadWifi() {
    if (!wifiSsid || !wifiStaEnable) return;
    try {
      const data = await fetchJson("/api/wifi");
      wifiSsid.value = data.ssid || "";
      if (wifiPass) wifiPass.value = "";
      wifiStaEnable.checked = !!data.staEnabled;
      if (wifiStatus) {
        wifiStatus.textContent = data.ssid ? `Saved SSID: ${data.ssid}` : "No saved hotspot";
      }
    } catch (e) {
      if (wifiStatus) wifiStatus.textContent = "Wi-Fi fetch failed: " + e.message;
    }
  }

  async function saveWifi() {
    if (!wifiSsid || !wifiStaEnable) return;
    const payload = {
      ssid: wifiSsid.value || "",
      password: wifiPass ? wifiPass.value || "" : "",
      staEnabled: !!wifiStaEnable.checked,
    };
    try {
      await fetchJson("/api/wifi", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      });
      if (wifiPass) wifiPass.value = "";
      if (wifiStatus) {
        wifiStatus.textContent = payload.ssid ? `Saved SSID: ${payload.ssid}` : "Wi-Fi cleared";
      }
    } catch (e) {
      if (wifiStatus) wifiStatus.textContent = "Wi-Fi save failed: " + e.message;
    }
  }

  async function saveStaOnly() {
    if (!wifiStaEnable) return;
    try {
      await fetchJson("/api/wifi", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ staEnabled: !!wifiStaEnable.checked }),
      });
      if (wifiStatus) {
        wifiStatus.textContent = wifiStaEnable.checked ? "STA enabled" : "STA disabled";
      }
    } catch (e) {
      if (wifiStatus) wifiStatus.textContent = "STA update failed: " + e.message;
    }
  }

  async function clearWifi() {
    if (!wifiSsid || !wifiStaEnable) return;
    wifiSsid.value = "";
    if (wifiPass) wifiPass.value = "";
    wifiStaEnable.checked = false;
    try {
      await fetchJson("/api/wifi", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ ssid: "", password: "", staEnabled: false }),
      });
      if (wifiStatus) wifiStatus.textContent = "Wi-Fi cleared";
    } catch (e) {
      if (wifiStatus) wifiStatus.textContent = "Wi-Fi clear failed: " + e.message;
    }
  }

  if (btnWifiSave) btnWifiSave.onclick = saveWifi;
  if (btnWifiClear) btnWifiClear.onclick = clearWifi;
  if (wifiStaEnable) wifiStaEnable.onchange = saveStaOnly;

  function formatBytes(bytes) {
    const b = Number(bytes) || 0;
    if (b >= 1024 * 1024) return (b / (1024 * 1024)).toFixed(2) + " MB";
    if (b >= 1024) return (b / 1024).toFixed(1) + " KB";
    return b + " B";
  }

  function formatSpeed(bps) {
    const b = Number(bps) || 0;
    if (b <= 0) return "";
    return formatBytes(b) + "/s";
  }

  function buildProgressText(data) {
    const stage = data.stage || "update";
    const done = Number(data.bytesDone) || 0;
    const total = Number(data.bytesTotal) || 0;
    const speed = Number(data.speedBps) || 0;
    let text = "Installing " + stage;
    if (total > 0) {
      const pct = Math.min(100, (done / total) * 100).toFixed(1);
      text += ": " + pct + "% (" + formatBytes(done) + " / " + formatBytes(total) + ")";
    } else if (done > 0) {
      text += ": " + formatBytes(done);
    }
    const speedText = formatSpeed(speed);
    if (speedText) text += " | " + speedText;
    return text;
  }
  async function installUpdate() {
    if (!updateStatus) return;
    updateStatus.textContent = "Starting update...";
    try {
      const res = await fetch("/api/update/install", { method: "POST" });
      if (!res.ok) {
        const text = await res.text();
        updateStatus.textContent = "Update start failed: " + text;
        return;
      }
      updateStatus.textContent = "Installing update...";
    } catch (e) {
      updateStatus.textContent = "Update start failed: " + e.message;
    }
  }

  async function refreshUpdate() {
    if (!updateStatus) return;
    try {
      const data = await fetchJson("/api/update");
      const current = data.current || "-";
      const latest = data.latest || "";

      if (data.installing) {
        updateStatus.textContent = buildProgressText(data) || "Installing update...";
        return;
      }

      if (data.installError) {
        updateStatus.textContent = "Update failed: " + data.installError;
        return;
      }

      if (data.available && latest) {
        updateStatus.innerHTML =
          'Update available: <button id="btnInstallUpdate">Install ' +
          latest +
          "</button> (current " +
          current +
          ")";
      } else if (latest) {
        updateStatus.textContent = "Up to date: " + current;
      } else if (data.error) {
        updateStatus.textContent = "Update: " + current + " (" + data.error + ")";
      } else {
        updateStatus.textContent = "Update: " + current;
      }

      const btn = document.getElementById("btnInstallUpdate");
      if (btn) btn.onclick = installUpdate;
    } catch (e) {
      updateStatus.textContent = "Update check failed: " + e.message;
    }
  }
  loadWifi();
  refreshUpdate();
  setInterval(refreshUpdate, 2000);
}
