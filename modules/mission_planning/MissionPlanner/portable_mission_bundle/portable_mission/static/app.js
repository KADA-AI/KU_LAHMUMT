const state = {
  dems: [],
  selectedDem: "",
  preview: null,
  roi: null,
  session: null,
  mode: "start",
  startCell: null,
  goalCells: [],
  result: null,
  plannerLayout: null,
  previewDrag: null,
  modelInfo: null,
};

const els = {};

document.addEventListener("DOMContentLoaded", async () => {
  bindElements();
  bindEvents();
  await loadModelInfo();
  await refreshDems();
  window.addEventListener("resize", () => {
    renderPreview();
    renderPlanner();
  });
});

function bindElements() {
  els.modelFile = document.getElementById("modelFile");
  els.modelDefaults = document.getElementById("modelDefaults");
  els.demSelect = document.getElementById("demSelect");
  els.loadDemBtn = document.getElementById("loadDemBtn");
  els.refreshDemsBtn = document.getElementById("refreshDemsBtn");
  els.demUploadInput = document.getElementById("demUploadInput");
  els.demMeta = document.getElementById("demMeta");
  els.previewCanvas = document.getElementById("previewCanvas");
  els.clearRoiBtn = document.getElementById("clearRoiBtn");
  els.roiLabel = document.getElementById("roiLabel");
  els.altitudeInput = document.getElementById("altitudeInput");
  els.hexStepInput = document.getElementById("hexStepInput");
  els.maxStepsInput = document.getElementById("maxStepsInput");
  els.maxGoalsInput = document.getElementById("maxGoalsInput");
  els.buildSessionBtn = document.getElementById("buildSessionBtn");
  els.sessionMeta = document.getElementById("sessionMeta");
  els.gridMeta = document.getElementById("gridMeta");
  els.statusText = document.getElementById("statusText");
  els.modeStartBtn = document.getElementById("modeStartBtn");
  els.modeGoalBtn = document.getElementById("modeGoalBtn");
  els.undoGoalBtn = document.getElementById("undoGoalBtn");
  els.clearMissionBtn = document.getElementById("clearMissionBtn");
  els.modeLabel = document.getElementById("modeLabel");
  els.startLabel = document.getElementById("startLabel");
  els.goalLabel = document.getElementById("goalLabel");
  els.plannerCanvas = document.getElementById("plannerCanvas");
  els.deterministicInput = document.getElementById("deterministicInput");
  els.simulateBtn = document.getElementById("simulateBtn");
  els.rewardMetric = document.getElementById("rewardMetric");
  els.stepsMetric = document.getElementById("stepsMetric");
  els.goalsMetric = document.getElementById("goalsMetric");
  els.terminationMetric = document.getElementById("terminationMetric");
  els.pathMeta = document.getElementById("pathMeta");
  els.jsonOutput = document.getElementById("jsonOutput");
  els.downloadJsonBtn = document.getElementById("downloadJsonBtn");
}

function bindEvents() {
  els.refreshDemsBtn.addEventListener("click", () => refreshDems(state.selectedDem));
  els.loadDemBtn.addEventListener("click", () => {
    if (els.demSelect.value) {
      loadPreview(els.demSelect.value);
    }
  });
  els.demUploadInput.addEventListener("change", uploadDem);
  els.clearRoiBtn.addEventListener("click", () => {
    state.roi = null;
    renderPreview();
    syncLabels();
  });
  els.buildSessionBtn.addEventListener("click", buildSession);
  els.modeStartBtn.addEventListener("click", () => setMode("start"));
  els.modeGoalBtn.addEventListener("click", () => setMode("goal"));
  els.undoGoalBtn.addEventListener("click", undoGoal);
  els.clearMissionBtn.addEventListener("click", clearMission);
  els.simulateBtn.addEventListener("click", runMission);
  els.downloadJsonBtn.addEventListener("click", downloadJson);

  els.previewCanvas.addEventListener("mousedown", handlePreviewPointerDown);
  window.addEventListener("mousemove", handlePreviewPointerMove);
  window.addEventListener("mouseup", handlePreviewPointerUp);

  els.plannerCanvas.addEventListener("click", handlePlannerClick);
}

async function loadModelInfo() {
  try {
    const payload = await fetchJson("/api/model");
    state.modelInfo = payload;
    els.modelFile.textContent = payload.model_file || "-";
    const defaults = payload.defaults || {};
    els.modelDefaults.textContent = [
      `alt ${defaults.altitude_m ?? "-"}`,
      `hex ${defaults.hex_step ?? "-"}`,
      `max_steps ${defaults.max_steps ?? "-"}`,
      `max_goals ${defaults.max_goals ?? "-"}`,
    ].join(" | ");
    applyDefaultInputs(defaults);
  } catch (error) {
    setStatus(error.message, true);
  }
}

function applyDefaultInputs(defaults) {
  els.altitudeInput.value = defaults.altitude_m ?? 700;
  els.hexStepInput.value = defaults.hex_step ?? 25;
  els.maxStepsInput.value = defaults.max_steps ?? 3000;
  els.maxGoalsInput.value = defaults.max_goals ?? 20;
}

async function refreshDems(preferredName = "") {
  try {
    const payload = await fetchJson("/api/dems");
    state.dems = Array.isArray(payload.items) ? payload.items : [];
    renderDemSelect(preferredName);
    if (state.dems.length) {
      const selected = preferredName && state.dems.some((item) => item.name === preferredName)
        ? preferredName
        : state.dems[0].name;
      els.demSelect.value = selected;
      state.selectedDem = selected;
      await loadPreview(selected);
    } else {
      state.selectedDem = "";
      state.preview = null;
      state.roi = null;
      state.session = null;
      clearMission();
      renderPreview();
      renderPlanner();
      els.demMeta.textContent = "No GeoTIFF found in data/inputs.";
      setStatus("Add a .tif or .tiff file to data/inputs or upload one.", false);
    }
  } catch (error) {
    setStatus(error.message, true);
  }
}

function renderDemSelect(preferredName) {
  els.demSelect.innerHTML = "";
  if (!state.dems.length) {
    const option = document.createElement("option");
    option.value = "";
    option.textContent = "No DEMs available";
    els.demSelect.appendChild(option);
    return;
  }
  for (const item of state.dems) {
    const option = document.createElement("option");
    option.value = item.name;
    option.textContent = `${item.name} (${item.width}x${item.height})`;
    if (item.name === preferredName) {
      option.selected = true;
    }
    els.demSelect.appendChild(option);
  }
}

async function loadPreview(name) {
  if (!name) {
    return;
  }
  try {
    setStatus("Loading DEM preview...", false);
    const payload = await fetchJson(`/api/dem-preview?name=${encodeURIComponent(name)}`);
    state.selectedDem = payload.name;
    state.preview = payload;
    state.roi = null;
    state.session = null;
    clearMission(false);
    els.demMeta.textContent = `${payload.dem.width}x${payload.dem.height} | ${payload.dem.crs || "CRS unknown"} | ${payload.dem.is_geographic ? "geographic" : "projected"}`;
    renderPreview();
    renderPlanner();
    syncLabels();
    setStatus("DEM preview loaded. Drag an ROI.", false);
  } catch (error) {
    setStatus(error.message, true);
  }
}

async function uploadDem(event) {
  const file = event.target.files?.[0];
  if (!file) {
    return;
  }

  const formData = new FormData();
  formData.append("file", file);
  try {
    setStatus("Uploading GeoTIFF...", false);
    const response = await fetch("/api/dems/upload", {
      method: "POST",
      body: formData,
    });
    const payload = await parseJson(response);
    if (!response.ok) {
      throw new Error(payload.error || `Upload failed with ${response.status}`);
    }
    await refreshDems(payload.name);
    setStatus(`Uploaded ${payload.name}.`, false);
  } catch (error) {
    setStatus(error.message, true);
  } finally {
    event.target.value = "";
  }
}

function renderPreview() {
  const canvas = els.previewCanvas;
  const ctx = canvas.getContext("2d");
  const parentRect = canvas.parentElement.getBoundingClientRect();
  const width = Math.max(320, Math.floor(parentRect.width));
  const height = Math.max(320, Math.floor(parentRect.width * 0.72));
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.floor(width * dpr);
  canvas.height = Math.floor(height * dpr);
  canvas.style.height = `${height}px`;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, width, height);

  if (!state.preview?.preview) {
    drawEmptyState(ctx, width, height, "No DEM preview");
    return;
  }

  const image = buildPreviewImageData(ctx, state.preview.preview);
  const fit = containRect(
    state.preview.preview.width,
    state.preview.preview.height,
    width,
    height,
    16,
  );
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(image, fit.x, fit.y, fit.width, fit.height);

  ctx.strokeStyle = "rgba(31,36,48,0.18)";
  ctx.strokeRect(fit.x, fit.y, fit.width, fit.height);

  if (state.roi) {
    drawNormalizedRect(ctx, fit, state.roi, "#0d5c63");
  }
  if (state.previewDrag) {
    drawNormalizedRect(ctx, fit, state.previewDrag, "#d97706");
  }
}

function buildPreviewImageData(ctx, preview) {
  const offscreen = document.createElement("canvas");
  offscreen.width = preview.width;
  offscreen.height = preview.height;
  const offCtx = offscreen.getContext("2d");
  const imageData = offCtx.createImageData(preview.width, preview.height);
  let index = 0;
  for (const row of preview.pixels) {
    for (const value of row) {
      imageData.data[index] = value;
      imageData.data[index + 1] = value;
      imageData.data[index + 2] = value;
      imageData.data[index + 3] = 255;
      index += 4;
    }
  }
  offCtx.putImageData(imageData, 0, 0);
  return offscreen;
}

function containRect(sourceWidth, sourceHeight, targetWidth, targetHeight, padding) {
  const scale = Math.min(
    (targetWidth - padding * 2) / sourceWidth,
    (targetHeight - padding * 2) / sourceHeight,
  );
  const width = sourceWidth * scale;
  const height = sourceHeight * scale;
  return {
    x: (targetWidth - width) / 2,
    y: (targetHeight - height) / 2,
    width,
    height,
  };
}

function drawNormalizedRect(ctx, fit, roi, stroke) {
  const x = fit.x + roi.x0 * fit.width;
  const y = fit.y + roi.y0 * fit.height;
  const width = (roi.x1 - roi.x0) * fit.width;
  const height = (roi.y1 - roi.y0) * fit.height;
  ctx.save();
  ctx.fillStyle = `${stroke}22`;
  ctx.strokeStyle = stroke;
  ctx.lineWidth = 2;
  ctx.fillRect(x, y, width, height);
  ctx.strokeRect(x, y, width, height);
  ctx.restore();
}

function drawEmptyState(ctx, width, height, message) {
  ctx.fillStyle = "#ede6d8";
  ctx.fillRect(0, 0, width, height);
  ctx.fillStyle = "#69707c";
  ctx.font = "16px Segoe UI";
  ctx.textAlign = "center";
  ctx.fillText(message, width / 2, height / 2);
}

function handlePreviewPointerDown(event) {
  if (!state.preview?.preview) {
    return;
  }
  const norm = previewPointerToNormalized(event);
  if (!norm) {
    return;
  }
  state.previewDrag = { x0: norm.x, y0: norm.y, x1: norm.x, y1: norm.y };
  renderPreview();
}

function handlePreviewPointerMove(event) {
  if (!state.previewDrag) {
    return;
  }
  const norm = previewPointerToNormalized(event);
  if (!norm) {
    return;
  }
  state.previewDrag.x1 = norm.x;
  state.previewDrag.y1 = norm.y;
  renderPreview();
}

function handlePreviewPointerUp() {
  if (!state.previewDrag) {
    return;
  }
  const roi = normalizeRect(state.previewDrag);
  state.previewDrag = null;
  if ((roi.x1 - roi.x0) >= 0.01 && (roi.y1 - roi.y0) >= 0.01) {
    state.roi = roi;
    setStatus("ROI selected. Build the hex grid next.", false);
  }
  renderPreview();
  syncLabels();
}

function previewPointerToNormalized(event) {
  const canvas = els.previewCanvas;
  const rect = canvas.getBoundingClientRect();
  if (!rect.width || !rect.height || !state.preview?.preview) {
    return null;
  }
  const fit = containRect(
    state.preview.preview.width,
    state.preview.preview.height,
    rect.width,
    rect.height,
    16,
  );
  const x = event.clientX - rect.left;
  const y = event.clientY - rect.top;
  if (x < fit.x || x > fit.x + fit.width || y < fit.y || y > fit.y + fit.height) {
    return null;
  }
  return {
    x: clamp((x - fit.x) / fit.width, 0, 1),
    y: clamp((y - fit.y) / fit.height, 0, 1),
  };
}

function normalizeRect(rect) {
  return {
    x0: Math.min(rect.x0, rect.x1),
    y0: Math.min(rect.y0, rect.y1),
    x1: Math.max(rect.x0, rect.x1),
    y1: Math.max(rect.y0, rect.y1),
  };
}

async function buildSession() {
  if (!state.selectedDem) {
    setStatus("Select a DEM first.", true);
    return;
  }
  if (!state.roi) {
    setStatus("Draw an ROI first.", true);
    return;
  }
  try {
    setStatus("Building hex session...", false);
    const payload = await postJson("/api/sessions", {
      dem_name: state.selectedDem,
      roi: state.roi,
      altitude_m: numberValue(els.altitudeInput.value),
      hex_step: intValue(els.hexStepInput.value),
      max_steps: intValue(els.maxStepsInput.value),
      max_goals: intValue(els.maxGoalsInput.value),
    });
    state.session = payload;
    clearMission(false);
    renderPlanner();
    syncLabels();
    els.sessionMeta.textContent = `${payload.coordinate_mode} | ${payload.crop.shape.width}x${payload.crop.shape.height} px`;
    els.gridMeta.textContent = `${payload.grid.rows} rows x ${payload.grid.cols} cols | safe ${payload.grid.safe_count} | blocked ${payload.grid.blocked_count}`;
    setStatus("Hex grid ready. Pick a start cell and one or more goals.", false);
  } catch (error) {
    setStatus(error.message, true);
  }
}

function setMode(mode) {
  state.mode = mode;
  els.modeStartBtn.classList.toggle("active", mode === "start");
  els.modeGoalBtn.classList.toggle("active", mode === "goal");
  syncLabels();
}

function undoGoal() {
  if (!state.goalCells.length) {
    return;
  }
  state.goalCells.pop();
  state.result = null;
  renderPlanner();
  syncLabels();
}

function clearMission(render = true) {
  state.startCell = null;
  state.goalCells = [];
  state.result = null;
  els.rewardMetric.textContent = "-";
  els.stepsMetric.textContent = "-";
  els.goalsMetric.textContent = "-";
  els.terminationMetric.textContent = "-";
  els.pathMeta.textContent = "No run yet.";
  els.jsonOutput.textContent = "{}";
  if (render) {
    renderPlanner();
  }
  syncLabels();
}

function renderPlanner() {
  const canvas = els.plannerCanvas;
  const ctx = canvas.getContext("2d");
  const parentRect = canvas.parentElement.getBoundingClientRect();
  const width = Math.max(360, Math.floor(parentRect.width));
  const height = Math.max(420, Math.floor(parentRect.width * 0.72));
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.floor(width * dpr);
  canvas.height = Math.floor(height * dpr);
  canvas.style.height = `${height}px`;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, width, height);

  if (!state.session?.grid) {
    state.plannerLayout = null;
    drawEmptyState(ctx, width, height, "Build a hex session");
    return;
  }

  const layout = buildPlannerLayout(
    state.session.grid.rows,
    state.session.grid.cols,
    width,
    height,
  );
  state.plannerLayout = layout;
  const occupancy = state.session.grid.occupancy;
  const pathCells = state.result?.path?.cells || [];
  const pathKey = new Set(pathCells.map(cellKey));
  const goalKey = new Set(state.goalCells.map(cellKey));

  ctx.fillStyle = "#faf6ee";
  ctx.fillRect(0, 0, width, height);

  for (const cell of layout.cells) {
    const blocked = Boolean(occupancy[cell.row]?.[cell.col]);
    const key = `${cell.row}:${cell.col}`;
    let fill = blocked ? "#524f47" : "#fffdf8";
    let stroke = blocked ? "#3b392f" : "#c9c1b1";
    if (pathKey.has(key)) {
      fill = "#b8e1dd";
      stroke = "#0d5c63";
    }
    if (goalKey.has(key)) {
      fill = "#f6c47d";
      stroke = "#d97706";
    }
    if (sameCell(cell, state.startCell)) {
      fill = "#b9dfbb";
      stroke = "#2b6a2b";
    }
    drawHex(ctx, cell.points, fill, stroke, blocked ? 0.95 : 1);
  }

  if (pathCells.length > 1) {
    ctx.save();
    ctx.strokeStyle = "#0d5c63";
    ctx.lineWidth = Math.max(2, layout.radius * 0.18);
    ctx.lineCap = "round";
    ctx.lineJoin = "round";
    ctx.beginPath();
    pathCells.forEach((cell, index) => {
      const match = layout.lookup.get(cellKey(cell));
      if (!match) {
        return;
      }
      if (index === 0) {
        ctx.moveTo(match.cx, match.cy);
      } else {
        ctx.lineTo(match.cx, match.cy);
      }
    });
    ctx.stroke();
    ctx.restore();
  }
}

function buildPlannerLayout(rows, cols, width, height) {
  const sqrt3 = Math.sqrt(3);
  const pad = 18;
  const template = [
    [0.0, 1.0],
    [sqrt3 / 2.0, 0.5],
    [sqrt3 / 2.0, -0.5],
    [0.0, -1.0],
    [-sqrt3 / 2.0, -0.5],
    [-sqrt3 / 2.0, 0.5],
  ];

  const base = [];
  let minX = Number.POSITIVE_INFINITY;
  let maxX = Number.NEGATIVE_INFINITY;
  let minY = Number.POSITIVE_INFINITY;
  let maxY = Number.NEGATIVE_INFINITY;

  for (let row = 0; row < rows; row += 1) {
    const cy = 1.5 * row;
    for (let col = 0; col < cols; col += 1) {
      const cx = sqrt3 * (col + 0.5 * (row & 1));
      base.push({ row, col, cx, cy });
      minX = Math.min(minX, cx - sqrt3 / 2);
      maxX = Math.max(maxX, cx + sqrt3 / 2);
      minY = Math.min(minY, cy - 1);
      maxY = Math.max(maxY, cy + 1);
    }
  }

  const scale = Math.min(
    (width - pad * 2) / Math.max(maxX - minX, 1),
    (height - pad * 2) / Math.max(maxY - minY, 1),
  );

  const cells = [];
  const lookup = new Map();
  for (const cell of base) {
    const cx = (cell.cx - minX) * scale + pad;
    const cy = (cell.cy - minY) * scale + pad;
    const points = template.map(([dx, dy]) => ({
      x: cx + dx * scale,
      y: cy + dy * scale,
    }));
    const current = { row: cell.row, col: cell.col, cx, cy, points };
    cells.push(current);
    lookup.set(cellKey(current), current);
  }

  return {
    radius: scale,
    cells,
    lookup,
  };
}

function drawHex(ctx, points, fill, stroke, alpha) {
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.beginPath();
  ctx.moveTo(points[0].x, points[0].y);
  for (let i = 1; i < points.length; i += 1) {
    ctx.lineTo(points[i].x, points[i].y);
  }
  ctx.closePath();
  ctx.fillStyle = fill;
  ctx.strokeStyle = stroke;
  ctx.lineWidth = 1;
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

function handlePlannerClick(event) {
  if (!state.plannerLayout || !state.session?.grid) {
    return;
  }
  const rect = els.plannerCanvas.getBoundingClientRect();
  const x = event.clientX - rect.left;
  const y = event.clientY - rect.top;
  const hit = findNearestCell(x, y, state.plannerLayout);
  if (!hit) {
    return;
  }
  if (Boolean(state.session.grid.occupancy[hit.row]?.[hit.col])) {
    return;
  }

  if (state.mode === "start") {
    state.startCell = { row: hit.row, col: hit.col };
    state.result = null;
  } else {
    if (!state.startCell) {
      setStatus("Pick a start cell first.", true);
      return;
    }
    if (sameCell(hit, state.startCell)) {
      return;
    }
    if (state.goalCells.some((goal) => sameCell(goal, hit))) {
      return;
    }
    if (state.goalCells.length >= intValue(els.maxGoalsInput.value)) {
      setStatus("Goal count reached the current max_goals value.", true);
      return;
    }
    state.goalCells.push({ row: hit.row, col: hit.col });
    state.result = null;
  }

  renderPlanner();
  syncLabels();
}

function findNearestCell(x, y, layout) {
  let best = null;
  let bestDist = Number.POSITIVE_INFINITY;
  for (const cell of layout.cells) {
    const distance = Math.hypot(x - cell.cx, y - cell.cy);
    if (distance < bestDist && distance <= layout.radius * 1.1) {
      best = cell;
      bestDist = distance;
    }
  }
  return best;
}

async function runMission() {
  if (!state.session?.session_id) {
    setStatus("Build a hex session first.", true);
    return;
  }
  if (!state.startCell) {
    setStatus("Select a start cell.", true);
    return;
  }
  if (!state.goalCells.length) {
    setStatus("Select at least one goal cell.", true);
    return;
  }
  try {
    setStatus("Running inference...", false);
    const payload = await postJson("/api/simulate", {
      session_id: state.session.session_id,
      start_cell: state.startCell,
      goal_cells: state.goalCells,
      deterministic: els.deterministicInput.checked,
    });
    state.result = payload;
    renderPlanner();
    renderResult();
    setStatus("Mission run completed.", false);
  } catch (error) {
    setStatus(error.message, true);
  }
}

function renderResult() {
  if (!state.result) {
    return;
  }
  const summary = state.result.summary || {};
  els.rewardMetric.textContent = formatNumber(summary.reward_total);
  els.stepsMetric.textContent = String(summary.steps ?? "-");
  els.goalsMetric.textContent = `${summary.goals_completed ?? 0}/${summary.goals_total ?? 0}`;
  els.terminationMetric.textContent = summary.termination_reason || "-";
  els.pathMeta.textContent = `${state.result.path?.cells?.length ?? 0} cells in returned path`;
  els.jsonOutput.textContent = JSON.stringify(state.result, null, 2);
}

function downloadJson() {
  const payload = state.result || state.session || state.preview || {};
  const blob = new Blob([JSON.stringify(payload, null, 2)], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement("a");
  anchor.href = url;
  anchor.download = state.result ? "mission_result.json" : "portable_bundle_payload.json";
  document.body.appendChild(anchor);
  anchor.click();
  anchor.remove();
  URL.revokeObjectURL(url);
}

function syncLabels() {
  els.modeLabel.textContent = state.mode;
  els.startLabel.textContent = state.startCell ? cellKey(state.startCell).replace(":", ",") : "-";
  els.goalLabel.textContent = state.goalCells.length
    ? state.goalCells.map((cell) => cellKey(cell).replace(":", ",")).join(" | ")
    : "-";
  els.roiLabel.textContent = state.roi
    ? `ROI ${(state.roi.x1 - state.roi.x0).toFixed(2)} x ${(state.roi.y1 - state.roi.y0).toFixed(2)}`
    : "Drag on the preview to choose an ROI.";
}

function setStatus(message, isError) {
  els.statusText.textContent = message;
  els.statusText.style.color = isError ? "#c2410c" : "#69707c";
}

function formatNumber(value) {
  if (typeof value !== "number" || !Number.isFinite(value)) {
    return "-";
  }
  return value.toFixed(2);
}

function cellKey(cell) {
  return `${cell.row}:${cell.col}`;
}

function sameCell(a, b) {
  return Boolean(a && b && a.row === b.row && a.col === b.col);
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function intValue(value) {
  return parseInt(value, 10);
}

function numberValue(value) {
  return Number(value);
}

async function fetchJson(url) {
  const response = await fetch(url, {
    headers: { Accept: "application/json" },
  });
  const payload = await parseJson(response);
  if (!response.ok) {
    throw new Error(payload.error || `Request failed with ${response.status}`);
  }
  return payload;
}

async function postJson(url, body) {
  const response = await fetch(url, {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
      Accept: "application/json",
    },
    body: JSON.stringify(body),
  });
  const payload = await parseJson(response);
  if (!response.ok) {
    throw new Error(payload.error || `Request failed with ${response.status}`);
  }
  return payload;
}

async function parseJson(response) {
  try {
    return await response.json();
  } catch {
    return {};
  }
}
