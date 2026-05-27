/**
 * DSS Log Analyzer — Main entry point.
 */

import { getConfig } from "./js/config.js";
import { AGENT_COLORS, mapPalette } from "./js/palette.js";
import { buildMapStyle } from "./js/map_style.js";
import { fetchScenarios, fetchScenario, pollScenario, fetchDynamicsAnalysis, saveDynamicsAnalysis } from "./js/api.js";
import { buildTimeline } from "./js/timeline.js";
import { createDetailPanel } from "./js/detail_panel.js";
import { createMapController } from "./js/map_view.js";
import { createReplanPanel } from "./js/replan_panel.js";
import { createAircraftFilter } from "./js/aircraft_filter.js";
import { createAnalysisPanel } from "./js/analysis_panel.js";
import { createDynamicsPanel } from "./js/dynamics_panel.js";

const config = getConfig();
const style = buildMapStyle(config, mapPalette);

const map = new maplibregl.Map({
  container: "map",
  style,
  center: config.center,
  zoom: config.startZoom,
  minZoom: config.minZoom,
  maxZoom: config.maxZoom,
  attributionControl: false,
});

map.addControl(new maplibregl.NavigationControl({ showCompass: true }), "top-right");

const scenarioSelect = document.getElementById("scenario-select");
const toolbarStatus = document.getElementById("toolbar-status");
const detailPanelEl = document.getElementById("detail-panel");
const detailBody = document.getElementById("detail-body");
const detailToggle = document.getElementById("detail-panel-toggle");
const timelineTrack = document.getElementById("timeline-track");
const filterContainer = document.getElementById("aircraft-filter");
const replanModal = document.getElementById("replan-modal");
const analysisPanelEl = document.getElementById("analysis-panel");
const coveragePanelEl = document.getElementById("coverage-panel");
const analysisBtn = document.getElementById("toolbar-analysis-btn");
const dynamicsBtn = document.getElementById("toolbar-dynamics-btn");

let mapCtrl = null;
let analysisCtrl = null;
let dynamicsCtrl = null;
let detailCtrl = null;
let replanCtrl = null;
let timelineCtrl = null;
let filterCtrl = null;
let currentScenario = null;
let currentPlan = null;
let currentScenarioName = null;
let currentVersion = 0;
let livePollingTimer = null;
let uiInitialized = false;
const LIVE_POLL_MS = 3000;

// ---- App ready ----

const initializeMapController = () => {
  if (mapCtrl) return;
  mapCtrl = createMapController(map);
  if (currentScenario) {
    mapCtrl.clearPaths();
    if (currentScenario.referenceFeatures) mapCtrl.loadReferenceGeoJSON(currentScenario.referenceFeatures);
    if (currentScenario.inputAreaFeatures) mapCtrl.loadAreaGeoJSON(currentScenario.inputAreaFeatures);
    if (currentPlan) mapCtrl.showPlanResolved(currentPlan, currentScenario);
    if (filterCtrl) {
      const vis = filterCtrl.getVisibility();
      for (const [id, visible] of Object.entries(vis)) {
        mapCtrl.setAgentVisible(Number(id), visible);
      }
    }
    initTrackPlayback(currentScenario.playback0401 || currentScenario.tracks);
    renderCoveragePanel(currentScenario.footprintCoverage, currentScenario.playback0401);
  }
};

const initializeUi = async () => {
  if (uiInitialized) return;
  uiInitialized = true;
  detailCtrl = createDetailPanel(detailBody);
  replanCtrl = createReplanPanel(replanModal);

  filterCtrl = createAircraftFilter(filterContainer, {
    onToggle: (aircraftId, visible) => {
      if (mapCtrl) mapCtrl.setAgentVisible(aircraftId, visible);
    },
  });

  analysisCtrl = createAnalysisPanel(analysisPanelEl);
  dynamicsCtrl = createDynamicsPanel(analysisPanelEl, {
    onSave: async () => {
      if (!currentScenarioName) throw new Error("No scenario selected");
      const saved = await saveDynamicsAnalysis(currentScenarioName);
      const fileName = saved?.fileNames?.markdown || saved?.fileNames?.json || "report";
      setStatus(`Dynamics report saved - ${fileName}`);
      return saved;
    },
  });

  try {
    setStatus("시나리오 목록 불러오는 중...");
    const scenarios = await fetchScenarios();
    populateScenarios(scenarios);
    setStatus(`${scenarios.length}개 시나리오 로드됨`);
  } catch (err) {
    console.error("Failed to fetch scenarios:", err);
    setStatus("시나리오 목록 로드 실패 — 폴더를 직접 선택하세요");
    enableManualInput();
  }
};

map.on("load", () => {
  initializeMapController();
  void initializeUi();
});

void initializeUi();

// ---- Scenario select ----

scenarioSelect.addEventListener("change", () => void loadScenario(scenarioSelect.value));

async function loadScenario(name) {
  if (!name) { resetAll(); return; }
  try {
    setStatus(`"${name}" 불러오는 중...`);
    const { data: raw, version } = await fetchScenario(name);
    currentScenarioName = name;
    currentVersion = version;
    currentScenario = normalizeScenario(raw);

    if (mapCtrl) {
      mapCtrl.clearPaths();
      if (currentScenario.referenceFeatures) mapCtrl.loadReferenceGeoJSON(currentScenario.referenceFeatures);
      if (currentScenario.inputAreaFeatures) mapCtrl.loadAreaGeoJSON(currentScenario.inputAreaFeatures);
    }

    timelineCtrl = buildTimeline(timelineTrack, currentScenario, {
      onPlanSelect: handlePlanSelect,
      onReplanSelect: handleReplanSelect,
    });

    const firstPlan = currentScenario.timeline.find((e) => e.plan);
    if (firstPlan) {
      timelineCtrl.selectPlan(firstPlan.plan.missionPlanID);
      handlePlanSelect(firstPlan.plan);
    }

    // Load 0401 tracks and sensor footprints.
    initTrackPlayback(currentScenario.playback0401 || currentScenario.tracks);
    renderCoveragePanel(currentScenario.footprintCoverage, currentScenario.playback0401);

    detailPanelEl.setAttribute("aria-hidden", "false");
    if (analysisBtn) analysisBtn.disabled = false;
    if (dynamicsBtn) dynamicsBtn.disabled = false;
    setStatus(`"${name}" 로드 완료 (v${currentVersion})`);

    // Start live polling
    startLivePolling(name);
  } catch (err) {
    console.error("Failed to fetch scenario:", err);
    setStatus("시나리오 로드 실패: " + err.message);
  }
}

// ---- Detail panel toggle ----

detailToggle?.addEventListener("click", () => {
  const hidden = detailPanelEl.getAttribute("aria-hidden") === "true";
  detailPanelEl.setAttribute("aria-hidden", hidden ? "false" : "true");
});

// ---- Analysis panel ----

analysisBtn?.addEventListener("click", async () => {
  if (!currentScenarioName) return;
  try {
    analysisBtn.disabled = true;
    analysisBtn.textContent = "분석 중...";
    setStatus("ICD 검증 실행 중...");
    const res = await fetch(`/api/scenario/validate?name=${encodeURIComponent(currentScenarioName)}`);
    if (!res.ok) throw new Error(`validate: ${res.status}`);
    const body = await res.json();
    if (!body.ok) throw new Error(body.error || "validation failed");
    analysisCtrl = createAnalysisPanel(analysisPanelEl);
    if (analysisCtrl) analysisCtrl.show(body.issues || []);
    const s = body.summary || {};
    setStatus(`분석 완료 — 에러 ${s.error || 0}, 경고 ${s.warning || 0}, 정보 ${s.info || 0}`);
  } catch (err) {
    console.error("Validation failed:", err);
    setStatus("분석 실패: " + err.message);
  } finally {
    analysisBtn.disabled = false;
    analysisBtn.textContent = "분석";
  }
});

dynamicsBtn?.addEventListener("click", async () => {
  if (!currentScenarioName) return;
  try {
    dynamicsBtn.disabled = true;
    dynamicsBtn.textContent = "Dynamics...";
    if (dynamicsCtrl) dynamicsCtrl.showLoading(currentScenarioName);
    setStatus("0401 dynamics analysis running...");
    const body = await fetchDynamicsAnalysis(currentScenarioName);
    if (dynamicsCtrl) dynamicsCtrl.show(body);
    const usable = body?.cohort?.usableUavCount || 0;
    const radius = body?.recommendations?.basis?.medianTurnRadiusM;
    setStatus(`Dynamics complete - usable UAV ${usable}, median radius ${radius ? Math.round(radius) : "-"}m`);
  } catch (err) {
    console.error("Dynamics analysis failed:", err);
    if (dynamicsCtrl) dynamicsCtrl.showError(err.message || String(err));
    setStatus("Dynamics failed: " + err.message);
  } finally {
    dynamicsBtn.disabled = false;
    dynamicsBtn.textContent = "Dynamics";
  }
});

// ---- Handlers ----

function handlePlanSelect(plan) {
  currentPlan = plan;
  if (mapCtrl && currentScenario) {
    mapCtrl.showPlanResolved(plan, currentScenario);
    if (filterCtrl) {
      const vis = filterCtrl.getVisibility();
      for (const [id, visible] of Object.entries(vis)) {
        mapCtrl.setAgentVisible(Number(id), visible);
      }
    }
  }
  if (detailCtrl && currentScenario) {
    const replanInfo = findReplanForPlan(currentScenario, plan.missionPlanID);
    detailCtrl.show(plan, currentScenario, {
      replanInfo,
      onPathClick: (pathId, coords) => { if (mapCtrl) mapCtrl.flyToPath(coords); },
    });
  }
}

function handleReplanSelect(replan) {
  if (replanCtrl && currentScenario) {
    replanCtrl.show(replan, currentScenario, {
      onPlanJump: (planId) => {
        const entry = currentScenario.timeline.find(
          (e) => e.plan && String(e.plan.missionPlanID) === String(planId),
        );
        if (entry && timelineCtrl) {
          timelineCtrl.selectPlan(planId);
          handlePlanSelect(entry.plan);
        }
      },
    });
  }
}

// ---- Data normalization ----

function normalizeScenario(raw) {
  const plans = (raw.missionPlans || []).map((mp) => ({
    missionPlanID: mp.missionPlanID,
    timestamp: (mp.plan || {}).timestamp || mp.timestamp,
    planningTime: (mp.plan || {}).planningTime,
    inputMissionPackageID: (mp.plan || {}).inputMissionPackageID,
    aircraftList: (mp.plan || {}).aircraftList || [],
    resolved: mp.resolved || {},
  }));

  const replans = (raw.replanRequests || []).map((rp) => ({
    timestamp: rp.timestamp,
    reason: rp.replanRequest || rp.reason || "",
    replanLevel: rp.replanLevel,
    source: rp.source,
    detail: rp.replanDetail || rp.detail || {},
    optionList: rp.optionList || rp.pendingOptionList || [],
    inputMissionIDList: rp.inputMissionIDList || [],
    linkedMissionPlanID: null,
  }));

  // Link replans to resulting plans
  for (const rp of replans) {
    for (const opt of rp.optionList) {
      if (opt.missionPlanID) {
        rp.linkedMissionPlanID = opt.missionPlanID;
        break;
      }
    }
  }

  // Build interleaved timeline
  const timelineRaw = raw.timeline || [];
  const timeline = [];
  for (const entry of timelineRaw) {
    if (entry.type === "replan") {
      const rp = replans.find((r) => r.timestamp === entry.timestamp) || entry.data || {};
      timeline.push({
        type: "replan",
        replan: {
          ...rp,
          reason: rp.reason || rp.replanRequest || (entry.data || {}).replanRequest || "",
          linkedMissionPlanID: entry.linkedMissionPlanID || rp.linkedMissionPlanID,
        },
      });
    } else if (entry.type === "missionPlan") {
      const plan = plans.find((p) => p.missionPlanID === entry.missionPlanID);
      if (plan) {
        plan.isSelected = !!entry.isSelected;
        timeline.push({ type: "plan", plan });
      }
    }
  }

  // Flatten flight paths for map_view: pathID → {coordinates: [[lon,lat], ...], waypoints: [...]}
  const flightPaths = {};
  for (const [key, fp] of Object.entries(raw.flightPaths || {})) {
    flightPaths[key] = fp;
  }

  return {
    ...raw,
    plans,
    replans,
    timeline,
    flightPaths,
    referenceFeatures: raw.referenceFeatures || null,
    inputAreaFeatures: raw.missionSectionFeatures || buildInputAreaFeatures(raw.inputMissionPlan),
    playback0401: raw.playback0401 || { tracks: raw.tracks || {}, footprints: {} },
  };
}

function buildInputAreaFeatures(inputPlan) {
  if (!inputPlan) return null;
  const features = [];
  const missions = inputPlan.inputMissionList || inputPlan.missionList || [];
  for (const m of missions) {
    const coordList = m.coordinateList || m.areaCoordinateList || [];
    const coords = coordList
      .map((c) => {
        const lon = c.longitude ?? c.lon;
        const lat = c.latitude ?? c.lat;
        return lon != null && lat != null ? [lon, lat] : null;
      })
      .filter(Boolean);
    if (coords.length < 2) continue;
    const missionType = m.inputMissionType ?? m.missionType;
    if (coords.length >= 3 && (missionType === 2 || missionType === 3)) {
      // Close ring
      const first = coords[0], last = coords[coords.length - 1];
      if (first[0] !== last[0] || first[1] !== last[1]) coords.push([...first]);
      features.push({
        type: "Feature",
        geometry: { type: "Polygon", coordinates: [coords] },
        properties: { inputMissionID: m.inputMissionID, missionType },
      });
    } else {
      features.push({
        type: "Feature",
        geometry: { type: "LineString", coordinates: coords },
        properties: { inputMissionID: m.inputMissionID, missionType },
      });
    }
  }
  return features.length > 0 ? { type: "FeatureCollection", features } : null;
}

// ---- Helpers ----

function populateScenarios(scenarios) {
  while (scenarioSelect.options.length > 1) scenarioSelect.remove(1);
  for (const entry of scenarios) {
    const name = typeof entry === "string" ? entry : entry.name;
    if (!name) continue;
    const opt = document.createElement("option");
    opt.value = name;
    opt.textContent = name;
    scenarioSelect.appendChild(opt);
  }
}

function enableManualInput() {
  const wrapper = scenarioSelect.parentElement;
  const input = document.createElement("input");
  input.type = "text";
  input.className = "scenario-select";
  input.placeholder = "시나리오 폴더명 입력";
  input.style.minWidth = "320px";
  wrapper.insertBefore(input, scenarioSelect.nextSibling);
  const btn = document.createElement("button");
  btn.textContent = "로드";
  btn.className = "toolbar-btn";
  btn.style.marginLeft = "6px";
  wrapper.insertBefore(btn, input.nextSibling);
  btn.addEventListener("click", () => {
    const name = input.value.trim();
    if (name) void loadScenario(name);
  });
  input.addEventListener("keydown", (e) => { if (e.key === "Enter") btn.click(); });
}

function resetAll() {
  stopLivePolling();
  currentScenario = null;
  currentPlan = null;
  currentScenarioName = null;
  currentVersion = 0;
  timelineTrack.innerHTML = '<div class="tl-empty">시나리오를 선택하세요</div>';
  detailPanelEl.setAttribute("aria-hidden", "true");
  if (detailCtrl) detailCtrl.hide();
  if (analysisCtrl) analysisCtrl.hide();
  if (dynamicsCtrl) dynamicsCtrl.hide();
  if (mapCtrl) {
    mapCtrl.clearPaths();
    mapCtrl.clearTracks();
  }
  renderCoveragePanel(null, null);
  if (filterCtrl) filterCtrl.reset();
  if (analysisBtn) analysisBtn.disabled = true;
  if (dynamicsBtn) dynamicsBtn.disabled = true;
  setStatus("");
}

function findReplanForPlan(scenario, planId) {
  for (const entry of scenario.timeline || []) {
    if (entry.replan && String(entry.replan.linkedMissionPlanID) === String(planId)) return entry.replan;
  }
  return null;
}

function setStatus(text) {
  if (toolbarStatus) toolbarStatus.textContent = text || "";
}

// ---- Live polling (실시간 갱신) ----

function renderCoveragePanel(coverage, playback) {
  if (!coveragePanelEl) return;
  const sections = coverage?.sections || [];
  if (!sections.length) {
    coveragePanelEl.setAttribute("aria-hidden", "true");
    coveragePanelEl.innerHTML = "";
    return;
  }
  const summary = coverage.summary || {};
  const sourceKind = playback?.sourceKind || "-";
  const fileCount = playback?.fileCount ?? 0;
  const messageCount = playback?.messageCount ?? 0;
  const footprintCount = playback?.footprintCount ?? summary.footprintSamples ?? 0;
  const covered = Number(summary.coveredPercent || 0);
  const status = coverage.available ? "READY" : (coverage.reason || "NO DATA");
  const rows = sections
    .map((section) => {
      const pct = Math.max(0, Math.min(100, Number(section.coveragePercent || 0)));
      const label = `${section.sectionId || ""} / M${section.inputMissionID ?? "-"}`;
      const type = section.sectionType === "corridor" ? "CORRIDOR" : "AREA";
      return `
        <div class="coverage-row">
          <div class="coverage-row-head">
            <span>${escapeHtml(label)}</span>
            <b>${pct.toFixed(1)}%</b>
          </div>
          <div class="coverage-meta">${type}${section.widthM ? ` / ${Math.round(section.widthM)}m` : ""}</div>
          <div class="coverage-bar"><span style="width:${pct}%"></span></div>
        </div>`;
    })
    .join("");

  coveragePanelEl.innerHTML = `
    <div class="coverage-head">
      <div>
        <div class="coverage-title">0401 FOOTPRINT</div>
        <div class="coverage-subtitle">${escapeHtml(sourceKind)} / ${fileCount} files / ${messageCount} messages</div>
      </div>
      <div class="coverage-score">${covered.toFixed(1)}%</div>
    </div>
    <div class="coverage-stats">
      <span>${escapeHtml(status)}</span>
      <span>${footprintCount} footprints</span>
      <span>${sections.length} sections</span>
    </div>
    <div class="coverage-list">${rows}</div>`;
  coveragePanelEl.setAttribute("aria-hidden", "false");
}

function escapeHtml(value) {
  return String(value ?? "")
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;");
}

function startLivePolling(name) {
  stopLivePolling();
  livePollingTimer = setInterval(() => void livePoll(name), LIVE_POLL_MS);
}

function stopLivePolling() {
  if (livePollingTimer) { clearInterval(livePollingTimer); livePollingTimer = null; }
}

async function livePoll(name) {
  if (name !== currentScenarioName) return;
  try {
    const result = await pollScenario(name, currentVersion);
    if (!result || !result.changed) return;

    // Remember current selection
    const selectedPlanId = currentPlan?.missionPlanID;

    currentVersion = result.version;
    currentScenario = normalizeScenario(result.data);

    // Rebuild timeline
    timelineCtrl = buildTimeline(timelineTrack, currentScenario, {
      onPlanSelect: handlePlanSelect,
      onReplanSelect: handleReplanSelect,
    });

    // Restore selection or select last plan
    const restorePlan = selectedPlanId
      ? currentScenario.timeline.find((e) => e.plan && e.plan.missionPlanID === selectedPlanId)
      : null;
    const lastPlan = [...currentScenario.timeline].reverse().find((e) => e.plan);
    const newPlanCount = currentScenario.timeline.filter((e) => e.plan).length;
    const prevPlanCount = currentPlan ? -1 : 0; // force update if new plans added

    if (restorePlan) {
      timelineCtrl.selectPlan(restorePlan.plan.missionPlanID);
      // 같은 plan이면 맵만 갱신, detail panel은 유지
      currentPlan = restorePlan.plan;
      if (mapCtrl) {
        mapCtrl.showPlanResolved(restorePlan.plan, currentScenario);
        if (filterCtrl) {
          const vis = filterCtrl.getVisibility();
          for (const [id, visible] of Object.entries(vis)) mapCtrl.setAgentVisible(Number(id), visible);
        }
      }
    } else if (lastPlan) {
      timelineCtrl.selectPlan(lastPlan.plan.missionPlanID);
      handlePlanSelect(lastPlan.plan);
    }

    // Refresh 0401 playback and coverage (preserve playback position)
    updateTrackData(currentScenario.playback0401 || currentScenario.tracks);
    renderCoveragePanel(currentScenario.footprintCoverage, currentScenario.playback0401);

    setStatus(`"${name}" 갱신됨 (v${currentVersion})`);
  } catch {
    // Silently ignore poll errors
  }
}

// ---- Track playback (항적 재생) ----

const trackBar = document.getElementById("track-bar");
const trackPlayBtn = document.getElementById("track-play");
const trackAllToggle = document.getElementById("track-all-toggle");
const trackSlider = document.getElementById("track-slider");
const trackTimeEl = document.getElementById("track-time");
let trackPlaying = false;
let trackAnimFrame = null;
let trackInfo = null;
let trackStartTs = 0;
let trackAllMode = false;

function initTrackPlayback(tracks) {
  stopTrackPlayback();
  if (!hasTrackPayload(tracks) || !mapCtrl) {
    trackBar?.setAttribute("aria-hidden", "true");
    return;
  }
  setTrackAllMode(false);
  trackInfo = mapCtrl.loadTracks(tracks);
  if (!trackInfo || trackInfo.totalFrames < 2) {
    trackBar?.setAttribute("aria-hidden", "true");
    return;
  }
  trackSlider.max = String(trackInfo.totalFrames - 1);
  trackSlider.value = "0";
  trackStartTs = trackInfo.minTs;
  updateTrackTime(0);
  mapCtrl.setTrackFrame(0);
  trackBar?.setAttribute("aria-hidden", "false");
}

function updateTrackData(tracks) {
  if (!hasTrackPayload(tracks) || !mapCtrl) return;
  const prevFrame = trackInfo ? Number(trackSlider.value) : 0;
  const wasPlaying = trackPlaying;
  const wasAllMode = trackAllMode;

  trackInfo = mapCtrl.loadTracks(tracks);
  if (!trackInfo || trackInfo.totalFrames < 2) return;

  trackSlider.max = String(trackInfo.totalFrames - 1);
  // Clamp previous frame to new range
  const restoredFrame = Math.min(prevFrame, trackInfo.totalFrames - 1);
  trackSlider.value = String(restoredFrame);
  trackStartTs = trackInfo.minTs;
  updateTrackTime(restoredFrame);
  mapCtrl.setTrackFrame(restoredFrame);
  setTrackAllMode(wasAllMode);
  trackBar?.setAttribute("aria-hidden", "false");

  // Resume playing if it was playing
  if (wasPlaying && !trackPlaying) {
    trackPlayBtn?.click();
  }
}

function updateTrackTime(frame) {
  if (!trackInfo) return;
  if (trackAllMode) {
    if (trackTimeEl) trackTimeEl.textContent = "ALL";
    return;
  }
  const ts = trackInfo.minTs + (trackInfo.maxTs - trackInfo.minTs) * (frame / Math.max(1, trackInfo.totalFrames - 1));
  const elapsed = Math.max(0, Math.round((ts - trackInfo.minTs) / 1000));
  const total = Math.max(0, Math.round((trackInfo.maxTs - trackInfo.minTs) / 1000));
  const fmt = (s) => `${String(Math.floor(s / 60)).padStart(2, "0")}:${String(s % 60).padStart(2, "0")}`;
  if (trackTimeEl) trackTimeEl.textContent = `${fmt(elapsed)} / ${fmt(total)}`;
}

function stopTrackPlayback() {
  trackPlaying = false;
  if (trackAnimFrame) { cancelAnimationFrame(trackAnimFrame); trackAnimFrame = null; }
  trackPlayBtn?.classList.remove("is-playing");
  if (trackPlayBtn) trackPlayBtn.innerHTML = "&#9654;";
}

function setTrackAllMode(enabled, options = {}) {
  const next = !!enabled;
  if (next && options.stopPlayback) stopTrackPlayback();
  trackAllMode = next;
  trackAllToggle?.classList.toggle("is-active", trackAllMode);
  if (mapCtrl?.setTrackAll) mapCtrl.setTrackAll(trackAllMode);
  updateTrackTime(Number(trackSlider?.value || 0));
}

trackPlayBtn?.addEventListener("click", () => {
  if (trackPlaying) { stopTrackPlayback(); return; }
  if (trackAllMode) setTrackAllMode(false);
  trackPlaying = true;
  trackPlayBtn.classList.add("is-playing");
  trackPlayBtn.innerHTML = "&#9646;&#9646;";
  let lastTime = performance.now();
  const speed = 5; // 5x realtime
  const step = (now) => {
    if (!trackPlaying) return;
    const dt = now - lastTime;
    lastTime = now;
    let frame = Number(trackSlider.value);
    frame += dt * speed / 200; // ~200ms per log entry
    if (frame >= trackInfo.totalFrames - 1) { frame = 0; } // loop
    trackSlider.value = String(Math.floor(frame));
    mapCtrl.setTrackFrame(Math.floor(frame));
    updateTrackTime(Math.floor(frame));
    trackAnimFrame = requestAnimationFrame(step);
  };
  trackAnimFrame = requestAnimationFrame(step);
});

trackSlider?.addEventListener("input", () => {
  const frame = Number(trackSlider.value);
  if (trackAllMode) setTrackAllMode(false);
  if (mapCtrl) mapCtrl.setTrackFrame(frame);
  updateTrackTime(frame);
});

trackAllToggle?.addEventListener("click", () => {
  if (!mapCtrl || !trackInfo) return;
  setTrackAllMode(!trackAllMode, { stopPlayback: !trackAllMode });
});

function hasTrackPayload(payload) {
  const tracks = payload?.tracks || payload;
  return !!tracks && Object.keys(tracks).length > 0;
}
