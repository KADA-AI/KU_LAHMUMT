/**
 * DSS Log Analyzer — Main entry point.
 */

import { getConfig } from "./js/config.js";
import { AGENT_COLORS, AGENT_LABELS, mapPalette } from "./js/palette.js";
import { buildMapStyle } from "./js/map_style.js";
import { fetchScenarios, fetchScenario, pollScenario, fetchDynamicsAnalysis, saveDynamicsAnalysis } from "./js/api.js";
import { buildTimeline } from "./js/timeline.js?v=20260724-plan-time-focus";
import { createDetailPanel } from "./js/detail_panel.js";
import { createMapController } from "./js/map_view.js?v=20260724-plan-time-focus";
import { createReplanPanel } from "./js/replan_panel.js";
import { createAircraftFilter } from "./js/aircraft_filter.js";
import { createAnalysisPanel } from "./js/analysis_panel.js";
import { createDynamicsPanel } from "./js/dynamics_panel.js";
import { createMissionLayerPanel } from "./js/mission_layers.js?v=20260724-plan-time-focus";

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
const missionLayerPanelEl = document.getElementById("mission-layer-panel");

let mapCtrl = null;
let analysisCtrl = null;
let dynamicsCtrl = null;
let detailCtrl = null;
let replanCtrl = null;
let timelineCtrl = null;
let filterCtrl = null;
let missionLayerCtrl = null;
let currentScenario = null;
let currentPlan = null;
let timelineTimeFocusPlanId = null;
let currentScenarioName = null;
let currentVersion = 0;
let coverageMode = "raw";
let coveragePanelPosition = null;
let livePollingTimer = null;
let uiInitialized = false;
let scenarioLoadRequestId = 0;
const LIVE_POLL_MS = 3000;
const EMPTY_FEATURE_COLLECTION = { type: "FeatureCollection", features: [] };

function selectedInputPackageId() {
  const value = currentPlan?.inputMissionPackageID ?? currentScenario?.defaultInputMissionPackageID;
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : null;
}

function filterFeatureCollectionByPackage(collection, packageId = selectedInputPackageId()) {
  if (!collection?.features || packageId == null) return collection || null;
  const tagged = collection.features.filter((feature) => feature?.properties?.inputMissionPackageID != null);
  if (!tagged.length) return collection;
  return {
    ...collection,
    features: collection.features.filter(
      (feature) => Number(feature?.properties?.inputMissionPackageID) === packageId,
    ),
  };
}

function filterCoverageByPackage(coverage, packageId = selectedInputPackageId()) {
  if (!coverage?.sections || packageId == null) return coverage || null;
  const tagged = coverage.sections.filter((row) => row?.inputMissionPackageID != null);
  if (!tagged.length) return coverage;
  const sections = tagged.filter((row) => Number(row.inputMissionPackageID) === packageId);
  const areaM2 = sections.reduce((sum, row) => sum + Number(row.areaM2 || 0), 0);
  const coveredM2 = sections.reduce((sum, row) => sum + Math.min(Number(row.coveredM2 || 0), Number(row.areaM2 || 0)), 0);
  const values = sections.map((row) => Number(row.coveragePercent || 0));
  return {
    ...coverage,
    sections,
    summary: {
      ...(coverage.summary || {}),
      sectionCount: sections.length,
      coveredPercent: areaM2 > 0 ? Number((coveredM2 / areaM2 * 100).toFixed(2)) : 0,
      minPercent: values.length ? Math.min(...values) : 0,
      maxPercent: values.length ? Math.max(...values) : 0,
      areaM2,
      coveredM2,
    },
  };
}

function getCoveragePayload(mode = coverageMode) {
  if (!currentScenario) return { coverage: null, playback: null };
  const adjusted = currentScenario.footprintCoverage15Hz;
  if (mode === "adjusted" && adjusted?.sections?.length) {
    return { coverage: filterCoverageByPackage(adjusted), playback: currentScenario.playback0401 };
  }
  return { coverage: filterCoverageByPackage(currentScenario.footprintCoverage), playback: currentScenario.playback0401 };
}

function getCoverageFeatures(mode = coverageMode) {
  if (!currentScenario) return null;
  if (mode === "adjusted" && currentScenario.inputAreaFeatures15Hz) {
    return filterFeatureCollectionByPackage(currentScenario.inputAreaFeatures15Hz);
  }
  return filterFeatureCollectionByPackage(currentScenario.inputAreaFeaturesRaw || currentScenario.inputAreaFeatures || null);
}

function refreshCoverageDisplay(options = {}) {
  const { coverage, playback } = getCoveragePayload();
  renderCoveragePanel(coverage, playback);
  if (options.updateMap && mapCtrl) {
    const features = getCoverageFeatures();
    if (features) mapCtrl.loadAreaGeoJSON(features);
  }
}

function setCoverageMode(mode) {
  const next = mode === "adjusted" ? "adjusted" : "raw";
  if (coverageMode === next) return;
  coverageMode = next;
  refreshCoverageDisplay({ updateMap: true });
}

function clampCoveragePanelPosition(x, y) {
  if (!coveragePanelEl) return { x, y };
  const rect = coveragePanelEl.getBoundingClientRect();
  const margin = 8;
  const width = rect.width || 320;
  const height = rect.height || 280;
  return {
    x: Math.max(margin, Math.min(x, window.innerWidth - width - margin)),
    y: Math.max(margin, Math.min(y, window.innerHeight - height - margin)),
  };
}

function applyCoveragePanelPosition() {
  if (!coveragePanelEl) return;
  if (!coveragePanelPosition) {
    coveragePanelEl.classList.remove("is-user-positioned");
    coveragePanelEl.style.left = "";
    coveragePanelEl.style.top = "";
    return;
  }
  coveragePanelPosition = clampCoveragePanelPosition(coveragePanelPosition.x, coveragePanelPosition.y);
  coveragePanelEl.classList.add("is-user-positioned");
  coveragePanelEl.style.left = `${coveragePanelPosition.x}px`;
  coveragePanelEl.style.top = `${coveragePanelPosition.y}px`;
}

function enableCoveragePanelDrag() {
  if (!coveragePanelEl) return;
  const handle = coveragePanelEl.querySelector(".coverage-head");
  if (!handle || handle.dataset.dragBound === "true") return;
  handle.dataset.dragBound = "true";
  handle.title = "Drag to move. Double-click to reset.";
  handle.addEventListener("dblclick", () => {
    coveragePanelPosition = null;
    applyCoveragePanelPosition();
  });
  handle.addEventListener("pointerdown", (event) => {
    if (event.button !== 0) return;
    const rect = coveragePanelEl.getBoundingClientRect();
    const offsetX = event.clientX - rect.left;
    const offsetY = event.clientY - rect.top;
    coveragePanelEl.classList.add("is-dragging");
    coveragePanelEl.setPointerCapture?.(event.pointerId);
    const onMove = (moveEvent) => {
      coveragePanelPosition = clampCoveragePanelPosition(
        moveEvent.clientX - offsetX,
        moveEvent.clientY - offsetY,
      );
      applyCoveragePanelPosition();
    };
    const onUp = () => {
      coveragePanelEl.classList.remove("is-dragging");
      coveragePanelEl.removeEventListener("pointermove", onMove);
      coveragePanelEl.removeEventListener("pointerup", onUp);
      coveragePanelEl.removeEventListener("pointercancel", onUp);
    };
    coveragePanelEl.addEventListener("pointermove", onMove);
    coveragePanelEl.addEventListener("pointerup", onUp);
    coveragePanelEl.addEventListener("pointercancel", onUp);
    event.preventDefault();
  });
}

window.addEventListener("resize", () => {
  if (coveragePanelPosition) applyCoveragePanelPosition();
});

// ---- App ready ----

const initializeMapController = () => {
  if (mapCtrl) return;
  mapCtrl = createMapController(map);
  if (currentScenario) {
    mapCtrl.clearPaths();
    if (currentScenario.referenceFeatures) mapCtrl.loadReferenceGeoJSON(currentScenario.referenceFeatures);
    mapCtrl.setInputMissionPackage(selectedInputPackageId());
    const areaFeatures = getCoverageFeatures();
    if (areaFeatures) mapCtrl.loadAreaGeoJSON(areaFeatures);
    if (currentPlan) mapCtrl.showPlanResolved(currentPlan, currentScenario);
    if (missionLayerCtrl) {
      for (const [kind, visible] of Object.entries(missionLayerCtrl.getLayerVisibility())) {
        mapCtrl.setLayerVisibility(kind, visible);
      }
      mapCtrl.setMissionFocus(missionLayerCtrl.getMissionFocus());
    }
    if (filterCtrl) {
      const vis = filterCtrl.getVisibility();
      for (const [id, visible] of Object.entries(vis)) {
        mapCtrl.setAgentVisible(Number(id), visible);
      }
    }
    initTrackPlayback(currentScenario.playback0401 || currentScenario.tracks);
    refreshCoverageDisplay();
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

  missionLayerCtrl = createMissionLayerPanel(missionLayerPanelEl, {
    onLayerToggle: (kind, visible) => mapCtrl?.setLayerVisibility(kind, visible),
    onMissionFocus: (focus) => mapCtrl?.setMissionFocus(focus),
    onFitMission: (focus) => {
      if (focus?.coordinates?.length) mapCtrl?.flyToPath(focus.coordinates);
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
  const requestId = ++scenarioLoadRequestId;
  stopLivePolling();
  currentScenarioName = null;
  currentScenario = null;
  currentPlan = null;
  currentVersion = 0;
  coverageMode = "raw";
  clearTrackPlaybackUi();
  timelineTrack.innerHTML = '<div class="tl-empty">시나리오 불러오는 중...</div>';
  detailCtrl?.hide();
  detailPanelEl.setAttribute("aria-hidden", "true");
  replanCtrl?.hide();
  analysisCtrl?.hide();
  dynamicsCtrl?.hide();
  missionLayerCtrl?.setAutoFollow(true);
  missionLayerCtrl?.setScenario(null);
  renderCoveragePanel(null, null);
  if (analysisBtn) analysisBtn.disabled = true;
  if (dynamicsBtn) dynamicsBtn.disabled = true;
  if (mapCtrl) {
    mapCtrl.clearPaths();
    mapCtrl.clearTargets();
    mapCtrl.setMissionFocus(null);
    mapCtrl.loadReferenceGeoJSON(EMPTY_FEATURE_COLLECTION);
    mapCtrl.loadAreaGeoJSON(EMPTY_FEATURE_COLLECTION);
  }
  try {
    setStatus(`"${name}" 불러오는 중...`);
    const { data: raw, version } = await fetchScenario(name);
    if (requestId !== scenarioLoadRequestId) return;
    currentScenarioName = name;
    currentVersion = version;
    currentScenario = normalizeScenario(raw);
    currentPlan = null;
    timelineTimeFocusPlanId = null;
    coverageMode = "raw";
    missionLayerCtrl?.setScenario(currentScenario);

    if (mapCtrl) {
      mapCtrl.clearPaths();
      mapCtrl.clearTracks();
      mapCtrl.clearTargets();
      mapCtrl.setMissionFocus(null);
      mapCtrl.setTrackTimeFocus(null);
      mapCtrl.loadReferenceGeoJSON(currentScenario.referenceFeatures || EMPTY_FEATURE_COLLECTION);
      mapCtrl.setInputMissionPackage(selectedInputPackageId());
      const areaFeatures = getCoverageFeatures();
      mapCtrl.loadAreaGeoJSON(areaFeatures || EMPTY_FEATURE_COLLECTION);
    }
    detailCtrl?.hide();
    replanCtrl?.hide();

    timelineCtrl = buildTimeline(timelineTrack, currentScenario, {
      onPlanSelect: handleTimelinePlanClick,
      onReplanSelect: handleReplanSelect,
    });

    const firstPlan = currentScenario.timeline.find((e) => e.plan);
    if (firstPlan) {
      timelineCtrl.selectPlan(firstPlan.plan.missionPlanID);
      handlePlanSelect(firstPlan.plan);
    }

    // Load 0401 tracks and sensor footprints.
    initTrackPlayback(currentScenario.playback0401 || currentScenario.tracks);
    refreshCoverageDisplay();

    detailPanelEl.setAttribute("aria-hidden", "false");
    if (analysisBtn) analysisBtn.disabled = false;
    if (dynamicsBtn) dynamicsBtn.disabled = !hasTrackPayload(currentScenario.playback0401 || currentScenario.tracks);
    const audit = currentScenario.loadAudit || {};
    const playback = currentScenario.playback0401 || {};
    const auditLabel = audit.status === "complete" ? "전체 적재" : audit.status === "partial" ? "부분 적재" : "데이터 없음";
    setStatus(`"${name}" ${auditLabel} · MP ${currentScenario.missionPlanCount || 0} · Path ${currentScenario.flightPathCount || 0} · 0401 ${playback.fileCount || 0}파일/${Number(playback.messageCount || 0).toLocaleString()}건`);

    // Start live polling
    startLivePolling(name);
  } catch (err) {
    if (requestId !== scenarioLoadRequestId) return;
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

function handlePlanSelect(plan, options = {}) {
  currentPlan = plan;
  const targetsInfo = collectTargetsForPlan(currentScenario, plan);
  missionLayerCtrl?.setPlan(plan, currentScenario, {
    preserveFocus: !!(options.preserveFocus || options.fromPlayback),
  });
  if (mapCtrl && currentScenario) {
    mapCtrl.setInputMissionPackage(selectedInputPackageId());
    mapCtrl.showPlanResolved(plan, currentScenario);
    mapCtrl.setTargets(targetsInfo.known);
    if (filterCtrl) {
      const vis = filterCtrl.getVisibility();
      for (const [id, visible] of Object.entries(vis)) {
        mapCtrl.setAgentVisible(Number(id), visible);
      }
    }
  }
  refreshCoverageDisplay({ updateMap: true });
  if (detailCtrl && currentScenario) {
    const replanInfo = findReplanForPlan(currentScenario, plan.missionPlanID);
    detailCtrl.show(plan, currentScenario, {
      replanInfo,
      targets: targetsInfo.known,
      targetTotal: targetsInfo.total,
      onTargetClick: (t) => { if (mapCtrl) mapCtrl.flyToPath([[t.lon, t.lat]]); },
      onPathClick: (pathId, coords) => { if (mapCtrl) mapCtrl.flyToPath(coords); },
    });
  }
}

function handleTimelinePlanClick(plan) {
  stopTrackPlayback();
  missionLayerCtrl?.setAutoFollow(false);
  const removeFocus = String(timelineTimeFocusPlanId) === String(plan.missionPlanID);
  handlePlanSelect(plan);
  if (removeFocus) {
    setTimelinePlanTimeFocus(null);
    setStatus(`Plan ${plan.missionPlanID} 시간 격리 해제 · 전체 경로/Footprint 표시`);
    return;
  }
  const range = setTimelinePlanTimeFocus(plan.missionPlanID);
  if (range) {
    setStatus(`Plan ${plan.missionPlanID} 적용 시간만 표시`);
  } else {
    setStatus(`Plan ${plan.missionPlanID}은 적용 시간 범위를 확인할 수 없습니다.`);
  }
}

function setTimelinePlanTimeFocus(planId) {
  timelineTimeFocusPlanId = planId == null ? null : String(planId);
  const range = timelineTimeFocusPlanId == null
    ? null
    : planAppliedTimeRange(currentScenario, timelineTimeFocusPlanId);
  if (!range) timelineTimeFocusPlanId = null;
  timelineCtrl?.setTimeFocus(timelineTimeFocusPlanId);
  mapCtrl?.setTrackTimeFocus(range);
  return range;
}

function planAppliedTimeRange(scenario, planId) {
  const allPlans = (scenario?.plans || [])
    .map((plan) => ({ plan, timestamp: timestampMs(plan.timestamp) }))
    .filter((entry) => entry.timestamp != null)
    .sort((a, b) => a.timestamp - b.timestamp);
  const applied = allPlans.filter((entry) => entry.plan.isSelected);
  const candidates = applied.length ? applied : allPlans;
  const index = candidates.findIndex(
    (entry) => String(entry.plan.missionPlanID) === String(planId),
  );
  if (index < 0) return null;
  const start = candidates[index].timestamp;
  const next = candidates.slice(index + 1).find((entry) => entry.timestamp > start);
  const playbackEnd = Number(trackInfo?.maxTs);
  const end = next
    ? next.timestamp - 1
    : (Number.isFinite(playbackEnd) ? playbackEnd : start);
  return end >= start ? [start, end] : null;
}

function timestampMs(value) {
  const numeric = Number(value);
  if (Number.isFinite(numeric)) return numeric;
  const parsed = Date.parse(value);
  return Number.isFinite(parsed) ? parsed : null;
}

function handleReplanSelect(replan) {
  if (replanCtrl && currentScenario) {
    replanCtrl.show(replan, currentScenario, {
      onPlanJump: (planId) => {
        const entry = currentScenario.timeline.find(
          (e) => e.plan && String(e.plan.missionPlanID) === String(planId),
        );
        if (entry && timelineCtrl) {
          setTimelinePlanTimeFocus(null);
          timelineCtrl.selectPlan(planId);
          handlePlanSelect(entry.plan);
        }
      },
    });
  }
}

// ---- Targets (적 표적) ----

const TARGET_TYPE_LABELS = {
  0: "표적", 1: "전차", 2: "장갑차", 3: "방사포",
  4: "곡사포", 5: "고정고사포", 6: "군인",
};

function scenarioBaseTs(scenario) {
  let min = null;
  for (const entry of scenario?.timeline || []) {
    const ts = Number(entry.plan?.timestamp ?? entry.replan?.timestamp) || null;
    if (ts != null && (min == null || ts < min)) min = ts;
  }
  return min;
}

function formatRelTs(ts, baseTs) {
  if (ts == null || baseTs == null) return "";
  const s = Math.max(0, Math.round((ts - baseTs) / 1000));
  return `T+${String(Math.floor(s / 60)).padStart(2, "0")}:${String(s % 60).padStart(2, "0")}`;
}

/**
 * Collect targets known at the selected plan's timestamp.
 * targetInfo.json keys are "{targetID}-{watcherID}" — dedupe by targetID,
 * keeping the earliest firstDetected and the freshest state.
 */
function collectTargetsForPlan(scenario, plan) {
  const rawList = scenario?.targetInfo?.targetList || {};
  const planTs = Number(plan?.timestamp) || null;
  const baseTs = scenarioBaseTs(scenario);

  const byId = new Map();
  for (const t of Object.values(rawList)) {
    if (!t || typeof t !== "object") continue;
    const coord = t.coordinate || {};
    if (coord.latitude == null || coord.longitude == null) continue;
    const entry = {
      targetID: t.targetID ?? "?",
      lat: coord.latitude,
      lon: coord.longitude,
      targetType: t.targetType,
      threat: t.threat,
      watcherID: t.watcherID,
      isDestroyed: !!t.isDestroyed,
      firstDetected: Number(t.firstDetected) || null,
      lastUpdated: Number(t.lastUpdated) || 0,
    };
    const prev = byId.get(entry.targetID);
    if (!prev) {
      byId.set(entry.targetID, entry);
    } else {
      const firsts = [prev.firstDetected, entry.firstDetected].filter((v) => v != null);
      const freshest = entry.lastUpdated >= prev.lastUpdated ? entry : prev;
      byId.set(entry.targetID, {
        ...freshest,
        firstDetected: firsts.length ? Math.min(...firsts) : null,
      });
    }
  }

  const all = [...byId.values()];
  const known = all
    .filter((t) => planTs == null || t.firstDetected == null || t.firstDetected <= planTs)
    .sort((a, b) => (a.firstDetected || 0) - (b.firstDetected || 0))
    .map((t) => ({
      ...t,
      // 파괴 시점(lastUpdated 근사)이 plan 시각 이후면 이 시점엔 아직 생존.
      isDestroyed: t.isDestroyed && (planTs == null || !t.lastUpdated || t.lastUpdated <= planTs),
      typeLabel: TARGET_TYPE_LABELS[t.targetType] || "표적",
      watcherLabel: AGENT_LABELS[t.watcherID] || (t.watcherID ? `AC${t.watcherID}` : ""),
      detectedText: formatRelTs(t.firstDetected, baseTs),
    }));
  return { known, total: all.length };
}

// ---- Data normalization ----

function nonNegativeFiniteNumber(value) {
  if (value == null || value === "") return null;
  const number = Number(value);
  return Number.isFinite(number) && number >= 0 ? number : null;
}

function timingResultPlanIDs(record) {
  if (!record || typeof record !== "object") return [];
  const values = Array.isArray(record.resultMissionPlanIDs)
    ? record.resultMissionPlanIDs
    : [];
  const ids = [];
  for (const value of values) {
    const raw = value && typeof value === "object"
      ? (value.missionPlanID ?? value.MissionPlanID)
      : value;
    const id = Number(raw);
    if (Number.isInteger(id) && id > 0 && !ids.includes(id)) ids.push(id);
  }
  return ids;
}

function normalizeReplanTimingData(raw) {
  const sourceRecords = Array.isArray(raw.replanTimingRecords)
    ? raw.replanTimingRecords
    : (Array.isArray(raw.replanTimingHistory?.records)
      ? raw.replanTimingHistory.records
      : []);
  const records = [];
  const seenTimingIDs = new Set();
  for (const row of sourceRecords) {
    if (!row || typeof row !== "object") continue;
    if (nonNegativeFiniteNumber(row.elapsedMs) == null) continue;
    const timingID = String(row.replanTimingId || "").trim();
    if (timingID) {
      if (seenTimingIDs.has(timingID)) continue;
      seenTimingIDs.add(timingID);
    }
    records.push(row);
  }

  const byPlanID = new Map();
  for (const record of records) {
    for (const planID of timingResultPlanIDs(record)) {
      byPlanID.set(String(planID), record);
    }
  }
  const explicitMap = raw.replanTimingByMissionPlanID;
  if (explicitMap && typeof explicitMap === "object") {
    for (const [planID, record] of Object.entries(explicitMap)) {
      if (record && typeof record === "object") byPlanID.set(String(planID), record);
    }
  }
  return { records, byPlanID };
}

function normalizeScenario(raw) {
  const replanTiming = normalizeReplanTimingData(raw);
  const plans = (raw.missionPlans || []).map((mp) => {
    const missionPlan = mp.plan || {};
    const timing = mp.replanTiming
      || replanTiming.byPlanID.get(String(mp.missionPlanID));
    const measuredElapsedMs = nonNegativeFiniteNumber(timing?.elapsedMs);
    return {
      missionPlanID: mp.missionPlanID,
      timestamp: missionPlan.timestamp || mp.timestamp,
      planningTime: measuredElapsedMs ?? missionPlan.planningTime,
      missionPlanPlanningTime: missionPlan.planningTime,
      planningTimeSource: measuredElapsedMs != null
        ? "replan_timing_history"
        : "mission_plan",
      replanTiming: timing || null,
      inputMissionPackageID: missionPlan.inputMissionPackageID
        ?? missionPlan.InputMissionPackageID
        ?? mp.inputMissionPackageID,
      aircraftList: missionPlan.aircraftList || [],
      resolved: mp.resolved || {},
    };
  });

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
    replanTimingRecords: replanTiming.records,
    replanTimingByMissionPlanID: Object.fromEntries(replanTiming.byPlanID),
    flightPaths,
    referenceFeatures: raw.referenceFeatures || null,
    inputAreaFeaturesRaw: raw.missionSectionFeatures || buildInputAreaFeatures(raw.inputMissionPlan),
    inputAreaFeatures15Hz: raw.missionSectionFeatures15Hz || null,
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
    const inventory = typeof entry === "object" ? entry.inventory : null;
    if (inventory) {
      const badge = inventory.status === "ready" ? "FILES" : inventory.status === "partial" ? "PARTIAL" : "EMPTY";
      opt.textContent = `${name} · ${badge} · MP ${inventory.missionPlanCount || 0} / Path ${inventory.flightPathCount || 0} / 0401 ${inventory.playback0401FileCount || 0}`;
      opt.dataset.status = inventory.status || "";
    } else {
      opt.textContent = name;
    }
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
  scenarioLoadRequestId += 1;
  stopLivePolling();
  clearTrackPlaybackUi();
  currentScenario = null;
  currentPlan = null;
  currentScenarioName = null;
  currentVersion = 0;
  coverageMode = "raw";
  document.body.classList.remove("right-panel-open");
  timelineTrack.innerHTML = '<div class="tl-empty">시나리오를 선택하세요</div>';
  detailPanelEl.setAttribute("aria-hidden", "true");
  if (detailCtrl) detailCtrl.hide();
  if (analysisCtrl) analysisCtrl.hide();
  if (dynamicsCtrl) dynamicsCtrl.hide();
  if (mapCtrl) {
    mapCtrl.clearPaths();
    mapCtrl.clearTargets();
    mapCtrl.setMissionFocus(null);
    mapCtrl.loadReferenceGeoJSON(EMPTY_FEATURE_COLLECTION);
    mapCtrl.loadAreaGeoJSON(EMPTY_FEATURE_COLLECTION);
  }
  renderCoveragePanel(null, null);
  if (filterCtrl) filterCtrl.reset();
  if (missionLayerCtrl) {
    missionLayerCtrl.setAutoFollow(true);
    missionLayerCtrl.setScenario(null);
  }
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
  const footprintCount = summary.totalFootprints ?? summary.footprintSamples ?? playback?.coverageFootprintCount ?? playback?.footprintCount ?? 0;
  const covered = Number(summary.coveredPercent || 0);
  const status = coverage.available ? "READY" : (coverage.reason || "NO DATA");
  const rawCoverage = filterCoverageByPackage(currentScenario?.footprintCoverage);
  const adjustedCoverage = filterCoverageByPackage(currentScenario?.footprintCoverage15Hz);
  const rawSummary = rawCoverage?.summary || {};
  const adjustedSummary = adjustedCoverage?.summary || {};
  const hasAdjusted = !!adjustedCoverage?.sections?.length;
  const rawPct = Number(rawSummary.coveredPercent || 0);
  const adjustedPct = Number(adjustedSummary.coveredPercent || 0);
  const modeControls = hasAdjusted ? `
    <div class="coverage-mode" role="group" aria-label="Coverage mode">
      <button class="${coverageMode === "raw" ? "is-active" : ""}" type="button" data-coverage-mode="raw">
        <span>5Hz raw</span><b>${rawPct.toFixed(1)}%</b>
      </button>
      <button class="${coverageMode === "adjusted" ? "is-active" : ""}" type="button" data-coverage-mode="adjusted">
        <span>15Hz adj</span><b>${adjustedPct.toFixed(1)}%</b>
      </button>
    </div>` : "";
  const timing = playback?.sampleTiming || {};
  const hzText = Number.isFinite(Number(timing.medianHz)) ? `${Number(timing.medianHz).toFixed(2)}Hz med` : "";
  const adjustment = coverage.adjustment || {};
  const targetHz = adjustment.targetHz ? `${Number(adjustment.targetHz).toFixed(0)}Hz` : "raw";
  const rows = sections
    .map((section) => {
      const pct = Math.max(0, Math.min(100, Number(section.coveragePercent || 0)));
      const region = section.regionDisplayLabel || section.regionLabel || section.sectionId || "지역 미지정";
      const label = `${region} / Input ${section.inputMissionID ?? "-"}`;
      const type = section.shapeLabel || (section.sectionType === "corridor" ? "LINE" : "AREA");
      const metaParts = [type];
      if (section.missionTypeLabel) metaParts.push(section.missionTypeLabel);
      if (section.widthM) metaParts.push(`${Math.round(section.widthM)}m`);
      if (section.coveredM2 != null && section.areaM2 != null) {
        metaParts.push(`${formatAreaM2(section.coveredM2)} / ${formatAreaM2(section.areaM2)}`);
      }
      return `
        <div class="coverage-row">
          <div class="coverage-row-head">
            <span>${escapeHtml(label)}</span>
            <b>${pct.toFixed(1)}%</b>
          </div>
          <div class="coverage-meta">${escapeHtml(metaParts.join(" / "))}</div>
          <div class="coverage-bar"><span style="width:${pct}%"></span></div>
        </div>`;
    })
    .join("");

  coveragePanelEl.innerHTML = `
    <div class="coverage-head">
      <div>
        <div class="coverage-title">0401 FOOTPRINT</div>
        <div class="coverage-subtitle">${escapeHtml(sourceKind)} / ${fileCount} files / ${messageCount} messages${hzText ? ` / ${escapeHtml(hzText)}` : ""}</div>
      </div>
      <div class="coverage-score">${covered.toFixed(1)}%</div>
    </div>
    ${modeControls}
    <div class="coverage-stats">
      <span>${escapeHtml(status)}</span>
      <span>${escapeHtml(targetHz)}</span>
      <span>${Number(footprintCount).toLocaleString()} footprints</span>
      <span>${sections.length} sections</span>
      <span>${formatAreaM2(summary.coveredM2)} / ${formatAreaM2(summary.areaM2)}</span>
    </div>
    <div class="coverage-list">${rows}</div>`;
  coveragePanelEl.querySelectorAll("[data-coverage-mode]").forEach((btn) => {
    btn.addEventListener("click", () => setCoverageMode(btn.getAttribute("data-coverage-mode")));
  });
  applyCoveragePanelPosition();
  enableCoveragePanelDrag();
  coveragePanelEl.setAttribute("aria-hidden", "false");
}

function formatAreaM2(value) {
  const num = Number(value);
  if (!Number.isFinite(num) || num <= 0) return "-";
  if (num >= 1_000_000) return `${(num / 1_000_000).toFixed(2)} km2`;
  return `${Math.round(num).toLocaleString()} m2`;
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
    if (name !== currentScenarioName) return;
    if (!result || !result.changed) return;

    // Remember current selection
    const selectedPlanId = currentPlan?.missionPlanID;

    currentVersion = result.version;
    currentScenario = normalizeScenario(result.data);

    // Rebuild timeline
    timelineCtrl = buildTimeline(timelineTrack, currentScenario, {
      onPlanSelect: handleTimelinePlanClick,
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
      handlePlanSelect(restorePlan.plan, { preserveFocus: true });
    } else if (lastPlan) {
      timelineCtrl.selectPlan(lastPlan.plan.missionPlanID);
      handlePlanSelect(lastPlan.plan);
    } else {
      currentPlan = null;
      missionLayerCtrl?.setScenario(currentScenario);
      detailCtrl?.hide();
      replanCtrl?.hide();
      if (mapCtrl) {
        mapCtrl.clearPaths();
        mapCtrl.clearTargets();
        mapCtrl.setMissionFocus(null);
      }
    }

    // Refresh 0401 playback and coverage (preserve playback position)
    updateTrackData(currentScenario.playback0401 || currentScenario.tracks);
    if (timelineTimeFocusPlanId != null) {
      setTimelinePlanTimeFocus(timelineTimeFocusPlanId);
    }
    refreshCoverageDisplay({ updateMap: true });
    if (dynamicsBtn) dynamicsBtn.disabled = !hasTrackPayload(currentScenario.playback0401 || currentScenario.tracks);

    setStatus(`"${name}" 갱신됨 (v${currentVersion})`);
  } catch {
    // Silently ignore poll errors
  }
}

// ---- Track playback (항적 재생) ----

const trackBar = document.getElementById("track-bar");
const trackPlayBtn = document.getElementById("track-play");
const trackAllToggle = document.getElementById("track-all-toggle");
const trackSpeedSelect = document.getElementById("track-speed");
const trackSlider = document.getElementById("track-slider");
const trackTimeEl = document.getElementById("track-time");
let trackPlaying = false;
let trackAnimFrame = null;
let trackInfo = null;
let trackAllMode = false;
let trackPlaybackTs = 0;

function clearTrackPlaybackUi() {
  stopTrackPlayback();
  trackInfo = null;
  trackAllMode = false;
  trackPlaybackTs = 0;
  mapCtrl?.clearTracks();
  missionLayerCtrl?.updatePlayback(null);
  if (trackSlider) {
    trackSlider.min = "0";
    trackSlider.max = "0";
    trackSlider.value = "0";
  }
  if (trackTimeEl) trackTimeEl.textContent = "00:00 / 00:00";
  trackAllToggle?.classList.remove("is-active");
  trackBar?.setAttribute("aria-hidden", "true");
}

function initTrackPlayback(tracks) {
  if (!hasTrackPayload(tracks) || !mapCtrl) {
    clearTrackPlaybackUi();
    return;
  }
  stopTrackPlayback();
  setTrackAllMode(false);
  trackInfo = mapCtrl.loadTracks(tracks);
  if (!trackInfo || trackInfo.totalFrames < 2) {
    clearTrackPlaybackUi();
    return;
  }
  trackSlider.max = String(trackInfo.totalFrames - 1);
  trackSlider.value = "0";
  trackPlaybackTs = trackInfo.minTs;
  applyTrackFrame(0, { syncPlan: true });
  trackBar?.setAttribute("aria-hidden", "false");
}

function updateTrackData(tracks) {
  if (!hasTrackPayload(tracks) || !mapCtrl) {
    clearTrackPlaybackUi();
    return;
  }
  const previousTs = trackInfo?.timestamps?.[Number(trackSlider.value)] ?? trackPlaybackTs;
  const wasPlaying = trackPlaying;
  const wasAllMode = trackAllMode;

  trackInfo = mapCtrl.loadTracks(tracks);
  if (!trackInfo || trackInfo.totalFrames < 2) {
    clearTrackPlaybackUi();
    return;
  }

  trackSlider.max = String(trackInfo.totalFrames - 1);
  const restoredFrame = frameAtOrBefore(Math.max(trackInfo.minTs, Math.min(previousTs, trackInfo.maxTs)));
  applyTrackFrame(restoredFrame, { syncPlan: true });
  setTrackAllMode(wasAllMode);
  trackBar?.setAttribute("aria-hidden", "false");

  if (wasPlaying) setPlayingButtonState(true);
}

function updateTrackTime(frame) {
  if (!trackInfo) return;
  if (trackAllMode) {
    if (trackTimeEl) trackTimeEl.textContent = "ALL";
    return;
  }
  const ts = trackInfo.timestamps?.[frame] ?? trackInfo.minTs;
  const elapsed = Math.max(0, Math.round((ts - trackInfo.minTs) / 1000));
  const total = Math.max(0, Math.round((trackInfo.maxTs - trackInfo.minTs) / 1000));
  const fmt = (s) => `${String(Math.floor(s / 60)).padStart(2, "0")}:${String(s % 60).padStart(2, "0")}`;
  if (trackTimeEl) trackTimeEl.textContent = `${fmt(elapsed)} / ${fmt(total)}`;
}

function applyTrackFrame(frame, options = {}) {
  if (!trackInfo || !mapCtrl) return null;
  const clamped = Math.max(0, Math.min(Number(frame) || 0, trackInfo.totalFrames - 1));
  const timestamp = trackInfo.timestamps?.[clamped] ?? trackInfo.minTs;
  if (!options.preserveClock) trackPlaybackTs = timestamp;
  trackSlider.value = String(clamped);
  if (options.syncPlan !== false) syncPlanToPlayback(timestamp);
  const snapshot = mapCtrl.setTrackFrame(clamped);
  missionLayerCtrl?.updatePlayback(snapshot);
  updateTrackTime(clamped);
  return snapshot;
}

function frameAtOrBefore(timestamp) {
  const values = trackInfo?.timestamps || [];
  let lo = 0, hi = values.length - 1, answer = 0;
  while (lo <= hi) {
    const mid = (lo + hi) >> 1;
    if (values[mid] <= timestamp) { answer = mid; lo = mid + 1; }
    else hi = mid - 1;
  }
  return answer;
}

function syncPlanToPlayback(timestamp) {
  if (!currentScenario || !missionLayerCtrl?.isAutoFollow()) return;
  const allPlans = currentScenario.plans || [];
  const applied = allPlans.filter((plan) => plan.isSelected);
  const candidates = (applied.length ? applied : allPlans)
    .filter((plan) => Number.isFinite(Number(plan.timestamp)))
    .sort((a, b) => Number(a.timestamp) - Number(b.timestamp));
  if (!candidates.length) return;
  let selected = candidates[0];
  for (const candidate of candidates) {
    if (Number(candidate.timestamp) <= timestamp) selected = candidate;
    else break;
  }
  if (String(currentPlan?.missionPlanID) === String(selected.missionPlanID)) return;
  timelineCtrl?.selectPlan(selected.missionPlanID);
  handlePlanSelect(selected, { fromPlayback: true });
}

function setPlayingButtonState(playing) {
  trackPlaying = !!playing;
  trackPlayBtn?.classList.toggle("is-playing", trackPlaying);
  if (trackPlayBtn) trackPlayBtn.innerHTML = trackPlaying ? "&#9646;&#9646;" : "&#9654;";
}

function stopTrackPlayback() {
  setPlayingButtonState(false);
  if (trackAnimFrame) { cancelAnimationFrame(trackAnimFrame); trackAnimFrame = null; }
}

function setTrackAllMode(enabled, options = {}) {
  const next = !!enabled;
  if (next && options.stopPlayback) stopTrackPlayback();
  trackAllMode = next;
  trackAllToggle?.classList.toggle("is-active", trackAllMode);
  if (mapCtrl?.setTrackAll) mapCtrl.setTrackAll(trackAllMode);
  if (!trackAllMode && trackInfo) applyTrackFrame(Number(trackSlider?.value || 0), { syncPlan: false });
  updateTrackTime(Number(trackSlider?.value || 0));
}

trackPlayBtn?.addEventListener("click", () => {
  if (trackPlaying) { stopTrackPlayback(); return; }
  if (!trackInfo || trackInfo.totalFrames < 2) return;
  if (trackAllMode) setTrackAllMode(false);
  if (Number(trackSlider.value) >= trackInfo.totalFrames - 1) applyTrackFrame(0, { syncPlan: true });
  setPlayingButtonState(true);
  let lastTime = performance.now();
  const step = (now) => {
    if (!trackPlaying) return;
    const dt = now - lastTime;
    lastTime = now;
    const speed = Math.max(0.1, Number(trackSpeedSelect?.value || 1));
    trackPlaybackTs += dt * speed;
    if (trackPlaybackTs >= trackInfo.maxTs) {
      applyTrackFrame(trackInfo.totalFrames - 1, { syncPlan: true });
      stopTrackPlayback();
      return;
    }
    const frame = frameAtOrBefore(trackPlaybackTs);
    if (frame !== Number(trackSlider.value)) {
      applyTrackFrame(frame, { syncPlan: true, preserveClock: true });
    }
    trackAnimFrame = requestAnimationFrame(step);
  };
  trackAnimFrame = requestAnimationFrame(step);
});

trackSlider?.addEventListener("input", () => {
  stopTrackPlayback();
  const frame = Number(trackSlider.value);
  if (trackAllMode) setTrackAllMode(false);
  applyTrackFrame(frame, { syncPlan: true });
});

trackAllToggle?.addEventListener("click", () => {
  if (!mapCtrl || !trackInfo) return;
  setTrackAllMode(!trackAllMode, { stopPlayback: !trackAllMode });
});

function hasTrackPayload(payload) {
  const tracks = payload?.tracks || payload;
  return !!tracks && Object.keys(tracks).length > 0;
}
