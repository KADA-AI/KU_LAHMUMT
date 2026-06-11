import { getConfig } from "./js/config.js";
import { palette } from "./js/palette.js";
import { buildStyle } from "./js/map_style.js";
import { createBuildingController } from "./js/buildings.js";
import { setupTerrainToggle } from "./js/terrain.js";
import { ZoomResetControl } from "./js/zoom_reset_control.js";
import { initPlaybackControls } from "./js/controls_playback.js";
import { initAgentPanel } from "./js/controls_agent.js";
import { initSidePanel } from "./js/controls_sidepanel.js";
import { initMissionPanel } from "./js/controls_mission.js";
import { initScenarioPanel } from "./js/controls_scenario.js";
import { init0401Panel } from "./js/panel_0401.js";
import { initRightSidePanel } from "./js/controls_sidepanel_right.js";
import { initLeftSidePanel } from "./js/controls_sidepanel_left.js";
import { initIntegrationPanel } from "./js/integration_panel.js";
import { initDynamicsCalibrationPanel } from "./js/dynamics_calibration_panel.js";
import { initMissionOptionPopup } from "./js/mission_option_popup.js";
import { initMissionRecommendPopup } from "./js/mission_recommend_popup.js";
import { initReplanNoticePopup } from "./js/replan_notice_popup.js";
import { initMissionPaths } from "./js/mission_paths.js";
import { initVehicleMarkers } from "./js/vehicle_markers.js";
import { initTargetMarkers } from "./js/target_markers.js";
import { initProjectileMarkers } from "./js/projectile_markers.js";
import { initImpactEffects } from "./js/impact_effects.js";
import { initRemainingAreas } from "./js/remaining_areas.js";
import { getAgentCoordinate, getUiState, updateUiField } from "./js/agent_store.js";
import { logStatus } from "./js/status_log.js";
import { initSimClient } from "./js/sim_client.js";

(() => {
  const buildingToggle = document.getElementById("toggle-buildings");
  const sweepLineToggle = document.getElementById("toggle-sweep-lines");

  const setStatus = (text) => {
    logStatus(text, { key: "app-status", ttlMs: 4500 });
  };

  const config = getConfig(document.body);

  if (!window.maplibregl) {
    setStatus("MapLibre missing.");
    return;
  }

  const map = new maplibregl.Map({
    container: "map",
    style: buildStyle(config, palette),
    center: config.center,
    zoom: config.startZoom,
    minZoom: config.minZoom,
    maxZoom: config.maxZoom + 4,
    maxPitch: 85,
    attributionControl: false,
  });

  initMissionRecommendPopup();

  let initialView = null;

  const captureInitialView = () => {
    const center = map.getCenter();
    initialView = {
      center: [center.lng, center.lat],
      zoom: map.getZoom(),
      bearing: map.getBearing(),
      pitch: map.getPitch(),
    };
  };

  const resetView = () => {
    const positions =
      typeof window.getAgentPositions === "function" ? window.getAgentPositions() : null;
    if (positions && typeof positions === "object") {
      const entries = Object.entries(positions).filter(([, pos]) => {
        if (!pos) {
          return false;
        }
        if (!Number.isFinite(pos.lat) || !Number.isFinite(pos.lon)) {
          return false;
        }
        if (pos.alive === false) {
          return false;
        }
        return true;
      });
      if (entries.length) {
        const uavLabels = entries
          .filter(([label]) => String(label).toUpperCase().startsWith("UAV"))
          .map(([label]) => label);
        const labels = uavLabels.length ? uavLabels : entries.map(([label]) => label);
        if (labels.length === 1 && typeof window.flyToAgent === "function") {
          window.flyToAgent(labels[0]);
          return;
        }
        let minLon = Infinity;
        let maxLon = -Infinity;
        let minLat = Infinity;
        let maxLat = -Infinity;
        labels.forEach((label) => {
          const pos = positions[label];
          if (!pos) {
            return;
          }
          minLon = Math.min(minLon, pos.lon);
          maxLon = Math.max(maxLon, pos.lon);
          minLat = Math.min(minLat, pos.lat);
          maxLat = Math.max(maxLat, pos.lat);
        });
        if (
          Number.isFinite(minLon) &&
          Number.isFinite(maxLon) &&
          Number.isFinite(minLat) &&
          Number.isFinite(maxLat)
        ) {
          let minLonAdj = minLon;
          let maxLonAdj = maxLon;
          let minLatAdj = minLat;
          let maxLatAdj = maxLat;
          const spanLon = Math.abs(maxLon - minLon);
          const spanLat = Math.abs(maxLat - minLat);
          if (spanLon < 1e-5 && spanLat < 1e-5) {
            const centerLat = (minLat + maxLat) * 0.5;
            const deltaLat = 0.02;
            const cosLat = Math.cos((centerLat * Math.PI) / 180) || 1e-6;
            const deltaLon = deltaLat / cosLat;
            minLonAdj -= deltaLon;
            maxLonAdj += deltaLon;
            minLatAdj -= deltaLat;
            maxLatAdj += deltaLat;
          }
          map.fitBounds(
            [
              [minLonAdj, minLatAdj],
              [maxLonAdj, maxLatAdj],
            ],
            { padding: 140, duration: 600, bearing: 0, pitch: 0, maxZoom: 13.2 },
          );
          return;
        }
      }
    }

    if (initialView) {
      map.easeTo({
        center: initialView.center,
        zoom: initialView.zoom,
        bearing: initialView.bearing,
        pitch: initialView.pitch,
        duration: 600,
      });
      return;
    }
    if (config.bounds) {
      map.fitBounds(config.bounds, { padding: 20, duration: 600, bearing: 0, pitch: 0 });
    } else {
      map.easeTo({
        center: config.center,
        zoom: config.startZoom,
        bearing: 0,
        pitch: 0,
        duration: 600,
      });
    }
  };
  map.addControl(new ZoomResetControl(resetView), "bottom-right");

  const buildingController = createBuildingController(map, buildingToggle, setStatus);

  map.once("load", () => {
    buildingController.ensureLayer();
    buildingController.applyPending();
    if (typeof map.setSky === "function") {
      map.setSky({
        "sky-color": palette.sky,
        "sky-horizon-blend": 0.6,
        "horizon-color": palette.skyHorizon,
        "horizon-fog-blend": 0.7,
        "fog-color": "#d7e6f7",
        "fog-ground-blend": 0.8,
        "atmosphere-blend": ["interpolate", ["linear"], ["zoom"], 0, 1, 8, 0.6, 12, 0],
      });
    }
  });

  if (config.bounds) {
    map.fitBounds(config.bounds, { padding: 20, duration: 0, bearing: 0, pitch: 0 });
  } else {
    map.jumpTo({ center: config.center, zoom: config.startZoom, bearing: 0, pitch: 0 });
  }

  setupTerrainToggle(map, config);
  const simClient = initSimClient();
  simClient.startPolling();
  window.simClient = simClient;
  simClient.subscribe((state) => {
    const vehicles = state?.vehicles || {};
    Object.entries(vehicles).forEach(([label, entry]) => {
      if (!entry || entry.alive !== false) {
        return;
      }
      const ui = getUiState(label);
      if (ui && Number(ui.health) !== 2) {
        updateUiField(label, "health", 2);
      }
    });
  });
  initPlaybackControls();
  initAgentPanel();
  initSidePanel();
  initMissionPanel();
  initScenarioPanel(map);
  init0401Panel();
  initRightSidePanel();
  initIntegrationPanel();
  initDynamicsCalibrationPanel();
  initMissionOptionPopup();
  initReplanNoticePopup();
  const missionPaths = initMissionPaths(map);
  const remainingAreas = initRemainingAreas(map, { intervalMs: 1000 });
  remainingAreas.start();
  window.remainingAreaSnapshotLayer = remainingAreas;
  window.missionPathLoader = missionPaths.loadFromResponse;
  window.loadMissionPathsFromServer = missionPaths.loadFromServer;
  window.setSelectedAgentPath = missionPaths.setSelectedAgent;
  window.setMissionCurrentWaypoints = missionPaths.setCurrentWaypoints;
  window.setSweepLineVisibility = missionPaths.setSweepLinesVisible;
  if (sweepLineToggle && typeof missionPaths.setSweepLinesVisible === "function") {
    const setSweepToggleState = (visible) => {
      sweepLineToggle.classList.toggle("is-active", visible);
      sweepLineToggle.setAttribute("aria-pressed", visible ? "true" : "false");
      missionPaths.setSweepLinesVisible(visible);
    };
    setSweepToggleState(true);
    sweepLineToggle.addEventListener("click", () => {
      const nextVisible = !sweepLineToggle.classList.contains("is-active");
      setSweepToggleState(nextVisible);
      setStatus(nextVisible ? "Sweep lines shown." : "Sweep lines hidden.");
    });
  }
  const vehicleMarkers = initVehicleMarkers(map);
  window.missionVehicleLoader = vehicleMarkers.loadFromReference;
  window.getAgentPosition = vehicleMarkers.getPosition;
  window.getAgentPositions = vehicleMarkers.getPositions;
  const targetMarkers = initTargetMarkers(map);
  window.missionTargetLoader = targetMarkers.loadFromReference;
  initLeftSidePanel({
    map,
    getFilmingViews: vehicleMarkers.getFilmingViews,
    subscribeFilmingViews: vehicleMarkers.subscribeFilmingViews,
    getTargets: targetMarkers.getTargets,
    subscribeTargets: targetMarkers.subscribeTargets,
  });
  const projectileMarkers = initProjectileMarkers(map);
  window.missionProjectileLoader = projectileMarkers.loadFromReference;
  const impactEffects = initImpactEffects(map);
  window.missionEffectLoader = impactEffects.loadFromReference;
  window.clearMissionData = () => {
    if (typeof window.missionPathLoader === "function") {
      window.missionPathLoader({
        ok: true,
        features: [],
        agents: {},
        count: 0,
        flightPaths: [],
      });
    }
    if (typeof window.missionVehicleLoader === "function") {
      window.missionVehicleLoader({ ok: true, vehicles: {} });
    }
    if (typeof window.missionTargetLoader === "function") {
      window.missionTargetLoader({ ok: true, targets: [] });
    }
    if (typeof window.missionProjectileLoader === "function") {
      window.missionProjectileLoader({ ok: true, projectiles: [] });
    }
    if (typeof window.missionEffectLoader === "function") {
      window.missionEffectLoader({ ok: true, effects: [] });
    }
    if (typeof window.setSelectedAgentPath === "function") {
      window.setSelectedAgentPath(null);
    }
    if (typeof window.setMissionPlanReady === "function") {
      window.setMissionPlanReady(false);
    }
  };
  window.flyToAgent = (label) => {
    if (!label || !map) {
      return;
    }
    const pos =
      (typeof window.getAgentPosition === "function"
        ? window.getAgentPosition(label)
        : null) || getAgentCoordinate(label);
    if (!pos || !Number.isFinite(pos.lat) || !Number.isFinite(pos.lon)) {
      return;
    }
    const altitude =
      Number.isFinite(pos.alt) ? pos.alt : Number.isFinite(pos.altitude) ? pos.altitude : 0;
    const zoom = Math.max(14.8, 16.6 - Math.log2(1 + altitude / 400));
    const pitch = Math.max(45, 62 - Math.log2(1 + altitude / 300) * 6);
    const offsetY = Math.min(220, 80 + altitude * 0.04);
    map.easeTo({
      center: [pos.lon, pos.lat],
      zoom,
      pitch,
      bearing: 0,
      offset: [0, offsetY],
      duration: 900,
      easing: (t) => t * (2 - t),
    });
  };

  map.once("idle", () => {
    captureInitialView();
  });

  const monitoringToggle = document.getElementById("monitoring-toggle");
  const monitoringMeta = document.getElementById("monitoring-meta");
  const setMonitoringMode = async (enabled) => {
    const next = Boolean(enabled);
    if (next && typeof simClient.ensureIntegrationReady === "function") {
      const result = await simClient.ensureIntegrationReady();
      if (!result?.ok) {
        setStatus(`Monitoring link pending: ${result?.error || "nFusion offline"}`);
      }
    }
    document.body.classList.toggle("is-monitoring", next);
    if (monitoringMeta) {
      monitoringMeta.textContent = next ? "0401 RX @ 5Hz" : "SIM playback";
    }
    if (typeof simClient.setMode === "function") {
      simClient.setMode(next ? "monitor" : "sim");
    }
    setStatus(next ? "Monitoring mode enabled." : "Monitoring mode disabled.");
  };
  if (monitoringToggle) {
    monitoringToggle.addEventListener("change", (event) => {
      void setMonitoringMode(event.target.checked);
    });
  }
  window.setMonitoringMode = (enabled) => {
    void setMonitoringMode(enabled);
  };

  const simTimeEl = document.getElementById("sim-time");
  if (simTimeEl && typeof simClient.subscribe === "function") {
    let lastSec = null;
    let lastMonitorTs = null;
    const EPOCH_2000 = Date.UTC(2000, 0, 1, 0, 0, 0, 0);
    const pad = (value) => String(value).padStart(2, "0");
    const formatTime = (secs) => {
      const s = Math.max(0, Math.floor(secs));
      const h = Math.floor(s / 3600);
      const m = Math.floor((s % 3600) / 60);
      const ss = s % 60;
      return `T+${pad(h)}:${pad(m)}:${pad(ss)}`;
    };
    const formatMonitorTime = (timestamp) => {
      const date = new Date(EPOCH_2000 + timestamp);
      if (Number.isNaN(date.getTime())) {
        return "LIVE 0401";
      }
      return `LIVE ${pad(date.getHours())}:${pad(date.getMinutes())}:${pad(date.getSeconds())}`;
    };
    simClient.subscribe((state) => {
      if (typeof simClient.getMode === "function" && simClient.getMode() === "monitor") {
        const timestamp = Number(state?.timestamp);
        if (!Number.isFinite(timestamp) || timestamp === lastMonitorTs) {
          return;
        }
        lastMonitorTs = timestamp;
        lastSec = null;
        simTimeEl.textContent = formatMonitorTime(timestamp);
        return;
      }
      const simTime = Number(state?.simTime);
      if (!Number.isFinite(simTime)) {
        return;
      }
      const sec = Math.floor(simTime);
      if (sec === lastSec) {
        return;
      }
      lastSec = sec;
      lastMonitorTs = null;
      simTimeEl.textContent = formatTime(sec);
    });
  }
})();
