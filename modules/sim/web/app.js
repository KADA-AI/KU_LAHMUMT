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
import { initIntegrationPanel } from "./js/integration_panel.js";
import { initMissionPaths } from "./js/mission_paths.js";
import { initVehicleMarkers } from "./js/vehicle_markers.js";
import { getAgentCoordinate } from "./js/agent_store.js";
import { logStatus } from "./js/status_log.js";
import { initSimClient } from "./js/sim_client.js";

(() => {
  const buildingToggle = document.getElementById("toggle-buildings");

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
  initPlaybackControls();
  initAgentPanel();
  initSidePanel();
  initMissionPanel();
  initScenarioPanel(map);
  init0401Panel();
  initRightSidePanel();
  initIntegrationPanel();
  const missionPaths = initMissionPaths(map);
  window.missionPathLoader = missionPaths.loadFromResponse;
  window.loadMissionPathsFromServer = missionPaths.loadFromServer;
  window.setSelectedAgentPath = missionPaths.setSelectedAgent;
  const vehicleMarkers = initVehicleMarkers(map);
  window.missionVehicleLoader = vehicleMarkers.loadFromReference;
  window.getAgentPosition = vehicleMarkers.getPosition;
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
})();
