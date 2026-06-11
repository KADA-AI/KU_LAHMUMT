import { logStatus } from "./status_log.js";

const ENDPOINT = "/api/sim/remaining_areas";
const SOURCE_ID = "remaining-area-snapshot";
const FILL_LAYER_ID = "remaining-area-snapshot-fill";
const LINE_LAYER_ID = "remaining-area-snapshot-line";
const DEFAULT_INTERVAL_MS = 1000;

const AGENT_COLOR_VARS = {
  LAH1: "--lah1",
  LAH2: "--lah2",
  LAH3: "--lah3",
  UAV1: "--uav1",
  UAV2: "--uav2",
  UAV3: "--uav3",
};

const fallbackColor = "#3ee6cf";

const getAgentColors = () => {
  const style = getComputedStyle(document.documentElement);
  const colors = {};
  Object.entries(AGENT_COLOR_VARS).forEach(([agent, varName]) => {
    colors[agent] = style.getPropertyValue(varName).trim() || fallbackColor;
  });
  return colors;
};

const asNumber = (value, fallback = null) => {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
};

const formatArea = (value) => {
  const area = asNumber(value, null);
  if (area === null) {
    return "-";
  }
  if (area >= 1000000) {
    return `${(area / 1000000).toFixed(2).replace(/\.?0+$/, "")} km²`;
  }
  return `${Math.round(area).toLocaleString()} m²`;
};

const featureCollectionFromPayload = (payload) => {
  if (payload?.featureCollection?.type === "FeatureCollection") {
    return payload.featureCollection;
  }
  if (Array.isArray(payload?.features)) {
    return { type: "FeatureCollection", features: payload.features };
  }
  return { type: "FeatureCollection", features: [] };
};

const decorateCollection = (payload, colors) => {
  const collection = featureCollectionFromPayload(payload);
  const features = Array.isArray(collection.features) ? collection.features : [];
  return {
    type: "FeatureCollection",
    features: features.map((feature, index) => {
      const props = { ...(feature?.properties || {}) };
      const agent = String(props.agent || "").toUpperCase();
      const color = colors[agent] || fallbackColor;
      const done = Number(props.isDone) === 1;
      props.color = color;
      props.fillOpacity = done ? 0.04 : 0.2;
      props.lineOpacity = done ? 0.22 : 0.86;
      props.lineWidth = done ? 1.1 : 2.4;
      props.remainingAreaLabel = formatArea(props.remainingAreaM2);
      return {
        type: "Feature",
        id: feature?.id ?? index + 1,
        geometry: feature?.geometry || null,
        properties: props,
      };
    }),
  };
};

export const initRemainingAreas = (map, options = {}) => {
  const intervalMs = Math.max(500, Number(options.intervalMs || DEFAULT_INTERVAL_MS));
  let timer = null;
  let mapReady = false;
  let lastSignature = "";
  let lastWarnAt = 0;
  let lastPayload = null;
  const colors = getAgentColors();

  const ensureLayers = () => {
    if (!map || !mapReady) {
      return;
    }
    if (!map.getSource(SOURCE_ID)) {
      map.addSource(SOURCE_ID, {
        type: "geojson",
        data: { type: "FeatureCollection", features: [] },
      });
    }
    if (!map.getLayer(FILL_LAYER_ID)) {
      map.addLayer({
        id: FILL_LAYER_ID,
        type: "fill",
        source: SOURCE_ID,
        paint: {
          "fill-color": ["get", "color"],
          "fill-opacity": ["get", "fillOpacity"],
        },
      });
    }
    if (!map.getLayer(LINE_LAYER_ID)) {
      map.addLayer({
        id: LINE_LAYER_ID,
        type: "line",
        source: SOURCE_ID,
        layout: {
          "line-join": "round",
          "line-cap": "round",
        },
        paint: {
          "line-color": ["get", "color"],
          "line-opacity": ["get", "lineOpacity"],
          "line-width": ["get", "lineWidth"],
          "line-dasharray": [1.6, 0.7],
        },
      });
    }
  };

  const setCollection = (payload) => {
    ensureLayers();
    const source = map?.getSource(SOURCE_ID);
    if (!source) {
      return;
    }
    const collection = decorateCollection(payload, colors);
    source.setData(collection);
    lastPayload = payload || null;
  };

  const clear = () => {
    lastSignature = "";
    setCollection({ featureCollection: { type: "FeatureCollection", features: [] } });
  };

  const refresh = async () => {
    if (typeof document !== "undefined" && document.hidden) {
      return;
    }
    try {
      const response = await fetch(ENDPOINT, { method: "GET", cache: "no-store" });
      const payload = await response.json();
      if (!payload?.ok || payload.available === false) {
        clear();
        return;
      }
      const signature = [
        payload.missionPlanID ?? "-",
        payload.dataRevision ?? "-",
        payload.snapshotMtimeMs ?? "-",
        payload.count ?? 0,
      ].join(":");
      if (signature === lastSignature) {
        return;
      }
      lastSignature = signature;
      setCollection(payload);
    } catch (err) {
      const now = Date.now();
      if (now - lastWarnAt >= 5000) {
        logStatus("Remaining area fetch failed", { level: "warn", ttlMs: 2500 });
        lastWarnAt = now;
      }
    }
  };

  const start = () => {
    if (timer) {
      return;
    }
    refresh();
    timer = setInterval(refresh, intervalMs);
  };

  const stop = () => {
    if (!timer) {
      return;
    }
    clearInterval(timer);
    timer = null;
  };

  map.on("load", () => {
    mapReady = true;
    ensureLayers();
    refresh();
  });
  if (map.loaded && map.loaded()) {
    mapReady = true;
    ensureLayers();
  }

  return {
    start,
    stop,
    refresh,
    clear,
    getLastPayload: () => lastPayload,
  };
};
