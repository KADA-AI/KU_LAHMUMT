import { logStatus } from "./status_log.js";

const ENDPOINT = "/api/sim/remaining_areas";
const SOURCE_ID = "remaining-area-snapshot";
const FILL_LAYER_ID = "remaining-area-snapshot-fill";
const LINE_LAYER_ID = "remaining-area-snapshot-line";
const FORWARD_LINE_LAYER_ID = "remaining-area-snapshot-forward-line";
const REVERSE_LINE_LAYER_ID = "remaining-area-snapshot-reverse-line";
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
const COVERAGE_PASS_COLORS = {
  forward: "#48ddff",
  reverse: "#ffb34d",
};
const COVERAGE_DEPTH_STYLES = {
  0: { color: "#ff5b68", fillOpacity: 0.34, lineOpacity: 0.96, lineWidth: 2.6, label: "0/2 · NEED 2" },
  1: { color: "#ffd166", fillOpacity: 0.27, lineOpacity: 0.9, lineWidth: 2.2, label: "1/2 · NEED 1" },
  2: { color: "#55d6a5", fillOpacity: 0.1, lineOpacity: 0.5, lineWidth: 1.3, label: "2/2 · COMPLETE" },
};

const normalizeCoveragePass = (value) => {
  const normalized = String(value || "").trim().toLowerCase();
  return Object.prototype.hasOwnProperty.call(COVERAGE_PASS_COLORS, normalized)
    ? normalized
    : null;
};

const normalizeCoverageDepth = (value) => {
  const parsed = Number(value);
  return Number.isInteger(parsed) && Object.prototype.hasOwnProperty.call(COVERAGE_DEPTH_STYLES, parsed)
    ? parsed
    : null;
};

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

export const decorateRemainingAreaCollection = (payload, colors) => {
  const collection = featureCollectionFromPayload(payload);
  const features = Array.isArray(collection.features) ? collection.features : [];
  return {
    type: "FeatureCollection",
    features: features.map((feature, index) => {
      const props = { ...(feature?.properties || {}) };
      const agent = String(props.agent || "").toUpperCase();
      const color = colors[agent] || fallbackColor;
      const done = Number(props.isDone) === 1;
      const isCenterline = props.geometrySource === "lineRemainingCenterline";
      const coveragePass = normalizeCoveragePass(props.coveragePass);
      const coverageDepth = normalizeCoverageDepth(props.coverageDepth);
      const depthStyle = coverageDepth === null ? null : COVERAGE_DEPTH_STYLES[coverageDepth];
      const passStatus = String(props.coveragePassStatus || (done ? "completed" : "active"));
      props.coveragePass = coveragePass;
      props.coverageDepth = coverageDepth;
      props.color = depthStyle?.color || (coveragePass ? COVERAGE_PASS_COLORS[coveragePass] : color);
      props.fillOpacity = depthStyle
        ? depthStyle.fillOpacity
        : coveragePass
          ? props.visualizationRole === "coveragePassAttribution" ? 0 : passStatus === "completed" ? 0.035 : passStatus === "planned" ? 0.12 : 0.24
        : done ? 0.04 : 0.2;
      props.lineOpacity = depthStyle
        ? depthStyle.lineOpacity
        : coveragePass
          ? passStatus === "completed" ? 0.22 : passStatus === "planned" ? 0.52 : 0.96
        : done ? 0.22 : isCenterline ? 0.65 : 0.86;
      props.lineWidth = depthStyle
        ? depthStyle.lineWidth
        : coveragePass
          ? passStatus === "active" ? 3.0 : 2.0
        : done ? 1.1 : isCenterline ? 1.4 : 2.4;
      props.coverageDepthLabel = depthStyle?.label || props.coverageDepthLabel || "";
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
  let missionPlanId = null;
  let requestGeneration = 0;
  let layersVisible = true;
  let popup = null;
  let interactionsAttached = false;
  const colors = getAgentColors();

  const applyVisibility = () => {
    if (!map || !mapReady) {
      return;
    }
    const value = layersVisible ? "visible" : "none";
    for (const layerId of [
      FILL_LAYER_ID,
      LINE_LAYER_ID,
      FORWARD_LINE_LAYER_ID,
      REVERSE_LINE_LAYER_ID,
    ]) {
      if (map.getLayer(layerId)) {
        map.setLayoutProperty(layerId, "visibility", value);
      }
    }
  };

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
        filter: ["!=", ["get", "visualizationRole"], "coveragePassAttribution"],
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
        filter: [
          "all",
          ["!=", ["get", "coveragePass"], "forward"],
          ["!=", ["get", "coveragePass"], "reverse"],
        ],
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
    if (!map.getLayer(FORWARD_LINE_LAYER_ID)) {
      map.addLayer({
        id: FORWARD_LINE_LAYER_ID,
        type: "line",
        source: SOURCE_ID,
        filter: ["==", ["get", "coveragePass"], "forward"],
        layout: { "line-join": "round", "line-cap": "round" },
        paint: {
          "line-color": ["get", "color"],
          "line-opacity": ["get", "lineOpacity"],
          "line-width": ["get", "lineWidth"],
        },
      });
    }
    if (!map.getLayer(REVERSE_LINE_LAYER_ID)) {
      map.addLayer({
        id: REVERSE_LINE_LAYER_ID,
        type: "line",
        source: SOURCE_ID,
        filter: ["==", ["get", "coveragePass"], "reverse"],
        layout: { "line-join": "round", "line-cap": "round" },
        paint: {
          "line-color": ["get", "color"],
          "line-opacity": ["get", "lineOpacity"],
          "line-width": ["get", "lineWidth"],
          "line-dasharray": [1.5, 1.2],
        },
      });
    }
    if (!interactionsAttached && map.getLayer(FILL_LAYER_ID)) {
      interactionsAttached = true;
      map.on("mouseenter", FILL_LAYER_ID, () => {
        map.getCanvas().style.cursor = "pointer";
      });
      map.on("mouseleave", FILL_LAYER_ID, () => {
        map.getCanvas().style.cursor = "";
      });
      map.on("click", FILL_LAYER_ID, (event) => {
        const feature = event.features && event.features[0];
        if (!feature) {
          return;
        }
        const props = feature.properties || {};
        const coveragePass = normalizeCoveragePass(props.coveragePass);
        const coverageDepth = normalizeCoverageDepth(props.coverageDepth);
        const passLabel = coveragePass === "forward"
          ? "OUTBOUND (갈 때)"
          : coveragePass === "reverse" ? "RETURN (올 때)" : null;
        const html = `
          <div style="font-size:12px;font-weight:700;margin-bottom:4px;">${props.agent || "REMAINING AREA"}</div>
          ${coverageDepth !== null ? `<div style="font-size:11px;font-weight:700;color:${COVERAGE_DEPTH_STYLES[coverageDepth].color};">Capture depth ${COVERAGE_DEPTH_STYLES[coverageDepth].label}</div>` : ""}
          ${coverageDepth !== null ? `<div style="font-size:11px;color:#333;">Remaining captures ${props.remainingCaptureCount ?? (2 - coverageDepth)}</div>` : ""}
          ${passLabel ? `<div style="font-size:11px;color:#333;">Coverage ${passLabel}</div>` : ""}
          ${props.activeAgents ? `<div style="font-size:11px;color:#333;">Active aircraft ${props.activeAgents}</div>` : ""}
          ${props.activeCoveragePasses ? `<div style="font-size:11px;color:#333;">Attribution ${props.activeCoveragePasses}</div>` : ""}
          <div style="font-size:11px;color:#333;">Status ${props.coveragePassStatus || (Number(props.isDone) === 1 ? "completed" : "active")}</div>
          <div style="font-size:11px;color:#333;">Progress ${props.coveragePassProgress ?? props.coveragePercent ?? "-"}%</div>
          <div style="font-size:11px;color:#333;">Remaining ${props.remainingAreaLabel || "-"}</div>
          <div style="font-size:11px;color:#333;">Input mission ${props.inputMissionID ?? "-"}</div>
        `;
        if (!popup) {
          popup = new maplibregl.Popup({ closeButton: true, closeOnClick: true });
        }
        popup.setLngLat(event.lngLat).setHTML(html).addTo(map);
      });
    }
    applyVisibility();
  };

  const updatePassProgressLegend = (payload) => {
    const summaries = Array.isArray(payload?.coveragePassSummaries)
      ? payload.coveragePassSummaries
      : [];
    if (!summaries.length) {
      const passLegend = document.getElementById("mission-area-pass-legend");
      if (passLegend) {
        passLegend.dataset.remainingPassAvailable = "0";
      }
      document.querySelectorAll(".mission-area-pass-progress").forEach((entry) => {
        entry.textContent = "";
      });
      return;
    }
    const passLegend = document.getElementById("mission-area-pass-legend");
    if (passLegend) {
      passLegend.dataset.remainingPassAvailable = "1";
      passLegend.hidden = false;
      const title = document.getElementById("mission-area-pass-title");
      if (title) {
        title.textContent = "PATH ATTRIBUTION · NOT A REQUIREMENT";
        title.title = "OUT/RETURN identify where observations came from; spatial depth decides completion.";
      }
      const outerLegend = passLegend.closest("#mission-legend");
      if (outerLegend) {
        outerLegend.classList.add("is-visible");
        outerLegend.setAttribute("aria-hidden", "false");
      }
    }
    ["forward", "reverse"].forEach((pass) => {
      const rows = summaries.filter(
        (row) => normalizeCoveragePass(row?.coveragePass) === pass,
      );
      const item = document.querySelector(
        `.mission-area-pass-item[data-coverage-pass="${pass}"]`,
      );
      const progressEl = item?.querySelector(".mission-area-pass-progress");
      if (!progressEl || !rows.length) {
        if (progressEl) {
          progressEl.textContent = "";
        }
        if (item) {
          item.classList.remove("is-active", "is-completed");
          const statusEl = item.querySelector(".mission-area-pass-status");
          if (statusEl) {
            statusEl.textContent = "PLANNED";
          }
        }
        return;
      }
      const planned = rows.reduce((sum, row) => sum + Math.max(0, asNumber(row?.plannedAreaM2, 0)), 0);
      const covered = rows.reduce((sum, row) => sum + Math.max(0, asNumber(row?.coveredAreaM2, 0)), 0);
      const progress = planned > 0
        ? Math.max(0, Math.min(100, Math.round((covered / planned) * 100)))
        : Math.round(
            rows.reduce((sum, row) => sum + Math.max(0, asNumber(row?.progress, 0)), 0) /
              rows.length,
          );
      const remaining = rows.reduce(
        (sum, row) => sum + Math.max(0, asNumber(row?.remainingAreaM2, 0)),
        0,
      );
      const active = rows.some((row) => String(row?.status || "") === "active");
      const completed = rows.every(
        (row) => Boolean(row?.isDone) || String(row?.status || "") === "completed",
      );
      const statusEl = item?.querySelector(".mission-area-pass-status");
      if (item) {
        item.classList.toggle("is-active", active);
        item.classList.toggle("is-completed", completed);
      }
      if (statusEl) {
        statusEl.textContent = active ? "ACTIVE" : completed ? "DONE" : "PLANNED";
      }
      progressEl.textContent = `${progress}% observed · overlay ${formatArea(remaining)}`;
    });
  };

  const updateCoverageDepthLegend = (payload) => {
    const summaries = Array.isArray(payload?.coverageDepthSummaries)
      ? payload.coverageDepthSummaries
      : [];
    const legend = document.getElementById("mission-area-depth-legend");
    if (!legend) {
      return;
    }
    legend.hidden = !summaries.length;
    legend.dataset.available = summaries.length ? "1" : "0";
    if (summaries.length) {
      const outerLegend = legend.closest("#mission-legend");
      if (outerLegend) {
        outerLegend.classList.add("is-visible");
        outerLegend.setAttribute("aria-hidden", "false");
      }
    }
    [0, 1, 2].forEach((depth) => {
      const item = legend.querySelector(`[data-coverage-depth="${depth}"]`);
      if (!item) {
        return;
      }
      const rows = summaries.filter((row) => normalizeCoverageDepth(row?.coverageDepth) === depth);
      const area = rows.reduce((sum, row) => sum + Math.max(0, asNumber(row?.areaM2, 0)), 0);
      const aircraft = [...new Set(rows.flatMap((row) => Array.isArray(row?.activeAgents) ? row.activeAgents : []))];
      const detail = item.querySelector(".mission-area-depth-detail");
      item.classList.toggle("is-empty", !rows.length);
      if (detail) {
        detail.textContent = rows.length
          ? `${area > 0 ? formatArea(area) : `${rows.reduce((sum, row) => sum + Math.max(0, asNumber(row?.geometryCount, 0)), 0)} areas`}${aircraft.length ? ` · ${aircraft.join(",")}` : ""}`
          : "-";
      }
    });
  };

  const setCollection = (payload) => {
    ensureLayers();
    const source = map?.getSource(SOURCE_ID);
    if (!source) {
      return false;
    }
    const collection = decorateRemainingAreaCollection(payload, colors);
    source.setData(collection);
    updateCoverageDepthLegend(payload);
    updatePassProgressLegend(payload);
    lastPayload = payload || null;
    return true;
  };

  const clear = () => {
    lastSignature = "";
    setCollection({ featureCollection: { type: "FeatureCollection", features: [] } });
  };

  const refresh = async () => {
    if (typeof document !== "undefined" && document.hidden) {
      return;
    }
    const requestedPlanId = missionPlanId;
    const requestedGeneration = requestGeneration;
    const endpoint = requestedPlanId === null
      ? ENDPOINT
      : `${ENDPOINT}?missionPlanID=${encodeURIComponent(requestedPlanId)}`;
    try {
      const response = await fetch(endpoint, { method: "GET", cache: "no-store" });
      const payload = await response.json();
      if (
        requestedGeneration !== requestGeneration ||
        requestedPlanId !== missionPlanId
      ) {
        return;
      }
      if (!payload?.ok || payload.available === false) {
        clear();
        return;
      }
      const payloadPlanId = Number(payload.missionPlanID);
      if (
        requestedPlanId !== null &&
        (!Number.isFinite(payloadPlanId) || Math.trunc(payloadPlanId) !== requestedPlanId)
      ) {
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
      if (setCollection(payload)) {
        lastSignature = signature;
      }
    } catch (err) {
      const now = Date.now();
      if (now - lastWarnAt >= 5000) {
        logStatus("Remaining area fetch failed", { level: "warn", ttlMs: 2500 });
        lastWarnAt = now;
      }
    }
  };

  const setMissionPlanId = (value) => {
    const parsed = Number(value);
    const nextPlanId = Number.isFinite(parsed) && parsed > 0
      ? Math.trunc(parsed)
      : null;
    if (nextPlanId === missionPlanId) {
      return;
    }
    missionPlanId = nextPlanId;
    requestGeneration += 1;
    clear();
    void refresh();
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

  // Visualization-only toggle: polling/state keep running, only the map layers hide.
  const setVisible = (visible) => {
    layersVisible = !!visible;
    applyVisibility();
  };

  return {
    start,
    stop,
    refresh,
    clear,
    setMissionPlanId,
    setVisible,
    isVisible: () => layersVisible,
    getLastPayload: () => lastPayload,
  };
};
