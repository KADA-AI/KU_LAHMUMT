import { buildDashedSegments, createLineLayer } from "./vehicle_markers.js";

const LAYER_DEFINITIONS = {
  connected: {
    id: "lah-uav-comm-connected",
    color: "#38bdf8",
    dashM: 0,
    gapM: 0,
  },
  terrainBlocked: {
    id: "lah-uav-comm-terrain-blocked",
    color: "#f59e0b",
    dashM: 140,
    gapM: 85,
  },
  outOfRange: {
    id: "lah-uav-comm-out-of-range",
    color: "#c084fc",
    dashM: 80,
    gapM: 55,
  },
  reportedDisconnected: {
    id: "lah-uav-comm-reported-disconnected",
    color: "#fb7185",
    dashM: 45,
    gapM: 45,
  },
  unknown: {
    id: "lah-uav-comm-unknown",
    color: "#94a3b8",
    dashM: 35,
    gapM: 80,
  },
};

const RENDER_INTERVAL_MS = 200;

const finiteCoordinate = (value) => {
  const lat = Number(value?.lat ?? value?.latitude);
  const lon = Number(value?.lon ?? value?.longitude);
  const alt = Number(value?.alt ?? value?.altitude);
  if (![lat, lon, alt].every(Number.isFinite)) {
    return null;
  }
  return { lat, lon, alt };
};

const normalizeStatus = (raw) => {
  const status = String(raw?.status || "");
  if (Object.prototype.hasOwnProperty.call(LAYER_DEFINITIONS, status)) {
    return status;
  }
  if (status === "pending" || status === "demUnknown") {
    return "unknown";
  }
  if (raw?.reportedConnected === false) {
    return "reportedDisconnected";
  }
  if (raw?.withinRange === false) {
    return "outOfRange";
  }
  if (raw?.terrainVisible === false) {
    return "terrainBlocked";
  }
  if (raw?.communicationAvailable === true) {
    return "connected";
  }
  return "unknown";
};

export const normalizeLahUavCommunicationLink = (raw) => {
  const from = finiteCoordinate(raw?.from);
  const to = finiteCoordinate(raw?.to);
  if (!from || !to) {
    return null;
  }
  const mannedAircraft = String(raw?.mannedAircraft || raw?.lah || "LAH");
  const uav = String(raw?.uav || "UAV");
  return {
    ...raw,
    id: String(raw?.id || `${mannedAircraft}:${uav}`),
    mannedAircraft,
    uav,
    from,
    to,
    status: normalizeStatus(raw),
  };
};

export const summarizeLahUavCommunicationLinks = (links) => {
  const counts = {
    connected: 0,
    terrainBlocked: 0,
    outOfRange: 0,
    reportedDisconnected: 0,
    unknown: 0,
  };
  (Array.isArray(links) ? links : []).forEach((link) => {
    const status = String(link?.status || "unknown");
    if (Object.prototype.hasOwnProperty.call(counts, status)) {
      counts[status] += 1;
    }
  });
  return counts;
};

const mercatorPoint = (point) =>
  maplibregl.MercatorCoordinate.fromLngLat(
    { lng: Number(point.lon), lat: Number(point.lat) },
    Number(point.alt),
  );

const straightSegment = (start, end) => [
  start.x,
  start.y,
  start.z,
  end.x,
  end.y,
  end.z,
];

const geometrySignature = (link) => [
  link.id,
  link.status,
  link.from.lat,
  link.from.lon,
  link.from.alt,
  link.to.lat,
  link.to.lon,
  link.to.alt,
].join(":");

export const initLahUavCommunicationLos = (
  map,
  { toggle = null, legend = null } = {},
) => {
  let mapReady = typeof map.isStyleLoaded === "function" ? map.isStyleLoaded() : map.loaded();
  let links = [];
  let visible = true;
  let lastStep = null;
  let lastPayloadSignature = null;
  let lastLegendSignature = null;
  let lastRenderAt = 0;
  let renderTimer = null;
  let appliedVisible = null;
  const layers = {};
  const bufferSignatures = Object.fromEntries(
    Object.keys(LAYER_DEFINITIONS).map((status) => [status, null]),
  );

  const updateLegend = () => {
    const counts = summarizeLahUavCommunicationLinks(links);
    const signature = [
      visible,
      links.length,
      ...Object.values(counts),
    ].join(":");
    if (signature === lastLegendSignature) {
      return;
    }
    lastLegendSignature = signature;
    if (legend) {
      legend.hidden = !visible || links.length === 0;
      Object.entries(counts).forEach(([status, count]) => {
        const output = legend.querySelector(`[data-comm-los-count="${status}"]`);
        if (output) {
          output.textContent = String(count);
        }
      });
    }
    if (toggle) {
      toggle.title = [
        "유인기–무인기 통신 LOS",
        `연결 ${counts.connected}`,
        `지형 차폐 ${counts.terrainBlocked}`,
        `20km 초과 ${counts.outOfRange}`,
        `상태 끊김 ${counts.reportedDisconnected}`,
        `확인 중 ${counts.unknown}`,
      ].join(" · ");
    }
  };

  const ensureLayers = () => {
    if (!mapReady) {
      return;
    }
    Object.entries(LAYER_DEFINITIONS).forEach(([status, definition]) => {
      if (!map.getLayer(definition.id)) {
        const layer = createLineLayer(definition.id, definition.color);
        layer._useDepth = false;
        map.addLayer(layer);
        layers[status] = layer;
        bufferSignatures[status] = null;
      } else if (!layers[status]) {
        layers[status] = map.getLayer(definition.id);
      }
    });
  };

  const render = () => {
    renderTimer = null;
    lastRenderAt = typeof performance !== "undefined" ? performance.now() : Date.now();
    ensureLayers();
    if (Object.keys(layers).length !== Object.keys(LAYER_DEFINITIONS).length) {
      return;
    }
    let changed = false;
    if (appliedVisible !== visible) {
      Object.values(layers).forEach((layer) => layer.setVisible(visible));
      appliedVisible = visible;
      changed = true;
    }
    if (!visible) {
      if (changed) {
        map.triggerRepaint();
      }
      updateLegend();
      return;
    }

    const byStatus = Object.fromEntries(
      Object.keys(LAYER_DEFINITIONS).map((status) => [status, []]),
    );
    links.forEach((link) => byStatus[link.status]?.push(link));
    Object.entries(byStatus).forEach(([status, statusLinks]) => {
      const signature = statusLinks.map(geometrySignature).join("|");
      if (signature === bufferSignatures[status]) {
        return;
      }
      const definition = LAYER_DEFINITIONS[status];
      const positions = [];
      statusLinks.forEach((link) => {
        let start;
        let end;
        try {
          start = mercatorPoint(link.from);
          end = mercatorPoint(link.to);
        } catch (_err) {
          return;
        }
        if (status === "connected") {
          positions.push(...straightSegment(start, end));
        } else {
          positions.push(
            ...buildDashedSegments(
              start,
              end,
              definition.dashM,
              definition.gapM,
            ),
          );
        }
      });
      layers[status].updatePositions(positions);
      bufferSignatures[status] = signature;
      changed = true;
    });
    if (changed) {
      map.triggerRepaint();
    }
    updateLegend();
  };

  const scheduleRender = ({ immediate = false } = {}) => {
    if (!visible) {
      updateLegend();
      return;
    }
    const now = typeof performance !== "undefined" ? performance.now() : Date.now();
    const delay = immediate
      ? 0
      : Math.max(0, RENDER_INTERVAL_MS - (now - lastRenderAt));
    if (delay <= 0) {
      if (renderTimer !== null) {
        clearTimeout(renderTimer);
      }
      render();
      return;
    }
    if (renderTimer === null) {
      renderTimer = setTimeout(render, delay);
    }
  };

  const setVisible = (nextVisible) => {
    const next = Boolean(nextVisible);
    if (next === visible && appliedVisible === visible) {
      return;
    }
    visible = next;
    if (toggle) {
      toggle.classList.toggle("is-active", visible);
      toggle.setAttribute("aria-pressed", visible ? "true" : "false");
    }
    if (!visible && renderTimer !== null) {
      clearTimeout(renderTimer);
      renderTimer = null;
    }
    if (visible) {
      scheduleRender({ immediate: true });
    } else {
      render();
    }
  };

  const loadFromReference = (payload) => {
    const rawStep = payload?.step;
    const rows = Array.isArray(payload?.communicationLinks)
      ? payload.communicationLinks
      : [];
    if ((rawStep === null || rawStep === undefined) && rows.length === 0) {
      lastStep = null;
      links = [];
      lastPayloadSignature = "";
      scheduleRender({ immediate: true });
      return;
    }
    const step = rawStep === null || rawStep === undefined ? Number.NaN : Number(rawStep);
    if (Number.isFinite(step) && Number.isFinite(lastStep) && step < lastStep) {
      return;
    }
    if (Number.isFinite(step)) {
      lastStep = step;
    }
    const nextLinks = rows.map(normalizeLahUavCommunicationLink).filter(Boolean);
    const nextSignature = nextLinks.map(geometrySignature).join("|");
    if (nextSignature === lastPayloadSignature) {
      return;
    }
    links = nextLinks;
    lastPayloadSignature = nextSignature;
    scheduleRender();
  };

  if (toggle) {
    toggle.addEventListener("click", () => setVisible(!visible));
  }
  if (mapReady) {
    ensureLayers();
  } else {
    map.once("load", () => {
      mapReady = true;
      ensureLayers();
      scheduleRender({ immediate: true });
    });
  }
  map.on("styledata", () => {
    mapReady = true;
    ensureLayers();
    appliedVisible = null;
    scheduleRender({ immediate: true });
  });
  setVisible(true);

  return {
    loadFromReference,
    setVisible,
    getLinks: () => links.map((link) => ({ ...link })),
  };
};
