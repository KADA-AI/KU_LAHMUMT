import { buildDashedSegments, createLineLayer } from "./vehicle_markers.js";
import { normalizeLahUavCommunicationLink } from "./lah_uav_communication_los.js?v=20260725-lah-role-status-1";

const CLEAR_LAYER_ID = "enemy-lah-los-clear";
const BLOCKED_LAYER_ID = "enemy-lah-los-blocked";
const UNKNOWN_LAYER_ID = "enemy-lah-los-unknown";

// For the role-oriented overlay, a clear enemy ray means exposure (red),
// while a terrain-blocked ray means the cover point is effective (green).
const CLEAR_COLOR = "#ff4d62";
const BLOCKED_COLOR = "#35e6a1";
const UNKNOWN_COLOR = "#a8b2c3";
const BLOCKED_DASH_M = 140;
const BLOCKED_GAP_M = 85;
const UNKNOWN_DASH_M = 45;
const UNKNOWN_GAP_M = 90;
const LOS_RENDER_INTERVAL_MS = 200;
// Gap kept between the role panel and the agent column beneath it.
const ROLE_LEGEND_CLEARANCE_PX = 12;

const finiteCoordinate = (value) => {
  const lat = Number(value?.lat ?? value?.latitude);
  const lon = Number(value?.lon ?? value?.longitude);
  const alt = Number(value?.alt ?? value?.altitude);
  if (![lat, lon, alt].every(Number.isFinite)) {
    return null;
  }
  return { lat, lon, alt };
};

export const normalizeEnemyLahLosLink = (raw) => {
  const from = finiteCoordinate(raw?.from);
  const to = finiteCoordinate(raw?.to);
  if (!from || !to) {
    return null;
  }
  const visible = typeof raw?.visible === "boolean" ? raw.visible : null;
  return {
    ...raw,
    id: String(raw?.id || `${raw?.aircraft || "LAH"}:${raw?.targetID ?? "?"}`),
    from,
    to,
    visible,
    status: visible === true ? "clear" : visible === false ? "blocked" : "unknown",
  };
};

export const summarizeEnemyLahLosLinks = (links) => {
  const counts = { clear: 0, blocked: 0, unknown: 0 };
  (Array.isArray(links) ? links : []).forEach((link) => {
    const status = String(link?.status || "unknown");
    if (Object.prototype.hasOwnProperty.call(counts, status)) {
      counts[status] += 1;
    }
  });
  return counts;
};

const COVER_PRIORITY = { clear: 0, unknown: 1, blocked: 2 };

export const selectRepresentativeEnemyLinks = (links) => {
  const byAircraft = new Map();
  (Array.isArray(links) ? links : []).forEach((link) => {
    const aircraft = String(link?.aircraft || "");
    if (!aircraft) return;
    const current = byAircraft.get(aircraft);
    const nextKey = [
      COVER_PRIORITY[link.status] ?? 9,
      Number(link.distanceM) || Number.POSITIVE_INFINITY,
      Number(link.targetID) || Number.POSITIVE_INFINITY,
    ];
    const currentKey = current?.key;
    if (!currentKey || nextKey[0] < currentKey[0]
      || (nextKey[0] === currentKey[0] && nextKey[1] < currentKey[1])
      || (nextKey[0] === currentKey[0] && nextKey[1] === currentKey[1] && nextKey[2] < currentKey[2])) {
      byAircraft.set(aircraft, { key: nextKey, link });
    }
  });
  return [...byAircraft.values()].map((entry) => entry.link);
};

export const assessCoverState = (links, aircraft) => {
  const rows = (Array.isArray(links) ? links : []).filter(
    (link) => String(link?.aircraft) === String(aircraft),
  );
  if (!rows.length) return { state: "no-target", text: "발견 적 없음" };
  const exposed = rows.filter((link) => link.status === "clear").length;
  const blocked = rows.filter((link) => link.status === "blocked").length;
  if (exposed > 0) {
    return { state: "bad", text: `노출 · ${exposed}개 적 LOS` };
  }
  if (blocked === rows.length) {
    return { state: "good", text: `엄폐 양호 · ${blocked}개 차폐` };
  }
  return { state: "checking", text: "엄폐 확인 중" };
};

export const assessRelayState = (links) => {
  const rows = (Array.isArray(links) ? links : []).filter(
    (link) => String(link?.mannedAircraft) === "LAH1",
  );
  if (!rows.length) return { state: "checking", text: "중계 대상 없음" };
  const connected = rows.filter((link) => link.status === "connected").length;
  const unresolved = rows.filter((link) => link.status === "unknown").length;
  if (connected === rows.length) {
    return { state: "good", text: `중계 양호 · ${connected}/${rows.length}` };
  }
  if (connected > 0) {
    return { state: "warn", text: `중계 일부 · ${connected}/${rows.length}` };
  }
  if (unresolved === rows.length) {
    return { state: "checking", text: "중계 확인 중" };
  }
  return { state: "bad", text: `중계 불량 · 0/${rows.length}` };
};

const ROLE_STATE_CLASSES = [
  "is-good",
  "is-warn",
  "is-bad",
  "is-checking",
  "is-no-target",
];

const applyRoleStatus = (element, status) => {
  if (!element) return;
  ROLE_STATE_CLASSES.forEach((className) => element.classList.remove(className));
  element.classList.add(`is-${status.state}`);
  element.textContent = status.text;
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

const linkGeometrySignature = (link) => [
  link.id,
  link.status,
  link.from.lat,
  link.from.lon,
  link.from.alt,
  link.to.lat,
  link.to.lon,
  link.to.alt,
].join(":");

const payloadGeometrySignature = (items) =>
  items.map(linkGeometrySignature).join("|");

export const initEnemyLahLos = (
  map,
  { toggle = null, legend = null, onVisibilityChange = null } = {},
) => {
  let mapReady = typeof map.isStyleLoaded === "function" ? map.isStyleLoaded() : map.loaded();
  let links = [];
  let communicationLinks = [];
  let visible = true;
  let lastStep = null;
  let lastPayloadSignature = null;
  let lastLegendSignature = null;
  let lastRenderAt = 0;
  let renderTimer = null;
  let appliedVisible = null;
  const bufferSignatures = { clear: null, blocked: null, unknown: null };
  let clearLayer = null;
  let blockedLayer = null;
  let unknownLayer = null;

  // The role panel and the agent column both live in the top-left corner.
  // Publishing the panel's real bottom edge lets the column sit below it at
  // any viewport width and however many rows the panel ends up rendering.
  const publishLegendBottom = () => {
    if (!legend) {
      return;
    }
    const occupied =
      legend.hidden || !legend.offsetHeight
        ? 0
        : legend.offsetTop + legend.offsetHeight + ROLE_LEGEND_CLEARANCE_PX;
    document.documentElement.style.setProperty(
      "--lah-role-legend-bottom",
      `${Math.round(occupied)}px`,
    );
  };

  const updateLegend = () => {
    const relay = assessRelayState(communicationLinks);
    const coverByAircraft = Object.fromEntries(
      ["LAH1", "LAH2", "LAH3"].map((aircraft) => [aircraft, assessCoverState(links, aircraft)]),
    );
    const signature = [
      visible,
      relay.state,
      relay.text,
      ...Object.values(coverByAircraft).flatMap((status) => [status.state, status.text]),
    ].join(":");
    if (signature === lastLegendSignature) {
      return;
    }
    lastLegendSignature = signature;
    if (legend) {
      legend.hidden = !visible;
      applyRoleStatus(legend.querySelector("[data-lah-relay-status]"), relay);
      Object.entries(coverByAircraft).forEach(([aircraft, status]) => {
        applyRoleStatus(
          legend.querySelector(`[data-lah-cover-status="${aircraft}"]`),
          status,
        );
      });
      publishLegendBottom();
    }
    if (toggle) {
      toggle.title = [
        "LAH 역할 상태",
        relay.text,
        `LAH1 ${coverByAircraft.LAH1.text}`,
        `LAH2 ${coverByAircraft.LAH2.text}`,
        `LAH3 ${coverByAircraft.LAH3.text}`,
      ].join(" · ");
    }
  };

  const ensureLayers = () => {
    if (!mapReady) {
      return;
    }
    if (!map.getLayer(CLEAR_LAYER_ID)) {
      clearLayer = createLineLayer(CLEAR_LAYER_ID, CLEAR_COLOR);
      clearLayer._useDepth = false;
      map.addLayer(clearLayer);
      bufferSignatures.clear = null;
    } else if (!clearLayer) {
      clearLayer = map.getLayer(CLEAR_LAYER_ID);
    }
    if (!map.getLayer(BLOCKED_LAYER_ID)) {
      blockedLayer = createLineLayer(BLOCKED_LAYER_ID, BLOCKED_COLOR);
      blockedLayer._useDepth = false;
      map.addLayer(blockedLayer);
      bufferSignatures.blocked = null;
    } else if (!blockedLayer) {
      blockedLayer = map.getLayer(BLOCKED_LAYER_ID);
    }
    if (!map.getLayer(UNKNOWN_LAYER_ID)) {
      unknownLayer = createLineLayer(UNKNOWN_LAYER_ID, UNKNOWN_COLOR);
      unknownLayer._useDepth = false;
      map.addLayer(unknownLayer);
      bufferSignatures.unknown = null;
    } else if (!unknownLayer) {
      unknownLayer = map.getLayer(UNKNOWN_LAYER_ID);
    }
  };

  const render = () => {
    renderTimer = null;
    lastRenderAt = typeof performance !== "undefined" ? performance.now() : Date.now();
    ensureLayers();
    if (!clearLayer || !blockedLayer || !unknownLayer) {
      return;
    }
    let changed = false;
    if (appliedVisible !== visible) {
      clearLayer.setVisible(visible);
      blockedLayer.setVisible(visible);
      unknownLayer.setVisible(visible);
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

    const byStatus = { clear: [], blocked: [], unknown: [] };
    selectRepresentativeEnemyLinks(links).forEach((link) => {
      byStatus[link.status]?.push(link);
    });
    const layerByStatus = {
      clear: clearLayer,
      blocked: blockedLayer,
      unknown: unknownLayer,
    };
    Object.entries(byStatus).forEach(([status, statusLinks]) => {
      const signature = statusLinks.map(linkGeometrySignature).join("|");
      if (signature === bufferSignatures[status]) {
        return;
      }
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
        if (status === "clear") {
          positions.push(...straightSegment(start, end));
        } else if (status === "blocked") {
          positions.push(
            ...buildDashedSegments(start, end, BLOCKED_DASH_M, BLOCKED_GAP_M),
          );
        } else {
          positions.push(
            ...buildDashedSegments(start, end, UNKNOWN_DASH_M, UNKNOWN_GAP_M),
          );
        }
      });
      layerByStatus[status].updatePositions(positions);
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
      : Math.max(0, LOS_RENDER_INTERVAL_MS - (now - lastRenderAt));
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
    if (typeof onVisibilityChange === "function") {
      onVisibilityChange(visible);
    }
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
    const rows = Array.isArray(payload?.losLinks) ? payload.losLinks : [];
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
    const nextLinks = rows.map(normalizeEnemyLahLosLink).filter(Boolean);
    const nextSignature = payloadGeometrySignature(nextLinks);
    if (nextSignature === lastPayloadSignature) {
      return;
    }
    links = nextLinks;
    lastPayloadSignature = nextSignature;
    scheduleRender();
  };

  const loadCommunicationFromReference = (payload) => {
    const rows = Array.isArray(payload?.communicationLinks)
      ? payload.communicationLinks
      : [];
    communicationLinks = rows
      .map(normalizeLahUavCommunicationLink)
      .filter(Boolean);
    updateLegend();
  };

  if (legend) {
    publishLegendBottom();
    if (typeof ResizeObserver === "function") {
      new ResizeObserver(publishLegendBottom).observe(legend);
    }
    window.addEventListener("resize", publishLegendBottom);
  }

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
    loadCommunicationFromReference,
    setVisible,
    getLinks: () => links.map((link) => ({ ...link })),
  };
};
