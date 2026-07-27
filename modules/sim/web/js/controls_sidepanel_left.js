import { getConfig } from "./config.js";
import { palette } from "./palette.js";
import { buildStyle, ELEVATION_TINT_LAYER_ID } from "./map_style.js";

const UAVS = ["UAV1", "UAV2", "UAV3"];
const PREVIEW_WIDTH = 320;
const PREVIEW_HEIGHT = 180;
const SOURCE_WIDTH = 320;
const SOURCE_HEIGHT = 180;
const RENDER_INTERVAL_MS = 200;
const CAMERA_PIXEL_RATIO = 1;
const MIN_PANEL_HEIGHT = 260;
const FEED_TARGET_SOURCE_ID = "camera-feed-targets";
const FEED_TARGET_GLOW_LAYER_ID = "camera-feed-targets-glow";
const FEED_TARGET_CORE_LAYER_ID = "camera-feed-targets-core";
const FEED_TARGET_LABEL_LAYER_ID = "camera-feed-targets-label";

const THREAT_STYLE = {
  0: { color: "#a9c1df", glow: "#ffd86e", size: 9, label: "ROI" },
  1: { color: "#d8c466", glow: "#ffe08a", size: 11, label: "TNK" },
  2: { color: "#6fb5c8", glow: "#95e0ef", size: 10, label: "ARM" },
  3: { color: "#da8952", glow: "#ffaf7a", size: 10, label: "ART" },
  4: { color: "#cdb27b", glow: "#ffe2a3", size: 9, label: "GUN" },
  5: { color: "#7cb2d9", glow: "#b8ddff", size: 12, label: "ADA" },
  6: { color: "#c8bc85", glow: "#efe3ad", size: 8, label: "INF" },
};

const clamp = (value, min, max) => Math.min(max, Math.max(min, value));

const formatNumber = (value, digits = 0, fallback = "--") => {
  return Number.isFinite(value) ? Number(value).toFixed(digits) : fallback;
};

const getAgentColor = (agent) => {
  const style = getComputedStyle(document.documentElement);
  return style.getPropertyValue(`--${String(agent || "").toLowerCase()}`).trim() || "#7bd3a3";
};

const llToLocalMeters = (origin, point) => {
  if (!origin || !point) {
    return null;
  }
  const lat0 = Number(origin.lat);
  const lon0 = Number(origin.lon);
  const lat = Number(point.lat);
  const lon = Number(point.lon);
  if (![lat0, lon0, lat, lon].every(Number.isFinite)) {
    return null;
  }
  const metersPerDegLat = 111320.0;
  const cosLat = Math.cos((lat0 * Math.PI) / 180) || 1e-6;
  const metersPerDegLon = metersPerDegLat * cosLat;
  return {
    x: (lon - lon0) * metersPerDegLon,
    y: (lat - lat0) * metersPerDegLat,
  };
};

const computeFootprintMetrics = (ring) => {
  if (!Array.isArray(ring) || ring.length < 4) {
    return { widthMeters: null, heightMeters: null };
  }
  const points = ring.slice(0, 4).map((coord) => ({ lon: coord[0], lat: coord[1] }));
  const origin = points[0];
  const local = points.map((point) => llToLocalMeters(origin, point));
  if (local.some((point) => !point)) {
    return { widthMeters: null, heightMeters: null };
  }
  const dist = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);
  const widthMeters = (dist(local[0], local[1]) + dist(local[2], local[3])) * 0.5;
  const heightMeters = (dist(local[1], local[2]) + dist(local[3], local[0])) * 0.5;
  return { widthMeters, heightMeters };
};

const roundedRectPath = (ctx, x, y, width, height, radius) => {
  const r = Math.min(radius, width * 0.5, height * 0.5);
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + width, y, x + width, y + height, r);
  ctx.arcTo(x + width, y + height, x, y + height, r);
  ctx.arcTo(x, y + height, x, y, r);
  ctx.arcTo(x, y, x + width, y, r);
  ctx.closePath();
};

const drawBackdrop = (ctx, color) => {
  const gradient = ctx.createLinearGradient(0, 0, PREVIEW_WIDTH, PREVIEW_HEIGHT);
  gradient.addColorStop(0, "rgba(14, 20, 28, 0.98)");
  gradient.addColorStop(1, "rgba(8, 12, 18, 0.98)");
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, PREVIEW_WIDTH, PREVIEW_HEIGHT);

  ctx.strokeStyle = "rgba(214, 226, 240, 0.08)";
  ctx.lineWidth = 1;
  [0.2, 0.5, 0.8].forEach((ratio) => {
    const x = PREVIEW_WIDTH * ratio;
    const y = PREVIEW_HEIGHT * ratio;
    ctx.beginPath();
    ctx.moveTo(x, 18);
    ctx.lineTo(x, PREVIEW_HEIGHT - 18);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(18, y);
    ctx.lineTo(PREVIEW_WIDTH - 18, y);
    ctx.stroke();
  });

  ctx.strokeStyle = `${color}40`;
  ctx.lineWidth = 2;
  roundedRectPath(ctx, 1, 1, PREVIEW_WIDTH - 2, PREVIEW_HEIGHT - 2, 16);
  ctx.stroke();
};

const drawReticle = (ctx) => {
  const x = PREVIEW_WIDTH * 0.5;
  const y = PREVIEW_HEIGHT * 0.5;
  ctx.save();
  ctx.strokeStyle = "rgba(244, 248, 252, 0.96)";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(x, y, 11, 0, Math.PI * 2);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(x - 16, y);
  ctx.lineTo(x + 16, y);
  ctx.moveTo(x, y - 16);
  ctx.lineTo(x, y + 16);
  ctx.stroke();
  ctx.fillStyle = "rgba(244, 248, 252, 0.96)";
  ctx.beginPath();
  ctx.arc(x, y, 3.2, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
};

const drawNoSignal = (ctx, color, label) => {
  drawBackdrop(ctx, color);
  ctx.fillStyle = "rgba(232, 239, 247, 0.92)";
  ctx.font = "700 18px Segoe UI";
  ctx.textAlign = "center";
  ctx.fillText("NO CAMERA FEED", PREVIEW_WIDTH * 0.5, PREVIEW_HEIGHT * 0.46);
  ctx.fillStyle = "rgba(161, 177, 196, 0.82)";
  ctx.font = "12px Segoe UI";
  ctx.fillText(label, PREVIEW_WIDTH * 0.5, PREVIEW_HEIGHT * 0.58);
};

const computeAffineTransform = (src, dst) => {
  const [s1, s2, s3] = src;
  const [d1, d2, d3] = dst;
  const den =
    s1.x * (s2.y - s3.y) +
    s2.x * (s3.y - s1.y) +
    s3.x * (s1.y - s2.y);
  if (!Number.isFinite(den) || Math.abs(den) < 1e-6) {
    return null;
  }
  const a =
    (d1.x * (s2.y - s3.y) +
      d2.x * (s3.y - s1.y) +
      d3.x * (s1.y - s2.y)) / den;
  const b =
    (d1.y * (s2.y - s3.y) +
      d2.y * (s3.y - s1.y) +
      d3.y * (s1.y - s2.y)) / den;
  const c =
    (d1.x * (s3.x - s2.x) +
      d2.x * (s1.x - s3.x) +
      d3.x * (s2.x - s1.x)) / den;
  const d =
    (d1.y * (s3.x - s2.x) +
      d2.y * (s1.x - s3.x) +
      d3.y * (s2.x - s1.x)) / den;
  const e =
    (d1.x * (s2.x * s3.y - s3.x * s2.y) +
      d2.x * (s3.x * s1.y - s1.x * s3.y) +
      d3.x * (s1.x * s2.y - s2.x * s1.y)) / den;
  const f =
    (d1.y * (s2.x * s3.y - s3.x * s2.y) +
      d2.y * (s3.x * s1.y - s1.x * s3.y) +
      d3.y * (s1.x * s2.y - s2.x * s1.y)) / den;
  return { a, b, c, d, e, f };
};

const drawMappedTriangle = (ctx, source, srcTri, dstTri) => {
  const matrix = computeAffineTransform(srcTri, dstTri);
  if (!matrix) {
    return;
  }
  ctx.save();
  ctx.beginPath();
  ctx.moveTo(dstTri[0].x, dstTri[0].y);
  ctx.lineTo(dstTri[1].x, dstTri[1].y);
  ctx.lineTo(dstTri[2].x, dstTri[2].y);
  ctx.closePath();
  ctx.clip();
  ctx.transform(matrix.a, matrix.b, matrix.c, matrix.d, matrix.e, matrix.f);
  ctx.drawImage(source, 0, 0);
  ctx.restore();
};

const buildCameraFeedStyle = (config) => {
  const style = buildStyle(config, palette);
  const excluded = new Set([
    "boundary",
    "transportation",
    "place-labels",
    "water-labels",
    "road-labels",
    "building",
    "building-3d",
    // Keep the extra relief pass on the main map only. The three camera maps
    // already use hillshade and refresh frequently, so they do not need it.
    ELEVATION_TINT_LAYER_ID,
  ]);
  return {
    ...style,
    layers: style.layers
      .filter((layer) => !excluded.has(layer.id))
      .map((layer) => {
        if (layer.id === "landcover") {
          return {
            ...layer,
            paint: {
              ...layer.paint,
              "fill-opacity": 0.92,
            },
          };
        }
        if (layer.id === "landuse" || layer.id === "park") {
          return {
            ...layer,
            paint: {
              ...layer.paint,
              "fill-opacity": 0.86,
            },
          };
        }
        return layer;
      }),
  };
};

const pointInPolygon = (point, ring) => {
  if (!Array.isArray(ring) || ring.length < 4) {
    return false;
  }
  const [x, y] = point;
  let inside = false;
  for (let i = 0, j = ring.length - 1; i < ring.length; j = i, i += 1) {
    const xi = ring[i][0];
    const yi = ring[i][1];
    const xj = ring[j][0];
    const yj = ring[j][1];
    const intersects =
      yi > y !== yj > y &&
      x < ((xj - xi) * (y - yi)) / ((yj - yi) || 1e-9) + xi;
    if (intersects) {
      inside = !inside;
    }
  }
  return inside;
};

const buildThreatFeatures = (targets) => {
  return targets.map((target) => {
    const style = THREAT_STYLE[target.type] || THREAT_STYLE[1];
    const alive = target.alive !== false;
    return {
      type: "Feature",
      properties: {
        color: alive ? style.color : "#7a807b",
        glowColor: alive ? style.glow : "#a0a6a1",
        size: alive ? style.size : Math.max(6, style.size - 2),
        label: target.name ? String(target.name) : style.label,
      },
      geometry: {
        type: "Point",
        coordinates: [target.lon, target.lat],
      },
    };
  });
};

const addThreatLayers = (map) => {
  if (!map.getSource(FEED_TARGET_SOURCE_ID)) {
    map.addSource(FEED_TARGET_SOURCE_ID, {
      type: "geojson",
      data: { type: "FeatureCollection", features: [] },
    });
  }
  if (!map.getLayer(FEED_TARGET_GLOW_LAYER_ID)) {
    map.addLayer({
      id: FEED_TARGET_GLOW_LAYER_ID,
      type: "circle",
      source: FEED_TARGET_SOURCE_ID,
      paint: {
        "circle-color": ["get", "glowColor"],
        "circle-radius": ["+", ["get", "size"], 10],
        "circle-opacity": 0.22,
        "circle-blur": 0.85,
      },
    });
  }
  if (!map.getLayer(FEED_TARGET_CORE_LAYER_ID)) {
    map.addLayer({
      id: FEED_TARGET_CORE_LAYER_ID,
      type: "circle",
      source: FEED_TARGET_SOURCE_ID,
      paint: {
        "circle-color": ["get", "color"],
        "circle-radius": ["get", "size"],
        "circle-stroke-color": "rgba(12, 18, 24, 0.9)",
        "circle-stroke-width": 2.2,
      },
    });
  }
  if (!map.getLayer(FEED_TARGET_LABEL_LAYER_ID)) {
    map.addLayer({
      id: FEED_TARGET_LABEL_LAYER_ID,
      type: "symbol",
      source: FEED_TARGET_SOURCE_ID,
      layout: {
        "text-field": ["get", "label"],
        "text-font": ["Noto Sans Regular"],
        "text-size": 10,
        "text-offset": [0, 1.35],
        "text-anchor": "top",
        "text-allow-overlap": true,
      },
      paint: {
        "text-color": "#f3f7fb",
        "text-halo-color": "rgba(6, 10, 16, 0.9)",
        "text-halo-width": 1.25,
        "text-opacity": 0.92,
      },
    });
  }
};

const getProjectedRing = (map, view) => {
  if (!map || !Array.isArray(view?.footprint) || view.footprint.length < 4) {
    return null;
  }
  const ring = view.footprint
    .slice(0, -1)
    .map((coord) => {
      const projected = map.project([coord[0], coord[1]]);
      return Number.isFinite(projected?.x) && Number.isFinite(projected?.y)
        ? { x: projected.x, y: projected.y }
        : null;
    })
    .filter(Boolean);
  return ring.length >= 4 ? ring : null;
};

const drawCameraFeed = (canvas, view, sourceMap) => {
  const ctx = canvas.getContext("2d");
  if (!ctx) {
    return;
  }
  const dpr = Math.min(CAMERA_PIXEL_RATIO, Math.max(0.5, window.devicePixelRatio || 1));
  if (canvas.width !== Math.round(PREVIEW_WIDTH * dpr) || canvas.height !== Math.round(PREVIEW_HEIGHT * dpr)) {
    canvas.width = Math.round(PREVIEW_WIDTH * dpr);
    canvas.height = Math.round(PREVIEW_HEIGHT * dpr);
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, PREVIEW_WIDTH, PREVIEW_HEIGHT);

  const color = getAgentColor(view.agent);
  if (!sourceMap) {
    drawNoSignal(ctx, color, "Camera map not ready");
    return;
  }
  const projectedRing = getProjectedRing(sourceMap, view);
  if (!projectedRing) {
    drawNoSignal(ctx, color, "Waiting for filming footprint");
    return;
  }

  const sourceCanvas = sourceMap.getCanvas();
  if (!sourceCanvas) {
    drawNoSignal(ctx, color, "Source canvas unavailable");
    return;
  }

  drawBackdrop(ctx, color);
  const quadCss = projectedRing.slice(0, 4);
  if (!quadCss) {
    drawNoSignal(ctx, color, "Invalid footprint quad");
    return;
  }
  const rect = sourceCanvas.getBoundingClientRect();
  const sourceWidth = Math.max(1, rect.width || sourceCanvas.clientWidth || sourceCanvas.width);
  const sourceHeight = Math.max(1, rect.height || sourceCanvas.clientHeight || sourceCanvas.height);
  const scaleX = sourceCanvas.width / sourceWidth;
  const scaleY = sourceCanvas.height / sourceHeight;
  const srcQuad = quadCss.map((point) => ({
    x: point.x * scaleX,
    y: point.y * scaleY,
  }));
  const dstQuad = [
    { x: 0, y: 0 },
    { x: PREVIEW_WIDTH, y: 0 },
    { x: PREVIEW_WIDTH, y: PREVIEW_HEIGHT },
    { x: 0, y: PREVIEW_HEIGHT },
  ];

  try {
    drawMappedTriangle(
      ctx,
      sourceCanvas,
      [srcQuad[0], srcQuad[1], srcQuad[3]],
      [dstQuad[0], dstQuad[1], dstQuad[3]],
    );
    drawMappedTriangle(
      ctx,
      sourceCanvas,
      [srcQuad[1], srcQuad[2], srcQuad[3]],
      [dstQuad[1], dstQuad[2], dstQuad[3]],
    );
  } catch (_error) {
    drawNoSignal(ctx, color, "Camera rectification blocked");
    return;
  }

  const overlay = ctx.createLinearGradient(0, 0, 0, PREVIEW_HEIGHT);
  overlay.addColorStop(0, "rgba(255,255,255,0.05)");
  overlay.addColorStop(0.5, "rgba(255,255,255,0)");
  overlay.addColorStop(1, "rgba(8,12,18,0.14)");
  ctx.fillStyle = overlay;
  ctx.fillRect(0, 0, PREVIEW_WIDTH, PREVIEW_HEIGHT);

  ctx.fillStyle = "rgba(7, 12, 18, 0.64)";
  ctx.fillRect(10, 10, 92, 24);
  ctx.strokeStyle = `${color}88`;
  ctx.lineWidth = 1;
  ctx.strokeRect(10, 10, 92, 24);
  ctx.fillStyle = "rgba(239, 245, 251, 0.96)";
  ctx.font = "700 12px Segoe UI";
  ctx.textAlign = "left";
  ctx.fillText("LIVE CAMERA", 20, 26);

  drawReticle(ctx);
};

const buildCardMarkup = (agent, color) => {
  return `
    <article class="sensor-card sensor-card-offline" data-agent="${agent}" style="--sensor-accent:${color}">
      <div class="sensor-card-head">
        <div>
          <div class="sensor-card-agent">${agent}</div>
          <div class="sensor-card-status" data-field="status">Offline</div>
        </div>
        <div class="sensor-card-heading" data-field="heading">No hdg</div>
      </div>
      <div class="sensor-preview">
        <canvas class="sensor-preview-canvas" data-field="canvas" width="${PREVIEW_WIDTH}" height="${PREVIEW_HEIGHT}"></canvas>
      </div>
      <div class="sensor-meta-grid">
        <div class="sensor-meta-item"><span>FOV</span><strong data-field="fov">--</strong></div>
        <div class="sensor-meta-item"><span>SEP</span><strong data-field="sep">--</strong></div>
        <div class="sensor-meta-item"><span>ALT</span><strong data-field="alt">--</strong></div>
        <div class="sensor-meta-item"><span>SPD</span><strong data-field="speed">--</strong></div>
      </div>
      <div class="sensor-footprint-meta">
        <span>Capture</span>
        <strong data-field="capture">No footprint</strong>
      </div>
    </article>
  `;
};

const buildShell = (body) => {
  const cardsHtml = UAVS.map((agent) => buildCardMarkup(agent, getAgentColor(agent))).join("");
  body.innerHTML = `
    <div class="sensor-panel-shell">
      <div class="sensor-panel-copy">
        <div class="sensor-panel-eyebrow">EO sensor feed</div>
        <div class="sensor-panel-title">Live Camera Views</div>
        <div class="sensor-panel-sub">Ground-only feed with threat overlays</div>
      </div>
      <div class="sensor-panel-cards">${cardsHtml}</div>
    </div>
  `;
};

const createSourceHost = () => {
  let host = document.getElementById("sensor-feed-source-host");
  if (host) {
    return host;
  }
  host = document.createElement("div");
  host.id = "sensor-feed-source-host";
  host.className = "sensor-feed-source-host";
  document.body.appendChild(host);
  return host;
};

const createCameraMap = (container, config) => {
  if (!window.maplibregl) {
    return null;
  }
  return new window.maplibregl.Map({
    container,
    style: buildCameraFeedStyle(config),
    center: config.center,
    zoom: 14,
    minZoom: config.minZoom,
    maxZoom: config.maxZoom + 2,
    pitch: 0,
    bearing: 0,
    interactive: false,
    pixelRatio: CAMERA_PIXEL_RATIO,
    attributionControl: false,
    fadeDuration: 0,
    renderWorldCopies: false,
  });
};

const getViewBounds = (view) => {
  const ring = Array.isArray(view?.footprint) ? view.footprint.slice(0, -1) : [];
  if (ring.length < 4) {
    return null;
  }
  let minLon = Infinity;
  let maxLon = -Infinity;
  let minLat = Infinity;
  let maxLat = -Infinity;
  ring.forEach((coord) => {
    minLon = Math.min(minLon, coord[0]);
    maxLon = Math.max(maxLon, coord[0]);
    minLat = Math.min(minLat, coord[1]);
    maxLat = Math.max(maxLat, coord[1]);
  });
  if (![minLon, maxLon, minLat, maxLat].every(Number.isFinite)) {
    return null;
  }
  return [
    [minLon, minLat],
    [maxLon, maxLat],
  ];
};

const filterTargetsForView = (targets, view) => {
  const ring = Array.isArray(view?.footprint) ? view.footprint.slice(0, -1) : [];
  if (ring.length < 4) {
    return [];
  }
  return targets.filter((target) => {
    if (!Number.isFinite(target?.lon) || !Number.isFinite(target?.lat)) {
      return false;
    }
    return pointInPolygon([target.lon, target.lat], ring);
  });
};

export const initLeftSidePanel = ({
  getFilmingViews,
  subscribeFilmingViews,
  getTargets,
  subscribeTargets,
} = {}) => {
  const toggle = document.getElementById("left-side-toggle");
  const panel = document.getElementById("left-side-panel");
  const title = panel?.querySelector(".side-panel-left-title");
  const body = panel?.querySelector(".side-panel-left-body");
  const leftControls = document.getElementById("left-controls");
  if (!toggle || !panel || !body) {
    return;
  }

  const config = getConfig(document.body);
  const sourceHost = createSourceHost();
  panel.classList.add("is-docked");
  toggle.classList.add("is-docked");
  if (title) {
    title.textContent = "UAV CAM";
  }
  buildShell(body);

  const cards = new Map();
  body.querySelectorAll(".sensor-card").forEach((element) => {
    const agent = element.dataset.agent;
    const sourceContainer = document.createElement("div");
    sourceContainer.className = "sensor-feed-source";
    sourceContainer.style.width = `${SOURCE_WIDTH}px`;
    sourceContainer.style.height = `${SOURCE_HEIGHT}px`;
    sourceHost.appendChild(sourceContainer);
    cards.set(agent, {
      agent,
      element,
      canvas: element.querySelector('[data-field="canvas"]'),
      status: element.querySelector('[data-field="status"]'),
      heading: element.querySelector('[data-field="heading"]'),
      fov: element.querySelector('[data-field="fov"]'),
      sep: element.querySelector('[data-field="sep"]'),
      alt: element.querySelector('[data-field="alt"]'),
      speed: element.querySelector('[data-field="speed"]'),
      capture: element.querySelector('[data-field="capture"]'),
      sourceContainer,
      sourceMap: null,
      mapReady: false,
      view: { agent, status: "offline", footprint: null },
    });
  });

  let mapsInitialized = false;
  let latestViews = UAVS.map((agent) => ({ agent, status: "offline", footprint: null }));
  let latestTargets = [];
  let lastRenderMs = 0;
  let renderTimer = null;

  const syncDocking = () => {
    const bottoms = [16];
    if (leftControls) {
      bottoms.push(leftControls.getBoundingClientRect().bottom + 14);
    }
    const top = clamp(Math.max(...bottoms), 120, window.innerHeight - MIN_PANEL_HEIGHT - 16);
    const toggleTop = clamp(top + 18, 132, window.innerHeight - 72);
    document.documentElement.style.setProperty("--left-sensor-panel-top", `${Math.round(top)}px`);
    document.documentElement.style.setProperty("--left-sensor-toggle-top", `${Math.round(toggleTop)}px`);
  };

  const paintCard = (card) => {
    if (!card?.canvas) {
      return;
    }
    drawCameraFeed(card.canvas, card.view, card.sourceMap);
  };

  const updateThreatSource = (card) => {
    if (!card?.sourceMap || !card.mapReady) {
      return;
    }
    const source = card.sourceMap.getSource(FEED_TARGET_SOURCE_ID);
    if (!source) {
      return;
    }
    const visibleTargets = filterTargetsForView(latestTargets, card.view);
    source.setData({
      type: "FeatureCollection",
      features: buildThreatFeatures(visibleTargets),
    });
  };

  const updateFeedMap = (card) => {
    if (!card?.sourceMap || !card.mapReady) {
      paintCard(card);
      return;
    }
    const bounds = getViewBounds(card.view);
    if (!bounds) {
      updateThreatSource(card);
      paintCard(card);
      return;
    }
    updateThreatSource(card);
    card.sourceMap.fitBounds(bounds, {
      padding: 32,
      duration: 0,
      pitch: 0,
      bearing: 0,
      linear: true,
      maxZoom: 17.5,
    });
    paintCard(card);
  };

  const ensureFeedMaps = () => {
    if (mapsInitialized) {
      return;
    }
    mapsInitialized = true;
    cards.forEach((card) => {
      card.sourceMap = createCameraMap(card.sourceContainer, config);
      if (!card.sourceMap) {
        return;
      }
      card.sourceMap.on("load", () => {
        addThreatLayers(card.sourceMap);
        card.mapReady = true;
        if (panel.classList.contains("is-open")) {
          updateFeedMap(card);
        }
      });
      card.sourceMap.on("render", () => {
        if (!panel.classList.contains("is-open")) {
          return;
        }
        paintCard(card);
      });
    });
  };

  const updateCardMeta = (card, view) => {
    card.view = view;
    const metrics = computeFootprintMetrics(view.footprint);
    const statusLabel =
      view.status === "active" ? "Tracking" : view.status === "idle" ? "Standby" : "Offline";
    card.element.classList.toggle("sensor-card-active", view.status === "active");
    card.element.classList.toggle("sensor-card-idle", view.status === "idle");
    card.element.classList.toggle("sensor-card-offline", view.status === "offline");
    card.status.textContent = statusLabel;
    card.heading.textContent = Number.isFinite(view.heading) ? `${Math.round(view.heading)} deg` : "No hdg";
    card.fov.textContent = Number.isFinite(view.filmingFov) ? `${Math.round(view.filmingFov)} deg` : "--";
    card.sep.textContent = Number.isFinite(view.separation) ? `${Math.round(view.separation)} m` : "--";
    card.alt.textContent = Number.isFinite(view.alt) ? `${Math.round(view.alt)} m` : "--";
    card.speed.textContent = Number.isFinite(view.speed) ? `${formatNumber(view.speed, 1)} m/s` : "--";
    card.capture.textContent =
      Number.isFinite(metrics.widthMeters) && Number.isFinite(metrics.heightMeters)
        ? `${Math.round(metrics.widthMeters)} x ${Math.round(metrics.heightMeters)} m`
        : "No footprint";
  };

  const refreshCards = () => {
    latestViews.forEach((view) => {
      const card = cards.get(view.agent);
      if (!card) {
        return;
      }
      updateCardMeta(card, view);
      if (panel.classList.contains("is-open")) {
        updateFeedMap(card);
      }
    });
  };

  const scheduleRefresh = () => {
    const delay = Math.max(0, RENDER_INTERVAL_MS - (Date.now() - lastRenderMs));
    if (!delay) {
      if (renderTimer) {
        clearTimeout(renderTimer);
        renderTimer = null;
      }
      lastRenderMs = Date.now();
      refreshCards();
      return;
    }
    if (renderTimer) {
      return;
    }
    renderTimer = window.setTimeout(() => {
      renderTimer = null;
      lastRenderMs = Date.now();
      refreshCards();
    }, delay);
  };

  const setOpen = (open) => {
    const next = Boolean(open);
    panel.classList.toggle("is-open", next);
    panel.setAttribute("aria-hidden", next ? "false" : "true");
    toggle.setAttribute("aria-expanded", next ? "true" : "false");
    toggle.classList.toggle("is-open", next);
    toggle.textContent = next ? "<" : ">";
    if (next) {
      ensureFeedMaps();
      refreshCards();
      cards.forEach((card) => {
        card.sourceMap?.resize();
      });
    }
  };

  toggle.addEventListener("click", (event) => {
    event.stopPropagation();
    setOpen(!panel.classList.contains("is-open"));
  });

  document.addEventListener("click", (event) => {
    const target = event.target;
    if (!panel.contains(target) && target !== toggle) {
      setOpen(false);
    }
  });

  if (typeof subscribeFilmingViews === "function") {
    subscribeFilmingViews((views) => {
      latestViews = UAVS.map(
        (agent) =>
          views.find((item) => item?.agent === agent) || { agent, status: "offline", footprint: null },
      );
      scheduleRefresh();
    });
  } else if (typeof getFilmingViews === "function") {
    latestViews = getFilmingViews();
  }

  if (typeof subscribeTargets === "function") {
    subscribeTargets((targets) => {
      latestTargets = Array.isArray(targets) ? targets : [];
      if (panel.classList.contains("is-open")) {
        scheduleRefresh();
      }
    });
  } else if (typeof getTargets === "function") {
    latestTargets = Array.isArray(getTargets()) ? getTargets() : [];
  }

  const resizeTargets = [leftControls, panel].filter(Boolean);
  if (typeof ResizeObserver !== "undefined" && resizeTargets.length) {
    const observer = new ResizeObserver(() => {
      syncDocking();
      if (panel.classList.contains("is-open")) {
        cards.forEach((card) => {
          card.sourceMap?.resize();
        });
      }
    });
    resizeTargets.forEach((target) => observer.observe(target));
  }

  window.addEventListener("resize", () => {
    syncDocking();
    if (panel.classList.contains("is-open")) {
      cards.forEach((card) => {
        card.sourceMap?.resize();
      });
      scheduleRefresh();
    }
  });

  syncDocking();
  refreshCards();
  setOpen(panel.classList.contains("is-open"));
};
