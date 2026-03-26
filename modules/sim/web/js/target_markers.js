const TARGET_SOURCE_ID = "enemy-targets";
const TARGET_SHADOW_LAYER_ID = "enemy-targets-shadow";
const TARGET_CORE_LAYER_ID = "enemy-targets-core";
const TARGET_CAP_LAYER_ID = "enemy-targets-cap";
const TARGET_TRAIL_LAYER_ID = "enemy-targets-trail";
const TARGET_GLOW_LAYER_ID = "enemy-targets-glow";
const TARGET_RING_LAYER_ID = "enemy-targets-ring";
const TARGET_MODEL_LAYER_ID = "enemy-targets-model";
const TARGET_OUTLINE_LAYER_ID = "enemy-targets-outline";
const TARGET_LABEL_LAYER_ID = "enemy-targets-label";

const TARGET_RENDER_FPS = 24;
const TARGET_SAMPLE_DURATION_MS = 220;
const RECENT_FIRE_WINDOW_S = 1.8;

const TYPE_LABELS = {
  1: "\uC804\uCC28",
  2: "\uC7A5\uAC11\uCC28",
  3: "\uBC29\uC0AC\uD3EC",
  4: "\uACE1\uC0AC\uD3EC",
  5: "\uACE0\uC815\uACE0\uC0AC\uD3EC",
  6: "\uAD70\uC778",
};

const TYPE_CONFIG = {
  1: {
    label: TYPE_LABELS[1],
    shadowSize: 8.6,
    palette: {
      body: "#627f36",
      top: "#91b953",
      accent: "#d9c26b",
      track: "#2f3b1e",
      outline: "#eef3bf",
      detect: "#ffd46f",
      fire: "#ff8d63",
      label: "#f2f6de",
    },
    parts: [
      { shape: "rect", forward: 0, lateral: 0, length: 23, width: 13.4, height: 1.65, base: 0.0, color: "body" },
      { shape: "rect", forward: 0, lateral: 0, length: 10.5, width: 8.2, height: 3.2, base: 1.2, color: "top" },
      { shape: "rect", forward: 13.6, lateral: 0, length: 13.2, width: 1.8, height: 3.4, base: 2.35, color: "accent" },
      { shape: "rect", forward: 0, lateral: 5.8, length: 22.5, width: 2.15, height: 0.8, base: 0.05, color: "track" },
      { shape: "rect", forward: 0, lateral: -5.8, length: 22.5, width: 2.15, height: 0.8, base: 0.05, color: "track" },
    ],
  },
  2: {
    label: TYPE_LABELS[2],
    shadowSize: 8.2,
    palette: {
      body: "#436f78",
      top: "#69a0ac",
      accent: "#b8d9cf",
      track: "#253c42",
      outline: "#dcf2ee",
      detect: "#ffd980",
      fire: "#ff9967",
      label: "#eef5e4",
    },
    parts: [
      { shape: "rect", forward: 0, lateral: 0, length: 21.5, width: 12.8, height: 1.5, base: 0.0, color: "body" },
      { shape: "rect", forward: 2.0, lateral: 0, length: 11.0, width: 10.0, height: 2.7, base: 1.0, color: "top" },
      { shape: "rect", forward: 11.5, lateral: 0, length: 7.2, width: 2.0, height: 2.3, base: 2.1, color: "accent" },
      { shape: "circle", forward: -2.4, lateral: 0, radius: 2.5, sides: 8, height: 3.1, base: 1.25, color: "accent" },
    ],
  },
  3: {
    label: TYPE_LABELS[3],
    shadowSize: 8.3,
    palette: {
      body: "#835437",
      top: "#b17a49",
      accent: "#efb26c",
      track: "#44281c",
      outline: "#f6d6ab",
      detect: "#ffd17f",
      fire: "#ff8f55",
      label: "#f5ebdd",
    },
    parts: [
      { shape: "rect", forward: -2.0, lateral: 0, length: 20.0, width: 11.8, height: 1.35, base: 0.0, color: "body" },
      { shape: "rect", forward: -7.8, lateral: 0, length: 6.2, width: 8.2, height: 2.5, base: 1.0, color: "top" },
      { shape: "rect", forward: 4.8, lateral: 0, length: 11.5, width: 7.0, height: 3.0, base: 1.35, color: "accent" },
      { shape: "rect", forward: 10.8, lateral: 2.1, length: 8.2, width: 1.2, height: 3.15, base: 1.55, color: "accent" },
      { shape: "rect", forward: 10.8, lateral: -2.1, length: 8.2, width: 1.2, height: 3.15, base: 1.55, color: "accent" },
    ],
  },
  4: {
    label: TYPE_LABELS[4],
    shadowSize: 7.6,
    palette: {
      body: "#7c6941",
      top: "#b3935e",
      accent: "#e1c184",
      track: "#46361e",
      outline: "#f3dfb1",
      detect: "#ffd48f",
      fire: "#ff9a62",
      label: "#f4e7d7",
    },
    parts: [
      { shape: "rect", forward: -2.4, lateral: 0, length: 12.8, width: 10.8, height: 0.9, base: 0.0, color: "body" },
      { shape: "rect", forward: 5.2, lateral: 0, length: 18.0, width: 1.55, height: 1.85, base: 0.85, color: "accent" },
      { shape: "rect", forward: -6.4, lateral: 4.4, length: 7.2, width: 1.0, height: 0.72, base: 0.05, color: "track" },
      { shape: "rect", forward: -6.4, lateral: -4.4, length: 7.2, width: 1.0, height: 0.72, base: 0.05, color: "track" },
      { shape: "circle", forward: 0.0, lateral: 0.0, radius: 2.25, sides: 8, height: 1.6, base: 0.5, color: "top" },
    ],
  },
  5: {
    label: TYPE_LABELS[5],
    shadowSize: 8.0,
    palette: {
      body: "#456d8d",
      top: "#70a3c5",
      accent: "#c3e1f3",
      track: "#20384a",
      outline: "#dcefff",
      detect: "#ffd885",
      fire: "#ff85b4",
      label: "#edf7f3",
    },
    parts: [
      { shape: "circle", forward: 0, lateral: 0, radius: 5.8, sides: 10, height: 1.45, base: 0.0, color: "body" },
      { shape: "rect", forward: 3.0, lateral: 4.8, length: 9.4, width: 2.7, height: 3.0, base: 1.15, color: "top" },
      { shape: "rect", forward: 3.0, lateral: -4.8, length: 9.4, width: 2.7, height: 3.0, base: 1.15, color: "top" },
      { shape: "rect", forward: -4.4, lateral: 0, length: 3.2, width: 2.0, height: 3.8, base: 1.2, color: "accent" },
      { shape: "rect", forward: -7.6, lateral: 0, length: 5.2, width: 0.9, height: 4.8, base: 1.2, color: "accent" },
    ],
  },
  6: {
    label: TYPE_LABELS[6],
    shadowSize: 6.4,
    palette: {
      body: "#8b7338",
      top: "#c7a860",
      accent: "#edd08d",
      track: "#5a4820",
      outline: "#f7e6b7",
      detect: "#ffd99b",
      fire: "#ff9f69",
      label: "#fbf2dd",
    },
    parts: [
      { shape: "circle", forward: 1.8, lateral: 0.0, radius: 1.15, sides: 8, height: 1.95, base: 0.0, color: "top" },
      { shape: "circle", forward: -1.3, lateral: 1.45, radius: 0.95, sides: 8, height: 1.6, base: 0.0, color: "body" },
      { shape: "circle", forward: -1.3, lateral: -1.45, radius: 0.95, sides: 8, height: 1.6, base: 0.0, color: "body" },
      { shape: "rect", forward: 4.8, lateral: 0.0, length: 4.0, width: 0.55, height: 1.6, base: 1.15, color: "accent" },
    ],
  },
};

const clamp = (value, min, max) => Math.min(max, Math.max(min, value));
const nowMs = () =>
  typeof performance !== "undefined" && typeof performance.now === "function"
    ? performance.now()
    : Date.now();

const metersToLonLat = (lon, lat, dx, dy) => {
  const metersPerDegLat = 111320.0;
  const cosLat = Math.cos((lat * Math.PI) / 180) || 1e-6;
  const metersPerDegLon = metersPerDegLat * cosLat;
  return {
    lon: lon + dx / metersPerDegLon,
    lat: lat + dy / metersPerDegLat,
  };
};

const localToLonLat = (lon, lat, headingDeg, forwardM, lateralM) => {
  const rad = (headingDeg * Math.PI) / 180;
  const dx = forwardM * Math.cos(rad) - lateralM * Math.sin(rad);
  const dy = forwardM * Math.sin(rad) + lateralM * Math.cos(rad);
  return metersToLonLat(lon, lat, dx, dy);
};

const wrapHeading = (value) => {
  let next = Number(value) || 0;
  while (next < 0) {
    next += 360;
  }
  while (next >= 360) {
    next -= 360;
  }
  return next;
};

const lerp = (from, to, alpha) => from + (to - from) * alpha;

const lerpAngle = (from, to, alpha) => {
  const start = wrapHeading(from);
  const end = wrapHeading(to);
  let diff = end - start;
  if (diff > 180) {
    diff -= 360;
  } else if (diff < -180) {
    diff += 360;
  }
  return wrapHeading(start + diff * alpha);
};

const easeOutCubic = (value) => 1 - Math.pow(1 - clamp(value, 0, 1), 3);

const toDeadColor = (hex) => {
  const cleaned = String(hex || "").replace("#", "");
  if (cleaned.length !== 6) {
    return "#5d615d";
  }
  const r = parseInt(cleaned.slice(0, 2), 16);
  const g = parseInt(cleaned.slice(2, 4), 16);
  const b = parseInt(cleaned.slice(4, 6), 16);
  const gray = Math.round(r * 0.25 + g * 0.5 + b * 0.25);
  const mixed = Math.round(gray * 0.72);
  const channel = clamp(mixed, 50, 140).toString(16).padStart(2, "0");
  return `#${channel}${channel}${channel}`;
};

const lightenColor = (hex, ratio = 0.2) => {
  const cleaned = String(hex || "").replace("#", "");
  if (cleaned.length !== 6) {
    return hex;
  }
  const next = [0, 2, 4]
    .map((offset) => {
      const channel = parseInt(cleaned.slice(offset, offset + 2), 16);
      const mixed = Math.round(channel + (255 - channel) * clamp(ratio, 0, 1));
      return clamp(mixed, 0, 255).toString(16).padStart(2, "0");
    })
    .join("");
  return `#${next}`;
};

const buildRectRing = (lon, lat, heading, forward, lateral, length, width) => {
  const halfLength = length * 0.5;
  const halfWidth = width * 0.5;
  const corners = [
    [forward + halfLength, lateral + halfWidth],
    [forward + halfLength, lateral - halfWidth],
    [forward - halfLength, lateral - halfWidth],
    [forward - halfLength, lateral + halfWidth],
  ];
  const ring = corners.map(([fwd, latOffset]) => {
    const point = localToLonLat(lon, lat, heading, fwd, latOffset);
    return [point.lon, point.lat];
  });
  ring.push(ring[0]);
  return ring;
};

const buildCircleRing = (lon, lat, heading, forward, lateral, radius, sides = 8) => {
  const total = Math.max(6, Math.trunc(sides) || 8);
  const ring = [];
  for (let index = 0; index < total; index += 1) {
    const theta = (Math.PI * 2 * index) / total;
    const fwd = forward + Math.cos(theta) * radius;
    const latOffset = lateral + Math.sin(theta) * radius;
    const point = localToLonLat(lon, lat, heading, fwd, latOffset);
    ring.push([point.lon, point.lat]);
  }
  ring.push(ring[0]);
  return ring;
};

const normalizeTarget = (raw) => {
  const lat = Number(raw?.lat);
  const lon = Number(raw?.lon);
  if (!Number.isFinite(lat) || !Number.isFinite(lon)) {
    return null;
  }
  const id = Number(raw?.id);
  if (!Number.isFinite(id)) {
    return null;
  }
  const typeId = Number(raw?.type);
  const config = TYPE_CONFIG[typeId] || TYPE_CONFIG[1];
  const label = raw?.name
    ? String(raw.name)
    : config?.label
      ? `${config.label}_${id}`
      : `T${id}`;
  return {
    id: Math.trunc(id),
    type: Number.isFinite(typeId) ? Math.trunc(typeId) : 1,
    name: label,
    lat,
    lon,
    alt: Number.isFinite(Number(raw?.alt)) ? Number(raw.alt) : 0,
    moving: Boolean(raw?.moving),
    alive: raw?.alive !== false,
    heading: wrapHeading(Number(raw?.heading) || 0),
    headingRate: Number.isFinite(Number(raw?.headingRate)) ? Number(raw.headingRate) : 0,
    speed: Number.isFinite(Number(raw?.speed)) ? Number(raw.speed) : 0,
    speedMin: Number.isFinite(Number(raw?.speedMin)) ? Number(raw.speedMin) : 0,
    speedMax: Number.isFinite(Number(raw?.speedMax)) ? Number(raw.speedMax) : 0,
    roamRadius: Number.isFinite(Number(raw?.roamRadius)) ? Number(raw.roamRadius) : 0,
    detected: Boolean(raw?.detected),
    exposureTime: Number.isFinite(Number(raw?.exposureTime)) ? Number(raw.exposureTime) : 0,
    ammo:
      raw?.ammo === null || raw?.ammo === undefined || raw?.ammo === ""
        ? null
        : Number.isFinite(Number(raw?.ammo))
          ? Math.trunc(Number(raw.ammo))
          : null,
    weaponKind: raw?.weaponKind ? String(raw.weaponKind) : "gun",
    weaponRange: Number.isFinite(Number(raw?.weaponRange)) ? Number(raw.weaponRange) : 0,
    reload: Number.isFinite(Number(raw?.reload)) ? Number(raw.reload) : 0,
    lastFireAge:
      raw?.lastFireAge === null || raw?.lastFireAge === undefined || raw?.lastFireAge === ""
        ? null
        : Number.isFinite(Number(raw?.lastFireAge))
          ? Number(raw.lastFireAge)
          : null,
  };
};

const makeTargetMap = (targets) => {
  const map = new Map();
  targets.forEach((target) => {
    if (!target) {
      return;
    }
    map.set(target.id, target);
  });
  return map;
};

const resolveConfig = (target) => TYPE_CONFIG[target?.type] || TYPE_CONFIG[1];

const buildInterpolatedTargets = (fromTargets, toTargets, sampleStart, sampleDuration, now) => {
  if (!toTargets.size) {
    return [];
  }
  const elapsedMs = Math.max(0, now - sampleStart);
  const mix = sampleDuration > 0 ? easeOutCubic(elapsedMs / sampleDuration) : 1;
  const result = [];
  toTargets.forEach((target, id) => {
    const previous = fromTargets.get(id);
    const recentFireAge =
      target.lastFireAge === null ? null : target.lastFireAge + elapsedMs / 1000;
    const exposureTime = target.detected ? target.exposureTime + elapsedMs / 1000 : target.exposureTime;
    if (!previous) {
      result.push({
        ...target,
        lastFireAge: recentFireAge,
        exposureTime,
      });
      return;
    }
    result.push({
      ...target,
      lat: lerp(previous.lat, target.lat, mix),
      lon: lerp(previous.lon, target.lon, mix),
      alt: lerp(previous.alt, target.alt, mix),
      heading: lerpAngle(previous.heading, target.heading, mix),
      headingRate: lerp(previous.headingRate, target.headingRate, mix),
      speed: lerp(previous.speed, target.speed, mix),
      lastFireAge: recentFireAge,
      exposureTime,
    });
  });
  return result;
};

const buildTargetFeatures = (targets, timeSeconds) => {
  const features = [];
  targets.forEach((target) => {
    const config = resolveConfig(target);
    const palette = config.palette;
    const alive = target.alive !== false;
    const heading = wrapHeading(target.heading);
    const pulse = 0.5 + 0.5 * Math.sin(timeSeconds * 3.8 + target.id * 0.71);
    const speedMax = Math.max(target.speedMax || 0, target.speed || 0, 1);
    const speedRatio = clamp(target.speed / speedMax, 0, 1);
    const moveLevel = alive && target.moving ? speedRatio : 0;
    const recentFireLevel =
      target.lastFireAge === null ? 0 : clamp(1 - target.lastFireAge / RECENT_FIRE_WINDOW_S, 0, 1);
    const detectLevel = target.detected ? clamp(0.42 + pulse * 0.26, 0.2, 0.72) : 0;
    const alertLevel = Math.max(detectLevel, recentFireLevel * 0.95);
    const modelOpacity = alive ? 0.98 : 0.76;
    const heightScale = alive ? 1.45 : 0.42;
    const outlineColor = alive ? palette.outline : "#858b88";
    const fireColor = target.weaponKind === "missile" ? palette.fire : "#ffb06b";
    const alertColor = recentFireLevel > detectLevel * 0.9 ? fireColor : palette.detect;
    const shadowSize = config.shadowSize * (alive ? 1 : 0.8);
    const labelColor = alive ? palette.label : "#b3b7b3";
    const markerColor = alive ? palette.body : "#666b67";
    const markerCapColor = alive ? lightenColor(palette.top, 0.16) : "#a4a8a4";
    const markerStrokeColor = alive ? palette.outline : "#c6cbc6";

    features.push({
      type: "Feature",
      properties: {
        featureKind: "center",
        id: target.id,
        type: target.type,
        name: target.name,
        color: labelColor,
        shadowSize,
        markerColor,
        markerCapColor,
        markerStrokeColor,
        glowColor: alertColor,
        glowOpacity: clamp(alertLevel * 0.34, 0, 0.44),
        ringOpacity: clamp(Math.max(detectLevel * 0.8, recentFireLevel), 0, 0.95),
        ringSize: shadowSize * (1.9 + pulse * 0.34 + recentFireLevel * 0.9),
        statusSize: shadowSize * (1.22 + recentFireLevel * 0.22),
      },
      geometry: {
        type: "Point",
        coordinates: [target.lon, target.lat],
      },
    });

    if (moveLevel > 0.04) {
      const trailLength = 14 + moveLevel * 34 + Math.abs(target.headingRate) * 0.08;
      const tail = localToLonLat(target.lon, target.lat, heading, -trailLength, 0);
      const mid = localToLonLat(target.lon, target.lat, heading, -trailLength * 0.45, 0);
      features.push({
        type: "Feature",
        properties: {
          featureKind: "trail",
          color: recentFireLevel > 0.35 ? fireColor : palette.accent,
          opacity: clamp(0.18 + moveLevel * 0.36, 0.16, 0.56),
          width: clamp(1.4 + moveLevel * 2.6, 1.4, 3.8),
        },
        geometry: {
          type: "LineString",
          coordinates: [
            [tail.lon, tail.lat],
            [mid.lon, mid.lat],
            [target.lon, target.lat],
          ],
        },
      });
    }

    config.parts.forEach((part, index) => {
      const baseColor = palette[part.color] || palette.body;
      const color = alive ? baseColor : toDeadColor(baseColor);
      const ring =
        part.shape === "circle"
          ? buildCircleRing(
              target.lon,
              target.lat,
              heading,
              part.forward,
              part.lateral,
              part.radius,
              part.sides,
            )
          : buildRectRing(
              target.lon,
              target.lat,
              heading,
              part.forward,
              part.lateral,
              part.length,
              part.width,
            );
      features.push({
        type: "Feature",
        properties: {
          featureKind: "model",
          id: target.id,
          partIndex: index,
          color,
          opacity: modelOpacity,
          height: Math.max(0.2, part.height * heightScale),
          base: Math.max(0, part.base * (alive ? 1 : 0.55)),
        },
        geometry: {
          type: "Polygon",
          coordinates: [ring],
        },
      });
      features.push({
        type: "Feature",
        properties: {
          featureKind: "outline",
          color: outlineColor,
          opacity: clamp(0.26 + alertLevel * 0.22, 0.22, 0.54),
        },
        geometry: {
          type: "LineString",
          coordinates: ring,
        },
      });
    });
  });
  return features;
};

export const initTargetMarkers = (map) => {
  let pendingPayload = null;
  let mapReady = typeof map.isStyleLoaded === "function" ? map.isStyleLoaded() : map.loaded();
  let fromTargets = new Map();
  let toTargets = new Map();
  let latestTargets = [];
  const targetSubscribers = new Set();
  let sampleStartMs = 0;
  let sampleDurationMs = TARGET_SAMPLE_DURATION_MS;
  let animationFrameId = null;
  let lastFrameMs = 0;

  const ensureLayer = () => {
    if (!map.getSource(TARGET_SOURCE_ID)) {
      map.addSource(TARGET_SOURCE_ID, {
        type: "geojson",
        data: { type: "FeatureCollection", features: [] },
      });
    }
    if (!map.getLayer(TARGET_SHADOW_LAYER_ID)) {
      map.addLayer({
        id: TARGET_SHADOW_LAYER_ID,
        type: "circle",
        source: TARGET_SOURCE_ID,
        filter: ["==", ["get", "featureKind"], "center"],
        paint: {
          "circle-color": "rgba(5, 8, 6, 0.78)",
          "circle-radius": [
            "interpolate",
            ["linear"],
            ["zoom"],
            8,
            ["*", ["get", "shadowSize"], 0.9],
            10,
            ["*", ["get", "shadowSize"], 1.0],
            13,
            ["*", ["get", "shadowSize"], 1.22],
            16,
            ["*", ["get", "shadowSize"], 1.7],
          ],
          "circle-opacity": 0.28,
          "circle-blur": 0.7,
        },
      });
    }
    if (!map.getLayer(TARGET_CORE_LAYER_ID)) {
      map.addLayer({
        id: TARGET_CORE_LAYER_ID,
        type: "circle",
        source: TARGET_SOURCE_ID,
        filter: ["==", ["get", "featureKind"], "center"],
        paint: {
          "circle-color": ["get", "markerColor"],
          "circle-opacity": 0.96,
          "circle-stroke-color": ["get", "markerStrokeColor"],
          "circle-stroke-opacity": 0.96,
          "circle-stroke-width": [
            "interpolate",
            ["linear"],
            ["zoom"],
            8,
            1.3,
            12,
            1.9,
            16,
            2.4,
          ],
          "circle-radius": [
            "interpolate",
            ["linear"],
            ["zoom"],
            8,
            ["*", ["get", "statusSize"], 0.82],
            10,
            ["*", ["get", "statusSize"], 0.92],
            13,
            ["*", ["get", "statusSize"], 1.0],
            16,
            ["*", ["get", "statusSize"], 1.14],
          ],
        },
      });
    }
    if (!map.getLayer(TARGET_CAP_LAYER_ID)) {
      map.addLayer({
        id: TARGET_CAP_LAYER_ID,
        type: "circle",
        source: TARGET_SOURCE_ID,
        filter: ["==", ["get", "featureKind"], "center"],
        paint: {
          "circle-color": ["get", "markerCapColor"],
          "circle-opacity": 0.92,
          "circle-radius": [
            "interpolate",
            ["linear"],
            ["zoom"],
            8,
            ["*", ["get", "statusSize"], 0.42],
            12,
            ["*", ["get", "statusSize"], 0.5],
            16,
            ["*", ["get", "statusSize"], 0.56],
          ],
        },
      });
    }
    if (!map.getLayer(TARGET_TRAIL_LAYER_ID)) {
      map.addLayer({
        id: TARGET_TRAIL_LAYER_ID,
        type: "line",
        source: TARGET_SOURCE_ID,
        filter: ["==", ["get", "featureKind"], "trail"],
        layout: {
          "line-cap": "round",
          "line-join": "round",
        },
        paint: {
          "line-color": ["get", "color"],
          "line-opacity": ["get", "opacity"],
          "line-width": [
            "interpolate",
            ["linear"],
            ["zoom"],
            10,
            ["*", ["get", "width"], 0.8],
            15,
            ["*", ["get", "width"], 1.45],
          ],
          "line-blur": 0.45,
        },
      });
    }
    if (!map.getLayer(TARGET_GLOW_LAYER_ID)) {
      map.addLayer({
        id: TARGET_GLOW_LAYER_ID,
        type: "circle",
        source: TARGET_SOURCE_ID,
        filter: ["==", ["get", "featureKind"], "center"],
        paint: {
          "circle-color": ["get", "glowColor"],
          "circle-opacity": ["get", "glowOpacity"],
          "circle-radius": [
            "interpolate",
            ["linear"],
            ["zoom"],
            8,
            ["*", ["get", "ringSize"], 0.95],
            10,
            ["*", ["get", "ringSize"], 0.8],
            14,
            ["*", ["get", "ringSize"], 1.25],
            16,
            ["*", ["get", "ringSize"], 1.75],
          ],
          "circle-blur": 0.68,
        },
      });
    }
    if (!map.getLayer(TARGET_RING_LAYER_ID)) {
      map.addLayer({
        id: TARGET_RING_LAYER_ID,
        type: "circle",
        source: TARGET_SOURCE_ID,
        filter: ["==", ["get", "featureKind"], "center"],
        paint: {
          "circle-color": "rgba(0,0,0,0)",
          "circle-opacity": 0,
          "circle-stroke-color": ["get", "glowColor"],
          "circle-stroke-opacity": ["get", "ringOpacity"],
          "circle-stroke-width": [
            "interpolate",
            ["linear"],
            ["zoom"],
            8,
            1.6,
            10,
            1.4,
            14,
            2.2,
            16,
            2.8,
          ],
          "circle-radius": [
            "interpolate",
            ["linear"],
            ["zoom"],
            8,
            ["*", ["get", "statusSize"], 1.04],
            10,
            ["*", ["get", "statusSize"], 0.9],
            14,
            ["*", ["get", "statusSize"], 1.35],
            16,
            ["*", ["get", "statusSize"], 1.9],
          ],
        },
      });
    }
    if (!map.getLayer(TARGET_MODEL_LAYER_ID)) {
      map.addLayer({
        id: TARGET_MODEL_LAYER_ID,
        type: "fill-extrusion",
        source: TARGET_SOURCE_ID,
        filter: ["==", ["get", "featureKind"], "model"],
        paint: {
          "fill-extrusion-color": ["get", "color"],
          "fill-extrusion-base": ["get", "base"],
          "fill-extrusion-height": [
            "+",
            ["get", "base"],
            ["*", ["get", "height"], 1.18],
          ],
          // MapLibre in this build rejects data-driven fill-extrusion opacity.
          "fill-extrusion-opacity": 0.94,
          "fill-extrusion-vertical-gradient": true,
        },
      });
    }
    if (!map.getLayer(TARGET_OUTLINE_LAYER_ID)) {
      map.addLayer({
        id: TARGET_OUTLINE_LAYER_ID,
        type: "line",
        source: TARGET_SOURCE_ID,
        filter: ["==", ["get", "featureKind"], "outline"],
        layout: {
          "line-cap": "round",
          "line-join": "round",
        },
        paint: {
          "line-color": ["get", "color"],
          "line-opacity": ["get", "opacity"],
          "line-width": [
            "interpolate",
            ["linear"],
            ["zoom"],
            8,
            1.0,
            10,
            0.9,
            14,
            1.5,
            16,
            2.1,
          ],
        },
      });
    }
    if (!map.getLayer(TARGET_LABEL_LAYER_ID)) {
      map.addLayer({
        id: TARGET_LABEL_LAYER_ID,
        type: "symbol",
        source: TARGET_SOURCE_ID,
        filter: ["==", ["get", "featureKind"], "center"],
        layout: {
          "text-field": ["get", "name"],
          "text-size": [
            "interpolate",
            ["linear"],
            ["zoom"],
            8,
            11.4,
            10,
            11.8,
            15,
            13.1,
          ],
          "text-offset": [0, 1.95],
          "text-anchor": "top",
          "text-allow-overlap": true,
          "text-ignore-placement": true,
        },
        paint: {
          "text-color": ["get", "color"],
          "text-halo-color": "rgba(4, 8, 6, 0.8)",
          "text-halo-width": 1.4,
          "text-opacity": 0.94,
        },
      });
    }
  };

  const applyFeatures = (targets, frameTimeMs) => {
    ensureLayer();
    const source = map.getSource(TARGET_SOURCE_ID);
    if (!source) {
      return;
    }
    const features = buildTargetFeatures(targets, frameTimeMs / 1000);
    source.setData({ type: "FeatureCollection", features });
  };

  const renderFrame = (frameTimeMs) => {
    animationFrameId = null;
    if (!mapReady) {
      return;
    }
    if (frameTimeMs - lastFrameMs < 1000 / TARGET_RENDER_FPS) {
      animationFrameId = requestAnimationFrame(renderFrame);
      return;
    }
    const renderedTargets = buildInterpolatedTargets(
      fromTargets,
      toTargets,
      sampleStartMs,
      sampleDurationMs,
      frameTimeMs,
    );
    applyFeatures(renderedTargets, frameTimeMs);
    lastFrameMs = frameTimeMs;

    const interpolationActive = frameTimeMs - sampleStartMs < sampleDurationMs;
    const dynamicActive = renderedTargets.some((target) => {
      const fireAge = target.lastFireAge;
      return (
        (target.alive !== false && target.moving && Math.abs(target.speed) > 0.05) ||
        target.detected ||
        (Number.isFinite(fireAge) && fireAge < RECENT_FIRE_WINDOW_S)
      );
    });
    if (interpolationActive || dynamicActive) {
      animationFrameId = requestAnimationFrame(renderFrame);
    }
  };

  const requestRender = () => {
    if (animationFrameId !== null) {
      return;
    }
    animationFrameId = requestAnimationFrame(renderFrame);
  };

  const notifySubscribers = () => {
    const snapshot = latestTargets.map((target) => ({ ...target }));
    targetSubscribers.forEach((listener) => {
      try {
        listener(snapshot);
      } catch (error) {
        console.warn("target_markers subscriber failed", error);
      }
    });
  };

  const loadFromReference = (payload) => {
    pendingPayload = payload;
    if (!mapReady) {
      return;
    }
    const normalizedTargets = Array.isArray(payload?.targets)
      ? payload.targets.map(normalizeTarget).filter(Boolean)
      : [];
    latestTargets = normalizedTargets;
    notifySubscribers();
    const timestampMs = nowMs();
    const currentTargets = buildInterpolatedTargets(
      fromTargets,
      toTargets,
      sampleStartMs,
      sampleDurationMs,
      timestampMs,
    );
    fromTargets = makeTargetMap(currentTargets);
    toTargets = makeTargetMap(normalizedTargets);
    sampleStartMs = timestampMs;
    sampleDurationMs = TARGET_SAMPLE_DURATION_MS;
    requestRender();
  };

  map.on("load", () => {
    mapReady = true;
    ensureLayer();
    if (pendingPayload) {
      loadFromReference(pendingPayload);
    } else {
      applyFeatures([], nowMs());
    }
  });

  if (typeof map.on === "function") {
    map.on("remove", () => {
      if (animationFrameId !== null) {
        cancelAnimationFrame(animationFrameId);
        animationFrameId = null;
      }
    });
  }

  const getTargets = () => latestTargets.map((target) => ({ ...target }));

  const subscribeTargets = (listener) => {
    if (typeof listener !== "function") {
      return () => {};
    }
    targetSubscribers.add(listener);
    try {
      listener(getTargets());
    } catch (error) {
      console.warn("target_markers initial subscriber push failed", error);
    }
    return () => {
      targetSubscribers.delete(listener);
    };
  };

  return { loadFromReference, getTargets, subscribeTargets };
};
