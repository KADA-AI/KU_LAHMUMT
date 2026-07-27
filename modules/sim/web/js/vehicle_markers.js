const AGENTS = ["LAH1", "LAH2", "LAH3", "UAV1", "UAV2", "UAV3"];
const LAH_IDS = new Set(["LAH1", "LAH2", "LAH3"]);
const SPHERE_LAYER_ID = "vehicle-spheres";
const FILMING_POINT_LAYER_ID = "filming-targets";
const FILMING_LINE_PREFIX = "filming-line-";
const FOOTPRINT_SOURCE_ID = "filming-footprints";
const FOOTPRINT_LAYER_ID = "filming-footprints-line";
const FOOTPRINT_FILL_LAYER_ID = "filming-footprints-fill";
const FOOTPRINT_TRAIL_SOURCE_ID = "filming-footprints-trail";
const FOOTPRINT_TRAIL_LAYER_ID = "filming-footprints-trail-fill";
const TRAIL_LINE_PREFIX = "vehicle-trail-";

const BASE_SIZE = 14;
const FILMING_POINT_SCALE = 0.65;
const FILMING_DASH_M = 120;
const FILMING_GAP_M = 80;
// The manned aircraft is drawn as a helicopter rather than a sphere: every
// waypoint on the map is also a sphere, so the two used to read the same.
// The silhouette needs the extra pixels to be legible at all.
const LAH_POINT_SCALE = 3.2;
const UAV_POINT_SCALE = 0.6;
// pos(3) + color(3) + size(1) + shape(1) + rotation(1)
const POINT_STRIDE_FLOATS = 9;
const SHAPE_SPHERE = 0;
const SHAPE_HELICOPTER = 1;
const FOOTPRINT_STEPS = 4;
const FOOTPRINT_MIN_RADIUS_M = 15;
const FOOTPRINT_MAX_RADIUS_M = 5000;
const FOOTPRINT_ASPECT = 16 / 9;
const FOOTPRINT_CURRENT_OPACITY = 0.2;
const FOOTPRINT_TRAIL_MAX = 240;
const FOOTPRINT_TRAIL_OPACITY = 0.12;
const FOOTPRINT_TRAIL_SAMPLE_STEP = 2;
const TRAIL_MAX_METERS = 1400;
const TRAIL_MIN_SEGMENT_M = 4;
const TRAIL_WIDTH = 2.0;
const TRAIL_Z_OFFSET_M = 0.8;
const EXCEEDED_SEP_COLOR = "#ef4444";
const LAH_MIN_TERRAIN_CLEARANCE_M = 50;
// 시각화 프레임레이트 — 데이터 생성(0401 5Hz)과 별개로 클라이언트 보간만 돌린다.
// Authoritative samples and visual updates both run at 5 Hz. Rendering faster
// only rebuilds the same terrain-facing buffers between authoritative states.
const VISUAL_UPDATE_HZ = 5;
const VISUAL_UPDATE_INTERVAL_MS = 1000 / VISUAL_UPDATE_HZ;
const FOOTPRINT_VISUAL_SAMPLE_INTERVAL_MS = Math.max(VISUAL_UPDATE_INTERVAL_MS, 220);
// 사이드패널 등 DOM 구독자 알림은 프레임레이트보다 낮게 유지 (렌더 지터 방지)
const FILMING_VIEW_NOTIFY_INTERVAL_MS = 200;

const getAgentColors = () => {
  const style = getComputedStyle(document.documentElement);
  return {
    LAH1: style.getPropertyValue("--lah1").trim() || "#d97845",
    LAH2: style.getPropertyValue("--lah2").trim() || "#e0b24a",
    LAH3: style.getPropertyValue("--lah3").trim() || "#98a96a",
    UAV1: style.getPropertyValue("--uav1").trim() || "#3d7ca6",
    UAV2: style.getPropertyValue("--uav2").trim() || "#4aa39c",
    UAV3: style.getPropertyValue("--uav3").trim() || "#6b7fa1",
  };
};

const hexToRgb = (hex) => {
  const cleaned = String(hex || "").replace("#", "");
  const full =
    cleaned.length === 3
      ? cleaned
          .split("")
          .map((char) => char + char)
          .join("")
      : cleaned;
  const value = Number.parseInt(full, 16);
  if (!Number.isFinite(value)) {
    return [1, 1, 1];
  }
  return [((value >> 16) & 255) / 255, ((value >> 8) & 255) / 255, (value & 255) / 255];
};

const normalizeMatrix = (matrix) => {
  if (!matrix) {
    return null;
  }
  if (matrix instanceof Float32Array) {
    return matrix;
  }
  if (typeof matrix.toFloat32Array === "function") {
    const arr = matrix.toFloat32Array();
    return arr instanceof Float32Array ? arr : arr ? new Float32Array(arr) : null;
  }
  if (typeof matrix.toFloat64Array === "function") {
    const arr = matrix.toFloat64Array();
    return arr ? new Float32Array(arr) : null;
  }
  if (ArrayBuffer.isView(matrix) && typeof matrix.length === "number") {
    return new Float32Array(matrix);
  }
  if (Array.isArray(matrix)) {
    return new Float32Array(matrix);
  }
  if (typeof matrix.toArray === "function") {
    const arr = matrix.toArray();
    return Array.isArray(arr) ? new Float32Array(arr) : null;
  }
  if (matrix.elements instanceof Float32Array) {
    return matrix.elements;
  }
  if (Array.isArray(matrix.elements)) {
    return new Float32Array(matrix.elements);
  }
  if (matrix.elements && typeof matrix.elements.length === "number") {
    return new Float32Array(matrix.elements);
  }
  return null;
};

const getProjectionMatrix = (argsOrMatrix) => {
  const direct = normalizeMatrix(argsOrMatrix);
  if (direct && direct.length === 16) {
    return direct;
  }
  if (argsOrMatrix && typeof argsOrMatrix === "object") {
    const proj = argsOrMatrix.defaultProjectionData;
    if (proj) {
      const main = normalizeMatrix(proj.mainMatrix);
      if (main && main.length === 16) {
        return main;
      }
      const fallback = normalizeMatrix(proj.fallbackMatrix);
      if (fallback && fallback.length === 16) {
        return fallback;
      }
    }
  }
  return null;
};

const projectToScreen = (matrix, x, y, z, width, height) => {
  const mx = matrix[0] * x + matrix[4] * y + matrix[8] * z + matrix[12];
  const my = matrix[1] * x + matrix[5] * y + matrix[9] * z + matrix[13];
  const mw = matrix[3] * x + matrix[7] * y + matrix[11] * z + matrix[15];
  if (!Number.isFinite(mw) || mw <= 0) {
    return null;
  }
  const nx = mx / mw;
  const ny = my / mw;
  if (nx < -1.2 || nx > 1.2 || ny < -1.2 || ny > 1.2) {
    return null;
  }
  return {
    x: (nx * 0.5 + 0.5) * width,
    y: (1 - (ny * 0.5 + 0.5)) * height,
  };
};

const metersToLonLat = (lon, lat, dx, dy) => {
  const metersPerDegLat = 111320.0;
  const cosLat = Math.cos((lat * Math.PI) / 180) || 1e-6;
  const metersPerDegLon = metersPerDegLat * cosLat;
  return {
    lon: lon + dx / metersPerDegLon,
    lat: lat + dy / metersPerDegLat,
  };
};

const distMeters = (a, b) => {
  if (!a || !b) {
    return 0;
  }
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const dz = b.z - a.z;
  const m2u = (a.m2u + b.m2u) * 0.5 || 1e-6;
  return Math.hypot(dx, dy, dz) / m2u;
};

const groundDistanceMeters = (origin, point) => {
  const local = llToLocalMeters(origin, point);
  if (!local) {
    return null;
  }
  return Math.hypot(local.x, local.y);
};

const llToLocalMeters = (origin, point) => {
  if (!origin || !point) {
    return null;
  }
  const lat0 = Number(origin.lat ?? origin.latitude);
  const lon0 = Number(origin.lon ?? origin.longitude);
  const lat = Number(point.lat ?? point.latitude);
  const lon = Number(point.lon ?? point.longitude);
  if (![lat0, lon0, lat, lon].every(Number.isFinite)) {
    return null;
  }
  const metersPerDegLat = 111320.0;
  const cosLat = Math.cos((lat0 * Math.PI) / 180) || 1e-6;
  const metersPerDegLon = metersPerDegLat * cosLat;
  return {
    x: (lon - lon0) * metersPerDegLon,
    y: (lat - lat0) * metersPerDegLat,
    metersPerDegLat,
    metersPerDegLon,
    originLat: lat0,
    originLon: lon0,
  };
};

const localMetersToLl = (origin, point) => {
  const lat0 = Number(origin.lat ?? origin.latitude);
  const lon0 = Number(origin.lon ?? origin.longitude);
  if (![lat0, lon0, point?.x, point?.y].every(Number.isFinite)) {
    return null;
  }
  const metersPerDegLat = 111320.0;
  const cosLat = Math.cos((lat0 * Math.PI) / 180) || 1e-6;
  const metersPerDegLon = metersPerDegLat * cosLat;
  return {
    lon: lon0 + point.x / metersPerDegLon,
    lat: lat0 + point.y / metersPerDegLat,
  };
};

const footprintSelfIntersects = (points) => {
  if (!Array.isArray(points) || points.length < 4) {
    return false;
  }
  const orient = (a, b, c) => ((b.x - a.x) * (c.y - a.y)) - ((b.y - a.y) * (c.x - a.x));
  const intersects = (a1, a2, b1, b2) => {
    const o1 = orient(a1, a2, b1);
    const o2 = orient(a1, a2, b2);
    const o3 = orient(b1, b2, a1);
    const o4 = orient(b1, b2, a2);
    return (o1 * o2) < 0 && (o3 * o4) < 0;
  };
  return intersects(points[0], points[1], points[2], points[3])
    || intersects(points[1], points[2], points[3], points[0]);
};

const normalizeFootprintRing = (ring) => {
  if (!Array.isArray(ring)) {
    return null;
  }
  let points = ring
    .map((coord) => {
      if (!Array.isArray(coord) || coord.length < 2) {
        return null;
      }
      const lon = Number(coord[0]);
      const lat = Number(coord[1]);
      if (!Number.isFinite(lon) || !Number.isFinite(lat)) {
        return null;
      }
      return [lon, lat];
    })
    .filter(Boolean);
  if (points.length >= 2) {
    const first = points[0];
    const last = points[points.length - 1];
    if (first[0] === last[0] && first[1] === last[1]) {
      points = points.slice(0, -1);
    }
  }
  points = points.slice(0, 4);
  if (points.length < 4) {
    return points.length ? [...points, [...points[0]]] : null;
  }

  const rows = points.map(([x, y]) => ({ x, y }));
  const topBottom = (() => {
    const sorted = [...rows].sort((left, right) => {
      if (right.y !== left.y) {
        return right.y - left.y;
      }
      return left.x - right.x;
    });
    const top = sorted.slice(0, 2).sort((left, right) => left.x - right.x);
    const bottom = sorted.slice(2).sort((left, right) => right.x - left.x);
    return [...top, ...bottom];
  })();

  const clockwise = (() => {
    const centerX = rows.reduce((sum, row) => sum + row.x, 0) / rows.length;
    const centerY = rows.reduce((sum, row) => sum + row.y, 0) / rows.length;
    const sorted = [...rows].sort(
      (left, right) => Math.atan2(right.y - centerY, right.x - centerX) - Math.atan2(left.y - centerY, left.x - centerX),
    );
    let startIdx = 0;
    for (let idx = 1; idx < sorted.length; idx += 1) {
      const current = sorted[idx];
      const best = sorted[startIdx];
      if (current.y > best.y || (current.y === best.y && current.x < best.x)) {
        startIdx = idx;
      }
    }
    const rotated = [...sorted.slice(startIdx), ...sorted.slice(0, startIdx)];
    if (rotated.length >= 4 && rotated[1].x < rotated[rotated.length - 1].x) {
      return [rotated[0], ...rotated.slice(1).reverse()];
    }
    return rotated;
  })();

  let ordered = topBottom;
  if (footprintSelfIntersects(ordered)) {
    ordered = clockwise;
  }
  if (footprintSelfIntersects(ordered)) {
    ordered = [ordered[0], ordered[1], ordered[3], ordered[2]];
  }
  const normalized = ordered.map((row) => [row.x, row.y]);
  normalized.push([...normalized[0]]);
  return normalized;
};

const closeOrderedFootprintRing = (ring) => {
  if (!Array.isArray(ring) || ring.length < 4) {
    return null;
  }
  const points = ring.slice(0, 4).map(([lon, lat]) => ({ x: lon, y: lat }));
  if (footprintSelfIntersects(points)) {
    return normalizeFootprintRing(ring);
  }
  const normalized = ring.slice(0, 4).map(([lon, lat]) => [lon, lat]);
  normalized.push([...normalized[0]]);
  return normalized;
};

const closeDemFootprintBoundary = (ring) => {
  if (!Array.isArray(ring)) {
    return null;
  }
  let points = ring
    .map((coord) => {
      if (!Array.isArray(coord) || coord.length < 2) {
        return null;
      }
      const lon = Number(coord[0]);
      const lat = Number(coord[1]);
      return Number.isFinite(lon) && Number.isFinite(lat) ? [lon, lat] : null;
    })
    .filter(Boolean);
  if (points.length < 4) {
    return null;
  }
  const first = points[0];
  const last = points[points.length - 1];
  if (first[0] === last[0] && first[1] === last[1]) {
    points = points.slice(0, -1);
  }
  if (points.length < 4) {
    return null;
  }
  points.push([...points[0]]);
  return points;
};

const estimateSweepWidthMeters = (center, corners, tangent) => {
  if (!center || !Array.isArray(corners) || corners.length < 4 || !tangent) {
    return null;
  }
  const normal = { x: -tangent.y, y: tangent.x };
  let minProj = Infinity;
  let maxProj = -Infinity;
  corners.forEach((coord) => {
    const local = llToLocalMeters(center, { lat: coord[1], lon: coord[0] });
    if (!local) {
      return;
    }
    const proj = local.x * normal.x + local.y * normal.y;
    minProj = Math.min(minProj, proj);
    maxProj = Math.max(maxProj, proj);
  });
  if (!Number.isFinite(minProj) || !Number.isFinite(maxProj)) {
    return null;
  }
  return Math.max(1, maxProj - minProj);
};

const buildSweepCoveragePolygon = (prevCenter, nextCenter, widthMeters) => {
  if (!prevCenter || !nextCenter || !Number.isFinite(widthMeters) || widthMeters <= 0) {
    return null;
  }
  const start = llToLocalMeters(prevCenter, prevCenter);
  const end = llToLocalMeters(prevCenter, nextCenter);
  if (!start || !end) {
    return null;
  }
  const dx = end.x - start.x;
  const dy = end.y - start.y;
  const len = Math.hypot(dx, dy);
  if (!Number.isFinite(len) || len < 0.5) {
    return null;
  }
  const tangent = { x: dx / len, y: dy / len };
  const normal = { x: -tangent.y, y: tangent.x };
  const half = widthMeters * 0.5;
  const ringLocal = [
    { x: start.x + normal.x * half, y: start.y + normal.y * half },
    { x: end.x + normal.x * half, y: end.y + normal.y * half },
    { x: end.x - normal.x * half, y: end.y - normal.y * half },
    { x: start.x - normal.x * half, y: start.y - normal.y * half },
  ];
  const ring = ringLocal.map((pt) => {
    const ll = localMetersToLl(prevCenter, pt);
    return ll ? [ll.lon, ll.lat] : null;
  }).filter(Boolean);
  if (ring.length < 4) {
    return null;
  }
  return normalizeFootprintRing(ring);
};

const buildFootprintGeometry = (entry) => {
  const explicitBoundary = Array.isArray(entry?.footprintBoundary) ? entry.footprintBoundary : null;
  if (explicitBoundary && explicitBoundary.length >= 4) {
    const boundaryRing = closeDemFootprintBoundary(explicitBoundary);
    if (boundaryRing) {
      return boundaryRing;
    }
  }
  const explicitCorners = Array.isArray(entry?.footprintCorners) ? entry.footprintCorners : null;
  if (explicitCorners && explicitCorners.length >= 4) {
    const ring = explicitCorners
      .map((coord) => {
        if (!Array.isArray(coord) || coord.length < 2) {
          return null;
        }
        const lon = Number(coord[0]);
        const lat = Number(coord[1]);
        return Number.isFinite(lon) && Number.isFinite(lat) ? [lon, lat] : null;
      })
      .filter(Boolean);
    if (ring.length >= 4) {
      return closeOrderedFootprintRing(ring);
    }
  }
  const target = entry?.filmingTarget;
  if (!target || !Number.isFinite(target.lat) || !Number.isFinite(target.lon)) {
    return null;
  }
  const fov = Number(entry.filmingFov);
  if (!Number.isFinite(fov) || fov <= 0) {
    return null;
  }
  const uavLat = Number(entry.lat);
  const uavLon = Number(entry.lon);
  if (!Number.isFinite(uavLat) || !Number.isFinite(uavLon)) {
    return null;
  }
  const targetAlt = Number.isFinite(target.alt) ? target.alt : 0;
  const uavAlt = Number.isFinite(entry.alt) ? entry.alt : 0;

  const metersPerDegLat = 111320.0;
  const cosLat = Math.cos((target.lat * Math.PI) / 180) || 1e-6;
  const metersPerDegLon = metersPerDegLat * cosLat;

  const origin = {
    x: (uavLon - target.lon) * metersPerDegLon,
    y: (uavLat - target.lat) * metersPerDegLat,
    z: uavAlt - targetAlt,
  };
  const forward = {
    x: -origin.x,
    y: -origin.y,
    z: -origin.z,
  };
  const fwdLen = Math.hypot(forward.x, forward.y, forward.z);
  if (!Number.isFinite(fwdLen) || fwdLen <= 1e-6) {
    return null;
  }
  forward.x /= fwdLen;
  forward.y /= fwdLen;
  forward.z /= fwdLen;

  let right = {
    x: forward.y,
    y: -forward.x,
    z: 0,
  };
  let rightLen = Math.hypot(right.x, right.y, right.z);
  if (rightLen < 1e-6) {
    right = { x: 1, y: 0, z: 0 };
    rightLen = 1;
  }
  right.x /= rightLen;
  right.y /= rightLen;
  right.z /= rightLen;
  const up = {
    x: right.y * forward.z - right.z * forward.y,
    y: right.z * forward.x - right.x * forward.z,
    z: right.x * forward.y - right.y * forward.x,
  };
  const upLen = Math.hypot(up.x, up.y, up.z);
  if (upLen > 1e-6) {
    up.x /= upLen;
    up.y /= upLen;
    up.z /= upLen;
  }

  const fovDiag = (fov * Math.PI) / 180;
  const ar = FOOTPRINT_ASPECT;
  const fovH = 2 * Math.atan((Math.tan(fovDiag / 2) * ar) / Math.sqrt(1 + ar * ar));
  const fovV = 2 * Math.atan((Math.tan(fovDiag / 2) * 1) / Math.sqrt(1 + ar * ar));
  const tanH = Math.tan(fovH / 2);
  const tanV = Math.tan(fovV / 2);

  const corners = [];
  const dirs = [
    { sx: -1, sy: 1 },
    { sx: 1, sy: 1 },
    { sx: 1, sy: -1 },
    { sx: -1, sy: -1 },
  ];
  dirs.forEach(({ sx, sy }) => {
    const dir = {
      x: forward.x + sx * tanH * right.x + sy * tanV * up.x,
      y: forward.y + sx * tanH * right.y + sy * tanV * up.y,
      z: forward.z + sx * tanH * right.z + sy * tanV * up.z,
    };
    const dirLen = Math.hypot(dir.x, dir.y, dir.z);
    if (dirLen <= 1e-6) {
      return;
    }
    dir.x /= dirLen;
    dir.y /= dirLen;
    dir.z /= dirLen;
    if (Math.abs(dir.z) < 1e-6) {
      return;
    }
    const t = (0 - origin.z) / dir.z;
    if (!Number.isFinite(t) || t <= 0) {
      return;
    }
    corners.push({
      x: origin.x + dir.x * t,
      y: origin.y + dir.y * t,
    });
  });
  if (corners.length !== 4) {
    return null;
  }

  const coords = [];
  corners.forEach((corner) => {
    const pt = metersToLonLat(target.lon, target.lat, corner.x, corner.y);
    coords.push([pt.lon, pt.lat]);
  });
  coords.push(coords[0]);
  return coords;
};

export const buildDashedSegments = (start, end, dashLenM, gapLenM) => {
  if (!start || !end) {
    return [];
  }
  const dx = end.x - start.x;
  const dy = end.y - start.y;
  const dz = end.z - start.z;
  const m2u = start.meterInMercatorCoordinateUnits
    ? start.meterInMercatorCoordinateUnits()
    : 0;
  if (!Number.isFinite(m2u) || m2u <= 0) {
    return [start.x, start.y, start.z, end.x, end.y, end.z];
  }
  const lenM = Math.hypot(dx / m2u, dy / m2u, dz / m2u);
  if (!Number.isFinite(lenM) || lenM <= 0) {
    return [];
  }
  if (lenM <= dashLenM) {
    return [start.x, start.y, start.z, end.x, end.y, end.z];
  }
  const step = dashLenM + gapLenM;
  const segments = [];
  for (let dist = 0; dist < lenM; dist += step) {
    const segLen = Math.min(dashLenM, lenM - dist);
    const t0 = dist / lenM;
    const t1 = (dist + segLen) / lenM;
    const sx = start.x + dx * t0;
    const sy = start.y + dy * t0;
    const sz = start.z + dz * t0;
    const ex = start.x + dx * t1;
    const ey = start.y + dy * t1;
    const ez = start.z + dz * t1;
    segments.push(sx, sy, sz, ex, ey, ez);
  }
  return segments;
};

// Top-down helicopter silhouette in point-sprite space, uv in [-0.5, 0.5] with
// the nose at -y before rotation. Shared verbatim by the WebGL1 and WebGL2
// fragment shaders so the two can never drift apart.
const HELICOPTER_GLSL = `
  float boxMask(vec2 p, vec2 halfSize, float feather) {
    vec2 d = abs(p) - halfSize;
    float outside = length(max(d, 0.0));
    float inside = min(max(d.x, d.y), 0.0);
    return 1.0 - smoothstep(0.0, feather, outside + inside);
  }

  float ellipseMask(vec2 p, vec2 radii, float feather) {
    float d = length(p / radii) - 1.0;
    return 1.0 - smoothstep(0.0, feather / min(radii.x, radii.y), d);
  }

  vec3 rgb2hsv(vec3 c) {
    vec4 K = vec4(0.0, -1.0 / 3.0, 2.0 / 3.0, -1.0);
    vec4 p = mix(vec4(c.bg, K.wz), vec4(c.gb, K.xy), step(c.b, c.g));
    vec4 q = mix(vec4(p.xyw, c.r), vec4(c.r, p.yzx), step(p.x, c.r));
    float d = q.x - min(q.w, q.y);
    float e = 1.0e-10;
    return vec3(abs(q.z + (q.w - q.y) / (6.0 * d + e)), d / (q.x + e), q.x);
  }

  vec3 hsv2rgb(vec3 c) {
    vec4 K = vec4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);
    vec3 p = abs(fract(c.xxx + K.xyz) * 6.0 - K.www);
    return c.z * mix(K.xxx, clamp(p - K.xxx, 0.0, 1.0), c.y);
  }

  // The aircraft and its own waypoints share one agent colour, so the live
  // aircraft is lifted out of its own trail. Saturation and value move; hue is
  // carried through untouched, which keeps it the same colour family.
  // Done in HSV on purpose - scaling RGB about the luma axis clips the top
  // channel on the warmer agents and drags the hue several degrees with it.
  vec3 emphasise(vec3 c) {
    vec3 hsv = rgb2hsv(c);
    hsv.y = clamp(hsv.y * 1.34, 0.0, 1.0);
    hsv.z = clamp(hsv.z * 1.16, 0.0, 1.0);
    return hsv2rgb(hsv);
  }

  vec4 helicopterColor(vec2 uv, vec3 baseTint, float rotation) {
    vec3 tint = emphasise(baseTint);
    float c = cos(rotation);
    float s = sin(rotation);
    vec2 p = vec2(c * uv.x - s * uv.y, s * uv.x + c * uv.y);
    float aa = 0.035;

    // Rotor disc: a faint hairline ring with two thin blades. It must stay
    // light - when it carries real weight the sprite reads as a circle and the
    // fuselage that actually shows the heading disappears inside it.
    float rotorR = length(p);
    float ring = smoothstep(0.47, 0.45, rotorR) * smoothstep(0.425, 0.445, rotorR);
    vec2 bladeB = vec2(p.x * 0.7071 + p.y * 0.7071, -p.x * 0.7071 + p.y * 0.7071);
    float blades = max(
      boxMask(p, vec2(0.014, 0.46), aa),
      boxMask(bladeB, vec2(0.014, 0.46), aa)
    ) * smoothstep(0.47, 0.45, rotorR);
    float rotor = max(ring, blades);

    // Fuselage, nose forward at -y: a pointed nose, the cabin, then a tail
    // boom with its stabiliser. This is the part that shows which way it flies.
    float cabin = ellipseMask(p - vec2(0.0, -0.17), vec2(0.120, 0.155), aa);
    float nose = ellipseMask(p - vec2(0.0, -0.29), vec2(0.070, 0.075), aa);
    float boom = boxMask(p - vec2(0.0, 0.19), vec2(0.026, 0.23), aa);
    float stabiliser = boxMask(p - vec2(0.0, 0.385), vec2(0.105, 0.026), aa);
    float tailRotor = boxMask(p - vec2(0.0, 0.425), vec2(0.021, 0.070), aa);
    float body = max(max(cabin, nose), max(max(boom, stabiliser), tailRotor));

    float alpha = clamp(max(body, rotor * 0.42), 0.0, 1.0);
    if (alpha < 0.02) {
      discard;
    }

    // Light the fuselage so it reads as a solid object; keep the rotor flat and
    // pale so it stays visible against dark terrain without competing.
    float lift = clamp(1.0 - length(p - vec2(0.0, -0.14)) * 1.7, 0.0, 1.0);
    vec3 solid = tint * (0.68 + 0.52 * lift);
    vec3 rotorTint = mix(tint, vec3(1.0), 0.55);
    vec3 color = mix(rotorTint, solid, clamp(body, 0.0, 1.0));
    // Darken the feathered rim so the silhouette keeps an edge on light ground.
    color *= mix(0.40, 1.0, smoothstep(0.0, 0.5, body));
    return vec4(color, alpha * 0.98);
  }

  vec4 sphereColor(vec2 uv, vec3 tint) {
    float r = length(uv);
    if (r > 0.5) {
      discard;
    }
    float z = sqrt(max(0.0, 0.25 - r * r)) / 0.5;
    vec3 normal = normalize(vec3(uv / 0.5, z));
    vec3 lightDir = normalize(vec3(-0.3, -0.2, 0.93));
    float diffuse = clamp(dot(normal, lightDir), 0.0, 1.0);
    float rim = smoothstep(0.3, 0.5, r);
    return vec4(tint * (0.55 + 0.45 * diffuse) + rim * 0.2, 0.98);
  }
`;

const createSphereLayer = (id) => {
  const layer = {
    id,
    type: "custom",
    renderingMode: "3d",
    _visible: true,
    _useDepth: false,
    _pointCount: 0,
    _pendingPoints: null,
    _maxPointSize: 0,
    setVisible(nextVisible) {
      this._visible = Boolean(nextVisible);
    },
    updatePoints(points) {
      this._pendingPoints = points;
      if (!this._gl || !this._buffer) {
        return;
      }
      const gl = this._gl;
      const data = new Float32Array(points);
      // gl_PointSize above the driver's range is implementation-defined. Most
      // desktop drivers allow 1024, but some cap far lower, so clamp rather
      // than leave the sprite size up to the GPU.
      const maxPointSize = this._maxPointSize;
      if (maxPointSize > 0) {
        for (let i = 6; i < data.length; i += POINT_STRIDE_FLOATS) {
          if (data[i] > maxPointSize) {
            data[i] = maxPointSize;
          }
        }
      }
      gl.bindBuffer(gl.ARRAY_BUFFER, this._buffer);
      gl.bufferData(gl.ARRAY_BUFFER, data, gl.DYNAMIC_DRAW);
      this._pointCount = data.length / POINT_STRIDE_FLOATS;
      this._pendingPoints = null;
    },
    onAdd(_map, gl) {
      this._gl = gl;
      try {
        const range = gl.getParameter(gl.ALIASED_POINT_SIZE_RANGE);
        const limit = Number(range && range[1]);
        this._maxPointSize = Number.isFinite(limit) && limit > 0 ? limit : 0;
      } catch {
        this._maxPointSize = 0;
      }
      const isWebGL2 =
        typeof WebGL2RenderingContext !== "undefined" &&
        gl instanceof WebGL2RenderingContext;
      const vertexSource = isWebGL2
        ? `#version 300 es
          in vec3 a_pos;
          in vec3 a_color;
          in float a_size;
          in float a_shape;
          in float a_rotation;
          uniform mat4 u_matrix;
          out vec3 v_color;
          out float v_shape;
          out float v_rotation;
          void main() {
            gl_Position = u_matrix * vec4(a_pos, 1.0);
            gl_PointSize = a_size;
            v_color = a_color;
            v_shape = a_shape;
            v_rotation = a_rotation;
          }`
        : `
          attribute vec3 a_pos;
          attribute vec3 a_color;
          attribute float a_size;
          attribute float a_shape;
          attribute float a_rotation;
          uniform mat4 u_matrix;
          varying vec3 v_color;
          varying float v_shape;
          varying float v_rotation;
          void main() {
            gl_Position = u_matrix * vec4(a_pos, 1.0);
            gl_PointSize = a_size;
            v_color = a_color;
            v_shape = a_shape;
            v_rotation = a_rotation;
          }`;
      const fragmentSource = isWebGL2
        ? `#version 300 es
          precision mediump float;
          in vec3 v_color;
          in float v_shape;
          in float v_rotation;
          out vec4 fragColor;
          ${HELICOPTER_GLSL}
          void main() {
            vec2 uv = gl_PointCoord - 0.5;
            fragColor = v_shape > 0.5
              ? helicopterColor(uv, v_color, v_rotation)
              : sphereColor(uv, v_color);
          }`
        : `
          precision mediump float;
          varying vec3 v_color;
          varying float v_shape;
          varying float v_rotation;
          ${HELICOPTER_GLSL}
          void main() {
            vec2 uv = gl_PointCoord - 0.5;
            gl_FragColor = v_shape > 0.5
              ? helicopterColor(uv, v_color, v_rotation)
              : sphereColor(uv, v_color);
          }`;
      const compile = (type, source) => {
        const shader = gl.createShader(type);
        gl.shaderSource(shader, source);
        gl.compileShader(shader);
        if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
          const log = gl.getShaderInfoLog(shader) || "shader compile failed";
          gl.deleteShader(shader);
          throw new Error(log);
        }
        return shader;
      };
      const linkProgram = (vs, fs) => {
        const program = gl.createProgram();
        gl.attachShader(program, vs);
        gl.attachShader(program, fs);
        gl.linkProgram(program);
        gl.deleteShader(vs);
        gl.deleteShader(fs);
        if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
          const log = gl.getProgramInfoLog(program) || "program link failed";
          gl.deleteProgram(program);
          throw new Error(log);
        }
        return program;
      };
      try {
        const vertexShader = compile(gl.VERTEX_SHADER, vertexSource);
        const fragmentShader = compile(gl.FRAGMENT_SHADER, fragmentSource);
        this._program = linkProgram(vertexShader, fragmentShader);
      } catch (err) {
        console.error(`[${id}] shader/program error:`, err);
        this._program = null;
        this._visible = false;
        return;
      }
      const program = this._program;
      this._aPos = gl.getAttribLocation(program, "a_pos");
      this._aColor = gl.getAttribLocation(program, "a_color");
      this._aSize = gl.getAttribLocation(program, "a_size");
      this._aShape = gl.getAttribLocation(program, "a_shape");
      this._aRotation = gl.getAttribLocation(program, "a_rotation");
      this._uMatrix = gl.getUniformLocation(program, "u_matrix");
      this._buffer = gl.createBuffer();
      gl.bindBuffer(gl.ARRAY_BUFFER, this._buffer);
      gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([]), gl.DYNAMIC_DRAW);
      this._pointCount = 0;
      if (this._pendingPoints) {
        this.updatePoints(this._pendingPoints);
      }
    },
    render(gl, argsOrMatrix) {
      if (!this._program || !this._visible || !this._pointCount) {
        return;
      }
      const mat = getProjectionMatrix(argsOrMatrix);
      if (!mat || mat.length !== 16) {
        return;
      }
      const depthWasEnabled = gl.isEnabled(gl.DEPTH_TEST);
      const cullWasEnabled = gl.isEnabled(gl.CULL_FACE);
      if (!this._useDepth) {
        gl.disable(gl.DEPTH_TEST);
      }
      if (cullWasEnabled) {
        gl.disable(gl.CULL_FACE);
      }
      gl.useProgram(this._program);
      gl.uniformMatrix4fv(this._uMatrix, false, mat);
      gl.bindBuffer(gl.ARRAY_BUFFER, this._buffer);
      const stride = POINT_STRIDE_FLOATS * 4;
      // A driver may optimise an attribute away and hand back -1; enabling that
      // raises INVALID_VALUE and kills the whole layer.
      const bindAttribute = (location, componentCount, offsetFloats) => {
        if (location === undefined || location === null || location < 0) {
          return;
        }
        gl.enableVertexAttribArray(location);
        gl.vertexAttribPointer(
          location,
          componentCount,
          gl.FLOAT,
          false,
          stride,
          offsetFloats * 4,
        );
      };
      bindAttribute(this._aPos, 3, 0);
      bindAttribute(this._aColor, 3, 3);
      bindAttribute(this._aSize, 1, 6);
      bindAttribute(this._aShape, 1, 7);
      bindAttribute(this._aRotation, 1, 8);
      gl.enable(gl.BLEND);
      gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
      gl.drawArrays(gl.POINTS, 0, this._pointCount);
      if (typeof this._renderHook === "function") {
        this._renderHook(mat);
      }
      if (cullWasEnabled) {
        gl.enable(gl.CULL_FACE);
      }
      if (!this._useDepth && depthWasEnabled) {
        gl.enable(gl.DEPTH_TEST);
      }
    },
  };
  return layer;
};

export const createLineLayer = (id, color) => {
  const layer = {
    id,
    type: "custom",
    renderingMode: "3d",
    _color: color,
    _visible: true,
    _useDepth: false,
    _lineCount: 0,
    _pendingPositions: null,
    setColor(nextColor) {
      if (typeof nextColor !== "string" || !nextColor.trim()) {
        return;
      }
      this._color = nextColor.trim();
    },
    setVisible(nextVisible) {
      this._visible = Boolean(nextVisible);
    },
    updatePositions(positions) {
      this._pendingPositions = positions;
      if (!this._gl || !this._lineBuffer) {
        return;
      }
      const gl = this._gl;
      const data = new Float32Array(positions);
      gl.bindBuffer(gl.ARRAY_BUFFER, this._lineBuffer);
      gl.bufferData(gl.ARRAY_BUFFER, data, gl.DYNAMIC_DRAW);
      this._lineCount = data.length / 3;
      this._pendingPositions = null;
    },
    onAdd(_map, gl) {
      this._gl = gl;
      const isWebGL2 =
        typeof WebGL2RenderingContext !== "undefined" &&
        gl instanceof WebGL2RenderingContext;
      const vertexSource = isWebGL2
        ? `#version 300 es
          in vec3 a_pos;
          uniform mat4 u_matrix;
          void main() {
            gl_Position = u_matrix * vec4(a_pos, 1.0);
          }`
        : `
          attribute vec3 a_pos;
          uniform mat4 u_matrix;
          void main() {
            gl_Position = u_matrix * vec4(a_pos, 1.0);
          }`;
      const fragmentSource = isWebGL2
        ? `#version 300 es
          precision mediump float;
          uniform vec4 u_color;
          out vec4 fragColor;
          void main() {
            fragColor = u_color;
          }`
        : `
          precision mediump float;
          uniform vec4 u_color;
          void main() {
            gl_FragColor = u_color;
          }`;
      const compile = (type, source) => {
        const shader = gl.createShader(type);
        gl.shaderSource(shader, source);
        gl.compileShader(shader);
        if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
          const log = gl.getShaderInfoLog(shader) || "shader compile failed";
          gl.deleteShader(shader);
          throw new Error(log);
        }
        return shader;
      };
      const linkProgram = (vs, fs) => {
        const program = gl.createProgram();
        gl.attachShader(program, vs);
        gl.attachShader(program, fs);
        gl.linkProgram(program);
        gl.deleteShader(vs);
        gl.deleteShader(fs);
        if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
          const log = gl.getProgramInfoLog(program) || "program link failed";
          gl.deleteProgram(program);
          throw new Error(log);
        }
        return program;
      };
      try {
        const vertexShader = compile(gl.VERTEX_SHADER, vertexSource);
        const fragmentShader = compile(gl.FRAGMENT_SHADER, fragmentSource);
        this._program = linkProgram(vertexShader, fragmentShader);
      } catch (err) {
        console.error(`[${id}] shader/program error:`, err);
        this._program = null;
        this._visible = false;
        return;
      }
      const program = this._program;
      this._aPos = gl.getAttribLocation(program, "a_pos");
      this._uMatrix = gl.getUniformLocation(program, "u_matrix");
      this._uColor = gl.getUniformLocation(program, "u_color");
      this._lineBuffer = gl.createBuffer();
      gl.bindBuffer(gl.ARRAY_BUFFER, this._lineBuffer);
      gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([]), gl.DYNAMIC_DRAW);
      this._lineCount = 0;
      if (this._pendingPositions) {
        this.updatePositions(this._pendingPositions);
      }
    },
    render(gl, argsOrMatrix) {
      if (!this._program || !this._visible || !this._lineCount) {
        return;
      }
      const mat = getProjectionMatrix(argsOrMatrix);
      if (!mat || mat.length !== 16) {
        return;
      }
      const depthWasEnabled = gl.isEnabled(gl.DEPTH_TEST);
      const cullWasEnabled = gl.isEnabled(gl.CULL_FACE);
      if (!this._useDepth) {
        gl.disable(gl.DEPTH_TEST);
      }
      if (cullWasEnabled) {
        gl.disable(gl.CULL_FACE);
      }
      gl.useProgram(this._program);
      gl.uniformMatrix4fv(this._uMatrix, false, mat);
      const color = hexToRgb(this._color || "#ffffff");
      gl.uniform4f(this._uColor, color[0], color[1], color[2], 0.95);
      gl.bindBuffer(gl.ARRAY_BUFFER, this._lineBuffer);
      gl.enableVertexAttribArray(this._aPos);
      gl.vertexAttribPointer(this._aPos, 3, gl.FLOAT, false, 0, 0);
      gl.enable(gl.BLEND);
      gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
      gl.lineWidth(2);
      gl.drawArrays(gl.LINES, 0, this._lineCount);
      if (cullWasEnabled) {
        gl.enable(gl.CULL_FACE);
      }
      if (!this._useDepth && depthWasEnabled) {
        gl.enable(gl.DEPTH_TEST);
      }
    },
  };
  return layer;
};

export const initVehicleMarkers = (map) => {
  let pendingData = null;
  let mapReady = typeof map.isStyleLoaded === "function" ? map.isStyleLoaded() : map.loaded();
  let currentPositions = {};
  let sampleFromPositions = null;
  let sampleToPositions = null;
  let sampleStartMs = 0;
  let sampleDurationMs = 200;
  let lastAuthoritativeMs = 0;
  let animationFrameId = null;
  let lastVisualUpdateMs = 0;
  let sphereLayer = null;
  let filmingPointLayer = null;
  const filmingLineLayers = new Map();
  const trailLineLayers = new Map();
  let mercatorByAgent = new Map();
  let filmingMercatorByAgent = new Map();
  let filmingSepByAgent = new Map();
  let filmingMaxSepByAgent = new Map();
  let labelContainer = null;
  const labelElements = new Map();
  const sepLabelElements = new Map();
  let footprintSource = null;
  let footprintTrailSource = null;
  const footprintHistory = new Map();
  const trailHistory = new Map();
  const filmingViewSubscribers = new Set();
  let lastFootprintSampleMs = 0;
  let lastStep = null;
  let lastVehiclePayloadSignature = null;
  let lastProjectionMatrix = null;
  let sourceCommitTimer = null;
  let pendingFootprintData = null;
  let pendingFootprintTrailData = null;
  let repaintQueued = false;
  // 트레일(과거 풋프린트 도장)은 220ms 간격으로만 변하므로, 매 프레임 재구성하지
  // 않고 변경 시에만 GeoJSON을 다시 만든다 (30Hz 인상 시 GC/​setData 부하 방지).
  let footprintTrailDirty = true;
  let lastFilmingViewNotifyMs = 0;

  const clonePositions = (positions) => {
    if (!positions || typeof positions !== "object") {
      return {};
    }
    const cloned = {};
    Object.entries(positions).forEach(([agent, entry]) => {
      cloned[agent] = entry ? JSON.parse(JSON.stringify(entry)) : entry;
    });
    return cloned;
  };

  const clearScheduledSourceCommit = () => {
    if (sourceCommitTimer !== null) {
      clearTimeout(sourceCommitTimer);
      sourceCommitTimer = null;
    }
  };

  const flushScheduledSourceCommit = () => {
    sourceCommitTimer = null;
    if (!mapReady) {
      return;
    }
    ensureLayer();
    if (footprintSource && pendingFootprintData) {
      footprintSource.setData(pendingFootprintData);
    }
    if (footprintTrailSource && pendingFootprintTrailData) {
      footprintTrailSource.setData(pendingFootprintTrailData);
    }
    pendingFootprintData = null;
    pendingFootprintTrailData = null;
    if (repaintQueued) {
      repaintQueued = false;
      map.triggerRepaint();
    }
  };

  const scheduleSourceCommit = (footprintData, footprintTrailData, requestRepaint = false) => {
    pendingFootprintData = footprintData;
    if (footprintTrailData) {
      // null이면 트레일은 변경 없음 — 이전 pending을 덮어쓰지 않는다.
      pendingFootprintTrailData = footprintTrailData;
    }
    repaintQueued = repaintQueued || requestRepaint;
    if (sourceCommitTimer !== null) {
      return;
    }
    sourceCommitTimer = window.setTimeout(flushScheduledSourceCommit, 0);
  };

  const lerp = (a, b, t) => a + (b - a) * t;

  const lerpAngleDeg = (a, b, t) => {
    const start = Number(a) || 0;
    const end = Number(b) || 0;
    let delta = ((end - start + 540) % 360) - 180;
    return start + delta * t;
  };

  const interpolateCoord = (from, to, t) => {
    const src = from || to || null;
    const dst = to || from || null;
    if (!src || !dst) {
      return null;
    }
    const lat1 = Number(src.lat ?? src.latitude);
    const lon1 = Number(src.lon ?? src.longitude);
    const alt1 = Number(src.alt ?? src.altitude ?? 0);
    const lat2 = Number(dst.lat ?? dst.latitude);
    const lon2 = Number(dst.lon ?? dst.longitude);
    const alt2 = Number(dst.alt ?? dst.altitude ?? 0);
    if (![lat1, lon1, lat2, lon2].every(Number.isFinite)) {
      return dst;
    }
    const lat = lerp(lat1, lat2, t);
    const lon = lerp(lon1, lon2, t);
    const alt = Number.isFinite(alt1) && Number.isFinite(alt2) ? lerp(alt1, alt2, t) : alt2;
    if ("latitude" in dst || "longitude" in dst || "altitude" in dst) {
      return { latitude: lat, longitude: lon, altitude: alt };
    }
    return { lat, lon, alt };
  };

  const interpolateFootprintCorners = (from, to, t) => {
    const src = Array.isArray(from) && from.length >= 4 ? from : null;
    const dst = Array.isArray(to) && to.length >= 4 ? to : null;
    if (!src && !dst) {
      return null;
    }
    if (!src || !dst || src.length !== dst.length) {
      return JSON.parse(JSON.stringify(dst || src));
    }
    const corners = [];
    for (let idx = 0; idx < dst.length; idx += 1) {
      const a = src[idx];
      const b = dst[idx];
      if (!Array.isArray(a) || !Array.isArray(b) || a.length < 2 || b.length < 2) {
        return JSON.parse(JSON.stringify(dst));
      }
      const lon1 = Number(a[0]);
      const lat1 = Number(a[1]);
      const alt1 = Number(a[2] ?? 0);
      const lon2 = Number(b[0]);
      const lat2 = Number(b[1]);
      const alt2 = Number(b[2] ?? 0);
      if (![lon1, lat1, lon2, lat2].every(Number.isFinite)) {
        return JSON.parse(JSON.stringify(dst));
      }
      const corner = [lerp(lon1, lon2, t), lerp(lat1, lat2, t)];
      if (Number.isFinite(alt1) && Number.isFinite(alt2)) {
        corner.push(lerp(alt1, alt2, t));
      }
      corners.push(corner);
    }
    return corners;
  };

  const interpolateEntry = (fromEntry, toEntry, t) => {
    const next = toEntry ? JSON.parse(JSON.stringify(toEntry)) : null;
    if (!fromEntry && !next) {
      return null;
    }
    if (!fromEntry) {
      return next;
    }
    if (!next) {
      return JSON.parse(JSON.stringify(fromEntry));
    }
    if (Number.isFinite(fromEntry.lat) && Number.isFinite(fromEntry.lon) && Number.isFinite(next.lat) && Number.isFinite(next.lon)) {
      next.lat = lerp(fromEntry.lat, next.lat, t);
      next.lon = lerp(fromEntry.lon, next.lon, t);
    }
    if (Number.isFinite(fromEntry.alt) && Number.isFinite(next.alt)) {
      next.alt = lerp(fromEntry.alt, next.alt, t);
    }
    if (Number.isFinite(fromEntry.speed) && Number.isFinite(next.speed)) {
      next.speed = lerp(fromEntry.speed, next.speed, t);
    }
    if (Number.isFinite(fromEntry.heading) && Number.isFinite(next.heading)) {
      next.heading = lerpAngleDeg(fromEntry.heading, next.heading, t);
    }
    const filmingTarget = interpolateCoord(fromEntry.filmingTarget, next.filmingTarget, t);
    if (filmingTarget) {
      next.filmingTarget = filmingTarget;
    }
    const footprintCorners = interpolateFootprintCorners(fromEntry.footprintCorners, next.footprintCorners, t);
    if (footprintCorners) {
      next.footprintCorners = footprintCorners;
    }
    const footprintBoundary = interpolateFootprintCorners(fromEntry.footprintBoundary, next.footprintBoundary, t);
    if (footprintBoundary) {
      next.footprintBoundary = footprintBoundary;
    }
    const loiterCoordinate = interpolateCoord(fromEntry.loiterCoordinate, next.loiterCoordinate, t);
    if (loiterCoordinate) {
      next.loiterCoordinate = loiterCoordinate;
    }
    return next;
  };

  const interpolatePositions = (fromPositions, toPositions, t) => {
    const fromMap = fromPositions || {};
    const toMap = toPositions || {};
    const agents = new Set([...Object.keys(fromMap), ...Object.keys(toMap)]);
    const blended = {};
    agents.forEach((agent) => {
      const entry = interpolateEntry(fromMap[agent], toMap[agent], t);
      if (entry) {
        blended[agent] = entry;
      }
    });
    return blended;
  };

  const applyInterpolatedState = (positions, step) => {
    currentPositions = positions || {};
    if (Number.isFinite(step)) {
      lastStep = step;
    }
    ensureLayer();
    updateLayers();
    if (!currentPositions || !Object.keys(currentPositions).length) {
      labelElements.forEach((el) => {
        el.style.display = "none";
      });
    }
    // DOM 구독자(좌측 센서 패널 등)는 30Hz로 두드릴 필요가 없다 — 스로틀.
    const nowMs = performance.now();
    if (nowMs - lastFilmingViewNotifyMs < FILMING_VIEW_NOTIFY_INTERVAL_MS) {
      return;
    }
    lastFilmingViewNotifyMs = nowMs;
    filmingViewSubscribers.forEach((listener) => {
      try {
        listener(getFilmingViews());
      } catch (error) {
        console.warn("vehicle_markers filming view subscriber failed", error);
      }
    });
  };

  const getFilmingViews = () => {
    return ["UAV1", "UAV2", "UAV3"].map((agent) => {
      const entry = currentPositions[agent];
      if (!entry) {
        return {
          agent,
          status: "offline",
          footprint: null,
        };
      }
      const filming = entry.filmingTarget;
      const footprint = buildFootprintGeometry(entry);
      const separation = filming
        ? groundDistanceMeters(
            { lat: entry.lat, lon: entry.lon },
            { lat: filming.lat, lon: filming.lon },
          )
        : null;
      return {
        agent,
        status: footprint ? "active" : "idle",
        lat: Number.isFinite(entry.lat) ? entry.lat : null,
        lon: Number.isFinite(entry.lon) ? entry.lon : null,
        alt: Number.isFinite(entry.alt) ? entry.alt : null,
        speed: Number.isFinite(entry.speed) ? entry.speed : null,
        heading: Number.isFinite(entry.heading) ? entry.heading : null,
        filmingFov: Number.isFinite(entry.filmingFov) ? entry.filmingFov : null,
        filmingMaxSep: Number.isFinite(entry.filmingMaxSep) ? entry.filmingMaxSep : null,
        separation: Number.isFinite(separation) ? separation : null,
        filmingTarget:
          filming && Number.isFinite(filming.lat) && Number.isFinite(filming.lon)
            ? {
                lat: filming.lat,
                lon: filming.lon,
                alt: Number.isFinite(filming.alt) ? filming.alt : null,
              }
            : null,
        footprint,
      };
    });
  };

  const subscribeFilmingViews = (listener) => {
    if (typeof listener !== "function") {
      return () => {};
    }
    filmingViewSubscribers.add(listener);
    try {
      listener(getFilmingViews());
    } catch (error) {
      console.warn("vehicle_markers filming view initial push failed", error);
    }
    return () => {
      filmingViewSubscribers.delete(listener);
    };
  };

  const recordFootprintHistory = (positions, now, force = false) => {
    if (!positions || !Object.keys(positions).length) {
      return;
    }
    if (!force && (now - lastFootprintSampleMs) < FOOTPRINT_VISUAL_SAMPLE_INTERVAL_MS) {
      return;
    }
    lastFootprintSampleMs = now;
    AGENTS.forEach((agent) => {
      const entry = positions[agent];
      if (!entry) {
        return;
      }
      const coords = buildFootprintGeometry(entry);
      if (!coords) {
        return;
      }
      const list = footprintHistory.get(agent) || [];
      list.push({ coords });
      if (list.length > FOOTPRINT_TRAIL_MAX) {
        list.splice(0, list.length - FOOTPRINT_TRAIL_MAX);
      }
      footprintHistory.set(agent, list);
      footprintTrailDirty = true;
    });
  };

  const animateVisuals = (now) => {
    animationFrameId = null;
    if (!mapReady) {
      return;
    }
    if ((now - lastVisualUpdateMs) < VISUAL_UPDATE_INTERVAL_MS) {
      animationFrameId = requestAnimationFrame(animateVisuals);
      return;
    }
    lastVisualUpdateMs = now;
    if (!sampleToPositions) {
      if (currentPositions && Object.keys(currentPositions).length) {
        recordFootprintHistory(currentPositions, now, false);
        updateLayers();
      }
      // Static terrain must not be repainted forever. applyData() starts a new
      // animation when the next authoritative 5 Hz sample arrives.
      return;
    }
    const duration = Math.max(1, sampleDurationMs || 200);
    const alpha = Math.max(0, Math.min(1, (now - sampleStartMs) / duration));
    const interpolated = interpolatePositions(sampleFromPositions, sampleToPositions, alpha);
    recordFootprintHistory(interpolated, now, false);
    applyInterpolatedState(interpolated, lastStep);
    if (alpha >= 1) {
      sampleFromPositions = clonePositions(sampleToPositions);
      sampleToPositions = null;
      sampleStartMs = now;
      recordFootprintHistory(sampleFromPositions, now, true);
      applyInterpolatedState(clonePositions(sampleFromPositions), lastStep);
      return;
    }
    animationFrameId = requestAnimationFrame(animateVisuals);
  };

  // Point sprites are axis-aligned to the screen, so the nose has to be turned
  // by the compass heading *and* by however far the map itself is rotated.
  const spriteRotationRad = (headingDeg) => {
    const heading = Number.isFinite(headingDeg) ? Number(headingDeg) : 0;
    const bearing = typeof map.getBearing === "function" ? Number(map.getBearing()) : 0;
    const screenHeading = heading - (Number.isFinite(bearing) ? bearing : 0);
    return (-screenHeading * Math.PI) / 180;
  };

  // The footprint is a GeoJSON fill/line, so MapLibre drapes it on the map
  // surface: on the terrain when terrain is on, on the zero plane when it is
  // off. The centre marker is a real 3-D point, so putting it at the target's
  // MSL altitude leaves it floating above that surface - and perspective then
  // slides it off the footprint, worst of all looking straight down. Pin it to
  // the surface the footprint is actually drawn on.
  const drapedSurfaceAltitude = (lon, lat, fallbackAltitude) => {
    const terrainOn = typeof map.getTerrain === "function" && Boolean(map.getTerrain());
    if (!terrainOn) {
      return 0;
    }
    if (typeof map.queryTerrainElevation === "function") {
      try {
        const elevation = map.queryTerrainElevation([lon, lat]);
        if (Number.isFinite(elevation)) {
          return Number(elevation);
        }
      } catch {
        // Terrain tiles not resident yet; fall through.
      }
    }
    return Number.isFinite(fallbackAltitude) ? Number(fallbackAltitude) : 0;
  };

  const buildBuffers = (colors) => {
    const zoom = typeof map.getZoom === "function" ? map.getZoom() : 12;
    const pixelRatio = typeof map.getPixelRatio === "function" ? map.getPixelRatio() : 1;
    const scale = Math.min(2.2, Math.max(0.9, 0.7 + (zoom - 11) * 0.12));
    const points = [];
    const filmingPoints = [];
    const lineBuffers = new Map();
    const footprintLineFeatures = [];
    const footprintFillFeatures = [];
    const nextFilmingMercator = new Map();
    const nextFilmingSep = new Map();
    const nextFilmingMaxSep = new Map();
    AGENTS.forEach((agent) => lineBuffers.set(agent, []));
    const nextMercator = new Map();
    AGENTS.forEach((agent) => {
      const entry = currentPositions[agent];
      if (!entry || !Number.isFinite(entry.lat) || !Number.isFinite(entry.lon)) {
        return;
      }
      const alt = Number.isFinite(entry.alt) ? entry.alt : 0;
      const coord = maplibregl.MercatorCoordinate.fromLngLat(
        [entry.lon, entry.lat],
        alt,
      );
      nextMercator.set(agent, coord);
      const [r, g, b] = hexToRgb(colors[agent] || "#ffffff");
      const isManned = LAH_IDS.has(agent);
      const size =
        BASE_SIZE *
        pixelRatio *
        scale *
        (isManned ? LAH_POINT_SCALE : UAV_POINT_SCALE);
      points.push(
        coord.x,
        coord.y,
        coord.z,
        r,
        g,
        b,
        size,
        isManned ? SHAPE_HELICOPTER : SHAPE_SPHERE,
        isManned ? spriteRotationRad(entry.heading) : 0,
      );

      const filming = entry.filmingTarget;
      if (
        filming &&
        Number.isFinite(filming.lat) &&
        Number.isFinite(filming.lon)
      ) {
        const tAlt = drapedSurfaceAltitude(filming.lon, filming.lat, filming.alt);
        const tCoord = maplibregl.MercatorCoordinate.fromLngLat(
          [filming.lon, filming.lat],
          tAlt,
        );
        nextFilmingMercator.set(agent, tCoord);
        const sepMeters = groundDistanceMeters(
          { lat: entry.lat, lon: entry.lon },
          { lat: filming.lat, lon: filming.lon },
        );
        if (Number.isFinite(sepMeters)) {
          nextFilmingSep.set(agent, sepMeters);
        }
        const maxSepMeters = Number(entry.filmingMaxSep);
        if (Number.isFinite(maxSepMeters) && maxSepMeters > 0) {
          nextFilmingMaxSep.set(agent, maxSepMeters);
        }
        filmingPoints.push(
          tCoord.x,
          tCoord.y,
          tCoord.z,
          r,
          g,
          b,
          size * FILMING_POINT_SCALE,
          SHAPE_SPHERE,
          0,
        );
        const segments = buildDashedSegments(tCoord, coord, FILMING_DASH_M, FILMING_GAP_M);
        if (segments.length) {
          lineBuffers.get(agent).push(...segments);
        }
      }
      const coords = buildFootprintGeometry(entry);
      if (coords) {
        const color = colors[agent] || "#ffffff";
        footprintLineFeatures.push({
          type: "Feature",
          properties: { agent, color, kind: "current" },
          geometry: { type: "LineString", coordinates: coords },
        });
        footprintFillFeatures.push({
          type: "Feature",
          properties: { agent, color, opacity: FOOTPRINT_CURRENT_OPACITY, kind: "current" },
          geometry: { type: "Polygon", coordinates: [coords] },
        });
      }
    });
    mercatorByAgent = nextMercator;
    filmingMercatorByAgent = nextFilmingMercator;
    filmingSepByAgent = nextFilmingSep;
    filmingMaxSepByAgent = nextFilmingMaxSep;
    return { points, filmingPoints, lineBuffers, footprintLineFeatures, footprintFillFeatures };
  };

  const updateLayers = () => {
    if (!sphereLayer) {
      return;
    }
    const colors = getAgentColors();
    const {
      points,
      filmingPoints,
      lineBuffers,
      footprintLineFeatures,
      footprintFillFeatures,
    } = buildBuffers(colors);
    sphereLayer.updatePoints(points);
    sphereLayer.setVisible(points.length > 0);
    if (filmingPointLayer) {
      filmingPointLayer.updatePoints(filmingPoints);
      filmingPointLayer.setVisible(filmingPoints.length > 0);
    }
    filmingLineLayers.forEach((layer, agent) => {
      const sepMeters = filmingSepByAgent.get(agent);
      const maxSepMeters = filmingMaxSepByAgent.get(agent);
      const isSepExceeded =
        Number.isFinite(sepMeters) &&
        Number.isFinite(maxSepMeters) &&
        sepMeters > maxSepMeters;
      if (typeof layer.setColor === "function") {
        layer.setColor(isSepExceeded ? EXCEEDED_SEP_COLOR : (colors[agent] || "#ffffff"));
      }
      const buf = lineBuffers.get(agent) || [];
      layer.updatePositions(buf);
      layer.setVisible(buf.length > 0);
    });
    trailLineLayers.forEach((layer, agent) => {
      if (typeof layer.setColor === "function") {
        layer.setColor(colors[agent] || "#ffffff");
      }
      const entry = trailHistory.get(agent);
      const coords = entry?.coords || [];
      if (coords.length < 2) {
        layer.updatePositions([]);
        layer.setVisible(false);
        return;
      }
      const positions = [];
      for (let i = 1; i < coords.length; i += 1) {
        const prev = coords[i - 1];
        const next = coords[i];
        positions.push(prev.x, prev.y, prev.z, next.x, next.y, next.z);
      }
      layer.updatePositions(positions);
      layer.setVisible(positions.length > 0);
    });
    const footprintData = {
      type: "FeatureCollection",
      features: [...footprintFillFeatures, ...footprintLineFeatures],
    };
    let footprintTrailData = null;
    if (footprintTrailDirty) {
      footprintTrailDirty = false;
      const trailFeatures = [];
      footprintHistory.forEach((list, agent) => {
        const color = colors[agent] || "#ffffff";
        list.forEach((item) => {
          trailFeatures.push({
            type: "Feature",
            properties: { agent, color, opacity: FOOTPRINT_TRAIL_OPACITY },
            geometry: { type: "Polygon", coordinates: [item.coords] },
          });
        });
      });
      footprintTrailData = {
        type: "FeatureCollection",
        features: trailFeatures,
      };
    }
    scheduleSourceCommit(footprintData, footprintTrailData, true);
  };

  const ensureLabelContainer = () => {
    if (labelContainer) {
      return;
    }
    labelContainer = document.createElement("div");
    labelContainer.style.position = "absolute";
    labelContainer.style.left = "0";
    labelContainer.style.top = "0";
    labelContainer.style.width = "100%";
    labelContainer.style.height = "100%";
    labelContainer.style.pointerEvents = "none";
    labelContainer.style.zIndex = "2";
    map.getContainer().appendChild(labelContainer);
  };

  const ensureLabelElement = (agent, color) => {
    let el = labelElements.get(agent);
    if (!el) {
      el = document.createElement("div");
      el.textContent = agent;
      el.style.position = "absolute";
      el.style.transform = "translate(-50%, -140%)";
      el.style.fontSize = "12px";
      el.style.fontWeight = "600";
      el.style.whiteSpace = "pre";
      el.style.color = color || "#e7eddc";
      el.style.textShadow = "0 1px 2px rgba(0,0,0,0.5)";
      el.style.opacity = "0.95";
      el.style.pointerEvents = "none";
      labelContainer.appendChild(el);
      labelElements.set(agent, el);
    }
    return el;
  };

  const ensureSepLabelElement = (agent, color) => {
    let el = sepLabelElements.get(agent);
    if (!el) {
      el = document.createElement("div");
      el.style.position = "absolute";
      el.style.transform = "translate(-50%, -50%)";
      el.style.padding = "2px 6px";
      el.style.borderRadius = "999px";
      el.style.fontSize = "11px";
      el.style.fontWeight = "700";
      el.style.letterSpacing = "0.02em";
      el.style.whiteSpace = "nowrap";
      el.style.color = color || "#e7eddc";
      el.style.background = "rgba(15, 23, 42, 0.72)";
      el.style.border = `1px solid ${color || "#e7eddc"}33`;
      el.style.textShadow = "0 1px 2px rgba(0,0,0,0.45)";
      el.style.boxShadow = "0 4px 12px rgba(0,0,0,0.18)";
      el.style.opacity = "0.92";
      el.style.pointerEvents = "none";
      labelContainer.appendChild(el);
      sepLabelElements.set(agent, el);
    }
    return el;
  };

  const updateLabelPositions = (matrix) => {
    if (!labelContainer) {
      return;
    }
    lastProjectionMatrix = matrix || null;
    const canvas = map.getCanvas();
    // MapLibre can render below the OS DPR.  Projection results and pointer
    // coordinates are CSS pixels, so use the canvas display size here.
    const width = canvas.clientWidth || canvas.getBoundingClientRect().width || canvas.width;
    const height = canvas.clientHeight || canvas.getBoundingClientRect().height || canvas.height;
    const colors = getAgentColors();
    labelElements.forEach((el) => {
      el.style.display = "none";
    });
    sepLabelElements.forEach((el) => {
      el.style.display = "none";
    });
    mercatorByAgent.forEach((coord, agent) => {
      let point = projectToScreen(matrix, coord.x, coord.y, coord.z, width, height);
      if (!point) {
        const entry = currentPositions[agent];
        if (entry && Number.isFinite(entry.lon) && Number.isFinite(entry.lat)) {
          const projected = map.project([entry.lon, entry.lat]);
          point = { x: projected.x, y: projected.y };
        }
      }
      if (!point) {
        return;
      }
      const el = ensureLabelElement(agent, colors[agent] || "#e7eddc");
      const sepMeters = filmingSepByAgent.get(agent);
      const maxSepMeters = filmingMaxSepByAgent.get(agent);
      const isSepExceeded =
        Number.isFinite(sepMeters) &&
        Number.isFinite(maxSepMeters) &&
        sepMeters > maxSepMeters;
      const entry = currentPositions[agent] || {};
      if (LAH_IDS.has(agent)) {
        const hasPlanDeviation = entry.planDeviationM !== null
          && entry.planDeviationM !== undefined
          && Number.isFinite(Number(entry.planDeviationM));
        const hasTerrainClearance = entry.terrainClearanceM !== null
          && entry.terrainClearanceM !== undefined
          && Number.isFinite(Number(entry.terrainClearanceM));
        const planDeviationM = Number(entry.planDeviationM);
        const terrainClearanceM = Number(entry.terrainClearanceM);
        const demAvailable = entry.demAvailable === true && hasTerrainClearance;
        const planFollowing = entry.planFollowing !== false && hasPlanDeviation;
        const planText = hasPlanDeviation
          ? `${Math.round(planDeviationM)}m`
          : "--";
        const clearanceText = demAvailable
          ? `${Math.round(terrainClearanceM)}m`
          : "범위 밖";
        el.textContent = `${agent}\n계획이탈 ${planText} | DEM이격 ${clearanceText}`;
        const unsafeClearance = demAvailable
          && terrainClearanceM < LAH_MIN_TERRAIN_CLEARANCE_M;
        const warning = !planFollowing || unsafeClearance || !demAvailable;
        const labelColor = warning ? EXCEEDED_SEP_COLOR : (colors[agent] || "#e7eddc");
        el.style.color = labelColor;
        el.style.padding = "4px 7px";
        el.style.borderRadius = "6px";
        el.style.background = warning ? "rgba(127, 29, 29, 0.84)" : "rgba(15, 23, 42, 0.80)";
        el.style.border = `1px solid ${labelColor}55`;
        el.style.boxShadow = "0 4px 12px rgba(0,0,0,0.28)";
        el.style.lineHeight = "1.35";
        el.style.transform = "translate(-50%, -155%)";
      } else {
        el.textContent = Number.isFinite(sepMeters)
          ? `${agent} | ${Math.round(sepMeters)}m`
          : agent;
        el.style.color = isSepExceeded ? EXCEEDED_SEP_COLOR : (colors[agent] || "#e7eddc");
        el.style.padding = "0";
        el.style.background = "transparent";
        el.style.border = "none";
        el.style.boxShadow = "none";
        el.style.lineHeight = "normal";
        el.style.transform = "translate(-50%, -140%)";
      }
      el.style.left = `${point.x}px`;
      el.style.top = `${point.y}px`;
      el.style.display = "block";

      const filmingCoord = filmingMercatorByAgent.get(agent);
      if (!filmingCoord || !Number.isFinite(sepMeters)) {
        return;
      }
      let filmingPoint = projectToScreen(
        matrix,
        filmingCoord.x,
        filmingCoord.y,
        filmingCoord.z,
        width,
        height,
      );
      if (!filmingPoint) {
        const filming = currentPositions[agent]?.filmingTarget;
        if (filming && Number.isFinite(filming.lon) && Number.isFinite(filming.lat)) {
          const projected = map.project([filming.lon, filming.lat]);
          filmingPoint = { x: projected.x, y: projected.y };
        }
      }
      if (!filmingPoint) {
        return;
      }
      const midX = (point.x + filmingPoint.x) * 0.5;
      const midY = (point.y + filmingPoint.y) * 0.5;
      const sepEl = ensureSepLabelElement(agent, colors[agent] || "#e7eddc");
      sepEl.textContent = `SEP ${Math.round(sepMeters)}m`;
      sepEl.style.color = isSepExceeded ? EXCEEDED_SEP_COLOR : (colors[agent] || "#e7eddc");
      sepEl.style.border = `1px solid ${(isSepExceeded ? EXCEEDED_SEP_COLOR : (colors[agent] || "#e7eddc"))}33`;
      sepEl.style.background = isSepExceeded ? "rgba(127, 29, 29, 0.82)" : "rgba(15, 23, 42, 0.72)";
      sepEl.style.left = `${midX}px`;
      sepEl.style.top = `${midY}px`;
      sepEl.style.display = "block";
    });
  };

  const isSelectionBlocked = () => {
    const overlay = document.getElementById("scenario-overlay");
    if (overlay && overlay.classList.contains("is-active")) {
      return true;
    }
    const enemyPicker = document.getElementById("enemy-picker");
    if (enemyPicker && enemyPicker.classList.contains("is-active")) {
      return true;
    }
    return false;
  };

  const getPickRadius = (agent) => {
    const zoom = typeof map.getZoom === "function" ? map.getZoom() : 12;
    const scale = Math.min(2.2, Math.max(0.9, 0.7 + (zoom - 11) * 0.12));
    const base =
      BASE_SIZE * scale * (LAH_IDS.has(agent) ? LAH_POINT_SCALE : UAV_POINT_SCALE);
    return Math.max(10, base * 0.6 + 6);
  };

  const pickAgentAtPoint = (point) => {
    if (!point || mercatorByAgent.size === 0) {
      return null;
    }
    const canvas = map.getCanvas();
    if (!canvas) {
      return null;
    }
    const width = canvas.clientWidth || canvas.getBoundingClientRect().width || canvas.width;
    const height = canvas.clientHeight || canvas.getBoundingClientRect().height || canvas.height;
    const matrix = lastProjectionMatrix;
    let best = null;
    let bestDist = Infinity;
    mercatorByAgent.forEach((coord, agent) => {
      let screen = null;
      if (matrix) {
        screen = projectToScreen(matrix, coord.x, coord.y, coord.z, width, height);
      }
      if (!screen) {
        const entry = currentPositions[agent];
        if (entry && Number.isFinite(entry.lon) && Number.isFinite(entry.lat)) {
          const projected = map.project([entry.lon, entry.lat]);
          screen = { x: projected.x, y: projected.y };
        }
      }
      if (!screen) {
        return;
      }
      const dx = screen.x - point.x;
      const dy = screen.y - point.y;
      const dist = Math.hypot(dx, dy);
      const radius = getPickRadius(agent);
      if (dist <= radius && dist < bestDist) {
        bestDist = dist;
        best = agent;
      }
    });
    return best;
  };

  const ensureLayer = () => {
    ensureLabelContainer();
    if (!map.getLayer(SPHERE_LAYER_ID)) {
      sphereLayer = createSphereLayer(SPHERE_LAYER_ID);
      sphereLayer._renderHook = updateLabelPositions;
      map.addLayer(sphereLayer);
    }
    if (!map.getLayer(FILMING_POINT_LAYER_ID)) {
      filmingPointLayer = createSphereLayer(FILMING_POINT_LAYER_ID);
      map.addLayer(filmingPointLayer);
    }
    if (!map.getSource(FOOTPRINT_SOURCE_ID)) {
      map.addSource(FOOTPRINT_SOURCE_ID, {
        type: "geojson",
        data: { type: "FeatureCollection", features: [] },
      });
    }
    footprintSource = map.getSource(FOOTPRINT_SOURCE_ID);
    if (!map.getSource(FOOTPRINT_TRAIL_SOURCE_ID)) {
      map.addSource(FOOTPRINT_TRAIL_SOURCE_ID, {
        type: "geojson",
        data: { type: "FeatureCollection", features: [] },
      });
    }
    footprintTrailSource = map.getSource(FOOTPRINT_TRAIL_SOURCE_ID);
    if (!map.getLayer(FOOTPRINT_TRAIL_LAYER_ID)) {
      map.addLayer({
        id: FOOTPRINT_TRAIL_LAYER_ID,
        type: "fill",
        source: FOOTPRINT_TRAIL_SOURCE_ID,
        paint: {
          "fill-color": ["get", "color"],
          "fill-opacity": ["get", "opacity"],
        },
      });
    }
    if (!map.getLayer(FOOTPRINT_FILL_LAYER_ID)) {
      map.addLayer({
        id: FOOTPRINT_FILL_LAYER_ID,
        type: "fill",
        source: FOOTPRINT_SOURCE_ID,
        filter: ["==", ["geometry-type"], "Polygon"],
        paint: {
          "fill-color": ["get", "color"],
          "fill-opacity": ["get", "opacity"],
        },
      });
    }
    if (!map.getLayer(FOOTPRINT_LAYER_ID)) {
      map.addLayer({
        id: FOOTPRINT_LAYER_ID,
        type: "line",
        source: FOOTPRINT_SOURCE_ID,
        filter: ["==", ["geometry-type"], "LineString"],
        layout: { "line-join": "round", "line-cap": "round" },
        paint: {
          "line-color": ["get", "color"],
          "line-width": 1.2,
          "line-opacity": 0.55,
          "line-dasharray": [1.2, 1.8],
        },
      });
    }
    const colors = getAgentColors();
    AGENTS.forEach((agent) => {
      const layerId = `${TRAIL_LINE_PREFIX}${agent.toLowerCase()}`;
      if (map.getLayer(layerId)) {
        return;
      }
      const layer = createLineLayer(layerId, colors[agent] || "#ffffff");
      layer._useDepth = true;
      trailLineLayers.set(agent, layer);
      map.addLayer(layer, SPHERE_LAYER_ID);
    });
    AGENTS.forEach((agent) => {
      const layerId = `${FILMING_LINE_PREFIX}${agent.toLowerCase()}`;
      if (map.getLayer(layerId)) {
        return;
      }
      const layer = createLineLayer(layerId, colors[agent] || "#ffffff");
      filmingLineLayers.set(agent, layer);
      map.addLayer(layer);
    });
  };

  const applyData = (payload) => {
    const ok = payload && payload.ok;
    const rawStep = payload?.step;
    const incomingStep = rawStep === null || rawStep === undefined ? Number.NaN : Number(rawStep);
    if (
      Number.isFinite(incomingStep) &&
      Number.isFinite(lastStep) &&
      incomingStep < lastStep
    ) {
      return;
    }
    const rawPositions = ok && payload.vehicles && typeof payload.vehicles === "object"
      ? payload.vehicles
      : {};
    const hasAuthoritativePositions = Object.keys(rawPositions).length > 0;
    let payloadSignature = null;
    try {
      payloadSignature = JSON.stringify(rawPositions);
    } catch (_err) {
      payloadSignature = null;
    }
    if (
      hasAuthoritativePositions &&
      payloadSignature !== null &&
      payloadSignature === lastVehiclePayloadSignature
    ) {
      if (Number.isFinite(incomingStep)) {
        lastStep = incomingStep;
      }
      return;
    }
    if (
      !hasAuthoritativePositions &&
      payloadSignature !== null &&
      payloadSignature === lastVehiclePayloadSignature &&
      !Object.keys(currentPositions || {}).length
    ) {
      return;
    }
    lastVehiclePayloadSignature = payloadSignature;
    const authoritativePositions = hasAuthoritativePositions ? clonePositions(rawPositions) : {};
    const now = (typeof performance !== "undefined" && typeof performance.now === "function")
      ? performance.now()
      : Date.now();
    if (!authoritativePositions || !Object.keys(authoritativePositions).length) {
      sampleFromPositions = null;
      sampleToPositions = null;
      currentPositions = {};
      footprintHistory.clear();
      footprintTrailDirty = true;
      trailHistory.clear();
      lastFootprintSampleMs = 0;
      lastStep = null;
      applyInterpolatedState({}, null);
      return;
    }
    const step = Number(payload?.step);
    if (
      Number.isFinite(step) &&
      step !== lastStep &&
      step % FOOTPRINT_TRAIL_SAMPLE_STEP === 0 &&
      !sampleFromPositions
    ) {
      recordFootprintHistory(authoritativePositions, now, true);
    }
    AGENTS.forEach((agent) => {
      const entry = authoritativePositions[agent];
      if (!entry || !Number.isFinite(entry.lat) || !Number.isFinite(entry.lon)) {
        return;
      }
      const alt = Number.isFinite(entry.alt) ? entry.alt : 0;
      const coord = maplibregl.MercatorCoordinate.fromLngLat(
        [Number(entry.lon), Number(entry.lat)],
        alt,
      );
      const m2u = coord.meterInMercatorCoordinateUnits();
      if (!Number.isFinite(m2u) || m2u <= 0) {
        return;
      }
      const point = {
        x: coord.x,
        y: coord.y,
        z: coord.z - m2u * TRAIL_Z_OFFSET_M,
        m2u,
      };
      const trail = trailHistory.get(agent) || { coords: [], length: 0 };
      const coords = trail.coords;
      if (!coords.length) {
        coords.push(point);
      } else {
        const last = coords[coords.length - 1];
        const seg = distMeters(last, point);
        if (seg >= TRAIL_MIN_SEGMENT_M) {
          coords.push(point);
          trail.length += seg;
        }
      }
      while (trail.length > TRAIL_MAX_METERS && coords.length > 1) {
        const removed = coords.shift();
        const next = coords[0];
        trail.length -= distMeters(removed, next);
      }
      trailHistory.set(agent, trail);
    });
    if (VISUAL_UPDATE_HZ <= 5) {
      if (animationFrameId !== null) {
        cancelAnimationFrame(animationFrameId);
        animationFrameId = null;
      }
      sampleFromPositions = clonePositions(authoritativePositions);
      sampleToPositions = null;
      sampleStartMs = now;
      sampleDurationMs = VISUAL_UPDATE_INTERVAL_MS;
      lastAuthoritativeMs = now;
      recordFootprintHistory(authoritativePositions, now, false);
      applyInterpolatedState(authoritativePositions, incomingStep);
      return;
    }
    const nextDuration = lastAuthoritativeMs > 0
      ? Math.max(VISUAL_UPDATE_INTERVAL_MS, Math.min(400, now - lastAuthoritativeMs))
      : 200;
    const fromPositions = Object.keys(currentPositions || {}).length
      ? clonePositions(currentPositions)
      : Object.keys(authoritativePositions).length
        ? clonePositions(authoritativePositions)
        : {};
    sampleFromPositions = fromPositions;
    sampleToPositions = authoritativePositions;
    sampleStartMs = now;
    sampleDurationMs = nextDuration;
    lastAuthoritativeMs = now;
    if (!animationFrameId) {
      animationFrameId = requestAnimationFrame(animateVisuals);
    }
    applyInterpolatedState(interpolatePositions(sampleFromPositions, sampleToPositions, 0), step);
  };

  const loadFromReference = (payload) => {
    pendingData = payload;
    if (mapReady) {
      applyData(payload);
    }
  };

  map.on("load", () => {
    mapReady = true;
    ensureLayer();
    if (!animationFrameId) {
      animationFrameId = requestAnimationFrame(animateVisuals);
    }
    if (pendingData) {
      applyData(pendingData);
    }
  });

  map.on("zoom", () => {
    updateLayers();
  });

  // The helicopter's nose is baked into the vertex buffer, so a map rotation
  // has to rebuild it or the aircraft points the wrong way.
  map.on("rotate", () => {
    updateLayers();
  });

  map.on("resize", () => {
    updateLayers();
  });

  if (typeof map.on === "function") {
    map.on("remove", () => {
      if (animationFrameId !== null) {
        cancelAnimationFrame(animationFrameId);
        animationFrameId = null;
      }
      clearScheduledSourceCommit();
      pendingFootprintData = null;
      pendingFootprintTrailData = null;
      repaintQueued = false;
    });
  }

  map.on("click", (event) => {
    if (!event || !event.point) {
      return;
    }
    const original = event.originalEvent;
    if (original && (original.defaultPrevented || original.cancelBubble)) {
      return;
    }
    if (isSelectionBlocked()) {
      return;
    }
    const label = pickAgentAtPoint(event.point);
    if (!label) {
      return;
    }
    if (typeof window.selectAgent === "function") {
      window.selectAgent(label, { flyTo: true, source: "map" });
      return;
    }
    const button = document.querySelector(`.ui-btn-text[data-agent="${label}"]`);
    if (button) {
      button.click();
    }
  });

  const getPosition = (label) => currentPositions[label] || null;
  const getPositions = () => ({ ...currentPositions });

  return {
    loadFromReference,
    getPosition,
    getPositions,
    getFilmingViews,
    subscribeFilmingViews,
  };
};
