import { logStatus } from "./status_log.js";

const SOURCE_ID = "mission-paths";
const HIT_LAYER_ID = "mission-paths-hit";
const AREA_SOURCE_ID = "mission-areas";
const AREA_FILL_LAYER_ID = "mission-areas-fill";
const AREA_LINE_LAYER_ID = "mission-areas-line";
const AREA_FORWARD_LINE_LAYER_ID = "mission-areas-forward-line";
const AREA_REVERSE_LINE_LAYER_ID = "mission-areas-reverse-line";
const SWEEP_SOURCE_ID = "mission-sweep";
const SWEEP_LINE_LAYER_ID = "mission-sweep-line";
const SWEEP_FORWARD_LINE_LAYER_ID = "mission-sweep-forward-line";
const SWEEP_REVERSE_LINE_LAYER_ID = "mission-sweep-reverse-line";
const SWEEP_POINT_LAYER_ID = "mission-sweep-point";
const WAYPOINT_HIT_SOURCE_ID = "mission-waypoint-hit";
const WAYPOINT_HIT_LAYER_ID = "mission-waypoint-hit";

const AGENTS = ["LAH1", "LAH2", "LAH3", "UAV1", "UAV2", "UAV3"];

const DEFAULT_ALPHA = 0.8;
const SELECT_ALPHA = 0.8;
const DIM_ALPHA = 0.12;
const DONE_ALPHA = 0.28;
const DONE_SELECT_ALPHA = 0.34;
const DONE_DIM_ALPHA = 0.08;
const AREA_FILL_ALPHA = 0.12;
const AREA_FILL_DONE_ALPHA = 0.06;
const AREA_FILL_DIM_ALPHA = 0.035;
const AREA_FILL_DONE_DIM_ALPHA = 0.02;
const AREA_LINE_ALPHA = 0.42;
const AREA_LINE_DONE_ALPHA = 0.22;
const AREA_LINE_DIM_ALPHA = 0.14;
const AREA_LINE_DONE_DIM_ALPHA = 0.08;
const SWEEP_LINE_ALPHA = 0.72;
const SWEEP_LINE_DONE_ALPHA = 0.3;
const SWEEP_LINE_DIM_ALPHA = 0.11;
const SWEEP_LINE_DONE_DIM_ALPHA = 0.06;
const SWEEP_POINT_ALPHA = 0.96;
const SWEEP_POINT_DONE_ALPHA = 0.52;
const SWEEP_POINT_DIM_ALPHA = 0.26;
const SWEEP_POINT_DONE_DIM_ALPHA = 0.14;
const SWEEP_SELECTED_LINE_ALPHA = 1.0;
const SWEEP_SELECTED_LINE_DONE_ALPHA = 0.88;
const SWEEP_SELECTED_POINT_ALPHA = 1.0;
const SWEEP_SELECTED_POINT_DONE_ALPHA = 0.9;
const SWEEP_WAYPOINT_CONTEXT_LINE_ALPHA = 0.1;
const SWEEP_WAYPOINT_CONTEXT_DONE_ALPHA = 0.05;
const SWEEP_WAYPOINT_CONTEXT_POINT_ALPHA = 0.2;
const SWEEP_WAYPOINT_CONTEXT_POINT_DONE_ALPHA = 0.1;

const PATH_WIDTH_PX = 2.5;
const WAYPOINT_SIZE_PX = 10;
const WAYPOINT_ALPHA = 0.9;
const WAYPOINT_DIM_ALPHA = 0.25;
const WAYPOINT_DONE_ALPHA = 0.3;
const WAYPOINT_DONE_SELECT_ALPHA = 0.35;
const WAYPOINT_DONE_DIM_ALPHA = 0.1;
const WAYPOINT_CURRENT_ALPHA = 1.0;
const WAYPOINT_CURRENT_DIM_ALPHA = 0.58;
const WAYPOINT_CURRENT_DONE_ALPHA = 0.96;
const WAYPOINT_CURRENT_DONE_DIM_ALPHA = 0.68;
const WAYPOINT_Z_OFFSET_M = 4.0;
const WAYPOINT_LABEL_Z_OFFSET_M = 14.0;
const WAYPOINT_LABEL_FONT_SIZE = 12;
const ALT_OFFSET_M = 5;
const DASH_ON_PX = 6;
const DASH_OFF_PX = 6;
const EARTH_RADIUS_M = 6371008.8;
const PASS_TYPE_LABELS = {
  1: "Fly-by",
  2: "Loiter",
  3: "Fly-over",
};

// ICD 0304 LAHWaypoint has no pass-type field: a manned waypoint's role is
// implied by which of its hovering / loiter / attack sub-structures is filled.
// Concealment has no ICD representation at all, so it arrives out-of-band via
// the tactical-point sidecar and is matched here by waypoint ID.
const LAH_ROLE_STYLES = {
  attack: { label: "ATTACK", color: "#ff5a5f", weight: 800, emphasis: 1.14, opacity: 1 },
  conceal: { label: "CONCEAL", color: "#35e6a1", weight: 800, emphasis: 1.1, opacity: 1 },
  hover: { label: "HOVER", color: "#ffd166", weight: 700, emphasis: 1.06, opacity: 1 },
  loiter: { label: "LOITER", color: "#ffd166", weight: 700, emphasis: 1.06, opacity: 1 },
  // A transit leg carries no decision, and a manned route is mostly transit.
  // These recede almost to nothing so the operational points stay legible;
  // hovering one brings it back (see .wp-label--quiet in style.css).
  transit: { label: "TRANSIT", color: null, weight: 400, emphasis: 0.9, opacity: 0.12 },
};

export const waypointListOf = (path) => {
  // A manned flight path carries lahWaypointList; reading only waypointList is
  // why every LAH label used to render as "Pass N/A".
  const list =
    path?.lahWaypointList ??
    path?.LAHWaypointList ??
    path?.uavWaypointList ??
    path?.UAVWaypointList ??
    path?.waypointList ??
    path?.WaypointList;
  return Array.isArray(list) ? list : [];
};

const positiveNumber = (value) => {
  const parsed = Number(value);
  return Number.isFinite(parsed) && parsed > 0 ? parsed : 0;
};

export const getLahPointRole = (wp, concealIds = null) => {
  const attack = wp?.attack ?? wp?.Attack ?? null;
  if (positiveNumber(attack?.targetID ?? attack?.TargetID ?? attack?.targetId) > 0) {
    return "attack";
  }
  const waypointId = Number(wp?.waypointID ?? wp?.WaypointID);
  if (concealIds && Number.isFinite(waypointId) && concealIds.has(waypointId)) {
    return "conceal";
  }
  const hovering = wp?.hovering ?? wp?.Hovering ?? null;
  if (positiveNumber(hovering?.time ?? hovering?.Time) > 0) {
    return "hover";
  }
  const loiter = wp?.loiter ?? wp?.Loiter ?? wp?.loiterProperty ?? wp?.LoiterProperty ?? null;
  if (
    positiveNumber(loiter?.radius ?? loiter?.Radius) > 0 ||
    positiveNumber(loiter?.time ?? loiter?.Time) > 0
  ) {
    return "loiter";
  }
  return "transit";
};

const concealIdsForPath = (payload, pathId) => {
  // Out-of-band tactical points: the concealment endpoint has no ICD field, so
  // the planner records it beside the flight path instead of inside it.
  const table = payload?.lahTacticalPoints;
  if (!table || !Number.isFinite(Number(pathId))) {
    return null;
  }
  const entry = table[String(pathId)] ?? table[Number(pathId)];
  const ids = entry?.concealWaypointIDs ?? entry?.concealWaypointIds;
  if (!Array.isArray(ids) || !ids.length) {
    return null;
  }
  const parsed = ids.map((value) => Number(value)).filter((value) => Number.isFinite(value));
  return parsed.length ? new Set(parsed) : null;
};

const lahRoleSummary = (wp, role) => {
  if (role === "attack") {
    const attack = wp?.attack ?? wp?.Attack ?? null;
    const targetId = positiveNumber(attack?.targetID ?? attack?.TargetID ?? attack?.targetId);
    const weapon = positiveNumber(attack?.weaponType ?? attack?.WeaponType);
    const bits = ["ATTACK"];
    if (targetId) bits.push(`T${targetId}`);
    if (weapon) bits.push(`W${weapon}`);
    return bits.join(" · ");
  }
  if (role === "hover") {
    const hovering = wp?.hovering ?? wp?.Hovering ?? null;
    const seconds = positiveNumber(hovering?.time ?? hovering?.Time);
    return seconds ? `HOVER ${Math.round(seconds)}s` : "HOVER";
  }
  if (role === "conceal") {
    return "CONCEAL";
  }
  return null;
};

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

const hexToRgba = (hex, alpha = 1) => {
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
    return new Float32Array([1, 1, 1, alpha]);
  }
  const r = ((value >> 16) & 255) / 255;
  const g = ((value >> 8) & 255) / 255;
  const b = (value & 255) / 255;
  return new Float32Array([r, g, b, alpha]);
};

const normalizeMatrix = (matrix) => {
  if (!matrix) {
    return null;
  }
  // MapLibre normally hands custom layers a Float32Array already. Keep that
  // buffer instead of allocating and copying another 16-float array for every
  // custom layer on every render.
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
  if (Array.isArray(matrix.elements)) {
    return new Float32Array(matrix.elements);
  }
  if (matrix.elements && typeof matrix.elements.length === "number") {
    return new Float32Array(Array.from(matrix.elements));
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

const toRad = (value) => (value * Math.PI) / 180;

const distanceMeters = (lon1, lat1, lon2, lat2) => {
  const phi1 = toRad(lat1);
  const phi2 = toRad(lat2);
  const dPhi = toRad(lat2 - lat1);
  const dLambda = toRad(lon2 - lon1);
  const sinDphi = Math.sin(dPhi / 2);
  const sinDlam = Math.sin(dLambda / 2);
  const a = sinDphi * sinDphi + Math.cos(phi1) * Math.cos(phi2) * sinDlam * sinDlam;
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  return EARTH_RADIUS_M * c;
};

const metersPerPixelAt = (lat, zoom) => {
  const scale = Math.pow(2, zoom);
  const meters = 156543.03392 * Math.cos(toRad(lat)) / scale;
  return Number.isFinite(meters) && meters > 0 ? meters : 1;
};

const formatAltRange = (minAlt, maxAlt) => {
  const minVal = Number.isFinite(minAlt) ? Math.round(minAlt) : null;
  const maxVal = Number.isFinite(maxAlt) ? Math.round(maxAlt) : null;
  if (minVal === null && maxVal === null) {
    return "ALT N/A";
  }
  if (minVal !== null && maxVal !== null) {
    if (minVal === maxVal) {
      return `ALT ${minVal} m`;
    }
    return `ALT ${minVal}-${maxVal} m`;
  }
  const single = minVal !== null ? minVal : maxVal;
  return `ALT ${single} m`;
};

const AREA_COVERAGE_PASS_STYLES = {
  forward: {
    label: "OUTBOUND (갈 때)",
    shortLabel: "OUTBOUND",
    color: "#48ddff",
  },
  reverse: {
    label: "RETURN (올 때)",
    shortLabel: "RETURN",
    color: "#ffb34d",
  },
};

const normalizeAreaCoveragePass = (value) => {
  const normalized = String(value || "").trim().toLowerCase();
  return Object.prototype.hasOwnProperty.call(AREA_COVERAGE_PASS_STYLES, normalized)
    ? normalized
    : null;
};

const isReciprocalAreaTurnWaypoint = (waypoint) =>
  String(waypoint?.areaTurnRole ?? waypoint?.AreaTurnRole ?? "")
    .trim()
    .toLowerCase() === "reciprocal_turn";

const agentFromAircraftId = (aircraftId) =>
  aircraftId >= 1 && aircraftId <= 3
    ? `LAH${aircraftId}`
    : aircraftId >= 4 && aircraftId <= 6
      ? `UAV${aircraftId - 3}`
      : `AC${aircraftId}`;

const coveragePassMapKey = (pathId, pass) => `${String(pathId ?? "")}:${pass}`;

/**
 * Resolve each reciprocal Area pass independently from immutable FlightPath
 * metadata plus the live SIM current waypoint.  A waypoint is the destination
 * of its incoming filming leg, so equality means that pass is actively being
 * captured; a later waypoint means it has completed.
 */
export const buildAreaCoveragePassIndex = (payload, currentWaypointsByAgent = new Map()) => {
  const result = new Map();
  const flightPaths = Array.isArray(payload?.flightPaths) ? payload.flightPaths : [];
  flightPaths.forEach((path) => {
    const aircraftId = Number(path?.aircraftID);
    const pathId = Number(path?.pathID);
    if (!Number.isFinite(aircraftId) || !Number.isFinite(pathId)) {
      return;
    }
    const agent = agentFromAircraftId(aircraftId);
    const waypointList = waypointListOf(path);
    const currentWaypointId = normalizeCurrentWaypointId(
      currentWaypointsByAgent instanceof Map
        ? currentWaypointsByAgent.get(agent)
        : currentWaypointsByAgent?.[agent],
    );
    const currentIndex = waypointList.findIndex(
      (waypoint) => normalizeCurrentWaypointId(waypoint?.waypointID ?? waypoint?.WaypointID) === currentWaypointId,
    );
    const grouped = new Map();
    waypointList.forEach((waypoint, waypointIndex) => {
      // The three compact turn gates connect OUTBOUND to RETURN but are not
      // capture waypoints. Never attribute them to either coverage pass even
      // when an older payload inherited the previous pass marker.
      if (isReciprocalAreaTurnWaypoint(waypoint)) {
        return;
      }
      const pass = normalizeAreaCoveragePass(
        waypoint?.areaCoveragePass ?? waypoint?.AreaCoveragePass,
      );
      if (!pass) {
        return;
      }
      if (!grouped.has(pass)) {
        grouped.set(pass, []);
      }
      grouped.get(pass).push({ waypoint, waypointIndex });
    });
    const reciprocal = grouped.has("forward") && grouped.has("reverse");
    grouped.forEach((entries, pass) => {
      const waypointIds = entries
        .map(({ waypoint }) => normalizeCurrentWaypointId(waypoint?.waypointID ?? waypoint?.WaypointID))
        .filter(Number.isFinite);
      const indices = entries.map(({ waypointIndex }) => waypointIndex);
      const minIndex = Math.min(...indices);
      const maxIndex = Math.max(...indices);
      let status = "planned";
      if (currentIndex >= 0) {
        status = currentIndex >= minIndex && currentIndex <= maxIndex
          ? "active"
          : currentIndex > maxIndex
            ? "completed"
            : "planned";
      } else if (Number.isFinite(currentWaypointId) && waypointIds.length) {
        const minWaypointId = Math.min(...waypointIds);
        const maxWaypointId = Math.max(...waypointIds);
        status = currentWaypointId >= minWaypointId && currentWaypointId <= maxWaypointId
          ? "active"
          : currentWaypointId > maxWaypointId
            ? "completed"
            : "planned";
      } else if (entries.every(({ waypoint }) => Boolean(waypoint?.isDone))) {
        status = "completed";
      }
      result.set(coveragePassMapKey(pathId, pass), {
        pathId,
        aircraftId,
        agent,
        pass,
        passIndex: pass === "forward" ? 1 : 2,
        status,
        reciprocal,
        waypointIds,
        waypointCount: entries.length,
      });
    });
  });
  return result;
};

const formatMeters = (value) => {
  const num = Number(value);
  if (!Number.isFinite(num)) {
    return "-";
  }
  if (Math.abs(num) >= 1000) {
    return `${(num / 1000).toFixed(2).replace(/\.?0+$/, "")} km`;
  }
  return `${Math.round(num * 10) / 10} m`;
};

const formatSweepSpacing = (summary) => {
  if (!summary) {
    return "Sweep line gap N/A";
  }
  const avg = Number(summary.averageLineSpacingM ?? summary.sweepAvgSpacingM);
  if (!Number.isFinite(avg)) {
    return "Sweep line gap N/A";
  }
  const lineCount = Number(summary.lineCount ?? summary.sweepLineCount);
  const pairCount = Number(summary.pairCount ?? summary.sweepPairCount);
  const countBits = [
    Number.isFinite(lineCount) ? `${lineCount} lines` : null,
    Number.isFinite(pairCount) ? `${pairCount} pairs` : null,
  ]
    .filter(Boolean)
    .join(" · ");
  return `Sweep line gap avg ${formatMeters(avg)}${countBits ? ` (${countBits})` : ""}`;
};

const formatFovDeg = (fovDeg) => {
  const value = Number(fovDeg);
  if (!Number.isFinite(value) || value <= 0) {
    return null;
  }
  const rounded = Math.round(value * 100) / 100;
  const text = rounded.toFixed(2).replace(/\.?0+$/, "");
  return `FOV ${text}°`;
};

const coerceInt = (value, fallback = null) => {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? Math.trunc(parsed) : fallback;
};

const getWaypointPassInfo = (wp) => {
  const rawPassType = wp?.waypointPassType ?? wp?.WaypointPassType ?? wp?.passType;
  const passType = coerceInt(rawPassType, null);
  const passLabel = PASS_TYPE_LABELS[passType] || null;
  const filming = wp?.filmingProperty || wp?.FilmingProperty || null;
  const rawFovDeg = filming?.fieldOfView ?? filming?.FieldOfView;
  const fovDeg = Number(rawFovDeg);
  const loiter = wp?.loiterProperty || wp?.loiter || wp?.LoiterProperty || wp?.Loiter || null;
  const loiterRadius = Number(loiter?.radius);
  const loiterTime = Number(loiter?.time);
  const loiterSpeed = Number(loiter?.speed);
  const loiterDirection = coerceInt(loiter?.direction, null);
  const loiterBits = [];
  const reciprocalAreaTurn = isReciprocalAreaTurnWaypoint(wp);
  const coveragePass = reciprocalAreaTurn
    ? null
    : normalizeAreaCoveragePass(
        wp?.areaCoveragePass ?? wp?.AreaCoveragePass,
      );
  const coveragePassLabel = reciprocalAreaTurn
    ? "TURN → RETURN"
    : coveragePass
      ? AREA_COVERAGE_PASS_STYLES[coveragePass]?.label || null
      : null;
  // Only describe a loiter the aircraft actually flies. Every waypoint carries
  // a zeroed loiter block, and rendering it read as "V0m/s" - an aircraft
  // commanded to stand still - when the real transit speed sits in `speed`.
  const loiterActive =
    (Number.isFinite(loiterRadius) && loiterRadius > 0) ||
    (Number.isFinite(loiterTime) && loiterTime > 0);
  if (loiterActive) {
    if (Number.isFinite(loiterRadius)) {
      loiterBits.push(`R${Math.round(loiterRadius)}m`);
    }
    if (Number.isFinite(loiterTime)) {
      loiterBits.push(`T${Math.round(loiterTime)}s`);
    }
    if (Number.isFinite(loiterSpeed) && loiterSpeed > 0) {
      loiterBits.push(`V${Math.round(loiterSpeed)}m/s`);
    }
    if (Number.isFinite(loiterDirection)) {
      loiterBits.push(loiterDirection === 1 ? "CW" : loiterDirection === 2 ? "CCW" : "DIR");
    }
  }
  const speedMps = Number(wp?.speed ?? wp?.Speed);
  const hoverRaw = wp?.hovering ?? wp?.Hovering ?? wp?.hoveringProperty;
  const hoverSeconds = Number(hoverRaw?.time ?? hoverRaw?.Time);
  const etaSeconds = Number(wp?.eta ?? wp?.ETA);
  return {
    speedMps: Number.isFinite(speedMps) ? speedMps : null,
    hoverSeconds: Number.isFinite(hoverSeconds) && hoverSeconds > 0 ? hoverSeconds : null,
    etaSeconds: Number.isFinite(etaSeconds) ? etaSeconds : null,
    passType,
    passLabel,
    isLoiter: passType === 2,
    isFlyBy: passType === 1,
    isFlyOver: passType === 3,
    loiter,
    loiterSummary: loiterBits.length ? loiterBits.join(" · ") : null,
    fovDeg: Number.isFinite(fovDeg) && fovDeg > 0 ? fovDeg : null,
    fovLabel: formatFovDeg(fovDeg),
    coveragePass,
    coveragePassLabel,
    reciprocalAreaTurn,
  };
};

const getFilmingSweepCoordinateList = (filming) => {
  if (!filming || typeof filming !== "object") {
    return { coordinateList: [], interpolationPoints: 0 };
  }
  const lineSearch = filming?.lineSearch || filming?.LineSearch || null;
  const coordinateList = Array.isArray(lineSearch?.coordinateList)
    ? lineSearch.coordinateList
    : Array.isArray(lineSearch?.CoordinateList)
      ? lineSearch.CoordinateList
      : Array.isArray(filming?.coordinateList)
        ? filming.coordinateList
        : Array.isArray(filming?.CoordinateList)
          ? filming.CoordinateList
          : [];
  const interpolationPoints =
    Number(lineSearch?.interpolationPoints ?? lineSearch?.InterpolationPoints) ||
    Number(filming?.interpolationPoints ?? filming?.InterpolationPoints) ||
    0;
  return { coordinateList, interpolationPoints };
};

const normalizeCurrentWaypointId = (entry) => {
  if (entry === null || entry === undefined) {
    return null;
  }
  if (typeof entry === "object") {
    const nested = entry.waypointID ?? entry.WaypointID ?? entry.id ?? entry.ID;
    return normalizeCurrentWaypointId(nested);
  }
  const value = Number(entry);
  return Number.isFinite(value) ? Math.trunc(value) : null;
};

const coerceBoolean = (value, fallback = false) => {
  if (typeof value === "boolean") {
    return value;
  }
  if (typeof value === "number") {
    return value !== 0;
  }
  const normalized = String(value ?? "").trim().toLowerCase();
  if (["1", "true", "yes", "on"].includes(normalized)) {
    return true;
  }
  if (["0", "false", "no", "off", ""].includes(normalized)) {
    return false;
  }
  return fallback;
};

const readFirstOwnValue = (source, keys) => {
  if (!source || typeof source !== "object") {
    return undefined;
  }
  for (const key of keys) {
    if (Object.prototype.hasOwnProperty.call(source, key)) {
      return source[key];
    }
  }
  return undefined;
};

const readBoundaryGuardValue = (entry, keys) => {
  const nested =
    entry?.unmannedInfo ??
    entry?.UnmannedInfo ??
    null;
  const direct = readFirstOwnValue(entry, keys);
  return direct !== undefined ? direct : readFirstOwnValue(nested, keys);
};

const normalizeBoundaryGuardSetId = (value) => {
  const normalized = String(value ?? "").trim();
  return normalized || null;
};

const mapValueForAgent = (source, agent) => {
  if (source instanceof Map) {
    return source.get(agent) ?? null;
  }
  if (source && typeof source === "object") {
    return source[agent] ?? source[String(agent || "").toLowerCase()] ?? null;
  }
  return null;
};

/**
 * Extract the authoritative boundary-guard loop state from the live SIM frame.
 * Only active rows with a concrete set ID are retained, so leaving the guard
 * mission immediately restores the legacy static isDone rendering contract.
 */
export const buildBoundaryGuardLiveStateIndex = (state) => {
  const result = new Map();
  const vehicles = state?.vehicles && typeof state.vehicles === "object"
    ? state.vehicles
    : {};
  Object.entries(vehicles).forEach(([rawAgent, entry]) => {
    const agent = String(rawAgent || "").trim().toUpperCase();
    if (!agent || !entry || typeof entry !== "object") {
      return;
    }
    const active = coerceBoolean(
      readBoundaryGuardValue(entry, [
        "boundaryGuardLoopActive",
        "BoundaryGuardLoopActive",
        "boundary_guard_loop_active",
      ]),
      false,
    );
    const setId = normalizeBoundaryGuardSetId(
      readBoundaryGuardValue(entry, [
        "boundaryGuardSetID",
        "BoundaryGuardSetID",
        "boundary_guard_set_id",
      ]),
    );
    if (!active || !setId) {
      return;
    }
    const cycleCount = Math.max(
      0,
      coerceInt(
        readBoundaryGuardValue(entry, [
          "boundaryGuardCycleCount",
          "BoundaryGuardCycleCount",
          "boundary_guard_cycle_count",
        ]),
        0,
      ),
    );
    const sequence = coerceInt(
      readBoundaryGuardValue(entry, [
        "boundaryGuardSequence",
        "BoundaryGuardSequence",
        "boundary_guard_sequence",
      ]),
      null,
    );
    const sequenceCount = coerceInt(
      readBoundaryGuardValue(entry, [
        "boundaryGuardSequenceCount",
        "BoundaryGuardSequenceCount",
        "boundary_guard_sequence_count",
      ]),
      null,
    );
    result.set(agent, {
      active: true,
      setId,
      cycleCount,
      sequence: Number.isFinite(sequence) && sequence > 0 ? sequence : null,
      sequenceCount: Number.isFinite(sequenceCount) && sequenceCount > 0
        ? sequenceCount
        : null,
    });
  });
  return result;
};

const boundaryGuardPathContract = (path) => {
  const loop = coerceBoolean(
    path?.boundaryGuardLoop ??
      path?.BoundaryGuardLoop ??
      path?.boundary_guard_loop,
    false,
  );
  const setId = normalizeBoundaryGuardSetId(
    path?.boundaryGuardSetID ??
      path?.BoundaryGuardSetID ??
      path?.boundary_guard_set_id,
  );
  const sequence = coerceInt(
    path?.boundaryGuardSequence ??
      path?.BoundaryGuardSequence ??
      path?.boundary_guard_sequence,
    null,
  );
  const sequenceCount = coerceInt(
    path?.boundaryGuardSequenceCount ??
      path?.BoundaryGuardSequenceCount ??
      path?.boundary_guard_sequence_count,
    null,
  );
  if (
    !loop ||
    !setId ||
    !Number.isFinite(sequence) ||
    sequence <= 0 ||
    !Number.isFinite(sequenceCount) ||
    sequenceCount <= 0 ||
    sequence > sequenceCount
  ) {
    return null;
  }
  return { setId, sequence, sequenceCount };
};

const resolveBoundaryGuardPathRuntime = (
  path,
  currentWaypointsByAgent,
  boundaryGuardStateByAgent,
) => {
  const contract = boundaryGuardPathContract(path);
  const aircraftId = Number(path?.aircraftID ?? path?.AircraftID);
  if (!contract || !Number.isFinite(aircraftId)) {
    return null;
  }
  const agent = agentFromAircraftId(aircraftId);
  const live = mapValueForAgent(boundaryGuardStateByAgent, agent);
  if (
    !live ||
    !coerceBoolean(live.active, false) ||
    normalizeBoundaryGuardSetId(live.setId) !== contract.setId
  ) {
    return null;
  }
  const liveSequenceCount = coerceInt(live.sequenceCount, null);
  // A guard set ID is stable across replans. Reject a frame that still
  // describes the previous plan's child count instead of applying it to the
  // newly loaded path set.
  if (
    Number.isFinite(liveSequenceCount) &&
    liveSequenceCount > 0 &&
    liveSequenceCount !== contract.sequenceCount
  ) {
    return null;
  }
  const waypointList = waypointListOf(path);
  const currentWaypointId = normalizeCurrentWaypointId(
    mapValueForAgent(currentWaypointsByAgent, agent),
  );
  const currentWaypointIndex = waypointList.findIndex(
    (waypoint) =>
      normalizeCurrentWaypointId(waypoint?.waypointID ?? waypoint?.WaypointID) ===
      currentWaypointId,
  );
  const liveSequence = coerceInt(live.sequence, null);
  let pathStatus = null;
  if (currentWaypointIndex >= 0) {
    pathStatus = "active";
  } else if (Number.isFinite(liveSequence) && liveSequence > 0) {
    pathStatus = contract.sequence < liveSequence
      ? "completed"
      : contract.sequence === liveSequence
        ? "active"
        : "planned";
  }
  if (!pathStatus) {
    return null;
  }
  return {
    agent,
    contract,
    live: {
      ...live,
      cycleCount: Math.max(0, coerceInt(live.cycleCount, 0)),
    },
    pathStatus,
    currentWaypointId,
    currentWaypointIndex,
  };
};

const buildBoundaryGuardPathRuntimeIndex = (
  payload,
  currentWaypointsByAgent,
  boundaryGuardStateByAgent,
) => {
  const result = new Map();
  const flightPaths = Array.isArray(payload?.flightPaths) ? payload.flightPaths : [];
  flightPaths.forEach((path) => {
    const pathId = Number(path?.pathID ?? path?.PathID);
    if (!Number.isFinite(pathId)) {
      return;
    }
    const runtime = resolveBoundaryGuardPathRuntime(
      path,
      currentWaypointsByAgent,
      boundaryGuardStateByAgent,
    );
    if (runtime) {
      result.set(pathId, runtime);
    }
  });
  return result;
};

const boundaryGuardWaypointStatus = (runtime, waypointIndex) => {
  if (!runtime) {
    return null;
  }
  if (runtime.pathStatus !== "active") {
    return runtime.pathStatus;
  }
  if (runtime.currentWaypointIndex < 0) {
    return "active";
  }
  if (waypointIndex < runtime.currentWaypointIndex) {
    return "completed";
  }
  if (waypointIndex === runtime.currentWaypointIndex) {
    return "active";
  }
  return "planned";
};

const buildPathMetaKey = (feature) => {
  const agent = String(feature?.agent || "");
  const pathId = feature?.pathId ?? feature?.pathID ?? feature?.pathid ?? null;
  return `${agent}:${String(pathId ?? "")}`;
};

const buildSweepSpacingMap = (payload) => {
  const map = new Map();
  const byInput = payload?.sweepLineSpacingByInputMissionID;
  if (byInput && typeof byInput === "object") {
    Object.entries(byInput).forEach(([key, value]) => {
      const inputMissionId = Number(value?.inputMissionID ?? key);
      if (Number.isFinite(inputMissionId)) {
        map.set(inputMissionId, value);
      }
    });
  }
  const summaries = Array.isArray(payload?.sweepLineSpacingSummaries)
    ? payload.sweepLineSpacingSummaries
    : [];
  summaries.forEach((summary) => {
    const inputMissionId = Number(summary?.inputMissionID);
    if (Number.isFinite(inputMissionId)) {
      map.set(inputMissionId, summary);
    }
  });
  return map;
};

const buildPathMissionMap = (payload) => {
  const map = new Map();
  const rawIndex = payload?.pathMissionIndex;
  if (rawIndex && typeof rawIndex === "object") {
    Object.entries(rawIndex).forEach(([pathKey, value]) => {
      const pathId = Number(value?.pathID ?? pathKey);
      if (!Number.isFinite(pathId)) {
        return;
      }
      const inputMissionId = Number(value?.inputMissionID);
      const individualMissionId = Number(value?.individualMissionID);
      const aircraftId = Number(value?.aircraftID);
      map.set(pathId, {
        inputMissionId: Number.isFinite(inputMissionId) ? inputMissionId : null,
        individualMissionId: Number.isFinite(individualMissionId) ? individualMissionId : null,
        aircraftId: Number.isFinite(aircraftId) ? aircraftId : null,
      });
    });
  }
  const plans = Array.isArray(payload?.individualMissionPlans) ? payload.individualMissionPlans : [];
  plans.forEach((plan) => {
    const aircraftId = Number(plan?.aircraftID);
    const missions = Array.isArray(plan?.individualMissionList) ? plan.individualMissionList : [];
    missions.forEach((mission) => {
      const pathId = Number(mission?.pathID);
      if (!Number.isFinite(pathId) || map.has(pathId)) {
        return;
      }
      const related = mission?.relatedMission || {};
      const inputMissionId = Number(related?.inputMissionID ?? mission?.inputMissionID);
      const individualMissionId = Number(mission?.individualMissionID);
      map.set(pathId, {
        inputMissionId: Number.isFinite(inputMissionId) ? inputMissionId : null,
        individualMissionId: Number.isFinite(individualMissionId) ? individualMissionId : null,
        aircraftId: Number.isFinite(aircraftId) ? aircraftId : null,
      });
    });
  });
  return map;
};

export const buildFiniteRouteCoordinates = (feature) => {
  const coords = Array.isArray(feature?.coords) ? feature.coords : [];
  const closure = feature?.loopClosureCoord;
  if (
    !Array.isArray(closure) ||
    closure.length < 2 ||
    !Number.isFinite(Number(closure[0])) ||
    !Number.isFinite(Number(closure[1])) ||
    !coords.length
  ) {
    return coords;
  }
  const last = coords[coords.length - 1];
  if (
    Array.isArray(last) &&
    Number(last[0]) === Number(closure[0]) &&
    Number(last[1]) === Number(closure[1])
  ) {
    return coords;
  }
  // Return one finite copy for rendering.  Never mutate the waypoint array:
  // marker/label/current-WP code must still see each real waypoint once.
  return [...coords, [Number(closure[0]), Number(closure[1])]];
};

const buildFiniteRouteAltitudes = (feature, routeCoords) => {
  const alts = Array.isArray(feature?.alts) ? feature.alts : [];
  if (!Array.isArray(routeCoords) || routeCoords.length <= alts.length) {
    return alts;
  }
  const closureAlt = Number(feature?.loopClosureAlt);
  return [...alts, Number.isFinite(closureAlt) ? closureAlt : null];
};

const buildGeoFeature = (feature, colors, pathMetaMap = null) => {
  const coords = buildFiniteRouteCoordinates(feature);
  const pathMeta =
    pathMetaMap && typeof pathMetaMap.get === "function"
      ? pathMetaMap.get(buildPathMetaKey(feature)) || null
      : null;
  const geometry =
    coords.length === 1
      ? {
          type: "Point",
          coordinates: coords[0],
        }
      : {
          type: "LineString",
          coordinates: coords,
        };
  return {
    type: "Feature",
    id: feature.id,
    geometry,
    properties: {
      agent: feature.agent,
      pathId: feature.pathId,
      isDone: feature.isDone ? 1 : 0,
      aircraftId: feature.aircraftId,
      points: feature.points,
      altMin: feature.altMin,
      altMax: feature.altMax,
      color: colors[feature.agent] || "#e7eddc",
      passSummary: pathMeta?.passSummary || feature.passSummary || null,
      boundaryGuardLoop: feature.boundaryGuardLoop ? 1 : 0,
      boundaryGuardSetID: feature.boundaryGuardSetID || null,
    },
  };
};

const sameAreaPoint = (left, right) =>
  Array.isArray(left) &&
  Array.isArray(right) &&
  left.length >= 2 &&
  right.length >= 2 &&
  left[0] === right[0] &&
  left[1] === right[1];

const normalizeAreaRing = (coordinateList) => {
  if (!Array.isArray(coordinateList)) {
    return null;
  }
  const ring = [];
  coordinateList.forEach((coord) => {
    const lat = Number(coord?.latitude);
    const lon = Number(coord?.longitude);
    if (!Number.isFinite(lat) || !Number.isFinite(lon)) {
      return;
    }
    const point = [lon, lat];
    if (!ring.length || !sameAreaPoint(ring[ring.length - 1], point)) {
      ring.push(point);
    }
  });
  if (ring.length >= 2 && sameAreaPoint(ring[0], ring[ring.length - 1])) {
    ring.pop();
  }
  if (ring.length < 3) {
    return null;
  }
  ring.push([...ring[0]]);
  return ring;
};

const areaRingAbsArea = (ring) => {
  let twiceArea = 0;
  for (let index = 0; index < ring.length - 1; index += 1) {
    twiceArea += ring[index][0] * ring[index + 1][1] - ring[index + 1][0] * ring[index][1];
  }
  return Math.abs(twiceArea) / 2;
};

const areaRingContainsPoint = (ring, point) => {
  let inside = false;
  const x = point[0];
  const y = point[1];
  for (let index = 0, previous = ring.length - 2; index < ring.length - 1; previous = index++) {
    const xi = ring[index][0];
    const yi = ring[index][1];
    const xj = ring[previous][0];
    const yj = ring[previous][1];
    const crosses = yi > y !== yj > y && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi;
    if (crosses) {
      inside = !inside;
    }
  }
  return inside;
};

/**
 * Convert one mission's ICD area rows to GeoJSON Polygon coordinates.  Holes
 * are attached to the smallest containing outer ring before owner polygons are
 * collected into a single MultiPolygon feature.
 */
const buildMissionAreaPolygons = (areaList) => {
  const outerPolygons = [];
  const holeRings = [];
  (Array.isArray(areaList) ? areaList : []).forEach((area) => {
    if (!area) {
      return;
    }
    const ring = normalizeAreaRing(area.coordinateList);
    if (!ring) {
      return;
    }
    if (area.isHole) {
      holeRings.push(ring);
      return;
    }
    outerPolygons.push({ rings: [ring], area: areaRingAbsArea(ring) });
  });
  holeRings.forEach((holeRing) => {
    const sample = holeRing[0];
    const container = outerPolygons
      .filter((polygon) => areaRingContainsPoint(polygon.rings[0], sample))
      .sort((left, right) => left.area - right.area)[0];
    if (container) {
      container.rings.push(holeRing);
    }
  });
  return outerPolygons.map((polygon) => polygon.rings);
};

/**
 * Render one logical assignment for each aircraft and coverage pass.
 * Reciprocal Area plans therefore expose exactly one OUT and one RETURN
 * feature per aircraft.  Legacy single-pass plans without an explicit pass
 * keep the previous (inputMissionID, aircraftID) grouping.
 */
export const buildAreaFeatures = (
  payload,
  colors,
  selectedAgent,
  currentWaypointsByAgent = new Map(),
  boundaryGuardStateByAgent = new Map(),
) => {
  const plans = Array.isArray(payload?.individualMissionPlans) ? payload.individualMissionPlans : [];
  const pathMissionMap = buildPathMissionMap(payload);
  const sweepSpacingMap = buildSweepSpacingMap(payload);
  const boundaryGuardRuntimeByPath = buildBoundaryGuardPathRuntimeIndex(
    payload,
    currentWaypointsByAgent,
    boundaryGuardStateByAgent,
  );
  const hasSelection = Boolean(selectedAgent);
  const ownerGroups = new Map();
  plans.forEach((plan) => {
    const aircraftId = Number(plan?.aircraftID);
    if (!Number.isFinite(aircraftId)) {
      return;
    }
    const agent = agentFromAircraftId(aircraftId);
    const baseColor = colors[agent] || "#e7eddc";
    const missions = Array.isArray(plan?.individualMissionList) ? plan.individualMissionList : [];
    missions.forEach((mission) => {
      const missionId = Number(mission?.individualMissionID);
      const pathId = Number(mission?.pathID);
      const missionInfo = pathMissionMap.get(pathId) || {};
      const rawInputMissionId = missionInfo.inputMissionId;
      const parsedInputMissionId = Number(rawInputMissionId);
      const inputMissionId = rawInputMissionId !== null &&
        rawInputMissionId !== undefined &&
        Number.isFinite(parsedInputMissionId)
        ? parsedInputMissionId
        : null;
      const spacing = inputMissionId !== null ? sweepSpacingMap.get(inputMissionId) : null;
      const boundaryGuardRuntime = Number.isFinite(pathId)
        ? boundaryGuardRuntimeByPath.get(pathId) || null
        : null;
      const missionStatus = boundaryGuardRuntime?.pathStatus || null;
      const isDone = missionStatus
        ? missionStatus === "completed"
        : Boolean(mission?.isDone);
      // AREA child missions are consecutive pieces of one owner assignment.
      // Once a child is complete, keeping its polygon in this owner feature
      // makes the already-covered strip look active until every sibling is
      // complete. The active boundary-guard cycle deliberately overrides the
      // persisted first-pass isDone flags, allowing planned/current children
      // to reappear immediately after the last-WP -> first-WP wrap.
      if (isDone) {
        return;
      }
      const info = mission?.individualMissionInfo || {};
      const detailPasses = Array.isArray(info?.coveragePassDetails)
        ? info.coveragePassDetails
        : Array.isArray(mission?.coveragePassDetails) ? mission.coveragePassDetails : [];
      const singleDetailPass = detailPasses.length === 1
        ? detailPasses[0]?.coveragePass
        : null;
      const coveragePass = normalizeAreaCoveragePass(
        info?.areaAssignedCoveragePass ??
        mission?.areaAssignedCoveragePass ??
        info?.activeCoveragePass ??
        mission?.activeCoveragePass ??
        singleDetailPass,
      );
      const areaList = Array.isArray(info?.areaList) ? info.areaList : [];
      const polygons = buildMissionAreaPolygons(areaList);
      if (!polygons.length) {
        return;
      }
      const fallbackIdentity = Number.isFinite(pathId)
        ? `path:${pathId}`
        : Number.isFinite(missionId) ? `mission:${missionId}` : "unknown";
      const ownerBaseKey = `${aircraftId}:${inputMissionId ?? fallbackIdentity}`;
      const ownerKey = coveragePass
        ? `${ownerBaseKey}:${coveragePass}`
        : ownerBaseKey;
      if (!ownerGroups.has(ownerKey)) {
        ownerGroups.set(ownerKey, {
          ownerKey,
          agent,
          aircraftId,
          inputMissionId,
          coveragePass,
          baseColor,
          spacing,
          polygons: [],
          missionIds: new Set(),
          pathIds: new Set(),
          allDone: true,
          boundaryGuardSetId: null,
          boundaryGuardCycleCount: null,
          boundaryGuardStatuses: new Set(),
        });
      }
      const group = ownerGroups.get(ownerKey);
      group.polygons.push(...polygons);
      group.allDone = group.allDone && isDone;
      if (boundaryGuardRuntime) {
        group.boundaryGuardSetId = boundaryGuardRuntime.contract.setId;
        group.boundaryGuardCycleCount = boundaryGuardRuntime.live.cycleCount;
        group.boundaryGuardStatuses.add(boundaryGuardRuntime.pathStatus);
      }
      if (!group.spacing && spacing) {
        group.spacing = spacing;
      }
      if (Number.isFinite(missionId)) {
        group.missionIds.add(missionId);
      }
      if (Number.isFinite(pathId)) {
        group.pathIds.add(pathId);
      }
    });
  });
  let featureId = 1;
  return Array.from(ownerGroups.values()).map((group) => {
    const missionIds = Array.from(group.missionIds);
    const pathIds = Array.from(group.pathIds);
    const isDone = group.allDone;
    const boundaryGuardStatus = group.boundaryGuardStatuses.has("active")
      ? "active"
      : group.boundaryGuardStatuses.has("planned")
        ? "planned"
        : group.boundaryGuardStatuses.has("completed")
          ? "completed"
          : null;
    const fillOpacity = hasSelection
      ? group.agent === selectedAgent
        ? isDone ? AREA_FILL_DONE_ALPHA : AREA_FILL_ALPHA
        : isDone ? AREA_FILL_DONE_DIM_ALPHA : AREA_FILL_DIM_ALPHA
      : isDone ? AREA_FILL_DONE_ALPHA : AREA_FILL_ALPHA;
    const lineOpacity = hasSelection
      ? group.agent === selectedAgent
        ? isDone ? AREA_LINE_DONE_ALPHA : AREA_LINE_ALPHA
        : isDone ? AREA_LINE_DONE_DIM_ALPHA : AREA_LINE_DIM_ALPHA
      : isDone ? AREA_LINE_DONE_ALPHA : AREA_LINE_ALPHA;
    return {
      type: "Feature",
      id: featureId++,
      geometry: {
        type: "MultiPolygon",
        coordinates: group.polygons,
      },
      properties: {
        visualizationRole: "assignmentOwner",
        ownerKey: group.ownerKey,
        agent: group.agent,
        aircraftId: group.aircraftId,
        inputMissionId: group.inputMissionId,
        missionId: missionIds.length === 1 ? missionIds[0] : null,
        pathId: pathIds.length === 1 ? pathIds[0] : null,
        missionIds: missionIds.join(","),
        pathIds: pathIds.join(","),
        missionCount: missionIds.length,
        pathCount: pathIds.length,
        areaPartCount: group.polygons.length,
        // The browser has no polygon-boolean dependency.  A server-side
        // unary-union can replace these coordinates without changing the
        // one-feature-per-owner rendering contract.
        topologicallyDissolved: 0,
        sweepAvgSpacingM: Number.isFinite(Number(group.spacing?.averageLineSpacingM))
          ? Number(group.spacing.averageLineSpacingM)
          : null,
        sweepLineCount: Number.isFinite(Number(group.spacing?.lineCount))
          ? Number(group.spacing.lineCount)
          : null,
        sweepPairCount: Number.isFinite(Number(group.spacing?.pairCount))
          ? Number(group.spacing.pairCount)
          : null,
        isDone: isDone ? 1 : 0,
        coveragePass: group.coveragePass,
        coveragePassLabel: group.coveragePass
          ? AREA_COVERAGE_PASS_STYLES[group.coveragePass]?.label || null
          : null,
        coveragePassStatus: group.coveragePass
          ? isDone ? "completed" : "active"
          : null,
        boundaryGuardSetID: group.boundaryGuardSetId,
        boundaryGuardCycleCount: group.boundaryGuardCycleCount,
        boundaryGuardStatus,
        color: group.baseColor,
        fillOpacity,
        lineOpacity,
      },
    };
  });
};

export const buildSweepFeatures = (
  payload,
  colors,
  selectedAgent,
  selectedWaypoint,
  currentWaypointsByAgent = new Map(),
  boundaryGuardStateByAgent = new Map(),
) => {
  const flightPaths = Array.isArray(payload?.flightPaths) ? payload.flightPaths : [];
  const pathMissionMap = buildPathMissionMap(payload);
  const sweepSpacingMap = buildSweepSpacingMap(payload);
  const coveragePassIndex = buildAreaCoveragePassIndex(payload, currentWaypointsByAgent);
  const boundaryGuardRuntimeByPath = buildBoundaryGuardPathRuntimeIndex(
    payload,
    currentWaypointsByAgent,
    boundaryGuardStateByAgent,
  );
  const hasSelection = Boolean(selectedAgent);
  const hasWaypointSelection = Boolean(selectedWaypoint);
  const features = [];
  let featureId = 1;
  flightPaths.forEach((path) => {
    const aircraftId = Number(path?.aircraftID);
    if (!Number.isFinite(aircraftId)) {
      return;
    }
    const agent = agentFromAircraftId(aircraftId);
    const agentColor = colors[agent] || "#e7eddc";
    const pathId = Number(path?.pathID);
    const missionInfo = pathMissionMap.get(pathId) || {};
    const inputMissionId = Number(missionInfo.inputMissionId);
    const spacing = Number.isFinite(inputMissionId) ? sweepSpacingMap.get(inputMissionId) : null;
    const waypointList = waypointListOf(path);
    const boundaryGuardRuntime = Number.isFinite(pathId)
      ? boundaryGuardRuntimeByPath.get(pathId) || null
      : null;
    const passCounts = { 1: 0, 2: 0, 3: 0 };
    waypointList.forEach((wp, waypointIndex) => {
      const waypointId = Number(wp?.waypointID);
      const coveragePass = normalizeAreaCoveragePass(
        wp?.areaCoveragePass ?? wp?.AreaCoveragePass,
      );
      const coverageInfo = coveragePass
        ? coveragePassIndex.get(coveragePassMapKey(pathId, coveragePass)) || null
        : null;
      const isReciprocalCoverage = Boolean(coverageInfo?.reciprocal);
      const guardWaypointStatus = boundaryGuardWaypointStatus(
        boundaryGuardRuntime,
        waypointIndex,
      );
      const coverageStatus = guardWaypointStatus ||
        (isReciprocalCoverage
          ? coverageInfo.status
          : Boolean(wp?.isDone) ? "completed" : "planned");
      const isDone = coverageStatus === "completed";
      const isCurrentCoverage = coverageStatus === "active";
      const coverageStyle = isReciprocalCoverage
        ? AREA_COVERAGE_PASS_STYLES[coveragePass]
        : null;
      const baseColor = coverageStyle?.color || agentColor;
      const passInfo = getWaypointPassInfo(wp);
      if (Number.isFinite(passInfo.passType) && passCounts[passInfo.passType] !== undefined) {
        passCounts[passInfo.passType] += 1;
      }
      const filming = wp?.filmingProperty || {};
      const { coordinateList, interpolationPoints } = getFilmingSweepCoordinateList(filming);
      const coords = coordinateList
        .map((coord) => {
          const lat = Number(coord?.latitude);
          const lon = Number(coord?.longitude);
          if (!Number.isFinite(lat) || !Number.isFinite(lon)) {
            return null;
          }
          return [lon, lat];
        })
        .filter(Boolean);
      if (coords.length < 1) {
        return;
      }
      const altitudes = coordinateList.map((coord) => Number(coord?.altitude)).filter(Number.isFinite);
      const minAlt = altitudes.length ? Math.min(...altitudes) : null;
      const maxAlt = altitudes.length ? Math.max(...altitudes) : null;
      const selectedPathId = Number(selectedWaypoint?.pathId);
      const selectedWaypointId = Number(selectedWaypoint?.waypointId);
      const isSelectedWaypoint =
        hasWaypointSelection &&
        (!selectedWaypoint?.agent || selectedWaypoint.agent === agent) &&
        (!Number.isFinite(selectedPathId) || selectedPathId === pathId) &&
        Number.isFinite(selectedWaypointId) &&
        Number.isFinite(waypointId) &&
        selectedWaypointId === waypointId;
      const baseLineOpacity = isReciprocalCoverage
        ? (hasSelection && agent !== selectedAgent
          ? isCurrentCoverage ? 0.28 : 0.09
          : isCurrentCoverage ? 1.0 : isDone ? 0.22 : 0.48)
        : hasSelection
          ? agent === selectedAgent
            ? isDone ? SWEEP_LINE_DONE_ALPHA : SWEEP_LINE_ALPHA
            : isDone ? SWEEP_LINE_DONE_DIM_ALPHA : SWEEP_LINE_DIM_ALPHA
          : isDone ? SWEEP_LINE_DONE_ALPHA : SWEEP_LINE_ALPHA;
      const basePointOpacity = isReciprocalCoverage
        ? (hasSelection && agent !== selectedAgent
          ? isCurrentCoverage ? 0.34 : 0.12
          : isCurrentCoverage ? 1.0 : isDone ? 0.3 : 0.58)
        : hasSelection
          ? agent === selectedAgent
            ? isDone ? SWEEP_POINT_DONE_ALPHA : SWEEP_POINT_ALPHA
            : isDone ? SWEEP_POINT_DONE_DIM_ALPHA : SWEEP_POINT_DIM_ALPHA
          : isDone ? SWEEP_POINT_DONE_ALPHA : SWEEP_POINT_ALPHA;
      const lineOpacity = isSelectedWaypoint
        ? isDone ? SWEEP_SELECTED_LINE_DONE_ALPHA : SWEEP_SELECTED_LINE_ALPHA
        : hasWaypointSelection
          ? isDone ? SWEEP_WAYPOINT_CONTEXT_DONE_ALPHA : SWEEP_WAYPOINT_CONTEXT_LINE_ALPHA
          : baseLineOpacity;
      const pointOpacity = isSelectedWaypoint
        ? isDone ? SWEEP_SELECTED_POINT_DONE_ALPHA : SWEEP_SELECTED_POINT_ALPHA
        : hasWaypointSelection
          ? isDone ? SWEEP_WAYPOINT_CONTEXT_POINT_DONE_ALPHA : SWEEP_WAYPOINT_CONTEXT_POINT_ALPHA
          : basePointOpacity;
      const pointStrokeOpacity = Math.min(1, pointOpacity + 0.12);
      const commonProps = {
        agent,
        aircraftId,
        pathId: Number.isFinite(pathId) ? pathId : null,
        inputMissionId: Number.isFinite(inputMissionId) ? inputMissionId : null,
        sweepAvgSpacingM: Number.isFinite(Number(spacing?.averageLineSpacingM))
          ? Number(spacing.averageLineSpacingM)
          : null,
        sweepLineCount: Number.isFinite(Number(spacing?.lineCount)) ? Number(spacing.lineCount) : null,
        sweepPairCount: Number.isFinite(Number(spacing?.pairCount)) ? Number(spacing.pairCount) : null,
        waypointId: Number.isFinite(waypointId) ? waypointId : null,
        pointCount: coords.length,
        isDone: isDone ? 1 : 0,
        color: baseColor,
        coveragePass: isReciprocalCoverage ? coveragePass : null,
        coveragePassIndex: isReciprocalCoverage ? coverageInfo?.passIndex || null : null,
        passIndex: isReciprocalCoverage ? coverageInfo?.passIndex || null : null,
        coveragePassLabel: coverageStyle?.label || null,
        coveragePassStatus: isReciprocalCoverage ? coverageStatus : null,
        isCurrentCoverage: isCurrentCoverage ? 1 : 0,
        boundaryGuardSetID: boundaryGuardRuntime?.contract.setId || null,
        boundaryGuardCycleCount: boundaryGuardRuntime?.live.cycleCount ?? null,
        boundaryGuardSequence: boundaryGuardRuntime?.contract.sequence ?? null,
        boundaryGuardSequenceCount: boundaryGuardRuntime?.contract.sequenceCount ?? null,
        boundaryGuardStatus: guardWaypointStatus,
        altMin: minAlt,
        altMax: maxAlt,
        passType: Number.isFinite(passInfo.passType) ? passInfo.passType : null,
        passLabel: passInfo.passLabel,
        loiterSummary: passInfo.loiterSummary,
        speedMps: passInfo.speedMps,
        hoverSeconds: passInfo.hoverSeconds,
        etaSeconds: passInfo.etaSeconds,
        fovDeg: passInfo.fovDeg,
        fovLabel: passInfo.fovLabel,
        isLoiter: passInfo.isLoiter ? 1 : 0,
        isFlyBy: passInfo.isFlyBy ? 1 : 0,
        isFlyOver: passInfo.isFlyOver ? 1 : 0,
        isSelectedWaypoint: isSelectedWaypoint ? 1 : 0,
      };
      const passSummary = [
        passCounts[1] ? `Fly-by ${passCounts[1]}` : null,
        passCounts[2] ? `Loiter ${passCounts[2]}` : null,
        passCounts[3] ? `Fly-over ${passCounts[3]}` : null,
      ]
        .filter(Boolean)
        .join(" · ");
      const sweepSegments = [];
      if (coords.length >= 2) {
        const chunkSize = Math.max(2, Number(interpolationPoints) || 0);
        if (chunkSize <= 2 || coords.length <= chunkSize) {
          sweepSegments.push(coords);
        } else {
          for (let base = 0; base < coords.length; base += chunkSize) {
            const seg = coords.slice(base, base + chunkSize);
            if (seg.length >= 2) {
              sweepSegments.push(seg);
            }
          }
        }
      }
      sweepSegments.forEach((segmentCoords, segmentIdx) => {
        features.push({
          type: "Feature",
          id: featureId++,
          geometry: {
            type: "LineString",
            coordinates: segmentCoords,
          },
          properties: {
            ...commonProps,
            featureKind: "line",
            lineOpacity,
            passSummary,
            segmentIndex: segmentIdx,
            segmentPointCount: segmentCoords.length,
            lineWidthBoost: isSelectedWaypoint ? 1 : 0,
          },
        });
      });
      coordinateList.forEach((coord, pointIdx) => {
        const lon = Number(coord?.longitude);
        const lat = Number(coord?.latitude);
        if (!Number.isFinite(lon) || !Number.isFinite(lat)) {
          return;
        }
        const isEndpoint = pointIdx === 0 || pointIdx === coords.length - 1;
        const basePointRadius = isSelectedWaypoint
          ? (isEndpoint ? 7.2 : 5.6)
          : isEndpoint ? 4.5 : 3.2;
        const coverageRadiusOffset = coveragePass === "forward"
          ? 0.9
          : coveragePass === "reverse" ? -0.35 : 0;
        features.push({
          type: "Feature",
          id: featureId++,
          geometry: {
            type: "Point",
            coordinates: [lon, lat],
          },
          properties: {
            ...commonProps,
            featureKind: "point",
            pointIndex: pointIdx + 1,
            altitude: Number(coord?.altitude),
            pointOpacity,
            pointStrokeOpacity,
            // The reverse coordinates normally overlap the forward pass
            // exactly.  A smaller return marker leaves the outbound ring
            // visible instead of visually erasing the first managed pass.
            pointRadius: Math.max(1.5, basePointRadius + coverageRadiusOffset),
            pointStrokeWidth: isSelectedWaypoint ? 2.4 : isEndpoint ? 1.8 : 1.1,
            isEndpoint: isEndpoint ? 1 : 0,
          },
        });
      });
    });
  });
  return features;
};

const createLineLayer3d = (id, color, drawModeName, lineWidth) => {
  const layer = {
    id,
    type: "custom",
    renderingMode: "3d",
    _color: color,
    _alpha: DEFAULT_ALPHA,
    _lineWidth: lineWidth,
    _drawModeName: drawModeName,
    _visible: true,
    _useDepth: false,
    _lineCount: 0,
    _pendingPositions: null,
    setAlpha(nextAlpha) {
      const value = Number(nextAlpha);
      if (!Number.isFinite(value)) {
        return;
      }
      this._alpha = Math.max(0, Math.min(1, value));
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
      this._drawMode =
        this._drawModeName === "LINE_STRIP"
          ? gl.LINE_STRIP
          : this._drawModeName === "TRIANGLES"
            ? gl.TRIANGLES
            : gl.LINES;
      if (gl.createVertexArray) {
        this._vao = gl.createVertexArray();
        gl.bindVertexArray(this._vao);
        gl.bindBuffer(gl.ARRAY_BUFFER, this._lineBuffer);
        gl.enableVertexAttribArray(this._aPos);
        gl.vertexAttribPointer(this._aPos, 3, gl.FLOAT, false, 0, 0);
        gl.bindVertexArray(null);
      }
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
      gl.uniform4fv(this._uColor, hexToRgba(this._color, this._alpha));
      if (gl.bindVertexArray && this._vao) {
        gl.bindVertexArray(this._vao);
      } else {
        gl.bindBuffer(gl.ARRAY_BUFFER, this._lineBuffer);
        gl.enableVertexAttribArray(this._aPos);
        gl.vertexAttribPointer(this._aPos, 3, gl.FLOAT, false, 0, 0);
      }
      gl.enable(gl.BLEND);
      gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
      gl.lineWidth(this._lineWidth);
      gl.drawArrays(this._drawMode, 0, this._lineCount);
      if (gl.bindVertexArray && this._vao) {
        gl.bindVertexArray(null);
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

const createPointLayer3d = (id, color, pointSize) => {
  const layer = {
    id,
    type: "custom",
    renderingMode: "3d",
    _color: color,
    _alpha: DEFAULT_ALPHA,
    _pointSize: pointSize,
    _visible: true,
    _useDepth: false,
    _pointCount: 0,
    _pendingPositions: null,
    setAlpha(nextAlpha) {
      const value = Number(nextAlpha);
      if (!Number.isFinite(value)) {
        return;
      }
      this._alpha = Math.max(0, Math.min(1, value));
    },
    setColor(nextColor) {
      if (typeof nextColor !== "string" || !nextColor.trim()) {
        return;
      }
      this._color = nextColor.trim();
    },
    setSize(nextSize) {
      const value = Number(nextSize);
      if (!Number.isFinite(value) || value <= 0) {
        return;
      }
      this._pointSize = value;
    },
    setVisible(nextVisible) {
      this._visible = Boolean(nextVisible);
    },
    updatePositions(positions) {
      this._pendingPositions = positions;
      if (!this._gl || !this._pointBuffer) {
        return;
      }
      const gl = this._gl;
      const data = new Float32Array(positions);
      gl.bindBuffer(gl.ARRAY_BUFFER, this._pointBuffer);
      gl.bufferData(gl.ARRAY_BUFFER, data, gl.DYNAMIC_DRAW);
      this._pointCount = data.length / 3;
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
          uniform float u_size;
          void main() {
            gl_Position = u_matrix * vec4(a_pos, 1.0);
            gl_PointSize = u_size;
          }`
        : `
          attribute vec3 a_pos;
          uniform mat4 u_matrix;
          uniform float u_size;
          void main() {
            gl_Position = u_matrix * vec4(a_pos, 1.0);
            gl_PointSize = u_size;
          }`;
      const fragmentSource = isWebGL2
        ? `#version 300 es
          precision mediump float;
          uniform vec4 u_color;
          out vec4 fragColor;
          void main() {
            vec2 uv = gl_PointCoord - 0.5;
            float r = length(uv);
            if (r > 0.5) {
              discard;
            }
            float edge = smoothstep(0.48, 0.5, r);
            vec4 color = u_color;
            color.a *= (1.0 - edge);
            fragColor = color;
          }`
        : `
          precision mediump float;
          uniform vec4 u_color;
          void main() {
            vec2 uv = gl_PointCoord - 0.5;
            float r = length(uv);
            if (r > 0.5) {
              discard;
            }
            float edge = smoothstep(0.48, 0.5, r);
            vec4 color = u_color;
            color.a *= (1.0 - edge);
            gl_FragColor = color;
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
      this._uSize = gl.getUniformLocation(program, "u_size");
      this._pointBuffer = gl.createBuffer();
      gl.bindBuffer(gl.ARRAY_BUFFER, this._pointBuffer);
      gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([]), gl.DYNAMIC_DRAW);
      this._pointCount = 0;
      if (this._pendingPositions) {
        this.updatePositions(this._pendingPositions);
      }
    },
    render(gl, argsOrMatrix) {
      const mat = getProjectionMatrix(argsOrMatrix);
      if (typeof this._renderHook === "function" && mat && mat.length === 16) {
        this._renderHook(mat);
      }
      if (!this._program || !this._visible || !this._pointCount) {
        return;
      }
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
      gl.uniform4fv(this._uColor, hexToRgba(this._color, this._alpha));
      gl.uniform1f(this._uSize, this._pointSize);
      if (typeof gl.PROGRAM_POINT_SIZE !== "undefined") {
        gl.enable(gl.PROGRAM_POINT_SIZE);
      }
      gl.bindBuffer(gl.ARRAY_BUFFER, this._pointBuffer);
      gl.enableVertexAttribArray(this._aPos);
      gl.vertexAttribPointer(this._aPos, 3, gl.FLOAT, false, 0, 0);
      gl.enable(gl.BLEND);
      gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
      gl.drawArrays(gl.POINTS, 0, this._pointCount);
      if (typeof gl.PROGRAM_POINT_SIZE !== "undefined") {
        gl.disable(gl.PROGRAM_POINT_SIZE);
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

export const initMissionPaths = (map) => {
  let pendingData = null;
  let selectedAgent = null;
  let selectedWaypoint = null;
  let waypointsVisible = true;
  let sweepLinesVisible = true;
  let currentWaypointsByAgent = new Map();
  let currentCoverageStateByAgent = new Map();
  let currentBoundaryGuardStateByAgent = new Map();
  let features = [];
  let areaFeatures = [];
  let sweepFeatures = [];
  let agentCounts = {};
  let pathMetaByKey = new Map();
  let mapReady = typeof map.isStyleLoaded === "function" ? map.isStyleLoaded() : map.loaded();
  let popup = null;
  let visualRebuildScheduled = false;
  let visualRebuildRevision = 0;
  let renderedVisualRevision = -1;
  let renderedZoomBucket = null;
  let didFitBounds = false;
  let interactionsAttached = false;

  const legend = document.getElementById("mission-legend");
  const legendItems = Array.from(document.querySelectorAll(".mission-legend-item"));
  const coveragePassLegend = document.getElementById("mission-area-pass-legend");
  const coverageDepthLegend = document.getElementById("mission-area-depth-legend");
  const coveragePassLegendTitle = document.getElementById("mission-area-pass-title");
  const coveragePassLegendItems = Array.from(
    document.querySelectorAll(".mission-area-pass-item[data-coverage-pass]"),
  );
  const colors = getAgentColors();
  const agentLayers = new Map();
  const doneAgentLayers = new Map();
  const waypointLayers = new Map();
  const doneWaypointLayers = new Map();
  const currentWaypointLayers = new Map();
  let labelContainer = null;
  const labelElements = new Map();
  let waypointLabelEntries = [];
  let visibleWaypointLabelKeys = new Set();
  let labelHookLayer = null;

  const setLegendVisibility = (visible) => {
    if (!legend) {
      return;
    }
    legend.classList.toggle("is-visible", visible);
    legend.setAttribute("aria-hidden", visible ? "false" : "true");
  };

  const updateCoveragePassLegend = () => {
    if (!coveragePassLegend) {
      return;
    }
    const index = buildAreaCoveragePassIndex(pendingData, currentWaypointsByAgent);
    const allEntries = Array.from(index.values()).filter((entry) => entry.reciprocal);
    const selectedEntries = selectedAgent
      ? allEntries.filter((entry) => entry.agent === selectedAgent)
      : [];
    const visibleEntries = selectedEntries.length ? selectedEntries : allEntries;
    const visible = visibleEntries.length > 0;
    const remainingPassAvailable = coveragePassLegend.dataset.remainingPassAvailable === "1";
    coveragePassLegend.hidden = !(visible || remainingPassAvailable);
    if (!visible) {
      return;
    }
    if (coveragePassLegendTitle) {
      const liveEntries = Array.from(currentCoverageStateByAgent.entries())
        .filter(([agent]) => !selectedEntries.length || agent === selectedAgent)
        .map(([, state]) => state);
      const activeTurn = liveEntries.find((state) => state.turnPhase);
      const baseTitle = selectedEntries.length
        ? `AREA PATH ATTRIBUTION · ${selectedAgent}`
        : "AREA PATH ATTRIBUTION";
      coveragePassLegendTitle.textContent = activeTurn
        ? `${baseTitle} · TURNING`
        : baseTitle;
      coveragePassLegendTitle.title = activeTurn
        ? [activeTurn.turnPhase, activeTurn.turnRole].filter(Boolean).join(" · ")
        : "";
    }
    coveragePassLegendItems.forEach((item) => {
      const pass = normalizeAreaCoveragePass(item.dataset.coveragePass);
      const entries = visibleEntries.filter((entry) => entry.pass === pass);
      const activeCount = entries.filter((entry) => entry.status === "active").length;
      const completedCount = entries.filter((entry) => entry.status === "completed").length;
      const allCompleted = entries.length > 0 && completedCount === entries.length;
      const status = activeCount > 0 ? "active" : allCompleted ? "completed" : "planned";
      item.classList.toggle("is-active", status === "active");
      item.classList.toggle("is-completed", status === "completed");
      item.dataset.status = status;
      const statusEl = item.querySelector(".mission-area-pass-status");
      if (statusEl) {
        statusEl.textContent = activeCount > 0
          ? `ACTIVE ${activeCount}`
          : allCompleted
            ? "DONE"
            : completedCount > 0
              ? `DONE ${completedCount}/${entries.length}`
              : "PLANNED";
      }
    });
  };

  const updateLegend = () => {
    if (legendItems.length === 0) {
      updateCoveragePassLegend();
      return;
    }
    legendItems.forEach((item) => {
      const agent = item.dataset.agent || "";
      const available = Boolean(agentCounts[agent]);
      item.classList.toggle("is-disabled", !available);
      item.classList.toggle("is-active", selectedAgent === agent);
      item.setAttribute("aria-disabled", available ? "false" : "true");
    });
    updateCoveragePassLegend();
  };

  const applySweepLineVisibility = () => {
    const visibility = sweepLinesVisible ? "visible" : "none";
    [
      SWEEP_LINE_LAYER_ID,
      SWEEP_FORWARD_LINE_LAYER_ID,
      SWEEP_REVERSE_LINE_LAYER_ID,
      SWEEP_POINT_LAYER_ID,
    ].forEach((layerId) => {
      if (!map.getLayer(layerId)) {
        return;
      }
      map.setLayoutProperty(layerId, "visibility", visibility);
    });
  };

  const buildPathMetaMap = (payload) => {
    const meta = new Map();
    const flightPaths = Array.isArray(payload?.flightPaths) ? payload.flightPaths : [];
    const pathMissionMap = buildPathMissionMap(payload);
    const sweepSpacingMap = buildSweepSpacingMap(payload);
    flightPaths.forEach((path) => {
      const aircraftId = Number(path?.aircraftID);
      const agent =
        aircraftId >= 1 && aircraftId <= 3
          ? `LAH${aircraftId}`
          : aircraftId >= 4 && aircraftId <= 6
            ? `UAV${aircraftId - 3}`
            : null;
      if (!agent) {
        return;
      }
      const pathId = Number(path?.pathID);
      const missionInfo = pathMissionMap.get(pathId) || {};
      const inputMissionId = Number(missionInfo.inputMissionId);
      const sweepSpacingSummary = Number.isFinite(inputMissionId)
        ? sweepSpacingMap.get(inputMissionId) || null
        : null;
      const waypointList = waypointListOf(path);
      const isManned = aircraftId >= 1 && aircraftId <= 3;
      const concealIds = isManned ? concealIdsForPath(payload, pathId) : null;
      const passCounts = { 1: 0, 2: 0, 3: 0 };
      const roleCounts = { attack: 0, conceal: 0, hover: 0, loiter: 0, transit: 0 };
      const waypointModes = new Map();
      waypointList.forEach((wp) => {
        const waypointId = normalizeCurrentWaypointId(wp?.waypointID ?? wp?.WaypointID);
        if (!Number.isFinite(waypointId)) {
          return;
        }
        const passInfo = getWaypointPassInfo(wp);
        if (Number.isFinite(passInfo.passType) && passCounts[passInfo.passType] !== undefined) {
          passCounts[passInfo.passType] += 1;
        }
        const lahRole = isManned ? getLahPointRole(wp, concealIds) : null;
        if (lahRole && roleCounts[lahRole] !== undefined) {
          roleCounts[lahRole] += 1;
        }
        waypointModes.set(waypointId, {
          passType: passInfo.passType,
          passLabel: passInfo.passLabel,
          loiterSummary: passInfo.loiterSummary,
          speedMps: passInfo.speedMps,
          hoverSeconds: passInfo.hoverSeconds,
          etaSeconds: passInfo.etaSeconds,
          fovDeg: passInfo.fovDeg,
          fovLabel: passInfo.fovLabel,
          isLoiter: passInfo.isLoiter,
          isFlyBy: passInfo.isFlyBy,
          isFlyOver: passInfo.isFlyOver,
          coveragePass: passInfo.coveragePass,
          coveragePassLabel: passInfo.coveragePassLabel,
          lahRole,
          lahRoleSummary: lahRole ? lahRoleSummary(wp, lahRole) : null,
        });
      });
      meta.set(buildPathMetaKey({ agent, pathId }), {
        agent,
        pathId: Number.isFinite(pathId) ? pathId : null,
        inputMissionId: Number.isFinite(inputMissionId) ? inputMissionId : null,
        individualMissionId: Number.isFinite(Number(missionInfo.individualMissionId))
          ? Number(missionInfo.individualMissionId)
          : null,
        sweepSpacingSummary,
        waypointCount: waypointList.length,
        passSummary: [
          passCounts[1] ? `Fly-by ${passCounts[1]}` : null,
          passCounts[2] ? `Loiter ${passCounts[2]}` : null,
          passCounts[3] ? `Fly-over ${passCounts[3]}` : null,
        ]
          .filter(Boolean)
          .join(" · "),
        waypointModes,
      });
    });
    return meta;
  };

  const getCurrentWaypointId = (agent) => currentWaypointsByAgent.get(agent) ?? null;

  const normalizeWaypointSelection = (entry) => {
    if (!entry || typeof entry !== "object") {
      return null;
    }
    const waypointId = normalizeCurrentWaypointId(
      entry.waypointId ?? entry.waypointID ?? entry.WaypointID ?? entry.idx,
    );
    if (!Number.isFinite(waypointId)) {
      return null;
    }
    const pathId = Number(entry.pathId ?? entry.pathID ?? entry.PathID);
    const agent = String(entry.agent || "").toUpperCase();
    return {
      agent: agent || null,
      pathId: Number.isFinite(pathId) ? pathId : null,
      waypointId,
    };
  };

  const waypointSelectionMatches = (selection, candidate) => {
    if (!selection || !candidate) {
      return false;
    }
    if (selection.agent && candidate.agent && selection.agent !== candidate.agent) {
      return false;
    }
    return selection.waypointId === candidate.waypointId;
  };

  const isSelectedWaypointEntry = (entry) => {
    const candidate = normalizeWaypointSelection(entry);
    return waypointSelectionMatches(selectedWaypoint, candidate);
  };

  const findWaypointEntry = (props) => {
    const key = props?.key;
    if (key) {
      const byKey = waypointLabelEntries.find((entry) => entry.key === key);
      if (byKey) {
        return byKey;
      }
    }
    const selection = normalizeWaypointSelection(props);
    if (!selection) {
      return null;
    }
    return waypointLabelEntries.find((entry) =>
      waypointSelectionMatches(selection, normalizeWaypointSelection(entry)),
    ) || null;
  };

  const summarizeWaypointSweep = (entry) => {
    const selection = normalizeWaypointSelection(entry);
    if (!selection) {
      return null;
    }
    const matched = sweepFeatures.filter((feature) =>
      waypointSelectionMatches(selection, normalizeWaypointSelection(feature?.properties || {})),
    );
    if (!matched.length) {
      return null;
    }
    const lineFeatures = matched.filter((feature) => feature?.geometry?.type === "LineString");
    const pointFeatures = matched.filter((feature) => feature?.geometry?.type === "Point");
    const sample = (lineFeatures[0] || pointFeatures[0] || matched[0])?.properties || {};
    return {
      lineCount: lineFeatures.length,
      pointCount: pointFeatures.length || Number(sample.pointCount) || null,
      inputMissionId: sample.inputMissionId ?? null,
      sweepAvgSpacingM: sample.sweepAvgSpacingM ?? null,
      sweepLineCount: sample.sweepLineCount ?? null,
      sweepPairCount: sample.sweepPairCount ?? null,
      altMin: Number(sample.altMin),
      altMax: Number(sample.altMax),
      fovLabel: sample.fovLabel || entry.fovLabel || null,
    };
  };

  const buildWaypointPopupHtml = (entry) => {
    const mode = entry.coveragePassLabel || entry.passLabel || "N/A";
    const currentBadge = entry.isCurrent ? "Current" : "Waypoint";
    const sweep = summarizeWaypointSweep(entry);
    const details = [
      entry.pathId !== null && entry.pathId !== undefined ? `Path ${entry.pathId}` : null,
      entry.waypointId !== null && entry.waypointId !== undefined ? `WP ${entry.waypointId}` : null,
      `Mode ${mode}`,
      entry.fovLabel,
      Number.isFinite(entry.speedMps)
        ? `Speed ${entry.speedMps.toFixed(1)} m/s`
        : "Speed -",
      Number.isFinite(entry.hoverSeconds) ? `Hover ${Math.round(entry.hoverSeconds)}s` : null,
      Number.isFinite(entry.etaSeconds) ? `ETA ${Math.round(entry.etaSeconds)}s` : null,
      entry.loiterSummary ? `Loiter ${entry.loiterSummary}` : null,
      sweep ? `Sweep ${sweep.lineCount} lines · ${sweep.pointCount ?? "-"} points` : "Sweep -",
      sweep?.inputMissionId !== null && sweep?.inputMissionId !== undefined
        ? `Input mission ${sweep.inputMissionId}`
        : null,
      sweep ? formatSweepSpacing(sweep) : null,
      sweep ? formatAltRange(sweep.altMin, sweep.altMax) : null,
      entry.passSummary ? `Path summary ${entry.passSummary}` : null,
      entry.isDone ? "Status Done" : "Status Active",
    ]
      .filter(Boolean)
      .map((line) => `<div style="font-size:11px;color:#333;">${line}</div>`)
      .join("");
    return `
      <div style="font-size:12px;font-weight:700;margin-bottom:4px;">${currentBadge} · ${entry.agent || "WP"}</div>
      ${details}
    `;
  };

  const openWaypointPopup = (entry) => {
    if (!entry) {
      return;
    }
    const selected = selectWaypoint(entry);
    if (!selected) {
      if (popup) {
        popup.remove();
      }
      return;
    }
    if (!popup) {
      popup = new maplibregl.Popup({ closeButton: true, closeOnClick: true });
    }
    popup
      .setLngLat(entry.lngLat || [entry.coord.x, entry.coord.y])
      .setHTML(buildWaypointPopupHtml(entry))
      .addTo(map);
  };

  const applyWaypointLabelStyle = (el, entry, color) => {
    const isCurrent = Boolean(entry.isCurrent);
    const isDone = Boolean(entry.isDone);
    const isSelected = isSelectedWaypointEntry(entry);
    const bg = isSelected
      ? "rgba(255, 217, 102, 0.96)"
      : isCurrent
      ? "rgba(255, 255, 255, 0.96)"
      : "rgba(0, 0, 0, 0.45)";
    const border = isSelected
      ? "1px solid rgba(37, 23, 0, 0.85)"
      : isCurrent ? `1px solid ${color || "#e7eddc"}` : "1px solid rgba(255,255,255,0.08)";
    const shadow = isSelected
      ? `0 0 0 2px rgba(255,255,255,0.42), 0 12px 22px rgba(0,0,0,0.3), 0 0 28px ${color || "#ffffff"}88`
      : isCurrent
      ? `0 0 0 2px rgba(255,255,255,0.35), 0 10px 18px rgba(0,0,0,0.24), 0 0 24px ${color || "#ffffff"}55`
      : "0 4px 10px rgba(0,0,0,0.18)";
    // A manned waypoint's role decides how loud it is: transit fades back so
    // the points that matter operationally - attack, concealment, hold - stay
    // readable at a glance.
    const roleStyle = entry.lahRole ? LAH_ROLE_STYLES[entry.lahRole] : null;
    const isQuietRole = Boolean(roleStyle) && entry.lahRole === "transit";
    const isLoudRole = Boolean(roleStyle) && !isQuietRole;
    const primaryColor = isSelected || isCurrent
      ? "#0d1117"
      : (isLoudRole && roleStyle.color) || color || "#e7eddc";
    const secondaryColor = isSelected ? "#4d3600" : isCurrent ? "#345" : "rgba(232, 240, 223, 0.72)";
    const summary = [
      entry.lahRoleSummary || null,
      entry.coveragePassLabel || entry.passLabel || entry.loiterSummary || null,
      entry.fovLabel,
    ]
      .filter(Boolean)
      .join(" · ") || (entry.lahRole ? LAH_ROLE_STYLES[entry.lahRole].label : "Pass N/A");
    // A quiet label keeps only its ID. The "TRANSIT" line said nothing that the
    // absence of a role marker did not already say, and stacked transit labels
    // were burying the attack/conceal ones they sat next to.
    const showSummary = !isQuietRole || isSelected || isCurrent;
    const titleSize = isCurrent ? 12 : isLoudRole ? 12 : isQuietRole ? 10 : 11;
    const styleSignature = [
      entry.label || `WP${entry.idx}`,
      summary,
      entry.lahRole || "",
      color || "",
      isCurrent ? 1 : 0,
      isDone ? 1 : 0,
      isSelected ? 1 : 0,
      showSummary ? 1 : 0,
      titleSize,
    ].join("\u001f");
    if (el._missionLabelStyleSignature === styleSignature) {
      return;
    }
    el.innerHTML = `
      <div style="font-size:${titleSize}px;line-height:1.05;letter-spacing:0.04em;${
        roleStyle ? `font-weight:${roleStyle.weight};` : ""
      }">${entry.label || `WP${entry.idx}`}</div>
      ${showSummary ? `<div style="margin-top:2px;font-size:9px;line-height:1.05;letter-spacing:0.08em;color:${secondaryColor};">${summary}</div>` : ""}
      ${isCurrent ? "<div style=\"margin-top:2px;font-size:8px;line-height:1;letter-spacing:0.14em;color:#0a6cff;\">CURRENT</div>" : ""}
    `;
    const isQuietChrome = isQuietRole && !isSelected && !isCurrent;
    el.classList.toggle("wp-label--quiet", isQuietChrome);
    el.style.color = primaryColor;
    el.style.background = isLoudRole && !isSelected && !isCurrent
      ? "rgba(0, 0, 0, 0.72)"
      // The pill itself is most of a transit label's visual weight.
      : isQuietChrome ? "transparent" : bg;
    el.style.border = isLoudRole && !isSelected && !isCurrent
      ? `1px solid ${roleStyle.color}`
      : isQuietChrome ? "1px solid transparent" : border;
    el.style.boxShadow = isLoudRole && !isSelected && !isCurrent
      ? `0 0 0 1px ${roleStyle.color}55, 0 6px 16px rgba(0,0,0,0.32), 0 0 18px ${roleStyle.color}44`
      : isQuietChrome ? "none" : shadow;
    el.style.opacity = isSelected || isCurrent
      ? "1"
      : roleStyle
        ? String(roleStyle.opacity)
        : "1";
    const roleScale = isLoudRole ? roleStyle.emphasis : isQuietRole ? roleStyle.emphasis : 1;
    el.style.transform = isSelected
      ? "translate(-50%, -120%) scale(1.12)"
      : isCurrent
        ? "translate(-50%, -120%) scale(1.08)"
        : `translate(-50%, -120%) scale(${roleScale})`;
    el.style.zIndex = isSelected
      ? "5"
      : isCurrent
        ? "4"
        : isLoudRole
          ? "3"
          : isQuietRole
            ? "0"
            : isDone
              ? "2"
              : "1";
    el.style.pointerEvents = "auto";
    el.style.cursor = "pointer";
    el.title = `${entry.agent || "WP"} ${entry.label || `WP${entry.idx}`} | ${summary}`;
    el._missionLabelStyleSignature = styleSignature;
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

  const ensureWaypointLabel = (entry, color) => {
    ensureLabelContainer();
    const key = entry.key;
    const agent = entry.agent;
    const idx = entry.idx;
    let el = labelElements.get(key);
    if (!el) {
      el = document.createElement("div");
      el.dataset.agent = agent;
      el.dataset.wp = String(idx);
      el.style.position = "absolute";
      el.style.transform = "translate(-50%, -120%)";
      el.style.fontSize = `${WAYPOINT_LABEL_FONT_SIZE}px`;
      el.style.fontWeight = "700";
      el.style.padding = "2px 6px";
      el.style.borderRadius = "8px";
      el.style.background = "rgba(0, 0, 0, 0.45)";
      el.style.whiteSpace = "nowrap";
      el.style.pointerEvents = "auto";
      el.style.textShadow = "0 1px 2px rgba(0,0,0,0.45)";
      el.style.display = "none";
      el._missionLabelVisible = false;
      labelContainer.appendChild(el);
      labelElements.set(key, el);
      el.addEventListener("click", (event) => {
        event.stopPropagation();
        const popupEntry = waypointLabelEntries.find((item) => item.key === key);
        if (!popupEntry) {
          return;
        }
        openWaypointPopup(popupEntry);
      });
    }
    const doneValue = entry.isDone ? "1" : "0";
    const currentValue = entry.isCurrent ? "1" : "0";
    if (el.dataset.done !== doneValue) {
      el.dataset.done = doneValue;
    }
    if (el.dataset.current !== currentValue) {
      el.dataset.current = currentValue;
    }
    applyWaypointLabelStyle(el, entry, color || "#e7eddc");
    return el;
  };

  const clearWaypointLabels = () => {
    labelElements.forEach((el) => el.remove());
    labelElements.clear();
    waypointLabelEntries = [];
    visibleWaypointLabelKeys.clear();
    updateWaypointHitSource();
  };

  const updateWaypointHitSource = () => {
    const source = map.getSource(WAYPOINT_HIT_SOURCE_ID);
    if (!source) {
      return;
    }
    source.setData({
      type: "FeatureCollection",
      features: waypointLabelEntries
        .filter((entry) => Array.isArray(entry.lngLat))
        .map((entry) => ({
          type: "Feature",
          id: entry.key,
          geometry: {
            type: "Point",
            coordinates: entry.lngLat,
          },
          properties: {
            key: entry.key,
            agent: entry.agent,
            pathId: entry.pathId,
            waypointId: entry.waypointId,
            isDone: entry.isDone ? 1 : 0,
            isCurrent: entry.isCurrent ? 1 : 0,
          },
        })),
    });
  };

  const applyWaypointVisibility = () => {
    const applyCustomLayer = (layer) => {
      if (!layer || typeof layer.setVisible !== "function") {
        return;
      }
      const hasPoints =
        Number(layer._pointCount) > 0 ||
        (Array.isArray(layer._pendingPositions) && layer._pendingPositions.length > 0);
      layer.setVisible(waypointsVisible && hasPoints);
    };
    waypointLayers.forEach(applyCustomLayer);
    doneWaypointLayers.forEach(applyCustomLayer);
    currentWaypointLayers.forEach(applyCustomLayer);
    if (labelContainer) {
      const display = waypointsVisible ? "block" : "none";
      if (labelContainer.style.display !== display) {
        labelContainer.style.display = display;
      }
    }
    if (map.getLayer(WAYPOINT_HIT_LAYER_ID)) {
      map.setLayoutProperty(
        WAYPOINT_HIT_LAYER_ID,
        "visibility",
        waypointsVisible ? "visible" : "none",
      );
    }
    map.triggerRepaint();
  };

  const refreshWaypointLabelStyles = () => {
    const colorsNow = getAgentColors();
    waypointLabelEntries.forEach((entry) => {
      const el = labelElements.get(entry.key);
      if (el) {
        applyWaypointLabelStyle(el, entry, colorsNow[entry.agent] || "#e7eddc");
      }
    });
  };

  const rebuildWaypointLabels = (entries) => {
    const colorsNow = getAgentColors();
    const keep = new Set();
    entries.forEach((entry) => {
      keep.add(entry.key);
      ensureWaypointLabel(entry, colorsNow[entry.agent] || "#e7eddc");
    });
    labelElements.forEach((el, key) => {
      if (!keep.has(key)) {
        el.remove();
        labelElements.delete(key);
        visibleWaypointLabelKeys.delete(key);
      }
    });
    waypointLabelEntries = entries;
    updateWaypointHitSource();
    updateWaypointLabelVisibility();
  };

  const updateWaypointLabelVisibility = () => {
    if (labelContainer) {
      const display = waypointsVisible ? "block" : "none";
      if (labelContainer.style.display !== display) {
        labelContainer.style.display = display;
      }
    }
    if (!waypointsVisible) {
      return;
    }
    const hasSelection = Boolean(selectedAgent);
    labelElements.forEach((el) => {
      const agent = el.dataset.agent || "";
      const isDone = el.dataset.done === "1";
      const isCurrent = el.dataset.current === "1";
      const activeAlpha = hasSelection
        ? agent === selectedAgent
          ? WAYPOINT_ALPHA
          : WAYPOINT_DIM_ALPHA
        : WAYPOINT_ALPHA;
      const doneAlpha = hasSelection
        ? agent === selectedAgent
          ? WAYPOINT_DONE_SELECT_ALPHA
          : WAYPOINT_DONE_DIM_ALPHA
        : WAYPOINT_DONE_ALPHA;
      const currentAlpha = hasSelection
        ? agent === selectedAgent
          ? WAYPOINT_CURRENT_ALPHA
          : WAYPOINT_CURRENT_DIM_ALPHA
        : WAYPOINT_CURRENT_ALPHA;
      const currentDoneAlpha = hasSelection
        ? agent === selectedAgent
          ? WAYPOINT_CURRENT_DONE_ALPHA
          : WAYPOINT_CURRENT_DONE_DIM_ALPHA
        : WAYPOINT_CURRENT_DONE_ALPHA;
      const alpha = isCurrent ? (isDone ? currentDoneAlpha : currentAlpha) : isDone ? doneAlpha : activeAlpha;
      const opacity = String(alpha);
      if (el.style.opacity !== opacity) {
        el.style.opacity = opacity;
      }
    });
  };

  const updateWaypointLabelPositions = (matrix) => {
    if (!waypointsVisible || !labelContainer || !matrix) {
      return;
    }
    const canvas = map.getCanvas();
    // Map projection and DOM labels both use CSS pixels even when the map's
    // internal render pixel ratio is capped for performance.
    const width = canvas.clientWidth || canvas.getBoundingClientRect().width || canvas.width;
    const height = canvas.clientHeight || canvas.getBoundingClientRect().height || canvas.height;
    const nextVisibleKeys = new Set();
    waypointLabelEntries.forEach((entry) => {
      const point = projectToScreen(
        matrix,
        entry.coord.x,
        entry.coord.y,
        entry.coord.z,
        width,
        height,
      );
      if (!point) {
        return;
      }
      const el = labelElements.get(entry.key);
      if (!el) {
        return;
      }
      nextVisibleKeys.add(entry.key);
      if (el._missionLabelX !== point.x) {
        el.style.left = `${point.x}px`;
        el._missionLabelX = point.x;
      }
      if (el._missionLabelY !== point.y) {
        el.style.top = `${point.y}px`;
        el._missionLabelY = point.y;
      }
      if (el._missionLabelVisible !== true) {
        el.style.display = "block";
        el._missionLabelVisible = true;
      }
    });
    visibleWaypointLabelKeys.forEach((key) => {
      if (nextVisibleKeys.has(key)) {
        return;
      }
      const el = labelElements.get(key);
      if (el && el._missionLabelVisible !== false) {
        el.style.display = "none";
        el._missionLabelVisible = false;
      }
    });
    visibleWaypointLabelKeys = nextVisibleKeys;
  };

  const buildLinePositions = (
    coords,
    alts,
    getTerrainElevation,
    { solidRibbon = false } = {},
  ) => {
    if (!Array.isArray(coords) || coords.length < 2) {
      return [];
    }
    const positions = [];
    const zoom = typeof map.getZoom === "function" ? map.getZoom() : 10;
    for (let i = 1; i < coords.length; i += 1) {
      const prev = coords[i - 1];
      const next = coords[i];
      if (!Array.isArray(prev) || !Array.isArray(next)) {
        continue;
      }
      const lonA = Number(prev[0]);
      const latA = Number(prev[1]);
      const lonB = Number(next[0]);
      const latB = Number(next[1]);
      if (
        !Number.isFinite(lonA) ||
        !Number.isFinite(latA) ||
        !Number.isFinite(lonB) ||
        !Number.isFinite(latB)
      ) {
        continue;
      }
      const altA = Array.isArray(alts) ? Number(alts[i - 1]) : Number(alts);
      const altB = Array.isArray(alts) ? Number(alts[i]) : Number(alts);
      const terrainA = Number.isFinite(altA) || typeof getTerrainElevation !== "function"
        ? 0
        : getTerrainElevation(lonA, latA);
      const terrainB = Number.isFinite(altB) || typeof getTerrainElevation !== "function"
        ? 0
        : getTerrainElevation(lonB, latB);
      const altStart = (Number.isFinite(altA) ? altA : terrainA) + ALT_OFFSET_M;
      const altEnd = (Number.isFinite(altB) ? altB : terrainB) + ALT_OFFSET_M;
      const lengthM = distanceMeters(lonA, latA, lonB, latB);
      if (!Number.isFinite(lengthM) || lengthM <= 0) {
        continue;
      }
      const midLat = (latA + latB) / 2;
      const mpp = metersPerPixelAt(midLat, zoom);
      const dashOn = Math.max(1, DASH_ON_PX * mpp);
      const dashOff = Math.max(1, DASH_OFF_PX * mpp);
      const dashCycle = dashOn + dashOff;
      const addSegment = (t0, t1) => {
        const lonS = lonA + (lonB - lonA) * t0;
        const latS = latA + (latB - latA) * t0;
        const lonE = lonA + (lonB - lonA) * t1;
        const latE = latA + (latB - latA) * t1;
        const altS = altStart + (altEnd - altStart) * t0;
        const altE = altStart + (altEnd - altStart) * t1;
        const start = maplibregl.MercatorCoordinate.fromLngLat([lonS, latS], altS);
        const end = maplibregl.MercatorCoordinate.fromLngLat([lonE, latE], altE);
        const dx = end.x - start.x;
        const dy = end.y - start.y;
        const dz = end.z - start.z;
        const segLen = Math.hypot(dx, dy, dz);
        if (!Number.isFinite(segLen) || segLen === 0) {
          return;
        }
        const cross = (a, b) => [
          a[1] * b[2] - a[2] * b[1],
          a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0],
        ];
        const normalize = (v) => {
          const len = Math.hypot(v[0], v[1], v[2]);
          if (!Number.isFinite(len) || len === 0) {
            return null;
          }
          return [v[0] / len, v[1] / len, v[2] / len];
        };
        const dir = [dx / segLen, dy / segLen, dz / segLen];
        let side1 = normalize(cross(dir, [0, 0, 1]));
        if (!side1) {
          side1 = normalize(cross(dir, [1, 0, 0]));
        }
        if (!side1) {
          return;
        }
        const side2 = normalize(cross(dir, side1));
        const m2u = start.meterInMercatorCoordinateUnits();
        const halfWidth = (PATH_WIDTH_PX * mpp * m2u) / 2;
        const pushQuad = (offset) => {
          const ox = offset[0] * halfWidth;
          const oy = offset[1] * halfWidth;
          const oz = offset[2] * halfWidth;
          const sx1 = start.x + ox;
          const sy1 = start.y + oy;
          const sz1 = start.z + oz;
          const sx2 = start.x - ox;
          const sy2 = start.y - oy;
          const sz2 = start.z - oz;
          const ex1 = end.x + ox;
          const ey1 = end.y + oy;
          const ez1 = end.z + oz;
          const ex2 = end.x - ox;
          const ey2 = end.y - oy;
          const ez2 = end.z - oz;
          positions.push(
            sx1,
            sy1,
            sz1,
            sx2,
            sy2,
            sz2,
            ex1,
            ey1,
            ez1,
            ex1,
            ey1,
            ez1,
            sx2,
            sy2,
            sz2,
            ex2,
            ey2,
            ez2,
          );
        };
        // Flight routes used to draw two perpendicular ribbons so the path stayed
        // thick from every camera angle.  The crossed planes are plainly
        // visible around steep altitude changes, however, and make an ordinary
        // waypoint leg look like a row of X markers.  A single ribbon is enough
        // for both LAH and UAV routes and reads as one continuous connecting line.
        pushQuad(side1);
        if (!solidRibbon && side2) {
          pushQuad(side2);
        }
      };
      if (solidRibbon) {
        addSegment(0, 1);
        continue;
      }
      if (lengthM <= dashOn) {
        addSegment(0, 1);
        continue;
      }
      let traveled = 0;
      while (traveled < lengthM) {
        const segStart = traveled;
        const segEnd = Math.min(traveled + dashOn, lengthM);
        const t0 = segStart / lengthM;
        const t1 = segEnd / lengthM;
        if (t1 > t0) {
          addSegment(t0, t1);
        }
        traveled += dashCycle;
      }
    }
    return positions;
  };

  const ensureHitLayer = () => {
    if (map.getSource(SOURCE_ID)) {
      return;
    }
    map.addSource(SOURCE_ID, {
      type: "geojson",
      data: { type: "FeatureCollection", features: [] },
    });
    map.addLayer({
      id: HIT_LAYER_ID,
      type: "line",
      source: SOURCE_ID,
      layout: { "line-join": "round", "line-cap": "round" },
      paint: {
        "line-color": "#000000",
        "line-width": 10,
        "line-opacity": 0.01,
      },
    });
  };

  const ensureWaypointHitLayer = () => {
    if (!map.getSource(WAYPOINT_HIT_SOURCE_ID)) {
      map.addSource(WAYPOINT_HIT_SOURCE_ID, {
        type: "geojson",
        data: { type: "FeatureCollection", features: [] },
      });
    }
    if (!map.getLayer(WAYPOINT_HIT_LAYER_ID)) {
      map.addLayer({
        id: WAYPOINT_HIT_LAYER_ID,
        type: "circle",
        source: WAYPOINT_HIT_SOURCE_ID,
        paint: {
          "circle-radius": ["interpolate", ["linear"], ["zoom"], 8, 10, 12, 14, 16, 18],
          "circle-color": "#ffffff",
          "circle-opacity": 0.01,
          "circle-stroke-opacity": 0,
        },
      });
    }
    updateWaypointHitSource();
  };

  const ensure3dLayers = () => {
    AGENTS.forEach((agent) => {
      const doneLayerId = `mission-paths-3d-${agent.toLowerCase()}-done`;
      if (!map.getLayer(doneLayerId)) {
        const doneLayer = createLineLayer3d(
          doneLayerId,
          colors[agent] || "#e7eddc",
          "TRIANGLES",
          PATH_WIDTH_PX,
        );
        map.addLayer(doneLayer);
        doneAgentLayers.set(agent, doneLayer);
      }
      const activeLayerId = `mission-paths-3d-${agent.toLowerCase()}`;
      if (!map.getLayer(activeLayerId)) {
        const activeLayer = createLineLayer3d(
          activeLayerId,
          colors[agent] || "#e7eddc",
          "TRIANGLES",
          PATH_WIDTH_PX,
        );
        map.addLayer(activeLayer);
        agentLayers.set(agent, activeLayer);
      }
    });
    const waypointSize = WAYPOINT_SIZE_PX * (
      typeof map.getPixelRatio === "function" ? map.getPixelRatio() : 1
    );
    AGENTS.forEach((agent) => {
      const doneLayerId = `mission-waypoints-3d-${agent.toLowerCase()}-done`;
      if (!map.getLayer(doneLayerId)) {
        const doneLayer = createPointLayer3d(doneLayerId, colors[agent] || "#e7eddc", waypointSize);
        map.addLayer(doneLayer);
        doneWaypointLayers.set(agent, doneLayer);
      }
      const activeLayerId = `mission-waypoints-3d-${agent.toLowerCase()}`;
      if (!map.getLayer(activeLayerId)) {
        const activeLayer = createPointLayer3d(activeLayerId, colors[agent] || "#e7eddc", waypointSize);
        map.addLayer(activeLayer);
        waypointLayers.set(agent, activeLayer);
        if (!labelHookLayer) {
          activeLayer._renderHook = updateWaypointLabelPositions;
          labelHookLayer = activeLayer;
        }
      }
      const currentLayerId = `mission-waypoints-3d-${agent.toLowerCase()}-current`;
      if (!map.getLayer(currentLayerId)) {
        const currentLayer = createPointLayer3d(
          currentLayerId,
          colors[agent] || "#ffffff",
          waypointSize * 1.55,
        );
        map.addLayer(currentLayer);
        currentWaypointLayers.set(agent, currentLayer);
      }
    });
  };

  const ensureAreaLayers = () => {
    if (!map.getSource(AREA_SOURCE_ID)) {
      map.addSource(AREA_SOURCE_ID, {
        type: "geojson",
        data: { type: "FeatureCollection", features: [] },
      });
    }
    if (!map.getLayer(AREA_FILL_LAYER_ID)) {
      map.addLayer({
        id: AREA_FILL_LAYER_ID,
        type: "fill",
        source: AREA_SOURCE_ID,
        paint: {
          "fill-color": ["get", "color"],
          "fill-opacity": ["get", "fillOpacity"],
        },
      }, HIT_LAYER_ID);
    }
    if (!map.getLayer(AREA_LINE_LAYER_ID)) {
      map.addLayer({
        id: AREA_LINE_LAYER_ID,
        type: "line",
        source: AREA_SOURCE_ID,
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
          "line-width": 1.6,
          "line-opacity": ["get", "lineOpacity"],
        },
      }, HIT_LAYER_ID);
    }
    if (!map.getLayer(AREA_FORWARD_LINE_LAYER_ID)) {
      map.addLayer({
        id: AREA_FORWARD_LINE_LAYER_ID,
        type: "line",
        source: AREA_SOURCE_ID,
        filter: ["==", ["get", "coveragePass"], "forward"],
        layout: { "line-join": "round", "line-cap": "round" },
        paint: {
          "line-color": ["get", "color"],
          "line-width": 2.6,
          "line-opacity": ["get", "lineOpacity"],
        },
      }, HIT_LAYER_ID);
    }
    if (!map.getLayer(AREA_REVERSE_LINE_LAYER_ID)) {
      map.addLayer({
        id: AREA_REVERSE_LINE_LAYER_ID,
        type: "line",
        source: AREA_SOURCE_ID,
        filter: ["==", ["get", "coveragePass"], "reverse"],
        layout: { "line-join": "round", "line-cap": "round" },
        paint: {
          "line-color": ["get", "color"],
          "line-width": 2.1,
          "line-dasharray": [1.5, 1.25],
          "line-opacity": ["get", "lineOpacity"],
        },
      }, HIT_LAYER_ID);
    }
  };

  const ensureSweepLayers = () => {
    if (!map.getSource(SWEEP_SOURCE_ID)) {
      map.addSource(SWEEP_SOURCE_ID, {
        type: "geojson",
        data: { type: "FeatureCollection", features: [] },
      });
    }
    if (!map.getLayer(SWEEP_LINE_LAYER_ID)) {
      map.addLayer({
        id: SWEEP_LINE_LAYER_ID,
        type: "line",
        source: SWEEP_SOURCE_ID,
        filter: [
          "all",
          ["==", ["geometry-type"], "LineString"],
          ["!=", ["get", "coveragePass"], "forward"],
          ["!=", ["get", "coveragePass"], "reverse"],
        ],
        layout: {
          "line-join": "round",
          "line-cap": "round",
        },
        paint: {
          "line-color": ["get", "color"],
          "line-width": [
            "interpolate",
            ["linear"],
            ["zoom"],
            8,
            ["case", ["==", ["get", "isSelectedWaypoint"], 1], 3.2, 1.0],
            12,
            ["case", ["==", ["get", "isSelectedWaypoint"], 1], 4.2, 1.5],
            16,
            ["case", ["==", ["get", "isSelectedWaypoint"], 1], 5.0, 2.2],
          ],
          "line-dasharray": [2.2, 1.1],
          "line-opacity": ["get", "lineOpacity"],
        },
      }, HIT_LAYER_ID);
    }
    if (!map.getLayer(SWEEP_FORWARD_LINE_LAYER_ID)) {
      map.addLayer({
        id: SWEEP_FORWARD_LINE_LAYER_ID,
        type: "line",
        source: SWEEP_SOURCE_ID,
        filter: [
          "all",
          ["==", ["geometry-type"], "LineString"],
          ["==", ["get", "coveragePass"], "forward"],
        ],
        layout: { "line-join": "round", "line-cap": "round" },
        paint: {
          "line-color": ["get", "color"],
          "line-width": [
            "interpolate",
            ["linear"],
            ["zoom"],
            8,
            ["case", ["==", ["get", "isSelectedWaypoint"], 1], 3.8, 2.0],
            12,
            ["case", ["==", ["get", "isSelectedWaypoint"], 1], 4.8, 2.8],
            16,
            ["case", ["==", ["get", "isSelectedWaypoint"], 1], 5.6, 3.6],
          ],
          "line-opacity": ["get", "lineOpacity"],
        },
      }, HIT_LAYER_ID);
    }
    if (!map.getLayer(SWEEP_REVERSE_LINE_LAYER_ID)) {
      map.addLayer({
        id: SWEEP_REVERSE_LINE_LAYER_ID,
        type: "line",
        source: SWEEP_SOURCE_ID,
        filter: [
          "all",
          ["==", ["geometry-type"], "LineString"],
          ["==", ["get", "coveragePass"], "reverse"],
        ],
        layout: { "line-join": "round", "line-cap": "round" },
        paint: {
          "line-color": ["get", "color"],
          "line-width": [
            "interpolate",
            ["linear"],
            ["zoom"],
            8,
            ["case", ["==", ["get", "isSelectedWaypoint"], 1], 3.1, 1.25],
            12,
            ["case", ["==", ["get", "isSelectedWaypoint"], 1], 4.0, 1.8],
            16,
            ["case", ["==", ["get", "isSelectedWaypoint"], 1], 4.8, 2.4],
          ],
          "line-dasharray": [1.25, 1.35],
          "line-opacity": ["get", "lineOpacity"],
        },
      }, HIT_LAYER_ID);
    }
    if (!map.getLayer(SWEEP_POINT_LAYER_ID)) {
      map.addLayer({
        id: SWEEP_POINT_LAYER_ID,
        type: "circle",
        source: SWEEP_SOURCE_ID,
        filter: ["==", ["geometry-type"], "Point"],
        paint: {
          "circle-color": ["get", "color"],
          "circle-radius": ["get", "pointRadius"],
          "circle-opacity": ["get", "pointOpacity"],
          "circle-stroke-color": "#08110c",
          "circle-stroke-width": ["get", "pointStrokeWidth"],
          "circle-stroke-opacity": ["get", "pointStrokeOpacity"],
        },
      }, HIT_LAYER_ID);
    }
    applySweepLineVisibility();
  };

  const updateHitSource = () => {
    const source = map.getSource(SOURCE_ID);
    if (!source) {
      return;
    }
    const geojson = {
      type: "FeatureCollection",
      features: features.map((feature) => buildGeoFeature(feature, colors, pathMetaByKey)),
    };
    source.setData(geojson);
  };

  const updateAreaSource = (payloadForAreas = pendingData) => {
    const source = map.getSource(AREA_SOURCE_ID);
    if (!source) {
      return;
    }
    areaFeatures = buildAreaFeatures(
      payloadForAreas,
      colors,
      selectedAgent,
      currentWaypointsByAgent,
      currentBoundaryGuardStateByAgent,
    );
    source.setData({
      type: "FeatureCollection",
      features: areaFeatures,
    });
  };

  const updateSweepSource = (payloadForAreas = pendingData) => {
    const source = map.getSource(SWEEP_SOURCE_ID);
    if (!source) {
      return;
    }
    const nextSweepFeatures = buildSweepFeatures(
      payloadForAreas,
      colors,
      selectedAgent,
      selectedWaypoint,
      currentWaypointsByAgent,
      currentBoundaryGuardStateByAgent,
    );
    const hasSelectedSweep = selectedWaypoint
      ? nextSweepFeatures.some((feature) => Number(feature?.properties?.isSelectedWaypoint) === 1)
      : false;
    sweepFeatures = selectedWaypoint && !hasSelectedSweep
      ? buildSweepFeatures(
          payloadForAreas,
          colors,
          selectedAgent,
          null,
          currentWaypointsByAgent,
          currentBoundaryGuardStateByAgent,
        )
      : nextSweepFeatures;
    source.setData({
      type: "FeatureCollection",
      features: sweepFeatures,
    });
  };

  const update3dPositions = () => {
    const zoomBucket = Math.round((typeof map.getZoom === "function" ? map.getZoom() : 0) * 10) / 10;
    if (!features.length) {
      agentLayers.forEach((layer) => layer.updatePositions([]));
      doneAgentLayers.forEach((layer) => layer.updatePositions([]));
      waypointLayers.forEach((layer) => layer.updatePositions([]));
      doneWaypointLayers.forEach((layer) => layer.updatePositions([]));
      currentWaypointLayers.forEach((layer) => layer.updatePositions([]));
      clearWaypointLabels();
      renderedVisualRevision = visualRebuildRevision;
      renderedZoomBucket = zoomBucket;
      logStatus("", { key: "mission-debug" });
      return;
    }
    const positionsByAgent = new Map();
    const donePositionsByAgent = new Map();
    const waypointPositionsByAgent = new Map();
    const doneWaypointPositionsByAgent = new Map();
    const currentWaypointPositionsByAgent = new Map();
    const labelEntries = [];
    AGENTS.forEach((agent) => positionsByAgent.set(agent, []));
    AGENTS.forEach((agent) => donePositionsByAgent.set(agent, []));
    AGENTS.forEach((agent) => waypointPositionsByAgent.set(agent, []));
    AGENTS.forEach((agent) => doneWaypointPositionsByAgent.set(agent, []));
    AGENTS.forEach((agent) => currentWaypointPositionsByAgent.set(agent, []));
    // A mission update often references the same waypoint in path, point and
    // label geometry. Cache only for this rebuild so newly loaded terrain can
    // still be observed by a later rebuild.
    const terrainElevationCache = new Map();
    const getTerrainElevation = (lon, lat) => {
      const key = `${lon}:${lat}`;
      if (terrainElevationCache.has(key)) {
        return terrainElevationCache.get(key);
      }
      let elevation = 0;
      if (typeof map.queryTerrainElevation !== "function") {
        terrainElevationCache.set(key, elevation);
        return elevation;
      }
      const elev = map.queryTerrainElevation({ lng: lon, lat: lat }, { exaggerated: false });
      if (Number.isFinite(elev)) {
        elevation = elev;
      }
      terrainElevationCache.set(key, elevation);
      return elevation;
    };
    const buildWaypointPositions = (coords, alts) => {
      if (!Array.isArray(coords) || coords.length < 1) {
        return [];
      }
      const positions = [];
      coords.forEach((coord, idx) => {
        if (!Array.isArray(coord)) {
          return;
        }
        const lon = Number(coord[0]);
        const lat = Number(coord[1]);
        if (!Number.isFinite(lon) || !Number.isFinite(lat)) {
          return;
        }
        const alt = Array.isArray(alts) ? Number(alts[idx]) : Number(alts);
        const altitude = (Number.isFinite(alt) ? alt : getTerrainElevation(lon, lat))
          + ALT_OFFSET_M
          + WAYPOINT_Z_OFFSET_M;
        const merc = maplibregl.MercatorCoordinate.fromLngLat([lon, lat], altitude);
        positions.push(merc.x, merc.y, merc.z);
      });
      return positions;
    };
    features.forEach((feature) => {
      const isDone = Boolean(feature && feature.isDone);
      const agent = feature.agent;
      const currentWaypointId = getCurrentWaypointId(agent);
      const target = isDone ? donePositionsByAgent.get(agent) : positionsByAgent.get(agent);
      if (!target) {
        return;
      }
      const routeCoords = buildFiniteRouteCoordinates(feature);
      const routeAlts = buildFiniteRouteAltitudes(feature, routeCoords);
      const segmentPositions = buildLinePositions(
        routeCoords,
        routeAlts,
        getTerrainElevation,
        // LAH and UAV waypoint routes share the same clean, solid connector.
        // Sweep/camera geometry is rendered by separate layers and is unchanged.
        { solidRibbon: true },
      );
      if (segmentPositions.length) {
        target.push(...segmentPositions);
      }
      const waypointTarget = isDone
        ? doneWaypointPositionsByAgent.get(agent)
        : waypointPositionsByAgent.get(agent);
      const currentWaypointTarget = currentWaypointPositionsByAgent.get(agent);
      const pathKey = buildPathMetaKey(feature);
      const pathMeta = pathMetaByKey.get(pathKey) || null;
      if (waypointTarget || currentWaypointTarget) {
        const waypointPositions = buildWaypointPositions(feature.coords, feature.alts);
        if (waypointPositions.length && waypointTarget) {
          waypointTarget.push(...waypointPositions);
        }
      }
      if (Array.isArray(feature.coords)) {
        const wpIds = Array.isArray(feature.wpIds) ? feature.wpIds : [];
        feature.coords.forEach((coord, idx) => {
          if (!Array.isArray(coord)) {
            return;
          }
          const lon = Number(coord[0]);
          const lat = Number(coord[1]);
          if (!Number.isFinite(lon) || !Number.isFinite(lat)) {
            return;
          }
          const alt = Array.isArray(feature.alts) ? Number(feature.alts[idx]) : Number(feature.alts);
          const altitude = (Number.isFinite(alt) ? alt : getTerrainElevation(lon, lat))
            + ALT_OFFSET_M
            + WAYPOINT_LABEL_Z_OFFSET_M;
          const merc = maplibregl.MercatorCoordinate.fromLngLat([lon, lat], altitude);
          const wpId = Number(wpIds[idx]);
          const labelId = Number.isFinite(wpId) ? wpId : idx + 1;
          const passMeta = pathMeta?.waypointModes?.get(labelId) || null;
          const isCurrent = Number.isFinite(currentWaypointId) && labelId === currentWaypointId;
          if (isCurrent && currentWaypointTarget) {
            currentWaypointTarget.push(merc.x, merc.y, merc.z);
          }
          labelEntries.push({
            key: `${agent}-${pathKey}-${idx + 1}`,
            agent,
            idx: labelId,
            label: `WP${labelId}`,
            isDone,
            isCurrent,
            waypointId: labelId,
            pathId: feature.pathId ?? null,
            passLabel: passMeta?.passLabel || null,
            coveragePass: passMeta?.coveragePass || null,
            coveragePassLabel: passMeta?.coveragePassLabel || null,
            loiterSummary: passMeta?.loiterSummary || null,
            fovDeg: passMeta?.fovDeg ?? null,
            fovLabel: passMeta?.fovLabel || null,
            passSummary: pathMeta?.passSummary || null,
            lahRole: passMeta?.lahRole || null,
            lahRoleSummary: passMeta?.lahRoleSummary || null,
            lngLat: [lon, lat],
            coord: merc,
          });
        });
      }
    });
    let totalPositions = 0;
    let totalLineCount = 0;
    positionsByAgent.forEach((positions, agent) => {
      const layer = agentLayers.get(agent);
      if (!layer) {
        return;
      }
      layer.setVisible(Boolean(positions.length));
      layer.updatePositions(positions);
      totalPositions += positions.length;
      if (Number.isFinite(layer._lineCount)) {
        totalLineCount += layer._lineCount;
      }
    });
    donePositionsByAgent.forEach((positions, agent) => {
      const layer = doneAgentLayers.get(agent);
      if (!layer) {
        return;
      }
      layer.setVisible(Boolean(positions.length));
      layer.updatePositions(positions);
      totalPositions += positions.length;
      if (Number.isFinite(layer._lineCount)) {
        totalLineCount += layer._lineCount;
      }
    });
    const waypointSize = WAYPOINT_SIZE_PX * (
      typeof map.getPixelRatio === "function" ? map.getPixelRatio() : 1
    );
    waypointPositionsByAgent.forEach((positions, agent) => {
      const layer = waypointLayers.get(agent);
      if (!layer) {
        return;
      }
      if (typeof layer.setColor === "function") {
        layer.setColor(colors[agent] || "#e7eddc");
      }
      layer.setVisible(waypointsVisible && Boolean(positions.length));
      layer.setSize(waypointSize);
      layer.updatePositions(positions);
    });
    doneWaypointPositionsByAgent.forEach((positions, agent) => {
      const layer = doneWaypointLayers.get(agent);
      if (!layer) {
        return;
      }
      if (typeof layer.setColor === "function") {
        layer.setColor(colors[agent] || "#e7eddc");
      }
      layer.setVisible(waypointsVisible && Boolean(positions.length));
      layer.setSize(waypointSize);
      layer.updatePositions(positions);
    });
    const currentWaypointSize = waypointSize * 1.55;
    currentWaypointPositionsByAgent.forEach((positions, agent) => {
      const layer = currentWaypointLayers.get(agent);
      if (!layer) {
        return;
      }
      if (typeof layer.setColor === "function") {
        layer.setColor(colors[agent] || "#ffffff");
      }
      layer.setVisible(waypointsVisible && Boolean(positions.length));
      layer.setSize(currentWaypointSize);
      layer.updatePositions(positions);
    });
    rebuildWaypointLabels(labelEntries);
    renderedVisualRevision = visualRebuildRevision;
    renderedZoomBucket = zoomBucket;
    const msg =
      totalPositions === 0
        ? "3D path positions empty."
        : `3D path vertices: ${Math.floor(totalPositions / 3)} | lines: ${totalLineCount}`;
    logStatus(msg, { key: "mission-debug", ttlMs: 4500 });
    map.triggerRepaint();
  };

  const scheduleRebuild = (force = false) => {
    if (visualRebuildScheduled) {
      return;
    }
    const zoomBucket = Math.round((typeof map.getZoom === "function" ? map.getZoom() : 0) * 10) / 10;
    if (
      !force &&
      renderedVisualRevision === visualRebuildRevision &&
      renderedZoomBucket === zoomBucket
    ) {
      return;
    }
    visualRebuildScheduled = true;
    requestAnimationFrame(() => {
      visualRebuildScheduled = false;
      update3dPositions();
    });
  };

  const updateLayerAlpha = () => {
    const hasSelection = Boolean(selectedAgent);
    agentLayers.forEach((layer, agent) => {
      const alpha = hasSelection
        ? agent === selectedAgent
          ? SELECT_ALPHA
          : DIM_ALPHA
        : DEFAULT_ALPHA;
      layer.setAlpha(alpha);
    });
    doneAgentLayers.forEach((layer, agent) => {
      const alpha = hasSelection
        ? agent === selectedAgent
          ? DONE_SELECT_ALPHA
          : DONE_DIM_ALPHA
        : DONE_ALPHA;
      layer.setAlpha(alpha);
    });
    waypointLayers.forEach((layer, agent) => {
      const alpha = hasSelection
        ? agent === selectedAgent
          ? WAYPOINT_ALPHA
          : WAYPOINT_DIM_ALPHA
        : WAYPOINT_ALPHA;
      layer.setAlpha(alpha);
    });
    doneWaypointLayers.forEach((layer, agent) => {
      const alpha = hasSelection
        ? agent === selectedAgent
          ? WAYPOINT_DONE_SELECT_ALPHA
          : WAYPOINT_DONE_DIM_ALPHA
        : WAYPOINT_DONE_ALPHA;
      layer.setAlpha(alpha);
    });
    currentWaypointLayers.forEach((layer, agent) => {
      const alpha = hasSelection
        ? agent === selectedAgent
          ? 1.0
          : 0.55
        : 1.0;
      layer.setAlpha(alpha);
    });
    updateAreaSource();
    updateSweepSource();
    refreshWaypointLabelStyles();
    updateWaypointLabelVisibility();
    map.triggerRepaint();
  };

  const setSelectedAgent = (agent) => {
    selectedAgent = agent || null;
    selectedWaypoint = null;
    updateLegend();
    updateLayerAlpha();
  };

  const selectWaypoint = (entry) => {
    const next = normalizeWaypointSelection(entry);
    if (!next) {
      return false;
    }
    if (waypointSelectionMatches(selectedWaypoint, next)) {
      selectedWaypoint = null;
      selectedAgent = null;
      updateLegend();
      updateLayerAlpha();
      return false;
    }
    selectedWaypoint = next;
    selectedAgent = next.agent || selectedAgent;
    if (!sweepLinesVisible) {
      setSweepLinesVisible(true);
    }
    updateLegend();
    updateLayerAlpha();
    return true;
  };

  const setCurrentWaypoints = (agentToWpId) => {
    const next = new Map();
    if (agentToWpId instanceof Map) {
      agentToWpId.forEach((value, key) => {
        const agent = String(key || "").toUpperCase();
        const wpId = normalizeCurrentWaypointId(value);
        if (agent && Number.isFinite(wpId)) {
          next.set(agent, wpId);
        }
      });
    } else if (agentToWpId && typeof agentToWpId === "object") {
      Object.entries(agentToWpId).forEach(([key, value]) => {
        const agent = String(key || "").toUpperCase();
        const wpId = normalizeCurrentWaypointId(value);
        if (agent && Number.isFinite(wpId)) {
          next.set(agent, wpId);
        }
      });
    }
    let changed = next.size !== currentWaypointsByAgent.size;
    if (!changed) {
      next.forEach((value, key) => {
        if (currentWaypointsByAgent.get(key) !== value) {
          changed = true;
        }
      });
    }
    if (!changed) {
      return false;
    }
    currentWaypointsByAgent = next;
    visualRebuildRevision += 1;
    updateCoveragePassLegend();
    updateLayerAlpha();
    scheduleRebuild(true);
    return true;
  };

  const setSweepLinesVisible = (visible) => {
    sweepLinesVisible = Boolean(visible);
    const sweepToggle = document.getElementById("toggle-sweep-lines");
    if (sweepToggle) {
      sweepToggle.classList.toggle("is-active", sweepLinesVisible);
      sweepToggle.setAttribute("aria-pressed", sweepLinesVisible ? "true" : "false");
    }
    applySweepLineVisibility();
  };

  const setWaypointsVisible = (visible) => {
    waypointsVisible = Boolean(visible);
    const waypointToggle = document.getElementById("toggle-waypoints");
    if (waypointToggle) {
      waypointToggle.classList.toggle("is-active", waypointsVisible);
      waypointToggle.setAttribute("aria-pressed", waypointsVisible ? "true" : "false");
    }
    applyWaypointVisibility();
  };

  const boundaryGuardStateIndexChanged = (next) => {
    if (next.size !== currentBoundaryGuardStateByAgent.size) {
      return true;
    }
    for (const [agent, value] of next.entries()) {
      const previous = currentBoundaryGuardStateByAgent.get(agent);
      if (
        !previous ||
        previous.active !== value.active ||
        previous.setId !== value.setId ||
        previous.cycleCount !== value.cycleCount ||
        previous.sequence !== value.sequence ||
        previous.sequenceCount !== value.sequenceCount
      ) {
        return true;
      }
    }
    return false;
  };

  const syncCurrentWaypointsFromState = (state) => {
    const next = {};
    const nextCoverageState = new Map();
    const nextBoundaryGuardState = buildBoundaryGuardLiveStateIndex(state);
    const vehicles = state?.vehicles || {};
    Object.entries(vehicles).forEach(([agent, entry]) => {
      const normalizedAgent = String(agent || "").toUpperCase();
      if (!normalizedAgent) {
        return;
      }
      const current =
        entry?.currentWaypointID ??
        entry?.CurrentWaypointID ??
        entry?.unmannedInfo?.currentWaypointID ??
        entry?.unmannedInfo?.CurrentWaypointID ??
        null;
      const wpId = normalizeCurrentWaypointId(current);
      if (Number.isFinite(wpId)) {
        next[normalizedAgent] = wpId;
      }
      const coveragePass = normalizeAreaCoveragePass(
        entry?.areaCoveragePass ?? entry?.AreaCoveragePass,
      );
      const turnPhase = entry?.areaTurnPhase ?? entry?.areaCoverageTurnPhase ?? null;
      const turnRole = entry?.areaTurnRole ?? null;
      if (coveragePass || turnPhase || turnRole) {
        nextCoverageState.set(normalizedAgent, {
          pass: coveragePass,
          turnPhase: turnPhase ? String(turnPhase) : null,
          turnRole: turnRole ? String(turnRole) : null,
        });
      }
    });
    let coverageStateChanged = nextCoverageState.size !== currentCoverageStateByAgent.size;
    if (!coverageStateChanged) {
      nextCoverageState.forEach((value, key) => {
        const previous = currentCoverageStateByAgent.get(key);
        if (
          !previous ||
          previous.pass !== value.pass ||
          previous.turnPhase !== value.turnPhase ||
          previous.turnRole !== value.turnRole
        ) {
          coverageStateChanged = true;
        }
      });
    }
    currentCoverageStateByAgent = nextCoverageState;
    const boundaryGuardStateChanged = boundaryGuardStateIndexChanged(
      nextBoundaryGuardState,
    );
    // Install the guard frame before setCurrentWaypoints refreshes the sources;
    // both values originate from the same authoritative SIM sample.
    currentBoundaryGuardStateByAgent = nextBoundaryGuardState;
    const waypointChanged = setCurrentWaypoints(next);
    if (boundaryGuardStateChanged && !waypointChanged) {
      updateAreaSource();
      updateSweepSource();
      map.triggerRepaint();
    }
    // A waypoint change already refreshes the legend. Refresh explicitly only
    // when the live pass/turn state changed on its own.
    if (coverageStateChanged && !waypointChanged) {
      updateCoveragePassLegend();
    }
  };

  if (window.simClient && typeof window.simClient.subscribe === "function") {
    window.simClient.subscribe((state) => {
      syncCurrentWaypointsFromState(state);
    });
  }

  const attachInteractions = () => {
    if (interactionsAttached || !map.getLayer(HIT_LAYER_ID)) {
      return;
    }
    interactionsAttached = true;
    map.on("mouseenter", HIT_LAYER_ID, () => {
      map.getCanvas().style.cursor = "pointer";
    });
    map.on("mouseleave", HIT_LAYER_ID, () => {
      map.getCanvas().style.cursor = "";
    });
    map.on("click", HIT_LAYER_ID, (event) => {
      if (map.getLayer(WAYPOINT_HIT_LAYER_ID)) {
        const waypointHits = map.queryRenderedFeatures(event.point, { layers: [WAYPOINT_HIT_LAYER_ID] });
        if (waypointHits.length) {
          return;
        }
      }
      const feature = event.features && event.features[0];
      if (!feature) {
        return;
      }
      const props = feature.properties || {};
      const agent = props.agent || "";
      const pathMeta = pathMetaByKey.get(buildPathMetaKey(props)) || pathMetaByKey.get(buildPathMetaKey(feature)) || null;
      const currentWaypointId = getCurrentWaypointId(agent);
      const currentWaypointMeta = Number.isFinite(currentWaypointId) && pathMeta?.waypointModes
        ? pathMeta.waypointModes.get(currentWaypointId) || null
        : null;
      setSelectedAgent(agent);
      const html = `
        <div style="font-size:12px;font-weight:700;margin-bottom:4px;">${agent || "PATH"}</div>
        <div style="font-size:11px;color:#333;">Path ${props.pathId ?? "-"}</div>
        <div style="font-size:11px;color:#333;">Status ${Number(props.isDone) === 1 ? "Done" : "Active"}</div>
        <div style="font-size:11px;color:#333;">Points ${props.points ?? "-"}</div>
        <div style="font-size:11px;color:#333;">${formatAltRange(props.altMin, props.altMax)}</div>
        <div style="font-size:11px;color:#333;">Current WP ${Number.isFinite(currentWaypointId) ? currentWaypointId : "-"}</div>
        <div style="font-size:11px;color:#333;">Current mode ${currentWaypointMeta?.coveragePassLabel || currentWaypointMeta?.passLabel || currentWaypointMeta?.loiterSummary || "N/A"}</div>
        <div style="font-size:11px;color:#333;">Current ${currentWaypointMeta?.fovLabel || "FOV -"}</div>
        <div style="font-size:11px;color:#333;">Pass ${pathMeta?.passSummary || "N/A"}</div>
        <div style="font-size:11px;color:#333;">Input mission ${pathMeta?.inputMissionId ?? "-"}</div>
        <div style="font-size:11px;color:#333;">${formatSweepSpacing(pathMeta?.sweepSpacingSummary)}</div>
      `;
      if (!popup) {
        popup = new maplibregl.Popup({ closeButton: true, closeOnClick: true });
      }
      popup.setLngLat(event.lngLat).setHTML(html).addTo(map);
    });
    map.on("mouseenter", WAYPOINT_HIT_LAYER_ID, () => {
      map.getCanvas().style.cursor = "pointer";
    });
    map.on("mouseleave", WAYPOINT_HIT_LAYER_ID, () => {
      map.getCanvas().style.cursor = "";
    });
    map.on("click", WAYPOINT_HIT_LAYER_ID, (event) => {
      const feature = event.features && event.features[0];
      if (!feature) {
        return;
      }
      const entry = findWaypointEntry(feature.properties || {});
      if (!entry) {
        return;
      }
      openWaypointPopup(entry);
    });
    map.on("mouseenter", AREA_FILL_LAYER_ID, () => {
      map.getCanvas().style.cursor = "pointer";
    });
    map.on("mouseleave", AREA_FILL_LAYER_ID, () => {
      map.getCanvas().style.cursor = "";
    });
    map.on("click", AREA_FILL_LAYER_ID, (event) => {
      const feature = event.features && event.features[0];
      if (!feature) {
        return;
      }
      const props = feature.properties || {};
      const agent = props.agent || "";
      setSelectedAgent(agent);
      const missionCount = Number(props.missionCount);
      const pathCount = Number(props.pathCount);
      const areaPartCount = Number(props.areaPartCount);
      const html = `
        <div style="font-size:12px;font-weight:700;margin-bottom:4px;">${agent || "AREA"}</div>
        <div style="font-size:11px;color:#333;">Assignment owner ${props.ownerKey || agent || "-"}</div>
        <div style="font-size:11px;color:#333;">Input mission ${props.inputMissionId ?? "-"}</div>
        <div style="font-size:11px;color:#333;">Missions ${Number.isFinite(missionCount) ? missionCount : "-"}</div>
        <div style="font-size:11px;color:#333;">Paths ${Number.isFinite(pathCount) ? pathCount : "-"}</div>
        <div style="font-size:11px;color:#333;">Area parts ${Number.isFinite(areaPartCount) ? areaPartCount : "-"}</div>
        <div style="font-size:11px;color:#333;">${formatSweepSpacing(props)}</div>
        <div style="font-size:11px;color:#333;">Status ${props.boundaryGuardStatus || (Number(props.isDone) === 1 ? "Done" : "Active")}</div>
        ${props.boundaryGuardSetID ? `<div style="font-size:11px;color:#333;">Guard cycle ${Number(props.boundaryGuardCycleCount) || 0}</div>` : ""}
      `;
      if (!popup) {
        popup = new maplibregl.Popup({ closeButton: true, closeOnClick: true });
      }
      popup.setLngLat(event.lngLat).setHTML(html).addTo(map);
    });
    [SWEEP_LINE_LAYER_ID, SWEEP_FORWARD_LINE_LAYER_ID, SWEEP_REVERSE_LINE_LAYER_ID]
      .filter((layerId) => Boolean(map.getLayer(layerId)))
      .forEach((layerId) => {
        map.on("mouseenter", layerId, () => {
          map.getCanvas().style.cursor = "pointer";
        });
        map.on("mouseleave", layerId, () => {
          map.getCanvas().style.cursor = "";
        });
        map.on("click", layerId, (event) => {
          const feature = event.features && event.features[0];
          if (!feature) {
            return;
          }
          const props = feature.properties || {};
          const agent = props.agent || "";
          const entry = findWaypointEntry(props);
          if (entry) {
            selectWaypoint(entry);
          } else {
            setSelectedAgent(agent);
          }
          const html = `
            <div style="font-size:12px;font-weight:700;margin-bottom:4px;">${agent || "SWEEP"}</div>
            <div style="font-size:11px;color:#333;">Path ${props.pathId ?? "-"}</div>
            <div style="font-size:11px;color:#333;">Waypoint ${props.waypointId ?? "-"}</div>
            ${props.coveragePassLabel ? `<div style="font-size:11px;color:#333;">Coverage ${props.coveragePassLabel}</div>` : ""}
            <div style="font-size:11px;color:#333;">Mode ${props.passLabel || "N/A"}</div>
            <div style="font-size:11px;color:#333;">${props.fovLabel || "FOV -"}</div>
            <div style="font-size:11px;color:#333;">${props.loiterSummary ? `Loiter ${props.loiterSummary}` : "Loiter -"}</div>
            <div style="font-size:11px;color:#333;">Points ${props.pointCount ?? "-"}</div>
            <div style="font-size:11px;color:#333;">Input mission ${props.inputMissionId ?? "-"}</div>
            <div style="font-size:11px;color:#333;">${formatSweepSpacing(props)}</div>
            <div style="font-size:11px;color:#333;">${formatAltRange(Number(props.altMin), Number(props.altMax))}</div>
            <div style="font-size:11px;color:#333;">Status ${props.boundaryGuardStatus || props.coveragePassStatus || (Number(props.isDone) === 1 ? "Done" : "Active")}</div>
          `;
          if (!popup) {
            popup = new maplibregl.Popup({ closeButton: true, closeOnClick: true });
          }
          popup.setLngLat(event.lngLat).setHTML(html).addTo(map);
        });
      });
    map.on("mouseenter", SWEEP_POINT_LAYER_ID, () => {
      map.getCanvas().style.cursor = "pointer";
    });
    map.on("mouseleave", SWEEP_POINT_LAYER_ID, () => {
      map.getCanvas().style.cursor = "";
    });
    map.on("click", SWEEP_POINT_LAYER_ID, (event) => {
      const feature = event.features && event.features[0];
      if (!feature) {
        return;
      }
      const props = feature.properties || {};
      const agent = props.agent || "";
      const entry = findWaypointEntry(props);
      if (entry) {
        selectWaypoint(entry);
      } else {
        setSelectedAgent(agent);
      }
      const html = `
        <div style="font-size:12px;font-weight:700;margin-bottom:4px;">${agent || "SWEEP POINT"}</div>
        <div style="font-size:11px;color:#333;">Path ${props.pathId ?? "-"}</div>
        <div style="font-size:11px;color:#333;">Waypoint ${props.waypointId ?? "-"}</div>
        ${props.coveragePassLabel ? `<div style="font-size:11px;color:#333;">Coverage ${props.coveragePassLabel}</div>` : ""}
        <div style="font-size:11px;color:#333;">Mode ${props.passLabel || "N/A"}</div>
        <div style="font-size:11px;color:#333;">${props.fovLabel || "FOV -"}</div>
        <div style="font-size:11px;color:#333;">${props.loiterSummary ? `Loiter ${props.loiterSummary}` : "Loiter -"}</div>
        <div style="font-size:11px;color:#333;">Point ${props.pointIndex ?? "-"} / ${props.pointCount ?? "-"}</div>
        <div style="font-size:11px;color:#333;">Input mission ${props.inputMissionId ?? "-"}</div>
        <div style="font-size:11px;color:#333;">${formatSweepSpacing(props)}</div>
        <div style="font-size:11px;color:#333;">Alt ${Math.round(Number(props.altitude) || 0)} m</div>
        <div style="font-size:11px;color:#333;">Status ${props.boundaryGuardStatus || props.coveragePassStatus || (Number(props.isDone) === 1 ? "Done" : "Active")}</div>
      `;
      if (!popup) {
        popup = new maplibregl.Popup({ closeButton: true, closeOnClick: true });
      }
      popup.setLngLat(event.lngLat).setHTML(html).addTo(map);
    });
  };

  const applyData = (payload) => {
    const ok = payload && payload.ok;
    features = ok && Array.isArray(payload.features) ? payload.features : [];
    agentCounts = ok && payload.agents ? payload.agents : {};
    pathMetaByKey = ok ? buildPathMetaMap(payload) : new Map();
    visualRebuildRevision += 1;
    ensureHitLayer();
    ensureWaypointHitLayer();
    ensureAreaLayers();
    ensureSweepLayers();
    ensure3dLayers();
    updateHitSource();
    updateAreaSource(payload);
    updateSweepSource(payload);
    update3dPositions();
    updateLayerAlpha();
    setLegendVisibility(
      features.length > 0 || coverageDepthLegend?.dataset.available === "1",
    );
    updateLegend();
    attachInteractions();
    if ((areaFeatures.length || features.length) && map && typeof map.fitBounds === "function" && !didFitBounds) {
      let minLon = Infinity;
      let minLat = Infinity;
      let maxLon = -Infinity;
      let maxLat = -Infinity;

      const includeCoordinate = (coord) => {
        if (!Array.isArray(coord)) {
          return;
        }
        if (coord.length >= 2 && Number.isFinite(Number(coord[0])) && Number.isFinite(Number(coord[1]))) {
          const lon = Number(coord[0]);
          const lat = Number(coord[1]);
          minLon = Math.min(minLon, lon);
          minLat = Math.min(minLat, lat);
          maxLon = Math.max(maxLon, lon);
          maxLat = Math.max(maxLat, lat);
          return;
        }
        coord.forEach(includeCoordinate);
      };

      // Prefer the actual assigned mission areas.  Fall back to flight paths
      // for line/point-only missions that do not carry area polygons.
      if (areaFeatures.length) {
        areaFeatures.forEach((feature) => includeCoordinate(feature?.geometry?.coordinates));
      } else {
        features.forEach((feature) => includeCoordinate(feature?.coords));
      }
      if (Number.isFinite(minLon) && Number.isFinite(minLat) && Number.isFinite(maxLon) && Number.isFinite(maxLat)) {
        const terrainMarginM = 5000;
        const centerLat = 0.5 * (minLat + maxLat);
        const latPad = terrainMarginM / 111320.0;
        const lonScale = Math.max(1e-6, Math.cos((centerLat * Math.PI) / 180));
        const lonPad = terrainMarginM / (111320.0 * lonScale);
        map.fitBounds(
          [
            [minLon - lonPad, minLat - latPad],
            [maxLon + lonPad, maxLat + latPad],
          ],
          { padding: 80, duration: 600 },
        );
        didFitBounds = true;
      }
    }
  };

  const loadFromResponse = (payload) => {
    pendingData = payload;
    didFitBounds = false;
    if (mapReady) {
      applyData(payload);
    }
  };

  const loadFromServer = async (path) => {
    const response = await fetch("/api/mission/load", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ path }),
    });
    const payload = await response.json();
    loadFromResponse(payload);
    return payload;
  };

  if (legendItems.length > 0) {
    legendItems.forEach((item) => {
      item.addEventListener("click", () => {
        if (item.classList.contains("is-disabled")) {
          return;
        }
        const agent = item.dataset.agent || null;
        if (selectedAgent === agent) {
          setSelectedAgent(null);
        } else {
          setSelectedAgent(agent);
        }
      });
    });
  }

  map.on("load", () => {
    mapReady = true;
    ensureHitLayer();
    ensureWaypointHitLayer();
    ensureAreaLayers();
    ensureSweepLayers();
    ensure3dLayers();
    attachInteractions();
    applyWaypointVisibility();
    applySweepLineVisibility();
    if (pendingData) {
      applyData(pendingData);
    }
  });

  // Dash geometry depends on zoom, but rebuilding every intermediate zoom
  // frame repeatedly queries/converts the complete mission. Rebuild once at
  // the settled zoom; MapLibre continues projecting the existing geometry
  // correctly during the gesture.
  map.on("zoomend", () => {
    if (features.length) {
      scheduleRebuild();
    }
  });

  map.on("resize", () => {
    if (features.length) {
      scheduleRebuild();
    }
  });

  map.on("idle", () => {
    if (features.length) {
      scheduleRebuild();
    }
  });

  return {
    loadFromResponse,
    loadFromServer,
    setSelectedAgent,
    setCurrentWaypoints,
    setWaypointsVisible,
    setSweepLinesVisible,
  };
};
