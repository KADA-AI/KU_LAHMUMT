import { logStatus } from "./status_log.js";

const SOURCE_ID = "mission-paths";
const HIT_LAYER_ID = "mission-paths-hit";
const AREA_SOURCE_ID = "mission-areas";
const AREA_FILL_LAYER_ID = "mission-areas-fill";
const AREA_LINE_LAYER_ID = "mission-areas-line";
const SWEEP_SOURCE_ID = "mission-sweep";
const SWEEP_LINE_LAYER_ID = "mission-sweep-line";
const SWEEP_POINT_LAYER_ID = "mission-sweep-point";

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
  if (typeof matrix.toFloat32Array === "function") {
    const arr = matrix.toFloat32Array();
    return arr ? new Float32Array(arr) : null;
  }
  if (typeof matrix.toFloat64Array === "function") {
    const arr = matrix.toFloat64Array();
    return arr ? new Float32Array(arr) : null;
  }
  if (ArrayBuffer.isView(matrix) && typeof matrix.length === "number") {
    return new Float32Array(matrix);
  }
  if (matrix instanceof Float32Array) {
    return matrix;
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
  if (Number.isFinite(loiterRadius)) {
    loiterBits.push(`R${Math.round(loiterRadius)}m`);
  }
  if (Number.isFinite(loiterTime)) {
    loiterBits.push(`T${Math.round(loiterTime)}s`);
  }
  if (Number.isFinite(loiterSpeed)) {
    loiterBits.push(`V${Math.round(loiterSpeed)}m/s`);
  }
  if (Number.isFinite(loiterDirection)) {
    loiterBits.push(loiterDirection === 1 ? "CW" : loiterDirection === 2 ? "CCW" : "DIR");
  }
  return {
    passType,
    passLabel,
    isLoiter: passType === 2,
    isFlyBy: passType === 1,
    isFlyOver: passType === 3,
    loiter,
    loiterSummary: loiterBits.length ? loiterBits.join(" · ") : null,
    fovDeg: Number.isFinite(fovDeg) && fovDeg > 0 ? fovDeg : null,
    fovLabel: formatFovDeg(fovDeg),
  };
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

const buildGeoFeature = (feature, colors, pathMetaMap = null) => {
  const coords = Array.isArray(feature?.coords) ? feature.coords : [];
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
    },
  };
};

const buildAreaFeatures = (payload, colors, selectedAgent) => {
  const plans = Array.isArray(payload?.individualMissionPlans) ? payload.individualMissionPlans : [];
  const pathMissionMap = buildPathMissionMap(payload);
  const sweepSpacingMap = buildSweepSpacingMap(payload);
  const hasSelection = Boolean(selectedAgent);
  const features = [];
  let featureId = 1;
  plans.forEach((plan) => {
    const aircraftId = Number(plan?.aircraftID);
    if (!Number.isFinite(aircraftId)) {
      return;
    }
    const agent = aircraftId >= 1 && aircraftId <= 3
      ? `LAH${aircraftId}`
      : aircraftId >= 4 && aircraftId <= 6
        ? `UAV${aircraftId - 3}`
        : `AC${aircraftId}`;
    const baseColor = colors[agent] || "#e7eddc";
    const missions = Array.isArray(plan?.individualMissionList) ? plan.individualMissionList : [];
    missions.forEach((mission) => {
      const missionId = Number(mission?.individualMissionID);
      const pathId = Number(mission?.pathID);
      const missionInfo = pathMissionMap.get(pathId) || {};
      const inputMissionId = Number(missionInfo.inputMissionId);
      const spacing = Number.isFinite(inputMissionId) ? sweepSpacingMap.get(inputMissionId) : null;
      const isDone = Boolean(mission?.isDone);
      const info = mission?.individualMissionInfo || {};
      const areaList = Array.isArray(info?.areaList) ? info.areaList : [];
      const outerAreas = areaList.filter((area) => area && !area.isHole);
      outerAreas.forEach((area, areaIdx) => {
        const coordinateList = Array.isArray(area?.coordinateList) ? area.coordinateList : [];
        const ring = coordinateList
          .map((coord) => {
            const lat = Number(coord?.latitude);
            const lon = Number(coord?.longitude);
            if (!Number.isFinite(lat) || !Number.isFinite(lon)) {
              return null;
            }
            return [lon, lat];
          })
          .filter(Boolean);
        if (ring.length < 3) {
          return;
        }
        const first = ring[0];
        const last = ring[ring.length - 1];
        if (first[0] !== last[0] || first[1] !== last[1]) {
          ring.push([...first]);
        }
        const fillOpacity = hasSelection
          ? agent === selectedAgent
            ? isDone ? AREA_FILL_DONE_ALPHA : AREA_FILL_ALPHA
            : isDone ? AREA_FILL_DONE_DIM_ALPHA : AREA_FILL_DIM_ALPHA
          : isDone ? AREA_FILL_DONE_ALPHA : AREA_FILL_ALPHA;
        const lineOpacity = hasSelection
          ? agent === selectedAgent
            ? isDone ? AREA_LINE_DONE_ALPHA : AREA_LINE_ALPHA
            : isDone ? AREA_LINE_DONE_DIM_ALPHA : AREA_LINE_DIM_ALPHA
          : isDone ? AREA_LINE_DONE_ALPHA : AREA_LINE_ALPHA;
        features.push({
          type: "Feature",
          id: featureId++,
          geometry: {
            type: "Polygon",
            coordinates: [ring],
          },
          properties: {
            agent,
            aircraftId,
            missionId: Number.isFinite(missionId) ? missionId : null,
            pathId: Number.isFinite(pathId) ? pathId : null,
            inputMissionId: Number.isFinite(inputMissionId) ? inputMissionId : null,
            sweepAvgSpacingM: Number.isFinite(Number(spacing?.averageLineSpacingM))
              ? Number(spacing.averageLineSpacingM)
              : null,
            sweepLineCount: Number.isFinite(Number(spacing?.lineCount)) ? Number(spacing.lineCount) : null,
            sweepPairCount: Number.isFinite(Number(spacing?.pairCount)) ? Number(spacing.pairCount) : null,
            areaIndex: areaIdx + 1,
            isDone: isDone ? 1 : 0,
            color: baseColor,
            fillOpacity,
            lineOpacity,
          },
        });
      });
    });
  });
  return features;
};

const buildSweepFeatures = (payload, colors, selectedAgent) => {
  const flightPaths = Array.isArray(payload?.flightPaths) ? payload.flightPaths : [];
  const pathMissionMap = buildPathMissionMap(payload);
  const sweepSpacingMap = buildSweepSpacingMap(payload);
  const hasSelection = Boolean(selectedAgent);
  const features = [];
  let featureId = 1;
  flightPaths.forEach((path) => {
    const aircraftId = Number(path?.aircraftID);
    if (!Number.isFinite(aircraftId)) {
      return;
    }
    const agent = aircraftId >= 1 && aircraftId <= 3
      ? `LAH${aircraftId}`
      : aircraftId >= 4 && aircraftId <= 6
        ? `UAV${aircraftId - 3}`
        : `AC${aircraftId}`;
    const baseColor = colors[agent] || "#e7eddc";
    const pathId = Number(path?.pathID);
    const missionInfo = pathMissionMap.get(pathId) || {};
    const inputMissionId = Number(missionInfo.inputMissionId);
    const spacing = Number.isFinite(inputMissionId) ? sweepSpacingMap.get(inputMissionId) : null;
    const waypointList = Array.isArray(path?.waypointList) ? path.waypointList : [];
    const passCounts = { 1: 0, 2: 0, 3: 0 };
    waypointList.forEach((wp) => {
      const waypointId = Number(wp?.waypointID);
      const isDone = Boolean(wp?.isDone);
      const passInfo = getWaypointPassInfo(wp);
      if (Number.isFinite(passInfo.passType) && passCounts[passInfo.passType] !== undefined) {
        passCounts[passInfo.passType] += 1;
      }
      const filming = wp?.filmingProperty || {};
      const lineSearch = filming?.lineSearch || {};
      const coordinateList = Array.isArray(lineSearch?.coordinateList) ? lineSearch.coordinateList : [];
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
      const lineOpacity = hasSelection
        ? agent === selectedAgent
          ? isDone ? SWEEP_LINE_DONE_ALPHA : SWEEP_LINE_ALPHA
          : isDone ? SWEEP_LINE_DONE_DIM_ALPHA : SWEEP_LINE_DIM_ALPHA
        : isDone ? SWEEP_LINE_DONE_ALPHA : SWEEP_LINE_ALPHA;
      const pointOpacity = hasSelection
        ? agent === selectedAgent
          ? isDone ? SWEEP_POINT_DONE_ALPHA : SWEEP_POINT_ALPHA
          : isDone ? SWEEP_POINT_DONE_DIM_ALPHA : SWEEP_POINT_DIM_ALPHA
        : isDone ? SWEEP_POINT_DONE_ALPHA : SWEEP_POINT_ALPHA;
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
        altMin: minAlt,
        altMax: maxAlt,
        passType: Number.isFinite(passInfo.passType) ? passInfo.passType : null,
        passLabel: passInfo.passLabel,
        loiterSummary: passInfo.loiterSummary,
        fovDeg: passInfo.fovDeg,
        fovLabel: passInfo.fovLabel,
        isLoiter: passInfo.isLoiter ? 1 : 0,
        isFlyBy: passInfo.isFlyBy ? 1 : 0,
        isFlyOver: passInfo.isFlyOver ? 1 : 0,
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
        const chunkSize = Math.max(2, Number(lineSearch?.interpolationPoints) || 0);
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
            pointRadius: isEndpoint ? 4.5 : 3.2,
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
      if (typeof this._renderHook === "function") {
        this._renderHook(mat);
      }
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
  let sweepLinesVisible = true;
  let currentWaypointsByAgent = new Map();
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
  const colors = getAgentColors();
  const agentLayers = new Map();
  const doneAgentLayers = new Map();
  const waypointLayers = new Map();
  const doneWaypointLayers = new Map();
  const currentWaypointLayers = new Map();
  let labelContainer = null;
  const labelElements = new Map();
  let waypointLabelEntries = [];
  let labelHookLayer = null;

  const setLegendVisibility = (visible) => {
    if (!legend) {
      return;
    }
    legend.classList.toggle("is-visible", visible);
    legend.setAttribute("aria-hidden", visible ? "false" : "true");
  };

  const updateLegend = () => {
    if (legendItems.length === 0) {
      return;
    }
    legendItems.forEach((item) => {
      const agent = item.dataset.agent || "";
      const available = Boolean(agentCounts[agent]);
      item.classList.toggle("is-disabled", !available);
      item.classList.toggle("is-active", selectedAgent === agent);
      item.setAttribute("aria-disabled", available ? "false" : "true");
    });
  };

  const applySweepLineVisibility = () => {
    const visibility = sweepLinesVisible ? "visible" : "none";
    [SWEEP_LINE_LAYER_ID, SWEEP_POINT_LAYER_ID].forEach((layerId) => {
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
      const waypointList = Array.isArray(path?.waypointList) ? path.waypointList : [];
      const passCounts = { 1: 0, 2: 0, 3: 0 };
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
        waypointModes.set(waypointId, {
          passType: passInfo.passType,
          passLabel: passInfo.passLabel,
          loiterSummary: passInfo.loiterSummary,
          fovDeg: passInfo.fovDeg,
          fovLabel: passInfo.fovLabel,
          isLoiter: passInfo.isLoiter,
          isFlyBy: passInfo.isFlyBy,
          isFlyOver: passInfo.isFlyOver,
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

  const buildWaypointPopupHtml = (entry) => {
    const mode = entry.passLabel || "N/A";
    const currentBadge = entry.isCurrent ? "Current" : "Waypoint";
    const details = [
      entry.pathId !== null && entry.pathId !== undefined ? `Path ${entry.pathId}` : null,
      entry.waypointId !== null && entry.waypointId !== undefined ? `WP ${entry.waypointId}` : null,
      `Mode ${mode}`,
      entry.fovLabel,
      entry.loiterSummary ? `Loiter ${entry.loiterSummary}` : null,
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

  const applyWaypointLabelStyle = (el, entry, color) => {
    const isCurrent = Boolean(entry.isCurrent);
    const isDone = Boolean(entry.isDone);
    const bg = isCurrent
      ? "rgba(255, 255, 255, 0.96)"
      : "rgba(0, 0, 0, 0.45)";
    const border = isCurrent ? `1px solid ${color || "#e7eddc"}` : "1px solid rgba(255,255,255,0.08)";
    const shadow = isCurrent
      ? `0 0 0 2px rgba(255,255,255,0.35), 0 10px 18px rgba(0,0,0,0.24), 0 0 24px ${color || "#ffffff"}55`
      : "0 4px 10px rgba(0,0,0,0.18)";
    const primaryColor = isCurrent ? "#0d1117" : color || "#e7eddc";
    const secondaryColor = isCurrent ? "#345" : "rgba(232, 240, 223, 0.72)";
    const summary = [entry.passLabel || entry.loiterSummary || null, entry.fovLabel]
      .filter(Boolean)
      .join(" · ") || "Pass N/A";
    el.innerHTML = `
      <div style="font-size:${isCurrent ? 12 : 11}px;line-height:1.05;letter-spacing:0.04em;">${entry.label || `WP${entry.idx}`}</div>
      <div style="margin-top:2px;font-size:9px;line-height:1.05;letter-spacing:0.08em;color:${secondaryColor};">${summary}</div>
      ${isCurrent ? "<div style=\"margin-top:2px;font-size:8px;line-height:1;letter-spacing:0.14em;color:#0a6cff;\">CURRENT</div>" : ""}
    `;
    el.style.color = primaryColor;
    el.style.background = bg;
    el.style.border = border;
    el.style.boxShadow = shadow;
    el.style.opacity = "1";
    el.style.transform = isCurrent
      ? "translate(-50%, -120%) scale(1.08)"
      : "translate(-50%, -120%) scale(1)";
    el.style.zIndex = isCurrent ? "4" : isDone ? "2" : "1";
    el.style.pointerEvents = "auto";
    el.style.cursor = "pointer";
    el.title = `${entry.agent || "WP"} ${entry.label || `WP${entry.idx}`} | ${summary}`;
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
      labelContainer.appendChild(el);
      labelElements.set(key, el);
      el.addEventListener("click", (event) => {
        event.stopPropagation();
        const popupEntry = waypointLabelEntries.find((item) => item.key === key);
        if (!popupEntry) {
          return;
        }
        if (!popup) {
          popup = new maplibregl.Popup({ closeButton: true, closeOnClick: true });
        }
        popup
          .setLngLat(popupEntry.lngLat || [popupEntry.coord.x, popupEntry.coord.y])
          .setHTML(buildWaypointPopupHtml(popupEntry))
          .addTo(map);
        setSelectedAgent(popupEntry.agent);
      });
    }
    el.dataset.done = entry.isDone ? "1" : "0";
    el.dataset.current = entry.isCurrent ? "1" : "0";
    applyWaypointLabelStyle(el, entry, color || "#e7eddc");
    return el;
  };

  const clearWaypointLabels = () => {
    labelElements.forEach((el) => el.remove());
    labelElements.clear();
    waypointLabelEntries = [];
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
      }
    });
    waypointLabelEntries = entries;
    updateWaypointLabelVisibility();
  };

  const updateWaypointLabelVisibility = () => {
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
      el.style.opacity = String(alpha);
    });
  };

  const updateWaypointLabelPositions = (matrix) => {
    if (!labelContainer || !matrix) {
      return;
    }
    const canvas = map.getCanvas();
    const dpr = window.devicePixelRatio || 1;
    const width = canvas.width / dpr;
    const height = canvas.height / dpr;
    labelElements.forEach((el) => {
      el.style.display = "none";
    });
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
      el.style.left = `${point.x}px`;
      el.style.top = `${point.y}px`;
      el.style.display = "block";
    });
  };

  const buildLinePositions = (coords, alts) => {
    if (!Array.isArray(coords) || coords.length < 2) {
      return [];
    }
    const positions = [];
    const getTerrainElevation = (lon, lat) => {
      if (typeof map.queryTerrainElevation !== "function") {
        return 0;
      }
      const elev = map.queryTerrainElevation({ lng: lon, lat: lat }, { exaggerated: false });
      return Number.isFinite(elev) ? elev : 0;
    };
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
      const terrainA = getTerrainElevation(lonA, latA);
      const terrainB = getTerrainElevation(lonB, latB);
      const altStart = Number.isFinite(altA) ? altA + ALT_OFFSET_M : terrainA + ALT_OFFSET_M;
      const altEnd = Number.isFinite(altB) ? altB + ALT_OFFSET_M : terrainB + ALT_OFFSET_M;
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
        pushQuad(side1);
        if (side2) {
          pushQuad(side2);
        }
      };
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
    const waypointSize = WAYPOINT_SIZE_PX * (window.devicePixelRatio || 1);
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
        filter: ["==", ["geometry-type"], "LineString"],
        layout: {
          "line-join": "round",
          "line-cap": "butt",
        },
        paint: {
          "line-color": ["get", "color"],
          "line-width": ["interpolate", ["linear"], ["zoom"], 8, 1.0, 12, 1.5, 16, 2.2],
          "line-dasharray": [1.5, 2.4],
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
          "circle-stroke-width": ["case", ["==", ["get", "isEndpoint"], 1], 1.8, 1.1],
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
    areaFeatures = buildAreaFeatures(payloadForAreas, colors, selectedAgent);
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
    sweepFeatures = buildSweepFeatures(payloadForAreas, colors, selectedAgent);
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
    const getTerrainElevation = (lon, lat) => {
      if (typeof map.queryTerrainElevation !== "function") {
        return 0;
      }
      const elev = map.queryTerrainElevation({ lng: lon, lat: lat }, { exaggerated: false });
      return Number.isFinite(elev) ? elev : 0;
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
        const terrain = getTerrainElevation(lon, lat);
        const altitude = Number.isFinite(alt)
          ? alt + ALT_OFFSET_M + WAYPOINT_Z_OFFSET_M
          : terrain + ALT_OFFSET_M + WAYPOINT_Z_OFFSET_M;
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
      const segmentPositions = buildLinePositions(feature.coords, feature.alts);
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
          const terrain = getTerrainElevation(lon, lat);
          const altitude = Number.isFinite(alt)
            ? alt + ALT_OFFSET_M + WAYPOINT_LABEL_Z_OFFSET_M
            : terrain + ALT_OFFSET_M + WAYPOINT_LABEL_Z_OFFSET_M;
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
            loiterSummary: passMeta?.loiterSummary || null,
            fovDeg: passMeta?.fovDeg ?? null,
            fovLabel: passMeta?.fovLabel || null,
            passSummary: pathMeta?.passSummary || null,
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
    const waypointSize = WAYPOINT_SIZE_PX * (window.devicePixelRatio || 1);
    waypointPositionsByAgent.forEach((positions, agent) => {
      const layer = waypointLayers.get(agent);
      if (!layer) {
        return;
      }
      if (typeof layer.setColor === "function") {
        layer.setColor(colors[agent] || "#e7eddc");
      }
      layer.setVisible(Boolean(positions.length));
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
      layer.setVisible(Boolean(positions.length));
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
      layer.setVisible(Boolean(positions.length));
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
    updateWaypointLabelVisibility();
    map.triggerRepaint();
  };

  const setSelectedAgent = (agent) => {
    selectedAgent = agent || null;
    updateLegend();
    updateLayerAlpha();
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
      return;
    }
    currentWaypointsByAgent = next;
    visualRebuildRevision += 1;
    updateLayerAlpha();
    scheduleRebuild(true);
  };

  const setSweepLinesVisible = (visible) => {
    sweepLinesVisible = Boolean(visible);
    applySweepLineVisibility();
  };

  const syncCurrentWaypointsFromState = (state) => {
    const next = {};
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
    });
    setCurrentWaypoints(next);
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
        <div style="font-size:11px;color:#333;">Current mode ${currentWaypointMeta?.passLabel || currentWaypointMeta?.loiterSummary || "N/A"}</div>
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
      const html = `
        <div style="font-size:12px;font-weight:700;margin-bottom:4px;">${agent || "AREA"}</div>
        <div style="font-size:11px;color:#333;">Mission ${props.missionId ?? "-"}</div>
        <div style="font-size:11px;color:#333;">Input mission ${props.inputMissionId ?? "-"}</div>
        <div style="font-size:11px;color:#333;">Path ${props.pathId ?? "-"}</div>
        <div style="font-size:11px;color:#333;">Area ${props.areaIndex ?? "-"}</div>
        <div style="font-size:11px;color:#333;">${formatSweepSpacing(props)}</div>
        <div style="font-size:11px;color:#333;">Status ${Number(props.isDone) === 1 ? "Done" : "Active"}</div>
      `;
      if (!popup) {
        popup = new maplibregl.Popup({ closeButton: true, closeOnClick: true });
      }
      popup.setLngLat(event.lngLat).setHTML(html).addTo(map);
    });
    map.on("mouseenter", SWEEP_LINE_LAYER_ID, () => {
      map.getCanvas().style.cursor = "pointer";
    });
    map.on("mouseleave", SWEEP_LINE_LAYER_ID, () => {
      map.getCanvas().style.cursor = "";
    });
    map.on("click", SWEEP_LINE_LAYER_ID, (event) => {
      const feature = event.features && event.features[0];
      if (!feature) {
        return;
      }
      const props = feature.properties || {};
      const agent = props.agent || "";
      setSelectedAgent(agent);
      const html = `
        <div style="font-size:12px;font-weight:700;margin-bottom:4px;">${agent || "SWEEP"}</div>
        <div style="font-size:11px;color:#333;">Path ${props.pathId ?? "-"}</div>
        <div style="font-size:11px;color:#333;">Waypoint ${props.waypointId ?? "-"}</div>
        <div style="font-size:11px;color:#333;">Mode ${props.passLabel || "N/A"}</div>
        <div style="font-size:11px;color:#333;">${props.fovLabel || "FOV -"}</div>
        <div style="font-size:11px;color:#333;">${props.loiterSummary ? `Loiter ${props.loiterSummary}` : "Loiter -"}</div>
        <div style="font-size:11px;color:#333;">Points ${props.pointCount ?? "-"}</div>
        <div style="font-size:11px;color:#333;">Input mission ${props.inputMissionId ?? "-"}</div>
        <div style="font-size:11px;color:#333;">${formatSweepSpacing(props)}</div>
        <div style="font-size:11px;color:#333;">${formatAltRange(Number(props.altMin), Number(props.altMax))}</div>
        <div style="font-size:11px;color:#333;">Status ${Number(props.isDone) === 1 ? "Done" : "Active"}</div>
      `;
      if (!popup) {
        popup = new maplibregl.Popup({ closeButton: true, closeOnClick: true });
      }
      popup.setLngLat(event.lngLat).setHTML(html).addTo(map);
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
      setSelectedAgent(agent);
      const html = `
        <div style="font-size:12px;font-weight:700;margin-bottom:4px;">${agent || "SWEEP POINT"}</div>
        <div style="font-size:11px;color:#333;">Path ${props.pathId ?? "-"}</div>
        <div style="font-size:11px;color:#333;">Waypoint ${props.waypointId ?? "-"}</div>
        <div style="font-size:11px;color:#333;">Mode ${props.passLabel || "N/A"}</div>
        <div style="font-size:11px;color:#333;">${props.fovLabel || "FOV -"}</div>
        <div style="font-size:11px;color:#333;">${props.loiterSummary ? `Loiter ${props.loiterSummary}` : "Loiter -"}</div>
        <div style="font-size:11px;color:#333;">Point ${props.pointIndex ?? "-"} / ${props.pointCount ?? "-"}</div>
        <div style="font-size:11px;color:#333;">Input mission ${props.inputMissionId ?? "-"}</div>
        <div style="font-size:11px;color:#333;">${formatSweepSpacing(props)}</div>
        <div style="font-size:11px;color:#333;">Alt ${Math.round(Number(props.altitude) || 0)} m</div>
        <div style="font-size:11px;color:#333;">Status ${Number(props.isDone) === 1 ? "Done" : "Active"}</div>
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
    ensureAreaLayers();
    ensureSweepLayers();
    ensure3dLayers();
    updateHitSource();
    updateAreaSource(payload);
    updateSweepSource(payload);
    update3dPositions();
    updateLayerAlpha();
    setLegendVisibility(features.length > 0);
    updateLegend();
    attachInteractions();
    if (features.length && map && typeof map.fitBounds === "function" && !didFitBounds) {
      let minLon = Infinity;
      let minLat = Infinity;
      let maxLon = -Infinity;
      let maxLat = -Infinity;
      features.forEach((feature) => {
        if (!Array.isArray(feature.coords)) {
          return;
        }
        feature.coords.forEach((coord) => {
          const lon = Number(coord[0]);
          const lat = Number(coord[1]);
          if (!Number.isFinite(lon) || !Number.isFinite(lat)) {
            return;
          }
          minLon = Math.min(minLon, lon);
          minLat = Math.min(minLat, lat);
          maxLon = Math.max(maxLon, lon);
          maxLat = Math.max(maxLat, lat);
        });
      });
      if (Number.isFinite(minLon) && Number.isFinite(minLat) && Number.isFinite(maxLon) && Number.isFinite(maxLat)) {
        map.fitBounds(
          [
            [minLon, minLat],
            [maxLon, maxLat],
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
    ensureAreaLayers();
    ensureSweepLayers();
    ensure3dLayers();
    attachInteractions();
    applySweepLineVisibility();
    if (pendingData) {
      applyData(pendingData);
    }
  });

  map.on("zoom", () => {
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
    setSweepLinesVisible,
  };
};
