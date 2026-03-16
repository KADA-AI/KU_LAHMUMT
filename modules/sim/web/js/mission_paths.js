import { logStatus } from "./status_log.js";

const SOURCE_ID = "mission-paths";
const HIT_LAYER_ID = "mission-paths-hit";
const AREA_SOURCE_ID = "mission-areas";
const AREA_FILL_LAYER_ID = "mission-areas-fill";
const AREA_LINE_LAYER_ID = "mission-areas-line";
const LINE_AREA_SOURCE_ID = "mission-line-areas";
const LINE_AREA_FILL_LAYER_ID = "mission-line-areas-fill";
const LINE_AREA_LAYER_ID = "mission-line-areas-line";

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
const LINE_AREA_FILL_ALPHA = 0.12;
const LINE_AREA_FILL_DONE_ALPHA = 0.07;
const LINE_AREA_FILL_DIM_ALPHA = 0.035;
const LINE_AREA_FILL_DONE_DIM_ALPHA = 0.02;
const LINE_AREA_ALPHA = 0.28;
const LINE_AREA_DONE_ALPHA = 0.16;
const LINE_AREA_DIM_ALPHA = 0.08;
const LINE_AREA_DONE_DIM_ALPHA = 0.05;

const PATH_WIDTH_PX = 2.5;
const WAYPOINT_SIZE_PX = 10;
const WAYPOINT_ALPHA = 0.9;
const WAYPOINT_DIM_ALPHA = 0.25;
const WAYPOINT_DONE_ALPHA = 0.3;
const WAYPOINT_DONE_SELECT_ALPHA = 0.35;
const WAYPOINT_DONE_DIM_ALPHA = 0.1;
const WAYPOINT_Z_OFFSET_M = 4.0;
const WAYPOINT_LABEL_Z_OFFSET_M = 14.0;
const WAYPOINT_LABEL_FONT_SIZE = 12;
const ALT_OFFSET_M = 5;
const DASH_ON_PX = 6;
const DASH_OFF_PX = 6;
const EARTH_RADIUS_M = 6371008.8;

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

const normalize2 = (vec) => {
  const x = Number(vec?.x);
  const y = Number(vec?.y);
  const mag = Math.hypot(x, y);
  if (!Number.isFinite(mag) || mag <= 1e-6) {
    return null;
  }
  return { x: x / mag, y: y / mag };
};

const buildLineCorridorRing = (coords, widthMeters) => {
  if (!Array.isArray(coords) || coords.length < 2) {
    return null;
  }
  const half = Number(widthMeters) * 0.5;
  if (!Number.isFinite(half) || half <= 0) {
    return null;
  }
  const avgLat = coords.reduce((sum, coord) => sum + Number(coord[1] || 0), 0) / Math.max(1, coords.length);
  const metersPerDegLat = 111320.0;
  const cosLat = Math.cos(toRad(avgLat)) || 1e-6;
  const metersPerDegLon = metersPerDegLat * cosLat;
  const originLon = Number(coords[0][0]);
  const originLat = Number(coords[0][1]);
  const points = coords.map(([lon, lat]) => ({
    x: (Number(lon) - originLon) * metersPerDegLon,
    y: (Number(lat) - originLat) * metersPerDegLat,
  }));
  const left = [];
  const right = [];
  for (let i = 0; i < points.length; i += 1) {
    const p = points[i];
    const prev = i > 0 ? points[i - 1] : null;
    const next = i < points.length - 1 ? points[i + 1] : null;
    let tangent = null;
    if (prev && next) {
      const inDir = normalize2({ x: p.x - prev.x, y: p.y - prev.y });
      const outDir = normalize2({ x: next.x - p.x, y: next.y - p.y });
      if (inDir && outDir) {
        tangent = normalize2({ x: inDir.x + outDir.x, y: inDir.y + outDir.y });
        if (!tangent) {
          tangent = outDir;
        }
      } else {
        tangent = outDir || inDir;
      }
    } else if (next) {
      tangent = normalize2({ x: next.x - p.x, y: next.y - p.y });
    } else if (prev) {
      tangent = normalize2({ x: p.x - prev.x, y: p.y - prev.y });
    }
    if (!tangent) {
      continue;
    }
    const normal = { x: -tangent.y, y: tangent.x };
    left.push({ x: p.x + normal.x * half, y: p.y + normal.y * half });
    right.push({ x: p.x - normal.x * half, y: p.y - normal.y * half });
  }
  if (left.length < 2 || right.length < 2) {
    return null;
  }
  const ring = [...left, ...right.reverse()].map((pt) => ([
    originLon + pt.x / metersPerDegLon,
    originLat + pt.y / metersPerDegLat,
  ]));
  if (ring.length < 4) {
    return null;
  }
  ring.push([...ring[0]]);
  return ring;
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

const buildGeoFeature = (feature, colors) => {
  const coords = Array.isArray(feature?.coords) ? feature.coords : [];
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
    },
  };
};

const buildAreaFeatures = (payload, colors, selectedAgent) => {
  const plans = Array.isArray(payload?.individualMissionPlans) ? payload.individualMissionPlans : [];
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

const buildLineAreaFeatures = (payload, colors, selectedAgent, map) => {
  const plans = Array.isArray(payload?.individualMissionPlans) ? payload.individualMissionPlans : [];
  const hasSelection = Boolean(selectedAgent);
  const features = [];
  let featureId = 1;
  const zoom = typeof map?.getZoom === "function" ? map.getZoom() : 10;
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
      const isDone = Boolean(mission?.isDone);
      const info = mission?.individualMissionInfo || {};
      const lineList = Array.isArray(info?.lineList) ? info.lineList : [];
      lineList.forEach((line, lineIdx) => {
        const coordinateList = Array.isArray(line?.coordinateList) ? line.coordinateList : [];
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
        if (coords.length < 2) {
          return;
        }
        const widthMeters = Number(line?.width);
        if (!Number.isFinite(widthMeters) || widthMeters <= 0) {
          return;
        }
        const ring = buildLineCorridorRing(coords, widthMeters);
        if (!ring) {
          return;
        }
        const fillOpacity = hasSelection
          ? agent === selectedAgent
            ? isDone ? LINE_AREA_FILL_DONE_ALPHA : LINE_AREA_FILL_ALPHA
            : isDone ? LINE_AREA_FILL_DONE_DIM_ALPHA : LINE_AREA_FILL_DIM_ALPHA
          : isDone ? LINE_AREA_FILL_DONE_ALPHA : LINE_AREA_FILL_ALPHA;
        const lineOpacity = hasSelection
          ? agent === selectedAgent
            ? isDone ? LINE_AREA_DONE_ALPHA : LINE_AREA_ALPHA
            : isDone ? LINE_AREA_DONE_DIM_ALPHA : LINE_AREA_DIM_ALPHA
          : isDone ? LINE_AREA_DONE_ALPHA : LINE_AREA_ALPHA;
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
            lineIndex: lineIdx + 1,
            isDone: isDone ? 1 : 0,
            color: baseColor,
            fillOpacity,
            lineOpacity,
            widthMeters,
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
  let features = [];
  let areaFeatures = [];
  let lineAreaFeatures = [];
  let agentCounts = {};
  let mapReady = typeof map.isStyleLoaded === "function" ? map.isStyleLoaded() : map.loaded();
  let popup = null;
  let rebuildScheduled = false;
  let didFitBounds = false;

  const legend = document.getElementById("mission-legend");
  const legendItems = Array.from(document.querySelectorAll(".mission-legend-item"));
  const colors = getAgentColors();
  const agentLayers = new Map();
  const doneAgentLayers = new Map();
  const waypointLayers = new Map();
  const doneWaypointLayers = new Map();
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
      el.style.pointerEvents = "none";
      el.style.textShadow = "0 1px 2px rgba(0,0,0,0.45)";
      labelContainer.appendChild(el);
      labelElements.set(key, el);
    }
    el.dataset.done = entry.isDone ? "1" : "0";
    el.textContent = entry.label || `WP${idx}`;
    el.style.color = color || "#e7eddc";
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
      const alpha = isDone ? doneAlpha : activeAlpha;
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

  const ensureLineAreaLayers = () => {
    if (!map.getSource(LINE_AREA_SOURCE_ID)) {
      map.addSource(LINE_AREA_SOURCE_ID, {
        type: "geojson",
        data: { type: "FeatureCollection", features: [] },
      });
    }
    if (!map.getLayer(LINE_AREA_FILL_LAYER_ID)) {
      map.addLayer({
        id: LINE_AREA_FILL_LAYER_ID,
        type: "fill",
        source: LINE_AREA_SOURCE_ID,
        paint: {
          "fill-color": ["get", "color"],
          "fill-opacity": ["get", "fillOpacity"],
        },
      }, AREA_FILL_LAYER_ID);
    }
    if (!map.getLayer(LINE_AREA_LAYER_ID)) {
      map.addLayer({
        id: LINE_AREA_LAYER_ID,
        type: "line",
        source: LINE_AREA_SOURCE_ID,
        layout: {
          "line-join": "round",
          "line-cap": "round",
        },
        paint: {
          "line-color": ["get", "color"],
          "line-width": 1.4,
          "line-opacity": ["get", "lineOpacity"],
        },
      }, AREA_FILL_LAYER_ID);
    }
  };

  const updateHitSource = () => {
    const source = map.getSource(SOURCE_ID);
    if (!source) {
      return;
    }
    const geojson = {
      type: "FeatureCollection",
      features: features.map((feature) => buildGeoFeature(feature, colors)),
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

  const updateLineAreaSource = (payloadForAreas = pendingData) => {
    const source = map.getSource(LINE_AREA_SOURCE_ID);
    if (!source) {
      return;
    }
    lineAreaFeatures = buildLineAreaFeatures(payloadForAreas, colors, selectedAgent, map);
    source.setData({
      type: "FeatureCollection",
      features: lineAreaFeatures,
    });
  };

  const update3dPositions = () => {
    if (!features.length) {
      agentLayers.forEach((layer) => layer.updatePositions([]));
      doneAgentLayers.forEach((layer) => layer.updatePositions([]));
      waypointLayers.forEach((layer) => layer.updatePositions([]));
      doneWaypointLayers.forEach((layer) => layer.updatePositions([]));
      clearWaypointLabels();
      logStatus("", { key: "mission-debug" });
      return;
    }
    const positionsByAgent = new Map();
    const donePositionsByAgent = new Map();
    const waypointPositionsByAgent = new Map();
    const doneWaypointPositionsByAgent = new Map();
    const labelEntries = [];
    AGENTS.forEach((agent) => positionsByAgent.set(agent, []));
    AGENTS.forEach((agent) => donePositionsByAgent.set(agent, []));
    AGENTS.forEach((agent) => waypointPositionsByAgent.set(agent, []));
    AGENTS.forEach((agent) => doneWaypointPositionsByAgent.set(agent, []));
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
      const target = isDone
        ? donePositionsByAgent.get(feature.agent)
        : positionsByAgent.get(feature.agent);
      if (!target) {
        return;
      }
      const segmentPositions = buildLinePositions(feature.coords, feature.alts);
      if (segmentPositions.length) {
        target.push(...segmentPositions);
      }
      const waypointTarget = isDone
        ? doneWaypointPositionsByAgent.get(feature.agent)
        : waypointPositionsByAgent.get(feature.agent);
      if (waypointTarget) {
        const waypointPositions = buildWaypointPositions(feature.coords, feature.alts);
        if (waypointPositions.length) {
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
          const pathKey =
            feature.pathId ?? feature.id ?? feature.aircraftId ?? feature.agent ?? "path";
          const wpId = Number(wpIds[idx]);
          const labelId = Number.isFinite(wpId) ? wpId : idx + 1;
          labelEntries.push({
            key: `${feature.agent}-${pathKey}-${idx + 1}`,
            agent: feature.agent,
            idx: labelId,
            label: `WP${labelId}`,
            isDone,
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
    rebuildWaypointLabels(labelEntries);
    const msg =
      totalPositions === 0
        ? "3D path positions empty."
        : `3D path vertices: ${Math.floor(totalPositions / 3)} | lines: ${totalLineCount}`;
    logStatus(msg, { key: "mission-debug", ttlMs: 4500 });
    map.triggerRepaint();
  };

  const scheduleRebuild = () => {
    if (rebuildScheduled) {
      return;
    }
    rebuildScheduled = true;
    requestAnimationFrame(() => {
      rebuildScheduled = false;
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
    updateAreaSource();
    updateLineAreaSource();
    updateWaypointLabelVisibility();
    map.triggerRepaint();
  };

  const setSelectedAgent = (agent) => {
    selectedAgent = agent || null;
    updateLegend();
    updateLayerAlpha();
  };

  const attachInteractions = () => {
    if (!map.getLayer(HIT_LAYER_ID)) {
      return;
    }
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
      setSelectedAgent(agent);
      const html = `
        <div style="font-size:12px;font-weight:700;margin-bottom:4px;">${agent || "PATH"}</div>
        <div style="font-size:11px;color:#333;">Path ${props.pathId ?? "-"}</div>
        <div style="font-size:11px;color:#333;">Status ${Number(props.isDone) === 1 ? "Done" : "Active"}</div>
        <div style="font-size:11px;color:#333;">Points ${props.points ?? "-"}</div>
        <div style="font-size:11px;color:#333;">${formatAltRange(props.altMin, props.altMax)}</div>
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
        <div style="font-size:11px;color:#333;">Path ${props.pathId ?? "-"}</div>
        <div style="font-size:11px;color:#333;">Area ${props.areaIndex ?? "-"}</div>
        <div style="font-size:11px;color:#333;">Status ${Number(props.isDone) === 1 ? "Done" : "Active"}</div>
      `;
      if (!popup) {
        popup = new maplibregl.Popup({ closeButton: true, closeOnClick: true });
      }
      popup.setLngLat(event.lngLat).setHTML(html).addTo(map);
    });
    map.on("mouseenter", LINE_AREA_FILL_LAYER_ID, () => {
      map.getCanvas().style.cursor = "pointer";
    });
    map.on("mouseleave", LINE_AREA_FILL_LAYER_ID, () => {
      map.getCanvas().style.cursor = "";
    });
    map.on("click", LINE_AREA_FILL_LAYER_ID, (event) => {
      const feature = event.features && event.features[0];
      if (!feature) {
        return;
      }
      const props = feature.properties || {};
      const agent = props.agent || "";
      setSelectedAgent(agent);
      const html = `
        <div style="font-size:12px;font-weight:700;margin-bottom:4px;">${agent || "LINE"}</div>
        <div style="font-size:11px;color:#333;">Mission ${props.missionId ?? "-"}</div>
        <div style="font-size:11px;color:#333;">Path ${props.pathId ?? "-"}</div>
        <div style="font-size:11px;color:#333;">Line ${props.lineIndex ?? "-"}</div>
        <div style="font-size:11px;color:#333;">Width ${Math.round(Number(props.widthMeters) || 0)} m</div>
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
    ensureHitLayer();
    ensureAreaLayers();
    ensureLineAreaLayers();
    ensure3dLayers();
    updateHitSource();
    updateAreaSource(payload);
    updateLineAreaSource(payload);
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
    ensureLineAreaLayers();
    ensure3dLayers();
    attachInteractions();
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
      update3dPositions();
    }
  });

  return {
    loadFromResponse,
    loadFromServer,
    setSelectedAgent,
  };
};
