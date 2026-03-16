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
const LAH_POINT_SCALE = 1.15;
const UAV_POINT_SCALE = 0.6;
const FOOTPRINT_STEPS = 4;
const FOOTPRINT_MIN_RADIUS_M = 15;
const FOOTPRINT_MAX_RADIUS_M = 5000;
const FOOTPRINT_ASPECT = 16 / 9;
const FOOTPRINT_CURRENT_OPACITY = 0.2;
const FOOTPRINT_TRAIL_MAX = 600;
const FOOTPRINT_TRAIL_OPACITY = 0.12;
const FOOTPRINT_TRAIL_SAMPLE_STEP = 1;
const TRAIL_MAX_METERS = 2000;
const TRAIL_MIN_SEGMENT_M = 4;
const TRAIL_WIDTH = 2.0;
const TRAIL_Z_OFFSET_M = 0.8;
const EXCEEDED_SEP_COLOR = "#ef4444";
const VISUAL_UPDATE_HZ = 15;
const VISUAL_UPDATE_INTERVAL_MS = 1000 / VISUAL_UPDATE_HZ;
const FOOTPRINT_VISUAL_SAMPLE_INTERVAL_MS = VISUAL_UPDATE_INTERVAL_MS;

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
  ring.push([...ring[0]]);
  return ring;
};

const buildFootprintGeometry = (entry) => {
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
    { sx: -1, sy: -1 },
    { sx: 1, sy: -1 },
    { sx: 1, sy: 1 },
    { sx: -1, sy: 1 },
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
    const hit = {
      x: origin.x + dir.x * t,
      y: origin.y + dir.y * t,
      z: 0,
    };
    corners.push(hit);
  });
  if (corners.length !== 4) {
    return null;
  }
  const cx = corners.reduce((sum, c) => sum + c.x, 0) / corners.length;
  const cy = corners.reduce((sum, c) => sum + c.y, 0) / corners.length;
  corners.sort((a, b) => Math.atan2(a.y - cy, a.x - cx) - Math.atan2(b.y - cy, b.x - cx));

  const coords = [];
  corners.forEach((corner) => {
    const pt = metersToLonLat(target.lon, target.lat, corner.x, corner.y);
    coords.push([pt.lon, pt.lat]);
  });
  coords.push(coords[0]);
  return coords;
};

const buildDashedSegments = (start, end, dashLenM, gapLenM) => {
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

const createSphereLayer = (id) => {
  const layer = {
    id,
    type: "custom",
    renderingMode: "3d",
    _visible: true,
    _useDepth: false,
    _pointCount: 0,
    _pendingPoints: null,
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
      gl.bindBuffer(gl.ARRAY_BUFFER, this._buffer);
      gl.bufferData(gl.ARRAY_BUFFER, data, gl.DYNAMIC_DRAW);
      this._pointCount = data.length / 7;
      this._pendingPoints = null;
    },
    onAdd(_map, gl) {
      this._gl = gl;
      const isWebGL2 =
        typeof WebGL2RenderingContext !== "undefined" &&
        gl instanceof WebGL2RenderingContext;
      const vertexSource = isWebGL2
        ? `#version 300 es
          in vec3 a_pos;
          in vec3 a_color;
          in float a_size;
          uniform mat4 u_matrix;
          out vec3 v_color;
          void main() {
            gl_Position = u_matrix * vec4(a_pos, 1.0);
            gl_PointSize = a_size;
            v_color = a_color;
          }`
        : `
          attribute vec3 a_pos;
          attribute vec3 a_color;
          attribute float a_size;
          uniform mat4 u_matrix;
          varying vec3 v_color;
          void main() {
            gl_Position = u_matrix * vec4(a_pos, 1.0);
            gl_PointSize = a_size;
            v_color = a_color;
          }`;
      const fragmentSource = isWebGL2
        ? `#version 300 es
          precision mediump float;
          in vec3 v_color;
          out vec4 fragColor;
          void main() {
            vec2 uv = gl_PointCoord - 0.5;
            float r = length(uv);
            if (r > 0.5) {
              discard;
            }
            float z = sqrt(max(0.0, 0.25 - r * r)) / 0.5;
            vec3 normal = normalize(vec3(uv / 0.5, z));
            vec3 lightDir = normalize(vec3(-0.3, -0.2, 0.93));
            float diffuse = clamp(dot(normal, lightDir), 0.0, 1.0);
            float rim = smoothstep(0.3, 0.5, r);
            vec3 color = v_color * (0.55 + 0.45 * diffuse) + rim * 0.2;
            fragColor = vec4(color, 0.98);
          }`
        : `
          precision mediump float;
          varying vec3 v_color;
          void main() {
            vec2 uv = gl_PointCoord - 0.5;
            float r = length(uv);
            if (r > 0.5) {
              discard;
            }
            float z = sqrt(max(0.0, 0.25 - r * r)) / 0.5;
            vec3 normal = normalize(vec3(uv / 0.5, z));
            vec3 lightDir = normalize(vec3(-0.3, -0.2, 0.93));
            float diffuse = clamp(dot(normal, lightDir), 0.0, 1.0);
            float rim = smoothstep(0.3, 0.5, r);
            vec3 color = v_color * (0.55 + 0.45 * diffuse) + rim * 0.2;
            gl_FragColor = vec4(color, 0.98);
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
      const stride = 7 * 4;
      gl.enableVertexAttribArray(this._aPos);
      gl.vertexAttribPointer(this._aPos, 3, gl.FLOAT, false, stride, 0);
      gl.enableVertexAttribArray(this._aColor);
      gl.vertexAttribPointer(this._aColor, 3, gl.FLOAT, false, stride, 3 * 4);
      gl.enableVertexAttribArray(this._aSize);
      gl.vertexAttribPointer(this._aSize, 1, gl.FLOAT, false, stride, 6 * 4);
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

const createLineLayer = (id, color) => {
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
  let lastFootprintSampleMs = 0;
  let lastStep = null;
  let lastProjectionMatrix = null;

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

    });
  };

  const animateVisuals = (now) => {
    animationFrameId = requestAnimationFrame(animateVisuals);
    if (!mapReady) {
      return;
    }
    if ((now - lastVisualUpdateMs) < VISUAL_UPDATE_INTERVAL_MS) {
      return;
    }
    lastVisualUpdateMs = now;
    if (!sampleToPositions) {
      if (currentPositions && Object.keys(currentPositions).length) {
        recordFootprintHistory(currentPositions, now, false);
        updateLayers();
      }
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
    }
  };

  const buildBuffers = (colors) => {
    const zoom = typeof map.getZoom === "function" ? map.getZoom() : 12;
    const pixelRatio = window.devicePixelRatio || 1;
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
      const size =
        BASE_SIZE *
        pixelRatio *
        scale *
        (LAH_IDS.has(agent) ? LAH_POINT_SCALE : UAV_POINT_SCALE);
      points.push(coord.x, coord.y, coord.z, r, g, b, size);

      const filming = entry.filmingTarget;
      if (
        filming &&
        Number.isFinite(filming.lat) &&
        Number.isFinite(filming.lon)
      ) {
        const tAlt = Number.isFinite(filming.alt) ? filming.alt : 0;
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
    if (footprintSource) {
      footprintSource.setData({
        type: "FeatureCollection",
        features: [...footprintFillFeatures, ...footprintLineFeatures],
      });
    }
    if (footprintTrailSource) {
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
      footprintTrailSource.setData({
        type: "FeatureCollection",
        features: trailFeatures,
      });
    }
    map.triggerRepaint();
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
      el.style.whiteSpace = "nowrap";
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
    const dpr = window.devicePixelRatio || 1;
    const width = canvas.width / dpr;
    const height = canvas.height / dpr;
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
      el.textContent = Number.isFinite(sepMeters)
        ? `${agent} | ${Math.round(sepMeters)}m`
        : agent;
      el.style.color = isSepExceeded ? EXCEEDED_SEP_COLOR : (colors[agent] || "#e7eddc");
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
    const dpr = window.devicePixelRatio || 1;
    const width = canvas.width / dpr;
    const height = canvas.height / dpr;
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
    const authoritativePositions = ok && payload.vehicles ? clonePositions(payload.vehicles) : {};
    const now = (typeof performance !== "undefined" && typeof performance.now === "function")
      ? performance.now()
      : Date.now();
    if (!authoritativePositions || !Object.keys(authoritativePositions).length) {
      sampleFromPositions = null;
      sampleToPositions = null;
      currentPositions = {};
      footprintHistory.clear();
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

  map.on("resize", () => {
    updateLayers();
  });

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

  return { loadFromReference, getPosition, getPositions };
};
