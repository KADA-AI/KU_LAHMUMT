const PROJECTILE_SOURCE_ID = "projectiles";
const PROJECTILE_LINE_LAYER_ID = "projectile-trails";
const PROJECTILE_HEAD_LAYER_ID = "projectile-heads";
const PROJECTILE_TRAIL_3D_LAYER_ID = "projectile-trails-3d";
const PROJECTILE_AIM_3D_LAYER_ID = "projectile-aim-3d";
const PROJECTILE_STEM_3D_LAYER_ID = "projectile-stems-3d";
const PROJECTILE_HEAD_3D_LAYER_ID = "projectile-heads-3d";

const FRIENDLY_MISSILE_COLOR = "#5ed6ff";
const FRIENDLY_GUN_COLOR = "#ffd27a";
const ENEMY_MISSILE_COLOR = "#ff6464";
const ENEMY_GUN_COLOR = "#ff9d66";
const ALT_OFFSET_M = 10;
const GROUND_OFFSET_M = 2;

const clamp = (value, min, max) => Math.min(max, Math.max(min, value));

const metersToLonLat = (lon, lat, dx, dy) => {
  const metersPerDegLat = 111320.0;
  const cosLat = Math.cos((lat * Math.PI) / 180) || 1e-6;
  const metersPerDegLon = metersPerDegLat * cosLat;
  return {
    lon: lon + dx / metersPerDegLon,
    lat: lat + dy / metersPerDegLat,
  };
};

const distanceMeters = (lonA, latA, lonB, latB) => {
  const metersPerDegLat = 111320.0;
  const midLat = ((latA + latB) * Math.PI) / 360;
  const metersPerDegLon = metersPerDegLat * (Math.cos(midLat) || 1e-6);
  const dx = (lonB - lonA) * metersPerDegLon;
  const dy = (latB - latA) * metersPerDegLat;
  return Math.hypot(dx, dy);
};

const hexToRgb = (hex) => {
  const value = String(hex || "").replace("#", "").trim();
  if (value.length !== 6) {
    return [1, 1, 1];
  }
  const r = Number.parseInt(value.slice(0, 2), 16);
  const g = Number.parseInt(value.slice(2, 4), 16);
  const b = Number.parseInt(value.slice(4, 6), 16);
  if (!Number.isFinite(r) || !Number.isFinite(g) || !Number.isFinite(b)) {
    return [1, 1, 1];
  }
  return [r / 255, g / 255, b / 255];
};

const resolveColor = (proj) => {
  const side = String(proj?.side || "").toLowerCase();
  const kind = String(proj?.kind || "").toLowerCase();
  if (side === "enemy") {
    return kind === "missile" ? ENEMY_MISSILE_COLOR : ENEMY_GUN_COLOR;
  }
  return kind === "missile" ? FRIENDLY_MISSILE_COLOR : FRIENDLY_GUN_COLOR;
};

const getProjectionMatrix = (argsOrMatrix) => {
  if (!argsOrMatrix) {
    return null;
  }
  if (Array.isArray(argsOrMatrix) || argsOrMatrix instanceof Float32Array) {
    return argsOrMatrix;
  }
  if (argsOrMatrix.defaultProjectionData?.mainMatrix) {
    return argsOrMatrix.defaultProjectionData.mainMatrix;
  }
  if (argsOrMatrix.matrix) {
    return argsOrMatrix.matrix;
  }
  return null;
};

const compileProgram = (gl, id, vertexSource, fragmentSource) => {
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
  const vertexShader = compile(gl.VERTEX_SHADER, vertexSource);
  const fragmentShader = compile(gl.FRAGMENT_SHADER, fragmentSource);
  const program = gl.createProgram();
  gl.attachShader(program, vertexShader);
  gl.attachShader(program, fragmentShader);
  gl.linkProgram(program);
  gl.deleteShader(vertexShader);
  gl.deleteShader(fragmentShader);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
    const log = gl.getProgramInfoLog(program) || "program link failed";
    gl.deleteProgram(program);
    throw new Error(`[${id}] ${log}`);
  }
  return program;
};

const createColoredLineLayer3d = (id, lineWidth = 1.6) => ({
  id,
  type: "custom",
  renderingMode: "3d",
  _visible: true,
  _lineWidth: lineWidth,
  _lineCount: 0,
  _pendingPositions: null,
  updatePositions(positions) {
    this._pendingPositions = positions;
    if (!this._gl || !this._lineBuffer) {
      return;
    }
    const gl = this._gl;
    const data = new Float32Array(positions);
    gl.bindBuffer(gl.ARRAY_BUFFER, this._lineBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, data, gl.DYNAMIC_DRAW);
    this._lineCount = data.length / 7;
    this._pendingPositions = null;
  },
  onAdd(_map, gl) {
    this._gl = gl;
    const isWebGL2 =
      typeof WebGL2RenderingContext !== "undefined" && gl instanceof WebGL2RenderingContext;
    const vertexSource = isWebGL2
      ? `#version 300 es
        in vec3 a_pos;
        in vec3 a_color;
        in float a_alpha;
        uniform mat4 u_matrix;
        out vec4 v_color;
        void main() {
          gl_Position = u_matrix * vec4(a_pos, 1.0);
          v_color = vec4(a_color, a_alpha);
        }`
      : `
        attribute vec3 a_pos;
        attribute vec3 a_color;
        attribute float a_alpha;
        uniform mat4 u_matrix;
        varying vec4 v_color;
        void main() {
          gl_Position = u_matrix * vec4(a_pos, 1.0);
          v_color = vec4(a_color, a_alpha);
        }`;
    const fragmentSource = isWebGL2
      ? `#version 300 es
        precision mediump float;
        in vec4 v_color;
        out vec4 fragColor;
        void main() {
          fragColor = v_color;
        }`
      : `
        precision mediump float;
        varying vec4 v_color;
        void main() {
          gl_FragColor = v_color;
        }`;
    try {
      this._program = compileProgram(gl, id, vertexSource, fragmentSource);
    } catch (err) {
      console.error(`[${id}] shader/program error:`, err);
      this._program = null;
      this._visible = false;
      return;
    }
    const program = this._program;
    this._aPos = gl.getAttribLocation(program, "a_pos");
    this._aColor = gl.getAttribLocation(program, "a_color");
    this._aAlpha = gl.getAttribLocation(program, "a_alpha");
    this._uMatrix = gl.getUniformLocation(program, "u_matrix");
    this._lineBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, this._lineBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([]), gl.DYNAMIC_DRAW);
    if (this._pendingPositions) {
      this.updatePositions(this._pendingPositions);
    }
  },
  render(gl, argsOrMatrix) {
    if (!this._program || !this._visible || !this._lineCount) {
      return;
    }
    const matrix = getProjectionMatrix(argsOrMatrix);
    if (!matrix || matrix.length !== 16) {
      return;
    }
    const depthWasEnabled = gl.isEnabled(gl.DEPTH_TEST);
    const cullWasEnabled = gl.isEnabled(gl.CULL_FACE);
    gl.disable(gl.DEPTH_TEST);
    if (cullWasEnabled) {
      gl.disable(gl.CULL_FACE);
    }
    gl.useProgram(this._program);
    gl.uniformMatrix4fv(this._uMatrix, false, matrix);
    gl.bindBuffer(gl.ARRAY_BUFFER, this._lineBuffer);
    const stride = 7 * 4;
    gl.enableVertexAttribArray(this._aPos);
    gl.vertexAttribPointer(this._aPos, 3, gl.FLOAT, false, stride, 0);
    gl.enableVertexAttribArray(this._aColor);
    gl.vertexAttribPointer(this._aColor, 3, gl.FLOAT, false, stride, 3 * 4);
    gl.enableVertexAttribArray(this._aAlpha);
    gl.vertexAttribPointer(this._aAlpha, 1, gl.FLOAT, false, stride, 6 * 4);
    gl.enable(gl.BLEND);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    gl.lineWidth(this._lineWidth);
    gl.drawArrays(gl.LINES, 0, this._lineCount);
    if (cullWasEnabled) {
      gl.enable(gl.CULL_FACE);
    }
    if (depthWasEnabled) {
      gl.enable(gl.DEPTH_TEST);
    }
  },
});

const createPointLayer3d = (id) => ({
  id,
  type: "custom",
  renderingMode: "3d",
  _visible: true,
  _pointCount: 0,
  _pendingPoints: null,
  updatePoints(points) {
    this._pendingPoints = points;
    if (!this._gl || !this._pointBuffer) {
      return;
    }
    const gl = this._gl;
    const data = new Float32Array(points);
    gl.bindBuffer(gl.ARRAY_BUFFER, this._pointBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, data, gl.DYNAMIC_DRAW);
    this._pointCount = data.length / 8;
    this._pendingPoints = null;
  },
  onAdd(_map, gl) {
    this._gl = gl;
    const isWebGL2 =
      typeof WebGL2RenderingContext !== "undefined" && gl instanceof WebGL2RenderingContext;
    const vertexSource = isWebGL2
      ? `#version 300 es
        in vec3 a_pos;
        in vec3 a_color;
        in float a_size;
        in float a_alpha;
        uniform mat4 u_matrix;
        out vec4 v_color;
        void main() {
          gl_Position = u_matrix * vec4(a_pos, 1.0);
          gl_PointSize = a_size;
          v_color = vec4(a_color, a_alpha);
        }`
      : `
        attribute vec3 a_pos;
        attribute vec3 a_color;
        attribute float a_size;
        attribute float a_alpha;
        uniform mat4 u_matrix;
        varying vec4 v_color;
        void main() {
          gl_Position = u_matrix * vec4(a_pos, 1.0);
          gl_PointSize = a_size;
          v_color = vec4(a_color, a_alpha);
        }`;
    const fragmentSource = isWebGL2
      ? `#version 300 es
        precision mediump float;
        in vec4 v_color;
        out vec4 fragColor;
        void main() {
          vec2 uv = gl_PointCoord - 0.5;
          float radius = length(uv);
          if (radius > 0.5) {
            discard;
          }
          float glow = 1.0 - smoothstep(0.1, 0.5, radius);
          fragColor = vec4(v_color.rgb * (0.78 + 0.35 * glow), v_color.a * glow);
        }`
      : `
        precision mediump float;
        varying vec4 v_color;
        void main() {
          vec2 uv = gl_PointCoord - 0.5;
          float radius = length(uv);
          if (radius > 0.5) {
            discard;
          }
          float glow = 1.0 - smoothstep(0.1, 0.5, radius);
          gl_FragColor = vec4(v_color.rgb * (0.78 + 0.35 * glow), v_color.a * glow);
        }`;
    try {
      this._program = compileProgram(gl, id, vertexSource, fragmentSource);
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
    this._aAlpha = gl.getAttribLocation(program, "a_alpha");
    this._uMatrix = gl.getUniformLocation(program, "u_matrix");
    this._pointBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, this._pointBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([]), gl.DYNAMIC_DRAW);
    if (this._pendingPoints) {
      this.updatePoints(this._pendingPoints);
    }
  },
  render(gl, argsOrMatrix) {
    if (!this._program || !this._visible || !this._pointCount) {
      return;
    }
    const matrix = getProjectionMatrix(argsOrMatrix);
    if (!matrix || matrix.length !== 16) {
      return;
    }
    const depthWasEnabled = gl.isEnabled(gl.DEPTH_TEST);
    const cullWasEnabled = gl.isEnabled(gl.CULL_FACE);
    gl.disable(gl.DEPTH_TEST);
    if (cullWasEnabled) {
      gl.disable(gl.CULL_FACE);
    }
    gl.useProgram(this._program);
    gl.uniformMatrix4fv(this._uMatrix, false, matrix);
    gl.bindBuffer(gl.ARRAY_BUFFER, this._pointBuffer);
    const stride = 8 * 4;
    gl.enableVertexAttribArray(this._aPos);
    gl.vertexAttribPointer(this._aPos, 3, gl.FLOAT, false, stride, 0);
    gl.enableVertexAttribArray(this._aColor);
    gl.vertexAttribPointer(this._aColor, 3, gl.FLOAT, false, stride, 3 * 4);
    gl.enableVertexAttribArray(this._aSize);
    gl.vertexAttribPointer(this._aSize, 1, gl.FLOAT, false, stride, 6 * 4);
    gl.enableVertexAttribArray(this._aAlpha);
    gl.vertexAttribPointer(this._aAlpha, 1, gl.FLOAT, false, stride, 7 * 4);
    gl.enable(gl.BLEND);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    gl.drawArrays(gl.POINTS, 0, this._pointCount);
    if (cullWasEnabled) {
      gl.enable(gl.CULL_FACE);
    }
    if (depthWasEnabled) {
      gl.enable(gl.DEPTH_TEST);
    }
  },
});

const pushVertex = (target, merc, rgb, alpha) => {
  target.push(merc.x, merc.y, merc.z, rgb[0], rgb[1], rgb[2], alpha);
};

const pushSegment = (target, from, to, rgb, alpha) => {
  pushVertex(target, from, rgb, alpha);
  pushVertex(target, to, rgb, alpha);
};

const terrainElevation = (map, lon, lat) => {
  if (typeof map.queryTerrainElevation !== "function") {
    return 0;
  }
  const elev = map.queryTerrainElevation({ lng: lon, lat }, { exaggerated: false });
  return Number.isFinite(elev) ? elev : 0;
};

const toMercator = (lon, lat, alt) =>
  maplibregl.MercatorCoordinate.fromLngLat([lon, lat], Number.isFinite(alt) ? alt : 0);

const buildVisualData = (map, projectiles) => {
  const shadowFeatures = [];
  const trail3d = [];
  const aim3d = [];
  const stem3d = [];
  const heads3d = [];
  const dpr = typeof map.getPixelRatio === "function" ? map.getPixelRatio() : 1;

  if (!Array.isArray(projectiles)) {
    return { shadowFeatures, trail3d, aim3d, stem3d, heads3d };
  }

  projectiles.forEach((proj) => {
    const lat = Number(proj?.lat);
    const lon = Number(proj?.lon);
    if (!Number.isFinite(lat) || !Number.isFinite(lon)) {
      return;
    }
    const alt = Number(proj?.alt);
    const vx = Number(proj?.vx);
    const vy = Number(proj?.vy);
    const vz = Number(proj?.vz);
    const speed = Number.isFinite(Number(proj?.speed))
      ? Number(proj.speed)
      : Math.hypot(vx || 0, vy || 0, vz || 0);
    const side = String(proj?.side || "").toLowerCase();
    const kind = String(proj?.kind || "").toLowerCase();
    const isMissile = kind === "missile";
    const color = resolveColor(proj);
    const rgb = hexToRgb(color);
    const hspeed = Math.hypot(vx || 0, vy || 0);
    const targetLat = Number(proj?.targetLat);
    const targetLon = Number(proj?.targetLon);
    const targetAlt = Number(proj?.targetAlt);
    const hasTarget = Number.isFinite(targetLat) && Number.isFinite(targetLon);
    const remainingMeters = hasTarget ? distanceMeters(lon, lat, targetLon, targetLat) : Infinity;
    const tailLenBase = isMissile ? clamp(speed * 0.24, 90, 280) : clamp(speed * 0.09, 28, 85);
    const tailLen = Math.min(tailLenBase, Math.max(18, remainingMeters * 0.55));
    const headSize = (isMissile ? clamp(5.5 + speed / 260, 6.0, 10.5) : clamp(3.8 + speed / 520, 3.8, 6.2)) * dpr;
    const groundElevationM = terrainElevation(map, lon, lat);
    const groundAltitude = groundElevationM + GROUND_OFFSET_M;
    const altitude = Number.isFinite(alt) ? alt + ALT_OFFSET_M : groundElevationM + ALT_OFFSET_M;
    const headMerc = toMercator(lon, lat, altitude);
    const groundMerc = toMercator(lon, lat, groundAltitude);
    const trailAlpha = side === "enemy" ? 0.88 : 0.92;
    const aimAlpha = isMissile ? 0.18 : 0.13;

    heads3d.push(headMerc.x, headMerc.y, headMerc.z, rgb[0], rgb[1], rgb[2], headSize, 0.98);
    if (altitude - groundAltitude > 35) {
      pushSegment(stem3d, groundMerc, headMerc, rgb, 0.18);
    }

    shadowFeatures.push({
      type: "Feature",
      properties: {
        color,
        size: clamp(headSize / dpr * 0.8, 2.2, 6.5),
        opacity: isMissile ? 0.32 : 0.24,
        kind,
      },
      geometry: {
        type: "Point",
        coordinates: [lon, lat],
      },
    });

    if (hspeed > 1e-3 && Number.isFinite(speed) && speed > 1) {
      const tail = metersToLonLat(lon, lat, (-vx / hspeed) * tailLen, (-vy / hspeed) * tailLen);
      const tailAlt = altitude - ((Number.isFinite(vz) ? vz : 0) / Math.max(1, speed)) * tailLen;
      const tailMerc = toMercator(tail.lon, tail.lat, tailAlt);
      pushSegment(trail3d, tailMerc, headMerc, rgb, trailAlpha);
      shadowFeatures.push({
        type: "Feature",
        properties: {
          color,
          width: isMissile ? 2.1 : 1.4,
          opacity: isMissile ? 0.36 : 0.24,
        },
        geometry: {
          type: "LineString",
          coordinates: [
            [tail.lon, tail.lat],
            [lon, lat],
          ],
        },
      });
    }

    if (hasTarget && remainingMeters > 8) {
      const targetGround = terrainElevation(map, targetLon, targetLat);
      const targetAltitude = Number.isFinite(targetAlt)
        ? targetAlt + GROUND_OFFSET_M
        : targetGround + GROUND_OFFSET_M;
      const targetMerc = toMercator(targetLon, targetLat, targetAltitude);
      pushSegment(aim3d, headMerc, targetMerc, rgb, aimAlpha);
    }
  });

  return { shadowFeatures, trail3d, aim3d, stem3d, heads3d };
};

export const initProjectileMarkers = (map) => {
  let pending = null;
  let mapReady = typeof map.isStyleLoaded === "function" ? map.isStyleLoaded() : map.loaded();
  let lastProjectilePayloadSignature = null;
  const trailLayer3d = createColoredLineLayer3d(PROJECTILE_TRAIL_3D_LAYER_ID, 2.0);
  const aimLayer3d = createColoredLineLayer3d(PROJECTILE_AIM_3D_LAYER_ID, 1.0);
  const stemLayer3d = createColoredLineLayer3d(PROJECTILE_STEM_3D_LAYER_ID, 1.0);
  const headLayer3d = createPointLayer3d(PROJECTILE_HEAD_3D_LAYER_ID);

  const ensureLayer = () => {
    if (!map.getSource(PROJECTILE_SOURCE_ID)) {
      map.addSource(PROJECTILE_SOURCE_ID, {
        type: "geojson",
        data: { type: "FeatureCollection", features: [] },
      });
    }
    if (!map.getLayer(PROJECTILE_LINE_LAYER_ID)) {
      map.addLayer({
        id: PROJECTILE_LINE_LAYER_ID,
        type: "line",
        source: PROJECTILE_SOURCE_ID,
        filter: ["==", ["geometry-type"], "LineString"],
        layout: { "line-cap": "round", "line-join": "round" },
        paint: {
          "line-color": ["get", "color"],
          "line-width": ["get", "width"],
          "line-opacity": ["get", "opacity"],
        },
      });
    }
    if (!map.getLayer(PROJECTILE_HEAD_LAYER_ID)) {
      map.addLayer({
        id: PROJECTILE_HEAD_LAYER_ID,
        type: "circle",
        source: PROJECTILE_SOURCE_ID,
        filter: ["==", ["geometry-type"], "Point"],
        paint: {
          "circle-color": ["get", "color"],
          "circle-radius": ["get", "size"],
          "circle-opacity": ["get", "opacity"],
          "circle-blur": 0.55,
          "circle-stroke-color": "rgba(20,20,20,0.35)",
          "circle-stroke-width": 0.8,
        },
      });
    }
    if (!map.getLayer(PROJECTILE_STEM_3D_LAYER_ID)) {
      map.addLayer(stemLayer3d);
    }
    if (!map.getLayer(PROJECTILE_AIM_3D_LAYER_ID)) {
      map.addLayer(aimLayer3d);
    }
    if (!map.getLayer(PROJECTILE_TRAIL_3D_LAYER_ID)) {
      map.addLayer(trailLayer3d);
    }
    if (!map.getLayer(PROJECTILE_HEAD_3D_LAYER_ID)) {
      map.addLayer(headLayer3d);
    }
  };

  const apply = (payload) => {
    ensureLayer();
    const source = map.getSource(PROJECTILE_SOURCE_ID);
    if (!source) {
      return;
    }
    const visualData = buildVisualData(map, payload?.projectiles || []);
    source.setData({ type: "FeatureCollection", features: visualData.shadowFeatures });
    stemLayer3d.updatePositions(visualData.stem3d);
    aimLayer3d.updatePositions(visualData.aim3d);
    trailLayer3d.updatePositions(visualData.trail3d);
    headLayer3d.updatePoints(visualData.heads3d);
    if (typeof map.triggerRepaint === "function") {
      map.triggerRepaint();
    }
  };

  const loadFromReference = (payload) => {
    pending = payload;
    const projectiles = Array.isArray(payload?.projectiles) ? payload.projectiles : [];
    let payloadSignature = null;
    try {
      payloadSignature = JSON.stringify(projectiles);
    } catch (_err) {
      payloadSignature = null;
    }
    if (payloadSignature !== null && payloadSignature === lastProjectilePayloadSignature) {
      return;
    }
    lastProjectilePayloadSignature = payloadSignature;
    if (mapReady) {
      apply(payload);
    }
  };

  map.on("load", () => {
    mapReady = true;
    ensureLayer();
    if (pending) {
      apply(pending);
    }
  });

  map.on("terrain", () => {
    if (pending) {
      apply(pending);
    }
  });

  return { loadFromReference };
};
