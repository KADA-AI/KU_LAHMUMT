import { logStatus } from "./status_log.js";

const SOURCE_ID = "mission-paths";
const HIT_LAYER_ID = "mission-paths-hit";

const AGENTS = ["LAH1", "LAH2", "LAH3", "UAV1", "UAV2", "UAV3"];

const DEFAULT_ALPHA = 1.0;
const SELECT_ALPHA = 1.0;
const DIM_ALPHA = 0.15;

const PATH_WIDTH_PX = 2.5;
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

const buildGeoFeature = (feature, colors) => ({
  type: "Feature",
  id: feature.id,
  geometry: {
    type: "LineString",
    coordinates: feature.coords,
  },
  properties: {
    agent: feature.agent,
    pathId: feature.pathId,
    aircraftId: feature.aircraftId,
    points: feature.points,
    altMin: feature.altMin,
    altMax: feature.altMax,
    color: colors[feature.agent] || "#e7eddc",
  },
});

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

export const initMissionPaths = (map) => {
  let pendingData = null;
  let selectedAgent = null;
  let features = [];
  let agentCounts = {};
  let mapReady = typeof map.isStyleLoaded === "function" ? map.isStyleLoaded() : map.loaded();
  let popup = null;
  let rebuildScheduled = false;
  let didFitBounds = false;

  const legend = document.getElementById("mission-legend");
  const legendItems = Array.from(document.querySelectorAll(".mission-legend-item"));
  const colors = getAgentColors();
  const agentLayers = new Map();

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
      const layerId = `mission-paths-3d-${agent.toLowerCase()}`;
      if (map.getLayer(layerId)) {
        return;
      }
      const layer = createLineLayer3d(
        layerId,
        colors[agent] || "#e7eddc",
        "TRIANGLES",
        PATH_WIDTH_PX,
      );
      map.addLayer(layer);
      agentLayers.set(agent, layer);
    });
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

  const update3dPositions = () => {
    if (!features.length) {
      agentLayers.forEach((layer) => layer.updatePositions([]));
      logStatus("", { key: "mission-debug" });
      return;
    }
    const positionsByAgent = new Map();
    AGENTS.forEach((agent) => positionsByAgent.set(agent, []));
    features.forEach((feature) => {
      const target = positionsByAgent.get(feature.agent);
      if (!target) {
        return;
      }
      const segmentPositions = buildLinePositions(feature.coords, feature.alts);
      if (segmentPositions.length) {
        target.push(...segmentPositions);
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
        <div style="font-size:11px;color:#333;">Points ${props.points ?? "-"}</div>
        <div style="font-size:11px;color:#333;">${formatAltRange(props.altMin, props.altMax)}</div>
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
    ensure3dLayers();
    updateHitSource();
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
