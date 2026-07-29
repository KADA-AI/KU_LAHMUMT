(function () {
  "use strict";

  const STATE_INTERVAL_MS = 400;
  const MISSION_INTERVAL_MS = 2000;
  const TRACK_LIMIT = 480;
  const FOOTPRINT_LIMIT = 36;
  const FOOTPRINT_RECENT_STRONG_COUNT = 5;
  const TREND_LIMIT = 80;
  const FOOTPRINT_SAMPLE_MS = 1200;
  const COMMAND_LABEL_LIMIT = 28;
  const DISCOVERY_LABEL_LIMIT = 36;
  const DISCOVERY_FEED_LIMIT = 80;
  const VEHICLE_COLORS = ["#37b7a1", "#e2a43b", "#de6c58", "#5b91ba", "#78a85b", "#c4835d"];
  const EMPTY_FC = Object.freeze({ type: "FeatureCollection", features: [] });

  const app = {
    map: null,
    mapLoaded: false,
    mapConfigured: false,
    mapInteracted: false,
    config: null,
    coverageSettings: null,
    state: null,
    mission: null,
    missionGeometry: null,
    missionParts: new Map(),
    missionLabels: new Map(),
    commandLabels: new Map(),
    commandLabelRenderKey: "",
    discoveryLabels: new Map(),
    discoveryLabelRenderKey: "",
    missionSignature: "",
    planID: "-",
    followID: "",
    vehicles: new Map(),
    markers: new Map(),
    tracks: new Map(),
    footprints: new Map(),
    discoveries: [],
    discoveryFeedRenderKey: null,
    uavCommands: [],
    lastFootprintSample: new Map(),
    lastTrendTimestamp: null,
    trends: {
      coverage: [],
      line: [],
      area: [],
      gsd: [],
      overlap: [],
      quality: [],
    },
    stateFailures: 0,
    stateReceivedAt: 0,
    stateGeneratedAt: null,
    hoverPopup: null,
    coordinatePopup: null,
    markerHoverActive: false,
    layerVisibility: {
      areas: true,
      areaProgress: true,
      optionAssignments: false,
      corridors: true,
      paths: true,
      tracks: true,
      footprints: true,
      footprintTrails: true,
      detectionFootprints: true,
      uavCommands: true,
      targets: true,
    },
    missionViewMode: "current",
    selectedInputMissionID: null,
    currentInputMissionIDs: [],
    showMissionHistory: false,
    missionRows: [],
    optionAssignmentSignature: "",
    optionAssignmentRenderKey: "",
    selectedOptionPlanID: null,
    optionAssignmentGeojson: EMPTY_FC,
    stopped: false,
    toastTimer: null,
  };

  const els = {};

  const SOURCE_IDS = {
    areas: "mission-areas-source",
    optionAssignments: "option-assignments-source",
    remainingAreas: "remaining-areas-source",
    coverageDepth: "coverage-depth-source",
    coveragePassAttribution: "coverage-pass-attribution-source",
    corridors: "mission-corridors-source",
    paths: "mission-paths-source",
    tracks: "vehicle-tracks-source",
    footprints: "current-footprints-source",
    footprintTrails: "footprint-trails-source",
    detectionFootprints: "detection-footprints-source",
    uavCommands: "uav-commands-source",
    targets: "targets-source",
  };

  const LAYER_GROUPS = {
    areas: [
      "mission-areas-fill",
      "mission-areas-line",
    ],
    areaProgress: [
      "remaining-areas-fill",
      "remaining-areas-line",
      "coverage-depth-fill",
      "coverage-depth-line",
      "coverage-pass-forward-line",
      "coverage-pass-reverse-line",
    ],
    optionAssignments: [
      "option-assignments-fill",
      "option-assignments-corridor-line",
      "option-assignments-centerline",
    ],
    corridors: ["mission-corridors-fill", "mission-corridors-line", "mission-input-lines"],
    paths: ["mission-paths-casing", "mission-paths-line"],
    tracks: ["vehicle-tracks-casing", "vehicle-tracks-line", "vehicle-position-arrow"],
    footprints: ["current-footprints-fill", "current-footprints-line"],
    footprintTrails: ["footprint-trails-fill", "footprint-trails-line"],
    detectionFootprints: [
      "detection-footprints-fill",
      "detection-footprints-line",
      "detection-points-halo",
      "detection-points",
    ],
    uavCommands: ["uav-command-points"],
    targets: ["targets-fill", "targets-line", "targets-halo", "targets-point"],
  };

  const INTERACTIVE_MISSION_LAYERS = [
    "coverage-depth-fill",
    "remaining-areas-fill",
    "mission-areas-fill",
    "mission-corridors-fill",
    "mission-input-lines",
  ];

  const cacheElements = () => {
    [
      "scenario-name",
      "connection-state",
      "connection-label",
      "top-plan-id",
      "top-signal",
      "top-updated",
      "clock",
      "shutdown-button",
      "follow-segments",
      "map-message",
      "map-coordinate",
      "map-coordinate-cursor",
      "history-status",
      "interpolation-hz",
      "save-interpolation",
      "interpolation-status",
      "data-freshness",
      "kpi-grid",
      "vehicle-count",
      "vehicle-list",
      "option-assignment-count",
      "option-assignment-tabs",
      "option-assignment-list",
      "mission-part-count",
      "mission-parts-body",
      "mission-focus-status",
      "mission-focus-hint",
      "mission-view-segments",
      "show-mission-history",
      "coverage-total",
      "coverage-rows",
      "coverage-chart",
      "quality-status",
      "quality-trends",
      "command-count",
      "command-feed",
      "discovery-count",
      "discovery-feed",
      "event-count",
      "event-feed",
      "toast",
    ].forEach((id) => {
      els[toCamel(id)] = document.getElementById(id);
    });
  };

  const toCamel = (value) => value.replace(/-([a-z])/g, (_, char) => char.toUpperCase());

  const escapeHtml = (value) =>
    String(value ?? "")
      .replaceAll("&", "&amp;")
      .replaceAll("<", "&lt;")
      .replaceAll(">", "&gt;")
      .replaceAll('"', "&quot;")
      .replaceAll("'", "&#039;");

  const firstDefined = (...values) => values.find((value) => value !== undefined && value !== null && value !== "");

  const numberOrNull = (value) => {
    if (value === null || value === undefined || value === "") return null;
    if (typeof value === "string") {
      value = value.replace(/[% ,]/g, "");
    }
    const number = Number(value);
    return Number.isFinite(number) ? number : null;
  };

  const clamp = (value, min, max) => Math.min(max, Math.max(min, value));

  const asPercent = (value) => {
    const number = numberOrNull(value);
    if (number === null) return null;
    return clamp(number, 0, 100);
  };

  const boolValue = (value) => {
    if (typeof value === "boolean") return value;
    if (typeof value === "number") return value !== 0;
    if (typeof value === "string") {
      const normalized = value.trim().toLowerCase();
      if (["true", "1", "yes", "y", "on", "normal", "ok", "pass", "satisfied", "충족", "정상", "비행", "촬영"].includes(normalized)) return true;
      if (["false", "0", "no", "n", "off", "fail", "unsatisfied", "미충족", "비정상", "정지"].includes(normalized)) return false;
    }
    return null;
  };

  const toArray = (value) => (Array.isArray(value) ? value : []);

  const formatNumber = (value, digits = 0, fallback = "-") => {
    const number = numberOrNull(value);
    if (number === null) return fallback;
    return number.toLocaleString("ko-KR", {
      minimumFractionDigits: digits,
      maximumFractionDigits: digits,
    });
  };

  const formatPercent = (value, digits = 0) => {
    const percent = asPercent(value);
    return percent === null ? "-" : `${formatNumber(percent, digits)}%`;
  };

  const formatTime = (value, withDate = false) => {
    if (value === null || value === undefined || value === "") return "--:--:--";
    let date;
    if (value instanceof Date) {
      date = value;
    } else if (typeof value === "number" && value < 10_000_000_000) {
      date = new Date(value * 1000);
    } else {
      date = new Date(value);
    }
    if (Number.isNaN(date.getTime())) return String(value);
    const options = withDate
      ? { month: "2-digit", day: "2-digit", hour: "2-digit", minute: "2-digit", second: "2-digit", hour12: false }
      : { hour: "2-digit", minute: "2-digit", second: "2-digit", hour12: false };
    return new Intl.DateTimeFormat("ko-KR", options).format(date);
  };

  const formatPreciseKstTime = (value) => {
    if (value === null || value === undefined || value === "") return "--:--:--.---";
    const date = value instanceof Date ? value : new Date(value);
    if (Number.isNaN(date.getTime())) return String(value);
    return new Intl.DateTimeFormat("ko-KR", {
      timeZone: "Asia/Seoul",
      hour: "2-digit",
      minute: "2-digit",
      second: "2-digit",
      fractionalSecondDigits: 3,
      hourCycle: "h23",
    }).format(date);
  };

  const formatAge = (timestamp) => {
    const date = timestamp ? new Date(timestamp) : null;
    if (!date || Number.isNaN(date.getTime())) return "방금 수신";
    const seconds = Math.max(0, Math.round((Date.now() - date.getTime()) / 1000));
    if (seconds < 2) return "실시간";
    if (seconds < 60) return `${seconds}초 전`;
    return `${Math.floor(seconds / 60)}분 전`;
  };

  const signalLabel = (signal) => {
    if (signal === null || signal === undefined || signal === "") return "대기";
    if (typeof signal === "object") {
      const text = firstDefined(signal.label, signal.status, signal.state, signal.name);
      if (text !== undefined) return String(text);
      const strength = asPercent(firstDefined(signal.strength, signal.quality, signal.percent));
      return strength === null ? "수신" : `${formatNumber(strength)}%`;
    }
    if (typeof signal === "boolean") return signal ? "정상" : "끊김";
    return String(signal);
  };

  const healthState = (value) => {
    const numeric = numberOrNull(value);
    if (numeric !== null) {
      if (numeric === 1 || numeric >= 80) return { label: "정상", tone: "ok" };
      if (numeric === 0) return { label: "미확인", tone: "warn" };
      if (numeric === 2 || numeric < 50) return { label: "비정상", tone: "bad" };
      return { label: "주의", tone: "warn" };
    }
    const boolean = boolValue(value);
    if (boolean === true) return { label: "정상", tone: "ok" };
    if (boolean === false) return { label: "비정상", tone: "bad" };
    const text = String(value ?? "").toLowerCase();
    if (/fail|error|bad|fault|비정상|고장|위험/.test(text)) return { label: "비정상", tone: "bad" };
    if (/warn|degrad|주의|경고/.test(text)) return { label: "주의", tone: "warn" };
    if (/ok|normal|healthy|정상/.test(text)) return { label: "정상", tone: "ok" };
    return { label: "미확인", tone: "warn" };
  };

  const statusState = (value) => {
    const numeric = numberOrNull(value);
    if (numeric === 1) return { label: "수행 중", tone: "active" };
    if (numeric === 2) return { label: "완료", tone: "done" };
    if (numeric === 3) return { label: "중단", tone: "bad" };
    if (numeric === 0) return { label: "대기", tone: "pending" };
    const text = String(value ?? "대기");
    const normalized = text.toLowerCase();
    if (/complete|done|finish|완료|종료/.test(normalized)) return { label: text, tone: "done" };
    if (/active|running|progress|execute|수행|진행|촬영/.test(normalized)) return { label: text, tone: "active" };
    if (/fail|error|cancel|중단|실패|오류/.test(normalized)) return { label: text, tone: "bad" };
    return { label: text, tone: "pending" };
  };

  const colorForVehicle = (id, index = 0) => {
    let hash = 0;
    for (const char of String(id)) hash = (hash * 31 + char.charCodeAt(0)) >>> 0;
    return VEHICLE_COLORS[(hash + index) % VEHICLE_COLORS.length];
  };

  const showToast = (message, duration = 2800) => {
    clearTimeout(app.toastTimer);
    els.toast.textContent = message;
    els.toast.hidden = false;
    app.toastTimer = setTimeout(() => {
      els.toast.hidden = true;
    }, duration);
  };

  const fetchJson = async (url, options = {}, timeout = 3500) => {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), timeout);
    try {
      const response = await fetch(url, {
        cache: "no-store",
        ...options,
        headers: { Accept: "application/json", ...(options.headers || {}) },
        signal: controller.signal,
      });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      return await response.json();
    } finally {
      clearTimeout(timer);
    }
  };

  const normalizeBounds = (bounds) => {
    if (!Array.isArray(bounds)) return null;
    if (bounds.length === 4 && bounds.every((value) => numberOrNull(value) !== null)) {
      return [[Number(bounds[0]), Number(bounds[1])], [Number(bounds[2]), Number(bounds[3])]];
    }
    if (bounds.length >= 2 && Array.isArray(bounds[0]) && Array.isArray(bounds[1])) {
      const west = numberOrNull(bounds[0][0]);
      const south = numberOrNull(bounds[0][1]);
      const east = numberOrNull(bounds[1][0]);
      const north = numberOrNull(bounds[1][1]);
      if ([west, south, east, north].every((value) => value !== null)) return [[west, south], [east, north]];
    }
    return null;
  };

  const normalizeConfig = (raw) => {
    const center = Array.isArray(raw?.center) && raw.center.length >= 2
      ? [numberOrNull(raw.center[0]), numberOrNull(raw.center[1])]
      : [127.5, 36.2];
    let tileUrl = "";
    if (typeof raw?.tileUrl === "string" && raw.tileUrl) {
      tileUrl = /^https?:\/\//i.test(raw.tileUrl)
        ? raw.tileUrl
        : `${window.location.origin}${raw.tileUrl.startsWith("/") ? "" : "/"}${raw.tileUrl}`;
    }
    return {
      tileUrl,
      minZoom: numberOrNull(raw?.minZoom) ?? 5,
      tileMaxZoom: numberOrNull(raw?.tileMaxZoom) ?? numberOrNull(raw?.maxZoom) ?? 18,
      maxZoom: numberOrNull(raw?.maxZoom) ?? 18,
      center: center.every((value) => value !== null) ? center : [127.5, 36.2],
      zoom: numberOrNull(raw?.zoom) ?? 9,
      bounds: normalizeBounds(raw?.bounds),
    };
  };

  const buildMapStyle = (config) => {
    const sources = {};
    const layers = [{ id: "base-background", type: "background", paint: { "background-color": "#29302c" } }];
    if (config.tileUrl) {
      sources.basemap = {
        type: "vector",
        tiles: [config.tileUrl],
        minzoom: config.minZoom,
        maxzoom: config.tileMaxZoom,
      };
      layers.push(
        { id: "base-landcover", type: "fill", source: "basemap", "source-layer": "landcover", paint: { "fill-color": "#333a35", "fill-opacity": 0.72 } },
        { id: "base-landuse", type: "fill", source: "basemap", "source-layer": "landuse", paint: { "fill-color": "#3a4038", "fill-opacity": 0.54 } },
        { id: "base-park", type: "fill", source: "basemap", "source-layer": "park", paint: { "fill-color": "#384538", "fill-opacity": 0.64 } },
        { id: "base-water", type: "fill", source: "basemap", "source-layer": "water", paint: { "fill-color": "#273b3c" } },
        { id: "base-waterway", type: "line", source: "basemap", "source-layer": "waterway", paint: { "line-color": "#3f5e5d", "line-width": 1 } },
        { id: "base-boundary", type: "line", source: "basemap", "source-layer": "boundary", paint: { "line-color": "#5e675f", "line-width": 0.8, "line-dasharray": [3, 3], "line-opacity": 0.6 } },
        { id: "base-roads", type: "line", source: "basemap", "source-layer": "transportation", paint: { "line-color": "#697069", "line-width": ["interpolate", ["linear"], ["zoom"], 7, 0.4, 14, 1.4], "line-opacity": 0.62 } },
        { id: "base-buildings", type: "fill", source: "basemap", "source-layer": "building", minzoom: 12, paint: { "fill-color": "#5a5d56", "fill-opacity": 0.56 } }
      );
    }
    return { version: 8, sources, layers };
  };

  const initMap = async () => {
    let config;
    try {
      config = normalizeConfig(await fetchJson("/api/config", {}, 4500));
      app.mapConfigured = true;
    } catch (error) {
      config = normalizeConfig({});
      els.mapMessage.hidden = false;
      console.warn("Map config unavailable", error);
    }
    app.config = config;

    if (!window.maplibregl) {
      els.mapMessage.hidden = false;
      els.mapMessage.querySelector("strong").textContent = "지도 모듈을 불러올 수 없습니다";
      return;
    }

    app.map = new window.maplibregl.Map({
      container: "map",
      style: buildMapStyle(config),
      center: config.center,
      zoom: config.zoom,
      minZoom: config.minZoom,
      maxZoom: config.maxZoom,
      attributionControl: true,
      dragRotate: false,
      pitchWithRotate: false,
      fadeDuration: 0,
    });

    app.map.addControl(new window.maplibregl.NavigationControl({ showCompass: false }), "top-right");
    app.map.addControl(new window.maplibregl.ScaleControl({ maxWidth: 100, unit: "metric" }), "bottom-left");

    app.map.on("load", () => {
      app.mapLoaded = true;
      addOperationalLayers();
      syncAllMapData();
      if (config.bounds) {
        app.map.fitBounds(config.bounds, { padding: 42, duration: 0, maxZoom: config.zoom + 1 });
      }
      if (app.mapConfigured) els.mapMessage.hidden = true;
    });

    app.map.on("error", (event) => {
      if (event?.error) console.warn("MapLibre", event.error.message || event.error);
    });

    app.map.on("mousemove", (event) => {
      updateCoordinateCursor(event);
      updateMissionHover(event);
    });

    app.map.on("mouseout", () => {
      hideCoordinateCursor();
      if (!app.markerHoverActive) hideMissionPopup();
    });

    app.map.on("click", handleMapClick);

    app.map.on("movestart", (event) => {
      if (event.originalEvent) {
        app.mapInteracted = true;
        if (app.followID) setFollow("");
      }
    });

    app.map.on("moveend", () => {
      syncCommandLabels(app.uavCommands, true);
      syncDiscoveryLabels(app.discoveries, true);
    });
    app.map.on("resize", () => {
      syncCommandLabels(app.uavCommands, true);
      syncDiscoveryLabels(app.discoveries, true);
    });
  };

  const coordinateText = (lngLat, digits = 6) => (
    `위도 ${Number(lngLat.lat).toFixed(digits)} · 경도 ${Number(lngLat.lng).toFixed(digits)}`
  );

  const updateCoordinateCursor = (event) => {
    const text = coordinateText(event.lngLat);
    els.mapCoordinate.textContent = text;
    if (!els.mapCoordinateCursor) return;
    els.mapCoordinateCursor.textContent = text;
    els.mapCoordinateCursor.style.left = `${event.point.x}px`;
    els.mapCoordinateCursor.style.top = `${event.point.y}px`;
    els.mapCoordinateCursor.hidden = false;
  };

  const hideCoordinateCursor = () => {
    if (els.mapCoordinateCursor) els.mapCoordinateCursor.hidden = true;
  };

  const copyCoordinateText = async (text) => {
    try {
      await navigator.clipboard.writeText(text);
      showToast("위경도 좌표를 복사했습니다.");
    } catch (_error) {
      showToast(`좌표: ${text}`);
    }
  };

  const showPinnedCoordinate = (lngLat) => {
    if (!app.mapLoaded || !lngLat) return;
    const latitude = Number(lngLat.lat);
    const longitude = Number(lngLat.lng);
    if (!Number.isFinite(latitude) || !Number.isFinite(longitude)) return;
    const copyText = `${latitude.toFixed(7)}, ${longitude.toFixed(7)}`;
    const content = document.createElement("div");
    content.className = "coordinate-popup-content";
    const title = document.createElement("strong");
    title.textContent = "선택 좌표";
    const coordinate = document.createElement("span");
    coordinate.textContent = `위도 ${latitude.toFixed(7)}`;
    const longitudeLine = document.createElement("span");
    longitudeLine.textContent = `경도 ${longitude.toFixed(7)}`;
    const copy = document.createElement("button");
    copy.type = "button";
    copy.textContent = "좌표 복사";
    copy.addEventListener("click", (event) => {
      event.stopPropagation();
      copyCoordinateText(copyText);
    });
    content.append(title, coordinate, longitudeLine, copy);
    app.coordinatePopup?.remove();
    app.coordinatePopup = new window.maplibregl.Popup({
      closeButton: true,
      closeOnClick: false,
      className: "coordinate-popup",
      maxWidth: "230px",
      offset: 10,
    })
      .setLngLat([longitude, latitude])
      .setDOMContent(content)
      .addTo(app.map);
  };

  const addGeoJsonSource = (id) => {
    if (!app.map.getSource(id)) app.map.addSource(id, { type: "geojson", data: EMPTY_FC });
  };

  const vehicleHeadingIconID = (color) => `vehicle-heading-${String(color).replace(/[^a-z0-9]/gi, "")}`;

  const addVehicleHeadingImages = () => {
    VEHICLE_COLORS.forEach((color) => {
      const iconID = vehicleHeadingIconID(color);
      if (app.map.hasImage(iconID)) return;
      const canvas = document.createElement("canvas");
      canvas.width = 28;
      canvas.height = 28;
      const context = canvas.getContext("2d");
      context.beginPath();
      context.moveTo(14, 2);
      context.lineTo(22, 23);
      context.lineTo(14, 19);
      context.lineTo(6, 23);
      context.closePath();
      context.fillStyle = color;
      context.fill();
      context.strokeStyle = "#202623";
      context.lineWidth = 1.5;
      context.stroke();
      context.beginPath();
      context.arc(14, 14, 3, 0, Math.PI * 2);
      context.fillStyle = "#f8faf8";
      context.fill();
      context.strokeStyle = "#202623";
      context.lineWidth = 1;
      context.stroke();
      app.map.addImage(iconID, context.getImageData(0, 0, 28, 28), { pixelRatio: 1 });
    });
  };

  const addOperationalLayers = () => {
    Object.values(SOURCE_IDS).forEach(addGeoJsonSource);
    addVehicleHeadingImages();
    const completedMission = [
      "any",
      ["==", ["get", "isHistorical"], true],
      ["==", ["get", "isDone"], true],
    ];

    const add = (definition) => {
      if (!app.map.getLayer(definition.id)) app.map.addLayer(definition);
    };

    add({ id: "mission-areas-fill", type: "fill", source: SOURCE_IDS.areas, filter: ["==", ["geometry-type"], "Polygon"], paint: {
      "fill-color": ["case", completedMission, "#668b88", ["coalesce", ["get", "color"], "#d79c37"]],
      "fill-opacity": ["match", ["get", "focusRole"], "selected", 0.2, "context", 0.025, "history", 0.055, ["case", completedMission, 0.08, 0.14]],
    } });
    add({ id: "mission-areas-line", type: "line", source: SOURCE_IDS.areas, paint: {
      "line-color": ["case", completedMission, "#8eb1ad", ["coalesce", ["get", "color"], "#e5b24e"]],
      "line-width": ["match", ["get", "focusRole"], "selected", 3.0, "context", 1.0, "history", 0.9, ["case", completedMission, 1.2, 1.7]],
      "line-opacity": ["match", ["get", "focusRole"], "selected", 1.0, "context", 0.38, "history", 0.3, 0.9],
      "line-dasharray": [5, 2],
    } });
    add({ id: "remaining-areas-fill", type: "fill", source: SOURCE_IDS.remainingAreas, filter: ["==", ["geometry-type"], "Polygon"], paint: { "fill-color": ["coalesce", ["get", "color"], "#3ee6cf"], "fill-opacity": ["case", ["==", ["get", "isDone"], 1], 0.04, 0.22] } });
    add({ id: "remaining-areas-line", type: "line", source: SOURCE_IDS.remainingAreas, paint: { "line-color": ["coalesce", ["get", "color"], "#3ee6cf"], "line-width": ["case", ["==", ["get", "isDone"], 1], 1.1, 2.4], "line-opacity": ["case", ["==", ["get", "isDone"], 1], 0.22, 0.9], "line-dasharray": [1.6, 0.7] } });
    add({ id: "option-assignments-fill", type: "fill", source: SOURCE_IDS.optionAssignments, filter: ["==", ["geometry-type"], "Polygon"], paint: { "fill-color": ["coalesce", ["get", "color"], "#58c7b5"], "fill-opacity": 0.23 } });
    add({ id: "option-assignments-corridor-line", type: "line", source: SOURCE_IDS.optionAssignments, filter: ["==", ["geometry-type"], "Polygon"], paint: { "line-color": ["coalesce", ["get", "color"], "#58c7b5"], "line-width": 2.4, "line-opacity": 0.96 } });
    add({ id: "option-assignments-centerline", type: "line", source: SOURCE_IDS.optionAssignments, filter: ["==", ["geometry-type"], "LineString"], paint: { "line-color": ["coalesce", ["get", "color"], "#58c7b5"], "line-width": 3.1, "line-opacity": 0.96, "line-dasharray": [2, 1] } });
    add({ id: "coverage-depth-fill", type: "fill", source: SOURCE_IDS.coverageDepth, paint: {
      "fill-color": ["match", ["get", "coverageDepth"], 0, "#ff5b68", 1, "#ffd166", 2, "#55d6a5", "#8b9b92"],
      "fill-opacity": ["match", ["get", "coverageDepth"], 0, 0.34, 1, 0.27, 2, 0.1, 0.12],
    } });
    add({ id: "coverage-depth-line", type: "line", source: SOURCE_IDS.coverageDepth, paint: {
      "line-color": ["match", ["get", "coverageDepth"], 0, "#ff7c86", 1, "#ffe194", 2, "#77e5bb", "#a7b3ac"],
      "line-width": ["match", ["get", "coverageDepth"], 0, 2.5, 1, 2.1, 2, 1.2, 1.2],
      "line-opacity": ["match", ["get", "coverageDepth"], 0, 0.96, 1, 0.9, 2, 0.5, 0.6],
    } });
    add({ id: "coverage-pass-forward-line", type: "line", source: SOURCE_IDS.coveragePassAttribution, filter: ["==", ["get", "coveragePass"], "forward"], paint: { "line-color": "#48ddff", "line-width": 2, "line-opacity": 0.82 } });
    add({ id: "coverage-pass-reverse-line", type: "line", source: SOURCE_IDS.coveragePassAttribution, filter: ["==", ["get", "coveragePass"], "reverse"], paint: { "line-color": "#ffb34d", "line-width": 2, "line-opacity": 0.82, "line-dasharray": [1.5, 1.2] } });
    add({ id: "mission-corridors-fill", type: "fill", source: SOURCE_IDS.corridors, filter: ["==", ["geometry-type"], "Polygon"], paint: {
      "fill-color": ["case", completedMission, "#5f7d84", "#da6654"],
      "fill-opacity": ["match", ["get", "focusRole"], "selected", 0.16, "context", 0.018, "history", 0.04, ["case", completedMission, 0.06, 0.1]],
    } });
    add({ id: "mission-corridors-line", type: "line", source: SOURCE_IDS.corridors, filter: ["==", ["geometry-type"], "Polygon"], paint: {
      "line-color": ["case", completedMission, "#83a6ad", "#e77763"],
      "line-width": ["match", ["get", "focusRole"], "selected", 2.7, "context", 0.9, "history", 0.8, 1.3],
      "line-opacity": ["match", ["get", "focusRole"], "selected", 1.0, "context", 0.32, "history", 0.28, 0.82],
    } });
    add({ id: "mission-input-lines", type: "line", source: SOURCE_IDS.corridors, filter: ["==", ["geometry-type"], "LineString"], paint: {
      "line-color": ["case", completedMission, "#9ab8bd", "#eb806c"],
      "line-width": ["match", ["get", "focusRole"], "selected", 3.2, "context", 1.0, "history", 0.9, ["case", completedMission, 1.5, 2.1]],
      "line-opacity": ["match", ["get", "focusRole"], "selected", 1.0, "context", 0.34, "history", 0.28, 0.88],
    } });
    add({ id: "mission-paths-casing", type: "line", source: SOURCE_IDS.paths, paint: { "line-color": "#151a17", "line-width": 4.0, "line-opacity": 0.66 } });
    add({ id: "mission-paths-line", type: "line", source: SOURCE_IDS.paths, paint: { "line-color": ["coalesce", ["get", "color"], "#f0eee1"], "line-width": 1.9, "line-opacity": 0.9, "line-dasharray": [3, 1.5] } });
    add({ id: "vehicle-tracks-casing", type: "line", source: SOURCE_IDS.tracks, paint: { "line-color": "#141916", "line-width": 4, "line-opacity": 0.52 } });
    add({ id: "vehicle-tracks-line", type: "line", source: SOURCE_IDS.tracks, paint: { "line-color": ["get", "color"], "line-width": 2.1, "line-opacity": 0.95 } });
    add({ id: "vehicle-position-arrow", type: "symbol", source: SOURCE_IDS.tracks, filter: ["==", ["geometry-type"], "Point"], layout: { "icon-image": ["get", "icon"], "icon-rotate": ["get", "heading"], "icon-rotation-alignment": "map", "icon-allow-overlap": true, "icon-ignore-placement": true } });
    add({ id: "footprint-trails-fill", type: "fill", source: SOURCE_IDS.footprintTrails, paint: { "fill-color": ["get", "color"], "fill-opacity": ["coalesce", ["get", "fillOpacity"], 0.002] } });
    add({ id: "footprint-trails-line", type: "line", source: SOURCE_IDS.footprintTrails, paint: { "line-color": ["get", "color"], "line-width": ["coalesce", ["get", "lineWidth"], 0.35], "line-opacity": ["coalesce", ["get", "lineOpacity"], 0.025] } });
    add({ id: "current-footprints-fill", type: "fill", source: SOURCE_IDS.footprints, paint: { "fill-color": ["get", "color"], "fill-opacity": 0.2 } });
    add({ id: "current-footprints-line", type: "line", source: SOURCE_IDS.footprints, paint: { "line-color": ["get", "color"], "line-width": 1.5, "line-opacity": 0.95 } });
    add({ id: "detection-footprints-fill", type: "fill", source: SOURCE_IDS.detectionFootprints, paint: { "fill-color": ["get", "color"], "fill-opacity": 0.13 } });
    add({ id: "detection-footprints-line", type: "line", source: SOURCE_IDS.detectionFootprints, paint: { "line-color": ["get", "color"], "line-width": 2.2, "line-opacity": 0.96, "line-dasharray": [2, 1] } });
    add({ id: "detection-points-halo", type: "circle", source: SOURCE_IDS.detectionFootprints, filter: ["==", ["geometry-type"], "Point"], paint: { "circle-radius": 8, "circle-color": "rgba(0,0,0,0)", "circle-stroke-color": ["get", "color"], "circle-stroke-width": 1.3, "circle-stroke-opacity": 0.72 } });
    add({ id: "detection-points", type: "circle", source: SOURCE_IDS.detectionFootprints, filter: ["==", ["geometry-type"], "Point"], paint: { "circle-radius": 4.2, "circle-color": ["get", "color"], "circle-stroke-color": "#fff8ef", "circle-stroke-width": 1.2 } });
    add({ id: "uav-command-points", type: "circle", source: SOURCE_IDS.uavCommands, paint: {
      "circle-radius": ["interpolate", ["linear"], ["zoom"], 7, 3.5, 14, 5.5, 19, 7],
      "circle-color": ["get", "color"],
      "circle-stroke-color": "#f7faf8",
      "circle-stroke-width": 1.5,
      "circle-opacity": 0.96,
    } });
    add({ id: "targets-fill", type: "fill", source: SOURCE_IDS.targets, filter: ["==", ["geometry-type"], "Polygon"], paint: { "fill-color": "#dc5749", "fill-opacity": 0.16 } });
    add({ id: "targets-line", type: "line", source: SOURCE_IDS.targets, filter: ["==", ["geometry-type"], "Polygon"], paint: { "line-color": "#f06b5d", "line-width": 1.5 } });
    add({ id: "targets-halo", type: "circle", source: SOURCE_IDS.targets, filter: ["==", ["geometry-type"], "Point"], paint: { "circle-radius": 8, "circle-color": "rgba(0,0,0,0)", "circle-stroke-color": "#f06b5d", "circle-stroke-width": 1.2, "circle-stroke-opacity": 0.7 } });
    add({ id: "targets-point", type: "circle", source: SOURCE_IDS.targets, filter: ["==", ["geometry-type"], "Point"], paint: { "circle-radius": 3.8, "circle-color": "#ef6153", "circle-stroke-color": "#fff3ec", "circle-stroke-width": 1 } });

    document.querySelectorAll("[data-layer-toggle]").forEach((input) => {
      setLayerVisibility(input.dataset.layerToggle, input.checked);
    });
  };

  const setLayerVisibility = (group, visible) => {
    app.layerVisibility[group] = Boolean(visible);
    if (!app.mapLoaded) return;
    for (const layerID of LAYER_GROUPS[group] || []) {
      if (app.map.getLayer(layerID)) app.map.setLayoutProperty(layerID, "visibility", visible ? "visible" : "none");
    }
    for (const record of app.missionLabels.values()) {
      if ((group === "areas" && record.shape === "AREA") || (group === "corridors" && record.shape === "LINE")) {
        record.element.hidden = !visible;
      }
    }
    if (group === "uavCommands") {
      if (visible) syncCommandLabels(app.uavCommands, true);
      else clearCommandLabels();
    }
    if (group === "detectionFootprints") {
      if (visible) syncDiscoveryLabels(app.discoveries, true);
      else clearDiscoveryLabels();
    }
  };

  const setLayerToggleChecked = (group, checked) => {
    const input = document.querySelector(`[data-layer-toggle="${group}"]`);
    if (input) input.checked = Boolean(checked);
    setLayerVisibility(group, checked);
  };

  const setSourceData = (sourceID, data) => {
    if (!app.mapLoaded) return;
    const source = app.map.getSource(sourceID);
    if (source) source.setData(data && data.type ? data : EMPTY_FC);
  };

  const featureCollection = (features) => ({ type: "FeatureCollection", features: features.filter(Boolean) });

  const isCoordinate = (value) => Array.isArray(value) && value.length >= 2 && numberOrNull(value[0]) !== null && numberOrNull(value[1]) !== null;

  const closeRing = (coordinates) => {
    const ring = coordinates.filter(isCoordinate).map((coord) => [Number(coord[0]), Number(coord[1])]);
    if (ring.length < 3) return [];
    const first = ring[0];
    const last = ring[ring.length - 1];
    if (first[0] !== last[0] || first[1] !== last[1]) ring.push([...first]);
    return ring;
  };

  const guessGeometry = (coordinates, preferredType = "") => {
    if (!Array.isArray(coordinates) || !coordinates.length) return null;
    if (isCoordinate(coordinates)) return { type: "Point", coordinates: [Number(coordinates[0]), Number(coordinates[1])] };
    if (coordinates.every(isCoordinate)) {
      if (/polygon|area|corridor/i.test(preferredType)) {
        const ring = closeRing(coordinates);
        return ring.length ? { type: "Polygon", coordinates: [ring] } : null;
      }
      return { type: "LineString", coordinates: coordinates.map((coord) => [Number(coord[0]), Number(coord[1])]) };
    }
    if (Array.isArray(coordinates[0]) && coordinates[0].every?.(isCoordinate)) {
      const rings = coordinates.map(closeRing).filter((ring) => ring.length);
      return rings.length ? { type: "Polygon", coordinates: rings } : null;
    }
    return null;
  };

  const normalizeFeature = (entry, index, preferredType = "") => {
    if (!entry) return null;
    if (entry.type === "Feature" && entry.geometry) {
      return { ...entry, properties: { ...(entry.properties || {}), _index: index } };
    }
    if (entry.type && entry.coordinates) {
      return { type: "Feature", properties: { _index: index }, geometry: entry };
    }
    const geometry = entry.geometry || guessGeometry(
      Array.isArray(entry)
        ? entry
        : firstDefined(entry.coordinates, entry.points, entry.path, entry.polygon, entry.vertices),
      preferredType
    );
    if (!geometry) return null;
    const properties = { ...entry };
    delete properties.geometry;
    delete properties.coordinates;
    delete properties.points;
    delete properties.path;
    delete properties.polygon;
    delete properties.vertices;
    return { type: "Feature", properties: { ...properties, _index: index }, geometry };
  };

  const normalizeGeoJson = (value, preferredType = "") => {
    if (!value) return featureCollection([]);
    if (value.type === "FeatureCollection") {
      return featureCollection(toArray(value.features).map((entry, index) => normalizeFeature(entry, index, preferredType)));
    }
    if (value.type === "Feature" || (value.type && value.coordinates)) {
      return featureCollection([normalizeFeature(value, 0, preferredType)]);
    }
    const items = Array.isArray(value) ? value : firstDefined(value.features, value.items, value.data, []);
    if (Array.isArray(items) && items.length && isCoordinate(items[0])) {
      return featureCollection([normalizeFeature({ coordinates: items }, 0, preferredType)]);
    }
    return featureCollection(toArray(items).map((entry, index) => normalizeFeature(entry, index, preferredType)));
  };

  const mergeCollections = (...collections) => featureCollection(collections.flatMap((collection) => toArray(collection?.features)));

  const missionIDKey = (value) => {
    if (value === undefined || value === null || value === "") return "";
    return String(value);
  };

  const featureInputMissionID = (feature) => firstDefined(
    feature?.properties?.inputMissionID,
    feature?.properties?.inputMissionId,
    feature?.properties?.inputID,
  );

  const focusedInputMissionIDs = () => {
    if (app.missionViewMode === "all") return [];
    if (app.missionViewMode === "selected") {
      const selected = missionIDKey(app.selectedInputMissionID);
      return selected ? [selected] : [];
    }
    return app.currentInputMissionIDs.map(missionIDKey).filter(Boolean);
  };

  const missionFocusRole = (properties = {}) => {
    if (properties.isHistorical === true || properties.isHistorical === "true") return "history";
    if (app.missionViewMode === "all") return "normal";
    const focused = new Set(focusedInputMissionIDs());
    if (!focused.size) return "normal";
    return focused.has(missionIDKey(firstDefined(
      properties.inputMissionID,
      properties.inputMissionId,
      properties.inputID,
    ))) ? "selected" : "context";
  };

  const applyMissionDisplayScope = (collection, { detail = false } = {}) => {
    const sourceFeatures = toArray(collection?.features);
    const historyFiltered = sourceFeatures.filter((feature) => (
      app.showMissionHistory
      || !(feature?.properties?.isHistorical === true || feature?.properties?.isHistorical === "true")
    ));
    const focused = new Set(focusedInputMissionIDs());
    const hasMissionIDs = historyFiltered.some((feature) => missionIDKey(featureInputMissionID(feature)));
    const visible = detail && app.missionViewMode !== "all" && focused.size && hasMissionIDs
      ? historyFiltered.filter((feature) => focused.has(missionIDKey(featureInputMissionID(feature))))
      : historyFiltered;
    return featureCollection(visible.map((feature) => ({
      ...feature,
      properties: {
        ...(feature?.properties || {}),
        focusRole: missionFocusRole(feature?.properties || {}),
      },
    })));
  };

  const partForFeature = (properties = {}, shape = "") => {
    const inputMissionID = firstDefined(properties.inputMissionID, properties.inputMissionId, properties.inputID);
    if (inputMissionID !== undefined) {
      const matched = app.missionParts.get(String(inputMissionID));
      if (matched) return matched;
    }
    const sequence = firstDefined(properties.sequence, properties.sequenceNumber);
    if (sequence === undefined) return null;
    return [...app.missionParts.values()].find((part) => (
      String(part.sequence) === String(sequence) && (!shape || part.shape === shape)
    )) || null;
  };

  const enrichMissionCollection = (collection, shape) => featureCollection(toArray(collection?.features).map((feature) => {
    const base = feature?.properties || {};
    const historical = base.isHistorical === true || base.isHistorical === "true";
    const part = historical ? null : partForFeature(base, shape);
    const coverageDetail = part?.coverageDetail || {};
    const partStatus = statusState(
      part?.status && typeof part.status === "object"
        ? firstDefined(part.status.label, part.status.name, part.status.state, "대기")
        : firstDefined(part?.status, "대기")
    );
    if (part?.statusTone) partStatus.tone = String(part.statusTone);
    const coverageValue = firstDefined(base.coverageValue, part?.coverage, -1);
    const numericCoverage = numberOrNull(coverageValue);
    const gsdState = historical
      ? String(firstDefined(base.gsdState, "unknown"))
      : part?.gsdSatisfied === true ? "pass" : part?.gsdSatisfied === false ? "fail" : "unknown";
    return {
      ...feature,
      properties: {
        ...base,
        inputMissionID: firstDefined(base.inputMissionID, part?.inputMissionID, "-"),
        sequence: firstDefined(base.sequence, part?.sequence, "-"),
        shape,
        typeLabel: String(firstDefined(base.typeLabel, part?.type, base.inputMissionType, "-")),
        regionLabel: String(firstDefined(base.regionLabel, part?.region, base.regionType, "-")),
        statusLabel: String(firstDefined(base.statusLabel, partStatus.label, "대기")),
        statusTone: String(firstDefined(base.statusTone, partStatus.tone, "pending")),
        coverageValue,
        coverageLabel: numericCoverage !== null && numericCoverage >= 0 ? formatPercent(numericCoverage, 1) : "-",
        coveredValue: numberOrNull(firstDefined(base.coveredValue, coverageDetail.covered)) ?? -1,
        plannedValue: numberOrNull(firstDefined(base.plannedValue, coverageDetail.planned)) ?? -1,
        coverageUnit: String(firstDefined(base.coverageUnit, coverageDetail.unit, "")),
        coverageSource: String(firstDefined(base.coverageSource, coverageDetail.source, "")),
        coveragePassDetails: firstDefined(base.coveragePassDetails, coverageDetail.passes, []),
        coverageDepthDetails: firstDefined(base.coverageDepthDetails, coverageDetail.coverageDepthDetails, []),
        coverageDepthPolicy: firstDefined(base.coverageDepthPolicy, coverageDetail.coverageDepthPolicy, ""),
        requiredCoverageDepth: firstDefined(base.requiredCoverageDepth, coverageDetail.requiredCoverageDepth, 2),
        remainingCoverageDepth: firstDefined(base.remainingCoverageDepth, coverageDetail.remainingCoverageDepth, -1),
        completedCoverageDepth: firstDefined(base.completedCoverageDepth, coverageDetail.completedCoverageDepth, -1),
        spatialCoveragePercent: firstDefined(base.spatialCoveragePercent, coverageDetail.spatialPercent, -1),
        spatialCoveredValue: firstDefined(base.spatialCoveredValue, coverageDetail.spatialCovered, -1),
        spatialPlannedValue: firstDefined(base.spatialPlannedValue, coverageDetail.spatialPlanned, -1),
        coverageRequirementsMet: firstDefined(base.coverageRequirementsMet, coverageDetail.requirementsMet, false),
        measuredGsd: firstDefined(base.measuredGsd, part?.measuredGsd, -1),
        targetGsd: firstDefined(base.targetGsd, part?.targetGsd, -1),
        gsdState,
        qualitySamples: firstDefined(base.qualitySamples, part?.qualitySamples, 0),
        qualitySatisfaction: firstDefined(base.qualitySatisfaction, part?.qualitySatisfaction, -1),
        activeAircraftCount: firstDefined(base.activeAircraftCount, part?.activeAircraftCount, 0),
        isCurrent: historical ? false : part?.isCurrent === true,
        isHistorical: historical,
        historyPlanID: firstDefined(base.historyPlanID, ""),
      },
    };
  }));

  const enrichRemainingAreaCollection = (collection) => {
    const enriched = enrichMissionCollection(collection, "AREA");
    return featureCollection(toArray(enriched.features).map((feature) => {
      const properties = feature?.properties || {};
      const aircraftID = firstDefined(properties.aircraftID, properties.aircraftId);
      const agent = String(firstDefined(properties.agent, "")).trim();
      const vehicle = aircraftID === undefined || aircraftID === null
        ? null
        : app.vehicles.get(String(aircraftID));
      const identity = aircraftID ?? agent;
      return {
        ...feature,
        properties: {
          ...properties,
          color: firstDefined(
            properties.color,
            vehicle?.color,
            identity !== "" && identity !== undefined && identity !== null
              ? colorForVehicle(identity)
              : "#3ee6cf",
          ),
          visualizationRole: firstDefined(properties.visualizationRole, "remainingArea"),
        },
      };
    }));
  };

  const lineCoordinateAtFraction = (coordinates, fraction = 0.5) => {
    const points = toArray(coordinates).filter(isCoordinate);
    if (!points.length) return null;
    if (points.length === 1) return [Number(points[0][0]), Number(points[0][1])];
    const segments = [];
    let total = 0;
    for (let index = 1; index < points.length; index += 1) {
      const dx = Number(points[index][0]) - Number(points[index - 1][0]);
      const dy = Number(points[index][1]) - Number(points[index - 1][1]);
      const length = Math.hypot(dx, dy);
      segments.push({ start: points[index - 1], end: points[index], length });
      total += length;
    }
    if (!total) return [Number(points[0][0]), Number(points[0][1])];
    let remaining = total * clamp(fraction, 0, 1);
    for (const segment of segments) {
      if (remaining <= segment.length) {
        const ratio = segment.length ? remaining / segment.length : 0;
        return [
          Number(segment.start[0]) + (Number(segment.end[0]) - Number(segment.start[0])) * ratio,
          Number(segment.start[1]) + (Number(segment.end[1]) - Number(segment.start[1])) * ratio,
        ];
      }
      remaining -= segment.length;
    }
    const last = points[points.length - 1];
    return [Number(last[0]), Number(last[1])];
  };

  const polygonAnchor = (ring) => {
    const points = toArray(ring).filter(isCoordinate);
    if (!points.length) return null;
    let twiceArea = 0;
    let x = 0;
    let y = 0;
    for (let index = 0; index < points.length - 1; index += 1) {
      const cross = Number(points[index][0]) * Number(points[index + 1][1]) - Number(points[index + 1][0]) * Number(points[index][1]);
      twiceArea += cross;
      x += (Number(points[index][0]) + Number(points[index + 1][0])) * cross;
      y += (Number(points[index][1]) + Number(points[index + 1][1])) * cross;
    }
    if (Math.abs(twiceArea) > 1e-12) return [x / (3 * twiceArea), y / (3 * twiceArea)];
    return [
      points.reduce((sum, point) => sum + Number(point[0]), 0) / points.length,
      points.reduce((sum, point) => sum + Number(point[1]), 0) / points.length,
    ];
  };

  const polygonRingArea = (ring) => {
    const points = toArray(ring).filter(isCoordinate);
    let area = 0;
    for (let index = 1; index < points.length; index += 1) {
      area += Number(points[index - 1][0]) * Number(points[index][1])
        - Number(points[index][0]) * Number(points[index - 1][1]);
    }
    return Math.abs(area) / 2;
  };

  const lineCoordinateLength = (coordinates) => {
    const points = toArray(coordinates).filter(isCoordinate);
    let length = 0;
    for (let index = 1; index < points.length; index += 1) {
      length += Math.hypot(
        Number(points[index][0]) - Number(points[index - 1][0]),
        Number(points[index][1]) - Number(points[index - 1][1]),
      );
    }
    return length;
  };

  const missionLabelDescriptors = (areas, inputLines) => {
    const descriptors = [];
    for (const feature of toArray(areas?.features)) {
      if (app.missionViewMode !== "all" && feature?.properties?.focusRole === "context") continue;
      const polygons = feature?.geometry?.type === "MultiPolygon"
        ? feature.geometry.coordinates
        : feature?.geometry?.type === "Polygon" ? [feature.geometry.coordinates] : [];
      const largestRing = polygons
        .map((polygon) => polygon?.[0])
        .filter((ring) => toArray(ring).length >= 3)
        .sort((left, right) => polygonRingArea(right) - polygonRingArea(left))[0];
      const coordinate = polygonAnchor(largestRing);
      if (!coordinate) continue;
      const inputMissionID = firstDefined(feature.properties?.inputMissionID, feature.properties?._index, "area");
      const historyKey = feature.properties?.isHistorical ? `:history:${feature.properties?.historyPlanID || "-"}` : "";
      descriptors.push({
        key: `area:${inputMissionID}${historyKey}`,
        shape: "AREA",
        coordinate,
        properties: feature.properties || {},
        offset: [0, 0],
      });
    }

    const lineDescriptors = new Map();
    for (const feature of toArray(inputLines?.features)) {
      if (app.missionViewMode !== "all" && feature?.properties?.focusRole === "context") continue;
      const coordinate = lineCoordinateAtFraction(feature?.geometry?.coordinates, 0.5);
      if (!coordinate) continue;
      const inputMissionID = firstDefined(feature.properties?.inputMissionID, feature.properties?._index, "line");
      const sequence = numberOrNull(feature.properties?.sequence) ?? 0;
      const historyKey = feature.properties?.isHistorical ? `:history:${feature.properties?.historyPlanID || "-"}` : "";
      const key = `line:${inputMissionID}${historyKey}`;
      const candidate = {
        key,
        shape: "LINE",
        coordinate,
        properties: feature.properties || {},
        offset: [0, sequence % 2 === 0 ? 12 : -12],
        length: lineCoordinateLength(feature?.geometry?.coordinates),
      };
      const existing = lineDescriptors.get(key);
      if (!existing || candidate.length > existing.length) lineDescriptors.set(key, candidate);
    }
    descriptors.push(...lineDescriptors.values());
    return descriptors.map(({ length: _length, ...descriptor }) => descriptor);
  };

  const declutterMissionLabels = (descriptors) => {
    if (!app.mapLoaded) return descriptors;
    const occupied = [];
    const canvas = app.map.getCanvas();
    const xOffsets = [0, -86, 86, -172, 172];
    const yOffsets = [0, -25, 25, -50, 50, -75, 75];
    const candidates = xOffsets
      .flatMap((x) => yOffsets.map((y) => [x, y]))
      .sort((left, right) => (Math.abs(left[0]) + Math.abs(left[1])) - (Math.abs(right[0]) + Math.abs(right[1])));
    const overlaps = (left, right) => !(
      left.right + 3 < right.left || left.left - 3 > right.right ||
      left.bottom + 3 < right.top || left.top - 3 > right.bottom
    );
    return [...descriptors]
      .sort((left, right) => (numberOrNull(left.properties?.sequence) ?? 999) - (numberOrNull(right.properties?.sequence) ?? 999))
      .map((descriptor) => {
        const anchor = app.map.project(descriptor.coordinate);
        const labelText = `M${descriptor.properties?.sequence ?? "-"} ${descriptor.shape} ${descriptor.properties?.coverageLabel || "-"}`;
        const width = clamp(labelText.length * 6.2 + 12, 66, 104);
        const height = 22;
        let selected = descriptor.offset || [0, 0];
        let selectedBox = null;
        let bestScore = Number.POSITIVE_INFINITY;
        for (const candidate of candidates) {
          const offset = [candidate[0] + (descriptor.offset?.[0] || 0), candidate[1] + (descriptor.offset?.[1] || 0)];
          const box = {
            left: anchor.x + offset[0] - width / 2,
            right: anchor.x + offset[0] + width / 2,
            top: anchor.y + offset[1] - height / 2,
            bottom: anchor.y + offset[1] + height / 2,
          };
          const collisions = occupied.filter((entry) => overlaps(box, entry)).length;
          const outside = Math.max(0, -box.left) + Math.max(0, box.right - canvas.clientWidth)
            + Math.max(0, -box.top) + Math.max(0, box.bottom - canvas.clientHeight);
          const score = collisions * 10000 + outside * 100 + Math.abs(offset[0]) + Math.abs(offset[1]);
          if (score < bestScore) {
            bestScore = score;
            selected = offset;
            selectedBox = box;
          }
          if (collisions === 0 && outside === 0) break;
        }
        if (selectedBox) occupied.push(selectedBox);
        return { ...descriptor, offset: selected };
      });
  };

  const coverageSourceLabel = (source) => {
    if (/footprint/i.test(String(source))) return "실측 footprint";
    if (/path|route/i.test(String(source))) return "비행 경로";
    return source ? String(source) : "수집 대기";
  };

  const normalizeCoveragePassRows = (value) => toArray(value).map((raw, index) => {
    const coveragePass = String(firstDefined(raw?.coveragePass, raw?.coverage_pass, "")).toLowerCase();
    const actualCoveredM2 = numberOrNull(firstDefined(raw?.actualCoveredM2, raw?.actual_covered_area_m2, raw?.covered_area_m2)) ?? 0;
    const requiredM2 = numberOrNull(firstDefined(raw?.requiredM2, raw?.required_area_m2, raw?.planned_area_m2)) ?? 0;
    const remainingM2 = numberOrNull(firstDefined(raw?.remainingM2, raw?.remaining_area_m2)) ?? Math.max(0, requiredM2 - actualCoveredM2);
    const percent = asPercent(firstDefined(raw?.percent, raw?.coverage_percent))
      ?? (requiredM2 > 0 ? clamp(actualCoveredM2 / requiredM2 * 100, 0, 100) : 0);
    return {
      coveragePass,
      passIndex: numberOrNull(firstDefined(raw?.passIndex, raw?.pass_index)) ?? index + 1,
      actualCoveredM2,
      requiredM2,
      remainingM2,
      percent,
      requirementsMet: boolValue(firstDefined(raw?.requirementsMet, raw?.requirement_met, raw?.is_done)) === true,
      status: String(firstDefined(raw?.status, "planned")),
    };
  }).filter((row) => row.coveragePass);

  const normalizeCoverageDepthRows = (value) => toArray(value).map((raw) => {
    const coverageDepth = numberOrNull(firstDefined(raw?.coverageDepth, raw?.coverage_depth));
    if (![0, 1, 2].includes(coverageDepth)) return null;
    const remainingCaptureCount = numberOrNull(
      firstDefined(raw?.remainingCaptureCount, raw?.remaining_capture_count),
    ) ?? (2 - coverageDepth);
    return {
      coverageDepth,
      remainingCaptureCount,
      areaM2: numberOrNull(firstDefined(raw?.areaM2, raw?.area_m2, raw?.remainingAreaM2)) ?? 0,
      coveragePercent: asPercent(firstDefined(raw?.coveragePercent, raw?.coverage_percent)),
      activeAircraftIDs: toArray(firstDefined(raw?.activeAircraftIDs, raw?.active_aircraft_ids)),
      activeCoveragePasses: toArray(firstDefined(raw?.activeCoveragePasses, raw?.active_coverage_passes)),
    };
  }).filter(Boolean).sort((left, right) => left.coverageDepth - right.coverageDepth);

  const coveragePassLabel = (coveragePass) => coveragePass === "forward"
    ? "OUT PATH · attribution"
    : coveragePass === "reverse" ? "RETURN PATH · attribution" : String(coveragePass || "PATH").toUpperCase();

  const coveragePassPercentLabel = (row) => {
    const percent = asPercent(row?.percent);
    if (percent === null) return "-";
    if (!row?.requirementsMet && percent >= 99.995) return "<100%";
    return formatPercent(percent, row?.requirementsMet ? 1 : 2);
  };

  const missionPopupContent = (properties = {}) => {
    const root = document.createElement("section");
    root.className = "mission-popup";
    root.dataset.quality = String(properties.gsdState || "unknown");

    const header = document.createElement("header");
    const title = document.createElement("strong");
    title.textContent = `M${properties.sequence ?? "-"} · ${properties.shape || "임무"}`;
    const status = document.createElement("span");
    status.className = `mission-popup-status ${properties.statusTone || "pending"}`;
    status.textContent = properties.statusLabel || "대기";
    header.append(title, status);

    const identity = document.createElement("p");
    identity.className = "mission-popup-identity";
    const historyPlan = properties.isHistorical ? ` · 완료 Plan ${properties.historyPlanID || "-"}` : "";
    identity.textContent = `${properties.typeLabel || "-"} · ${properties.regionLabel || "-"}${historyPlan}`;

    const coverageDepth = numberOrNull(properties.coverageDepth);
    const depthCard = document.createElement("div");
    depthCard.className = `mission-popup-depth depth-${coverageDepth ?? "unknown"}`;
    if (coverageDepth !== null && coverageDepth >= 0 && coverageDepth <= 2) {
      const depthTitle = document.createElement("strong");
      depthTitle.textContent = `Capture depth ${coverageDepth}/2`;
      const depthRemaining = document.createElement("span");
      depthRemaining.textContent = coverageDepth >= 2
        ? "COMPLETE"
        : `NEED ${numberOrNull(properties.remainingCaptureCount) ?? (2 - coverageDepth)}`;
      const depthAttribution = document.createElement("small");
      const activeAgents = String(properties.activeAgents || "").trim();
      const activePasses = String(properties.activeCoveragePasses || "").trim();
      depthAttribution.textContent = [
        activeAgents ? `Aircraft ${activeAgents}` : "",
        activePasses ? `Attribution ${activePasses}` : "",
      ].filter(Boolean).join(" · ") || "Attribution pending";
      depthCard.append(depthTitle, depthRemaining, depthAttribution);
    }

    const rawCoverageValue = numberOrNull(properties.coverageValue);
    const coverageValue = rawCoverageValue !== null && rawCoverageValue >= 0 ? rawCoverageValue : null;
    const coverage = document.createElement("div");
    coverage.className = "mission-popup-coverage";
    const coverageHead = document.createElement("div");
    const coverageLabel = document.createElement("span");
    coverageLabel.textContent = `커버리지 · ${coverageSourceLabel(properties.coverageSource)}`;
    const coverageStrong = document.createElement("strong");
    coverageStrong.textContent = formatPercent(coverageValue, 1);
    coverageHead.append(coverageLabel, coverageStrong);
    const track = document.createElement("div");
    track.className = "mission-popup-progress";
    const fill = document.createElement("span");
    fill.style.width = `${clamp(coverageValue ?? 0, 0, 100)}%`;
    track.append(fill);
    const covered = numberOrNull(properties.coveredValue);
    const planned = numberOrNull(properties.plannedValue);
    const coverageMeta = document.createElement("small");
    coverageMeta.textContent = covered === null || covered < 0 || planned === null || planned < 0
      ? "면적 산출 대기"
      : `${formatNumber(covered, 1)} / ${formatNumber(planned, 1)} ${properties.coverageUnit || ""}`;
    coverage.append(coverageHead, track, coverageMeta);

    const passRows = normalizeCoveragePassRows(properties.coveragePassDetails);
    const depthRows = normalizeCoverageDepthRows(properties.coverageDepthDetails);
    const depthBands = document.createElement("div");
    depthBands.className = "mission-popup-depth-bands";
    depthRows.forEach((row) => {
      const band = document.createElement("div");
      band.className = `mission-popup-depth-band depth-${row.coverageDepth}`;
      const label = document.createElement("strong");
      label.textContent = `${row.coverageDepth}/2`;
      const area = document.createElement("span");
      area.textContent = `${formatNumber(row.areaM2, 1)} m²`;
      const remaining = document.createElement("small");
      remaining.textContent = row.remainingCaptureCount > 0
        ? `NEED ${row.remainingCaptureCount}`
        : "COMPLETE";
      band.append(label, area, remaining);
      depthBands.append(band);
    });
    const passes = document.createElement("div");
    passes.className = "mission-popup-passes";
    passRows.forEach((row) => {
      const wrapper = document.createElement("div");
      wrapper.className = `mission-popup-pass ${row.coveragePass} ${row.requirementsMet ? "is-done" : row.status === "active" ? "is-active" : ""}`;
      const heading = document.createElement("div");
      const label = document.createElement("span");
      label.textContent = coveragePassLabel(row.coveragePass);
      const value = document.createElement("strong");
      value.textContent = coveragePassPercentLabel(row);
      heading.append(label, value);
      const passTrack = document.createElement("div");
      passTrack.className = "mission-popup-pass-track";
      const passFill = document.createElement("span");
      passFill.style.width = `${clamp(row.percent ?? 0, 0, 100)}%`;
      passTrack.append(passFill);
      const detail = document.createElement("small");
      detail.textContent = `${formatNumber(row.actualCoveredM2, 1)} / ${formatNumber(row.requiredM2, 1)} m² · 잔여 ${formatNumber(row.remainingM2, 1)} m²`;
      wrapper.append(heading, passTrack, detail);
      passes.append(wrapper);
    });

    const metrics = document.createElement("dl");
    metrics.className = "mission-popup-metrics";
    const addMetric = (term, value, tone = "") => {
      const wrapper = document.createElement("div");
      const dt = document.createElement("dt");
      const dd = document.createElement("dd");
      dt.textContent = term;
      dd.textContent = value;
      if (tone) dd.dataset.tone = tone;
      wrapper.append(dt, dd);
      metrics.append(wrapper);
    };
    const measured = numberOrNull(properties.measuredGsd);
    const target = numberOrNull(properties.targetGsd);
    const gsdLabel = properties.gsdState === "pass" ? "충족" : properties.gsdState === "fail" ? "미충족" : "미평가";
    const gsdValue = measured === null || measured < 0
      ? gsdLabel
      : `${formatNumber(measured, 2)} cm/px · ${gsdLabel}${target === null || target < 0 ? "" : ` (기준 ${formatNumber(target, 2)})`}`;
    addMetric("공간해상도", gsdValue, properties.gsdState || "unknown");
    const samples = numberOrNull(properties.qualitySamples) ?? 0;
    const satisfaction = numberOrNull(properties.qualitySatisfaction);
    addMetric("촬영 품질", samples > 0 ? `${formatPercent(satisfaction, 1)} · ${samples}회` : "샘플 대기", samples > 0 ? "pass" : "unknown");
    if (passRows.length) {
      const spatialPercent = asPercent(properties.spatialCoveragePercent);
      addMetric(
        "OUT·RETURN 통합 영역",
        `${formatPercent(spatialPercent, 1)} · ${boolValue(properties.coverageRequirementsMet) === true ? "충족" : "미충족"}`,
        boolValue(properties.coverageRequirementsMet) === true ? "pass" : "fail",
      );
    }
    addMetric("투입 기체", `${numberOrNull(properties.activeAircraftCount) ?? 0}대`);

    root.append(header, identity);
    if (coverageDepth !== null && coverageDepth >= 0 && coverageDepth <= 2) root.append(depthCard);
    root.append(coverage);
    if (depthRows.length) root.append(depthBands);
    if (passRows.length) root.append(passes);
    root.append(metrics);
    return root;
  };

  const showMissionPopup = (properties, coordinate) => {
    if (!app.mapLoaded || !coordinate) return;
    if (!app.hoverPopup) {
      app.hoverPopup = new window.maplibregl.Popup({
        closeButton: false,
        closeOnClick: false,
        offset: 13,
        className: "mission-hover-popup",
      });
    }
    app.hoverPopup.setLngLat(coordinate).setDOMContent(missionPopupContent(properties)).addTo(app.map);
  };

  const hideMissionPopup = () => {
    app.hoverPopup?.remove();
  };

  const missionFeatureAtPoint = (point) => {
    if (!app.mapLoaded) return null;
    const layers = INTERACTIVE_MISSION_LAYERS.filter((layerID) => app.map.getLayer(layerID));
    return layers.length
      ? app.map.queryRenderedFeatures(point, { layers }).find(
        (entry) => entry?.properties?.inputMissionID !== undefined,
      ) || null
      : null;
  };

  const updateMissionHover = (event) => {
    if (!app.mapLoaded || app.markerHoverActive) return;
    const feature = missionFeatureAtPoint(event.point);
    app.map.getCanvas().style.cursor = feature ? "pointer" : "";
    if (feature) showMissionPopup(feature.properties, event.lngLat);
    else hideMissionPopup();
  };

  const handleMapClick = (event) => {
    const feature = missionFeatureAtPoint(event.point);
    if (feature) {
      selectInputMission(feature.properties?.inputMissionID, { fit: false });
      showMissionPopup(feature.properties, event.lngLat);
      return;
    }
    showPinnedCoordinate(event.lngLat);
  };

  const syncMissionLabels = (areas, inputLines) => {
    if (!app.mapLoaded) return;
    const descriptors = declutterMissionLabels(missionLabelDescriptors(areas, inputLines));
    const activeKeys = new Set(descriptors.map((descriptor) => descriptor.key));
    for (const [key, record] of app.missionLabels) {
      if (!activeKeys.has(key)) {
        record.marker.remove();
        app.missionLabels.delete(key);
      }
    }
    for (const descriptor of descriptors) {
      let record = app.missionLabels.get(descriptor.key);
      if (!record) {
        const element = document.createElement("button");
        element.type = "button";
        element.className = "mission-map-label";
        const marker = new window.maplibregl.Marker({ element, anchor: "center", offset: descriptor.offset })
          .setLngLat(descriptor.coordinate)
          .addTo(app.map);
        record = { marker, element, shape: descriptor.shape, properties: descriptor.properties, coordinate: descriptor.coordinate };
        element.addEventListener("mouseenter", () => {
          app.markerHoverActive = true;
          showMissionPopup(record.properties, record.coordinate);
        });
        element.addEventListener("mouseleave", () => {
          app.markerHoverActive = false;
          hideMissionPopup();
        });
        element.addEventListener("focus", () => {
          app.markerHoverActive = true;
          showMissionPopup(record.properties, record.coordinate);
        });
        element.addEventListener("blur", () => {
          app.markerHoverActive = false;
          hideMissionPopup();
        });
        element.addEventListener("click", (event) => {
          event.stopPropagation();
          selectInputMission(record.properties?.inputMissionID, { fit: false });
          showMissionPopup(record.properties, record.coordinate);
        });
        app.missionLabels.set(descriptor.key, record);
      }
      record.shape = descriptor.shape;
      record.properties = descriptor.properties;
      record.coordinate = descriptor.coordinate;
      record.marker.setLngLat(descriptor.coordinate);
      record.marker.setOffset(descriptor.offset);
      const historyPrefix = descriptor.properties.isHistorical ? "완료 " : "";
      record.element.textContent = `${historyPrefix}M${descriptor.properties.sequence ?? "-"} ${descriptor.shape} ${descriptor.properties.coverageLabel || "-"}`;
      record.element.title = descriptor.properties.isHistorical
        ? `완료 계획 ${descriptor.properties.historyPlanID || "-"} · 임무 ${descriptor.properties.sequence ?? "-"}`
        : `임무 ${descriptor.properties.sequence ?? "-"} 상세 보기`;
      record.element.dataset.shape = descriptor.shape;
      record.element.dataset.status = descriptor.properties.statusTone || "pending";
      record.element.dataset.quality = descriptor.properties.gsdState || "unknown";
      record.element.dataset.focus = descriptor.properties.focusRole || "normal";
      record.element.setAttribute(
        "aria-pressed",
        String(descriptor.properties.focusRole === "selected"),
      );
      record.element.hidden = descriptor.shape === "AREA" ? !app.layerVisibility.areas : !app.layerVisibility.corridors;
    }
  };

  const syncMissionMapData = () => {
    if (!app.mapLoaded || !app.missionGeometry) return;
    const areas = applyMissionDisplayScope(
      enrichMissionCollection(app.missionGeometry.areas, "AREA"),
    );
    const inputLines = applyMissionDisplayScope(
      enrichMissionCollection(app.missionGeometry.inputLines, "LINE"),
    );
    const corridors = applyMissionDisplayScope(
      enrichMissionCollection(app.missionGeometry.corridors, "LINE"),
    );
    const remainingAreas = applyMissionDisplayScope(
      enrichRemainingAreaCollection(app.missionGeometry.remainingAreas),
      { detail: true },
    );
    const coverageDepth = applyMissionDisplayScope(
      enrichMissionCollection(app.missionGeometry.coverageDepth, "AREA"),
      { detail: true },
    );
    const coveragePassAttribution = applyMissionDisplayScope(
      app.missionGeometry.coveragePassAttribution,
      { detail: true },
    );
    const paths = applyMissionDisplayScope(app.missionGeometry.paths, { detail: true });
    setSourceData(SOURCE_IDS.areas, areas);
    setSourceData(SOURCE_IDS.remainingAreas, remainingAreas);
    setSourceData(SOURCE_IDS.coverageDepth, coverageDepth);
    setSourceData(SOURCE_IDS.coveragePassAttribution, coveragePassAttribution);
    setSourceData(SOURCE_IDS.corridors, mergeCollections(inputLines, corridors));
    setSourceData(SOURCE_IDS.paths, paths);
    setSourceData(
      SOURCE_IDS.optionAssignments,
      applyMissionDisplayScope(app.optionAssignmentGeojson, { detail: true }),
    );
    syncMissionLabels(areas, inputLines);
    syncMissionFocusControls();
  };

  const applyMission = (payload) => {
    if (!payload || payload.ok === false) return;
    app.mission = payload;
    app.missionSignature = String(firstDefined(payload.signature, app.missionSignature, ""));
    app.planID = String(firstDefined(payload.planID, payload.planId, app.state?.summary?.planID, app.planID, "-"));
    els.topPlanId.textContent = app.planID;

    const geojson = payload.geojson || {};
    const areas = normalizeGeoJson(geojson.inputAreas, "Polygon");
    const inputLines = normalizeGeoJson(geojson.inputLines, "LineString");
    const corridors = normalizeGeoJson(geojson.lineCorridors, "Polygon");
    const paths = normalizeGeoJson(geojson.paths, "LineString");
    const remainingAreas = normalizeGeoJson(geojson.remainingAreas, "Polygon");
    const coverageDepth = normalizeGeoJson(geojson.coverageDepth, "Polygon");
    const coveragePassAttribution = normalizeGeoJson(geojson.coveragePassAttribution, "Polygon");

    app.missionGeometry = { areas, inputLines, corridors, paths, remainingAreas, coverageDepth, coveragePassAttribution };
    syncMissionMapData();

    const bounds = normalizeBounds(payload.bounds);
    if (bounds && app.mapLoaded && !app.mapInteracted && !app.followID) {
      app.map.fitBounds(bounds, { padding: 54, duration: 650, maxZoom: 14 });
    }
  };

  const normalizeVehicle = (raw, index) => {
    const id = String(firstDefined(raw?.aircraftID, raw?.aircraftId, raw?.id, raw?.label, `UAV-${index + 1}`));
    const health = healthState(firstDefined(raw?.health, raw?.systemHealth, raw?.alive));
    const payloadHealth = healthState(firstDefined(raw?.payloadHealth, raw?.sensorHealth, raw?.payload?.health));
    return {
      raw,
      id,
      label: String(firstDefined(raw?.label, raw?.name, id)),
      kind: String(firstDefined(raw?.kind, raw?.type, raw?.aircraftType, "UAV")),
      lat: numberOrNull(firstDefined(raw?.lat, raw?.latitude, raw?.coordinate?.lat, raw?.coordinate?.latitude)),
      lon: numberOrNull(firstDefined(raw?.lon, raw?.lng, raw?.longitude, raw?.coordinate?.lon, raw?.coordinate?.longitude)),
      alt: numberOrNull(firstDefined(raw?.alt, raw?.altitude, raw?.coordinate?.alt, raw?.coordinate?.altitude)),
      speed: numberOrNull(firstDefined(raw?.speed, raw?.groundSpeed, raw?.velocity?.speed)),
      heading: numberOrNull(firstDefined(raw?.heading, raw?.yaw, raw?.velocity?.heading)) ?? 0,
      roll: numberOrNull(raw?.roll),
      pitch: numberOrNull(raw?.pitch),
      fuel: asPercent(firstDefined(raw?.fuel, raw?.fuelPercent, raw?.remainingFuel)),
      health,
      payloadHealth,
      flying: boolValue(raw?.flying),
      filming: boolValue(raw?.filming),
      waypointID: firstDefined(raw?.currentWaypointID, raw?.waypointID, raw?.currentWaypointId),
      inputID: firstDefined(raw?.currentInputID, raw?.inputID, raw?.currentInputId),
      missionID: firstDefined(raw?.currentMissionID, raw?.missionID, raw?.currentMissionId),
      footprint: normalizeFootprint(raw?.footprint),
      quality: raw?.quality || {},
      coverage: raw?.coverage,
      color: colorForVehicle(id, index),
    };
  };

  const normalizeFootprint = (footprint) => {
    if (!Array.isArray(footprint)) return [];
    const points = footprint.map((point) => {
      if (isCoordinate(point)) return [Number(point[0]), Number(point[1])];
      const lon = numberOrNull(firstDefined(point?.lon, point?.lng, point?.longitude));
      const lat = numberOrNull(firstDefined(point?.lat, point?.latitude));
      return lon !== null && lat !== null ? [lon, lat] : null;
    }).filter(Boolean);
    return closeRing(points);
  };

  const appendBounded = (array, value, limit) => {
    array.push(value);
    if (array.length > limit) array.splice(0, array.length - limit);
  };

  const footprintTrailStyle = (index, historyLength) => {
    const length = Math.max(1, Number(historyLength) || 1);
    const ageFromNewest = Math.max(0, length - 1 - Number(index || 0));
    if (ageFromNewest < FOOTPRINT_RECENT_STRONG_COUNT) {
      const strength = 1 - (ageFromNewest / FOOTPRINT_RECENT_STRONG_COUNT);
      return {
        fillOpacity: 0.055 + (strength * 0.085),
        lineOpacity: 0.28 + (strength * 0.52),
        lineWidth: 0.7 + (strength * 0.5),
        isRecent: 1,
        recencyRank: ageFromNewest + 1,
      };
    }

    // Preserve older coverage context without letting dozens of overlapping
    // polygons turn the whole corridor into one opaque color band.
    const oldCount = Math.max(1, length - FOOTPRINT_RECENT_STRONG_COUNT);
    const oldPosition = Math.max(0, Math.min(1, Number(index || 0) / oldCount));
    return {
      fillOpacity: 0.0015 + (oldPosition * 0.0025),
      lineOpacity: 0.012 + (oldPosition * 0.018),
      lineWidth: 0.3,
      isRecent: 0,
      recencyRank: ageFromNewest + 1,
    };
  };

  const updateHistories = (vehicles, sampleTimestamp) => {
    const activeIDs = new Set(vehicles.map((vehicle) => vehicle.id));
    vehicles.forEach((vehicle) => {
      if (vehicle.lon !== null && vehicle.lat !== null) {
        const track = app.tracks.get(vehicle.id) || [];
        const last = track[track.length - 1];
        if (!last || Math.abs(last[0] - vehicle.lon) > 0.000001 || Math.abs(last[1] - vehicle.lat) > 0.000001) {
          appendBounded(track, [vehicle.lon, vehicle.lat], TRACK_LIMIT);
        }
        app.tracks.set(vehicle.id, track);
      }

      if (vehicle.filming === true && vehicle.footprint.length >= 4 && sampleTimestamp !== null) {
        const history = app.footprints.get(vehicle.id) || [];
        const lastSample = app.lastFootprintSample.get(vehicle.id);
        if (lastSample === undefined || sampleTimestamp - lastSample >= FOOTPRINT_SAMPLE_MS) {
          appendBounded(history, vehicle.footprint, FOOTPRINT_LIMIT);
          app.lastFootprintSample.set(vehicle.id, sampleTimestamp);
        }
        app.footprints.set(vehicle.id, history);
      }
    });

    for (const id of app.markers.keys()) {
      if (!activeIDs.has(id)) {
        app.markers.get(id)?.remove();
        app.markers.delete(id);
      }
    }
  };

  const syncVehicleMapData = (vehicles) => {
    const trackFeatures = [];
    const footprintFeatures = [];
    const trailFeatures = [];
    let trackPoints = 0;
    let trailCount = 0;

    vehicles.forEach((vehicle) => {
      const track = app.tracks.get(vehicle.id) || [];
      trackPoints += track.length;
      if (track.length >= 2) {
        trackFeatures.push({
          type: "Feature",
          properties: { aircraftID: vehicle.id, color: vehicle.color },
          geometry: { type: "LineString", coordinates: track },
        });
      }
      if (vehicle.lon !== null && vehicle.lat !== null) {
        trackFeatures.push({
          type: "Feature",
          properties: {
            aircraftID: vehicle.id,
            color: vehicle.color,
            heading: numberOrNull(vehicle.heading) ?? 0,
            icon: vehicleHeadingIconID(vehicle.color),
          },
          geometry: { type: "Point", coordinates: [vehicle.lon, vehicle.lat] },
        });
      }
      if (vehicle.filming === true && vehicle.footprint.length >= 4) {
        footprintFeatures.push({
          type: "Feature",
          properties: { aircraftID: vehicle.id, color: vehicle.color },
          geometry: { type: "Polygon", coordinates: [vehicle.footprint] },
        });
      }
      const history = app.footprints.get(vehicle.id) || [];
      history.forEach((footprint, index) => {
        const trailStyle = footprintTrailStyle(index, history.length);
        trailCount += 1;
        trailFeatures.push({
          type: "Feature",
          properties: {
            aircraftID: vehicle.id,
            color: vehicle.color,
            ...trailStyle,
          },
          geometry: { type: "Polygon", coordinates: [footprint] },
        });
      });
    });

    setSourceData(SOURCE_IDS.tracks, featureCollection(trackFeatures));
    setSourceData(SOURCE_IDS.footprints, featureCollection(footprintFeatures));
    setSourceData(SOURCE_IDS.footprintTrails, featureCollection(trailFeatures));
    els.historyStatus.textContent = `궤적 ${trackPoints.toLocaleString("ko-KR")} · 촬영폭 ${trailCount.toLocaleString("ko-KR")}`;
    syncMarkers(vehicles);
  };

  const syncMarkers = (vehicles) => {
    if (!app.mapLoaded) return;
    vehicles.forEach((vehicle) => {
      if (vehicle.lon === null || vehicle.lat === null) return;
      let marker = app.markers.get(vehicle.id);
      if (!marker) {
        const element = document.createElement("div");
        element.className = "vehicle-marker vehicle-label-marker";
        element.style.setProperty("--vehicle-color", vehicle.color);
        element.innerHTML = `<span class="vehicle-marker-label"></span>`;
        marker = new window.maplibregl.Marker({ element, anchor: "center" })
          .setLngLat([vehicle.lon, vehicle.lat])
          .addTo(app.map);
        app.markers.set(vehicle.id, marker);
      }
      marker.setLngLat([vehicle.lon, vehicle.lat]);
      marker.getElement().querySelector(".vehicle-marker-label").textContent = vehicle.label;
      marker.getElement().title = `${vehicle.label} · ${formatNumber(vehicle.alt)} m`;
    });

    if (app.followID) {
      const vehicle = vehicles.find((entry) => entry.id === app.followID);
      if (vehicle && vehicle.lon !== null && vehicle.lat !== null) {
        app.map.easeTo({ center: [vehicle.lon, vehicle.lat], duration: 280, essential: true });
      }
    }
  };

  const normalizeTargets = (targets) => {
    return featureCollection(toArray(targets).map((target, index) => {
      const existing = normalizeFeature(target, index, "Point");
      if (existing) return existing;
      const lon = numberOrNull(firstDefined(target?.lon, target?.lng, target?.longitude, target?.coordinate?.lon, target?.coordinate?.longitude));
      const lat = numberOrNull(firstDefined(target?.lat, target?.latitude, target?.coordinate?.lat, target?.coordinate?.latitude));
      if (lon === null || lat === null) return null;
      return {
        type: "Feature",
        properties: { ...target, _index: index },
        geometry: { type: "Point", coordinates: [lon, lat] },
      };
    }));
  };

  const commandCoordinate = (command) => {
    const position = command?.position || {};
    const latitude = numberOrNull(firstDefined(position?.latitude, position?.lat));
    const longitude = numberOrNull(firstDefined(position?.longitude, position?.lon, position?.lng));
    return { latitude, longitude };
  };

  const commandColor = (command) => {
    const commandType = numberOrNull(command?.commandModeType);
    if (commandType === 1) return "#4f94d4";
    if (commandType === 2) return "#ed9a3d";
    return "#a86cce";
  };

  const commandLines = (command) => {
    const lines = [`${formatPreciseKstTime(command?.timestamp)} · ${firstDefined(command?.uavLabel, `UAV ${firstDefined(command?.aircraftID, "-")}`)}`];
    if (command?.flightCommandText) lines.push(`비행 ${command.flightCommandText}`);
    if (command?.filmingCommandText) lines.push(`촬영 ${command.filmingCommandText}`);
    if (lines.length === 1) lines.push(firstDefined(command?.commandModeTypeName, "통제 명령"));
    return lines;
  };

  const commandKey = (command, index = 0) => {
    const coordinate = commandCoordinate(command);
    return [
      firstDefined(command?.messageTimestamp, command?.timestamp, index),
      firstDefined(command?.aircraftID, "-"),
      firstDefined(command?.flightMode, "-"),
      firstDefined(command?.filmingMode, "-"),
      coordinate.latitude ?? "-",
      coordinate.longitude ?? "-",
    ].join(":");
  };

  const normalizeUavCommandPoints = (commands) => featureCollection(
    toArray(commands).map((command, index) => {
      const coordinate = commandCoordinate(command);
      if (coordinate.latitude === null || coordinate.longitude === null) return null;
      return {
        type: "Feature",
        properties: {
          commandIndex: index,
          commandModeType: numberOrNull(command?.commandModeType),
          color: commandColor(command),
        },
        geometry: { type: "Point", coordinates: [coordinate.longitude, coordinate.latitude] },
      };
    }).filter(Boolean)
  );

  const clearCommandLabels = () => {
    for (const record of app.commandLabels.values()) record.marker.remove();
    app.commandLabels.clear();
    app.commandLabelRenderKey = "";
  };

  const rectanglesOverlap = (left, right, gap = 4) => !(
    left.right + gap <= right.left
    || left.left >= right.right + gap
    || left.bottom + gap <= right.top
    || left.top >= right.bottom + gap
  );

  const commandLabelOccupiedRects = () => {
    const container = app.map.getContainer();
    const mapRect = container.getBoundingClientRect();
    return [...container.querySelectorAll(
      ".layer-control, .coverage-depth-control, .follow-control, .interpolation-control, "
      + ".map-footer, .maplibregl-ctrl, .mission-map-label, .discovery-map-label, .map-message:not([hidden])"
    )].map((element) => {
      const rect = element.getBoundingClientRect();
      return {
        left: rect.left - mapRect.left,
        top: rect.top - mapRect.top,
        right: rect.right - mapRect.left,
        bottom: rect.bottom - mapRect.top,
      };
    }).filter((rect) => rect.right > 0 && rect.bottom > 0 && rect.left < mapRect.width && rect.top < mapRect.height);
  };

  const commandLabelPlacement = (point, width, height, occupied, mapWidth, mapHeight) => {
    const candidates = [
      [12, -height - 9],
      [12, 9],
      [-width - 12, -height - 9],
      [-width - 12, 9],
      [10, -height / 2],
      [-width - 10, -height / 2],
    ];
    for (const [offsetX, offsetY] of candidates) {
      const rect = {
        left: point.x + offsetX,
        top: point.y + offsetY,
        right: point.x + offsetX + width,
        bottom: point.y + offsetY + height,
      };
      if (rect.left < 5 || rect.top < 5 || rect.right > mapWidth - 5 || rect.bottom > mapHeight - 5) continue;
      if (occupied.some((other) => rectanglesOverlap(rect, other))) continue;
      return { offset: [offsetX, offsetY], rect };
    }
    return null;
  };

  const syncCommandLabels = (commands, force = false) => {
    if (!app.mapLoaded || !app.layerVisibility.uavCommands) {
      clearCommandLabels();
      return;
    }
    const candidates = toArray(commands).slice(0, COMMAND_LABEL_LIMIT);
    const renderKey = candidates.map(commandKey).join("|");
    if (!force && renderKey === app.commandLabelRenderKey) return;
    clearCommandLabels();
    app.commandLabelRenderKey = renderKey;
    if (!renderKey) return;

    const container = app.map.getContainer();
    const occupied = commandLabelOccupiedRects();
    const mapWidth = container.clientWidth;
    const mapHeight = container.clientHeight;
    candidates.forEach((command, index) => {
      const coordinate = commandCoordinate(command);
      if (coordinate.latitude === null || coordinate.longitude === null) return;
      const point = app.map.project([coordinate.longitude, coordinate.latitude]);
      if (point.x < 0 || point.y < 0 || point.x > mapWidth || point.y > mapHeight) return;
      const lines = commandLines(command);
      const width = clamp(Math.max(...lines.map((line) => line.length)) * 6 + 14, 94, 174);
      const height = 8 + lines.length * 12;
      const placement = commandLabelPlacement(point, width, height, occupied, mapWidth, mapHeight);
      if (!placement) return;

      const element = document.createElement("div");
      element.className = "uav-command-map-label";
      element.setAttribute("aria-hidden", "true");
      element.style.setProperty("--command-color", commandColor(command));
      lines.forEach((line, lineIndex) => {
        const row = document.createElement("span");
        if (lineIndex === 0) row.className = "uav-command-label-head";
        row.textContent = line;
        element.append(row);
      });
      const marker = new window.maplibregl.Marker({
        element,
        anchor: "top-left",
        offset: placement.offset,
      }).setLngLat([coordinate.longitude, coordinate.latitude]).addTo(app.map);
      app.commandLabels.set(commandKey(command, index), { marker, element });
      occupied.push(placement.rect);
    });
  };

  const syncAllMapData = () => {
    if (!app.mapLoaded) return;
    syncMissionMapData();
    setSourceData(SOURCE_IDS.optionAssignments, app.optionAssignmentGeojson);
    const vehicles = [...app.vehicles.values()];
    syncVehicleMapData(vehicles);
    setSourceData(SOURCE_IDS.uavCommands, normalizeUavCommandPoints(app.uavCommands));
    syncCommandLabels(app.uavCommands);
    setSourceData(SOURCE_IDS.detectionFootprints, normalizeDetectionFootprints(app.discoveries));
    syncDiscoveryLabels(app.discoveries);
    setSourceData(SOURCE_IDS.targets, normalizeTargets(app.state?.targets));
  };

  const renderFollowSegments = (vehicles) => {
    const ids = vehicles.map((vehicle) => vehicle.id).join("|");
    if (els.followSegments.dataset.ids === ids) return;
    if (app.followID && !vehicles.some((vehicle) => vehicle.id === app.followID)) app.followID = "";
    els.followSegments.dataset.ids = ids;
    els.followSegments.innerHTML = [
      `<button type="button" class="segment${app.followID ? "" : " is-active"}" data-follow-id="" aria-pressed="${app.followID ? "false" : "true"}">자유</button>`,
      ...vehicles.map((vehicle) => `<button type="button" class="segment${app.followID === vehicle.id ? " is-active" : ""}" data-follow-id="${escapeHtml(vehicle.id)}" aria-pressed="${app.followID === vehicle.id}">${escapeHtml(shortVehicleLabel(vehicle.label))}</button>`),
    ].join("");
  };

  const shortVehicleLabel = (label) => {
    const text = String(label);
    return text.length > 7 ? text.slice(0, 7) : text;
  };

  const setFollow = (id) => {
    app.followID = id;
    app.mapInteracted = Boolean(id) || app.mapInteracted;
    els.followSegments.querySelectorAll("[data-follow-id]").forEach((button) => {
      const active = button.dataset.followId === id;
      button.classList.toggle("is-active", active);
      button.setAttribute("aria-pressed", String(active));
    });
    if (id) syncMarkers([...app.vehicles.values()]);
  };

  const getCoverageValues = (state, vehicles, parts) => {
    const coverage = state?.coverage || {};
    const summary = state?.summary || {};
    const partValues = parts.map((part) => asPercent(firstDefined(part.coverage, part.coverageRate, part.progress))).filter((value) => value !== null);
    const vehicleValues = vehicles.map((vehicle) => asPercent(vehicle.coverage)).filter((value) => value !== null);
    const overall = asPercent(firstDefined(coverage.overall, coverage.total, coverage.percent, summary.coverage, summary.coverageRate, average(partValues), average(vehicleValues)));
    const lineParts = parts.filter((part) => /line|corridor|선형|회랑/i.test(String(firstDefined(part.type, part.missionType, part.shape, part.geometryType, ""))));
    const areaParts = parts.filter((part) => /area|polygon|grid|면적|구역/i.test(String(firstDefined(part.type, part.missionType, part.shape, part.geometryType, ""))));
    const partAverage = (entries) => average(entries.map((part) => asPercent(firstDefined(part.coverage, part.coverageRate, part.progress))).filter((value) => value !== null));
    return {
      overall,
      line: asPercent(firstDefined(coverage.line, coverage.lineCoverage, coverage.linear, coverage.byType?.line, partAverage(lineParts))),
      area: asPercent(firstDefined(coverage.area, coverage.areaCoverage, coverage.areal, coverage.byType?.area, partAverage(areaParts))),
      areaSpatial: asPercent(firstDefined(coverage.areaSpatial, coverage.spatialAreaCoverage)),
    };
  };

  const average = (values) => {
    const valid = toArray(values).map(numberOrNull).filter((value) => value !== null);
    return valid.length ? valid.reduce((sum, value) => sum + value, 0) / valid.length : null;
  };

  const extractQualityMetrics = (state, vehicles) => {
    const quality = state?.quality || {};
    const vehicleQuality = vehicles.map((vehicle) => vehicle.quality || {});
    const pickAverage = (keys) => average(vehicleQuality.map((entry) => firstDefined(...keys.map((key) => entry[key]))));
    return {
      gsd: numberOrNull(firstDefined(quality.gsd, quality.currentGSD, quality.meanGSD, quality.avgGSD, pickAverage(["gsd", "currentGSD", "meanGSD"]))),
      overlap: asPercent(firstDefined(quality.overlap, quality.overlapRate, quality.meanOverlap, pickAverage(["overlap", "overlapRate"]))),
      quality: asPercent(firstDefined(quality.score, quality.qualityScore, quality.satisfaction, pickAverage(["score", "qualityScore", "satisfaction"]))),
    };
  };

  const recordTrends = (coverage, quality) => {
    const record = (key, value) => {
      if (value === null) return;
      appendBounded(app.trends[key], value, TREND_LIMIT);
    };
    record("coverage", coverage.overall);
    record("line", coverage.line);
    record("area", coverage.area);
    record("gsd", quality.gsd);
    record("overlap", quality.overlap);
    record("quality", quality.quality);
  };

  const renderState = (state) => {
    const rawVehicles = toArray(state?.vehicles).slice(0, 12);
    const vehicles = rawVehicles.map(normalizeVehicle);
    const sampleTimestamp = numberOrNull(firstDefined(state?.signal?.timestampUnix, state?.signal?.timestamp));
    app.vehicles = new Map(vehicles.map((vehicle) => [vehicle.id, vehicle]));
    updateHistories(vehicles, sampleTimestamp);
    syncVehicleMapData(vehicles);
    setSourceData(SOURCE_IDS.targets, normalizeTargets(state?.targets));
    renderFollowSegments(vehicles);

    const parts = toArray(state?.missionParts);
    const historyParts = toArray(state?.missionPartHistory);
    const normalizedParts = parts.map(normalizeMissionPart);
    app.missionParts = new Map(normalizedParts
      .filter((part) => part.inputMissionID !== undefined && part.inputMissionID !== null)
      .map((part) => [String(part.inputMissionID), part]));
    app.missionRows = [...historyParts, ...parts];
    syncCurrentMissionSelection(state, normalizedParts);
    syncMissionMapData();
    const coverage = getCoverageValues(state, vehicles, parts);
    const quality = extractQualityMetrics(state, vehicles);
    if (sampleTimestamp !== app.lastTrendTimestamp) {
      recordTrends(coverage, quality);
      app.lastTrendTimestamp = sampleTimestamp;
    }

    renderTopStatus(state);
    renderKpis(state, vehicles, coverage);
    renderVehicles(vehicles);
    renderOptionAssignments(state?.optionAssignments);
    renderMissionParts(app.missionRows);
    syncMissionFocusControls();
    renderCoverage(coverage);
    renderQuality(quality);
    const commands = toArray(state?.uavCommands);
    app.uavCommands = commands;
    setSourceData(SOURCE_IDS.uavCommands, normalizeUavCommandPoints(commands));
    syncCommandLabels(commands);
    renderUavCommands(commands);
    const discoveries = [...toArray(state?.discoveries)].sort(
      (left, right) => discoveryTimestampValue(right) - discoveryTimestampValue(left)
    );
    app.discoveries = discoveries;
    setSourceData(SOURCE_IDS.detectionFootprints, normalizeDetectionFootprints(discoveries));
    syncDiscoveryLabels(discoveries);
    renderDiscoveries(discoveries);
    renderEvents(toArray(state?.events));
  };

  const renderTopStatus = (state) => {
    const scenario = firstDefined(state?.scenario?.name, state?.scenario?.label, state?.scenario?.id, state?.scenario, "시나리오 미지정");
    els.scenarioName.textContent = typeof scenario === "object" ? "시나리오 실행 중" : String(scenario);
    els.topSignal.textContent = signalLabel(state?.signal);
    const signalTimestamp = firstDefined(state?.signal?.timestampUnix, state?.generatedAt);
    const signalStatus = String(state?.signal?.status || "WAIT").toUpperCase();
    const signalTone = signalStatus === "LIVE" ? "ok" : signalStatus === "STALE" ? "warn" : "bad";
    els.topUpdated.textContent = formatTime(signalTimestamp || new Date());
    els.connectionState.dataset.tone = signalTone;
    els.connectionLabel.textContent = signalStatus === "LIVE" ? "실시간 연결" : signalStatus === "STALE" ? "수신 지연" : "데이터 대기";
    els.dataFreshness.dataset.tone = signalTone;
    els.dataFreshness.textContent = signalTimestamp ? formatAge(signalTimestamp) : "0401 대기";
    renderInterpolationStatus(state?.signal?.rateHz);

    const summaryPlanID = firstDefined(state?.summary?.planID, state?.summary?.planId, state?.summary?.currentPlanID);
    if (summaryPlanID !== undefined) {
      app.planID = String(summaryPlanID);
      els.topPlanId.textContent = app.planID;
    }
  };

  const renderKpis = (state, vehicles, coverage) => {
    const active = vehicles.filter((vehicle) => vehicle.flying !== false && vehicle.health.tone !== "bad").length;
    const filming = vehicles.filter((vehicle) => vehicle.filming === true).length;
    const healthy = vehicles.filter((vehicle) => vehicle.health.tone === "ok" && vehicle.payloadHealth.tone !== "bad").length;
    const summary = state?.summary || {};
    const activeValue = numberOrNull(firstDefined(summary.activeVehicles, summary.flyingVehicles, active)) ?? active;
    const filmingValue = numberOrNull(firstDefined(summary.filmingVehicles, summary.recordingVehicles, filming)) ?? filming;
    const healthyValue = numberOrNull(firstDefined(summary.healthyVehicles, healthy)) ?? healthy;
    els.kpiGrid.innerHTML = `
      <div class="kpi"><span>활성 기체</span><strong>${formatNumber(activeValue)}</strong><small>전체 ${formatNumber(vehicles.length)}대</small></div>
      <div class="kpi"><span>촬영 기체</span><strong>${formatNumber(filmingValue)}</strong><small>0401 촬영 상태</small></div>
      <div class="kpi"><span>종합 진척</span><strong>${formatPercent(coverage.overall, 1)}</strong><small>계획 ${escapeHtml(app.planID)}</small></div>
      <div class="kpi"><span>정상 기체</span><strong>${formatNumber(healthyValue)}</strong><small>기체·탑재체 정상</small></div>
    `;
  };

  const renderVehicles = (vehicles) => {
    els.vehicleCount.textContent = `${vehicles.length}대`;
    if (!vehicles.length) {
      els.vehicleList.innerHTML = '<div class="empty-state">수신된 기체 상태가 없습니다.</div>';
      return;
    }
    els.vehicleList.innerHTML = vehicles.slice(0, 6).map((vehicle) => {
      const flightLabel = vehicle.flying === true ? "비행" : vehicle.flying === false ? "지상" : "상태 대기";
      const badgeTone = vehicle.health.tone === "bad" ? "bad" : vehicle.filming ? "" : "warn";
      const fuel = vehicle.fuel ?? 0;
      const fuelColor = fuel < 20 ? "#c9473b" : fuel < 40 ? "#d8952f" : vehicle.color;
      return `
        <article class="vehicle-card" data-tone="${vehicle.health.tone}" style="--vehicle-color:${vehicle.color}">
          <div class="vehicle-identity">
            <div class="vehicle-name-row">
              <span class="vehicle-name" title="${escapeHtml(vehicle.label)}">${escapeHtml(vehicle.label)}</span>
              <span class="state-badge ${badgeTone}">${vehicle.filming ? "촬영" : flightLabel}</span>
            </div>
            <div class="vehicle-kind">${escapeHtml(vehicle.kind)} · WP ${escapeHtml(firstDefined(vehicle.waypointID, "-"))}</div>
          </div>
          <div class="vehicle-flight">
            <div class="metric-pair"><span>고도</span><strong>${formatNumber(vehicle.alt)} m</strong></div>
            <div class="metric-pair"><span>속도</span><strong>${formatNumber(vehicle.speed, 1)} m/s</strong></div>
            <div class="metric-pair"><span>방위</span><strong>${formatNumber(vehicle.heading)}°</strong></div>
          </div>
          <div class="vehicle-health">
            <div class="fuel-line">
              <div class="fuel-bar"><span style="width:${fuel}%;background:${fuelColor}"></span></div>
              <strong>${vehicle.fuel === null ? "-" : `${formatNumber(vehicle.fuel)}%`}</strong>
            </div>
            <div class="health-line"><span>기체 ${vehicle.health.label}</span><span>탑재체 ${vehicle.payloadHealth.label}</span></div>
          </div>
        </article>
      `;
    }).join("");
  };

  const selectedOptionAssignment = (snapshot = app.state?.optionAssignments) => {
    const options = toArray(snapshot?.options);
    return options.find((option) => String(option?.missionPlanID) === String(app.selectedOptionPlanID)) || null;
  };

  const assignmentShapeText = (assignment) => {
    const labels = [];
    const areaCount = numberOrNull(assignment?.areaCount) ?? 0;
    const lineCount = numberOrNull(assignment?.lineCount) ?? 0;
    if (areaCount > 0) labels.push(`면적 ${formatNumber(areaCount)}개`);
    if (lineCount > 0) labels.push(`선형 ${formatNumber(lineCount)}개`);
    return labels.join(" · ") || "영역 없음";
  };

  const assignmentCenterText = (center) => {
    const lat = numberOrNull(center?.latitude);
    const lon = numberOrNull(center?.longitude);
    return lat !== null && lon !== null ? `${lat.toFixed(5)}, ${lon.toFixed(5)}` : "중심좌표 없음";
  };

  const renderOptionAssignments = (snapshot) => {
    const options = toArray(snapshot?.options);
    els.optionAssignmentCount.textContent = `옵션 ${options.length}개`;
    if (!options.length) {
      app.optionAssignmentSignature = "";
      app.optionAssignmentRenderKey = "";
      app.selectedOptionPlanID = null;
      app.optionAssignmentGeojson = EMPTY_FC;
      setSourceData(SOURCE_IDS.optionAssignments, EMPTY_FC);
      els.optionAssignmentTabs.innerHTML = '<span class="empty-state compact">0701 후보 옵션을 기다리는 중입니다.</span>';
      els.optionAssignmentList.innerHTML = '<div class="empty-state compact">표시할 UAV 할당 영역이 없습니다.</div>';
      return;
    }

    const signature = String(firstDefined(snapshot?.signature, ""));
    const selectedStillExists = options.some(
      (option) => String(option?.missionPlanID) === String(app.selectedOptionPlanID),
    );
    if (signature !== app.optionAssignmentSignature || !selectedStillExists) {
      const preferred = options.find((option) => boolValue(option?.recommend) === true) || options[0];
      app.selectedOptionPlanID = preferred?.missionPlanID ?? null;
      app.optionAssignmentSignature = signature;
      app.optionAssignmentRenderKey = "";
    }

    const selected = selectedOptionAssignment(snapshot) || options[0];
    const focusIDs = focusedInputMissionIDs();
    const focusKey = app.missionViewMode === "all" ? "all" : focusIDs.join(",");
    const renderKey = `${signature}|${selected?.missionPlanID ?? "-"}|${app.missionViewMode}|${focusKey}`;
    if (renderKey === app.optionAssignmentRenderKey) return;
    app.optionAssignmentRenderKey = renderKey;

    els.optionAssignmentTabs.innerHTML = options.map((option, index) => {
      const planID = firstDefined(option?.missionPlanID, "-");
      const optionID = firstDefined(option?.optionID, index + 1);
      const active = String(planID) === String(selected?.missionPlanID);
      const recommended = boolValue(option?.recommend) === true;
      return `
        <button type="button" class="assignment-option-tab${active ? " is-active" : ""}"
          data-option-plan-id="${escapeHtml(planID)}" role="tab" aria-selected="${active}">
          <span>옵션 ${escapeHtml(optionID)}</span>
          <small>PLAN ${escapeHtml(planID)}</small>
          ${recommended ? '<em>추천</em>' : ""}
        </button>
      `;
    }).join("");

    app.optionAssignmentGeojson = selected?.geojson?.type ? selected.geojson : EMPTY_FC;
    setSourceData(
      SOURCE_IDS.optionAssignments,
      applyMissionDisplayScope(app.optionAssignmentGeojson, { detail: true }),
    );

    if (selected?.available === false) {
      els.optionAssignmentList.innerHTML = `<div class="empty-state compact">MissionPlan ${escapeHtml(selected?.missionPlanID ?? "-")} 산출물을 기다리는 중입니다.</div>`;
      return;
    }
    const focusedSet = new Set(focusIDs);
    const filterAssignments = app.missionViewMode !== "all" && focusedSet.size > 0;
    const aircraftRows = toArray(selected?.aircraft).map((aircraft) => {
      const assignments = toArray(aircraft?.assignments);
      const visibleAssignments = filterAssignments
        ? assignments.filter((assignment) => focusedSet.has(missionIDKey(assignment?.inputMissionID)))
        : assignments;
      return { ...aircraft, assignments: visibleAssignments };
    }).filter((aircraft) => !filterAssignments || aircraft.assignments.length);

    if (filterAssignments && !aircraftRows.length) {
      els.optionAssignmentList.innerHTML = `<div class="empty-state compact">집중 임무 ${focusIDs.map((value) => `M${escapeHtml(value)}`).join(" · ")}에 대한 후보 할당이 없습니다.</div>`;
      return;
    }

    els.optionAssignmentList.innerHTML = aircraftRows.map((aircraft, index) => {
      const aircraftID = numberOrNull(aircraft?.aircraftID);
      const label = firstDefined(aircraft?.label, aircraftID !== null ? `UAV${aircraftID - 3}` : `UAV${index + 1}`);
      const color = String(firstDefined(aircraft?.color, VEHICLE_COLORS[index % VEHICLE_COLORS.length]));
      const assignments = toArray(aircraft?.assignments);
      const visibleAreaCount = assignments.reduce(
        (sum, assignment) => sum + (numberOrNull(assignment?.areaCount) ?? 0),
        0,
      );
      const visibleLineCount = assignments.reduce(
        (sum, assignment) => sum + (numberOrNull(assignment?.lineCount) ?? 0),
        0,
      );
      const assignmentRows = assignments.length
        ? assignments.map((assignment) => `
            <li>
              <span>입력 ${escapeHtml(firstDefined(assignment?.inputMissionID, "-"))}</span>
              <strong>${escapeHtml(assignmentShapeText(assignment))}</strong>
              <small>${escapeHtml(assignmentCenterText(assignment?.center))}</small>
            </li>
          `).join("")
        : '<li class="assignment-empty">영역형 임무 할당 없음</li>';
      return `
        <article class="assignment-uav-card" data-assignment-aircraft-id="${escapeHtml(aircraftID ?? "")}" style="--assignment-color:${escapeHtml(color)}">
          <header>
            <span class="assignment-uav-name">${escapeHtml(label)}</span>
            <strong>${escapeHtml(assignmentShapeText({ areaCount: visibleAreaCount, lineCount: visibleLineCount }))}</strong>
          </header>
          <ul>${assignmentRows}</ul>
        </article>
      `;
    }).join("");
  };

  const focusOptionAssignment = (aircraftID) => {
    const focusIDs = focusedInputMissionIDs();
    if (app.missionViewMode !== "all" && focusIDs.length === 1) {
      focusMissionOnMap(focusIDs[0]);
      return;
    }
    const selected = selectedOptionAssignment();
    const aircraft = toArray(selected?.aircraft).find(
      (item) => String(item?.aircraftID) === String(aircraftID),
    );
    const bounds = normalizeBounds(aircraft?.bounds);
    if (bounds && app.mapLoaded) {
      app.map.fitBounds(bounds, { padding: 78, duration: 450, maxZoom: 15 });
    }
  };

  const normalizeMissionPart = (part, index) => {
    const rawStatus = firstDefined(part?.status, part?.state, part?.missionStatus, "대기");
    const status = statusState(
      rawStatus && typeof rawStatus === "object"
        ? firstDefined(rawStatus.label, rawStatus.name, rawStatus.state, "대기")
        : rawStatus
    );
    if (part?.statusTone) status.tone = String(part.statusTone);
    const coverage = asPercent(firstDefined(part?.coverage, part?.coverageRate, part?.progress));
    const coverageDetail = part?.coverageDetail || {};
    const coveragePasses = normalizeCoveragePassRows(
      firstDefined(coverageDetail.passes, part?.coveragePassDetails, part?.coverage_pass_details, []),
    );
    const coverageDepthDetails = normalizeCoverageDepthRows(
      firstDefined(
        coverageDetail.coverageDepthDetails,
        part?.coverageDepthDetails,
        part?.coverage_depth_details,
        [],
      ),
    );
    const measuredGsd = numberOrNull(firstDefined(part?.gsd, part?.measuredGsd, part?.actualGSD, part?.currentGSD, part?.quality?.gsd));
    const targetGsd = numberOrNull(firstDefined(part?.targetGSD, part?.targetGsd, part?.requiredGSD, part?.maxGSD, part?.quality?.targetGSD));
    const quality = part?.quality || {};
    let gsdSatisfied = boolValue(firstDefined(part?.gsdSatisfied, part?.gsdSatisfaction, part?.quality?.gsdSatisfied));
    if (gsdSatisfied === null && measuredGsd !== null && targetGsd !== null) gsdSatisfied = measuredGsd <= targetGsd;
    return {
      inputMissionID: firstDefined(part?.inputMissionID, part?.currentInputMissionID, part?.currentInputID, part?.inputID),
      sequence: firstDefined(part?.sequence, part?.order, part?.index, index + 1),
      type: firstDefined(part?.type, part?.missionType, part?.kind, "-") ,
      region: firstDefined(part?.region, part?.regionID, part?.areaID, part?.currentInputID, part?.inputID, "-"),
      shape: String(firstDefined(part?.shape, part?.geometryType, part?.searchShape, part?.geometry?.type, "-")).toUpperCase(),
      status,
      coverage,
      coverageDetail,
      coveragePasses,
      coverageDepthDetails,
      spatialCoverage: asPercent(firstDefined(coverageDetail.spatialPercent, part?.spatialCoveragePercent)),
      coverageRequirementsMet: boolValue(firstDefined(coverageDetail.requirementsMet, part?.isCoverageDone)) === true,
      coverageRequired: boolValue(part?.coverageRequired) === true,
      isExecutionDone: boolValue(firstDefined(part?.isExecutionDone, part?.isDone)) === true,
      measuredGsd,
      targetGsd,
      gsdSatisfied,
      qualitySamples: numberOrNull(firstDefined(quality.samples, part?.qualitySamples)) ?? 0,
      qualitySatisfaction: asPercent(firstDefined(quality.satisfactionPercent, quality.satisfaction, part?.qualitySatisfaction)),
      activeAircraftCount: numberOrNull(part?.activeAircraftCount) ?? 0,
      isCurrent: boolValue(part?.isCurrent) === true,
      isDone: boolValue(part?.isDone) === true,
      isHistorical: boolValue(part?.isHistorical) === true,
      historyPlanID: firstDefined(part?.historyPlanID, ""),
    };
  };

  const deriveCurrentInputMissionIDs = (state, parts) => {
    const values = [];
    const add = (value) => {
      const key = missionIDKey(value);
      if (key && !values.includes(key)) values.push(key);
    };
    toArray(state?.summary?.currentInputMissionIDs).forEach(add);
    [...parts]
      .filter((part) => part.isCurrent && !part.isHistorical)
      .sort((left, right) => Number(left.sequence || 0) - Number(right.sequence || 0))
      .forEach((part) => add(part.inputMissionID));
    for (const vehicle of app.vehicles.values()) add(vehicle.inputID);
    if (!values.length) {
      const pending = [...parts]
        .filter((part) => !part.isDone && !part.isHistorical)
        .sort((left, right) => Number(left.sequence || 0) - Number(right.sequence || 0))[0];
      add(pending?.inputMissionID);
    }
    return values;
  };

  const syncCurrentMissionSelection = (state, parts) => {
    app.currentInputMissionIDs = deriveCurrentInputMissionIDs(state, parts);
    const selectedKey = missionIDKey(app.selectedInputMissionID);
    if (selectedKey && !app.missionParts.has(selectedKey)) {
      app.selectedInputMissionID = null;
      if (app.missionViewMode === "selected") app.missionViewMode = "current";
    }
  };

  const syncMissionFocusControls = () => {
    const focused = new Set(focusedInputMissionIDs());
    const selectedKey = missionIDKey(app.selectedInputMissionID);
    document.querySelectorAll("[data-mission-view]").forEach((button) => {
      const active = button.dataset.missionView === app.missionViewMode;
      button.classList.toggle("is-active", active);
      button.setAttribute("aria-pressed", String(active));
      if (button.dataset.missionView === "selected") button.disabled = !selectedKey;
    });
    if (els.showMissionHistory) els.showMissionHistory.checked = app.showMissionHistory;
    if (els.missionFocusStatus) {
      if (app.missionViewMode === "selected") {
        els.missionFocusStatus.textContent = selectedKey ? `선택 M${selectedKey}` : "선택 없음";
      } else if (app.missionViewMode === "all") {
        els.missionFocusStatus.textContent = app.showMissionHistory ? "전체 + 완료 이력" : "전체 현행";
      } else {
        const current = app.currentInputMissionIDs;
        els.missionFocusStatus.textContent = current.length
          ? `현재 ${current.map((value) => `M${value}`).join(" · ")}`
          : "현재 임무 대기";
      }
    }
    if (els.missionFocusHint) {
      els.missionFocusHint.textContent = app.missionViewMode === "all"
        ? "모든 현행 임무를 같은 무게로 표시합니다."
        : "진행·촬영심도·계획경로·후보 옵션은 집중 임무만 표시합니다.";
    }
    document.querySelectorAll("#mission-parts-body tr[data-input-mission-id]").forEach((row) => {
      const active = focused.has(missionIDKey(row.dataset.inputMissionId));
      row.classList.toggle("is-selected", active);
      row.setAttribute("aria-selected", String(active));
    });
  };

  const missionGeometryCoordinates = (geometry) => {
    const result = [];
    const visit = (value) => {
      if (!Array.isArray(value)) return;
      if (isCoordinate(value)) {
        result.push([Number(value[0]), Number(value[1])]);
        return;
      }
      value.forEach(visit);
    };
    visit(geometry?.coordinates);
    return result;
  };

  const focusMissionOnMap = (inputMissionID) => {
    if (!app.mapLoaded) return;
    const targetKey = missionIDKey(inputMissionID);
    if (!targetKey) return;
    const collections = [
      app.missionGeometry?.areas,
      app.missionGeometry?.inputLines,
      app.missionGeometry?.corridors,
      app.optionAssignmentGeojson,
    ];
    const coordinates = collections.flatMap((collection) => (
      toArray(collection?.features)
        .filter((feature) => missionIDKey(featureInputMissionID(feature)) === targetKey)
        .flatMap((feature) => missionGeometryCoordinates(feature?.geometry))
    ));
    if (!coordinates.length) return;
    const bounds = coordinates.slice(1).reduce(
      (value, coordinate) => value.extend(coordinate),
      new window.maplibregl.LngLatBounds(coordinates[0], coordinates[0]),
    );
    app.mapInteracted = true;
    if (app.followID) setFollow("");
    const mapRect = app.map.getContainer().getBoundingClientRect();
    const toolbarRect = document.querySelector(".map-toolbar")?.getBoundingClientRect();
    const topPadding = toolbarRect
      ? clamp(toolbarRect.bottom - mapRect.top + 18, 86, Math.min(250, mapRect.height * 0.44))
      : 86;
    app.map.fitBounds(bounds, {
      padding: { top: topPadding, right: 76, bottom: 76, left: 76 },
      duration: 450,
      maxZoom: 15,
    });
  };

  const refreshMissionFocusedViews = () => {
    app.optionAssignmentRenderKey = "";
    syncMissionMapData();
    renderOptionAssignments(app.state?.optionAssignments);
    renderMissionParts(app.missionRows);
    syncMissionFocusControls();
  };

  const setMissionViewMode = (mode) => {
    const normalized = ["current", "selected", "all"].includes(mode) ? mode : "current";
    if (normalized === "selected" && !missionIDKey(app.selectedInputMissionID)) return;
    app.missionViewMode = normalized;
    refreshMissionFocusedViews();
  };

  const selectInputMission = (inputMissionID, { fit = true } = {}) => {
    const key = missionIDKey(inputMissionID);
    if (!key || !app.missionParts.has(key)) return;
    app.selectedInputMissionID = key;
    app.missionViewMode = "selected";
    refreshMissionFocusedViews();
    if (fit) focusMissionOnMap(key);
  };

  const renderMissionParts = (parts) => {
    const normalizedParts = parts.map(normalizeMissionPart);
    const visibleParts = normalizedParts.filter((part) => app.showMissionHistory || !part.isHistorical);
    const historyCount = normalizedParts.length - visibleParts.length;
    els.missionPartCount.textContent = app.showMissionHistory
      ? `${visibleParts.length}개`
      : `현행 ${visibleParts.length}개${historyCount ? ` · 이력 ${historyCount}` : ""}`;
    if (!visibleParts.length) {
      els.missionPartsBody.innerHTML = '<tr><td colspan="7" class="table-empty">임무 계획을 기다리는 중입니다.</td></tr>';
      return;
    }
    const focused = new Set(focusedInputMissionIDs());
    els.missionPartsBody.innerHTML = visibleParts.map((part) => {
      const gsdClass = part.gsdSatisfied === true ? "pass" : part.gsdSatisfied === false ? "fail" : "unknown";
      const hasSatisfaction = part.qualitySamples > 0 && part.qualitySatisfaction !== null;
      const satisfactionLabel = hasSatisfaction ? ` ${formatPercent(part.qualitySatisfaction, 1)}` : "";
      const gsdLabel = part.gsdSatisfied === true
        ? `충족${satisfactionLabel}`
        : part.gsdSatisfied === false
          ? `미충족${satisfactionLabel}`
          : hasSatisfaction ? `충족률${satisfactionLabel}` : "미평가";
      const gsdMeasurement = part.measuredGsd === null
        ? ""
        : `${formatNumber(part.measuredGsd, 2)} cm/px${part.targetGsd === null ? "" : ` / 기준 ${formatNumber(part.targetGsd, 2)}`}`;
      const gsdSamples = hasSatisfaction
        ? `표본 충족률 ${formatPercent(part.qualitySatisfaction, 1)} · ${part.qualitySamples}회`
        : "";
      const gsdTitle = [gsdMeasurement, gsdSamples].filter(Boolean).join(" · ") || gsdLabel;
      const passTitle = part.coveragePasses.map((row) => (
        `${coveragePassLabel(row.coveragePass)} ${coveragePassPercentLabel(row)} · `
        + `${formatNumber(row.actualCoveredM2, 1)}/${formatNumber(row.requiredM2, 1)} m² · `
        + `잔여 ${formatNumber(row.remainingM2, 1)} m²`
      )).join(" | ");
      const depthTitle = part.coverageDepthDetails.map((row) => (
        `${row.coverageDepth}/2: ${formatNumber(row.areaM2, 1)} m² · `
        + `${row.remainingCaptureCount > 0 ? `need ${row.remainingCaptureCount}` : "complete"}`
      )).join(" | ");
      const coverageCell = part.coverageDepthDetails.length
        ? `<div class="coverage-cell" title="${escapeHtml([depthTitle, passTitle ? `Path attribution: ${passTitle}` : ""].filter(Boolean).join(" | "))}">
            <strong>${formatPercent(part.coverage, 1)}</strong>
            <div class="coverage-depth-chips">
              ${part.coverageDepthDetails.map((row) => `<span class="coverage-depth-chip depth-${row.coverageDepth}">${row.coverageDepth}/2 ${formatNumber(row.areaM2, 0)}m²</span>`).join("")}
            </div>
          </div>`
        : part.coveragePasses.length
        ? `<div class="coverage-cell" title="${escapeHtml(passTitle)}">
            <strong>${formatPercent(part.coverage, 1)}</strong>
            <div class="coverage-pass-chips">
              ${part.coveragePasses.map((row) => `<span class="coverage-pass-chip ${escapeHtml(row.coveragePass)}${row.requirementsMet ? " is-done" : row.status === "active" ? " is-active" : ""}">${row.coveragePass === "forward" ? "OUT·SRC" : row.coveragePass === "reverse" ? "RET·SRC" : escapeHtml(row.coveragePass.toUpperCase())} ${coveragePassPercentLabel(row)}</span>`).join("")}
            </div>
          </div>`
        : formatPercent(part.coverage, 1);
      const selectable = !part.isHistorical && missionIDKey(part.inputMissionID);
      const selected = selectable && focused.has(missionIDKey(part.inputMissionID));
      return `
        <tr class="${part.status.tone === "active" ? "is-current" : ""}${part.isHistorical ? " is-history" : ""}${selected ? " is-selected" : ""}"
          ${selectable ? `data-input-mission-id="${escapeHtml(part.inputMissionID)}" tabindex="0" aria-selected="${selected}"` : ""}
          ${part.isHistorical ? `title="완료 계획 ${escapeHtml(part.historyPlanID || "-")}"` : selectable ? `title="입력임무 ${escapeHtml(part.inputMissionID)} 집중 표시"` : ""}>
          <td>${escapeHtml(part.sequence)}</td>
          <td title="${escapeHtml(part.type)}">${escapeHtml(part.type)}</td>
          <td title="${escapeHtml(part.region)}">${escapeHtml(part.region)}</td>
          <td title="${escapeHtml(part.shape)}">${escapeHtml(part.shape)}</td>
          <td><span class="cell-status ${part.status.tone}">${escapeHtml(part.status.label)}</span></td>
          <td>${coverageCell}</td>
          <td><span class="gsd-chip ${gsdClass}" title="${escapeHtml(gsdTitle)}">${gsdLabel}</span></td>
        </tr>
      `;
    }).join("");
  };

  const renderCoverage = (coverage) => {
    els.coverageTotal.textContent = `전체 ${formatPercent(coverage.overall, 1)}`;
    els.coverageRows.innerHTML = `
      ${coverageRow("선형", coverage.line, false)}
      ${coverageRow("면적 작업", coverage.area, true)}
      ${coverage.areaSpatial === null ? "" : coverageRow("왕복 공통", coverage.areaSpatial, true)}
    `;
    const lineSeries = app.trends.line.length ? app.trends.line : app.trends.coverage;
    const areaSeries = app.trends.area.length ? app.trends.area : app.trends.coverage;
    els.coverageChart.innerHTML = multiSeriesChart([
      { values: lineSeries, color: "#238b7c" },
      { values: areaSeries, color: "#d8952f" },
    ], 360, 47, 0, 100);
  };

  const coverageRow = (label, value, area) => {
    const percent = asPercent(value) ?? 0;
    return `<div class="coverage-row${area ? " area-tone" : ""}"><span class="coverage-label">${label}</span><div class="progress-track"><span style="width:${percent}%"></span></div><strong>${formatPercent(value, 1)}</strong></div>`;
  };

  const normalizeSeries = (source, key) => {
    if (Array.isArray(source)) {
      return source.map((entry) => numberOrNull(typeof entry === "object" ? firstDefined(entry?.[key], entry?.value, entry?.y) : entry)).filter((value) => value !== null);
    }
    if (source && typeof source === "object") {
      return normalizeSeries(firstDefined(source.history, source.trend, source.series, source.values), key);
    }
    return [];
  };

  const renderQuality = (quality) => {
    const external = app.state?.quality || {};
    const series = {
      gsd: normalizeSeries(firstDefined(external.gsdHistory, external.history, external.gsd), "gsd"),
      overlap: normalizeSeries(firstDefined(external.overlapHistory, external.history, external.overlap), "overlap"),
      quality: normalizeSeries(firstDefined(external.scoreHistory, external.history, external.score), "score"),
    };
    Object.keys(series).forEach((key) => {
      if (series[key].length < 2) series[key] = app.trends[key];
    });
    const hasData = Object.values(series).some((values) => values.length) || Object.values(quality).some((value) => value !== null);
    els.qualityStatus.textContent = quality.quality === null ? "평가 대기" : quality.quality >= 80 ? "기준 충족" : "품질 확인";
    if (!hasData) {
      els.qualityTrends.innerHTML = '<div class="empty-state compact">품질 표본이 없습니다.</div>';
      return;
    }
    els.qualityTrends.innerHTML = [
      qualityRow("평균 GSD", series.gsd, quality.gsd, "cm/px", "#d8952f", null, null, 2),
      qualityRow("중복도", series.overlap, quality.overlap, "%", "#238b7c", 0, 100, 0),
      qualityRow("품질 점수", series.quality, quality.quality, "점", "#4b82ad", 0, 100, 0),
    ].join("");
  };

  const qualityRow = (label, values, current, unit, color, min, max, digits) => {
    const valid = toArray(values).map(numberOrNull).filter((value) => value !== null);
    const last = current ?? valid[valid.length - 1] ?? null;
    return `
      <div class="quality-row">
        <span class="quality-label">${label}</span>
        <div class="sparkline">${sparkline(valid, color, 220, 28, min, max)}</div>
        <strong class="quality-value">${last === null ? "-" : `${formatNumber(last, digits)} ${unit}`}</strong>
      </div>
    `;
  };

  const pointsForSeries = (values, width, height, minValue = null, maxValue = null, padding = 3) => {
    const valid = toArray(values).map(numberOrNull).filter((value) => value !== null);
    if (!valid.length) return [];
    const min = minValue ?? Math.min(...valid);
    const max = maxValue ?? Math.max(...valid);
    const range = max - min || 1;
    return valid.map((value, index) => {
      const x = padding + (index / Math.max(1, valid.length - 1)) * (width - padding * 2);
      const y = height - padding - ((value - min) / range) * (height - padding * 2);
      return [x, y];
    });
  };

  const sparkline = (values, color, width, height, min = null, max = null) => {
    const points = pointsForSeries(values, width, height, min, max);
    if (!points.length) return '<span class="sparkline-empty"></span>';
    const polyline = points.map(([x, y]) => `${x.toFixed(1)},${y.toFixed(1)}`).join(" ");
    const last = points[points.length - 1];
    return `<svg viewBox="0 0 ${width} ${height}" preserveAspectRatio="none" aria-hidden="true"><polyline points="${polyline}" fill="none" stroke="${color}" stroke-width="1.7" vector-effect="non-scaling-stroke"/><circle cx="${last[0]}" cy="${last[1]}" r="2.2" fill="${color}"/></svg>`;
  };

  const multiSeriesChart = (series, width, height, min, max) => {
    const lines = series.map((item) => {
      const points = pointsForSeries(item.values, width, height, min, max, 4);
      if (!points.length) return "";
      const polyline = points.map(([x, y]) => `${x.toFixed(1)},${y.toFixed(1)}`).join(" ");
      return `<polyline points="${polyline}" fill="none" stroke="${item.color}" stroke-width="1.5" vector-effect="non-scaling-stroke"/>`;
    }).join("");
    return `<svg viewBox="0 0 ${width} ${height}" preserveAspectRatio="none" aria-hidden="true"><line x1="0" y1="${height - 1}" x2="${width}" y2="${height - 1}" stroke="#d8ddda"/>${lines}</svg>`;
  };

  const eventTone = (event) => {
    const level = String(firstDefined(event?.severity, event?.level, event?.status, event?.type, "")).toLowerCase();
    if (/critical|error|fail|danger|fatal|오류|실패|위험/.test(level)) return "bad";
    if (/warn|caution|주의|경고/.test(level)) return "warn";
    if (/success|complete|ok|정상|완료/.test(level)) return "ok";
    return "info";
  };

  const renderUavCommands = (commands) => {
    els.commandCount.textContent = `${commands.length}건`;
    if (!commands.length) {
      els.commandFeed.innerHTML = '<li class="empty-state">수신된 0602 통제 명령이 없습니다.</li>';
      return;
    }
    els.commandFeed.innerHTML = commands.slice(0, 40).map((command, index) => {
      const coordinate = commandCoordinate(command);
      const hasPosition = coordinate.latitude !== null && coordinate.longitude !== null;
      const coordinateLabel = hasPosition
        ? `${coordinate.latitude.toFixed(6)}, ${coordinate.longitude.toFixed(6)}`
        : "명령 당시 0401 위치 없음";
      const detailLines = [];
      if (command?.flightCommandText) detailLines.push(`<span><b>비행</b> ${escapeHtml(command.flightCommandText)}</span>`);
      if (command?.filmingCommandText) detailLines.push(`<span><b>촬영</b> ${escapeHtml(command.filmingCommandText)}</span>`);
      if (!detailLines.length) detailLines.push(`<span>${escapeHtml(firstDefined(command?.commandModeTypeName, "통제 명령"))}</span>`);
      const positionTime = command?.positionTimestampUnix
        ? ` · 위치 ${formatPreciseKstTime(command.positionTimestampUnix)}`
        : "";
      return `
        <li class="uav-command-item" data-command-type="${escapeHtml(firstDefined(command?.commandModeType, 0))}"
          data-command-index="${index}" tabindex="${hasPosition ? "0" : "-1"}">
          <div class="uav-command-head">
            <time>${escapeHtml(formatPreciseKstTime(command?.timestamp))}</time>
            <span>${escapeHtml(firstDefined(command?.commandModeTypeName, "통제 명령"))}</span>
            <small>${escapeHtml(firstDefined(command?.uavLabel, `UAV ${firstDefined(command?.aircraftID, "-")}`))}</small>
          </div>
          <div class="uav-command-lines">${detailLines.join("")}</div>
          <small class="uav-command-position">${escapeHtml(coordinateLabel + positionTime)}</small>
        </li>
      `;
    }).join("");
  };

  const focusUavCommand = (item) => {
    if (!item || !app.mapLoaded) return;
    const command = app.uavCommands[Number(item.dataset.commandIndex)] || {};
    const coordinate = commandCoordinate(command);
    if (coordinate.latitude === null || coordinate.longitude === null) return;
    app.mapInteracted = true;
    if (app.followID) setFollow("");
    app.map.easeTo({
      center: [coordinate.longitude, coordinate.latitude],
      zoom: Math.max(app.map.getZoom(), 16),
      duration: 450,
      essential: true,
    });
    showPinnedCoordinate({ lat: coordinate.latitude, lng: coordinate.longitude });
  };

  const discoveryKindLabel = (discovery) => (
    String(discovery?.kind || "").toUpperCase() === "TARGET" ? "TARGET" : "ROI"
  );

  const discoveryTimestampValue = (discovery) => {
    const raw = firstDefined(discovery?.timestamp, discovery?.messageTimestamp, 0);
    const numeric = numberOrNull(raw);
    if (numeric !== null) return numeric;
    const parsed = Date.parse(String(raw));
    return Number.isFinite(parsed) ? parsed : 0;
  };

  const discoveryColor = (discovery) => (
    discoveryKindLabel(discovery) === "TARGET" ? "#ef554b" : "#2997c7"
  );

  const discoveryIdentity = (discovery) => {
    if (discoveryKindLabel(discovery) === "TARGET") {
      const targetID = firstDefined(discovery?.targetID, "-");
      const targetType = firstDefined(discovery?.targetType, "-");
      return `ID ${targetID} · TYPE ${targetType}`;
    }
    return `UAV ${firstDefined(discovery?.aircraftID, "-")} · FOV ${firstDefined(discovery?.fov, "-")}`;
  };

  const discoveryCoordinate = (discovery) => {
    const coordinate = discovery?.coordinate || {};
    const latitude = numberOrNull(firstDefined(coordinate?.latitude, coordinate?.lat));
    const longitude = numberOrNull(firstDefined(coordinate?.longitude, coordinate?.lon, coordinate?.lng));
    return { latitude, longitude };
  };

  const discoveryFootprint = (discovery) => normalizeFootprint(discovery?.footprint);

  const normalizeDetectionFootprints = (discoveries) => {
    const features = [];
    toArray(discoveries).forEach((discovery, index) => {
      const footprint = discoveryFootprint(discovery);
      const kind = discoveryKindLabel(discovery);
      const color = discoveryColor(discovery);
      const properties = { discoveryIndex: index, kind, color };
      if (footprint.length >= 4) {
        features.push({
          type: "Feature",
          properties: { ...properties, featureRole: "footprint" },
          geometry: { type: "Polygon", coordinates: [footprint] },
        });
      }
      const coordinate = discoveryCoordinate(discovery);
      if (coordinate.latitude !== null && coordinate.longitude !== null) {
        features.push({
          type: "Feature",
          properties: { ...properties, featureRole: "discovery-point" },
          geometry: { type: "Point", coordinates: [coordinate.longitude, coordinate.latitude] },
        });
      }
    });
    return featureCollection(features);
  };

  const discoveryLabelText = (discovery) => {
    const kind = discoveryKindLabel(discovery);
    if (kind === "TARGET") {
      return `TARGET · ID ${firstDefined(discovery?.targetID, "-")} · TYPE ${firstDefined(discovery?.targetType, "-")}`;
    }
    return `ROI · UAV ${firstDefined(discovery?.aircraftID, "-")} · FOV ${firstDefined(discovery?.fov, "-")}`;
  };

  const discoveryKey = (discovery, index = 0) => {
    const coordinate = discoveryCoordinate(discovery);
    return [
      firstDefined(discovery?.messageTimestamp, discovery?.timestamp, index),
      discoveryKindLabel(discovery),
      firstDefined(discovery?.targetID, discovery?.aircraftID, "-"),
      coordinate.latitude ?? "-",
      coordinate.longitude ?? "-",
    ].join(":");
  };

  const clearDiscoveryLabels = () => {
    for (const record of app.discoveryLabels.values()) record.marker.remove();
    app.discoveryLabels.clear();
    app.discoveryLabelRenderKey = "";
  };

  const discoveryLabelPlacement = (point, width, height, occupied, mapWidth, mapHeight) => {
    const candidates = [
      [-width / 2, 11],
      [10, 11],
      [-width - 10, 11],
      [-width / 2, 18],
    ];
    for (const [offsetX, offsetY] of candidates) {
      const rect = {
        left: point.x + offsetX,
        top: point.y + offsetY,
        right: point.x + offsetX + width,
        bottom: point.y + offsetY + height,
      };
      if (rect.left < 5 || rect.top < 5 || rect.right > mapWidth - 5 || rect.bottom > mapHeight - 5) continue;
      if (occupied.some((other) => rectanglesOverlap(rect, other))) continue;
      return { offset: [offsetX, offsetY], rect };
    }
    return null;
  };

  const syncDiscoveryLabels = (discoveries, force = false) => {
    if (!app.mapLoaded || !app.layerVisibility.detectionFootprints) {
      clearDiscoveryLabels();
      return;
    }
    const candidates = toArray(discoveries).slice(0, DISCOVERY_LABEL_LIMIT);
    const renderKey = candidates.map(discoveryKey).join("|");
    if (!force && renderKey === app.discoveryLabelRenderKey) return;
    clearDiscoveryLabels();
    app.discoveryLabelRenderKey = renderKey;
    if (!renderKey) return;

    const container = app.map.getContainer();
    const occupied = commandLabelOccupiedRects();
    const mapWidth = container.clientWidth;
    const mapHeight = container.clientHeight;
    candidates.forEach((discovery, index) => {
      const coordinate = discoveryCoordinate(discovery);
      if (coordinate.latitude === null || coordinate.longitude === null) return;
      const point = app.map.project([coordinate.longitude, coordinate.latitude]);
      if (point.x < 0 || point.y < 0 || point.x > mapWidth || point.y > mapHeight) return;
      const text = discoveryLabelText(discovery);
      const width = clamp(text.length * 6 + 14, 104, 178);
      const height = 30;
      const placement = discoveryLabelPlacement(point, width, height, occupied, mapWidth, mapHeight);
      if (!placement) return;

      const element = document.createElement("div");
      element.className = "discovery-map-label";
      element.dataset.kind = discoveryKindLabel(discovery).toLowerCase();
      element.setAttribute("aria-hidden", "true");
      const identity = document.createElement("strong");
      identity.textContent = text;
      const time = document.createElement("time");
      time.textContent = formatPreciseKstTime(discovery?.timestamp);
      element.append(identity, time);
      const marker = new window.maplibregl.Marker({
        element,
        anchor: "top-left",
        offset: placement.offset,
      }).setLngLat([coordinate.longitude, coordinate.latitude]).addTo(app.map);
      app.discoveryLabels.set(discoveryKey(discovery, index), { marker, element });
      occupied.push(placement.rect);
    });
  };

  const renderDiscoveries = (discoveries) => {
    els.discoveryCount.textContent = discoveries.length > DISCOVERY_FEED_LIMIT
      ? `${discoveries.length}건 · 최근 ${DISCOVERY_FEED_LIMIT}`
      : `${discoveries.length}건`;
    const visibleDiscoveries = discoveries.slice(0, DISCOVERY_FEED_LIMIT);
    const renderKey = visibleDiscoveries.map(discoveryKey).join("|");
    if (renderKey === app.discoveryFeedRenderKey) return;
    app.discoveryFeedRenderKey = renderKey;
    if (!discoveries.length) {
      els.discoveryFeed.innerHTML = '<li class="empty-state">수신된 ROI/표적 발견이 없습니다.</li>';
      els.discoveryFeed.scrollTop = 0;
      return;
    }
    els.discoveryFeed.innerHTML = visibleDiscoveries.map((discovery, index) => {
      const kind = discoveryKindLabel(discovery);
      const coordinate = discoveryCoordinate(discovery);
      const footprint = discoveryFootprint(discovery);
      const coordinateTextValue = coordinate.latitude !== null && coordinate.longitude !== null
        ? `${coordinate.latitude.toFixed(6)}, ${coordinate.longitude.toFixed(6)}`
        : "좌표 없음";
      const watcher = firstDefined(discovery?.watcherID, discovery?.aircraftID);
      const source = watcher === undefined ? "0402" : `UAV ${watcher}`;
      const footprintTime = firstDefined(discovery?.footprintTimestampUnix);
      const footprintText = footprint.length >= 4
        ? `발견 Footprint ${footprint.length - 1}점${footprintTime ? ` · 0401 ${formatPreciseKstTime(footprintTime)}` : ""}`
        : "발견 Footprint 없음";
      return `
        <li class="discovery-item" data-kind="${kind.toLowerCase()}"
          data-detection-lat="${coordinate.latitude ?? ""}" data-detection-lon="${coordinate.longitude ?? ""}"
          data-discovery-index="${index}"
          tabindex="${(coordinate.latitude !== null && coordinate.longitude !== null) || footprint.length >= 4 ? "0" : "-1"}">
          <div class="discovery-head">
            <time>${escapeHtml(formatPreciseKstTime(discovery?.timestamp))}</time>
            <span>${escapeHtml(kind)}</span>
            <small>${escapeHtml(source)}</small>
          </div>
          <strong>${escapeHtml(discoveryIdentity(discovery))}</strong>
          <span class="discovery-coordinate">${escapeHtml(coordinateTextValue)}</span>
          <small class="discovery-footprint">${escapeHtml(footprintText)}</small>
          <small class="discovery-raw-time">원본 timestamp ${escapeHtml(firstDefined(discovery?.messageTimestamp, "-"))}</small>
        </li>
      `;
    }).join("");
    els.discoveryFeed.scrollTop = 0;
  };

  const focusDiscovery = (item) => {
    if (!item || !app.mapLoaded) return;
    const latitude = numberOrNull(item.dataset.detectionLat);
    const longitude = numberOrNull(item.dataset.detectionLon);
    const discovery = app.discoveries[Number(item.dataset.discoveryIndex)] || {};
    const footprint = discoveryFootprint(discovery);
    if ((latitude === null || longitude === null) && footprint.length < 4) return;
    app.mapInteracted = true;
    if (app.followID) setFollow("");
    if (footprint.length >= 4) {
      const bounds = footprint.reduce(
        (value, point) => value.extend(point),
        new window.maplibregl.LngLatBounds(footprint[0], footprint[0])
      );
      if (latitude !== null && longitude !== null) bounds.extend([longitude, latitude]);
      const mapRect = app.map.getContainer().getBoundingClientRect();
      const toolbarRect = app.map.getContainer().querySelector(".map-toolbar")?.getBoundingClientRect();
      const topPadding = toolbarRect
        ? clamp(toolbarRect.bottom - mapRect.top + 20, 88, Math.min(230, mapRect.height * 0.4))
        : 88;
      app.map.fitBounds(bounds, {
        padding: { top: topPadding, right: 88, bottom: 88, left: 88 },
        duration: 450,
        maxZoom: 18,
      });
    } else {
      app.map.easeTo({
        center: [longitude, latitude],
        zoom: Math.max(app.map.getZoom(), 17),
        duration: 450,
        essential: true,
      });
    }
    app.coordinatePopup?.remove();
    app.coordinatePopup = null;
  };

  const renderEvents = (events) => {
    els.eventCount.textContent = `${events.length}건`;
    if (!events.length) {
      els.eventFeed.innerHTML = '<li class="empty-state">새 이벤트가 없습니다.</li>';
      return;
    }
    els.eventFeed.innerHTML = events.slice(0, 14).map((event) => {
      const message = firstDefined(typeof event === "string" ? event : undefined, event?.message, event?.text, event?.description, event?.title, event?.name, "이벤트 수신");
      const time = firstDefined(event?.time, event?.timestamp, event?.generatedAt, event?.createdAt);
      const source = firstDefined(event?.source, event?.aircraftID, event?.vehicle, event?.module, event?.category, "SYSTEM");
      return `<li class="event-item" data-tone="${eventTone(event)}"><time class="event-time">${escapeHtml(formatTime(time))}</time><span class="event-copy">${escapeHtml(message)}</span><span class="event-source" title="${escapeHtml(source)}">${escapeHtml(source)}</span></li>`;
    }).join("");
  };

  const updateDisconnectedState = () => {
    const tone = app.stateFailures >= 5 ? "bad" : "warn";
    els.connectionState.dataset.tone = tone;
    els.connectionLabel.textContent = app.stateFailures >= 5 ? "연결 끊김" : "재연결 중";
    els.dataFreshness.dataset.tone = tone;
    els.dataFreshness.textContent = app.stateReceivedAt ? `${Math.max(1, Math.round((Date.now() - app.stateReceivedAt) / 1000))}초 지연` : "데이터 없음";
  };

  const pollState = async () => {
    if (app.stopped) return;
    const started = performance.now();
    try {
      const payload = await fetchJson("/api/state", {}, 3000);
      if (payload?.ok === false) throw new Error(payload.error || "State response not ok");
      app.state = payload || {};
      app.stateFailures = 0;
      app.stateReceivedAt = Date.now();
      app.stateGeneratedAt = payload?.generatedAt || null;
      renderState(app.state);
    } catch (error) {
      app.stateFailures += 1;
      updateDisconnectedState();
      if (app.stateFailures === 1 || app.stateFailures % 10 === 0) console.warn("State polling failed", error);
    } finally {
      if (!app.stopped) setTimeout(pollState, Math.max(0, STATE_INTERVAL_MS - (performance.now() - started)));
    }
  };

  const pollMission = async () => {
    if (app.stopped) return;
    const started = performance.now();
    try {
      const query = new URLSearchParams({ since: app.missionSignature });
      const payload = await fetchJson(`/api/mission?${query.toString()}`, {}, 5000);
      if (payload?.ok === false) throw new Error(payload.error || "Mission response not ok");
      if (payload && (payload.changed !== false || !app.mission)) applyMission(payload);
      else if (payload?.signature) app.missionSignature = String(payload.signature);
    } catch (error) {
      console.warn("Mission polling failed", error);
    } finally {
      if (!app.stopped) setTimeout(pollMission, Math.max(0, MISSION_INTERVAL_MS - (performance.now() - started)));
    }
  };

  const handleShutdown = async () => {
    if (!window.confirm("모니터링 서버를 종료하시겠습니까?")) return;
    els.shutdownButton.disabled = true;
    try {
      await fetchJson("/api/shutdown", { method: "POST" }, 5000);
      app.stopped = true;
      els.connectionState.dataset.tone = "bad";
      els.connectionLabel.textContent = "종료됨";
      els.dataFreshness.dataset.tone = "bad";
      els.dataFreshness.textContent = "모니터링 종료";
      showToast("모니터링 서버 종료 요청을 전송했습니다.", 5000);
    } catch (error) {
      els.shutdownButton.disabled = false;
      showToast(`종료 요청 실패: ${error.message}`);
    }
  };

  const renderInterpolationStatus = (receiveRate = app.state?.signal?.rateHz) => {
    const receiveHz = numberOrNull(receiveRate);
    const interpolationHz = numberOrNull(app.coverageSettings?.footprint_interpolation_hz)
      ?? numberOrNull(els.interpolationHz?.value)
      ?? 30;
    els.interpolationStatus.textContent = `수신 ${receiveHz === null ? "-" : formatNumber(receiveHz, 1)} Hz → 보간 ${formatNumber(interpolationHz, 0)} Hz`;
  };

  const loadCoverageSettings = async () => {
    try {
      const payload = await fetchJson("/api/coverage-settings", {}, 3500);
      if (payload?.ok === false) throw new Error(payload.error || "설정 응답 오류");
      app.coverageSettings = payload || {};
      const hz = numberOrNull(payload?.footprint_interpolation_hz) ?? 30;
      els.interpolationHz.value = String(hz);
      renderInterpolationStatus();
    } catch (error) {
      els.interpolationStatus.textContent = "보간 설정 확인 실패";
      console.warn("Coverage settings unavailable", error);
    }
  };

  const saveCoverageSettings = async () => {
    const hz = numberOrNull(els.interpolationHz.value);
    if (hz === null || hz < 1 || hz > 120) {
      showToast("촬영 보간 주파수는 1~120 Hz로 입력해 주세요.");
      els.interpolationHz.focus();
      return;
    }
    els.saveInterpolation.disabled = true;
    try {
      const payload = await fetchJson("/api/coverage-settings", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ footprint_interpolation_hz: hz }),
      }, 4000);
      if (payload?.ok === false) throw new Error(payload.error || "설정 저장 오류");
      app.coverageSettings = payload;
      els.interpolationHz.value = String(payload.footprint_interpolation_hz);
      renderInterpolationStatus();
      showToast(`촬영 footprint 보간을 ${formatNumber(payload.footprint_interpolation_hz, 0)} Hz로 적용했습니다.`);
    } catch (error) {
      showToast(`보간 설정 저장 실패: ${error.message}`);
    } finally {
      els.saveInterpolation.disabled = false;
    }
  };

  const bindEvents = () => {
    document.querySelectorAll("[data-layer-toggle]").forEach((input) => {
      input.addEventListener("change", () => setLayerVisibility(input.dataset.layerToggle, input.checked));
    });
    els.missionViewSegments.addEventListener("click", (event) => {
      const button = event.target.closest("[data-mission-view]");
      if (button && !button.disabled) setMissionViewMode(button.dataset.missionView);
    });
    els.showMissionHistory.addEventListener("change", () => {
      app.showMissionHistory = Boolean(els.showMissionHistory.checked);
      refreshMissionFocusedViews();
    });
    els.missionPartsBody.addEventListener("click", (event) => {
      const row = event.target.closest("[data-input-mission-id]");
      if (row) selectInputMission(row.dataset.inputMissionId);
    });
    els.missionPartsBody.addEventListener("keydown", (event) => {
      if (event.key !== "Enter" && event.key !== " ") return;
      const row = event.target.closest("[data-input-mission-id]");
      if (!row) return;
      event.preventDefault();
      selectInputMission(row.dataset.inputMissionId);
    });
    els.followSegments.addEventListener("click", (event) => {
      const button = event.target.closest("[data-follow-id]");
      if (button) setFollow(button.dataset.followId);
    });
    els.optionAssignmentTabs.addEventListener("click", (event) => {
      const button = event.target.closest("[data-option-plan-id]");
      if (!button) return;
      app.selectedOptionPlanID = button.dataset.optionPlanId;
      app.optionAssignmentRenderKey = "";
      setLayerToggleChecked("optionAssignments", true);
      renderOptionAssignments(app.state?.optionAssignments);
    });
    els.optionAssignmentList.addEventListener("click", (event) => {
      const card = event.target.closest("[data-assignment-aircraft-id]");
      if (card) focusOptionAssignment(card.dataset.assignmentAircraftId);
    });
    els.commandFeed.addEventListener("click", (event) => {
      focusUavCommand(event.target.closest("[data-command-index]"));
    });
    els.commandFeed.addEventListener("keydown", (event) => {
      if (event.key === "Enter" || event.key === " ") {
        event.preventDefault();
        focusUavCommand(event.target.closest("[data-command-index]"));
      }
    });
    els.discoveryFeed.addEventListener("click", (event) => {
      focusDiscovery(event.target.closest("[data-detection-lat]"));
    });
    els.discoveryFeed.addEventListener("keydown", (event) => {
      if (event.key === "Enter" || event.key === " ") {
        event.preventDefault();
        focusDiscovery(event.target.closest("[data-detection-lat]"));
      }
    });
    els.shutdownButton.addEventListener("click", handleShutdown);
    els.saveInterpolation.addEventListener("click", saveCoverageSettings);
    els.interpolationHz.addEventListener("keydown", (event) => {
      if (event.key === "Enter") saveCoverageSettings();
    });
    document.addEventListener("visibilitychange", () => {
      if (!document.hidden && app.map) app.map.resize();
    });
    window.addEventListener("resize", () => app.map?.resize(), { passive: true });
  };

  const updateClock = () => {
    els.clock.textContent = formatTime(new Date());
    if (app.stateReceivedAt && app.stateFailures === 0) {
      const transportAge = Date.now() - app.stateReceivedAt;
      const sourceAge = numberOrNull(app.state?.signal?.ageMs);
      const age = (sourceAge ?? 0) + transportAge;
      if (String(app.state?.signal?.status || "").toUpperCase() === "WAIT") {
        els.dataFreshness.dataset.tone = "bad";
        els.dataFreshness.textContent = "0401 대기";
      } else if (age > 2500) {
        els.dataFreshness.dataset.tone = "warn";
        els.dataFreshness.textContent = `${Math.round(age / 1000)}초 지연`;
      } else {
        els.dataFreshness.dataset.tone = "ok";
        els.dataFreshness.textContent = "실시간";
      }
    }
  };

  const init = () => {
    cacheElements();
    bindEvents();
    updateClock();
    setInterval(updateClock, 1000);
    initMap();
    loadCoverageSettings();
    pollState();
    pollMission();
  };

  if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", init, { once: true });
  else init();
})();
