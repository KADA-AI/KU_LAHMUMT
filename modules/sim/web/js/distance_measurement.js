const SOURCE_ID = "sim-distance-measurement";
const FILL_LAYER_ID = "sim-distance-measurement-fill";
const LINE_LAYER_ID = "sim-distance-measurement-line";
const POINT_LAYER_ID = "sim-distance-measurement-points";
const LABEL_LAYER_ID = "sim-distance-measurement-labels";
const EARTH_RADIUS_M = 6371008.8;

const emptyCollection = () => ({ type: "FeatureCollection", features: [] });

const coordinateFrom = (point) => {
  if (Array.isArray(point) && point.length >= 2) {
    const longitude = Number(point[0]);
    const latitude = Number(point[1]);
    return Number.isFinite(longitude) && Number.isFinite(latitude)
      ? [longitude, latitude]
      : null;
  }
  if (!point || typeof point !== "object") {
    return null;
  }
  const longitude = Number(point.lng ?? point.lon ?? point.longitude);
  const latitude = Number(point.lat ?? point.latitude);
  return Number.isFinite(longitude) && Number.isFinite(latitude)
    ? [longitude, latitude]
    : null;
};

const radians = (degrees) => (Number(degrees) * Math.PI) / 180;

export const distanceMeters = (fromPoint, toPoint) => {
  const from = coordinateFrom(fromPoint);
  const to = coordinateFrom(toPoint);
  if (!from || !to) {
    return Number.NaN;
  }
  const lat1 = radians(from[1]);
  const lat2 = radians(to[1]);
  const deltaLat = lat2 - lat1;
  const deltaLon = radians(to[0] - from[0]);
  const sinLat = Math.sin(deltaLat * 0.5);
  const sinLon = Math.sin(deltaLon * 0.5);
  const a = Math.min(
    1,
    Math.max(
      0,
      sinLat * sinLat + Math.cos(lat1) * Math.cos(lat2) * sinLon * sinLon,
    ),
  );
  return 2 * EARTH_RADIUS_M * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
};

export const formatDistance = (distanceM) => {
  const value = Number(distanceM);
  if (!Number.isFinite(value) || value < 0) {
    return "-";
  }
  if (value >= 1000) {
    return `${(value / 1000).toFixed(2)} km`;
  }
  return `${Math.round(value)} m`;
};

const segmentMidpoint = (from, to) => [
  (Number(from[0]) + Number(to[0])) * 0.5,
  (Number(from[1]) + Number(to[1])) * 0.5,
];

export const buildMeasurementFeatureCollection = (points) => {
  const coordinates = (Array.isArray(points) ? points : [])
    .map(coordinateFrom)
    .filter(Boolean);
  if (!coordinates.length) {
    return emptyCollection();
  }

  const features = [];
  const isPolygon = coordinates.length >= 3;
  const boundaryCoordinates = isPolygon
    ? [...coordinates, coordinates[0]]
    : coordinates.slice();

  if (isPolygon) {
    features.push({
      type: "Feature",
      properties: { kind: "measurement-area" },
      geometry: {
        type: "Polygon",
        coordinates: [boundaryCoordinates],
      },
    });
  }

  if (coordinates.length >= 2) {
    features.push({
      type: "Feature",
      properties: { kind: "measurement-line" },
      geometry: {
        type: "LineString",
        coordinates: boundaryCoordinates,
      },
    });
  }

  coordinates.forEach((coordinate, index) => {
    features.push({
      type: "Feature",
      properties: {
        kind: "measurement-point",
        pointIndex: index + 1,
      },
      geometry: {
        type: "Point",
        coordinates: coordinate,
      },
    });
  });

  const segmentCount = isPolygon ? coordinates.length : Math.max(0, coordinates.length - 1);
  for (let index = 0; index < segmentCount; index += 1) {
    const from = coordinates[index];
    const to = coordinates[(index + 1) % coordinates.length];
    const lengthM = distanceMeters(from, to);
    features.push({
      type: "Feature",
      properties: {
        kind: "measurement-label",
        segmentIndex: index + 1,
        distanceM: Number(lengthM.toFixed(3)),
        label: formatDistance(lengthM),
      },
      geometry: {
        type: "Point",
        coordinates: segmentMidpoint(from, to),
      },
    });
  }

  return { type: "FeatureCollection", features };
};

const isMapChrome = (target) =>
  Boolean(
    target &&
      typeof target.closest === "function" &&
      target.closest(".maplibregl-control-container, .maplibregl-popup"),
  );

const consumeEvent = (event) => {
  if (typeof event?.preventDefault === "function") {
    event.preventDefault();
  }
  if (typeof event?.stopPropagation === "function") {
    event.stopPropagation();
  }
  if (typeof event?.stopImmediatePropagation === "function") {
    event.stopImmediatePropagation();
  }
};

export const initDistanceMeasurement = (
  map,
  {
    button = globalThis.document?.getElementById("toggle-distance-measurement") || null,
    documentRef = globalThis.document,
    onStatus = null,
  } = {},
) => {
  const container = map && typeof map.getContainer === "function" ? map.getContainer() : null;
  const canvas = map && typeof map.getCanvas === "function" ? map.getCanvas() : null;
  let points = [];
  let active = false;
  let finalized = false;
  let destroyed = false;
  let mapReady = Boolean(
    map && typeof map.isStyleLoaded === "function" && map.isStyleLoaded(),
  );
  let ensuringLayers = false;

  const notify = (message) => {
    if (typeof onStatus === "function") {
      onStatus(message);
    }
  };

  const ensureLayers = () => {
    if (!mapReady || ensuringLayers || !map) {
      return;
    }
    ensuringLayers = true;
    try {
      if (!map.getSource(SOURCE_ID)) {
        map.addSource(SOURCE_ID, {
          type: "geojson",
          data: emptyCollection(),
        });
      }
      if (!map.getLayer(FILL_LAYER_ID)) {
        map.addLayer({
          id: FILL_LAYER_ID,
          type: "fill",
          source: SOURCE_ID,
          filter: ["==", ["get", "kind"], "measurement-area"],
          paint: {
            "fill-color": "#38d7ff",
            "fill-opacity": 0.13,
            "fill-outline-color": "#b9f4ff",
          },
        });
      }
      if (!map.getLayer(LINE_LAYER_ID)) {
        map.addLayer({
          id: LINE_LAYER_ID,
          type: "line",
          source: SOURCE_ID,
          filter: ["==", ["get", "kind"], "measurement-line"],
          layout: {
            "line-cap": "round",
            "line-join": "round",
          },
          paint: {
            "line-color": "#73e6ff",
            "line-width": 3,
            "line-opacity": 0.98,
          },
        });
      }
      if (!map.getLayer(POINT_LAYER_ID)) {
        map.addLayer({
          id: POINT_LAYER_ID,
          type: "circle",
          source: SOURCE_ID,
          filter: ["==", ["get", "kind"], "measurement-point"],
          paint: {
            "circle-radius": 5,
            "circle-color": "#e8fbff",
            "circle-stroke-color": "#087b98",
            "circle-stroke-width": 2,
          },
        });
      }
      if (!map.getLayer(LABEL_LAYER_ID)) {
        map.addLayer({
          id: LABEL_LAYER_ID,
          type: "symbol",
          source: SOURCE_ID,
          filter: ["==", ["get", "kind"], "measurement-label"],
          layout: {
            "text-field": ["get", "label"],
            "text-font": ["Noto Sans Regular"],
            "text-size": 12,
            "text-anchor": "center",
            "text-allow-overlap": true,
            "text-ignore-placement": true,
          },
          paint: {
            "text-color": "#e7fbff",
            "text-halo-color": "rgba(5, 19, 24, 0.98)",
            "text-halo-width": 2,
          },
        });
      }
    } finally {
      ensuringLayers = false;
    }
  };

  const render = () => {
    if (!mapReady || destroyed) {
      return;
    }
    ensureLayers();
    const source = map.getSource(SOURCE_ID);
    if (source && typeof source.setData === "function") {
      source.setData(buildMeasurementFeatureCollection(points));
    }
  };

  const updateUi = () => {
    if (button) {
      button.classList.toggle("is-active", active);
      button.setAttribute("aria-pressed", active ? "true" : "false");
    }
    if (canvas?.classList) {
      canvas.classList.toggle("sim-map-distance-measure", active);
    }
  };

  const clear = ({ notifyUser = false } = {}) => {
    points = [];
    active = false;
    finalized = false;
    render();
    updateUi();
    if (notifyUser) {
      notify("거리 측정을 지웠습니다.");
    }
  };

  const closeOtherMapInputModes = () => {
    if (typeof globalThis.window?.setScenarioPanelOpen === "function") {
      globalThis.window.setScenarioPanelOpen(false);
    }
    if (typeof globalThis.window?.cancelType1NewTargetInput === "function") {
      globalThis.window.cancelType1NewTargetInput();
    }
    if (typeof globalThis.window?.cancelType1TargetOrderInput === "function") {
      globalThis.window.cancelType1TargetOrderInput();
    }
  };

  const start = () => {
    if (destroyed) {
      return;
    }
    closeOtherMapInputModes();
    points = [];
    finalized = false;
    active = true;
    render();
    updateUi();
    notify("거리 측정: 좌클릭으로 점 추가 · 우클릭으로 완료");
  };

  const finalize = () => {
    if (!active) {
      return false;
    }
    if (points.length < 2) {
      notify("거리 측정은 점을 2개 이상 찍어야 합니다.");
      return false;
    }
    active = false;
    finalized = true;
    render();
    updateUi();
    notify("측정 완료 · 아무 곳이나 클릭하면 결과가 지워집니다.");
    return true;
  };

  const mapPointFromEvent = (event) => {
    if (!container || !map || typeof map.unproject !== "function") {
      return null;
    }
    const rect = container.getBoundingClientRect();
    const x = Number(event.clientX) - Number(rect.left);
    const y = Number(event.clientY) - Number(rect.top);
    if (!Number.isFinite(x) || !Number.isFinite(y)) {
      return null;
    }
    const lngLat = map.unproject([x, y]);
    const coordinate = coordinateFrom(lngLat);
    return coordinate ? { lng: coordinate[0], lat: coordinate[1] } : null;
  };

  const onMapClickCapture = (event) => {
    if (destroyed || isMapChrome(event.target)) {
      if (finalized) {
        clear();
      }
      return;
    }
    if (finalized && !active) {
      consumeEvent(event);
      clear();
      return;
    }
    if (!active || Number(event.button || 0) !== 0) {
      return;
    }
    const point = mapPointFromEvent(event);
    if (!point) {
      return;
    }
    consumeEvent(event);
    points.push(point);
    render();
    notify(
      points.length >= 3
        ? `${points.length}점 도형 · 우클릭으로 완료`
        : `${points.length}점 · 다음 점을 좌클릭하세요`,
    );
  };

  const onMapContextMenuCapture = (event) => {
    if (!active || destroyed || isMapChrome(event.target)) {
      return;
    }
    consumeEvent(event);
    finalize();
  };

  const onButtonClick = (event) => {
    event.preventDefault();
    event.stopPropagation();
    if (active) {
      clear({ notifyUser: true });
      return;
    }
    start();
  };

  const onDocumentClickCapture = (event) => {
    if (destroyed || button?.contains(event.target) || container?.contains(event.target)) {
      return;
    }
    if (finalized) {
      clear();
      return;
    }
    if (active) {
      clear({ notifyUser: true });
    }
  };

  const onKeyDown = (event) => {
    if ((active || finalized) && event.key === "Escape") {
      event.preventDefault();
      clear({ notifyUser: true });
    }
  };

  const onStyleData = () => {
    if (destroyed || typeof map?.isStyleLoaded !== "function" || !map.isStyleLoaded()) {
      return;
    }
    mapReady = true;
    render();
  };

  const destroy = () => {
    if (destroyed) {
      return;
    }
    destroyed = true;
    container?.removeEventListener("click", onMapClickCapture, true);
    container?.removeEventListener("contextmenu", onMapContextMenuCapture, true);
    button?.removeEventListener("click", onButtonClick);
    documentRef?.removeEventListener("click", onDocumentClickCapture, true);
    documentRef?.removeEventListener("keydown", onKeyDown);
    if (typeof map?.off === "function") {
      map.off("styledata", onStyleData);
      map.off("remove", destroy);
    }
    if (canvas?.classList) {
      canvas.classList.remove("sim-map-distance-measure");
    }
  };

  container?.addEventListener("click", onMapClickCapture, true);
  container?.addEventListener("contextmenu", onMapContextMenuCapture, true);
  button?.addEventListener("click", onButtonClick);
  documentRef?.addEventListener("click", onDocumentClickCapture, true);
  documentRef?.addEventListener("keydown", onKeyDown);
  if (typeof map?.on === "function") {
    map.on("styledata", onStyleData);
    map.on("remove", destroy);
  }
  if (mapReady) {
    render();
  } else if (typeof map?.once === "function") {
    map.once("load", () => {
      if (destroyed) {
        return;
      }
      mapReady = true;
      render();
    });
  }
  updateUi();

  return {
    start,
    clear,
    finalize,
    destroy,
    isActive: () => active,
    isFinalized: () => finalized,
    getPoints: () => points.map((point) => ({ ...point })),
    getFeatureCollection: () => buildMeasurementFeatureCollection(points),
  };
};

