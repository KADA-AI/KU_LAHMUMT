import { logStatus } from "./status_log.js";

export const initMissionPanel = (map) => {
  const toggle = document.getElementById("mission-toggle");
  const panel = document.getElementById("mission-panel");
  const status = panel ? panel.querySelector("[data-mission-status]") : null;
  const statusText = status ? status.querySelector(".mission-status-text") : null;
  const loadBtn = document.getElementById("mission-load");
  const reissueInput0201Btn = document.getElementById("mission-reinput-0201");
  const type1NewTargetBtn = document.getElementById("mission-type1-new-target");
  const type1NewTargetMode = document.getElementById("type1-new-target-mode");
  const type1NewTargetCount = document.getElementById("type1-new-target-count");
  const type1NewTargetCancel = document.getElementById("type1-new-target-cancel");
  const type1TargetOrderBtn = document.getElementById("mission-type1-target-order");
  const type1TargetOrderMode = document.getElementById("type1-target-order-mode");
  const type1TargetOrderCount = document.getElementById("type1-target-order-count");
  const type1TargetOrderCancel = document.getElementById("type1-target-order-cancel");
  const folderInput = document.getElementById("mission-folder-input");
  const seedSelect = document.getElementById("multi-seed-select");
  const seedModeInputs = Array.from(
    document.querySelectorAll('input[name="multi-seed-mode"]'),
  );
  if (!toggle || !panel) {
    return;
  }

  const setStatusMessage = (message) => {
    if (!message) {
      return;
    }
    logStatus(message, { ttlMs: 4000 });
  };

  const newTargetDraftSourceId = "sim-type1-new-target-draft";
  const newTargetSentSourceId = "sim-type1-new-target-sent";
  const targetOrderSourceId = "sim-type1-target-order";
  const targetOrderLayerPrefix = "sim-type1-target-order";
  let newTargetActive = false;
  let newTargetPending = false;
  let newTargetPoints = [];
  let sentTargetPoints = [];
  let targetOrderActive = false;
  let targetOrderPending = false;
  let targetOrderSourcePackageId = null;
  let targetOrderCandidates = [];
  let targetOrderSelection = [];

  const emptyFeatureCollection = () => ({ type: "FeatureCollection", features: [] });

  const areaFeatures = (points, state) => {
    const rows = Array.isArray(points) ? points : [];
    const features = rows.map((coord, index) => ({
      type: "Feature",
      properties: { state, index: index + 1 },
      geometry: {
        type: "Point",
        coordinates: [Number(coord.longitude), Number(coord.latitude)],
      },
    }));
    if (rows.length >= 2) {
      features.unshift({
        type: "Feature",
        properties: { state },
        geometry: {
          type: "LineString",
          coordinates: rows.map((coord) => [Number(coord.longitude), Number(coord.latitude)]),
        },
      });
    }
    if (rows.length >= 3) {
      const ring = rows.map((coord) => [Number(coord.longitude), Number(coord.latitude)]);
      ring.push([...ring[0]]);
      features.unshift({
        type: "Feature",
        properties: { state },
        geometry: { type: "Polygon", coordinates: [ring] },
      });
    }
    return { type: "FeatureCollection", features };
  };

  const ensureAreaLayers = (sourceId, prefix, { fill, line, point }) => {
    if (!map || typeof map.getSource !== "function") {
      return;
    }
    if (!map.getSource(sourceId)) {
      map.addSource(sourceId, { type: "geojson", data: emptyFeatureCollection() });
    }
    if (!map.getLayer(`${prefix}-fill`)) {
      map.addLayer({
        id: `${prefix}-fill`,
        type: "fill",
        source: sourceId,
        filter: ["==", "$type", "Polygon"],
        paint: { "fill-color": fill, "fill-opacity": 0.24 },
      });
    }
    if (!map.getLayer(`${prefix}-line`)) {
      map.addLayer({
        id: `${prefix}-line`,
        type: "line",
        source: sourceId,
        filter: ["==", "$type", "LineString"],
        paint: { "line-color": line, "line-width": 2.4, "line-opacity": 0.94 },
      });
    }
    if (!map.getLayer(`${prefix}-point`)) {
      map.addLayer({
        id: `${prefix}-point`,
        type: "circle",
        source: sourceId,
        filter: ["==", "$type", "Point"],
        paint: {
          "circle-radius": 5,
          "circle-color": point,
          "circle-stroke-color": "#1b211b",
          "circle-stroke-width": 1.5,
        },
      });
    }
  };

  const ensureNewTargetLayers = () => {
    if (!map || (typeof map.isStyleLoaded === "function" && !map.isStyleLoaded())) {
      return;
    }
    ensureAreaLayers(newTargetSentSourceId, "sim-type1-new-target-sent", {
      fill: "#42c6a5",
      line: "#6be0c0",
      point: "#b8f5e3",
    });
    ensureAreaLayers(newTargetDraftSourceId, "sim-type1-new-target-draft", {
      fill: "#e2ae3f",
      line: "#ffd35a",
      point: "#ffe69b",
    });
  };

  const renderNewTargetSource = (sourceId, points, state) => {
    ensureNewTargetLayers();
    const source = map && typeof map.getSource === "function" ? map.getSource(sourceId) : null;
    if (source && typeof source.setData === "function") {
      source.setData(areaFeatures(points, state));
    }
  };

  const updateNewTargetModeUi = () => {
    if (type1NewTargetBtn) {
      type1NewTargetBtn.classList.toggle("is-active", newTargetActive);
      type1NewTargetBtn.setAttribute("aria-pressed", newTargetActive ? "true" : "false");
      type1NewTargetBtn.disabled = newTargetPending;
    }
    if (type1NewTargetMode) {
      type1NewTargetMode.classList.toggle("is-visible", newTargetActive);
      type1NewTargetMode.classList.toggle("is-pending", newTargetPending);
      type1NewTargetMode.setAttribute("aria-hidden", newTargetActive ? "false" : "true");
    }
    if (type1NewTargetCount) {
      type1NewTargetCount.textContent = newTargetPending
        ? "0201 생성·전송 중"
        : `${newTargetPoints.length}점 · 우클릭 완료`;
    }
    if (map && typeof map.getCanvas === "function") {
      map.getCanvas().classList.toggle("sim-map-polygon-input", newTargetActive);
    }
  };

  const setNewTargetActive = (active, { keepPoints = false } = {}) => {
    if (newTargetPending && !active) {
      return;
    }
    newTargetActive = Boolean(active);
    if (!newTargetActive && !keepPoints) {
      newTargetPoints = [];
      renderNewTargetSource(newTargetDraftSourceId, [], "draft");
    }
    updateNewTargetModeUi();
  };

  const undoNewTargetPoint = () => {
    if (!newTargetActive || newTargetPending || !newTargetPoints.length) {
      return;
    }
    newTargetPoints.pop();
    renderNewTargetSource(newTargetDraftSourceId, newTargetPoints, "draft");
    updateNewTargetModeUi();
  };

  const finishNewTargetArea = async () => {
    if (!newTargetActive || newTargetPending) {
      return;
    }
    if (newTargetPoints.length < 3) {
      setStatusMessage("신규 목표지역은 점을 3개 이상 입력해야 합니다");
      return;
    }
    const sim = window.simClient;
    if (!sim || typeof sim.sendType1NewTarget0201 !== "function") {
      setStatusMessage("신규 목표지역 0201 API를 사용할 수 없습니다");
      return;
    }

    newTargetPending = true;
    updateNewTargetModeUi();
    const submitted = newTargetPoints.map((coord) => ({ ...coord }));
    const result = await sim.sendType1NewTarget0201({ coordinateList: submitted });
    newTargetPending = false;
    if (!result || !result.ok) {
      updateNewTargetModeUi();
      return;
    }

    sentTargetPoints = submitted;
    renderNewTargetSource(newTargetSentSourceId, sentTargetPoints, "sent");
    setNewTargetActive(false);
    const packageId = result.newPackageID ?? "";
    setStatusMessage(`신규 목표지역 입력 완료${packageId ? ` · 0201 #${packageId}` : ""}`);
  };

  const targetOrderFeatures = () => {
    const features = [];
    targetOrderCandidates.forEach((target) => {
      const targetId = Number(target.targetInputMissionID);
      const selectedIndex = targetOrderSelection.indexOf(targetId);
      const selectedOrder = selectedIndex >= 0 ? selectedIndex + 1 : 0;
      const coordinates = Array.isArray(target.coordinateList)
        ? target.coordinateList
            .map((coord) => [Number(coord?.longitude), Number(coord?.latitude)])
            .filter(([lon, lat]) => Number.isFinite(lon) && Number.isFinite(lat))
        : [];
      if (coordinates.length < 3) {
        return;
      }
      const ring = coordinates.map((coord) => [...coord]);
      if (ring[0][0] !== ring[ring.length - 1][0] || ring[0][1] !== ring[ring.length - 1][1]) {
        ring.push([...ring[0]]);
      }
      const properties = {
        targetInputMissionID: targetId,
        originalOrder: Number(target.order) || 0,
        selected: selectedOrder > 0,
        selectedOrder,
        isNew: Boolean(target.isNew),
      };
      features.push({
        type: "Feature",
        properties,
        geometry: { type: "Polygon", coordinates: [ring] },
      });
      const centroid = target.centroid || {};
      const centroidLon = Number.isFinite(Number(centroid.longitude))
        ? Number(centroid.longitude)
        : coordinates.reduce((sum, coord) => sum + coord[0], 0) / coordinates.length;
      const centroidLat = Number.isFinite(Number(centroid.latitude))
        ? Number(centroid.latitude)
        : coordinates.reduce((sum, coord) => sum + coord[1], 0) / coordinates.length;
      features.push({
        type: "Feature",
        properties: {
          ...properties,
          label: selectedOrder > 0
            ? `${selectedOrder}순위`
            : `목표 ${properties.originalOrder}${properties.isNew ? " · 신규" : ""}`,
        },
        geometry: { type: "Point", coordinates: [centroidLon, centroidLat] },
      });
    });
    return { type: "FeatureCollection", features };
  };

  const ensureTargetOrderLayers = () => {
    if (!map || (typeof map.isStyleLoaded === "function" && !map.isStyleLoaded())) {
      return;
    }
    if (!map.getSource(targetOrderSourceId)) {
      map.addSource(targetOrderSourceId, { type: "geojson", data: emptyFeatureCollection() });
    }
    if (!map.getLayer(`${targetOrderLayerPrefix}-fill`)) {
      map.addLayer({
        id: `${targetOrderLayerPrefix}-fill`,
        type: "fill",
        source: targetOrderSourceId,
        filter: ["==", "$type", "Polygon"],
        paint: {
          "fill-color": [
            "case",
            ["==", ["get", "selected"], true],
            "#ff9f43",
            ["==", ["get", "isNew"], true],
            "#42c6a5",
            "#4f9dff",
          ],
          "fill-opacity": ["case", ["==", ["get", "selected"], true], 0.5, 0.32],
        },
      });
    }
    if (!map.getLayer(`${targetOrderLayerPrefix}-line`)) {
      map.addLayer({
        id: `${targetOrderLayerPrefix}-line`,
        type: "line",
        source: targetOrderSourceId,
        filter: ["==", "$type", "Polygon"],
        paint: {
          "line-color": [
            "case",
            ["==", ["get", "selected"], true],
            "#ffd166",
            "#b9dcff",
          ],
          "line-width": ["case", ["==", ["get", "selected"], true], 4, 2.5],
          "line-opacity": 0.98,
        },
      });
    }
    if (!map.getLayer(`${targetOrderLayerPrefix}-label`)) {
      map.addLayer({
        id: `${targetOrderLayerPrefix}-label`,
        type: "symbol",
        source: targetOrderSourceId,
        filter: ["==", "$type", "Point"],
        layout: {
          "text-field": ["get", "label"],
          "text-font": ["Noto Sans Regular"],
          "text-size": 14,
          "text-allow-overlap": true,
        },
        paint: {
          "text-color": "#ffffff",
          "text-halo-color": "#17202a",
          "text-halo-width": 2,
        },
      });
    }
  };

  const renderTargetOrderSource = () => {
    ensureTargetOrderLayers();
    const source = map && typeof map.getSource === "function"
      ? map.getSource(targetOrderSourceId)
      : null;
    if (source && typeof source.setData === "function") {
      source.setData(targetOrderFeatures());
    }
    if (map && typeof map.moveLayer === "function") {
      ["fill", "line", "label"].forEach((suffix) => {
        const layerId = `${targetOrderLayerPrefix}-${suffix}`;
        if (map.getLayer(layerId)) {
          map.moveLayer(layerId);
        }
      });
    }
  };

  const updateTargetOrderModeUi = () => {
    if (type1TargetOrderBtn) {
      type1TargetOrderBtn.classList.toggle("is-active", targetOrderActive);
      type1TargetOrderBtn.setAttribute("aria-pressed", targetOrderActive ? "true" : "false");
      type1TargetOrderBtn.disabled = targetOrderPending;
    }
    if (type1TargetOrderMode) {
      type1TargetOrderMode.classList.toggle("is-visible", targetOrderActive);
      type1TargetOrderMode.classList.toggle("is-pending", targetOrderPending);
      type1TargetOrderMode.setAttribute("aria-hidden", targetOrderActive ? "false" : "true");
    }
    if (type1TargetOrderCount) {
      type1TargetOrderCount.textContent = targetOrderPending
        ? "0201 생성·전송 중"
        : `${targetOrderSelection.length} / ${targetOrderCandidates.length} · 순서대로 클릭`;
    }
    if (map && typeof map.getCanvas === "function") {
      map.getCanvas().classList.toggle("sim-map-target-order", targetOrderActive);
    }
    if (map && typeof map.getContainer === "function") {
      map.getContainer().classList.toggle("sim-target-order-active", targetOrderActive);
    }
  };

  const setTargetOrderActive = (active) => {
    if (targetOrderPending && !active) {
      return;
    }
    targetOrderActive = Boolean(active);
    if (!targetOrderActive) {
      targetOrderSourcePackageId = null;
      targetOrderCandidates = [];
      targetOrderSelection = [];
      renderTargetOrderSource();
    }
    updateTargetOrderModeUi();
  };

  const fitTargetOrderCandidates = () => {
    const points = targetOrderCandidates.flatMap((target) => (
      Array.isArray(target.coordinateList)
        ? target.coordinateList.map((coord) => [Number(coord?.longitude), Number(coord?.latitude)])
        : []
    )).filter(([lon, lat]) => Number.isFinite(lon) && Number.isFinite(lat));
    if (!points.length || !map || typeof map.fitBounds !== "function") {
      return;
    }
    const bounds = points.reduce(
      (value, point) => value.extend(point),
      new window.maplibregl.LngLatBounds(points[0], points[0]),
    );
    map.fitBounds(bounds, { padding: 90, maxZoom: 13.5, duration: 650 });
  };

  const startTargetOrderInput = async () => {
    const sim = window.simClient;
    if (!sim || typeof sim.getType1TargetOrder !== "function") {
      setStatusMessage("목표지역 순서 변경 API를 사용할 수 없습니다");
      return;
    }
    if (newTargetActive) {
      setNewTargetActive(false);
    }
    targetOrderPending = true;
    updateTargetOrderModeUi();
    setStatusMessage("목표지역 목록을 불러오는 중...");
    const result = await sim.getType1TargetOrder();
    targetOrderPending = false;
    if (!result || !result.ok) {
      updateTargetOrderModeUi();
      return;
    }
    const targets = Array.isArray(result.targets) ? result.targets : [];
    if (targets.length < 2) {
      updateTargetOrderModeUi();
      setStatusMessage("순서를 변경하려면 목표지역이 2개 이상 필요합니다");
      return;
    }
    targetOrderSourcePackageId = Number(result.sourcePackageID);
    targetOrderCandidates = targets;
    targetOrderSelection = [];
    targetOrderActive = true;
    renderTargetOrderSource();
    updateTargetOrderModeUi();
    fitTargetOrderCandidates();
    setOpen(false);
    setStatusMessage("목표지역만 원하는 임무 순서대로 모두 클릭하세요");
  };

  const submitTargetOrder = async () => {
    if (!targetOrderActive || targetOrderPending) {
      return;
    }
    const originalOrder = targetOrderCandidates.map((target) => Number(target.targetInputMissionID));
    if (targetOrderSelection.every((targetId, index) => targetId === originalOrder[index])) {
      targetOrderSelection = [];
      renderTargetOrderSource();
      updateTargetOrderModeUi();
      setStatusMessage("기존 순서와 같습니다. 변경할 순서로 다시 선택하세요");
      return;
    }
    const sim = window.simClient;
    if (!sim || typeof sim.sendType1TargetOrder0201 !== "function") {
      setStatusMessage("목표지역 순서 변경 0201 API를 사용할 수 없습니다");
      return;
    }
    targetOrderPending = true;
    updateTargetOrderModeUi();
    const result = await sim.sendType1TargetOrder0201({
      sourcePackageID: targetOrderSourcePackageId,
      targetInputMissionIDOrder: [...targetOrderSelection],
    });
    targetOrderPending = false;
    if (!result || !result.ok) {
      targetOrderSelection = [];
      renderTargetOrderSource();
      updateTargetOrderModeUi();
      return;
    }
    const packageId = result.newPackageID ?? "";
    setTargetOrderActive(false);
    setStatusMessage(`목표지역 순서 변경 완료${packageId ? ` · 0201 #${packageId}` : ""}`);
  };

  const pointOnTargetEdge = (longitude, latitude, start, end) => {
    const startLon = Number(start?.longitude);
    const startLat = Number(start?.latitude);
    const endLon = Number(end?.longitude);
    const endLat = Number(end?.latitude);
    if (![startLon, startLat, endLon, endLat].every(Number.isFinite)) {
      return false;
    }
    const cross = (longitude - startLon) * (endLat - startLat)
      - (latitude - startLat) * (endLon - startLon);
    if (Math.abs(cross) > 1e-9) {
      return false;
    }
    const dot = (longitude - startLon) * (endLon - startLon)
      + (latitude - startLat) * (endLat - startLat);
    const lengthSquared = (endLon - startLon) ** 2 + (endLat - startLat) ** 2;
    return dot >= -1e-9 && dot <= lengthSquared + 1e-9;
  };

  const targetContainsCoordinate = (target, longitude, latitude) => {
    const polygon = Array.isArray(target?.coordinateList) ? target.coordinateList : [];
    if (polygon.length < 3) {
      return false;
    }
    let inside = false;
    for (let index = 0, previous = polygon.length - 1; index < polygon.length; previous = index++) {
      const start = polygon[previous];
      const end = polygon[index];
      if (pointOnTargetEdge(longitude, latitude, start, end)) {
        return true;
      }
      const startLon = Number(start?.longitude);
      const startLat = Number(start?.latitude);
      const endLon = Number(end?.longitude);
      const endLat = Number(end?.latitude);
      if (![startLon, startLat, endLon, endLat].every(Number.isFinite)) {
        continue;
      }
      const crossesLatitude = (startLat > latitude) !== (endLat > latitude);
      const crossingLongitude = ((endLon - startLon) * (latitude - startLat))
        / ((endLat - startLat) || Number.EPSILON) + startLon;
      if (crossesLatitude && longitude < crossingLongitude) {
        inside = !inside;
      }
    }
    return inside;
  };

  const selectTargetOrderAt = (lngLat) => {
    if (!targetOrderActive || targetOrderPending || !lngLat) {
      return;
    }
    const longitude = Number(lngLat.lng ?? lngLat.longitude);
    const latitude = Number(lngLat.lat ?? lngLat.latitude);
    if (!Number.isFinite(longitude) || !Number.isFinite(latitude)) {
      return;
    }
    const target = targetOrderCandidates.find((candidate) => {
      const targetId = Number(candidate.targetInputMissionID);
      return !targetOrderSelection.includes(targetId)
        && targetContainsCoordinate(candidate, longitude, latitude);
    });
    const targetId = Number(target?.targetInputMissionID);
    if (!Number.isFinite(targetId)) {
      setStatusMessage("강조된 목표지역 내부를 클릭하세요");
      return;
    }
    targetOrderSelection.push(targetId);
    renderTargetOrderSource();
    updateTargetOrderModeUi();
    if (targetOrderSelection.length === targetOrderCandidates.length) {
      void submitTargetOrder();
    }
  };

  const captureTargetOrderClick = (event) => {
    if (!targetOrderActive || targetOrderPending || !map || event.button !== 0) {
      return;
    }
    if (event.target instanceof Element && event.target.closest(".maplibregl-control-container")) {
      return;
    }
    const container = map.getContainer();
    const bounds = container.getBoundingClientRect();
    const point = [event.clientX - bounds.left, event.clientY - bounds.top];
    const lngLat = map.unproject(point);
    event.preventDefault();
    event.stopPropagation();
    if (typeof event.stopImmediatePropagation === "function") {
      event.stopImmediatePropagation();
    }
    selectTargetOrderAt(lngLat);
  };

  const undoTargetOrderSelection = () => {
    if (!targetOrderActive || targetOrderPending || !targetOrderSelection.length) {
      return;
    }
    targetOrderSelection.pop();
    renderTargetOrderSource();
    updateTargetOrderModeUi();
  };

  const setMissionReady = (ready) => {
    if (!status) {
      return;
    }
    const next = Boolean(ready);
    status.classList.toggle("is-ready", next);
    if (statusText) {
      statusText.textContent = next ? "완료" : "대기";
    }
  };

  const setOpen = (open) => {
    const next = Boolean(open);
    panel.classList.toggle("is-open", next);
    panel.setAttribute("aria-hidden", next ? "false" : "true");
    toggle.setAttribute("aria-expanded", next ? "true" : "false");
    toggle.classList.toggle("is-active", next);
    if (next && typeof window.setScenarioPanelOpen === "function") {
      window.setScenarioPanelOpen(false);
    }
  };

  window.setMissionPanelOpen = setOpen;

  toggle.addEventListener("click", (event) => {
    event.stopPropagation();
    setOpen(!panel.classList.contains("is-open"));
  });

  panel.addEventListener("click", (event) => {
    event.stopPropagation();
  });

  document.addEventListener("click", (event) => {
    const target = event.target;
    if (!panel.contains(target) && target !== toggle) {
      setOpen(false);
    }
  });

  const parseFilesToFeatures = async (files) => {
    const features = [];
    let featureId = 1;
    const agents = {};
    const flightPaths = [];
    const missionPlans = [];
    const missionPlanOptions = [];
    const individualPlans = [];
    const inputPlans = [];
    const normalize = (value) => (value === null || value === undefined ? null : Number(value));
    const getCoord = (item) => {
      const coord = item?.coordinate || item?.Coordinate;
      if (!coord) return null;
      const lat = coord.latitude ?? coord.Latitude;
      const lon = coord.longitude ?? coord.Longitude;
      const alt = coord.altitude ?? coord.Altitude;
      if (lat === undefined || lon === undefined) return null;
      return { lat: Number(lat), lon: Number(lon), alt: alt !== undefined ? Number(alt) : null };
    };
    const getWaypoints = (data) =>
      data?.lahWaypointList || data?.uavWaypointList || data?.waypointList || [];
    const orderWaypoints = (raw) => {
      if (!Array.isArray(raw) || raw.length < 2) {
        return raw;
      }
      const byId = new Map();
      const nextIds = new Set();
      raw.forEach((wp) => {
        if (!wp || typeof wp !== "object") return;
        const wid = wp.waypointID ?? wp.WaypointID;
        if (!Number.isFinite(Number(wid))) return;
        const id = Number(wid);
        byId.set(id, wp);
        const nextId = wp.nextWaypointID ?? wp.NextWaypointID;
        if (Number.isFinite(Number(nextId)) && Number(nextId) > 0) {
          nextIds.add(Number(nextId));
        }
      });
      if (!byId.size) {
        return raw;
      }
      let startId = null;
      byId.forEach((_value, key) => {
        if (startId !== null) return;
        if (!nextIds.has(key)) {
          startId = key;
        }
      });
      const ordered = [];
      const visited = new Set();
      if (startId !== null) {
        let curr = startId;
        while (curr && byId.has(curr) && !visited.has(curr)) {
          const wp = byId.get(curr);
          ordered.push(wp);
          visited.add(curr);
          const nextId = wp.nextWaypointID ?? wp.NextWaypointID;
          const nextVal = Number(nextId);
          if (!Number.isFinite(nextVal) || nextVal === 0) {
            break;
          }
          curr = nextVal;
        }
      }
      raw.forEach((wp) => {
        if (!wp || typeof wp !== "object") {
          ordered.push(wp);
          return;
        }
        const wid = wp.waypointID ?? wp.WaypointID;
        const id = Number(wid);
        if (!Number.isFinite(id) || !visited.has(id)) {
          ordered.push(wp);
        }
      });
      return ordered;
    };
    const agentLabel = (aircraftId) => {
      const id = Number(aircraftId);
      if (id >= 1 && id <= 3) return `LAH${id}`;
      if (id >= 4 && id <= 6) return `UAV${id - 3}`;
      return `AC${id}`;
    };
    const getLineSearch = (wp) => {
      const filming = wp?.filmingProperty || wp?.FilmingProperty || null;
      if (!filming || typeof filming !== "object") {
        return null;
      }
      const lineSearch = filming.lineSearch || filming.LineSearch || null;
      return lineSearch && typeof lineSearch === "object" ? lineSearch : null;
    };
    const getSearchCoord = (item) => {
      if (!item || typeof item !== "object") {
        return null;
      }
      const lat = Number(item.latitude ?? item.Latitude);
      const lon = Number(item.longitude ?? item.Longitude);
      if (
        !Number.isFinite(lat) ||
        !Number.isFinite(lon) ||
        lat < -90 ||
        lat > 90 ||
        lon < -180 ||
        lon > 180
      ) {
        return null;
      }
      return { lat, lon };
    };
    const extractSweepLinesFromPath = (path) => {
      const lines = [];
      const waypoints = orderWaypoints(getWaypoints(path));
      if (!Array.isArray(waypoints)) {
        return lines;
      }
      waypoints.forEach((wp) => {
        const lineSearch = getLineSearch(wp);
        const coordinateList = Array.isArray(lineSearch?.coordinateList)
          ? lineSearch.coordinateList
          : Array.isArray(lineSearch?.CoordinateList)
            ? lineSearch.CoordinateList
            : [];
        const points = coordinateList.map(getSearchCoord).filter(Boolean);
        if (points.length < 2) {
          return;
        }
        const chunkSize = Math.trunc(
          Number(
            lineSearch?.interpolationPoints ??
              lineSearch?.InterpolationPoints ??
              lineSearch?.interpolationPoint ??
              lineSearch?.InterpolationPoint ??
              0,
          ),
        );
        if (chunkSize > 2 && points.length > chunkSize) {
          for (let start = 0; start < points.length; start += chunkSize) {
            const chunk = points.slice(start, start + chunkSize);
            if (chunk.length >= 2) {
              lines.push(chunk);
            }
          }
          return;
        }
        lines.push(points);
      });
      return lines;
    };
    const buildPathMissionIndex = () => {
      const index = {};
      individualPlans.forEach((plan) => {
        if (!plan || typeof plan !== "object") {
          return;
        }
        const aircraftId = Number(plan.aircraftID ?? plan.AircraftID);
        const missions = Array.isArray(plan.individualMissionList) ? plan.individualMissionList : [];
        missions.forEach((mission) => {
          const pathId = Number(mission?.pathID ?? mission?.PathID);
          if (!Number.isFinite(pathId)) {
            return;
          }
          const related = mission?.relatedMission || mission?.RelatedMission || {};
          const inputMissionId = Number(
            related.inputMissionID ??
              related.InputMissionID ??
              mission.inputMissionID ??
              mission.InputMissionID,
          );
          const individualMissionId = Number(
            mission.individualMissionID ?? mission.IndividualMissionID,
          );
          index[String(Math.trunc(pathId))] = {
            pathID: Math.trunc(pathId),
            aircraftID: Number.isFinite(aircraftId) ? Math.trunc(aircraftId) : null,
            inputMissionID: Number.isFinite(inputMissionId) ? Math.trunc(inputMissionId) : null,
            individualMissionID: Number.isFinite(individualMissionId)
              ? Math.trunc(individualMissionId)
              : null,
          };
        });
      });
      return index;
    };
    const polylineLength = (points) => {
      let total = 0;
      for (let idx = 1; idx < points.length; idx += 1) {
        total += Math.hypot(points[idx].x - points[idx - 1].x, points[idx].y - points[idx - 1].y);
      }
      return total;
    };
    const pointAtFraction = (points, fraction) => {
      if (!points.length) {
        return { x: 0, y: 0 };
      }
      if (points.length === 1) {
        return points[0];
      }
      const target = Math.max(0, Math.min(1, Number(fraction))) * polylineLength(points);
      if (target <= 0) {
        return points[0];
      }
      let walked = 0;
      for (let idx = 1; idx < points.length; idx += 1) {
        const start = points[idx - 1];
        const end = points[idx];
        const segLen = Math.hypot(end.x - start.x, end.y - start.y);
        if (segLen <= 0) {
          continue;
        }
        if (walked + segLen >= target) {
          const ratio = (target - walked) / segLen;
          return {
            x: start.x + (end.x - start.x) * ratio,
            y: start.y + (end.y - start.y) * ratio,
          };
        }
        walked += segLen;
      }
      return points[points.length - 1];
    };
    const samplePolyline = (points, sampleCount = 9) => {
      if (points.length <= 2) {
        return points.slice();
      }
      const count = Math.max(2, Math.trunc(sampleCount));
      return Array.from({ length: count }, (_value, idx) => pointAtFraction(points, idx / (count - 1)));
    };
    const pointSegmentDistance = (point, start, end) => {
      const dx = end.x - start.x;
      const dy = end.y - start.y;
      const denom = dx * dx + dy * dy;
      if (denom <= 0) {
        return Math.hypot(point.x - start.x, point.y - start.y);
      }
      const ratio = Math.max(
        0,
        Math.min(1, ((point.x - start.x) * dx + (point.y - start.y) * dy) / denom),
      );
      const cx = start.x + dx * ratio;
      const cy = start.y + dy * ratio;
      return Math.hypot(point.x - cx, point.y - cy);
    };
    const pointPolylineDistance = (point, line) => {
      if (!line.length) {
        return Infinity;
      }
      if (line.length === 1) {
        return Math.hypot(point.x - line[0].x, point.y - line[0].y);
      }
      let best = Infinity;
      for (let idx = 1; idx < line.length; idx += 1) {
        best = Math.min(best, pointSegmentDistance(point, line[idx - 1], line[idx]));
      }
      return best;
    };
    const meanPolylineSpacing = (left, right) => {
      if (left.length < 2 || right.length < 2) {
        return null;
      }
      const distances = [
        ...samplePolyline(left).map((point) => pointPolylineDistance(point, right)),
        ...samplePolyline(right).map((point) => pointPolylineDistance(point, left)),
      ].filter(Number.isFinite);
      if (!distances.length) {
        return null;
      }
      return distances.reduce((sum, value) => sum + value, 0) / distances.length;
    };
    const projectLine = (line, originLat, originLon) => {
      const earthRadiusM = 6371008.8;
      const cosLat = Math.cos((originLat * Math.PI) / 180);
      return line.map((point) => ({
        x: (((point.lon - originLon) * Math.PI) / 180) * earthRadiusM * cosLat,
        y: (((point.lat - originLat) * Math.PI) / 180) * earthRadiusM,
      }));
    };
    const buildSweepLineSpacingSummaries = (pathMissionIndex) => {
      const grouped = new Map();
      flightPaths.forEach((path) => {
        const pathId = Number(path?.pathID ?? path?.PathID);
        if (!Number.isFinite(pathId)) {
          return;
        }
        const missionMeta = pathMissionIndex[String(Math.trunc(pathId))] || {};
        const inputMissionId = Number(missionMeta.inputMissionID);
        if (!Number.isFinite(inputMissionId)) {
          return;
        }
        const lines = extractSweepLinesFromPath(path);
        if (!lines.length) {
          return;
        }
        const key = Math.trunc(inputMissionId);
        if (!grouped.has(key)) {
          grouped.set(key, {
            inputMissionID: key,
            pathIds: new Set(),
            aircraftIds: new Set(),
            linesByPath: new Map(),
            allCoords: [],
          });
        }
        const group = grouped.get(key);
        const normalizedPathId = Math.trunc(pathId);
        group.pathIds.add(normalizedPathId);
        const aircraftId = Number(path?.aircraftID ?? path?.AircraftID ?? missionMeta.aircraftID);
        if (Number.isFinite(aircraftId)) {
          group.aircraftIds.add(Math.trunc(aircraftId));
        }
        const pathLines = group.linesByPath.get(normalizedPathId) || [];
        pathLines.push(...lines);
        group.linesByPath.set(normalizedPathId, pathLines);
        lines.forEach((line) => {
          group.allCoords.push(...line);
        });
      });

      return Array.from(grouped.values())
        .sort((a, b) => a.inputMissionID - b.inputMissionID)
        .map((group) => {
          if (!group.allCoords.length) {
            return null;
          }
          const originLat =
            group.allCoords.reduce((sum, point) => sum + point.lat, 0) / group.allCoords.length;
          const originLon =
            group.allCoords.reduce((sum, point) => sum + point.lon, 0) / group.allCoords.length;
          const distances = [];
          let lineCount = 0;
          Array.from(group.linesByPath.keys())
            .sort((a, b) => a - b)
            .forEach((pathId) => {
              const projectedLines = (group.linesByPath.get(pathId) || [])
                .filter((line) => line.length >= 2)
                .map((line) => projectLine(line, originLat, originLon));
              lineCount += projectedLines.length;
              for (let idx = 1; idx < projectedLines.length; idx += 1) {
                const spacing = meanPolylineSpacing(projectedLines[idx - 1], projectedLines[idx]);
                if (Number.isFinite(spacing)) {
                  distances.push(spacing);
                }
              }
            });
          if (!distances.length) {
            return null;
          }
          const averageLineSpacingM =
            distances.reduce((sum, value) => sum + value, 0) / distances.length;
          return {
            inputMissionID: group.inputMissionID,
            averageLineSpacingM,
            minLineSpacingM: Math.min(...distances),
            maxLineSpacingM: Math.max(...distances),
            lineCount,
            pairCount: distances.length,
            pathIds: Array.from(group.pathIds).sort((a, b) => a - b),
            aircraftIds: Array.from(group.aircraftIds).sort((a, b) => a - b),
          };
        })
        .filter(Boolean);
    };

    for (const file of files) {
      const rel = (file.webkitRelativePath || file.name || "").replace(/\\/g, "/");
      const lower = rel.toLowerCase();
      if (!lower.endsWith(".json")) {
        continue;
      }
      let data = null;
      try {
        const text = await file.text();
        data = JSON.parse(text);
      } catch (err) {
        continue;
      }
      if (lower.includes("/missionplanoptioninfo/")) {
        missionPlanOptions.push(data);
        continue;
      }
      if (lower.includes("/missionplan/")) {
        missionPlans.push(data);
        continue;
      }
      if (lower.includes("/individualmissionplan/")) {
        individualPlans.push(data);
        continue;
      }
      if (lower.includes("/inputmissionplan/")) {
        inputPlans.push(data);
        continue;
      }
      if (!lower.includes("/flightpath/")) {
        continue;
      }

      const waypoints = orderWaypoints(getWaypoints(data));
      if (!Array.isArray(waypoints) || waypoints.length < 2) {
        continue;
      }
      flightPaths.push(data);
      const coords = [];
      const alts = [];
      const wpIds = [];
      for (const wp of waypoints) {
        const coord = getCoord(wp || {});
        if (!coord) continue;
        const wid = wp?.waypointID ?? wp?.WaypointID ?? null;
        const widNum = Number(wid);
        coords.push([coord.lon, coord.lat]);
        wpIds.push(Number.isFinite(widNum) ? widNum : null);
        if (coord.alt !== null && !Number.isNaN(coord.alt)) {
          alts.push(coord.alt);
        } else {
          alts.push(null);
        }
      }
      if (coords.length < 2) {
        continue;
      }
      const aircraftId = data?.aircraftID ?? data?.AircraftID ?? null;
      const pathId = data?.pathID ?? data?.PathID ?? file.name;
      const agent = agentLabel(aircraftId);
      agents[agent] = (agents[agent] || 0) + 1;
      features.push({
        id: featureId++,
        agent,
        aircraftId: normalize(aircraftId),
        pathId,
        points: coords.length,
        coords,
        alts,
        wpIds,
        altMin: alts.some((v) => v !== null)
          ? Math.min(...alts.filter((v) => v !== null))
          : null,
        altMax: alts.some((v) => v !== null)
          ? Math.max(...alts.filter((v) => v !== null))
          : null,
      });
    }

    const buildMissionOrder = () => {
      if (!missionPlans.length || !individualPlans.length || !flightPaths.length) {
        return null;
      }
      let selectedMissionPlan = null;
      if (missionPlanOptions.length) {
        const optionInfo = missionPlanOptions.reduce((best, item) => {
          if (!best) return item;
          const tsA = Number(best.timestamp) || 0;
          const tsB = Number(item.timestamp) || 0;
          return tsB >= tsA ? item : best;
        }, null);
        const options = optionInfo?.optionList || [];
        const recommended = options.find((opt) => opt?.recommend);
        const planId = recommended?.missionPlanID;
        selectedMissionPlan = missionPlans.find(
          (plan) => Number(plan?.missionPlanID) === Number(planId),
        );
      }
      if (!selectedMissionPlan) {
        selectedMissionPlan = missionPlans.reduce((best, item) => {
          if (!best) return item;
          const tsA = Number(best.timestamp) || Number(best.missionPlanTimestamp) || 0;
          const tsB = Number(item.timestamp) || Number(item.missionPlanTimestamp) || 0;
          return tsB >= tsA ? item : best;
        }, null);
      }
      if (!selectedMissionPlan) {
        return null;
      }

      const inputPlan = inputPlans.reduce((best, item) => {
        if (!best) return item;
        const tsA = Number(best.timestamp) || 0;
        const tsB = Number(item.timestamp) || 0;
        return tsB >= tsA ? item : best;
      }, null);
      const inputOrder = new Map();
      const inputList = inputPlan?.inputMissionList || [];
      if (Array.isArray(inputList)) {
        inputList.forEach((item, idx) => {
          const id = Number(item?.inputMissionID);
          if (Number.isFinite(id)) {
            inputOrder.set(id, idx);
          }
        });
      }

      const missionOrder = {};
      const aircraftList = selectedMissionPlan?.aircraftList || [];
      aircraftList.forEach((air) => {
        const aircraftId = Number(air?.aircraftID);
        const pkgId = Number(air?.individualMissionPackageID);
        if (!Number.isFinite(aircraftId) || !Number.isFinite(pkgId)) {
          return;
        }
        const plan = individualPlans.find(
          (entry) => Number(entry?.individualMissionPackageID) === pkgId,
        );
        if (!plan) {
          return;
        }
        const list = Array.isArray(plan.individualMissionList)
          ? plan.individualMissionList.slice()
          : [];
        const ordered = list
          .map((mission, idx) => {
            const inputId = Number(mission?.relatedMission?.inputMissionID);
            const orderIdx = inputOrder.has(inputId) ? inputOrder.get(inputId) : idx;
            return { mission, orderIdx };
          })
          .sort((a, b) => (a.orderIdx ?? 0) - (b.orderIdx ?? 0))
          .map((item) => Number(item.mission?.pathID))
          .filter((id) => Number.isFinite(id));
        if (ordered.length) {
          missionOrder[aircraftId] = ordered;
        }
      });

      return Object.keys(missionOrder).length ? missionOrder : null;
    };

    const missionOrder = buildMissionOrder();
    const pathMissionIndex = buildPathMissionIndex();
    const sweepLineSpacingSummaries = buildSweepLineSpacingSummaries(pathMissionIndex);
    const sweepLineSpacingByInputMissionID = Object.fromEntries(
      sweepLineSpacingSummaries.map((item) => [String(item.inputMissionID), item]),
    );
    return {
      ok: true,
      features,
      agents,
      count: features.length,
      flightPaths,
      missionOrder,
      inputMissionPlans: inputPlans,
      individualMissionPlans: individualPlans,
      pathMissionIndex,
      sweepLineSpacingSummaries,
      sweepLineSpacingByInputMissionID,
    };
  };

  const parseMissionReference = async (files) => {
    let best = null;
    for (const file of files) {
      const rel = (file.webkitRelativePath || file.name || "").replace(/\\/g, "/");
      if (!rel.toLowerCase().includes("/missionreferenceinfo/")) {
        continue;
      }
      if (!rel.toLowerCase().endsWith(".json")) {
        continue;
      }
      let data = null;
      try {
        const text = await file.text();
        data = JSON.parse(text);
      } catch (err) {
        continue;
      }
      if (Array.isArray(data)) {
        data = data
          .filter((row) => row && typeof row === "object")
          .sort((a, b) => (Number(a.timestamp) || 0) - (Number(b.timestamp) || 0))
          .at(-1) || null;
      }
      const hasReferencePoints = [
        data?.takeOverInfoList,
        data?.handOverInfoList,
        data?.rtbCoordinateList,
      ].some((list) => Array.isArray(list) && list.length);
      if (!hasReferencePoints) {
        continue;
      }
      const ts = Number(data.timestamp) || 0;
      if (!best || ts >= best.timestamp) {
        best = { timestamp: ts, data };
      }
    }
    if (!best) {
      return { ok: false, vehicles: {} };
    }
    const vehicles = {};
    const list = best.data.takeOverInfoList || [];
    const toCoord = (entry) => {
      const coord = entry?.coordinate || entry?.Coordinate;
      if (!coord) return null;
      const lat = coord.latitude ?? coord.Latitude;
      const lon = coord.longitude ?? coord.Longitude;
      const alt = coord.altitude ?? coord.Altitude;
      if (!Number.isFinite(Number(lat)) || !Number.isFinite(Number(lon))) {
        return null;
      }
      return { lat: Number(lat), lon: Number(lon), alt: Number(alt) || 0 };
    };
    list.forEach((entry) => {
      const id = Number(entry?.aircraftID ?? entry?.AircraftID);
      if (!Number.isFinite(id) || id < 4 || id > 6) {
        return;
      }
      const coord = toCoord(entry);
      if (!coord) {
        return;
      }
      const agent = `UAV${id - 3}`;
      vehicles[agent] = coord;
    });
    const formationNorthM = [0, 100, -100];
    ["UAV1", "UAV2", "UAV3"].forEach((uav, index) => {
      const base = vehicles[uav];
      if (!base) {
        return;
      }
      vehicles[`LAH${index + 1}`] = {
        lat: base.lat + formationNorthM[index] / 111320,
        lon: base.lon,
        alt: base.alt,
      };
    });
    return {
      ok: true,
      vehicles,
      takeOverInfoList: best.data.takeOverInfoList || [],
      handOverInfoList: best.data.handOverInfoList || [],
      rtbCoordinateList: best.data.rtbCoordinateList || [],
    };
  };

  if (loadBtn) {
    loadBtn.addEventListener("click", () => {
      if (folderInput) {
        folderInput.value = "";
        folderInput.click();
      } else {
        setStatusMessage("Folder picker unavailable");
      }
    });
  }

  if (folderInput) {
    folderInput.addEventListener("change", async (event) => {
      const files = Array.from(event.target.files || []);
      if (!files.length) {
        return;
      }
      if (loadBtn) {
        loadBtn.disabled = true;
      }
      setStatusMessage("Loading mission...");
      try {
        const [data, reference] = await Promise.all([
          parseFilesToFeatures(files),
          parseMissionReference(files),
        ]);
        if (typeof window.missionPathLoader === "function") {
          window.missionPathLoader(data);
        }
        if (typeof window.missionVehicleLoader === "function") {
          window.missionVehicleLoader(reference);
        }
        if (typeof window.missionReferenceLoader === "function") {
          window.missionReferenceLoader(reference);
        }
        if (data && Array.isArray(data.flightPaths) && data.flightPaths.length > 0) {
          if (window.simClient && typeof window.simClient.setMission === "function") {
            window.simClient.setMission({
              flightPaths: data.flightPaths,
              missionOrder: data.missionOrder || null,
              inputMissionPlans: data.inputMissionPlans || [],
              individualMissionPlans: data.individualMissionPlans || [],
              takeOverInfoList: reference?.takeOverInfoList || [],
              handOverInfoList: reference?.handOverInfoList || [],
              rtbCoordinateList: reference?.rtbCoordinateList || [],
            });
          }
        }
        if (data.ok && data.count) {
          setMissionReady(true);
          setStatusMessage(`Mission loaded (${data.count})`);
        } else {
          setMissionReady(false);
          setStatusMessage("No FlightPath data found");
        }
      } catch (err) {
        setMissionReady(false);
        setStatusMessage("Mission load failed");
      } finally {
        if (loadBtn) {
          loadBtn.disabled = false;
        }
      }
    });
  }

  if (reissueInput0201Btn) {
    reissueInput0201Btn.addEventListener("click", async () => {
      const sim = window.simClient;
      if (!sim || typeof sim.reissueInput0201 !== "function") {
        setStatusMessage("0201 reissue API unavailable");
        return;
      }
      reissueInput0201Btn.disabled = true;
      setStatusMessage("0201 재입력 준비 중...");
      try {
        const result = await sim.reissueInput0201();
        if (result && result.ok) {
          const packageId = result.newPackageID ?? "";
          setStatusMessage(`0201 재입력 전송 완료${packageId ? ` (#${packageId})` : ""}`);
        }
      } finally {
        reissueInput0201Btn.disabled = false;
      }
    });
  }

  if (type1NewTargetBtn && map) {
    type1NewTargetBtn.addEventListener("click", () => {
      if (newTargetPending) {
        return;
      }
      if (!newTargetActive && targetOrderActive) {
        setTargetOrderActive(false);
      }
      if (!newTargetActive && typeof window.setScenarioPanelOpen === "function") {
        window.setScenarioPanelOpen(false);
      }
      setNewTargetActive(!newTargetActive);
      if (newTargetActive) {
        setStatusMessage("신규 목표지역 입력 모드");
      }
    });
  }

  if (type1TargetOrderBtn && map) {
    type1TargetOrderBtn.addEventListener("click", () => {
      if (targetOrderPending) {
        return;
      }
      if (targetOrderActive) {
        setTargetOrderActive(false);
        setStatusMessage("목표지역 순서 변경 취소");
        return;
      }
      if (typeof window.setScenarioPanelOpen === "function") {
        window.setScenarioPanelOpen(false);
      }
      void startTargetOrderInput();
    });
  }

  if (type1NewTargetCancel) {
    type1NewTargetCancel.addEventListener("click", (event) => {
      event.preventDefault();
      event.stopPropagation();
      if (!newTargetPending) {
        setNewTargetActive(false);
        setStatusMessage("신규 목표지역 입력 취소");
      }
    });
  }

  if (type1TargetOrderCancel) {
    type1TargetOrderCancel.addEventListener("click", (event) => {
      event.preventDefault();
      event.stopPropagation();
      if (!targetOrderPending) {
        setTargetOrderActive(false);
        setStatusMessage("목표지역 순서 변경 취소");
      }
    });
  }

  window.cancelType1NewTargetInput = () => {
    if (!newTargetPending) {
      setNewTargetActive(false);
    }
  };

  window.cancelType1TargetOrderInput = () => {
    if (!targetOrderPending) {
      setTargetOrderActive(false);
    }
  };

  if (map) {
    map.getContainer().addEventListener("click", captureTargetOrderClick, true);
    if (typeof map.isStyleLoaded === "function" && map.isStyleLoaded()) {
      ensureNewTargetLayers();
      ensureTargetOrderLayers();
    } else {
      map.on("load", () => {
        ensureNewTargetLayers();
        ensureTargetOrderLayers();
        renderNewTargetSource(newTargetDraftSourceId, newTargetPoints, "draft");
        renderNewTargetSource(newTargetSentSourceId, sentTargetPoints, "sent");
        renderTargetOrderSource();
      });
    }

    map.on("click", (event) => {
      if (targetOrderActive) {
        if (event.originalEvent) {
          event.originalEvent.preventDefault();
          event.originalEvent.stopPropagation();
        }
        selectTargetOrderAt(event.lngLat);
        return;
      }
      if (!newTargetActive || newTargetPending) {
        return;
      }
      if (event.originalEvent) {
        event.originalEvent.preventDefault();
        event.originalEvent.stopPropagation();
      }
      newTargetPoints.push({
        latitude: Number(Number(event.lngLat.lat).toFixed(8)),
        longitude: Number(Number(event.lngLat.lng).toFixed(8)),
        altitude: 0,
      });
      renderNewTargetSource(newTargetDraftSourceId, newTargetPoints, "draft");
      updateNewTargetModeUi();
    });

    map.on("contextmenu", (event) => {
      if (!newTargetActive) {
        return;
      }
      if (typeof event.preventDefault === "function") {
        event.preventDefault();
      }
      if (event.originalEvent) {
        event.originalEvent.preventDefault();
        event.originalEvent.stopPropagation();
      }
      finishNewTargetArea();
    });
  }

  document.addEventListener("keydown", (event) => {
    if ((!newTargetActive || newTargetPending) && (!targetOrderActive || targetOrderPending)) {
      return;
    }
    const tagName = String(event.target?.tagName || "").toLowerCase();
    if (tagName === "input" || tagName === "textarea" || tagName === "select") {
      return;
    }
    if (event.key === "Escape") {
      event.preventDefault();
      if (targetOrderActive) {
        setTargetOrderActive(false);
        setStatusMessage("목표지역 순서 변경 취소");
      } else {
        setNewTargetActive(false);
        setStatusMessage("신규 목표지역 입력 취소");
      }
      return;
    }
    if (event.key === "Backspace" || ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === "z")) {
      event.preventDefault();
      if (targetOrderActive) {
        undoTargetOrderSelection();
      } else {
        undoNewTargetPoint();
      }
    }
  });

  setMissionReady(false);
  window.setMissionPlanReady = setMissionReady;

  if (seedSelect && seedModeInputs.length) {
    const syncSeedMode = () => {
      const active = seedModeInputs.find((input) => input.checked);
      const fixed = active && active.value === "fixed";
      seedSelect.disabled = !fixed;
    };
    seedModeInputs.forEach((input) => {
      input.addEventListener("change", syncSeedMode);
    });
    syncSeedMode();
  }
};
