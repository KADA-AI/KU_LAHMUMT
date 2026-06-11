/**
 * Map view controller — renders flight paths, reference zones, input areas,
 * and waypoint popups on a MapLibre map.
 */

import { AGENT_COLORS, AGENT_LABELS } from "./palette.js";

const PATH_SOURCE_PREFIX = "la-path-";
const PATH_LINE_PREFIX = "la-line-";
const PATH_POINT_PREFIX = "la-point-";
const PATH_LABEL_PREFIX = "la-label-";
const REF_SOURCE = "la-reference";
const REF_FILL = "la-ref-fill";
const REF_LINE = "la-ref-line";
const REF_MARKER = "la-ref-markers";
const AREA_SOURCE = "la-areas";
const AREA_FILL = "la-area-fill";
const AREA_LINE = "la-area-line";
const TRACK_LINE_PREFIX = "la-track-line-";
const TRACK_SRC_PREFIX = "la-track-src-";
const FOOTPRINT_SRC_PREFIX = "la-footprint-src-";
const FOOTPRINT_FILL_PREFIX = "la-footprint-fill-";
const FOOTPRINT_LINE_PREFIX = "la-footprint-line-";

export const createMapController = (map) => {
  const activeLayers = new Set();
  const activeSources = new Set();
  const trackLayers = new Set();
  const trackSources = new Set();
  const agentVisibility = {};
  let popup = null;

  // ---- Reference zones (GeoJSON from server) ----

  const loadReferenceGeoJSON = (geojson) => {
    if (!geojson) return;
    _removeLayers(REF_SOURCE, [REF_FILL, REF_LINE, REF_MARKER]);

    map.addSource(REF_SOURCE, { type: "geojson", data: geojson });

    map.addLayer({
      id: REF_FILL, type: "fill", source: REF_SOURCE,
      filter: ["==", ["geometry-type"], "Polygon"],
      paint: {
        "fill-color": ["match", ["get", "areaType"], "prohibitedArea", "rgba(248,113,113,0.1)", "rgba(91,156,246,0.06)"],
        "fill-opacity": 0.7,
      },
    });
    map.addLayer({
      id: REF_LINE, type: "line", source: REF_SOURCE,
      filter: ["==", ["geometry-type"], "Polygon"],
      paint: {
        "line-color": ["match", ["get", "areaType"], "prohibitedArea", "rgba(248,113,113,0.5)", "rgba(91,156,246,0.35)"],
        "line-width": 1.5, "line-dasharray": [4, 3],
      },
    });
    map.addLayer({
      id: REF_MARKER, type: "circle", source: REF_SOURCE,
      filter: ["==", ["geometry-type"], "Point"],
      paint: {
        "circle-radius": 6,
        "circle-color": ["match", ["get", "pointType"], "takeOver", "#34d399", "handOver", "#fbbf24", "#888"],
        "circle-stroke-width": 2, "circle-stroke-color": "rgba(0,0,0,0.4)",
      },
    });
  };

  // ---- Input mission areas (GeoJSON) ----

  const loadAreaGeoJSON = (geojson) => {
    if (!geojson) return;
    _removeLayers(AREA_SOURCE, [AREA_FILL, AREA_LINE]);

    map.addSource(AREA_SOURCE, { type: "geojson", data: geojson });

    map.addLayer({
      id: AREA_FILL, type: "fill", source: AREA_SOURCE,
      filter: ["==", ["geometry-type"], "Polygon"],
      paint: {
        "fill-color": [
          "interpolate", ["linear"], ["coalesce", ["get", "coveragePercent"], 0],
          0, "rgba(248,113,113,0.13)",
          50, "rgba(251,191,36,0.13)",
          100, "rgba(52,211,153,0.14)",
        ],
        "fill-opacity": 0.75,
      },
    });
    map.addLayer({
      id: AREA_LINE, type: "line", source: AREA_SOURCE,
      paint: {
        "line-color": [
          "interpolate", ["linear"], ["coalesce", ["get", "coveragePercent"], 0],
          0, "rgba(248,113,113,0.55)",
          50, "rgba(251,191,36,0.58)",
          100, "rgba(52,211,153,0.62)",
        ],
        "line-width": 1.4,
      },
    });
  };

  // ---- Flight paths from resolved plan data ----

  const showPlanResolved = (plan, scenario) => {
    clearPaths();
    const resolved = plan.resolved || {};
    const aircraftMap = resolved.aircraft || {};
    const allCoords = [];

    for (const [aidStr, info] of Object.entries(aircraftMap)) {
      const aid = Number(aidStr);
      const color = AGENT_COLORS[aid] || "#888";
      const label = AGENT_LABELS[aid] || `AC${aid}`;
      const lineFeatures = [];
      const pointFeatures = [];

      for (const pathEntry of info.paths || []) {
        const coords = pathEntry.coordinates || [];
        if (coords.length < 2) continue;

        lineFeatures.push({
          type: "Feature",
          geometry: { type: "LineString", coordinates: coords },
          properties: { aircraftId: aid, pathId: pathEntry.pathID },
        });

        // Also load waypoints from the full flight path data
        const fpData = (scenario.flightPaths || {})[String(pathEntry.pathID)];
        const waypoints = fpData ? (fpData.waypoints || []) : [];
        for (const wp of waypoints) {
          const lon = wp.longitude ?? wp.lon;
          const lat = wp.latitude ?? wp.lat;
          if (lon == null || lat == null) continue;
          allCoords.push([lon, lat]);
          pointFeatures.push({
            type: "Feature",
            geometry: { type: "Point", coordinates: [lon, lat] },
            properties: {
              waypointID: wp.waypointID ?? "",
              pathID: pathEntry.pathID,
              aircraftId: aid,
              agent: label,
              lat: Number(lat).toFixed(6),
              lon: Number(lon).toFixed(6),
              altitude: wp.altitude ?? "",
              speed: wp.speed ?? "",
              eta: wp.eta ?? "",
              passType: wp.waypointPassType ?? "",
              lahType: wp.lahType ?? "",
              attackTarget: wp.attack?.targetID ?? "",
              attackWeapon: wp.attack?.weaponType ?? "",
              hoverTime: wp.hovering?.time ?? "",
              loiterRadius: wp.loiter?.radius ?? "",
              loiterTime: wp.loiter?.time ?? "",
            },
          });
        }

        // If no waypoints from fpData, use coordinates as fallback
        if (pointFeatures.length === 0) {
          for (const c of coords) allCoords.push(c);
        }
      }

      if (lineFeatures.length === 0 && pointFeatures.length === 0) continue;

      const srcId = PATH_SOURCE_PREFIX + aid;
      const lineId = PATH_LINE_PREFIX + aid;
      const pointId = PATH_POINT_PREFIX + aid;

      map.addSource(srcId, {
        type: "geojson",
        data: { type: "FeatureCollection", features: [...lineFeatures, ...pointFeatures] },
      });
      activeSources.add(srcId);

      map.addLayer({
        id: lineId, type: "line", source: srcId,
        filter: ["==", ["geometry-type"], "LineString"],
        paint: { "line-color": color, "line-width": 2.5, "line-opacity": 0.85 },
        layout: { "line-cap": "round", "line-join": "round" },
      });
      activeLayers.add(lineId);

      map.addLayer({
        id: pointId, type: "circle", source: srcId,
        filter: ["==", ["geometry-type"], "Point"],
        paint: {
          "circle-radius": 3.5, "circle-color": color,
          "circle-opacity": 0.9, "circle-stroke-width": 1, "circle-stroke-color": "rgba(0,0,0,0.35)",
        },
      });
      activeLayers.add(pointId);

      // WP 번호 라벨
      const labelId = PATH_LABEL_PREFIX + aid;
      map.addLayer({
        id: labelId, type: "symbol", source: srcId,
        filter: ["==", ["geometry-type"], "Point"],
        layout: {
          "text-field": ["to-string", ["get", "waypointID"]],
          "text-size": 10,
          "text-offset": [0, -1.2],
          "text-anchor": "bottom",
          "text-allow-overlap": false,
          "text-ignore-placement": false,
          "text-optional": true,
          "text-font": ["Noto Sans Regular"],
        },
        paint: {
          "text-color": color,
          "text-halo-color": "rgba(10,13,16,0.85)",
          "text-halo-width": 1.5,
        },
      });
      activeLayers.add(labelId);

      map.on("click", pointId, (e) => {
        if (!e.features?.length) return;
        const f = e.features[0];
        const p = f.properties;
        if (popup) popup.remove();
        popup = new maplibregl.Popup({ offset: 12, closeButton: true, maxWidth: "260px" })
          .setLngLat(f.geometry.coordinates.slice())
          .setHTML(wpPopup(p))
          .addTo(map);
      });
      map.on("mouseenter", pointId, () => { map.getCanvas().style.cursor = "pointer"; });
      map.on("mouseleave", pointId, () => { map.getCanvas().style.cursor = ""; });
    }

    if (allCoords.length > 0) fitTo(allCoords);
  };

  const clearPaths = () => {
    if (popup) { popup.remove(); popup = null; }
    for (const id of activeLayers) { if (map.getLayer(id)) map.removeLayer(id); }
    activeLayers.clear();
    for (const id of activeSources) { if (map.getSource(id)) map.removeSource(id); }
    activeSources.clear();
  };

  const setAgentVisible = (aircraftId, visible) => {
    agentVisibility[Number(aircraftId)] = !!visible;
    for (const prefix of [PATH_LINE_PREFIX, PATH_POINT_PREFIX, PATH_LABEL_PREFIX, TRACK_LINE_PREFIX, FOOTPRINT_FILL_PREFIX, FOOTPRINT_LINE_PREFIX]) {
      const id = prefix + aircraftId;
      if (map.getLayer(id)) map.setLayoutProperty(id, "visibility", visible ? "visible" : "none");
    }
    if (trackMarkers[aircraftId]?.getElement) {
      trackMarkers[aircraftId].getElement().style.display = visible ? "" : "none";
    }
  };

  const flyToPath = (coords) => { if (coords?.length) fitTo(coords); };

  // ---- Track (항적) rendering ----

  let trackData = null;
  let footprintData = null;
  let trackTimestamps = null;
  let trackMarkers = {};
  let currentTrackFrame = 0;
  let trackAllMode = false;

  const loadTracks = (payload) => {
    clearTracks();
    const playback = payload?.tracks ? payload : { tracks: payload || {}, footprints: {} };
    const tracks = playback.tracks || {};
    const footprints = playback.footprints || {};
    if (Object.keys(tracks).length === 0 && Object.keys(footprints).length === 0) return null;
    trackData = tracks;
    footprintData = footprints;
    trackAllMode = false;

    const tsSet = new Set();
    for (const t of Object.values(tracks)) {
      for (const ts of t.timestamps || []) tsSet.add(ts);
    }
    for (const f of Object.values(footprints)) {
      for (const ts of f.timestamps || []) tsSet.add(ts);
    }
    trackTimestamps = [...tsSet].sort((a, b) => a - b);
    if (trackTimestamps.length === 0) return null;

    for (const [aidStr, t] of Object.entries(tracks)) {
      const aid = Number(aidStr);
      const color = AGENT_COLORS[aid] || "#888";
      const srcId = TRACK_SRC_PREFIX + aid;
      const lineId = TRACK_LINE_PREFIX + aid;

      // 항적 점선 (지나온 길)
      map.addSource(srcId, {
        type: "geojson",
        data: { type: "Feature", geometry: { type: "LineString", coordinates: [] } },
      });
      trackSources.add(srcId);

      map.addLayer({
        id: lineId, type: "line", source: srcId,
        paint: { "line-color": color, "line-width": 2.2, "line-opacity": 0.6, "line-dasharray": [4, 3] },
        layout: { "line-cap": "round", "line-join": "round", "visibility": agentVisibility[aid] === false ? "none" : "visible" },
      });
      trackLayers.add(lineId);

      if (footprints[aidStr]) {
        const footprintSrcId = FOOTPRINT_SRC_PREFIX + aid;
        const fillId = FOOTPRINT_FILL_PREFIX + aid;
        const footprintLineId = FOOTPRINT_LINE_PREFIX + aid;
        map.addSource(footprintSrcId, {
          type: "geojson",
          data: { type: "FeatureCollection", features: [] },
        });
        trackSources.add(footprintSrcId);
        map.addLayer({
          id: fillId, type: "fill", source: footprintSrcId,
          paint: { "fill-color": color, "fill-opacity": 0.12 },
          layout: { "visibility": agentVisibility[aid] === false ? "none" : "visible" },
        });
        map.addLayer({
          id: footprintLineId, type: "line", source: footprintSrcId,
          paint: { "line-color": color, "line-width": 1.1, "line-opacity": 0.48 },
          layout: { "visibility": agentVisibility[aid] === false ? "none" : "visible" },
        });
        trackLayers.add(fillId);
        trackLayers.add(footprintLineId);
      }
    }

    // 마지막 프레임으로 초기화 (현재 위치 마커 표시)
    const info = { totalFrames: trackTimestamps.length, minTs: trackTimestamps[0], maxTs: trackTimestamps[trackTimestamps.length - 1] };
    setTrackFrame(0);
    return info;
  };

  const setTrackFrame = (frameIndex) => {
    if (!trackTimestamps) return;
    currentTrackFrame = Math.max(0, Math.min(Number(frameIndex) || 0, trackTimestamps.length - 1));
    const targetTs = trackTimestamps[currentTrackFrame];

    for (const [aidStr, t] of Object.entries(trackData || {})) {
      const aid = Number(aidStr);
      const color = AGENT_COLORS[aid] || "#888";
      const label = AGENT_LABELS[aid] || `AC${aid}`;

      const idx = trackAllMode ? (t.coordinates || []).length - 1 : _lastIndexAtOrBefore(t.timestamps || [], targetTs);
      const coord = (t.coordinates || [])[idx];
      if (!coord) continue;

      // Update partial trail line
      const srcId = TRACK_SRC_PREFIX + aid;
      const src = map.getSource(srcId);
      if (src) {
        src.setData({
          type: "Feature",
          geometry: {
            type: "LineString",
            coordinates: trackAllMode ? (t.coordinates || []) : (t.coordinates || []).slice(0, idx + 1),
          },
        });
      }

      // Update/create marker for current position
      if (!trackMarkers[aid]) {
        const el = document.createElement("div");
        el.className = "track-marker";
        el.style.cssText = `width:14px;height:14px;border-radius:50%;background:${color};border:2px solid #fff;box-shadow:0 0 6px ${color};`;
        el.title = label;
        el.style.display = agentVisibility[aid] === false ? "none" : "";
        trackMarkers[aid] = new maplibregl.Marker({ element: el }).setLngLat(coord).addTo(map);
      } else {
        trackMarkers[aid].setLngLat(coord);
      }
    }

    for (const [aidStr, data] of Object.entries(footprintData || {})) {
      const src = map.getSource(FOOTPRINT_SRC_PREFIX + Number(aidStr));
      if (!src) continue;
      src.setData({
        type: "FeatureCollection",
        features: _footprintFeatures(aidStr, data, targetTs, trackAllMode),
      });
    }
  };

  const clearTracks = () => {
    for (const m of Object.values(trackMarkers)) m.remove();
    trackMarkers = {};
    for (const id of trackLayers) { if (map.getLayer(id)) map.removeLayer(id); }
    trackLayers.clear();
    for (const id of trackSources) { if (map.getSource(id)) map.removeSource(id); }
    trackSources.clear();
    trackData = null;
    footprintData = null;
    trackTimestamps = null;
    currentTrackFrame = 0;
    trackAllMode = false;
  };

  const setTrackAll = (enabled) => {
    trackAllMode = !!enabled;
    setTrackFrame(currentTrackFrame);
  };

  function _lastIndexAtOrBefore(timestamps, targetTs) {
    let idx = 0;
    for (let i = 0; i < timestamps.length; i++) {
      if (timestamps[i] <= targetTs) idx = i;
      else break;
    }
    return idx;
  }

  function _footprintFeatures(aidStr, data, targetTs, showAll) {
    const timestamps = data.timestamps || [];
    const polygons = data.polygons || [];
    const features = [];
    for (let i = 0; i < polygons.length; i++) {
      if (!showAll && timestamps[i] > targetTs) break;
      const coords = polygons[i];
      if (!coords || coords.length < 4) continue;
      features.push({
        type: "Feature",
        geometry: { type: "Polygon", coordinates: [coords] },
        properties: { aircraftId: Number(aidStr), timestamp: timestamps[i] || 0, index: i },
      });
    }
    return features;
  }

  function fitTo(coords) {
    const b = coords.reduce((a, c) => {
      a[0] = Math.min(a[0], c[0]); a[1] = Math.min(a[1], c[1]);
      a[2] = Math.max(a[2], c[0]); a[3] = Math.max(a[3], c[1]);
      return a;
    }, [Infinity, Infinity, -Infinity, -Infinity]);
    map.fitBounds([[b[0], b[1]], [b[2], b[3]]], {
      padding: { top: 80, bottom: 100, left: 360, right: 40 }, maxZoom: 15, duration: 800,
    });
  }

  function _removeLayers(srcId, layerIds) {
    for (const id of layerIds) { if (map.getLayer(id)) map.removeLayer(id); }
    if (map.getSource(srcId)) map.removeSource(srcId);
  }

  return { loadReferenceGeoJSON, loadAreaGeoJSON, showPlanResolved, clearPaths, setAgentVisible, flyToPath, loadTracks, setTrackFrame, setTrackAll, clearTracks };
};

function wpPopup(p) {
  const s = (v) => `<span style="color:rgba(224,232,240,0.55)">`;
  let html = `<div style="font-weight:700;margin-bottom:4px;color:#5b9cf6">${p.agent} — WP ${p.waypointID}</div>`;
  html += `<div style="font-size:11px;color:rgba(224,232,240,0.7)">Path: ${p.pathID}</div>`;
  html += `<div style="font-size:11px;margin-top:4px">Lat: ${p.lat} Lon: ${p.lon}</div>`;
  if (p.altitude) html += `<div style="font-size:11px">Alt: ${p.altitude}m</div>`;
  if (p.speed) html += `<div style="font-size:11px">Speed: ${p.speed}</div>`;
  if (p.eta) html += `<div style="font-size:11px">ETA: ${p.eta}s</div>`;

  // Pass type
  const PASS = { 1: "Fly-by", 2: "Loiter", 3: "Fly-Over" };
  if (p.passType && PASS[p.passType]) {
    html += `<div style="font-size:11px;margin-top:2px;color:#5b9cf6">통과: ${PASS[p.passType]}</div>`;
  }

  // LAH type
  if (p.lahType === "attack") {
    let atk = "공격";
    if (p.attackTarget) atk += ` 표적#${p.attackTarget}`;
    if (p.attackWeapon) atk += ` 무장#${p.attackWeapon}`;
    html += `<div style="font-size:11px;margin-top:2px;color:#f87171;font-weight:600">${atk}</div>`;
  } else if (p.lahType === "hovering") {
    html += `<div style="font-size:11px;margin-top:2px;color:#fbbf24">호버링 ${p.hoverTime || ""}s</div>`;
  } else if (p.lahType === "loiter") {
    let ltr = "선회";
    if (p.loiterRadius) ltr += ` R${p.loiterRadius}m`;
    if (p.loiterTime) ltr += ` ${p.loiterTime}s`;
    html += `<div style="font-size:11px;margin-top:2px;color:#34d399">${ltr}</div>`;
  }

  return html;
}
