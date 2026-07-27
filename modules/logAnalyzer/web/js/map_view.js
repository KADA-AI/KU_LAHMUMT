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
const AREA_LABEL = "la-area-label";
const BATTLE_ANCHOR_RING = "la-battle-anchor-ring";
const BATTLE_ANCHOR_DOT = "la-battle-anchor-dot";
const BATTLE_ANCHOR_LABEL = "la-battle-anchor-label";
const TRACK_LINE_PREFIX = "la-track-line-";
const TRACK_SRC_PREFIX = "la-track-src-";
const FOOTPRINT_SRC_PREFIX = "la-footprint-src-";
const FOOTPRINT_FILL_PREFIX = "la-footprint-fill-";
const FOOTPRINT_LINE_PREFIX = "la-footprint-line-";
const ALLOC_SRC_PREFIX = "la-alloc-src-";
const ALLOC_FILL_PREFIX = "la-alloc-fill-";
const ALLOC_OUTLINE_PREFIX = "la-alloc-outline-";
const ALLOC_CORRIDOR_PREFIX = "la-alloc-corridor-";
const ALLOC_LABEL_PREFIX = "la-alloc-label-";
const ALLOC_POINT_PREFIX = "la-alloc-point-";
const TARGET_SOURCE = "la-targets";
const TARGET_CIRCLE = "la-target-circle";
const TARGET_LABEL = "la-target-label";
const CURRENT_WP_SOURCE = "la-current-waypoints";
const CURRENT_WP_RING = "la-current-waypoint-ring";
const CURRENT_WP_LABEL = "la-current-waypoint-label";
const ATTACK_SOURCE = "la-lah-attack-points";
const ATTACK_HALO = "la-lah-attack-halo";
const ATTACK_POINT = "la-lah-attack-point";
const ATTACK_LABEL = "la-lah-attack-label";
const MAX_PLAYBACK_FRAMES = 100_000;

export const createMapController = (map) => {
  const activeLayers = new Set();
  const activeSources = new Set();
  const trackLayers = new Set();
  const trackSources = new Set();
  const agentVisibility = {};
  const knownAgentIds = new Set();
  const layerVisibility = {
    paths: true,
    allocations: true,
    tracks: true,
    footprints: true,
    inputAreas: true,
    reference: true,
    targets: true,
  };
  let missionFocus = null;
  let missionFocusTimestampRange;
  let trackTimeFocus = null;
  let selectedInputPackageId = null;
  let waypointLookup = {};
  let lastTrackSnapshot = null;
  let popup = null;
  let battleAnchorHandlers = null;
  const pathLayerHandlers = new Map();

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
    _applyReferenceFilters();
    _applyGlobalLayerState();
  };

  // ---- Input mission areas (GeoJSON) ----

  const loadAreaGeoJSON = (geojson) => {
    if (!geojson) return;
    if (battleAnchorHandlers) {
      map.off("click", BATTLE_ANCHOR_RING, battleAnchorHandlers.clickHandler);
      map.off("mouseenter", BATTLE_ANCHOR_RING, battleAnchorHandlers.enterHandler);
      map.off("mouseleave", BATTLE_ANCHOR_RING, battleAnchorHandlers.leaveHandler);
      battleAnchorHandlers = null;
    }
    _removeLayers(AREA_SOURCE, [
      AREA_FILL,
      AREA_LINE,
      AREA_LABEL,
      BATTLE_ANCHOR_RING,
      BATTLE_ANCHOR_DOT,
      BATTLE_ANCHOR_LABEL,
    ]);

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
    map.addLayer({
      id: AREA_LABEL, type: "symbol", source: AREA_SOURCE,
      filter: _areaRegionLabelFilter(),
      layout: {
        "text-field": [
          "concat",
          ["coalesce", ["get", "regionDisplayLabel"], ["get", "regionLabel"]],
          " · ",
          ["get", "shapeLabel"],
          "\nInput ",
          ["to-string", ["get", "inputMissionID"]],
        ],
        "text-size": 11,
        "text-font": ["Noto Sans Regular"],
        "text-anchor": "center",
        "text-allow-overlap": false,
        "text-optional": true,
      },
      paint: {
        "text-color": _regionColorExpression(),
        "text-halo-color": "rgba(10,13,16,0.94)",
        "text-halo-width": 2,
      },
    });
    map.addLayer({
      id: BATTLE_ANCHOR_RING, type: "circle", source: AREA_SOURCE,
      filter: _battleAnchorFilter(),
      paint: {
        "circle-radius": ["case", ["==", ["get", "anchorSource"], "area-centroid-fallback"], 18, 12],
        "circle-color": "rgba(0,0,0,0)",
        "circle-stroke-width": 3,
        "circle-stroke-color": "#f59e0b",
        "circle-blur": 0.05,
      },
    });
    map.addLayer({
      id: BATTLE_ANCHOR_DOT, type: "circle", source: AREA_SOURCE,
      filter: _battleAnchorFilter(),
      paint: {
        "circle-radius": 4.5,
        "circle-color": "#f59e0b",
        "circle-stroke-width": 1.5,
        "circle-stroke-color": "#fff7ed",
      },
    });
    map.addLayer({
      id: BATTLE_ANCHOR_LABEL, type: "symbol", source: AREA_SOURCE,
      filter: _battleAnchorFilter(),
      layout: {
        "text-field": [
          "concat",
          ["get", "landmarkLabel"],
          " · Input ", ["to-string", ["get", "inputMissionID"]],
          "\n", ["get", "anchorSourceLabel"],
        ],
        "text-size": 11,
        "text-font": ["Noto Sans Regular"],
        "text-offset": [1.15, -1.3],
        "text-anchor": "bottom-left",
        "text-allow-overlap": true,
      },
      paint: {
        "text-color": "#fbbf24",
        "text-halo-color": "rgba(10,13,16,0.94)",
        "text-halo-width": 2,
      },
    });
    const clickHandler = (event) => {
      if (!event.features?.length) return;
      // A LAH attack waypoint can intentionally coincide with the battle
      // position anchor.  In that case the attack popup is more actionable
      // and already includes the anchor-selection context, so do not replace
      // it with the generic anchor popup.
      const overlappingAttack = map.getLayer(ATTACK_HALO)
        ? (map.queryRenderedFeatures?.(event.point, { layers: [ATTACK_HALO] }) || [])
        : [];
      if (overlappingAttack.length) return;
      const feature = event.features[0];
      if (popup) popup.remove();
      popup = new maplibregl.Popup({ offset: 14, closeButton: true, maxWidth: "290px" })
        .setLngLat(feature.geometry.coordinates.slice())
        .setHTML(battleAnchorPopup(feature.properties))
        .addTo(map);
    };
    const enterHandler = () => { map.getCanvas().style.cursor = "pointer"; };
    const leaveHandler = () => { map.getCanvas().style.cursor = ""; };
    map.on("click", BATTLE_ANCHOR_RING, clickHandler);
    map.on("mouseenter", BATTLE_ANCHOR_RING, enterHandler);
    map.on("mouseleave", BATTLE_ANCHOR_RING, leaveHandler);
    battleAnchorHandlers = { clickHandler, enterHandler, leaveHandler };
    _applyInputAreaFilters();
    _applyGlobalLayerState();
  };

  // ---- Flight paths from resolved plan data ----

  const showPlanResolved = (plan, scenario) => {
    clearPaths();
    waypointLookup = {};
    const resolved = plan.resolved || {};
    const aircraftMap = resolved.aircraft || {};
    const allCoords = [];
    const attackFeatures = [];
    const planLandmarks = (scenario?.missionLandmarkFeatures?.features || []).filter(
      (feature) => String(feature?.properties?.inputMissionPackageID) === String(plan.inputMissionPackageID),
    );

    // 1st pass — per-aircraft assigned areas/corridors, below the flight paths.
    for (const [aidStr, info] of Object.entries(aircraftMap)) {
      const aid = Number(aidStr);
      knownAgentIds.add(aid);
      const color = AGENT_COLORS[aid] || "#888";
      const features = _allocationFeatures(aid, info);
      if (features.length === 0) continue;

      for (const f of features) {
        if (f.geometry.type === "Polygon") {
          for (const c of f.geometry.coordinates[0]) allCoords.push(c);
        } else if (f.geometry.type === "LineString") {
          for (const c of f.geometry.coordinates) allCoords.push(c);
        }
      }

      const srcId = ALLOC_SRC_PREFIX + aid;
      const visibility = agentVisibility[aid] === false ? "none" : "visible";
      map.addSource(srcId, {
        type: "geojson",
        data: { type: "FeatureCollection", features },
      });
      activeSources.add(srcId);

      map.addLayer({
        id: ALLOC_FILL_PREFIX + aid, type: "fill", source: srcId,
        filter: ["==", ["geometry-type"], "Polygon"],
        paint: {
          "fill-color": color,
          "fill-opacity": ["case", ["==", ["get", "isDone"], 1], 0.05, 0.14],
        },
        layout: { visibility },
      });
      activeLayers.add(ALLOC_FILL_PREFIX + aid);

      map.addLayer({
        id: ALLOC_OUTLINE_PREFIX + aid, type: "line", source: srcId,
        filter: ["==", ["geometry-type"], "Polygon"],
        paint: {
          "line-color": color,
          "line-width": 1.8,
          "line-opacity": ["case", ["==", ["get", "isDone"], 1], 0.35, 0.9],
          "line-dasharray": [5, 2.5],
        },
        layout: { visibility },
      });
      activeLayers.add(ALLOC_OUTLINE_PREFIX + aid);

      map.addLayer({
        id: ALLOC_CORRIDOR_PREFIX + aid, type: "line", source: srcId,
        filter: ["all", ["==", ["geometry-type"], "LineString"], ["==", ["get", "kind"], "corridor"]],
        paint: {
          "line-color": color,
          "line-width": 7,
          "line-opacity": ["case", ["==", ["get", "isDone"], 1], 0.1, 0.22],
        },
        layout: { "line-cap": "round", "line-join": "round", visibility },
      });
      activeLayers.add(ALLOC_CORRIDOR_PREFIX + aid);

      map.addLayer({
        id: ALLOC_LABEL_PREFIX + aid, type: "symbol", source: srcId,
        filter: ["==", ["get", "kind"], "label"],
        layout: {
          "text-field": ["get", "labelText"],
          "text-size": 11,
          "text-font": ["Noto Sans Regular"],
          "text-anchor": "center",
          "text-allow-overlap": false,
          "text-optional": true,
          visibility,
        },
        paint: {
          "text-color": color,
          "text-halo-color": "rgba(10,13,16,0.9)",
          "text-halo-width": 1.6,
        },
      });
      activeLayers.add(ALLOC_LABEL_PREFIX + aid);

      map.addLayer({
        id: ALLOC_POINT_PREFIX + aid, type: "circle", source: srcId,
        filter: ["==", ["get", "kind"], "assignment-point"],
        paint: {
          "circle-radius": 6,
          "circle-color": color,
          "circle-opacity": ["case", ["==", ["get", "isDone"], 1], 0.35, 0.9],
          "circle-stroke-width": 2,
          "circle-stroke-color": "rgba(255,255,255,0.82)",
        },
        layout: { visibility },
      });
      activeLayers.add(ALLOC_POINT_PREFIX + aid);
    }

    for (const [aidStr, info] of Object.entries(aircraftMap)) {
      const aid = Number(aidStr);
      knownAgentIds.add(aid);
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
          properties: {
            aircraftId: aid,
            pathId: pathEntry.pathID,
            missionId: pathEntry.missionID ?? "",
            inputMissionId: pathEntry.inputMissionID ?? "",
            missionType: pathEntry.missionType ?? "",
          },
        });

        // Also load waypoints from the full flight path data
        const fpData = (scenario.flightPaths || {})[String(pathEntry.pathID)];
        const waypoints = fpData ? (fpData.waypoints || []) : [];
        let meaningfulAttackCount = 0;
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
              missionId: pathEntry.missionID ?? "",
              inputMissionId: pathEntry.inputMissionID ?? "",
              missionType: pathEntry.missionType ?? "",
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
          waypointLookup[aid] ||= {};
          waypointLookup[aid][String(wp.waypointID)] = {
            coordinate: [lon, lat],
            missionId: pathEntry.missionID,
            pathId: pathEntry.pathID,
          };
          if (aid >= 1 && aid <= 3 && _isMeaningfulAttackWaypoint(wp)) {
            meaningfulAttackCount += 1;
            attackFeatures.push(_attackPointFeature({
              aid,
              missionId: pathEntry.missionID,
              inputMissionId: pathEntry.inputMissionID,
              pathId: pathEntry.pathID,
              waypoint: wp,
              coordinate: [lon, lat],
              source: "waypoint-attack",
              anchorContext: _matchingBattleAnchor(planLandmarks, [lon, lat]),
            }));
          }
        }

        // Older plan logs can omit the attack sub-structure even though the
        // individual mission is target attack (Type 2).  Retain an honest
        // fallback marker from individualMissionInfo.coordinateList.
        if (aid >= 1 && aid <= 3 && Number(pathEntry.missionType) === 2 && meaningfulAttackCount === 0) {
          const missionDef = (info.missions || []).find(
            (mission) => String(mission.id) === String(pathEntry.missionID),
          );
          const fallback = missionDef?.coordinateList?.[0];
          if (fallback?.length >= 2) {
            attackFeatures.push(_attackPointFeature({
              aid,
              missionId: pathEntry.missionID,
              inputMissionId: pathEntry.inputMissionID,
              pathId: pathEntry.pathID,
              waypoint: {
                waypointID: "-",
                attack: { targetID: missionDef?.targetID ?? "", weaponType: "" },
              },
              coordinate: fallback,
              source: "mission-coordinate",
              anchorContext: _matchingBattleAnchor(planLandmarks, fallback),
            }));
          }
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

      const clickHandler = (e) => {
        if (!e.features?.length) return;
        const f = e.features[0];
        const p = f.properties;
        if (popup) popup.remove();
        popup = new maplibregl.Popup({ offset: 12, closeButton: true, maxWidth: "260px" })
          .setLngLat(f.geometry.coordinates.slice())
          .setHTML(wpPopup(p))
          .addTo(map);
      };
      const enterHandler = () => { map.getCanvas().style.cursor = "pointer"; };
      const leaveHandler = () => { map.getCanvas().style.cursor = ""; };
      map.on("click", pointId, clickHandler);
      map.on("mouseenter", pointId, enterHandler);
      map.on("mouseleave", pointId, leaveHandler);
      pathLayerHandlers.set(pointId, { clickHandler, enterHandler, leaveHandler });
    }

    _loadAttackPointFeatures(attackFeatures);

    _ensureCurrentWaypointLayers();
    _applyAllLayerStates();
    if (lastTrackSnapshot) _updateCurrentWaypointFeatures(lastTrackSnapshot);

    // Keep target markers above freshly added plan layers.
    if (map.getLayer(TARGET_CIRCLE)) map.moveLayer(TARGET_CIRCLE);
    if (map.getLayer(TARGET_LABEL)) map.moveLayer(TARGET_LABEL);

    if (allCoords.length > 0) fitTo(allCoords);
  };

  const _loadAttackPointFeatures = (features) => {
    if (!features.length) return;
    const coordinateGroups = new Map();
    for (const feature of features) {
      const key = feature.geometry.coordinates.map((value) => Number(value).toFixed(7)).join(":");
      if (!coordinateGroups.has(key)) coordinateGroups.set(key, []);
      coordinateGroups.get(key).push(feature);
    }
    for (const group of coordinateGroups.values()) {
      group.forEach((feature) => {
        feature.properties.showGroupLabel = 0;
        feature.properties.individualLabelText = feature.properties.labelText;
        feature.properties.groupLabelText = feature.properties.labelText;
      });
      const primary = group[0];
      primary.properties.showGroupLabel = 1;
      if (group.length > 1) {
        const missionText = group
          .map((feature) => `${feature.properties.aircraftLabel} T${feature.properties.targetID || "-"} ${feature.properties.weaponLabel}`)
          .join(" / ");
        const altitude = primary.properties.altitude === "" ? "" : ` · ${primary.properties.altitude}m`;
        primary.properties.groupLabelText = `공격 Point · ${group.length}개 LAH 임무${altitude}\n${missionText}`;
      }
    }
    map.addSource(ATTACK_SOURCE, {
      type: "geojson",
      data: { type: "FeatureCollection", features },
    });
    activeSources.add(ATTACK_SOURCE);

    map.addLayer({
      id: ATTACK_HALO, type: "circle", source: ATTACK_SOURCE,
      filter: ["all", _attackPointFilter(), ["==", ["get", "showGroupLabel"], 1]],
      paint: {
        "circle-radius": 13,
        "circle-color": "rgba(0,0,0,0)",
        "circle-stroke-width": 3.5,
        "circle-stroke-color": "#fb7185",
        "circle-blur": 0.06,
      },
    });
    map.addLayer({
      id: ATTACK_POINT, type: "circle", source: ATTACK_SOURCE,
      filter: _attackPointFilter(),
      paint: {
        "circle-radius": 5.5,
        "circle-color": "#e11d48",
        "circle-stroke-width": 2,
        "circle-stroke-color": "#fff1f2",
      },
    });
    map.addLayer({
      id: ATTACK_LABEL, type: "symbol", source: ATTACK_SOURCE,
      filter: _attackPointFilter(),
      layout: {
        "text-field": ["get", "groupLabelText"],
        "text-size": 11,
        "text-font": ["Noto Sans Regular"],
        "text-offset": [1.15, 1.3],
        "text-anchor": "top-left",
        "text-allow-overlap": true,
      },
      paint: {
        "text-color": "#fb7185",
        "text-halo-color": "rgba(10,13,16,0.96)",
        "text-halo-width": 2,
      },
    });
    activeLayers.add(ATTACK_HALO);
    activeLayers.add(ATTACK_POINT);
    activeLayers.add(ATTACK_LABEL);

    const clickHandler = (event) => {
      if (!event.features?.length) return;
      const rendered = map.queryRenderedFeatures?.(event.point, { layers: [ATTACK_HALO] }) || [];
      const featuresAtPoint = rendered.length ? rendered : event.features;
      const feature = featuresAtPoint[0];
      if (popup) popup.remove();
      popup = new maplibregl.Popup({ offset: 15, closeButton: true, maxWidth: "310px" })
        .setLngLat(feature.geometry.coordinates.slice())
        .setHTML(attackPointsPopup(featuresAtPoint.map((item) => item.properties)))
        .addTo(map);
    };
    const enterHandler = () => { map.getCanvas().style.cursor = "pointer"; };
    const leaveHandler = () => { map.getCanvas().style.cursor = ""; };
    map.on("click", ATTACK_HALO, clickHandler);
    map.on("mouseenter", ATTACK_HALO, enterHandler);
    map.on("mouseleave", ATTACK_HALO, leaveHandler);
    pathLayerHandlers.set(ATTACK_HALO, { clickHandler, enterHandler, leaveHandler });
  };

  const clearPaths = () => {
    if (popup) { popup.remove(); popup = null; }
    for (const [layerId, handlers] of pathLayerHandlers) {
      map.off("click", layerId, handlers.clickHandler);
      map.off("mouseenter", layerId, handlers.enterHandler);
      map.off("mouseleave", layerId, handlers.leaveHandler);
    }
    pathLayerHandlers.clear();
    for (const id of activeLayers) { if (map.getLayer(id)) map.removeLayer(id); }
    activeLayers.clear();
    for (const id of activeSources) { if (map.getSource(id)) map.removeSource(id); }
    activeSources.clear();
  };

  const setAgentVisible = (aircraftId, visible) => {
    agentVisibility[Number(aircraftId)] = !!visible;
    knownAgentIds.add(Number(aircraftId));
    _applyAgentLayerState(Number(aircraftId));
    _applyAttackPointFilters();
    if (trackTimestamps) setTrackFrame(currentTrackFrame);
    else if (lastTrackSnapshot) _updateCurrentWaypointFeatures(lastTrackSnapshot);
  };

  const flyToPath = (coords) => { if (coords?.length) fitTo(coords); };

  const setLayerVisibility = (kind, visible) => {
    if (!(kind in layerVisibility)) return;
    layerVisibility[kind] = !!visible;
    _applyAllLayerStates();
    if (kind === "tracks" || kind === "footprints") setTrackFrame(currentTrackFrame);
  };

  const setMissionFocus = (focus) => {
    missionFocus = focus ? {
      aircraftId: Number(focus.aircraftId),
      missionId: focus.missionId,
      pathId: focus.pathId,
      inputMissionId: focus.inputMissionId,
      waypointIds: (focus.waypointIds || []).map(Number).filter(Number.isFinite),
    } : null;
    missionFocusTimestampRange = undefined;
    _applyAllLayerStates();
    _applyInputAreaFilters();
    setTrackFrame(currentTrackFrame);
  };

  const setTrackTimeFocus = (range) => {
    const start = Number(range?.[0]);
    const end = Number(range?.[1]);
    trackTimeFocus = Number.isFinite(start) && Number.isFinite(end) && end >= start
      ? [start, end]
      : null;
    setTrackFrame(currentTrackFrame);
  };

  const setInputMissionPackage = (packageId) => {
    selectedInputPackageId = packageId == null ? null : Number(packageId);
    _applyReferenceFilters();
    _applyInputAreaFilters();
  };

  const _focusedFilter = (baseFilter, aid) => {
    if (!missionFocus || Number(aid) !== missionFocus.aircraftId) return baseFilter;
    return ["all", baseFilter, ["==", ["to-string", ["get", "missionId"]], String(missionFocus.missionId)]];
  };

  const _applyAgentLayerState = (aid) => {
    const baseVisible = agentVisibility[aid] !== false && (!missionFocus || missionFocus.aircraftId === aid);
    const setVisible = (id, visible) => {
      if (map.getLayer(id)) map.setLayoutProperty(id, "visibility", visible ? "visible" : "none");
    };

    setVisible(PATH_LINE_PREFIX + aid, baseVisible && layerVisibility.paths);
    setVisible(PATH_POINT_PREFIX + aid, baseVisible && layerVisibility.paths);
    setVisible(PATH_LABEL_PREFIX + aid, baseVisible && layerVisibility.paths);
    setVisible(ALLOC_FILL_PREFIX + aid, baseVisible && layerVisibility.allocations);
    setVisible(ALLOC_OUTLINE_PREFIX + aid, baseVisible && layerVisibility.allocations);
    setVisible(ALLOC_CORRIDOR_PREFIX + aid, baseVisible && layerVisibility.allocations);
    setVisible(ALLOC_LABEL_PREFIX + aid, baseVisible && layerVisibility.allocations);
    setVisible(ALLOC_POINT_PREFIX + aid, baseVisible && layerVisibility.allocations);
    setVisible(TRACK_LINE_PREFIX + aid, baseVisible && layerVisibility.tracks);
    setVisible(FOOTPRINT_FILL_PREFIX + aid, baseVisible && layerVisibility.footprints);
    setVisible(FOOTPRINT_LINE_PREFIX + aid, baseVisible && layerVisibility.footprints);

    if (map.getLayer(PATH_LINE_PREFIX + aid)) map.setFilter(PATH_LINE_PREFIX + aid, _focusedFilter(["==", ["geometry-type"], "LineString"], aid));
    if (map.getLayer(PATH_POINT_PREFIX + aid)) map.setFilter(PATH_POINT_PREFIX + aid, _focusedFilter(["==", ["geometry-type"], "Point"], aid));
    if (map.getLayer(PATH_LABEL_PREFIX + aid)) map.setFilter(PATH_LABEL_PREFIX + aid, _focusedFilter(["==", ["geometry-type"], "Point"], aid));
    if (map.getLayer(ALLOC_FILL_PREFIX + aid)) map.setFilter(ALLOC_FILL_PREFIX + aid, _focusedFilter(["==", ["geometry-type"], "Polygon"], aid));
    if (map.getLayer(ALLOC_OUTLINE_PREFIX + aid)) map.setFilter(ALLOC_OUTLINE_PREFIX + aid, _focusedFilter(["==", ["geometry-type"], "Polygon"], aid));
    if (map.getLayer(ALLOC_CORRIDOR_PREFIX + aid)) map.setFilter(ALLOC_CORRIDOR_PREFIX + aid, _focusedFilter(["all", ["==", ["geometry-type"], "LineString"], ["==", ["get", "kind"], "corridor"]], aid));
    if (map.getLayer(ALLOC_LABEL_PREFIX + aid)) map.setFilter(ALLOC_LABEL_PREFIX + aid, _focusedFilter(["==", ["get", "kind"], "label"], aid));
    if (map.getLayer(ALLOC_POINT_PREFIX + aid)) map.setFilter(ALLOC_POINT_PREFIX + aid, _focusedFilter(["==", ["get", "kind"], "assignment-point"], aid));

    const marker = trackMarkers[aid]?.getElement?.();
    if (marker) marker.style.display = baseVisible && layerVisibility.tracks ? "" : "none";
  };

  const _applyGlobalLayerState = () => {
    const setVisible = (id, visible) => {
      if (map.getLayer(id)) map.setLayoutProperty(id, "visibility", visible ? "visible" : "none");
    };
    for (const id of [REF_FILL, REF_LINE, REF_MARKER]) setVisible(id, layerVisibility.reference);
    for (const id of [
      AREA_FILL,
      AREA_LINE,
      AREA_LABEL,
      BATTLE_ANCHOR_RING,
      BATTLE_ANCHOR_DOT,
      BATTLE_ANCHOR_LABEL,
    ]) setVisible(id, layerVisibility.inputAreas);
    for (const id of [TARGET_CIRCLE, TARGET_LABEL]) setVisible(id, layerVisibility.targets);
    for (const id of [
      CURRENT_WP_RING,
      CURRENT_WP_LABEL,
      ATTACK_HALO,
      ATTACK_POINT,
      ATTACK_LABEL,
    ]) setVisible(id, layerVisibility.paths);
  };

  const _applyAllLayerStates = () => {
    _applyGlobalLayerState();
    for (const aid of knownAgentIds) _applyAgentLayerState(aid);
    _applyAttackPointFilters();
  };

  const _attackPointFilter = () => {
    const clauses = [["==", ["get", "kind"], "lah-attack-point"]];
    for (const [aidText, visible] of Object.entries(agentVisibility)) {
      if (visible !== false) continue;
      clauses.push(["!=", ["to-number", ["get", "aircraftId"]], Number(aidText)]);
    }
    if (missionFocus) {
      clauses.push(["==", ["to-number", ["get", "aircraftId"]], missionFocus.aircraftId]);
      clauses.push(["==", ["to-string", ["get", "missionId"]], String(missionFocus.missionId)]);
    }
    return ["all", ...clauses];
  };

  const _applyAttackPointFilters = () => {
    const filter = _attackPointFilter();
    for (const id of [ATTACK_HALO, ATTACK_POINT]) {
      if (map.getLayer(id)) map.setFilter(id, filter);
    }
    if (map.getLayer(ATTACK_LABEL)) {
      map.setLayoutProperty(
        ATTACK_LABEL,
        "text-field",
        ["get", missionFocus ? "individualLabelText" : "groupLabelText"],
      );
      map.setFilter(ATTACK_LABEL, missionFocus
        ? filter
        : ["all", filter, ["==", ["get", "showGroupLabel"], 1]]);
    }
  };

  const _packageClause = () => selectedInputPackageId == null
    ? null
    : ["==", ["to-number", ["get", "inputMissionPackageID"]], selectedInputPackageId];

  const _applyReferenceFilters = () => {
    const packageClause = _packageClause();
    const missionAreaClause = packageClause
      ? ["any", ["!=", ["get", "areaType"], "missionArea"], packageClause]
      : null;
    if (map.getLayer(REF_FILL)) map.setFilter(REF_FILL, missionAreaClause ? ["all", ["==", ["geometry-type"], "Polygon"], missionAreaClause] : ["==", ["geometry-type"], "Polygon"]);
    if (map.getLayer(REF_LINE)) map.setFilter(REF_LINE, missionAreaClause ? ["all", ["==", ["geometry-type"], "Polygon"], missionAreaClause] : ["==", ["geometry-type"], "Polygon"]);
  };

  const _applyInputAreaFilters = () => {
    const packageClause = _packageClause();
    const missionClause = missionFocus?.inputMissionId == null
      ? null
      : ["==", ["to-string", ["get", "inputMissionID"]], String(missionFocus.inputMissionId)];
    const clauses = [packageClause, missionClause].filter(Boolean);
    const combined = clauses.length > 1 ? ["all", ...clauses] : clauses[0] || null;
    if (map.getLayer(AREA_FILL)) {
      map.setFilter(AREA_FILL, combined
        ? ["all", ["==", ["geometry-type"], "Polygon"], combined]
        : ["==", ["geometry-type"], "Polygon"]);
    }
    if (map.getLayer(AREA_LINE)) {
      map.setFilter(AREA_LINE, combined
        ? ["all", ["any", ["==", ["geometry-type"], "Polygon"], ["==", ["geometry-type"], "LineString"]], combined]
        : ["any", ["==", ["geometry-type"], "Polygon"], ["==", ["geometry-type"], "LineString"]]);
    }
    if (map.getLayer(AREA_LABEL)) {
      const base = _areaRegionLabelFilter();
      map.setFilter(AREA_LABEL, combined ? ["all", base, combined] : base);
    }
    const battleBase = _battleAnchorFilter();
    for (const id of [BATTLE_ANCHOR_RING, BATTLE_ANCHOR_DOT, BATTLE_ANCHOR_LABEL]) {
      if (map.getLayer(id)) map.setFilter(id, combined ? ["all", battleBase, combined] : battleBase);
    }
  };

  // ---- Detected targets (적 표적) ----

  const setTargets = (targets) => {
    const features = (targets || []).map((t) => ({
      type: "Feature",
      geometry: { type: "Point", coordinates: [t.lon, t.lat] },
      properties: {
        targetID: t.targetID ?? "?",
        typeLabel: t.typeLabel || "표적",
        threat: t.threat ?? "",
        watcher: t.watcherLabel || "",
        destroyed: t.isDestroyed ? 1 : 0,
        detectedText: t.detectedText || "",
        lat: Number(t.lat).toFixed(6),
        lon: Number(t.lon).toFixed(6),
      },
    }));
    const data = { type: "FeatureCollection", features };
    const src = map.getSource(TARGET_SOURCE);
    if (src) {
      src.setData(data);
      return;
    }

    map.addSource(TARGET_SOURCE, { type: "geojson", data });
    map.addLayer({
      id: TARGET_CIRCLE, type: "circle", source: TARGET_SOURCE,
      paint: {
        "circle-radius": 7,
        "circle-color": ["case", ["==", ["get", "destroyed"], 1], "#64748b", "#ef4444"],
        "circle-opacity": ["case", ["==", ["get", "destroyed"], 1], 0.55, 0.9],
        "circle-stroke-width": 2,
        "circle-stroke-color": "#fff",
      },
    });
    map.addLayer({
      id: TARGET_LABEL, type: "symbol", source: TARGET_SOURCE,
      layout: {
        "text-field": ["concat", ["get", "typeLabel"], " ", ["to-string", ["get", "targetID"]]],
        "text-size": 11,
        "text-font": ["Noto Sans Regular"],
        "text-offset": [0, 1.1],
        "text-anchor": "top",
        "text-optional": true,
      },
      paint: {
        "text-color": ["case", ["==", ["get", "destroyed"], 1], "#94a3b8", "#f87171"],
        "text-halo-color": "rgba(10,13,16,0.9)",
        "text-halo-width": 1.6,
      },
    });

    map.on("click", TARGET_CIRCLE, (e) => {
      if (!e.features?.length) return;
      const f = e.features[0];
      if (popup) popup.remove();
      popup = new maplibregl.Popup({ offset: 12, closeButton: true, maxWidth: "260px" })
        .setLngLat(f.geometry.coordinates.slice())
        .setHTML(targetPopup(f.properties))
        .addTo(map);
    });
    map.on("mouseenter", TARGET_CIRCLE, () => { map.getCanvas().style.cursor = "pointer"; });
    map.on("mouseleave", TARGET_CIRCLE, () => { map.getCanvas().style.cursor = ""; });
    _applyGlobalLayerState();
  };

  const clearTargets = () => {
    const src = map.getSource(TARGET_SOURCE);
    if (src) src.setData({ type: "FeatureCollection", features: [] });
  };

  const _ensureCurrentWaypointLayers = () => {
    if (map.getSource(CURRENT_WP_SOURCE)) return;
    map.addSource(CURRENT_WP_SOURCE, {
      type: "geojson",
      data: { type: "FeatureCollection", features: [] },
    });
    activeSources.add(CURRENT_WP_SOURCE);
    const colorMatch = ["match", ["get", "aircraftId"]];
    for (const [aid, color] of Object.entries(AGENT_COLORS)) colorMatch.push(Number(aid), color);
    colorMatch.push("#f8fafc");
    map.addLayer({
      id: CURRENT_WP_RING, type: "circle", source: CURRENT_WP_SOURCE,
      paint: {
        "circle-radius": 10,
        "circle-color": "rgba(0,0,0,0)",
        "circle-stroke-width": 3,
        "circle-stroke-color": colorMatch,
        "circle-blur": 0.08,
      },
    });
    map.addLayer({
      id: CURRENT_WP_LABEL, type: "symbol", source: CURRENT_WP_SOURCE,
      layout: {
        "text-field": ["concat", "NOW WP ", ["to-string", ["get", "waypointID"]]],
        "text-size": 11,
        "text-offset": [0, 1.55],
        "text-anchor": "top",
        "text-font": ["Noto Sans Regular"],
        "text-allow-overlap": true,
      },
      paint: {
        "text-color": colorMatch,
        "text-halo-color": "rgba(10,13,16,0.92)",
        "text-halo-width": 2,
      },
    });
    activeLayers.add(CURRENT_WP_RING);
    activeLayers.add(CURRENT_WP_LABEL);
    _applyGlobalLayerState();
  };

  const _updateCurrentWaypointFeatures = (snapshot) => {
    const src = map.getSource(CURRENT_WP_SOURCE);
    if (!src) return;
    const features = [];
    for (const [aidText, state] of Object.entries(snapshot?.aircraft || {})) {
      const aid = Number(aidText);
      if (agentVisibility[aid] === false || (missionFocus && missionFocus.aircraftId !== aid)) continue;
      if (state.waypointID == null) continue;
      const lookup = waypointLookup[aid]?.[String(state.waypointID)];
      if (!lookup) continue;
      if (missionFocus && String(lookup.missionId) !== String(missionFocus.missionId)) continue;
      features.push({
        type: "Feature",
        geometry: { type: "Point", coordinates: lookup.coordinate },
        properties: { aircraftId: aid, waypointID: state.waypointID, missionId: lookup.missionId ?? "" },
      });
    }
    src.setData({ type: "FeatureCollection", features });
  };

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
    const sourceTimestamps = [...tsSet]
      .map(Number)
      .filter(Number.isFinite)
      .sort((a, b) => a - b);
    const configuredInterval = Number(
      playback?.ingestion?.visualizationIntervalMs || playback?.ingestion?.downsampleIntervalMs || 200,
    );
    const requestedIntervalMs = Number.isFinite(configuredInterval) && configuredInterval > 0
      ? configuredInterval
      : 200;
    const playbackClock = _buildPlaybackClock(sourceTimestamps, requestedIntervalMs);
    const playbackIntervalMs = playbackClock.intervalMs;
    trackTimestamps = playbackClock.timestamps;
    if (trackTimestamps.length === 0) return null;

    const playbackAircraftIds = new Set([
      ...Object.keys(tracks),
      ...Object.keys(footprints),
    ]);
    for (const aidStr of playbackAircraftIds) {
      const t = tracks[aidStr];
      const aid = Number(aidStr);
      knownAgentIds.add(aid);
      const color = AGENT_COLORS[aid] || "#888";
      if (t) {
        t.hasWaypointTelemetry = (t.waypointIDs || []).some((value) => value != null);
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
      }

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
    const info = {
      totalFrames: trackTimestamps.length,
      sourceFrameCount: sourceTimestamps.length,
      playbackIntervalMs,
      minTs: trackTimestamps[0],
      maxTs: trackTimestamps[trackTimestamps.length - 1],
      timestamps: trackTimestamps.slice(),
    };
    _applyAllLayerStates();
    setTrackFrame(0);
    return info;
  };

  const setTrackFrame = (frameIndex) => {
    if (!trackTimestamps) return null;
    currentTrackFrame = Math.max(0, Math.min(Number(frameIndex) || 0, trackTimestamps.length - 1));
    const targetTs = trackTimestamps[currentTrackFrame];
    const snapshot = { frameIndex: currentTrackFrame, timestamp: targetTs, aircraft: {} };
    if (missionFocus && missionFocusTimestampRange === undefined) {
      const focusedTrack = trackData?.[String(missionFocus.aircraftId)];
      missionFocusTimestampRange = focusedTrack?.hasWaypointTelemetry && missionFocus.waypointIds.length
        ? _focusedTimestampRange(focusedTrack, missionFocus.waypointIds)
        : null;
    }
    // A mission focus is more specific than a Plan interval. Both the actual
    // trail and sensor footprints must use these exact same time bounds.
    const focusedRange = missionFocus ? missionFocusTimestampRange : trackTimeFocus;
    const hasTimeFocus = !!focusedRange;

    for (const [aidStr, t] of Object.entries(trackData || {})) {
      const aid = Number(aidStr);
      const color = AGENT_COLORS[aid] || "#888";
      const label = AGENT_LABELS[aid] || `AC${aid}`;

      const idx = trackAllMode ? (t.coordinates || []).length - 1 : _lastIndexAtOrBefore(t.timestamps || [], targetTs);
      const rawCoord = idx >= 0 ? (t.coordinates || [])[idx] : null;
      const waypointID = idx >= 0 ? (t.waypointIDs || [])[idx] ?? null : null;
      snapshot.aircraft[aidStr] = {
        coordinate: rawCoord || null,
        waypointID,
        flightMode: idx >= 0 ? (t.flightModes || [])[idx] ?? null : null,
        flying: idx >= 0 ? (t.flyingStates || [])[idx] ?? null : null,
        hasWaypointTelemetry: !!t.hasWaypointTelemetry,
      };

      const isolateByMissionTime = missionFocus
        ? missionFocus.aircraftId === aid
        : hasTimeFocus;
      let displayedCoords = trackAllMode
        ? (t.coordinates || []).slice()
        : (idx >= 0 ? (t.coordinates || []).slice(0, idx + 1) : []);
      if (isolateByMissionTime) {
        displayedCoords = focusedRange
          ? _focusedTrackCoordinatesByTime(t, (t.coordinates || []).length - 1, focusedRange)
          : [];
      }
      const coord = displayedCoords[displayedCoords.length - 1] || (isolateByMissionTime ? null : rawCoord);

      // Update partial trail line
      const srcId = TRACK_SRC_PREFIX + aid;
      const src = map.getSource(srcId);
      if (src) {
        const lineCoords = displayedCoords.length === 1 ? [displayedCoords[0], displayedCoords[0]] : displayedCoords;
        src.setData(lineCoords.length >= 2
          ? { type: "Feature", geometry: { type: "LineString", coordinates: lineCoords } }
          : { type: "FeatureCollection", features: [] });
      }

      // Update/create marker for current position
      if (coord && !trackMarkers[aid]) {
        const el = document.createElement("div");
        el.className = "track-marker";
        el.style.cssText = `width:14px;height:14px;border-radius:50%;background:${color};border:2px solid #fff;box-shadow:0 0 6px ${color};`;
        el.title = label;
        trackMarkers[aid] = new maplibregl.Marker({ element: el }).setLngLat(coord).addTo(map);
      } else if (coord && trackMarkers[aid]) {
        trackMarkers[aid].setLngLat(coord);
      } else if (!coord && trackMarkers[aid]) {
        trackMarkers[aid].remove();
        delete trackMarkers[aid];
      }
      _applyAgentLayerState(aid);
    }

    for (const [aidStr, data] of Object.entries(footprintData || {})) {
      const aid = Number(aidStr);
      const src = map.getSource(FOOTPRINT_SRC_PREFIX + aid);
      if (!src) continue;
      const shouldRender = (
        layerVisibility.footprints
        && agentVisibility[aid] !== false
        && (!missionFocus || missionFocus.aircraftId === aid)
      );
      const isolateFootprints = missionFocus
        ? missionFocus.aircraftId === aid
        : hasTimeFocus;
      src.setData({
        type: "FeatureCollection",
        features: shouldRender && (!isolateFootprints || focusedRange)
          ? _footprintFeatures(aidStr, data, targetTs, trackAllMode || isolateFootprints, focusedRange)
          : [],
      });
    }
    lastTrackSnapshot = snapshot;
    _updateCurrentWaypointFeatures(snapshot);
    return snapshot;
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
    lastTrackSnapshot = null;
    missionFocusTimestampRange = undefined;
    _updateCurrentWaypointFeatures(null);
  };

  const setTrackAll = (enabled) => {
    trackAllMode = !!enabled;
    setTrackFrame(currentTrackFrame);
  };

  function _lastIndexAtOrBefore(timestamps, targetTs) {
    let lo = 0, hi = timestamps.length - 1, answer = -1;
    while (lo <= hi) {
      const mid = (lo + hi) >> 1;
      if (timestamps[mid] <= targetTs) { answer = mid; lo = mid + 1; }
      else hi = mid - 1;
    }
    return answer;
  }

  function _buildPlaybackClock(sourceTimestamps, intervalMs) {
    if (!sourceTimestamps.length) return { timestamps: [], intervalMs };
    const first = sourceTimestamps[0];
    const last = sourceTimestamps[sourceTimestamps.length - 1];
    if (first === last) return { timestamps: [first], intervalMs };
    const span = Math.max(0, last - first);
    const safeIntervalMs = Math.max(intervalMs, Math.ceil(span / (MAX_PLAYBACK_FRAMES - 1)));
    const frames = [];
    for (
      let timestamp = first;
      timestamp < last && frames.length < MAX_PLAYBACK_FRAMES - 1;
      timestamp += safeIntervalMs
    ) {
      frames.push(timestamp);
    }
    if (frames[frames.length - 1] !== last) frames.push(last);
    return { timestamps: frames, intervalMs: safeIntervalMs };
  }

  function _focusedTrackCoordinatesByTime(track, endIndex, focusedRange) {
    if (!focusedRange || endIndex < 0) return [];
    const timestamps = track.timestamps || [];
    const coordinates = track.coordinates || [];
    const end = Math.min(endIndex, timestamps.length - 1, coordinates.length - 1);
    const focused = [];
    for (let i = 0; i <= end; i++) {
      const timestamp = Number(timestamps[i]);
      if (!Number.isFinite(timestamp) || timestamp < focusedRange[0]) continue;
      if (timestamp > focusedRange[1]) break;
      if (coordinates[i]) focused.push(coordinates[i]);
    }
    return focused;
  }

  function _focusedTimestampRange(track, waypointIds) {
    const wanted = new Set(waypointIds.map(Number));
    const states = track.waypointIDs || [];
    const timestamps = track.timestamps || [];
    let first = -1, last = -1;
    for (let i = 0; i < states.length; i++) {
      if (!wanted.has(Number(states[i]))) continue;
      if (first < 0) first = i;
      last = i;
    }
    if (first < 0 || last < 0) return null;
    const start = Number(timestamps[first]);
    const end = Number(timestamps[last]);
    return Number.isFinite(start) && Number.isFinite(end) ? [start, end] : null;
  }

  function _footprintFeatures(aidStr, data, targetTs, showAll, focusedRange = null) {
    const timestamps = data.timestamps || [];
    const polygons = data.polygons || [];
    const features = [];
    for (let i = 0; i < polygons.length; i++) {
      if (!showAll && timestamps[i] > targetTs) break;
      if (focusedRange && timestamps[i] < focusedRange[0]) continue;
      if (focusedRange && timestamps[i] > focusedRange[1]) break;
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

  return {
    loadReferenceGeoJSON,
    loadAreaGeoJSON,
    showPlanResolved,
    clearPaths,
    setAgentVisible,
    setLayerVisibility,
    setMissionFocus,
    setTrackTimeFocus,
    setInputMissionPackage,
    flyToPath,
    loadTracks,
    setTrackFrame,
    setTrackAll,
    clearTracks,
    setTargets,
    clearTargets,
  };
};

function _areaRegionLabelFilter() {
  return [
    "all",
    ["has", "regionLabel"],
    ["match", ["to-number", ["get", "regionType"]], 4, true, 5, true, 6, true, false],
    ["any", ["==", ["geometry-type"], "Polygon"], ["==", ["geometry-type"], "LineString"]],
  ];
}

function _battleAnchorFilter() {
  return ["==", ["get", "landmarkType"], "battleAttackCoordinate"];
}

function _regionColorExpression() {
  return [
    "match",
    ["to-number", ["get", "regionType"]],
    4, "#fde047",
    5, "#fbbf24",
    6, "#fb7185",
    10, "#c084fc",
    "#7dd3fc",
  ];
}

function _isMeaningfulAttackWaypoint(waypoint) {
  const attack = waypoint?.attack;
  if (waypoint?.lahType === "attack") return true;
  if (!attack) return false;
  const targetId = Number(attack.targetID);
  const weaponType = Number(attack.weaponType);
  return (Number.isFinite(targetId) && targetId !== 0)
    || (Number.isFinite(weaponType) && weaponType !== 0);
}

function _weaponLabel(weaponType) {
  const value = Number(weaponType);
  if (value === 1) return "기관포";
  if (value === 2) return "유도탄";
  if (value === 3) return "로켓";
  if (weaponType == null || weaponType === "" || value === 0) return "무장 미기록";
  return `무장 Type ${weaponType}`;
}

function _etaLabel(value) {
  const seconds = Number(value);
  if (!Number.isFinite(seconds)) return "";
  return `${seconds.toFixed(1)}s`;
}

function _attackPointFeature({
  aid,
  missionId,
  inputMissionId,
  pathId,
  waypoint,
  coordinate,
  source,
  anchorContext,
}) {
  const aircraftLabel = AGENT_LABELS[aid] || `AC${aid}`;
  const targetId = waypoint?.attack?.targetID ?? "";
  const weaponType = waypoint?.attack?.weaponType ?? "";
  const weaponLabel = _weaponLabel(weaponType);
  const altitude = waypoint?.altitude;
  const eta = waypoint?.eta;
  const wpText = waypoint?.waypointID == null || waypoint.waypointID === "-"
    ? "계획 좌표"
    : `WP ${waypoint.waypointID}`;
  const targetText = targetId === "" || Number(targetId) === 0 ? "" : ` · 표적 ${targetId}`;
  const altitudeText = altitude == null || altitude === "" ? "" : ` · ${altitude}m`;
  return {
    type: "Feature",
    geometry: { type: "Point", coordinates: coordinate.slice(0, 2).map(Number) },
    properties: {
      kind: "lah-attack-point",
      aircraftId: aid,
      aircraftLabel,
      missionId: missionId ?? "",
      inputMissionId: inputMissionId ?? "",
      pathId: pathId ?? "",
      waypointID: waypoint?.waypointID ?? "",
      targetID: targetId,
      weaponType,
      weaponLabel,
      altitude: altitude ?? "",
      eta: eta ?? "",
      attackSource: source,
      battleAnchorInputMissionID: anchorContext?.inputMissionID ?? "",
      battleAnchorSource: anchorContext?.anchorSource ?? "",
      battleAnchorSourceLabel: anchorContext?.anchorSourceLabel ?? "",
      labelText: `${aircraftLabel} · 공격 Point ${wpText} · IM ${missionId ?? "-"}${targetText} · ${weaponLabel}${altitudeText}`,
      lat: Number(coordinate[1]).toFixed(6),
      lon: Number(coordinate[0]).toFixed(6),
    },
  };
}

function battleAnchorPopup(properties) {
  const fallback = properties.anchorSource === "area-centroid-fallback";
  const explanation = fallback
    ? "명시 battleAttackCoordinate가 없어 플래너와 동일하게 전투진지 AREA 꼭짓점 평균을 사용하고 고도 1,500m를 강제한 fallback 기준점입니다."
    : "InputMissionPlan의 명시 battleAttackCoordinate입니다. LAH 전투진지 대기·공격 전개 기준점이며 실제 표적 좌표와는 구분됩니다.";
  return `<div style="font-weight:700;margin-bottom:5px;color:#fbbf24">${_html(properties.landmarkLabel || "전투진지 공격기준점")}</div>
    <div style="font-size:11px">${_html(properties.regionLabel || "전투진지")} · Input ${_html(properties.inputMissionID ?? "-")}</div>
    <div style="font-size:11px;margin-top:3px;color:#fcd34d">${_html(properties.anchorSourceLabel || "명시 기준점")}</div>
    <div style="font-size:11px;margin-top:4px;color:rgba(224,232,240,0.72)">${_html(explanation)}</div>
    <div style="font-size:11px;margin-top:4px;color:rgba(224,232,240,0.58)">${_html(properties.lat ?? "-")}, ${_html(properties.lon ?? "-")}</div>`;
}

function attackPointPopup(properties) {
  const sourceText = properties.attackSource === "waypoint-attack"
    ? `FlightPath 공격 명령 WP ${properties.waypointID}`
    : "IndividualMissionInfo 계획 좌표 (공격 WP 미기록)";
  const targetText = properties.targetID === "" || Number(properties.targetID) === 0
    ? "표적 ID 미기록"
    : `표적 #${properties.targetID}`;
  const flightText = [
    properties.altitude === "" ? "" : `${properties.altitude}m`,
    properties.eta === "" ? "" : `ETA ${_etaLabel(properties.eta)}`,
  ].filter(Boolean).join(" · ");
  const anchorText = properties.battleAnchorSourceLabel
    ? `선정 기준: 전투진지 Input ${properties.battleAnchorInputMissionID} · ${properties.battleAnchorSourceLabel}`
    : "";
  return `<div style="font-weight:700;margin-bottom:5px;color:#fb7185">${_html(properties.aircraftLabel)} 공격 Point</div>
    <div style="font-size:11px">IM ${_html(properties.missionId ?? "-")} · Input ${_html(properties.inputMissionId ?? "-")}</div>
    <div style="font-size:11px;margin-top:3px">${_html(targetText)} · ${_html(properties.weaponLabel || "무장 미기록")}</div>
    ${flightText ? `<div style="font-size:11px;margin-top:2px">${_html(flightText)}</div>` : ""}
    ${anchorText ? `<div style="font-size:11px;margin-top:3px;color:#fcd34d">${_html(anchorText)}</div>` : ""}
    <div style="font-size:11px;margin-top:4px;color:rgba(224,232,240,0.72)">${_html(sourceText)}</div>
    <div style="font-size:11px;color:rgba(224,232,240,0.72)">이 점은 LAH의 공격/발사 waypoint이며, 실제 표적 위치는 표적 ID 레이어에서 확인합니다.</div>
    <div style="font-size:11px;margin-top:4px;color:rgba(224,232,240,0.58)">${_html(properties.lat ?? "-")}, ${_html(properties.lon ?? "-")}</div>`;
}

function attackPointsPopup(rows) {
  const unique = [];
  const seen = new Set();
  for (const row of rows || []) {
    const key = `${row.aircraftId}:${row.missionId}:${row.waypointID}`;
    if (seen.has(key)) continue;
    seen.add(key);
    unique.push(row);
  }
  if (unique.length <= 1) return attackPointPopup(unique[0] || {});
  const anchor = unique.find((row) => row.battleAnchorSourceLabel);
  return `<div style="font-weight:700;margin-bottom:5px;color:#fb7185">동일 좌표 공격 Point ${unique.length}건</div>
    ${unique.map((row) => `<div style="padding:5px 0;border-top:1px solid rgba(251,113,133,0.22)">
      <b style="font-size:11px;color:#fda4af">${_html(row.aircraftLabel)} · IM ${_html(row.missionId ?? "-")}</b>
      <div style="font-size:11px">WP ${_html(row.waypointID ?? "-")} · 표적 #${_html(row.targetID ?? "-")} · ${_html(row.weaponLabel || "무장 미기록")}${row.altitude === "" ? "" : ` · ${_html(row.altitude)}m`}</div>
    </div>`).join("")}
    ${anchor ? `<div style="font-size:11px;margin-top:4px;color:#fcd34d">선정 기준: 전투진지 Input ${_html(anchor.battleAnchorInputMissionID)} · ${_html(anchor.battleAnchorSourceLabel)}</div>` : ""}
    <div style="font-size:11px;margin-top:4px;color:rgba(224,232,240,0.68)">각 행은 서로 다른 유인기 임무이며 같은 공격/발사 좌표를 공유합니다.</div>`;
}

function _matchingBattleAnchor(landmarks, coordinate) {
  const lon = Number(coordinate?.[0]);
  const lat = Number(coordinate?.[1]);
  if (!Number.isFinite(lon) || !Number.isFinite(lat)) return null;
  const feature = (landmarks || []).find((item) => {
    if (item?.properties?.landmarkType !== "battleAttackCoordinate") return false;
    const [anchorLon, anchorLat] = item?.geometry?.coordinates || [];
    return Math.abs(Number(anchorLon) - lon) <= 1e-7
      && Math.abs(Number(anchorLat) - lat) <= 1e-7;
  });
  return feature?.properties || null;
}

function _html(value) {
  return String(value ?? "")
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;");
}

/**
 * Build allocation features (assigned area polygons, corridor segments, and
 * label anchors) for one aircraft from its resolved missions.
 */
function _allocationFeatures(aid, info) {
  const label = AGENT_LABELS[aid] || `AC${aid}`;
  const features = [];

  for (const m of info.missions || []) {
    const props = {
      aircraftId: aid,
      missionId: m.id ?? "",
      missionType: m.type ?? "",
      isDone: m.isDone ? 1 : 0,
    };
    const missionLabel = `${label} · IM ${m.id ?? "-"}`;
    let hasGeometry = false;

    let current = null;
    for (const ringDef of m.areaList || []) {
      const ring = (ringDef.coordinates || []).map((c) => c.slice(0, 2));
      if (ring.length < 3) continue;
      const first = ring[0], last = ring[ring.length - 1];
      if (first[0] !== last[0] || first[1] !== last[1]) ring.push([...first]);
      if (ringDef.isHole && current) {
        current.geometry.coordinates.push(ring);
      } else {
        current = {
          type: "Feature",
          geometry: { type: "Polygon", coordinates: [ring] },
          properties: { ...props, kind: "area" },
        };
        features.push(current);
        features.push(_labelFeature(_ringCentroid(ring), missionLabel, props));
        hasGeometry = true;
      }
    }

    for (const lineDef of m.lineList || []) {
      const coords = (lineDef.coordinates || []).map((c) => c.slice(0, 2));
      if (coords.length < 2) continue;
      features.push({
        type: "Feature",
        geometry: { type: "LineString", coordinates: coords },
        properties: { ...props, kind: "corridor", widthM: lineDef.width || 0 },
      });
      features.push(_labelFeature(coords[Math.floor(coords.length / 2)], missionLabel, props));
      hasGeometry = true;
    }

    if (!hasGeometry) {
      const coords = (m.coordinateList || []).map((c) => c.slice(0, 2));
      if (coords.length === 1) {
        features.push({
          type: "Feature",
          geometry: { type: "Point", coordinates: coords[0] },
          properties: { ...props, kind: "assignment-point", labelText: missionLabel },
        });
        features.push(_labelFeature(coords[0], missionLabel, props));
      } else if (coords.length >= 2) {
        features.push({
          type: "Feature",
          geometry: { type: "LineString", coordinates: coords },
          properties: { ...props, kind: "corridor", widthM: 0 },
        });
        features.push(_labelFeature(coords[Math.floor(coords.length / 2)], missionLabel, props));
      }
    }
  }

  return features;
}

function _labelFeature(coord, text, props) {
  return {
    type: "Feature",
    geometry: { type: "Point", coordinates: coord },
    properties: { ...props, kind: "label", labelText: text },
  };
}

function _ringCentroid(ring) {
  // Average of vertices (excluding the closing point) — good enough for labels.
  const n = ring.length - 1;
  let x = 0, y = 0;
  for (let i = 0; i < n; i++) { x += ring[i][0]; y += ring[i][1]; }
  return [x / Math.max(1, n), y / Math.max(1, n)];
}

function targetPopup(p) {
  const destroyed = Number(p.destroyed) === 1;
  const titleColor = destroyed ? "#94a3b8" : "#f87171";
  let html = `<div style="font-weight:700;margin-bottom:4px;color:${titleColor}">${p.typeLabel} ${p.targetID}${destroyed ? " (파괴)" : ""}</div>`;
  if (p.detectedText) html += `<div style="font-size:11px">발견: ${p.detectedText}</div>`;
  if (p.watcher) html += `<div style="font-size:11px">발견기체: ${p.watcher}</div>`;
  if (p.threat !== "" && p.threat != null) html += `<div style="font-size:11px">위협도: ${p.threat}</div>`;
  html += `<div style="font-size:11px;margin-top:2px;color:rgba(224,232,240,0.6)">${p.lat}, ${p.lon}</div>`;
  return html;
}

function wpPopup(p) {
  const s = (v) => `<span style="color:rgba(224,232,240,0.55)">`;
  let html = `<div style="font-weight:700;margin-bottom:4px;color:#5b9cf6">${p.agent} — WP ${p.waypointID}</div>`;
  html += `<div style="font-size:11px;color:rgba(224,232,240,0.7)">Path: ${p.pathID}</div>`;
  html += `<div style="font-size:11px;margin-top:4px">Lat: ${p.lat} Lon: ${p.lon}</div>`;
  if (p.altitude) html += `<div style="font-size:11px">Alt: ${p.altitude}m</div>`;
  if (p.speed) html += `<div style="font-size:11px">Speed: ${p.speed}</div>`;
  if (p.eta) html += `<div style="font-size:11px">ETA: ${_etaLabel(p.eta)}</div>`;

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
