const SOURCE_ID = "mission-reference-points";
const HALO_LAYER_ID = "mission-reference-points-halo";
const MARKER_LAYER_ID = "mission-reference-points-marker";
const LABEL_LAYER_ID = "mission-reference-points-label";

const REFERENCE_SPECS = [
  {
    type: "takeover",
    aliases: ["takeOverInfoList", "TakeOverInfoList"],
    shortLabel: "TO",
    title: "Take Over",
    color: "#35e0a1",
  },
  {
    type: "handover",
    aliases: ["handOverInfoList", "HandOverInfoList"],
    shortLabel: "HO",
    title: "Hand Over",
    color: "#ffb454",
  },
  {
    type: "rtb",
    aliases: ["rtbCoordinateList", "RTBCoordinateList"],
    shortLabel: "RTB",
    title: "Return To Base",
    color: "#cf82ff",
  },
];

const emptyCollection = () => ({ type: "FeatureCollection", features: [] });

const finiteNumber = (value) => {
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
};

const payloadCandidates = (payload) => {
  if (!payload || typeof payload !== "object") {
    return [];
  }
  return [
    payload,
    payload.payload,
    payload.missionReference,
    payload.missionReferenceInfo,
  ].filter((candidate) => candidate && typeof candidate === "object");
};

const listFromPayload = (payload, aliases) => {
  let emptyList = null;
  for (const candidate of payloadCandidates(payload)) {
    for (const key of aliases) {
      const rows = candidate[key];
      if (!Array.isArray(rows)) {
        continue;
      }
      if (rows.length) {
        return rows;
      }
      emptyList = rows;
    }
  }
  return emptyList || [];
};

const coordinateFromRow = (row) => {
  if (!row || typeof row !== "object") {
    return null;
  }
  const coordinate = row.coordinate || row.Coordinate || row;
  if (!coordinate || typeof coordinate !== "object") {
    return null;
  }
  const latitude = finiteNumber(coordinate.latitude ?? coordinate.Latitude);
  const longitude = finiteNumber(coordinate.longitude ?? coordinate.Longitude);
  const altitude = finiteNumber(coordinate.altitude ?? coordinate.Altitude);
  if (
    latitude === null ||
    longitude === null ||
    latitude < -90 ||
    latitude > 90 ||
    longitude < -180 ||
    longitude > 180
  ) {
    return null;
  }
  return { latitude, longitude, altitude };
};

const aircraftLabel = (aircraftId) => {
  if (!Number.isInteger(aircraftId) || aircraftId <= 0) {
    return "";
  }
  if (aircraftId <= 3) {
    return `LAH${aircraftId}`;
  }
  if (aircraftId <= 6) {
    return `UAV${aircraftId - 3}`;
  }
  return `AC${aircraftId}`;
};

export const buildMissionReferenceFeatureCollection = (payload) => {
  const features = [];
  REFERENCE_SPECS.forEach((spec) => {
    const rows = listFromPayload(payload, spec.aliases);
    rows.forEach((row, index) => {
      const coordinate = coordinateFromRow(row);
      if (!coordinate) {
        return;
      }
      const rawAircraftId = finiteNumber(row?.aircraftID ?? row?.AircraftID);
      const aircraftId = rawAircraftId === null ? null : Math.trunc(rawAircraftId);
      const agent = aircraftLabel(aircraftId);
      const ordinal = index + 1;
      const suffix = agent || String(ordinal);
      features.push({
        type: "Feature",
        id: `${spec.type}-${aircraftId ?? ordinal}-${ordinal}`,
        properties: {
          referenceType: spec.type,
          referenceTitle: spec.title,
          label: `${spec.shortLabel} ${suffix}`,
          color: spec.color,
          aircraftID: aircraftId,
          agent,
          altitude: coordinate.altitude,
          latitude: coordinate.latitude,
          longitude: coordinate.longitude,
        },
        geometry: {
          type: "Point",
          coordinates: [coordinate.longitude, coordinate.latitude],
        },
      });
    });
  });
  return { type: "FeatureCollection", features };
};

const popupContent = (properties) => {
  const root = document.createElement("div");
  root.className = "mission-reference-popup";
  const title = document.createElement("strong");
  title.textContent = `${properties.referenceTitle || "Mission Reference"} · ${properties.label || ""}`;
  root.appendChild(title);
  const details = [
    properties.aircraftID ? `Aircraft ID: ${properties.aircraftID}` : null,
    `Latitude: ${Number(properties.latitude).toFixed(6)}`,
    `Longitude: ${Number(properties.longitude).toFixed(6)}`,
    properties.altitude !== null && properties.altitude !== "null" && properties.altitude !== ""
      ? `Altitude: ${Number(properties.altitude).toFixed(0)} m`
      : null,
  ].filter(Boolean);
  details.forEach((text) => {
    const line = document.createElement("div");
    line.textContent = text;
    root.appendChild(line);
  });
  return root;
};

export const initMissionReferenceMarkers = (map) => {
  let mapReady = Boolean(map && typeof map.isStyleLoaded === "function" && map.isStyleLoaded());
  let pendingCollection = emptyCollection();
  let visible = true;
  let interactionsAttached = false;
  let popup = null;

  const applyVisibility = () => {
    if (!mapReady) {
      return;
    }
    const visibility = visible ? "visible" : "none";
    [HALO_LAYER_ID, MARKER_LAYER_ID, LABEL_LAYER_ID].forEach((layerId) => {
      if (map.getLayer(layerId)) {
        map.setLayoutProperty(layerId, "visibility", visibility);
      }
    });
  };

  const attachInteractions = () => {
    if (interactionsAttached || !map.getLayer(MARKER_LAYER_ID)) {
      return;
    }
    interactionsAttached = true;
    map.on("mouseenter", MARKER_LAYER_ID, () => {
      map.getCanvas().style.cursor = "pointer";
    });
    map.on("mouseleave", MARKER_LAYER_ID, () => {
      map.getCanvas().style.cursor = "";
    });
    map.on("click", MARKER_LAYER_ID, (event) => {
      const feature = event?.features?.[0];
      if (!feature || feature.geometry?.type !== "Point") {
        return;
      }
      if (popup) {
        popup.remove();
      }
      popup = new maplibregl.Popup({ closeButton: true, closeOnClick: true, offset: 12 })
        .setLngLat(feature.geometry.coordinates)
        .setDOMContent(popupContent(feature.properties || {}))
        .addTo(map);
    });
  };

  const ensureLayers = () => {
    if (!mapReady) {
      return;
    }
    if (!map.getSource(SOURCE_ID)) {
      map.addSource(SOURCE_ID, { type: "geojson", data: pendingCollection });
    }
    if (!map.getLayer(HALO_LAYER_ID)) {
      map.addLayer({
        id: HALO_LAYER_ID,
        type: "circle",
        source: SOURCE_ID,
        paint: {
          "circle-radius": ["interpolate", ["linear"], ["zoom"], 8, 8, 14, 12],
          "circle-color": ["get", "color"],
          "circle-opacity": 0.2,
        },
      });
    }
    if (!map.getLayer(MARKER_LAYER_ID)) {
      map.addLayer({
        id: MARKER_LAYER_ID,
        type: "circle",
        source: SOURCE_ID,
        paint: {
          "circle-radius": ["match", ["get", "referenceType"], "takeover", 6.5, "handover", 6, 5.5],
          "circle-color": ["get", "color"],
          "circle-stroke-color": "#162018",
          "circle-stroke-width": 2,
          "circle-opacity": 0.96,
        },
      });
    }
    if (!map.getLayer(LABEL_LAYER_ID)) {
      map.addLayer({
        id: LABEL_LAYER_ID,
        type: "symbol",
        source: SOURCE_ID,
        minzoom: 8,
        layout: {
          "text-field": ["get", "label"],
          "text-font": ["Noto Sans Regular"],
          "text-size": 11,
          "text-offset": [0, 1.25],
          "text-anchor": "top",
          "text-allow-overlap": true,
          "text-ignore-placement": true,
        },
        paint: {
          "text-color": ["get", "color"],
          "text-halo-color": "rgba(18, 25, 18, 0.96)",
          "text-halo-width": 1.5,
        },
      });
    }
    attachInteractions();
    applyVisibility();
  };

  const render = () => {
    if (!mapReady) {
      return;
    }
    ensureLayers();
    const source = map.getSource(SOURCE_ID);
    if (source && typeof source.setData === "function") {
      source.setData(pendingCollection);
    }
  };

  const loadFromReference = (payload) => {
    pendingCollection = buildMissionReferenceFeatureCollection(payload);
    render();
    return pendingCollection;
  };

  const setVisible = (nextVisible) => {
    visible = Boolean(nextVisible);
    applyVisibility();
  };

  if (mapReady) {
    ensureLayers();
  } else if (map && typeof map.once === "function") {
    map.once("load", () => {
      mapReady = true;
      render();
    });
  }

  return {
    loadFromReference,
    setVisible,
    getFeatureCollection: () => pendingCollection,
  };
};
