const TARGET_SOURCE_ID = "enemy-targets";
const TARGET_LAYER_ID = "enemy-targets-circle";
const TARGET_LABEL_LAYER_ID = "enemy-targets-label";

const TYPE_LABELS = {
  1: "\uC804\uCC28",
  2: "\uC7A5\uAC11\uCC28",
  3: "\uBC29\uC0AC\uD3EC",
  4: "\uACE1\uC0AC\uD3EC",
  5: "\uACE0\uC815\uACE0\uC0AC\uD3EC",
  6: "\uAD70\uC778",
};

const TYPE_COLORS = {
  1: "#c85d5d",
  2: "#d08a4b",
  3: "#b56dc9",
  4: "#8c6a5c",
  5: "#b23b3b",
  6: "#d2c27d",
};

const toColor = (typeId, alive) => {
  if (!alive) {
    return "#4a4a4a";
  }
  return TYPE_COLORS[typeId] || "#b56d6d";
};

export const initTargetMarkers = (map) => {
  let pending = null;
  let mapReady = typeof map.isStyleLoaded === "function" ? map.isStyleLoaded() : map.loaded();

  const ensureLayer = () => {
    if (!map.getSource(TARGET_SOURCE_ID)) {
      map.addSource(TARGET_SOURCE_ID, {
        type: "geojson",
        data: { type: "FeatureCollection", features: [] },
      });
    }
    if (!map.getLayer(TARGET_LAYER_ID)) {
      map.addLayer({
        id: TARGET_LAYER_ID,
        type: "circle",
        source: TARGET_SOURCE_ID,
        paint: {
          "circle-color": ["get", "color"],
          "circle-radius": [
            "interpolate",
            ["linear"],
            ["zoom"],
            9,
            3.5,
            12,
            5,
            15,
            7.5,
          ],
          "circle-opacity": 0.9,
          "circle-stroke-color": "rgba(30,30,30,0.6)",
          "circle-stroke-width": 1,
        },
      });
    }
    if (!map.getLayer(TARGET_LABEL_LAYER_ID)) {
      map.addLayer({
        id: TARGET_LABEL_LAYER_ID,
        type: "symbol",
        source: TARGET_SOURCE_ID,
        layout: {
          "text-field": ["get", "name"],
          "text-size": 11,
          "text-offset": [0, 1.1],
          "text-anchor": "top",
          "text-allow-overlap": true,
          "text-ignore-placement": true,
        },
        paint: {
          "text-color": ["get", "color"],
          "text-halo-color": "rgba(0,0,0,0.55)",
          "text-halo-width": 1.2,
        },
      });
    }
  };

  const buildFeatures = (targets) => {
    if (!Array.isArray(targets)) {
      return [];
    }
    return targets
      .map((t) => {
        const lat = Number(t?.lat);
        const lon = Number(t?.lon);
        if (!Number.isFinite(lat) || !Number.isFinite(lon)) {
          return null;
        }
        const typeId = Number(t?.type);
        const alive = t?.alive !== false;
        const fallbackName = TYPE_LABELS[typeId]
          ? `${TYPE_LABELS[typeId]}_${Number(t?.id || 0)}`
          : `T${Number(t?.id || 0)}`;
        const name = t?.name ? String(t.name) : fallbackName;
        return {
          type: "Feature",
          properties: {
            id: Number(t?.id || 0),
            type: typeId,
            name,
            alive,
            moving: Boolean(t?.moving),
            color: toColor(typeId, alive),
          },
          geometry: {
            type: "Point",
            coordinates: [lon, lat],
          },
        };
      })
      .filter(Boolean);
  };

  const apply = (payload) => {
    const features = buildFeatures(payload?.targets || []);
    ensureLayer();
    const source = map.getSource(TARGET_SOURCE_ID);
    if (source) {
      source.setData({ type: "FeatureCollection", features });
    }
  };

  const loadFromReference = (payload) => {
    pending = payload;
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

  return { loadFromReference };
};
