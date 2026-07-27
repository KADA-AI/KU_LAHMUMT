const EFFECT_SOURCE_ID = "impact-effects";
const FLASH_LAYER_ID = "impact-flash";
const RING_LAYER_ID = "impact-ring";

const FRIENDLY_COLOR = "#6dd8ff";
const ENEMY_COLOR = "#ff7a6b";
const NEUTRAL_COLOR = "#ffd58a";

const clamp = (value, min, max) => Math.min(max, Math.max(min, value));

const metersPerPixel = (lat, zoom) => {
  const cosLat = Math.cos((lat * Math.PI) / 180);
  return (156543.03392 * Math.max(0.2, cosLat)) / Math.pow(2, zoom);
};

const toPixels = (meters, lat, zoom) => {
  const mpp = metersPerPixel(lat, zoom);
  if (!Number.isFinite(mpp) || mpp <= 0) {
    return meters;
  }
  return meters / mpp;
};

const resolveColor = (side) => {
  const key = String(side || "").toLowerCase();
  if (key === "enemy") {
    return ENEMY_COLOR;
  }
  if (key === "friendly") {
    return FRIENDLY_COLOR;
  }
  return NEUTRAL_COLOR;
};

const buildFeatures = (effects, zoom) => {
  const features = [];
  if (!Array.isArray(effects)) {
    return features;
  }
  effects.forEach((eff) => {
    const lat = Number(eff?.lat);
    const lon = Number(eff?.lon);
    if (!Number.isFinite(lat) || !Number.isFinite(lon)) {
      return;
    }
    const ttl = Math.max(0.05, Number(eff?.ttl) || 0.6);
    const age = Math.max(0, Number(eff?.age) || 0);
    const progress = clamp(age / ttl, 0, 1);
    const baseRadius = Math.max(5, Number(eff?.radius) || 50);
    const baseFlash = Math.max(4, Number(eff?.flash) || baseRadius * 0.45);
    const color = resolveColor(eff?.side);

    const ringRadiusM = baseRadius * (0.3 + 0.7 * progress);
    const flashRadiusM = baseFlash * (1 - progress * 0.6);
    const ringPx = clamp(toPixels(ringRadiusM, lat, zoom), 3, 180);
    const flashPx = clamp(toPixels(flashRadiusM, lat, zoom), 3, 90);
    const ringAlpha = clamp(0.9 * (1 - progress), 0, 0.9);
    const flashAlpha = clamp(0.95 * (1 - progress * 1.1), 0, 0.95);
    const ringStroke = clamp(1.2 + (1 - progress) * 2.4, 1.2, 4.0);

    features.push({
      type: "Feature",
      properties: {
        kind: "ring",
        color,
        opacity: ringAlpha,
        radius: ringPx,
        stroke: ringStroke,
      },
      geometry: {
        type: "Point",
        coordinates: [lon, lat],
      },
    });
    features.push({
      type: "Feature",
      properties: {
        kind: "flash",
        color,
        opacity: flashAlpha,
        radius: flashPx,
      },
      geometry: {
        type: "Point",
        coordinates: [lon, lat],
      },
    });
  });
  return features;
};

export const initImpactEffects = (map) => {
  let pending = null;
  let mapReady = typeof map.isStyleLoaded === "function" ? map.isStyleLoaded() : map.loaded();
  let lastEffectPayloadSignature = null;

  const ensureLayer = () => {
    if (!map.getSource(EFFECT_SOURCE_ID)) {
      map.addSource(EFFECT_SOURCE_ID, {
        type: "geojson",
        data: { type: "FeatureCollection", features: [] },
      });
    }
    if (!map.getLayer(FLASH_LAYER_ID)) {
      map.addLayer({
        id: FLASH_LAYER_ID,
        type: "circle",
        source: EFFECT_SOURCE_ID,
        filter: ["==", ["get", "kind"], "flash"],
        paint: {
          "circle-color": ["get", "color"],
          "circle-opacity": ["get", "opacity"],
          "circle-radius": ["get", "radius"],
          "circle-blur": 0.35,
        },
      });
    }
    if (!map.getLayer(RING_LAYER_ID)) {
      map.addLayer({
        id: RING_LAYER_ID,
        type: "circle",
        source: EFFECT_SOURCE_ID,
        filter: ["==", ["get", "kind"], "ring"],
        paint: {
          "circle-color": "rgba(0,0,0,0)",
          "circle-opacity": 0,
          "circle-stroke-color": ["get", "color"],
          "circle-stroke-opacity": ["get", "opacity"],
          "circle-stroke-width": ["get", "stroke"],
          "circle-radius": ["get", "radius"],
        },
      });
    }
  };

  const apply = (payload) => {
    ensureLayer();
    const source = map.getSource(EFFECT_SOURCE_ID);
    if (!source) {
      return;
    }
    const zoom = typeof map.getZoom === "function" ? map.getZoom() : 12;
    const features = buildFeatures(payload?.effects || [], zoom);
    source.setData({ type: "FeatureCollection", features });
  };

  const loadFromReference = (payload) => {
    pending = payload;
    const effects = Array.isArray(payload?.effects) ? payload.effects : [];
    let payloadSignature = null;
    try {
      payloadSignature = JSON.stringify(effects);
    } catch (_err) {
      payloadSignature = null;
    }
    if (payloadSignature !== null && payloadSignature === lastEffectPayloadSignature) {
      return;
    }
    lastEffectPayloadSignature = payloadSignature;
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

  map.on("zoomend", () => {
    if (pending) {
      apply(pending);
    }
  });

  return { loadFromReference };
};
