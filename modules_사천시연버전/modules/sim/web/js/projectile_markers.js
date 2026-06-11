const PROJECTILE_SOURCE_ID = "projectiles";
const PROJECTILE_LINE_LAYER_ID = "projectile-trails";
const PROJECTILE_HEAD_LAYER_ID = "projectile-heads";

const FRIENDLY_COLOR = "#40c8ff";
const ENEMY_COLOR = "#ff6464";

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

const buildFeatures = (projectiles) => {
  const features = [];
  if (!Array.isArray(projectiles)) {
    return features;
  }
  projectiles.forEach((proj) => {
    const lat = Number(proj?.lat);
    const lon = Number(proj?.lon);
    if (!Number.isFinite(lat) || !Number.isFinite(lon)) {
      return;
    }
    const vx = Number(proj?.vx);
    const vy = Number(proj?.vy);
    const vz = Number(proj?.vz);
    const speed = Number.isFinite(proj?.speed)
      ? Number(proj.speed)
      : Math.hypot(vx || 0, vy || 0, vz || 0);
    const side = String(proj?.side || "").toLowerCase();
    const color = side === "enemy" ? ENEMY_COLOR : FRIENDLY_COLOR;
    const headSize = clamp(2.4 + speed / 260, 2.4, 6.5);
    features.push({
      type: "Feature",
      properties: {
        color,
        size: headSize,
        kind: proj?.kind || "",
      },
      geometry: {
        type: "Point",
        coordinates: [lon, lat],
      },
    });

    const hspeed = Math.hypot(vx || 0, vy || 0);
    if (hspeed > 1e-3) {
      const tailLen = clamp(speed * 0.25, 25, 180);
      const dx = (-vx / hspeed) * tailLen;
      const dy = (-vy / hspeed) * tailLen;
      const tail = metersToLonLat(lon, lat, dx, dy);
      const width = clamp(1.2 + speed / 420, 1.2, 3.0);
      features.push({
        type: "Feature",
        properties: {
          color,
          width,
        },
        geometry: {
          type: "LineString",
          coordinates: [
            [lon, lat],
            [tail.lon, tail.lat],
          ],
        },
      });
    }
  });
  return features;
};

export const initProjectileMarkers = (map) => {
  let pending = null;
  let mapReady = typeof map.isStyleLoaded === "function" ? map.isStyleLoaded() : map.loaded();

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
          "line-opacity": 0.85,
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
          "circle-opacity": 0.95,
          "circle-stroke-color": "rgba(20,20,20,0.55)",
          "circle-stroke-width": 1,
        },
      });
    }
  };

  const apply = (payload) => {
    ensureLayer();
    const source = map.getSource(PROJECTILE_SOURCE_ID);
    if (!source) {
      return;
    }
    const features = buildFeatures(payload?.projectiles || []);
    source.setData({ type: "FeatureCollection", features });
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
