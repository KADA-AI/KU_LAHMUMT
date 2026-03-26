const parseNumber = (value, fallback) => {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
};

const parseJson = (value, fallback) => {
  if (!value || value === "null") {
    return fallback;
  }
  try {
    return JSON.parse(value);
  } catch (err) {
    return fallback;
  }
};

export const getConfig = (body = document.body) => ({
  tileUrl: body.dataset.tileUrl,
  minZoom: parseNumber(body.dataset.minZoom, 0),
  maxZoom: parseNumber(body.dataset.maxZoom, 14),
  center: [
    parseNumber(body.dataset.centerLon, 127.0),
    parseNumber(body.dataset.centerLat, 37.5),
  ],
  startZoom: parseNumber(body.dataset.startZoom, 10.5),
  bounds: parseJson(body.dataset.bounds, null),
  dem: {
    available: body.dataset.demAvailable === "1",
    tileUrl: body.dataset.demUrl,
    tileSize: parseNumber(body.dataset.demTileSize, 256),
    maxZoom: parseNumber(body.dataset.demMaxZoom, 12),
    bounds: parseJson(body.dataset.demBounds, null),
    encoding: body.dataset.demEncoding || "terrarium",
    exaggeration: parseNumber(body.dataset.demExaggeration, 1.0),
    pitchThreshold: parseNumber(body.dataset.demPitchThreshold, 2),
  },
});
