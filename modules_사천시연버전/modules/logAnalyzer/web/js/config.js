const parseNumber = (value, fallback) => {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
};

const parseJson = (value, fallback) => {
  if (!value || value === "null") return fallback;
  try {
    return JSON.parse(value);
  } catch {
    return fallback;
  }
};

export const getConfig = (body = document.body) => ({
  tileUrl: body.dataset.tileUrl,
  minZoom: parseNumber(body.dataset.minZoom, 6),
  maxZoom: parseNumber(body.dataset.maxZoom, 18),
  center: [
    parseNumber(body.dataset.centerLon, 127.0),
    parseNumber(body.dataset.centerLat, 37.5),
  ],
  startZoom: parseNumber(body.dataset.startZoom, 10.5),
  bounds: parseJson(body.dataset.bounds, null),
});
