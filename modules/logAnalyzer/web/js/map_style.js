/**
 * Build a MapLibre style object for the dark military-blue theme.
 * Uses PBF vector tiles from mbtiles, same pattern as the sim module.
 */
export const buildMapStyle = (config, palette) => {
  const sources = {
    mbtiles: {
      type: "vector",
      tiles: [config.tileUrl],
      minzoom: 0,
      maxzoom: 14,
    },
  };

  const layers = [
    {
      id: "background",
      type: "background",
      paint: { "background-color": palette.background },
    },
    {
      id: "landcover",
      type: "fill",
      source: "mbtiles",
      "source-layer": "landcover",
      paint: { "fill-color": palette.landcover, "fill-opacity": 0.7 },
    },
    {
      id: "landuse",
      type: "fill",
      source: "mbtiles",
      "source-layer": "landuse",
      paint: { "fill-color": palette.landuse, "fill-opacity": 0.7 },
    },
    {
      id: "park",
      type: "fill",
      source: "mbtiles",
      "source-layer": "park",
      paint: { "fill-color": palette.park, "fill-opacity": 0.85 },
    },
    {
      id: "water",
      type: "fill",
      source: "mbtiles",
      "source-layer": "water",
      paint: { "fill-color": palette.water },
    },
    {
      id: "waterway",
      type: "line",
      source: "mbtiles",
      "source-layer": "waterway",
      paint: { "line-color": palette.waterway, "line-width": 1 },
    },
    {
      id: "boundary",
      type: "line",
      source: "mbtiles",
      "source-layer": "boundary",
      paint: {
        "line-color": palette.boundary,
        "line-width": 1,
        "line-dasharray": [2, 2],
      },
    },
    {
      id: "transportation",
      type: "line",
      source: "mbtiles",
      "source-layer": "transportation",
      paint: { "line-color": palette.transportation, "line-width": 1 },
    },
    {
      id: "building",
      type: "fill",
      source: "mbtiles",
      "source-layer": "building",
      minzoom: 13,
      paint: { "fill-color": palette.building, "fill-opacity": 0.6 },
    },
    {
      id: "place-labels",
      type: "symbol",
      source: "mbtiles",
      "source-layer": "place",
      minzoom: 4,
      layout: {
        "text-field": ["coalesce", ["get", "name:latin"], ["get", "name"]],
        "text-font": ["Noto Sans Regular"],
        "text-size": ["interpolate", ["linear"], ["zoom"], 5, 10, 9, 13, 12, 16],
        "text-transform": "uppercase",
      },
      paint: {
        "text-color": palette.label,
        "text-opacity": 0.2,
        "text-halo-color": palette.labelHalo,
        "text-halo-width": 1.2,
        "text-halo-blur": 0.6,
      },
    },
    {
      id: "water-labels",
      type: "symbol",
      source: "mbtiles",
      "source-layer": "water_name",
      minzoom: 10,
      layout: {
        "text-field": ["coalesce", ["get", "name:latin"], ["get", "name"]],
        "text-font": ["Noto Sans Regular"],
        "text-size": ["interpolate", ["linear"], ["zoom"], 10, 11, 14, 14],
      },
      paint: {
        "text-color": "#4a6a8a",
        "text-opacity": 0.25,
        "text-halo-color": palette.labelHalo,
        "text-halo-width": 1.1,
        "text-halo-blur": 0.5,
      },
    },
    {
      id: "road-labels",
      type: "symbol",
      source: "mbtiles",
      "source-layer": "transportation_name",
      minzoom: 12,
      layout: {
        "symbol-placement": "line",
        "text-field": ["coalesce", ["get", "name:latin"], ["get", "name"], ["get", "ref"]],
        "text-font": ["Noto Sans Regular"],
        "text-size": ["interpolate", ["linear"], ["zoom"], 12, 11, 15, 13],
      },
      paint: {
        "text-color": "#5a6a7a",
        "text-opacity": 0.2,
        "text-halo-color": palette.labelHalo,
        "text-halo-width": 1.0,
        "text-halo-blur": 0.5,
      },
    },
  ];

  return {
    version: 8,
    sources,
    glyphs: "https://demotiles.maplibre.org/font/{fontstack}/{range}.pbf",
    layers,
  };
};
