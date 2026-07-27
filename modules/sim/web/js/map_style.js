export const BUILDING_3D_LAYER_ID = "building-3d";
export const BUILDING_2D_LAYER_ID = "building";
export const BUILDING_3D_MIN_ZOOM = 11;
export const BUILDING_3D_OPACITY = 0.72;
export const BUILDING_3D_HEIGHT_SCALE = 0.8;
export const BUILDING_3D_MAX_HEIGHT = 160;
export const BUILDING_3D_COLOR = "#747d60";
export const HILLSHADE_LAYER_ID = "dem-hillshade";
export const ELEVATION_TINT_LAYER_ID = "dem-elevation-tint";

export const buildStyle = (config, palette) => {
  const sources = {
    mbtiles: {
      type: "vector",
      tiles: [config.tileUrl],
      minzoom: config.minZoom,
      maxzoom: config.maxZoom,
    },
  };
  if (config.dem.available) {
    const demSource = {
      type: "raster-dem",
      tiles: [config.dem.tileUrl],
      tileSize: config.dem.tileSize,
      maxzoom: config.dem.maxZoom,
      encoding: config.dem.encoding,
    };
    if (config.dem.bounds) {
      demSource.bounds = [
        config.dem.bounds[0][0],
        config.dem.bounds[0][1],
        config.dem.bounds[1][0],
        config.dem.bounds[1][1],
      ];
    }
    // Terrain, color relief, and hillshade can share one raster-dem source.
    // Registering the same URL twice creates separate source/tile lifecycles.
    sources["dem-terrain"] = demSource;
  }

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
      id: BUILDING_2D_LAYER_ID,
      type: "fill",
      source: "mbtiles",
      "source-layer": "building",
      minzoom: BUILDING_3D_MIN_ZOOM,
      paint: { "fill-color": palette.building, "fill-opacity": 0.6 },
    },
  ];

  if (config.dem.available) {
    const reliefInsertIndex = layers.findIndex((layer) => layer.id === "water");
    layers.splice(
      reliefInsertIndex,
      0,
      {
        // Reuse the already decoded terrain DEM. This adds no DEM request or
        // server-side contour calculation; it is only one inexpensive draw pass.
        id: ELEVATION_TINT_LAYER_ID,
        type: "color-relief",
        source: "dem-terrain",
        paint: {
          "color-relief-opacity": [
            "interpolate",
            ["linear"],
            ["zoom"],
            7,
            0.1,
            11,
            0.16,
            14,
            0.2,
          ],
          "color-relief-color": [
            "interpolate",
            ["linear"],
            ["elevation"],
            0,
            "#29452d",
            250,
            "#365033",
            500,
            "#4f603b",
            800,
            "#696b43",
            1200,
            "#7d704f",
            1800,
            "#91866f",
            2500,
            "#aaa69a",
          ],
        },
      },
      {
        id: HILLSHADE_LAYER_ID,
        type: "hillshade",
        source: "dem-terrain",
        paint: {
          // Relief lighting only: the physical terrain remains at the exact
          // 1.0x DEM height. Keep this above green land fills so it is visible.
          "hillshade-exaggeration": [
            "interpolate",
            ["linear"],
            ["zoom"],
            7,
            0.35,
            11,
            0.42,
            14,
            0.48,
          ],
          "hillshade-illumination-direction": 315,
          "hillshade-illumination-altitude": 38,
          "hillshade-illumination-anchor": "map",
          "hillshade-method": "standard",
          "hillshade-shadow-color": "rgba(8, 13, 9, 0.46)",
          "hillshade-highlight-color": "rgba(235, 238, 215, 0.3)",
          "hillshade-accent-color": "rgba(70, 88, 58, 0.26)",
        },
      }
    );
  }

  layers.push(
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
        "text-opacity": 0.15,
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
        "text-color": "#c9d7ff",
        "text-opacity": 0.15,
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
        "text-color": "#d5dbc8",
        "text-opacity": 0.15,
        "text-halo-color": palette.labelHalo,
        "text-halo-width": 1.0,
        "text-halo-blur": 0.5,
      },
    }
  );

  return {
    version: 8,
    sources,
    glyphs: "https://demotiles.maplibre.org/font/{fontstack}/{range}.pbf",
    layers,
  };
};
