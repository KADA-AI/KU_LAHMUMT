import {
  BUILDING_2D_LAYER_ID,
  BUILDING_3D_COLOR,
  BUILDING_3D_HEIGHT_SCALE,
  BUILDING_3D_LAYER_ID,
  BUILDING_3D_MAX_HEIGHT,
  BUILDING_3D_MIN_ZOOM,
  BUILDING_3D_OPACITY,
} from "./map_style.js";

export const createBuildingController = (map, toggleEl, setStatus) => {
  let enabled = false;
  let pending = null;

  const ensureLayer = () => {
    if (!map || map.getLayer(BUILDING_3D_LAYER_ID)) {
      return;
    }
    if (!map.getSource("mbtiles")) {
      if (setStatus) {
        setStatus("Base map source not ready.");
      }
      return;
    }
    map.addLayer({
      id: BUILDING_3D_LAYER_ID,
      type: "fill-extrusion",
      source: "mbtiles",
      "source-layer": "building",
      minzoom: BUILDING_3D_MIN_ZOOM,
      layout: { visibility: "none" },
      paint: {
        "fill-extrusion-color": BUILDING_3D_COLOR,
        "fill-extrusion-opacity": BUILDING_3D_OPACITY,
        "fill-extrusion-height": [
          "min",
          BUILDING_3D_MAX_HEIGHT,
          [
            "*",
            BUILDING_3D_HEIGHT_SCALE,
            ["coalesce", ["get", "render_height"], ["get", "height"], 12],
          ],
        ],
        "fill-extrusion-base": [
          "*",
          BUILDING_3D_HEIGHT_SCALE,
          ["coalesce", ["get", "render_min_height"], ["get", "min_height"], 0],
        ],
        "fill-extrusion-vertical-gradient": true,
      },
    });
  };

  const applyEnabled = (next) => {
    const visibility = next ? "visible" : "none";
    if (map.getLayer(BUILDING_3D_LAYER_ID)) {
      map.setLayoutProperty(BUILDING_3D_LAYER_ID, "visibility", visibility);
    }
    if (map.getLayer(BUILDING_2D_LAYER_ID)) {
      map.setLayoutProperty(BUILDING_2D_LAYER_ID, "visibility", next ? "none" : "visible");
    }
    map.triggerRepaint();
  };

  const setEnabled = (nextValue) => {
    enabled = Boolean(nextValue);
    if (toggleEl) {
      toggleEl.classList.toggle("is-active", enabled);
    }
    if (!map || !map.isStyleLoaded()) {
      pending = enabled;
      return;
    }
    pending = null;
    ensureLayer();
    applyEnabled(enabled);
  };

  if (toggleEl) {
    toggleEl.addEventListener("click", () => {
      setEnabled(!enabled);
    });
  }

  return {
    ensureLayer,
    setEnabled,
    applyPending: () => {
      if (pending !== null) {
        setEnabled(pending);
      }
    },
  };
};
