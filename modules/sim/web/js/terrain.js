export const setupTerrainToggle = (map, config) => {
  if (!config.dem.available) {
    return;
  }
  let terrainEnabled = false;
  const updateTerrain = () => {
    const shouldEnable = map.getPitch() >= config.dem.pitchThreshold;
    if (shouldEnable === terrainEnabled) {
      return;
    }
    if (shouldEnable) {
      if (map.getSource("dem")) {
        map.setTerrain({ source: "dem", exaggeration: config.dem.exaggeration });
      }
    } else {
      map.setTerrain(null);
    }
    terrainEnabled = shouldEnable;
  };
  map.on("load", () => {
    updateTerrain();
    map.on("pitch", updateTerrain);
  });
};
