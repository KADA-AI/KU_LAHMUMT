const EPOCH_2000 = Date.UTC(2000, 0, 1, 0, 0, 0);
const subscribers = new Set();
const store = new Map();
let activeAgent = null;

const DEFAULT_UI_STATE = {
  health: 1,
  fuelConsumption: 1.0,
  payloadHealth: 1,
  fuelWarning: 0,
  flightMode: 0,
  onMission: 0,
  currentWaypointID: 0,
  targetID: 0,
  weapons: { type1: 5, type2: 10, type3: 100 },
  datalink: { uav1: true, uav2: true, uav3: true },
  sensor: { operationalMode: 0, sensorType: 0, fov: 60 },
};

const deepCopy = (value) => JSON.parse(JSON.stringify(value));

const nowSince2000 = () => Math.max(0, Date.now() - EPOCH_2000);

const clamp = (value, min, max) => Math.min(max, Math.max(min, value));

const parseLabelIndex = (label) => {
  const match = String(label).match(/(\d+)/);
  if (!match) {
    return 0;
  }
  return Math.max(0, Number(match[1]) - 1);
};

export const isManned = (label) => String(label).toUpperCase().startsWith("LAH");

const getBaseCenter = () => {
  const lat = Number(document.body.dataset.centerLat) || 38.057393;
  const lon = Number(document.body.dataset.centerLon) || 127.41063;
  return { lat, lon };
};

const makeBaseData = (label) => {
  const manned = isManned(label);
  const idx = parseLabelIndex(label);
  const { lat: baseLat, lon: baseLon } = getBaseCenter();
  const latOffset = (manned ? -0.015 : 0.02) + idx * 0.01;
  const lonOffset = (manned ? 0.01 : -0.012) + idx * 0.012;
  const lat = baseLat + latOffset;
  const lon = baseLon + lonOffset;
  const altitude = 180 + idx * 20 + (manned ? 40 : 0);
  const aircraftID = manned ? idx + 1 : idx + 4;

  const loiter = {
    latitude: lat + 0.005,
    longitude: lon - 0.004,
    altitude: altitude + 200,
  };
  const corners = [
    { latitude: lat + 0.002, longitude: lon - 0.002, altitude },
    { latitude: lat + 0.002, longitude: lon + 0.002, altitude },
    { latitude: lat - 0.002, longitude: lon + 0.002, altitude },
    { latitude: lat - 0.002, longitude: lon - 0.002, altitude },
  ];

  return {
    aircraftID,
    isUnmanned: !manned,
    coordinate: { latitude: lat, longitude: lon, altitude },
    velocity: {
      speed: 45 + idx * 6 + (manned ? 6 : 0),
      heading: 70 + idx * 15,
    },
    fuelBase: 820 - idx * 60,
    lastSignalOffset: 1200 + idx * 340,
    leaderAircraftID: { aircraftID: manned ? 1 : 4 },
    loiterCoordinate: loiter,
    sensorInfo: {
      operationalMode: 2,
      sensorType: 2,
      fov: 60,
      centerCoordinate: { latitude: lat, longitude: lon, altitude },
      footprintCornerList: corners,
    },
  };
};

const ensureAgent = (label) => {
  const key = String(label || "");
  if (!store.has(key)) {
    store.set(key, {
      ui: deepCopy(DEFAULT_UI_STATE),
      base: makeBaseData(key),
    });
  }
  return store.get(key);
};

const setNested = (state, path, value) => {
  const keys = path.split(".");
  let node = state;
  keys.slice(0, -1).forEach((key) => {
    if (!node[key]) {
      node[key] = {};
    }
    node = node[key];
  });
  node[keys[keys.length - 1]] = value;
};

const notify = (event) => {
  subscribers.forEach((handler) => {
    handler(event);
  });
};

export const subscribe = (handler) => {
  subscribers.add(handler);
  return () => subscribers.delete(handler);
};

export const getUiState = (label) => ensureAgent(label).ui;

export const updateUiField = (label, path, value) => {
  const entry = ensureAgent(label);
  setNested(entry.ui, path, value);
  notify({ type: "update", label });
};

export const setActiveAgent = (label) => {
  activeAgent = label || null;
  notify({ type: "active", label: activeAgent });
};

export const getActiveAgent = () => activeAgent;

export const buildAgentState = (label) => {
  const entry = ensureAgent(label);
  const { base, ui } = entry;
  const now = nowSince2000();
  const fuel = clamp(base.fuelBase - ui.fuelConsumption * 50, 0, 1000);

  return {
    aircraftID: base.aircraftID,
    isUnmanned: base.isUnmanned,
    coordinate: base.coordinate,
    velocity: base.velocity,
    fuel,
    health: ui.health,
    lastSignalTime: Math.max(0, now - base.lastSignalOffset),
    mannedInfo: {
      weapons: ui.weapons,
      datalinkStatus: {
        isConnectedToUAV1: ui.datalink.uav1,
        isConnectedToUAV2: ui.datalink.uav2,
        isConnectedToUAV3: ui.datalink.uav3,
      },
    },
    unmannedInfo: {
      currentWaypointID: ui.currentWaypointID,
      flightMode: ui.flightMode,
      onMission: ui.onMission,
      loiterCoordinate: base.loiterCoordinate,
      targetFollowing: { targetID: ui.targetID },
      leaderAircraftID: base.leaderAircraftID,
      sensorInfo: base.sensorInfo,
      payloadHealth: ui.payloadHealth,
      fuelWarning: ui.fuelWarning,
    },
  };
};

export const buildAgentStatus = (label) => ({
  timestamp: nowSince2000(),
  source: "SIM_UI",
  agentStateList: [buildAgentState(label)],
});

export const getAgentCoordinate = (label) => {
  const entry = ensureAgent(label);
  return entry && entry.base ? entry.base.coordinate : null;
};
