import { logStatus } from "./status_log.js";

const STATE_ENDPOINT = "/api/sim/state";
const MISSION_ENDPOINT = "/api/sim/mission";
const PLAY_ENDPOINT = "/api/sim/play";
const PAUSE_ENDPOINT = "/api/sim/pause";
const STOP_ENDPOINT = "/api/sim/stop";
const CLEAR_ENDPOINT = "/api/sim/clear";
const RESET_ENDPOINT = "/api/sim/reset";
const SPEED_ENDPOINT = "/api/sim/speed";
const NEXT_MISSION_ENDPOINT = "/api/sim/next_mission";

const fetchJson = async (url, options) => {
  const response = await fetch(url, options);
  const payload = await response.json();
  return payload;
};

export const initSimClient = () => {
  let pollTimer = null;
  let lastState = null;
  const subscribers = new Set();

  const notify = () => {
    subscribers.forEach((cb) => {
      try {
        cb(lastState);
      } catch (err) {
        // ignore
      }
    });
  };

  const updateMarkers = (state) => {
    if (!state || !state.vehicles) {
      return;
    }
    if (typeof window.missionVehicleLoader === "function") {
      window.missionVehicleLoader({ ok: true, vehicles: state.vehicles });
    }
  };

  const poll = async () => {
    try {
      const state = await fetchJson(STATE_ENDPOINT, { method: "GET" });
      lastState = state;
      updateMarkers(state);
      notify();
    } catch (err) {
      logStatus("SIM state fetch failed", { level: "warn", ttlMs: 4000 });
    }
  };

  const startPolling = (intervalMs = 200) => {
    if (pollTimer) {
      return;
    }
    poll();
    pollTimer = setInterval(poll, Math.max(80, intervalMs));
  };

  const stopPolling = () => {
    if (pollTimer) {
      clearInterval(pollTimer);
      pollTimer = null;
    }
  };

  const setMission = async (payload) => {
    try {
      const result = await fetchJson(MISSION_ENDPOINT, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload || {}),
      });
      if (!result.ok) {
        throw new Error(result.error || "mission load failed");
      }
      logStatus("SIM mission loaded", { level: "success", ttlMs: 3000 });
      return result;
    } catch (err) {
      logStatus(`SIM mission failed: ${err.message}`, { level: "error", ttlMs: 5000 });
      return { ok: false, error: err.message };
    }
  };

  const play = async () => {
    try {
      const result = await fetchJson(PLAY_ENDPOINT, { method: "POST" });
      if (!result.ok) {
        throw new Error(result.error || "play failed");
      }
      return result;
    } catch (err) {
      logStatus(`SIM play failed: ${err.message}`, { level: "error", ttlMs: 5000 });
      return { ok: false, error: err.message };
    }
  };

  const pause = async () => {
    try {
      return await fetchJson(PAUSE_ENDPOINT, { method: "POST" });
    } catch (err) {
      logStatus(`SIM pause failed: ${err.message}`, { level: "error", ttlMs: 5000 });
      return { ok: false, error: err.message };
    }
  };

  const stop = async () => {
    try {
      return await fetchJson(STOP_ENDPOINT, { method: "POST" });
    } catch (err) {
      logStatus(`SIM stop failed: ${err.message}`, { level: "error", ttlMs: 5000 });
      return { ok: false, error: err.message };
    }
  };

  const reset = async () => {
    try {
      return await fetchJson(RESET_ENDPOINT, { method: "POST" });
    } catch (err) {
      logStatus(`SIM reset failed: ${err.message}`, { level: "error", ttlMs: 5000 });
      return { ok: false, error: err.message };
    }
  };

  const clear = async () => {
    try {
      return await fetchJson(CLEAR_ENDPOINT, { method: "POST" });
    } catch (err) {
      logStatus(`SIM clear failed: ${err.message}`, { level: "error", ttlMs: 5000 });
      return { ok: false, error: err.message };
    }
  };

  const setSpeed = async (speed) => {
    try {
      const result = await fetchJson(SPEED_ENDPOINT, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ speed }),
      });
      if (!result.ok) {
        throw new Error(result.error || "speed set failed");
      }
      return result;
    } catch (err) {
      logStatus(`SIM speed failed: ${err.message}`, { level: "error", ttlMs: 5000 });
      return { ok: false, error: err.message };
    }
  };

  const nextMission = async (payload) => {
    try {
      const result = await fetchJson(NEXT_MISSION_ENDPOINT, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload || {}),
      });
      if (!result.ok) {
        throw new Error(result.error || "next mission failed");
      }
      logStatus("SIM next mission", { level: "success", ttlMs: 3000 });
      return result;
    } catch (err) {
      logStatus(`SIM next mission failed: ${err.message}`, { level: "error", ttlMs: 5000 });
      return { ok: false, error: err.message };
    }
  };

  const subscribe = (cb) => {
    if (typeof cb !== "function") {
      return () => {};
    }
    subscribers.add(cb);
    return () => subscribers.delete(cb);
  };

  return {
    startPolling,
    stopPolling,
    setMission,
    play,
    pause,
    stop,
    clear,
    reset,
    setSpeed,
    nextMission,
    subscribe,
    getState: () => lastState,
  };
};
