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
const TARGET_ADD_ENDPOINT = "/api/sim/targets/add";
const TARGET_CLEAR_ENDPOINT = "/api/sim/targets/clear";

const fetchJson = async (url, options) => {
  const response = await fetch(url, options);
  const payload = await response.json();
  return payload;
};

export const initSimClient = () => {
  let pollTimer = null;
  let lastState = null;
  let lastStep = null;
  let frameQueue = [];
  let draining = false;
  let suspendRender = typeof document !== "undefined" ? document.hidden : false;
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
    if (!state) {
      return;
    }
    if (typeof window.missionVehicleLoader === "function") {
      window.missionVehicleLoader({
        ok: true,
        vehicles: state.vehicles || {},
        step: state.step,
      });
    }
    if (typeof window.missionTargetLoader === "function") {
      window.missionTargetLoader({
        ok: true,
        targets: state.targets || [],
        step: state.step,
      });
    }
    if (typeof window.missionProjectileLoader === "function") {
      window.missionProjectileLoader({
        ok: true,
        projectiles: state.projectiles || [],
        step: state.step,
      });
    }
    if (typeof window.missionEffectLoader === "function") {
      window.missionEffectLoader({
        ok: true,
        effects: state.effects || [],
        step: state.step,
      });
    }
  };

  const enqueueFrames = (frames) => {
    if (!Array.isArray(frames) || frames.length === 0) {
      return;
    }
    frameQueue.push(...frames);
    if (!draining) {
      draining = true;
      requestAnimationFrame(drainQueue);
    }
  };

  const drainQueue = () => {
    if (suspendRender) {
      frameQueue = [];
      draining = false;
      return;
    }
    if (!frameQueue.length) {
      draining = false;
      return;
    }
    const size = frameQueue.length;
    const batch = size > 600 ? 6 : size > 300 ? 4 : size > 120 ? 2 : 1;
    for (let i = 0; i < batch && frameQueue.length; i += 1) {
      const frame = frameQueue.shift();
      updateMarkers(frame);
    }
    requestAnimationFrame(drainQueue);
  };

  const poll = async () => {
    try {
      const query = Number.isFinite(lastStep) ? `?since=${lastStep}` : "";
      const state = await fetchJson(`${STATE_ENDPOINT}${query}`, { method: "GET" });
      if (state && Array.isArray(state.history)) {
        if (state.history.length) {
          if (!suspendRender) {
            enqueueFrames(state.history.map((frame) => ({ ok: true, ...frame })));
          }
          const last = state.history[state.history.length - 1];
          if (Number.isFinite(last?.step)) {
            lastStep = last.step;
          }
        }
        if (state.latest) {
          lastState = state.latest;
          const vehiclesEmpty =
            !state.latest.vehicles || Object.keys(state.latest.vehicles).length === 0;
          if (vehiclesEmpty) {
            frameQueue = [];
            lastStep = null;
          }
          if (Number.isFinite(state.latest.step)) {
            if (Number.isFinite(lastStep) && state.latest.step < lastStep) {
              lastStep = state.latest.step;
            } else {
              lastStep = state.latest.step;
            }
          }
          if (!state.history.length && !suspendRender) {
            updateMarkers(state.latest);
          }
        }
        notify();
        return;
      }
      lastState = state;
      if (!suspendRender) {
        updateMarkers(state);
      }
      if (Number.isFinite(state?.step)) {
        lastStep = state.step;
      }
      notify();
    } catch (err) {
      logStatus("SIM state fetch failed", { level: "warn", ttlMs: 4000 });
    }
  };

  const startPolling = (intervalMs = 80) => {
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

  const addTarget = async (payload) => {
    try {
      const result = await fetchJson(TARGET_ADD_ENDPOINT, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload || {}),
      });
      if (!result.ok) {
        throw new Error(result.error || "target add failed");
      }
      return result;
    } catch (err) {
      logStatus(`SIM target failed: ${err.message}`, { level: "error", ttlMs: 5000 });
      return { ok: false, error: err.message };
    }
  };

  const clearTargets = async () => {
    try {
      return await fetchJson(TARGET_CLEAR_ENDPOINT, { method: "POST" });
    } catch (err) {
      logStatus(`SIM target clear failed: ${err.message}`, { level: "error", ttlMs: 5000 });
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

  if (typeof document !== "undefined") {
    document.addEventListener("visibilitychange", () => {
      suspendRender = document.hidden;
      if (!suspendRender) {
        frameQueue = [];
        draining = false;
        if (lastState) {
          updateMarkers(lastState);
        }
      }
    });
  }

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
    addTarget,
    clearTargets,
    subscribe,
    getState: () => lastState,
  };
};
