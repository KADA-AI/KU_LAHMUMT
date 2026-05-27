import { logStatus } from "./status_log.js";
import { reportMissionValidation } from "./mission_validation_alert.js";

const SIM_STATE_ENDPOINT = "/api/sim/state";
const MONITOR_STATE_ENDPOINT = "/api/monitoring/state";
const MISSION_ENDPOINT = "/api/sim/mission";
const PLAY_ENDPOINT = "/api/sim/play";
const PAUSE_ENDPOINT = "/api/sim/pause";
const STOP_ENDPOINT = "/api/sim/stop";
const CLEAR_ENDPOINT = "/api/sim/clear";
const RESET_ENDPOINT = "/api/sim/reset";
const SPEED_ENDPOINT = "/api/sim/speed";
const NEXT_MISSION_ENDPOINT = "/api/sim/next_mission";
const AGENT_STATE_ENDPOINT = "/api/sim/agent_state";
const FORCE_COMMAND_ENDPOINT = "/api/sim/force_command";
const TARGET_ADD_ENDPOINT = "/api/sim/targets/add";
const ROI_ADD_ENDPOINT = "/api/sim/roi/add";
const TARGET_CLEAR_ENDPOINT = "/api/sim/targets/clear";
const INTEGRATION_RESET_ENDPOINT = "/api/integration/reset";
const INTEGRATION_SETTINGS_ENDPOINT = "/api/integration/settings";
const INTEGRATION_ACTIVATE_ENDPOINT = "/api/integration/activate";
const SIM_POLL_INTERVAL_MS = 120;
const MONITOR_POLL_INTERVAL_MS = 200;

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
  let lastSpeedFactor = null;
  let speedRampTimer = null;
  let speedRampSeq = 0;
  let skipHistoryOnce = false;
  let catchUpMode = false;
  let forceCatchUp = false;
  let mode = "sim";
  let currentPollIntervalMs = SIM_POLL_INTERVAL_MS;
  let missionSignature = null;
  let lastFetchWarnAt = 0;
  const subscribers = new Set();
  const SPEED_RAMP_SEQUENCE = [10, 8, 6, 4, 2, 1];
  const SPEED_RAMP_INTERVAL_MS = 200;

  const readCurrentWaypointId = (entry) => {
    if (entry === null || entry === undefined) {
      return null;
    }
    if (typeof entry === "object") {
      return readCurrentWaypointId(
        entry.waypointID ?? entry.WaypointID ?? entry.id ?? entry.ID ?? null,
      );
    }
    const parsed = Number(entry);
    return Number.isFinite(parsed) ? Math.trunc(parsed) : null;
  };

  const deriveCurrentWaypoints = (state) => {
    const result = {};
    const direct = state?.currentWaypoints;
    if (direct && typeof direct === "object" && !Array.isArray(direct)) {
      Object.entries(direct).forEach(([agent, value]) => {
        const wpId = readCurrentWaypointId(value);
        if (Number.isFinite(wpId)) {
          result[String(agent || "").toUpperCase()] = wpId;
        }
      });
      return result;
    }
    const vehicles = state?.vehicles || {};
    Object.entries(vehicles).forEach(([agent, entry]) => {
      const wpId = readCurrentWaypointId(
        entry?.currentWaypointID ??
          entry?.CurrentWaypointID ??
          entry?.unmannedInfo?.currentWaypointID ??
          entry?.unmannedInfo?.CurrentWaypointID ??
          null,
      );
      if (Number.isFinite(wpId)) {
        result[String(agent || "").toUpperCase()] = wpId;
      }
    });
    return result;
  };

  const updateMissionState = (state) => {
    const currentWaypoints = deriveCurrentWaypoints(state);
    if (typeof window.setMissionCurrentWaypoints === "function") {
      window.setMissionCurrentWaypoints(currentWaypoints);
    }
    const mission = state?.mission;
    const monitorMode = state?.mode === "monitor";
    const currentSignature = state?.missionSignature || null;
    if (!mission) {
      if (monitorMode && state?.missionAvailable === false) {
        missionSignature = null;
        if (typeof window.missionPathLoader === "function") {
          window.missionPathLoader({
            ok: true,
            features: [],
            agents: {},
            count: 0,
            flightPaths: [],
          });
        }
        if (typeof window.setMissionPlanReady === "function") {
          window.setMissionPlanReady(false);
        }
      }
      return;
    }
    if (typeof window.missionPathLoader !== "function") {
      return;
    }
    const signature =
      mission.signature ||
      currentSignature ||
      `${mission.missionPlanID ?? "none"}:${mission.source ?? "unknown"}:${mission.selectedTimestamp ?? 0}`;
    if (signature === missionSignature) {
      return;
    }
    missionSignature = signature;
    window.missionPathLoader(mission);
    reportMissionValidation(mission.validation, `Mission ${mission.missionPlanID ?? ""}`.trim());
    if (typeof window.setMissionPlanReady === "function") {
      window.setMissionPlanReady(Boolean(mission.ok && Array.isArray(mission.features) && mission.features.length));
    }
  };

  const resetQueueToState = (state, backtrack = 0) => {
    frameQueue = [];
    draining = false;
    skipHistoryOnce = true;
    const nextStep = Number(state?.step);
    if (Number.isFinite(nextStep)) {
      const rollback = Number.isFinite(backtrack) ? Math.max(0, Number(backtrack)) : 0;
      lastStep = Math.max(0, nextStep - rollback);
    }
    if (!suspendRender && state) {
      updateMarkers(state);
    }
  };

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
        rois: state.rois || [],
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
    updateMissionState(state);
  };

  const downsampleHistory = (history, speedFactor) => {
    if (!Array.isArray(history) || history.length <= 1) {
      return history || [];
    }
    const speed = Number(speedFactor);
    if (!Number.isFinite(speed) || speed <= 1) {
      return history;
    }
    const step = Math.max(2, Math.round(speed));
    const sampled = [];
    for (let i = 0; i < history.length; i += step) {
      sampled.push(history[i]);
    }
    const last = history[history.length - 1];
    if (sampled[sampled.length - 1] !== last) {
      sampled.push(last);
    }
    return sampled;
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
      if (catchUpMode) {
        catchUpMode = false;
        skipHistoryOnce = true;
      }
      draining = false;
      return;
    }
    const size = frameQueue.length;
    let batch = size > 600 ? 6 : size > 300 ? 4 : size > 120 ? 2 : 1;
    if (catchUpMode) {
      batch = size > 600 ? 14 : size > 300 ? 10 : size > 120 ? 6 : size > 40 ? 3 : 2;
    }
    for (let i = 0; i < batch && frameQueue.length; i += 1) {
      const frame = frameQueue.shift();
      updateMarkers(frame);
    }
    requestAnimationFrame(drainQueue);
  };

  const poll = async () => {
    try {
      const endpoint = mode === "monitor" ? MONITOR_STATE_ENDPOINT : SIM_STATE_ENDPOINT;
      const query =
        mode === "monitor"
          ? `?missionSince=${encodeURIComponent(missionSignature || "")}`
          : mode === "sim" && Number.isFinite(lastStep)
            ? `?since=${lastStep}`
            : "";
      const state = await fetchJson(`${endpoint}${query}`, { method: "GET" });
      if (mode === "monitor") {
        frameQueue = [];
        draining = false;
        catchUpMode = false;
        skipHistoryOnce = true;
        lastState = state || null;
        if (!suspendRender && state) {
          updateMarkers(state);
        }
        if (Number.isFinite(state?.step)) {
          lastStep = state.step;
        }
        notify();
        return;
      }
      if (catchUpMode) {
        const speed = Number(state?.latest?.speedFactor ?? state?.speedFactor);
        if (Number.isFinite(speed)) {
          lastSpeedFactor = speed;
        }
        if (!frameQueue.length) {
          catchUpMode = false;
          skipHistoryOnce = true;
        }
        return;
      }
      if (state && Array.isArray(state.history)) {
        const speedValue = Number(state.latest?.speedFactor);
        const latestOnly =
          (Number.isFinite(speedValue) && speedValue >= 3) || state.history.length > 80;
        if (latestOnly) {
          const latest =
            state.latest ||
            (state.history.length ? state.history[state.history.length - 1] : null);
          if (Number.isFinite(speedValue)) {
            lastSpeedFactor = speedValue;
          }
          frameQueue = [];
          draining = false;
          catchUpMode = false;
          skipHistoryOnce = true;
          if (latest) {
            lastState = latest;
            if (!suspendRender) {
              updateMarkers(latest);
            }
            if (Number.isFinite(latest?.step)) {
              lastStep = latest.step;
            }
          }
          notify();
          return;
        }
        if (skipHistoryOnce) {
          if (Number.isFinite(speedValue)) {
            lastSpeedFactor = speedValue;
          }
          const latest =
            state.latest ||
            (state.history.length ? state.history[state.history.length - 1] : null);
          if (latest) {
            lastState = latest;
            if (!suspendRender) {
              updateMarkers(latest);
            }
            if (Number.isFinite(latest?.step)) {
              lastStep = latest.step;
            }
          }
          frameQueue = [];
          draining = false;
          skipHistoryOnce = false;
          notify();
          return;
        }
        let shouldCatchUp = false;
        if (Number.isFinite(speedValue)) {
          if (lastSpeedFactor !== null && speedValue !== lastSpeedFactor) {
            if (speedValue < lastSpeedFactor) {
              shouldCatchUp = true;
            } else {
              resetQueueToState(state.latest || state, 10);
            }
          }
          lastSpeedFactor = speedValue;
        }
        if (forceCatchUp) {
          shouldCatchUp = true;
        }
        if (state.history.length) {
          if (!suspendRender) {
            const historySpeed = Number.isFinite(speedValue) ? speedValue : lastSpeedFactor;
            const sampled = downsampleHistory(state.history, historySpeed);
            enqueueFrames(sampled.map((frame) => ({ ok: true, ...frame })));
          }
          const last = state.history[state.history.length - 1];
          if (Number.isFinite(last?.step)) {
            lastStep = last.step;
          }
        }
        if (shouldCatchUp && frameQueue.length) {
          catchUpMode = true;
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
        if (forceCatchUp) {
          forceCatchUp = false;
        }
        notify();
        return;
      }
      const speed = Number(state?.speedFactor);
      if (skipHistoryOnce) {
        if (Number.isFinite(speed)) {
          lastSpeedFactor = speed;
        }
        lastState = state;
        if (!suspendRender) {
          updateMarkers(state);
        }
        if (Number.isFinite(state?.step)) {
          lastStep = state.step;
        }
        frameQueue = [];
        draining = false;
        skipHistoryOnce = false;
        notify();
        return;
      }
      if (Number.isFinite(speed)) {
        if (lastSpeedFactor !== null && speed !== lastSpeedFactor) {
          if (speed < lastSpeedFactor) {
            if (frameQueue.length) {
              catchUpMode = true;
            }
          } else {
            resetQueueToState(state, 10);
          }
        }
        lastSpeedFactor = speed;
      }
      if (forceCatchUp) {
        forceCatchUp = false;
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
      const now = Date.now();
      if ((now - lastFetchWarnAt) >= 2500) {
        logStatus(mode === "monitor" ? "Monitoring state fetch failed" : "SIM state fetch failed", {
          level: "warn",
          ttlMs: 2500,
        });
        lastFetchWarnAt = now;
      }
    }
  };

  const startPolling = (intervalMs = currentPollIntervalMs) => {
    if (pollTimer) {
      return;
    }
    currentPollIntervalMs = Math.max(80, intervalMs);
    poll();
    pollTimer = setInterval(poll, currentPollIntervalMs);
  };

  const stopPolling = () => {
    if (pollTimer) {
      clearInterval(pollTimer);
      pollTimer = null;
    }
  };

  const setMode = (nextMode) => {
    const normalized = nextMode === "monitor" ? "monitor" : "sim";
    if (mode === normalized) {
      return mode;
    }
    mode = normalized;
    currentPollIntervalMs =
      mode === "monitor" ? MONITOR_POLL_INTERVAL_MS : SIM_POLL_INTERVAL_MS;
    frameQueue = [];
    draining = false;
    catchUpMode = false;
    forceCatchUp = false;
    skipHistoryOnce = true;
    lastStep = null;
    missionSignature = null;
    if (pollTimer) {
      stopPolling();
      startPolling(currentPollIntervalMs);
    }
    return mode;
  };

  const getMode = () => mode;

  if (typeof document !== "undefined") {
    document.addEventListener("visibilitychange", () => {
      suspendRender = document.hidden;
      if (suspendRender) {
        frameQueue = [];
        draining = false;
        stopPolling();
        return;
      }
      skipHistoryOnce = true;
      frameQueue = [];
      draining = false;
      if (lastState) {
        updateMarkers(lastState);
      }
      startPolling();
    });
  }

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
      reportMissionValidation(result.validation, "SIM mission");
      logStatus("SIM mission loaded", { level: "success", ttlMs: 3000 });
      return result;
    } catch (err) {
      logStatus(`SIM mission failed: ${err.message}`, { level: "error", ttlMs: 5000 });
      return { ok: false, error: err.message };
    }
  };

  const play = async (payload) => {
    try {
      const result = await fetchJson(PLAY_ENDPOINT, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload || {}),
      });
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
    const applySpeed = async (nextSpeed) => {
      try {
        const result = await fetchJson(SPEED_ENDPOINT, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ speed: nextSpeed }),
        });
        if (!result.ok) {
          throw new Error(result.error || "speed set failed");
        }
        if (Number.isFinite(result.speedFactor)) {
          lastSpeedFactor = result.speedFactor;
        }
        return result;
      } catch (err) {
        logStatus(`SIM speed failed: ${err.message}`, { level: "error", ttlMs: 5000 });
        return { ok: false, error: err.message };
      }
    };

    const cancelRamp = () => {
      speedRampSeq += 1;
      if (speedRampTimer) {
        clearTimeout(speedRampTimer);
        speedRampTimer = null;
      }
    };

    const buildRampSteps = (fromSpeed, toSpeed) => {
      if (!Number.isFinite(fromSpeed) || !Number.isFinite(toSpeed) || toSpeed >= fromSpeed) {
        return [toSpeed];
      }
      const steps = [];
      for (const step of SPEED_RAMP_SEQUENCE) {
        if (step < fromSpeed && step > toSpeed) {
          steps.push(step);
        }
      }
      steps.push(toSpeed);
      return steps;
    };

    const startRamp = (steps) => {
      if (!steps.length) {
        return Promise.resolve({ ok: true, speedFactor: speed });
      }
      cancelRamp();
      const seq = speedRampSeq;
      const queue = steps.slice();
      const first = queue.shift();
      const advance = () => {
        if (seq !== speedRampSeq) {
          return;
        }
        if (!queue.length) {
          speedRampTimer = null;
          return;
        }
        speedRampTimer = setTimeout(async () => {
          if (seq !== speedRampSeq) {
            return;
          }
          const next = queue.shift();
          await applySpeed(next);
          advance();
        }, SPEED_RAMP_INTERVAL_MS);
      };
      return applySpeed(first).finally(advance);
    };

    const numericSpeed = Number(speed);
    if (!Number.isFinite(numericSpeed)) {
      return { ok: false, error: "invalid speed" };
    }
    const current = Number.isFinite(lastSpeedFactor)
      ? lastSpeedFactor
      : Number(lastState?.speedFactor);
    if (Number.isFinite(current) && numericSpeed < current) {
      forceCatchUp = true;
      const rampSteps = buildRampSteps(current, numericSpeed);
      return startRamp(rampSteps);
    }
    cancelRamp();
    return applySpeed(numericSpeed);
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

  const setAgentState = async (payload) => {
    try {
      return await fetchJson(AGENT_STATE_ENDPOINT, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload || {}),
      });
    } catch (err) {
      logStatus(`SIM agent update failed: ${err.message}`, { level: "error", ttlMs: 4000 });
      return { ok: false, error: err.message };
    }
  };

  const forceCommand = async (payload) => {
    try {
      return await fetchJson(FORCE_COMMAND_ENDPOINT, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload || {}),
      });
    } catch (err) {
      logStatus(`SIM force command failed: ${err.message}`, { level: "error", ttlMs: 4000 });
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

  const addRoi = async (payload) => {
    try {
      const result = await fetchJson(ROI_ADD_ENDPOINT, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload || {}),
      });
      if (!result.ok) {
        throw new Error(result.error || "ROI add failed");
      }
      return result;
    } catch (err) {
      logStatus(`SIM ROI failed: ${err.message}`, { level: "error", ttlMs: 5000 });
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

  const resetIntegration = async () => {
    try {
      return await fetchJson(INTEGRATION_RESET_ENDPOINT, { method: "POST" });
    } catch (err) {
      logStatus(`INT reset failed: ${err.message}`, { level: "warn", ttlMs: 3000 });
      return { ok: false, error: err.message };
    }
  };

  const getIntegrationSettings = async () => {
    try {
      return await fetchJson(INTEGRATION_SETTINGS_ENDPOINT, { method: "GET" });
    } catch (err) {
      logStatus(`nFusion settings load failed: ${err.message}`, { level: "error", ttlMs: 4000 });
      return { ok: false, error: err.message };
    }
  };

  const saveIntegrationSettings = async (settings) => {
    try {
      return await fetchJson(INTEGRATION_SETTINGS_ENDPOINT, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(settings || {}),
      });
    } catch (err) {
      logStatus(`nFusion settings save failed: ${err.message}`, { level: "error", ttlMs: 4000 });
      return { ok: false, error: err.message };
    }
  };

  const ensureIntegrationReady = async () => {
    try {
      return await fetchJson(INTEGRATION_ACTIVATE_ENDPOINT, { method: "POST" });
    } catch (err) {
      logStatus(`nFusion activate failed: ${err.message}`, { level: "warn", ttlMs: 4000 });
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
    setMode,
    getMode,
    setMission,
    play,
    pause,
    stop,
    clear,
    reset,
    setSpeed,
    nextMission,
    setAgentState,
    forceCommand,
    addTarget,
    addRoi,
    clearTargets,
    resetIntegration,
    getIntegrationSettings,
    saveIntegrationSettings,
    ensureIntegrationReady,
    subscribe,
    getState: () => lastState,
  };
};
