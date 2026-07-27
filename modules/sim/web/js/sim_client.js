import { logStatus } from "./status_log.js";
import { reportMissionValidation } from "./mission_validation_alert.js";

const SIM_STATE_ENDPOINT = "/api/sim/state";
const MISSION_ENDPOINT = "/api/sim/mission";
const PLAY_ENDPOINT = "/api/sim/play";
const PAUSE_ENDPOINT = "/api/sim/pause";
const STOP_ENDPOINT = "/api/sim/stop";
const CLEAR_ENDPOINT = "/api/sim/clear";
const RESET_ENDPOINT = "/api/sim/reset";
const SPEED_ENDPOINT = "/api/sim/speed";
const SHINIL_MISSION_PROFILE_ENDPOINT = "/api/sim/shinil_mission_profile";
const AUTO_TARGET_PLACEMENT_ENDPOINT = "/api/sim/auto_target_placement";
const NEXT_MISSION_ENDPOINT = "/api/sim/next_mission";
const SKIP_TO_MISSION_START_ENDPOINT = "/api/sim/skip_to_mission_start";
const AGENT_STATE_ENDPOINT = "/api/sim/agent_state";
const FORCE_COMMAND_ENDPOINT = "/api/sim/force_command";
const TARGET_ADD_ENDPOINT = "/api/sim/targets/add";
const ROI_ADD_ENDPOINT = "/api/sim/roi/add";
const TARGET_CLEAR_ENDPOINT = "/api/sim/targets/clear";
const REISSUE_INPUT_0201_ENDPOINT = "/api/sim/reissue_input_0201";
const TYPE1_NEW_TARGET_0201_ENDPOINT = "/api/sim/type1_new_target_0201";
const TYPE1_TARGET_ORDER_ENDPOINT = "/api/sim/type1_target_order";
const TYPE1_TARGET_ORDER_0201_ENDPOINT = "/api/sim/type1_target_order_0201";
const INTEGRATION_RESET_ENDPOINT = "/api/integration/reset";
const INTEGRATION_SETTINGS_ENDPOINT = "/api/integration/settings";
const INTEGRATION_ACTIVATE_ENDPOINT = "/api/integration/activate";
// The authoritative SIM/UI stream is 5 Hz. Visual modules interpolate these
// samples separately, so polling faster only rebuilt the same heavy map state.
const SIM_POLL_INTERVAL_MS = 200;
const MAX_ANIMATION_QUEUE_FRAMES = 30;
const MAX_HISTORY_FRAMES_PER_POLL = 1;

const fetchJson = async (url, options) => {
  const response = await fetch(url, options);
  const payload = await response.json();
  return payload;
};

export const initSimClient = () => {
  let pollTimer = null;
  let lastState = null;
  let lastStep = null;
  let lastRenderedStep = null;
  let frameQueue = [];
  let draining = false;
  let pollInFlight = false;
  let timelineGeneration = 0;
  let suspendRender = typeof document !== "undefined" ? document.hidden : false;
  let lastSpeedFactor = null;
  let speedRampTimer = null;
  let speedRampSeq = 0;
  let skipHistoryOnce = false;
  let catchUpMode = false;
  let forceCatchUp = false;
  let currentPollIntervalMs = SIM_POLL_INTERVAL_MS;
  let missionSignature = null;
  let lastFetchWarnAt = 0;
  const subscribers = new Set();
  const SPEED_RAMP_SEQUENCE = [10, 8, 6, 4, 2, 1];
  const SPEED_RAMP_INTERVAL_MS = 200;
  const parseStep = (value) => value === null || value === undefined ? Number.NaN : Number(value);

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
    const currentSignature = state?.missionSignature || null;
    const hasStatePlanId = Boolean(
      state && Object.prototype.hasOwnProperty.call(state, "missionPlanID"),
    );
    const remainingAreaPlanId = hasStatePlanId
      ? state.missionPlanID
      : mission?.missionPlanID;
    if (
      (hasStatePlanId || remainingAreaPlanId !== undefined) &&
      typeof window.setRemainingAreaMissionPlanID === "function"
    ) {
      window.setRemainingAreaMissionPlanID(remainingAreaPlanId);
    }
    if (!mission) {
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

  const resetQueueToState = (state) => {
    frameQueue = [];
    draining = false;
    skipHistoryOnce = true;
    const nextStep = parseStep(state?.step);
    if (Number.isFinite(nextStep)) {
      lastStep = Math.max(0, nextStep);
    }
    if (!suspendRender && state) {
      updateMarkers(state);
    }
  };

  const clearClientTimeline = ({ clearMarkers = false } = {}) => {
    timelineGeneration += 1;
    frameQueue = [];
    draining = false;
    catchUpMode = false;
    forceCatchUp = false;
    skipHistoryOnce = true;
    lastStep = null;
    lastRenderedStep = null;
    if (clearMarkers && typeof window.missionVehicleLoader === "function") {
      window.missionVehicleLoader({ ok: true, vehicles: {}, step: null });
    }
    if (clearMarkers && typeof window.missionLosLoader === "function") {
      window.missionLosLoader({ ok: true, losLinks: [], step: null });
    }
    if (clearMarkers && typeof window.missionCommunicationLosLoader === "function") {
      window.missionCommunicationLosLoader({
        ok: true,
        communicationLinks: [],
        step: null,
      });
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
      return false;
    }
    const nextStep = parseStep(state?.step);
    if (
      Number.isFinite(nextStep) &&
      Number.isFinite(lastRenderedStep) &&
      nextStep < lastRenderedStep
    ) {
      return false;
    }
    if (Number.isFinite(nextStep)) {
      lastRenderedStep = nextStep;
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
    // History frames intentionally omit LOS links because native DEM ray
    // sampling is performed only for the latest snapshot. Preserve each last
    // result until its authoritative field arrives in a current snapshot.
    if (
      Object.prototype.hasOwnProperty.call(state, "losLinks") &&
      typeof window.missionLosLoader === "function"
    ) {
      window.missionLosLoader({
        ok: true,
        losLinks: Array.isArray(state.losLinks) ? state.losLinks : [],
        step: state.step,
      });
    }
    if (
      Object.prototype.hasOwnProperty.call(state, "communicationLinks") &&
      typeof window.missionCommunicationLosLoader === "function"
    ) {
      window.missionCommunicationLosLoader({
        ok: true,
        communicationLinks: Array.isArray(state.communicationLinks)
          ? state.communicationLinks
          : [],
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
    return true;
  };

  const downsampleHistory = (history, speedFactor) => {
    if (!Array.isArray(history) || history.length <= 1) {
      return history || [];
    }
    // The latest authoritative frame is sufficient at the 5 Hz UI cadence.
    // Keeping older frames here only makes the map replay stale work after a
    // slow response or a high-speed simulation burst.
    void speedFactor;
    const maxFrames = Math.max(1, Math.floor(MAX_HISTORY_FRAMES_PER_POLL));
    if (history.length <= maxFrames) {
      return history;
    }
    if (maxFrames === 1) {
      return [history[history.length - 1]];
    }
    const step = Math.max(2, Math.ceil(history.length / maxFrames));
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
    if (frameQueue.length > MAX_ANIMATION_QUEUE_FRAMES) {
      frameQueue.splice(0, frameQueue.length - MAX_ANIMATION_QUEUE_FRAMES);
    }
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
    if (pollInFlight) {
      return;
    }
    pollInFlight = true;
    const pollGeneration = timelineGeneration;
    try {
      const query = Number.isFinite(lastStep) ? `?since=${lastStep}` : "";
      const state = await fetchJson(`${SIM_STATE_ENDPOINT}${query}`, { method: "GET" });
      if (pollGeneration !== timelineGeneration) {
        return;
      }
      const responseLatest = state?.latest || state;
      const responseStep = parseStep(responseLatest?.step);
      if (
        Number.isFinite(responseStep) &&
        Number.isFinite(lastStep) &&
        responseStep < lastStep
      ) {
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
        const latestOnly = state.history.length > 120 || frameQueue.length > MAX_ANIMATION_QUEUE_FRAMES;
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
        let skipCurrentHistory = false;
        if (Number.isFinite(speedValue)) {
          if (lastSpeedFactor !== null && speedValue !== lastSpeedFactor) {
            if (speedValue < lastSpeedFactor) {
              shouldCatchUp = true;
            } else {
              resetQueueToState(state.latest || state);
              skipCurrentHistory = true;
            }
          }
          lastSpeedFactor = speedValue;
        }
        if (forceCatchUp) {
          shouldCatchUp = true;
        }
        if (state.history.length && !skipCurrentHistory) {
          if (!suspendRender) {
            const historySpeed = Number.isFinite(speedValue) ? speedValue : lastSpeedFactor;
            const frames = state.history.slice();
            const latestStep = Number(state.latest?.step);
            const lastHistoryStep = Number(frames[frames.length - 1]?.step);
            if (state.latest && (!Number.isFinite(latestStep) || latestStep !== lastHistoryStep)) {
              frames.push(state.latest);
            }
            const sampled = downsampleHistory(frames, historySpeed);
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
            lastStep = Number.isFinite(lastStep)
              ? Math.max(lastStep, state.latest.step)
              : state.latest.step;
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
            resetQueueToState(state);
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
        logStatus("SIM state fetch failed", {
          level: "warn",
          ttlMs: 2500,
        });
        lastFetchWarnAt = now;
      }
    } finally {
      pollInFlight = false;
    }
  };

  const startPolling = (intervalMs = currentPollIntervalMs) => {
    if (
      pollTimer ||
      suspendRender ||
      (typeof document !== "undefined" && document.hidden)
    ) {
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
      if (!(payload?.preserveState || payload?.preserve_state || payload?.keepState || payload?.keep_state)) {
        clearClientTimeline({ clearMarkers: true });
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
      const result = await fetchJson(STOP_ENDPOINT, { method: "POST" });
      if (result?.ok) {
        clearClientTimeline({ clearMarkers: true });
      }
      return result;
    } catch (err) {
      logStatus(`SIM stop failed: ${err.message}`, { level: "error", ttlMs: 5000 });
      return { ok: false, error: err.message };
    }
  };

  const reset = async () => {
    try {
      const result = await fetchJson(RESET_ENDPOINT, { method: "POST" });
      if (result?.ok) {
        clearClientTimeline({ clearMarkers: true });
      }
      return result;
    } catch (err) {
      logStatus(`SIM reset failed: ${err.message}`, { level: "error", ttlMs: 5000 });
      return { ok: false, error: err.message };
    }
  };

  const clear = async () => {
    try {
      const result = await fetchJson(CLEAR_ENDPOINT, { method: "POST" });
      if (result?.ok) {
        clearClientTimeline({ clearMarkers: true });
      }
      return result;
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
    if (Number.isFinite(current) && numericSpeed > current) {
      // Do not keep replaying frames sampled for the previous, slower speed.
      // The next poll paints the newest authoritative state directly.
      frameQueue = [];
      draining = false;
      catchUpMode = false;
      forceCatchUp = false;
      skipHistoryOnce = true;
      timelineGeneration += 1;
    }
    cancelRamp();
    return applySpeed(numericSpeed);
  };

  const setShinilMissionProfile = async (enabled) => {
    try {
      const result = await fetchJson(SHINIL_MISSION_PROFILE_ENDPOINT, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ enabled: Boolean(enabled) }),
      });
      if (!result.ok) {
        throw new Error(result.error || "Shinil mission profile update failed");
      }
      return result;
    } catch (err) {
      logStatus(`신일 임무 특성 설정 실패: ${err.message}`, {
        level: "error",
        ttlMs: 5000,
      });
      return { ok: false, error: err.message };
    }
  };

  const setAutoTargetPlacement = async (enabled, applyCurrent = true) => {
    try {
      const result = await fetchJson(AUTO_TARGET_PLACEMENT_ENDPOINT, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          enabled: Boolean(enabled),
          applyCurrent: Boolean(applyCurrent),
        }),
      });
      if (!result.ok) {
        throw new Error(result.error || "auto target placement update failed");
      }
      return result;
    } catch (err) {
      logStatus(`자동 적 배치 설정 실패: ${err.message}`, { level: "error", ttlMs: 5000 });
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

  const skipToMissionStart = async (payload) => {
    try {
      const result = await fetchJson(SKIP_TO_MISSION_START_ENDPOINT, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload || {}),
      });
      if (!result.ok) {
        throw new Error(result.error || "mission start skip failed");
      }
      clearClientTimeline();
      const count = Number(result.count) || 0;
      logStatus(`SIM mission start skip: ${count} aircraft`, {
        level: "success",
        ttlMs: 3500,
      });
      return result;
    } catch (err) {
      logStatus(`SIM mission start skip failed: ${err.message}`, {
        level: "error",
        ttlMs: 5000,
      });
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

  const reissueInput0201 = async (payload) => {
    try {
      const result = await fetchJson(REISSUE_INPUT_0201_ENDPOINT, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload || {}),
      });
      if (!result.ok) {
        throw new Error(result.error || "0201 reissue failed");
      }
      const packageId = result.newPackageID ?? result.inputMissionPackageID ?? "";
      logStatus(`0201 재입력 전송 완료${packageId ? ` (#${packageId})` : ""}`, {
        level: "success",
        ttlMs: 4000,
      });
      return result;
    } catch (err) {
      logStatus(`0201 재입력 전송 실패: ${err.message}`, { level: "error", ttlMs: 6000 });
      return { ok: false, error: err.message };
    }
  };

  const sendType1NewTarget0201 = async (payload) => {
    try {
      const result = await fetchJson(TYPE1_NEW_TARGET_0201_ENDPOINT, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload || {}),
      });
      if (!result.ok) {
        throw new Error(result.error || "Type-1 new target 0201 failed");
      }
      const packageId = result.newPackageID ?? result.inputMissionPackageID ?? "";
      logStatus(`신규 목표지역 0201 전송 완료${packageId ? ` (#${packageId})` : ""}`, {
        level: "success",
        ttlMs: 4500,
      });
      return result;
    } catch (err) {
      logStatus(`신규 목표지역 0201 전송 실패: ${err.message}`, {
        level: "error",
        ttlMs: 6500,
      });
      return { ok: false, error: err.message };
    }
  };

  const getType1TargetOrder = async () => {
    try {
      const result = await fetchJson(TYPE1_TARGET_ORDER_ENDPOINT, { method: "GET" });
      if (!result.ok) {
        throw new Error(result.error || "Type-1 target order load failed");
      }
      return result;
    } catch (err) {
      logStatus(`목표지역 순서 정보 불러오기 실패: ${err.message}`, {
        level: "error",
        ttlMs: 6500,
      });
      return { ok: false, error: err.message };
    }
  };

  const sendType1TargetOrder0201 = async (payload) => {
    try {
      const result = await fetchJson(TYPE1_TARGET_ORDER_0201_ENDPOINT, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload || {}),
      });
      if (!result.ok) {
        throw new Error(result.error || "Type-1 target order 0201 failed");
      }
      const packageId = result.newPackageID ?? result.inputMissionPackageID ?? "";
      logStatus(`목표지역 순서 변경 0201 전송 완료${packageId ? ` (#${packageId})` : ""}`, {
        level: "success",
        ttlMs: 4500,
      });
      return result;
    } catch (err) {
      logStatus(`목표지역 순서 변경 0201 전송 실패: ${err.message}`, {
        level: "error",
        ttlMs: 6500,
      });
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
    setMission,
    play,
    pause,
    stop,
    clear,
    reset,
    setSpeed,
    setShinilMissionProfile,
    setAutoTargetPlacement,
    nextMission,
    skipToMissionStart,
    setAgentState,
    forceCommand,
    addTarget,
    addRoi,
    clearTargets,
    reissueInput0201,
    sendType1NewTarget0201,
    getType1TargetOrder,
    sendType1TargetOrder0201,
    resetIntegration,
    getIntegrationSettings,
    saveIntegrationSettings,
    ensureIntegrationReady,
    subscribe,
    getState: () => lastState,
  };
};
