const PAYLOAD_ENDPOINT = "/api/integration/payload";
const POLL_INTERVAL_MS = 1500;
const AUTO_CLOSE_MS = 3000;
const MAX_REPLAN_DURATION_SEC = 30 * 60;

const STATUS_LABELS = {
  1: "재계획 수행 중",
  2: "재계획 완료",
};

let last0305Signature = null;
let last0902Signature = null;
let last0001Signature = null;
let activeReplanStartedAtSec = null;
let activeReplanStartSource = null;
let pollTimer = null;
let closeTimer = null;

let modal = null;
let elIcon = null;
let elTitle = null;
let elBody = null;
let elTimerBar = null;

const open = (icon, title, body, variant) => {
  if (!modal) {
    return;
  }
  modal.className = "replan-notice-modal is-open";
  if (variant) {
    modal.classList.add(`replan-notice-modal--${variant}`);
  }
  elIcon.textContent = icon;
  elTitle.textContent = title;
  elBody.textContent = body;

  /* restart timer bar animation */
  if (elTimerBar) {
    elTimerBar.classList.remove("is-running");
    void elTimerBar.offsetHeight;
    elTimerBar.classList.add("is-running");
  }

  if (closeTimer) {
    clearTimeout(closeTimer);
  }
  closeTimer = setTimeout(close, AUTO_CLOSE_MS);
};

const close = () => {
  if (!modal) {
    return;
  }
  modal.classList.remove("is-open");
  if (closeTimer) {
    clearTimeout(closeTimer);
    closeTimer = null;
  }
};

const fetchPayload = async (msgId, payloadType = "rx") => {
  try {
    const url = new URL(PAYLOAD_ENDPOINT, window.location.origin);
    url.searchParams.set("msgId", msgId);
    url.searchParams.set("type", payloadType);
    const res = await fetch(url.toString(), { cache: "no-store" });
    if (!res.ok) {
      return null;
    }
    const data = await res.json();
    if (!data || !data.ok) {
      return null;
    }
    return data;
  } catch {
    return null;
  }
};

const recordPayloadObservation = async (body) => {
  try {
    await fetch("/api/integration/payload_observation", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      cache: "no-store",
      keepalive: true,
      body: JSON.stringify(body),
    });
  } catch {
    /* ignore */
  }
};

const payloadOf = (record) => record?.payload ?? null;

const observedAtSecOf = (record) => {
  const value = Number(record?.observedAtSec ?? record?.receivedAtSec ?? record?.sentAtSec);
  if (Number.isFinite(value) && value > 0) {
    return value;
  }
  return Date.now() / 1000;
};

const latestRecord = (...records) => {
  const valid = records.filter((record) => record && payloadOf(record));
  if (!valid.length) {
    return null;
  }
  valid.sort((a, b) => observedAtSecOf(b) - observedAtSecOf(a));
  return valid[0];
};

const formatDuration = (seconds) => {
  if (!Number.isFinite(seconds) || seconds < 0) {
    return null;
  }
  if (seconds < 60) {
    return `${seconds.toFixed(seconds < 10 ? 1 : 0)}초`;
  }
  const mins = Math.floor(seconds / 60);
  const secs = Math.round(seconds - mins * 60);
  return secs > 0 ? `${mins}분 ${secs}초` : `${mins}분`;
};

const makeSignature = (payload, observedAtSec = null) => {
  if (!payload || typeof payload !== "object") {
    return null;
  }
  const ts = payload.timestamp ?? payload.Timestamp ?? "";
  const src = payload.source ?? payload.Source ?? "";
  const status = payload.missionPlanningStatus ?? payload.MissionPlanningStatus ?? "";
  const missionPlanId = payload.missionPlanID ?? payload.MissionPlanID ?? "";
  const observed = Number.isFinite(observedAtSec) ? observedAtSec.toFixed(3) : "";
  return `${ts}|${src}|${status}|${missionPlanId}|${observed}|${JSON.stringify(payload).length}`;
};

const markReplanStarted = (observedAtSec, { force = false, source = null } = {}) => {
  if (!Number.isFinite(observedAtSec) || observedAtSec <= 0) {
    return;
  }
  const stale =
    activeReplanStartedAtSec === null ||
    observedAtSec < activeReplanStartedAtSec ||
    observedAtSec - activeReplanStartedAtSec > MAX_REPLAN_DURATION_SEC;
  if (force || stale) {
    activeReplanStartedAtSec = observedAtSec;
    activeReplanStartSource = source;
  }
};

const consumeReplanElapsed = (completedAtSec) => {
  if (!Number.isFinite(completedAtSec) || activeReplanStartedAtSec === null) {
    return null;
  }
  const elapsed = completedAtSec - activeReplanStartedAtSec;
  activeReplanStartedAtSec = null;
  activeReplanStartSource = null;
  if (elapsed < 0 || elapsed > MAX_REPLAN_DURATION_SEC) {
    return null;
  }
  return elapsed;
};

const handle0902 = (record) => {
  const payload = payloadOf(record);
  const observedAtSec = observedAtSecOf(record);
  const sig = makeSignature(payload, observedAtSec);
  if (!sig || sig === last0902Signature) {
    return;
  }
  last0902Signature = sig;
  markReplanStarted(observedAtSec, { force: true, source: "0902" });
};

const handle0305 = (record) => {
  const payload = payloadOf(record);
  const observedAtSec = observedAtSecOf(record);
  const sig = makeSignature(payload, observedAtSec);
  if (!sig || sig === last0305Signature) {
    return;
  }
  last0305Signature = sig;

  const status = Number(
    payload.missionPlanningStatus ?? payload.MissionPlanningStatus ?? 0,
  );
  const reason =
    payload.replanReason ?? payload.ReplanReason ?? payload.reason ?? "";
  const statusText = STATUS_LABELS[status] || `상태 ${status}`;

  const icon = status === 2 ? "\u2705" : "\u23F3";
  const variant = status === 2 ? "success" : "";
  if (status === 1) {
    const keep0902Start =
      activeReplanStartSource === "0902" &&
      activeReplanStartedAtSec !== null &&
      observedAtSec >= activeReplanStartedAtSec &&
      observedAtSec - activeReplanStartedAtSec <= 5;
    markReplanStarted(observedAtSec, { force: !keep0902Start, source: "0305" });
  }
  const startedAtSecForTiming = activeReplanStartedAtSec;
  const startSourceForTiming = activeReplanStartSource;
  const elapsed = status === 2 ? consumeReplanElapsed(observedAtSec) : null;
  const displayedAtSec = Date.now() / 1000;
  if (typeof window !== "undefined") {
    window.__replanNoticeLastTiming = {
      status,
      reason,
      observedAtSec,
      startedAtSec: startedAtSecForTiming,
      startSource: startSourceForTiming,
      elapsedSec: elapsed,
      displayedAtSec,
    };
  }
  recordPayloadObservation({
    msgId: "0305",
    type: "rx",
    observedAtSec,
    displayedAtSec,
    status,
    reason,
    payload,
  });
  const elapsedText = status === 2 ? formatDuration(elapsed) : null;
  const body = elapsedText
    ? `${reason || "-"} · 완료 수신까지 ${elapsedText}`
    : reason || "-";

  open(icon, `[0305] ${statusText}`, body, variant);
};

const handle0001 = (record) => {
  const payload = payloadOf(record);
  const observedAtSec = observedAtSecOf(record);
  const sig = makeSignature(payload, observedAtSec);
  if (!sig || sig === last0001Signature) {
    return;
  }
  last0001Signature = sig;

  const contents =
    payload.contents ?? payload.Contents ?? payload.content ?? "";
  const source = payload.source ?? payload.Source ?? "";
  if (!contents) {
    return;
  }

  const isError =
    contents.includes("\uC2E4\uD328") ||
    contents.toLowerCase().includes("fail") ||
    contents.toLowerCase().includes("error");

  const icon = isError ? "\u274C" : "\u26A0\uFE0F";
  const variant = isError ? "error" : "warn";
  const title = source ? `[0001] \uC54C\uB9BC - ${source}` : "[0001] \uC54C\uB9BC";

  open(icon, title, contents, variant);
};

const poll = async () => {
  try {
    const [payload0902Rx, payload0902Tx, payload0305, payload0001] = await Promise.all([
      fetchPayload("0902", "rx"),
      fetchPayload("0902", "tx"),
      fetchPayload("0305", "rx"),
      fetchPayload("0001", "rx"),
    ]);
    const payload0902 = latestRecord(payload0902Rx, payload0902Tx);
    if (payload0902) {
      handle0902(payload0902);
    }
    if (payload0305) {
      handle0305(payload0305);
    }
    if (payload0001) {
      handle0001(payload0001);
    }
  } catch {
    /* ignore */
  }
};

export const initReplanNoticePopup = () => {
  modal = document.getElementById("replan-notice-modal");
  if (!modal) {
    return;
  }
  elIcon = modal.querySelector("[data-notice-icon]");
  elTitle = modal.querySelector("[data-notice-title]");
  elBody = modal.querySelector("[data-notice-body]");
  elTimerBar = modal.querySelector("[data-notice-timer-bar]");

  modal.addEventListener("click", (e) => {
    if (e.target === modal) {
      close();
    }
  });

  if (pollTimer) {
    return;
  }
  pollTimer = setInterval(poll, POLL_INTERVAL_MS);
  poll();
};
