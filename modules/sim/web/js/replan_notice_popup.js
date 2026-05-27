const PAYLOAD_ENDPOINT = "/api/integration/payload";
const POLL_INTERVAL_MS = 1500;
const AUTO_CLOSE_MS = 3000;

const STATUS_LABELS = {
  1: "재계획 수행 중",
  2: "재계획 완료",
};

let last0305Signature = null;
let last0001Signature = null;
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

const fetchPayload = async (msgId) => {
  try {
    const url = new URL(PAYLOAD_ENDPOINT, window.location.origin);
    url.searchParams.set("msgId", msgId);
    url.searchParams.set("type", "rx");
    const res = await fetch(url.toString(), { cache: "no-store" });
    if (!res.ok) {
      return null;
    }
    const data = await res.json();
    if (!data || !data.ok) {
      return null;
    }
    return data.payload;
  } catch {
    return null;
  }
};

const makeSignature = (payload) => {
  if (!payload || typeof payload !== "object") {
    return null;
  }
  const ts = payload.timestamp ?? payload.Timestamp ?? "";
  const src = payload.source ?? payload.Source ?? "";
  return `${ts}|${src}|${JSON.stringify(payload).length}`;
};

const handle0305 = (payload) => {
  const sig = makeSignature(payload);
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
  const body = reason || "-";

  open(icon, `[0305] ${statusText}`, body, variant);
};

const handle0001 = (payload) => {
  const sig = makeSignature(payload);
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
    const [payload0305, payload0001] = await Promise.all([
      fetchPayload("0305"),
      fetchPayload("0001"),
    ]);
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
