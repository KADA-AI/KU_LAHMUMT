import { logStatus } from "./status_log.js";

const PAYLOAD_ENDPOINT = "/api/integration/payload";
const SEND_CUSTOM_ENDPOINT = "/api/integration/send_custom";

const EPOCH_2000 = Date.UTC(2000, 0, 1, 0, 0, 0, 0);
const POLL_INTERVAL_MS = 1200;
const AUTO_CLOSE_MS = 5000;

const RECOMMEND_LABELS = {
  1: "다음 협업기저임무 추천",
  2: "현재 협업기저임무 재수행 추천",
  3: "모든 협업기저임무 완료",
};

const nowMs2000 = () => Date.now() - EPOCH_2000;

const parsePayload = (raw) => {
  if (!raw) {
    return null;
  }
  if (Array.isArray(raw)) {
    for (let i = raw.length - 1; i >= 0; i -= 1) {
      const entry = parsePayload(raw[i]);
      if (entry) {
        return entry;
      }
    }
    return null;
  }
  if (typeof raw === "object") {
    return raw;
  }
  if (typeof raw === "string") {
    try {
      return JSON.parse(raw);
    } catch (err) {
      const match = raw.match(/\{[\s\S]*\}/);
      if (match) {
        try {
          return JSON.parse(match[0]);
        } catch (err2) {
          return null;
        }
      }
    }
  }
  return null;
};

const pickValue = (obj, keys) => {
  if (!obj || typeof obj !== "object") {
    return undefined;
  }
  for (const key of keys) {
    if (Object.prototype.hasOwnProperty.call(obj, key)) {
      return obj[key];
    }
  }
  return undefined;
};

const toInt = (value, fallback = null) => {
  if (value === "" || value === null || value === undefined) {
    return fallback;
  }
  const parsed = Number(value);
  return Number.isFinite(parsed) ? Math.trunc(parsed) : fallback;
};

const formatTimestamp = (timestamp) => {
  const ms = toInt(timestamp, null);
  if (ms === null) {
    return "-";
  }
  const date = new Date(EPOCH_2000 + ms);
  if (Number.isNaN(date.getTime())) {
    return String(ms);
  }
  const pad = (val) => String(val).padStart(2, "0");
  return `${date.getFullYear()}-${pad(date.getMonth() + 1)}-${pad(date.getDate())} ${pad(
    date.getHours(),
  )}:${pad(date.getMinutes())}:${pad(date.getSeconds())}`;
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
  } catch (err) {
    return null;
  }
};

const sendCustom = async (msgId, body) => {
  try {
    const response = await fetch(SEND_CUSTOM_ENDPOINT, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ msgId, body }),
    });
    const data = await response.json();
    if (!data.ok) {
      throw new Error(data.error || "Send failed");
    }
    return true;
  } catch (err) {
    logStatus(`${msgId} 전송 실패: ${err.message}`, {
      level: "error",
      ttlMs: 6000,
    });
    return false;
  }
};

const cleanText = (value, fallback = "") => {
  const text = typeof value === "string" ? value.trim() : "";
  return text || fallback;
};

const triggerSimNextMission = () => {
  const sim = window.simClient;
  if (sim && typeof sim.nextMission === "function") {
    sim.nextMission();
  }
};

export const initMissionRecommendPopup = () => {
  const modal = document.getElementById("mission-recommend-modal");
  if (!modal) {
    return null;
  }

  const titleEl = modal.querySelector("[data-recommend-title]");
  const timeEl = modal.querySelector("[data-recommend-time]");
  const stateEl = modal.querySelector("[data-recommend-state]");
  const bodyEl = modal.querySelector("[data-recommend-body]");
  const btnNext = modal.querySelector("[data-recommend-next]");
  const btnRepeat = modal.querySelector("[data-recommend-repeat]");
  const btnOk = modal.querySelector("[data-recommend-ok]");
  const closeButtons = modal.querySelectorAll("[data-recommend-close]");

  let pollTimer = null;
  let pending = false;
  let lastKey = null;
  let autoCloseTimer = null;
  let lastPayload = null;
  const sourceInput = document.getElementById("scn-0803-source");

  const setOpen = (open) => {
    modal.classList.toggle("is-open", open);
    modal.setAttribute("aria-hidden", open ? "false" : "true");
  };

  const clearAutoClose = () => {
    if (autoCloseTimer) {
      window.clearTimeout(autoCloseTimer);
      autoCloseTimer = null;
    }
  };

  const setButtonState = (recommend) => {
    const recVal = toInt(recommend, null);
    const showOkOnly = recVal === 3;
    const setHidden = (el, hidden) => {
      if (!el) return;
      el.classList.toggle("is-hidden", hidden);
      el.disabled = hidden;
    };

    setHidden(btnNext, showOkOnly);
    setHidden(btnRepeat, showOkOnly);
    setHidden(btnOk, !showOkOnly);

    if (btnNext) {
      btnNext.classList.toggle("is-recommend", recVal === 1);
    }
    if (btnRepeat) {
      btnRepeat.classList.toggle("is-recommend", recVal === 2);
    }
  };

  const render = (payload) => {
    const recVal = toInt(pickValue(payload, ["systemRecommend", "SystemRecommend"]), null);
    const timestamp = pickValue(payload, ["timestamp", "Timestamp", "timeStamp", "TimeStamp"]);
    const label = RECOMMEND_LABELS[recVal] || `systemRecommend=${recVal ?? "-"}`;

    if (titleEl) {
      titleEl.textContent = "0503 협업기저임무 완료";
    }
    if (timeEl) {
      timeEl.textContent = `수신: ${formatTimestamp(timestamp)}`;
    }
    if (stateEl) {
      stateEl.textContent = label;
    }
    if (bodyEl) {
      if (recVal === 3) {
        bodyEl.textContent = "모든 협업기저임무 완료";
      } else if (recVal === 2) {
        bodyEl.textContent = "현재 협업임무 재수행?";
      } else {
        bodyEl.textContent = "다음 협업임무 수행?";
      }
    }

    setButtonState(recVal);
  };

  const openWithPayload = (payload) => {
    clearAutoClose();
    lastPayload = payload;
    render(payload);
    setOpen(true);

    const recVal = toInt(pickValue(payload, ["systemRecommend", "SystemRecommend"]), null);
    if (recVal === 3) {
      autoCloseTimer = window.setTimeout(() => {
        setOpen(false);
      }, AUTO_CLOSE_MS);
    }
  };

  const handleAction = async (execute) => {
    if (pending) {
      return;
    }
    pending = true;
    const body = {
      timestamp: nowMs2000(),
      source: cleanText(sourceInput?.value, "CSP"),
      execute: Number(execute),
    };
    const sent = await sendCustom("0803", body);
    pending = false;
    if (sent) {
      setOpen(false);
      logStatus(`0803 execute=${execute} 전송`, { level: "info", ttlMs: 3500 });
    }
  };

  const poll = async () => {
    if (pending) {
      return;
    }
    const payload = await fetchPayload("0503");
    const parsed = parsePayload(payload);
    if (!parsed) {
      return;
    }
    const recVal = toInt(pickValue(parsed, ["systemRecommend", "SystemRecommend"]), null);
    if (recVal === null) {
      return;
    }
    const ts = toInt(pickValue(parsed, ["timestamp", "Timestamp", "timeStamp", "TimeStamp"]), null);
    const key = `${ts ?? "?"}|${recVal}`;
    if (key === lastKey) {
      return;
    }
    lastKey = key;
    openWithPayload(parsed);
    logStatus(`0503 수신: ${RECOMMEND_LABELS[recVal] || recVal}`, { level: "info", ttlMs: 3000 });
  };

  if (btnNext) {
    btnNext.addEventListener("click", () => handleAction(1));
  }
  if (btnRepeat) {
    btnRepeat.addEventListener("click", () => handleAction(2));
  }
  if (btnOk) {
    btnOk.addEventListener("click", () => {
      setOpen(false);
    });
  }

  closeButtons.forEach((btn) => {
    btn.addEventListener("click", () => {
      setOpen(false);
    });
  });

  pollTimer = window.setInterval(poll, POLL_INTERVAL_MS);
  poll();

  return () => {
    if (pollTimer) {
      window.clearInterval(pollTimer);
      pollTimer = null;
    }
    clearAutoClose();
  };
};
