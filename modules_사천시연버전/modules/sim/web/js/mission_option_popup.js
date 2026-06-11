import { logStatus } from "./status_log.js";
import { reportMissionValidation } from "./mission_validation_alert.js";

const PAYLOAD_ENDPOINT = "/api/integration/payload";
const SEND_CUSTOM_ENDPOINT = "/api/integration/send_custom";
const PLAN_LOAD_ENDPOINT = "/api/mission/plan_load";

const EPOCH_2000 = Date.UTC(2000, 0, 1, 0, 0, 0, 0);
const POLL_INTERVAL_MS = 1200;

const OPTION_NAME_MAP = {
  2: "공격 특화",
  3: "공격 배제",
  4: "정찰 특화",
  5: "최소 시간",
  6: "정찰/시간 균형",
};

const escapeHtml = (value) =>
  String(value ?? "")
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;")
    .replace(/'/g, "&#039;");

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

const toBool = (value, fallback = false) => {
  if (typeof value === "boolean") {
    return value;
  }
  if (value === null || value === undefined) {
    return fallback;
  }
  if (typeof value === "number") {
    return Boolean(value);
  }
  if (typeof value === "string") {
    const lower = value.trim().toLowerCase();
    if (["1", "true", "yes", "y", "on"].includes(lower)) return true;
    if (["0", "false", "no", "n", "off"].includes(lower)) return false;
  }
  return fallback;
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

const metricLabel = (key) => {
  if (key === "survivalRate") return "생존율";
  if (key === "timeContraction") return "소요 시간";
  if (key === "recogEffectiveness") return "촬영 효과";
  if (key === "fuelWarning") return "연료";
  return key;
};

const metricText = (key, value) => {
  if (key === "timeContraction") {
    if (value > 0) return "단축";
    if (value < 0) return "지연";
    return "유지";
  }
  if (key === "recogEffectiveness") {
    if (value > 0) return "향상";
    if (value < 0) return "저하";
    return "유지";
  }
  if (key === "fuelWarning") {
    if (value < 0) return "경고";
    if (value > 0) return "여유";
    return "정상";
  }
  if (value > 0) return "증가";
  if (value < 0) return "감소";
  return "유지";
};

const metricClass = (value) => {
  if (value > 0) return "metric-up";
  if (value < 0) return "metric-down";
  return "metric-flat";
};

const normalizeOption = (option) => {
  if (!option || typeof option !== "object") {
    return null;
  }
  const optionId = toInt(pickValue(option, ["optionID", "OptionID", "optionId"]), null);
  const missionPlanId = toInt(
    pickValue(option, ["missionPlanID", "MissionPlanID", "missionPlanId"]),
    null,
  );
  const optionName = toInt(pickValue(option, ["optionName", "OptionName", "option_name"]), null);
  const recommend = toBool(pickValue(option, ["recommend", "Recommend", "recommended"]), false);
  const survivalRate = toInt(pickValue(option, ["survivalRate", "SurvivalRate"]), 0) || 0;
  const timeContraction = toInt(pickValue(option, ["timeContraction", "TimeContraction"]), 0) || 0;
  const recogEffectiveness =
    toInt(pickValue(option, ["recogEffectiveness", "RecogEffectiveness"]), 0) || 0;
  const fuelWarning = toInt(pickValue(option, ["fuelWarning", "FuelWarning"]), 0) || 0;
  const distance = toInt(pickValue(option, ["distance", "Distance"]), null);
  const target = toInt(pickValue(option, ["target", "Target"]), null);
  return {
    optionId,
    missionPlanId,
    optionName,
    optionNameLabel: OPTION_NAME_MAP[optionName] || null,
    recommend,
    survivalRate,
    timeContraction,
    recogEffectiveness,
    fuelWarning,
    distance,
    target,
  };
};

const buildOptionKey = (payload, options) => {
  if (!payload) {
    return null;
  }
  const ts = toInt(
    pickValue(payload, ["timestamp", "Timestamp", "timeStamp", "TimeStamp"]),
    null,
  );
  const ids = (options || [])
    .map((opt) => `${opt.optionId ?? "?"}:${opt.missionPlanId ?? "?"}`)
    .join(",");
  if (ts === null && !ids) {
    return null;
  }
  return `${ts ?? "?"}|${ids}`;
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

const loadMissionPlan = async (missionPlanId, reason, preserveState = false) => {
  const id = toInt(missionPlanId, null);
  if (id === null) {
    logStatus("MissionPlan ID가 없습니다.", { level: "warn", ttlMs: 3500 });
    return false;
  }
  try {
    const response = await fetch(PLAN_LOAD_ENDPOINT, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ missionPlanID: id, preserveState: Boolean(preserveState) }),
    });
    const data = await response.json();
    if (!data.ok) {
      throw new Error(data.error || "Mission load failed");
    }
    reportMissionValidation(data.validation, `Mission ${id}`);
    if (typeof window.missionPathLoader === "function") {
      window.missionPathLoader(data);
    }
    if (typeof window.setMissionPlanReady === "function") {
      window.setMissionPlanReady(true);
    }
    logStatus(`Mission ${id} 적용 (${reason || "update"})`, {
      level: "success",
      ttlMs: 4000,
    });
    return true;
  } catch (err) {
    logStatus(`Mission ${id} 적용 실패: ${err.message}`, {
      level: "error",
      ttlMs: 6000,
    });
    return false;
  }
};

export const initMissionOptionPopup = () => {
  const modal = document.getElementById("mission-option-modal");
  if (!modal) {
    return null;
  }
  const listEl = modal.querySelector("[data-option-list]");
  const titleEl = modal.querySelector("[data-option-title]");
  const timeEl = modal.querySelector("[data-option-time]");
  const autoEl = modal.querySelector("[data-option-auto]");
  const closeButtons = modal.querySelectorAll("[data-option-close]");
  const keepButton = modal.querySelector("[data-option-keep]");

  let last0701Key = null;
  let last0702Key = null;
  let last0903Key = null;
  let pollTimer = null;
  let pending = false;
  let lastPayload = null;
  let autoApproveTimer = null;

  const setOpen = (open) => {
    modal.classList.toggle("is-open", open);
    modal.setAttribute("aria-hidden", open ? "false" : "true");
  };

  const renderOptions = (payload, options) => {
    if (!listEl) {
      return;
    }
    const timestamp = pickValue(payload, ["timestamp", "Timestamp", "timeStamp", "TimeStamp"]);
    const autoExecution = toBool(
      pickValue(payload, ["autoExecution", "AutoExecution", "auto_exec"]),
      false,
    );
    if (titleEl) {
      titleEl.textContent = "0701 임무 선택";
    }
    if (timeEl) {
      timeEl.textContent = `수신: ${formatTimestamp(timestamp)}`;
    }
    if (autoEl) {
      autoEl.textContent = autoExecution ? "자동 실행: ON" : "자동 실행: OFF";
    }

    if (!options.length) {
      listEl.innerHTML = `<div class="mission-option-empty">선택 가능한 옵션이 없습니다.</div>`;
      return;
    }

    listEl.innerHTML = options
      .map((opt) => {
        const title = opt.optionNameLabel || (opt.optionName ? `옵션 ${opt.optionName}` : "옵션");
        const badges = opt.recommend
          ? `<span class="option-badge option-badge-recommend">추천</span>`
          : "";
        const missionPlanText =
          opt.missionPlanId !== null && opt.missionPlanId !== undefined
            ? `MissionPlan ${opt.missionPlanId}`
            : "MissionPlan ?";
        const metaParts = [];
        if (opt.distance !== null) {
          metaParts.push(`거리 ${opt.distance}m`);
        }
        if (opt.target !== null) {
          metaParts.push(`표적 ${opt.target}`);
        }
        const meta = metaParts.length ? metaParts.join(" · ") : "";
        const metrics = [
          { key: "survivalRate", value: opt.survivalRate },
          { key: "timeContraction", value: opt.timeContraction },
          { key: "recogEffectiveness", value: opt.recogEffectiveness },
          { key: "fuelWarning", value: opt.fuelWarning },
        ]
          .map((item) => {
            const value = Number(item.value) || 0;
            return `
              <div class="option-metric ${metricClass(value)}">
                <span class="option-metric-label">${escapeHtml(metricLabel(item.key))}</span>
                <span class="option-metric-value">${escapeHtml(metricText(item.key, value))}</span>
              </div>
            `;
          })
          .join("");
        const disabled = opt.missionPlanId === null ? "is-disabled" : "";
        return `
          <button
            type="button"
            class="mission-option-item ${disabled}"
            data-option-id="${escapeHtml(opt.optionId ?? "")}"
            data-mission-plan-id="${escapeHtml(opt.missionPlanId ?? "")}"
          >
            <div class="option-header">
              <div class="option-id">Option ${escapeHtml(opt.optionId ?? "?")}</div>
              <div class="option-badges">${badges}</div>
            </div>
            <div class="option-title">${escapeHtml(title)}</div>
            <div class="option-plan">${escapeHtml(missionPlanText)}</div>
            <div class="option-metrics">${metrics}</div>
            ${meta ? `<div class="option-meta">${escapeHtml(meta)}</div>` : ""}
          </button>
        `;
      })
      .join("");
  };

  const handleOptionClick = async (event) => {
    const button = event.target.closest(".mission-option-item");
    if (!button || pending || button.classList.contains("is-disabled")) {
      return;
    }
    const missionPlanId = toInt(button.dataset.missionPlanId, null);
    if (missionPlanId === null) {
      logStatus("MissionPlan ID가 없습니다.", { level: "warn", ttlMs: 3000 });
      return;
    }
    pending = true;
    const body = {
      timestamp: nowMs2000(),
      source: "SIM_UI",
      ignore: 2,
      missionPlanID: missionPlanId,
    };
    const sent = await sendCustom("0702", body);
    if (sent) {
      await loadMissionPlan(missionPlanId, "0702", true);
    }
    pending = false;
    setOpen(false);
  };

  const firstSelectableOption = (options) =>
    (options || []).find((opt) => opt && opt.missionPlanId !== null && opt.missionPlanId !== undefined);

  const isAutoApproveFirstOptionEnabled = () => {
    const toggle = document.getElementById("auto-option-approve-toggle");
    if (toggle) {
      return toggle.getAttribute("aria-pressed") === "true";
    }
    return Boolean(window.simAutoApproveFirstOption);
  };

  const autoApproveFirstOption = (options) => {
    if (!isAutoApproveFirstOptionEnabled()) {
      return;
    }
    const first = firstSelectableOption(options);
    if (!first) {
      logStatus("0701 auto option: selectable option missing.", { level: "warn", ttlMs: 3500 });
      return;
    }
    if (autoApproveTimer) {
      window.clearTimeout(autoApproveTimer);
      autoApproveTimer = null;
    }
    autoApproveTimer = window.setTimeout(async () => {
      if (pending || !isAutoApproveFirstOptionEnabled()) {
        return;
      }
      pending = true;
      const missionPlanId = first.missionPlanId;
      logStatus(`0701 auto option: Option ${first.optionId ?? 1} -> Mission ${missionPlanId}`, {
        level: "info",
        ttlMs: 3500,
      });
      const body = {
        timestamp: nowMs2000(),
        source: "SIM_UI",
        ignore: 2,
        missionPlanID: missionPlanId,
      };
      const sent = await sendCustom("0702", body);
      if (sent) {
        await loadMissionPlan(missionPlanId, "0702 auto", true);
      }
      pending = false;
      setOpen(false);
    }, 150);
  };

  const handle0701 = (payload) => {
    const parsed = parsePayload(payload);
    if (!parsed) {
      return;
    }
    const rawList =
      parsed.optionList ||
      parsed.OptionList ||
      parsed.option_list ||
      parsed.optionInfoList ||
      [];
    if (!Array.isArray(rawList) || rawList.length === 0) {
      return;
    }
    const options = rawList.map(normalizeOption).filter(Boolean);
    if (!options.length) {
      return;
    }
    const key = buildOptionKey(parsed, options);
    if (key && key === last0701Key) {
      return;
    }
    last0701Key = key;
    lastPayload = parsed;
    renderOptions(parsed, options);
    setOpen(true);
    logStatus("0701 임무 옵션 수신", { level: "info", ttlMs: 3500 });
    autoApproveFirstOption(options);
  };

  const handle0903 = async (payload) => {
    const parsed = parsePayload(payload);
    if (!parsed) {
      return;
    }
    const missionPlanId = toInt(
      pickValue(parsed, ["missionPlanID", "MissionPlanID", "missionPlanId", "mission_plan_id"]),
      null,
    );
    if (missionPlanId === null) {
      return;
    }
    const ts = toInt(
      pickValue(parsed, ["timestamp", "Timestamp", "timeStamp", "TimeStamp"]),
      null,
    );
    const key = `${ts ?? "?"}|${missionPlanId}`;
    if (key === last0903Key) {
      return;
    }
    last0903Key = key;
    setOpen(false);
    await loadMissionPlan(missionPlanId, "0903", true);
  };

  const handle0702 = async (payload) => {
    const parsed = parsePayload(payload);
    if (!parsed) {
      return;
    }
    const ignore = toInt(pickValue(parsed, ["ignore", "Ignore"]), null);
    if (ignore !== 2) {
      return;
    }
    const missionPlanId = toInt(
      pickValue(parsed, ["missionPlanID", "MissionPlanID", "missionPlanId", "mission_plan_id"]),
      null,
    );
    if (missionPlanId === null) {
      return;
    }
    const ts = toInt(
      pickValue(parsed, ["timestamp", "Timestamp", "timeStamp", "TimeStamp"]),
      null,
    );
    const key = `${ts ?? "?"}|${missionPlanId}`;
    if (key === last0702Key) {
      return;
    }
    last0702Key = key;
    setOpen(false);
    await loadMissionPlan(missionPlanId, "0702 RX", true);
  };

  const poll = async () => {
    if (pending) {
      return;
    }
    const [payload0701, payload0702, payload0903] = await Promise.all([
      fetchPayload("0701"),
      fetchPayload("0702"),
      fetchPayload("0903"),
    ]);
    if (payload0701) {
      handle0701(payload0701);
    }
    if (payload0702) {
      handle0702(payload0702);
    }
    if (payload0903) {
      handle0903(payload0903);
    }
  };

  if (listEl) {
    listEl.addEventListener("click", handleOptionClick);
  }
  closeButtons.forEach((btn) => {
    btn.addEventListener("click", () => {
      setOpen(false);
    });
  });
  if (keepButton) {
    keepButton.addEventListener("click", () => {
      setOpen(false);
    });
  }

  pollTimer = window.setInterval(poll, POLL_INTERVAL_MS);
  poll();

  return () => {
    if (pollTimer) {
      window.clearInterval(pollTimer);
      pollTimer = null;
    }
    if (autoApproveTimer) {
      window.clearTimeout(autoApproveTimer);
      autoApproveTimer = null;
    }
  };
};
