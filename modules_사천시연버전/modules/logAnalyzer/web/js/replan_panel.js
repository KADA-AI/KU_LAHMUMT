/**
 * Replan modal — shows replan details in a centered overlay.
 */

import { AGENT_COLORS, AGENT_LABELS } from "./palette.js";

const REASON_LABELS = {
  operator: "운용자 재계획",
  pathDeviation: "경로이탈",
  priorTarget: "선행표적",
  mandatory: "강제명령",
  collabBase: "협업기저",
  targetRediscovery: "표적재발견",
  coverageUpdate: "엄호갱신",
};

const LEVEL_LABELS = {
  1: "레벨 1 (단일기체)",
  2: "레벨 2 (복수기체)",
  3: "레벨 3 (전체)",
};

/**
 * @param {HTMLElement} modalEl - The #replan-modal element
 * @returns {{ show(replanData, scenarioData, opts?): void, hide(): void }}
 */
export const createReplanPanel = (modalEl) => {
  const titleEl = document.getElementById("replan-title");
  const bodyEl = document.getElementById("replan-body");
  const closeBtn = document.getElementById("replan-close");
  const backdrop = modalEl.querySelector(".replan-modal-backdrop");

  /** @type {Function|null} */
  let onPlanJump = null;

  const hide = () => {
    modalEl.setAttribute("aria-hidden", "true");
    onPlanJump = null;
  };

  closeBtn?.addEventListener("click", hide);
  backdrop?.addEventListener("click", hide);

  // Close on Escape
  document.addEventListener("keydown", (e) => {
    if (e.key === "Escape" && modalEl.getAttribute("aria-hidden") === "false") {
      hide();
    }
  });

  /**
   * @param {Object} replanData - The replan entry from timeline
   * @param {Object} scenarioData - Full scenario data
   * @param {Object} [opts]
   * @param {Function} [opts.onPlanJump] - Called with planID to navigate to that plan
   */
  const show = (replanData, scenarioData, opts = {}) => {
    onPlanJump = opts.onPlanJump || null;
    bodyEl.innerHTML = "";

    if (titleEl) {
      titleEl.textContent = "재계획 상세";
    }

    // Reason section
    const reasonSec = buildSection("재계획 사유");
    const reasonBadge = document.createElement("div");
    reasonBadge.className = "replan-reason-badge";
    reasonBadge.textContent = REASON_LABELS[replanData.reason] || replanData.reason || "알 수 없음";
    reasonSec.appendChild(reasonBadge);
    bodyEl.appendChild(reasonSec);

    // Basic info
    const infoSec = buildSection("기본 정보");
    if (replanData.level != null) {
      addField(infoSec, "재계획 레벨", LEVEL_LABELS[replanData.level] || `레벨 ${replanData.level}`);
    }
    if (replanData.timestamp) {
      addField(infoSec, "시각", formatTimestamp(replanData.timestamp));
    }
    if (replanData.replanRequestID) {
      addField(infoSec, "요청 ID", replanData.replanRequestID);
    }
    bodyEl.appendChild(infoSec);

    // Source → Result plan links
    if (replanData.sourcePlanID || replanData.resultPlanID) {
      const linkSec = buildSection("계획 연결");
      const linkRow = document.createElement("div");
      linkRow.style.cssText = "display:flex;align-items:center;gap:4px;font-size:12px";

      if (replanData.sourcePlanID) {
        const src = buildPlanLink(replanData.sourcePlanID);
        linkRow.appendChild(src);
      }

      const arrow = document.createElement("span");
      arrow.className = "replan-arrow";
      arrow.textContent = "\u2192";
      linkRow.appendChild(arrow);

      if (replanData.resultPlanID) {
        const res = buildPlanLink(replanData.resultPlanID);
        linkRow.appendChild(res);
      }

      linkSec.appendChild(linkRow);
      bodyEl.appendChild(linkSec);
    }

    // Replan detail fields
    const detail = replanData.replanDetail || replanData.detail || {};
    const detailKeys = Object.keys(detail);
    if (detailKeys.length > 0) {
      const detailSec = buildSection("상세 정보");
      for (const key of detailKeys) {
        const val = detail[key];
        const displayVal = typeof val === "object" ? JSON.stringify(val) : String(val);
        addField(detailSec, translateKey(key), displayVal);
      }
      bodyEl.appendChild(detailSec);
    }

    // Diff: which aircraft changed
    if (replanData.sourcePlanID && replanData.resultPlanID && scenarioData.missionPlans) {
      const srcPlan = findPlan(scenarioData, replanData.sourcePlanID);
      const resPlan = findPlan(scenarioData, replanData.resultPlanID);
      if (srcPlan && resPlan) {
        const diffSec = buildSection("변경된 항공기");
        const diffs = computeDiff(srcPlan, resPlan);
        for (const d of diffs) {
          const row = document.createElement("div");
          row.className = d.changed ? "replan-diff-row replan-diff-changed" : "replan-diff-row replan-diff-same";

          const dot = document.createElement("span");
          dot.className = "agent-dot";
          dot.style.background = AGENT_COLORS[d.aircraftId] || "#888";
          row.appendChild(dot);

          const name = document.createElement("span");
          name.className = "agent-name";
          name.style.cssText = "font-size:11px;min-width:40px";
          name.textContent = AGENT_LABELS[d.aircraftId] || `AC${d.aircraftId}`;
          row.appendChild(name);

          const status = document.createElement("span");
          status.style.cssText = "font-size:11px;color:var(--text-dim)";
          status.textContent = d.changed ? "IMP 변경됨" : "동일";
          row.appendChild(status);

          diffSec.appendChild(row);
        }
        bodyEl.appendChild(diffSec);
      }
    }

    modalEl.setAttribute("aria-hidden", "false");
  };

  return { show, hide };

  // ---- Internal ----

  function buildSection(title) {
    const sec = document.createElement("div");
    sec.className = "replan-section";
    const h = document.createElement("div");
    h.className = "replan-section-title";
    h.textContent = title;
    sec.appendChild(h);
    return sec;
  }

  function addField(sec, label, value) {
    const row = document.createElement("div");
    row.className = "replan-field";
    row.innerHTML = `<span class="replan-field-label">${escHtml(label)}</span>
      <span class="replan-field-value">${escHtml(String(value ?? "-"))}</span>`;
    sec.appendChild(row);
  }

  function buildPlanLink(planId) {
    const el = document.createElement("span");
    el.className = "replan-plan-link";
    el.textContent = `Plan ${planId}`;
    el.addEventListener("click", () => {
      if (onPlanJump) {
        onPlanJump(planId);
        hide();
      }
    });
    return el;
  }
};

function findPlan(scenarioData, planId) {
  // Search in timeline
  for (const entry of scenarioData.timeline || []) {
    if (entry.plan && String(entry.plan.missionPlanID) === String(planId)) {
      return entry.plan;
    }
  }
  // Search in missionPlans map
  if (scenarioData.missionPlans) {
    return scenarioData.missionPlans[planId] || null;
  }
  return null;
}

function computeDiff(srcPlan, resPlan) {
  const srcMap = {};
  for (const ac of srcPlan.aircraftList || []) {
    srcMap[ac.aircraftID] = ac.individualMissionPackageID;
  }

  const allIds = new Set([
    ...(srcPlan.aircraftList || []).map((a) => a.aircraftID),
    ...(resPlan.aircraftList || []).map((a) => a.aircraftID),
  ]);

  const result = [];
  for (const aid of [...allIds].sort()) {
    const resAc = (resPlan.aircraftList || []).find((a) => a.aircraftID === aid);
    const srcImpId = srcMap[aid];
    const resImpId = resAc?.individualMissionPackageID;
    result.push({
      aircraftId: aid,
      changed: srcImpId !== resImpId,
      srcImpId,
      resImpId,
    });
  }
  return result;
}

function formatTimestamp(ts) {
  try {
    const d = new Date(ts);
    return d.toLocaleString("ko-KR");
  } catch {
    return String(ts);
  }
}

function translateKey(key) {
  const map = {
    targetID: "표적 ID",
    watcherID: "감시기체 ID",
    coordinate: "좌표",
    latitude: "위도",
    longitude: "경도",
    altitude: "고도",
    aircraftID: "기체 ID",
    mandatoryType: "강제명령 유형",
    priorMissionID: "선행임무 ID",
    pathDeviationID: "경로이탈 ID",
  };
  return map[key] || key;
}

function escHtml(str) {
  return str.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
}
