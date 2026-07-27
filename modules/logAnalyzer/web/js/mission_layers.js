/** Mission-oriented map controls, load audit, and live waypoint status. */

import { AGENT_COLORS, AGENT_LABELS } from "./palette.js";

const LAYERS = [
  ["paths", "계획 경로·WP"],
  ["allocations", "할당 영역·회랑"],
  ["tracks", "실제 항적"],
  ["footprints", "센서 Footprint 누적"],
  ["inputAreas", "입력 임무 영역"],
  ["reference", "비행·금지 영역"],
  ["targets", "탐지 표적"],
];

const DEFAULT_VISIBILITY = {
  paths: true,
  allocations: true,
  tracks: true,
  footprints: true,
  inputAreas: true,
  reference: true,
  targets: true,
};

const MISSION_TYPE_LABELS = {
  0: "없음",
  1: "표적추적",
  2: "표적공격",
  3: "지역수색",
  4: "지역감시",
  5: "지점정찰",
  6: "회랑정찰",
  7: "이동",
  8: "엄호",
  9: "은폐엄호",
};

export const createMissionLayerPanel = (container, callbacks = {}) => {
  const currentMissionBar = document.getElementById("current-mission-bar");
  let scenario = null;
  let plan = null;
  let missionEntries = [];
  let focusedKey = null;
  let playbackSnapshot = null;
  let autoFollow = true;
  const visibility = { ...DEFAULT_VISIBILITY };

  const render = () => {
    if (!container) return;
    if (!scenario) {
      container.innerHTML = "";
      container.setAttribute("aria-hidden", "true");
      renderCurrentMissionBar();
      return;
    }
    container.innerHTML = `
      <div class="mlp-head">
        <div>
          <div class="mlp-title">MISSION VIEW</div>
          <div class="mlp-subtitle">${escapeHtml(plan ? `Plan ${plan.missionPlanID} · Input ${plan.inputMissionPackageID ?? "-"}` : "Plan 없음")}</div>
        </div>
        ${auditBadge(scenario.loadAudit)}
      </div>
      ${auditHtml(scenario)}
      <div class="mlp-section">
        <div class="mlp-section-title">레이어 분리</div>
        <div class="mlp-layer-grid">
          ${LAYERS.map(([key, label]) => `
            <label class="mlp-layer-toggle ${visibility[key] ? "is-active" : ""}">
              <input type="checkbox" data-layer="${key}" ${visibility[key] ? "checked" : ""} />
              <span>${escapeHtml(label)}</span>
            </label>`).join("")}
        </div>
        <label class="mlp-follow-toggle">
          <input type="checkbox" data-auto-follow ${autoFollow ? "checked" : ""} />
          <span>재생 시 적용 Plan 자동 전환</span>
        </label>
      </div>
      <div class="mlp-section">
        <div class="mlp-section-head">
          <div class="mlp-section-title">임무별 격리</div>
          <button type="button" class="mlp-all-btn ${focusedKey == null ? "is-active" : ""}" data-mission-all>전체 임무</button>
        </div>
        <div class="mlp-mission-list">
          ${missionEntries.length ? missionEntries.map((entry) => missionRowHtml(entry, focusedKey)).join("") : '<div class="mlp-empty">선택 Plan에 임무 정보가 없습니다.</div>'}
        </div>
      </div>
      <div class="mlp-section mlp-progress-section">
        <div class="mlp-section-title">0401 진행 WP</div>
        <div class="mlp-progress-list">${progressHtml(playbackSnapshot, focusedEntry())}</div>
      </div>`;

    container.querySelectorAll("[data-layer]").forEach((input) => {
      input.addEventListener("change", () => {
        const key = input.getAttribute("data-layer");
        visibility[key] = !!input.checked;
        input.closest(".mlp-layer-toggle")?.classList.toggle("is-active", visibility[key]);
        callbacks.onLayerToggle?.(key, visibility[key]);
      });
    });
    container.querySelector("[data-auto-follow]")?.addEventListener("change", (event) => {
      autoFollow = !!event.currentTarget.checked;
      callbacks.onAutoFollowToggle?.(autoFollow);
    });
    container.querySelector("[data-mission-all]")?.addEventListener("click", () => selectMission(null));
    container.querySelectorAll("[data-mission-key]").forEach((button) => {
      button.addEventListener("click", () => {
        const key = button.getAttribute("data-mission-key");
        selectMission(focusedKey === key ? null : key);
      });
      button.querySelector("[data-fit-mission]")?.addEventListener("click", (event) => {
        event.stopPropagation();
        const entry = missionEntries.find((item) => item.key === button.getAttribute("data-mission-key"));
        if (entry) callbacks.onFitMission?.(entry);
      });
    });
    updateMissionProgressClasses(container, missionEntries, playbackSnapshot);
    renderCurrentMissionBar();
    container.setAttribute("aria-hidden", "false");
  };

  const renderCurrentMissionBar = () => {
    if (!currentMissionBar) return;
    if (!scenario) {
      currentMissionBar.innerHTML = "";
      currentMissionBar.setAttribute("aria-hidden", "true");
      return;
    }

    currentMissionBar.innerHTML = currentMissionBarHtml(
      playbackSnapshot,
      missionEntries,
      plan,
      focusedKey,
    );
    currentMissionBar.querySelectorAll("[data-current-mission-key]").forEach((button) => {
      button.addEventListener("click", () => {
        const key = button.getAttribute("data-current-mission-key");
        selectMission(focusedKey === key ? null : key);
      });
    });
    currentMissionBar.setAttribute("aria-hidden", "false");
  };

  const selectMission = (key) => {
    focusedKey = key && missionEntries.some((entry) => entry.key === key) ? key : null;
    callbacks.onMissionFocus?.(focusedEntry());
    render();
  };

  const focusedEntry = () => missionEntries.find((entry) => entry.key === focusedKey) || null;

  const setScenario = (nextScenario) => {
    scenario = nextScenario || null;
    plan = null;
    missionEntries = [];
    focusedKey = null;
    playbackSnapshot = null;
    callbacks.onMissionFocus?.(null);
    render();
  };

  const setPlan = (nextPlan, nextScenario = scenario, options = {}) => {
    scenario = nextScenario || scenario;
    plan = nextPlan || null;
    const prior = options.preserveFocus ? focusedEntry() : null;
    missionEntries = collectMissionEntries(plan, scenario);
    focusedKey = prior
      ? missionEntries.find((entry) => entry.aircraftId === prior.aircraftId && entry.missionId === prior.missionId)?.key || null
      : null;
    callbacks.onMissionFocus?.(focusedEntry());
    render();
  };

  const updatePlayback = (snapshot) => {
    playbackSnapshot = snapshot || null;
    renderCurrentMissionBar();
    if (container && container.getAttribute("aria-hidden") !== "true") {
      const target = container.querySelector(".mlp-progress-list");
      if (target) target.innerHTML = progressHtml(playbackSnapshot, focusedEntry());
      updateMissionProgressClasses(container, missionEntries, playbackSnapshot);
    }
  };

  const setAutoFollow = (enabled) => {
    autoFollow = !!enabled;
    const input = container?.querySelector("[data-auto-follow]");
    if (input) input.checked = autoFollow;
  };

  const reset = () => {
    Object.assign(visibility, DEFAULT_VISIBILITY);
    autoFollow = true;
    for (const [key] of LAYERS) callbacks.onLayerToggle?.(key, visibility[key]);
    callbacks.onAutoFollowToggle?.(true);
    setScenario(null);
  };

  return {
    setScenario,
    setPlan,
    updatePlayback,
    setAutoFollow,
    isAutoFollow: () => autoFollow,
    getMissionFocus: focusedEntry,
    getLayerVisibility: () => ({ ...visibility }),
    reset,
  };
};

function collectMissionEntries(plan, scenario) {
  const entries = [];
  for (const [aidText, info] of Object.entries(plan?.resolved?.aircraft || {})) {
    const aircraftId = Number(aidText);
    for (const mission of info.missions || []) {
      const pathId = mission.pathID;
      const flightPath = pathId != null ? scenario?.flightPaths?.[String(pathId)] : null;
      const waypoints = flightPath?.waypoints || [];
      entries.push({
        key: `${aircraftId}:${mission.id ?? "?"}`,
        aircraftId,
        aircraftLabel: AGENT_LABELS[aircraftId] || `AC${aircraftId}`,
        color: AGENT_COLORS[aircraftId] || "#888",
        missionId: mission.id,
        missionType: mission.type,
        inputMissionId: mission.inputMissionID,
        pathId,
        isDone: !!mission.isDone,
        waypointIds: waypoints
          .map((wp) => wp.waypointID == null ? null : Number(wp.waypointID))
          .filter((value) => value != null && Number.isFinite(value)),
        waypointCount: waypoints.length,
        coordinates: flightPath?.coordinates || mission.coordinateList || [],
      });
    }
  }
  return entries;
}

function missionRowHtml(entry, focusedKey) {
  return `<button type="button" class="mlp-mission-row ${entry.key === focusedKey ? "is-active" : ""}" data-mission-key="${escapeHtml(entry.key)}" style="--mission-color:${entry.color}">
    <span class="mlp-mission-agent">${escapeHtml(entry.aircraftLabel)}</span>
    <span class="mlp-mission-main">
      <b>IM ${entry.missionId ?? "-"}</b>
      <small>Input ${entry.inputMissionId ?? "-"} · Type ${entry.missionType ?? "-"} · WP ${entry.waypointCount}</small>
    </span>
    <span class="mlp-mission-state">${entry.isDone ? "완료" : "진행"}</span>
    <span class="mlp-fit" data-fit-mission title="이 임무로 이동">⌖</span>
  </button>`;
}

function updateMissionProgressClasses(container, entries, snapshot) {
  const states = snapshot?.aircraft || {};
  for (const entry of entries) {
    const rawWp = states[String(entry.aircraftId)]?.waypointID;
    const wp = rawWp == null ? null : Number(rawWp);
    const active = wp != null && Number.isFinite(wp) && entry.waypointIds.includes(wp);
    container.querySelector(`[data-mission-key="${cssEscape(entry.key)}"]`)?.classList.toggle("is-current", active);
  }
}

function currentMissionBarHtml(snapshot, entries, plan, focusedKey) {
  const states = snapshot?.aircraft || {};
  const aircraftIds = new Set([
    ...entries.map((entry) => entry.aircraftId),
    ...Object.keys(states).map(Number).filter(Number.isFinite),
  ]);
  const chips = [...aircraftIds]
    .sort((a, b) => a - b)
    .map((aircraftId) => currentMissionChipHtml(
      aircraftId,
      states[String(aircraftId)] || null,
      entries.filter((entry) => entry.aircraftId === aircraftId),
      focusedKey,
      !!snapshot,
    ));

  const planLabel = plan?.missionPlanID == null ? "Plan 없음" : `Plan ${plan.missionPlanID}`;
  return `<div class="cmb-head">
    <span class="cmb-title"><i></i>현재 수행 임무</span>
    <small>0401 · ${escapeHtml(planLabel)}</small>
  </div>
  <div class="cmb-list">
    ${chips.length ? chips.join("") : '<div class="cmb-empty">선택 Plan의 항공기 임무가 없습니다.</div>'}
  </div>`;
}

function currentMissionChipHtml(aircraftId, state, aircraftEntries, focusedKey, hasSnapshot) {
  const label = AGENT_LABELS[aircraftId] || `AC${aircraftId}`;
  const color = AGENT_COLORS[aircraftId] || "#888";
  const rawWp = state?.waypointID;
  const waypointId = rawWp == null ? null : Number(rawWp);
  const matchingEntry = Number.isFinite(waypointId)
    ? aircraftEntries.find((entry) => entry.waypointIds.includes(waypointId)) || null
    : null;
  const focused = !!matchingEntry && matchingEntry.key === focusedKey;

  let stateClass = "is-waiting";
  let title = "재생 대기";
  let detail = aircraftEntries.length ? `배정 임무 ${aircraftEntries.length}개` : "Plan 배정 없음";

  if (hasSnapshot && !state) {
    stateClass = "is-unknown";
    title = "0401 기록 없음";
    detail = aircraftEntries.length ? "임무는 배정됨" : "Plan 배정 없음";
  } else if (state && state.hasWaypointTelemetry === false) {
    stateClass = "is-unknown";
    title = "WP telemetry 없음";
    detail = aircraftEntries.length ? "현재 임무 매핑 불가" : "Plan 배정 없음";
  } else if (state && waypointId == null) {
    stateClass = "is-waiting";
    title = "현재 WP 없음";
    detail = aircraftEntries.length ? "임무 진입 전·종료 후" : "Plan 배정 없음";
  } else if (state && matchingEntry) {
    stateClass = "is-current";
    title = missionTypeLabel(matchingEntry.missionType);
    detail = `IM ${matchingEntry.missionId ?? "-"} · Input ${matchingEntry.inputMissionId ?? "-"} · WP ${rawWp}`;
  } else if (state) {
    stateClass = "is-unmapped";
    title = aircraftEntries.length ? "임무 매핑 없음" : "Plan 배정 없음";
    detail = `현재 WP ${rawWp ?? "-"}`;
  }

  const tag = matchingEntry ? "button" : "div";
  const interactiveAttrs = matchingEntry
    ? `type="button" data-current-mission-key="${escapeHtml(matchingEntry.key)}" aria-pressed="${focused}"`
    : "";
  const titleText = matchingEntry
    ? `${label} · ${title} · ${detail} · 클릭: 임무 시간 구간만 보기 / 다시 클릭: 전체보기`
    : `${label} · ${title} · ${detail}`;
  return `<${tag} ${interactiveAttrs} class="cmb-chip ${stateClass} ${focused ? "is-focused" : ""}" style="--mission-color:${color}" title="${escapeHtml(titleText)}">
    <span class="cmb-agent">${escapeHtml(label)}</span>
    <span class="cmb-mission"><b>${escapeHtml(title)}</b><small>${escapeHtml(detail)}</small></span>
  </${tag}>`;
}

function missionTypeLabel(type) {
  const numeric = Number(type);
  if (Number.isFinite(numeric) && MISSION_TYPE_LABELS[numeric] != null) {
    return MISSION_TYPE_LABELS[numeric];
  }
  return type == null ? "임무 타입 없음" : `타입 ${type}`;
}

function progressHtml(snapshot, focus) {
  if (!snapshot?.aircraft) return '<div class="mlp-empty">재생 데이터가 없습니다.</div>';
  const rows = [];
  for (const [aid, state] of Object.entries(snapshot.aircraft)) {
    if (focus && Number(aid) !== focus.aircraftId) continue;
    const label = AGENT_LABELS[Number(aid)] || `AC${aid}`;
    const hasTelemetry = !!state.hasWaypointTelemetry;
    const wp = state.waypointID == null ? "-" : state.waypointID;
    const inFocusedMission = !focus || (
      state.waypointID != null
      && focus.waypointIds.includes(Number(state.waypointID))
    );
    const progressLabel = !hasTelemetry
      ? "WP telemetry 없음"
      : inFocusedMission
        ? `WP ${escapeHtml(wp)}`
        : state.waypointID == null
          ? "선택 임무 대기"
          : `선택 임무 외 · 현재 WP ${escapeHtml(wp)}`;
    rows.push(`<div class="mlp-progress-row">
      <span style="--mission-color:${AGENT_COLORS[Number(aid)] || "#888"}">${escapeHtml(label)}</span>
      <b>${progressLabel}</b>
      <small>${state.flightMode == null ? "" : `Mode ${escapeHtml(state.flightMode)}`}</small>
    </div>`);
  }
  return rows.length ? rows.join("") : '<div class="mlp-empty">선택 임무의 진행 데이터가 없습니다.</div>';
}

function auditBadge(audit) {
  const status = audit?.status || "empty";
  const label = status === "complete" ? "COMPLETE" : status === "partial" ? "PARTIAL" : "EMPTY";
  return `<span class="mlp-audit-badge is-${status}">${label}</span>`;
}

function auditHtml(scenario) {
  const audit = scenario?.loadAudit || {};
  const playback = scenario?.playback0401 || {};
  const ingestion = playback.ingestion || audit.playback0401 || {};
  const count = (section, key) => Number(audit?.[section]?.[key] || 0).toLocaleString();
  const retained = Number(ingestion.retainedTrackSampleCount ?? playback.sampleCount ?? 0);
  const raw = Number(ingestion.coordinateSampleCount ?? 0);
  const recognized = Number(ingestion.recognizedMessageCount ?? playback.messageCount ?? 0);
  const records = Number(ingestion.rawRecordCount ?? recognized);
  const targetHz = Number(ingestion.visualizationTargetHz ?? ingestion.playbackTargetHz ?? 5);
  return `<div class="mlp-audit">
    <div><span>핵심 파일</span><b>MP ${count("missionPlan", "loaded")}/${count("missionPlan", "files")} · IMP ${count("individualMissionPlan", "loaded")}/${count("individualMissionPlan", "files")} · Path ${count("flightPath", "loaded")}/${count("flightPath", "files")} · Input ${count("inputMissionPlan", "loaded")}/${count("inputMissionPlan", "files")}</b></div>
    <div><span>0401 적재</span><b>${Number(playback.fileCount || 0)} files · ${recognized.toLocaleString()}/${records.toLocaleString()} messages</b></div>
    <div><span>항적 샘플</span><b>${raw.toLocaleString()} raw · ${retained.toLocaleString()} retained · UI ${targetHz.toFixed(0)}Hz</b></div>
  </div>`;
}

function escapeHtml(value) {
  return String(value ?? "")
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;");
}

function cssEscape(value) {
  return String(value).replace(/([:\\.])/g, "\\$1");
}
