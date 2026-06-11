/**
 * Left detail panel — shows plan metadata, aircraft table, IMP + WP details.
 */

import { AGENT_COLORS, AGENT_LABELS } from "./palette.js";

const MISSION_TYPE_LABELS = {
  0: "없음", 1: "표적추적", 2: "표적공격", 3: "지역수색",
  4: "지역감시", 5: "지점정찰", 6: "회랑정찰", 7: "이동",
  8: "엄호", 9: "은폐엄호",
};

const PATTERN_TYPE_LABELS = {
  0: "없음", 1: "표적중심선회", 2: "은폐후공격", 3: "Nadir-BF",
  4: "Offset-BF", 5: "구간왕복-BF", 6: "선형반복-BF", 7: "상공선회",
  8: "구간중심종-선형", 9: "구간중심종-자동", 10: "목적지이동",
  11: "표적엄호", 12: "은폐엄호",
};

const PASS_TYPE_LABELS = { 0: "-", 1: "Fly-by", 2: "Loiter", 3: "Fly-Over" };
const LAH_TYPE_LABELS = { attack: "공격", hovering: "호버링", loiter: "선회" };

function wpPassLabel(wp) {
  // UAV: waypointPassType
  if (wp.waypointPassType != null) return PASS_TYPE_LABELS[wp.waypointPassType] || "-";
  // LAH: attack/hovering/loiter
  if (wp.lahType) {
    let label = LAH_TYPE_LABELS[wp.lahType] || wp.lahType;
    if (wp.lahType === "attack" && wp.attack) {
      const tid = wp.attack.targetID;
      const wtype = wp.attack.weaponType;
      if (tid) label += ` T${tid}`;
      if (wtype) label += ` W${wtype}`;
    }
    if (wp.lahType === "hovering" && wp.hovering) {
      label += ` ${wp.hovering.time}s`;
    }
    if (wp.lahType === "loiter" && wp.loiter) {
      const parts = [];
      if (wp.loiter.radius) parts.push(`R${wp.loiter.radius}m`);
      if (wp.loiter.time) parts.push(`${wp.loiter.time}s`);
      if (parts.length) label += ` ${parts.join(" ")}`;
    }
    return label;
  }
  return "-";
}

export const createDetailPanel = (container) => {
  const titleEl = document.getElementById("detail-title");
  const subtitleEl = document.getElementById("detail-subtitle");
  let onPathClick = null;

  const hide = () => {
    container.innerHTML = '<div class="detail-empty">플랜을 선택하세요</div>';
    if (titleEl) titleEl.textContent = "Mission Plan";
    if (subtitleEl) subtitleEl.textContent = "";
  };

  const show = (plan, scenario, opts = {}) => {
    onPathClick = opts.onPathClick || null;
    container.innerHTML = "";

    if (titleEl) titleEl.textContent = `Mission Plan ${plan.missionPlanID || ""}`;
    if (subtitleEl) subtitleEl.textContent = plan.timestamp ? fmtTs(plan.timestamp) : "";

    // Replan badge
    if (opts.replanInfo) {
      const badge = el("div", "detail-replan-badge");
      badge.textContent = `재계획: ${opts.replanInfo.reason || ""}`;
      container.appendChild(badge);
    }

    // Metadata
    const metaSec = section("기본 정보");
    const mg = el("div", "detail-meta");
    meta(mg, "Plan ID", plan.missionPlanID);
    meta(mg, "계획 시간", plan.planningTime != null ? `${plan.planningTime}ms` : "-");
    meta(mg, "입력 패키지", plan.inputMissionPackageID || "-");
    metaSec.appendChild(mg);
    container.appendChild(metaSec);

    // Aircraft table
    const resolved = plan.resolved?.aircraft || {};
    const aircraftList = plan.aircraftList || [];
    if (aircraftList.length > 0) {
      const tSec = section("항공기 배정");
      tSec.appendChild(buildAircraftTable(aircraftList, resolved));
      container.appendChild(tSec);

      // IMP + missions + WP details
      const impSec = section("개별 임무 패키지 (IMP)");
      for (const ac of aircraftList) {
        const aid = ac.aircraftID;
        const info = resolved[aid] || resolved[String(aid)] || {};
        impSec.appendChild(buildImpBlock(ac, info, scenario));
      }
      container.appendChild(impSec);
    }
  };

  return { show, hide };

  // ── helpers ──

  function section(title) {
    const s = el("div", "detail-section");
    const h = el("div", "detail-section-title");
    h.textContent = title;
    s.appendChild(h);
    return s;
  }

  function meta(grid, label, value) {
    const l = el("span", "detail-meta-label"); l.textContent = label;
    const v = el("span", "detail-meta-value"); v.textContent = value ?? "-";
    grid.appendChild(l); grid.appendChild(v);
  }

  function buildAircraftTable(aircraftList, resolved) {
    const table = el("table", "aircraft-table");
    table.innerHTML = `<thead><tr><th>기체</th><th>IMP</th><th>임무</th><th>경로</th></tr></thead>`;
    const tbody = el("tbody");
    for (const ac of aircraftList) {
      const aid = ac.aircraftID;
      const info = resolved[aid] || resolved[String(aid)] || {};
      const color = AGENT_COLORS[aid] || "#888";
      const label = AGENT_LABELS[aid] || `AC${aid}`;
      const tr = el("tr");
      tr.innerHTML = `
        <td><span class="agent-dot" style="background:${color}"></span>${label}</td>
        <td class="mono" style="font-size:10px">${info.impID || ac.individualMissionPackageID || "-"}</td>
        <td>${(info.missions || []).length}</td>
        <td>${(info.paths || []).length}</td>`;
      tbody.appendChild(tr);
    }
    table.appendChild(tbody);
    return table;
  }

  function buildImpBlock(ac, info, scenario) {
    const aid = ac.aircraftID;
    const color = AGENT_COLORS[aid] || "#888";
    const label = AGENT_LABELS[aid] || `AC${aid}`;

    const toggle = el("button", "imp-toggle");
    toggle.innerHTML = `<span class="imp-toggle-arrow">\u25B6</span>
      <span class="agent-dot" style="background:${color}"></span>
      <span>${label} &mdash; IMP ${info.impID || ac.individualMissionPackageID || "?"}</span>`;

    const detail = el("div", "imp-detail");
    const missions = info.missions || [];
    const paths = info.paths || [];

    if (missions.length === 0 && paths.length === 0) {
      detail.innerHTML = '<div style="color:var(--text-muted);font-size:11px;padding:6px 0">데이터 없음</div>';
    } else {
      // Missions
      for (const m of missions) {
        const mRow = el("div", "imp-mission-row");

        const typeTag = el("span", "imp-mission-type");
        typeTag.textContent = MISSION_TYPE_LABELS[m.type] || `타입${m.type || "?"}`;

        const patternTag = el("span", "imp-mission-pattern");
        if (m.patternType) patternTag.textContent = PATTERN_TYPE_LABELS[m.patternType] || "";

        const pathLink = el("span", "imp-path-link");
        pathLink.textContent = `Path ${m.pathID || "-"}`;
        if (m.pathID && onPathClick) {
          pathLink.style.cursor = "pointer";
          pathLink.addEventListener("click", (e) => {
            e.stopPropagation();
            const fp = scenario.flightPaths?.[String(m.pathID)];
            if (fp?.coordinates) onPathClick(m.pathID, fp.coordinates);
          });
        }

        const statusTag = el("span", m.isDone ? "imp-status-done" : "imp-status-pending");
        statusTag.textContent = m.isDone ? "완료" : "대기";

        mRow.appendChild(typeTag);
        if (m.patternType) mRow.appendChild(patternTag);
        mRow.appendChild(pathLink);
        mRow.appendChild(statusTag);
        detail.appendChild(mRow);

        // WP list for this mission's path
        const fp = scenario.flightPaths?.[String(m.pathID)];
        if (fp?.waypoints?.length) {
          const wpBlock = buildWpBlock(fp.waypoints, m.pathID);
          detail.appendChild(wpBlock);
        }
      }
    }

    toggle.addEventListener("click", () => {
      toggle.classList.toggle("is-open");
      detail.classList.toggle("is-open");
    });

    const wrapper = el("div");
    wrapper.appendChild(toggle);
    wrapper.appendChild(detail);
    return wrapper;
  }

  function buildWpBlock(waypoints, pathID) {
    const wrapper = el("div", "wp-block");
    const header = el("div", "wp-block-header");
    header.textContent = `웨이포인트 (${waypoints.length}개)`;
    wrapper.appendChild(header);

    const table = el("table", "wp-table");
    table.innerHTML = `<thead><tr><th>WP</th><th>좌표</th><th>고도</th><th>속도</th><th>ETA</th><th>통과</th></tr></thead>`;
    const tbody = el("tbody");

    for (const wp of waypoints) {
      const tr = el("tr");
      const lat = wp.latitude != null ? Number(wp.latitude).toFixed(5) : "-";
      const lon = wp.longitude != null ? Number(wp.longitude).toFixed(5) : "-";
      const alt = wp.altitude != null ? wp.altitude : "-";
      const spd = wp.speed != null ? wp.speed : "-";
      const eta = wp.eta != null ? `${wp.eta}s` : "-";
      const pass = wpPassLabel(wp);

      tr.innerHTML = `
        <td class="mono">${wp.waypointID ?? "-"}</td>
        <td class="mono" style="font-size:9px">${lat}, ${lon}</td>
        <td>${alt}m</td>
        <td>${spd}</td>
        <td>${eta}</td>
        <td>${pass}</td>`;

      if (onPathClick && wp.longitude != null && wp.latitude != null) {
        tr.style.cursor = "pointer";
        tr.addEventListener("click", () => onPathClick(pathID, [[wp.longitude, wp.latitude]]));
      }
      tbody.appendChild(tr);
    }
    table.appendChild(tbody);
    wrapper.appendChild(table);
    return wrapper;
  }
};

function el(tag, cls) {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  return e;
}

function fmtTs(ts) {
  try {
    return new Date(ts).toLocaleString("ko-KR");
  } catch { return String(ts); }
}
