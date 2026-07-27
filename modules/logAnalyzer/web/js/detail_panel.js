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
const WEAPON_TYPE_LABELS = { 1: "기관포", 2: "유도탄", 3: "로켓" };

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

function etaLabel(value) {
  const seconds = Number(value);
  return Number.isFinite(seconds) ? `${seconds.toFixed(1)}s` : "-";
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

    // Assigned area allocation (그때 그때 할당된 영역)
    const allocSec = buildAllocationSection(plan);
    if (allocSec) container.appendChild(allocSec);

    // Detected targets at this plan's time
    const targetSec = buildTargetSection(opts.targets || [], opts.targetTotal ?? 0, opts.onTargetClick);
    if (targetSec) container.appendChild(targetSec);

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

  function buildAllocationSection(plan) {
    const resolved = plan.resolved?.aircraft || {};
    const rows = [];
    let totalM2 = 0;

    for (const ac of plan.aircraftList || []) {
      const aid = ac.aircraftID;
      const info = resolved[aid] || resolved[String(aid)] || {};
      let areaM2 = 0;
      let corridorM = 0;
      let done = 0;
      let total = 0;
      for (const m of info.missions || []) {
        total += 1;
        if (m.isDone) done += 1;
        for (const ring of m.areaList || []) {
          const a = polygonAreaM2(ring.coordinates || []);
          areaM2 += ring.isHole ? -a : a;
        }
        for (const line of m.lineList || []) {
          const len = lineLengthM(line.coordinates || []);
          corridorM += len;
          if (line.width) areaM2 += len * line.width;
        }
      }
      areaM2 = Math.max(0, areaM2);
      rows.push({ aid, areaM2, corridorM, done, total });
      totalM2 += areaM2;
    }

    if (totalM2 <= 0) return null;

    const sec = section("영역 할당");

    // Stacked share bar
    const bar = el("div", "alloc-bar");
    for (const r of rows) {
      if (r.areaM2 <= 0) continue;
      const pct = (r.areaM2 / totalM2) * 100;
      const seg = el("span", "alloc-bar-seg");
      seg.style.width = `${pct}%`;
      seg.style.background = AGENT_COLORS[r.aid] || "#888";
      seg.title = `${AGENT_LABELS[r.aid] || r.aid}: ${fmtAreaM2(r.areaM2)} (${pct.toFixed(1)}%)`;
      bar.appendChild(seg);
    }
    sec.appendChild(bar);

    const totalRow = el("div", "alloc-total");
    totalRow.textContent = `총 할당 ${fmtAreaM2(totalM2)}`;
    sec.appendChild(totalRow);

    for (const r of rows) {
      if (r.areaM2 <= 0 && r.total === 0) continue;
      const row = el("div", "alloc-row");

      const dot = el("span", "agent-dot");
      dot.style.background = AGENT_COLORS[r.aid] || "#888";
      row.appendChild(dot);

      const name = el("span", "alloc-name");
      name.textContent = AGENT_LABELS[r.aid] || `AC${r.aid}`;
      row.appendChild(name);

      const val = el("span", "alloc-val");
      if (r.areaM2 > 0) {
        const pct = ((r.areaM2 / totalM2) * 100).toFixed(1);
        val.textContent = `${fmtAreaM2(r.areaM2)} · ${pct}%`;
        if (r.corridorM > 0) val.textContent += ` · 회랑 ${fmtDistM(r.corridorM)}`;
      } else {
        val.textContent = "할당 영역 없음";
      }
      row.appendChild(val);

      const prog = el("span", "alloc-progress");
      const fill = el("span");
      fill.style.width = r.total > 0 ? `${(r.done / r.total) * 100}%` : "0";
      prog.appendChild(fill);
      row.appendChild(prog);

      const progText = el("span", "alloc-progress-text");
      progText.textContent = `${r.done}/${r.total}`;
      progText.title = "완료 임무 / 전체 임무";
      row.appendChild(progText);

      sec.appendChild(row);
    }

    return sec;
  }

  function buildTargetSection(targets, total, onTargetClick) {
    if (!total) return null;
    const sec = section(`발견 표적 (${targets.length}/${total})`);

    if (targets.length === 0) {
      const none = el("div", "target-empty");
      none.textContent = "이 시점까지 발견된 표적 없음";
      sec.appendChild(none);
      return sec;
    }

    for (const t of targets) {
      const row = el("div", t.isDestroyed ? "target-row is-destroyed" : "target-row");

      const dot = el("span", t.isDestroyed ? "target-dot is-destroyed" : "target-dot");
      row.appendChild(dot);

      const name = el("span", "target-name");
      name.textContent = `${t.typeLabel} ${t.targetID}`;
      row.appendChild(name);

      const metaEl = el("span", "target-meta");
      const bits = [];
      if (t.detectedText) bits.push(t.detectedText);
      if (t.watcherLabel) bits.push(t.watcherLabel);
      metaEl.textContent = bits.join(" · ");
      row.appendChild(metaEl);

      if (t.isDestroyed) {
        const badge = el("span", "target-destroyed-badge");
        badge.textContent = "파괴";
        row.appendChild(badge);
      }

      if (onTargetClick) {
        row.style.cursor = "pointer";
        row.title = "지도에서 보기";
        row.addEventListener("click", () => onTargetClick(t));
      }
      sec.appendChild(row);
    }
    return sec;
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

        const fp = scenario.flightPaths?.[String(m.pathID)];
        if (aid >= 1 && aid <= 3 && Number(m.type) === 2) {
          const attackWp = (fp?.waypoints || []).find((wp) => wp.lahType === "attack");
          const fallback = m.coordinateList?.[0];
          const coordinate = attackWp
            ? [attackWp.longitude, attackWp.latitude]
            : fallback?.length >= 2 ? fallback : null;
          if (coordinate) {
            const attackPoint = el("button", "imp-attack-point");
            attackPoint.type = "button";
            const targetId = attackWp?.attack?.targetID ?? m.targetID;
            const weaponType = attackWp?.attack?.weaponType;
            const source = attackWp ? `WP ${attackWp.waypointID}` : "계획 좌표";
            const target = targetId == null || Number(targetId) === 0 ? "표적 미기록" : `T${targetId}`;
            const weapon = WEAPON_TYPE_LABELS[Number(weaponType)] || (weaponType ? `W${weaponType}` : "무장 미기록");
            attackPoint.textContent = `공격 Point · ${source} · ${target} · ${weapon}`;
            attackPoint.title = `${Number(coordinate[1]).toFixed(6)}, ${Number(coordinate[0]).toFixed(6)}`;
            if (onPathClick) {
              attackPoint.addEventListener("click", () => onPathClick(m.pathID, [coordinate]));
            }
            detail.appendChild(attackPoint);
          }
        }

        // WP list for this mission's path
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
      if (wp.lahType === "attack") tr.classList.add("is-attack-point");
      const lat = wp.latitude != null ? Number(wp.latitude).toFixed(5) : "-";
      const lon = wp.longitude != null ? Number(wp.longitude).toFixed(5) : "-";
      const alt = wp.altitude != null ? wp.altitude : "-";
      const spd = wp.speed != null ? wp.speed : "-";
      const eta = wp.eta != null ? etaLabel(wp.eta) : "-";
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

/** Shoelace area on an equirectangular projection — coords: [[lon, lat], ...]. */
function polygonAreaM2(coords) {
  if (!coords || coords.length < 3) return 0;
  const lat0 = (coords[0][1] * Math.PI) / 180;
  const mPerDegLat = 111320;
  const mPerDegLon = 111320 * Math.cos(lat0);
  let sum = 0;
  for (let i = 0; i < coords.length; i++) {
    const [x1, y1] = coords[i];
    const [x2, y2] = coords[(i + 1) % coords.length];
    sum += x1 * mPerDegLon * (y2 * mPerDegLat) - x2 * mPerDegLon * (y1 * mPerDegLat);
  }
  return Math.abs(sum) / 2;
}

function lineLengthM(coords) {
  if (!coords || coords.length < 2) return 0;
  const lat0 = (coords[0][1] * Math.PI) / 180;
  const mPerDegLat = 111320;
  const mPerDegLon = 111320 * Math.cos(lat0);
  let len = 0;
  for (let i = 1; i < coords.length; i++) {
    const dx = (coords[i][0] - coords[i - 1][0]) * mPerDegLon;
    const dy = (coords[i][1] - coords[i - 1][1]) * mPerDegLat;
    len += Math.hypot(dx, dy);
  }
  return len;
}

function fmtAreaM2(value) {
  const num = Number(value);
  if (!Number.isFinite(num) || num <= 0) return "-";
  if (num >= 1_000_000) return `${(num / 1_000_000).toFixed(2)} km²`;
  return `${Math.round(num).toLocaleString()} m²`;
}

function fmtDistM(value) {
  const num = Number(value);
  if (!Number.isFinite(num) || num <= 0) return "-";
  if (num >= 1000) return `${(num / 1000).toFixed(1)} km`;
  return `${Math.round(num)} m`;
}

function fmtTs(ts) {
  try {
    return new Date(ts).toLocaleString("ko-KR");
  } catch { return String(ts); }
}
