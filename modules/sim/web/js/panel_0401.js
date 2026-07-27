import { buildAgentStatus, getActiveAgent, subscribe } from "./agent_store.js";

const HEALTH_LABELS = {
  0: { text: "미확인", tone: "muted" },
  1: { text: "정상", tone: "ok" },
  2: { text: "비정상", tone: "bad" },
};

const PAYLOAD_LABELS = {
  0: { text: "없음", tone: "muted" },
  1: { text: "정상", tone: "ok" },
  2: { text: "비정상", tone: "bad" },
};

const FUEL_WARN_LABELS = {
  0: { text: "없음", tone: "muted" },
  1: { text: "양호", tone: "ok" },
  2: { text: "경고", tone: "warn" },
  3: { text: "위험", tone: "bad" },
};

const MISSION_LABELS = {
  0: { text: "기본", tone: "muted" },
  1: { text: "수행중", tone: "ok" },
  2: { text: "완료", tone: "ok" },
};

const FLIGHT_MODE_LABELS = {
  0: "미사용",
  1: "자동이륙",
  2: "자동착륙",
  3: "통제권이양지이동",
  4: "전술집결지이동",
  5: "RTB",
  6: "편대비행",
  7: "경로이동비행",
  8: "점항법비행",
  9: "표적추적비행",
};

const coerceInt = (value, fallback = null) => {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? Math.trunc(parsed) : fallback;
};

const pickNested = (value, keys) => {
  if (!value || typeof value !== "object") {
    return null;
  }
  for (const key of keys) {
    if (value[key] !== undefined && value[key] !== null) {
      return value[key];
    }
  }
  return null;
};

const readWaypointId = (entry) => {
  const direct = entry?.currentWaypointID ?? entry?.CurrentWaypointID;
  const nested = pickNested(direct, [
    "waypointID",
    "WaypointID",
    "id",
    "ID",
  ]);
  if (nested !== null) {
    return coerceInt(nested, null);
  }
  const viaInfo = entry?.unmannedInfo?.currentWaypointID ?? entry?.unmannedInfo?.CurrentWaypointID;
  const viaInfoNested = pickNested(viaInfo, ["waypointID", "WaypointID", "id", "ID"]);
  if (viaInfoNested !== null) {
    return coerceInt(viaInfoNested, null);
  }
  return coerceInt(direct ?? viaInfo, null);
};

const readTargetId = (entry) => {
  const target = entry?.targetFollowing || entry?.TargetFollowing || entry?.unmannedInfo?.targetFollowing || entry?.unmannedInfo?.TargetFollowing;
  const nested = pickNested(target, ["targetID", "TargetID", "id", "ID"]);
  if (nested !== null) {
    return coerceInt(nested, null);
  }
  return coerceInt(entry?.targetID ?? entry?.TargetID, null);
};

const readFlightMode = (entry) => {
  const value = entry?.flightMode ?? entry?.FlightMode ?? entry?.unmannedInfo?.flightMode ?? entry?.unmannedInfo?.FlightMode;
  return coerceInt(value, null);
};

const readFlying = (entry) => {
  const value = entry?.flying ?? entry?.Flying ?? entry?.unmannedInfo?.flying ?? entry?.unmannedInfo?.Flying;
  return coerceInt(value, null);
};

const readFilming = (entry) => {
  const sensorInfo = entry?.unmannedInfo?.sensorInfo ?? entry?.unmannedInfo?.SensorInfo;
  const value = entry?.filming ?? entry?.Filming ?? sensorInfo?.filming ?? sensorInfo?.Filming;
  return coerceInt(value, null);
};

const applySimState = (status, simEntry) => {
  if (!status || !simEntry) {
    return status;
  }
  const agent = status.agentStateList?.[0];
  if (!agent) {
    return status;
  }
  const lat = Number(simEntry.lat);
  const lon = Number(simEntry.lon);
  const alt = Number(simEntry.alt);
  if (Number.isFinite(lat)) {
    agent.coordinate.latitude = lat;
  }
  if (Number.isFinite(lon)) {
    agent.coordinate.longitude = lon;
  }
  if (Number.isFinite(alt)) {
    agent.coordinate.altitude = alt;
  }
  const speed = Number(simEntry.speed);
  const heading = Number(simEntry.heading);
  if (Number.isFinite(speed)) {
    agent.velocity.speed = speed;
  }
  if (Number.isFinite(heading)) {
    agent.velocity.heading = heading;
  }
  const fuel = Number(simEntry.fuel);
  if (Number.isFinite(fuel)) {
    agent.fuel = fuel;
  }
  const simHealth = Number(simEntry.health);
  if (Number.isFinite(simHealth)) {
    agent.health = simHealth;
  } else if (typeof simEntry.alive === "boolean") {
    agent.health = simEntry.alive ? 1 : 2;
  }
  const currentWaypointID = readWaypointId(simEntry);
  const targetID = readTargetId(simEntry);
  if (agent.isUnmanned) {
    const flightMode = readFlightMode(simEntry);
    const flying = readFlying(simEntry);
    const filming = readFilming(simEntry);
    if (Number.isFinite(flightMode)) {
      agent.unmannedInfo.flightMode = flightMode;
    }
    if (Number.isFinite(flying)) {
      agent.unmannedInfo.flying = flying;
    }
    if (Number.isFinite(filming)) {
      agent.unmannedInfo.sensorInfo = agent.unmannedInfo.sensorInfo || {};
      agent.unmannedInfo.sensorInfo.filming = filming;
    }
    if (Number.isFinite(currentWaypointID)) {
      agent.unmannedInfo.currentWaypointID = currentWaypointID;
    }
    if (Number.isFinite(targetID)) {
      if (agent.unmannedInfo.targetFollowing && typeof agent.unmannedInfo.targetFollowing === "object") {
        agent.unmannedInfo.targetFollowing.targetID = targetID;
      } else {
        agent.unmannedInfo.targetFollowing = { targetID };
      }
    }
  }
  if (!agent.isUnmanned && simEntry.weapons) {
    const weapons = simEntry.weapons;
    if (Number.isFinite(Number(weapons.type1))) {
      agent.mannedInfo.weapons.type1 = Number(weapons.type1);
    }
    if (Number.isFinite(Number(weapons.type2))) {
      agent.mannedInfo.weapons.type2 = Number(weapons.type2);
    }
    if (Number.isFinite(Number(weapons.type3))) {
      agent.mannedInfo.weapons.type3 = Number(weapons.type3);
    }
  }
  return status;
};

const formatNumber = (value, digits = 2) => {
  if (value === null || value === undefined || Number.isNaN(value)) {
    return "-";
  }
  return Number(value).toFixed(digits);
};

const formatCoord = (value) => formatNumber(value, 5);

const formatTime = (msSince2000) => {
  const base = Date.UTC(2000, 0, 1, 0, 0, 0);
  const date = new Date(base + Number(msSince2000 || 0));
  return `${date.getFullYear()}-${String(date.getMonth() + 1).padStart(2, "0")}-${String(
    date.getDate()
  ).padStart(2, "0")} ${String(date.getHours()).padStart(2, "0")}:${String(
    date.getMinutes()
  ).padStart(2, "0")}:${String(date.getSeconds()).padStart(2, "0")}`;
};

const toneClass = (tone) => {
  if (tone === "ok") return "chip chip-ok";
  if (tone === "warn") return "chip chip-warn";
  if (tone === "bad") return "chip chip-bad";
  return "chip chip-muted";
};

const statusChip = (label, tone) => `<span class="${toneClass(tone)}">${label}</span>`;

const buildTopCard = (status, agent, label) => {
  const health = HEALTH_LABELS[agent.health] || HEALTH_LABELS[0];
  return `
    <div class="data-card span-2 data-card-top">
      <div class="data-card-title">0401 AgentStatus · ${label}</div>
      <div class="top-grid">
        <div class="kv-grid">
          <div class="kv"><span class="kv-label">timestamp</span><span class="kv-value">${formatTime(
            status.timestamp
          )}</span></div>
          <div class="kv"><span class="kv-label">source</span><span class="kv-value">${status.source}</span></div>
          <div class="kv"><span class="kv-label">messageId</span><span class="kv-value">0401</span></div>
          <div class="kv"><span class="kv-label">agentStateList</span><span class="kv-value">1</span></div>
        </div>
        <div class="kv-grid">
          <div class="kv"><span class="kv-label">aircraftID</span><span class="kv-value">${agent.aircraftID}</span></div>
          <div class="kv"><span class="kv-label">isUnmanned</span><span class="kv-value">${agent.isUnmanned ? "true" : "false"}</span></div>
          <div class="kv"><span class="kv-label">health</span><span class="kv-value">${statusChip(
            health.text,
            health.tone
          )}</span></div>
          <div class="kv"><span class="kv-label">fuel (L)</span><span class="kv-value">${formatNumber(
            agent.fuel,
            1
          )}</span></div>
          <div class="kv"><span class="kv-label">lastSignal</span><span class="kv-value">${formatTime(
            agent.lastSignalTime
          )}</span></div>
        </div>
      </div>
    </div>
  `;
};

const buildPositionCard = (agent) => `
  <div class="data-card">
    <div class="data-card-title">좌표 · 속도</div>
    <div class="kv-grid kv-grid-3">
      <div class="kv"><span class="kv-label">lat</span><span class="kv-value">${formatCoord(
        agent.coordinate.latitude
      )}</span></div>
      <div class="kv"><span class="kv-label">lon</span><span class="kv-value">${formatCoord(
        agent.coordinate.longitude
      )}</span></div>
      <div class="kv"><span class="kv-label">alt</span><span class="kv-value">${formatNumber(
        agent.coordinate.altitude,
        0
      )} m</span></div>
      <div class="kv"><span class="kv-label">speed</span><span class="kv-value">${formatNumber(
        agent.velocity.speed,
        1
      )} m/s</span></div>
      <div class="kv"><span class="kv-label">heading</span><span class="kv-value">${formatNumber(
        agent.velocity.heading,
        1
      )}°</span></div>
    </div>
  </div>
`;

const buildMannedSystemCard = (agent) => {
  const link = agent.mannedInfo.datalinkStatus;
  const linkChip = (value) => statusChip(value ? "연결" : "끊김", value ? "ok" : "warn");
  return `
    <div class="data-card">
      <div class="data-card-title">시스템 상태</div>
      <div class="kv-grid kv-grid-3">
        <div class="kv"><span class="kv-label">UAV1 링크</span><span class="kv-value">${linkChip(
          link.isConnectedToUAV1
        )}</span></div>
        <div class="kv"><span class="kv-label">UAV2 링크</span><span class="kv-value">${linkChip(
          link.isConnectedToUAV2
        )}</span></div>
        <div class="kv"><span class="kv-label">UAV3 링크</span><span class="kv-value">${linkChip(
          link.isConnectedToUAV3
        )}</span></div>
      </div>
      <div class="kv-grid kv-grid-3">
        <div class="kv"><span class="kv-label">무장1</span><span class="kv-value">${agent.mannedInfo.weapons.type1}</span></div>
        <div class="kv"><span class="kv-label">무장2</span><span class="kv-value">${agent.mannedInfo.weapons.type2}</span></div>
        <div class="kv"><span class="kv-label">무장3</span><span class="kv-value">${agent.mannedInfo.weapons.type3}</span></div>
      </div>
    </div>
  `;
};

const buildUnmannedSystemCard = (agent) => {
  const payload = PAYLOAD_LABELS[agent.unmannedInfo.payloadHealth] || PAYLOAD_LABELS[0];
  const fuelWarn = FUEL_WARN_LABELS[agent.unmannedInfo.fuelWarning] || FUEL_WARN_LABELS[0];
  const mission = MISSION_LABELS[agent.unmannedInfo.flying] || MISSION_LABELS[0];
  const flightModeLabel = FLIGHT_MODE_LABELS[agent.unmannedInfo.flightMode] || "-";
  return `
    <div class="data-card">
      <div class="data-card-title">임무 · 장비</div>
      <div class="kv-grid kv-grid-3">
        <div class="kv"><span class="kv-label">flightMode</span><span class="kv-value">${flightModeLabel}</span></div>
        <div class="kv"><span class="kv-label">mission</span><span class="kv-value">${statusChip(
          mission.text,
          mission.tone
        )}</span></div>
        <div class="kv"><span class="kv-label">currentWP</span><span class="kv-value">${agent.unmannedInfo.currentWaypointID}</span></div>
        <div class="kv"><span class="kv-label">targetID</span><span class="kv-value">${agent.unmannedInfo.targetFollowing.targetID}</span></div>
        <div class="kv"><span class="kv-label">payload</span><span class="kv-value">${statusChip(
          payload.text,
          payload.tone
        )}</span></div>
        <div class="kv"><span class="kv-label">fuelWarn</span><span class="kv-value">${statusChip(
          fuelWarn.text,
          fuelWarn.tone
        )}</span></div>
      </div>
    </div>
  `;
};

const buildPanelContent = (label, simState) => {
  if (!label) {
    return `
      <div class="data-empty">
        <div class="data-empty-title">선택된 비행체 없음</div>
        <div class="data-empty-sub">좌측 버튼에서 비행체를 선택하세요.</div>
      </div>
    `;
  }
  const simEntry = simState?.vehicles?.[label] || null;
  const status = applySimState(buildAgentStatus(label), simEntry);
  const agent = status.agentStateList[0];
  const manned = !agent.isUnmanned;

  return `
    <div class="data-grid">
      ${buildTopCard(status, agent, label)}
      ${buildPositionCard(agent)}
      ${manned ? buildMannedSystemCard(agent) : buildUnmannedSystemCard(agent)}
    </div>
  `;
};

export const init0401Panel = () => {
  const panel = document.getElementById("right-panel");
  if (!panel) {
    return;
  }
  const title = panel.querySelector(".side-panel-title");
  const body = panel.querySelector(".side-panel-body");
  if (!body) {
    return;
  }

  let simState = null;
  let dirty = true;
  let lastTitleText = null;
  let lastBodyHtml = null;

  const isOpen = () => panel.classList.contains("is-open");

  const render = () => {
    const label = getActiveAgent();
    const nextTitleText = label ? `0401 DATA · ${label}` : "0401 DATA";
    const nextBodyHtml = buildPanelContent(label, simState);
    if (title) {
      if (nextTitleText !== lastTitleText) {
        title.textContent = nextTitleText;
      }
    }
    if (nextBodyHtml !== lastBodyHtml) {
      body.innerHTML = nextBodyHtml;
    }
    lastTitleText = nextTitleText;
    lastBodyHtml = nextBodyHtml;
    dirty = false;
  };

  const requestRender = () => {
    dirty = true;
    if (isOpen()) {
      render();
    }
  };

  subscribe(() => {
    requestRender();
  });

  if (window.simClient && typeof window.simClient.subscribe === "function") {
    window.simClient.subscribe((state) => {
      simState = state || null;
      requestRender();
    });
  }

  if (typeof MutationObserver === "function") {
    const observer = new MutationObserver(() => {
      if (isOpen() && dirty) {
        render();
      }
    });
    observer.observe(panel, { attributes: true, attributeFilter: ["class"] });
  }

  if (isOpen()) {
    render();
  } else {
    // Keep the closed panel cold; the observer renders the latest state once
    // when it is opened.
    dirty = true;
  }
};
