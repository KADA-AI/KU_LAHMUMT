import { getUiState, updateUiField, setActiveAgent, isManned } from "./agent_store.js";

import { logStatus } from "./status_log.js";

const FLIGHT_MODES = [
  { value: 0, label: "미사용" },
  { value: 1, label: "자동이륙" },
  { value: 2, label: "자동착륙" },
  { value: 3, label: "통제권이양지이동" },
  { value: 4, label: "전술집결지이동" },
  { value: 5, label: "RTB" },
  { value: 6, label: "편대비행" },
  { value: 7, label: "경로이동비행" },
  { value: 8, label: "점항법비행" },
  { value: 9, label: "표적추적비행" },
];

const getLabel = (button) => {
  const data = button.dataset.agent;
  if (data) {
    return data.trim();
  }
  return button.textContent ? button.textContent.trim() : "";
};


const notifyMissionSelection = (label) => {
  if (typeof window.setSelectedAgentPath === "function") {
    window.setSelectedAgentPath(label || null);
  }
};

const getNested = (state, path) => {
  if (!path) {
    return undefined;
  }
  return path.split(".").reduce((acc, key) => (acc ? acc[key] : undefined), state);
};

const createOptionList = (items, current) =>
  items
    .map(
      (item) =>
        `<option value="${item.value}"${item.value === current ? " selected" : ""}>${item.label}</option>`
    )
    .join("");

const renderManned = (state) => `
  <div class="agent-section">
    <div class="agent-section-title">기체 상태</div>
    <div class="agent-field">
      <div class="agent-label">Health</div>
      <div class="segmented" data-field="health" data-columns="3" data-default="1">
        <button type="button" data-value="0">미확인</button>
        <button type="button" data-value="1">정상</button>
        <button type="button" data-value="2">비정상</button>
      </div>
    </div>
    <div class="agent-field">
      <div class="agent-label">연료 소모율</div>
      <div class="stepper" data-field="fuelConsumption">
        <button type="button" class="stepper-btn" data-step="-0.1">-</button>
        <input class="agent-input" type="number" min="0" max="10" step="0.1" data-field="fuelConsumption" data-input="number" />
        <button type="button" class="stepper-btn" data-step="0.1">+</button>
      </div>
    </div>
  </div>
  <div class="agent-section">
    <div class="agent-section-title">데이터링크</div>
    <div class="agent-field">
      <div class="agent-label">UAV1</div>
      <div class="segmented" data-field="datalink.uav1" data-boolean="true" data-columns="2" data-default="1">
        <button type="button" data-value="1">연결</button>
        <button type="button" data-value="0">끊김</button>
      </div>
    </div>
    <div class="agent-field">
      <div class="agent-label">UAV2</div>
      <div class="segmented" data-field="datalink.uav2" data-boolean="true" data-columns="2" data-default="1">
        <button type="button" data-value="1">연결</button>
        <button type="button" data-value="0">끊김</button>
      </div>
    </div>
    <div class="agent-field">
      <div class="agent-label">UAV3</div>
      <div class="segmented" data-field="datalink.uav3" data-boolean="true" data-columns="2" data-default="1">
        <button type="button" data-value="1">연결</button>
        <button type="button" data-value="0">끊김</button>
      </div>
    </div>
  </div>
  <div class="agent-section">
    <div class="agent-section-title">무장 재고</div>
    <div class="agent-field">
      <div class="agent-label">타입 1</div>
      <input class="agent-input" type="number" min="0" max="1000" data-field="weapons.type1" />
    </div>
    <div class="agent-field">
      <div class="agent-label">타입 2</div>
      <input class="agent-input" type="number" min="0" max="1000" data-field="weapons.type2" />
    </div>
    <div class="agent-field">
      <div class="agent-label">타입 3</div>
      <input class="agent-input" type="number" min="0" max="1000" data-field="weapons.type3" />
    </div>
  </div>
`;

const renderUnmanned = (state) => `
  <div class="agent-section">
    <div class="agent-section-title">기체 상태</div>
    <div class="agent-field">
      <div class="agent-label">Health</div>
      <div class="segmented" data-field="health" data-columns="3" data-default="1">
        <button type="button" data-value="0">미확인</button>
        <button type="button" data-value="1">정상</button>
        <button type="button" data-value="2">비정상</button>
      </div>
    </div>
    <div class="agent-field">
      <div class="agent-label">임무장비</div>
      <div class="segmented" data-field="payloadHealth" data-columns="3" data-default="1">
        <button type="button" data-value="0">없음</button>
        <button type="button" data-value="1">정상</button>
        <button type="button" data-value="2">비정상</button>
      </div>
    </div>
    <div class="agent-field">
      <div class="agent-label">연료 경고</div>
      <div class="segmented" data-field="fuelWarning" data-columns="2" data-default="0">
        <button type="button" data-value="0">없음</button>
        <button type="button" data-value="1">양호</button>
        <button type="button" data-value="2">경고</button>
        <button type="button" data-value="3">위험</button>
      </div>
    </div>
    <div class="agent-field">
      <div class="agent-label">연료 소모율</div>
      <div class="stepper" data-field="fuelConsumption">
        <button type="button" class="stepper-btn" data-step="-0.1">-</button>
        <input class="agent-input" type="number" min="0" max="10" step="0.1" data-field="fuelConsumption" data-input="number" />
        <button type="button" class="stepper-btn" data-step="0.1">+</button>
      </div>
    </div>
  </div>
  <div class="agent-section">
    <div class="agent-section-title">이상상태 시험</div>
    <div class="agent-action-note">
      현재 WP 기준 강제 원선회 5바퀴를 발생시켜 재계획 감시 로직을 시험합니다.
    </div>
    <div class="agent-action-row">
      <button type="button" class="agent-action-btn agent-action-btn-danger" data-action="orbit-fault">
        원선회 이상 5바퀴
      </button>
    </div>
  </div>
`;

const overrideTimers = new Map();

const buildOverridePayload = (label) => {
  const state = getUiState(label);
  return {
    agent: label,
    health: Number(state.health),
    fuelConsumption: Number(state.fuelConsumption),
    payloadHealth: Number(state.payloadHealth),
    fuelWarning: Number(state.fuelWarning),
    weapons: {
      type1: Number(state.weapons?.type1 ?? 0),
      type2: Number(state.weapons?.type2 ?? 0),
      type3: Number(state.weapons?.type3 ?? 0),
    },
    datalink: {
      uav1: Boolean(state.datalink?.uav1),
      uav2: Boolean(state.datalink?.uav2),
      uav3: Boolean(state.datalink?.uav3),
    },
  };
};

const queueAgentOverride = (label) => {
  if (!label) {
    return;
  }
  if (!window.simClient || typeof window.simClient.setAgentState !== "function") {
    return;
  }
  const key = String(label);
  if (overrideTimers.has(key)) {
    clearTimeout(overrideTimers.get(key));
  }
  const handle = setTimeout(() => {
    overrideTimers.delete(key);
    window.simClient.setAgentState(buildOverridePayload(label));
  }, 80);
  overrideTimers.set(key, handle);
};

const updateStatusDot = (button, state, manned) => {
  const dot = button.querySelector(".ui-btn-dot");
  if (!dot) {
    return;
  }
  dot.classList.remove("dot-ok", "dot-warn", "dot-bad", "dot-unknown");
  const health = Number(state.health);
  if (health === 2) {
    dot.classList.add("dot-bad");
    return;
  }
  if (!manned && Number(state.payloadHealth) === 2) {
    dot.classList.add("dot-bad");
    return;
  }
  if (!manned && Number(state.fuelWarning) === 3) {
    dot.classList.add("dot-bad");
    return;
  }
  if (!manned && Number(state.fuelWarning) === 2) {
    dot.classList.add("dot-warn");
    return;
  }
  if (manned) {
    const linkDown = !state.datalink.uav1 || !state.datalink.uav2 || !state.datalink.uav3;
    if (linkDown) {
      dot.classList.add("dot-warn");
      return;
    }
  }
  if (health === 1) {
    dot.classList.add("dot-ok");
    return;
  }
  dot.classList.add("dot-unknown");
};

const applyConditionalVisibility = (root, state) => {
  root.querySelectorAll("[data-when]").forEach((el) => {
    const rule = el.dataset.when || "";
    const [field, value] = rule.split(":");
    if (!field || value === undefined) {
      return;
    }
    const current = getNested(state, field);
    const match = String(current) === value;
    el.classList.toggle("is-hidden", !match);
  });
};

const applyDisabledState = (root, state) => {
  const locked = Number(state.health) === 2;
  root.classList.toggle("is-locked", locked);
  root.querySelectorAll("button, input, select").forEach((el) => {
    el.disabled = locked;
  });
};

const normalizeSegmentValue = (group, value) =>
  group.dataset.boolean === "true" ? (value ? 1 : 0) : Number(value);

const getSegmentDefault = (group) =>
  group.dataset.default !== undefined ? Number(group.dataset.default) : null;

const syncSegmented = (root, state) => {
  root.querySelectorAll(".segmented").forEach((group) => {
    const field = group.dataset.field;
    if (!field) {
      return;
    }
    const current = normalizeSegmentValue(group, getNested(state, field));
    const defaultValue = getSegmentDefault(group);
    group.querySelectorAll("button[data-value]").forEach((btn) => {
      const value = Number(btn.dataset.value);
      const isActive = value === Number(current);
      btn.classList.toggle("is-active", isActive);
      if (isActive) {
        const isDefault = defaultValue === null ? true : value === defaultValue;
        btn.classList.toggle("is-ok", isDefault);
        btn.classList.toggle("is-danger", !isDefault);
      } else {
        btn.classList.remove("is-ok", "is-danger");
      }
    });
  });
  applyDisabledState(root, state);
};

const bindSegmented = (root, state, button, label, manned) => {
  root.querySelectorAll(".segmented").forEach((group) => {
    const field = group.dataset.field;
    if (!field || group.dataset.bound === "true") {
      return;
    }
    group.dataset.bound = "true";
    group.querySelectorAll("button[data-value]").forEach((btn) => {
      const value = Number(btn.dataset.value);
      btn.addEventListener("click", () => {
        if (Number(state.health) === 2) {
          return;
        }
        const current = normalizeSegmentValue(group, getNested(state, field));
        const defaultValue = getSegmentDefault(group);
        let nextValue = value;
        if (defaultValue !== null && value === current && value !== defaultValue) {
          nextValue = defaultValue;
        }
        const next = group.dataset.boolean === "true" ? nextValue === 1 : nextValue;
        updateUiField(label, field, next);
        queueAgentOverride(label);
        syncSegmented(root, state);
        applyConditionalVisibility(root, state);
        updateStatusDot(button, state, manned);
      });
    });
  });
  syncSegmented(root, state);
};

const bindSelects = (root, state, button, label, manned) => {
  root.querySelectorAll("select[data-field]").forEach((select) => {
    const field = select.dataset.field;
    select.addEventListener("change", () => {
      if (Number(state.health) === 2) {
        return;
      }
      updateUiField(label, field, Number(select.value));
      queueAgentOverride(label);
      applyConditionalVisibility(root, state);
      updateStatusDot(button, state, manned);
    });
  });
};

const bindNumbers = (root, state, button, label, manned) => {
  root.querySelectorAll("input[type=number][data-field]:not([data-input])").forEach((input) => {
    const field = input.dataset.field;
    const current = getNested(state, field);
    if (current !== undefined) {
      input.value = current;
    }
    input.addEventListener("input", () => {
      if (Number(state.health) === 2) {
        return;
      }
      updateUiField(label, field, Number(input.value));
      queueAgentOverride(label);
      updateStatusDot(button, state, manned);
    });
  });
};

const bindSteppers = (root, state, button, label, manned) => {
  root.querySelectorAll(".stepper").forEach((stepper) => {
    const field = stepper.dataset.field;
    if (!field || stepper.dataset.bound === "true") {
      return;
    }
    stepper.dataset.bound = "true";
    const input = stepper.querySelector("input[data-input=\"number\"]");
    const buttons = stepper.querySelectorAll(".stepper-btn");
    if (input) {
      const current = getNested(state, field);
      if (current !== undefined) {
        input.value = current;
      }
      input.addEventListener("input", () => {
        if (Number(state.health) === 2) {
          return;
        }
        updateUiField(label, field, Number(input.value));
        queueAgentOverride(label);
        updateStatusDot(button, state, manned);
      });
    }
    buttons.forEach((btn) => {
      const step = Number(btn.dataset.step);
      btn.addEventListener("click", () => {
        if (Number(state.health) === 2) {
          return;
        }
        const current = Number(getNested(state, field) || 0);
        const next = Math.max(0, Math.min(10, Number((current + step).toFixed(1))));
        updateUiField(label, field, next);
        queueAgentOverride(label);
        if (input) {
          input.value = next;
        }
        updateStatusDot(button, state, manned);
      });
    });
  });
};

const bindActions = (root, state, label) => {
  root.querySelectorAll("button[data-action]").forEach((btn) => {
    if (btn.dataset.bound === "true") {
      return;
    }
    btn.dataset.bound = "true";
    btn.addEventListener("click", async () => {
      if (Number(state.health) === 2) {
        return;
      }
      const action = btn.dataset.action;
      if (action !== "orbit-fault") {
        return;
      }
      if (!window.simClient || typeof window.simClient.setAgentState !== "function") {
        logStatus("SIM agent update unavailable", { level: "error", ttlMs: 4000 });
        return;
      }
      btn.disabled = true;
      try {
        const result = await window.simClient.setAgentState({
          agent: label,
          orbitFault: { circles: 5 },
        });
        if (!result || !result.ok) {
          throw new Error((result && result.error) || "orbit fault request failed");
        }
        logStatus(`${label} 원선회 이상 5바퀴 시작`, {
          key: `orbit-fault-${label}`,
          level: "warn",
          ttlMs: 4500,
        });
      } catch (err) {
        logStatus(`${label} 원선회 이상 시작 실패: ${err.message}`, {
          key: `orbit-fault-${label}`,
          level: "error",
          ttlMs: 5000,
        });
      } finally {
        btn.disabled = Number(state.health) === 2;
      }
    });
  });
};

const renderPanelBody = (bodyEl, state, manned) => {
  bodyEl.innerHTML = manned ? renderManned(state) : renderUnmanned(state);
};

export const initAgentPanel = () => {
  const agentButtons = Array.from(document.querySelectorAll(".ui-btn-text"));
  const agentPanel = document.getElementById("agent-panel");
  const agentPanelTitle = document.getElementById("agent-panel-title");
  const agentPanelClose = document.getElementById("agent-panel-close");
  const agentPanelBody = agentPanel ? agentPanel.querySelector(".agent-panel-body") : null;
  let activeAgentButton = null;
  let lastSelectedLabel = null;
  let agentSwitchTimer = null;
  let agentSwitchEndTimer = null;
  let ignoreOutsideClick = false;

  if (!agentPanel || !agentPanelBody || agentButtons.length === 0) {
    return;
  }

  const setActiveAgentButton = (button) => {
    agentButtons.forEach((btn) => btn.classList.toggle("is-active", btn === button));
    activeAgentButton = button;
  };

  const setAgentPanelTitle = (label, manned) => {
    if (!agentPanelTitle) {
      return;
    }
    agentPanelTitle.textContent = `${label} · ${manned ? "유인기" : "무인기"}`;
  };

  const setAgentPanelArrow = (button) => {
    const panelRect = agentPanel.getBoundingClientRect();
    const buttonRect = button.getBoundingClientRect();
    const arrowTop = buttonRect.top + buttonRect.height / 2 - panelRect.top;
    agentPanel.style.setProperty("--agent-arrow-top", `${arrowTop}px`);
  };

  const bindPanelControls = (button, label) => {
    const manned = isManned(label);
    const state = getUiState(label);
    renderPanelBody(agentPanelBody, state, manned);
    applyConditionalVisibility(agentPanelBody, state);
    applyDisabledState(agentPanelBody, state);
    bindSegmented(agentPanelBody, state, button, label, manned);
    bindSelects(agentPanelBody, state, button, label, manned);
    bindNumbers(agentPanelBody, state, button, label, manned);
    bindSteppers(agentPanelBody, state, button, label, manned);
    bindActions(agentPanelBody, state, label);
    updateStatusDot(button, state, manned);
  };

  const open0401Panel = (options) => {
    if (typeof window.open0401Panel === "function") {
      window.open0401Panel(options);
    }
  };

  const closeAgentPanel = () => {
    if (agentSwitchTimer) {
      clearTimeout(agentSwitchTimer);
      agentSwitchTimer = null;
    }
    if (agentSwitchEndTimer) {
      clearTimeout(agentSwitchEndTimer);
      agentSwitchEndTimer = null;
    }
    agentPanel.classList.remove("is-open");
    agentPanel.classList.remove("is-switching");
    agentPanel.setAttribute("aria-hidden", "true");
    agentButtons.forEach((button) => button.classList.remove("is-active"));
    activeAgentButton = null;
    setActiveAgent(null);
    notifyMissionSelection(null);
  };

  const openAgentPanel = (button) => {
    const label = getLabel(button);
    const manned = isManned(label);
    setAgentPanelTitle(label, manned);
    setAgentPanelArrow(button);
    setActiveAgentButton(button);
    agentPanel.classList.remove("is-switching");
    agentPanel.classList.add("is-open");
    agentPanel.setAttribute("aria-hidden", "false");
    setActiveAgent(label);
    notifyMissionSelection(label);
    bindPanelControls(button, label);
  };

  const switchAgentPanel = (button) => {
    if (activeAgentButton === button) {
      openAgentPanel(button);
      return;
    }
    if (!agentPanel.classList.contains("is-open")) {
      openAgentPanel(button);
      return;
    }
    if (agentSwitchTimer) {
      clearTimeout(agentSwitchTimer);
    }
    if (agentSwitchEndTimer) {
      clearTimeout(agentSwitchEndTimer);
    }
    const label = getLabel(button);
    const manned = isManned(label);
    setActiveAgentButton(button);
    setAgentPanelArrow(button);
    agentPanel.classList.add("is-switching");
    setActiveAgent(label);
    notifyMissionSelection(label);
    agentSwitchTimer = setTimeout(() => {
      setAgentPanelTitle(label, manned);
      bindPanelControls(button, label);
    }, 120);
    agentSwitchEndTimer = setTimeout(() => {
      agentPanel.classList.remove("is-switching");
    }, 220);
  };

  const selectAgent = (label, options = {}) => {
    const nextLabel = String(label || "").trim();
    if (!nextLabel) {
      return;
    }
    lastSelectedLabel = nextLabel;
    const button = agentButtons.find((btn) => getLabel(btn) === nextLabel);
    if (!button) {
      return;
    }
    const { flyTo = true } = options || {};
    if (flyTo && typeof window.flyToAgent === "function") {
      window.flyToAgent(nextLabel);
    }
    if (options && options.source === "map") {
      ignoreOutsideClick = true;
    }
    switchAgentPanel(button);
    open0401Panel(options);
  };

  window.selectAgent = selectAgent;
  window.openAgentPanel = (label) => {
    const nextLabel = String(label || lastSelectedLabel || "").trim();
    if (!nextLabel) {
      return;
    }
    const button = agentButtons.find((btn) => getLabel(btn) === nextLabel);
    if (!button) {
      return;
    }
    switchAgentPanel(button);
  };

  agentButtons.forEach((button) => {
    const label = getLabel(button);
    const manned = isManned(label);
    updateStatusDot(button, getUiState(label), manned);
    button.addEventListener("click", (event) => {
      event.stopPropagation();
      selectAgent(label, { flyTo: true });
    });
  });

  if (agentPanelClose) {
    agentPanelClose.addEventListener("click", (event) => {
      event.stopPropagation();
      closeAgentPanel();
    });
  }

  document.addEventListener("click", (event) => {
    const target = event.target;
    if (!agentPanel.classList.contains("is-open")) {
      return;
    }
    if (ignoreOutsideClick) {
      ignoreOutsideClick = false;
      return;
    }
    const clickedButton = agentButtons.some((btn) => btn.contains(target));
    const clickedSidePanel = Boolean(target.closest("#right-panel"));
    const clickedSideToggle = Boolean(target.closest("#right-panel-toggle"));
    if (!agentPanel.contains(target) && !clickedButton && !clickedSidePanel && !clickedSideToggle) {
      closeAgentPanel();
    }
  });

  window.addEventListener("resize", () => {
    if (activeAgentButton) {
      openAgentPanel(activeAgentButton);
    }
  });
};
