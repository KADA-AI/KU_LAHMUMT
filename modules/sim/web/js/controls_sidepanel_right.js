export const initRightSidePanel = () => {
  const toggle = document.getElementById("right-side-toggle");
  const panel = document.getElementById("right-side-panel");
  const autoMissionFlowToggle = document.getElementById("auto-mission-flow-toggle");
  const tabButtons = Array.from(document.querySelectorAll("[data-right-tool-tab]"));
  const tabPanels = Array.from(document.querySelectorAll("[data-right-tool-panel]"));
  if (!toggle || !panel) {
    return;
  }

  const storageKey = "sim.autoMissionFlow";
  const legacyStorageKey = "sim.autoApproveFirstOption";
  const setAutoMissionFlowMode = (enabled) => {
    const next = Boolean(enabled);
    window.simAutoMissionFlow = next;
    // Keep the old flag in sync for any cached SIM page that still reads it.
    window.simAutoApproveFirstOption = next;
    if (autoMissionFlowToggle) {
      autoMissionFlowToggle.classList.toggle("is-active", next);
      autoMissionFlowToggle.setAttribute("aria-pressed", next ? "true" : "false");
      autoMissionFlowToggle.textContent = next ? "자동 진행 ON" : "자동 진행 OFF";
      autoMissionFlowToggle.title = next
        ? "자동 진행 ON: 0701 첫 옵션 선택 + 0503 다음 협업기저임무 즉시 수행"
        : "자동 진행 OFF: 재계획 옵션과 다음 협업기저임무를 수동으로 선택";
    }
    try {
      window.localStorage.setItem(storageKey, next ? "1" : "0");
      window.localStorage.removeItem(legacyStorageKey);
    } catch (err) {
      // Ignore storage failures; the in-memory flag still works for this session.
    }
    window.dispatchEvent(
      new CustomEvent("sim:auto-mission-flow", { detail: { enabled: next } }),
    );
  };

  let initialAutoMissionFlowMode = false;
  try {
    const saved = window.localStorage.getItem(storageKey);
    initialAutoMissionFlowMode =
      saved === null
        ? window.localStorage.getItem(legacyStorageKey) === "1"
        : saved === "1";
  } catch (err) {
    initialAutoMissionFlowMode = false;
  }
  setAutoMissionFlowMode(initialAutoMissionFlowMode);

  if (autoMissionFlowToggle) {
    autoMissionFlowToggle.addEventListener("click", (event) => {
      event.stopPropagation();
      setAutoMissionFlowMode(!window.simAutoMissionFlow);
    });
  }

  const setActiveTab = (tabName) => {
    const next = String(tabName || "io");
    tabButtons.forEach((btn) => {
      const active = btn.dataset.rightToolTab === next;
      btn.classList.toggle("is-active", active);
      btn.setAttribute("aria-selected", active ? "true" : "false");
    });
    tabPanels.forEach((item) => {
      const active = item.dataset.rightToolPanel === next;
      item.classList.toggle("is-active", active);
      item.setAttribute("aria-hidden", active ? "false" : "true");
    });
    try {
      window.localStorage.setItem("sim.rightToolTab", next);
    } catch (err) {
      // Storage is optional.
    }
  };

  tabButtons.forEach((btn) => {
    btn.addEventListener("click", (event) => {
      event.stopPropagation();
      setActiveTab(btn.dataset.rightToolTab || "io");
    });
  });

  let initialTab = "io";
  try {
    initialTab = window.localStorage.getItem("sim.rightToolTab") || "io";
  } catch (err) {
    initialTab = "io";
  }
  setActiveTab(initialTab);

  const setOpen = (open) => {
    const next = Boolean(open);
    panel.classList.toggle("is-open", next);
    panel.setAttribute("aria-hidden", next ? "false" : "true");
    toggle.setAttribute("aria-expanded", next ? "true" : "false");
    toggle.classList.toggle("is-open", next);
    toggle.textContent = next ? ">" : "<";
  };

  toggle.addEventListener("click", (event) => {
    event.stopPropagation();
    setOpen(!panel.classList.contains("is-open"));
  });

  panel.addEventListener("click", (event) => {
    event.stopPropagation();
  });

  document.addEventListener("click", (event) => {
    const target = event.target;
    if (!panel.contains(target) && target !== toggle) {
      setOpen(false);
    }
  });

  setOpen(panel.classList.contains("is-open"));
};
