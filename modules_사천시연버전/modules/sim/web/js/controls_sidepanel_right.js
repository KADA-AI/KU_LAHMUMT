export const initRightSidePanel = () => {
  const toggle = document.getElementById("right-side-toggle");
  const panel = document.getElementById("right-side-panel");
  const autoOptionToggle = document.getElementById("auto-option-approve-toggle");
  const tabButtons = Array.from(document.querySelectorAll("[data-right-tool-tab]"));
  const tabPanels = Array.from(document.querySelectorAll("[data-right-tool-panel]"));
  if (!toggle || !panel) {
    return;
  }

  const storageKey = "sim.autoApproveFirstOption";
  const setAutoOptionMode = (enabled) => {
    const next = Boolean(enabled);
    window.simAutoApproveFirstOption = next;
    if (autoOptionToggle) {
      autoOptionToggle.classList.toggle("is-active", next);
      autoOptionToggle.setAttribute("aria-pressed", next ? "true" : "false");
      autoOptionToggle.textContent = next ? "Auto Opt1 ON" : "Auto Opt1 OFF";
      autoOptionToggle.title = next
        ? "Auto approve first 0701 option is ON"
        : "Auto approve first 0701 option is OFF";
    }
    try {
      window.localStorage.setItem(storageKey, next ? "1" : "0");
    } catch (err) {
      // Ignore storage failures; the in-memory flag still works for this session.
    }
    window.dispatchEvent(
      new CustomEvent("sim:auto-approve-first-option", { detail: { enabled: next } }),
    );
  };

  let initialAutoOptionMode = false;
  try {
    initialAutoOptionMode = window.localStorage.getItem(storageKey) === "1";
  } catch (err) {
    initialAutoOptionMode = false;
  }
  setAutoOptionMode(initialAutoOptionMode);

  if (autoOptionToggle) {
    autoOptionToggle.addEventListener("click", (event) => {
      event.stopPropagation();
      setAutoOptionMode(!window.simAutoApproveFirstOption);
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
