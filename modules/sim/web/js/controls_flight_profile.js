import { logStatus } from "./status_log.js";

export const initFlightProfileControls = () => {
  const toggle = document.getElementById("toggle-shinil-mission-profile");
  const sim = window.simClient || null;
  if (!toggle || !sim) {
    return;
  }

  let requestPending = false;

  const setToggleState = (enabled) => {
    const next = Boolean(enabled);
    toggle.classList.toggle("is-active", next);
    toggle.setAttribute("aria-pressed", next ? "true" : "false");
  };

  const syncFromSnapshot = (state) => {
    if (requestPending || typeof state?.shinilMissionProfileEnabled !== "boolean") {
      return;
    }
    setToggleState(state.shinilMissionProfileEnabled);
  };

  toggle.addEventListener("click", async () => {
    if (requestPending || typeof sim.setShinilMissionProfile !== "function") {
      return;
    }

    const enabled = toggle.getAttribute("aria-pressed") !== "true";
    requestPending = true;
    toggle.disabled = true;
    try {
      const result = await sim.setShinilMissionProfile(enabled);
      if (!result?.ok) {
        return;
      }
      const applied =
        typeof result.shinilMissionProfileEnabled === "boolean"
          ? result.shinilMissionProfileEnabled
          : typeof result.enabled === "boolean"
            ? result.enabled
            : enabled;
      setToggleState(applied);
      logStatus(applied ? "신일 비행·카메라·footprint 특성 모드가 켜졌습니다." : "신일 비행·카메라·footprint 특성 모드가 꺼졌습니다.", {
        level: "success",
        ttlMs: 3000,
      });
    } finally {
      requestPending = false;
      toggle.disabled = false;
    }
  });

  if (typeof sim.subscribe === "function") {
    sim.subscribe(syncFromSnapshot);
  }
  syncFromSnapshot(typeof sim.getState === "function" ? sim.getState() : null);
};
