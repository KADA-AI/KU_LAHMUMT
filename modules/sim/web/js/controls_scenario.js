import { logStatus } from "./status_log.js";

const SEND_CUSTOM_ENDPOINT = "/api/integration/send_custom";
const EPOCH_2000 = Date.UTC(2000, 0, 1, 0, 0, 0, 0);
const DEFAULT_PRIOR_ALT = 1000;
const ENEMY_PICKER_OFFSETS = new Map([
  ["roi", { x: 0, y: -112 }],
  ["1", { x: 86, y: -78 }],
  ["2", { x: 122, y: 0 }],
  ["3", { x: 86, y: 78 }],
  ["4", { x: 0, y: 112 }],
  ["5", { x: -86, y: 78 }],
  ["6", { x: -122, y: 0 }],
  ["0", { x: -86, y: -78 }],
]);

const nowMs2000 = () => Date.now() - EPOCH_2000;

const num = (value, fallback = null) => {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
};

const cleanText = (value, fallback = "") => {
  const text = typeof value === "string" ? value.trim() : "";
  return text || fallback;
};

const sendCustom = async (msgId, body, label) => {
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
    logStatus(`${label || msgId} 전송 완료`, { level: "success" });
    return true;
  } catch (err) {
    logStatus(`${label || msgId} 전송 실패: ${err.message}`, {
      level: "error",
      ttlMs: 6000,
    });
    return false;
  }
};

export const initScenarioPanel = (map) => {
  const toggle = document.getElementById("scenario-toggle");
  const panel = document.getElementById("scenario-panel");
  if (!toggle || !panel || !map) {
    return;
  }

  const overlay = document.getElementById("scenario-overlay");
  const overlayDot = overlay ? overlay.querySelector("[data-scn-dot]") : null;
  const overlayTooltip = overlay ? overlay.querySelector("[data-scn-tooltip]") : null;
  const overlayCoord = overlay ? overlay.querySelector("[data-scn-coord]") : null;
  const overlayTitle = overlay ? overlay.querySelector("[data-scn-title]") : null;
  const overlayHint = overlay ? overlay.querySelector("[data-scn-hint]") : null;

  const source0202 = document.getElementById("scn-0202-source");
  const missionId0202 = document.getElementById("scn-0202-mission-id");
  const type0202 = document.getElementById("scn-0202-type");
  const lat0202 = document.getElementById("scn-0202-lat");
  const lon0202 = document.getElementById("scn-0202-lon");
  const alt0202 = document.getElementById("scn-0202-alt");
  const target0202 = document.getElementById("scn-0202-target-id");
  const pick0202 = document.getElementById("scn-0202-pick");
  const send0202 = document.getElementById("scn-0202-send");
  const hint0202 = document.getElementById("scn-0202-hint");
  const coordBlock = panel.querySelector("[data-scn-coord-block]");
  const targetBlock = panel.querySelector("[data-scn-target-block]");

  const source0801 = document.getElementById("scn-0801-source");
  const input0801 = document.getElementById("scn-0801-input-id");
  const ref0801 = document.getElementById("scn-0801-ref-id");
  const time0801 = document.getElementById("scn-0801-time");
  const send0801 = document.getElementById("scn-0801-send");

  const source0802 = document.getElementById("scn-0802-source");
  const aircraft0802 = document.getElementById("scn-0802-aircraft");
  const type0802 = document.getElementById("scn-0802-type");
  const send0802 = document.getElementById("scn-0802-send");

  const source0803 = document.getElementById("scn-0803-source");
  const send0803Next = document.getElementById("scn-0803-next");
  const send0803Repeat = document.getElementById("scn-0803-repeat");

  const enemyPick = document.getElementById("scn-enemy-pick");
  const enemyClear = document.getElementById("scn-enemy-clear");
  const autoEnemyToggle = document.getElementById("auto-enemy-placement-toggle");
  const enemyHint = document.getElementById("scn-enemy-hint");
  const enemyPicker = document.getElementById("enemy-picker");
  const enemyButtons = enemyPicker
    ? Array.from(enemyPicker.querySelectorAll("[data-enemy-type]"))
    : [];
  const modeChip = document.getElementById("scenario-mode-chip");
  const tabButtons = Array.from(panel.querySelectorAll("[data-scn-tab]"));
  const cards = Array.from(panel.querySelectorAll(".scenario-card"));
  const tabOrder = ["0202", "0801", "0802", "0803", "enemy"];
  const cardByTab = new Map(tabOrder.map((key, index) => [key, cards[index] || null]));

  let picking = false;
  let enemyPicking = false;
  let pending = false;
  let lastHover = null;
  let enemyHover = null;
  let enemyMenuActive = false;
  let enemySelection = null;
  let overlayMode = null;
  let overlayAlt = null;
  let last0202Coord = null;
  let activeTab = "0202";
  let autoEnemyPending = false;

  const autoEnemyStorageKey = "sim.autoTargetPlacement";
  const updateAutoEnemyButton = (enabled) => {
    const next = Boolean(enabled);
    window.simAutoTargetPlacement = next;
    if (!autoEnemyToggle) {
      return;
    }
    autoEnemyToggle.classList.toggle("is-active", next);
    autoEnemyToggle.setAttribute("aria-pressed", next ? "true" : "false");
    autoEnemyToggle.textContent = next ? "적 자동배치 ON" : "적 자동배치 OFF";
    autoEnemyToggle.title = next
      ? "자동 적 배치 ON: 생성 수량을 Area 70% / Line 30%로 배치"
      : "자동 적 배치 OFF: 이미 배치된 적은 유지";
  };

  const setAutoEnemyMode = async (enabled, { notify = false } = {}) => {
    if (autoEnemyPending) {
      return false;
    }
    const next = Boolean(enabled);
    updateAutoEnemyButton(next);
    try {
      window.localStorage.setItem(autoEnemyStorageKey, next ? "1" : "0");
    } catch (err) {
      // Storage is optional.
    }

    const sim = window.simClient;
    if (!sim || typeof sim.setAutoTargetPlacement !== "function") {
      if (notify) {
        logStatus("SIM 자동 적 배치 API unavailable", { level: "warn" });
      }
      return false;
    }
    autoEnemyPending = true;
    const result = await sim.setAutoTargetPlacement(next, true);
    autoEnemyPending = false;
    if (!result?.ok) {
      updateAutoEnemyButton(!next);
      try {
        window.localStorage.setItem(autoEnemyStorageKey, !next ? "1" : "0");
      } catch (err) {
        // Storage is optional.
      }
      return false;
    }
    if (notify) {
      if (next) {
        const placed = Number(result.placed) || 0;
        const targetCount = Number(result.targetRegionCount) || 0;
        const areaCount = Number(result.areaCount) || 0;
        const lineCount = Number(result.lineCount) || 0;
        logStatus(
          placed > 0
            ? `자동 적 배치 ON · ${placed}개 생성 (Area ${areaCount} / Line ${lineCount} / 목표지역 ${targetCount})`
            : "자동 적 배치 ON · 다음 임무 로드 시 적용",
          { level: "success", ttlMs: 4500 },
        );
      } else {
        logStatus("자동 적 배치 OFF · 기존 적은 유지", { level: "info", ttlMs: 3500 });
      }
    }
    return true;
  };

  let initialAutoEnemyMode = false;
  try {
    initialAutoEnemyMode = window.localStorage.getItem(autoEnemyStorageKey) === "1";
  } catch (err) {
    initialAutoEnemyMode = false;
  }
  updateAutoEnemyButton(initialAutoEnemyMode);
  setAutoEnemyMode(initialAutoEnemyMode);

  const overlayTexts = {
    prior: {
      title: "0202 선행임무 정보",
      hint: "맵 클릭: 전송 · 우클릭: 취소",
    },
    enemy: {
      title: "적/ROI 배치",
      hint: "맵 클릭 후 항목 선택 · 우클릭: 취소",
    },
  };

  const setOpen = (open) => {
    const next = Boolean(open);
    panel.classList.toggle("is-open", next);
    panel.setAttribute("aria-hidden", next ? "false" : "true");
    toggle.setAttribute("aria-expanded", next ? "true" : "false");
    toggle.classList.toggle("is-active", next);
    if (!next) {
      setPicking(false);
      setEnemyPicking(false);
    }
    if (next && typeof window.setMissionPanelOpen === "function") {
      window.setMissionPanelOpen(false);
    }
    if (next && typeof window.cancelType1NewTargetInput === "function") {
      window.cancelType1NewTargetInput();
    }
    if (next && typeof window.cancelType1TargetOrderInput === "function") {
      window.cancelType1TargetOrderInput();
    }
    if (next) {
      updateModeChip();
    }
  };

  window.setScenarioPanelOpen = setOpen;

  const setActiveTab = (tabId) => {
    activeTab = tabOrder.includes(tabId) ? tabId : "0202";
    tabButtons.forEach((button) => {
      const selected = button.dataset.scnTab === activeTab;
      button.classList.toggle("is-active", selected);
      button.setAttribute("aria-selected", selected ? "true" : "false");
    });
    cardByTab.forEach((card, key) => {
      if (!card) {
        return;
      }
      const selected = key === activeTab;
      card.classList.toggle("is-active", selected);
      card.classList.toggle("is-hidden", !selected);
    });
  };

  const updateModeChip = () => {
    if (!modeChip) {
      return;
    }
    if (enemyPicking) {
      modeChip.textContent = "Enemy Placement";
      return;
    }
    if (picking) {
      modeChip.textContent = "Map Input";
      return;
    }
    if (activeTab === "0801") {
      modeChip.textContent = "Mission Trigger";
      return;
    }
    if (activeTab === "0802") {
      modeChip.textContent = "Force Command";
      return;
    }
    if (activeTab === "0803") {
      modeChip.textContent = "Execution Control";
      return;
    }
    if (activeTab === "enemy") {
      modeChip.textContent = "Threat Layout";
      return;
    }
    const isCoord = !type0202 || type0202.value === "coord";
    modeChip.textContent = isCoord ? "Coordinate Orientation" : "Target Orientation";
  };

  tabButtons.forEach((button) => {
    button.setAttribute("role", "tab");
    button.addEventListener("click", (event) => {
      event.stopPropagation();
      setActiveTab(button.dataset.scnTab || "0202");
      updateModeChip();
    });
  });

  const applyOverlayMode = (mode) => {
    overlayMode = mode;
    if (overlay) {
      overlay.classList.toggle("is-enemy-mode", mode === "enemy");
    }
    if (!overlayTitle || !overlayHint) {
      return;
    }
    const text = overlayTexts[mode] || {};
    if (text.title) {
      overlayTitle.textContent = text.title;
    }
    if (text.hint) {
      overlayHint.textContent = text.hint;
    }
  };

  const updateOverlayState = () => {
    if (!overlay) {
      return;
    }
    const active = picking || enemyPicking;
    overlay.classList.toggle("is-active", active);
    overlay.setAttribute("aria-hidden", active ? "false" : "true");
    if (!active) {
      overlayMode = null;
    }
  };

  const setPicking = (next) => {
    picking = Boolean(next);
    if (picking) {
      setActiveTab("0202");
      enemyPicking = false;
      enemyMenuActive = false;
      enemySelection = null;
      hideEnemyPicker();
    }
    overlayAlt = enemyPicking ? 0 : null;
    applyOverlayMode(picking ? "prior" : enemyPicking ? "enemy" : null);
    updateOverlayState();
    if (pick0202) {
      pick0202.classList.toggle("is-active", picking);
    }
    if (hint0202) {
      hint0202.textContent = picking
        ? "맵에서 좌표를 선택하세요. 클릭: 전송 · 우클릭: 취소"
        : "좌표 모드에서 Map Input Mode를 누르면 맵에서 바로 선택합니다.";
    }
    const canvas = map.getCanvas();
    if (canvas) {
      canvas.style.cursor = picking || enemyPicking ? "crosshair" : "";
    }
    if (picking && type0202) {
      type0202.value = "coord";
      syncType();
    }
    updateModeChip();
  };

  const hideEnemyPicker = () => {
    if (!enemyPicker) {
      return;
    }
    enemyPicker.classList.remove("is-active");
    enemyPicker.setAttribute("aria-hidden", "true");
    enemyMenuActive = false;
  };

  const setEnemyPicking = (next) => {
    enemyPicking = Boolean(next);
    if (enemyPicking) {
      setActiveTab("enemy");
      setPicking(false);
      overlayAlt = 0;
      applyOverlayMode("enemy");
    } else if (!picking) {
      overlayAlt = null;
      applyOverlayMode(null);
    }
    hideEnemyPicker();
    enemySelection = null;
    updateOverlayState();
    if (enemyPick) {
      enemyPick.classList.toggle("is-active", enemyPicking);
    }
    if (enemyHint) {
      enemyHint.textContent = enemyPicking
        ? "맵에서 좌표를 선택한 뒤 적 또는 ROI를 클릭하세요."
        : "Map Input Mode를 누르면 적/ROI 배치 모드가 시작됩니다.";
    }
    const canvas = map.getCanvas();
    if (canvas) {
      canvas.style.cursor = enemyPicking || picking ? "crosshair" : "";
    }
    updateModeChip();
  };

  const syncType = () => {
    const isCoord = !type0202 || type0202.value === "coord";
    if (coordBlock) {
      coordBlock.classList.toggle("is-hidden", !isCoord);
    }
    if (targetBlock) {
      targetBlock.classList.toggle("is-hidden", isCoord);
    }
    if (!isCoord && picking) {
      setPicking(false);
    }
    updateModeChip();
  };

  const updateOverlay = (lngLat, point, altOverride) => {
    if (!overlay || !overlayCoord) {
      return;
    }
    const x = Math.round(point.x);
    const y = Math.round(point.y);
    overlay.style.setProperty("--spot-x", `${x}px`);
    overlay.style.setProperty("--spot-y", `${y}px`);
    if (overlayDot) {
      overlayDot.style.left = `${x}px`;
      overlayDot.style.top = `${y}px`;
    }
    if (overlayTooltip) {
      overlayTooltip.style.left = `${x}px`;
      overlayTooltip.style.top = `${y}px`;
    }
    const alt =
      Number.isFinite(altOverride)
        ? altOverride
        : overlayAlt !== null
          ? overlayAlt
          : last0202Coord?.alt ?? (num(alt0202?.value, DEFAULT_PRIOR_ALT) ?? DEFAULT_PRIOR_ALT);
    overlayCoord.textContent = `Lat ${lngLat.lat.toFixed(6)} / Lon ${lngLat.lng.toFixed(6)} / Alt ${Math.round(alt)}`;
  };

  const layoutEnemyPicker = () => {
    if (!enemyButtons.length) {
      return;
    }
    const count = enemyButtons.length;
    enemyButtons.forEach((btn, idx) => {
      const typeKey = String(btn.dataset.enemyType || idx).toLowerCase();
      const offset = ENEMY_PICKER_OFFSETS.get(typeKey);
      if (offset) {
        btn.style.transform =
          `translate(-50%, -50%) translate(${offset.x.toFixed(1)}px, ${offset.y.toFixed(1)}px)`;
        return;
      }
      const angle = (idx / count) * Math.PI * 2 - Math.PI / 2;
      const dx = Math.cos(angle) * 112;
      const dy = Math.sin(angle) * 88;
      btn.style.transform = `translate(-50%, -50%) translate(${dx.toFixed(1)}px, ${dy.toFixed(1)}px)`;
    });
  };

  const showEnemyPicker = (point) => {
    if (!enemyPicker) {
      return;
    }
    enemyPicker.style.setProperty("--picker-x", `${Math.round(point.x)}px`);
    enemyPicker.style.setProperty("--picker-y", `${Math.round(point.y)}px`);
    enemyPicker.classList.add("is-active");
    enemyPicker.setAttribute("aria-hidden", "false");
    enemyMenuActive = true;
  };

  const placeEnemy = async (typeValue) => {
    if (!enemySelection || !enemySelection.lngLat) {
      return;
    }
    const typeKey = String(typeValue || "0").toLowerCase();
    if (typeKey === "0") {
      hideEnemyPicker();
      return;
    }
    const sim = window.simClient;
    if (!sim) {
      logStatus("SIM target API unavailable", { level: "warn" });
      hideEnemyPicker();
      return;
    }
    if (typeKey === "roi") {
      if (typeof sim.addRoi !== "function") {
        logStatus("SIM ROI API unavailable", { level: "warn" });
        hideEnemyPicker();
        return;
      }
      const result = await sim.addRoi({
        lat: enemySelection.lngLat.lat,
        lon: enemySelection.lngLat.lng,
        alt: 0,
      });
      if (result && result.ok) {
        const name = result?.roi?.name || "ROI";
        if (result.queued) {
          logStatus(`${name} 임시 저장됨 (임무 로드 후 적용)`, { level: "info", ttlMs: 4000 });
        } else {
          logStatus(`${name} 배치 완료`, { level: "success", ttlMs: 3500 });
        }
      }
      hideEnemyPicker();
      return;
    }
    const typeId = Math.trunc(num(typeKey, 0) || 0);
    if (!sim || typeof sim.addTarget !== "function") {
      logStatus("SIM target API unavailable", { level: "warn" });
      hideEnemyPicker();
      return;
    }
    const payload = {
      type: typeId,
      lat: enemySelection.lngLat.lat,
      lon: enemySelection.lngLat.lng,
      alt: 0,
    };
    const result = await sim.addTarget(payload);
    if (result && result.ok) {
      const name = result?.target?.name;
      if (result.queued) {
        logStatus(
          name ? `${name} 임시 저장됨 (임무 로드 후 적용)` : "적 배치 임시 저장됨",
          { level: "info", ttlMs: 4000 },
        );
      } else {
        logStatus(name ? `${name} 배치 완료` : "적 배치 완료", { level: "success", ttlMs: 3500 });
      }
    }
    hideEnemyPicker();
  };

  const build0202Body = (overrideCoord) => {
    const ts = nowMs2000();
    const source = cleanText(source0202?.value, "DSC");
    const missionId = Math.max(1, Math.trunc(num(missionId0202?.value, 1) || 1));
    if (type0202 && type0202.value === "target") {
      const rawTargetId = num(target0202?.value, null);
      const targetId = Math.trunc(rawTargetId);
      if (!Number.isFinite(rawTargetId) || targetId <= 0) {
        return null;
      }
      return {
        timestamp: ts,
        source,
        priorMissionList: [
          {
            priorMissionID: missionId,
            missionType: 2,
            targetOrientation: { targetID: targetId },
          },
        ],
      };
    }
    const coord =
      overrideCoord ??
      last0202Coord ?? {
        lat: num(lat0202?.value, null),
        lon: num(lon0202?.value, null),
        alt: num(alt0202?.value, DEFAULT_PRIOR_ALT),
      };
    const lat = coord?.lat;
    const lon = coord?.lon;
    const alt = coord?.alt;
    if (!Number.isFinite(lat) || !Number.isFinite(lon)) {
      return null;
    }
    return {
      timestamp: ts,
      source,
      priorMissionList: [
        {
          priorMissionID: missionId,
          missionType: 1,
          coordinateOrientation: {
            coordinate: {
              latitude: Number(lat),
              longitude: Number(lon),
              altitude: Math.trunc(Number.isFinite(alt) ? alt : 0),
            },
          },
        },
      ],
    };
  };

  const bumpPriorMissionId = () => {
    if (!missionId0202) {
      return;
    }
    const current = Math.max(1, Math.trunc(num(missionId0202.value, 1) || 1));
    missionId0202.value = String(current + 1);
  };

  const handleSend0202 = async (overrideCoord) => {
    const body = build0202Body(overrideCoord);
    if (!body) {
      const missingField = type0202?.value === "target" ? "Target ID" : "좌표";
      logStatus(`0202 ${missingField}가 비어 있거나 올바르지 않습니다.`, { level: "warn" });
      return false;
    }
    const ok = await sendCustom("0202", body, "0202");
    if (ok) {
      bumpPriorMissionId();
    }
    return ok;
  };

  const handleSend0801 = async () => {
    const ts = nowMs2000();
    const source = cleanText(source0801?.value, "DSC");
    const opTime = num(time0801?.value, null);
    const body = {
      timestamp: ts,
      source,
      operatorReplanRequestTime: Number.isFinite(opTime) ? Math.trunc(opTime) : ts,
      inputMissionPackageID: Math.trunc(num(input0801?.value, 0) || 0),
      missionReferencePackageID: Math.trunc(num(ref0801?.value, 0) || 0),
    };
    return sendCustom("0801", body, "0801");
  };

  const handleSend0802 = async () => {
    const ts = nowMs2000();
    const source = cleanText(source0802?.value, "DSC");
    const body = {
      timestamp: ts,
      source,
      aircraftID: Math.trunc(num(aircraft0802?.value, 4) || 4),
      mandatoryType: Math.trunc(num(type0802?.value, 1) || 1),
    };
    const sendResult = await sendCustom("0802", body, "0802");
    const sim = window.simClient;
    if (sim && typeof sim.forceCommand === "function") {
      sim.forceCommand({
        aircraftID: body.aircraftID,
        mandatoryType: body.mandatoryType,
      });
    }
    return sendResult;
  };

  const handleSend0803 = async (execute) => {
    const ts = nowMs2000();
    const source = cleanText(source0803?.value, "CSP");
    const body = {
      timestamp: ts,
      source,
      execute: Math.trunc(execute),
    };
    return sendCustom("0803", body, "0803");
  };

  const triggerSimNextMission = () => {
    const sim = window.simClient;
    if (sim && typeof sim.nextMission === "function") {
      sim.nextMission();
    }
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

  document.addEventListener("keydown", (event) => {
    if (event.key === "Escape" && (picking || enemyPicking)) {
      if (picking) {
        setPicking(false);
        logStatus("좌표 선택 취소", { level: "info" });
      }
      if (enemyPicking) {
        setEnemyPicking(false);
        logStatus("적 배치 취소", { level: "info" });
      }
    }
  });

  if (type0202) {
    type0202.addEventListener("change", syncType);
  }
  setActiveTab(activeTab);
  syncType();
  updateModeChip();

  if (pick0202) {
    pick0202.addEventListener("click", () => {
      setPicking(!picking);
    });
  }

  if (enemyPick) {
    enemyPick.addEventListener("click", () => {
      setEnemyPicking(!enemyPicking);
    });
  }

  if (enemyClear) {
    enemyClear.addEventListener("click", async () => {
      const sim = window.simClient;
      if (!sim || typeof sim.clearTargets !== "function") {
        logStatus("SIM target API unavailable", { level: "warn" });
        return;
      }
      const result = await sim.clearTargets();
      if (result && result.ok) {
        if (typeof window.missionTargetLoader === "function") {
          window.missionTargetLoader({ ok: true, targets: [], rois: [] });
        }
        logStatus("적/ROI 배치 초기화 완료", { level: "success", ttlMs: 3500 });
      }
      hideEnemyPicker();
    });
  }

  if (send0202) {
    send0202.addEventListener("click", () => {
      handleSend0202();
    });
  }

  if (send0801) {
    send0801.addEventListener("click", () => {
      handleSend0801();
    });
  }

  if (send0802) {
    send0802.addEventListener("click", () => {
      handleSend0802();
    });
  }

  if (send0803Next) {
    send0803Next.addEventListener("click", () => {
      handleSend0803(1);
    });
  }

  if (send0803Repeat) {
    send0803Repeat.addEventListener("click", () => {
      handleSend0803(2);
    });
  }

  if (enemyButtons.length) {
    layoutEnemyPicker();
    enemyButtons.forEach((btn) => {
      btn.addEventListener("click", (event) => {
        event.preventDefault();
        event.stopPropagation();
        placeEnemy(btn.dataset.enemyType || "0");
      });
    });
  }

  if (autoEnemyToggle) {
    autoEnemyToggle.addEventListener("click", (event) => {
      event.stopPropagation();
      setAutoEnemyMode(!window.simAutoTargetPlacement, { notify: true });
    });
  }

  map.on("mousemove", (event) => {
    if (picking) {
      lastHover = event;
      updateOverlay(event.lngLat, event.point);
      return;
    }
    if (enemyPicking) {
      enemyHover = event;
      updateOverlay(event.lngLat, event.point, 0);
    }
  });

  map.on("click", async (event) => {
    if (enemyPicking) {
      if (event.originalEvent) {
        event.originalEvent.preventDefault();
        event.originalEvent.stopPropagation();
      }
      enemySelection = { lngLat: event.lngLat, point: event.point };
      enemyHover = enemySelection;
      updateOverlay(event.lngLat, event.point, 0);
      showEnemyPicker(event.point);
      return;
    }
    if (!picking || pending) {
      return;
    }
    if (event.originalEvent) {
      event.originalEvent.preventDefault();
      event.originalEvent.stopPropagation();
    }
    pending = true;
    const alt = Number.isFinite(last0202Coord?.alt)
      ? last0202Coord.alt
      : num(alt0202?.value, DEFAULT_PRIOR_ALT) ?? DEFAULT_PRIOR_ALT;
    const override = { lat: event.lngLat.lat, lon: event.lngLat.lng, alt };
    last0202Coord = override;
    const ok = await handleSend0202(override);
    pending = false;
    if (ok) {
      setPicking(false);
    }
  });

  map.on("contextmenu", (event) => {
    if (enemyPicking) {
      if (event.originalEvent) {
        event.originalEvent.preventDefault();
        event.originalEvent.stopPropagation();
      }
      setEnemyPicking(false);
      logStatus("적 배치 취소", { level: "info" });
      return;
    }
    if (!picking) {
      return;
    }
    if (event.originalEvent) {
      event.originalEvent.preventDefault();
      event.originalEvent.stopPropagation();
    }
    setPicking(false);
    logStatus("좌표 선택 취소", { level: "info" });
  });

  if (alt0202) {
    alt0202.addEventListener("input", () => {
      if (picking && lastHover) {
        updateOverlay(lastHover.lngLat, lastHover.point);
      }
    });
  }
};
