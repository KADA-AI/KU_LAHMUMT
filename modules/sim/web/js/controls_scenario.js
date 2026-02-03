import { logStatus } from "./status_log.js";

const SEND_CUSTOM_ENDPOINT = "/api/integration/send_custom";
const EPOCH_2000 = Date.UTC(2000, 0, 1, 0, 0, 0, 0);

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
  const enemyHint = document.getElementById("scn-enemy-hint");
  const enemyPicker = document.getElementById("enemy-picker");
  const enemyButtons = enemyPicker
    ? Array.from(enemyPicker.querySelectorAll("[data-enemy-type]"))
    : [];

  let picking = false;
  let enemyPicking = false;
  let pending = false;
  let lastHover = null;
  let enemyHover = null;
  let enemyMenuActive = false;
  let enemySelection = null;
  let overlayMode = null;
  let overlayAlt = null;

  const overlayTexts = {
    prior: {
      title: "0202 선행임무 정보",
      hint: "맵 클릭: 전송 · 우클릭: 취소",
    },
    enemy: {
      title: "적 배치",
      hint: "맵 클릭 후 번호 선택 · 우클릭: 취소",
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
  };

  window.setScenarioPanelOpen = setOpen;

  const applyOverlayMode = (mode) => {
    overlayMode = mode;
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
        ? "맵에서 좌표를 선택한 뒤 번호를 클릭하세요."
        : "Map Input Mode를 누르면 적 배치 모드가 시작됩니다.";
    }
    const canvas = map.getCanvas();
    if (canvas) {
      canvas.style.cursor = enemyPicking || picking ? "crosshair" : "";
    }
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
          : num(alt0202?.value, 0) ?? 0;
    overlayCoord.textContent = `Lat ${lngLat.lat.toFixed(6)} / Lon ${lngLat.lng.toFixed(6)} / Alt ${Math.round(alt)}`;
  };

  const updateCoordInputs = (lngLat) => {
    if (!lat0202 || !lon0202) {
      return;
    }
    lat0202.value = lngLat.lat.toFixed(6);
    lon0202.value = lngLat.lng.toFixed(6);
  };

  const layoutEnemyPicker = () => {
    if (!enemyButtons.length) {
      return;
    }
    const radius = 64;
    const count = enemyButtons.length;
    enemyButtons.forEach((btn, idx) => {
      const angle = (idx / count) * Math.PI * 2 - Math.PI / 2;
      const dx = Math.cos(angle) * radius;
      const dy = Math.sin(angle) * radius;
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

  const placeEnemy = async (typeId) => {
    if (!enemySelection || !enemySelection.lngLat) {
      return;
    }
    if (typeId === 0) {
      hideEnemyPicker();
      return;
    }
    const sim = window.simClient;
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
      const targetId = Math.trunc(num(target0202?.value, 0) || 0);
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
    const lat = overrideCoord?.lat ?? num(lat0202?.value, null);
    const lon = overrideCoord?.lon ?? num(lon0202?.value, null);
    const alt = overrideCoord?.alt ?? num(alt0202?.value, 0);
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

  const handleSend0202 = async (overrideCoord) => {
    const body = build0202Body(overrideCoord);
    if (!body) {
      logStatus("0202 좌표가 비어 있습니다.", { level: "warn" });
      return false;
    }
    return sendCustom("0202", body, "0202");
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
    return sendCustom("0802", body, "0802");
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
  syncType();

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
          window.missionTargetLoader({ ok: true, targets: [] });
        }
        logStatus("적 배치 초기화 완료", { level: "success", ttlMs: 3500 });
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
      triggerSimNextMission();
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
        const typeId = Math.trunc(num(btn.dataset.enemyType, 0) || 0);
        placeEnemy(typeId);
      });
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
    const alt = num(alt0202?.value, 0) ?? 0;
    const override = { lat: event.lngLat.lat, lon: event.lngLat.lng, alt };
    updateCoordInputs(event.lngLat);
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
