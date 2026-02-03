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

  let picking = false;
  let pending = false;
  let lastHover = null;

  const setOpen = (open) => {
    const next = Boolean(open);
    panel.classList.toggle("is-open", next);
    panel.setAttribute("aria-hidden", next ? "false" : "true");
    toggle.setAttribute("aria-expanded", next ? "true" : "false");
    toggle.classList.toggle("is-active", next);
    if (!next) {
      setPicking(false);
    }
  };

  const setPicking = (next) => {
    picking = Boolean(next);
    if (overlay) {
      overlay.classList.toggle("is-active", picking);
      overlay.setAttribute("aria-hidden", picking ? "false" : "true");
    }
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
      canvas.style.cursor = picking ? "crosshair" : "";
    }
    if (picking && type0202) {
      type0202.value = "coord";
      syncType();
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

  const updateOverlay = (lngLat, point) => {
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
    const alt = num(alt0202?.value, 0) ?? 0;
    overlayCoord.textContent = `Lat ${lngLat.lat.toFixed(6)} / Lon ${lngLat.lng.toFixed(6)} / Alt ${Math.round(alt)}`;
  };

  const updateCoordInputs = (lngLat) => {
    if (!lat0202 || !lon0202) {
      return;
    }
    lat0202.value = lngLat.lat.toFixed(6);
    lon0202.value = lngLat.lng.toFixed(6);
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
    if (event.key === "Escape" && picking) {
      setPicking(false);
      logStatus("좌표 선택 취소", { level: "info" });
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

  map.on("mousemove", (event) => {
    if (!picking) {
      return;
    }
    lastHover = event;
    updateOverlay(event.lngLat, event.point);
  });

  map.on("click", async (event) => {
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
