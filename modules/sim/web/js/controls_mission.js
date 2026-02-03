import { logStatus } from "./status_log.js";

export const initMissionPanel = () => {
  const toggle = document.getElementById("mission-toggle");
  const panel = document.getElementById("mission-panel");
  const status = panel ? panel.querySelector("[data-mission-status]") : null;
  const statusText = status ? status.querySelector(".mission-status-text") : null;
  const loadBtn = document.getElementById("mission-load");
  const folderInput = document.getElementById("mission-folder-input");
  const seedSelect = document.getElementById("multi-seed-select");
  const seedModeInputs = Array.from(
    document.querySelectorAll('input[name="multi-seed-mode"]'),
  );
  if (!toggle || !panel) {
    return;
  }

  const setStatusMessage = (message) => {
    if (!message) {
      return;
    }
    logStatus(message, { ttlMs: 4000 });
  };

  const setMissionReady = (ready) => {
    if (!status) {
      return;
    }
    const next = Boolean(ready);
    status.classList.toggle("is-ready", next);
    if (statusText) {
      statusText.textContent = next ? "완료" : "대기";
    }
  };

  const setOpen = (open) => {
    const next = Boolean(open);
    panel.classList.toggle("is-open", next);
    panel.setAttribute("aria-hidden", next ? "false" : "true");
    toggle.setAttribute("aria-expanded", next ? "true" : "false");
    toggle.classList.toggle("is-active", next);
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

  const parseFilesToFeatures = async (files) => {
    const features = [];
    let featureId = 1;
    const agents = {};
    const flightPaths = [];
    const missionPlans = [];
    const missionPlanOptions = [];
    const individualPlans = [];
    const inputPlans = [];
    const normalize = (value) => (value === null || value === undefined ? null : Number(value));
    const getCoord = (item) => {
      const coord = item?.coordinate || item?.Coordinate;
      if (!coord) return null;
      const lat = coord.latitude ?? coord.Latitude;
      const lon = coord.longitude ?? coord.Longitude;
      const alt = coord.altitude ?? coord.Altitude;
      if (lat === undefined || lon === undefined) return null;
      return { lat: Number(lat), lon: Number(lon), alt: alt !== undefined ? Number(alt) : null };
    };
    const getWaypoints = (data) =>
      data?.lahWaypointList || data?.uavWaypointList || data?.waypointList || [];
    const orderWaypoints = (raw) => {
      if (!Array.isArray(raw) || raw.length < 2) {
        return raw;
      }
      const byId = new Map();
      const nextIds = new Set();
      raw.forEach((wp) => {
        if (!wp || typeof wp !== "object") return;
        const wid = wp.waypointID ?? wp.WaypointID;
        if (!Number.isFinite(Number(wid))) return;
        const id = Number(wid);
        byId.set(id, wp);
        const nextId = wp.nextWaypointID ?? wp.NextWaypointID;
        if (Number.isFinite(Number(nextId)) && Number(nextId) > 0) {
          nextIds.add(Number(nextId));
        }
      });
      if (!byId.size) {
        return raw;
      }
      let startId = null;
      byId.forEach((_value, key) => {
        if (startId !== null) return;
        if (!nextIds.has(key)) {
          startId = key;
        }
      });
      const ordered = [];
      const visited = new Set();
      if (startId !== null) {
        let curr = startId;
        while (curr && byId.has(curr) && !visited.has(curr)) {
          const wp = byId.get(curr);
          ordered.push(wp);
          visited.add(curr);
          const nextId = wp.nextWaypointID ?? wp.NextWaypointID;
          const nextVal = Number(nextId);
          if (!Number.isFinite(nextVal) || nextVal === 0) {
            break;
          }
          curr = nextVal;
        }
      }
      raw.forEach((wp) => {
        if (!wp || typeof wp !== "object") {
          ordered.push(wp);
          return;
        }
        const wid = wp.waypointID ?? wp.WaypointID;
        const id = Number(wid);
        if (!Number.isFinite(id) || !visited.has(id)) {
          ordered.push(wp);
        }
      });
      return ordered;
    };
    const agentLabel = (aircraftId) => {
      const id = Number(aircraftId);
      if (id >= 1 && id <= 3) return `LAH${id}`;
      if (id >= 4 && id <= 6) return `UAV${id - 3}`;
      return `AC${id}`;
    };

    for (const file of files) {
      const rel = (file.webkitRelativePath || file.name || "").replace(/\\/g, "/");
      const lower = rel.toLowerCase();
      if (!lower.endsWith(".json")) {
        continue;
      }
      let data = null;
      try {
        const text = await file.text();
        data = JSON.parse(text);
      } catch (err) {
        continue;
      }
      if (lower.includes("/missionplanoptioninfo/")) {
        missionPlanOptions.push(data);
        continue;
      }
      if (lower.includes("/missionplan/")) {
        missionPlans.push(data);
        continue;
      }
      if (lower.includes("/individualmissionplan/")) {
        individualPlans.push(data);
        continue;
      }
      if (lower.includes("/inputmissionplan/")) {
        inputPlans.push(data);
        continue;
      }
      if (!lower.includes("/flightpath/")) {
        continue;
      }

      const waypoints = orderWaypoints(getWaypoints(data));
      if (!Array.isArray(waypoints) || waypoints.length < 2) {
        continue;
      }
      flightPaths.push(data);
      const coords = [];
      const alts = [];
      for (const wp of waypoints) {
        const coord = getCoord(wp || {});
        if (!coord) continue;
        coords.push([coord.lon, coord.lat]);
        if (coord.alt !== null && !Number.isNaN(coord.alt)) {
          alts.push(coord.alt);
        } else {
          alts.push(null);
        }
      }
      if (coords.length < 2) {
        continue;
      }
      const aircraftId = data?.aircraftID ?? data?.AircraftID ?? null;
      const pathId = data?.pathID ?? data?.PathID ?? file.name;
      const agent = agentLabel(aircraftId);
      agents[agent] = (agents[agent] || 0) + 1;
      features.push({
        id: featureId++,
        agent,
        aircraftId: normalize(aircraftId),
        pathId,
        points: coords.length,
        coords,
        alts,
        altMin: alts.some((v) => v !== null)
          ? Math.min(...alts.filter((v) => v !== null))
          : null,
        altMax: alts.some((v) => v !== null)
          ? Math.max(...alts.filter((v) => v !== null))
          : null,
      });
    }

    const buildMissionOrder = () => {
      if (!missionPlans.length || !individualPlans.length || !flightPaths.length) {
        return null;
      }
      let selectedMissionPlan = null;
      if (missionPlanOptions.length) {
        const optionInfo = missionPlanOptions.reduce((best, item) => {
          if (!best) return item;
          const tsA = Number(best.timestamp) || 0;
          const tsB = Number(item.timestamp) || 0;
          return tsB >= tsA ? item : best;
        }, null);
        const options = optionInfo?.optionList || [];
        const recommended = options.find((opt) => opt?.recommend);
        const planId = recommended?.missionPlanID;
        selectedMissionPlan = missionPlans.find(
          (plan) => Number(plan?.missionPlanID) === Number(planId),
        );
      }
      if (!selectedMissionPlan) {
        selectedMissionPlan = missionPlans.reduce((best, item) => {
          if (!best) return item;
          const tsA = Number(best.timestamp) || Number(best.missionPlanTimestamp) || 0;
          const tsB = Number(item.timestamp) || Number(item.missionPlanTimestamp) || 0;
          return tsB >= tsA ? item : best;
        }, null);
      }
      if (!selectedMissionPlan) {
        return null;
      }

      const inputPlan = inputPlans.reduce((best, item) => {
        if (!best) return item;
        const tsA = Number(best.timestamp) || 0;
        const tsB = Number(item.timestamp) || 0;
        return tsB >= tsA ? item : best;
      }, null);
      const inputOrder = new Map();
      const inputList = inputPlan?.inputMissionList || [];
      if (Array.isArray(inputList)) {
        inputList.forEach((item, idx) => {
          const id = Number(item?.inputMissionID);
          if (Number.isFinite(id)) {
            inputOrder.set(id, idx);
          }
        });
      }

      const missionOrder = {};
      const aircraftList = selectedMissionPlan?.aircraftList || [];
      aircraftList.forEach((air) => {
        const aircraftId = Number(air?.aircraftID);
        const pkgId = Number(air?.individualMissionPackageID);
        if (!Number.isFinite(aircraftId) || !Number.isFinite(pkgId)) {
          return;
        }
        const plan = individualPlans.find(
          (entry) => Number(entry?.individualMissionPackageID) === pkgId,
        );
        if (!plan) {
          return;
        }
        const list = Array.isArray(plan.individualMissionList)
          ? plan.individualMissionList.slice()
          : [];
        const ordered = list
          .map((mission, idx) => {
            const inputId = Number(mission?.relatedMission?.inputMissionID);
            const orderIdx = inputOrder.has(inputId) ? inputOrder.get(inputId) : idx;
            return { mission, orderIdx };
          })
          .sort((a, b) => (a.orderIdx ?? 0) - (b.orderIdx ?? 0))
          .map((item) => Number(item.mission?.pathID))
          .filter((id) => Number.isFinite(id));
        if (ordered.length) {
          missionOrder[aircraftId] = ordered;
        }
      });

      return Object.keys(missionOrder).length ? missionOrder : null;
    };

    const missionOrder = buildMissionOrder();
    return {
      ok: true,
      features,
      agents,
      count: features.length,
      flightPaths,
      missionOrder,
      inputMissionPlans: inputPlans,
      individualMissionPlans: individualPlans,
    };
  };

  const parseMissionReference = async (files) => {
    let best = null;
    for (const file of files) {
      const rel = (file.webkitRelativePath || file.name || "").replace(/\\/g, "/");
      if (!rel.toLowerCase().includes("/missionreferenceinfo/")) {
        continue;
      }
      if (!rel.toLowerCase().endsWith(".json")) {
        continue;
      }
      let data = null;
      try {
        const text = await file.text();
        data = JSON.parse(text);
      } catch (err) {
        continue;
      }
      const list = data?.takeOverInfoList;
      if (!Array.isArray(list) || !list.length) {
        continue;
      }
      const ts = Number(data.timestamp) || 0;
      if (!best || ts >= best.timestamp) {
        best = { timestamp: ts, data };
      }
    }
    if (!best) {
      return { ok: false, vehicles: {} };
    }
    const vehicles = {};
    const list = best.data.takeOverInfoList || [];
    const toCoord = (entry) => {
      const coord = entry?.coordinate || entry?.Coordinate;
      if (!coord) return null;
      const lat = coord.latitude ?? coord.Latitude;
      const lon = coord.longitude ?? coord.Longitude;
      const alt = coord.altitude ?? coord.Altitude;
      if (!Number.isFinite(Number(lat)) || !Number.isFinite(Number(lon))) {
        return null;
      }
      return { lat: Number(lat), lon: Number(lon), alt: Number(alt) || 0 };
    };
    list.forEach((entry) => {
      const id = Number(entry?.aircraftID ?? entry?.AircraftID);
      if (!Number.isFinite(id) || id < 4 || id > 6) {
        return;
      }
      const coord = toCoord(entry);
      if (!coord) {
        return;
      }
      const agent = `UAV${id - 3}`;
      vehicles[agent] = coord;
    });
    const deltaLat = -300 / 111320;
    ["UAV1", "UAV2", "UAV3"].forEach((uav, index) => {
      const base = vehicles[uav];
      if (!base) {
        return;
      }
      vehicles[`LAH${index + 1}`] = {
        lat: base.lat + deltaLat,
        lon: base.lon,
        alt: base.alt,
      };
    });
    return { ok: true, vehicles, takeOverInfoList: best.data.takeOverInfoList || [] };
  };

  if (loadBtn) {
    loadBtn.addEventListener("click", () => {
      if (folderInput) {
        folderInput.value = "";
        folderInput.click();
      } else {
        setStatusMessage("Folder picker unavailable");
      }
    });
  }

  if (folderInput) {
    folderInput.addEventListener("change", async (event) => {
      const files = Array.from(event.target.files || []);
      if (!files.length) {
        return;
      }
      if (loadBtn) {
        loadBtn.disabled = true;
      }
      setStatusMessage("Loading mission...");
      try {
        const [data, reference] = await Promise.all([
          parseFilesToFeatures(files),
          parseMissionReference(files),
        ]);
        if (typeof window.missionPathLoader === "function") {
          window.missionPathLoader(data);
        }
        if (typeof window.missionVehicleLoader === "function") {
          window.missionVehicleLoader(reference);
        }
        if (data && Array.isArray(data.flightPaths) && data.flightPaths.length > 0) {
          if (window.simClient && typeof window.simClient.setMission === "function") {
            window.simClient.setMission({
              flightPaths: data.flightPaths,
              missionOrder: data.missionOrder || null,
              inputMissionPlans: data.inputMissionPlans || [],
              individualMissionPlans: data.individualMissionPlans || [],
              takeOverInfoList: reference?.takeOverInfoList || [],
            });
          }
        }
        if (data.ok && data.count) {
          setMissionReady(true);
          setStatusMessage(`Mission loaded (${data.count})`);
        } else {
          setMissionReady(false);
          setStatusMessage("No FlightPath data found");
        }
      } catch (err) {
        setMissionReady(false);
        setStatusMessage("Mission load failed");
      } finally {
        if (loadBtn) {
          loadBtn.disabled = false;
        }
      }
    });
  }

  setMissionReady(false);
  window.setMissionPlanReady = setMissionReady;

  if (seedSelect && seedModeInputs.length) {
    const syncSeedMode = () => {
      const active = seedModeInputs.find((input) => input.checked);
      const fixed = active && active.value === "fixed";
      seedSelect.disabled = !fixed;
    };
    seedModeInputs.forEach((input) => {
      input.addEventListener("change", syncSeedMode);
    });
    syncSeedMode();
  }
};
