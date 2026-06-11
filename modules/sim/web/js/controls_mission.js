import { logStatus } from "./status_log.js";

export const initMissionPanel = () => {
  const toggle = document.getElementById("mission-toggle");
  const panel = document.getElementById("mission-panel");
  const status = panel ? panel.querySelector("[data-mission-status]") : null;
  const statusText = status ? status.querySelector(".mission-status-text") : null;
  const loadBtn = document.getElementById("mission-load");
  const reissueInput0201Btn = document.getElementById("mission-reinput-0201");
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
    if (next && typeof window.setScenarioPanelOpen === "function") {
      window.setScenarioPanelOpen(false);
    }
  };

  window.setMissionPanelOpen = setOpen;

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
    const getLineSearch = (wp) => {
      const filming = wp?.filmingProperty || wp?.FilmingProperty || null;
      if (!filming || typeof filming !== "object") {
        return null;
      }
      const lineSearch = filming.lineSearch || filming.LineSearch || null;
      return lineSearch && typeof lineSearch === "object" ? lineSearch : null;
    };
    const getSearchCoord = (item) => {
      if (!item || typeof item !== "object") {
        return null;
      }
      const lat = Number(item.latitude ?? item.Latitude);
      const lon = Number(item.longitude ?? item.Longitude);
      if (
        !Number.isFinite(lat) ||
        !Number.isFinite(lon) ||
        lat < -90 ||
        lat > 90 ||
        lon < -180 ||
        lon > 180
      ) {
        return null;
      }
      return { lat, lon };
    };
    const extractSweepLinesFromPath = (path) => {
      const lines = [];
      const waypoints = orderWaypoints(getWaypoints(path));
      if (!Array.isArray(waypoints)) {
        return lines;
      }
      waypoints.forEach((wp) => {
        const lineSearch = getLineSearch(wp);
        const coordinateList = Array.isArray(lineSearch?.coordinateList)
          ? lineSearch.coordinateList
          : Array.isArray(lineSearch?.CoordinateList)
            ? lineSearch.CoordinateList
            : [];
        const points = coordinateList.map(getSearchCoord).filter(Boolean);
        if (points.length < 2) {
          return;
        }
        const chunkSize = Math.trunc(
          Number(
            lineSearch?.interpolationPoints ??
              lineSearch?.InterpolationPoints ??
              lineSearch?.interpolationPoint ??
              lineSearch?.InterpolationPoint ??
              0,
          ),
        );
        if (chunkSize > 2 && points.length > chunkSize) {
          for (let start = 0; start < points.length; start += chunkSize) {
            const chunk = points.slice(start, start + chunkSize);
            if (chunk.length >= 2) {
              lines.push(chunk);
            }
          }
          return;
        }
        lines.push(points);
      });
      return lines;
    };
    const buildPathMissionIndex = () => {
      const index = {};
      individualPlans.forEach((plan) => {
        if (!plan || typeof plan !== "object") {
          return;
        }
        const aircraftId = Number(plan.aircraftID ?? plan.AircraftID);
        const missions = Array.isArray(plan.individualMissionList) ? plan.individualMissionList : [];
        missions.forEach((mission) => {
          const pathId = Number(mission?.pathID ?? mission?.PathID);
          if (!Number.isFinite(pathId)) {
            return;
          }
          const related = mission?.relatedMission || mission?.RelatedMission || {};
          const inputMissionId = Number(
            related.inputMissionID ??
              related.InputMissionID ??
              mission.inputMissionID ??
              mission.InputMissionID,
          );
          const individualMissionId = Number(
            mission.individualMissionID ?? mission.IndividualMissionID,
          );
          index[String(Math.trunc(pathId))] = {
            pathID: Math.trunc(pathId),
            aircraftID: Number.isFinite(aircraftId) ? Math.trunc(aircraftId) : null,
            inputMissionID: Number.isFinite(inputMissionId) ? Math.trunc(inputMissionId) : null,
            individualMissionID: Number.isFinite(individualMissionId)
              ? Math.trunc(individualMissionId)
              : null,
          };
        });
      });
      return index;
    };
    const polylineLength = (points) => {
      let total = 0;
      for (let idx = 1; idx < points.length; idx += 1) {
        total += Math.hypot(points[idx].x - points[idx - 1].x, points[idx].y - points[idx - 1].y);
      }
      return total;
    };
    const pointAtFraction = (points, fraction) => {
      if (!points.length) {
        return { x: 0, y: 0 };
      }
      if (points.length === 1) {
        return points[0];
      }
      const target = Math.max(0, Math.min(1, Number(fraction))) * polylineLength(points);
      if (target <= 0) {
        return points[0];
      }
      let walked = 0;
      for (let idx = 1; idx < points.length; idx += 1) {
        const start = points[idx - 1];
        const end = points[idx];
        const segLen = Math.hypot(end.x - start.x, end.y - start.y);
        if (segLen <= 0) {
          continue;
        }
        if (walked + segLen >= target) {
          const ratio = (target - walked) / segLen;
          return {
            x: start.x + (end.x - start.x) * ratio,
            y: start.y + (end.y - start.y) * ratio,
          };
        }
        walked += segLen;
      }
      return points[points.length - 1];
    };
    const samplePolyline = (points, sampleCount = 9) => {
      if (points.length <= 2) {
        return points.slice();
      }
      const count = Math.max(2, Math.trunc(sampleCount));
      return Array.from({ length: count }, (_value, idx) => pointAtFraction(points, idx / (count - 1)));
    };
    const pointSegmentDistance = (point, start, end) => {
      const dx = end.x - start.x;
      const dy = end.y - start.y;
      const denom = dx * dx + dy * dy;
      if (denom <= 0) {
        return Math.hypot(point.x - start.x, point.y - start.y);
      }
      const ratio = Math.max(
        0,
        Math.min(1, ((point.x - start.x) * dx + (point.y - start.y) * dy) / denom),
      );
      const cx = start.x + dx * ratio;
      const cy = start.y + dy * ratio;
      return Math.hypot(point.x - cx, point.y - cy);
    };
    const pointPolylineDistance = (point, line) => {
      if (!line.length) {
        return Infinity;
      }
      if (line.length === 1) {
        return Math.hypot(point.x - line[0].x, point.y - line[0].y);
      }
      let best = Infinity;
      for (let idx = 1; idx < line.length; idx += 1) {
        best = Math.min(best, pointSegmentDistance(point, line[idx - 1], line[idx]));
      }
      return best;
    };
    const meanPolylineSpacing = (left, right) => {
      if (left.length < 2 || right.length < 2) {
        return null;
      }
      const distances = [
        ...samplePolyline(left).map((point) => pointPolylineDistance(point, right)),
        ...samplePolyline(right).map((point) => pointPolylineDistance(point, left)),
      ].filter(Number.isFinite);
      if (!distances.length) {
        return null;
      }
      return distances.reduce((sum, value) => sum + value, 0) / distances.length;
    };
    const projectLine = (line, originLat, originLon) => {
      const earthRadiusM = 6371008.8;
      const cosLat = Math.cos((originLat * Math.PI) / 180);
      return line.map((point) => ({
        x: (((point.lon - originLon) * Math.PI) / 180) * earthRadiusM * cosLat,
        y: (((point.lat - originLat) * Math.PI) / 180) * earthRadiusM,
      }));
    };
    const buildSweepLineSpacingSummaries = (pathMissionIndex) => {
      const grouped = new Map();
      flightPaths.forEach((path) => {
        const pathId = Number(path?.pathID ?? path?.PathID);
        if (!Number.isFinite(pathId)) {
          return;
        }
        const missionMeta = pathMissionIndex[String(Math.trunc(pathId))] || {};
        const inputMissionId = Number(missionMeta.inputMissionID);
        if (!Number.isFinite(inputMissionId)) {
          return;
        }
        const lines = extractSweepLinesFromPath(path);
        if (!lines.length) {
          return;
        }
        const key = Math.trunc(inputMissionId);
        if (!grouped.has(key)) {
          grouped.set(key, {
            inputMissionID: key,
            pathIds: new Set(),
            aircraftIds: new Set(),
            linesByPath: new Map(),
            allCoords: [],
          });
        }
        const group = grouped.get(key);
        const normalizedPathId = Math.trunc(pathId);
        group.pathIds.add(normalizedPathId);
        const aircraftId = Number(path?.aircraftID ?? path?.AircraftID ?? missionMeta.aircraftID);
        if (Number.isFinite(aircraftId)) {
          group.aircraftIds.add(Math.trunc(aircraftId));
        }
        const pathLines = group.linesByPath.get(normalizedPathId) || [];
        pathLines.push(...lines);
        group.linesByPath.set(normalizedPathId, pathLines);
        lines.forEach((line) => {
          group.allCoords.push(...line);
        });
      });

      return Array.from(grouped.values())
        .sort((a, b) => a.inputMissionID - b.inputMissionID)
        .map((group) => {
          if (!group.allCoords.length) {
            return null;
          }
          const originLat =
            group.allCoords.reduce((sum, point) => sum + point.lat, 0) / group.allCoords.length;
          const originLon =
            group.allCoords.reduce((sum, point) => sum + point.lon, 0) / group.allCoords.length;
          const distances = [];
          let lineCount = 0;
          Array.from(group.linesByPath.keys())
            .sort((a, b) => a - b)
            .forEach((pathId) => {
              const projectedLines = (group.linesByPath.get(pathId) || [])
                .filter((line) => line.length >= 2)
                .map((line) => projectLine(line, originLat, originLon));
              lineCount += projectedLines.length;
              for (let idx = 1; idx < projectedLines.length; idx += 1) {
                const spacing = meanPolylineSpacing(projectedLines[idx - 1], projectedLines[idx]);
                if (Number.isFinite(spacing)) {
                  distances.push(spacing);
                }
              }
            });
          if (!distances.length) {
            return null;
          }
          const averageLineSpacingM =
            distances.reduce((sum, value) => sum + value, 0) / distances.length;
          return {
            inputMissionID: group.inputMissionID,
            averageLineSpacingM,
            minLineSpacingM: Math.min(...distances),
            maxLineSpacingM: Math.max(...distances),
            lineCount,
            pairCount: distances.length,
            pathIds: Array.from(group.pathIds).sort((a, b) => a - b),
            aircraftIds: Array.from(group.aircraftIds).sort((a, b) => a - b),
          };
        })
        .filter(Boolean);
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
      const wpIds = [];
      for (const wp of waypoints) {
        const coord = getCoord(wp || {});
        if (!coord) continue;
        const wid = wp?.waypointID ?? wp?.WaypointID ?? null;
        const widNum = Number(wid);
        coords.push([coord.lon, coord.lat]);
        wpIds.push(Number.isFinite(widNum) ? widNum : null);
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
        wpIds,
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
    const pathMissionIndex = buildPathMissionIndex();
    const sweepLineSpacingSummaries = buildSweepLineSpacingSummaries(pathMissionIndex);
    const sweepLineSpacingByInputMissionID = Object.fromEntries(
      sweepLineSpacingSummaries.map((item) => [String(item.inputMissionID), item]),
    );
    return {
      ok: true,
      features,
      agents,
      count: features.length,
      flightPaths,
      missionOrder,
      inputMissionPlans: inputPlans,
      individualMissionPlans: individualPlans,
      pathMissionIndex,
      sweepLineSpacingSummaries,
      sweepLineSpacingByInputMissionID,
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

  if (reissueInput0201Btn) {
    reissueInput0201Btn.addEventListener("click", async () => {
      const sim = window.simClient;
      if (!sim || typeof sim.reissueInput0201 !== "function") {
        setStatusMessage("0201 reissue API unavailable");
        return;
      }
      reissueInput0201Btn.disabled = true;
      setStatusMessage("0201 재입력 준비 중...");
      try {
        const result = await sim.reissueInput0201();
        if (result && result.ok) {
          const packageId = result.newPackageID ?? "";
          setStatusMessage(`0201 재입력 전송 완료${packageId ? ` (#${packageId})` : ""}`);
        }
      } finally {
        reissueInput0201Btn.disabled = false;
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
