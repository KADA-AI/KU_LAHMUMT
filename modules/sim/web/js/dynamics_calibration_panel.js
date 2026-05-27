import { logStatus } from "./status_log.js";

const STATUS_ENDPOINT = "/api/sim/dynamics_calibration/status";
const ANALYZE_ENDPOINT = "/api/sim/dynamics_calibration/analyze";
const ANALYZE_UPLOAD_ENDPOINT = "/api/sim/dynamics_calibration/analyze_upload";
const APPLY_ENDPOINT = "/api/sim/dynamics_calibration/apply";
const DISABLE_ENDPOINT = "/api/sim/dynamics_calibration/disable";

const fmt = (value, digits = 1, suffix = "") => {
  const num = Number(value);
  if (!Number.isFinite(num)) return "-";
  return `${num.toFixed(digits)}${suffix}`;
};

const escapeHtml = (value) =>
  String(value ?? "")
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;")
    .replace(/'/g, "&#39;");

const postJson = async (url, payload) => {
  const res = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload || {}),
  });
  let data = null;
  try {
    data = await res.json();
  } catch (err) {
    data = null;
  }
  if (!res.ok || !data || data.ok === false) {
    throw new Error((data && data.error) || `HTTP ${res.status}`);
  }
  return data;
};

const readFileText = (file) =>
  new Promise((resolve, reject) => {
    const reader = new FileReader();
    reader.onload = () => resolve(String(reader.result || ""));
    reader.onerror = () => reject(reader.error || new Error("failed to read file"));
    reader.readAsText(file, "utf-8");
  });

const is0401Json = (file) => {
  const rel = String(file.webkitRelativePath || file.name || "").replace(/\\/g, "/");
  const base = rel.split("/").pop() || "";
  return /^0401.*\.json$/i.test(base);
};

const renderMetrics = (container, result) => {
  if (!container) return;
  const summary = result?.summary || {};
  const proposal = result?.proposal || {};
  const basis = proposal.basis || {};
  const agents = summary.agents || {};
  const rows = Object.entries(agents)
    .map(([aircraftId, item]) => {
      const label = Number(aircraftId) >= 4 ? `UAV${Number(aircraftId) - 3}` : `AC${aircraftId}`;
      const usable = item?.usable ? "usable" : "weak";
      return `
        <div class="dynfit-agent">
          <div class="dynfit-agent-name">${escapeHtml(label)}</div>
          <div class="dynfit-agent-value">${fmt(item?.median_turn_radius_m, 0, "m")}</div>
          <div class="dynfit-agent-sub">${fmt(item?.abs_yaw_rate_p95_dps, 2, " dps")} / ${escapeHtml(usable)}</div>
        </div>
      `;
    })
    .join("");
  container.innerHTML = `
    <div class="dynfit-kpis">
      <div class="dynfit-kpi">
        <span>Tracks</span>
        <strong>${Number(summary.usable_agent_count || 0)} / ${Number(summary.agent_count || 0)}</strong>
      </div>
      <div class="dynfit-kpi">
        <span>Turn radius</span>
        <strong>${fmt(basis.median_turn_radius_m, 0, "m")}</strong>
      </div>
      <div class="dynfit-kpi">
        <span>Radius scale</span>
        <strong>${fmt(basis.reference_turn_radius_scale, 2, "x")}</strong>
      </div>
      <div class="dynfit-kpi">
        <span>Opp turn gap</span>
        <strong>${fmt(basis.opposite_turn_gap_p50_s, 2, "s")}</strong>
      </div>
    </div>
    <div class="dynfit-agent-grid">${rows || '<div class="dynfit-empty">No UAV track</div>'}</div>
  `;
};

export const initDynamicsCalibrationPanel = () => {
  const pathInput = document.getElementById("dynfit-log-path");
  const folderPicker = document.getElementById("dynfit-folder-picker");
  const browseBtn = document.getElementById("dynfit-browse-log");
  const analyzeBtn = document.getElementById("dynfit-analyze");
  const applyBtn = document.getElementById("dynfit-apply");
  const disableBtn = document.getElementById("dynfit-disable");
  const statusEl = document.getElementById("dynfit-status");
  const metricsEl = document.getElementById("dynfit-metrics");
  const proposalEl = document.getElementById("dynfit-proposal");
  const profileState = document.getElementById("dynfit-profile-state");

  if (!pathInput || !analyzeBtn || !applyBtn || !statusEl || !metricsEl || !proposalEl) {
    return;
  }

  let lastProposal = null;
  let uploadedFiles = null;
  let uploadedDisplay = "";

  const setBusy = (busy) => {
    analyzeBtn.disabled = Boolean(busy);
    applyBtn.disabled = Boolean(busy) || !lastProposal;
    [browseBtn, disableBtn].forEach((btn) => {
      if (btn) btn.disabled = Boolean(busy);
    });
  };

  const setStatus = (text, level = "") => {
    statusEl.textContent = text || "";
    statusEl.classList.toggle("is-warn", level === "warn");
    statusEl.classList.toggle("is-error", level === "error");
    statusEl.classList.toggle("is-ok", level === "ok");
  };

  const setAnalysisResult = (data, sourceLabel) => {
    lastProposal = data.proposal || null;
    renderMetrics(metricsEl, data);
    proposalEl.textContent = JSON.stringify(lastProposal, null, 2);
    const quality = lastProposal?.quality === "ok" ? "ok" : "warn";
    const prefix = sourceLabel ? `${sourceLabel}: ` : "";
    setStatus(
      quality === "ok"
        ? `${prefix}proposal ready. Check it, then press Apply.`
        : `${prefix}proposal ready with conservative defaults.`,
      quality,
    );
    logStatus("Dynamics calibration analysis complete.", {
      key: "dynfit",
      level: quality === "ok" ? "success" : "warn",
      ttlMs: 5000,
    });
  };

  const refreshProfileStatus = async () => {
    if (!profileState) return;
    try {
      const res = await fetch(STATUS_ENDPOINT, { cache: "no-store" });
      const data = await res.json();
      const enabled = Boolean(data?.profile?.enabled);
      profileState.textContent = enabled ? "PROFILE ON" : "PROFILE OFF";
      profileState.classList.toggle("is-on", enabled);
    } catch (err) {
      profileState.textContent = "PROFILE ?";
      profileState.classList.remove("is-on");
    }
  };

  const resetAnalysisUi = () => {
    lastProposal = null;
    proposalEl.textContent = "";
    metricsEl.innerHTML = "";
  };

  const analyzeUploadPayload = async (files, sourceLabel = "Folder") => {
    resetAnalysisUi();
    setBusy(true);
    setStatus(`Analyzing ${files.length} uploaded 0401 log file(s)...`);
    try {
      const data = await postJson(ANALYZE_UPLOAD_ENDPOINT, { files });
      setAnalysisResult(data, sourceLabel);
    } catch (err) {
      setStatus(`Analyze failed: ${err.message || err}`, "error");
      logStatus(`Dynamics analysis failed: ${err.message || err}`, {
        key: "dynfit",
        level: "error",
        ttlMs: 6000,
      });
    } finally {
      setBusy(false);
    }
  };

  const analyzePath = async () => {
    const logPath = String(pathInput.value || "").trim();
    if (uploadedFiles && uploadedDisplay && logPath === uploadedDisplay) {
      await analyzeUploadPayload(uploadedFiles, "Folder");
      return;
    }
    if (!logPath) {
      setStatus("Enter a log path or choose a log folder.", "warn");
      return;
    }
    resetAnalysisUi();
    setBusy(true);
    setStatus("Analyzing 0401 log path...");
    try {
      const data = await postJson(ANALYZE_ENDPOINT, { path: logPath });
      setAnalysisResult(data, "Path");
    } catch (err) {
      setStatus(`Analyze failed: ${err.message || err}`, "error");
      logStatus(`Dynamics analysis failed: ${err.message || err}`, {
        key: "dynfit",
        level: "error",
        ttlMs: 6000,
      });
    } finally {
      setBusy(false);
    }
  };

  const analyzeSelectedFiles = async (fileList) => {
    const selected = Array.from(fileList || []).filter(is0401Json);
    if (!selected.length) {
      setStatus("Selected folder has no 0401*.json files.", "warn");
      return;
    }
    resetAnalysisUi();
    setBusy(true);
    setStatus(`Reading ${selected.length} 0401 log file(s)...`);
    try {
      const files = await Promise.all(
        selected.map(async (file) => ({
          name: file.name,
          relativePath: file.webkitRelativePath || file.name,
          content: await readFileText(file),
        })),
      );
      const data = await postJson(ANALYZE_UPLOAD_ENDPOINT, { files });
      const folderLabel = selected[0]?.webkitRelativePath
        ? selected[0].webkitRelativePath.split("/")[0]
        : "Selected log";
      uploadedFiles = files;
      uploadedDisplay = `${folderLabel} (${selected.length} uploaded 0401 file(s))`;
      pathInput.value = uploadedDisplay;
      setAnalysisResult(data, "Folder");
    } catch (err) {
      setStatus(`Folder analyze failed: ${err.message || err}`, "error");
      logStatus(`Dynamics folder analysis failed: ${err.message || err}`, {
        key: "dynfit",
        level: "error",
        ttlMs: 6000,
      });
    } finally {
      setBusy(false);
      if (folderPicker) folderPicker.value = "";
    }
  };

  pathInput.addEventListener("input", () => {
    if (String(pathInput.value || "").trim() !== uploadedDisplay) {
      uploadedFiles = null;
      uploadedDisplay = "";
    }
  });

  const apply = async () => {
    if (!lastProposal) {
      setStatus("Run Analyze first.", "warn");
      return;
    }
    setBusy(true);
    setStatus("Applying dynamics proposal...");
    try {
      const data = await postJson(APPLY_ENDPOINT, { proposal: lastProposal });
      setStatus("Applied. It will affect UAVs from the next mission load/reset.", "ok");
      proposalEl.textContent = JSON.stringify(
        {
          applied: data.applied,
          backups: data.backups,
          profile: data.profile,
        },
        null,
        2,
      );
      logStatus("Dynamics calibration applied.", {
        key: "dynfit",
        level: "success",
        ttlMs: 5000,
      });
      await refreshProfileStatus();
    } catch (err) {
      setStatus(`Apply failed: ${err.message || err}`, "error");
      logStatus(`Dynamics apply failed: ${err.message || err}`, {
        key: "dynfit",
        level: "error",
        ttlMs: 6000,
      });
    } finally {
      setBusy(false);
    }
  };

  const disable = async () => {
    setBusy(true);
    setStatus("Disabling dynamics profile...");
    try {
      const data = await postJson(DISABLE_ENDPOINT, {});
      setStatus("Profile OFF. Default UAV dynamics will be used from the next mission load/reset.", "ok");
      proposalEl.textContent = JSON.stringify(
        {
          profile_path: data.profile_path,
          backup: data.backup,
          profile: data.profile,
        },
        null,
        2,
      );
      logStatus("Dynamics profile disabled.", {
        key: "dynfit",
        level: "success",
        ttlMs: 5000,
      });
      await refreshProfileStatus();
    } catch (err) {
      setStatus(`OFF failed: ${err.message || err}`, "error");
      logStatus(`Dynamics disable failed: ${err.message || err}`, {
        key: "dynfit",
        level: "error",
        ttlMs: 6000,
      });
    } finally {
      setBusy(false);
    }
  };

  if (browseBtn && folderPicker) {
    browseBtn.addEventListener("click", () => {
      folderPicker.click();
    });
    folderPicker.addEventListener("change", () => {
      analyzeSelectedFiles(folderPicker.files);
    });
  }

  analyzeBtn.addEventListener("click", analyzePath);
  applyBtn.addEventListener("click", apply);
  if (disableBtn) {
    disableBtn.addEventListener("click", disable);
  }

  refreshProfileStatus();
};
