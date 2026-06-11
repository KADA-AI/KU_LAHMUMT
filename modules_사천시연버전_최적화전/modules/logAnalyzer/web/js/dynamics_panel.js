/**
 * Detailed 0401 dynamics analysis panel.
 */

const fmt = (value, digits = 1, suffix = "") => {
  const num = Number(value);
  if (!Number.isFinite(num)) return "-";
  return `${num.toFixed(digits)}${suffix}`;
};

const esc = (value) =>
  String(value ?? "")
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;")
    .replace(/'/g, "&#39;");

const stat = (obj, key = "p50", digits = 1, suffix = "") => {
  if (!obj || !obj.count) return "-";
  return fmt(obj[key], digits, suffix);
};

const pct = (count, total) => {
  const c = Number(count || 0);
  const t = Number(total || 0);
  if (!t) return "0%";
  return `${Math.round((c / t) * 100)}%`;
};

const profileRows = (profile) =>
  Object.entries(profile || {})
    .map(([key, value]) => `
      <div class="dyn-row">
        <span>${esc(key)}</span>
        <strong>${typeof value === "number" ? fmt(value, 3) : esc(value)}</strong>
      </div>
    `)
    .join("");

const renderAgents = (agents) => {
  const rows = Object.values(agents || {})
    .sort((a, b) => Number(a.aircraftId || 0) - Number(b.aircraftId || 0))
    .map((agent) => `
      <div class="dyn-table-row">
        <span class="dyn-agent-label ${agent.isUav ? "is-uav" : ""}">${esc(agent.label)}</span>
        <span>${Number(agent.sampleCount || 0).toLocaleString()}</span>
        <span>${fmt(agent.durationS, 1, "s")}</span>
        <span>${stat(agent.speedMps, "p50", 1, "m/s")}</span>
        <span>${stat(agent.turnRadiusM, "p50", 0, "m")}</span>
        <span>${stat(agent.turnRadiusM, "p75", 0, "m")}</span>
        <span>${stat(agent.absYawRateDps, "p95", 2)}</span>
        <span>${stat(agent.bankProxyDeg, "p95", 1, "deg")}</span>
        <span>${agent.rollObserved ? "direct" : "proxy"}</span>
      </div>
    `)
    .join("");
  return `
    <section class="dyn-section">
      <div class="dyn-section-title">Aircraft Breakdown</div>
      <div class="dyn-table dyn-agent-table">
        <div class="dyn-table-head">
          <span>AC</span><span>Samples</span><span>Dur</span><span>V50</span><span>R50</span><span>R75</span><span>Yaw95</span><span>Bank95</span><span>Roll</span>
        </div>
        ${rows || '<div class="dyn-empty">No aircraft samples</div>'}
      </div>
    </section>
  `;
};

const renderPhases = (agents) => {
  const phaseRows = [];
  for (const agent of Object.values(agents || {})) {
    for (const phase of agent.phaseMetrics || []) {
      if (!phase.sampleCount) continue;
      phaseRows.push({ agent, phase });
    }
  }
  phaseRows.sort((a, b) => Number(b.phase.sampleCount || 0) - Number(a.phase.sampleCount || 0));
  const rows = phaseRows.slice(0, 36).map(({ agent, phase }) => `
    <div class="dyn-table-row">
      <span class="dyn-agent-label ${agent.isUav ? "is-uav" : ""}">${esc(agent.label)}</span>
      <span>${esc(phase.phase)}</span>
      <span>${Number(phase.sampleCount || 0).toLocaleString()}</span>
      <span>${stat(phase.speedMps, "p50", 1, "m/s")}</span>
      <span>${stat(phase.turnRadiusM, "p50", 0, "m")}</span>
      <span>${stat(phase.absYawRateDps, "p95", 2)}</span>
      <span>${stat(phase.bankProxyDeg, "p95", 1, "deg")}</span>
    </div>
  `).join("");
  return `
    <section class="dyn-section">
      <div class="dyn-section-title">Mission Phase Sensitivity</div>
      <div class="dyn-table dyn-phase-table">
        <div class="dyn-table-head">
          <span>AC</span><span>Phase</span><span>N</span><span>V50</span><span>R50</span><span>Yaw95</span><span>Bank95</span>
        </div>
        ${rows || '<div class="dyn-empty">No phase metrics</div>'}
      </div>
    </section>
  `;
};

const renderEvents = (events) => {
  const rows = (events || []).slice(0, 24).map((event) => `
    <div class="dyn-event">
      <div class="dyn-event-top">
        <strong>${esc(event.label)}</strong>
        <span>${fmt(event.durationS, 2, "s")}</span>
        <span>${esc(event.phase)}</span>
      </div>
      <div class="dyn-event-metrics">
        <span>yaw ${fmt(event.maxYawRateDps, 2)} dps</span>
        <span>radius ${fmt(event.minRadiusM, 0)} m</span>
        <span>bank ${fmt(event.maxBankProxyDeg, 1)} deg</span>
        <span>angle ${fmt(event.turnAngleDeg, 0)} deg</span>
      </div>
    </div>
  `).join("");
  return `
    <section class="dyn-section">
      <div class="dyn-section-title">Aggressive Turn Events</div>
      <div class="dyn-event-list">${rows || '<div class="dyn-empty">No aggressive turn event detected</div>'}</div>
    </section>
  `;
};

const renderCommands = (commands) => {
  const rows = (commands?.rows || []).slice(0, 30).map((row) => `
    <div class="dyn-table-row">
      <span class="dyn-agent-label">${esc(row.label)}</span>
      <span>${esc(row.type)}</span>
      <span>${row.matched ? "matched" : "miss"}</span>
      <span>${row.latencyS == null ? "-" : fmt(row.latencyS, 3, "s")}</span>
      <span>${esc(JSON.stringify(row.expected || {}))}</span>
    </div>
  `).join("");
  return `
    <section class="dyn-section">
      <div class="dyn-section-title">0602 Command Response</div>
      <div class="dyn-command-summary">
        <span>${Number(commands?.matchedCount || 0)} / ${Number(commands?.commandCount || 0)} matched</span>
        <span>${pct(commands?.matchedCount, commands?.commandCount)}</span>
        <span>p50 ${stat(commands?.latencyS, "p50", 3, "s")}</span>
        <span>p95 ${stat(commands?.latencyS, "p95", 3, "s")}</span>
      </div>
      <div class="dyn-table dyn-command-table">
        <div class="dyn-table-head"><span>AC</span><span>Cmd</span><span>State</span><span>Latency</span><span>Expected</span></div>
        ${rows || '<div class="dyn-empty">No 0602 command rows</div>'}
      </div>
    </section>
  `;
};

const renderSignals = (signals) => {
  const counts = signals?.signalCounts || {};
  const rows = Object.entries(counts)
    .map(([key, value]) => `<div class="dyn-row"><span>${esc(key)}</span><strong>${Number(value || 0).toLocaleString()}</strong></div>`)
    .join("");
  return `
    <section class="dyn-section">
      <div class="dyn-section-title">All-Log Signal Scan</div>
      <div class="dyn-signal-meta">
        <span>${Number(signals?.totalFilesScanned || 0).toLocaleString()} files</span>
        <span>${signals?.rollSignalsAvailable ? "roll fields present" : "roll fields absent; bank proxy used"}</span>
      </div>
      <div class="dyn-reco-grid">${rows}</div>
    </section>
  `;
};

const renderSpeedBuckets = (agents) => {
  const rows = [];
  for (const agent of Object.values(agents || {})) {
    for (const bucket of agent.speedBuckets || []) {
      rows.push(`
        <div class="dyn-table-row">
          <span class="dyn-agent-label ${agent.isUav ? "is-uav" : ""}">${esc(agent.label)}</span>
          <span>${esc(bucket.bucket)} m/s</span>
          <span>${stat(bucket.turnRadiusM, "p50", 0, "m")}</span>
          <span>${stat(bucket.turnRadiusM, "p75", 0, "m")}</span>
          <span>${stat(bucket.turnRadiusM, "p90", 0, "m")}</span>
        </div>
      `);
    }
  }
  return `
    <section class="dyn-section">
      <div class="dyn-section-title">Speed Bucket Turn Radius</div>
      <div class="dyn-table dyn-bucket-table">
        <div class="dyn-table-head"><span>AC</span><span>Speed</span><span>R50</span><span>R75</span><span>R90</span></div>
        ${rows.join("") || '<div class="dyn-empty">No speed-bucket turn samples</div>'}
      </div>
    </section>
  `;
};

const renderPlannerSpeedTable = (rows) => {
  const tableRows = (rows || []).map((row) => `
    <div class="dyn-table-row">
      <span>${esc(row.configKey || "-")}</span>
      <span>${fmt(row.speedMps, 0, "m/s")}</span>
      <span>${fmt(row.currentReferenceRadiusM, 0, "m")}</span>
      <span>${Number(row.observedSampleCount || 0).toLocaleString()}</span>
      <span>${Number(row.observedAgentCount || 0).toLocaleString()}</span>
      <span>${stat(row.observedAgentP50RadiusM, "p50", 0, "m")}</span>
      <span>${fmt(row.scaledReferenceRadiusM, 0, "m")}</span>
      <span>${fmt(row.recommendedTurnRadiusM, 0, "m")}</span>
      <span>${esc(row.source || "-")}</span>
    </div>
  `).join("");
  return `
    <div class="dyn-profile dyn-planner-speed-fit">
      <div class="dyn-profile-title">Planner speed turn radius table</div>
      <div class="dyn-table dyn-planner-speed-table">
        <div class="dyn-table-head">
          <span>Config</span><span>Speed</span><span>Current</span><span>Samples</span><span>AC</span><span>Obs R50</span><span>Scaled</span><span>Reco</span><span>Source</span>
        </div>
        ${tableRows || '<div class="dyn-empty">No planner speed turn radius rows</div>'}
      </div>
    </div>
  `;
};

export function createDynamicsPanel(container, options = {}) {
  if (!container) return { show() {}, hide() {}, showLoading() {}, showError() {} };

  let latestResult = null;
  let saving = false;

  const hide = () => {
    container.setAttribute("aria-hidden", "true");
    document.body.classList.remove("right-panel-open");
  };

  const setSaveState = (message = "") => {
    const btn = container.querySelector(".dyn-save-btn");
    const status = container.querySelector(".dyn-save-status");
    if (btn) {
      btn.disabled = !latestResult || saving;
      btn.textContent = saving ? "Saving..." : "Save Report";
    }
    if (status) status.textContent = message;
  };

  const handleSave = async () => {
    if (!latestResult || saving || typeof options.onSave !== "function") return;
    saving = true;
    setSaveState("Writing report files...");
    try {
      const saved = await options.onSave(latestResult);
      const fileName = saved?.fileNames?.markdown || saved?.fileNames?.json || "saved";
      setSaveState(`Saved ${fileName}`);
    } catch (err) {
      setSaveState(`Save failed: ${err?.message || err}`);
    } finally {
      saving = false;
      setSaveState(container.querySelector(".dyn-save-status")?.textContent || "");
    }
  };

  const renderShell = (bodyHtml) => {
    container.innerHTML = `
      <div class="ap-header dyn-header">
        <div class="ap-title-row">
          <div>
            <div class="ap-title">Dynamics Analysis</div>
            <div class="dyn-subtitle">0401 trajectory, turn, response, and planning fit</div>
          </div>
          <div class="dyn-header-actions">
            ${typeof options.onSave === "function" ? '<button class="dyn-save-btn" type="button" disabled>Save Report</button>' : ""}
            <button class="ap-close" type="button" aria-label="Close">&times;</button>
          </div>
        </div>
        ${typeof options.onSave === "function" ? '<div class="dyn-save-status" aria-live="polite"></div>' : ""}
      </div>
      <div class="dyn-body">${bodyHtml}</div>
    `;
    container.querySelector(".ap-close")?.addEventListener("click", hide);
    container.querySelector(".dyn-save-btn")?.addEventListener("click", () => void handleSave());
    setSaveState();
    container.setAttribute("aria-hidden", "false");
    document.body.classList.add("right-panel-open");
  };

  const showLoading = (scenarioName) => {
    latestResult = null;
    renderShell(`<div class="dyn-loading">${esc(scenarioName)} analyzing...</div>`);
  };

  const showError = (message) => {
    latestResult = null;
    renderShell(`<div class="dyn-error">${esc(message)}</div>`);
  };

  const show = (result) => {
    latestResult = result;
    const source = result?.source || {};
    const cohort = result?.cohort || {};
    const rec = result?.recommendations || {};
    const basis = rec.basis || {};
    const hints = rec.missionPlanningHints || {};
    const profile = rec.simDynamicsProfile || {};
    const commandLatency = result?.commands?.latencyS || {};
    const quality = rec.quality === "ok" ? "ok" : "warn";
    renderShell(`
      <section class="dyn-hero ${quality === "ok" ? "is-ok" : "is-warn"}">
        <div>
          <div class="dyn-hero-title">${esc(result?.scenario || "-")}</div>
          <div class="dyn-hero-sub">${Number(source.fileCount0401 || 0)} 0401 files · ${Number(source.sampleCount0401 || 0).toLocaleString()} samples</div>
        </div>
        <span>${quality.toUpperCase()}</span>
      </section>

      <section class="dyn-kpi-grid">
        <div><span>Usable UAV</span><strong>${Number(cohort.usableUavCount || 0)} / ${Number(cohort.uavCount || 0)}</strong></div>
        <div><span>Speed p50</span><strong>${stat(cohort.speedMps, "p50", 1, "m/s")}</strong></div>
        <div><span>Turn R p50</span><strong>${stat(cohort.turnRadiusM, "p50", 0, "m")}</strong></div>
        <div><span>Yaw p95</span><strong>${stat(cohort.absYawRateDps, "p50", 2, " dps")}</strong></div>
        <div><span>Bank p95</span><strong>${stat(cohort.bankProxyDeg, "p50", 1, "deg")}</strong></div>
        <div><span>Cmd p50</span><strong>${stat(commandLatency, "p50", 3, "s")}</strong></div>
      </section>

      <section class="dyn-section">
        <div class="dyn-section-title">Planning / Runtime Fit</div>
        <div class="dyn-reco-grid">
          <div class="dyn-row"><span>median speed</span><strong>${fmt(basis.medianSpeedMps, 2, " m/s")}</strong></div>
          <div class="dyn-row"><span>median radius</span><strong>${fmt(basis.medianTurnRadiusM, 1, " m")}</strong></div>
          <div class="dyn-row"><span>reference radius</span><strong>${fmt(basis.currentReferenceRadiusM, 1, " m")}</strong></div>
          <div class="dyn-row"><span>radius scale</span><strong>${fmt(profile.reference_turn_radius_scale, 4, "x")}</strong></div>
          <div class="dyn-row"><span>lookahead</span><strong>${fmt(hints.lookaheadM, 1, " m")}</strong></div>
          <div class="dyn-row"><span>freeze yaw</span><strong>${fmt(hints.freezeYawDistanceM, 1, " m")}</strong></div>
          <div class="dyn-row"><span>nominal radius</span><strong>${fmt(hints.nominalTurnRadiusM, 1, " m")}</strong></div>
          <div class="dyn-row"><span>conservative radius</span><strong>${fmt(hints.conservativeTurnRadiusM, 1, " m")}</strong></div>
        </div>
        ${renderPlannerSpeedTable(hints.speedTurnRadiusTable)}
        <div class="dyn-profile">
          <div class="dyn-profile-title">SIM-compatible dynamics profile</div>
          ${profileRows(profile)}
        </div>
      </section>

      ${renderAgents(result?.agents)}
      ${renderSpeedBuckets(result?.agents)}
      ${renderPhases(result?.agents)}
      ${renderCommands(result?.commands)}
      ${renderEvents(result?.events)}
      ${renderSignals(result?.logSignals)}
    `);
  };

  return { show, hide, showLoading, showError };
}
