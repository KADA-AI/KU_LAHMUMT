/**
 * Analysis Panel — ICD validation results display.
 *
 * Usage:
 *   const panel = createAnalysisPanel(document.getElementById('analysis-panel'));
 *   panel.show(issues);  // issues = array from /api/scenario/validate
 *   panel.hide();
 */

const SEVERITY_ICON = { error: "\u274C", warning: "\u26A0\uFE0F", info: "\u2139\uFE0F" };
const SEVERITY_LABEL = { error: "\uC5D0\uB7EC", warning: "\uACBD\uACE0", info: "\uC815\uBCF4" };
const SEVERITY_CLASS = { error: "ap-severity-error", warning: "ap-severity-warning", info: "ap-severity-info" };
const CATEGORY_LABEL = { type: "\uD0C0\uC785", range: "\uBC94\uC704", rule: "\uADDC\uCE59", structure: "\uAD6C\uC870" };

/**
 * @param {HTMLElement} container  The #analysis-panel element.
 * @returns {{ show(issues: object[]), hide() }}
 */
export function createAnalysisPanel(container) {
  if (!container) return { show() {}, hide() {} };

  // State
  let allIssues = [];
  let severityFilter = { error: true, warning: true, info: true };
  let categoryFilter = { type: true, range: true, rule: true, structure: true };

  // Build DOM skeleton
  container.innerHTML = "";

  // Header
  const header = _el("div", "ap-header");
  const titleRow = _el("div", "ap-title-row");
  const title = _el("div", "ap-title", "\uBD84\uC11D \uACB0\uACFC");
  const closeBtn = _el("button", "ap-close", "\u00D7");
  closeBtn.setAttribute("aria-label", "\uB2EB\uAE30");
  closeBtn.addEventListener("click", () => hide());
  titleRow.append(title, closeBtn);
  header.append(titleRow);

  // Summary
  const summaryEl = _el("div", "ap-summary");
  header.append(summaryEl);

  // Severity filter
  const sevFilterRow = _el("div", "ap-filter-row");
  const sevLabel = _el("span", "ap-filter-label", "Severity");
  sevFilterRow.append(sevLabel);
  const sevBtns = {};
  for (const sev of ["error", "warning", "info"]) {
    const btn = _el("button", `ap-filter-btn ${SEVERITY_CLASS[sev]} is-active`, `${SEVERITY_ICON[sev]} ${SEVERITY_LABEL[sev]}`);
    btn.dataset.severity = sev;
    btn.addEventListener("click", () => {
      severityFilter[sev] = !severityFilter[sev];
      btn.classList.toggle("is-active", severityFilter[sev]);
      renderList();
    });
    sevBtns[sev] = btn;
    sevFilterRow.append(btn);
  }
  header.append(sevFilterRow);

  // Category filter
  const catFilterRow = _el("div", "ap-filter-row");
  const catLabel = _el("span", "ap-filter-label", "Category");
  catFilterRow.append(catLabel);
  const catBtns = {};
  for (const cat of ["type", "range", "rule", "structure"]) {
    const btn = _el("button", "ap-filter-btn ap-cat-btn is-active", CATEGORY_LABEL[cat]);
    btn.dataset.category = cat;
    btn.addEventListener("click", () => {
      categoryFilter[cat] = !categoryFilter[cat];
      btn.classList.toggle("is-active", categoryFilter[cat]);
      renderList();
    });
    catBtns[cat] = btn;
    catFilterRow.append(btn);
  }
  header.append(catFilterRow);

  // Issue list
  const listEl = _el("div", "ap-list");

  container.append(header, listEl);

  // ---- Render functions ----

  function renderSummary() {
    const counts = { error: 0, warning: 0, info: 0 };
    for (const issue of allIssues) {
      if (counts[issue.severity] !== undefined) counts[issue.severity]++;
    }
    summaryEl.innerHTML = "";
    for (const sev of ["error", "warning", "info"]) {
      const badge = _el("span", `ap-summary-badge ${SEVERITY_CLASS[sev]}`, `${SEVERITY_ICON[sev]} ${counts[sev]}`);
      summaryEl.append(badge);
    }
    const total = _el("span", "ap-summary-total", `\uCD1D ${allIssues.length}\uAC74`);
    summaryEl.append(total);
  }

  function renderList() {
    listEl.innerHTML = "";
    const filtered = allIssues.filter(
      (i) => severityFilter[i.severity] && categoryFilter[i.category],
    );

    if (filtered.length === 0) {
      const empty = _el("div", "ap-empty", severityFilter.error || severityFilter.warning || severityFilter.info
        ? "\uD544\uD130 \uC870\uAC74\uC5D0 \uB9DE\uB294 \uC774\uC288\uAC00 \uC5C6\uC2B5\uB2C8\uB2E4"
        : "\uD544\uD130\uB97C \uC120\uD0DD\uD558\uC138\uC694");
      listEl.append(empty);
      return;
    }

    const countLabel = _el("div", "ap-list-count", `${filtered.length}\uAC74 \uD45C\uC2DC \uC911`);
    listEl.append(countLabel);

    for (const issue of filtered) {
      listEl.append(buildIssueCard(issue));
    }
  }

  function buildIssueCard(issue) {
    const card = _el("div", `ap-card ${SEVERITY_CLASS[issue.severity]}`);

    // Top row: icon + location + category badge
    const topRow = _el("div", "ap-card-top");
    const icon = _el("span", "ap-card-icon", SEVERITY_ICON[issue.severity]);
    const loc = _el("span", "ap-card-location", issue.location || "");
    const catBadge = _el("span", "ap-card-category", CATEGORY_LABEL[issue.category] || issue.category);
    topRow.append(icon, loc, catBadge);

    // Field
    const fieldRow = _el("div", "ap-card-field", issue.field || "");

    // Message
    const msgRow = _el("div", "ap-card-message", issue.message || "");

    card.append(topRow, fieldRow, msgRow);

    // Value / Expected (if present)
    if (issue.value !== null && issue.value !== undefined) {
      const valRow = _el("div", "ap-card-values");
      const valLabel = _el("span", "ap-card-val-label", "\uC2E4\uC81C:");
      const valValue = _el("span", "ap-card-val-value", String(issue.value));
      valRow.append(valLabel, valValue);
      if (issue.expected) {
        const expLabel = _el("span", "ap-card-val-label", "\uAE30\uB300:");
        const expValue = _el("span", "ap-card-val-expected", String(issue.expected));
        valRow.append(expLabel, expValue);
      }
      card.append(valRow);
    }

    return card;
  }

  // ---- Public API ----

  function show(issues) {
    allIssues = Array.isArray(issues) ? issues : [];
    // Reset filters
    severityFilter = { error: true, warning: true, info: true };
    categoryFilter = { type: true, range: true, rule: true, structure: true };
    for (const btn of Object.values(sevBtns)) btn.classList.add("is-active");
    for (const btn of Object.values(catBtns)) btn.classList.add("is-active");
    renderSummary();
    renderList();
    container.setAttribute("aria-hidden", "false");
  }

  function hide() {
    container.setAttribute("aria-hidden", "true");
  }

  return { show, hide };
}

// ---- DOM helper ----
function _el(tag, className, text) {
  const el = document.createElement(tag);
  if (className) el.className = className;
  if (text !== undefined) el.textContent = text;
  return el;
}
