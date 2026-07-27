/**
 * Timeline strip builder.
 * Renders plan nodes and replan connectors in a horizontal scrollable track.
 */

/**
 * @param {HTMLElement} container - The #timeline-track element
 * @param {Object} scenarioData - Full scenario data from API
 * @param {Object} callbacks
 * @param {Function} callbacks.onPlanSelect - Called with plan object
 * @param {Function} callbacks.onReplanSelect - Called with replan object
 * @returns {{ selectPlan(planID: string): void, setTimeFocus(planID: string | null): void }}
 */
export const buildTimeline = (container, scenarioData, { onPlanSelect, onReplanSelect }) => {
  enableTimelineScrolling(container);
  container.innerHTML = "";

  const timeline = scenarioData.timeline || [];
  if (timeline.length === 0) {
    container.innerHTML = '<div class="tl-empty">타임라인 데이터 없음</div>';
    return { selectPlan() {} };
  }

  /** @type {Map<string, HTMLElement>} */
  const planNodes = new Map();
  let selectedId = null;
  let timeFocusedId = null;

  let planSeq = 0;
  timeline.forEach((entry) => {
    // If there's a replan before this plan, render the connector
    if (entry.replan) {
      const connector = buildConnector(entry.replan, onReplanSelect);
      container.appendChild(connector);
    }

    // Plan node
    if (entry.plan) {
      planSeq += 1;
      const node = buildPlanNode(entry.plan, planSeq, () => {
        selectPlan(entry.plan.missionPlanID);
        onPlanSelect(entry.plan);
      });
      planNodes.set(String(entry.plan.missionPlanID), node);
      container.appendChild(node);
    }
  });

  const selectPlan = (planID) => {
    const id = String(planID);
    if (selectedId === id) return;

    // Deselect previous
    if (selectedId && planNodes.has(selectedId)) {
      planNodes.get(selectedId).classList.remove("is-selected");
    }

    selectedId = id;
    if (planNodes.has(id)) {
      const node = planNodes.get(id);
      node.classList.add("is-selected");
      // Scroll into view
      node.scrollIntoView({ behavior: "smooth", block: "nearest", inline: "center" });
    }
  };

  const setTimeFocus = (planID) => {
    if (timeFocusedId && planNodes.has(timeFocusedId)) {
      planNodes.get(timeFocusedId).classList.remove("is-time-focused");
      planNodes.get(timeFocusedId).setAttribute("aria-pressed", "false");
    }
    timeFocusedId = planID == null ? null : String(planID);
    if (timeFocusedId && planNodes.has(timeFocusedId)) {
      planNodes.get(timeFocusedId).classList.add("is-time-focused");
      planNodes.get(timeFocusedId).setAttribute("aria-pressed", "true");
    }
  };

  return { selectPlan, setTimeFocus };
};

/**
 * Make the bottom timeline usable with a normal mouse wheel and drag gesture.
 * The strip itself is horizontal-only, so vertical wheel deltas are intentionally
 * translated into scrollLeft while the cursor is over the timeline.
 */
const enableTimelineScrolling = (container) => {
  if (!container || container.dataset.scrollBehaviorReady === "true") return;
  container.dataset.scrollBehaviorReady = "true";

  container.addEventListener("wheel", (event) => {
    if (container.scrollWidth <= container.clientWidth) return;

    const dominantDelta = Math.abs(event.deltaY) >= Math.abs(event.deltaX)
      ? event.deltaY
      : event.deltaX;
    if (!dominantDelta) return;

    event.preventDefault();
    container.scrollLeft += dominantDelta;
  }, { passive: false });

  let dragStartX = 0;
  let dragStartScrollLeft = 0;
  let isPointerDown = false;
  let didDrag = false;

  container.addEventListener("pointerdown", (event) => {
    if (event.button !== 0 || container.scrollWidth <= container.clientWidth) return;
    isPointerDown = true;
    didDrag = false;
    dragStartX = event.clientX;
    dragStartScrollLeft = container.scrollLeft;
    container.classList.add("is-dragging");
    container.setPointerCapture?.(event.pointerId);
  });

  container.addEventListener("pointermove", (event) => {
    if (!isPointerDown) return;
    const deltaX = event.clientX - dragStartX;
    if (Math.abs(deltaX) > 3) didDrag = true;
    if (didDrag) {
      event.preventDefault();
      container.scrollLeft = dragStartScrollLeft - deltaX;
    }
  });

  const endDrag = (event) => {
    if (!isPointerDown) return;
    isPointerDown = false;
    container.classList.remove("is-dragging");
    container.releasePointerCapture?.(event.pointerId);
  };

  container.addEventListener("pointerup", endDrag);
  container.addEventListener("pointercancel", endDrag);
  container.addEventListener("click", (event) => {
    if (!didDrag) return;
    event.preventDefault();
    event.stopPropagation();
    didDrag = false;
  }, true);
};

/**
 * Build a single plan node element.
 */
const buildPlanNode = (plan, seqIndex, onClick) => {
  const el = document.createElement("div");
  el.className = plan.isSelected ? "tl-node is-applied" : "tl-node";
  el.setAttribute("data-plan-id", plan.missionPlanID);
  el.setAttribute("role", "button");
  el.setAttribute("aria-pressed", "false");
  el.title = "클릭: 이 Plan의 적용 시간만 보기 · 다시 클릭: 전체보기";

  const label = document.createElement("div");
  label.className = "tl-node-label";
  label.textContent = `Plan ${seqIndex}`;

  const sub = document.createElement("div");
  sub.className = "tl-node-sub";
  sub.textContent = formatPlanSub(plan);

  el.appendChild(label);
  el.appendChild(sub);
  el.addEventListener("click", onClick);

  return el;
};

/**
 * Build a replan connector element between two plan nodes.
 */
const buildConnector = (replan, onReplanSelect) => {
  const wrapper = document.createElement("div");
  wrapper.className = "tl-connector";

  const lineL = document.createElement("div");
  lineL.className = "tl-connector-line";

  const badge = document.createElement("div");
  badge.className = "tl-connector-badge";

  const badgeLabel = document.createElement("div");
  badgeLabel.className = "tl-connector-badge-label";
  badgeLabel.textContent = "REPLAN";

  const badgeSub = document.createElement("div");
  badgeSub.className = "tl-connector-badge-sub";
  badgeSub.textContent = getReplanReasonText(replan.reason);

  badge.appendChild(badgeLabel);
  badge.appendChild(badgeSub);
  badge.addEventListener("click", () => onReplanSelect(replan));

  const lineR = document.createElement("div");
  lineR.className = "tl-connector-line";

  wrapper.appendChild(lineL);
  wrapper.appendChild(badge);
  wrapper.appendChild(lineR);

  return wrapper;
};

const formatPlanSub = (plan) => {
  if (plan.timestamp) {
    try {
      const d = new Date(plan.timestamp);
      return d.toLocaleTimeString("ko-KR", { hour: "2-digit", minute: "2-digit", second: "2-digit" });
    } catch {
      return plan.missionPlanID || "";
    }
  }
  return `ID: ${plan.missionPlanID || "?"}`;
};

const REPLAN_REASONS = {
  operator: "운용자 재계획",
  pathDeviation: "경로이탈",
  priorTarget: "선행표적",
  mandatory: "강제명령",
  collabBase: "협업기저",
  targetRediscovery: "표적재발견",
  coverageUpdate: "엄호갱신",
};

const getReplanReasonText = (reason) => {
  if (!reason) return "재계획";
  return REPLAN_REASONS[reason] || reason;
};
