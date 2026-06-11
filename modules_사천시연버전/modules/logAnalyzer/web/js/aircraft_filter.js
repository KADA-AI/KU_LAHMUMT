/**
 * Aircraft filter — 6 toggle buttons (LAH1-3, UAV1-3) at bottom-left.
 */

import { AGENT_COLORS, AGENT_LABELS } from "./palette.js";

const AIRCRAFT_IDS = [1, 2, 3, 4, 5, 6];

/**
 * @param {HTMLElement} container - The #aircraft-filter element
 * @param {Object} callbacks
 * @param {Function} callbacks.onToggle - Called with (aircraftId, visible)
 * @returns {{ getVisibility(): Object<number, boolean>, reset(): void }}
 */
export const createAircraftFilter = (container, { onToggle }) => {
  const visibility = {};
  const buttons = {};

  for (const id of AIRCRAFT_IDS) {
    visibility[id] = true;

    const btn = document.createElement("button");
    btn.type = "button";
    btn.className = "af-btn is-active";
    btn.style.setProperty("--agent-color", AGENT_COLORS[id] || "#888");
    btn.textContent = AGENT_LABELS[id] || `${id}`;
    btn.title = AGENT_LABELS[id] || `Aircraft ${id}`;
    btn.setAttribute("data-aircraft", id);

    btn.addEventListener("click", () => {
      visibility[id] = !visibility[id];
      btn.classList.toggle("is-active", visibility[id]);
      onToggle(id, visibility[id]);
    });

    buttons[id] = btn;
    container.appendChild(btn);
  }

  const getVisibility = () => ({ ...visibility });

  const reset = () => {
    for (const id of AIRCRAFT_IDS) {
      visibility[id] = true;
      buttons[id].classList.add("is-active");
    }
  };

  return { getVisibility, reset };
};
