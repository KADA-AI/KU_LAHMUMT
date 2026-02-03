export const initSidePanel = () => {
  const toggle = document.getElementById("right-panel-toggle");
  const panel = document.getElementById("right-panel");
  if (!toggle || !panel) {
    return;
  }

  const setOpen = (open) => {
    const next = Boolean(open);
    panel.classList.toggle("is-open", next);
    panel.setAttribute("aria-hidden", next ? "false" : "true");
    toggle.setAttribute("aria-expanded", next ? "true" : "false");
    toggle.classList.toggle("is-open", next);
    toggle.textContent = next ? "▼" : "▲";
  };

  window.set0401PanelOpen = setOpen;
  window.open0401Panel = () => setOpen(true);
  window.close0401Panel = () => setOpen(false);

  toggle.addEventListener("click", (event) => {
    event.stopPropagation();
    setOpen(!panel.classList.contains("is-open"));
  });

  panel.addEventListener("click", (event) => {
    event.stopPropagation();
  });

  document.addEventListener("click", (event) => {
    const target = event.target;
    const keepOpen =
      Boolean(target.closest("#agent-panel")) ||
      Boolean(target.closest("#left-controls")) ||
      Boolean(target.closest("#bottom-controls")) ||
      Boolean(target.closest(".agent-panel")) ||
      Boolean(target.closest(".toolbar")) ||
      Boolean(target.closest(".top-right-controls")) ||
      Boolean(target.closest("#right-side-panel")) ||
      Boolean(target.closest("#right-side-toggle"));
    if (keepOpen) {
      return;
    }
    if (!panel.contains(target) && target !== toggle) {
      setOpen(false);
    }
  });

  setOpen(panel.classList.contains("is-open"));
};
