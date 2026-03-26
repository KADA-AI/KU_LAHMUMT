export const initRightSidePanel = () => {
  const toggle = document.getElementById("right-side-toggle");
  const panel = document.getElementById("right-side-panel");
  if (!toggle || !panel) {
    return;
  }

  const setOpen = (open) => {
    const next = Boolean(open);
    panel.classList.toggle("is-open", next);
    panel.setAttribute("aria-hidden", next ? "false" : "true");
    toggle.setAttribute("aria-expanded", next ? "true" : "false");
    toggle.classList.toggle("is-open", next);
    toggle.textContent = next ? ">" : "<";
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

  setOpen(panel.classList.contains("is-open"));
};
