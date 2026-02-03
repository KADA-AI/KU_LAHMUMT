const timers = new WeakMap();

const getContainer = () => document.getElementById("status");

const clearTimer = (item) => {
  const timer = timers.get(item);
  if (timer) {
    clearTimeout(timer);
    timers.delete(item);
  }
};

const removeItem = (item) => {
  if (!item || !item.parentNode) {
    return;
  }
  clearTimer(item);
  item.classList.add("is-hiding");
  const finalize = () => {
    if (item.parentNode) {
      item.parentNode.removeChild(item);
    }
  };
  item.addEventListener("transitionend", finalize, { once: true });
  setTimeout(finalize, 220);
};

const normalizeLevel = (level) =>
  level && typeof level === "string" ? level.trim().toLowerCase() : "";

const escapeKey = (value) => {
  if (typeof CSS !== "undefined" && typeof CSS.escape === "function") {
    return CSS.escape(String(value));
  }
  return String(value).replace(/["\\]/g, "\\$&");
};

const applyLevel = (item, level) => {
  const levels = ["info", "warn", "error", "success"];
  levels.forEach((name) => item.classList.remove(`status-item--${name}`));
  if (level && levels.includes(level)) {
    item.classList.add(`status-item--${level}`);
  }
};

export const clearStatus = (key) => {
  const container = getContainer();
  if (!container) {
    return;
  }
  if (!key) {
    container.innerHTML = "";
    return;
  }
  const escaped = escapeKey(key);
  const item = container.querySelector(`[data-status-key="${escaped}"]`);
  if (item) {
    removeItem(item);
  }
};

export const logStatus = (message, options = {}) => {
  const container = getContainer();
  if (!container) {
    return;
  }
  const key = options.key ? String(options.key) : "";
  const ttlMs = Number.isFinite(options.ttlMs) ? Number(options.ttlMs) : 4000;
  const level = normalizeLevel(options.level);
  if (!message) {
    if (key) {
      clearStatus(key);
    }
    return;
  }

  const escapedKey = key ? escapeKey(key) : "";
  let item = escapedKey
    ? container.querySelector(`[data-status-key="${escapedKey}"]`)
    : null;
  if (!item) {
    item = document.createElement("div");
    item.className = "status-item";
    if (key) {
      item.dataset.statusKey = key;
    }
    container.appendChild(item);
  }

  item.textContent = String(message);
  applyLevel(item, level);
  clearTimer(item);
  if (ttlMs > 0) {
    const timer = setTimeout(() => removeItem(item), ttlMs);
    timers.set(item, timer);
  }
};
