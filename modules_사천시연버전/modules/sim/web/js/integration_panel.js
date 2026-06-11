const STATE_ENDPOINT = "/api/integration/state";
const SEND_ENDPOINT = "/api/integration/send";
const GENERATE_ENDPOINT = "/api/integration/generate";
const PAYLOAD_ENDPOINT = "/api/integration/payload";

const POLL_INTERVAL_MS = 1000;

const pillClassFor = (level) => {
  if (level === "ok") return "io-pill io-pill-ok";
  if (level === "warn") return "io-pill io-pill-warn";
  if (level === "bad") return "io-pill io-pill-bad";
  return "io-pill io-pill-muted";
};

const escapeHtml = (value) =>
  String(value)
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;")
    .replace(/'/g, "&#039;");

const buildRow = (row, type) => {
  const state = row.state || "Idle";
  const level = row.level || "muted";
  const pill = `<span class="${pillClassFor(level)}">${escapeHtml(state)}</span>`;
  if (type === "tx") {
    const sendLabel = row.periodic ? (row.running ? "Stop" : "Start") : "Send";
    return `
      <tr data-msg-id="${escapeHtml(row.id)}">
        <td>${escapeHtml(row.id)}</td>
        <td title="${escapeHtml(row.name)}">${escapeHtml(row.name)}</td>
        <td>${pill}</td>
        <td class="io-col-action">
          <button type="button" class="io-action" data-io-action="send">${sendLabel}</button>
        </td>
        <td class="io-col-action">
          <button type="button" class="io-action" data-io-action="view">View</button>
        </td>
      </tr>
    `;
  }
  return `
    <tr data-msg-id="${escapeHtml(row.id)}">
      <td>${escapeHtml(row.id)}</td>
      <td title="${escapeHtml(row.name)}">${escapeHtml(row.name)}</td>
      <td>${pill}</td>
      <td class="io-col-action">
        <button type="button" class="io-action" data-io-action="view">View</button>
      </td>
    </tr>
  `;
};

const renderTable = (tbody, rows, type) => {
  if (!tbody) return;
  const html = rows.map((row) => buildRow(row, type)).join("");
  tbody.innerHTML = html || `<tr><td colspan="${type === "tx" ? 5 : 4}">No data</td></tr>`;
};

const renderLog = (container, lines) => {
  if (!container) return;
  const items = lines && lines.length ? lines : ["No log yet."];
  container.innerHTML = items.map((line) => `<div>${escapeHtml(line)}</div>`).join("");
};

export const initIntegrationPanel = () => {
  const txBody = document.getElementById("io-tx-body");
  const rxBody = document.getElementById("io-rx-body");
  const txLog = document.getElementById("io-tx-log");
  const rxLog = document.getElementById("io-rx-log");
  const connection = document.getElementById("io-connection");
  const modal = document.getElementById("io-modal");
  const modalTitle = document.getElementById("io-modal-title");
  const modalBody = document.getElementById("io-modal-body");
  const modalClose = document.getElementById("io-modal-close");

  if (!txBody || !rxBody || !txLog || !rxLog || !connection) {
    return;
  }

  let pollTimer = null;

  const setConnection = (online, message) => {
    connection.classList.toggle("io-connection-on", online);
    connection.classList.toggle("io-connection-off", !online);
    connection.textContent = message || (online ? "ONLINE" : "OFFLINE");
  };

  const openModal = (title, payload) => {
    if (!modal || !modalTitle || !modalBody) return;
    modalTitle.textContent = title || "Payload";
    if (payload === null || payload === undefined) {
      modalBody.textContent = "No payload.";
    } else if (typeof payload === "string") {
      modalBody.textContent = payload;
    } else {
      try {
        modalBody.textContent = JSON.stringify(payload, null, 2);
      } catch (err) {
        modalBody.textContent = String(payload);
      }
    }
    modal.classList.add("is-open");
    modal.setAttribute("aria-hidden", "false");
  };

  const closeModal = () => {
    if (!modal) return;
    modal.classList.remove("is-open");
    modal.setAttribute("aria-hidden", "true");
  };

  const fetchState = async () => {
    try {
      const res = await fetch(STATE_ENDPOINT, { cache: "no-store" });
      if (!res.ok) {
        setConnection(false, "ERROR");
        return;
      }
      const data = await res.json();
      if (!data || !data.ok) {
        setConnection(false, "OFFLINE");
        return;
      }
      setConnection(Boolean(data.enabled), data.enabled ? "ONLINE" : "OFFLINE");
      renderTable(txBody, data.tx || [], "tx");
      renderTable(rxBody, data.rx || [], "rx");
      renderLog(txLog, data.logs?.tx || []);
      renderLog(rxLog, data.logs?.rx || []);
    } catch (err) {
      setConnection(false, "OFFLINE");
    }
  };

  const handleTableClick = async (event) => {
    const btn = event.target.closest("button[data-io-action]");
    if (!btn) return;
    const action = btn.dataset.ioAction;
    const row = btn.closest("tr");
    if (!row) return;
    const msgId = row.dataset.msgId;
    const table = row.closest("tbody");
    const type = table?.dataset.ioType || "tx";

    if (action === "send") {
      btn.disabled = true;
      try {
        const res = await fetch(SEND_ENDPOINT, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ msgId }),
        });
        if (!res.ok) {
          openModal(`SEND ${msgId}`, "Failed to send.");
        } else {
          const data = await res.json();
          if (!data || !data.ok) {
            openModal(`SEND ${msgId}`, data?.error || "Failed to send.");
          }
        }
      } catch (err) {
        openModal(`SEND ${msgId}`, "Failed to send.");
      } finally {
        btn.disabled = false;
        fetchState();
      }
      return;
    }

    if (action === "view") {
      try {
        const url = new URL(PAYLOAD_ENDPOINT, window.location.origin);
        url.searchParams.set("msgId", msgId);
        url.searchParams.set("type", type);
        const res = await fetch(url.toString(), { cache: "no-store" });
        let data = null;
        if (res.ok) {
          data = await res.json();
        }
        if (!data || !data.ok) {
          const genRes = await fetch(GENERATE_ENDPOINT, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ msgId }),
          });
          if (genRes.ok) {
            const genData = await genRes.json();
            if (genData && genData.ok) {
              openModal(`${msgId} GENERATED`, genData.payload);
              return;
            }
            openModal(`VIEW ${msgId}`, genData?.error || "No payload.");
            return;
          }
          openModal(`VIEW ${msgId}`, "No payload.");
          return;
        }
        openModal(`${msgId} ${type.toUpperCase()}`, data.payload);
      } catch (err) {
        openModal(`VIEW ${msgId}`, "No payload.");
      }
    }
  };

  txBody.addEventListener("click", handleTableClick);
  rxBody.addEventListener("click", handleTableClick);

  if (modal) {
    modal.addEventListener("click", (event) => {
      if (event.target === modal) {
        closeModal();
      }
    });
  }

  if (modalClose) {
    modalClose.addEventListener("click", closeModal);
  }

  fetchState();
  pollTimer = window.setInterval(fetchState, POLL_INTERVAL_MS);

  return () => {
    if (pollTimer) {
      window.clearInterval(pollTimer);
      pollTimer = null;
    }
  };
};
