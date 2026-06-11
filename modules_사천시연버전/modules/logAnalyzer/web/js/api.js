/**
 * API wrappers for the Log Analyzer backend.
 */

export const fetchScenarios = async () => {
  const res = await fetch("/api/scenarios");
  if (!res.ok) throw new Error(`scenarios: ${res.status}`);
  const body = await res.json();
  if (!body.ok) throw new Error(body.error || "unknown");
  return body.data;
};

export const fetchScenario = async (name) => {
  const res = await fetch(`/api/scenario?name=${encodeURIComponent(name)}`);
  if (!res.ok) throw new Error(`scenario: ${res.status}`);
  const body = await res.json();
  if (!body.ok) throw new Error(body.error || "unknown");
  return { data: body.data, version: body.version || 0 };
};

export const pollScenario = async (name, version) => {
  const res = await fetch(`/api/scenario/poll?name=${encodeURIComponent(name)}&v=${version}`);
  if (!res.ok) return null;
  const body = await res.json();
  if (!body.ok) return null;
  if (!body.changed) return { changed: false, version: body.version };
  return { changed: true, version: body.version, data: body.data };
};

export const fetchDynamicsAnalysis = async (name) => {
  const res = await fetch(`/api/scenario/dynamics?name=${encodeURIComponent(name)}`, {
    cache: "no-store",
  });
  const body = await res.json();
  if (!res.ok || !body.ok) {
    throw new Error((body && body.error) || `dynamics: ${res.status}`);
  }
  return body;
};

export const saveDynamicsAnalysis = async (name) => {
  const res = await fetch(`/api/scenario/dynamics/save?name=${encodeURIComponent(name)}`, {
    method: "POST",
    cache: "no-store",
  });
  const body = await res.json();
  if (!res.ok || !body.ok) {
    throw new Error((body && body.error) || `dynamics save: ${res.status}`);
  }
  return body;
};
