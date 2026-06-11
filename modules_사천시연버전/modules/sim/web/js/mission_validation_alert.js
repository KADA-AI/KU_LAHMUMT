import { logStatus } from "./status_log.js";

const STATUS_KEY = "mission-validation";
const MAX_INLINE_LENGTH = 180;

const trimText = (value, maxLength = MAX_INLINE_LENGTH) => {
  const text = String(value ?? "").replace(/\s+/g, " ").trim();
  if (text.length <= maxLength) {
    return text;
  }
  return `${text.slice(0, Math.max(0, maxLength - 1))}...`;
};

const issueSummary = (issue) => {
  if (!issue || typeof issue !== "object") {
    return "";
  }
  const path = issue.path ? `${issue.path}: ` : "";
  return trimText(`${path}${issue.message || issue.code || "unknown issue"}`);
};

export const reportMissionValidation = (validation, context = "Mission") => {
  if (!validation || typeof validation !== "object") {
    return;
  }
  const issueCount = Number(validation.issueCount || 0);
  if (!Number.isFinite(issueCount) || issueCount <= 0) {
    logStatus("", { key: STATUS_KEY });
    return;
  }

  const counts = validation.counts || {};
  const errorCount = Number(counts.error || 0);
  const warnCount = Number(counts.warn || 0);
  const level = errorCount > 0 ? "error" : "warn";
  const firstIssue = Array.isArray(validation.issues) ? validation.issues[0] : null;
  const detail = issueSummary(firstIssue);
  const countText =
    errorCount > 0
      ? `${errorCount} error${errorCount === 1 ? "" : "s"}`
      : `${warnCount || issueCount} warning${(warnCount || issueCount) === 1 ? "" : "s"}`;
  const message = detail
    ? `${context} validation ${countText}: ${detail}`
    : `${context} validation ${countText}`;

  if (typeof window !== "undefined") {
    window.lastMissionValidation = {
      context,
      validation,
      receivedAt: Date.now(),
    };
  }
  console.warn("[MissionValidation]", context, validation);
  logStatus(message, {
    key: STATUS_KEY,
    level,
    ttlMs: 12000,
  });
};
