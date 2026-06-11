from __future__ import annotations

import time
import json
import os
import threading
from datetime import datetime, timezone
from typing import Any, Callable, Dict, Optional


REPLAN_CHECKPOINT_PHASES = (
    "0902_received",
    "context_parsed",
    "pipeline_selected",
    "source_artifacts_loaded",
    "branch_decision",
    "id_reserve",
    "algorithm",
    "build",
    "validation",
    "write",
    "delivery",
    "fallback_noop_failure",
)


def new_replan_transaction_id(prefix: str = "replan") -> str:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    return f"{prefix}-{os.getpid()}-{threading.get_ident()}-{stamp}"


def emit_pipeline_event(
    *,
    event: str,
    module: str = "mission_planning",
    process_id: int | None = None,
    thread_name: str | None = None,
    replan_transaction_id: str | None = None,
    trigger: str | None = None,
    trigger_type: str | None = None,
    pipeline: str | None = None,
    phase: str | None = None,
    mission_plan_id: int | None = None,
    aircraft_id: int | None = None,
    elapsed_ms: float | None = None,
    outcome: str | None = None,
    reason: str | None = None,
    extra: Optional[Dict[str, Any]] = None,
) -> None:
    payload: Dict[str, Any] = {
        "event": str(event),
        "module": str(module),
        "processId": int(process_id if process_id is not None else os.getpid()),
        "threadName": str(thread_name or threading.current_thread().name),
        "timestamp": datetime.now(timezone.utc).isoformat(),
    }
    optional = {
        "replanTransactionId": replan_transaction_id,
        "trigger": trigger,
        "triggerType": trigger_type,
        "pipeline": pipeline,
        "phase": phase,
        "missionPlanID": mission_plan_id,
        "aircraftID": aircraft_id,
        "elapsedMs": elapsed_ms,
        "outcome": outcome,
        "reason": reason,
    }
    for key, value in optional.items():
        if value is not None:
            payload[key] = value
    if extra:
        payload.update(extra)
    try:
        from modules.common.process_console import emit_process_log

        emit_process_log(module, "[REPLAN][EVENT] " + json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
    except Exception:
        return


def infer_checkpoint_outcome(name: str) -> str:
    normalized = str(name or "").strip().lower()
    if any(token in normalized for token in ("fail", "error", "crash", "exception")):
        return "failure"
    if any(token in normalized for token in ("suppress", "skip", "noop", "blocked", "fallback")):
        return "skipped"
    if any(token in normalized for token in ("queued", "sent", "done", "complete", "evaluated", "success")):
        return "ok"
    return "checkpoint"


def normalize_replan_checkpoint(name: str) -> str:
    normalized = str(name or "").strip()
    if not normalized:
        return "checkpoint"
    lower = normalized.lower()
    if lower.startswith("0902"):
        return "0902_received"
    if "context" in lower:
        return "context_parsed"
    if "selected" in lower or "pipeline" in lower and "scheduled" not in lower:
        return "pipeline_selected"
    if "source" in lower or "artifact" in lower:
        return "source_artifacts_loaded"
    if "branch" in lower or "decision" in lower:
        return "branch_decision"
    if "reserve" in lower or "id_" in lower:
        return "id_reserve"
    if "algorithm" in lower or "planner" in lower:
        return "algorithm"
    if "build" in lower:
        return "build"
    if "validation" in lower or "validator" in lower:
        return "validation"
    if "write" in lower or "persist" in lower:
        return "write"
    if any(token in lower for token in ("0301", "0305", "0702", "0901", "0903", "delivery", "queued", "sent")):
        return "delivery"
    if any(token in lower for token in ("fallback", "noop", "skip", "fail", "failure", "blocked")):
        return "fallback_noop_failure"
    return normalized


def emit_replan_checkpoint(
    *,
    name: str,
    replan_transaction_id: str | None = None,
    trigger: str | None = None,
    trigger_type: str | None = None,
    pipeline: str | None = None,
    mission_plan_id: int | None = None,
    elapsed_ms: float | None = None,
    outcome: str | None = None,
    reason: str | None = None,
    extra: Optional[Dict[str, Any]] = None,
) -> None:
    detail = dict(extra or {})
    detail.setdefault("checkpoint", str(name or ""))
    emit_pipeline_event(
        event="replan_checkpoint",
        replan_transaction_id=replan_transaction_id,
        trigger=trigger,
        trigger_type=trigger_type,
        pipeline=pipeline,
        phase=normalize_replan_checkpoint(name),
        mission_plan_id=mission_plan_id,
        elapsed_ms=elapsed_ms,
        outcome=outcome or infer_checkpoint_outcome(name),
        reason=reason,
        extra=detail,
    )


class PipelinePhaseTimer:
    """Append-only phase duration collector for pipeline diagnostics."""

    def __init__(
        self,
        *,
        pipeline: str | None = None,
        replan_transaction_id: str | None = None,
        emit_events: bool = False,
    ) -> None:
        now = time.perf_counter()
        self._started_at = now
        self._last_mark = now
        self._timings: Dict[str, float] = {}
        self.pipeline = pipeline
        self.replan_transaction_id = replan_transaction_id
        self.emit_events = bool(emit_events)

    def mark(self, name: str) -> float:
        label = str(name or "").strip()
        if not label:
            label = "phase"
        now = time.perf_counter()
        elapsed_ms = max(0.0, (now - self._last_mark) * 1000.0)
        self._last_mark = now
        self._timings[label] = round(elapsed_ms, 3)
        if self.emit_events:
            emit_pipeline_event(
                event="pipeline_phase",
                replan_transaction_id=self.replan_transaction_id,
                pipeline=self.pipeline,
                phase=label,
                elapsed_ms=round(elapsed_ms, 3),
                outcome="ok",
            )
        return elapsed_ms

    def total(self) -> float:
        elapsed_ms = max(0.0, (time.perf_counter() - self._started_at) * 1000.0)
        self._timings["total"] = round(elapsed_ms, 3)
        if self.emit_events:
            emit_pipeline_event(
                event="pipeline_total",
                replan_transaction_id=self.replan_transaction_id,
                pipeline=self.pipeline,
                phase="total",
                elapsed_ms=round(elapsed_ms, 3),
                outcome="ok",
            )
        return elapsed_ms

    def snapshot(self, *, include_total: bool = True) -> Dict[str, float]:
        if include_total:
            self.total()
        return dict(self._timings)


class PipelineLogManager:
    def __init__(
        self,
        *,
        emit_callback: Callable[[Dict[str, Any]], None],
        log_tab_provider: Callable[[], Any],
        sanitize_reason: Callable[[Any, str], str],
    ) -> None:
        self._emit = emit_callback
        self._log_tab_provider = log_tab_provider
        self._sanitize_reason = sanitize_reason
        self._counter = 0

    def handle_event(self, payload: Dict[str, Any]) -> None:
        tab = self._log_tab_provider()
        if not tab or not isinstance(payload, dict):
            return
        action = payload.get("action")
        session_id = payload.get("session_id")
        if not session_id:
            return
        if action == "start":
            tab.start_session(session_id, payload.get("meta") or {})
        elif action == "append":
            tab.append_event(
                session_id,
                payload.get("level") or "info",
                payload.get("message") or "",
                detail=payload.get("detail"),
                timestamp=payload.get("timestamp"),
            )
        elif action == "finish":
            tab.finish_session(
                session_id,
                payload.get("status") or "done",
                summary=payload.get("summary"),
            )

    def open_session(self, ctx: Dict[str, Any], reason: str) -> Optional[str]:
        if not self._log_tab_provider():
            return None
        self._counter += 1
        session_id = f"run-{self._counter:04d}"
        meta = {
            "timestamp": time.time(),
            "reason": self._sanitize_reason(reason, "init-plan"),
            "plan_ids": list(ctx.get("plan_ids") or []),
            "mission_ids": list(ctx.get("mission_ids") or []),
            "replan_level": ctx.get("replan_level") or ctx.get("replanLevel"),
        }
        self._emit({"action": "start", "session_id": session_id, "meta": meta})
        return session_id

    def log_event(
        self,
        session_id: Optional[str],
        level: str,
        message: str,
        *,
        detail: Optional[Dict[str, Any]] = None,
    ) -> None:
        if not session_id:
            return
        payload = {
            "action": "append",
            "session_id": session_id,
            "level": level,
            "message": message,
            "timestamp": time.time(),
        }
        if detail is not None:
            payload["detail"] = detail
        self._emit(payload)

    def close_session(
        self,
        session_id: Optional[str],
        status: str,
        *,
        summary: Optional[Dict[str, Any]] = None,
    ) -> None:
        if not session_id:
            return
        payload = {"action": "finish", "session_id": session_id, "status": status, "summary": summary}
        self._emit(payload)
