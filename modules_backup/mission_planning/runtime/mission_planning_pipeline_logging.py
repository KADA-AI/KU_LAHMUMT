from __future__ import annotations

import time
from typing import Any, Callable, Dict, Optional


class PipelinePhaseTimer:
    """Append-only phase duration collector for pipeline diagnostics."""

    def __init__(self) -> None:
        now = time.perf_counter()
        self._started_at = now
        self._last_mark = now
        self._timings: Dict[str, float] = {}

    def mark(self, name: str) -> float:
        label = str(name or "").strip()
        if not label:
            label = "phase"
        now = time.perf_counter()
        elapsed_ms = max(0.0, (now - self._last_mark) * 1000.0)
        self._last_mark = now
        self._timings[label] = round(elapsed_ms, 3)
        return elapsed_ms

    def total(self) -> float:
        elapsed_ms = max(0.0, (time.perf_counter() - self._started_at) * 1000.0)
        self._timings["total"] = round(elapsed_ms, 3)
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
