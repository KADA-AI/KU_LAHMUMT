from __future__ import annotations

import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

_ROOT = Path(__file__).resolve().parents[3]
_ROOT_STR = str(_ROOT)
if _ROOT_STR not in sys.path:
    sys.path.insert(0, _ROOT_STR)

from modules.common import db_paths
from modules.mission_planning.runtime.debug_artifacts import write_debug_json


def _json_safe(value: Any) -> Any:
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if isinstance(value, dict):
        return {str(k): _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_json_safe(v) for v in value]
    try:
        return _json_safe(dict(value))
    except Exception:
        return str(value)


class MissionPlanRunLog:
    def __init__(
        self,
        base_dir: Path,
        *,
        session_id: Optional[str],
        reason: str,
        context: Dict[str, Any],
        replan_level: Optional[int],
    ) -> None:
        self._base_dir = base_dir
        self.session_id = session_id
        self.reason = reason
        self.replan_level = replan_level
        self.started_at = datetime.now(timezone.utc)
        self.context = _json_safe(context or {})
        self.plan_ids: List[int] = []
        self.steps: List[Dict[str, Any]] = []
        self.issues: List[Dict[str, Any]] = []
        self.messages: List[Dict[str, Any]] = []
        self.summary: Dict[str, Any] = {}
        self.stop_reason: Optional[str] = None
        self.written_paths: List[str] = []

    def set_plan_ids(self, plan_ids: Any) -> None:
        ids: List[int] = []
        for value in plan_ids or []:
            try:
                ids.append(int(value))
            except Exception:
                continue
        self.plan_ids = ids

    def add_message(self, text: str) -> None:
        try:
            stamp = datetime.now(timezone.utc).isoformat()
            self.messages.append({"ts": stamp, "text": str(text)})
        except Exception:
            pass

    def add_step(self, name: str, status: str, *, detail: Optional[Dict[str, Any]] = None, message: str = "") -> None:
        entry: Dict[str, Any] = {"step": name, "status": status}
        if message:
            entry["message"] = message
        if detail is not None:
            entry["detail"] = _json_safe(detail)
        self.steps.append(entry)

    def add_issue(self, code: str, message: Optional[str] = None, detail: Optional[Dict[str, Any]] = None) -> None:
        entry: Dict[str, Any] = {"code": code}
        if message:
            entry["message"] = message
        if detail is not None:
            entry["detail"] = _json_safe(detail)
        self.issues.append(entry)

    def set_stop_reason(self, reason: str) -> None:
        self.stop_reason = reason

    def update_summary(self, payload: Dict[str, Any]) -> None:
        try:
            self.summary.update(_json_safe(payload))
        except Exception:
            pass

    def _path_for_plan(self, plan_id: Optional[int]) -> Path:
        token = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        if plan_id is not None:
            primary = self._base_dir / f"missionPlan_{plan_id}.json"
            if primary.exists():
                return self._base_dir / f"missionPlan_{plan_id}_{token}.json"
            return primary
        return self._base_dir / f"missionPlan_pending_{token}.json"

    def finalize(self, status: str, *, summary: Optional[Dict[str, Any]] = None) -> List[str]:
        if summary:
            self.update_summary(summary)
        payload_base = {
            "timestamp": self.started_at.isoformat(),
            "status": status,
            "stopReason": self.stop_reason,
            "reason": self.reason,
            "replanLevel": self.replan_level,
            "sessionId": self.session_id,
            "context": self.context,
            "planIds": self.plan_ids,
            "summary": self.summary,
            "steps": self.steps,
            "issues": self.issues,
            "messages": self.messages,
        }
        targets = self.plan_ids or [None]
        written: List[str] = []
        for plan_id in targets:
            payload = dict(payload_base)
            payload["missionPlanID"] = plan_id
            path = self._path_for_plan(plan_id)
            payload["logPath"] = str(path)
            try:
                path.parent.mkdir(parents=True, exist_ok=True)
                if write_debug_json(path, payload, pretty=True, ensure_ascii=False, skip_if_unchanged=False):
                    written.append(str(path))
            except Exception:
                continue
        self.written_paths = written
        return written


class MissionPlanFileLogger:
    def __init__(self) -> None:
        self._active_run: Optional[MissionPlanRunLog] = None

    @staticmethod
    def _safe_level(ctx: Dict[str, Any]) -> Optional[int]:
        try:
            return int(ctx.get("replan_level", ctx.get("replanLevel")))
        except Exception:
            return None

    def start_run(
        self,
        ctx: Dict[str, Any],
        reason: str,
        *,
        session_id: Optional[str],
    ) -> Optional[MissionPlanRunLog]:
        try:
            base_dir = db_paths.ensure_db_payload("DSS_Internal")
        except Exception:
            return None
        run = MissionPlanRunLog(
            base_dir,
            session_id=session_id,
            reason=reason,
            context=dict(ctx or {}),
            replan_level=self._safe_level(ctx or {}),
        )
        run.set_plan_ids((ctx or {}).get("plan_ids") or [])
        self._active_run = run
        return run

    def current_run(self) -> Optional[MissionPlanRunLog]:
        return self._active_run

    def clear_active(self) -> None:
        self._active_run = None

    def log_blocked(self, ctx: Dict[str, Any], reason: str, message: str, *, session_id: Optional[str] = None) -> None:
        run = self.start_run(ctx, reason, session_id=session_id)
        if not run:
            return
        run.add_step("preflight", "blocked", detail={"message": message})
        run.add_issue("blocked", message=message)
        run.set_stop_reason(message)
        run.finalize("blocked", summary={"blocked": message})
        self.clear_active()
