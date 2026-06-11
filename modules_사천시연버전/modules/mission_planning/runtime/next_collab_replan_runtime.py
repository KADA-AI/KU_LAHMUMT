from __future__ import annotations

from typing import Any, Dict, Iterable, Optional

from . import next_collab_replan_store


REPLAN_REASON = "_협업 기저 전환으로 인한 재계획"
OPTION_NAME = "비행/촬영"
TRIGGER_TYPE = "nextCollaborativeMission"
REASON_KEYWORDS = ("협업 기저 전환", "_협업 기저 전환", "조기 임무 전환")


def is_next_collab_reason_text(value: object | None) -> bool:
    text = str(value or "")
    return any(keyword in text for keyword in REASON_KEYWORDS)


def load_next_collab_detail(plan_ids: Iterable[object] | None) -> Optional[Dict[str, Any]]:
    for value in plan_ids or []:
        try:
            plan_id = int(value)
        except Exception:
            continue
        payload = next_collab_replan_store.load_detail(plan_id)
        if payload:
            return payload
    return None


def record_next_collab_event(stage: str, payload: Dict[str, Any]) -> None:
    try:
        next_collab_replan_store.save_event(stage, payload)
    except Exception:
        pass
