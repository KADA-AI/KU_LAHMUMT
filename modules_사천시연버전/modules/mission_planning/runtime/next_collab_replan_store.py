from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Optional

from . import replan_store


_STORE_NAME = "next_collab_replan"
_DETAIL_PREFIX = "next_collab_detail"


def save_detail(mission_plan_id: int, payload: Dict[str, Any]) -> Path:
    return replan_store.save_detail(_STORE_NAME, _DETAIL_PREFIX, mission_plan_id, payload)


def load_detail(mission_plan_id: int) -> Optional[Dict[str, Any]]:
    return replan_store.load_detail(_STORE_NAME, _DETAIL_PREFIX, mission_plan_id)


def save_event(stage: str, payload: Dict[str, Any]) -> Path:
    return replan_store.save_event(_STORE_NAME, stage, payload)
