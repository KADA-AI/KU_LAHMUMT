from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Optional

from . import replan_store


_STORE_NAME = "next_collab_replan"
_DETAIL_PREFIX = "next_collab_detail"


def save_detail(mission_plan_id: int, payload: Dict[str, Any]) -> Path:
    return replan_store.save_detail(_STORE_NAME, _DETAIL_PREFIX, mission_plan_id, payload)


def load_detail(mission_plan_id: int) -> Optional[Dict[str, Any]]:
    return replan_store.load_detail(_STORE_NAME, _DETAIL_PREFIX, mission_plan_id)


def load_latest_detail_at_or_before(
    mission_plan_id: int,
    *,
    input_mission_package_id: int | None = None,
) -> Optional[Dict[str, Any]]:
    """Recover the last explicit next-collab transition for a derived plan."""

    try:
        upper_plan_id = int(mission_plan_id)
    except (TypeError, ValueError):
        return None
    try:
        expected_package_id = (
            int(input_mission_package_id)
            if input_mission_package_id is not None
            else None
        )
    except (TypeError, ValueError):
        expected_package_id = None

    try:
        store_dir = replan_store._store_dir(_STORE_NAME)
        candidates = list(store_dir.glob(f"{_DETAIL_PREFIX}_*.json"))
    except Exception:
        return None

    ranked_paths: list[tuple[int, Path]] = []
    prefix = f"{_DETAIL_PREFIX}_"
    for path in candidates:
        stem = str(path.stem)
        if not stem.startswith(prefix):
            continue
        try:
            candidate_plan_id = int(stem[len(prefix) :])
        except (TypeError, ValueError):
            continue
        if candidate_plan_id <= upper_plan_id:
            ranked_paths.append((candidate_plan_id, path))

    for _candidate_plan_id, path in sorted(ranked_paths, reverse=True):
        try:
            detail = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            continue
        if not isinstance(detail, dict):
            continue
        if expected_package_id is not None:
            try:
                detail_package_id = int(detail.get("inputMissionPackageID"))
            except (TypeError, ValueError):
                continue
            if detail_package_id != expected_package_id:
                continue
        return detail
    return None


def save_event(stage: str, payload: Dict[str, Any]) -> Path:
    return replan_store.save_event(_STORE_NAME, stage, payload)
