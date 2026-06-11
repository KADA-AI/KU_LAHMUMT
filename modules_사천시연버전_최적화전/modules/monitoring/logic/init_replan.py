# -*- coding: utf-8 -*-
from __future__ import annotations

import json
from pathlib import Path
from typing import Callable, Iterable

from modules.common import db_paths
from modules.mission_planning.MissionPlanner.data_def.id_allocator import (
    next_mission_plan_id,
    reserve_mission_plan_ids,
)

DEFAULT_INPUT_MISSION_IDS = (107, 108)
DEFAULT_OPTION_CODES = (1,)


def _coerce_int_list(values: Iterable[object] | None) -> list[int]:
    result: list[int] = []
    if values is None:
        return result
    for value in values:
        try:
            result.append(int(value))
        except Exception:
            continue
    return result


def _read_input_ids(path: Path) -> list[int]:
    try:
        data = json.loads(path.read_text(encoding="utf-8", errors="ignore"))
    except Exception:
        return []
    ids: list[int] = []
    for item in data.get("inputMissionList") or []:
        try:
            ids.append(int(item.get("inputMissionID")))
        except Exception:
            continue
    return ids


def collect_input_mission_ids(
    db_root: Path | str | None = None,
    *,
    fallback: Iterable[int] = DEFAULT_INPUT_MISSION_IDS,
) -> list[int]:
    ids: list[int] = []
    if db_root is None:
        base = db_paths.get_db_subpath("InputMissionPlan")
        single = db_paths.get_db_subpath("InputMissionPlan.json")
    else:
        root = Path(db_root)
        base = root / "InputMissionPlan"
        single = root / "InputMissionPlan.json"

    candidates: list[Path] = []
    if base.exists() and base.is_dir():
        candidates.extend(p for p in base.glob("*.json") if p.is_file())
    if single.exists():
        candidates.append(single)

    for fp in candidates:
        ids.extend(_read_input_ids(fp))

    unique_ids = sorted({value for value in ids if value is not None})
    if not unique_ids:
        unique_ids = _coerce_int_list(fallback)
    return unique_ids


def allocate_mission_plan_ids(
    count: int,
    allocator: Callable[[], int] = next_mission_plan_id,
) -> list[int]:
    try:
        total = max(int(count), 0)
    except Exception:
        return []
    if total <= 0:
        return []
    if allocator is next_mission_plan_id:
        try:
            return [int(value) for value in reserve_mission_plan_ids(total)]
        except Exception:
            pass
    allocated: list[int] = []
    for _ in range(total):
        try:
            allocated.append(int(allocator()))
        except Exception:
            return []
    return allocated


def build_replan_context(
    mission_ids: Iterable[object],
    plan_ids: Iterable[object],
    *,
    option_codes: Iterable[object] | None = None,
    replan_level: int = 1,
) -> dict:
    codes = _coerce_int_list(option_codes or DEFAULT_OPTION_CODES)
    return {
        "mission_ids": _coerce_int_list(mission_ids),
        "plan_ids": _coerce_int_list(plan_ids),
        "option_names": codes,
        "replan_level": int(replan_level),
    }
