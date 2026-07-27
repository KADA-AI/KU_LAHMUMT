# -*- coding: utf-8 -*-
from __future__ import annotations

import copy
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from modules.common import db_paths


@dataclass(frozen=True)
class ReexecuteInputCloneResult:
    ok: bool
    message: str
    db_root: Path | None = None
    source_package_id: int | None = None
    new_package_id: int | None = None
    source_input_id: int | None = None
    new_input_id: int | None = None
    output_path: Path | None = None
    payload: dict[str, Any] | None = None


def _coerce_int(value: object) -> int | None:
    try:
        return int(value)  # type: ignore[arg-type]
    except Exception:
        return None


def _coerce_bool(value: object) -> bool:
    if isinstance(value, str):
        text = value.strip().lower()
        if text in {"1", "true", "yes", "y"}:
            return True
        if text in {"0", "false", "no", "n", ""}:
            return False
    try:
        return bool(int(value))  # type: ignore[arg-type]
    except Exception:
        return bool(value)


def _key_ci(container: dict[str, Any], *names: str) -> str | None:
    if not isinstance(container, dict):
        return None
    by_lower = {str(key).lower(): str(key) for key in container.keys()}
    for name in names:
        actual = by_lower.get(str(name).lower())
        if actual is not None:
            return actual
    return None


def _get_ci(container: dict[str, Any], *names: str) -> Any:
    key = _key_ci(container, *names)
    return container.get(key) if key is not None else None


def _set_existing_or_default(container: dict[str, Any], default_key: str, value: Any, *names: str) -> str:
    matched = False
    actual_key = default_key
    for name in (default_key, *names):
        key = _key_ci(container, name)
        if key is None:
            continue
        container[key] = value
        actual_key = key
        matched = True
    if not matched:
        container[default_key] = value
    return actual_key


def _numeric_json_ids(directory: Path) -> list[int]:
    try:
        entries = list(directory.glob("*.json"))
    except Exception:
        return []
    out: list[int] = []
    for path in entries:
        try:
            out.append(int(path.stem))
        except Exception:
            continue
    return sorted(set(out))


def _next_free_id(existing: set[int], seed: int) -> int:
    value = int(seed)
    while value in existing or value <= 0:
        value += 1
    return int(value)


def _mission_id(mission: dict[str, Any]) -> int | None:
    return _coerce_int(_get_ci(mission, "inputMissionID", "InputMissionID"))


def _mission_done(mission: dict[str, Any]) -> bool:
    return _coerce_bool(_get_ci(mission, "isDone", "IsDone"))


def _same_input_id(actual: int, requested: int) -> bool:
    if int(actual) == int(requested):
        return True
    if 0 < int(requested) < 1_000_000:
        for modulus in (1_000_000, 100_000, 10_000):
            if int(actual) % modulus == int(requested):
                return True
    return False


def _select_current_index(missions: list[dict[str, Any]], current_input_id: int | None) -> int | None:
    if current_input_id is not None:
        for idx, mission in enumerate(missions):
            mission_id = _mission_id(mission)
            if mission_id is not None and _same_input_id(mission_id, int(current_input_id)):
                return idx

    for idx, mission in enumerate(missions):
        if _mission_id(mission) is not None and not _mission_done(mission):
            return idx

    for idx in range(len(missions) - 1, -1, -1):
        if _mission_id(missions[idx]) is not None:
            return idx
    return None


def _choose_source_package(input_dir: Path, source_package_id: int | None) -> tuple[int | None, Path | None, list[int]]:
    package_ids = _numeric_json_ids(input_dir)
    if source_package_id is not None:
        source_path = input_dir / f"{int(source_package_id)}.json"
        if source_path.exists():
            return int(source_package_id), source_path, package_ids
    if not package_ids:
        return None, None, package_ids
    source_id = int(package_ids[-1])
    return source_id, input_dir / f"{source_id}.json", package_ids


def clone_current_input_for_reexecute(
    *,
    current_input_id: int | None = None,
    source_package_id: int | None = None,
    db_root: str | Path | None = None,
    now_ms: Callable[[], int] | None = None,
) -> ReexecuteInputCloneResult:
    """Clone the active 0201 package and insert a fresh pending copy of the current input mission."""

    root = Path(db_root) if db_root is not None else db_paths.get_active_db_root()
    input_dir = root / "InputMissionPlan"
    if not input_dir.exists():
        return ReexecuteInputCloneResult(
            ok=False,
            message=f"InputMissionPlan directory not found: {input_dir}",
            db_root=root,
        )

    source_id, source_path, package_ids = _choose_source_package(input_dir, source_package_id)
    if source_id is None or source_path is None:
        return ReexecuteInputCloneResult(
            ok=False,
            message="No numeric InputMissionPlan package exists.",
            db_root=root,
        )

    try:
        source_payload = json.loads(source_path.read_text(encoding="utf-8"))
    except Exception as exc:
        return ReexecuteInputCloneResult(
            ok=False,
            message=f"InputMissionPlan load failed ({source_path}): {exc}",
            db_root=root,
            source_package_id=source_id,
        )
    if not isinstance(source_payload, dict):
        return ReexecuteInputCloneResult(
            ok=False,
            message=f"InputMissionPlan is not an object: {source_path}",
            db_root=root,
            source_package_id=source_id,
        )

    mission_list_key = _key_ci(source_payload, "inputMissionList", "InputMissionList")
    missions_raw = source_payload.get(mission_list_key) if mission_list_key else None
    if not isinstance(missions_raw, list):
        return ReexecuteInputCloneResult(
            ok=False,
            message=f"inputMissionList missing in package {source_id}",
            db_root=root,
            source_package_id=source_id,
        )
    missions = [mission for mission in missions_raw if isinstance(mission, dict)]
    if len(missions) != len(missions_raw):
        return ReexecuteInputCloneResult(
            ok=False,
            message=f"inputMissionList contains non-object entries in package {source_id}",
            db_root=root,
            source_package_id=source_id,
        )

    current_idx = _select_current_index(missions, current_input_id)
    if current_idx is None:
        return ReexecuteInputCloneResult(
            ok=False,
            message=f"No cloneable input mission in package {source_id}",
            db_root=root,
            source_package_id=source_id,
        )

    source_mission = missions[current_idx]
    source_input_id = _mission_id(source_mission)
    if source_input_id is None:
        return ReexecuteInputCloneResult(
            ok=False,
            message=f"Selected input mission has no inputMissionID in package {source_id}",
            db_root=root,
            source_package_id=source_id,
        )

    existing_input_ids = {
        int(value)
        for value in (_mission_id(mission) for mission in missions)
        if value is not None
    }
    new_input_id = _next_free_id(existing_input_ids, max(existing_input_ids or {0}) + 1)
    new_package_id = _next_free_id(set(package_ids), int(source_id) + 1)

    payload = copy.deepcopy(source_payload)
    cloned_missions = payload.get(mission_list_key) if mission_list_key else None
    if not isinstance(cloned_missions, list) or not isinstance(cloned_missions[current_idx], dict):
        return ReexecuteInputCloneResult(
            ok=False,
            message="Internal clone structure error.",
            db_root=root,
            source_package_id=source_id,
        )

    original_in_clone = cloned_missions[current_idx]
    done_key = _set_existing_or_default(original_in_clone, "isDone", True, "IsDone")

    cloned_current = copy.deepcopy(original_in_clone)
    id_key = _key_ci(cloned_current, "inputMissionID", "InputMissionID") or "inputMissionID"
    cloned_current[id_key] = int(new_input_id)
    cloned_current[done_key] = False
    if done_key != "isDone" and "isDone" in cloned_current:
        cloned_current["isDone"] = False
    elif done_key != "IsDone" and "IsDone" in cloned_current:
        cloned_current["IsDone"] = False

    cloned_missions.insert(current_idx + 1, cloned_current)
    payload[mission_list_key or "inputMissionList"] = cloned_missions

    _set_existing_or_default(
        payload,
        "inputMissionPackageID",
        int(new_package_id),
        "InputMissionPackageID",
    )
    timestamp = int(now_ms()) if callable(now_ms) else None
    if timestamp is not None:
        _set_existing_or_default(payload, "timestamp", timestamp, "Timestamp", "timeStamp", "TimeStamp")

    output_path = input_dir / f"{int(new_package_id)}.json"
    try:
        output_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
    except Exception as exc:
        return ReexecuteInputCloneResult(
            ok=False,
            message=f"InputMissionPlan write failed ({output_path}): {exc}",
            db_root=root,
            source_package_id=source_id,
            new_package_id=new_package_id,
            source_input_id=source_input_id,
            new_input_id=new_input_id,
        )

    return ReexecuteInputCloneResult(
        ok=True,
        message=(
            f"InputMissionPlan {source_id}->{new_package_id}, "
            f"inputMission {source_input_id}->{new_input_id}"
        ),
        db_root=root,
        source_package_id=int(source_id),
        new_package_id=int(new_package_id),
        source_input_id=int(source_input_id),
        new_input_id=int(new_input_id),
        output_path=output_path,
        payload=payload,
    )
