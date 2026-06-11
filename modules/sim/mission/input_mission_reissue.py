from __future__ import annotations

import copy
import json
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

from modules.common import db_paths


_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_LOCK = threading.Lock()


def now_ms_2000() -> int:
    return int((datetime.now(timezone.utc) - _EPOCH_2000).total_seconds() * 1000)


def _coerce_int(value: object) -> int | None:
    try:
        return int(value)  # type: ignore[arg-type]
    except Exception:
        return None


def _coerce_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        text = value.strip().lower()
        if text in {"1", "true", "yes", "y", "on"}:
            return True
        if text in {"0", "false", "no", "n", "off", ""}:
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
    ids: list[int] = []
    for path in entries:
        try:
            ids.append(int(path.stem))
        except Exception:
            continue
    return sorted(set(ids))


def _choose_source_package(input_dir: Path, source_package_id: int | None) -> tuple[int | None, Path | None, list[int]]:
    package_ids = _numeric_json_ids(input_dir)
    if source_package_id is not None and int(source_package_id) > 0:
        source_path = input_dir / f"{int(source_package_id)}.json"
        if source_path.exists():
            return int(source_package_id), source_path, package_ids
    if not package_ids:
        return None, None, package_ids
    source_id = int(package_ids[-1])
    return source_id, input_dir / f"{source_id}.json", package_ids


def _reset_input_mission_done_flags(payload: dict[str, Any]) -> tuple[int, int, str | None]:
    mission_list_key = _key_ci(payload, "inputMissionList", "InputMissionList")
    missions_raw = payload.get(mission_list_key) if mission_list_key else None
    if not isinstance(missions_raw, list):
        return 0, 0, "inputMissionList missing"

    changed = 0
    mission_count = 0
    for mission in missions_raw:
        if not isinstance(mission, dict):
            return changed, mission_count, "inputMissionList contains non-object entry"
        done_key = _key_ci(mission, "isDone", "IsDone")
        if done_key is None:
            continue
        mission_count += 1
        if _coerce_bool(mission.get(done_key)):
            changed += 1
        mission[done_key] = False
        if done_key != "isDone" and "isDone" in mission:
            mission["isDone"] = False
        if done_key != "IsDone" and "IsDone" in mission:
            mission["IsDone"] = False
    return changed, mission_count, None


def prepare_reissued_input_mission_0201(
    *,
    source_package_id: int | None = None,
    db_root: str | Path | None = None,
    now_ms: Callable[[], int] | None = None,
) -> dict[str, Any]:
    root = Path(db_root) if db_root is not None else db_paths.get_active_db_root()
    input_dir = root / "InputMissionPlan"
    if not input_dir.exists():
        return {
            "ok": False,
            "error": f"InputMissionPlan directory not found: {input_dir}",
            "dbRoot": str(root),
        }

    with _LOCK:
        source_id, source_path, package_ids = _choose_source_package(input_dir, source_package_id)
        if source_id is None or source_path is None:
            return {
                "ok": False,
                "error": "No numeric InputMissionPlan package exists.",
                "dbRoot": str(root),
                "inputDir": str(input_dir),
            }

        new_package_id = int(source_id) + 1
        output_path = input_dir / f"{new_package_id}.json"
        if output_path.exists():
            return {
                "ok": False,
                "error": f"Target InputMissionPlan already exists: {output_path}",
                "dbRoot": str(root),
                "inputDir": str(input_dir),
                "sourcePackageID": int(source_id),
                "newPackageID": int(new_package_id),
            }

        try:
            source_payload = json.loads(source_path.read_text(encoding="utf-8"))
        except Exception as exc:
            return {
                "ok": False,
                "error": f"InputMissionPlan load failed ({source_path}): {exc}",
                "dbRoot": str(root),
                "sourcePackageID": int(source_id),
            }
        if not isinstance(source_payload, dict):
            return {
                "ok": False,
                "error": f"InputMissionPlan is not an object: {source_path}",
                "dbRoot": str(root),
                "sourcePackageID": int(source_id),
            }

        payload = copy.deepcopy(source_payload)
        changed_count, mission_count, reset_error = _reset_input_mission_done_flags(payload)
        if reset_error:
            return {
                "ok": False,
                "error": reset_error,
                "dbRoot": str(root),
                "sourcePackageID": int(source_id),
            }

        timestamp = int(now_ms() if callable(now_ms) else now_ms_2000())
        _set_existing_or_default(payload, "inputMissionPackageID", int(new_package_id), "InputMissionPackageID")
        _set_existing_or_default(payload, "timestamp", timestamp, "Timestamp", "timeStamp", "TimeStamp")

        try:
            output_path.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
        except Exception as exc:
            return {
                "ok": False,
                "error": f"InputMissionPlan write failed ({output_path}): {exc}",
                "dbRoot": str(root),
                "sourcePackageID": int(source_id),
                "newPackageID": int(new_package_id),
            }

    return {
        "ok": True,
        "message": f"InputMissionPlan {source_id}->{new_package_id} prepared for 0201",
        "dbRoot": str(root),
        "inputDir": str(input_dir),
        "sourcePackageID": int(source_id),
        "newPackageID": int(new_package_id),
        "timestamp": int(timestamp),
        "outputPath": str(output_path),
        "resetIsDoneCount": int(changed_count),
        "inputMissionCount": int(mission_count),
        "knownPackageIDs": [int(value) for value in package_ids],
    }
