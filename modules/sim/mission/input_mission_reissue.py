from __future__ import annotations

import copy
import json
import math
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

from modules.common import db_paths
from .mission_plan_loader import normalize_input_mission_plan_float_fields


_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_LOCK = threading.Lock()
_TYPE1_REVIEWED_PATTERN = (
    (1, 3),
    (1, 4),
    (2, 4),
    (1, 5),
    (2, 5),
    (1, 6),
    (2, 6),
    (1, 5),
    (1, 3),
    (1, 2),
)


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


def _input_mission_pattern(payload: dict[str, Any]) -> tuple[tuple[int | None, int | None], ...]:
    missions = _get_ci(payload, "inputMissionList", "InputMissionList")
    if not isinstance(missions, list):
        return ()
    pattern: list[tuple[int | None, int | None]] = []
    for mission in missions:
        if not isinstance(mission, dict):
            return ()
        pattern.append(
            (
                _coerce_int(_get_ci(mission, "inputMissionType", "InputMissionType")),
                _coerce_int(_get_ci(mission, "regionType", "RegionType")),
            )
        )
    return tuple(pattern)


def _load_type1_reviewed_source(
    input_dir: Path,
    source_package_id: int | None,
) -> tuple[int | None, Path | None, dict[str, Any] | None, str | None]:
    package_ids = _numeric_json_ids(input_dir)
    candidates = (
        [int(source_package_id)]
        if source_package_id is not None and int(source_package_id) > 0
        else list(reversed(package_ids))
    )
    for package_id in candidates:
        path = input_dir / f"{int(package_id)}.json"
        if not path.is_file():
            continue
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except Exception as exc:
            if source_package_id is not None:
                return None, None, None, f"InputMissionPlan load failed ({path}): {exc}"
            continue
        if not isinstance(payload, dict):
            continue
        package_type = _coerce_int(
            _get_ci(payload, "inputMissionPackageType", "InputMissionPackageType")
        )
        if package_type == 1 and _input_mission_pattern(payload) == _TYPE1_REVIEWED_PATTERN:
            return int(package_id), path, payload, None
    if source_package_id is not None:
        return (
            None,
            None,
            None,
            f"InputMissionPlan {int(source_package_id)} is not the reviewed Type-1 10-mission pattern.",
        )
    return None, None, None, "No reviewed Type-1 10-mission InputMissionPlan exists."


def _load_type1_target_order_source(
    input_dir: Path,
    source_package_id: int | None,
) -> tuple[int | None, Path | None, dict[str, Any] | None, dict[str, Any] | None, str | None]:
    """Load the latest reviewed Type-1 plan that contains complete target bundles."""

    from modules.monitoring.logic.anti_armor_air_strike_review import (
        AntiArmorReviewError,
        describe_anti_armor_target_order,
    )

    package_ids = _numeric_json_ids(input_dir)
    candidates = (
        [int(source_package_id)]
        if source_package_id is not None and int(source_package_id) > 0
        else list(reversed(package_ids))
    )
    for package_id in candidates:
        path = input_dir / f"{int(package_id)}.json"
        if not path.is_file():
            continue
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except Exception as exc:
            if source_package_id is not None:
                return None, None, None, None, f"InputMissionPlan load failed ({path}): {exc}"
            continue
        if not isinstance(payload, dict):
            continue
        try:
            description = describe_anti_armor_target_order(payload)
        except (AntiArmorReviewError, TypeError, ValueError):
            continue
        if int(description.get("targetCount") or 0) < 2:
            continue
        return int(package_id), path, payload, description, None

    if source_package_id is not None:
        return (
            None,
            None,
            None,
            None,
            f"InputMissionPlan {int(source_package_id)} has no reorderable Type-1 target bundles.",
        )
    return None, None, None, None, "No reviewed Type-1 plan with at least two target areas exists."


def _earliest_type1_target_ids(input_dir: Path) -> set[int]:
    """Identify targets in the original reviewed plan for a small UI origin hint."""

    from modules.monitoring.logic.anti_armor_air_strike_review import (
        AntiArmorReviewError,
        describe_anti_armor_target_order,
    )

    for package_id in _numeric_json_ids(input_dir):
        path = input_dir / f"{int(package_id)}.json"
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            description = describe_anti_armor_target_order(payload)
        except (OSError, json.JSONDecodeError, AntiArmorReviewError, TypeError, ValueError):
            continue
        target_ids = description.get("targetInputMissionIDs")
        if isinstance(target_ids, list) and target_ids:
            return {int(value) for value in target_ids}
    return set()


def load_type1_target_order_candidates(
    *,
    source_package_id: int | None = None,
    db_root: str | Path | None = None,
) -> dict[str, Any]:
    """Return the target-only map data used by the SIM ordering interaction."""

    root = Path(db_root) if db_root is not None else db_paths.get_active_db_root()
    input_dir = root / "InputMissionPlan"
    if not input_dir.exists():
        return {
            "ok": False,
            "error": f"InputMissionPlan directory not found: {input_dir}",
            "dbRoot": str(root),
        }

    with _LOCK:
        source_id, _source_path, _source_payload, description, source_error = (
            _load_type1_target_order_source(input_dir, source_package_id)
        )
        if source_error or source_id is None or description is None:
            return {
                "ok": False,
                "error": source_error or "Type-1 target order source is unavailable.",
                "dbRoot": str(root),
            }
        original_target_ids = _earliest_type1_target_ids(input_dir)

    targets: list[dict[str, Any]] = []
    for row in description.get("targets") or []:
        if not isinstance(row, dict):
            continue
        target_id = _coerce_int(row.get("targetInputMissionID"))
        if target_id is None:
            continue
        target = copy.deepcopy(row)
        target["isNew"] = bool(original_target_ids and int(target_id) not in original_target_ids)
        targets.append(target)
    return {
        "ok": True,
        "sourcePackageID": int(source_id),
        "targetCount": len(targets),
        "targetInputMissionIDs": [int(row["targetInputMissionID"]) for row in targets],
        "targets": targets,
    }


def _normalize_new_target_coordinates(value: Any) -> tuple[list[dict[str, float]], str | None]:
    if not isinstance(value, list) or len(value) < 3:
        return [], "New target area requires at least three coordinates."
    coords: list[dict[str, float]] = []
    for index, row in enumerate(value, start=1):
        if not isinstance(row, dict):
            return [], f"New target coordinate #{index} is not an object."
        try:
            latitude = float(_get_ci(row, "latitude", "Latitude", "lat"))
            longitude = float(_get_ci(row, "longitude", "Longitude", "lon", "lng"))
            altitude_raw = _get_ci(row, "altitude", "Altitude", "alt")
            altitude_value = float(altitude_raw) if altitude_raw is not None else 0.0
        except Exception:
            return [], f"New target coordinate #{index} contains a non-numeric value."
        if not all(math.isfinite(value) for value in (latitude, longitude, altitude_value)):
            return [], f"New target coordinate #{index} contains a non-finite value."
        if not (-90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0):
            return [], f"New target coordinate #{index} is outside the latitude/longitude range."
        altitude = int(round(altitude_value))
        coords.append(
            {
                "latitude": float(latitude),
                "longitude": float(longitude),
                "altitude": int(altitude),
            }
        )
    unique_vertices = {
        (round(float(coord["latitude"]), 10), round(float(coord["longitude"]), 10))
        for coord in coords
    }
    if len(unique_vertices) < 3:
        return [], "New target area requires at least three distinct coordinates."
    signed_area = 0.0
    for index, coord in enumerate(coords):
        next_coord = coords[(index + 1) % len(coords)]
        signed_area += (
            float(coord["longitude"]) * float(next_coord["latitude"])
            - float(next_coord["longitude"]) * float(coord["latitude"])
        )
    if abs(signed_area) <= 1e-12:
        return [], "New target area vertices do not form a polygon."
    return coords, None


def prepare_type1_new_target_input_mission_0201(
    *,
    coordinate_list: Any,
    source_package_id: int | None = None,
    db_root: str | Path | None = None,
    now_ms: Callable[[], int] | None = None,
) -> dict[str, Any]:
    coords, coord_error = _normalize_new_target_coordinates(coordinate_list)
    if coord_error:
        return {"ok": False, "error": coord_error}

    root = Path(db_root) if db_root is not None else db_paths.get_active_db_root()
    input_dir = root / "InputMissionPlan"
    if not input_dir.exists():
        return {
            "ok": False,
            "error": f"InputMissionPlan directory not found: {input_dir}",
            "dbRoot": str(root),
        }

    with _LOCK:
        source_id, source_path, source_payload, source_error = _load_type1_reviewed_source(
            input_dir,
            source_package_id,
        )
        if source_error or source_id is None or source_path is None or source_payload is None:
            return {
                "ok": False,
                "error": source_error or "Reviewed Type-1 source package is unavailable.",
                "dbRoot": str(root),
            }

        source_missions = _get_ci(source_payload, "inputMissionList", "InputMissionList")
        assert isinstance(source_missions, list)
        mission_ids = [
            _coerce_int(_get_ci(mission, "inputMissionID", "InputMissionID"))
            for mission in source_missions
            if isinstance(mission, dict)
        ]
        if (
            len(mission_ids) != len(source_missions)
            or any(mission_id is None or mission_id <= 0 for mission_id in mission_ids)
            or len(set(mission_ids)) != len(mission_ids)
        ):
            return {
                "ok": False,
                "error": "Source Type-1 package requires unique positive inputMissionID values.",
                "sourcePackageID": int(source_id),
            }

        payload = copy.deepcopy(source_payload)
        output_missions = copy.deepcopy(source_missions)
        for mission in output_missions:
            if isinstance(mission, dict):
                _set_existing_or_default(mission, "isDone", False, "IsDone")

        new_target_id = max(int(value) for value in mission_ids if value is not None) + 1
        new_target_mission = copy.deepcopy(output_missions[6])
        _set_existing_or_default(
            new_target_mission,
            "inputMissionID",
            int(new_target_id),
            "InputMissionID",
        )
        _set_existing_or_default(
            new_target_mission,
            "inputMissionType",
            2,
            "InputMissionType",
        )
        _set_existing_or_default(new_target_mission, "regionType", 6, "RegionType")
        _set_existing_or_default(new_target_mission, "isDone", False, "IsDone")
        new_target_mission["missionDetail"] = {
            "coordinateList": None,
            "lineList": None,
            "areaList": [
                {
                    "isHole": False,
                    "coordinateList": copy.deepcopy(coords),
                }
            ],
        }
        output_missions.insert(8, new_target_mission)
        _set_existing_or_default(
            payload,
            "inputMissionList",
            output_missions,
            "InputMissionList",
        )

        for provenance_name in (
            "reviewSource",
            "reviewedFromInputMissionPackageID",
            "reviewKind",
        ):
            provenance_key = _key_ci(payload, provenance_name)
            if provenance_key is not None:
                payload.pop(provenance_key, None)

        package_ids = _numeric_json_ids(input_dir)
        new_package_id = (max(package_ids) + 1) if package_ids else (int(source_id) + 1)
        output_path = input_dir / f"{int(new_package_id)}.json"
        timestamp = int(now_ms() if callable(now_ms) else now_ms_2000())
        _set_existing_or_default(
            payload,
            "inputMissionPackageID",
            int(new_package_id),
            "InputMissionPackageID",
        )
        _set_existing_or_default(payload, "timestamp", timestamp, "Timestamp", "timeStamp", "TimeStamp")
        normalize_input_mission_plan_float_fields(payload)

        try:
            output_path.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
        except Exception as exc:
            return {
                "ok": False,
                "error": f"InputMissionPlan write failed ({output_path}): {exc}",
                "sourcePackageID": int(source_id),
                "newPackageID": int(new_package_id),
            }

    return {
        "ok": True,
        "message": f"Type-1 new target InputMissionPlan {source_id}->{new_package_id} prepared for 0201",
        "dbRoot": str(root),
        "inputDir": str(input_dir),
        "sourcePackageID": int(source_id),
        "newPackageID": int(new_package_id),
        "newTargetInputMissionID": int(new_target_id),
        "timestamp": int(timestamp),
        "outputPath": str(output_path),
        "inputMissionCount": len(output_missions),
        "pattern": [list(row) for row in _input_mission_pattern(payload)],
    }


def prepare_type1_target_order_input_mission_0201(
    *,
    ordered_target_input_mission_ids: Any,
    source_package_id: int | None = None,
    db_root: str | Path | None = None,
    now_ms: Callable[[], int] | None = None,
) -> dict[str, Any]:
    """Create an incoming 0201 package with complete target sorties reordered."""

    if not isinstance(ordered_target_input_mission_ids, list):
        return {"ok": False, "error": "targetInputMissionIDOrder must be an array."}
    try:
        requested_ids = [int(value) for value in ordered_target_input_mission_ids]
    except Exception:
        return {"ok": False, "error": "targetInputMissionIDOrder contains a non-integer value."}

    root = Path(db_root) if db_root is not None else db_paths.get_active_db_root()
    input_dir = root / "InputMissionPlan"
    if not input_dir.exists():
        return {
            "ok": False,
            "error": f"InputMissionPlan directory not found: {input_dir}",
            "dbRoot": str(root),
        }

    from modules.monitoring.logic.anti_armor_air_strike_review import (
        AntiArmorReviewError,
        build_anti_armor_target_order_payload,
    )

    with _LOCK:
        source_id, _source_path, source_payload, description, source_error = (
            _load_type1_target_order_source(input_dir, source_package_id)
        )
        if (
            source_error
            or source_id is None
            or source_payload is None
            or description is None
        ):
            return {
                "ok": False,
                "error": source_error or "Type-1 target order source is unavailable.",
                "dbRoot": str(root),
            }

        package_ids = _numeric_json_ids(input_dir)
        new_package_id = (max(package_ids) + 1) if package_ids else (int(source_id) + 1)
        output_path = input_dir / f"{int(new_package_id)}.json"
        if output_path.exists():
            return {
                "ok": False,
                "error": f"Target InputMissionPlan already exists: {output_path}",
                "sourcePackageID": int(source_id),
                "newPackageID": int(new_package_id),
            }

        timestamp = int(now_ms() if callable(now_ms) else now_ms_2000())
        try:
            build_result = build_anti_armor_target_order_payload(
                source_payload,
                ordered_target_input_mission_ids=requested_ids,
                new_package_id=int(new_package_id),
                timestamp_ms=int(timestamp),
            )
        except (AntiArmorReviewError, TypeError, ValueError) as exc:
            return {
                "ok": False,
                "error": str(exc),
                "sourcePackageID": int(source_id),
                "targetInputMissionIDs": description.get("targetInputMissionIDs") or [],
            }
        normalize_input_mission_plan_float_fields(build_result.payload)

        try:
            output_path.write_text(
                json.dumps(build_result.payload, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
        except Exception as exc:
            return {
                "ok": False,
                "error": f"InputMissionPlan write failed ({output_path}): {exc}",
                "sourcePackageID": int(source_id),
                "newPackageID": int(new_package_id),
            }

    return {
        "ok": True,
        "message": f"Type-1 target order InputMissionPlan {source_id}->{new_package_id} prepared for 0201",
        "dbRoot": str(root),
        "inputDir": str(input_dir),
        "sourcePackageID": int(source_id),
        "newPackageID": int(new_package_id),
        "timestamp": int(timestamp),
        "outputPath": str(output_path),
        "targetCount": int(build_result.summary.get("targetCount") or 0),
        "previousTargetInputMissionIDs": build_result.summary.get("previousTargetInputMissionIDs") or [],
        "currentTargetInputMissionIDs": build_result.summary.get("currentTargetInputMissionIDs") or [],
        "inputMissionCount": int(build_result.summary.get("missionCount") or 0),
        "summary": copy.deepcopy(build_result.summary),
    }


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
        normalize_input_mission_plan_float_fields(payload)

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
