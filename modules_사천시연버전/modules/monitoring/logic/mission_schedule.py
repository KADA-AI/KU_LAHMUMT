from __future__ import annotations

from typing import Any


IMAGING_OPERATION_MODES = {1, 2, 3, 4, 5}
IMAGING_PATTERN_TYPES = {3, 4, 5, 6, 7, 8, 9}
ON_TIME_THRESHOLD_S = 5


def _coerce_int(value: object | None) -> int | None:
    try:
        if value is None:
            return None
        return int(value)
    except Exception:
        return None


def _coerce_bool(value: object | None) -> bool:
    return bool(value)


def classify_imaging_waypoint(mission: dict[str, Any], waypoint: dict[str, Any]) -> bool:
    sensor_type = _coerce_int(waypoint.get("sensor_type") or waypoint.get("sensorType"))
    if sensor_type is not None and sensor_type > 0:
        return True
    operation_mode = _coerce_int(waypoint.get("operation_mode") or waypoint.get("operationMode"))
    if operation_mode in IMAGING_OPERATION_MODES:
        return True
    if _coerce_bool(waypoint.get("has_filming_property") or waypoint.get("hasFilmingProperty")):
        return True
    pattern_type = _coerce_int(mission.get("pattern_type") or mission.get("patternType"))
    return pattern_type in IMAGING_PATTERN_TYPES


def schedule_state_label(status: str, delta_seconds: int | None) -> str:
    normalized = str(status or "pending").strip().lower()
    if normalized == "pending":
        return "Pending"
    if normalized == "skipped":
        return "Skipped"
    if delta_seconds is None:
        return "Reached"
    delta = int(delta_seconds)
    if abs(delta) <= ON_TIME_THRESHOLD_S:
        return "On time"
    if delta < 0:
        return "Early"
    return "Late"


def format_duration(value: object | None) -> str:
    try:
        if value is None:
            return "-"
        total = int(round(float(value)))
    except Exception:
        return "-"
    sign = "-" if total < 0 else ""
    total = abs(total)
    hours, rem = divmod(total, 3600)
    minutes, seconds = divmod(rem, 60)
    if hours > 0:
        return f"{sign}{hours:02d}:{minutes:02d}:{seconds:02d}"
    return f"{sign}{minutes:02d}:{seconds:02d}"


def build_aircraft_schedule_view(
    mission_view: dict[str, Any] | None,
    progress_snapshot: dict[str, Any] | None,
    aircraft_id: int,
) -> dict[str, Any]:
    entry = None
    for item in (mission_view or {}).get("uav_entries") or []:
        if not isinstance(item, dict):
            continue
        if _coerce_int(item.get("aircraft_id")) == int(aircraft_id):
            entry = item
            break
    if not isinstance(entry, dict):
        return {
            "aircraft_id": int(aircraft_id),
            "path_rows": [],
            "imaging_rows": [],
            "current_mission_id": None,
            "path_summary": {},
            "imaging_summary": {},
            "timestamp_ms": None,
        }

    snapshot = progress_snapshot or {}
    mission_progress = snapshot.get("mission_progress") or {}
    current_mission_map = snapshot.get("aircraft_current_mission") or {}
    timestamp_ms = _coerce_int((progress_snapshot or {}).get("timestamp_ms"))
    current_mission_id = _coerce_int(current_mission_map.get(int(aircraft_id)))
    if current_mission_id is None:
        current_mission_id = _coerce_int(entry.get("current_individual_mission_id"))

    path_rows: list[dict[str, Any]] = []
    imaging_rows: list[dict[str, Any]] = []
    current_path_rows: list[dict[str, Any]] = []
    current_imaging_rows: list[dict[str, Any]] = []

    for mission in entry.get("missions") or []:
        if not isinstance(mission, dict):
            continue
        mission_id = _coerce_int(mission.get("individual_mission_id"))
        path_id = _coerce_int(mission.get("path_id"))
        input_id = _coerce_int(mission.get("input_id"))
        progress = mission_progress.get(int(mission_id)) if mission_id is not None else {}
        if not isinstance(progress, dict):
            progress = {}
        status_items = progress.get("waypoint_status") or []
        status_map: dict[int, dict[str, Any]] = {}
        for item in status_items:
            if not isinstance(item, dict):
                continue
            waypoint_id = _coerce_int(item.get("waypoint_id"))
            if waypoint_id is None:
                continue
            status_map[int(waypoint_id)] = item

        for order, waypoint in enumerate(mission.get("waypoints") or [], start=1):
            if not isinstance(waypoint, dict):
                continue
            waypoint_id = _coerce_int(waypoint.get("waypoint_id"))
            if waypoint_id is None:
                continue
            status_item = status_map.get(int(waypoint_id), {})
            status = str(status_item.get("status") or "pending")
            planned_seconds = _coerce_int(status_item.get("planned_seconds"))
            if planned_seconds is None:
                planned_seconds = _coerce_int(waypoint.get("eta_cumulative"))
            actual_seconds = _coerce_int(status_item.get("actual_seconds_real"))
            delta_seconds = _coerce_int(status_item.get("delta_seconds"))
            is_imaging = classify_imaging_waypoint(mission, waypoint)
            row = {
                "mission_id": mission_id,
                "input_id": input_id,
                "path_id": path_id,
                "waypoint_id": int(waypoint_id),
                "order": int(order),
                "status": status,
                "planned_seconds": planned_seconds,
                "actual_seconds": actual_seconds,
                "delta_seconds": delta_seconds,
                "completion_timestamp_ms": _coerce_int(status_item.get("completion_timestamp_ms")),
                "schedule_state": schedule_state_label(status, delta_seconds),
                "is_imaging": bool(is_imaging),
                "sensor_type": _coerce_int(waypoint.get("sensor_type")),
                "operation_mode": _coerce_int(waypoint.get("operation_mode")),
            }
            path_rows.append(row)
            if mission_id is not None and mission_id == current_mission_id:
                current_path_rows.append(row)
            if is_imaging:
                imaging_rows.append(row)
                if mission_id is not None and mission_id == current_mission_id:
                    current_imaging_rows.append(row)

    path_summary = _build_path_summary(current_mission_id, entry, mission_progress, current_path_rows)
    imaging_summary = _build_imaging_summary(current_mission_id, current_imaging_rows, mission_progress)
    return {
        "aircraft_id": int(aircraft_id),
        "path_rows": path_rows,
        "imaging_rows": imaging_rows,
        "current_mission_id": current_mission_id,
        "path_summary": path_summary,
        "imaging_summary": imaging_summary,
        "timestamp_ms": timestamp_ms,
    }


def _build_path_summary(
    current_mission_id: int | None,
    entry: dict[str, Any],
    mission_progress: dict[str | int, Any],
    current_rows: list[dict[str, Any]],
) -> dict[str, Any]:
    current_progress = mission_progress.get(int(current_mission_id)) if current_mission_id is not None else {}
    if not isinstance(current_progress, dict):
        current_progress = {}
    current_mission = None
    for mission in entry.get("missions") or []:
        if not isinstance(mission, dict):
            continue
        if _coerce_int(mission.get("individual_mission_id")) == current_mission_id:
            current_mission = mission
            break

    current_waypoint_id = _coerce_int(current_progress.get("current_waypoint_id"))
    current_row = None
    for row in current_rows:
        if _coerce_int(row.get("waypoint_id")) == current_waypoint_id:
            current_row = row
            break

    return {
        "mission_id": current_mission_id,
        "current_waypoint_id": current_waypoint_id,
        "planned_total_seconds": _coerce_int(current_progress.get("planned_seconds"))
        if current_progress
        else _coerce_int((current_mission or {}).get("eta_seconds")),
        "actual_total_seconds": _coerce_int(current_progress.get("actual_seconds_real")),
        "current_waypoint_planned_seconds": _coerce_int((current_row or {}).get("planned_seconds")),
        "current_waypoint_actual_seconds": _coerce_int((current_row or {}).get("actual_seconds")),
        "current_waypoint_delta_seconds": _coerce_int((current_row or {}).get("delta_seconds")),
        "current_waypoint_state": (current_row or {}).get("schedule_state"),
    }


def _build_imaging_summary(
    current_mission_id: int | None,
    current_rows: list[dict[str, Any]],
    mission_progress: dict[str | int, Any],
) -> dict[str, Any]:
    progress = mission_progress.get(int(current_mission_id)) if current_mission_id is not None else {}
    if not isinstance(progress, dict):
        progress = {}
    if not current_rows:
        return {
            "mission_id": current_mission_id,
            "imaging_waypoint_count": 0,
            "reached_imaging_waypoint_count": 0,
            "planned_latest_seconds": None,
            "actual_latest_seconds": None,
            "delta_latest_seconds": None,
            "latest_state": "-",
        }

    reached_rows = [row for row in current_rows if row.get("actual_seconds") is not None]
    latest_row = reached_rows[-1] if reached_rows else current_rows[0]
    return {
        "mission_id": current_mission_id,
        "imaging_waypoint_count": len(current_rows),
        "reached_imaging_waypoint_count": len(reached_rows),
        "planned_latest_seconds": _coerce_int(latest_row.get("planned_seconds")),
        "actual_latest_seconds": _coerce_int(latest_row.get("actual_seconds"))
        if reached_rows
        else _coerce_int(progress.get("actual_seconds_real")),
        "delta_latest_seconds": _coerce_int(latest_row.get("delta_seconds"))
        if reached_rows
        else None,
        "latest_state": latest_row.get("schedule_state"),
    }
