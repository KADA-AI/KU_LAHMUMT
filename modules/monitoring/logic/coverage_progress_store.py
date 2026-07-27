# -*- coding: utf-8 -*-
from __future__ import annotations

import json
from typing import Any

from modules.common import db_paths


def build_coverage_progress_payload(
    *,
    mission_view: dict[str, Any] | None,
    snapshot: dict[str, Any] | None,
    timestamp_ms: int | None = None,
) -> dict[str, Any] | None:
    """Build the coverage_progress.json payload without depending on UI widgets."""
    view = mission_view if isinstance(mission_view, dict) else None
    if not view:
        return None
    snapshot = snapshot if isinstance(snapshot, dict) else {}
    mission_progress = snapshot.get("mission_progress") or {}
    mission_entries: list[dict[str, object]] = []
    for entry in view.get("uav_entries") or []:
        if not isinstance(entry, dict):
            continue
        aircraft_id = entry.get("aircraft_id")
        for mission in entry.get("missions") or []:
            if not isinstance(mission, dict):
                continue
            mission_id = mission.get("individual_mission_id")
            progress = (
                mission_progress.get(mission_id)
                or mission_progress.get(str(mission_id))
                or {}
            )
            if not isinstance(progress, dict):
                progress = {}
            mission_entries.append(
                {
                    "aircraft_id": aircraft_id,
                    "input_id": mission.get("input_id"),
                    "mission_id": mission.get("individual_mission_id"),
                    "mission_type": mission.get("individual_mission_type"),
                    "coverage_enabled": bool(progress.get("coverage_enabled", mission.get("coverage_enabled"))),
                    "coverage_percent": int(progress.get("coverage_percent", mission.get("coverage_percent")) or 0),
                    "covered_area_m2": float(progress.get("covered_area_m2", mission.get("covered_area_m2")) or 0.0),
                    "planned_area_m2": float(progress.get("planned_area_m2", mission.get("planned_area_m2")) or 0.0),
                    "coverage_source": progress.get("coverage_source", mission.get("coverage_source")),
                    "coverage_unit": progress.get("coverage_unit", mission.get("coverage_unit")),
                    "coverage_pass_count": int(progress.get("coverage_pass_count") or 0),
                    "coverage_pass_policy": progress.get("coverage_pass_policy"),
                    "coverage_pass_details": list(progress.get("coverage_pass_details") or []),
                    "coverage_pass_requirement_mode": progress.get("coverage_pass_requirement_mode"),
                    "coverage_depth_policy": progress.get("coverage_depth_policy"),
                    "required_coverage_depth": int(progress.get("required_coverage_depth") or 1),
                    "remaining_coverage_depth": int(progress.get("remaining_coverage_depth") or 0),
                    "completed_coverage_depth": int(progress.get("completed_coverage_depth") or 0),
                    "coverage_depth_satisfied": bool(progress.get("coverage_depth_satisfied")),
                    "coverage_depth_area_m2": dict(progress.get("coverage_depth_area_m2") or {}),
                    "coverage_depth_details": list(progress.get("coverage_depth_details") or []),
                    "coverage_observation_details": list(progress.get("coverage_observation_details") or []),
                    "coverage_work_covered_area_m2": float(progress.get("coverage_work_covered_area_m2") or 0.0),
                    "coverage_work_required_area_m2": float(progress.get("coverage_work_required_area_m2") or 0.0),
                    "coverage_work_remaining_area_m2": float(progress.get("coverage_work_remaining_area_m2") or 0.0),
                    "coverage_completion_tolerance_m2": float(progress.get("coverage_completion_tolerance_m2") or 0.0),
                    "coverage_requirement_met": bool(progress.get("coverage_requirement_met")),
                    "spatial_coverage_percent": int(progress.get("spatial_coverage_percent") or 0),
                    "spatial_covered_area_m2": float(progress.get("spatial_covered_area_m2") or 0.0),
                    "spatial_required_area_m2": float(progress.get("spatial_required_area_m2") or 0.0),
                    "footprint_coverage_enabled": bool(progress.get("footprint_coverage_enabled", mission.get("footprint_coverage_enabled"))),
                    "footprint_coverage_percent": int(progress.get("footprint_coverage_percent", mission.get("footprint_coverage_percent")) or 0),
                    "footprint_covered_area_m2": float(progress.get("footprint_covered_area_m2", mission.get("footprint_covered_area_m2")) or 0.0),
                    "footprint_planned_area_m2": float(progress.get("footprint_planned_area_m2", mission.get("footprint_planned_area_m2")) or 0.0),
                    "done": bool(mission.get("is_done")),
                }
            )
    return {
        "timestamp_ms": snapshot.get("timestamp_ms") or timestamp_ms,
        "mission_plan_id": view.get("mission_plan_id"),
        "plan_coverage": dict(snapshot.get("plan_coverage") or {}),
        "input_coverage": {
            str(key): dict(value)
            for key, value in (snapshot.get("input_coverage") or {}).items()
            if isinstance(value, dict)
        },
        "package_coverage": {
            str(key): dict(value)
            for key, value in (snapshot.get("package_coverage") or {}).items()
            if isinstance(value, dict)
        },
        "plan_footprint_coverage": dict(snapshot.get("plan_footprint_coverage") or {}),
        "input_footprint_coverage": {
            str(key): dict(value)
            for key, value in (snapshot.get("input_footprint_coverage") or {}).items()
            if isinstance(value, dict)
        },
        "package_footprint_coverage": {
            str(key): dict(value)
            for key, value in (snapshot.get("package_footprint_coverage") or {}).items()
            if isinstance(value, dict)
        },
        "missions": mission_entries,
    }


def persist_coverage_progress(
    *,
    mission_view: dict[str, Any] | None,
    snapshot: dict[str, Any] | None,
    timestamp_ms: int | None = None,
    previous_signature: str | None = None,
) -> str | None:
    """Persist coverage_progress.json and return the active signature."""
    payload = build_coverage_progress_payload(
        mission_view=mission_view,
        snapshot=snapshot,
        timestamp_ms=timestamp_ms,
    )
    if not payload:
        return previous_signature
    try:
        signature = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    except Exception:
        return previous_signature
    if signature == previous_signature:
        return previous_signature
    try:
        base = db_paths.get_db_subpath("DSS_Internal")
        base.mkdir(parents=True, exist_ok=True)
        (base / "coverage_progress.json").write_text(signature, encoding="utf-8")
    except Exception:
        return previous_signature
    return signature
