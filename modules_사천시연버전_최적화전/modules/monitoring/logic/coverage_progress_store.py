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
    mission_entries: list[dict[str, object]] = []
    for entry in view.get("uav_entries") or []:
        if not isinstance(entry, dict):
            continue
        aircraft_id = entry.get("aircraft_id")
        for mission in entry.get("missions") or []:
            if not isinstance(mission, dict):
                continue
            mission_entries.append(
                {
                    "aircraft_id": aircraft_id,
                    "input_id": mission.get("input_id"),
                    "mission_id": mission.get("individual_mission_id"),
                    "mission_type": mission.get("individual_mission_type"),
                    "coverage_enabled": bool(mission.get("coverage_enabled")),
                    "coverage_percent": int(mission.get("coverage_percent") or 0),
                    "covered_area_m2": float(mission.get("covered_area_m2") or 0.0),
                    "planned_area_m2": float(mission.get("planned_area_m2") or 0.0),
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
