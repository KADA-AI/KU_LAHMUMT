from __future__ import annotations

from typing import Any

from modules.monitoring.logic.next_collab_replan import (
    _centroid_coordinate,
    _coerce_float,
    _math_rad_to_heading_deg,
    _normalize_coordinate,
    _planner_turn_radius_scale,
)


def resolve_remaining_entry_aircraft_list(
    *,
    aircraft_ids: list[int],
    turn_views: dict[int, Any] | None,
    entry_strategy: str,
    log_prefix: str = "[CURRENT]",
) -> tuple[list[dict[str, Any]], dict[str, float] | None, float, list[str]]:
    logs: list[str] = []
    entries: list[dict[str, Any]] = []
    coords_for_centroid: list[dict[str, float]] = []
    views = turn_views if isinstance(turn_views, dict) else {}
    strategy = str(entry_strategy or "").strip().lower()
    use_turn_projection = strategy == "turn_projection"
    use_current_position = strategy == "current_position"
    turn_radius_scale = _planner_turn_radius_scale()

    for aircraft_id in aircraft_ids:
        view = views.get(int(aircraft_id))
        coord = None
        source = None
        eta_s = None
        raw_heading_deg = None
        heading_rad = None
        if view is not None:
            position_coord = _normalize_coordinate(getattr(view, "position_coordinate", None))
            raw_heading_deg = _coerce_float(getattr(view, "raw_heading_deg", None))
            heading_rad = _coerce_float(getattr(view, "heading_rad", None))
            if use_current_position:
                coord = position_coord
                eta_s = 0.0 if coord is not None else None
                if coord is not None:
                    source = "currentPosition0401"
            elif use_turn_projection:
                coord = position_coord
                eta_s = 0.0 if coord is not None else None
                if coord is not None:
                    source = "currentPosition0401"
            else:
                coord = position_coord
                if coord is not None:
                    eta_s = 0.0
                    source = "currentPosition0401"

        if coord is None:
            logs.append(f"{log_prefix} entry skipped aircraft {aircraft_id}: no entry coordinate")
            continue

        coords_for_centroid.append(coord)
        entry: dict[str, Any] = {
            "aircraftID": int(aircraft_id),
            "coordinate": dict(coord),
            "source": str(source or "unknown"),
        }
        if raw_heading_deg is None and heading_rad is not None:
            raw_heading_deg = _math_rad_to_heading_deg(float(heading_rad))
        if raw_heading_deg is not None:
            entry["headingDeg"] = float(raw_heading_deg) % 360.0
        if eta_s is not None:
            entry["etaS"] = float(eta_s)
        entries.append(entry)

    return entries, _centroid_coordinate(coords_for_centroid), float(turn_radius_scale), logs
