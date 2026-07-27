# -*- coding: utf-8 -*-
from __future__ import annotations

from copy import deepcopy
from typing import Any

from shapely.geometry import Polygon

from modules.monitoring.logic.mission_area_progress_monitor import (
    MissionProgressAreaSnapshotMonitor,
    _MissionAreaState,
    _apply_sweep_point_progress_to_area_state,
    _area_ownership_details_for_states,
    _area_progress_details_for_states,
    _attach_remaining_detail_to_area_ownership,
    _build_planned_sweep_lines,
    _coord,
    _inverse_project_coord,
    _iter_polygons,
    _mission_geometry_kind,
    _remaining_geometry_diagnostics,
    _remaining_geometry_for_state,
)

_AREA_DIAGNOSTICS_GROUP_TITLE = "Area 재계획 진단"


def _area_remaining_segments_for_state(
    state: _MissionAreaState,
    remaining_geometry,
    transformer,
    *,
    altitude: float | int | None = 0.0,
) -> list[dict[str, Any]]:
    """Build planned-row area segments from the remaining geometry for replan input."""
    if state is None or remaining_geometry is None or getattr(remaining_geometry, "is_empty", True):
        return []
    segments: list[dict[str, Any]] = []
    half_width_m = max(float(getattr(state, "cut_half_width_m", 0.0) or 0.0), 1.0)
    area_threshold_m2 = max(8.0, half_width_m * half_width_m * 0.05)
    input_id = getattr(state, "input_id", None)
    for line_index, line in enumerate(getattr(state, "planned_cut_lines", []) or []):
        try:
            if line is None or line.is_empty:
                continue
            strip = line.buffer(half_width_m, cap_style=2, join_style=2)
            intersection = remaining_geometry.intersection(strip)
        except Exception:
            continue
        for poly in _iter_polygons(intersection):
            try:
                area_m2 = float(poly.area or 0.0)
            except Exception:
                area_m2 = 0.0
            if area_m2 < area_threshold_m2:
                continue
            coords = [
                coord
                for coord in (
                    _inverse_project_coord(transformer, x_val, y_val, altitude)
                    for x_val, y_val in list(poly.exterior.coords)
                )
                if coord is not None
            ]
            if len(coords) >= 2 and coords[0] == coords[-1]:
                coords = coords[:-1]
            if len(coords) < 3:
                continue
            segments.append(
                {
                    "source": "planned_sweep_row",
                    "lineIndex": int(line_index),
                    "aircraftID": int(getattr(state, "aircraft_id", 0) or 0),
                    "individualMissionID": int(getattr(state, "mission_id", 0) or 0),
                    "inputMissionID": int(input_id) if input_id is not None else None,
                    "areaM2": float(area_m2),
                    "coordinateList": coords,
                }
            )
    return segments


def _snapshot_mission_for_selected_mission(
    snapshot: dict[str, Any] | None,
    selected_mission: dict[str, Any] | None,
) -> dict[str, Any] | None:
    if not isinstance(snapshot, dict) or not isinstance(selected_mission, dict):
        return None
    selected_input_id = selected_mission.get("input_id") or selected_mission.get("inputMissionID")
    selected_individual_id = (
        selected_mission.get("individual_mission_id")
        or selected_mission.get("individualMissionID")
    )
    for mission in snapshot.get("missions") or []:
        if not isinstance(mission, dict):
            continue
        if selected_input_id is not None and mission.get("inputMissionID") == selected_input_id:
            return mission
        ids = mission.get("individualMissionIDs")
        if selected_individual_id is not None and isinstance(ids, list) and selected_individual_id in ids:
            return mission
    return None


def _populate_area_diagnostics_table(table: Any, diagnostics: dict[str, Any] | None) -> None:
    """Populate a two-column diagnostics table when a Qt table is supplied."""
    if table is None or not hasattr(table, "setRowCount"):
        return
    data = diagnostics if isinstance(diagnostics, dict) else {}
    operator_decisions = data.get("operatorDecisions") or data.get("operator_decisions") or []
    rows = [
        ("replanInputGeometry", data.get("replanInputGeometry")),
        ("displayCoverageSource", data.get("displayCoverageSource")),
        ("areaProgressDetailCount", data.get("areaProgressDetailCount")),
        ("areaOwnershipDetailCount", data.get("areaOwnershipDetailCount")),
        ("operator_decisions", len(operator_decisions) if isinstance(operator_decisions, list) else 0),
    ]
    try:
        from PyQt5.QtWidgets import QTableWidgetItem

        table.setRowCount(len(rows))
        table.setColumnCount(2)
        if hasattr(table, "setHorizontalHeaderLabels"):
            table.setHorizontalHeaderLabels(["항목", "값"])
        for row_idx, (key, value) in enumerate(rows):
            table.setItem(row_idx, 0, QTableWidgetItem(str(key)))
            table.setItem(row_idx, 1, QTableWidgetItem(str(value if value is not None else "")))
    except Exception:
        return


__all__ = [
    "MissionProgressAreaSnapshotMonitor",
    "_AREA_DIAGNOSTICS_GROUP_TITLE",
    "_MissionAreaState",
    "_apply_sweep_point_progress_to_area_state",
    "_area_ownership_details_for_states",
    "_area_progress_details_for_states",
    "_area_remaining_segments_for_state",
    "_attach_remaining_detail_to_area_ownership",
    "_build_planned_sweep_lines",
    "_coord",
    "_mission_geometry_kind",
    "_populate_area_diagnostics_table",
    "_remaining_geometry_diagnostics",
    "_remaining_geometry_for_state",
    "_snapshot_mission_for_selected_mission",
]
