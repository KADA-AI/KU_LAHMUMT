"""CanvasState dataclass shared between PlanningCanvas and DivisionPlannerWindow."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from ._constants import MODE_IDLE, _UAV_IDS, SplitRunResult


@dataclass
class CanvasState:
    mode: str = MODE_IDLE
    mission_kind: Optional[str] = None
    draft_points_xy: List[Tuple[float, float]] = field(default_factory=list)
    mission_points_xy: List[Tuple[float, float]] = field(default_factory=list)
    line_width_m: float = 300.0
    line_width_pending: bool = False
    uav_positions_xy: List[Tuple[float, float]] = field(default_factory=list)
    uav_heading_deg: List[Optional[float]] = field(default_factory=list)
    uav_ids: List[int] = field(default_factory=list)
    split_result: Optional[SplitRunResult] = None
    expected_paths: List[Dict[str, Any]] = field(default_factory=list)
    assignment_path_rows: List[Dict[str, Any]] = field(default_factory=list)
    mission_check_rows: List[Dict[str, Any]] = field(default_factory=list)
    flight_plans_0303: List[Dict[str, Any]] = field(default_factory=list)
    flight_plans_0304: List[Dict[str, Any]] = field(default_factory=list)
    visibility_segments: List[Dict[str, Any]] = field(default_factory=list)
    mid_line_segments: List[Dict[str, Any]] = field(default_factory=list)
    tangent_checks: List[Dict[str, Any]] = field(default_factory=list)
    next_mission_rows: List[Dict[str, Any]] = field(default_factory=list)
    show_next_mission_circles: bool = False
    show_turn_overlays: bool = True
    visible_uav_ids: List[int] = field(default_factory=lambda: list(_UAV_IDS))
