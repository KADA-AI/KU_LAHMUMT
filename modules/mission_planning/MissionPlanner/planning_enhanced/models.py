from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


Coord = Dict[str, float]


@dataclass
class MissionRecord:
    order: int
    mission_id: Any
    mission_type: int
    mission_detail: Dict[str, Any]


@dataclass
class SplitPiece:
    parent_order: int
    mission_id: Any
    mission_type: int
    piece_index: int
    data: Dict[str, Any]
    assigned_uav: Optional[int] = None


@dataclass
class DirectionDebug:
    parent_order: int
    mission_id: Any
    mission_type: int
    source_area_index: Optional[int] = None
    prev_point: Optional[Coord] = None
    next_point: Optional[Coord] = None
    center_point: Optional[Coord] = None
    bearing_in_deg: Optional[float] = None
    bearing_out_deg: Optional[float] = None
    bearing_move_deg: Optional[float] = None
    bearing_split_deg: Optional[float] = None
    line_start: Optional[Coord] = None
    line_end: Optional[Coord] = None


@dataclass
class SplitRunResult:
    uav_count: int
    uav_ids: List[int]
    pieces: List[SplitPiece] = field(default_factory=list)
    directions: List[DirectionDebug] = field(default_factory=list)
    expected_paths: List[Dict[str, Any]] = field(default_factory=list)
    schedule_result: Dict[str, Any] = field(default_factory=dict)
    planning_mode: Dict[str, Any] = field(default_factory=dict)
    # Type 2 각자도생: sticky ``branch_index -> [aircraftID, ...]`` ownership and the
    # 1-based mission orders that were split per-branch. Empty for other packages.
    branch_ownership: Dict[int, List[int]] = field(default_factory=dict)
    branch_orders: List[int] = field(default_factory=list)
    # Branch indices whose owner UAVs are all permanently unavailable. Their
    # geometry is dropped from the plan rather than handed to another UAV.
    dropped_branches: List[int] = field(default_factory=list)
