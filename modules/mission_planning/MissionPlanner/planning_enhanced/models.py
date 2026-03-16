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
