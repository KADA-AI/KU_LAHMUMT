from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


Coord = Dict[str, float]
PieceRef = Tuple[int, int]  # (parent_order, piece_index)


@dataclass
class ScheduleCandidate:
    slot_key: str
    candidate_id: str
    parent_order: int
    mission_type: int
    source_area_index: int
    split_stage: int
    base_piece_index: int
    piece_refs: List[PieceRef] = field(default_factory=list)
    distance_m: float = 0.0
    width_ref_m: float = 0.0
    vel_candidates_kmh: List[float] = field(default_factory=list)
    start_point: Optional[Coord] = None
    end_point: Optional[Coord] = None
    label: str = ""
    is_dummy: bool = False


@dataclass
class ScheduleSlot:
    order: int
    slot_key: str
    parent_order: int
    source_area_index: int
    split_stage: int
    candidates: List[ScheduleCandidate] = field(default_factory=list)

