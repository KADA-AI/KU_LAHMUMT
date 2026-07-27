from __future__ import annotations

from collections import OrderedDict
import copy
import hashlib
import json
import math
import numbers
import os
import threading
import time
from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path
from types import MethodType
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from PyQt5.QtWidgets import QMessageBox
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import split as geom_split

from modules.common import db_paths, replan_perf
from modules.mission_planning.runtime.next_collab_heading import (
    monitor_heading_to_planner_bearing_deg,
)
try:
    from modules.mission_planning.MissionPlanner.data_def.dubins_turn_link import (
        Pose2D as _DubinsPose2D,
        dubins_shortest_path as _dubins_shortest_path,
    )
except Exception:
    _DubinsPose2D = None
    _dubins_shortest_path = None
from modules.mission_planning.MissionPlanner.planning_enhanced.algo.split_runner import (
    run_split_pipeline,
)
from modules.mission_planning.MissionPlanner.planning_enhanced.models import (
    DirectionDebug,
    SplitPiece,
    SplitRunResult,
)
from modules.mission_planning.MissionPlanner.capture_geometry import (
    capture_aircraft_speed_kmh,
    vertical_sweep_spacing_m,
)
from modules.mission_planning.MissionPlanner.config import SWEEP_SPACING_MARGIN
from modules.mission_planning.MissionPlanner.runtime_settings import (
    MIN_LINE_FOV_DEG,
    apply_runtime_camera_adjusted_fov_deg,
    apply_runtime_camera_adjusted_search_speed,
    format_runtime_camera_fov_adjustment_log,
    get_runtime_bool,
    get_runtime_float,
    load_runtime_settings,
)
from modules.mission_planning.planning_modes import (
    MissionPlanningMode,
    mission_mode_context,
    resolve_mission_planning_mode,
)
from modules.mission_planning.next_area_mode.config import (
    MISSION_LINE,
    MODE_MISSION_READY,
    TURN_PREVIEW_HORIZON_S,
    TURN_PREVIEW_RADIUS_M,
)
from modules.mission_planning.next_area_mode.planner_window import (
    CanvasState,
    NextAreaPlanningWindow,
    coord_to_xy,
    coords_to_xy,
)


@dataclass
class NextCollabLinePlanResult:
    workflow: str
    split_result: SplitRunResult
    mid_line_segments: List[Dict[str, Any]]
    expected_paths: List[Dict[str, Any]]
    planner_result_text: str
    planning_mode: Dict[str, Any] = field(default_factory=dict)


LINE_ROUTE_WP_SPACING_M = 2000.0
LINE_SWEEP_DENSITY_SCALE = 1.2
LINE_ROUTE_OFFSET_SCALE = 1.0
NEXT_COLLAB_ENTRY_TPRIME_TARGET_SEP_RATIO = 0.30
NEXT_COLLAB_ENTRY_TPRIME_RATIO_SCALE = 0.50
NEXT_COLLAB_LINE_DB_WIDTH_WEIGHT = 0.30
NEXT_COLLAB_LINE_DB_SEP_WEIGHT = 0.25
NEXT_COLLAB_LINE_DB_FOV_WEIGHT = 0.45
FOV_DB_SEP_SAFETY_FACTOR = 1.7
LINE_OFFSET_CANDIDATE_SCALES = (0.45, 0.65, 0.85, 1.0)
LINE_SPLIT_CACHE_VERSION = "next-collab-line-split-v3"
LINE_PLAN_CACHE_VERSION = "next-collab-line-plan-v8"
LINE_GEOMETRY_CACHE_VERSION = "next-collab-line-geometry-v4"
LINE_SPLIT_CACHE_DEFAULT_MAX_ENTRIES = 32
LINE_PLAN_CACHE_DEFAULT_MAX_ENTRIES = 32
LINE_GEOMETRY_CACHE_DEFAULT_MAX_ENTRIES = 256
LINE_ENTRY_TPRIME_BINARY_SEARCH_STEPS = 20
LINE_TURN_SOFT_CONFIDENCE = 0.45
LINE_TURN_HARD_CONFIDENCE = 0.70

_LINE_SPLIT_CACHE_LOCK = threading.RLock()
_LINE_SPLIT_CACHE: OrderedDict[str, SplitRunResult] = OrderedDict()
_LINE_SPLIT_CACHE_STATS: Dict[str, int] = {
    "hits": 0,
    "diskHits": 0,
    "misses": 0,
    "stores": 0,
    "skips": 0,
    "errors": 0,
    "evictions": 0,
}
_LINE_GEOMETRY_CACHE_LOCK = threading.RLock()
_LINE_GEOMETRY_CACHE: OrderedDict[str, Any] = OrderedDict()
_LINE_GEOMETRY_CACHE_STATS: Dict[str, int] = {
    "hits": 0,
    "misses": 0,
    "stores": 0,
    "skips": 0,
    "errors": 0,
    "evictions": 0,
}
_LINE_PLAN_CACHE_LOCK = threading.RLock()
_LINE_PLAN_CACHE: OrderedDict[str, "NextCollabLinePlanResult"] = OrderedDict()
_LINE_PLAN_CACHE_STATS: Dict[str, int] = {
    "hits": 0,
    "diskHits": 0,
    "misses": 0,
    "stores": 0,
    "skips": 0,
    "errors": 0,
    "evictions": 0,
}


@dataclass(frozen=True)
class _LineOffsetPlan:
    start_side: float
    end_side: float
    offset_m: float
    score: float
    dubins_length_m: float | None = None
    first_turn_sign: int = 0
    clearance_m: float | None = None

    def side_at(self, progress_ratio: float) -> float:
        t = max(0.0, min(1.0, float(progress_ratio)))
        return float(self.start_side) + ((float(self.end_side) - float(self.start_side)) * t)


def _line_split_cache_enabled() -> bool:
    raw = str(os.environ.get("REPLAN_NEXT_COLLAB_LINE_SPLIT_CACHE", "1") or "").strip().lower()
    return raw not in {"0", "false", "off", "no"}


def _line_split_cache_disk_enabled() -> bool:
    raw = str(os.environ.get("REPLAN_NEXT_COLLAB_LINE_SPLIT_CACHE_PERSIST", "1") or "").strip().lower()
    return raw not in {"0", "false", "off", "no"}


def _line_split_cache_max_entries() -> int:
    try:
        value = int(os.environ.get("REPLAN_NEXT_COLLAB_LINE_SPLIT_CACHE_MAX", str(LINE_SPLIT_CACHE_DEFAULT_MAX_ENTRIES)))
    except Exception:
        value = LINE_SPLIT_CACHE_DEFAULT_MAX_ENTRIES
    return max(1, int(value))


def _line_geometry_cache_enabled() -> bool:
    raw = str(os.environ.get("REPLAN_NEXT_COLLAB_LINE_GEOMETRY_CACHE", "0") or "").strip().lower()
    return raw not in {"0", "false", "off", "no"}


def _line_geometry_cache_max_entries() -> int:
    try:
        value = int(os.environ.get("REPLAN_NEXT_COLLAB_LINE_GEOMETRY_CACHE_MAX", str(LINE_GEOMETRY_CACHE_DEFAULT_MAX_ENTRIES)))
    except Exception:
        value = LINE_GEOMETRY_CACHE_DEFAULT_MAX_ENTRIES
    return max(1, int(value))


def _line_entry_tprime_binary_search_steps() -> int:
    raw_value = os.environ.get("REPLAN_NEXT_COLLAB_TPRIME_ITERATIONS")
    if raw_value is None:
        return int(LINE_ENTRY_TPRIME_BINARY_SEARCH_STEPS)
    try:
        value = int(float(str(raw_value).strip()))
    except Exception:
        value = int(LINE_ENTRY_TPRIME_BINARY_SEARCH_STEPS)
    return max(8, min(40, int(value)))


def _line_plan_cache_enabled() -> bool:
    raw = str(os.environ.get("REPLAN_NEXT_COLLAB_LINE_PLAN_CACHE", "0") or "").strip().lower()
    return raw not in {"0", "false", "off", "no"}


def _line_plan_cache_disk_enabled() -> bool:
    raw = str(os.environ.get("REPLAN_NEXT_COLLAB_LINE_PLAN_CACHE_PERSIST", "0") or "").strip().lower()
    return raw not in {"0", "false", "off", "no"}


def _line_plan_cache_max_entries() -> int:
    try:
        value = int(os.environ.get("REPLAN_NEXT_COLLAB_LINE_PLAN_CACHE_MAX", str(LINE_PLAN_CACHE_DEFAULT_MAX_ENTRIES)))
    except Exception:
        value = LINE_PLAN_CACHE_DEFAULT_MAX_ENTRIES
    return max(1, int(value))


def _line_split_cache_dir() -> Path:
    override = str(os.environ.get("REPLAN_NEXT_COLLAB_LINE_SPLIT_CACHE_DIR", "") or "").strip()
    if override:
        return Path(override)
    return db_paths.get_db_subpath("DSS_Internal", "next_collab_line_split_cache")


def _line_plan_cache_dir() -> Path:
    override = str(os.environ.get("REPLAN_NEXT_COLLAB_LINE_PLAN_CACHE_DIR", "") or "").strip()
    if override:
        return Path(override)
    return db_paths.get_db_subpath("DSS_Internal", "next_collab_line_plan_cache")


def _stable_cache_payload(value: Any) -> Any:
    if isinstance(value, dict):
        return {
            str(key): _stable_cache_payload(child)
            for key, child in sorted(value.items(), key=lambda item: str(item[0]))
        }
    if isinstance(value, (list, tuple)):
        return [_stable_cache_payload(child) for child in value]
    if value is None or isinstance(value, (str, bool)):
        return value
    if isinstance(value, numbers.Integral):
        return int(value)
    if isinstance(value, numbers.Real):
        numeric = float(value)
        if not math.isfinite(numeric):
            return None
        return numeric
    if hasattr(value, "item"):
        try:
            return _stable_cache_payload(value.item())
        except Exception:
            pass
    return str(value)


def _branch_ownership_cache_signature(payload: Dict[str, Any] | None) -> Dict[str, Any]:
    """Include immutable branch ownership in cache identity.

    Package IDs can be reused after an initial-plan refresh while the process is
    still alive; geometry alone must therefore never resurrect a cached mapping
    from another ownership generation.
    """
    source = payload if isinstance(payload, dict) else {}
    mode = source.get("planningMode") if isinstance(source.get("planningMode"), dict) else source
    try:
        package_type = int(mode.get("package_type") or source.get("inputMissionPackageType") or 0)
        package_id = int(
            mode.get("inputMissionPackageID")
            or source.get("inputMissionPackageID")
            or 0
        )
    except Exception:
        return {}
    if package_type not in (2, 3) or package_id <= 0:
        return {}
    try:
        from modules.mission_planning.runtime.state import branch_ownership as store

        meta = store.get_branch_meta(package_id)
    except Exception:
        return {}
    if not isinstance(meta, dict) or not meta.get("ownership"):
        return {}
    return {
        "packageID": int(package_id),
        "ownershipVersion": int(meta.get("ownership_version") or 1),
        "ownership": _stable_cache_payload(meta.get("ownership") or {}),
        "branchMissionIDs": _stable_cache_payload(meta.get("branch_mission_ids") or []),
    }


def _line_split_cache_key(
    *,
    cmpk_payload: Dict[str, Any],
    mrpk_payload: Dict[str, Any],
    uav_ids: List[int],
) -> str:
    key_payload = {
        "version": LINE_SPLIT_CACHE_VERSION,
        "cmpk": _stable_cache_payload(cmpk_payload),
        "mrpk": _stable_cache_payload(mrpk_payload),
        "uavIds": [int(aid) for aid in uav_ids],
        "branchOwnership": _branch_ownership_cache_signature(cmpk_payload),
        "applyAssignment": False,
        "applyScheduling": False,
    }
    raw = json.dumps(key_payload, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def _line_plan_cache_key(
    *,
    mission_payload: Dict[str, Any],
    line_specs: List[Dict[str, Any]],
    aircraft_rows: List[Dict[str, Any]],
    turn_radius_scale: float,
    runtime_cfg: Dict[str, Any],
    fov_db_rows: List[Dict[str, float]],
    planning_mode: Dict[str, Any] | None = None,
) -> str:
    key_payload = {
        "version": LINE_PLAN_CACHE_VERSION,
        "missionPayload": _stable_cache_payload(mission_payload),
        "lineSpecs": _stable_cache_payload(line_specs),
        "aircraftRows": _stable_cache_payload(aircraft_rows),
        "turnRadiusScale": float(turn_radius_scale),
        "runtimeCfg": _stable_cache_payload(runtime_cfg),
        "fovDbRows": _stable_cache_payload(fov_db_rows),
        "planningMode": _stable_cache_payload(planning_mode or {}),
        "branchOwnership": _branch_ownership_cache_signature(planning_mode or {}),
    }
    raw = json.dumps(key_payload, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def _line_geometry_cache_key(kind: str, payload: Dict[str, Any]) -> str:
    key_payload = {
        "version": LINE_GEOMETRY_CACHE_VERSION,
        "kind": str(kind),
        "payload": _stable_cache_payload(payload),
    }
    raw = json.dumps(key_payload, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def _line_geometry_cache_get(key: str) -> Any:
    if not _line_geometry_cache_enabled():
        with _LINE_GEOMETRY_CACHE_LOCK:
            _LINE_GEOMETRY_CACHE_STATS["skips"] += 1
        return None
    try:
        with _LINE_GEOMETRY_CACHE_LOCK:
            cached = _LINE_GEOMETRY_CACHE.get(str(key))
            if cached is None:
                _LINE_GEOMETRY_CACHE_STATS["misses"] += 1
                return None
            _LINE_GEOMETRY_CACHE.move_to_end(str(key))
            _LINE_GEOMETRY_CACHE_STATS["hits"] += 1
            return copy.deepcopy(cached)
    except Exception:
        with _LINE_GEOMETRY_CACHE_LOCK:
            _LINE_GEOMETRY_CACHE_STATS["errors"] += 1
        return None


def _line_geometry_cache_store(key: str, value: Any) -> None:
    if not _line_geometry_cache_enabled():
        with _LINE_GEOMETRY_CACHE_LOCK:
            _LINE_GEOMETRY_CACHE_STATS["skips"] += 1
        return
    try:
        with _LINE_GEOMETRY_CACHE_LOCK:
            _LINE_GEOMETRY_CACHE[str(key)] = copy.deepcopy(value)
            _LINE_GEOMETRY_CACHE.move_to_end(str(key))
            _LINE_GEOMETRY_CACHE_STATS["stores"] += 1
            max_entries = _line_geometry_cache_max_entries()
            while len(_LINE_GEOMETRY_CACHE) > max_entries:
                _LINE_GEOMETRY_CACHE.popitem(last=False)
                _LINE_GEOMETRY_CACHE_STATS["evictions"] += 1
    except Exception:
        with _LINE_GEOMETRY_CACHE_LOCK:
            _LINE_GEOMETRY_CACHE_STATS["errors"] += 1


def _planner_fov_db_signature(planner: _HeadlessLinePlanner) -> List[Dict[str, float]]:
    try:
        rows = planner._fov_db_rows()
    except Exception:
        rows = []
    out: List[Dict[str, float]] = []
    for row in rows or []:
        if not isinstance(row, dict):
            continue
        out.append(
            {
                "width": round(float(row.get("width", 0.0) or 0.0), 6),
                "sep": round(float(row.get("sep", 0.0) or 0.0), 6),
                "vel": round(float(row.get("vel", 0.0) or 0.0), 6),
                "fov": round(float(row.get("fov", 0.0) or 0.0), 6),
            }
        )
    return out


def _piece_geometry_cache_payload(piece: SplitPiece) -> Dict[str, Any]:
    data = piece.data if isinstance(piece.data, dict) else {}
    return {
        "parentOrder": int(piece.parent_order or 0),
        "missionId": _stable_cache_payload(piece.mission_id),
        "missionType": int(piece.mission_type or 0),
        "pieceIndex": int(piece.piece_index or 0),
        "assignedUav": int(piece.assigned_uav) if piece.assigned_uav is not None else None,
        "coordinateList": _stable_cache_payload(data.get("coordinateList") or []),
        "rawCoordinateList": _stable_cache_payload(data.get("rawCoordinateList") or []),
        "centerLineXY": _stable_cache_payload(_piece_centerline_xy(piece)),
        "widthM": _stable_cache_payload(_piece_width_m(piece)),
    }


def _line_runtime_cache_value(value: Any) -> Any:
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value.hex() if math.isfinite(value) else None
    if isinstance(value, numbers.Real):
        numeric = float(value)
        return numeric.hex() if math.isfinite(numeric) else None
    if isinstance(value, (list, tuple)):
        return tuple(_line_runtime_cache_value(item) for item in value)
    return str(value)


def _line_runtime_cache_get(planner: _HeadlessLinePlanner, key: tuple[Any, ...]) -> Any:
    cache = getattr(planner, "_line_runtime_geometry_cache", None)
    if not isinstance(cache, dict):
        cache = {}
        setattr(planner, "_line_runtime_geometry_cache", cache)
    cached = cache.get(key)
    if cached is None:
        return None
    return copy.deepcopy(cached)


def _line_runtime_cache_store(planner: _HeadlessLinePlanner, key: tuple[Any, ...], value: Any) -> None:
    cache = getattr(planner, "_line_runtime_geometry_cache", None)
    if not isinstance(cache, dict):
        cache = {}
        setattr(planner, "_line_runtime_geometry_cache", cache)
    cache[key] = copy.deepcopy(value)


def _line_split_cache_path(key: str) -> Path:
    safe_key = "".join(ch for ch in str(key) if ch.isalnum() or ch in {"-", "_"})
    return _line_split_cache_dir() / f"{safe_key or hashlib.sha256(str(key).encode('utf-8')).hexdigest()}.json"


def _split_piece_to_payload(piece: SplitPiece) -> Dict[str, Any]:
    return {
        "parent_order": int(piece.parent_order),
        "mission_id": _stable_cache_payload(piece.mission_id),
        "mission_type": int(piece.mission_type),
        "piece_index": int(piece.piece_index),
        "data": _stable_cache_payload(piece.data),
        "assigned_uav": int(piece.assigned_uav) if piece.assigned_uav is not None else None,
    }


def _direction_to_payload(direction: DirectionDebug) -> Dict[str, Any]:
    return {
        "parent_order": int(direction.parent_order),
        "mission_id": _stable_cache_payload(direction.mission_id),
        "mission_type": int(direction.mission_type),
        "source_area_index": (
            int(direction.source_area_index)
            if direction.source_area_index is not None
            else None
        ),
        "prev_point": _stable_cache_payload(direction.prev_point),
        "next_point": _stable_cache_payload(direction.next_point),
        "center_point": _stable_cache_payload(direction.center_point),
        "bearing_in_deg": _stable_cache_payload(direction.bearing_in_deg),
        "bearing_out_deg": _stable_cache_payload(direction.bearing_out_deg),
        "bearing_move_deg": _stable_cache_payload(direction.bearing_move_deg),
        "bearing_split_deg": _stable_cache_payload(direction.bearing_split_deg),
        "line_start": _stable_cache_payload(direction.line_start),
        "line_end": _stable_cache_payload(direction.line_end),
    }


def _split_result_to_payload(result: SplitRunResult) -> Dict[str, Any]:
    return {
        "uav_count": int(result.uav_count),
        "uav_ids": [int(aid) for aid in (result.uav_ids or [])],
        "pieces": [_split_piece_to_payload(piece) for piece in (result.pieces or [])],
        "directions": [_direction_to_payload(direction) for direction in (result.directions or [])],
        "expected_paths": _stable_cache_payload(result.expected_paths or []),
        "schedule_result": _stable_cache_payload(result.schedule_result or {}),
        "planning_mode": _stable_cache_payload(getattr(result, "planning_mode", {}) or {}),
        "branch_ownership": _stable_cache_payload(
            getattr(result, "branch_ownership", {}) or {}
        ),
        "branch_orders": [
            int(order) for order in (getattr(result, "branch_orders", []) or [])
        ],
    }


def _split_piece_from_payload(payload: Dict[str, Any]) -> SplitPiece:
    return SplitPiece(
        parent_order=int(payload.get("parent_order") or 0),
        mission_id=payload.get("mission_id"),
        mission_type=int(payload.get("mission_type") or 0),
        piece_index=int(payload.get("piece_index") or 0),
        data=copy.deepcopy(payload.get("data") if isinstance(payload.get("data"), dict) else {}),
        assigned_uav=(
            int(payload.get("assigned_uav"))
            if payload.get("assigned_uav") is not None
            else None
        ),
    )


def _direction_from_payload(payload: Dict[str, Any]) -> DirectionDebug:
    return DirectionDebug(
        parent_order=int(payload.get("parent_order") or 0),
        mission_id=payload.get("mission_id"),
        mission_type=int(payload.get("mission_type") or 0),
        source_area_index=(
            int(payload.get("source_area_index"))
            if payload.get("source_area_index") is not None
            else None
        ),
        prev_point=copy.deepcopy(payload.get("prev_point")),
        next_point=copy.deepcopy(payload.get("next_point")),
        center_point=copy.deepcopy(payload.get("center_point")),
        bearing_in_deg=payload.get("bearing_in_deg"),
        bearing_out_deg=payload.get("bearing_out_deg"),
        bearing_move_deg=payload.get("bearing_move_deg"),
        bearing_split_deg=payload.get("bearing_split_deg"),
        line_start=copy.deepcopy(payload.get("line_start")),
        line_end=copy.deepcopy(payload.get("line_end")),
    )


def _split_result_from_payload(payload: Dict[str, Any]) -> SplitRunResult:
    return SplitRunResult(
        uav_count=int(payload.get("uav_count") or 0),
        uav_ids=[int(aid) for aid in (payload.get("uav_ids") or [])],
        pieces=[
            _split_piece_from_payload(piece)
            for piece in (payload.get("pieces") or [])
            if isinstance(piece, dict)
        ],
        directions=[
            _direction_from_payload(direction)
            for direction in (payload.get("directions") or [])
            if isinstance(direction, dict)
        ],
        expected_paths=copy.deepcopy(payload.get("expected_paths") if isinstance(payload.get("expected_paths"), list) else []),
        schedule_result=copy.deepcopy(payload.get("schedule_result") if isinstance(payload.get("schedule_result"), dict) else {}),
        planning_mode=copy.deepcopy(payload.get("planning_mode") if isinstance(payload.get("planning_mode"), dict) else {}),
        branch_ownership={
            int(branch_index): [int(aircraft_id) for aircraft_id in owners]
            for branch_index, owners in (
                payload.get("branch_ownership")
                if isinstance(payload.get("branch_ownership"), dict)
                else {}
            ).items()
            if isinstance(owners, list)
        },
        branch_orders=[int(order) for order in (payload.get("branch_orders") or [])],
    )


def _clone_split_result(result: SplitRunResult) -> SplitRunResult:
    return _split_result_from_payload(_split_result_to_payload(result))


def _line_plan_result_to_payload(result: NextCollabLinePlanResult) -> Dict[str, Any]:
    return {
        "workflow": str(result.workflow or ""),
        "splitResult": _split_result_to_payload(result.split_result),
        "midLineSegments": _stable_cache_payload(result.mid_line_segments or []),
        "expectedPaths": _stable_cache_payload(result.expected_paths or []),
        "plannerResultText": str(result.planner_result_text or ""),
        "planningMode": _stable_cache_payload(result.planning_mode or {}),
    }


def _line_plan_result_from_payload(payload: Dict[str, Any]) -> NextCollabLinePlanResult:
    split_payload = payload.get("splitResult") if isinstance(payload.get("splitResult"), dict) else {}
    return NextCollabLinePlanResult(
        workflow=str(payload.get("workflow") or ""),
        split_result=_split_result_from_payload(split_payload),
        mid_line_segments=copy.deepcopy(payload.get("midLineSegments") if isinstance(payload.get("midLineSegments"), list) else []),
        expected_paths=copy.deepcopy(payload.get("expectedPaths") if isinstance(payload.get("expectedPaths"), list) else []),
        planner_result_text=str(payload.get("plannerResultText") or ""),
        planning_mode=copy.deepcopy(payload.get("planningMode") if isinstance(payload.get("planningMode"), dict) else {}),
    )


def _clone_line_plan_result(result: NextCollabLinePlanResult) -> NextCollabLinePlanResult:
    return _line_plan_result_from_payload(_line_plan_result_to_payload(result))


def _remember_line_split_cache(key: str, result: SplitRunResult) -> None:
    with _LINE_SPLIT_CACHE_LOCK:
        _LINE_SPLIT_CACHE[str(key)] = _clone_split_result(result)
        _LINE_SPLIT_CACHE.move_to_end(str(key))
        max_entries = _line_split_cache_max_entries()
        while len(_LINE_SPLIT_CACHE) > max_entries:
            _LINE_SPLIT_CACHE.popitem(last=False)
            _LINE_SPLIT_CACHE_STATS["evictions"] += 1


def _load_line_split_cache_from_disk(key: str) -> SplitRunResult | None:
    if not _line_split_cache_disk_enabled():
        return None
    path = _line_split_cache_path(key)
    try:
        with path.open("r", encoding="utf-8") as fh:
            payload = json.load(fh)
        if not isinstance(payload, dict):
            return None
        if str(payload.get("key") or "") != str(key):
            return None
        if str(payload.get("version") or "") != LINE_SPLIT_CACHE_VERSION:
            return None
        result_payload = payload.get("splitResult")
        if not isinstance(result_payload, dict):
            return None
        result = _split_result_from_payload(result_payload)
        if not result.pieces:
            return None
        try:
            os.utime(path, None)
        except Exception:
            pass
        return result
    except FileNotFoundError:
        return None
    except Exception:
        with _LINE_SPLIT_CACHE_LOCK:
            _LINE_SPLIT_CACHE_STATS["errors"] += 1
        return None


def _store_line_split_cache_to_disk(key: str, result: SplitRunResult) -> None:
    if not _line_split_cache_disk_enabled():
        return
    try:
        cache_dir = _line_split_cache_dir()
        cache_dir.mkdir(parents=True, exist_ok=True)
        path = _line_split_cache_path(key)
        tmp = path.with_suffix(f".tmp-{os.getpid()}-{threading.get_ident()}")
        payload = {
            "version": LINE_SPLIT_CACHE_VERSION,
            "key": str(key),
            "createdUnixMs": int(time.time() * 1000),
            "splitResult": _split_result_to_payload(result),
        }
        with tmp.open("w", encoding="utf-8") as fh:
            json.dump(payload, fh, ensure_ascii=False, separators=(",", ":"))
        os.replace(tmp, path)
        entries = [item for item in cache_dir.glob("*.json") if item.is_file()]
        overflow = len(entries) - _line_split_cache_max_entries()
        if overflow > 0:
            entries.sort(key=lambda item: item.stat().st_mtime)
            for stale in entries[:overflow]:
                try:
                    stale.unlink()
                    with _LINE_SPLIT_CACHE_LOCK:
                        _LINE_SPLIT_CACHE_STATS["evictions"] += 1
                except Exception:
                    pass
    except Exception:
        with _LINE_SPLIT_CACHE_LOCK:
            _LINE_SPLIT_CACHE_STATS["errors"] += 1


def _cached_line_split_result(key: str) -> tuple[SplitRunResult | None, str]:
    with _LINE_SPLIT_CACHE_LOCK:
        cached = _LINE_SPLIT_CACHE.get(str(key))
        if cached is not None:
            _LINE_SPLIT_CACHE.move_to_end(str(key))
            _LINE_SPLIT_CACHE_STATS["hits"] += 1
            return _clone_split_result(cached), "memory"
    disk_result = _load_line_split_cache_from_disk(str(key))
    if disk_result is not None:
        _remember_line_split_cache(str(key), disk_result)
        with _LINE_SPLIT_CACHE_LOCK:
            _LINE_SPLIT_CACHE_STATS["hits"] += 1
            _LINE_SPLIT_CACHE_STATS["diskHits"] += 1
        return _clone_split_result(disk_result), "disk"
    with _LINE_SPLIT_CACHE_LOCK:
        _LINE_SPLIT_CACHE_STATS["misses"] += 1
    return None, "miss"


def _remember_line_plan_cache(key: str, result: NextCollabLinePlanResult) -> None:
    with _LINE_PLAN_CACHE_LOCK:
        _LINE_PLAN_CACHE[str(key)] = _clone_line_plan_result(result)
        _LINE_PLAN_CACHE.move_to_end(str(key))
        max_entries = _line_plan_cache_max_entries()
        while len(_LINE_PLAN_CACHE) > max_entries:
            _LINE_PLAN_CACHE.popitem(last=False)
            _LINE_PLAN_CACHE_STATS["evictions"] += 1


def _line_plan_cache_path(key: str) -> Path:
    safe_key = "".join(ch for ch in str(key) if ch.isalnum() or ch in {"-", "_"})
    return _line_plan_cache_dir() / f"{safe_key or hashlib.sha256(str(key).encode('utf-8')).hexdigest()}.json"


def _load_line_plan_cache_from_disk(key: str) -> NextCollabLinePlanResult | None:
    if not _line_plan_cache_disk_enabled():
        return None
    path = _line_plan_cache_path(key)
    try:
        with path.open("r", encoding="utf-8") as fh:
            payload = json.load(fh)
        if not isinstance(payload, dict):
            return None
        if str(payload.get("key") or "") != str(key):
            return None
        if str(payload.get("version") or "") != LINE_PLAN_CACHE_VERSION:
            return None
        result_payload = payload.get("planResult")
        if not isinstance(result_payload, dict):
            return None
        result = _line_plan_result_from_payload(result_payload)
        if not result.expected_paths:
            return None
        try:
            os.utime(path, None)
        except Exception:
            pass
        return result
    except FileNotFoundError:
        return None
    except Exception:
        with _LINE_PLAN_CACHE_LOCK:
            _LINE_PLAN_CACHE_STATS["errors"] += 1
        return None


def _store_line_plan_cache_to_disk(key: str, result: NextCollabLinePlanResult) -> None:
    if not _line_plan_cache_disk_enabled():
        return
    try:
        cache_dir = _line_plan_cache_dir()
        cache_dir.mkdir(parents=True, exist_ok=True)
        path = _line_plan_cache_path(key)
        tmp = path.with_suffix(f".tmp-{os.getpid()}-{threading.get_ident()}")
        payload = {
            "version": LINE_PLAN_CACHE_VERSION,
            "key": str(key),
            "createdUnixMs": int(time.time() * 1000),
            "planResult": _line_plan_result_to_payload(result),
        }
        with tmp.open("w", encoding="utf-8") as fh:
            json.dump(payload, fh, ensure_ascii=False, separators=(",", ":"))
        os.replace(tmp, path)
        entries = [item for item in cache_dir.glob("*.json") if item.is_file()]
        overflow = len(entries) - _line_plan_cache_max_entries()
        if overflow > 0:
            entries.sort(key=lambda item: item.stat().st_mtime)
            for stale in entries[:overflow]:
                try:
                    stale.unlink()
                    with _LINE_PLAN_CACHE_LOCK:
                        _LINE_PLAN_CACHE_STATS["evictions"] += 1
                except Exception:
                    pass
    except Exception:
        with _LINE_PLAN_CACHE_LOCK:
            _LINE_PLAN_CACHE_STATS["errors"] += 1


def _cached_line_plan_result(key: str) -> tuple[NextCollabLinePlanResult | None, str]:
    with _LINE_PLAN_CACHE_LOCK:
        cached = _LINE_PLAN_CACHE.get(str(key))
        if cached is not None:
            _LINE_PLAN_CACHE.move_to_end(str(key))
            _LINE_PLAN_CACHE_STATS["hits"] += 1
            return _clone_line_plan_result(cached), "memory"
    disk_result = _load_line_plan_cache_from_disk(str(key))
    if disk_result is not None:
        _remember_line_plan_cache(str(key), disk_result)
        with _LINE_PLAN_CACHE_LOCK:
            _LINE_PLAN_CACHE_STATS["hits"] += 1
            _LINE_PLAN_CACHE_STATS["diskHits"] += 1
        return _clone_line_plan_result(disk_result), "disk"
    with _LINE_PLAN_CACHE_LOCK:
        _LINE_PLAN_CACHE_STATS["misses"] += 1
    return None, "miss"


def _run_line_split_pipeline_cached(
    cmpk_payload: Dict[str, Any],
    mrpk_payload: Dict[str, Any],
    uav_ids: List[int],
    *,
    planning_mode: MissionPlanningMode | Dict[str, Any] | None = None,
    log: Callable[[str], None] | None = None,
) -> SplitRunResult:
    if not _line_split_cache_enabled():
        with _LINE_SPLIT_CACHE_LOCK:
            _LINE_SPLIT_CACHE_STATS["skips"] += 1
        return run_split_pipeline(
            cmpk_payload,
            mrpk_payload,
            list(uav_ids),
            apply_assignment=False,
            apply_scheduling=False,
            planning_mode=planning_mode,
        )
    try:
        key = _line_split_cache_key(
            cmpk_payload=cmpk_payload,
            mrpk_payload=mrpk_payload,
            uav_ids=list(uav_ids),
        )
    except Exception:
        with _LINE_SPLIT_CACHE_LOCK:
            _LINE_SPLIT_CACHE_STATS["errors"] += 1
        return run_split_pipeline(
            cmpk_payload,
            mrpk_payload,
            list(uav_ids),
            apply_assignment=False,
            apply_scheduling=False,
            planning_mode=planning_mode,
        )

    cached, source = _cached_line_split_result(key)
    if cached is not None:
        if log is not None:
            log(
                "[NEXTCOLLAB][LINE][CACHE] split_result hit "
                f"source={source} pieces={len(cached.pieces)} key={key[:12]}"
            )
        return cached

    started = time.perf_counter()
    result = run_split_pipeline(
        cmpk_payload,
        mrpk_payload,
        list(uav_ids),
        apply_assignment=False,
        apply_scheduling=False,
        planning_mode=planning_mode,
    )
    elapsed_ms = (time.perf_counter() - started) * 1000.0
    try:
        _remember_line_split_cache(key, result)
        _store_line_split_cache_to_disk(key, result)
        with _LINE_SPLIT_CACHE_LOCK:
            _LINE_SPLIT_CACHE_STATS["stores"] += 1
        if log is not None:
            log(
                "[NEXTCOLLAB][LINE][CACHE] split_result stored "
                f"pieces={len(result.pieces)} key={key[:12]} elapsed={elapsed_ms:.1f} ms"
            )
    except Exception:
        with _LINE_SPLIT_CACHE_LOCK:
            _LINE_SPLIT_CACHE_STATS["errors"] += 1
    return result


def _cached_mid_line_overlay_bundle(
    planner: _HeadlessLinePlanner,
    split_result: SplitRunResult,
) -> Tuple[float, List[Dict[str, Any]], List[str]]:
    if not _line_geometry_cache_enabled():
        return planner._mid_line_overlay_bundle(split_result)
    payload = {
        "pieces": [
            _piece_geometry_cache_payload(piece)
            for piece in sorted(split_result.pieces or [], key=lambda row: int(row.piece_index or 0))
        ],
        "fovDbRows": _planner_fov_db_signature(planner),
    }
    key = _line_geometry_cache_key("mid_line_overlay_bundle", payload)
    cached = _line_geometry_cache_get(key)
    if cached is not None:
        replan_perf.add("mission_planning.next_collab_line.cache.mid_overlay", hit=1)
        reference_bearing_deg, overlays, lines = cached
        return float(reference_bearing_deg), list(overlays or []), list(lines or [])
    replan_perf.add("mission_planning.next_collab_line.cache.mid_overlay", miss=1)
    result = planner._mid_line_overlay_bundle(split_result)
    _line_geometry_cache_store(key, result)
    reference_bearing_deg, overlays, lines = copy.deepcopy(result)
    return float(reference_bearing_deg), list(overlays or []), list(lines or [])


class _HeadlessLinePlanner:
    def __init__(self) -> None:
        self.state = CanvasState(line_width_m=300.0)
        self._selected_uav_count = 1
        self._cmpk_payload = None
        self._mrpk_payload = None
        self._fov_db_rows_cache = None
        self._fov_db_widths_cache = None
        self.stage2_ratio_spins = {}
        self._result_text_buffer = ""
        self.canvas = None

    def hide(self) -> None:
        return

    def close(self) -> None:
        return

    def _sync_canvas(self) -> None:
        return

    def _refresh_ui(self) -> None:
        return

    def _append_result(self, text: str) -> None:
        existing = str(getattr(self, "_result_text_buffer", "") or "")
        self._result_text_buffer = f"{existing}\n{text}".strip()

    def _set_result(self, text: str) -> None:
        self._result_text_buffer = str(text or "")


for _name, _value in NextAreaPlanningWindow.__dict__.items():
    if _name.startswith("__"):
        continue
    if hasattr(_HeadlessLinePlanner, _name):
        continue
    if callable(_value):
        setattr(_HeadlessLinePlanner, _name, _value)


def _planner_fov_db_numeric_rows(
    planner: _HeadlessLinePlanner,
) -> tuple[tuple[float, float, float, float], ...]:
    """Return cached FOV DB rows as (width, sep, fov, vel) tuples for hot-path scans."""
    if getattr(planner, "_fov_db_rows_cache", None) is None:
        try:
            planner._fov_db_rows()
        except Exception:
            return tuple()
    sig = getattr(planner, "_fov_db_cache_sig", None)
    cached_sig = getattr(planner, "_line_fov_db_numeric_rows_sig", None)
    cached_rows = getattr(planner, "_line_fov_db_numeric_rows_cache", None)
    if cached_sig == sig and isinstance(cached_rows, tuple):
        return cached_rows

    rows: List[tuple[float, float, float, float]] = []
    for row in getattr(planner, "_fov_db_rows_cache", None) or []:
        if not isinstance(row, dict):
            continue
        try:
            width_m = float(row.get("width", 0.0) or 0.0)
            sep_m = float(row.get("sep", 0.0) or 0.0)
            fov_deg = float(row.get("fov", 0.0) or 0.0)
            vel_mps = float(row.get("vel", 0.0) or 0.0)
        except Exception:
            continue
        rows.append((float(width_m), float(sep_m), float(fov_deg), float(vel_mps)))
    cached = tuple(rows)
    setattr(planner, "_line_fov_db_numeric_rows_sig", sig)
    setattr(planner, "_line_fov_db_numeric_rows_cache", cached)
    return cached


def _distance_xy(start_xy: Tuple[float, float], end_xy: Tuple[float, float]) -> float:
    dx = float(end_xy[0]) - float(start_xy[0])
    dy = float(end_xy[1]) - float(start_xy[1])
    return math.hypot(dx, dy)


def _runtime_fov_db_sep_safety_factor(runtime_cfg: Dict[str, Any] | None = None) -> float:
    factor = float(get_runtime_float("fov_db_sep_safety_factor", FOV_DB_SEP_SAFETY_FACTOR, runtime_cfg))
    if factor <= 0.0:
        factor = 1.0
    return float(factor)


def _db_sep_requirement_m(sep_m: float, runtime_cfg: Dict[str, Any] | None = None) -> float:
    factor = _runtime_fov_db_sep_safety_factor(runtime_cfg)
    return max(0.0, float(sep_m or 0.0)) * float(factor)


def _runtime_line_density_scale(runtime_cfg: Dict[str, Any] | None = None) -> float:
    value = float(get_runtime_float("line_density_scale", LINE_SWEEP_DENSITY_SCALE, runtime_cfg))
    return max(value, 1e-6)


def _runtime_line_search_speed_weight(runtime_cfg: Dict[str, Any] | None = None) -> float:
    value = float(get_runtime_float("search_speed_weight", 1.0, runtime_cfg))
    return max(value, 0.1)


def _runtime_line_route_offset_scale(runtime_cfg: Dict[str, Any] | None = None) -> float:
    value = float(get_runtime_float("line_route_offset_scale", LINE_ROUTE_OFFSET_SCALE, runtime_cfg))
    return max(value, 0.0)


def _fov_db_min_sep_for_fov(
    planner: _HeadlessLinePlanner,
    fov_deg: float,
    *,
    runtime_cfg: Dict[str, Any] | None = None,
) -> float:
    try:
        target_fov = float(fov_deg)
    except Exception:
        return 0.0
    if target_fov <= 0.0:
        return 0.0
    cache_key = (
        "fov_db_min_sep_for_fov",
        _line_runtime_cache_value(float(target_fov)),
        id(runtime_cfg) if runtime_cfg is not None else None,
    )
    cached = _line_runtime_cache_get(planner, cache_key)
    if cached is not None:
        return float(cached)
    rows = _planner_fov_db_numeric_rows(planner)
    if not rows:
        return 0.0

    matches: List[float] = []
    for _width_m, row_sep, row_fov, _vel_mps in rows:
        if row_fov <= 0.0 or row_sep <= 0.0:
            continue
        if abs(row_fov - target_fov) <= 0.05:
            matches.append(float(row_sep))

    if not matches:
        for _width_m, row_sep, row_fov, _vel_mps in rows:
            if row_fov <= 0.0 or row_sep <= 0.0:
                continue
            try:
                adjusted_fov = float(
                    apply_runtime_camera_adjusted_fov_deg(
                        row_fov,
                        runtime_cfg,
                        minimum_fov_deg=MIN_LINE_FOV_DEG,
                        context="NEXTCOLLAB LINE OFFSET_DB",
                    )
                )
            except Exception:
                adjusted_fov = row_fov
            if abs(adjusted_fov - target_fov) <= 0.05:
                matches.append(float(row_sep))

    result = float(min(matches) if matches else 0.0)
    _line_runtime_cache_store(planner, cache_key, result)
    return result


def _route_offset_sep_for_fov(
    planner: _HeadlessLinePlanner,
    fov_deg: float,
    default_sep_m: float,
    *,
    runtime_cfg: Dict[str, Any] | None = None,
) -> float:
    try:
        default_sep = float(default_sep_m)
    except Exception:
        default_sep = 0.0
    min_sep = _fov_db_min_sep_for_fov(planner, float(fov_deg or 0.0), runtime_cfg=runtime_cfg)
    if min_sep > 0.0:
        return float(min_sep)
    return max(float(default_sep), 0.0)


def _runtime_line_route_wp_spacing_m(runtime_cfg: Dict[str, Any] | None = None) -> float:
    value = float(get_runtime_float("uav_wp_interval_m", LINE_ROUTE_WP_SPACING_M, runtime_cfg))
    return max(value, 1.0)


def _sweep_spacing_margin_for_density() -> float:
    try:
        margin = float(SWEEP_SPACING_MARGIN)
    except Exception:
        margin = 1.0
    if margin <= 0.0:
        margin = 1.0
    return max(margin, 1e-6)


def _sweep_spacing_m(
    *,
    separation_m: float,
    fov_deg: float,
    spacing_scale: float = 1.0,
    runtime_cfg: Dict[str, Any] | None = None,
) -> float:
    # Unified capture law: spacing = nadir 세로 footprint * (1 - vertical
    # overlap). The FOV DB only selects the FOV; separation_m/spacing_scale
    # are kept for signature compatibility and used as a fallback when the
    # FOV cannot produce a footprint.
    spacing = vertical_sweep_spacing_m(fov_deg, runtime_cfg=runtime_cfg)
    if spacing is not None and spacing > 0.0:
        return float(spacing)
    base = 2.0 * max(float(separation_m), 1.0) * math.tan(max(math.radians(float(fov_deg)) / 2.0, 1e-6))
    try:
        effective_scale = float(spacing_scale)
    except Exception:
        effective_scale = 1.0
    if effective_scale <= 0.0:
        effective_scale = 1.0
    return max(base * effective_scale, 1.0)


def _line_raw_spacing_scale(runtime_cfg: Dict[str, Any] | None = None) -> float:
    density = _runtime_line_density_scale(runtime_cfg)
    sep_safety = _runtime_fov_db_sep_safety_factor(runtime_cfg)
    if sep_safety <= 0.0:
        sep_safety = 1.0
    return max(1.0 / (density * sep_safety * _sweep_spacing_margin_for_density()), 1e-6)


def _line_sweep_spacing_m(
    *,
    separation_m: float,
    fov_deg: float,
    runtime_cfg: Dict[str, Any] | None = None,
) -> float:
    return _sweep_spacing_m(
        separation_m=float(separation_m),
        fov_deg=float(fov_deg),
        spacing_scale=_line_raw_spacing_scale(runtime_cfg),
        runtime_cfg=runtime_cfg,
    )


def _avg_sweep_line_length_m(sweep_lines_xy: Any) -> float:
    total = 0.0
    count = 0
    for line_xy in sweep_lines_xy or []:
        if not isinstance(line_xy, (list, tuple)) or len(line_xy) < 2:
            continue
        length = 0.0
        for (x0, y0), (x1, y1) in zip(line_xy[:-1], line_xy[1:]):
            length += math.hypot(float(x1) - float(x0), float(y1) - float(y0))
        if length > 0.0:
            total += length
            count += 1
    return total / count if count else 0.0


def _capture_line_speed_kmh(
    fov_deg: float,
    sweep_lines_xy: Any,
    *,
    runtime_cfg: Dict[str, Any] | None = None,
    fallback_kmh: float = 0.0,
) -> float:
    avg_length_m = _avg_sweep_line_length_m(sweep_lines_xy)
    speed_kmh = capture_aircraft_speed_kmh(
        fov_deg,
        line_length_m=avg_length_m if avg_length_m > 0.0 else None,
        runtime_cfg=runtime_cfg,
    )
    if speed_kmh is not None and speed_kmh > 0.0:
        return float(speed_kmh)
    return float(fallback_kmh)


def _runtime_line_manual_base_fov_deg(runtime_cfg: Dict[str, Any] | None = None) -> float | None:
    auto_from_db = bool(get_runtime_bool("enhanced_auto_fov_from_db", True, runtime_cfg))
    if auto_from_db:
        return None
    manual_fov_deg = _to_float(
        get_runtime_float(
            "line_custom_fov_deg",
            0.0,
            runtime_cfg,
        )
    )
    if manual_fov_deg is None or manual_fov_deg <= 0.0:
        return None
    return _clamp_line_fov_deg(manual_fov_deg)


def _runtime_line_manual_fov_deg(runtime_cfg: Dict[str, Any] | None = None) -> float | None:
    manual_fov_deg = _runtime_line_manual_base_fov_deg(runtime_cfg)
    if manual_fov_deg is None:
        return None
    adjusted_fov_deg = apply_runtime_camera_adjusted_fov_deg(
        manual_fov_deg,
        runtime_cfg,
        minimum_fov_deg=MIN_LINE_FOV_DEG,
        context="NEXTCOLLAB LINE MANUAL",
    )
    return _clamp_line_fov_deg(adjusted_fov_deg) if adjusted_fov_deg > 0.0 else None


def _runtime_line_fov_db_smaller_fov_steps(runtime_cfg: Dict[str, Any] | None = None) -> int:
    try:
        steps = int(float(get_runtime_float("fov_db_smaller_fov_steps", 4, runtime_cfg)))
    except Exception:
        steps = 4
    return max(3, min(10, int(steps)))


def _bearing_deg_from_xy(start_xy: Tuple[float, float], end_xy: Tuple[float, float]) -> float:
    dx = float(end_xy[0]) - float(start_xy[0])
    dy = float(end_xy[1]) - float(start_xy[1])
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return 0.0
    return float((math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0)


def _bearing_unit_xy(bearing_deg: float) -> Tuple[float, float]:
    theta = math.radians(float(bearing_deg) % 360.0)
    return math.sin(theta), math.cos(theta)


def _bearing_to_yaw_rad(bearing_deg: float) -> float:
    return math.radians(90.0 - (float(bearing_deg) % 360.0))


def _angle_delta_deg(left_deg: float, right_deg: float) -> float:
    return abs(((float(left_deg) - float(right_deg) + 180.0) % 360.0) - 180.0)


def _line_route_frame_xy(
    route_line_xy: Sequence[Tuple[float, float]],
) -> tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float], Tuple[float, float]] | None:
    if len(route_line_xy) < 2:
        return None
    start_xy = (float(route_line_xy[0][0]), float(route_line_xy[0][1]))
    end_xy = (float(route_line_xy[-1][0]), float(route_line_xy[-1][1]))
    dx = float(end_xy[0]) - float(start_xy[0])
    dy = float(end_xy[1]) - float(start_xy[1])
    norm = math.hypot(dx, dy)
    if norm <= 1e-6:
        return None
    unit_xy = (dx / norm, dy / norm)
    normal_xy = (-unit_xy[1], unit_xy[0])
    return start_xy, end_xy, unit_xy, normal_xy


def _project_to_route_ratio_xy(
    point_xy: Tuple[float, float],
    route_start_xy: Tuple[float, float],
    route_end_xy: Tuple[float, float],
) -> tuple[Tuple[float, float], float]:
    sx, sy = float(route_start_xy[0]), float(route_start_xy[1])
    ex, ey = float(route_end_xy[0]), float(route_end_xy[1])
    px, py = float(point_xy[0]), float(point_xy[1])
    dx = ex - sx
    dy = ey - sy
    denom = (dx * dx) + (dy * dy)
    if denom <= 1e-9:
        return (sx, sy), 0.0
    t = (((px - sx) * dx) + ((py - sy) * dy)) / denom
    t = max(0.0, min(1.0, float(t)))
    return (sx + (dx * t), sy + (dy * t)), float(t)


def _route_polyline_rows_xy(
    route_line_xy: Sequence[Tuple[float, float]],
) -> List[Tuple[float, float]]:
    rows: List[Tuple[float, float]] = []
    for row in route_line_xy or []:
        if not isinstance(row, (tuple, list)) or len(row) < 2:
            continue
        try:
            point_xy = (float(row[0]), float(row[1]))
        except Exception:
            continue
        if rows and _distance_xy(rows[-1], point_xy) <= 0.5:
            continue
        rows.append(point_xy)
    return rows


def _project_to_route_polyline_xy(
    point_xy: Tuple[float, float],
    route_line_xy: Sequence[Tuple[float, float]],
) -> tuple[Tuple[float, float], Tuple[float, float], float] | None:
    rows = _route_polyline_rows_xy(route_line_xy)
    if len(rows) < 2:
        return None
    total_len_m = 0.0
    for idx in range(len(rows) - 1):
        total_len_m += _distance_xy(rows[idx], rows[idx + 1])
    if total_len_m <= 1e-6:
        return None

    px, py = float(point_xy[0]), float(point_xy[1])
    walked_m = 0.0
    best: tuple[float, Tuple[float, float], Tuple[float, float], float] | None = None
    for idx in range(len(rows) - 1):
        sx, sy = float(rows[idx][0]), float(rows[idx][1])
        ex, ey = float(rows[idx + 1][0]), float(rows[idx + 1][1])
        dx = ex - sx
        dy = ey - sy
        seg_len_m = math.hypot(dx, dy)
        if seg_len_m <= 1e-6:
            continue
        denom = (dx * dx) + (dy * dy)
        ratio = (((px - sx) * dx) + ((py - sy) * dy)) / max(denom, 1e-9)
        ratio = max(0.0, min(1.0, float(ratio)))
        center_xy = (sx + (dx * ratio), sy + (dy * ratio))
        tangent_xy = (dx / seg_len_m, dy / seg_len_m)
        progress_ratio = max(0.0, min(1.0, (walked_m + (seg_len_m * ratio)) / total_len_m))
        dist_m = _distance_xy((px, py), center_xy)
        if best is None or dist_m < best[0]:
            best = (float(dist_m), center_xy, tangent_xy, float(progress_ratio))
        walked_m += seg_len_m
    if best is None:
        return None
    return best[1], best[2], best[3]


def _line_anchor_from_route_plan_xy(
    sweep_xy: Sequence[Tuple[float, float]],
    *,
    route_line_xy: Sequence[Tuple[float, float]],
    offset_plan: _LineOffsetPlan,
) -> Tuple[float, float] | None:
    if len(sweep_xy) < 2:
        return None
    midpoint_xy = _midpoint_xy(sweep_xy)
    if midpoint_xy is None:
        return None
    projected = _project_to_route_polyline_xy(midpoint_xy, route_line_xy)
    if projected is not None:
        center_xy, tangent_xy, progress_ratio = projected
        normal_xy = (-float(tangent_xy[1]), float(tangent_xy[0]))
        signed_offset_m = float(offset_plan.offset_m) * float(offset_plan.side_at(progress_ratio))
        return (
            float(center_xy[0]) + (float(normal_xy[0]) * signed_offset_m),
            float(center_xy[1]) + (float(normal_xy[1]) * signed_offset_m),
        )

    frame = _line_route_frame_xy(route_line_xy)
    if frame is None:
        return None
    route_start_xy, route_end_xy, _unit_xy, normal_xy = frame
    center_xy, progress_ratio = _project_to_route_ratio_xy(midpoint_xy, route_start_xy, route_end_xy)
    signed_offset_m = float(offset_plan.offset_m) * float(offset_plan.side_at(progress_ratio))
    return (
        float(center_xy[0]) + (float(normal_xy[0]) * signed_offset_m),
        float(center_xy[1]) + (float(normal_xy[1]) * signed_offset_m),
    )


def _line_anchor_clearance_m(
    anchor_xy: Tuple[float, float] | None,
    sweep_xy: Sequence[Tuple[float, float]],
) -> float | None:
    if anchor_xy is None or len(sweep_xy) < 2:
        return None
    endpoints = [
        (float(sweep_xy[0][0]), float(sweep_xy[0][1])),
        (float(sweep_xy[-1][0]), float(sweep_xy[-1][1])),
    ]
    return max(_distance_xy(anchor_xy, endpoint_xy) for endpoint_xy in endpoints)


def _route_endpoint_offset_xy(
    route_line_xy: Sequence[Tuple[float, float]],
    *,
    side: float,
    offset_m: float,
    at_end: bool,
) -> Tuple[float, float] | None:
    rows = _route_polyline_rows_xy(route_line_xy)
    if len(rows) < 2:
        return None
    if at_end:
        endpoint_xy = rows[-1]
        tangent_start_xy = rows[-2]
        tangent_end_xy = rows[-1]
    else:
        endpoint_xy = rows[0]
        tangent_start_xy = rows[0]
        tangent_end_xy = rows[1]
    dx = float(tangent_end_xy[0]) - float(tangent_start_xy[0])
    dy = float(tangent_end_xy[1]) - float(tangent_start_xy[1])
    norm = math.hypot(dx, dy)
    if norm <= 1e-6:
        return None
    tangent_xy = (dx / norm, dy / norm)
    normal_xy = (-float(tangent_xy[1]), float(tangent_xy[0]))
    return (
        float(endpoint_xy[0]) + (normal_xy[0] * float(offset_m) * float(side)),
        float(endpoint_xy[1]) + (normal_xy[1] * float(offset_m) * float(side)),
    )


def _turn_sign_toward_xy(
    *,
    origin_xy: Tuple[float, float],
    heading_deg: float,
    target_xy: Tuple[float, float],
) -> int:
    hx, hy = _bearing_unit_xy(float(heading_deg))
    vx = float(target_xy[0]) - float(origin_xy[0])
    vy = float(target_xy[1]) - float(origin_xy[1])
    norm = math.hypot(vx, vy)
    if norm <= 1e-6:
        return 0
    cross = (hx * (vy / norm)) - (hy * (vx / norm))
    if cross > 0.05:
        return 1
    if cross < -0.05:
        return -1
    return 0


def _first_float(*values: Any) -> float | None:
    for value in values:
        parsed = _to_float(value)
        if parsed is not None and math.isfinite(float(parsed)):
            return float(parsed)
    return None


def _first_xy_from_aircraft_row(row: Dict[str, Any] | None) -> Tuple[float, float] | None:
    if not isinstance(row, dict):
        return None
    for key in ("planningPositionXY", "currentPositionXY", "positionXY"):
        value = row.get(key)
        if isinstance(value, (tuple, list)) and len(value) >= 2:
            try:
                return (float(value[0]), float(value[1]))
            except Exception:
                continue
    return None


def _actual_xy_from_aircraft_row(row: Dict[str, Any] | None) -> Tuple[float, float] | None:
    if not isinstance(row, dict):
        return None
    for key in ("currentPositionXY", "positionXY"):
        value = row.get(key)
        if isinstance(value, (tuple, list)) and len(value) >= 2:
            try:
                return (float(value[0]), float(value[1]))
            except Exception:
                continue
    return None


def _dubins_length_m(
    *,
    start_xy: Tuple[float, float],
    start_heading_deg: float,
    end_xy: Tuple[float, float],
    end_heading_deg: float,
    radius_m: float | None,
) -> float | None:
    if _DubinsPose2D is None or _dubins_shortest_path is None:
        return None
    if radius_m is None or not math.isfinite(float(radius_m)) or float(radius_m) <= 1.0:
        return None
    try:
        result = _dubins_shortest_path(
            _DubinsPose2D(float(start_xy[0]), float(start_xy[1]), _bearing_to_yaw_rad(start_heading_deg)),
            _DubinsPose2D(float(end_xy[0]), float(end_xy[1]), _bearing_to_yaw_rad(end_heading_deg)),
            float(radius_m),
            allow_ccc=False,
        )
        return float(result.get("length", 0.0) or 0.0)
    except Exception:
        return None


def _score_line_offset_plan(
    *,
    route_start_xy: Tuple[float, float],
    route_end_xy: Tuple[float, float],
    normal_xy: Tuple[float, float],
    start_side: float,
    end_side: float,
    offset_m: float,
    aircraft_row: Dict[str, Any] | None,
    fallback_origin_xy: Tuple[float, float] | None,
    start_anchor_xy: Tuple[float, float] | None = None,
    end_anchor_xy: Tuple[float, float] | None = None,
) -> _LineOffsetPlan:
    if start_anchor_xy is None:
        start_anchor_xy = (
            float(route_start_xy[0]) + (float(normal_xy[0]) * float(offset_m) * float(start_side)),
            float(route_start_xy[1]) + (float(normal_xy[1]) * float(offset_m) * float(start_side)),
        )
    if end_anchor_xy is None:
        end_anchor_xy = (
            float(route_end_xy[0]) + (float(normal_xy[0]) * float(offset_m) * float(end_side)),
            float(route_end_xy[1]) + (float(normal_xy[1]) * float(offset_m) * float(end_side)),
        )
    route_bearing_deg = _bearing_deg_from_xy(route_start_xy, route_end_xy)
    planned_bearing_deg = _bearing_deg_from_xy(start_anchor_xy, end_anchor_xy)
    origin_xy = _first_xy_from_aircraft_row(aircraft_row) or fallback_origin_xy or route_start_xy
    heading_deg = (
        _first_float(
            (aircraft_row or {}).get("planningHeadingDeg"),
            (aircraft_row or {}).get("headingDeg"),
        )
        if isinstance(aircraft_row, dict)
        else None
    )
    if heading_deg is None:
        heading_deg = _bearing_deg_from_xy(origin_xy, start_anchor_xy)
    turn_sign = 0
    try:
        turn_sign = int((aircraft_row or {}).get("turnSign") or 0)
    except Exception:
        turn_sign = 0
    confidence_raw = (
        _first_float((aircraft_row or {}).get("turnDirectionConfidence"))
        if isinstance(aircraft_row, dict)
        else None
    )
    turn_confidence = (
        max(0.0, min(1.0, float(confidence_raw)))
        if confidence_raw is not None
        else 1.0
    )
    turn_radius_m = _first_float(
        (aircraft_row or {}).get("turnRadiusM") if isinstance(aircraft_row, dict) else None,
        (aircraft_row or {}).get("turnCircleRadiusM") if isinstance(aircraft_row, dict) else None,
        (aircraft_row or {}).get("actualTurnRadiusM") if isinstance(aircraft_row, dict) else None,
        (aircraft_row or {}).get("idealTurnRadiusM") if isinstance(aircraft_row, dict) else None,
        TURN_PREVIEW_RADIUS_M,
    )

    distance_m = _distance_xy(origin_xy, start_anchor_xy)
    initial_turn_sign = _turn_sign_toward_xy(
        origin_xy=origin_xy,
        heading_deg=float(heading_deg),
        target_xy=start_anchor_xy,
    )
    hx, hy = _bearing_unit_xy(float(heading_deg))
    approach_vx = float(start_anchor_xy[0]) - float(origin_xy[0])
    approach_vy = float(start_anchor_xy[1]) - float(origin_xy[1])
    forward_projection_m = (approach_vx * hx) + (approach_vy * hy)
    behind_penalty_m = abs(min(0.0, float(forward_projection_m))) * 2.5
    route_angle_penalty = _angle_delta_deg(planned_bearing_deg, route_bearing_deg) * 6.0
    offset_penalty_m = float(offset_m) * 0.15
    side_switch_penalty_m = 80.0 if float(start_side) != float(end_side) else 0.0

    turn_penalty_m = 0.0
    if (
        turn_sign != 0
        and initial_turn_sign != 0
        and float(turn_confidence) >= float(LINE_TURN_SOFT_CONFIDENCE)
    ):
        if int(turn_sign) == int(initial_turn_sign):
            turn_penalty_m -= float(turn_confidence) * min(
                160.0,
                max(40.0, float(turn_radius_m or TURN_PREVIEW_RADIUS_M) * 0.25),
            )
        else:
            turn_penalty_m += float(turn_confidence) * max(
                350.0,
                float(turn_radius_m or TURN_PREVIEW_RADIUS_M) * 0.9,
            )

    dubins_length_m = _dubins_length_m(
        start_xy=origin_xy,
        start_heading_deg=float(heading_deg),
        end_xy=start_anchor_xy,
        end_heading_deg=float(planned_bearing_deg),
        radius_m=turn_radius_m,
    )
    dubins_penalty_m = 0.0
    if dubins_length_m is not None and dubins_length_m > 0.0:
        dubins_penalty_m = max(0.0, float(dubins_length_m) - float(distance_m)) * 0.35

    score = (
        float(distance_m)
        + float(behind_penalty_m)
        + float(route_angle_penalty)
        + float(offset_penalty_m)
        + float(side_switch_penalty_m)
        + float(turn_penalty_m)
        + float(dubins_penalty_m)
    )
    return _LineOffsetPlan(
        start_side=float(start_side),
        end_side=float(end_side),
        offset_m=float(offset_m),
        score=float(score),
        dubins_length_m=dubins_length_m,
        first_turn_sign=int(initial_turn_sign),
    )


def _choose_line_offset_plan(
    route_line_xy: Sequence[Tuple[float, float]],
    *,
    route_offset_m: float,
    db_sep_m: float,
    aircraft_row: Dict[str, Any] | None,
    fallback_origin_xy: Tuple[float, float] | None,
) -> _LineOffsetPlan | None:
    frame = _line_route_frame_xy(route_line_xy)
    if frame is None:
        return None
    route_start_xy, route_end_xy, _unit_xy, normal_xy = frame
    max_offset_m = max(1.0, float(route_offset_m))
    if db_sep_m > 0.0:
        max_offset_m = min(float(max_offset_m), float(db_sep_m))
    candidates: List[_LineOffsetPlan] = []
    side_pairs = ((1.0, 1.0), (-1.0, -1.0), (1.0, -1.0), (-1.0, 1.0))
    for scale in LINE_OFFSET_CANDIDATE_SCALES:
        candidate_offset_m = max(1.0, float(max_offset_m) * float(scale))
        for start_side, end_side in side_pairs:
            start_anchor_xy = _route_endpoint_offset_xy(
                route_line_xy,
                side=float(start_side),
                offset_m=float(candidate_offset_m),
                at_end=False,
            )
            end_anchor_xy = _route_endpoint_offset_xy(
                route_line_xy,
                side=float(end_side),
                offset_m=float(candidate_offset_m),
                at_end=True,
            )
            candidates.append(
                _score_line_offset_plan(
                    route_start_xy=route_start_xy,
                    route_end_xy=route_end_xy,
                    normal_xy=normal_xy,
                    start_side=float(start_side),
                    end_side=float(end_side),
                    offset_m=float(candidate_offset_m),
                    aircraft_row=aircraft_row,
                    fallback_origin_xy=fallback_origin_xy,
                    start_anchor_xy=start_anchor_xy,
                    end_anchor_xy=end_anchor_xy,
                )
            )
    if not candidates:
        return None
    candidate_key = lambda plan: (
        float(plan.score),
        float(plan.offset_m),
        0 if float(plan.start_side) == float(plan.end_side) else 1,
    )
    best_unlocked_candidate = min(candidates, key=candidate_key)
    confidence_raw = (
        _first_float((aircraft_row or {}).get("turnDirectionConfidence"))
        if isinstance(aircraft_row, dict)
        else None
    )
    required_turn_sign = 0
    try:
        required_turn_sign = int((aircraft_row or {}).get("turnSign") or 0)
    except Exception:
        required_turn_sign = 0
    if (
        confidence_raw is not None
        and float(confidence_raw) >= float(LINE_TURN_HARD_CONFIDENCE)
        and required_turn_sign != 0
    ):
        locked_candidates = [
            candidate
            for candidate in candidates
            if int(candidate.first_turn_sign) == int(required_turn_sign)
        ]
        if locked_candidates:
            best_locked_candidate = min(locked_candidates, key=candidate_key)
            turn_radius_m = _first_float(
                (aircraft_row or {}).get("turnRadiusM"),
                (aircraft_row or {}).get("turnCircleRadiusM"),
                (aircraft_row or {}).get("actualTurnRadiusM"),
                (aircraft_row or {}).get("idealTurnRadiusM"),
                TURN_PREVIEW_RADIUS_M,
            )
            # A confident turn sign is useful only while it produces a sane
            # ingress.  Near a Line-piece boundary, the strict sign filter can
            # otherwise select a far/behind anchor merely because it is the
            # sole candidate with that sign (the UAV6 reverse-entry case).
            # One half-circle of the observed radius is a generous allowance
            # for continuing the turn; anything beyond it is a detour, so fall
            # back to the already turn-penalized best candidate.
            turn_lock_detour_budget_m = max(
                600.0,
                min(1800.0, math.pi * max(1.0, float(turn_radius_m or TURN_PREVIEW_RADIUS_M))),
            )
            if float(best_locked_candidate.score) <= (
                float(best_unlocked_candidate.score) + float(turn_lock_detour_budget_m)
            ):
                candidates = locked_candidates
    return min(candidates, key=candidate_key)


def _clamp_line_offset_plan_to_sweep(
    plan: _LineOffsetPlan | None,
    *,
    sweep_xy: Sequence[Tuple[float, float]],
    route_line_xy: Sequence[Tuple[float, float]],
    db_sep_m: float,
) -> _LineOffsetPlan | None:
    if plan is None or db_sep_m <= 0.0 or len(sweep_xy) < 2:
        return plan
    anchor_xy = _line_anchor_from_route_plan_xy(
        sweep_xy,
        route_line_xy=route_line_xy,
        offset_plan=plan,
    )
    clearance_m = _line_anchor_clearance_m(anchor_xy, sweep_xy)
    if clearance_m is None or clearance_m <= float(db_sep_m) + 1e-6:
        return _LineOffsetPlan(
            start_side=plan.start_side,
            end_side=plan.end_side,
            offset_m=plan.offset_m,
            score=plan.score,
            dubins_length_m=plan.dubins_length_m,
            first_turn_sign=plan.first_turn_sign,
            clearance_m=clearance_m,
        )

    lo = 0.0
    hi = float(plan.offset_m)
    best_clearance_m = clearance_m
    for _ in range(32):
        mid = (lo + hi) * 0.5
        candidate = _LineOffsetPlan(
            start_side=plan.start_side,
            end_side=plan.end_side,
            offset_m=float(mid),
            score=plan.score,
            dubins_length_m=plan.dubins_length_m,
            first_turn_sign=plan.first_turn_sign,
        )
        candidate_anchor = _line_anchor_from_route_plan_xy(
            sweep_xy,
            route_line_xy=route_line_xy,
            offset_plan=candidate,
        )
        candidate_clearance_m = _line_anchor_clearance_m(candidate_anchor, sweep_xy)
        if candidate_clearance_m is None:
            hi = mid
            continue
        best_clearance_m = float(candidate_clearance_m)
        if candidate_clearance_m <= float(db_sep_m) + 1e-6:
            lo = mid
        else:
            hi = mid
    return _LineOffsetPlan(
        start_side=plan.start_side,
        end_side=plan.end_side,
        offset_m=max(1.0, float(lo)),
        score=plan.score,
        dubins_length_m=plan.dubins_length_m,
        first_turn_sign=plan.first_turn_sign,
        clearance_m=float(best_clearance_m),
    )


def _ensure(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def _to_float(value: Any) -> float | None:
    try:
        if value is None:
            return None
        return float(value)
    except Exception:
        return None


def _to_int(value: Any) -> int | None:
    try:
        if value is None:
            return None
        return int(value)
    except Exception:
        return None


def _clamp_line_fov_deg(value: Any, default: float | None = None) -> float:
    fov_deg = _to_float(value)
    if fov_deg is None or fov_deg <= 0.0:
        fov_deg = _to_float(default)
    if fov_deg is None or fov_deg <= 0.0:
        fov_deg = float(MIN_LINE_FOV_DEG)
    return max(float(MIN_LINE_FOV_DEG), float(fov_deg))


def _normalize_coordinate(payload: object | None) -> Dict[str, float] | None:
    if not isinstance(payload, dict):
        return None
    lat = _to_float(payload.get("latitude"))
    lon = _to_float(payload.get("longitude"))
    if lat is None or lon is None:
        return None
    out: Dict[str, float] = {"latitude": float(lat), "longitude": float(lon)}
    alt = _to_float(payload.get("altitude"))
    if alt is not None:
        out["altitude"] = float(alt)
    return out


def _normalize_coord_list(payload: object | None) -> List[Dict[str, float]]:
    coords = payload if isinstance(payload, list) else []
    out: List[Dict[str, float]] = []
    for item in coords:
        coord = _normalize_coordinate(item)
        if coord is not None:
            out.append(coord)
    return out


def _choose_width_fallback(detail: Dict[str, Any], mission: Dict[str, Any]) -> float:
    for payload in (detail, mission):
        for key in ("lineWidthM", "width"):
            width = _to_float(payload.get(key))
            if width is not None and width > 0.0:
                return float(width)
    return 1.0


def _normalize_line_specs(
    target_mission: Dict[str, Any],
) -> tuple[Dict[str, Any], List[Dict[str, Any]]]:
    mission = copy.deepcopy(target_mission if isinstance(target_mission, dict) else {})
    detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else None
    if not isinstance(detail, dict):
        detail = mission
        mission = {"missionDetail": copy.deepcopy(detail)}

    width_fallback = _choose_width_fallback(detail, mission)
    line_specs: List[Dict[str, Any]] = []

    for line in detail.get("lineList") or []:
        if not isinstance(line, dict):
            continue
        coords = _normalize_coord_list(line.get("coordinateList"))
        if len(coords) < 2:
            continue
        width_m = _to_float(line.get("width"))
        if width_m is None or width_m <= 0.0:
            width_m = width_fallback
        line_specs.append({"width": max(0, min(50000, int(round(float(width_m))))), "coordinateList": coords})

    coord_list = _normalize_coord_list(detail.get("coordinateList"))
    if not line_specs and len(coord_list) >= 2:
        line_specs.append({"width": max(0, min(50000, int(round(float(width_fallback))))), "coordinateList": coord_list})

    _ensure(bool(line_specs), "next-collab line planner requires valid line geometry.")

    detail = dict(detail)
    detail["lineList"] = copy.deepcopy(line_specs)
    detail["coordinateList"] = copy.deepcopy(coord_list) if len(coord_list) >= 2 else None
    mission["missionDetail"] = detail
    if _to_float(mission.get("inputMissionID")) is None:
        mission["inputMissionID"] = 1
    try:
        mission_type = int(mission.get("inputMissionType"))
    except Exception:
        mission_type = 1
    if mission_type not in (1, 7):
        mission["inputMissionType"] = 1
    return mission, line_specs


def _resolve_source_line_width_m(
    mission: Dict[str, Any],
    detail: Dict[str, Any],
    line_specs: Sequence[Dict[str, Any]],
) -> float:
    for payload in (detail, mission):
        width_m = _to_float(payload.get("sourceLineWidthM"))
        if width_m is not None and width_m > 0.0:
            return float(width_m)
    for spec in line_specs:
        width_m = _to_float(spec.get("width"))
        if width_m is not None and width_m > 0.0:
            return float(width_m)
    return float(_choose_width_fallback(detail, mission))


def _resolve_source_coordinate_list(
    detail: Dict[str, Any],
    line_specs: Sequence[Dict[str, Any]],
) -> List[Dict[str, float]]:
    source_coords = _normalize_coord_list(detail.get("sourceCoordinateList"))
    if len(source_coords) >= 2:
        return source_coords
    for spec in line_specs:
        coords = _normalize_coord_list(spec.get("coordinateList"))
        if len(coords) >= 2:
            return coords
    fallback_coords = _normalize_coord_list(detail.get("coordinateList"))
    if len(fallback_coords) >= 2:
        return fallback_coords
    return []


def _resolve_line_deployment_coordinate_list(
    detail: Dict[str, Any],
) -> List[Dict[str, float]]:
    """Return the persisted first-plan LINE execution direction, if present."""

    coords = _normalize_coord_list(detail.get("lineDeploymentCoordinateList"))
    return coords if len(coords) >= 2 else []


def _mission_center_xy(line_specs: List[Dict[str, Any]]) -> tuple[float, float] | None:
    points_xy: List[Tuple[float, float]] = []
    for spec in line_specs:
        points_xy.extend(coords_to_xy(spec.get("coordinateList") or []))
    if not points_xy:
        return None
    return (
        sum(float(x) for x, _ in points_xy) / float(len(points_xy)),
        sum(float(y) for _, y in points_xy) / float(len(points_xy)),
    )


def _normalize_aircraft_rows(
    aircraft_entries: List[Dict[str, Any]],
    *,
    mission_center_xy: tuple[float, float] | None,
) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for entry in aircraft_entries:
        if not isinstance(entry, dict):
            continue
        try:
            aircraft_id = int(entry.get("aircraftID"))
        except Exception:
            continue
        coord = _normalize_coordinate(entry.get("coordinate"))
        position_xy = coord_to_xy(coord) if coord is not None else None
        if aircraft_id <= 0 or coord is None or position_xy is None:
            continue
        current_coord = _normalize_coordinate(entry.get("currentCoordinate"))
        current_position_xy = coord_to_xy(current_coord) if current_coord is not None else None
        reported_heading_deg = _to_float(entry.get("headingDeg"))
        if reported_heading_deg is None:
            reported_heading_deg = _to_float(entry.get("heading"))
        trend_heading_deg = _to_float(entry.get("lineTrendHeadingDeg"))
        predicted_heading_deg = _to_float(entry.get("linePredictedHeadingDeg"))
        confidence_value = _to_float(entry.get("lineTurnDirectionConfidence"))
        turn_confidence = (
            max(0.0, min(1.0, float(confidence_value)))
            if confidence_value is not None
            else None
        )

        def _planner_bearing(value: float | None) -> float | None:
            if value is None:
                return None
            try:
                return float(monitor_heading_to_planner_bearing_deg(value))
            except Exception:
                return None

        reported_planner_heading = _planner_bearing(reported_heading_deg)
        trend_planner_heading = _planner_bearing(trend_heading_deg)
        predicted_planner_heading = _planner_bearing(predicted_heading_deg)
        planner_heading = reported_planner_heading
        heading_source = "reported"
        if (
            turn_confidence is not None
            and turn_confidence >= float(LINE_TURN_SOFT_CONFIDENCE)
            and trend_planner_heading is not None
        ):
            planner_heading = trend_planner_heading
            heading_source = "recent-trend"
        try:
            raw_turn_sign = int(entry.get("turnSign") or 0)
        except Exception:
            raw_turn_sign = 0
        try:
            trend_turn_sign = int(entry.get("lineTrendTurnSign") or 0)
        except Exception:
            trend_turn_sign = 0
        turn_sign = int(raw_turn_sign)
        if turn_confidence is not None:
            if (
                turn_confidence >= float(LINE_TURN_SOFT_CONFIDENCE)
                and trend_turn_sign != 0
            ):
                turn_sign = 1 if trend_turn_sign > 0 else -1
            else:
                # A confidence-bearing recent estimate owns the Line decision.
                # Its zero sign means "no reliable live turn", so do not revive
                # the stale long-window sign that caused terminal reversals.
                turn_sign = 0
        if planner_heading is None and mission_center_xy is not None:
            planner_heading = _bearing_deg_from_xy(position_xy, mission_center_xy)
            heading_source = "mission-center-fallback"
        if planner_heading is None:
            planner_heading = 0.0
            heading_source = "zero-fallback"
        planning_heading = float(planner_heading)
        if (
            turn_confidence is not None
            and turn_confidence >= float(LINE_TURN_HARD_CONFIDENCE)
            and predicted_planner_heading is not None
        ):
            planning_heading = float(predicted_planner_heading)

        line_predicted_coord = _normalize_coordinate(entry.get("linePredictedEntryCoordinate"))
        line_predicted_position_xy = (
            coord_to_xy(line_predicted_coord) if line_predicted_coord is not None else None
        )
        planning_position_xy = current_position_xy or position_xy
        if (
            turn_confidence is not None
            and turn_confidence >= float(LINE_TURN_HARD_CONFIDENCE)
            and line_predicted_position_xy is not None
        ):
            planning_position_xy = line_predicted_position_xy

        speed_mps = _first_float(entry.get("speedMps"), entry.get("speed_mps"))
        trend_turn_rate_dps = _first_float(entry.get("lineTrendTurnRateDps"))
        reported_turn_radius_m = _first_float(
            entry.get("turnRadiusM"),
            entry.get("turnCircleRadiusM"),
            entry.get("actualTurnRadiusM"),
            entry.get("idealTurnRadiusM"),
        )
        turn_radius_m = reported_turn_radius_m
        turn_radius_source = "reported"
        if (
            turn_confidence is not None
            and turn_confidence >= float(LINE_TURN_SOFT_CONFIDENCE)
            and trend_turn_sign != 0
            and speed_mps is not None
            and trend_turn_rate_dps is not None
            and abs(float(trend_turn_rate_dps)) >= 0.2
        ):
            trend_radius_m = float(speed_mps) / math.radians(
                abs(float(trend_turn_rate_dps))
            )
            # The recent trend is useful for recognizing a genuinely tighter
            # live turn, but a shallow/noisy rate must never expand a valid
            # aircraft capability radius into a multi-kilometre circle.  That
            # made otherwise valid branch entry points geometrically
            # unreachable.  The reported/ideal radius remains the upper bound.
            if reported_turn_radius_m is not None and reported_turn_radius_m > 1.0:
                turn_radius_m = min(
                    float(reported_turn_radius_m),
                    float(trend_radius_m),
                )
                turn_radius_source = "trend_capped_by_reported"
            else:
                turn_radius_m = float(trend_radius_m)
                turn_radius_source = "trend"
        elif trend_turn_rate_dps is not None and trend_turn_sign == 0:
            turn_radius_source = "reported_unreliable_trend_direction"
        rows.append(
            {
                "aircraftID": int(aircraft_id),
                "coordinate": {
                    "latitude": float(coord["latitude"]),
                    "longitude": float(coord["longitude"]),
                    "altitude": float(coord.get("altitude", 0.0) or 0.0),
                },
                "positionXY": (float(position_xy[0]), float(position_xy[1])),
                "currentCoordinate": dict(current_coord) if current_coord is not None else None,
                "currentPositionXY": (
                    (float(current_position_xy[0]), float(current_position_xy[1]))
                    if current_position_xy is not None
                    else None
                ),
                "planningPositionXY": (
                    (float(planning_position_xy[0]), float(planning_position_xy[1]))
                    if planning_position_xy is not None
                    else None
                ),
                "headingDeg": float(planner_heading),
                "planningHeadingDeg": float(planning_heading),
                "reportedHeadingDeg": reported_planner_heading,
                "trendHeadingDeg": trend_planner_heading,
                "predictedHeadingDeg": predicted_planner_heading,
                "headingSource": str(heading_source),
                "speedMps": speed_mps,
                "turnRateDps": (
                    float(trend_turn_rate_dps)
                    if turn_confidence is not None
                    and turn_confidence >= float(LINE_TURN_SOFT_CONFIDENCE)
                    and trend_turn_rate_dps is not None
                    else _first_float(entry.get("turnRateDps"), entry.get("turn_rate_dps"))
                ),
                "turnSign": int(turn_sign),
                "reportedTurnSign": int(raw_turn_sign),
                "trendTurnSign": int(trend_turn_sign),
                "turnDirectionConfidence": turn_confidence,
                "turnTrendSampleCount": _to_int(entry.get("lineTrendSampleCount")),
                "turnTrendSource": str(entry.get("lineTrendSource") or ""),
                "predictionLeadS": _first_float(entry.get("linePredictionLeadS")),
                "turnDataAgeS": _first_float(entry.get("lineTurnDataAgeS")),
                "turnRadiusM": turn_radius_m,
                "turnRadiusSource": str(turn_radius_source),
                "idealTurnRadiusM": _first_float(entry.get("idealTurnRadiusM")),
                "actualTurnRadiusM": _first_float(entry.get("actualTurnRadiusM")),
                "turnCircleRadiusM": _first_float(entry.get("turnCircleRadiusM")),
            }
        )
    rows.sort(key=lambda item: int(item["aircraftID"]))
    return rows


def _piece_centerline_xy(piece: SplitPiece) -> List[Tuple[float, float]]:
    data = piece.data if isinstance(piece.data, dict) else {}
    for key in ("Centerline", "coordinateList", "rawCoordinateList"):
        coords = coords_to_xy(data.get(key) or [])
        if len(coords) >= 2:
            return [(float(x), float(y)) for x, y in coords]
    return []


def _piece_target_xy(piece: SplitPiece) -> Tuple[float, float] | None:
    coords_xy = _piece_centerline_xy(piece)
    if len(coords_xy) >= 3:
        try:
            poly = Polygon(coords_xy)
            if not poly.is_empty:
                return float(poly.centroid.x), float(poly.centroid.y)
        except Exception:
            pass
    if len(coords_xy) >= 2:
        try:
            line = LineString(coords_xy)
            mid = line.interpolate(0.5, normalized=True)
            return float(mid.x), float(mid.y)
        except Exception:
            return coords_xy[len(coords_xy) // 2]
    return None


def _piece_width_m(piece: SplitPiece) -> float:
    data = piece.data if isinstance(piece.data, dict) else {}
    width_m = _to_float(data.get("width"))
    if width_m is None or width_m <= 0.0:
        return 1.0
    return float(width_m)


def _midpoint_xy(points_xy: Sequence[Tuple[float, float]]) -> Tuple[float, float] | None:
    rows = [(float(x), float(y)) for x, y in points_xy]
    if not rows:
        return None
    return (
        sum(float(x) for x, _ in rows) / float(len(rows)),
        sum(float(y) for _, y in rows) / float(len(rows)),
    )


def _marker_xy(marker_rows: Any, kind: str) -> Tuple[float, float] | None:
    if not isinstance(marker_rows, list):
        return None
    for row in marker_rows:
        if not isinstance(row, dict):
            continue
        if str(row.get("kind") or "") != str(kind):
            continue
        point_xy = row.get("xy")
        if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2:
            return float(point_xy[0]), float(point_xy[1])
    return None


def _orient_polyline_xy(
    points_xy: Sequence[Tuple[float, float]],
    *,
    reference_xy: Tuple[float, float] | None,
) -> List[Tuple[float, float]]:
    rows = [(float(x), float(y)) for x, y in points_xy]
    if len(rows) < 2 or reference_xy is None:
        return rows
    start_dist = _distance_xy(reference_xy, rows[0])
    end_dist = _distance_xy(reference_xy, rows[-1])
    if end_dist + 1e-6 < start_dist:
        rows.reverse()
    return rows


def _orient_polyline_xy_to_direction(
    points_xy: Sequence[Tuple[float, float]],
    *,
    direction_xy: Sequence[Tuple[float, float]] | None,
) -> List[Tuple[float, float]]:
    """Orient a LINE fragment along a previously selected execution direction.

    Projection onto the original directed LINE handles curved and trimmed
    fragments.  The endpoint-vector fallback covers synthetic/fallback pieces
    that are only approximately on that reference.
    """

    rows = [(float(x), float(y)) for x, y in points_xy]
    reference = [(float(x), float(y)) for x, y in (direction_xy or [])]
    if len(rows) < 2 or len(reference) < 2:
        return rows

    try:
        reference_line = LineString(reference)
        if not reference_line.is_empty and float(reference_line.length) > 1e-6:
            start_progress = float(reference_line.project(Point(rows[0])))
            end_progress = float(reference_line.project(Point(rows[-1])))
            if abs(end_progress - start_progress) > 1e-3:
                if end_progress < start_progress:
                    rows.reverse()
                return rows
    except Exception:
        pass

    ref_dx = float(reference[-1][0]) - float(reference[0][0])
    ref_dy = float(reference[-1][1]) - float(reference[0][1])
    row_dx = float(rows[-1][0]) - float(rows[0][0])
    row_dy = float(rows[-1][1]) - float(rows[0][1])
    if (ref_dx * row_dx) + (ref_dy * row_dy) < 0.0:
        rows.reverse()
    return rows


def _derive_initial_line_deployment_reference(
    source_coordinate_list: Sequence[Dict[str, Any]],
    piece: SplitPiece,
    path_row: Dict[str, Any],
) -> tuple[List[Tuple[float, float]], List[Dict[str, float]]]:
    """Freeze the direction actually selected for the first generated path."""

    centerline_xy = _piece_centerline_xy(piece)
    marker_rows = path_row.get("markerRows") if isinstance(path_row.get("markerRows"), list) else []
    reference_raw = (
        path_row.get("waypointStartXY")
        or path_row.get("tangentXY")
        or path_row.get("targetFaceXY")
    )
    reference_xy = (
        (float(reference_raw[0]), float(reference_raw[1]))
        if isinstance(reference_raw, (tuple, list)) and len(reference_raw) >= 2
        else _marker_xy(marker_rows, "tangent")
    )
    selected_piece_direction = _orient_polyline_xy(
        centerline_xy,
        reference_xy=reference_xy,
    )

    source_coords = [dict(coord) for coord in source_coordinate_list if isinstance(coord, dict)]
    source_xy = coords_to_xy(source_coords)
    if len(source_xy) >= 2 and len(selected_piece_direction) >= 2:
        oriented_source_xy = _orient_polyline_xy_to_direction(
            source_xy,
            direction_xy=selected_piece_direction,
        )
        if _distance_xy(oriented_source_xy[0], source_xy[-1]) + 1e-6 < _distance_xy(
            oriented_source_xy[0], source_xy[0]
        ):
            source_coords.reverse()
        return oriented_source_xy, source_coords
    if len(source_xy) >= 2:
        return list(source_xy), source_coords
    return selected_piece_direction, source_coords


def _type2_branch_deployment_reference(
    piece: SplitPiece,
    line_specs: Sequence[Dict[str, Any]],
    mode_context: Dict[str, Any],
) -> tuple[List[Tuple[float, float]], List[Dict[str, float]]]:
    """Return the declared direction of this Type-2 branch.

    A Type-2 mission contains independent LINE branches.  Their coordinate
    order is operational (for example, boundary -> target on ingress and
    target -> boundary on egress), so it must not be replaced by the nearest
    endpoint selected for another branch.
    """

    if _to_int(mode_context.get("package_type")) != 2 or len(line_specs) < 2:
        return [], []
    data = piece.data if isinstance(piece.data, dict) else {}
    branch_index = _to_int(data.get("branchIndex"))
    if branch_index is None or not (0 <= int(branch_index) < len(line_specs)):
        return [], []
    spec = line_specs[int(branch_index)]
    coords = _normalize_coord_list(spec.get("coordinateList"))
    direction_xy = coords_to_xy(coords)
    if len(direction_xy) < 2:
        return [], []
    return list(direction_xy), [dict(coord) for coord in coords]


def _build_line_sweep_items(
    sweep_lines_xy: Sequence[Sequence[Tuple[float, float]]],
    *,
    reference_xy: Tuple[float, float] | None,
    offset_m: float,
    route_line_xy: Sequence[Tuple[float, float]] | None = None,
    offset_plan: _LineOffsetPlan | None = None,
    direction_locked: bool = False,
) -> List[Dict[str, Any]]:
    items: List[Dict[str, Any]] = []
    for sweep_xy in sweep_lines_xy:
        line_xy = [(float(x), float(y)) for x, y in sweep_xy]
        if len(line_xy) < 2:
            continue
        anchor_xy = _line_anchor_xy_from_sweep(
            line_xy,
            offset_m=float(offset_m),
            route_line_xy=route_line_xy,
            reference_xy=reference_xy,
            offset_plan=offset_plan,
        )
        if anchor_xy is None:
            continue
        items.append(
            {
                "anchorXY": (float(anchor_xy[0]), float(anchor_xy[1])),
                "sweepXY": line_xy,
            }
        )
    if len(items) >= 2 and reference_xy is not None and not direction_locked:
        if _distance_xy(reference_xy, items[-1]["anchorXY"]) + 1e-6 < _distance_xy(reference_xy, items[0]["anchorXY"]):
            items.reverse()
    prev_target_xy = reference_xy
    for item in items:
        line_xy = [(float(x), float(y)) for x, y in item.get("sweepXY") or []]
        if len(line_xy) < 2:
            continue
        if prev_target_xy is not None:
            start_dist = _distance_xy(prev_target_xy, line_xy[0])
            end_dist = _distance_xy(prev_target_xy, line_xy[-1])
            if end_dist + 1e-6 < start_dist:
                line_xy.reverse()
        item["sweepXY"] = line_xy
        prev_target_xy = line_xy[-1]
    return items


def _polyline_length_xy(points_xy: Sequence[Tuple[float, float]]) -> float:
    rows = [(float(x), float(y)) for x, y in points_xy]
    if len(rows) < 2:
        return 0.0
    total = 0.0
    for idx in range(1, len(rows)):
        total += _distance_xy(rows[idx - 1], rows[idx])
    return float(total)


def _resample_polyline_xy(
    points_xy: Sequence[Tuple[float, float]],
    spacing_m: float,
) -> List[Tuple[float, float]]:
    rows = [(float(x), float(y)) for x, y in points_xy]
    if len(rows) < 2:
        return rows
    spacing_m = max(float(spacing_m), 1.0)
    total_len_m = _polyline_length_xy(rows)
    if total_len_m <= spacing_m + 1e-6:
        return [rows[0], rows[-1]]

    # Route sample distances at `spacing_m`, with the sub-spacing trailing
    # remainder merged into the previous full segment instead of emitted as a
    # stubby last waypoint (2000m grouping + short-tail absorb, matching the
    # confirmed area-sweep tail rule: 1-2 full + short 2-3 -> merge to 1-3).
    n_full = int(total_len_m // float(spacing_m))
    interior = [float(spacing_m) * float(idx) for idx in range(1, n_full + 1)]
    remainder_m = total_len_m - float(n_full) * float(spacing_m)
    if remainder_m > 1e-6 and interior:
        interior.pop()
    targets: List[float] = [0.0] + interior + [float(total_len_m)]

    out: List[Tuple[float, float]] = []
    seg_start_idx = 0
    walked_m = 0.0
    for target_m in targets:
        while seg_start_idx < len(rows) - 1:
            seg_len_m = _distance_xy(rows[seg_start_idx], rows[seg_start_idx + 1])
            if walked_m + seg_len_m + 1e-6 >= target_m:
                if seg_len_m <= 1e-6:
                    out.append(rows[seg_start_idx])
                else:
                    ratio = max(0.0, min(1.0, (target_m - walked_m) / seg_len_m))
                    x = float(rows[seg_start_idx][0]) + (
                        (float(rows[seg_start_idx + 1][0]) - float(rows[seg_start_idx][0])) * ratio
                    )
                    y = float(rows[seg_start_idx][1]) + (
                        (float(rows[seg_start_idx + 1][1]) - float(rows[seg_start_idx][1])) * ratio
                    )
                    out.append((x, y))
                break
            walked_m += seg_len_m
            seg_start_idx += 1
        else:
            out.append(rows[-1])
    deduped: List[Tuple[float, float]] = []
    for point_xy in out:
        if deduped and _distance_xy(deduped[-1], point_xy) <= 1.0:
            continue
        deduped.append(point_xy)
    if len(deduped) == 1:
        deduped.append(rows[-1])
    return deduped


def _select_route_sweep_indices(
    route_points_xy: Sequence[Tuple[float, float]],
    sweep_midpoints_xy: Sequence[Tuple[float, float]],
) -> List[int]:
    if not route_points_xy or not sweep_midpoints_xy:
        return []
    total_pts = len(route_points_xy)
    total_sweeps = len(sweep_midpoints_xy)
    if total_sweeps <= total_pts:
        return list(range(total_sweeps))

    selected: List[int] = []
    start_idx = 0
    for pos, point_xy in enumerate(route_points_xy):
        remaining = total_pts - pos - 1
        max_idx = total_sweeps - 1 - remaining
        if max_idx < start_idx:
            max_idx = start_idx
        best_idx = start_idx
        best_dist = None
        for idx in range(start_idx, max_idx + 1):
            dist = _distance_xy(point_xy, sweep_midpoints_xy[idx])
            if best_dist is None or dist < best_dist:
                best_dist = dist
                best_idx = idx
        selected.append(int(best_idx))
        start_idx = int(best_idx) + 1
    if selected:
        selected[-1] = int(total_sweeps - 1)
    return selected


def _line_bearing_deg(points_xy: Sequence[Tuple[float, float]]) -> float:
    if len(points_xy) < 2:
        return 0.0
    return _bearing_deg_from_xy(points_xy[0], points_xy[-1])


def _line_anchor_xy_from_sweep(
    sweep_xy: Sequence[Tuple[float, float]],
    *,
    offset_m: float,
    route_line_xy: Sequence[Tuple[float, float]] | None = None,
    reference_xy: Tuple[float, float] | None = None,
    offset_plan: _LineOffsetPlan | None = None,
) -> Tuple[float, float] | None:
    if len(sweep_xy) < 2:
        return None
    if offset_plan is not None and route_line_xy is not None and len(route_line_xy) >= 2:
        anchor_xy = _line_anchor_from_route_plan_xy(
            sweep_xy,
            route_line_xy=route_line_xy,
            offset_plan=offset_plan,
        )
        if anchor_xy is not None:
            return anchor_xy
    if route_line_xy is not None and len(route_line_xy) >= 2:
        midpoint_xy = _midpoint_xy(sweep_xy)
        if midpoint_xy is not None:
            projected = _project_to_route_polyline_xy(midpoint_xy, route_line_xy)
            if projected is not None:
                center_xy, tangent_xy, _progress_ratio = projected
                normal_xy = (-float(tangent_xy[1]), float(tangent_xy[0]))
                offset_abs_m = max(float(offset_m), 0.0)
                candidates = [
                    (
                        float(center_xy[0]) + (normal_xy[0] * offset_abs_m),
                        float(center_xy[1]) + (normal_xy[1] * offset_abs_m),
                    ),
                    (
                        float(center_xy[0]) - (normal_xy[0] * offset_abs_m),
                        float(center_xy[1]) - (normal_xy[1] * offset_abs_m),
                    ),
                ]
                if reference_xy is not None:
                    return min(candidates, key=lambda anchor: _distance_xy(reference_xy, anchor))
                return candidates[0]
    (x1, y1), (x2, y2) = sweep_xy[0], sweep_xy[-1]
    mid_x = (float(x1) + float(x2)) * 0.5
    mid_y = (float(y1) + float(y2)) * 0.5
    dx = float(x2) - float(x1)
    dy = float(y2) - float(y1)
    norm = math.hypot(dx, dy)
    if norm <= 1e-6:
        return mid_x, mid_y
    ux = dy / norm
    uy = -dx / norm
    return (
        mid_x + (ux * float(max(offset_m, 0.0))),
        mid_y + (uy * float(max(offset_m, 0.0))),
    )


def _linestring_segments_xy(
    planner: _HeadlessLinePlanner,
    geom: Any,
) -> List[LineString]:
    segment_fn = getattr(planner, "_linestring_segments_xy", None)
    if callable(segment_fn):
        try:
            segments = segment_fn(geom)
            return [
                line
                for line in segments
                if isinstance(line, LineString) and float(line.length) > 1e-6
            ]
        except Exception:
            pass
    if geom is None or getattr(geom, "is_empty", True):
        return []
    if isinstance(geom, LineString):
        return [geom] if float(geom.length) > 1e-6 else []
    if str(getattr(geom, "geom_type", "")) not in {"MultiLineString", "GeometryCollection"}:
        return []
    return [
        line
        for line in getattr(geom, "geoms", [])
        if isinstance(line, LineString) and float(line.length) > 1e-6
    ]


def _nearest_linestring_to_xy(
    planner: _HeadlessLinePlanner,
    geom: Any,
    reference_xy: Tuple[float, float] | None,
) -> Optional[LineString]:
    lines = _linestring_segments_xy(planner, geom)
    if not lines:
        return None
    if reference_xy is None:
        return max(lines, key=lambda line: float(line.length))
    ref_point = Point(float(reference_xy[0]), float(reference_xy[1]))
    return min(
        lines,
        key=lambda line: (
            float(line.distance(ref_point)),
            -float(line.length),
        ),
    )


def _point_in_polygon_xy(point_xy: Tuple[float, float], polygon_xy: Sequence[Tuple[float, float]]) -> bool:
    px, py = float(point_xy[0]), float(point_xy[1])
    rows = [(float(x), float(y)) for x, y in polygon_xy]
    if len(rows) < 3:
        return False
    inside = False
    prev_x, prev_y = rows[-1]
    for curr_x, curr_y in rows:
        if (
            min(prev_y, curr_y) - 1e-9 <= py <= max(prev_y, curr_y) + 1e-9
            and abs(curr_y - prev_y) > 1e-12
        ):
            x_at_y = prev_x + ((py - prev_y) * (curr_x - prev_x) / (curr_y - prev_y))
            if abs(x_at_y - px) <= 1e-7:
                return True
        if (
            min(prev_x, curr_x) - 1e-9 <= px <= max(prev_x, curr_x) + 1e-9
            and min(prev_y, curr_y) - 1e-9 <= py <= max(prev_y, curr_y) + 1e-9
        ):
            cross = ((curr_x - prev_x) * (py - prev_y)) - ((curr_y - prev_y) * (px - prev_x))
            if abs(cross) <= 1e-7:
                return True
        intersects = ((curr_y > py) != (prev_y > py))
        if intersects:
            x_at_y = prev_x + ((py - prev_y) * (curr_x - prev_x) / (curr_y - prev_y))
            if px < x_at_y:
                inside = not inside
        prev_x, prev_y = curr_x, curr_y
    return bool(inside)


def _dedupe_sorted_values(values: Sequence[float], *, eps: float = 1e-7) -> List[float]:
    out: List[float] = []
    for value in sorted(float(item) for item in values if math.isfinite(float(item))):
        if out and abs(float(value) - float(out[-1])) <= eps:
            continue
        out.append(float(value))
    return out


def _scanline_polygon_segment_xy(
    polygon_xy: Sequence[Tuple[float, float]],
    origin_xy: Tuple[float, float],
    direction_xy: Tuple[float, float],
    *,
    reference_xy: Tuple[float, float] | None = None,
) -> List[Tuple[float, float]] | None:
    rows = [(float(x), float(y)) for x, y in polygon_xy]
    if len(rows) < 3:
        return None
    dir_unit = _unit_xy(float(direction_xy[0]), float(direction_xy[1]))
    if dir_unit is None:
        return None
    dx, dy = dir_unit
    ox, oy = float(origin_xy[0]), float(origin_xy[1])

    def _signed_distance(point: Tuple[float, float]) -> float:
        return ((float(point[0]) - ox) * dy) - ((float(point[1]) - oy) * dx)

    def _project_t(point: Tuple[float, float]) -> float:
        return ((float(point[0]) - ox) * dx) + ((float(point[1]) - oy) * dy)

    t_values: List[float] = []
    prev = rows[-1]
    prev_d = _signed_distance(prev)
    for curr in rows:
        curr_d = _signed_distance(curr)
        if abs(prev_d) <= 1e-7:
            t_values.append(_project_t(prev))
        if abs(curr_d) <= 1e-7:
            t_values.append(_project_t(curr))
        if (prev_d < -1e-7 and curr_d > 1e-7) or (prev_d > 1e-7 and curr_d < -1e-7):
            ratio = prev_d / (prev_d - curr_d)
            ix = float(prev[0]) + ((float(curr[0]) - float(prev[0])) * ratio)
            iy = float(prev[1]) + ((float(curr[1]) - float(prev[1])) * ratio)
            t_values.append(((ix - ox) * dx) + ((iy - oy) * dy))
        prev = curr
        prev_d = curr_d

    t_rows = _dedupe_sorted_values(t_values)
    if len(t_rows) < 2:
        return None

    intervals: List[Tuple[float, float]] = []
    for left_t, right_t in zip(t_rows, t_rows[1:]):
        if float(right_t) - float(left_t) <= 1e-6:
            continue
        mid_t = (float(left_t) + float(right_t)) * 0.5
        mid_xy = (ox + (dx * mid_t), oy + (dy * mid_t))
        if _point_in_polygon_xy(mid_xy, rows):
            intervals.append((float(left_t), float(right_t)))
    if not intervals:
        return None

    ref_t = _project_t(reference_xy) if reference_xy is not None else None
    if ref_t is None:
        best_interval = max(intervals, key=lambda item: float(item[1]) - float(item[0]))
    else:
        best_interval = min(
            intervals,
            key=lambda item: (
                0.0
                if float(item[0]) - 1e-6 <= float(ref_t) <= float(item[1]) + 1e-6
                else min(abs(float(ref_t) - float(item[0])), abs(float(ref_t) - float(item[1]))),
                -float(item[1]) + float(item[0]),
            ),
        )
    start_t, end_t = best_interval
    return [
        (ox + (dx * float(start_t)), oy + (dy * float(start_t))),
        (ox + (dx * float(end_t)), oy + (dy * float(end_t))),
    ]


def _polygon_exterior_xy(poly: Polygon | None) -> List[Tuple[float, float]]:
    if poly is None or poly.is_empty:
        return []
    try:
        return [(float(x), float(y)) for x, y in list(poly.exterior.coords)[:-1]]
    except Exception:
        return []


def _dedupe_scan_points_xy(
    points_xy: Sequence[Tuple[float, float]],
    *,
    min_dist_m: float = 1.0,
) -> List[Tuple[float, float]]:
    out: List[Tuple[float, float]] = []
    for point_xy in points_xy:
        point = (float(point_xy[0]), float(point_xy[1]))
        if out and _distance_xy(out[-1], point) < float(min_dist_m):
            continue
        out.append(point)
    if len(out) >= 2 and _distance_xy(out[0], out[-1]) < float(min_dist_m):
        out.pop()
    return out


def _orient_line_sweep_for_anchor(
    sweep_xy: Sequence[Tuple[float, float]],
    *,
    anchor_ref_xy: Tuple[float, float] | None,
    offset_m: float,
    route_line_xy: Sequence[Tuple[float, float]] | None = None,
) -> List[Tuple[float, float]]:
    if len(sweep_xy) < 2:
        return [(float(x), float(y)) for x, y in sweep_xy]
    forward = [
        (float(sweep_xy[0][0]), float(sweep_xy[0][1])),
        (float(sweep_xy[-1][0]), float(sweep_xy[-1][1])),
    ]
    reverse = [forward[-1], forward[0]]
    if anchor_ref_xy is None:
        return forward
    anchor_forward = _line_anchor_xy_from_sweep(
        forward,
        offset_m=offset_m,
        route_line_xy=route_line_xy,
        reference_xy=anchor_ref_xy,
    )
    anchor_reverse = _line_anchor_xy_from_sweep(
        reverse,
        offset_m=offset_m,
        route_line_xy=route_line_xy,
        reference_xy=anchor_ref_xy,
    )
    if anchor_forward is None:
        return reverse if anchor_reverse is not None else forward
    if anchor_reverse is None:
        return forward
    if _distance_xy(anchor_reverse, anchor_ref_xy) + 1e-6 < _distance_xy(anchor_forward, anchor_ref_xy):
        return reverse
    return forward


def _point_on_linestring_xy(line: LineString, distance_m: float) -> Tuple[float, float]:
    point = line.interpolate(max(0.0, min(float(line.length), float(distance_m))))
    return float(point.x), float(point.y)


def _unit_xy(dx: float, dy: float) -> Tuple[float, float] | None:
    norm = math.hypot(float(dx), float(dy))
    if norm <= 1e-6:
        return None
    return float(dx) / norm, float(dy) / norm


def _blend_unit_xy(
    first_xy: Tuple[float, float],
    second_xy: Tuple[float, float],
    ratio: float,
) -> Tuple[float, float] | None:
    ax, ay = first_xy
    bx, by = second_xy
    if (float(ax) * float(bx)) + (float(ay) * float(by)) < 0.0:
        bx, by = -float(bx), -float(by)
    ratio = max(0.0, min(1.0, float(ratio)))
    return _unit_xy(
        (float(ax) * (1.0 - ratio)) + (float(bx) * ratio),
        (float(ay) * (1.0 - ratio)) + (float(by) * ratio),
    )


def _centerline_tangent_xy(
    points_xy: Sequence[Tuple[float, float]],
    *,
    distance_m: float,
) -> Tuple[float, float] | None:
    rows = [(float(x), float(y)) for x, y in points_xy]
    if len(rows) < 2:
        return None

    segment_dirs: List[Tuple[float, float]] = []
    segment_lengths: List[float] = []
    for idx in range(1, len(rows)):
        dx = float(rows[idx][0]) - float(rows[idx - 1][0])
        dy = float(rows[idx][1]) - float(rows[idx - 1][1])
        seg_len_m = math.hypot(dx, dy)
        unit_xy = _unit_xy(dx, dy)
        if unit_xy is None or seg_len_m <= 1e-6:
            continue
        segment_dirs.append(unit_xy)
        segment_lengths.append(float(seg_len_m))
    if not segment_dirs:
        return None

    total_len_m = sum(segment_lengths)
    target_m = max(0.0, min(float(total_len_m), float(distance_m)))
    walked_m = 0.0
    for idx, seg_len_m in enumerate(segment_lengths):
        if target_m <= walked_m + seg_len_m + 1e-6 or idx == len(segment_lengths) - 1:
            ratio = 0.0 if seg_len_m <= 1e-6 else (target_m - walked_m) / seg_len_m
            current_dir = segment_dirs[idx]
            start_dir = (
                _blend_unit_xy(segment_dirs[idx - 1], current_dir, 0.5)
                if idx > 0
                else current_dir
            ) or current_dir
            end_dir = (
                _blend_unit_xy(current_dir, segment_dirs[idx + 1], 0.5)
                if idx < len(segment_dirs) - 1
                else current_dir
            ) or current_dir
            return _blend_unit_xy(start_dir, end_dir, ratio) or current_dir
        walked_m += seg_len_m
    return segment_dirs[-1]


def _centerline_sweep_distances(total_len_m: float, sweep_spacing_m: float) -> List[float]:
    total_len_m = float(total_len_m)
    sweep_spacing_m = float(sweep_spacing_m)
    if total_len_m <= 1e-6 or sweep_spacing_m <= 0.0:
        return []
    distances_m: List[float] = [0.0]
    cursor_m = float(sweep_spacing_m)
    while cursor_m < total_len_m - 1e-6:
        distances_m.append(float(cursor_m))
        cursor_m += float(sweep_spacing_m)
    if abs(float(distances_m[-1]) - total_len_m) > max(5.0, sweep_spacing_m * 0.25):
        distances_m.append(total_len_m)
    return distances_m


def _centerline_corridor_sweep_lines_xy(
    planner: _HeadlessLinePlanner,
    piece: SplitPiece,
    *,
    sweep_spacing_m: float,
    route_offset_m: float,
    origin_xy: Tuple[float, float] | None,
    deployment_direction_xy: Sequence[Tuple[float, float]] | None = None,
    db_sep_m: float = 0.0,
    aircraft_row: Dict[str, Any] | None = None,
) -> tuple[
    List[List[Tuple[float, float]]],
    List[Dict[str, Any]],
    Tuple[float, float] | None,
    _LineOffsetPlan | None,
    float | None,
]:
    piece_poly = planner._piece_polygon_xy(piece)
    centerline_xy = _piece_centerline_xy(piece)
    if piece_poly is None or piece_poly.is_empty or len(centerline_xy) < 2:
        return [], [], None, None, None
    polygon_xy = _polygon_exterior_xy(piece_poly)
    if len(polygon_xy) < 3:
        return [], [], None, None, None

    sweep_spacing_m = float(sweep_spacing_m)
    route_offset_m = max(float(route_offset_m), 0.0)
    if sweep_spacing_m <= 0.0:
        return [], [], None, None, None

    if deployment_direction_xy:
        centerline_xy = _orient_polyline_xy_to_direction(
            centerline_xy,
            direction_xy=deployment_direction_xy,
        )
    else:
        centerline_xy = _orient_polyline_xy(centerline_xy, reference_xy=origin_xy)
    line = LineString(centerline_xy)
    if line.is_empty or float(line.length) <= 1e-6:
        return [], [], None, None, None

    min_x, min_y, max_x, max_y = piece_poly.bounds
    extend_m = max(
        200.0,
        math.hypot(float(max_x) - float(min_x), float(max_y) - float(min_y))
        + (float(route_offset_m) * 2.0)
        + (float(sweep_spacing_m) * 2.0),
    )
    distances_m = _centerline_sweep_distances(float(line.length), float(sweep_spacing_m))
    if not distances_m:
        return [], [], None

    route_side_ref_xy = origin_xy
    base_dir_xy: Tuple[float, float] | None = None
    first_anchor_xy: Tuple[float, float] | None = None
    first_anchor_clearance_m: float | None = None
    scan_lines_xy: List[List[Tuple[float, float]]] = []
    line_sweep_items: List[Dict[str, Any]] = []
    offset_plan = _choose_line_offset_plan(
        centerline_xy,
        route_offset_m=float(route_offset_m),
        db_sep_m=float(db_sep_m),
        aircraft_row=aircraft_row,
        fallback_origin_xy=origin_xy,
    )

    for idx, dist_m in enumerate(distances_m):
        center_xy = _point_on_linestring_xy(line, dist_m)
        tangent_xy = _centerline_tangent_xy(
            centerline_xy,
            distance_m=float(dist_m),
        )
        if tangent_xy is None:
            continue
        tx, ty = tangent_xy
        guide_dx, guide_dy = ty, -tx
        forward_sweep_xy = _scanline_polygon_segment_xy(
            polygon_xy,
            center_xy,
            (float(guide_dx), float(guide_dy)),
            reference_xy=center_xy,
        )
        if forward_sweep_xy is None:
            guide = LineString(
                [
                    (center_xy[0] - (guide_dx * extend_m), center_xy[1] - (guide_dy * extend_m)),
                    (center_xy[0] + (guide_dx * extend_m), center_xy[1] + (guide_dy * extend_m)),
                ]
            )
            seg = _nearest_linestring_to_xy(
                planner,
                piece_poly.intersection(guide),
                center_xy,
            )
            if seg is None or len(seg.coords) < 2:
                continue
            forward_sweep_xy = [
                (float(seg.coords[0][0]), float(seg.coords[0][1])),
                (float(seg.coords[-1][0]), float(seg.coords[-1][1])),
            ]
        reverse_sweep_xy = [forward_sweep_xy[-1], forward_sweep_xy[0]]
        if base_dir_xy is None:
            base_sweep_xy = _orient_line_sweep_for_anchor(
                forward_sweep_xy,
                anchor_ref_xy=route_side_ref_xy,
                offset_m=float(route_offset_m),
                route_line_xy=centerline_xy,
            )
            base_dir_xy = (
                float(base_sweep_xy[-1][0]) - float(base_sweep_xy[0][0]),
                float(base_sweep_xy[-1][1]) - float(base_sweep_xy[0][1]),
            )
        else:
            forward_dot = (
                (float(forward_sweep_xy[-1][0]) - float(forward_sweep_xy[0][0])) * float(base_dir_xy[0])
                + (float(forward_sweep_xy[-1][1]) - float(forward_sweep_xy[0][1])) * float(base_dir_xy[1])
            )
            reverse_dot = (
                (float(reverse_sweep_xy[-1][0]) - float(reverse_sweep_xy[0][0])) * float(base_dir_xy[0])
                + (float(reverse_sweep_xy[-1][1]) - float(reverse_sweep_xy[0][1])) * float(base_dir_xy[1])
            )
            base_sweep_xy = forward_sweep_xy if forward_dot >= reverse_dot else reverse_sweep_xy
        if idx == 0:
            offset_plan = _clamp_line_offset_plan_to_sweep(
                offset_plan,
                sweep_xy=base_sweep_xy,
                route_line_xy=centerline_xy,
                db_sep_m=float(db_sep_m),
            )
        anchor_xy = _line_anchor_xy_from_sweep(
            base_sweep_xy,
            offset_m=float(route_offset_m),
            route_line_xy=centerline_xy,
            reference_xy=route_side_ref_xy,
            offset_plan=offset_plan,
        )
        if anchor_xy is not None and first_anchor_xy is None:
            first_anchor_xy = anchor_xy
            first_anchor_clearance_m = _line_anchor_clearance_m(anchor_xy, base_sweep_xy)
        if anchor_xy is not None and route_side_ref_xy is None:
            route_side_ref_xy = anchor_xy
        scan_sweep_xy = list(base_sweep_xy)
        if idx % 2 == 1:
            scan_sweep_xy.reverse()
        scan_lines_xy.append(scan_sweep_xy)
        if anchor_xy is not None:
            line_sweep_items.append(
                {
                    "anchorXY": (float(anchor_xy[0]), float(anchor_xy[1])),
                    "sweepXY": [(float(x), float(y)) for x, y in scan_sweep_xy],
                }
            )

    return scan_lines_xy, line_sweep_items, first_anchor_xy, offset_plan, first_anchor_clearance_m


def _cached_centerline_corridor_sweep_lines_xy(
    planner: _HeadlessLinePlanner,
    piece: SplitPiece,
    *,
    sweep_spacing_m: float,
    route_offset_m: float,
    origin_xy: Tuple[float, float] | None,
    deployment_direction_xy: Sequence[Tuple[float, float]] | None = None,
    db_sep_m: float = 0.0,
    aircraft_row: Dict[str, Any] | None = None,
) -> tuple[
    List[List[Tuple[float, float]]],
    List[Dict[str, Any]],
    Tuple[float, float] | None,
    _LineOffsetPlan | None,
    float | None,
]:
    runtime_key = (
        "centerline_corridor_sweep",
        id(piece),
        _line_runtime_cache_value(float(sweep_spacing_m)),
        _line_runtime_cache_value(float(route_offset_m)),
        _line_runtime_cache_value(origin_xy),
        _line_runtime_cache_value(deployment_direction_xy),
        _line_runtime_cache_value(float(db_sep_m)),
        id(aircraft_row) if aircraft_row is not None else None,
    )
    runtime_cached = _line_runtime_cache_get(planner, runtime_key)
    if runtime_cached is not None:
        replan_perf.add("mission_planning.next_collab_line.cache.centerline_sweep", runtimeHit=1)
        return runtime_cached
    if not _line_geometry_cache_enabled():
        result = _centerline_corridor_sweep_lines_xy(
            planner,
            piece,
            sweep_spacing_m=float(sweep_spacing_m),
            route_offset_m=float(route_offset_m),
            origin_xy=origin_xy,
            deployment_direction_xy=deployment_direction_xy,
            db_sep_m=float(db_sep_m),
            aircraft_row=aircraft_row,
        )
        _line_runtime_cache_store(planner, runtime_key, result)
        return copy.deepcopy(result)
    payload = {
        "piece": _piece_geometry_cache_payload(piece),
        "sweepSpacingM": round(float(sweep_spacing_m), 6),
        "routeOffsetM": round(float(route_offset_m), 6),
        "originXY": _stable_cache_payload(origin_xy),
        "deploymentDirectionXY": _stable_cache_payload(deployment_direction_xy or []),
        "dbSepM": round(float(db_sep_m), 6),
        "aircraftRow": _stable_cache_payload(aircraft_row or {}),
    }
    key = _line_geometry_cache_key("centerline_corridor_sweep", payload)
    cached = _line_geometry_cache_get(key)
    if cached is not None:
        replan_perf.add("mission_planning.next_collab_line.cache.centerline_sweep", hit=1)
        return cached
    replan_perf.add("mission_planning.next_collab_line.cache.centerline_sweep", miss=1)
    result = _centerline_corridor_sweep_lines_xy(
        planner,
        piece,
        sweep_spacing_m=float(sweep_spacing_m),
        route_offset_m=float(route_offset_m),
        origin_xy=origin_xy,
        deployment_direction_xy=deployment_direction_xy,
        db_sep_m=float(db_sep_m),
        aircraft_row=aircraft_row,
    )
    _line_geometry_cache_store(key, result)
    _line_runtime_cache_store(planner, runtime_key, result)
    return copy.deepcopy(result)


def _corridor_sweep_lines_xy(
    planner: _HeadlessLinePlanner,
    piece: SplitPiece,
    overlay: Dict[str, Any],
    *,
    sweep_spacing_m: float,
    route_offset_m: float,
    origin_xy: Tuple[float, float] | None,
    deployment_direction_xy: Sequence[Tuple[float, float]] | None = None,
    db_sep_m: float = 0.0,
    aircraft_row: Dict[str, Any] | None = None,
) -> tuple[
    List[List[Tuple[float, float]]],
    List[Dict[str, Any]],
    Tuple[float, float] | None,
    _LineOffsetPlan | None,
    float | None,
]:
    piece_poly = planner._piece_polygon_xy(piece)
    bounds = planner._overlay_st_bounds(overlay)
    if piece_poly is None or piece_poly.is_empty or bounds is None:
        return [], [], None, None, None
    polygon_xy = _polygon_exterior_xy(piece_poly)
    if len(polygon_xy) < 3:
        return [], [], None, None, None
    sweep_spacing_m = float(sweep_spacing_m)
    route_offset_m = max(float(route_offset_m), 0.0)
    if sweep_spacing_m <= 0.0:
        return [], [], None, None, None

    _bearing_deg, ux, uy, vx, vy, min_s, max_s, min_t, max_t = bounds
    mid_t = 0.5 * (float(min_t) + float(max_t))
    min_face_xy = planner._from_st_xy(min_s, mid_t, ux, uy, vx, vy)
    max_face_xy = planner._from_st_xy(max_s, mid_t, ux, uy, vx, vy)

    oriented_axis_xy = _orient_polyline_xy_to_direction(
        [min_face_xy, max_face_xy],
        direction_xy=deployment_direction_xy,
    ) if deployment_direction_xy else []
    if oriented_axis_xy and _distance_xy(oriented_axis_xy[0], max_face_xy) < _distance_xy(
        oriented_axis_xy[0], min_face_xy
    ):
        start_s = float(max_s)
        end_s = float(min_s)
        step_sign = -1.0
    elif oriented_axis_xy:
        start_s = float(min_s)
        end_s = float(max_s)
        step_sign = 1.0
    elif origin_xy is not None and _distance_xy(origin_xy, max_face_xy) + 1e-6 < _distance_xy(origin_xy, min_face_xy):
        start_s = float(max_s)
        end_s = float(min_s)
        step_sign = -1.0
    else:
        start_s = float(min_s)
        end_s = float(max_s)
        step_sign = 1.0
    route_axis_xy = [
        planner._from_st_xy(float(start_s), float(mid_t), ux, uy, vx, vy),
        planner._from_st_xy(float(end_s), float(mid_t), ux, uy, vx, vy),
    ]

    s_values: List[float] = [float(start_s)]
    cursor_s = float(start_s)
    while True:
        next_s = float(cursor_s + (step_sign * float(sweep_spacing_m)))
        if (step_sign > 0.0 and next_s >= float(end_s) - 1e-6) or (
            step_sign < 0.0 and next_s <= float(end_s) + 1e-6
        ):
            break
        s_values.append(float(next_s))
        cursor_s = float(next_s)
    if abs(float(s_values[-1]) - float(end_s)) > max(5.0, float(sweep_spacing_m) * 0.25):
        s_values.append(float(end_s))

    pad_t = max(80.0, float(abs(max_t - min_t)) * 0.25)
    route_side_ref_xy = origin_xy
    base_dir_xy: Tuple[float, float] | None = None
    first_anchor_xy: Tuple[float, float] | None = None
    first_anchor_clearance_m: float | None = None
    scan_lines_xy: List[List[Tuple[float, float]]] = []
    line_sweep_items: List[Dict[str, Any]] = []
    offset_plan = _choose_line_offset_plan(
        route_axis_xy,
        route_offset_m=float(route_offset_m),
        db_sep_m=float(db_sep_m),
        aircraft_row=aircraft_row,
        fallback_origin_xy=origin_xy,
    )

    for idx, s_val in enumerate(s_values):
        guide_center_xy = planner._from_st_xy(float(s_val), float(mid_t), ux, uy, vx, vy)
        forward_sweep_xy = _scanline_polygon_segment_xy(
            polygon_xy,
            guide_center_xy,
            (float(vx), float(vy)),
            reference_xy=guide_center_xy,
        )
        if forward_sweep_xy is None:
            guide = LineString(
                [
                    planner._from_st_xy(float(s_val), float(min_t - pad_t), ux, uy, vx, vy),
                    planner._from_st_xy(float(s_val), float(max_t + pad_t), ux, uy, vx, vy),
                ]
            )
            seg = _nearest_linestring_to_xy(
                planner,
                piece_poly.intersection(guide),
                guide_center_xy,
            )
            if seg is None or len(seg.coords) < 2:
                continue
            forward_sweep_xy = [
                (float(seg.coords[0][0]), float(seg.coords[0][1])),
                (float(seg.coords[-1][0]), float(seg.coords[-1][1])),
            ]
        reverse_sweep_xy = [forward_sweep_xy[-1], forward_sweep_xy[0]]
        if base_dir_xy is None:
            base_sweep_xy = _orient_line_sweep_for_anchor(
                forward_sweep_xy,
                anchor_ref_xy=route_side_ref_xy,
                offset_m=float(route_offset_m),
                route_line_xy=route_axis_xy,
            )
            base_dir_xy = (
                float(base_sweep_xy[-1][0]) - float(base_sweep_xy[0][0]),
                float(base_sweep_xy[-1][1]) - float(base_sweep_xy[0][1]),
            )
        else:
            forward_dot = (
                (float(forward_sweep_xy[-1][0]) - float(forward_sweep_xy[0][0])) * float(base_dir_xy[0])
                + (float(forward_sweep_xy[-1][1]) - float(forward_sweep_xy[0][1])) * float(base_dir_xy[1])
            )
            reverse_dot = (
                (float(reverse_sweep_xy[-1][0]) - float(reverse_sweep_xy[0][0])) * float(base_dir_xy[0])
                + (float(reverse_sweep_xy[-1][1]) - float(reverse_sweep_xy[0][1])) * float(base_dir_xy[1])
            )
            base_sweep_xy = forward_sweep_xy if forward_dot >= reverse_dot else reverse_sweep_xy
        if idx == 0:
            offset_plan = _clamp_line_offset_plan_to_sweep(
                offset_plan,
                sweep_xy=base_sweep_xy,
                route_line_xy=route_axis_xy,
                db_sep_m=float(db_sep_m),
            )
        anchor_xy = _line_anchor_xy_from_sweep(
            base_sweep_xy,
            offset_m=float(route_offset_m),
            route_line_xy=route_axis_xy,
            reference_xy=route_side_ref_xy,
            offset_plan=offset_plan,
        )
        if anchor_xy is not None and first_anchor_xy is None:
            first_anchor_xy = anchor_xy
            first_anchor_clearance_m = _line_anchor_clearance_m(anchor_xy, base_sweep_xy)
        if anchor_xy is not None and route_side_ref_xy is None:
            route_side_ref_xy = anchor_xy
        scan_sweep_xy = list(base_sweep_xy)
        if idx % 2 == 1:
            scan_sweep_xy.reverse()
        scan_lines_xy.append(scan_sweep_xy)
        if anchor_xy is not None:
            line_sweep_items.append(
                {
                    "anchorXY": (float(anchor_xy[0]), float(anchor_xy[1])),
                    "sweepXY": [(float(x), float(y)) for x, y in scan_sweep_xy],
                }
            )

    return scan_lines_xy, line_sweep_items, first_anchor_xy, offset_plan, first_anchor_clearance_m


def _synthetic_line_overlay(
    piece: SplitPiece,
    *,
    reference_bearing_deg: float,
) -> Optional[Dict[str, Any]]:
    centerline_xy = _piece_centerline_xy(piece)
    if len(centerline_xy) < 2:
        return None
    line = LineString(centerline_xy)
    length_m = float(line.length)
    if length_m <= 1e-6:
        return None
    bearing_deg = _line_bearing_deg(centerline_xy) or float(reference_bearing_deg)
    width_m = max(1.0, float(_piece_width_m(piece)))
    half_width_m = max(0.5, width_m * 0.5)
    half_length_m = max(1.0, length_m * 0.5)
    center_point = line.interpolate(0.5, normalized=True)
    center_xy = (float(center_point.x), float(center_point.y))
    theta = math.radians(float(bearing_deg) % 360.0)
    ux = math.sin(theta)
    uy = math.cos(theta)
    vx = uy
    vy = -ux

    box_xy = [
        (center_xy[0] - (ux * half_length_m) - (vx * half_width_m), center_xy[1] - (uy * half_length_m) - (vy * half_width_m)),
        (center_xy[0] + (ux * half_length_m) - (vx * half_width_m), center_xy[1] + (uy * half_length_m) - (vy * half_width_m)),
        (center_xy[0] + (ux * half_length_m) + (vx * half_width_m), center_xy[1] + (uy * half_length_m) + (vy * half_width_m)),
        (center_xy[0] - (ux * half_length_m) + (vx * half_width_m), center_xy[1] - (uy * half_length_m) + (vy * half_width_m)),
    ]

    return {
        "pieceIndex": int(piece.piece_index or 0),
        "aircraftID": int(piece.assigned_uav or 0),
        "bearingDeg": float(bearing_deg),
        "boxXY": box_xy,
        "midLineXY": [centerline_xy[0], centerline_xy[-1]],
        "centerXY": center_xy,
        "lengthM": float(length_m),
        "widthM": float(width_m),
        "maxWidthM": float(width_m),
        "maxWidthLeftM": float(half_width_m),
        "maxWidthRightM": float(half_width_m),
        "dbCoverWidthM": float(width_m),
        "dbMaxWidthM": float(width_m),
        "midLineRequired": False,
        "synthetic": True,
    }


def _line_reference_bearing_deg(
    line_specs: List[Dict[str, Any]],
    pieces: Sequence[SplitPiece],
) -> float:
    for spec in line_specs:
        coords_xy = coords_to_xy(spec.get("coordinateList") or [])
        if len(coords_xy) >= 2:
            return _line_bearing_deg(coords_xy)
    for piece in pieces:
        coords_xy = _piece_centerline_xy(piece)
        if len(coords_xy) >= 2:
            return _line_bearing_deg(coords_xy)
    return 0.0


def _next_collab_entry_tprime_target_sep_ratio(runtime_cfg: Dict[str, Any] | None = None) -> float:
    value = float(
        get_runtime_float(
            "next_collab_entry_tprime_target_sep_ratio",
            NEXT_COLLAB_ENTRY_TPRIME_TARGET_SEP_RATIO,
            runtime_cfg,
        )
    )
    scale = float(
        get_runtime_float(
            "next_collab_entry_tprime_ratio_scale",
            NEXT_COLLAB_ENTRY_TPRIME_RATIO_SCALE,
            runtime_cfg,
        )
    )
    return max(0.05, min(1.0, float(value) * max(0.10, min(5.0, float(scale)))))


def _next_collab_entry_tprime_db_row(
    planner: _HeadlessLinePlanner,
    width_m: float,
    *,
    runtime_cfg: Dict[str, Any] | None = None,
) -> Dict[str, float] | None:
    target_m = max(0.0, float(width_m))
    if target_m <= 0.0:
        return None

    candidates = [
        (float(width_m), float(sep_m), float(fov_deg), float(vel_mps))
        for width_m, sep_m, fov_deg, vel_mps in _planner_fov_db_numeric_rows(planner)
        if float(width_m) + 1e-6 >= target_m
    ]
    if not candidates:
        fallback_row = planner._covering_db_row(float(width_m))
        return dict(fallback_row) if isinstance(fallback_row, dict) else None

    max_sep_row = min(
        candidates,
        key=lambda row: (
            -float(row[1]),
            float(row[0]),
            -float(row[2]),
        ),
    )
    max_sep_m = float(max_sep_row[1])
    if max_sep_m <= 0.0:
        return {"width": float(max_sep_row[0]), "sep": float(max_sep_row[1]), "fov": float(max_sep_row[2]), "vel": float(max_sep_row[3])}

    target_sep_m = max_sep_m * _next_collab_entry_tprime_target_sep_ratio(runtime_cfg)
    lower_or_equal = [
        row
        for row in candidates
        if float(row[1]) <= target_sep_m + 1e-6
    ]
    if lower_or_equal:
        best = max(
            lower_or_equal,
            key=lambda row: (
                float(row[1]),
                -float(row[2]),
                -float(row[0]),
            ),
        )
        return {"width": float(best[0]), "sep": float(best[1]), "fov": float(best[2]), "vel": float(best[3])}

    best = min(
        candidates,
        key=lambda row: (
            abs(float(row[1]) - target_sep_m),
            float(row[0]),
            -float(row[2]),
        ),
    )
    return {"width": float(best[0]), "sep": float(best[1]), "fov": float(best[2]), "vel": float(best[3])}


def _next_collab_entry_tprime_target_sep_m(
    planner: _HeadlessLinePlanner,
    width_m: float,
    *,
    runtime_cfg: Dict[str, Any] | None = None,
) -> float:
    db_row = _next_collab_entry_tprime_db_row(planner, width_m, runtime_cfg=runtime_cfg)
    if not isinstance(db_row, dict):
        return 0.0
    sep_m = float(db_row.get("sep", 0.0) or 0.0)
    if sep_m <= 0.0:
        return 0.0
    return float(sep_m)


def _cached_next_collab_entry_tprime_target_sep_m(
    planner: _HeadlessLinePlanner,
    width_m: float,
    *,
    runtime_cfg: Dict[str, Any] | None = None,
) -> float:
    key = (
        "entry_tprime_target_sep_m",
        _line_runtime_cache_value(float(width_m)),
        id(runtime_cfg) if runtime_cfg is not None else None,
    )
    cached = _line_runtime_cache_get(planner, key)
    if cached is not None:
        return float(cached)
    result = float(_next_collab_entry_tprime_target_sep_m(planner, float(width_m), runtime_cfg=runtime_cfg))
    _line_runtime_cache_store(planner, key, result)
    return float(result)


def _next_collab_line_db_weights(
    runtime_cfg: Dict[str, Any] | None = None,
) -> tuple[float, float, float]:
    width_weight = max(
        0.0,
        float(
            get_runtime_float(
                "next_collab_line_db_width_weight",
                NEXT_COLLAB_LINE_DB_WIDTH_WEIGHT,
                runtime_cfg,
            )
        ),
    )
    sep_weight = max(
        0.0,
        float(
            get_runtime_float(
                "next_collab_line_db_sep_weight",
                NEXT_COLLAB_LINE_DB_SEP_WEIGHT,
                runtime_cfg,
            )
        ),
    )
    fov_weight = max(
        0.0,
        float(
            get_runtime_float(
                "next_collab_line_db_fov_weight",
                NEXT_COLLAB_LINE_DB_FOV_WEIGHT,
                runtime_cfg,
            )
        ),
    )
    total = float(width_weight + sep_weight + fov_weight)
    if total <= 1e-9:
        return (
            float(NEXT_COLLAB_LINE_DB_WIDTH_WEIGHT),
            float(NEXT_COLLAB_LINE_DB_SEP_WEIGHT),
            float(NEXT_COLLAB_LINE_DB_FOV_WEIGHT),
        )
    return (
        float(width_weight) / total,
        float(sep_weight) / total,
        float(fov_weight) / total,
    )


def _normalized_unit_score(value: float, minimum: float, maximum: float, *, prefer_low: bool) -> float:
    if maximum - minimum <= 1e-9:
        return 1.0
    ratio = (float(value) - float(minimum)) / float(maximum - minimum)
    ratio = max(0.0, min(1.0, float(ratio)))
    return 1.0 - ratio if prefer_low else ratio


def _apply_line_smaller_fov_steps(
    planner: _HeadlessLinePlanner,
    base_row: Dict[str, float] | None,
    *,
    width_m: float,
    sep_m: float,
    runtime_cfg: Dict[str, Any] | None = None,
) -> Dict[str, float] | None:
    if not isinstance(base_row, dict):
        return base_row
    steps = _runtime_line_fov_db_smaller_fov_steps(runtime_cfg)
    if steps <= 0:
        return dict(base_row)
    try:
        base_fov = float(base_row.get("fov", 0.0) or 0.0)
    except Exception:
        base_fov = 0.0
    if base_fov <= 0.0:
        return dict(base_row)

    target_m = max(0.0, float(width_m))
    limit_sep_m = _db_sep_requirement_m(float(sep_m), runtime_cfg)
    if target_m <= 0.0:
        return dict(base_row)

    candidates = [
        (float(width_m), float(sep_m), float(fov_deg), float(vel_mps))
        for width_m, sep_m, fov_deg, vel_mps in _planner_fov_db_numeric_rows(planner)
        if float(width_m) + 1e-6 >= target_m and float(fov_deg) > 0.0
    ]
    if not candidates:
        return dict(base_row)

    pool = candidates
    if limit_sep_m > 0.0:
        matching = [row for row in candidates if float(row[1]) >= limit_sep_m]
        if matching:
            pool = matching

    lower_fovs = sorted(
        {round(float(row[2]), 6) for row in pool if float(row[2]) < base_fov - 1e-6},
        reverse=True,
    )
    if not lower_fovs:
        return dict(base_row)

    if steps > len(lower_fovs):
        # Not enough lower FOV levels: keep the base row instead of jumping to
        # the lowest FOV in the DB, whose rows can carry a far larger sep and
        # blow up the entry/route offset.
        return dict(base_row)
    target_fov = float(lower_fovs[steps - 1])
    fov_rows = [row for row in pool if abs(float(row[2]) - target_fov) <= 1e-6]
    if not fov_rows:
        return dict(base_row)

    if limit_sep_m > 0.0 and any(float(row[1]) >= limit_sep_m for row in fov_rows):
        eligible = [row for row in fov_rows if float(row[1]) >= limit_sep_m]
        best = min(
            eligible,
            key=lambda row: (
                float(row[1]) - limit_sep_m,
                -float(row[3]),
                -float(row[0]),
            ),
        )
    else:
        best = min(
            fov_rows,
            key=lambda row: (
                abs(float(row[1]) - max(limit_sep_m, 0.0)),
                -float(row[3]),
                -float(row[0]),
            ),
        )

    return {
        "width": float(best[0]),
        "sep": float(best[1]),
        "fov": float(best[2]),
        "vel": float(best[3]),
    }


def _next_collab_resolved_db_row(
    planner: _HeadlessLinePlanner,
    width_m: float,
    sep_m: float,
    *,
    runtime_cfg: Dict[str, Any] | None = None,
) -> Dict[str, float] | None:
    target_m = max(0.0, float(width_m))
    # The pool loops below rebind width_m/sep_m; keep the requested sep for
    # the smaller-FOV downstep call at the end.
    request_sep_m = float(sep_m)
    limit_sep_m = _db_sep_requirement_m(request_sep_m, runtime_cfg)
    if target_m <= 0.0:
        return None

    candidates = [
        (float(width_m), float(sep_m), float(fov_deg), float(vel_mps))
        for width_m, sep_m, fov_deg, vel_mps in _planner_fov_db_numeric_rows(planner)
        if float(width_m) + 1e-6 >= target_m
    ]
    if not candidates:
        return None

    matching: List[tuple[float, float, float, float]] = []
    if limit_sep_m > 0.0:
        matching = [
            row
            for row in candidates
            if float(row[1]) >= limit_sep_m
        ]
    pool = matching or candidates
    width_weight, sep_weight, fov_weight = _next_collab_line_db_weights(runtime_cfg)

    min_width_error = math.inf
    max_width_error = -math.inf
    min_sep_error = math.inf
    max_sep_error = -math.inf
    min_fov = math.inf
    max_fov = -math.inf
    pool_metrics: List[tuple[tuple[float, float, float, float], float, float, float]] = []
    for row in pool:
        width_m, sep_m, fov_value, vel_mps = row
        width_error = max(float(width_m) - target_m, 0.0)
        if limit_sep_m > 0.0 and matching:
            sep_error = max(float(sep_m) - limit_sep_m, 0.0)
        elif limit_sep_m > 0.0:
            sep_error = abs(float(sep_m) - limit_sep_m)
        else:
            sep_error = 0.0
        fov_float = float(fov_value)
        pool_metrics.append(((float(width_m), float(sep_m), fov_float, float(vel_mps)), width_error, sep_error, fov_float))
        min_width_error = min(min_width_error, width_error)
        max_width_error = max(max_width_error, width_error)
        min_sep_error = min(min_sep_error, sep_error)
        max_sep_error = max(max_sep_error, sep_error)
        min_fov = min(min_fov, fov_float)
        max_fov = max(max_fov, fov_float)

    if not pool_metrics:
        return None

    width_span = max_width_error - min_width_error
    sep_span = max_sep_error - min_sep_error
    fov_span = max_fov - min_fov
    best_key: tuple[float, float, float, float, float, float] | None = None
    best_row: tuple[float, float, float, float] | None = None
    for row, width_error, sep_error, fov_value in pool_metrics:
        width_score = 1.0 if width_span <= 1e-9 else 1.0 - max(0.0, min(1.0, (width_error - min_width_error) / width_span))
        sep_score = 1.0 if sep_span <= 1e-9 else 1.0 - max(0.0, min(1.0, (sep_error - min_sep_error) / sep_span))
        fov_score = 1.0 if fov_span <= 1e-9 else max(0.0, min(1.0, (fov_value - min_fov) / fov_span))
        total_score = (
            (float(width_weight) * float(width_score))
            + (float(sep_weight) * float(sep_score))
            + (float(fov_weight) * float(fov_score))
        )
        width_m, sep_m, fov_m, vel_mps = row
        key = (
            float(total_score),
            -max(float(width_m) - target_m, 0.0),
            -(
                max(float(sep_m) - limit_sep_m, 0.0)
                if limit_sep_m > 0.0 and matching
                else abs(float(sep_m) - limit_sep_m)
            ),
            float(fov_m),
            float(sep_m),
            float(vel_mps),
        )
        if best_key is None or key > best_key:
            best_key = key
            best_row = row
    if best_row is None:
        return None
    selected_row = {
        "width": float(best_row[0]),
        "sep": float(best_row[1]),
        "fov": float(best_row[2]),
        "vel": float(best_row[3]),
    }
    return _apply_line_smaller_fov_steps(
        planner,
        selected_row,
        width_m=float(target_m),
        sep_m=float(request_sep_m),
        runtime_cfg=runtime_cfg,
    )


def _cached_next_collab_resolved_db_row(
    planner: _HeadlessLinePlanner,
    width_m: float,
    sep_m: float,
    *,
    runtime_cfg: Dict[str, Any] | None = None,
) -> Dict[str, float] | None:
    key = (
        "next_collab_resolved_db_row",
        _line_runtime_cache_value(float(width_m)),
        _line_runtime_cache_value(float(sep_m)),
        int(_runtime_line_fov_db_smaller_fov_steps(runtime_cfg)),
        id(runtime_cfg) if runtime_cfg is not None else None,
    )
    cached = _line_runtime_cache_get(planner, key)
    if cached is not None:
        return dict(cached) if isinstance(cached, dict) else None
    result = _next_collab_resolved_db_row(
        planner,
        float(width_m),
        float(sep_m),
        runtime_cfg=runtime_cfg,
    )
    _line_runtime_cache_store(planner, key, dict(result) if isinstance(result, dict) else None)
    return dict(result) if isinstance(result, dict) else None


def _line_db_row_covering_anchor_sep(
    planner: _HeadlessLinePlanner,
    *,
    width_m: float,
    required_sep_m: float,
) -> Dict[str, float] | None:
    target_width_m = max(0.0, float(width_m))
    target_sep_m = max(0.0, float(required_sep_m))
    if target_width_m <= 0.0 or target_sep_m <= 0.0:
        return None
    rows = [
        dict(row)
        for row in (planner._fov_db_rows() or [])
        if float(row.get("width", 0.0) or 0.0) + 1e-6 >= target_width_m
        and float(row.get("sep", 0.0) or 0.0) + 1e-6 >= target_sep_m
    ]
    if not rows:
        return None
    return min(
        rows,
        key=lambda row: (
            float(row.get("fov", 0.0) or 0.0),
            float(row.get("sep", 0.0) or 0.0),
            float(row.get("width", 0.0) or 0.0),
        ),
    )


def _line_entry_endpoints_xy(
    sweep_lines_xy: Sequence[Sequence[Tuple[float, float]]],
) -> List[Tuple[float, float]] | None:
    for sweep_xy in sweep_lines_xy:
        rows = [(float(x), float(y)) for x, y in sweep_xy]
        if len(rows) < 2:
            continue
        return [rows[0], rows[-1]]
    return None


def _resolve_line_entry_tprime_state(
    planner: _HeadlessLinePlanner,
    *,
    part_width_m: float,
    tangent_xy: Tuple[float, float] | None,
    entry_target_xy: Tuple[float, float] | None,
    entry_line_endpoints_xy: Sequence[Tuple[float, float]] | None,
    runtime_cfg: Dict[str, Any] | None = None,
) -> tuple[Tuple[float, float] | None, float | None, Dict[str, float] | None]:
    if tangent_xy is None or entry_target_xy is None:
        return None, None, None
    if not isinstance(entry_line_endpoints_xy, Sequence) or len(entry_line_endpoints_xy) < 2:
        return None, None, None

    ep0 = entry_line_endpoints_xy[0]
    ep1 = entry_line_endpoints_xy[1]
    tprime_sep_m = max(
        _distance_xy(tangent_xy, (float(ep0[0]), float(ep0[1]))),
        _distance_xy(tangent_xy, (float(ep1[0]), float(ep1[1]))),
    )
    target_sep_m = _cached_next_collab_entry_tprime_target_sep_m(
        planner,
        float(part_width_m),
        runtime_cfg=runtime_cfg,
    )
    entry_t_prime_xy: Tuple[float, float] | None = None

    ingress_dx = float(entry_target_xy[0]) - float(tangent_xy[0])
    ingress_dy = float(entry_target_xy[1]) - float(tangent_xy[1])
    ingress_len_m = math.hypot(ingress_dx, ingress_dy)
    if tprime_sep_m > target_sep_m > 0.0 and ingress_len_m > 1e-6:
        ux_i = ingress_dx / ingress_len_m
        uy_i = ingress_dy / ingress_len_m
        lo = 0.0
        hi = float(ingress_len_m)
        for _ in range(_line_entry_tprime_binary_search_steps()):
            mid = (lo + hi) * 0.5
            test_xy = (
                float(tangent_xy[0]) + (ux_i * mid),
                float(tangent_xy[1]) + (uy_i * mid),
            )
            test_sep_m = max(
                _distance_xy(test_xy, (float(ep0[0]), float(ep0[1]))),
                _distance_xy(test_xy, (float(ep1[0]), float(ep1[1]))),
            )
            if test_sep_m > target_sep_m:
                lo = mid
            else:
                hi = mid
        entry_t_prime_xy = (
            float(tangent_xy[0]) + (ux_i * lo),
            float(tangent_xy[1]) + (uy_i * lo),
        )
        tprime_sep_m = max(
            _distance_xy(entry_t_prime_xy, (float(ep0[0]), float(ep0[1]))),
            _distance_xy(entry_t_prime_xy, (float(ep1[0]), float(ep1[1]))),
        )

    sep_cand_m = float(tprime_sep_m)

    resolved_db_row = _cached_next_collab_resolved_db_row(
        planner,
        float(part_width_m),
        float(sep_cand_m),
        runtime_cfg=runtime_cfg,
    )
    if not isinstance(resolved_db_row, dict):
        resolved_db_row = _next_collab_entry_tprime_db_row(
            planner,
            float(part_width_m),
            runtime_cfg=runtime_cfg,
        )
    return entry_t_prime_xy, float(sep_cand_m), dict(resolved_db_row) if isinstance(resolved_db_row, dict) else None


def _cached_resolve_line_entry_tprime_state(
    planner: _HeadlessLinePlanner,
    *,
    part_width_m: float,
    tangent_xy: Tuple[float, float] | None,
    entry_target_xy: Tuple[float, float] | None,
    entry_line_endpoints_xy: Tuple[Tuple[float, float], Tuple[float, float]] | None,
    runtime_cfg: Dict[str, Any] | None,
) -> tuple[Tuple[float, float] | None, float | None, Dict[str, Any] | None]:
    runtime_key = (
        "entry_tprime_state",
        _line_runtime_cache_value(float(part_width_m)),
        _line_runtime_cache_value(tangent_xy),
        _line_runtime_cache_value(entry_target_xy),
        _line_runtime_cache_value(entry_line_endpoints_xy),
        id(runtime_cfg) if runtime_cfg is not None else None,
    )
    runtime_cached = _line_runtime_cache_get(planner, runtime_key)
    if runtime_cached is not None:
        replan_perf.add("mission_planning.next_collab_line.cache.entry_tprime", runtimeHit=1)
        return runtime_cached
    if not _line_geometry_cache_enabled():
        result = _resolve_line_entry_tprime_state(
            planner,
            part_width_m=float(part_width_m),
            tangent_xy=tangent_xy,
            entry_target_xy=entry_target_xy,
            entry_line_endpoints_xy=entry_line_endpoints_xy,
            runtime_cfg=runtime_cfg,
        )
        _line_runtime_cache_store(planner, runtime_key, result)
        return copy.deepcopy(result)
    payload = {
        "partWidthM": round(float(part_width_m), 6),
        "tangentXY": _stable_cache_payload(tangent_xy),
        "entryTargetXY": _stable_cache_payload(entry_target_xy),
        "entryLineEndpointsXY": _stable_cache_payload(entry_line_endpoints_xy),
        "runtimeCfg": _stable_cache_payload(runtime_cfg or {}),
        "fovDbRows": _planner_fov_db_signature(planner),
    }
    key = _line_geometry_cache_key("entry_tprime_state", payload)
    cached = _line_geometry_cache_get(key)
    if cached is not None:
        replan_perf.add("mission_planning.next_collab_line.cache.entry_tprime", hit=1)
        return cached
    replan_perf.add("mission_planning.next_collab_line.cache.entry_tprime", miss=1)
    result = _resolve_line_entry_tprime_state(
        planner,
        part_width_m=float(part_width_m),
        tangent_xy=tangent_xy,
        entry_target_xy=entry_target_xy,
        entry_line_endpoints_xy=entry_line_endpoints_xy,
        runtime_cfg=runtime_cfg,
    )
    _line_geometry_cache_store(key, result)
    _line_runtime_cache_store(planner, runtime_key, result)
    return copy.deepcopy(result)


def _augment_line_path_row(
    planner: _HeadlessLinePlanner,
    piece: SplitPiece,
    overlay: Dict[str, Any],
    path_row: Dict[str, Any],
    *,
    shared_db_row: Dict[str, Any] | None = None,
    aircraft_row: Dict[str, Any] | None = None,
    deployment_direction_xy: Sequence[Tuple[float, float]] | None = None,
    runtime_cfg: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    perf_total_started = replan_perf.start_timer()
    perf_piece_idx = int(piece.piece_index or 0)
    out = dict(path_row)
    if runtime_cfg is None:
        runtime_cfg = load_runtime_settings()
    forced_shared_row = dict(shared_db_row) if isinstance(shared_db_row, dict) and shared_db_row else None
    marker_rows = out.get("markerRows") if isinstance(out.get("markerRows"), list) else []
    centerline_xy = _piece_centerline_xy(piece)
    target_xy = _piece_target_xy(piece)
    target_face_raw = out.get("targetFaceXY")
    target_face_xy = (
        (float(target_face_raw[0]), float(target_face_raw[1]))
        if isinstance(target_face_raw, (tuple, list)) and len(target_face_raw) >= 2
        else None
    )
    tangent_raw = out.get("tangentXY") or out.get("waypointStartXY")
    tangent_xy = (
        (float(tangent_raw[0]), float(tangent_raw[1]))
        if isinstance(tangent_raw, (tuple, list)) and len(tangent_raw) >= 2
        else None
    )
    if tangent_xy is None:
        tangent_xy = _marker_xy(marker_rows, "tangent")
    waypoint_start_xy = (
        (float(out["waypointStartXY"][0]), float(out["waypointStartXY"][1]))
        if isinstance(out.get("waypointStartXY"), (tuple, list)) and len(out["waypointStartXY"]) >= 2
        else tangent_xy
    )
    waypoint_end_xy = (
        (float(out["waypointEndXY"][0]), float(out["waypointEndXY"][1]))
        if isinstance(out.get("waypointEndXY"), (tuple, list)) and len(out["waypointEndXY"]) >= 2
        else target_face_xy
    )
    reference_xy = waypoint_start_xy or tangent_xy or target_face_xy
    if len(centerline_xy) >= 2:
        if deployment_direction_xy:
            centerline_xy = _orient_polyline_xy_to_direction(
                centerline_xy,
                direction_xy=deployment_direction_xy,
            )
        else:
            centerline_xy = _orient_polyline_xy(centerline_xy, reference_xy=reference_xy)
    piece_width_m = float(_piece_width_m(piece))
    overlay_width_m = float(overlay.get("maxWidthM", 0.0) or overlay.get("widthM", 0.0) or 0.0)
    width_ref_m = float(piece_width_m if piece_width_m > 0.0 else overlay_width_m)
    if width_ref_m <= 0.0:
        width_ref_m = float(overlay_width_m)
    initial_sep_hint_m = _to_float(out.get("sepCandM")) or _to_float(out.get("dbSepM")) or 0.0
    db_row = _cached_next_collab_resolved_db_row(
        planner,
        float(width_ref_m),
        float(initial_sep_hint_m),
        runtime_cfg=runtime_cfg,
    ) or planner._covering_db_row(width_ref_m) or {}
    db_sep_m = float(db_row.get("sep", out.get("dbSepM", 0.0)) or 0.0)
    if db_sep_m <= 0.0:
        db_sep_m = max(30.0, float(width_ref_m) * 0.5)
    manual_line_base_fov_deg = _runtime_line_manual_base_fov_deg(runtime_cfg)
    manual_line_fov_deg = _runtime_line_manual_fov_deg(runtime_cfg)
    resolved_base_fov_deg = _clamp_line_fov_deg(
        manual_line_base_fov_deg or db_row.get("fov", out.get("resolvedFovDeg", 0.0)),
        MIN_LINE_FOV_DEG,
    )
    if resolved_base_fov_deg <= 0.0:
        resolved_base_fov_deg = _clamp_line_fov_deg(db_row.get("fov", 0.0), MIN_LINE_FOV_DEG)
    resolved_fov_deg = _clamp_line_fov_deg(
        manual_line_fov_deg
        or apply_runtime_camera_adjusted_fov_deg(
            resolved_base_fov_deg,
            runtime_cfg,
            minimum_fov_deg=MIN_LINE_FOV_DEG,
            context="NEXTCOLLAB LINE RESOLVED",
        )
        or 0.0,
        resolved_base_fov_deg,
    )
    route_wp_spacing_m = _runtime_line_route_wp_spacing_m(runtime_cfg)

    route_xy = out.get("routeXY") if isinstance(out.get("routeXY"), list) else []
    route_origin_xy = (
        (float(route_xy[0][0]), float(route_xy[0][1]))
        if route_xy and isinstance(route_xy[0], (tuple, list)) and len(route_xy[0]) >= 2
        else None
    )
    aircraft_entry_xy = _first_xy_from_aircraft_row(aircraft_row)
    origin_xy = aircraft_entry_xy or route_origin_xy
    entry_t_prime_xy = (
        (float(out["entryTPrimeXY"][0]), float(out["entryTPrimeXY"][1]))
        if isinstance(out.get("entryTPrimeXY"), (tuple, list)) and len(out["entryTPrimeXY"]) >= 2
        else None
    )
    sep_cand_m = _to_float(out.get("sepCandM"))
    explicit_route_offset_sep_m = _to_float(out.get("lineRouteOffsetSepM"))
    # sepCandM is an ingress/T' clearance candidate. Keep route offset tied to
    # the sweep DB separation so one aircraft's entry geometry cannot push the
    # LINE anchors far away from the corridor.
    route_offset_sep_m = _route_offset_sep_for_fov(
        planner,
        resolved_base_fov_deg or resolved_fov_deg,
        (
            explicit_route_offset_sep_m
            or _to_float(out.get("dbSepM"))
            or db_sep_m
            or 0.0
        ),
        runtime_cfg=runtime_cfg,
    )
    corridor_sweeps_xy: List[List[Tuple[float, float]]] = []
    corridor_sweep_items: List[Dict[str, Any]] = []
    first_anchor_xy: Tuple[float, float] | None = None
    line_offset_plan: _LineOffsetPlan | None = None
    first_anchor_clearance_m: float | None = None
    active_db_row = dict(forced_shared_row) if forced_shared_row is not None else (dict(db_row) if isinstance(db_row, dict) else {})

    def _db_row_signature(row: Dict[str, Any]) -> tuple[float, float, float]:
        return (
            round(float(row.get("width", 0.0) or 0.0), 6),
            round(float(row.get("sep", 0.0) or 0.0), 6),
            round(float(row.get("fov", 0.0) or 0.0), 6),
        )

    perf_attempt_count = 0
    perf_centerline_sweep_ms = 0.0
    perf_corridor_sweep_ms = 0.0
    perf_tprime_ms = 0.0
    perf_centerline_used = 0
    perf_corridor_used = 0
    for attempt in range(1 if forced_shared_row is not None else 3):
        perf_attempt_count += 1
        active_db_sep_m = float(active_db_row.get("sep", db_sep_m) or db_sep_m or 0.0)
        if active_db_sep_m <= 0.0:
            active_db_sep_m = max(30.0, float(width_ref_m) * 0.5)
        active_base_fov_deg = _clamp_line_fov_deg(
            manual_line_base_fov_deg
            or active_db_row.get("fov", resolved_base_fov_deg)
            or resolved_base_fov_deg
            or 0.0,
            resolved_base_fov_deg,
        )
        active_fov_deg = _clamp_line_fov_deg(
            manual_line_fov_deg
            or apply_runtime_camera_adjusted_fov_deg(
                active_base_fov_deg,
                runtime_cfg,
                minimum_fov_deg=MIN_LINE_FOV_DEG,
                context="NEXTCOLLAB LINE ACTIVE_DB",
            )
            or 0.0,
            active_base_fov_deg,
        )
        if active_fov_deg <= 0.0:
            active_fov_deg = _clamp_line_fov_deg(resolved_fov_deg)
        active_raw_sweep_spacing_m = _line_sweep_spacing_m(
            separation_m=float(active_db_sep_m),
            fov_deg=float(active_fov_deg or 1.0),
            runtime_cfg=runtime_cfg,
        )
        active_route_offset_sep_m = _route_offset_sep_for_fov(
            planner,
            active_base_fov_deg or active_fov_deg,
            (
                explicit_route_offset_sep_m
                or route_offset_sep_m
                or active_db_sep_m
            ),
            runtime_cfg=runtime_cfg,
        )
        active_route_offset_m = max(
            float(active_route_offset_sep_m) * float(_runtime_line_route_offset_scale(runtime_cfg)),
            1.0,
        )

        perf_stage_started = replan_perf.start_timer()
        (
            corridor_sweeps_xy,
            corridor_sweep_items,
            first_anchor_xy,
            line_offset_plan,
            first_anchor_clearance_m,
        ) = _cached_centerline_corridor_sweep_lines_xy(
            planner,
            piece,
            sweep_spacing_m=float(active_raw_sweep_spacing_m),
            route_offset_m=float(active_route_offset_m),
            origin_xy=origin_xy,
            deployment_direction_xy=deployment_direction_xy,
            db_sep_m=float(active_db_sep_m),
            aircraft_row=aircraft_row,
        )
        if perf_stage_started is not None:
            perf_centerline_sweep_ms += (time.perf_counter() - perf_stage_started) * 1000.0
        if not corridor_sweeps_xy:
            perf_stage_started = replan_perf.start_timer()
            (
                corridor_sweeps_xy,
                corridor_sweep_items,
                first_anchor_xy,
                line_offset_plan,
                first_anchor_clearance_m,
            ) = _corridor_sweep_lines_xy(
                planner,
                piece,
                overlay,
                sweep_spacing_m=float(active_raw_sweep_spacing_m),
                route_offset_m=float(active_route_offset_m),
                origin_xy=origin_xy,
                deployment_direction_xy=deployment_direction_xy,
                db_sep_m=float(active_db_sep_m),
                aircraft_row=aircraft_row,
            )
            if perf_stage_started is not None:
                perf_corridor_sweep_ms += (time.perf_counter() - perf_stage_started) * 1000.0
                perf_corridor_used += 1
        else:
            perf_centerline_used += 1
        entry_line_endpoints_xy = _line_entry_endpoints_xy(corridor_sweeps_xy)
        entry_target_xy = (
            (float(first_anchor_xy[0]), float(first_anchor_xy[1]))
            if isinstance(first_anchor_xy, tuple)
            else _midpoint_xy(entry_line_endpoints_xy or [])
        )
        if entry_target_xy is None:
            entry_target_xy = waypoint_end_xy or target_face_xy or target_xy

        perf_stage_started = replan_perf.start_timer()
        computed_entry_t_prime_xy, computed_sep_cand_m, next_db_row = _cached_resolve_line_entry_tprime_state(
            planner,
            part_width_m=float(width_ref_m),
            tangent_xy=tangent_xy,
            entry_target_xy=entry_target_xy,
            entry_line_endpoints_xy=entry_line_endpoints_xy,
            runtime_cfg=runtime_cfg,
        )
        if perf_stage_started is not None:
            perf_tprime_ms += (time.perf_counter() - perf_stage_started) * 1000.0
        if computed_entry_t_prime_xy is not None:
            entry_t_prime_xy = computed_entry_t_prime_xy
            waypoint_start_xy = entry_t_prime_xy
        if computed_sep_cand_m is not None:
            sep_cand_m = float(computed_sep_cand_m)
        if (
            first_anchor_clearance_m is not None
            and float(first_anchor_clearance_m) > float(active_db_sep_m) + 1e-6
        ):
            anchor_cover_row = _line_db_row_covering_anchor_sep(
                planner,
                width_m=float(width_ref_m),
                required_sep_m=float(first_anchor_clearance_m),
            )
            if isinstance(anchor_cover_row, dict):
                next_db_row = anchor_cover_row

        db_sep_m = float(active_db_sep_m)
        resolved_base_fov_deg = float(active_base_fov_deg)
        resolved_fov_deg = float(active_fov_deg)
        raw_sweep_spacing_m = float(active_raw_sweep_spacing_m)
        route_offset_sep_m = float(active_route_offset_sep_m)
        route_offset_m = float(active_route_offset_m)
        if line_offset_plan is not None and float(line_offset_plan.offset_m) > 0.0:
            route_offset_m = float(line_offset_plan.offset_m)

        if forced_shared_row is not None:
            break
        if not isinstance(next_db_row, dict):
            break
        if _db_row_signature(next_db_row) == _db_row_signature(active_db_row):
            active_db_row = dict(next_db_row)
            break
        if attempt >= 2:
            break
        active_db_row = dict(next_db_row)

    final_db_row = dict(forced_shared_row) if forced_shared_row is not None else (active_db_row if isinstance(active_db_row, dict) else {})
    final_base_fov_deg = _clamp_line_fov_deg(
        manual_line_base_fov_deg
        or final_db_row.get("fov", resolved_base_fov_deg)
        or resolved_base_fov_deg
        or 0.0,
        resolved_base_fov_deg,
    )
    resolved_fov_deg = _clamp_line_fov_deg(
        manual_line_fov_deg
        or apply_runtime_camera_adjusted_fov_deg(
            final_base_fov_deg,
            runtime_cfg,
            minimum_fov_deg=MIN_LINE_FOV_DEG,
            context="NEXTCOLLAB LINE FINAL",
        )
        or 0.0,
        final_base_fov_deg,
    )
    out["resolvedBaseFovDeg"] = float(final_base_fov_deg)
    out["resolvedFovDeg"] = float(resolved_fov_deg)
    out["dbSepM"] = float(final_db_row.get("sep", out.get("dbSepM", db_sep_m)) or db_sep_m or 0.0)
    out["dbWidthM"] = float(final_db_row.get("width", out.get("dbWidthM", width_ref_m)) or width_ref_m or 0.0)
    if forced_shared_row is not None:
        sep_cand_m = float(final_db_row.get("sep", sep_cand_m or 0.0) or 0.0)
    if (
        forced_shared_row is not None
        or out.get("resolvedVelMps") is None
        or float(out.get("resolvedVelMps", 0.0) or 0.0) <= 0.0
    ):
        # A forced shared row may change the FOV, so the capture speed must
        # be recomputed even when the incoming row already carries one.
        out["resolvedVelMps"] = _capture_line_speed_kmh(
            resolved_fov_deg,
            corridor_sweeps_xy or out.get("sweepLineListXY"),
            runtime_cfg=runtime_cfg,
            fallback_kmh=float(final_db_row.get("vel", 0.0) or 0.0),
        )

    if corridor_sweeps_xy:
        out["sweepLineListXY"] = corridor_sweeps_xy
    elif not isinstance(out.get("sweepLineListXY"), list) or not out.get("sweepLineListXY"):
        if len(centerline_xy) >= 2:
            out["sweepLineListXY"] = [centerline_xy]
    else:
        out["sweepLineListXY"] = [
            [(float(x), float(y)) for x, y in line_xy]
            for line_xy in out.get("sweepLineListXY") or []
            if isinstance(line_xy, list) and len(line_xy) >= 2
        ]

    if target_face_xy is not None:
        out["targetFaceXY"] = target_face_xy
    if target_xy is not None:
        out["targetXY"] = (float(target_xy[0]), float(target_xy[1]))
    elif target_face_xy is not None:
        out["targetXY"] = target_face_xy
    if tangent_xy is not None:
        out["tangentXY"] = tangent_xy
    if waypoint_start_xy is not None:
        out["waypointStartXY"] = waypoint_start_xy
    if first_anchor_xy is not None:
        out["waypointEndXY"] = (float(first_anchor_xy[0]), float(first_anchor_xy[1]))
    elif waypoint_end_xy is not None:
        out["waypointEndXY"] = waypoint_end_xy
    if entry_line_endpoints_xy is not None:
        out["entryLineEndpointsXY"] = [
            (float(entry_line_endpoints_xy[0][0]), float(entry_line_endpoints_xy[0][1])),
            (float(entry_line_endpoints_xy[1][0]), float(entry_line_endpoints_xy[1][1])),
        ]

    out["source"] = str(out.get("source") or "line_make_path")
    out["partWidthM"] = float(width_ref_m)
    out["pieceWidthM"] = float(piece_width_m)
    out["overlayWidthM"] = float(overlay_width_m)
    out["dbWidthM"] = float(out.get("dbWidthM", db_row.get("width", width_ref_m)) or 0.0)
    out["dbSepM"] = float(out.get("dbSepM", db_sep_m) or 0.0)
    out["lineSweepSpacingM"] = float(raw_sweep_spacing_m)
    out["lineRouteOffsetSepM"] = float(route_offset_sep_m)
    out["lineRouteOffsetM"] = float(route_offset_m)
    out["lineRouteWpSpacingM"] = float(route_wp_spacing_m)
    if isinstance(aircraft_row, dict):
        turn_confidence = _first_float(aircraft_row.get("turnDirectionConfidence"))
        resolved_turn_sign = 0
        try:
            resolved_turn_sign = int(aircraft_row.get("turnSign") or 0)
        except Exception:
            resolved_turn_sign = 0
        if turn_confidence is not None:
            out["lineTurnDirectionConfidence"] = max(
                0.0,
                min(1.0, float(turn_confidence)),
            )
        out["lineTurnDirectionSign"] = int(resolved_turn_sign)
        out["lineTurnDirectionLocked"] = bool(
            turn_confidence is not None
            and float(turn_confidence) >= float(LINE_TURN_HARD_CONFIDENCE)
            and resolved_turn_sign != 0
        )
        entry_heading_deg = _first_float(aircraft_row.get("headingDeg"))
        planning_heading_deg = _first_float(aircraft_row.get("planningHeadingDeg"))
        if entry_heading_deg is not None:
            out["lineEntryHeadingDeg"] = float(entry_heading_deg) % 360.0
        if planning_heading_deg is not None:
            out["linePlanningHeadingDeg"] = float(planning_heading_deg) % 360.0
        out["lineTurnTrendSource"] = str(aircraft_row.get("turnTrendSource") or "")
        out["lineTurnTrendSampleCount"] = int(
            _to_int(aircraft_row.get("turnTrendSampleCount")) or 0
        )
        out["lineEntryHeadingSource"] = str(aircraft_row.get("headingSource") or "")
        actual_entry_xy = _actual_xy_from_aircraft_row(aircraft_row)
        planning_entry_xy = _first_xy_from_aircraft_row(aircraft_row)
        if actual_entry_xy is not None:
            out["lineEntryCurrentXY"] = (
                float(actual_entry_xy[0]),
                float(actual_entry_xy[1]),
            )
        if planning_entry_xy is not None:
            out["linePredictedEntryXY"] = (
                float(planning_entry_xy[0]),
                float(planning_entry_xy[1]),
            )
    if line_offset_plan is not None:
        out["lineRouteOffsetStartSide"] = float(line_offset_plan.start_side)
        out["lineRouteOffsetEndSide"] = float(line_offset_plan.end_side)
        out["lineRouteOffsetPlanScore"] = float(line_offset_plan.score)
        out["lineRouteOffsetFirstTurnSign"] = int(line_offset_plan.first_turn_sign)
        out["lineRouteOffsetTurnLockRelaxed"] = bool(
            out.get("lineTurnDirectionLocked")
            and int(line_offset_plan.first_turn_sign) != int(out.get("lineTurnDirectionSign") or 0)
        )
        if line_offset_plan.dubins_length_m is not None:
            out["lineRouteOffsetDubinsLengthM"] = float(line_offset_plan.dubins_length_m)
        if first_anchor_clearance_m is not None:
            out["lineRouteFirstAnchorClearanceM"] = float(first_anchor_clearance_m)
    out["resolvedBaseFovDeg"] = float(final_base_fov_deg)
    out["resolvedFovDeg"] = float(resolved_fov_deg)
    if out.get("resolvedVelMps") is None:
        out["resolvedVelMps"] = _capture_line_speed_kmh(
            resolved_fov_deg,
            out.get("sweepLineListXY"),
            runtime_cfg=runtime_cfg,
            fallback_kmh=float(db_row.get("vel", 0.0) or 0.0),
        )
    out["bearingDeg"] = float(out.get("bearingDeg", overlay.get("bearingDeg", 0.0)) or 0.0)
    if out.get("horizonSec") is None:
        out["horizonSec"] = 0.0
    if out.get("branch") is None:
        out["branch"] = "direct"
    if sep_cand_m is not None:
        out["sepCandM"] = float(sep_cand_m)
    if tangent_xy is not None and waypoint_start_xy is not None:
        out["entryTPrimeXY"] = (
            entry_t_prime_xy
            if entry_t_prime_xy is not None
            else ((float(waypoint_start_xy[0]), float(waypoint_start_xy[1])) if _distance_xy(tangent_xy, waypoint_start_xy) > 1e-6 else None)
        )
        if out["entryTPrimeXY"] is not None:
            out["startLabel"] = "T'"
    if len(centerline_xy) >= 2:
        out["centerLineXY"] = list(centerline_xy)
    if deployment_direction_xy:
        out["lineDeploymentDirectionLocked"] = True
        out["lineDeploymentDirectionXY"] = [
            (float(x), float(y))
            for x, y in deployment_direction_xy
        ]
    line_sweep_items = list(corridor_sweep_items or [])
    if not line_sweep_items:
        line_sweep_items = _build_line_sweep_items(
            out.get("sweepLineListXY") or [],
            reference_xy=reference_xy,
            offset_m=float(route_offset_m),
            route_line_xy=centerline_xy,
            offset_plan=line_offset_plan,
            direction_locked=bool(deployment_direction_xy),
        )
    perf_route_selection_started = replan_perf.start_timer()
    if line_sweep_items:
        if len(centerline_xy) >= 2:
            route_points_xy = _resample_polyline_xy(
                centerline_xy,
                spacing_m=float(route_wp_spacing_m),
            )
            sweep_midpoints_xy: List[Tuple[float, float]] = []
            for item in line_sweep_items:
                sweep_xy = item.get("sweepXY") if isinstance(item, dict) else []
                midpoint_xy = _midpoint_xy(sweep_xy if isinstance(sweep_xy, list) else [])
                sweep_midpoints_xy.append(midpoint_xy or (0.0, 0.0))
            selected_indices = _select_route_sweep_indices(route_points_xy, sweep_midpoints_xy)
            if selected_indices:
                selected_items: List[Dict[str, Any]] = []
                for idx in selected_indices:
                    if idx < 0 or idx >= len(line_sweep_items):
                        continue
                    item = dict(line_sweep_items[idx])
                    item["sweepIndex"] = int(idx)
                    selected_items.append(item)
                out["lineSweepItemsXY"] = selected_items
            else:
                for idx, item in enumerate(line_sweep_items):
                    item["sweepIndex"] = int(idx)
                out["lineSweepItemsXY"] = line_sweep_items
        else:
            for idx, item in enumerate(line_sweep_items):
                item["sweepIndex"] = int(idx)
            out["lineSweepItemsXY"] = line_sweep_items
    replan_perf.add_elapsed(
        "mission_planning.next_collab_line.augment.route_selection",
        perf_route_selection_started,
        pieceIndex=perf_piece_idx,
        sweepItems=len(line_sweep_items or []),
    )
    replan_perf.add(
        "mission_planning.next_collab_line.augment.substeps",
        elapsed_ms=0.0,
        centerlineSweepMs=perf_centerline_sweep_ms,
        corridorSweepMs=perf_corridor_sweep_ms,
        tprimeMs=perf_tprime_ms,
        attempts=perf_attempt_count,
        centerlineUsed=perf_centerline_used,
        corridorUsed=perf_corridor_used,
    )
    replan_perf.add_elapsed(
        "mission_planning.next_collab_line.augment.total",
        perf_total_started,
        pieceIndex=perf_piece_idx,
        forcedShared=bool(forced_shared_row is not None),
        sweepLines=len(out.get("sweepLineListXY") or []),
        lineSweepItems=len(out.get("lineSweepItemsXY") or []),
    )
    return out


def _shared_line_db_row_from_expected_rows(
    expected_rows: Sequence[Dict[str, Any]],
    *,
    runtime_cfg: Dict[str, Any] | None = None,
) -> Dict[str, Any] | None:
    candidates: List[Dict[str, Any]] = []
    required_sep_m = 0.0
    required_width_m = 0.0
    for row in expected_rows:
        if not isinstance(row, dict):
            continue
        spacing_m = _to_float(row.get("lineSweepSpacingM"))
        fov_deg = _to_float(row.get("resolvedFovDeg"))
        sep_m = _to_float(row.get("dbSepM"))
        width_m = _to_float(row.get("dbWidthM")) or _to_float(row.get("partWidthM"))
        sep_cand_m = _to_float(row.get("sepCandM"))
        part_width_m = _to_float(row.get("partWidthM"))
        vel_mps = _to_float(row.get("resolvedVelMps"))
        if spacing_m is None or spacing_m <= 0.0:
            continue
        if fov_deg is None or fov_deg <= 0.0:
            continue
        if sep_m is None or sep_m <= 0.0:
            continue
        if sep_cand_m is not None and sep_cand_m > 0.0:
            required_sep_m = max(float(required_sep_m), _db_sep_requirement_m(float(sep_cand_m), runtime_cfg))
        if part_width_m is not None and part_width_m > 0.0:
            required_width_m = max(float(required_width_m), float(part_width_m))
        candidates.append(
            {
                "spacing": float(spacing_m),
                "fov": float(fov_deg),
                "sep": float(sep_m),
                "width": float(width_m or 0.0),
                "vel": float(vel_mps or 0.0),
            }
        )
    if not candidates:
        return None
    covering_candidates = [
        item for item in candidates
        if float(item["sep"]) + 1e-6 >= float(required_sep_m)
        and float(item["width"]) + 1e-6 >= float(required_width_m)
    ]
    pool = covering_candidates or candidates
    best = min(
        pool,
        key=lambda item: (
            float(item["spacing"]),
            float(item["sep"]),
            -float(item["fov"]),
        ),
    )
    return {
        "width": float(best["width"]),
        "sep": float(best["sep"]),
        "fov": float(best["fov"]),
        "vel": float(best["vel"]),
    }


def _shared_reaugment_fast_enabled() -> bool:
    raw = str(os.environ.get("REPLAN_NEXT_COLLAB_SHARED_REAUGMENT_FAST", "0") or "").strip().lower()
    return raw not in {"0", "false", "off", "no"}


def _line_float_close(left: Any, right: Any, *, abs_tol: float = 1e-3) -> bool:
    left_f = _to_float(left)
    right_f = _to_float(right)
    if left_f is None or right_f is None:
        return False
    return abs(float(left_f) - float(right_f)) <= float(abs_tol)


def _apply_shared_line_db_row_fast(
    planner: _HeadlessLinePlanner,
    row: Dict[str, Any],
    shared_db_row: Dict[str, Any],
    *,
    runtime_cfg: Dict[str, Any] | None = None,
) -> Dict[str, Any] | None:
    if not _shared_reaugment_fast_enabled():
        return None
    if not isinstance(row, dict) or not isinstance(shared_db_row, dict):
        return None

    manual_line_base_fov_deg = _runtime_line_manual_base_fov_deg(runtime_cfg)
    manual_line_fov_deg = _runtime_line_manual_fov_deg(runtime_cfg)
    existing_base_fov_deg = _to_float(row.get("resolvedBaseFovDeg")) or _to_float(row.get("resolvedFovDeg")) or 0.0
    final_base_fov_deg = _clamp_line_fov_deg(
        manual_line_base_fov_deg
        or shared_db_row.get("fov", existing_base_fov_deg)
        or existing_base_fov_deg
        or 0.0,
        existing_base_fov_deg,
    )
    final_fov_deg = _clamp_line_fov_deg(
        manual_line_fov_deg
        or apply_runtime_camera_adjusted_fov_deg(
            final_base_fov_deg,
            runtime_cfg,
            minimum_fov_deg=MIN_LINE_FOV_DEG,
            context="NEXTCOLLAB LINE FAST_SHARED",
        )
        or 0.0,
        final_base_fov_deg,
    )
    shared_sep_m = float(shared_db_row.get("sep", 0.0) or 0.0)
    shared_width_m = float(shared_db_row.get("width", 0.0) or 0.0)
    if shared_sep_m <= 0.0 or shared_width_m <= 0.0 or final_fov_deg <= 0.0:
        return None
    expected_spacing_m = _line_sweep_spacing_m(
        separation_m=float(shared_sep_m),
        fov_deg=float(final_fov_deg),
        runtime_cfg=runtime_cfg,
    )
    explicit_route_offset_sep_m = _to_float(row.get("lineRouteOffsetSepM"))
    expected_route_offset_sep_m = _route_offset_sep_for_fov(
        planner,
        final_base_fov_deg or final_fov_deg,
        explicit_route_offset_sep_m or shared_sep_m,
        runtime_cfg=runtime_cfg,
    )
    expected_route_offset_m = max(
        float(expected_route_offset_sep_m) * float(_runtime_line_route_offset_scale(runtime_cfg)),
        1.0,
    )

    if not _line_float_close(row.get("dbSepM"), shared_sep_m, abs_tol=1e-3):
        return None
    if not _line_float_close(row.get("dbWidthM"), shared_width_m, abs_tol=1e-3):
        return None
    if not _line_float_close(row.get("resolvedBaseFovDeg"), final_base_fov_deg, abs_tol=1e-3):
        return None
    if not _line_float_close(row.get("resolvedFovDeg"), final_fov_deg, abs_tol=1e-3):
        return None
    if not _line_float_close(row.get("lineSweepSpacingM"), expected_spacing_m, abs_tol=1e-3):
        return None
    if not _line_float_close(row.get("lineRouteOffsetSepM"), expected_route_offset_sep_m, abs_tol=1e-3):
        return None
    existing_offset_m = _to_float(row.get("lineRouteOffsetM"))
    if existing_offset_m is not None and existing_offset_m + 1e-6 < expected_route_offset_m:
        return None

    out = dict(row)
    out["dbSepM"] = float(shared_sep_m)
    out["dbWidthM"] = float(shared_width_m)
    out["resolvedBaseFovDeg"] = float(final_base_fov_deg)
    out["resolvedFovDeg"] = float(final_fov_deg)
    out["lineSweepSpacingM"] = float(expected_spacing_m)
    out["lineRouteOffsetSepM"] = float(expected_route_offset_sep_m)
    out["sepCandM"] = float(shared_sep_m)
    out["resolvedVelMps"] = _capture_line_speed_kmh(
        final_fov_deg,
        out.get("sweepLineListXY"),
        runtime_cfg=runtime_cfg,
        fallback_kmh=float(shared_db_row.get("vel", 0.0) or 0.0),
    )
    return out


def _fast_mid_line_overlay_geometry(
    planner: _HeadlessLinePlanner,
    coords_xy: Sequence[Tuple[float, float]],
    bearing_deg: float,
) -> Optional[Dict[str, Any]]:
    if len(coords_xy) < 3:
        return None

    poly = planner._largest_polygon_xy(Polygon(coords_xy).buffer(0))
    if poly is None or poly.is_empty:
        return None
    polygon_xy = _polygon_exterior_xy(poly)
    if len(polygon_xy) < 3:
        return None

    theta = math.radians(float(bearing_deg) % 360.0)
    ux = math.sin(theta)
    uy = math.cos(theta)
    vx = uy
    vy = -ux

    s_vals = [(float(x) * ux) + (float(y) * uy) for (x, y) in polygon_xy]
    t_vals = [(float(x) * vx) + (float(y) * vy) for (x, y) in polygon_xy]
    if not s_vals or not t_vals:
        return None

    min_s = float(min(s_vals))
    max_s = float(max(s_vals))
    min_t = float(min(t_vals))
    max_t = float(max(t_vals))

    def _from_st(s_val: float, t_val: float) -> Tuple[float, float]:
        return (
            (float(s_val) * ux) + (float(t_val) * vx),
            (float(s_val) * uy) + (float(t_val) * vy),
        )

    mid_t = 0.5 * (min_t + max_t)
    mid_s = 0.5 * (min_s + max_s)
    overlay: Dict[str, Any] = {
        "boxXY": [
            _from_st(min_s, min_t),
            _from_st(max_s, min_t),
            _from_st(max_s, max_t),
            _from_st(min_s, max_t),
        ],
        "midLineXY": [
            _from_st(min_s, mid_t),
            _from_st(max_s, mid_t),
        ],
        "centerXY": _from_st(mid_s, mid_t),
        "lengthM": max(0.0, max_s - min_s),
        "widthM": max(0.0, max_t - min_t),
    }
    length_m = max(0.0, max_s - min_s)
    sample_count = max(25, min(161, int(math.ceil(length_m / 25.0)) + 1))

    best_total_m = -1.0
    best_geometry: Optional[Dict[str, Any]] = None
    for idx in range(sample_count):
        ratio = (float(idx) / float(sample_count - 1)) if sample_count > 1 else 0.5
        s_probe = min_s + ((max_s - min_s) * ratio)
        guide_center_xy = _from_st(s_probe, mid_t)
        width_line_xy = _scanline_polygon_segment_xy(
            polygon_xy,
            guide_center_xy,
            (float(vx), float(vy)),
            reference_xy=guide_center_xy,
        )
        if not isinstance(width_line_xy, list) or len(width_line_xy) < 2:
            continue

        start_xy = width_line_xy[0]
        end_xy = width_line_xy[-1]
        start_t = (float(start_xy[0]) * vx) + (float(start_xy[1]) * vy)
        end_t = (float(end_xy[0]) * vx) + (float(end_xy[1]) * vy)
        seg_min_t = min(start_t, end_t)
        seg_max_t = max(start_t, end_t)
        if (seg_min_t - 1.0) > mid_t or (seg_max_t + 1.0) < mid_t:
            continue

        left_m = max(0.0, mid_t - seg_min_t)
        right_m = max(0.0, seg_max_t - mid_t)
        total_m = left_m + right_m
        if total_m <= best_total_m:
            continue
        best_total_m = float(total_m)
        best_geometry = {
            "maxWidthLineXY": [
                _from_st(s_probe, seg_min_t),
                _from_st(s_probe, seg_max_t),
            ],
            "maxWidthCenterXY": _from_st(s_probe, mid_t),
            "maxWidthLeftM": float(left_m),
            "maxWidthRightM": float(right_m),
            "maxWidthM": float(total_m),
        }

    if best_geometry is not None:
        overlay.update(best_geometry)

    max_width_m = float(overlay.get("maxWidthM", 0.0) or 0.0)
    db_widths = planner._fov_db_widths_m()
    db_cover_width_m = planner._covering_db_width_m(max_width_m) if db_widths and max_width_m > 0.0 else None
    overlay["dbMaxWidthM"] = float(db_widths[-1]) if db_widths else 0.0
    overlay["dbCoverWidthM"] = float(db_cover_width_m) if db_cover_width_m is not None else 0.0
    overlay["midLineRequired"] = (
        False if (max_width_m > 0.0 and db_cover_width_m is not None) else True
    )

    if not bool(overlay.get("midLineRequired", True)):
        return overlay

    split_parts: List[Dict[str, Any]] = []
    try:
        split_geom = geom_split(poly, LineString(overlay["midLineXY"]))
        part_polys = [geom for geom in getattr(split_geom, "geoms", []) if isinstance(geom, Polygon) and not geom.is_empty]
    except Exception:
        part_polys = []
    if len(part_polys) >= 2:
        part_rows: List[Tuple[float, Dict[str, Any]]] = []
        for part_poly in part_polys:
            coords_part = _dedupe_scan_points_xy(
                [(float(x), float(y)) for (x, y) in list(part_poly.exterior.coords)[:-1]],
                min_dist_m=1.0,
            )
            if len(coords_part) < 3:
                continue
            center = part_poly.representative_point()
            center_t = (float(center.x) * vx) + (float(center.y) * vy)
            part_rows.append(
                (
                    float(center_t),
                    {
                        "polygonXY": coords_part,
                        "centroidXY": (float(center.x), float(center.y)),
                    },
                )
            )
        part_rows.sort(key=lambda item: float(item[0]))
        for part_idx, (_center_t, part_row) in enumerate(part_rows):
            prefix = "A" if part_idx == 0 else "B"
            labeled_points = [
                {"label": f"{prefix}{idx + 1}", "xy": point_xy}
                for idx, point_xy in enumerate(part_row["polygonXY"])
            ]
            split_parts.append(
                {
                    "name": prefix,
                    "polygonXY": part_row["polygonXY"],
                    "centroidXY": part_row["centroidXY"],
                    "pointLabels": labeled_points,
                }
            )
    if split_parts:
        overlay["splitParts"] = split_parts
    return overlay


def _bind_fast_mid_line_methods(planner: _HeadlessLinePlanner) -> None:
    original_geometry = planner._mid_line_overlay_geometry

    def _mid_line_overlay_geometry_fast(
        self: _HeadlessLinePlanner,
        coords_xy: Sequence[Tuple[float, float]],
        bearing_deg: float,
    ) -> Optional[Dict[str, Any]]:
        try:
            result = _fast_mid_line_overlay_geometry(self, coords_xy, bearing_deg)
            if isinstance(result, dict):
                return result
        except Exception:
            pass
        return original_geometry(coords_xy, bearing_deg)

    planner._mid_line_overlay_geometry = MethodType(_mid_line_overlay_geometry_fast, planner)


def _forced_line_visibility_segment(
    planner: _HeadlessLinePlanner,
    *,
    aircraft_id: int,
    origin_xy: Tuple[float, float],
    bearing_deg: float,
    target_xy: Tuple[float, float],
    branch: str,
) -> Dict[str, Any] | None:
    """Find a tangent while continuing the observed Line-entry turn branch."""

    radius_m = float(planner._default_turn_radius_m(int(aircraft_id)))
    speed_mps = float(planner._aircraft_turn_speed_mps(int(aircraft_id)))
    if radius_m <= 1.0 or speed_mps <= 1.0:
        return None
    horizon_step_s = max(0.5, float(TURN_PREVIEW_HORIZON_S))
    turn_period_s = (2.0 * math.pi * float(radius_m)) / float(speed_mps)
    max_steps = max(1, int(math.ceil(float(turn_period_s) / horizon_step_s)))
    selected_branch = "L" if str(branch).upper() == "L" else "R"

    def _point_at(horizon_s: float) -> Tuple[float, float]:
        left_xy, right_xy = planner._turn_prediction_points_xy(
            origin_xy,
            bearing_deg,
            speed_mps=float(speed_mps),
            horizon_s=float(horizon_s),
            radius_m=float(radius_m),
        )
        return left_xy if selected_branch == "L" else right_xy

    for step_idx in range(1, max_steps + 1):
        horizon_s = float(step_idx) * horizon_step_s
        candidate_xy = _point_at(horizon_s)
        if not planner._line_avoids_turn_circles(
            candidate_xy,
            target_xy,
            origin_xy,
            bearing_deg,
            radius_m=float(radius_m),
        ):
            continue
        turn_points = [
            _point_at(float(prior_idx) * horizon_step_s)
            for prior_idx in range(1, step_idx + 1)
        ]
        start_xy = candidate_xy
        start_horizon_s = float(horizon_s)
        if step_idx >= 2:
            refined = planner._refine_visibility_start_xy(
                origin_xy,
                bearing_deg,
                target_xy,
                branch=selected_branch,
                min_horizon_s=float(step_idx - 1) * horizon_step_s,
                max_horizon_s=float(horizon_s),
                radius_m=float(radius_m),
                speed_mps=float(speed_mps),
            )
            if isinstance(refined, dict):
                refined_xy = refined.get("startXY")
                if isinstance(refined_xy, (tuple, list)) and len(refined_xy) >= 2:
                    start_xy = (float(refined_xy[0]), float(refined_xy[1]))
                    start_horizon_s = float(refined.get("horizonSec", horizon_s) or horizon_s)
                    turn_points[-1] = start_xy
        return {
            "aircraftID": int(aircraft_id),
            "startXY": (float(start_xy[0]), float(start_xy[1])),
            "endXY": (float(target_xy[0]), float(target_xy[1])),
            "horizonSec": float(start_horizon_s),
            "branch": str(selected_branch),
            "turnPoints": [(float(x), float(y)) for x, y in turn_points],
            "turnRadiusM": float(radius_m),
            "turnSpeedMps": float(speed_mps),
            "lineTurnDirectionLocked": True,
        }

    # Degenerate case (for example target inside the current circle): retain a
    # short continuation arc so the emitted first leg never reverses the live
    # turn, then let the downstream route reconnect from that safe pose.
    fallback_horizon_s = horizon_step_s
    fallback_xy = _point_at(fallback_horizon_s)
    return {
        "aircraftID": int(aircraft_id),
        "startXY": (float(fallback_xy[0]), float(fallback_xy[1])),
        "endXY": (float(target_xy[0]), float(target_xy[1])),
        "horizonSec": float(fallback_horizon_s),
        "branch": str(selected_branch),
        "turnPoints": [(float(fallback_xy[0]), float(fallback_xy[1]))],
        "turnRadiusM": float(radius_m),
        "turnSpeedMps": float(speed_mps),
        "lineTurnDirectionLocked": True,
        "lineTurnDirectionFallback": True,
    }


def _bind_line_turn_direction_methods(planner: _HeadlessLinePlanner) -> None:
    original_find_visibility_segment = planner._find_visibility_segment

    def _find_visibility_segment_line_locked(
        self: _HeadlessLinePlanner,
        aircraft_id: int,
        origin_xy: Tuple[float, float],
        bearing_deg: float,
        target_xy: Tuple[float, float],
    ) -> Optional[Dict[str, Any]]:
        confidence_map = getattr(self, "_aircraft_turn_confidence_overrides", {})
        sign_map = getattr(self, "_aircraft_turn_sign_overrides", {})
        try:
            confidence = float(confidence_map.get(int(aircraft_id), 0.0) or 0.0)
            turn_sign = int(sign_map.get(int(aircraft_id), 0) or 0)
        except Exception:
            confidence = 0.0
            turn_sign = 0
        if confidence >= float(LINE_TURN_HARD_CONFIDENCE) and turn_sign != 0:
            forced = _forced_line_visibility_segment(
                self,
                aircraft_id=int(aircraft_id),
                origin_xy=origin_xy,
                bearing_deg=float(bearing_deg),
                target_xy=target_xy,
                branch="L" if turn_sign > 0 else "R",
            )
            if forced is not None:
                return forced
        return original_find_visibility_segment(
            int(aircraft_id),
            origin_xy,
            float(bearing_deg),
            target_xy,
        )

    planner._find_visibility_segment = MethodType(
        _find_visibility_segment_line_locked,
        planner,
    )


def _bind_scaled_turn_methods(
    planner: _HeadlessLinePlanner,
    *,
    turn_radius_scale: float,
) -> None:
    planner._turn_radius_scale = max(0.1, float(turn_radius_scale))


@contextmanager
def _silent_message_boxes():
    original_warning = QMessageBox.warning
    original_information = QMessageBox.information
    original_critical = QMessageBox.critical
    try:
        QMessageBox.warning = staticmethod(lambda *args, **kwargs: 0)  # type: ignore[assignment]
        QMessageBox.information = staticmethod(lambda *args, **kwargs: 0)  # type: ignore[assignment]
        QMessageBox.critical = staticmethod(lambda *args, **kwargs: 0)  # type: ignore[assignment]
        yield
    finally:
        QMessageBox.warning = original_warning  # type: ignore[assignment]
        QMessageBox.information = original_information  # type: ignore[assignment]
        QMessageBox.critical = original_critical  # type: ignore[assignment]


def _persist_branch_line_reassignment(
    split_result: SplitRunResult,
    mission_payload: Dict[str, Any],
    mode_context: Dict[str, Any],
    *,
    log: Callable[[str], None] | None = None,
) -> None:
    """Persist only the legacy mutable Type-3 entry-LINE reassignment."""
    try:
        package_type = int(mode_context.get("package_type") or 0)
    except Exception:
        package_type = 0
    if package_type == 2:
        if log is not None and getattr(split_result, "branch_ownership", None):
            log("[NEXTCOLLAB][LINE] Type-2 immutable branch ownership preserved.")
        return
    if package_type != 3:
        return
    try:
        package_id = int(mode_context.get("inputMissionPackageID") or 0)
        region_type = int(mission_payload.get("regionType") or 0)
    except Exception:
        return
    if package_id <= 0 or region_type != 7:
        return
    mapping: Dict[int, List[int]] = {}
    for piece in split_result.pieces:
        data = piece.data if isinstance(piece.data, dict) else {}
        branch_index = data.get("branchIndex")
        aircraft_id = piece.assigned_uav
        if branch_index is None or not aircraft_id:
            continue
        owners = mapping.setdefault(int(branch_index), [])
        if int(aircraft_id) not in owners:
            owners.append(int(aircraft_id))
    if len(mapping) < 2:
        return
    try:
        from modules.mission_planning.runtime.state import branch_ownership as store

        stored = store.update_branch_ownership(
            package_id,
            mapping,
            source="next_collab_line_reassign_type3",
        )
    except Exception:
        return
    if log is not None and stored == mapping:
        log(
            "[NEXTCOLLAB][LINE] Type-3 branch ownership re-anchored by position: "
            + ", ".join(
                f"b{branch_index}->UAV{'/'.join(str(a) for a in owners)}"
                for branch_index, owners in sorted(mapping.items())
            )
        )


def run_next_collab_line_plan(
    *,
    target_mission: Dict[str, Any] | None = None,
    mission_detail: Dict[str, Any] | None = None,
    aircraft_entries: List[Dict[str, Any]],
    turn_radius_scale: float | None = None,
    planning_mode: MissionPlanningMode | Dict[str, Any] | None = None,
    log: Callable[[str], None] | None = None,
) -> NextCollabLinePlanResult:
    perf_total_started = replan_perf.start_timer()
    perf_path_row_count = 0
    perf_augment_count = 0
    perf_shared_augment_count = 0
    perf_fallback_path_row_count = 0
    mission_source: Dict[str, Any]
    if isinstance(target_mission, dict):
        mission_source = target_mission
    elif isinstance(mission_detail, dict):
        mission_source = {"missionDetail": mission_detail}
    else:
        raise RuntimeError("next-collab line planner requires a target mission or missionDetail payload.")

    if isinstance(mission_detail, dict) and not isinstance(mission_source.get("missionDetail"), dict):
        mission_source = dict(mission_source)
        mission_source["missionDetail"] = mission_detail

    resolved_mode = planning_mode or resolve_mission_planning_mode(mission_source)
    mode_context = mission_mode_context(mode=resolved_mode)
    runtime_cfg = load_runtime_settings()
    mission_payload, line_specs = _normalize_line_specs(mission_source)
    mission_detail_payload = (
        mission_payload.get("missionDetail")
        if isinstance(mission_payload.get("missionDetail"), dict)
        else {}
    )
    source_line_width_m = _resolve_source_line_width_m(mission_payload, mission_detail_payload, line_specs)
    source_coordinate_list = _resolve_source_coordinate_list(mission_detail_payload, line_specs)
    deployment_coordinate_list = _resolve_line_deployment_coordinate_list(mission_detail_payload)
    deployment_direction_xy = coords_to_xy(deployment_coordinate_list)
    deployment_direction_carried = len(deployment_direction_xy) >= 2
    mission_center_xy = _mission_center_xy(line_specs)
    aircraft_rows = _normalize_aircraft_rows(aircraft_entries, mission_center_xy=mission_center_xy)
    _ensure(bool(aircraft_rows), "next-collab line planner requires at least one aircraft entry.")
    aircraft_row_by_id = {int(row["aircraftID"]): dict(row) for row in aircraft_rows}
    if log is not None:
        if deployment_direction_carried:
            log("[NEXTCOLLAB][LINE] first execution deployment direction carried forward.")
        for row in aircraft_rows:
            confidence = row.get("turnDirectionConfidence")
            if confidence is None:
                continue
            log(
                "[NEXTCOLLAB][LINE][TURN] "
                f"UAV{int(row['aircraftID'])} sign={int(row.get('turnSign') or 0):+d} "
                f"confidence={float(confidence):.2f} "
                f"heading={float(row.get('headingDeg') or 0.0):.1f} "
                f"predictedHeading={float(row.get('planningHeadingDeg') or 0.0):.1f} "
                f"radius={float(row.get('turnRadiusM') or 0.0):.1f}m "
                f"radiusSource={str(row.get('turnRadiusSource') or 'unknown')} "
                f"source={str(row.get('turnTrendSource') or row.get('headingSource') or 'fallback')}"
            )

    planner = _HeadlessLinePlanner()
    planner.hide()
    planner.state.mission_kind = MISSION_LINE
    planner.state.mode = MODE_MISSION_READY
    planner.state.mission_points_xy = [
        point_xy
        for spec in line_specs
        for point_xy in coords_to_xy(spec.get("coordinateList") or [])
    ]
    planner.state.line_width_m = max(float(spec.get("width", 1.0) or 1.0) for spec in line_specs)
    planner.state.uav_ids = [int(row["aircraftID"]) for row in aircraft_rows]
    planner.state.uav_positions_xy = [row["positionXY"] for row in aircraft_rows]
    planner.state.uav_heading_deg = [float(row["headingDeg"]) for row in aircraft_rows]
    planner._aircraft_turn_radius_overrides = {
        int(row["aircraftID"]): float(row["turnRadiusM"])
        for row in aircraft_rows
        if row.get("turnRadiusM") is not None and float(row["turnRadiusM"]) > 1.0
    }
    planner._aircraft_speed_overrides = {
        int(row["aircraftID"]): float(row["speedMps"])
        for row in aircraft_rows
        if row.get("speedMps") is not None and float(row["speedMps"]) > 1.0
    }
    planner._aircraft_turn_sign_overrides = {
        int(row["aircraftID"]): int(row.get("turnSign") or 0)
        for row in aircraft_rows
    }
    planner._aircraft_turn_confidence_overrides = {
        int(row["aircraftID"]): float(row.get("turnDirectionConfidence") or 0.0)
        for row in aircraft_rows
    }
    planner._selected_uav_count = max(1, len(planner.state.uav_ids))
    _bind_scaled_turn_methods(planner, turn_radius_scale=float(turn_radius_scale or 1.2))
    _bind_fast_mid_line_methods(planner)
    _bind_line_turn_direction_methods(planner)

    line_plan_cache_key: str | None = None
    if _line_plan_cache_enabled():
        try:
            line_plan_cache_key = _line_plan_cache_key(
                mission_payload=mission_payload,
                line_specs=line_specs,
                aircraft_rows=aircraft_rows,
                turn_radius_scale=float(turn_radius_scale or 1.2),
                runtime_cfg=runtime_cfg,
                fov_db_rows=_planner_fov_db_signature(planner),
                planning_mode=mode_context,
            )
            cached_plan, cache_source = _cached_line_plan_result(line_plan_cache_key)
            if cached_plan is not None:
                if log is not None:
                    log(
                        "[NEXTCOLLAB][LINE][CACHE] plan_result hit "
                        f"source={cache_source} rows={len(cached_plan.expected_paths)} "
                        f"key={line_plan_cache_key[:12]}"
                    )
                    for cached_line in str(cached_plan.planner_result_text or "").splitlines():
                        if cached_line.strip():
                            log(cached_line)
                # Cached anchor-line plans must still commit their assignment as
                # the sticky ownership so the rest of the set follows.
                _persist_branch_line_reassignment(
                    cached_plan.split_result, mission_payload, mode_context, log=log
                )
                replan_perf.add_elapsed(
                    "mission_planning.next_collab_line.run.total",
                    perf_total_started,
                    pieces=len(cached_plan.split_result.pieces or []),
                    rows=len(cached_plan.expected_paths or []),
                    planCacheHit=True,
                )
                return cached_plan
        except Exception:
            line_plan_cache_key = None
            with _LINE_PLAN_CACHE_LOCK:
                _LINE_PLAN_CACHE_STATS["errors"] += 1

    cmpk_payload = {
        "inputMissionPackageType": int(mode_context.get("package_type") or 0),
        "inputMissionPackageID": int(mode_context.get("inputMissionPackageID") or 0),
        "planningMode": dict(mode_context),
        "availableAircraftList": [{"aircraftID": int(row["aircraftID"])} for row in aircraft_rows],
        "inputMissionList": [mission_payload],
    }
    mrpk_payload = {
        "takeOverInfoList": [
            {
                "aircraftID": int(row["aircraftID"]),
                "coordinate": dict(row["coordinate"]),
                "headingDeg": float(row["headingDeg"]),
            }
            for row in aircraft_rows
        ]
    }
    planner._cmpk_payload = cmpk_payload
    planner._mrpk_payload = mrpk_payload

    with _silent_message_boxes():
        perf_stage_started = replan_perf.start_timer()
        split_result = _run_line_split_pipeline_cached(
            cmpk_payload,
            mrpk_payload,
            list(planner.state.uav_ids),
            planning_mode=resolved_mode,
            log=log,
        )
        replan_perf.add_elapsed(
            "mission_planning.next_collab_line.run.split",
            perf_stage_started,
            aircraft=len(aircraft_rows),
        )
        _ensure(
            split_result is not None and bool(split_result.pieces),
            "next-collab line planner failed to produce split pieces.",
        )
        perf_stage_started = replan_perf.start_timer()
        assign_report = planner._assign_split_result_by_prediction_distance(split_result)
        replan_perf.add_elapsed(
            "mission_planning.next_collab_line.run.assignment",
            perf_stage_started,
            pieces=len(split_result.pieces),
            aircraft=len(aircraft_rows),
        )
        _persist_branch_line_reassignment(split_result, mission_payload, mode_context, log=log)
        planner.state.split_result = split_result

        perf_stage_started = replan_perf.start_timer()
        try:
            reference_bearing_deg, overlays, mid_lines = _cached_mid_line_overlay_bundle(planner, split_result)
        except Exception:
            reference_bearing_deg = _line_reference_bearing_deg(line_specs, split_result.pieces)
            overlays = []
            mid_lines = []
        replan_perf.add_elapsed(
            "mission_planning.next_collab_line.run.mid_overlay",
            perf_stage_started,
            pieces=len(split_result.pieces),
            overlays=len(overlays),
        )
        if not isinstance(reference_bearing_deg, (int, float)):
            reference_bearing_deg = _line_reference_bearing_deg(line_specs, split_result.pieces)

        perf_stage_started = replan_perf.start_timer()
        overlay_by_piece: Dict[int, Dict[str, Any]] = {
            int(row.get("pieceIndex", 0) or 0): dict(row)
            for row in overlays
            if isinstance(row, dict)
        }
        for piece in split_result.pieces:
            piece_idx = int(piece.piece_index or 0)
            if piece_idx in overlay_by_piece:
                continue
            synthetic_overlay = _synthetic_line_overlay(piece, reference_bearing_deg=float(reference_bearing_deg))
            if synthetic_overlay is None:
                continue
            overlay_by_piece[piece_idx] = synthetic_overlay
            overlays.append(synthetic_overlay)
        overlays = sorted(
            (dict(row) for row in overlays if isinstance(row, dict)),
            key=lambda row: int(row.get("pieceIndex", 0) or 0),
        )
        replan_perf.add_elapsed(
            "mission_planning.next_collab_line.run.overlay_fill",
            perf_stage_started,
            overlays=len(overlays),
        )
        planner.state.mid_line_segments = overlays

        expected_rows: List[Dict[str, Any]] = []
        piece_by_index: Dict[int, SplitPiece] = {}
        successful_piece_indices: set[int] = set()
        perf_stage_started = replan_perf.start_timer()
        for piece in sorted(split_result.pieces, key=lambda row: int(row.piece_index or 0)):
            piece_idx = int(piece.piece_index or 0)
            piece_by_index[piece_idx] = piece
            overlay = overlay_by_piece.get(piece_idx)
            if overlay is None:
                continue
            perf_make_started = replan_perf.start_timer()
            path_row = planner._make_path_row(piece, overlay)
            replan_perf.add_elapsed(
                "mission_planning.next_collab_line.run.make_path_row",
                perf_make_started,
                pieceIndex=int(piece.piece_index or 0),
                allowAlt=False,
            )
            perf_path_row_count += 1
            if not isinstance(path_row, dict):
                continue
            row_deployment_direction_xy, row_deployment_coordinate_list = (
                _type2_branch_deployment_reference(piece, line_specs, mode_context)
            )
            if len(row_deployment_direction_xy) < 2:
                row_deployment_direction_xy = list(deployment_direction_xy)
                row_deployment_coordinate_list = copy.deepcopy(deployment_coordinate_list)
            if len(row_deployment_direction_xy) < 2:
                deployment_direction_xy, deployment_coordinate_list = _derive_initial_line_deployment_reference(
                    source_coordinate_list,
                    piece,
                    path_row,
                )
                row_deployment_direction_xy = list(deployment_direction_xy)
                row_deployment_coordinate_list = copy.deepcopy(deployment_coordinate_list)
                if len(deployment_direction_xy) >= 2 and log is not None:
                    log("[NEXTCOLLAB][LINE] first execution deployment direction locked.")
            piece_aircraft_id = int(piece.assigned_uav or path_row.get("aircraftID") or 0)
            perf_augment_started = replan_perf.start_timer()
            augmented_row = _augment_line_path_row(
                planner,
                piece,
                overlay,
                path_row,
                aircraft_row=aircraft_row_by_id.get(piece_aircraft_id),
                deployment_direction_xy=row_deployment_direction_xy,
                runtime_cfg=runtime_cfg,
            )
            if len(row_deployment_coordinate_list) >= 2:
                augmented_row["sourceCoordinateList"] = copy.deepcopy(row_deployment_coordinate_list)
                augmented_row["lineDeploymentCoordinateList"] = copy.deepcopy(row_deployment_coordinate_list)
                augmented_row["lineDeploymentDirectionLocked"] = True
                augmented_row["lineDeploymentDirectionXY"] = list(row_deployment_direction_xy)
            expected_rows.append(augmented_row)
            successful_piece_indices.add(piece_idx)
            replan_perf.add_elapsed(
                "mission_planning.next_collab_line.run.augment",
                perf_augment_started,
                pieceIndex=int(piece.piece_index or 0),
                shared=False,
            )
            perf_augment_count += 1
        replan_perf.add_elapsed(
            "mission_planning.next_collab_line.run.path_rows_initial",
            perf_stage_started,
            pieces=len(split_result.pieces),
            rows=len(expected_rows),
        )
        missing_pieces = [
            piece
            for piece in sorted(split_result.pieces, key=lambda row: int(row.piece_index or 0))
            if int(piece.piece_index or 0) not in successful_piece_indices
        ]
        if missing_pieces:
            if log is not None:
                missing_labels = ",".join(
                    f"P{int(piece.piece_index or 0)}/UAV{int(piece.assigned_uav or 0)}"
                    for piece in missing_pieces
                )
                log(
                    "[NEXTCOLLAB][LINE] retrying missing path rows with alternate "
                    f"overlay target points: {missing_labels}."
                )
            perf_stage_started = replan_perf.start_timer()
            for piece in missing_pieces:
                overlay = overlay_by_piece.get(int(piece.piece_index or 0))
                if overlay is None:
                    continue
                perf_make_started = replan_perf.start_timer()
                path_row = planner._make_path_row(piece, overlay, allow_alt_targets=True)
                replan_perf.add_elapsed(
                    "mission_planning.next_collab_line.run.make_path_row",
                    perf_make_started,
                    pieceIndex=int(piece.piece_index or 0),
                    allowAlt=True,
                )
                perf_fallback_path_row_count += 1
                if not isinstance(path_row, dict):
                    continue
                row_deployment_direction_xy, row_deployment_coordinate_list = (
                    _type2_branch_deployment_reference(piece, line_specs, mode_context)
                )
                if len(row_deployment_direction_xy) < 2:
                    row_deployment_direction_xy = list(deployment_direction_xy)
                    row_deployment_coordinate_list = copy.deepcopy(deployment_coordinate_list)
                if len(row_deployment_direction_xy) < 2:
                    deployment_direction_xy, deployment_coordinate_list = _derive_initial_line_deployment_reference(
                        source_coordinate_list,
                        piece,
                        path_row,
                    )
                    row_deployment_direction_xy = list(deployment_direction_xy)
                    row_deployment_coordinate_list = copy.deepcopy(deployment_coordinate_list)
                    if len(deployment_direction_xy) >= 2 and log is not None:
                        log("[NEXTCOLLAB][LINE] first execution deployment direction locked.")
                piece_aircraft_id = int(piece.assigned_uav or path_row.get("aircraftID") or 0)
                perf_augment_started = replan_perf.start_timer()
                augmented_row = _augment_line_path_row(
                    planner,
                    piece,
                    overlay,
                    path_row,
                    aircraft_row=aircraft_row_by_id.get(piece_aircraft_id),
                    deployment_direction_xy=row_deployment_direction_xy,
                    runtime_cfg=runtime_cfg,
                )
                if len(row_deployment_coordinate_list) >= 2:
                    augmented_row["sourceCoordinateList"] = copy.deepcopy(row_deployment_coordinate_list)
                    augmented_row["lineDeploymentCoordinateList"] = copy.deepcopy(row_deployment_coordinate_list)
                    augmented_row["lineDeploymentDirectionLocked"] = True
                    augmented_row["lineDeploymentDirectionXY"] = list(row_deployment_direction_xy)
                expected_rows.append(augmented_row)
                replan_perf.add_elapsed(
                    "mission_planning.next_collab_line.run.augment",
                    perf_augment_started,
                    pieceIndex=int(piece.piece_index or 0),
                    shared=False,
                )
                perf_augment_count += 1
            replan_perf.add_elapsed(
                "mission_planning.next_collab_line.run.path_rows_fallback",
                perf_stage_started,
                pieces=len(split_result.pieces),
                rows=len(expected_rows),
            )
        _ensure(bool(expected_rows), "next-collab line planner produced no path rows.")
        perf_stage_started = replan_perf.start_timer()
        shared_db_row = _shared_line_db_row_from_expected_rows(expected_rows, runtime_cfg=runtime_cfg)
        replan_perf.add_elapsed(
            "mission_planning.next_collab_line.run.shared_db_row",
            perf_stage_started,
            rows=len(expected_rows),
        )
        if isinstance(shared_db_row, dict) and len(expected_rows) >= 2:
            shared_expected_rows: List[Dict[str, Any]] = []
            perf_stage_started = replan_perf.start_timer()
            perf_shared_fast_count = 0
            for row in expected_rows:
                piece_idx = int(row.get("pieceIndex", 0) or 0)
                piece = piece_by_index.get(piece_idx)
                overlay = overlay_by_piece.get(piece_idx)
                if piece is None or overlay is None:
                    shared_expected_rows.append(dict(row))
                    continue
                fast_shared_row = _apply_shared_line_db_row_fast(
                    planner,
                    row,
                    shared_db_row,
                    runtime_cfg=runtime_cfg,
                )
                if isinstance(fast_shared_row, dict):
                    shared_expected_rows.append(fast_shared_row)
                    perf_shared_fast_count += 1
                    continue
                row_direction_xy = [
                    (float(point[0]), float(point[1]))
                    for point in (row.get("lineDeploymentDirectionXY") or [])
                    if isinstance(point, (tuple, list)) and len(point) >= 2
                ]
                perf_augment_started = replan_perf.start_timer()
                shared_expected_rows.append(
                    _augment_line_path_row(
                        planner,
                        piece,
                        overlay,
                        row,
                        shared_db_row=shared_db_row,
                        aircraft_row=aircraft_row_by_id.get(int(row.get("aircraftID") or piece.assigned_uav or 0)),
                        deployment_direction_xy=(row_direction_xy or deployment_direction_xy),
                        runtime_cfg=runtime_cfg,
                    )
                )
                replan_perf.add_elapsed(
                    "mission_planning.next_collab_line.run.augment",
                    perf_augment_started,
                    pieceIndex=piece_idx,
                    shared=True,
                )
                perf_shared_augment_count += 1
            expected_rows = shared_expected_rows
            replan_perf.add_elapsed(
                "mission_planning.next_collab_line.run.shared_reaugment",
                perf_stage_started,
                rows=len(expected_rows),
                fastRows=perf_shared_fast_count,
            )
        for row in expected_rows:
            if not isinstance(row, dict):
                continue
            row["sourceLineWidthM"] = float(source_line_width_m)
            if len(source_coordinate_list) >= 2 and len(row.get("sourceCoordinateList") or []) < 2:
                row["sourceCoordinateList"] = copy.deepcopy(source_coordinate_list)
            if len(deployment_direction_xy) >= 2 and len(row.get("lineDeploymentDirectionXY") or []) < 2:
                row["lineDeploymentDirectionLocked"] = True
                row["lineDeploymentDirectionXY"] = [
                    (float(x), float(y))
                    for x, y in deployment_direction_xy
                ]
            if len(deployment_coordinate_list) >= 2 and len(row.get("lineDeploymentCoordinateList") or []) < 2:
                row["lineDeploymentCoordinateList"] = copy.deepcopy(deployment_coordinate_list)
        split_result.expected_paths = list(expected_rows)
        planner.state.expected_paths = list(expected_rows)

    workflow = "line_prediction_path" if len(expected_rows) > 1 else "line_single_path"
    perf_stage_started = replan_perf.start_timer()
    result_lines = [
        "[NEXTCOLLAB][LINE] split "
        f"pieces={len(split_result.pieces)} "
        f"assignment={planner._assignment_summary_text(split_result)}",
        "[NEXTCOLLAB][LINE] prediction-assign "
        f"assigned={int(assign_report.get('assignedPieces', 0))}/"
        f"{int(assign_report.get('pieceCount', 0))}",
        f"[NEXTCOLLAB][LINE] mid-line refBearing={float(reference_bearing_deg):.1f}deg overlays={len(overlays)}",
        f"[NEXTCOLLAB][LINE] path rows={len(expected_rows)}",
    ]
    shared_db_row = _shared_line_db_row_from_expected_rows(expected_rows, runtime_cfg=runtime_cfg)
    if isinstance(shared_db_row, dict):
        result_lines.append(
            "[NEXTCOLLAB][LINE] shared sweep row "
            f"FOV={float(shared_db_row.get('fov', 0.0) or 0.0):.1f} "
            f"SEP={float(shared_db_row.get('sep', 0.0) or 0.0):.1f}m"
        )
    for row in expected_rows:
        if not isinstance(row, dict):
            continue
        base_fov = row.get("resolvedBaseFovDeg")
        adjusted_fov = row.get("resolvedFovDeg")
        if base_fov is None or adjusted_fov is None:
            continue
        fov_adjust_log = format_runtime_camera_fov_adjustment_log(
            base_fov,
            adjusted_fov,
            runtime_cfg,
            context=(
                f"NEXTCOLLAB LINE UAV{int(row.get('aircraftID', 0) or 0)}"
                f"/P{int(row.get('pieceIndex', 0) or 0)}"
            ),
        )
        if fov_adjust_log:
            result_lines.append(fov_adjust_log)
    for piece_line in assign_report.get("pieceLines") or []:
        result_lines.append(str(piece_line))
    for line in result_lines:
        if log is not None:
            log(line)
    replan_perf.add_elapsed(
        "mission_planning.next_collab_line.run.result_log_build",
        perf_stage_started,
        lines=len(result_lines),
        rows=len(expected_rows),
    )

    planner_result_text = "\n".join(result_lines)
    planner._set_result(planner_result_text)
    replan_perf.add_elapsed(
        "mission_planning.next_collab_line.run.total",
        perf_total_started,
        pieces=len(split_result.pieces),
        rows=len(expected_rows),
        pathRowCalls=perf_path_row_count,
        fallbackPathRowCalls=perf_fallback_path_row_count,
        augmentCalls=perf_augment_count,
        sharedAugmentCalls=perf_shared_augment_count,
    )
    result = NextCollabLinePlanResult(
        workflow=workflow,
        split_result=split_result,
        mid_line_segments=list(overlays),
        expected_paths=list(expected_rows),
        planner_result_text=str(planner._result_text_buffer or planner_result_text),
        planning_mode=dict(mode_context),
    )
    if line_plan_cache_key is not None:
        try:
            _remember_line_plan_cache(line_plan_cache_key, result)
            _store_line_plan_cache_to_disk(line_plan_cache_key, result)
            with _LINE_PLAN_CACHE_LOCK:
                _LINE_PLAN_CACHE_STATS["stores"] += 1
            if log is not None:
                log(
                    "[NEXTCOLLAB][LINE][CACHE] plan_result stored "
                    f"rows={len(result.expected_paths)} key={line_plan_cache_key[:12]}"
                )
        except Exception:
            with _LINE_PLAN_CACHE_LOCK:
                _LINE_PLAN_CACHE_STATS["errors"] += 1
    return result


__all__ = [
    "NextCollabLinePlanResult",
    "run_next_collab_line_plan",
]
