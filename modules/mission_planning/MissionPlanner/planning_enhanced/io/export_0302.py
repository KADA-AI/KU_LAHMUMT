from __future__ import annotations

import concurrent.futures
import copy
import json
import math
import os
import threading
import time
from contextlib import contextmanager
from bisect import bisect_left
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from ..algo.split_algorithms import mission_geometry
from ..models import SplitPiece, SplitRunResult
from modules.mission_planning.pipelines.lah_operational_mode import build_lah_special_sequence
from modules.mission_planning.pipelines.ground_maneuver_mode import (
    TYPE2_SELF_RELIANCE_OUTBOUND_LINE,
    TYPE2_SELF_RELIANCE_RETURN_LINE,
    build_ground_maneuver_lah_sequence,
    build_urban_operation_lah_sequence,
    resolve_type2_self_reliance_phase,
)
from modules.mission_planning.pipelines.type2_boundary_guard_loop import (
    BOUNDARY_GUARD_CONTRACT_KEYS,
    annotate_boundary_guard_set,
    extract_boundary_guard_contract,
)
from modules.mission_planning.planning_modes import (
    MissionPlanningMode,
    mission_mode_context,
    resolve_mission_planning_mode,
)
try:
    from ...runtime_settings import (
        apply_runtime_camera_adjusted_fov_deg,
        fov_db_path as runtime_fov_db_path,
        get_runtime_altitude_layers_m,
        get_runtime_area_auto_fov_from_db,
        get_runtime_str,
        get_runtime_value,
        load_runtime_settings,
        read_fov_db_rows_from_path,
    )
except Exception:
    from modules.mission_planning.MissionPlanner.runtime_settings import (  # type: ignore
        apply_runtime_camera_adjusted_fov_deg,
        fov_db_path as runtime_fov_db_path,
        get_runtime_altitude_layers_m,
        get_runtime_area_auto_fov_from_db,
        get_runtime_str,
        get_runtime_value,
        load_runtime_settings,
        read_fov_db_rows_from_path,
    )
try:
    from ...capture_geometry import (
        capture_aircraft_speed_kmh as _capture_aircraft_speed_kmh,
    )
except Exception:
    try:
        from modules.mission_planning.MissionPlanner.capture_geometry import (  # type: ignore
            capture_aircraft_speed_kmh as _capture_aircraft_speed_kmh,
        )
    except Exception:
        _capture_aircraft_speed_kmh = None
try:
    from ... import capture_physics as _capture_physics
except Exception:
    try:
        from modules.mission_planning.MissionPlanner import capture_physics as _capture_physics  # type: ignore
    except Exception:
        _capture_physics = None
try:
    from ....runtime.json_io import dumps_json
except Exception:
    try:
        from modules.mission_planning.runtime.json_io import dumps_json  # type: ignore
    except Exception:
        dumps_json = None

try:
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
except Exception:
    Polygon = None  # type: ignore
    unary_union = None  # type: ignore


_R = 6_378_137.0
_AREA_TYPES = {2, 3, 4, 5, 6}
_MISSION_NEEDS_COORD = {5, 7, 9}
_MISSION_NEEDS_LINE = {6}
_MISSION_NEEDS_AREA = {3, 4}
_MISSION_NEEDS_TARGET = {1, 2}
_FOV_DB_CACHE: Optional[List[Dict[str, float]]] = None
_FOV_DB_WIDTHS_CACHE: Optional[Tuple[float, ...]] = None
_FOV_DB_CACHE_SIG: Optional[Tuple[str, int, int]] = None
_ROWS_BY_WIDTH_CACHE: Dict[float, List[Dict[str, float]]] = {}
_BALANCED_ROW_CACHE: Dict[Tuple[int, int, float, float, int, float, float, bool], Optional[Dict[str, float]]] = {}
_CACHE_MISS = object()
_FOV_SELECTION_PREFERENCE = 0.65
_FOV_DB_PREFER_NEXT_SMALLER_AT_SAME_SEP = True
_FOV_DB_SEP_SAFETY_FACTOR = 1.7
_FOV_DB_QUALITY_SEP_MARGIN_M = 1.0
_ALTITUDE_LAYERS_M = (1000.0, 1010.0, 1020.0)
_RUNTIME_CACHE_LOCAL = threading.local()


def _now_ms_since_2000() -> int:
    epoch_2000 = 946684800.0
    return int((time.time() - epoch_2000) * 1000.0)


def _sw_code(default: str = "MMR") -> str:
    role = (os.environ.get("KU_ROLE") or "").lower()
    return {"mission": "MMR", "monitoring": "MSM", "decision": "MOB"}.get(role, default)


def _path_base(aid: int) -> int:
    return {
        1: 100_000_001,
        2: 200_000_001,
        3: 300_000_001,
        4: 400_000_001,
        5: 500_000_001,
        6: 600_000_001,
    }.get(aid, aid * 100_000_000 + 1)


def _package_id(aid: int) -> int:
    return 800_000_000 + int(aid)


def _as_pos_int(value: Any, fallback: int) -> int:
    try:
        v = int(value)
        return v if v > 0 else fallback
    except Exception:
        return fallback


def _to_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except Exception:
        return default


def _to_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return default


def _icd_width(value, default=0):
    try:
        return max(0, min(50000, int(round(float(value)))))
    except Exception:
        return int(default)


def _speed_key_kmh(v: float) -> int:
    return int(round(float(v) * 1000.0))


def _mission_perf_block(
    *,
    bearing_deg: Optional[float],
    move_bearing_deg: Optional[float],
    speed_kmh: Optional[float],
    fov_deg: Optional[float],
    sep_m: Optional[float],
) -> Dict[str, float]:
    bdeg = float(bearing_deg) if bearing_deg is not None else 0.0
    mbdeg = float(move_bearing_deg) if move_bearing_deg is not None else None
    v = max(0.0, float(speed_kmh) if speed_kmh is not None else 0.0)
    fov = max(0.0, float(fov_deg) if fov_deg is not None else 0.0)
    sep = max(0.0, float(sep_m) if sep_m is not None else 0.0)
    out = {
        "BEARING": float(bdeg),
        "SPEED": float(v),
        "FOV": float(fov),
        "SEP": float(sep),
    }
    if mbdeg is not None:
        out["MOVE_BEARING"] = float(mbdeg)
    return out


def _fov_db_sig(path: Path) -> Optional[Tuple[str, int, int]]:
    try:
        resolved = Path(path).resolve()
        stat = resolved.stat()
    except Exception:
        return None
    return str(resolved), int(stat.st_mtime_ns), int(stat.st_size)


def _load_fov_db_rows(path: Optional[Path] = None) -> List[Dict[str, float]]:
    global _FOV_DB_CACHE, _FOV_DB_WIDTHS_CACHE, _FOV_DB_CACHE_SIG
    db_path = Path(path) if path is not None else runtime_fov_db_path()
    sig = _fov_db_sig(db_path)
    if path is None and sig is not None and _FOV_DB_CACHE_SIG == sig and _FOV_DB_CACHE is not None:
        return _FOV_DB_CACHE

    out: List[Dict[str, float]] = []
    if sig is not None:
        for row in read_fov_db_rows_from_path(db_path):
            width = _to_float(row.get("width"), -1.0)
            vel = _to_float(row.get("vel"), -1.0)
            fov = _to_float(row.get("fov"), 0.0)
            sep = _to_float(row.get("sep"), 0.0)
            if width > 0.0 and vel > 0.0:
                out.append(
                    {
                        "width": float(width),
                        "vel": float(vel),
                        "fov": float(fov),
                        "sep": float(max(0.0, sep)),
                    }
                )

    out.sort(key=lambda r: (float(r["width"]), float(r["vel"]), float(r["fov"]), float(r["sep"])))
    if path is None:
        _FOV_DB_CACHE = out
        _FOV_DB_CACHE_SIG = sig
        _FOV_DB_WIDTHS_CACHE = tuple(float(r["width"]) for r in out)
        _ROWS_BY_WIDTH_CACHE.clear()
        _BALANCED_ROW_CACHE.clear()
    return out


def _runtime_cache_begin() -> None:
    stack = getattr(_RUNTIME_CACHE_LOCAL, "stack", None)
    if not isinstance(stack, list):
        stack = []
        _RUNTIME_CACHE_LOCAL.stack = stack
    previous_payload = getattr(_RUNTIME_CACHE_LOCAL, "payload", None)
    previous_values = getattr(_RUNTIME_CACHE_LOCAL, "values", None)
    stack.append((
        previous_payload if isinstance(previous_payload, dict) else None,
        previous_values if isinstance(previous_values, dict) else None,
    ))
    try:
        payload = load_runtime_settings()
    except Exception:
        payload = {}
    values = payload.get("values") if isinstance(payload, dict) else {}
    _RUNTIME_CACHE_LOCAL.payload = payload if isinstance(payload, dict) else {}
    _RUNTIME_CACHE_LOCAL.values = values if isinstance(values, dict) else {}


def _runtime_cache_end() -> None:
    stack = getattr(_RUNTIME_CACHE_LOCAL, "stack", None)
    previous_payload = None
    previous_values = None
    if isinstance(stack, list) and stack:
        previous_payload, previous_values = stack.pop()
    if isinstance(previous_payload, dict):
        _RUNTIME_CACHE_LOCAL.payload = previous_payload
    else:
        try:
            delattr(_RUNTIME_CACHE_LOCAL, "payload")
        except Exception:
            pass
    if isinstance(previous_values, dict):
        _RUNTIME_CACHE_LOCAL.values = previous_values
    else:
        try:
            delattr(_RUNTIME_CACHE_LOCAL, "values")
        except Exception:
            pass


@contextmanager
def _runtime_cache_scope():
    _runtime_cache_begin()
    try:
        yield
    finally:
        _runtime_cache_end()


def _runtime_payload() -> Optional[Dict[str, Any]]:
    payload = getattr(_RUNTIME_CACHE_LOCAL, "payload", None)
    return payload if isinstance(payload, dict) else None


def _runtime_value(key: str, default: Any) -> Any:
    values = getattr(_RUNTIME_CACHE_LOCAL, "values", None)
    if isinstance(values, dict):
        return values.get(key, default)
    return get_runtime_value(key, default)


def _runtime_area_sweep_mode() -> str:
    raw = str(_runtime_value("area_sweep_mode", "vertical") or "vertical").strip().lower()
    if raw in {"vertical", "ver", "perpendicular", "orthogonal"}:
        return "vertical"
    if raw in {"nadir", "directdown", "bf_nadir"}:
        return "nadir"
    return "parallel"


def _runtime_area_auto_fov_from_db(default: bool) -> bool:
    try:
        return bool(_runtime_value("enhanced_auto_fov_from_db", default))
    except Exception:
        return bool(default)


def _runtime_altitude_layers_m() -> Tuple[float, ...]:
    payload = _runtime_payload()
    if isinstance(payload, dict):
        return tuple(float(v) for v in get_runtime_altitude_layers_m(payload))
    return tuple(float(v) for v in get_runtime_altitude_layers_m())


def _runtime_db_fov_weight() -> float:
    weight = _to_float(_runtime_value("db_fov_weight", 1.0), 1.0)
    return weight if weight > 0.0 else 1.0


def _runtime_fov_db_smaller_fov_steps() -> int:
    try:
        steps = int(_to_float(_runtime_value("fov_db_smaller_fov_steps", 4), 4.0))
    except Exception:
        steps = 4
    return max(3, min(10, int(steps)))


def _runtime_area_fov_db_smaller_fov_steps() -> int:
    try:
        steps = int(_to_float(_runtime_value("area_fov_db_smaller_fov_steps", 3), 3.0))
    except Exception:
        steps = 3
    return max(3, min(10, int(steps)))


def _apply_db_fov_weight(fov_deg: float) -> float:
    fov = max(0.0, _to_float(fov_deg, 0.0))
    if fov <= 0.0:
        return 0.0
    return fov * _runtime_db_fov_weight()


def _vel_candidates_from_exp(exp: Dict[str, Any]) -> List[float]:
    vals = exp.get("velCandidatesKmh")
    out: List[float] = []
    if isinstance(vals, list):
        out = sorted({_to_float(v) for v in vals if _to_float(v) > 0.0})
    if out:
        return out
    vmin = _to_float(exp.get("velMinKmh"), 0.0)
    vmax = _to_float(exp.get("velMaxKmh"), 0.0)
    if vmin > 0.0:
        out.append(float(vmin))
    if vmax > 0.0:
        out.append(float(vmax))
    return sorted(set(out))


def _rows_by_width(rows: List[Dict[str, float]], width_ref_m: float) -> List[Dict[str, float]]:
    if not rows:
        return []
    req = max(0.0, float(width_ref_m))
    if req <= 0.0:
        return list(rows)

    if rows is _FOV_DB_CACHE and _FOV_DB_WIDTHS_CACHE:
        cached = _ROWS_BY_WIDTH_CACHE.get(req)
        if cached is not None:
            return cached
        widths = _FOV_DB_WIDTHS_CACHE
        idx = bisect_left(widths, req)
        if idx < len(rows):
            result = rows[idx:]
        else:
            max_width = widths[-1]
            start_idx = bisect_left(widths, max_width)
            result = rows[start_idx:]
        _ROWS_BY_WIDTH_CACHE[req] = result
        return result

    cands = [r for r in rows if float(r.get("width", 0.0)) + 1e-9 >= req]
    if cands:
        return cands

    nearest_w = min((float(r.get("width", 0.0)) for r in rows), key=lambda w: abs(w - req))
    return [r for r in rows if abs(float(r.get("width", 0.0)) - nearest_w) <= 1e-9]


def _db_row_quality_sep_m(row: Optional[Dict[str, float]]) -> float:
    if not isinstance(row, dict):
        return 0.0
    sep_m = max(0.0, _to_float(row.get("sep"), 0.0))
    width_m = max(0.0, _to_float(row.get("width"), 0.0))
    return math.hypot(sep_m, width_m * 0.5)


def _runtime_fov_db_sep_safety_factor() -> float:
    factor = _to_float(_runtime_value("fov_db_sep_safety_factor", _FOV_DB_SEP_SAFETY_FACTOR), 1.0)
    if factor <= 0.0:
        factor = 1.0
    return float(factor)


def _db_sep_requirement_m(sep_m: float) -> float:
    factor = _runtime_fov_db_sep_safety_factor()
    return max(0.0, float(sep_m or 0.0)) * float(factor)


def _select_balanced_row(
    rows: List[Dict[str, float]],
    *,
    width_ref_m: float,
    min_quality_sep_m: float = 0.0,
    smaller_fov_steps: int | None = None,
) -> Optional[Dict[str, float]]:
    sep_safety_factor = _runtime_fov_db_sep_safety_factor()
    min_quality_sep = max(0.0, float(min_quality_sep_m or 0.0)) * float(sep_safety_factor)
    try:
        steps_key = -1 if smaller_fov_steps is None else int(smaller_fov_steps)
    except Exception:
        steps_key = -1
    cache_key = (
        id(rows),
        len(rows),
        round(float(width_ref_m or 0.0), 6),
        round(float(min_quality_sep), 6),
        int(steps_key),
        round(float(sep_safety_factor), 9),
        round(float(_FOV_SELECTION_PREFERENCE), 9),
        bool(_FOV_DB_PREFER_NEXT_SMALLER_AT_SAME_SEP),
    )
    cached = _BALANCED_ROW_CACHE.get(cache_key, _CACHE_MISS)
    if cached is not _CACHE_MISS:
        return cached  # type: ignore[return-value]

    def _remember(row: Optional[Dict[str, float]]) -> Optional[Dict[str, float]]:
        if len(_BALANCED_ROW_CACHE) > 4096:
            _BALANCED_ROW_CACHE.clear()
        _BALANCED_ROW_CACHE[cache_key] = row
        return row

    if rows and min_quality_sep > 0.0:
        quality_rows = [
            r for r in rows
            if _to_float(r.get("sep"), 0.0) + 1e-9 >= min_quality_sep
        ]
        if quality_rows:
            rows = quality_rows
    if not rows:
        return _remember(None)
    if len(rows) == 1:
        return _remember(rows[0])

    width_ref = max(0.0, float(width_ref_m))
    min_fov = math.inf
    max_fov = -math.inf
    min_vel = math.inf
    max_vel = -math.inf
    max_fov_row: Optional[Dict[str, float]] = None
    max_vel_row: Optional[Dict[str, float]] = None
    max_fov_key: Optional[Tuple[float, float, float, float]] = None
    max_vel_key: Optional[Tuple[float, float, float, float]] = None

    for row in rows:
        fov = _to_float(row.get("fov"), 0.0)
        vel = _to_float(row.get("vel"), 0.0)
        width = _to_float(row.get("width"), 0.0)
        sep = _to_float(row.get("sep"), 0.0)
        min_fov = min(min_fov, fov)
        max_fov = max(max_fov, fov)
        min_vel = min(min_vel, vel)
        max_vel = max(max_vel, vel)

        fov_key = (
            fov,
            vel,
            -max(width - width_ref, 0.0),
            sep,
        )
        if max_fov_key is None or fov_key > max_fov_key:
            max_fov_key = fov_key
            max_fov_row = row

        vel_key = (
            vel,
            fov,
            -max(width - width_ref, 0.0),
            sep,
        )
        if max_vel_key is None or vel_key > max_vel_key:
            max_vel_key = vel_key
            max_vel_row = row

    if max_fov_row is None or max_vel_row is None:
        return _remember(rows[0])

    fov_scale = max(max_fov - min_fov, 1e-9)
    vel_scale = max(max_vel - min_vel, 1e-9)
    target_fov = (
        (_to_float(max_fov_row.get("fov"), 0.0) * _FOV_SELECTION_PREFERENCE)
        + (_to_float(max_vel_row.get("fov"), 0.0) * (1.0 - _FOV_SELECTION_PREFERENCE))
    )
    target_vel = (
        _to_float(max_fov_row.get("vel"), 0.0)
        + _to_float(max_vel_row.get("vel"), 0.0)
    ) / 2.0

    best_row: Optional[Dict[str, float]] = None
    best_key: Optional[Tuple[float, float, float, float, float]] = None
    for row in rows:
        fov = _to_float(row.get("fov"), 0.0)
        vel = _to_float(row.get("vel"), 0.0)
        width = _to_float(row.get("width"), 0.0)
        sep = _to_float(row.get("sep"), 0.0)
        score_key = (
            (((fov - target_fov) / fov_scale) ** 2 + ((vel - target_vel) / vel_scale) ** 2),
            max(width - width_ref, 0.0),
            -sep,
            -fov,
            -vel,
        )
        if best_key is None or score_key < best_key:
            best_key = score_key
            best_row = row

    selected = best_row or rows[0]
    if _FOV_DB_PREFER_NEXT_SMALLER_AT_SAME_SEP:
        selected = _prefer_next_smaller_fov_same_sep_row(
            rows,
            selected,
            width_ref_m=width_ref_m,
            smaller_fov_steps=smaller_fov_steps,
        )
    return _remember(selected)


def _prefer_next_smaller_fov_same_sep_row(
    rows: List[Dict[str, float]],
    selected: Dict[str, float],
    *,
    width_ref_m: float,
    smaller_fov_steps: int | None = None,
) -> Dict[str, float]:
    """Prefer one lower FOV step when it does not increase SEP or reduce speed."""
    if not rows or not isinstance(selected, dict):
        return selected
    base_fov = _to_float(selected.get("fov"), 0.0)
    base_sep = _to_float(selected.get("sep"), 0.0)
    base_vel = _to_float(selected.get("vel"), 0.0)
    if base_fov <= 0.0 or base_sep <= 0.0:
        return selected
    target_steps = (
        _runtime_fov_db_smaller_fov_steps()
        if smaller_fov_steps is None
        else max(3, min(10, int(smaller_fov_steps)))
    )
    if target_steps <= 0:
        return selected
    lower_fovs = sorted(
        {
            _to_float(row.get("fov"), 0.0)
            for row in rows
            if _to_float(row.get("fov"), 0.0) + 1e-9 < base_fov
        },
        reverse=True,
    )
    matched_steps = 0
    fallback: Dict[str, float] | None = None
    width_ref = max(0.0, float(width_ref_m))
    for target_fov in lower_fovs:
        candidates = [
            row for row in rows
            if abs(_to_float(row.get("fov"), 0.0) - target_fov) <= 1e-9
            and _to_float(row.get("sep"), 0.0) <= base_sep + 1e-9
            and _to_float(row.get("vel"), 0.0) + 1e-9 >= base_vel
        ]
        if candidates:
            candidate = min(
                candidates,
                key=lambda row: (
                    abs(_to_float(row.get("sep"), 0.0) - base_sep),
                    max(_to_float(row.get("width"), 0.0) - width_ref, 0.0),
                    -_to_float(row.get("vel"), 0.0),
                    -_to_float(row.get("sep"), 0.0),
                ),
            )
            fallback = candidate
            matched_steps += 1
            if matched_steps >= target_steps:
                return candidate
    return fallback or selected


def _piece_selected_speed_kmh(piece: SplitPiece) -> float:
    data = piece.data if isinstance(piece.data, dict) else {}
    sch = data.get("schedule")
    if isinstance(sch, dict):
        v = _to_float(sch.get("selectedVelKmh"), 0.0)
        if v > 0.0:
            return float(v)
    exp = data.get("expVel")
    if isinstance(exp, dict):
        v = _to_float(exp.get("velSelectedKmh"), 0.0)
        if v > 0.0:
            return float(v)
        vmax = _to_float(exp.get("velMaxKmh"), 0.0)
        if vmax > 0.0:
            return float(vmax)
        vmin = _to_float(exp.get("velMinKmh"), 0.0)
        if vmin > 0.0:
            return float(vmin)
    return 0.0


def _piece_has_line_geometry(data: Dict[str, Any]) -> bool:
    if _normalize_coord_list(data.get("Centerline")):
        return True
    if _normalize_coord_list(data.get("lineCoordinateList")):
        return True
    line_list = data.get("lineList")
    if isinstance(line_list, list):
        for row in line_list:
            if not isinstance(row, dict):
                continue
            if _normalize_coord_list(row.get("coordinateList")):
                return True
    return False


def _piece_is_area_like(piece: SplitPiece) -> bool:
    data = piece.data if isinstance(piece.data, dict) else {}
    if _piece_has_line_geometry(data):
        return False
    if int(piece.mission_type) in _MISSION_NEEDS_AREA:
        return True
    if isinstance(data.get("reviewArea"), dict):
        return True
    if _normalize_coord_list(data.get("rawCoordinateList")):
        return True
    if _normalize_coord_list(data.get("coordinateList")) and int(piece.mission_type) in _AREA_TYPES:
        return True
    return False


def _aircraft_alt_offset_m(aircraft_id: int) -> float:
    layers = _runtime_altitude_layers_m()
    try:
        idx = (int(aircraft_id) - 1) % len(layers)
    except Exception:
        idx = 0
    return float(layers[idx])


def _pick_nadir_fov_row_by_altitude(rows: List[Dict[str, float]], altitude_m: float) -> Optional[Dict[str, float]]:
    if not rows:
        return None
    alt_ref = max(0.0, float(altitude_m))
    cands = [r for r in rows if _to_float(r.get("sep"), -1.0) <= alt_ref + 1e-9]
    if not cands:
        cands = list(rows)
    if not cands:
        return None
    return max(
        cands,
        key=lambda r: (
            _to_float(r.get("fov"), 0.0),
            _to_float(r.get("sep"), 0.0),
            -abs(_to_float(r.get("sep"), 0.0) - alt_ref),
            _to_float(r.get("vel"), 0.0),
        ),
    )


def _capture_speed_kmh_or(fallback_kmh: float, fov_deg: float) -> float:
    """Unified capture-law aircraft speed (km/h) for the selected FOV.

    The FOV DB only selects the FOV; the DB vel (fallback_kmh) is used only
    when the FOV is invalid or the capture helper is unavailable.
    """
    if _capture_aircraft_speed_kmh is None:
        return float(fallback_kmh)
    try:
        if not (float(fov_deg) > 0.0):
            return float(fallback_kmh)
        speed = _capture_aircraft_speed_kmh(float(fov_deg), runtime_cfg=_runtime_payload())
    except Exception:
        speed = None
    if speed is None or not (float(speed) > 0.0):
        return float(fallback_kmh)
    return float(speed)


def _piece_speed_fov_sep(piece: SplitPiece) -> Tuple[float, float, float]:
    data = piece.data if isinstance(piece.data, dict) else {}
    exp = data.get("expVel") if isinstance(data.get("expVel"), dict) else {}
    speed_kmh = _piece_selected_speed_kmh(piece)
    area_mode = _runtime_area_sweep_mode()
    is_area_piece = _piece_is_area_like(piece)
    global_auto_fov_from_db = bool(_runtime_value("enhanced_auto_fov_from_db", True))
    auto_fov_from_db = (
        bool(_runtime_area_auto_fov_from_db(global_auto_fov_from_db))
        if is_area_piece
        else global_auto_fov_from_db
    )
    manual_fov_override = 0.0
    manual_sep_override = 0.0
    if area_mode == "nadir" and is_area_piece:
        alt_ref_m = _aircraft_alt_offset_m(int(piece.assigned_uav or 4))
        row = _pick_nadir_fov_row_by_altitude(_load_fov_db_rows(), alt_ref_m)
        if row is not None:
            base_fov_deg = _apply_db_fov_weight(_to_float(row.get("fov"), 0.0))
            nadir_fov_deg = float(
                apply_runtime_camera_adjusted_fov_deg(
                    base_fov_deg,
                    _runtime_payload(),
                    context=f"MISSION_PLAN 0302 UAV{int(piece.assigned_uav or 0)} NADIR_DB",
                )
            )
            return (
                _capture_speed_kmh_or(speed_kmh, nadir_fov_deg),
                nadir_fov_deg,
                float(alt_ref_m),
            )
    if not auto_fov_from_db:
        try:
            pattern_type = int(data.get("patternType", 0) or 0)
        except Exception:
            pattern_type = 0
        is_nadir_area_mode = (
            area_mode == "nadir"
            and is_area_piece
        )
        try:
            fov_key = "line_custom_fov_deg"
            if pattern_type == 3 or is_nadir_area_mode:
                fov_key = "area_nadir_fov_deg"
            elif is_area_piece:
                fov_key = "area_custom_fov_deg"
            else:
                fov_key = "line_custom_fov_deg"
            manual_fov = float(
                _runtime_value(
                    fov_key,
                    0.0,
                )
            )
        except Exception:
            manual_fov = 0.0
        try:
            manual_sep = float(_runtime_value("default_sweep_separation_m", 0.0))
        except Exception:
            manual_sep = 0.0
        if manual_fov > 0.0:
            manual_fov_override = float(manual_fov)
        if manual_sep > 0.0:
            manual_sep_override = float(manual_sep)
        if manual_fov_override > 0.0 and manual_sep_override > 0.0:
            manual_adjusted_fov_deg = float(
                apply_runtime_camera_adjusted_fov_deg(
                    manual_fov_override,
                    _runtime_payload(),
                    context=f"MISSION_PLAN 0302 UAV{int(piece.assigned_uav or 0)} MANUAL",
                )
            )
            return (
                _capture_speed_kmh_or(speed_kmh, manual_adjusted_fov_deg),
                manual_adjusted_fov_deg,
                float(manual_sep_override),
            )

    width_ref_m = _to_float(exp.get("widthRefM"), 0.0)
    # FOV 는 이 기체에 할당된 회랑 폭으로 골라야 한다.  widthRefM 은 스윕 기준축
    # 투영 스팬이라 LINE 조각에서는 회랑 길이 쪽(실측 8 km 급)이 잡히는 경우가
    # 있고, 그러면 physics_line_row 의 far_ground = 이격 + 폭/2 가 4 km 를 넘어
    # FOV 가 1.7~1.9° 로 붕괴한다(실측: 선언 폭 970~1,490 m 인데 FOV 1.74).
    # 붕괴한 FOV 로 촬영이 방출되는 사이 스윕 간격은 폭 기준으로 계산돼 서로
    # 어긋났다.  조각이 자기 폭을 들고 있으면 그것을 우선한다.
    piece_width_m = _to_float(data.get("width"), 0.0)
    if piece_width_m > 0.0:
        width_ref_m = float(piece_width_m)

    # ── 물리 선택 우선 (DB-free) ────────────────────────────────────────────
    # 요구공간해상도를 만족하는 실현가능 최대 FOV × margin 을 실시간 계산한다.
    # AREA는 area 전용 마진(physics_area_fov_margin_ratio, 기본 0.6)이 적용돼
    # 카메라 상한보다 한 단계 보수적으로 내려간다 — 실비행 급선회·뱅킹에서도
    # 요구해상도가 유지되게.  sep = 촬영 이격거리 = 고도.  (스윕 bearing 은
    # d0303 에서 정해지므로 행 기하는 여기서 전달하지 않는다.)
    # LINE은 초기계획이라 측정 이격이 없으므로 기본 보어사이트(45°) 기준.
    # None이면(비활성/실패) 아래 기존 FOV DB 경로로 그대로 진행한다.
    try:
        if _capture_physics is None:
            raise RuntimeError("capture_physics unavailable")
        if is_area_piece:
            physics_fov = _capture_physics.physics_area_fov_deg()
            if physics_fov is not None and float(physics_fov) > 0.0:
                adjusted_fov = float(
                    apply_runtime_camera_adjusted_fov_deg(
                        float(physics_fov),
                        _runtime_payload(),
                        context=f"MISSION_PLAN 0302 UAV{int(piece.assigned_uav or 0)} PHYSICS_AREA",
                    )
                )
                sep_out = float(
                    _capture_physics.capture_altitude_m(_runtime_payload())
                )
                return (
                    _capture_speed_kmh_or(speed_kmh, adjusted_fov),
                    adjusted_fov,
                    sep_out,
                )
        else:
            physics_row = _capture_physics.physics_line_row(
                float(width_ref_m or 0.0), 0.0
            )
            if isinstance(physics_row, dict) and float(physics_row.get("fov", 0.0) or 0.0) > 0.0:
                sep_out = float(physics_row.get("sep", 0.0) or 0.0)
                selected_fov = float(physics_row["fov"])
                # LINE 기체는 회랑 길이축을 따라 이동하면서 횡방향 스윕을
                # 순서대로 촬영한다. lineLengthM/uav_wp_interval_m은 카메라의
                # 동시 지상거리가 아니므로 FOV 선택에 넣지 않는다. 여기서의
                # 실제 품질거리는 할당된 회랑 폭 + SEP이며 physics_line_row가
                # 이미 그 최악 횡거리를 계산한다.
                adjusted_fov = float(
                    apply_runtime_camera_adjusted_fov_deg(
                        selected_fov,
                        _runtime_payload(),
                        context=f"MISSION_PLAN 0302 UAV{int(piece.assigned_uav or 0)} PHYSICS_LINE",
                    )
                )
                return (
                    _capture_speed_kmh_or(speed_kmh, adjusted_fov),
                    adjusted_fov,
                    sep_out,
                )
    except Exception:
        pass

    rows = _rows_by_width(_load_fov_db_rows(), width_ref_m)
    if not rows:
        return float(speed_kmh), 0.0, 0.0

    # Keep the same balanced selection policy, but avoid repeated full-table scans.
    smaller_fov_steps = (
        _runtime_area_fov_db_smaller_fov_steps()
        if is_area_piece
        else _runtime_fov_db_smaller_fov_steps()
    )
    best = _select_balanced_row(
        rows,
        width_ref_m=width_ref_m,
        smaller_fov_steps=smaller_fov_steps,
    )
    if best is None:
        return float(speed_kmh), 0.0, 0.0
    if not is_area_piece:
        required_quality_sep_m = (
            math.hypot(_to_float(best.get("sep"), 0.0), max(width_ref_m, 0.0) * 0.5)
            + float(_FOV_DB_QUALITY_SEP_MARGIN_M)
        )
        if _to_float(best.get("sep"), 0.0) + 1e-9 < _db_sep_requirement_m(required_quality_sep_m):
            refined = _select_balanced_row(
                rows,
                width_ref_m=width_ref_m,
                min_quality_sep_m=required_quality_sep_m,
                smaller_fov_steps=smaller_fov_steps,
            )
            if refined is not None:
                best = refined
    base_fov_deg = _apply_db_fov_weight(_to_float(best.get("fov"), 0.0))
    fov_deg = apply_runtime_camera_adjusted_fov_deg(
        base_fov_deg,
        _runtime_payload(),
        context=f"MISSION_PLAN 0302 UAV{int(piece.assigned_uav or 0)} DB",
    )
    if manual_fov_override > 0.0:
        fov_deg = float(
            apply_runtime_camera_adjusted_fov_deg(
                manual_fov_override,
                _runtime_payload(),
                context=f"MISSION_PLAN 0302 UAV{int(piece.assigned_uav or 0)} MANUAL",
            )
        )
    sep_m = _to_float(best.get("sep"), 0.0)
    if manual_sep_override > 0.0:
        sep_m = float(manual_sep_override)
    return _capture_speed_kmh_or(speed_kmh, float(fov_deg)), float(fov_deg), float(sep_m)


def _extract_aircraft_id(entry: Any) -> Optional[int]:
    if entry is None:
        return None
    if isinstance(entry, dict):
        for key in ("aircraftID", "aircraftId", "id"):
            if key in entry:
                entry = entry.get(key)
                break
    if isinstance(entry, int):
        return entry
    if isinstance(entry, str):
        token = entry.strip().upper()
        if token.startswith(("UAV", "LAH")):
            token = "".join(ch for ch in token if ch.isdigit())
        if not token:
            return None
        try:
            return int(token)
        except Exception:
            return None
    try:
        return int(entry)
    except Exception:
        return None


def _normalize_coord_list(coords: Any) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    if not isinstance(coords, list):
        return out
    for c in coords:
        if not isinstance(c, dict):
            continue
        try:
            lat = float(c.get("latitude", 0.0))
            lon = float(c.get("longitude", 0.0))
            alt = int(round(float(c.get("altitude", 0) or 0)))
        except Exception:
            lat, lon, alt = 0.0, 0.0, 0
        out.append({"latitude": lat, "longitude": lon, "altitude": alt})
    return out


def _bearing_deg_llh(p0: Dict[str, Any], p1: Dict[str, Any]) -> float:
    lat0 = math.radians(_to_float(p0.get("latitude"), 0.0))
    lon0 = math.radians(_to_float(p0.get("longitude"), 0.0))
    lat1 = math.radians(_to_float(p1.get("latitude"), 0.0))
    lon1 = math.radians(_to_float(p1.get("longitude"), 0.0))
    dlon = lon1 - lon0
    x = math.sin(dlon) * math.cos(lat1)
    y = math.cos(lat0) * math.sin(lat1) - math.sin(lat0) * math.cos(lat1) * math.cos(dlon)
    return float((math.degrees(math.atan2(x, y)) + 360.0) % 360.0)


def _bearing_from_coords(coords: List[Dict[str, Any]]) -> float:
    if len(coords) >= 2:
        try:
            return _bearing_deg_llh(coords[0], coords[-1])
        except Exception:
            return 0.0
    return 0.0


def _llh_to_xy(lat: float, lon: float, lat0: float, lon0: float) -> Tuple[float, float]:
    lat0_r = math.radians(lat0)
    x = math.radians(lon - lon0) * _R * math.cos(lat0_r)
    y = math.radians(lat - lat0) * _R
    return x, y


def _xy_to_llh(x: float, y: float, lat0: float, lon0: float) -> Tuple[float, float]:
    lat0_r = math.radians(lat0)
    lat = lat0 + math.degrees(y / _R)
    lon = lon0 + math.degrees(x / (_R * math.cos(lat0_r)))
    return lat, lon


def _largest_polygon(geom: Any) -> Optional[Any]:
    if Polygon is None:
        return None
    if isinstance(geom, Polygon):
        return geom
    if geom is None or geom.is_empty:
        return None
    gt = getattr(geom, "geom_type", "")
    if gt in ("MultiPolygon", "GeometryCollection"):
        polys = [g for g in geom.geoms if isinstance(g, Polygon) and not g.is_empty]
        if polys:
            return max(polys, key=lambda g: g.area)
    return None


def _is_area_piece(piece: SplitPiece) -> bool:
    return int(piece.mission_type) in _AREA_TYPES


def _piece_schedule_start(piece: SplitPiece) -> Optional[float]:
    data = piece.data if isinstance(piece.data, dict) else {}
    sch = data.get("schedule")
    if not isinstance(sch, dict):
        return None
    try:
        v = float(sch.get("startSec"))
    except Exception:
        return None
    if v < 0.0:
        return None
    return v


def _area_stage(piece: SplitPiece) -> int:
    data = piece.data if isinstance(piece.data, dict) else {}
    v = _to_int(data.get("splitStage"), 0)
    if v > 0:
        return v
    return 1


def _area_source_idx(piece: SplitPiece) -> int:
    data = piece.data if isinstance(piece.data, dict) else {}
    v = _to_int(data.get("sourceAreaIndex"), 0)
    return v if v > 0 else 1


def _area_base_idx(piece: SplitPiece) -> int:
    data = piece.data if isinstance(piece.data, dict) else {}
    rv = data.get("reviewArea")
    if isinstance(rv, dict) and bool(rv.get("subdivided", False)):
        fidx = _to_int(rv.get("fromPieceIndex"), 0)
        if fidx > 0:
            return fidx
    return int(piece.piece_index)


def _merge_area_group_to_piece(pieces: List[SplitPiece]) -> SplitPiece:
    first = pieces[0]
    first_stage = _area_stage(first)
    if len(pieces) == 1:
        data0 = first.data if isinstance(first.data, dict) else {}
        rv0 = data0.get("reviewArea") if isinstance(data0.get("reviewArea"), dict) else {}
        if isinstance(rv0, dict) and bool(rv0.get("subdivided", False)):
            src_coords = _normalize_coord_list(rv0.get("fromPieceCoordinateList"))
            if len(src_coords) >= 3:
                ndata = copy.deepcopy(data0)
                ndata["coordinateList"] = src_coords
                src_raw = _normalize_coord_list(rv0.get("fromPieceRawCoordinateList"))
                if len(src_raw) >= 3:
                    ndata["rawCoordinateList"] = src_raw
                ndata["splitStage"] = int(first_stage)
                ndata["sourceAreaIndex"] = _area_source_idx(first)
                rv = ndata.get("reviewArea") if isinstance(ndata.get("reviewArea"), dict) else {}
                rv["restoredFor0302"] = True
                ndata["reviewArea"] = rv
                return SplitPiece(
                    parent_order=int(first.parent_order),
                    mission_id=first.mission_id,
                    mission_type=int(first.mission_type),
                    piece_index=int(_area_base_idx(first)),
                    data=ndata,
                    assigned_uav=int(first.assigned_uav) if first.assigned_uav is not None else None,
                )
        return first

    data0 = first.data if isinstance(first.data, dict) else {}
    coord0 = _normalize_coord_list(data0.get("coordinateList"))
    if not coord0:
        return first

    # Preferred: restore pre-review source geometry directly when available.
    # This is the "original split + 10% expanded" shape before review re-split.
    rv0 = data0.get("reviewArea") if isinstance(data0.get("reviewArea"), dict) else {}
    if isinstance(rv0, dict) and bool(rv0.get("subdivided", False)):
        src_coords = _normalize_coord_list(rv0.get("fromPieceCoordinateList"))
        if len(src_coords) >= 3:
            ndata = copy.deepcopy(data0)
            ndata["coordinateList"] = src_coords
            src_raw = _normalize_coord_list(rv0.get("fromPieceRawCoordinateList"))
            if len(src_raw) >= 3:
                ndata["rawCoordinateList"] = src_raw
            ndata["splitStage"] = int(first_stage)
            ndata["sourceAreaIndex"] = _area_source_idx(first)
            rv = ndata.get("reviewArea") if isinstance(ndata.get("reviewArea"), dict) else {}
            rv["mergedFor0302"] = True
            rv["mergedPieceCount"] = int(len(pieces))
            rv["fromPieceIndex"] = int(_area_base_idx(first))
            ndata["reviewArea"] = rv
            starts = [s for s in (_piece_schedule_start(p) for p in pieces) if s is not None]
            if starts:
                sch = ndata.get("schedule") if isinstance(ndata.get("schedule"), dict) else {}
                sch["startSec"] = float(min(starts))
                ndata["schedule"] = sch
            return SplitPiece(
                parent_order=int(first.parent_order),
                mission_id=first.mission_id,
                mission_type=int(first.mission_type),
                piece_index=int(_area_base_idx(first)),
                data=ndata,
                assigned_uav=int(first.assigned_uav) if first.assigned_uav is not None else None,
            )

    lat0 = float(coord0[0]["latitude"])
    lon0 = float(coord0[0]["longitude"])
    alt0 = int(coord0[0].get("altitude", 0))

    merged_coord = coord0
    merged_ok = False
    if Polygon is not None and unary_union is not None:
        polys: List[Any] = []
        for p in pieces:
            pd = p.data if isinstance(p.data, dict) else {}
            pr = pd.get("reviewArea") if isinstance(pd.get("reviewArea"), dict) else {}
            # For review-subdivided pieces, prefer raw clipped polygons (before 2nd post-process).
            if isinstance(pr, dict) and bool(pr.get("subdivided", False)):
                coords = _normalize_coord_list(pd.get("rawCoordinateList"))
                if len(coords) < 3:
                    coords = _normalize_coord_list(pd.get("coordinateList"))
            else:
                coords = _normalize_coord_list(pd.get("coordinateList"))
            if len(coords) < 3:
                continue
            xy = [_llh_to_xy(float(c["latitude"]), float(c["longitude"]), lat0, lon0) for c in coords]
            try:
                poly = Polygon(xy)
                if not poly.is_valid:
                    poly = poly.buffer(0)
                if (not poly.is_empty) and float(poly.area) > 1e-9:
                    polys.append(poly)
            except Exception:
                continue
        if polys:
            try:
                uni = unary_union(polys)
                picked = _largest_polygon(uni)
                if picked is not None:
                    xy_out = list(picked.exterior.coords)[:-1]
                    out: List[Dict[str, Any]] = []
                    for x, y in xy_out:
                        lat, lon = _xy_to_llh(float(x), float(y), lat0, lon0)
                        out.append({"latitude": float(lat), "longitude": float(lon), "altitude": int(alt0)})
                    if len(out) >= 3:
                        merged_coord = out
                        merged_ok = True
            except Exception:
                merged_ok = False

    if not merged_ok:
        best = first
        best_n = len(coord0)
        for p in pieces[1:]:
            pd = p.data if isinstance(p.data, dict) else {}
            cc = _normalize_coord_list(pd.get("coordinateList"))
            if len(cc) > best_n:
                best = p
                best_n = len(cc)
        return best

    ndata = copy.deepcopy(data0)
    ndata["coordinateList"] = merged_coord
    ndata["splitStage"] = int(first_stage)
    ndata["sourceAreaIndex"] = _area_source_idx(first)
    rv = ndata.get("reviewArea") if isinstance(ndata.get("reviewArea"), dict) else {}
    rv["mergedFor0302"] = True
    rv["mergedPieceCount"] = int(len(pieces))
    rv["fromPieceIndex"] = int(_area_base_idx(first))
    ndata["reviewArea"] = rv

    starts = [s for s in (_piece_schedule_start(p) for p in pieces) if s is not None]
    if starts:
        sch = ndata.get("schedule") if isinstance(ndata.get("schedule"), dict) else {}
        sch["startSec"] = float(min(starts))
        ndata["schedule"] = sch

    return SplitPiece(
        parent_order=int(first.parent_order),
        mission_id=first.mission_id,
        mission_type=int(first.mission_type),
        piece_index=int(_area_base_idx(first)),
        data=ndata,
        assigned_uav=int(first.assigned_uav) if first.assigned_uav is not None else None,
    )


def _collect_export_pieces(split_result: SplitRunResult) -> Dict[int, List[SplitPiece]]:
    by_uav: Dict[int, List[SplitPiece]] = {}

    # Export assigned split pieces as-is.
    # Do not merge reviewed/subdivided area pieces back to pre-split geometry.
    for piece in split_result.pieces:
        aid = int(piece.assigned_uav or 0)
        if aid <= 0:
            continue
        if _should_skip_vertical_review_sliver(piece):
            continue
        by_uav.setdefault(aid, []).append(piece)

    for aid in list(by_uav.keys()):
        by_uav[aid].sort(
            key=lambda p: (
                int(p.parent_order),
                int(_area_source_idx(p)),
                int(_area_stage(p)),
                int(p.piece_index),
            )
        )
    return by_uav


def _dedup_xy_points(points_xy: List[Tuple[float, float]], eps_m: float = 1.0) -> List[Tuple[float, float]]:
    if not points_xy:
        return []
    out: List[Tuple[float, float]] = []
    eps2 = float(eps_m) * float(eps_m)
    for x, y in points_xy:
        keep = True
        for ux, uy in out:
            dx = float(x) - float(ux)
            dy = float(y) - float(uy)
            if (dx * dx + dy * dy) <= eps2:
                keep = False
                break
        if keep:
            out.append((float(x), float(y)))
    return out


def _should_skip_vertical_review_sliver(piece: SplitPiece) -> bool:
    if _runtime_area_sweep_mode() != "vertical":
        return False
    if not _is_area_piece(piece):
        return False
    data = piece.data if isinstance(piece.data, dict) else {}
    review = data.get("reviewArea") if isinstance(data.get("reviewArea"), dict) else {}
    if not (isinstance(review, dict) and bool(review.get("subdivided", False))):
        return False

    coords = _normalize_coord_list(data.get("rawCoordinateList"))
    if len(coords) < 3:
        coords = _normalize_coord_list(data.get("coordinateList"))
    if len(coords) < 3:
        return True

    lat0 = float(coords[0]["latitude"])
    lon0 = float(coords[0]["longitude"])
    points_xy = [
        _llh_to_xy(float(c["latitude"]), float(c["longitude"]), lat0, lon0)
        for c in coords
    ]
    unique_xy = _dedup_xy_points(points_xy, eps_m=1.0)
    if len(unique_xy) < 3:
        return True

    if Polygon is None:
        return False
    try:
        poly = Polygon(unique_xy)
        if not poly.is_valid:
            poly = poly.buffer(0)
        picked = _largest_polygon(poly)
        if picked is None:
            return True
        return float(picked.area) < 1.0
    except Exception:
        return False


def _piece_move_bearing_deg(piece: SplitPiece) -> float:
    data = piece.data if isinstance(piece.data, dict) else {}
    # Priority: incoming direction from previous mission, then piece move fallback.
    for k in ("bearingFromPrev", "phaseMoveBearing_deg", "bearing_deg"):
        if k in data:
            try:
                return float(data.get(k))
            except Exception:
                pass
    center = _normalize_coord_list(data.get("Centerline"))
    if len(center) >= 2:
        return _bearing_from_coords(center)
    coords = _normalize_coord_list(data.get("coordinateList"))
    if len(coords) >= 2:
        return _bearing_from_coords(coords)
    return 0.0


def _piece_area_sweep_bearing_deg(piece: SplitPiece, *, fallback_move_bearing: float) -> float:
    mode = _runtime_area_sweep_mode()
    if mode == "parallel":
        return float(fallback_move_bearing)
    if mode == "vertical":
        return (float(fallback_move_bearing) + 90.0) % 360.0
    if mode == "nadir":
        return float(fallback_move_bearing)
    data = piece.data if isinstance(piece.data, dict) else {}
    for k in ("boundaryAxisBearing_deg", "splitBearing_deg", "phaseSplitBearing_deg", "bearing_split_deg"):
        if k in data:
            try:
                return float(data.get(k))
            except Exception:
                pass
    if "phaseMoveBearing_deg" in data:
        try:
            return (float(data.get("phaseMoveBearing_deg")) + 90.0) % 360.0
        except Exception:
            pass
    return (float(fallback_move_bearing) + 90.0) % 360.0


def _mission_info_by_type(
    im_type: int,
    pattern_type: int,
    data: Dict[str, Any],
    bearing_deg: Optional[float] = None,
    move_bearing_deg: Optional[float] = None,
    speed_kmh: Optional[float] = None,
    fov_deg: Optional[float] = None,
    sep_m: Optional[float] = None,
) -> Dict[str, Any]:
    perf = _mission_perf_block(
        bearing_deg=bearing_deg,
        move_bearing_deg=move_bearing_deg,
        speed_kmh=speed_kmh,
        fov_deg=fov_deg,
        sep_m=sep_m,
    )
    if im_type == 5:
        coord_list = _normalize_coord_list(data.get("coordinateList"))
        if not coord_list:
            coord_list = _normalize_coord_list(data.get("Centerline"))
        return {
            "individualMissionType": 5,
            "patternType": int(pattern_type),
            "autoZoomIn": True,
            **perf,
            "coordinateList": coord_list,
            "targetID": None,
        }

    if im_type == 6:
        width = _icd_width(data.get("width", 0.0) or 0.0)
        centerline = _normalize_coord_list(data.get("Centerline"))
        if not centerline:
            centerline = _normalize_coord_list(data.get("coordinateList"))
        return {
            "individualMissionType": 6,
            "patternType": int(pattern_type),
            "autoZoomIn": True,
            **perf,
            "lineList": [{"width": _icd_width(width), "coordinateList": centerline}],
            "targetID": None,
        }

    if im_type == 7:
        centerline = _normalize_coord_list(data.get("Centerline"))
        if not centerline:
            centerline = _normalize_coord_list(data.get("coordinateList"))
        return {
            "individualMissionType": 7,
            "patternType": int(pattern_type),
            "autoZoomIn": True,
            **perf,
            "coordinateList": centerline,
            "lineList": [{"width": _icd_width(data.get("width", 1.0) or 1.0, default=1), "coordinateList": centerline}],
            "targetID": None,
        }

    if im_type in (3, 4):
        review = data.get("reviewArea") if isinstance(data.get("reviewArea"), dict) else {}
        if isinstance(review, dict) and bool(review.get("subdivided", False)):
            coords = _normalize_coord_list(data.get("rawCoordinateList"))
            if len(coords) < 3:
                coords = _normalize_coord_list(data.get("coordinateList"))
        else:
            coords = _normalize_coord_list(data.get("coordinateList"))
        return {
            "individualMissionType": int(im_type),
            "patternType": int(pattern_type),
            "autoZoomIn": True,
            **perf,
            "areaList": [{"isHole": False, "coordinateList": coords}],
            "targetID": None,
        }

    return {
        "individualMissionType": int(im_type),
        "patternType": int(pattern_type),
        "autoZoomIn": True,
        **perf,
        "targetID": None,
    }


def _piece_to_mission_info(piece: SplitPiece) -> Dict[str, Any]:
    info = _piece_to_mission_info_base(piece)
    data = piece.data if isinstance(piece.data, dict) else {}
    contract = extract_boundary_guard_contract(data)
    for key, value in contract.items():
        info[key] = copy.deepcopy(value)
    # Type 4 donut patrol marker rides through the 0302 info so d0303 can build
    # the ring-lane route + radial lineSearch instead of the generic area sweep.
    if isinstance(data.get("_donutPatrol"), dict):
        info["_donutPatrol"] = copy.deepcopy(data.get("_donutPatrol"))
        # A patrol band is an annulus: emit it as outer(isHole=False) +
        # inner(isHole=True) so it renders as a ring, not a self-intersecting
        # bowtie (which the SIM/monitoring fill rule shows half-filled).
        inner = _normalize_coord_list(data.get("_donutBandInner"))
        if len(inner) >= 3 and isinstance(info.get("areaList"), list) and info["areaList"]:
            outer_coords = _normalize_coord_list(info["areaList"][0].get("coordinateList"))
            if len(outer_coords) >= 3:
                info["areaList"] = [
                    {"isHole": False, "coordinateList": outer_coords},
                    {"isHole": True, "coordinateList": inner},
                ]
    return info


def _piece_to_mission_info_base(piece: SplitPiece) -> Dict[str, Any]:
    orig_type = int(piece.mission_type)
    data = piece.data if isinstance(piece.data, dict) else {}
    move_bdeg = _piece_move_bearing_deg(piece)
    sweep_bdeg = move_bdeg
    area_mode = _runtime_area_sweep_mode()
    if orig_type in (2, 3, 4, 5, 6):
        sweep_bdeg = _piece_area_sweep_bearing_deg(piece, fallback_move_bearing=move_bdeg)
    speed_kmh, fov_deg, sep_m = _piece_speed_fov_sep(piece)

    if "individualMissionType" in data:
        try:
            im_type = int(data.get("individualMissionType"))
            if im_type > 0:
                pattern = int(data.get("patternType", 0) or 0)
                if area_mode == "nadir" and im_type in (3, 4):
                    pattern = 3
                bdeg = move_bdeg
                if im_type in (3, 4):
                    bdeg = _piece_area_sweep_bearing_deg(piece, fallback_move_bearing=move_bdeg)
                return _mission_info_by_type(
                    im_type,
                    pattern,
                    data,
                    bearing_deg=bdeg,
                    move_bearing_deg=move_bdeg,
                    speed_kmh=speed_kmh,
                    fov_deg=fov_deg,
                    sep_m=sep_m,
                )
        except Exception:
            pass

    if orig_type in (1, 4, 5):
        width = _icd_width(data.get("width", 0.0) or 0.0)
        centerline = _normalize_coord_list(data.get("Centerline"))
        return {
            "individualMissionType": 6,
            "patternType": int(data.get("patternType", 0) or 0),
            "autoZoomIn": True,
            **_mission_perf_block(
                bearing_deg=move_bdeg,
                move_bearing_deg=move_bdeg,
                speed_kmh=speed_kmh,
                fov_deg=fov_deg,
                sep_m=sep_m,
            ),
            "lineList": [{"width": _icd_width(width), "coordinateList": centerline}],
            "targetID": None,
        }

    if orig_type == 7:
        centerline = _normalize_coord_list(data.get("Centerline"))
        return {
            "individualMissionType": 7,
            "patternType": int(data.get("patternType", 10) or 10),
            "autoZoomIn": True,
            **_mission_perf_block(
                bearing_deg=move_bdeg,
                move_bearing_deg=move_bdeg,
                speed_kmh=speed_kmh,
                fov_deg=fov_deg,
                sep_m=sep_m,
            ),
            "coordinateList": centerline,
            "lineList": [{"width": _icd_width(data.get("width", 1.0) or 1.0, default=1), "coordinateList": centerline}],
            "targetID": None,
        }

    area_type = 3 if orig_type in (2, 6) else 4
    review = data.get("reviewArea") if isinstance(data.get("reviewArea"), dict) else {}
    if isinstance(review, dict) and bool(review.get("subdivided", False)):
        coords = _normalize_coord_list(data.get("rawCoordinateList"))
        if len(coords) < 3:
            coords = _normalize_coord_list(data.get("coordinateList"))
    else:
        coords = _normalize_coord_list(data.get("coordinateList"))
    return {
        "individualMissionType": area_type,
        "patternType": 3 if area_mode == "nadir" else int(data.get("patternType", 0) or 0),
        "autoZoomIn": True,
        **_mission_perf_block(
            bearing_deg=sweep_bdeg,
            move_bearing_deg=move_bdeg,
            speed_kmh=speed_kmh,
            fov_deg=fov_deg,
            sep_m=sep_m,
        ),
        "areaList": [{"isHole": False, "coordinateList": coords}],
        "targetID": None,
    }


def _inserted_for_uav(split_result: SplitRunResult, aid: int) -> List[Dict[str, Any]]:
    sched = split_result.schedule_result if isinstance(split_result.schedule_result, dict) else {}
    rows = sched.get("insertedMissions")
    if not isinstance(rows, list):
        return []
    out: List[Dict[str, Any]] = []
    for row in rows:
        if not isinstance(row, dict):
            continue
        if int(row.get("uavID", 0) or 0) != int(aid):
            continue
        out.append(row)
    return out


def _piece_runtime_meta(piece: SplitPiece) -> Dict[str, Any]:
    data = piece.data if isinstance(piece.data, dict) else {}
    meta: Dict[str, Any] = {}
    for key in (
        "bearing_deg",
        "splitBearing_deg",
        "phaseMoveBearing_deg",
        "phaseSplitBearing_deg",
        "boundaryAxisBearing_deg",
        "bearingIn_deg",
        "bearingOut_deg",
    ):
        if key in data:
            meta[key] = data.get(key)
    for key in (
        "branchIndex",
        "branchAreaSequentialSplit",
        "branchAreaSequence",
        "splitStage",
        "splitCount",
        "areaSequentialOwnerSlot",
        "areaSequentialOwnerCount",
        "areaSequentialWidthSplit",
        "areaSequentialWidthSpanM",
        "areaSequentialWidthTargetM",
        "areaSequentialWidthLimitM",
        "areaOuterOwner",
        "areaOuterSide",
        "areaOuterFirstSweep",
        *BOUNDARY_GUARD_CONTRACT_KEYS,
    ):
        if key in data:
            meta[key] = copy.deepcopy(data.get(key))
    for key in ("prevPoint", "nextPoint"):
        value = data.get(key)
        if isinstance(value, dict):
            meta[key] = {
                "latitude": float(value.get("latitude", 0.0)),
                "longitude": float(value.get("longitude", 0.0)),
                "altitude": int(round(float(value.get("altitude", 0.0) or 0.0))),
            }
    return meta


def _inserted_to_mission_info(row: Dict[str, Any]) -> Dict[str, Any]:
    coords = _normalize_coord_list(row.get("coordinateList"))
    bdeg = _to_float(row.get("bearingDeg"), 0.0)
    hold_s = _to_float(row.get("durationSec"), 0.0)
    return {
        "individualMissionType": 5,
        "patternType": 1,
        "autoZoomIn": True,
        **_mission_perf_block(
            bearing_deg=bdeg,
            move_bearing_deg=bdeg,
            speed_kmh=0.0,
            fov_deg=0.0,
            sep_m=0.0,
        ),
        "holdingReqTime": float(max(0.0, hold_s)),
        "coordinateList": coords,
        "targetID": None,
    }


def _parent_mission_id_map(split_result: SplitRunResult, aid: int) -> Dict[int, int]:
    out: Dict[int, int] = {}
    for p in split_result.pieces:
        if int(p.assigned_uav or 0) != int(aid):
            continue
        parent = int(p.parent_order)
        mid = _as_pos_int(p.mission_id, parent)
        if parent > 0 and mid > 0 and parent not in out:
            out[parent] = mid
    return out


def _validate_0302_packages(packages: List[Dict[str, Any]]) -> None:
    if not isinstance(packages, list) or not packages:
        raise ValueError("[0302] empty packages")

    seen_aircraft: set[int] = set()
    seen_pkg: set[int] = set()
    seen_im: set[int] = set()
    seen_path: set[int] = set()

    for pkg in packages:
        for k in ("timestamp", "individualMissionPackageID", "aircraftID", "individualMissionList"):
            if k not in pkg:
                raise ValueError(f"[0302] package missing '{k}'")

        aid = _to_int(pkg.get("aircraftID"), 0)
        if aid < 1 or aid > 6:
            raise ValueError(f"[0302] invalid aircraftID={aid}")
        if aid in seen_aircraft:
            raise ValueError(f"[0302] duplicate aircraft package {aid}")
        seen_aircraft.add(aid)

        pkg_id = _to_int(pkg.get("individualMissionPackageID"), 0)
        if pkg_id <= 0:
            raise ValueError(f"[0302] invalid package id {pkg_id}")
        if pkg_id in seen_pkg:
            raise ValueError(f"[0302] duplicate package id {pkg_id}")
        seen_pkg.add(pkg_id)

        ims = pkg.get("individualMissionList")
        if not isinstance(ims, list) or not ims:
            raise ValueError(f"[0302] aircraft {aid}: empty individualMissionList")

        base = _path_base(aid)
        upper = base + 100_000_000
        prev_im: Optional[int] = None
        prev_path: Optional[int] = None
        for im in ims:
            for k in ("individualMissionID", "isDone", "relatedMission", "individualMissionInfo", "pathID"):
                if k not in im:
                    raise ValueError(f"[0302] mission missing '{k}'")

            im_id = _to_int(im.get("individualMissionID"), 0)
            if im_id < 900_000_001:
                raise ValueError(f"[0302] invalid individualMissionID {im_id}")
            if im_id in seen_im:
                raise ValueError(f"[0302] duplicate individualMissionID {im_id}")
            if prev_im is not None and im_id <= prev_im:
                raise ValueError("[0302] individualMissionID not ascending in package")
            prev_im = im_id
            seen_im.add(im_id)

            pid = _to_int(im.get("pathID"), 0)
            if pid < base or pid >= upper:
                raise ValueError(f"[0302] pathID {pid} out of range for aircraft {aid}")
            if pid in seen_path:
                raise ValueError(f"[0302] duplicate pathID {pid}")
            if prev_path is not None and pid <= prev_path:
                raise ValueError("[0302] pathID not ascending in package")
            prev_path = pid
            seen_path.add(pid)

            rel = im.get("relatedMission")
            if not isinstance(rel, dict):
                raise ValueError(f"[0302] IM {im_id}: relatedMission invalid")
            rt = _to_int(rel.get("relatedMissionType"), 0)
            iid = _to_int(rel.get("inputMissionID"), 0)
            prior = _to_int(rel.get("priorMissionID"), 0)
            if rt not in (1, 2):
                raise ValueError(f"[0302] IM {im_id}: relatedMissionType must be 1 or 2")
            if iid <= 0:
                raise ValueError(f"[0302] IM {im_id}: inputMissionID must be positive")
            if rt == 1 and prior != 0:
                raise ValueError(f"[0302] IM {im_id}: priorMissionID must be 0 when relatedMissionType=1")
            if rt == 2 and prior <= 0:
                raise ValueError(f"[0302] IM {im_id}: priorMissionID must be positive when relatedMissionType=2")

            info = im.get("individualMissionInfo")
            if not isinstance(info, dict):
                raise ValueError(f"[0302] IM {im_id}: individualMissionInfo invalid")
            mtype = _to_int(info.get("individualMissionType"), 0)
            if mtype in _MISSION_NEEDS_COORD and not info.get("coordinateList"):
                raise ValueError(f"[0302] IM {im_id}: coordinateList required")
            if mtype in _MISSION_NEEDS_LINE and not info.get("lineList"):
                raise ValueError(f"[0302] IM {im_id}: lineList required")
            if mtype in _MISSION_NEEDS_AREA and not info.get("areaList"):
                raise ValueError(f"[0302] IM {im_id}: areaList required")
            if mtype in _MISSION_NEEDS_TARGET and not info.get("targetID"):
                raise ValueError(f"[0302] IM {im_id}: targetID required")


def _related_input_mission_id(mission: Dict[str, Any]) -> int:
    related = (
        mission.get("relatedMission")
        if isinstance(mission.get("relatedMission"), dict)
        else {}
    )
    return _to_int(related.get("inputMissionID"), 0)


def _apply_input_order_execution_barrier(
    packages: List[Dict[str, Any]],
    *,
    input_plan: Dict[str, Any],
    target_input_id: int,
) -> Dict[str, int]:
    """Open one UAV input group and lock every later group.

    Input mission IDs are identifiers, not a sortable execution sequence.  The
    authoritative order is therefore the order in ``InputMissionPlan``.  This
    helper intentionally leaves earlier and unknown missions untouched.
    """

    input_order: Dict[int, int] = {}
    for index, row in enumerate(input_plan.get("inputMissionList") or []):
        if not isinstance(row, dict):
            continue
        input_id = _to_int(row.get("inputMissionID"), 0)
        if input_id > 0 and input_id not in input_order:
            input_order[int(input_id)] = int(index)

    target_index = input_order.get(int(target_input_id))
    if target_index is None:
        return {"targetUnblocked": 0, "laterBlocked": 0}

    target_unblocked = 0
    later_blocked = 0
    for package in packages:
        if not isinstance(package, dict) or _to_int(package.get("aircraftID"), 0) <= 3:
            continue
        missions = package.get("individualMissionList")
        if not isinstance(missions, list):
            continue
        for mission in missions:
            if not isinstance(mission, dict):
                continue
            input_id = _related_input_mission_id(mission)
            mission_index = input_order.get(int(input_id))
            if mission_index is None:
                continue
            if int(mission_index) == int(target_index):
                mission.pop("executionBlockedUntilNextCollab", None)
                mission.pop("ExecutionBlockedUntilNextCollab", None)
                target_unblocked += 1
            elif int(mission_index) > int(target_index):
                mission["executionBlockedUntilNextCollab"] = True
                mission.pop("ExecutionBlockedUntilNextCollab", None)
                later_blocked += 1

    return {
        "targetUnblocked": int(target_unblocked),
        "laterBlocked": int(later_blocked),
    }


def _apply_initial_type2_line_execution_barrier(
    packages: List[Dict[str, Any]],
    *,
    input_plan: Dict[str, Any],
) -> Optional[int]:
    """Lock the initial UAV suffix at the first exact Type-2 branch LINE.

    A valid Type-2 self-reliance package has an outbound LINE, guard AREA, and
    return LINE.  The first represented LINE is the only initially open group;
    each next-collaboration replan advances the barrier one input group at a
    time.  Stale or malformed Type-2 lookalikes are ignored by the strict phase
    resolver.
    """

    represented_input_ids = {
        _related_input_mission_id(mission)
        for package in packages
        if isinstance(package, dict) and _to_int(package.get("aircraftID"), 0) > 3
        for mission in (package.get("individualMissionList") or [])
        if isinstance(mission, dict)
    }
    type2_line_phases = {
        TYPE2_SELF_RELIANCE_OUTBOUND_LINE,
        TYPE2_SELF_RELIANCE_RETURN_LINE,
    }
    for input_mission in input_plan.get("inputMissionList") or []:
        if not isinstance(input_mission, dict):
            continue
        input_id = _to_int(input_mission.get("inputMissionID"), 0)
        if input_id <= 0 or input_id not in represented_input_ids:
            continue
        phase = resolve_type2_self_reliance_phase(input_plan, int(input_id))
        if phase not in type2_line_phases:
            continue
        _apply_input_order_execution_barrier(
            packages,
            input_plan=input_plan,
            target_input_id=int(input_id),
        )
        return int(input_id)
    return None


def build_0302_packages_from_split(
    split_result: SplitRunResult,
    source: Optional[str] = None,
    planning_mode: MissionPlanningMode | Dict[str, Any] | None = None,
) -> List[Dict[str, Any]]:
    _ = planning_mode
    with _runtime_cache_scope():
        return _build_0302_packages_from_split_impl(split_result, source=source)


def _build_0302_packages_from_split_impl(split_result: SplitRunResult, source: Optional[str] = None) -> List[Dict[str, Any]]:
    grouped = _collect_export_pieces(split_result)
    if not grouped:
        raise ValueError("No assigned split pieces to export.")

    ts = _now_ms_since_2000()
    src = source or _sw_code()
    next_im_id = 900_000_001
    packages: List[Dict[str, Any]] = []

    for aid in sorted(grouped):
        pieces = list(grouped[aid])
        inserted_rows = _inserted_for_uav(split_result, aid)
        mission_id_by_parent = _parent_mission_id_map(split_result, aid)
        path_id = _path_base(aid)
        im_list: List[Dict[str, Any]] = []

        events: List[Tuple[float, int, int, str, Any]] = []
        for i, piece in enumerate(pieces):
            st = _piece_schedule_start(piece)
            if st is None:
                st = 1e9 + float(i)
            events.append((float(st), 1, int(i), "piece", piece))
        for j, row in enumerate(inserted_rows):
            st = _to_float(row.get("startSec"), 0.0)
            events.append((float(st), 0, int(j), "inserted", row))
        events.sort(key=lambda x: (x[0], x[1], x[2]))

        guard_rows_by_mission: Dict[Tuple[str, str], List[Dict[str, Any]]] = {}
        for _, _, _, event_type, event_payload in events:
            if event_type != "piece" or not isinstance(event_payload, SplitPiece):
                continue
            data = event_payload.data if isinstance(event_payload.data, dict) else {}
            if data.get("_type2BoundaryGuardArea") is not True:
                continue
            package_token = str(data.get("_type2BoundaryGuardPackageID") or "0")
            mission_token = str(
                data.get("_type2BoundaryGuardInputMissionID")
                or event_payload.mission_id
                or event_payload.parent_order
            )
            guard_rows_by_mission.setdefault(
                (package_token, mission_token), []
            ).append(data)

        guard_duration_s = _to_float(
            get_runtime_value("type2_boundary_guard_duration_s", 600.0),
            600.0,
        )
        for (package_token, mission_token), guard_rows in guard_rows_by_mission.items():
            annotate_boundary_guard_set(
                guard_rows,
                set_id=(
                    f"type2-boundary:{package_token}:{mission_token}:aircraft-{int(aid)}"
                ),
                duration_s=guard_duration_s,
            )

        for _, _, _, etype, payload in events:
            runtime_meta: Dict[str, Any] = {}
            if etype == "piece":
                piece = payload
                mission_id = _as_pos_int(piece.mission_id, fallback=piece.parent_order)
                im_info = _piece_to_mission_info(piece)
                runtime_meta = _piece_runtime_meta(piece)
            else:
                row = payload if isinstance(payload, dict) else {}
                before_parent = _to_int(row.get("beforeParentOrder"), 0)
                fallback_mid = mission_id_by_parent.get(before_parent, before_parent if before_parent > 0 else 1)
                mission_id = _as_pos_int(row.get("parentMissionID"), fallback=fallback_mid)
                im_info = _inserted_to_mission_info(row)
            entry = {
                "individualMissionID": int(next_im_id),
                "isDone": False,
                "relatedMission": {
                    "relatedMissionType": 1,
                    "inputMissionID": int(mission_id),
                    "priorMissionID": 0,
                },
                "individualMissionInfo": im_info,
                "pathID": int(path_id),
            }
            if runtime_meta:
                entry.update(runtime_meta)
            im_list.append(entry)
            next_im_id += 1
            path_id += 1

        packages.append(
            {
                "timestamp": int(ts),
                "Source": src,
                "individualMissionPackageID": int(_package_id(aid)),
                "aircraftID": int(aid),
                "individualMissionList": im_list,
            }
        )

    _validate_0302_packages(packages)
    return packages


def _build_lah_0302_packages_from_cmpk(
    cmpk: Dict[str, Any],
    *,
    source: Optional[str],
    timestamp_ms: int,
    start_im_id: int,
    planning_mode: MissionPlanningMode | Dict[str, Any] | None = None,
) -> Tuple[List[Dict[str, Any]], int]:
    avail = cmpk.get("availableAircraftList")
    if not isinstance(avail, list):
        return [], start_im_id

    lah_ids: List[int] = []
    for row in avail:
        aid = _extract_aircraft_id(row)
        if aid is None:
            continue
        if 1 <= int(aid) <= 3:
            lah_ids.append(int(aid))
    lah_ids = sorted(set(lah_ids))
    if not lah_ids:
        return [], start_im_id

    missions = cmpk.get("inputMissionList")
    if not isinstance(missions, list):
        return [], start_im_id

    src = source or _sw_code()
    packages_by_aid: Dict[int, Dict[str, Any]] = {}
    next_path_by_aid: Dict[int, int] = {}
    for aid in lah_ids:
        packages_by_aid[aid] = {
            "timestamp": int(timestamp_ms),
            "Source": src,
            "individualMissionPackageID": int(_package_id(aid)),
            "aircraftID": int(aid),
            "individualMissionList": [],
        }
        next_path_by_aid[aid] = int(_path_base(aid))

    next_im_id = int(start_im_id)
    special_rows = build_lah_special_sequence(missions, planning_mode=planning_mode)
    if not special_rows:
        # Type 2/3 (지상기동 보장/공중강습 엄호): manned hold/escort/follow sequence.
        special_rows = build_ground_maneuver_lah_sequence(
            cmpk, planning_mode=planning_mode, package_type=cmpk.get("inputMissionPackageType")
        )
    if not special_rows:
        # Type 5 (도시지역 작전): manned staging at 공격대기지역, follow-out.
        special_rows = build_urban_operation_lah_sequence(
            cmpk, planning_mode=planning_mode, package_type=cmpk.get("inputMissionPackageType")
        )
    if special_rows:
        for row in special_rows:
            input_mid = _as_pos_int(row.get("inputMissionID"), 1)
            info = row.get("individualMissionInfo") if isinstance(row.get("individualMissionInfo"), dict) else None
            if not info:
                continue
            for aid in lah_ids:
                pkg = packages_by_aid[aid]
                pid = next_path_by_aid[aid]
                pkg["individualMissionList"].append(
                    {
                        "individualMissionID": int(next_im_id),
                        "isDone": False,
                        "relatedMission": {
                            "relatedMissionType": 1,
                            "inputMissionID": int(input_mid),
                            "priorMissionID": 0,
                        },
                        "individualMissionInfo": copy.deepcopy(info),
                        "pathID": int(pid),
                    }
                )
                next_im_id += 1
                next_path_by_aid[aid] = int(pid + 1)

        out = []
        for aid in lah_ids:
            pkg = packages_by_aid[aid]
            if not pkg["individualMissionList"]:
                continue
            out.append(pkg)
        return out, next_im_id

    for idx, im in enumerate(missions, start=1):
        if not isinstance(im, dict):
            continue
        geometry = mission_geometry(im)
        input_mid = _as_pos_int(im.get("inputMissionID"), idx)
        detail = im.get("missionDetail") if isinstance(im.get("missionDetail"), dict) else {}

        if geometry == "line":
            line_list = detail.get("lineList")
            if not isinstance(line_list, list):
                continue
            for seg in line_list:
                if not isinstance(seg, dict):
                    continue
                coords = _normalize_coord_list(seg.get("coordinateList"))
                if not coords:
                    continue
                im_type = 7
                pattern = 10
                for aid in lah_ids:
                    pkg = packages_by_aid[aid]
                    pid = next_path_by_aid[aid]
                    pkg["individualMissionList"].append(
                        {
                            "individualMissionID": int(next_im_id),
                            "isDone": False,
                            "relatedMission": {
                                "relatedMissionType": 1,
                                "inputMissionID": int(input_mid),
                                "priorMissionID": 0,
                            },
                            "individualMissionInfo": {
                                "individualMissionType": int(im_type),
                                "patternType": int(pattern),
                                "autoZoomIn": False,
                                "coordinateList": coords,
                                "targetID": None,
                            },
                            "pathID": int(pid),
                        }
                    )
                    next_im_id += 1
                    next_path_by_aid[aid] = int(pid + 1)
            continue

        if geometry == "area":
            area_list = detail.get("areaList")
            if not isinstance(area_list, list) or not area_list:
                continue
            area0 = area_list[0] if isinstance(area_list[0], dict) else {}
            coords = _normalize_coord_list(area0.get("coordinateList"))
            if not coords:
                continue
            p0 = dict(coords[0])
            for aid in lah_ids:
                pkg = packages_by_aid[aid]
                pid = next_path_by_aid[aid]
                pkg["individualMissionList"].append(
                    {
                        "individualMissionID": int(next_im_id),
                        "isDone": False,
                        "relatedMission": {
                            "relatedMissionType": 1,
                            "inputMissionID": int(input_mid),
                            "priorMissionID": 0,
                        },
                        "individualMissionInfo": {
                            "individualMissionType": 9,
                            "patternType": 12,
                            "autoZoomIn": False,
                            "coordinateList": [p0],
                            "targetID": None,
                        },
                        "pathID": int(pid),
                    }
                )
                next_im_id += 1
                next_path_by_aid[aid] = int(pid + 1)

    out = []
    for aid in lah_ids:
        pkg = packages_by_aid[aid]
        if not pkg["individualMissionList"]:
            continue
        out.append(pkg)
    return out, next_im_id


def build_0302_packages_from_split_with_lah(
    split_result: SplitRunResult,
    cmpk: Optional[Dict[str, Any]] = None,
    source: Optional[str] = None,
    planning_mode: MissionPlanningMode | Dict[str, Any] | None = None,
) -> List[Dict[str, Any]]:
    resolved_mode = (
        planning_mode
        or getattr(split_result, "planning_mode", None)
        or (resolve_mission_planning_mode(cmpk) if isinstance(cmpk, dict) else None)
    )
    split_result.planning_mode = mission_mode_context(mode=resolved_mode)
    uav_packages = build_0302_packages_from_split(
        split_result,
        source=source,
        planning_mode=resolved_mode,
    )
    if not isinstance(cmpk, dict):
        return uav_packages

    _apply_initial_type2_line_execution_barrier(
        uav_packages,
        input_plan=cmpk,
    )

    max_im_id = 900_000_000
    for pkg in uav_packages:
        for im in pkg.get("individualMissionList", []):
            max_im_id = max(max_im_id, _to_int(im.get("individualMissionID"), 0))

    ts = _now_ms_since_2000()
    lah_packages, _ = _build_lah_0302_packages_from_cmpk(
        cmpk,
        source=source,
        timestamp_ms=ts,
        start_im_id=max_im_id + 1,
        planning_mode=resolved_mode,
    )
    if not lah_packages:
        return uav_packages

    combined = sorted(lah_packages + uav_packages, key=lambda p: _to_int(p.get("aircraftID"), 0))
    _validate_0302_packages(combined)
    return combined


def save_0302_packages(packages: List[Dict[str, Any]], out_dir: str | Path) -> List[Path]:
    root = Path(out_dir)
    root.mkdir(parents=True, exist_ok=True)
    rows: List[tuple[Path, Dict[str, Any]]] = []
    for pkg in packages:
        aid = int(pkg.get("aircraftID", 0) or 0)
        if 1 <= aid <= 3:
            path = root / f"IndividualMissionPlan_LAH{aid:03d}.json"
        else:
            path = root / f"IndividualMissionPlan_UAV{aid}.json"
        rows.append((path, pkg))
    _write_package_rows(rows)
    written = [path for path, _ in rows]
    return written


def _write_package_rows(rows: List[tuple[Path, Dict[str, Any]]]) -> None:
    if not rows:
        return

    def _write_one(item: tuple[Path, Dict[str, Any]]) -> None:
        path, payload = item
        if dumps_json is not None:
            path.write_bytes(dumps_json(payload, pretty=True, ensure_ascii=False))
        else:
            path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    if len(rows) == 1:
        _write_one(rows[0])
        return

    with concurrent.futures.ThreadPoolExecutor(max_workers=min(8, len(rows)), thread_name_prefix="Save0302") as executor:
        futures = [executor.submit(_write_one, row) for row in rows]
        for future in concurrent.futures.as_completed(futures):
            future.result()
