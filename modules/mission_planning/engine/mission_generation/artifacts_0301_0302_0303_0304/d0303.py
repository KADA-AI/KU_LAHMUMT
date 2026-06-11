from __future__ import annotations
import os
import math
import csv
import sys
import threading
import time
import concurrent.futures
from collections import OrderedDict
from contextlib import contextmanager, nullcontext
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Optional, Tuple

_CANONICAL_NAME = "modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304.d0303"
_ALIASES = (
    "modules.mission_planning.MissionPlanner.data_def.d0303",
    "data_def.d0303",
)
if __name__ == _CANONICAL_NAME:
    for _ALIAS in _ALIASES:
        sys.modules.setdefault(_ALIAS, sys.modules[__name__])
elif __name__ in _ALIASES:
    sys.modules.setdefault(_CANONICAL_NAME, sys.modules[__name__])

from modules.mission_planning.MissionPlanner.data_def import mission_helpers as _mission_helpers
from modules.mission_planning.MissionPlanner.data_def.mission_helpers import (
    get_terrain_elev_many_metrics,
    now_ms_since_2000,
    reset_terrain_elev_many_metrics,
    terrain_elev,
    terrain_elev_many,
)
from modules.mission_planning.MissionPlanner.data_def.filming_altitude_guard import (
    get_filming_altitude_metrics,
    normalize_filming_target_altitudes_in_waypoints,
    reset_filming_altitude_metrics,
)
from modules.mission_planning.engine.mission_generation.id_allocation.allocator import (
    reserve_waypoint_block as _reserve_waypoint_block,
)
from modules.mission_planning.MissionPlanner.data_def.search_speed import spacing_based_search_speed
try:
    from modules.mission_planning.MissionPlanner.Search_Speed_2 import SearchSpeedCalculator as _SearchSpeedCalculator
except ImportError:
    try:
        from Search_Speed_2 import SearchSpeedCalculator as _SearchSpeedCalculator  # type: ignore
    except ImportError:
        _SearchSpeedCalculator = None

try:
    from modules.mission_planning.MissionPlanner.config import (
        DEFAULT_SWEEP_SEPARATION_M,
        USE_DB_FOR_CORRIDOR,
        SWEEP_SPACING_MARGIN,
        DB_FOV_WEIGHT,
        SEARCH_SPEED_WEIGHT,
    )
except ImportError:
    from config import (  # type: ignore
        DEFAULT_SWEEP_SEPARATION_M,
        USE_DB_FOR_CORRIDOR,
        SWEEP_SPACING_MARGIN,
        DB_FOV_WEIGHT,
        SEARCH_SPEED_WEIGHT,
    )
try:
    from modules.mission_planning.MissionPlanner.runtime_settings import (
        apply_runtime_camera_adjusted_fov_deg as _apply_runtime_camera_adjusted_fov_deg,
        get_runtime_area_auto_fov_from_db as _get_runtime_area_auto_fov_from_db,
        get_runtime_str as _get_runtime_str,
        get_runtime_area_review_max_segment_m as _get_runtime_area_review_max_segment_m,
        get_runtime_manual_fov_deg as _get_runtime_manual_fov_deg,
        get_runtime_manual_fov_sync_active as _get_runtime_manual_fov_sync_active,
        get_runtime_altitude_layers_m as _get_runtime_altitude_layers_m,
        get_runtime_value as _get_runtime_value,
        load_fov_db_rows as _load_runtime_fov_db_rows,
        load_runtime_settings as _load_runtime_settings,
        runtime_override as _runtime_settings_override,
    )
except Exception:
    try:
        from runtime_settings import (  # type: ignore
            apply_runtime_camera_adjusted_fov_deg as _apply_runtime_camera_adjusted_fov_deg,
            get_runtime_area_auto_fov_from_db as _get_runtime_area_auto_fov_from_db,
            get_runtime_str as _get_runtime_str,
            get_runtime_area_review_max_segment_m as _get_runtime_area_review_max_segment_m,
            get_runtime_manual_fov_deg as _get_runtime_manual_fov_deg,
            get_runtime_manual_fov_sync_active as _get_runtime_manual_fov_sync_active,
            get_runtime_altitude_layers_m as _get_runtime_altitude_layers_m,
            get_runtime_value as _get_runtime_value,
            load_fov_db_rows as _load_runtime_fov_db_rows,
            load_runtime_settings as _load_runtime_settings,
            runtime_override as _runtime_settings_override,
        )
    except Exception:
        _apply_runtime_camera_adjusted_fov_deg = None
        _get_runtime_area_auto_fov_from_db = None
        _get_runtime_str = None
        _get_runtime_area_review_max_segment_m = None
        _get_runtime_manual_fov_deg = None
        _get_runtime_manual_fov_sync_active = None
        _get_runtime_altitude_layers_m = None
        _get_runtime_value = None
        _load_runtime_fov_db_rows = None
        _load_runtime_settings = None
        _runtime_settings_override = None
try:
    from modules.mission_planning.MissionPlanner.data_def.dubins_turn_link import (
        Point2D as _DubinsPoint2D,
        compute_turn_link as _compute_turn_link,
    )
except Exception:
    _DubinsPoint2D = None
    _compute_turn_link = None
try:
    from modules.mission_planning.MissionPlanner.DB import select_best_config as _select_best_config
except ImportError:
    try:
        from DB import select_best_config as _select_best_config  # type: ignore
    except ImportError:
        _select_best_config = None
from modules.mission_planning.MissionPlanner.UAV_missionPlanning import UAVMissionPlanner
from modules.mission_planning.MissionPlanner.data_def import route_planner_algorithms as route_algos
from modules.mission_planning.MissionPlanner.data_def.coord_transform import llh_to_xy, xy_to_llh
try:
    from modules.mission_planning.MissionPlanner.tools.UAV_pattern.Nadir_BF.area_nadir_bf_planner import (
        build_nadir_bf_overflight_coords,
    )
except Exception:
    try:
        from tools.UAV_pattern.Nadir_BF.area_nadir_bf_planner import build_nadir_bf_overflight_coords  # type: ignore
    except Exception:
        build_nadir_bf_overflight_coords = None
try:
    from modules.common.eta import _order_by_next_chain, _time_from_prev_to_curr_s
except ImportError:
    from common.eta import _order_by_next_chain, _time_from_prev_to_curr_s  # type: ignore


def _fov_db_cache_signature() -> tuple:
    raw_path: object = None
    try:
        values = _runtime_setting_values()
        if isinstance(values, dict):
            raw_path = values.get("fov_db_path")
    except Exception:
        raw_path = None
    try:
        path = Path(str(raw_path)) if raw_path else _FOV_DB_PATH
        if not path.is_absolute():
            path = _PROJECT_ROOT / path
        resolved = path.resolve()
        stat = resolved.stat()
        return (str(resolved), int(stat.st_mtime_ns), int(stat.st_size))
    except Exception:
        return ("default",)


def _clear_fov_db_selection_caches() -> None:
    try:
        _FOV_DB_ROWS_BY_WIDTH_CACHE.clear()
        _BALANCED_FOV_DB_ROW_CACHE.clear()
        _FOV_DB_MIN_SEP_CACHE.clear()
    except Exception:
        pass


def _load_fov_db_rows() -> list[dict]:
    global _FOV_DB_ROWS_CACHE, _FOV_DB_ROWS_CACHE_SIG
    sig = _fov_db_cache_signature()
    if _FOV_DB_ROWS_CACHE is not None and _FOV_DB_ROWS_CACHE_SIG == sig:
        return _FOV_DB_ROWS_CACHE

    if _load_runtime_fov_db_rows is not None:
        try:
            rows = [dict(row) for row in _load_runtime_fov_db_rows()]
            _FOV_DB_ROWS_CACHE = rows
            _FOV_DB_ROWS_CACHE_SIG = sig
            _clear_fov_db_selection_caches()
            return rows
        except Exception:
            pass

    rows: list[dict] = []
    try:
        csv_rows: list[dict] = []
        for encoding in ("utf-8-sig", "cp949", "euc-kr"):
            try:
                with _FOV_DB_PATH.open("r", encoding=encoding, newline="") as fh:
                    csv_rows = list(csv.DictReader(fh))
                break
            except UnicodeDecodeError:
                csv_rows = []
                continue
        for row in csv_rows:
            try:
                rows.append(
                    {
                        "width": float(row.get("width", 0.0) or 0.0),
                        "vel": float(row.get("vel", 0.0) or 0.0),
                        "fov": float(row.get("fov", 0.0) or 0.0),
                        "sep": float(row.get("sep", 0.0) or 0.0),
                    }
                )
            except Exception:
                continue
    except Exception:
        rows = []
    _FOV_DB_ROWS_CACHE = rows
    _FOV_DB_ROWS_CACHE_SIG = sig
    _clear_fov_db_selection_caches()
    return rows


def _apply_db_fov_weight(fov_deg: float) -> float:
    try:
        weight = float(DB_FOV_WEIGHT)
    except Exception:
        weight = 1.0
    if weight <= 0.0:
        weight = 1.0
    try:
        fov = float(fov_deg)
    except Exception:
        fov = 0.0
    return max(fov * weight, 0.0)


def _load_runtime_settings_payload_uncached() -> Dict[str, object] | None:
    if _load_runtime_settings is None:
        return None
    try:
        payload = _load_runtime_settings()
    except Exception:
        payload = None
    return payload if isinstance(payload, dict) else None


def _runtime_settings_payload() -> Dict[str, object] | None:
    cached = getattr(_RUNTIME_SETTING_CACHE_LOCAL, "payload", None)
    if isinstance(cached, dict):
        return cached
    return _load_runtime_settings_payload_uncached()


def _runtime_setting_values() -> Dict[str, object] | None:
    cached = getattr(_RUNTIME_SETTING_CACHE_LOCAL, "values", None)
    if isinstance(cached, dict):
        return cached
    payload = _runtime_settings_payload()
    values = payload.get("values") if isinstance(payload, dict) else None
    return values if isinstance(values, dict) else None


def _runtime_setting_cache_begin() -> None:
    stack = getattr(_RUNTIME_SETTING_CACHE_LOCAL, "stack", None)
    if not isinstance(stack, list):
        stack = []
        _RUNTIME_SETTING_CACHE_LOCAL.stack = stack
    previous_payload = getattr(_RUNTIME_SETTING_CACHE_LOCAL, "payload", None)
    previous_values = getattr(_RUNTIME_SETTING_CACHE_LOCAL, "values", None)
    stack.append((
        previous_payload if isinstance(previous_payload, dict) else None,
        previous_values if isinstance(previous_values, dict) else None,
    ))
    payload = _load_runtime_settings_payload_uncached()
    values = payload.get("values") if isinstance(payload, dict) else None
    _RUNTIME_SETTING_CACHE_LOCAL.payload = payload if isinstance(payload, dict) else {}
    _RUNTIME_SETTING_CACHE_LOCAL.values = values if isinstance(values, dict) else {}


def _runtime_setting_cache_end() -> None:
    stack = getattr(_RUNTIME_SETTING_CACHE_LOCAL, "stack", None)
    previous_payload = None
    previous_values = None
    if isinstance(stack, list) and stack:
        previous_payload, previous_values = stack.pop()
    if isinstance(previous_payload, dict):
        _RUNTIME_SETTING_CACHE_LOCAL.payload = previous_payload
    else:
        try:
            delattr(_RUNTIME_SETTING_CACHE_LOCAL, "payload")
        except Exception:
            pass
    if isinstance(previous_values, dict):
        _RUNTIME_SETTING_CACHE_LOCAL.values = previous_values
    else:
        try:
            delattr(_RUNTIME_SETTING_CACHE_LOCAL, "values")
        except Exception:
            pass


@contextmanager
def suppress_linesearch_inner_parallel():
    depth = int(getattr(_LINESEARCH_INNER_PARALLEL_SUPPRESS_LOCAL, "depth", 0) or 0)
    _LINESEARCH_INNER_PARALLEL_SUPPRESS_LOCAL.depth = depth + 1
    try:
        yield
    finally:
        if depth > 0:
            _LINESEARCH_INNER_PARALLEL_SUPPRESS_LOCAL.depth = depth
        else:
            try:
                delattr(_LINESEARCH_INNER_PARALLEL_SUPPRESS_LOCAL, "depth")
            except Exception:
                pass


def _linesearch_inner_parallel_suppressed() -> bool:
    try:
        return int(getattr(_LINESEARCH_INNER_PARALLEL_SUPPRESS_LOCAL, "depth", 0) or 0) > 0
    except Exception:
        return False


def _runtime_manual_fov_sync_active(payload: Dict[str, object] | None = None) -> bool:
    if _get_runtime_manual_fov_sync_active is None:
        return False
    try:
        return bool(_get_runtime_manual_fov_sync_active(payload))
    except Exception:
        return False


def _runtime_manual_fov_deg(
    key: str,
    default: float,
    payload: Dict[str, object] | None = None,
) -> float:
    if _get_runtime_manual_fov_deg is None:
        return float(default)
    try:
        return float(_get_runtime_manual_fov_deg(key, float(default), payload))
    except Exception:
        return float(default)


def _apply_runtime_adjusted_db_fov(fov_deg: float) -> float:
    weighted_fov_deg = _apply_db_fov_weight(fov_deg)
    if _apply_runtime_camera_adjusted_fov_deg is None:
        return float(weighted_fov_deg)
    try:
        return float(_apply_runtime_camera_adjusted_fov_deg(weighted_fov_deg, _runtime_settings_payload()))
    except Exception:
        return float(weighted_fov_deg)


def _runtime_fov_db_smaller_fov_steps() -> int:
    raw: object = 3
    try:
        values = _runtime_setting_values()
        if isinstance(values, dict):
            raw = values.get("fov_db_smaller_fov_steps", raw)
        elif _get_runtime_value is not None:
            raw = _get_runtime_value("fov_db_smaller_fov_steps", raw)
    except Exception:
        raw = 3
    try:
        steps = int(float(raw))
    except Exception:
        steps = 3
    return max(0, min(20, int(steps)))


def _runtime_area_fov_db_smaller_fov_steps() -> int:
    raw: object = 1
    try:
        values = _runtime_setting_values()
        if isinstance(values, dict):
            raw = values.get("area_fov_db_smaller_fov_steps", raw)
        elif _get_runtime_value is not None:
            raw = _get_runtime_value("area_fov_db_smaller_fov_steps", raw)
    except Exception:
        raw = 1
    try:
        steps = int(float(raw))
    except Exception:
        steps = 1
    return max(0, min(20, int(steps)))


def _select_nadir_fov_by_altitude(altitude_m: float, default_fov: float) -> float:
    rows = _load_fov_db_rows()
    if not rows:
        return float(default_fov)
    alt_ref = max(0.0, float(altitude_m))
    cands = [r for r in rows if float(r.get("sep", 0.0) or 0.0) <= alt_ref + 1e-9]
    if not cands:
        return float(default_fov)
    best = max(
        cands,
        key=lambda r: (
            float(r.get("fov", 0.0) or 0.0),
            float(r.get("sep", 0.0) or 0.0),
            -abs(float(r.get("sep", 0.0) or 0.0) - alt_ref),
            float(r.get("vel", 0.0) or 0.0),
        ),
    )
    picked = float(best.get("fov", default_fov) or default_fov)
    if picked <= 0.0:
        return float(default_fov)
    return _apply_runtime_adjusted_db_fov(picked)


def _rows_by_width(rows: list[dict], width_ref_m: float) -> list[dict]:
    if not rows:
        return []
    req = max(0.0, float(width_ref_m))
    if req <= 0.0:
        return list(rows)

    cache_key = (id(rows), len(rows), round(req, 6))
    cached = _FOV_DB_ROWS_BY_WIDTH_CACHE.get(cache_key)
    if cached is not None:
        return cached

    cands = [r for r in rows if float(r.get("width", 0.0) or 0.0) + 1e-9 >= req]
    if cands:
        _FOV_DB_ROWS_BY_WIDTH_CACHE[cache_key] = cands
        return cands

    nearest_w = min(
        (float(r.get("width", 0.0) or 0.0) for r in rows),
        key=lambda w: abs(w - req),
    )
    result = [r for r in rows if abs(float(r.get("width", 0.0) or 0.0) - nearest_w) <= 1e-9]
    _FOV_DB_ROWS_BY_WIDTH_CACHE[cache_key] = result
    return result


def _db_row_quality_sep_m(row: dict | None) -> float:
    if not isinstance(row, dict):
        return 0.0
    try:
        sep_m = max(float(row.get("sep", 0.0) or 0.0), 0.0)
        width_m = max(float(row.get("width", 0.0) or 0.0), 0.0)
    except Exception:
        return 0.0
    return math.hypot(sep_m, width_m * 0.5)


def _runtime_fov_db_sep_safety_factor(payload: Dict[str, object] | None = None) -> float:
    default = 1.7
    try:
        default = float(FOV_DB_SEP_SAFETY_FACTOR)
    except Exception:
        pass
    raw: object = default
    values = payload.get("values") if isinstance(payload, dict) else None
    if isinstance(values, dict) and "fov_db_sep_safety_factor" in values:
        raw = values.get("fov_db_sep_safety_factor")
    elif _get_runtime_value is not None:
        try:
            raw = _get_runtime_value("fov_db_sep_safety_factor", default, payload)
        except TypeError:
            raw = _get_runtime_value("fov_db_sep_safety_factor", default)
        except Exception:
            raw = default
    try:
        factor = float(raw)
    except Exception:
        factor = default
    if factor <= 0.0:
        factor = default if default > 0.0 else 1.0
    return float(factor)


def _db_sep_requirement_m(sep_m: float) -> float:
    try:
        factor = _runtime_fov_db_sep_safety_factor(_runtime_settings_payload())
    except Exception:
        factor = 1.0
    return max(0.0, float(sep_m or 0.0)) * float(factor)


def _fov_db_min_sep_for_fov(fov_deg: float) -> float:
    try:
        target_fov = float(fov_deg)
    except Exception:
        return 0.0
    if target_fov <= 0.0:
        return 0.0
    rows = _load_fov_db_rows()
    if not rows:
        return 0.0
    cache_key = (id(rows), len(rows), round(target_fov, 6))
    cached = _FOV_DB_MIN_SEP_CACHE.get(cache_key)
    if cached is not None:
        return float(cached)

    matches: list[float] = []
    for row in rows:
        try:
            row_fov = float(row.get("fov", 0.0) or 0.0)
            row_sep = float(row.get("sep", 0.0) or 0.0)
        except Exception:
            continue
        if row_fov <= 0.0 or row_sep <= 0.0:
            continue
        if abs(row_fov - target_fov) <= 0.05:
            matches.append(float(row_sep))

    if not matches:
        for row in rows:
            try:
                row_fov = float(row.get("fov", 0.0) or 0.0)
                row_sep = float(row.get("sep", 0.0) or 0.0)
            except Exception:
                continue
            if row_fov <= 0.0 or row_sep <= 0.0:
                continue
            try:
                adjusted_fov = float(_apply_runtime_adjusted_db_fov(row_fov))
            except Exception:
                adjusted_fov = row_fov
            if abs(adjusted_fov - target_fov) <= 0.05:
                matches.append(float(row_sep))

    result = min(matches) if matches else 0.0
    _FOV_DB_MIN_SEP_CACHE[cache_key] = float(result)
    return float(result)


def _route_offset_sep_for_fov(fov_deg: float, default_sep_m: float) -> float:
    try:
        default_sep = float(default_sep_m)
    except Exception:
        default_sep = 0.0
    min_sep = _fov_db_min_sep_for_fov(float(fov_deg or 0.0))
    if min_sep > 0.0:
        return float(min_sep)
    return max(float(default_sep), 0.0)


def _select_balanced_fov_db_row(
    width_ref_m: float,
    min_sep_m: float = 0.0,
    *,
    smaller_fov_steps: int | None = None,
) -> dict | None:
    rows = _rows_by_width(_load_fov_db_rows(), width_ref_m)
    try:
        steps_key = -1 if smaller_fov_steps is None else int(smaller_fov_steps)
    except Exception:
        steps_key = -1
    cache_key = (
        id(rows),
        len(rows),
        round(float(width_ref_m or 0.0), 6),
        round(float(min_sep_m or 0.0), 6),
        int(steps_key),
        round(float(_runtime_fov_db_sep_safety_factor(_runtime_settings_payload())), 9),
        round(float(_FOV_SELECTION_PREFERENCE), 9),
        int(bool(_FOV_DB_PREFER_NEXT_SMALLER_AT_SAME_SEP)),
    )
    cached = _BALANCED_FOV_DB_ROW_CACHE.get(cache_key)
    if cached is not None:
        return cached

    def _remember(row: dict | None) -> dict | None:
        if row is not None:
            if len(_BALANCED_FOV_DB_ROW_CACHE) > 4096:
                _BALANCED_FOV_DB_ROW_CACHE.clear()
            _BALANCED_FOV_DB_ROW_CACHE[cache_key] = row
        return row

    min_sep = _db_sep_requirement_m(float(min_sep_m or 0.0))
    if rows and min_sep > 0.0:
        sep_rows = [
            r for r in rows
            if float(r.get("sep", 0.0) or 0.0) + 1e-9 >= min_sep
        ]
        if sep_rows:
            rows = sep_rows
    if not rows:
        return None
    if len(rows) == 1:
        return _remember(rows[0])

    width_ref = max(0.0, float(width_ref_m))
    fov_values = [float(r.get("fov", 0.0) or 0.0) for r in rows]
    vel_values = [float(r.get("vel", 0.0) or 0.0) for r in rows]
    fov_scale = max(max(fov_values) - min(fov_values), 1e-9)
    vel_scale = max(max(vel_values) - min(vel_values), 1e-9)

    max_fov_row = max(
        rows,
        key=lambda r: (
            float(r.get("fov", 0.0) or 0.0),
            float(r.get("vel", 0.0) or 0.0),
            -max(float(r.get("width", 0.0) or 0.0) - width_ref, 0.0),
            float(r.get("sep", 0.0) or 0.0),
        ),
    )
    max_vel_row = max(
        rows,
        key=lambda r: (
            float(r.get("vel", 0.0) or 0.0),
            float(r.get("fov", 0.0) or 0.0),
            -max(float(r.get("width", 0.0) or 0.0) - width_ref, 0.0),
            float(r.get("sep", 0.0) or 0.0),
        ),
    )
    target_fov = (
        (float(max_fov_row.get("fov", 0.0) or 0.0) * _FOV_SELECTION_PREFERENCE)
        + (float(max_vel_row.get("fov", 0.0) or 0.0) * (1.0 - _FOV_SELECTION_PREFERENCE))
    )
    target_vel = (
        float(max_fov_row.get("vel", 0.0) or 0.0)
        + float(max_vel_row.get("vel", 0.0) or 0.0)
    ) / 2.0

    selected = min(
        rows,
        key=lambda r: (
            (
                ((float(r.get("fov", 0.0) or 0.0) - target_fov) / fov_scale) ** 2
                + ((float(r.get("vel", 0.0) or 0.0) - target_vel) / vel_scale) ** 2
            ),
            max(float(r.get("width", 0.0) or 0.0) - width_ref, 0.0),
            -float(r.get("sep", 0.0) or 0.0),
            -float(r.get("fov", 0.0) or 0.0),
            -float(r.get("vel", 0.0) or 0.0),
        ),
    )
    if _FOV_DB_PREFER_NEXT_SMALLER_AT_SAME_SEP:
        selected = _prefer_next_smaller_fov_same_sep_row(
            rows,
            selected,
            width_ref_m=width_ref_m,
            smaller_fov_steps=smaller_fov_steps,
        )
    return _remember(selected)


def _prefer_next_smaller_fov_same_sep_row(
    rows: list[dict],
    selected: dict,
    *,
    width_ref_m: float,
    smaller_fov_steps: int | None = None,
) -> dict:
    """Prefer one lower FOV step when it does not increase SEP or reduce speed."""
    if not rows or not isinstance(selected, dict):
        return selected
    try:
        base_fov = float(selected.get("fov", 0.0) or 0.0)
        base_sep = float(selected.get("sep", 0.0) or 0.0)
        base_vel = float(selected.get("vel", 0.0) or 0.0)
    except Exception:
        return selected
    if base_fov <= 0.0 or base_sep <= 0.0:
        return selected
    target_steps = (
        _runtime_fov_db_smaller_fov_steps()
        if smaller_fov_steps is None
        else max(0, min(20, int(smaller_fov_steps)))
    )
    if target_steps <= 0:
        return selected
    lower_fovs = sorted(
        {
            float(row.get("fov", 0.0) or 0.0)
            for row in rows
            if float(row.get("fov", 0.0) or 0.0) + 1e-9 < base_fov
        },
        reverse=True,
    )
    matched_steps = 0
    fallback: dict | None = None
    width_ref = max(0.0, float(width_ref_m))
    for target_fov in lower_fovs:
        candidates = [
            row for row in rows
            if abs(float(row.get("fov", 0.0) or 0.0) - target_fov) <= 1e-9
            and float(row.get("sep", 0.0) or 0.0) <= base_sep + 1e-9
            and float(row.get("vel", 0.0) or 0.0) + 1e-9 >= base_vel
        ]
        if candidates:
            candidate = min(
                candidates,
                key=lambda row: (
                    abs(float(row.get("sep", 0.0) or 0.0) - base_sep),
                    max(float(row.get("width", 0.0) or 0.0) - width_ref, 0.0),
                    -float(row.get("vel", 0.0) or 0.0),
                    -float(row.get("sep", 0.0) or 0.0),
                ),
            )
            fallback = candidate
            matched_steps += 1
            if matched_steps >= target_steps:
                return candidate
    return fallback or selected


def _sw_code(default: str = "MMR") -> str:
    role = (os.environ.get("KU_ROLE") or "").lower()
    return {
        "mission": "MMR",
        "monitoring": "MSM",
        "decision": "MOB",
    }.get(role, default)


# ── 타입 alias (가독용) ─────────────────────────────
Point = Tuple[float, float]
Line  = Tuple[Point, Point]

# ── 고정 상수 ───────────────────────────────────────────
FOV_DEG         = 2.4
AREA_CUSTOM_FOV_DEG = 2.4
AREA_NADIR_FOV_DEG = 31.2
LINE_CLOSE_TAKEOVER_RADIUS_M = 2000.0
LINE_CLOSE_TAKEOVER_EXTEND_FIRST_SEARCH = False
LINE_CLOSE_TAKEOVER_FIRST_GROUP_SEARCH_SPEED_SCALE = 2.0
SWEEP_MERGE_HEADING_DEG = 5
AREA_SWEEP_MERGE_HEADING_DEG = 15.0
AREA_FORCE_STRAIGHT_LINESEARCH = True
SWEEP_LINE_INTERP_POINTS = 3  # >=2; controls how many sample points are emitted per sweep line
MAX_LINESEARCH_COORDS_PER_WAYPOINT = 2000
LINESEARCH_INNER_PARALLEL_MIN_STRIPS = 256
LINESEARCH_INNER_PARALLEL_MIN_COORDS = 512
LINESEARCH_INNER_PARALLEL_WORKERS = 2
FORMATION_FOLLOWER_POSTPROCESS_PARALLEL_MIN_FOLLOWERS = 2
FORMATION_FOLLOWER_POSTPROCESS_WORKERS = 2
Altitude = 1000
ALTITUDE_LAYERS_M = (1000.0, 1010.0, 1020.0)
UAV_CLIMB_RATE_MPS: float = 5.0  # UAV vertical climb rate used for WP altitude reachability constraint
FILMING_TARGET_ALTITUDE_FLOOR_CLEARANCE_M = 30.0
DEFAULT_SEARCH_SPEED_MULTIPLIER = 16.0
LINE_SEARCH_SPEED_WEIGHT = float(SEARCH_SPEED_WEIGHT)
AREA_SEARCH_SPEED_WEIGHT = 1.0
AREA_FIRST_PACKET_SEARCH_SPEED_SCALE = 1.2
AREA_FIRST_PACKET_SWEEP_GROUP_SCALE = 1.5
POINT_FOV_DEG = 31.2
MIN_SWEEP_LEN_M = 3.0
MIN_ROUTE_SPACING_M = 200.0
SWEEP_ROUTE_WP_SPACING_M = 1200.0
AREA_SWEEP_ROUTE_WP_SPACING_M = 1200.0
LINE_SWEEP_DENSITY_SCALE = 1.2
# 1.5x denser area sweep than the previous baseline (spacing ~= 2/3).
AREA_SWEEP_DENSITY_SCALE = 1.2
LINE_ROUTE_OFFSET_SCALE = 1.0
FOV_DB_SEP_SAFETY_FACTOR = 1.7
FOV_DB_QUALITY_SEP_MARGIN_M = 1.0
AREA_POINT_ANCHOR_LEAD_IN_M = 1000.0
AREA_ROUTE_OFFSET_SCALE = 1.0
AREA_OUTPUT_FOV_SCALE = 1.0
LINE_FIRST_SWEEP_DENSIFY_ENABLED = False
SWEEP_MERGE_MODE = "heading"
ENTRY_HOLD_FOV_DEG = 10.0
ENTRY_HOLD_GIMBAL_PITCH = -90.0
ENTRY_HOLD_GIMBAL_YAW = 0.0
LOITER_RADIUS_M = 800
LOITER_DIRECTION = 1
LOITER_TIME_S = 30
LOITER_SPEED_MPS = 30
DUBINS_TURN_RADIUS_M = 500.0
AREA_DUBINS_ENTRY_LINKS_ENABLED = True
SWEEP_MERGE_MODE = "all"
_PROJECT_ROOT = Path(__file__).resolve().parents[5]
_FOV_DB_PATH = _PROJECT_ROOT / "resource" / "db" / "fov_db.csv"
_FOV_DB_ROWS_CACHE: list[dict] | None = None
_FOV_DB_ROWS_CACHE_SIG: tuple | None = None
_FOV_DB_ROWS_BY_WIDTH_CACHE: dict[tuple, list[dict]] = {}
_BALANCED_FOV_DB_ROW_CACHE: dict[tuple, dict] = {}
_FOV_DB_MIN_SEP_CACHE: dict[tuple, float] = {}
_FOV_SELECTION_PREFERENCE = 0.65
_FOV_DB_PREFER_NEXT_SMALLER_AT_SAME_SEP = True
_SWEEP_DEBUG_CACHE: set[tuple[str, float, float, float]] = set()
_SWEEP_DEBUG_LOCK = threading.Lock()


def _select_corridor_db_config(width: float, min_sep_m: float = 0.0) -> dict | None:
    if not USE_DB_FOR_CORRIDOR:
        return None
    cfg = _select_balanced_fov_db_row(float(width), min_sep_m=float(min_sep_m or 0.0))
    if cfg is not None:
        return cfg
    if _select_best_config is None:
        return None
    try:
        return _select_best_config(float(width))
    except Exception:
        return None


def _runtime_auto_fov_from_db(default: bool = True) -> bool:
    payload = None
    if _load_runtime_settings is not None:
        try:
            payload = _load_runtime_settings()
        except Exception:
            payload = None
    if _get_runtime_value is None:
        return bool(default)
    try:
        return bool(_get_runtime_value("enhanced_auto_fov_from_db", default, payload))
    except TypeError:
        try:
            return bool(_get_runtime_value("enhanced_auto_fov_from_db", default))
        except Exception:
            return bool(default)
    except Exception:
        return bool(default)


def _runtime_area_auto_fov_from_db(default: bool = True) -> bool:
    if _get_runtime_area_auto_fov_from_db is not None:
        try:
            return bool(_get_runtime_area_auto_fov_from_db(default))
        except Exception:
            pass
    return _runtime_auto_fov_from_db(default)


def _runtime_area_review_max_segment_m(default: float = 3000.0) -> float:
    if _get_runtime_area_review_max_segment_m is not None:
        try:
            return float(_get_runtime_area_review_max_segment_m(default))
        except Exception:
            pass
    if _get_runtime_value is None:
        return float(default)
    try:
        value = float(_get_runtime_value("enhanced_area_review_max_segment_m", default))
    except Exception:
        value = float(default)
    return max(value, 0.0)


def _runtime_uav_climb_rate_mps() -> float:
    if _get_runtime_value is None:
        return float(UAV_CLIMB_RATE_MPS)
    try:
        value = float(_get_runtime_value("uav_climb_rate_mps", UAV_CLIMB_RATE_MPS))
    except Exception:
        value = float(UAV_CLIMB_RATE_MPS)
    return max(value, 0.1)


def _db_velocity_to_cruise_speed_mps(vel_kmh: float) -> float:
    try:
        return round(max(float(vel_kmh), 0.0) / 3.6, 2)
    except Exception:
        return 0.0


def _projected_polygon_span_m(poly_llh: list[tuple[float, float]], bearing_deg: float) -> float:
    if len(poly_llh) < 3:
        return 0.0
    lat0, lon0 = poly_llh[0]
    poly_xy = [llh_to_xy(lat, lon, lat0, lon0) for lat, lon in poly_llh]
    th = math.radians(float(bearing_deg))
    nx, ny = math.cos(th), -math.sin(th)
    vals = [float(nx * x + ny * y) for x, y in poly_xy]
    if not vals:
        return 0.0
    return max(vals) - min(vals)


def _select_area_db_config(poly_llh: list[tuple[float, float]], bearing_deg: float) -> dict | None:
    if not _runtime_area_auto_fov_from_db():
        return None
    span_m = _projected_polygon_span_m(poly_llh, bearing_deg)
    if span_m <= 0.0:
        return None
    max_segment_m = _runtime_area_review_max_segment_m()
    width_ref_m = span_m
    if max_segment_m > 0.0:
        width_ref_m = min(width_ref_m, max_segment_m)
    width_ref_m = max(float(width_ref_m), 1.0)
    cfg = _select_balanced_fov_db_row(
        width_ref_m,
        smaller_fov_steps=_runtime_area_fov_db_smaller_fov_steps(),
    )
    if cfg is None:
        if _select_best_config is None:
            return None
        try:
            cfg = _select_best_config(width_ref_m)
        except Exception:
            return None
    if not isinstance(cfg, dict):
        return None
    return {
        "config": cfg,
        "span_m": float(span_m),
        "width_ref_m": float(width_ref_m),
        "max_segment_m": float(max_segment_m),
    }


def _normalize_altitude(value: Optional[float], default: int = Altitude) -> int:
    """고도를 정수(m)로 정규화."""
    if value is None:
        return int(default)
    try:
        alt = float(value)
    except (TypeError, ValueError):
        alt = float(default)
    return int(round(alt))


@dataclass(frozen=True)
class SweepConfig:
    separation_m: float
    fov_deg: float

SWEEP_GEOMETRY = SweepConfig(
    separation_m=DEFAULT_SWEEP_SEPARATION_M,
    fov_deg=FOV_DEG,
)

def _active_sweep_geometry() -> SweepConfig:
    return SWEEP_GEOMETRY


def _plan_route_points(
    base: list[tuple[float, float]],
    *,
    cruise_speed: float,
    heading_tol_deg: float,
) -> list[dict]:
    _ = heading_tol_deg
    return route_algos.plan_route_linear(base, cruise_speed=cruise_speed)


def _resample_polyline_xy(
    points_xy: list[tuple[float, float]],
    spacing_m: float,
) -> list[tuple[float, float]]:
    if len(points_xy) <= 1:
        return list(points_xy)
    try:
        spacing = max(float(spacing_m), 1.0)
    except Exception:
        spacing = 500.0

    seg_lengths: list[float] = []
    cumulative: list[float] = [0.0]
    total = 0.0
    for idx in range(len(points_xy) - 1):
        x0, y0 = points_xy[idx]
        x1, y1 = points_xy[idx + 1]
        seg_len = math.hypot(float(x1) - float(x0), float(y1) - float(y0))
        seg_lengths.append(seg_len)
        total += seg_len
        cumulative.append(total)

    if total <= spacing * 1.25:
        return [points_xy[0], points_xy[-1]]

    targets: list[float] = [0.0]
    cursor = spacing
    while cursor < total:
        targets.append(float(cursor))
        cursor += spacing
    if total - targets[-1] > 1e-6:
        targets.append(float(total))

    samples: list[tuple[float, float]] = []
    seg_idx = 0
    for target in targets:
        if target >= total:
            samples.append(points_xy[-1])
            continue
        while seg_idx < len(seg_lengths) - 1 and cumulative[seg_idx + 1] < target:
            seg_idx += 1
        start = points_xy[seg_idx]
        end = points_xy[seg_idx + 1]
        seg_start = cumulative[seg_idx]
        seg_len = max(seg_lengths[seg_idx], 1e-9)
        ratio = max(0.0, min(1.0, (target - seg_start) / seg_len))
        x = float(start[0]) + (float(end[0]) - float(start[0])) * ratio
        y = float(start[1]) + (float(end[1]) - float(start[1])) * ratio
        samples.append((x, y))

    deduped: list[tuple[float, float]] = []
    for point in samples:
        if not deduped:
            deduped.append(point)
            continue
        if math.hypot(point[0] - deduped[-1][0], point[1] - deduped[-1][1]) >= 1.0:
            deduped.append(point)
    if deduped[-1] != points_xy[-1]:
        deduped.append(points_xy[-1])
    return deduped


def _sweep_spacing_m(*, separation_m: float, fov_deg: float, spacing_scale: float = 1.0) -> float:
    """Return physical spacing between sweep strips in meters."""
    base = 2.0 * max(separation_m, 1.0) * math.tan(max(math.radians(fov_deg) / 2.0, 1e-6))
    try:
        margin = float(SWEEP_SPACING_MARGIN)
    except Exception:
        margin = 1.0
    if margin <= 0:
        margin = 1.0
    try:
        effective_scale = float(spacing_scale)
    except Exception:
        effective_scale = 1.0
    if effective_scale <= 0.0:
        effective_scale = 1.0
    return max(base * margin * effective_scale, 1.0)


def _debug_sweep(label: str, *, separation: float, fov: float, spacing: float) -> None:
    """Lightweight debug printer for sweep spacing."""
    try:
        key = (
            str(label),
            round(float(separation), 2),
            round(float(fov), 2),
            round(float(spacing), 2),
        )
        with _SWEEP_DEBUG_LOCK:
            if key in _SWEEP_DEBUG_CACHE:
                return
            _SWEEP_DEBUG_CACHE.add(key)
        approx = max(int(round(1000.0 / max(spacing, 1e-6))), 1)
        print(
            f"[SWEEP][{label}] separation={separation:.2f}m, fov={fov:.2f}°, "
            f"spacing={spacing:.2f}m (~{approx} strips/1km)"
        )
    except Exception:
        pass


def _coord_to_xy(coord: dict, ref: dict) -> Tuple[float, float]:
    lat0 = float(ref.get("latitude", 0.0))
    lon0 = float(ref.get("longitude", 0.0))
    lat = float(coord.get("latitude", lat0))
    lon = float(coord.get("longitude", lon0))
    return llh_to_xy(lat, lon, lat0, lon0)


def _xy_to_coord(x: float, y: float, ref: dict, *, altitude: Optional[float] = None) -> Dict[str, float]:
    lat0 = float(ref.get("latitude", 0.0))
    lon0 = float(ref.get("longitude", 0.0))
    lat, lon = xy_to_llh(x, y, lat0, lon0)
    alt_source = altitude if altitude is not None else ref.get("altitude", Altitude)
    alt = _normalize_altitude(alt_source)
    return {
        "latitude": round(lat, 6),
        "longitude": round(lon, 6),
        "altitude": alt,
    }


def _unit_vec(vec: Tuple[float, float]) -> Tuple[float, float]:
    mag = math.hypot(vec[0], vec[1])
    if mag <= 1e-6:
        return (0.0, 0.0)
    return (vec[0] / mag, vec[1] / mag)


def _heading_deg(vec: Tuple[float, float]) -> float:
    return (math.degrees(math.atan2(vec[1], vec[0])) + 360.0) % 360.0


def _wrap_delta(deg: float) -> float:
    return ((deg + 180.0) % 360.0) - 180.0


_TAU = 2.0 * math.pi


def _mod2pi(x: float) -> float:
    return x - _TAU * math.floor(x / _TAU)


def _dubins_LSL(alpha: float, beta: float, d: float):
    tmp0 = d + math.sin(alpha) - math.sin(beta)
    p2 = 2 + d * d - 2 * math.cos(alpha - beta) + 2 * d * (math.sin(alpha) - math.sin(beta))
    if p2 < -1e-12:
        return None
    p = math.sqrt(max(p2, 0.0))
    tmp1 = math.atan2((math.cos(beta) - math.cos(alpha)), tmp0)
    t = _mod2pi(-alpha + tmp1)
    q = _mod2pi(beta - tmp1)
    return t, p, q


def _dubins_RSR(alpha: float, beta: float, d: float):
    tmp0 = d - math.sin(alpha) + math.sin(beta)
    p2 = 2 + d * d - 2 * math.cos(alpha - beta) + 2 * d * (-math.sin(alpha) + math.sin(beta))
    if p2 < -1e-12:
        return None
    p = math.sqrt(max(p2, 0.0))
    tmp1 = math.atan2((math.cos(alpha) - math.cos(beta)), tmp0)
    t = _mod2pi(alpha - tmp1)
    q = _mod2pi(-beta + tmp1)
    return t, p, q


def _dubins_LSR(alpha: float, beta: float, d: float):
    p2 = -2 + d * d + 2 * math.cos(alpha - beta) + 2 * d * (math.sin(alpha) + math.sin(beta))
    if p2 < -1e-12:
        return None
    p = math.sqrt(max(p2, 0.0))
    tmp2 = (
        math.atan2((-math.cos(alpha) - math.cos(beta)), (d + math.sin(alpha) + math.sin(beta)))
        - math.atan2(-2.0, p)
    )
    t = _mod2pi(-alpha + tmp2)
    q = _mod2pi(-beta + tmp2)
    return t, p, q


def _dubins_RSL(alpha: float, beta: float, d: float):
    p2 = -2 + d * d + 2 * math.cos(alpha - beta) - 2 * d * (math.sin(alpha) + math.sin(beta))
    if p2 < -1e-12:
        return None
    p = math.sqrt(max(p2, 0.0))
    tmp2 = (
        math.atan2((math.cos(alpha) + math.cos(beta)), (d - math.sin(alpha) - math.sin(beta)))
        - math.atan2(2.0, p)
    )
    t = _mod2pi(alpha - tmp2)
    q = _mod2pi(beta - tmp2)
    return t, p, q


def _dubins_RLR(alpha: float, beta: float, d: float):
    tmp0 = (6 - d * d + 2 * math.cos(alpha - beta) + 2 * d * (math.sin(alpha) - math.sin(beta))) / 8
    if abs(tmp0) > 1.0:
        return None
    p = _mod2pi(2 * math.pi - math.acos(tmp0))
    t = _mod2pi(
        alpha
        - math.atan2((math.cos(alpha) - math.cos(beta)), (d - math.sin(alpha) + math.sin(beta)))
        + p / 2
    )
    q = _mod2pi(alpha - beta - t + p)
    return t, p, q


def _dubins_LRL(alpha: float, beta: float, d: float):
    tmp0 = (6 - d * d + 2 * math.cos(alpha - beta) + 2 * d * (-math.sin(alpha) + math.sin(beta))) / 8
    if abs(tmp0) > 1.0:
        return None
    p = _mod2pi(2 * math.pi - math.acos(tmp0))
    t = _mod2pi(
        -alpha
        - math.atan2((math.cos(alpha) - math.cos(beta)), (d + math.sin(alpha) - math.sin(beta)))
        + p / 2
    )
    q = _mod2pi(beta - alpha - t + p)
    return t, p, q


@dataclass
class _DubinsPath:
    q0: Tuple[float, float, float]
    rho: float
    theta: float
    alpha: float
    beta: float
    d: float
    word: str
    params: Tuple[float, float, float]
    length: float


def _dubins_shortest_path(
    q0: Tuple[float, float, float],
    q1: Tuple[float, float, float],
    rho: float,
) -> Optional[_DubinsPath]:
    if rho <= 0:
        return None

    x0, y0, th0 = q0
    x1, y1, th1 = q1
    dx = x1 - x0
    dy = y1 - y0
    D = math.hypot(dx, dy)
    if D < 1e-9:
        return None

    d = D / rho
    theta = math.atan2(dy, dx)
    alpha = _mod2pi(th0 - theta)
    beta = _mod2pi(th1 - theta)

    candidates = [
        ("LSL", _dubins_LSL),
        ("LSR", _dubins_LSR),
        ("RSL", _dubins_RSL),
        ("RSR", _dubins_RSR),
        ("RLR", _dubins_RLR),
        ("LRL", _dubins_LRL),
    ]

    best: Optional[_DubinsPath] = None
    for word, fn in candidates:
        res = fn(alpha, beta, d)
        if res is None:
            continue
        t, p, q = res
        length = (t + p + q) * rho
        if best is None or length < best.length:
            best = _DubinsPath(
                q0=q0, rho=rho, theta=theta, alpha=alpha, beta=beta, d=d,
                word=word, params=(t, p, q), length=length,
            )
    return best


def _advance_segment(x: float, y: float, hd: float, seg_type: str, seg_len: float):
    if seg_type == "S":
        x += seg_len * math.cos(hd)
        y += seg_len * math.sin(hd)
    elif seg_type == "L":
        x += math.sin(hd + seg_len) - math.sin(hd)
        y += -math.cos(hd + seg_len) + math.cos(hd)
        hd = _mod2pi(hd + seg_len)
    elif seg_type == "R":
        x += -math.sin(hd - seg_len) + math.sin(hd)
        y += math.cos(hd - seg_len) - math.cos(hd)
        hd = _mod2pi(hd - seg_len)
    else:
        raise RuntimeError(f"unknown segment type: {seg_type}")
    return x, y, hd


def _dubins_point_at(path: _DubinsPath, dist: float) -> Tuple[float, float]:
    if dist <= 0.0:
        x, y = 0.0, 0.0
    else:
        ds = dist / path.rho
        x, y, hd = 0.0, 0.0, path.alpha
        for seg_type, seg_len in zip(path.word, path.params):
            if ds > seg_len:
                x, y, hd = _advance_segment(x, y, hd, seg_type, seg_len)
                ds -= seg_len
            else:
                x, y, hd = _advance_segment(x, y, hd, seg_type, ds)
                ds = 0.0
                break
    cg = math.cos(path.theta)
    sg = math.sin(path.theta)
    X = path.q0[0] + path.rho * (cg * x - sg * y)
    Y = path.q0[1] + path.rho * (sg * x + cg * y)
    return X, Y


def _dubins_three_points_xy(
    q0: Tuple[float, float, float],
    q1: Tuple[float, float, float],
    rho: float,
) -> list[Tuple[float, float]]:
    path = _dubins_shortest_path(q0, q1, rho)
    if path is None:
        mid = ((q0[0] + q1[0]) * 0.5, (q0[1] + q1[1]) * 0.5)
        return [(q0[0], q0[1]), mid, (q1[0], q1[1])]
    length = path.length
    return [
        _dubins_point_at(path, 0.0),
        _dubins_point_at(path, length * 0.5),
        _dubins_point_at(path, length),
    ]


def _dubins_three_points_llh(
    start_coord: dict,
    end_coord: dict,
    heading0_rad: float,
    heading1_rad: float,
    rho: float,
) -> list[Tuple[float, float]]:
    lat0 = float(start_coord.get("latitude", 0.0))
    lon0 = float(start_coord.get("longitude", 0.0))
    lat1 = float(end_coord.get("latitude", lat0))
    lon1 = float(end_coord.get("longitude", lon0))
    dx, dy = llh_to_xy(lat1, lon1, lat0, lon0)
    points_xy = _dubins_three_points_xy((0.0, 0.0, heading0_rad), (dx, dy, heading1_rad), rho)
    return [xy_to_llh(x, y, lat0, lon0) for x, y in points_xy]

SENSOR_NONE     = 0
SENSOR_EO_IR    = 1        # 예) EO/IR 센서

# WaypointPassType
PASS_NONE = 0
PASS_FLYBY = 1
PASS_LOITER = 2
PASS_FLYOVER = 3

# Fly-over options
FLYOVER_ENTRY_OFFSET = False
FLYOVER_DUBINS_PREFIX = False
FLYOVER_LAST_POINT = False
FLYOVER_ALL_WPS = False


def _runtime_flyover_options_from_settings() -> dict[str, bool] | None:
    payload = _runtime_settings_payload()
    flyover = payload.get("flyover") if isinstance(payload, dict) else None
    if not isinstance(flyover, dict):
        return None
    return {
        "entry_offset": bool(flyover.get("entry_offset", False)),
        "dubins_prefix": bool(flyover.get("dubins_prefix", False)),
        "last_point": bool(flyover.get("last_point", False)),
        "all_wps": bool(flyover.get("all_wps", False)),
    }


def _effective_flyover_options() -> dict[str, bool]:
    runtime_options = _runtime_flyover_options_from_settings()
    if runtime_options is not None:
        return runtime_options
    return {
        "entry_offset": bool(FLYOVER_ENTRY_OFFSET),
        "dubins_prefix": bool(FLYOVER_DUBINS_PREFIX),
        "last_point": bool(FLYOVER_LAST_POINT),
        "all_wps": bool(FLYOVER_ALL_WPS),
    }


def set_flyover_options(
    *,
    entry_offset: bool = False,
    dubins_prefix: bool = False,
    last_point: bool = False,
    all_wps: bool = False,
) -> None:
    global FLYOVER_ENTRY_OFFSET, FLYOVER_DUBINS_PREFIX, FLYOVER_LAST_POINT, FLYOVER_ALL_WPS
    FLYOVER_ENTRY_OFFSET = bool(entry_offset)
    FLYOVER_DUBINS_PREFIX = bool(dubins_prefix)
    FLYOVER_LAST_POINT = bool(last_point)
    FLYOVER_ALL_WPS = bool(all_wps)


def _turn_radius_m_for_speed(speed_mps: float) -> float:
    try:
        fixed_radius = float(DUBINS_TURN_RADIUS_M)
    except Exception:
        fixed_radius = 0.0
    if _get_runtime_value is not None:
        try:
            fixed_radius = float(_get_runtime_value("dubins_turn_radius_m", fixed_radius))
        except Exception:
            pass
    if fixed_radius > 0.0:
        return fixed_radius
    table = (
        (30.0, 340.0),
        (40.0, 450.0),
        (50.0, 560.0),
    )
    try:
        spd = float(speed_mps)
    except Exception:
        spd = 40.0
    if spd <= table[0][0]:
        return table[0][1]
    if spd >= table[-1][0]:
        return table[-1][1]
    for (s0, r0), (s1, r1) in zip(table[:-1], table[1:]):
        if s0 <= spd <= s1:
            alpha = (spd - s0) / max(1e-6, (s1 - s0))
            return r0 + (r1 - r0) * alpha
    return table[1][1]


def _dubins_link_available() -> bool:
    return bool(AREA_DUBINS_ENTRY_LINKS_ENABLED and _DubinsPoint2D is not None and _compute_turn_link is not None)


def _coord_with_altitude(
    coord: dict,
    altitude_fn: Callable[[float, float], int],
) -> OrderedDict:
    lat = float(coord.get("latitude", 0.0))
    lon = float(coord.get("longitude", 0.0))
    alt = coord.get("altitude")
    if alt is None:
        alt_i = int(altitude_fn(lat, lon))
    else:
        alt_i = _normalize_altitude(alt)
    return OrderedDict([
        ("latitude", round(lat, 6)),
        ("longitude", round(lon, 6)),
        ("altitude", int(alt_i)),
    ])


def _coord_with_dem_altitude(coord: dict) -> OrderedDict:
    """Return a coordinate whose altitude is the DEM ground elevation at that lat/lon."""
    lat = float(coord.get("latitude", 0.0))
    lon = float(coord.get("longitude", 0.0))
    return OrderedDict([
        ("latitude", round(lat, 6)),
        ("longitude", round(lon, 6)),
        ("altitude", int(_dem_alt(lat, lon))),
    ])


def _make_area_link_wp(
    coord: dict,
    speed_mps: float,
    *,
    target_coord: dict | None = None,
    fov: float | None = None,
) -> OrderedDict:
    try:
        link_fov = float(fov) if fov is not None else _runtime_manual_fov_deg("point_fov_deg", float(POINT_FOV_DEG))
    except Exception:
        link_fov = _runtime_manual_fov_deg("point_fov_deg", float(POINT_FOV_DEG))
    if link_fov <= 0.0:
        link_fov = _runtime_manual_fov_deg("point_fov_deg", float(POINT_FOV_DEG))

    filming = _mk_filming(
        operation_mode=OPMODE_HOLD,
        fov=_runtime_manual_fov_deg("entry_hold_fov_deg", float(ENTRY_HOLD_FOV_DEG)),
        sensor=SENSOR_EO_IR,
        gimbal_pitch=ENTRY_HOLD_GIMBAL_PITCH,
        gimbal_yaw=ENTRY_HOLD_GIMBAL_YAW,
    )
    if isinstance(target_coord, dict):
        try:
            target_lat = float(target_coord.get("latitude", 0.0))
            target_lon = float(target_coord.get("longitude", 0.0))
            target_orientation_coord = _coord_with_dem_altitude({
                "latitude": target_lat,
                "longitude": target_lon,
            })
            filming = OrderedDict([
                ("fieldOfView", link_fov),
                ("sensorType", SENSOR_EO_IR),
                ("operationMode", OPMODE_POINT),
                ("coordinateOrientation", OrderedDict([
                    ("coordinate", target_orientation_coord)
                ])),
            ])
        except Exception:
            pass

    return OrderedDict([
        ("waypointID", 0),
        ("_area_dubins_link", True),
        ("_flyover_dubins_prefix", True),
        ("coordinate", coord),
        ("speed", float(speed_mps)),
        ("eta", 0),
        ("ecf", 0.0),
        ("nextWaypointID", 0),
        ("waypointPassType", PASS_FLYBY),
        ("filmingProperty", filming),
    ])


def _build_area_dubins_entry_wps(
    *,
    prev_start_coord: dict,
    prev_end_coord: dict,
    next_first_coord: dict,
    next_second_coord: dict,
    turn_radius_m: float,
    cruise_speed_mps: float,
    altitude_fn: Callable[[float, float], int],
    target_coord: dict | None = None,
    target_fov: float | None = None,
    min_link_gap_m: float | None = None,
) -> list[OrderedDict]:
    if _compute_turn_link is None or _DubinsPoint2D is None:
        return []
    if not (
        isinstance(prev_start_coord, dict)
        and isinstance(prev_end_coord, dict)
        and isinstance(next_first_coord, dict)
        and isinstance(next_second_coord, dict)
    ):
        return []

    origin_lat = float(prev_end_coord.get("latitude", 0.0))
    origin_lon = float(prev_end_coord.get("longitude", 0.0))
    prev_start_x, prev_start_y = llh_to_xy(
        float(prev_start_coord.get("latitude", 0.0)),
        float(prev_start_coord.get("longitude", 0.0)),
        origin_lat,
        origin_lon,
    )
    next_first_x, next_first_y = llh_to_xy(
        float(next_first_coord.get("latitude", 0.0)),
        float(next_first_coord.get("longitude", 0.0)),
        origin_lat,
        origin_lon,
    )
    next_second_x, next_second_y = llh_to_xy(
        float(next_second_coord.get("latitude", 0.0)),
        float(next_second_coord.get("longitude", 0.0)),
        origin_lat,
        origin_lon,
    )
    if math.hypot(prev_start_x, prev_start_y) < 1.0:
        return []
    if math.hypot(next_second_x - next_first_x, next_second_y - next_first_y) < 1.0:
        return []

    try:
        result = _compute_turn_link(
            prev_start=_DubinsPoint2D(prev_start_x, prev_start_y),
            prev_end=_DubinsPoint2D(0.0, 0.0),
            next_start=_DubinsPoint2D(next_first_x, next_first_y),
            next_end=_DubinsPoint2D(next_second_x, next_second_y),
            radius_m=max(1.0, float(turn_radius_m)),
            sample_step_m=10.0,
            allow_ccc=True,
            path_policy="all_shortest",
        )
    except Exception:
        return []

    try:
        effective_min_gap_m = float(min_link_gap_m) if min_link_gap_m is not None else (float(turn_radius_m) * 0.25)
    except Exception:
        effective_min_gap_m = float(turn_radius_m) * 0.25
    effective_min_gap_m = max(120.0, effective_min_gap_m)

    candidates: list[tuple[dict, float, float]] = []
    for link_point in (result.link_point_1, result.link_point_2):
        lat, lon = xy_to_llh(link_point.x, link_point.y, origin_lat, origin_lon)
        coord = _coord_with_altitude({"latitude": lat, "longitude": lon}, altitude_fn)
        d_prev = _dist_between_coords(prev_end_coord, coord)
        d_next = _dist_between_coords(coord, next_first_coord)
        if d_prev <= 1.0 or d_next <= 1.0:
            continue
        candidates.append((coord, d_prev, d_next))

    prefix: list[OrderedDict] = []
    previous_coord = prev_end_coord
    for coord, d_prev, d_next in candidates:
        if d_prev + 1e-6 < effective_min_gap_m or d_next + 1e-6 < effective_min_gap_m:
            continue
        if _dist_between_coords(previous_coord, coord) <= 1.0:
            continue
        if prefix and _dist_between_coords(prefix[-1].get("coordinate") or {}, coord) <= 1.0:
            continue
        prefix.append(_make_area_link_wp(
            coord,
            cruise_speed_mps,
            target_coord=target_coord,
            fov=target_fov,
        ))
        previous_coord = coord
    return prefix


def _safe_float_or_none(value) -> float | None:
    try:
        result = float(value)
    except Exception:
        return None
    return result if math.isfinite(result) else None


def _dubins_transition_prefix_wps(
    *,
    start_coord: dict,
    end_coord: dict,
    start_heading_rad: float,
    end_heading_rad: float,
    rho_m: float,
    speed_mps: float,
    altitude_fn,
) -> list[OrderedDict]:
    lat0 = _safe_float_or_none(start_coord.get("latitude"))
    lon0 = _safe_float_or_none(start_coord.get("longitude"))
    lat1 = _safe_float_or_none(end_coord.get("latitude"))
    lon1 = _safe_float_or_none(end_coord.get("longitude"))
    if None in (lat0, lon0, lat1, lon1):
        return []
    points = _dubins_three_points_llh(
        {"latitude": lat0, "longitude": lon0},
        {"latitude": lat1, "longitude": lon1},
        float(start_heading_rad),
        float(end_heading_rad),
        max(1.0, float(rho_m)),
    )
    if len(points) < 3:
        return []
    prefix: list[OrderedDict] = []
    for lat, lon in points[1:-1]:
        coord = OrderedDict([
            ("latitude", round(float(lat), 6)),
            ("longitude", round(float(lon), 6)),
            ("altitude", int(altitude_fn(float(lat), float(lon)))),
        ])
        prefix.append(OrderedDict([
            ("waypointID", 0),
            ("_flyover_dubins_prefix", True),
            ("coordinate", coord),
            ("speed", float(speed_mps)),
            ("eta", 0),
            ("ecf", 0.0),
            ("nextWaypointID", 0),
            ("waypointPassType", PASS_FLYBY),
            ("filmingProperty", _mk_filming(
                operation_mode=OPMODE_HOLD,
                fov=_runtime_manual_fov_deg("entry_hold_fov_deg", float(ENTRY_HOLD_FOV_DEG)),
                sensor=SENSOR_EO_IR,
                gimbal_pitch=ENTRY_HOLD_GIMBAL_PITCH,
                gimbal_yaw=ENTRY_HOLD_GIMBAL_YAW,
            )),
        ]))
    return prefix

# OperationMode
OPMODE_NONE   = 0
OPMODE_POINT  = 1
OPMODE_LINE   = 2
OPMODE_TRACK  = 3
OPMODE_HOLD   = 4
OPMODE_SWEEP  = 5

MISSION_DISPATCH = {
    0: "없음",
    1: "표적추적", # ID 필요 -> 패턴은 1
    2: "표적공격", # 헬기만 -> ID 필요, 패턴은 2번
    3: "영역수색", # 정해진 횟수만큼 영역에 대한 촬영 계획 세움 -> 패턴은 3,4,5,6,7,8,9
    4: "영역경계", # 정해진 시간동안 영역에 대한 촬영 계획 세움-> 패턴은 3,4,5,6,7,8,9
    5: "좌표점정찰", # 점만 보는 형태 -> 패턴은 1이 될 듯
    6: "통로정찰", # 중심선 기준 좌우 스윕 -> 패턴은 4번
    7: "이동", # 이동 계획 세움 -> 카메라들은 직하방으로 고정
    8: "엄호", # TBD
    9: "은엄폐", # 헬기의 유지 포지션 설정해줌
}

PATTERN_DISPATCH = {
    0:  "없음",
    1:  "표적중심선회·추적", # 0303의 Operation Mode를 3번으로, WaypointPassType가 3 -> 비행 계획은 LoiterProperty 가 생성되어야 함, 선회반경은 1000으로 고정하고 Direction도 0 : None, 1 : CW, 2 : CCW -> Time은 5분 고정 -> Speed는 40고정
    2:  "은엄폐후공격", #유인기에서 할것 TBD
    3:  "직하방-BF촬영", # 직하방 촬영으로 고정하고 비행계획 알고리즘을 CPP 알고리즘으로 대체해서 촬영 중심점 = 비행 경로인 상황임, 한 라인찍고 다음라인을 찍기위해 도는 선회 부분이 중요함
    4:  "이격-BF촬영", # 지금과 동일한 상태
    5:  "구간왕복-BF촬영", #TBD
    6:  "선형반복주사-BF촬영", #TBD
    7:  "직상공순회촬영", #TBD
    8:  "구간중심종단-선형반복주사촬영", #TBD
    9:  "구간중심종단-자동반복주사촬영", #TBD
    10: "목적지이동", #카메라를 직하방으로 고정함 -> 지금과 동일하게 이동 경로 계획함
    11: "대상엄호", #TBD
    12: "은엄폐", #해당 점에서 고도를 최대한(안전고도까지) 낮춰서 숨어있음(유인기만 사용)
}

_DEM_ALT_CACHE_MAX = 262144
_DEM_ALT_CACHE: dict[tuple, int] = {}
_DEM_ALT_CACHE_LOCK = threading.Lock()
_DEM_ALT_CACHE_MISSING_LOCK = threading.Lock()
_DEM_ALT_RUN_MAP_LOCAL = threading.local()
_RUNTIME_SETTING_CACHE_LOCAL = threading.local()
_LINESEARCH_INNER_PARALLEL_SUPPRESS_LOCAL = threading.local()
_DEM_ALT_COORD_TRUSTED_KEY = "_demAltitudeTrusted"
_DENSE_LINESEARCH_METRICS_LOCAL = threading.local()
_GROUND_REQUIRED_CACHE_KEY = "_groundRequiredM"
_GROUND_REQUIRED_SIGNATURE_KEY = "_groundRequiredSignature"
_GROUND_REQUIRED_COORDS_ID_KEY = "_groundRequiredCoordsObjectId"
_GROUND_REQUIRED_COORDS_COUNT_KEY = "_groundRequiredCoordsCount"
_GROUND_REQUIRED_LINE_SIGNATURE_KEY = "_groundRequiredLineSignature"
_GROUND_REQUIRED_LINE_SIGNATURE_COORDS_ID_KEY = "_groundRequiredLineSignatureCoordsObjectId"
_GROUND_REQUIRED_LINE_SIGNATURE_COORDS_COUNT_KEY = "_groundRequiredLineSignatureCoordsCount"
_ALTITUDE_FLOOR_SIGNATURE_KEY = "_altitudeFloorSignature"
_GROUND_REQUIRED_PROCESS_CACHE_LOCK = threading.Lock()
_GROUND_REQUIRED_PROCESS_CACHE: "OrderedDict[tuple, float]" = OrderedDict()
_GROUND_REQUIRED_PROCESS_CACHE_INFLIGHT: dict[tuple, threading.Event] = {}
_GROUND_REQUIRED_PROCESS_CACHE_MAX = 32768
_RECOMPUTE_LINE_SEARCH_SPEED_CACHE: bool | None = None
_DENSE_LINESEARCH_MAX_METRIC_KEYS = frozenset({
    "maxLineSearchCoords",
    "lineSearchCoordinateCap",
    "innerParallelWorkers",
    "demTileCount",
    "formationPostProcessWorkers",
    "demRunMapSize",
})
_DENSE_LINESEARCH_REASON_METRIC_KEYS = frozenset({
    "demBatchFallbackReason",
    "groundRequiredFastPathRejectReason",
    "groundRequiredCoordAltitudeFastPathRejectReason",
})
_DENSE_LINESEARCH_FLOAT_METRIC_KEYS = frozenset({
    "generateLineSearchMs",
    "altitudeGuardMs",
    "altitudeApplyMs",
    "altitudeFloorMs",
    "formationLeaderBuildMs",
    "formationFollowerBuildMs",
    "formationGroupingMs",
    "missionPacketBuildMs",
    "formationDemMs",
    "formationPostProcessMs",
    "etaEcfMs",
    "searchSpeedRecalcMs",
    "filmingNormalizeMs",
    "filmingTerrainBatchMs",
    "filmingCandidateScanMs",
    "groundRequiredPreOwnerMs",
    "groundRequiredMaterializationWaitMs",
    "groundRequiredFinalPrepassFreshSkipMs",
    "groundRequiredComputeMs",
    "groundPrepassMs",
    "groundRequiredLineCoordStampMs",
    "groundRequiredLineCoordCacheSeedMs",
    "groundRequiredLineCoordCacheWarmMs",
    "groundRequiredScanMs",
    "groundRequiredHitAssignMs",
    "groundRequiredWaiterAssignMs",
    "groundRequiredOwnerComputeMs",
    "groundRequiredCacheReadMs",
    "groundRequiredCacheLockWaitMs",
    "groundRequiredSampleBuildMs",
    "groundRequiredDemLookupMs",
    "groundRequiredResultAssignMs",
    "groundRequiredProcessCacheMs",
    "groundRequiredProcessCacheInflightWaitMs",
    "demBulkLookupMs",
    "lineSearchSpeedMs",
    "lineSearchMergeMs",
    "areaPostProcessMs",
    "areaRepositionMs",
    "areaPrePackAltitudeMs",
    "areaPackMs",
    "jsonReadyMs",
    "outputNormalizeMs",
    "unaccounted0303Ms",
    "demTileResolveMs",
    "demTileCandidateIndexMs",
    "demTileCandidateAssignMs",
    "demTileApplyMs",
    "demTileFallbackScanMs",
    "demTileLoadMs",
    "demTileLoadWaitMs",
    "demNativeTransformMs",
    "demRowColTransformMs",
    "demPixelReadMs",
    "demCacheReadMs",
    "demCacheWriteMs",
    "demAltCacheReadMs",
    "demAltCacheWriteMs",
})


def reset_dense_linesearch_metrics() -> None:
    _DENSE_LINESEARCH_METRICS_LOCAL.metrics = {
        "lineSearchCoordCount": 0,
        "maxLineSearchCoords": 0,
        "lineSearchCoordinateCap": 0,
        "lineSearchCoordinateCapExceededCount": 0,
        "sweepStripCount": 0,
        "sweepEndpointCount": 0,
        "demLookupCount": 0,
        "demCacheHitCount": 0,
        "demCacheMissCount": 0,
        "groundProfileSampleCount": 0,
        "generateLineSearchMs": 0.0,
        "altitudeGuardMs": 0.0,
        "altitudeApplyMs": 0.0,
        "altitudeFloorMs": 0.0,
        "formationLeaderBuildMs": 0.0,
        "formationFollowerBuildMs": 0.0,
        "formationGroupingMs": 0.0,
        "missionPacketBuildMs": 0.0,
        "formationDemMs": 0.0,
        "formationPostProcessMs": 0.0,
        "formationPostProcessWorkers": 0,
        "formationPostProcessTasks": 0,
        "etaEcfMs": 0.0,
        "searchSpeedRecalcMs": 0.0,
        "searchSpeedRecalcCount": 0,
        "searchSpeedRecalcCoordCount": 0,
        "searchSpeedRecalcSkippedCount": 0,
        "filmingNormalizeMs": 0.0,
        "filmingTerrainBatchMs": 0.0,
        "filmingCandidateScanMs": 0.0,
        "filmingTargetCoordCount": 0,
        "filmingUniqueCoordCount": 0,
        "filmingCandidateWaypointCount": 0,
        "filmingNormalizeCallCount": 0,
        "filmingNormalizeChangedCount": 0,
        "filmingNormalizeSkippedCount": 0,
        "filmingTerrainBatchFallbackCount": 0,
        "groundRequiredComputeMs": 0.0,
        "groundPrepassMs": 0.0,
        "demBulkLookupMs": 0.0,
        "lineSearchSpeedMs": 0.0,
        "lineSearchMergeMs": 0.0,
        "areaPostProcessMs": 0.0,
        "areaRepositionMs": 0.0,
        "areaPrePackAltitudeMs": 0.0,
        "areaPackMs": 0.0,
        "jsonReadyMs": 0.0,
        "outputNormalizeMs": 0.0,
        "unaccounted0303Ms": 0.0,
        "groundRequiredPreOwnerMs": 0.0,
        "groundRequiredPreOwnerSignatureCount": 0,
        "groundRequiredPreOwnerPublishedCount": 0,
        "groundRequiredPreOwnerHitCount": 0,
        "groundRequiredPreOwnerSkippedCount": 0,
        "groundRequiredMaterializationCacheHitCount": 0,
        "groundRequiredMaterializationWaitMs": 0.0,
        "groundRequiredFinalPrepassFreshSkipCount": 0,
        "groundRequiredFinalPrepassFreshSkipMs": 0.0,
        "groundRequiredPrecomputeCount": 0,
        "groundRequiredPrecomputeHitCount": 0,
        "groundRequiredLineCoordReuseCount": 0,
        "groundRequiredSignatureCacheHitCount": 0,
        "groundRequiredSignatureCacheStoreCount": 0,
        "groundRequiredProcessCacheHitCount": 0,
        "groundRequiredProcessCacheStoreCount": 0,
        "groundRequiredProcessCacheInflightLeaderCount": 0,
        "groundRequiredProcessCacheInflightWaiterCount": 0,
        "groundRequiredProcessCacheInflightLocalFastPathCount": 0,
        "groundRequiredProcessCacheInflightTimeoutCount": 0,
        "groundRequiredProcessCacheInflightWaitMs": 0.0,
        "groundRequiredLazyFallbackCount": 0,
        "groundRequiredDenseFastPathCount": 0,
        "groundRequiredDenseFastPathCoordCount": 0,
        "groundRequiredFastPathRejectCount": 0,
        "groundRequiredFastPathRejectReason": "",
        "groundRequiredCoordAltitudeFastPathCount": 0,
        "groundRequiredTrustedCoordAltitudeFastPathCount": 0,
        "groundRequiredCoordAltitudeFastPathCoordCount": 0,
        "groundRequiredCoordAltitudeFastPathRejectCount": 0,
        "groundRequiredCoordAltitudeFastPathRejectReason": "",
        "groundRequiredTrustedCoordMarkedCount": 0,
        "groundRequiredLineCoordStampCount": 0,
        "groundRequiredLineCoordStampChangedCount": 0,
        "groundRequiredLineCoordStampMs": 0.0,
        "groundRequiredLineCoordCacheSeedCount": 0,
        "groundRequiredLineCoordCacheSeedSkippedCount": 0,
        "groundRequiredLineCoordCacheSeedMs": 0.0,
        "groundRequiredLineCoordCacheWarmCount": 0,
        "groundRequiredLineCoordCacheWarmSkippedCount": 0,
        "groundRequiredLineCoordCacheWarmMs": 0.0,
        "groundRequiredScanMs": 0.0,
        "groundRequiredHitAssignMs": 0.0,
        "groundRequiredWaiterAssignMs": 0.0,
        "groundRequiredOwnerComputeMs": 0.0,
        "groundRequiredCacheReadMs": 0.0,
        "groundRequiredCacheLockWaitMs": 0.0,
        "groundRequiredSampleBuildMs": 0.0,
        "groundRequiredDemLookupMs": 0.0,
        "groundRequiredResultAssignMs": 0.0,
        "groundRequiredProcessCacheMs": 0.0,
        "innerParallelWorkers": 0,
        "innerParallelBatches": 0,
        "innerParallelSuppressedCount": 0,
        "demTileResolveMs": 0.0,
        "demTileCandidateIndexMs": 0.0,
        "demTileCandidateAssignMs": 0.0,
        "demTileApplyMs": 0.0,
        "demTileFallbackScanMs": 0.0,
        "demTileLoadMs": 0.0,
        "demTileLoadWaitMs": 0.0,
        "demNativeTransformMs": 0.0,
        "demRowColTransformMs": 0.0,
        "demPixelReadMs": 0.0,
        "demCacheReadMs": 0.0,
        "demCacheWriteMs": 0.0,
        "demUniquePointCount": 0,
        "demTileCount": 0,
        "demTileCandidateCount": 0,
        "demTileFallbackCandidateCount": 0,
        "demTileApplyCallCount": 0,
        "demTileLoadLeaderCount": 0,
        "demTileLoadWaiterCount": 0,
        "demTileLoadTimeoutCount": 0,
        "demResolvedByTile": 0,
        "demPixelCacheHitCount": 0,
        "demPixelCacheMissCount": 0,
        "demPixelCacheUniqueMissCount": 0,
        "demPixelCacheClearCount": 0,
        "demAltCacheReadMs": 0.0,
        "demAltCacheWriteMs": 0.0,
        "demAltCacheClearCount": 0,
        "demRunMapHitCount": 0,
        "demRunMapStoreCount": 0,
        "demRunMapSize": 0,
        "demBatchFallbackCount": 0,
        "demBatchFallbackReason": "",
    }


def get_dense_linesearch_metrics(*, reset: bool = False) -> dict:
    metrics = getattr(_DENSE_LINESEARCH_METRICS_LOCAL, "metrics", None)
    if not isinstance(metrics, dict):
        reset_dense_linesearch_metrics()
        metrics = getattr(_DENSE_LINESEARCH_METRICS_LOCAL, "metrics", {})
    out = dict(metrics)
    for key in (
        "generateLineSearchMs",
        "altitudeGuardMs",
        "altitudeApplyMs",
        "altitudeFloorMs",
        "formationLeaderBuildMs",
        "formationFollowerBuildMs",
        "formationGroupingMs",
        "missionPacketBuildMs",
        "formationDemMs",
        "formationPostProcessMs",
        "etaEcfMs",
        "searchSpeedRecalcMs",
        "filmingNormalizeMs",
        "filmingTerrainBatchMs",
        "filmingCandidateScanMs",
        "groundRequiredPreOwnerMs",
        "groundRequiredMaterializationWaitMs",
        "groundRequiredFinalPrepassFreshSkipMs",
        "groundRequiredComputeMs",
        "groundPrepassMs",
        "groundRequiredLineCoordStampMs",
        "groundRequiredLineCoordCacheSeedMs",
        "groundRequiredLineCoordCacheWarmMs",
        "groundRequiredScanMs",
        "groundRequiredHitAssignMs",
        "groundRequiredWaiterAssignMs",
        "groundRequiredOwnerComputeMs",
        "groundRequiredCacheReadMs",
        "groundRequiredCacheLockWaitMs",
        "groundRequiredSampleBuildMs",
        "groundRequiredDemLookupMs",
        "groundRequiredResultAssignMs",
        "groundRequiredProcessCacheMs",
        "groundRequiredProcessCacheInflightWaitMs",
        "demBulkLookupMs",
        "lineSearchSpeedMs",
        "lineSearchMergeMs",
        "areaPostProcessMs",
        "areaRepositionMs",
        "areaPrePackAltitudeMs",
        "areaPackMs",
        "jsonReadyMs",
        "outputNormalizeMs",
        "unaccounted0303Ms",
        "demTileResolveMs",
        "demTileCandidateIndexMs",
        "demTileCandidateAssignMs",
        "demTileApplyMs",
        "demTileFallbackScanMs",
        "demTileLoadMs",
        "demTileLoadWaitMs",
        "demNativeTransformMs",
        "demRowColTransformMs",
        "demPixelReadMs",
        "demCacheReadMs",
        "demCacheWriteMs",
        "demAltCacheReadMs",
        "demAltCacheWriteMs",
    ):
        out[key] = round(float(out.get(key) or 0.0), 3)
    if reset:
        reset_dense_linesearch_metrics()
    return out


def _record_dense_linesearch_metrics(**values: float | int | str) -> None:
    metrics = getattr(_DENSE_LINESEARCH_METRICS_LOCAL, "metrics", None)
    if not isinstance(metrics, dict):
        reset_dense_linesearch_metrics()
        metrics = getattr(_DENSE_LINESEARCH_METRICS_LOCAL, "metrics", {})
    for key, value in values.items():
        if key in _DENSE_LINESEARCH_MAX_METRIC_KEYS:
            metrics[key] = max(int(metrics.get(key, 0) or 0), int(value or 0))
        elif key in _DENSE_LINESEARCH_REASON_METRIC_KEYS:
            text = str(value or "").strip()
            if text:
                existing = str(metrics.get(key) or "").strip()
                if not existing:
                    metrics[key] = text
                elif text not in set(existing.split(",")):
                    metrics[key] = existing + "," + text
        elif key in _DENSE_LINESEARCH_FLOAT_METRIC_KEYS:
            metrics[key] = float(metrics.get(key, 0.0) or 0.0) + float(value or 0.0)
        else:
            metrics[key] = int(metrics.get(key, 0) or 0) + int(value or 0)


def _max_linesearch_coords_per_waypoint() -> int:
    raw = os.environ.get("MISSION_PLAN_MAX_LINESEARCH_COORDS_PER_WAYPOINT")
    if raw is None:
        try:
            raw = _get_runtime_value("max_linesearch_coords_per_waypoint", MAX_LINESEARCH_COORDS_PER_WAYPOINT)
        except Exception:
            raw = MAX_LINESEARCH_COORDS_PER_WAYPOINT
    try:
        value = int(float(raw))
    except Exception:
        value = int(MAX_LINESEARCH_COORDS_PER_WAYPOINT)
    return max(value, 0)


def _runtime_int_setting(name: str, default: int) -> int:
    env_name = "MISSION_PLAN_" + name.upper()
    raw = os.environ.get(env_name)
    if raw is None:
        try:
            values = _runtime_setting_values()
            raw = values.get(name, default) if isinstance(values, dict) else _get_runtime_value(name, default)
        except Exception:
            raw = default
    try:
        return int(float(raw))
    except Exception:
        return int(default)


def _runtime_float_setting(name: str, default: float) -> float:
    env_name = "MISSION_PLAN_" + name.upper()
    raw = os.environ.get(env_name)
    if raw is None:
        try:
            values = _runtime_setting_values()
            raw = values.get(name, default) if isinstance(values, dict) else _get_runtime_value(name, default)
        except Exception:
            raw = default
    try:
        return float(raw)
    except Exception:
        return float(default)


def _runtime_bool_setting(name: str, default: bool) -> bool:
    env_name = "MISSION_PLAN_" + name.upper()
    raw = os.environ.get(env_name)
    if raw is None:
        try:
            values = _runtime_setting_values()
            raw = values.get(name, default) if isinstance(values, dict) else _get_runtime_value(name, default)
        except Exception:
            raw = default
    if isinstance(raw, str):
        lowered = raw.strip().lower()
        if lowered in {"1", "true", "yes", "on"}:
            return True
        if lowered in {"0", "false", "no", "off"}:
            return False
    return bool(raw)


def _recompute_line_search_speed_enabled() -> bool:
    raw = os.environ.get("MISSION_PLAN_REPLAN_0303_RECOMPUTE_LINE_SEARCH_SPEED")
    if isinstance(raw, str):
        lowered = raw.strip().lower()
        if lowered in {"1", "true", "yes", "on"}:
            return True
        if lowered in {"0", "false", "no", "off"}:
            return False
    global _RECOMPUTE_LINE_SEARCH_SPEED_CACHE
    if _RECOMPUTE_LINE_SEARCH_SPEED_CACHE is None:
        _RECOMPUTE_LINE_SEARCH_SPEED_CACHE = _runtime_bool_setting(
            "replan_0303_recompute_line_search_speed",
            False,
        )
    return bool(_RECOMPUTE_LINE_SEARCH_SPEED_CACHE)


def _runtime_override_context(payload: dict | None):
    if callable(_runtime_settings_override):
        try:
            return _runtime_settings_override(payload)
        except Exception:
            return nullcontext()
    return nullcontext()


def _ground_required_process_cache_enabled() -> bool:
    return _runtime_bool_setting("ground_required_process_cache_enabled", True)


def _ground_required_process_cache_max() -> int:
    return max(0, _runtime_int_setting("ground_required_process_cache_max", _GROUND_REQUIRED_PROCESS_CACHE_MAX))


def _ground_required_process_cache_prefix() -> tuple:
    try:
        terrain_signature = _mission_helpers.terrain_data_signature()
    except Exception:
        terrain_signature = tuple()
    runtime_version = os.environ.get("MISSION_PLAN_GROUND_REQUIRED_PROCESS_CACHE_VERSION", "default")
    return ("groundRequired:v1", terrain_signature, str(runtime_version))


def _ground_required_process_cache_key(signature: tuple, *, key_prefix: tuple | None = None) -> tuple:
    prefix = tuple(key_prefix) if key_prefix is not None else _ground_required_process_cache_prefix()
    return prefix + (signature,)


def _ground_required_process_cache_get(
    signature: tuple,
    *,
    key_prefix: tuple | None = None,
) -> float | None:
    if not _ground_required_process_cache_enabled():
        return None
    started_at = time.perf_counter()
    try:
        cache_key = _ground_required_process_cache_key(signature, key_prefix=key_prefix)
        with _GROUND_REQUIRED_PROCESS_CACHE_LOCK:
            cached = _GROUND_REQUIRED_PROCESS_CACHE.get(cache_key)
            if cached is None:
                return None
            _GROUND_REQUIRED_PROCESS_CACHE.move_to_end(cache_key)
            return float(cached)
    finally:
        _record_dense_linesearch_metrics(
            groundRequiredProcessCacheMs=(time.perf_counter() - started_at) * 1000.0,
        )


def _ground_required_process_cache_wait_timeout_s() -> float:
    timeout_ms = _runtime_float_setting("ground_required_process_cache_inflight_wait_ms", 5000.0)
    return max(0.0, float(timeout_ms) / 1000.0)


def _ground_required_process_cache_get_or_claim(
    signature: tuple,
    *,
    key_prefix: tuple | None = None,
) -> tuple[float | None, threading.Event | None, bool]:
    if not _ground_required_process_cache_enabled():
        return None, None, True
    started_at = time.perf_counter()
    try:
        cache_key = _ground_required_process_cache_key(signature, key_prefix=key_prefix)
        with _GROUND_REQUIRED_PROCESS_CACHE_LOCK:
            cached = _GROUND_REQUIRED_PROCESS_CACHE.get(cache_key)
            if cached is not None:
                _GROUND_REQUIRED_PROCESS_CACHE.move_to_end(cache_key)
                _record_dense_linesearch_metrics(
                    groundRequiredPrecomputeHitCount=1,
                    groundRequiredProcessCacheHitCount=1,
                    groundRequiredMaterializationCacheHitCount=1,
                )
                return float(cached), None, False
            event = _GROUND_REQUIRED_PROCESS_CACHE_INFLIGHT.get(cache_key)
            if event is not None:
                _record_dense_linesearch_metrics(groundRequiredProcessCacheInflightWaiterCount=1)
                return None, event, False
            event = threading.Event()
            _GROUND_REQUIRED_PROCESS_CACHE_INFLIGHT[cache_key] = event
            _record_dense_linesearch_metrics(groundRequiredProcessCacheInflightLeaderCount=1)
            return None, event, True
    finally:
        _record_dense_linesearch_metrics(
            groundRequiredProcessCacheMs=(time.perf_counter() - started_at) * 1000.0,
        )


def _ground_required_process_cache_finish(
    signature: tuple,
    *,
    key_prefix: tuple | None = None,
) -> None:
    if not _ground_required_process_cache_enabled():
        return
    try:
        cache_key = _ground_required_process_cache_key(signature, key_prefix=key_prefix)
    except Exception:
        return
    event: threading.Event | None = None
    with _GROUND_REQUIRED_PROCESS_CACHE_LOCK:
        event = _GROUND_REQUIRED_PROCESS_CACHE_INFLIGHT.pop(cache_key, None)
    if event is not None:
        event.set()


def _ground_required_process_cache_wait(
    signature: tuple,
    event: threading.Event,
    *,
    key_prefix: tuple | None = None,
) -> float | None:
    started_at = time.perf_counter()
    ok = False
    try:
        ok = event.wait(_ground_required_process_cache_wait_timeout_s())
    finally:
        elapsed_ms = (time.perf_counter() - started_at) * 1000.0
        _record_dense_linesearch_metrics(
            groundRequiredProcessCacheInflightWaitMs=elapsed_ms,
            groundRequiredMaterializationWaitMs=elapsed_ms,
        )
    if not ok:
        _record_dense_linesearch_metrics(groundRequiredProcessCacheInflightTimeoutCount=1)
        return None
    cached = _ground_required_process_cache_get(signature, key_prefix=key_prefix)
    if cached is not None:
        _record_dense_linesearch_metrics(
            groundRequiredPrecomputeHitCount=1,
            groundRequiredProcessCacheHitCount=1,
            groundRequiredMaterializationCacheHitCount=1,
        )
    return cached


def _ground_required_process_cache_put(
    signature: tuple,
    value: float,
    *,
    key_prefix: tuple | None = None,
) -> None:
    if not _ground_required_process_cache_enabled():
        return
    started_at = time.perf_counter()
    try:
        max_size = _ground_required_process_cache_max()
        if max_size <= 0:
            return
        cache_key = _ground_required_process_cache_key(signature, key_prefix=key_prefix)
        with _GROUND_REQUIRED_PROCESS_CACHE_LOCK:
            _GROUND_REQUIRED_PROCESS_CACHE[cache_key] = float(value)
            _GROUND_REQUIRED_PROCESS_CACHE.move_to_end(cache_key)
            while len(_GROUND_REQUIRED_PROCESS_CACHE) > max_size:
                _GROUND_REQUIRED_PROCESS_CACHE.popitem(last=False)
        _record_dense_linesearch_metrics(groundRequiredProcessCacheStoreCount=1)
    finally:
        _record_dense_linesearch_metrics(
            groundRequiredProcessCacheMs=(time.perf_counter() - started_at) * 1000.0,
        )


def _ground_required_process_cache_put_many(
    values_by_signature: dict[tuple, float],
    *,
    key_prefix: tuple | None = None,
) -> None:
    if not values_by_signature or not _ground_required_process_cache_enabled():
        return
    started_at = time.perf_counter()
    try:
        max_size = _ground_required_process_cache_max()
        if max_size <= 0:
            return
        prefix = tuple(key_prefix) if key_prefix is not None else _ground_required_process_cache_prefix()
        with _GROUND_REQUIRED_PROCESS_CACHE_LOCK:
            for signature, value in values_by_signature.items():
                cache_key = prefix + (signature,)
                _GROUND_REQUIRED_PROCESS_CACHE[cache_key] = float(value)
                _GROUND_REQUIRED_PROCESS_CACHE.move_to_end(cache_key)
            while len(_GROUND_REQUIRED_PROCESS_CACHE) > max_size:
                _GROUND_REQUIRED_PROCESS_CACHE.popitem(last=False)
        _record_dense_linesearch_metrics(groundRequiredProcessCacheStoreCount=len(values_by_signature))
    finally:
        _record_dense_linesearch_metrics(
            groundRequiredProcessCacheMs=(time.perf_counter() - started_at) * 1000.0,
        )


def _trusted_line_coord_dem_values(
    line_coords: list[dict],
    *,
    key_prefix: tuple | None = None,
) -> dict[tuple[float, float], float]:
    candidates: dict[tuple[float, float], int] = {}
    for coord in line_coords or []:
        if not isinstance(coord, dict):
            continue
        lat = _safe_float_or_none(coord.get("latitude"))
        lon = _safe_float_or_none(coord.get("longitude"))
        alt = _safe_float_or_none(coord.get("altitude"))
        if lat is None or lon is None or alt is None:
            continue
        key = _dem_alt_cache_key(float(lat), float(lon), prefix=key_prefix)
        candidates[key] = int(round(float(alt)))
    if not candidates:
        return {}

    trusted: dict[tuple[float, float], float] = {}
    lock_wait_started_at = time.perf_counter()
    _DEM_ALT_CACHE_LOCK.acquire()
    lock_wait_ms = (time.perf_counter() - lock_wait_started_at) * 1000.0
    cache_read_started_at = time.perf_counter()
    try:
        for key, coord_alt in candidates.items():
            cached = _DEM_ALT_CACHE.get(key)
            if cached is None:
                continue
            if int(cached) == int(coord_alt):
                trusted[key] = float(cached)
    finally:
        _DEM_ALT_CACHE_LOCK.release()
        _record_dense_linesearch_metrics(
            groundRequiredCacheLockWaitMs=lock_wait_ms,
            groundRequiredCacheReadMs=(time.perf_counter() - cache_read_started_at) * 1000.0,
        )
    return trusted


def _cached_line_coord_dem_values(
    line_coords: list[dict],
    *,
    key_prefix: tuple | None = None,
    lock_blocking: bool = True,
) -> dict[tuple[float, float], float] | None:
    keys: list[tuple[float, float]] = []
    for coord in line_coords or []:
        if not isinstance(coord, dict):
            continue
        lat = _safe_float_or_none(coord.get("latitude"))
        lon = _safe_float_or_none(coord.get("longitude"))
        if lat is None or lon is None:
            continue
        keys.append(_dem_alt_cache_key(float(lat), float(lon), prefix=key_prefix))
    if not keys:
        return {}

    cached_values: dict[tuple[float, float], float] = {}
    lock_wait_started_at = time.perf_counter()
    acquired = _DEM_ALT_CACHE_LOCK.acquire(blocking=bool(lock_blocking))
    lock_wait_ms = (time.perf_counter() - lock_wait_started_at) * 1000.0
    if not acquired:
        _record_dense_linesearch_metrics(groundRequiredCacheLockWaitMs=lock_wait_ms)
        return None
    cache_read_started_at = time.perf_counter()
    try:
        for key in keys:
            cached = _DEM_ALT_CACHE.get(key)
            if cached is not None:
                cached_values[key] = float(cached)
    finally:
        _DEM_ALT_CACHE_LOCK.release()
        _record_dense_linesearch_metrics(
            groundRequiredCacheLockWaitMs=lock_wait_ms,
            groundRequiredCacheReadMs=(time.perf_counter() - cache_read_started_at) * 1000.0,
        )
    return cached_values


def _stamp_line_coord_dem_altitudes(
    line_coord_groups: list[list[dict]],
    *,
    key_prefix: tuple | None = None,
) -> None:
    if not line_coord_groups:
        return
    started_at = time.perf_counter()
    refs_by_key: "OrderedDict[tuple, list[dict]]" = OrderedDict()
    point_by_key: "OrderedDict[tuple, tuple[float, float]]" = OrderedDict()
    coord_count = 0
    for line_coords in line_coord_groups:
        for coord in line_coords or []:
            if not isinstance(coord, dict):
                continue
            lat = _safe_float_or_none(coord.get("latitude"))
            lon = _safe_float_or_none(coord.get("longitude"))
            if lat is None or lon is None:
                continue
            key = _dem_alt_cache_key(float(lat), float(lon), prefix=key_prefix)
            refs_by_key.setdefault(key, []).append(coord)
            point_by_key.setdefault(key, (float(lat), float(lon)))
            coord_count += 1
    if not point_by_key:
        return
    changed_count = 0
    try:
        missing_points_by_key: "OrderedDict[tuple, tuple[float, float]]" = OrderedDict()
        cached_values_by_key: dict[tuple, int] = {}
        cache_read_started = time.perf_counter()
        with _DEM_ALT_CACHE_LOCK:
            for key, point in point_by_key.items():
                cached = _DEM_ALT_CACHE.get(key)
                if cached is None:
                    missing_points_by_key[key] = point
                else:
                    cached_values_by_key[key] = int(cached)
        _record_dense_linesearch_metrics(
            demAltCacheReadMs=(time.perf_counter() - cache_read_started) * 1000.0,
        )

        values_by_key: dict[tuple, int] = dict(cached_values_by_key)
        if missing_points_by_key:
            points = list(missing_points_by_key.values())
            values = _dem_alt_many(points, prefix=key_prefix)
            if len(values) != len(points):
                raise RuntimeError("DEM line coordinate stamp result length mismatch")
            values_by_key.update(
                {
                    key: int(round(float(value)))
                    for key, value in zip(missing_points_by_key.keys(), values)
                }
            )

        for key, dem_alt in values_by_key.items():
            for coord in refs_by_key.get(key, []):
                previous = _safe_float_or_none(coord.get("altitude"))
                if previous is None or int(round(float(previous))) != dem_alt:
                    changed_count += 1
                coord["altitude"] = int(dem_alt)
    except Exception:
        return
    finally:
        _record_dense_linesearch_metrics(
            groundRequiredLineCoordStampMs=(time.perf_counter() - started_at) * 1000.0,
            groundRequiredLineCoordStampCount=coord_count,
            groundRequiredLineCoordStampChangedCount=changed_count,
        )


def _seed_dem_alt_cache_from_line_coord_altitudes(
    wps: list[dict],
    *,
    key_prefix: tuple | None = None,
) -> None:
    if not wps or not _runtime_bool_setting("ground_required_line_coord_cache_seed_enabled", False):
        return
    started_at = time.perf_counter()
    seeded_count = 0
    skipped_count = 0
    values_by_key: "OrderedDict[tuple, int]" = OrderedDict()
    conflicted_keys: set[tuple] = set()
    cache_prefix = tuple(key_prefix) if key_prefix is not None else _dem_alt_cache_prefix()

    try:
        for wp in wps or []:
            if not isinstance(wp, dict):
                continue
            fp = wp.get("filmingProperty") if isinstance(wp.get("filmingProperty"), dict) else {}
            ls = fp.get("lineSearch") if isinstance(fp.get("lineSearch"), dict) else {}
            line_coords = ls.get("coordinateList") or []
            if not line_coords:
                continue
            if not _line_coords_dense_fast_path_geometry_ok(
                line_coords,
                sample_step_m=_ground_required_sample_step_m(ls),
            ):
                skipped_count += len(line_coords)
                continue
            for coord in line_coords:
                if not isinstance(coord, dict):
                    skipped_count += 1
                    continue
                lat = _safe_float_or_none(coord.get("latitude"))
                lon = _safe_float_or_none(coord.get("longitude"))
                alt = _safe_float_or_none(coord.get("altitude"))
                if lat is None or lon is None or alt is None:
                    skipped_count += 1
                    continue
                key = _dem_alt_cache_key(float(lat), float(lon), prefix=cache_prefix)
                value = int(round(float(alt)))
                previous = values_by_key.get(key)
                if previous is not None and int(previous) != value:
                    conflicted_keys.add(key)
                    skipped_count += 1
                    continue
                values_by_key[key] = value

        for key in conflicted_keys:
            values_by_key.pop(key, None)
        if not values_by_key:
            return

        cache_write_started = time.perf_counter()
        clear_count = 0
        with _DEM_ALT_CACHE_LOCK:
            if len(_DEM_ALT_CACHE) + len(values_by_key) >= _DEM_ALT_CACHE_MAX:
                _DEM_ALT_CACHE.clear()
                clear_count = 1
            for key, value in values_by_key.items():
                cached = _DEM_ALT_CACHE.get(key)
                if cached is None:
                    _DEM_ALT_CACHE[key] = int(value)
                    seeded_count += 1
                    continue
                if int(cached) == int(value):
                    seeded_count += 1
                    continue
                skipped_count += 1
        _record_dense_linesearch_metrics(
            demAltCacheWriteMs=(time.perf_counter() - cache_write_started) * 1000.0,
            demAltCacheClearCount=clear_count,
        )
    finally:
        _record_dense_linesearch_metrics(
            groundRequiredLineCoordCacheSeedCount=seeded_count,
            groundRequiredLineCoordCacheSeedSkippedCount=skipped_count,
            groundRequiredLineCoordCacheSeedMs=(time.perf_counter() - started_at) * 1000.0,
        )


def _compute_dense_line_coord_ground_required_map(
    pending_entries: list[tuple[tuple, list[dict], dict, dict]],
    *,
    key_prefix: tuple | None = None,
) -> dict[tuple, float]:
    if not pending_entries or not _runtime_bool_setting("ground_required_line_coord_cache_warm_enabled", True):
        return {}
    started_at = time.perf_counter()
    warmed_count = 0
    skipped_count = 0
    cache_prefix = tuple(key_prefix) if key_prefix is not None else _dem_alt_cache_prefix()
    points_by_key: "OrderedDict[tuple, tuple[float, float]]" = OrderedDict()
    keys_by_signature: dict[tuple, tuple[list[tuple], int]] = {}

    try:
        for signature, line_coords, ls, _coord in pending_entries:
            key_result = _line_coords_dense_fast_path_cache_keys(
                line_coords,
                sample_step_m=_ground_required_sample_step_m(ls),
                key_prefix=cache_prefix,
            )
            if key_result is None:
                continue
            keys, points, unique_key_count = key_result
            for key, point in zip(keys, points):
                if key not in points_by_key:
                    points_by_key[key] = point
            keys_by_signature[signature] = (keys, unique_key_count)

        if not keys_by_signature:
            return {}

        values_by_key: dict[tuple, float] = {}
        missing_points_by_key: "OrderedDict[tuple, tuple[float, float]]" = OrderedDict()
        cache_lock_wait_started = time.perf_counter()
        _DEM_ALT_CACHE_LOCK.acquire()
        cache_lock_wait_ms = (time.perf_counter() - cache_lock_wait_started) * 1000.0
        cache_read_started = time.perf_counter()
        try:
            for key, point in points_by_key.items():
                cached = _DEM_ALT_CACHE.get(key)
                if cached is None:
                    missing_points_by_key[key] = point
                else:
                    values_by_key[key] = float(cached)
        finally:
            _DEM_ALT_CACHE_LOCK.release()
            cache_read_ms = (time.perf_counter() - cache_read_started) * 1000.0
        _record_dense_linesearch_metrics(
            demAltCacheReadMs=cache_read_ms,
            groundRequiredCacheLockWaitMs=cache_lock_wait_ms,
            groundRequiredCacheReadMs=cache_read_ms,
        )

        if missing_points_by_key:
            values = _dem_alt_many(list(missing_points_by_key.values()), prefix=cache_prefix)
            for key, value in zip(missing_points_by_key.keys(), values):
                values_by_key[key] = float(value)
            warmed_count = len(missing_points_by_key)

        dense_ground_by_signature: dict[tuple, float] = {}
        dense_path_count = 0
        dense_coord_count = 0
        reused_key_count = 0
        for signature, (keys, unique_key_count) in keys_by_signature.items():
            if any(key not in values_by_key for key in keys):
                continue
            dense_ground_by_signature[signature] = max(float(values_by_key[key]) for key in keys)
            dense_path_count += 1
            dense_coord_count += len(keys)
            reused_key_count += int(unique_key_count)

        if dense_ground_by_signature:
            _record_dense_linesearch_metrics(
                groundRequiredDenseFastPathCount=dense_path_count,
                groundRequiredDenseFastPathCoordCount=dense_coord_count,
                groundRequiredLineCoordReuseCount=reused_key_count,
            )
        return dense_ground_by_signature
    finally:
        _record_dense_linesearch_metrics(
            groundRequiredLineCoordCacheWarmCount=warmed_count,
            groundRequiredLineCoordCacheWarmSkippedCount=skipped_count,
            groundRequiredLineCoordCacheWarmMs=(time.perf_counter() - started_at) * 1000.0,
        )


def _preowner_ground_required_process_cache_for_waypoints(
    wps: list[dict],
    *,
    signature_cache: dict[tuple, float] | None = None,
    fallback_ground_ref_m: float | None = None,
) -> dict[tuple, float]:
    """Publish final-geometry groundRequired values before waypoint materialization."""
    if not wps or not _altitude_precompute_enabled() or not _ground_required_process_cache_enabled():
        return {}

    started_at = time.perf_counter()
    cache = signature_cache if isinstance(signature_cache, dict) else {}
    key_prefix = _dem_alt_cache_prefix()
    seen_signatures: set[tuple] = set()
    pending_entries: list[tuple[tuple, list[dict], dict, dict]] = []
    published: dict[tuple, float] = {}
    hit_count = 0
    skipped_count = 0

    def _fallback_ground_required_for_entry(
        line_coords: list[dict],
        ls: dict,
        coord: dict,
    ) -> float | None:
        sample_points = _sample_points_along_coords(
            line_coords,
            sample_step_m=_ground_required_sample_step_m(ls),
        )
        if not sample_points and isinstance(coord, dict):
            lat = _safe_float_or_none(coord.get("latitude"))
            lon = _safe_float_or_none(coord.get("longitude"))
            if lat is not None and lon is not None:
                sample_points = [(float(lat), float(lon))]
        if sample_points:
            values = _dem_alt_many(sample_points, prefix=key_prefix)
            if values:
                return float(max(float(value) for value in values))
        if fallback_ground_ref_m is not None:
            return float(fallback_ground_ref_m)
        return None

    try:
        for wp in wps or []:
            if not isinstance(wp, dict):
                skipped_count += 1
                continue
            fp = wp.get("filmingProperty") if isinstance(wp.get("filmingProperty"), dict) else {}
            ls = fp.get("lineSearch") if isinstance(fp.get("lineSearch"), dict) else {}
            line_coords = ls.get("coordinateList") or []
            if not line_coords:
                continue
            cached_ground = _safe_float_or_none(wp.get(_GROUND_REQUIRED_CACHE_KEY))
            if (
                cached_ground is not None
                and wp.get(_GROUND_REQUIRED_COORDS_ID_KEY) == id(line_coords)
                and wp.get(_GROUND_REQUIRED_COORDS_COUNT_KEY) == len(line_coords)
            ):
                hit_count += 1
                _record_dense_linesearch_metrics(groundRequiredPrecomputeHitCount=1)
                continue
            signature = _line_coords_ground_signature_for_search(line_coords, ls)
            if signature is None:
                skipped_count += 1
                continue
            if cached_ground is not None and wp.get(_GROUND_REQUIRED_SIGNATURE_KEY) == signature:
                cache[signature] = float(cached_ground)
                hit_count += 1
                _record_dense_linesearch_metrics(groundRequiredPrecomputeHitCount=1)
                continue
            if signature in seen_signatures:
                continue
            seen_signatures.add(signature)
            cached_ground = cache.get(signature)
            if cached_ground is None:
                cached_ground = _ground_required_process_cache_get(signature)
            if cached_ground is not None:
                cache[signature] = float(cached_ground)
                hit_count += 1
                _record_dense_linesearch_metrics(
                    groundRequiredPrecomputeHitCount=1,
                    groundRequiredProcessCacheHitCount=1,
                )
                continue
            coord = wp.get("coordinate") if isinstance(wp.get("coordinate"), dict) else {}
            pending_entries.append((signature, line_coords, ls, coord if isinstance(coord, dict) else {}))

        dense_ground_by_signature, remaining_pending_entries = (
            _compute_coord_altitude_ground_required_map(
                pending_entries,
                allow_untrusted=False,
            )
        )
        if remaining_pending_entries:
            dense_ground_by_signature.update(
                _compute_dense_line_coord_ground_required_map(
                    remaining_pending_entries,
                    key_prefix=key_prefix,
                )
            )
        for signature, value in dense_ground_by_signature.items():
            published[signature] = float(value)

        for signature, line_coords, ls, coord in remaining_pending_entries:
            if signature in published:
                continue
            fallback_ground = _fallback_ground_required_for_entry(line_coords, ls, coord)
            if fallback_ground is not None:
                published[signature] = float(fallback_ground)
            else:
                skipped_count += 1

        if published:
            _ground_required_process_cache_put_many(published)
            cache.update(published)
        return published
    finally:
        _record_dense_linesearch_metrics(
            groundRequiredPreOwnerMs=(time.perf_counter() - started_at) * 1000.0,
            groundRequiredPreOwnerSignatureCount=len(seen_signatures),
            groundRequiredPreOwnerPublishedCount=len(published),
            groundRequiredPreOwnerHitCount=hit_count,
            groundRequiredPreOwnerSkippedCount=skipped_count,
        )


def _prewarm_ground_required_process_cache_for_packets(
    packets: list[dict],
    *,
    signature_cache: dict[tuple, float] | None = None,
    runtime_payload: dict | None = None,
) -> None:
    if (
        not packets
        or not _altitude_precompute_enabled()
        or not _runtime_bool_setting("ground_required_packet_prewarm_enabled", True)
    ):
        return

    cache = signature_cache if isinstance(signature_cache, dict) else {}
    key_prefix = _dem_alt_cache_prefix()
    async_enabled = _runtime_bool_setting("ground_required_packet_prewarm_async_enabled", True)
    seen_signatures: set[tuple] = set()
    pending_entries: list[tuple[tuple, list[dict], dict, dict]] = []
    wait_entries: list[tuple[tuple, threading.Event]] = []
    owned_signatures: set[tuple] = set()

    def _snapshot_line_coords(line_coords: list[dict]) -> list[dict]:
        return [dict(coord) for coord in line_coords or [] if isinstance(coord, dict)]

    def _fallback_ground_required_for_entry(
        line_coords: list[dict],
        ls: dict,
        coord: dict,
    ) -> float | None:
        sample_points = _sample_points_along_coords(
            line_coords,
            sample_step_m=_ground_required_sample_step_m(ls),
        )
        if not sample_points and isinstance(coord, dict):
            lat = _safe_float_or_none(coord.get("latitude"))
            lon = _safe_float_or_none(coord.get("longitude"))
            if lat is not None and lon is not None:
                sample_points = [(float(lat), float(lon))]
        if not sample_points:
            return None
        values = _dem_alt_many(sample_points, prefix=key_prefix)
        if not values:
            return None
        return float(max(float(value) for value in values))

    def _compute_owned_ground_required() -> dict[tuple, float]:
        values_by_signature: dict[tuple, float] = {}
        if not pending_entries:
            return values_by_signature
        with _runtime_override_context(runtime_payload):
            dense_ground_by_signature = _compute_dense_line_coord_ground_required_map(
                pending_entries,
                key_prefix=key_prefix,
            )
            for signature, value in dense_ground_by_signature.items():
                if signature in owned_signatures:
                    values_by_signature[signature] = float(value)
            for signature, line_coords, ls, coord in pending_entries:
                if signature not in owned_signatures or signature in values_by_signature:
                    continue
                fallback_ground = _fallback_ground_required_for_entry(line_coords, ls, coord)
                if fallback_ground is not None:
                    values_by_signature[signature] = float(fallback_ground)
            if values_by_signature:
                _ground_required_process_cache_put_many(values_by_signature)
            return values_by_signature

    def _run_async_owned_ground_required() -> None:
        try:
            _compute_owned_ground_required()
        finally:
            for signature in owned_signatures:
                _ground_required_process_cache_finish(signature)

    for pkt in packets or []:
        if not isinstance(pkt, dict):
            continue
        for wp in pkt.get("wplist") or []:
            if not isinstance(wp, dict):
                continue
            fp = wp.get("filmingProperty") if isinstance(wp.get("filmingProperty"), dict) else {}
            ls = fp.get("lineSearch") if isinstance(fp.get("lineSearch"), dict) else {}
            line_coords = ls.get("coordinateList") or []
            if not line_coords:
                continue
            signature = _line_coords_ground_signature_for_search(line_coords, ls)
            if signature is None or signature in seen_signatures or signature in cache:
                continue
            seen_signatures.add(signature)
            sample_step_m = _ground_required_sample_step_m(ls)
            if not _line_coords_dense_fast_path_geometry_ok(
                line_coords,
                sample_step_m=sample_step_m,
            ):
                continue
            process_cached_ground, process_event, process_owner = (
                _ground_required_process_cache_get_or_claim(signature)
            )
            if process_cached_ground is not None:
                cache[signature] = float(process_cached_ground)
                continue
            if process_event is not None and not process_owner:
                if not async_enabled:
                    wait_entries.append((signature, process_event))
                continue
            if process_owner and process_event is not None:
                owned_signatures.add(signature)
            coord = wp.get("coordinate") if isinstance(wp.get("coordinate"), dict) else {}
            pending_entries.append((
                signature,
                _snapshot_line_coords(line_coords),
                dict(ls),
                dict(coord) if isinstance(coord, dict) else {},
            ))

    if pending_entries and owned_signatures and async_enabled:
        thread = threading.Thread(
            target=_run_async_owned_ground_required,
            name="GroundRequiredPrewarm",
            daemon=True,
        )
        thread.start()
        return

    try:
        values_by_signature = _compute_owned_ground_required()
        if values_by_signature:
            cache.update(values_by_signature)
    finally:
        for signature in owned_signatures:
            _ground_required_process_cache_finish(signature)

    for signature, event in wait_entries:
        if signature in cache:
            continue
        ground_required = _ground_required_process_cache_wait(signature, event)
        if ground_required is not None:
            cache[signature] = float(ground_required)


def _record_ground_required_fast_path_reject(reason: str) -> None:
    _record_dense_linesearch_metrics(
        groundRequiredFastPathRejectCount=1,
        groundRequiredFastPathRejectReason=str(reason or "unknown"),
    )


def _dense_line_coords_ground_required_fast_path(
    line_coords: list[dict],
    *,
    sample_step_m: float,
    key_prefix: tuple | None = None,
    cache_lock_blocking: bool = True,
) -> float | None:
    if not _runtime_bool_setting("ground_required_line_coord_fast_path_enabled", True):
        _record_ground_required_fast_path_reject("disabled")
        return None
    min_count = max(2, _runtime_int_setting("ground_required_line_coord_fast_path_min_count", 8))
    usable: list[tuple[float, float]] = []
    unique_keys: set[tuple] = set()
    for coord in line_coords or []:
        if not isinstance(coord, dict):
            _record_ground_required_fast_path_reject("invalidCoordinate")
            return None
        lat = _safe_float_or_none(coord.get("latitude"))
        lon = _safe_float_or_none(coord.get("longitude"))
        if lat is None or lon is None:
            _record_ground_required_fast_path_reject("missingCoordinate")
            return None
        lat_f = float(lat)
        lon_f = float(lon)
        usable.append((lat_f, lon_f))
        unique_keys.add(_dem_alt_cache_key(lat_f, lon_f, prefix=key_prefix))
    if len(usable) < min_count:
        _record_ground_required_fast_path_reject("tooFewCoords")
        return None
    cache_prefix = tuple(key_prefix) if key_prefix is not None else _dem_alt_cache_prefix()
    cached = _cached_line_coord_dem_values(
        line_coords,
        key_prefix=cache_prefix,
        lock_blocking=cache_lock_blocking,
    )
    if cached is None:
        _record_ground_required_fast_path_reject("cacheBusy")
        return None
    if len(cached) < len(unique_keys):
        _record_ground_required_fast_path_reject("cacheMiss")
        return None

    max_step = max(1.0, float(sample_step_m)) * max(
        1.0,
        _runtime_float_setting("ground_required_line_coord_fast_path_step_tolerance", 1.05),
    )
    for prev, curr in zip(usable[:-1], usable[1:]):
        prev_lat, prev_lon = prev
        curr_lat, curr_lon = curr
        mean_lat_rad = math.radians((prev_lat + curr_lat) * 0.5)
        dy = (curr_lat - prev_lat) * 111_320.0
        dx = (curr_lon - prev_lon) * 111_320.0 * math.cos(mean_lat_rad)
        if math.hypot(dx, dy) > max_step:
            _record_ground_required_fast_path_reject("gapTooLarge")
            return None

    _record_dense_linesearch_metrics(
        groundRequiredDenseFastPathCount=1,
        groundRequiredDenseFastPathCoordCount=len(usable),
        groundRequiredLineCoordReuseCount=len(cached),
    )
    return max(cached.values()) if cached else None


def _line_coords_dense_fast_path_geometry_ok(
    line_coords: list[dict],
    *,
    sample_step_m: float,
) -> bool:
    min_count = max(2, _runtime_int_setting("ground_required_line_coord_fast_path_min_count", 8))
    usable: list[tuple[float, float]] = []
    for coord in line_coords or []:
        if not isinstance(coord, dict):
            return False
        lat = _safe_float_or_none(coord.get("latitude"))
        lon = _safe_float_or_none(coord.get("longitude"))
        if lat is None or lon is None:
            return False
        usable.append((float(lat), float(lon)))
    if len(usable) < min_count:
        return False

    max_step = max(1.0, float(sample_step_m)) * max(
        1.0,
        _runtime_float_setting("ground_required_line_coord_fast_path_step_tolerance", 1.05),
    )
    for prev, curr in zip(usable[:-1], usable[1:]):
        prev_lat, prev_lon = prev
        curr_lat, curr_lon = curr
        mean_lat_rad = math.radians((prev_lat + curr_lat) * 0.5)
        dy = (curr_lat - prev_lat) * 111_320.0
        dx = (curr_lon - prev_lon) * 111_320.0 * math.cos(mean_lat_rad)
        if math.hypot(dx, dy) > max_step:
            return False
    return True


def _line_coords_dense_fast_path_cache_keys(
    line_coords: list[dict],
    *,
    sample_step_m: float,
    key_prefix: tuple,
) -> tuple[list[tuple], list[tuple[float, float]], int] | None:
    min_count = max(2, _runtime_int_setting("ground_required_line_coord_fast_path_min_count", 8))
    keys: list[tuple] = []
    points: list[tuple[float, float]] = []
    seen_keys: set[tuple] = set()
    unique_key_count = 0

    for coord in line_coords or []:
        if not isinstance(coord, dict):
            return None
        lat = _safe_float_or_none(coord.get("latitude"))
        lon = _safe_float_or_none(coord.get("longitude"))
        if lat is None or lon is None:
            return None
        lat_f = float(lat)
        lon_f = float(lon)
        key = _dem_alt_cache_key_from_prefix(key_prefix, lat_f, lon_f)
        keys.append(key)
        points.append((lat_f, lon_f))
        if key not in seen_keys:
            seen_keys.add(key)
            unique_key_count += 1

    if len(points) < min_count:
        return None

    max_step = max(1.0, float(sample_step_m)) * max(
        1.0,
        _runtime_float_setting("ground_required_line_coord_fast_path_step_tolerance", 1.05),
    )
    for prev, curr in zip(points[:-1], points[1:]):
        prev_lat, prev_lon = prev
        curr_lat, curr_lon = curr
        mean_lat_rad = math.radians((prev_lat + curr_lat) * 0.5)
        dy = (curr_lat - prev_lat) * 111_320.0
        dx = (curr_lon - prev_lon) * 111_320.0 * math.cos(mean_lat_rad)
        if math.hypot(dx, dy) > max_step:
            return None
    return keys, points, unique_key_count


def _record_ground_required_coord_altitude_fast_path_reject(reason: str) -> None:
    _record_dense_linesearch_metrics(
        groundRequiredCoordAltitudeFastPathRejectCount=1,
        groundRequiredCoordAltitudeFastPathRejectReason=str(reason or "unknown"),
    )


def _coord_has_trusted_dem_altitude(coord: dict) -> bool:
    return isinstance(coord, dict) and coord.get(_DEM_ALT_COORD_TRUSTED_KEY) is True


def _dense_line_coords_ground_required_from_coord_altitudes(
    line_coords: list[dict],
    *,
    line_search: dict,
    sample_step_m: float,
    require_trusted_dem_altitude: bool = False,
) -> float | None:
    setting_name = (
        "ground_required_trusted_coord_altitude_fast_path_enabled"
        if require_trusted_dem_altitude
        else "ground_required_coord_altitude_fast_path_enabled"
    )
    if not _runtime_bool_setting(setting_name, True):
        _record_ground_required_coord_altitude_fast_path_reject("disabled")
        return None

    interp_points = _interp_points_hint(line_search.get("interpolationPoints"))
    if not require_trusted_dem_altitude and (interp_points is None or int(interp_points) > 2):
        _record_ground_required_coord_altitude_fast_path_reject("interpolatedAltitude")
        return None

    if not _line_coords_dense_fast_path_geometry_ok(line_coords, sample_step_m=sample_step_m):
        _record_ground_required_coord_altitude_fast_path_reject("geometry")
        return None

    altitudes: list[float] = []
    for coord in line_coords or []:
        if not isinstance(coord, dict):
            _record_ground_required_coord_altitude_fast_path_reject("invalidCoordinate")
            return None
        if require_trusted_dem_altitude and not _coord_has_trusted_dem_altitude(coord):
            _record_ground_required_coord_altitude_fast_path_reject("untrustedAltitude")
            return None
        alt = _safe_float_or_none(coord.get("altitude"))
        if alt is None:
            _record_ground_required_coord_altitude_fast_path_reject("missingAltitude")
            return None
        altitudes.append(float(alt))
    if not altitudes:
        _record_ground_required_coord_altitude_fast_path_reject("empty")
        return None

    _record_dense_linesearch_metrics(
        groundRequiredCoordAltitudeFastPathCount=1,
        groundRequiredTrustedCoordAltitudeFastPathCount=1 if require_trusted_dem_altitude else 0,
        groundRequiredCoordAltitudeFastPathCoordCount=len(altitudes),
        groundRequiredDenseFastPathCount=1,
        groundRequiredDenseFastPathCoordCount=len(altitudes),
        groundRequiredLineCoordReuseCount=len(altitudes),
    )
    return max(altitudes)


def _linesearch_inner_parallel_workers(strip_count: int, *, coordinate_count: int = 0) -> int:
    if _linesearch_inner_parallel_suppressed():
        _record_dense_linesearch_metrics(innerParallelSuppressedCount=1)
        return 1
    min_strips = max(1, _runtime_int_setting("linesearch_inner_parallel_min_strips", LINESEARCH_INNER_PARALLEL_MIN_STRIPS))
    min_coords = max(1, _runtime_int_setting("linesearch_inner_parallel_min_coords", LINESEARCH_INNER_PARALLEL_MIN_COORDS))
    strip_trigger = int(strip_count or 0) >= int(min_strips)
    coord_trigger = int(coordinate_count or 0) >= int(min_coords)
    if not strip_trigger and not coord_trigger:
        return 1
    requested = _runtime_int_setting("linesearch_inner_parallel_workers", LINESEARCH_INNER_PARALLEL_WORKERS)
    if requested <= 1:
        return 1
    cpu_count = os.cpu_count() or requested
    return max(1, min(int(requested), int(cpu_count), int(strip_count or 1)))


def _ground_required_sample_step_m(line_search: dict | None = None) -> float:
    base_step_m = max(1.0, _runtime_float_setting("ground_required_sample_step_m", 120.0))
    if not isinstance(line_search, dict):
        return float(base_step_m)
    interp_points = _interp_points_hint(line_search.get("interpolationPoints"))
    dense_threshold = max(
        1,
        _runtime_int_setting("dense_linesearch_ground_sample_min_interp_points", 3),
    )
    if int(interp_points or 0) < dense_threshold:
        return float(base_step_m)
    return max(
        1.0,
        _runtime_float_setting("dense_linesearch_ground_sample_step_m", 240.0),
    )


def _chunk_sequence(values: list[float], chunks: int) -> list[list[float]]:
    if chunks <= 1 or len(values) <= 1:
        return [list(values)]
    chunk_size = max(1, int(math.ceil(len(values) / float(chunks))))
    return [values[idx:idx + chunk_size] for idx in range(0, len(values), chunk_size)]


def _dem_alt_cache_prefix() -> tuple:
    try:
        ensure_current = getattr(_mission_helpers, "ensure_terrain_cache_current", None)
        if callable(ensure_current):
            ensure_current()
        terrain_signature = _mission_helpers.terrain_data_signature()
    except Exception:
        terrain_signature = tuple()
    runtime_version = os.environ.get("MISSION_PLAN_DEM_ALT_CACHE_VERSION", "default")
    return (
        "demAlt:v2",
        terrain_signature,
        str(runtime_version),
    )


def _dem_alt_cache_key_from_prefix(key_prefix: tuple, lat: float, lon: float) -> tuple:
    return key_prefix + (
        round(float(lat), 7),
        round(float(lon), 7),
    )


def _dem_alt_cache_key(lat: float, lon: float, *, prefix: tuple | None = None) -> tuple:
    key_prefix = tuple(prefix) if prefix is not None else _dem_alt_cache_prefix()
    return _dem_alt_cache_key_from_prefix(key_prefix, lat, lon)


def _active_dem_alt_run_map() -> dict[tuple, int] | None:
    values_by_key = getattr(_DEM_ALT_RUN_MAP_LOCAL, "values_by_key", None)
    return values_by_key if isinstance(values_by_key, dict) else None


def _dem_alt_run_map_begin() -> None:
    stack = getattr(_DEM_ALT_RUN_MAP_LOCAL, "stack", None)
    if not isinstance(stack, list):
        stack = []
        _DEM_ALT_RUN_MAP_LOCAL.stack = stack
    previous = getattr(_DEM_ALT_RUN_MAP_LOCAL, "values_by_key", None)
    stack.append(previous if isinstance(previous, dict) else None)
    _DEM_ALT_RUN_MAP_LOCAL.values_by_key = {}


def _dem_alt_run_map_end() -> None:
    current = _active_dem_alt_run_map()
    if current is not None:
        _record_dense_linesearch_metrics(demRunMapSize=len(current))
    stack = getattr(_DEM_ALT_RUN_MAP_LOCAL, "stack", None)
    previous = stack.pop() if isinstance(stack, list) and stack else None
    if isinstance(previous, dict):
        _DEM_ALT_RUN_MAP_LOCAL.values_by_key = previous
    else:
        try:
            delattr(_DEM_ALT_RUN_MAP_LOCAL, "values_by_key")
        except Exception:
            pass


def _dem_alt_run_map_store(values_by_key: dict[tuple, int] | None) -> None:
    if not values_by_key:
        return
    run_map = _active_dem_alt_run_map()
    if run_map is None:
        return
    store_count = 0
    for key, value in values_by_key.items():
        try:
            int_value = int(round(float(value)))
        except Exception:
            continue
        previous = run_map.get(key)
        run_map[key] = int_value
        if previous is None or int(previous) != int_value:
            store_count += 1
    if store_count:
        _record_dense_linesearch_metrics(demRunMapStoreCount=store_count)


def _dem_alt(lat: float, lon: float) -> int:
    """DEM 기반 고도를 정수(m)로 반환."""
    key = _dem_alt_cache_key(lat, lon)
    run_map = _active_dem_alt_run_map()
    if run_map is not None:
        cached = run_map.get(key)
        if cached is not None:
            _record_dense_linesearch_metrics(
                demLookupCount=1,
                demCacheHitCount=1,
                demRunMapHitCount=1,
            )
            return int(cached)

    cache_read_started = time.perf_counter()
    with _DEM_ALT_CACHE_LOCK:
        cached = _DEM_ALT_CACHE.get(key)
    _record_dense_linesearch_metrics(
        demAltCacheReadMs=(time.perf_counter() - cache_read_started) * 1000.0,
    )
    if cached is not None:
        _dem_alt_run_map_store({key: int(cached)})
        _record_dense_linesearch_metrics(demLookupCount=1, demCacheHitCount=1)
        return int(cached)

    value = int(round(terrain_elev(float(lat), float(lon))))
    cache_write_started = time.perf_counter()
    clear_count = 0
    with _DEM_ALT_CACHE_LOCK:
        if len(_DEM_ALT_CACHE) >= _DEM_ALT_CACHE_MAX:
            _DEM_ALT_CACHE.clear()
            clear_count = 1
        _DEM_ALT_CACHE[key] = value
    _dem_alt_run_map_store({key: int(value)})
    _record_dense_linesearch_metrics(
        demLookupCount=1,
        demCacheMissCount=1,
        demAltCacheWriteMs=(time.perf_counter() - cache_write_started) * 1000.0,
        demAltCacheClearCount=clear_count,
    )
    return value


def _dem_alt_many(
    points: list[tuple[float, float]] | tuple[tuple[float, float], ...],
    *,
    prefix: tuple | None = None,
) -> list[int]:
    if not points:
        return []

    normalized: list[tuple[float, float]] = [(float(lat), float(lon)) for lat, lon in points]
    key_prefix = tuple(prefix) if prefix is not None else _dem_alt_cache_prefix()
    keys = [_dem_alt_cache_key(lat, lon, prefix=key_prefix) for lat, lon in normalized]
    unique_keys = list(dict.fromkeys(keys))
    point_by_key: dict[tuple, tuple[float, float]] = dict(zip(keys, normalized))
    values_by_key: dict[tuple, int] = {}
    missing: list[tuple] = []
    computed_missing_count = 0

    run_map = _active_dem_alt_run_map()
    cache_candidate_keys = unique_keys
    if run_map is not None:
        cache_candidate_keys = []
        run_map_hit_count = 0
        for key in unique_keys:
            cached = run_map.get(key)
            if cached is None:
                cache_candidate_keys.append(key)
            else:
                values_by_key[key] = int(cached)
                run_map_hit_count += 1
        if run_map_hit_count:
            _record_dense_linesearch_metrics(demRunMapHitCount=run_map_hit_count)

    cache_hits_for_run_map: dict[tuple, int] = {}
    cache_read_started = time.perf_counter()
    with _DEM_ALT_CACHE_LOCK:
        for key in cache_candidate_keys:
            cached = _DEM_ALT_CACHE.get(key)
            if cached is None:
                missing.append(key)
            else:
                values_by_key[key] = int(cached)
                cache_hits_for_run_map[key] = int(cached)
    _record_dense_linesearch_metrics(
        demAltCacheReadMs=(time.perf_counter() - cache_read_started) * 1000.0,
    )
    _dem_alt_run_map_store(cache_hits_for_run_map)

    def _refresh_missing_from_cache(candidate_keys: list[tuple]) -> list[tuple]:
        if not candidate_keys:
            return []
        refreshed: list[tuple] = []
        refreshed_for_run_map: dict[tuple, int] = {}
        refreshed_read_started = time.perf_counter()
        with _DEM_ALT_CACHE_LOCK:
            for key in candidate_keys:
                cached = _DEM_ALT_CACHE.get(key)
                if cached is None:
                    refreshed.append(key)
                else:
                    values_by_key[key] = int(cached)
                    refreshed_for_run_map[key] = int(cached)
        _record_dense_linesearch_metrics(
            demAltCacheReadMs=(time.perf_counter() - refreshed_read_started) * 1000.0,
        )
        _dem_alt_run_map_store(refreshed_for_run_map)
        return refreshed

    def _compute_and_store_missing(candidate_keys: list[tuple]) -> None:
        nonlocal computed_missing_count
        if not candidate_keys:
            return
        computed_missing_count = len(candidate_keys)
        computed: dict[tuple[float, float], int] = {}
        bulk_started = time.perf_counter()
        fallback_reason = ""
        try:
            batch_points = [point_by_key[key] for key in candidate_keys]
            if not _runtime_bool_setting("replan_dem_batch_enabled", True):
                fallback_reason = "batch_disabled"
                raise RuntimeError("DEM batch lookup disabled by runtime setting")
            reset_terrain_elev_many_metrics()
            batch_values = terrain_elev_many(batch_points)
            terrain_metrics = get_terrain_elev_many_metrics(reset=True)
            _record_dense_linesearch_metrics(
                demTileResolveMs=float(terrain_metrics.get("demTileResolveMs") or 0.0),
                demTileCandidateIndexMs=float(terrain_metrics.get("demTileCandidateIndexMs") or 0.0),
                demTileCandidateAssignMs=float(terrain_metrics.get("demTileCandidateAssignMs") or 0.0),
                demTileApplyMs=float(terrain_metrics.get("demTileApplyMs") or 0.0),
                demTileFallbackScanMs=float(terrain_metrics.get("demTileFallbackScanMs") or 0.0),
                demTileLoadMs=float(terrain_metrics.get("demTileLoadMs") or 0.0),
                demTileLoadWaitMs=float(terrain_metrics.get("demTileLoadWaitMs") or 0.0),
                demNativeTransformMs=float(terrain_metrics.get("demNativeTransformMs") or 0.0),
                demRowColTransformMs=float(terrain_metrics.get("demRowColTransformMs") or 0.0),
                demPixelReadMs=float(terrain_metrics.get("demPixelReadMs") or 0.0),
                demCacheReadMs=float(terrain_metrics.get("demCacheReadMs") or 0.0),
                demCacheWriteMs=float(terrain_metrics.get("demCacheWriteMs") or 0.0),
                demTileCount=int(terrain_metrics.get("demTileCount") or 0),
                demTileCandidateCount=int(terrain_metrics.get("demTileCandidateCount") or 0),
                demTileFallbackCandidateCount=int(terrain_metrics.get("demTileFallbackCandidateCount") or 0),
                demTileApplyCallCount=int(terrain_metrics.get("demTileApplyCallCount") or 0),
                demTileLoadLeaderCount=int(terrain_metrics.get("demTileLoadLeaderCount") or 0),
                demTileLoadWaiterCount=int(terrain_metrics.get("demTileLoadWaiterCount") or 0),
                demTileLoadTimeoutCount=int(terrain_metrics.get("demTileLoadTimeoutCount") or 0),
                demResolvedByTile=int(terrain_metrics.get("demResolvedByTile") or 0),
                demPixelCacheHitCount=int(terrain_metrics.get("demPixelCacheHitCount") or 0),
                demPixelCacheMissCount=int(terrain_metrics.get("demPixelCacheMissCount") or 0),
                demPixelCacheUniqueMissCount=int(terrain_metrics.get("demPixelCacheUniqueMissCount") or 0),
                demPixelCacheClearCount=int(terrain_metrics.get("demPixelCacheClearCount") or 0),
            )
            if len(batch_values) != len(batch_points):
                fallback_reason = "result_length_mismatch"
                raise RuntimeError("terrain_elev_many returned an unexpected result length")
            for key, value in zip(candidate_keys, batch_values):
                computed[key] = int(round(float(value)))
        except Exception as exc:
            if not fallback_reason:
                fallback_reason = type(exc).__name__ or "unknown"
            _record_dense_linesearch_metrics(
                demBatchFallbackCount=1,
                demBatchFallbackReason=fallback_reason,
            )
            computed.clear()
            for key in candidate_keys:
                lat, lon = point_by_key[key]
                computed[key] = int(round(terrain_elev(float(lat), float(lon))))
        finally:
            _record_dense_linesearch_metrics(
                demBulkLookupMs=(time.perf_counter() - bulk_started) * 1000.0,
            )
        cache_write_started = time.perf_counter()
        clear_count = 0
        with _DEM_ALT_CACHE_LOCK:
            if len(computed) >= _DEM_ALT_CACHE_MAX:
                _DEM_ALT_CACHE.clear()
                clear_count = 1
                computed_items = list(computed.items())[-_DEM_ALT_CACHE_MAX:]
                _DEM_ALT_CACHE.update(dict(computed_items))
            else:
                if len(_DEM_ALT_CACHE) + len(computed) >= _DEM_ALT_CACHE_MAX:
                    _DEM_ALT_CACHE.clear()
                    clear_count = 1
                _DEM_ALT_CACHE.update(computed)
        _record_dense_linesearch_metrics(
            demAltCacheWriteMs=(time.perf_counter() - cache_write_started) * 1000.0,
            demAltCacheClearCount=clear_count,
        )
        values_by_key.update(computed)
        _dem_alt_run_map_store(computed)

    if missing:
        if _runtime_bool_setting("dem_alt_missing_singleflight_enabled", False):
            with _DEM_ALT_CACHE_MISSING_LOCK:
                missing = _refresh_missing_from_cache(missing)
                _compute_and_store_missing(missing)
        else:
            _compute_and_store_missing(missing)

    _record_dense_linesearch_metrics(
        demLookupCount=len(keys),
        demCacheHitCount=max(0, len(keys) - computed_missing_count),
        demCacheMissCount=computed_missing_count,
        demUniquePointCount=len(unique_keys),
    )
    return [int(values_by_key[key]) for key in keys]


def _materialize_dem_coords(points: list[tuple[float, float]]) -> list[dict]:
    if not points:
        return []
    alts = _dem_alt_many(points)
    return [
        {
            "latitude": float(lat),
            "longitude": float(lon),
            "altitude": int(alt),
            _DEM_ALT_COORD_TRUSTED_KEY: True,
        }
        for (lat, lon), alt in zip(points, alts)
    ]


def _materialize_dem_coord_groups(point_groups: list[list[tuple[float, float]]]) -> list[list[dict]]:
    if not point_groups:
        return []
    flat_points: list[tuple[float, float]] = []
    lengths: list[int] = []
    for points in point_groups:
        length = len(points or [])
        lengths.append(length)
        if length:
            flat_points.extend(points)
    if not flat_points:
        return [[] for _points in point_groups]

    flat_alts = _dem_alt_many(flat_points)
    groups: list[list[dict]] = []
    cursor = 0
    for length, points in zip(lengths, point_groups):
        group: list[dict] = []
        for lat, lon in points:
            alt = flat_alts[cursor]
            group.append({
                "latitude": float(lat),
                "longitude": float(lon),
                "altitude": int(alt),
                _DEM_ALT_COORD_TRUSTED_KEY: True,
            })
            cursor += 1
        groups.append(group)
    return groups


def _aircraft_alt_offset_m(aircraft_id: int) -> float:
    """Return per-aircraft altitude offset layer (1000/1010/1020m)."""
    if _get_runtime_altitude_layers_m is not None:
        try:
            layers = tuple(float(value) for value in _get_runtime_altitude_layers_m() or ())
            if layers:
                idx = (int(aircraft_id) - 1) % len(layers)
                return float(layers[idx])
        except Exception:
            pass
    try:
        idx = (int(aircraft_id) - 1) % len(ALTITUDE_LAYERS_M)
    except Exception:
        idx = 0
    return float(ALTITUDE_LAYERS_M[idx])


def _collect_ref_points_from_info(info: dict) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []

    for c in info.get("coordinateList") or []:
        try:
            points.append((float(c["latitude"]), float(c["longitude"])))
        except Exception:
            continue

    for line in info.get("lineList") or []:
        for c in line.get("coordinateList") or []:
            try:
                points.append((float(c["latitude"]), float(c["longitude"])))
            except Exception:
                continue

    for area in info.get("areaList") or []:
        for c in area.get("coordinateList") or []:
            try:
                points.append((float(c["latitude"]), float(c["longitude"])))
            except Exception:
                continue

    return points


def _median_ground_m(points: list[tuple[float, float]]) -> float | None:
    if not points:
        return None
    samples: list[int] = []
    usable: list[tuple[float, float]] = []
    for lat, lon in points:
        try:
            usable.append((float(lat), float(lon)))
        except Exception:
            continue
    if usable:
        try:
            samples.extend(_dem_alt_many(usable))
        except Exception:
            for lat, lon in usable:
                try:
                    samples.append(_dem_alt(lat, lon))
                except Exception:
                    continue
    if not samples:
        return None
    samples.sort()
    n = len(samples)
    mid = n // 2
    if n % 2:
        return float(samples[mid])
    return (float(samples[mid - 1]) + float(samples[mid])) / 2.0


def _sample_ground_profile_along_coords(
    coords: list[dict],
    *,
    sample_step_m: float = 120.0,
    max_samples_per_leg: int = 24,
) -> list[float]:
    sample_points = _sample_points_along_coords(
        coords,
        sample_step_m=sample_step_m,
        max_samples_per_leg=max_samples_per_leg,
    )
    if not sample_points:
        return []
    _record_dense_linesearch_metrics(groundProfileSampleCount=len(sample_points))
    try:
        return [float(value) for value in _dem_alt_many(sample_points)]
    except Exception:
        return [float(_dem_alt(lat, lon)) for lat, lon in sample_points]


def _sample_points_along_coords(
    coords: list[dict],
    *,
    sample_step_m: float = 120.0,
    max_samples_per_leg: int = 24,
) -> list[tuple[float, float]]:
    usable: list[tuple[float, float]] = []
    for coord in coords or []:
        if not isinstance(coord, dict):
            continue
        try:
            usable.append((float(coord["latitude"]), float(coord["longitude"])))
        except Exception:
            continue
    if not usable:
        return []

    if _runtime_bool_setting("ground_required_polyline_resample_enabled", False):
        sample_points: list[tuple[float, float]] = [usable[0]]
        step_m = max(float(sample_step_m), 1.0)
        max_steps = max(int(max_samples_per_leg), 1)
        distance_since_last = 0.0

        for (prev_lat, prev_lon), (curr_lat, curr_lon) in zip(usable[:-1], usable[1:]):
            seg_dist = _dist_between_coords(
                {"latitude": prev_lat, "longitude": prev_lon},
                {"latitude": curr_lat, "longitude": curr_lon},
            )
            if seg_dist <= 0.0:
                continue

            if seg_dist > step_m * max_steps:
                subdivisions = max_steps
                for step_idx in range(1, subdivisions + 1):
                    ratio = step_idx / subdivisions
                    sample_points.append((
                        prev_lat + ((curr_lat - prev_lat) * ratio),
                        prev_lon + ((curr_lon - prev_lon) * ratio),
                    ))
                distance_since_last = 0.0
                continue

            travelled = 0.0
            while distance_since_last + (seg_dist - travelled) >= step_m:
                distance_to_next = step_m - distance_since_last
                travelled += distance_to_next
                ratio = min(1.0, max(0.0, travelled / seg_dist))
                sample_points.append((
                    prev_lat + ((curr_lat - prev_lat) * ratio),
                    prev_lon + ((curr_lon - prev_lon) * ratio),
                ))
                distance_since_last = 0.0
            distance_since_last += max(0.0, seg_dist - travelled)

        last = usable[-1]
        if sample_points[-1] != last:
            sample_points.append(last)
        return sample_points

    sample_points: list[tuple[float, float]] = [usable[0]]
    step_m = max(float(sample_step_m), 1.0)
    max_steps = max(int(max_samples_per_leg), 1)
    for (prev_lat, prev_lon), (curr_lat, curr_lon) in zip(usable[:-1], usable[1:]):
        seg_dist = _dist_between_coords(
            {"latitude": prev_lat, "longitude": prev_lon},
            {"latitude": curr_lat, "longitude": curr_lon},
        )
        subdivisions = max(1, int(math.ceil(seg_dist / step_m)))
        subdivisions = min(subdivisions, max_steps)
        for step_idx in range(1, subdivisions + 1):
            ratio = step_idx / subdivisions
            lat = prev_lat + ((curr_lat - prev_lat) * ratio)
            lon = prev_lon + ((curr_lon - prev_lon) * ratio)
            sample_points.append((lat, lon))
    return sample_points


def _ground_mid_m_from_coords(
    coords: list[dict],
    *,
    fallback_coord: dict | None = None,
    fallback_ground_ref_m: float | None = None,
) -> float | None:
    samples = _sample_ground_profile_along_coords(coords)
    if not samples and isinstance(fallback_coord, dict):
        lat = _safe_float_or_none(fallback_coord.get("latitude"))
        lon = _safe_float_or_none(fallback_coord.get("longitude"))
        if lat is not None and lon is not None:
            samples.append(float(_dem_alt(lat, lon)))
    if not samples:
        return float(fallback_ground_ref_m) if fallback_ground_ref_m is not None else None
    return (min(samples) + max(samples)) / 2.0


def _ground_mean_m_from_sweep_coords(
    line_coords: list[dict],
    *,
    fallback_coord: dict | None = None,
    fallback_ground_ref_m: float | None = None,
) -> float | None:
    """Return mean DEM elevation sampled at the exact sweep coordinateList points (p1, centroid, p2)."""
    sample_points: list[tuple[float, float]] = []
    for c in line_coords or []:
        if not isinstance(c, dict):
            continue
        lat = _safe_float_or_none(c.get("latitude"))
        lon = _safe_float_or_none(c.get("longitude"))
        if lat is None or lon is None:
            continue
        sample_points.append((float(lat), float(lon)))
    try:
        samples = [float(value) for value in _dem_alt_many(sample_points)]
    except Exception:
        samples = []
        for lat, lon in sample_points:
            try:
                samples.append(float(_dem_alt(lat, lon)))
            except Exception:
                continue
    if not samples and isinstance(fallback_coord, dict):
        lat = _safe_float_or_none(fallback_coord.get("latitude"))
        lon = _safe_float_or_none(fallback_coord.get("longitude"))
        if lat is not None and lon is not None:
            try:
                samples.append(float(_dem_alt(lat, lon)))
            except Exception:
                pass
    if not samples:
        return float(fallback_ground_ref_m) if fallback_ground_ref_m is not None else None
    return sum(samples) / len(samples)


def _line_coords_ground_signature(line_coords: list[dict]) -> tuple | None:
    count = 0
    first: tuple[float, float] | None = None
    last: tuple[float, float] | None = None
    lat_sum = 0.0
    lon_sum = 0.0
    for c in line_coords or []:
        if not isinstance(c, dict):
            continue
        lat = _safe_float_or_none(c.get("latitude"))
        lon = _safe_float_or_none(c.get("longitude"))
        if lat is None or lon is None:
            continue
        pair = (round(float(lat), 7), round(float(lon), 7))
        if first is None:
            first = pair
        last = pair
        lat_sum += pair[0]
        lon_sum += pair[1]
        count += 1
    if count <= 0 or first is None or last is None:
        return None
    return (
        count,
        first[0],
        first[1],
        last[0],
        last[1],
        round(lat_sum, 7),
        round(lon_sum, 7),
    )


def _line_coords_ground_signature_for_search(
    line_coords: list[dict],
    line_search: dict | None = None,
) -> tuple | None:
    if isinstance(line_search, dict):
        cached_signature = line_search.get(_GROUND_REQUIRED_LINE_SIGNATURE_KEY)
        if (
            isinstance(cached_signature, tuple)
            and line_search.get(_GROUND_REQUIRED_LINE_SIGNATURE_COORDS_ID_KEY) == id(line_coords)
            and line_search.get(_GROUND_REQUIRED_LINE_SIGNATURE_COORDS_COUNT_KEY) == len(line_coords)
        ):
            _record_dense_linesearch_metrics(groundRequiredSignatureCacheHitCount=1)
            return cached_signature
    signature = _line_coords_ground_signature(line_coords)
    if signature is not None and isinstance(line_search, dict):
        line_search[_GROUND_REQUIRED_LINE_SIGNATURE_KEY] = signature
        line_search[_GROUND_REQUIRED_LINE_SIGNATURE_COORDS_ID_KEY] = id(line_coords)
        line_search[_GROUND_REQUIRED_LINE_SIGNATURE_COORDS_COUNT_KEY] = len(line_coords)
        _record_dense_linesearch_metrics(groundRequiredSignatureCacheStoreCount=1)
    return signature


def _ground_required_m_from_sweep_coords(
    line_coords: list[dict],
    *,
    sample_step_m: float = 120.0,
    fallback_coord: dict | None = None,
    fallback_ground_ref_m: float | None = None,
) -> float | None:
    """Return the highest DEM elevation in the filmed sweep footprint."""
    started_at = time.perf_counter()
    try:
        samples: list[float] = _sample_ground_profile_along_coords(
            line_coords,
            sample_step_m=sample_step_m,
        )
        if not samples and isinstance(fallback_coord, dict):
            lat = _safe_float_or_none(fallback_coord.get("latitude"))
            lon = _safe_float_or_none(fallback_coord.get("longitude"))
            if lat is not None and lon is not None:
                try:
                    samples.append(float(_dem_alt(lat, lon)))
                except Exception:
                    pass
        if not samples:
            return float(fallback_ground_ref_m) if fallback_ground_ref_m is not None else None
        return max(samples)
    finally:
        _record_dense_linesearch_metrics(
            groundRequiredComputeMs=(time.perf_counter() - started_at) * 1000.0,
        )


def _waypoint_required_ground_m(
    wp: dict,
    coord: dict,
    *,
    fallback_ground_ref_m: float | None = None,
) -> float | None:
    fp = wp.get("filmingProperty") if isinstance(wp.get("filmingProperty"), dict) else {}
    ls = fp.get("lineSearch") if isinstance(fp.get("lineSearch"), dict) else {}
    line_coords = ls.get("coordinateList") or []
    if line_coords:
        cached_ground = _safe_float_or_none(wp.get(_GROUND_REQUIRED_CACHE_KEY))
        if (
            cached_ground is not None
            and wp.get(_GROUND_REQUIRED_COORDS_ID_KEY) == id(line_coords)
            and wp.get(_GROUND_REQUIRED_COORDS_COUNT_KEY) == len(line_coords)
        ):
            return float(cached_ground)
        signature = _line_coords_ground_signature_for_search(line_coords, ls)
        if signature is not None:
            if cached_ground is not None and wp.get(_GROUND_REQUIRED_SIGNATURE_KEY) == signature:
                wp[_GROUND_REQUIRED_COORDS_ID_KEY] = id(line_coords)
                wp[_GROUND_REQUIRED_COORDS_COUNT_KEY] = len(line_coords)
                return float(cached_ground)
        _record_dense_linesearch_metrics(groundRequiredLazyFallbackCount=1)
        ground_required = _ground_required_m_from_sweep_coords(
            line_coords,
            sample_step_m=_ground_required_sample_step_m(ls),
            fallback_coord=coord,
            fallback_ground_ref_m=fallback_ground_ref_m,
        )
        if ground_required is not None and signature is not None:
            wp[_GROUND_REQUIRED_CACHE_KEY] = float(ground_required)
            wp[_GROUND_REQUIRED_SIGNATURE_KEY] = signature
            wp[_GROUND_REQUIRED_COORDS_ID_KEY] = id(line_coords)
            wp[_GROUND_REQUIRED_COORDS_COUNT_KEY] = len(line_coords)
        return ground_required

    orientation = fp.get("coordinateOrientation") if isinstance(fp.get("coordinateOrientation"), dict) else {}
    target_coord = orientation.get("coordinate") if isinstance(orientation.get("coordinate"), dict) else None
    ref_coord = target_coord if isinstance(target_coord, dict) else coord
    lat = _safe_float_or_none(ref_coord.get("latitude")) if isinstance(ref_coord, dict) else None
    lon = _safe_float_or_none(ref_coord.get("longitude")) if isinstance(ref_coord, dict) else None
    if lat is None or lon is None:
        return float(fallback_ground_ref_m) if fallback_ground_ref_m is not None else None
    try:
        return float(_dem_alt(lat, lon))
    except Exception:
        return float(fallback_ground_ref_m) if fallback_ground_ref_m is not None else None


def _altitude_precompute_enabled() -> bool:
    return _runtime_bool_setting("replan_0303_altitude_precompute_enabled", True)


def _ground_required_final_geometry_preowner_enabled() -> bool:
    return _runtime_bool_setting("ground_required_final_geometry_preowner_enabled", False)


def _compute_coord_altitude_ground_required_map(
    pending_entries: list[tuple[tuple, list[dict], dict, dict]],
    *,
    allow_untrusted: bool = False,
) -> tuple[dict[tuple, float], list[tuple[tuple, list[dict], dict, dict]]]:
    if not pending_entries:
        return {}, []

    trusted_enabled = _runtime_bool_setting(
        "ground_required_trusted_coord_altitude_fast_path_enabled",
        True,
    )
    untrusted_enabled = bool(
        allow_untrusted
        and _runtime_bool_setting("ground_required_coord_altitude_fast_path_enabled", False)
    )
    if not trusted_enabled and not untrusted_enabled:
        return {}, list(pending_entries)

    resolved: dict[tuple, float] = {}
    remaining: list[tuple[tuple, list[dict], dict, dict]] = []
    for signature, line_coords, ls, coord in pending_entries:
        sample_step_m = _ground_required_sample_step_m(ls)
        coord_altitude_ground: float | None = None
        if trusted_enabled:
            coord_altitude_ground = _dense_line_coords_ground_required_from_coord_altitudes(
                line_coords,
                line_search=ls,
                sample_step_m=sample_step_m,
                require_trusted_dem_altitude=True,
            )
        if coord_altitude_ground is None and untrusted_enabled:
            coord_altitude_ground = _dense_line_coords_ground_required_from_coord_altitudes(
                line_coords,
                line_search=ls,
                sample_step_m=sample_step_m,
                require_trusted_dem_altitude=False,
            )
        if coord_altitude_ground is None:
            remaining.append((signature, line_coords, ls, coord))
            continue
        resolved[signature] = float(coord_altitude_ground)
    return resolved, remaining


def _formation_follower_postprocess_workers(follower_count: int) -> int:
    count = max(0, int(follower_count or 0))
    if count < max(1, int(FORMATION_FOLLOWER_POSTPROCESS_PARALLEL_MIN_FOLLOWERS)):
        return 1
    if not _runtime_bool_setting("formation_follower_postprocess_parallel_enabled", True):
        return 1
    requested = max(
        1,
        _runtime_int_setting(
            "formation_follower_postprocess_workers",
            int(FORMATION_FOLLOWER_POSTPROCESS_WORKERS),
        ),
    )
    return max(1, min(count, requested))


def _ground_required_final_prepass_fresh_skip_enabled() -> bool:
    return _runtime_bool_setting("ground_required_final_prepass_fresh_skip_enabled", True)


def _ground_required_precompute_already_fresh(
    wps: list[OrderedDict],
    *,
    signature_cache: dict[tuple, float] | None = None,
) -> bool:
    if not wps or not _altitude_precompute_enabled():
        return True
    started_at = time.perf_counter()
    cache = signature_cache if isinstance(signature_cache, dict) else {}
    saw_line_search = False
    try:
        for wp in wps or []:
            if not isinstance(wp, dict):
                continue
            fp = wp.get("filmingProperty") if isinstance(wp.get("filmingProperty"), dict) else {}
            ls = fp.get("lineSearch") if isinstance(fp.get("lineSearch"), dict) else {}
            line_coords = ls.get("coordinateList") or []
            if not line_coords:
                continue
            saw_line_search = True
            cached_ground = _safe_float_or_none(wp.get(_GROUND_REQUIRED_CACHE_KEY))
            if (
                cached_ground is not None
                and wp.get(_GROUND_REQUIRED_COORDS_ID_KEY) == id(line_coords)
                and wp.get(_GROUND_REQUIRED_COORDS_COUNT_KEY) == len(line_coords)
            ):
                continue
            signature = _line_coords_ground_signature_for_search(line_coords, ls)
            if signature is None:
                continue
            if cached_ground is not None and wp.get(_GROUND_REQUIRED_SIGNATURE_KEY) == signature:
                wp[_GROUND_REQUIRED_COORDS_ID_KEY] = id(line_coords)
                wp[_GROUND_REQUIRED_COORDS_COUNT_KEY] = len(line_coords)
                continue
            if signature in cache:
                wp[_GROUND_REQUIRED_CACHE_KEY] = float(cache[signature])
                wp[_GROUND_REQUIRED_SIGNATURE_KEY] = signature
                wp[_GROUND_REQUIRED_COORDS_ID_KEY] = id(line_coords)
                wp[_GROUND_REQUIRED_COORDS_COUNT_KEY] = len(line_coords)
                continue
            return False
        if saw_line_search:
            _record_dense_linesearch_metrics(groundRequiredFinalPrepassFreshSkipCount=1)
        return True
    finally:
        _record_dense_linesearch_metrics(
            groundRequiredFinalPrepassFreshSkipMs=(time.perf_counter() - started_at) * 1000.0,
        )


def _precompute_ground_required_for_waypoints(
    wps: list[OrderedDict],
    *,
    signature_cache: dict[tuple, float] | None = None,
    fallback_ground_ref_m: float | None = None,
) -> None:
    if not wps or not _altitude_precompute_enabled():
        return
    started_at = time.perf_counter()
    cache = signature_cache if isinstance(signature_cache, dict) else {}
    pending_entries: list[tuple[tuple, list[dict], dict, dict]] = []
    pending: list[tuple[tuple, list[tuple[float, float]], list[dict]]] = []
    pending_waiters: dict[tuple, list[tuple[dict, list[dict], dict, dict]]] = {}
    inflight_entries: list[tuple[tuple, threading.Event]] = []
    owned_process_cache_signatures: set[tuple] = set()
    finished_process_cache_signatures: set[tuple] = set()
    published_process_cache_signatures: set[tuple] = set()
    total_sample_count = 0
    key_prefix = _dem_alt_cache_prefix()
    process_cache_prefix = (
        _ground_required_process_cache_prefix()
        if _ground_required_process_cache_enabled()
        else None
    )
    owner_compute_started_at: float | None = None
    owner_compute_recorded = False

    def _record_owner_compute_elapsed() -> None:
        nonlocal owner_compute_recorded
        if owner_compute_recorded or owner_compute_started_at is None:
            return
        _record_dense_linesearch_metrics(
            groundRequiredOwnerComputeMs=(time.perf_counter() - owner_compute_started_at) * 1000.0,
        )
        owner_compute_recorded = True

    def _assign_ground_required_to_wp(
        wp: dict,
        line_coords: list[dict],
        signature: tuple,
        ground_required: float,
        *,
        metric_key: str = "groundRequiredHitAssignMs",
    ) -> None:
        assign_started_at = time.perf_counter()
        wp[_GROUND_REQUIRED_CACHE_KEY] = float(ground_required)
        wp[_GROUND_REQUIRED_SIGNATURE_KEY] = signature
        wp[_GROUND_REQUIRED_COORDS_ID_KEY] = id(line_coords)
        wp[_GROUND_REQUIRED_COORDS_COUNT_KEY] = len(line_coords)
        _record_dense_linesearch_metrics(
            **{metric_key: (time.perf_counter() - assign_started_at) * 1000.0}
        )

    def _finish_owned_process_cache_signatures(signatures: Iterable[tuple] | None = None) -> None:
        selected = set(signatures or owned_process_cache_signatures)
        for signature in selected:
            if signature in finished_process_cache_signatures:
                continue
            if signature not in owned_process_cache_signatures:
                continue
            _ground_required_process_cache_finish(signature, key_prefix=process_cache_prefix)
            finished_process_cache_signatures.add(signature)

    def _put_and_finish_owned_process_cache_signature(signature: tuple, ground_required: float) -> None:
        _ground_required_process_cache_put(
            signature,
            float(ground_required),
            key_prefix=process_cache_prefix,
        )
        if signature in owned_process_cache_signatures:
            published_process_cache_signatures.add(signature)
            _finish_owned_process_cache_signatures([signature])

    def _assign_ground_required_to_waiters(signature: tuple, ground_required: float) -> None:
        assign_started_at = time.perf_counter()
        cache[signature] = float(ground_required)
        for wp, line_coords, _ls, _coord in pending_waiters.get(signature, []):
            wp[_GROUND_REQUIRED_CACHE_KEY] = float(ground_required)
            wp[_GROUND_REQUIRED_SIGNATURE_KEY] = signature
            wp[_GROUND_REQUIRED_COORDS_ID_KEY] = id(line_coords)
            wp[_GROUND_REQUIRED_COORDS_COUNT_KEY] = len(line_coords)
        _record_dense_linesearch_metrics(
            groundRequiredWaiterAssignMs=(time.perf_counter() - assign_started_at) * 1000.0,
        )

    def _waiter_local_fast_path_ground(
        signature: tuple,
        *,
        publish_to_process_cache: bool = True,
        finish_process_cache: bool = True,
    ) -> float | None:
        if not (
            _runtime_bool_setting("ground_required_process_cache_waiter_grace_fast_path_enabled", False)
            or _runtime_bool_setting("ground_required_process_cache_waiter_batch_local_fast_path_enabled", True)
        ):
            return None
        waiters = pending_waiters.get(signature) or []
        if not waiters:
            return None
        _wp, line_coords, ls, _coord = waiters[0]
        coord_altitude_ground, _remaining = _compute_coord_altitude_ground_required_map(
            [(signature, line_coords, ls, _coord if isinstance(_coord, dict) else {})],
            allow_untrusted=True,
        )
        local_coord_ground = coord_altitude_ground.get(signature)
        if local_coord_ground is not None:
            ground_required = float(local_coord_ground)
            cache[signature] = ground_required
            if publish_to_process_cache:
                _ground_required_process_cache_put(signature, ground_required, key_prefix=process_cache_prefix)
            if finish_process_cache:
                _ground_required_process_cache_finish(signature, key_prefix=process_cache_prefix)
            _record_dense_linesearch_metrics(
                groundRequiredPrecomputeCount=1,
                groundRequiredProcessCacheInflightLocalFastPathCount=1,
            )
            return ground_required
        local_ground_required = _dense_line_coords_ground_required_fast_path(
            line_coords,
            sample_step_m=_ground_required_sample_step_m(ls),
            key_prefix=key_prefix,
            cache_lock_blocking=False,
        )
        if local_ground_required is None:
            return None
        ground_required = float(local_ground_required)
        cache[signature] = ground_required
        if publish_to_process_cache:
            _ground_required_process_cache_put(signature, ground_required, key_prefix=process_cache_prefix)
        if finish_process_cache:
            _ground_required_process_cache_finish(signature, key_prefix=process_cache_prefix)
        _record_dense_linesearch_metrics(
            groundRequiredPrecomputeCount=1,
            groundRequiredProcessCacheInflightLocalFastPathCount=1,
        )
        return ground_required

    def _try_assign_waiter_local_fast_path(signature: tuple) -> bool:
        local_ground = _waiter_local_fast_path_ground(
            signature,
            publish_to_process_cache=False,
            finish_process_cache=False,
        )
        if local_ground is None:
            return False
        _assign_ground_required_to_waiters(signature, float(local_ground))
        return True

    def _wait_for_inflight_entries() -> None:
        if (
            inflight_entries
            and _runtime_bool_setting(
                "ground_required_process_cache_waiter_batch_local_fast_path_enabled",
                True,
            )
            and not _runtime_bool_setting("ground_required_process_cache_waiter_grace_fast_path_enabled", False)
        ):
            grace_ms = max(
                0.0,
                _runtime_float_setting(
                    "ground_required_process_cache_waiter_batch_local_fast_path_grace_ms",
                    0.0,
                ),
            )
            if grace_ms > 0.0:
                time.sleep(grace_ms / 1000.0)
            for signature, event in inflight_entries:
                if signature in cache:
                    continue
                cached_ground = None
                if event.is_set():
                    cached_ground = _ground_required_process_cache_get(
                        signature,
                        key_prefix=process_cache_prefix,
                    )
                if cached_ground is not None:
                    _assign_ground_required_to_waiters(signature, float(cached_ground))
                    continue
                _try_assign_waiter_local_fast_path(signature)

        for signature, event in inflight_entries:
            if signature in cache:
                continue
            if _runtime_bool_setting("ground_required_process_cache_waiter_grace_fast_path_enabled", False):
                wait_started_at = time.perf_counter()
                grace_ms = max(
                    0.0,
                    _runtime_float_setting(
                        "ground_required_process_cache_waiter_fast_path_grace_ms",
                        30.0,
                    ),
                )
                if grace_ms > 0.0:
                    if not event.wait(grace_ms / 1000.0):
                        elapsed_ms = (time.perf_counter() - wait_started_at) * 1000.0
                        _record_dense_linesearch_metrics(
                            groundRequiredProcessCacheInflightWaitMs=elapsed_ms,
                            groundRequiredMaterializationWaitMs=elapsed_ms,
                        )
                        local_ground = _waiter_local_fast_path_ground(signature)
                        if local_ground is not None:
                            _assign_ground_required_to_waiters(signature, float(local_ground))
                            continue
                    else:
                        elapsed_ms = (time.perf_counter() - wait_started_at) * 1000.0
                        _record_dense_linesearch_metrics(
                            groundRequiredProcessCacheInflightWaitMs=elapsed_ms,
                            groundRequiredMaterializationWaitMs=elapsed_ms,
                        )
                        cached_ground = _ground_required_process_cache_get(
                            signature,
                            key_prefix=process_cache_prefix,
                        )
                        if cached_ground is not None:
                            _assign_ground_required_to_waiters(signature, float(cached_ground))
                            continue
            ground_required = _ground_required_process_cache_wait(
                signature,
                event,
                key_prefix=process_cache_prefix,
            )
            if ground_required is None:
                continue
            _assign_ground_required_to_waiters(signature, float(ground_required))

    try:
        scan_started_at = time.perf_counter()
        for wp in wps or []:
            if not isinstance(wp, dict):
                continue
            fp = wp.get("filmingProperty") if isinstance(wp.get("filmingProperty"), dict) else {}
            ls = fp.get("lineSearch") if isinstance(fp.get("lineSearch"), dict) else {}
            line_coords = ls.get("coordinateList") or []
            if not line_coords:
                continue
            cached_ground = _safe_float_or_none(wp.get(_GROUND_REQUIRED_CACHE_KEY))
            if (
                cached_ground is not None
                and wp.get(_GROUND_REQUIRED_COORDS_ID_KEY) == id(line_coords)
                and wp.get(_GROUND_REQUIRED_COORDS_COUNT_KEY) == len(line_coords)
            ):
                _record_dense_linesearch_metrics(groundRequiredPrecomputeHitCount=1)
                continue
            signature = _line_coords_ground_signature_for_search(line_coords, ls)
            if signature is None:
                continue
            if cached_ground is not None and wp.get(_GROUND_REQUIRED_SIGNATURE_KEY) == signature:
                assign_started_at = time.perf_counter()
                wp[_GROUND_REQUIRED_COORDS_ID_KEY] = id(line_coords)
                wp[_GROUND_REQUIRED_COORDS_COUNT_KEY] = len(line_coords)
                _record_dense_linesearch_metrics(groundRequiredPrecomputeHitCount=1)
                _record_dense_linesearch_metrics(
                    groundRequiredHitAssignMs=(time.perf_counter() - assign_started_at) * 1000.0,
                )
                continue
            if signature in cache:
                ground_required = float(cache[signature])
                _record_dense_linesearch_metrics(
                    groundRequiredPrecomputeHitCount=1,
                    groundRequiredMaterializationCacheHitCount=1,
                )
                _assign_ground_required_to_wp(wp, line_coords, signature, ground_required)
                continue
            if signature in pending_waiters:
                coord = wp.get("coordinate") if isinstance(wp.get("coordinate"), dict) else {}
                pending_waiters[signature].append((
                    wp,
                    line_coords,
                    ls,
                    coord if isinstance(coord, dict) else {},
                ))
                _record_dense_linesearch_metrics(groundRequiredPrecomputeHitCount=1)
                continue
            process_cached_ground, process_event, process_owner = (
                _ground_required_process_cache_get_or_claim(signature, key_prefix=process_cache_prefix)
            )
            if process_cached_ground is not None:
                ground_required = float(process_cached_ground)
                cache[signature] = float(ground_required)
                _assign_ground_required_to_wp(wp, line_coords, signature, ground_required)
                continue
            if process_event is not None and not process_owner:
                if _runtime_bool_setting(
                    "ground_required_process_cache_inflight_local_fast_path_enabled",
                    False,
                ):
                    sample_step_m = _ground_required_sample_step_m(ls)
                    local_ground_required = _dense_line_coords_ground_required_fast_path(
                        line_coords,
                        sample_step_m=sample_step_m,
                        key_prefix=key_prefix,
                        cache_lock_blocking=False,
                    )
                    if local_ground_required is not None:
                        ground_required = float(local_ground_required)
                        cache[signature] = float(ground_required)
                        _ground_required_process_cache_put(
                            signature,
                            float(ground_required),
                            key_prefix=process_cache_prefix,
                        )
                        _record_dense_linesearch_metrics(
                            groundRequiredPrecomputeCount=1,
                            groundRequiredProcessCacheInflightLocalFastPathCount=1,
                        )
                        _assign_ground_required_to_wp(wp, line_coords, signature, ground_required)
                        continue
                coord = wp.get("coordinate") if isinstance(wp.get("coordinate"), dict) else {}
                pending_waiters[signature] = [(
                    wp,
                    line_coords,
                    ls,
                    coord if isinstance(coord, dict) else {},
                )]
                inflight_entries.append((signature, process_event))
                continue
            if process_owner and process_event is not None:
                owned_process_cache_signatures.add(signature)

            coord = wp.get("coordinate") if isinstance(wp.get("coordinate"), dict) else {}
            pending_entries.append((signature, line_coords, ls, coord if isinstance(coord, dict) else {}))
            pending_waiters[signature] = [(wp, line_coords, ls, coord if isinstance(coord, dict) else {})]
        _record_dense_linesearch_metrics(
            groundRequiredScanMs=(time.perf_counter() - scan_started_at) * 1000.0,
        )
        if pending_entries:
            owner_compute_started_at = time.perf_counter()

        dense_ground_by_signature: dict[tuple, float] = {}
        dense_ground_by_signature, remaining_pending_entries = (
            _compute_coord_altitude_ground_required_map(
                pending_entries,
                allow_untrusted=True,
            )
        )

        stamp_enabled = bool(
            remaining_pending_entries and _runtime_bool_setting("ground_required_line_coord_stamp_enabled", False)
        )
        if stamp_enabled:
            stamp_groups: list[list[dict]] = []
            for _signature, line_coords, ls, _coord in remaining_pending_entries:
                if _line_coords_dense_fast_path_geometry_ok(
                    line_coords,
                    sample_step_m=_ground_required_sample_step_m(ls),
                ):
                    stamp_groups.append(line_coords)
            _stamp_line_coord_dem_altitudes(
                stamp_groups,
                key_prefix=key_prefix,
            )
        if not stamp_enabled:
            dense_ground_by_signature.update(
                _compute_dense_line_coord_ground_required_map(
                    remaining_pending_entries,
                    key_prefix=key_prefix,
                )
            )
        early_dense_process_cache_puts = {
            signature: float(ground_required)
            for signature, ground_required in dense_ground_by_signature.items()
            if signature in owned_process_cache_signatures
        }
        if early_dense_process_cache_puts:
            _ground_required_process_cache_put_many(
                early_dense_process_cache_puts,
                key_prefix=process_cache_prefix,
            )
            published_process_cache_signatures.update(early_dense_process_cache_puts.keys())
            _finish_owned_process_cache_signatures(early_dense_process_cache_puts.keys())

        sample_build_started_at = time.perf_counter()
        dense_process_cache_puts: dict[tuple, float] = {}
        for signature, line_coords, ls, coord in pending_entries:
            dense_ground_required = dense_ground_by_signature.get(signature)
            if dense_ground_required is not None:
                ground_required = float(dense_ground_required)
                cache[signature] = float(ground_required)
                if signature not in published_process_cache_signatures:
                    dense_process_cache_puts[signature] = float(ground_required)
                _record_dense_linesearch_metrics(groundRequiredPrecomputeCount=1)
                for wp, waiter_line_coords, _ls, _coord in pending_waiters.get(signature, []):
                    wp[_GROUND_REQUIRED_CACHE_KEY] = float(ground_required)
                    wp[_GROUND_REQUIRED_SIGNATURE_KEY] = signature
                    wp[_GROUND_REQUIRED_COORDS_ID_KEY] = id(waiter_line_coords)
                    wp[_GROUND_REQUIRED_COORDS_COUNT_KEY] = len(waiter_line_coords)
                continue

            sample_step_m = _ground_required_sample_step_m(ls)
            dense_ground_required = _dense_line_coords_ground_required_fast_path(
                line_coords,
                sample_step_m=sample_step_m,
                key_prefix=key_prefix,
            )
            if dense_ground_required is not None:
                ground_required = float(dense_ground_required)
                cache[signature] = float(ground_required)
                _put_and_finish_owned_process_cache_signature(signature, ground_required)
                _record_dense_linesearch_metrics(groundRequiredPrecomputeCount=1)
                for wp, waiter_line_coords, _ls, _coord in pending_waiters.get(signature, []):
                    wp[_GROUND_REQUIRED_CACHE_KEY] = float(ground_required)
                    wp[_GROUND_REQUIRED_SIGNATURE_KEY] = signature
                    wp[_GROUND_REQUIRED_COORDS_ID_KEY] = id(waiter_line_coords)
                    wp[_GROUND_REQUIRED_COORDS_COUNT_KEY] = len(waiter_line_coords)
                continue

            sample_points = _sample_points_along_coords(
                line_coords,
                sample_step_m=sample_step_m,
            )
            if not sample_points and isinstance(coord, dict):
                lat = _safe_float_or_none(coord.get("latitude"))
                lon = _safe_float_or_none(coord.get("longitude"))
                if lat is not None and lon is not None:
                    sample_points = [(float(lat), float(lon))]
            if not sample_points:
                if fallback_ground_ref_m is None:
                    continue
                ground_required = float(fallback_ground_ref_m)
                cache[signature] = float(ground_required)
                _put_and_finish_owned_process_cache_signature(signature, ground_required)
                _record_dense_linesearch_metrics(groundRequiredPrecomputeCount=1)
                for wp, waiter_line_coords, _ls, _coord in pending_waiters.get(signature, []):
                    wp[_GROUND_REQUIRED_CACHE_KEY] = float(ground_required)
                    wp[_GROUND_REQUIRED_SIGNATURE_KEY] = signature
                    wp[_GROUND_REQUIRED_COORDS_ID_KEY] = id(waiter_line_coords)
                    wp[_GROUND_REQUIRED_COORDS_COUNT_KEY] = len(waiter_line_coords)
                continue
            pending.append((signature, sample_points, line_coords))
            total_sample_count += len(sample_points)

        if dense_process_cache_puts:
            _ground_required_process_cache_put_many(
                dense_process_cache_puts,
                key_prefix=process_cache_prefix,
            )
            _finish_owned_process_cache_signatures(dense_process_cache_puts.keys())

        if not pending:
            _record_dense_linesearch_metrics(
                groundRequiredSampleBuildMs=(time.perf_counter() - sample_build_started_at) * 1000.0,
            )
            _record_owner_compute_elapsed()
            _finish_owned_process_cache_signatures()
            _wait_for_inflight_entries()
            return

        if total_sample_count:
            _record_dense_linesearch_metrics(groundProfileSampleCount=total_sample_count)

        unique_points_by_key: "OrderedDict[tuple[float, float], tuple[float, float]]" = OrderedDict()
        values_by_key: dict[tuple[float, float], float] = {}
        reused_line_coord_keys: set[tuple[float, float]] = set()
        for _signature, sample_points, line_coords in pending:
            if line_coords:
                for key, value in _trusted_line_coord_dem_values(line_coords, key_prefix=key_prefix).items():
                    if key not in values_by_key:
                        values_by_key[key] = float(value)
                        reused_line_coord_keys.add(key)
            for lat, lon in sample_points:
                key = _dem_alt_cache_key(float(lat), float(lon), prefix=key_prefix)
                if key in values_by_key:
                    continue
                if key not in unique_points_by_key:
                    unique_points_by_key[key] = (float(lat), float(lon))
        _record_dense_linesearch_metrics(
            groundRequiredSampleBuildMs=(time.perf_counter() - sample_build_started_at) * 1000.0,
        )
        if reused_line_coord_keys:
            _record_dense_linesearch_metrics(
                groundRequiredLineCoordReuseCount=len(reused_line_coord_keys),
            )

        if unique_points_by_key:
            dem_lookup_started_at = time.perf_counter()
            try:
                unique_points = list(unique_points_by_key.values())
                values = _dem_alt_many(unique_points, prefix=key_prefix)
                if len(values) != len(unique_points):
                    raise RuntimeError("DEM precompute result length mismatch")
                computed_values_by_key = {
                    key: float(value)
                    for key, value in zip(unique_points_by_key.keys(), values)
                }
                values_by_key.update(computed_values_by_key)
            except Exception:
                pass
            finally:
                _record_dense_linesearch_metrics(
                    groundRequiredDemLookupMs=(time.perf_counter() - dem_lookup_started_at) * 1000.0,
                )

        result_assign_started_at = time.perf_counter()
        for signature, sample_points, _line_coords in pending:
            samples = [
                values_by_key[key]
                for key in (_dem_alt_cache_key(float(lat), float(lon), prefix=key_prefix) for lat, lon in sample_points)
                if key in values_by_key
            ]
            if not samples:
                if fallback_ground_ref_m is None:
                    _record_dense_linesearch_metrics(groundRequiredLazyFallbackCount=1)
                    continue
                ground_required = float(fallback_ground_ref_m)
            else:
                ground_required = max(samples)
            cache[signature] = float(ground_required)
            _put_and_finish_owned_process_cache_signature(signature, ground_required)
            _record_dense_linesearch_metrics(groundRequiredPrecomputeCount=1)
            for wp, line_coords, _ls, _coord in pending_waiters.get(signature, []):
                wp[_GROUND_REQUIRED_CACHE_KEY] = float(ground_required)
                wp[_GROUND_REQUIRED_SIGNATURE_KEY] = signature
                wp[_GROUND_REQUIRED_COORDS_ID_KEY] = id(line_coords)
                wp[_GROUND_REQUIRED_COORDS_COUNT_KEY] = len(line_coords)
        _record_dense_linesearch_metrics(
            groundRequiredResultAssignMs=(time.perf_counter() - result_assign_started_at) * 1000.0,
        )
        _record_owner_compute_elapsed()
        _finish_owned_process_cache_signatures()
        _wait_for_inflight_entries()
    finally:
        _record_owner_compute_elapsed()
        _finish_owned_process_cache_signatures()
        _record_dense_linesearch_metrics(
            groundRequiredComputeMs=(time.perf_counter() - started_at) * 1000.0,
        )


def _apply_segment_altitude_to_search_waypoints(
    wps: list[OrderedDict],
    *,
    altitude_offset_m: float,
    fallback_ground_ref_m: float | None = None,
    cruise_speed_mps: float = 50.0,
    climb_rate_mps: float = UAV_CLIMB_RATE_MPS,
    prev_altitude_m: float | None = None,
) -> None:
    """Set flight altitude from photographed terrain + UAV altitude layer."""
    started_at = time.perf_counter()
    offset_m = float(altitude_offset_m)

    try:
        for wp in wps or []:
            if not isinstance(wp, dict):
                continue
            coord = wp.get("coordinate") or {}
            if not isinstance(coord, dict):
                continue
            cur_lat = _safe_float_or_none(coord.get("latitude"))
            cur_lon = _safe_float_or_none(coord.get("longitude"))

            if cur_lat is None or cur_lon is None:
                continue
            ground_required = _waypoint_required_ground_m(
                wp,
                coord,
                fallback_ground_ref_m=fallback_ground_ref_m,
            )
            if ground_required is None:
                continue

            final_alt = int(round(float(ground_required) + offset_m))
            wp["coordinate"] = OrderedDict([
                ("latitude", round(cur_lat, 6)),
                ("longitude", round(cur_lon, 6)),
                ("altitude", final_alt),
            ])
    finally:
        elapsed_ms = (time.perf_counter() - started_at) * 1000.0
        _record_dense_linesearch_metrics(
            altitudeGuardMs=elapsed_ms,
            altitudeApplyMs=elapsed_ms,
        )


def _enforce_filming_target_altitude_floor_inplace(
    wps: list[OrderedDict],
    *,
    clearance_m: float = FILMING_TARGET_ALTITUDE_FLOOR_CLEARANCE_M,
) -> None:
    def _floor_signature(wp: dict, coord: dict) -> tuple | None:
        ground_signature = wp.get(_GROUND_REQUIRED_SIGNATURE_KEY)
        ground_required = _safe_float_or_none(wp.get(_GROUND_REQUIRED_CACHE_KEY))
        altitude = _safe_float_or_none(coord.get("altitude"))
        if ground_signature is None or ground_required is None or altitude is None:
            return None
        return (
            ground_signature,
            round(max(float(clearance_m), 0.0), 3),
            round(float(ground_required), 3),
            int(round(float(altitude))),
        )

    started_at = time.perf_counter()
    try:
        for wp in wps or []:
            if not isinstance(wp, dict):
                continue
            coord = wp.get("coordinate") or {}
            if not isinstance(coord, dict):
                continue
            cur_alt = _safe_float_or_none(coord.get("altitude"))
            if cur_alt is None:
                continue
            before_signature = _floor_signature(wp, coord)
            if before_signature is not None and wp.get(_ALTITUDE_FLOOR_SIGNATURE_KEY) == before_signature:
                _record_dense_linesearch_metrics(altitudeFloorSkipCount=1)
                continue
            ground_required = _waypoint_required_ground_m(wp, coord)
            if ground_required is None:
                continue
            minimum_alt = int(math.ceil(float(ground_required) + max(float(clearance_m), 0.0)))
            if int(round(float(cur_alt))) >= minimum_alt:
                after_signature = _floor_signature(wp, coord)
                if after_signature is not None:
                    wp[_ALTITUDE_FLOOR_SIGNATURE_KEY] = after_signature
                continue
            lat = _safe_float_or_none(coord.get("latitude"))
            lon = _safe_float_or_none(coord.get("longitude"))
            if lat is None or lon is None:
                continue
            coord["latitude"] = round(float(lat), 6)
            coord["longitude"] = round(float(lon), 6)
            coord["altitude"] = int(minimum_alt)
            wp["coordinate"] = coord
            after_signature = _floor_signature(wp, coord)
            if after_signature is not None:
                wp[_ALTITUDE_FLOOR_SIGNATURE_KEY] = after_signature
    finally:
        elapsed_ms = (time.perf_counter() - started_at) * 1000.0
        _record_dense_linesearch_metrics(
            altitudeGuardMs=elapsed_ms,
            altitudeFloorMs=elapsed_ms,
        )


def _clear_ground_required_cache_inplace(wps: list[OrderedDict]) -> None:
    for wp in wps or []:
        if not isinstance(wp, dict):
            continue
        wp.pop(_GROUND_REQUIRED_CACHE_KEY, None)
        wp.pop(_GROUND_REQUIRED_SIGNATURE_KEY, None)
        wp.pop(_GROUND_REQUIRED_COORDS_ID_KEY, None)
        wp.pop(_GROUND_REQUIRED_COORDS_COUNT_KEY, None)
        wp.pop(_ALTITUDE_FLOOR_SIGNATURE_KEY, None)
        fp = wp.get("filmingProperty") if isinstance(wp.get("filmingProperty"), dict) else {}
        ls = fp.get("lineSearch") if isinstance(fp.get("lineSearch"), dict) else {}
        if isinstance(ls, dict):
            ls.pop(_GROUND_REQUIRED_LINE_SIGNATURE_KEY, None)
            ls.pop(_GROUND_REQUIRED_LINE_SIGNATURE_COORDS_ID_KEY, None)
            ls.pop(_GROUND_REQUIRED_LINE_SIGNATURE_COORDS_COUNT_KEY, None)


def _mark_line_search_coords_trusted_dem_altitudes(wps: list[dict]) -> int:
    marked_count = 0
    for wp in wps or []:
        if not isinstance(wp, dict):
            continue
        fp = wp.get("filmingProperty") if isinstance(wp.get("filmingProperty"), dict) else {}
        ls = fp.get("lineSearch") if isinstance(fp.get("lineSearch"), dict) else {}
        for coord in ls.get("coordinateList") or []:
            if not isinstance(coord, dict):
                continue
            if (
                _safe_float_or_none(coord.get("latitude")) is None
                or _safe_float_or_none(coord.get("longitude")) is None
                or _safe_float_or_none(coord.get("altitude")) is None
            ):
                continue
            if coord.get(_DEM_ALT_COORD_TRUSTED_KEY) is not True:
                marked_count += 1
            coord[_DEM_ALT_COORD_TRUSTED_KEY] = True
    if marked_count:
        _record_dense_linesearch_metrics(groundRequiredTrustedCoordMarkedCount=marked_count)
    return marked_count


def _has_filming_target_coords(waypoint: dict) -> bool:
    filming = waypoint.get("filmingProperty")
    if not isinstance(filming, dict) or not filming:
        return False
    orientation = filming.get("coordinateOrientation")
    if isinstance(orientation, dict) and isinstance(orientation.get("coordinate"), dict):
        return True
    line_search = filming.get("lineSearch")
    if isinstance(line_search, dict) and any(
        isinstance(coord, dict) for coord in (line_search.get("coordinateList") or [])
    ):
        return True
    area_search = filming.get("areaSearch")
    if isinstance(area_search, dict) and any(
        isinstance(coord, dict) for coord in (area_search.get("coordinateList") or [])
    ):
        return True
    return False


def _align_point_anchor_altitude_with_search_waypoints(
    wps: list[OrderedDict],
) -> None:
    if not wps:
        return

    next_line_altitude_by_idx: dict[int, int] = {}
    next_altitude: int | None = None
    for idx in range(len(wps) - 1, -1, -1):
        wp = wps[idx]
        coord = wp.get("coordinate") or {}
        if _has_line_search(wp):
            alt = _safe_float_or_none(coord.get("altitude"))
            if alt is not None:
                next_altitude = int(round(alt))
        if next_altitude is not None:
            next_line_altitude_by_idx[idx] = next_altitude

    for idx, wp in enumerate(wps):
        fp = wp.get("filmingProperty") or {}
        if int(fp.get("operationMode", OPMODE_NONE) or OPMODE_NONE) != OPMODE_POINT:
            continue
        if "coordinateOrientation" not in fp:
            continue
        coord = wp.get("coordinate") or {}
        if not isinstance(coord, dict) or "latitude" not in coord or "longitude" not in coord:
            continue
        target_altitude = next_line_altitude_by_idx.get(idx)
        if target_altitude is None:
            continue
        wp["coordinate"] = OrderedDict([
            ("latitude", round(float(coord.get("latitude", 0.0)), 6)),
            ("longitude", round(float(coord.get("longitude", 0.0)), 6)),
            ("altitude", int(target_altitude)),
        ])


def _enforce_waypoint_altitude_rate_limit_inplace(
    wps: list[OrderedDict],
    *,
    vertical_rate_mps: float = UAV_CLIMB_RATE_MPS,
    default_speed_mps: float = 50.0,
) -> None:
    """Raise earlier waypoints when needed so later climb targets are reachable.

    The altitude policy floor is photographed terrain + UAV layer.  The climb
    rule must not lower a later search waypoint below that floor; instead it
    pre-raises previous flight waypoints enough to satisfy the climb rate.
    """
    if not wps:
        return

    try:
        rate_mps = max(float(vertical_rate_mps), 0.1)
    except Exception:
        rate_mps = float(UAV_CLIMB_RATE_MPS)
    try:
        fallback_speed_mps = max(float(default_speed_mps), 1.0)
    except Exception:
        fallback_speed_mps = 50.0

    items: list[tuple[int, dict, float, float, float]] = []
    for idx, wp in enumerate(wps):
        if not isinstance(wp, dict):
            continue
        coord = wp.get("coordinate") or {}
        if not isinstance(coord, dict):
            continue

        cur_lat = _safe_float_or_none(coord.get("latitude"))
        cur_lon = _safe_float_or_none(coord.get("longitude"))
        cur_alt = _safe_float_or_none(coord.get("altitude"))
        if None in (cur_lat, cur_lon, cur_alt):
            continue
        items.append((idx, wp, float(cur_lat), float(cur_lon), float(cur_alt)))

    if len(items) < 2:
        return

    mutable_alts = [float(item[4]) for item in items]
    for pos in range(len(items) - 2, -1, -1):
        _, prev_wp, prev_lat, prev_lon, _ = items[pos]
        _, next_wp, next_lat, next_lon, _ = items[pos + 1]
        try:
            seg_dist_m = _dist_between_coords(
                {"latitude": prev_lat, "longitude": prev_lon},
                {"latitude": next_lat, "longitude": next_lon},
            )
        except Exception:
            seg_dist_m = 0.0
        if seg_dist_m <= 0.0:
            continue
        speed_raw = _safe_float_or_none(next_wp.get("speed"))
        speed_mps = max(float(speed_raw), 1.0) if speed_raw is not None else fallback_speed_mps
        allowed_delta_m = rate_mps * (float(seg_dist_m) / speed_mps)
        required_prev_alt = float(mutable_alts[pos + 1]) - float(allowed_delta_m)
        if float(mutable_alts[pos]) >= required_prev_alt:
            continue
        mutable_alts[pos] = float(required_prev_alt)
        prev_wp["coordinate"] = OrderedDict([
            ("latitude", round(float(prev_lat), 6)),
            ("longitude", round(float(prev_lon), 6)),
            ("altitude", int(math.ceil(required_prev_alt))),
        ])
        mutable_alts[pos] = float(int(math.ceil(required_prev_alt)))


def _sweep_d_values(d_min: float, d_max: float, spacing_m: float) -> list[float]:
    d_values: list[float] = []
    current = d_min
    # ensure we always include the last strip by overshooting slightly
    while current <= d_max + spacing_m * 0.25:
        d_values.append(current)
        current += spacing_m
    if not d_values:
        return [d_min, d_max]
    if len(d_values) == 1 and d_max > d_min:
        d_values.append(d_max)
    else:
        last = d_values[-1]
        if d_max - last > spacing_m * 0.25:
            d_values.append(d_max)
    return d_values


def _band_ranges(d_min: float, d_max: float, max_width_m: float) -> list[tuple[float, float]]:
    span = d_max - d_min
    if max_width_m <= 0.0 or span <= max_width_m:
        return [(d_min, d_max)]
    bands = int(math.ceil(span / max_width_m))
    band_width = span / bands
    ranges: list[tuple[float, float]] = []
    for idx in range(bands):
        start = d_min + band_width * idx
        end = d_min + band_width * (idx + 1)
        ranges.append((start, end))
    return ranges


def _coerce_anchor_llh(anchor_llh: object, fallback_llh: tuple[float, float]) -> tuple[float, float]:
    """Accept either (lat, lon) tuple/list or {'latitude','longitude'} dict."""
    lat_fallback, lon_fallback = fallback_llh
    if isinstance(anchor_llh, dict):
        try:
            lat = float(anchor_llh.get("latitude", lat_fallback))
        except Exception:
            lat = float(lat_fallback)
        try:
            lon = float(anchor_llh.get("longitude", lon_fallback))
        except Exception:
            lon = float(lon_fallback)
        return lat, lon
    if isinstance(anchor_llh, (list, tuple)) and len(anchor_llh) >= 2:
        try:
            lat = float(anchor_llh[0])
        except Exception:
            lat = float(lat_fallback)
        try:
            lon = float(anchor_llh[1])
        except Exception:
            lon = float(lon_fallback)
        return lat, lon
    return float(lat_fallback), float(lon_fallback)


def _dedup_xy_hits(hits: list[tuple[float, float]], eps_m: float = 0.01) -> list[tuple[float, float]]:
    unique: list[tuple[float, float]] = []
    eps2 = float(eps_m) * float(eps_m)
    for x, y in hits:
        duplicated = False
        for ux, uy in unique:
            if (x - ux) * (x - ux) + (y - uy) * (y - uy) <= eps2:
                duplicated = True
                break
        if not duplicated:
            unique.append((x, y))
    return unique


def _pick_sweep_endpoints(
    hits: list[tuple[float, float]],
    tx: float,
    ty: float,
) -> tuple[tuple[float, float], tuple[float, float]] | None:
    unique_hits = _dedup_xy_hits(hits)
    if len(unique_hits) < 2:
        return None
    unique_hits.sort(key=lambda p: tx * p[0] + ty * p[1])
    return unique_hits[0], unique_hits[-1]


def _fallback_poly_long_axis_sweep(poly_llh: list[tuple[float, float]]) -> list[dict]:
    if len(poly_llh) < 2:
        return []
    lat0, lon0 = poly_llh[0]
    pts_xy = [llh_to_xy(lat, lon, lat0, lon0) for lat, lon in poly_llh]
    pts_xy = _dedup_xy_hits(pts_xy, eps_m=0.1)
    if len(pts_xy) < 2:
        return []

    best_pair: tuple[tuple[float, float], tuple[float, float]] | None = None
    best_dist2 = -1.0
    for i in range(len(pts_xy)):
        for j in range(i + 1, len(pts_xy)):
            ax, ay = pts_xy[i]
            bx, by = pts_xy[j]
            dist2 = (bx - ax) * (bx - ax) + (by - ay) * (by - ay)
            if dist2 > best_dist2:
                best_dist2 = dist2
                best_pair = (pts_xy[i], pts_xy[j])
    if best_pair is None:
        return []

    points: list[tuple[float, float]] = []
    for x, y in best_pair:
        lat, lon = xy_to_llh(x, y, lat0, lon0)
        points.append((lat, lon))
    coords = _materialize_dem_coords(points)
    _record_dense_linesearch_metrics(sweepEndpointCount=len(coords))
    return coords


def _poly_sweeps_banded(
    poly_llh: list[tuple[float, float]],
    anchor_llh: object,
    bearing_deg: float,
    fov_deg: float,
    separation_m: float,
    max_width_m: float,
    spacing_scale: float = 1.0,
) -> tuple[list[list[dict]], list[tuple[float, float]]]:
    started_at = time.perf_counter()
    lat0, lon0 = poly_llh[0]
    poly_xy = [llh_to_xy(lat, lon, lat0, lon0) for lat, lon in poly_llh]
    anchor_lat, anchor_lon = _coerce_anchor_llh(anchor_llh, (lat0, lon0))
    _ = llh_to_xy(anchor_lat, anchor_lon, lat0, lon0)

    th = math.radians(bearing_deg)
    tx, ty = math.sin(th), math.cos(th)             # bearing 방향
    nx, ny = math.cos(th), -math.sin(th)            # bearing sweep 의 수직 노멀
    proj_norm = [nx * x + ny * y for x, y in poly_xy]
    proj_along = [tx * x + ty * y for x, y in poly_xy]
    d_min, d_max = min(proj_norm), max(proj_norm)
    a_min, a_max = min(proj_along), max(proj_along)

    spacing_m = _sweep_spacing_m(separation_m=separation_m, fov_deg=fov_deg, spacing_scale=spacing_scale)
    edges = [
        (
            a[0],
            a[1],
            b[0],
            b[1],
            nx * a[0] + ny * a[1],
            nx * b[0] + ny * b[1],
            b[0] - a[0],
            b[1] - a[1],
        )
        for a, b in ((poly_xy[i], poly_xy[(i + 1) % len(poly_xy)]) for i in range(len(poly_xy)))
    ]
    band_ranges_raw = _band_ranges(a_min, a_max, max_width_m)
    band_ranges: list[tuple[float, float]] = []
    band_coord_lists: list[list[dict]] = []
    band_point_lists: list[list[tuple[float, float]]] = []

    d_values = _sweep_d_values(d_min, d_max, spacing_m)
    def _points_for_band(band_min: float, band_max: float) -> list[tuple[float, float]]:
        coord_points: list[tuple[float, float]] = []
        for d in d_values:
            hits = []
            for ax, ay, _bx, _by, proj_a, proj_b, dx, dy in edges:
                f1, f2 = proj_a - d, proj_b - d
                if f1 * f2 <= 0 and f1 != f2:
                    t = f1 / (f1 - f2)
                    x = ax + t * dx
                    y = ay + t * dy
                    hits.append((x, y))
            picked = _pick_sweep_endpoints(hits, tx, ty)
            if picked is not None:
                p1, p2 = picked
                a1 = tx * p1[0] + ty * p1[1]
                a2 = tx * p2[0] + ty * p2[1]
                seg_min, seg_max = (a1, a2) if a1 <= a2 else (a2, a1)
                if seg_max < band_min or seg_min > band_max:
                    continue
                denom = a2 - a1
                if abs(denom) < 1e-6:
                    if band_min <= a1 <= band_max:
                        for x, y in (p1, p2):
                            lat, lon = xy_to_llh(x, y, lat0, lon0)
                            coord_points.append((lat, lon))
                    continue
                clip_min = max(seg_min, band_min)
                clip_max = min(seg_max, band_max)
                t0 = (clip_min - a1) / denom
                t1 = (clip_max - a1) / denom
                t_low, t_high = (t0, t1) if t0 <= t1 else (t1, t0)
                t_low = max(0.0, min(1.0, t_low))
                t_high = max(0.0, min(1.0, t_high))
                if t_high <= t_low + 1e-9:
                    continue
                c1 = (p1[0] + (p2[0] - p1[0]) * t_low, p1[1] + (p2[1] - p1[1]) * t_low)
                c2 = (p1[0] + (p2[0] - p1[0]) * t_high, p1[1] + (p2[1] - p1[1]) * t_high)
                for x, y in (c1, c2):
                    lat, lon = xy_to_llh(x, y, lat0, lon0)
                    coord_points.append((lat, lon))
        return coord_points

    estimated_coord_count = len(d_values) * max(len(band_ranges_raw), 1) * 2
    workers = _linesearch_inner_parallel_workers(
        len(d_values) * max(len(band_ranges_raw), 1),
        coordinate_count=estimated_coord_count,
    )
    if workers > 1 and len(band_ranges_raw) > 1:
        _record_dense_linesearch_metrics(innerParallelWorkers=workers, innerParallelBatches=len(band_ranges_raw))
        with concurrent.futures.ThreadPoolExecutor(
            max_workers=min(workers, len(band_ranges_raw)),
            thread_name_prefix="LineSearchBand",
        ) as executor:
            futures = [
                executor.submit(_points_for_band, band_min, band_max)
                for band_min, band_max in band_ranges_raw
            ]
            for (band_min, band_max), future in zip(band_ranges_raw, futures):
                coord_points = future.result()
                if coord_points:
                    band_point_lists.append(coord_points)
                    band_ranges.append((band_min, band_max))
    else:
        for band_min, band_max in band_ranges_raw:
            coord_points = _points_for_band(band_min, band_max)
            if coord_points:
                band_point_lists.append(coord_points)
                band_ranges.append((band_min, band_max))
    band_coord_lists = _materialize_dem_coord_groups(band_point_lists)
    total_coords = sum(len(coords) for coords in band_coord_lists)
    _record_dense_linesearch_metrics(
        sweepStripCount=len(d_values) * max(len(band_ranges_raw), 1),
        sweepEndpointCount=total_coords,
        generateLineSearchMs=(time.perf_counter() - started_at) * 1000.0,
    )
    return band_coord_lists, band_ranges

def _poly_sweeps_general(
    poly_llh: list[tuple[float,float]],
    anchor_llh: object,
    bearing_deg: float,
    fov_deg: float,
    separation_m: float,
    spacing_scale: float = 1.0,
) -> list[dict]:
    """
    ▸ Convex polygon LLH → bearing 과 평행한 띠 스윕 → lineSearch.coordinateList
    ▸ 리턴: [{"latitude":…, "longitude":…, "altitude":5}, …]  (짝수/홀수=한 라인)
    """
    started_at = time.perf_counter()
    lat0, lon0 = poly_llh[0]
    poly_xy = [llh_to_xy(lat, lon, lat0, lon0) for lat, lon in poly_llh]
    anchor_lat, anchor_lon = _coerce_anchor_llh(anchor_llh, (lat0, lon0))
    _ = llh_to_xy(anchor_lat, anchor_lon, lat0, lon0)

    th = math.radians(bearing_deg)
    tx, ty = math.sin(th), math.cos(th)
    nx, ny =  math.cos(th), -math.sin(th)            # bearing sweep 의 수직 노멀
    proj = [nx*x + ny*y for x, y in poly_xy]
    d_min, d_max = min(proj), max(proj)

    spacing_m = _sweep_spacing_m(separation_m=separation_m, fov_deg=fov_deg, spacing_scale=spacing_scale)
    d_values = _sweep_d_values(d_min, d_max, spacing_m)

    # 다각형 edge 리스트
    edges = [
        (
            a[0],
            a[1],
            b[0],
            b[1],
            nx * a[0] + ny * a[1],
            nx * b[0] + ny * b[1],
            b[0] - a[0],
            b[1] - a[1],
        )
        for a, b in ((poly_xy[i], poly_xy[(i+1)%len(poly_xy)]) for i in range(len(poly_xy)))
    ]
    coord_points: list[tuple[float, float]] = []

    def _points_for_d_values(values: list[float]) -> list[tuple[float, float]]:
        out: list[tuple[float, float]] = []
        for d in values:
            # 직선: n·x = d  ↔  (−ny, nx) 방향 unit 벡터
            hits = []
            for ax, ay, _bx, _by, proj_a, proj_b, dx, dy in edges:
                f1, f2 = proj_a - d, proj_b - d
                if f1*f2 <= 0 and f1 != f2:
                    t = f1/(f1-f2)
                    x = ax + t*dx;   y = ay + t*dy
                    hits.append((x,y))
            picked = _pick_sweep_endpoints(hits, tx, ty)
            if picked is not None:
                # 스윕 선은 교차점 두 개를 연결
                p1, p2 = picked
                for x,y in (p1,p2):
                    lat, lon = xy_to_llh(x,y,lat0,lon0)
                    out.append((lat, lon))
        return out

    estimated_coord_count = len(d_values) * max(_interp_points_hint(SWEEP_LINE_INTERP_POINTS) or 2, 2)
    workers = _linesearch_inner_parallel_workers(len(d_values), coordinate_count=estimated_coord_count)
    if workers > 1:
        chunks = _chunk_sequence(d_values, workers)
        _record_dense_linesearch_metrics(innerParallelWorkers=workers, innerParallelBatches=len(chunks))
        with concurrent.futures.ThreadPoolExecutor(
            max_workers=min(workers, len(chunks)),
            thread_name_prefix="LineSearchSweep",
        ) as executor:
            futures = [executor.submit(_points_for_d_values, chunk) for chunk in chunks]
            for future in futures:
                coord_points.extend(future.result())
    else:
        coord_points.extend(_points_for_d_values(d_values))
    coord_list = _materialize_dem_coords(coord_points)
    _record_dense_linesearch_metrics(
        sweepStripCount=len(d_values),
        sweepEndpointCount=len(coord_list),
        generateLineSearchMs=(time.perf_counter() - started_at) * 1000.0,
    )
    return coord_list

def _mk_filming(operation_mode: int = OPMODE_NONE,
                fov: float = FOV_DEG,
                sensor: int = SENSOR_NONE,
                line_search: OrderedDict | None = None,
                gimbal_pitch: float | None = None,
                gimbal_yaw: float | None = None) -> OrderedDict:
    """
    filmingProperty 블록 생성 헬퍼

    • OPMODE_LINE  → lineSearch 필드 삽입
    • OPMODE_HOLD → aircraftFixed  블록 삽입
                  ↳ gimbalPitch / gimbalYaw 포함
    """
    fp = OrderedDict([
        ("fieldOfView",   fov),
        ("sensorType",    sensor),
        ("operationMode", operation_mode),
    ])

    # ① 선형 탐색(lineSearch)
    if operation_mode == OPMODE_LINE and line_search is not None:
        fp["lineSearch"] = line_search
        coords = line_search.get("coordinateList") if isinstance(line_search, dict) else None
        if isinstance(coords, list):
            coord_cap = _max_linesearch_coords_per_waypoint()
            _record_dense_linesearch_metrics(
                lineSearchCoordCount=len(coords),
                maxLineSearchCoords=len(coords),
                lineSearchCoordinateCap=coord_cap,
                lineSearchCoordinateCapExceededCount=1
                if coord_cap > 0 and len(coords) > int(coord_cap)
                else 0,
            )

    # ② 고정 촬영(aircraftFixed + 짐벌 각도)
    if operation_mode == OPMODE_HOLD:
        # 기본값:  직하방 -90°, Yaw 0°
        gimbal_pitch = -90.0 if gimbal_pitch is None else gimbal_pitch
        gimbal_yaw   =   0.0 if gimbal_yaw   is None else gimbal_yaw
        fp["aircraftFixed"] = OrderedDict([
            ("gimbalPitch", gimbal_pitch),
            ("gimbalYaw",   gimbal_yaw),
        ])
    return fp


def _scale_output_fov_inplace(wps: list[OrderedDict], *, scale: float) -> None:
    try:
        effective_scale = float(scale)
    except Exception:
        return
    if effective_scale <= 0.0 or abs(effective_scale - 1.0) <= 1e-9:
        return
    for wp in wps:
        fp = wp.get("filmingProperty")
        if not isinstance(fp, dict):
            continue
        try:
            base_fov = float(fp.get("fieldOfView", FOV_DEG) or FOV_DEG)
        except Exception:
            continue
        fp["fieldOfView"] = round(base_fov * effective_scale, 6)


def _has_line_search(wp: dict) -> bool:
    fp = wp.get("filmingProperty") or {}
    if int(fp.get("operationMode", OPMODE_NONE)) != OPMODE_LINE:
        return False
    line_search = fp.get("lineSearch") or {}
    coords = line_search.get("coordinateList") or []
    return bool(coords)


def _angle_between(v1: tuple[float, float], v2: tuple[float, float]) -> float:
    x1, y1 = v1
    x2, y2 = v2
    n1 = math.hypot(x1, y1)
    n2 = math.hypot(x2, y2)
    if n1 <= 1e-6 or n2 <= 1e-6:
        return 0.0
    cos_th = max(-1.0, min(1.0, (x1 * x2 + y1 * y2) / (n1 * n2)))
    return math.degrees(math.acos(cos_th))


def _vector_between_coords(coord_from: dict, coord_to: dict) -> tuple[float, float]:
    lat_a = float(coord_from.get("latitude", 0.0))
    lon_a = float(coord_from.get("longitude", 0.0))
    lat_b = float(coord_to.get("latitude", lat_a))
    lon_b = float(coord_to.get("longitude", lon_a))
    return llh_to_xy(lat_b, lon_b, lat_a, lon_a)


def _heading_rad_between_coords(coord_from: dict, coord_to: dict) -> float:
    dx, dy = _vector_between_coords(coord_from, coord_to)
    return math.atan2(dy, dx)


def _dist_between_coords(coord_from: dict, coord_to: dict) -> float:
    dx, dy = _vector_between_coords(coord_from, coord_to)
    return math.hypot(dx, dy)


def _copy_coord_dict(coord: dict) -> dict:
    if isinstance(coord, dict):
        return dict(coord)
    return deepcopy(coord)


def _copy_coord_list(coords: Iterable[dict] | None) -> list[dict]:
    return [_copy_coord_dict(coord) for coord in (coords or [])]


def _coord_midpoint(coord_list: list[dict]) -> dict | None:
    if not coord_list:
        return None
    if len(coord_list) == 1:
        return _copy_coord_dict(coord_list[0])
    first = coord_list[0]
    last = coord_list[-1]
    return {
        "latitude": (float(first.get("latitude", 0.0)) + float(last.get("latitude", 0.0))) * 0.5,
        "longitude": (float(first.get("longitude", 0.0)) + float(last.get("longitude", 0.0))) * 0.5,
        "altitude": int(round((float(first.get("altitude", 0.0)) + float(last.get("altitude", 0.0))) * 0.5)),
    }


def _area_wp_coord_from_sweep(
    coords: list[dict],
    *,
    bearing_deg: float | None,
    offset_m: float,
    altitude_fn: Callable[[float, float], int],
    reference_coord: dict | None = None,
    offset_sign: int | None = None,
) -> OrderedDict | None:
    center = _coord_midpoint(coords)
    if not isinstance(center, dict):
        return None

    center_lat = float(center.get("latitude", 0.0))
    center_lon = float(center.get("longitude", 0.0))
    try:
        effective_offset_m = max(float(offset_m), 0.0)
    except Exception:
        effective_offset_m = 0.0

    shifted_lat = center_lat
    shifted_lon = center_lon
    try:
        if bearing_deg is not None:
            th = math.radians(float(bearing_deg))
            neg_lat, neg_lon = xy_to_llh(
                -math.cos(th) * effective_offset_m,
                math.sin(th) * effective_offset_m,
                center_lat,
                center_lon,
            )
            pos_lat, pos_lon = xy_to_llh(
                math.cos(th) * effective_offset_m,
                -math.sin(th) * effective_offset_m,
                center_lat,
                center_lon,
            )
            forced_sign = 0
            if offset_sign is not None:
                try:
                    forced_sign = 1 if int(offset_sign) >= 0 else -1
                except Exception:
                    forced_sign = 0
            if forced_sign > 0:
                shifted_lat, shifted_lon = pos_lat, pos_lon
            elif forced_sign < 0:
                shifted_lat, shifted_lon = neg_lat, neg_lon
            if (
                forced_sign == 0
                and
                isinstance(reference_coord, dict)
                and "latitude" in reference_coord
                and "longitude" in reference_coord
            ):
                neg_coord = {"latitude": neg_lat, "longitude": neg_lon}
                pos_coord = {"latitude": pos_lat, "longitude": pos_lon}
                if _dist_between_coords(reference_coord, pos_coord) + 1e-6 < _dist_between_coords(reference_coord, neg_coord):
                    shifted_lat, shifted_lon = pos_lat, pos_lon
                else:
                    shifted_lat, shifted_lon = neg_lat, neg_lon
            elif forced_sign == 0:
                shifted_lat, shifted_lon = neg_lat, neg_lon
    except Exception:
        shifted_lat = center_lat
        shifted_lon = center_lon

    return OrderedDict([
        ("latitude", round(float(shifted_lat), 6)),
        ("longitude", round(float(shifted_lon), 6)),
        ("altitude", altitude_fn(float(shifted_lat), float(shifted_lon))),
    ])


def _line_wp_xy_from_sweep(
    sweep_xy: tuple[tuple[float, float], tuple[float, float]],
    *,
    offset_m: float,
) -> tuple[float, float]:
    (x1, y1), (x2, y2) = sweep_xy
    mid_x = (float(x1) + float(x2)) * 0.5
    mid_y = (float(y1) + float(y2)) * 0.5
    dx = float(x2) - float(x1)
    dy = float(y2) - float(y1)
    norm = math.hypot(dx, dy)
    if norm < 1e-9:
        return (mid_x, mid_y)
    try:
        effective_offset_m = max(float(offset_m), 0.0)
    except Exception:
        effective_offset_m = 0.0
    ux = dy / norm
    uy = -dx / norm
    return (mid_x + (ux * effective_offset_m), mid_y + (uy * effective_offset_m))


def _corridor_first_anchor_quality_sep_m(
    planner: object,
    *,
    route_offset_m: float,
) -> float:
    sweeps = getattr(planner, "sweeps", None)
    if not sweeps:
        return 0.0
    try:
        first_sweep = sweeps[0]
        anchor_xy = _line_wp_xy_from_sweep(first_sweep, offset_m=float(route_offset_m))
        return max(
            math.hypot(float(anchor_xy[0]) - float(first_sweep[0][0]), float(anchor_xy[1]) - float(first_sweep[0][1])),
            math.hypot(float(anchor_xy[0]) - float(first_sweep[1][0]), float(anchor_xy[1]) - float(first_sweep[1][1])),
        )
    except Exception:
        return 0.0


def _infer_area_offset_sign_from_sweep(
    coords: list[dict],
    *,
    bearing_deg: float | None,
    offset_m: float,
    reference_coord: dict | None = None,
) -> int:
    anchor = _area_wp_coord_from_sweep(
        coords,
        bearing_deg=bearing_deg,
        offset_m=offset_m,
        altitude_fn=lambda lat, lon: 0,
        reference_coord=reference_coord,
    )
    if not anchor or bearing_deg is None or len(coords) < 2:
        return -1
    center = _coord_midpoint(coords)
    if not center:
        return -1
    th = math.radians(float(bearing_deg))
    nx, ny = math.cos(th), -math.sin(th)
    anchor_x, anchor_y = llh_to_xy(
        float(anchor.get("latitude", 0.0)),
        float(anchor.get("longitude", 0.0)),
        float(center.get("latitude", 0.0)),
        float(center.get("longitude", 0.0)),
    )
    signed = (anchor_x * nx) + (anchor_y * ny)
    return 1 if signed >= 0.0 else -1


def _wp_effective_start_coord(wp: dict) -> dict:
    fp = wp.get("filmingProperty") or {}
    ls = fp.get("lineSearch") or {}
    coords = ls.get("coordinateList") or []
    if coords:
        return dict(coords[0])
    orient = fp.get("coordinateOrientation") or {}
    orient_coord = orient.get("coordinate")
    if isinstance(orient_coord, dict) and "latitude" in orient_coord and "longitude" in orient_coord:
        return dict(orient_coord)
    return dict(wp.get("coordinate") or {})


def _wp_effective_end_coord(wp: dict) -> dict:
    fp = wp.get("filmingProperty") or {}
    ls = fp.get("lineSearch") or {}
    coords = ls.get("coordinateList") or []
    if coords:
        return dict(coords[-1])
    return dict(wp.get("coordinate") or {})


def _orient_sweep_items(
    sweep_items: list[dict],
    reference_coord: dict | None,
) -> list[dict]:
    if len(sweep_items) <= 1:
        return sweep_items

    items = []
    for item in sweep_items:
        copied = dict(item)
        copied["coords"] = list(item.get("coords") or [])
        items.append(copied)

    if reference_coord and "latitude" in reference_coord and "longitude" in reference_coord:
        def _best_item_start_dist(item: dict) -> float:
            coords = item.get("coords") or []
            if len(coords) >= 2:
                return min(
                    _dist_between_coords(reference_coord, coords[0]),
                    _dist_between_coords(reference_coord, coords[-1]),
                )
            anchor = item.get("coord") or _coord_midpoint(coords) or {}
            return _dist_between_coords(reference_coord, anchor)

        if _best_item_start_dist(items[-1]) + 1e-6 < _best_item_start_dist(items[0]):
            items.reverse()

        prev_target = reference_coord
        for item in items:
            coords = item.get("coords") or []
            if len(coords) >= 2:
                first_coord = coords[0]
                last_coord = coords[-1]
                if _dist_between_coords(prev_target, last_coord) + 1e-6 < _dist_between_coords(prev_target, first_coord):
                    coords.reverse()
                prev_target = coords[-1]
                item["coords"] = coords

    return items


def _apply_oriented_sweep_items_to_wps(
    wps: list[OrderedDict],
    sweep_items: list[dict],
) -> None:
    if not wps or not sweep_items:
        return

    ordered_wps = [item.get("wp") for item in sweep_items if isinstance(item.get("wp"), dict)]
    if not ordered_wps:
        return

    ordered_ids = {id(wp) for wp in ordered_wps}
    remainder = [wp for wp in wps if id(wp) not in ordered_ids]
    wps[:] = ordered_wps + remainder

    index_by_id = {id(wp): idx for idx, wp in enumerate(wps)}
    for item in sweep_items:
        wp = item.get("wp")
        if not isinstance(wp, dict):
            continue
        item["idx"] = index_by_id.get(id(wp), item.get("idx", 0))
        item["coord"] = wp.get("coordinate") or item.get("coord") or {}
        item["fp"] = wp.get("filmingProperty") or item.get("fp") or OrderedDict()


def _reorder_line_search_subset_inplace(
    wps: list[OrderedDict],
    *,
    reference_coord: dict | None,
) -> None:
    if not wps:
        return

    line_positions = [idx for idx, wp in enumerate(wps) if _has_line_search(wp)]
    if len(line_positions) <= 1:
        return

    first_line_pos = line_positions[0]
    prefix = wps[:first_line_pos]
    line_wps = [wps[idx] for idx in line_positions]
    suffix_positions = set(line_positions)
    suffix = [wp for idx, wp in enumerate(wps[first_line_pos:]) if (idx + first_line_pos) not in suffix_positions]

    sweep_items: list[dict] = []
    for idx, wp in enumerate(line_wps):
        fp = wp.get("filmingProperty") or OrderedDict()
        ls = fp.get("lineSearch") or {}
        coords = list(ls.get("coordinateList") or [])
        sweep_items.append({
            "idx": idx,
            "wp": wp,
            "fp": fp,
            "coord": wp.get("coordinate") or {},
            "coords": coords,
            "search_speed": ls.get("searchSpeed"),
            "fov": fp.get("fieldOfView", FOV_DEG),
            "interp_points": _interp_points_hint(ls.get("interpolationPoints")),
            "sweep_idx": wp.get("_sweepIdx"),
        })

    sweep_items = _orient_sweep_items(sweep_items, reference_coord)
    ordered_line_wps: list[OrderedDict] = []
    for item in sweep_items:
        wp = item.get("wp")
        if not isinstance(wp, dict):
            continue
        fp = wp.get("filmingProperty") or OrderedDict()
        ls = fp.get("lineSearch") or OrderedDict()
        coords = list(item.get("coords") or [])
        if coords:
            ls["coordinateList"] = coords
            fp["lineSearch"] = ls
            wp["filmingProperty"] = fp
        ordered_line_wps.append(wp)

    wps[:] = prefix + ordered_line_wps + suffix


def _split_record_groups_by_spacing(
    records: list[dict],
    groups: list[list[int]],
    *,
    anchor_coord: dict | None,
    spacing_m: float,
    max_coords_per_group: int | None = None,
    merge_short_tail: bool = False,
) -> list[list[int]]:
    try:
        max_spacing_m = max(float(spacing_m), 1.0)
    except Exception:
        max_spacing_m = _line_route_wp_spacing_m()
    try:
        coord_cap = int(max_coords_per_group or 0)
    except Exception:
        coord_cap = 0
    if not groups or max_spacing_m <= 0.0:
        return groups

    def _record_coord_count(pos: int) -> int:
        try:
            return max(0, len(records[pos].get("coords") or []))
        except Exception:
            return 0

    def _group_coord_count(group: list[int]) -> int:
        return sum(_record_coord_count(pos) for pos in group)

    out: list[list[int]] = []
    group_anchor = anchor_coord if isinstance(anchor_coord, dict) else None
    for group in groups:
        if len(group) <= 1:
            out.append(group)
            if group:
                anchor_candidate = records[group[-1]].get("coord") or records[group[-1]]["wp"].get("coordinate") or {}
                if isinstance(anchor_candidate, dict) and anchor_candidate:
                    group_anchor = anchor_candidate
            continue

        subgroup: list[int] = []
        progressed = 0.0
        subgroup_coords = 0
        prev_coord = group_anchor
        for pos in group:
            curr_coord = records[pos].get("coord") or records[pos]["wp"].get("coordinate") or {}
            step = _dist_between_coords(prev_coord, curr_coord) if prev_coord else 0.0
            projected = progressed + step
            pos_coords = _record_coord_count(pos)
            projected_coords = subgroup_coords + pos_coords
            split_for_spacing = projected > max_spacing_m and progressed >= 1.0
            split_for_coords = bool(coord_cap > 0 and projected_coords > coord_cap)
            if subgroup and (split_for_spacing or split_for_coords):
                out.append(subgroup)
                subgroup = [pos]
                progressed = step
                subgroup_coords = pos_coords
            else:
                subgroup.append(pos)
                progressed = projected
                subgroup_coords = projected_coords
            prev_coord = curr_coord
        if subgroup:
            out.append(subgroup)
            anchor_candidate = records[subgroup[-1]].get("coord") or records[subgroup[-1]]["wp"].get("coordinate") or {}
            if isinstance(anchor_candidate, dict) and anchor_candidate:
                group_anchor = anchor_candidate
    if merge_short_tail:
        while len(out) >= 2:
            prev_anchor = records[out[-2][-1]].get("coord") or records[out[-2][-1]]["wp"].get("coordinate") or {}
            tail_anchor = records[out[-1][-1]].get("coord") or records[out[-1][-1]]["wp"].get("coordinate") or {}
            if not (isinstance(prev_anchor, dict) and prev_anchor and isinstance(tail_anchor, dict) and tail_anchor):
                break
            tail_gap_m = _dist_between_coords(prev_anchor, tail_anchor)
            if tail_gap_m + 1e-6 >= max_spacing_m:
                break
            if coord_cap > 0 and (_group_coord_count(out[-2]) + _group_coord_count(out[-1])) > coord_cap:
                break
            out[-1] = out[-2] + out[-1]
            out.pop(-2)
    return out


def _compute_group_sweep_span_counts(
    records: list[dict],
    groups: list[list[int]],
    *,
    initial_prev_sweep_idx: int | None,
    sweep_step: int,
) -> list[int | None]:
    counts: list[int | None] = []
    prev_sweep_idx = initial_prev_sweep_idx
    for g_idx, group in enumerate(groups):
        if not group:
            counts.append(None)
            continue
        rep_sweep_idx = records[group[-1]].get("sweep_idx")
        if rep_sweep_idx is None or prev_sweep_idx is None:
            counts.append(None)
            if rep_sweep_idx is not None:
                prev_sweep_idx = int(rep_sweep_idx)
            continue
        if g_idx > 0:
            start_sweep_idx = int(prev_sweep_idx) + int(sweep_step)
        else:
            start_sweep_idx = int(prev_sweep_idx)
        rep_sweep_idx = int(rep_sweep_idx)
        counts.append(abs(rep_sweep_idx - start_sweep_idx) + 1)
        prev_sweep_idx = rep_sweep_idx
    return counts


def _rebalance_groups_by_sweep_span(
    records: list[dict],
    groups: list[list[int]],
    *,
    initial_prev_sweep_idx: int | None,
    sweep_step: int,
    min_span: int = 4,
    tiny_span_threshold: int = 2,
) -> list[list[int]]:
    if len(groups) < 2 or initial_prev_sweep_idx is None:
        return groups

    out = [list(group) for group in groups if group]
    if len(out) < 2:
        return out

    while True:
        counts = _compute_group_sweep_span_counts(
            records,
            out,
            initial_prev_sweep_idx=initial_prev_sweep_idx,
            sweep_step=sweep_step,
        )
        changed = False
        for idx in range(1, len(out)):
            prev_count = counts[idx - 1]
            curr_count = counts[idx]
            if prev_count is None or curr_count is None:
                continue
            if curr_count > int(tiny_span_threshold):
                continue
            if len(out[idx - 1]) <= 1:
                continue

            candidate = [list(group) for group in out]
            candidate[idx].insert(0, candidate[idx - 1].pop())
            candidate_counts = _compute_group_sweep_span_counts(
                records,
                candidate,
                initial_prev_sweep_idx=initial_prev_sweep_idx,
                sweep_step=sweep_step,
            )
            cand_prev = candidate_counts[idx - 1]
            cand_curr = candidate_counts[idx]
            if cand_prev is None or cand_curr is None:
                continue
            if cand_prev < int(min_span):
                continue
            if cand_curr <= curr_count:
                continue
            if max(cand_prev, cand_curr) >= max(prev_count, curr_count):
                continue

            out = candidate
            changed = True
            break
        if not changed:
            return out


def _line_route_wp_spacing_m() -> float:
    try:
        fallback_spacing = float(SWEEP_ROUTE_WP_SPACING_M)
    except Exception:
        fallback_spacing = 2000.0
    try:
        base_spacing = _runtime_float_setting("uav_wp_interval_m", fallback_spacing)
    except Exception:
        base_spacing = fallback_spacing
    return max(base_spacing, 1.0)


def _area_route_wp_spacing_m() -> float:
    try:
        fallback_spacing = float(AREA_SWEEP_ROUTE_WP_SPACING_M)
    except Exception:
        fallback_spacing = 2000.0
    try:
        base_spacing = _runtime_float_setting("uav_wp_interval_m", fallback_spacing)
    except Exception:
        base_spacing = fallback_spacing
    return max(base_spacing, 1.0)


def _sweep_spacing_margin_for_density() -> float:
    try:
        margin = float(SWEEP_SPACING_MARGIN)
    except Exception:
        margin = 1.0
    if margin <= 0.0:
        margin = 1.0
    return max(margin, 1e-6)


def _line_raw_spacing_scale() -> float:
    try:
        density = float(LINE_SWEEP_DENSITY_SCALE)
    except Exception:
        density = 1.0
    if density <= 0.0:
        density = 1.0
    sep_safety = _runtime_fov_db_sep_safety_factor(_runtime_settings_payload())
    if sep_safety <= 0.0:
        sep_safety = 1.0
    return max(1.0 / (density * sep_safety * _sweep_spacing_margin_for_density()), 1e-6)


def _area_raw_spacing_scale() -> float:
    try:
        fov_scale = float(AREA_OUTPUT_FOV_SCALE)
    except Exception:
        fov_scale = 1.0
    if _runtime_manual_fov_sync_active(_runtime_settings_payload()):
        fov_scale = 1.0
    if fov_scale <= 0.0:
        fov_scale = 1.0
    try:
        density = float(AREA_SWEEP_DENSITY_SCALE)
    except Exception:
        density = 1.0
    if density <= 0.0:
        density = 1.0
    return max(fov_scale / (density * _sweep_spacing_margin_for_density()), 1e-6)


def _is_required_wp(wp: dict) -> bool:
    if wp.get("_area_dubins_link"):
        return True
    if wp.get("waypointPassType") == PASS_LOITER:
        return True
    fp = wp.get("filmingProperty") or {}
    if not fp:
        return False
    return int(fp.get("operationMode", OPMODE_NONE)) != OPMODE_NONE


def _simplify_area_wps(
    wps: list[OrderedDict],
    *,
    angle_deg: float,
    min_spacing_m: float,
) -> list[OrderedDict]:
    if len(wps) <= 2:
        return wps
    keep = [0]
    last_keep = 0
    for idx in range(1, len(wps) - 1):
        if _is_required_wp(wps[idx]):
            keep.append(idx)
            last_keep = idx
            continue
        prev_coord = wps[last_keep].get("coordinate") or {}
        curr_coord = wps[idx].get("coordinate") or {}
        next_coord = wps[idx + 1].get("coordinate") or {}
        dist = _dist_between_coords(prev_coord, curr_coord)
        angle = _angle_between(
            _vector_between_coords(prev_coord, curr_coord),
            _vector_between_coords(curr_coord, next_coord),
        )
        if dist >= min_spacing_m or angle >= angle_deg:
            keep.append(idx)
            last_keep = idx
    keep.append(len(wps) - 1)
    keep = sorted(set(keep))
    return [wps[i] for i in keep]


def _reposition_area_wps_by_sep(
    wps: list[OrderedDict],
    *,
    offset_m: float,
    altitude_fn: Callable[[float, float], int],
    bearing_deg: float | None = None,
    reference_coord: dict | None = None,
    offset_sign: int | None = None,
) -> None:
    if not wps:
        return

    def _align_route_wps_to_endpoint_axis() -> None:
        route_positions = [idx for idx, wp in enumerate(wps) if _has_line_search(wp)]
        if len(route_positions) < 3:
            return

        first_coord = wps[route_positions[0]].get("coordinate") or {}
        last_coord = wps[route_positions[-1]].get("coordinate") or {}
        route_dx, route_dy = _vector_between_coords(first_coord, last_coord)
        route_norm = math.hypot(route_dx, route_dy)
        if route_norm < 1.0:
            return

        ux = route_dx / route_norm
        uy = route_dy / route_norm
        origin_lat = float(first_coord.get("latitude", 0.0))
        origin_lon = float(first_coord.get("longitude", 0.0))
        previous_progress = 0.0

        for pos in route_positions[1:-1]:
            curr_coord = wps[pos].get("coordinate") or {}
            curr_lat = float(curr_coord.get("latitude", origin_lat))
            curr_lon = float(curr_coord.get("longitude", origin_lon))
            curr_x, curr_y = llh_to_xy(curr_lat, curr_lon, origin_lat, origin_lon)
            progress_m = max(previous_progress, (curr_x * ux) + (curr_y * uy))
            progress_m = min(progress_m, route_norm)
            projected_lat, projected_lon = xy_to_llh(
                progress_m * ux,
                progress_m * uy,
                origin_lat,
                origin_lon,
            )
            altitude = curr_coord.get("altitude")
            if altitude is None:
                altitude = altitude_fn(projected_lat, projected_lon)
            wps[pos]["coordinate"] = OrderedDict([
                ("latitude", round(projected_lat, 6)),
                ("longitude", round(projected_lon, 6)),
                ("altitude", int(round(float(altitude)))),
            ])
            previous_progress = progress_m

    if bearing_deg is not None:
        try:
            effective_offset_m = max(float(offset_m), 0.0)
        except Exception:
            effective_offset_m = 0.0
        for wp in wps:
            fp = wp.get("filmingProperty") or {}
            ls = fp.get("lineSearch") or {}
            coords = ls.get("coordinateList") or []
            shifted = _area_wp_coord_from_sweep(
                coords,
                bearing_deg=bearing_deg,
                offset_m=effective_offset_m,
                altitude_fn=altitude_fn,
                reference_coord=reference_coord,
                offset_sign=offset_sign,
            )
            if shifted is not None:
                wp["coordinate"] = shifted
        _align_route_wps_to_endpoint_axis()
        return

    if len(wps) < 2:
        return

    centers: list[dict | None] = []
    for wp in wps:
        fp = wp.get("filmingProperty") or {}
        ls = fp.get("lineSearch") or {}
        coords = ls.get("coordinateList") or []
        centers.append(_coord_midpoint(coords) if coords else dict(wp.get("coordinate") or {}))

    first_center = centers[0] if isinstance(centers[0], dict) else None
    if not first_center:
        return

    sample_centers = [center for center in centers[: min(len(centers), 5)] if isinstance(center, dict)]
    if len(sample_centers) < 2:
        return

    dir_x = 0.0
    dir_y = 0.0
    for idx in range(len(sample_centers) - 1):
        vec_x, vec_y = _vector_between_coords(sample_centers[idx], sample_centers[idx + 1])
        seg_norm = math.hypot(vec_x, vec_y)
        if seg_norm < 1.0:
            continue
        dir_x += vec_x / seg_norm
        dir_y += vec_y / seg_norm

    norm = math.hypot(dir_x, dir_y)
    if norm < 1.0:
        vec_x, vec_y = _vector_between_coords(sample_centers[0], sample_centers[-1])
        norm = math.hypot(vec_x, vec_y)
        if norm < 1.0:
            return
        dir_x, dir_y = vec_x, vec_y

    ux = dir_x / norm
    uy = dir_y / norm
    origin_lat = float(first_center.get("latitude", 0.0))
    origin_lon = float(first_center.get("longitude", 0.0))
    effective_offset_m = max(float(offset_m), 0.0)
    sign = -1.0
    if offset_sign is not None:
        try:
            sign = 1.0 if int(offset_sign) >= 0 else -1.0
        except Exception:
            sign = -1.0
    offset_x = sign * ux * effective_offset_m
    offset_y = sign * uy * effective_offset_m

    for wp, center in zip(wps, centers):
        if not isinstance(center, dict):
            continue
        base_lat = float(center.get("latitude", 0.0))
        base_lon = float(center.get("longitude", 0.0))
        center_x, center_y = llh_to_xy(base_lat, base_lon, origin_lat, origin_lon)
        progress_m = (center_x * ux) + (center_y * uy)
        shifted_lat, shifted_lon = xy_to_llh(
            offset_x + (progress_m * ux),
            offset_y + (progress_m * uy),
            origin_lat,
            origin_lon,
        )
        wp["coordinate"] = OrderedDict([
            ("latitude", round(shifted_lat, 6)),
            ("longitude", round(shifted_lon, 6)),
            ("altitude", altitude_fn(shifted_lat, shifted_lon)),
        ])
    _align_route_wps_to_endpoint_axis()


def _collapse_area_wps_to_two_points(
    wps: list[OrderedDict],
    *,
    full_coords: list[dict] | None,
    search_speed: float | None,
    interp_points: int | None,
    altitude_fn: Callable[[float, float], int],
    bearing_deg: float | None = None,
    offset_m: float = 0.0,
    reference_coord: dict | None = None,
    offset_sign: int | None = None,
) -> list[OrderedDict]:
    if not wps:
        return wps

    area_route_spacing_m = _area_route_wp_spacing_m()

    line_positions = [idx for idx, wp in enumerate(wps) if _has_line_search(wp)]
    if not line_positions:
        return wps

    line_wps = [wps[idx] for idx in line_positions]
    first_line_wp = line_wps[0]
    first_coord = dict(wps[0].get("coordinate") or {})
    coord_cap = _max_linesearch_coords_per_waypoint()

    def _line_coord_count_at(pos: int) -> int:
        try:
            fp = wps[pos].get("filmingProperty") or {}
            ls = fp.get("lineSearch") or {}
            return max(0, len(ls.get("coordinateList") or []))
        except Exception:
            return 0

    def _group_coord_count(group: list[int]) -> int:
        return sum(_line_coord_count_at(pos) for pos in group)

    first_fp = first_line_wp.get("filmingProperty") or {}
    first_ls = first_fp.get("lineSearch") or {}
    first_ls_coords = _copy_coord_list(first_ls.get("coordinateList") or [])
    if not first_ls_coords:
        first_ls_coords = _copy_coord_list(full_coords or [])
    if not first_ls_coords:
        return wps

    sweep_start = dict(first_ls_coords[0])
    orientation_coord = _coord_with_dem_altitude(sweep_start)

    merged_coords = _copy_coord_list(full_coords or [])
    if not merged_coords:
        for wp in line_wps:
            coords = _copy_coord_list(((wp.get("filmingProperty") or {}).get("lineSearch") or {}).get("coordinateList") or [])
            if coords:
                merged_coords.extend(coords)
    if not merged_coords:
        return wps

    first_waypoint = OrderedDict(deepcopy(wps[0]))
    first_anchor_coord = _area_wp_coord_from_sweep(
        first_ls_coords,
        bearing_deg=bearing_deg,
        offset_m=offset_m,
        altitude_fn=altitude_fn,
        reference_coord=reference_coord,
        offset_sign=offset_sign,
    )
    if first_anchor_coord is None:
        first_anchor_coord = OrderedDict([
            ("latitude", round(float(first_coord.get("latitude", 0.0)), 6)),
            ("longitude", round(float(first_coord.get("longitude", 0.0)), 6)),
            ("altitude", altitude_fn(float(first_coord.get("latitude", 0.0)), float(first_coord.get("longitude", 0.0)))),
        ])
    first_waypoint["coordinate"] = first_anchor_coord
    first_waypoint["waypointPassType"] = PASS_FLYBY
    first_waypoint["filmingProperty"] = OrderedDict([
        ("fieldOfView", float(first_fp.get("fieldOfView", FOV_DEG) or FOV_DEG)),
        ("sensorType", int(first_fp.get("sensorType", SENSOR_EO_IR) or SENSOR_EO_IR)),
        ("operationMode", OPMODE_POINT),
        ("coordinateOrientation", OrderedDict([
            ("coordinate", orientation_coord),
        ])),
    ])

    groups: list[list[int]] = []
    current_group: list[int] = [line_positions[0]]
    progressed = 0.0
    current_group_coords = _line_coord_count_at(line_positions[0])
    prev_coord = wps[line_positions[0]].get("coordinate") or {}
    for pos in line_positions[1:]:
        curr_coord = wps[pos].get("coordinate") or {}
        step = _dist_between_coords(prev_coord, curr_coord)
        projected = progressed + step
        pos_coords = _line_coord_count_at(pos)
        projected_coords = current_group_coords + pos_coords
        split_for_spacing = projected > area_route_spacing_m and progressed >= 1.0
        split_for_coords = bool(coord_cap > 0 and projected_coords > coord_cap)
        if current_group and (split_for_spacing or split_for_coords):
            groups.append(current_group)
            current_group = [pos]
            progressed = step
            current_group_coords = pos_coords
        else:
            current_group.append(pos)
            progressed = projected
            current_group_coords = projected_coords
        prev_coord = curr_coord
    if current_group:
        groups.append(current_group)
    if len(groups) >= 2 and len(groups[0]) == 1 and groups[0][0] == line_positions[0]:
        groups[1] = groups[0] + groups[1]
        groups = groups[1:]
    while len(groups) >= 2:
        prev_anchor = wps[groups[-2][-1]].get("coordinate") or {}
        tail_anchor = wps[groups[-1][-1]].get("coordinate") or {}
        if not (isinstance(prev_anchor, dict) and prev_anchor and isinstance(tail_anchor, dict) and tail_anchor):
            break
        tail_gap_m = _dist_between_coords(prev_anchor, tail_anchor)
        if tail_gap_m + 1e-6 >= area_route_spacing_m:
            break
        if coord_cap > 0 and (_group_coord_count(groups[-2]) + _group_coord_count(groups[-1])) > coord_cap:
            break
        groups[-1] = groups[-2] + groups[-1]
        groups.pop(-2)

    result: list[OrderedDict] = [first_waypoint]
    for group_idx, group in enumerate(groups):
        rep_idx = group[-1]
        rep_src = OrderedDict(deepcopy(wps[rep_idx]))
        rep_coord = dict(rep_src.get("coordinate") or {})
        rep_fp = rep_src.get("filmingProperty") or {}
        chunk_coords: list[dict] = []
        chunk_speeds: list[float] = []
        chunk_fov = rep_fp.get("fieldOfView", first_fp.get("fieldOfView", FOV_DEG))
        chunk_sensor = rep_fp.get("sensorType", first_fp.get("sensorType", SENSOR_EO_IR))
        points_hint = interp_points
        for pos in group:
            wp = wps[pos]
            fp = wp.get("filmingProperty") or {}
            ls = fp.get("lineSearch") or {}
            coords = _copy_coord_list(ls.get("coordinateList") or [])
            if coords:
                chunk_coords.extend(coords)
            speed = ls.get("searchSpeed")
            if speed is not None:
                try:
                    chunk_speeds.append(float(speed))
                except Exception:
                    pass
            if points_hint is None:
                points_hint = ls.get("interpolationPoints")
            if "fieldOfView" in fp:
                chunk_fov = fp.get("fieldOfView", chunk_fov)
            if "sensorType" in fp:
                chunk_sensor = fp.get("sensorType", chunk_sensor)
        if not chunk_coords:
            continue
        line_speed = round(sum(chunk_speeds) / max(len(chunk_speeds), 1), 2) if chunk_speeds else None
        if line_speed is None:
            line_speed = search_speed
        if line_speed is None:
            line_speed = rep_src.get("speed", 0.0)
        if points_hint is None:
            points_hint = SWEEP_LINE_INTERP_POINTS
        anchor_source_coords = chunk_coords
        if group_idx == 0 and first_ls_coords:
            # Keep the first line waypoint tied to the first actual sweep line.
            anchor_source_coords = first_ls_coords
        rep_anchor_coord = _area_wp_coord_from_sweep(
            anchor_source_coords,
            bearing_deg=bearing_deg,
            offset_m=offset_m,
            altitude_fn=altitude_fn,
            reference_coord=reference_coord,
            offset_sign=offset_sign,
        )
        if rep_anchor_coord is None:
            rep_anchor_coord = OrderedDict([
                ("latitude", round(float(rep_coord.get("latitude", 0.0)), 6)),
                ("longitude", round(float(rep_coord.get("longitude", 0.0)), 6)),
                ("altitude", altitude_fn(float(rep_coord.get("latitude", 0.0)), float(rep_coord.get("longitude", 0.0)))),
            ])
        rep_src["coordinate"] = rep_anchor_coord
        rep_src["waypointPassType"] = PASS_FLYOVER if group_idx == len(groups) - 1 else PASS_FLYBY
        rep_src["filmingProperty"] = _mk_filming(
            operation_mode=OPMODE_LINE,
            fov=float(chunk_fov or FOV_DEG),
            sensor=int(chunk_sensor or SENSOR_EO_IR),
            line_search=OrderedDict([
                ("coordinateList", chunk_coords),
                ("searchSpeed", float(line_speed)),
                ("interpolationPoints", int(points_hint)),
            ]),
        )
        result.append(rep_src)

    if len(result) == 1:
        last_fp = line_wps[-1].get("filmingProperty") or {}
        last_ls = last_fp.get("lineSearch") or {}
        line_speed = search_speed
        if line_speed is None:
            line_speed = last_ls.get("searchSpeed")
        if line_speed is None:
            line_speed = first_ls.get("searchSpeed")
        if line_speed is None:
            line_speed = wps[-1].get("speed", 0.0)
        points_hint = interp_points
        if points_hint is None:
            points_hint = last_ls.get("interpolationPoints")
        if points_hint is None:
            points_hint = first_ls.get("interpolationPoints")
        if points_hint is None:
            points_hint = SWEEP_LINE_INTERP_POINTS
        fallback_waypoint = OrderedDict(deepcopy(wps[-1]))
        fallback_anchor_coord = _area_wp_coord_from_sweep(
            merged_coords,
            bearing_deg=bearing_deg,
            offset_m=offset_m,
            altitude_fn=altitude_fn,
            reference_coord=reference_coord,
            offset_sign=offset_sign,
        )
        if fallback_anchor_coord is not None:
            fallback_waypoint["coordinate"] = fallback_anchor_coord
        fallback_waypoint["waypointPassType"] = PASS_FLYOVER
        fallback_waypoint["filmingProperty"] = _mk_filming(
            operation_mode=OPMODE_LINE,
            fov=float(last_fp.get("fieldOfView", first_fp.get("fieldOfView", FOV_DEG)) or FOV_DEG),
            sensor=int(last_fp.get("sensorType", first_fp.get("sensorType", SENSOR_EO_IR)) or SENSOR_EO_IR),
            line_search=OrderedDict([
                ("coordinateList", merged_coords),
                ("searchSpeed", float(line_speed)),
                ("interpolationPoints", int(points_hint)),
            ]),
        )
        result.append(fallback_waypoint)

    route_positions = [idx for idx, wp in enumerate(result) if _has_line_search(wp)]
    if len(route_positions) >= 2:
        first_route_pos = route_positions[0]
        start_anchor = result[first_route_pos].get("coordinate") or {}
        end_anchor = result[route_positions[-1]].get("coordinate") or {}
        route_dx, route_dy = _vector_between_coords(start_anchor, end_anchor)
        route_norm = math.hypot(route_dx, route_dy)
        if route_norm >= 1.0:
            ux = route_dx / route_norm
            uy = route_dy / route_norm
            origin_lat = float(start_anchor.get("latitude", 0.0))
            origin_lon = float(start_anchor.get("longitude", 0.0))
            previous_progress = 0.0
            for pos in route_positions[1:]:
                curr_anchor = result[pos].get("coordinate") or {}
                curr_x, curr_y = llh_to_xy(
                    float(curr_anchor.get("latitude", origin_lat)),
                    float(curr_anchor.get("longitude", origin_lon)),
                    origin_lat,
                    origin_lon,
                )
                progress_m = max(previous_progress, (curr_x * ux) + (curr_y * uy))
                projected_lat, projected_lon = xy_to_llh(
                    progress_m * ux,
                    progress_m * uy,
                    origin_lat,
                    origin_lon,
                )
                result[pos]["coordinate"] = OrderedDict([
                    ("latitude", round(projected_lat, 6)),
                    ("longitude", round(projected_lon, 6)),
                    ("altitude", altitude_fn(projected_lat, projected_lon)),
                ])
                previous_progress = progress_m

    if result and route_positions:
        first_route_pos = route_positions[0]
        first_route_anchor = result[first_route_pos].get("coordinate") or {}
        forward_coord: dict | None = None
        if len(route_positions) >= 2:
            candidate = result[route_positions[1]].get("coordinate") or {}
            if isinstance(candidate, dict) and candidate:
                forward_coord = candidate
        if forward_coord is None:
            candidate = (
                ((result[0].get("filmingProperty") or {}).get("coordinateOrientation") or {}).get("coordinate")
                or {}
            )
            if isinstance(candidate, dict) and candidate:
                forward_coord = candidate
        if forward_coord is not None:
            vec_x, vec_y = _vector_between_coords(first_route_anchor, forward_coord)
            vec_norm = math.hypot(vec_x, vec_y)
            if vec_norm >= 1.0:
                lead_in_m = max(float(AREA_POINT_ANCHOR_LEAD_IN_M), 0.0)
                origin_lat = float(first_route_anchor.get("latitude", 0.0))
                origin_lon = float(first_route_anchor.get("longitude", 0.0))
                anchor_lat, anchor_lon = xy_to_llh(
                    -(vec_x / vec_norm) * lead_in_m,
                    -(vec_y / vec_norm) * lead_in_m,
                    origin_lat,
                    origin_lon,
                )
                result[0]["coordinate"] = OrderedDict([
                    ("latitude", round(anchor_lat, 6)),
                    ("longitude", round(anchor_lon, 6)),
                    ("altitude", altitude_fn(anchor_lat, anchor_lon)),
                ])

    return result


def _pack_area_wps_by_spacing(
    wps: list[OrderedDict],
    *,
    route_spacing_m: float,
    spacing_line_m: float,
    cruise_speed_mps: float,
    search_speed_weight: float,
    default_search_speed: float,
    post_entry_search_speed_scale: float = 1.0,
    post_entry_route_spacing_scale: float = 1.0,
) -> list[OrderedDict]:
    if not wps:
        return wps

    try:
        target_spacing_m = max(float(route_spacing_m), 1.0)
    except Exception:
        target_spacing_m = _area_route_wp_spacing_m()
    try:
        route_spacing_scale = float(post_entry_route_spacing_scale)
    except Exception:
        route_spacing_scale = 1.0
    if route_spacing_scale <= 0.0:
        route_spacing_scale = 1.0
    target_spacing_m *= route_spacing_scale

    try:
        line_search_speed_scale = float(post_entry_search_speed_scale)
    except Exception:
        line_search_speed_scale = 1.0
    if line_search_speed_scale <= 0.0:
        line_search_speed_scale = 1.0

    line_positions = [idx for idx, wp in enumerate(wps) if _has_line_search(wp)]
    if not line_positions:
        return wps

    first_wp = OrderedDict(deepcopy(wps[line_positions[0]]))
    first_fp = first_wp.get("filmingProperty") or {}
    first_ls = first_fp.get("lineSearch") or {}
    first_coords = _copy_coord_list(first_ls.get("coordinateList") or [])
    if not first_coords:
        return wps

    first_coord = wps[line_positions[0]].get("coordinate") or {}
    last_coord = wps[line_positions[-1]].get("coordinate") or {}
    progress_by_pos: dict[int, float] = {}

    route_dx, route_dy = _vector_between_coords(first_coord, last_coord)
    route_norm = math.hypot(route_dx, route_dy)
    if route_norm >= 1.0:
        ux = route_dx / route_norm
        uy = route_dy / route_norm
        origin_lat = float(first_coord.get("latitude", 0.0))
        origin_lon = float(first_coord.get("longitude", 0.0))
        previous_progress = 0.0
        for pos in line_positions:
            curr_coord = wps[pos].get("coordinate") or {}
            curr_lat = float(curr_coord.get("latitude", origin_lat))
            curr_lon = float(curr_coord.get("longitude", origin_lon))
            curr_x, curr_y = llh_to_xy(curr_lat, curr_lon, origin_lat, origin_lon)
            progress_m = max(previous_progress, (curr_x * ux) + (curr_y * uy))
            progress_m = min(progress_m, route_norm)
            progress_by_pos[pos] = progress_m
            previous_progress = progress_m
    else:
        progressed = 0.0
        prev_coord = first_coord
        for pos in line_positions:
            curr_coord = wps[pos].get("coordinate") or {}
            if pos != line_positions[0]:
                progressed += _dist_between_coords(prev_coord, curr_coord)
            progress_by_pos[pos] = progressed
            prev_coord = curr_coord

    sweep_start = _coord_with_dem_altitude(dict(first_coords[0]))
    first_point = OrderedDict(deepcopy(first_wp))
    first_point["waypointPassType"] = PASS_FLYBY
    first_point["filmingProperty"] = OrderedDict([
        ("fieldOfView", float(first_fp.get("fieldOfView", FOV_DEG) or FOV_DEG)),
        ("sensorType", int(first_fp.get("sensorType", SENSOR_EO_IR) or SENSOR_EO_IR)),
        ("operationMode", OPMODE_POINT),
        ("coordinateOrientation", OrderedDict([
            ("coordinate", OrderedDict(sweep_start)),
        ])),
    ])

    groups: list[list[int]] = []
    current_group: list[int] = [line_positions[0]]
    anchor_progress = float(progress_by_pos.get(line_positions[0], 0.0))
    for pos in line_positions[1:]:
        candidate_progress = float(progress_by_pos.get(pos, anchor_progress))
        last_progress = float(progress_by_pos.get(current_group[-1], anchor_progress))
        if (
            current_group
            and (candidate_progress - anchor_progress) > target_spacing_m
            and (last_progress - anchor_progress) >= 1.0
        ):
            groups.append(current_group)
            anchor_progress = last_progress
            current_group = [pos]
        else:
            current_group.append(pos)
    if current_group:
        groups.append(current_group)

    while len(groups) >= 2:
        prev_progress = float(progress_by_pos.get(groups[-2][-1], 0.0))
        tail_progress = float(progress_by_pos.get(groups[-1][-1], prev_progress))
        tail_gap_m = tail_progress - prev_progress
        if tail_gap_m + 1e-6 >= target_spacing_m:
            break
        groups[-1] = groups[-2] + groups[-1]
        groups.pop(-2)

    result: list[OrderedDict] = [first_point]
    for group_idx, group in enumerate(groups):
        rep_idx = group[-1]
        rep_src = OrderedDict(deepcopy(wps[rep_idx]))
        rep_fp = rep_src.get("filmingProperty") or {}
        chunk_coords: list[dict] = []
        chunk_speeds: list[float] = []
        chunk_fov = rep_fp.get("fieldOfView", first_fp.get("fieldOfView", FOV_DEG))
        chunk_sensor = rep_fp.get("sensorType", first_fp.get("sensorType", SENSOR_EO_IR))
        points_hint = None

        for pos in group:
            wp = wps[pos]
            fp = wp.get("filmingProperty") or {}
            ls = fp.get("lineSearch") or {}
            coords = _copy_coord_list(ls.get("coordinateList") or [])
            if coords:
                chunk_coords.extend(coords)
            speed = ls.get("searchSpeed")
            if speed is not None:
                try:
                    chunk_speeds.append(float(speed))
                except Exception:
                    pass
            if points_hint is None:
                points_hint = _interp_points_hint(ls.get("interpolationPoints"))
            if "fieldOfView" in fp:
                chunk_fov = fp.get("fieldOfView", chunk_fov)
            if "sensorType" in fp:
                chunk_sensor = fp.get("sensorType", chunk_sensor)

        if not chunk_coords:
            continue

        # Preserve the maximum pre-computed altitude from the group's individual WPs.
        # Since altitude was already computed per WP from its own sweep terrain before
        # packing, the max ensures the UAV can cover the highest terrain in the group.
        group_max_alt: int | None = None
        for pos in group:
            grp_alt = _safe_float_or_none((wps[pos].get("coordinate") or {}).get("altitude"))
            if grp_alt is not None:
                grp_alt_int = int(round(grp_alt))
                if group_max_alt is None or grp_alt_int > group_max_alt:
                    group_max_alt = grp_alt_int
        if group_max_alt is not None:
            grp_coord = dict(rep_src.get("coordinate") or {})
            rep_src["coordinate"] = OrderedDict([
                ("latitude", round(float(grp_coord.get("latitude", 0.0)), 6)),
                ("longitude", round(float(grp_coord.get("longitude", 0.0)), 6)),
                ("altitude", group_max_alt),
            ])

        sweep_width_m = _avg_sweep_width_m(chunk_coords, points_per_line=points_hint)
        line_speed = spacing_based_search_speed(
            sweep_len_m=sweep_width_m,
            spacing_m=spacing_line_m,
            cruise_speed_mps=cruise_speed_mps,
            weight=float(search_speed_weight) * line_search_speed_scale,
        )
        if line_speed is None and chunk_speeds:
            line_speed = round(sum(chunk_speeds) / len(chunk_speeds), 2)
        if line_speed is None:
            line_speed = default_search_speed
        if points_hint is None:
            points_hint = SWEEP_LINE_INTERP_POINTS

        rep_src["waypointPassType"] = PASS_FLYOVER if group_idx == len(groups) - 1 else PASS_FLYBY
        rep_src["filmingProperty"] = _mk_filming(
            operation_mode=OPMODE_LINE,
            fov=float(chunk_fov or FOV_DEG),
            sensor=int(chunk_sensor or SENSOR_EO_IR),
            line_search=OrderedDict([
                ("coordinateList", chunk_coords),
                ("searchSpeed", float(line_speed)),
                ("interpolationPoints", int(points_hint)),
            ]),
        )
        result.append(rep_src)

    return result


def _interp_points_hint(value: object) -> int | None:
    """Convert a stored interpolation hint into an int ≥ 2."""
    try:
        points = int(value)
    except (TypeError, ValueError):
        return None
    return points if points >= 2 else None


def _to_straight_coords_if_area(
    coords: list[dict],
    *,
    force_straight: bool,
) -> list[dict]:
    if not force_straight or len(coords) < 2:
        return coords
    return [_copy_coord_dict(coords[0]), _copy_coord_dict(coords[-1])]


def _collect_sweep_spans(coords: list[dict], points_per_line: int | None) -> list[float]:
    """Return individual sweep spans extracted from the coordinate list."""
    if not coords or len(coords) < 2:
        return []
    chunk = max(points_per_line or 2, 2)
    spans: list[float] = []
    idx = 0
    while idx < len(coords) - 1:
        start = coords[idx]
        end_idx = min(idx + chunk - 1, len(coords) - 1)
        end = coords[end_idx]
        lat0 = float(start.get("latitude", 0.0))
        lon0 = float(start.get("longitude", 0.0))
        lat1 = float(end.get("latitude", lat0))
        lon1 = float(end.get("longitude", lon0))
        dx, dy = llh_to_xy(lat1, lon1, lat0, lon0)
        dist = math.hypot(dx, dy)
        if dist > 0.0:
            spans.append(dist)
        idx = end_idx + 1

    return spans


def _avg_sweep_width_m(coords: list[dict], *, points_per_line: int | None = None) -> float | None:
    """
    Estimate the average span of each sweep line in meters.

    When coordinates include interpolated mid-points, `points_per_line` tells the helper
    how many samples belong to a single sweep (>=2). The default pairs entries (0-1, 2-3, ...).
    """
    spans = _collect_sweep_spans(coords, points_per_line)
    if not spans:
        return None
    return round(sum(spans) / len(spans), 2)


def _subdivide_segment_points(start: dict, end: dict, points: int) -> list[tuple[float, float]]:
    lat0 = float(start.get("latitude", 0.0))
    lon0 = float(start.get("longitude", 0.0))
    lat1 = float(end.get("latitude", lat0))
    lon1 = float(end.get("longitude", lon0))
    if points <= 2:
        return [(lat0, lon0), (lat1, lon1)]
    dx, dy = llh_to_xy(lat1, lon1, lat0, lon0)
    points_out: list[tuple[float, float]] = []
    for idx in range(points):
        if idx == 0:
            points_out.append((lat0, lon0))
            continue
        if idx == points - 1:
            points_out.append((lat1, lon1))
            continue
        t = idx / (points - 1)
        xi = dx * t
        yi = dy * t
        lat_i, lon_i = xy_to_llh(xi, yi, lat0, lon0)
        points_out.append((round(lat_i, 6), round(lon_i, 6)))
    return points_out


def _subdivide_segment(start: dict, end: dict, points: int) -> list[dict]:
    """Generate evenly spaced coordinates along start-end inclusive."""
    if points <= 2:
        return [start.copy(), end.copy()]
    alt0 = float(start.get("altitude", Altitude))
    alt1 = float(end.get("altitude", alt0))
    coords: list[dict] = []
    for idx, (lat_i, lon_i) in enumerate(_subdivide_segment_points(start, end, points)):
        if idx == 0:
            coords.append(start.copy())
            continue
        if idx == points - 1:
            coords.append(end.copy())
            continue
        t = idx / (points - 1)
        alt_i = alt0 + (alt1 - alt0) * t
        coords.append(OrderedDict([
            ("latitude", float(lat_i)),
            ("longitude", float(lon_i)),
            ("altitude", int(round(alt_i))),
        ]))
    return coords


def _interpolate_line_coords(coords: list[dict], points: int) -> list[dict]:
    """Split sweep line coordinate pairs into interpolated segments."""
    started_at = time.perf_counter()
    if points <= 2 or not coords:
        return coords
    trusted_materialize_raw = os.environ.get(
        "MISSION_PLAN_TRUSTED_LINE_INTERPOLATION_DEM_MATERIALIZE_ENABLED",
        "",
    )
    if str(trusted_materialize_raw).strip().lower() in {"1", "true", "yes", "on"}:
        point_groups: list[list[tuple[float, float]]] = []
        can_materialize_trusted = True
        for idx in range(0, len(coords), 2):
            start = coords[idx]
            end = coords[idx + 1] if idx + 1 < len(coords) else start
            if (
                not isinstance(start, dict)
                or not isinstance(end, dict)
                or not _coord_has_trusted_dem_altitude(start)
                or not _coord_has_trusted_dem_altitude(end)
            ):
                can_materialize_trusted = False
                break
            point_groups.append(_subdivide_segment_points(start, end, points))
        if can_materialize_trusted and point_groups:
            result: list[dict] = []
            for subdivided in _materialize_dem_coord_groups(point_groups):
                if (
                    result
                    and subdivided
                    and result[-1].get("latitude") == subdivided[0].get("latitude")
                    and result[-1].get("longitude") == subdivided[0].get("longitude")
                ):
                    subdivided = subdivided[1:]
                result.extend(subdivided)
            _record_dense_linesearch_metrics(generateLineSearchMs=(time.perf_counter() - started_at) * 1000.0)
            return result
    result: list[dict] = []
    for idx in range(0, len(coords), 2):
        start = coords[idx]
        end = coords[idx + 1] if idx + 1 < len(coords) else start
        subdivided = _subdivide_segment(start, end, points)
        if result and subdivided and result[-1] == subdivided[0]:
            subdivided = subdivided[1:]
        result.extend(subdivided)
    _record_dense_linesearch_metrics(generateLineSearchMs=(time.perf_counter() - started_at) * 1000.0)
    return result


class _WPAllocator:
    def __init__(
        self,
        start: int | None = None,
        end: int | None = None,
        overflow_block_size: int | None = None,
    ) -> None:
        self._local_next = start
        self._local_end = end
        self._use_global = start is None
        try:
            self._overflow_block_size = max(0, int(overflow_block_size or 0))
        except Exception:
            self._overflow_block_size = 0

    def _reserve_overflow_block(self) -> bool:
        if self._overflow_block_size <= 0:
            return False
        start = int(_reserve_waypoint_block(self._overflow_block_size))
        self._local_next = start
        self._local_end = start + self._overflow_block_size - 1
        return True

    def alloc(self) -> int:
        if self._use_global:
            return int(_reserve_waypoint_block(1))
        if self._local_next is None:
            raise RuntimeError("Waypoint allocator misconfigured (local start unset)")
        if self._local_next <= 0:
            raise RuntimeError("WaypointID pool exhausted")
        wid = self._local_next
        if self._local_end is not None and wid > int(self._local_end):
            if self._reserve_overflow_block():
                return self.alloc()
            raise RuntimeError("WaypointID reserved block exhausted")
        self._local_next += 1
        return wid

def _index_refpoints(ref0203: dict | None):
    """
    0203에서 기체ID→좌표 맵 구성.
    return: (to_map, ho_map)  각 값은 {aid: {"latitude":..,"longitude":..,"altitude":..}}
    """
    if not ref0203:
        return {}, {}
    to_map = {}
    for it in ref0203.get("takeOverInfoList", []) or []:
        aid = it.get("aircraftID")
        c   = it.get("coordinate") or {}
        if isinstance(aid, int) and 1 <= aid <= 6 and "latitude" in c and "longitude" in c:
            to_map[aid] = {
                "latitude": float(c.get("latitude", 0.0)),
                "longitude": float(c.get("longitude", 0.0)),
                "altitude": _normalize_altitude(c.get("altitude", Altitude)),
            }
    ho_map = {}
    for it in ref0203.get("handOverInfoList", []) or []:
        aid = it.get("aircraftID")
        c   = it.get("coordinate") or {}
        if isinstance(aid, int) and 1 <= aid <= 6 and "latitude" in c and "longitude" in c:
            ho_map[aid] = {
                "latitude": float(c.get("latitude", 0.0)),
                "longitude": float(c.get("longitude", 0.0)),
                "altitude": _normalize_altitude(c.get("altitude", Altitude)),
            }
    return to_map, ho_map


def _eta_ms_llh(c1: dict, c2: dict, speed_mps: float) -> int:
    """
    간단한 구면 근사로 거리→ETA(s) 계산. speed_mps<=0이면 0.
    c* = {"latitude": float, "longitude": float}
    """
    try:
        lat1, lon1 = float(c1["latitude"]), float(c1["longitude"])
        lat2, lon2 = float(c2["latitude"]), float(c2["longitude"])
    except Exception:
        return 0
    DEG_M = 111_132.0
    dx = (lon2 - lon1) * DEG_M * math.cos(math.radians((lat1 + lat2) / 2.0))
    dy = (lat2 - lat1) * DEG_M
    dist_m = math.hypot(dx, dy)
    if speed_mps and speed_mps > 0.0:
        return int(round(dist_m / speed_mps))
    return 0


def _annotate_eta_ms_inplace(waypoints: list[OrderedDict], default_speed_mps: float) -> None:
    # NOTE: historical function name kept for compatibility; ETA is emitted in seconds.
    if not waypoints:
        return

    # d0303 builds waypoint objects before final waypointID / nextWaypointID allocation.
    # At that stage many WPs still carry waypointID=0, nextWaypointID=0 placeholders.
    # If we try to restore order from the next-chain too early, only a tail subset can be
    # observed and pre-filled placeholder ETA values (e.g. 2500) leak into the final file.
    # Monitoring then interprets them as mission planned time, which is why some missions
    # show absurd values such as 5000s.
    waypoint_ids = [int(wp.get("waypointID", 0) or 0) for wp in waypoints if isinstance(wp, dict)]
    next_ids = [int(wp.get("nextWaypointID", 0) or 0) for wp in waypoints if isinstance(wp, dict)]
    has_resolved_chain = (
        len(waypoint_ids) == len(waypoints)
        and len(set(waypoint_ids)) == len(waypoints)
        and all(wid > 0 for wid in waypoint_ids)
        and any(nid > 0 for nid in next_ids)
    )
    ordered = _order_by_next_chain(waypoints) if has_resolved_chain else list(waypoints)
    if not ordered:
        return

    # ETA is stored as cumulative seconds within this flight path.
    for wp in waypoints:
        wp["eta"] = 0
    ordered[0]["eta"] = 0
    acc_s = 0.0
    for i in range(1, len(ordered)):
        dt_s = _time_from_prev_to_curr_s(ordered[i - 1], ordered[i], default_speed_mps=default_speed_mps)
        acc_s += dt_s
        cum_s = int(round(acc_s))
        ordered[i]["eta"] = max(0, cum_s)

def _calc_search_speed_from_paths(
    cur_coord: dict,
    next_coord: dict,
    line_coords: list[dict],
    cruise_speed: float,
    weight: float | None = None,
) -> float | None:
    if _SearchSpeedCalculator is None:
        return None
    if not cur_coord or not next_coord:
        return None
    lat0 = cur_coord.get("latitude")
    lon0 = cur_coord.get("longitude")
    if lat0 is None or lon0 is None:
        if line_coords:
            lat0 = line_coords[0].get("latitude")
            lon0 = line_coords[0].get("longitude")
    if lat0 is None or lon0 is None:
        return None

    def _to_xyz(coord: dict) -> tuple[float, float, float] | None:
        if not coord or "latitude" not in coord or "longitude" not in coord:
            return None
        x, y = llh_to_xy(coord["latitude"], coord["longitude"], lat0, lon0)
        alt = float(coord.get("altitude") or 0.0)
        return (x, y, alt)

    u0 = _to_xyz(cur_coord)
    u1 = _to_xyz(next_coord)
    if u0 is None or u1 is None:
        return None

    sweep_pts: list[tuple[float, float, float]] = []
    for c in line_coords:
        pt = _to_xyz(c)
        if pt is not None:
            sweep_pts.append(pt)
    if len(sweep_pts) < 2:
        return None

    try:
        calc = _SearchSpeedCalculator([u0, u1], sweep_pts, float(cruise_speed))
        speed = float(calc.compute_search_speed())
    except Exception:
        return None
    try:
        effective_weight = float(weight) if weight is not None else 1.0
    except Exception:
        effective_weight = 1.0
    if effective_weight <= 0.0:
        effective_weight = 1.0
    return round(max(0.0, speed * effective_weight), 2)


def _apply_line_search_speed_from_paths(wps: list[OrderedDict], cruise_speed: float) -> None:
    if _SearchSpeedCalculator is None or len(wps) < 2:
        return
    started_at = time.perf_counter()
    recalc_count = 0
    recalc_coord_count = 0
    skipped_count = 0
    force_recalc = _recompute_line_search_speed_enabled()
    for idx in range(1, len(wps)):
        wp = wps[idx]
        if wp.get("_skip_path_search_speed_recalc"):
            skipped_count += 1
            continue
        prev_wp = wps[idx - 1]
        fp = wp.get("filmingProperty") or OrderedDict()
        line_search = fp.get("lineSearch")
        if not line_search:
            continue
        # Speeds generated before sweep grouping use the nominal strip spacing.
        # Recompute planner-owned lineSearch speeds on the final waypoint leg so
        # dense/packed sweeps finish before the controller advances to the next WP.
        planner_generated_speed = wp.get("_path_search_speed_weight") is not None
        if not force_recalc and line_search.get("searchSpeed") is not None and not planner_generated_speed:
            skipped_count += 1
            continue
        coords = line_search.get("coordinateList") or []
        if not coords:
            continue
        recalc_count += 1
        recalc_coord_count += len(coords)
        speed = _calc_search_speed_from_paths(
            prev_wp.get("coordinate") or {},
            wp.get("coordinate") or {},
            coords,
            cruise_speed,
            weight=_safe_float_or_none(wp.get("_path_search_speed_weight")),
        )
        if speed is None:
            continue
        line_search["searchSpeed"] = speed
    elapsed_ms = (time.perf_counter() - started_at) * 1000.0
    _record_dense_linesearch_metrics(
        searchSpeedRecalcMs=elapsed_ms,
        lineSearchSpeedMs=elapsed_ms,
        searchSpeedRecalcCount=recalc_count,
        searchSpeedRecalcCoordCount=recalc_coord_count,
        searchSpeedRecalcSkippedCount=skipped_count,
    )


def _recompute_eta_ecf_inplace(waypoints: list[OrderedDict], default_speed_mps: float) -> None:
    started_at = time.perf_counter()
    if not waypoints:
        return
    for wp in waypoints:
        wp.setdefault("isDone", False)
    _apply_line_search_speed_from_paths(waypoints, default_speed_mps)
    _annotate_eta_ms_inplace(waypoints, default_speed_mps=default_speed_mps)
    total_eta = sum(max(0, int(wp.get("eta", 0))) for wp in waypoints) or 1
    cum = 0
    for wp in waypoints:
        step_eta = max(0, int(wp.get("eta", 0)))
        cum += step_eta
        wp["ecf"] = round(min(cum / total_eta, 1.0), 2)
    waypoints[-1]["ecf"] = 1.0
    _record_dense_linesearch_metrics(etaEcfMs=(time.perf_counter() - started_at) * 1000.0)


def _normalize_altitudes_recursive_inplace(node: object) -> None:
    if isinstance(node, dict):
        node.pop(_DEM_ALT_COORD_TRUSTED_KEY, None)
        if "altitude" in node:
            node["altitude"] = _normalize_altitude(node.get("altitude"))
        for value in node.values():
            _normalize_altitudes_recursive_inplace(value)
        return
    if isinstance(node, list):
        for item in node:
            _normalize_altitudes_recursive_inplace(item)


def _ensure_minimum_waypoint_chain_inplace(waypoints: list[OrderedDict]) -> None:
    if len(waypoints) != 1:
        return
    original = waypoints[0]
    if not isinstance(original, dict):
        return

    terminal = deepcopy(original)

    if "hovering" in original:
        terminal["hovering"] = deepcopy(original.get("hovering"))
        original.pop("hovering", None)
    if "loiterProperty" in original:
        terminal["loiterProperty"] = deepcopy(original.get("loiterProperty"))
        original.pop("loiterProperty", None)

    terminal.pop("attack", None)
    terminal.pop("filmingProperty", None)

    try:
        original["eta"] = int(original.get("eta", 0) or 0)
    except Exception:
        original["eta"] = 0
    terminal["eta"] = int(original["eta"])
    original["ecf"] = 0.0
    terminal["ecf"] = 1.0
    original["nextWaypointID"] = 0
    terminal["nextWaypointID"] = 0

    waypoints.append(terminal)


def _normalize_output_waypoints_inplace(waypoints: list[OrderedDict]) -> None:
    if not isinstance(waypoints, list):
        return
    started_at = time.perf_counter()
    try:
        _clear_ground_required_cache_inplace(waypoints)
        for wp in waypoints:
            _normalize_altitudes_recursive_inplace(wp)
        before_len = len(waypoints)
        _ensure_minimum_waypoint_chain_inplace(waypoints)
        for wp in waypoints[before_len:]:
            if isinstance(wp, dict):
                _clear_ground_required_cache_inplace([wp])
                _normalize_altitudes_recursive_inplace(wp)
    finally:
        _record_dense_linesearch_metrics(
            outputNormalizeMs=(time.perf_counter() - started_at) * 1000.0,
        )


def _drop_redundant_close_area_entry_anchor_inplace(
    prev_tail_coord: dict | None,
    wps: list[OrderedDict],
    *,
    min_gap_m: float,
) -> bool:
    if not isinstance(prev_tail_coord, dict) or len(wps) < 2:
        return False

    first_wp = wps[0]
    second_wp = wps[1]
    first_fp = first_wp.get("filmingProperty") or {}
    if int(first_fp.get("operationMode", OPMODE_NONE) or OPMODE_NONE) != OPMODE_POINT:
        return False
    if "coordinateOrientation" not in first_fp:
        return False

    first_coord = first_wp.get("coordinate") or {}
    if not (isinstance(first_coord, dict) and "latitude" in first_coord and "longitude" in first_coord):
        return False

    if _dist_between_coords(prev_tail_coord, first_coord) + 1e-6 >= max(float(min_gap_m), 1.0):
        return False

    first_target = _wp_effective_start_coord(first_wp)
    second_target = _wp_effective_start_coord(second_wp)
    if not first_target or not second_target:
        return False
    if _dist_between_coords(first_target, second_target) > 20.0:
        return False

    wps.pop(0)
    return True


def _merge_duplicate_entry_line_search_after_point_inplace(
    wps: list[OrderedDict],
    *,
    max_entry_gap_m: float = 3.0,
    max_short_line_coords: int | None = None,
) -> bool:
    if len(wps) < 3:
        return False

    entry_wp = wps[0]
    short_line_wp = wps[1]
    next_line_wp = wps[2]

    entry_fp = entry_wp.get("filmingProperty") or {}
    entry_mode = int(entry_fp.get("operationMode", OPMODE_NONE) or OPMODE_NONE)
    if entry_mode not in (OPMODE_HOLD, OPMODE_POINT):
        return False
    if _has_line_search(entry_wp):
        return False
    if not _has_line_search(short_line_wp) or not _has_line_search(next_line_wp):
        return False

    entry_coord = entry_wp.get("coordinate") or {}
    short_coord = short_line_wp.get("coordinate") or {}
    if not (
        isinstance(entry_coord, dict)
        and "latitude" in entry_coord
        and "longitude" in entry_coord
        and isinstance(short_coord, dict)
        and "latitude" in short_coord
        and "longitude" in short_coord
    ):
        return False

    try:
        if _dist_between_coords(entry_coord, short_coord) > max(float(max_entry_gap_m), 0.1):
            return False
    except Exception:
        return False

    short_fp = short_line_wp.get("filmingProperty") or {}
    short_ls = short_fp.get("lineSearch") or {}
    next_fp = next_line_wp.get("filmingProperty") or {}
    next_ls = next_fp.get("lineSearch") or OrderedDict()
    short_coords = _copy_coord_list(short_ls.get("coordinateList") or [])
    next_coords = _copy_coord_list(next_ls.get("coordinateList") or [])
    if not short_coords or not next_coords:
        return False

    short_interp = _interp_points_hint(short_ls.get("interpolationPoints"))
    if short_interp is None:
        short_interp = _interp_points_hint(SWEEP_LINE_INTERP_POINTS) or 2
    coord_limit = int(max_short_line_coords) if max_short_line_coords is not None else max(2, int(short_interp))
    if len(short_coords) > max(coord_limit, 2):
        return False

    try:
        if _dist_between_coords(entry_coord, short_coords[0]) > max(float(max_entry_gap_m), 5.0):
            return False
    except Exception:
        return False

    merged_coords = _copy_coord_list(short_coords)
    for coord in next_coords:
        if merged_coords:
            try:
                if _dist_between_coords(merged_coords[-1], coord) <= 1.0:
                    continue
            except Exception:
                pass
        merged_coords.append(_copy_coord_dict(coord))
    if not merged_coords:
        return False

    next_ls["coordinateList"] = merged_coords
    if not next_ls.get("interpolationPoints"):
        next_ls["interpolationPoints"] = (
            short_ls.get("interpolationPoints")
            or SWEEP_LINE_INTERP_POINTS
        )
    for key in ("fieldOfView", "sensorType"):
        if key in short_fp:
            next_fp[key] = deepcopy(short_fp[key])
    next_fp["lineSearch"] = next_ls
    next_line_wp["filmingProperty"] = next_fp
    if next_line_wp.get("_path_search_speed_weight") is None:
        inherited_weight = (
            short_line_wp.get("_path_search_speed_weight")
            if short_line_wp.get("_path_search_speed_weight") is not None
            else next_line_wp.get("_path_search_speed_weight")
        )
        next_line_wp["_path_search_speed_weight"] = (
            inherited_weight if inherited_weight is not None else 1.0
        )
    del wps[1]
    return True


def _prepend_point_before_leading_line_search_inplace(
    wps: list[OrderedDict],
    *,
    entry_coord: dict | None,
    speed_mps: float,
) -> bool:
    if not wps or not _has_line_search(wps[0]):
        return False

    first_line_wp = wps[0]
    first_fp = first_line_wp.get("filmingProperty") or {}
    first_ls = first_fp.get("lineSearch") or {}
    first_coords = _copy_coord_list(first_ls.get("coordinateList") or [])
    orientation_coord = first_coords[0] if first_coords else first_line_wp.get("coordinate")
    if not isinstance(orientation_coord, dict):
        return False

    base_coord = entry_coord if isinstance(entry_coord, dict) else first_line_wp.get("coordinate")
    if not isinstance(base_coord, dict):
        return False
    if "altitude" in base_coord:
        waypoint_coord = _copy_coord_dict(base_coord)
    else:
        waypoint_coord = _coord_with_dem_altitude(base_coord)

    try:
        fov = float(first_fp.get("fieldOfView", FOV_DEG) or FOV_DEG)
    except Exception:
        fov = float(FOV_DEG)
    try:
        sensor = int(first_fp.get("sensorType", SENSOR_EO_IR) or SENSOR_EO_IR)
    except Exception:
        sensor = int(SENSOR_EO_IR)

    entry_wp = OrderedDict([
        ("waypointID", 0),
        ("coordinate", OrderedDict([
            ("latitude", round(float(waypoint_coord.get("latitude", 0.0)), 6)),
            ("longitude", round(float(waypoint_coord.get("longitude", 0.0)), 6)),
            ("altitude", _normalize_altitude(waypoint_coord.get("altitude", 0))),
        ])),
        ("speed", float(speed_mps)),
        ("eta", 0),
        ("ecf", 0.0),
        ("nextWaypointID", 0),
        ("waypointPassType", PASS_FLYBY),
        ("filmingProperty", OrderedDict([
            ("fieldOfView", fov),
            ("sensorType", sensor),
            ("operationMode", OPMODE_POINT),
            ("coordinateOrientation", OrderedDict([
                ("coordinate", _coord_with_dem_altitude(orientation_coord)),
            ])),
        ])),
    ])
    wps.insert(0, entry_wp)
    return True


def _move_leading_close_takeover_scan_to_next_line_inplace(
    wps: list[OrderedDict],
    *,
    max_entry_gap_m: float = 300.0,
    scan_overlap_tol_m: float = 5.0,
) -> bool:
    if len(wps) < 3:
        return False

    entry_wp = wps[0]
    first_line_wp = wps[1]
    next_line_wp = wps[2]
    entry_fp = entry_wp.get("filmingProperty") or {}
    entry_mode = int(entry_fp.get("operationMode", OPMODE_NONE) or OPMODE_NONE)
    if entry_mode not in (OPMODE_HOLD, OPMODE_POINT):
        return False
    if _has_line_search(entry_wp):
        return False
    if not _has_line_search(first_line_wp) or not _has_line_search(next_line_wp):
        return False

    entry_coord = entry_wp.get("coordinate") or {}
    first_line_coord = first_line_wp.get("coordinate") or {}
    if not (
        isinstance(entry_coord, dict)
        and "latitude" in entry_coord
        and "longitude" in entry_coord
        and isinstance(first_line_coord, dict)
        and "latitude" in first_line_coord
        and "longitude" in first_line_coord
    ):
        return False

    first_fp = first_line_wp.get("filmingProperty") or {}
    first_ls = first_fp.get("lineSearch") or {}
    first_coords = _copy_coord_list(first_ls.get("coordinateList") or [])
    if not first_coords:
        return False

    try:
        entry_to_line_m = _dist_between_coords(entry_coord, first_line_coord)
    except Exception:
        return False
    if entry_to_line_m > max(float(max_entry_gap_m), 1.0):
        return False

    next_fp = next_line_wp.get("filmingProperty") or {}
    next_ls = next_fp.get("lineSearch") or OrderedDict()
    next_coords = _copy_coord_list(next_ls.get("coordinateList") or [])
    merged_coords = first_coords + next_coords
    if not merged_coords:
        return False

    first_speed = first_ls.get("searchSpeed")
    next_speed = next_ls.get("searchSpeed")
    merged_speed = next_speed if next_speed is not None else first_speed
    if first_speed is not None and next_speed is not None:
        try:
            merged_speed = round((float(first_speed) + float(next_speed)) * 0.5, 2)
        except Exception:
            merged_speed = next_speed
    next_ls["coordinateList"] = merged_coords
    if merged_speed is not None:
        next_ls["searchSpeed"] = merged_speed
    next_ls["interpolationPoints"] = (
        next_ls.get("interpolationPoints")
        or first_ls.get("interpolationPoints")
        or SWEEP_LINE_INTERP_POINTS
    )
    next_fp["lineSearch"] = next_ls
    for key in ("fieldOfView", "sensorType"):
        if key in first_fp:
            next_fp[key] = deepcopy(first_fp[key])
    next_line_wp["filmingProperty"] = next_fp

    del wps[1]
    return True


def _append_area_transition_links_inplace(
    prev_pkt: dict,
    next_pkt: dict,
    *,
    altitude_fn: Callable[[float, float], int],
) -> int:
    if not _dubins_link_available():
        return 0

    prev_wps = prev_pkt.get("wplist") or []
    next_wps = next_pkt.get("wplist") or []
    if len(prev_wps) < 2 or len(next_wps) < 2:
        return 0

    prev_start_coord = prev_wps[-2].get("coordinate") or {}
    prev_end_coord = prev_wps[-1].get("coordinate") or {}
    next_first_coord = next_wps[0].get("coordinate") or {}
    next_second_coord = next_wps[1].get("coordinate") or {}
    if not all(
        isinstance(coord, dict) and "latitude" in coord and "longitude" in coord
        for coord in (prev_start_coord, prev_end_coord, next_first_coord, next_second_coord)
    ):
        return 0

    target_coord = _wp_effective_start_coord(next_wps[0])
    next_fp = next_wps[0].get("filmingProperty") or {}
    try:
        target_fov = float(next_fp.get("fieldOfView", POINT_FOV_DEG) or POINT_FOV_DEG)
    except Exception:
        target_fov = float(POINT_FOV_DEG)

    try:
        cruise_speed_mps = float(
            prev_pkt.get(
                "_mission_cruise_speed_mps",
                next_pkt.get("_mission_cruise_speed_mps", 40.0),
            ) or 40.0
        )
    except Exception:
        cruise_speed_mps = 40.0

    turn_radius_m = _turn_radius_m_for_speed(cruise_speed_mps)
    link_wps = _build_area_dubins_entry_wps(
        prev_start_coord=prev_start_coord,
        prev_end_coord=prev_end_coord,
        next_first_coord=next_first_coord,
        next_second_coord=next_second_coord,
        turn_radius_m=turn_radius_m,
        cruise_speed_mps=cruise_speed_mps,
        altitude_fn=altitude_fn,
        target_coord=target_coord,
        target_fov=target_fov,
        min_link_gap_m=max(80.0, turn_radius_m * 0.2),
    )
    if not link_wps:
        return 0

    for link_wp in link_wps:
        link_wp.setdefault("isDone", False)
    prev_wps.extend(link_wps)
    _recompute_eta_ecf_inplace(prev_wps, default_speed_mps=cruise_speed_mps)
    return len(link_wps)


def build_flight_plans(
    missions: list[dict],
    wp_alloc: _WPAllocator | None = None,
    cruise_speed: float = 30.0,
    turn_step_deg: float = 45.0,
    ref0203: dict | None = None,
) -> list[dict]:
    _runtime_setting_cache_begin()
    _dem_alt_run_map_begin()
    try:
        return _build_flight_plans_impl(
            missions,
            wp_alloc=wp_alloc,
            cruise_speed=cruise_speed,
            turn_step_deg=turn_step_deg,
            ref0203=ref0203,
        )
    finally:
        _dem_alt_run_map_end()
        _runtime_setting_cache_end()


def _build_flight_plans_impl(
    missions: list[dict],
    wp_alloc: _WPAllocator | None = None,
    cruise_speed: float = 30.0,
    turn_step_deg: float = 45.0,
    ref0203: dict | None = None,
) -> list[dict]:
    with _SWEEP_DEBUG_LOCK:
        _SWEEP_DEBUG_CACHE.clear()
    wp_alloc = wp_alloc or _WPAllocator()
    now_ms = now_ms_since_2000()
    take_over_map, _ = _index_refpoints(ref0203)
    # ── 상수 ───────────────────────────────────────────────
    SENSOR, OPMODE = 1, 2
    DEFAULT_SEARCH_SPEED = round(cruise_speed * DEFAULT_SEARCH_SPEED_MULTIPLIER, 2)
    geom = _active_sweep_geometry()
    ALT_M = geom.separation_m
    geom_fov_deg = geom.fov_deg
    sweep_spacing_m = _sweep_spacing_m(separation_m=ALT_M, fov_deg=geom_fov_deg)
    runtime_line_auto_fov_from_db = _runtime_auto_fov_from_db()
    runtime_area_sweep_mode = _runtime_area_sweep_mode()
    runtime_area_auto_fov_from_db = _runtime_area_auto_fov_from_db()
    runtime_line_spacing_scale = _line_raw_spacing_scale()
    runtime_area_spacing_scale = _area_raw_spacing_scale()
    runtime_line_route_spacing_m = _line_route_wp_spacing_m()
    runtime_payload = _runtime_settings_payload()
    runtime_manual_fov_active = _runtime_manual_fov_sync_active(runtime_payload)
    runtime_line_fov_deg = _runtime_manual_fov_deg("line_custom_fov_deg", float(FOV_DEG), runtime_payload)
    runtime_area_fov_deg = _runtime_manual_fov_deg("area_custom_fov_deg", float(AREA_CUSTOM_FOV_DEG), runtime_payload)
    runtime_nadir_fov_deg = _runtime_manual_fov_deg("area_nadir_fov_deg", float(AREA_NADIR_FOV_DEG), runtime_payload)
    runtime_point_fov_deg = _runtime_manual_fov_deg("point_fov_deg", float(POINT_FOV_DEG), runtime_payload)
    runtime_entry_hold_fov_deg = _runtime_manual_fov_deg("entry_hold_fov_deg", float(ENTRY_HOLD_FOV_DEG), runtime_payload)
    if not runtime_manual_fov_active:
        runtime_values = runtime_payload.get("values") if isinstance(runtime_payload, dict) else {}
        if isinstance(runtime_values, dict):
            try:
                runtime_line_fov_deg = float(runtime_values.get("line_custom_fov_deg", runtime_line_fov_deg))
            except Exception:
                pass
            try:
                runtime_area_fov_deg = float(runtime_values.get("area_custom_fov_deg", runtime_area_fov_deg))
            except Exception:
                pass
            try:
                runtime_nadir_fov_deg = float(runtime_values.get("area_nadir_fov_deg", runtime_nadir_fov_deg))
            except Exception:
                pass
            try:
                runtime_point_fov_deg = float(runtime_values.get("point_fov_deg", runtime_point_fov_deg))
            except Exception:
                pass
            try:
                runtime_entry_hold_fov_deg = float(runtime_values.get("entry_hold_fov_deg", runtime_entry_hold_fov_deg))
            except Exception:
                pass
    def _wp_alt(
        lat: float,
        lon: float,
        fallback: float | int | None = None,
        ground_ref_m: float | None = None,
    ) -> int:
        base = fallback if fallback is not None else Altitude
        try:
            base_alt = float(base)
        except Exception:
            base_alt = float(Altitude)
        if ground_ref_m is None:
            ground = float(_dem_alt(lat, lon))
        else:
            try:
                ground = float(ground_ref_m)
            except Exception:
                ground = float(_dem_alt(lat, lon))
        return int(round(ground + base_alt))
    DEG_M = 111_132
    # ── 마지막점용 POINT 촬영 블록 생성기 ─────────────────
    def _mk_point_filming_for_coord(coord: dict) -> OrderedDict:
        orientation_coord = _coord_with_dem_altitude(coord)
        return OrderedDict([
            ("fieldOfView", runtime_point_fov_deg),
            ("sensorType", SENSOR_EO_IR),
            ("operationMode", OPMODE_POINT),
            ("coordinateOrientation", OrderedDict([
                ("coordinate", orientation_coord)
            ])),
        ])

    packets: list[dict] = []

    formation_offsets = [(-100, -100, 0), (100, -100, 0)]

    def _formation_key(miss: dict) -> int | None:
        rel = miss.get("relatedMission") if isinstance(miss.get("relatedMission"), dict) else {}
        raw = rel.get("inputMissionID") or miss.get("inputMissionID") or miss.get("pathID")
        try:
            return int(raw)
        except Exception:
            return None

    def _formation_role(info: dict) -> Optional[str]:
        if not isinstance(info, dict):
            return None
        if info.get("individualMissionType") != 7:
            return None
        line_list = info.get("lineList") or []
        if not line_list:
            return None
        try:
            width_val = float(line_list[0].get("width", 0))
        except Exception:
            return None
        width_tag = int(round(width_val))
        if width_tag == 0:
            return "leader"
        if width_tag == 1:
            return "follower"
        return None

    def _is_formation_line(info: dict) -> bool:
        if not isinstance(info, dict):
            return False
        line_list = info.get("lineList") or []
        if not line_list:
            return False
        try:
            width_val = float(line_list[0].get("width", 0))
        except Exception:
            return False
        return width_val <= 1.0

    formation_grouping_started = time.perf_counter()
    formation_groups: dict[int, dict[str, list[int]]] = {}
    for miss in missions:
        aid = miss.get("aircraftID")
        if aid not in (4, 5, 6):
            continue
        info = miss.get("individualMissionInfo") if isinstance(miss.get("individualMissionInfo"), dict) else {}
        role = _formation_role(info)
        if role is None and _is_formation_line(info):
            role = "follower"
        if role is None:
            continue
        key = _formation_key(miss)
        if key is None:
            continue
        bucket = formation_groups.setdefault(key, {"leader": [], "follower": []})
        bucket[role].append(int(aid))

    formation_group_sorted: dict[int, dict[str, list[int]]] = {}
    formation_leaders: dict[int, int] = {}
    for key, bucket in formation_groups.items():
        leaders = sorted(set(bucket.get("leader") or []))
        followers = sorted(set(bucket.get("follower") or []))
        formation_group_sorted[key] = {"leader": leaders, "follower": followers}
        preferred = None
        for cid in (4, 5, 6):
            if cid in leaders or cid in followers:
                preferred = cid
                break
        if preferred is not None:
            formation_leaders[key] = preferred
        elif leaders:
            formation_leaders[key] = leaders[0]
        elif followers:
            formation_leaders[key] = followers[0]
    _record_dense_linesearch_metrics(
        formationGroupingMs=(time.perf_counter() - formation_grouping_started) * 1000.0,
    )

    formation_followers: list[tuple[int, dict]] = []

    def _strip_filming_properties(wplist: list[OrderedDict]) -> None:
        for wp in wplist:
            if isinstance(wp, dict):
                wp.pop("filmingProperty", None)
                wp.pop("FilmingProperty", None)

    mission_ground_ref_cache: dict[tuple[tuple[float, float], ...], float | None] = {}

    def _cached_mission_ground_ref_m(info: dict) -> float | None:
        points = _collect_ref_points_from_info(info)
        if not points:
            return None
        key = tuple((round(float(lat), 7), round(float(lon), 7)) for lat, lon in points)
        if key not in mission_ground_ref_cache:
            mission_ground_ref_cache[key] = _median_ground_m(points)
        return mission_ground_ref_cache.get(key)

    # ────────────────────────── 1) 미션 → 패킷 ──────────────────────────
    seen_aircraft: set[int] = set()
    seen_area_aircraft: set[int] = set()

    mission_packet_build_started = time.perf_counter()
    for miss in missions:
        mission_build_started = time.perf_counter()
        aid = miss["aircraftID"]
        if aid not in (4, 5, 6):
            continue
        is_first_mission_for_aircraft = aid not in seen_aircraft
        seen_aircraft.add(aid)

        entry_anchor = take_over_map.get(aid) if is_first_mission_for_aircraft else None

        info = miss["individualMissionInfo"]
        mission_alt_offset_m = _aircraft_alt_offset_m(aid)
        mission_ground_ref_m: float | None = None
        mission_ground_ref_ready = False

        def _get_mission_ground_ref_m() -> float | None:
            nonlocal mission_ground_ref_m, mission_ground_ref_ready
            if not mission_ground_ref_ready:
                mission_ground_ref_m = _cached_mission_ground_ref_m(info)
                mission_ground_ref_ready = True
            return mission_ground_ref_m

        def _mission_wp_alt(lat: float, lon: float) -> int:
            return _wp_alt(
                lat,
                lon,
                mission_alt_offset_m,
                _get_mission_ground_ref_m(),
            )

        try:
            mission_pattern_type = int(info.get("patternType", 0) or 0)
        except Exception:
            mission_pattern_type = 0
        try:
            mission_sep_hint = float(info.get("SEP", 0.0) or 0.0)
        except Exception:
            mission_sep_hint = 0.0
        mission_route_offset_sep_present = False
        mission_route_offset_sep_raw = None
        if isinstance(info, dict):
            if "routeOffsetSepM" in info:
                mission_route_offset_sep_present = True
                mission_route_offset_sep_raw = info.get("routeOffsetSepM")
            elif "ROUTE_OFFSET_SEP" in info:
                mission_route_offset_sep_present = True
                mission_route_offset_sep_raw = info.get("ROUTE_OFFSET_SEP")
        try:
            mission_route_offset_sep_hint = float(mission_route_offset_sep_raw if mission_route_offset_sep_raw is not None else 0.0)
        except Exception:
            mission_route_offset_sep_hint = 0.0
        try:
            mission_fov_hint = float(info.get("FOV", 0.0) or 0.0)
        except Exception:
            mission_fov_hint = 0.0
        if runtime_manual_fov_active:
            mission_fov_hint = 0.0
        try:
            mission_speed_hint_kmh = float(info.get("SPEED", 0.0) or 0.0)
        except Exception:
            mission_speed_hint_kmh = 0.0
        mtype = info.get("individualMissionType")
        wplist: list[OrderedDict] = []
        full_sweep_coords: list[dict] | None = None
        full_sweep_speed: float | None = None
        full_sweep_interp_points: int | None = None
        is_area_mission = False
        mission_sep_m = ALT_M
        mission_fov_deg = geom_fov_deg
        mission_cruise_speed = cruise_speed
        mission_default_search_speed = DEFAULT_SEARCH_SPEED
        mission_spacing_line = sweep_spacing_m
        mission_search_speed_weight = float(LINE_SEARCH_SPEED_WEIGHT)
        mission_route_offset_sep_m = 0.0
        mission_route_offset_sep_explicit_zero = False
        mission_route_offset_fov_ref_deg = mission_fov_hint
        if mission_sep_hint > 0.0:
            mission_sep_m = mission_sep_hint
            mission_route_offset_sep_m = mission_sep_hint
        if mission_route_offset_sep_present:
            if mission_route_offset_sep_hint > 0.0:
                mission_route_offset_sep_m = mission_route_offset_sep_hint
            else:
                mission_route_offset_sep_m = 0.0
                mission_route_offset_sep_explicit_zero = True
        if mission_fov_hint > 0.0:
            mission_fov_deg = mission_fov_hint
        if mission_speed_hint_kmh > 0.0:
            mission_cruise_speed = round(mission_speed_hint_kmh / 3.6, 2)
            mission_default_search_speed = round(
                mission_cruise_speed * DEFAULT_SEARCH_SPEED_MULTIPLIER, 2
            )

        def _mission_route_offset_sep_ref_m() -> float:
            if mission_route_offset_sep_explicit_zero:
                return 0.0
            try:
                value = float(mission_route_offset_sep_m)
            except Exception:
                value = 0.0
            if value <= 0.0:
                try:
                    value = float(mission_sep_m)
                except Exception:
                    value = 0.0
            try:
                fov_ref = float(mission_route_offset_fov_ref_deg or mission_fov_hint or mission_fov_deg)
            except Exception:
                fov_ref = 0.0
            if fov_ref > 0.0:
                value = _route_offset_sep_for_fov(fov_ref, value)
            return max(float(value), 0.0)

        formation_info: OrderedDict | None = None
        formation_is_flight = False
        formation_allow_empty = False

        formation_key = _formation_key(miss)
        formation_role = _formation_role(info)
        legacy_form = False
        if formation_role is None and _is_formation_line(info):
            formation_role = "follower"
            legacy_form = True

        leader_id = formation_leaders.get(formation_key) if formation_key is not None else None
        followers_all = (formation_group_sorted.get(formation_key) or {}).get("follower") or []
        if leader_id is not None:
            if aid == leader_id:
                formation_role = "leader"
            elif formation_role is None and aid in followers_all:
                formation_role = "follower"

        if leader_id is not None and formation_role == "follower" and aid != leader_id:
            followers = [fid for fid in followers_all if fid != leader_id]
            follower_idx = followers.index(aid) if aid in followers else 0
            dx, dy, dz = formation_offsets[min(follower_idx, len(formation_offsets) - 1)]
            formation_info = OrderedDict([
                ("leaderAircraftID", int(leader_id)),
                ("formation", OrderedDict([
                    ("dX", int(dx)),
                    ("dY", int(dy)),
                    ("dZ", int(dz)),
                ])),
            ])
            formation_is_flight = True
            formation_allow_empty = True
            packet = {
                "pathID": miss["pathID"],
                "aircraftID": aid,
                "wplist": [],
                "isFormationFlight": True,
                "formationInfo": formation_info,
                "_formation_key": formation_key,
                "_formation_follower": True,
            }
            packets.append(packet)
            formation_followers.append((formation_key, packet))
            _record_dense_linesearch_metrics(
                formationFollowerBuildMs=(time.perf_counter() - mission_build_started) * 1000.0,
            )
            continue

        formation_line_leader = legacy_form and formation_key is not None and formation_leaders.get(formation_key) == aid and mtype in (3, 4, 6)

        # 1-A. 통로정찰 / 영역수색 (type 3·4·6)
        if formation_line_leader:
            line = (info.get("lineList") or [{}])[0]
            coords = line.get("coordinateList") or []
            for coord in coords:
                lat = float(coord.get("latitude", 0.0))
                lon = float(coord.get("longitude", 0.0))
                wplist.append(OrderedDict([
                    ("waypointID", 0),
                    ("coordinate", {"latitude": lat, "longitude": lon, "altitude": _mission_wp_alt(lat, lon)}),
                    ("speed", cruise_speed),
                    ("eta", 0),
                    ("ecf", 0.0),
                    ("nextWaypointID", 0),
                    ("waypointPassType", PASS_FLYBY),
                    ("filmingProperty", _mk_filming(
                        operation_mode=OPMODE_HOLD,
                        sensor=SENSOR_EO_IR,
                    )),
                ]))
        elif mtype in (3, 4, 6):
            base, width = None, 100.0
            spacing_line = sweep_spacing_m
            is_area_mission = False

            # (i) lineList → corridor
            if info.get("lineList"):
                line = info["lineList"][0]
                width = line["width"]
                base = [(c["latitude"], c["longitude"]) for c in line["coordinateList"]]
                db_cfg = None
                if not runtime_line_auto_fov_from_db:
                    try:
                        mission_fov_deg = float(runtime_line_fov_deg)
                    except Exception:
                        pass
                    if mission_sep_hint <= 0.0:
                        try:
                            mission_sep_m = float(getattr(SWEEP_GEOMETRY, "separation_m", mission_sep_m))
                        except Exception:
                            pass
                    try:
                        print(
                            f"[SWEEP][LINE-MANUAL] missionID={miss.get('individualMissionID')} "
                            f"fov={float(mission_fov_deg):.2f} sep={float(mission_sep_m):.2f}"
                        )
                    except Exception:
                        pass
                elif mtype == 6 and not (mission_sep_hint > 0.0 and mission_fov_hint > 0.0):
                    db_cfg = _select_corridor_db_config(width)
                if db_cfg:
                    mission_sep_m = float(db_cfg["sep"])
                    raw_fov_deg = float(db_cfg["fov"])
                    mission_route_offset_fov_ref_deg = raw_fov_deg
                    mission_fov_deg = _apply_runtime_adjusted_db_fov(raw_fov_deg)
                    vel = float(db_cfg.get("vel", 0.0) or 0.0)
                    if vel > 0:
                        mission_cruise_speed = _db_velocity_to_cruise_speed_mps(vel)
                        mission_default_search_speed = round(
                            mission_cruise_speed * DEFAULT_SEARCH_SPEED_MULTIPLIER, 2
                        )
                spacing_line = _sweep_spacing_m(
                    separation_m=mission_sep_m,
                    fov_deg=mission_fov_deg,
                    spacing_scale=runtime_line_spacing_scale,
                )
                _debug_sweep("LINE", separation=mission_sep_m, fov=mission_fov_deg, spacing=spacing_line)

            # (ii) areaList
            elif info.get("areaList"):
                is_area_mission = True
                mission_search_speed_weight = float(AREA_SEARCH_SPEED_WEIGHT)
                area_auto_fov_from_db = runtime_area_auto_fov_from_db
                if mission_pattern_type != 3 and (not area_auto_fov_from_db):
                    try:
                        mission_fov_deg = float(runtime_area_fov_deg)
                    except Exception:
                        pass
                if mission_pattern_type != 3 and (not area_auto_fov_from_db) and mission_sep_hint <= 0.0:
                    try:
                        mission_sep_m = float(getattr(SWEEP_GEOMETRY, "separation_m", mission_sep_m))
                    except Exception:
                        pass
                pts = [(p["latitude"], p["longitude"]) for p in info["areaList"][0]["coordinateList"]]

                area_sweep_mode = runtime_area_sweep_mode
                if area_sweep_mode == "vertical":
                    sweep_bearing_raw = info.get("BEARING")
                    move_bearing_raw = (
                        info.get("MOVE_BEARING")
                        if info.get("MOVE_BEARING") is not None
                        else (
                            miss.get("phaseMoveBearing_deg")
                            if miss.get("phaseMoveBearing_deg") is not None
                            else miss.get("move_bearing_deg")
                        )
                    )
                    if sweep_bearing_raw is None and move_bearing_raw is not None:
                        try:
                            sweep_bearing_raw = (float(move_bearing_raw) + 90.0) % 360.0
                        except Exception:
                            sweep_bearing_raw = None
                    if sweep_bearing_raw is None:
                        sweep_bearing_raw = miss.get("splitBearing_deg")
                    if sweep_bearing_raw is None:
                        sweep_bearing_raw = miss.get("bearing_deg")
                    if sweep_bearing_raw is None:
                        sweep_bearing_raw = 180.0
                else:
                    sweep_bearing_raw = (
                        miss.get("phaseMoveBearing_deg")
                        if miss.get("phaseMoveBearing_deg") is not None
                        else miss.get("move_bearing_deg")
                    )
                    if sweep_bearing_raw is None:
                        sweep_bearing_raw = info.get("MOVE_BEARING")
                    if sweep_bearing_raw is None:
                        sweep_bearing_raw = info.get("BEARING")
                    if sweep_bearing_raw is None:
                        sweep_bearing_raw = miss.get("bearing_deg", 90.0)
                try:
                    bearing = float(sweep_bearing_raw)
                except Exception:
                    bearing = 90.0
                th = math.radians(bearing)
                sweep_nx, sweep_ny = math.cos(th), -math.sin(th)
                area_db_meta = None
                if mission_pattern_type != 3 and runtime_area_auto_fov_from_db:
                    area_db_meta = _select_area_db_config(pts, bearing)
                    if area_db_meta:
                        db_cfg = area_db_meta["config"]
                        mission_sep_m = float(db_cfg.get("sep", mission_sep_m) or mission_sep_m)
                        raw_area_fov_deg = float(db_cfg.get("fov", mission_fov_deg) or mission_fov_deg)
                        mission_route_offset_fov_ref_deg = raw_area_fov_deg
                        mission_fov_deg = _apply_runtime_adjusted_db_fov(raw_area_fov_deg)
                        try:
                            print(
                                f"[SWEEP][AREA-DB] missionID={miss.get('individualMissionID')} "
                                f"widthRef={area_db_meta['width_ref_m']:.2f}m "
                                f"dbWidth={float(db_cfg.get('width', 0.0) or 0.0):.2f}m "
                                f"fov={mission_fov_deg:.2f} sep={mission_sep_m:.2f}"
                            )
                        except Exception:
                            pass
                elif mission_pattern_type != 3:
                    try:
                        print(
                            f"[SWEEP][AREA-MANUAL] missionID={miss.get('individualMissionID')} "
                            f"fov={float(mission_fov_deg):.2f} sep={float(mission_sep_m):.2f}"
                        )
                    except Exception:
                        pass

                prev_pt = miss.get("prevPoint", pts[0])

                strip_spacing = mission_sep_m
                # patternType 3: area는 lineSearch 없이 직하방 고정 촬영으로 계획
                if mission_pattern_type == 3:
                    nadir_sep_ref = float(mission_sep_hint) if mission_sep_hint > 0.0 else float(mission_alt_offset_m)
                    if area_auto_fov_from_db:
                        nadir_fov = (
                            float(mission_fov_hint)
                            if mission_fov_hint > 0.0
                            else _select_nadir_fov_by_altitude(nadir_sep_ref, float(runtime_nadir_fov_deg))
                        )
                    else:
                        nadir_fov = float(runtime_nadir_fov_deg)
                    mission_route_offset_fov_ref_deg = float(nadir_fov)
                    strip_spacing = _sweep_spacing_m(
                        separation_m=nadir_sep_ref,
                        fov_deg=nadir_fov,
                        spacing_scale=runtime_area_spacing_scale,
                    )
                    _debug_sweep(
                        "AREA-NADIR",
                        separation=nadir_sep_ref,
                        fov=nadir_fov,
                        spacing=strip_spacing,
                    )
                    if build_nadir_bf_overflight_coords is None:
                        raise RuntimeError("BF nadir planner import failed (patternType=3 requires BF-only planner).")

                    nadir_coords, runtime_fov = build_nadir_bf_overflight_coords(
                        polygon_llh=pts,
                        anchor_llh=prev_pt,
                        bearing_deg=bearing,
                        separation_m=nadir_sep_ref,
                        fov_deg=nadir_fov,
                        min_segment_m=MIN_SWEEP_LEN_M,
                        spacing_margin=float(SWEEP_SPACING_MARGIN) * runtime_area_spacing_scale,
                        altitude_fn=lambda lat, lon: _mission_wp_alt(lat, lon),
                        cruise_alt_m=float(mission_alt_offset_m),
                        r_min=200.0,
                        goal_length_m=300.0,
                    )
                    if runtime_fov and runtime_fov > 0:
                        nadir_fov = float(runtime_fov)
                        mission_route_offset_fov_ref_deg = float(nadir_fov)
                    if not nadir_coords:
                        raise RuntimeError(
                            f"BF nadir planner returned empty path (missionID={miss.get('individualMissionID')})."
                        )
                    print(
                        f"[SWEEP][AREA-NADIR][bf-only] missionID={miss.get('individualMissionID')} "
                        f"points={len(nadir_coords)} fov={nadir_fov:.1f}"
                    )

                    for point in nadir_coords:
                        lat = float(point.get("latitude", 0.0))
                        lon = float(point.get("longitude", 0.0))
                        alt = int(point.get("altitude", _mission_wp_alt(lat, lon)))
                        wplist.append(OrderedDict([
                            ("waypointID", 0),
                            ("coordinate", {
                                "latitude": lat,
                                "longitude": lon,
                                "altitude": alt,
                            }),
                            ("speed", mission_cruise_speed),
                            ("eta", 2500),
                            ("ecf", 0.0),
                            ("nextWaypointID", 0),
                            ("waypointPassType", PASS_FLYOVER),
                            ("filmingProperty", _mk_filming(
                                operation_mode=OPMODE_HOLD,
                                sensor=SENSOR_EO_IR,
                                fov=nadir_fov,
                            )),
                        ]))
                else:
                    spacing_line = _sweep_spacing_m(
                        separation_m=mission_sep_m,
                        fov_deg=mission_fov_deg,
                        spacing_scale=runtime_area_spacing_scale,
                    )
                    mission_spacing_line = spacing_line
                    strip_spacing = spacing_line
                    _debug_sweep("AREA", separation=mission_sep_m, fov=mission_fov_deg, spacing=spacing_line)
                    banded_coords = [_poly_sweeps_general(
                        poly_llh=pts,
                        anchor_llh=prev_pt,
                        bearing_deg=bearing,
                        fov_deg=mission_fov_deg,
                        separation_m=mission_sep_m,
                        spacing_scale=runtime_area_spacing_scale,
                    )]
                    if not any(coord_list for coord_list in banded_coords):
                        fallback_coords = _fallback_poly_long_axis_sweep(pts)
                        if fallback_coords:
                            banded_coords = [fallback_coords]
                    if not banded_coords:
                        continue

                    line_counter = 0
                    last_off_xy: tuple[float, float] | None = None
                    lat0, lon0 = pts[0]

                    for band_idx, coord_list in enumerate(banded_coords):
                        lines = [coord_list[i:i+2] for i in range(0, len(coord_list), 2)]
                        if not lines:
                            continue
                        # 짧은 스윕 필터
                        MIN_SWEEP_LEN = MIN_SWEEP_LEN_M
                        filtered = []
                        lat0, lon0 = lines[0][0]["latitude"], lines[0][0]["longitude"]
                        for ln in lines:
                            s_lat, s_lon = ln[0]["latitude"], ln[0]["longitude"]
                            e_lat, e_lon = ln[1]["latitude"], ln[1]["longitude"]
                            dx = (e_lon - s_lon) * 111_132 * math.cos(math.radians((s_lat + e_lat) / 2))
                            dy = (e_lat - s_lat) * 111_132
                            if math.hypot(dx, dy) >= MIN_SWEEP_LEN:
                                filtered.append(ln)
                        lines = filtered
                        if not lines:
                            fallback_coords = _fallback_poly_long_axis_sweep(pts)
                            if len(fallback_coords) >= 2:
                                lines = [fallback_coords[:2]]
                            else:
                                continue
                        lat0, lon0 = lines[0][0]["latitude"], lines[0][0]["longitude"]

                        for ln in lines:
                            idx = line_counter
                            line_counter += 1
                            s, e = ln
                            s_xy = llh_to_xy(s['latitude'], s['longitude'], lat0, lon0)
                            e_xy = llh_to_xy(e['latitude'], e['longitude'], lat0, lon0)

                            mid_xy = ((s_xy[0] + e_xy[0]) / 2, (s_xy[1] + e_xy[1]) / 2)
                            off_xy = (
                                mid_xy[0] - sweep_nx * strip_spacing,
                                mid_xy[1] - sweep_ny * strip_spacing,
                            )
                            last_off_xy = off_xy
                            off_lat, off_lon = xy_to_llh(*off_xy, lat0, lon0)

                            sweep = [e, s] if idx % 2 else [s, e]
                            sweep_width = _avg_sweep_width_m(sweep)
                            sweep_interp_points = _interp_points_hint(SWEEP_LINE_INTERP_POINTS) or 2
                            sweep_speed = spacing_based_search_speed(
                                sweep_len_m=sweep_width,
                                spacing_m=spacing_line,
                                cruise_speed_mps=mission_cruise_speed,
                                weight=mission_search_speed_weight,
                            )
                            if sweep_speed is None:
                                sweep_speed = mission_default_search_speed

                            wplist.append(OrderedDict([
                                ("waypointID", 0),
                                ("_path_search_speed_weight", float(mission_search_speed_weight)),
                                ("coordinate", {"latitude": off_lat, "longitude": off_lon, "altitude": _mission_wp_alt(off_lat, off_lon)}),
                                ("speed", mission_cruise_speed),
                                ("eta", 2500),
                                ("ecf", 0.0),
                                ("nextWaypointID", 0),
                                ("waypointPassType", PASS_FLYBY),
                                ("filmingProperty", _mk_filming(
                                    operation_mode=OPMODE_LINE,
                                    fov=mission_fov_deg,
                                    sensor=SENSOR_EO_IR,
                                    line_search=OrderedDict([
                                        ("coordinateList", _interpolate_line_coords(sweep, sweep_interp_points)),
                                        ("searchSpeed", sweep_speed),
                                        ("interpolationPoints", sweep_interp_points),
                                    ]),
                                )),
                            ]))

                # ➌ 종료 WP (Loiter + POINT 촬영)
            # (iii) Corridor-planner
            if base and len(base) >= 2:
                active_corridor_db_cfg = (
                    dict(db_cfg)
                    if isinstance(db_cfg, dict)
                    else (_select_corridor_db_config(width) or {})
                )

                def _apply_corridor_db_cfg(next_cfg: dict) -> None:
                    nonlocal mission_sep_m, mission_fov_deg, mission_cruise_speed
                    nonlocal mission_default_search_speed, spacing_line, mission_spacing_line
                    nonlocal mission_route_offset_fov_ref_deg
                    mission_sep_m = float(next_cfg["sep"])
                    raw_fov_deg = float(next_cfg["fov"])
                    mission_route_offset_fov_ref_deg = raw_fov_deg
                    mission_fov_deg = _apply_runtime_adjusted_db_fov(raw_fov_deg)
                    vel = float(next_cfg.get("vel", 0.0) or 0.0)
                    if vel > 0:
                        mission_cruise_speed = _db_velocity_to_cruise_speed_mps(vel)
                        mission_default_search_speed = round(
                            mission_cruise_speed * DEFAULT_SEARCH_SPEED_MULTIPLIER,
                            2,
                        )
                    spacing_line = _sweep_spacing_m(
                        separation_m=mission_sep_m,
                        fov_deg=mission_fov_deg,
                        spacing_scale=runtime_line_spacing_scale,
                    )
                    mission_spacing_line = spacing_line

                try:
                    line_route_offset_scale_for_db = max(
                        float(_runtime_float_setting("line_route_offset_scale", LINE_ROUTE_OFFSET_SCALE)),
                        0.0,
                    )
                except Exception:
                    line_route_offset_scale_for_db = 1.0

                planner = None
                quality_sep_limit_reference_m: float | None = None
                quality_db_check_enabled = (
                    bool(runtime_line_auto_fov_from_db)
                    and mtype == 6
                    and not (mission_sep_hint > 0.0 and mission_fov_hint > 0.0)
                )
                for _quality_db_attempt in range(3):
                    planner = UAVMissionPlanner(
                        base, corridor_width=width, separation=mission_sep_m,
                        fov_deg=mission_fov_deg, sweep_spacing_m=spacing_line,
                        cruise_speed=mission_cruise_speed, simulate_path=False, crs="lla",
                    )
                    if (
                        not quality_db_check_enabled
                        or _runtime_uav_plan_mode() != "dub_path"
                    ):
                        break
                    if quality_sep_limit_reference_m is None:
                        quality_sep_limit_reference_m = _corridor_first_anchor_quality_sep_m(
                            planner,
                            route_offset_m=float(_mission_route_offset_sep_ref_m()) * float(line_route_offset_scale_for_db),
                        )
                        if quality_sep_limit_reference_m <= 0.0:
                            break
                        quality_sep_limit_reference_m += float(FOV_DB_QUALITY_SEP_MARGIN_M)
                    quality_sep_limit_m = float(quality_sep_limit_reference_m)
                    conservative_sep_limit_m = _db_sep_requirement_m(quality_sep_limit_m)
                    current_db_sep_m = float(active_corridor_db_cfg.get("sep", mission_sep_m) or mission_sep_m or 0.0)
                    if current_db_sep_m + 1e-6 >= conservative_sep_limit_m:
                        break
                    quality_db_cfg = _select_corridor_db_config(
                        width,
                        min_sep_m=float(quality_sep_limit_m),
                    )
                    if not isinstance(quality_db_cfg, dict):
                        break
                    if float(quality_db_cfg.get("sep", 0.0) or 0.0) + 1e-6 < conservative_sep_limit_m:
                        break
                    next_sig = (
                        round(float(quality_db_cfg.get("sep", 0.0) or 0.0), 6),
                        round(float(quality_db_cfg.get("width", 0.0) or 0.0), 6),
                        round(float(quality_db_cfg.get("fov", 0.0) or 0.0), 6),
                    )
                    cur_sig = (
                        round(float(active_corridor_db_cfg.get("sep", 0.0) or 0.0), 6),
                        round(float(active_corridor_db_cfg.get("width", 0.0) or 0.0), 6),
                        round(float(active_corridor_db_cfg.get("fov", 0.0) or 0.0), 6),
                    )
                    if next_sig == cur_sig:
                        break
                    active_corridor_db_cfg = dict(quality_db_cfg)
                    _apply_corridor_db_cfg(active_corridor_db_cfg)
                if planner is None:
                    planner = UAVMissionPlanner(
                        base, corridor_width=width, separation=mission_sep_m,
                        fov_deg=mission_fov_deg, sweep_spacing_m=spacing_line,
                        cruise_speed=mission_cruise_speed, simulate_path=False, crs="lla",
                    )
                use_centerline = True

                def _oriented_sweep_points(
                    sw_idx: int,
                    sw: tuple[tuple[float, float], tuple[float, float]],
                ) -> tuple[tuple[float, float], tuple[float, float]]:
                    s_xy, e_xy = sw
                    if int(sw_idx) % 2:
                        s_xy, e_xy = e_xy, s_xy
                    s_lat, s_lon = planner._proj_back(s_xy[0], s_xy[1])[::-1]
                    e_lat, e_lon = planner._proj_back(e_xy[0], e_xy[1])[::-1]
                    return (float(s_lat), float(s_lon)), (float(e_lat), float(e_lon))

                def _precompute_sweep_coord_pairs() -> dict[int, list[dict]]:
                    sweep_rows = list(enumerate(planner.sweeps))
                    if not sweep_rows:
                        return {}

                    def _oriented_rows(
                        rows: list[tuple[int, tuple[tuple[float, float], tuple[float, float]]]]
                    ) -> list[tuple[int, tuple[float, float], tuple[float, float]]]:
                        out: list[tuple[int, tuple[float, float], tuple[float, float]]] = []
                        for sw_idx, sw in rows:
                            try:
                                s_ll, e_ll = _oriented_sweep_points(sw_idx, sw)
                            except Exception:
                                continue
                            out.append((int(sw_idx), s_ll, e_ll))
                        return out

                    estimated_coord_count = len(sweep_rows) * max(_interp_points_hint(SWEEP_LINE_INTERP_POINTS) or 2, 2)
                    workers = _linesearch_inner_parallel_workers(
                        len(sweep_rows),
                        coordinate_count=estimated_coord_count,
                    )
                    oriented_rows: list[tuple[int, tuple[float, float], tuple[float, float]]] = []
                    if workers > 1 and len(sweep_rows) > 1:
                        chunks = _chunk_sequence(sweep_rows, workers)
                        _record_dense_linesearch_metrics(
                            innerParallelWorkers=workers,
                            innerParallelBatches=len(chunks),
                        )
                        with concurrent.futures.ThreadPoolExecutor(
                            max_workers=min(workers, len(chunks)),
                            thread_name_prefix="LineSearchCorridor",
                        ) as executor:
                            futures = [executor.submit(_oriented_rows, chunk) for chunk in chunks]
                            for future in futures:
                                oriented_rows.extend(future.result())
                    else:
                        oriented_rows.extend(_oriented_rows(sweep_rows))

                    endpoint_points: list[tuple[float, float]] = []
                    ordered_indices: list[int] = []
                    for sw_idx, s_ll, e_ll in oriented_rows:
                        try:
                            ordered_indices.append(int(sw_idx))
                            endpoint_points.extend([s_ll, e_ll])
                        except Exception:
                            continue
                    if not endpoint_points:
                        return {}
                    materialized = _materialize_dem_coords(endpoint_points)
                    if len(materialized) != len(endpoint_points):
                        return {}
                    pairs_by_index: dict[int, tuple[dict, dict]] = {}
                    offset = 0
                    for sw_idx in ordered_indices:
                        pairs_by_index[int(sw_idx)] = (
                            materialized[offset],
                            materialized[offset + 1],
                        )
                        offset += 2
                    return pairs_by_index

                sweep_coord_pairs_by_index = _precompute_sweep_coord_pairs()

                def _sweep_coord_pair(sw_idx: int, sw: tuple[tuple[float, float], tuple[float, float]]) -> list[dict]:
                    cached_pair = sweep_coord_pairs_by_index.get(int(sw_idx))
                    if cached_pair:
                        return list(cached_pair)
                    return _materialize_dem_coords(list(_oriented_sweep_points(sw_idx, sw)))

                if use_centerline:
                    base_xy: list[tuple[float, float]] = []
                    proj_fwd = getattr(planner, "_proj_fwd", None)
                    if proj_fwd is not None:
                        for lat, lon in base:
                            x, y = proj_fwd(lon, lat)
                            base_xy.append((x, y))
                    else:
                        lat0, lon0 = base[0]
                        base_xy = [llh_to_xy(lat, lon, lat0, lon0) for lat, lon in base]

                    sweep_mid_xy: list[tuple[float, float]] = []
                    for sw in planner.sweeps:
                        sweep_mid_xy.append(((sw[0][0] + sw[1][0]) / 2, (sw[0][1] + sw[1][1]) / 2))

                    def _select_sweep_indices(
                        points_xy: list[tuple[float, float]],
                        sweep_midpoints: list[tuple[float, float]],
                    ) -> list[int]:
                        if not points_xy or not sweep_midpoints:
                            return []
                        total_pts = len(points_xy)
                        total_sweeps = len(sweep_midpoints)
                        if total_sweeps <= total_pts:
                            return list(range(total_sweeps))
                        selected: list[int] = []
                        start_idx = 0
                        for pos, pt in enumerate(points_xy):
                            remaining = total_pts - pos - 1
                            max_idx = total_sweeps - 1 - remaining
                            if max_idx < start_idx:
                                max_idx = start_idx
                            best_idx = start_idx
                            best_dist = None
                            for idx in range(start_idx, max_idx + 1):
                                dx = sweep_midpoints[idx][0] - pt[0]
                                dy = sweep_midpoints[idx][1] - pt[1]
                                dist = dx * dx + dy * dy
                                if best_dist is None or dist < best_dist:
                                    best_dist = dist
                                    best_idx = idx
                            selected.append(best_idx)
                            start_idx = best_idx + 1
                        return selected

                    sampled_base_xy = _resample_polyline_xy(base_xy, runtime_line_route_spacing_m)
                    sweep_indices = _select_sweep_indices(sampled_base_xy, sweep_mid_xy)
                    if not sweep_indices:
                        sweep_indices = list(range(len(planner.sweeps)))
                    anchor_list = planner.offset_wps

                    merged_coords: list[dict] = []
                    merged_sweep_speeds: list[float] = []
                    sweep_speed_by_index: dict[int, float] = {}
                    sweep_width_by_index: dict[int, float | None] = {}
                    for sw_idx, sw in enumerate(planner.sweeps):
                        sweep_coords = _sweep_coord_pair(sw_idx, sw)
                        merged_coords.extend(sweep_coords)
                        sweep_width = _avg_sweep_width_m(sweep_coords)
                        sweep_width_by_index[int(sw_idx)] = sweep_width
                        sweep_speed = spacing_based_search_speed(
                            sweep_len_m=sweep_width,
                            spacing_m=spacing_line,
                            cruise_speed_mps=mission_cruise_speed,
                            weight=mission_search_speed_weight,
                        )
                        if sweep_speed is None:
                            sweep_speed = mission_default_search_speed
                        sweep_speed = float(sweep_speed)
                        sweep_speed_by_index[int(sw_idx)] = sweep_speed
                        merged_sweep_speeds.append(sweep_speed)
                    if merged_coords:
                        full_sweep_coords = merged_coords
                        full_sweep_interp_points = _interp_points_hint(SWEEP_LINE_INTERP_POINTS) or 2
                        if _runtime_uav_plan_mode() == "dub_path":
                            if merged_sweep_speeds:
                                full_sweep_speed = round(
                                    sum(merged_sweep_speeds) / max(len(merged_sweep_speeds), 1),
                                    2,
                                )
                        if full_sweep_speed is None:
                            merged_width = _avg_sweep_width_m(
                                merged_coords,
                                points_per_line=2,
                            )
                            full_sweep_speed = spacing_based_search_speed(
                                sweep_len_m=merged_width,
                                spacing_m=spacing_line,
                                cruise_speed_mps=mission_cruise_speed,
                                weight=mission_search_speed_weight,
                            )
                        if full_sweep_speed is None:
                            full_sweep_speed = mission_default_search_speed
                else:
                    sweep_indices = list(range(len(planner.sweeps)))
                    anchor_list = planner.orange_pts

                last_anchor_xy: tuple[float, float] | None = None
                last_first_xy: tuple[float, float] | None = None
                try:
                    line_route_offset_scale = max(
                        float(_runtime_float_setting("line_route_offset_scale", LINE_ROUTE_OFFSET_SCALE)),
                        0.0,
                    )
                except Exception:
                    line_route_offset_scale = 1.0
                for idx in sweep_indices:
                    if idx >= len(planner.sweeps) or idx >= len(anchor_list):
                        continue
                    sw = planner.sweeps[idx]
                    anchor_xy = anchor_list[idx]
                    if use_centerline:
                        anchor_xy = _line_wp_xy_from_sweep(
                            sw,
                            offset_m=float(_mission_route_offset_sep_ref_m()) * float(line_route_offset_scale),
                        )

                    s_xy, e_xy = sw
                    if idx % 2:
                        s_xy, e_xy = e_xy, s_xy
                    w_lat, w_lon = planner._proj_back(anchor_xy[0], anchor_xy[1])[::-1]

                    coord_list = _sweep_coord_pair(idx, sw)

                    sweep_interp_points = _interp_points_hint(SWEEP_LINE_INTERP_POINTS) or 2
                    coord_speed = sweep_speed_by_index.get(int(idx))
                    if coord_speed is None:
                        coord_width = sweep_width_by_index.get(int(idx))
                        if coord_width is None:
                            coord_width = _avg_sweep_width_m(coord_list)
                        coord_speed = spacing_based_search_speed(
                            sweep_len_m=coord_width,
                            spacing_m=spacing_line,
                            cruise_speed_mps=mission_cruise_speed,
                            weight=mission_search_speed_weight,
                        )
                        if coord_speed is None:
                            coord_speed = mission_default_search_speed

                    wplist.append(OrderedDict([
                        ("waypointID", 0),
                        ("_path_search_speed_weight", float(mission_search_speed_weight)),
                        ("coordinate", {"latitude": w_lat, "longitude": w_lon, "altitude": _mission_wp_alt(w_lat, w_lon)}),
                        ("speed", cruise_speed),
                        ("eta", 2500),
                        ("ecf", 0.0),
                        ("nextWaypointID", 0),
                        ("waypointPassType", PASS_FLYBY),
                        ("filmingProperty", _mk_filming(
                            operation_mode=OPMODE_LINE,
                            fov=mission_fov_deg,
                            sensor=SENSOR_EO_IR,
                            line_search=OrderedDict([
                                ("coordinateList", _interpolate_line_coords(coord_list, sweep_interp_points)),
                                ("searchSpeed", coord_speed),
                                ("interpolationPoints", sweep_interp_points),
                            ]),
                        )),
                        ("_sweepIdx", int(idx)),
                    ]))

                    last_anchor_xy = anchor_xy
                    last_first_xy = s_xy

        # 1-B. 좌표점정찰 / hold 미션 (type 5)
        elif mtype == 5:
            base: list[tuple[float, float]] = []
            if info.get("coordinateList"):
                base = [(c["latitude"], c["longitude"]) for c in info["coordinateList"]]
            elif info.get("lineList"):
                base = [(c["latitude"], c["longitude"]) for c in info["lineList"][0]["coordinateList"]]

            try:
                hold_req_s = float(info.get("holdingReqTime", 0.0) or 0.0)
            except Exception:
                hold_req_s = 0.0
            pass_type = PASS_LOITER if hold_req_s > 0.05 else PASS_FLYBY

            if len(base) == 1:
                lat, lon = base[0]
                coord = {"latitude": lat, "longitude": lon, "altitude": _mission_wp_alt(lat, lon)}
                wplist.append(OrderedDict([
                    ("waypointID", 0),
                    ("coordinate", coord),
                    ("speed", mission_cruise_speed),
                    ("eta", 0),
                    ("ecf", 1.0),
                    ("nextWaypointID", 0),
                    ("waypointPassType", pass_type),
                    ("filmingProperty", _mk_point_filming_for_coord(coord)),
                ]))
            elif len(base) >= 2:
                raw_pts = _plan_route_points(
                    base,
                    cruise_speed=mission_cruise_speed,
                    heading_tol_deg=turn_step_deg,
                )
                simp: list[dict] = [raw_pts[0]]
                for p in raw_pts[1:]:
                    d = math.hypot(
                        (p["lon"] - simp[-1]["lon"]) * DEG_M * math.cos(
                            math.radians((p["lat"] + simp[-1]["lat"]) / 2)),
                        (p["lat"] - simp[-1]["lat"]) * DEG_M
                    )
                    if d >= MIN_ROUTE_SPACING_M or p is raw_pts[-1]:
                        simp.append(p)

                for idx, p in enumerate(simp):
                    coord = {
                        "latitude": p["lat"],
                        "longitude": p["lon"],
                        "altitude": _mission_wp_alt(p["lat"], p["lon"]),
                    }
                    is_last = idx == len(simp) - 1
                    wplist.append(OrderedDict([
                        ("waypointID", 0),
                        ("coordinate", coord),
                        ("speed", mission_cruise_speed),
                        ("eta", p["eta_ms"]),
                        ("ecf", 0.0),
                        ("nextWaypointID", 0),
                        ("waypointPassType", pass_type if is_last else PASS_FLYBY),
                        ("filmingProperty", _mk_point_filming_for_coord(coord) if is_last else {}),
                    ]))

        # 1-C. 이동 미션 (type 7)
        elif mtype == 7:
            base: list[tuple[float, float]] = []
            if info.get("coordinateList"):
                base = [(c["latitude"], c["longitude"]) for c in info["coordinateList"]]
            elif info.get("lineList"):
                base = [(c["latitude"], c["longitude"]) for c in info["lineList"][0]["coordinateList"]]

            if len(base) == 1:
                lat, lon = base[0]
                wplist.append(OrderedDict([
                    ("waypointID", 0),
                    ("coordinate", {"latitude": lat, "longitude": lon, "altitude": _mission_wp_alt(lat, lon)}),
                    ("speed", cruise_speed), ("eta", 0), ("ecf", 1.0),
                    ("nextWaypointID", 0), ("waypointPassType", 1),
                    ("filmingProperty", {}),
                ]))
            elif len(base) >= 2:
                raw_pts = _plan_route_points(
                    base,
                    cruise_speed=cruise_speed,
                    heading_tol_deg=turn_step_deg,
                )
                MIN_SPACING_M = MIN_ROUTE_SPACING_M
                simp: list[dict] = [raw_pts[0]]
                for p in raw_pts[1:]:
                    d = math.hypot(
                        (p["lon"] - simp[-1]["lon"]) * DEG_M * math.cos(
                            math.radians((p["lat"] + simp[-1]["lat"]) / 2)),
                        (p["lat"] - simp[-1]["lat"]) * DEG_M
                    )
                    if d >= MIN_SPACING_M or p is raw_pts[-1]:
                        simp.append(p)

                for p in simp:
                    wplist.append(OrderedDict([
                        ("waypointID", 0),
                        ("coordinate", {"latitude": p["lat"], "longitude": p["lon"], "altitude": _mission_wp_alt(p["lat"], p["lon"])}),
                        ("speed", cruise_speed),
                        ("eta", p["eta_ms"]),
                        ("ecf", 0.0),
                        ("nextWaypointID", 0),
                        ("waypointPassType", 1),
                        ("filmingProperty", _mk_filming(
                            operation_mode=OPMODE_HOLD,
                            sensor=SENSOR_EO_IR
                        )),
                    ]))

        if leader_id is not None and aid == leader_id:
            formation_info = OrderedDict([
                ("leaderAircraftID", int(leader_id)),
                ("formation", OrderedDict([
                    ("dX", 0),
                    ("dY", 0),
                    ("dZ", 0),
                ])),
            ])
            formation_is_flight = True
            if wplist:
                _strip_filming_properties(wplist)

        skip_area_entry_prefix = False
        if mtype in (3, 4, 6) and is_area_mission:
            skip_area_entry_prefix = aid not in seen_area_aircraft
            seen_area_aircraft.add(aid)

        # 1-D. 패킷 저장
        if wplist or formation_allow_empty:
            packet = {
                "pathID": miss["pathID"],
                "aircraftID": aid,
                "wplist": wplist,
                "_mission_sep_m": float(mission_sep_m),
                "_mission_route_offset_sep_m": float(_mission_route_offset_sep_ref_m()),
                "_mission_route_offset_sep_explicit_zero": bool(mission_route_offset_sep_explicit_zero),
                "_mission_route_offset_fov_ref_deg": float(
                    mission_route_offset_fov_ref_deg or mission_fov_hint or mission_fov_deg or 0.0
                ),
                "_mission_cruise_speed_mps": float(mission_cruise_speed),
                "_mission_default_search_speed": float(mission_default_search_speed),
                "_mission_spacing_line_m": float(mission_spacing_line),
                "_mission_search_speed_weight": float(mission_search_speed_weight),
                "_mission_alt_offset_m": float(mission_alt_offset_m),
                "_mission_ground_ref_m": _get_mission_ground_ref_m(),
                "_entry_anchor": entry_anchor,
                "_skip_area_entry_prefix": bool(skip_area_entry_prefix),
            }
            if isinstance(miss.get("prevPoint"), dict):
                packet["_prev_point"] = deepcopy(miss.get("prevPoint"))
            if formation_info:
                packet["formationInfo"] = formation_info
                packet["isFormationFlight"] = bool(formation_is_flight)
                packet["_formation_key"] = formation_key
            if mtype in (3, 4, 6) and is_area_mission:
                packet["_is_area"] = True
                packet["_area_sweep_mode"] = str(area_sweep_mode or "parallel")
                packet["_area_bearing_deg"] = float(bearing)
                if mission_pattern_type == 3:
                    packet["_area_nadir"] = True
            if full_sweep_coords:
                packet["fullSweepCoords"] = full_sweep_coords
                packet["fullSweepSearchSpeed"] = full_sweep_speed
                packet["fullSweepInterpPoints"] = full_sweep_interp_points
            packets.append(packet)
            if formation_key is not None and leader_id is not None and aid == leader_id:
                _record_dense_linesearch_metrics(
                    formationLeaderBuildMs=(time.perf_counter() - mission_build_started) * 1000.0,
                )

    if formation_followers:
        formation_follower_started = time.perf_counter()
        leader_wplist_by_key: dict[int, list[OrderedDict]] = {}
        for pkt in packets:
            fkey = pkt.get("_formation_key")
            if fkey is None:
                continue
            leader_id = formation_leaders.get(fkey)
            if leader_id is None:
                continue
            if int(pkt.get("aircraftID", 0)) == int(leader_id) and pkt.get("wplist"):
                leader_wplist_by_key[int(fkey)] = pkt["wplist"]
        for fkey, fpkt in formation_followers:
            if leader_wplist_by_key.get(int(fkey)):
                fpkt["_formation_initial_clone_skipped"] = True
        _record_dense_linesearch_metrics(
            formationFollowerBuildMs=(time.perf_counter() - formation_follower_started) * 1000.0,
        )
    _record_dense_linesearch_metrics(
        missionPacketBuildMs=(time.perf_counter() - mission_packet_build_started) * 1000.0,
    )

    # ────────────────────────── 3) WP ID · 링크 · ECF ────────────────
    prev_tail_by_aircraft: Dict[int, dict] = {}
    prev_last_wp_coord_by_aircraft: Dict[int, dict] = {}
    prev_heading_by_aircraft: Dict[int, float] = {}
    prev_area_offset_sign_by_aircraft: Dict[int, int] = {}
    prev_area_pkt_by_aircraft: Dict[int, dict] = {}
    ground_required_precompute_cache: dict[tuple, float] = {}
    _prewarm_ground_required_process_cache_for_packets(
        packets,
        signature_cache=ground_required_precompute_cache,
        runtime_payload=runtime_payload,
    )

    def _packet_route_offset_sep_m(pkt: dict) -> float:
        if bool(pkt.get("_mission_route_offset_sep_explicit_zero")):
            return 0.0
        try:
            value = float(pkt.get("_mission_route_offset_sep_m", 0.0) or 0.0)
        except Exception:
            value = 0.0
        if value <= 0.0:
            try:
                value = float(pkt.get("_mission_sep_m", 0.0) or 0.0)
            except Exception:
                value = 0.0
        try:
            fov_ref = float(pkt.get("_mission_route_offset_fov_ref_deg", 0.0) or 0.0)
        except Exception:
            fov_ref = 0.0
        if fov_ref > 0.0:
            value = _route_offset_sep_for_fov(fov_ref, value)
        return max(float(value), 0.0)

    for pkt in packets:
        wps = pkt["wplist"]
        aid = int(pkt.get("aircraftID", 0))
        prev_tail_coord = prev_tail_by_aircraft.get(aid)
        prev_last_wp_coord = prev_last_wp_coord_by_aircraft.get(aid)
        entry_anchor = pkt.get("_entry_anchor") if isinstance(pkt.get("_entry_anchor"), dict) else None
        if pkt.get("_formation_follower"):
            continue
        if not wps:
            prev_area_pkt_by_aircraft.pop(aid, None)
            continue
        try:
            packet_cruise_speed = float(pkt.get("_mission_cruise_speed_mps", cruise_speed) or cruise_speed)
        except Exception:
            packet_cruise_speed = float(cruise_speed)
        try:
            packet_default_search_speed = float(
                pkt.get("_mission_default_search_speed", DEFAULT_SEARCH_SPEED) or DEFAULT_SEARCH_SPEED
            )
        except Exception:
            packet_default_search_speed = float(DEFAULT_SEARCH_SPEED)
        try:
            packet_search_speed_weight = float(pkt.get("_mission_search_speed_weight", 1.0) or 1.0)
        except Exception:
            packet_search_speed_weight = 1.0
        try:
            packet_spacing_line = float(pkt.get("_mission_spacing_line_m", sweep_spacing_m) or sweep_spacing_m)
        except Exception:
            packet_spacing_line = float(sweep_spacing_m)

        is_area_pkt = bool(pkt.get("_is_area"))
        is_area_nadir = bool(pkt.get("_area_nadir"))
        area_sweep_mode = str(pkt.get("_area_sweep_mode") or "parallel").strip().lower()
        uav_plan_mode = _runtime_uav_plan_mode()
        sweep_indices = [idx for idx, wp in enumerate(wps) if _has_line_search(wp)]
        entry_wp: OrderedDict | None = None
        merge_mode = SWEEP_MERGE_MODE
        merge_heading_deg = AREA_SWEEP_MERGE_HEADING_DEG if is_area_pkt else SWEEP_MERGE_HEADING_DEG
        if is_area_pkt and not is_area_nadir:
            merge_mode = "none"
            # Preserve one route waypoint per sweep line for area missions.
        line_merge_started = time.perf_counter()
        if len(sweep_indices) >= 2:
            first_idx = sweep_indices[0]

            has_existing_preline_prefix = bool(first_idx > 0)
            initial_first_wp = wps[first_idx]
            initial_first_fp = initial_first_wp.get("filmingProperty") or OrderedDict()
            initial_first_line_search = initial_first_fp.get("lineSearch") or {}
            initial_first_coords = _copy_coord_list(initial_first_line_search.get("coordinateList") or [])
            initial_first_search_speed = initial_first_line_search.get("searchSpeed")
            initial_first_interp_points = _interp_points_hint(initial_first_line_search.get("interpolationPoints"))
            if initial_first_search_speed is None:
                first_width = _avg_sweep_width_m(initial_first_coords, points_per_line=initial_first_interp_points)
                initial_first_search_speed = spacing_based_search_speed(
                    sweep_len_m=first_width,
                    spacing_m=packet_spacing_line,
                    cruise_speed_mps=packet_cruise_speed,
                    weight=packet_search_speed_weight,
                )
            if initial_first_search_speed is None:
                initial_first_search_speed = packet_default_search_speed
            sweep_items: list[dict] = [{
                "idx": first_idx,
                "wp": initial_first_wp,
                "fp": initial_first_fp,
                "coord": initial_first_wp.get("coordinate") or {},
                "coords": initial_first_coords,
                "search_speed": initial_first_search_speed,
                "fov": initial_first_fp.get("fieldOfView", FOV_DEG),
                "interp_points": initial_first_interp_points,
                "sweep_idx": initial_first_wp.get("_sweepIdx"),
            }]
            for idx in sweep_indices[1:]:
                wp = wps[idx]
                fp = wp.get("filmingProperty") or OrderedDict()
                ls = fp.get("lineSearch") or {}
                coords = _copy_coord_list(ls.get("coordinateList") or [])
                if not coords:
                    continue
                search_speed = ls.get("searchSpeed")
                interp_points = _interp_points_hint(ls.get("interpolationPoints"))
                if search_speed is None:
                    width_m = _avg_sweep_width_m(coords, points_per_line=interp_points)
                    search_speed = spacing_based_search_speed(
                        sweep_len_m=width_m,
                        spacing_m=packet_spacing_line,
                        cruise_speed_mps=packet_cruise_speed,
                        weight=packet_search_speed_weight,
                    )
                sweep_items.append({
                    "idx": idx,
                    "wp": wp,
                    "fp": fp,
                    "coord": wp.get("coordinate") or {},
                    "coords": coords,
                    "search_speed": search_speed,
                    "fov": fp.get("fieldOfView", FOV_DEG),
                    "interp_points": interp_points,
                    "sweep_idx": wp.get("_sweepIdx"),
                })

            reference_coord = prev_tail_coord
            if is_area_pkt and isinstance(prev_last_wp_coord, dict):
                reference_coord = prev_last_wp_coord
            if not isinstance(reference_coord, dict):
                reference_coord = pkt.get("_prev_point") if isinstance(pkt.get("_prev_point"), dict) else None
            if not isinstance(reference_coord, dict):
                reference_coord = entry_anchor if isinstance(entry_anchor, dict) else None

            sweep_items = _orient_sweep_items(sweep_items, reference_coord)
            if uav_plan_mode == "dub_path" and is_area_pkt:
                area_bearing_deg = pkt.get("_area_bearing_deg")
                area_sep_offset_m = _packet_route_offset_sep_m(pkt) * float(AREA_ROUTE_OFFSET_SCALE)
                for item in sweep_items:
                    wp = item["wp"]
                    fp = wp.get("filmingProperty") or OrderedDict()
                    ls = fp.get("lineSearch") or OrderedDict()
                    coords = _copy_coord_list(item.get("coords") or [])
                    if coords:
                        ls["coordinateList"] = coords
                        fp["lineSearch"] = ls
                        shifted = _area_wp_coord_from_sweep(
                            coords,
                            bearing_deg=area_bearing_deg,
                            offset_m=area_sep_offset_m,
                            altitude_fn=_mission_wp_alt,
                            reference_coord=reference_coord,
                        )
                        if shifted is not None:
                            wp["coordinate"] = shifted
                            item["coord"] = _copy_coord_dict(shifted)
            _apply_oriented_sweep_items_to_wps(wps, sweep_items)
            first_item = sweep_items[0]
            first_idx = int(first_item["idx"])
            first_wp = first_item["wp"]
            first_fp = first_item["fp"]
            first_coord = first_item.get("coord") or {}
            first_coords = _copy_coord_list(first_item.get("coords") or [])
            first_search_speed = first_item.get("search_speed")
            first_interp_points = first_item.get("interp_points")
            records = sweep_items[1:]
            forced_remove_indices: list[int] = []

            if first_coords and records and not is_area_pkt:
                start_target = _copy_coord_dict(first_coords[0])
                first_fov = first_fp.get("fieldOfView", FOV_DEG)
                close_takeover_line_entry = False
                keep_first_line_search = False
                close_takeover_sep_m = 0.0
                close_takeover_route_m = 0.0
                close_takeover_scan_end_idx: int | None = None
                close_takeover_progress_xy: tuple[float, float] | None = None
                fold_close_takeover_first_sweep_into_next_group = False
                remove_close_takeover_transit_wp = False
                takeover_coord: OrderedDict | None = None
                if (
                    uav_plan_mode == "dub_path"
                    and (not is_area_pkt)
                    and isinstance(entry_anchor, dict)
                ):
                    close_takeover_sep_m = _packet_route_offset_sep_m(pkt)
                    close_takeover_radius_m = float(LINE_CLOSE_TAKEOVER_RADIUS_M)
                    if close_takeover_radius_m > 0.0:
                        takeover_coord = _coord_with_altitude(entry_anchor, _mission_wp_alt)
                        close_takeover_compare_coord = None
                        try:
                            line_list = info.get("lineList") or []
                            line_coords = (line_list[0] or {}).get("coordinateList") or []
                            if line_coords:
                                close_takeover_compare_coord = _coord_with_altitude(line_coords[0], _mission_wp_alt)
                        except Exception:
                            close_takeover_compare_coord = None
                        if not isinstance(close_takeover_compare_coord, dict):
                            close_takeover_compare_coord = _coord_midpoint(first_coords)
                        if not isinstance(close_takeover_compare_coord, dict):
                            close_takeover_compare_coord = start_target if isinstance(start_target, dict) else first_coord
                        close_takeover_route_m = _dist_between_coords(takeover_coord, close_takeover_compare_coord)
                        if close_takeover_route_m <= close_takeover_radius_m:
                            close_takeover_line_entry = True

                if close_takeover_line_entry and takeover_coord is not None:
                    entry_wp = OrderedDict([
                        ("waypointID", 0),
                        ("coordinate", OrderedDict(takeover_coord)),
                        ("speed", packet_cruise_speed),
                        ("eta", 0),
                        ("ecf", 0.0),
                        ("nextWaypointID", 0),
                        ("waypointPassType", PASS_FLYOVER),
                        ("filmingProperty", _mk_filming(
                            operation_mode=OPMODE_HOLD,
                            fov=first_fov,
                            sensor=SENSOR_EO_IR,
                            gimbal_pitch=ENTRY_HOLD_GIMBAL_PITCH,
                            gimbal_yaw=ENTRY_HOLD_GIMBAL_YAW,
                        )),
                    ])
                    if not LINE_CLOSE_TAKEOVER_EXTEND_FIRST_SEARCH:
                        keep_first_line_search = True
                        fold_close_takeover_first_sweep_into_next_group = True
                        remove_close_takeover_transit_wp = True
                        first_wp["filmingProperty"] = _mk_filming(
                            operation_mode=OPMODE_HOLD,
                            fov=first_fov,
                            sensor=SENSOR_EO_IR,
                            gimbal_pitch=ENTRY_HOLD_GIMBAL_PITCH,
                            gimbal_yaw=ENTRY_HOLD_GIMBAL_YAW,
                        )
                if has_existing_preline_prefix and not close_takeover_line_entry:
                    keep_first_line_search = True

                elif not close_takeover_line_entry and not (uav_plan_mode == "dub_path" and is_area_pkt):
                    start_target_coord = _coord_with_dem_altitude(start_target)
                    first_wp["filmingProperty"] = OrderedDict([
                        ("fieldOfView", first_fov),
                        ("sensorType", SENSOR_EO_IR),
                        ("operationMode", OPMODE_POINT),
                        ("coordinateOrientation", OrderedDict([
                            ("coordinate", start_target_coord)
                        ])),
                    ])

                def _vector_between(coord_from: dict, coord_to: dict) -> tuple[float, float]:
                    lat_a = float(coord_from.get("latitude", 0.0))
                    lon_a = float(coord_from.get("longitude", 0.0))
                    lat_b = float(coord_to.get("latitude", lat_a))
                    lon_b = float(coord_to.get("longitude", lon_a))
                    return llh_to_xy(lat_b, lon_b, lat_a, lon_a)

                full_coords_all = pkt.get("fullSweepCoords") or []
                first_sweep_idx = first_wp.get("_sweepIdx")
                def _segment_full_coords(start_idx: int | None, end_idx: int | None) -> list[dict] | None:
                    if not full_coords_all or start_idx is None or end_idx is None:
                        return None
                    start_sweep = max(0, int(start_idx))
                    end_sweep = max(0, int(end_idx))
                    max_sweep_idx = max(0, (len(full_coords_all) // 2) - 1)
                    start_sweep = min(start_sweep, max_sweep_idx)
                    end_sweep = min(end_sweep, max_sweep_idx)
                    if start_sweep <= end_sweep:
                        s = start_sweep * 2
                        e = (end_sweep + 1) * 2
                        if s >= len(full_coords_all) or e <= s:
                            return None
                        return _copy_coord_list(full_coords_all[s:min(e, len(full_coords_all))])

                    # When sweep order is reversed by reference_coord, retain the
                    # original per-sweep endpoint direction and only reverse sweep order.
                    pairs: list[dict] = []
                    for sweep_idx in range(start_sweep, end_sweep - 1, -1):
                        s = sweep_idx * 2
                        e = min(s + 2, len(full_coords_all))
                        if s >= len(full_coords_all) or e <= s:
                            continue
                        pairs.extend(_copy_coord_list(full_coords_all[s:e]))
                    return pairs or None

                def _record_anchor_coord(record: dict | None) -> dict | None:
                    if not isinstance(record, dict):
                        return None
                    coord = record.get("coord")
                    if not isinstance(coord, dict) or not coord:
                        wp = record.get("wp") if isinstance(record.get("wp"), dict) else {}
                        coord = wp.get("coordinate") if isinstance(wp, dict) else None
                    if not isinstance(coord, dict) or "latitude" not in coord or "longitude" not in coord:
                        return None
                    return _copy_coord_dict(coord)

                def _group_start_anchor_coord(group: list[int], start_idx: int | None) -> dict | None:
                    if start_idx is not None:
                        try:
                            start_sweep = int(start_idx)
                        except Exception:
                            start_sweep = None
                        if start_sweep is not None:
                            try:
                                if int(first_item.get("sweep_idx")) == start_sweep:
                                    first_anchor = _record_anchor_coord(first_item)
                                    if first_anchor is not None:
                                        return first_anchor
                            except Exception:
                                pass
                            for pos in group:
                                try:
                                    record = records[pos]
                                except Exception:
                                    continue
                                try:
                                    if int(record.get("sweep_idx")) == start_sweep:
                                        anchor = _record_anchor_coord(record)
                                        if anchor is not None:
                                            return anchor
                                except Exception:
                                    continue
                    if group:
                        try:
                            return _record_anchor_coord(records[group[0]])
                        except Exception:
                            return None
                    return None

                folded_first_sweep_coords = None
                if first_sweep_idx is not None:
                    folded_first_sweep_coords = _segment_full_coords(first_sweep_idx, first_sweep_idx)

                if (
                    LINE_CLOSE_TAKEOVER_EXTEND_FIRST_SEARCH
                    and close_takeover_line_entry
                    and takeover_coord is not None
                ):
                    try:
                        rep0 = records[0]
                        rep0_sweep_idx = rep0.get("sweep_idx")
                        rep0_coord = rep0.get("coord") or rep0["wp"].get("coordinate") or {}
                        base_progress_xy = _vector_between(first_coord, rep0_coord)
                        base_route_span_m = math.hypot(base_progress_xy[0], base_progress_xy[1])
                        if base_route_span_m >= 1.0:
                            close_takeover_route_m = max(float(close_takeover_sep_m) * 2.0, 1.0)
                            close_takeover_progress_xy = (
                                (float(base_progress_xy[0]) / float(base_route_span_m)) * float(close_takeover_route_m),
                                (float(base_progress_xy[1]) / float(base_route_span_m)) * float(close_takeover_route_m),
                            )
                        if first_sweep_idx is not None and rep0_sweep_idx is not None and full_coords_all:
                            span_count = max(1, int(rep0_sweep_idx) - int(first_sweep_idx) + 1)
                            meters_per_sweep = base_route_span_m / max(float(span_count), 1.0)
                            target_scan_m = close_takeover_route_m + close_takeover_sep_m
                            sweep_count = max(1, int(math.ceil(target_scan_m / max(meters_per_sweep, 1.0))))
                            max_sweep_idx = max(0, (len(full_coords_all) // 2) - 1)
                            close_takeover_scan_end_idx = min(max_sweep_idx, int(first_sweep_idx) + sweep_count - 1)
                    except Exception:
                        close_takeover_progress_xy = None
                        close_takeover_route_m = 0.0
                        close_takeover_scan_end_idx = None

                    if close_takeover_progress_xy is not None and close_takeover_route_m >= 1.0:
                        tk_lat = float(takeover_coord.get("latitude", 0.0))
                        tk_lon = float(takeover_coord.get("longitude", 0.0))
                        wp2_lat, wp2_lon = xy_to_llh(
                            float(close_takeover_progress_xy[0]),
                            float(close_takeover_progress_xy[1]),
                            tk_lat,
                            tk_lon,
                        )
                        first_wp["coordinate"] = OrderedDict([
                            ("latitude", round(wp2_lat, 6)),
                            ("longitude", round(wp2_lon, 6)),
                            ("altitude", _mission_wp_alt(wp2_lat, wp2_lon)),
                        ])

                    initial_scan_coords = _segment_full_coords(first_sweep_idx, close_takeover_scan_end_idx)
                    if initial_scan_coords:
                        initial_scan_spans = _collect_sweep_spans(initial_scan_coords, 2)
                        initial_width = None
                        if initial_scan_spans:
                            initial_width = round(sum(initial_scan_spans) / len(initial_scan_spans), 2)
                        initial_speed = spacing_based_search_speed(
                            sweep_len_m=initial_width,
                            spacing_m=packet_spacing_line,
                            cruise_speed_mps=packet_cruise_speed,
                            weight=packet_search_speed_weight,
                        )
                        if initial_speed is None:
                            initial_speed = first_search_speed
                        if initial_speed is None:
                            initial_speed = packet_default_search_speed
                        first_wp["filmingProperty"] = _mk_filming(
                            operation_mode=OPMODE_LINE,
                            fov=first_fov,
                            sensor=SENSOR_EO_IR,
                            line_search=OrderedDict([
                                ("coordinateList", _interpolate_line_coords(
                                    initial_scan_coords,
                                    SWEEP_LINE_INTERP_POINTS,
                                )),
                                ("searchSpeed", initial_speed),
                                ("interpolationPoints", SWEEP_LINE_INTERP_POINTS),
                            ]),
                        )
                        if close_takeover_scan_end_idx is not None:
                            kept_records = [
                                record for record in records
                                if record.get("sweep_idx") is None or int(record.get("sweep_idx")) > int(close_takeover_scan_end_idx)
                            ]
                            forced_remove_indices.extend(
                                int(record["idx"]) for record in records
                                if int(record["idx"]) not in {int(kept["idx"]) for kept in kept_records}
                            )
                            records = kept_records
                        if close_takeover_progress_xy is not None and close_takeover_route_m >= 1.0:
                            prog_ux = float(close_takeover_progress_xy[0]) / float(close_takeover_route_m)
                            prog_uy = float(close_takeover_progress_xy[1]) / float(close_takeover_route_m)
                            filtered_records: list[dict] = []
                            for record in records:
                                rec_coord = record.get("coord") or record["wp"].get("coordinate") or {}
                                rec_dx, rec_dy = _vector_between(takeover_coord, rec_coord)
                                rec_progress_m = (rec_dx * prog_ux) + (rec_dy * prog_uy)
                                if rec_progress_m > float(close_takeover_route_m) + 1.0:
                                    filtered_records.append(record)
                            forced_remove_indices.extend(
                                int(record["idx"]) for record in records
                                if int(record["idx"]) not in {int(kept["idx"]) for kept in filtered_records}
                            )
                            records = filtered_records
                elif close_takeover_line_entry and LINE_CLOSE_TAKEOVER_EXTEND_FIRST_SEARCH:
                    first_wp["filmingProperty"] = _mk_filming(
                        operation_mode=OPMODE_HOLD,
                        fov=first_fov,
                        sensor=SENSOR_EO_IR,
                        gimbal_pitch=ENTRY_HOLD_GIMBAL_PITCH,
                        gimbal_yaw=ENTRY_HOLD_GIMBAL_YAW,
                    )

                groups: list[list[int]] = []
                if merge_mode == "all":
                    groups = [list(range(len(records)))] if records else []
                elif merge_mode == "heading":
                    current_group = [0]
                    for pos in range(1, len(records)):
                        prev_vec = _vector_between(records[pos - 2]["coord"], records[pos - 1]["coord"]) if pos >= 2 else None
                        curr_vec = _vector_between(records[pos - 1]["coord"], records[pos]["coord"])
                        angle = 0.0 if prev_vec is None else _angle_between(prev_vec, curr_vec)
                        if angle <= merge_heading_deg:
                            current_group.append(pos)
                        else:
                            groups.append(current_group)
                            current_group = [pos]
                    groups.append(current_group)
                elif merge_mode == "curve":
                    coords_chain = [first_coord] + [record["coord"] for record in records]
                    signs = [0] * len(coords_chain)
                    for idx in range(1, len(coords_chain) - 1):
                        v1 = _vector_between(coords_chain[idx - 1], coords_chain[idx])
                        v2 = _vector_between(coords_chain[idx], coords_chain[idx + 1])
                        angle = _angle_between(v1, v2)
                        if angle <= merge_heading_deg:
                            signs[idx] = 0
                        else:
                            cross = (v1[0] * v2[1]) - (v1[1] * v2[0])
                            signs[idx] = 1 if cross >= 0 else -1
                    for idx in range(1, len(signs) - 1):
                        if signs[idx] == 0 and signs[idx - 1] == signs[idx + 1] != 0:
                            signs[idx] = signs[idx - 1]
                    current_group = [0]
                    current_sign = signs[1] if len(signs) > 1 else 0
                    for pos in range(1, len(records)):
                        sign = signs[pos + 1]
                        if sign == current_sign:
                            current_group.append(pos)
                        else:
                            groups.append(current_group)
                            current_group = [pos]
                            current_sign = sign
                    groups.append(current_group)
                else:
                    groups = [[idx] for idx in range(len(records))]

                groups = _split_record_groups_by_spacing(
                    records,
                    groups,
                    anchor_coord=first_coord,
                    spacing_m=(
                            _area_route_wp_spacing_m()
                            if is_area_pkt
                            else runtime_line_route_spacing_m
                        ),
                    max_coords_per_group=_max_linesearch_coords_per_waypoint(),
                    merge_short_tail=not is_area_pkt,
                )
                to_remove: list[int] = list(dict.fromkeys(forced_remove_indices))
                sweep_step = 1
                if records and first_sweep_idx is not None and records[0].get("sweep_idx") is not None:
                    try:
                        sweep_step = 1 if int(records[0].get("sweep_idx")) >= int(first_sweep_idx) else -1
                    except Exception:
                        sweep_step = 1
                prev_sweep_idx = first_sweep_idx
                if keep_first_line_search and prev_sweep_idx is not None:
                    prev_sweep_idx = int(prev_sweep_idx) + int(sweep_step)
                if close_takeover_line_entry and close_takeover_scan_end_idx is not None:
                    prev_sweep_idx = close_takeover_scan_end_idx
                if not is_area_pkt and prev_sweep_idx is not None:
                    groups = _rebalance_groups_by_sweep_span(
                        records,
                        groups,
                        initial_prev_sweep_idx=int(prev_sweep_idx),
                        sweep_step=int(sweep_step),
                    )
                for g_idx, group in enumerate(groups):
                    rep_pos = group[-1]
                    rep = records[rep_pos]
                    rep_fp = rep["fp"]
                    merged_coords: list[dict] = []
                    merged_spans: list[float] = []
                    used_segment_coords = False
                    start_sweep_idx = prev_sweep_idx
                    if g_idx > 0 and start_sweep_idx is not None:
                        start_sweep_idx = int(start_sweep_idx) + int(sweep_step)
                    elif (
                        close_takeover_line_entry
                        and start_sweep_idx is not None
                        and not fold_close_takeover_first_sweep_into_next_group
                    ):
                        start_sweep_idx = int(start_sweep_idx) + int(sweep_step)
                    group_anchor_coord = None
                    if g_idx > 0 or not fold_close_takeover_first_sweep_into_next_group:
                        group_anchor_coord = _group_start_anchor_coord(group, start_sweep_idx)
                    segment_coords = _segment_full_coords(start_sweep_idx, rep.get("sweep_idx"))
                    if segment_coords:
                        merged_coords.extend(segment_coords)
                        merged_spans.extend(_collect_sweep_spans(segment_coords, 2))
                        used_segment_coords = True
                    if g_idx == 0 and fold_close_takeover_first_sweep_into_next_group:
                        folded_coords = _copy_coord_list(folded_first_sweep_coords or first_coords)
                        folded_interp = 2 if folded_first_sweep_coords else first_interp_points
                        if folded_coords:
                            merged_coords = folded_coords + merged_coords
                            merged_spans = _collect_sweep_spans(folded_coords, folded_interp) + merged_spans
                    elif g_idx == 0 and not keep_first_line_search and not used_segment_coords:
                        # When the packed segment already starts from the first sweep via
                        # fullSweepCoords, re-appending the interpolated first waypoint
                        # lineSearch corrupts the first packet and creates a diagonal tail
                        # back to the starting sweep.
                        initial_group_coords = _copy_coord_list(folded_first_sweep_coords or first_coords)
                        initial_group_interp = 2 if folded_first_sweep_coords else first_interp_points
                        if initial_group_coords:
                            merged_coords.extend(initial_group_coords)
                            merged_spans.extend(
                                _collect_sweep_spans(initial_group_coords, initial_group_interp)
                            )
                    for pos in group:
                        if used_segment_coords:
                            continue
                        coords_copy = _copy_coord_list(records[pos]["coords"])
                        merged_coords.extend(coords_copy)
                        merged_spans.extend(_collect_sweep_spans(
                            records[pos]["coords"],
                            records[pos].get("interp_points"),
                        ))
                    merged_width = None
                    if merged_spans:
                        merged_width = round(sum(merged_spans) / len(merged_spans), 2)
                    rep_speed = spacing_based_search_speed(
                        sweep_len_m=merged_width,
                        spacing_m=packet_spacing_line,
                        cruise_speed_mps=packet_cruise_speed,
                        weight=packet_search_speed_weight,
                    )
                    if rep_speed is None:
                        rep_speed = rep["search_speed"]
                    if rep_speed is None:
                        rep_speed = first_search_speed
                    if g_idx == 0 and fold_close_takeover_first_sweep_into_next_group and rep_speed is not None:
                        rep_speed = round(
                            float(rep_speed) * float(LINE_CLOSE_TAKEOVER_FIRST_GROUP_SEARCH_SPEED_SCALE),
                            2,
                        )
                    rep_fp["fieldOfView"] = rep["fov"]
                    rep_fp["sensorType"] = SENSOR_EO_IR
                    rep_fp["operationMode"] = OPMODE_LINE
                    if group_anchor_coord is not None:
                        # Keep the visible/transit WP at the start of the merged
                        # LINE segment; the kept representative is otherwise the
                        # last sweep in the group.
                        rep["wp"]["coordinate"] = OrderedDict(_copy_coord_dict(group_anchor_coord))
                        rep["coord"] = _copy_coord_dict(group_anchor_coord)
                    interpolated_coords = _interpolate_line_coords(
                        merged_coords,
                        SWEEP_LINE_INTERP_POINTS,
                    )
                    rep_fp["lineSearch"] = OrderedDict([
                        ("coordinateList", interpolated_coords),
                        ("searchSpeed", rep_speed),
                        ("interpolationPoints", SWEEP_LINE_INTERP_POINTS),
                    ])
                    if uav_plan_mode == "dub_path" and is_area_pkt and g_idx == len(groups) - 1:
                        rep["wp"]["_skip_path_search_speed_recalc"] = True
                        rep["wp"]["waypointPassType"] = PASS_FLYOVER
                    for pos in group:
                        if pos != rep_pos:
                            to_remove.append(records[pos]["idx"])
                    prev_sweep_idx = rep.get("sweep_idx", prev_sweep_idx)

                if remove_close_takeover_transit_wp:
                    to_remove.append(first_idx)

                for idx in sorted(to_remove, reverse=True):
                    del wps[idx]

            leading_entry_line_merged = False
            if entry_wp is not None:
                wps.insert(0, entry_wp)

                # ── take over 직후 첫 sweep WP만: sweep line 사이에 중간선 삽입 (2배 촘촘) ──
                _first_sweep_wp = None
                for _fw in wps[1:]:
                    _fw_fp = _fw.get("filmingProperty") or {}
                    if int(_fw_fp.get("operationMode", 0) or 0) == OPMODE_LINE:
                        _first_sweep_wp = _fw
                        break
                if LINE_FIRST_SWEEP_DENSIFY_ENABLED and _first_sweep_wp is not None:
                    _fs_fp = _first_sweep_wp.get("filmingProperty") or {}
                    _fs_ls = _fs_fp.get("lineSearch") or {}
                    _fs_coords = _fs_ls.get("coordinateList") or []
                    _fs_interp = int(_fs_ls.get("interpolationPoints", 0) or 0)
                    if _fs_interp >= 2 and len(_fs_coords) >= 2 * _fs_interp:
                        # interp 단위로 sweep line 분리
                        _sweep_lines: list[list[dict]] = []
                        for _ci in range(0, len(_fs_coords), _fs_interp):
                            _chunk = _fs_coords[_ci:_ci + _fs_interp]
                            if len(_chunk) >= 2:
                                _sweep_lines.append(_chunk)
                        # 1) 모든 line을 동일 방향으로 정규화 (첫 line 시작점 기준)
                        _normalized: list[list[dict]] = []
                        _ref_start: dict | None = None
                        for _sl in _sweep_lines:
                            if _ref_start is None:
                                _ref_start = _sl[0]
                                _normalized.append(list(_sl))
                                continue
                            _d_fwd = (float(_sl[0].get("latitude", 0)) - float(_ref_start.get("latitude", 0))) ** 2 + \
                                     (float(_sl[0].get("longitude", 0)) - float(_ref_start.get("longitude", 0))) ** 2
                            _d_rev = (float(_sl[-1].get("latitude", 0)) - float(_ref_start.get("latitude", 0))) ** 2 + \
                                     (float(_sl[-1].get("longitude", 0)) - float(_ref_start.get("longitude", 0))) ** 2
                            if _d_rev < _d_fwd:
                                _normalized.append(list(reversed(_sl)))
                            else:
                                _normalized.append(list(_sl))
                        # 2) 정규화 상태에서 인접 line 사이에 중간선 삽입
                        _all_normalized: list[list[dict]] = []
                        for _li in range(len(_normalized)):
                            _all_normalized.append(_copy_coord_list(_normalized[_li]))
                            if _li + 1 < len(_normalized):
                                _cur_n = _normalized[_li]
                                _nxt_n = _normalized[_li + 1]
                                _mid_line: list[dict] = []
                                for _pi in range(min(len(_cur_n), len(_nxt_n))):
                                    _mid_line.append(OrderedDict([
                                        ("latitude", (float(_cur_n[_pi].get("latitude", 0)) + float(_nxt_n[_pi].get("latitude", 0))) / 2.0),
                                        ("longitude", (float(_cur_n[_pi].get("longitude", 0)) + float(_nxt_n[_pi].get("longitude", 0))) / 2.0),
                                        ("altitude", int((float(_cur_n[_pi].get("altitude", 0)) + float(_nxt_n[_pi].get("altitude", 0))) / 2.0)),
                                    ]))
                                if len(_mid_line) >= 2:
                                    _all_normalized.append(_mid_line)
                        # 3) ㄹ자 패턴 적용: 짝수=정방향, 홀수=역방향
                        _dense_coords_out: list[dict] = []
                        for _out_idx, _nl in enumerate(_all_normalized):
                            if _out_idx % 2 == 0:
                                _dense_coords_out.extend(_copy_coord_list(_nl))
                            else:
                                _dense_coords_out.extend(_copy_coord_list(reversed(_nl)))
                        _fs_ls["coordinateList"] = _dense_coords_out
                        _fs_ls["interpolationPoints"] = _fs_interp
                        _fs_fp["lineSearch"] = _fs_ls

                leading_entry_line_merged = _merge_duplicate_entry_line_search_after_point_inplace(wps)
                if not leading_entry_line_merged:
                    _move_leading_close_takeover_scan_to_next_line_inplace(
                        wps,
                        max_entry_gap_m=max(300.0, float(MIN_ROUTE_SPACING_M)),
                    )
            elif not is_area_pkt:
                _prepend_point_before_leading_line_search_inplace(
                    wps,
                    entry_coord=entry_anchor if isinstance(entry_anchor, dict) else None,
                    speed_mps=packet_cruise_speed,
                )
                leading_entry_line_merged = _merge_duplicate_entry_line_search_after_point_inplace(wps)

            if not is_area_pkt:
                line_positions_after = [idx for idx, wp in enumerate(wps) if _has_line_search(wp)]
                if line_positions_after:
                    ref_coord = None
                    first_line_after = line_positions_after[0]
                    if first_line_after > 0:
                        ref_coord = _wp_effective_end_coord(wps[first_line_after - 1])
                    if not isinstance(ref_coord, dict) or not ref_coord:
                        ref_coord = reference_coord if isinstance(reference_coord, dict) else None
                    _reorder_line_search_subset_inplace(
                        wps,
                        reference_coord=ref_coord,
                    )
        _record_dense_linesearch_metrics(
            lineSearchMergeMs=(time.perf_counter() - line_merge_started) * 1000.0,
        )

        for wp in wps:
            if "_sweepIdx" in wp:
                del wp["_sweepIdx"]

        if is_area_pkt:
            area_postprocess_started = time.perf_counter()
            is_area_nadir = bool(pkt.get("_area_nadir"))
            area_offset_sign: int | None = None
            if not is_area_nadir:
                prev_area_sign = prev_area_offset_sign_by_aircraft.get(aid)
                if prev_area_sign in (-1, 1):
                    area_offset_sign = -int(prev_area_sign)
                else:
                    first_area_line_coords: list[dict] = []
                    for area_wp in wps:
                        area_fp = area_wp.get("filmingProperty") or {}
                        area_ls = area_fp.get("lineSearch") or {}
                        first_area_line_coords = area_ls.get("coordinateList") or []
                        if first_area_line_coords:
                            break
                    area_offset_sign = _infer_area_offset_sign_from_sweep(
                        first_area_line_coords,
                        bearing_deg=pkt.get("_area_bearing_deg"),
                        offset_m=_packet_route_offset_sep_m(pkt) * float(AREA_ROUTE_OFFSET_SCALE),
                        reference_coord=(
                            prev_last_wp_coord
                            if isinstance(prev_last_wp_coord, dict)
                            else (
                                prev_tail_coord
                                if isinstance(prev_tail_coord, dict)
                                else (
                                    pkt.get("_prev_point")
                                    if isinstance(pkt.get("_prev_point"), dict)
                                    else (
                                        pkt.get("_entry_anchor")
                                        if isinstance(pkt.get("_entry_anchor"), dict)
                                        else None
                                    )
                                )
                            )
                        ),
                    )
            if not is_area_nadir:
                # Preserve the raw ordered area sweep WPs.
                # Area chunk packing below must operate on the aligned/raw route chain
                # rather than on angle/min-spacing simplified representatives.
                pass

            try:
                area_sep_offset_m = _packet_route_offset_sep_m(pkt) * float(AREA_ROUTE_OFFSET_SCALE)
            except Exception:
                area_sep_offset_m = 0.0
            area_reference_coord = pkt.get("_prev_point") if isinstance(pkt.get("_prev_point"), dict) else None
            if not isinstance(area_reference_coord, dict):
                area_reference_coord = prev_last_wp_coord if isinstance(prev_last_wp_coord, dict) else None
            if not isinstance(area_reference_coord, dict):
                area_reference_coord = pkt.get("_entry_anchor") if isinstance(pkt.get("_entry_anchor"), dict) else None
            area_reposition_started = time.perf_counter()
            _reposition_area_wps_by_sep(
                wps,
                offset_m=area_sep_offset_m,
                altitude_fn=_mission_wp_alt,
                bearing_deg=pkt.get("_area_bearing_deg"),
                reference_coord=area_reference_coord,
                offset_sign=area_offset_sign,
            )
            _record_dense_linesearch_metrics(
                areaRepositionMs=(time.perf_counter() - area_reposition_started) * 1000.0,
            )
            if not is_area_nadir:
                # Compute per-WP altitude from each individual sweep's terrain BEFORE
                # packing.  _pack_area_wps_by_spacing then preserves the max altitude
                # per group, so the packed WP altitude covers the highest terrain in
                # that group.  The post-pack altitude call is skipped for area missions.
                area_altitude_started = time.perf_counter()
                if _ground_required_final_geometry_preowner_enabled():
                    _preowner_ground_required_process_cache_for_waypoints(
                        wps,
                        signature_cache=ground_required_precompute_cache,
                        fallback_ground_ref_m=_safe_float_or_none(pkt.get("_mission_ground_ref_m")),
                    )
                _precompute_ground_required_for_waypoints(
                    wps,
                    signature_cache=ground_required_precompute_cache,
                    fallback_ground_ref_m=_safe_float_or_none(pkt.get("_mission_ground_ref_m")),
                )
                _apply_segment_altitude_to_search_waypoints(
                    wps,
                    altitude_offset_m=float(pkt.get("_mission_alt_offset_m", Altitude) or Altitude),
                    fallback_ground_ref_m=_safe_float_or_none(pkt.get("_mission_ground_ref_m")),
                    cruise_speed_mps=float(pkt.get("_mission_cruise_speed_mps") or 50.0),
                    climb_rate_mps=_runtime_uav_climb_rate_mps(),
                )
                _record_dense_linesearch_metrics(
                    areaPrePackAltitudeMs=(time.perf_counter() - area_altitude_started) * 1000.0,
                )
                is_first_area_packet = bool(pkt.get("_skip_area_entry_prefix"))
                area_pack_started = time.perf_counter()
                wps[:] = _pack_area_wps_by_spacing(
                    wps,
                    route_spacing_m=_area_route_wp_spacing_m(),
                    spacing_line_m=packet_spacing_line,
                    cruise_speed_mps=packet_cruise_speed,
                    search_speed_weight=packet_search_speed_weight,
                    default_search_speed=packet_default_search_speed,
                    post_entry_search_speed_scale=(
                        AREA_FIRST_PACKET_SEARCH_SPEED_SCALE
                        if is_first_area_packet
                        else 1.0
                    ),
                    post_entry_route_spacing_scale=(
                        AREA_FIRST_PACKET_SWEEP_GROUP_SCALE
                        if is_first_area_packet
                        else 1.0
                    ),
                )
                _record_dense_linesearch_metrics(
                    areaPackMs=(time.perf_counter() - area_pack_started) * 1000.0,
                )
            if not is_area_nadir and area_offset_sign in (-1, 1):
                prev_area_offset_sign_by_aircraft[aid] = int(area_offset_sign)
            _record_dense_linesearch_metrics(
                areaPostProcessMs=(time.perf_counter() - area_postprocess_started) * 1000.0,
            )
        # Keep _is_area until the final leading-line cleanup; it prevents the
        # close-TAKEOVER LINE merge from touching area lineSearch packets.
        pkt.pop("_area_nadir", None)
        pkt.pop("_area_sweep_mode", None)
        pkt.pop("_area_bearing_deg", None)
        pkt.pop("_mission_sep_m", None)
        pkt.pop("_mission_route_offset_sep_m", None)
        pkt.pop("_mission_route_offset_sep_explicit_zero", None)
        pkt.pop("_mission_route_offset_fov_ref_deg", None)
        pkt.pop("_entry_anchor", None)
        pkt.pop("_skip_area_entry_prefix", None)
        pkt.pop("_prev_point", None)

        if prev_tail_coord and wps:
            def _same_coord(a: dict, b: dict, tol_m: float = 1.0) -> bool:
                if not a or not b:
                    return False
                if "latitude" not in a or "longitude" not in a:
                    return False
                if "latitude" not in b or "longitude" not in b:
                    return False
                return _dist_between_coords(a, b) <= tol_m

            while wps and _same_coord(prev_tail_coord, wps[0].get("coordinate") or {}):
                wps.pop(0)

            if is_area_pkt and not is_area_nadir:
                try:
                    area_entry_min_gap_m = max(120.0, float(pkt.get("_mission_sep_m", 0.0) or 0.0) * 0.1)
                except Exception:
                    area_entry_min_gap_m = 120.0
                _drop_redundant_close_area_entry_anchor_inplace(
                    prev_tail_coord,
                    wps,
                    min_gap_m=area_entry_min_gap_m,
                )

        if not wps:
            continue

        # Area non-nadir: altitude already computed per individual WP before packing.
        # All other cases (Line, Corridor, Area nadir): compute altitude here.
        if not (is_area_pkt and not is_area_nadir):
            if _ground_required_final_geometry_preowner_enabled():
                _preowner_ground_required_process_cache_for_waypoints(
                    wps,
                    signature_cache=ground_required_precompute_cache,
                    fallback_ground_ref_m=_safe_float_or_none(pkt.get("_mission_ground_ref_m")),
                )
            _precompute_ground_required_for_waypoints(
                wps,
                signature_cache=ground_required_precompute_cache,
                fallback_ground_ref_m=_safe_float_or_none(pkt.get("_mission_ground_ref_m")),
            )
            _apply_segment_altitude_to_search_waypoints(
                wps,
                altitude_offset_m=float(pkt.get("_mission_alt_offset_m", Altitude) or Altitude),
                fallback_ground_ref_m=_safe_float_or_none(pkt.get("_mission_ground_ref_m")),
                cruise_speed_mps=float(pkt.get("_mission_cruise_speed_mps") or 50.0),
                climb_rate_mps=_runtime_uav_climb_rate_mps(),
            )
        _align_point_anchor_altitude_with_search_waypoints(wps)
        _enforce_waypoint_altitude_rate_limit_inplace(
            wps,
            vertical_rate_mps=_runtime_uav_climb_rate_mps(),
            default_speed_mps=float(pkt.get("_mission_cruise_speed_mps") or 50.0),
        )

        for wp in wps:
            if wp.get("waypointPassType") == PASS_LOITER:
                wp["_was_loiter"] = True
            elif wp.get("waypointPassType") == PASS_FLYOVER:
                wp["waypointPassType"] = PASS_FLYBY

        flyover_options = _effective_flyover_options()
        flyover_entry_offset = bool(flyover_options.get("entry_offset", False))
        flyover_dubins_prefix = bool(flyover_options.get("dubins_prefix", False))
        flyover_last_point = bool(flyover_options.get("last_point", False))
        flyover_all_wps = bool(flyover_options.get("all_wps", False))

        if flyover_all_wps:
            for wp in wps:
                if wp.get("waypointPassType") == PASS_LOITER:
                    continue
                wp["waypointPassType"] = PASS_FLYOVER
        else:
            if flyover_entry_offset:
                # Preserve the persisted option key, but apply it to the
                # single start waypoint of the collaborative base mission.
                for wp in wps:
                    if wp.get("waypointPassType") == PASS_LOITER:
                        continue
                    if isinstance(wp, dict):
                        wp["waypointPassType"] = PASS_FLYOVER
                        break
            if flyover_dubins_prefix:
                for wp in wps:
                    if wp.get("_flyover_dubins_prefix"):
                        if wp.get("waypointPassType") == PASS_LOITER:
                            continue
                        wp["waypointPassType"] = PASS_FLYOVER
            if flyover_last_point:
                for wp in reversed(wps):
                    if wp.get("waypointPassType") == PASS_LOITER:
                        continue
                    if isinstance(wp, dict):
                        wp["waypointPassType"] = PASS_FLYOVER
                        break

        for wp in wps:
            if "_flyover_dubins_prefix" in wp:
                del wp["_flyover_dubins_prefix"]
            if "_area_dubins_link" in wp:
                del wp["_area_dubins_link"]

        for wp in wps:
            was_loiter = wp.get("_was_loiter")
            is_loiter = wp.get("waypointPassType") == PASS_LOITER
            if not is_loiter and flyover_all_wps and was_loiter:
                is_loiter = True
            if is_loiter:
                if not wp.get("filmingProperty"):
                    wp["filmingProperty"] = _mk_point_filming_for_coord(wp.get("coordinate") or {})
                wp["loiterProperty"] = OrderedDict([
                    ("radius", LOITER_RADIUS_M),
                    ("direction", LOITER_DIRECTION),
                    ("time", LOITER_TIME_S),
                    ("speed", LOITER_SPEED_MPS),
                ])
            if "_was_loiter" in wp:
                del wp["_was_loiter"]

        _recompute_eta_ecf_inplace(wps, default_speed_mps=packet_cruise_speed)
        if is_area_pkt and not runtime_manual_fov_active:
            _scale_output_fov_inplace(wps, scale=float(AREA_OUTPUT_FOV_SCALE))

        for wp in wps:
            if "_skip_path_search_speed_recalc" in wp:
                del wp["_skip_path_search_speed_recalc"]
            if "_path_search_speed_weight" in wp:
                del wp["_path_search_speed_weight"]

        if is_area_pkt and not is_area_nadir:
            prev_area_pkt = prev_area_pkt_by_aircraft.get(aid)
            if isinstance(prev_area_pkt, dict):
                _append_area_transition_links_inplace(
                    prev_area_pkt,
                    pkt,
                    altitude_fn=_mission_wp_alt,
                )
            prev_area_pkt_by_aircraft[aid] = pkt
        else:
            prev_area_pkt_by_aircraft.pop(aid, None)

        last_coord = _wp_effective_end_coord(wps[-1])
        if last_coord:
            prev_tail_by_aircraft[aid] = dict(last_coord)
            last_wp_coord = wps[-1].get("coordinate") or {}
            if isinstance(last_wp_coord, dict) and last_wp_coord:
                prev_last_wp_coord_by_aircraft[aid] = dict(last_wp_coord)
            heading_from = None
            last_fp = wps[-1].get("filmingProperty") or {}
            last_ls = last_fp.get("lineSearch") or {}
            last_ls_coords = last_ls.get("coordinateList") or []
            if len(last_ls_coords) >= 2:
                heading_from = last_ls_coords[-2]
            elif len(wps) >= 2:
                heading_from = _wp_effective_end_coord(wps[-2])
            if isinstance(heading_from, dict) and heading_from:
                prev_heading_by_aircraft[aid] = _heading_rad_between_coords(
                    heading_from,
                    last_coord,
                )
            else:
                prev_heading_by_aircraft.pop(aid, None)

    if formation_followers:
        formation_post_started = time.perf_counter()
        leader_wplist_by_key: dict[int, list[OrderedDict]] = {}
        for pkt in packets:
            fkey = pkt.get("_formation_key")
            if fkey is None:
                continue
            leader_id = formation_leaders.get(fkey)
            if leader_id is None:
                continue
            if int(pkt.get("aircraftID", 0)) == int(leader_id) and pkt.get("wplist"):
                leader_wplist_by_key[int(fkey)] = pkt["wplist"]

        try:
            formation_runtime_payload = _load_runtime_settings() if callable(_load_runtime_settings) else None
        except Exception:
            formation_runtime_payload = None

        def _process_formation_follower_post(
            fkey: int,
            fpkt: dict,
            *,
            signature_cache: dict[tuple, float] | None,
            collect_metrics: bool,
        ) -> tuple[list[OrderedDict] | None, dict]:
            if collect_metrics:
                reset_dense_linesearch_metrics()
            with _runtime_override_context(formation_runtime_payload):
                leader_wplist = leader_wplist_by_key.get(int(fkey))
                if not leader_wplist:
                    return None, get_dense_linesearch_metrics(reset=True) if collect_metrics else {}
                follower_wplist = deepcopy(leader_wplist)
                _strip_filming_properties(follower_wplist)
                follower_aid = int(fpkt.get("aircraftID", 0) or 0)
                if follower_aid in (4, 5, 6):
                    formation_dem_started = time.perf_counter()
                    if _ground_required_final_geometry_preowner_enabled():
                        _preowner_ground_required_process_cache_for_waypoints(
                            follower_wplist,
                            signature_cache=signature_cache if isinstance(signature_cache, dict) else {},
                        )
                    _precompute_ground_required_for_waypoints(
                        follower_wplist,
                        signature_cache=signature_cache if isinstance(signature_cache, dict) else {},
                    )
                    _record_dense_linesearch_metrics(
                        formationDemMs=(time.perf_counter() - formation_dem_started) * 1000.0,
                    )
                    _apply_segment_altitude_to_search_waypoints(
                        follower_wplist,
                        altitude_offset_m=_aircraft_alt_offset_m(follower_aid),
                        cruise_speed_mps=float(cruise_speed),
                        climb_rate_mps=_runtime_uav_climb_rate_mps(),
                    )
                    _align_point_anchor_altitude_with_search_waypoints(follower_wplist)
                    _enforce_waypoint_altitude_rate_limit_inplace(
                        follower_wplist,
                        vertical_rate_mps=_runtime_uav_climb_rate_mps(),
                        default_speed_mps=float(cruise_speed),
                    )
                metrics = get_dense_linesearch_metrics(reset=True) if collect_metrics else {}
                return follower_wplist, metrics

        def _record_formation_worker_metrics(metrics: dict) -> None:
            if not isinstance(metrics, dict):
                return
            for key, value in metrics.items():
                try:
                    if float(value or 0.0) == 0.0:
                        continue
                except Exception:
                    continue
                _record_dense_linesearch_metrics(**{key: value})

        def _run_formation_postprocess_sequential() -> None:
            for fkey, fpkt in formation_followers:
                follower_wplist, _metrics = _process_formation_follower_post(
                    fkey,
                    fpkt,
                    signature_cache=ground_required_precompute_cache,
                    collect_metrics=False,
                )
                if follower_wplist is not None:
                    fpkt["wplist"] = follower_wplist
            _record_dense_linesearch_metrics(
                formationPostProcessWorkers=1,
                formationPostProcessTasks=len(formation_followers),
            )

        post_workers = _formation_follower_postprocess_workers(len(formation_followers))
        if post_workers > 1:
            try:
                results_by_index: dict[int, tuple[list[OrderedDict] | None, dict]] = {}
                with concurrent.futures.ThreadPoolExecutor(
                    max_workers=post_workers,
                    thread_name_prefix="FormationPost",
                ) as executor:
                    futures = {
                        executor.submit(
                            _process_formation_follower_post,
                            fkey,
                            fpkt,
                            signature_cache={},
                            collect_metrics=True,
                        ): idx
                        for idx, (fkey, fpkt) in enumerate(formation_followers)
                    }
                    for future in concurrent.futures.as_completed(futures):
                        results_by_index[int(futures[future])] = future.result()
                for idx, (_fkey, fpkt) in enumerate(formation_followers):
                    follower_wplist, metrics = results_by_index.get(idx, (None, {}))
                    if follower_wplist is not None:
                        fpkt["wplist"] = follower_wplist
                    _record_formation_worker_metrics(metrics)
                _record_dense_linesearch_metrics(
                    formationPostProcessWorkers=post_workers,
                    formationPostProcessTasks=len(formation_followers),
                )
            except Exception:
                _run_formation_postprocess_sequential()
        else:
            _run_formation_postprocess_sequential()

        _record_dense_linesearch_metrics(
            formationPostProcessMs=(time.perf_counter() - formation_post_started) * 1000.0,
        )

    for pkt in packets:
        wps = pkt.get("wplist") or []
        if pkt.get("isFormationFlight"):
            _strip_filming_properties(wps)
        elif not bool(pkt.get("_is_area")):
            leading_line_merged = _move_leading_close_takeover_scan_to_next_line_inplace(
                wps,
                max_entry_gap_m=max(300.0, float(MIN_ROUTE_SPACING_M)),
            )
            if leading_line_merged:
                merged_line_wp = next((wp for wp in wps[1:] if _has_line_search(wp)), None)
                if isinstance(merged_line_wp, dict):
                    merged_line_wp["_path_search_speed_weight"] = float(
                        pkt.get("_mission_search_speed_weight", 1.0) or 1.0
                    )
                _recompute_eta_ecf_inplace(
                    wps,
                    default_speed_mps=float(pkt.get("_mission_cruise_speed_mps") or cruise_speed),
                )
                for wp in wps:
                    if isinstance(wp, dict):
                        wp.pop("_path_search_speed_weight", None)

    all_packet_waypoints_for_filming: list[dict] = []
    for pkt in packets:
        all_packet_waypoints_for_filming.extend(
            wp for wp in (pkt.get("wplist") or []) if isinstance(wp, dict)
        )

    filming_candidate_scan_started = time.perf_counter()
    filming_target_waypoints = [
        wp for wp in all_packet_waypoints_for_filming if _has_filming_target_coords(wp)
    ]
    _record_dense_linesearch_metrics(
        filmingCandidateScanMs=(time.perf_counter() - filming_candidate_scan_started) * 1000.0,
        filmingCandidateWaypointCount=len(filming_target_waypoints),
    )
    if filming_target_waypoints:
        filming_normalize_started = time.perf_counter()
        reset_filming_altitude_metrics()
        filming_changed = normalize_filming_target_altitudes_in_waypoints(
            filming_target_waypoints,
            clearance_m=FILMING_TARGET_ALTITUDE_FLOOR_CLEARANCE_M,
            dem_lookup_many=_dem_alt_many,
        )
        _mark_line_search_coords_trusted_dem_altitudes(filming_target_waypoints)
        filming_metrics = get_filming_altitude_metrics(reset=True)
        _record_dense_linesearch_metrics(
            filmingNormalizeMs=(time.perf_counter() - filming_normalize_started) * 1000.0,
            filmingTerrainBatchMs=float(filming_metrics.get("filmingTerrainBatchMs") or 0.0),
            filmingTargetCoordCount=int(filming_metrics.get("filmingTargetCoordCount") or 0),
            filmingUniqueCoordCount=int(filming_metrics.get("filmingUniqueCoordCount") or 0),
            filmingNormalizeCallCount=1,
            filmingNormalizeChangedCount=int(filming_changed or 0),
            filmingTerrainBatchFallbackCount=int(filming_metrics.get("filmingTerrainBatchFallbackCount") or 0),
        )
        _seed_dem_alt_cache_from_line_coord_altitudes(
            filming_target_waypoints,
            key_prefix=_dem_alt_cache_prefix(),
        )
    else:
        reset_filming_altitude_metrics()
        get_filming_altitude_metrics(reset=True)
        _record_dense_linesearch_metrics(
            filmingNormalizeMs=0.0,
            filmingTerrainBatchMs=0.0,
            filmingTargetCoordCount=0,
            filmingUniqueCoordCount=0,
            filmingNormalizeSkippedCount=1,
            filmingTerrainBatchFallbackCount=0,
        )

    all_packet_waypoints: list[OrderedDict] = []
    for pkt in packets:
        all_packet_waypoints.extend(wp for wp in (pkt.get("wplist") or []) if isinstance(wp, dict))
    ground_prepass_started = time.perf_counter()
    try:
        if _ground_required_final_prepass_fresh_skip_enabled() and _ground_required_precompute_already_fresh(
            all_packet_waypoints,
            signature_cache=ground_required_precompute_cache,
        ):
            pass
        elif _ground_required_final_geometry_preowner_enabled():
            _preowner_ground_required_process_cache_for_waypoints(
                all_packet_waypoints,
                signature_cache=ground_required_precompute_cache,
            )
            _precompute_ground_required_for_waypoints(
                all_packet_waypoints,
                signature_cache=ground_required_precompute_cache,
            )
        else:
            _precompute_ground_required_for_waypoints(
                all_packet_waypoints,
                signature_cache=ground_required_precompute_cache,
            )
    finally:
        _record_dense_linesearch_metrics(
            groundPrepassMs=(time.perf_counter() - ground_prepass_started) * 1000.0,
        )

    for pkt in packets:
        wps = pkt.get("wplist") or []
        _enforce_filming_target_altitude_floor_inplace(wps)

    for pkt in packets:
        _normalize_output_waypoints_inplace(pkt.get("wplist") or [])

    if getattr(wp_alloc, "_use_global", False):
        total_wp_count = sum(len(pkt.get("wplist") or []) for pkt in packets)
        if total_wp_count > 0:
            wp_alloc = _WPAllocator(start=int(_reserve_waypoint_block(total_wp_count)))

    for pkt in packets:
        wps = pkt.get("wplist") or []
        if not wps:
            continue
        for wp in wps:
            wp["waypointID"] = wp_alloc.alloc()
        for idx in range(len(wps) - 1):
            wps[idx]["nextWaypointID"] = wps[idx + 1]["waypointID"]
        wps[-1]["nextWaypointID"] = 0

    # ────────────────────────── 4) 최종 조립 ─────────────────────────
    json_ready_started = time.perf_counter()
    result = []
    for pkt in packets:
        pkt.pop("_is_area", None)
        pkt.pop("_area_sweep_mode", None)
        pkt.pop("_area_bearing_deg", None)
        pkt.pop("_mission_sep_m", None)
        pkt.pop("_mission_route_offset_sep_m", None)
        pkt.pop("_mission_route_offset_sep_explicit_zero", None)
        pkt.pop("_mission_route_offset_fov_ref_deg", None)
        pkt.pop("_mission_cruise_speed_mps", None)
        pkt.pop("_mission_default_search_speed", None)
        pkt.pop("_mission_spacing_line_m", None)
        pkt.pop("_mission_search_speed_weight", None)
        pkt.pop("_mission_alt_offset_m", None)
        pkt.pop("_mission_ground_ref_m", None)
        pkt.pop("_entry_anchor", None)
        formation_info = pkt.get("formationInfo")
        items = [
            ("timestamp", now_ms),
            ("Source", _sw_code()),
            ("pathID", pkt["pathID"]),
            ("aircraftID", pkt["aircraftID"]),
            ("isFormationFlight", bool(pkt.get("isFormationFlight", False))),
        ]
        if formation_info:
            items.append(("formationInfo", formation_info))
        items.append(("waypointList", pkt["wplist"]))
        result.append(OrderedDict(items))
    _record_dense_linesearch_metrics(
        jsonReadyMs=(time.perf_counter() - json_ready_started) * 1000.0,
    )
    return result




def _runtime_area_sweep_mode() -> str:
    if _get_runtime_str is None:
        return "vertical"
    try:
        raw = str(_get_runtime_str("area_sweep_mode", "vertical") or "vertical").strip().lower()
    except Exception:
        return "vertical"
    if raw in {"vertical", "ver", "perpendicular", "orthogonal"}:
        return "vertical"
    if raw in {"nadir", "directdown", "bf_nadir"}:
        return "nadir"
    return "parallel"


def _runtime_uav_plan_mode() -> str:
    if _get_runtime_str is None:
        return "dub_path"
    try:
        raw = str(_get_runtime_str("uav_plan_mode", "dub_path") or "dub_path").strip().lower()
    except Exception:
        return "dub_path"
    if raw == "dub_path":
        return "dub_path"
    return "normal"
