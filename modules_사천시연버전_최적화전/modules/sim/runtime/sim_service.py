from __future__ import annotations

import json
import math
import random
import re
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from functools import lru_cache
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

import numpy as np
import tifffile

from ..config import (
    DEM_DIR,
    SIM_BASE_DT,
    SIM_0401_ACTIVE_HZ,
    SIM_0402_HISTORY_MAX,
    SIM_HISTORY_MAX,
    SIM_HISTORY_RESPONSE_MAX,
    SIM_HISTORY_SAMPLE_HZ,
    SIM_INTERNAL_STEP_HZ,
    SIM_MAX_STEPS_PER_LOOP,
    SIM_POS_TOL,
    SIM_PROJECTILE_HIT_RADIUS_GUN,
    SIM_PROJECTILE_HIT_RADIUS_MISSILE,
    SIM_PROJECTILE_MAX,
    SIM_PROJECTILE_SPEED_GUN,
    SIM_PROJECTILE_SPEED_MISSILE,
    SIM_LAH_AUTO_ATTACK,
    SIM_ENEMY_HIT_SCALE,
    SIM_FRIENDLY_HIT_SCALE,
    SIM_SPEED_LAH,
    SIM_SPEED_UAV,
    SIM_TIME_SCALE,
    SIM_AUTO_TRACK_ALWAYS,
    SIM_AUTO_TRACK_TAKEOVER,
    SIM_TRACK_ALT_BUFFER_M,
    SIM_TRACK_LOITER_RADIUS_M,
    SIM_TRACK_LOITER_SPEED_MPS,
    SIM_TRACK_LOST_TIMEOUT_S,
    SIM_UAV_DETECT_RANGE_M,
    SIM_INPUT_ADVANCE_GUARD_SEC,
    SIM_MULTI_TARGET_PREVIEW_SEC,
    SIM_ROI_GAZE_DURATION_S,
)
from modules.common import agent_status_snapshot, db_paths
from modules.monitoring.logic.target_info import reset_target_info
from .geo import GeoConverter
from .lah import LAH, LAHParams
from .uav import UAV, UAVParams, load_uav_params_profile
from .controllers.waypoint_pid import WaypointPIDController, WaypointTarget, load_pid_gains_for_time_scale
from .operation_mode import OperationContext, OperationMode, build_operation_mode
from .targets import AirDefenseThreat, GroundTarget, RadarParams, WeaponParams, WeaponType


def _agent_label(aircraft_id: int) -> str:
    if 1 <= aircraft_id <= 3:
        return f"LAH{aircraft_id}"
    if 4 <= aircraft_id <= 6:
        return f"UAV{aircraft_id - 3}"
    return f"AC{aircraft_id}"


def _airframe_type(aircraft_id: int) -> str:
    if 1 <= aircraft_id <= 3:
        return "lah"
    if 4 <= aircraft_id <= 6:
        return "uav"
    return "uav"


_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)


def _now_ms_2000() -> int:
    return int((datetime.now(timezone.utc) - _EPOCH_2000).total_seconds() * 1000)


def _coerce_int(value: Any, default: int) -> int:
    try:
        return int(value)
    except Exception:
        return default


def _coerce_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except Exception:
        return default


def _get_ci(mapping: Any, *keys: str) -> Any:
    if not isinstance(mapping, dict):
        return None
    for key in keys:
        if key in mapping:
            return mapping[key]
    lowered = {str(k).lower(): v for k, v in mapping.items()}
    for key in keys:
        value = lowered.get(str(key).lower())
        if value is not None:
            return value
    return None


def _coerce_bool(value: Any, default: bool) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return default
    if isinstance(value, (int, float)):
        return bool(int(value))
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in ("1", "true", "yes", "y", "on"):
            return True
        if lowered in ("0", "false", "no", "n", "off"):
            return False
    return default


def _wrap_heading_deg(value: float) -> float:
    return float(value) % 360.0


def _aircraft_yaw_to_nav_heading_deg(yaw_deg: float) -> float:
    # Internal aircraft yaw uses 0=east, 90=south because local +Y points north
    # while the flight model advances with vy=-sin(yaw). External consumers use
    # the navigation convention 0=north, 90=east.
    return _wrap_heading_deg(float(yaw_deg) + 90.0)


def _target_heading_to_nav_heading_deg(heading_deg: float) -> float:
    # Ground target motion uses local XY math heading: 0=east, 90=north.
    # External consumers use the navigation convention 0=north, 90=east.
    return _wrap_heading_deg(90.0 - float(heading_deg))


def _target_heading_rate_to_nav_heading_rate_dps(heading_rate_dps: float) -> float:
    # Switching from counter-clockwise-positive math heading to clockwise-
    # positive navigation heading flips the sign of the angular rate.
    return -float(heading_rate_dps)


_TURN_RADIUS_SPEED_TABLE = (
    (30.0, 340.0),
    (40.0, 450.0),
    (50.0, 560.0),
)


def _interpolate_turn_radius(speed_mps: float) -> float:
    speed = max(float(_TURN_RADIUS_SPEED_TABLE[0][0]), min(float(_TURN_RADIUS_SPEED_TABLE[-1][0]), float(speed_mps)))
    if speed <= _TURN_RADIUS_SPEED_TABLE[0][0]:
        return float(_TURN_RADIUS_SPEED_TABLE[0][1])
    if speed >= _TURN_RADIUS_SPEED_TABLE[-1][0]:
        return float(_TURN_RADIUS_SPEED_TABLE[-1][1])
    for left, right in zip(_TURN_RADIUS_SPEED_TABLE[:-1], _TURN_RADIUS_SPEED_TABLE[1:]):
        s0, r0 = left
        s1, r1 = right
        if s0 <= speed <= s1:
            alpha = (speed - s0) / max(1e-6, (s1 - s0))
            return float(r0 + ((r1 - r0) * alpha))
    return float(_TURN_RADIUS_SPEED_TABLE[1][1])


def _extract_waypoints(data: Dict[str, Any]) -> list[Dict[str, Any]]:
    for key in ("lahWaypointList", "uavWaypointList", "waypointList"):
        lst = data.get(key)
        if isinstance(lst, list):
            return lst
    return []


def _extract_coord(item: Dict[str, Any]) -> Optional[tuple[float, float, Optional[float]]]:
    coord = item.get("coordinate") or item.get("Coordinate")
    if not isinstance(coord, dict):
        return None
    lat = coord.get("latitude") if "latitude" in coord else coord.get("Latitude")
    lon = coord.get("longitude") if "longitude" in coord else coord.get("Longitude")
    alt = coord.get("altitude") if "altitude" in coord else coord.get("Altitude")
    if lat is None or lon is None:
        return None
    try:
        lat_v = float(lat)
        lon_v = float(lon)
        alt_v = float(alt) if alt is not None else None
    except Exception:
        return None
    return lat_v, lon_v, alt_v


def _override_coord_for_loiter(
    wp: Dict[str, Any],
    coord: tuple[float, float, Optional[float]],
) -> tuple[float, float, Optional[float]]:
    pass_type = wp.get("waypointPassType") or wp.get("WaypointPassType")
    try:
        pass_type_val = int(pass_type)
    except Exception:
        pass_type_val = 0
    if pass_type_val != 2:
        return coord

    filming = wp.get("filmingProperty")
    if not isinstance(filming, dict):
        return coord
    op_mode = filming.get("operationMode") or filming.get("operationalMode")
    try:
        op_mode_val = int(op_mode)
    except Exception:
        op_mode_val = 0
    if op_mode_val != 1:
        return coord

    coord_orient = filming.get("coordinateOrientation") or filming.get("CoordinateOrientation")
    if not isinstance(coord_orient, dict):
        return coord
    coord_item = coord_orient.get("coordinate") or coord_orient.get("Coordinate")
    if not isinstance(coord_item, dict):
        return coord

    lat = coord_item.get("latitude") if "latitude" in coord_item else coord_item.get("Latitude")
    lon = coord_item.get("longitude") if "longitude" in coord_item else coord_item.get("Longitude")
    alt = coord_item.get("altitude") if "altitude" in coord_item else coord_item.get("Altitude")
    if lat is None or lon is None:
        return coord
    try:
        lat_v = float(lat)
        lon_v = float(lon)
        # Loiter may be centered on the filming target, but the aircraft must
        # keep the planned flight altitude instead of descending to target/DEM height.
        alt_v = coord[2] if coord[2] is not None else (float(alt) if alt is not None else None)
    except Exception:
        return coord
    return lat_v, lon_v, alt_v


def _order_waypoints(raw: list[Dict[str, Any]]) -> list[Dict[str, Any]]:
    if not raw:
        return []
    by_id: dict[int, Dict[str, Any]] = {}
    next_ids: set[int] = set()
    for wp in raw:
        if not isinstance(wp, dict):
            continue
        wid = wp.get("waypointID") or wp.get("WaypointID")
        if wid is None:
            continue
        try:
            wid_i = int(wid)
        except Exception:
            continue
        by_id[wid_i] = wp
        nxt = wp.get("nextWaypointID") or wp.get("NextWaypointID")
        if nxt is None:
            continue
        try:
            nxt_i = int(nxt)
        except Exception:
            continue
        if nxt_i > 0:
            next_ids.add(nxt_i)

    if not by_id:
        return list(raw)

    start_id = None
    for wid in by_id:
        if wid not in next_ids:
            start_id = wid
            break

    ordered: list[Dict[str, Any]] = []
    visited: set[int] = set()
    if start_id is not None:
        curr = start_id
        while curr and curr in by_id and curr not in visited:
            wp = by_id[curr]
            ordered.append(wp)
            visited.add(curr)
            nxt = wp.get("nextWaypointID") or wp.get("NextWaypointID")
            try:
                curr = int(nxt)
            except Exception:
                break
            if curr == 0:
                break

    # Append any leftover waypoints in original order.
    for wp in raw:
        if not isinstance(wp, dict):
            continue
        wid = wp.get("waypointID") or wp.get("WaypointID")
        if wid is None:
            ordered.append(wp)
            continue
        try:
            wid_i = int(wid)
        except Exception:
            ordered.append(wp)
            continue
        if wid_i not in visited:
            ordered.append(wp)

    return ordered


def _label_to_aircraft_id(label: str) -> Optional[int]:
    text = str(label or "").strip().upper()
    if text.startswith("LAH"):
        try:
            idx = int(text.replace("LAH", ""))
        except Exception:
            return None
        return idx if 1 <= idx <= 3 else None
    if text.startswith("UAV"):
        try:
            idx = int(text.replace("UAV", ""))
        except Exception:
            return None
        return idx + 3 if 1 <= idx <= 3 else None
    return None


def _extract_hover_time(item: Dict[str, Any]) -> Optional[float]:
    hover = (
        item.get("hovering")
        or item.get("Hovering")
        or item.get("hover_prop")
        or item.get("hoveringProperty")
        or {}
    )
    if isinstance(hover, dict):
        val = hover.get("time") or hover.get("Time")
        if val is not None:
            try:
                return float(val)
            except Exception:
                return None
    for key in ("hover_time", "hoverTime", "hover"):
        if key in item and item[key] is not None:
            try:
                return float(item[key])
            except Exception:
                return None
    return None


def _extract_loiter(item: Dict[str, Any]) -> Optional[dict]:
    loiter = (
        item.get("loiter")
        or item.get("Loiter")
        or item.get("loiterProperty")
        or item.get("LoiterProperty")
        or item.get("loiter_prop")
    )
    if isinstance(loiter, dict):
        return loiter
    return None


def _normalize_loiter(item: Dict[str, Any]) -> Optional[dict]:
    loiter = _extract_loiter(item)
    pass_type = item.get("waypointPassType") or item.get("WaypointPassType")
    try:
        pass_type = int(pass_type) if pass_type is not None else None
    except Exception:
        pass_type = None

    if loiter is None and pass_type == 2:
        loiter = {"radius": 400, "time": 30, "speed": 30, "direction": 1}

    if isinstance(loiter, dict) and pass_type == 2:
        loiter.setdefault("radius", 400)
        loiter.setdefault("time", 30)
        loiter.setdefault("speed", 30)
        loiter.setdefault("direction", 1)

    return loiter


_MAX_THREAT_RANGE_M = 5000.0
_TARGET_TYPE_CONFIG: dict[int, dict[str, Any]] = {
    1: {
        "label": "\uC804\uCC28",
        "moving": True,
        "speed": (3.0, 8.0),
        "roam": 220.0,
        "weapon": WeaponParams(a_range=1200.0, omega=0.12, weapon_type=WeaponType.GUN),
    },
    2: {
        "label": "\uC7A5\uAC11\uCC28",
        "moving": True,
        "speed": (4.0, 10.0),
        "roam": 240.0,
        "weapon": WeaponParams(a_range=1400.0, omega=0.15, weapon_type=WeaponType.GUN),
    },
    3: {
        "label": "\uBC29\uC0AC\uD3EC",
        "moving": True,
        "speed": (3.0, 7.0),
        "roam": 200.0,
        "weapon": WeaponParams(a_range=1600.0, omega=0.18, weapon_type=WeaponType.GUN),
    },
    4: {
        "label": "\uACE1\uC0AC\uD3EC",
        "moving": False,
        "speed": (0.0, 0.0),
        "roam": 0.0,
        "weapon": WeaponParams(a_range=1800.0, omega=0.16, weapon_type=WeaponType.GUN),
    },
    5: {
        "label": "\uACE0\uC815\uACE0\uC0AC\uD3EC",
        "moving": False,
        "speed": (0.0, 0.0),
        "roam": 0.0,
        "weapon": WeaponParams(a_range=2600.0, omega=0.35, weapon_type=WeaponType.MISSILE, t_fire=4.0),
    },
    6: {
        "label": "\uAD70\uC778",
        "moving": True,
        "speed": (1.0, 3.0),
        "roam": 140.0,
        "weapon": WeaponParams(a_range=600.0, omega=0.05, weapon_type=WeaponType.GUN),
    },
}

_FRIENDLY_WEAPON_CONFIG: dict[str, dict[str, float | str]] = {
    "lah": {
        "range": 8000.0,
        "reload": 4.0,
        "speed": SIM_PROJECTILE_SPEED_MISSILE,
        "hit_radius": SIM_PROJECTILE_HIT_RADIUS_MISSILE,
        "kind": "missile",
    },
}

_FRIENDLY_WEAPON_TYPE_CONFIG: dict[int, dict[str, float | str]] = {
    1: {
        "range": 8000.0,
        "reload": 4.0,
        "speed": SIM_PROJECTILE_SPEED_MISSILE,
        "hit_radius": SIM_PROJECTILE_HIT_RADIUS_MISSILE,
        "kind": "missile",
    },
    2: {
        "range": 8000.0,
        "reload": 4.0,
        "speed": SIM_PROJECTILE_SPEED_MISSILE,
        "hit_radius": SIM_PROJECTILE_HIT_RADIUS_MISSILE,
        "kind": "missile",
    },
    3: {
        "range": 8000.0,
        "reload": 0.25,
        "speed": SIM_PROJECTILE_SPEED_GUN,
        "hit_radius": SIM_PROJECTILE_HIT_RADIUS_GUN,
        "kind": "gun",
    },
}

_PROJECTILE_KIND_BY_WEAPON = {
    WeaponType.GUN: "gun",
    WeaponType.MISSILE: "missile",
}

_TARGET_INFO_MATCH_RADIUS_M = 2000.0

_IMPACT_CONFIG: dict[str, dict[str, float]] = {
    "gun": {"ttl": 0.55, "radius": 45.0, "flash": 22.0},
    "missile": {"ttl": 1.1, "radius": 140.0, "flash": 70.0},
    "default": {"ttl": 0.6, "radius": 60.0, "flash": 30.0},
}

_FOOTPRINT_ASPECT = 16.0 / 9.0
_FOOTPRINT_RAY_STEP_M = 20.0
_FOOTPRINT_RAY_BINARY_STEPS = 45
_FOOTPRINT_RAY_MAX_RANGE_FACTOR = 3.0
_FOOTPRINT_RAY_EXTRA_RANGE_M = 2000.0
_FOOTPRINT_RAY_MIN_RANGE_M = 1000.0
_FOOTPRINT_MIN_RAY_STEP_M = 1.0
_FOOTPRINT_MAX_DISTANCE_M = 50_000.0
_FOOTPRINT_LINE_AXIS_RATE_DPS = 30.0
_FOOTPRINT_LINE_AXIS_MAX_BIAS_DEG = 18.0
_FOOTPRINT_MIN_PREFERRED_RIGHT_PROJECTION = 0.20
_FOOTPRINT_FOV_INTERPRETATION = "diagonal_full"
_FOOTPRINT_EARTH_RADIUS_M = 6_378_137.0
_FOOTPRINT_CORNER_DEFINITIONS = (
    (-1.0, 1.0),
    (1.0, 1.0),
    (1.0, -1.0),
    (-1.0, -1.0),
)
_FOOTPRINT_DEM_DIR = Path(DEM_DIR)
_FOOTPRINT_DEM_TILE_RE = re.compile(r"([ns])(\d+)_([ew])(\d+)", re.IGNORECASE)
_FOOTPRINT_MODEL_PIXEL_SCALE_TAG = 33550
_FOOTPRINT_MODEL_TIEPOINT_TAG = 33922
_FOOTPRINT_GDAL_NODATA_TAG = 42113


@dataclass(frozen=True)
class _FootprintDemTile:
    path: Path
    data: np.ndarray
    pixel_width_deg: float
    pixel_height_deg: float
    lon_min: float
    lon_max: float
    lat_min: float
    lat_max: float

    def contains(self, lat: float, lon: float) -> bool:
        return self.lat_min <= lat <= self.lat_max and self.lon_min <= lon <= self.lon_max

    def sample(self, lat: float, lon: float) -> float:
        if not self.contains(lat, lon):
            raise ValueError(f"Point ({lat:.6f}, {lon:.6f}) is outside {self.path.name}")

        row = (self.lat_max - lat) / self.pixel_height_deg
        col = (lon - self.lon_min) / self.pixel_width_deg
        row0 = max(0, min(self.data.shape[0] - 1, int(math.floor(row))))
        col0 = max(0, min(self.data.shape[1] - 1, int(math.floor(col))))
        row1 = min(row0 + 1, self.data.shape[0] - 1)
        col1 = min(col0 + 1, self.data.shape[1] - 1)

        fr = row - row0
        fc = col - col0
        samples = (
            (self.data[row0, col0], (1.0 - fr) * (1.0 - fc)),
            (self.data[row0, col1], (1.0 - fr) * fc),
            (self.data[row1, col0], fr * (1.0 - fc)),
            (self.data[row1, col1], fr * fc),
        )

        weighted_sum = 0.0
        weight_sum = 0.0
        for raw_value, weight in samples:
            value = float(raw_value)
            if weight <= 0.0 or not math.isfinite(value):
                continue
            weighted_sum += value * weight
            weight_sum += weight

        if weight_sum <= 0.0:
            raise ValueError(f"No valid DEM value around ({lat:.6f}, {lon:.6f})")
        return float(weighted_sum / weight_sum)

    def ground_resolution_m(self, lat: float) -> tuple[float, float]:
        dx = _FOOTPRINT_EARTH_RADIUS_M * math.cos(math.radians(lat)) * math.radians(abs(self.pixel_width_deg))
        dy = _FOOTPRINT_EARTH_RADIUS_M * math.radians(abs(self.pixel_height_deg))
        return dx, dy


@dataclass(frozen=True)
class _FootprintProjection:
    center_hit: tuple[float, float, float]
    corners: list[tuple[float, float, float]]
    ray_step_m: float


def _calculate_footprint_fov_components(
    fov_value_deg: float,
    aspect_ratio: float,
    interpretation: str = _FOOTPRINT_FOV_INTERPRETATION,
) -> tuple[float, float]:
    tan_half = math.tan(math.radians(float(fov_value_deg)) / 2.0)
    denom = math.sqrt(1.0 + (aspect_ratio * aspect_ratio))
    if interpretation == "diagonal_full":
        vertical = 2.0 * math.atan(tan_half / denom)
        horizontal = 2.0 * math.atan((aspect_ratio * tan_half) / denom)
        return math.degrees(horizontal), math.degrees(vertical)
    if interpretation == "diagonal_half":
        return _calculate_footprint_fov_components(float(fov_value_deg) * 2.0, aspect_ratio, "diagonal_full")
    if interpretation == "horizontal_full":
        horizontal = float(fov_value_deg)
        vertical = math.degrees(2.0 * math.atan(math.tan(math.radians(horizontal) / 2.0) / aspect_ratio))
        return horizontal, vertical
    if interpretation == "horizontal_half":
        return _calculate_footprint_fov_components(float(fov_value_deg) * 2.0, aspect_ratio, "horizontal_full")
    if interpretation == "vertical_full":
        vertical = float(fov_value_deg)
        horizontal = math.degrees(2.0 * math.atan(math.tan(math.radians(vertical) / 2.0) * aspect_ratio))
        return horizontal, vertical
    if interpretation == "vertical_half":
        return _calculate_footprint_fov_components(float(fov_value_deg) * 2.0, aspect_ratio, "vertical_full")
    raise ValueError(f"Unsupported footprint FOV interpretation: {interpretation}")


def _vector_norm3(x: float, y: float, z: float) -> float:
    return math.sqrt((x * x) + (y * y) + (z * z))


def _normalize3(x: float, y: float, z: float) -> tuple[float, float, float]:
    norm = _vector_norm3(x, y, z)
    if norm < 1e-9:
        raise ValueError("Zero-length vector")
    return (x / norm, y / norm, z / norm)


def _dot3(ax: float, ay: float, az: float, bx: float, by: float, bz: float) -> float:
    return (ax * bx) + (ay * by) + (az * bz)


def _cross3(ax: float, ay: float, az: float, bx: float, by: float, bz: float) -> tuple[float, float, float]:
    return (
        (ay * bz) - (az * by),
        (az * bx) - (ax * bz),
        (ax * by) - (ay * bx),
    )


def _build_footprint_camera_axes(
    origin: tuple[float, float, float],
    focus: tuple[float, float, float],
    fallback_right: tuple[float, float, float] | None = None,
    preferred_right: tuple[float, float, float] | None = None,
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float], float]:
    fwd_x = float(focus[0] - origin[0])
    fwd_y = float(focus[1] - origin[1])
    fwd_z = float(focus[2] - origin[2])
    forward = _normalize3(fwd_x, fwd_y, fwd_z)
    world_up = (0.0, 0.0, 1.0)
    right = (0.0, 0.0, 0.0)
    if preferred_right is not None:
        dot = _dot3(
            preferred_right[0],
            preferred_right[1],
            preferred_right[2],
            forward[0],
            forward[1],
            forward[2],
        )
        right = (
            preferred_right[0] - (dot * forward[0]),
            preferred_right[1] - (dot * forward[1]),
            preferred_right[2] - (dot * forward[2]),
        )
    if _vector_norm3(*right) < float(_FOOTPRINT_MIN_PREFERRED_RIGHT_PROJECTION):
        right = _cross3(forward[0], forward[1], forward[2], world_up[0], world_up[1], world_up[2])
    if _vector_norm3(*right) < 1e-8:
        if fallback_right is not None:
            dot = _dot3(
                fallback_right[0],
                fallback_right[1],
                fallback_right[2],
                forward[0],
                forward[1],
                forward[2],
            )
            right = (
                fallback_right[0] - (dot * forward[0]),
                fallback_right[1] - (dot * forward[1]),
                fallback_right[2] - (dot * forward[2]),
            )
        if _vector_norm3(*right) < 1e-8:
            right = (1.0, 0.0, 0.0)
    right = _normalize3(*right)
    up = _normalize3(*_cross3(right[0], right[1], right[2], forward[0], forward[1], forward[2]))
    return forward, right, up, _vector_norm3(fwd_x, fwd_y, fwd_z)


@lru_cache(maxsize=1)
def _footprint_dem_tile_index() -> tuple[tuple[Path, tuple[float, float, float, float]], ...]:
    if not _FOOTPRINT_DEM_DIR.exists():
        return tuple()

    tiles: list[tuple[Path, tuple[float, float, float, float]]] = []
    for tif_path in sorted(_FOOTPRINT_DEM_DIR.glob("*.tif")):
        match = _FOOTPRINT_DEM_TILE_RE.search(tif_path.stem)
        if not match:
            continue
        lat_sign = 1 if match.group(1).lower() == "n" else -1
        lon_sign = 1 if match.group(3).lower() == "e" else -1
        lat0 = lat_sign * int(match.group(2))
        lon0 = lon_sign * int(match.group(4))
        lat1 = lat0 + lat_sign
        lon1 = lon0 + lon_sign
        tiles.append((tif_path, (min(lat0, lat1), max(lat0, lat1), min(lon0, lon1), max(lon0, lon1))))
    return tuple(tiles)


@lru_cache(maxsize=8)
def _load_footprint_dem_tile(path: Path) -> _FootprintDemTile:
    with tifffile.TiffFile(path) as tif:
        page = tif.pages[0]
        tags = page.tags
        data = page.asarray().astype(np.float64)
        scale = tags[_FOOTPRINT_MODEL_PIXEL_SCALE_TAG].value
        tie = tags[_FOOTPRINT_MODEL_TIEPOINT_TAG].value
        nodata = None
        if _FOOTPRINT_GDAL_NODATA_TAG in tags:
            try:
                nodata = float(tags[_FOOTPRINT_GDAL_NODATA_TAG].value)
            except Exception:
                nodata = None

    if nodata is not None:
        data[data == nodata] = np.nan

    pixel_width_deg = float(scale[0])
    pixel_height_deg = float(scale[1])
    lon_min = float(tie[3])
    lat_max = float(tie[4])
    height, width = data.shape
    lon_max = lon_min + (pixel_width_deg * (width - 1))
    lat_min = lat_max - (pixel_height_deg * (height - 1))
    return _FootprintDemTile(
        path=path,
        data=data,
        pixel_width_deg=pixel_width_deg,
        pixel_height_deg=pixel_height_deg,
        lon_min=lon_min,
        lon_max=lon_max,
        lat_min=lat_min,
        lat_max=lat_max,
    )


def _resolve_footprint_dem_tile(lat: float, lon: float) -> _FootprintDemTile | None:
    preferred: list[Path] = []
    fallback: list[Path] = []
    for tif_path, (lat_min, lat_max, lon_min, lon_max) in _footprint_dem_tile_index():
        if lat_min <= lat <= lat_max and lon_min <= lon <= lon_max:
            preferred.append(tif_path)
        else:
            fallback.append(tif_path)

    for tif_path in (*preferred, *fallback):
        try:
            tile = _load_footprint_dem_tile(tif_path)
        except Exception:
            continue
        if tile.contains(lat, lon):
            return tile
    return None


@dataclass
class PathDefinition:
    label: str
    aircraft_id: int
    airframe: str
    path_id: int | None
    waypoints: list[dict]


@dataclass
class FormationSpec:
    leader_id: int
    dx: float
    dy: float
    dz: float
    is_leader: bool = False


def _parse_formation_spec(data: dict, aircraft_id: int) -> FormationSpec | None:
    if not _coerce_bool(_get_ci(data, "isFormationFlight"), False):
        return None
    info = _get_ci(data, "formationInfo")
    if not isinstance(info, dict):
        return None
    leader_id = _coerce_int(_get_ci(info, "leaderAircraftID"), 0)
    if leader_id <= 0:
        return None
    formation = _get_ci(info, "formation")
    if not isinstance(formation, dict):
        formation = {}
    dx = _coerce_float(_get_ci(formation, "dX", "dx", "x"), 0.0)
    dy = _coerce_float(_get_ci(formation, "dY", "dy", "y"), 0.0)
    dz = _coerce_float(_get_ci(formation, "dZ", "dz", "z"), 0.0)
    return FormationSpec(
        leader_id=int(leader_id),
        dx=float(dx),
        dy=float(dy),
        dz=float(dz),
        is_leader=int(leader_id) == int(aircraft_id),
    )


@dataclass
class SimVehicle:
    label: str
    aircraft_id: int
    airframe: str
    vehicle: object
    controller: WaypointPIDController
    path_id: int | None
    formation: FormationSpec | None = None
    formation_target: WaypointTarget | None = None
    alive: bool = True
    crashed: bool = False


@dataclass
class Projectile:
    id: int
    side: str
    kind: str
    source_kind: str
    source_id: object
    target_kind: str
    target_id: object
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    speed: float
    ttl: float
    hit_radius: float
    p_hit: float | None


@dataclass
class ImpactEffect:
    id: int
    side: str
    kind: str
    x: float
    y: float
    z: float
    age: float
    ttl: float
    radius_m: float
    flash_m: float


@dataclass
class TrackingState:
    target_id: int
    target: GroundTarget
    saved_controller: WaypointPIDController
    saved_wp_id: int | None
    tracking_controller: WaypointPIDController
    loiter_wp: WaypointTarget
    fov_deg: float
    stage: int
    start_step: int
    last_seen: float
    filming_prop: dict | None
    end_time: float | None
    advance_on_complete: bool
    manual: bool
    track_radius: float | None
    track_speed: float | None
    override_seq: int | None = None


@dataclass
class TrackingPreviewState:
    target_ids: list[int]
    chosen_target_id: int
    fov_deg: float
    filming_prop: dict | None
    start_time: float
    end_time: float


@dataclass
class RoiMock:
    id: int
    name: str
    x: float
    y: float
    z: float
    discovered_by: set[int]


@dataclass
class RoiFocusState:
    roi_id: int
    roi: RoiMock
    fov_deg: float
    start_time: float
    end_time: float


class SimulationService:
    def __init__(
        self,
        *,
        base_dt: float = SIM_BASE_DT,
        time_scale: float = SIM_TIME_SCALE,
        pos_tol: float = SIM_POS_TOL,
        speed_uav: float = SIM_SPEED_UAV,
        speed_lah: float = SIM_SPEED_LAH,
    ) -> None:
        self.base_dt = float(base_dt)
        self.time_scale = float(time_scale)
        requested_dt = float(self.base_dt * self.time_scale)
        internal_step_hz = max(0.1, float(SIM_INTERNAL_STEP_HZ))
        self.dt = max(1e-4, min(requested_dt, 1.0 / internal_step_hz))
        self.pos_tol = float(pos_tol)
        self.speed_uav = float(speed_uav)
        self.speed_lah = float(speed_lah)
        self.uav_detection_range_m = float(SIM_UAV_DETECT_RANGE_M)
        self.track_loiter_radius_m = float(SIM_TRACK_LOITER_RADIUS_M)
        self.track_loiter_speed_mps = float(SIM_TRACK_LOITER_SPEED_MPS)
        self.track_alt_buffer_m = float(SIM_TRACK_ALT_BUFFER_M)
        self.track_lost_timeout_s = float(SIM_TRACK_LOST_TIMEOUT_S)
        self.auto_track_always = bool(SIM_AUTO_TRACK_ALWAYS)
        self.auto_track_takeover = bool(SIM_AUTO_TRACK_TAKEOVER)
        self.input_advance_guard_sec = max(0.0, float(SIM_INPUT_ADVANCE_GUARD_SEC))
        self.multi_target_preview_sec = max(0.0, float(SIM_MULTI_TARGET_PREVIEW_SEC))
        self.roi_gaze_duration_s = max(0.0, float(SIM_ROI_GAZE_DURATION_S))
        self.lah_auto_attack = bool(SIM_LAH_AUTO_ATTACK)
        self.enemy_hit_scale = float(SIM_ENEMY_HIT_SCALE)
        self.friendly_hit_scale = float(SIM_FRIENDLY_HIT_SCALE)

        self._lock = threading.RLock()
        self._shutdown = threading.Event()
        self._thread: threading.Thread | None = None

        self.running = False
        self.paused = True
        self.speed_factor = 1.0
        self._speed_change_seq = 0
        self.sim_time = 0.0
        self.step_count = 0
        self._sim_timestamp_anchor_ms_2000 = int(_now_ms_2000())
        self.last_error: str | None = None

        self.geo: GeoConverter | None = None
        self._paths: list[PathDefinition] = []
        self.vehicles: dict[str, SimVehicle] = {}
        self._block_indices: dict[int, dict[int, int]] = {}
        self._spawn_by_aircraft: dict[int, tuple[float, float, float]] = {}
        self._operation_handlers: dict[int, OperationMode] = {}
        self._filming_props: dict[str, dict | None] = {}
        self._filming_targets: dict[str, tuple[float, float, float] | None] = {}
        self._filming_wp_ids: dict[str, int | None] = {}
        self._filming_max_sep_m: dict[str, float | None] = {}
        self._line_search_state: dict[str, object | None] = {}
        self._line_search_debug: dict[str, object | None] = {}
        self._footprint_line_right_axis: dict[str, tuple[float, float, float]] = {}
        self._tracking_state: dict[str, TrackingState] = {}
        self._tracking_preview_state: dict[str, TrackingPreviewState] = {}
        self._tracking_target_owner: dict[int, str] = {}
        self._tracking_overrides: dict[str, dict[str, Any]] = {}
        self._tracking_override_seq = 0
        self._virtual_targets: dict[int, GroundTarget] = {}
        self._roi_mocks: list[RoiMock] = []
        self._pending_roi_mocks: list[dict[str, Any]] = []
        self._roi_mock_id_seq = 1
        self._roi_focus_state: dict[str, RoiFocusState] = {}
        self._formation_by_aircraft: dict[int, FormationSpec] = {}
        self._formation_by_path_id: dict[int, FormationSpec] = {}
        self._history = deque(maxlen=int(SIM_HISTORY_MAX))
        self._history_sample_hz = max(1.0, float(SIM_HISTORY_SAMPLE_HZ))
        self._history_response_max = max(1, int(SIM_HISTORY_RESPONSE_MAX))
        self._max_steps_per_loop = max(1, int(SIM_MAX_STEPS_PER_LOOP))
        self._last_history_record_sim_time: float | None = None
        self._events_0402 = deque(maxlen=int(SIM_0402_HISTORY_MAX))
        self._reported_0402_roi: set[tuple[int, int]] = set()
        self._reported_0402_list: set[tuple[int, int]] = set()
        self._reported_0402_destroyed: set[int] = set()
        self._target_id_map_0402: dict[int, int] = {}
        self._target_id_seq_0402 = 7
        self.input_mission_order_by_aircraft: dict[int, list[int]] = {}
        self.current_input_mission_idx_by_aircraft: dict[int, int] = {}
        self._pending_input_advances: dict[int, int] = {}
        self._input_advance_guard_until: dict[str, float] = {}
        self.targets: list[GroundTarget] = []
        self._target_counts: dict[int, int] = {}
        self._target_id_seq = 1
        self._pending_targets: list[dict[str, Any]] = []
        self._projectiles: list[Projectile] = []
        self._projectile_id_seq = 1
        self._last_enemy_fire: dict[int, float] = {}
        self._last_vehicle_fire: dict[str, float] = {}
        self._friendly_attack_attempts: dict[tuple[str, int], int] = {}
        self._attack_holds: dict[str, dict[str, object]] = {}
        self._destroyed_target_ids: set[int] = set()
        self._effects: list[ImpactEffect] = []
        self._effect_id_seq = 1
        self.integration = None
        self._0401_active_interval_sec = 1.0 / max(0.1, float(SIM_0401_ACTIVE_HZ))
        self._last_0401_emit_wall_time: float | None = None
        self._last_0402_sim_time: float | None = None
        self._target_watcher_0402: dict[int, int] = {}
        self._on_mission_pulse: dict[str, float] = {}
        self._filming_pulse: dict[str, float] = {}
        self._agent_overrides: dict[str, dict[str, Any]] = {}
        self._forced_commands: dict[str, dict[str, Any]] = {}
        self._rtb_coord_cache: tuple[float, float, float] | None = None
        self._initial_aircraft_ids: set[int] = set()
        self._terrain_elev_fn = None
        self._target_info_cache_mtime_ns: int | None = None
        self._target_info_cache_map: dict[str, dict[str, Any]] = {}
        self._loaded_db_root: str | None = None

    def _ensure_thread(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def shutdown(self) -> None:
        self._shutdown.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def set_integration(self, integration) -> None:
        self.integration = integration

    def set_speed_factor(self, value: float) -> float:
        try:
            factor = float(value)
        except Exception:
            factor = 1.0
        factor = max(0.1, min(30.0, factor))
        with self._lock:
            self.speed_factor = factor
            self._speed_change_seq += 1
            self._last_history_record_sim_time = None
        return factor

    def _sync_sim_timestamp_anchor(self) -> None:
        """Align simulation time origin to the current wall-clock timestamp."""
        sim_ms = max(0.0, float(self.sim_time)) * 1000.0
        self._sim_timestamp_anchor_ms_2000 = int(round(_now_ms_2000() - sim_ms))

    def _sim_timestamp_ms_2000(self, sim_time: float | None = None) -> int:
        ts_sim = float(self.sim_time if sim_time is None else sim_time)
        if ts_sim < 0.0:
            ts_sim = 0.0
        anchor = int(getattr(self, "_sim_timestamp_anchor_ms_2000", 0) or 0)
        if anchor <= 0:
            anchor = int(round(_now_ms_2000() - (ts_sim * 1000.0)))
            self._sim_timestamp_anchor_ms_2000 = anchor
        return int(round(anchor + (ts_sim * 1000.0)))

    def _clear_tracking_preview(self, label: str) -> None:
        self._tracking_preview_state.pop(str(label), None)

    def _arm_input_advance_guard(self, aircraft_id: int) -> None:
        duration = float(self.input_advance_guard_sec)
        if duration <= 0.0:
            return
        label = _agent_label(int(aircraft_id))
        until = float(self.sim_time) + duration
        prev = self._input_advance_guard_until.get(label)
        if prev is None or float(prev) < until:
            self._input_advance_guard_until[label] = until

    def _clear_input_advance_guard(self, aircraft_id: int) -> None:
        self._input_advance_guard_until.pop(_agent_label(int(aircraft_id)), None)

    def update_agent_state(self, label: str, payload: dict) -> dict:
        label = str(label or "").strip().upper()
        if not label:
            return {"ok": False, "error": "agent label required"}
        if not isinstance(payload, dict):
            payload = {}
        with self._lock:
            simv = self.vehicles.get(label)
            if simv is None:
                return {"ok": False, "error": "unknown agent"}
            if not simv.alive or self._agent_overrides.get(label, {}).get("health") == 2:
                return {"ok": False, "error": "agent is not controllable"}
            overrides = dict(self._agent_overrides.get(label, {}))
            manned_info = payload.get("mannedInfo") if isinstance(payload.get("mannedInfo"), dict) else {}
            unmanned_info = payload.get("unmannedInfo") if isinstance(payload.get("unmannedInfo"), dict) else {}

            if "health" in payload:
                overrides["health"] = _coerce_int(payload.get("health"), overrides.get("health", 1))

            if "fuelConsumption" in payload:
                overrides["fuelConsumption"] = _coerce_float(
                    payload.get("fuelConsumption"), overrides.get("fuelConsumption", 1.0)
                )

            payload_health = payload.get("payloadHealth", unmanned_info.get("payloadHealth"))
            if payload_health is not None:
                overrides["payloadHealth"] = _coerce_int(payload_health, overrides.get("payloadHealth", 1))

            fuel_warn = payload.get("fuelWarning", unmanned_info.get("fuelWarning"))
            if fuel_warn is not None:
                overrides["fuelWarning"] = _coerce_int(fuel_warn, overrides.get("fuelWarning", 0))

            weapons = payload.get("weapons", manned_info.get("weapons"))
            if isinstance(weapons, dict):
                weapon_state = dict(overrides.get("weapons", {}))
                for key in ("type1", "type2", "type3"):
                    if key in weapons:
                        weapon_state[key] = _coerce_int(weapons.get(key), weapon_state.get(key, 0))
                overrides["weapons"] = weapon_state

            link_payload = payload.get("datalink")
            if link_payload is None:
                link_payload = payload.get("datalinkStatus")
            if link_payload is None:
                link_payload = manned_info.get("datalinkStatus")
            if isinstance(link_payload, dict):
                link_state = dict(overrides.get("datalink", {}))
                if "uav1" in link_payload or "isConnectedToUAV1" in link_payload:
                    link_state["uav1"] = _coerce_bool(
                        link_payload.get("uav1", link_payload.get("isConnectedToUAV1")), True
                    )
                if "uav2" in link_payload or "isConnectedToUAV2" in link_payload:
                    link_state["uav2"] = _coerce_bool(
                        link_payload.get("uav2", link_payload.get("isConnectedToUAV2")), True
                    )
                if "uav3" in link_payload or "isConnectedToUAV3" in link_payload:
                    link_state["uav3"] = _coerce_bool(
                        link_payload.get("uav3", link_payload.get("isConnectedToUAV3")), True
                    )
                overrides["datalink"] = link_state

            self._agent_overrides[label] = overrides
            orbit_fault = payload.get("orbitFault")

            if _coerce_int(overrides.get("health"), 0) == 2:
                self._apply_vehicle_hit(simv)
                return {"ok": True, "agent": label}

            if orbit_fault is not None:
                if simv.airframe != "uav":
                    return {"ok": False, "error": "orbit fault is only supported for UAV"}
                if isinstance(orbit_fault, dict):
                    circles = _coerce_int(orbit_fault.get("circles"), 5)
                else:
                    circles = 5
                circles = max(1, min(12, int(circles or 5)))
                if not self._force_orbit_fault(simv, circles=circles):
                    return {"ok": False, "error": "failed to arm orbit fault"}
                return {"ok": True, "agent": label, "orbitFault": {"circles": circles}}

        return {"ok": True, "agent": label}

    def _resolve_rtb_coordinate(self) -> tuple[float, float, float] | None:
        if self._rtb_coord_cache is not None:
            return self._rtb_coord_cache
        root = Path(__file__).resolve().parents[3]
        candidate = (
            root
            / "Logs"
            / "Scenario_2026-02-03T191349"
            / "SBC3"
            / "MissionReferenceInfo"
            / "0.json"
        )
        if not candidate.exists():
            return None
        try:
            import json as _json

            data = _json.loads(candidate.read_text(encoding="utf-8"))
        except Exception:
            return None
        coords = data.get("rtbCoordinateList") if isinstance(data, dict) else None
        if not isinstance(coords, list) or not coords:
            return None
        first = coords[0] if isinstance(coords[0], dict) else None
        if not first:
            return None
        try:
            lat = float(first.get("latitude"))
            lon = float(first.get("longitude"))
            alt = float(first.get("altitude") or 0.0)
        except Exception:
            return None
        self._rtb_coord_cache = (lat, lon, alt)
        return self._rtb_coord_cache

    def _clear_forced(self, simv: SimVehicle, *, restore: bool) -> None:
        entry = self._forced_commands.pop(simv.label, None)
        if not entry:
            return
        if restore:
            saved = entry.get("saved_controller")
            if isinstance(saved, WaypointPIDController):
                simv.controller = saved

    def _make_loiter_controller(
        self,
        simv: SimVehicle,
        *,
        center: tuple[float, float, float],
        duration: float,
        radius: float,
        speed: float,
        wp_id: int | None = None,
        direction: int = 1,
    ) -> WaypointPIDController:
        loiter_prop = {
            "time": float(duration),
            "radius": float(radius),
            "speed": float(speed),
            "direction": int(direction),
        }
        loiter_wp = WaypointTarget(
            pos=center,
            speed=float(speed),
            loiter=loiter_prop,
            filming=None,
            wp_id=int(wp_id) if wp_id is not None else None,
        )
        base_ctrl = simv.controller
        return WaypointPIDController(
            simv.vehicle,
            [loiter_wp],
            gains=base_ctrl.gains,
            speed_target=float(speed),
            pos_tol=float(self.pos_tol),
            name=base_ctrl.name,
            allow_hover=getattr(base_ctrl, "allow_hover", False),
        )

    def _make_rtb_controller(
        self,
        simv: SimVehicle,
        *,
        rtb: tuple[float, float, float],
    ) -> WaypointPIDController | None:
        geo = self.geo
        if geo is None:
            return None
        lat, lon, alt = rtb
        try:
            x, y = geo.lonlat_to_xy(float(lon), float(lat))
        except Exception:
            return None
        alt_val = float(alt)
        base_ctrl = simv.controller
        speed = float(getattr(base_ctrl, "speed_target", self.speed_uav))
        wp1 = WaypointTarget(pos=(float(x), float(y), alt_val), speed=speed, filming=None)
        wp2 = WaypointTarget(pos=(float(x), float(y), 0.0), speed=speed, filming=None)
        return WaypointPIDController(
            simv.vehicle,
            [wp1, wp2],
            gains=base_ctrl.gains,
            speed_target=speed,
            pos_tol=float(self.pos_tol),
            name=base_ctrl.name,
            allow_hover=getattr(base_ctrl, "allow_hover", False),
        )

    def _force_hold(self, simv: SimVehicle) -> bool:
        if simv.label in self._tracking_state:
            self._stop_tracking(simv)
        self._clear_tracking_preview(simv.label)
        self._clear_forced(simv, restore=True)
        s = simv.vehicle.s
        base_ctrl = simv.controller
        radius = 300.0
        speed = float(getattr(base_ctrl, "speed_target", self.speed_uav))
        center = (float(s.x), float(s.y), float(s.z))
        hold_ctrl = self._make_loiter_controller(
            simv,
            center=center,
            duration=50.0,
            radius=radius,
            speed=speed,
        )
        hold_ctrl.is_hovering = False
        hold_ctrl.force_hover = False
        hold_ctrl.is_loitering = True
        hold_ctrl.loiter_timer = 50.0
        hold_ctrl.loiter_center = center
        hold_ctrl.loiter_radius = float(radius)
        hold_ctrl.loiter_speed = float(speed)
        try:
            hold_ctrl.loiter_angle = math.atan2(s.y - center[1], s.x - center[0])
        except Exception:
            hold_ctrl.loiter_angle = 0.0
        simv.controller = hold_ctrl
        self._forced_commands[simv.label] = {
            "type": "hold",
            "saved_controller": base_ctrl,
            "controller": hold_ctrl,
            "end_time": float(self.sim_time) + 50.0,
            "flight_mode": 8,
            "block_mission": False,
        }
        return True

    def _force_hold_custom(
        self,
        simv: SimVehicle,
        *,
        center: tuple[float, float, float],
        duration: float,
        radius: float,
        speed: float,
        angle: float | None = None,
    ) -> bool:
        if simv.label in self._tracking_state:
            self._stop_tracking(simv)
        self._clear_tracking_preview(simv.label)
        self._clear_forced(simv, restore=True)
        base_ctrl = simv.controller
        hold_ctrl = self._make_loiter_controller(
            simv,
            center=center,
            duration=duration,
            radius=radius,
            speed=speed,
        )
        hold_ctrl.is_hovering = False
        hold_ctrl.force_hover = False
        hold_ctrl.is_loitering = True
        hold_ctrl.loiter_timer = float(duration)
        hold_ctrl.loiter_center = center
        hold_ctrl.loiter_radius = float(radius)
        hold_ctrl.loiter_speed = float(speed)
        if angle is not None:
            hold_ctrl.loiter_angle = float(angle)
        else:
            try:
                s = simv.vehicle.s
                hold_ctrl.loiter_angle = math.atan2(s.y - center[1], s.x - center[0])
            except Exception:
                hold_ctrl.loiter_angle = 0.0
        simv.controller = hold_ctrl
        self._forced_commands[simv.label] = {
            "type": "hold",
            "saved_controller": base_ctrl,
            "controller": hold_ctrl,
            "end_time": float(self.sim_time) + float(duration),
            "flight_mode": 8,
            "block_mission": False,
        }
        return True

    def _resolve_orbit_fault_anchor(
        self,
        simv: SimVehicle,
    ) -> tuple[tuple[float, float, float], int | None]:
        candidates: list[WaypointPIDController] = []
        tracking = self._tracking_state.get(simv.label)
        if tracking is not None and isinstance(getattr(tracking, "saved_controller", None), WaypointPIDController):
            candidates.append(tracking.saved_controller)
        if isinstance(simv.controller, WaypointPIDController):
            candidates.append(simv.controller)
        for ctrl in candidates:
            try:
                target = ctrl.current_target()
            except Exception:
                target = None
            if target is None:
                continue
            pos = getattr(target, "pos", None)
            if isinstance(pos, (list, tuple)) and len(pos) == 3:
                wp_id = getattr(target, "wp_id", None)
                try:
                    wp_id_val = int(wp_id) if wp_id is not None else None
                except Exception:
                    wp_id_val = None
                return (float(pos[0]), float(pos[1]), float(pos[2])), wp_id_val
        s = simv.vehicle.s
        return (float(s.x), float(s.y), float(s.z)), None

    def _force_orbit_fault(self, simv: SimVehicle, *, circles: int = 5) -> bool:
        if simv.airframe != "uav":
            return False
        if simv.label in self._tracking_state:
            self._stop_tracking(simv)
        self._clear_tracking_preview(simv.label)
        self._clear_forced(simv, restore=True)
        base_ctrl = simv.controller
        center, current_wp_id = self._resolve_orbit_fault_anchor(simv)
        s = simv.vehicle.s
        speed_candidates = (
            getattr(s, "u", None),
            getattr(base_ctrl, "speed_target", None),
            self.speed_uav,
        )
        speed = None
        for candidate in speed_candidates:
            try:
                numeric = float(candidate)
            except Exception:
                continue
            if math.isfinite(numeric) and numeric > 1.0:
                speed = numeric
                break
        if speed is None:
            speed = float(self.speed_uav)
        radius = float(_interpolate_turn_radius(speed))
        circle_time = (2.0 * math.pi * radius) / max(1.0, float(speed))
        duration = max(circle_time, float(circles) * circle_time)
        orbit_ctrl = self._make_loiter_controller(
            simv,
            center=center,
            duration=duration,
            radius=radius,
            speed=float(speed),
            wp_id=current_wp_id,
            direction=1,
        )
        orbit_ctrl.is_hovering = False
        orbit_ctrl.force_hover = False
        orbit_ctrl.is_loitering = True
        orbit_ctrl.loiter_timer = float(duration)
        orbit_ctrl.loiter_center = center
        orbit_ctrl.loiter_radius = float(radius)
        orbit_ctrl.loiter_speed = float(speed)
        try:
            orbit_ctrl.loiter_angle = math.atan2(float(s.y) - center[1], float(s.x) - center[0])
        except Exception:
            orbit_ctrl.loiter_angle = 0.0
        simv.controller = orbit_ctrl
        self._forced_commands[simv.label] = {
            "type": "orbit_fault",
            "saved_controller": base_ctrl,
            "controller": orbit_ctrl,
            "end_time": float(self.sim_time) + float(duration),
            "flight_mode": 8,
            "block_mission": False,
            "current_wp_id": int(current_wp_id) if current_wp_id is not None else 0,
            "circles": int(circles),
        }
        return True

    def _force_rtb(self, simv: SimVehicle) -> bool:
        if simv.label in self._tracking_state:
            self._stop_tracking(simv)
        self._clear_tracking_preview(simv.label)
        self._clear_forced(simv, restore=True)
        rtb = self._resolve_rtb_coordinate()
        if rtb is None:
            return False
        base_ctrl = simv.controller
        rtb_ctrl = self._make_rtb_controller(simv, rtb=rtb)
        if rtb_ctrl is None:
            return False
        simv.controller = rtb_ctrl
        self._forced_commands[simv.label] = {
            "type": "rtb",
            "saved_controller": base_ctrl,
            "controller": rtb_ctrl,
            "end_time": None,
            "flight_mode": 5,
            "block_mission": True,
            "completed": False,
        }
        return True

    def _force_resume(self, simv: SimVehicle) -> bool:
        forced = self._forced_commands.get(simv.label)
        if not forced:
            return False
        self._clear_forced(simv, restore=True)
        return True

    def apply_force_command(self, aircraft_id: int, mandatory_type: int) -> dict:
        label = _agent_label(aircraft_id)
        with self._lock:
            simv = self.vehicles.get(label)
            if simv is None:
                return {"ok": False, "error": "unknown agent"}
            try:
                mtype = int(mandatory_type)
            except Exception:
                mtype = 0
            if mtype == 1:
                ok = self._force_hold(simv)
            elif mtype == 2:
                ok = self._force_rtb(simv)
            elif mtype == 3:
                ok = self._force_resume(simv)
            else:
                ok = False
        return {"ok": bool(ok), "aircraftID": int(aircraft_id), "mandatoryType": int(mtype)}

    def _apply_forced_state(self, simv: SimVehicle) -> bool:
        forced = self._forced_commands.get(simv.label)
        if not forced:
            return False
        ftype = forced.get("type")
        if ftype in ("hold", "orbit_fault"):
            end_time = forced.get("end_time")
            if end_time is not None and float(self.sim_time) >= float(end_time):
                self._clear_forced(simv, restore=True)
                return False
            return True
        if ftype == "rtb":
            if simv.controller.finished:
                forced["completed"] = True
            return True
        return False

    def play(self) -> dict:
        with self._lock:
            if not self.vehicles:
                return {"ok": False, "error": "no mission loaded"}
            self.running = True
            self.paused = False
            self._last_0401_emit_wall_time = None
        self._ensure_thread()
        return {"ok": True, "systemModePush": self._push_system_mode_0101(3)}

    def _push_system_mode_0101(self, system_mode: int) -> dict:
        integration = getattr(self, "integration", None)
        if integration is None:
            return {"ok": False, "error": "integration unavailable"}
        payload = {
            "timestamp": int(_now_ms_2000()),
            "source": "SIM",
            "systemMode": int(system_mode),
        }
        try:
            result = integration.send_custom("0101", payload)
        except Exception as exc:
            return {"ok": False, "error": str(exc), "payload": payload}
        if isinstance(result, dict):
            result = dict(result)
            result.setdefault("payload", payload)
            return result
        return {"ok": False, "error": "unexpected integration result", "payload": payload}

    def pause(self) -> dict:
        with self._lock:
            self.paused = True
            self._last_0401_emit_wall_time = None
        return {"ok": True}

    def stop(self) -> dict:
        with self._lock:
            self.running = False
            self.paused = True
            self._last_0401_emit_wall_time = None
        self.reset()
        return {"ok": True}

    def clear(self) -> dict:
        with self._lock:
            self.running = False
            self.paused = True
            self.sim_time = 0.0
            self.step_count = 0
            self._sync_sim_timestamp_anchor()
            self.last_error = None
            self.geo = None
            self._paths = []
            self.vehicles = {}
            self._block_indices = {}
            self._spawn_by_aircraft = {}
            self.input_mission_order_by_aircraft = {}
            self.current_input_mission_idx_by_aircraft = {}
            self._pending_input_advances = {}
            self._input_advance_guard_until = {}
            self._filming_props = {}
            self._filming_targets = {}
            self._filming_wp_ids = {}
            self._filming_max_sep_m = {}
            self._line_search_state = {}
            self._line_search_debug = {}
            self._footprint_line_right_axis = {}
            self._tracking_preview_state = {}
            self._history.clear()
            self._last_history_record_sim_time = None
            self._reset_persistent_target_detection_state()
            self._reset_0402_state()
            self._tracking_overrides = {}
            self._tracking_override_seq = 0
            self._virtual_targets = {}
            self._roi_mocks = []
            self._pending_roi_mocks = []
            self._roi_mock_id_seq = 1
            self._roi_focus_state = {}
            self._formation_by_aircraft = {}
            self._formation_by_path_id = {}
            self.targets = []
            self._target_counts = {}
            self._target_id_seq = 1
            pending = list(self._pending_targets)
            self._pending_targets = []
            self._pending_targets = []
            self._projectiles = []
            self._projectile_id_seq = 1
            self._last_enemy_fire = {}
            self._last_vehicle_fire = {}
            self._friendly_attack_attempts = {}
            self._attack_holds = {}
            self._destroyed_target_ids = set()
            self._effects = []
            self._effect_id_seq = 1
            self._last_0401_emit_wall_time = None
            self._last_0402_sim_time = None
            self._on_mission_pulse = {}
            self._filming_pulse = {}
            self._pending_input_advances = {}
            self._input_advance_guard_until = {}
            self._agent_overrides = {}
            self._forced_commands = {}
            self._rtb_coord_cache = None
            self._initial_aircraft_ids = set()
        return {"ok": True}

    def reset(self) -> dict:
        with self._lock:
            self.sim_time = 0.0
            self.step_count = 0
            self._sync_sim_timestamp_anchor()
            if self._paths:
                self._build_vehicles(self._paths)
            for tgt in self.targets:
                try:
                    tgt.reset()
                except Exception:
                    continue
            self._history.clear()
            self._last_history_record_sim_time = None
            self._reset_persistent_target_detection_state()
            self._reset_0402_state()
            self._tracking_preview_state = {}
            self._tracking_overrides = {}
            self._tracking_override_seq = 0
            self._virtual_targets = {}
            self._roi_focus_state = {}
            for roi in self._roi_mocks:
                roi.discovered_by.clear()
            self._formation_by_aircraft = {}
            self._formation_by_path_id = {}
            self._projectiles = []
            self._projectile_id_seq = 1
            self._last_enemy_fire = {}
            self._last_vehicle_fire = {}
            self._friendly_attack_attempts = {}
            self._attack_holds = {}
            self._destroyed_target_ids = set()
            self._effects = []
            self._effect_id_seq = 1
            self._last_0401_emit_wall_time = None
            self._last_0402_sim_time = None
            self._on_mission_pulse = {}
            self._filming_pulse = {}
            self._agent_overrides = {}
            self._forced_commands = {}
            self._rtb_coord_cache = None
        return {"ok": True}

    def _reset_0402_state(self) -> None:
        self._events_0402.clear()
        self._reported_0402_roi = set()
        self._reported_0402_list = set()
        self._reported_0402_destroyed = set()
        self._target_id_map_0402 = {}
        self._target_id_seq_0402 = 7
        self._target_watcher_0402 = {}
        self._tracking_target_owner = {}

    @staticmethod
    def _same_db_root(left: str | None, right: str | None) -> bool:
        if not left or not right:
            return False
        try:
            return Path(str(left)).resolve() == Path(str(right)).resolve()
        except Exception:
            return str(left).replace("\\", "/").rstrip("/").lower() == str(right).replace("\\", "/").rstrip("/").lower()

    def _reset_target_state_for_fresh_load(self, *, clear_pending: bool) -> None:
        self.targets = []
        self._target_counts = {}
        self._target_id_seq = 1
        self._virtual_targets = {}
        self._roi_mocks = []
        self._roi_focus_state = {}
        if clear_pending:
            self._pending_targets = []
            self._pending_roi_mocks = []
            self._roi_mock_id_seq = 1
        self._destroyed_target_ids = set()
        self._friendly_attack_attempts = {}
        self._attack_holds = {}
        self._tracking_state = {}
        self._tracking_preview_state = {}
        self._tracking_overrides = {}
        self._tracking_target_owner = {}
        self._reset_0402_state()

    def _reset_persistent_target_detection_state(self) -> None:
        try:
            reset_target_info(clear_prior_rediscovery=True)
        except Exception:
            pass
        self._target_info_cache_mtime_ns = None
        self._target_info_cache_map = {}

    def _current_input_mission_id_for(self, aircraft_id: int) -> Optional[int]:
        order = self.input_mission_order_by_aircraft.get(aircraft_id) or []
        idx = self.current_input_mission_idx_by_aircraft.get(aircraft_id, 0)
        if idx < 0 or idx >= len(order):
            return None
        return order[idx]

    def _next_input_mission_id_for(self, aircraft_id: int) -> Optional[int]:
        order = self.input_mission_order_by_aircraft.get(aircraft_id) or []
        idx = self.current_input_mission_idx_by_aircraft.get(aircraft_id, 0) + 1
        if idx < 0 or idx >= len(order):
            return None
        return order[idx]

    def _set_current_input_idx_for_id(self, aircraft_id: int, input_id: int | None) -> bool:
        if input_id is None:
            return False
        order = self.input_mission_order_by_aircraft.get(int(aircraft_id)) or []
        for idx, mid in enumerate(order):
            if int(mid) == int(input_id):
                self.current_input_mission_idx_by_aircraft[int(aircraft_id)] = int(idx)
                return True
        return False

    def _sync_current_input_idx_to_controller_target(self, simv: SimVehicle) -> bool:
        try:
            target = simv.controller.current_target()
        except Exception:
            target = None
        input_id = _coerce_int(getattr(target, "input_mission_id", None), None)
        return self._set_current_input_idx_for_id(int(simv.aircraft_id), input_id)

    def _advance_current_input_idx_from_id(self, aircraft_id: int, input_id: int | None) -> None:
        if input_id is None:
            return
        order = self.input_mission_order_by_aircraft.get(int(aircraft_id)) or []
        if not order:
            return
        for idx, mid in enumerate(order):
            if int(mid) == int(input_id):
                self.current_input_mission_idx_by_aircraft[int(aircraft_id)] = min(
                    int(idx) + 1,
                    len(order) - 1,
                )
                return

    def _blocked_input_has_next_target(self, simv: SimVehicle, input_id: int | None) -> bool:
        if input_id is None:
            return False
        block_map = self._block_indices.get(int(simv.aircraft_id)) or {}
        end_idx = None
        for idx, mid in block_map.items():
            if int(mid) == int(input_id):
                end_idx = int(idx)
                break
        if end_idx is None:
            return False
        targets = getattr(simv.controller, "targets", None)
        return isinstance(targets, list) and int(end_idx) + 1 < len(targets)

    def _resolve_input_mission_targets(self, aircraft_id: Optional[int] = None) -> list[int]:
        targets = (
            [aircraft_id]
            if aircraft_id is not None
            else sorted(self.input_mission_order_by_aircraft.keys())
        )
        if aircraft_id is not None and _airframe_type(int(aircraft_id)) == "uav":
            # Keep the manned package synchronized with the UAV input mission transition.
            for aid in (1, 2, 3):
                if aid in self.input_mission_order_by_aircraft:
                    targets.append(aid)
        return sorted({int(t) for t in targets if t is not None})

    def _advance_input_mission_for_aircraft_locked(
        self,
        aid: int,
        *,
        allow_queue: bool,
    ) -> bool:
        cur_id = self._current_input_mission_id_for(aid)
        label = _agent_label(aid)
        simv = self.vehicles.get(label)
        if not simv:
            self._pending_input_advances.pop(int(aid), None)
            self._clear_input_advance_guard(aid)
            return False
        forced = self._forced_commands.get(label)
        if forced and forced.get("block_mission"):
            return False
        ap = simv.controller
        blocked_id = _coerce_int(getattr(ap, "blocked_input_id", None), None)
        if (
            blocked_id is not None
            and cur_id != blocked_id
            and self._blocked_input_has_next_target(simv, blocked_id)
        ):
            # 0803 can arrive after a replan or pending advance has already
            # moved the bookkeeping index. The controller block is the
            # authoritative boundary to release.
            cur_id = int(blocked_id)
            self._set_current_input_idx_for_id(aid, cur_id)
        if cur_id is None:
            self._pending_input_advances.pop(int(aid), None)
            self._clear_input_advance_guard(aid)
            return False
        has_next_by_order = self._next_input_mission_id_for(aid) is not None
        has_next_by_block = (
            blocked_id is not None
            and int(blocked_id) == int(cur_id)
            and self._blocked_input_has_next_target(simv, cur_id)
        )
        if not has_next_by_order and not has_next_by_block:
            self._pending_input_advances.pop(int(aid), None)
            self._clear_input_advance_guard(aid)
            return False
        if getattr(ap, "blocked_input_id", None) != cur_id:
            if self._commit_passed_input_mission_boundary_locked(simv, cur_id):
                self._advance_current_input_idx_from_id(aid, cur_id)
                self._pending_input_advances.pop(int(aid), None)
                self._arm_input_advance_guard(aid)
                return True
            if simv.airframe == "lah" and self._skip_to_next_input_mission(simv, cur_id):
                self._advance_current_input_idx_from_id(aid, cur_id)
                self._pending_input_advances.pop(int(aid), None)
                self._arm_input_advance_guard(aid)
                return True
            if allow_queue:
                self._pending_input_advances[int(aid)] = int(cur_id)
            return False
        try:
            ap.release_block()
        except Exception:
            if allow_queue:
                self._pending_input_advances[int(aid)] = int(cur_id)
            return False
        self._advance_current_input_idx_from_id(aid, cur_id)
        self._pending_input_advances.pop(int(aid), None)
        self._arm_input_advance_guard(aid)
        return True

    def _apply_pending_input_advances_locked(self) -> int:
        if not self._pending_input_advances:
            return 0
        advanced = 0
        for aid, expected_input_id in list(self._pending_input_advances.items()):
            current_input_id = self._current_input_mission_id_for(int(aid))
            if current_input_id is None or int(current_input_id) != int(expected_input_id):
                simv = self.vehicles.get(_agent_label(int(aid)))
                blocked_id = (
                    _coerce_int(getattr(simv.controller, "blocked_input_id", None), None)
                    if simv is not None
                    else None
                )
                if blocked_id is not None and int(blocked_id) == int(expected_input_id):
                    if self._advance_input_mission_for_aircraft_locked(int(aid), allow_queue=False):
                        advanced += 1
                        continue
                self._pending_input_advances.pop(int(aid), None)
                self._clear_input_advance_guard(int(aid))
                continue
            if self._advance_input_mission_for_aircraft_locked(int(aid), allow_queue=False):
                advanced += 1
        return advanced

    def advance_input_mission(self, aircraft_id: Optional[int] = None) -> int:
        advanced = 0
        with self._lock:
            for aid in self._resolve_input_mission_targets(aircraft_id):
                if (
                    self._current_input_mission_id_for(aid) is not None
                    and self._next_input_mission_id_for(aid) is not None
                ):
                    self._arm_input_advance_guard(aid)
                if self._advance_input_mission_for_aircraft_locked(aid, allow_queue=True):
                    advanced += 1
        return advanced

    def clear_pending_input_advances(self, aircraft_id: Optional[int] = None) -> int:
        cleared = 0
        with self._lock:
            for aid in self._resolve_input_mission_targets(aircraft_id):
                if self._pending_input_advances.pop(int(aid), None) is not None:
                    cleared += 1
                self._clear_input_advance_guard(aid)
        return cleared

    def _commit_passed_input_mission_boundary_locked(self, simv: SimVehicle, cur_id: int) -> bool:
        block_map = self._block_indices.get(int(simv.aircraft_id)) or {}
        end_idx = None
        for idx, mid in block_map.items():
            if mid == cur_id:
                end_idx = int(idx)
                break
        if end_idx is None:
            return False
        curr_idx = getattr(simv.controller, "curr_idx", None)
        if not isinstance(curr_idx, int):
            return False
        try:
            current_target = simv.controller.current_target()
        except Exception:
            current_target = None
        target_input_id = _coerce_int(getattr(current_target, "input_mission_id", None), None)
        if target_input_id is not None and int(target_input_id) != int(cur_id):
            return True
        if bool(getattr(simv.controller, "finished", False)) and int(curr_idx) >= int(end_idx):
            return True
        return int(curr_idx) > int(end_idx)

    def _skip_to_next_input_mission(self, simv: SimVehicle, cur_id: int) -> bool:
        block_map = self._block_indices.get(int(simv.aircraft_id)) or {}
        end_idx = None
        for idx, mid in block_map.items():
            if mid == cur_id:
                end_idx = int(idx)
                break
        if end_idx is None:
            return False
        ctrl = simv.controller
        targets = getattr(ctrl, "targets", None)
        if not isinstance(targets, list) or not targets:
            return False
        next_idx = min(end_idx + 1, len(targets) - 1)
        if next_idx <= int(getattr(ctrl, "curr_idx", 0)):
            return False
        try:
            ctrl.blocked = False
            ctrl.blocked_input_id = None
            ctrl._blocked_idx = None
            ctrl.is_loitering = False
            ctrl.is_hovering = False
            ctrl.force_hover = False
            ctrl.loiter_timer = 0.0
            ctrl.hover_timer = 0.0
            ctrl.curr_idx = next_idx
            try:
                ctrl._skip_done_targets()
            except Exception:
                pass
            ctrl.finished = False
            ctrl.yaw_int = 0.0
            ctrl.alt_int = 0.0
            ctrl.speed_int = 0.0
            ctrl.just_advanced = True
            ctrl.advance_reason = "skip"
        except Exception:
            return False
        return True

    def _build_target(
        self,
        *,
        type_id: int,
        x: float,
        y: float,
        z: float,
        id_override: int | None = None,
        name_override: str | None = None,
    ) -> GroundTarget:
        cfg = _TARGET_TYPE_CONFIG.get(type_id)
        if cfg is None:
            raise ValueError("invalid target type")
        label = cfg.get("label") or f"T{type_id}"
        count = self._target_counts.get(type_id, 0) + 1
        self._target_counts[type_id] = count
        name = str(name_override) if name_override else f"{label}_{count}"
        if id_override is None:
            target_id = int(self._target_id_seq)
            self._target_id_seq += 1
        else:
            target_id = int(id_override)
            if target_id >= self._target_id_seq:
                self._target_id_seq = target_id + 1
        weapon_proto = cfg.get("weapon")
        weapon = WeaponParams(**vars(weapon_proto)) if isinstance(weapon_proto, WeaponParams) else WeaponParams()
        radar = RadarParams()
        threat = AirDefenseThreat(radar=radar, weapon=weapon)
        moving = bool(cfg.get("moving", False))
        speed = cfg.get("speed") or (0.0, 0.0)
        try:
            vmin, vmax = float(speed[0]), float(speed[1])
        except Exception:
            vmin, vmax = 0.0, 0.0
        roam = float(cfg.get("roam") or 0.0)
        roam_center = (x, y) if moving and roam > 0 else None
        roam_radius = roam if moving and roam > 0 else None
        target = GroundTarget(
            id=target_id,
            type_id=int(type_id),
            name=str(name),
            x=float(x),
            y=float(y),
            z=float(z),
            moving=moving,
            vmin=vmin,
            vmax=vmax,
            roam_center=roam_center,
            roam_radius=roam_radius,
            threat=threat,
            spawn_x=float(x),
            spawn_y=float(y),
            spawn_z=float(z),
        )
        return target

    def _weapon_projectile_speed(self, weapon: WeaponParams) -> float:
        if weapon.weapon_type is WeaponType.MISSILE:
            return float(SIM_PROJECTILE_SPEED_MISSILE)
        return float(SIM_PROJECTILE_SPEED_GUN)

    def _weapon_hit_radius(self, weapon: WeaponParams) -> float:
        if weapon.weapon_type is WeaponType.MISSILE:
            return float(SIM_PROJECTILE_HIT_RADIUS_MISSILE)
        return float(SIM_PROJECTILE_HIT_RADIUS_GUN)

    @staticmethod
    def _bearing_deg(from_x: float, from_y: float, to_x: float, to_y: float) -> float:
        return math.degrees(math.atan2(to_y - from_y, to_x - from_x)) % 360.0

    def _target_fire_age(self, target_id: int) -> float | None:
        last_fire = self._last_enemy_fire.get(int(target_id))
        if last_fire is None:
            return None
        try:
            age = float(self.sim_time) - float(last_fire)
        except Exception:
            return None
        return max(0.0, age)

    def _spawn_projectile(
        self,
        *,
        side: str,
        kind: str,
        source_kind: str,
        source_id: object,
        target_kind: str,
        target_id: object,
        start: tuple[float, float, float],
        target: tuple[float, float, float],
        speed: float,
        hit_radius: float,
        max_range: float,
        p_hit: float | None = None,
        force_hit: bool = False,
    ) -> None:
        if len(self._projectiles) >= int(SIM_PROJECTILE_MAX):
            return
        sx, sy, sz = start
        tx, ty, tz = target
        dx = tx - sx
        dy = ty - sy
        dz = tz - sz
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist <= 1e-6:
            return
        speed = max(1.0, float(speed))
        force_hit = bool(force_hit) and str(side) == "friendly"
        if p_hit is None and kind == "missile" and max_range > 0:
            ratio = max(0.0, min(1.0, dist / float(max_range)))
            p_hit = 0.25 + 0.7 * (1.0 - ratio)
        if p_hit is None and kind == "gun" and str(side) == "friendly":
            p_hit = 0.6
        if force_hit:
            p_hit = 1.0
        if p_hit is not None:
            if str(side) == "friendly" and not force_hit:
                try:
                    p_hit = float(p_hit) * float(self.friendly_hit_scale)
                except Exception:
                    p_hit = float(p_hit)
            p_hit = max(0.0, min(1.0, float(p_hit)))
        vx = dx / dist * speed
        vy = dy / dist * speed
        vz = dz / dist * speed
        ttl = max(0.5, float(max_range) / speed + 1.0)
        if kind == "missile":
            ttl = max(ttl, float(max_range) / speed + 4.0)
        proj = Projectile(
            id=int(self._projectile_id_seq),
            side=str(side),
            kind=str(kind),
            source_kind=str(source_kind),
            source_id=source_id,
            target_kind=str(target_kind),
            target_id=target_id,
            x=float(sx),
            y=float(sy),
            z=float(sz),
            vx=float(vx),
            vy=float(vy),
            vz=float(vz),
            speed=float(speed),
            ttl=float(ttl),
            hit_radius=float(hit_radius),
            p_hit=p_hit,
        )
        self._projectile_id_seq += 1
        self._projectiles.append(proj)

    def _spawn_effect(self, *, side: str, kind: str, x: float, y: float, z: float) -> None:
        cfg = _IMPACT_CONFIG.get(kind, _IMPACT_CONFIG["default"])
        eff = ImpactEffect(
            id=int(self._effect_id_seq),
            side=str(side),
            kind=str(kind),
            x=float(x),
            y=float(y),
            z=float(z),
            age=0.0,
            ttl=float(cfg.get("ttl", 0.6)),
            radius_m=float(cfg.get("radius", 60.0)),
            flash_m=float(cfg.get("flash", 30.0)),
        )
        self._effect_id_seq += 1
        self._effects.append(eff)

    def _make_pending_target(self, *, type_id: int, lat: float, lon: float, alt: float) -> dict[str, Any]:
        cfg = _TARGET_TYPE_CONFIG.get(type_id)
        if cfg is None:
            raise ValueError("invalid target type")
        label = cfg.get("label") or f"T{type_id}"
        count = self._target_counts.get(type_id, 0) + 1
        self._target_counts[type_id] = count
        name = f"{label}_{count}"
        target_id = int(self._target_id_seq)
        self._target_id_seq += 1
        return {
            "id": target_id,
            "type": int(type_id),
            "name": name,
            "lat": float(lat),
            "lon": float(lon),
            "alt": float(alt),
            "moving": bool(cfg.get("moving", False)),
            "alive": True,
        }

    def _build_roi_mock(
        self,
        *,
        x: float,
        y: float,
        z: float,
        id_override: int | None = None,
        name_override: str | None = None,
    ) -> RoiMock:
        if id_override is None:
            roi_id = int(self._roi_mock_id_seq)
            self._roi_mock_id_seq += 1
        else:
            roi_id = int(id_override)
            if roi_id >= self._roi_mock_id_seq:
                self._roi_mock_id_seq = roi_id + 1
        name = str(name_override) if name_override else f"ROI_{roi_id}"
        return RoiMock(
            id=int(roi_id),
            name=name,
            x=float(x),
            y=float(y),
            z=float(z),
            discovered_by=set(),
        )

    def _make_pending_roi_mock(self, *, lat: float, lon: float, alt: float) -> dict[str, Any]:
        roi_id = int(self._roi_mock_id_seq)
        self._roi_mock_id_seq += 1
        return {
            "id": roi_id,
            "kind": "roi",
            "type": 0,
            "name": f"ROI_{roi_id}",
            "lat": float(lat),
            "lon": float(lon),
            "alt": float(alt),
            "alive": True,
        }

    def _target_to_dict(self, target: GroundTarget, geo: GeoConverter) -> dict:
        lon, lat = geo.xy_to_lonlat(target.x, target.y)
        threat = getattr(target, "threat", None)
        weapon = getattr(threat, "weapon", None) if threat is not None else None
        threat_state = getattr(threat, "state", None) if threat is not None else None
        weapon_kind = None
        weapon_range = 0.0
        weapon_reload = 0.0
        ammo = None
        detected = False
        exposure_time = 0.0
        if weapon is not None:
            weapon_kind = "missile" if weapon.weapon_type is WeaponType.MISSILE else "gun"
            weapon_range = float(getattr(weapon, "a_range", 0.0) or 0.0)
            weapon_reload = float(getattr(weapon, "reload", 0.0) or 0.0)
        if threat_state is not None:
            detected = bool(getattr(threat_state, "detected", False))
            exposure_time = float(getattr(threat_state, "t_exposed", 0.0) or 0.0)
            ammo_value = getattr(threat_state, "ammo", None)
            if ammo_value is not None:
                try:
                    ammo = int(ammo_value)
                except Exception:
                    ammo = None
        return {
            "id": int(target.id),
            "type": int(target.type_id),
            "name": str(target.name),
            "lat": float(lat),
            "lon": float(lon),
            "alt": float(target.z),
            "moving": bool(target.moving),
            "alive": bool(target.alive),
            "heading": _target_heading_to_nav_heading_deg(float(getattr(target, "heading", 0.0) or 0.0)),
            "headingRate": _target_heading_rate_to_nav_heading_rate_dps(
                float(getattr(target, "heading_rate", 0.0) or 0.0)
            ),
            "speed": float(getattr(target, "v", 0.0) or 0.0),
            "speedMin": float(getattr(target, "vmin", 0.0) or 0.0),
            "speedMax": float(getattr(target, "vmax", 0.0) or 0.0),
            "roamRadius": float(getattr(target, "roam_radius", 0.0) or 0.0) if getattr(target, "roam_radius", None) is not None else None,
            "detected": detected,
            "exposureTime": exposure_time,
            "ammo": ammo,
            "weaponKind": weapon_kind,
            "weaponRange": weapon_range,
            "reload": weapon_reload,
            "lastFireAge": self._target_fire_age(int(target.id)),
        }

    def _roi_mock_to_dict(self, roi: RoiMock, geo: GeoConverter) -> dict[str, Any]:
        lon, lat = geo.xy_to_lonlat(float(roi.x), float(roi.y))
        discovered_by = sorted(int(value) for value in roi.discovered_by if int(value) > 0)
        return {
            "id": int(roi.id),
            "kind": "roi",
            "type": 0,
            "name": str(roi.name),
            "lat": float(lat),
            "lon": float(lon),
            "alt": float(roi.z),
            "moving": False,
            "alive": True,
            "detected": bool(discovered_by),
            "discoveredBy": discovered_by,
        }

    def add_target(self, payload: dict) -> dict:
        try:
            type_id = int((payload or {}).get("type") or (payload or {}).get("targetType") or 0)
        except Exception:
            type_id = 0
        if type_id == 0:
            return {"ok": True, "skipped": True}
        lat = (payload or {}).get("lat")
        lon = (payload or {}).get("lon")
        if lon is None and "lng" in (payload or {}):
            lon = (payload or {}).get("lng")
        alt = (payload or {}).get("alt") or 0.0
        try:
            lat = float(lat)
            lon = float(lon)
            alt = float(alt) if alt is not None else 0.0
        except Exception:
            return {"ok": False, "error": "invalid coordinate"}
        with self._lock:
            if type_id not in _TARGET_TYPE_CONFIG:
                return {"ok": False, "error": "invalid target type"}
            if self.geo is None:
                pending = self._make_pending_target(type_id=type_id, lat=lat, lon=lon, alt=alt)
                self._pending_targets.append(pending)
                return {"ok": True, "queued": True, "target": pending}
            x, y = self.geo.lonlat_to_xy(lon, lat)
            target = self._build_target(type_id=type_id, x=x, y=y, z=alt)
            self.targets.append(target)
            return {"ok": True, "target": self._target_to_dict(target, self.geo)}

    def add_roi_mock(self, payload: dict) -> dict:
        lat = (payload or {}).get("lat")
        lon = (payload or {}).get("lon")
        if lon is None and "lng" in (payload or {}):
            lon = (payload or {}).get("lng")
        alt = (payload or {}).get("alt") or 0.0
        try:
            lat = float(lat)
            lon = float(lon)
            alt = float(alt) if alt is not None else 0.0
        except Exception:
            return {"ok": False, "error": "invalid coordinate"}
        with self._lock:
            if self.geo is None:
                pending = self._make_pending_roi_mock(lat=lat, lon=lon, alt=alt)
                self._pending_roi_mocks.append(pending)
                return {"ok": True, "queued": True, "roi": pending}
            x, y = self.geo.lonlat_to_xy(lon, lat)
            roi = self._build_roi_mock(x=x, y=y, z=alt)
            self._roi_mocks.append(roi)
            return {"ok": True, "roi": self._roi_mock_to_dict(roi, self.geo)}

    def clear_targets(self) -> dict:
        with self._lock:
            self.targets = []
            self._target_counts = {}
            self._target_id_seq = 1
            self._pending_targets = []
            self._roi_mocks = []
            self._pending_roi_mocks = []
            self._roi_mock_id_seq = 1
            self._roi_focus_state = {}
            self._tracking_state = {}
            self._tracking_preview_state = {}
            self._reset_persistent_target_detection_state()
            self._reset_0402_state()
            self._projectiles = []
            self._projectile_id_seq = 1
            self._last_enemy_fire = {}
            self._last_vehicle_fire = {}
            self._friendly_attack_attempts = {}
            self._effects = []
            self._effect_id_seq = 1
            self._last_0401_emit_wall_time = None
            self._last_0402_sim_time = None
        return {"ok": True}

    @staticmethod
    def _controller_progress_signature(controller: WaypointPIDController | None) -> tuple[tuple[int, int, int, int], ...]:
        if not isinstance(controller, WaypointPIDController):
            return ()
        targets = getattr(controller, "targets", None)
        if not isinstance(targets, list) or not targets:
            return ()
        signature: list[tuple[int, int, int, int]] = []
        for idx, target in enumerate(targets):
            wp_id = _coerce_int(getattr(target, "wp_id", None), None)
            input_id = _coerce_int(getattr(target, "input_mission_id", None), None)
            individual_id = _coerce_int(getattr(target, "individual_mission_id", None), None)
            path_id = _coerce_int(getattr(target, "path_id", None), None)
            signature.append((
                int(wp_id) if wp_id is not None else -int(idx + 1),
                int(input_id or 0),
                int(individual_id or 0),
                int(path_id or 0),
            ))
        return tuple(signature)

    def _capture_controller_progress(self, controller: WaypointPIDController | None) -> dict[str, Any] | None:
        signature = self._controller_progress_signature(controller)
        if not signature:
            return None
        return {
            "signature": signature,
            "curr_idx": int(getattr(controller, "curr_idx", 0) or 0),
            "closest_wp_idx": int(getattr(controller, "_closest_wp_idx", 0) or 0),
            "finished": bool(getattr(controller, "finished", False)),
            "blocked": bool(getattr(controller, "blocked", False)),
            "blocked_input_id": _coerce_int(getattr(controller, "blocked_input_id", None), None),
            "blocked_idx": _coerce_int(getattr(controller, "_blocked_idx", None), None),
            "force_hover": bool(getattr(controller, "force_hover", False)),
            "is_hovering": bool(getattr(controller, "is_hovering", False)),
            "hover_timer": float(getattr(controller, "hover_timer", 0.0) or 0.0),
            "is_loitering": bool(getattr(controller, "is_loitering", False)),
            "loiter_timer": float(getattr(controller, "loiter_timer", 0.0) or 0.0),
            "loiter_center": tuple(getattr(controller, "loiter_center", (0.0, 0.0, 0.0)) or (0.0, 0.0, 0.0)),
            "loiter_radius": float(getattr(controller, "loiter_radius", 0.0) or 0.0),
            "loiter_speed": float(getattr(controller, "loiter_speed", 0.0) or 0.0),
            "loiter_dir": float(getattr(controller, "loiter_dir", 1.0) or 1.0),
            "loiter_angle": float(getattr(controller, "loiter_angle", 0.0) or 0.0),
            "loiter_angle_locked": bool(getattr(controller, "loiter_angle_locked", False)),
        }

    def _restore_controller_progress(
        self,
        controller: WaypointPIDController | None,
        snapshot: dict[str, Any] | None,
    ) -> bool:
        if not isinstance(controller, WaypointPIDController) or not isinstance(snapshot, dict):
            return False
        current_signature = self._controller_progress_signature(controller)
        saved_signature = tuple(snapshot.get("signature") or ())
        if not current_signature or current_signature != saved_signature:
            return False
        targets = getattr(controller, "targets", None)
        if not isinstance(targets, list) or not targets:
            return False
        last_idx = max(0, len(targets) - 1)
        curr_idx = _coerce_int(snapshot.get("curr_idx"), 0)
        closest_idx = _coerce_int(snapshot.get("closest_wp_idx"), 0)
        blocked_idx = _coerce_int(snapshot.get("blocked_idx"), None)
        controller.curr_idx = max(0, min(int(curr_idx or 0), last_idx))
        controller._closest_wp_idx = max(0, min(int(closest_idx or controller.curr_idx), last_idx))
        controller.finished = bool(snapshot.get("finished", False))
        controller.blocked = bool(snapshot.get("blocked", False))
        controller.blocked_input_id = _coerce_int(snapshot.get("blocked_input_id"), None)
        controller._blocked_idx = (
            max(0, min(int(blocked_idx), last_idx))
            if blocked_idx is not None
            else None
        )
        controller.force_hover = bool(snapshot.get("force_hover", False))
        controller.is_hovering = bool(snapshot.get("is_hovering", False))
        controller.hover_timer = float(snapshot.get("hover_timer", 0.0) or 0.0)
        controller.is_loitering = bool(snapshot.get("is_loitering", False))
        controller.loiter_timer = float(snapshot.get("loiter_timer", 0.0) or 0.0)
        loiter_center = snapshot.get("loiter_center")
        if isinstance(loiter_center, (list, tuple)) and len(loiter_center) == 3:
            controller.loiter_center = (
                float(loiter_center[0]),
                float(loiter_center[1]),
                float(loiter_center[2]),
            )
        controller.loiter_radius = float(snapshot.get("loiter_radius", 0.0) or 0.0)
        controller.loiter_speed = float(snapshot.get("loiter_speed", 0.0) or 0.0)
        controller.loiter_dir = float(snapshot.get("loiter_dir", 1.0) or 1.0)
        controller.loiter_angle = float(snapshot.get("loiter_angle", 0.0) or 0.0)
        controller.loiter_angle_locked = bool(snapshot.get("loiter_angle_locked", False))
        controller.just_advanced = False
        controller.advance_reason = None
        return True

    def load_mission(self, payload: dict) -> dict:
        preserve_state = _coerce_bool(
            payload.get("preserveState")
            or payload.get("preserve_state")
            or payload.get("keepState")
            or payload.get("keep_state"),
            False,
        )
        try:
            active_db_root = str(db_paths.get_active_db_root())
        except Exception:
            active_db_root = None
        loaded_db_root = self._loaded_db_root
        first_db_root_load = bool(active_db_root and not loaded_db_root)
        fresh_db_root = bool(
            active_db_root
            and loaded_db_root
            and not self._same_db_root(active_db_root, loaded_db_root)
        )
        if first_db_root_load or fresh_db_root:
            preserve_state = False
        prev_geo = None
        prev_vehicles: dict[str, SimVehicle] = {}
        prev_tracking: dict[str, TrackingState] | None = None
        prev_tracking_owner: dict[int, str] | None = None
        prev_input_order: dict[int, list[int]] | None = None
        prev_input_idx: dict[int, int] | None = None
        prev_initial_ids: set[int] = set()
        prev_line_search_state: dict[str, object | None] | None = None
        prev_line_search_debug: dict[str, object | None] | None = None
        prev_positions: dict[int, tuple[float, float, float]] = {}
        prev_states: dict[int, dict[str, Any]] = {}
        prev_overrides: dict[str, dict[str, Any]] | None = None
        prev_sim_time: float | None = None
        prev_step_count: int | None = None
        prev_running: bool | None = None
        prev_paused: bool | None = None
        prev_last_0401_emit_wall_time: float | None = None
        prev_last_0402: float | None = None
        prev_ts_anchor: int | None = None
        prev_forced: dict[str, dict[str, Any]] | None = None
        prev_controller_progress: dict[int, dict[str, Any]] = {}
        if preserve_state:
            with self._lock:
                prev_geo = self.geo
                prev_vehicles = dict(self.vehicles)
                prev_tracking = dict(self._tracking_state)
                prev_tracking_owner = dict(self._tracking_target_owner)
                prev_input_order = {
                    key: list(value) for key, value in (self.input_mission_order_by_aircraft or {}).items()
                }
                prev_input_idx = dict(self.current_input_mission_idx_by_aircraft or {})
                prev_initial_ids = set(self._initial_aircraft_ids)
                prev_line_search_state = dict(self._line_search_state)
                prev_line_search_debug = dict(self._line_search_debug)
                if prev_geo is not None and self.vehicles:
                    for simv in self.vehicles.values():
                        s = simv.vehicle.s
                        try:
                            lon, lat = prev_geo.xy_to_lonlat(float(s.x), float(s.y))
                        except Exception:
                            continue
                        prev_positions[int(simv.aircraft_id)] = (float(lat), float(lon), float(s.z))
                        prev_states[int(simv.aircraft_id)] = {
                            "yaw": float(getattr(s, "yaw", 0.0)),
                            "speed": float(getattr(s, "u", 0.0)),
                            "alive": bool(simv.alive),
                            "crashed": bool(simv.crashed),
                            "alt": float(getattr(s, "z", 0.0)),
                        }
                        controller_snapshot = self._capture_controller_progress(
                            getattr(simv, "controller", None)
                        )
                        if controller_snapshot is not None:
                            prev_controller_progress[int(simv.aircraft_id)] = controller_snapshot
                prev_overrides = {k: dict(v) for k, v in (self._agent_overrides or {}).items()}
                prev_sim_time = float(self.sim_time)
                prev_step_count = int(self.step_count)
                prev_running = bool(self.running)
                prev_paused = bool(self.paused)
                prev_last_0401_emit_wall_time = self._last_0401_emit_wall_time
                prev_last_0402 = self._last_0402_sim_time
                prev_ts_anchor = int(getattr(self, "_sim_timestamp_anchor_ms_2000", _now_ms_2000()))
                if self._forced_commands:
                    prev_forced = {}
                    for label, entry in self._forced_commands.items():
                        ftype = entry.get("type")
                        if ftype not in ("hold", "rtb"):
                            continue
                        snapshot = {"type": ftype}
                        if ftype == "hold":
                            ctrl = entry.get("controller")
                            snapshot["center"] = getattr(ctrl, "loiter_center", None)
                            snapshot["radius"] = getattr(ctrl, "loiter_radius", None)
                            snapshot["speed"] = getattr(ctrl, "loiter_speed", None)
                            snapshot["angle"] = getattr(ctrl, "loiter_angle", None)
                            snapshot["end_time"] = entry.get("end_time")
                        prev_forced[label] = snapshot
        flight_paths = payload.get("flightPaths") or payload.get("paths") or payload.get("flightpaths")
        if not isinstance(flight_paths, list):
            return {"ok": False, "error": "flightPaths list required"}
        mission_order = payload.get("missionOrder") or {}
        input_mission_plans = payload.get("inputMissionPlans") or []
        individual_mission_plans = payload.get("individualMissionPlans") or []
        take_over_list = payload.get("takeOverInfoList")
        if not isinstance(take_over_list, list):
            ref = payload.get("missionReference") or payload.get("missionReferenceInfo") or {}
            take_over_list = ref.get("takeOverInfoList") if isinstance(ref, dict) else []
        if not isinstance(take_over_list, list):
            take_over_list = []

        paths: list[PathDefinition] = []
        all_latlons: list[tuple[float, float]] = []
        flight_by_path: dict[int, dict] = {}
        flight_by_aircraft: dict[int, list[int]] = {}
        path_sep_by_id: dict[int, float] = {}
        tracking_overrides_by_aircraft: dict[int, dict[str, Any]] = {}
        formation_by_aircraft: dict[int, FormationSpec] = {}
        formation_by_path_id: dict[int, FormationSpec] = {}
        formation_leader_waypoints: dict[int, list[dict]] = {}

        for entry in individual_mission_plans:
            if not isinstance(entry, dict):
                continue
            for im in entry.get("individualMissionList") or []:
                if not isinstance(im, dict):
                    continue
                try:
                    path_id = int(im.get("pathID"))
                except Exception:
                    path_id = None
                info = im.get("individualMissionInfo") or {}
                try:
                    sep_m = float(info.get("SEP"))
                except Exception:
                    sep_m = None
                if path_id is not None and sep_m is not None and sep_m > 0.0:
                    path_sep_by_id[int(path_id)] = float(sep_m)

        for entry in flight_paths:
            if not isinstance(entry, dict):
                continue
            data = entry.get("data") if isinstance(entry.get("data"), dict) else entry
            if not isinstance(data, dict):
                continue
            aircraft_id = data.get("aircraftID") or data.get("AircraftID") or entry.get("aircraftID")
            try:
                aircraft_id = int(aircraft_id)
            except Exception:
                aircraft_id = -1
            formation_spec = _parse_formation_spec(data, aircraft_id)
            if formation_spec is None or not formation_spec.is_leader:
                continue
            waypoints_raw = _extract_waypoints(data)
            if waypoints_raw:
                formation_leader_waypoints[int(formation_spec.leader_id)] = list(waypoints_raw)

        for entry in flight_paths:
            if not isinstance(entry, dict):
                continue
            data = entry.get("data") if isinstance(entry.get("data"), dict) else entry
            if not isinstance(data, dict):
                continue

            aircraft_id = data.get("aircraftID") or data.get("AircraftID") or entry.get("aircraftID")
            path_id = data.get("pathID") or data.get("PathID") or entry.get("pathID")
            try:
                aircraft_id = int(aircraft_id)
            except Exception:
                aircraft_id = -1
            try:
                path_id = int(path_id)
            except Exception:
                path_id = None
            formation_spec = _parse_formation_spec(data, aircraft_id)
            if formation_spec is not None:
                formation_by_aircraft[int(aircraft_id)] = formation_spec
                if path_id is not None:
                    formation_by_path_id[int(path_id)] = formation_spec

            waypoints_raw = _extract_waypoints(data)
            if not waypoints_raw and formation_spec is not None and not formation_spec.is_leader:
                waypoints_raw = list(formation_leader_waypoints.get(int(formation_spec.leader_id)) or [])
            if not waypoints_raw:
                continue
            if not _extract_waypoints(data):
                data = dict(data)
                data["waypointList"] = list(waypoints_raw)
            waypoints_raw = _order_waypoints(waypoints_raw)
            if path_id is not None:
                flight_by_path[path_id] = data
                if aircraft_id > 0:
                    flight_by_aircraft.setdefault(aircraft_id, []).append(path_id)
            path_sep_m = path_sep_by_id.get(int(path_id)) if path_id is not None else None

            waypoints: list[dict] = []
            for wp in waypoints_raw:
                if not isinstance(wp, dict):
                    continue
                coord = _extract_coord(wp)
                if coord is None:
                    continue
                lat, lon, alt = _override_coord_for_loiter(wp, coord)
                all_latlons.append((lon, lat))
                speed = wp.get("speed")
                try:
                    speed = float(speed) if speed is not None else None
                except Exception:
                    speed = None
                wp_id = wp.get("waypointID") or wp.get("WaypointID")
                try:
                    wp_id = int(wp_id) if wp_id is not None else None
                except Exception:
                    wp_id = None
                pass_type = wp.get("waypointPassType") or wp.get("WaypointPassType")
                try:
                    pass_type = int(pass_type) if pass_type is not None else None
                except Exception:
                    pass_type = None
                hover_time = _extract_hover_time(wp)
                loiter = _normalize_loiter(wp)
                wp_is_done = _coerce_bool(wp.get("isDone"), False)
                filming = wp.get("filmingProperty")
                if aircraft_id >= 4 and not wp_is_done:
                    override = self._build_tracking_override(filming, loiter)
                    if override:
                        tracking_overrides_by_aircraft[aircraft_id] = override
                attack = (
                    wp.get("attack")
                    or wp.get("Attack")
                    or wp.get("attackProperty")
                    or wp.get("attack_prop")
                )
                waypoints.append(
                    {
                        "lat": lat,
                        "lon": lon,
                        "alt": alt,
                        "speed": speed,
                        "wp_id": wp_id,
                        "is_done": wp_is_done,
                        "hover_time": hover_time,
                        "loiter": loiter,
                        "pass_type": pass_type,
                        "filming": filming,
                        "attack": attack if isinstance(attack, dict) else None,
                        "path_id": path_id,
                        "sep_m": path_sep_m,
                    }
                )

            if len(waypoints) < 2:
                continue

            label = _agent_label(aircraft_id)
            airframe = _airframe_type(aircraft_id)
            paths.append(
                PathDefinition(
                    label=label,
                    aircraft_id=aircraft_id,
                    airframe=airframe,
                    path_id=path_id,
                    waypoints=waypoints,
                )
            )

        prev_target_path_by_aircraft: dict[int, int | None] = {}
        if preserve_state and prev_vehicles:
            for prev_simv in prev_vehicles.values():
                try:
                    prev_target = prev_simv.controller.current_target()
                except Exception:
                    prev_target = None
                prev_target_path_by_aircraft[int(prev_simv.aircraft_id)] = _coerce_int(
                    getattr(prev_target, "path_id", None),
                    None,
                )

        latest_wp_by_aircraft: dict[int, int] = {}
        try:
            latest_path = Path(str(active_db_root or db_paths.get_active_db_root())) / "DSS_Internal" / "latest_0401_agent_status.json"
            latest = json.loads(latest_path.read_text(encoding="utf-8"))
            states = (
                latest.get("agent_states")
                or latest.get("agentStateList")
                or (latest.get("raw") or {}).get("agentStateList")
                or []
            )
            if isinstance(states, list):
                for state in states:
                    if not isinstance(state, dict):
                        continue
                    aid = _coerce_int(_get_ci(state, "aircraftID", "AircraftID"), 0)
                    if aid <= 0:
                        continue
                    info = _get_ci(state, "unmannedInfo", "UnmannedInfo")
                    if not isinstance(info, dict):
                        continue
                    current_wp = _get_ci(info, "currentWaypointID", "CurrentWaypointID")
                    if isinstance(current_wp, dict):
                        current_wp = _get_ci(current_wp, "waypointID", "WaypointID")
                    wp_id = _coerce_int(current_wp, None)
                    if wp_id is not None and int(wp_id) > 0:
                        latest_wp_by_aircraft[int(aid)] = int(wp_id)
        except Exception:
            latest_wp_by_aircraft = {}

        formation_wp_ids_by_path: dict[int, set[int]] = {}
        for pid in formation_by_path_id.keys():
            data = flight_by_path.get(int(pid))
            ids: set[int] = set()
            if isinstance(data, dict):
                for wp in _extract_waypoints(data):
                    if not isinstance(wp, dict):
                        continue
                    wp_id = _coerce_int(_get_ci(wp, "waypointID", "WaypointID"), None)
                    if wp_id is not None and int(wp_id) > 0:
                        ids.add(int(wp_id))
            formation_wp_ids_by_path[int(pid)] = ids

        seq_by_aircraft: dict[int, list[dict]] = {}
        for entry in individual_mission_plans:
            if not isinstance(entry, dict):
                continue
            try:
                aircraft_id = int(entry.get("aircraftID", 0))
            except Exception:
                aircraft_id = 0
            if not aircraft_id:
                continue
            seq = seq_by_aircraft.setdefault(aircraft_id, [])
            for im in entry.get("individualMissionList") or []:
                if not isinstance(im, dict):
                    continue
                rel = im.get("relatedMission") or im.get("RelatedMission") or {}
                input_id = _coerce_int(
                    _get_ci(rel, "inputMissionID", "InputMissionID")
                    or _get_ci(im, "inputMissionID", "InputMissionID"),
                    None,
                )
                try:
                    path_id = int(im.get("pathID"))
                except Exception:
                    path_id = None
                mission_done = _coerce_bool(im.get("isDone"), False)
                formation_not_reached = (
                    mission_done
                    and path_id is not None
                    and int(path_id) in formation_by_path_id
                    and (
                        (
                            preserve_state
                            and aircraft_id in prev_target_path_by_aircraft
                            and prev_target_path_by_aircraft.get(aircraft_id) != int(path_id)
                        )
                        or (
                            latest_wp_by_aircraft.get(int(aircraft_id)) is not None
                            and latest_wp_by_aircraft.get(int(aircraft_id))
                            not in formation_wp_ids_by_path.get(int(path_id), set())
                        )
                    )
                )
                # Keep done missions in payload for visualization/history, but
                # exclude them from execution. Formation is the exception: a
                # replan can mark it done before the simulator has ever entered
                # the formation path, so keep it executable until reached.
                if mission_done and not formation_not_reached:
                    continue
                try:
                    individual_id = int(im.get("individualMissionID"))
                except Exception:
                    individual_id = None
                if path_id is None:
                    continue
                seq.append(
                    {
                        "input_mission_id": input_id,
                        "path_id": path_id,
                        "individual_mission_id": individual_id,
                    }
                )

        input_order: list[int] = []
        if isinstance(input_mission_plans, list) and input_mission_plans:
            latest = input_mission_plans[0]
            latest_ts = float(latest.get("timestamp") or 0)
            for plan in input_mission_plans:
                try:
                    ts = float(plan.get("timestamp") or 0)
                except Exception:
                    ts = 0
                if ts >= latest_ts:
                    latest = plan
                    latest_ts = ts
            for item in latest.get("inputMissionList") or []:
                try:
                    input_order.append(int(item.get("inputMissionID")))
                except Exception:
                    continue

        order_per_aircraft: dict[int, list[int]] = {}
        if not input_order and seq_by_aircraft:
            seen: set[int] = set()
            for seq in seq_by_aircraft.values():
                for entry in seq:
                    mid = entry.get("input_mission_id")
                    if mid is None or mid in seen:
                        continue
                    seen.add(mid)
                    input_order.append(mid)
        if seq_by_aircraft:
            for aircraft_id, seq in seq_by_aircraft.items():
                seen: set[int] = set()
                order: list[int] = []
                for entry in seq:
                    mid = entry.get("input_mission_id")
                    if mid is None or mid in seen:
                        continue
                    seen.add(mid)
                    order.append(mid)
                order_per_aircraft[aircraft_id] = order
        if input_order:
            for aid in list(order_per_aircraft.keys()):
                if not order_per_aircraft.get(aid):
                    order_per_aircraft[aid] = list(input_order)
            if not order_per_aircraft and input_order:
                for aid in flight_by_aircraft.keys():
                    order_per_aircraft[aid] = list(input_order)

        block_indices: dict[int, dict[int, int]] = {}

        if seq_by_aircraft:
            ordered_paths: list[PathDefinition] = []
            ordered_latlons: list[tuple[float, float]] = []
            for aircraft_id, seq in seq_by_aircraft.items():
                combined: list[dict] = []
                block_indices.setdefault(aircraft_id, {})
                last_input = None
                order_list = order_per_aircraft.get(aircraft_id) or []
                if order_list:
                    last_input = order_list[-1]
                last_path_for_input: dict[int, int] = {}
                for entry in seq:
                    mid = entry.get("input_mission_id")
                    pid = entry.get("path_id")
                    if mid is None or pid is None:
                        continue
                    last_path_for_input[mid] = pid

                for entry in seq:
                    pid = entry.get("path_id")
                    if pid is None:
                        continue
                    data = flight_by_path.get(pid)
                    if not isinstance(data, dict):
                        continue
                    path_sep_m = path_sep_by_id.get(int(pid))
                    wps = _extract_waypoints(data)
                    if not wps:
                        continue
                    wps = _order_waypoints(wps)
                    start_idx = len(combined)
                    for wp in wps:
                        if not isinstance(wp, dict):
                            continue
                        coord = _extract_coord(wp)
                        if coord is None:
                            continue
                        lat, lon, alt = _override_coord_for_loiter(wp, coord)
                        ordered_latlons.append((lon, lat))
                        speed = wp.get("speed")
                        try:
                            speed = float(speed) if speed is not None else None
                        except Exception:
                            speed = None
                        wp_id = wp.get("waypointID") or wp.get("WaypointID")
                        try:
                            wp_id = int(wp_id) if wp_id is not None else None
                        except Exception:
                            wp_id = None
                        pass_type = wp.get("waypointPassType") or wp.get("WaypointPassType")
                        try:
                            pass_type = int(pass_type) if pass_type is not None else None
                        except Exception:
                            pass_type = None
                        hover_time = _extract_hover_time(wp)
                        loiter = _normalize_loiter(wp)
                        wp_is_done = _coerce_bool(wp.get("isDone"), False)
                        filming = wp.get("filmingProperty")
                        if aircraft_id >= 4 and not wp_is_done:
                            override = self._build_tracking_override(filming, loiter)
                            if override:
                                tracking_overrides_by_aircraft[aircraft_id] = override
                        attack = (
                            wp.get("attack")
                            or wp.get("Attack")
                            or wp.get("attackProperty")
                            or wp.get("attack_prop")
                        )
                        alt = self._adjust_lah_altitude(
                            aircraft_id,
                            lat,
                            lon,
                            alt,
                            hover_time=hover_time,
                            loiter=loiter,
                            attack=attack if isinstance(attack, dict) else None,
                        )
                        combined.append(
                            {
                                "lat": lat,
                                "lon": lon,
                                "alt": alt,
                                "speed": speed,
                                "wp_id": wp_id,
                                "is_done": wp_is_done,
                                "hover_time": hover_time,
                                "loiter": loiter,
                                "pass_type": pass_type,
                                "filming": filming,
                                "attack": attack if isinstance(attack, dict) else None,
                                "path_id": pid,
                                "input_mission_id": entry.get("input_mission_id"),
                                "individual_mission_id": entry.get("individual_mission_id"),
                                "sep_m": path_sep_m,
                            }
                        )
                    end_idx = len(combined) - 1
                    mid = entry.get("input_mission_id")
                    if (
                        mid is not None
                        and end_idx >= start_idx
                        and last_path_for_input.get(mid) == pid
                        and (last_input is None or mid != last_input)
                    ):
                        block_indices[aircraft_id][end_idx] = mid

                if len(combined) < 2:
                    continue
                label = _agent_label(aircraft_id)
                airframe = _airframe_type(aircraft_id)
                ordered_paths.append(
                    PathDefinition(
                        label=label,
                        aircraft_id=aircraft_id,
                        airframe=airframe,
                        path_id=None,
                        waypoints=combined,
                    )
                )

            if ordered_paths:
                paths = ordered_paths
                all_latlons = ordered_latlons
                self.input_mission_order_by_aircraft = order_per_aircraft
                self.current_input_mission_idx_by_aircraft = {
                    aid: 0 for aid in order_per_aircraft.keys()
                }
                self._block_indices = block_indices
            elif formation_by_aircraft or formation_by_path_id:
                # For formation payloads, IndividualMissionPlan ordering is
                # authoritative. If every mission is already marked done, do
                # not fall back to raw FlightPath rows and publish completed
                # non-formation paths as active UAV missions.
                paths = []
                all_latlons = []
                self.input_mission_order_by_aircraft = order_per_aircraft
                self.current_input_mission_idx_by_aircraft = {
                    aid: max(0, len(order) - 1)
                    for aid, order in order_per_aircraft.items()
                }
                self._block_indices = {}
        elif mission_order:
            ordered_paths: list[PathDefinition] = []
            ordered_latlons: list[tuple[float, float]] = []
            for key, path_ids in mission_order.items():
                if not isinstance(path_ids, list):
                    continue
                aircraft_id = None
                if isinstance(key, (int, float)):
                    aircraft_id = int(key)
                elif isinstance(key, str):
                    aircraft_id = _label_to_aircraft_id(key)
                    if aircraft_id is None:
                        try:
                            aircraft_id = int(key)
                        except Exception:
                            aircraft_id = None
                if aircraft_id is None:
                    continue

                combined: list[dict] = []
                for pid in path_ids:
                    try:
                        pid_int = int(pid)
                    except Exception:
                        continue
                    data = flight_by_path.get(pid_int)
                    if not isinstance(data, dict):
                        continue
                    path_sep_m = path_sep_by_id.get(int(pid_int))
                    wps = _extract_waypoints(data)
                    if not wps:
                        continue
                    wps = _order_waypoints(wps)
                    for wp in wps:
                        if not isinstance(wp, dict):
                            continue
                        coord = _extract_coord(wp)
                        if coord is None:
                            continue
                        lat, lon, alt = _override_coord_for_loiter(wp, coord)
                        ordered_latlons.append((lon, lat))
                        speed = wp.get("speed")
                        try:
                            speed = float(speed) if speed is not None else None
                        except Exception:
                            speed = None
                        wp_id = wp.get("waypointID") or wp.get("WaypointID")
                        try:
                            wp_id = int(wp_id) if wp_id is not None else None
                        except Exception:
                            wp_id = None
                        pass_type = wp.get("waypointPassType") or wp.get("WaypointPassType")
                        try:
                            pass_type = int(pass_type) if pass_type is not None else None
                        except Exception:
                            pass_type = None
                        hover_time = _extract_hover_time(wp)
                        loiter = _normalize_loiter(wp)
                        wp_is_done = _coerce_bool(wp.get("isDone"), False)
                        filming = wp.get("filmingProperty")
                        if aircraft_id >= 4 and not wp_is_done:
                            override = self._build_tracking_override(filming, loiter)
                            if override:
                                tracking_overrides_by_aircraft[aircraft_id] = override
                        attack = (
                            wp.get("attack")
                            or wp.get("Attack")
                            or wp.get("attackProperty")
                            or wp.get("attack_prop")
                        )
                        alt = self._adjust_lah_altitude(
                            aircraft_id,
                            lat,
                            lon,
                            alt,
                            hover_time=hover_time,
                            loiter=loiter,
                            attack=attack if isinstance(attack, dict) else None,
                        )
                        combined.append(
                            {
                                "lat": lat,
                                "lon": lon,
                                "alt": alt,
                                "speed": speed,
                                "wp_id": wp_id,
                                "is_done": wp_is_done,
                                "hover_time": hover_time,
                                "loiter": loiter,
                                "pass_type": pass_type,
                                "filming": filming,
                                "attack": attack if isinstance(attack, dict) else None,
                                "path_id": pid_int,
                                "sep_m": path_sep_m,
                            }
                        )
                if len(combined) < 2:
                    continue
                label = _agent_label(aircraft_id)
                airframe = _airframe_type(aircraft_id)
                ordered_paths.append(
                    PathDefinition(
                        label=label,
                        aircraft_id=aircraft_id,
                        airframe=airframe,
                        path_id=None,
                        waypoints=combined,
                    )
                )

            if ordered_paths:
                paths = ordered_paths
                all_latlons = ordered_latlons
        else:
            # Fallback: map FlightPath by pathID prefix (1..6)
            combined_by_aircraft: dict[int, list[dict]] = {}
            for pid, data in flight_by_path.items():
                try:
                    prefix = int(str(pid)[0])
                except Exception:
                    continue
                if prefix < 1 or prefix > 6:
                    continue
                aircraft_id = prefix
                path_sep_m = path_sep_by_id.get(int(pid))
                wps = _extract_waypoints(data)
                if not wps:
                    continue
                wps = _order_waypoints(wps)
                combined = combined_by_aircraft.setdefault(aircraft_id, [])
                for wp in wps:
                    if not isinstance(wp, dict):
                        continue
                    coord = _extract_coord(wp)
                    if coord is None:
                        continue
                    lat, lon, alt = _override_coord_for_loiter(wp, coord)
                    all_latlons.append((lon, lat))
                    speed = wp.get("speed")
                    try:
                        speed = float(speed) if speed is not None else None
                    except Exception:
                        speed = None
                    wp_id = wp.get("waypointID") or wp.get("WaypointID")
                    try:
                        wp_id = int(wp_id) if wp_id is not None else None
                    except Exception:
                        wp_id = None
                    pass_type = wp.get("waypointPassType") or wp.get("WaypointPassType")
                    try:
                        pass_type = int(pass_type) if pass_type is not None else None
                    except Exception:
                        pass_type = None
                    hover_time = _extract_hover_time(wp)
                    loiter = _normalize_loiter(wp)
                    wp_is_done = _coerce_bool(wp.get("isDone"), False)
                    filming = wp.get("filmingProperty")
                    if aircraft_id >= 4 and not wp_is_done:
                        override = self._build_tracking_override(filming, loiter)
                        if override:
                            tracking_overrides_by_aircraft[aircraft_id] = override
                    attack = (
                        wp.get("attack")
                        or wp.get("Attack")
                        or wp.get("attackProperty")
                        or wp.get("attack_prop")
                    )
                    alt = self._adjust_lah_altitude(
                        aircraft_id,
                        lat,
                        lon,
                        alt,
                        hover_time=hover_time,
                        loiter=loiter,
                        attack=attack if isinstance(attack, dict) else None,
                    )
                    combined.append(
                        {
                            "lat": lat,
                            "lon": lon,
                            "alt": alt,
                            "speed": speed,
                            "wp_id": wp_id,
                            "is_done": wp_is_done,
                            "hover_time": hover_time,
                            "loiter": loiter,
                            "pass_type": pass_type,
                            "filming": filming,
                            "attack": attack if isinstance(attack, dict) else None,
                            "path_id": pid,
                            "sep_m": path_sep_m,
                        }
                    )
            if combined_by_aircraft:
                for aircraft_id, combined in combined_by_aircraft.items():
                    if len(combined) < 2:
                        continue
                    label = _agent_label(aircraft_id)
                    airframe = _airframe_type(aircraft_id)
                    paths.append(
                        PathDefinition(
                            label=label,
                            aircraft_id=aircraft_id,
                            airframe=airframe,
                            path_id=None,
                            waypoints=combined,
                        )
                    )

        initial_ids: set[int] = set()
        forced_labels: set[str] = set(prev_forced or {})
        if preserve_state:
            if prev_initial_ids:
                initial_ids = set(prev_initial_ids)
            elif prev_vehicles:
                initial_ids = {int(v.aircraft_id) for v in prev_vehicles.values()}
            if initial_ids:
                paths = [
                    p
                    for p in paths
                    if int(p.aircraft_id) in initial_ids and p.label not in forced_labels
                ]

        if not paths:
            if tracking_overrides_by_aircraft and (preserve_state or prev_vehicles or self.vehicles):
                with self._lock:
                    self._tracking_override_seq += 1
                    overrides: dict[str, dict[str, Any]] = {}
                    for aid, override in tracking_overrides_by_aircraft.items():
                        label = _agent_label(int(aid))
                        if not label:
                            continue
                        entry = dict(override)
                        entry["seq"] = self._tracking_override_seq
                        overrides[label] = entry
                    self._tracking_overrides = overrides
                return {"ok": True, "count": 0, "preserved": True}
            if seq_by_aircraft and (formation_by_aircraft or formation_by_path_id):
                if preserve_state and prev_vehicles:
                    return {
                        "ok": True,
                        "count": len(prev_vehicles),
                        "preserved": True,
                        "noActiveMissions": True,
                    }
                return {"ok": True, "count": 0, "noActiveMissions": True}
            return {"ok": False, "error": "no valid flight paths"}

        if not all_latlons:
            return {"ok": False, "error": "waypoints missing coordinates"}

        lon_avg = sum(lon for lon, _ in all_latlons) / len(all_latlons)
        lat_avg = sum(lat for _, lat in all_latlons) / len(all_latlons)

        spawn_latlon: dict[int, tuple[float, float, float]] = {}
        for path in paths:
            if not isinstance(path, PathDefinition):
                continue
            first_wp = (path.waypoints or [None])[0]
            if not isinstance(first_wp, dict):
                continue
            try:
                aircraft_id = int(path.aircraft_id)
            except Exception:
                continue
            if aircraft_id <= 0:
                continue
            lat = _coerce_float(first_wp.get("lat"), float("nan"))
            lon = _coerce_float(first_wp.get("lon"), float("nan"))
            if not math.isfinite(lat) or not math.isfinite(lon):
                continue
            alt = _coerce_float(first_wp.get("alt"), 0.0)
            spawn_latlon[aircraft_id] = (
                lat,
                lon,
                self._resolve_spawn_altitude(aircraft_id, lat, lon, alt),
            )
        for item in take_over_list:
            if not isinstance(item, dict):
                continue
            try:
                aircraft_id = int(item.get("aircraftID") or item.get("AircraftID") or 0)
            except Exception:
                aircraft_id = 0
            if aircraft_id <= 0:
                continue
            if aircraft_id in spawn_latlon:
                continue
            coord = _extract_coord(item)
            if coord is None:
                continue
            lat, lon, alt = coord
            spawn_latlon[aircraft_id] = (
                lat,
                lon,
                self._resolve_spawn_altitude(aircraft_id, lat, lon, alt),
            )

        with self._lock:
            if not preserve_state:
                self._reset_persistent_target_detection_state()
                self._reset_target_state_for_fresh_load(clear_pending=fresh_db_root)
            pending = [] if fresh_db_root else list(self._pending_targets)
            pending_roi = [] if fresh_db_root else list(self._pending_roi_mocks)
            self._pending_targets = []
            self._pending_roi_mocks = []
            if preserve_state and prev_geo is not None:
                self.geo = prev_geo
            else:
                self.geo = GeoConverter(lon_avg, lat_avg)
            geo = self.geo
            spawn_by_aircraft: dict[int, tuple[float, float, float]] = {}
            if geo and spawn_latlon:
                for aid, (lat, lon, alt) in spawn_latlon.items():
                    x, y = geo.lonlat_to_xy(lon, lat)
                    spawn_by_aircraft[aid] = (x, y, alt)
            if geo and preserve_state and prev_positions:
                for aid, (lat, lon, alt) in prev_positions.items():
                    try:
                        x, y = geo.lonlat_to_xy(lon, lat)
                        spawn_by_aircraft[aid] = (x, y, alt)
                    except Exception:
                        continue
            # LAH spawns: 300m south of matching UAV if missing
            for lah_id in (1, 2, 3):
                if lah_id in spawn_by_aircraft:
                    continue
                uav_spawn = spawn_by_aircraft.get(lah_id + 3)
                if uav_spawn is None:
                    continue
                ux, uy, uz = uav_spawn
                lah_x = float(ux)
                lah_y = float(uy - 300.0)
                lah_alt = float(uz)
                if geo is not None:
                    try:
                        lah_lon, lah_lat = geo.xy_to_lonlat(lah_x, lah_y)
                        lah_alt = self._resolve_spawn_altitude(lah_id, lah_lat, lah_lon, 0.0)
                    except Exception:
                        lah_alt = float(uz)
                spawn_by_aircraft[lah_id] = (lah_x, lah_y, lah_alt)
            self._spawn_by_aircraft = spawn_by_aircraft
            self._paths = paths
            # Replan loads preserve targetInfo; keep the 0402 target ID map in
            # lockstep so destroyed IDs are not reused for later detections.
            self._build_vehicles(paths, reset_detection_state=not preserve_state)
            if seq_by_aircraft:
                for simv in self.vehicles.values():
                    self._sync_current_input_idx_to_controller_target(simv)
            retained_labels: set[str] = set()
            retained_aircraft_ids: set[int] = set()
            compatible_updated_labels: set[str] = set()
            compatible_updated_aircraft_ids: set[int] = set()
            updated_aircraft_ids = {int(p.aircraft_id) for p in paths}
            if preserve_state and prev_states:
                for simv in self.vehicles.values():
                    state = prev_states.get(int(simv.aircraft_id))
                    if not state:
                        continue
                    s = simv.vehicle.s
                    s.yaw = float(state.get("yaw", getattr(s, "yaw", 0.0)))
                    s.u = float(state.get("speed", getattr(s, "u", 0.0)))
                    s.z = float(state.get("alt", getattr(s, "z", 0.0)))
                    # health/alive/crashed 상태는 새 미션 로드 시 항상 초기화
                    # (이전 고장 상태가 다음 시나리오로 이어지는 것을 방지)
            if preserve_state and prev_controller_progress:
                for simv in self.vehicles.values():
                    aircraft_id = int(simv.aircraft_id)
                    snapshot = prev_controller_progress.get(aircraft_id)
                    if snapshot is None:
                        continue
                    if self._restore_controller_progress(simv.controller, snapshot):
                        self._sync_current_input_idx_to_controller_target(simv)
                        compatible_updated_labels.add(simv.label)
                        compatible_updated_aircraft_ids.add(aircraft_id)
            if preserve_state and prev_vehicles:
                for label, prev_simv in prev_vehicles.items():
                    aid = int(prev_simv.aircraft_id)
                    if initial_ids and aid not in initial_ids:
                        continue
                    if aid in updated_aircraft_ids and label not in forced_labels:
                        continue
                    if geo and prev_positions and aid in prev_positions:
                        lat, lon, alt = prev_positions[aid]
                        try:
                            x, y = geo.lonlat_to_xy(lon, lat)
                            prev_simv.vehicle.s.x = float(x)
                            prev_simv.vehicle.s.y = float(y)
                            prev_simv.vehicle.s.z = float(alt)
                        except Exception:
                            pass
                    self.vehicles[label] = prev_simv
                    retained_labels.add(label)
                    retained_aircraft_ids.add(aid)
            if preserve_state and prev_forced:
                self._forced_commands = {}
            if preserve_state and prev_forced:
                for label, info in prev_forced.items():
                    simv = self.vehicles.get(label)
                    if simv is None or not simv.alive:
                        continue
                    ftype = info.get("type")
                    if ftype == "rtb":
                        self._force_rtb(simv)
                    elif ftype == "hold":
                        center = info.get("center")
                        if not center or not isinstance(center, (list, tuple)) or len(center) != 3:
                            s = simv.vehicle.s
                            center = (float(s.x), float(s.y), float(s.z))
                        radius = float(info.get("radius") or 300.0)
                        speed = float(info.get("speed") or getattr(simv.controller, "speed_target", self.speed_uav))
                        end_time = info.get("end_time")
                        if end_time is not None and prev_sim_time is not None:
                            remaining = max(0.0, float(end_time) - float(prev_sim_time))
                        else:
                            remaining = 50.0
                        if remaining > 0.0:
                            self._force_hold_custom(
                                simv,
                                center=center,
                                duration=remaining,
                                radius=radius,
                                speed=speed,
                                angle=info.get("angle"),
                            )
            tracking_override_labels = {
                _agent_label(int(aid))
                for aid in tracking_overrides_by_aircraft.keys()
                if _agent_label(int(aid))
            }
            restored_labels = set(retained_labels) | set(compatible_updated_labels)
            if preserve_state and restored_labels:
                if prev_tracking is not None:
                    tracking_restore_labels = set(restored_labels)
                    if updated_aircraft_ids:
                        tracking_restore_labels &= set(tracking_override_labels)
                    self._tracking_state = {
                        label: prev_tracking[label]
                        for label in tracking_restore_labels
                        if label in prev_tracking
                    }
                if prev_tracking_owner is not None:
                    tracking_owner_labels = (
                        set(self._tracking_state.keys())
                        if updated_aircraft_ids
                        else set(restored_labels)
                    )
                    self._tracking_target_owner = {
                        int(tid): lbl
                        for tid, lbl in prev_tracking_owner.items()
                        if lbl in tracking_owner_labels
                    }
                if prev_line_search_state is not None:
                    for label in restored_labels:
                        if label in prev_line_search_state:
                            self._line_search_state[label] = prev_line_search_state[label]
                if prev_line_search_debug is not None:
                    for label in restored_labels:
                        if label in prev_line_search_debug:
                            self._line_search_debug[label] = prev_line_search_debug[label]
                for label in restored_labels:
                    simv = self.vehicles.get(label)
                    if simv is not None:
                        self._update_filming_target(simv, 0.0)
            if pending and geo:
                for item in pending:
                    try:
                        type_id = int(item.get("type", 0))
                        lat = float(item.get("lat"))
                        lon = float(item.get("lon"))
                        alt = float(item.get("alt") or 0.0)
                    except Exception:
                        continue
                    try:
                        x, y = geo.lonlat_to_xy(lon, lat)
                        tgt = self._build_target(
                            type_id=type_id,
                            x=x,
                            y=y,
                            z=alt,
                            id_override=item.get("id"),
                            name_override=item.get("name"),
                        )
                        self.targets.append(tgt)
                    except Exception:
                        continue
            if preserve_state and prev_running is not None and prev_paused is not None:
                self.running = bool(prev_running)
                self.paused = bool(prev_paused)
            else:
                self.running = False
                self.paused = True
            if preserve_state and prev_sim_time is not None and prev_step_count is not None:
                self.sim_time = float(prev_sim_time)
                self.step_count = int(prev_step_count)
                if prev_ts_anchor is not None and int(prev_ts_anchor) > 0:
                    self._sim_timestamp_anchor_ms_2000 = int(prev_ts_anchor)
                else:
                    self._sync_sim_timestamp_anchor()
            else:
                self.sim_time = 0.0
                self.step_count = 0
                self._sync_sim_timestamp_anchor()
            self.last_error = None
            self._projectiles = []
            self._projectile_id_seq = 1
            self._last_enemy_fire = {}
            self._last_vehicle_fire = {}
            self._friendly_attack_attempts = {}
            self._effects = []
            self._effect_id_seq = 1
            self._last_0401_emit_wall_time = prev_last_0401_emit_wall_time if preserve_state else None
            self._last_0402_sim_time = prev_last_0402 if preserve_state else None
            self._pending_input_advances = {}
            self._input_advance_guard_until = {}
            if preserve_state and prev_overrides is not None:
                # 이전 override 복원하되, health=2 (고장) 항목은 제거
                self._agent_overrides = {}
                for label, ovr in prev_overrides.items():
                    cleaned = {k: v for k, v in ovr.items() if not (k == "health" and v == 2)}
                    if cleaned:
                        self._agent_overrides[label] = cleaned
            else:
                self._agent_overrides = {}
            if not (preserve_state and prev_forced):
                self._forced_commands = {}
            self._rtb_coord_cache = None
            if not seq_by_aircraft:
                self.input_mission_order_by_aircraft = {}
                self.current_input_mission_idx_by_aircraft = {}
                self._block_indices = {}
                self._spawn_by_aircraft = spawn_by_aircraft
            preserved_input_aircraft_ids = set(retained_aircraft_ids) | set(compatible_updated_aircraft_ids)
            if preserve_state and prev_input_order and preserved_input_aircraft_ids:
                for aid in preserved_input_aircraft_ids:
                    if aid in updated_aircraft_ids and aid not in compatible_updated_aircraft_ids:
                        continue
                    if aid in prev_input_order:
                        self.input_mission_order_by_aircraft[aid] = list(prev_input_order[aid])
                    if prev_input_idx and aid in prev_input_idx:
                        self.current_input_mission_idx_by_aircraft[aid] = int(prev_input_idx[aid])
            if formation_by_aircraft:
                self._formation_by_aircraft = formation_by_aircraft
            else:
                self._formation_by_aircraft = {}
            if formation_by_path_id:
                self._formation_by_path_id = formation_by_path_id
            else:
                self._formation_by_path_id = {}
            if not self._initial_aircraft_ids and self.vehicles:
                self._initial_aircraft_ids = {int(v.aircraft_id) for v in self.vehicles.values()}
            if tracking_overrides_by_aircraft:
                self._tracking_override_seq += 1
                overrides: dict[str, dict[str, Any]] = {}
                for aid, override in tracking_overrides_by_aircraft.items():
                    label = _agent_label(int(aid))
                    if not label:
                        continue
                    entry = dict(override)
                    entry["seq"] = self._tracking_override_seq
                    overrides[label] = entry
                self._tracking_overrides = overrides
            else:
                self._tracking_overrides = {}
            if active_db_root:
                self._loaded_db_root = active_db_root

        return {"ok": True, "count": len(paths)}

    def _ground_height(self, x: float, y: float) -> float:
        geo = self.geo
        if geo is None:
            return 0.0
        try:
            lon, lat = geo.xy_to_lonlat(float(x), float(y))
        except Exception:
            return 0.0
        return self._terrain_elev(float(lat), float(lon))

    def _terrain_elev(self, lat: float, lon: float) -> float:
        if self._terrain_elev_fn is False:
            return 0.0
        if self._terrain_elev_fn is None:
            try:
                from modules.mission_planning.MissionPlanner.data_def.mission_helpers import (
                    terrain_elev as _terrain_elev,
                )
                self._terrain_elev_fn = _terrain_elev
            except Exception:
                self._terrain_elev_fn = False
                return 0.0
        try:
            return float(self._terrain_elev_fn(float(lat), float(lon)))
        except Exception:
            return 0.0

    def _spawn_altitude_offset_m(self, aircraft_id: int) -> float:
        aid = int(aircraft_id)
        if aid == 1:
            return 310.0
        if aid == 2:
            return 320.0
        if aid == 3:
            return 330.0
        if aid == 4:
            return 1000.0
        if aid == 5:
            return 1010.0
        if aid == 6:
            return 1020.0
        return 0.0

    def _resolve_spawn_altitude(self, aircraft_id: int, lat: float, lon: float, alt: float | None) -> float:
        try:
            alt_f = float(alt) if alt is not None else 0.0
        except Exception:
            alt_f = 0.0
        if alt_f > 0.0:
            return alt_f
        ground = self._terrain_elev(float(lat), float(lon))
        return float(ground) + float(self._spawn_altitude_offset_m(int(aircraft_id)))

    def _adjust_lah_altitude(
        self,
        aircraft_id: int,
        lat: float,
        lon: float,
        alt: float | None,
        *,
        hover_time: float | None,
        loiter: dict | None,
        attack: dict | None,
    ) -> float | None:
        if aircraft_id not in (1, 2, 3):
            return alt
        if hover_time and hover_time > 0:
            return alt
        if isinstance(loiter, dict):
            if (loiter.get("time") or 0) or (loiter.get("radius") or 0) or (loiter.get("speed") or 0):
                return alt
        if isinstance(attack, dict):
            try:
                if int(attack.get("targetID") or 0) > 0:
                    return alt
            except Exception:
                return alt
        ground = self._terrain_elev(lat, lon)
        min_alt = float(ground) + 200.0
        if alt is None or not math.isfinite(float(alt)) or float(alt) < min_alt:
            return float(min_alt)
        return alt

    def _default_downward_target(self, vehicle) -> tuple[float, float, float]:
        return (float(vehicle.s.x), float(vehicle.s.y), float(self._ground_height(vehicle.s.x, vehicle.s.y)))

    def _intersect_view_ray_with_terrain(
        self,
        *,
        origin_x: float,
        origin_y: float,
        origin_z: float,
        dir_x: float,
        dir_y: float,
        dir_z: float,
        initial_t: float,
        ground_height_fn: Callable[[float, float], float] | None = None,
        step_m: float | None = None,
        max_distance_m: float | None = None,
        binary_steps: int | None = None,
    ) -> tuple[float, float, float] | None:
        if abs(dir_z) < 1e-9:
            return None

        height_at = ground_height_fn or self._ground_height
        step_distance = max(
            float(_FOOTPRINT_MIN_RAY_STEP_M),
            float(step_m if step_m is not None else _FOOTPRINT_RAY_STEP_M),
        )
        max_range = float(max_distance_m) if max_distance_m is not None else max(
            float(_FOOTPRINT_RAY_MIN_RANGE_M),
            float(initial_t) * float(_FOOTPRINT_RAY_MAX_RANGE_FACTOR),
            float(initial_t) + float(_FOOTPRINT_RAY_EXTRA_RANGE_M),
        )
        binary_search_steps = int(binary_steps if binary_steps is not None else _FOOTPRINT_RAY_BINARY_STEPS)

        origin_ground = height_at(origin_x, origin_y)
        if origin_z <= origin_ground:
            return None

        def _diff_at(distance_m: float) -> float:
            px = origin_x + dir_x * distance_m
            py = origin_y + dir_y * distance_m
            pz = origin_z + dir_z * distance_m
            return pz - height_at(px, py)

        lower = 0.0
        lower_diff = origin_z - origin_ground
        guess = max(float(initial_t), step_distance)
        upper = guess
        upper_diff = _diff_at(upper)

        if upper_diff > 0.0:
            step = max(step_distance, guess * 0.25)
            lower = upper
            lower_diff = upper_diff
            while upper < max_range:
                upper = min(max_range, upper + step)
                upper_diff = _diff_at(upper)
                if upper_diff <= 0.0:
                    break
                lower = upper
                lower_diff = upper_diff
            if upper_diff > 0.0:
                return None

        if lower_diff <= 0.0:
            lower = 0.0
            lower_diff = origin_z - origin_ground
            if lower_diff <= 0.0:
                return None

        for _ in range(binary_search_steps):
            mid = 0.5 * (lower + upper)
            mid_diff = _diff_at(mid)
            if mid_diff > 0.0:
                lower = mid
            else:
                upper = mid

        hit_t = float(upper)
        hit_x = origin_x + dir_x * hit_t
        hit_y = origin_y + dir_y * hit_t
        hit_z = height_at(hit_x, hit_y)
        return float(hit_x), float(hit_y), float(hit_z)

    def _build_footprint_terrain_context(
        self,
        focus_x: float,
        focus_y: float,
    ) -> tuple[Callable[[float, float], float], float]:
        geo = self.geo
        default_step = float(_FOOTPRINT_RAY_STEP_M)
        if geo is None:
            return self._ground_height, default_step

        try:
            focus_lon, focus_lat = geo.xy_to_lonlat(float(focus_x), float(focus_y))
        except Exception:
            return self._ground_height, default_step

        primary_tile = _resolve_footprint_dem_tile(float(focus_lat), float(focus_lon))
        if primary_tile is None:
            return self._ground_height, default_step

        dx, dy = primary_tile.ground_resolution_m(float(focus_lat))
        recommended_step = max(float(_FOOTPRINT_MIN_RAY_STEP_M), 0.5 * min(dx, dy))
        ray_step_m = max(float(_FOOTPRINT_MIN_RAY_STEP_M), min(default_step, recommended_step))
        last_tile: _FootprintDemTile | None = primary_tile

        def _ground_height_xy(x: float, y: float) -> float:
            nonlocal last_tile
            try:
                lon, lat = geo.xy_to_lonlat(float(x), float(y))
            except Exception:
                return self._ground_height(x, y)

            tile = last_tile
            if tile is None or not tile.contains(float(lat), float(lon)):
                tile = _resolve_footprint_dem_tile(float(lat), float(lon))
                if tile is not None:
                    last_tile = tile
            if tile is not None:
                try:
                    return float(tile.sample(float(lat), float(lon)))
                except Exception:
                    pass
            return self._terrain_elev(float(lat), float(lon))

        return _ground_height_xy, float(ray_step_m)

    def _resolve_view_center_hit(
        self,
        simv: SimVehicle,
        focus: tuple[float, float, float],
        *,
        ground_height_fn: Callable[[float, float], float],
        step_m: float,
    ) -> tuple[float, float, float] | None:
        s = simv.vehicle.s
        origin = (float(s.x), float(s.y), float(s.z))
        try:
            forward, _right, _up, fwd_len = _build_footprint_camera_axes(origin, focus)
        except ValueError:
            return None

        hit = self._intersect_view_ray_with_terrain(
            origin_x=origin[0],
            origin_y=origin[1],
            origin_z=origin[2],
            dir_x=forward[0],
            dir_y=forward[1],
            dir_z=forward[2],
            initial_t=float(fwd_len),
            ground_height_fn=ground_height_fn,
            step_m=step_m,
            max_distance_m=float(_FOOTPRINT_MAX_DISTANCE_M),
            binary_steps=int(_FOOTPRINT_RAY_BINARY_STEPS),
        )
        if hit is not None:
            return hit
        try:
            focus_ground = float(ground_height_fn(float(focus[0]), float(focus[1])))
            return (float(focus[0]), float(focus[1]), float(focus_ground))
        except Exception:
            return (float(focus[0]), float(focus[1]), float(focus[2]))

    def _footprint_body_right_axis(self, simv: SimVehicle) -> tuple[float, float, float]:
        yaw_rad = math.radians(float(getattr(simv.vehicle.s, "yaw", 0.0)))
        return (-math.sin(yaw_rad), -math.cos(yaw_rad), 0.0)

    def _rotate_xy_axis(
        self,
        axis: tuple[float, float, float],
        angle_deg: float,
    ) -> tuple[float, float, float]:
        ang = math.radians(float(angle_deg))
        c = math.cos(ang)
        s = math.sin(ang)
        x = float(axis[0])
        y = float(axis[1])
        return ((x * c) - (y * s), (x * s) + (y * c), 0.0)

    def _line_search_debug_target_right_axis(
        self,
        simv: SimVehicle,
        debug: object,
    ) -> tuple[float, float, float] | None:
        if not isinstance(debug, dict):
            return None
        try:
            seg_t = max(0.0, min(1.0, float(debug.get("segmentT", 0.5))))
        except Exception:
            seg_t = 0.5
        base_axis = self._footprint_body_right_axis(simv)
        bias = (0.5 - seg_t) * 2.0 * float(_FOOTPRINT_LINE_AXIS_MAX_BIAS_DEG)
        return self._rotate_xy_axis(base_axis, bias)

    def _line_search_operation_active(self, label: str) -> bool:
        filming_prop = self._filming_props.get(str(label))
        if not isinstance(filming_prop, dict):
            return False
        try:
            op_mode = int(filming_prop.get("operationMode") or filming_prop.get("operationalMode") or 0)
        except Exception:
            op_mode = 0
        return op_mode == 2

    def _clear_footprint_line_right_axis(self, label: str) -> None:
        self._footprint_line_right_axis.pop(str(label), None)

    def _update_footprint_line_right_axis(
        self,
        label: str,
        target_axis: tuple[float, float, float] | None,
        dt: float,
    ) -> None:
        label = str(label)
        if target_axis is None:
            self._clear_footprint_line_right_axis(label)
            return
        prev_axis = self._footprint_line_right_axis.get(label)
        if prev_axis is None or float(dt or 0.0) <= 0.0:
            self._footprint_line_right_axis[label] = target_axis
            return

        prev_ang = math.atan2(float(prev_axis[1]), float(prev_axis[0]))
        target_ang = math.atan2(float(target_axis[1]), float(target_axis[0]))
        delta = ((target_ang - prev_ang + math.pi) % (math.pi * 2.0)) - math.pi
        max_delta = math.radians(float(_FOOTPRINT_LINE_AXIS_RATE_DPS)) * max(0.0, float(dt or 0.0))
        if max_delta <= 0.0 or abs(delta) <= max_delta:
            new_ang = target_ang
        else:
            new_ang = prev_ang + math.copysign(max_delta, delta)
        self._footprint_line_right_axis[label] = (math.cos(new_ang), math.sin(new_ang), 0.0)

    def _footprint_line_search_right_axis(self, simv: SimVehicle) -> tuple[float, float, float] | None:
        label = str(simv.label)
        if not self._line_search_operation_active(label):
            return None
        axis = self._footprint_line_right_axis.get(label)
        if axis is not None:
            return axis
        return self._line_search_debug_target_right_axis(simv, self._line_search_debug.get(label))

    def _resolve_footprint_projection(
        self,
        simv: SimVehicle,
        focus: tuple[float, float, float],
        fov_deg: float,
    ) -> _FootprintProjection | None:
        if fov_deg <= 0.0:
            return None

        s = simv.vehicle.s
        origin = (float(s.x), float(s.y), float(s.z))
        try:
            horizontal_fov_deg, vertical_fov_deg = _calculate_footprint_fov_components(
                fov_value_deg=float(fov_deg),
                aspect_ratio=float(_FOOTPRINT_ASPECT),
                interpretation=_FOOTPRINT_FOV_INTERPRETATION,
            )
            forward, right, up, fwd_len = _build_footprint_camera_axes(
                origin,
                focus,
                fallback_right=self._footprint_body_right_axis(simv),
                preferred_right=self._footprint_line_search_right_axis(simv),
            )
        except Exception:
            return None

        ground_height_fn, ray_step_m = self._build_footprint_terrain_context(float(focus[0]), float(focus[1]))
        center_hit = self._resolve_view_center_hit(
            simv,
            focus,
            ground_height_fn=ground_height_fn,
            step_m=ray_step_m,
        )
        if center_hit is None:
            return None

        tan_h = math.tan(math.radians(horizontal_fov_deg) / 2.0)
        tan_v = math.tan(math.radians(vertical_fov_deg) / 2.0)

        def _project_sensor_point(sensor_x: float, sensor_y: float) -> tuple[float, float, float] | None:
            dir_x = forward[0] + (sensor_x * tan_h * right[0]) + (sensor_y * tan_v * up[0])
            dir_y = forward[1] + (sensor_x * tan_h * right[1]) + (sensor_y * tan_v * up[1])
            dir_z = forward[2] + (sensor_x * tan_h * right[2]) + (sensor_y * tan_v * up[2])
            try:
                dir_x, dir_y, dir_z = _normalize3(dir_x, dir_y, dir_z)
            except ValueError:
                return None

            dir_dot_forward = max(1e-3, _dot3(dir_x, dir_y, dir_z, forward[0], forward[1], forward[2]))
            initial_t = float(fwd_len / dir_dot_forward)
            hit = self._intersect_view_ray_with_terrain(
                origin_x=origin[0],
                origin_y=origin[1],
                origin_z=origin[2],
                dir_x=dir_x,
                dir_y=dir_y,
                dir_z=dir_z,
                initial_t=initial_t,
                ground_height_fn=ground_height_fn,
                step_m=ray_step_m,
                max_distance_m=float(_FOOTPRINT_MAX_DISTANCE_M),
                binary_steps=int(_FOOTPRINT_RAY_BINARY_STEPS),
            )
            if hit is not None:
                return hit
            if abs(dir_z) < 1e-6:
                return None
            plane_z = float(center_hit[2])
            t = (plane_z - origin[2]) / dir_z
            if t <= 0.0:
                return None
            return (
                float(origin[0] + (dir_x * t)),
                float(origin[1] + (dir_y * t)),
                plane_z,
            )

        corners: list[tuple[float, float, float]] = []
        for sx, sy in _FOOTPRINT_CORNER_DEFINITIONS:
            hit = _project_sensor_point(float(sx), float(sy))
            if hit is None:
                return None
            corners.append(hit)

        return _FootprintProjection(
            center_hit=(float(center_hit[0]), float(center_hit[1]), float(center_hit[2])),
            corners=corners,
            ray_step_m=float(ray_step_m),
        )

    def _get_operation_handler(self, mode_id: int | None) -> OperationMode | None:
        if mode_id is None:
            return None
        handler = self._operation_handlers.get(mode_id)
        if handler is not None:
            return handler
        try:
            handler = build_operation_mode(mode_id)
        except Exception:
            return None
        self._operation_handlers[mode_id] = handler
        return handler

    def _resolve_tracking_fov(self, filming_prop: dict | None) -> float:
        if not isinstance(filming_prop, dict):
            return 60.0
        value = filming_prop.get("fieldOfView") or filming_prop.get("fov")
        try:
            fov = float(value)
        except Exception:
            fov = 60.0
        return max(5.0, min(160.0, fov))

    def _build_tracking_override(
        self, filming_prop: dict | None, loiter_prop: dict | None
    ) -> dict[str, Any] | None:
        if not isinstance(filming_prop, dict):
            return None
        op_mode = filming_prop.get("operationMode") or filming_prop.get("operationalMode")
        try:
            op_mode = int(op_mode or 0)
        except Exception:
            op_mode = 0
        if op_mode != 3:
            return None
        auto = filming_prop.get("autoTracking") or filming_prop.get("AutoTracking")
        if not isinstance(auto, dict):
            return None
        target_id = auto.get("targetID") or auto.get("TargetID") or auto.get("targetId")
        try:
            target_id = int(target_id)
        except Exception:
            target_id = 0
        if target_id <= 0:
            return None
        duration = None
        if isinstance(loiter_prop, dict):
            try:
                duration = float(loiter_prop.get("time") or 0.0)
            except Exception:
                duration = None
            if duration is not None and duration <= 0.0:
                duration = None
        return {
            "target_id": int(target_id),
            "filming": dict(filming_prop),
            "loiter": dict(loiter_prop) if isinstance(loiter_prop, dict) else None,
            "duration": duration,
            "fov_deg": float(self._resolve_tracking_fov(filming_prop)),
        }

    def _footprint_polygon(
        self, simv: SimVehicle, focus: tuple[float, float, float], fov_deg: float
    ) -> list[tuple[float, float]] | None:
        projection = self._resolve_footprint_projection(simv, focus, fov_deg)
        if projection is None:
            return None
        return [(float(px), float(py)) for px, py, _pz in projection.corners]

    def _point_in_poly(self, x: float, y: float, poly: list[tuple[float, float]]) -> bool:
        inside = False
        n = len(poly)
        if n < 3:
            return False
        j = n - 1
        for i in range(n):
            xi, yi = poly[i]
            xj, yj = poly[j]
            intersect = (yi > y) != (yj > y)
            if intersect:
                x_at_y = (xj - xi) * (y - yi) / (yj - yi + 1e-9) + xi
                if x < x_at_y:
                    inside = not inside
            j = i
        return inside

    def _footprint_contains(self, simv: SimVehicle, target: GroundTarget, fov_deg: float) -> bool:
        focus = self._filming_targets.get(simv.label)
        if focus is None:
            focus = self._default_downward_target(simv.vehicle)
        poly = self._footprint_polygon(simv, focus, fov_deg)
        if not poly:
            return False
        return self._point_in_poly(float(target.x), float(target.y), poly)

    def _camera_view_polygon(
        self,
        simv: SimVehicle,
        fov_deg: float,
    ) -> list[tuple[float, float]] | None:
        if fov_deg <= 0.0:
            return None
        focus = self._filming_targets.get(simv.label)
        if focus is None:
            focus = self._default_downward_target(simv.vehicle)
        return self._footprint_polygon(simv, focus, fov_deg)

    def _point_in_camera_view_with_polygon(
        self,
        simv: SimVehicle,
        *,
        x: float,
        y: float,
        z: float,
        fov_deg: float,
        polygon: list[tuple[float, float]] | None,
    ) -> bool:
        s = simv.vehicle.s
        dx = float(x - s.x)
        dy = float(y - s.y)
        dz = float(z - s.z)
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if self.uav_detection_range_m > 0.0 and dist > self.uav_detection_range_m:
            return False
        if not self._check_los_flat(s.x, s.y, s.z, x, y, z):
            return False
        if fov_deg <= 0.0:
            return True
        if not polygon:
            return False
        return self._point_in_poly(float(x), float(y), polygon)

    def _load_target_info_map(self) -> dict[str, dict[str, Any]]:
        path = db_paths.get_db_subpath("DSS_Internal", "targetInfo.json")
        try:
            stat = path.stat()
            mtime_ns = int(getattr(stat, "st_mtime_ns", int(stat.st_mtime * 1_000_000_000)))
        except Exception:
            self._target_info_cache_mtime_ns = None
            self._target_info_cache_map = {}
            return {}
        if (
            self._target_info_cache_mtime_ns is not None
            and self._target_info_cache_mtime_ns == mtime_ns
            and isinstance(self._target_info_cache_map, dict)
        ):
            return self._target_info_cache_map
        try:
            raw = path.read_text(encoding="utf-8")
            data = json.loads(raw) if raw else {}
            target_list = data.get("targetList") if isinstance(data, dict) else {}
            if not isinstance(target_list, dict):
                target_list = {}
        except Exception:
            target_list = {}
        normalized: dict[str, dict[str, Any]] = {}
        for key, value in target_list.items():
            if isinstance(value, dict):
                normalized[str(key)] = value
        self._target_info_cache_mtime_ns = mtime_ns
        self._target_info_cache_map = normalized
        return normalized

    def _target_info_visibility_context(self) -> dict[str, Any]:
        target_map = self._load_target_info_map()
        entries_by_id: dict[int, list[dict[str, Any]]] = {}
        ignored_ids: set[int] = set()
        ignored_xy: list[tuple[float, float]] = []
        if not target_map:
            return {
                "entries_by_id": entries_by_id,
                "ignored_ids": ignored_ids,
                "ignored_xy": ignored_xy,
            }

        for entry in target_map.values():
            if not isinstance(entry, dict):
                continue
            try:
                entry_id = int(entry.get("targetID", -1))
            except Exception:
                entry_id = -1
            if entry_id > 0:
                entries_by_id.setdefault(entry_id, []).append(entry)

            try:
                ignored = int(entry.get("isIgnored") or 0) != 0
            except Exception:
                ignored = False
            if not ignored:
                continue
            if entry_id > 0:
                ignored_ids.add(entry_id)
            coord = entry.get("coordinate")
            if not isinstance(coord, dict) or self.geo is None:
                continue
            try:
                lat = float(coord.get("latitude"))
                lon = float(coord.get("longitude"))
                x, y = self.geo.lonlat_to_xy(lon, lat)
                ignored_xy.append((float(x), float(y)))
            except Exception:
                continue

        return {
            "entries_by_id": entries_by_id,
            "ignored_ids": ignored_ids,
            "ignored_xy": ignored_xy,
        }

    def _target_is_ignored_in_info(self, target_id: int) -> bool:
        try:
            target_id = int(target_id)
        except Exception:
            return False
        if target_id <= 0:
            return False
        related_ids = self._related_target_ids_in_info(target_id)
        target_map = self._load_target_info_map()
        for entry in target_map.values():
            if not isinstance(entry, dict):
                continue
            try:
                entry_id = int(entry.get("targetID", -1))
            except Exception:
                continue
            if entry_id not in related_ids:
                continue
            try:
                if int(entry.get("isIgnored") or 0) != 0:
                    return True
            except Exception:
                continue
        return False

    def _related_target_ids_in_info(self, target_id: int) -> set[int]:
        related_ids: set[int] = set()
        try:
            target_id_int = int(target_id)
        except Exception:
            return related_ids
        if target_id_int <= 0:
            return related_ids
        related_ids.add(target_id_int)
        try:
            mapped = self._target_id_map_0402.get(target_id_int)
            if mapped is not None:
                mapped_int = int(mapped)
                if mapped_int > 0:
                    related_ids.add(mapped_int)
        except Exception:
            pass
        for raw_id, assigned_id in list(self._target_id_map_0402.items()):
            try:
                raw_int = int(raw_id)
                assigned_int = int(assigned_id)
            except Exception:
                continue
            if raw_int <= 0 or assigned_int <= 0:
                continue
            if raw_int == target_id_int or assigned_int == target_id_int:
                related_ids.add(raw_int)
                related_ids.add(assigned_int)
        return related_ids

    def _target_is_destroyed_in_info(self, target_id: int) -> bool:
        related_ids = self._related_target_ids_in_info(target_id)
        if not related_ids:
            return False
        target_map = self._load_target_info_map()
        matched = False
        for entry in target_map.values():
            if not isinstance(entry, dict):
                continue
            try:
                entry_id = int(entry.get("targetID", -1))
            except Exception:
                continue
            if entry_id not in related_ids:
                continue
            matched = True
            # targetInfo can retain stale destroyed summary rows while the
            # active watcher-specific row is still alive; only stop when all
            # matching rows agree the target is destroyed.
            if not _coerce_bool(entry.get("isDestroyed"), False):
                return False
        return matched

    def _ground_target_is_ignored(self, target: GroundTarget) -> bool:
        try:
            raw_id = int(getattr(target, "id", 0))
        except Exception:
            raw_id = 0
        if raw_id > 0:
            assigned_id = self._target_id_map_0402.get(raw_id)
            if assigned_id is not None and self._target_is_ignored_in_info(int(assigned_id)):
                return True
        target_map = self._load_target_info_map()
        if not target_map:
            return False
        for entry in target_map.values():
            if not isinstance(entry, dict):
                continue
            try:
                if int(entry.get("isIgnored") or 0) == 0:
                    continue
            except Exception:
                continue
            coord = entry.get("coordinate")
            if not isinstance(coord, dict):
                continue
            try:
                lat = float(coord.get("latitude"))
                lon = float(coord.get("longitude"))
            except Exception:
                continue
            try:
                x, y = self.geo.lonlat_to_xy(lon, lat) if self.geo is not None else (None, None)
            except Exception:
                continue
            if x is None or y is None:
                continue
            dx = float(target.x - x)
            dy = float(target.y - y)
            if math.hypot(dx, dy) <= _TARGET_INFO_MATCH_RADIUS_M:
                return True
        return False

    def _target_in_view(self, simv: SimVehicle, tgt: GroundTarget, fov_deg: float) -> bool:
        polygon = self._camera_view_polygon(simv, float(fov_deg))
        return self._point_in_camera_view_with_polygon(
            simv,
            x=float(tgt.x),
            y=float(tgt.y),
            z=float(tgt.z),
            fov_deg=float(fov_deg),
            polygon=polygon,
        )

    def _visible_tracking_targets(
        self,
        simv: SimVehicle,
        fov_deg: float,
        *,
        candidate_ids: set[int] | None = None,
    ) -> list[GroundTarget]:
        if not self.targets:
            return []
        visible: list[tuple[float, int, GroundTarget]] = []
        s = simv.vehicle.s
        polygon = self._camera_view_polygon(simv, float(fov_deg)) if float(fov_deg) > 0.0 else None
        if float(fov_deg) > 0.0 and not polygon:
            return []
        info_context = self._target_info_visibility_context()
        entries_by_id: dict[int, list[dict[str, Any]]] = info_context.get("entries_by_id") or {}
        ignored_ids: set[int] = info_context.get("ignored_ids") or set()
        ignored_xy: list[tuple[float, float]] = info_context.get("ignored_xy") or []
        related_cache: dict[int, set[int]] = {}
        destroyed_info_cache: dict[int, bool] = {}

        def _related_ids(target_id: int) -> set[int]:
            try:
                target_id = int(target_id)
            except Exception:
                return set()
            if target_id <= 0:
                return set()
            cached = related_cache.get(target_id)
            if cached is not None:
                return cached
            related = self._related_target_ids_in_info(target_id)
            related.add(int(target_id))
            related_cache[target_id] = related
            return related

        def _destroyed_in_info(target_id: int) -> bool:
            try:
                target_id = int(target_id)
            except Exception:
                return False
            if target_id <= 0:
                return False
            cached = destroyed_info_cache.get(target_id)
            if cached is not None:
                return cached
            matched: list[dict[str, Any]] = []
            for related_id in _related_ids(target_id):
                matched.extend(entries_by_id.get(int(related_id), []))
            result = bool(matched) and all(
                _coerce_bool(entry.get("isDestroyed"), False)
                for entry in matched
                if isinstance(entry, dict)
            )
            destroyed_info_cache[target_id] = result
            return result

        def _ignored_in_info(target: GroundTarget, raw_id: int, assigned_id: int | None) -> bool:
            if ignored_ids:
                candidate_ids = set()
                if raw_id > 0:
                    candidate_ids.update(_related_ids(raw_id))
                if assigned_id is not None and int(assigned_id) > 0:
                    candidate_ids.update(_related_ids(int(assigned_id)))
                if candidate_ids & ignored_ids:
                    return True
            if ignored_xy:
                for ix, iy in ignored_xy:
                    dx = float(target.x - ix)
                    dy = float(target.y - iy)
                    if math.hypot(dx, dy) <= _TARGET_INFO_MATCH_RADIUS_M:
                        return True
            return False

        for tgt in self.targets:
            if not tgt.alive:
                continue
            try:
                raw_id = int(getattr(tgt, "id", 0))
            except Exception:
                raw_id = 0
            assigned_id = self._target_id_map_0402.get(raw_id) if raw_id > 0 else None
            if _ignored_in_info(tgt, raw_id, assigned_id):
                continue
            if raw_id > 0 and raw_id in self._destroyed_target_ids:
                continue
            if assigned_id is not None and assigned_id in self._destroyed_target_ids:
                continue
            if raw_id > 0 and _destroyed_in_info(raw_id):
                continue
            if candidate_ids is not None and raw_id not in candidate_ids:
                continue
            if candidate_ids is None and raw_id > 0:
                watcher_id = self._target_watcher_0402.get(raw_id)
                if watcher_id is not None and int(watcher_id) != int(simv.aircraft_id):
                    continue
            owner = self._tracking_target_owner.get(raw_id)
            if owner and owner != simv.label:
                continue
            if not self._point_in_camera_view_with_polygon(
                simv,
                x=float(tgt.x),
                y=float(tgt.y),
                z=float(tgt.z),
                fov_deg=float(fov_deg),
                polygon=polygon,
            ):
                continue
            dx = float(tgt.x - s.x)
            dy = float(tgt.y - s.y)
            dz = float(tgt.z - s.z)
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            visible.append((dist, raw_id, tgt))
        visible.sort(key=lambda item: (item[0], item[1]))
        return [tgt for _dist, _raw_id, tgt in visible]

    def _find_detected_target(self, simv: SimVehicle, fov_deg: float) -> tuple[int, GroundTarget] | None:
        visible = self._visible_tracking_targets(simv, fov_deg)
        if not visible:
            return None
        best = visible[0]
        try:
            target_id = int(getattr(best, "id", 0))
        except Exception:
            target_id = 0
        if target_id <= 0:
            return None
        return target_id, best

    def _start_tracking_preview(
        self,
        simv: SimVehicle,
        targets: list[GroundTarget],
        filming_prop: dict | None,
        fov_deg: float,
    ) -> bool:
        if self.multi_target_preview_sec <= 0.0:
            return False
        target_ids: list[int] = []
        seen: set[int] = set()
        for target in targets:
            try:
                raw_id = int(getattr(target, "id", 0))
            except Exception:
                raw_id = 0
            if raw_id <= 0 or raw_id in seen:
                continue
            seen.add(raw_id)
            target_ids.append(raw_id)
        if len(target_ids) < 2:
            return False
        try:
            aircraft_id = int(simv.aircraft_id)
        except Exception:
            aircraft_id = 0
        if aircraft_id > 0:
            for raw_id in target_ids:
                self._target_watcher_0402[int(raw_id)] = int(aircraft_id)
        self._tracking_preview_state[simv.label] = TrackingPreviewState(
            target_ids=target_ids,
            chosen_target_id=int(target_ids[0]),
            fov_deg=float(fov_deg),
            filming_prop=dict(filming_prop) if isinstance(filming_prop, dict) else None,
            start_time=float(self.sim_time),
            end_time=float(self.sim_time) + float(self.multi_target_preview_sec),
        )
        return True

    def _assign_0402_target_id(self, target: GroundTarget) -> int:
        try:
            raw_id = int(getattr(target, "id", 0))
        except Exception:
            raw_id = 0
        if raw_id <= 0:
            try:
                raw_id = self.targets.index(target) + 1
            except Exception:
                raw_id = 0
        if raw_id <= 0:
            return 0
        if raw_id in self._target_id_map_0402:
            return self._target_id_map_0402[raw_id]
        assigned = int(self._target_id_seq_0402)
        self._target_id_seq_0402 += 1
        self._target_id_map_0402[raw_id] = assigned
        return assigned

    def _fuel_multiplier_for(self, simv: SimVehicle) -> float:
        override = self._agent_overrides.get(simv.label, {})
        mult = _coerce_float(override.get("fuelConsumption", 1.0), 1.0)
        if not math.isfinite(mult):
            mult = 1.0
        return max(0.0, mult)

    def _fuel_remaining(self, multiplier: float = 1.0) -> float:
        full = 15.0
        endurance = 3.0 * 3600.0
        mult = max(0.0, float(multiplier))
        ratio = 1.0 - (max(0.0, float(self.sim_time)) * mult) / endurance
        return max(0.0, full * ratio)

    def _pulse_on_mission(self, label: str, duration: float | None = None) -> None:
        try:
            dur = float(duration) if duration is not None else max(0.25, float(self.dt))
        except Exception:
            dur = max(0.25, float(self.dt))
        self._on_mission_pulse[str(label)] = float(self.sim_time) + dur

    def _pulse_filming(self, label: str, duration: float | None = None) -> None:
        try:
            dur = float(duration) if duration is not None else max(0.25, float(self.dt))
        except Exception:
            dur = max(0.25, float(self.dt))
        self._filming_pulse[str(label)] = float(self.sim_time) + dur

    def _filming_operation_mode(self, filming_prop: Any) -> int:
        if not isinstance(filming_prop, dict):
            return 0
        value = filming_prop.get("operationMode")
        if value is None:
            value = filming_prop.get("operationalMode")
        try:
            return int(value or 0)
        except Exception:
            return 0

    def _has_active_filming_prop(self, filming_prop: Any) -> bool:
        if not isinstance(filming_prop, dict) or not filming_prop:
            return False
        if self._filming_operation_mode(filming_prop) > 0:
            return True
        for key in (
            "coordinateOrientation",
            "lineSearch",
            "areaSearch",
            "autoTracking",
            "aircraftFixed",
            "CoordinateOrientation",
            "LineSearch",
            "AreaSearch",
            "AutoTracking",
            "AircraftFixed",
        ):
            if isinstance(filming_prop.get(key), dict):
                return True
        return False

    def _line_search_complete(self, label: str) -> bool:
        state = self._line_search_state.get(str(label))
        if not isinstance(state, dict):
            return False
        if bool(state.get("complete")):
            return True
        segments = state.get("segments") or []
        if not segments:
            return False
        try:
            seg_idx = int(state.get("seg_idx", 0))
            seg_t = float(state.get("seg_t", 0.0))
        except Exception:
            return False
        return bool(seg_idx >= len(segments) - 1 and seg_t >= 1.0 - 1e-6)

    def _target_filming_complete_for_pulse(self, simv: SimVehicle, target: WaypointTarget | None) -> bool:
        filming_prop = getattr(target, "filming", None) if target is not None else None
        if not self._has_active_filming_prop(filming_prop):
            return False
        if self._filming_operation_mode(filming_prop) == 2:
            return self._line_search_complete(simv.label)
        return True

    def _filming_status_for(
        self,
        simv: SimVehicle,
        *,
        filming_prop: Any,
        flying_status: int,
    ) -> int:
        if simv.airframe != "uav":
            return 0

        label = str(simv.label)
        if self._has_active_filming_prop(filming_prop):
            op_mode = self._filming_operation_mode(filming_prop)
            if op_mode == 2:
                return 2 if self._line_search_complete(label) else 1
            if op_mode == 3:
                tracking = self._tracking_state.get(label)
                if tracking is not None and tracking.stage >= 1:
                    return 1
            return 2 if int(flying_status) == 2 else 1

        pulse = self._filming_pulse.get(label)
        if pulse is not None:
            if float(self.sim_time) < float(pulse):
                return 2
            self._filming_pulse.pop(label, None)
        return 0

    def _flight_mode_for(self, simv: SimVehicle) -> int:
        forced = self._forced_commands.get(simv.label)
        if forced and "flight_mode" in forced:
            return int(forced.get("flight_mode") or 0)
        tracking = self._tracking_state.get(simv.label)
        if tracking is not None and tracking.stage >= 1:
            return 9
        if self._active_formation_spec(simv) is not None:
            return 6
        ctrl = simv.controller
        try:
            current = ctrl.current_target()
        except Exception:
            current = None
        if current is not None:
            loiter_prop = current.loiter if isinstance(getattr(current, "loiter", None), dict) else None
            if loiter_prop is not None:
                if (loiter_prop.get("time") or 0) or (loiter_prop.get("radius") or 0) or (loiter_prop.get("speed") or 0):
                    return 8
        if getattr(ctrl, "finished", False) or getattr(ctrl, "is_loitering", False):
            return 8
        return 7

    def _reported_uav_current_waypoint_id(
        self,
        *,
        simv: SimVehicle,
        tracking: TrackingState | None,
        forced: dict[str, Any] | None,
        flight_mode: int,
    ) -> int | None:
        # While target-following is active, mirror the external system behavior:
        # keep the saved waypoint internally for resume, but publish no live WP in 0401.
        if int(flight_mode) == 9 and tracking is not None and tracking.stage >= 1:
            return None

        current_wp = 0
        if tracking is not None and tracking.saved_wp_id is not None:
            current_wp = int(tracking.saved_wp_id)
        else:
            try:
                tgt = simv.controller.current_target()
            except Exception:
                tgt = None
            if tgt is not None and tgt.wp_id is not None:
                try:
                    current_wp = int(tgt.wp_id)
                except Exception:
                    current_wp = 0
            elif forced is not None:
                try:
                    current_wp = int(forced.get("current_wp_id") or 0)
                except Exception:
                    current_wp = 0

        if tracking is None:
            try:
                if getattr(simv.controller, "finished", False):
                    current_wp = 0
            except Exception:
                pass
        return int(current_wp)

    def _on_mission_for(self, simv: SimVehicle) -> int:
        if simv.airframe != "uav":
            return 0
        pending_expected = self._pending_input_advances.get(int(simv.aircraft_id))
        if pending_expected is not None:
            current_input_id = self._current_input_mission_id_for(int(simv.aircraft_id))
            if current_input_id is not None and int(current_input_id) == int(pending_expected):
                return 1
        guard_until = self._input_advance_guard_until.get(simv.label)
        if guard_until is not None:
            if float(self.sim_time) < float(guard_until):
                return 1
            self._input_advance_guard_until.pop(simv.label, None)
        pulse = self._on_mission_pulse.get(simv.label)
        if pulse is not None:
            if float(self.sim_time) < float(pulse):
                return 2
            self._on_mission_pulse.pop(simv.label, None)
        tracking = self._tracking_state.get(simv.label)
        if tracking is not None and tracking.stage >= 1:
            return 1
        forced = self._forced_commands.get(simv.label)
        if forced and forced.get("type") in ("hold", "rtb", "orbit_fault"):
            return 1
        ctrl = simv.controller
        if getattr(ctrl, "blocked", False):
            return 2
        if getattr(ctrl, "finished", False):
            return 2
        if getattr(ctrl, "is_loitering", False):
            targets = getattr(ctrl, "targets", None)
            curr_idx = getattr(ctrl, "curr_idx", None)
            if isinstance(targets, list) and targets and isinstance(curr_idx, int):
                if curr_idx >= len(targets) - 1:
                    return 2
        return 1

    def _build_agent_state_0401(self, simv: SimVehicle, *, timestamp: int) -> dict | None:
        geo = self.geo
        if geo is None:
            return None
        s = simv.vehicle.s
        lon, lat = geo.xy_to_lonlat(float(s.x), float(s.y))
        overrides = self._agent_overrides.get(simv.label, {})
        fuel = self._fuel_remaining(self._fuel_multiplier_for(simv))
        health_value = 1 if simv.alive else 2
        if simv.alive and "health" in overrides:
            health_value = max(0, min(2, _coerce_int(overrides.get("health"), health_value)))
        agent = {
            "aircraftID": int(simv.aircraft_id),
            "isUnmanned": simv.airframe == "uav",
            "coordinate": {
                "latitude": float(lat),
                "longitude": float(lon),
                "altitude": float(s.z),
            },
            "velocity": {
                "speed": float(getattr(s, "u", 0.0)),
                "heading": _aircraft_yaw_to_nav_heading_deg(float(getattr(s, "yaw", 0.0))),
            },
            "fuel": float(fuel),
            "health": int(health_value),
            "lastSignalTime": int(timestamp),
        }
        if simv.airframe == "uav":
            tracking = self._tracking_state.get(simv.label)
            forced = self._forced_commands.get(simv.label)
            target_id = 0
            if tracking is not None and tracking.stage >= 1 and tracking.target.alive:
                target_id = self._assign_0402_target_id(tracking.target)
            payload_health = _coerce_int(overrides.get("payloadHealth", 1), 1)
            fuel_warn = _coerce_int(overrides.get("fuelWarning", 0), 0)
            flight_mode = int(self._flight_mode_for(simv))
            flying = int(self._on_mission_for(simv))
            filming_prop = self._filming_props.get(simv.label)
            filming = int(self._filming_status_for(simv, filming_prop=filming_prop, flying_status=flying))
            current_wp = self._reported_uav_current_waypoint_id(
                simv=simv,
                tracking=tracking,
                forced=forced,
                flight_mode=flight_mode,
            )
            loiter_coord = None
            try:
                tgt = simv.controller.current_target()
            except Exception:
                tgt = None
            if flight_mode == 8:
                center = None
                ctrl = simv.controller
                if getattr(ctrl, "is_loitering", False):
                    center = getattr(ctrl, "loiter_center", None)
                if center is None and tgt is not None:
                    center = getattr(tgt, "pos", None)
                if center is None:
                    center = (float(s.x), float(s.y), float(s.z))
                try:
                    lon_l, lat_l = geo.xy_to_lonlat(float(center[0]), float(center[1]))
                    loiter_coord = {
                        "latitude": float(lat_l),
                        "longitude": float(lon_l),
                        "altitude": float(center[2]),
                    }
                except Exception:
                    loiter_coord = None
            agent["unmannedInfo"] = {
                "currentWaypointID": (
                    {"waypointID": int(current_wp)} if current_wp is not None else None
                ),
                "flightMode": int(flight_mode),
                "targetFollowing": {"targetID": int(target_id)},
                "payloadHealth": int(payload_health),
                "fuelWarning": int(fuel_warn),
                "flying": int(flying),
            }
            if loiter_coord is not None:
                agent["unmannedInfo"]["loiterCoordinate"] = loiter_coord
            sensor_info: dict[str, Any] = {"filming": int(filming)}
            if self._has_active_filming_prop(filming_prop):
                op_mode = filming_prop.get("operationMode") or filming_prop.get("operationalMode")
                sensor_type = filming_prop.get("sensorType")
                fov_val = filming_prop.get("fieldOfView") or filming_prop.get("fov")
                try:
                    op_mode = int(op_mode) if op_mode is not None else None
                except Exception:
                    op_mode = None
                try:
                    sensor_type = int(sensor_type) if sensor_type is not None else None
                except Exception:
                    sensor_type = None
                try:
                    fov_val = float(fov_val) if fov_val is not None else None
                except Exception:
                    fov_val = None
                if op_mode is not None:
                    sensor_info["operationalMode"] = int(op_mode)
                if sensor_type is not None:
                    sensor_info["sensorType"] = int(sensor_type)
                if fov_val is not None:
                    sensor_info["fov"] = float(fov_val)
                center = self._filming_targets.get(simv.label)
                if center is None:
                    center = self._default_downward_target(simv.vehicle)
                if center is not None and geo is not None:
                    try:
                        projection = None
                        center_coordinate = (
                            float(center[0]),
                            float(center[1]),
                            float(center[2]),
                        )
                        if fov_val is not None and float(fov_val) > 0.0:
                            projection = self._resolve_footprint_projection(simv, center, float(fov_val))
                            if projection is not None:
                                center_coordinate = projection.center_hit
                        c_lon, c_lat = geo.xy_to_lonlat(float(center_coordinate[0]), float(center_coordinate[1]))
                        sensor_info["centerCoordinate"] = {
                            "latitude": float(c_lat),
                            "longitude": float(c_lon),
                            "altitude": float(center_coordinate[2]),
                        }
                        if projection is not None and projection.corners:
                            corners = []
                            for px, py, pz in projection.corners:
                                lon_fp, lat_fp = geo.xy_to_lonlat(float(px), float(py))
                                corners.append(
                                    {
                                        "latitude": float(lat_fp),
                                        "longitude": float(lon_fp),
                                        "altitude": float(pz),
                                    }
                                )
                            if corners:
                                sensor_info["footprintCornerList"] = corners
                    except Exception:
                        pass
            agent["unmannedInfo"]["sensorInfo"] = sensor_info
        else:
            weapons_override = overrides.get("weapons") if isinstance(overrides.get("weapons"), dict) else {}
            type1 = _coerce_int(weapons_override.get("type1", 5), 5)
            type2 = _coerce_int(weapons_override.get("type2", 10), 10)
            type3 = _coerce_int(weapons_override.get("type3", 100), 100)
            link_override = overrides.get("datalink") if isinstance(overrides.get("datalink"), dict) else {}
            link_uav1 = _coerce_bool(link_override.get("uav1", True), True)
            link_uav2 = _coerce_bool(link_override.get("uav2", True), True)
            link_uav3 = _coerce_bool(link_override.get("uav3", True), True)
            agent["mannedInfo"] = {
                "weapons": {"type1": int(type1), "type2": int(type2), "type3": int(type3)},
                "datalinkStatus": {
                    "isConnectedToUAV1": bool(link_uav1),
                    "isConnectedToUAV2": bool(link_uav2),
                    "isConnectedToUAV3": bool(link_uav3),
                },
            }
        return agent

    def _push_0401_once(self) -> None:
        if not self.integration:
            return
        if not getattr(self.integration, "enabled", False):
            return
        if not self.vehicles:
            return
        now_ts = self._sim_timestamp_ms_2000()
        agent_states = []
        for simv in self.vehicles.values():
            state = self._build_agent_state_0401(simv, timestamp=now_ts)
            if state:
                agent_states.append(state)
        if not agent_states:
            return
        payload = {
            "timestamp": int(now_ts),
            "source": "IDM",
            "agentStateList": agent_states,
        }
        try:
            agent_status_snapshot.append_agent_status_log(payload, source="SIM")
        except Exception:
            pass
        try:
            agent_status_snapshot.append_agent_status_json_log(payload)
        except Exception:
            pass
        try:
            self.integration.send_custom("0401", payload)
        except Exception:
            return

    def _maybe_push_0401(self) -> None:
        if not self.integration or not getattr(self.integration, "enabled", False):
            return
        if not self.vehicles:
            return
        now = time.monotonic()
        last_emit = self._last_0401_emit_wall_time
        if last_emit is None or (now - last_emit) >= self._0401_active_interval_sec:
            self._last_0401_emit_wall_time = now
            self._push_0401_once()

    def _emit_0402(self, *, body: dict, aircraft_id: int, target_id: int) -> None:
        self._events_0402.append(
            {
                "step": int(self.step_count),
                "simTime": float(self.sim_time),
                "aircraftID": int(aircraft_id),
                "targetID": int(target_id),
                "body": body,
            }
        )
        if self.integration and getattr(self.integration, "enabled", False):
            try:
                self.integration.send_custom("0402", body)
            except Exception:
                pass

    def _build_0402_target_payload(
        self,
        *,
        target: GroundTarget,
        aircraft_id: int,
        watcher_id: int | None,
        fov_deg: float | None,
        target_in_frame: int,
        is_destroyed: int,
        threat: float,
    ) -> tuple[dict, int] | None:
        built = self._build_0402_target_entry(
            target=target,
            watcher_id=watcher_id,
            target_in_frame=target_in_frame,
            is_destroyed=is_destroyed,
            threat=threat,
        )
        if built is None:
            return None
        target_entry, coord, assigned_id = built
        body: dict[str, object] = {
            "timestamp": self._sim_timestamp_ms_2000(),
            "source": "SIM",
            "targetList": [target_entry],
        }
        if fov_deg is not None:
            body["roiInfo"] = {
                "aircraftID": int(aircraft_id),
                "coordinate": coord,
                "fov": float(fov_deg),
            }
        return body, assigned_id

    def _build_0402_target_entry(
        self,
        *,
        target: GroundTarget,
        watcher_id: int | None,
        target_in_frame: int,
        is_destroyed: int,
        threat: float,
    ) -> tuple[dict[str, object], dict[str, float], int] | None:
        geo = self.geo
        if geo is None:
            return None
        try:
            lon, lat = geo.xy_to_lonlat(float(target.x), float(target.y))
        except Exception:
            return None
        coord: dict[str, float] = {
            "latitude": float(lat),
            "longitude": float(lon),
            "altitude": float(target.z),
        }
        try:
            target_type = int(getattr(target, "type_id", 0) or 0)
        except Exception:
            target_type = 0
        assigned_id = int(self._assign_0402_target_id(target))
        target_entry: dict[str, object] = {
            "targetID": int(assigned_id),
            "targetType": int(target_type),
            "coordinate": coord,
            "targetInFrame": int(target_in_frame),
            "isDestroyed": int(is_destroyed),
            "threat": float(threat),
        }
        if watcher_id is not None:
            target_entry["watcher"] = {"aircraftID": int(watcher_id)}
        return target_entry, coord, assigned_id

    def _emit_tracking_preview_0402(self, simv: SimVehicle, preview: TrackingPreviewState) -> None:
        try:
            aircraft_id = int(simv.aircraft_id)
        except Exception:
            aircraft_id = 0
        candidate_ids = {int(value) for value in (preview.target_ids or []) if int(value) > 0}
        if not candidate_ids:
            return
        targets = self._visible_tracking_targets(simv, float(preview.fov_deg), candidate_ids=candidate_ids)
        if not targets:
            return
        roi_coord: dict[str, float] | None = None
        target_entries: list[dict[str, object]] = []
        for target in targets:
            built = self._build_0402_target_entry(
                target=target,
                watcher_id=aircraft_id,
                target_in_frame=1,
                is_destroyed=0,
                threat=100.0,
            )
            if built is None:
                continue
            target_entry, coord, _assigned_id = built
            try:
                raw_id = int(getattr(target, "id", 0))
            except Exception:
                raw_id = 0
            if raw_id > 0:
                self._target_watcher_0402[raw_id] = int(aircraft_id)
            if roi_coord is None or raw_id == int(preview.chosen_target_id):
                roi_coord = coord
            target_entries.append(target_entry)
        if not target_entries:
            return
        body: dict[str, object] = {
            "timestamp": self._sim_timestamp_ms_2000(),
            "source": "SIM",
            "targetList": target_entries,
        }
        if roi_coord is not None:
            body["roiInfo"] = {
                "aircraftID": int(aircraft_id),
                "coordinate": roi_coord,
                "fov": float(preview.fov_deg),
            }
        self._emit_0402(body=body, aircraft_id=aircraft_id, target_id=0)

    def _push_0402_updates_once(self) -> None:
        if self.geo is None:
            return
        for label, preview in list(self._tracking_preview_state.items()):
            simv = self.vehicles.get(label)
            if simv is None or not simv.alive or label in self._tracking_state:
                self._clear_tracking_preview(label)
                continue
            self._emit_tracking_preview_0402(simv, preview)
        for label, state in list(self._tracking_state.items()):
            simv = self.vehicles.get(label)
            if simv is None or not simv.alive:
                continue
            target = state.target
            if target is None or not target.alive:
                continue
            try:
                aircraft_id = int(simv.aircraft_id)
            except Exception:
                aircraft_id = 0
            try:
                raw_id = int(getattr(target, "id", 0))
            except Exception:
                raw_id = 0
            if raw_id > 0:
                self._target_watcher_0402[raw_id] = int(aircraft_id)
            payload = self._build_0402_target_payload(
                target=target,
                aircraft_id=aircraft_id,
                watcher_id=aircraft_id,
                fov_deg=float(state.fov_deg),
                target_in_frame=1,
                is_destroyed=0,
                threat=100.0,
            )
            if payload is None:
                continue
            body, assigned_id = payload
            self._emit_0402(body=body, aircraft_id=aircraft_id, target_id=assigned_id)

        for target in self.targets:
            if target.alive:
                continue
            try:
                raw_id = int(getattr(target, "id", 0))
            except Exception:
                raw_id = 0
            if raw_id <= 0:
                continue
            if raw_id in self._reported_0402_destroyed:
                continue
            watcher_id = self._target_watcher_0402.get(raw_id)
            if watcher_id is None:
                owner = self._tracking_target_owner.get(raw_id)
                if owner:
                    watcher_id = _label_to_aircraft_id(owner)
            aircraft_id = int(watcher_id) if watcher_id is not None else 0
            payload = self._build_0402_target_payload(
                target=target,
                aircraft_id=aircraft_id,
                watcher_id=watcher_id,
                fov_deg=None,
                target_in_frame=0,
                is_destroyed=1,
                threat=0.0,
            )
            if payload is None:
                continue
            body, assigned_id = payload
            self._emit_0402(body=body, aircraft_id=aircraft_id, target_id=assigned_id)
            self._reported_0402_destroyed.add(raw_id)

    def _maybe_push_0402(self) -> None:
        interval = 0.2
        if self._last_0402_sim_time is None:
            self._last_0402_sim_time = float(self.sim_time)
            self._push_0402_updates_once()
            return
        if (self.sim_time - self._last_0402_sim_time) >= interval:
            self._last_0402_sim_time = float(self.sim_time)
            self._push_0402_updates_once()

    def _record_0402_roi(self, simv: SimVehicle, target: GroundTarget, fov_deg: float) -> None:
        geo = self.geo
        if geo is None:
            return
        try:
            aircraft_id = int(simv.aircraft_id)
        except Exception:
            aircraft_id = 0
        try:
            raw_id = int(getattr(target, "id", 0))
        except Exception:
            raw_id = 0
        if raw_id <= 0:
            try:
                raw_id = self.targets.index(target) + 1
            except Exception:
                raw_id = 0
        key = (aircraft_id, raw_id)
        if key in self._reported_0402_roi:
            return
        try:
            lon, lat = geo.xy_to_lonlat(float(target.x), float(target.y))
        except Exception:
            return
        coord = {"latitude": float(lat), "longitude": float(lon), "altitude": float(target.z)}
        body = {
            "timestamp": self._sim_timestamp_ms_2000(),
            "source": "SIM",
            "roiInfo": {
                "aircraftID": aircraft_id,
                "coordinate": coord,
                "fov": float(fov_deg),
            },
        }
        self._emit_0402(body=body, aircraft_id=aircraft_id, target_id=0)
        self._reported_0402_roi.add(key)

    def _record_0402_roi_mock(self, simv: SimVehicle, roi: RoiMock, fov_deg: float) -> None:
        geo = self.geo
        if geo is None:
            return
        try:
            aircraft_id = int(simv.aircraft_id)
        except Exception:
            aircraft_id = 0
        try:
            lon, lat = geo.xy_to_lonlat(float(roi.x), float(roi.y))
        except Exception:
            return
        coord = {
            "latitude": float(lat),
            "longitude": float(lon),
            "altitude": float(roi.z),
        }
        body = {
            "timestamp": self._sim_timestamp_ms_2000(),
            "source": "SIM",
            "roiInfo": {
                "aircraftID": int(aircraft_id),
                "coordinate": coord,
                "fov": float(fov_deg),
            },
        }
        self._emit_0402(body=body, aircraft_id=aircraft_id, target_id=0)

    def _roi_focus_active(self, label: str) -> RoiFocusState | None:
        state = self._roi_focus_state.get(str(label))
        if state is None:
            return None
        roi_exists = any(int(roi.id) == int(state.roi_id) for roi in self._roi_mocks)
        if roi_exists and float(self.sim_time) < float(state.end_time):
            return state
        self._roi_focus_state.pop(str(label), None)
        return None

    def _point_in_camera_view(
        self,
        simv: SimVehicle,
        *,
        x: float,
        y: float,
        z: float,
        fov_deg: float,
    ) -> bool:
        polygon = self._camera_view_polygon(simv, float(fov_deg)) if float(fov_deg) > 0.0 else None
        return self._point_in_camera_view_with_polygon(
            simv,
            x=float(x),
            y=float(y),
            z=float(z),
            fov_deg=float(fov_deg),
            polygon=polygon,
        )

    def _visible_roi_mocks(self, simv: SimVehicle, fov_deg: float) -> list[RoiMock]:
        if not self._roi_mocks:
            return []
        try:
            aircraft_id = int(simv.aircraft_id)
        except Exception:
            aircraft_id = 0
        visible: list[tuple[float, int, RoiMock]] = []
        s = simv.vehicle.s
        polygon = self._camera_view_polygon(simv, float(fov_deg)) if float(fov_deg) > 0.0 else None
        if float(fov_deg) > 0.0 and not polygon:
            return []
        for roi in self._roi_mocks:
            if aircraft_id > 0 and aircraft_id in roi.discovered_by:
                continue
            if not self._point_in_camera_view_with_polygon(
                simv,
                x=float(roi.x),
                y=float(roi.y),
                z=float(roi.z),
                fov_deg=float(fov_deg),
                polygon=polygon,
            ):
                continue
            dx = float(roi.x - s.x)
            dy = float(roi.y - s.y)
            dz = float(roi.z - s.z)
            visible.append((math.sqrt(dx * dx + dy * dy + dz * dz), int(roi.id), roi))
        visible.sort(key=lambda item: (item[0], item[1]))
        return [roi for _dist, _id, roi in visible]

    def _start_roi_focus(self, simv: SimVehicle, roi: RoiMock, fov_deg: float) -> None:
        duration = max(0.0, float(self.roi_gaze_duration_s))
        try:
            aircraft_id = int(simv.aircraft_id)
        except Exception:
            aircraft_id = 0
        if aircraft_id > 0:
            roi.discovered_by.add(int(aircraft_id))
        self._record_0402_roi_mock(simv, roi, fov_deg)
        if duration <= 0.0:
            return
        self._roi_focus_state[str(simv.label)] = RoiFocusState(
            roi_id=int(roi.id),
            roi=roi,
            fov_deg=float(fov_deg),
            start_time=float(self.sim_time),
            end_time=float(self.sim_time) + duration,
        )

    def _record_0402_target_list(self, simv: SimVehicle, target: GroundTarget, fov_deg: float) -> None:
        geo = self.geo
        if geo is None:
            return
        try:
            aircraft_id = int(simv.aircraft_id)
        except Exception:
            aircraft_id = 0
        try:
            raw_id = int(getattr(target, "id", 0))
        except Exception:
            raw_id = 0
        if raw_id <= 0:
            try:
                raw_id = self.targets.index(target) + 1
            except Exception:
                raw_id = 0
        key = (aircraft_id, raw_id)
        if key in self._reported_0402_list:
            return
        try:
            lon, lat = geo.xy_to_lonlat(float(target.x), float(target.y))
        except Exception:
            return
        coord = {"latitude": float(lat), "longitude": float(lon), "altitude": float(target.z)}
        try:
            target_type = int(getattr(target, "type_id", 0) or 0)
        except Exception:
            target_type = 0
        assigned_id = self._assign_0402_target_id(target)
        body = {
            "timestamp": self._sim_timestamp_ms_2000(),
            "source": "SIM",
            "roiInfo": {
                "aircraftID": aircraft_id,
                "coordinate": coord,
                "fov": float(fov_deg),
            },
            "targetList": [
                {
                    "targetID": assigned_id,
                    "targetType": target_type,
                    "coordinate": coord,
                    "watcher": {"aircraftID": aircraft_id},
                    "targetInFrame": 1,
                    "isDestroyed": 0,
                    "threat": 100.0,
                }
            ],
        }
        if raw_id > 0:
            self._target_watcher_0402[raw_id] = int(aircraft_id)
        self._emit_0402(body=body, aircraft_id=aircraft_id, target_id=assigned_id)
        self._reported_0402_list.add(key)

    def _record_0402_target_destroyed(self, target: GroundTarget, watcher_id: int | None = None) -> None:
        geo = self.geo
        if geo is None:
            return
        try:
            target_id = int(getattr(target, "id", 0))
        except Exception:
            return
        if target_id <= 0:
            return
        try:
            target_type = int(getattr(target, "type_id", 0) or 0)
        except Exception:
            target_type = 0
        if watcher_id is None:
            owner = self._tracking_target_owner.get(target_id)
            if owner:
                watcher_id = _label_to_aircraft_id(owner)
        try:
            lon, lat = geo.xy_to_lonlat(float(target.x), float(target.y))
            coord = {"latitude": float(lat), "longitude": float(lon), "altitude": float(target.z)}
        except Exception:
            return
        assigned_id = self._assign_0402_target_id(target)
        watcher_payload = {"aircraftID": int(watcher_id)} if watcher_id is not None else None
        target_entry: dict[str, object] = {
            "targetID": int(assigned_id),
            "targetType": int(target_type),
            "coordinate": coord,
            "targetInFrame": 0,
            "isDestroyed": 1,
            "threat": 0.0,
        }
        if watcher_payload is not None:
            target_entry["watcher"] = watcher_payload
        body = {
            "timestamp": self._sim_timestamp_ms_2000(),
            "source": "SIM",
            "targetList": [target_entry],
        }
        if watcher_id is not None:
            self._target_watcher_0402[target_id] = int(watcher_id)
        self._emit_0402(
            body=body,
            aircraft_id=int(watcher_id) if watcher_id is not None else 0,
            target_id=int(assigned_id),
        )
        self._reported_0402_destroyed.add(target_id)

    def _update_target_info_destroyed(self, target: GroundTarget, watcher_id: int | None = None) -> None:
        try:
            raw_target_id = int(getattr(target, "id", 0))
        except Exception:
            return
        if raw_target_id <= 0:
            return
        assigned_target_id = int(self._assign_0402_target_id(target) or 0)
        target_info_id = assigned_target_id if assigned_target_id > 0 else raw_target_id
        related_ids = self._related_target_ids_in_info(raw_target_id) or {raw_target_id}
        if assigned_target_id > 0:
            related_ids.add(int(assigned_target_id))
        related_ids.add(int(raw_target_id))
        path = db_paths.get_db_subpath("DSS_Internal", "targetInfo.json")
        data: dict[str, Any] = {}
        if path.exists():
            try:
                raw = path.read_text(encoding="utf-8")
                data = json.loads(raw) if raw else {}
            except Exception:
                data = {}
        target_list = data.get("targetList")
        if not isinstance(target_list, dict):
            target_list = {}
        updated = False
        normalized_target_list = dict(target_list)
        for key, entry in list(target_list.items()):
            if not isinstance(entry, dict):
                continue
            try:
                entry_id = int(entry.get("targetID", -1))
            except Exception:
                continue
            if entry_id not in related_ids:
                continue
            merged_entry = dict(entry)
            merged_entry["targetID"] = int(target_info_id)
            merged_entry["isDestroyed"] = True
            merged_entry["threat"] = 0
            merged_entry["targetInFrame"] = False
            final_watcher_id = _coerce_int(merged_entry.get("watcherID"), 0)
            if watcher_id is not None and final_watcher_id <= 0:
                final_watcher_id = int(watcher_id)
                merged_entry["watcherID"] = int(watcher_id)

            desired_key = (
                f"{int(target_info_id)}-{int(final_watcher_id)}"
                if final_watcher_id > 0
                else str(int(target_info_id))
            )
            existing = normalized_target_list.get(desired_key)
            if isinstance(existing, dict):
                combined = dict(existing)
                combined.update({k: v for k, v in merged_entry.items() if v is not None})
                normalized_target_list[desired_key] = combined
            else:
                normalized_target_list[desired_key] = merged_entry
            if desired_key != str(key):
                normalized_target_list.pop(str(key), None)
            updated = True
        target_list = normalized_target_list
        if not updated:
            coord = None
            if self.geo is not None:
                try:
                    lon, lat = self.geo.xy_to_lonlat(float(target.x), float(target.y))
                    coord = {"latitude": float(lat), "longitude": float(lon), "altitude": float(target.z)}
                except Exception:
                    coord = None
            entry: dict[str, object] = {
                "targetID": int(target_info_id),
                "targetType": int(getattr(target, "type_id", 0) or 0),
                "isDestroyed": True,
                "targetInFrame": False,
                "threat": 0.0,
                "isUsed": 0,
                "isIgnored": 0,
            }
            if coord is not None:
                entry["coordinate"] = coord
            if watcher_id is not None:
                entry["watcherID"] = int(watcher_id)
            key = (
                f"{int(target_info_id)}-{int(watcher_id)}"
                if watcher_id is not None
                else str(int(target_info_id))
            )
            target_list[key] = entry
            updated = True
        if updated:
            data["targetList"] = target_list
            path.parent.mkdir(parents=True, exist_ok=True)
            try:
                path.write_text(
                    json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True),
                    encoding="utf-8",
                )
            except Exception:
                pass

    def _stop_tracking_for_destroyed_target_ids(self, target_ids: set[int]) -> None:
        normalized_ids = {
            int(value)
            for value in (target_ids or set())
            if _coerce_int(value, 0) > 0
        }
        if not normalized_ids:
            return

        for label, preview in list(self._tracking_preview_state.items()):
            preview_ids: set[int] = set()
            for value in getattr(preview, "target_ids", []) or []:
                try:
                    candidate = int(value)
                except Exception:
                    continue
                if candidate > 0:
                    preview_ids.add(candidate)
            try:
                chosen_id = int(getattr(preview, "chosen_target_id", 0) or 0)
            except Exception:
                chosen_id = 0
            if chosen_id > 0:
                preview_ids.add(chosen_id)
            if preview_ids & normalized_ids:
                self._clear_tracking_preview(str(label))

        for label, override in list(self._tracking_overrides.items()):
            if not isinstance(override, dict):
                continue
            try:
                override_target_id = int(override.get("target_id") or 0)
            except Exception:
                override_target_id = 0
            if override_target_id in normalized_ids:
                self._tracking_overrides.pop(str(label), None)

        for label, state in list(self._tracking_state.items()):
            related_state_ids: set[int] = set()
            try:
                state_target_id = int(getattr(state, "target_id", 0) or 0)
            except Exception:
                state_target_id = 0
            if state_target_id > 0:
                related_state_ids.add(state_target_id)
            try:
                raw_target_id = int(getattr(getattr(state, "target", None), "id", 0) or 0)
            except Exception:
                raw_target_id = 0
            if raw_target_id > 0:
                related_state_ids.add(raw_target_id)
                try:
                    mapped_target_id = int(self._target_id_map_0402.get(raw_target_id) or 0)
                except Exception:
                    mapped_target_id = 0
                if mapped_target_id > 0:
                    related_state_ids.add(mapped_target_id)
            if not (related_state_ids & normalized_ids):
                continue
            simv = self.vehicles.get(str(label))
            self._tracking_overrides.pop(str(label), None)
            self._clear_tracking_preview(str(label))
            if simv is None:
                self._tracking_state.pop(str(label), None)
                continue
            self._stop_tracking(simv, advance=True)

        for tracked_id in list(self._tracking_target_owner.keys()):
            try:
                tracked_id_int = int(tracked_id)
            except Exception:
                continue
            if tracked_id_int in normalized_ids:
                self._tracking_target_owner.pop(tracked_id_int, None)

    def _handle_target_destroyed(self, target: GroundTarget, watcher_id: int | None = None) -> None:
        try:
            target_id = int(getattr(target, "id", 0))
        except Exception:
            return
        if target_id <= 0:
            return
        self._clear_friendly_attack_attempts_for_target(target_id)
        if target_id in self._destroyed_target_ids:
            return
        self._destroyed_target_ids.add(target_id)
        related_ids = self._related_target_ids_in_info(target_id) or {target_id}
        for related_id in related_ids:
            self._destroyed_target_ids.add(related_id)
            virtual = self._virtual_targets.get(int(related_id))
            if virtual is not None:
                virtual.alive = False
        # 원본 GroundTarget도 alive=False 처리
        target.alive = False
        self._stop_tracking_for_destroyed_target_ids(
            {int(target_id)} | {int(value) for value in related_ids if _coerce_int(value, 0) > 0}
        )
        self._update_target_info_destroyed(target, watcher_id)
        self._record_0402_target_destroyed(target, watcher_id)

    def _start_tracking(
        self,
        simv: SimVehicle,
        target_id: int,
        target: GroundTarget,
        filming_prop: dict | None,
        fov_deg: float,
        *,
        loiter_prop: dict | None = None,
        duration: float | None = None,
        manual: bool = False,
        advance_on_complete: bool = False,
    ) -> None:
        self._clear_tracking_preview(simv.label)
        if simv.label in self._tracking_state:
            return
        owner = self._tracking_target_owner.get(int(target.id))
        if owner and owner != simv.label:
            return
        try:
            raw_target_id = int(getattr(target, "id", 0))
            aircraft_id = int(simv.aircraft_id)
        except Exception:
            raw_target_id = 0
            aircraft_id = 0
        if raw_target_id > 0 and aircraft_id > 0:
            self._target_watcher_0402[raw_target_id] = aircraft_id
        tracking_film = filming_prop if isinstance(filming_prop, dict) else {}
        tracking_film = dict(tracking_film)
        tracking_film.setdefault("operationMode", 3)
        tracking_film["fieldOfView"] = float(fov_deg)
        if manual:
            try:
                raw_id = int(getattr(target, "id", 0))
            except Exception:
                raw_id = 0
            if raw_id > 0:
                self._target_id_map_0402[raw_id] = int(target_id)
                if int(self._target_id_seq_0402) <= int(target_id):
                    self._target_id_seq_0402 = int(target_id) + 1
        saved_controller = simv.controller
        saved_wp_id = None
        try:
            saved_target = saved_controller.current_target()
            if saved_target is not None and saved_target.wp_id is not None:
                saved_wp_id = int(saved_target.wp_id)
        except Exception:
            saved_wp_id = None
        radius = max(50.0, float(self.track_loiter_radius_m))
        speed = float(self.track_loiter_speed_mps)
        if speed <= 0.0:
            speed = float(getattr(saved_controller, "speed_target", self.speed_uav))
        source_loiter = loiter_prop if isinstance(loiter_prop, dict) else None
        if source_loiter is not None:
            try:
                lp_radius = float(source_loiter.get("radius", radius) or radius)
            except Exception:
                lp_radius = radius
            if lp_radius > 1.0:
                radius = max(50.0, lp_radius)
            try:
                lp_speed = float(source_loiter.get("speed", speed) or speed)
            except Exception:
                lp_speed = speed
            if lp_speed > 0.0:
                speed = lp_speed
        track_end = None
        if duration is not None:
            try:
                dur_val = float(duration)
            except Exception:
                dur_val = 0.0
            if dur_val > 0.0:
                track_end = float(self.sim_time) + dur_val
        alt = max(float(simv.vehicle.s.z), float(target.z) + float(self.track_alt_buffer_m))
        tracking_loiter = {"time": 1e9, "radius": radius, "speed": speed, "direction": 1}
        loiter_wp = WaypointTarget(
            pos=(float(target.x), float(target.y), float(alt)),
            speed=float(speed),
            loiter=tracking_loiter,
            filming=tracking_film,
        )
        tracking_controller = WaypointPIDController(
            simv.vehicle,
            [loiter_wp],
            gains=saved_controller.gains,
            speed_target=float(speed),
            pos_tol=float(self.pos_tol),
            name=f"{simv.label}-track",
            allow_hover=False,
        )
        tracking_controller.is_loitering = True
        tracking_controller.loiter_timer = math.inf
        tracking_controller.loiter_center = (float(target.x), float(target.y), float(alt))
        tracking_controller.loiter_radius = radius
        tracking_controller.loiter_speed = float(speed)
        direction = 1
        if source_loiter is not None:
            try:
                direction = int(source_loiter.get("direction", 1) or 1)
            except Exception:
                direction = 1
        tracking_controller.loiter_dir = -1.0 if direction == 1 else 1.0
        tracking_controller.loiter_angle = math.atan2(simv.vehicle.s.y - target.y, simv.vehicle.s.x - target.x)
        simv.controller = tracking_controller
        self._tracking_state[simv.label] = TrackingState(
            target_id=int(target_id),
            target=target,
            saved_controller=saved_controller,
            saved_wp_id=saved_wp_id,
            tracking_controller=tracking_controller,
            loiter_wp=loiter_wp,
            fov_deg=float(fov_deg),
            stage=1 if manual else 0,
            start_step=int(self.step_count),
            last_seen=float(self.sim_time),
            filming_prop=tracking_film,
            end_time=track_end,
            advance_on_complete=bool(advance_on_complete),
            manual=bool(manual),
            track_radius=radius,
            track_speed=float(speed),
        )
        self._tracking_target_owner[int(target.id)] = simv.label
        self._record_0402_roi(simv, target, fov_deg)

    def _stop_tracking(self, simv: SimVehicle, *, advance: bool = False) -> None:
        state = self._tracking_state.pop(simv.label, None)
        if state is None:
            return
        try:
            owner = self._tracking_target_owner.get(int(state.target.id))
            if owner == simv.label:
                del self._tracking_target_owner[int(state.target.id)]
        except Exception:
            pass
        if advance:
            if simv.airframe == "uav":
                self._pulse_on_mission(simv.label)
                self._pulse_filming(simv.label)
            try:
                current = state.saved_controller.current_target()
                if current is not None and state.saved_wp_id is not None:
                    if current.wp_id == state.saved_wp_id:
                        state.saved_controller._advance_wp()
                elif current is not None and state.saved_wp_id is None:
                    state.saved_controller._advance_wp()
            except Exception:
                pass
        simv.controller = state.saved_controller

    def _update_tracking_center(self, state: TrackingState, simv: SimVehicle) -> None:
        target = state.target
        if not target.alive:
            return
        alt = max(float(simv.vehicle.s.z), float(target.z) + float(self.track_alt_buffer_m))
        center = (float(target.x), float(target.y), float(alt))
        state.loiter_wp.pos = center
        ctrl = state.tracking_controller
        ctrl.loiter_center = center
        if state.track_radius is not None:
            ctrl.loiter_radius = max(50.0, float(state.track_radius))
        else:
            ctrl.loiter_radius = max(50.0, float(self.track_loiter_radius_m))
        speed = float(self.track_loiter_speed_mps)
        if speed <= 0.0:
            speed = float(getattr(state.saved_controller, "speed_target", self.speed_uav))
        if state.track_speed is not None and float(state.track_speed) > 0.0:
            ctrl.loiter_speed = float(state.track_speed)
        else:
            ctrl.loiter_speed = speed

    def _apply_tracking_override(
        self,
        simv: SimVehicle,
        state: TrackingState | None,
        override: dict[str, Any],
    ) -> bool:
        target_id = override.get("target_id")
        try:
            target_id = int(target_id)
        except Exception:
            return False
        if target_id <= 0:
            return False
        if self._target_is_ignored_in_info(target_id) or self._target_is_destroyed_in_info(target_id):
            self._tracking_overrides.pop(str(simv.label), None)
            return False
        target = self._resolve_tracking_target(target_id)
        if target is None or not target.alive:
            return False
        filming_prop = override.get("filming") if isinstance(override.get("filming"), dict) else {}
        loiter_prop = override.get("loiter") if isinstance(override.get("loiter"), dict) else None
        duration = override.get("duration")
        fov_deg = float(override.get("fov_deg") or self._resolve_tracking_fov(filming_prop))
        if state is None:
            self._start_tracking(
                simv,
                target_id,
                target,
                filming_prop,
                fov_deg,
                loiter_prop=loiter_prop,
                duration=duration if isinstance(duration, (int, float)) else None,
                manual=True,
                advance_on_complete=True,
            )
            new_state = self._tracking_state.get(simv.label)
            if new_state is not None:
                new_state.override_seq = override.get("seq")
            return new_state is not None

        # Update existing tracking with new parameters.
        try:
            old_target_id = int(state.target.id)
        except Exception:
            old_target_id = None
        if old_target_id is not None and old_target_id != int(target.id):
            try:
                owner = self._tracking_target_owner.get(old_target_id)
                if owner == simv.label:
                    del self._tracking_target_owner[old_target_id]
            except Exception:
                pass
        self._tracking_target_owner[int(target.id)] = simv.label
        state.target = target
        state.target_id = int(target_id)
        state.manual = True
        state.stage = 1
        state.fov_deg = float(fov_deg)
        state.advance_on_complete = True
        new_prop = dict(filming_prop or {})
        new_prop.setdefault("operationMode", 3)
        new_prop["fieldOfView"] = float(fov_deg)
        state.filming_prop = new_prop
        try:
            state.loiter_wp.filming = new_prop
        except Exception:
            pass

        radius = state.track_radius if state.track_radius is not None else self.track_loiter_radius_m
        speed = state.track_speed if state.track_speed is not None else self.track_loiter_speed_mps
        direction = None
        if loiter_prop is not None:
            try:
                radius = float(loiter_prop.get("radius", radius) or radius)
            except Exception:
                pass
            try:
                speed = float(loiter_prop.get("speed", speed) or speed)
            except Exception:
                pass
            try:
                direction = int(loiter_prop.get("direction", 1) or 1)
            except Exception:
                direction = 1
        radius = max(50.0, float(radius))
        if speed <= 0.0:
            speed = float(getattr(state.saved_controller, "speed_target", self.speed_uav))
        state.track_radius = radius
        state.track_speed = float(speed)
        ctrl = state.tracking_controller
        ctrl.loiter_radius = float(radius)
        ctrl.loiter_speed = float(speed)
        if direction is not None:
            ctrl.loiter_dir = -1.0 if int(direction) == 1 else 1.0

        if isinstance(duration, (int, float)) and float(duration) > 0.0:
            state.end_time = float(self.sim_time) + float(duration)
        else:
            state.end_time = None
        state.override_seq = override.get("seq")
        self._update_tracking_center(state, simv)
        return True

    def _update_roi_mock_focus(self, simv: SimVehicle) -> None:
        if simv.airframe != "uav":
            return
        label = str(simv.label)
        if self._roi_focus_active(label) is not None:
            return
        if label in self._tracking_state:
            return
        try:
            current = simv.controller.current_target()
        except Exception:
            current = None
        filming_prop = current.filming if current else None
        if not self._has_active_filming_prop(filming_prop):
            return
        fov_deg = self._resolve_tracking_fov(filming_prop if isinstance(filming_prop, dict) else None)
        visible = self._visible_roi_mocks(simv, fov_deg)
        if not visible:
            return
        self._start_roi_focus(simv, visible[0], fov_deg)

    def _update_tracking(self, simv: SimVehicle, dt: float) -> None:
        label = simv.label
        state = self._tracking_state.get(label)
        override = self._tracking_overrides.get(label)
        if override and (state is None or override.get("seq") != state.override_seq):
            if self._apply_tracking_override(simv, state, override):
                return
        if state is not None:
            if self._target_is_ignored_in_info(int(state.target_id)):
                self._tracking_overrides.pop(str(label), None)
                self._stop_tracking(simv, advance=True)
                return
            if self._target_is_destroyed_in_info(int(state.target_id)):
                self._tracking_overrides.pop(str(label), None)
                self._stop_tracking(simv, advance=state.advance_on_complete)
                return
            if not state.target.alive:
                self._stop_tracking(simv, advance=state.advance_on_complete)
                return
            if state.end_time is not None and float(self.sim_time) >= float(state.end_time):
                self._stop_tracking(simv, advance=state.advance_on_complete)
                return
            # Keep tracking once acquired; do not drop due to transient FOV/range loss.
            state.last_seen = float(self.sim_time)
            if not state.manual:
                if state.stage == 0 and int(self.step_count) > int(state.start_step):
                    state.stage = 1
                    state.fov_deg = 2.0
                    new_prop = dict(state.filming_prop or {})
                    new_prop.setdefault("operationMode", 3)
                    new_prop["fieldOfView"] = float(state.fov_deg)
                    state.filming_prop = new_prop
                    try:
                        state.loiter_wp.filming = new_prop
                    except Exception:
                        pass
                    self._record_0402_target_list(simv, state.target, state.fov_deg)
            self._update_tracking_center(state, simv)
            return

        if simv.airframe != "uav":
            return
        if self._roi_focus_active(label) is not None:
            return
        current = simv.controller.current_target()
        filming_prop = current.filming if current else None
        if not self._has_active_filming_prop(filming_prop):
            self._clear_tracking_preview(label)
            return
        if isinstance(filming_prop, dict):
            op_mode = filming_prop.get("operationMode") or filming_prop.get("operationalMode")
            try:
                op_mode = int(op_mode or 0)
            except Exception:
                op_mode = 0
            auto = filming_prop.get("autoTracking") or filming_prop.get("AutoTracking")
            if isinstance(auto, dict) and op_mode == 3 and current is not None:
                target_id = auto.get("targetID") or auto.get("TargetID") or auto.get("targetId")
                try:
                    target_id = int(target_id)
                except Exception:
                    target_id = 0
                if target_id > 0:
                    target = self._resolve_tracking_target(target_id)
                    if target is not None and target.alive:
                        loiter_prop = current.loiter if isinstance(current.loiter, dict) else None
                        duration = None
                        if loiter_prop is not None:
                            try:
                                duration = float(loiter_prop.get("time") or 0.0)
                            except Exception:
                                duration = None
                            if duration is not None and duration <= 0.0:
                                duration = None
                        fov_deg = self._resolve_tracking_fov(filming_prop)
                        self._start_tracking(
                            simv,
                            target_id,
                            target,
                            filming_prop,
                            fov_deg,
                            loiter_prop=loiter_prop,
                            duration=duration,
                            manual=True,
                            advance_on_complete=True,
                        )
                        return
        preview = self._tracking_preview_state.get(label)
        if preview is not None:
            preview_targets = self._visible_tracking_targets(
                simv,
                float(preview.fov_deg),
                candidate_ids={int(value) for value in (preview.target_ids or []) if int(value) > 0},
            )
            if float(self.sim_time) < float(preview.end_time):
                if preview_targets:
                    return
                self._clear_tracking_preview(label)
            else:
                self._clear_tracking_preview(label)
                chosen = None
                for target in preview_targets:
                    try:
                        raw_id = int(getattr(target, "id", 0))
                    except Exception:
                        raw_id = 0
                    if raw_id == int(preview.chosen_target_id):
                        chosen = target
                        break
                if chosen is None and preview_targets:
                    chosen = preview_targets[0]
                if chosen is not None:
                    try:
                        chosen_id = int(getattr(chosen, "id", 0))
                    except Exception:
                        chosen_id = 0
                    if chosen_id > 0:
                        self._start_tracking(
                            simv,
                            chosen_id,
                            chosen,
                            preview.filming_prop,
                            float(preview.fov_deg),
                        )
                        return
        if not self.auto_track_always:
            if not (isinstance(filming_prop, dict) and int(filming_prop.get("operationMode") or 0) == 3):
                return
        fov_deg = self._resolve_tracking_fov(filming_prop if isinstance(filming_prop, dict) else None)
        visible_targets = self._visible_tracking_targets(simv, fov_deg)
        if not visible_targets:
            return
        if not self.auto_track_takeover:
            self._clear_tracking_preview(label)
            self._record_0402_target_list(simv, visible_targets[0], fov_deg)
            return
        if len(visible_targets) >= 2 and self._start_tracking_preview(simv, visible_targets, filming_prop, fov_deg):
            return
        target = visible_targets[0]
        try:
            target_id = int(getattr(target, "id", 0))
        except Exception:
            target_id = 0
        if target_id <= 0:
            return
        self._start_tracking(simv, target_id, target, filming_prop, fov_deg)

    def _formation_leader(self, simv: SimVehicle) -> SimVehicle | None:
        spec = simv.formation
        if spec is None:
            return None
        leader_label = _agent_label(int(spec.leader_id))
        if not leader_label:
            return None
        return self.vehicles.get(leader_label)

    def _active_formation_spec(self, simv: SimVehicle) -> FormationSpec | None:
        try:
            target = simv.controller.current_target()
        except Exception:
            target = None
        spec = None
        if target is not None:
            path_id = _coerce_int(getattr(target, "path_id", None), None)
            if path_id is not None:
                spec = self._formation_by_path_id.get(int(path_id))
        simv.formation = spec
        return spec

    def _formation_segment_indices(
        self,
        controller: WaypointPIDController,
        *,
        input_id: int | None = None,
        path_id: int | None = None,
    ) -> list[int]:
        targets = getattr(controller, "targets", None)
        if not isinstance(targets, list) or not targets:
            return []
        if input_id is not None:
            indices = [
                idx
                for idx, target in enumerate(targets)
                if _coerce_int(getattr(target, "input_mission_id", None), None) == int(input_id)
            ]
            if indices:
                return indices
        if path_id is not None:
            return [
                idx
                for idx, target in enumerate(targets)
                if _coerce_int(getattr(target, "path_id", None), None) == int(path_id)
            ]
        return []

    def _align_formation_follower_progress(self, simv: SimVehicle, leader: SimVehicle) -> None:
        follower_ctrl = simv.controller
        leader_ctrl = leader.controller
        try:
            follower_target = follower_ctrl.current_target()
            leader_target = leader_ctrl.current_target()
        except Exception:
            return
        if follower_target is None or leader_target is None:
            return

        follower_input = _coerce_int(getattr(follower_target, "input_mission_id", None), None)
        leader_input = _coerce_int(getattr(leader_target, "input_mission_id", None), None)
        common_input = leader_input if leader_input is not None and leader_input == follower_input else None
        follower_path = _coerce_int(getattr(follower_target, "path_id", None), None)
        leader_path = _coerce_int(getattr(leader_target, "path_id", None), None)

        leader_indices = self._formation_segment_indices(
            leader_ctrl,
            input_id=common_input,
            path_id=leader_path,
        )
        follower_indices = self._formation_segment_indices(
            follower_ctrl,
            input_id=common_input,
            path_id=follower_path,
        )
        if not leader_indices or not follower_indices:
            return

        leader_idx = int(getattr(leader_ctrl, "curr_idx", leader_indices[0]))
        leader_offset = max(0, min(len(leader_indices) - 1, leader_idx - leader_indices[0]))
        desired_idx = follower_indices[min(leader_offset, len(follower_indices) - 1)]
        if int(getattr(follower_ctrl, "curr_idx", desired_idx)) != int(desired_idx):
            follower_ctrl.curr_idx = int(desired_idx)
            follower_ctrl._closest_wp_idx = int(desired_idx)
            follower_ctrl._closest_wp_dist_xy = float("inf")
            follower_ctrl.just_advanced = True
            follower_ctrl.advance_reason = "formation"

        follower_ctrl.finished = bool(getattr(leader_ctrl, "finished", False))
        follower_ctrl.blocked = bool(getattr(leader_ctrl, "blocked", False))
        follower_ctrl.blocked_input_id = getattr(leader_ctrl, "blocked_input_id", None)
        follower_ctrl.is_loitering = bool(getattr(leader_ctrl, "is_loitering", False))
        follower_ctrl.is_hovering = bool(getattr(leader_ctrl, "is_hovering", False))
        follower_ctrl.loiter_timer = float(getattr(leader_ctrl, "loiter_timer", 0.0))
        follower_ctrl.hover_timer = float(getattr(leader_ctrl, "hover_timer", 0.0))

    def _formation_offset_position(
        self,
        leader: SimVehicle,
        spec: FormationSpec,
    ) -> tuple[float, float, float]:
        s = leader.vehicle.s
        yaw = math.radians(float(getattr(s, "yaw", 0.0)))
        fwd_x = math.cos(yaw)
        fwd_y = -math.sin(yaw)
        right_x = -math.sin(yaw)
        right_y = -math.cos(yaw)
        tx = float(s.x) + (fwd_x * float(spec.dx)) + (right_x * float(spec.dy))
        ty = float(s.y) + (fwd_y * float(spec.dx)) + (right_y * float(spec.dy))
        tz = float(s.z) + float(spec.dz)
        return float(tx), float(ty), float(tz)

    def _sync_formation_follower_to_leader(self, simv: SimVehicle) -> bool:
        spec = self._active_formation_spec(simv)
        if spec is None or spec.is_leader:
            return False
        leader = self._formation_leader(simv)
        if leader is None or not leader.alive:
            return False
        leader_spec = self._active_formation_spec(leader)
        if leader_spec is None or int(leader_spec.leader_id) != int(spec.leader_id):
            return False

        self._align_formation_follower_progress(simv, leader)
        tx, ty, tz = self._formation_offset_position(leader, spec)
        fs = simv.vehicle.s
        ls = leader.vehicle.s
        fs.x = float(tx)
        fs.y = float(ty)
        fs.z = float(tz)
        for attr in ("yaw", "pitch", "roll", "u", "p", "q", "r"):
            try:
                setattr(fs, attr, float(getattr(ls, attr)))
            except Exception:
                pass
        try:
            simv.vehicle.cmd_yaw_rate = float(getattr(leader.vehicle, "cmd_yaw_rate", 0.0))
            simv.vehicle.cmd_pitch_rate = float(getattr(leader.vehicle, "cmd_pitch_rate", 0.0))
            simv.vehicle.cmd_roll_rate = float(getattr(leader.vehicle, "cmd_roll_rate", 0.0))
            simv.vehicle.cmd_throttle = float(getattr(leader.vehicle, "cmd_throttle", 0.0))
        except Exception:
            pass

        target = simv.controller.current_target()
        if target is not None:
            target.pos = (float(tx), float(ty), float(tz))
        if getattr(simv.controller, "is_loitering", False):
            simv.controller.loiter_center = (float(tx), float(ty), float(tz))
        return True

    def _is_formation_leader(self, simv: SimVehicle) -> bool:
        try:
            leader_id = int(simv.aircraft_id)
        except Exception:
            return False
        for other in self.vehicles.values():
            spec = self._active_formation_spec(other)
            if spec is None or spec.is_leader:
                continue
            try:
                if int(spec.leader_id) == leader_id:
                    return True
            except Exception:
                continue
        return False

    def _update_formation_target(self, simv: SimVehicle) -> None:
        spec = self._active_formation_spec(simv)
        if spec is None or spec.is_leader:
            return
        leader = self._formation_leader(simv)
        if leader is None:
            return
        s = leader.vehicle.s
        yaw = math.radians(float(getattr(s, "yaw", 0.0)))
        fwd_x = math.cos(yaw)
        fwd_y = -math.sin(yaw)
        right_x = -math.sin(yaw)
        right_y = -math.cos(yaw)
        dx = float(spec.dx)
        dy = float(spec.dy)
        dz = float(spec.dz)
        tx = float(s.x) + fwd_x * dx + right_x * dy
        ty = float(s.y) + fwd_y * dx + right_y * dy
        tz = float(s.z) + dz
        target = simv.formation_target
        if target is None:
            target = simv.controller.current_target()
        if target is None:
            return
        target.pos = (tx, ty, tz)
        try:
            target.speed = float(getattr(s, "u", 0.0))
        except Exception:
            pass
        leader_wp = None
        try:
            leader_wp = leader.controller.current_target()
        except Exception:
            leader_wp = None
        if leader_wp is not None and getattr(leader_wp, "filming", None) is not None:
            target.filming = leader_wp.filming
        else:
            target.filming = None
        try:
            simv.controller.speed_target = float(getattr(s, "u", simv.controller.speed_target))
        except Exception:
            pass

    def _update_filming_target(self, simv: SimVehicle, dt: float) -> None:
        geo = self.geo
        if geo is None:
            return
        label = simv.label
        roi_focus = self._roi_focus_active(label)
        if roi_focus is not None:
            controller = simv.controller
            try:
                tgt = controller.current_target()
            except Exception:
                tgt = None
            filming_prop = tgt.filming if tgt else None
            current_wp_id = tgt.wp_id if tgt else None
            current_max_sep_m = None
            try:
                if tgt is not None and getattr(tgt, "sep_m", None) is not None:
                    current_max_sep_m = float(getattr(tgt, "sep_m"))
            except Exception:
                current_max_sep_m = None
            roi_filming = dict(filming_prop) if isinstance(filming_prop, dict) else {}
            roi_filming["operationMode"] = 1
            roi_filming["fieldOfView"] = float(roi_focus.fov_deg)
            roi_filming["roiMock"] = True
            self._filming_props[label] = roi_filming
            self._filming_wp_ids[label] = current_wp_id
            self._filming_max_sep_m[label] = current_max_sep_m
            self._filming_targets[label] = (
                float(roi_focus.roi.x),
                float(roi_focus.roi.y),
                float(roi_focus.roi.z),
            )
            self._clear_footprint_line_right_axis(label)
            return
        tracking = self._tracking_state.get(label)
        if tracking is not None and tracking.target.alive:
            self._line_search_state[label] = None
            self._filming_props[label] = tracking.filming_prop
            self._filming_wp_ids[label] = tracking.saved_wp_id
            self._filming_max_sep_m[label] = None
            self._filming_targets[label] = (
                float(tracking.target.x),
                float(tracking.target.y),
                float(tracking.target.z),
            )
            self._clear_footprint_line_right_axis(label)
            return
        controller = simv.controller
        tgt = controller.current_target()
        filming_prop = tgt.filming if tgt else None
        current_wp_id = tgt.wp_id if tgt else None
        current_max_sep_m = None
        try:
            if tgt is not None and getattr(tgt, "sep_m", None) is not None:
                current_max_sep_m = float(getattr(tgt, "sep_m"))
        except Exception:
            current_max_sep_m = None

        if not self._has_active_filming_prop(filming_prop):
            self._line_search_state[label] = None
            self._filming_props[label] = None
            self._filming_wp_ids[label] = current_wp_id
            self._filming_max_sep_m[label] = current_max_sep_m
            self._filming_targets[label] = None
            self._clear_footprint_line_right_axis(label)
            return

        handler = self._get_operation_handler(filming_prop.get("operationMode"))
        if handler is None:
            self._line_search_state[label] = None
            self._filming_props[label] = filming_prop
            self._filming_wp_ids[label] = current_wp_id
            self._filming_max_sep_m[label] = current_max_sep_m
            self._filming_targets[label] = self._default_downward_target(simv.vehicle)
            self._clear_footprint_line_right_axis(label)
            return

        ctx = OperationContext(
            geo=geo,
            ground_height_fn=self._ground_height,
            default_target_fn=self._default_downward_target,
        )
        prev_state = self._line_search_state.get(label)
        try:
            result = handler.apply(
                uav=simv.vehicle,
                filming_prop=filming_prop,
                ctx=ctx,
                dt=float(dt or 0.0),
                current_wp_id=current_wp_id,
                prev_state=prev_state,
            )
        except Exception:
            self._line_search_state[label] = None
            self._filming_props[label] = filming_prop
            self._filming_wp_ids[label] = current_wp_id
            self._filming_max_sep_m[label] = current_max_sep_m
            self._filming_targets[label] = self._default_downward_target(simv.vehicle)
            self._clear_footprint_line_right_axis(label)
            return

        self._filming_props[label] = filming_prop
        self._filming_wp_ids[label] = current_wp_id
        self._filming_max_sep_m[label] = current_max_sep_m
        self._filming_targets[label] = result.target or self._default_downward_target(simv.vehicle)
        self._line_search_state[label] = result.state
        if result.reset_debug or result.debug is not None:
            self._line_search_debug[label] = result.debug
        if self._filming_operation_mode(filming_prop) == 2:
            self._update_footprint_line_right_axis(
                label,
                self._line_search_debug_target_right_axis(simv, result.debug),
                float(dt or 0.0),
            )
        else:
            self._clear_footprint_line_right_axis(label)
        try:
            if (
                getattr(simv.controller, "is_loitering", False)
                and isinstance(filming_prop, dict)
                and int(filming_prop.get("operationMode") or 0) == 1
                and result.target is not None
            ):
                center = result.target
                flight_alt = float("nan")
                current_center = getattr(simv.controller, "loiter_center", None)
                if isinstance(current_center, (list, tuple)) and len(current_center) >= 3:
                    flight_alt = _coerce_float(current_center[2], float("nan"))
                if not math.isfinite(flight_alt):
                    current_target = simv.controller.current_target()
                    target_pos = getattr(current_target, "pos", None) if current_target is not None else None
                    if isinstance(target_pos, (list, tuple)) and len(target_pos) >= 3:
                        flight_alt = _coerce_float(target_pos[2], float("nan"))
                if not math.isfinite(flight_alt):
                    flight_alt = float(simv.vehicle.s.z)
                simv.controller.loiter_center = (float(center[0]), float(center[1]), float(flight_alt))
                if not getattr(simv.controller, "loiter_angle_locked", False):
                    simv.controller.loiter_angle = math.atan2(
                        simv.vehicle.s.y - float(center[1]),
                        simv.vehicle.s.x - float(center[0]),
                    )
                    simv.controller.loiter_angle_locked = True
        except Exception:
            pass

        tracking = self._tracking_state.get(label)
        if tracking is not None:
            tgt = tracking.target
            alt = max(float(simv.vehicle.s.z), float(tgt.z) + float(self.track_alt_buffer_m))
            self._filming_targets[label] = (float(tgt.x), float(tgt.y), float(alt))
            if tracking.filming_prop is not None:
                self._filming_props[label] = tracking.filming_prop

    def _build_vehicles(
        self,
        paths: list[PathDefinition],
        *,
        reset_detection_state: bool = True,
    ) -> None:
        geo = self.geo
        if geo is None:
            return
        self._tracking_state = {}
        self._tracking_preview_state = {}
        self._roi_focus_state = {}
        if reset_detection_state:
            self._reset_0402_state()
        self._attack_holds = {}

        uav_db = Path(__file__).resolve().parent / "controllers" / "uav_pid_db.json"
        lah_db = Path(__file__).resolve().parent / "controllers" / "lah_pid_db.json"
        uav_dynamics_profile = Path(__file__).resolve().parent / "controllers" / "uav_dynamics_profile.json"

        vehicles: dict[str, SimVehicle] = {}
        self._filming_props = {}
        self._filming_targets = {}
        self._filming_wp_ids = {}
        self._filming_max_sep_m = {}
        self._filming_pulse = {}
        self._line_search_state = {}
        self._line_search_debug = {}
        self._footprint_line_right_axis = {}
        self._history.clear()
        self._last_history_record_sim_time = None
        spawn_by_aircraft = self._spawn_by_aircraft or {}
        for path in paths:
            wp_targets: list[WaypointTarget] = []
            for wp in path.waypoints:
                lat = wp.get("lat")
                lon = wp.get("lon")
                alt = wp.get("alt")
                if lat is None or lon is None:
                    continue
                x, y = geo.lonlat_to_xy(lon, lat)
                z = float(alt) if alt is not None else 0.0
                wp_targets.append(
                    WaypointTarget(
                        pos=(x, y, z),
                        speed=wp.get("speed"),
                        wp_id=wp.get("wp_id"),
                        is_done=bool(wp.get("is_done", False)),
                        hover_time=wp.get("hover_time"),
                        loiter=wp.get("loiter"),
                        filming=wp.get("filming"),
                        attack=wp.get("attack") if isinstance(wp.get("attack"), dict) else None,
                        input_mission_id=wp.get("input_mission_id"),
                        individual_mission_id=wp.get("individual_mission_id"),
                        path_id=wp.get("path_id") or path.path_id,
                        pass_type=wp.get("pass_type"),
                        sep_m=wp.get("sep_m"),
                    )
                )

            if len(wp_targets) < 2:
                continue

            if path.airframe == "lah":
                vehicle = LAH(LAHParams())
                speed_target = self.speed_lah
                gains = load_pid_gains_for_time_scale(lah_db, self.time_scale)
                allow_hover = True
            else:
                vehicle = UAV(load_uav_params_profile(uav_dynamics_profile, UAVParams()))
                speed_target = self.speed_uav
                gains = load_pid_gains_for_time_scale(uav_db, self.time_scale)
                allow_hover = False

            first = wp_targets[0].pos
            spawn = spawn_by_aircraft.get(path.aircraft_id)
            if spawn is not None:
                vehicle.s.x = float(spawn[0])
                vehicle.s.y = float(spawn[1])
                vehicle.s.z = float(spawn[2])
                dx = first[0] - vehicle.s.x
                dy = first[1] - vehicle.s.y
                if abs(dx) + abs(dy) > 1e-6:
                    vehicle.s.yaw = (math.degrees(math.atan2(-dy, dx)) + 360.0) % 360.0
                elif len(wp_targets) >= 2:
                    dx = wp_targets[1].pos[0] - first[0]
                    dy = wp_targets[1].pos[1] - first[1]
                    if abs(dx) + abs(dy) > 1e-6:
                        vehicle.s.yaw = (math.degrees(math.atan2(-dy, dx)) + 360.0) % 360.0
            else:
                vehicle.s.x = float(first[0])
                vehicle.s.y = float(first[1])
                vehicle.s.z = float(first[2])
                if len(wp_targets) >= 2:
                    dx = wp_targets[1].pos[0] - first[0]
                    dy = wp_targets[1].pos[1] - first[1]
                    if abs(dx) + abs(dy) > 1e-6:
                        vehicle.s.yaw = (math.degrees(math.atan2(-dy, dx)) + 360.0) % 360.0
            init_speed = wp_targets[0].speed if wp_targets[0].speed is not None else speed_target
            if init_speed is not None:
                try:
                    vehicle.s.u = float(init_speed)
                except Exception:
                    pass

            controller = WaypointPIDController(
                vehicle,
                wp_targets,
                gains=gains,
                speed_target=float(speed_target),
                pos_tol=float(self.pos_tol),
                name=path.label,
                allow_hover=allow_hover,
                hold_on_complete=path.airframe == "uav",
            )
            first_active_idx = next(
                (idx for idx, target in enumerate(wp_targets) if not bool(getattr(target, "is_done", False))),
                0,
            )
            controller.curr_idx = int(first_active_idx)
            controller._closest_wp_idx = int(first_active_idx)
            block = self._block_indices.get(path.aircraft_id)
            if block:
                try:
                    controller.set_block_indices(block)
                except Exception:
                    pass

            vehicles[path.label] = SimVehicle(
                label=path.label,
                aircraft_id=path.aircraft_id,
                airframe=path.airframe,
                vehicle=vehicle,
                controller=controller,
                path_id=path.path_id,
                formation=None,
                formation_target=None,
            )

        self.vehicles = vehicles
        for simv in self.vehicles.values():
            self._update_filming_target(simv, 0.0)

    def _apply_vehicle_hit(self, simv: SimVehicle) -> None:
        if not simv.alive:
            return
        simv.alive = False
        simv.crashed = True
        if hasattr(simv.vehicle, "hover_mode"):
            simv.vehicle.hover_mode = False
        if hasattr(simv.vehicle, "cmd_vertical_speed"):
            simv.vehicle.cmd_vertical_speed = 0.0
        simv.vehicle.cmd_throttle = -1.0
        simv.vehicle.cmd_pitch_rate = -getattr(simv.vehicle.p, "max_pitch_rate_dps", 0.0)
        simv.vehicle.cmd_roll_rate = 0.0
        simv.vehicle.cmd_yaw_rate = 0.0

    def _apply_crash(self, simv: SimVehicle, dt: float) -> None:
        if simv.label in self._tracking_state:
            self._tracking_state.pop(simv.label, None)
        s = simv.vehicle.s
        s.u = max(0.0, float(s.u) - 40.0 * dt)
        s.z = max(0.0, float(s.z) - 30.0 * dt)

    def _step_targets(self, dt: float) -> None:
        for tgt in self.targets:
            try:
                tgt.step(dt)
            except Exception:
                continue

    def _check_los_flat(self, sx: float, sy: float, sz: float, tx: float, ty: float, tz: float) -> bool:
        return sz >= tz - 5.0

    def _at_waypoint(self, simv: SimVehicle, wp: WaypointTarget) -> bool:
        s = simv.vehicle.s
        dx = float(wp.pos[0] - s.x)
        dy = float(wp.pos[1] - s.y)
        dz = float(wp.pos[2] - s.z)
        return math.hypot(dx, dy) < float(self.pos_tol) and abs(dz) < float(self.pos_tol) * 0.6

    def _weapon_slot_for_type(self, weapon_type: int) -> str | None:
        if int(weapon_type) == 1:
            return "type1"
        if int(weapon_type) == 2:
            return "type2"
        if int(weapon_type) == 3:
            return "type3"
        return None

    def _weapon_config_for_type(self, weapon_type: int) -> dict[str, float | str]:
        cfg = _FRIENDLY_WEAPON_TYPE_CONFIG.get(int(weapon_type))
        if isinstance(cfg, dict):
            return cfg
        fallback = _FRIENDLY_WEAPON_CONFIG.get("lah")
        return dict(fallback) if isinstance(fallback, dict) else {}

    def _get_weapon_counts(self, simv: SimVehicle) -> dict[str, int]:
        overrides = self._agent_overrides.get(simv.label, {})
        weapons_override = overrides.get("weapons") if isinstance(overrides.get("weapons"), dict) else {}
        return {
            "type1": _coerce_int(weapons_override.get("type1", 5), 5),
            "type2": _coerce_int(weapons_override.get("type2", 10), 10),
            "type3": _coerce_int(weapons_override.get("type3", 100), 100),
        }

    def _set_weapon_count(self, simv: SimVehicle, slot: str, count: int) -> None:
        overrides = dict(self._agent_overrides.get(simv.label, {}))
        weapons_override = overrides.get("weapons") if isinstance(overrides.get("weapons"), dict) else {}
        weapons_override = dict(weapons_override)
        weapons_override[slot] = max(0, int(count))
        overrides["weapons"] = weapons_override
        self._agent_overrides[simv.label] = overrides

    def _friendly_attack_probability(
        self,
        *,
        simv: SimVehicle,
        target_id: int,
        kind: str,
        dist: float,
        max_range: float,
    ) -> tuple[float | None, bool]:
        base_p_hit: float | None = None
        if kind == "missile" and max_range > 0.0:
            ratio = max(0.0, min(1.0, float(dist) / float(max_range)))
            base_p_hit = 0.25 + 0.7 * (1.0 - ratio)
        elif kind == "gun":
            base_p_hit = 0.6

        forced = False
        try:
            target_key = int(target_id)
        except Exception:
            target_key = 0
        if target_key > 0:
            key = (str(simv.label), target_key)
            shot_count = int(self._friendly_attack_attempts.get(key, 0)) + 1
            self._friendly_attack_attempts[key] = shot_count
            # Guarantee hit no later than the 10th friendly shot for this attacker-target pair.
            forced = shot_count >= 10
        if forced:
            return 1.0, True
        return base_p_hit, False

    def _clear_friendly_attack_attempts_for_target(self, target_id: int) -> None:
        try:
            target_key = int(target_id)
        except Exception:
            return
        if target_key <= 0 or not self._friendly_attack_attempts:
            return
        stale = [key for key in self._friendly_attack_attempts if int(key[1]) == target_key]
        for key in stale:
            self._friendly_attack_attempts.pop(key, None)

    def _resolve_attack_target(self, target_id: int) -> GroundTarget | None:
        try:
            target_id = int(target_id)
        except Exception:
            return None
        mapped = self._resolve_actual_target_from_info(target_id)
        if mapped is not None:
            return mapped
        if self._target_id_map_0402:
            raw_id = None
            for raw, assigned in self._target_id_map_0402.items():
                if int(assigned) == target_id:
                    raw_id = raw
                    break
            if raw_id is not None:
                for tgt in self.targets:
                    try:
                        if int(getattr(tgt, "id", 0)) == int(raw_id):
                            return tgt
                    except Exception:
                        continue
            if pending_roi and geo:
                for item in pending_roi:
                    try:
                        lat = float(item.get("lat"))
                        lon = float(item.get("lon"))
                        alt = float(item.get("alt") or 0.0)
                    except Exception:
                        continue
                    try:
                        x, y = geo.lonlat_to_xy(lon, lat)
                        self._roi_mocks.append(
                            self._build_roi_mock(
                                x=x,
                                y=y,
                                z=alt,
                                id_override=item.get("id"),
                                name_override=item.get("name"),
                            )
                        )
                    except Exception:
                        continue
        virtual = self._virtual_target_from_info(target_id)
        if virtual is not None:
            return virtual
        for tgt in self.targets:
            try:
                if int(getattr(tgt, "id", 0)) == target_id:
                    return tgt
            except Exception:
                continue
        return None

    def _resolve_tracking_target(self, target_id: int) -> GroundTarget | None:
        target = self._resolve_attack_target(target_id)
        if target is not None:
            try:
                raw_id = int(getattr(target, "id", 0))
            except Exception:
                raw_id = 0
            if self._target_is_destroyed_in_info(int(target_id)):
                return None
            if raw_id > 0 and self._target_is_destroyed_in_info(raw_id):
                return None
            if not bool(getattr(target, "alive", True)):
                return None
            return target
        virtual = self._virtual_target_from_info(target_id)
        if virtual is not None and not bool(getattr(virtual, "alive", True)):
            return None
        return virtual

    def _resolve_actual_target_from_info(self, target_id: int) -> GroundTarget | None:
        if self.geo is None or not self.targets:
            return None
        related_ids = self._related_target_ids_in_info(int(target_id))
        if self._target_is_destroyed_in_info(int(target_id)):
            return None
        target_list = self._load_target_info_map()
        if not target_list:
            return None
        coord = None
        for entry in target_list.values():
            if not isinstance(entry, dict):
                continue
            try:
                entry_id = int(entry.get("targetID", -1))
            except Exception:
                continue
            if related_ids and entry_id not in related_ids:
                continue
            if _coerce_bool(entry.get("isDestroyed"), False):
                continue
            if self._target_is_ignored_in_info(int(target_id)):
                return None
            coord = entry.get("coordinate")
            if isinstance(coord, dict):
                break
        if not isinstance(coord, dict):
            return None
        try:
            lat = float(coord.get("latitude"))
            lon = float(coord.get("longitude"))
        except Exception:
            return None
        try:
            x, y = self.geo.lonlat_to_xy(lon, lat)
        except Exception:
            return None
        best = None
        best_dist = float("inf")
        for tgt in self.targets:
            if not tgt.alive:
                continue
            dx = float(tgt.x - x)
            dy = float(tgt.y - y)
            dist = math.hypot(dx, dy)
            if dist < best_dist:
                best_dist = dist
                best = tgt
        if best is None or best_dist > _TARGET_INFO_MATCH_RADIUS_M:
            return None
        try:
            self._target_id_map_0402[int(getattr(best, "id", 0))] = int(target_id)
            if int(self._target_id_seq_0402) <= int(target_id):
                self._target_id_seq_0402 = int(target_id) + 1
        except Exception:
            pass
        return best

    def _virtual_target_from_info(self, target_id: int) -> GroundTarget | None:
        if self.geo is None or not self.targets:
            # allow virtual targets even if real targets are missing
            if self.geo is None:
                return None
        related_ids = self._related_target_ids_in_info(int(target_id))
        if self._target_is_destroyed_in_info(int(target_id)):
            for related_id in related_ids:
                virtual = self._virtual_targets.get(int(related_id))
                if virtual is not None:
                    virtual.alive = False
            return None
        target_list = self._load_target_info_map()
        if not target_list:
            return None
        coord = None
        target_type = 0
        for entry in target_list.values():
            if not isinstance(entry, dict):
                continue
            try:
                entry_id = int(entry.get("targetID", -1))
            except Exception:
                continue
            if related_ids and entry_id not in related_ids:
                continue
            if _coerce_bool(entry.get("isDestroyed"), False):
                continue
            if self._target_is_ignored_in_info(int(target_id)):
                return None
            coord = entry.get("coordinate")
            try:
                target_type = int(entry.get("targetType") or 0)
            except Exception:
                target_type = 0
            if isinstance(coord, dict):
                break
        if not isinstance(coord, dict):
            return None
        try:
            lat = float(coord.get("latitude"))
            lon = float(coord.get("longitude"))
            alt = float(coord.get("altitude") or 0.0)
        except Exception:
            return None
        try:
            x, y = self.geo.lonlat_to_xy(lon, lat)
        except Exception:
            return None
        try:
            self._target_id_map_0402[int(target_id)] = int(target_id)
            if int(self._target_id_seq_0402) <= int(target_id):
                self._target_id_seq_0402 = int(target_id) + 1
        except Exception:
            pass
        virtual = self._virtual_targets.get(int(target_id))
        if virtual is None:
            virtual = GroundTarget(
                id=int(target_id),
                type_id=int(target_type),
                name=f"VIRTUAL_{int(target_id)}",
                x=float(x),
                y=float(y),
                z=float(alt),
                moving=False,
                vmin=0.0,
                vmax=0.0,
                roam_center=None,
                roam_radius=None,
                threat=None,
            )
            self._virtual_targets[int(target_id)] = virtual
        else:
            virtual.x = float(x)
            virtual.y = float(y)
            virtual.z = float(alt)
            try:
                virtual.type_id = int(target_type)
            except Exception:
                pass
            virtual.alive = True
        return virtual

    def _current_attack_command(self, simv: SimVehicle) -> dict[str, object] | None:
        try:
            target = simv.controller.current_target()
        except Exception:
            target = None
        if target is None:
            return None
        attack = getattr(target, "attack", None)
        if not isinstance(attack, dict):
            return None
        target_id = attack.get("targetID") or attack.get("TargetID") or attack.get("targetId")
        if target_id is None:
            return None
        try:
            target_id = int(target_id)
        except Exception:
            return None
        if target_id <= 0:
            return None
        weapon_type = attack.get("weaponType") or attack.get("WeaponType") or attack.get("weapon_type")
        try:
            weapon_type = int(weapon_type) if weapon_type is not None else 1
        except Exception:
            weapon_type = 1
        return {
            "target_id": int(target_id),
            "weapon_type": int(weapon_type),
            "wp": target,
            "wp_id": getattr(target, "wp_id", None),
        }

    def _ensure_attack_hold(self, simv: SimVehicle, wp: WaypointTarget | None) -> None:
        if wp is None:
            return
        state = self._attack_holds.get(simv.label)
        if state is None or state.get("wp") is not wp:
            self._attack_holds[simv.label] = {
                "wp": wp,
                "hover_time": wp.hover_time,
                "hover_end": None,
                "restore_hover": wp.hover_time,
                "loiter": wp.loiter,
            }
            if wp.loiter is not None:
                wp.loiter = None
        try:
            ctrl = simv.controller
            if getattr(ctrl, "is_loitering", False):
                ctrl.is_loitering = False
                ctrl.loiter_timer = 0.0
        except Exception:
            pass
        if wp.hover_time is None or float(wp.hover_time) < 1e8:
            wp.hover_time = 1e9

    def _clear_attack_hold(self, simv: SimVehicle, wp: WaypointTarget | None = None) -> None:
        state = self._attack_holds.get(simv.label)
        if not state:
            return
        saved_wp = state.get("wp")
        if wp is not None and saved_wp is not wp:
            return
        if saved_wp is not None:
            try:
                restore_hover = state.get("restore_hover", state.get("hover_time"))
                if restore_hover is None:
                    saved_wp.hover_time = None
                else:
                    try:
                        restore_val = float(restore_hover)
                    except Exception:
                        restore_val = None
                    if restore_val is None or restore_val <= 0.0:
                        saved_wp.hover_time = None
                    else:
                        saved_wp.hover_time = restore_val
            except Exception:
                pass
            try:
                saved_wp.loiter = state.get("loiter")
            except Exception:
                pass
        try:
            ctrl = simv.controller
            if hasattr(ctrl, "is_hovering"):
                ctrl.is_hovering = False
                ctrl.hover_timer = 0.0
            if hasattr(ctrl, "force_hover"):
                ctrl.force_hover = False
        except Exception:
            pass
        self._attack_holds.pop(simv.label, None)

    def _evaluate_vehicle_attacks(self, dt: float) -> None:
        if not self.targets or not self.vehicles:
            return
        for simv in self.vehicles.values():
            if not simv.alive:
                continue
            if simv.airframe != "lah":
                continue
            attack_cmd = self._current_attack_command(simv)
            if attack_cmd is None and not self.lah_auto_attack:
                self._clear_attack_hold(simv)
                continue

            s = simv.vehicle.s

            if attack_cmd is None:
                cfg = _FRIENDLY_WEAPON_CONFIG.get(simv.airframe)
                if not cfg:
                    continue
                last_fire = self._last_vehicle_fire.get(simv.label, -1e9)
                reload_time = max(0.2, float(cfg.get("reload", 0.0) or 0.0))
                if (self.sim_time - last_fire) < reload_time:
                    continue
                best = None
                best_dist = float("inf")
                max_range = float(cfg.get("range", 0.0) or 0.0)
                for tgt in self.targets:
                    if not tgt.alive:
                        continue
                    dx = tgt.x - s.x
                    dy = tgt.y - s.y
                    dz = tgt.z - s.z
                    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                    if dist <= 0.0 or dist > max_range:
                        continue
                    if dist < best_dist:
                        best_dist = dist
                        best = tgt
                if best is None:
                    continue
                speed = float(cfg.get("speed", SIM_PROJECTILE_SPEED_GUN))
                hit_radius = float(cfg.get("hit_radius", SIM_PROJECTILE_HIT_RADIUS_GUN))
                kind = str(cfg.get("kind", "gun"))
                p_hit, force_hit = self._friendly_attack_probability(
                    simv=simv,
                    target_id=int(best.id),
                    kind=kind,
                    dist=float(best_dist),
                    max_range=float(max_range),
                )
                self._spawn_projectile(
                    side="friendly",
                    kind=kind,
                    source_kind="vehicle",
                    source_id=simv.label,
                    target_kind="enemy",
                    target_id=int(best.id),
                    start=(float(s.x), float(s.y), float(s.z)),
                    target=(float(best.x), float(best.y), float(best.z)),
                    speed=speed,
                    hit_radius=hit_radius,
                    max_range=max_range,
                    p_hit=p_hit,
                    force_hit=force_hit,
                )
                self._last_vehicle_fire[simv.label] = float(self.sim_time)
                continue

            attack_target = self._resolve_attack_target(int(attack_cmd["target_id"]))
            wp = attack_cmd.get("wp") if isinstance(attack_cmd.get("wp"), WaypointTarget) else None
            if attack_target is None or not attack_target.alive:
                self._clear_attack_hold(simv, wp)
                continue

            # Only attack after reaching the designated waypoint.
            if wp is not None:
                if not self._at_waypoint(simv, wp):
                    continue
                self._ensure_attack_hold(simv, wp)
                state = self._attack_holds.get(simv.label)
                if state is not None and state.get("hover_end") is None:
                    hover_time = state.get("hover_time")
                    try:
                        hover_val = float(hover_time) if hover_time is not None else 0.0
                    except Exception:
                        hover_val = 0.0
                    if hover_val > 0.0:
                        state["hover_end"] = float(self.sim_time) + hover_val
                    else:
                        state["hover_end"] = float(self.sim_time)
                    state["restore_hover"] = 0.0
                if state is not None and state.get("hover_end") is not None:
                    if float(self.sim_time) < float(state.get("hover_end", 0.0)):
                        continue

            requested_type = int(attack_cmd["weapon_type"])
            if requested_type == 1:
                preference = (1, 2, 3)
            elif requested_type == 2:
                preference = (2, 1, 3)
            elif requested_type == 3:
                preference = (3, 2, 1)
            else:
                preference = (1, 2, 3)
            weapon_type = None
            slot = None
            ammo = None
            for wtype in preference:
                slot = self._weapon_slot_for_type(int(wtype))
                if slot is None:
                    continue
                ammo = self._get_weapon_counts(simv).get(slot, 0)
                if ammo > 0:
                    weapon_type = int(wtype)
                    break
            if weapon_type is None or slot is None or ammo is None or ammo <= 0:
                self._clear_attack_hold(simv, wp)
                continue

            cfg = self._weapon_config_for_type(int(weapon_type))
            if not cfg:
                continue
            max_range = float(cfg.get("range", 0.0) or 0.0)
            dx = attack_target.x - s.x
            dy = attack_target.y - s.y
            dz = attack_target.z - s.z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist <= 0.0:
                continue
            if max_range > 0.0 and dist > max_range:
                continue

            last_fire = self._last_vehicle_fire.get(simv.label, -1e9)
            reload_time = max(0.2, float(cfg.get("reload", 0.0) or 0.0))
            if (self.sim_time - last_fire) < reload_time:
                continue

            speed = float(cfg.get("speed", SIM_PROJECTILE_SPEED_GUN))
            hit_radius = float(cfg.get("hit_radius", SIM_PROJECTILE_HIT_RADIUS_GUN))
            kind = str(cfg.get("kind", "gun"))
            p_hit, force_hit = self._friendly_attack_probability(
                simv=simv,
                target_id=int(attack_target.id),
                kind=kind,
                dist=float(dist),
                max_range=float(max_range),
            )
            self._spawn_projectile(
                side="friendly",
                kind=kind,
                source_kind="vehicle",
                source_id=simv.label,
                target_kind="enemy",
                target_id=int(attack_target.id),
                start=(float(s.x), float(s.y), float(s.z)),
                target=(float(attack_target.x), float(attack_target.y), float(attack_target.z)),
                speed=speed,
                hit_radius=hit_radius,
                max_range=max_range,
                p_hit=p_hit,
                force_hit=force_hit,
            )
            self._last_vehicle_fire[simv.label] = float(self.sim_time)
            if slot is not None and ammo is not None:
                self._set_weapon_count(simv, slot, ammo - 1)

    def _evaluate_threats(self, dt: float) -> None:
        if not self.targets or not self.vehicles:
            return
        for tgt in self.targets:
            if not tgt.alive or tgt.threat is None:
                continue
            weapon = tgt.threat.weapon
            if weapon is None:
                continue
            max_range = float(weapon.a_range or 0.0)
            if max_range <= 0.0:
                continue
            best = None
            best_dist = float("inf")
            best_state = None
            for simv in self.vehicles.values():
                if not simv.alive:
                    continue
                s = simv.vehicle.s
                dx = tgt.x - s.x
                dy = tgt.y - s.y
                dz = tgt.z - s.z
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                if dist <= 0.0 or dist > _MAX_THREAT_RANGE_M:
                    continue
                if dist < best_dist:
                    best_dist = dist
                    best = simv
                    best_state = (s.x, s.y, s.z)
            if best is None or best_state is None:
                continue
            if not tgt.moving:
                desired_heading = self._bearing_deg(tgt.x, tgt.y, best_state[0], best_state[1])
                tgt.heading = float(desired_heading)
                tgt.heading_rate = 0.0
            los = self._check_los_flat(best_state[0], best_state[1], best_state[2], tgt.x, tgt.y, tgt.z)
            tgt.threat.detection_prob(best_dist, los, dt)
            if not tgt.threat.state.detected:
                continue
            if best_dist > max_range:
                continue
            last_fire = self._last_enemy_fire.get(int(tgt.id), -1e9)
            reload_time = max(0.2, float(weapon.reload or 0.0))
            if (self.sim_time - last_fire) < reload_time:
                continue
            shot_dt = max(float(dt or 0.0), reload_time)
            p_hit = tgt.threat.kill_prob(best_dist, shot_dt)
            if p_hit > 0.0:
                try:
                    p_hit = float(p_hit) * float(self.enemy_hit_scale)
                except Exception:
                    p_hit = float(p_hit)
                p_hit = max(0.0, min(1.0, float(p_hit)))
            if p_hit <= 0.0:
                continue
            speed = self._weapon_projectile_speed(weapon)
            hit_radius = self._weapon_hit_radius(weapon)
            kind = _PROJECTILE_KIND_BY_WEAPON.get(weapon.weapon_type, "gun")
            if weapon.weapon_type is WeaponType.GUN and random.random() < p_hit:
                self._apply_vehicle_hit(best)
                self._spawn_effect(
                    side="enemy",
                    kind=kind,
                    x=float(best_state[0]),
                    y=float(best_state[1]),
                    z=float(best_state[2]),
                )
            self._spawn_projectile(
                side="enemy",
                kind=kind,
                source_kind="enemy",
                source_id=int(tgt.id),
                target_kind="vehicle",
                target_id=best.label,
                start=(float(tgt.x), float(tgt.y), float(tgt.z)),
                target=(float(best_state[0]), float(best_state[1]), float(best_state[2])),
                speed=speed,
                hit_radius=hit_radius,
                max_range=max_range,
                p_hit=p_hit,
            )
            self._last_enemy_fire[int(tgt.id)] = float(self.sim_time)

    def _step_projectiles(self, dt: float) -> None:
        if not self._projectiles:
            return
        target_by_id = {int(tgt.id): tgt for tgt in self.targets}
        if self._virtual_targets:
            for vid, vtgt in self._virtual_targets.items():
                try:
                    vid_int = int(vid)
                except Exception:
                    continue
                if vid_int not in target_by_id:
                    target_by_id[vid_int] = vtgt
        remaining: list[Projectile] = []
        for proj in self._projectiles:
            proj.ttl -= float(dt)
            if proj.ttl <= 0.0:
                continue
            snap_hit = False
            if proj.kind == "missile":
                tx = ty = tz = None
                if proj.target_kind == "vehicle":
                    simv = self.vehicles.get(str(proj.target_id))
                    if simv is not None and simv.alive:
                        s = simv.vehicle.s
                        tx, ty, tz = float(s.x), float(s.y), float(s.z)
                elif proj.target_kind == "enemy":
                    tgt = target_by_id.get(int(proj.target_id))
                    if tgt is not None and tgt.alive:
                        tx, ty, tz = float(tgt.x), float(tgt.y), float(tgt.z)
                if tx is not None:
                    dx = tx - proj.x
                    dy = ty - proj.y
                    dz = tz - proj.z
                    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                    if dist > 1e-6:
                        speed = max(1.0, float(proj.speed))
                        step_len = speed * float(dt)
                        if dist <= step_len:
                            proj.x, proj.y, proj.z = tx, ty, tz
                            snap_hit = True
                        else:
                            proj.vx = dx / dist * speed
                            proj.vy = dy / dist * speed
                            proj.vz = dz / dist * speed
            if not snap_hit:
                proj.x += proj.vx * float(dt)
                proj.y += proj.vy * float(dt)
                proj.z += proj.vz * float(dt)
            if proj.target_kind == "vehicle":
                simv = self.vehicles.get(str(proj.target_id))
                if simv is None or not simv.alive:
                    continue
                s = simv.vehicle.s
                dx = proj.x - s.x
                dy = proj.y - s.y
                dz = proj.z - s.z
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                if snap_hit or dist <= proj.hit_radius:
                    if proj.p_hit is None or random.random() < proj.p_hit:
                        self._apply_vehicle_hit(simv)
                        self._spawn_effect(
                            side=proj.side,
                            kind=proj.kind,
                            x=float(s.x),
                            y=float(s.y),
                            z=float(s.z),
                        )
                    continue
            elif proj.target_kind == "enemy":
                tgt = target_by_id.get(int(proj.target_id))
                if tgt is None or not tgt.alive:
                    continue
                dx = proj.x - tgt.x
                dy = proj.y - tgt.y
                dz = proj.z - tgt.z
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                if snap_hit or dist <= proj.hit_radius:
                    if proj.p_hit is None or random.random() < proj.p_hit:
                        tgt.alive = False
                        if tgt.threat is not None:
                            tgt.threat.reset()
                        watcher_id = None
                        if proj.source_kind == "vehicle":
                            watcher_id = _label_to_aircraft_id(str(proj.source_id))
                        self._handle_target_destroyed(tgt, watcher_id)
                        self._spawn_effect(
                            side=proj.side,
                            kind=proj.kind,
                            x=float(tgt.x),
                            y=float(tgt.y),
                            z=float(tgt.z),
                        )
                    continue
            remaining.append(proj)
        self._projectiles = remaining

    def _step_effects(self, dt: float) -> None:
        if not self._effects:
            return
        remaining: list[ImpactEffect] = []
        for eff in self._effects:
            eff.age += float(dt)
            if eff.age <= eff.ttl:
                remaining.append(eff)
        self._effects = remaining

    def _step_once(self, dt: float) -> None:
        for simv in self.vehicles.values():
            try:
                if not simv.alive:
                    self._apply_crash(simv, dt)
                    continue
                forced_active = self._apply_forced_state(simv)
                if forced_active:
                    simv.controller.update(dt)
                    self._update_filming_target(simv, dt)
                    simv.vehicle.step(dt)
                    continue
                formation_spec = self._active_formation_spec(simv)
                if formation_spec is not None and not formation_spec.is_leader:
                    if self._sync_formation_follower_to_leader(simv):
                        self._update_filming_target(simv, dt)
                        continue
                    self._update_filming_target(simv, dt)
                    continue
                self._update_roi_mock_focus(simv)
                self._update_tracking(simv, dt)
                if simv.airframe == "lah":
                    try:
                        attack_cmd = self._current_attack_command(simv)
                        attack_target = (
                            self._resolve_attack_target(int(attack_cmd["target_id"]))
                            if isinstance(attack_cmd, dict)
                            else None
                        )
                        wp = attack_cmd.get("wp") if isinstance(attack_cmd, dict) else None
                        if (
                            attack_target is not None
                            and attack_target.alive
                            and isinstance(wp, WaypointTarget)
                        ):
                            self._ensure_attack_hold(simv, wp)
                    except Exception:
                        pass
                prev_target = None
                if simv.airframe == "uav":
                    try:
                        prev_target = simv.controller.current_target()
                    except Exception:
                        prev_target = None
                simv.controller.update(dt)
                if simv.airframe == "uav":
                    try:
                        ctrl = simv.controller
                        if (
                            getattr(ctrl, "just_advanced", False)
                            and getattr(ctrl, "advance_reason", None) == "loiter"
                        ):
                            self._pulse_on_mission(simv.label)
                        if (
                            getattr(ctrl, "just_advanced", False)
                            and self._target_filming_complete_for_pulse(simv, prev_target)
                        ):
                            self._pulse_filming(simv.label)
                    except Exception:
                        pass
                if simv.airframe == "lah":
                    try:
                        wp = simv.controller.current_target()
                    except Exception:
                        wp = None
                    if wp is not None:
                        attack = getattr(wp, "attack", None)
                        if isinstance(attack, dict):
                            target_id = attack.get("targetID") or attack.get("TargetID") or attack.get("targetId")
                            try:
                                target_id = int(target_id)
                            except Exception:
                                target_id = 0
                            if target_id > 0 and self._at_waypoint(simv, wp):
                                self._ensure_attack_hold(simv, wp)
                                # Force a hard hover for attack waypoints.
                                try:
                                    simv.vehicle.s.u = 0.0
                                except Exception:
                                    pass
                                if hasattr(simv.vehicle, "hover_mode"):
                                    simv.vehicle.hover_mode = True
                                if hasattr(simv.vehicle, "cmd_vertical_speed"):
                                    simv.vehicle.cmd_vertical_speed = 0.0
                                simv.vehicle.cmd_throttle = 0.0
                                simv.vehicle.cmd_yaw_rate = 0.0
                                simv.vehicle.cmd_pitch_rate = 0.0
                                simv.vehicle.cmd_roll_rate = 0.0
                self._update_filming_target(simv, dt)
                simv.vehicle.step(dt)
            except Exception:
                continue
        self._step_targets(dt)
        self._evaluate_vehicle_attacks(dt)
        self._evaluate_threats(dt)
        self._step_projectiles(dt)
        self._step_effects(dt)
        self._apply_pending_input_advances_locked()

    def _build_frame(
        self,
        *,
        geo: GeoConverter,
        vehicles: list[SimVehicle],
        targets: list[GroundTarget] | None = None,
        roi_mocks: list[RoiMock] | None = None,
        projectiles: list[Projectile] | None = None,
        effects: list[ImpactEffect] | None = None,
        sim_time: float,
        step_count: int,
    ) -> dict:
        frame: dict[str, Any] = {
            "step": int(step_count),
            "simTime": float(sim_time),
            "vehicles": {},
            "targets": [],
            "rois": [],
            "projectiles": [],
            "effects": [],
        }
        targets_list = targets if targets is not None else self.targets
        for simv in vehicles:
            s = simv.vehicle.s
            lon, lat = geo.xy_to_lonlat(s.x, s.y)
            overrides = self._agent_overrides.get(simv.label, {})
            health_value = 1 if simv.alive else 2
            if simv.alive and "health" in overrides:
                health_value = max(0, min(2, _coerce_int(overrides.get("health"), health_value)))
            fuel = self._fuel_remaining(self._fuel_multiplier_for(simv))
            entry: dict[str, Any] = {
                "lat": float(lat),
                "lon": float(lon),
                "alt": float(s.z),
                "speed": float(getattr(s, "u", 0.0)),
                "heading": _aircraft_yaw_to_nav_heading_deg(float(getattr(s, "yaw", 0.0))),
                "alive": bool(simv.alive),
                "health": int(health_value),
                "fuel": float(fuel),
            }
            if simv.airframe == "lah":
                try:
                    entry["weapons"] = dict(self._get_weapon_counts(simv))
                except Exception:
                    pass
            if simv.airframe == "uav":
                flight_mode = int(self._flight_mode_for(simv))
                flying = int(self._on_mission_for(simv))
                filming_status = int(
                    self._filming_status_for(
                        simv,
                        filming_prop=self._filming_props.get(simv.label),
                        flying_status=flying,
                    )
                )
                target_id = 0
                tracking = self._tracking_state.get(simv.label)
                forced = self._forced_commands.get(simv.label)
                if tracking is not None and tracking.stage >= 1 and tracking.target.alive:
                    target_id = self._assign_0402_target_id(tracking.target)
                current_wp = self._reported_uav_current_waypoint_id(
                    simv=simv,
                    tracking=tracking,
                    forced=forced,
                    flight_mode=flight_mode,
                )
                entry["flightMode"] = flight_mode
                entry["flying"] = flying
                entry["filming"] = filming_status
                entry["currentWaypointID"] = int(current_wp) if current_wp is not None else None
                entry["targetID"] = int(target_id)
                if flight_mode == 8:
                    center = None
                    ctrl = simv.controller
                    if getattr(ctrl, "is_loitering", False):
                        center = getattr(ctrl, "loiter_center", None)
                    if center is None:
                        center = (float(s.x), float(s.y), float(s.z))
                    try:
                        l_lon, l_lat = geo.xy_to_lonlat(float(center[0]), float(center[1]))
                        entry["loiterCoordinate"] = {
                            "latitude": float(l_lat),
                            "longitude": float(l_lon),
                            "altitude": float(center[2]),
                        }
                    except Exception:
                        pass
            target = self._filming_targets.get(simv.label)
            if target is not None:
                t_lon, t_lat = geo.xy_to_lonlat(target[0], target[1])
                entry["filmingTarget"] = {
                    "lat": float(t_lat),
                    "lon": float(t_lon),
                    "alt": float(target[2]),
                }
            max_sep_m = self._filming_max_sep_m.get(simv.label)
            if isinstance(max_sep_m, (int, float)) and float(max_sep_m) > 0.0:
                entry["filmingMaxSep"] = float(max_sep_m)
            filming_prop = self._filming_props.get(simv.label)
            filming_fov: float | None = None
            if isinstance(filming_prop, dict):
                entry["filmingMode"] = filming_prop.get("operationMode")
                fov = filming_prop.get("fieldOfView")
                if fov is None:
                    fov = filming_prop.get("fov")
                try:
                    filming_fov = float(fov) if fov is not None else None
                except Exception:
                    filming_fov = None
                if filming_fov is not None and math.isfinite(filming_fov):
                    entry["filmingFov"] = float(filming_fov)
            if target is not None and filming_fov is not None and filming_fov > 0.0:
                try:
                    projection = self._resolve_footprint_projection(
                        simv,
                        (float(target[0]), float(target[1]), float(target[2])),
                        float(filming_fov),
                    )
                    if projection is not None:
                        center = projection.center_hit
                        c_lon, c_lat = geo.xy_to_lonlat(float(center[0]), float(center[1]))
                        entry["filmingTarget"] = {
                            "lat": float(c_lat),
                            "lon": float(c_lon),
                            "alt": float(center[2]),
                        }
                        corners = []
                        for px, py, pz in projection.corners:
                            lon_fp, lat_fp = geo.xy_to_lonlat(float(px), float(py))
                            corners.append([float(lon_fp), float(lat_fp), float(pz)])
                        if corners:
                            entry["footprintCorners"] = corners
                except Exception:
                    pass
            if simv.label in self._filming_wp_ids:
                entry["filmingWpId"] = self._filming_wp_ids.get(simv.label)
            frame["vehicles"][simv.label] = entry
        for tgt in targets_list:
            try:
                frame["targets"].append(self._target_to_dict(tgt, geo))
            except Exception:
                continue
        roi_list = roi_mocks if roi_mocks is not None else self._roi_mocks
        for roi in roi_list:
            try:
                frame["rois"].append(self._roi_mock_to_dict(roi, geo))
            except Exception:
                continue
        projectiles_list = projectiles if projectiles is not None else self._projectiles
        for proj in projectiles_list:
            try:
                lon, lat = geo.xy_to_lonlat(proj.x, proj.y)
                frame["projectiles"].append(
                    {
                        "id": int(proj.id),
                        "lat": float(lat),
                        "lon": float(lon),
                        "alt": float(proj.z),
                        "vx": float(proj.vx),
                        "vy": float(proj.vy),
                        "vz": float(proj.vz),
                        "speed": float(proj.speed),
                        "side": proj.side,
                        "kind": proj.kind,
                        "target": proj.target_id,
                    }
                )
            except Exception:
                continue
        effects_list = effects if effects is not None else self._effects
        for eff in effects_list:
            try:
                lon, lat = geo.xy_to_lonlat(eff.x, eff.y)
                frame["effects"].append(
                    {
                        "id": int(eff.id),
                        "lat": float(lat),
                        "lon": float(lon),
                        "alt": float(eff.z),
                        "age": float(eff.age),
                        "ttl": float(eff.ttl),
                        "radius": float(eff.radius_m),
                        "flash": float(eff.flash_m),
                        "side": eff.side,
                        "kind": eff.kind,
                    }
                )
            except Exception:
                continue
        return frame

    def _record_history(self) -> None:
        geo = self.geo
        if geo is None:
            return
        vehicles = list(self.vehicles.values())
        targets = list(self.targets)
        roi_mocks = list(self._roi_mocks)
        frame = self._build_frame(
            geo=geo,
            vehicles=vehicles,
            targets=targets,
            roi_mocks=roi_mocks,
            sim_time=self.sim_time,
            step_count=self.step_count,
        )
        self._history.append(frame)

    def _history_sample_interval_sim(self, speed_factor: float, dt: float) -> float:
        speed = max(0.1, abs(float(speed_factor or 1.0)))
        sample_hz = max(1.0, float(self._history_sample_hz))
        return max(float(dt), speed / sample_hz)

    def _record_history_if_due(self, speed_factor: float, dt: float) -> None:
        interval = self._history_sample_interval_sim(speed_factor, dt)
        last_recorded = self._last_history_record_sim_time
        if last_recorded is None or (float(self.sim_time) - float(last_recorded)) >= interval:
            self._record_history()
            self._last_history_record_sim_time = float(self.sim_time)

    def _loop(self) -> None:
        last = time.perf_counter()
        accum = 0.0
        last_speed_seq = -1
        while not self._shutdown.is_set():
            with self._lock:
                running = self.running
                paused = self.paused
                speed = float(self.speed_factor)
                dt = float(self.dt)
                speed_seq = int(self._speed_change_seq)
                vehicles_ready = bool(self.vehicles)

            if speed_seq != last_speed_seq:
                # Drop any accumulated backlog on speed change for immediate response.
                accum = 0.0
                last_speed_seq = speed_seq

            if not running or paused or not vehicles_ready:
                with self._lock:
                    self._last_0401_emit_wall_time = None
                last = time.perf_counter()
                self._shutdown.wait(0.02)
                continue

            now = time.perf_counter()
            wall_dt = max(0.0, now - last)
            last = now

            accum += wall_dt * speed
            advanced = 0
            max_steps = max(1, int(self._max_steps_per_loop))
            while accum >= dt and advanced < max_steps:
                with self._lock:
                    if not (self.running and not self.paused and self.vehicles):
                        accum = 0.0
                        break
                    self._step_once(dt)
                    self.sim_time += dt
                    self.step_count += 1
                    self._record_history_if_due(speed, dt)
                    self._maybe_push_0401()
                    self._maybe_push_0402()
                accum -= dt
                advanced += 1

            if advanced >= max_steps and accum >= dt:
                accum = min(accum, dt * max_steps)

            if advanced == 0:
                with self._lock:
                    if self.running and not self.paused and self.vehicles:
                        self._maybe_push_0401()
                self._shutdown.wait(0.005)
            else:
                self._shutdown.wait(0.001)

    def build_snapshot(self, *, since_step: int | None = None) -> dict:
        with self._lock:
            geo = self.geo
            vehicles = list(self.vehicles.values())
            targets = list(self.targets)
            roi_mocks = list(self._roi_mocks)
            pending_targets = list(self._pending_targets)
            pending_roi_mocks = list(self._pending_roi_mocks)
            running = self.running
            paused = self.paused
            speed = float(self.speed_factor)
            dt = float(self.dt)
            sim_time = float(self.sim_time)
            step_count = int(self.step_count)
            error = self.last_error
            history = list(self._history)
            events_0402 = list(self._events_0402)

        payload: dict[str, Any] = {
            "ok": True,
            "running": running,
            "paused": paused,
            "speedFactor": speed,
            "dt": dt,
            "historySampleHz": float(self._history_sample_hz),
            "simTime": sim_time,
            "step": step_count,
            "vehicles": {},
            "targets": [],
            "rois": [],
            "projectiles": [],
            "effects": [],
            "events0402": events_0402,
        }
        if error:
            payload["error"] = error

        if geo is None:
            if pending_targets:
                payload["targets"] = pending_targets
            if pending_roi_mocks:
                payload["rois"] = pending_roi_mocks
            return payload

        latest_frame = self._build_frame(
            geo=geo,
            vehicles=vehicles,
            targets=targets,
            roi_mocks=roi_mocks,
            sim_time=sim_time,
            step_count=step_count,
        )
        payload["vehicles"] = latest_frame["vehicles"]
        payload["targets"] = latest_frame.get("targets", [])
        payload["rois"] = latest_frame.get("rois", [])
        payload["projectiles"] = latest_frame.get("projectiles", [])
        payload["effects"] = latest_frame.get("effects", [])

        if since_step is None:
            return payload

        history_frames = [frame for frame in history if frame.get("step", -1) > since_step]
        if len(history_frames) > self._history_response_max:
            history_frames = history_frames[-self._history_response_max :]
        events_since = [event for event in events_0402 if event.get("step", -1) > since_step]
        payload["events0402"] = events_since
        return {
            "ok": True,
            "history": history_frames,
            "latest": payload,
            "events0402": events_since,
        }
