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
    SIM_BASE_DT,
    SIM_0401_ACTIVE_HZ,
    SIM_0402_HISTORY_MAX,
    SIM_HISTORY_MAX,
    SIM_INTERNAL_STEP_HZ,
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
    SIM_TRACK_ALT_BUFFER_M,
    SIM_TRACK_LOITER_RADIUS_M,
    SIM_TRACK_LOITER_SPEED_MPS,
    SIM_TRACK_LOST_TIMEOUT_S,
    SIM_UAV_DETECT_RANGE_M,
)
from modules.common import agent_status_snapshot, db_paths
from .geo import GeoConverter
from .lah import LAH, LAHParams
from .uav import UAV, UAVParams
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
        alt_v = float(alt) if alt is not None else coord[2]
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
_FOOTPRINT_FOV_INTERPRETATION = "diagonal_full"
_FOOTPRINT_EARTH_RADIUS_M = 6_378_137.0
_FOOTPRINT_CORNER_DEFINITIONS = (
    (-1.0, 1.0),
    (1.0, 1.0),
    (1.0, -1.0),
    (-1.0, -1.0),
)
_FOOTPRINT_DEM_DIR = Path(__file__).resolve().parents[3] / "resource"
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
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float], float]:
    fwd_x = float(focus[0] - origin[0])
    fwd_y = float(focus[1] - origin[1])
    fwd_z = float(focus[2] - origin[2])
    forward = _normalize3(fwd_x, fwd_y, fwd_z)
    world_up = (0.0, 0.0, 1.0)
    right = _cross3(forward[0], forward[1], forward[2], world_up[0], world_up[1], world_up[2])
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
        self._line_search_state: dict[str, object | None] = {}
        self._line_search_debug: dict[str, object | None] = {}
        self._tracking_state: dict[str, TrackingState] = {}
        self._tracking_target_owner: dict[int, str] = {}
        self._tracking_overrides: dict[str, dict[str, Any]] = {}
        self._tracking_override_seq = 0
        self._virtual_targets: dict[int, GroundTarget] = {}
        self._formation_by_aircraft: dict[int, FormationSpec] = {}
        self._history = deque(maxlen=int(SIM_HISTORY_MAX))
        self._events_0402 = deque(maxlen=int(SIM_0402_HISTORY_MAX))
        self._reported_0402_roi: set[tuple[int, int]] = set()
        self._reported_0402_list: set[tuple[int, int]] = set()
        self._target_id_map_0402: dict[int, int] = {}
        self._target_id_seq_0402 = 7
        self.input_mission_order_by_aircraft: dict[int, list[int]] = {}
        self.current_input_mission_idx_by_aircraft: dict[int, int] = {}
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
        self._agent_overrides: dict[str, dict[str, Any]] = {}
        self._forced_commands: dict[str, dict[str, Any]] = {}
        self._rtb_coord_cache: tuple[float, float, float] | None = None
        self._initial_aircraft_ids: set[int] = set()
        self._terrain_elev_fn = None
        self._target_info_cache_mtime_ns: int | None = None
        self._target_info_cache_map: dict[str, dict[str, Any]] = {}

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
        return {"ok": True}

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
            self._filming_props = {}
            self._filming_targets = {}
            self._filming_wp_ids = {}
            self._line_search_state = {}
            self._line_search_debug = {}
            self._history.clear()
            self._reset_0402_state()
            self._tracking_overrides = {}
            self._tracking_override_seq = 0
            self._virtual_targets = {}
            self._formation_by_aircraft = {}
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
            self._reset_0402_state()
            self._tracking_overrides = {}
            self._tracking_override_seq = 0
            self._virtual_targets = {}
            self._formation_by_aircraft = {}
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
            self._agent_overrides = {}
            self._forced_commands = {}
            self._rtb_coord_cache = None
        return {"ok": True}

    def _reset_0402_state(self) -> None:
        self._events_0402.clear()
        self._reported_0402_roi = set()
        self._reported_0402_list = set()
        self._target_id_map_0402 = {}
        self._target_id_seq_0402 = 7
        self._target_watcher_0402 = {}
        self._tracking_target_owner = {}

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

    def advance_input_mission(self, aircraft_id: Optional[int] = None) -> int:
        advanced = 0
        with self._lock:
            targets = (
                [aircraft_id]
                if aircraft_id is not None
                else sorted(self.input_mission_order_by_aircraft.keys())
            )
            if aircraft_id is not None and _airframe_type(int(aircraft_id)) == "uav":
                # When UAVs advance, keep manned aircraft in sync.
                for aid in (1, 2, 3):
                    if aid in self.input_mission_order_by_aircraft:
                        targets.append(aid)
            targets = sorted({int(t) for t in targets if t is not None})
            for aid in targets:
                cur_id = self._current_input_mission_id_for(aid)
                if cur_id is None:
                    continue
                if self._next_input_mission_id_for(aid) is None:
                    continue
                label = _agent_label(aid)
                simv = self.vehicles.get(label)
                if not simv:
                    continue
                forced = self._forced_commands.get(label)
                if forced and forced.get("block_mission"):
                    continue
                ap = simv.controller
                if getattr(ap, "blocked_input_id", None) != cur_id:
                    if simv.airframe == "lah":
                        if self._skip_to_next_input_mission(simv, cur_id):
                            self.current_input_mission_idx_by_aircraft[aid] = (
                                self.current_input_mission_idx_by_aircraft.get(aid, 0) + 1
                            )
                            advanced += 1
                    continue
                try:
                    ap.release_block()
                except Exception:
                    continue
                self.current_input_mission_idx_by_aircraft[aid] = (
                    self.current_input_mission_idx_by_aircraft.get(aid, 0) + 1
                )
                advanced += 1
        return advanced

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

    def _target_to_dict(self, target: GroundTarget, geo: GeoConverter) -> dict:
        lon, lat = geo.xy_to_lonlat(target.x, target.y)
        return {
            "id": int(target.id),
            "type": int(target.type_id),
            "name": str(target.name),
            "lat": float(lat),
            "lon": float(lon),
            "alt": float(target.z),
            "moving": bool(target.moving),
            "alive": bool(target.alive),
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

    def clear_targets(self) -> dict:
        with self._lock:
            self.targets = []
            self._target_counts = {}
            self._target_id_seq = 1
            self._pending_targets = []
            self._tracking_state = {}
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

    def load_mission(self, payload: dict) -> dict:
        preserve_state = _coerce_bool(
            payload.get("preserveState")
            or payload.get("preserve_state")
            or payload.get("keepState")
            or payload.get("keep_state"),
            False,
        )
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
        tracking_overrides_by_aircraft: dict[int, dict[str, Any]] = {}
        formation_by_aircraft: dict[int, FormationSpec] = {}

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
            # Formation flight is currently disabled; ignore formationInfo.

            waypoints_raw = _extract_waypoints(data)
            if not waypoints_raw:
                continue
            waypoints_raw = _order_waypoints(waypoints_raw)
            if path_id is not None:
                flight_by_path[path_id] = data
                if aircraft_id > 0:
                    flight_by_aircraft.setdefault(aircraft_id, []).append(path_id)

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
                filming = wp.get("filmingProperty")
                if aircraft_id >= 4:
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
                        "hover_time": hover_time,
                        "loiter": loiter,
                        "pass_type": pass_type,
                        "filming": filming,
                        "attack": attack if isinstance(attack, dict) else None,
                        "path_id": path_id,
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
                # Keep done missions in payload for visualization/history, but exclude from execution sequence.
                if _coerce_bool(im.get("isDone"), False):
                    continue
                rel = im.get("relatedMission") or {}
                try:
                    input_id = int(rel.get("inputMissionID"))
                except Exception:
                    input_id = None
                try:
                    path_id = int(im.get("pathID"))
                except Exception:
                    path_id = None
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
                        filming = wp.get("filmingProperty")
                        if aircraft_id >= 4:
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
                                "hover_time": hover_time,
                                "loiter": loiter,
                                "pass_type": pass_type,
                                "filming": filming,
                                "attack": attack if isinstance(attack, dict) else None,
                                "path_id": pid,
                                "input_mission_id": entry.get("input_mission_id"),
                                "individual_mission_id": entry.get("individual_mission_id"),
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
                        filming = wp.get("filmingProperty")
                        if aircraft_id >= 4:
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
                                "hover_time": hover_time,
                                "loiter": loiter,
                                "pass_type": pass_type,
                                "filming": filming,
                                "attack": attack if isinstance(attack, dict) else None,
                                "path_id": pid_int,
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
                    filming = wp.get("filmingProperty")
                    if aircraft_id >= 4:
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
                        "hover_time": hover_time,
                        "loiter": loiter,
                        "pass_type": pass_type,
                        "filming": filming,
                        "attack": attack if isinstance(attack, dict) else None,
                        "path_id": pid,
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
            return {"ok": False, "error": "no valid flight paths"}

        if not all_latlons:
            return {"ok": False, "error": "waypoints missing coordinates"}

        lon_avg = sum(lon for lon, _ in all_latlons) / len(all_latlons)
        lat_avg = sum(lat for _, lat in all_latlons) / len(all_latlons)

        spawn_latlon: dict[int, tuple[float, float, float]] = {}
        for item in take_over_list:
            if not isinstance(item, dict):
                continue
            try:
                aircraft_id = int(item.get("aircraftID") or item.get("AircraftID") or 0)
            except Exception:
                aircraft_id = 0
            if aircraft_id <= 0:
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
            pending = list(self._pending_targets)
            self._pending_targets = []
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
            self._build_vehicles(paths)
            retained_labels: set[str] = set()
            retained_aircraft_ids: set[int] = set()
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
                    if not state.get("alive", True):
                        self._apply_vehicle_hit(simv)
                        simv.crashed = bool(state.get("crashed", True))
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
            if preserve_state and retained_labels:
                if prev_tracking is not None:
                    self._tracking_state = {
                        label: prev_tracking[label]
                        for label in retained_labels
                        if label in prev_tracking
                    }
                if prev_tracking_owner is not None:
                    self._tracking_target_owner = {
                        int(tid): lbl
                        for tid, lbl in prev_tracking_owner.items()
                        if lbl in retained_labels
                    }
                if prev_line_search_state is not None:
                    for label in retained_labels:
                        if label in prev_line_search_state:
                            self._line_search_state[label] = prev_line_search_state[label]
                if prev_line_search_debug is not None:
                    for label in retained_labels:
                        if label in prev_line_search_debug:
                            self._line_search_debug[label] = prev_line_search_debug[label]
                for label in retained_labels:
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
            self._agent_overrides = prev_overrides if preserve_state and prev_overrides is not None else {}
            if not (preserve_state and prev_forced):
                self._forced_commands = {}
            self._rtb_coord_cache = None
            if not seq_by_aircraft:
                self.input_mission_order_by_aircraft = {}
                self.current_input_mission_idx_by_aircraft = {}
                self._block_indices = {}
                self._spawn_by_aircraft = spawn_by_aircraft
            if preserve_state and prev_input_order and retained_aircraft_ids:
                for aid in retained_aircraft_ids:
                    if aid in updated_aircraft_ids:
                        continue
                    if aid in prev_input_order:
                        self.input_mission_order_by_aircraft[aid] = list(prev_input_order[aid])
                    if prev_input_idx and aid in prev_input_idx:
                        self.current_input_mission_idx_by_aircraft[aid] = int(prev_input_idx[aid])
            if formation_by_aircraft:
                self._formation_by_aircraft = formation_by_aircraft
            else:
                self._formation_by_aircraft = {}
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
            return 610.0
        if aid == 5:
            return 620.0
        if aid == 6:
            return 630.0
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
            forward, right, up, fwd_len = _build_footprint_camera_axes(origin, focus)
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
        corners: list[tuple[float, float, float]] = []
        for sx, sy in _FOOTPRINT_CORNER_DEFINITIONS:
            dir_x = forward[0] + (sx * tan_h * right[0]) + (sy * tan_v * up[0])
            dir_y = forward[1] + (sx * tan_h * right[1]) + (sy * tan_v * up[1])
            dir_z = forward[2] + (sx * tan_h * right[2]) + (sy * tan_v * up[2])
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
            if hit is None:
                if abs(dir_z) < 1e-6:
                    return None
                plane_z = float(center_hit[2])
                t = (plane_z - origin[2]) / dir_z
                if t <= 0.0:
                    return None
                hit = (
                    float(origin[0] + (dir_x * t)),
                    float(origin[1] + (dir_y * t)),
                    plane_z,
                )
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

    def _target_is_ignored_in_info(self, target_id: int) -> bool:
        try:
            target_id = int(target_id)
        except Exception:
            return False
        if target_id <= 0:
            return False
        target_map = self._load_target_info_map()
        for entry in target_map.values():
            if not isinstance(entry, dict):
                continue
            try:
                entry_id = int(entry.get("targetID", -1))
            except Exception:
                continue
            if entry_id != target_id:
                continue
            try:
                if int(entry.get("isIgnored") or 0) != 0:
                    return True
            except Exception:
                continue
        return False

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
        s = simv.vehicle.s
        dx = float(tgt.x - s.x)
        dy = float(tgt.y - s.y)
        dz = float(tgt.z - s.z)
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if self.uav_detection_range_m > 0.0 and dist > self.uav_detection_range_m:
            return False
        if not self._check_los_flat(s.x, s.y, s.z, tgt.x, tgt.y, tgt.z):
            return False
        if fov_deg <= 0.0:
            return True
        return self._footprint_contains(simv, tgt, fov_deg)

    def _find_detected_target(self, simv: SimVehicle, fov_deg: float) -> tuple[int, GroundTarget] | None:
        if not self.targets:
            return None
        best = None
        best_dist = float("inf")
        s = simv.vehicle.s
        for tgt in self.targets:
            if not tgt.alive:
                continue
            if self._ground_target_is_ignored(tgt):
                continue
            owner = self._tracking_target_owner.get(int(tgt.id))
            if owner and owner != simv.label:
                continue
            if not self._target_in_view(simv, tgt, fov_deg):
                continue
            dx = float(tgt.x - s.x)
            dy = float(tgt.y - s.y)
            dz = float(tgt.z - s.z)
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist < best_dist:
                best_dist = dist
                best = tgt
        if best is None:
            return None
        return int(best.id), best

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

    def _flight_mode_for(self, simv: SimVehicle) -> int:
        forced = self._forced_commands.get(simv.label)
        if forced and "flight_mode" in forced:
            return int(forced.get("flight_mode") or 0)
        tracking = self._tracking_state.get(simv.label)
        if tracking is not None and tracking.stage >= 1:
            return 9
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

    def _on_mission_for(self, simv: SimVehicle) -> int:
        if simv.airframe != "uav":
            return 0
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
                "heading": float(getattr(s, "yaw", 0.0)),
            },
            "fuel": float(fuel),
            "health": int(health_value),
            "lastSignalTime": int(timestamp),
        }
        if simv.airframe == "uav":
            current_wp = 0
            tracking = self._tracking_state.get(simv.label)
            forced = self._forced_commands.get(simv.label)
            if tracking is not None and tracking.saved_wp_id is not None:
                current_wp = int(tracking.saved_wp_id)
            else:
                tgt = simv.controller.current_target()
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
            target_id = 0
            if tracking is not None and tracking.stage >= 1 and tracking.target.alive:
                target_id = self._assign_0402_target_id(tracking.target)
            payload_health = _coerce_int(overrides.get("payloadHealth", 1), 1)
            fuel_warn = _coerce_int(overrides.get("fuelWarning", 0), 0)
            flight_mode = int(self._flight_mode_for(simv))
            on_mission = int(self._on_mission_for(simv))
            loiter_coord = None
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
                "currentWaypointID": {"waypointID": int(current_wp)},
                "flightMode": int(flight_mode),
                "targetFollowing": {"targetID": int(target_id)},
                "payloadHealth": int(payload_health),
                "fuelWarning": int(fuel_warn),
                "onMission": int(on_mission),
            }
            if loiter_coord is not None:
                agent["unmannedInfo"]["loiterCoordinate"] = loiter_coord
            filming_prop = self._filming_props.get(simv.label)
            if isinstance(filming_prop, dict):
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
                sensor_info: dict[str, Any] = {}
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
                if sensor_info:
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
        geo = self.geo
        if geo is None:
            return None
        try:
            lon, lat = geo.xy_to_lonlat(float(target.x), float(target.y))
        except Exception:
            return None
        coord = {"latitude": float(lat), "longitude": float(lon), "altitude": float(target.z)}
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

    def _push_0402_updates_once(self) -> None:
        if self.geo is None:
            return
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

    def _update_target_info_destroyed(self, target: GroundTarget, watcher_id: int | None = None) -> None:
        try:
            target_id = int(getattr(target, "id", 0))
        except Exception:
            return
        if target_id <= 0:
            return
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
        for key, entry in list(target_list.items()):
            if not isinstance(entry, dict):
                continue
            try:
                entry_id = int(entry.get("targetID", -1))
            except Exception:
                continue
            if entry_id != target_id:
                continue
            entry = dict(entry)
            entry["isDestroyed"] = True
            entry["threat"] = 0
            entry["targetInFrame"] = False
            if watcher_id is not None and entry.get("watcherID") is None:
                entry["watcherID"] = watcher_id
            target_list[str(key)] = entry
            updated = True
        if not updated:
            coord = None
            if self.geo is not None:
                try:
                    lon, lat = self.geo.xy_to_lonlat(float(target.x), float(target.y))
                    coord = {"latitude": float(lat), "longitude": float(lon), "altitude": float(target.z)}
                except Exception:
                    coord = None
            entry: dict[str, object] = {
                "targetID": int(target_id),
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
            key = f"{target_id}-{watcher_id}" if watcher_id is not None else str(target_id)
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
        if simv.label in self._tracking_state:
            return
        owner = self._tracking_target_owner.get(int(target.id))
        if owner and owner != simv.label:
            return
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

    def _update_tracking(self, simv: SimVehicle, dt: float) -> None:
        label = simv.label
        state = self._tracking_state.get(label)
        override = self._tracking_overrides.get(label)
        if override and (state is None or override.get("seq") != state.override_seq):
            if self._apply_tracking_override(simv, state, override):
                return
        if state is not None:
            if self._target_is_ignored_in_info(int(state.target_id)):
                self._stop_tracking(simv, advance=False)
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
        current = simv.controller.current_target()
        filming_prop = current.filming if current else None
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
        if not self.auto_track_always:
            if not (isinstance(filming_prop, dict) and int(filming_prop.get("operationMode") or 0) == 3):
                return
        fov_deg = self._resolve_tracking_fov(filming_prop if isinstance(filming_prop, dict) else None)
        found = self._find_detected_target(simv, fov_deg)
        if found is None:
            return
        target_id, target = found
        self._start_tracking(simv, target_id, target, filming_prop, fov_deg)

    def _formation_leader(self, simv: SimVehicle) -> SimVehicle | None:
        spec = simv.formation
        if spec is None:
            return None
        leader_label = _agent_label(int(spec.leader_id))
        if not leader_label:
            return None
        return self.vehicles.get(leader_label)

    def _is_formation_leader(self, simv: SimVehicle) -> bool:
        try:
            leader_id = int(simv.aircraft_id)
        except Exception:
            return False
        for other in self.vehicles.values():
            spec = other.formation
            if spec is None or spec.is_leader:
                continue
            try:
                if int(spec.leader_id) == leader_id:
                    return True
            except Exception:
                continue
        return False

    def _update_formation_target(self, simv: SimVehicle) -> None:
        spec = simv.formation
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
        try:
            simv.controller.speed_target = float(getattr(s, "u", simv.controller.speed_target))
        except Exception:
            pass

    def _update_filming_target(self, simv: SimVehicle, dt: float) -> None:
        geo = self.geo
        if geo is None:
            return
        label = simv.label
        tracking = self._tracking_state.get(label)
        if tracking is not None and tracking.target.alive:
            self._line_search_state[label] = None
            self._filming_props[label] = tracking.filming_prop
            self._filming_wp_ids[label] = tracking.saved_wp_id
            self._filming_targets[label] = (
                float(tracking.target.x),
                float(tracking.target.y),
                float(tracking.target.z),
            )
            return
        controller = simv.controller
        tgt = controller.current_target()
        filming_prop = tgt.filming if tgt else None
        current_wp_id = tgt.wp_id if tgt else None

        if filming_prop is None:
            self._line_search_state[label] = None
            self._filming_props[label] = None
            self._filming_wp_ids[label] = current_wp_id
            if getattr(controller, "is_loitering", False):
                center = getattr(controller, "loiter_center", None)
                if isinstance(center, (tuple, list)) and len(center) == 3:
                    self._filming_targets[label] = (
                        float(center[0]),
                        float(center[1]),
                        float(center[2]),
                    )
                    return
            self._filming_targets[label] = self._default_downward_target(simv.vehicle)
            return

        handler = self._get_operation_handler(filming_prop.get("operationMode"))
        if handler is None:
            self._line_search_state[label] = None
            self._filming_props[label] = filming_prop
            self._filming_wp_ids[label] = current_wp_id
            self._filming_targets[label] = self._default_downward_target(simv.vehicle)
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
            self._filming_targets[label] = self._default_downward_target(simv.vehicle)
            return

        self._filming_props[label] = filming_prop
        self._filming_wp_ids[label] = current_wp_id
        self._filming_targets[label] = result.target or self._default_downward_target(simv.vehicle)
        self._line_search_state[label] = result.state
        if result.reset_debug or result.debug is not None:
            self._line_search_debug[label] = result.debug
        try:
            if (
                getattr(simv.controller, "is_loitering", False)
                and isinstance(filming_prop, dict)
                and int(filming_prop.get("operationMode") or 0) == 1
                and result.target is not None
            ):
                center = result.target
                simv.controller.loiter_center = (float(center[0]), float(center[1]), float(center[2]))
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

    def _build_vehicles(self, paths: list[PathDefinition]) -> None:
        geo = self.geo
        if geo is None:
            return
        self._tracking_state = {}
        self._reset_0402_state()
        self._attack_holds = {}

        uav_db = Path(__file__).resolve().parent / "controllers" / "uav_pid_db.json"
        lah_db = Path(__file__).resolve().parent / "controllers" / "lah_pid_db.json"

        vehicles: dict[str, SimVehicle] = {}
        self._filming_props = {}
        self._filming_targets = {}
        self._filming_wp_ids = {}
        self._line_search_state = {}
        self._line_search_debug = {}
        self._history.clear()
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
                        hover_time=wp.get("hover_time"),
                        loiter=wp.get("loiter"),
                        filming=wp.get("filming"),
                        attack=wp.get("attack") if isinstance(wp.get("attack"), dict) else None,
                        input_mission_id=wp.get("input_mission_id"),
                        individual_mission_id=wp.get("individual_mission_id"),
                        path_id=wp.get("path_id") or path.path_id,
                        pass_type=wp.get("pass_type"),
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
                vehicle = UAV(UAVParams())
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
        for tgt in self.targets:
            try:
                if int(getattr(tgt, "id", 0)) == target_id:
                    return tgt
            except Exception:
                continue
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
        mapped = self._resolve_actual_target_from_info(target_id)
        if mapped is not None:
            return mapped
        virtual = self._virtual_target_from_info(target_id)
        if virtual is not None:
            return virtual
        return None

    def _resolve_tracking_target(self, target_id: int) -> GroundTarget | None:
        target = self._resolve_attack_target(target_id)
        if target is not None:
            return target
        return self._virtual_target_from_info(target_id)

    def _resolve_actual_target_from_info(self, target_id: int) -> GroundTarget | None:
        if self.geo is None or not self.targets:
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
            if entry_id != int(target_id):
                continue
            if entry.get("isDestroyed") is True:
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
            if entry_id != int(target_id):
                continue
            if entry.get("isDestroyed") is True:
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
                self._update_tracking(simv, dt)
                simv.controller.update(dt)
                if simv.airframe == "uav":
                    try:
                        ctrl = simv.controller
                        if (
                            getattr(ctrl, "just_advanced", False)
                            and getattr(ctrl, "advance_reason", None) == "loiter"
                        ):
                            self._pulse_on_mission(simv.label)
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

    def _build_frame(
        self,
        *,
        geo: GeoConverter,
        vehicles: list[SimVehicle],
        targets: list[GroundTarget] | None = None,
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
                "heading": float(getattr(s, "yaw", 0.0)),
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
                on_mission = int(self._on_mission_for(simv))
                entry["flightMode"] = flight_mode
                entry["onMission"] = on_mission
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
            filming_prop = self._filming_props.get(simv.label)
            if isinstance(filming_prop, dict):
                entry["filmingMode"] = filming_prop.get("operationMode")
                fov = filming_prop.get("fieldOfView")
                if isinstance(fov, (int, float)):
                    entry["filmingFov"] = float(fov)
            if simv.label in self._filming_wp_ids:
                entry["filmingWpId"] = self._filming_wp_ids.get(simv.label)
            frame["vehicles"][simv.label] = entry
        for tgt in targets_list:
            try:
                frame["targets"].append(self._target_to_dict(tgt, geo))
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
        frame = self._build_frame(
            geo=geo,
            vehicles=vehicles,
            targets=targets,
            sim_time=self.sim_time,
            step_count=self.step_count,
        )
        self._history.append(frame)

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
            while accum >= dt:
                with self._lock:
                    if not (self.running and not self.paused and self.vehicles):
                        accum = 0.0
                        break
                    self._step_once(dt)
                    self.sim_time += dt
                    self.step_count += 1
                    self._record_history()
                    self._maybe_push_0401()
                    self._maybe_push_0402()
                accum -= dt
                advanced += 1

            if advanced == 0:
                with self._lock:
                    if self.running and not self.paused and self.vehicles:
                        self._maybe_push_0401()
                self._shutdown.wait(0.005)

    def build_snapshot(self, *, since_step: int | None = None) -> dict:
        with self._lock:
            geo = self.geo
            vehicles = list(self.vehicles.values())
            targets = list(self.targets)
            pending_targets = list(self._pending_targets)
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
            "simTime": sim_time,
            "step": step_count,
            "vehicles": {},
            "targets": [],
            "projectiles": [],
            "effects": [],
            "events0402": events_0402,
        }
        if error:
            payload["error"] = error

        if geo is None:
            if pending_targets:
                payload["targets"] = pending_targets
            return payload

        latest_frame = self._build_frame(
            geo=geo,
            vehicles=vehicles,
            targets=targets,
            sim_time=sim_time,
            step_count=step_count,
        )
        payload["vehicles"] = latest_frame["vehicles"]
        payload["targets"] = latest_frame.get("targets", [])
        payload["projectiles"] = latest_frame.get("projectiles", [])
        payload["effects"] = latest_frame.get("effects", [])

        if since_step is None:
            return payload

        history_frames = [frame for frame in history if frame.get("step", -1) > since_step]
        events_since = [event for event in events_0402 if event.get("step", -1) > since_step]
        payload["events0402"] = events_since
        return {
            "ok": True,
            "history": history_frames,
            "latest": payload,
            "events0402": events_since,
        }
