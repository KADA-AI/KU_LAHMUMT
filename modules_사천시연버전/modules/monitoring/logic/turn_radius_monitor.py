from __future__ import annotations

import json
import math
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from modules.common import db_paths
from modules.mission_planning.MissionPlanner.runtime_settings import get_runtime_float
from modules.monitoring.logic.replan_runtime_settings import get_path_deviation_settings


TRACKED_AIRCRAFT_IDS = (4, 5, 6)
EARTH_RADIUS_M = 6378137.0
GRAVITY_MPS2 = 9.80665
MIN_SPEED_MPS = 30.0
MAX_SPEED_MPS = 50.0
PATH_SAMPLE_STEP_M = 4.0
MIN_HEADING_MOVE_M = 2.0
COORD_HEADING_FALLBACK_MIN_DELTA_DEG = 25.0
COORD_HEADING_FALLBACK_MAX_DELTA_DEG = 120.0
COORD_HEADING_FALLBACK_MAX_TURN_DPS = 15.0
HISTORY_RETENTION_EXTRA_S = 15.0
MAX_HISTORY_HARD_CAP = 4096
MAX_PATH_POINTS = 2400
TURN_GAP_RESET_S = 2.5
WAYPOINT_SCAN_INTERVAL_S = 0.5
NEXT_MISSION_ENTRY_LEAD_TIME_S = 5.0
ALT_WAYPOINT_ID_BASE = 900000
WAYPOINT_COORD_OVERRIDE_NEAR_M = 1200.0
WAYPOINT_COORD_OVERRIDE_MARGIN_M = 1000.0
_SIM_DYNAMICS_PROFILE_CACHE_SIG: tuple[int, int] | None = None
_SIM_DYNAMICS_PROFILE_CACHE_SCALE = 1.0
_ADAPTIVE_STATE_FILENAME = "path_deviation_adaptive_state.json"


def _path_deviation_config() -> dict[str, Any]:
    return get_path_deviation_settings()


def _project_root() -> Path:
    current = Path(__file__).resolve()
    for parent in current.parents:
        if (parent / "run.py").exists():
            return parent
    return current.parents[3]


def _sim_reference_turn_radius_scale() -> float:
    global _SIM_DYNAMICS_PROFILE_CACHE_SIG
    global _SIM_DYNAMICS_PROFILE_CACHE_SCALE

    profile_path = _project_root() / "modules" / "sim" / "runtime" / "controllers" / "uav_dynamics_profile.json"
    try:
        stat = profile_path.stat()
        sig = (int(stat.st_mtime_ns), int(stat.st_size))
    except Exception:
        return 1.0
    if _SIM_DYNAMICS_PROFILE_CACHE_SIG == sig:
        return float(_SIM_DYNAMICS_PROFILE_CACHE_SCALE)

    scale = 1.0
    try:
        payload = json.loads(profile_path.read_text(encoding="utf-8"))
        if isinstance(payload, dict) and bool(payload.get("enabled", False)):
            params = payload.get("params") if isinstance(payload.get("params"), dict) else payload
            parsed = _coerce_float((params or {}).get("reference_turn_radius_scale"))
            if parsed is not None and math.isfinite(float(parsed)) and float(parsed) > 0.0:
                scale = max(0.05, min(float(parsed), 5.0))
    except Exception:
        scale = 1.0

    _SIM_DYNAMICS_PROFILE_CACHE_SIG = sig
    _SIM_DYNAMICS_PROFILE_CACHE_SCALE = float(scale)
    return float(scale)


def _turn_radius_scale(cfg: dict[str, Any] | None = None) -> float:
    data = cfg if isinstance(cfg, dict) else _path_deviation_config()
    explicit = _coerce_float(data.get("turn_radius_scale"))
    if explicit is not None and math.isfinite(float(explicit)) and explicit > 0.0:
        return max(0.05, min(float(explicit), 5.0))
    sync_raw = data.get("sync_sim_dynamics_turn_radius", True)
    if isinstance(sync_raw, str) and sync_raw.strip().lower() in {"0", "false", "no", "off"}:
        return 1.0
    if sync_raw is False:
        return 1.0
    return _sim_reference_turn_radius_scale()


def _history_retention_s(cfg: dict[str, Any] | None = None) -> float:
    data = cfg if isinstance(cfg, dict) else _path_deviation_config()
    return float(data.get("spiral_window_s", 75.0)) + float(HISTORY_RETENTION_EXTRA_S)


def _speed_radius_table(cfg: dict[str, Any] | None = None) -> tuple[tuple[float, float], ...]:
    data = cfg if isinstance(cfg, dict) else _path_deviation_config()
    scale = _turn_radius_scale(data)
    return (
        (30.0, float(data.get("turn_radius_30_m", 340.0)) * float(scale)),
        (40.0, float(data.get("turn_radius_40_m", 450.0)) * float(scale)),
        (50.0, float(data.get("turn_radius_50_m", 560.0)) * float(scale)),
    )


def interpolate_turn_radius(speed_mps: float) -> float:
    speed_radius_table = _speed_radius_table()
    speed = max(MIN_SPEED_MPS, min(MAX_SPEED_MPS, float(speed_mps)))
    if speed <= speed_radius_table[0][0]:
        return float(speed_radius_table[0][1])
    if speed >= speed_radius_table[-1][0]:
        return float(speed_radius_table[-1][1])
    for left, right in zip(speed_radius_table[:-1], speed_radius_table[1:]):
        s0, r0 = left
        s1, r1 = right
        if s0 <= speed <= s1:
            alpha = (speed - s0) / max(1e-6, s1 - s0)
            return r0 + ((r1 - r0) * alpha)
    return float(speed_radius_table[1][1])


def _effective_turn_radius(
    *,
    ideal_radius_m: float | None,
    actual_radius_m: float | None,
    cfg: dict[str, Any] | None = None,
) -> float | None:
    ideal = _coerce_float(ideal_radius_m)
    actual = _coerce_float(actual_radius_m)
    if actual is None or not math.isfinite(float(actual)) or actual <= 1.0:
        return ideal if ideal is not None and ideal > 1.0 else None
    if ideal is None or not math.isfinite(float(ideal)) or ideal <= 1.0:
        return actual

    data = cfg if isinstance(cfg, dict) else _path_deviation_config()
    min_factor = max(0.05, _coerce_float(data.get("actual_radius_min_factor")) or 0.55)
    max_factor = max(min_factor, _coerce_float(data.get("actual_radius_max_factor")) or 2.25)
    if (float(ideal) * min_factor) <= float(actual) <= (float(ideal) * max_factor):
        return float(actual)
    return float(ideal)


def _next_collab_turn_radius_scale(cfg: dict[str, Any] | None = None) -> float:
    _ = cfg
    try:
        scale = float(get_runtime_float("next_collab_turn_radius_scale", 1.20))
    except Exception:
        scale = 1.20
    return max(0.1, float(scale))


def _coerce_float(value: object | None) -> float | None:
    try:
        if value is None:
            return None
        return float(value)
    except Exception:
        return None


def _coerce_int(value: object | None) -> int | None:
    try:
        if value is None:
            return None
        return int(value)
    except Exception:
        return None


def _coerce_bool(value: object | None, default: bool = False) -> bool:
    if isinstance(value, str):
        text = value.strip().lower()
        if text in {"1", "true", "yes", "y", "on"}:
            return True
        if text in {"0", "false", "no", "n", "off", ""}:
            return False
    if value is None:
        return bool(default)
    try:
        return bool(value)
    except Exception:
        return bool(default)


def _clamp_float(value: float, low: float, high: float) -> float:
    return max(float(low), min(float(high), float(value)))


def _first_present(mapping: dict[str, Any], *keys: str) -> object | None:
    for key in keys:
        if key in mapping:
            return mapping.get(key)
    return None


def _wrap_pi(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def _unwrap_angles(angles_rad: list[float]) -> list[float]:
    if not angles_rad:
        return []
    out = [angles_rad[0]]
    prev_raw = angles_rad[0]
    prev_unwrapped = angles_rad[0]
    for raw in angles_rad[1:]:
        delta = _wrap_pi(raw - prev_raw)
        prev_unwrapped += delta
        out.append(prev_unwrapped)
        prev_raw = raw
    return out


def _linear_slope(xs: list[float], ys: list[float]) -> float | None:
    if len(xs) < 2 or len(xs) != len(ys):
        return None
    x_mean = sum(xs) / len(xs)
    y_mean = sum(ys) / len(ys)
    numer = 0.0
    denom = 0.0
    for x_val, y_val in zip(xs, ys):
        dx = x_val - x_mean
        numer += dx * (y_val - y_mean)
        denom += dx * dx
    if abs(denom) < 1e-9:
        return None
    return numer / denom


def _heading_deg_to_math_rad(heading_deg: float) -> float:
    # 0401 heading follows the navigation convention:
    # 0=north, 90=east, 180=south, 270=west.
    # Internal XY math uses 0=+X(east), counter-clockwise positive.
    return math.radians(90.0 - float(heading_deg))


def _math_rad_to_heading_deg(angle_rad: float) -> float:
    return (90.0 - math.degrees(float(angle_rad))) % 360.0


def _angle_delta_deg(left_rad: float, right_rad: float) -> float:
    return abs(math.degrees(_wrap_pi(float(left_rad) - float(right_rad))))


def _latlon_to_local_m(
    lat_deg: float,
    lon_deg: float,
    ref_lat_deg: float,
    ref_lon_deg: float,
) -> tuple[float, float]:
    lat_rad = math.radians(lat_deg)
    lon_rad = math.radians(lon_deg)
    ref_lat_rad = math.radians(ref_lat_deg)
    ref_lon_rad = math.radians(ref_lon_deg)
    x_m = (lon_rad - ref_lon_rad) * EARTH_RADIUS_M * math.cos((lat_rad + ref_lat_rad) * 0.5)
    y_m = (lat_rad - ref_lat_rad) * EARTH_RADIUS_M
    return x_m, y_m


def _local_m_to_latlon(
    x_m: float,
    y_m: float,
    ref_lat_deg: float,
    ref_lon_deg: float,
) -> tuple[float, float]:
    ref_lat_rad = math.radians(ref_lat_deg)
    ref_lon_rad = math.radians(ref_lon_deg)
    lat_rad = ref_lat_rad + (float(y_m) / EARTH_RADIUS_M)
    mean_lat_cos = math.cos((lat_rad + ref_lat_rad) * 0.5)
    if abs(mean_lat_cos) < 1e-9:
        mean_lat_cos = 1e-9 if mean_lat_cos >= 0.0 else -1e-9
    lon_rad = ref_lon_rad + (float(x_m) / (EARTH_RADIUS_M * mean_lat_cos))
    return math.degrees(lat_rad), math.degrees(lon_rad)


def _distance_m(left: tuple[float, float], right: tuple[float, float]) -> float:
    return math.hypot(left[0] - right[0], left[1] - right[1])


def _haversine_distance_m(
    left_lat_deg: float,
    left_lon_deg: float,
    right_lat_deg: float,
    right_lon_deg: float,
) -> float:
    left_lat_rad = math.radians(float(left_lat_deg))
    left_lon_rad = math.radians(float(left_lon_deg))
    right_lat_rad = math.radians(float(right_lat_deg))
    right_lon_rad = math.radians(float(right_lon_deg))
    dlat = right_lat_rad - left_lat_rad
    dlon = right_lon_rad - left_lon_rad
    a = (
        math.sin(dlat * 0.5) ** 2
        + math.cos(left_lat_rad) * math.cos(right_lat_rad) * (math.sin(dlon * 0.5) ** 2)
    )
    c = 2.0 * math.atan2(math.sqrt(max(0.0, a)), math.sqrt(max(1e-12, 1.0 - a)))
    return float(EARTH_RADIUS_M * c)


def _adaptive_enabled(cfg: dict[str, Any] | None = None) -> bool:
    data = cfg if isinstance(cfg, dict) else _path_deviation_config()
    return _coerce_bool(data.get("adaptive_enabled"), True)


def _adaptive_threshold_scale_from_radius_scale(
    cfg: dict[str, Any],
    radius_scale: float | None,
    sample_count: int,
) -> float:
    if not _adaptive_enabled(cfg):
        return 1.0
    scale = _coerce_float(radius_scale)
    if scale is None or not math.isfinite(float(scale)) or scale <= 0.0:
        return 1.0
    warmup_samples = max(1, int(cfg.get("adaptive_warmup_samples", 4)))
    if int(sample_count) < warmup_samples:
        blend = max(0.0, min(1.0, float(sample_count) / float(warmup_samples)))
        scale = 1.0 + ((float(scale) - 1.0) * blend)
    if scale >= 1.0:
        raw_gain = _coerce_float(cfg.get("adaptive_threshold_gain"))
        gain = _clamp_float(0.65 if raw_gain is None else raw_gain, 0.0, 1.0)
    else:
        raw_gain = _coerce_float(cfg.get("adaptive_threshold_down_gain"))
        gain = _clamp_float(0.35 if raw_gain is None else raw_gain, 0.0, 1.0)
    return max(0.5, 1.0 + ((float(scale) - 1.0) * gain))


def _adaptive_state_path() -> Path:
    return db_paths.get_db_subpath("DSS_Internal", _ADAPTIVE_STATE_FILENAME)


def _load_adaptive_aircraft_state(aircraft_id: int) -> dict[str, Any]:
    try:
        path = _adaptive_state_path()
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    if not isinstance(payload, dict):
        return {}
    aircraft_map = payload.get("aircraft")
    if not isinstance(aircraft_map, dict):
        return {}
    item = aircraft_map.get(str(int(aircraft_id)))
    return dict(item) if isinstance(item, dict) else {}


def _save_adaptive_aircraft_state(aircraft_id: int, state: dict[str, Any]) -> None:
    path = _adaptive_state_path()
    payload: dict[str, Any] = {}
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        payload = {}
    if not isinstance(payload, dict):
        payload = {}
    aircraft_map = payload.get("aircraft")
    if not isinstance(aircraft_map, dict):
        aircraft_map = {}
        payload["aircraft"] = aircraft_map
    payload["version"] = 1
    payload["updatedAtUnix"] = float(time.time())
    aircraft_map[str(int(aircraft_id))] = dict(state or {})
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp.replace(path)


def project_coordinate_forward(
    coordinate: dict[str, float] | None,
    *,
    speed_mps: float | None,
    lead_s: float,
    raw_heading_deg: float | None = None,
    heading_rad: float | None = None,
) -> tuple[dict[str, float] | None, float | None]:
    if not isinstance(coordinate, dict):
        return None, None
    latitude = _coerce_float(coordinate.get("latitude"))
    longitude = _coerce_float(coordinate.get("longitude"))
    if latitude is None or longitude is None:
        return None, None

    lead_time_s = max(0.0, _coerce_float(lead_s) or 0.0)
    speed = _coerce_float(speed_mps)
    projected_heading_rad = _coerce_float(heading_rad)
    if projected_heading_rad is None and raw_heading_deg is not None:
        projected_heading_rad = _heading_deg_to_math_rad(float(raw_heading_deg))

    projected: dict[str, float] = {
        "latitude": float(latitude),
        "longitude": float(longitude),
    }
    altitude = _coerce_float(coordinate.get("altitude"))
    if altitude is not None:
        projected["altitude"] = float(altitude)

    if (
        lead_time_s <= 0.0
        or speed is None
        or speed <= 0.0
        or projected_heading_rad is None
    ):
        return projected, 0.0

    distance_m = float(speed) * float(lead_time_s)
    dx_m = math.cos(float(projected_heading_rad)) * distance_m
    dy_m = math.sin(float(projected_heading_rad)) * distance_m
    lat_deg, lon_deg = _local_m_to_latlon(dx_m, dy_m, float(latitude), float(longitude))
    projected["latitude"] = float(lat_deg)
    projected["longitude"] = float(lon_deg)
    return projected, float(lead_time_s)


def _extract_current_waypoint_id(state: dict[str, Any]) -> int | None:
    containers = [
        state.get("unmannedInfo"),
        state.get("UnmannedInfo"),
        state,
    ]
    for container in containers:
        if not isinstance(container, dict):
            continue
        value = _first_present(container, "currentWaypointID", "CurrentWaypointID", "currentWaypointId")
        if value is None:
            continue
        if isinstance(value, dict):
            waypoint_id = _coerce_int(_first_present(value, "waypointID", "WaypointID", "waypointId"))
        else:
            waypoint_id = _coerce_int(value)
        if waypoint_id is not None and waypoint_id > 0:
            return waypoint_id
    return None


def _extract_flying(state: dict[str, Any]) -> int | None:
    containers = [
        state.get("unmannedInfo"),
        state.get("UnmannedInfo"),
        state.get("flightMode"),
        state.get("FlightMode"),
        state,
    ]
    for container in containers:
        if not isinstance(container, dict):
            continue
        value = _first_present(container, "flying", "Flying")
        parsed = _coerce_int(value)
        if parsed is not None:
            return parsed
    return None


@dataclass(frozen=True)
class TrackPoint:
    timestamp_s: float
    wall_time_s: float
    x_m: float
    y_m: float
    altitude_m: float | None
    speed_mps: float | None
    raw_heading_deg: float | None
    heading_rad: float | None
    current_waypoint_id: int | None


@dataclass(frozen=True)
class IdealTurnState:
    timestamp_s: float
    x_m: float
    y_m: float
    heading_rad: float
    radius_m: float
    turn_sign: int


@dataclass(frozen=True)
class AircraftTurnView:
    aircraft_id: int
    label: str
    has_data: bool
    actual_points: list[tuple[float, float]]
    ideal_points: list[tuple[float, float]]
    position_m: tuple[float, float] | None
    position_coordinate: dict[str, float] | None
    heading_rad: float | None
    raw_heading_deg: float | None
    speed_mps: float | None
    turn_rate_dps: float | None
    ideal_radius_m: float | None
    actual_radius_m: float | None
    turn_sign: int
    turn_circle_center_m: tuple[float, float] | None
    turn_circle_radius_m: float | None
    current_waypoint_id: int | None
    current_waypoint_pass_type: int | None
    flying: int | None
    current_waypoint_position_m: tuple[float, float] | None
    alternate_waypoint_id: int | None
    alternate_waypoint_position_m: tuple[float, float] | None
    alternate_waypoint_coordinate: dict[str, float] | None
    alternate_waypoint_eta_s: float | None
    predicted_entry_coordinate: dict[str, float] | None
    predicted_entry_eta_s: float | None
    current_waypoint_is_loiter: bool
    spiral_state: str
    spiral_score: float | None
    spiral_angle_deg: float | None
    spiral_radius_m: float | None
    spiral_radial_span_m: float | None
    age_s: float | None
    is_turning: bool
    adaptive_radius_scale: float | None = None
    adaptive_threshold_scale: float | None = None
    adaptive_expected_roll_deg: float | None = None
    adaptive_sample_count: int = 0
    adaptive_last_actual_radius_m: float | None = None
    adaptive_last_ideal_radius_m: float | None = None
    adaptive_last_speed_mps: float | None = None


def is_path_deviation_turn_warning_view(view: AircraftTurnView | None) -> bool:
    if not isinstance(view, AircraftTurnView):
        return False
    if not view.has_data or not view.is_turning:
        return False
    if str(view.spiral_state or "none") != "warning":
        return False
    current_waypoint_id = _coerce_int(view.current_waypoint_id)
    if current_waypoint_id is None or current_waypoint_id <= 0:
        return False
    alt_waypoint_id = _coerce_int(view.alternate_waypoint_id)
    if alt_waypoint_id is None or alt_waypoint_id <= 0:
        return False
    coordinate = view.alternate_waypoint_coordinate
    if not isinstance(coordinate, dict):
        return False
    return (
        _coerce_float(coordinate.get("latitude")) is not None
        and _coerce_float(coordinate.get("longitude")) is not None
    )


@dataclass(frozen=True)
class WaypointInfo:
    latitude: float
    longitude: float
    waypoint_pass_type: int | None
    has_loiter_property: bool


@dataclass(frozen=True)
class IndexedWaypoint:
    waypoint_id: int
    path_id: int
    order_index: int
    latitude: float
    longitude: float
    waypoint_pass_type: int | None
    has_loiter_property: bool
    is_done: bool

    def to_waypoint_info(self) -> WaypointInfo:
        return WaypointInfo(
            latitude=float(self.latitude),
            longitude=float(self.longitude),
            waypoint_pass_type=self.waypoint_pass_type,
            has_loiter_property=bool(self.has_loiter_property),
        )


class WaypointLookupCache:
    def __init__(self) -> None:
        self._waypoints: dict[int, dict[int, WaypointInfo]] = {
            aircraft_id: {} for aircraft_id in TRACKED_AIRCRAFT_IDS
        }
        self._path_waypoints: dict[int, dict[int, list[IndexedWaypoint]]] = {
            aircraft_id: {} for aircraft_id in TRACKED_AIRCRAFT_IDS
        }
        self._file_mtimes: dict[Path, float] = {}
        self._last_scan_s = 0.0

    def resolve(
        self,
        aircraft_id: int,
        waypoint_id: int | None,
        *,
        current_coordinate: dict[str, float] | None = None,
    ) -> tuple[int | None, WaypointInfo | None]:
        raw_waypoint_id = _coerce_int(waypoint_id)
        if raw_waypoint_id is not None and raw_waypoint_id <= 0:
            raw_waypoint_id = None
        aircraft_map = self._waypoints.setdefault(int(aircraft_id), {})
        inferred = None
        if raw_waypoint_id is not None:
            found = aircraft_map.get(int(raw_waypoint_id))
            if found is not None:
                return int(raw_waypoint_id), found
        self._scan(force=False)
        if raw_waypoint_id is not None:
            found = aircraft_map.get(int(raw_waypoint_id))
            if found is not None:
                return int(raw_waypoint_id), found
        inferred = self._infer_from_coordinate(int(aircraft_id), current_coordinate)
        if inferred is not None:
            return int(inferred.waypoint_id), inferred.to_waypoint_info()
        self._scan(force=True)
        if raw_waypoint_id is not None:
            found = aircraft_map.get(int(raw_waypoint_id))
            if found is not None:
                return int(raw_waypoint_id), found
        inferred = self._infer_from_coordinate(int(aircraft_id), current_coordinate)
        if inferred is not None:
            return int(inferred.waypoint_id), inferred.to_waypoint_info()
        return raw_waypoint_id, None

    def _select_preferred_candidate(
        self,
        *,
        raw_waypoint_id: int,
        raw_info: WaypointInfo,
        inferred: IndexedWaypoint | None,
        current_coordinate: dict[str, float] | None,
    ) -> tuple[int, WaypointInfo]:
        if inferred is None:
            return int(raw_waypoint_id), raw_info
        raw_distance_m = self._distance_to_waypoint_info(current_coordinate, raw_info)
        inferred_distance_m = self._distance_to_indexed_waypoint(current_coordinate, inferred)
        if (
            raw_distance_m is not None
            and inferred_distance_m is not None
            and inferred_distance_m <= WAYPOINT_COORD_OVERRIDE_NEAR_M
            and raw_distance_m >= (inferred_distance_m + WAYPOINT_COORD_OVERRIDE_MARGIN_M)
        ):
            return int(inferred.waypoint_id), inferred.to_waypoint_info()
        return int(raw_waypoint_id), raw_info

    def _scan(self, force: bool) -> None:
        now = time.monotonic()
        if not force and (now - self._last_scan_s) < WAYPOINT_SCAN_INTERVAL_S:
            return
        self._last_scan_s = now

        try:
            folder = db_paths.get_db_subpath("FlightPath")
        except Exception:
            return
        if not folder.exists():
            return

        for path in folder.glob("*.json"):
            try:
                mtime = path.stat().st_mtime
            except Exception:
                continue
            if self._file_mtimes.get(path) == mtime:
                continue
            self._file_mtimes[path] = mtime
            self._index_file(path)

    def _index_file(self, path: Path) -> None:
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            return
        if not isinstance(payload, dict):
            return
        aircraft_id = _coerce_int(_first_present(payload, "aircraftID", "AircraftID"))
        if aircraft_id not in TRACKED_AIRCRAFT_IDS:
            return
        path_id = _coerce_int(_first_present(payload, "pathID", "PathID"))
        aircraft_map = self._waypoints.setdefault(int(aircraft_id), {})
        indexed_waypoints: list[IndexedWaypoint] = []
        for key in ("waypointList", "WaypointList", "lahWaypointList", "LahWaypointList"):
            items = payload.get(key)
            if not isinstance(items, list):
                continue
            for order_index, waypoint in enumerate(items):
                if not isinstance(waypoint, dict):
                    continue
                waypoint_id = _coerce_int(_first_present(waypoint, "waypointID", "WaypointID"))
                coordinate = _first_present(waypoint, "coordinate", "Coordinate")
                if waypoint_id is None or not isinstance(coordinate, dict):
                    continue
                latitude = _coerce_float(_first_present(coordinate, "latitude", "Latitude"))
                longitude = _coerce_float(_first_present(coordinate, "longitude", "Longitude"))
                if latitude is None or longitude is None:
                    continue
                waypoint_pass_type = _coerce_int(_first_present(waypoint, "waypointPassType", "WaypointPassType"))
                has_loiter_property = isinstance(_first_present(waypoint, "loiterProperty", "LoiterProperty"), dict)
                aircraft_map[int(waypoint_id)] = WaypointInfo(
                    latitude=float(latitude),
                    longitude=float(longitude),
                    waypoint_pass_type=waypoint_pass_type,
                    has_loiter_property=has_loiter_property,
                )
                if path_id is not None and path_id > 0:
                    indexed_waypoints.append(
                        IndexedWaypoint(
                            waypoint_id=int(waypoint_id),
                            path_id=int(path_id),
                            order_index=int(order_index),
                            latitude=float(latitude),
                            longitude=float(longitude),
                            waypoint_pass_type=waypoint_pass_type,
                            has_loiter_property=bool(has_loiter_property),
                            is_done=bool(waypoint.get("isDone")),
                        )
                    )
            if indexed_waypoints:
                break
        if path_id is not None and path_id > 0 and indexed_waypoints:
            self._path_waypoints.setdefault(int(aircraft_id), {})[int(path_id)] = indexed_waypoints

    def _infer_from_coordinate(
        self,
        aircraft_id: int,
        current_coordinate: dict[str, float] | None,
    ) -> IndexedWaypoint | None:
        if not isinstance(current_coordinate, dict):
            return None
        latitude = _coerce_float(current_coordinate.get("latitude"))
        longitude = _coerce_float(current_coordinate.get("longitude"))
        if latitude is None or longitude is None:
            return None
        aircraft_paths = self._path_waypoints.setdefault(int(aircraft_id), {})
        if not aircraft_paths:
            return None

        best_item: IndexedWaypoint | None = None
        best_score: tuple[float, int, int, int, int] | None = None
        for path_id, waypoints in aircraft_paths.items():
            for item in waypoints:
                distance_m = _haversine_distance_m(
                    float(latitude),
                    float(longitude),
                    float(item.latitude),
                    float(item.longitude),
                )
                score = (
                    float(distance_m),
                    1 if bool(item.is_done) else 0,
                    int(item.order_index),
                    -int(path_id),
                    int(item.waypoint_id),
                )
                if best_score is None or score < best_score:
                    best_score = score
                    best_item = item
        return best_item

    @staticmethod
    def _distance_to_waypoint_info(
        current_coordinate: dict[str, float] | None,
        waypoint_info: WaypointInfo | None,
    ) -> float | None:
        if not isinstance(current_coordinate, dict) or waypoint_info is None:
            return None
        latitude = _coerce_float(current_coordinate.get("latitude"))
        longitude = _coerce_float(current_coordinate.get("longitude"))
        if latitude is None or longitude is None:
            return None
        return _haversine_distance_m(
            float(latitude),
            float(longitude),
            float(waypoint_info.latitude),
            float(waypoint_info.longitude),
        )

    @staticmethod
    def _distance_to_indexed_waypoint(
        current_coordinate: dict[str, float] | None,
        waypoint_info: IndexedWaypoint | None,
    ) -> float | None:
        if not isinstance(current_coordinate, dict) or waypoint_info is None:
            return None
        latitude = _coerce_float(current_coordinate.get("latitude"))
        longitude = _coerce_float(current_coordinate.get("longitude"))
        if latitude is None or longitude is None:
            return None
        return _haversine_distance_m(
            float(latitude),
            float(longitude),
            float(waypoint_info.latitude),
            float(waypoint_info.longitude),
        )


class AircraftTurnMonitor:
    def __init__(self, aircraft_id: int, label: str) -> None:
        self.aircraft_id = aircraft_id
        self.label = label
        self._ref_lat_deg: float | None = None
        self._ref_lon_deg: float | None = None
        self._history: deque[TrackPoint] = deque()
        self._actual_points: deque[tuple[float, float]] = deque(maxlen=MAX_PATH_POINTS)
        self._ideal_points: deque[tuple[float, float]] = deque(maxlen=MAX_PATH_POINTS)
        self._active_turn: IdealTurnState | None = None
        self._last_wall_time_s: float | None = None
        self._current_waypoint_id: int | None = None
        self._flying: int | None = None
        self._current_waypoint_info: WaypointInfo | None = None
        self._current_waypoint_position_m: tuple[float, float] | None = None
        self._current_waypoint_is_loiter = False
        self._spiral_hold_state = "none"
        self._spiral_hold_until_s = 0.0
        self._spiral_hold_wp_id: int | None = None
        self._spiral_hold_score: float | None = None
        self._spiral_hold_angle_deg: float | None = None
        self._spiral_hold_radius_m: float | None = None
        self._spiral_hold_radial_span_m: float | None = None
        self._spiral_accum_deg = 0.0
        self._spiral_last_bearing_rad: float | None = None
        self._spiral_last_bearing_wp_id: int | None = None
        self._spiral_last_bearing_timestamp_s: float | None = None
        self._alt_waypoint_arm_s: float | None = None
        self._alt_waypoint_active_id: int | None = None
        self._alt_waypoint_source_wp_id: int | None = None
        self._alt_waypoint_seq = 0
        adaptive_state = _load_adaptive_aircraft_state(int(aircraft_id))
        cfg = _path_deviation_config()
        min_scale = _coerce_float(cfg.get("adaptive_min_scale")) or 0.85
        max_scale = max(min_scale, _coerce_float(cfg.get("adaptive_max_scale")) or 1.45)
        self._adaptive_radius_scale = _clamp_float(
            _coerce_float(adaptive_state.get("radiusScale")) or 1.0,
            min_scale,
            max_scale,
        )
        self._adaptive_sample_count = max(0, _coerce_int(adaptive_state.get("sampleCount")) or 0)
        self._adaptive_expected_roll_deg = _coerce_float(adaptive_state.get("expectedRollDeg"))
        self._adaptive_last_actual_radius_m = _coerce_float(adaptive_state.get("lastActualRadiusM"))
        self._adaptive_last_ideal_radius_m = _coerce_float(adaptive_state.get("lastIdealRadiusM"))
        self._adaptive_last_speed_mps = _coerce_float(adaptive_state.get("lastSpeedMps"))
        self._adaptive_last_sample_s: float | None = None
        self._adaptive_last_save_s = 0.0

    def update(
        self,
        timestamp_s: float,
        wall_time_s: float,
        latitude: float,
        longitude: float,
        altitude_m: float | None,
        speed_mps: float | None,
        raw_heading_deg: float | None,
        current_waypoint_id: int | None = None,
        flying: int | None = None,
        current_waypoint_info: WaypointInfo | None = None,
    ) -> None:
        if self._ref_lat_deg is None or self._ref_lon_deg is None:
            self._ref_lat_deg = float(latitude)
            self._ref_lon_deg = float(longitude)
        x_m, y_m = _latlon_to_local_m(latitude, longitude, self._ref_lat_deg, self._ref_lon_deg)

        prev = self._history[-1] if self._history else None
        if prev is not None and timestamp_s <= prev.timestamp_s:
            timestamp_s = prev.timestamp_s + 0.05

        heading_rad = self._select_heading_rad(
            prev=prev,
            x_m=x_m,
            y_m=y_m,
            timestamp_s=timestamp_s,
            raw_heading_deg=raw_heading_deg,
        )
        point = TrackPoint(
            timestamp_s=timestamp_s,
            wall_time_s=wall_time_s,
            x_m=x_m,
            y_m=y_m,
            altitude_m=altitude_m,
            speed_mps=speed_mps,
            raw_heading_deg=raw_heading_deg,
            heading_rad=heading_rad,
            current_waypoint_id=int(current_waypoint_id) if current_waypoint_id else None,
        )
        self._history.append(point)
        self._trim_history(timestamp_s)
        self._last_wall_time_s = wall_time_s
        self._flying = int(flying) if flying is not None else None
        self._append_if_far_enough(self._actual_points, (x_m, y_m))
        self._update_current_waypoint(current_waypoint_id, current_waypoint_info)
        self._update_ideal_track(point)
        self._update_spiral_accumulator(point)

    def _adaptive_radius_scale_for_use(self, cfg: dict[str, Any] | None = None) -> float:
        data = cfg if isinstance(cfg, dict) else _path_deviation_config()
        if not _adaptive_enabled(data):
            return 1.0
        scale = _coerce_float(self._adaptive_radius_scale) or 1.0
        min_scale = _coerce_float(data.get("adaptive_min_scale")) or 0.85
        max_scale = max(min_scale, _coerce_float(data.get("adaptive_max_scale")) or 1.45)
        warmup_samples = max(1, int(data.get("adaptive_warmup_samples", 4)))
        if self._adaptive_sample_count < warmup_samples:
            blend = max(0.0, min(1.0, float(self._adaptive_sample_count) / float(warmup_samples)))
            scale = 1.0 + ((scale - 1.0) * blend)
        return _clamp_float(scale, min_scale, max_scale)

    def _adaptive_threshold_scale(self, cfg: dict[str, Any] | None = None) -> float:
        data = cfg if isinstance(cfg, dict) else _path_deviation_config()
        return _adaptive_threshold_scale_from_radius_scale(
            data,
            self._adaptive_radius_scale,
            self._adaptive_sample_count,
        )

    def _maybe_update_adaptive_radius(
        self,
        *,
        cfg: dict[str, Any],
        latest: TrackPoint,
        turn_rate_dps: float | None,
        actual_radius_m: float | None,
        ideal_radius_m: float | None,
        turn_sign: int,
    ) -> None:
        if not _adaptive_enabled(cfg):
            return
        if turn_sign == 0 or latest.speed_mps is None:
            return
        turn_rate = _coerce_float(turn_rate_dps)
        actual_radius = _coerce_float(actual_radius_m)
        ideal_radius = _coerce_float(ideal_radius_m)
        speed_mps = _coerce_float(latest.speed_mps)
        if (
            turn_rate is None
            or actual_radius is None
            or ideal_radius is None
            or speed_mps is None
            or abs(float(turn_rate)) < float(cfg.get("adaptive_min_turn_rate_dps", 2.5))
            or actual_radius <= 1.0
            or ideal_radius <= 1.0
        ):
            return
        if self._adaptive_last_sample_s is not None:
            min_interval_s = max(0.2, float(cfg.get("adaptive_sample_min_interval_s", 0.75)))
            if (latest.timestamp_s - float(self._adaptive_last_sample_s)) < min_interval_s:
                return
        min_speed = max(0.0, float(cfg.get("adaptive_min_speed_mps", 25.0)))
        max_speed = max(min_speed, float(cfg.get("adaptive_max_speed_mps", 70.0)))
        if not (min_speed <= float(speed_mps) <= max_speed):
            return
        min_radius = max(1.0, float(cfg.get("adaptive_min_radius_m", 80.0)))
        max_radius = max(min_radius, float(cfg.get("adaptive_max_radius_m", 2000.0)))
        if not (min_radius <= float(actual_radius) <= max_radius):
            return

        min_scale = _coerce_float(cfg.get("adaptive_min_scale")) or 0.85
        max_scale = max(min_scale, _coerce_float(cfg.get("adaptive_max_scale")) or 1.45)
        observed_scale = _clamp_float(float(actual_radius) / max(1e-6, float(ideal_radius)), min_scale, max_scale)
        alpha = _clamp_float(_coerce_float(cfg.get("adaptive_ema_alpha")) or 0.08, 0.01, 1.0)
        previous = _clamp_float(_coerce_float(self._adaptive_radius_scale) or 1.0, min_scale, max_scale)
        self._adaptive_radius_scale = _clamp_float(
            previous + ((observed_scale - previous) * alpha),
            min_scale,
            max_scale,
        )
        self._adaptive_sample_count = min(1000000, int(self._adaptive_sample_count) + 1)
        self._adaptive_last_sample_s = float(latest.timestamp_s)
        self._adaptive_last_actual_radius_m = float(actual_radius)
        self._adaptive_last_ideal_radius_m = float(ideal_radius)
        self._adaptive_last_speed_mps = float(speed_mps)
        self._adaptive_expected_roll_deg = math.degrees(
            math.atan((float(speed_mps) * float(speed_mps)) / (GRAVITY_MPS2 * max(1.0, float(actual_radius))))
        )
        self._save_adaptive_radius_state_if_due(cfg)

    def _save_adaptive_radius_state_if_due(self, cfg: dict[str, Any]) -> None:
        save_interval_s = max(1.0, float(cfg.get("adaptive_save_interval_s", 10.0)))
        now_s = time.monotonic()
        if (now_s - float(self._adaptive_last_save_s)) < save_interval_s:
            return
        self._adaptive_last_save_s = now_s
        try:
            _save_adaptive_aircraft_state(
                self.aircraft_id,
                {
                    "radiusScale": float(self._adaptive_radius_scale),
                    "sampleCount": int(self._adaptive_sample_count),
                    "expectedRollDeg": self._adaptive_expected_roll_deg,
                    "lastActualRadiusM": self._adaptive_last_actual_radius_m,
                    "lastIdealRadiusM": self._adaptive_last_ideal_radius_m,
                    "lastSpeedMps": self._adaptive_last_speed_mps,
                    "updatedAtUnix": float(time.time()),
                },
            )
        except Exception:
            return

    def _trim_history(self, latest_timestamp_s: float) -> None:
        cfg = _path_deviation_config()
        min_timestamp_s = float(latest_timestamp_s) - _history_retention_s(cfg)
        while len(self._history) > 1 and self._history[0].timestamp_s < min_timestamp_s:
            self._history.popleft()
        while len(self._history) > MAX_HISTORY_HARD_CAP:
            self._history.popleft()

    def _clear_spiral_hold(self) -> None:
        self._spiral_hold_state = "none"
        self._spiral_hold_until_s = 0.0
        self._spiral_hold_wp_id = None
        self._spiral_hold_score = None
        self._spiral_hold_angle_deg = None
        self._spiral_hold_radius_m = None
        self._spiral_hold_radial_span_m = None

    def _reset_spiral_accumulator(self) -> None:
        self._spiral_accum_deg = 0.0
        self._spiral_last_bearing_rad = None
        self._spiral_last_bearing_wp_id = None
        self._spiral_last_bearing_timestamp_s = None

    def _clear_alt_waypoint(self) -> None:
        self._alt_waypoint_arm_s = None
        self._alt_waypoint_active_id = None
        self._alt_waypoint_source_wp_id = None

    def _alloc_alt_waypoint_id(self) -> int:
        self._alt_waypoint_seq += 1
        return int(ALT_WAYPOINT_ID_BASE + (self.aircraft_id * 1000) + self._alt_waypoint_seq)

    def _update_current_waypoint(
        self,
        current_waypoint_id: int | None,
        current_waypoint_info: WaypointInfo | None,
    ) -> None:
        self._current_waypoint_id = int(current_waypoint_id) if current_waypoint_id else None
        self._current_waypoint_info = current_waypoint_info if isinstance(current_waypoint_info, WaypointInfo) else None
        self._current_waypoint_is_loiter = bool(
            current_waypoint_info is not None
            and (
                current_waypoint_info.waypoint_pass_type == 2
                or current_waypoint_info.has_loiter_property
            )
        )
        if self._spiral_hold_wp_id is not None and self._spiral_hold_wp_id != self._current_waypoint_id:
            self._clear_spiral_hold()
        if self._alt_waypoint_source_wp_id is not None and self._alt_waypoint_source_wp_id != self._current_waypoint_id:
            self._clear_alt_waypoint()
        if self._spiral_last_bearing_wp_id is not None and self._spiral_last_bearing_wp_id != self._current_waypoint_id:
            self._reset_spiral_accumulator()
        if (
            self._current_waypoint_id is None
            or current_waypoint_info is None
            or self._ref_lat_deg is None
            or self._ref_lon_deg is None
        ):
            self._clear_alt_waypoint()
            self._current_waypoint_position_m = None
            return
        self._current_waypoint_position_m = _latlon_to_local_m(
            current_waypoint_info.latitude,
            current_waypoint_info.longitude,
            self._ref_lat_deg,
            self._ref_lon_deg,
        )

    def _update_spiral_accumulator(self, point: TrackPoint) -> None:
        cfg = _path_deviation_config()
        current_waypoint_id = self._current_waypoint_id
        current_waypoint_position_m = self._current_waypoint_position_m
        if (
            current_waypoint_id is None
            or current_waypoint_position_m is None
            or self._current_waypoint_is_loiter
        ):
            self._reset_spiral_accumulator()
            return

        wp_x_m, wp_y_m = current_waypoint_position_m
        dx_m = point.x_m - wp_x_m
        dy_m = point.y_m - wp_y_m
        distance_m = math.hypot(dx_m, dy_m)
        if distance_m < float(cfg.get("center_ignore_radius_m", 40.0)):
            return

        turn_rate_dps, actual_radius_m, turn_sign, ideal_radius_m = self._estimate_turn_metrics()
        ref_radius_m = _effective_turn_radius(
            ideal_radius_m=ideal_radius_m,
            actual_radius_m=actual_radius_m,
            cfg=cfg,
        )
        if turn_sign == 0 or ref_radius_m is None or ref_radius_m <= 1.0:
            self._spiral_last_bearing_rad = math.atan2(dy_m, dx_m)
            self._spiral_last_bearing_wp_id = current_waypoint_id
            self._spiral_last_bearing_timestamp_s = point.timestamp_s
            return

        threshold_scale = self._adaptive_threshold_scale(cfg)
        near_threshold_m = max(
            float(cfg.get("near_threshold_min_m", 200.0)) * threshold_scale,
            float(ref_radius_m) * float(cfg.get("near_threshold_radius_factor", 2.0)) * threshold_scale,
        )
        if distance_m > near_threshold_m:
            self._reset_spiral_accumulator()
            return

        bearing_rad = math.atan2(dy_m, dx_m)
        if self._spiral_last_bearing_wp_id != current_waypoint_id:
            self._spiral_accum_deg = 0.0
            self._spiral_last_bearing_rad = bearing_rad
            self._spiral_last_bearing_wp_id = current_waypoint_id
            self._spiral_last_bearing_timestamp_s = point.timestamp_s
            return

        if self._spiral_last_bearing_rad is not None:
            delta_rad = _wrap_pi(bearing_rad - self._spiral_last_bearing_rad)
            delta_deg = abs(math.degrees(delta_rad))
            if delta_deg <= float(cfg.get("accumulation_step_max_deg", 120.0)):
                self._spiral_accum_deg += delta_deg

        self._spiral_last_bearing_rad = bearing_rad
        self._spiral_last_bearing_wp_id = current_waypoint_id
        self._spiral_last_bearing_timestamp_s = point.timestamp_s

    def _estimate_spiral_metrics(
        self,
        *,
        ideal_radius_m: float | None,
        actual_radius_m: float | None,
    ) -> tuple[str, float | None, float | None, float | None, float | None]:
        cfg = _path_deviation_config()
        current_waypoint_id = self._current_waypoint_id
        current_waypoint_position_m = self._current_waypoint_position_m
        if (
            current_waypoint_id is None
            or current_waypoint_position_m is None
            or self._current_waypoint_is_loiter
        ):
            return "none", None, None, None, None

        recent: list[TrackPoint] = []
        latest_time = self._history[-1].timestamp_s if self._history else 0.0
        for point in reversed(self._history):
            if point.current_waypoint_id != current_waypoint_id:
                break
            if (latest_time - point.timestamp_s) > float(cfg.get("spiral_window_s", 75.0)):
                break
            recent.append(point)
        recent.reverse()
        min_points = max(3, int(cfg.get("spiral_min_points", 5)))
        if len(recent) < min_points:
            return "none", None, None, None, None

        ref_radius_m = _effective_turn_radius(
            ideal_radius_m=ideal_radius_m,
            actual_radius_m=actual_radius_m,
            cfg=cfg,
        )
        if ref_radius_m is None or ref_radius_m <= 1.0:
            return "none", None, None, None, None

        samples: list[tuple[TrackPoint, float, float]] = []
        wp_x_m, wp_y_m = current_waypoint_position_m
        for point in recent:
            dx_m = point.x_m - wp_x_m
            dy_m = point.y_m - wp_y_m
            distance_m = math.hypot(dx_m, dy_m)
            if distance_m < float(cfg.get("center_ignore_radius_m", 40.0)):
                continue
            samples.append((point, distance_m, math.atan2(dy_m, dx_m)))
        if len(samples) < min_points:
            return "none", None, None, None, None

        threshold_scale = self._adaptive_threshold_scale(cfg)
        radius_error_limit_m = max(
            float(cfg.get("radius_error_min_m", 140.0)) * threshold_scale,
            ref_radius_m * float(cfg.get("radius_error_factor", 0.55)) * threshold_scale,
        )
        stable_suffix: list[tuple[TrackPoint, float, float]] = []
        for sample in reversed(samples):
            if abs(sample[1] - ref_radius_m) <= radius_error_limit_m:
                stable_suffix.append(sample)
                continue
            if stable_suffix:
                break
        if len(stable_suffix) >= min_points:
            stable_suffix.reverse()
            samples = stable_suffix

        angles_rad = [sample[2] for sample in samples]
        distances_m = [sample[1] for sample in samples]
        if len(angles_rad) < min_points:
            return "none", None, None, None, None

        unwrapped = _unwrap_angles(angles_rad)
        window_angle_deg = abs(math.degrees(unwrapped[-1] - unwrapped[0]))
        accum_angle_deg = 0.0
        if self._spiral_last_bearing_wp_id == current_waypoint_id:
            accum_angle_deg = max(0.0, float(self._spiral_accum_deg))
        angle_deg = max(window_angle_deg, accum_angle_deg)
        mean_radius_m = sum(distances_m) / len(distances_m)
        radial_span_m = max(distances_m) - min(distances_m)
        latest_distance_m = distances_m[-1]

        near_threshold_m = max(
            float(cfg.get("near_threshold_min_m", 200.0)) * threshold_scale,
            ref_radius_m * float(cfg.get("near_threshold_radius_factor", 2.0)) * threshold_scale,
        )
        if mean_radius_m > near_threshold_m:
            return "none", None, angle_deg, mean_radius_m, radial_span_m

        radius_matched = abs(mean_radius_m - ref_radius_m) <= radius_error_limit_m
        latest_radius_limit_m = max(
            float(cfg.get("latest_radius_min_m", 60.0)) * threshold_scale,
            ref_radius_m * float(cfg.get("latest_radius_factor", 0.16)) * threshold_scale,
        )
        latest_radius_matched = abs(latest_distance_m - ref_radius_m) <= latest_radius_limit_m
        orbit_reference_radius_m = max(1.0, float(ideal_radius_m or ref_radius_m))
        waypoint_orbit_radius_limit_m = max(
            float(cfg.get("near_threshold_min_m", 200.0)) * threshold_scale,
            orbit_reference_radius_m * 2.2 * threshold_scale,
        )
        waypoint_orbit_span_limit_m = max(
            float(cfg.get("radius_error_min_m", 140.0)) * threshold_scale,
            mean_radius_m * 0.25 * threshold_scale,
        )
        waypoint_orbit_latest_limit_m = max(
            float(cfg.get("latest_radius_min_m", 60.0)) * threshold_scale,
            mean_radius_m * 0.28 * threshold_scale,
        )
        stable_waypoint_orbit = (
            mean_radius_m <= waypoint_orbit_radius_limit_m
            and radial_span_m <= waypoint_orbit_span_limit_m
            and abs(latest_distance_m - mean_radius_m) <= waypoint_orbit_latest_limit_m
        )
        if not (radius_matched and latest_radius_matched) and not stable_waypoint_orbit:
            return "none", None, angle_deg, mean_radius_m, radial_span_m

        warning_angle_deg = float(cfg.get("warning_angle_deg", 150.0))
        watch_angle_deg = float(cfg.get("watch_angle_deg", 90.0))
        score = min(1.0, angle_deg / max(1e-6, warning_angle_deg))
        if angle_deg >= warning_angle_deg:
            return "warning", score, angle_deg, mean_radius_m, radial_span_m
        if angle_deg >= watch_angle_deg:
            return "watch", score, angle_deg, mean_radius_m, radial_span_m
        return "none", score, angle_deg, mean_radius_m, radial_span_m

    def _select_heading_rad(
        self,
        *,
        prev: TrackPoint | None,
        x_m: float,
        y_m: float,
        timestamp_s: float,
        raw_heading_deg: float | None,
    ) -> float | None:
        reported_heading = None
        if raw_heading_deg is not None:
            reported_heading = _heading_deg_to_math_rad(raw_heading_deg)
        if prev is None:
            return reported_heading
        dt_s = max(1e-3, timestamp_s - prev.timestamp_s)
        dx_m = x_m - prev.x_m
        dy_m = y_m - prev.y_m
        motion_heading = None
        cfg = _path_deviation_config()
        heading_move_min_m = max(0.0, float(cfg.get("heading_move_min_m", MIN_HEADING_MOVE_M)))
        if math.hypot(dx_m, dy_m) >= heading_move_min_m and dt_s > 0.0:
            motion_heading = math.atan2(dy_m, dx_m)

        # 0401 reported heading is the primary source. Coordinate-derived heading
        # is kept only as a guarded fallback when the feed omits heading.
        if reported_heading is not None:
            return reported_heading
        if motion_heading is None:
            return prev.heading_rad
        if prev.heading_rad is None:
            return motion_heading

        max_delta_deg = min(
            float(cfg.get("coord_heading_fallback_max_delta_deg", COORD_HEADING_FALLBACK_MAX_DELTA_DEG)),
            max(
                float(cfg.get("coord_heading_fallback_min_delta_deg", COORD_HEADING_FALLBACK_MIN_DELTA_DEG)),
                float(cfg.get("coord_heading_fallback_max_turn_dps", COORD_HEADING_FALLBACK_MAX_TURN_DPS)) * dt_s,
            ),
        )
        if _angle_delta_deg(motion_heading, prev.heading_rad) > max_delta_deg:
            return prev.heading_rad
        return motion_heading

    def _append_if_far_enough(
        self,
        points: deque[tuple[float, float]],
        item: tuple[float, float],
    ) -> None:
        if not points or _distance_m(points[-1], item) >= PATH_SAMPLE_STEP_M:
            points.append(item)

    def _estimate_turn_metrics(self) -> tuple[float | None, float | None, int, float | None]:
        cfg = _path_deviation_config()
        usable = [point for point in self._history if point.heading_rad is not None]
        if usable:
            latest_time = usable[-1].timestamp_s
            recent = [
                point
                for point in usable
                if (latest_time - point.timestamp_s) <= float(cfg.get("turn_window_s", 8.0))
            ]
            if len(recent) >= 4:
                usable = recent
            else:
                usable = usable[-8:]
        if len(usable) < 4:
            return None, None, 0, None
        base_time = usable[0].timestamp_s
        xs = [point.timestamp_s - base_time for point in usable]
        ys = _unwrap_angles([float(point.heading_rad) for point in usable])
        slope = _linear_slope(xs, ys)
        if slope is None:
            return None, None, 0, None
        turn_rate_dps = math.degrees(slope)
        turn_sign = 0
        turn_rate_threshold_dps = float(cfg.get("turn_rate_threshold_dps", 1.0))
        if turn_rate_dps > turn_rate_threshold_dps:
            turn_sign = 1
        elif turn_rate_dps < -turn_rate_threshold_dps:
            turn_sign = -1
        latest_speed = self._history[-1].speed_mps
        actual_radius = None
        if latest_speed is not None and abs(slope) > 1e-6 and turn_sign != 0:
            actual_radius = abs(float(latest_speed) / slope)
        ideal_radius = None
        if latest_speed is not None:
            ideal_radius = interpolate_turn_radius(latest_speed)
        if self._history:
            self._maybe_update_adaptive_radius(
                cfg=cfg,
                latest=self._history[-1],
                turn_rate_dps=turn_rate_dps,
                actual_radius_m=actual_radius,
                ideal_radius_m=ideal_radius,
                turn_sign=turn_sign,
            )
        if ideal_radius is not None:
            ideal_radius = float(ideal_radius) * self._adaptive_radius_scale_for_use(cfg)
        return turn_rate_dps, actual_radius, turn_sign, ideal_radius

    def _update_ideal_track(self, point: TrackPoint) -> None:
        cfg = _path_deviation_config()
        turn_rate_dps, _actual_radius, turn_sign, ideal_radius = self._estimate_turn_metrics()
        speed_mps = point.speed_mps
        if (
            point.heading_rad is None
            or speed_mps is None
            or ideal_radius is None
            or turn_sign == 0
            or turn_rate_dps is None
        ):
            self._active_turn = None
            return
        if (
            self._active_turn is None
            or self._active_turn.turn_sign != turn_sign
            or (point.timestamp_s - self._active_turn.timestamp_s)
            > float(cfg.get("turn_gap_reset_s", TURN_GAP_RESET_S))
        ):
            self._active_turn = IdealTurnState(
                timestamp_s=point.timestamp_s,
                x_m=point.x_m,
                y_m=point.y_m,
                heading_rad=point.heading_rad,
                radius_m=ideal_radius,
                turn_sign=turn_sign,
            )
            self._append_if_far_enough(self._ideal_points, (point.x_m, point.y_m))
            return
        dt_s = max(0.0, min(1.0, point.timestamp_s - self._active_turn.timestamp_s))
        if dt_s <= 0.0:
            return
        yaw_rate_rad_s = (float(speed_mps) / ideal_radius) * float(turn_sign)
        heading_delta = yaw_rate_rad_s * dt_s
        avg_heading = self._active_turn.heading_rad + (heading_delta * 0.5)
        next_x_m = self._active_turn.x_m + (math.cos(avg_heading) * float(speed_mps) * dt_s)
        next_y_m = self._active_turn.y_m + (math.sin(avg_heading) * float(speed_mps) * dt_s)
        self._active_turn = IdealTurnState(
            timestamp_s=point.timestamp_s,
            x_m=next_x_m,
            y_m=next_y_m,
            heading_rad=self._active_turn.heading_rad + heading_delta,
            radius_m=ideal_radius,
            turn_sign=turn_sign,
        )
        self._append_if_far_enough(self._ideal_points, (next_x_m, next_y_m))

    def _stabilize_spiral_state(
        self,
        *,
        latest_timestamp_s: float,
        is_turning: bool,
        position_m: tuple[float, float] | None,
        ref_radius_m: float | None,
        spiral_state: str,
        spiral_score: float | None,
        spiral_angle_deg: float | None,
        spiral_radius_m: float | None,
        spiral_radial_span_m: float | None,
    ) -> tuple[str, float | None, float | None, float | None, float | None]:
        cfg = _path_deviation_config()
        if spiral_state != "none":
            self._spiral_hold_state = str(spiral_state)
            self._spiral_hold_until_s = float(latest_timestamp_s) + float(cfg.get("hold_s", 2.0))
            self._spiral_hold_wp_id = self._current_waypoint_id
            self._spiral_hold_score = spiral_score
            self._spiral_hold_angle_deg = spiral_angle_deg
            self._spiral_hold_radius_m = spiral_radius_m
            self._spiral_hold_radial_span_m = spiral_radial_span_m
            return spiral_state, spiral_score, spiral_angle_deg, spiral_radius_m, spiral_radial_span_m

        if (
            self._spiral_hold_state != "none"
            and self._spiral_hold_wp_id == self._current_waypoint_id
        ):
            if float(latest_timestamp_s) <= float(self._spiral_hold_until_s):
                return (
                    self._spiral_hold_state,
                    self._spiral_hold_score,
                    self._spiral_hold_angle_deg,
                    self._spiral_hold_radius_m,
                    self._spiral_hold_radial_span_m,
                )

            hold_radius_m = ref_radius_m if ref_radius_m is not None else self._spiral_hold_radius_m
            if (
                is_turning
                and hold_radius_m is not None
                and hold_radius_m > 1.0
                and position_m is not None
                and self._current_waypoint_position_m is not None
            ):
                current_distance_m = _distance_m(position_m, self._current_waypoint_position_m)
                threshold_scale = self._adaptive_threshold_scale(cfg)
                release_distance_m = max(
                    float(cfg.get("release_min_distance_m", 240.0)) * threshold_scale,
                    float(hold_radius_m) * float(cfg.get("release_factor", 2.3)) * threshold_scale,
                )
                if current_distance_m <= release_distance_m:
                    return (
                        self._spiral_hold_state,
                        self._spiral_hold_score,
                        self._spiral_hold_angle_deg,
                        self._spiral_hold_radius_m,
                        self._spiral_hold_radial_span_m,
                    )

            self._clear_spiral_hold()
            return (
                "none",
                None,
                None,
                None,
                None,
            )

        self._clear_spiral_hold()
        return spiral_state, spiral_score, spiral_angle_deg, spiral_radius_m, spiral_radial_span_m

    def _estimate_alt_waypoint(
        self,
        *,
        latest: TrackPoint,
        spiral_state: str,
        is_turning: bool,
        turn_sign: int,
        ideal_radius_m: float | None,
        turn_circle_center_m: tuple[float, float] | None,
    ) -> tuple[int | None, tuple[float, float] | None, dict[str, float] | None, float | None]:
        cfg = _path_deviation_config()
        candidate_allowed = spiral_state == "warning" or (
            self._alt_waypoint_active_id is not None and spiral_state in {"watch", "warning"}
        )
        if (
            not candidate_allowed
            or not is_turning
            or turn_sign == 0
            or ideal_radius_m is None
            or ideal_radius_m <= 1.0
            or turn_circle_center_m is None
            or latest.speed_mps is None
            or self._current_waypoint_id is None
            or self._current_waypoint_is_loiter
        ):
            self._clear_alt_waypoint()
            return None, None, None, None

        if self._alt_waypoint_source_wp_id != self._current_waypoint_id:
            self._alt_waypoint_source_wp_id = self._current_waypoint_id
            self._alt_waypoint_arm_s = latest.timestamp_s
            self._alt_waypoint_active_id = None
        elif self._alt_waypoint_arm_s is None:
            self._alt_waypoint_arm_s = latest.timestamp_s

        armed_s = float(self._alt_waypoint_arm_s if self._alt_waypoint_arm_s is not None else latest.timestamp_s)
        alt_waypoint_trigger_s = float(cfg.get("alt_waypoint_trigger_s", 2.0))
        if self._alt_waypoint_active_id is None and (latest.timestamp_s - armed_s) < alt_waypoint_trigger_s:
            return None, None, None, max(0.0, alt_waypoint_trigger_s - (latest.timestamp_s - armed_s))

        if self._alt_waypoint_active_id is None:
            self._alt_waypoint_active_id = self._alloc_alt_waypoint_id()

        center_x_m, center_y_m = turn_circle_center_m
        current_angle_rad = math.atan2(latest.y_m - center_y_m, latest.x_m - center_x_m)
        alt_waypoint_lead_time_s = float(cfg.get("alt_waypoint_lead_time_s", 8.0))
        lead_angle_rad = (
            (float(latest.speed_mps) / float(ideal_radius_m))
            * alt_waypoint_lead_time_s
            * float(turn_sign)
        )
        candidate_angle_rad = current_angle_rad + lead_angle_rad
        candidate_position_m = (
            center_x_m + (math.cos(candidate_angle_rad) * float(ideal_radius_m)),
            center_y_m + (math.sin(candidate_angle_rad) * float(ideal_radius_m)),
        )
        candidate_coordinate = None
        if self._ref_lat_deg is not None and self._ref_lon_deg is not None:
            lat_deg, lon_deg = _local_m_to_latlon(
                candidate_position_m[0],
                candidate_position_m[1],
                self._ref_lat_deg,
                self._ref_lon_deg,
            )
            candidate_coordinate = {
                "latitude": float(lat_deg),
                "longitude": float(lon_deg),
            }
            if latest.altitude_m is not None:
                candidate_coordinate["altitude"] = float(latest.altitude_m)
        return (
            self._alt_waypoint_active_id,
            candidate_position_m,
            candidate_coordinate,
            alt_waypoint_lead_time_s,
        )

    def _current_coordinate(self, latest: TrackPoint) -> dict[str, float] | None:
        if self._ref_lat_deg is None or self._ref_lon_deg is None:
            return None
        lat_deg, lon_deg = _local_m_to_latlon(
            latest.x_m,
            latest.y_m,
            self._ref_lat_deg,
            self._ref_lon_deg,
        )
        coord = {
            "latitude": float(lat_deg),
            "longitude": float(lon_deg),
        }
        if latest.altitude_m is not None:
            coord["altitude"] = float(latest.altitude_m)
        return coord

    def _estimate_next_mission_entry_coordinate(
        self,
        *,
        latest: TrackPoint,
        is_turning: bool,
        turn_sign: int,
        ideal_radius_m: float | None,
        actual_radius_m: float | None,
        turn_circle_center_m: tuple[float, float] | None,
        fallback_coordinate: dict[str, float] | None,
    ) -> tuple[dict[str, float] | None, float | None]:
        lead_s = max(0.0, float(NEXT_MISSION_ENTRY_LEAD_TIME_S))
        turn_radius_scale = _next_collab_turn_radius_scale()
        forward_coordinate, forward_eta_s = project_coordinate_forward(
            fallback_coordinate,
            speed_mps=latest.speed_mps,
            lead_s=lead_s,
            raw_heading_deg=latest.raw_heading_deg,
            heading_rad=latest.heading_rad,
        )
        if self._ref_lat_deg is None or self._ref_lon_deg is None:
            return forward_coordinate, forward_eta_s

        if (
            not is_turning
            or turn_sign == 0
            or turn_circle_center_m is None
            or latest.speed_mps is None
        ):
            return forward_coordinate, forward_eta_s

        radius_m = None
        for candidate in (ideal_radius_m, actual_radius_m):
            if candidate is None:
                continue
            try:
                radius_val = float(candidate)
            except Exception:
                continue
            if radius_val > 1.0:
                radius_m = radius_val * float(turn_radius_scale)
                break
        if radius_m is None:
            return forward_coordinate, forward_eta_s

        center_x_m, center_y_m = turn_circle_center_m
        current_angle_rad = math.atan2(latest.y_m - center_y_m, latest.x_m - center_x_m)
        lead_angle_rad = (
            (float(latest.speed_mps) / float(radius_m))
            * float(lead_s)
            * float(turn_sign)
        )
        projected_angle_rad = current_angle_rad + lead_angle_rad
        projected_x_m = center_x_m + (math.cos(projected_angle_rad) * float(radius_m))
        projected_y_m = center_y_m + (math.sin(projected_angle_rad) * float(radius_m))
        lat_deg, lon_deg = _local_m_to_latlon(
            projected_x_m,
            projected_y_m,
            self._ref_lat_deg,
            self._ref_lon_deg,
        )
        coord: dict[str, float] = {
            "latitude": float(lat_deg),
            "longitude": float(lon_deg),
        }
        if fallback_coordinate is not None and "altitude" in fallback_coordinate:
            try:
                coord["altitude"] = float(fallback_coordinate.get("altitude", 0.0))
            except Exception:
                pass
        return coord, float(lead_s)

    def build_view(
        self,
        now_wall_time_s: float | None = None,
        *,
        include_paths: bool = True,
    ) -> AircraftTurnView:
        now_wall_time_s = float(now_wall_time_s if now_wall_time_s is not None else time.monotonic())
        if not self._history:
            return AircraftTurnView(
                aircraft_id=self.aircraft_id,
                label=self.label,
                has_data=False,
                actual_points=[],
                ideal_points=[],
                position_m=None,
                position_coordinate=None,
                heading_rad=None,
                raw_heading_deg=None,
                speed_mps=None,
                turn_rate_dps=None,
                ideal_radius_m=None,
                actual_radius_m=None,
                turn_sign=0,
                turn_circle_center_m=None,
                turn_circle_radius_m=None,
                current_waypoint_id=None,
                current_waypoint_pass_type=None,
                flying=None,
                current_waypoint_position_m=None,
                alternate_waypoint_id=None,
                alternate_waypoint_position_m=None,
                alternate_waypoint_coordinate=None,
                alternate_waypoint_eta_s=None,
                predicted_entry_coordinate=None,
                predicted_entry_eta_s=None,
                current_waypoint_is_loiter=False,
                spiral_state="none",
                spiral_score=None,
                spiral_angle_deg=None,
                spiral_radius_m=None,
                spiral_radial_span_m=None,
                age_s=None,
                is_turning=False,
            )
        latest = self._history[-1]
        turn_rate_dps, actual_radius_m, turn_sign, ideal_radius_m = self._estimate_turn_metrics()
        spiral_state, spiral_score, spiral_angle_deg, spiral_radius_m, spiral_radial_span_m = (
            self._estimate_spiral_metrics(
                ideal_radius_m=ideal_radius_m,
                actual_radius_m=actual_radius_m,
            )
        )
        cfg = _path_deviation_config()
        effective_radius_m = _effective_turn_radius(
            ideal_radius_m=ideal_radius_m,
            actual_radius_m=actual_radius_m,
            cfg=cfg,
        )
        turn_circle_center = None
        if latest.heading_rad is not None and effective_radius_m is not None and turn_sign != 0:
            normal_x = -math.sin(latest.heading_rad) * float(turn_sign)
            normal_y = math.cos(latest.heading_rad) * float(turn_sign)
            turn_circle_center = (
                latest.x_m + (normal_x * effective_radius_m),
                latest.y_m + (normal_y * effective_radius_m),
            )
        age_s = None
        if self._last_wall_time_s is not None:
            age_s = max(0.0, now_wall_time_s - self._last_wall_time_s)
        is_turning = bool(
            turn_sign != 0
            and age_s is not None
            and age_s <= float(cfg.get("stale_timeout_s", 5.0))
        )
        if not is_turning:
            turn_sign = 0
            turn_circle_center = None
            ideal_radius_m = ideal_radius_m
        spiral_state, spiral_score, spiral_angle_deg, spiral_radius_m, spiral_radial_span_m = (
            self._stabilize_spiral_state(
                latest_timestamp_s=latest.timestamp_s,
                is_turning=is_turning,
                position_m=(latest.x_m, latest.y_m),
                ref_radius_m=effective_radius_m,
                spiral_state=spiral_state,
                spiral_score=spiral_score,
                spiral_angle_deg=spiral_angle_deg,
                spiral_radius_m=spiral_radius_m,
                spiral_radial_span_m=spiral_radial_span_m,
            )
        )
        (
            alternate_waypoint_id,
            alternate_waypoint_position_m,
            alternate_waypoint_coordinate,
            alternate_waypoint_eta_s,
        ) = self._estimate_alt_waypoint(
            latest=latest,
            spiral_state=spiral_state,
            is_turning=is_turning,
            turn_sign=turn_sign,
            ideal_radius_m=effective_radius_m,
            turn_circle_center_m=turn_circle_center,
        )
        position_coordinate = self._current_coordinate(latest)
        predicted_entry_coordinate, predicted_entry_eta_s = (
            self._estimate_next_mission_entry_coordinate(
                latest=latest,
                is_turning=is_turning,
                turn_sign=turn_sign,
                ideal_radius_m=effective_radius_m,
                actual_radius_m=None,
                turn_circle_center_m=turn_circle_center,
                fallback_coordinate=position_coordinate,
            )
        )
        return AircraftTurnView(
            aircraft_id=self.aircraft_id,
            label=self.label,
            has_data=True,
            actual_points=list(self._actual_points) if include_paths else [],
            ideal_points=list(self._ideal_points) if include_paths else [],
            position_m=(latest.x_m, latest.y_m),
            position_coordinate=position_coordinate,
            heading_rad=latest.heading_rad,
            raw_heading_deg=latest.raw_heading_deg,
            speed_mps=latest.speed_mps,
            turn_rate_dps=turn_rate_dps,
            ideal_radius_m=ideal_radius_m,
            actual_radius_m=actual_radius_m,
            turn_sign=turn_sign,
            turn_circle_center_m=turn_circle_center,
            turn_circle_radius_m=effective_radius_m if is_turning else None,
            current_waypoint_id=self._current_waypoint_id,
            current_waypoint_pass_type=(
                self._current_waypoint_info.waypoint_pass_type
                if self._current_waypoint_info is not None
                else None
            ),
            flying=self._flying,
            current_waypoint_position_m=self._current_waypoint_position_m,
            alternate_waypoint_id=alternate_waypoint_id,
            alternate_waypoint_position_m=alternate_waypoint_position_m,
            alternate_waypoint_coordinate=alternate_waypoint_coordinate,
            alternate_waypoint_eta_s=alternate_waypoint_eta_s,
            predicted_entry_coordinate=predicted_entry_coordinate,
            predicted_entry_eta_s=predicted_entry_eta_s,
            current_waypoint_is_loiter=self._current_waypoint_is_loiter,
            spiral_state=spiral_state,
            spiral_score=spiral_score,
            spiral_angle_deg=spiral_angle_deg,
            spiral_radius_m=spiral_radius_m,
            spiral_radial_span_m=spiral_radial_span_m,
            age_s=age_s,
            is_turning=is_turning,
            adaptive_radius_scale=self._adaptive_radius_scale_for_use(cfg),
            adaptive_threshold_scale=self._adaptive_threshold_scale(cfg),
            adaptive_expected_roll_deg=self._adaptive_expected_roll_deg,
            adaptive_sample_count=int(self._adaptive_sample_count),
            adaptive_last_actual_radius_m=self._adaptive_last_actual_radius_m,
            adaptive_last_ideal_radius_m=self._adaptive_last_ideal_radius_m,
            adaptive_last_speed_mps=self._adaptive_last_speed_mps,
        )


class TurnRadiusMonitorStore:
    def __init__(self) -> None:
        self._monitors = {
            4: AircraftTurnMonitor(4, "UAV1"),
            5: AircraftTurnMonitor(5, "UAV2"),
            6: AircraftTurnMonitor(6, "UAV3"),
        }
        self._waypoint_lookup = WaypointLookupCache()

    def ingest_message(self, body: dict[str, Any] | None) -> None:
        if not isinstance(body, dict):
            return
        wall_time_s = time.monotonic()
        timestamp_s = self._extract_timestamp_s(body, wall_time_s)
        raw_list = body.get("agentStateList") or body.get("AgentStateList") or []
        if not isinstance(raw_list, list):
            return
        for item in raw_list:
            if not isinstance(item, dict):
                continue
            aircraft_id = _coerce_int(_first_present(item, "aircraftID", "AircraftID"))
            if aircraft_id not in TRACKED_AIRCRAFT_IDS:
                continue
            coordinate = _first_present(item, "coordinate", "Coordinate") or {}
            velocity = _first_present(item, "velocity", "Velocity") or {}
            if not isinstance(coordinate, dict) or not isinstance(velocity, dict):
                continue
            latitude = _coerce_float(_first_present(coordinate, "latitude", "Latitude"))
            longitude = _coerce_float(_first_present(coordinate, "longitude", "Longitude"))
            altitude_m = _coerce_float(_first_present(coordinate, "altitude", "Altitude"))
            speed_mps = _coerce_float(_first_present(velocity, "speed", "Speed"))
            raw_heading_deg = _coerce_float(_first_present(velocity, "heading", "Heading"))
            if latitude is None or longitude is None:
                continue
            current_waypoint_id_raw = _extract_current_waypoint_id(item)
            flying = _extract_flying(item)
            current_waypoint_id, current_waypoint_info = self._waypoint_lookup.resolve(
                int(aircraft_id),
                current_waypoint_id_raw,
                current_coordinate={
                    "latitude": float(latitude),
                    "longitude": float(longitude),
                },
            )
            self._monitors[int(aircraft_id)].update(
                timestamp_s=timestamp_s,
                wall_time_s=wall_time_s,
                latitude=latitude,
                longitude=longitude,
                altitude_m=altitude_m,
                speed_mps=speed_mps,
                raw_heading_deg=raw_heading_deg,
                current_waypoint_id=current_waypoint_id,
                flying=flying,
                current_waypoint_info=current_waypoint_info,
            )

    def build_view(self, aircraft_id: int, *, include_paths: bool = True) -> AircraftTurnView:
        monitor = self._monitors.get(int(aircraft_id))
        if monitor is None:
            return AircraftTurnView(
                aircraft_id=int(aircraft_id),
                label=f"UAV{aircraft_id}",
                has_data=False,
                actual_points=[],
                ideal_points=[],
                position_m=None,
                position_coordinate=None,
                heading_rad=None,
                raw_heading_deg=None,
                speed_mps=None,
                turn_rate_dps=None,
                ideal_radius_m=None,
                actual_radius_m=None,
                turn_sign=0,
                turn_circle_center_m=None,
                turn_circle_radius_m=None,
                current_waypoint_id=None,
                current_waypoint_pass_type=None,
                flying=None,
                current_waypoint_position_m=None,
                alternate_waypoint_id=None,
                alternate_waypoint_position_m=None,
                alternate_waypoint_coordinate=None,
                alternate_waypoint_eta_s=None,
                predicted_entry_coordinate=None,
                predicted_entry_eta_s=None,
                current_waypoint_is_loiter=False,
                spiral_state="none",
                spiral_score=None,
                spiral_angle_deg=None,
                spiral_radius_m=None,
                spiral_radial_span_m=None,
                age_s=None,
                is_turning=False,
            )
        return monitor.build_view(include_paths=include_paths)

    def build_views(self, *, include_paths: bool = True) -> dict[int, AircraftTurnView]:
        return {
            int(aircraft_id): self.build_view(int(aircraft_id), include_paths=include_paths)
            for aircraft_id in TRACKED_AIRCRAFT_IDS
        }

    @staticmethod
    def _extract_timestamp_s(body: dict[str, Any], default_value: float) -> float:
        raw = _first_present(body, "timestamp", "Timestamp")
        value = _coerce_float(raw)
        if value is None:
            return float(default_value)
        return float(value) / 1000.0
