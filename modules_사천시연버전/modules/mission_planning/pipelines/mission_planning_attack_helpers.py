from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

from modules.common import agent_status_snapshot

try:
    from modules.mission_planning.MissionPlanner.dynamics.lah_op_envlp import DEFAULT_ENVELOPE
except Exception:
    DEFAULT_ENVELOPE = None  # type: ignore

try:
    from ..runtime.state.attack_assignment import (
        get_last_assigned_manned_id,
        set_last_assigned_manned_id,
    )
    from ..MissionPlanner.runtime_settings import (
        get_runtime_attack_float,
        get_runtime_attack_int,
        get_runtime_attack_int_list,
        get_runtime_attack_weapon_type,
        get_runtime_attack_weapon_type_for_target_type,
    )
except Exception:
    from modules.mission_planning.runtime.state.attack_assignment import (
        get_last_assigned_manned_id,
        set_last_assigned_manned_id,
    )
    from modules.mission_planning.MissionPlanner.runtime_settings import (
        get_runtime_attack_float,
        get_runtime_attack_int,
        get_runtime_attack_int_list,
        get_runtime_attack_weapon_type,
        get_runtime_attack_weapon_type_for_target_type,
    )

LogEmitter = Optional[Callable[[str], None]]
MIN_ATTACK_STANDOFF_M = 300.0
_ATTACK_STANDOFF_MARGIN_M = 1.0
_EARTH_RADIUS_M = 6_371_000.0
_LAH_ATTACK_MAX_SPEED_KMH_FALLBACK = 265.0
_ATTACK_POINT_TIMEOUT_S = 2.0
_ATTACK_LOS_MIN_RAYS = 36
_ATTACK_LOS_MAX_RAYS = 360
_ATTACK_LOS_MIN_RADIUS_M = 500.0
_ATTACK_LOS_MAX_RADIUS_M = 2000.0


def _safe_emit(log_cb: LogEmitter, message: str) -> None:
    if not log_cb:
        return
    try:
        log_cb(message)
    except Exception:
        pass


def _to_int(value: Any) -> Optional[int]:
    try:
        if value is None:
            return None
        return int(float(value))
    except Exception:
        return None


def _to_float(value: Any) -> Optional[float]:
    try:
        if value is None:
            return None
        return float(value)
    except Exception:
        return None


def _lah_max_attack_speed_mps() -> float:
    try:
        max_speed_kmh = float(getattr(DEFAULT_ENVELOPE, "max_speed_kmh", _LAH_ATTACK_MAX_SPEED_KMH_FALLBACK))
    except Exception:
        max_speed_kmh = _LAH_ATTACK_MAX_SPEED_KMH_FALLBACK
    if not math.isfinite(max_speed_kmh) or max_speed_kmh <= 0.0:
        max_speed_kmh = _LAH_ATTACK_MAX_SPEED_KMH_FALLBACK
    return round(float(max_speed_kmh) / 3.6, 2)


def _normalize_geo_coordinate(raw: Optional[Dict[str, Any]]) -> Optional[Dict[str, float]]:
    if not isinstance(raw, dict):
        return None
    latitude = raw.get("latitude")
    longitude = raw.get("longitude")
    altitude = raw.get("altitude")
    if latitude is None:
        latitude = raw.get("lat")
    if longitude is None:
        longitude = raw.get("lon")
    if altitude is None:
        altitude = raw.get("alt")
    lat_val = _to_float(latitude)
    lon_val = _to_float(longitude)
    alt_val = _to_float(altitude)
    if lat_val is None or lon_val is None:
        return None
    payload: Dict[str, float] = {
        "latitude": float(lat_val),
        "longitude": float(lon_val),
    }
    if alt_val is not None:
        payload["altitude"] = float(alt_val)
    return payload


def _haversine_distance_m(a: Dict[str, Any], b: Dict[str, Any]) -> Optional[float]:
    coord_a = _normalize_geo_coordinate(a)
    coord_b = _normalize_geo_coordinate(b)
    if coord_a is None or coord_b is None:
        return None
    lat1 = math.radians(coord_a["latitude"])
    lon1 = math.radians(coord_a["longitude"])
    lat2 = math.radians(coord_b["latitude"])
    lon2 = math.radians(coord_b["longitude"])
    d_lat = lat2 - lat1
    d_lon = lon2 - lon1
    a_val = math.sin(d_lat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(d_lon / 2.0) ** 2
    c_val = 2.0 * math.atan2(math.sqrt(a_val), math.sqrt(1.0 - a_val))
    return _EARTH_RADIUS_M * c_val


def _bearing_between(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    d_lambda = math.radians(lon2 - lon1)
    y = math.sin(d_lambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(d_lambda)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def _project_coordinate(coord: Dict[str, Any], heading_deg: float, distance_m: float) -> Optional[Dict[str, float]]:
    normalized = _normalize_geo_coordinate(coord)
    if normalized is None:
        return None
    lat_rad = math.radians(normalized["latitude"])
    lon_rad = math.radians(normalized["longitude"])
    heading_rad = math.radians(float(heading_deg))
    angular_distance = max(0.0, float(distance_m)) / _EARTH_RADIUS_M
    new_lat = math.asin(
        math.sin(lat_rad) * math.cos(angular_distance)
        + math.cos(lat_rad) * math.sin(angular_distance) * math.cos(heading_rad)
    )
    new_lon = lon_rad + math.atan2(
        math.sin(heading_rad) * math.sin(angular_distance) * math.cos(lat_rad),
        math.cos(angular_distance) - math.sin(lat_rad) * math.sin(new_lat),
    )
    payload = {
        "latitude": math.degrees(new_lat),
        "longitude": math.degrees(new_lon),
    }
    altitude_val = _to_float(normalized.get("altitude"))
    if altitude_val is not None:
        payload["altitude"] = float(altitude_val)
    return payload


def ensure_attack_standoff_coordinate(
    candidate_coord: Optional[Dict[str, Any]],
    target_coord: Optional[Dict[str, Any]],
    *,
    friendly_coord: Optional[Dict[str, Any]] = None,
    min_distance_m: float = MIN_ATTACK_STANDOFF_M,
    fallback_heading_deg: Optional[float] = None,
) -> Dict[str, float]:
    min_distance = max(0.0, _to_float(min_distance_m) or MIN_ATTACK_STANDOFF_M)
    projection_distance = min_distance + _ATTACK_STANDOFF_MARGIN_M
    target = _normalize_geo_coordinate(target_coord)
    candidate = _normalize_geo_coordinate(candidate_coord)
    friendly = _normalize_geo_coordinate(friendly_coord)

    if target is None:
        if candidate is not None:
            return dict(candidate)
        if friendly is not None:
            return dict(friendly)
        return {"latitude": 0.0, "longitude": 0.0, "altitude": 0.0}

    base_altitude = _to_float((candidate or {}).get("altitude"))
    if base_altitude is None:
        base_altitude = _to_float(target.get("altitude"))
    if base_altitude is None:
        base_altitude = _to_float((friendly or {}).get("altitude"))

    candidate_distance = _haversine_distance_m(candidate or target, target)
    if candidate is not None and candidate_distance is not None and candidate_distance > min_distance:
        result = dict(candidate)
        if result.get("altitude") is None and base_altitude is not None:
            result["altitude"] = float(base_altitude)
        return result

    bearing_deg: Optional[float] = None
    for anchor in (friendly, candidate):
        anchor_distance = _haversine_distance_m(anchor or {}, target)
        if anchor is None or anchor_distance is None or anchor_distance <= 1.0:
            continue
        bearing_deg = _bearing_between(
            target["latitude"],
            target["longitude"],
            anchor["latitude"],
            anchor["longitude"],
        )
        break

    if bearing_deg is None and fallback_heading_deg is not None:
        bearing_deg = (float(fallback_heading_deg) + 180.0) % 360.0
    if bearing_deg is None:
        bearing_deg = 0.0

    projected = _project_coordinate(target, bearing_deg, projection_distance)
    result = dict(projected or target)
    if base_altitude is not None:
        result["altitude"] = float(base_altitude)
    elif target.get("altitude") is not None:
        result["altitude"] = float(target["altitude"])
    else:
        result["altitude"] = 0.0
    return result


def get_attack_standoff_distances() -> Tuple[float, float]:
    min_distance = get_runtime_attack_float("attack_min_standoff_m", MIN_ATTACK_STANDOFF_M)
    preferred_distance = get_runtime_attack_float("attack_preferred_standoff_m", 1000.0)
    try:
        min_distance = float(min_distance)
    except Exception:
        min_distance = MIN_ATTACK_STANDOFF_M
    try:
        preferred_distance = float(preferred_distance)
    except Exception:
        preferred_distance = 1000.0
    if not math.isfinite(min_distance) or min_distance < 0.0:
        min_distance = MIN_ATTACK_STANDOFF_M
    if not math.isfinite(preferred_distance) or preferred_distance < 0.0:
        preferred_distance = 1000.0
    if preferred_distance < min_distance:
        preferred_distance = min_distance
    return float(min_distance), float(preferred_distance)


def select_attack_standoff_coordinate(
    friendly_coord: Optional[Dict[str, Any]],
    target_coord: Optional[Dict[str, Any]],
    *,
    candidate_coord: Optional[Dict[str, Any]] = None,
    min_distance_m: Optional[float] = None,
    preferred_distance_m: Optional[float] = None,
    fallback_heading_deg: Optional[float] = None,
) -> Dict[str, Any]:
    configured_min, configured_preferred = get_attack_standoff_distances()
    min_distance = configured_min if min_distance_m is None else float(min_distance_m)
    preferred_distance = configured_preferred if preferred_distance_m is None else float(preferred_distance_m)
    if not math.isfinite(min_distance) or min_distance < 0.0:
        min_distance = configured_min
    if not math.isfinite(preferred_distance) or preferred_distance < 0.0:
        preferred_distance = configured_preferred
    if preferred_distance < min_distance:
        preferred_distance = min_distance

    target = _normalize_geo_coordinate(target_coord)
    friendly = _normalize_geo_coordinate(friendly_coord)
    candidate = _normalize_geo_coordinate(candidate_coord)
    if target is None:
        selected = candidate or friendly or {"latitude": 0.0, "longitude": 0.0, "altitude": 0.0}
        return {
            "coordinate": dict(selected),
            "mode": "fallback_no_target",
            "current_distance_m": None,
            "candidate_distance_m": None,
            "min_standoff_m": float(min_distance),
            "preferred_standoff_m": float(preferred_distance),
        }

    base_altitude = _to_float((candidate or {}).get("altitude"))
    if base_altitude is None:
        base_altitude = _to_float(target.get("altitude"))
    if base_altitude is None:
        base_altitude = _to_float((friendly or {}).get("altitude"))

    current_distance = _haversine_distance_m(friendly or {}, target) if friendly is not None else None
    candidate_distance = _haversine_distance_m(candidate or {}, target) if candidate is not None else None

    def _with_alt(coord: Dict[str, Any]) -> Dict[str, float]:
        result = dict(coord)
        if result.get("altitude") is None and base_altitude is not None:
            result["altitude"] = float(base_altitude)
        elif result.get("altitude") is None:
            result["altitude"] = 0.0
        return result

    if friendly is not None and current_distance is not None:
        if min_distance <= float(current_distance) <= preferred_distance:
            return {
                "coordinate": _with_alt(friendly),
                "mode": "current_position_within_standoff",
                "current_distance_m": float(current_distance),
                "candidate_distance_m": float(candidate_distance) if candidate_distance is not None else None,
                "min_standoff_m": float(min_distance),
                "preferred_standoff_m": float(preferred_distance),
            }

    if candidate is not None and candidate_distance is not None:
        if min_distance <= float(candidate_distance) <= preferred_distance:
            return {
                "coordinate": _with_alt(candidate),
                "mode": "candidate_within_standoff",
                "current_distance_m": float(current_distance) if current_distance is not None else None,
                "candidate_distance_m": float(candidate_distance),
                "min_standoff_m": float(min_distance),
                "preferred_standoff_m": float(preferred_distance),
            }

    bearing_deg: Optional[float] = None
    if friendly is not None and current_distance is not None and current_distance > 1.0:
        bearing_deg = _bearing_between(
            target["latitude"],
            target["longitude"],
            friendly["latitude"],
            friendly["longitude"],
        )
    elif candidate is not None and candidate_distance is not None and candidate_distance > 1.0:
        bearing_deg = _bearing_between(
            target["latitude"],
            target["longitude"],
            candidate["latitude"],
            candidate["longitude"],
        )
    elif fallback_heading_deg is not None:
        bearing_deg = (float(fallback_heading_deg) + 180.0) % 360.0
    else:
        bearing_deg = 0.0

    if friendly is not None and current_distance is not None and float(current_distance) < min_distance:
        projection_distance = min_distance + _ATTACK_STANDOFF_MARGIN_M
        mode = "current_too_close_projected_to_min_standoff"
    else:
        projection_distance = preferred_distance if preferred_distance > 0.0 else min_distance
        mode = "far_projected_to_preferred_standoff"

    projected = _project_coordinate(target, bearing_deg, projection_distance)
    selected = _with_alt(projected or target)
    return {
        "coordinate": selected,
        "mode": mode,
        "current_distance_m": float(current_distance) if current_distance is not None else None,
        "candidate_distance_m": float(candidate_distance) if candidate_distance is not None else None,
        "min_standoff_m": float(min_distance),
        "preferred_standoff_m": float(preferred_distance),
    }


def extract_attack_weapon_inventory(payload: Any) -> Dict[str, int]:
    def _from_block(block: Any) -> Optional[Dict[str, int]]:
        if not isinstance(block, dict):
            return None
        type1 = _to_int(block.get("type1"))
        type2 = _to_int(block.get("type2"))
        type3 = _to_int(block.get("type3"))
        if type1 is None and type2 is None and type3 is None:
            return None
        return {
            "type1": max(0, int(type1 or 0)),
            "type2": max(0, int(type2 or 0)),
            "type3": max(0, int(type3 or 0)),
        }

    if isinstance(payload, dict):
        for block in (
            payload.get("weapon_inventory"),
            payload.get("weaponInventory"),
            payload,
            payload.get("weapons"),
            (payload.get("mannedInfo") or {}).get("weapons"),
            (payload.get("manned_info") or {}).get("weapons"),
        ):
            inventory = _from_block(block)
            if inventory is not None:
                return inventory
    return {"type1": 0, "type2": 0, "type3": 0}


def attack_weapon_preference_order(preferred_weapon_type: int | None) -> Tuple[int, int, int]:
    preferred = _to_int(preferred_weapon_type)
    if preferred == 1:
        return (1, 2, 3)
    if preferred == 2:
        return (2, 1, 3)
    if preferred == 3:
        return (3, 2, 1)
    return (1, 2, 3)


def choose_attack_weapon_type(
    preferred_weapon_type: int | None,
    weapon_inventory: Optional[Dict[str, int]],
) -> Dict[str, Any]:
    preferred = _to_int(preferred_weapon_type)
    if preferred is None or preferred < 1 or preferred > 3:
        preferred = 1
    inventory = extract_attack_weapon_inventory(weapon_inventory or {})
    preference = attack_weapon_preference_order(preferred)
    slot_map = {1: "type1", 2: "type2", 3: "type3"}
    selected = None
    for weapon_type in preference:
        if int(inventory.get(slot_map[weapon_type], 0)) > 0:
            selected = int(weapon_type)
            break
    ammo_available = selected is not None
    if selected is None:
        selected = int(preferred)
    selected_slot = slot_map.get(int(selected))
    selected_ammo = int(inventory.get(selected_slot, 0)) if selected_slot else 0
    return {
        "preferredWeaponType": int(preferred),
        "selectedWeaponType": int(selected),
        "weaponPreference": list(preference),
        "weaponInventory": inventory,
        "ammoAvailable": bool(ammo_available),
        "selectedAmmo": int(selected_ammo),
    }


def load_current_aircraft_weapon_inventory(aircraft_id: int | None) -> Dict[str, int]:
    aircraft_id_value = _to_int(aircraft_id)
    if aircraft_id_value is None:
        return {"type1": 0, "type2": 0, "type3": 0}
    snapshot = agent_status_snapshot.load_agent_status_snapshot() or {}
    agent_states = snapshot.get("agent_states") or snapshot.get("raw", {}).get("agentStateList") or []
    for state in agent_states:
        if not isinstance(state, dict):
            continue
        state_aircraft_id = _to_int(state.get("aircraftID") or state.get("aircraftId"))
        if state_aircraft_id == aircraft_id_value:
            return extract_attack_weapon_inventory(state)
    return {"type1": 0, "type2": 0, "type3": 0}


def load_attack_context(cmpk_path: Path, log_cb: LogEmitter = None) -> Optional[Dict[str, Any]]:
    try:
        with cmpk_path.open("r", encoding="utf-8") as fh:
            data = json.load(fh)
    except Exception as exc:
        _safe_emit(log_cb, f"[WARN] 공격용 0201 메타데이터 읽기 실패({cmpk_path.name}): {exc}")
        return None
    context = data.get("_attackContext") or data.get("attackContext")
    if isinstance(context, dict):
        normalized = dict(context)
        detail = normalized.get("detail")
        if normalized.get("targetType") is None and isinstance(detail, dict):
            normalized["targetType"] = detail.get("targetType")
        return normalized
    return None


def build_attack_context_from_replan_detail(detail: Any) -> Optional[Dict[str, Any]]:
    if not isinstance(detail, dict):
        return None
    raw_targets = detail.get("attackTargetList") or detail.get("targetList") or detail.get("targets") or []
    normalized_targets: list[Dict[str, Any]] = []
    if isinstance(raw_targets, list):
        for item in raw_targets:
            if not isinstance(item, dict):
                continue
            coordinate = item.get("coordinate") or item.get("targetCoordinate") or {}
            lat = coordinate.get("latitude")
            lon = coordinate.get("longitude")
            if lat is None or lon is None:
                continue
            altitude = coordinate.get("altitude") or 0.0
            try:
                lat = float(lat)
                lon = float(lon)
                altitude = float(altitude)
            except Exception:
                continue
            normalized = {
                "target": {
                    "latitude": lat,
                    "longitude": lon,
                    "altitude": altitude,
                },
                "targetID": item.get("targetID"),
                "targetType": item.get("targetType"),
            }
            watcher_id = item.get("watcherID")
            if watcher_id is not None:
                normalized["watcherID"] = watcher_id
            normalized_targets.append(normalized)

    coordinate = detail.get("coordinate") or detail.get("targetCoordinate") or {}
    lat = coordinate.get("latitude")
    lon = coordinate.get("longitude")
    if lat is None or lon is None:
        if not normalized_targets:
            return None
        target_context = dict(normalized_targets[0])
        target_context["detail"] = detail
        target_context["targets"] = normalized_targets
        target_context["targetCount"] = len(normalized_targets)
        return target_context

    altitude = coordinate.get("altitude") or 0.0
    try:
        lat = float(lat)
        lon = float(lon)
        altitude = float(altitude)
    except Exception:
        return None
    target_context = {
        "target": {
            "latitude": lat,
            "longitude": lon,
            "altitude": altitude,
        },
        "targetID": detail.get("targetID"),
        "targetType": detail.get("targetType"),
        "detail": detail,
    }
    watcher_id = detail.get("watcherID")
    if watcher_id is not None:
        target_context["watcherID"] = watcher_id
    if normalized_targets:
        target_context["targets"] = normalized_targets
        target_context["targetCount"] = len(normalized_targets)
    return target_context


def compute_attack_waypoint(
    project_root: Path,
    friendly: Dict[str, Any],
    target: Dict[str, Any],
    variant_no: int,
    log_cb: LogEmitter = None,
) -> Dict[str, float]:
    fallback_selection = select_attack_standoff_coordinate(
        friendly,
        target,
    )
    fallback = dict(fallback_selection.get("coordinate") or {})
    script_path = (
        project_root
        / "modules"
        / "mission_planning"
        / "MissionPlanner"
        / "data_def"
        / "lah_attack_assistance.py"
    )
    friendly_lat = friendly.get("latitude")
    friendly_lon = friendly.get("longitude")
    target_lat = target.get("latitude")
    target_lon = target.get("longitude")
    if (
        script_path.exists()
        and friendly_lat is not None
        and friendly_lon is not None
        and target_lat is not None
        and target_lon is not None
    ):
        min_standoff_m, preferred_standoff_m = get_attack_standoff_distances()
        try:
            num_rays = int(get_runtime_attack_int("fast_num_arc_rays", 180))
        except Exception:
            num_rays = 180
        num_rays = max(_ATTACK_LOS_MIN_RAYS, min(_ATTACK_LOS_MAX_RAYS, int(num_rays)))
        analysis_radius_m = max(
            _ATTACK_LOS_MIN_RADIUS_M,
            min(_ATTACK_LOS_MAX_RADIUS_M, float(preferred_standoff_m or _ATTACK_LOS_MAX_RADIUS_M)),
        )
        cmd = [
            sys.executable or "python",
            str(script_path),
            "--friendly-lat",
            str(friendly_lat),
            "--friendly-lon",
            str(friendly_lon),
            "--enemy-lat",
            str(target_lat),
            "--enemy-lon",
            str(target_lon),
            "--radius-m",
            str(analysis_radius_m),
            "--num-rays",
            str(num_rays),
            "--output-json",
        ]
        try:
            timeout_s = float(get_runtime_attack_float("attack_point_subprocess_timeout_s", _ATTACK_POINT_TIMEOUT_S))
            if not math.isfinite(timeout_s) or timeout_s <= 0.0:
                timeout_s = _ATTACK_POINT_TIMEOUT_S
            result = subprocess.run(cmd, capture_output=True, text=True, check=False, timeout=timeout_s)
            if result.returncode == 0:
                data = json.loads(result.stdout or "{}")
                attack_point = data.get("attack_point") or {}
                lat_val = attack_point.get("lat") or attack_point.get("latitude") or target_lat
                lon_val = attack_point.get("lon") or attack_point.get("longitude") or target_lon
                alt_val = (
                    attack_point.get("alt_m")
                    or attack_point.get("altitude")
                    or target.get("altitude")
                    or friendly.get("altitude")
                    or 0.0
                )
                altitude_offset_m = get_runtime_attack_float("attack_point_altitude_offset_m", 300.0)
                selected = {
                    "latitude": float(lat_val),
                    "longitude": float(lon_val),
                    "altitude": float(alt_val) + float(altitude_offset_m),
                    "selection_mode": "los_area",
                    "los_area": True,
                    "terrain_altitude_m": float(alt_val),
                    "altitude_offset_m": float(altitude_offset_m),
                    "raster_sources": list(data.get("raster_sources") or []),
                    "analysis_radius_m": float(analysis_radius_m),
                    "num_rays": int(num_rays),
                }
                candidate_distance_m = _haversine_distance_m(selected, target)
                if candidate_distance_m is None or float(candidate_distance_m) < float(min_standoff_m):
                    _safe_emit(
                        log_cb,
                        f"[variant {variant_no}] LOS attack point rejected "
                        f"(distance={candidate_distance_m}, min={min_standoff_m}); fallback standoff used",
                    )
                    return fallback
                _safe_emit(
                    log_cb,
                    f"[variant {variant_no}] LOS attack point selected "
                    f"candidateDist={candidate_distance_m}m rays={num_rays} radius={analysis_radius_m}m",
                )
                return selected
            else:
                stderr_msg = (result.stderr or "").strip()
                _safe_emit(
                    log_cb,
                    f"[WARN] 공격 추천 좌표 계산 실패(variant={variant_no}, code={result.returncode}): {stderr_msg}",
                )
        except subprocess.TimeoutExpired:
            _safe_emit(log_cb, f"[WARN] attack point LOS calculation timed out (variant={variant_no})")
        except Exception as exc:
            _safe_emit(log_cb, f"[WARN] 공격 추천 좌표 계산 중 예외 발생(variant={variant_no}): {exc}")
    return fallback


def apply_attack_customizations(
    missions: List[Dict[str, Any]],
    flight_plans_0304: List[Dict[str, Any]],
    attack_context: Dict[str, Any],
    variant_no: int,
    *,
    replan_detail: Optional[Dict[str, Any]] = None,
    project_root: Path,
    log_cb: LogEmitter = None,
) -> Dict[str, Any]:
    def _normalize_coord(raw: Optional[Dict[str, Any]]) -> Optional[Dict[str, float]]:
        if not isinstance(raw, dict):
            return None
        lat = raw.get("latitude")
        lon = raw.get("longitude")
        if lat is None or lon is None:
            return None
        alt = raw.get("altitude", 0.0)
        try:
            return {
                "latitude": float(lat),
                "longitude": float(lon),
                "altitude": float(alt),
            }
        except Exception:
            return None

    def _estimate_eta_ms(p0: Dict[str, float], p1: Dict[str, float], speed_mps: float = 40.0) -> int:
        lat1, lon1 = p0["latitude"], p0["longitude"]
        lat2, lon2 = p1["latitude"], p1["longitude"]
        k = 111_132.92
        cos = math.cos(math.radians((lat1 + lat2) / 2))
        dx = (lon2 - lon1) * k * cos
        dy = (lat2 - lat1) * k
        dist_m = math.hypot(dx, dy)
        if dist_m <= 0 or speed_mps <= 0:
            return 0
        return int(round(1000 * dist_m / speed_mps))

    def _build_wp_entry(
        coord: Dict[str, float],
        waypoint_id: int,
        next_waypoint_id: int,
        eta_ms: int,
        *,
        target_id_value: int,
        weapon_type_value: int,
        ecf_value: float,
        speed_value: float = 40.0,
    ) -> Dict[str, Any]:
        return {
            "waypointID": waypoint_id,
            "coordinate": {
                "latitude": round(coord["latitude"], 6),
                "longitude": round(coord["longitude"], 6),
                "altitude": int(round(coord.get("altitude", 0.0))),
            },
            "speed": speed_value,
            "eta": int(eta_ms),
            "ecf": float(ecf_value),
            "nextWaypointID": next_waypoint_id,
            "hovering": {"time": 0},
            "loiter": {"radius": 0, "direction": 0, "time": 0, "speed": 0},
            "attack": {
                "targetID": max(0, int(target_id_value)),
                "weaponType": max(0, min(3, int(weapon_type_value))),
            },
        }

    target = attack_context.get("target") or {}
    target_coord = _normalize_coord(target)
    if target_coord is None:
        notice = "공격 불가: 표적 좌표 없음"
        _safe_emit(
            log_cb,
            f"[WARN] 공격 옵션(variant={variant_no})에 target 좌표 정보가 없어 기본 임무를 유지합니다.",
        )
        return {"applied": False, "failure_notice": notice}
    allowed_manned_ids = set(get_runtime_attack_int_list("manned_candidate_ids", [2, 3]))
    manned_missions = [im for im in missions if int(im.get("aircraftID", 0)) in allowed_manned_ids]
    if not manned_missions:
        notice = "공격 불가: 공격기 후보 없음"
        _safe_emit(log_cb, f"[WARN] 공격 옵션(variant={variant_no}) 대상 유인기 임무를 찾지 못했습니다.")
        return {"applied": False, "failure_notice": notice}
    manned_missions.sort(key=lambda im: int(im.get("individualMissionID") or 0))
    last_assigned = get_last_assigned_manned_id()
    if last_assigned is not None:
        filtered = [im for im in manned_missions if int(im.get("aircraftID", 0)) != last_assigned]
        if filtered:
            manned_missions = filtered
    primary_mission = manned_missions[0]
    mission_info = primary_mission.get("individualMissionInfo") or {}
    coord_list = mission_info.get("coordinateList") or []
    friendly_coord = None
    if coord_list and isinstance(coord_list[0], dict):
        friendly_coord = _normalize_coord(coord_list[0])
    if friendly_coord is None:
        friendly_coord = dict(target_coord)
    attack_waypoint = compute_attack_waypoint(
        project_root, friendly_coord, target_coord, variant_no, log_cb
    )
    target_id = attack_context.get("targetID")
    try:
        target_id_int = int(target_id) if target_id is not None else 0
    except Exception:
        target_id_int = 0
    target_type_value = _to_int(attack_context.get("targetType"))
    if target_type_value is None and isinstance(replan_detail, dict):
        target_type_value = _to_int(replan_detail.get("targetType"))
    context_detail = attack_context.get("detail")
    if target_type_value is None and isinstance(context_detail, dict):
        target_type_value = _to_int(context_detail.get("targetType"))
    preferred_weapon_type = get_runtime_attack_weapon_type_for_target_type(
        target_type_value,
        default=get_runtime_attack_weapon_type(2),
    )
    weapon_inventory = load_current_aircraft_weapon_inventory(primary_mission.get("aircraftID"))
    weapon_choice = choose_attack_weapon_type(preferred_weapon_type, weapon_inventory)
    selected_weapon_type = int(weapon_choice["selectedWeaponType"])
    if not bool(weapon_choice.get("ammoAvailable")):
        notice = "공격 불가: 공격기 탄약 부족"
        _safe_emit(
            log_cb,
            f"[variant {variant_no}] 공격 무기 선택 실패 preferred={preferred_weapon_type} ammo={weapon_choice.get('weaponInventory')}",
        )
        return {"applied": False, "failure_notice": notice, "weapon_choice": weapon_choice}
    _safe_emit(
        log_cb,
        f"[variant {variant_no}] 공격 무기 선택 targetType={target_type_value if target_type_value is not None else 'fallback'} "
        f"preferred={preferred_weapon_type} selected={selected_weapon_type} "
        f"ammo={weapon_choice.get('weaponInventory')}",
    )

    coordinate_entries: List[Dict[str, float]] = []
    if friendly_coord:
        coordinate_entries.append(
            {
                "latitude": friendly_coord["latitude"],
                "longitude": friendly_coord["longitude"],
                "altitude": friendly_coord.get("altitude", 0.0),
            }
        )
    coordinate_entries.append(
        {
            "latitude": target_coord["latitude"],
            "longitude": target_coord["longitude"],
            "altitude": target_coord.get("altitude", 0.0),
        }
    )

    mission_info_override = {
        "individualMissionType": 2,
        "patternType": 2,
        "autoZoomIn": 0,
        "coordinateList": coordinate_entries,
    }
    if target_id_int:
        mission_info_override["targetID"] = target_id_int
    if replan_detail:
        mission_info_override["_attackDetail"] = replan_detail
    primary_mission["individualMissionInfo"] = mission_info_override
    primary_mission["isDone"] = False

    attack_path_id = int(primary_mission.get("pathID") or 0)
    attack_aircraft_id = int(primary_mission.get("aircraftID") or 0)
    base_wp_id = 10_000 + variant_no * 10
    approach_coord = friendly_coord or dict(attack_waypoint)
    attack_speed_mps = _lah_max_attack_speed_mps()
    travel_eta_ms = _estimate_eta_ms(approach_coord, attack_waypoint, speed_mps=attack_speed_mps)
    _safe_emit(
        log_cb,
        f"[variant {variant_no}] LAH attack speed set to max "
        f"{attack_speed_mps:.2f}m/s ({attack_speed_mps * 3.6:.1f}km/h)",
    )

    start_wp = _build_wp_entry(
        approach_coord,
        waypoint_id=base_wp_id,
        next_waypoint_id=base_wp_id + 1,
        eta_ms=0,
        target_id_value=0,
        weapon_type_value=0,
        ecf_value=0.0,
        speed_value=attack_speed_mps,
    )
    attack_wp = _build_wp_entry(
        attack_waypoint,
        waypoint_id=base_wp_id + 1,
        next_waypoint_id=0,
        eta_ms=travel_eta_ms,
        target_id_value=target_id_int,
        weapon_type_value=selected_weapon_type,
        ecf_value=1.0,
        speed_value=attack_speed_mps,
    )

    replaced_fp = False
    for fp in flight_plans_0304 or []:
        try:
            fp_path_id = int(fp.get("pathID"))
        except Exception:
            fp_path_id = None
        if fp_path_id == attack_path_id and int(fp.get("aircraftID", 0)) == attack_aircraft_id:
            fp["lahWaypointList"] = [start_wp, attack_wp]
            replaced_fp = True
            break
    if not replaced_fp:
        notice = f"공격 불가: 경로 없음({attack_path_id})"
        _safe_emit(
            log_cb,
            f"[WARN] 공격 비행경로를 덮어쓸 pathID {attack_path_id}를 찾지 못했습니다.",
        )
        return {"applied": False, "failure_notice": notice}
    else:
        _safe_emit(
            log_cb,
            f"[variant {variant_no}] 공격 임무 설정 완료 (aircraft={attack_aircraft_id}, targetID={target_id_int})",
        )
        set_last_assigned_manned_id(attack_aircraft_id)
        return {"applied": True, "weapon_choice": weapon_choice, "aircraft_id": attack_aircraft_id}
