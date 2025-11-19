from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

LogEmitter = Optional[Callable[[str], None]]


def _safe_emit(log_cb: LogEmitter, message: str) -> None:
    if not log_cb:
        return
    try:
        log_cb(message)
    except Exception:
        pass


def load_attack_context(cmpk_path: Path, log_cb: LogEmitter = None) -> Optional[Dict[str, Any]]:
    try:
        with cmpk_path.open("r", encoding="utf-8") as fh:
            data = json.load(fh)
    except Exception as exc:
        _safe_emit(log_cb, f"[WARN] 공격용 0201 메타데이터 읽기 실패({cmpk_path.name}): {exc}")
        return None
    context = data.get("_attackContext") or data.get("attackContext")
    if isinstance(context, dict):
        return context
    return None


def build_attack_context_from_replan_detail(detail: Any) -> Optional[Dict[str, Any]]:
    if not isinstance(detail, dict):
        return None
    coordinate = detail.get("coordinate") or detail.get("targetCoordinate") or {}
    lat = coordinate.get("latitude")
    lon = coordinate.get("longitude")
    if lat is None or lon is None:
        return None
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
        "detail": detail,
    }
    watcher_id = detail.get("watcherID")
    if watcher_id is not None:
        target_context["watcherID"] = watcher_id
    return target_context


def compute_attack_waypoint(
    project_root: Path,
    friendly: Dict[str, Any],
    target: Dict[str, Any],
    variant_no: int,
    log_cb: LogEmitter = None,
) -> Dict[str, float]:
    fallback = {
        "latitude": float(target.get("latitude") or friendly.get("latitude") or 0.0),
        "longitude": float(target.get("longitude") or friendly.get("longitude") or 0.0),
        "altitude": float(target.get("altitude") or friendly.get("altitude") or 0.0),
    }
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
            "--output-json",
        ]
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, check=False)
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
                return {
                    "latitude": float(lat_val),
                    "longitude": float(lon_val),
                    "altitude": float(alt_val),
                }
            else:
                stderr_msg = (result.stderr or "").strip()
                _safe_emit(
                    log_cb,
                    f"[WARN] 공격 추천 좌표 계산 실패(variant={variant_no}, code={result.returncode}): {stderr_msg}",
                )
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
) -> None:
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
                "weaponType": max(0, int(weapon_type_value)),
            },
        }

    target = attack_context.get("target") or {}
    target_coord = _normalize_coord(target)
    if target_coord is None:
        _safe_emit(
            log_cb,
            f"[WARN] 공격 옵션(variant={variant_no})에 target 좌표 정보가 없어 기본 임무를 유지합니다.",
        )
        return
    manned_missions = [im for im in missions if int(im.get("aircraftID", 0)) in (1, 2)]
    if not manned_missions:
        _safe_emit(log_cb, f"[WARN] 공격 옵션(variant={variant_no}) 대상 유인기 임무를 찾지 못했습니다.")
        return
    manned_missions.sort(key=lambda im: int(im.get("individualMissionID") or 0))
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
    travel_eta_ms = _estimate_eta_ms(approach_coord, attack_waypoint)

    start_wp = _build_wp_entry(
        approach_coord,
        waypoint_id=base_wp_id,
        next_waypoint_id=base_wp_id + 1,
        eta_ms=0,
        target_id_value=0,
        weapon_type_value=0,
        ecf_value=0.0,
    )
    attack_wp = _build_wp_entry(
        attack_waypoint,
        waypoint_id=base_wp_id + 1,
        next_waypoint_id=0,
        eta_ms=travel_eta_ms,
        target_id_value=target_id_int,
        weapon_type_value=1,
        ecf_value=1.0,
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
        _safe_emit(
            log_cb,
            f"[WARN] 공격 비행경로를 덮어쓸 pathID {attack_path_id}를 찾지 못했습니다.",
        )
    else:
        _safe_emit(
            log_cb,
            f"[variant {variant_no}] 공격 임무 설정 완료 (aircraft={attack_aircraft_id}, targetID={target_id_int})",
        )
