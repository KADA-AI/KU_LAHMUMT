from __future__ import annotations

import json
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

from modules.common import agent_status_snapshot, db_paths
from modules.mission_planning.prior_mission_pipeline_impl import (
    _inject_prior_waypoint,
    _load_latest_mission_progress_plan_id,
    _resolve_plan_artifacts,
    _scan_latest_source_plan_id,
    _next_imp_id,
    _next_individual_mission_id,
    _next_path_id,
    _next_waypoint_id,
)

LogCallback = Callable[[str], None]

LOG_FILENAME = "log_attack_algorithm.json"


def run_attack_plan_pipeline(
    ctx: Dict[str, Any],
    log_callback: Optional[LogCallback] = None,
) -> Dict[str, Any]:
    """
    Execute the specialized attack-planning pre-processing flow.
    Returns a dictionary that is also persisted to DSS_Internal/log_attack_algorithm.json.
    """

    log_messages: List[str] = []

    def _emit(message: str) -> None:
        log_messages.append(message)
        if log_callback:
            log_callback(f"[ATTACK] {message}")

    attack_log: Dict[str, Any] = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "context": _json_safe(
            {
                "reason": ctx.get("reason"),
                "replan_level": ctx.get("replan_level"),
                "mission_ids": ctx.get("mission_ids"),
                "option_names": ctx.get("option_names"),
                "replan_detail": ctx.get("replan_detail"),
            }
        ),
        "steps": [],
        "logMessages": log_messages,
        "result": {},
    }

    # Step 1) Select the manned aircraft (aircraft 2 or 3) with the greatest fuel.
    agent_snapshot = agent_status_snapshot.load_agent_status_snapshot() or {}
    agent_states = agent_snapshot.get("agent_states") or []
    best_aircraft, candidates = _select_preferred_manned_aircraft(agent_states)
    attack_log["result"]["manned_candidates"] = candidates
    attack_log["result"]["selected_aircraft"] = best_aircraft
    if best_aircraft:
        _emit(
            "STEP1 유인기 선택: "
            f"aircraft {best_aircraft['aircraft_id']} "
            f"(fuel={best_aircraft.get('fuel')}, "
            f"coord={_coord_text(best_aircraft.get('coordinate'))})"
        )
        attack_log["steps"].append(
            {
                "name": "select_manned_aircraft",
                "status": "ok",
                "selected": best_aircraft,
                "candidates": candidates,
            }
        )
    else:
        _emit("STEP1 유인기 선택 실패: latest_0401_agent_status.json 누락")
        attack_log["steps"].append(
            {
                "name": "select_manned_aircraft",
                "status": "error",
                "message": "latest_0401_agent_status.json unavailable or malformed",
                "candidates": candidates,
            }
        )

    # Step 2) Determine which UAVs are currently tracking targets.
    target_entries, target_error = _load_target_entries()
    attack_log["result"]["target_tracking"] = target_entries
    if target_entries:
        tracking_summary = ", ".join(
            f"watcher {entry.get('watcher_id')}→target {entry.get('target_id') or entry.get('key')}"
            for entry in target_entries
        )
        _emit(f"STEP2 무인기 추적 현황: {tracking_summary or '추적 없음'}")
        attack_log["steps"].append(
            {
                "name": "analyze_uav_tracking",
                "status": "ok",
                "entries": target_entries,
            }
        )
    else:
        _emit(f"STEP2 무인기 추적 정보 없음: {target_error or 'targetInfo.json missing'}")
        attack_log["steps"].append(
            {
                "name": "analyze_uav_tracking",
                "status": "warn",
                "message": target_error or "targetInfo.json missing",
            }
        )

    # Step 3) Attempt to build an attack mission snapshot using lah_attack_assistance.
    friendly_coord = (best_aircraft or {}).get("coordinate")
    primary_target = _pick_primary_target(target_entries)
    attack_log["result"]["primary_target"] = primary_target
    if not friendly_coord:
        attack_log["steps"].append(
            {
                "name": "generate_attack_point",
                "status": "error",
                "message": "Missing friendly coordinate (cannot derive aircraft position)",
            }
        )
        _emit("STEP3 공격 임무 생성 실패: 유인기 좌표 없음")
        return _persist_attack_log(attack_log)
    if not primary_target or not primary_target.get("coordinate"):
        attack_log["steps"].append(
            {
                "name": "generate_attack_point",
                "status": "warn",
                "message": "No active target with coordinates found.",
            }
        )
        _emit("STEP3 공격 임무 생성 보류: 추적 중 표적 좌표 없음")
        return _persist_attack_log(attack_log)

    attack_point, attack_error = _compute_attack_point(
        friendly_coord,
        primary_target["coordinate"],
    )
    attack_log["result"]["attack_point"] = attack_point
    mission_updates: Optional[Dict[str, Any]] = None
    if attack_point:
        _emit(
            "STEP3 공격 임무 생성 완료: "
            f"lat={attack_point['latitude']:.6f}, lon={attack_point['longitude']:.6f}, "
            f"alt≈{attack_point['altitude']:.1f}m"
        )
        attack_log["steps"].append(
            {
                "name": "generate_attack_point",
                "status": "ok",
                "attack_point": attack_point,
            }
        )
        mission_updates = _apply_attack_plan_overrides(
            ctx=ctx,
            attack_point=attack_point,
            manned_aircraft=best_aircraft,
            primary_target=primary_target,
            agent_states=agent_states,
            emit=_emit,
        )
        if mission_updates:
            attack_log["result"]["missionUpdates"] = mission_updates
    else:
        _emit(f"STEP3 공격 임무 생성 실패: {attack_error}")
        attack_log["steps"].append(
            {
                "name": "generate_attack_point",
                "status": "error",
                "message": attack_error,
            }
        )

    return _persist_attack_log(attack_log)


def _select_preferred_manned_aircraft(agent_states: List[Any]) -> Tuple[Optional[Dict[str, Any]], List[Dict[str, Any]]]:
    candidates: List[Dict[str, Any]] = []
    for state in agent_states:
        aircraft_id = _to_int(
            (state.get("aircraftID") if isinstance(state, dict) else None)
            or (state.get("aircraftId") if isinstance(state, dict) else None)
        )
        if aircraft_id not in (2, 3):
            continue
        is_unmanned = _to_bool(state.get("isUnmanned")) if isinstance(state, dict) else None
        if is_unmanned:
            continue
        fuel = _to_float(state.get("fuel")) if isinstance(state, dict) else None
        coordinate = _normalize_coordinate(state.get("coordinate")) if isinstance(state, dict) else None
        candidates.append(
            {
                "aircraft_id": aircraft_id,
                "fuel": fuel,
                "coordinate": coordinate,
            }
        )
    if not candidates:
        return None, candidates
    candidates.sort(
        key=lambda item: (
            item["fuel"] is not None,
            item["fuel"] if item["fuel"] is not None else float("-inf"),
        ),
        reverse=True,
    )
    return candidates[0], candidates


def _load_target_entries() -> Tuple[List[Dict[str, Any]], Optional[str]]:
    target_entries: List[Dict[str, Any]] = []
    target_path = db_paths.get_db_subpath("DSS_Internal") / "targetInfo.json"
    if not target_path.exists():
        return [], f"{target_path} does not exist"
    try:
        raw = json.loads(target_path.read_text(encoding="utf-8"))
    except Exception as exc:
        return [], f"Failed to parse targetInfo.json: {exc}"
    target_map = raw.get("targetList") if isinstance(raw, dict) else None
    if not isinstance(target_map, dict):
        return [], "targetInfo.json lacks targetList"
    for key, value in target_map.items():
        entry = value or {}
        target_id = _to_int(entry.get("targetID"))
        watcher_id = _to_int(entry.get("watcherID"))
        if watcher_id is None:
            watcher_field = entry.get("watcher")
            if isinstance(watcher_field, dict):
                watcher_id = _to_int(
                    watcher_field.get("aircraftID")
                    or watcher_field.get("watcherID")
                    or watcher_field.get("id")
                )
            else:
                watcher_id = _to_int(watcher_field)
        if watcher_id is None:
            watcher_id = _extract_watcher_from_key(str(key))
        target_entries.append(
            {
                "key": str(key),
                "target_id": target_id,
                "watcher_id": watcher_id,
                "coordinate": _normalize_coordinate(entry.get("coordinate")),
                "is_destroyed": bool(entry.get("isDestroyed")),
                "is_used": _to_int(entry.get("isUsed")),
                "target_in_frame": bool(entry.get("targetInFrame")),
                "threat": _to_float(entry.get("threat")),
                "raw": entry,
            }
        )
    target_entries.sort(
        key=lambda item: (
            not item["is_destroyed"],
            item["target_in_frame"],
            item["is_used"] == 0,
            item["target_id"] if item["target_id"] is not None else -1,
        ),
        reverse=True,
    )
    return target_entries, None


def _pick_primary_target(target_entries: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    for entry in target_entries:
        if entry["is_destroyed"]:
            continue
        if entry["target_in_frame"] and entry.get("coordinate"):
            return entry
    for entry in target_entries:
        if not entry["is_destroyed"] and entry.get("coordinate"):
            return entry
    return target_entries[0] if target_entries else None


def _compute_attack_point(
    friendly_coord: Dict[str, Any],
    enemy_coord: Dict[str, Any],
) -> Tuple[Optional[Dict[str, Any]], Optional[str]]:
    friendly = _coordinate_to_world(friendly_coord)
    enemy = _coordinate_to_world(enemy_coord)
    if not friendly or not enemy:
        return None, "Insufficient coordinate data for attack calculation."
    try:
        from modules.mission_planning.MissionPlanner.data_def import (
            lah_attack_assistance as attack_assist,
        )
    except SystemExit as exc:
        return None, f"GDAL/Shapely dependencies missing: {exc}"
    except Exception as exc:
        return None, f"Failed to import lah_attack_assistance: {exc}"

    try:
        raster_paths = attack_assist.detect_raster_paths()
        elevation, geotransform, used_rasters = attack_assist.load_elevation(
            raster_paths,
            enemy,
            radius_m=attack_assist.ANALYSIS_RADIUS_METERS,
        )
        enemy_px = attack_assist.ensure_point_inside(enemy, geotransform, elevation)
        arc = attack_assist.compute_cover_disk(
            elevation,
            geotransform,
            enemy_px,
            radius_m=attack_assist.ANALYSIS_RADIUS_METERS,
            num_rays=attack_assist.NUM_ARC_RAYS,
        )
        cell_data = attack_assist.compute_cell_data(arc)
        polygons = attack_assist.build_danger_polygons(
            cell_data,
            arc.world_x,
            arc.world_y,
            arc,
            geotransform,
        )
        if not polygons:
            return None, "No LOS polygons generated from terrain data."
        best = attack_assist.choose_attack_point(
            polygons,
            friendly,
            enemy,
            geotransform,
        )
        if not best:
            return None, "Attack candidate selection failed."
        altitude = attack_assist.sample_elevation_at_world(
            elevation,
            best["centroid"],
            geotransform,
        )
        return (
            {
                "latitude": best["centroid"][1],
                "longitude": best["centroid"][0],
                "altitude": altitude,
                "friendly_distance_m": best["friendly_distance"],
                "enemy_distance_m": best["enemy_distance"],
                "raster_sources": used_rasters,
            },
            None,
        )
    except Exception as exc:
        return None, f"Attack point computation error: {exc}"


def _persist_attack_log(payload: Dict[str, Any]) -> Dict[str, Any]:
    directory = db_paths.get_db_subpath("DSS_Internal")
    directory.mkdir(parents=True, exist_ok=True)
    target_path = directory / LOG_FILENAME
    try:
        target_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    except Exception as exc:
        if isinstance(payload.setdefault("steps", []), list):
            payload["steps"].append(
                {
                    "name": "persist_log",
                    "status": "error",
                    "message": f"Failed to write {target_path}: {exc}",
                }
            )
        return payload
    payload["log_path"] = str(target_path)
    payload.setdefault("logMessages", []).append(f"[LOG] Saved attack analysis → {target_path}")
    return payload


def _normalize_coordinate(value: Any) -> Optional[Dict[str, float]]:
    if not isinstance(value, dict):
        return None
    lat = _to_float(value.get("latitude") or value.get("lat"))
    lon = _to_float(value.get("longitude") or value.get("lon"))
    alt = _to_float(value.get("altitude") or value.get("alt"))
    if lat is None or lon is None:
        return None
    return {"latitude": lat, "longitude": lon, "altitude": alt}


def _coordinate_to_world(coord: Dict[str, Any]) -> Optional[Tuple[float, float]]:
    if not coord:
        return None
    lat = _to_float(coord.get("latitude"))
    lon = _to_float(coord.get("longitude"))
    if lat is None or lon is None:
        return None
    return (lon, lat)


def _coord_text(coord: Optional[Dict[str, Any]]) -> str:
    if not coord:
        return "-"
    return f"({coord.get('latitude')}, {coord.get('longitude')}, alt={coord.get('altitude')})"


def _extract_watcher_from_key(key: str) -> Optional[int]:
    if "-" not in key:
        return None
    try:
        _, watcher = key.split("-", 1)
        return int(watcher)
    except Exception:
        return None


def _json_safe(value: Any) -> Any:
    try:
        json.dumps(value, ensure_ascii=False)
        return value
    except TypeError:
        if isinstance(value, dict):
            return {str(k): _json_safe(v) for k, v in value.items()}
        if isinstance(value, (list, tuple, set)):
            return [_json_safe(v) for v in value]
        return str(value)


def _to_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _to_int(value: Any) -> Optional[int]:
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _to_bool(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"true", "1", "y", "yes"}:
            return True
        if lowered in {"false", "0", "n", "no"}:
            return False
    return None


def _apply_attack_plan_overrides(
    *,
    ctx: Dict[str, Any],
    attack_point: Dict[str, Any],
    manned_aircraft: Optional[Dict[str, Any]],
    primary_target: Optional[Dict[str, Any]],
    agent_states: List[Any],
    emit: Callable[[str], None],
) -> Optional[Dict[str, Any]]:
    if not attack_point or not manned_aircraft or not primary_target:
        emit("[ATTACK] Mission override skipped (insufficient attack metadata).")
        return None

    manned_id = _to_int(manned_aircraft.get("aircraft_id"))
    watcher_id = _to_int(primary_target.get("watcher_id"))
    if manned_id is None or watcher_id is None:
        emit("[ATTACK] Mission override skipped (aircraft IDs missing).")
        return None

    agent_index = _index_agent_states(agent_states)
    manned_state = agent_index.get(manned_id)
    uav_state = agent_index.get(watcher_id)
    if not manned_state or not manned_state.get("current_waypoint_id"):
        emit(f"[ATTACK] Current waypoint unavailable for manned aircraft {manned_id}.")
        return None
    if not uav_state or not uav_state.get("current_waypoint_id"):
        emit(f"[ATTACK] Current waypoint unavailable for UAV {watcher_id}.")
        return None

    source_plan_id = _load_latest_mission_progress_plan_id() or _scan_latest_source_plan_id()
    if source_plan_id is None:
        emit("[ATTACK] Mission override skipped (no MissionPlan found).")
        return None

    plan_id_candidates = [
        _to_int(value) for value in ctx.get("plan_ids") or [] if _to_int(value) is not None
    ]
    new_plan_id = plan_id_candidates[0] if plan_id_candidates else source_plan_id

    try:
        plan_src = db_paths.get_db_subpath("MissionPlan", f"{int(source_plan_id)}.json")
        plan_data = json.loads(plan_src.read_text(encoding="utf-8"))
    except Exception as exc:
        emit(f"[ATTACK] MissionPlan {source_plan_id} load failed: {exc}")
        return None

    new_plan_data = deepcopy(plan_data)
    now_ms = _now_timestamp_ms()
    new_plan_data["missionPlanID"] = new_plan_id
    new_plan_data["timestamp"] = now_ms
    if "missionPlanTimestamp" in new_plan_data:
        new_plan_data["missionPlanTimestamp"] = now_ms

    attack_coord = {
        "latitude": attack_point.get("latitude"),
        "longitude": attack_point.get("longitude"),
        "altitude": attack_point.get("altitude"),
    }
    uav_target_coord = primary_target.get("coordinate") or attack_coord

    descriptors = [
        {
            "label": "manned",
            "aircraft_id": manned_id,
            "state": manned_state,
            "target_coord": attack_coord,
            "target_id": primary_target.get("target_id"),
            "mode": "LAH",
        },
        {
            "label": "uav",
            "aircraft_id": watcher_id,
            "state": uav_state,
            "target_coord": uav_target_coord,
            "target_id": primary_target.get("target_id"),
            "mode": "UAV",
        },
    ]

    aircraft_updates: List[Dict[str, Any]] = []
    for descriptor in descriptors:
        aircraft_id = descriptor["aircraft_id"]
        state = descriptor["state"] or {}
        current_wp = _to_int(state.get("current_waypoint_id"))
        previous_wp = _to_int(state.get("previous_waypoint_id"))
        if aircraft_id is None or current_wp is None:
            emit(f"[ATTACK] {descriptor['label']} aircraft lacks waypoint context; skipping.")
            continue
        artifacts = _resolve_plan_artifacts(
            source_plan_id=source_plan_id,
            aircraft_id=aircraft_id,
            current_waypoint_id=current_wp,
            emit=emit,
        )
        if artifacts is None:
            continue
        try:
            imp_src = db_paths.get_db_subpath(
                "IndividualMissionPlan", f"{artifacts.individual_mission_package_id}.json"
            )
            fp_src = db_paths.get_db_subpath("FlightPath", f"{artifacts.path_id}.json")
            imp_data = json.loads(imp_src.read_text(encoding="utf-8"))
            fp_data = json.loads(fp_src.read_text(encoding="utf-8"))
        except Exception as exc:
            emit(f"[ATTACK] Failed to load artifacts for aircraft {aircraft_id}: {exc}")
            continue

        new_imp_id = _next_imp_id()
        new_path_id = _next_path_id(aircraft_id)
        new_individual_id = _next_individual_mission_id()
        new_waypoint_id = _next_waypoint_id()
        emit(
            f"[ATTACK][STEP4-{descriptor['label']}] "
            f"IDs 할당 → imp:{new_imp_id} path:{new_path_id} indiv:{new_individual_id} wp:{new_waypoint_id}"
        )

        if not _update_plan_aircraft_entry(new_plan_data, aircraft_id, new_imp_id, emit):
            continue

        new_imp_data = deepcopy(imp_data)
        target_mission = None
        for mission in new_imp_data.get("individualMissionList", []):
            if _to_int(mission.get("individualMissionID")) == artifacts.individual_mission_id:
                target_mission = mission
                break
        if target_mission is None:
            emit(
                f"[ATTACK] Individual mission {artifacts.individual_mission_id} "
                f"not found for aircraft {aircraft_id}."
            )
            continue
        target_mission["individualMissionID"] = new_individual_id
        target_mission["pathID"] = new_path_id
        target_mission["isDone"] = False
        rel_block = dict(target_mission.get("relatedMission") or {})
        rel_block["relatedMissionType"] = 3
        rel_block.setdefault("inputMissionID", _to_int((ctx.get("mission_ids") or [None])[0]) or 0)
        rel_block["attackReason"] = ctx.get("reason")
        target_mission["relatedMission"] = rel_block
        new_imp_data["individualMissionPackageID"] = new_imp_id

        new_fp_data = deepcopy(fp_data)
        new_fp_data["pathID"] = new_path_id
        new_fp_data["timestamp"] = now_ms
        new_fp_data["Source"] = new_fp_data.get("Source") or "MMR"

        removed_wp_id: Optional[int] = None
        inserted_wp: Optional[Dict[str, Any]] = None
        try:
            if descriptor["mode"] == "UAV":
                removed_wp_id, inserted_wp = _inject_prior_waypoint(
                    new_fp_data,
                    artifacts.current_waypoint_id,
                    artifacts.previous_waypoint_id,
                    descriptor["target_coord"],
                    new_waypoint_id,
                    mission_type=3,
                    target_tracking={"targetID": descriptor.get("target_id")},
                )
            else:
                inserted_wp = _insert_lah_attack_waypoint(
                    new_fp_data,
                    artifacts.current_waypoint_id,
                    attack_coord,
                    new_waypoint_id,
                    descriptor.get("target_id"),
                )
        except Exception as exc:
            emit(f"[ATTACK] Waypoint injection failed for aircraft {aircraft_id}: {exc}")
            continue

        imp_dest = db_paths.get_db_subpath("IndividualMissionPlan", f"{new_imp_id}.json")
        fp_dest = db_paths.get_db_subpath("FlightPath", f"{new_path_id}.json")
        imp_dest.parent.mkdir(parents=True, exist_ok=True)
        fp_dest.parent.mkdir(parents=True, exist_ok=True)
        _write_json_file(imp_dest, new_imp_data)
        _write_json_file(fp_dest, new_fp_data)
        emit(
            f"[ATTACK][ARTIFACT] aircraft {aircraft_id} → "
            f"IMP:{imp_dest.name} PATH:{fp_dest.name}"
        )

        aircraft_updates.append(
            {
                "aircraft_id": aircraft_id,
                "role": descriptor["label"],
                "individualMissionPackageID": new_imp_id,
                "individualMissionID": new_individual_id,
                "pathID": new_path_id,
                "waypointID": new_waypoint_id,
                "removedWaypointID": removed_wp_id,
                "flightPath": str(fp_dest),
                "missionPackage": str(imp_dest),
                "insertedWaypoint": inserted_wp,
            }
        )

    if not aircraft_updates:
        emit("[ATTACK] Mission override produced no artifacts.")
        return None

    plan_dest = db_paths.get_db_subpath("MissionPlan", f"{new_plan_id}.json")
    plan_dest.parent.mkdir(parents=True, exist_ok=True)
    _write_json_file(plan_dest, new_plan_data)
    emit(f"[ATTACK][PLAN] MissionPlan 저장 → {plan_dest.name} (planID={new_plan_id})")

    plan_meta_map = dict(ctx.get("_option_meta") or {})
    plan_meta_entry = plan_meta_map.setdefault(new_plan_id, {})
    plan_meta_entry.update(
        {
            "attack": True,
            "attackTargets": {
                "targetID": primary_target.get("target_id"),
                "watcherID": watcher_id,
                "attackPoint": attack_coord,
            },
        }
    )
    ctx["_option_meta"] = plan_meta_map

    return {
        "source_plan_id": source_plan_id,
        "mission_plan_id": new_plan_id,
        "plan_path": str(plan_dest),
        "aircraft": aircraft_updates,
    }


def _index_agent_states(agent_states: List[Any]) -> Dict[int, Dict[str, Any]]:
    index: Dict[int, Dict[str, Any]] = {}
    for entry in agent_states:
        if not isinstance(entry, dict):
            continue
        aircraft_id = _to_int(entry.get("aircraftID") or entry.get("aircraftId"))
        if aircraft_id is None:
            continue
        coord = (
            _normalize_coordinate(entry.get("coordinate"))
            or _normalize_coordinate((entry.get("mannedInfo") or {}).get("coordinate"))
            or _normalize_coordinate((entry.get("unmannedInfo") or {}).get("coordinate"))
        )
        wp_block = entry.get("currentWaypointID") or {}
        if not wp_block:
            unm_info = entry.get("unmannedInfo") or {}
            wp_block = unm_info.get("currentWaypointID") or {}
        current_wp = _to_int(wp_block.get("waypointID"))
        index[aircraft_id] = {
            "aircraft_id": aircraft_id,
            "coordinate": coord,
            "current_waypoint_id": current_wp,
            "is_unmanned": bool(entry.get("isUnmanned")),
        }
    return index


def _insert_lah_attack_waypoint(
    flight_path: Dict[str, Any],
    current_waypoint_id: int,
    attack_coord: Dict[str, Any],
    new_waypoint_id: int,
    target_id: Optional[int],
) -> Dict[str, Any]:
    waypoints = list(flight_path.get("lahWaypointList") or [])
    if not waypoints:
        raise ValueError("LAH flight path is empty.")
    current_index = next(
        (idx for idx, wp in enumerate(waypoints) if _to_int(wp.get("waypointID")) == current_waypoint_id),
        None,
    )
    if current_index is None:
        raise ValueError(f"Current waypoint {current_waypoint_id} not found in LAH flight path.")

    template = deepcopy(waypoints[current_index])
    prev_index = current_index - 1
    if prev_index >= 0 and "nextWaypointID" in waypoints[prev_index]:
        waypoints[prev_index]["nextWaypointID"] = new_waypoint_id

    coordinate = {
        "latitude": attack_coord.get("latitude"),
        "longitude": attack_coord.get("longitude"),
        "altitude": attack_coord.get("altitude")
        or (template.get("coordinate") or {}).get("altitude")
        or 800.0,
    }
    new_wp = {
        "waypointID": new_waypoint_id,
        "coordinate": coordinate,
        "speed": template.get("speed", 40),
        "eta": template.get("eta", 0),
        "ecf": template.get("ecf", 0.0),
        "nextWaypointID": current_waypoint_id,
        "hovering": deepcopy(template.get("hovering") or {"time": 0}),
        "loiter": deepcopy(template.get("loiter") or {"radius": 0, "direction": 0, "time": 0, "speed": 0}),
        "attack": deepcopy(template.get("attack") or {}),
    }
    attack_block = new_wp["attack"] or {}
    attack_block["targetID"] = target_id or 0
    attack_block["weaponType"] = attack_block.get("weaponType") or 1
    new_wp["attack"] = attack_block

    waypoints.insert(current_index, new_wp)
    flight_path["lahWaypointList"] = waypoints
    return new_wp


def _update_plan_aircraft_entry(
    plan_data: Dict[str, Any],
    aircraft_id: int,
    new_package_id: int,
    emit: Callable[[str], None],
) -> bool:
    for entry in plan_data.get("aircraftList", []):
        if _to_int(entry.get("aircraftID")) == aircraft_id:
            entry["individualMissionPackageID"] = new_package_id
            return True
    emit(f"[ATTACK] Aircraft {aircraft_id} not found in mission plan.")
    return False


def _write_json_file(path: Path, data: Dict[str, Any]) -> None:
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def _now_timestamp_ms() -> int:
    epoch = datetime(2000, 1, 1, tzinfo=timezone.utc)
    return int((datetime.now(timezone.utc) - epoch).total_seconds() * 1000)
