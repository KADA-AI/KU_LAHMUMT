from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterable

from modules.common import db_paths


def _coerce_int(value: Any) -> int | None:
    try:
        return int(value)
    except Exception:
        return None


def _coerce_bool(value: Any, default: bool = False) -> bool:
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


def _load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def _extract_waypoints(data: dict[str, Any]) -> list[dict[str, Any]]:
    for key in ("lahWaypointList", "uavWaypointList", "waypointList"):
        lst = data.get(key)
        if isinstance(lst, list):
            return lst
    return []


def _extract_coord(item: dict[str, Any]) -> tuple[float, float, float | None] | None:
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


def _order_waypoints(raw: list[dict[str, Any]]) -> list[dict[str, Any]]:
    if not raw:
        return []
    by_id: dict[int, dict[str, Any]] = {}
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

    ordered: list[dict[str, Any]] = []
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


def _agent_label(aircraft_id: int) -> str:
    if 1 <= aircraft_id <= 3:
        return f"LAH{aircraft_id}"
    if 4 <= aircraft_id <= 6:
        return f"UAV{aircraft_id - 3}"
    return f"AC{aircraft_id}"


def _latest_reference_info(folder: Path) -> dict[str, Any]:
    if not folder.exists():
        return {}
    best = None
    best_ts = None
    for path in folder.glob("*.json"):
        data = _load_json(path)
        if not data:
            continue
        ts = _coerce_int(data.get("timestamp"))
        if ts is None:
            ts = _coerce_int(path.stem)
        if ts is None:
            continue
        if best is None or ts >= (best_ts or -1):
            best = data
            best_ts = ts
    return best or {}


def build_features_from_flight_paths(
    flight_paths: Iterable[dict[str, Any]],
    *,
    done_path_ids: set[int] | None = None,
) -> tuple[list[dict[str, Any]], dict[str, int]]:
    features: list[dict[str, Any]] = []
    agent_counts: dict[str, int] = {}
    feature_id = 1
    done_paths = done_path_ids or set()

    for entry in flight_paths:
        if not isinstance(entry, dict):
            continue
        data = entry.get("data") if isinstance(entry.get("data"), dict) else entry
        if not isinstance(data, dict):
            continue
        aircraft_id = data.get("aircraftID") or data.get("AircraftID")
        path_id = data.get("pathID") or data.get("PathID")
        try:
            aircraft_id = int(aircraft_id)
        except Exception:
            aircraft_id = -1
        try:
            path_id = int(path_id)
        except Exception:
            path_id = None

        waypoints = _extract_waypoints(data)
        if not waypoints:
            continue
        waypoints = _order_waypoints(waypoints)

        coords: list[list[float]] = []
        alts: list[float | None] = []
        wp_ids: list[int | None] = []
        alt_values: list[float] = []
        for wp in waypoints:
            if not isinstance(wp, dict):
                continue
            coord = _extract_coord(wp)
            if coord is None:
                continue
            lat, lon, alt = coord
            coords.append([float(lon), float(lat)])
            wp_id = wp.get("waypointID") or wp.get("WaypointID")
            try:
                wp_id = int(wp_id) if wp_id is not None else None
            except Exception:
                wp_id = None
            wp_ids.append(wp_id)
            alts.append(float(alt) if alt is not None else None)
            if alt is not None:
                alt_values.append(float(alt))

        if len(coords) < 1:
            continue

        agent = _agent_label(aircraft_id)
        agent_counts[agent] = agent_counts.get(agent, 0) + 1
        feature = {
            "id": feature_id,
            "agent": agent,
            "aircraftId": aircraft_id,
            "pathId": path_id,
            "isDone": bool(path_id is not None and path_id in done_paths),
            "points": len(coords),
            "coords": coords,
            "alts": alts,
            "wpIds": wp_ids,
            "altMin": min(alt_values) if alt_values else None,
            "altMax": max(alt_values) if alt_values else None,
        }
        feature_id += 1
        features.append(feature)

    return features, agent_counts


def build_mission_plan_payload(
    mission_plan_id: int | None,
    *,
    db_root: Path | None = None,
) -> dict[str, Any]:
    if mission_plan_id is None:
        return {"ok": False, "error": "missionPlanID required"}
    if db_root is None:
        db_root = db_paths.get_active_db_root()
    base = Path(db_root)

    plan_path = base / "MissionPlan" / f"{int(mission_plan_id)}.json"
    mission_plan = _load_json(plan_path)
    if not mission_plan:
        return {
            "ok": False,
            "error": f"MissionPlan {mission_plan_id} not found",
            "planPath": str(plan_path),
        }

    input_package_id = _coerce_int(
        mission_plan.get("inputMissionPackageID")
        or mission_plan.get("InputMissionPackageID")
        or mission_plan.get("inputMissionPackageId")
    )
    input_plan = (
        _load_json(base / "InputMissionPlan" / f"{int(input_package_id)}.json")
        if input_package_id is not None
        else {}
    )
    input_plans = [input_plan] if input_plan else []

    aircraft_list = mission_plan.get("aircraftList") or []
    individual_plans: list[dict[str, Any]] = []
    path_ids: set[int] = set()
    individual_package_ids: list[int] = []
    for entry in aircraft_list:
        if not isinstance(entry, dict):
            continue
        package_id = _coerce_int(
            entry.get("individualMissionPackageID")
            or entry.get("individualMissionPackageId")
            or entry.get("IndividualMissionPackageID")
        )
        if package_id is None:
            continue
        individual_package_ids.append(package_id)
        plan = _load_json(base / "IndividualMissionPlan" / f"{int(package_id)}.json")
        if not plan:
            continue
        individual_plans.append(plan)
        for mission in plan.get("individualMissionList") or []:
            if not isinstance(mission, dict):
                continue
            pid = _coerce_int(mission.get("pathID") or mission.get("PathID"))
            if pid is not None:
                path_ids.add(pid)

    flight_paths: list[dict[str, Any]] = []
    missing_paths: list[int] = []
    for pid in sorted(path_ids):
        data = _load_json(base / "FlightPath" / f"{int(pid)}.json")
        if data:
            flight_paths.append(data)
        else:
            missing_paths.append(pid)

    mission_ref_id = _coerce_int(
        mission_plan.get("missionReferencePackageID")
        or mission_plan.get("missionReferencePackageId")
        or mission_plan.get("MissionReferencePackageID")
    )
    mission_ref_info = {}
    if mission_ref_id is not None:
        mission_ref_info = _load_json(base / "MissionReferenceInfo" / f"{int(mission_ref_id)}.json")
    if not mission_ref_info:
        mission_ref_info = _latest_reference_info(base / "MissionReferenceInfo")
    take_over_list = (
        mission_ref_info.get("takeOverInfoList") if isinstance(mission_ref_info, dict) else []
    )
    if not isinstance(take_over_list, list):
        take_over_list = []

    path_done_map: dict[int, bool] = {}
    for plan in individual_plans:
        if not isinstance(plan, dict):
            continue
        for mission in plan.get("individualMissionList") or []:
            if not isinstance(mission, dict):
                continue
            pid = _coerce_int(mission.get("pathID") or mission.get("PathID"))
            if pid is not None:
                mission_done = _coerce_bool(mission.get("isDone"), False)
                prev_done = path_done_map.get(pid)
                if prev_done is None:
                    path_done_map[pid] = mission_done
                else:
                    # If one mission referencing this path is active, keep it active.
                    path_done_map[pid] = bool(prev_done and mission_done)

    done_path_ids = {pid for pid, is_done in path_done_map.items() if is_done}

    features, agent_counts = build_features_from_flight_paths(
        flight_paths,
        done_path_ids=done_path_ids,
    )

    payload = {
        "flightPaths": flight_paths,
        "inputMissionPlans": input_plans,
        "individualMissionPlans": individual_plans,
        "takeOverInfoList": take_over_list,
    }

    return {
        "ok": True,
        "missionPlanID": int(mission_plan_id),
        "inputMissionPackageID": input_package_id,
        "missionReferencePackageID": mission_ref_id,
        "individualMissionPackageIDs": individual_package_ids,
        "missionPlan": mission_plan,
        "inputMissionPlans": input_plans,
        "individualMissionPlans": individual_plans,
        "flightPaths": flight_paths,
        "flightPathCount": len(flight_paths),
        "missingPathIds": missing_paths,
        "features": features,
        "agents": agent_counts,
        "count": len(features),
        "payload": payload,
    }
