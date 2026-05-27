from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Set

from modules.common import agent_status_snapshot, db_paths
from modules.mission_planning.pipelines.next_collab_replan_pipeline_impl import (
    prepare_next_collab_input_replacements,
)

_EPOCH2000_MS = 946684800000
_TRACKED_UAV_IDS = (4, 5, 6)


def _now_ms_since_2000() -> int:
    return int(time.time() * 1000) - _EPOCH2000_MS


def _to_int(value: Any) -> int | None:
    try:
        if value is None:
            return None
        return int(value)
    except Exception:
        return None


def _to_float(value: Any) -> float | None:
    try:
        if value is None:
            return None
        return float(value)
    except Exception:
        return None


def _normalize_coordinate(payload: Any) -> Dict[str, float] | None:
    if not isinstance(payload, dict):
        return None
    lat = _to_float(payload.get("latitude") or payload.get("Latitude"))
    lon = _to_float(payload.get("longitude") or payload.get("Longitude"))
    alt = _to_float(payload.get("altitude") or payload.get("Altitude"))
    if lat is None or lon is None:
        return None
    out: Dict[str, float] = {
        "latitude": float(lat),
        "longitude": float(lon),
    }
    if alt is not None:
        out["altitude"] = int(round(float(alt)))
    return out


def _load_vehicle_status_available() -> Set[int]:
    try:
        status_path = db_paths.get_db_subpath("VehicleStatus", "status.json")
    except Exception:
        return set()
    if not status_path.exists():
        return set()
    try:
        payload = json.loads(status_path.read_text(encoding="utf-8"))
    except Exception:
        return set()
    raw = payload.get("available")
    if not isinstance(raw, list):
        return set()
    available: Set[int] = set()
    for item in raw:
        value = _to_int(item)
        if value is None or value <= 0:
            continue
        available.add(int(value))
    return available


def _extract_unmanned_entry_rows(
    snapshot_payload: Dict[str, Any] | None,
    *,
    allowed_ids: Set[int],
) -> tuple[Dict[int, Dict[str, Any]], Dict[int, float]]:
    entry_coord_map: Dict[int, Dict[str, Any]] = {}
    heading_map: Dict[int, float] = {}
    if not isinstance(snapshot_payload, dict):
        return entry_coord_map, heading_map
    states = snapshot_payload.get("agent_states") or snapshot_payload.get("raw", {}).get("agentStateList") or []
    for entry in states:
        if not isinstance(entry, dict):
            continue
        aircraft_id = _to_int(
            entry.get("aircraftID")
            if entry.get("aircraftID") is not None
            else entry.get("aircraft_id")
        )
        if aircraft_id is None or aircraft_id not in allowed_ids:
            continue
        is_unmanned = entry.get("isUnmanned")
        if is_unmanned is not None:
            if isinstance(is_unmanned, bool):
                if not is_unmanned:
                    continue
            elif _to_int(is_unmanned) != 1:
                continue
        coord = _normalize_coordinate(entry.get("coordinate"))
        if coord is None:
            unmanned_info = entry.get("unmannedInfo") if isinstance(entry.get("unmannedInfo"), dict) else {}
            coord = _normalize_coordinate(unmanned_info.get("coordinate"))
        if coord is None:
            continue
        entry_coord_map[int(aircraft_id)] = coord
        heading = None
        velocity = entry.get("velocity") if isinstance(entry.get("velocity"), dict) else {}
        if velocity:
            heading = _to_float(
                velocity.get("heading")
                if velocity.get("heading") is not None
                else velocity.get("Heading")
            )
        if heading is None:
            heading = _to_float(entry.get("heading") if entry.get("heading") is not None else entry.get("Heading"))
        if heading is not None:
            heading_map[int(aircraft_id)] = float(heading) % 360.0
    return entry_coord_map, heading_map


def _centroid_coordinate(coords: List[Dict[str, Any]]) -> Dict[str, Any] | None:
    if not coords:
        return None
    lat_vals = [float(item["latitude"]) for item in coords if "latitude" in item]
    lon_vals = [float(item["longitude"]) for item in coords if "longitude" in item]
    if not lat_vals or not lon_vals:
        return None
    out: Dict[str, Any] = {
        "latitude": sum(lat_vals) / float(len(lat_vals)),
        "longitude": sum(lon_vals) / float(len(lon_vals)),
    }
    alt_vals = [float(item["altitude"]) for item in coords if "altitude" in item]
    if alt_vals:
        out["altitude"] = int(round(sum(alt_vals) / float(len(alt_vals))))
    return out


@dataclass
class CurrentRemainingHybridResult:
    prepared: Any
    current_input_id: int
    target_aircraft_ids: List[int]
    entry_coord_map: Dict[int, Dict[str, Any]]
    heading_map: Dict[int, float]


def prepare_current_remaining_hybrid_replacements(
    *,
    source_plan_id: int,
    current_input_mission: Dict[str, Any],
    next_input_mission: Dict[str, Any] | None = None,
    log: Callable[[str], None] | None = None,
) -> Optional[CurrentRemainingHybridResult]:
    emit = log or (lambda _msg: None)
    current_input_id = _to_int(current_input_mission.get("inputMissionID"))
    if current_input_id is None or current_input_id <= 0:
        emit("[CURRENT-HYBRID] current input mission ID is invalid.")
        return None

    available_ids = {
        int(aid)
        for aid in _load_vehicle_status_available()
        if int(aid) in _TRACKED_UAV_IDS
    }
    if not available_ids:
        emit("[CURRENT-HYBRID] no available UAVs from VehicleStatus.")
        return None

    agent_snapshot = agent_status_snapshot.load_agent_status_snapshot()
    entry_coord_map, heading_map = _extract_unmanned_entry_rows(
        agent_snapshot,
        allowed_ids=available_ids,
    )
    if not entry_coord_map:
        emit("[CURRENT-HYBRID] no live UAV coordinates in latest 0401 snapshot.")
        return None

    representative_entry = _centroid_coordinate(list(entry_coord_map.values()))
    prepared = prepare_next_collab_input_replacements(
        source_plan_id=int(source_plan_id),
        target_input_mission=current_input_mission,
        entry_coord_map=entry_coord_map,
        heading_map=heading_map,
        representative_entry=representative_entry,
        next_input_mission=next_input_mission,
        turn_radius_scale=None,
        now_ms=_now_ms_since_2000(),
        log=emit,
    )
    if prepared is None:
        return None

    target_aircraft_ids = sorted(
        int(aid)
        for aid in getattr(prepared, "replacement_by_aircraft", {}).keys()
        if _to_int(aid) is not None
    )
    if not target_aircraft_ids:
        emit("[CURRENT-HYBRID] next-collab helper returned no target aircraft replacements.")
        return None

    return CurrentRemainingHybridResult(
        prepared=prepared,
        current_input_id=int(current_input_id),
        target_aircraft_ids=target_aircraft_ids,
        entry_coord_map={int(aid): dict(coord) for aid, coord in entry_coord_map.items()},
        heading_map={int(aid): float(val) for aid, val in heading_map.items()},
    )
