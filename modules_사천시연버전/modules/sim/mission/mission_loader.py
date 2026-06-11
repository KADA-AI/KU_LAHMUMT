from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from .mission_plan_loader import (
    _build_sweep_line_spacing_summaries,
    _path_input_mission_index,
    normalize_input_mission_plan_float_fields,
)


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


def _resolve_path(path_str: str, root: Path) -> Path:
    if not path_str:
        return root
    expanded = Path(path_str).expanduser()
    if expanded.is_absolute():
        return expanded
    return (root / expanded).resolve()


def _find_flight_path_dir(base: Path) -> tuple[Optional[Path], List[Path]]:
    if not base.exists():
        return None, []
    if base.is_dir() and base.name.lower() == "flightpath":
        return base, [base]

    candidates: List[Path] = []
    direct = base / "FlightPath"
    if direct.is_dir():
        candidates.append(direct)

    sbc3 = base / "SBC3" / "FlightPath"
    if sbc3.is_dir():
        candidates.append(sbc3)

    # Shallow search (depth <= 2)
    try:
        base_parts = len(base.resolve().parts)
        for root, dirs, _files in os.walk(base):
            depth = len(Path(root).resolve().parts) - base_parts
            if depth > 2:
                dirs[:] = []
                continue
            for d in list(dirs):
                if d.lower() == "flightpath":
                    candidates.append(Path(root) / d)
    except Exception:
        pass

    if not candidates:
        return None, []

    # Prefer SBC3 if present
    for cand in candidates:
        if "SBC3" in {p.upper() for p in cand.parts}:
            return cand, candidates
    return candidates[0], candidates


def _agent_label(aircraft_id: int) -> str:
    if 1 <= aircraft_id <= 3:
        return f"LAH{aircraft_id}"
    if 4 <= aircraft_id <= 6:
        return f"UAV{aircraft_id - 3}"
    return f"AC{aircraft_id}"


def _extract_waypoints(data: Dict[str, Any]) -> List[Dict[str, Any]]:
    for key in ("lahWaypointList", "uavWaypointList", "waypointList"):
        lst = data.get(key)
        if isinstance(lst, list):
            return lst
    return []


def _extract_coord(item: Dict[str, Any]) -> Optional[Tuple[float, float, Optional[float]]]:
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


def _load_json_files(folder: Path, *, normalize_input_plan: bool = False) -> List[Dict[str, Any]]:
    if not folder.is_dir():
        return []
    loaded: List[Dict[str, Any]] = []
    for path in sorted(folder.glob("*.json")):
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            continue
        if isinstance(data, dict):
            if normalize_input_plan:
                normalize_input_mission_plan_float_fields(data)
            loaded.append(data)
    return loaded


def _resolve_db_root_for_flight_dir(base: Path, flight_dir: Path) -> Path:
    candidates = [
        flight_dir.parent,
        base,
        base / "SBC3",
    ]
    for candidate in candidates:
        if (candidate / "IndividualMissionPlan").is_dir():
            return candidate
    return flight_dir.parent


def load_flight_paths(base_path: str, *, project_root: Path) -> Dict[str, Any]:
    base = _resolve_path(base_path, project_root)
    flight_dir, found = _find_flight_path_dir(base)
    if not flight_dir or not flight_dir.exists():
        return {
            "ok": False,
            "error": "FlightPath directory not found",
            "basePath": str(base),
            "candidates": [str(p) for p in found],
        }

    features: List[Dict[str, Any]] = []
    flight_paths: List[Dict[str, Any]] = []
    feature_id = 1
    for fp in sorted(flight_dir.glob("*.json")):
        try:
            data = json.loads(fp.read_text(encoding="utf-8"))
        except Exception:
            continue
        if not isinstance(data, dict):
            continue

        aircraft_id = data.get("aircraftID") or data.get("AircraftID")
        path_id = data.get("pathID") or data.get("PathID") or fp.stem
        try:
            aircraft_id = int(aircraft_id)
        except Exception:
            aircraft_id = -1
        agent = _agent_label(aircraft_id) if aircraft_id > 0 else "AC?"

        coords: List[List[float]] = []
        altitudes: List[float] = []
        alts: List[Optional[float]] = []
        wp_ids: List[Optional[int]] = []
        wp_done_flags: List[bool] = []
        for wp in _extract_waypoints(data):
            if not isinstance(wp, dict):
                continue
            coord = _extract_coord(wp)
            if coord is None:
                continue
            lat, lon, alt = coord
            wp_id_raw = wp.get("waypointID") if "waypointID" in wp else wp.get("WaypointID")
            try:
                wp_id = int(wp_id_raw) if wp_id_raw is not None else None
            except Exception:
                wp_id = None
            coords.append([lon, lat])
            wp_ids.append(wp_id)
            wp_done_flags.append(_coerce_bool(wp.get("isDone") or wp.get("IsDone"), False))
            if alt is not None:
                altitudes.append(alt)
                alts.append(alt)
            else:
                alts.append(None)

        is_formation = _coerce_bool(data.get("isFormationFlight") or data.get("IsFormationFlight"), False)
        if len(coords) < 2:
            if is_formation:
                flight_paths.append(data)
            continue

        flight_paths.append(data)
        feature = {
            "id": feature_id,
            "agent": agent,
            "aircraftId": aircraft_id,
            "pathId": path_id,
            "isDone": bool(wp_done_flags and all(wp_done_flags)),
            "points": len(coords),
            "coords": coords,
            "alts": alts,
            "wpIds": wp_ids,
            "altMin": min(altitudes) if altitudes else None,
            "altMax": max(altitudes) if altitudes else None,
        }
        feature_id += 1
        features.append(feature)

    agents: Dict[str, int] = {}
    for feat in features:
        agents[feat["agent"]] = agents.get(feat["agent"], 0) + 1

    db_root = _resolve_db_root_for_flight_dir(base, flight_dir)
    input_plans = _load_json_files(db_root / "InputMissionPlan", normalize_input_plan=True)
    individual_plans = _load_json_files(db_root / "IndividualMissionPlan")
    path_mission_index = _path_input_mission_index(individual_plans)
    sweep_line_spacing_summaries = _build_sweep_line_spacing_summaries(
        flight_paths,
        path_mission_index,
    )
    sweep_line_spacing_by_input = {
        str(item["inputMissionID"]): item for item in sweep_line_spacing_summaries
    }

    return {
        "ok": True,
        "basePath": str(base),
        "flightPathDir": str(flight_dir),
        "features": features,
        "flightPaths": flight_paths,
        "inputMissionPlans": input_plans,
        "individualMissionPlans": individual_plans,
        "pathMissionIndex": path_mission_index,
        "sweepLineSpacingSummaries": sweep_line_spacing_summaries,
        "sweepLineSpacingByInputMissionID": sweep_line_spacing_by_input,
        "agents": agents,
        "count": len(features),
    }
