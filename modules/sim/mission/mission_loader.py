from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


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
    feature_id = 1
    for fp in sorted(flight_dir.glob("*.json")):
        try:
            data = json.loads(fp.read_text(encoding="utf-8"))
        except Exception:
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
        for wp in _extract_waypoints(data):
            if not isinstance(wp, dict):
                continue
            coord = _extract_coord(wp)
            if coord is None:
                continue
            lat, lon, alt = coord
            coords.append([lon, lat])
            if alt is not None:
                altitudes.append(alt)
                alts.append(alt)
            else:
                alts.append(None)

        if len(coords) < 2:
            continue

        feature = {
            "id": feature_id,
            "agent": agent,
            "aircraftId": aircraft_id,
            "pathId": path_id,
            "points": len(coords),
            "coords": coords,
            "alts": alts,
            "altMin": min(altitudes) if altitudes else None,
            "altMax": max(altitudes) if altitudes else None,
        }
        feature_id += 1
        features.append(feature)

    agents: Dict[str, int] = {}
    for feat in features:
        agents[feat["agent"]] = agents.get(feat["agent"], 0) + 1

    return {
        "ok": True,
        "basePath": str(base),
        "flightPathDir": str(flight_dir),
        "features": features,
        "agents": agents,
        "count": len(features),
    }
