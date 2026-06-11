from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional


def load_json(path: str | Path) -> Dict[str, Any]:
    fp = Path(path)
    if not fp.is_file():
        raise FileNotFoundError(f"File not found: {fp}")
    with fp.open("r", encoding="utf-8") as f:
        payload = json.load(f)
    if not isinstance(payload, dict):
        raise ValueError(f"Top-level JSON must be an object: {fp}")
    return payload


def load_0201(path: str | Path) -> Dict[str, Any]:
    cmpk = load_json(path)
    missions = cmpk.get("inputMissionList")
    if not isinstance(missions, list):
        raise ValueError("0201 must include 'inputMissionList' as a list.")
    return cmpk


def load_0203(path: str | Path) -> Dict[str, Any]:
    return load_json(path)


def _extract_aircraft_id(entry: Any) -> Optional[int]:
    if entry is None:
        return None
    if isinstance(entry, dict):
        entry = entry.get("aircraftID")
    if isinstance(entry, int):
        return entry
    if isinstance(entry, str):
        token = entry.strip().upper()
        if token.startswith(("UAV", "LAH")):
            token = "".join(ch for ch in token if ch.isdigit())
        if not token:
            return None
        try:
            return int(token)
        except ValueError:
            return None
    try:
        return int(entry)
    except (TypeError, ValueError):
        return None


def extract_uav_ids(cmpk: Dict[str, Any]) -> List[int]:
    raw = cmpk.get("availableAircraftList")
    if not isinstance(raw, list):
        return []
    uavs: List[int] = []
    for item in raw:
        aid = _extract_aircraft_id(item)
        if aid is None:
            continue
        if 4 <= aid <= 6:
            uavs.append(aid)
    return sorted(set(uavs))


def centroid_from_coords(coords: List[Dict[str, Any]]) -> Optional[Dict[str, float]]:
    if not coords:
        return None
    valid = [
        c for c in coords
        if isinstance(c, dict) and "latitude" in c and "longitude" in c
    ]
    if not valid:
        return None
    lat = sum(float(c["latitude"]) for c in valid) / len(valid)
    lon = sum(float(c["longitude"]) for c in valid) / len(valid)
    return {"latitude": lat, "longitude": lon, "altitude": float(valid[0].get("altitude", 0.0))}


def extract_takeover_centroid(mrpk: Dict[str, Any]) -> Optional[Dict[str, float]]:
    take_over = mrpk.get("takeOverInfoList")
    if not isinstance(take_over, list):
        return None
    coords = []
    for item in take_over:
        if not isinstance(item, dict):
            continue
        coord = item.get("coordinate")
        if isinstance(coord, dict) and "latitude" in coord and "longitude" in coord:
            coords.append(coord)
    return centroid_from_coords(coords)
