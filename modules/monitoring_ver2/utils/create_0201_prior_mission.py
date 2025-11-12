from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from .create_0201_attack import (
    _square_area,
    build_payload,
    load_active_agency_dir,
    select_base_input_plan,
    write_payload,
)


CoordinateDict = Dict[str, float]


def _get(obj: Any, *names: str) -> Any:
    """Case-insensitive getter that works for dicts and dataclass-like objects."""

    if obj is None:
        return None
    for name in names:
        alt = name[0].upper() + name[1:] if name else name
        if isinstance(obj, dict):
            if name in obj:
                return obj[name]
            if alt in obj:
                return obj[alt]
        else:
            if hasattr(obj, name):
                return getattr(obj, name)
            if hasattr(obj, alt):
                return getattr(obj, alt)
    return None


def _coerce_int(value: Any) -> Optional[int]:
    try:
        if value is None:
            return None
        return int(value)
    except (TypeError, ValueError):
        try:
            return int(float(str(value).strip()))
        except Exception:
            return None


def _coerce_float(value: Any) -> Optional[float]:
    try:
        if value is None:
            return None
        return float(value)
    except (TypeError, ValueError):
        try:
            return float(str(value).strip())
        except Exception:
            return None


def _coord_dict(source: Any) -> Optional[CoordinateDict]:
    coord = source
    if coord is None:
        return None
    lat = _coerce_float(_get(coord, "latitude", "Latitude"))
    lon = _coerce_float(_get(coord, "longitude", "Longitude"))
    alt = _coerce_float(_get(coord, "altitude", "Altitude"))
    if lat is None or lon is None:
        return None
    if alt is None:
        alt = 0.0
    return {"latitude": lat, "longitude": lon, "altitude": alt}


def _extract_coordinate_from_entry(
    entry: Any, agency_dir: Path
) -> Tuple[Optional[CoordinateDict], str]:
    mission_type = _coerce_int(_get(entry, "missionType"))
    coord_orientation = _get(entry, "coordinateOrientation")
    target_orientation = _get(entry, "targetOrientation")

    # 1) Coordinate-oriented missions → take provided coordinate.
    if mission_type == 1:
        coord = _coord_dict(_get(coord_orientation, "coordinate") or coord_orientation)
        if coord:
            return coord, "coordinateOrientation"

    # 2) Target-tracking missions → resolve latest target coordinate.
    if mission_type == 2 and target_orientation:
        target_id = _coerce_int(_get(target_orientation, "targetID"))
        if target_id is not None:
            coord = _lookup_target_coordinate(agency_dir, target_id)
            if coord:
                return coord, "targetOrientation"

    # Fallbacks: some payloads may swap fields, so try both.
    coord = _coord_dict(_get(coord_orientation, "coordinate") or coord_orientation)
    if coord:
        return coord, "coordinateOrientation"
    if target_orientation:
        target_id = _coerce_int(_get(target_orientation, "targetID"))
        if target_id is not None:
            coord = _lookup_target_coordinate(agency_dir, target_id)
            if coord:
                return coord, "targetOrientation"

    return None, ""


def _lookup_target_coordinate(agency_dir: Path, target_id: int) -> Optional[CoordinateDict]:
    target_path = agency_dir / "DSS_Internal" / "targetInfo.json"
    if not target_path.exists():
        return None

    try:
        with target_path.open("r", encoding="utf-8") as fh:
            target_data = json.load(fh)
    except Exception:
        return None

    target_list: Dict[str, Any] = target_data.get("targetList") or {}
    for entry in target_list.values():
        candidate_id = _coerce_int(_get(entry, "targetID"))
        if candidate_id == target_id:
            return _coord_dict(entry.get("coordinate") or entry)
    return None


def _select_prior_entry(prior_info: Any) -> Any:
    candidates: List[Any] = []
    if prior_info is None:
        return None

    if isinstance(prior_info, dict):
        candidates = prior_info.get("priorMissionList") or []
    else:
        candidates = getattr(prior_info, "priorMissionList", []) or []

    if not candidates:
        return None

    # Prefer entries with coordinateOrientation to minimize lookups.
    for entry in candidates:
        if _get(entry, "coordinateOrientation"):
            return entry
    return candidates[0]


def create_plan_from_prior_mission(
    *,
    prior_info: Any,
    scenario_dir: Optional[str] = None,
    agency: Optional[str] = None,
    package_id: Optional[int] = None,
    size_km: float = 2.0,
    output_name: Optional[str] = None,
) -> Tuple[Path, Dict[str, Any]]:
    """
    Clone an existing InputMissionPlan and inject a 2x2km AOI derived from PriorMissionInfo.
    Returns the output path and metadata describing the generated file.
    """

    if prior_info is None:
        raise RuntimeError("PriorMissionInfo payload is required.")

    entry = _select_prior_entry(prior_info)
    if entry is None:
        raise RuntimeError("PriorMissionInfo does not include any mission entries.")

    agency_dir = load_active_agency_dir(scenario_dir, agency)
    coord, coord_source = _extract_coordinate_from_entry(entry, agency_dir)
    if coord is None:
        raise RuntimeError("Unable to resolve coordinate data from PriorMissionInfo.")

    base_data, base_path = select_base_input_plan(agency_dir, package_id)

    area_coords = _square_area(
        coord["latitude"],
        coord["longitude"],
        coord.get("altitude", 0.0),
        size_km,
    )

    timestamp = _coerce_int(_get(prior_info, "timestamp"))
    payload, mission_id = build_payload(base_data, area_coords, timestamp)
    meta = {
        "mission_id": mission_id,
        "base_plan": base_path.name,
        "priorMissionID": _coerce_int(_get(entry, "priorMissionID")),
        "missionType": _coerce_int(_get(entry, "missionType")),
        "coordinateSource": coord_source,
        "coordinate": coord,
        "targetID": target_id_value,
    }
    payload["_priorMissionContext"] = {
        "inputMissionID": mission_id,
        "priorMissionID": meta["priorMissionID"],
        "missionType": meta["missionType"],
        "coordinate": coord,
        "targetID": target_id_value,
    }
    output_path = write_payload(agency_dir=agency_dir, payload=payload, output_name=output_name)
    meta["output_path"] = str(output_path)
    return output_path, meta
