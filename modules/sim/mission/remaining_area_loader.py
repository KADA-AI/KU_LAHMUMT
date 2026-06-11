from __future__ import annotations

import hashlib
import json
import time
from pathlib import Path
from typing import Any

from modules.common import db_paths, mission_area_replan_store


SNAPSHOT_PREFIX = "mission_area_snapshot_"


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
        parsed = float(value)
    except Exception:
        return None
    if parsed != parsed:
        return None
    return parsed


def _pick(mapping: Any, *keys: str) -> Any:
    if not isinstance(mapping, dict):
        return None
    for key in keys:
        if key in mapping:
            return mapping.get(key)
    lowered = {str(key).lower(): value for key, value in mapping.items()}
    for key in keys:
        if str(key).lower() in lowered:
            return lowered[str(key).lower()]
    return None


def _agent_label(aircraft_id: int | None) -> str | None:
    if aircraft_id is None:
        return None
    if 1 <= int(aircraft_id) <= 3:
        return f"LAH{int(aircraft_id)}"
    if 4 <= int(aircraft_id) <= 6:
        return f"UAV{int(aircraft_id) - 3}"
    return f"AC{int(aircraft_id)}"


def _snapshot_dir() -> Path:
    return db_paths.get_db_subpath("DSS_Internal", "mission_area_replan")


def _snapshot_plan_id(path: Path) -> int | None:
    try:
        return _to_int(str(path.stem).rsplit("_", 1)[-1])
    except Exception:
        return None


def _latest_snapshot_path() -> Path | None:
    try:
        candidates = [
            path
            for path in _snapshot_dir().glob(f"{SNAPSHOT_PREFIX}*.json")
            if _snapshot_plan_id(path) is not None
        ]
    except Exception:
        candidates = []
    if not candidates:
        return None
    return max(
        candidates,
        key=lambda path: (
            path.stat().st_mtime if path.exists() else 0.0,
            path.name,
        ),
    )


def _load_snapshot(mission_plan_id: int | None) -> tuple[dict[str, Any] | None, Path | None, float | None]:
    path: Path | None = None
    if mission_plan_id is None:
        path = _latest_snapshot_path()
        mission_plan_id = _snapshot_plan_id(path) if path is not None else None
    else:
        path = _snapshot_dir() / f"{SNAPSHOT_PREFIX}{int(mission_plan_id)}.json"
    if mission_plan_id is None or path is None or not path.exists():
        return None, path, None
    data = mission_area_replan_store.load_snapshot(int(mission_plan_id))
    try:
        mtime = float(path.stat().st_mtime)
    except Exception:
        mtime = None
    return data if isinstance(data, dict) else None, path, mtime


def _coordinate_ring(coords: Any) -> list[list[float]]:
    if not isinstance(coords, list):
        return []
    ring: list[list[float]] = []
    for coord in coords:
        if not isinstance(coord, dict):
            continue
        lat = _to_float(_pick(coord, "latitude", "Latitude", "lat"))
        lon = _to_float(_pick(coord, "longitude", "Longitude", "lon", "lng"))
        if lat is None or lon is None:
            continue
        if not (-90.0 <= float(lat) <= 90.0 and -180.0 <= float(lon) <= 180.0):
            continue
        ring.append([float(lon), float(lat)])
    if len(ring) < 3:
        return []
    first = ring[0]
    last = ring[-1]
    if first[0] != last[0] or first[1] != last[1]:
        ring.append([first[0], first[1]])
    return ring if len(ring) >= 4 else []


def _area_polygons(remaining_detail: Any) -> list[list[list[list[float]]]]:
    if not isinstance(remaining_detail, dict):
        return []

    polygons: list[list[list[list[float]]]] = []
    area_list = _pick(remaining_detail, "areaList", "AreaList")
    if isinstance(area_list, list):
        outer_rings: list[list[list[float]]] = []
        hole_rings: list[list[list[float]]] = []
        for area in area_list:
            if not isinstance(area, dict):
                continue
            ring = _coordinate_ring(_pick(area, "coordinateList", "CoordinateList"))
            if not ring:
                continue
            if bool(_pick(area, "isHole", "IsHole")):
                hole_rings.append(ring)
            else:
                outer_rings.append(ring)
        for outer in outer_rings:
            polygons.append([outer, *hole_rings])

    segment_list = _pick(remaining_detail, "areaSegmentList", "AreaSegmentList")
    if isinstance(segment_list, list):
        for segment in segment_list:
            if not isinstance(segment, dict):
                continue
            ring = _coordinate_ring(_pick(segment, "coordinateList", "CoordinateList"))
            if ring:
                polygons.append([ring])

    if not polygons:
        ring = _coordinate_ring(_pick(remaining_detail, "coordinateList", "CoordinateList"))
        if ring:
            polygons.append([ring])
    return polygons


def _ring_area_degrees(ring: list[list[float]]) -> float:
    if not isinstance(ring, list) or len(ring) < 4:
        return 0.0
    area = 0.0
    for idx in range(1, len(ring)):
        x1, y1 = ring[idx - 1]
        x2, y2 = ring[idx]
        area += (float(x1) * float(y2)) - (float(x2) * float(y1))
    return abs(float(area)) * 0.5


def _polygon_area_degrees(polygon: list[list[list[float]]]) -> float:
    if not polygon:
        return 0.0
    outer_area = _ring_area_degrees(polygon[0])
    hole_area = sum(_ring_area_degrees(ring) for ring in polygon[1:])
    return max(0.0, float(outer_area) - float(hole_area))


def _polygons_area_degrees(polygons: list[list[list[list[float]]]]) -> float:
    return sum(_polygon_area_degrees(polygon) for polygon in polygons or [])


def _feature_properties(
    *,
    snapshot_plan_id: int | None,
    mission: dict[str, Any],
    owner: dict[str, Any] | None,
    geometry_source: str,
    area_index: int,
) -> dict[str, Any]:
    row = owner if isinstance(owner, dict) else mission
    aircraft_id = _to_int(_pick(row, "aircraftID", "aircraftId", "aircraft_id"))
    agent = _agent_label(aircraft_id)
    input_id = _to_int(_pick(row, "inputMissionID", "inputMissionId", "input_id"))
    individual_id = _to_int(_pick(row, "individualMissionID", "individualMissionId", "missionID"))
    remaining_area_m2 = _to_float(_pick(row, "remainingAreaM2", "remaining_area_m2"))
    if remaining_area_m2 is None:
        remaining_area_m2 = _to_float(_pick(mission, "remainingAreaM2", "remaining_area_m2"))
    progress_points = _to_int(_pick(mission, "sweepProgressPoints", "sweep_progress_points"))
    progress_total = _to_int(_pick(mission, "sweepPointCount", "sweep_point_count"))
    boundary_index = _to_int(_pick(mission, "mappedBoundaryLineIndex", "mapped_boundary_line_index"))
    return {
        "missionPlanID": _to_int(_pick(mission, "missionPlanID")) or snapshot_plan_id,
        "inputMissionID": input_id,
        "individualMissionID": individual_id,
        "aircraftID": aircraft_id,
        "agent": agent,
        "geometrySource": geometry_source,
        "areaIndex": int(area_index),
        "remainingAreaM2": remaining_area_m2,
        "coveragePercent": _to_int(_pick(mission, "coveragePercent", "coverage_percent")),
        "isDone": 1 if bool(_pick(row, "isDone", "done")) or bool(_pick(mission, "isDone", "done")) else 0,
        "progressSource": _pick(mission, "progressSource", "areaProgressSource"),
        "sweepProgressPoints": progress_points,
        "sweepPointCount": progress_total,
        "mappedBoundaryLineIndex": boundary_index,
    }


def _features_from_snapshot(snapshot: dict[str, Any]) -> list[dict[str, Any]]:
    snapshot_plan_id = _to_int(_pick(snapshot, "missionPlanID", "missionPlanId"))
    features: list[dict[str, Any]] = []
    feature_id = 1
    for mission in _pick(snapshot, "missions", "missionList") or []:
        if not isinstance(mission, dict):
            continue
        if str(_pick(mission, "missionType", "type") or "").strip().lower() != "area":
            continue

        mission_polygons = _area_polygons(_pick(mission, "remainingDetail", "remaining_detail"))
        ownership = _pick(mission, "areaOwnershipDetails", "areaOwnershipDetailList")
        owner_polygons: list[tuple[dict[str, Any], list[list[list[float]]]]] = []
        if isinstance(ownership, list):
            for owner in ownership:
                if not isinstance(owner, dict):
                    continue
                detail = _pick(owner, "remainingDetail", "remaining_detail")
                for polygon in _area_polygons(detail):
                    owner_polygons.append((owner, polygon))

        mission_area = _polygons_area_degrees(mission_polygons)
        owner_area = _polygons_area_degrees([polygon for _owner, polygon in owner_polygons])
        use_owner_projection = bool(
            owner_polygons
            and not mission_polygons
            and owner_area > 0.0
            and mission_area <= 0.0
        )
        if use_owner_projection:
            for area_index, (owner, polygon) in enumerate(owner_polygons, start=1):
                features.append(
                    {
                        "type": "Feature",
                        "id": feature_id,
                        "geometry": {"type": "Polygon", "coordinates": polygon},
                        "properties": _feature_properties(
                            snapshot_plan_id=snapshot_plan_id,
                            mission=mission,
                            owner=owner,
                            geometry_source="areaOwnershipProjection",
                            area_index=area_index,
                        ),
                    }
                )
                feature_id += 1
            continue

        for area_index, polygon in enumerate(mission_polygons, start=1):
            features.append(
                {
                    "type": "Feature",
                    "id": feature_id,
                    "geometry": {"type": "Polygon", "coordinates": polygon},
                    "properties": _feature_properties(
                        snapshot_plan_id=snapshot_plan_id,
                        mission=mission,
                        owner=None,
                        geometry_source="remainingDetail",
                        area_index=area_index,
                    ),
                }
            )
            feature_id += 1
    return features


def _features_revision(features: list[dict[str, Any]]) -> str:
    try:
        serialized = json.dumps(
            features,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )
    except Exception:
        serialized = str(len(features))
    return hashlib.sha1(serialized.encode("utf-8", errors="ignore")).hexdigest()


def build_remaining_area_snapshot(mission_plan_id: int | None = None) -> dict[str, Any]:
    snapshot, path, mtime = _load_snapshot(mission_plan_id)
    if not isinstance(snapshot, dict):
        return {
            "ok": True,
            "available": False,
            "missionPlanID": mission_plan_id,
            "snapshotPath": str(path) if path is not None else None,
            "timestamp": int(time.time() * 1000),
            "featureCollection": {"type": "FeatureCollection", "features": []},
            "features": [],
            "count": 0,
        }
    snapshot_plan_id = _to_int(_pick(snapshot, "missionPlanID", "missionPlanId"))
    features = _features_from_snapshot(snapshot)
    data_revision = _features_revision(features)
    return {
        "ok": True,
        "available": True,
        "missionPlanID": snapshot_plan_id,
        "snapshotPath": str(path) if path is not None else None,
        "snapshotMtimeMs": int(float(mtime) * 1000.0) if mtime is not None else None,
        "dataRevision": data_revision,
        "timestamp": _to_int(_pick(snapshot, "timestamp")) or int(time.time() * 1000),
        "featureCollection": {
            "type": "FeatureCollection",
            "features": features,
        },
        "features": features,
        "count": len(features),
    }
