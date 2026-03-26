from __future__ import annotations

import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

from ..models import DirectionDebug, SplitPiece, SplitRunResult


LINE_TYPES = {1, 4, 5, 7}
AREA_TYPES = {2, 3, 6}


def _to_float(v: Any, default: float = 0.0) -> float:
    try:
        return float(v)
    except Exception:
        return default


def _normalize_json(value: Any) -> Any:
    if value is None:
        return None
    if isinstance(value, (str, bool)):
        return value
    if isinstance(value, int):
        return int(value)
    if isinstance(value, float):
        if not math.isfinite(value):
            return None
        return float(value)
    if isinstance(value, dict):
        out: Dict[str, Any] = {}
        for k, v in value.items():
            out[str(k)] = _normalize_json(v)
        return out
    if isinstance(value, (list, tuple)):
        return [_normalize_json(v) for v in value]
    return str(value)


def _normalize_coord_list(coords: Any) -> List[Dict[str, float]]:
    out: List[Dict[str, float]] = []
    if not isinstance(coords, list):
        return out
    for c in coords:
        if not isinstance(c, dict):
            continue
        out.append(
            {
                "latitude": _to_float(c.get("latitude"), 0.0),
                "longitude": _to_float(c.get("longitude"), 0.0),
                "altitude": _to_float(c.get("altitude"), 0.0),
            }
        )
    return out


def _centroid_ll(coords: List[Dict[str, float]]) -> Optional[Dict[str, float]]:
    if not coords:
        return None
    lat = sum(float(p["latitude"]) for p in coords) / float(len(coords))
    lon = sum(float(p["longitude"]) for p in coords) / float(len(coords))
    alt = sum(float(p.get("altitude", 0.0)) for p in coords) / float(len(coords))
    return {"latitude": lat, "longitude": lon, "altitude": alt}


def _piece_geometry_type(piece: SplitPiece) -> str:
    if int(piece.mission_type) in LINE_TYPES:
        return "line"
    if int(piece.mission_type) in AREA_TYPES:
        return "area"
    return "unknown"


def _piece_struct(piece: SplitPiece) -> Dict[str, Any]:
    data = piece.data if isinstance(piece.data, dict) else {}
    mission_type = int(piece.mission_type)
    geometry_type = _piece_geometry_type(piece)

    out: Dict[str, Any] = {
        "pieceUID": f"M{int(piece.parent_order)}-P{int(piece.piece_index)}",
        "parentOrder": int(piece.parent_order),
        "pieceIndex": int(piece.piece_index),
        "missionID": _normalize_json(piece.mission_id),
        "missionType": mission_type,
        "geometryType": geometry_type,
        "assignedUAV": int(piece.assigned_uav) if (piece.assigned_uav is not None and int(piece.assigned_uav) > 0) else None,
        "splitStage": int(data.get("splitStage", 0) or 0),
        "splitCount": int(data.get("splitCount", 0) or 0),
        "bearing": {
            "moveDeg": _normalize_json(data.get("phaseMoveBearing_deg", data.get("bearing_deg"))),
            "splitDeg": _normalize_json(data.get("phaseSplitBearing_deg", data.get("splitBearing_deg"))),
            "boundaryAxisDeg": _normalize_json(data.get("boundaryAxisBearing_deg")),
            "inDeg": _normalize_json(data.get("bearingIn_deg")),
            "outDeg": _normalize_json(data.get("bearingOut_deg")),
        },
        "stats": {
            "meanAltitude": _normalize_json(data.get("meanAltitude")),
            "altitudeVariance": _normalize_json(data.get("altitudeVariance")),
        },
    }

    if geometry_type == "line":
        centerline = _normalize_coord_list(data.get("Centerline"))
        out["line"] = {
            "widthM": _to_float(data.get("width"), 0.0),
            "centerline": centerline,
            "start": centerline[0] if centerline else None,
            "end": centerline[-1] if centerline else None,
        }
    elif geometry_type == "area":
        coords = _normalize_coord_list(data.get("coordinateList"))
        raw_coords = _normalize_coord_list(data.get("rawCoordinateList"))
        out["area"] = {
            "coordinateList": coords,
            "rawCoordinateList": raw_coords,
            "centroid": _centroid_ll(coords),
            "postProcess": _normalize_json(data.get("postProcess")),
            "reviewArea": _normalize_json(data.get("reviewArea")),
        }

    # Keep full original piece payload for maximum reusability.
    out["rawData"] = _normalize_json(data)
    return out


def _direction_struct(dbg: DirectionDebug) -> Dict[str, Any]:
    return {
        "parentOrder": int(dbg.parent_order),
        "sourceAreaIndex": int(dbg.source_area_index) if dbg.source_area_index is not None else None,
        "missionID": _normalize_json(dbg.mission_id),
        "missionType": int(dbg.mission_type),
        "prevPoint": _normalize_json(dbg.prev_point),
        "centerPoint": _normalize_json(dbg.center_point),
        "nextPoint": _normalize_json(dbg.next_point),
        "lineStart": _normalize_json(dbg.line_start),
        "lineEnd": _normalize_json(dbg.line_end),
        "bearing": {
            "inDeg": _normalize_json(dbg.bearing_in_deg),
            "outDeg": _normalize_json(dbg.bearing_out_deg),
            "moveDeg": _normalize_json(dbg.bearing_move_deg),
            "splitDeg": _normalize_json(dbg.bearing_split_deg),
        },
    }


def _mission_summary(cmpk: Optional[Dict[str, Any]]) -> List[Dict[str, Any]]:
    if not isinstance(cmpk, dict):
        return []
    missions = cmpk.get("inputMissionList")
    if not isinstance(missions, list):
        return []
    out: List[Dict[str, Any]] = []
    for i, m in enumerate(missions, start=1):
        if not isinstance(m, dict):
            continue
        detail = m.get("missionDetail") if isinstance(m.get("missionDetail"), dict) else {}
        lines = detail.get("lineList") if isinstance(detail.get("lineList"), list) else []
        areas = detail.get("areaList") if isinstance(detail.get("areaList"), list) else []
        out.append(
            {
                "order": i,
                "inputMissionID": _normalize_json(m.get("inputMissionID")),
                "inputMissionType": int(m.get("inputMissionType", 0) or 0),
                "lineCount": len(lines),
                "areaCount": len(areas),
            }
        )
    return out


def _expected_path_struct(rows: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    for row in rows:
        if not isinstance(row, dict):
            continue
        item = dict(row)
        item["coordinateList"] = _normalize_coord_list(row.get("coordinateList"))
        point_list = row.get("pointList")
        if isinstance(point_list, list):
            plist: List[Dict[str, Any]] = []
            for p in point_list:
                if not isinstance(p, dict):
                    continue
                plist.append(
                    {
                        "name": str(p.get("name", "")),
                        "role": str(p.get("role", "")),
                        "coordinate": _normalize_json(p.get("coordinate")),
                    }
                )
            item["pointList"] = plist
        out.append(_normalize_json(item))
    return out


def build_internal_icd_payload(
    split_result: SplitRunResult,
    cmpk: Optional[Dict[str, Any]] = None,
    mrpk: Optional[Dict[str, Any]] = None,
    cmpk_path: Optional[str] = None,
    mrpk_path: Optional[str] = None,
) -> Dict[str, Any]:
    pieces = sorted(split_result.pieces, key=lambda p: (p.parent_order, p.piece_index))
    directions = sorted(split_result.directions, key=lambda d: d.parent_order)
    expected_paths = list(split_result.expected_paths) if isinstance(split_result.expected_paths, list) else []

    piece_rows = [_piece_struct(p) for p in pieces]
    line_piece_count = sum(1 for p in pieces if int(p.mission_type) in LINE_TYPES)
    area_piece_count = sum(1 for p in pieces if int(p.mission_type) in AREA_TYPES)
    schedule_result = split_result.schedule_result if isinstance(split_result.schedule_result, dict) else {}
    schedule_timelines = schedule_result.get("timelines", []) if isinstance(schedule_result, dict) else []
    if not isinstance(schedule_timelines, list):
        schedule_timelines = []

    payload: Dict[str, Any] = {
        "schema": {
            "name": "AMTrainer_Assignment_InternalICD",
            "version": "1.0.0",
        },
        "generatedAtUTC": datetime.now(timezone.utc).isoformat(),
        "source": {
            "cmpkPath": str(cmpk_path) if cmpk_path else None,
            "mrpkPath": str(mrpk_path) if mrpk_path else None,
        },
        "uav": {
            "count": int(split_result.uav_count),
            "ids": [int(x) for x in split_result.uav_ids],
        },
        "summary": {
            "missionCount": len(_mission_summary(cmpk)),
            "pieceCount": len(piece_rows),
            "linePieceCount": int(line_piece_count),
            "areaPieceCount": int(area_piece_count),
            "directionCount": len(directions),
            "expectedPathCount": len(expected_paths),
            "scheduleTimelineCount": len(schedule_timelines),
        },
        "inputMissionSummary": _mission_summary(cmpk),
        "directionDebugList": [_direction_struct(d) for d in directions],
        "splitPieceList": piece_rows,
        "expectedPathList": _expected_path_struct(expected_paths),
        "scheduleResult": _normalize_json(schedule_result),
        # Keep original payloads for full traceability.
        "sourcePayload": {
            "cmpk": _normalize_json(cmpk),
            "mrpk": _normalize_json(mrpk),
        },
    }
    return payload


def save_internal_icd_payload(payload: Dict[str, Any], out_path: str | Path) -> Path:
    path = Path(out_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    return path
