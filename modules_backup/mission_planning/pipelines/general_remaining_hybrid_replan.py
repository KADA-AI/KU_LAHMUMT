from __future__ import annotations

import copy
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

from shapely.geometry import MultiPolygon, Polygon
from shapely.ops import unary_union

from modules.common import agent_status_snapshot
from modules.mission_planning.pipelines.next_collab_path_builder import (
    build_flight_path_from_planned_row,
    build_mission_info_from_planned_row,
)
from modules.mission_planning.planners.next_collab_division._geo_utils import (
    coords_to_xy,
    meters_to_coord,
)
from modules.mission_planning.runtime.next_collab_division_runner import (
    run_next_collab_division_plan,
)
from modules.mission_planning.runtime.next_collab_line_runner import (
    run_next_collab_line_plan,
)


@dataclass
class RemainingHybridResult:
    applied: bool
    mode: str | None = None
    input_mission_id: int | None = None
    aircraft_ids: List[int] | None = None
    planner_workflow: str | None = None
    reason: str | None = None


@dataclass
class _AgentEntry:
    aircraft_id: int
    coordinate: Dict[str, Any]
    heading_deg: float | None


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


def _normalize_coordinate(payload: object | None) -> Dict[str, float] | None:
    if not isinstance(payload, dict):
        return None
    lat = _to_float(payload.get("latitude"))
    lon = _to_float(payload.get("longitude"))
    if lat is None or lon is None:
        return None
    out: Dict[str, float] = {
        "latitude": float(lat),
        "longitude": float(lon),
    }
    alt = _to_float(payload.get("altitude"))
    if alt is not None:
        out["altitude"] = float(alt)
    return out


def _normalize_coord_list(payload: object | None) -> List[Dict[str, float]]:
    coords = payload if isinstance(payload, list) else []
    out: List[Dict[str, float]] = []
    for item in coords:
        coord = _normalize_coordinate(item)
        if coord is not None:
            out.append(coord)
    return out


def _load_json(path: Path) -> Dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    return payload if isinstance(payload, dict) else {}


def _has_nonempty_geometry(detail: Dict[str, Any]) -> bool:
    line_list = detail.get("lineList") if isinstance(detail.get("lineList"), list) else []
    if line_list:
        return True
    area_list = detail.get("areaList") if isinstance(detail.get("areaList"), list) else []
    if area_list:
        return True
    coord_list = detail.get("coordinateList") if isinstance(detail.get("coordinateList"), list) else []
    return len(coord_list) >= 2


def _is_line_input_mission(mission: Dict[str, Any]) -> bool:
    try:
        mission_type = int(mission.get("inputMissionType"))
    except Exception:
        mission_type = None
    if mission_type in (1, 4, 5, 7):
        return True
    if mission_type in (2, 3, 6):
        return False
    detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
    return bool(detail.get("lineList"))


def _mission_input_id(mission: Dict[str, Any]) -> int | None:
    related = mission.get("relatedMission") if isinstance(mission.get("relatedMission"), dict) else {}
    return _to_int(related.get("inputMissionID"))


def _current_remaining_input_mission(cmpk_payload: Dict[str, Any]) -> Dict[str, Any] | None:
    missions = cmpk_payload.get("inputMissionList") if isinstance(cmpk_payload.get("inputMissionList"), list) else []
    for mission in missions:
        if not isinstance(mission, dict):
            continue
        if bool(mission.get("isDone")):
            continue
        detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
        if not _has_nonempty_geometry(detail):
            continue
        return mission
    return None


def _representative_altitude(detail: Dict[str, Any]) -> float:
    for key in ("lineList", "areaList"):
        rows = detail.get(key) if isinstance(detail.get(key), list) else []
        for row in rows:
            if not isinstance(row, dict):
                continue
            for coord in row.get("coordinateList") or []:
                alt = _to_float(coord.get("altitude")) if isinstance(coord, dict) else None
                if alt is not None:
                    return float(alt)
    for coord in detail.get("coordinateList") or []:
        alt = _to_float(coord.get("altitude")) if isinstance(coord, dict) else None
        if alt is not None:
            return float(alt)
    return 0.0


def _build_single_area_polygon(detail: Dict[str, Any]) -> List[Dict[str, float]]:
    area_blocks = [
        block
        for block in (detail.get("areaList") or [])
        if isinstance(block, dict) and not bool(block.get("isHole"))
    ]
    if len(area_blocks) == 1:
        coords = _normalize_coord_list(area_blocks[0].get("coordinateList"))
        if len(coords) >= 3:
            return coords

    polygons_xy: List[Polygon] = []
    for block in area_blocks:
        coords = _normalize_coord_list(block.get("coordinateList"))
        points_xy = coords_to_xy(coords)
        if len(points_xy) < 3:
            continue
        try:
            poly = Polygon(points_xy)
        except Exception:
            continue
        if not poly.is_valid:
            try:
                poly = poly.buffer(0)
            except Exception:
                continue
        if poly.is_empty:
            continue
        if isinstance(poly, MultiPolygon):
            try:
                poly = max(
                    (geom for geom in poly.geoms if isinstance(geom, Polygon) and not geom.is_empty),
                    key=lambda geom: float(geom.area or 0.0),
                )
            except Exception:
                continue
        if isinstance(poly, Polygon) and not poly.is_empty:
            polygons_xy.append(poly)

    if polygons_xy:
        try:
            merged = unary_union(polygons_xy)
        except Exception:
            merged = polygons_xy[0]
        if isinstance(merged, MultiPolygon):
            try:
                merged = merged.convex_hull
            except Exception:
                merged = max(
                    (geom for geom in merged.geoms if isinstance(geom, Polygon) and not geom.is_empty),
                    key=lambda geom: float(geom.area or 0.0),
                )
        if isinstance(merged, Polygon) and not merged.is_empty:
            exterior = [(float(x), float(y)) for (x, y) in list(merged.exterior.coords)[:-1]]
            if len(exterior) >= 3:
                tolerance = 0.0
                if len(exterior) > 120:
                    min_x = min(point[0] for point in exterior)
                    max_x = max(point[0] for point in exterior)
                    min_y = min(point[1] for point in exterior)
                    max_y = max(point[1] for point in exterior)
                    tolerance = max(max(max_x - min_x, max_y - min_y) * 0.01, 10.0)
                    tolerance = min(tolerance, 120.0)
                if tolerance > 0.0:
                    try:
                        simplified = merged.simplify(tolerance, preserve_topology=True)
                        if isinstance(simplified, Polygon) and not simplified.is_empty:
                            exterior = [
                                (float(x), float(y))
                                for (x, y) in list(simplified.exterior.coords)[:-1]
                            ]
                    except Exception:
                        pass
                if len(exterior) >= 3:
                    alt = _representative_altitude(detail)
                    return [meters_to_coord(x, y, alt_m=alt) for (x, y) in exterior]

    coord_list = _normalize_coord_list(detail.get("coordinateList"))
    if len(coord_list) >= 3:
        return coord_list
    return []


def _summarize_agent_entries() -> Dict[int, _AgentEntry]:
    snapshot_payload = agent_status_snapshot.load_agent_status_snapshot() or {}
    states = snapshot_payload.get("agent_states") or snapshot_payload.get("raw", {}).get("agentStateList") or []
    out: Dict[int, _AgentEntry] = {}
    for entry in states:
        if not isinstance(entry, dict):
            continue
        if not bool(entry.get("isUnmanned")):
            continue
        aircraft_id = _to_int(entry.get("aircraftID"))
        if aircraft_id is None or aircraft_id <= 0:
            continue
        coord = entry.get("coordinate") or {}
        if not coord:
            unmanned_info = entry.get("unmannedInfo") or {}
            coord = unmanned_info.get("coordinate") or {}
        normalized_coord = _normalize_coordinate(coord)
        if normalized_coord is None:
            continue
        heading = None
        velocity = entry.get("velocity") if isinstance(entry.get("velocity"), dict) else {}
        if velocity:
            heading = _to_float(velocity.get("heading"))
        if heading is None:
            heading = _to_float(entry.get("heading"))
        out[int(aircraft_id)] = _AgentEntry(
            aircraft_id=int(aircraft_id),
            coordinate=normalized_coord,
            heading_deg=heading,
        )
    return out


def _current_template_records(
    missions: List[Dict[str, Any]],
    flight_plans_0303: List[Dict[str, Any]],
    *,
    current_input_id: int,
) -> Dict[int, Dict[str, Any]] | None:
    fp_by_path = {
        int(path_id): fp
        for fp in flight_plans_0303
        if isinstance(fp, dict) and (_to_int(fp.get("pathID")) or 0) > 0
        for path_id in [_to_int(fp.get("pathID"))]
    }
    rows_by_aircraft: Dict[int, List[Dict[str, Any]]] = {}
    for index, mission in enumerate(missions):
        if not isinstance(mission, dict):
            continue
        aircraft_id = _to_int(mission.get("aircraftID")) or 0
        if aircraft_id not in (4, 5, 6):
            continue
        if _mission_input_id(mission) != int(current_input_id):
            continue
        path_id = _to_int(mission.get("pathID")) or 0
        rows_by_aircraft.setdefault(int(aircraft_id), []).append(
            {
                "index": int(index),
                "mission": mission,
                "flight_path": copy.deepcopy(fp_by_path.get(int(path_id)) or {}),
            }
        )
    if not rows_by_aircraft:
        return None
    if any(len(items) != 1 for items in rows_by_aircraft.values()):
        return None
    return {int(aid): items[0] for aid, items in rows_by_aircraft.items()}


def _apply_runtime_meta(path_row: Dict[str, Any], mission_entry: Dict[str, Any]) -> None:
    bearing_deg = _to_float(path_row.get("bearingDeg"))
    if bearing_deg is not None:
        mission_entry["bearing_deg"] = float(bearing_deg)


def _replace_current_missions(
    missions: List[Dict[str, Any]],
    *,
    replacements_by_aircraft: Dict[int, Dict[str, Any]],
) -> None:
    for aircraft_id, replacement in replacements_by_aircraft.items():
        index = _to_int(replacement.get("_replaceIndex"))
        if index is None or not (0 <= int(index) < len(missions)):
            continue
        replacement_copy = copy.deepcopy(replacement)
        replacement_copy.pop("_replaceIndex", None)
        missions[int(index)] = replacement_copy


def _replace_current_flight_paths(
    flight_plans_0303: List[Dict[str, Any]],
    *,
    replacements_by_path: Dict[int, Dict[str, Any]],
) -> None:
    for idx, flight_path in enumerate(flight_plans_0303):
        if not isinstance(flight_path, dict):
            continue
        path_id = _to_int(flight_path.get("pathID")) or 0
        if path_id <= 0:
            continue
        replacement = replacements_by_path.get(int(path_id))
        if replacement is None:
            continue
        flight_plans_0303[idx] = copy.deepcopy(replacement)


def apply_remaining_hybrid_replan(
    *,
    cmpk_source_path: Path,
    replan_detail: Dict[str, Any] | None,
    missions: List[Dict[str, Any]],
    flight_plans_0303: List[Dict[str, Any]],
    timestamp_ms: int,
    log: Callable[[str], None] | None = None,
) -> RemainingHybridResult:
    detail = replan_detail if isinstance(replan_detail, dict) else {}
    trigger = str(detail.get("trigger") or "").strip()
    if trigger not in {"0401", "0802"}:
        return RemainingHybridResult(applied=False, reason=f"trigger {trigger or '-'} not eligible")

    cmpk_payload = _load_json(Path(cmpk_source_path))
    current_input_mission = _current_remaining_input_mission(cmpk_payload)
    if not isinstance(current_input_mission, dict):
        return RemainingHybridResult(applied=False, reason="current remaining input mission unavailable")

    current_input_id = _to_int(current_input_mission.get("inputMissionID"))
    if current_input_id is None or current_input_id <= 0:
        return RemainingHybridResult(applied=False, reason="current inputMissionID unavailable")

    template_records = _current_template_records(
        missions,
        flight_plans_0303,
        current_input_id=int(current_input_id),
    )
    if not template_records:
        return RemainingHybridResult(
            applied=False,
            input_mission_id=int(current_input_id),
            reason="current template missions unavailable or ambiguous",
        )

    target_aircraft_ids = sorted(int(aid) for aid in template_records.keys())
    agent_entries = _summarize_agent_entries()
    aircraft_entries: List[Dict[str, Any]] = []
    for aircraft_id in target_aircraft_ids:
        agent_entry = agent_entries.get(int(aircraft_id))
        if agent_entry is None:
            return RemainingHybridResult(
                applied=False,
                input_mission_id=int(current_input_id),
                aircraft_ids=target_aircraft_ids,
                reason=f"agent status missing for aircraft {aircraft_id}",
            )
        row: Dict[str, Any] = {
            "aircraftID": int(aircraft_id),
            "coordinate": dict(agent_entry.coordinate),
        }
        if agent_entry.heading_deg is not None:
            row["headingDeg"] = float(agent_entry.heading_deg)
        aircraft_entries.append(row)

    mission_detail = current_input_mission.get("missionDetail") if isinstance(current_input_mission.get("missionDetail"), dict) else {}
    turn_radius_scale = _to_float(detail.get("turnRadiusScale"))
    if turn_radius_scale is None or turn_radius_scale <= 0.0:
        turn_radius_scale = 1.4

    replacements_by_aircraft: Dict[int, Dict[str, Any]] = {}
    replacement_fp_by_path: Dict[int, Dict[str, Any]] = {}

    if _is_line_input_mission(current_input_mission):
        planner_result = run_next_collab_line_plan(
            target_mission=current_input_mission,
            aircraft_entries=aircraft_entries,
            turn_radius_scale=float(turn_radius_scale),
            log=log,
        )
        expected_rows = [
            dict(row)
            for row in planner_result.expected_paths
            if isinstance(row, dict) and (_to_int(row.get("aircraftID")) or 0) in target_aircraft_ids
        ]
        rows_by_aircraft: Dict[int, List[Dict[str, Any]]] = {}
        for row in expected_rows:
            aircraft_id = _to_int(row.get("aircraftID")) or 0
            rows_by_aircraft.setdefault(int(aircraft_id), []).append(row)
        if sorted(rows_by_aircraft.keys()) != target_aircraft_ids or any(len(rows) != 1 for rows in rows_by_aircraft.values()):
            return RemainingHybridResult(
                applied=False,
                mode="line",
                input_mission_id=int(current_input_id),
                aircraft_ids=target_aircraft_ids,
                planner_workflow=str(planner_result.workflow),
                reason="planned line rows do not match current aircraft templates",
            )
        for aircraft_id in target_aircraft_ids:
            template_record = template_records[int(aircraft_id)]
            template_mission = copy.deepcopy(template_record["mission"])
            template_path = copy.deepcopy(template_record["flight_path"])
            path_row = rows_by_aircraft[int(aircraft_id)][0]
            mission_info = build_mission_info_from_planned_row(
                path_row,
                template_info=copy.deepcopy(template_mission.get("individualMissionInfo") or {}),
                fallback_polygon_coords=[],
            )
            template_mission["individualMissionInfo"] = mission_info
            template_mission["isDone"] = False
            _apply_runtime_meta(path_row, template_mission)
            path_id = _to_int(template_mission.get("pathID")) or 0
            individual_mission_id = _to_int(template_mission.get("individualMissionID")) or 0
            if path_id <= 0 or individual_mission_id <= 0:
                return RemainingHybridResult(
                    applied=False,
                    mode="line",
                    input_mission_id=int(current_input_id),
                    aircraft_ids=target_aircraft_ids,
                    reason=f"template IDs unavailable for aircraft {aircraft_id}",
                )
            replacements_by_aircraft[int(aircraft_id)] = {
                "_replaceIndex": int(template_record["index"]),
                **template_mission,
            }
            replacement_fp_by_path[int(path_id)] = build_flight_path_from_planned_row(
                path_row,
                template_path=template_path,
                mission_info=mission_info,
                individual_mission_id=int(individual_mission_id),
                path_id=int(path_id),
                aircraft_id=int(aircraft_id),
                entry_coord=copy.deepcopy(aircraft_entries[target_aircraft_ids.index(int(aircraft_id))]["coordinate"]),
                timestamp_ms=int(timestamp_ms),
                source="MMR",
            )
        _replace_current_missions(missions, replacements_by_aircraft=replacements_by_aircraft)
        _replace_current_flight_paths(flight_plans_0303, replacements_by_path=replacement_fp_by_path)
        return RemainingHybridResult(
            applied=True,
            mode="line",
            input_mission_id=int(current_input_id),
            aircraft_ids=target_aircraft_ids,
            planner_workflow=str(planner_result.workflow),
            reason="applied",
        )

    mission_polygon = _build_single_area_polygon(mission_detail)
    if len(mission_polygon) < 3:
        return RemainingHybridResult(
            applied=False,
            mode="area",
            input_mission_id=int(current_input_id),
            aircraft_ids=target_aircraft_ids,
            reason="merged area polygon unavailable",
        )

    planner_result = run_next_collab_division_plan(
        mission_polygon=mission_polygon,
        aircraft_entries=aircraft_entries,
        turn_radius_scale=float(turn_radius_scale),
        log=log,
    )
    expected_rows = [
        dict(row)
        for row in planner_result.expected_paths
        if isinstance(row, dict) and (_to_int(row.get("aircraftID")) or 0) in target_aircraft_ids
    ]
    rows_by_aircraft: Dict[int, List[Dict[str, Any]]] = {}
    for row in expected_rows:
        aircraft_id = _to_int(row.get("aircraftID")) or 0
        rows_by_aircraft.setdefault(int(aircraft_id), []).append(row)
    if sorted(rows_by_aircraft.keys()) != target_aircraft_ids or any(len(rows) != 1 for rows in rows_by_aircraft.values()):
        return RemainingHybridResult(
            applied=False,
            mode="area",
            input_mission_id=int(current_input_id),
            aircraft_ids=target_aircraft_ids,
            planner_workflow=str(planner_result.workflow),
            reason="planned area rows do not match current aircraft templates",
        )

    piece_polygon_map: Dict[Tuple[int, int], List[Dict[str, Any]]] = {}
    split_result = getattr(planner_result, "split_result", None)
    for piece in getattr(split_result, "pieces", []) or []:
        aircraft_id = _to_int(getattr(piece, "assigned_uav", None)) or 0
        piece_index = _to_int(getattr(piece, "piece_index", None)) or 0
        data = getattr(piece, "data", {}) if getattr(piece, "data", None) is not None else {}
        coords = _normalize_coord_list((data or {}).get("coordinateList") if isinstance(data, dict) else None)
        if aircraft_id > 0 and piece_index > 0 and coords:
            piece_polygon_map[(int(aircraft_id), int(piece_index))] = coords

    for aircraft_id in target_aircraft_ids:
        template_record = template_records[int(aircraft_id)]
        template_mission = copy.deepcopy(template_record["mission"])
        template_path = copy.deepcopy(template_record["flight_path"])
        path_row = rows_by_aircraft[int(aircraft_id)][0]
        piece_index = _to_int(path_row.get("pieceIndex")) or 0
        mission_info = build_mission_info_from_planned_row(
            path_row,
            template_info=copy.deepcopy(template_mission.get("individualMissionInfo") or {}),
            fallback_polygon_coords=piece_polygon_map.get((int(aircraft_id), int(piece_index))) or mission_polygon,
        )
        template_mission["individualMissionInfo"] = mission_info
        template_mission["isDone"] = False
        _apply_runtime_meta(path_row, template_mission)
        path_id = _to_int(template_mission.get("pathID")) or 0
        individual_mission_id = _to_int(template_mission.get("individualMissionID")) or 0
        if path_id <= 0 or individual_mission_id <= 0:
            return RemainingHybridResult(
                applied=False,
                mode="area",
                input_mission_id=int(current_input_id),
                aircraft_ids=target_aircraft_ids,
                reason=f"template IDs unavailable for aircraft {aircraft_id}",
            )
        replacements_by_aircraft[int(aircraft_id)] = {
            "_replaceIndex": int(template_record["index"]),
            **template_mission,
        }
        replacement_fp_by_path[int(path_id)] = build_flight_path_from_planned_row(
            path_row,
            template_path=template_path,
            mission_info=mission_info,
            individual_mission_id=int(individual_mission_id),
            path_id=int(path_id),
            aircraft_id=int(aircraft_id),
            entry_coord=copy.deepcopy(aircraft_entries[target_aircraft_ids.index(int(aircraft_id))]["coordinate"]),
            timestamp_ms=int(timestamp_ms),
            source="MMR",
        )

    _replace_current_missions(missions, replacements_by_aircraft=replacements_by_aircraft)
    _replace_current_flight_paths(flight_plans_0303, replacements_by_path=replacement_fp_by_path)
    return RemainingHybridResult(
        applied=True,
        mode="area",
        input_mission_id=int(current_input_id),
        aircraft_ids=target_aircraft_ids,
        planner_workflow=str(planner_result.workflow),
        reason="applied",
    )
