from __future__ import annotations

import importlib.util
import json
import os
import sys
import tempfile
from copy import deepcopy
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def configure_import_paths(project_root: Path = PROJECT_ROOT) -> None:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    desired = (
        project_root,
        project_root / "modules",
        project_root / "modules" / "mission_planning",
        project_root / "modules" / "mission_planning" / "MissionPlanner",
    )
    for path in reversed(desired):
        path_str = str(path)
        if not path.exists():
            continue
        while path_str in sys.path:
            sys.path.remove(path_str)
        sys.path.insert(0, path_str)


def fail(message: str) -> None:
    raise AssertionError(message)


def expect_true(label: str, condition: bool) -> None:
    if not condition:
        fail(f"{label} expected true")


def expect_equal(label: str, actual: object, expected: object) -> None:
    if actual != expected:
        fail(f"{label} changed: expected {expected!r}, got {actual!r}")


def load_current_db_root() -> Path | None:
    path = PROJECT_ROOT / "settings" / "current_scenario.json"
    if not path.exists():
        return None
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None
    db_root = payload.get("db_root")
    if not db_root:
        return None
    root = Path(str(db_root))
    return root if root.exists() else None


def latest_plan_id(db_root: Path) -> int | None:
    plan_dir = db_root / "MissionPlan"
    if not plan_dir.exists():
        return None
    ids: list[int] = []
    for path in plan_dir.glob("*.json"):
        try:
            ids.append(int(path.stem))
        except Exception:
            continue
    return max(ids) if ids else None


def select_area_mission(view: dict[str, Any]) -> tuple[int, dict[str, Any]] | None:
    from modules.monitoring.gui.tabs.mission_progress_area_management_tab import _mission_geometry_kind

    for entry in view.get("uav_entries") or []:
        try:
            aircraft_id = int(entry.get("aircraft_id"))
        except Exception:
            continue
        for mission in entry.get("missions") or []:
            if not isinstance(mission, dict):
                continue
            if _mission_geometry_kind(mission) != "area":
                continue
            if len(mission.get("sweep_line_coordinate_lists") or []) < 3:
                continue
            if len(mission.get("waypoints") or []) < 3:
                continue
            return aircraft_id, mission
    return None


def build_area_state(aircraft_id: int, mission: dict[str, Any], *, source_plan_id: int | None) -> Any:
    from modules.monitoring.logic.mission_coverage import build_mission_coverage_definition
    from modules.monitoring.gui.tabs.mission_progress_area_management_tab import (
        _MissionAreaState,
        _build_planned_sweep_lines,
    )

    coverage_def = build_mission_coverage_definition(mission)
    expect_true("area coverage definition", coverage_def is not None)
    planned_lines, cut_half_width_m = _build_planned_sweep_lines(mission, coverage_def)
    expect_true("area planned sweep lines", len(planned_lines) > 0)
    return _MissionAreaState(
        mission_id=int(mission["individual_mission_id"]),
        aircraft_id=int(aircraft_id),
        input_id=mission.get("input_id"),
        mission_type="area",
        source_plan_id=source_plan_id,
        path_id=mission.get("path_id"),
        coverage_def=coverage_def,
        width_hint_m=mission.get("width_m"),
        assignment_geometry=coverage_def.assignment_geometry,
        planned_area_m2=float(coverage_def.planned_area_m2),
        source_area_list=list(mission.get("area_list") or []),
        source_coordinate_list=list(mission.get("coordinate_list") or []),
        input_area_list=list(mission.get("input_area_list") or []),
        input_coordinate_list=list(mission.get("input_coordinate_list") or []),
        is_current=True,
        planned_cut_lines=planned_lines,
        sweep_waypoint_ids={
            int(wp["waypoint_id"])
            for wp in mission.get("waypoints") or []
            if wp.get("waypoint_id") is not None
            and int(wp.get("line_search_point_count") or 0) > 0
        },
        cut_half_width_m=cut_half_width_m,
    )


def progress_for_area_point(
    view: dict[str, Any],
    aircraft_id: int,
    mission: dict[str, Any],
    *,
    sweep_index: int,
    point_index: int,
) -> dict[str, Any]:
    from modules.monitoring.logic.mission_progress import MissionProgressTracker

    sweep_lists = mission.get("sweep_line_coordinate_lists") or []
    line_waypoint_ids = [
        int(wp["waypoint_id"])
        for wp in mission.get("waypoints") or []
        if int(wp.get("line_search_point_count") or 0) > 0
    ]
    expect_true("area sweep index available", sweep_index < len(sweep_lists))
    expect_true("area waypoint index available", sweep_index < len(line_waypoint_ids))
    coords = sweep_lists[int(sweep_index)]
    expect_true("area sweep coordinate list", bool(coords))
    point_index = max(0, min(int(point_index), len(coords) - 1))
    target_coord = dict(coords[point_index])

    tracker = MissionProgressTracker()
    tracker.reset(view)
    # This smoke jumps directly to an area waypoint, so clear the initial active
    # mission baseline and test the waypoint-resolution path itself.
    tracker._aircraft_current_mission.pop(int(aircraft_id), None)
    state = {
        "aircraft_id": int(aircraft_id),
        "current_waypoint_id": int(line_waypoint_ids[int(sweep_index)]),
        "coordinate": dict(target_coord),
        "sensor_center_coordinate": dict(target_coord),
        "flying": 1,
        "filming": 1,
    }
    snapshot = tracker.update(123456789, [state])
    mission_id = int(mission["individual_mission_id"])
    progress = (snapshot.get("mission_progress") or {}).get(mission_id) or {}
    expect_equal("area progress current waypoint", progress.get("current_waypoint_id"), state["current_waypoint_id"])
    expect_true("area progress points", int(progress.get("sweep_progress_points") or 0) > 0)
    expect_true("area sweep point count", int(progress.get("sweep_point_count") or 0) > 0)
    return progress


def check_area_progress_mapping(db_root: Path, plan_id: int) -> None:
    from modules.monitoring.logic.mission_update import build_uav_mission_view
    from modules.monitoring.gui.tabs.mission_progress_area_management_tab import (
        _apply_sweep_point_progress_to_area_state,
        _area_remaining_segments_for_state,
        _attach_remaining_detail_to_area_ownership,
        _area_ownership_details_for_states,
        _area_progress_details_for_states,
        _remaining_geometry_for_state,
        _remaining_geometry_diagnostics,
    )
    from modules.mission_planning.replanning.triggers.next_collab.pipeline import _area_planner_components_from_detail

    view = build_uav_mission_view(plan_id, uav_ids=(4, 5, 6), db_root=db_root)
    selected = select_area_mission(view)
    if selected is None:
        print("area 0401 progress smoke skipped: no area mission with sweep lines")
        return
    aircraft_id, mission = selected

    progress = progress_for_area_point(
        view,
        aircraft_id,
        mission,
        sweep_index=1,
        point_index=10,
    )
    state = build_area_state(aircraft_id, mission, source_plan_id=plan_id)
    changed = _apply_sweep_point_progress_to_area_state(state, progress)
    expect_true("area progress changed boundary", changed)
    expect_true("area progress boundary set", state.progress_boundary_line_index is not None)
    expect_true("area completed indexes", bool(state.completed_cut_line_indexes))
    expect_equal("area progress source", state.area_progress_source, "0401_sweep_points")
    expect_equal("area progress waypoint", state.area_progress_current_waypoint_id, progress.get("current_waypoint_id"))
    progress_details = _area_progress_details_for_states([state])
    expect_equal("area progress detail count", len(progress_details), 1)
    expect_equal("area progress detail source", progress_details[0].get("progressSource"), "0401_sweep_points")
    expect_equal("area progress detail area source", progress_details[0].get("areaProgressSource"), "0401_sweep_points")
    expect_equal("area progress detail source plan", progress_details[0].get("sourceMissionPlanID"), plan_id)
    expect_equal("area progress detail path", progress_details[0].get("pathID"), mission.get("path_id"))
    expect_equal("area progress detail confidence", progress_details[0].get("confidence"), "current_waypoint_line_search_match")
    remaining_geometry = _remaining_geometry_for_state(state)
    area_segments = _area_remaining_segments_for_state(
        state,
        remaining_geometry,
        state.coverage_def.transformer,
        altitude=0.0,
    )
    expect_true("area remaining row segments", bool(area_segments))
    segment_components = _area_planner_components_from_detail(
        {
            "areaSegmentList": area_segments,
            "areaSegmentPolicy": "planned_sweep_row_remaining",
            "areaList": [],
        }
    )
    expect_true("area segment planner components", bool(segment_components))
    expect_equal(
        "area segment component source",
        segment_components[0].get("componentSource"),
        "planned_sweep_row_segment",
    )
    ownership_details = _area_ownership_details_for_states([state])
    expect_equal("area ownership detail count", len(ownership_details), 1)
    expect_equal("area ownership detail source plan", ownership_details[0].get("sourceMissionPlanID"), plan_id)
    expect_equal("area ownership detail path", ownership_details[0].get("pathID"), mission.get("path_id"))
    owner_remaining_detail = {
        "coordinateList": [],
        "lineList": [],
        "areaList": [{"isHole": False, "coordinateList": [{"latitude": 0.0, "longitude": 0.0}]}],
    }
    _attach_remaining_detail_to_area_ownership(
        ownership_details,
        [state],
        lambda _state: (owner_remaining_detail, 12.5),
    )
    expect_equal("area ownership remaining area", ownership_details[0].get("remainingAreaM2"), 12.5)
    expect_equal("area ownership takeover policy", ownership_details[0].get("takeoverPolicy"), "piece_only")
    expect_equal("area ownership remaining detail", ownership_details[0].get("remainingDetail"), owner_remaining_detail)
    diagnostics = _remaining_geometry_diagnostics(
        mission_type="area",
        remaining_detail={
            "coordinateList": [],
            "lineList": [],
            "areaList": [],
            "areaSegmentList": area_segments,
            "areaSegmentPolicy": "planned_sweep_row_remaining",
        },
        area_progress_details=progress_details,
        area_ownership_details=ownership_details,
    )
    expect_equal("area diagnostics display source", diagnostics.get("displayCoverageSource"), "footprint_plus_0401_sweep_frontier")
    expect_equal("area diagnostics segment geometry", diagnostics.get("replanInputGeometry"), "area_segment_list")
    expect_equal("area diagnostics progress count", diagnostics.get("areaProgressDetailCount"), 1)
    expect_equal("area diagnostics ownership count", diagnostics.get("areaOwnershipDetailCount"), 1)
    decision_categories = {
        str(item.get("category"))
        for item in diagnostics.get("operatorDecisions") or []
        if isinstance(item, dict)
    }
    expect_true("area diagnostics monotonic decision", "monotonic_progress_trim" in decision_categories)
    expect_true("area diagnostics preserved decision", "preserved_assignment" in decision_categories)

    current_boundary = int(state.progress_boundary_line_index or 0)
    current_progress_boundary = state.area_progress_boundary_line_index
    current_progress_points = state.area_progress_sweep_points
    lower_progress = dict(progress)
    lower_progress["sweep_progress_points"] = 1
    changed = _apply_sweep_point_progress_to_area_state(state, lower_progress)
    expect_equal("area lower progress does not change", changed, False)
    expect_equal("area boundary monotonic", state.progress_boundary_line_index, current_boundary)
    expect_equal("area progress provenance boundary monotonic", state.area_progress_boundary_line_index, current_progress_boundary)
    expect_equal("area progress provenance points monotonic", state.area_progress_sweep_points, current_progress_points)

    stale_state = build_area_state(aircraft_id, mission, source_plan_id=plan_id)
    stale_progress = dict(progress)
    stale_progress["current_waypoint_id"] = None
    changed = _apply_sweep_point_progress_to_area_state(stale_state, stale_progress)
    expect_equal("area stale waypoint fails closed", changed, False)
    expect_equal("area stale boundary", stale_state.progress_boundary_line_index, None)

    wrong_waypoint_state = build_area_state(aircraft_id, mission, source_plan_id=plan_id)
    wrong_waypoint_progress = dict(progress)
    wrong_waypoint_progress["current_waypoint_id"] = 999999999
    changed = _apply_sweep_point_progress_to_area_state(wrong_waypoint_state, wrong_waypoint_progress)
    expect_equal("area wrong waypoint fails closed", changed, False)
    expect_equal("area wrong waypoint boundary", wrong_waypoint_state.progress_boundary_line_index, None)

    missing_count_state = build_area_state(aircraft_id, mission, source_plan_id=plan_id)
    missing_count_progress = dict(progress)
    missing_count_progress["sweep_point_count"] = 0
    changed = _apply_sweep_point_progress_to_area_state(missing_count_state, missing_count_progress)
    expect_equal("area missing sweep count fallback unchanged", changed, False)
    expect_equal("area missing sweep count boundary", missing_count_state.progress_boundary_line_index, None)


def check_area_polygon_component_support() -> None:
    from modules.monitoring.gui.tabs.mission_progress_area_management_tab import _remaining_geometry_diagnostics
    from modules.mission_planning.replanning.triggers.remaining_hybrid.general import _build_single_area_polygon
    from modules.mission_planning.replanning.triggers.next_collab import pipeline as next_collab

    outer_a = {
        "isHole": False,
        "coordinateList": [
            {"latitude": 37.0, "longitude": 127.0, "altitude": 100},
            {"latitude": 37.0, "longitude": 127.01, "altitude": 100},
            {"latitude": 37.01, "longitude": 127.01, "altitude": 100},
            {"latitude": 37.01, "longitude": 127.0, "altitude": 100},
        ],
    }
    outer_b = {
        "isHole": False,
        "coordinateList": [
            {"latitude": 37.02, "longitude": 127.02, "altitude": 100},
            {"latitude": 37.02, "longitude": 127.03, "altitude": 100},
            {"latitude": 37.03, "longitude": 127.03, "altitude": 100},
            {"latitude": 37.03, "longitude": 127.02, "altitude": 100},
        ],
    }
    hole = {
        "isHole": True,
        "coordinateList": [
            {"latitude": 37.002, "longitude": 127.002, "altitude": 100},
            {"latitude": 37.002, "longitude": 127.004, "altitude": 100},
            {"latitude": 37.004, "longitude": 127.004, "altitude": 100},
            {"latitude": 37.004, "longitude": 127.002, "altitude": 100},
        ],
    }

    multi_components = next_collab._area_planner_components_from_detail({"areaList": [outer_a, outer_b]})
    hole_components = next_collab._area_planner_components_from_detail({"areaList": [outer_a, hole]})
    expect_equal("area multi-polygon component count", len(multi_components), 2)
    expect_true("area hole component decomposition", len(hole_components) >= 1)
    expect_equal("area multi component source", multi_components[0].get("componentSource"), "multi_polygon")
    expect_equal("area hole component source", hole_components[0].get("componentSource"), "hole_decomposition")
    expect_true(
        "area hole component decomposition mode",
        str(hole_components[0].get("componentDecomposition") or "") in {"hole_cut", "triangulated_hole"},
    )
    complex_holes: list[dict[str, Any]] = [outer_a]
    for idx in range(30):
        base_lat = 37.001 + (idx % 10) * 0.0007
        base_lon = 127.001 + (idx // 10) * 0.002
        complex_holes.append(
            {
                "isHole": True,
                "coordinateList": [
                    {"latitude": base_lat, "longitude": base_lon, "altitude": 100},
                    {"latitude": base_lat, "longitude": base_lon + 0.0003, "altitude": 100},
                    {"latitude": base_lat + 0.0003, "longitude": base_lon + 0.0003, "altitude": 100},
                    {"latitude": base_lat + 0.0003, "longitude": base_lon, "altitude": 100},
                ],
            }
        )
    complex_hole_components = next_collab._area_planner_components_from_detail({"areaList": complex_holes})
    expect_true("area complex holes planned below cap", bool(complex_hole_components))
    expect_true(
        "area complex hole component count below cap",
        len(complex_hole_components) <= int(next_collab.AREA_PLANNER_COMPONENT_MAX_COUNT),
    )
    expect_true(
        "area complex hole cut decomposition",
        any(str(component.get("componentDecomposition") or "") == "hole_cut" for component in complex_hole_components),
    )
    expect_equal(
        "area segment component limit fails closed",
        next_collab._area_planner_components_from_detail(
            {
                "areaSegmentList": [
                    {
                        "source": "planned_sweep_row",
                        "lineIndex": idx,
                        "coordinateList": outer_a["coordinateList"],
                    }
                    for idx in range(int(next_collab.AREA_PLANNER_COMPONENT_MAX_COUNT) + 1)
                ]
            }
        ),
        [],
    )

    class FakeReservation:
        def __init__(self) -> None:
            self.next_individual_id = 9000
            self.next_path_id = 6000

        def next_individual(self) -> int:
            value = self.next_individual_id
            self.next_individual_id += 1
            return int(value)

        def next_path(self, _aircraft_id: int) -> int:
            value = self.next_path_id
            self.next_path_id += 1
            return int(value)

        def summary(self) -> dict[str, Any]:
            return {"fake": True}

    planner_calls: list[list[dict[str, Any]]] = []

    def fake_planner(*, mission_polygon, aircraft_entries, turn_radius_scale, log):
        planner_calls.append([dict(coord) for coord in mission_polygon])
        piece = next_collab.SplitPiece(
            parent_order=1,
            mission_id=1,
            mission_type=2,
            piece_index=1,
            data={"coordinateList": [dict(coord) for coord in mission_polygon]},
            assigned_uav=4,
        )
        split_result = next_collab.SplitRunResult(
            uav_count=1,
            uav_ids=[4],
            pieces=[piece],
        )
        class FakePlannerResult:
            pass

        result = FakePlannerResult()
        result.workflow = f"fake_component_{len(planner_calls)}"
        result.split_result = split_result
        result.mid_line_segments = [{"component": len(planner_calls)}]
        result.expected_paths = [
            {
                "aircraftID": 4,
                "pieceIndex": 1,
                "source": "make_path_2",
                "targetLabel": f"C{len(planner_calls)}",
            }
        ]
        result.planner_result_text = "ok"
        return result

    original_planner = next_collab.run_next_collab_division_plan
    original_reserve = next_collab._reserve_next_collab_replacement_ids
    original_build_fps = next_collab._build_replacement_flight_paths
    original_build_info = next_collab.build_mission_info_from_planned_row
    try:
        next_collab.run_next_collab_division_plan = fake_planner
        next_collab._reserve_next_collab_replacement_ids = lambda **_kwargs: FakeReservation()
        next_collab._build_replacement_flight_paths = lambda items, **_kwargs: {
            int(path_id): {"pathID": int(path_id), "aircraftID": 4}
            for path_id, _builder in items
        }
        next_collab.build_mission_info_from_planned_row = lambda _row, template_info=None, fallback_polygon_coords=None: {
            "coordinateList": [],
            "lineList": [],
            "areaList": [{"isHole": False, "coordinateList": list(fallback_polygon_coords or [])}],
        }
        prepared = next_collab._prepare_area_replacements(
            target_input_mission={"inputMissionID": 1, "missionDetail": {"areaList": [outer_a, outer_b]}},
            target_input_id=1,
            target_aircraft_ids=[4],
            entry_coord_map={4: {"latitude": 37.0, "longitude": 127.0, "altitude": 100}},
            heading_map={},
            representative_entry=None,
            template_record_map={},
            now_ms=1,
            turn_radius_scale=1.0,
            emit=lambda _message: None,
        )
    finally:
        next_collab.run_next_collab_division_plan = original_planner
        next_collab._reserve_next_collab_replacement_ids = original_reserve
        next_collab._build_replacement_flight_paths = original_build_fps
        next_collab.build_mission_info_from_planned_row = original_build_info

    expect_true("area multi component prepared", prepared is not None)
    expect_equal("area multi component planner calls", len(planner_calls), 2)
    expect_equal("area multi component review count", (prepared.review_report or {}).get("areaPlannerComponentCount"), 2)
    expect_true(
        "area multi component workflow",
        str(getattr(prepared, "planner_workflow", "")).startswith("multi_component:"),
    )

    expect_equal(
        "legacy remaining hybrid multi-polygon fail closed",
        _build_single_area_polygon({"areaList": [outer_a, outer_b]}),
        [],
    )
    expect_equal(
        "legacy remaining hybrid hole fail closed",
        _build_single_area_polygon({"areaList": [outer_a, hole]}),
        [],
    )
    component_diag = _remaining_geometry_diagnostics(
        mission_type="area",
        remaining_detail={"coordinateList": [], "lineList": [], "areaList": [outer_a, hole]},
        area_progress_details=[],
        area_ownership_details=[],
    )
    component_categories = {
        str(item.get("category"))
        for item in component_diag.get("operatorDecisions") or []
        if isinstance(item, dict)
    }
    expect_equal(
        "area diagnostics component geometry",
        component_diag.get("replanInputGeometry"),
        "area_component_decomposition_multi_polygon_or_hole",
    )
    expect_true("area diagnostics component planner decision", "planner_redivision" in component_categories)
    empty_diag = _remaining_geometry_diagnostics(
        mission_type="area",
        remaining_detail={"coordinateList": [], "lineList": [], "areaList": []},
        area_progress_details=[],
        area_ownership_details=[],
    )
    empty_categories = {
        str(item.get("category"))
        for item in empty_diag.get("operatorDecisions") or []
        if isinstance(item, dict)
    }
    expect_true("area diagnostics empty fail-closed decision", "fail_closed_skip" in empty_categories)


def check_area_piece_only_takeover_policy() -> None:
    from modules.mission_planning.replanning.triggers.prior import pipeline as prior
    from modules.mission_planning.replanning.triggers.next_collab import pipeline as next_collab

    def area_row(min_lat: float, min_lon: float, max_lat: float, max_lon: float) -> dict[str, Any]:
        return {
            "isHole": False,
            "coordinateList": [
                {"latitude": min_lat, "longitude": min_lon, "altitude": 100},
                {"latitude": min_lat, "longitude": max_lon, "altitude": 100},
                {"latitude": max_lat, "longitude": max_lon, "altitude": 100},
                {"latitude": max_lat, "longitude": min_lon, "altitude": 100},
            ],
        }

    def area_bounds(detail: dict[str, Any]) -> tuple[float, float, float, float]:
        coords: list[dict[str, Any]] = []
        for row in detail.get("areaList") or []:
            if isinstance(row, dict):
                coords.extend([dict(coord) for coord in row.get("coordinateList") or [] if isinstance(coord, dict)])
        expect_true("area bounds coords", bool(coords))
        lats = [float(coord["latitude"]) for coord in coords]
        lons = [float(coord["longitude"]) for coord in coords]
        return min(lats), min(lons), max(lats), max(lons)

    full_area = area_row(0.0, 0.0, 10.0, 10.0)
    owner_4 = area_row(0.0, 0.0, 1.0, 1.0)
    owner_5 = area_row(2.0, 2.0, 3.0, 3.0)
    snapshot_entry = {
        "inputMissionID": 1,
        "missionType": "area",
        "remainingAreaM2": 5000.0,
        "areaProgressDetails": [
            {
                "progressSource": "0401_sweep_point",
                "sourceMissionPlanID": 700000001,
                "pathID": 400000001,
                "currentWaypointID": 3001,
                "sweepProgressPoints": 40,
                "sweepPointCount": 100,
                "mappedBoundaryLineIndex": 4,
                "confidence": "tracked",
            }
        ],
        "remainingDetail": {
            "coordinateList": [],
            "lineList": [],
            "areaList": [full_area],
            "areaSegmentList": [
                {
                    "source": "planned_sweep_row",
                    "lineIndex": 6,
                    "aircraftID": 5,
                    "individualMissionID": 900000005,
                    "inputMissionID": 1,
                    "areaM2": 5000.0,
                    "coordinateList": full_area["coordinateList"],
                }
            ],
            "areaSegmentPolicy": "planned_sweep_row_remaining",
        },
        "areaOwnershipPolicy": "piece_only_takeover",
        "areaOwnershipDetails": [
            {
                "aircraftID": 4,
                "individualMissionID": 900000004,
                "inputMissionID": 1,
                "sourceMissionPlanID": 700000001,
                "pathID": 400000004,
                "takeoverPolicy": "piece_only",
                "remainingDetail": {"coordinateList": [], "lineList": [], "areaList": [owner_4]},
            },
            {
                "aircraftID": 5,
                "individualMissionID": 900000005,
                "inputMissionID": 1,
                "sourceMissionPlanID": 700000001,
                "pathID": 400000005,
                "takeoverPolicy": "piece_only",
                "remainingDetail": {
                    "coordinateList": [],
                    "lineList": [],
                    "areaList": [owner_5],
                    "areaSegmentList": [
                        {
                            "source": "planned_sweep_row",
                            "lineIndex": 7,
                            "aircraftID": 5,
                            "individualMissionID": 900000005,
                            "inputMissionID": 1,
                            "areaM2": 1000.0,
                            "coordinateList": owner_5["coordinateList"],
                        }
                    ],
                    "areaSegmentPolicy": "planned_sweep_row_remaining",
                },
            },
        ],
        "geometryDiagnostics": {"replanInputGeometry": "area_segment_list", "operatorDecisions": []},
    }

    owner_detail, owner_ids = prior._area_owner_remaining_detail_for_unavailable(snapshot_entry, {5})
    expect_equal("area owner helper ids", owner_ids, [5])
    expect_true("area owner helper geometry", prior._remaining_detail_has_geometry(owner_detail))
    expect_equal("area owner helper bounds", area_bounds(owner_detail or {}), (2.0, 2.0, 3.0, 3.0))
    expect_true(
        "piece-only next-collab policy detected",
        next_collab._is_piece_only_area_takeover_input(
            {
                "areaOwnershipPolicy": "piece_only_takeover",
                "areaTakeoverSourceAircraftIDs": [5],
            }
        ),
    )

    missing_detail, missing_ids = prior._area_owner_remaining_detail_for_unavailable(snapshot_entry, {6})
    expect_equal("area owner helper missing detail", missing_detail, None)
    expect_equal("area owner helper missing ids", missing_ids, [])

    original_load_input = prior._load_input_plan_for_source_plan
    original_load_snapshot_entry = prior.mission_area_replan_store.load_snapshot_entry
    original_audit_snapshot_entry_rejected = prior.mission_area_replan_store.audit_snapshot_entry_rejected
    rejected_entries: list[dict[str, Any]] = []
    try:
        prior._load_input_plan_for_source_plan = lambda _source_plan_id: {
            "inputMissionList": [
                {
                    "inputMissionID": 1,
                    "isDone": False,
                    "missionDetail": {
                        "coordinateList": [],
                        "lineList": [],
                        "areaList": [full_area],
                    },
                },
                {
                    "inputMissionID": 2,
                    "isDone": False,
                    "missionDetail": {"coordinateList": [{"latitude": 20.0, "longitude": 20.0}]},
                },
            ],
        }
        prior.mission_area_replan_store.load_snapshot_entry = lambda *_args, **_kwargs: {"entry": snapshot_entry}
        prior.mission_area_replan_store.audit_snapshot_entry_rejected = (
            lambda entry, **kwargs: rejected_entries.append({"entry": deepcopy(entry), **dict(kwargs)})
        )

        current_mission, next_mission = prior._build_remaining_input_mission_for_collaborative_replan(
            source_plan_id=700000001,
            current_input_id=1,
            unavailable_aircraft_ids={5},
        )
        expect_true("piece-only current mission", isinstance(current_mission, dict))
        expect_true("piece-only current mission active", not bool((current_mission or {}).get("isDone")))
        expect_equal("piece-only policy", (current_mission or {}).get("areaOwnershipPolicy"), "piece_only_takeover")
        expect_equal("piece-only source ids", (current_mission or {}).get("areaTakeoverSourceAircraftIDs"), [5])
        expect_equal(
            "piece-only mission detail bounds",
            area_bounds(((current_mission or {}).get("missionDetail") or {})),
            (2.0, 2.0, 3.0, 3.0),
        )
        expect_equal(
            "piece-only segment preserved",
            (((current_mission or {}).get("missionDetail") or {}).get("areaSegmentList") or [{}])[0].get("lineIndex"),
            7,
        )
        expect_equal("piece-only next mission", (next_mission or {}).get("inputMissionID"), 2)
        active_missions = [
            {
                "individualMissionID": 101,
                "relatedMission": {"inputMissionID": 1},
                "pathID": 1001,
            },
            {
                "individualMissionID": 102,
                "relatedMission": {"inputMissionID": 2},
                "pathID": 1002,
            },
        ]
        takeover_mission = {
            "individualMissionID": 201,
            "relatedMission": {"inputMissionID": 1},
            "pathID": 2001,
        }
        preserved_missions, preserved_policy = next_collab._merge_replacements_into_active_missions(
            active_mission_list=active_missions,
            replacements=[takeover_mission],
            target_input_id=1,
            preserve_current_input=True,
        )
        expect_equal(
            "piece-only preserves original then inserts takeover",
            [mission.get("individualMissionID") for mission in preserved_missions],
            [101, 201, 102],
        )
        expect_equal(
            "piece-only insert policy",
            preserved_policy.get("policy"),
            "preserve_current_input_then_takeover_piece",
        )
        replaced_missions, replaced_policy = next_collab._merge_replacements_into_active_missions(
            active_mission_list=active_missions,
            replacements=[takeover_mission],
            target_input_id=1,
            preserve_current_input=False,
        )
        expect_equal(
            "normal replacement removes original target",
            [mission.get("individualMissionID") for mission in replaced_missions],
            [201, 102],
        )
        expect_equal("normal insert policy", replaced_policy.get("policy"), "replace_current_input")

        missing_mission, _ = prior._build_remaining_input_mission_for_collaborative_replan(
            source_plan_id=700000001,
            current_input_id=1,
            unavailable_aircraft_ids={6},
        )
        expect_true("piece-only missing mission", isinstance(missing_mission, dict))
        expect_equal("piece-only missing done", (missing_mission or {}).get("isDone"), True)
        expect_equal(
            "piece-only missing reason",
            (missing_mission or {}).get("areaTakeoverSkippedReason"),
            "missing_unavailable_owner_remaining_detail",
        )
        unready_snapshot_entry = {
            "inputMissionID": 1,
            "missionType": "area",
            "remainingAreaM2": 5000.0,
            "remainingDetail": {
                "coordinateList": [],
                "lineList": [],
                "areaList": [full_area],
            },
        }
        prior.mission_area_replan_store.load_snapshot_entry = (
            lambda *_args, **_kwargs: {"entry": unready_snapshot_entry, "snapshotMissionPlanID": 700000001}
        )
        unready_mission, _ = prior._build_remaining_input_mission_for_collaborative_replan(
            source_plan_id=700000001,
            current_input_id=1,
            unavailable_aircraft_ids={5},
        )
        expect_true("unready area snapshot mission returned", isinstance(unready_mission, dict))
        expect_equal("unready area snapshot done", (unready_mission or {}).get("isDone"), True)
        expect_equal(
            "unready area snapshot reason",
            (unready_mission or {}).get("areaTakeoverSkippedReason"),
            "area_snapshot_not_ready_for_replan",
        )
        expect_equal("unready area snapshot rejection audit count", len(rejected_entries), 1)
        expect_equal(
            "unready area snapshot rejection context",
            rejected_entries[-1].get("audit_context"),
            "prior_collaborative_resume_remaining_input",
        )
    finally:
        prior._load_input_plan_for_source_plan = original_load_input
        prior.mission_area_replan_store.load_snapshot_entry = original_load_snapshot_entry
        prior.mission_area_replan_store.audit_snapshot_entry_rejected = original_audit_snapshot_entry_rejected


def check_snapshot_store_area_field_summary() -> None:
    from modules.common.mission_area_replan_store import (
        _snapshot_area_field_summary,
        _snapshot_entry_area_field_summary,
        snapshot_entry_ready_for_replan,
    )

    empty_entry = {
        "inputMissionID": 1,
        "missionType": "area",
        "remainingDetail": {"coordinateList": [], "lineList": [], "areaList": []},
    }
    empty_entry_summary = _snapshot_entry_area_field_summary(empty_entry)
    expect_equal("empty entry area", empty_entry_summary.get("isAreaEntry"), True)
    expect_equal("empty entry field ready", empty_entry_summary.get("areaEntryNewFieldReady"), False)
    expect_equal("empty entry replan ready", snapshot_entry_ready_for_replan(empty_entry), False)
    expect_equal(
        "empty entry missing categories",
        empty_entry_summary.get("areaEntryMissingNewFieldCategories"),
        ["areaProgressDetails", "areaOwnershipDetails", "areaSegmentList", "geometryDiagnostics"],
    )
    empty_summary = _snapshot_area_field_summary({"missions": [empty_entry]})
    expect_equal("empty snapshot area mission count", empty_summary.get("areaMissionCount"), 1)
    expect_equal("empty snapshot field ready", empty_summary.get("areaSnapshotNewFieldReady"), False)
    expect_equal(
        "empty snapshot missing categories",
        empty_summary.get("missingNewFieldCategories"),
        ["areaProgressDetails", "areaOwnershipDetails", "areaSegmentList", "geometryDiagnostics"],
    )
    line_only_summary = _snapshot_area_field_summary(
        {
            "missions": [
                {
                    "inputMissionID": 2,
                    "missionType": "line",
                    "remainingDetail": {"coordinateList": [], "lineList": []},
                }
            ]
        }
    )
    expect_equal("line-only snapshot area count", line_only_summary.get("areaMissionCount"), 0)
    expect_equal("line-only snapshot missing categories", line_only_summary.get("missingNewFieldCategories"), [])
    expect_equal("line-only snapshot ready", line_only_summary.get("areaSnapshotNewFieldReady"), False)

    incomplete_entry = {
        "inputMissionID": 3,
        "missionType": "area",
        "areaProgressDetails": [{"mappedBoundaryLineIndex": 3}],
        "areaOwnershipDetails": [{"takeoverPolicy": "piece_only"}],
        "geometryDiagnostics": {"replanInputGeometry": "area_segment_list"},
        "remainingDetail": {
            "coordinateList": [],
            "lineList": [],
            "areaList": [],
            "areaSegmentPolicy": "planned_sweep_row_remaining",
            "areaSegmentList": [
                {
                    "source": "planned_sweep_row",
                    "lineIndex": 4,
                    "coordinateList": [
                        {"latitude": 37.0, "longitude": 127.0},
                        {"latitude": 37.0, "longitude": 127.1},
                        {"latitude": 37.1, "longitude": 127.1},
                    ],
                }
            ],
        },
    }
    incomplete_entry_summary = _snapshot_entry_area_field_summary(incomplete_entry)
    expect_equal("incomplete entry field ready", incomplete_entry_summary.get("areaEntryNewFieldReady"), False)
    expect_equal(
        "incomplete entry missing strict categories",
        incomplete_entry_summary.get("areaEntryMissingNewFieldCategories"),
        [
            "areaProgressDetails.requiredKeys",
            "areaOwnershipDetails.requiredKeys",
            "areaSegmentList.validRows",
        ],
    )
    expect_true(
        "incomplete entry progress missing keys",
        bool((incomplete_entry_summary.get("areaProgressMissingKeys") or [[]])[0]),
    )
    expect_equal("incomplete entry invalid segment indexes", incomplete_entry_summary.get("invalidAreaSegmentIndexes"), [0])
    incomplete_snapshot_summary = _snapshot_area_field_summary({"missions": [incomplete_entry]})
    expect_equal("incomplete snapshot ready", incomplete_snapshot_summary.get("areaSnapshotNewFieldReady"), False)

    full_entry = {
        "inputMissionID": 1,
        "missionType": "area",
        "areaProgressDetails": [
            {
                "progressSource": "0401_sweep_point",
                "sourceMissionPlanID": 700000100,
                "pathID": 400000001,
                "currentWaypointID": 3001,
                "sweepProgressPoints": 25,
                "sweepPointCount": 100,
                "mappedBoundaryLineIndex": 3,
                "confidence": "tracked",
            }
        ],
        "areaOwnershipDetails": [
            {
                "aircraftID": 5,
                "individualMissionID": 900000001,
                "inputMissionID": 1,
                "sourceMissionPlanID": 700000100,
                "pathID": 400000001,
                "takeoverPolicy": "piece_only",
                "remainingDetail": {
                    "coordinateList": [],
                    "lineList": [],
                    "areaList": [],
                    "areaSegmentPolicy": "planned_sweep_row_remaining",
                    "areaSegmentList": [
                        {
                            "source": "planned_sweep_row",
                            "lineIndex": 4,
                            "aircraftID": 5,
                            "individualMissionID": 900000001,
                            "inputMissionID": 1,
                            "areaM2": 1200.0,
                            "coordinateList": [
                                {"latitude": 37.0, "longitude": 127.0},
                                {"latitude": 37.0, "longitude": 127.1},
                                {"latitude": 37.1, "longitude": 127.1},
                            ],
                        }
                    ],
                },
            }
        ],
        "geometryDiagnostics": {"replanInputGeometry": "area_segment_list"},
        "remainingDetail": {
            "coordinateList": [],
            "lineList": [],
            "areaList": [],
            "areaSegmentPolicy": "planned_sweep_row_remaining",
            "areaSegmentList": [
                {
                    "source": "planned_sweep_row",
                    "lineIndex": 4,
                    "aircraftID": 5,
                    "individualMissionID": 900000001,
                    "inputMissionID": 1,
                    "areaM2": 1200.0,
                    "coordinateList": [
                        {"latitude": 37.0, "longitude": 127.0},
                        {"latitude": 37.0, "longitude": 127.1},
                        {"latitude": 37.1, "longitude": 127.1},
                    ],
                }
            ],
        },
    }
    full_entry_summary = _snapshot_entry_area_field_summary(full_entry)
    expect_equal("full entry readiness schema", full_entry_summary.get("areaReadinessSchemaVersion"), 2)
    expect_equal("full entry field ready", full_entry_summary.get("areaEntryNewFieldReady"), True)
    expect_equal("full entry replan ready", snapshot_entry_ready_for_replan(full_entry), True)
    expect_equal("full entry missing categories", full_entry_summary.get("areaEntryMissingNewFieldCategories"), [])
    expect_equal("full entry replan geometry", full_entry_summary.get("replanInputGeometry"), "area_segment_list")
    full_summary = _snapshot_area_field_summary({"missions": [full_entry]})
    expect_equal("full snapshot area mission count", full_summary.get("areaMissionCount"), 1)
    expect_equal("full snapshot readiness schema", full_summary.get("areaReadinessSchemaVersion"), 2)
    expect_equal("full snapshot progress count", full_summary.get("areaProgressDetailCount"), 1)
    expect_equal("full snapshot ownership count", full_summary.get("areaOwnershipDetailCount"), 1)
    expect_equal("full snapshot segment count", full_summary.get("areaSegmentCount"), 1)
    expect_equal("full snapshot diagnostic count", full_summary.get("geometryDiagnosticsMissionCount"), 1)
    expect_equal("full snapshot field ready", full_summary.get("areaSnapshotNewFieldReady"), True)
    expect_equal("full snapshot missing categories", full_summary.get("missingNewFieldCategories"), [])
    expect_equal(
        "full snapshot geometry counts",
        full_summary.get("replanInputGeometryCounts"),
        {"area_segment_list": 1},
    )
    expect_equal(
        "full snapshot segment policy counts",
        full_summary.get("areaSegmentPolicyCounts"),
        {"planned_sweep_row_remaining": 1},
    )

    done_entry = {
        "inputMissionID": 4,
        "missionType": "area",
        "remainingAreaM2": 0.0,
        "isDone": True,
        "areaProgressDetails": [
            {
                "progressSource": "0401_sweep_point",
                "sourceMissionPlanID": 700000100,
                "pathID": 400000001,
                "currentWaypointID": 3099,
                "sweepProgressPoints": 100,
                "sweepPointCount": 100,
                "mappedBoundaryLineIndex": 9,
                "confidence": "tracked",
            }
        ],
        "areaOwnershipDetails": [
            {
                "aircraftID": 5,
                "individualMissionID": 900000004,
                "inputMissionID": 4,
                "sourceMissionPlanID": 700000100,
                "pathID": 400000001,
                "takeoverPolicy": "piece_only",
                "remainingAreaM2": 0.0,
                "isDone": True,
                "remainingDetail": {"coordinateList": [], "lineList": [], "areaList": []},
            }
        ],
        "geometryDiagnostics": {
            "replanInputGeometry": "none",
            "areaSegmentPolicy": "",
        },
        "remainingDetail": {"coordinateList": [], "lineList": [], "areaList": []},
    }
    done_entry_summary = _snapshot_entry_area_field_summary(done_entry)
    expect_equal("done entry field ready", done_entry_summary.get("areaEntryNewFieldReady"), True)
    expect_equal("done entry replan ready", snapshot_entry_ready_for_replan(done_entry), True)
    expect_equal("done entry segment count", done_entry_summary.get("areaSegmentCount"), 0)
    expect_equal(
        "done entry completed empty flag",
        done_entry_summary.get("areaEntryCompletedWithoutRemainingGeometry"),
        True,
    )
    expect_equal("done entry missing categories", done_entry_summary.get("areaEntryMissingNewFieldCategories"), [])
    done_summary = _snapshot_area_field_summary({"missions": [done_entry]})
    expect_equal("done snapshot field ready", done_summary.get("areaSnapshotNewFieldReady"), True)
    expect_equal("done snapshot segment count", done_summary.get("areaSegmentCount"), 0)
    expect_equal("done snapshot missing categories", done_summary.get("missingNewFieldCategories"), [])


def check_snapshot_store_area_audit_persistence() -> None:
    from modules.common import mission_area_replan_store

    area_entry = {
        "inputMissionID": 10,
        "missionType": "area",
        "remainingAreaM2": 1200.0,
        "areaProgressDetails": [
            {
                "progressSource": "0401_sweep_point",
                "sourceMissionPlanID": 700000101,
                "pathID": 400000010,
                "currentWaypointID": 3001,
                "sweepProgressPoints": 18,
                "sweepPointCount": 72,
                "mappedBoundaryLineIndex": 2,
                "confidence": "tracked",
            }
        ],
        "areaOwnershipDetails": [
            {
                "aircraftID": 5,
                "individualMissionID": 900000010,
                "inputMissionID": 10,
                "sourceMissionPlanID": 700000101,
                "pathID": 400000010,
                "takeoverPolicy": "piece_only",
                "remainingDetail": {
                    "coordinateList": [],
                    "lineList": [],
                    "areaList": [],
                    "areaSegmentPolicy": "planned_sweep_row_remaining",
                    "areaSegmentList": [
                        {
                            "source": "planned_sweep_row",
                            "lineIndex": 2,
                            "aircraftID": 5,
                            "individualMissionID": 900000010,
                            "inputMissionID": 10,
                            "areaM2": 1200.0,
                            "coordinateList": [
                                {"latitude": 37.0, "longitude": 127.0},
                                {"latitude": 37.0, "longitude": 127.1},
                                {"latitude": 37.1, "longitude": 127.1},
                            ],
                        }
                    ],
                },
            }
        ],
        "geometryDiagnostics": {"replanInputGeometry": "area_segment_list", "operatorDecisions": []},
        "remainingDetail": {
            "coordinateList": [],
            "lineList": [],
            "areaList": [],
            "areaSegmentPolicy": "planned_sweep_row_remaining",
            "areaSegmentList": [
                {
                    "source": "planned_sweep_row",
                    "lineIndex": 2,
                    "aircraftID": 5,
                    "individualMissionID": 900000010,
                    "inputMissionID": 10,
                    "areaM2": 1200.0,
                    "coordinateList": [
                        {"latitude": 37.0, "longitude": 127.0},
                        {"latitude": 37.0, "longitude": 127.1},
                        {"latitude": 37.1, "longitude": 127.1},
                    ],
                }
            ],
        },
    }
    original_detail_dir = mission_area_replan_store._detail_dir
    with tempfile.TemporaryDirectory() as tmp_dir:
        temp_root = Path(tmp_dir)
        mission_area_replan_store._detail_dir = lambda: temp_root
        try:
            mission_area_replan_store.save_snapshot(
                700000101,
                {
                    "missionPlanID": 700000101,
                    "missionCount": 1,
                    "missions": [area_entry],
                },
            )
            exact = mission_area_replan_store.load_snapshot_entry(
                700000101,
                10,
                allow_latest=False,
                audit_context="smoke_area_exact_read",
            )
            fallback = mission_area_replan_store.load_snapshot_entry(
                700000102,
                10,
                allow_latest=True,
                audit_context="smoke_area_latest_fallback",
            )
            carried_path = mission_area_replan_store.carry_forward_snapshot(
                700000101,
                700000103,
                reason="smoke_area_carry_forward",
            )
            grown_entry = deepcopy(area_entry)
            grown_entry["missionPlanID"] = 700000101
            grown_entry["remainingAreaM2"] = 999999.0
            grown_entry["remainingDetail"] = {
                "coordinateList": [],
                "lineList": [],
                "areaList": [
                    {
                        "isHole": False,
                        "coordinateList": [
                            {"latitude": 37.0, "longitude": 127.0},
                            {"latitude": 37.0, "longitude": 127.5},
                            {"latitude": 37.5, "longitude": 127.5},
                        ],
                    }
                ],
            }
            mission_area_replan_store.save_snapshot(
                700000101,
                {
                    "missionPlanID": 700000101,
                    "missionCount": 1,
                    "missions": [grown_entry],
                },
            )
            for context in (
                "mission_planning_gui_current_remaining_snapshot_apply",
                "mission_planning_gui_reexecute_first_snapshot_apply",
                "prior_collaborative_resume_remaining_input",
                "attack_collaborative_resume_remaining_input",
                "post_attack_collaborative_resume_remaining_input",
                "post_attack_remaining_area_detail",
            ):
                mission_area_replan_store.audit_snapshot_entry_access(
                    area_entry,
                    requested_mission_plan_id=700000101,
                    snapshot_mission_plan_id=700000101,
                    audit_context=context,
                    event="snapshot_entry_exact",
                )
            mission_area_replan_store.audit_snapshot_entry_access(
                area_entry,
                requested_mission_plan_id=700000101,
                snapshot_mission_plan_id=700000101,
                audit_context="smoke_area_direct_map",
                event="snapshot_entry_exact",
            )
            unready_area_entry = {
                "inputMissionID": 10,
                "missionType": "area",
                "remainingAreaM2": 1200.0,
                "remainingDetail": {
                    "coordinateList": [],
                    "lineList": [],
                    "areaList": [
                        {
                            "isHole": False,
                            "coordinateList": [
                                {"latitude": 37.0, "longitude": 127.0},
                                {"latitude": 37.0, "longitude": 127.1},
                                {"latitude": 37.1, "longitude": 127.1},
                            ],
                        }
                    ],
                },
            }
            mission_area_replan_store.audit_snapshot_entry_rejected(
                unready_area_entry,
                requested_mission_plan_id=700000101,
                snapshot_mission_plan_id=700000101,
                audit_context="smoke_area_unready_reject",
            )
            expect_true("audit exact snapshot entry", isinstance(exact, dict) and bool(exact.get("exact")))
            expect_true("audit fallback snapshot entry", isinstance(fallback, dict) and not bool(fallback.get("exact")))
            expect_true("audit carried snapshot path", carried_path is not None and Path(carried_path).exists())

            audit_path = temp_root / mission_area_replan_store._AUDIT_BASENAME
            expect_true("area audit file exists", audit_path.exists())
            rows = [
                json.loads(line)
                for line in audit_path.read_text(encoding="utf-8").splitlines()
                if line.strip()
            ]
            saved_rows = [row for row in rows if row.get("event") == "snapshot_saved"]
            carried_rows = [row for row in rows if row.get("event") == "snapshot_carried_forward"]
            preserved_rows = [row for row in rows if row.get("event") == "snapshot_entry_preserved"]
            exact_rows = [row for row in rows if row.get("event") == "snapshot_entry_exact"]
            fallback_rows = [row for row in rows if row.get("event") == "snapshot_entry_latest_fallback"]
            rejected_rows = [row for row in rows if row.get("event") == "snapshot_entry_rejected_unready"]
            exact_read_rows = [row for row in exact_rows if row.get("auditContext") == "smoke_area_exact_read"]
            direct_rows = [row for row in exact_rows if row.get("auditContext") == "smoke_area_direct_map"]
            expect_true("snapshot_saved audit row", bool(saved_rows))
            expect_true("snapshot_carried_forward audit row", bool(carried_rows))
            expect_true("snapshot_entry_preserved audit row", bool(preserved_rows))
            expect_true("snapshot_entry_exact audit row", bool(exact_rows))
            expect_true("snapshot_entry exact-read audit row", bool(exact_read_rows))
            expect_true("snapshot_entry_latest_fallback audit row", bool(fallback_rows))
            expect_true("snapshot_entry direct-map audit row", bool(direct_rows))
            expect_true("snapshot_entry rejected audit row", bool(rejected_rows))
            expect_equal("snapshot_saved field ready", saved_rows[-1].get("areaSnapshotNewFieldReady"), True)
            expect_equal("snapshot_saved readiness schema", saved_rows[-1].get("areaReadinessSchemaVersion"), 2)
            expect_equal("snapshot_saved segment count", saved_rows[-1].get("areaSegmentCount"), 1)
            expect_equal("snapshot_carried field ready", carried_rows[-1].get("areaSnapshotNewFieldReady"), True)
            expect_equal("snapshot_carried readiness schema", carried_rows[-1].get("areaReadinessSchemaVersion"), 2)
            expect_equal("snapshot_preserved field ready", preserved_rows[-1].get("areaEntryNewFieldReady"), True)
            expect_equal("snapshot_preserved reason", preserved_rows[-1].get("reason"), "remaining_area_grew")
            expect_equal("snapshot_entry_exact field ready", exact_read_rows[-1].get("areaEntryNewFieldReady"), True)
            expect_equal("snapshot_entry_exact readiness schema", exact_read_rows[-1].get("areaReadinessSchemaVersion"), 2)
            expect_equal("snapshot_entry_exact geometry", exact_read_rows[-1].get("replanInputGeometry"), "area_segment_list")
            expect_equal("snapshot_entry_exact context", exact_read_rows[-1].get("auditContext"), "smoke_area_exact_read")
            expect_equal("snapshot_entry_fallback field ready", fallback_rows[-1].get("areaEntryNewFieldReady"), True)
            expect_equal("snapshot_entry_fallback source plan", fallback_rows[-1].get("snapshotMissionPlanID"), 700000101)
            expect_equal(
                "snapshot_entry_fallback context",
                fallback_rows[-1].get("auditContext"),
                "smoke_area_latest_fallback",
            )
            expect_equal("snapshot_entry direct-map field ready", direct_rows[-1].get("areaEntryNewFieldReady"), True)
            expect_equal("snapshot_entry rejected field ready", rejected_rows[-1].get("areaEntryNewFieldReady"), False)
            expect_equal(
                "snapshot_entry rejected reason",
                rejected_rows[-1].get("rejectReason"),
                "area_snapshot_not_ready_for_replan",
            )

            verified_done_entry = deepcopy(area_entry)
            verified_done_entry["remainingAreaM2"] = 0.0
            verified_done_entry["coveragePercent"] = 100
            verified_done_entry["isDone"] = True
            verified_done_entry["remainingDetail"] = {"coordinateList": [], "lineList": [], "areaList": []}
            verified_done_entry["geometryDiagnostics"] = {"replanInputGeometry": "none", "operatorDecisions": []}
            for owner in verified_done_entry.get("areaOwnershipDetails") or []:
                if isinstance(owner, dict):
                    owner["remainingAreaM2"] = 0.0
                    owner["isDone"] = True
                    owner["remainingDetail"] = {"coordinateList": [], "lineList": [], "areaList": []}
            expect_equal(
                "verified done entry ready before merge",
                mission_area_replan_store.snapshot_entry_ready_for_replan(verified_done_entry),
                True,
            )
            mission_area_replan_store.save_snapshot(
                700000104,
                {
                    "missionPlanID": 700000104,
                    "missionCount": 1,
                    "missions": [verified_done_entry],
                },
            )
            resurrecting_ready_entry = deepcopy(area_entry)
            resurrecting_ready_entry["remainingAreaM2"] = 1200.0
            resurrecting_ready_entry["isDone"] = False
            mission_area_replan_store.save_snapshot(
                700000104,
                {
                    "missionPlanID": 700000104,
                    "missionCount": 1,
                    "missions": [resurrecting_ready_entry],
                },
            )
            preserved_done_snapshot = mission_area_replan_store.load_snapshot(700000104)
            preserved_done_entry = ((preserved_done_snapshot or {}).get("missions") or [{}])[0]
            expect_equal("verified done anti-resurrection", preserved_done_entry.get("isDone"), True)
            expect_equal("verified done remaining area preserved", preserved_done_entry.get("remainingAreaM2"), 0.0)

            legacy_done_entry = {
                "inputMissionID": 10,
                "missionType": "area",
                "remainingAreaM2": 0.0,
                "isDone": True,
                "remainingDetail": {"coordinateList": [], "lineList": [], "areaList": []},
            }
            mission_area_replan_store.save_snapshot(
                700000105,
                {
                    "missionPlanID": 700000105,
                    "missionCount": 1,
                    "missions": [legacy_done_entry],
                },
            )
            mission_area_replan_store.save_snapshot(
                700000105,
                {
                    "missionPlanID": 700000105,
                    "missionCount": 1,
                    "missions": [{**deepcopy(area_entry), "isDone": False}],
                },
            )
            recovered_snapshot = mission_area_replan_store.load_snapshot(700000105)
            recovered_entry = ((recovered_snapshot or {}).get("missions") or [{}])[0]
            expect_equal("legacy done strict-ready recovery active", recovered_entry.get("isDone"), False)
            expect_equal("legacy done strict-ready recovery segments", recovered_entry.get("remainingAreaM2"), 1200.0)

            collector_path = PROJECT_ROOT / "docs" / "mission planning refactoring" / "collect_area_0401_replay_fixture.py"
            spec = importlib.util.spec_from_file_location("collect_area_0401_replay_fixture_smoke", collector_path)
            expect_true("collector import spec", spec is not None and spec.loader is not None)
            collector = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(collector)
            collector_done_status = collector._mission_field_status(
                {
                    "inputMissionID": 11,
                    "missionType": "area",
                    "remainingAreaM2": 0.0,
                    "isDone": True,
                    "areaProgressDetails": area_entry["areaProgressDetails"],
                    "areaOwnershipDetails": [
                        {
                            "aircraftID": 5,
                            "individualMissionID": 900000011,
                            "inputMissionID": 11,
                            "sourceMissionPlanID": 700000101,
                            "pathID": 400000010,
                            "takeoverPolicy": "piece_only",
                            "remainingAreaM2": 0.0,
                            "isDone": True,
                            "remainingDetail": {"coordinateList": [], "lineList": [], "areaList": []},
                        }
                    ],
                    "geometryDiagnostics": {"replanInputGeometry": "none", "operatorDecisions": []},
                    "remainingDetail": {"coordinateList": [], "lineList": [], "areaList": []},
                }
            )
            expect_equal("collector done area status ready", collector_done_status.get("ready"), True)
            expect_equal("collector done area missing categories", collector_done_status.get("missingCategories"), [])
            report = collector.collect_candidates(temp_root)
            expect_true("collector audit ready count", int((report.get("counts") or {}).get("auditReady") or 0) >= 5)
            expect_equal("collector rejected unready count", (report.get("counts") or {}).get("rejectedUnready"), 1)
            expect_equal("collector missing flow groups", report.get("missingFlowGroups"), [])
            expect_equal(
                "collector flow coverage line count",
                len(report.get("flowCoverageLines") or []),
                len(report.get("flowAuditContextGroups") or {}),
            )
            expect_true(
                "collector flow coverage line detail",
                any(
                    "attack_collaborative_resume" in str(line)
                    and "attack_collaborative_resume_remaining_input" in str(line)
                    for line in (report.get("flowCoverageLines") or [])
                ),
            )
        finally:
            mission_area_replan_store._detail_dir = original_detail_dir


def check_mission_gui_area_snapshot_apply_preserves_segments() -> None:
    from modules.mission_planning import mission_planning_gui as gui
    from modules.mission_planning.replanning.triggers.post_attack import pipeline as post_attack

    def segment(line_index: int) -> dict[str, Any]:
        return {
            "source": "planned_sweep_row",
            "lineIndex": int(line_index),
            "aircraftID": 5,
            "individualMissionID": 900000201,
            "inputMissionID": 1,
            "areaM2": 1200.0,
            "coordinateList": [
                {"latitude": 37.0, "longitude": 127.0},
                {"latitude": 37.0, "longitude": 127.1},
                {"latitude": 37.1, "longitude": 127.1},
            ],
        }

    ready_entry = {
        "inputMissionID": 1,
        "missionType": "area",
        "remainingAreaM2": 1200.0,
        "areaProgressDetails": [
            {
                "progressSource": "0401_sweep_point",
                "sourceMissionPlanID": 700000201,
                "pathID": 400000201,
                "currentWaypointID": 3201,
                "sweepProgressPoints": 30,
                "sweepPointCount": 90,
                "mappedBoundaryLineIndex": 3,
                "confidence": "tracked",
            }
        ],
        "areaOwnershipDetails": [
            {
                "aircraftID": 5,
                "individualMissionID": 900000201,
                "inputMissionID": 1,
                "sourceMissionPlanID": 700000201,
                "pathID": 400000201,
                "takeoverPolicy": "piece_only",
                "remainingDetail": {
                    "coordinateList": [],
                    "lineList": [],
                    "areaList": [],
                    "areaSegmentPolicy": "planned_sweep_row_remaining",
                    "areaSegmentList": [segment(8)],
                },
            }
        ],
        "geometryDiagnostics": {"replanInputGeometry": "area_segment_list", "operatorDecisions": []},
        "remainingDetail": {
            "coordinateList": [
                {"latitude": 0.0, "longitude": 0.0},
                {"latitude": 0.0, "longitude": 1.0},
                {"latitude": 1.0, "longitude": 1.0},
            ],
            "lineList": [],
            "areaList": [],
            "areaSegmentPolicy": "planned_sweep_row_remaining",
            "areaSegmentList": [segment(8)],
        },
    }
    unready_entry = {
        "inputMissionID": 1,
        "missionType": "area",
        "remainingAreaM2": 1200.0,
        "remainingDetail": {
            "coordinateList": [],
            "lineList": [],
            "areaList": [
                {
                    "isHole": False,
                    "coordinateList": [
                        {"latitude": 38.0, "longitude": 128.0},
                        {"latitude": 38.0, "longitude": 128.1},
                        {"latitude": 38.1, "longitude": 128.1},
                    ],
                }
            ],
        },
    }

    original_load_snapshot = gui.mission_area_replan_store.load_snapshot
    original_load_snapshot_entry = gui.mission_area_replan_store.load_snapshot_entry
    original_audit_access = gui.mission_area_replan_store.audit_snapshot_entry_access
    original_audit_rejected = gui.mission_area_replan_store.audit_snapshot_entry_rejected
    audit_rows: list[dict[str, Any]] = []
    rejected_rows: list[dict[str, Any]] = []
    try:
        gui.mission_area_replan_store.load_snapshot = lambda _plan_id: {
            "missionPlanID": 700000201,
            "missions": [deepcopy(ready_entry)],
        }
        gui.mission_area_replan_store.load_snapshot_entry = lambda *_args, **_kwargs: None
        gui.mission_area_replan_store.audit_snapshot_entry_access = (
            lambda entry, **kwargs: audit_rows.append({"entry": deepcopy(entry), **dict(kwargs)})
        )
        gui.mission_area_replan_store.audit_snapshot_entry_rejected = (
            lambda entry, **kwargs: rejected_rows.append({"entry": deepcopy(entry), **dict(kwargs)})
        )
        payload = {
            "inputMissionList": [
                {
                    "inputMissionID": 1,
                    "isDone": False,
                    "missionDetail": {
                        "coordinateList": [{"latitude": 1.0, "longitude": 1.0}],
                        "areaList": [],
                        "areaSegmentList": [segment(99)],
                    },
                }
            ]
        }
        result = gui._override_input_missions_with_remaining_snapshot(
            payload,
            source_plan_id=700000201,
            mission_whitelist={1},
            audit_context="smoke_gui_segment_apply",
        )
        mission_detail = ((payload.get("inputMissionList") or [{}])[0].get("missionDetail") or {})
        expect_equal("gui area segment apply count", result.get("applied"), 1)
        expect_equal("gui area segment marked done count", result.get("marked_done"), 0)
        expect_equal("gui area segment preserved", (mission_detail.get("areaSegmentList") or [{}])[0].get("lineIndex"), 8)
        expect_equal("gui area segment policy", mission_detail.get("areaSegmentPolicy"), "planned_sweep_row_remaining")
        expect_equal("gui area segment coordinate cleared", mission_detail.get("coordinateList"), [])
        expect_equal("gui area stale segment replaced", len(mission_detail.get("areaSegmentList") or []), 1)
        expect_equal("gui area rejected ready snapshot count", len(rejected_rows), 0)
        expect_true("gui area exact audit", bool(audit_rows))
        post_attack_rows = post_attack._area_rows_from_detail(
            {
                "coordinateList": [],
                "lineList": [],
                "areaList": [],
                "areaSegmentList": [segment(12)],
            }
        )
        expect_equal("post-attack segment-only area row count", len(post_attack_rows), 1)
        expect_equal(
            "post-attack segment-only first coordinate",
            post_attack_rows[0]["coordinateList"][0]["latitude"],
            37.0,
        )

        audit_rows.clear()
        rejected_rows.clear()
        gui.mission_area_replan_store.load_snapshot = lambda _plan_id: {
            "missionPlanID": 700000201,
            "missions": [deepcopy(unready_entry)],
        }
        unready_payload = {
            "inputMissionList": [
                {
                    "inputMissionID": 1,
                    "isDone": False,
                    "missionDetail": {"coordinateList": [{"latitude": 1.0, "longitude": 1.0}]},
                }
            ]
        }
        unready_result = gui._override_input_missions_with_remaining_snapshot(
            unready_payload,
            source_plan_id=700000201,
            mission_whitelist={1},
            audit_context="smoke_gui_unready_reject",
        )
        unready_mission = (unready_payload.get("inputMissionList") or [{}])[0]
        expect_equal("gui unready geometry marked done", unready_result.get("marked_done"), 0)
        expect_equal("gui unready geometry applied", unready_result.get("applied"), 1)
        expect_equal("gui unready geometry mission active", unready_mission.get("isDone"), False)
        expect_equal(
            "gui unready geometry preserved",
            len(((unready_mission.get("missionDetail") or {}).get("areaList") or [])),
            1,
        )
        expect_equal("gui unready rejection count", len(rejected_rows), 1)
        expect_equal("gui unready rejection context", rejected_rows[-1].get("audit_context"), "smoke_gui_unready_reject")
    finally:
        gui.mission_area_replan_store.load_snapshot = original_load_snapshot
        gui.mission_area_replan_store.load_snapshot_entry = original_load_snapshot_entry
        gui.mission_area_replan_store.audit_snapshot_entry_access = original_audit_access
        gui.mission_area_replan_store.audit_snapshot_entry_rejected = original_audit_rejected


def check_area_flow_contracts() -> None:
    checks = [
        (
            "current remaining uses next-collab prepare",
            PROJECT_ROOT / "modules" / "mission_planning" / "replanning" / "triggers" / "remaining_hybrid" / "current.py",
            "prepare_next_collab_input_replacements(",
        ),
        (
            "current remaining routes reexecute-first",
            PROJECT_ROOT / "modules" / "mission_planning" / "replanning" / "triggers" / "remaining_hybrid" / "current.py",
            "prepare_reexecute_first_mission_replacements(",
        ),
        (
            "reexecute-first uses next-collab prepare",
            PROJECT_ROOT / "modules" / "mission_planning" / "replanning" / "triggers" / "remaining_hybrid" / "reexecute_first.py",
            "prepare_next_collab_input_replacements(",
        ),
        (
            "attack collaborative resume route",
            PROJECT_ROOT / "modules" / "mission_planning" / "replanning" / "triggers" / "attack" / "pipeline.py",
            "_prepare_uav_collaborative_resume_replan(",
        ),
        (
            "prior collaborative resume passes unavailable",
            PROJECT_ROOT / "modules" / "mission_planning" / "replanning" / "triggers" / "prior" / "pipeline.py",
            "unavailable_aircraft_ids={int(aid) for aid in unavailable_aircraft_ids}",
        ),
        (
            "post-attack precheck passes unavailable",
            PROJECT_ROOT / "modules" / "mission_planning" / "replanning" / "triggers" / "post_attack" / "pipeline.py",
            "unavailable_aircraft_ids={int(aid) for aid in unavailable_aircraft_ids}",
        ),
        (
            "next-collab preserves piece-only current area",
            PROJECT_ROOT / "modules" / "mission_planning" / "replanning" / "triggers" / "next_collab" / "pipeline.py",
            "preserve_current_input_then_takeover_piece",
        ),
        (
            "next-collab reports area component failure reason",
            PROJECT_ROOT / "modules" / "mission_planning" / "replanning" / "triggers" / "next_collab" / "pipeline.py",
            "_area_planner_component_input_summary(",
        ),
        (
            "monitoring tab exposes area diagnostics group",
            PROJECT_ROOT / "modules" / "monitoring" / "gui" / "tabs" / "mission_progress_area_management_tab.py",
            "Area 재계획 진단",
        ),
        (
            "monitoring tab populates area diagnostics",
            PROJECT_ROOT / "modules" / "monitoring" / "gui" / "tabs" / "mission_progress_area_management_tab.py",
            "_populate_area_diagnostics_table(",
        ),
        (
            "monitoring tab resolves selected snapshot mission",
            PROJECT_ROOT / "modules" / "monitoring" / "gui" / "tabs" / "mission_progress_area_management_tab.py",
            "_snapshot_mission_for_selected_mission(",
        ),
        (
            "monitoring tab displays operator decisions",
            PROJECT_ROOT / "modules" / "monitoring" / "gui" / "tabs" / "mission_progress_area_management_tab.py",
            "operator_decisions",
        ),
        (
            "mission GUI audits exact snapshot map entries",
            PROJECT_ROOT / "modules" / "mission_planning" / "mission_planning_gui.py",
            "audit_snapshot_entry_access(",
        ),
        (
            "mission GUI rejects unready area snapshots",
            PROJECT_ROOT / "modules" / "mission_planning" / "mission_planning_gui.py",
            "snapshot_entry_ready_for_replan(snapshot_entry)",
        ),
        (
            "mission GUI separates current remaining area audit context",
            PROJECT_ROOT / "modules" / "mission_planning" / "mission_planning_gui.py",
            "mission_planning_gui_current_remaining_snapshot_apply",
        ),
        (
            "mission GUI separates reexecute area audit context",
            PROJECT_ROOT / "modules" / "mission_planning" / "mission_planning_gui.py",
            "mission_planning_gui_reexecute_first_snapshot_apply",
        ),
        (
            "area replay fixture collector exists",
            PROJECT_ROOT / "docs" / "mission planning refactoring" / "collect_area_0401_replay_fixture.py",
            "collect_candidates(",
        ),
        (
            "area replay fixture collector strict option",
            PROJECT_ROOT / "docs" / "mission planning refactoring" / "collect_area_0401_replay_fixture.py",
            "--strict",
        ),
        (
            "area replay fixture collector flow strict option",
            PROJECT_ROOT / "docs" / "mission planning refactoring" / "collect_area_0401_replay_fixture.py",
            "--require-flow-contexts",
        ),
        (
            "area replay fixture collector audit scan",
            PROJECT_ROOT / "docs" / "mission planning refactoring" / "collect_area_0401_replay_fixture.py",
            "mission_area_snapshot_audit.jsonl",
        ),
        (
            "area replay fixture collector flow coverage lines",
            PROJECT_ROOT / "docs" / "mission planning refactoring" / "collect_area_0401_replay_fixture.py",
            "flowCoverageLines",
        ),
        (
            "attack area audit context",
            PROJECT_ROOT / "modules" / "mission_planning" / "replanning" / "triggers" / "attack" / "pipeline.py",
            "attack_collaborative_resume_remaining_input",
        ),
        (
            "post-attack collaborative area audit context",
            PROJECT_ROOT / "modules" / "mission_planning" / "replanning" / "triggers" / "post_attack" / "pipeline.py",
            "post_attack_collaborative_resume_remaining_input",
        ),
        (
            "post-attack rejects unready area snapshots",
            PROJECT_ROOT / "modules" / "mission_planning" / "replanning" / "triggers" / "post_attack" / "pipeline.py",
            "snapshot_entry_ready_for_replan(entry)",
        ),
        (
            "prior rejects unready area snapshots",
            PROJECT_ROOT / "modules" / "mission_planning" / "replanning" / "triggers" / "prior" / "pipeline.py",
            "area_snapshot_not_ready_for_replan",
        ),
    ]
    for label, path, needle in checks:
        expect_true(f"{label} file exists", path.exists())
        text = path.read_text(encoding="utf-8")
        expect_true(label, needle in text)


def check_captured_area_snapshot_replay() -> None:
    from modules.monitoring.gui.tabs.mission_progress_area_management_tab import _remaining_geometry_diagnostics
    from modules.mission_planning.replanning.triggers.next_collab.pipeline import _area_planner_components_from_detail

    logs_root = PROJECT_ROOT / "Logs"
    if not logs_root.exists():
        print("captured area snapshot replay skipped: Logs directory unavailable")
        return

    replayed: list[dict[str, Any]] = []
    for path in logs_root.rglob("mission_area_snapshot_*.json"):
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            continue
        for mission in payload.get("missions") or []:
            if not isinstance(mission, dict):
                continue
            if str(mission.get("missionType") or "").lower() != "area":
                continue
            detail = mission.get("remainingDetail") if isinstance(mission.get("remainingDetail"), dict) else {}
            area_list = detail.get("areaList") if isinstance(detail.get("areaList"), list) else []
            outer_count = sum(1 for row in area_list if isinstance(row, dict) and not bool(row.get("isHole")))
            hole_count = sum(1 for row in area_list if isinstance(row, dict) and bool(row.get("isHole")))
            if outer_count <= 1 and hole_count <= 0:
                continue
            components = _area_planner_components_from_detail(detail)
            expect_true(f"captured area components for {path.name}", bool(components))
            diagnostics = _remaining_geometry_diagnostics(
                mission_type="area",
                remaining_detail=detail,
                area_progress_details=[],
                area_ownership_details=[],
            )
            expect_equal(
                "captured multi/hole diagnostic geometry",
                diagnostics.get("replanInputGeometry"),
                "area_component_decomposition_multi_polygon_or_hole",
            )
            replayed.append(
                {
                    "path": str(path),
                    "missionPlanID": payload.get("missionPlanID"),
                    "inputMissionID": mission.get("inputMissionID"),
                    "componentCount": len(components),
                }
            )
            if len(replayed) >= 6:
                break
        if len(replayed) >= 6:
            break

    expect_true("captured multi/hole area snapshot replay cases", bool(replayed))


def check_captured_new_area_snapshot_field_audit() -> None:
    from modules.mission_planning.replanning.triggers.next_collab.pipeline import _area_planner_components_from_detail

    logs_root = PROJECT_ROOT / "Logs"
    if not logs_root.exists():
        print("captured new area snapshot field audit skipped: Logs directory unavailable")
        return

    strict = str(os.environ.get("AREA_0401_REQUIRE_CAPTURED_NEW_FIELDS") or "").lower() in {"1", "true", "yes"}
    counts = {
        "files": 0,
        "areaMissions": 0,
        "progress": 0,
        "ownership": 0,
        "segments": 0,
        "diagnostics": 0,
    }
    required_progress_keys = {
        "progressSource",
        "sourceMissionPlanID",
        "pathID",
        "currentWaypointID",
        "sweepProgressPoints",
        "sweepPointCount",
        "mappedBoundaryLineIndex",
        "confidence",
    }
    for path in logs_root.rglob("mission_area_snapshot_*.json"):
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            continue
        counts["files"] += 1
        for mission in payload.get("missions") or []:
            if not isinstance(mission, dict):
                continue
            if str(mission.get("missionType") or "").lower() != "area":
                continue
            counts["areaMissions"] += 1
            progress_details = [
                item
                for item in mission.get("areaProgressDetails") or []
                if isinstance(item, dict)
            ]
            if progress_details:
                counts["progress"] += 1
                for detail in progress_details:
                    missing = sorted(
                        key
                        for key in required_progress_keys
                        if detail.get(key) is None
                    )
                    expect_equal(f"captured area progress fields {path.name}", missing, [])
            ownership_details = [
                item
                for item in mission.get("areaOwnershipDetails") or []
                if isinstance(item, dict)
            ]
            if ownership_details:
                counts["ownership"] += 1
                for detail in ownership_details:
                    missing_owner_keys = sorted(
                        key
                        for key in (
                            "aircraftID",
                            "individualMissionID",
                            "inputMissionID",
                            "sourceMissionPlanID",
                            "pathID",
                            "takeoverPolicy",
                            "remainingDetail",
                        )
                        if detail.get(key) is None
                    )
                    expect_equal(f"captured area ownership fields {path.name}", missing_owner_keys, [])
                    expect_equal(
                        f"captured area ownership takeover policy {path.name}",
                        detail.get("takeoverPolicy"),
                        "piece_only",
                    )
                    expect_true(
                        f"captured area ownership remaining detail {path.name}",
                        isinstance(detail.get("remainingDetail"), dict),
                    )
            diagnostics = mission.get("geometryDiagnostics")
            if isinstance(diagnostics, dict):
                counts["diagnostics"] += 1
                expect_true(
                    f"captured area diagnostic geometry {path.name}",
                    bool(str(diagnostics.get("replanInputGeometry") or "")),
                )
                expect_true(
                    f"captured area diagnostic decisions {path.name}",
                    isinstance(diagnostics.get("operatorDecisions"), list),
                )
            detail = mission.get("remainingDetail") if isinstance(mission.get("remainingDetail"), dict) else {}
            area_segments = detail.get("areaSegmentList") if isinstance(detail.get("areaSegmentList"), list) else []
            if area_segments:
                counts["segments"] += 1
                components = _area_planner_components_from_detail(detail)
                expect_true(f"captured area segment components {path.name}", bool(components))
                for row in area_segments:
                    if not isinstance(row, dict):
                        continue
                    expect_equal(f"captured area segment source {path.name}", row.get("source"), "planned_sweep_row")
                    expect_true(f"captured area segment line index {path.name}", row.get("lineIndex") is not None)
                    expect_true(f"captured area segment aircraft {path.name}", row.get("aircraftID") is not None)
                    expect_true(
                        f"captured area segment mission {path.name}",
                        row.get("individualMissionID") is not None,
                    )
                    expect_true(
                        f"captured area segment input mission {path.name}",
                        row.get("inputMissionID") is not None,
                    )
                    expect_true(
                        f"captured area segment area {path.name}",
                        row.get("areaM2") is not None,
                    )
                    expect_true(
                        f"captured area segment coordinates {path.name}",
                        len(row.get("coordinateList") or []) >= 3,
                    )

    required_categories = ("progress", "ownership", "segments", "diagnostics")
    missing_categories = [name for name in required_categories if int(counts.get(name) or 0) <= 0]
    if strict and missing_categories:
        fail(f"captured new area snapshot field audit missing {missing_categories}: {counts}")
    if len(missing_categories) < len(required_categories):
        status = "ok" if not missing_categories else f"partial missing {missing_categories}"
        print(f"captured new area snapshot field audit {status}: {counts}")
        return
    message = f"captured new area snapshot field audit pending: {counts}"
    print(message)


def main() -> None:
    configure_import_paths()
    check_snapshot_store_area_field_summary()
    check_snapshot_store_area_audit_persistence()
    check_mission_gui_area_snapshot_apply_preserves_segments()
    check_area_flow_contracts()
    check_captured_area_snapshot_replay()
    check_captured_new_area_snapshot_field_audit()
    db_root = load_current_db_root()
    if db_root is None:
        print("area 0401 progress smoke skipped: current scenario unavailable")
        return
    plan_id = latest_plan_id(db_root)
    if plan_id is None:
        print("area 0401 progress smoke skipped: mission plan unavailable")
        return
    check_area_progress_mapping(db_root, plan_id)
    check_area_polygon_component_support()
    check_area_piece_only_takeover_policy()
    print("area 0401 progress smoke ok")


if __name__ == "__main__":
    main()
