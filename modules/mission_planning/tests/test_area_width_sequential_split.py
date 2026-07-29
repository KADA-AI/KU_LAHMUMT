# -*- coding: utf-8 -*-
"""AREA 폭 기반 순차 2분할 계약 (초기계획 + 다음협업기저임무).

할당된 영역의 스윕 행 폭(최대 현)이 임계(기본 700 m — fps15·좌우중첩 50%에서
한 행이 덮을 수 있는 좌우 폭)를 넘으면, 그 소유 기체의 영역을 인접 두 갈래로
쪼개 순차 수행한다.  기존 메커니즘 재사용:
- 초기계획: two_stage 조각 계약(splitStage 1/2, 같은 기체에 인접 쌍)
- 재계획: 가상 파트너 분할(_branch_aircraft_sequential_area_entries) 후 접기

폭 측정은 바운딩 투영이 아니라 실제 스윕 행 최대 현이다 — 비스듬한 500 m 폭
영역이 6.7° 회전만으로 731 m 로 보이는 과대평가를 막는다.
"""
from __future__ import annotations

import math
from copy import deepcopy
from types import SimpleNamespace

import pytest

from modules.mission_planning.MissionPlanner import capture_physics as cp
from modules.mission_planning.MissionPlanner.planning_enhanced.algo.split_algorithms import (
    divide_search_area_clip,
    divide_search_area_two_stage,
    llh_to_xy,
    split_mission_into_subareas,
)
from modules.mission_planning.MissionPlanner.runtime_settings import runtime_override
from modules.mission_planning.replanning.triggers.next_collab import pipeline

LAT0, LON0 = 38.0, 127.0


def _rect_coords(width_m: float, height_m: float) -> list[dict]:
    dlat = height_m / 111_132.9
    dlon = width_m / (111_320.0 * math.cos(math.radians(LAT0)))
    return [
        {"latitude": LAT0, "longitude": LON0, "altitude": 100},
        {"latitude": LAT0, "longitude": LON0 + dlon, "altitude": 100},
        {"latitude": LAT0 + dlat, "longitude": LON0 + dlon, "altitude": 100},
        {"latitude": LAT0 + dlat, "longitude": LON0, "altitude": 100},
    ]


def _area_mission(width_m: float, height_m: float) -> dict:
    return {
        "inputMissionID": 9,
        "inputMissionType": 2,
        "missionDetail": {
            "areaList": [{"isHole": False, "coordinateList": _rect_coords(width_m, height_m)}]
        },
    }


def _xy_coords(points: list[tuple[float, float]]) -> list[dict]:
    radius_m = 6_378_137.0
    cos_lat = math.cos(math.radians(LAT0))
    return [
        {
            "latitude": LAT0 + math.degrees(y / radius_m),
            "longitude": LON0 + math.degrees(x / (radius_m * cos_lat)),
            "altitude": 100,
        }
        for x, y in points
    ]


def _raw_piece_area_m2(piece: dict) -> float:
    from shapely.geometry import Polygon

    coords = piece.get("rawCoordinateList") or piece["coordinateList"]
    points_xy = [
        llh_to_xy(
            float(coord["latitude"]),
            float(coord["longitude"]),
            LAT0,
            LON0,
        )
        for coord in coords
    ]
    return float(Polygon(points_xy).area)


PREV = {"latitude": 37.99, "longitude": 127.0, "altitude": 100}


# ------------------------------------------------------------------ 폭 측정


def test_threshold_defaults_to_700_and_honors_runtime_override() -> None:
    assert abs(cp.area_sequential_split_width_m() - 700.0) < 1e-9
    assert abs(cp.area_sequential_split_max_width_m() - 900.0) < 1e-9
    assert (
        cp.area_sequential_split_width_m({"values": {"area_sequential_split_width_m": 0.0}})
        == 0.0
    )
    assert (
        cp.area_sequential_split_max_width_m(
            {"values": {"area_sequential_split_width_m": 0.0}}
        )
        == 0.0
    )


def test_row_chord_is_immune_to_small_rotations_unlike_the_bbox_span() -> None:
    coords = _rect_coords(500.0, 2000.0)
    bbox = cp.projected_span_m_llh(coords, 6.7)
    chord = cp.max_sweep_row_chord_m_llh(coords, 6.7)
    assert bbox > 700.0          # 바운딩 투영은 과대평가한다
    assert chord < 520.0         # 실제 행 현은 폭 그대로


# ------------------------------------------------------------- 초기임무계획


def test_tapered_area_split_balances_area_instead_of_projection_width() -> None:
    """A triangular end must not become a visibly tiny capture assignment."""

    triangle = _xy_coords([(0.0, 0.0), (3000.0, 0.0), (0.0, 2000.0)])
    pieces = divide_search_area_clip(triangle, 3, 0.0)
    areas = [_raw_piece_area_m2(piece) for piece in pieces]

    assert len(pieces) == 3
    assert max(areas) / min(areas) < 1.001


def test_two_stage_tapered_area_uses_an_area_bisector() -> None:
    """The two sequential filming stages should carry equal work on a taper."""

    triangle = _xy_coords([(0.0, 0.0), (3000.0, 0.0), (0.0, 2000.0)])
    pieces = divide_search_area_two_stage(
        triangle,
        1,
        boundary_axis_bearing_deg=0.0,
        entry_move_bearing_deg=0.0,
        exit_move_bearing_deg=0.0,
        prev_pt=_xy_coords([(-1000.0, 1000.0)])[0],
    )
    areas = [_raw_piece_area_m2(piece) for piece in pieces]

    assert len(pieces) == 2
    assert sorted(int(piece["splitStage"]) for piece in pieces) == [1, 2]
    assert max(areas) / min(areas) < 1.001


def test_two_stage_area_marks_only_edge_owners_outer_first() -> None:
    pieces = divide_search_area_two_stage(
        _rect_coords(3000.0, 2000.0),
        3,
        boundary_axis_bearing_deg=90.0,
        entry_move_bearing_deg=0.0,
        exit_move_bearing_deg=0.0,
        prev_pt=PREV,
    )

    assert len(pieces) == 6
    stage_one = [
        piece for piece in pieces if int(piece["splitStage"]) == 1
    ]
    stage_two = [
        piece for piece in pieces if int(piece["splitStage"]) == 2
    ]
    assert {
        str(piece.get("areaOuterSide"))
        for piece in stage_one
        if piece.get("areaOuterFirstSweep")
    } == {"min", "max"}
    assert not any(
        bool(piece.get("areaOuterFirstSweep"))
        for piece in stage_two
    )


def test_initial_narrow_area_stays_one_piece_per_owner() -> None:
    subs = split_mission_into_subareas(_area_mission(500.0, 2000.0), 1, PREV)
    assert len(subs) == 1
    assert not any(s.get("areaSequentialWidthSplit") for s in subs)


def test_initial_wide_area_uses_700m_target_with_900m_allowed_cap() -> None:
    subs = split_mission_into_subareas(_area_mission(2000.0, 2000.0), 1, PREV)
    assert len(subs) >= 3
    assert sorted(int(s.get("splitStage") or 0) for s in subs) == list(
        range(1, len(subs) + 1)
    )
    assert all(int(s.get("splitCount") or 0) == len(subs) for s in subs)
    assert all(s.get("areaSequentialWidthSplit") for s in subs)
    widest_stage_m = max(
        cp.max_sweep_row_chord_m_llh(
            s["coordinateList"],
            s["bearing_deg"],
        )
        for s in subs
    )
    assert 700.0 < widest_stage_m <= 901.0


def test_initial_wide_area_with_three_owners_keeps_every_stage_grouped() -> None:
    subs = split_mission_into_subareas(_area_mission(3000.0, 2000.0), 3, PREV)
    assert len(subs) >= 6
    assert len(subs) % 3 == 0
    stage_count = len(subs) // 3
    assert sorted(int(s.get("splitStage") or 0) for s in subs) == [
        stage
        for stage in range(1, stage_count + 1)
        for _owner in range(3)
    ]
    assert all(int(s.get("splitCount") or 0) == stage_count for s in subs)
    widest_stage_m = max(
        cp.max_sweep_row_chord_m_llh(
            s["coordinateList"],
            s["bearing_deg"],
        )
        for s in subs
    )
    assert 700.0 < widest_stage_m <= 901.0


def test_outer_owners_run_sequential_stages_from_hull_edge_inward() -> None:
    """Both edge UAVs must receive their real convex-hull edge as stage 1."""

    subs = split_mission_into_subareas(
        _area_mission(3000.0, 2000.0),
        3,
        PREV,
    )
    by_owner: dict[int, list[dict]] = {}
    for sub in subs:
        by_owner.setdefault(
            int(sub.get("areaSequentialOwnerSlot") or 0),
            [],
        ).append(sub)

    assert set(by_owner) == {1, 2, 3}
    assert all(len(rows) >= 2 for rows in by_owner.values())

    def _centroid_projection(sub: dict) -> float:
        coords = sub.get("rawCoordinateList") or sub["coordinateList"]
        points_xy = [
            llh_to_xy(
                float(coord["latitude"]),
                float(coord["longitude"]),
                LAT0,
                LON0,
            )
            for coord in coords
        ]
        bearing_rad = math.radians(float(sub["bearing_deg"]))
        normal_x = math.cos(bearing_rad)
        normal_y = -math.sin(bearing_rad)
        center_x = sum(point[0] for point in points_xy) / len(points_xy)
        center_y = sum(point[1] for point in points_xy) / len(points_xy)
        return normal_x * center_x + normal_y * center_y

    low_rows = sorted(by_owner[1], key=lambda row: int(row["splitStage"]))
    high_rows = sorted(by_owner[3], key=lambda row: int(row["splitStage"]))
    assert _centroid_projection(low_rows[0]) < _centroid_projection(low_rows[-1])
    assert _centroid_projection(high_rows[0]) > _centroid_projection(high_rows[-1])
    assert low_rows[0]["areaOuterSide"] == "min"
    assert high_rows[0]["areaOuterSide"] == "max"
    assert low_rows[0]["areaOuterFirstSweep"] is True
    assert high_rows[0]["areaOuterFirstSweep"] is True
    assert not any(
        bool(row.get("areaOuterFirstSweep"))
        for row in low_rows[1:] + high_rows[1:] + by_owner[2]
    )


def test_tapered_area_uses_uniform_width_stages_without_fragment_explosion() -> None:
    """Equal workload between owners must not force tiny sequential stages."""

    mission = {
        "inputMissionID": 9,
        "inputMissionType": 2,
        "missionDetail": {
            "areaList": [
                {
                    "isHole": False,
                    "coordinateList": _xy_coords(
                        [(0.0, 0.0), (3000.0, 0.0), (0.0, 2000.0)]
                    ),
                }
            ]
        },
    }

    subs = split_mission_into_subareas(mission, 1, PREV)
    spans_m = [
        cp.max_sweep_row_chord_m_llh(
            sub["coordinateList"],
            sub["bearing_deg"],
        )
        for sub in subs
    ]

    assert 4 <= len(subs) <= 6
    assert max(spans_m) <= 901.0


def test_initial_split_disabled_when_threshold_is_zero() -> None:
    with runtime_override({"values": {"area_sequential_split_width_m": 0.0}}):
        subs = split_mission_into_subareas(_area_mission(2000.0, 2000.0), 1, PREV)
    assert len(subs) == 1


# ------------------------------------------------- 초기계획: 쌍 단위 할당


def _pieces_for(width_m: float, height_m: float, owners: int):
    from modules.mission_planning.MissionPlanner.planning_enhanced.algo.split_runner import (
        SplitPiece,
    )

    subs = split_mission_into_subareas(_area_mission(width_m, height_m), owners, PREV)
    return [SplitPiece(1, 9, 2, i + 1, s) for i, s in enumerate(subs)]


def test_sequence_units_assign_one_complete_stage_set_per_uav_when_clustered() -> None:
    """기체들이 한 지점에 몰려 있어도 전부 한 대로 몰리면 안 된다."""

    from modules.mission_planning.MissionPlanner.planning_enhanced.algo.split_runner import (
        _assign_group_with_width_pairs,
    )

    group = _pieces_for(3000.0, 2000.0, 3)
    clustered = {a: {"latitude": 37.99, "longitude": 127.0} for a in (4, 5, 6)}
    assert _assign_group_with_width_pairs(group, [4, 5, 6], clustered) is True

    by_uav: dict[int, list[int]] = {}
    for piece in group:
        by_uav.setdefault(int(piece.assigned_uav), []).append(
            int(piece.data.get("splitStage"))
        )
    assert set(by_uav.keys()) == {4, 5, 6}
    stage_count = int(group[0].data["splitCount"])
    assert all(
        sorted(stages) == list(range(1, stage_count + 1))
        for stages in by_uav.values()
    )


def test_public_replan_preassignment_preserves_width_pairs() -> None:
    """The second assignment pass must not undo the pair-aware first pass."""

    from modules.mission_planning.MissionPlanner.planning_enhanced.algo.split_runner import (
        assign_split_result_by_takeover_distance,
    )

    pieces = _pieces_for(3000.0, 2000.0, 3)
    result = SimpleNamespace(pieces=pieces)
    report = assign_split_result_by_takeover_distance(
        result,
        {
            "takeOverInfoList": [
                {
                    "aircraftID": aircraft_id,
                    "coordinate": {
                        "latitude": 37.99,
                        "longitude": 127.0,
                        "altitude": 100,
                    },
                }
                for aircraft_id in (4, 5, 6)
            ]
        },
        [4, 5, 6],
    )

    stage_count = int(pieces[0].data["splitCount"])
    assert report["uavSummary"] == {
        4: stage_count,
        5: stage_count,
        6: stage_count,
    }
    owners_by_stage = [
        [
            int(piece.assigned_uav)
            for piece in pieces
            if int(piece.data["splitStage"]) == stage
        ]
        for stage in range(1, stage_count + 1)
    ]
    assert all(owners == owners_by_stage[0] for owners in owners_by_stage[1:])


def test_prediction_reassignment_solves_width_pairs_as_three_units() -> None:
    """The headless next-collab planner is the assignment path used in logs."""

    from modules.mission_planning.planners.next_collab_division._planner_window import (
        DivisionPlannerWindow,
    )

    pieces = _pieces_for(3000.0, 2000.0, 3)
    result = SimpleNamespace(pieces=pieces, branch_ownership={})
    targets = {
        int(piece.piece_index): (
            float(
                (
                    (int(piece.piece_index) - 1)
                    % 3
                )
                * 1000
            ),
            0.0,
        )
        for piece in pieces
    }
    fake_window = SimpleNamespace(
        _uav_prediction_points_by_id=lambda: {
            4: ((0.0, 0.0), (0.0, 0.0)),
            5: ((1000.0, 0.0), (1000.0, 0.0)),
            6: ((2000.0, 0.0), (2000.0, 0.0)),
        },
        _piece_assignment_target_xy=lambda piece: targets[int(piece.piece_index)],
        _mid_line_reference_bearing_deg=lambda _pieces: None,
        _default_turn_radius_m=lambda _aircraft_id=None: 500.0,
        _uav_state_for_aircraft=lambda _aircraft_id: None,
    )

    report = DivisionPlannerWindow._assign_split_result_by_prediction_distance(
        fake_window,
        result,
    )

    assert report["assignmentUnitCount"] == 3
    assert report["widthPairCount"] == 3
    stage_count = int(pieces[0].data["splitCount"])
    assert report["uavSummary"] == {
        4: stage_count,
        5: stage_count,
        6: stage_count,
    }
    owners_by_stage = [
        [
            int(piece.assigned_uav)
            for piece in pieces
            if int(piece.data["splitStage"]) == stage
        ]
        for stage in range(1, stage_count + 1)
    ]
    assert all(owners == owners_by_stage[0] for owners in owners_by_stage[1:])


def test_next_collab_area_waypoint_length_uses_full_polygon_projection() -> None:
    """An oblique/concave AREA must not inherit its short center chord."""

    from modules.mission_planning.planners.next_collab_division._planner_window import (
        DivisionPlannerWindow,
    )

    fake_window = SimpleNamespace(
        _build_turn_prefix_rows=lambda *_args, **_kwargs: (
            [(0.0, 0.0)],
            [],
            [],
        ),
    )
    row = {
        "aircraftID": 4,
        "originXY": (0.0, 0.0),
        "originHeadingDeg": 90.0,
        "tangentXY": (0.0, 0.0),
        "targetXY": (900.0, 0.0),
        "turnRadiusM": 500.0,
        "turnSpeedMps": 40.0,
        "midLineLengthM": 900.0,
        # The center chord is only 900 m, while the complete AREA continues
        # 3.3 km along the commanded route axis.
        "partPolygonXY": [
            (0.0, -500.0),
            (3300.0, -500.0),
            (3300.0, 500.0),
            (900.0, 500.0),
            (900.0, 100.0),
            (0.0, 100.0),
        ],
    }

    result = DivisionPlannerWindow._build_make_waypoint_row(fake_window, row)

    assert result is not None
    assert float(result["shapeLengthM"]) == pytest.approx(3300.0)
    assert result["waypointStartXY"] == pytest.approx((0.0, 0.0))
    assert result["waypointEndXY"] == pytest.approx((3300.0, 0.0))


def test_sequential_area_sweep_axis_uses_shared_bearing_not_piece_ingress() -> None:
    """Oblique T0 solutions must not rotate each 700 m piece's scan frame."""

    from modules.mission_planning.planners.next_collab_division._planner_window import (
        _area_sweep_route_axis_xy,
    )

    expected_bearing_deg = 345.9
    rows = [
        {
            "bearingDeg": expected_bearing_deg,
            "waypointStartXY": (0.0, 0.0),
            "waypointEndXY": (1000.0, 1000.0),
        },
        {
            "bearingDeg": expected_bearing_deg,
            "waypointStartXY": (0.0, 0.0),
            "waypointEndXY": (-1000.0, 100.0),
        },
        {
            "bearingDeg": expected_bearing_deg,
            "waypointStartXY": (0.0, 0.0),
            "waypointEndXY": (100.0, -1000.0),
        },
    ]

    resolved_bearings: list[float] = []
    for row in rows:
        ux, uy, source = _area_sweep_route_axis_xy(row)
        resolved_bearings.append(
            (math.degrees(math.atan2(ux, uy)) + 360.0) % 360.0
        )
        assert source == "planner_bearing"

    assert resolved_bearings == pytest.approx(
        [expected_bearing_deg] * len(rows),
        abs=1.0e-9,
    )


def test_make_sweep_keeps_sequential_piece_rows_parallel() -> None:
    """The no-split AREA workflow must materialize the shared planner axis."""

    from shapely.geometry import Polygon

    from modules.mission_planning.planners.next_collab_division._planner_window import (
        DivisionPlannerWindow,
    )

    path_rows = [
        {
            "source": "make_path_0",
            "aircraftID": 6,
            "pieceIndex": piece_index,
            "bearingDeg": 0.0,
            "waypointStartXY": start_xy,
            "waypointEndXY": end_xy,
            "partPolygonXY": [
                (-400.0, float((piece_index - 1) * 700)),
                (400.0, float((piece_index - 1) * 700)),
                (400.0, float(piece_index * 700)),
                (-400.0, float(piece_index * 700)),
            ],
            "sepCandM": 700.0,
            "dbSepM": 700.0,
            "resolvedFovDeg": 4.7,
            "resolvedDbFovDeg": 4.7,
        }
        for piece_index, (start_xy, end_xy) in enumerate(
            [
                ((-1500.0, -1000.0), (400.0, 700.0)),
                ((-1500.0, -1000.0), (-100.0, 1400.0)),
                ((-1500.0, -1000.0), (-700.0, 2100.0)),
            ],
            start=1,
        )
    ]
    overlays = [
        {
            "aircraftID": 6,
            "pieceIndex": int(row["pieceIndex"]),
            "bearingDeg": 0.0,
        }
        for row in path_rows
    ]
    fake_window = SimpleNamespace(
        state=SimpleNamespace(
            expected_paths=path_rows,
            mid_line_segments=overlays,
        ),
        _fov_db_rows=lambda: [],
        _next_collab_area_spacing_footprint_m=lambda *_a, **_k: 60.0,
        _path_row_piece_polygon_xy=lambda row: Polygon(row["partPolygonXY"]),
        _next_collab_area_sweep_spacing_m=lambda *_a, **_k: 100.0,
        _next_collab_takeover_first_step_ratio=lambda: 0.5,
        _next_collab_area_density_speed_scale=lambda: 1.25,
        _append_result=lambda *_a, **_k: None,
        _refresh_ui=lambda: None,
    )

    DivisionPlannerWindow._make_sweep(fake_window)

    for row in path_rows:
        assert row["areaSweepAxisSource"] == "planner_bearing"
        assert float(row["areaSweepAxisBearingDeg"]) == pytest.approx(0.0)
        assert row["sweepLineListXY"]
        for sweep_line in row["sweepLineListXY"]:
            dx = float(sweep_line[-1][0]) - float(sweep_line[0][0])
            dy = float(sweep_line[-1][1]) - float(sweep_line[0][1])
            strip_bearing = (
                math.degrees(math.atan2(dx, dy)) + 360.0
            ) % 180.0
            assert strip_bearing == pytest.approx(90.0, abs=1.0e-9)
        route_start = row["areaSweepRouteStartXY"]
        route_end = row["areaSweepRouteEndXY"]
        assert float(route_end[0]) == pytest.approx(float(route_start[0]))
        assert float(route_end[1]) > float(route_start[1])


def test_sequential_area_capture_anchors_ignore_oblique_ingress_axis() -> None:
    """Carrier WP direction follows the scan axis and reverses at the near end."""

    row = {
        # Deliberately unrelated ingress line: this used to become the capture
        # carrier and made later stages rotate back toward the old UAV position.
        "waypointStartXY": (-1000.0, -500.0),
        "waypointEndXY": (1000.0, 500.0),
        "areaSweepRouteStartXY": (0.0, 0.0),
        "areaSweepRouteEndXY": (0.0, 2000.0),
        "sweepLineListXY": [
            [(-500.0, 0.0), (500.0, 0.0)],
            [(500.0, 1000.0), (-500.0, 1000.0)],
            [(-500.0, 2000.0), (500.0, 2000.0)],
        ],
    }

    anchors = pipeline._area_path_row_capture_anchors_xy(row)
    assert anchors == pytest.approx(
        [(0.0, 0.0), (0.0, 1000.0), (0.0, 2000.0)],
        abs=1.0e-9,
    )

    exit_xy, exit_bearing_deg, execution_reversed = (
        pipeline._area_path_row_execution_state(row, (0.0, 1900.0))
    )
    assert execution_reversed is True
    assert exit_xy == pytest.approx((0.0, 0.0), abs=1.0e-9)
    assert exit_bearing_deg == pytest.approx(180.0, abs=1.0e-9)


def _single_owner_area_stage_order_fixture(
    *,
    owner_count: int = 1,
    missing_turn_field: str | None = None,
) -> tuple[list[dict], list[object]]:
    """Reproduce the four-stage UAV6 ordering from the 014155 scenario."""

    from modules.mission_planning.MissionPlanner.planning_enhanced.algo.split_runner import (
        SplitPiece,
    )

    # NextCollab_5 reported P1..P4 T0 approach times of
    # 62.5 / 60.5 / 49.4 / 45.7 seconds.  The physical pieces stay numbered
    # P1..P4, but a single owner should execute the adjacent chain P4..P1.
    approach_eta_s = (62.5, 60.5, 49.4, 45.7)
    rows: list[dict] = []
    pieces: list[object] = []
    for stage, eta_s in enumerate(approach_eta_s, start=1):
        piece_data = {
            "areaSequentialWidthSplit": True,
            "splitStage": int(stage),
            "splitCount": len(approach_eta_s),
            "areaSequentialOwnerSlot": 1 if int(owner_count) == 1 else 3,
            "areaSequentialOwnerCount": int(owner_count),
        }
        if int(owner_count) >= 2:
            piece_data.update(
                {
                    "areaOuterOwner": True,
                    "areaOuterSide": "max",
                    "areaOuterFirstSweep": bool(stage == 1),
                }
            )
        pieces.append(
            SplitPiece(
                parent_order=1,
                mission_id=9,
                mission_type=2,
                piece_index=int(stage),
                data=piece_data,
                assigned_uav=6,
            )
        )
        row = {
            "aircraftID": 6,
            "pieceIndex": int(stage),
            "originXY": (0.0, 0.0),
            "originHeadingDeg": 40.337,
            # Historical planner field name; its value is km/h.
            "resolvedVelMps": 144.0,
            "phaseRows": [
                {
                    "kind": "waypoint",
                    "startSec": float(eta_s),
                }
            ],
            "waypointStartXY": (float(stage * 1000), 0.0),
            "waypointEndXY": (float(stage * 1000), 1000.0),
            "routeXY": [
                (float(stage * 1000), 0.0),
                (float(stage * 1000), 1000.0),
            ],
        }
        if missing_turn_field is not None:
            row.pop(str(missing_turn_field), None)
        rows.append(row)
    return rows, pieces


def test_single_owner_area_executes_from_the_natural_outer_stage() -> None:
    """P4 is the shorter fixed-wing approach, so execute P4->P1."""

    rows, pieces = _single_owner_area_stage_order_fixture()

    assert pipeline._apply_width_split_sequence_metadata(rows, pieces) == 1

    rows_by_physical_stage = {
        int(row["splitStage"]): row
        for row in rows
    }
    assert sorted(rows_by_physical_stage) == [1, 2, 3, 4]
    assert [
        int(rows_by_physical_stage[stage]["areaSingleAircraftSequence"])
        for stage in range(1, 5)
    ] == [4, 3, 2, 1]
    assert all(
        row.get("areaSequentialExecutionOrderReversed") is True
        for row in rows
    )


def test_area_stage_approach_cost_uses_turn_speed_and_straight_ingress() -> None:
    row = {
        "originXY": (0.0, 0.0),
        "originHeadingDeg": 0.0,
        "resolvedVelMps": 144.0,
        "turnSpeedMps": 30.0,
        "horizonSec": 10.0,
        "tangentXY": (0.0, 0.0),
        "waypointStartXY": (100.0, 0.0),
    }

    assert pipeline._area_path_row_planned_turn_approach_cost_m(row) == pytest.approx(
        400.0
    )


@pytest.mark.parametrize(
    "missing_turn_field",
    [
        "originHeadingDeg",
        "resolvedVelMps",
    ],
)
def test_single_owner_area_keeps_canonical_order_without_turn_context(
    missing_turn_field: str,
) -> None:
    """Old/partial rows must retain their deterministic P1->PN order."""

    rows, pieces = _single_owner_area_stage_order_fixture(
        missing_turn_field=missing_turn_field,
    )

    assert pipeline._apply_width_split_sequence_metadata(rows, pieces) == 1

    assert [
        int(row["areaSingleAircraftSequence"])
        for row in sorted(rows, key=lambda item: int(item["splitStage"]))
    ] == [1, 2, 3, 4]
    assert not any(
        row.get("areaSequentialExecutionOrderReversed")
        for row in rows
    )


def test_single_owner_area_keeps_canonical_order_for_near_tied_approaches() -> None:
    """A sub-second numerical advantage must not flip the complete chain."""

    rows, pieces = _single_owner_area_stage_order_fixture()
    rows[0]["phaseRows"][0]["startSec"] = 45.9
    rows[-1]["phaseRows"][0]["startSec"] = 45.7

    assert pipeline._apply_width_split_sequence_metadata(rows, pieces) == 1

    assert [
        int(row["areaSingleAircraftSequence"])
        for row in sorted(rows, key=lambda item: int(item["splitStage"]))
    ] == [1, 2, 3, 4]


def test_single_owner_area_keeps_canonical_order_for_different_origins() -> None:
    """Costs from independently re-seeded stages are not comparable."""

    rows, pieces = _single_owner_area_stage_order_fixture()
    rows[-1]["originXY"] = (10.0, 0.0)

    assert pipeline._apply_width_split_sequence_metadata(rows, pieces) == 1

    assert [
        int(row["areaSingleAircraftSequence"])
        for row in sorted(rows, key=lambda item: int(item["splitStage"]))
    ] == [1, 2, 3, 4]


def test_multi_owner_outer_contract_is_not_reordered_by_single_owner_rule() -> None:
    """A max-edge owner's stage 1 remains its declared convex-hull edge."""

    rows, pieces = _single_owner_area_stage_order_fixture(owner_count=3)

    assert pipeline._apply_width_split_sequence_metadata(rows, pieces) == 1

    rows_by_physical_stage = {
        int(row["splitStage"]): row
        for row in rows
    }
    assert [
        int(rows_by_physical_stage[stage]["areaSingleAircraftSequence"])
        for stage in range(1, 5)
    ] == [1, 2, 3, 4]
    assert rows_by_physical_stage[1]["areaOuterSide"] == "max"
    assert rows_by_physical_stage[1]["areaOuterFirstSweep"] is True
    assert not any(
        bool(rows_by_physical_stage[stage]["areaOuterFirstSweep"])
        for stage in range(2, 5)
    )


def test_width_pair_stage_metadata_reaches_final_path_rows() -> None:
    """Each UAV's second area must inherit the first area's exit as entry."""

    from modules.mission_planning.MissionPlanner.planning_enhanced.algo.split_runner import (
        _assign_group_with_width_pairs,
    )
    from modules.mission_planning.planners.next_collab_division._geo_utils import (
        coord_to_xy,
    )

    pieces = _pieces_for(3000.0, 2000.0, 3)
    takeover = {aid: {"latitude": 37.99, "longitude": 127.0} for aid in (4, 5, 6)}
    assert _assign_group_with_width_pairs(pieces, [4, 5, 6], takeover) is True

    rows: list[dict] = []
    for piece in pieces:
        aircraft_id = int(piece.assigned_uav)
        sequence = int(piece.data["splitStage"])
        base_x = float(aircraft_id * 1000)
        rows.append(
            {
                "aircraftID": aircraft_id,
                "pieceIndex": int(piece.piece_index),
                "waypointStartXY": (base_x, float(sequence * 100)),
                "waypointEndXY": (base_x + 700.0, float(sequence * 100)),
            }
        )

    sequence_count = pipeline._apply_width_split_sequence_metadata(
        rows,
        pieces,
        {
            aircraft_id: {
                "latitude": LAT0,
                "longitude": LON0,
                "altitude": 1100.0,
            }
            for aircraft_id in (4, 5, 6)
        },
    )

    assert sequence_count == 3
    stage_count = int(pieces[0].data["splitCount"])
    for aircraft_id in (4, 5, 6):
        sequence_rows = sorted(
            (row for row in rows if int(row["aircraftID"]) == aircraft_id),
            key=lambda row: int(row["areaSingleAircraftSequence"]),
        )
        assert [
            int(row["areaSingleAircraftSequence"]) for row in sequence_rows
        ] == list(range(1, stage_count + 1))
        for sequence in range(1, stage_count):
            next_entry_xy = coord_to_xy(
                sequence_rows[sequence]["areaPassEntryCoordinate"]
            )
            assert next_entry_xy is not None
            assert (
                math.dist(
                    next_entry_xy,
                    sequence_rows[sequence - 1]["waypointEndXY"],
                )
                < 1.0
            )


def test_width_stage_handover_follows_the_actual_reversed_exit() -> None:
    """Stage 3 must begin where the reversed stage 2 really finishes."""

    from modules.mission_planning.MissionPlanner.planning_enhanced.algo.split_runner import (
        _assign_group_with_width_pairs,
    )
    from modules.mission_planning.planners.next_collab_division._geo_utils import (
        coord_to_xy,
    )

    pieces = _pieces_for(7200.0, 2000.0, 3)
    takeover = {aid: {"latitude": 37.99, "longitude": 127.0} for aid in (4, 5, 6)}
    assert _assign_group_with_width_pairs(pieces, [4, 5, 6], takeover) is True

    rows: list[dict] = []
    for piece in pieces:
        aircraft_id = int(piece.assigned_uav)
        sequence = int(piece.data["splitStage"])
        base_x = float(aircraft_id * 1000)
        rows.append(
            {
                "aircraftID": aircraft_id,
                "pieceIndex": int(piece.piece_index),
                "waypointStartXY": (base_x, float(sequence * 100)),
                "waypointEndXY": (base_x + 700.0, float(sequence * 100)),
            }
        )

    sequence_count = pipeline._apply_width_split_sequence_metadata(
        rows,
        pieces,
        {
            aircraft_id: {
                "latitude": LAT0,
                "longitude": LON0,
                "altitude": 1100.0,
            }
            for aircraft_id in (4, 5, 6)
        },
    )

    assert sequence_count == 3
    for aircraft_id in (4, 5, 6):
        sequence_rows = sorted(
            (row for row in rows if int(row["aircraftID"]) == aircraft_id),
            key=lambda row: int(row["areaSingleAircraftSequence"]),
        )
        assert len(sequence_rows) >= 3
        assert [
            bool(row["areaSingleAircraftExecutionReversed"])
            for row in sequence_rows[:3]
        ] == [False, True, False]
        second_entry_xy = coord_to_xy(
            sequence_rows[1]["areaPassEntryCoordinate"]
        )
        third_entry_xy = coord_to_xy(
            sequence_rows[2]["areaPassEntryCoordinate"]
        )
        assert second_entry_xy is not None
        assert third_entry_xy is not None
        assert second_entry_xy == pytest.approx(
            sequence_rows[0]["waypointEndXY"],
            abs=1.0,
        )
        assert third_entry_xy == pytest.approx(
            sequence_rows[1]["waypointStartXY"],
            abs=1.0,
        )


def test_width_stage_metadata_survives_the_0302_export_boundary() -> None:
    from modules.mission_planning.MissionPlanner.planning_enhanced.algo.split_runner import (
        SplitPiece,
    )
    from modules.mission_planning.MissionPlanner.planning_enhanced.io.export_0302 import (
        _piece_runtime_meta,
    )

    meta = _piece_runtime_meta(
        SplitPiece(
            parent_order=1,
            mission_id=9,
            mission_type=2,
            piece_index=1,
            data={
                "splitStage": 2,
                "splitCount": 5,
                "areaSequentialOwnerSlot": 1,
                "areaSequentialOwnerCount": 3,
                "areaSequentialWidthSplit": True,
                "areaSequentialWidthSpanM": 3295.0,
                "areaSequentialWidthTargetM": 700.0,
                "areaSequentialWidthLimitM": 800.0,
                "areaOuterOwner": True,
                "areaOuterSide": "min",
                "areaOuterFirstSweep": True,
            },
        )
    )

    assert meta == {
        "splitStage": 2,
        "splitCount": 5,
        "areaSequentialOwnerSlot": 1,
        "areaSequentialOwnerCount": 3,
        "areaSequentialWidthSplit": True,
        "areaSequentialWidthSpanM": 3295.0,
        "areaSequentialWidthTargetM": 700.0,
        "areaSequentialWidthLimitM": 800.0,
        "areaOuterOwner": True,
        "areaOuterSide": "min",
        "areaOuterFirstSweep": True,
    }


def test_outer_owner_metadata_reaches_next_collab_path_row() -> None:
    from modules.mission_planning.MissionPlanner.planning_enhanced.algo.split_runner import (
        SplitPiece,
    )

    piece = SplitPiece(
        parent_order=1,
        mission_id=9,
        mission_type=2,
        piece_index=7,
        data={
            "areaSequentialOwnerSlot": 3,
            "areaSequentialOwnerCount": 3,
            "areaOuterOwner": True,
            "areaOuterSide": "max",
            "areaOuterFirstSweep": True,
        },
        assigned_uav=6,
    )
    rows = [
        {
            "aircraftID": 6,
            "pieceIndex": 7,
            "waypointStartXY": (0.0, 0.0),
            "waypointEndXY": (1000.0, 0.0),
        }
    ]

    sequence_count = pipeline._apply_width_split_sequence_metadata(
        rows,
        [piece],
    )

    assert sequence_count == 0
    assert rows[0]["areaSequentialOwnerSlot"] == 3
    assert rows[0]["areaSequentialOwnerCount"] == 3
    assert rows[0]["areaOuterOwner"] is True
    assert rows[0]["areaOuterSide"] == "max"
    assert rows[0]["areaOuterFirstSweep"] is True


def test_initial_area_outer_endpoint_keeps_rows_and_reverses_every_sweep() -> None:
    from modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304 import (
        d0303,
    )

    rows = [
        {"coords": _xy_coords([(0.0, 0.0), (100.0, 0.0)])},
        {"coords": _xy_coords([(100.0, 100.0), (0.0, 100.0)])},
    ]

    oriented = d0303._orient_area_sweep_endpoints_from_outer_side(
        rows,
        outer_side="max",
        sweep_bearing_deg=90.0,
    )

    first_row_xy = [
        llh_to_xy(
            float(coord["latitude"]),
            float(coord["longitude"]),
            LAT0,
            LON0,
        )
        for coord in oriented[0]["coords"]
    ]
    second_row_xy = [
        llh_to_xy(
            float(coord["latitude"]),
            float(coord["longitude"]),
            LAT0,
            LON0,
        )
        for coord in oriented[1]["coords"]
    ]
    assert first_row_xy[0][0] > first_row_xy[-1][0]
    assert second_row_xy[0][0] < second_row_xy[-1][0]


def test_sequential_handover_uses_capture_exit_not_far_route_offset() -> None:
    row = {
        "waypointStartXY": (0.0, 0.0),
        "waypointEndXY": (5000.0, 0.0),
        "sweepLineListXY": [
            [(100.0, -300.0), (100.0, 300.0)],
            [(700.0, -300.0), (700.0, 300.0)],
        ],
    }

    capture_exit = pipeline._area_path_row_capture_exit_xy(row)

    assert capture_exit is not None
    assert math.dist(capture_exit, (700.0, 0.0)) < 1.0
    assert math.dist(capture_exit, row["waypointEndXY"]) > 4000.0


def test_initial_area_stages_alternate_from_the_actual_capture_exit(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Adjacent strips fly out/back/out instead of restarting in one direction."""

    from modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304 import (
        d0303,
    )

    missions: list[dict] = []
    for stage, (x_start, x_end) in enumerate(
        [(0.0, 700.0), (660.0, 1360.0), (1320.0, 2020.0)],
        start=1,
    ):
        missions.append(
            {
                "aircraftID": 4,
                "pathID": 500000000 + stage,
                "individualMissionID": 900000000 + stage,
                "relatedMission": {"inputMissionID": 5},
                "splitStage": stage,
                "splitCount": 3,
                "individualMissionInfo": {
                    "individualMissionType": 4,
                    "patternType": 6,
                    "autoZoomIn": True,
                    "BEARING": 90.0,
                    "MOVE_BEARING": 0.0,
                    "SPEED": 144.0,
                    "FOV": 4.92,
                    "SEP": 1000.0,
                    "areaList": [
                        {
                            "isHole": False,
                            "coordinateList": _xy_coords(
                                [
                                    (x_start, 0.0),
                                    (x_end, 0.0),
                                    (x_end, 1500.0),
                                    (x_start, 1500.0),
                                ]
                            ),
                        }
                    ],
                    "coordinateList": [],
                },
            }
        )

    monkeypatch.setattr(d0303, "_dem_alt", lambda _lat, _lon: 100.0)
    with runtime_override(
        {
            "values": {
                "enhanced_auto_fov_from_db": False,
                "physics_fov_selection_enabled": False,
            }
        }
    ):
        paths = d0303.build_flight_plans(missions, cruise_speed=40.0)

    latitude_directions: list[int] = []
    for path in paths:
        filming_coords: list[dict] = []
        for waypoint in path.get("waypointList") or []:
            line_search = (
                (waypoint.get("filmingProperty") or {}).get("lineSearch") or {}
            )
            filming_coords.extend(line_search.get("coordinateList") or [])
        assert len(filming_coords) >= 2
        latitude_directions.append(
            1
            if float(filming_coords[-1]["latitude"])
            > float(filming_coords[0]["latitude"])
            else -1
        )

    assert latitude_directions in ([-1, 1, -1], [1, -1, 1])


def test_pair_units_bind_spatially_adjacent_stage_pieces() -> None:
    """같은 기체의 stage1·stage2 는 경계를 공유하는 인접 스트립이어야 한다."""

    from shapely.geometry import Polygon

    from modules.mission_planning.MissionPlanner.planning_enhanced.algo.split_runner import (
        _assign_group_with_width_pairs,
    )

    group = _pieces_for(3000.0, 2000.0, 3)
    clustered = {a: {"latitude": 37.99, "longitude": 127.0} for a in (4, 5, 6)}
    _assign_group_with_width_pairs(group, [4, 5, 6], clustered)

    def _poly(piece) -> Polygon:
        coords = piece.data.get("coordinateList") or []
        return Polygon([(c["longitude"], c["latitude"]) for c in coords]).buffer(1e-9)

    stage_two = [p for p in group if int(p.data.get("splitStage")) == 2]
    for piece in group:
        if int(piece.data.get("splitStage")) != 1:
            continue
        partner = next(p for p in stage_two if p.assigned_uav == piece.assigned_uav)
        assert _poly(piece).intersection(_poly(partner)).area > 0.0


def test_groups_without_width_pairs_keep_the_existing_assignment_path() -> None:
    from modules.mission_planning.MissionPlanner.planning_enhanced.algo.split_runner import (
        _assign_group_with_width_pairs,
    )

    group = _pieces_for(500.0, 2000.0, 1)  # 좁음 → 분할 없음 → 쌍 없음
    assert (
        _assign_group_with_width_pairs(
            group, [4], {4: {"latitude": 37.99, "longitude": 127.0}}
        )
        is False
    )


# ------------------------------------------------------- 다음협업기저임무


def _fake_row(aircraft_id: int, piece_index: int, width_m: float) -> dict:
    half = width_m / 2.0
    return {
        "aircraftID": aircraft_id,
        "pieceIndex": piece_index,
        "waypointStartXY": (0.0, 0.0),
        "waypointEndXY": (100.0, 0.0),
        "routeXY": [(0.0, 0.0), (100.0, 0.0)],
        "bearingDeg": 0.0,
        "partPolygonXY": [
            (-half, 0.0),
            (half, 0.0),
            (half, 2000.0),
            (-half, 2000.0),
        ],
    }


def _run_prepare(
    monkeypatch: pytest.MonkeyPatch,
    part_width_m: float,
    *,
    owner_count: int = 1,
) -> list[dict]:
    planner_calls: list[dict] = []

    def _fake_planner(**kwargs: object) -> SimpleNamespace:
        planner_calls.append(deepcopy(kwargs))
        entries = list(kwargs["aircraft_entries"])
        per_piece_width = part_width_m / max(1, len(entries))
        rows = [
            _fake_row(int(entry["aircraftID"]), index + 1, per_piece_width)
            for index, entry in enumerate(entries)
        ]
        return SimpleNamespace(
            expected_paths=rows,
            split_result=SimpleNamespace(pieces=[]),
            mid_line_segments=[],
            workflow="width-split-test",
            planner_result_text="",
        )

    monkeypatch.setattr(pipeline, "run_next_collab_division_plan", _fake_planner)
    monkeypatch.setattr(
        pipeline, "_branch_area_ownership_for_target", lambda *a, **k: None
    )
    monkeypatch.setattr(
        pipeline,
        "_prewarm_dem_altitudes_for_path_rows_if_enabled",
        lambda *a, **k: {"uniquePairs": 0},
    )
    monkeypatch.setattr(
        pipeline, "_reserve_next_collab_replacement_ids", lambda **k: None
    )

    owner_ids = list(range(6, 6 + int(owner_count)))
    result = pipeline._prepare_area_replacements(
        target_input_mission={
            "inputMissionID": 9,
            "inputMissionType": 2,
            "missionDetail": {
                "areaList": [
                    {"isHole": False, "coordinateList": _rect_coords(3000.0, 2000.0)}
                ]
            },
        },
        target_input_id=9,
        target_aircraft_ids=owner_ids,
        entry_coord_map={
            aircraft_id: {
                "latitude": 37.995,
                "longitude": 127.0 + index * 0.0001,
                "altitude": 100,
            }
            for index, aircraft_id in enumerate(owner_ids)
        },
        heading_map={aircraft_id: 90.0 for aircraft_id in owner_ids},
        entry_aircraft_context_map=None,
        representative_entry=None,
        template_record_map={},
        now_ms=1,
        turn_radius_scale=1.0,
        emit=lambda _m: None,
        split_single_aircraft_into_two=True,
    )
    assert result is None  # ID 예약 경계에서 의도적으로 중단
    return planner_calls


def test_next_collab_wide_assignment_triggers_the_sequential_replan(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls = _run_prepare(monkeypatch, part_width_m=1600.0)
    assert len(calls) >= 2
    # 700 m는 목표값이고 허용 상한은 900 m이므로 1600 m는 2단계다.
    assert len(list(calls[0]["aircraft_entries"])) == 1
    assert len(list(calls[1]["aircraft_entries"])) == 2
    real_ids = {int(r["aircraftID"]) for r in calls[0]["aircraft_entries"]}
    rerun_ids = {int(r["aircraftID"]) for r in calls[1]["aircraft_entries"]}
    assert real_ids < rerun_ids  # 실기체 + 가상 파트너


@pytest.mark.parametrize(
    ("owner_count", "total_width_m", "expected_stages_per_owner"),
    [
        (1, 1600.0, 2),
        (2, 3200.0, 2),
        (3, 4500.0, 2),
        (1, 2400.0, 3),
        (2, 4800.0, 3),
        (3, 7200.0, 3),
    ],
)
def test_attack_area_replan_scales_width_stages_for_every_remaining_owner(
    monkeypatch: pytest.MonkeyPatch,
    owner_count: int,
    total_width_m: float,
    expected_stages_per_owner: int,
) -> None:
    calls = _run_prepare(
        monkeypatch,
        part_width_m=total_width_m,
        owner_count=owner_count,
    )

    assert len(list(calls[0]["aircraft_entries"])) == owner_count
    assert len(list(calls[1]["aircraft_entries"])) == (
        owner_count * expected_stages_per_owner
    )


def test_attack_replan_narrow_assignment_still_keeps_two_stages_per_owner(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls = _run_prepare(monkeypatch, part_width_m=600.0)
    assert len(list(calls[0]["aircraft_entries"])) == 1
    # 폭이 임계 이하면 가상 파트너 재분할이 없어야 한다
    # (2번째 호출이 있다면 그건 second-entry 재계획이며 엔트리 수가 같다).
    assert len(list(calls[1]["aircraft_entries"])) == 2


def test_attack_minimum_two_stages_remains_when_width_rule_is_disabled(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    with runtime_override({"values": {"area_sequential_split_width_m": 0.0}}):
        calls = _run_prepare(monkeypatch, part_width_m=1600.0)
    assert len(list(calls[1]["aircraft_entries"])) == 2
