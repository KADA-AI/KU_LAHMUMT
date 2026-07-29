# -*- coding: utf-8 -*-
"""다음협업 AREA 진입점 앵커 방향 계약.

한 기체가 연속 두 조각(순차 폭분할, 단일기체 2분할)을 받으면 두 번째 조각은
첫 조각이 끝난 쪽에서 이어받아 반대로 "내려오는" 서펜타인이 되어야 한다.
플래너의 정방향(canonical outbound) 행 순서를 그대로 렌더링하면 조각마다 같은
방향으로 올라가는 경로가 나온다(회귀).  진입이 경로 끝쪽이면 행 순서·행 방향·
경로 축을 통째로 뒤집는다.  명시적 패스 계약(out/turn/return)은 자체 방향
기계가 있으므로 건드리지 않는다.
"""

from __future__ import annotations

import math
from copy import deepcopy

import pytest

from modules.mission_planning.pipelines import next_collab_path_builder as builder


def _coord(x_m: float, y_m: float, altitude_m: float = 1000.0) -> dict:
    coord = builder._xy_to_coord((float(x_m), float(y_m)))
    coord["altitude"] = float(altitude_m)
    return coord


def _configure_area_builder(monkeypatch: pytest.MonkeyPatch) -> None:
    runtime_floats = {
        "uav_wp_interval_m": 420.0,
        "area_first_packet_sweep_group_scale": 1.5,
        "next_collab_area_terrain_route_spacing_min_m": 500.0,
        "next_collab_area_terrain_group_relief_limit_m": 100.0,
        "next_collab_area_terrain_group_heading_limit_deg": 8.0,
        "next_collab_capture_completion_speed_scale": 1.10,
        "next_collab_first_capture_activation_delay_s": 0.0,
        "next_collab_area_first_capture_stale_entry_guard_s": 0.0,
        "uav_climb_rate_mps": 5.0,
        "uav_descent_rate_mps": 7.0,
        "uav_min_forward_speed_mps": 30.0,
    }
    monkeypatch.setattr(
        builder,
        "get_runtime_float",
        lambda key, default: float(runtime_floats.get(key, default)),
    )
    monkeypatch.setattr(
        builder,
        "get_runtime_bool",
        lambda key, default: bool(
            {
                "next_collab_auto_sweep_points": False,
            }.get(key, default)
        ),
    )
    monkeypatch.setattr(builder, "_runtime_manual_fov_active", lambda: False)
    monkeypatch.setattr(
        builder,
        "_runtime_manual_fov_value",
        lambda _key, default: float(default),
    )
    monkeypatch.setattr(builder, "_dem_alt", lambda _lat, _lon: 100.0)
    monkeypatch.setattr(
        builder,
        "_dem_altitudes_for_pairs_cached",
        lambda pairs, invalid_default=0.0: [100.0 for _ in pairs],
    )
    monkeypatch.setattr(
        builder,
        "_mission_altitude_policy",
        lambda **_kwargs: (1000.0, 100.0, lambda _lat, _lon: 1100),
    )
    monkeypatch.setattr(
        builder,
        "_runtime_flyover_options",
        lambda: {
            "entry_offset": False,
            "dubins_prefix": False,
            "last_point": False,
            "all_wps": False,
        },
    )


def _area_path_row() -> dict:
    scan_lines = [
        [(float(x), -320.0), (float(x), 0.0), (float(x), 320.0)]
        for x in (100.0, 450.0, 800.0, 1150.0, 1500.0, 1850.0, 2200.0, 2550.0)
    ]
    return {
        "aircraftID": 5,
        "pieceIndex": 1,
        "originXY": (-1800.0, 0.0),
        "originHeadingDeg": 90.0,
        "waypointStartXY": (0.0, 0.0),
        "waypointEndXY": (2700.0, 0.0),
        "areaCenterLineXY": [(0.0, 0.0), (2700.0, 0.0)],
        "areaMissionStartXY": (0.0, 0.0),
        "areaMissionEndXY": (2700.0, 0.0),
        "partPolygonXY": [
            (0.0, -350.0),
            (2700.0, -350.0),
            (2700.0, 350.0),
            (0.0, 350.0),
        ],
        "sweepLineListXY": scan_lines,
        "lineSweepSpacingM": 350.0,
        "resolvedVelMps": 165.2976,
        "resolvedFovDeg": 10.0,
        "turnRadiusM": 450.0,
    }


def _build_with_entry(
    monkeypatch: pytest.MonkeyPatch,
    entry_x_m: float,
    *,
    entry_y_m: float = 0.0,
    path_row: dict | None = None,
    runtime_entry_flyover: bool = False,
) -> tuple[dict, dict]:
    _configure_area_builder(monkeypatch)
    if runtime_entry_flyover:
        monkeypatch.setattr(
            builder,
            "_runtime_flyover_options",
            lambda: {
                "entry_offset": True,
                "dubins_prefix": False,
                "last_point": False,
                "all_wps": False,
            },
        )
    entry = _coord(entry_x_m, entry_y_m, 1100.0)
    template_path = {
        "waypointList": [
            {
                "coordinate": deepcopy(entry),
                "speed": 30.0,
                "filmingProperty": {
                    "fieldOfView": 10.0,
                    "sensorType": 1,
                    "operationMode": 1,
                },
            }
        ]
    }
    mission_info = {
        "individualMissionType": 3,
        "patternType": 1,
        "FOV": 10.0,
        "areaList": [
            {
                "coordinateList": [
                    _coord(-100.0, -400.0),
                    _coord(2800.0, -400.0),
                    _coord(2800.0, 400.0),
                    _coord(-100.0, 400.0),
                ]
            }
        ],
    }
    metrics: dict = {}
    path = builder.build_flight_path_from_planned_row(
        path_row if isinstance(path_row, dict) else _area_path_row(),
        template_path=template_path,
        mission_info=mission_info,
        individual_mission_id=900000274,
        path_id=500000043,
        aircraft_id=5,
        entry_coord=entry,
        timestamp_ms=1,
        assign_waypoint_ids=False,
        metrics_callback=lambda values: metrics.update(values),
    )
    return path, metrics


def _scan_row_xs(path: dict) -> list[float]:
    """모든 lineSearch 촬영점의 x — 행이 한 WP로 묶여도 소비 순서가 남는다."""

    xs: list[float] = []
    for waypoint in path.get("waypointList") or []:
        line_search = (waypoint.get("filmingProperty") or {}).get("lineSearch") or {}
        for coord in line_search.get("coordinateList") or []:
            xy = builder.coord_to_xy(coord)
            if xy is not None:
                xs.append(float(xy[0]))
    return xs


def test_entry_at_route_start_keeps_the_forward_serpentine(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    path, metrics = _build_with_entry(monkeypatch, -1800.0)
    xs = _scan_row_xs(path)
    assert len(xs) >= 2
    assert xs[0] < xs[-1]  # 행 순서가 진입쪽(시작)에서 멀어지는 정방향
    assert not metrics.get("areaEntryAnchoredReverse")


def test_entry_at_route_end_reverses_the_serpentine(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """끝점에서 이어받는 조각은 내려오는 순서로 소비해야 한다."""

    path, metrics = _build_with_entry(monkeypatch, 4500.0)
    xs = _scan_row_xs(path)
    assert len(xs) >= 2
    assert xs[0] > xs[-1]  # 마지막 행부터 소비 — "내려오는" 서펜타인
    assert metrics.get("areaEntryAnchoredReverse") is True


def test_outer_owner_starts_first_scan_at_convex_hull_edge(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Exterior preference flips endpoints without changing route-row order."""

    row = _area_path_row()
    row["bearingDeg"] = 90.0
    row["areaOuterOwner"] = True
    row["areaOuterSide"] = "min"
    row["areaOuterFirstSweep"] = True

    path, metrics = _build_with_entry(
        monkeypatch,
        -1800.0,
        path_row=row,
    )

    xs = _scan_row_xs(path)
    assert xs[0] < xs[-1]
    first_line_search = next(
        (
            (waypoint.get("filmingProperty") or {}).get("lineSearch") or {}
            for waypoint in path.get("waypointList") or []
            if ((waypoint.get("filmingProperty") or {}).get("lineSearch") or {}).get(
                "coordinateList"
            )
        ),
        {},
    )
    coords = list(first_line_search.get("coordinateList") or [])
    interpolation_points = int(
        first_line_search.get("interpolationPoints") or 3
    )
    first_sweep = coords[:interpolation_points]
    first_xy = [builder.coord_to_xy(coord) for coord in first_sweep]
    assert len(first_xy) >= 2
    assert all(point is not None for point in first_xy)

    # bearing=90 deg -> split normal=(0,-1); "min" is the +Y hull edge.
    start_projection = -float(first_xy[0][1])
    end_projection = -float(first_xy[-1][1])
    assert start_projection < end_projection
    assert metrics.get("areaOuterFirstSweep") is True
    assert metrics.get("areaOuterEndpointReversed") is True


def test_sequential_pair_flies_out_then_back_like_the_initial_plan(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """순차 두 갈래 = 초기계획과 같은 왕복.

    초기계획은 다음 임무의 스윕 앵커가 "이전 임무 마지막 점"이라 두 번째
    갈래가 자동으로 끝점에서 내려온다.  다음협업도 같은 체인이어야 한다:
    collapse 가 두 번째 행에 넘긴 진입(=첫 행의 끝) → 빌더가 그 끝에서부터
    반대 방향으로 소비.
    """

    from types import SimpleNamespace

    from modules.mission_planning.replanning.triggers.next_collab import (
        pipeline as ncp,
    )

    _configure_area_builder(monkeypatch)

    # 첫 갈래: x 0~2700 을 올라가고 (2700, 0) 에서 끝난다.
    first_row = _area_path_row()
    first_row["aircraftID"] = 5
    # 두 번째 갈래: 인접 스트립, 플래너는 똑같이 정방향(올라가는) 순서로 준다.
    second_row = _area_path_row()
    second_row["aircraftID"] = 905  # 가상 파트너

    collapse_ok = ncp._collapse_single_aircraft_sequential_area_result(
        SimpleNamespace(
            expected_paths=[first_row, second_row],
            split_result=SimpleNamespace(pieces=[]),
        ),
        {
            "realAircraftID": 5,
            "virtualAircraftID": 905,
            "realEntryXY": (-1800.0, 0.0),
            "realEntryCoordinate": {"altitude": 1100.0},
        },
    )
    assert collapse_ok is True
    assert int(second_row["areaSingleAircraftSequence"]) == 2
    handover_entry = second_row["areaSingleAircraftEntryCoordinate"]
    handover_xy = builder.coord_to_xy(handover_entry)
    assert handover_xy is not None
    # collapse 가 넘긴 진입 = 첫 갈래의 끝점 (2700, 0).
    assert abs(float(handover_xy[0]) - 2700.0) < 1.0
    assert abs(float(handover_xy[1]) - 0.0) < 1.0

    # 첫 갈래는 실기체 진입에서 올라간다.
    first_path, first_metrics = _build_with_entry(monkeypatch, -1800.0)
    first_xs = _scan_row_xs(first_path)
    assert first_xs[0] < first_xs[-1]
    assert not first_metrics.get("areaEntryAnchoredReverse")

    # 두 번째 갈래는 collapse 진입(끝점)에서 내려온다 — 파이프라인과 동일하게
    # areaPassEntryCoordinate 가 entry_coord 로 들어간다.
    metrics: dict = {}
    second_path = builder.build_flight_path_from_planned_row(
        second_row,
        template_path={
            "waypointList": [
                {
                    "coordinate": deepcopy(handover_entry),
                    "speed": 30.0,
                    "filmingProperty": {
                        "fieldOfView": 10.0,
                        "sensorType": 1,
                        "operationMode": 1,
                    },
                }
            ]
        },
        mission_info={
            "individualMissionType": 3,
            "patternType": 1,
            "FOV": 10.0,
            "areaList": [
                {
                    "coordinateList": [
                        _coord(-100.0, -400.0),
                        _coord(2800.0, -400.0),
                        _coord(2800.0, 400.0),
                        _coord(-100.0, 400.0),
                    ]
                }
            ],
        },
        individual_mission_id=900000276,
        path_id=500000045,
        aircraft_id=5,
        entry_coord=handover_entry,
        timestamp_ms=1,
        assign_waypoint_ids=False,
        metrics_callback=lambda values: metrics.update(values),
    )
    second_xs = _scan_row_xs(second_path)
    assert second_xs[0] > second_xs[-1]  # 끝점에서 이어받아 내려온다
    assert metrics.get("areaEntryAnchoredReverse") is True
    second_waypoints = second_path.get("waypointList") or []
    assert len(second_waypoints) >= 2
    entry_waypoint = second_waypoints[0]
    entry_filming = entry_waypoint.get("filmingProperty") or {}
    first_capture_filming = second_waypoints[1].get("filmingProperty") or {}
    first_capture_coords = (
        (first_capture_filming.get("lineSearch") or {}).get("coordinateList") or []
    )
    assert int(entry_filming.get("operationMode") or 0) == 1
    assert not (entry_filming.get("lineSearch") or {}).get("coordinateList")
    assert first_capture_coords
    assert (
        (entry_filming.get("coordinateOrientation") or {}).get("coordinate")
        == first_capture_coords[0]
    )
    entry_xy = builder.coord_to_xy(entry_waypoint.get("coordinate"))
    handover_xy = builder.coord_to_xy(handover_entry)
    assert entry_xy is not None
    assert handover_xy is not None
    assert math.dist(entry_xy, handover_xy) > 10.0
    assert all(
        ((waypoint.get("filmingProperty") or {}).get("lineSearch") or {}).get(
            "coordinateList"
        )
        for waypoint in second_waypoints[1:]
    )
    assert metrics.get("areaSequentialCaptureOnly") is False
    assert metrics.get("areaSequentialEntryWaypointAdded") is True
    assert metrics.get("areaSequentialFirstCaptureReusedAsEntry") is False
    assert metrics.get("areaSequentialIngressHelpersEmitted") is False


def test_later_sequential_area_uses_one_aligned_flyby_entry(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """WP1 is the first capture-axis anchor, with no route-start out-leg."""

    row = _area_path_row()
    row["areaSingleAircraftSequentialSplit"] = True
    row["areaSingleAircraftSequence"] = 2
    row["areaSingleAircraftExecutionReversed"] = True

    path, metrics = _build_with_entry(
        monkeypatch,
        2700.0,
        entry_y_m=600.0,
        path_row=row,
        runtime_entry_flyover=True,
    )

    waypoints = path.get("waypointList") or []
    assert len(waypoints) >= 2
    entry_xy = builder.coord_to_xy(waypoints[0].get("coordinate"))
    first_capture_xy = builder.coord_to_xy(waypoints[1].get("coordinate"))
    assert entry_xy is not None
    assert first_capture_xy is not None
    # Neither the hand-over point nor the route-start helper is public.
    assert math.dist(entry_xy, (2700.0, 0.0)) > 10.0
    assert math.dist(entry_xy, (2700.0, 600.0)) > 500.0
    # WP1 and WP2 lie on the same reversed capture axis.
    assert abs(float(entry_xy[1]) - float(first_capture_xy[1])) < 1.0
    assert float(entry_xy[0]) > float(first_capture_xy[0])
    assert int(waypoints[0].get("waypointPassType") or 0) == builder.PASS_FLYBY
    assert not (
        ((waypoints[0].get("filmingProperty") or {}).get("lineSearch") or {}).get(
            "coordinateList"
        )
    )
    assert all(
        ((waypoint.get("filmingProperty") or {}).get("lineSearch") or {}).get(
            "coordinateList"
        )
        for waypoint in waypoints[1:]
    )
    assert metrics.get("areaSequentialEntryWaypointSource") == (
        "first_sweep_axis_anchor"
    )
    assert metrics.get("areaSequentialFirstCaptureReusedAsEntry") is False
    assert metrics.get("areaSequentialEntryTurnFlybyLocked") is True
    assert len(waypoints) == 2
    assert builder._distance_xy(entry_xy, first_capture_xy) > 2500.0
    for index in range(1, len(waypoints)):
        capture = waypoints[index]
        previous_xy = builder.coord_to_xy(waypoints[index - 1].get("coordinate"))
        capture_xy = builder.coord_to_xy(capture.get("coordinate"))
        line_search = (
            (capture.get("filmingProperty") or {}).get("lineSearch") or {}
        )
        sweep_xy = [
            builder.coord_to_xy(coord)
            for coord in line_search.get("coordinateList") or []
        ]
        assert previous_xy is not None and capture_xy is not None
        assert all(point is not None for point in sweep_xy)
        flight_time_s = (
            builder._distance_xy(previous_xy, capture_xy)
            / float(capture["speed"])
        )
        scan_time_s = (
            builder._line_length_xy(sweep_xy)
            / float(line_search["searchSpeed"])
        )
        assert scan_time_s / flight_time_s == pytest.approx(
            1.0 / 1.10,
            rel=1e-3,
        )
    assert metrics.get("areaSequentialCaptureSpeedsResynced") == len(
        waypoints
    ) - 1


def test_short_sequential_area_keeps_one_natural_capture_group() -> None:
    items = [
        {"progressM": 100.0, "sweepIndex": 0},
        {"progressM": 350.0, "sweepIndex": 1},
        {"progressM": 700.0, "sweepIndex": 2},
    ]

    groups = builder._group_area_sweep_items_by_spacing(
        items,
        spacing_m=1500.0,
        anchor_to_first_item=True,
    )

    assert groups == [items]


def test_short_sequential_area_uses_its_full_axis_for_the_capture_leg(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    row = _area_path_row()
    route_end_m = 400.0
    row["waypointEndXY"] = (route_end_m, 0.0)
    row["areaCenterLineXY"] = [(0.0, 0.0), (route_end_m, 0.0)]
    row["areaMissionEndXY"] = (route_end_m, 0.0)
    row["partPolygonXY"] = [
        (0.0, -1550.0),
        (route_end_m, -1550.0),
        (route_end_m, 1550.0),
        (0.0, 1550.0),
    ]
    row["sweepLineListXY"] = [
        [(float(x), -1500.0), (float(x), 0.0), (float(x), 1500.0)]
        for x in (0.0, 100.0, 200.0, 300.0, 400.0)
    ]
    row["lineSweepSpacingM"] = 100.0
    row["areaSingleAircraftSequentialSplit"] = True
    row["areaSingleAircraftSequence"] = 2

    path, metrics = _build_with_entry(
        monkeypatch,
        -500.0,
        path_row=row,
    )

    waypoints = path.get("waypointList") or []
    assert len(waypoints) == 2
    start_xy = builder.coord_to_xy(waypoints[0].get("coordinate"))
    capture_xy = builder.coord_to_xy(waypoints[1].get("coordinate"))
    assert start_xy is not None and capture_xy is not None
    gap_m = builder._distance_xy(start_xy, capture_xy)
    assert gap_m == pytest.approx(route_end_m, abs=1.0)
    line_search = (
        (waypoints[1].get("filmingProperty") or {}).get("lineSearch") or {}
    )
    sweep_xy = [
        builder.coord_to_xy(coord)
        for coord in line_search.get("coordinateList") or []
    ]
    assert all(point is not None for point in sweep_xy)
    flight_time_s = gap_m / float(waypoints[1]["speed"])
    scan_time_s = (
        builder._line_length_xy(sweep_xy)
        / float(line_search["searchSpeed"])
    )
    assert scan_time_s / flight_time_s == pytest.approx(
        1.0 / 1.10,
        rel=1e-3,
    )
    assert metrics.get("areaGroups") == 1
    assert metrics.get("areaSequentialCaptureSpeedsResynced") == 1


def test_long_sequential_area_keeps_distance_groups_and_syncs_every_scan(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    row = _area_path_row()
    route_end_m = 8000.0
    row["waypointEndXY"] = (route_end_m, 0.0)
    row["areaCenterLineXY"] = [(0.0, 0.0), (route_end_m, 0.0)]
    row["areaMissionEndXY"] = (route_end_m, 0.0)
    row["partPolygonXY"] = [
        (0.0, -350.0),
        (route_end_m, -350.0),
        (route_end_m, 350.0),
        (0.0, 350.0),
    ]
    row["sweepLineListXY"] = [
        [(float(x), -320.0), (float(x), 0.0), (float(x), 320.0)]
        for x in range(100, 8000, 350)
    ]
    row["areaSingleAircraftSequentialSplit"] = True
    row["areaSingleAircraftSequence"] = 2

    path, metrics = _build_with_entry(
        monkeypatch,
        -1800.0,
        path_row=row,
    )

    waypoints = path.get("waypointList") or []
    assert len(waypoints) > 2
    gaps_m: list[float] = []
    for index in range(1, len(waypoints)):
        previous_xy = builder.coord_to_xy(
            waypoints[index - 1].get("coordinate")
        )
        capture = waypoints[index]
        capture_xy = builder.coord_to_xy(capture.get("coordinate"))
        line_search = (
            (capture.get("filmingProperty") or {}).get("lineSearch") or {}
        )
        sweep_xy = [
            builder.coord_to_xy(coord)
            for coord in line_search.get("coordinateList") or []
        ]
        assert previous_xy is not None and capture_xy is not None
        assert all(point is not None for point in sweep_xy)
        gap_m = builder._distance_xy(previous_xy, capture_xy)
        gaps_m.append(gap_m)
        flight_time_s = gap_m / float(capture["speed"])
        scan_time_s = (
            builder._line_length_xy(sweep_xy)
            / float(line_search["searchSpeed"])
        )
        assert scan_time_s / flight_time_s == pytest.approx(
            1.0 / 1.10,
            rel=1e-3,
        )

    # Configured spacing is 1000 m * 1.5. The first public point is now the
    # first real sweep anchor, so no artificial half-spacing leg remains.
    assert min(gaps_m) >= 1200.0
    assert metrics.get("areaSequentialCaptureSpeedsResynced") == len(
        waypoints
    ) - 1


def test_explicit_pass_contract_rows_are_not_touched(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """out/turn/return 패스 계약은 자체 방향 기계를 그대로 쓴다."""

    _configure_area_builder(monkeypatch)
    row = _area_path_row()
    row["areaAssignedCoveragePass"] = "forward"
    entry = _coord(4500.0, 0.0, 1100.0)
    metrics: dict = {}
    path = builder.build_flight_path_from_planned_row(
        row,
        template_path={
            "waypointList": [
                {
                    "coordinate": deepcopy(entry),
                    "speed": 30.0,
                    "filmingProperty": {
                        "fieldOfView": 10.0,
                        "sensorType": 1,
                        "operationMode": 1,
                    },
                }
            ]
        },
        mission_info={
            "individualMissionType": 3,
            "patternType": 1,
            "FOV": 10.0,
            "areaList": [
                {
                    "coordinateList": [
                        _coord(-100.0, -400.0),
                        _coord(2800.0, -400.0),
                        _coord(2800.0, 400.0),
                        _coord(-100.0, 400.0),
                    ]
                }
            ],
        },
        individual_mission_id=900000275,
        path_id=500000044,
        aircraft_id=5,
        entry_coord=entry,
        timestamp_ms=1,
        assign_waypoint_ids=False,
        metrics_callback=lambda values: metrics.update(values),
    )
    assert not metrics.get("areaEntryAnchoredReverse")
    xs = _scan_row_xs(path)
    assert len(xs) >= 2
    assert xs[0] < xs[-1]  # 정방향 유지
