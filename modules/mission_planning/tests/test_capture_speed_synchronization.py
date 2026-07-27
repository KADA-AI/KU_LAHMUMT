from __future__ import annotations

from copy import deepcopy

import pytest

from modules.mission_planning.pipelines import line_search_speed_guard
from modules.mission_planning.pipelines import mission_path_trim
from modules.mission_planning.pipelines import next_collab_path_builder as builder
from modules.mission_planning.replanning.triggers.attack import pipeline as attack
from modules.mission_planning.replanning.triggers.post_attack import pipeline as post_attack


def _coord(x_m: float, y_m: float, altitude_m: float = 1000.0) -> dict:
    coord = builder._xy_to_coord((float(x_m), float(y_m)))
    coord["altitude"] = float(altitude_m)
    return coord


def _line_search_waypoint(
    *,
    anchor_x_m: float,
    sweep_length_m: float,
    flight_speed_mps: float,
    search_speed_mps: float,
    area: bool = False,
) -> dict:
    waypoint = {
        "waypointID": 2,
        "coordinate": _coord(anchor_x_m, 0.0),
        "speed": float(flight_speed_mps),
        "eta": 0,
        "ecf": 0.0,
        "nextWaypointID": 0,
        "waypointPassType": 1,
        "filmingProperty": {
            "fieldOfView": 5.2,
            "sensorType": 1,
            "operationMode": 2,
            "lineSearch": {
                "coordinateList": [
                    _coord(0.0, 0.0, 0.0),
                    _coord(sweep_length_m, 0.0, 0.0),
                ],
                "searchSpeed": float(search_speed_mps),
                "interpolationPoints": 2,
            },
        },
        "isDone": False,
    }
    if area:
        waypoint["coverageAcquisitionID"] = "areaMission:aircraft:5:path:500000043:traversal:forward"
    return waypoint


def _configure_area_builder(monkeypatch: pytest.MonkeyPatch) -> None:
    runtime_floats = {
        "uav_wp_interval_m": 420.0,
        "area_first_packet_sweep_group_scale": 1.5,
        "next_collab_area_terrain_route_spacing_min_m": 500.0,
        "next_collab_area_terrain_group_relief_limit_m": 100.0,
        "next_collab_area_terrain_group_heading_limit_deg": 8.0,
        "next_collab_area_scan_completion_speed_scale": 1.10,
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
                "next_collab_area_reciprocal_pass_enabled": False,
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


def _build_area_path_with_mismatched_planner_speed(
    monkeypatch: pytest.MonkeyPatch,
) -> tuple[dict, dict]:
    _configure_area_builder(monkeypatch)
    scan_lines = [
        [(float(x), -320.0), (float(x), 0.0), (float(x), 320.0)]
        for x in (100.0, 450.0, 800.0, 1150.0, 1500.0, 1850.0, 2200.0, 2550.0)
    ]
    path_row = {
        "aircraftID": 5,
        "pieceIndex": 1,
        "originXY": (-1800.0, 0.0),
        "originHeadingDeg": 90.0,
        "waypointStartXY": (0.0, 0.0),
        "waypointEndXY": (2700.0, 0.0),
        "areaCenterLineXY": [(0.0, 0.0), (2700.0, 0.0)],
        "areaMissionStartXY": (0.0, 0.0),
        "areaMissionEndXY": (2700.0, 0.0),
        "areaStabilizationLeadXY": (-1200.0, 0.0),
        "partPolygonXY": [
            (0.0, -350.0),
            (2700.0, -350.0),
            (2700.0, 350.0),
            (0.0, 350.0),
        ],
        "sweepLineListXY": scan_lines,
        "lineSweepSpacingM": 350.0,
        # Legacy planner field is km/h: 165.2976 / 3.6 = 45.916 m/s.
        "resolvedVelMps": 165.2976,
        "resolvedFovDeg": 10.0,
        "turnRadiusM": 450.0,
    }
    entry = _coord(-1800.0, 0.0, 1100.0)
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
        path_row,
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


def test_type2_line_can_disable_generic_16x_camera_cap(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def _runtime_float(key: str, default: float | None) -> float | None:
        return {
            "line_search_speed_max_mps": 0.0,
            "default_search_speed_multiplier": 16.0,
            "line_search_speed_min_transit_m": 1.0,
        }.get(key, default)

    monkeypatch.setattr(line_search_speed_guard, "_runtime_float", _runtime_float)

    kwargs = {
        "prev_xy": (0.0, 0.0),
        "anchor_xy": (1200.0, 0.0),
        "sweep_xy": [(0.0, 0.0), (48_000.0, 0.0)],
        "cruise_speed_mps": 40.0,
        "fallback_search_speed_mps": 40.0,
    }
    capped = builder._estimate_line_search_speed_xy_mps(
        **kwargs,
        multiplier_cap_enabled=True,
    )
    synchronized = builder._estimate_line_search_speed_xy_mps(
        **kwargs,
        multiplier_cap_enabled=False,
    )

    assert capped == pytest.approx(640.0)
    assert synchronized == pytest.approx(1600.0)
    assert 48_000.0 / synchronized == pytest.approx(1200.0 / 40.0)


def test_area_builder_synchronizes_to_emitted_waypoint_speed(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    path, metrics = _build_area_path_with_mismatched_planner_speed(monkeypatch)
    waypoints = path["waypointList"]
    capture_index = next(
        index
        for index, waypoint in enumerate(waypoints)
        if isinstance(
            ((waypoint.get("filmingProperty") or {}).get("lineSearch")),
            dict,
        )
    )
    assert capture_index > 0
    capture = waypoints[capture_index]
    previous = waypoints[capture_index - 1]
    line_search = capture["filmingProperty"]["lineSearch"]
    previous_xy = builder.coord_to_xy(previous["coordinate"])
    capture_xy = builder.coord_to_xy(capture["coordinate"])
    sweep_xy = [builder.coord_to_xy(coord) for coord in line_search["coordinateList"]]
    assert previous_xy is not None and capture_xy is not None
    assert all(point is not None for point in sweep_xy)

    flight_time_s = builder._distance_xy(previous_xy, capture_xy) / float(capture["speed"])
    scan_time_s = builder._line_length_xy(sweep_xy) / float(line_search["searchSpeed"])

    assert metrics["transitSpeedMps"] == pytest.approx(30.0)
    assert metrics["pathRowTransitSpeedMps"] == pytest.approx(45.916)
    assert metrics["areaScanCompletionReferenceSpeedMps"] == pytest.approx(30.0)
    # Keep the configured 10% completion margin, but use almost the whole leg.
    assert scan_time_s / flight_time_s == pytest.approx(1.0 / 1.10, rel=1e-3)


def test_next_collab_eta_runs_incoming_flight_and_capture_concurrently() -> None:
    entry = {
        "waypointID": 1,
        "coordinate": _coord(0.0, 0.0),
        "speed": 40.0,
        "eta": 0,
        "ecf": 0.0,
        "nextWaypointID": 2,
        "waypointPassType": 1,
        "isDone": False,
    }
    capture = _line_search_waypoint(
        anchor_x_m=1200.0,
        sweep_length_m=48_000.0,
        flight_speed_mps=40.0,
        search_speed_mps=1600.0,
    )

    builder._recompute_waypoint_timeline(
        [entry, capture],
        default_speed_mps=40.0,
    )

    # Both operations take about 30 seconds and happen together in the SIM.
    assert capture["eta"] == pytest.approx(30, abs=1)


def test_post_attack_does_not_double_boost_area_first_sweep() -> None:
    waypoint = _line_search_waypoint(
        anchor_x_m=1200.0,
        sweep_length_m=48_000.0,
        flight_speed_mps=30.0,
        search_speed_mps=1200.0,
        area=True,
    )
    payload = {"waypointList": [waypoint]}
    messages: list[str] = []

    result = post_attack._boost_post_attack_collab_first_sweep_search_speed(
        5,
        500000043,
        payload,
        emit=messages.append,
        speed_scale=1.3,
        reference_coord=_coord(0.0, 0.0),
    )

    search_speed = result["waypointList"][0]["filmingProperty"]["lineSearch"]["searchSpeed"]
    assert search_speed == pytest.approx(1200.0)
    assert any("AREA first sweep searchSpeed kept" in message for message in messages)


def test_attack_does_not_double_boost_area_first_sweep() -> None:
    waypoint = _line_search_waypoint(
        anchor_x_m=1200.0,
        sweep_length_m=48_000.0,
        flight_speed_mps=30.0,
        search_speed_mps=1200.0,
        area=True,
    )
    messages: list[str] = []

    result = attack._boost_attack_collab_first_sweep_search_speed(
        5,
        500000043,
        {"waypointList": [waypoint]},
        emit=messages.append,
        speed_scale=1.3,
        reference_coord=_coord(0.0, 0.0),
    )

    search_speed = result["waypointList"][0]["filmingProperty"]["lineSearch"]["searchSpeed"]
    assert search_speed == pytest.approx(1200.0)
    assert any("AREA first sweep searchSpeed kept" in message for message in messages)


def test_post_attack_line_boost_is_geometry_based_and_idempotent() -> None:
    waypoint = _line_search_waypoint(
        anchor_x_m=1200.0,
        sweep_length_m=48_000.0,
        flight_speed_mps=40.0,
        search_speed_mps=640.0,
    )
    reference = _coord(0.0, 0.0)
    payload = {"waypointList": [waypoint]}

    first = post_attack._boost_post_attack_collab_first_sweep_search_speed(
        4,
        400000057,
        payload,
        emit=lambda _message: None,
        speed_scale=1.3,
        reference_coord=reference,
    )
    first_speed = first["waypointList"][0]["filmingProperty"]["lineSearch"]["searchSpeed"]
    second = post_attack._boost_post_attack_collab_first_sweep_search_speed(
        4,
        400000057,
        deepcopy(first),
        emit=lambda _message: None,
        speed_scale=1.3,
        reference_coord=reference,
    )
    second_speed = second["waypointList"][0]["filmingProperty"]["lineSearch"]["searchSpeed"]

    assert first_speed > 640.0
    assert second_speed == pytest.approx(first_speed, rel=1e-6)


def test_attack_line_boost_is_geometry_based_and_idempotent() -> None:
    waypoint = _line_search_waypoint(
        anchor_x_m=1200.0,
        sweep_length_m=48_000.0,
        flight_speed_mps=40.0,
        search_speed_mps=640.0,
    )
    reference = _coord(0.0, 0.0)
    first = attack._boost_attack_collab_first_sweep_search_speed(
        4,
        400000057,
        {"waypointList": [waypoint]},
        emit=lambda _message: None,
        speed_scale=1.3,
        reference_coord=reference,
    )
    first_speed = first["waypointList"][0]["filmingProperty"]["lineSearch"]["searchSpeed"]
    second = attack._boost_attack_collab_first_sweep_search_speed(
        4,
        400000057,
        deepcopy(first),
        emit=lambda _message: None,
        speed_scale=1.3,
        reference_coord=reference,
    )
    second_speed = second["waypointList"][0]["filmingProperty"]["lineSearch"]["searchSpeed"]

    assert first_speed > 640.0
    assert second_speed == pytest.approx(first_speed, rel=1e-6)


def test_type2_attack_resume_recompute_is_uncapped_and_idempotent(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def _runtime_float(key: str, default: float | None) -> float | None:
        return {
            "line_search_speed_max_mps": 0.0,
            "default_search_speed_multiplier": 16.0,
            "line_search_speed_min_transit_m": 1.0,
        }.get(key, default)

    monkeypatch.setattr(line_search_speed_guard, "_runtime_float", _runtime_float)
    reference = _coord(0.0, 0.0)
    waypoint = _line_search_waypoint(
        anchor_x_m=1200.0,
        sweep_length_m=48_000.0,
        flight_speed_mps=40.0,
        # Deliberately model an already-boosted historical path.
        search_speed_mps=5000.0,
    )
    waypoints = [waypoint]

    changed = mission_path_trim.recompute_line_search_speed_from_geometry(
        waypoints,
        first_reference_coord=reference,
        speed_scale=1.3,
        only_increase=False,
        multiplier_cap_enabled=False,
    )
    first_speed = waypoints[0]["filmingProperty"]["lineSearch"]["searchSpeed"]
    changed_again = mission_path_trim.recompute_line_search_speed_from_geometry(
        waypoints,
        first_reference_coord=reference,
        speed_scale=1.3,
        only_increase=False,
        multiplier_cap_enabled=False,
    )
    second_speed = waypoints[0]["filmingProperty"]["lineSearch"]["searchSpeed"]

    assert changed == 1
    assert changed_again == 1
    assert first_speed > 640.0
    assert second_speed == pytest.approx(first_speed, rel=1e-6)
