from __future__ import annotations

import math
from pathlib import Path

from modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304 import d0304
from modules.mission_planning.pipelines import mission_planning_attack_helpers as legacy_attack_helpers
from modules.mission_planning.replanning.triggers.attack import pipeline


def test_attack_route_starts_at_least_ten_seconds_ahead(monkeypatch) -> None:
    projection: dict[str, float] = {}

    def _project(coord, heading, distance_m):
        projection.update({"heading": float(heading), "distance_m": float(distance_m)})
        return {
            "latitude": float(coord["latitude"]),
            "longitude": float(coord["longitude"]) + 0.001,
            "altitude": 0,
        }

    monkeypatch.setattr(pipeline, "_project_coordinate", _project)
    monkeypatch.setattr(
        pipeline,
        "get_runtime_attack_float",
        lambda _key, default: default,
    )
    current = {"latitude": 37.0, "longitude": 127.0, "altitude": 900}

    predicted = pipeline._predict_lah_attack_route_start(
        current,
        {"heading": 90.0, "speed": 42.0},
    )

    assert predicted is not None
    assert projection == {"heading": 90.0, "distance_m": 420.0}
    assert predicted["altitude"] == 900


def test_attack_route_stays_inside_connected_mission_zones(monkeypatch) -> None:
    zones = [
        {
            "zoneType": "line",
            "widthM": 200.0,
            "coordinateList": [
                {"latitude": 37.0, "longitude": 127.0},
                {"latitude": 37.0, "longitude": 127.01},
            ],
        },
        {
            "zoneType": "line",
            "widthM": 200.0,
            "coordinateList": [
                {"latitude": 37.0, "longitude": 127.01},
                {"latitude": 37.01, "longitude": 127.01},
            ],
        },
    ]
    monkeypatch.setattr(pipeline, "_load_attack_operation_zones", lambda _source_plan_id: zones)
    monkeypatch.setattr(
        pipeline,
        "get_runtime_attack_float",
        lambda _key, default: default,
    )
    start = {"latitude": 37.0, "longitude": 127.0, "altitude": 900}
    attack = {"latitude": 37.01, "longitude": 127.01, "altitude": 1200}
    direct_midpoint = {"latitude": 37.005, "longitude": 127.005}
    assert not pipeline._attack_point_inside_line_coverage(
        direct_midpoint,
        zones,
        tolerance_m=0.0,
    )

    route, metadata = pipeline._build_lah_mission_constrained_attack_route(
        start_coord=start,
        attack_coord=attack,
        source_plan_id=700000001,
    )

    assert metadata["constrained"] is True
    assert metadata["startInside"] is True
    assert metadata["attackInside"] is True
    assert len(route) >= 3
    assert route[0] == start
    assert route[-1] == attack
    for left, right in zip(route, route[1:]):
        for step in range(21):
            ratio = step / 20.0
            sample = {
                "latitude": float(left["latitude"])
                + ((float(right["latitude"]) - float(left["latitude"])) * ratio),
                "longitude": float(left["longitude"])
                + ((float(right["longitude"]) - float(left["longitude"])) * ratio),
            }
            assert pipeline._attack_point_inside_line_coverage(sample, zones, tolerance_m=1.0)


def test_attack_path_is_low_approach_then_vertical_attack(monkeypatch) -> None:
    monkeypatch.setattr(
        pipeline,
        "build_lah_terrain_following_path",
        lambda coords, **_kwargs: [
            {"latitude": 37.0, "longitude": 127.0, "altitude": 150, "cum_m": 0.0},
            {"latitude": 37.01, "longitude": 127.01, "altitude": 180, "cum_m": 1500.0},
        ],
    )
    allocated = iter([101, 102, 103])

    waypoints = pipeline._build_lah_low_level_attack_waypoints(
        template_wp=pipeline._default_lah_waypoint_template(),
        start_coord={"latitude": 37.0, "longitude": 127.0, "altitude": 900},
        attack_coord={"latitude": 37.01, "longitude": 127.01, "altitude": 600},
        attack_waypoint_id=100,
        waypoint_id_provider=lambda: next(allocated),
        target_id=7,
        weapon_type=2,
        speed_mps=50.0,
    )

    assert [waypoint["waypointID"] for waypoint in waypoints] == [101, 102, 103, 100]
    assert [waypoint["nextWaypointID"] for waypoint in waypoints] == [102, 103, 100, 0]
    assert [waypoint["coordinate"]["altitude"] for waypoint in waypoints] == [900, 150, 180, 600]
    assert waypoints[0]["coordinate"]["latitude"] == waypoints[1]["coordinate"]["latitude"]
    assert waypoints[0]["coordinate"]["longitude"] == waypoints[1]["coordinate"]["longitude"]
    assert waypoints[-2]["coordinate"]["latitude"] == waypoints[-1]["coordinate"]["latitude"]
    assert waypoints[-2]["coordinate"]["longitude"] == waypoints[-1]["coordinate"]["longitude"]
    assert waypoints[0]["attack"]["targetID"] == 0
    assert waypoints[1]["attack"]["targetID"] == 0
    assert waypoints[2]["attack"]["targetID"] == 0
    assert waypoints[3]["attack"] == {"targetID": 7, "weaponType": 2}
    expected_descent_s = int(
        math.ceil(750 / (pipeline.DEFAULT_ENVELOPE.descent_rate_mps * pipeline.LAH_VERTICAL_RATE_USE_RATIO))
    )
    expected_vertical_s = int(
        math.ceil(420 / (pipeline.DEFAULT_ENVELOPE.climb_rate_mps * pipeline.LAH_VERTICAL_RATE_USE_RATIO))
    )
    assert [waypoint["eta"] for waypoint in waypoints] == [
        0,
        expected_descent_s,
        expected_descent_s + 30,
        expected_descent_s + 30 + expected_vertical_s,
    ]


def test_resume_path_descends_at_attack_point_and_keeps_terminal_hold(monkeypatch) -> None:
    monkeypatch.setattr(
        pipeline,
        "build_lah_terrain_following_path",
        lambda coords, **_kwargs: [
            {"latitude": 37.01, "longitude": 127.01, "altitude": 180, "cum_m": 0.0},
            {"latitude": 37.02, "longitude": 127.02, "altitude": 170, "cum_m": 1500.0},
        ],
    )
    terminal = pipeline._default_lah_waypoint_template()
    terminal["coordinate"] = {"latitude": 37.02, "longitude": 127.02, "altitude": 999}
    terminal["hovering"] = {"time": 300}
    allocated = iter([201, 202, 203])

    waypoints = pipeline._rebuild_lah_low_level_resume_waypoints(
        attack_coord={"latitude": 37.01, "longitude": 127.01, "altitude": 600},
        resume_waypoints=[terminal],
        template_wp=pipeline._default_lah_waypoint_template(),
        waypoint_id_provider=lambda: next(allocated),
    )

    assert waypoints[0]["coordinate"]["altitude"] == 600
    assert waypoints[0]["coordinate"]["latitude"] == 37.01
    assert waypoints[1]["coordinate"]["altitude"] == 180
    assert waypoints[1]["coordinate"]["latitude"] == 37.01
    assert waypoints[-1]["coordinate"]["altitude"] == 170
    assert waypoints[-1]["hovering"] == {"time": 300}
    assert all(waypoint["attack"]["targetID"] == 0 for waypoint in waypoints)
    expected_descent_s = int(
        math.ceil(420 / (pipeline.DEFAULT_ENVELOPE.descent_rate_mps * pipeline.LAH_VERTICAL_RATE_USE_RATIO))
    )
    expected_horizontal_s = int(math.ceil(1500 / 40.0))
    assert [waypoint["eta"] for waypoint in waypoints] == [
        0,
        expected_descent_s,
        expected_descent_s + expected_horizontal_s,
    ]


def test_single_lah_waypoint_keeps_operation_duration_separate_from_eta() -> None:
    hover_waypoint = {
        "waypointID": 101,
        "eta": 0,
        "ecf": 0.0,
        "nextWaypointID": 999,
        "hovering": {"time": 300},
        "_allowSingleLahWaypoint": True,
    }
    d0304._normalize_lah_waypoint_list_inplace([hover_waypoint])

    assert hover_waypoint["eta"] == 0
    assert hover_waypoint["ecf"] == 1.0
    assert hover_waypoint["nextWaypointID"] == 0
    assert "_allowSingleLahWaypoint" not in hover_waypoint

    loiter_waypoint = {
        "waypointID": 102,
        "eta": 0,
        "ecf": 0.0,
        "nextWaypointID": 0,
        "loiter": {"time": 15},
        "_allowSingleLahWaypoint": True,
    }
    d0304._normalize_lah_waypoint_list_inplace([loiter_waypoint])

    assert loiter_waypoint["eta"] == 0
    assert loiter_waypoint["ecf"] == 1.0


def test_attack_hold_anchor_starts_at_zero_eta() -> None:
    waypoint = pipeline._build_lah_anchor_waypoint(
        pipeline._default_lah_waypoint_template(),
        coord={"latitude": 37.0, "longitude": 127.0, "altitude": 500},
        hovering_time=300,
        waypoint_id=103,
    )

    assert waypoint["eta"] == 0
    assert waypoint["hovering"] == {"time": 300}
    assert waypoint["ecf"] == 1.0
    assert waypoint["nextWaypointID"] == 0


def test_lah_eta_is_cumulative_uint32_seconds_and_resets_per_plan() -> None:
    first_plan = [
        {"eta": 120},
        {"eta": 140},
        {"eta": 2**40},
    ]
    second_plan = [{"eta": 300}, {"eta": 307}]

    d0304.normalize_lah_eta_seconds_inplace(first_plan)
    d0304.normalize_lah_eta_seconds_inplace(second_plan)

    assert [waypoint["eta"] for waypoint in first_plan] == [0, 20, 0xFFFFFFFF]
    assert [waypoint["eta"] for waypoint in second_plan] == [0, 7]


def test_lah_eta_recompute_uses_seconds_not_milliseconds() -> None:
    start = {"latitude": 37.0, "longitude": 127.0, "altitude": 500}
    end = {"latitude": 37.0, "longitude": 127.005, "altitude": 500}
    packet = {
        "lahWaypointList": [
            {"coordinate": start, "speed": 24.0, "eta": 999},
            {"coordinate": end, "speed": 24.0, "eta": 999_999},
        ]
    }

    d0304._recompute_lah_eta_inplace(packet)

    expected_s = int(math.ceil(d0304._coord_dist_m(start, end) / 24.0 - 1e-9))
    assert [waypoint["eta"] for waypoint in packet["lahWaypointList"]] == [0, expected_s]
    assert expected_s < 100


def test_lah_eta_recompute_accounts_for_same_coordinate_climb_time() -> None:
    transitions = [
        (37.882897, 128.130908, 770, 948),
        (37.909489, 128.154783, 932, 1005),
    ]
    speed_mps = 40.0
    climb_rate_mps = d0304._lah_vertical_rate_mps(1.0)

    for latitude, longitude, start_altitude, end_altitude in transitions:
        start = {
            "latitude": latitude,
            "longitude": longitude,
            "altitude": start_altitude,
        }
        end = {
            "latitude": latitude,
            "longitude": longitude,
            "altitude": end_altitude,
        }
        packet = {
            "lahWaypointList": [
                {"coordinate": start, "speed": speed_mps, "eta": 0},
                {"coordinate": end, "speed": speed_mps, "eta": 0},
            ]
        }

        d0304._recompute_lah_eta_inplace(packet)

        horizontal_s = d0304._coord_dist_m(start, end) / speed_mps
        vertical_s = (end_altitude - start_altitude) / climb_rate_mps
        expected_s = int(math.ceil(max(horizontal_s, vertical_s) - 1e-9))
        actual_s = packet["lahWaypointList"][1]["eta"]
        assert horizontal_s == 0.0
        assert actual_s == expected_s
        assert actual_s > 0


def test_lah_boundary_climb_is_scheduled_without_moving_attack_action() -> None:
    boundary = {"latitude": 37.882897, "longitude": 128.130908}
    previous = {
        "aircraftID": 3,
        "pathID": 300000011,
        "lahWaypointList": [
            {
                "waypointID": 0,
                "coordinate": {**boundary, "altitude": 760},
                "speed": 40.0,
                "eta": 0,
                "ecf": 0.0,
                "nextWaypointID": 0,
            },
            {
                "waypointID": 0,
                "coordinate": {**boundary, "altitude": 770},
                "speed": 40.0,
                "eta": 0,
                "ecf": 1.0,
                "nextWaypointID": 0,
                "attack": {"targetID": 157, "weaponType": 2},
            },
        ],
    }
    following = {
        "aircraftID": 3,
        "pathID": 300000004,
        "lahWaypointList": [
            {
                "waypointID": 0,
                "coordinate": {**boundary, "altitude": 948},
                "speed": 40.0,
                "eta": 0,
                "ecf": 0.0,
                "nextWaypointID": 0,
            },
            {
                "waypointID": 0,
                "coordinate": {"latitude": 37.89, "longitude": 128.14, "altitude": 1000},
                "speed": 40.0,
                "eta": 1,
                "ecf": 1.0,
                "nextWaypointID": 0,
            },
        ],
    }

    inserted = d0304._harmonize_lah_packet_boundary_altitudes_inplace(
        [previous, following]
    )

    assert inserted == 1
    assert previous["lahWaypointList"][-2]["attack"]["targetID"] == 157
    assert previous["lahWaypointList"][-2]["coordinate"]["altitude"] == 770
    assert previous["lahWaypointList"][-1]["coordinate"]["altitude"] == 948
    assert previous["lahWaypointList"][-1]["eta"] > previous["lahWaypointList"][-2]["eta"]
    assert following["lahWaypointList"][0]["coordinate"]["altitude"] == 948
    assert following["lahWaypointList"][0]["eta"] == 0


def test_generic_attack_terrain_refinement_preserves_target_waypoint(monkeypatch) -> None:
    monkeypatch.setattr(
        d0304,
        "build_lah_terrain_following_path",
        lambda *args, **kwargs: [
            {"latitude": 37.0, "longitude": 127.0, "altitude": 175, "cum_m": 0.0},
            {"latitude": 37.01, "longitude": 127.01, "altitude": 180, "cum_m": 1500.0},
        ],
    )
    source = [
        {
            "waypointID": 0,
            "coordinate": {"latitude": 37.0, "longitude": 127.0, "altitude": 900},
            "speed": 40.0,
            "eta": 0,
            "ecf": 0.0,
            "nextWaypointID": 0,
        },
        {
            "waypointID": 0,
            "coordinate": {"latitude": 37.01, "longitude": 127.01, "altitude": 600},
            "speed": 40.0,
            "eta": 0,
            "ecf": 1.0,
            "nextWaypointID": 0,
            "attack": {"targetID": 157, "weaponType": 2},
        },
    ]

    refined = d0304._terrain_refine_existing_lah_waypoints(
        source,
        cruise_speed=40.0,
    )

    assert [wp["coordinate"]["altitude"] for wp in refined] == [900, 180, 600]
    assert refined[-1]["attack"] == {"targetID": 157, "weaponType": 2}
    assert "attack" not in refined[-2]
    assert refined[-1]["eta"] > refined[-2]["eta"]


def test_legacy_attack_customization_keeps_seconds_and_vertical_eta(monkeypatch) -> None:
    monkeypatch.setattr(legacy_attack_helpers, "get_last_assigned_manned_id", lambda: None)
    monkeypatch.setattr(legacy_attack_helpers, "set_last_assigned_manned_id", lambda _aid: None)
    monkeypatch.setattr(
        legacy_attack_helpers,
        "get_runtime_attack_int_list",
        lambda _key, _default: [2, 3],
    )
    monkeypatch.setattr(
        legacy_attack_helpers,
        "get_runtime_attack_weapon_type_for_target_type",
        lambda *args, **kwargs: 2,
    )
    monkeypatch.setattr(
        legacy_attack_helpers,
        "load_current_aircraft_weapon_inventory",
        lambda _aid: {2: 1},
    )
    monkeypatch.setattr(
        legacy_attack_helpers,
        "choose_attack_weapon_type",
        lambda _preferred, _inventory: {
            "selectedWeaponType": 2,
            "ammoAvailable": True,
            "weaponInventory": {2: 1},
        },
    )
    monkeypatch.setattr(
        legacy_attack_helpers,
        "compute_attack_waypoint",
        lambda *args, **kwargs: {
            "latitude": 37.01,
            "longitude": 127.01,
            "altitude": 600,
        },
    )
    monkeypatch.setattr(
        legacy_attack_helpers,
        "build_lah_terrain_following_path",
        lambda _coords, **_kwargs: [
            {"latitude": 37.0, "longitude": 127.0, "altitude": 150, "cum_m": 0.0},
            {"latitude": 37.01, "longitude": 127.01, "altitude": 180, "cum_m": 1500.0},
        ],
    )
    missions = [
        {
            "aircraftID": 2,
            "individualMissionID": 900000001,
            "pathID": 200000001,
            "individualMissionInfo": {
                "coordinateList": [
                    {"latitude": 37.0, "longitude": 127.0, "altitude": 900}
                ]
            },
        }
    ]
    flight_plans = [
        {
            "aircraftID": 2,
            "pathID": 200000001,
            "lahWaypointList": [
                {
                    "coordinate": {
                        "latitude": 36.999,
                        "longitude": 126.999,
                        "altitude": 770,
                    }
                }
            ],
        }
    ]

    result = legacy_attack_helpers.apply_attack_customizations(
        missions,
        flight_plans,
        {
            "targetID": 157,
            "targetType": 1,
            "target": {"latitude": 37.01, "longitude": 127.01, "altitude": 0},
        },
        1,
        project_root=Path("."),
    )

    assert result["applied"] is True
    waypoints = flight_plans[0]["lahWaypointList"]
    assert waypoints[0]["coordinate"] == {
        "latitude": 36.999,
        "longitude": 126.999,
        "altitude": 770,
    }
    assert [wp["coordinate"]["altitude"] for wp in waypoints] == [770, 150, 180, 600]
    assert waypoints[0]["eta"] == 0
    assert all(left["eta"] <= right["eta"] for left, right in zip(waypoints, waypoints[1:]))
    assert waypoints[1]["eta"] > 0
    assert waypoints[-1]["eta"] > waypoints[-2]["eta"]
    assert waypoints[-1]["attack"] == {"targetID": 157, "weaponType": 2}
    assert waypoints[-1]["eta"] < 1000


def test_attack_write_boundary_resets_trimmed_lah_eta() -> None:
    payload = {
        "aircraftID": 1,
        "lahWaypointList": [
            {"waypointID": 1, "eta": 120},
            {"waypointID": 2, "eta": 145},
        ],
    }

    pipeline._prepare_attack_json_payload(Path("FlightPath") / "100000001.json", payload)

    assert [waypoint["eta"] for waypoint in payload["lahWaypointList"]] == [0, 25]
