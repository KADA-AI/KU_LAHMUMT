from __future__ import annotations

from copy import deepcopy

import pytest

from modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304 import (
    d0303,
)
from modules.mission_planning.pipelines import next_collab_path_builder as next_builder
from modules.mission_planning.pipelines.handover_terminal import (
    CONTROL_TRANSFER_COORDINATE_LIST_KEY,
    CONTROL_TRANSFER_DIRECT_MISSION_MARKER,
    HANDOVER_TERMINAL_MISSION_MARKER,
    handover_coordinate_map,
    mark_handover_terminal_missions,
    mission_requests_handover_terminal,
)


def _coord(latitude: float, longitude: float, altitude: float = 1100.0) -> dict:
    return {
        "latitude": float(latitude),
        "longitude": float(longitude),
        "altitude": float(altitude),
    }


def test_handover_remains_reference_data_and_region2_marks_direct_transit() -> None:
    route = [_coord(38.0, 127.0), _coord(38.02, 127.03)]
    input_plan = {
        "inputMissionList": [
            {"inputMissionID": 70000004, "regionType": 1},
            {
                "inputMissionID": 70000005,
                "regionType": 2,
                "missionDetail": {
                    "lineList": [{"width": 600, "coordinateList": route}],
                },
            },
        ]
    }
    missions = [
        {
            "aircraftID": 4,
            "relatedMission": {"inputMissionID": 70000004},
            "individualMissionInfo": {},
        },
        {
            "aircraftID": 4,
            "relatedMission": {"inputMissionID": 70000005},
            "individualMissionInfo": {
                "lineList": [{"width": 200, "coordinateList": route}],
            },
            HANDOVER_TERMINAL_MISSION_MARKER: True,
        },
    ]
    reference = {
        "handOverInfoList": [
            {"aircraftID": 4, "coordinate": _coord(38.1, 127.1, 0.0)},
            {"aircraftID": 5, "coordinate": _coord(38.2, 127.2, 0.0)},
        ]
    }

    assert mark_handover_terminal_missions(missions, input_plan) == 1
    assert HANDOVER_TERMINAL_MISSION_MARKER not in missions[0]
    assert HANDOVER_TERMINAL_MISSION_MARKER not in missions[1]
    assert CONTROL_TRANSFER_DIRECT_MISSION_MARKER not in missions[0]
    assert missions[1][CONTROL_TRANSFER_DIRECT_MISSION_MARKER] is True
    marked_info = missions[1]["individualMissionInfo"]
    assert marked_info["sourceRegionType"] == 2
    assert marked_info["FOV"] == 15.0
    assert marked_info[CONTROL_TRANSFER_COORDINATE_LIST_KEY] == route
    assert not mission_requests_handover_terminal(
        {"individualMissionInfo": {"sourceRegionType": 2}}
    )
    assert handover_coordinate_map(reference) == {
        4: _coord(38.1, 127.1, 0.0),
        5: _coord(38.2, 127.2, 0.0),
    }


def test_initial_region2_uses_direct_nadir_route_without_line_search(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(d0303, "_dem_alt", lambda _lat, _lon: 100.0)
    route = [_coord(38.0, 127.0), _coord(38.02, 127.03)]
    mission = {
        "aircraftID": 4,
        "pathID": 400000123,
        "individualMissionID": 900000123,
        "relatedMission": {"inputMissionID": 70000005},
        CONTROL_TRANSFER_DIRECT_MISSION_MARKER: True,
        "individualMissionInfo": {
            "individualMissionType": 6,
            "sourceRegionType": 2,
            CONTROL_TRANSFER_COORDINATE_LIST_KEY: route,
            "lineList": [{"width": 600, "coordinateList": route}],
        },
    }

    [flight_path] = d0303.build_flight_plans([mission], cruise_speed=40.0)
    waypoints = flight_path["waypointList"]

    assert len(waypoints) == 4
    assert waypoints[0]["coordinate"]["latitude"] == pytest.approx(38.0)
    assert waypoints[0]["coordinate"]["longitude"] == pytest.approx(127.0)
    assert waypoints[-1]["coordinate"]["latitude"] == pytest.approx(38.02)
    assert waypoints[-1]["coordinate"]["longitude"] == pytest.approx(127.03)
    route_gaps_m = [
        d0303._dist_between_coords(
            waypoints[index - 1]["coordinate"],
            waypoints[index]["coordinate"],
        )
        for index in range(1, len(waypoints))
    ]
    assert max(route_gaps_m) <= 1500.0
    for waypoint in waypoints:
        filming = waypoint["filmingProperty"]
        assert filming["fieldOfView"] == 15.0
        assert filming["operationMode"] == d0303.OPMODE_HOLD
        assert filming["aircraftFixed"] == {"gimbalPitch": -90.0, "gimbalYaw": 0.0}
        assert "lineSearch" not in filming


def test_next_collab_region2_uses_original_centerline_without_search(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(next_builder, "_dem_altitudes_for_pairs_cached", lambda pairs: [100.0] * len(pairs))
    route = [_coord(38.0, 127.0), _coord(38.02, 127.03)]
    mission_info = {
        "individualMissionType": 6,
        "sourceRegionType": 2,
        CONTROL_TRANSFER_DIRECT_MISSION_MARKER: True,
        CONTROL_TRANSFER_COORDINATE_LIST_KEY: route,
        "lineList": [{"width": 200, "coordinateList": route}],
    }
    flight_path = next_builder.build_flight_path_from_planned_row(
        {
            "aircraftID": 4,
            "centerLineXY": [(0.0, 0.0), (100.0, 0.0)],
            "sweepLineListXY": [[(0.0, -100.0), (100.0, -100.0)]],
        },
        template_path=None,
        mission_info=mission_info,
        individual_mission_id=900000123,
        path_id=400000123,
        aircraft_id=4,
        entry_coord=None,
        timestamp_ms=123,
    )
    waypoints = flight_path["waypointList"]

    assert len(waypoints) == 2
    assert [wp["coordinate"]["latitude"] for wp in waypoints] == [38.0, 38.02]
    assert [wp["coordinate"]["longitude"] for wp in waypoints] == [127.0, 127.03]
    for waypoint in waypoints:
        filming = waypoint["filmingProperty"]
        assert filming["fieldOfView"] == 15.0
        assert filming["operationMode"] == 4
        assert filming["aircraftFixed"] == {"gimbalPitch": -90.0, "gimbalYaw": 0.0}
        assert "lineSearch" not in filming


def test_initial_0303_does_not_append_handover_waypoint(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(d0303, "_dem_alt", lambda _lat, _lon: 100.0)
    path_id = 400000123
    packets = [
        {
            "aircraftID": 4,
            "pathID": path_id,
            "_mission_alt_offset_m": 1000.0,
            "_mission_cruise_speed_mps": 50.0,
            "wplist": [
                {
                    "waypointID": 0,
                    "nextWaypointID": 0,
                    "coordinate": _coord(38.0, 127.0),
                    "speed": 50.0,
                    "eta": 0,
                    "ecf": 0.0,
                    "waypointPassType": 1,
                    "filmingProperty": {
                        "fieldOfView": 7.2,
                        "sensorType": 1,
                        "operationMode": 2,
                        "lineSearch": {
                            "coordinateList": [_coord(38.0, 127.0)],
                            "searchSpeed": 50.0,
                        },
                    },
                }
            ],
        }
    ]
    before = deepcopy(packets)
    target = _coord(38.01, 127.02, 0.0)

    applied = d0303._append_handover_terminal_waypoints_inplace(
        packets,
        hand_over_map={4: target},
        terminal_path_ids={4: path_id},
        default_speed_mps=30.0,
        default_fov_deg=10.0,
    )

    assert applied == 0
    assert packets == before


def test_next_collab_does_not_append_handover_waypoint() -> None:
    waypoints = [
        {
            "waypointID": 0,
            "nextWaypointID": 0,
            "coordinate": _coord(38.0, 127.0),
            "speed": 40.0,
            "eta": 0,
            "ecf": 0.0,
            "waypointPassType": next_builder.PASS_FLYBY,
            "filmingProperty": {"operationMode": 2, "lineSearch": {}},
        }
    ]
    before = deepcopy(waypoints)
    target = _coord(38.02, 127.03, 0.0)

    assert not next_builder._append_handover_terminal_waypoint_inplace(
        waypoints,
        handover_coord=target,
        altitude_fn=lambda _lat, _lon: 1234,
        speed_mps=40.0,
        sensor_type=1,
        field_of_view_deg=7.2,
    )
    assert waypoints == before


def test_coincident_endpoint_still_does_not_get_handover_camera_state() -> None:
    target = _coord(38.0, 127.0, 0.0)
    waypoints = [
        {
            "coordinate": _coord(38.0, 127.0, 1200.0),
            "filmingProperty": {"operationMode": 2, "lineSearch": {"coordinateList": [target]}},
        }
    ]

    before = deepcopy(waypoints)
    assert not next_builder._append_handover_terminal_waypoint_inplace(
        waypoints,
        handover_coord=target,
        altitude_fn=lambda _lat, _lon: 1200,
        speed_mps=30.0,
        sensor_type=1,
        field_of_view_deg=10.0,
    )

    assert waypoints == before
