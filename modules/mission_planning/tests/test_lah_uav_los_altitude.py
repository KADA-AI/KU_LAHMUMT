from __future__ import annotations

import math
from copy import deepcopy

from modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304 import (
    d0304,
)


def _waypoint(latitude: float, longitude: float, altitude: int, eta: int) -> dict:
    return {
        "coordinate": {
            "latitude": latitude,
            "longitude": longitude,
            "altitude": altitude,
        },
        "speed": 65.0,
        "eta": eta,
        "ecf": 1.0,
        "nextWaypointID": 0,
    }


def test_los_altitude_uses_uav_position_and_altitude_at_lah_eta(monkeypatch) -> None:
    lah_waypoint = _waypoint(37.0, 127.0, 150, 50)
    lah_waypoint["hovering"] = {"time": 300}
    packet = {"aircraftID": 1, "lahWaypointList": [lah_waypoint]}
    uav_packet = {
        "aircraftID": 4,
        "waypointList": [
            _waypoint(37.0, 127.02, 300, 0),
            _waypoint(37.0, 127.04, 500, 100),
        ],
    }
    queried: list[tuple[float, float]] = []

    def terrain_profile(coords):
        queried.extend((float(lat), float(lon)) for lat, lon in coords)
        return [
            500.0 if 127.0145 <= float(lon) <= 127.0155 else 100.0
            for _lat, lon in coords
        ]

    monkeypatch.setattr(d0304, "_terrain_profile_many", terrain_profile)

    result = d0304._apply_lah_uav_los_altitudes_inplace(
        packet,
        d0304._uav_timeline_from_packet(uav_packet),
    )

    # ETA 50 s is the midpoint of the UAV plan: lon=127.03, altitude=400 m.
    assert max(lon for _lat, lon in queried) == 127.03
    assert 600 <= int(lah_waypoint["coordinate"]["altitude"]) <= 660
    assert lah_waypoint["hovering"] == {"time": 300}
    assert result["reason"] == "ok"
    assert result["applied"] == 1


def test_los_dem_lookup_is_one_bounded_batch_per_lah_packet(monkeypatch) -> None:
    waypoints = [
        _waypoint(37.0, 127.0 + index * 0.00001, 150, index)
        for index in range(100)
    ]
    packet = {"aircraftID": 1, "lahWaypointList": waypoints}
    timeline = [
        {
            "eta_s": 0.0,
            "coord": {"latitude": 37.0, "longitude": 127.225, "altitude": 500},
        }
    ]
    calls: list[int] = []

    def terrain_profile(coords):
        pairs = list(coords)
        calls.append(len(pairs))
        return [100.0] * len(pairs)

    monkeypatch.setattr(d0304, "_terrain_profile_many", terrain_profile)

    result = d0304._apply_lah_uav_los_altitudes_inplace(packet, timeline)

    assert len(calls) == 1
    assert calls[0] == result["sampleCount"]
    assert calls[0] <= d0304.LAH_UAV_LOS_MAX_TOTAL_SAMPLES


def test_los_dem_failure_keeps_existing_route_unchanged(monkeypatch) -> None:
    original = _waypoint(37.0, 127.0, 250, 0)
    packet = {"aircraftID": 1, "lahWaypointList": [deepcopy(original)]}
    timeline = [
        {
            "eta_s": 0.0,
            "coord": {"latitude": 37.0, "longitude": 127.01, "altitude": 400},
        }
    ]

    def terrain_failure(_coords):
        raise RuntimeError("synthetic DEM failure")

    monkeypatch.setattr(d0304, "_terrain_profile_many", terrain_failure)

    result = d0304._apply_lah_uav_los_altitudes_inplace(packet, timeline)

    assert packet["lahWaypointList"][0] == original
    assert result["reason"] == "terrain_lookup_failed"


def test_los_altitude_is_lift_smoothed_for_lah_vertical_rates() -> None:
    waypoints = [
        _waypoint(37.0, 127.0, 100, 0),
        _waypoint(37.0, 127.001, 600, 10),
        _waypoint(37.0, 127.002, 100, 20),
    ]

    d0304._smooth_lah_vertical_altitudes_inplace(waypoints)

    altitudes = [int(wp["coordinate"]["altitude"]) for wp in waypoints]
    climb_rate_mps = d0304._lah_vertical_rate_mps(1.0)
    descent_rate_mps = d0304._lah_vertical_rate_mps(-1.0)
    assert altitudes == [
        int(math.ceil(600 - climb_rate_mps * 10)),
        600,
        int(math.ceil(600 - descent_rate_mps * 10)),
    ]


def test_eta_follow_pipeline_applies_los_after_speed_and_terminal_planning(monkeypatch) -> None:
    lah_packet = {
        "aircraftID": 1,
        "pathID": 101,
        "lahWaypointList": [
            _waypoint(37.0, 127.0, 150, 0),
            _waypoint(37.0, 127.001, 150, 20),
        ],
    }
    uav_packet = {
        "aircraftID": 4,
        "pathID": 401,
        "waypointList": [
            _waypoint(37.0, 127.02, 400, 0),
            _waypoint(37.0, 127.02, 400, 100),
        ],
    }

    def terrain_profile(coords):
        return [
            500.0 if 127.0095 <= float(lon) <= 127.0105 else 100.0
            for _lat, lon in coords
        ]

    monkeypatch.setattr(d0304, "_terrain_profile_many", terrain_profile)

    d0304.apply_uav_eta_follow_speed_plan([lah_packet], [uav_packet])

    assert all(
        int(waypoint["coordinate"]["altitude"]) > 150
        for waypoint in lah_packet["lahWaypointList"]
    )


def _uav_packet(aircraft_id: int, longitude: float, altitude: int) -> dict:
    return {
        "aircraftID": aircraft_id,
        "waypointList": [
            _waypoint(37.0, longitude, altitude, 0),
            _waypoint(37.0, longitude, altitude, 100),
        ],
    }


def _ridge_terrain(ridge_lon: float, height: float):
    def terrain_profile(coords):
        return [
            height if abs(float(lon) - ridge_lon) <= 0.0006 else 100.0
            for _lat, lon in coords
        ]

    return terrain_profile


def test_command_aircraft_altitude_clears_every_uav_not_only_its_pair(monkeypatch) -> None:
    """LAH1 relays for the team, so one blocked UAV must still raise it."""

    lah_waypoint = _waypoint(37.0, 127.0, 150, 0)
    lah_packet = {"aircraftID": 1, "pathID": 11, "lahWaypointList": [lah_waypoint]}
    # UAV4 (the ETA pair) sits behind flat ground; UAV6 sits behind a ridge.
    uav_packets = [
        _uav_packet(4, 127.01, 300),
        _uav_packet(5, 127.02, 300),
        _uav_packet(6, 127.06, 300),
    ]
    monkeypatch.setattr(d0304, "_terrain_profile_many", _ridge_terrain(127.03, 900.0))

    paired = {id(lah_packet): uav_packets[0]}
    relay = {id(lah_packet): uav_packets}
    d0304._apply_lah_uav_los_altitude_plan(
        [lah_packet],
        paired,
        immutable_path_ids=set(),
        relay_uav_packets_by_lah=relay,
    )
    relay_altitude = int(lah_waypoint["coordinate"]["altitude"])

    # The same geometry with only the paired UAV must stay lower.
    paired_only_waypoint = _waypoint(37.0, 127.0, 150, 0)
    paired_only_packet = {
        "aircraftID": 1,
        "pathID": 12,
        "lahWaypointList": [paired_only_waypoint],
    }
    d0304._apply_lah_uav_los_altitude_plan(
        [paired_only_packet],
        {id(paired_only_packet): uav_packets[0]},
        immutable_path_ids=set(),
    )
    paired_altitude = int(paired_only_waypoint["coordinate"]["altitude"])

    assert relay_altitude > paired_altitude, (relay_altitude, paired_altitude)
    assert relay_altitude >= 900


def test_wingmen_keep_the_paired_uav_behaviour(monkeypatch) -> None:
    lah_waypoint = _waypoint(37.0, 127.0, 150, 0)
    lah_packet = {"aircraftID": 2, "pathID": 21, "lahWaypointList": [lah_waypoint]}
    monkeypatch.setattr(d0304, "_terrain_profile_many", _ridge_terrain(127.03, 900.0))

    d0304._apply_lah_uav_los_altitude_plan(
        [lah_packet],
        {id(lah_packet): _uav_packet(5, 127.01, 300)},
        immutable_path_ids=set(),
        relay_uav_packets_by_lah={},
    )

    # Its own UAV is behind flat ground, so the far ridge must not lift it.
    assert int(lah_waypoint["coordinate"]["altitude"]) < 900


def test_relay_pass_never_lowers_an_established_altitude(monkeypatch) -> None:
    lah_waypoint = _waypoint(37.0, 127.0, 1500, 0)
    lah_packet = {"aircraftID": 1, "pathID": 31, "lahWaypointList": [lah_waypoint]}
    monkeypatch.setattr(d0304, "_terrain_profile_many", _ridge_terrain(127.03, 900.0))

    d0304._apply_lah_uav_los_altitude_plan(
        [lah_packet],
        {},
        immutable_path_ids=set(),
        relay_uav_packets_by_lah={
            id(lah_packet): [_uav_packet(4, 127.01, 300), _uav_packet(6, 127.06, 300)]
        },
    )

    assert int(lah_waypoint["coordinate"]["altitude"]) >= 1500
