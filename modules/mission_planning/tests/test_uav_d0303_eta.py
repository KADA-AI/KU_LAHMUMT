from __future__ import annotations

import math
from pathlib import Path

import pytest

from modules.common import ecf as ecf_module

from modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304 import (
    d0303,
)
from modules.mission_planning.runtime import json_io


EARTH_RADIUS_M = 6_371_000.0


def _coord(x_m: float, altitude_m: float = 0.0) -> dict:
    return {
        "latitude": 0.0,
        "longitude": math.degrees(float(x_m) / EARTH_RADIUS_M),
        "altitude": float(altitude_m),
    }


def _waypoint(
    x_m: float,
    *,
    speed_mps: float = 10.0,
    sweep_x_m: list[float] | None = None,
    search_speed_mps: float = 10.0,
    loiter_time_s: float | None = None,
) -> dict:
    waypoint = {
        "waypointID": 0,
        "nextWaypointID": 0,
        "coordinate": _coord(x_m),
        "speed": float(speed_mps),
        "eta": 999,
        "ecf": 0.0,
    }
    if sweep_x_m is not None:
        waypoint["filmingProperty"] = {
            "operationMode": 2,
            "lineSearch": {
                "coordinateList": [_coord(value) for value in sweep_x_m],
                "searchSpeed": float(search_speed_mps),
            },
        }
    if loiter_time_s is not None:
        waypoint["loiterProperty"] = {
            "time": float(loiter_time_s),
            "radius": 100.0,
            "speed": float(speed_mps),
        }
    return waypoint


def _etas(waypoints: list[dict]) -> list[int]:
    return [int(waypoint["eta"]) for waypoint in waypoints]


def test_area_same_anchor_destination_linesearch_produces_positive_eta() -> None:
    waypoints = [
        _waypoint(0.0),
        _waypoint(0.0, sweep_x_m=[0.0, 100.0, 0.0]),
    ]

    d0303._annotate_eta_ms_inplace(waypoints, default_speed_mps=10.0)

    assert _etas(waypoints) == [0, 20]


def test_incoming_linesearch_runs_concurrently_with_uav_flight() -> None:
    waypoints = [
        _waypoint(0.0),
        _waypoint(100.0, sweep_x_m=[0.0, 100.0]),
    ]

    d0303._annotate_eta_ms_inplace(waypoints, default_speed_mps=10.0)

    # 10 s flight and 10 s camera sweep happen together, not one after another.
    assert _etas(waypoints) == [0, 10]


def test_line_eta_charges_each_destination_sweep_once_and_includes_terminal() -> None:
    waypoints = [
        _waypoint(0.0),
        _waypoint(100.0, sweep_x_m=[0.0, 100.0]),
        _waypoint(200.0, sweep_x_m=[0.0, 200.0]),
    ]

    d0303._annotate_eta_ms_inplace(waypoints, default_speed_mps=10.0)

    assert _etas(waypoints) == [0, 10, 30]


def test_previous_loiter_remains_sequential_before_incoming_leg() -> None:
    waypoints = [
        _waypoint(0.0, loiter_time_s=15.0),
        _waypoint(100.0, sweep_x_m=[0.0, 100.0]),
    ]

    d0303._annotate_eta_ms_inplace(waypoints, default_speed_mps=10.0)

    assert _etas(waypoints) == [0, 25]


def test_incoming_leg_uses_current_waypoint_aircraft_speed_like_sim() -> None:
    waypoints = [
        _waypoint(0.0, speed_mps=40.0),
        _waypoint(400.0, speed_mps=20.0),
    ]

    d0303._annotate_eta_ms_inplace(waypoints, default_speed_mps=40.0)

    assert _etas(waypoints) == [0, 20]


def test_single_sweep_keeps_first_eta_zero_and_uses_normalized_terminal_eta() -> None:
    waypoints = [_waypoint(0.0, sweep_x_m=[0.0, 100.0])]

    d0303._normalize_output_waypoints_inplace(waypoints)
    d0303._annotate_eta_ms_inplace(waypoints, default_speed_mps=10.0)
    d0303._annotate_ecf_inplace(waypoints)

    assert len(waypoints) == 2
    assert _etas(waypoints) == [0, 10]
    assert "filmingProperty" in waypoints[0]
    assert "filmingProperty" not in waypoints[1]
    # ICD ecf is per-leg fuel in litres: the first waypoint has no preceding
    # leg, the 10 s leg burns 10 x NOMINAL_BURN_L_PER_S.
    assert [waypoint["ecf"] for waypoint in waypoints] == pytest.approx(
        [0.0, 10.0 * ecf_module.NOMINAL_BURN_L_PER_S]
    )


def test_ecf_is_per_leg_fuel_not_cumulative_progress() -> None:
    """ICD 0303/0304: litres burned on the leg into each waypoint."""

    waypoints = [
        {"eta": 0},
        {"eta": 10},
        {"eta": 30},
    ]

    d0303._annotate_ecf_inplace(waypoints)

    burn = ecf_module.NOMINAL_BURN_L_PER_S
    assert [waypoint["ecf"] for waypoint in waypoints] == pytest.approx(
        [0.0, 10.0 * burn, 20.0 * burn]
    )
    assert all(0.0 <= wp["ecf"] <= ecf_module.ECF_MAX_L for wp in waypoints)


def test_uav_speed_write_boundary_preserves_incoming_linesearch_eta(monkeypatch) -> None:
    from modules.mission_planning.MissionPlanner import runtime_settings

    monkeypatch.setattr(runtime_settings, "get_runtime_uav_speed_weight", lambda _default=1.0: 2.0)
    payload = {
        "pathID": 400000001,
        "aircraftID": 4,
        "waypointList": [
            _waypoint(0.0, speed_mps=40.0),
            _waypoint(
                0.0,
                speed_mps=40.0,
                sweep_x_m=[0.0, 100.0],
                search_speed_mps=10.0,
            ),
        ],
    }
    payload["waypointList"][0]["waypointID"] = 1
    payload["waypointList"][0]["nextWaypointID"] = 2
    payload["waypointList"][1]["waypointID"] = 2

    prepared = json_io.prepare_json_payload(
        Path("FlightPath") / "400000001.json",
        payload,
    )

    assert [waypoint["speed"] for waypoint in prepared["waypointList"]] == [58.0, 58.0]
    assert _etas(prepared["waypointList"]) == [0, 10]
