from __future__ import annotations

from modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304 import (
    d0304,
)


def _coordinate_mission() -> dict:
    return {
        "aircraftID": 1,
        "individualMissionID": 900000001,
        "pathID": 100000001,
        "individualMissionInfo": {
            "individualMissionType": 9,
            "coordinateList": [
                {"latitude": 37.0, "longitude": 127.0, "altitude": 999}
            ],
        },
    }


def test_nonattack_sparse_altitude_is_not_looked_up_when_profile_exists(monkeypatch) -> None:
    scalar_calls: list[tuple[float, float]] = []
    monkeypatch.setattr(
        d0304,
        "_lah_non_attack_altitude_m",
        lambda lat, lon: scalar_calls.append((lat, lon)) or 777,
    )
    monkeypatch.setattr(
        d0304,
        "build_lah_terrain_following_path",
        lambda *_args, **_kwargs: [
            {
                "latitude": 37.0,
                "longitude": 127.0,
                "altitude": 321,
                "cum_m": 0.0,
            }
        ],
    )

    [packet] = d0304.build_lah_flight_plans_fixed(
        [_coordinate_mission()],
        wp_alloc=d0304._WPAllocator(start=100, end=200),
    )

    assert scalar_calls == []
    assert packet["lahWaypointList"][0]["coordinate"]["altitude"] == 321


def test_nonattack_sparse_altitude_is_restored_when_profile_is_empty(monkeypatch) -> None:
    scalar_calls: list[tuple[float, float]] = []
    monkeypatch.setattr(
        d0304,
        "_lah_non_attack_altitude_m",
        lambda lat, lon: scalar_calls.append((lat, lon)) or 654,
    )
    monkeypatch.setattr(
        d0304,
        "build_lah_terrain_following_path",
        lambda *_args, **_kwargs: [],
    )

    [packet] = d0304.build_lah_flight_plans_fixed(
        [_coordinate_mission()],
        wp_alloc=d0304._WPAllocator(start=100, end=200),
    )

    assert scalar_calls == [(37.0, 127.0)]
    assert packet["lahWaypointList"][0]["coordinate"]["altitude"] == 654
