from __future__ import annotations

import math

from modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304 import (
    d0304,
)
from modules.mission_planning.pipelines import lah_operational_mode as lah_mode


def _coord(latitude: float, longitude: float) -> dict:
    return {"latitude": latitude, "longitude": longitude}


def _mission(aircraft_id: int, path_id: int, info: dict) -> dict:
    return {
        "aircraftID": aircraft_id,
        "pathID": path_id,
        "_lahPreserveLineEndpoints": True,
        "individualMissionInfo": info,
    }


def _route_coordinates(packet: dict) -> list[tuple[float, float]]:
    return [
        (
            float(waypoint["coordinate"]["latitude"]),
            float(waypoint["coordinate"]["longitude"]),
        )
        for waypoint in packet["lahWaypointList"]
    ]


def _segment_cross_track_m(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    metres_per_lat = 111_132.92
    metres_per_lon = metres_per_lat * math.cos(math.radians((start[0] + end[0]) * 0.5))
    px = (point[1] - start[1]) * metres_per_lon
    py = (point[0] - start[0]) * metres_per_lat
    dx = (end[1] - start[1]) * metres_per_lon
    dy = (end[0] - start[0]) * metres_per_lat
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-9:
        return math.hypot(px, py)
    fraction = max(0.0, min(1.0, (px * dx + py * dy) / length_sq))
    return math.hypot(px - dx * fraction, py - dy * fraction)


def test_mission_checker_enforces_line_width_and_area_holes() -> None:
    line_info = {
        "lineList": [
            {
                "width": 400,
                "coordinateList": [_coord(37.0, 127.0), _coord(37.0, 127.06)],
            }
        ],
        "areaList": [],
    }
    line_checker = d0304._mission_low_terrain_segment_checker(line_info)
    assert line_checker is not None
    assert line_checker((37.001, 127.01), (37.001, 127.05))
    assert not line_checker((37.003, 127.01), (37.003, 127.05))

    # Low-terrain candidates retain 20% of the LINE half-width as turn/rounding
    # reserve instead of riding directly on the declared edge.
    wide_line_checker = d0304._mission_low_terrain_segment_checker(
        {
            "lineList": [
                {
                    "width": 1000,
                    "coordinateList": [_coord(37.0, 127.0), _coord(37.0, 127.06)],
                }
            ]
        }
    )
    assert wide_line_checker is not None
    assert wide_line_checker((37.0035, 127.01), (37.0035, 127.05))
    assert not wide_line_checker((37.0042, 127.01), (37.0042, 127.05))

    area_info = {
        "lineList": [],
        "areaList": [
            {
                "isHole": False,
                "coordinateList": [
                    _coord(36.99, 127.0),
                    _coord(36.99, 127.06),
                    _coord(37.02, 127.06),
                    _coord(37.02, 127.0),
                ],
            },
            {
                "isHole": True,
                "coordinateList": [
                    _coord(37.004, 127.02),
                    _coord(37.004, 127.04),
                    _coord(37.012, 127.04),
                    _coord(37.012, 127.02),
                ],
            },
        ],
    }
    area_checker = d0304._mission_low_terrain_segment_checker(area_info)
    assert area_checker is not None
    assert area_checker((37.0, 127.005), (37.0, 127.055))
    assert not area_checker((37.008, 127.005), (37.008, 127.055))
    assert not area_checker((37.025, 127.005), (37.025, 127.055))
    assert not area_checker((37.0195, 127.005), (37.0195, 127.055))


def test_line_geometry_survives_lah_conversion_and_limits_detour(monkeypatch) -> None:
    source_mission = {
        "inputMissionID": 6,
        "inputMissionType": 1,
        "regionType": 2,
        "missionDetail": {
            "lineList": [
                {
                    "width": 1000,
                    "coordinateList": [_coord(37.0, 127.0), _coord(37.0, 127.06)],
                }
            ]
        },
    }
    info = lah_mode._line_mission_info(source_mission, preserve_endpoints=True)

    assert info is not None
    assert info["lineList"][0]["width"] == 1000
    assert type(info["lineList"][0]["width"]) is int
    assert info["lineList"] is not source_mission["missionDetail"]["lineList"]

    def terrain_profile(coords):
        # Make terrain beyond the usable 400 m half-width very attractive. The
        # planner must still keep every output point inside that inner corridor.
        values = []
        for latitude, _longitude in coords:
            north_m = (float(latitude) - 37.0) * 111_132.92
            values.append(10.0 if north_m > 410.0 else 900.0)
        return values

    monkeypatch.setattr(d0304, "_terrain_profile_many", terrain_profile)
    [packet] = d0304.build_lah_flight_plans_fixed(
        [_mission(1, 100_000_100, info)]
    )
    route = _route_coordinates(packet)
    max_cross_track_m = max(
        abs((latitude - 37.0) * 111_132.92) for latitude, _longitude in route
    )

    assert max_cross_track_m <= 401.0
    checker = d0304._mission_low_terrain_segment_checker(info)
    assert checker is not None
    assert all(checker(start, end) for start, end in zip(route, route[1:]))


def test_merged_lah_egress_widths_are_icd_uint_json_values() -> None:
    missions = [
        {
            "inputMissionID": 4,
            "inputMissionType": 1,
            "regionType": 3,
            "missionDetail": {
                "lineList": [
                    {
                        "width": 1000,
                        "coordinateList": [_coord(37.0, 127.0), _coord(37.01, 127.01)],
                    }
                ]
            },
        },
        {
            "inputMissionID": 5,
            "inputMissionType": 1,
            "regionType": 2,
            "missionDetail": {
                "lineList": [
                    {
                        # A decoded numeric source may already be float; the
                        # emitted ICD geometry still has to be JSON uint.
                        "width": 1000.0,
                        "coordinateList": [_coord(37.01, 127.01), _coord(37.02, 127.02)],
                    }
                ]
            },
        },
    ]

    info = lah_mode._merged_line_mission_info(missions, [0, 1])

    assert info is not None
    widths = [line["width"] for line in info["lineList"]]
    assert widths == [1000, 1000]
    assert all(type(width) is int for width in widths)


def test_non_finite_line_widths_do_not_crash_lah_planning() -> None:
    for width in (math.nan, math.inf, -math.inf):
        mission = {
            "inputMissionID": 5,
            "inputMissionType": 1,
            "regionType": 2,
            "missionDetail": {
                "lineList": [
                    {
                        "width": width,
                        "coordinateList": [_coord(37.0, 127.0), _coord(37.01, 127.01)],
                    }
                ]
            },
        }

        assert lah_mode._constraint_line_rows(mission) == []


def test_geometryless_line_keeps_original_horizontal_course(monkeypatch) -> None:
    info = {
        "individualMissionType": 7,
        "coordinateList": [_coord(37.0, 127.0), _coord(37.0, 127.06)],
        "_lahPreserveLineEndpoints": True,
    }

    monkeypatch.setattr(
        d0304,
        "_terrain_profile_many",
        lambda coords: [10.0 if float(latitude) > 37.002 else 900.0 for latitude, _ in coords],
    )
    [packet] = d0304.build_lah_flight_plans_fixed(
        [_mission(1, 100_000_103, info)]
    )

    assert all(
        abs(float(latitude) - 37.0) <= 1e-7
        for latitude, _longitude in _route_coordinates(packet)
    )


def test_width_1000_diagonal_line_keeps_margin_after_formation_offset(monkeypatch) -> None:
    start = (37.780078966276676, 128.23788428886212)
    end = (37.718752718082875, 128.17829923794753)
    source_mission = {
        "inputMissionID": 6,
        "inputMissionType": 1,
        "missionDetail": {
            "lineList": [
                {
                    "width": 1000,
                    "coordinateList": [
                        _coord(start[0], start[1]),
                        _coord(end[0], end[1]),
                    ],
                }
            ]
        },
    }
    info = lah_mode._line_mission_info(source_mission, preserve_endpoints=True)
    assert info is not None

    monkeypatch.setattr(
        d0304,
        "_terrain_profile_many",
        lambda coords: [
            10.0
            if _segment_cross_track_m(
                (float(latitude), float(longitude)), start, end
            )
            > 300.0
            else 900.0
            for latitude, longitude in coords
        ],
    )
    [packet] = d0304.build_lah_flight_plans_fixed(
        [_mission(2, 200_000_010, info)]
    )
    cross_track = [
        _segment_cross_track_m(point, start, end)
        for point in _route_coordinates(packet)
    ]

    assert max(cross_track) <= 401.0
    assert sum(distance > 500.0 for distance in cross_track) == 0


def test_area_geometry_survives_point_conversion_as_private_constraint() -> None:
    source_mission = {
        "inputMissionID": 7,
        "inputMissionType": 2,
        "missionDetail": {
            "areaList": [
                {
                    "isHole": False,
                    "coordinateList": [
                        _coord(37.0, 127.0),
                        _coord(37.0, 127.02),
                        _coord(37.02, 127.02),
                        _coord(37.02, 127.0),
                    ],
                }
            ]
        },
    }
    info = lah_mode._generic_lah_info_from_input(source_mission)

    assert info is not None
    assert "areaList" not in info
    assert len(info["_lahConstraintAreaList"]) == 1
    checker = d0304._mission_low_terrain_segment_checker(info)
    assert checker is not None
    assert checker((37.01, 127.01), (37.01, 127.01))
    assert not checker((37.0002, 127.01), (37.0002, 127.01))


def test_line_low_terrain_route_stays_inside_declared_width(monkeypatch) -> None:
    info = {
        "individualMissionType": 6,
        "coordinateList": [_coord(37.0, 127.0), _coord(37.0, 127.06)],
        "lineList": [
            {
                "width": 400,
                "coordinateList": [_coord(37.0, 127.0), _coord(37.0, 127.06)],
            }
        ],
        "areaList": [],
    }

    def terrain_profile(coords):
        values = []
        for latitude, _longitude in coords:
            north_m = (float(latitude) - 37.0) * 111_132.92
            if north_m > 220.0:
                values.append(20.0)  # Lower, but outside LINE width.
            elif north_m > 60.0:
                values.append(120.0)  # Usable valley inside the corridor.
            else:
                values.append(900.0)
        return values

    monkeypatch.setattr(d0304, "_terrain_profile_many", terrain_profile)

    [packet] = d0304.build_lah_flight_plans_fixed([_mission(1, 100_000_101, info)])
    route = _route_coordinates(packet)
    checker = d0304._mission_low_terrain_segment_checker(info)

    assert checker is not None
    assert max((latitude - 37.0) * 111_132.92 for latitude, _ in route) > 60.0
    assert all(checker(start, end) for start, end in zip(route, route[1:]))


def test_area_low_terrain_route_does_not_enter_hole(monkeypatch) -> None:
    route = [_coord(37.0, 127.005), _coord(37.0, 127.055)]
    info = {
        "individualMissionType": 5,
        "coordinateList": route,
        "lineList": [],
        "areaList": [
            {
                "isHole": False,
                "coordinateList": [
                    _coord(36.99, 127.0),
                    _coord(36.99, 127.06),
                    _coord(37.02, 127.06),
                    _coord(37.02, 127.0),
                ],
            },
            {
                "isHole": True,
                "coordinateList": [
                    _coord(37.004, 127.02),
                    _coord(37.004, 127.04),
                    _coord(37.012, 127.04),
                    _coord(37.012, 127.02),
                ],
            },
        ],
    }

    def terrain_profile(coords):
        return [
            20.0
            if 37.004 <= float(latitude) <= 37.012 and 127.02 <= float(longitude) <= 127.04
            else 800.0
            for latitude, longitude in coords
        ]

    monkeypatch.setattr(d0304, "_terrain_profile_many", terrain_profile)

    [packet] = d0304.build_lah_flight_plans_fixed([_mission(1, 100_000_102, info)])
    planned_route = _route_coordinates(packet)
    checker = d0304._mission_low_terrain_segment_checker(info)

    assert checker is not None
    assert all(checker(start, end) for start, end in zip(planned_route, planned_route[1:]))


def test_narrow_line_suppresses_formation_offset_outside_width(monkeypatch) -> None:
    info = {
        "individualMissionType": 6,
        "coordinateList": [_coord(37.0, 127.0), _coord(37.0, 127.03)],
        "lineList": [
            {
                "width": 100,
                "coordinateList": [_coord(37.0, 127.0), _coord(37.0, 127.03)],
            }
        ],
        "areaList": [],
    }
    monkeypatch.setattr(
        d0304,
        "_terrain_profile_many",
        lambda coords: [100.0 for _ in coords],
    )

    [packet] = d0304.build_lah_flight_plans_fixed([_mission(2, 200_000_201, info)])
    route = _route_coordinates(packet)
    checker = d0304._mission_low_terrain_segment_checker(info)

    assert checker is not None
    assert all(checker(start, end) for start, end in zip(route, route[1:]))
    assert max(abs((latitude - 37.0) * 111_132.92) for latitude, _ in route) < 1.0
