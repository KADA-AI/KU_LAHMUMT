from __future__ import annotations

import math

from modules.mission_planning.MissionPlanner.data_def.lah_terrain_path import (
    LAH_LOW_LEVEL_CLEARANCE_M,
    LAH_TERRAIN_MAX_WAYPOINT_SPACING_M,
    LAH_TERRAIN_MAX_OUTPUT_WAYPOINTS,
    build_lah_terrain_following_path,
)


START_LON = 127.0
END_LON = 127.018


def _terrain_for_fraction(fraction: float) -> float:
    # Two ridges with a valley between them exercise both collision avoidance
    # and the "do not float far above the DEM" simplification rule.
    ridge_one = max(0.0, 180.0 - abs(fraction - 0.32) * 900.0)
    ridge_two = max(0.0, 120.0 - abs(fraction - 0.76) * 750.0)
    return 100.0 + max(ridge_one, ridge_two)


def _terrain_provider(coords):
    values = []
    for _latitude, longitude in coords:
        fraction = (float(longitude) - START_LON) / (END_LON - START_LON)
        values.append(_terrain_for_fraction(fraction))
    return values


def test_profile_waypoints_clear_every_dense_dem_sample() -> None:
    path = build_lah_terrain_following_path(
        [(37.0, START_LON), (37.0, END_LON)],
        terrain_provider=_terrain_provider,
        sample_spacing_m=5.0,
        max_waypoint_spacing_m=400.0,
        max_profile_excess_m=12.0,
    )

    assert len(path) > 2
    assert path[0]["longitude"] == START_LON
    assert path[-1]["longitude"] == END_LON

    for sample_index in range(101):
        fraction = sample_index / 100.0
        longitude = START_LON + (END_LON - START_LON) * fraction
        left, right = next(
            (a, b)
            for a, b in zip(path, path[1:])
            if float(a["longitude"]) - 1e-10 <= longitude <= float(b["longitude"]) + 1e-10
        )
        span = float(right["longitude"]) - float(left["longitude"])
        local_fraction = 0.0 if abs(span) < 1e-12 else (longitude - float(left["longitude"])) / span
        planned_altitude = float(left["altitude"]) + (
            float(right["altitude"]) - float(left["altitude"])
        ) * local_fraction
        required_altitude = _terrain_for_fraction(fraction) + LAH_LOW_LEVEL_CLEARANCE_M
        # check allows a small interpolation tolerance for the triangular test DEM.
        # The route is certified at dense DEM samples.  This independent check
        # allows a small interpolation tolerance for the triangular test DEM.
        # check allows a small interpolation tolerance for the triangular test DEM.
        assert planned_altitude + 2.0 >= required_altitude


def test_route_corner_is_preserved() -> None:
    corner = (37.005, 127.005)
    path = build_lah_terrain_following_path(
        [(37.0, 127.0), corner, (37.01, 127.0)],
        terrain_provider=lambda coords: [100.0 for _ in coords],
    )

    assert any(
        math.isclose(float(point["latitude"]), corner[0], abs_tol=1e-7)
        and math.isclose(float(point["longitude"]), corner[1], abs_tol=1e-7)
        for point in path
    )
    assert all(
        int(point["altitude"]) == int(100 + LAH_LOW_LEVEL_CLEARANCE_M)
        for point in path
    )


def test_single_point_uses_planning_clearance() -> None:
    path = build_lah_terrain_following_path(
        [{"latitude": 37.1, "longitude": 127.1, "altitude": 9999}],
        terrain_provider=lambda coords: [123.2 for _ in coords],
    )

    assert len(path) == 1
    assert path[0]["altitude"] == math.ceil(123.2 + LAH_LOW_LEVEL_CLEARANCE_M)


def test_adaptive_waypoints_respect_max_spacing_and_add_terrain_detail() -> None:
    flat_path = build_lah_terrain_following_path(
        [(37.0, 127.0), (37.0, 127.0563)],
        terrain_provider=lambda coords: [100.0 for _ in coords],
    )

    def rugged_terrain_provider(coords):
        values = []
        for _latitude, longitude in coords:
            fraction = (float(longitude) - 127.0) / 0.0563
            ridge_one = max(0.0, 300.0 - abs(fraction - 0.31) * 3000.0)
            ridge_two = max(0.0, 220.0 - abs(fraction - 0.72) * 2400.0)
            values.append(100.0 + max(ridge_one, ridge_two))
        return values

    rugged_path = build_lah_terrain_following_path(
        [(37.0, 127.0), (37.0, 127.0563)],
        terrain_provider=rugged_terrain_provider,
    )

    gaps = [
        float(right["cum_m"]) - float(left["cum_m"])
        for left, right in zip(flat_path, flat_path[1:])
    ]
    assert gaps
    assert max(gaps) <= LAH_TERRAIN_MAX_WAYPOINT_SPACING_M + 1e-6
    assert len(rugged_path) > len(flat_path)


def test_extreme_profile_is_safely_bounded_for_normal_operational_length() -> None:
    path = build_lah_terrain_following_path(
        [(37.0, 127.0), (37.0, 127.10)],
        terrain_provider=lambda coords: [
            100.0 + (500.0 if index % 2 else 0.0)
            for index, _coordinate in enumerate(coords)
        ],
    )

    assert len(path) <= LAH_TERRAIN_MAX_OUTPUT_WAYPOINTS
    assert max(
        float(right["cum_m"]) - float(left["cum_m"])
        for left, right in zip(path, path[1:])
    ) <= LAH_TERRAIN_MAX_WAYPOINT_SPACING_M + 1e-6
    assert all(
        int(point["altitude"]) >= int(600 + LAH_LOW_LEVEL_CLEARANCE_M)
        for point in path
    )


def test_low_terrain_option_detours_through_nearby_valley() -> None:
    start = (37.0, 127.0)
    end = (37.0, 127.06)

    def valley_terrain_provider(coords):
        # The direct east-west route crosses a high plateau.  A broad valley
        # sits a few hundred metres north, well inside the bounded corridor.
        return [80.0 if float(latitude) >= 37.004 else 900.0 for latitude, _ in coords]

    direct_path = build_lah_terrain_following_path(
        [start, end],
        terrain_provider=valley_terrain_provider,
    )
    valley_path = build_lah_terrain_following_path(
        [start, end],
        terrain_provider=valley_terrain_provider,
        prefer_low_terrain=True,
    )

    assert all(math.isclose(float(point["latitude"]), start[0]) for point in direct_path)
    assert max(float(point["latitude"]) for point in valley_path) >= 37.004
    assert min(int(point["altitude"]) for point in valley_path) == int(
        80 + LAH_LOW_LEVEL_CLEARANCE_M
    )
    assert math.isclose(float(valley_path[0]["latitude"]), start[0], abs_tol=1e-7)
    assert math.isclose(float(valley_path[0]["longitude"]), start[1], abs_tol=1e-7)
    assert math.isclose(float(valley_path[-1]["latitude"]), end[0], abs_tol=1e-7)
    assert math.isclose(float(valley_path[-1]["longitude"]), end[1], abs_tol=1e-7)
    assert float(valley_path[-1]["cum_m"]) < float(direct_path[-1]["cum_m"]) * 1.1


def test_requested_narrow_corridor_is_not_expanded() -> None:
    start = (37.0, 127.0)
    end = (37.0, 127.06)

    path = build_lah_terrain_following_path(
        [start, end],
        terrain_provider=lambda coords: [
            10.0 if float(latitude) > 37.0005 else 900.0
            for latitude, _longitude in coords
        ],
        prefer_low_terrain=True,
        low_terrain_corridor_m=75.0,
    )
    max_offset_m = max(
        abs((float(point["latitude"]) - start[0]) * 111_132.92)
        for point in path
    )

    assert 30.0 < max_offset_m <= 76.0
