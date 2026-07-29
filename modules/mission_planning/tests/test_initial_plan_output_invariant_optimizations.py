from __future__ import annotations

import math
from types import SimpleNamespace

from shapely.geometry import LineString, Point

from modules.mission_planning.MissionPlanner import runtime_settings
from modules.mission_planning.MissionPlanner.data_def import (
    lah_terminal_cover,
    lah_terrain_path,
)
from modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304 import (
    d0303,
    d0304,
)


def test_d0303_physics_selection_reuses_the_build_runtime_payload(monkeypatch) -> None:
    payload = {"values": {"physics_fov_selection_enabled": True}}
    calls: list[tuple[str, object]] = []

    physics = SimpleNamespace(
        physics_route_offset_cap_m=lambda _fov, _sep, *, runtime_cfg=None: (
            calls.append(("route", runtime_cfg)) or 123.0
        ),
        physics_line_row=lambda _width, _sep, *, runtime_cfg=None: (
            calls.append(("line", runtime_cfg))
            or {"fov": 4.2, "sep": 123.0, "width": 500.0, "vel": 120.0}
        ),
        max_sweep_row_chord_m_xy=lambda _poly, _bearing: 700.0,
        physics_area_fov_deg=lambda *, row_length_m, runtime_cfg=None: (
            calls.append(("area", runtime_cfg)) or 4.3
        ),
    )
    monkeypatch.setattr(d0303, "_capture_physics", physics)
    monkeypatch.setattr(d0303, "_runtime_settings_payload", lambda: payload)
    monkeypatch.setattr(d0303, "USE_DB_FOR_CORRIDOR", True)
    monkeypatch.setattr(d0303, "_runtime_area_auto_fov_from_db", lambda: True)
    monkeypatch.setattr(d0303, "_runtime_area_review_max_segment_m", lambda: 0.0)

    assert d0303._route_offset_sep_for_fov(4.2, 500.0) == 123.0
    assert d0303._select_corridor_db_config(500.0, 123.0)["fov"] == 4.2
    area = d0303._select_area_db_config(
        [(38.0, 127.0), (38.0, 127.01), (38.01, 127.01)],
        0.0,
    )

    assert area is not None and area["config"]["fov"] == 4.3
    assert calls == [
        ("route", payload),
        ("line", payload),
        ("area", payload),
    ]


def test_linesearch_coordinate_limit_uses_the_existing_runtime_cache(
    monkeypatch,
) -> None:
    monkeypatch.delenv(
        "MISSION_PLAN_MAX_LINESEARCH_COORDS_PER_WAYPOINT",
        raising=False,
    )
    monkeypatch.setattr(
        d0303,
        "_runtime_setting_values",
        lambda: {"max_linesearch_coords_per_waypoint": "321"},
    )

    def _unexpected_uncached_read(*_args, **_kwargs):
        raise AssertionError("the build-local runtime cache must be used")

    monkeypatch.setattr(d0303, "_get_runtime_value", _unexpected_uncached_read)
    assert d0303._max_linesearch_coords_per_waypoint() == 321


def test_lah_terrain_runtime_snapshot_matches_the_legacy_runtime_lookup() -> None:
    payload = {
        "values": {
            "lah_low_terrain_strength": 1.0,
            "lah_altitude_smoothing_enabled": True,
            "lah_altitude_short_dip_max_depth_m": 30.0,
            "lah_altitude_short_dip_max_span_m": 1200.0,
            "lah_altitude_redundant_tolerance_m": 3.0,
        }
    }
    route = [(38.0, 127.0), (38.02, 127.0)]

    def flat_terrain(coords):
        return [100.0 for _ in coords]

    with runtime_settings.runtime_override(payload):
        legacy = lah_terrain_path.build_lah_terrain_following_path(
            route,
            terrain_provider=flat_terrain,
            prefer_low_terrain=True,
        )
    explicit = lah_terrain_path.build_lah_terrain_following_path(
        route,
        terrain_provider=flat_terrain,
        prefer_low_terrain=True,
        runtime_payload=payload,
    )

    assert explicit == legacy


def test_d0304_passes_one_runtime_snapshot_to_the_terrain_builder(
    monkeypatch,
) -> None:
    payload = {"values": {"lah_low_terrain_strength": 1.0}}
    received: list[object] = []

    def fake_terrain_builder(_coordinates, **kwargs):
        received.append(kwargs.get("runtime_payload"))
        return [
            {
                "latitude": 38.0,
                "longitude": 127.0,
                "altitude": 130,
                "cum_m": 0.0,
            }
        ]

    monkeypatch.setattr(
        d0304,
        "build_lah_terrain_following_path",
        fake_terrain_builder,
    )
    result = d0304._terrain_follow_non_attack_waypoints(
        [
            {
                "coordinate": {
                    "latitude": 38.0,
                    "longitude": 127.0,
                    "altitude": 0,
                }
            }
        ],
        cruise_speed=40.0,
        runtime_payload=payload,
    )

    assert received == [payload]
    assert result[0]["coordinate"]["altitude"] == 130


def test_prepared_lah_segment_checker_keeps_exact_covers_semantics() -> None:
    start = (38.0, 127.0)
    end = (38.01, 127.0)
    info = {
        "lineList": [
            {
                "width": 200.0,
                "coordinateList": [
                    {"latitude": start[0], "longitude": start[1]},
                    {"latitude": end[0], "longitude": end[1]},
                ],
            }
        ]
    }
    checker = d0304._mission_low_terrain_segment_checker(info)
    assert checker is not None

    origin_lat = (start[0] + end[0]) / 2.0
    origin_lon = (start[1] + end[1]) / 2.0
    metres_per_lat = 111_132.92
    metres_per_lon = metres_per_lat * math.cos(math.radians(origin_lat))

    def xy(point):
        return (
            (point[1] - origin_lon) * metres_per_lon,
            (point[0] - origin_lat) * metres_per_lat,
        )

    raw_geometry = LineString([xy(start), xy(end)]).buffer(
        200.0 * 0.5 * d0304.LAH_LOW_TERRAIN_BOUNDARY_KEEP_RATIO,
        cap_style=2,
        join_style=1,
    )
    samples = [
        (start, end),
        ((38.002, 127.0002), (38.008, 127.0002)),
        ((38.002, 127.002), (38.008, 127.002)),
        ((38.005, 127.0), (38.005, 127.0)),
    ]

    for left, right in samples:
        expected = (
            raw_geometry.covers(Point(xy(left)))
            if d0304._dist_ll_m(left, right) <= 0.01
            else raw_geometry.covers(LineString([xy(left), xy(right)]))
        )
        assert checker(left, right) is bool(expected)


def test_incremental_cover_candidate_spacing_matches_legacy_selection() -> None:
    from shapely.geometry import Point, Polygon

    geometry = Polygon(
        [
            (0.0, 0.0),
            (8_000.0, 0.0),
            (8_000.0, 5_000.0),
            (4_500.0, 3_000.0),
            (0.0, 5_000.0),
        ],
        holes=[
            [
                (3_000.0, 1_500.0),
                (4_000.0, 1_500.0),
                (4_000.0, 2_500.0),
                (3_000.0, 2_500.0),
            ]
        ],
    )
    fallback_xy = (1_000.0, 1_000.0)
    budget = 121

    def legacy_candidate_points():
        seeds = []

        def add_geometry_point(point):
            if point is not None and not point.is_empty and geometry.covers(point):
                seeds.append((float(point.x), float(point.y)))

        add_geometry_point(geometry.representative_point())
        add_geometry_point(geometry.centroid)
        for part in sorted(
            lah_terminal_cover._polygon_parts(geometry),
            key=lambda item: (
                -float(item.area),
                tuple(float(value) for value in item.bounds),
            ),
        ):
            add_geometry_point(part.representative_point())
            add_geometry_point(part.centroid)
        add_geometry_point(Point(*fallback_xy))

        selected = lah_terminal_cover._dedupe_xy(seeds)[:budget]
        min_x, min_y, max_x, max_y = (float(value) for value in geometry.bounds)
        grid_side = max(
            7,
            min(25, int(math.ceil(math.sqrt(budget * 8.0)))),
        )
        grid_points = []
        for row in range(grid_side):
            y = min_y + (row + 0.5) * (max_y - min_y) / grid_side
            for column in range(grid_side):
                x = min_x + (column + 0.5) * (max_x - min_x) / grid_side
                if geometry.covers(Point(x, y)):
                    grid_points.append((float(x), float(y)))
        pool = [
            point
            for point in lah_terminal_cover._dedupe_xy(grid_points)
            if point not in selected
        ]
        pool.sort(key=lambda point: (point[1], point[0]))
        while pool and len(selected) < budget:
            if not selected:
                chosen_index = 0
            else:
                chosen_index = max(
                    range(len(pool)),
                    key=lambda index: min(
                        (pool[index][0] - current[0]) ** 2
                        + (pool[index][1] - current[1]) ** 2
                        for current in selected
                    ),
                )
            selected.append(pool.pop(chosen_index))
        return selected[:budget]

    assert lah_terminal_cover._candidate_points(
        geometry,
        fallback_xy,
        budget,
    ) == legacy_candidate_points()
