from __future__ import annotations

from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def _line_wp(lat: float, lon: float, coords: list[tuple[float, float]]) -> dict:
    return {
        "coordinate": {"latitude": lat, "longitude": lon, "altitude": 1000},
        "filmingProperty": {
            "fieldOfView": 5.7,
            "sensorType": 1,
            "operationMode": 2,
            "lineSearch": {
                "coordinateList": [
                    {"latitude": row_lat, "longitude": row_lon, "altitude": 100}
                    for row_lat, row_lon in coords
                ],
                "searchSpeed": 25.0,
            },
        },
    }


def test_trailing_short_line_search_wp_is_squashed() -> None:
    from modules.mission_planning.pipelines import next_collab_path_builder as builder

    waypoints = [
        _line_wp(37.0000, 128.0000, [(37.0000, 128.0000), (37.0005, 128.0005)]),
        _line_wp(37.0040, 128.0040, [(37.0040, 128.0040), (37.0045, 128.0045)]),
    ]

    builder._squash_trailing_short_line_search_waypoints(
        waypoints,
        spacing_m=2000.0,
        transit_speed_mps=40.0,
        fallback_search_speed_mps=25.0,
        speed_scale=1.0,
    )

    assert len(waypoints) == 1
    merged_coords = builder._line_search_coordinate_list(waypoints[0])
    assert len(merged_coords) == 4


def test_last_line_search_waypoint_snaps_to_route_endpoint() -> None:
    from modules.mission_planning.pipelines import next_collab_path_builder as builder

    route_coords = [
        {"latitude": 37.0000, "longitude": 128.0000, "altitude": 1000},
        {"latitude": 37.0000, "longitude": 128.0200, "altitude": 1000},
    ]
    route_xy = [builder.coord_to_xy(coord) for coord in route_coords]
    path_row = {
        "centerLineXY": route_xy,
        "lineRouteOffsetM": 100.0,
    }
    waypoints = [
        _line_wp(
            37.0007,
            128.0150,
            [
                (37.0000, 128.0180),
                (37.0000, 128.0190),
                (37.0000, 128.0200),
            ],
        )
    ]

    before_xy = builder.coord_to_xy(waypoints[-1]["coordinate"])
    before = builder._project_point_to_route_polyline_xy(before_xy, route_xy)
    assert before is not None
    assert before[2] < 0.9

    changed = builder._snap_last_line_search_waypoint_to_route_endpoint(
        waypoints,
        path_row=path_row,
        scan_lines_xy=None,
        altitude_fn=None,
    )

    after_xy = builder.coord_to_xy(waypoints[-1]["coordinate"])
    after = builder._project_point_to_route_polyline_xy(after_xy, route_xy)
    assert changed == 1
    assert after is not None
    assert after[2] > 0.99


def test_regenerated_line_sweep_items_always_include_tail() -> None:
    from modules.mission_planning.pipelines import next_collab_path_builder as builder

    scan_lines_xy = [
        [(float(x_m), -100.0), (float(x_m), 100.0)]
        for x_m in (0.0, 500.0, 1000.0, 1500.0, 2000.0, 2300.0)
    ]
    path_row = {
        "centerLineXY": [(0.0, 0.0), (2300.0, 0.0)],
        "lineRouteWpSpacingM": 2000.0,
        "lineRouteOffsetM": 100.0,
    }

    items = builder._line_sweep_items_from_scan_lines(
        path_row,
        scan_lines_xy,
        reference_xy=(0.0, -500.0),
    )

    assert items
    assert int(items[-1]["sweepIndex"]) == len(scan_lines_xy) - 1
    assert len(items) >= 2
    assert int(items[-2]["sweepIndex"]) < len(scan_lines_xy) - 1

    tail_projection = builder._project_point_to_route_polyline_xy(
        items[-1]["anchorXY"],
        path_row["centerLineXY"],
    )
    assert tail_projection is not None
    assert tail_projection[2] > 0.99


if __name__ == "__main__":
    test_trailing_short_line_search_wp_is_squashed()
    test_last_line_search_waypoint_snaps_to_route_endpoint()
    test_regenerated_line_sweep_items_always_include_tail()
    print("ok")
