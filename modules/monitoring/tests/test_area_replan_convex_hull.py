# -*- coding: utf-8 -*-
"""AREA top-level replan geometry must use a bounded convex envelope."""
from __future__ import annotations

from types import SimpleNamespace

from pyproj import Transformer
from shapely.geometry import Polygon

from modules.monitoring.logic.mission_area_progress_monitor import (
    _per_assignment_area_replan_geometry,
    _stable_input_area_geometry,
    _top_level_area_replan_geometry,
)


def _area_row(coords: list[tuple[float, float]]) -> dict:
    return {
        "isHole": False,
        "coordinateList": [
            {"longitude": float(lon), "latitude": float(lat)}
            for lon, lat in coords
        ],
    }


def test_connected_stair_step_is_always_reduced_to_its_convex_hull() -> None:
    remaining = Polygon(
        [
            (0, 0),
            (8, 0),
            (8, 2),
            (5, 2),
            (5, 4),
            (7, 4),
            (7, 6),
            (2, 6),
            (2, 3),
            (0, 3),
        ]
    )
    immutable_input_area = Polygon([(-1, -1), (9, -1), (9, 7), (-1, 7)])

    result = _top_level_area_replan_geometry(
        remaining,
        immutable_input_area,
        area_threshold_m2=0.01,
        bridge_gap_m=35.0,
    )

    assert result.equals(remaining.convex_hull)
    assert len(result.exterior.coords) < len(remaining.exterior.coords)


def test_convex_envelope_never_leaves_the_input_area() -> None:
    remaining = Polygon(
        [
            (0, 0),
            (6, 0),
            (6, 1),
            (1, 1),
            (1, 6),
            (0, 6),
        ]
    )
    immutable_input_area = Polygon(
        [
            (0, 0),
            (5, 0),
            (5, 2),
            (2, 2),
            (2, 5),
            (0, 5),
        ]
    )

    result = _top_level_area_replan_geometry(
        remaining.intersection(immutable_input_area),
        immutable_input_area,
        area_threshold_m2=0.01,
        bridge_gap_m=35.0,
    )

    assert result.difference(immutable_input_area).is_empty
    assert result.area <= immutable_input_area.area


def test_stable_hull_cap_comes_from_input_area_not_replanned_assignment() -> None:
    transformer = Transformer.from_crs(
        "EPSG:4326",
        "EPSG:32652",
        always_xy=True,
    )
    input_coords = [
        (127.300, 38.000),
        (127.340, 38.000),
        (127.340, 38.030),
        (127.300, 38.030),
    ]
    state = SimpleNamespace(
        coverage_def=SimpleNamespace(transformer=transformer),
        input_area_list=[_area_row(input_coords)],
        input_coordinate_list=[],
        assignment_geometry=Polygon(
            [(0, 0), (100, 0), (100, 20), (20, 20), (20, 100), (0, 100)]
        ),
    )

    result = _stable_input_area_geometry([state])

    expected_xy = Polygon([transformer.transform(*coord) for coord in input_coords])
    assert result.equals_exact(expected_xy, 0.01)
    assert result.convex_hull.equals(result)


def test_completed_child_between_active_children_is_not_restored_by_hull() -> None:
    immutable_input_area = Polygon([(0, 0), (12, 0), (12, 8), (0, 8)])
    left_assignment = Polygon([(0, 0), (4, 0), (4, 8), (0, 8)])
    completed_assignment = Polygon([(4, 0), (8, 0), (8, 8), (4, 8)])
    right_assignment = Polygon([(8, 0), (12, 0), (12, 8), (8, 8)])
    left_remaining = Polygon([(0, 0), (4, 0), (4, 6), (2, 8), (0, 8)])
    right_remaining = Polygon([(8, 0), (12, 0), (12, 8), (10, 8), (8, 6)])

    result = _per_assignment_area_replan_geometry(
        [
            (left_remaining, left_assignment),
            # A completed child contributes no remaining geometry.
            (Polygon(), completed_assignment),
            (right_remaining, right_assignment),
        ],
        area_threshold_m2=0.01,
        bridge_gap_m=35.0,
    )

    assert result.covers(left_remaining)
    assert result.covers(right_remaining)
    assert result.intersection(completed_assignment).area == 0.0
    assert result.difference(immutable_input_area).is_empty

    # This is the exact regression: one global hull would refill the completed
    # middle child even though its own remaining geometry is empty.
    global_result = _top_level_area_replan_geometry(
        left_remaining.union(right_remaining),
        immutable_input_area,
        area_threshold_m2=0.01,
        bridge_gap_m=35.0,
    )
    assert global_result.intersection(completed_assignment).area > 0.0
