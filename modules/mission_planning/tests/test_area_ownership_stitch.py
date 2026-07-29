# -*- coding: utf-8 -*-
"""잔여 AREA의 제한형 convex-hull 소유영역 계약."""
from __future__ import annotations

from copy import deepcopy
from unittest.mock import patch

from shapely.geometry import Polygon
from shapely.ops import unary_union

from modules.mission_planning.pipelines.area_ownership_stitch import (
    convex_hull_area_fragments_xy,
)
from modules.mission_planning.replanning.triggers.next_collab import (
    pipeline as next_collab_pipeline,
)
from modules.mission_planning.replanning.triggers.next_collab.pipeline import (
    _area_planner_components_from_detail,
    _coord_list_to_polygon_xy,
    _single_area_ownership_component,
)


def rect(x: float, y: float, w: float, h: float) -> Polygon:
    return Polygon([(x, y), (x + w, y), (x + w, y + h), (x, y + h)])


def area_row(lat0: float, lon0: float, dlat: float, dlon: float) -> dict:
    return {
        "isHole": False,
        "coordinateList": [
            {"latitude": lat0, "longitude": lon0},
            {"latitude": lat0, "longitude": lon0 + dlon},
            {"latitude": lat0 + dlat, "longitude": lon0 + dlon},
            {"latitude": lat0 + dlat, "longitude": lon0},
        ],
    }


def l_boundary_coords() -> list[dict]:
    return [
        {"latitude": 38.000, "longitude": 127.000},
        {"latitude": 38.000, "longitude": 127.040},
        {"latitude": 38.010, "longitude": 127.040},
        {"latitude": 38.010, "longitude": 127.010},
        {"latitude": 38.040, "longitude": 127.010},
        {"latitude": 38.040, "longitude": 127.000},
    ]


def fragmented_detail(*, with_limit: bool = False) -> dict:
    detail = {
        "coordinateList": [],
        "lineList": [],
        "areaList": [
            area_row(38.001, 127.025, 0.006, 0.010),
            area_row(38.025, 127.001, 0.010, 0.006),
            area_row(38.002, 127.002, 0.006, 0.006),
        ],
    }
    if with_limit:
        detail["areaAssignmentDetail"] = {
            "coordinateList": l_boundary_coords(),
            "lineList": [],
            "areaList": [],
        }
    return detail


def test_disjoint_fragments_use_the_plain_convex_hull() -> None:
    fragments = [
        rect(0, 0, 1600, 1200),
        rect(3200, -300, 1400, 1100),
        rect(1500, 2600, 500, 420),
    ]
    result, info = convex_hull_area_fragments_xy(fragments)
    expected = unary_union(fragments).convex_hull

    assert result is not None
    assert result.equals_exact(expected, 1e-6)
    assert info["method"] == "convex_hull"
    assert info["partsBefore"] == 3
    assert info["partsAfter"] == 1
    assert info["clippedToLimit"] is False


def test_convex_hull_is_clipped_to_the_original_area_boundary() -> None:
    limit = Polygon(
        [
            (0, 0),
            (4000, 0),
            (4000, 1000),
            (1000, 1000),
            (1000, 4000),
            (0, 4000),
        ]
    )
    fragments = [
        rect(2500, 100, 900, 700),
        rect(100, 2500, 700, 900),
        rect(100, 100, 700, 700),
    ]
    source = unary_union(fragments)
    raw_hull = source.convex_hull

    result, info = convex_hull_area_fragments_xy(
        fragments,
        limit_geometry=limit,
    )

    assert result is not None
    assert result.buffer(0.05).contains(source)
    assert result.difference(limit.buffer(0.05)).is_empty
    assert result.area <= limit.area + 1.0
    assert result.area < raw_hull.area
    assert info["clippedToLimit"] is True


def test_disconnected_clip_uses_the_original_boundary_not_an_oversized_hull() -> None:
    limit = Polygon(
        [
            (0, 0),
            (4000, 0),
            (4000, 1000),
            (1000, 1000),
            (1000, 4000),
            (0, 4000),
        ]
    )
    fragments = [
        rect(2600, 200, 900, 600),
        rect(200, 2600, 600, 900),
    ]

    result, info = convex_hull_area_fragments_xy(
        fragments,
        limit_geometry=limit,
    )

    assert result is not None
    assert result.equals_exact(limit, 1e-6)
    assert info["usedLimitFallback"] is True
    assert result.area <= limit.area + 1.0


def test_overlapping_fragments_still_produce_their_convex_hull() -> None:
    fragments = [rect(0, 0, 1000, 1000), rect(500, 500, 1000, 1000)]
    result, info = convex_hull_area_fragments_xy(fragments)

    assert result is not None
    assert result.equals_exact(unary_union(fragments).convex_hull, 1e-6)
    assert info["addedAreaM2"] > 0.0


def test_invalid_and_excessive_fragment_inputs_fail_closed() -> None:
    result, info = convex_hull_area_fragments_xy([])
    assert result is None
    assert info["reason"] == "no_valid_fragments"

    fragments = [rect(i * 700.0, 0, 300, 300) for i in range(41)]
    result, info = convex_hull_area_fragments_xy(fragments)
    assert result is None
    assert info["reason"] == "fragment_cap_exceeded"


def test_pipeline_uses_convex_hull_for_fragmented_ownership() -> None:
    detail = fragmented_detail()
    component = _single_area_ownership_component(detail)

    assert isinstance(component, dict)
    assert component["componentDecomposition"] == "convex_hull_ownership_envelope"
    assert component["componentHull"]["partsBefore"] == 3

    source = unary_union(
        [
            _coord_list_to_polygon_xy(row["coordinateList"])
            for row in detail["areaList"]
        ]
    )
    assert abs(float(component["areaM2"]) - float(source.convex_hull.area)) < 1.0


def test_pipeline_never_exceeds_the_stable_assignment_boundary() -> None:
    detail = fragmented_detail(with_limit=True)
    component = _single_area_ownership_component(detail)
    component_polygon = _coord_list_to_polygon_xy(component["coordinateList"])
    limit_polygon = _coord_list_to_polygon_xy(l_boundary_coords())

    assert component["componentDecomposition"] == "convex_hull_ownership_envelope"
    assert component["componentHull"]["clippedToLimit"] is True
    assert component_polygon is not None and limit_polygon is not None
    assert component_polygon.difference(limit_polygon.buffer(0.05)).is_empty
    assert component_polygon.area <= limit_polygon.area + 1.0


def test_single_concave_polygon_detail_is_also_hulled() -> None:
    detail = fragmented_detail()
    detail["areaList"] = [
        {
            "isHole": False,
            "coordinateList": l_boundary_coords(),
        }
    ]
    component = _single_area_ownership_component(detail)
    source = _coord_list_to_polygon_xy(l_boundary_coords())
    component_polygon = _coord_list_to_polygon_xy(component["coordinateList"])

    assert component["componentDecomposition"] == "convex_hull_ownership_envelope"
    assert component["componentHull"]["partsBefore"] == 1
    assert source is not None and component_polygon is not None
    assert component_polygon.equals_exact(source.convex_hull, 0.05)


def test_hull_failure_uses_the_original_boundary_as_the_size_cap() -> None:
    detail = fragmented_detail(with_limit=True)
    with patch.object(
        next_collab_pipeline,
        "convex_hull_area_fragments_xy",
        return_value=(None, {"reason": "forced"}),
    ):
        component = _single_area_ownership_component(detail)

    assert isinstance(component, dict)
    assert component["componentDecomposition"] == "original_boundary_ownership_fallback"
    component_polygon = _coord_list_to_polygon_xy(component["coordinateList"])
    limit_polygon = _coord_list_to_polygon_xy(l_boundary_coords())
    assert component_polygon is not None and limit_polygon is not None
    assert component_polygon.equals_exact(limit_polygon, 0.05)


def test_branch_ownership_components_are_never_hulled_together() -> None:
    detail = fragmented_detail()
    components = _area_planner_components_from_detail(deepcopy(detail))

    assert len(components) == len(detail["areaList"])
    assert all(
        component.get("componentDecomposition")
        != "convex_hull_ownership_envelope"
        for component in components
    )
