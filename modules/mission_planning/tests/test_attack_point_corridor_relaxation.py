"""The photographed-corridor rule must not cancel an otherwise valid attack.

On the first discovered target the corridor is still short and the manned
aircraft is approaching over unphotographed ground, so the segment towards it
leaves the corridor well inside the minimum standoff.  Requiring both at once
then has no solution and the whole attack used to be cancelled.
"""

from __future__ import annotations

from typing import Any

from modules.mission_planning.replanning.triggers.attack import pipeline as ap

# The 2026-07-25 field geometry: corridor exit at ~875 m, minimum standoff 3 km.
FRIENDLY = {"latitude": 37.831872526919504, "longitude": 128.1138124873806, "altitude": 691}
TARGET = {"latitude": 37.86940498279169, "longitude": 128.12629622697938, "altitude": 0}
CORRIDOR = [
    {
        "zoneType": "line",
        "inputMissionID": 70000000,
        "widthM": 1000.0,
        "coordinateList": [
            {"latitude": 37.864374980768766, "longitude": 128.11906281555204},
            {"latitude": 37.883797068574594, "longitude": 128.1309074505618},
        ],
    }
]
STRICT_META: dict[str, Any] = {
    "requireInsideDiscoveredMissionZone": True,
    "allowBeforeDiscoveredMissionZone": False,
    "reason": "first_discovered_mission_zone_inside",
    "corridorCount": 1,
}


def _standoffs() -> tuple[float, float]:
    from modules.mission_planning.pipelines.mission_planning_attack_helpers import (
        get_attack_standoff_distances,
    )

    return get_attack_standoff_distances()


def test_strict_corridor_alone_has_no_solution_for_this_geometry() -> None:
    """Pins the premise: inside-only genuinely cannot be satisfied here."""

    minimum_m, preferred_m = _standoffs()
    assert (
        ap._select_attack_standoff_inside_constraints(
            FRIENDLY,
            TARGET,
            min_standoff_m=minimum_m,
            preferred_standoff_m=preferred_m,
            line_coverage_corridors=CORRIDOR,
            require_inside_mission_zone=True,
        )
        is None
    )


def test_relaxed_corridor_finds_a_point_that_still_meets_the_standoff() -> None:
    minimum_m, preferred_m = _standoffs()
    relaxed = ap._select_attack_standoff_inside_constraints(
        FRIENDLY,
        TARGET,
        min_standoff_m=minimum_m,
        preferred_standoff_m=preferred_m,
        line_coverage_corridors=CORRIDOR,
        require_inside_mission_zone=False,
    )
    assert relaxed is not None
    assert float(relaxed["enemy_distance_m"]) >= minimum_m


def test_attack_point_is_produced_and_the_relaxation_is_reported() -> None:
    cache_stats: dict[str, Any] = {}
    point, error = ap._compute_attack_point(
        FRIENDLY,
        TARGET,
        friendly_heading_deg=None,
        friendly_speed_mps=None,
        cache_stats=cache_stats,
        line_coverage_corridors=CORRIDOR,
        line_coverage_metadata=STRICT_META,
    )

    assert error is None, error
    assert point is not None
    minimum_m, _preferred_m = _standoffs()
    standoff_m = ap._to_float(point.get("enemy_distance_m"))
    if standoff_m is not None:
        assert standoff_m >= minimum_m
    # The relaxation must never be silent.
    assert cache_stats.get("missionZoneRequirementRelaxed") is True


def test_a_corridor_that_can_satisfy_the_standoff_is_not_relaxed() -> None:
    """Relaxation is a last resort, not the default."""

    minimum_m, preferred_m = _standoffs()
    # A corridor running along the friendly bearing keeps the whole segment inside.
    wide_corridor = [
        {
            "zoneType": "line",
            "inputMissionID": 70000000,
            "widthM": 1000.0,
            "coordinateList": [
                {"latitude": TARGET["latitude"], "longitude": TARGET["longitude"]},
                {"latitude": FRIENDLY["latitude"], "longitude": FRIENDLY["longitude"]},
            ],
        }
    ]
    strict = ap._select_attack_standoff_inside_constraints(
        FRIENDLY,
        TARGET,
        min_standoff_m=minimum_m,
        preferred_standoff_m=preferred_m,
        line_coverage_corridors=wide_corridor,
        require_inside_mission_zone=True,
    )
    assert strict is not None

    cache_stats: dict[str, Any] = {}
    point, error = ap._compute_attack_point(
        FRIENDLY,
        TARGET,
        friendly_heading_deg=None,
        friendly_speed_mps=None,
        cache_stats=cache_stats,
        line_coverage_corridors=wide_corridor,
        line_coverage_metadata=STRICT_META,
    )
    assert error is None
    assert point is not None
    assert not cache_stats.get("missionZoneRequirementRelaxed")


def test_no_corridor_at_all_is_unaffected() -> None:
    cache_stats: dict[str, Any] = {}
    point, error = ap._compute_attack_point(
        FRIENDLY,
        TARGET,
        friendly_heading_deg=None,
        friendly_speed_mps=None,
        cache_stats=cache_stats,
        line_coverage_corridors=[],
        line_coverage_metadata={},
    )
    assert error is None
    assert point is not None
    assert not cache_stats.get("missionZoneRequirementRelaxed")


def test_valid_coordinates_always_produce_a_best_effort_attack_point(
    monkeypatch,
) -> None:
    """Optional LOS/zone solvers may degrade the point, never erase it."""

    monkeypatch.setattr(ap, "_attack_los_enabled", lambda: False)
    monkeypatch.setattr(
        ap,
        "_select_attack_standoff_inside_constraints",
        lambda *_args, **_kwargs: None,
    )
    ap._ATTACK_POINT_CACHE.clear()
    cache_stats: dict[str, Any] = {}

    point, error = ap._compute_attack_point(
        {"latitude": 36.1234, "longitude": 126.1234, "altitude": 500},
        {"latitude": 36.2234, "longitude": 126.2234, "altitude": 700},
        cache_stats=cache_stats,
        line_coverage_corridors=CORRIDOR,
        line_coverage_metadata=STRICT_META,
    )

    assert error is None
    assert point is not None
    assert point["best_effort_attack_point"] is True
    assert point["selection_mode"] == "best_effort_direct_segment"
    assert point["los_verified"] is False
    assert cache_stats["method"] == "best_effort_direct_segment"
