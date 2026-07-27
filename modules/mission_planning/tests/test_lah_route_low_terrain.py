"""Go around the hill, not over it.

Terrain following alone climbs whatever ridge lies on the straight line, which
costs climb time and puts the aircraft on a skyline exactly where it is most
visible. Routing the leg horizontally through the low ground in a corridor
either side of it trades a few percent of distance for a markedly lower flight
profile. Leg endpoints are never moved, so destinations and mission corners
stay exactly where they were.
"""

from __future__ import annotations

import math
from typing import Any

import pytest

from modules.mission_planning.MissionPlanner.data_def import lah_terrain_path as ltp

START = (37.0, 127.0)
END = (37.0, 127.05)  # ~4.4 km due east

# The real dial function, captured before the conftest autouse fixture pins it.
_REAL_STRENGTH = ltp._low_terrain_strength


def _ridge_provider(pairs: Any) -> list[float]:
    """A ridge along the straight line, with lower ground to the north.

    The provider is handed ``(latitude, longitude)`` pairs.
    """

    out: list[float] = []
    for latitude, _longitude in pairs:
        north_m = (float(latitude) - START[0]) * 111_132.0
        out.append(max(100.0, 900.0 - max(0.0, north_m)))
    return out


def _flat_provider(pairs: Any) -> list[float]:
    return [500.0 for _ in pairs]


def _leg(provider, **overrides: Any):
    kwargs: dict[str, Any] = {
        "terrain_provider": provider,
        "corridor_width_m": ltp.LAH_LOW_TERRAIN_CORRIDOR_M,
        "min_leg_m": ltp.LAH_LOW_TERRAIN_MIN_LEG_M,
        "stage_spacing_m": ltp.LAH_LOW_TERRAIN_STAGE_SPACING_M,
        "max_stages": ltp.LAH_LOW_TERRAIN_MAX_STAGES,
        "edge_samples": ltp.LAH_LOW_TERRAIN_EDGE_SAMPLES,
        "segment_allowed": None,
    }
    kwargs.update(overrides)
    return ltp._low_terrain_route_for_leg(START, END, **kwargs)


def test_the_route_steps_aside_towards_lower_ground() -> None:
    route = _leg(_ridge_provider)

    assert len(route) > 2
    # Every interior point moved north, off the ridge.
    assert all(point[0] > START[0] for point in route[1:-1])


def test_the_detour_really_is_lower() -> None:
    route = _leg(_ridge_provider)

    detour = ltp._mean_route_ground_m(route, _ridge_provider)
    straight = ltp._mean_route_ground_m([START, END], _ridge_provider)

    assert detour < straight


def test_the_endpoints_are_never_moved() -> None:
    route = _leg(_ridge_provider)

    assert route[0] == START
    assert route[-1] == END


def test_flat_ground_is_left_as_a_straight_line() -> None:
    """No terrain to avoid means no detour to pay for."""

    assert _leg(_flat_provider) == [START, END]


def test_a_short_leg_is_never_detoured() -> None:
    near = (START[0], START[1] + 0.002)  # ~180 m
    route = ltp._low_terrain_route_for_leg(
        START,
        near,
        terrain_provider=_ridge_provider,
        corridor_width_m=ltp.LAH_LOW_TERRAIN_CORRIDOR_M,
        min_leg_m=ltp.LAH_LOW_TERRAIN_MIN_LEG_M,
        stage_spacing_m=ltp.LAH_LOW_TERRAIN_STAGE_SPACING_M,
        max_stages=ltp.LAH_LOW_TERRAIN_MAX_STAGES,
        edge_samples=ltp.LAH_LOW_TERRAIN_EDGE_SAMPLES,
        segment_allowed=None,
    )

    assert route == [START, near]


def test_a_detour_that_is_not_actually_lower_is_discarded() -> None:
    """The edge cost is a proxy; the final check is the terrain itself."""

    def inverted(pairs: Any) -> list[float]:
        # Cheap-looking to the north by the proxy, but genuinely higher.
        out: list[float] = []
        for latitude, _longitude in pairs:
            north_m = (float(latitude) - START[0]) * 111_132.0
            out.append(500.0 + max(0.0, north_m))
        return out

    route = _leg(inverted)
    detour = ltp._mean_route_ground_m(route, inverted)
    straight = ltp._mean_route_ground_m([START, END], inverted)

    assert detour <= straight


def test_the_corridor_bounds_how_far_the_detour_can_wander() -> None:
    route = _leg(_ridge_provider)
    leg_m = ltp._distance_m(START, END)

    cap_m = min(
        ltp.LAH_LOW_TERRAIN_CORRIDOR_M, leg_m * ltp.LAH_LOW_TERRAIN_CORRIDOR_RATIO
    )
    for point in route[1:-1]:
        lateral_m = abs(point[0] - START[0]) * 111_132.0
        assert lateral_m <= cap_m + 1.0


def test_the_attack_route_builder_asks_for_low_terrain_by_default() -> None:
    import inspect

    from modules.mission_planning.replanning.triggers.attack import pipeline as ap

    source = inspect.getsource(ap._build_lah_low_level_waypoint_route)
    assert "prefer_low_terrain" in source
    assert ap.get_runtime_attack_int("lah_route_prefer_low_terrain", 1) == 1


def test_strength_never_widens_an_explicitly_narrow_corridor(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A mission-declared narrow corridor is a promise even at high strength."""

    monkeypatch.setattr(ltp, "_low_terrain_strength", lambda: 2.0)
    narrow_m = 75.0
    route = _leg(_ridge_provider, corridor_width_m=narrow_m)

    for point in route[1:-1]:
        lateral_m = abs(point[0] - START[0]) * 111_132.0
        assert lateral_m <= narrow_m + 1.0


def test_higher_strength_buys_a_lower_route(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(ltp, "_low_terrain_strength", lambda: 0.5)
    mild = _leg(_ridge_provider)
    monkeypatch.setattr(ltp, "_low_terrain_strength", lambda: 2.0)
    strong = _leg(_ridge_provider)

    assert ltp._mean_route_ground_m(strong, _ridge_provider) <= ltp._mean_route_ground_m(
        mild, _ridge_provider
    )


def test_strength_reads_clamps_and_defaults_from_runtime_settings(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import modules.mission_planning.MissionPlanner.runtime_settings as rs

    def fake(key: str, default: float, payload=None):
        assert key == "lah_low_terrain_strength"
        return fake.value

    monkeypatch.setattr(rs, "get_runtime_float", fake)
    for raw, expected in ((1.5, 1.5), (-2.0, 0.0), (99.0, 3.0), (float("nan"), 1.0)):
        fake.value = raw
        assert _REAL_STRENGTH() == expected

    def boom(key: str, default: float, payload=None):
        raise RuntimeError("no settings in this context")

    monkeypatch.setattr(rs, "get_runtime_float", boom)
    assert _REAL_STRENGTH() == 1.0
