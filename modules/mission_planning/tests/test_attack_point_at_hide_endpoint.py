"""The attack point pops up out of cover, climbing only as far as it must.

A gunship engages from cover: it does not fly to a separate firing position and
come back. The concealment endpoint already stands on masking terrain, so the
firing point stays in its immediate neighbourhood - stepping aside far enough
to clear a close ridge diagonally rather than climbing the whole of it.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import pytest

from modules.common.regional_dem import regional_dem_path_for_coordinate
from modules.mission_planning.replanning.triggers.attack import pipeline as ap

TARGET = {"latitude": 37.97138154195951, "longitude": 127.32839121305919, "altitude": 0}
HIDE = {"latitude": 37.955997, "longitude": 127.321100, "altitude": 212}


def _dem_available() -> bool:
    path = regional_dem_path_for_coordinate(
        Path(__file__).resolve().parents[3] / "resource",
        TARGET["latitude"],
        TARGET["longitude"],
    )
    return path is not None and Path(path).is_file()


def test_firing_point_stays_in_the_hide_points_neighbourhood() -> None:
    """It may step aside for a lower sightline, but not relocate."""

    if not _dem_available():
        pytest.skip("operational regional DEM is not installed")

    result = ap._attack_coordinate_at_hide_endpoint(HIDE, TARGET)

    assert result is not None
    assert result["attack_point_at_hide_endpoint"] is True

    radius_m = ap.get_runtime_attack_float("attack_popup_search_radius_m", 600.0)
    lateral_m = math.hypot(
        (result["latitude"] - HIDE["latitude"]) * 111_132.0,
        (result["longitude"] - HIDE["longitude"])
        * 111_320.0
        * math.cos(math.radians(HIDE["latitude"])),
    )
    assert lateral_m >= 1.0, "the armed WP must be distinct from the hide WP"
    assert lateral_m <= radius_m + 1.0
    assert lateral_m == pytest.approx(result.get("attack_point_popup_offset_m", 0.0), abs=1.0)


def test_stepping_aside_never_costs_altitude() -> None:
    """The neighbourhood search may only lower the climb, never raise it."""

    if not _dem_available():
        pytest.skip("operational regional DEM is not installed")

    straight_up, error = ap._compute_attack_los_altitude_batch_dem(
        HIDE,
        TARGET,
        lah_floor_coord=HIDE,
        altitude_offset_m=ap.get_runtime_attack_float(
            "attack_point_hide_popup_margin_m", 30.0
        ),
    )
    if not isinstance(straight_up, dict) or straight_up.get("altitude") is None:
        pytest.skip(f"straight-up profile unavailable: {error}")

    result = ap._attack_coordinate_at_hide_endpoint(HIDE, TARGET)

    assert result is not None
    assert int(result["altitude"]) <= int(straight_up["altitude"])


def test_altitude_is_a_low_popup_base_not_the_los_firing_altitude() -> None:
    """SIM, not the mission packet, owns the climb needed to open LOS."""

    if not _dem_available():
        pytest.skip("operational regional DEM is not installed")

    result = ap._attack_coordinate_at_hide_endpoint(HIDE, TARGET)
    assert result is not None
    assert result["attack_altitude_control"] == "sim_los_popup"
    assert result["attack_point_popup_los_certified"] is True
    assert result["los_verified"] is False
    assert "los_required_altitude_m" not in result
    assert "los_selected_altitude_m" not in result

    firing, error = ap._compute_attack_los_altitude_batch_dem(
        result,
        TARGET,
        lah_floor_coord=result,
        altitude_offset_m=ap.get_runtime_attack_float(
            "attack_point_hide_popup_margin_m", 30.0
        ),
    )
    assert error is None and firing is not None
    assert float(result["altitude"]) < float(firing["altitude"])


def test_the_aircraft_is_never_told_to_descend_for_the_shot() -> None:
    if not _dem_available():
        pytest.skip("operational regional DEM is not installed")

    # A hide point already above the sightline needs no climb at all.
    high_hide = dict(HIDE)
    high_hide["altitude"] = 900
    result = ap._attack_coordinate_at_hide_endpoint(high_hide, TARGET)

    assert result is not None
    assert float(result["altitude"]) >= float(high_hide["altitude"])


def test_the_knob_can_restore_the_separate_attack_point(monkeypatch) -> None:
    monkeypatch.setattr(
        ap,
        "get_runtime_attack_int",
        lambda key, default=0, *a, **k: 0 if key == "attack_point_at_hide_endpoint" else default,
    )
    assert ap._attack_coordinate_at_hide_endpoint(HIDE, TARGET) is None


def test_missing_inputs_degrade_instead_of_raising() -> None:
    assert ap._attack_coordinate_at_hide_endpoint(None, TARGET) is None
    assert ap._attack_coordinate_at_hide_endpoint(HIDE, None) is None
    assert ap._attack_coordinate_at_hide_endpoint({"latitude": 1.0}, TARGET) is None


def test_the_enemy_contact_cover_search_has_no_area_constraint_to_relax() -> None:
    """Concealment under contact is solved around the aircraft, not in an AREA.

    The tasked-AREA selector in ``lah_terminal_cover`` is a different code path
    - it serves the planned 공격대기지역/전투진지 holds, where leaving the AREA
    means not flying the mission.  Nothing here may reintroduce an AREA bound.
    """

    import inspect

    from modules.mission_planning.pipelines import lah_enemy_contact

    source = inspect.getsource(lah_enemy_contact)
    for marker in ("area_list", "areaList", "require_inside_mission_zone"):
        assert marker not in source
    # The solve is anchored on the aircraft and the contacts, nothing else.
    assert "enemy_coordinates" in source
    assert callable(lah_enemy_contact.plan_enemy_contact_response)


def test_the_tasked_area_hold_stays_inside_the_area_it_was_given() -> None:
    """A hold walked out of its AREA reads as the aircraft skipping the mission.

    The search radius widens *where in the AREA* cover may be taken; it is not
    licence to leave.  Without the clip the scorer runs the hold to the far rim
    of the disk - always directly away from the threat - and the manned
    aircraft never occupies the 전투진지 it was tasked with.
    """

    from modules.mission_planning.MissionPlanner.data_def import lah_terminal_cover as cover

    area: list[dict[str, Any]] = [
        {
            "coordinateList": [
                {"latitude": 37.9550, "longitude": 127.3200},
                {"latitude": 37.9552, "longitude": 127.3200},
                {"latitude": 37.9552, "longitude": 127.3202},
                {"latitude": 37.9550, "longitude": 127.3202},
            ]
        }
    ]
    anchor = {"latitude": 37.9551, "longitude": 127.3201, "altitude": 300}

    confined, _safe, _to_xy, _to_latlon, _margin, error = cover._build_safe_geometry(
        area, 0.10, 100.0
    )
    widened, _safe2, _to_xy2, _to_latlon2, _margin2, error2 = cover._build_safe_geometry(
        area,
        0.10,
        100.0,
        search_center=cover._coerce_coordinate(anchor),
        search_radius_m=1500.0,
    )

    assert error is None and error2 is None
    assert confined is not None and widened is not None
    # A 1.5 km disk cannot buy the aircraft any ground outside a ~20 m AREA.
    assert float(widened.area) <= float(confined.area) + 1e-6


def test_keep_out_holes_still_bound_a_search_with_no_tasked_area() -> None:
    """Hole-only geometry is keep-out, not a region to stay inside."""

    from modules.mission_planning.MissionPlanner.data_def import lah_terminal_cover as cover

    holes: list[dict[str, Any]] = [
        {
            "isHole": True,
            "coordinateList": [
                {"latitude": 37.9550, "longitude": 127.3200},
                {"latitude": 37.9552, "longitude": 127.3200},
                {"latitude": 37.9552, "longitude": 127.3202},
                {"latitude": 37.9550, "longitude": 127.3202},
            ],
        }
    ]
    anchor = {"latitude": 37.9600, "longitude": 127.3300, "altitude": 300}

    allowed, _safe, _to_xy, _to_latlon, _margin, error = cover._build_safe_geometry(
        holes,
        0.10,
        100.0,
        search_center=cover._coerce_coordinate(anchor),
        search_radius_m=1500.0,
    )

    assert error is None and allowed is not None
    # Nearly the whole disk survives - only the keep-out is punched out.
    disk_area = math.pi * 1500.0 * 1500.0
    assert 0.99 * disk_area < float(allowed.area) < disk_area
