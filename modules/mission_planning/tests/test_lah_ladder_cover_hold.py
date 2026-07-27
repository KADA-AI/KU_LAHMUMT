"""Type 2/3 ladder holds slide onto masking terrain, facing away from the mission.

The geometric anchors (previous-line midpoint, corridor point, area centroid)
still decide roughly where the manned aircraft waits; the DEM cover selector
then picks nearby low ground that puts a ridge between the aircraft and the
mission the UAVs are working.

Containment is the hard rule: an initial-plan hold stays inside the declared
LINE corridor or 목표지역 polygon it came from.  Every failure path - no width
to build a corridor from, a selection outside that geometry, a selector error -
must return the plain geometric anchor.
"""

from __future__ import annotations

import math
from typing import Any

import pytest

from modules.mission_planning.MissionPlanner.data_def import lah_terminal_cover
from modules.mission_planning.pipelines import ground_maneuver_mode as gmm

# Every maneuver LINE in the operational plans carries a declared width; the
# corridor band it sweeps is what bounds a hold.
_LINE_WIDTH_M = 1400.0


def _line(*points: tuple[float, float], width: float | None = _LINE_WIDTH_M) -> dict[str, Any]:
    row: dict[str, Any] = {
        "coordinateList": [{"latitude": lat, "longitude": lon} for lat, lon in points]
    }
    if width is not None:
        row["width"] = width
    return {"lineList": [row]}


def _area(*points: tuple[float, float]) -> dict[str, Any]:
    return {
        "areaList": [
            {"coordinateList": [{"latitude": lat, "longitude": lon} for lat, lon in points]}
        ]
    }


def _missions(*, maneuver_width: float | None = _LINE_WIDTH_M) -> list[dict[str, Any]]:
    missions = [
        {"inputMissionID": 1, "inputMissionType": 1, "regionType": 3,
         "missionDetail": _line((37.90, 127.30), (37.92, 127.32))},
        {"inputMissionID": 2, "inputMissionType": 1, "regionType": 6,
         "missionDetail": _line((37.94, 127.34), (37.96, 127.36), (37.98, 127.38))},
        {"inputMissionID": 3, "inputMissionType": 5, "regionType": 6,
         "missionDetail": _area((37.95, 127.35), (37.99, 127.35), (37.99, 127.39), (37.95, 127.39))},
        {"inputMissionID": 4, "inputMissionType": 1, "regionType": 7,
         "missionDetail": _line((37.94, 127.40), (37.96, 127.42))},
        {"inputMissionID": 5, "inputMissionType": 3, "regionType": 7,
         "missionDetail": _area((37.94, 127.40), (37.97, 127.40), (37.97, 127.43), (37.94, 127.43))},
        {"inputMissionID": 6, "inputMissionType": 1, "regionType": 6,
         "missionDetail": _line((37.96, 127.36), (37.98, 127.38))},
        {"inputMissionID": 7, "inputMissionType": 1, "regionType": 3,
         "missionDetail": _line((37.93, 127.33), (37.91, 127.31))},
        {"inputMissionID": 8, "inputMissionType": 1, "regionType": 2,
         "missionDetail": _line((37.90, 127.30), (37.88, 127.28))},
    ]
    if maneuver_width is None:
        for mission in missions:
            for row in (mission.get("missionDetail") or {}).get("lineList") or []:
                row.pop("width", None)
    return missions


def _info_for(
    mission_index: int, *, maneuver_width: float | None = _LINE_WIDTH_M
) -> tuple[dict[str, Any] | None, str]:
    missions = _missions(maneuver_width=maneuver_width)
    anchors = gmm.resolve_ground_maneuver_lah_anchors(missions)
    assert anchors is not None
    return gmm.ground_maneuver_lah_info_for_index(missions, anchors, mission_index)


def _point(info: dict[str, Any]) -> tuple[float, float]:
    coord = info["coordinateList"][0]
    return float(coord["latitude"]), float(coord["longitude"])


@pytest.fixture(autouse=True)
def _fresh_cover_state(monkeypatch: pytest.MonkeyPatch):
    with gmm._COVER_HOLD_CACHE_LOCK:
        gmm._COVER_HOLD_CACHE.clear()
    monkeypatch.setattr(gmm, "_cover_hold_enabled", lambda: True)
    monkeypatch.setattr(gmm, "_cover_hold_search_radius_m", lambda: 1500.0)
    yield
    with gmm._COVER_HOLD_CACHE_LOCK:
        gmm._COVER_HOLD_CACHE.clear()


class _SelectorStub:
    """Stands in for select_lah_terminal_cover_point with a fixed answer."""

    def __init__(self, latitude: float, longitude: float):
        self.latitude = latitude
        self.longitude = longitude
        self.calls: list[dict[str, Any]] = []

    def __call__(self, area_list, fallback_coordinate, **kwargs):
        self.calls.append(
            {
                "area_list": area_list,
                "fallback": dict(fallback_coordinate),
                **kwargs,
            }
        )
        return (
            {"latitude": self.latitude, "longitude": self.longitude},
            {"reason": "ok"},
        )


# ~300 m off the mission-2 centreline, well inside its 700 m half-width, and
# no closer to the mission-3 AREA than the anchor is.
_INSIDE_CORRIDOR = (37.96167, 127.35732)
# Beyond the same corridor's half-width: a valid ridge, but not a hold the
# operator declared.
_OUTSIDE_CORRIDOR = (37.9700, 127.3400)


def test_previous_mid_hold_moves_to_the_cover_point(monkeypatch: pytest.MonkeyPatch) -> None:
    # Mission 3 (index 2): LAH trails at the mission-2 line midpoint (~37.96,
    # 127.36).  Offer a cover point a few hundred metres off the centreline.
    stub = _SelectorStub(*_INSIDE_CORRIDOR)
    monkeypatch.setattr(lah_terminal_cover, "select_lah_terminal_cover_point", stub)

    info, behavior = _info_for(2)

    assert behavior == "previous_maneuver_mid_hold"
    assert _point(info) == _INSIDE_CORRIDOR
    assert len(stub.calls) == 1
    call = stub.calls[0]
    # The declared LINE corridor rides along as containment: the radius search
    # is clipped to the band the operator actually declared.
    assert call["area_list"], "corridor holds must pass containment rows"
    assert call["search_radius_m"] == 1500.0
    outer = [row for row in call["area_list"] if not row.get("isHole")]
    assert len(outer) == 1
    assert len(outer[0]["coordinateList"]) >= 4
    # Threats sampled from the mission the UAVs are working (mission 3 AREA).
    threats = call["threat_coordinates"]
    assert threats
    lats = [float(t["latitude"]) for t in threats]
    lons = [float(t["longitude"]) for t in threats]
    assert all(37.94 <= lat <= 38.00 for lat in lats)
    assert all(127.34 <= lon <= 127.40 for lon in lons)


def test_a_cover_point_outside_the_declared_corridor_is_rejected(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The regression this guard exists for: a hold outside the LINE band."""

    stub = _SelectorStub(*_OUTSIDE_CORRIDOR)
    monkeypatch.setattr(lah_terminal_cover, "select_lah_terminal_cover_point", stub)

    info, behavior = _info_for(2)

    assert behavior == "previous_maneuver_mid_hold"
    lat, lon = _point(info)
    assert (round(lat, 4), round(lon, 4)) == (37.9600, 127.3600)


def test_a_line_without_a_declared_width_keeps_its_anchor(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """No width means no corridor to stay inside, so no slide is offered."""

    stub = _SelectorStub(*_INSIDE_CORRIDOR)
    monkeypatch.setattr(lah_terminal_cover, "select_lah_terminal_cover_point", stub)

    info, behavior = _info_for(2, maneuver_width=None)

    assert behavior == "previous_maneuver_mid_hold"
    lat, lon = _point(info)
    assert (round(lat, 4), round(lon, 4)) == (37.9600, 127.3600)
    assert stub.calls == [], "the selector must not even be consulted"


def test_the_corridor_band_matches_the_declared_width() -> None:
    """The synthesized containment band is the polyline swept by its width."""

    branch = {
        "coordinateList": [
            {"latitude": 37.94, "longitude": 127.34},
            {"latitude": 37.98, "longitude": 127.38},
        ],
        "width": _LINE_WIDTH_M,
    }
    rows = gmm._line_corridor_area_rows(branch)
    assert rows and not rows[0]["isHole"]

    half_width_m = _LINE_WIDTH_M * 0.5
    centre = {"latitude": 37.96, "longitude": 127.36}
    metres_per_lat = 111_132.92
    metres_per_lon = metres_per_lat * math.cos(math.radians(37.96))
    # Perpendicular to a NE-running leg, normalised.
    dx = 0.04 * metres_per_lon
    dy = 0.04 * metres_per_lat
    norm = math.hypot(dx, dy)
    perp = (-dy / norm, dx / norm)

    def _offset(distance_m: float) -> dict[str, Any]:
        return {
            "latitude": centre["latitude"] + perp[1] * distance_m / metres_per_lat,
            "longitude": centre["longitude"] + perp[0] * distance_m / metres_per_lon,
        }

    assert gmm._point_in_area_rows(centre, rows)
    assert gmm._point_in_area_rows(_offset(half_width_m * 0.9), rows)
    assert gmm._point_in_area_rows(_offset(-half_width_m * 0.9), rows)
    assert not gmm._point_in_area_rows(_offset(half_width_m * 1.2), rows)
    assert not gmm._point_in_area_rows(_offset(-half_width_m * 1.2), rows)


def test_a_hole_in_the_containment_geometry_is_excluded() -> None:
    rows = [
        {
            "coordinateList": [
                {"latitude": 37.90, "longitude": 127.30},
                {"latitude": 37.99, "longitude": 127.30},
                {"latitude": 37.99, "longitude": 127.39},
                {"latitude": 37.90, "longitude": 127.39},
            ],
            "isHole": False,
        },
        {
            "coordinateList": [
                {"latitude": 37.94, "longitude": 127.34},
                {"latitude": 37.96, "longitude": 127.34},
                {"latitude": 37.96, "longitude": 127.36},
                {"latitude": 37.94, "longitude": 127.36},
            ],
            "isHole": True,
        },
    ]

    assert gmm._point_in_area_rows({"latitude": 37.92, "longitude": 127.32}, rows)
    assert not gmm._point_in_area_rows({"latitude": 37.95, "longitude": 127.35}, rows)
    assert not gmm._point_in_area_rows({"latitude": 38.50, "longitude": 127.35}, rows)


def test_a_cover_point_that_advances_on_the_mission_is_rejected(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # Deep inside the mission-3 AREA: closer to the threat centroid than 60%
    # of the anchor's distance, so the ladder anchor must win.
    stub = _SelectorStub(37.97, 127.37)
    monkeypatch.setattr(lah_terminal_cover, "select_lah_terminal_cover_point", stub)

    info, behavior = _info_for(2)

    assert behavior == "previous_maneuver_mid_hold"
    lat, lon = _point(info)
    assert (round(lat, 4), round(lon, 4)) == (37.9600, 127.3600)


def test_selector_failure_keeps_the_geometric_anchor(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def _boom(*_args, **_kwargs):
        raise RuntimeError("selector unavailable")

    monkeypatch.setattr(lah_terminal_cover, "select_lah_terminal_cover_point", _boom)

    info, behavior = _info_for(2)

    assert behavior == "previous_maneuver_mid_hold"
    lat, lon = _point(info)
    assert (round(lat, 4), round(lon, 4)) == (37.9600, 127.3600)


def test_destination_area_hold_is_containment_mode_with_cover_contract(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # Mission 5 (index 4) is the guard phase: LAH holds inside the 목표지역.
    stub = _SelectorStub(37.966, 127.368)
    monkeypatch.setattr(lah_terminal_cover, "select_lah_terminal_cover_point", stub)

    info, behavior = _info_for(4)

    assert behavior == "destination_area_hold"
    assert _point(info) == (37.966, 127.368)
    call = stub.calls[0]
    # The 목표지역 AREA rows ride along, so the disk is clipped to the area.
    assert call["area_list"], "destination hold must pass containment rows"
    assert all("coordinateList" in row for row in call["area_list"])
    # The d0304 UAV-ETA refine contract keys are seeded.
    assert info["_lahTerminalCoverEnabled"] is True
    assert info["_lahConstraintAreaList"]
    assert info["_lahTerminalCoverThreatCoordinateList"]
    assert info["_lahTerminalCoverFallbackCoordinate"]


def test_toggle_off_keeps_every_anchor(monkeypatch: pytest.MonkeyPatch) -> None:
    stub = _SelectorStub(*_INSIDE_CORRIDOR)
    monkeypatch.setattr(lah_terminal_cover, "select_lah_terminal_cover_point", stub)
    monkeypatch.setattr(gmm, "_cover_hold_enabled", lambda: False)

    info, behavior = _info_for(2)

    assert behavior == "previous_maneuver_mid_hold"
    lat, lon = _point(info)
    assert (round(lat, 4), round(lon, 4)) == (37.9600, 127.3600)
    assert stub.calls == []


def test_the_cover_selection_is_cached_per_anchor(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    stub = _SelectorStub(*_INSIDE_CORRIDOR)
    monkeypatch.setattr(lah_terminal_cover, "select_lah_terminal_cover_point", stub)

    first, _ = _info_for(2)
    second, _ = _info_for(2)

    assert _point(first) == _point(second) == _INSIDE_CORRIDOR
    assert len(stub.calls) == 1
