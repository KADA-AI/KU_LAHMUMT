"""Type 2/3 ladder holds slide onto masking terrain, facing away from the mission.

The geometric anchors (previous-line midpoint, corridor point, area centroid)
still decide roughly where the manned aircraft waits; the DEM cover selector
then picks nearby low ground that puts a ridge between the aircraft and the
mission the UAVs are working.  Every failure path must return the plain anchor.
"""

from __future__ import annotations

from typing import Any

import pytest

from modules.mission_planning.MissionPlanner.data_def import lah_terminal_cover
from modules.mission_planning.pipelines import ground_maneuver_mode as gmm


def _line(*points: tuple[float, float]) -> dict[str, Any]:
    return {
        "lineList": [
            {"coordinateList": [{"latitude": lat, "longitude": lon} for lat, lon in points]}
        ]
    }


def _area(*points: tuple[float, float]) -> dict[str, Any]:
    return {
        "areaList": [
            {"coordinateList": [{"latitude": lat, "longitude": lon} for lat, lon in points]}
        ]
    }


def _missions() -> list[dict[str, Any]]:
    return [
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


def _info_for(mission_index: int) -> tuple[dict[str, Any] | None, str]:
    missions = _missions()
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


def test_previous_mid_hold_moves_to_the_cover_point(monkeypatch: pytest.MonkeyPatch) -> None:
    # Mission 3 (index 2): LAH trails at the mission-2 line midpoint (~37.96,
    # 127.36).  Offer a cover point a few hundred metres behind it.
    stub = _SelectorStub(37.955, 127.352)
    monkeypatch.setattr(lah_terminal_cover, "select_lah_terminal_cover_point", stub)

    info, behavior = _info_for(2)

    assert behavior == "previous_maneuver_mid_hold"
    assert _point(info) == (37.955, 127.352)
    assert len(stub.calls) == 1
    call = stub.calls[0]
    # Radius search around the ladder anchor, no containment rows.
    assert call["area_list"] == []
    assert call["search_radius_m"] == 1500.0
    # Threats sampled from the mission the UAVs are working (mission 3 AREA).
    threats = call["threat_coordinates"]
    assert threats
    lats = [float(t["latitude"]) for t in threats]
    lons = [float(t["longitude"]) for t in threats]
    assert all(37.94 <= lat <= 38.00 for lat in lats)
    assert all(127.34 <= lon <= 127.40 for lon in lons)


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
    stub = _SelectorStub(37.955, 127.352)
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
    stub = _SelectorStub(37.955, 127.352)
    monkeypatch.setattr(lah_terminal_cover, "select_lah_terminal_cover_point", stub)

    first, _ = _info_for(2)
    second, _ = _info_for(2)

    assert _point(first) == _point(second) == (37.955, 127.352)
    assert len(stub.calls) == 1
