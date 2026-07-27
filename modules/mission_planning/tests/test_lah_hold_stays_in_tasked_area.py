"""A manned hold belongs in the region it was tasked with.

The cover search may pick which ground inside the 공격대기지역/전투진지 to sit
on; it may not pick ground outside.  When it could, the scorer walked every
hold to the far rim of its 1.5 km search disk - always directly away from the
threat - so the aircraft parked next to the *last* mission of the plan while the
UAVs were still working the target area, and the route read as if it had skipped
straight to the end instead of stepping through the regions in order.
"""

from __future__ import annotations

import math
from typing import Any

from modules.mission_planning.MissionPlanner.data_def import lah_terminal_cover as cover
from modules.mission_planning.pipelines import lah_operational_mode as lom

REGION_CONTROL = 2
REGION_ACP = 3
REGION_ATTACK_WAIT = 4
REGION_BATTLE = 5
REGION_TARGET = 6


def _line(*points: tuple[float, float]) -> dict[str, Any]:
    return {
        "lineList": [
            {
                "coordinateList": [
                    {"latitude": lat, "longitude": lon} for lat, lon in points
                ],
                "width": 500.0,
            }
        ]
    }


def _area(lat0: float, lon0: float, size: float = 0.02) -> dict[str, Any]:
    return {
        "areaList": [
            {
                "coordinateList": [
                    {"latitude": lat0, "longitude": lon0},
                    {"latitude": lat0 + size, "longitude": lon0},
                    {"latitude": lat0 + size, "longitude": lon0 + size},
                    {"latitude": lat0, "longitude": lon0 + size},
                ]
            }
        ]
    }


WAIT_AREA = _area(37.84, 127.24)
BATTLE_AREA = _area(37.88, 127.28)


def _missions() -> list[dict[str, Any]]:
    spec = [
        (1, REGION_ACP, _line((37.80, 127.20), (37.82, 127.22))),
        (1, REGION_ATTACK_WAIT, WAIT_AREA),
        (2, REGION_ATTACK_WAIT, WAIT_AREA),
        (1, REGION_BATTLE, BATTLE_AREA),
        (2, REGION_BATTLE, BATTLE_AREA),
        (1, REGION_TARGET, _area(37.94, 127.34)),
        (2, REGION_TARGET, _area(37.94, 127.34)),
        (1, REGION_BATTLE, BATTLE_AREA),
        (1, REGION_ACP, _line((37.82, 127.22), (37.80, 127.20))),
        (1, REGION_CONTROL, _line((37.80, 127.20), (37.78, 127.18))),
    ]
    return [
        {
            "inputMissionID": index,
            "inputMissionType": mission_type,
            "regionType": region,
            "missionDetail": detail,
        }
        for index, (mission_type, region, detail) in enumerate(spec, start=1)
    ]


def _inside(coordinate: dict[str, Any], detail: dict[str, Any]) -> bool:
    return any(
        lom._point_in_polygon(coordinate, row.get("coordinateList"))
        for row in detail["areaList"]
    )


def _holds_by_behavior() -> dict[str, dict[str, Any]]:
    lom._TERMINAL_COVER_CACHE.clear()
    rows = lom.build_lah_special_sequence(_missions())
    assert rows
    out: dict[str, dict[str, Any]] = {}
    for row in rows:
        info = row.get("individualMissionInfo") or row.get("info") or {}
        coords = info.get("coordinateList") or []
        if coords and str(row.get("behavior")) not in out:
            out[str(row["behavior"])] = coords[0]
    return out


def test_the_attack_wait_hold_sits_inside_the_공격대기지역() -> None:
    holds = _holds_by_behavior()

    assert "attack_wait_hold" in holds
    assert _inside(holds["attack_wait_hold"], WAIT_AREA)


def test_the_battle_hold_sits_inside_the_전투진지() -> None:
    holds = _holds_by_behavior()

    assert "battle_position_hold" in holds
    assert _inside(holds["battle_position_hold"], BATTLE_AREA)


def test_the_search_disk_cannot_buy_ground_outside_the_tasked_area() -> None:
    anchor = cover._coerce_coordinate(
        {"latitude": 37.85, "longitude": 127.25, "altitude": 300}
    )

    confined, _safe, _to_xy, _to_latlon, _margin, error = cover._build_safe_geometry(
        WAIT_AREA["areaList"], 0.10, 100.0
    )
    widened, _safe2, _to_xy2, _to_latlon2, _margin2, error2 = cover._build_safe_geometry(
        WAIT_AREA["areaList"],
        0.10,
        100.0,
        search_center=anchor,
        search_radius_m=1500.0,
    )

    assert error is None and error2 is None
    assert float(widened.area) <= float(confined.area) + 1e-6


def test_the_boundary_standoff_survives_the_clip() -> None:
    """A hold on the AREA edge is one step from being outside it."""

    anchor = cover._coerce_coordinate(
        {"latitude": 37.85, "longitude": 127.25, "altitude": 300}
    )

    allowed, safe, _to_xy, _to_latlon, margin_m, error = cover._build_safe_geometry(
        WAIT_AREA["areaList"],
        0.10,
        100.0,
        search_center=anchor,
        search_radius_m=1500.0,
    )

    assert error is None
    assert margin_m > 0.0
    # The selectable region is strictly inside the clipped search region.
    assert float(safe.area) < float(allowed.area)


def test_a_search_with_no_tasked_area_is_still_the_whole_disk() -> None:
    """Hole-only geometry is keep-out, not a region to stay inside."""

    holes = [dict(row, isHole=True) for row in _area(37.84, 127.24, 0.002)["areaList"]]
    anchor = cover._coerce_coordinate(
        {"latitude": 37.90, "longitude": 127.30, "altitude": 300}
    )

    allowed, safe, _to_xy, _to_latlon, margin_m, error = cover._build_safe_geometry(
        holes, 0.10, 100.0, search_center=anchor, search_radius_m=1500.0
    )

    assert error is None
    disk_area = math.pi * 1500.0 * 1500.0
    assert 0.99 * disk_area < float(allowed.area) < disk_area
    # No mission boundary to stand off from.
    assert margin_m == 0.0
    assert safe is allowed


def test_the_manned_sequence_still_follows_the_input_mission_order() -> None:
    """Nothing reorders the manned missions by proximity."""

    missions = _missions()
    lom._TERMINAL_COVER_CACHE.clear()
    rows = lom.build_lah_special_sequence(missions)

    assert [int(row["inputMissionID"]) for row in rows] == [
        int(mission["inputMissionID"]) for mission in missions
    ]
