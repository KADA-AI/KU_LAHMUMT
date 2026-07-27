"""Type 1 (대기갑항공타격) 공격대기지역 hold must take terrain cover.

The battle-position hold already selects a covered point inside its own area.
The 공격대기 hold used to sit on the bare area centroid, which can be fully
exposed to the 목표지역 the UAVs are working.
"""

from __future__ import annotations

from typing import Any

from modules.mission_planning.pipelines import lah_operational_mode as lom

REGION_ACP = 3
REGION_ATTACK_WAIT = 4
REGION_BATTLE = 5
REGION_TARGET = 6
REGION_CONTROL = 2


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


def _missions() -> list[dict[str, Any]]:
    """The reviewed 10-mission anti-armor pattern."""

    wait_detail = _area(37.84, 127.24)
    spec = [
        (1, REGION_ACP, _line((37.80, 127.20), (37.82, 127.22))),
        (1, REGION_ATTACK_WAIT, wait_detail),
        (2, REGION_ATTACK_WAIT, wait_detail),
        (1, REGION_BATTLE, _area(37.88, 127.28)),
        (2, REGION_BATTLE, _area(37.88, 127.28)),
        (1, REGION_TARGET, _area(37.94, 127.34)),
        (2, REGION_TARGET, _area(37.94, 127.34)),
        (1, REGION_BATTLE, _area(37.88, 127.28)),
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


def _rows_by_id(missions: list[dict[str, Any]]) -> dict[int, dict[str, Any]]:
    rows = lom.build_lah_special_sequence(missions)
    assert rows, "Type 1 must produce a manned sequence"
    return {int(row["inputMissionID"]): row for row in rows}


def test_profile_carries_the_attack_wait_area_geometry() -> None:
    profile = lom.detect_lah_special_operation(_missions())
    assert profile is not None
    assert profile["mode"] == "anti_armor_air_strike_review"
    assert profile["attackWaitAreaList"], "공격대기지역 polygon must reach the hold builder"


def test_attack_wait_hold_asks_the_cover_selector_with_the_target_as_threat(monkeypatch) -> None:
    calls: list[dict[str, Any]] = []
    import modules.mission_planning.MissionPlanner.data_def.lah_terminal_cover as cover

    def fake_select(area_list, fallback_coordinate, **kwargs):
        calls.append(
            {
                "area_list": area_list,
                "fallback": dict(fallback_coordinate),
                "threats": kwargs.get("threat_coordinates"),
            }
        )
        moved = dict(fallback_coordinate)
        moved["latitude"] = float(moved["latitude"]) + 0.001
        return moved, {"applied": True, "reason": "ok"}

    monkeypatch.setattr(cover, "select_lah_terminal_cover_point", fake_select)
    lom._TERMINAL_COVER_CACHE.clear()

    rows = _rows_by_id(_missions())
    # The manned aircraft lags one region behind the UAVs: while they fly the
    # 전투진지 missions (4,5) it holds in the 공격대기지역.
    assert rows[4]["behavior"] == "attack_wait_hold"
    assert rows[5]["behavior"] == "attack_wait_hold"

    wait_calls = [
        call
        for call in calls
        # The 공격대기 polygon starts at 37.84; the battle polygon at 37.88.
        if abs(float(call["area_list"][0]["coordinateList"][0]["latitude"]) - 37.84) < 1e-6
    ]
    assert wait_calls, "the 공격대기 hold must consult the cover selector"
    assert wait_calls[0]["threats"], "the 목표지역 must be supplied as the threat"


def test_attack_wait_hold_does_not_pin_an_altitude() -> None:
    """The legacy 1500 m hold altitude is gone: terrain/LOS passes decide."""

    lom._TERMINAL_COVER_CACHE.clear()
    rows = _rows_by_id(_missions())
    info = rows[4]["individualMissionInfo"]
    assert "forceAltitudeM" not in info
    assert info["individualMissionType"] == 9
    assert info["patternType"] == 12
    assert len(info["coordinateList"]) == 1


def test_without_area_geometry_the_previous_anchor_is_kept() -> None:
    """Cover selection is an upgrade, never a precondition for the hold."""

    lom._TERMINAL_COVER_CACHE.clear()
    anchor = {"latitude": 37.85, "longitude": 127.25, "altitude": 400}
    info = lom._attack_wait_hold_mission_info({"attackWaitCoordinate": anchor}, anchor)

    assert info is not None
    assert info["coordinateList"] == [anchor]
    assert "forceAltitudeM" not in info


def test_battle_position_hold_still_takes_cover() -> None:
    """The behaviour this change was modelled on must be unaffected."""

    lom._TERMINAL_COVER_CACHE.clear()
    rows = _rows_by_id(_missions())
    for mission_id in (6, 7):
        assert rows[mission_id]["behavior"] == "battle_position_hold"
        info = rows[mission_id]["individualMissionInfo"]
        assert "forceAltitudeM" not in info
        assert info["_lahTerminalCoverEnabled"] is True


def test_type1_egress_line_widths_are_icd_uint_json_values() -> None:
    rows = _rows_by_id(_missions())

    for mission_id in (9, 10):
        info = rows[mission_id]["individualMissionInfo"]
        widths = [line["width"] for line in info["lineList"]]
        assert widths == [500]
        assert all(type(width) is int for width in widths)
