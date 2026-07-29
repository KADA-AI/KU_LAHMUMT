from __future__ import annotations

from typing import Any

import pytest

from modules.mission_planning.pipelines.ground_maneuver_mode import (
    build_urban_operation_lah_sequence,
    resolve_urban_operation_lah_anchors,
)


def _line(start: tuple[float, float], end: tuple[float, float]) -> dict[str, Any]:
    return {
        "lineList": [
            {
                "coordinateList": [
                    {"latitude": start[0], "longitude": start[1]},
                    {"latitude": end[0], "longitude": end[1]},
                ]
            }
        ]
    }


def _area(center: tuple[float, float], *, is_hole: bool = False) -> dict[str, Any]:
    lat, lon = center
    return {
        "areaList": [
            {
                "isHole": bool(is_hole),
                "coordinateList": [
                    {"latitude": lat - 0.01, "longitude": lon - 0.01},
                    {"latitude": lat - 0.01, "longitude": lon + 0.01},
                    {"latitude": lat + 0.01, "longitude": lon + 0.01},
                    {"latitude": lat + 0.01, "longitude": lon - 0.01},
                ],
            }
        ]
    }


def _mission(
    mission_id: int,
    mission_type: int,
    region_type: int,
    detail: dict[str, Any],
) -> dict[str, Any]:
    return {
        "inputMissionID": int(mission_id),
        "inputMissionType": int(mission_type),
        "regionType": int(region_type),
        "missionDetail": detail,
    }


def _staged_package(package_type: int, *, return_region: int) -> dict[str, Any]:
    if int(package_type) == 4:
        operation_line = _mission(3, 1, 7, _line((38.10, 127.10), (38.11, 127.11)))
        operation_area = _mission(4, 3, 7, _area((38.12, 127.12), is_hole=True))
    else:
        operation_line = _mission(3, 1, 11, _line((38.10, 127.10), (38.11, 127.11)))
        operation_area = _mission(4, 6, 11, _area((38.12, 127.12)))
    return {
        "inputMissionPackageType": int(package_type),
        "inputMissionList": [
            _mission(1, 1, 4, _line((38.00, 127.00), (38.01, 127.01))),
            _mission(2, 2, 4, _area((38.02, 127.02))),
            operation_line,
            operation_area,
            _mission(5, 1, int(return_region), _line((38.13, 127.13), (38.03, 127.03))),
            _mission(6, 1, 3, _line((38.03, 127.03), (38.20, 127.20))),
            _mission(7, 1, 2, _line((38.20, 127.20), (38.30, 127.30))),
        ],
    }


@pytest.mark.parametrize("package_type", [4, 5])
@pytest.mark.parametrize("return_region", [4, 3])
def test_staged_operation_distinguishes_return_line_from_real_acp(
    package_type: int,
    return_region: int,
) -> None:
    package = _staged_package(package_type, return_region=return_region)

    anchors = resolve_urban_operation_lah_anchors(package)
    assert anchors is not None
    assert anchors["returnOrder"] == 5
    assert anchors["acp2"] == {"latitude": 38.20, "longitude": 127.20}

    rows = build_urban_operation_lah_sequence(
        package,
        package_type=package_type,
    )
    assert rows
    behaviors = {int(row["inputMissionID"]): row["behavior"] for row in rows}
    assert behaviors[5] == "attack_wait_hold"
    assert behaviors[6] == "attack_wait_to_acp2_follow"
    assert behaviors[7] == "acp2_to_control_follow"
