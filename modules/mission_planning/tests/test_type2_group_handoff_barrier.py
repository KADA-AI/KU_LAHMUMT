from __future__ import annotations

from copy import deepcopy
from types import SimpleNamespace
from typing import Any

import pytest

from modules.mission_planning.MissionPlanner.planning_enhanced.io import (
    export_0302,
)
from modules.mission_planning.replanning.triggers.next_collab import (
    pipeline as next_collab_pipeline,
)


def _coord(latitude: float, longitude: float) -> dict[str, float]:
    return {
        "latitude": float(latitude),
        "longitude": float(longitude),
        "altitude": 700.0,
    }


def _line(longitude: float) -> dict[str, Any]:
    return {
        "width": 700.0,
        "coordinateList": [
            _coord(38.0, longitude),
            _coord(38.01, longitude),
        ],
    }


def _area(longitude: float) -> dict[str, Any]:
    return {
        "isHole": False,
        "coordinateList": [
            _coord(38.01, longitude - 0.003),
            _coord(38.01, longitude + 0.003),
            _coord(38.02, longitude + 0.003),
            _coord(38.02, longitude - 0.003),
        ],
    }


def _type2_input_plan() -> dict[str, Any]:
    longitudes = (127.0, 127.1, 127.2)
    return {
        "inputMissionPackageID": 200_000_901,
        "inputMissionPackageType": 2,
        "availableAircraftList": [
            {"aircraftID": aircraft_id} for aircraft_id in (4, 5, 6)
        ],
        "inputMissionList": [
            {
                "inputMissionID": 901,
                "inputMissionType": 1,
                "regionType": 7,
                "missionDetail": {
                    "coordinateList": [],
                    "lineList": [_line(longitude) for longitude in longitudes],
                    "areaList": [],
                },
            },
            {
                "inputMissionID": 17,
                "inputMissionType": 3,
                "regionType": 7,
                "missionDetail": {
                    "coordinateList": [],
                    "lineList": [],
                    "areaList": [_area(longitude) for longitude in longitudes],
                },
            },
            {
                "inputMissionID": 803,
                "inputMissionType": 1,
                "regionType": 6,
                "missionDetail": {
                    "coordinateList": [],
                    "lineList": [_line(longitude) for longitude in longitudes],
                    "areaList": [],
                },
            },
            {
                "inputMissionID": 4,
                "inputMissionType": 3,
                "regionType": 6,
                "missionDetail": {
                    "coordinateList": [],
                    "lineList": [],
                    "areaList": [_area(127.3)],
                },
            },
        ],
    }


def _individual_mission(
    input_mission_id: int,
    individual_mission_id: int,
    *,
    blocked: bool | None = None,
) -> dict[str, Any]:
    mission: dict[str, Any] = {
        "individualMissionID": int(individual_mission_id),
        "isDone": False,
        "relatedMission": {
            "relatedMissionType": 1,
            "inputMissionID": int(input_mission_id),
            "priorMissionID": 0,
        },
        "individualMissionInfo": {
            "individualMissionType": 3,
            "areaList": [{"isHole": False, "coordinateList": [_coord(38.0, 127.0)]}],
        },
        "pathID": 400_000_000 + int(individual_mission_id),
    }
    if blocked is not None:
        mission["executionBlockedUntilNextCollab"] = bool(blocked)
    return mission


def _package(
    aircraft_id: int,
    input_ids: list[int],
    *,
    first_mission_id: int,
) -> dict[str, Any]:
    return {
        "timestamp": 1,
        "Source": "MMR",
        "individualMissionPackageID": 800_000_000 + int(aircraft_id),
        "aircraftID": int(aircraft_id),
        "individualMissionList": [
            _individual_mission(
                input_id,
                first_mission_id + offset,
                blocked=True if offset == 0 else None,
            )
            for offset, input_id in enumerate(input_ids)
        ],
    }


def _by_input(package: dict[str, Any]) -> dict[int, dict[str, Any]]:
    return {
        int(mission["relatedMission"]["inputMissionID"]): mission
        for mission in package["individualMissionList"]
    }


def test_input_order_barrier_uses_plan_order_and_leaves_earlier_or_lah_unchanged() -> None:
    input_plan = {
        "inputMissionList": [
            {"inputMissionID": 900},
            {"inputMissionID": 12},
            {"inputMissionID": 700},
            {"inputMissionID": 3},
        ]
    }
    uav = _package(4, [3, 900, 700, 12, 999], first_mission_id=900_010_000)
    uav_rows = _by_input(uav)
    uav_rows[900]["executionBlockedUntilNextCollab"] = True
    uav_rows[12]["executionBlockedUntilNextCollab"] = True
    lah = _package(3, [900, 12, 700, 3], first_mission_id=900_020_000)
    lah_before = deepcopy(lah)

    result = next_collab_pipeline._apply_input_order_execution_barrier(
        [uav, lah],
        input_plan=input_plan,
        target_input_id=12,
    )

    assert result == {"targetUnblocked": 1, "laterBlocked": 2}
    assert uav_rows[900]["executionBlockedUntilNextCollab"] is True
    assert "executionBlockedUntilNextCollab" not in uav_rows[12]
    assert uav_rows[700]["executionBlockedUntilNextCollab"] is True
    assert uav_rows[3]["executionBlockedUntilNextCollab"] is True
    assert "executionBlockedUntilNextCollab" not in uav_rows[999]
    assert lah == lah_before


def test_enhanced_0302_export_opens_type2_outbound_and_blocks_its_suffix(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    input_plan = _type2_input_plan()
    source_packages = [
        _package(4, [901, 17, 803, 4], first_mission_id=900_030_000),
        _package(5, [901, 17, 803, 4], first_mission_id=900_040_000),
        _package(6, [901, 17, 803, 4], first_mission_id=900_050_000),
    ]
    monkeypatch.setattr(
        export_0302,
        "build_0302_packages_from_split",
        lambda *_args, **_kwargs: deepcopy(source_packages),
    )
    monkeypatch.setattr(
        export_0302,
        "_build_lah_0302_packages_from_cmpk",
        lambda *_args, start_im_id, **_kwargs: ([], int(start_im_id)),
    )

    result = export_0302.build_0302_packages_from_split_with_lah(
        SimpleNamespace(planning_mode=None),
        cmpk=input_plan,
    )

    assert len(result) == 3
    for package in result:
        missions = _by_input(package)
        assert "executionBlockedUntilNextCollab" not in missions[901]
        assert missions[17]["executionBlockedUntilNextCollab"] is True
        assert missions[803]["executionBlockedUntilNextCollab"] is True
        assert missions[4]["executionBlockedUntilNextCollab"] is True


def test_initial_barrier_can_start_at_exact_type2_return_line() -> None:
    input_plan = _type2_input_plan()
    package = _package(4, [803, 4], first_mission_id=900_060_000)

    target_input_id = export_0302._apply_initial_type2_line_execution_barrier(
        [package],
        input_plan=input_plan,
    )

    missions = _by_input(package)
    assert target_input_id == 803
    assert "executionBlockedUntilNextCollab" not in missions[803]
    assert missions[4]["executionBlockedUntilNextCollab"] is True
