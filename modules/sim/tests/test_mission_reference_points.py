from __future__ import annotations

import json
from pathlib import Path

from modules.sim.mission.mission_loader import load_flight_paths
from modules.sim.mission.mission_plan_loader import build_mission_plan_payload


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")


def _reference_payload() -> dict:
    return {
        "timestamp": 200,
        "takeOverInfoList": [
            {
                "aircraftID": 4,
                "coordinate": {"latitude": 37.1, "longitude": 128.1, "altitude": 1000},
            }
        ],
        "handOverInfoList": [
            {
                "aircraftID": 4,
                "coordinate": {"latitude": 37.2, "longitude": 128.2, "altitude": 900},
            }
        ],
        "rtbCoordinateList": [
            {"latitude": 37.3, "longitude": 128.3, "altitude": 800}
        ],
    }


def test_mission_plan_payload_exposes_all_reference_point_lists(tmp_path: Path) -> None:
    _write_json(
        tmp_path / "MissionPlan" / "1.json",
        {
            "missionPlanID": 1,
            "missionReferencePackageID": 7,
            "aircraftList": [],
        },
    )
    _write_json(tmp_path / "MissionReferenceInfo" / "7.json", _reference_payload())

    result = build_mission_plan_payload(1, db_root=tmp_path)

    assert result["ok"] is True
    for key in ("takeOverInfoList", "handOverInfoList", "rtbCoordinateList"):
        assert len(result[key]) == 1
        assert result["payload"][key] == result[key]


def test_folder_loader_exposes_latest_reference_point_lists(tmp_path: Path) -> None:
    (tmp_path / "FlightPath").mkdir()
    (tmp_path / "IndividualMissionPlan").mkdir()
    older = _reference_payload()
    older["timestamp"] = 100
    older["takeOverInfoList"][0]["aircraftID"] = 5
    _write_json(tmp_path / "MissionReferenceInfo" / "1.json", older)
    _write_json(tmp_path / "MissionReferenceInfo" / "2.json", _reference_payload())

    result = load_flight_paths(str(tmp_path), project_root=tmp_path)

    assert result["ok"] is True
    assert result["takeOverInfoList"][0]["aircraftID"] == 4
    assert len(result["handOverInfoList"]) == 1
    assert len(result["rtbCoordinateList"]) == 1
