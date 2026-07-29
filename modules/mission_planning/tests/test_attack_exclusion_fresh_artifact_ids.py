"""Attack-exclusion options must publish a wholly fresh artifact graph."""

from __future__ import annotations

import json
from copy import deepcopy
from pathlib import Path
from typing import Any

from modules.mission_planning.replanning.triggers.attack import pipeline as attack
from modules.mission_planning.runtime.ids.replan_reservation import (
    ReplanIdReservation,
    ReservedIdBlock,
)


def _write(root: Path, kind: str, artifact_id: int, payload: dict[str, Any]) -> None:
    dest = root / kind / f"{artifact_id}.json"
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(payload), encoding="utf-8")


def _read(root: Path, kind: str, artifact_id: int) -> dict[str, Any]:
    return json.loads((root / kind / f"{artifact_id}.json").read_text(encoding="utf-8"))


def _reservation() -> ReplanIdReservation:
    return ReplanIdReservation(
        imp_ids=ReservedIdBlock("imp", [8_000_101, 8_000_102]),
        individual_ids=ReservedIdBlock("individual", [9_000_101, 9_000_102]),
        waypoint_ids=ReservedIdBlock("waypoint", [10_101, 10_102, 10_103, 10_104]),
        path_ids_by_aircraft={
            1: ReservedIdBlock("path[1]", [1_000_101]),
            4: ReservedIdBlock("path[4]", [4_000_101]),
        },
    )


def test_every_published_exclusion_artifact_gets_a_new_id(tmp_path, monkeypatch) -> None:
    monkeypatch.setattr(
        attack.db_paths,
        "get_db_subpath",
        lambda kind, name: tmp_path / kind / name,
    )
    monkeypatch.setattr(
        attack,
        "read_json_cached",
        lambda path, kind=None: json.loads(Path(path).read_text(encoding="utf-8")),
    )
    monkeypatch.setattr(
        attack,
        "_validate_generated_artifact_write_entries",
        lambda **_kwargs: {},
    )

    def _write_batch(entries):
        results = []
        for path, payload in entries:
            path = Path(path)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(payload), encoding="utf-8")
            results.append({"path": str(path), "written": True, "skipped": False})
        return results

    monkeypatch.setattr(attack, "_write_json_files_batch", _write_batch)

    lah_imp = {
        "individualMissionPackageID": 8_000_001,
        "aircraftID": 1,
        "individualMissionList": [
            {
                "individualMissionID": 9_000_001,
                "pathID": 1_000_001,
                "isDone": True,
                "individualMissionInfo": {"individualMissionType": 6},
            }
        ],
    }
    lah_path = {
        "pathID": 1_000_001,
        "aircraftID": 1,
        "individualMissionID": 9_000_001,
        "lahWaypointList": [
            {"waypointID": 101, "nextWaypointID": 102, "coordinate": {"altitude": 500}},
            {"waypointID": 102, "nextWaypointID": 0, "coordinate": {"altitude": 510}},
        ],
    }
    uav_imp = {
        "individualMissionPackageID": 8_000_004,
        "aircraftID": 4,
        "individualMissionList": [
            {
                "individualMissionID": 9_000_004,
                "pathID": 4_000_001,
                "isDone": False,
                "relatedMission": {"inputMissionID": 5},
            }
        ],
    }
    uav_path = {
        "pathID": 4_000_001,
        "aircraftID": 4,
        "individualMissionID": 9_000_004,
        "waypointList": [
            {"waypointID": 401, "nextWaypointID": 402, "filmingProperty": {"operationMode": 2}},
            {"waypointID": 402, "nextWaypointID": 0, "filmingProperty": {"operationMode": 2}},
        ],
    }
    for kind, artifact_id, payload in (
        ("IndividualMissionPlan", 8_000_001, lah_imp),
        ("FlightPath", 1_000_001, lah_path),
        ("IndividualMissionPlan", 8_000_004, uav_imp),
        ("FlightPath", 4_000_001, uav_path),
    ):
        _write(tmp_path, kind, artifact_id, payload)

    plan = {
        "missionPlanID": 7_000_006,
        "aircraftList": [
            {"aircraftID": 1, "individualMissionPackageID": 8_000_001},
            {"aircraftID": 4, "individualMissionPackageID": 8_000_004},
        ],
    }
    source_plan = deepcopy(plan)

    summary = attack._freshen_attack_exclusion_artifact_ids(
        plan,
        now_ms=123456,
        emit=lambda _message: None,
        id_reservation=_reservation(),
    )

    assert [row["individualMissionPackageID"] for row in plan["aircraftList"]] == [
        8_000_101,
        8_000_102,
    ]
    assert plan != source_plan
    assert summary["policy"] == "all_artifact_ids_fresh"
    assert summary["individualMissionCount"] == 2
    assert summary["pathCount"] == 2
    assert summary["waypointCount"] == 4

    new_lah_imp = _read(tmp_path, "IndividualMissionPlan", 8_000_101)
    new_uav_imp = _read(tmp_path, "IndividualMissionPlan", 8_000_102)
    new_lah_mission = new_lah_imp["individualMissionList"][0]
    new_uav_mission = new_uav_imp["individualMissionList"][0]
    assert new_lah_mission["individualMissionID"] == 9_000_101
    assert new_lah_mission["pathID"] == 1_000_101
    assert new_lah_mission["isDone"] is True
    assert new_uav_mission["individualMissionID"] == 9_000_102
    assert new_uav_mission["pathID"] == 4_000_101
    assert new_uav_mission["relatedMission"] == {"inputMissionID": 5}

    new_lah_path = _read(tmp_path, "FlightPath", 1_000_101)
    new_uav_path = _read(tmp_path, "FlightPath", 4_000_101)
    assert new_lah_path["individualMissionID"] == 9_000_101
    assert new_uav_path["individualMissionID"] == 9_000_102
    assert [row["waypointID"] for row in new_lah_path["lahWaypointList"]] == [10_101, 10_102]
    assert [row["nextWaypointID"] for row in new_lah_path["lahWaypointList"]] == [10_102, 0]
    assert [row["waypointID"] for row in new_uav_path["waypointList"]] == [10_103, 10_104]
    assert [row["nextWaypointID"] for row in new_uav_path["waypointList"]] == [10_104, 0]

    # Source packages and paths remain untouched; only the candidate references
    # the fresh graph.
    assert _read(tmp_path, "IndividualMissionPlan", 8_000_001) == lah_imp
    assert _read(tmp_path, "IndividualMissionPlan", 8_000_004) == uav_imp
    assert _read(tmp_path, "FlightPath", 1_000_001) == lah_path
    assert _read(tmp_path, "FlightPath", 4_000_001) == uav_path


def test_fresh_id_pass_runs_before_tracking_strip_and_validation() -> None:
    import inspect

    source = inspect.getsource(attack.run_attack_exclusion_pipeline)
    fresh_at = source.index("_freshen_attack_exclusion_artifact_ids(")
    strip_at = source.index("_strip_tracking_from_exclusion_plan(new_plan_data")
    validate_at = source.index('scope="attack_exclusion"')

    assert fresh_at < strip_at < validate_at


def test_fresh_id_pass_relinks_boundary_guard_children_after_waypoint_refresh(
    tmp_path,
    monkeypatch,
) -> None:
    monkeypatch.setattr(
        attack.db_paths,
        "get_db_subpath",
        lambda kind, name: tmp_path / kind / name,
    )
    monkeypatch.setattr(
        attack,
        "read_json_cached",
        lambda path, kind=None: json.loads(Path(path).read_text(encoding="utf-8")),
    )
    monkeypatch.setattr(
        attack,
        "_validate_generated_artifact_write_entries",
        lambda **_kwargs: {},
    )

    def _write_batch(entries):
        for path, payload in entries:
            path = Path(path)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(payload), encoding="utf-8")
        return []

    monkeypatch.setattr(attack, "_write_json_files_batch", _write_batch)

    contract = {
        "boundaryGuardLoop": True,
        "boundaryGuardLoopVersion": 1,
        "boundaryGuardSetID": "type2-boundary:3:5:aircraft-4",
        "boundaryGuardSequenceCount": 2,
        "boundaryGuardDurationS": 600.0,
    }
    missions = []
    paths = []
    for sequence, mission_id, path_id, waypoint_ids in (
        (1, 9_000_011, 4_000_011, (411, 412)),
        (2, 9_000_012, 4_000_012, (421, 422)),
    ):
        row_contract = {**contract, "boundaryGuardSequence": sequence}
        missions.append(
            {
                "individualMissionID": mission_id,
                "pathID": path_id,
                **row_contract,
                "individualMissionInfo": dict(row_contract),
            }
        )
        paths.append(
            {
                "pathID": path_id,
                "aircraftID": 4,
                "individualMissionID": mission_id,
                **row_contract,
                "waypointList": [
                    {
                        "waypointID": waypoint_ids[0],
                        "nextWaypointID": waypoint_ids[1],
                    },
                    {
                        "waypointID": waypoint_ids[1],
                        "nextWaypointID": 421 if sequence == 1 else 411,
                    },
                ],
            }
        )

    _write(
        tmp_path,
        "IndividualMissionPlan",
        8_000_014,
        {
            "individualMissionPackageID": 8_000_014,
            "aircraftID": 4,
            "individualMissionList": missions,
        },
    )
    for path in paths:
        _write(tmp_path, "FlightPath", path["pathID"], path)

    reservation = ReplanIdReservation(
        imp_ids=ReservedIdBlock("imp", [8_000_114]),
        individual_ids=ReservedIdBlock(
            "individual",
            [9_000_111, 9_000_112],
        ),
        waypoint_ids=ReservedIdBlock(
            "waypoint",
            [11_101, 11_102, 11_103, 11_104],
        ),
        path_ids_by_aircraft={
            4: ReservedIdBlock("path[4]", [4_000_111, 4_000_112]),
        },
    )
    plan = {
        "missionPlanID": 7_000_029,
        "aircraftList": [
            {"aircraftID": 4, "individualMissionPackageID": 8_000_014}
        ],
    }

    attack._freshen_attack_exclusion_artifact_ids(
        plan,
        now_ms=123456,
        emit=lambda _message: None,
        id_reservation=reservation,
    )

    first_path = _read(tmp_path, "FlightPath", 4_000_111)
    second_path = _read(tmp_path, "FlightPath", 4_000_112)
    assert first_path["boundaryGuardSequence"] == 1
    assert second_path["boundaryGuardSequence"] == 2
    assert first_path["waypointList"][-1]["nextWaypointID"] == 11_103
    assert second_path["waypointList"][-1]["nextWaypointID"] == 11_101

    refreshed_imp = _read(tmp_path, "IndividualMissionPlan", 8_000_114)
    refreshed_missions = refreshed_imp["individualMissionList"]
    assert [row["boundaryGuardSequence"] for row in refreshed_missions] == [1, 2]
    assert all(row["boundaryGuardSequenceCount"] == 2 for row in refreshed_missions)
